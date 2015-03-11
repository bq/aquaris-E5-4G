/*
 * Japan Display Inc. BU21150 touch screen driver.
 *
 * Copyright (C) 2013-2015 Japan Display Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 *
 */
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/input/bu21150.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <asm/byteorder.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>

/* define */
#define DEVICE_NAME   "jdi-bu21150"
#define REG_SENS_START (0x0086)
#define REG_INT_RUN_ENB (0x00CE)
#define REG_READ_DATA (0x0400)
#define MAX_FRAME_SIZE (8*1024+16)  /* byte */
#define SPI_HEADER_SIZE (3)
#define SPI_BITS_PER_WORD_READ (8)
#define SPI_BITS_PER_WORD_WRITE (8)
#define FRAME_HEADER_SIZE (16)  /* byte */
#define GPIO_LOW  (0)
#define GPIO_HIGH (1)
#define DUR_RESET_HIGH	(1000)
#define DUR_RESET_LOW	(10)
#define WAITQ_WAIT   (0)
#define WAITQ_WAKEUP (1)
/* #define CHECK_SAME_FRAME */
#define APQ8074_DRAGONBOARD (0x01)
#define MSM8974_FLUID       (0x02)
#define TIMEOUT_SCALE       (20)
#define BU21150_VTG_MIN_UV (2700000)
#define BU21150_VTG_MAX_UV (3300000)
#define BU21150_VTG_DIG_MIN_UV (1800000)
#define BU21150_VTG_DIG_MAX_UV (1800000)

/* struct */
struct bu21150_data {
	/* system */
	struct spi_device *client;
	struct workqueue_struct *workq;
	struct work_struct work;
	u16 scan_mode;
	/* frame */
	struct bu21150_ioctl_get_frame_data req_get;
	u8 frame[MAX_FRAME_SIZE];
	struct bu21150_ioctl_get_frame_data frame_get;
	struct timeval tv;
	struct mutex mutex_frame;
	/* frame work */
	u8 frame_work[MAX_FRAME_SIZE];
	struct bu21150_ioctl_get_frame_data frame_work_get;
	/* waitq */
	u8 frame_waitq_flag;
	wait_queue_head_t frame_waitq;
	/* reset */
	u8 reset_flag;
	/* timeout */
	u8 timeout_enb;
	u8 set_timer_flag;
	u8 timeout_flag;
	u32 timeout;
	/* spi */
	u8 spi_buf[MAX_FRAME_SIZE];
    /* power */
	struct regulator *vcc_ana;
	struct regulator *vcc_dig;
	/* dtsi */
	int irq_gpio;
	int rst_gpio;
	int power_supply;
	/* suspend */
	struct notifier_block fb_notif;
	u8 suspended_flag;
	u8 unblock_flag;
	u8 force_unblock_flag;
};

struct ser_req {
	struct spi_message    msg;
	struct spi_transfer    xfer[2];
	u16 sample ____cacheline_aligned;
};

/* static function declaration */
static int bu21150_probe(struct spi_device *client);
static int bu21150_remove(struct spi_device *client);
static int bu21150_open(struct inode *inode, struct file *filp);
static int bu21150_release(struct inode *inode, struct file *filp);
static int bu21150_suspend(struct device *dev);
static int bu21150_resume(struct device *dev);
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
static long bu21150_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg);
static long bu21150_ioctl_get_frame(unsigned long arg);
static long bu21150_ioctl_reset(unsigned long arg);
static long bu21150_ioctl_spi_read(unsigned long arg);
static long bu21150_ioctl_spi_write(unsigned long arg);
static long bu21150_ioctl_suspend(void);
static long bu21150_ioctl_resume(void);
static long bu21150_ioctl_unblock(void);
static long bu21150_ioctl_unblock_release(void);
static long bu21150_ioctl_set_timeout(unsigned long arg);
static long bu21150_ioctl_set_scan_mode(unsigned long arg);
static irqreturn_t bu21150_irq_handler(int irq, void *dev_id);
static void bu21150_irq_work_func(struct work_struct *work);
static void swap_2byte(unsigned char *buf, unsigned int size);
static int bu21150_read_register(u32 addr, u16 size, u8 *data);
static int bu21150_write_register(u32 addr, u16 size, u8 *data);
static void wake_up_frame_waitq(struct bu21150_data *ts);
static long wait_frame_waitq(struct bu21150_data *ts, u8 flag);
static int is_same_bu21150_ioctl_get_frame_data(
	struct bu21150_ioctl_get_frame_data *data1,
	struct bu21150_ioctl_get_frame_data *data2);
static void copy_frame(struct bu21150_data *ts);
#ifdef CHECK_SAME_FRAME
static void check_same_frame(struct bu21150_data *ts);
#endif
static bool parse_dtsi(struct device *dev, struct bu21150_data *ts);
static void get_frame_timer_init(void);
static void get_frame_timer_handler(unsigned long data);
static void get_frame_timer_delete(void);

/* static variables */
static struct spi_device *g_client_bu21150;
static int g_io_opened;
static struct timer_list get_frame_timer;

static const struct of_device_id g_bu21150_psoc_match_table[] = {
	{	.compatible = "jdi,bu21150", },
};

static const struct file_operations g_bu21150_fops = {
	.owner = THIS_MODULE,
	.open = bu21150_open,
	.release = bu21150_release,
	.unlocked_ioctl = bu21150_ioctl,
};

static struct miscdevice g_bu21150_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &g_bu21150_fops,
};

static const struct spi_device_id g_bu21150_device_id[] = {
	{ DEVICE_NAME, 0 },
};
MODULE_DEVICE_TABLE(spi, g_bu21150_device_id);

static struct spi_driver g_bu21150_spi_driver = {
	.probe = bu21150_probe,
	.remove = __devexit_p(bu21150_remove),
	.id_table = g_bu21150_device_id,
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.bus = &spi_bus_type,
		.of_match_table = g_bu21150_psoc_match_table,
	},
};

module_spi_driver(g_bu21150_spi_driver);
MODULE_AUTHOR("Japan Display Inc");
MODULE_DESCRIPTION("JDI BU21150 Device Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:bu21150");

/* static functions */
static int reg_set_optimum_mode_check(struct regulator *reg, int load_ua)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_ua) : 0;
}

static int bu21150_probe(struct spi_device *client)
{
	struct bu21150_data *ts;
	int error;
	int rc;

	ts = kzalloc(sizeof(struct bu21150_data), GFP_KERNEL);
	if (!ts) {
		dev_err(&client->dev, "Out of memory\n");
		return -ENOMEM;
	}

	/* parse dtsi */
	if (!parse_dtsi(&client->dev, ts)) {
		dev_err(&client->dev, "Invalid dtsi\n");
		error = -EINVAL;
		goto err1;
	}

	/* Panel and AFE Power on sequence */
	if (ts->power_supply == APQ8074_DRAGONBOARD) {
		rc = gpio_request(1, "GPIO1");
		if (rc)
			pr_err("%s: gpio_request(%d) failed\n",
				__func__, 1);
		gpio_direction_output(1, 1);
		gpio_set_value(1, 1);
		usleep(1000);
		rc = gpio_request(92, "GPIO2");
		if (rc)
			pr_err("%s: gpio_request(%d) failed\n",
				__func__, 92);
		gpio_direction_output(92, 1);
		gpio_set_value(92, 1);
		usleep(1000);
		rc = gpio_request(0, "GPIO3");
		if (rc)
			pr_err("%s: gpio_request(%d) failed\n",
				__func__, 0);
		gpio_direction_output(0, 1);
		gpio_set_value(0, 1);
		usleep(1000);

		ts->vcc_ana = regulator_get(&client->dev, "vdd_ana");
		if (IS_ERR(ts->vcc_ana)) {
			rc = PTR_ERR(ts->vcc_ana);
			dev_err(&client->dev,
				"Regulator get failed vcc_ana rc=%d\n", rc);
			error = -EINVAL;
			goto error_get_vcc_ana;
		}
		if (regulator_count_voltages(ts->vcc_ana) > 0) {
			rc = regulator_set_voltage(ts->vcc_ana,
				BU21150_VTG_MIN_UV, BU21150_VTG_MAX_UV);
			if (rc) {
				dev_err(&client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
				error = -EINVAL;
				goto error_set_vtg_vcc_ana;
			}
		}
		rc = reg_set_optimum_mode_check(ts->vcc_ana, 150000);
		if (rc < 0) {
			dev_err(&client->dev,
				"Regulator vcc_ana set_opt failed rc=%d\n", rc);
			error = -EINVAL;
			goto error_set_vtg_vcc_ana;
		}
		rc = regulator_enable(ts->vcc_ana);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_ana enable failed rc=%d\n", rc);
			error = -EINVAL;
			goto error_reg_en_vcc_ana;
		}

		ts->vcc_dig = regulator_get(&client->dev, "vdd_dig");
		if (IS_ERR(ts->vcc_dig)) {
			rc = PTR_ERR(ts->vcc_dig);
			dev_err(&client->dev,
				"Regulator get failed vcc_dig rc=%d\n", rc);
			error = -EINVAL;
			goto error_get_vcc_dig;
		}
		if (regulator_count_voltages(ts->vcc_dig) > 0) {
			rc = regulator_set_voltage(ts->vcc_dig,
				BU21150_VTG_DIG_MIN_UV, BU21150_VTG_DIG_MAX_UV);
			if (rc) {
				dev_err(&client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
				error = -EINVAL;
				goto error_set_vtg_vcc_dig;
			}
		}
		rc = reg_set_optimum_mode_check(ts->vcc_dig, 100000);
		if (rc < 0) {
			dev_err(&client->dev,
				"Regulator vcc_dig set_opt failed rc=%d\n", rc);
			error = -EINVAL;
			goto error_set_vtg_vcc_dig;
		}
		rc = regulator_enable(ts->vcc_dig);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_dig enable failed rc=%d\n", rc);
			error = -EINVAL;
			goto error_reg_en_vcc_dig;
		}
	} else if (ts->power_supply == MSM8974_FLUID) {
		ts->vcc_ana = regulator_get(&client->dev, "vdd_ana");
		if (IS_ERR(ts->vcc_ana)) {
			rc = PTR_ERR(ts->vcc_ana);
			dev_err(&client->dev,
				"Regulator get failed vcc_ana rc=%d\n", rc);
			error = -EINVAL;
			goto err1;
		}

		if (regulator_count_voltages(ts->vcc_ana) > 0) {
			rc = regulator_set_voltage(ts->vcc_ana, 2700000,
								3300000);
			if (rc) {
				dev_err(&client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
				error = -EINVAL;
				goto error_set_vtg_vcc_ana;
			}
		}
		rc = reg_set_optimum_mode_check(ts->vcc_ana, 150000);
		if (rc < 0) {
			dev_err(&client->dev,
				"Regulator vcc_ana set_opt failed rc=%d\n", rc);
			error = -EINVAL;
			goto error_set_vtg_vcc_ana;
		}

		rc = regulator_enable(ts->vcc_ana);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_ana enable failed rc=%d\n", rc);
			error = -EINVAL;
			goto error_reg_en_vcc_ana;
		}
	}

	rc = gpio_request(ts->irq_gpio, "bu21150_ts_int");
	if (rc)
		pr_err("%s: gpio_request(%d) failed\n", __func__, ts->irq_gpio);
	gpio_direction_input(ts->irq_gpio);

	/* set reset */
	rc = gpio_request(ts->rst_gpio, "bu21150_ts_reset");
	if (rc)
		pr_err("%s: gpio_request(%d) failed\n", __func__, ts->rst_gpio);
	gpio_direction_output(ts->rst_gpio, GPIO_LOW);
	gpio_set_value(ts->rst_gpio, BU21150_RESET_LOW);
	usleep(DUR_RESET_LOW);
	gpio_set_value(ts->rst_gpio, BU21150_RESET_HIGH);
	usleep(DUR_RESET_HIGH);

	mutex_init(&ts->mutex_frame);
	init_waitqueue_head(&(ts->frame_waitq));

	g_client_bu21150 = client;
	ts->client = client;

	ts->workq = create_singlethread_workqueue("bu21150_workq");
	if (!ts->workq) {
		dev_err(&client->dev, "Unable to create workq\n");
		error =  -ENOMEM;
		goto err2;
	}
	INIT_WORK(&ts->work, bu21150_irq_work_func);

	if (!client->irq) {
		dev_err(&client->dev, "Bad irq\n");
		error = -EINVAL;
		goto err3;
	}

	error = request_irq(client->irq, bu21150_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				client->dev.driver->name, ts);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err3;
	}
	disable_irq(client->irq);

	ts->fb_notif.notifier_call = fb_notifier_callback;
	error = fb_register_client(&ts->fb_notif);
	if (error) {
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			error);
		goto err4;
	}

	error = misc_register(&g_bu21150_misc_device);
	if (error) {
		dev_err(&client->dev, "Failed to register misc device\n");
		goto err4;
	}
	dev_set_drvdata(&client->dev, ts);

	return 0;

err4:
	free_irq(client->irq, ts);
err3:
	destroy_workqueue(ts->workq);
err2:
	if (ts->power_supply == APQ8074_DRAGONBOARD || MSM8974_FLUID) {
		regulator_disable(ts->vcc_ana);
		regulator_disable(ts->vcc_dig);
	}
error_get_vcc_ana:
	if (ts->power_supply == APQ8074_DRAGONBOARD || MSM8974_FLUID)
		kfree(ts);
error_reg_en_vcc_ana:
	if (ts->power_supply == APQ8074_DRAGONBOARD || MSM8974_FLUID)
		reg_set_optimum_mode_check(ts->vcc_ana, 0);
error_set_vtg_vcc_ana:
	if (ts->power_supply == APQ8074_DRAGONBOARD || MSM8974_FLUID)
		regulator_put(ts->vcc_ana);
error_get_vcc_dig:
	if (ts->power_supply == APQ8074_DRAGONBOARD) {
		regulator_disable(ts->vcc_ana);
		kfree(ts);
	}
error_reg_en_vcc_dig:
	if (ts->power_supply == APQ8074_DRAGONBOARD) {
		regulator_disable(ts->vcc_ana);
		reg_set_optimum_mode_check(ts->vcc_dig, 0);
	}
error_set_vtg_vcc_dig:
	if (ts->power_supply == APQ8074_DRAGONBOARD) {
		regulator_disable(ts->vcc_ana);
		regulator_put(ts->vcc_dig);
	}
err1:
	kfree(ts);
	return error;
}

static int bu21150_suspend(struct device *dev)
{
	struct spi_device *client = to_spi_device(dev);
	struct bu21150_data *ts = spi_get_drvdata(client);
	u8 buf1[2] = {0x00, 0x00};
	u8 buf2[2] = {0x04, 0x00};

	bu21150_write_register(REG_SENS_START, (u16)sizeof(buf1), buf1);

	ts->suspended_flag = 1;
	ts->unblock_flag = 1;
	wake_up_frame_waitq(ts);

	bu21150_write_register(REG_INT_RUN_ENB, (u16)sizeof(buf2), buf2);

	return 0;
}

static int bu21150_resume(struct device *dev)
{
	struct spi_device *client = to_spi_device(dev);
	struct bu21150_data *ts = spi_get_drvdata(client);
	u8 buf[2] = {0x01, 0x00};

	bu21150_write_register(REG_INT_RUN_ENB, (u16)sizeof(buf), buf);

	ts->suspended_flag = 0;

	return 0;
}

static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct bu21150_data *bu21150_dev_data =
		container_of(self, struct bu21150_data, fb_notif);

	if (evdata && evdata->data &&
		bu21150_dev_data && bu21150_dev_data->client) {
		blank = evdata->data;
		if (event == FB_EARLY_EVENT_BLANK && *blank == FB_BLANK_UNBLANK)
			bu21150_resume(&bu21150_dev_data->client->dev);
		else if (event == FB_EVENT_BLANK &&
			*blank == FB_BLANK_POWERDOWN)
			bu21150_suspend(&bu21150_dev_data->client->dev);
	}
	return 0;
}


static void get_frame_timer_init(void)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);

	if (ts->set_timer_flag == 1) {
		del_timer_sync(&get_frame_timer);
		ts->set_timer_flag = 0;
	}

	if (ts->timeout > 0) {
		ts->set_timer_flag = 1;
		ts->timeout_flag = 0;

		init_timer(&get_frame_timer);

		get_frame_timer.expires  = jiffies + ts->timeout;
		get_frame_timer.data     = (unsigned long)jiffies;
		get_frame_timer.function = get_frame_timer_handler;

		add_timer(&get_frame_timer);
	}
}

static void get_frame_timer_handler(unsigned long data)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);

	ts->timeout_flag = 1;
	wake_up_frame_waitq(ts);
}

static void get_frame_timer_delete(void)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);

	if (ts->set_timer_flag == 1) {
		ts->set_timer_flag = 0;
		del_timer_sync(&get_frame_timer);
	}
}

static int bu21150_remove(struct spi_device *client)
{
	struct bu21150_data *ts = spi_get_drvdata(client);

	misc_deregister(&g_bu21150_misc_device);
	destroy_workqueue(ts->workq);
	free_irq(client->irq, ts);
	kfree(ts);

	return 0;
}

static int bu21150_open(struct inode *inode, struct file *filp)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);
	struct spi_device *client = ts->client;

	if (g_io_opened) {
		pr_err("%s: g_io_opened not zero.\n", __func__);
		return -EBUSY;
	}
	++g_io_opened;

	get_frame_timer_delete();
	ts->reset_flag = 0;
	ts->set_timer_flag = 0;
	ts->timeout_flag = 0;
	ts->timeout_enb = 0;
	ts->suspended_flag = 0;
	ts->unblock_flag = 0;
	ts->force_unblock_flag = 0;
	ts->scan_mode = BU21150_MUTUAL;
	memset(&(ts->req_get), 0, sizeof(struct bu21150_ioctl_get_frame_data));
	/* set default value. */
	ts->req_get.size = FRAME_HEADER_SIZE;
	memset(&(ts->frame_get), 0,
		sizeof(struct bu21150_ioctl_get_frame_data));
	memset(&(ts->frame_work_get), 0,
		sizeof(struct bu21150_ioctl_get_frame_data));

	enable_irq(client->irq);
	return 0;
}

static int bu21150_release(struct inode *inode, struct file *filp)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);
	struct spi_device *client = ts->client;

	if (!g_io_opened) {
		pr_err("%s: !g_io_opened\n", __func__);
		return -ENOTTY;
	}
	--g_io_opened;

	if (g_io_opened < 0)
		g_io_opened = 0;

	disable_irq(client->irq);

	return 0;
}

static long bu21150_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	long ret;

	switch (cmd) {
	case BU21150_IOCTL_CMD_GET_FRAME:
		ret = bu21150_ioctl_get_frame(arg);
		return ret;
	case BU21150_IOCTL_CMD_RESET:
		ret = bu21150_ioctl_reset(arg);
		return ret;
	case BU21150_IOCTL_CMD_SPI_READ:
		ret = bu21150_ioctl_spi_read(arg);
		return ret;
	case BU21150_IOCTL_CMD_SPI_WRITE:
		ret = bu21150_ioctl_spi_write(arg);
		return ret;
	case BU21150_IOCTL_CMD_UNBLOCK:
		ret = bu21150_ioctl_unblock();
		return ret;
	case BU21150_IOCTL_CMD_UNBLOCK_RELEASE:
		ret = bu21150_ioctl_unblock_release();
		return ret;
	case BU21150_IOCTL_CMD_SUSPEND:
		ret = bu21150_ioctl_suspend();
		return ret;
	case BU21150_IOCTL_CMD_RESUME:
		ret = bu21150_ioctl_resume();
		return ret;
	case BU21150_IOCTL_CMD_SET_TIMEOUT:
		ret = bu21150_ioctl_set_timeout(arg);
		return ret;
	case BU21150_IOCTL_CMD_SET_SCAN_MODE:
		ret = bu21150_ioctl_set_scan_mode(arg);
		return ret;
	default:
		pr_err("%s: cmd unkown.\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static long bu21150_ioctl_get_frame(unsigned long arg)
{
	long ret;
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);
	void __user *argp = (void __user *)arg;
	struct bu21150_ioctl_get_frame_data data;
	u32 frame_size;

	if (arg == 0) {
		pr_err("%s: arg == 0.\n", __func__);
		return -EINVAL;
	}
	if (copy_from_user(&data, argp,
		sizeof(struct bu21150_ioctl_get_frame_data))) {
		pr_err("%s: Failed to copy_from_user().\n", __func__);
		return -EFAULT;
	}
	if (data.buf == 0 || data.size == 0 ||
		MAX_FRAME_SIZE < data.size || data.tv == 0) {
		pr_err("%s: data.buf == 0 ...\n", __func__);
		return -EINVAL;
	}

	if (ts->timeout_enb == 1)
		get_frame_timer_init();

	do {
		ts->req_get = data;
		ret = wait_frame_waitq(ts, data.keep_block_flag);
		ts->unblock_flag = 0;
		if (ret != 0)
			return ret;
	} while (!is_same_bu21150_ioctl_get_frame_data(&data,
				&(ts->frame_get)));

	if (ts->timeout_enb == 1)
		get_frame_timer_delete();

	/* copy frame */
	mutex_lock(&ts->mutex_frame);
	frame_size = ts->frame_get.size;
	if (copy_to_user(data.buf, ts->frame, frame_size)) {
		mutex_unlock(&ts->mutex_frame);
		pr_err("%s: Failed to copy_to_user().\n", __func__);
		return -EFAULT;
	}
	if (copy_to_user(data.tv, &(ts->tv), sizeof(struct timeval))) {
		mutex_unlock(&ts->mutex_frame);
		pr_err("%s: Failed to copy_to_user().\n", __func__);
		return -EFAULT;
	}
	mutex_unlock(&ts->mutex_frame);

	return 0;
}

static long bu21150_ioctl_reset(unsigned long reset)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);

	if (!(reset == BU21150_RESET_LOW || reset == BU21150_RESET_HIGH)) {
		pr_err("%s: arg unknown.\n", __func__);
		return -EINVAL;
	}

	gpio_set_value(ts->rst_gpio, reset);

	ts->frame_waitq_flag = WAITQ_WAIT;
	if (reset == BU21150_RESET_LOW)
		ts->reset_flag = 1;

	return 0;
}

static long bu21150_ioctl_spi_read(unsigned long arg)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);
	void __user *argp = (void __user *)arg;
	struct bu21150_ioctl_spi_data data;

	if (arg == 0) {
		pr_err("%s: arg == 0.\n", __func__);
		return -EINVAL;
	}
	if (copy_from_user(&data, argp,
		sizeof(struct bu21150_ioctl_spi_data))) {
		pr_err("%s: Failed to copy_from_user().\n", __func__);
		return -EFAULT;
	}
	if (data.buf == 0 || data.count == 0 ||
		MAX_FRAME_SIZE < data.count) {
		pr_err("%s: data.buf == 0 ...\n", __func__);
		return -EINVAL;
	}

	bu21150_read_register(data.addr, data.count, ts->spi_buf);

	if (copy_to_user(data.buf, ts->spi_buf, data.count)) {
		pr_err("%s: Failed to copy_to_user().\n", __func__);
		return -EFAULT;
	}

	return 0;
}

static long bu21150_ioctl_spi_write(unsigned long arg)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);
	void __user *argp = (void __user *)arg;
	struct bu21150_ioctl_spi_data data;

	if (arg == 0) {
		pr_err("%s: arg == 0.\n", __func__);
		return -EINVAL;
	}
	if (copy_from_user(&data, argp,
		sizeof(struct bu21150_ioctl_spi_data))) {
		pr_err("%s: Failed to copy_from_user().\n", __func__);
		return -EFAULT;
	}
	if (data.buf == 0 || data.count == 0 ||
		MAX_FRAME_SIZE < data.count) {
		pr_err("%s: data.buf == 0 ...\n", __func__);
		return -EINVAL;
	}
	if ((ts->suspended_flag == 0 &&
		(data.next_mode == BU21150_GESTURE_SELF ||
		data.next_mode == BU21150_GESTURE_MUTUAL)) ||
		(ts->suspended_flag == 1 &&
		(data.next_mode == BU21150_SELF ||
		data.next_mode == BU21150_MUTUAL))) {
		pr_err("%s: invalid ts->suspended_flag = %d, mode = %d\n",
				__func__, ts->suspended_flag, data.next_mode);
		return -EINVAL;
	}
	if (copy_from_user(ts->spi_buf, data.buf, data.count)) {
		pr_err("%s: Failed to copy_from_user()..\n", __func__);
		return -EFAULT;
	}

	bu21150_write_register(data.addr, data.count, ts->spi_buf);

	return 0;
}

static long bu21150_ioctl_unblock(void)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);

	ts->force_unblock_flag = 1;
	ts->frame_waitq_flag = WAITQ_WAIT;
	wake_up_frame_waitq(ts);

	return 0;
}

static long bu21150_ioctl_unblock_release(void)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);

	ts->force_unblock_flag = 0;
	ts->frame_waitq_flag = WAITQ_WAIT;

	return 0;
}

static long bu21150_ioctl_suspend(void)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);
	struct spi_device *client = ts->client;

	bu21150_ioctl_unblock();
	disable_irq(client->irq);

	return 0;
}

static long bu21150_ioctl_resume(void)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);
	struct spi_device *client = ts->client;

	bu21150_ioctl_unblock_release();
	enable_irq(client->irq);

	return 0;
}

static long bu21150_ioctl_set_timeout(unsigned long arg)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);
	void __user *argp = (void __user *)arg;
	struct bu21150_ioctl_timeout_data data;

	if (copy_from_user(&data, argp,
		sizeof(struct bu21150_ioctl_timeout_data))) {
		pr_err("%s: Failed to copy_from_user().\n", __func__);
		return -EFAULT;
	}

	ts->timeout_enb = data.timeout_enb;
	if (data.timeout_enb == 1) {
		ts->timeout = (unsigned int)(data.report_interval_us
			* TIMEOUT_SCALE * HZ / 1000000);
	} else {
		get_frame_timer_delete();
	}

	return 0;
}

static long bu21150_ioctl_set_scan_mode(unsigned long arg)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);
	void __user *argp = (void __user *)arg;

	if (copy_from_user(&ts->scan_mode, argp,
		sizeof(u16))) {
		pr_err("%s: Failed to copy_from_user().\n", __func__);
		return -EFAULT;
	}

	return 0;
}

static irqreturn_t bu21150_irq_handler(int irq, void *dev_id)
{
	struct bu21150_data *ts = dev_id;

	disable_irq_nosync(irq);

	/* add work to queue */
	queue_work(ts->workq, &ts->work);

	return IRQ_HANDLED;
}

static void bu21150_irq_work_func(struct work_struct *work)
{
	struct bu21150_data *ts = container_of(work, struct bu21150_data, work);
	u8 *psbuf = (u8 *)ts->frame_work;
	struct spi_device *client = ts->client;

	/* get frame */
	ts->frame_work_get = ts->req_get;
	bu21150_read_register(REG_READ_DATA, ts->frame_work_get.size, psbuf);

	if (ts->reset_flag == 0) {
#ifdef CHECK_SAME_FRAME
		check_same_frame(ts);
#endif
		copy_frame(ts);
		wake_up_frame_waitq(ts);
	} else {
		ts->reset_flag = 0;
	}

	enable_irq(client->irq);
}

static int bu21150_read_register(u32 addr, u16 size, u8 *data)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);
	struct spi_device *client = ts->client;
	struct ser_req *req;
	int ret;
	u8 *input;
	u8 *output;

	input = kzalloc(sizeof(u8)*(size)+SPI_HEADER_SIZE, GFP_KERNEL);
	output = kzalloc(sizeof(u8)*(size)+SPI_HEADER_SIZE, GFP_KERNEL);
	req = kzalloc(sizeof(*req), GFP_KERNEL);

	/* set header */
	input[0] = 0x03;                 /* read command */
	input[1] = (addr & 0xFF00) >> 8; /* address hi */
	input[2] = (addr & 0x00FF) >> 0; /* address lo */

	/* read data */
	spi_message_init(&req->msg);
	req->xfer[0].tx_buf = input;
	req->xfer[0].rx_buf = output;
	req->xfer[0].len = size+SPI_HEADER_SIZE;
	req->xfer[0].cs_change = 0;
	req->xfer[0].bits_per_word = SPI_BITS_PER_WORD_READ;
	spi_message_add_tail(&req->xfer[0], &req->msg);
	ret = spi_sync(client, &req->msg);
	if (ret)
		pr_err("%s : spi_sync read data error:ret=[%d]", __func__, ret);

	memcpy(data, output+SPI_HEADER_SIZE, size);
	swap_2byte(data, size);

	kfree(req);
	kfree(input);
	kfree(output);

	return ret;
}

static int bu21150_write_register(u32 addr, u16 size, u8 *data)
{
	struct bu21150_data *ts = spi_get_drvdata(g_client_bu21150);
	struct spi_device *client = ts->client;
	struct ser_req *req;
	int ret;
	u8 *input;

	input = kzalloc(sizeof(u8)*(size)+SPI_HEADER_SIZE, GFP_KERNEL);
	req = kzalloc(sizeof(*req), GFP_KERNEL);

	/* set header */
	input[0] = 0x02;                 /* write command */
	input[1] = (addr & 0xFF00) >> 8; /* address hi */
	input[2] = (addr & 0x00FF) >> 0; /* address lo */

	/* set data */
	memcpy(input+SPI_HEADER_SIZE, data, size);
	swap_2byte(input+SPI_HEADER_SIZE, size);

	/* write data */
	spi_message_init(&req->msg);
	req->xfer[0].tx_buf = input;
	req->xfer[0].rx_buf = NULL;
	req->xfer[0].len = size+SPI_HEADER_SIZE;
	req->xfer[0].cs_change = 0;
	req->xfer[0].bits_per_word = SPI_BITS_PER_WORD_WRITE;
	spi_message_add_tail(&req->xfer[0], &req->msg);
	ret = spi_sync(client, &req->msg);
	if (ret)
		pr_err("%s : spi_sync read data error:ret=[%d]", __func__, ret);

	kfree(req);
	kfree(input);

	return ret;
}

static void wake_up_frame_waitq(struct bu21150_data *ts)
{
	ts->frame_waitq_flag = WAITQ_WAKEUP;
	wake_up_interruptible(&(ts->frame_waitq));
}

static long wait_frame_waitq(struct bu21150_data *ts, u8 flag)
{
	if (ts->force_unblock_flag == 1)
		return BU21150_UNBLOCK;

	if (ts->unblock_flag == 1 && flag == 0)
		return BU21150_UNBLOCK;

	/* wait event */
	if (wait_event_interruptible(ts->frame_waitq,
			ts->frame_waitq_flag == WAITQ_WAKEUP)) {
		pr_err("%s: -ERESTARTSYS\n", __func__);
		return -ERESTARTSYS;
	}
	ts->frame_waitq_flag = WAITQ_WAIT;

	if (ts->timeout_enb == 1) {
		if (ts->timeout_flag == 1) {
			ts->set_timer_flag = 0;
			ts->timeout_flag = 0;
			return BU21150_TIMEOUT;
		}
	}

	if (ts->force_unblock_flag == 1)
		return BU21150_UNBLOCK;

	if (ts->unblock_flag == 1 && flag == 0)
		return BU21150_UNBLOCK;

	return 0;
}

static int is_same_bu21150_ioctl_get_frame_data(
	struct bu21150_ioctl_get_frame_data *data1,
	struct bu21150_ioctl_get_frame_data *data2)
{
	int i;
	u8 *p1 = (u8 *)data1;
	u8 *p2 = (u8 *)data2;

	for (i = 0; i < sizeof(struct bu21150_ioctl_get_frame_data); i++) {
		if (p1[i] != p2[i])
			return 0;
	}

	return 1;
}

static void copy_frame(struct bu21150_data *ts)
{
	mutex_lock(&(ts->mutex_frame));
	ts->frame_get = ts->frame_work_get;
	memcpy(ts->frame, ts->frame_work, MAX_FRAME_SIZE);
	do_gettimeofday(&(ts->tv));
	mutex_unlock(&(ts->mutex_frame));
}

static void swap_2byte(unsigned char *buf, unsigned int size)
{
	int i;
	u16 *psbuf = (u16 *)buf;

	if (size%2 == 1) {
		pr_err("%s: error size is odd. size=[%u]\n", __func__, size);
		return;
	}

	for (i = 0; i < size/2; i++)
		be16_to_cpus(psbuf+i);
}

#ifdef CHECK_SAME_FRAME
static void check_same_frame(struct bu21150_data *ts)
{
	static int frame_no = -1;
	u16 *ps = (u16 *)ts->frame;
	if (ps[2] == frame_no)
		pr_err("%s:same_frame_no=[%d]\n", __func__, frame_no);
	frame_no = ps[2];
}
#endif

static bool parse_dtsi(struct device *dev, struct bu21150_data *ts)
{
	int rc;
	enum of_gpio_flags dummy;
	const char *str;
	struct device_node *np = dev->of_node;

	ts->irq_gpio = of_get_named_gpio_flags(np,
		"irq-gpio", 0, &dummy);
	ts->rst_gpio = of_get_named_gpio_flags(np,
		"rst-gpio", 0, &dummy);
	rc = of_property_read_string(np,
		"power-supply", &str);
	if (rc && (rc != -EINVAL))
		dev_err(dev, "Unable to read power-supply\n");
	if (!strcmp(str, "apq8074-dragonboard"))
		ts->power_supply = APQ8074_DRAGONBOARD;
	else if (!strcmp(str, "msm8974-fluid"))
		ts->power_supply = MSM8974_FLUID;
	else
		return false;

	return true;
}

