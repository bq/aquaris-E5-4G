/*
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/of_gpio.h>
#include <mach/gpiomux.h>
#include <linux/sensors.h>

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/fs.h>
#include	<linux/miscdevice.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input/mxc6225.h>
#include <linux/input-polldev.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include        <linux/earlysuspend.h>
#endif

#define ACCEL_INPUT_DEV_NAME	"accelerometer"
#define DEVICE_NAME		"mxc6225"
#if 0
#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define DATA_CTRL		0x21
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE			(1 << 5)
/* DATA CONTROL REGISTER BITS */
#define ODR12_5F		0
#define ODR25F			1
#define ODR50F			2
#define ODR100F		3
#define ODR200F		4
#define ODR400F		5
#define ODR800F		6
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTJ9_IEL		(1 << 3)
#define KXTJ9_IEA		(1 << 4)
#define KXTJ9_IEN		(1 << 5)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3
/* POWER SUPPLY VOLTAGE RANGE */
#define KXTJ9_VDD_MIN_UV	2000000
#define KXTJ9_VDD_MAX_UV	3300000
#define KXTJ9_VIO_MIN_UV	1750000
#define KXTJ9_VIO_MAX_UV	1950000
#else
#define	G_MAX		16000	/** Maximum polled-device-reported g value */
#define WHOAMI_MXC622X_ACC	0x05	/*	Expctd content for WAI	*/  //MXC6255XU=0x25  MXC6255XC=0x05

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x08	/*	WhoAmI register		*/

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */

#define DEVICE_INFO         "Memsic, MXC622X"
#define DEVICE_INFO_LEN     32

/* end RESUME STATE INDICES */

#ifdef DEBUG
#undef DEBUG
#endif
#ifdef MXC622X_DEBUG
#undef MXC622X_DEBUG
#endif

//#define MXC622X_DEBUG

#define	MAX_INTERVAL	50

/* POWER SUPPLY VOLTAGE RANGE */
#define KXTJ9_VDD_MIN_UV	2000000
#define KXTJ9_VDD_MAX_UV	3300000
#define KXTJ9_VIO_MIN_UV	1750000
#define KXTJ9_VIO_MAX_UV	1950000
#endif

#if 0
/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} mxc6225_odr_table[] = {
	{ 3,	ODR800F },
	{ 5,	ODR400F },
	{ 10,	ODR200F },
	{ 20,	ODR100F },
	{ 40,	ODR50F  },
	{ 80,	ODR25F  },
	{ 0,	ODR12_5F},
};

struct mxc622x_acc_data {
	struct i2c_client *client;
	struct mxc622x_acc_platform_data pdata;
	struct input_dev *input_dev;
#ifdef CONFIG_SENSORS_MXC6225_POLLED_MODE
	struct input_polled_dev *poll_dev;
#endif
	unsigned int last_poll_interval;
	bool	enable;
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
	u8 int_ctrl;
	bool	power_enabled;
	struct regulator *vdd;
	struct regulator *vio;
};
#else

struct mxc622x_acc_data {
	struct i2c_client *client;
	struct mxc622x_acc_platform_data pdata;
	struct mutex op_lock;
	struct delayed_work input_work;

	struct device *dev;
	struct input_dev *accel_dev;
	struct sensors_classdev accel_cdev;

	int hw_initialized;
	bool	power_enabled;
	atomic_t enabled;

	struct regulator *vdd;
	struct regulator *vio;
#ifdef CONFIG_HAS_EARLYSUSPEND
        struct early_suspend early_suspend;
#endif
};

#endif

/* Accelerometer information read by HAL */
static struct sensors_classdev mxc622x_acc_cdev = {
	.name = "memsic-accel",
	.vendor = "memsic",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",	/* m/s^2 */
	.resolution = "0.000598144",	/* m/s^2 */
	.sensor_power = "0.5",	/* 0.5 mA */
	.min_delay = 1000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static int mxc622x_acc_i2c_read(struct mxc622x_acc_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf, },
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf, },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int mxc622x_acc_i2c_write(struct mxc622x_acc_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = { { .addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = len + 1, .buf = buf, }, };
	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int mxc622x_acc_get_acceleration_data(struct mxc622x_acc_data *acc,
		int *xyz)
{
	int err = -1;
	static int datum_x=0;
	static int datum_z=1020;
	static int datum_sign = 1;
	/* Data bytes from hardware x, y */
	#ifdef MXC622X_DEBUG
	u8 acc_data[3];
	#else
	u8 acc_data[2];
	#endif

	acc_data[0] = MXC622X_REG_DATA;
	err = mxc622x_acc_i2c_read(acc, acc_data, 2);

	if (err < 0)
        {
                #ifdef DEBUG
                printk(KERN_INFO "%s I2C read error %d\n", MXC622X_ACC_I2C_NAME, err);
                #endif
		return err;
        }

	#ifdef MXC622X_DEBUG
		printk(KERN_INFO "%s read acc_data[%d, %d, %d]\n",MXC622X_ACC_DEV_NAME, acc_data[0], acc_data[1], acc_data[2]);
	#endif

	xyz[0] = -(signed char)acc_data[0]*16; // 1024/64.0f=16
	xyz[1] = -(signed char)acc_data[1]*16;
	datum_z = datum_z==1020 ? 1030 : 1020;
	xyz[2] = datum_z; // 32*1024/64.0f=512
	if(datum_x>900 && datum_x>xyz[0]){
		datum_sign = (-1)*datum_sign;
		printk(KERN_INFO "%s datum_sign=%d\n",MXC622X_ACC_DEV_NAME, datum_sign);
	}
	xyz[2] = datum_sign*xyz[2];
	datum_x = xyz[0];

	#ifdef MXC622X_DEBUG
		printk(KERN_INFO "%s read x=%d, y=%d, z=%d\n",MXC622X_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
		//printk(KERN_INFO "%s poll interval %d\n", MXC622X_ACC_DEV_NAME, acc->pdata.poll_interval);
	#endif
	return err;
}

static void mxc622x_acc_report_values(struct mxc622x_acc_data *acc, int *xyz)
{
	input_report_abs(acc->accel_dev, ABS_X, xyz[0]);
	input_report_abs(acc->accel_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->accel_dev, ABS_Z, xyz[2]);
	input_sync(acc->accel_dev);
}

static int mxc622x_acc_hw_init(struct mxc622x_acc_data *acc)
{
	int err = -1;
	u8 buf[7];

	printk(KERN_INFO "%s: hw init start\n", MXC622X_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = mxc622x_acc_i2c_read(acc, buf, 1);
	if (err < 0)
		goto error_firstread;

	if ((buf[0] & 0x3F) != WHOAMI_MXC622X_ACC) {
		err = -1; /* choose the right coded error */
		goto error_unknown_device;
	}

	acc->hw_initialized = 1;
	printk(KERN_INFO "%s: hw init done\n", MXC622X_ACC_DEV_NAME);
	return 0;

error_firstread:
	dev_warn(&acc->client->dev, "Error reading WHO_AM_I: is device "
		"available/working?\n");
	goto error1;
error_unknown_device:
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x,"
		" Replies: 0x%x\n", WHOAMI_MXC622X_ACC, buf[0]);
error1:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static int mxc622x_acc_power_on(struct mxc622x_acc_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {//pwroff		
		/*
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			regulator_enable(data->vdd);
		}
		*/
		data->power_enabled = false;
	} else if (on && !data->power_enabled) {///pwron
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(data->vdd);
		}
		data->power_enabled = true;
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return rc;
}

static int mxc622x_acc_power_init(struct mxc622x_acc_data *acc, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(acc->vdd) > 0)
			regulator_set_voltage(acc->vdd, 0, acc->pdata.vdd_max_uv);

		regulator_put(acc->vdd);

		if (regulator_count_voltages(acc->vio) > 0)
			regulator_set_voltage(acc->vio, 0, acc->pdata.vio_max_uv);

		regulator_put(acc->vio);
	} else {
		acc->vdd = regulator_get(&acc->client->dev, "vdd");
		if (IS_ERR(acc->vdd)) {
			rc = PTR_ERR(acc->vdd);
			dev_err(&acc->client->dev,"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(acc->vdd) > 0) {
			rc = regulator_set_voltage(acc->vdd, acc->pdata.vdd_min_uv,acc->pdata.vdd_max_uv);
			if (rc) {
				dev_err(&acc->client->dev,"Regulator set failed vdd rc=%d\n",rc);
				goto reg_vdd_put;
			}
		}

		acc->vio = regulator_get(&acc->client->dev, "vio");
		if (IS_ERR(acc->vio)) {
			rc = PTR_ERR(acc->vio);
			dev_err(&acc->client->dev,"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(acc->vio) > 0) {
			rc = regulator_set_voltage(acc->vio, acc->pdata.vio_min_uv,acc->pdata.vio_max_uv);
			if (rc) {
				dev_err(&acc->client->dev,"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(acc->vio);
reg_vdd_set:
	if (regulator_count_voltages(acc->vdd) > 0)
		regulator_set_voltage(acc->vdd, 0, acc->pdata.vdd_max_uv);
reg_vdd_put:
	regulator_put(acc->vdd);
	return rc;
}

static void mxc622x_acc_device_power_off(struct mxc622x_acc_data *acc)
{
	int err;
	u8 buf_off[2] = { MXC622X_REG_CTRL, MXC622X_CTRL_PWRDN };

	err = mxc622x_acc_i2c_write(acc, buf_off, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);
	
	mxc622x_acc_power_on(acc, false);

	dev_dbg(&acc->client->dev, "soft power off complete.\n");
	return ;
}

static int mxc622x_acc_device_power_on(struct mxc622x_acc_data *acc)
{
	int err = 0;
	u8 buf_on[2] = { MXC622X_REG_CTRL, MXC622X_CTRL_PWRON };
	
	err = mxc622x_acc_power_on(acc, true);
	if (err) {
		dev_err(&acc->client->dev, "power on failed\n");
		goto err_exit;
	}
	msleep(20);
	
	err = mxc622x_acc_i2c_write(acc, buf_on, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power on failed: %d\n", err);
		
	if (!acc->hw_initialized) {
		err = mxc622x_acc_hw_init(acc);
		if (err < 0) {
			mxc622x_acc_device_power_off(acc);
			return err;
		}
	}

err_exit:
	dev_dbg(&acc->client->dev, "soft power on complete err=%d.\n", err);
	return err;
}

static int mxc622x_acc_enable(struct mxc622x_acc_data *acc)
{
	int err;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = mxc622x_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}		
		schedule_delayed_work(&acc->input_work, msecs_to_jiffies(acc->pdata.poll_interval));
	}

	return 0;
}

static int mxc622x_acc_disable(struct mxc622x_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		mxc622x_acc_device_power_off(acc);
	}

	return 0;
}

static int mxc622x_acc_set_enable(struct mxc622x_acc_data *acc, unsigned int enable)
{
	int err;

	mutex_lock(&acc->op_lock);
	if(enable)
		err = mxc622x_acc_enable(acc);
	else
		err = mxc622x_acc_disable(acc);
	mutex_unlock(&acc->op_lock);

	pr_info("%s, enable=%d,err=%d\n",__func__,enable,err);

	return err;
}

static int mxc622x_acc_set_poll_delay(struct mxc622x_acc_data *acc,unsigned long delay)
{
	mutex_lock(&acc->op_lock);
	if (delay < MXC622X_ACCEL_MIN_POLL_INTERVAL_MS)
		delay = MXC622X_ACCEL_MIN_POLL_INTERVAL_MS;
	if (delay > MXC622X_ACCEL_MAX_POLL_INTERVAL_MS)
		delay = MXC622X_ACCEL_MAX_POLL_INTERVAL_MS;

	if (acc->pdata.poll_interval!= delay) {
			acc->pdata.poll_interval = delay;
	}
	pr_info("%s poll_interval=%d",__func__,acc->pdata.poll_interval);
	cancel_delayed_work_sync(&acc->input_work);
	schedule_delayed_work(&acc->input_work,msecs_to_jiffies(acc->pdata.poll_interval));
	mutex_unlock(&acc->op_lock);
	return 0;
}

static void mxc622x_acc_input_work_func(struct work_struct *work)
{
	struct mxc622x_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			struct mxc622x_acc_data,	input_work);

	mutex_lock(&acc->op_lock);
	err = mxc622x_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		mxc622x_acc_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(acc->pdata.poll_interval));
	mutex_unlock(&acc->op_lock);
}

static int mxc622x_input_device_init(struct mxc622x_acc_data *acc)
{
	int err;

	acc->accel_dev = input_allocate_device();
	if (!acc->accel_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocate failed\n");
		goto err0;
	}

	set_bit(EV_ABS, acc->accel_dev->evbit);

	input_set_abs_params(acc->accel_dev, ABS_X, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->accel_dev, ABS_Y, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->accel_dev, ABS_Z, -G_MAX, G_MAX, 0, 0);

	acc->accel_dev->name = MXC622X_ACC_INPUT_NAME;
	acc->accel_dev->id.bustype = BUS_I2C;
	acc->accel_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->accel_dev, acc);

	err = input_register_device(acc->accel_dev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input polled device %s\n",
				acc->accel_dev->name);
		goto err1;
	}

	// Polling rx data when the interrupt is not used.
	INIT_DELAYED_WORK(&acc->input_work, mxc622x_acc_input_work_func);

	return 0;

err1:
	input_free_device(acc->accel_dev);
err0:
	return err;
}

static int mxc622x_accel_cdev_enable(struct sensors_classdev *sensors_cdev,
			unsigned int enable)
{
	struct mxc622x_acc_data *acc = container_of(sensors_cdev,
			struct mxc622x_acc_data, accel_cdev);
	return mxc622x_acc_set_enable(acc, enable);
}

static int mxc622x_accel_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
			unsigned int delay_ms)
{
	struct mxc622x_acc_data *acc = container_of(sensors_cdev,
			struct mxc622x_acc_data, accel_cdev);

	return mxc622x_acc_set_poll_delay(acc, delay_ms);
}

static int mxc622x_sensors_classdev_init(struct mxc622x_acc_data *acc)
{
	int ret;
	acc->accel_cdev = mxc622x_acc_cdev;
	acc->accel_cdev.delay_msec = acc->pdata.poll_interval;
	acc->accel_cdev.sensors_enable = mxc622x_accel_cdev_enable;
	acc->accel_cdev.sensors_poll_delay = mxc622x_accel_cdev_poll_delay;
	ret = sensors_classdev_register(&acc->client->dev, &acc->accel_cdev);
	if (ret) {
		dev_err(&acc->client->dev,"create accel class device file failed!\n");
		ret = -EINVAL;
		goto err0;
	}
	return 0;
err0:
	return ret;
}

static int mxc622x_acc_verify(struct mxc622x_acc_data *acc)
{
	int retval;

	retval = i2c_smbus_read_byte_data(acc->client, WHO_AM_I);
	if (retval < 0) {
		dev_err(&acc->client->dev, "read err int source\n");
		goto out;
	}

	if ((retval & 0x003F) == WHOAMI_MXC622X_ACC) {
		printk(KERN_INFO "%s I2C driver registered!\n",
							MXC622X_ACC_DEV_NAME);
	} else {
		acc->client = NULL;
		printk(KERN_INFO "I2C driver not registered!\n"
				" Device unknown 0x%x\n", retval);
		retval = -EIO;
	}

out:
	return retval;
}

static int mxc622x_acc_parse_dt(struct device *dev,
				struct mxc622x_acc_platform_data *acc_pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "memsic,min-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read min-interval\n");
		return rc;
	} else {
		acc_pdata->min_interval = temp_val;
	}

	rc = of_property_read_u32(np, "memsic,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else {
		acc_pdata->poll_interval = temp_val;
	}

	rc = of_property_read_u32(np, "memsic,max-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read max-interval\n");
		return rc;
	} else {
		acc_pdata->max_interval = temp_val;
	}

	rc = of_property_read_u32(np, "memsic,vdd-max-uv", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read vdd-max-uv\n");
		return rc;
	} else {
		acc_pdata->vdd_max_uv = temp_val;
	}

	rc = of_property_read_u32(np, "memsic,vdd-min-uv", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read vdd-min-uv\n");
		return rc;
	} else {
		acc_pdata->vdd_min_uv = temp_val;
	}

	rc = of_property_read_u32(np, "memsic,vio-max-uv", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read vio-max-uv\n");
		return rc;
	} else {
		acc_pdata->vio_max_uv = temp_val;
	}

	rc = of_property_read_u32(np, "memsic,vio-min-uv", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read vio-min-uv\n");
		return rc;
	} else {
		acc_pdata->vio_min_uv = temp_val;
	}

	return 0;
}

static int mxc622x_acc_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct mxc622x_acc_data *acc;
	int err;

	pr_info("%s start\n",__func__);
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	acc = kzalloc(sizeof(struct mxc622x_acc_data), GFP_KERNEL);
	if (!acc) {
		dev_err(&client->dev,"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		memset(&acc->pdata, 0 , sizeof(acc->pdata));
		err = mxc622x_acc_parse_dt(&client->dev, &acc->pdata);
		if (err) {
			dev_err(&client->dev,"Unable to parse platfrom data err=%d\n", err);
			err = -EINVAL;
			goto err_free_mem;
		}
	} else {
		if (client->dev.platform_data)
			memcpy(&acc->pdata, client->dev.platform_data, sizeof(acc->pdata));
		else {
			dev_err(&client->dev,"platform data is NULL; exiting\n");
			err = -EINVAL;
			goto err_free_mem;
		}
	}

	mutex_init(&acc->op_lock);
	acc->client = client;
	acc->dev = &client->dev;
	acc->power_enabled = false;

	err = mxc622x_acc_power_init(acc, true);
	if (err < 0) {
		dev_err(&acc->client->dev, "power init failed! err=%d\n", err);
		goto err_free_mem;
	}

	err = mxc622x_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed! err=%d\n", err);
		goto err_power_deinit;
	}

	err = mxc622x_acc_verify(acc);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_power_off;
	}

	i2c_set_clientdata(client, acc);
	
	err = mxc622x_input_device_init(acc);
	if (err)
		goto err_destroy_input;
	acc->accel_dev->dev.parent = &client->dev;

	err = mxc622x_sensors_classdev_init(acc);
	if (err)
		goto err_destroy_input;

	dev_info(&client->dev, "%s: mxc622x_acc_probe OK.\n", __func__);
	mxc622x_acc_device_power_off(acc);
	return 0;

err_destroy_input:
	sensors_classdev_unregister(&acc->accel_cdev);
	input_unregister_device(acc->accel_dev);
err_power_off:
	mxc622x_acc_device_power_off(acc);
err_power_deinit:
	mxc622x_acc_power_init(acc, false);
err_free_mem:
	kfree(acc);

	dev_err(&client->dev, "%s: mxc622x_acc_probe err=%d\n", __func__, err);
	return err;
}

static int mxc622x_acc_remove(struct i2c_client *client)
{
	struct mxc622x_acc_data *acc = i2c_get_clientdata(client);

	sensors_classdev_unregister(&acc->accel_cdev);
	input_unregister_device(acc->accel_dev);
	mxc622x_acc_device_power_off(acc);
	mxc622x_acc_power_init(acc, false);

	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mxc622x_acc_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxc622x_acc_data *acc = i2c_get_clientdata(client);
	struct input_dev *input_dev = acc->accel_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxc622x_acc_disable(acc);

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int mxc622x_acc_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxc622x_acc_data *acc = i2c_get_clientdata(client);
	struct input_dev *input_dev = acc->accel_dev;
	int retval = 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxc622x_acc_enable(acc);

	mutex_unlock(&input_dev->mutex);
	return retval;
}
#endif

static SIMPLE_DEV_PM_OPS(mxc622x_acc_pm_ops, mxc622x_acc_suspend, mxc622x_acc_resume);

static const struct i2c_device_id mxc622x_acc_id[] = {
	{ DEVICE_NAME, 0 },
	{ },
};

static struct of_device_id mxc622x_acc_match_table[] = {
	{ .compatible = "memsic,mxc6225", },
	{ },
};


MODULE_DEVICE_TABLE(i2c, mxc622x_acc_id);

static struct i2c_driver mxc622x_acc_driver = {
	.driver = {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = mxc622x_acc_match_table,
		.pm	= &mxc622x_acc_pm_ops,
	},
	.probe		= mxc622x_acc_probe,
	.remove		= mxc622x_acc_remove,
	.id_table	= mxc622x_acc_id,
};

module_i2c_driver(mxc622x_acc_driver);

MODULE_DESCRIPTION("mxc622x_acc accelerometer misc driver");
MODULE_AUTHOR("Memsic");
MODULE_LICENSE("GPL");
