/*  Copyright (c) 2014  Consensic Sensortec

    This driver supports the cps121 digital barometric pressure
    and temperature sensors from Consensic Sensortec.

    A pressure measurement is issued by reading from register 0x30.
    The return value ranges from 30000 to 110000 pascal with a resulution
    of 1 pascal (0.01 millibar).

    The temperature values resolution is 0.1 celsius.
*/

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/device.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/input/cps121.h>

//#define CPS_DEBUG  //if need debug, please open CPS_DEBUG

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cps121_early_suspend(struct early_suspend *h);
static void cps121_late_resume(struct early_suspend *h);
#endif

static struct sensors_classdev sensors_pressure_cdev = {
	.name = "cps121-pressure",
	.vendor = "Consensic",
	.version = 1,
	.handle = SENSORS_PRESSURE_HANDLE,
	.type = SENSOR_TYPE_PRESSURE,
	.max_range = "1100.0",
	.resolution = "0.01",
	.sensor_power = "0.67",
	.min_delay = 3000,	/* microsecond */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,	/* millisecond */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static int cps121_i2c_read_block_data(struct cps121_data *data,
				      u8 addr, u8 length, void *values)
{
	struct i2c_client *client = data->client;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = values,
		},
	};
	int retval;

	retval = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (retval < 0)
		return retval;

	return retval != ARRAY_SIZE(msgs) ? -EIO : 0;
}
 
static inline unsigned long cps121_calibration_pressure_data(struct cps121_data *data, struct cps121_raw_data *raw_data)
{
	return (((raw_data->pressure[0] << 16) | (raw_data->pressure[1] << 8) | raw_data->pressure[2]) >> 6);
}
 
static inline int cps121_calibration_temperature_data(struct cps121_data *data, struct cps121_raw_data *raw_data)
{
	return (((raw_data->temperature[0] << 8) | raw_data->temperature[1]) >> 8);
}

static int cps121_measurement_request(struct cps121_data *data)
{
	i2c_smbus_write_byte_data(data->client, CPS121_REG_MEASUREREQUEST, CPS121_CMD_MEASUREREQUEST);
	msleep(data->platform_data->measure_delay);
	return 0;
}

static int cps121_get_temperature(struct cps121_data *data)
{
	int status;
	struct cps121_raw_data raw_data;

	status = cps121_i2c_read_block_data(data, CPS121_REG_MEASUREREQUEST, CPS121_GETDATA_T_LENGTH, raw_data.value);
	if (status != 0){
		pr_err("%s read block err\n",__func__);
		return 0;//err
	}

	#ifdef CPS_DEBUG
	pr_info("%s data=%x %x %x %x %x\n",__func__,raw_data.value[0],raw_data.value[1],raw_data.value[2],raw_data.value[3],raw_data.value[4]);
	#endif

	return cps121_calibration_temperature_data(data, &raw_data);
}

/*
 * This function starts the pressure measurement and returns the value
 * in millibar. Since the pressure depends on the ambient temperature,
 * a temperature measurement is executed according to the given temperature
 * measurememt period (default is 1 sec boundary). This period could vary
 * and needs to be adjusted accoring to the sensor environment, i.e. if big
 * temperature variations then the temperature needs to be read out often.
 */
static unsigned long cps121_get_pressure(struct cps121_data *data)
{
	int status;
	struct cps121_raw_data raw_data;

	status = cps121_i2c_read_block_data(data, CPS121_REG_GETDATA, CPS121_GETDATA_T_LENGTH, raw_data.value);
	if (status != 0){
		pr_err("%s read block err\n",__func__);
		return 0;//err
	}

	#ifdef CPS_DEBUG
	pr_info("%s data=%x %x %x %x %x\n",__func__,raw_data.value[0],raw_data.value[1],raw_data.value[2],raw_data.value[3],raw_data.value[4]);
	#endif

	return cps121_calibration_pressure_data(data, &raw_data);
}

static ssize_t cps121_set_poll_delay(struct sensors_classdev *sensors_cdev,
						unsigned int delay_msec)
{
	struct cps121_data *data = container_of(sensors_cdev,
					struct cps121_data, cdev);
	if(data==NULL || data->platform_data==NULL) return -1;

	mutex_lock(&data->lock);
	data->platform_data->polling_interval = delay_msec;
	mutex_unlock(&data->lock);

	return 0;
}

static ssize_t show_delay(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cps121_data *data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u\n", data->platform_data->polling_interval);
}

static ssize_t store_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cps121_data *data = dev_get_drvdata(dev);
	unsigned long delay;
	int err = kstrtoul(buf, 10, &delay);
	if (err < 0)
		return err;

	err = cps121_set_poll_delay(&data->cdev, delay);
	if (err < 0)
		return err;

	return count;
}

static DEVICE_ATTR(poll_delay, S_IWUSR | S_IRUGO,
				show_delay, store_delay);

static int cps121_enable(struct cps121_data *data, unsigned int enable)
{
	if (data->enable == enable) {
		pr_notice("%s already %s\n", __func__, enable ? "enabled" : "disabled");
		return -1;
	}

	cancel_delayed_work_sync(&data->work);
	if (enable) {
		schedule_delayed_work(&data->work,
					msecs_to_jiffies(data->platform_data->polling_interval));
	}

	data->enable = enable;
	return 0;
}

static int cps121_set_enable(struct sensors_classdev *sensors_cdev,
						unsigned int enable)
{
	struct cps121_data *data = container_of(sensors_cdev,
					struct cps121_data, cdev);

	enable = enable ? 1 : 0;

	mutex_lock(&data->lock);
	cps121_enable(data,enable);
	mutex_unlock(&data->lock);

	return 0;
}

static int show_enable(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cps121_data *data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u\n", data->enable);
}

static int store_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cps121_data *data = dev_get_drvdata(dev);
	unsigned long enable;
	int err = kstrtoul(buf, 10, &enable);
	if (err < 0)
		return err;

	err = cps121_enable(data, enable);
	if (err < 0)
		return err;

	return count;
}
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
				show_enable, store_enable);

static ssize_t show_temperature(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int temperature;
	struct cps121_data *data = dev_get_drvdata(dev);

	temperature = cps121_get_temperature(data);
	return snprintf(buf, PAGE_SIZE,"%d\n", temperature);
}
static DEVICE_ATTR(temperature, S_IRUGO, show_temperature, NULL);


static ssize_t show_pressure(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int pressure;
	struct cps121_data *data = dev_get_drvdata(dev);

	pressure = cps121_get_pressure(data);
	return snprintf(buf, PAGE_SIZE, "%d\n", pressure);
}
static DEVICE_ATTR(pressure, S_IRUGO, show_pressure, NULL);


static struct attribute *cps121_attributes[] = {
	&dev_attr_temperature.attr,
	&dev_attr_pressure.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_enable.attr,
	NULL
};

static const struct attribute_group cps121_attr_group = {
	.attrs = cps121_attributes,
};

static int cps121_check_pressure_data_validity(int pressure)
{
	if(pressure < ABS_MIN_PRESSURE || pressure > ABS_MAX_PRESSURE)
		return -1;
	return 0;
}

static void cps121_work_func(struct work_struct *work)
{
	struct cps121_data *data =
		container_of((struct delayed_work *)work, struct cps121_data, work);
	int delay = msecs_to_jiffies(data->platform_data->polling_interval);
	int pressure=0;
	int err = 0;

	mutex_lock(&data->lock);
	cps121_measurement_request(data);
	pressure = (int)cps121_get_pressure(data);
	mutex_unlock(&data->lock);

	err = cps121_check_pressure_data_validity(pressure);
	if(err < 0)
		pr_err("%s pressure data invalid!\n",__func__);

	input_report_abs(data->input, ABS_PRESSURE, pressure);
	input_sync(data->input);

	schedule_delayed_work(&data->work, delay);
}

static int cps121_input_init(struct cps121_data *data)
{
	int err=0;

	data->input = input_allocate_device();
	if (!data->input){
		pr_err("%s input dev is NULL\n",__func__);
		return -ENOMEM;
	}

	set_bit(EV_ABS, data->input->evbit);
	input_set_abs_params(data->input, ABS_PRESSURE, ABS_MIN_PRESSURE, ABS_MAX_PRESSURE, 0, 0);

	data->input->name = CPS121_NAME;
	data->input->id.bustype = BUS_I2C;
	data->input->dev.parent =&data->client->dev;
	input_set_drvdata(data->input, data);

	err = input_register_device(data->input);
	if (err < 0) {
		pr_err("%s err:%d\n",__func__,err);
		input_free_device(data->input);
		return err;
	}

	return 0;
}

static void cps121_input_delete(struct cps121_data *data)
{
	input_unregister_device(data->input);
	input_free_device(data->input);
}

static int cps121_sensor_regulator_configure(struct cps121_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, CPS121_VDD_MAX_UV);
		regulator_put(data->vdd);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd, CPS121_VDD_MIN_UV, CPS121_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,"Regulator set failed vdd rc=%d\n",rc);
				goto reg_vdd_put;
			}
		}
	}

	return 0;

reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int cps121_sensor_regulator_power_on(struct cps121_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
	}

	msleep(100);
	dev_dbg(&data->client->dev, "Sensor regulator power on =%d\n", on);
	return rc;
}

static int cps121_hw_power_on(struct cps121_data *data, bool on)
{
	int err = 0;

	if (data->power_state != on) {
		if(on){
			err = cps121_sensor_regulator_configure(data, true);
			if(err){
				dev_err(&data->client->dev, "unable to configure regulator on=%d\n", on);
				goto power_out;
			}

			err = cps121_sensor_regulator_power_on(data, true);
			if (err){
				dev_err(&data->client->dev, "Can't configure regulator on=%d\n", on);
				goto power_out;
			}

			data->power_state = true;
		}else{
			err = cps121_sensor_regulator_power_on(data, false);
			if (err){
				dev_err(&data->client->dev, "Can't configure regulator on=%d\n", on);
				goto power_out;
			}

			err = cps121_sensor_regulator_configure(data, false);
			if(err){
				dev_err(&data->client->dev, "unable to configure regulator on=%d\n", on);
				goto power_out;
			}

			data->power_state = false;
		}
	}
power_out:
	return err;
}
 
static int cps121_parse_dt(struct device *dev, struct cps121_data *data)
{
	struct cps121_platform_data *pdata = data->platform_data;
	struct device_node *np = dev->of_node;
	unsigned int tmp;
	int rc = 0;

	/* pressure tuning data*/
	rc = of_property_read_u32(np, "consensic,polling-time", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ps threshold\n");
		return rc;
	}
	pdata->polling_interval = tmp;

	rc = of_property_read_u32(np, "consensic,measure-time", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ps threshold\n");
		return rc;
	}
	pdata->measure_delay = tmp;

	return 0;
}

int cps121_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct cps121_data *data;
	struct cps121_platform_data *pdata;
	int err = 0;

	/* check i2c*/
	if (!i2c_check_functionality(adapter,I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
			dev_err(&client->dev, "i2c functionality check failed.\n");
			return -EIO;
	}

	/* platform data memory allocation*/
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct cps121_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "No platform data\n");
			return -ENODEV;
		}
	}

	/* data memory allocation */
	data = kzalloc(sizeof(struct cps121_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		dev_err(&client->dev, "allocate data NULL\n");
		goto exit_free_pdata;
	}
	data->client = client;
	data->platform_data = pdata;
	err = cps121_parse_dt(&client->dev, data);
	if (err) {
		dev_err(&client->dev, "can't parse platform data\n");
		goto exit_free_data;
	}

	/* set client data as cps121_data*/
	i2c_set_clientdata(client, data);

	/* power initialization */
	err = cps121_hw_power_on(data, true);
	if (err) {
		dev_err(&client->dev,"power on fail\n");
		goto exit_free_data;
	}

	/* init mutex */
	mutex_init(&data->lock);

	/* Register Input Device */
	err = cps121_input_init(data);
	if (err){
		dev_err(&client->dev,"cps121_input_init fail\n");
		goto error_power;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &cps121_attr_group);
	if (err){
		dev_err(&client->dev,"sysfs_create_group fail\n");
		goto error_sysfs;
	}

	/* Register sensors class */
	data->cdev = sensors_pressure_cdev;
	data->cdev.sensors_enable = cps121_set_enable;
	data->cdev.sensors_poll_delay = cps121_set_poll_delay;
	err = sensors_classdev_register(&client->dev, &data->cdev);
	if (err) {
		dev_err(&client->dev, "class device create failed: %d\n", err);
		goto error_class_sysfs;
	}

	/* workqueue init */
	INIT_DELAYED_WORK(&data->work, cps121_work_func);

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = cps121_early_suspend;
	data->early_suspend.resume = cps121_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	dev_info(&client->dev, "Succesfully initialized cps121\n");
	return 0;

error_class_sysfs:
	sysfs_remove_group(&client->dev.kobj, &cps121_attr_group);
error_sysfs:
	cps121_input_delete(data);
error_power:
	cps121_hw_power_on(data, false);
exit_free_data:
	kfree(data);
exit_free_pdata:
	if (pdata && (client->dev.of_node))
		devm_kfree(&client->dev, pdata);
	data->platform_data= NULL;

	return err;
}

static int cps121_remove(struct i2c_client *client)
{
	struct cps121_data *data = i2c_get_clientdata(client);
	struct cps121_platform_data *pdata = data->platform_data;

	if (data == NULL || pdata == NULL) 
		return 0;

	cps121_hw_power_on(data, false);
	sensors_classdev_unregister(&data->cdev);
	cps121_input_delete(data);
	sysfs_remove_group(&client->dev.kobj, &cps121_attr_group);
	cancel_delayed_work_sync(&data->work);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	if (pdata && (client->dev.of_node))
		devm_kfree(&client->dev, pdata);
	pdata = NULL;
	kfree(data);
	data = NULL;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cps121_early_suspend(struct early_suspend *h)
{
	struct cps121_data *data =
		container_of(h, struct cps121_data, early_suspend);
	if (data->enable) {
		cancel_delayed_work_sync(&data->work);
	}
}

static void cps121_late_resume(struct early_suspend *h)
{
	struct cps121_data *data =
		container_of(h, struct cps121_data, early_suspend);

	if (data->enable) {
		schedule_delayed_work(&data->work,
					msecs_to_jiffies(data->platform_data->polling_interval));
	}

}
#endif

#ifdef CONFIG_PM
static int cps121_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct cps121_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->lock);
	if (data->enable) {
		cancel_delayed_work_sync(&data->work);
	}
	mutex_unlock(&data->lock);
	return 0;
}

static int cps121_resume(struct i2c_client *client)
{
	struct cps121_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->lock);
	if (data->enable) {
		schedule_delayed_work(&data->work,
					msecs_to_jiffies(data->platform_data->polling_interval));
	}
	mutex_unlock(&data->lock);

	return 0;
}

#else

#define cps121_suspend      NULL
#define cps121_resume       NULL

#endif /* CONFIG_PM */


static struct i2c_device_id cps121_id[] = {
		{"cps121", 0},
		{}
};

static struct of_device_id cps121_match_table[] = {
		{ .compatible = "consensic,cps121",},
		{ },
};

MODULE_DEVICE_TABLE(i2c, cps121_id);

static struct i2c_driver cps121_driver = {
		.driver = {
				.name = CPS121_DRV_NAME,
				.owner = THIS_MODULE,
				.of_match_table = cps121_match_table,
		},
		.suspend = cps121_suspend,
		.resume = cps121_resume,
		.probe = cps121_probe,
		.remove = cps121_remove,
		.id_table = cps121_id,
};

static int cps121_driver_init(void)
{
	pr_info("Driver cps121 init.\n");
	return i2c_add_driver(&cps121_driver);
};

static void cps121_driver_exit(void)
{
	pr_info("Unload cps121 module...\n");
	i2c_del_driver(&cps121_driver);
}

module_init(cps121_driver_init);
module_exit(cps121_driver_exit);

MODULE_AUTHOR("Consensic Technology Corp.");
MODULE_DESCRIPTION("CPS121 driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
