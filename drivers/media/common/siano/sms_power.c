#include <linux/bug.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h>

#include <linux/platform_device.h>


#define PINCTRL_STATE_ACTIVE	"pmx_sms_active"
#define PINCTRL_STATE_SUSPEND	"pmx_sms_suspend"

#define SMS4470_VDD_IO_MIN_UV	1800000
#define SMS4470_VDD_IO_MAX_UV	2950000
#define SMS4470_VDD_EMI_MIN_UV	1800000
#define SMS4470_VDD_EMI_MAX_UV	1800000

#define HOST_SMS_POWER_IO_PIN

struct sms_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *sms_pins_active;
	struct pinctrl_state *sms_pins_suspend;
};

struct sms_pinctrl_info pinctrl_info;

struct sms_power_ctrl_info {
	int sms_host_reset_pin;
	int sms_host_reset1_pin;
	int sms_host_pwrcore_pin;
	int sms_host_pwrio_pin;
	struct regulator *vdd_emi;
	struct regulator *vdd_io;
	bool power_enabled;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
	struct pinctrl_state *sms_pins_active;
	struct pinctrl_state *sms_pins_suspend;
};

struct sms_power_ctrl_info *sms_pwr_ctrl = NULL;

int sms_ldo_enable;

#if 1
typedef void (*pm_callback_t)(pm_message_t state, void *data);
static void *sms_sdio_pm_data = NULL;
static pm_callback_t sms_sdio_pm_cb = NULL;

void sms_sdio_register_pm(pm_callback_t pm_cb, void *data){
	/* register pm change callback */
	sms_sdio_pm_cb = pm_cb;
	sms_sdio_pm_data = data;
}
EXPORT_SYMBOL(sms_sdio_register_pm);

static void sms_sdio_on (void){
	pm_message_t state = { .event = PM_EVENT_USER_RESUME };
	
	/* 1. call sd callback */    
	if (sms_sdio_pm_cb) {
		printk(KERN_INFO "%s\n", __func__);
		sms_sdio_pm_cb(state, sms_sdio_pm_data);
	} else {
		printk(KERN_WARNING "%s no sd callback!!\n", __func__);
	}
}

static void sms_sdio_off (void){
	pm_message_t state = { .event = PM_EVENT_USER_SUSPEND };

	/* 1. call sd callback */
	if (sms_sdio_pm_cb) {
		printk(KERN_INFO "%s\n", __func__);
		sms_sdio_pm_cb(state, sms_sdio_pm_data);
	} else {
		printk(KERN_WARNING "%s no sd callback!!\n", __func__); 
	}
}

static int sms_sdio_status = 0;
int sms_sdio_ctrl (unsigned int sdio_port_num, unsigned int on){
	if (!sms_sdio_status && on) {
		/* off -> on */
		sms_sdio_on();
		sms_sdio_status = 1;
	} else if (sms_sdio_status && !on) {
		/* on -> off */
		sms_sdio_off();
		sms_sdio_status = 0;
	} else {
		pr_err("Ignore sms sdio status change from %d to %d\n",
				on, sms_sdio_status);    
	}
	return 0;
}

EXPORT_SYMBOL(sms_sdio_ctrl);
#endif

static int sms_power_ctrl_of_get_pinctrl(struct device *dev)
{
	pr_info("sms_power_ctrl_of_get_pinctrl\n");
	pinctrl_info.pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl_info.pinctrl)) {
		pr_err("%s:failed to get pinctrl\n", __func__);
		return PTR_ERR(pinctrl_info.pinctrl);
	}
#if  0
	pinctrl_info.sms_pins_active = pinctrl_lookup_state(pinctrl_info.pinctrl,
					PINCTRL_STATE_ACTIVE);
	if (IS_ERR(pinctrl_info.sms_pins_active)) {
		pr_err("%s Can not get sms power control gpio default pinstate\n", __func__);
		return PTR_ERR(pinctrl_info.sms_pins_active);
	}

	pinctrl_info.sms_pins_suspend = pinctrl_lookup_state(pinctrl_info.pinctrl,
					PINCTRL_STATE_SUSPEND);
	if (IS_ERR(pinctrl_info.sms_pins_suspend)) {
		pr_err("%s Can not get sms power control gpio suspend pinstate\n", __func__);
		return PTR_ERR(pinctrl_info.sms_pins_suspend);
	}
#endif
	return 0;
}

int sms_chip_poweron(void)
{
	int ret=0;

	if(!sms_pwr_ctrl->power_enabled) {
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_reset_pin)) {
		ret = gpio_request(sms_pwr_ctrl->sms_host_reset_pin, "SMSXXXX_RST");
		if (ret) {
			pr_err("%s: Failed to request gpio %d,rc = %d\n",
				__func__, sms_pwr_ctrl->sms_host_reset_pin, ret);
			goto free_reset_gpio;
		}
	}
	
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_reset1_pin)) {
		ret = gpio_request(sms_pwr_ctrl->sms_host_reset1_pin, "SMSXXXX_RST");
		if (ret) {
			pr_err("%s: Failed to request gpio %d,rc = %d\n",
				__func__, sms_pwr_ctrl->sms_host_reset1_pin, ret);
			goto free_reset1_gpio;
		}
	}

#ifdef HOST_SMS_POWER_IO_PIN
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_pwrio_pin)) {
		ret = gpio_request(sms_pwr_ctrl->sms_host_pwrio_pin, "SMSXXXX_PWR_IO");
		if (ret) {
			pr_err("%s: Failed to request gpio %d,rc = %d\n",
				__func__, sms_pwr_ctrl->sms_host_pwrio_pin, ret);
			goto free_pwrio_gpio;
		}
	}
#endif

	if (gpio_is_valid(sms_pwr_ctrl->sms_host_pwrcore_pin)) {
		ret = gpio_request(sms_pwr_ctrl->sms_host_pwrcore_pin, "SMSXXXX_PWR_CORE");
		if (ret) {
			pr_err("%s: Failed to request gpio %d,rc = %d\n",
				__func__, sms_pwr_ctrl->sms_host_pwrcore_pin, ret);
			goto free_pwrcore_gpio;
		}
	}

	ret = gpio_direction_output(sms_pwr_ctrl->sms_host_reset_pin, 0);
	if (ret) {
		pr_err( "set_direction for sms_host_reset_pin gpio failed\n");
		goto free_pwrcore_gpio;		
	}
	msleep(50);

	ret = gpio_direction_output(sms_pwr_ctrl->sms_host_reset1_pin, 0);
	if (ret) {
		pr_err( "set_direction for sms_host_reset1_pin gpio failed\n");
		goto free_pwrcore_gpio;		
	}
	msleep(50);

	
#ifdef HOST_SMS_POWER_IO_PIN
	ret = gpio_direction_output(sms_pwr_ctrl->sms_host_pwrio_pin, 0);
	if (ret) {
		pr_err( "set_direction for sms_host_pwrio_pin gpio failed\n");
		goto free_pwrcore_gpio;		
	}
	msleep(10);
#endif
	
	ret = gpio_direction_output(sms_pwr_ctrl->sms_host_pwrcore_pin, 0);
	if (ret) {
		pr_err( "set_direction for sms_host_pwrcore_pin gpio failed\n");
		goto free_pwrcore_gpio;		
	}
	msleep(20);

	ret = regulator_enable(sms_pwr_ctrl->vdd_emi);
	if (ret) {
		pr_err("Regulator vdd_emi enable failed rc=%d\n", ret);
		gpio_set_value(sms_pwr_ctrl->sms_host_pwrcore_pin, 0);
		goto free_pwrcore_gpio;
	}
	msleep(10);

	gpio_set_value(sms_pwr_ctrl->sms_host_pwrcore_pin, 1);
	usleep(10000);

#if 0
	ret = regulator_set_voltage(sms_pwr_ctrl->vdd_io, SMS4470_VDD_IO_MIN_UV, SMS4470_VDD_IO_MIN_UV);
	ret = regulator_enable(sms_pwr_ctrl->vdd_io);
	if (ret) {
		pr_err("Regulator vdd_io enable failed rc=%d\n", ret);
		regulator_disable(sms_pwr_ctrl->vdd_emi);
		gpio_set_value(sms_pwr_ctrl->sms_host_pwrcore_pin, 0);
		goto free_pwrcore_gpio;
	}
	msleep(50);
#endif

#ifdef HOST_SMS_POWER_IO_PIN
	gpio_set_value(sms_pwr_ctrl->sms_host_pwrio_pin, 1);
	msleep(10);
#endif

	gpio_set_value(sms_pwr_ctrl->sms_host_reset_pin, 1);
	msleep(200);

	ret = gpio_direction_output(sms_pwr_ctrl->sms_host_reset_pin, 0);
	if (ret) {
		pr_err( "set_direction for sms_host_reset_pin gpio failed\n");
		goto free_pwrcore_gpio;		
	}
	msleep(10);

	gpio_set_value(sms_pwr_ctrl->sms_host_reset_pin, 1);
	msleep(200);

	gpio_set_value(sms_pwr_ctrl->sms_host_reset1_pin, 1);
	msleep(200);

	ret = gpio_direction_output(sms_pwr_ctrl->sms_host_reset1_pin, 0);
	if (ret) {
		pr_err( "set_direction for sms_host_reset1_pin gpio failed\n");
		goto free_pwrcore_gpio;		
	}
	msleep(10);

	gpio_set_value(sms_pwr_ctrl->sms_host_reset1_pin, 1);
	msleep(200);
	
	//return 0;
free_pwrcore_gpio:
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_pwrcore_pin))
		gpio_free(sms_pwr_ctrl->sms_host_pwrcore_pin);
	
#ifdef HOST_SMS_POWER_IO_PIN
free_pwrio_gpio:
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_pwrio_pin))
		gpio_free(sms_pwr_ctrl->sms_host_pwrio_pin);
#endif

free_reset1_gpio:
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_reset1_pin))
		gpio_free(sms_pwr_ctrl->sms_host_reset1_pin);
		
free_reset_gpio:
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_reset_pin))
		gpio_free(sms_pwr_ctrl->sms_host_reset_pin);

	sms_pwr_ctrl->power_enabled = true;
	pr_info("sms_power_on\n");
	}else {
		pr_info("sms chip has already power on\n");
	}

	//ret = pinctrl_select_state(pinctrl_info.pinctrl, pinctrl_info.sms_pins_active);
	if (ret) {
		pr_err("sms_power_on failed: %d\n", ret);
	}
	
	return ret;
}
EXPORT_SYMBOL(sms_chip_poweron);

int sms_chip_poweroff(void)
{
	int ret;
	pr_info("sms_power_off\n");
	sms_sdio_off();
	sms_sdio_status = 0;
	//return 0;

	if (gpio_is_valid(sms_pwr_ctrl->sms_host_reset_pin)) {
		ret = gpio_request(sms_pwr_ctrl->sms_host_reset_pin, "SMSXXXX_RST");
		if (ret) {
			pr_err("%s: Failed to request gpio %d,rc = %d\n",
				__func__, sms_pwr_ctrl->sms_host_reset_pin, ret);
			goto free_reset_gpio;
		}
	}

	if (gpio_is_valid(sms_pwr_ctrl->sms_host_reset1_pin)) {
		ret = gpio_request(sms_pwr_ctrl->sms_host_reset1_pin, "SMSXXXX_RST");
		if (ret) {
			pr_err("%s: Failed to request gpio %d,rc = %d\n",
				__func__, sms_pwr_ctrl->sms_host_reset1_pin, ret);
			goto free_reset1_gpio;
		}
	}

#ifdef HOST_SMS_POWER_IO_PIN
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_pwrio_pin)) {
		ret = gpio_request(sms_pwr_ctrl->sms_host_pwrio_pin, "SMSXXXX_PWR_IO");
		if (ret) {
			pr_err("%s: Failed to request gpio %d,rc = %d\n",
				__func__, sms_pwr_ctrl->sms_host_pwrio_pin, ret);
			goto free_pwrio_gpio;
		}
	}
#endif

	if (gpio_is_valid(sms_pwr_ctrl->sms_host_pwrcore_pin)) {
		ret = gpio_request(sms_pwr_ctrl->sms_host_pwrcore_pin, "SMSXXXX_PWR_CORE");
		if (ret) {
			pr_err("%s: Failed to request gpio %d,rc = %d\n",
				__func__, sms_pwr_ctrl->sms_host_pwrcore_pin, ret);
			goto free_pwrcore_gpio;
		}
	}

	ret = gpio_direction_output(sms_pwr_ctrl->sms_host_reset_pin, 0);
	if (ret) {
		pr_err( "set_direction for sms_host_reset_pin gpio failed\n");
		goto free_pwrcore_gpio;		
	}
	msleep(50);


	ret = gpio_direction_output(sms_pwr_ctrl->sms_host_reset1_pin, 0);
	if (ret) {
		pr_err( "set_direction for sms_host_reset1_pin gpio failed\n");
		goto free_pwrcore_gpio;		
	}
	msleep(50);
	//gpio_set_value(sms_pwr_ctrl->sms_host_reset_pin, 0);
	//msleep(200);
	
#if 0
	ret = regulator_disable(sms_pwr_ctrl->vdd_io);
	if (ret) {
		pr_err("Regulator vdd_io disable failed rc=%d\n", ret);
		goto free_pwrcore_gpio;
	}
	msleep(10);
#endif

#ifdef HOST_SMS_POWER_IO_PIN
	ret = gpio_direction_output(sms_pwr_ctrl->sms_host_pwrio_pin, 0);
	if (ret) {
		pr_err( "set_direction for sms_host_pwrio_pin gpio failed\n");
		goto free_pwrcore_gpio;		
	}
	msleep(10);
#endif
	
	//gpio_set_value(sms_pwr_ctrl->sms_host_pwrio_pin, 0);
	//msleep(10);

	ret = gpio_direction_output(sms_pwr_ctrl->sms_host_pwrcore_pin, 0);
	if (ret) {
		pr_err( "set_direction for sms_host_pwrcore_pin gpio failed\n");
		goto free_pwrcore_gpio;		
	}
	usleep(5000);

	ret = regulator_disable(sms_pwr_ctrl->vdd_emi);
	if (ret) {
		pr_err("Regulator vdd_emi enable failed rc=%d\n", ret);
		//ret = regulator_enable(sms_pwr_ctrl->vdd_io);
		gpio_set_value(sms_pwr_ctrl->sms_host_pwrio_pin, 1);
		msleep(10);
		gpio_set_value(sms_pwr_ctrl->sms_host_pwrcore_pin, 1);
		msleep(200);
		goto free_pwrcore_gpio;
	}
	msleep(50);

	//gpio_set_value(sms_pwr_ctrl->sms_host_pwrcore_pin, 0);
	//msleep(200);
	
free_pwrcore_gpio:
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_pwrcore_pin))
		gpio_free(sms_pwr_ctrl->sms_host_pwrcore_pin);

#ifdef HOST_SMS_POWER_IO_PIN
free_pwrio_gpio:
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_pwrio_pin))
		gpio_free(sms_pwr_ctrl->sms_host_pwrio_pin);
#endif

free_reset1_gpio:
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_reset1_pin))
		gpio_free(sms_pwr_ctrl->sms_host_reset1_pin);

free_reset_gpio:
	if (gpio_is_valid(sms_pwr_ctrl->sms_host_reset_pin))
		gpio_free(sms_pwr_ctrl->sms_host_reset_pin);

	sms_pwr_ctrl->power_enabled = false;

	//ret = pinctrl_select_state(pinctrl_info.pinctrl, pinctrl_info.sms_pins_suspend);
	if (ret) {
		pr_err("sms_power_off failed: %d\n", ret);
	}

	return ret;
}
EXPORT_SYMBOL(sms_chip_poweroff);

#if 0
static int mpu6050_power_init(struct platform_device *pdev)
{
	int ret = 0;

	sms_pwr_ctrl->vdd_io = regulator_get(&pdev->dev, "vdd-io");
	if (IS_ERR(sms_pwr_ctrl->vdd_io)) {
		ret = PTR_ERR(sms_pwr_ctrl->vdd_io);
		dev_err(&pdev->dev,
			"Regulator get failed vdd io ret=%d\n", ret);
		goto error;
	}

	if (regulator_count_voltages(sms_pwr_ctrl->vdd_io) > 0) {
		ret = regulator_set_voltage(sms_pwr_ctrl->vdd_io,
				SMS4470_VDD_IO_MIN_UV,
				SMS4470_VDD_IO_MAX_UV);
		if (ret) {
			dev_err(&pdev->dev,
			"Regulator set_vtg failed vdd_io ret=%d\n", ret);
			goto reg_vddio_put;
		}
	}

	sms_pwr_ctrl->vdd_emi= regulator_get(&&pdev->dev, "vdd-emi");
	if (IS_ERR(sms_pwr_ctrl->vdd_emi)) {
		ret = PTR_ERR(sms_pwr_ctrl->vdd_emi);
		dev_err(&pdev->dev,
			"Regulator get failed vdd io ret=%d\n", ret);
		goto reg_vddio_set_vtg;
	}

	if (regulator_count_voltages(sms_pwr_ctrl->vdd_emi) > 0) {
		ret = regulator_set_voltage(sms_pwr_ctrl->vdd_emi,
				SMS4470_VDD_IO_MIN_UV,
				SMS4470_VDD_IO_MAX_UV);
		if (ret) {
			dev_err(&pdev->dev,
			"Regulator set_vtg failed vdd_emi ret=%d\n", ret);
			goto reg_vddemi_put;
		}
	}

	return 0;

reg_vddemi_put:
	regulator_put(sms_power_ctrl->vdd_emi);
reg_vddio_set_vtg:
	if (regulator_count_voltages(sms_power_ctrl->vdd_io) > 0)
		regulator_set_voltage(sms_power_ctrl->vdd_io, 0, SMS4470_VDD_IO_MAX_UV);
reg_vddio_put:
	regulator_put(sms_power_ctrl->vdd_io);	
	return ret;
}
#endif

static int sms_power_ctrl_of_probe(struct platform_device *pdev)
{
	int ret =0;
	struct sms_power_ctrl_info *sms_power_ctrl =NULL;
	struct device_node *node = pdev->dev.of_node;

	pr_info("sms_power_ctrl_of_probe\n");

	sms_power_ctrl = devm_kzalloc(&pdev->dev, sizeof(struct sms_power_ctrl_info),
				 GFP_KERNEL);
	if (sms_power_ctrl == NULL) {
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	sms_pwr_ctrl = sms_power_ctrl;

	sms_power_ctrl->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(sms_power_ctrl->pinctrl)) {
		pr_err("%s:failed to get pinctrl\n", __func__);
		return PTR_ERR(sms_power_ctrl->pinctrl);
	}

	sms_power_ctrl->gpio_state_default = pinctrl_lookup_state(sms_power_ctrl->pinctrl,
		"default");
	if (IS_ERR(sms_power_ctrl->gpio_state_default)) {
		pr_err("%s:can not get active pinstate\n", __func__);
		return -EINVAL;
	}

	ret = pinctrl_select_state(sms_power_ctrl->pinctrl,
		sms_power_ctrl->gpio_state_default);
	if (ret)
		pr_err("%s:set state failed!\n", __func__);

	sms_power_ctrl->sms_host_reset_pin = of_get_named_gpio(node, "dmtv,host-reset-pin", 0);
	if (sms_power_ctrl->sms_host_reset_pin < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"host-reset-pin", node->full_name, sms_power_ctrl->sms_host_reset_pin);
		goto error;
	}
	
	sms_power_ctrl->sms_host_reset1_pin = of_get_named_gpio(node, "dmtv,host-reset1-pin", 0);
	if (sms_power_ctrl->sms_host_reset1_pin < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"host-reset1-pin", node->full_name, sms_power_ctrl->sms_host_reset1_pin);
		goto error;
	}

	sms_power_ctrl->sms_host_pwrio_pin = of_get_named_gpio(node, "dmtv,host-powerio-pin", 0);
	if (sms_power_ctrl->sms_host_pwrio_pin < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"host-powerio-pin", node->full_name, sms_power_ctrl->sms_host_pwrio_pin);
		goto error;
	}

	sms_power_ctrl->sms_host_pwrcore_pin = of_get_named_gpio(node, "dmtv,host-powercore-pin", 0);
	if (sms_power_ctrl->sms_host_pwrcore_pin < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"host-powercore-pin", node->full_name, sms_power_ctrl->sms_host_pwrcore_pin);
		goto error;
	}

	sms_power_ctrl->vdd_io = regulator_get(&pdev->dev, "vdd-io");
	if (IS_ERR(sms_power_ctrl->vdd_io)) {
		ret = PTR_ERR(sms_power_ctrl->vdd_io);
		dev_err(&pdev->dev,
			"Regulator get failed vdd io ret=%d\n", ret);
		goto error;
	}

	if (regulator_count_voltages(sms_power_ctrl->vdd_io) > 0) {
		ret = regulator_set_voltage(sms_power_ctrl->vdd_io,
				SMS4470_VDD_IO_MIN_UV,
				SMS4470_VDD_IO_MAX_UV);
		if (ret) {
			dev_err(&pdev->dev,
			"Regulator set_vtg failed vdd_io ret=%d\n", ret);
			goto reg_vddio_put;
		}
	}

	sms_power_ctrl->vdd_emi= regulator_get(&pdev->dev, "vdd-emi");
	if (IS_ERR(sms_power_ctrl->vdd_emi)) {
		ret = PTR_ERR(sms_power_ctrl->vdd_emi);
		dev_err(&pdev->dev,
			"Regulator get failed vdd io ret=%d\n", ret);
		goto reg_vddio_set_vtg;
	}

	if (regulator_count_voltages(sms_power_ctrl->vdd_emi) > 0) {
		ret = regulator_set_voltage(sms_power_ctrl->vdd_emi,
				SMS4470_VDD_EMI_MIN_UV,
				SMS4470_VDD_EMI_MAX_UV);
		if (ret) {
			dev_err(&pdev->dev,
			"Regulator set_vtg failed vdd_emi ret=%d\n", ret);
			goto reg_vddemi_put;
		}
	}
	
	ret = sms_power_ctrl_of_get_pinctrl(&pdev->dev);
	if (ret < 0) {
		if (ret != EPROBE_DEFER)
			pr_err("sms_of_probe: get pinctrl fail: %d\n", ret);
	}

	//sms_chip_poweron();
	//sms_sdio_status = 1;
	//sms_sdio_on();
	
	return 0;

reg_vddemi_put:
	regulator_put(sms_power_ctrl->vdd_emi);
reg_vddio_set_vtg:
	if (regulator_count_voltages(sms_power_ctrl->vdd_io) > 0)
		regulator_set_voltage(sms_power_ctrl->vdd_io, 0, SMS4470_VDD_IO_MAX_UV);
reg_vddio_put:
	regulator_put(sms_power_ctrl->vdd_io);

error:
	devm_kfree(&pdev->dev, sms_power_ctrl);
	return ret;

}

static int sms_power_ctrl_of_remove(struct platform_device *pdev)
{
	struct sms_power_ctrl_info *sms_power_ctrl =
	    (struct sms_power_ctrl_info *)platform_get_drvdata(pdev);

	devm_kfree(&pdev->dev, sms_power_ctrl);
	return 0;
}

static const struct of_device_id sms_power_ctrl_of_match[] = {
	{ .compatible = "dmtv,sms4470_power" },
	{ },
};
MODULE_DEVICE_TABLE(of, sms_power_ctrl_of_match);

static struct platform_driver sms_power_driver = {
	.probe		= sms_power_ctrl_of_probe,
	.remove		= sms_power_ctrl_of_remove,
	.driver = {
		.of_match_table = of_match_ptr(sms_power_ctrl_of_match),
		.name		= "dmtv,sms4470_power",
		.owner		= THIS_MODULE,
	}
};


static int __init sms_power_ctrl_init(void)
{
	return platform_driver_register(&sms_power_driver);
}

static void __exit sms_power_ctr_exit(void)
{
	platform_driver_unregister(&sms_power_driver);
}

module_init(sms_power_ctrl_init);
module_exit(sms_power_ctr_exit);

MODULE_AUTHOR("longcheer");
MODULE_DESCRIPTION("sms power control");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");


