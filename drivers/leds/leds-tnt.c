
/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/list.h>
#include <linux/pinctrl/consumer.h>
MODULE_LICENSE("Dual BSD/GPL");
extern int qpnp_lbc_led_sink_set(int);


#define LED_GPIO_FLASH_DRIVER_NAME	"qcom,leds-gpio-flash"
#define LED_TRIGGER_DEFAULT		"none"

struct led_gpio_flash_data {
	int flash_en;
	int flash_now;
	int brightness;

	struct led_classdev		cdev_r;
	struct led_classdev		cdev_g;
	struct led_classdev		cdev_b;

	
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
};


static struct of_device_id led_gpio_flash_of_match[] = {
	{.compatible = "leds_gpio_rgb,tnt",},
	{},
};

static void led_gpio_brightness_set_R(struct led_classdev *led_cdev,enum led_brightness value)
{
	int rc = 0;
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev_r);
    
	printk("%s, name=%s, value=%d\n", __func__, flash_led->cdev_r.name, value);
	
	printk("flash_en=%d,flash_now=%d\n", flash_led->flash_en,flash_led->flash_now);
	rc = gpio_direction_output(flash_led->flash_en, 0);
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
			       flash_led->flash_en);
			goto err;
		}
	rc = gpio_direction_output(flash_led->flash_now, 0);
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
			       flash_led->flash_now);
			goto err;
		}
	gpio_set_value(flash_led->flash_en, 0);
	gpio_set_value(flash_led->flash_now, 0);
	if(value == LED_OFF)
	qpnp_lbc_led_sink_set(0);
	else
	qpnp_lbc_led_sink_set(1);	
	
	err:
		return;

}

static void led_gpio_brightness_set_G(struct led_classdev *led_cdev,enum led_brightness value)
{
	int rc = 0;
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev_g);
    
	printk("%s, name=%s,value=%d\n", __func__, flash_led->cdev_g.name,value);
	
	printk("flash_en=%d,flash_now=%d\n", flash_led->flash_en,flash_led->flash_now);
	if(value == LED_OFF){
		 
		 rc = gpio_direction_output(flash_led->flash_en, 0);
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
			       flash_led->flash_en);
			goto err;
		}
	   gpio_set_value(flash_led->flash_en, 0);
	}else{
          
		  rc = gpio_direction_output(flash_led->flash_en, 1);
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
			       flash_led->flash_en);
			goto err;
		}
		gpio_set_value(flash_led->flash_en, 1);
		 rc = gpio_direction_output(flash_led->flash_now, 0);
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
			       flash_led->flash_now);
			goto err;
		}
		gpio_set_value(flash_led->flash_now, 0);
	}
  err:
	return;
}

static void led_gpio_brightness_set_B(struct led_classdev *led_cdev,enum led_brightness value)
{
	int rc = 0;
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev_b);
    
	printk("%s, name=%s value=%d\n", __func__, flash_led->cdev_b.name,value);
	
	printk("flash_en=%d,flash_now=%d\n", flash_led->flash_en,flash_led->flash_now);
	
	if(value == LED_OFF){
		 
		 rc = gpio_direction_output(flash_led->flash_now, 0);
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
			       flash_led->flash_now);
			goto err;
		}
		gpio_set_value(flash_led->flash_now, 0);
	}else{
	
	     rc = gpio_direction_output(flash_led->flash_en, 0);
          
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
			       flash_led->flash_en);
			goto err;
		}
		 gpio_set_value(flash_led->flash_en, 0);
		 
	     rc = gpio_direction_output(flash_led->flash_now, 1);
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
			       flash_led->flash_now);
			goto err;
		}
		gpio_set_value(flash_led->flash_now, 1);
	}
	
  err:
	return;
}

/*static enum led_brightness led_gpio_brightness_get(struct led_classdev
						   *led_cdev)
{
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);
	return flash_led->brightness;
}
*/
	


int leds_gpio_rgb_probe(struct platform_device *pdev)
{
	int rc = 0;
	//const char *temp_str;
	struct led_gpio_flash_data *flash_led = NULL;
	struct device_node *node = pdev->dev.of_node;

	printk("%s\n", __func__);
	flash_led = devm_kzalloc(&pdev->dev, sizeof(struct led_gpio_flash_data),
				 GFP_KERNEL);
	if (flash_led == NULL) {
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}
   #if 0
	flash_led->cdev.default_trigger = LED_TRIGGER_DEFAULT;
	rc = of_property_read_string(node, "linux,default-trigger", &temp_str);
	if (!rc)
		flash_led->cdev.default_trigger = temp_str;
   #endif
	flash_led->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flash_led->pinctrl)) {
		pr_err("%s:failed to get pinctrl\n", __func__);
		return PTR_ERR(flash_led->pinctrl);
	}

	flash_led->gpio_state_default = pinctrl_lookup_state(flash_led->pinctrl,
		"msm8916_default");
	if (IS_ERR(flash_led->gpio_state_default)) {
		pr_err("%s:can not get active pinstate\n", __func__);
		return -EINVAL;
	}

	rc = pinctrl_select_state(flash_led->pinctrl,
		flash_led->gpio_state_default);
	if (rc)
		pr_err("%s:set state failed!\n", __func__);

	flash_led->flash_en = of_get_named_gpio(node, "qcom,flash-en", 0);
	if (flash_led->flash_en < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-en", node->full_name, flash_led->flash_en);
		goto error;
	} else {
		rc = gpio_request(flash_led->flash_en, "FLASH_EN");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->flash_en, rc);

			goto error;
		}
         
	}

	flash_led->flash_now = of_get_named_gpio(node, "qcom,flash-now", 0);
	if (flash_led->flash_now < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-now", node->full_name, flash_led->flash_now);
		goto error;
	} else {
		rc = gpio_request(flash_led->flash_now, "FLASH_NOW");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->flash_now, rc);
			goto error;
		}
		
	}
#if 0
	rc = of_property_read_string(node, "linux,name", &flash_led->cdev.name);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to read linux name. rc = %d\n",
			__func__, rc);
		goto error;
	}
	else
		printk("%s, name=%s\n", __func__, flash_led->cdev.name);
#endif
	platform_set_drvdata(pdev, flash_led);
	
	#if 1
    flash_led->cdev_r.name = "red";
	flash_led->cdev_r.brightness = LED_OFF;
	flash_led->cdev_r.brightness_set = led_gpio_brightness_set_R;
	//flash_led->cdev.brightness_get = led_gpio_brightness_get;
	rc = led_classdev_register(&pdev->dev, &flash_led->cdev_r);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to register led dev. rc = %d\n",
			__func__, rc);
		goto error;
	}
	flash_led->cdev_b.name = "blue";
	flash_led->cdev_b.brightness = LED_OFF;
	flash_led->cdev_b.brightness_set = led_gpio_brightness_set_B;
	//flash_led->cdev.brightness_get = led_gpio_brightness_get;
	rc = led_classdev_register(&pdev->dev, &flash_led->cdev_b);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to register led dev. rc = %d\n",
			__func__, rc);
		goto error;
	}
	flash_led->cdev_g.name = "green";
	flash_led->cdev_g.brightness = LED_OFF;
	flash_led->cdev_g.brightness_set = led_gpio_brightness_set_G;
	//flash_led->cdev.brightness_get = led_gpio_brightness_get;
	rc = led_classdev_register(&pdev->dev, &flash_led->cdev_g);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to register led dev. rc = %d\n",
			__func__, rc);
		goto error;
	}
	pr_err("%s:probe successfully!\n", __func__);
	
	return 0;
    #endif
error:
	devm_kfree(&pdev->dev, flash_led);
	return rc;
}

int leds_gpio_rgb_remove(struct platform_device *pdev)
{
	struct led_gpio_flash_data *flash_led =
	    (struct led_gpio_flash_data *)platform_get_drvdata(pdev);

	led_classdev_unregister(&flash_led->cdev_r);
	led_classdev_unregister(&flash_led->cdev_g);
	led_classdev_unregister(&flash_led->cdev_b);
	devm_kfree(&pdev->dev, flash_led);
	return 0;
}

static struct platform_driver leds_gpio_rgb_driver = {
	.probe = leds_gpio_rgb_probe,
	.remove = leds_gpio_rgb_remove,
	.driver = {
		   .name = "leds_gpio_rgb",
		   .owner = THIS_MODULE,
		   .of_match_table = led_gpio_flash_of_match,
		   }
};

static int __init leds_gpio_rgb_init(void)
{
	printk("%s\n", __func__);
	return platform_driver_register(&leds_gpio_rgb_driver);
}

static void __exit leds_gpio_rgb_exit(void)
{
	return platform_driver_unregister(&leds_gpio_rgb_driver);
}

late_initcall(leds_gpio_rgb_init);
module_exit(leds_gpio_rgb_exit);

MODULE_DESCRIPTION("QCOM GPIO LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-tnt");
