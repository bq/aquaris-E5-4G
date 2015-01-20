#include <linux/init.h>	
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

#include <linux/fb.h>

/*add by Wenke Ma, for hall switch key code*/
#define KEY_HALL_OPEN                0x222
#define KEY_HALL_CLOSE               0x223


#define GPIO_HALL_EINT_PIN 114

struct hall_switch_info
{
	struct mutex io_lock;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	int irq;
	int hall_switch_state;
	struct input_dev *ipdev;
};


static irqreturn_t hall_interrupt(int irq, void *data)
{
	struct hall_switch_info *hall_info = data;
	int hall_gpio;
/*	int ret;
	ret = mutex_lock(&hall_info->io_lock);
	if(ret == 0){
		pr_err("Macle mutex lock already locked,so return %d\n", ret);
		return IRQ_HANDLED;
	}*/
	disable_irq_nosync(irq);
	hall_gpio = gpio_get_value(hall_info->irq_gpio);
	pr_err("Macle irq interrupt gpio = %d\n", hall_gpio);
	if(hall_gpio == hall_info->hall_switch_state){
		return IRQ_HANDLED;
	}else{
		hall_info->hall_switch_state = hall_gpio;
		pr_err("Macle report key\n");
	}
	if (hall_gpio) {
			input_report_key(hall_info->ipdev, KEY_HALL_OPEN, 1);
			input_report_key(hall_info->ipdev, KEY_HALL_OPEN, 0);
			input_sync(hall_info->ipdev);
	}else{
			input_report_key(hall_info->ipdev, KEY_HALL_CLOSE, 1);
			input_report_key(hall_info->ipdev, KEY_HALL_CLOSE, 0);
			input_sync(hall_info->ipdev);
	}
	 enable_irq(irq);
//	 mutex_unlock(&hall_info->io_lock);
     return IRQ_HANDLED;
}

static int hall_parse_dt(struct device *dev, struct hall_switch_info *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "qcom,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pr_info("Macle irq_gpio=%d\n", pdata->irq_gpio);
	return 0;
}

static int hall_probe(struct platform_device *pdev)
{
        int rc = 0;
		int err;
		struct hall_switch_info *hall_info;
        pr_err("Macle hall_probe\n");
		
		if (pdev->dev.of_node) {
			hall_info = kzalloc(sizeof(struct hall_switch_info), GFP_KERNEL);		  
			if (!hall_info) {
				pr_err("Macle %s: failed to alloc memory for module data\n",
					 __func__);
				return -ENOMEM;
			}
			err = hall_parse_dt(&pdev->dev, hall_info);
			if (err) {
			   dev_err(&pdev->dev, "Macle hall_probe DT parsing failed\n");
				goto free_struct;
			}
		} else{
			return -ENOMEM;
		}
		mutex_init(&hall_info->io_lock);

		platform_set_drvdata(pdev, hall_info);

/*input system config*/
        hall_info->ipdev = input_allocate_device();
        if (!hall_info->ipdev) {
                pr_err("hall_probe: input_allocate_device fail\n");
				goto input_error;
        }
        hall_info->ipdev->name = "hall-switch-input";
        input_set_capability(hall_info->ipdev, EV_KEY, KEY_HALL_OPEN);
		input_set_capability(hall_info->ipdev, EV_KEY, KEY_HALL_CLOSE);
        set_bit(INPUT_PROP_NO_DUMMY_RELEASE, hall_info->ipdev->propbit);
        rc = input_register_device(hall_info->ipdev);
        if (rc) {
                pr_err("hall_probe: input_register_device fail rc=%d\n", rc);
				goto input_error;
        }

/*interrupt config*/
		if (gpio_is_valid(hall_info->irq_gpio)) {
	        rc = gpio_request(hall_info->irq_gpio, "hall-switch-gpio");
	        if (rc < 0) {
	                pr_err("hall_probe: gpio_request fail rc=%d\n", rc);
	                goto free_input_device;
	        }

			rc = gpio_direction_input(hall_info->irq_gpio);
			if (rc < 0) {
	                pr_err("hall_probe: gpio_direction_input fail rc=%d\n", rc);
	                goto err_irq;
	        }

			
			hall_info->hall_switch_state = gpio_get_value(hall_info->irq_gpio);
	        pr_err("hall_probe: gpios = %d, gpion=%d\n", hall_info->hall_switch_state, hall_info->hall_switch_state);

	        hall_info->irq = gpio_to_irq(hall_info->irq_gpio);
			pr_err("Macle irq = %d\n", hall_info->irq);
			irq_set_irq_wake(hall_info->irq,1);
	        rc = request_threaded_irq(hall_info->irq, hall_interrupt, NULL,
	                                IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
	                                "hall-switch-irq", hall_info);
	        if (rc < 0) {
	                pr_err("hall_probe: request_irq fail rc=%d\n", rc);
	                goto err_irq;
	        }
		}else{
			pr_err("Macle irq gpio not provided\n");
			goto free_input_device;
		}
       pr_err("hall_probe end\n");
       return 0;
err_irq:
        gpio_free(hall_info->irq_gpio);

free_input_device:
        input_unregister_device(hall_info->ipdev);

input_error:
		platform_set_drvdata(pdev, NULL);
free_struct:
		kfree(hall_info);

        return rc;
}

static int hall_remove(struct platform_device *pdev)
{
		struct hall_switch_info *hall = platform_get_drvdata(pdev);
        pr_err("hall_remove\n");
        free_irq(hall->irq, hall->ipdev);
        gpio_free(hall->irq_gpio);
        input_unregister_device(hall->ipdev);
        return 0;
}


static struct of_device_id sn_match_table[] = {
	{ .compatible = "hall-switch,BU52031NVX", },
	{ },
}; 

static struct platform_driver hall_driver = {
        .probe                = hall_probe,
        .remove                = hall_remove,
        .driver                = {
			.name        = "hall-switch",
			.owner        = THIS_MODULE,
			.of_match_table = sn_match_table,
		},
};

static int __init hall_init(void)
{
	return platform_driver_register(&hall_driver);
}

static void __exit hall_exit(void)
{
	platform_driver_unregister(&hall_driver);
}

//module_platform_driver(hall_driver);

module_init(hall_init);
module_exit(hall_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("mawenke");
