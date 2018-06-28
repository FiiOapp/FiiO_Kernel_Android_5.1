/*
 * Copyright (C) Fiio JU 2016
 *
 * Author: yunxiZhang
 * License terms: GNU General Public License (GPL) version 2
 */
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>


struct au6465r {

    int irq;
    struct platform_device *dev;
    int (*power)(char on);
    struct au6465r_platform_data *pdata;
};

static struct au6465r *gpGU6465R;

static void au6465r_work_func(struct work_struct *work)
{
    int value;

    printk(KERN_WARNING "au6465r_work_func \n");

    value=gpio_get_value(gpGU6465R->pdata->irq_gpio);

    gpGU6465R->power(!value);

}

static DECLARE_DELAYED_WORK(au6465r_delay_work, au6465r_work_func);

static irqreturn_t au6465r_interrupt(int irq, void *handle)
{
	struct au6465r *pau6465r= handle;

    printk("Enter:%s %d\n",__FUNCTION__,__LINE__);
	disable_irq_nosync(irq);
    schedule_delayed_work(&au6465r_delay_work,msecs_to_jiffies(200));
	return IRQ_HANDLED;
}




static int __devinit au6465r_probe(struct platform_device *pdev)
{
	struct pwm_device *pwm;
    int ret;
	/*
	 * Nothing to be done in probe, this is required to get the
	 * device which is required for ab8500 read and write
	 */
	gpGU6465R= kzalloc(sizeof(struct au6465r), GFP_KERNEL);
	if (gpGU6465R== NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	gpGU6465R->dev = &pdev->dev;
    gpGU6465R->pdata=(struct au6465r_platform_data*) pdev->platform_data;
    gpio_request(gpGU6465R->pdata->irq_gpio,"GU6465R det gpio");
    if (ret != 0) {
        gpio_free(gpGU6465R->pdata->irq_gpio);
        printk("GU6465R det gpio request failed \n");
        return -EIO;
    }
    gpio_direction_input(gpGU6465R->pdata->irq_gpio);
    gpGU6465R->irq=gpio_to_irq(gpGU6465R->pdata->irq_gpio);


	ret = request_irq(gpGU6465R->irq, au6465r_interrupt,
                      IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
                      pdev->dev.driver->name, gpGU6465R);

	if (ret ) {
		printk(KERN_ERR "au6465r: request irq failed,ret is %d\n", ret);
        return ret;
	}










	platform_set_drvdata(pdev, gpGU6465R);
	dev_dbg(pwm->dev, "GU6465R probe successful\n");
	return 0;

}

static int __devexit au6465r_remove(struct platform_device *pdev)
{
	struct au6465r *pau6465r= platform_get_drvdata(pdev);
	dev_dbg(&pdev->dev, "au6465r driver removed\n");
	kfree(pau6465r);
	return 0;
}

static struct platform_driver au6465r_driver = {
	.driver = {
		.name = "au6465r",
		.owner = THIS_MODULE,
	},
	.probe = au6465r_probe,
	.remove = __devexit_p(au6465r_remove),
};

static int __init au6465r_init(void)
{
	return platform_driver_register(&au6465r_driver);
}

static void __exit au6465r_exit(void)
{
	platform_driver_unregister(&au6465r_driver);
}

subsys_initcall(au6465r_init);
module_exit(au6465r_exit);
MODULE_AUTHOR("Arun MURTHY <arun.murthy@stericsson.com>");
MODULE_DESCRIPTION("AB8500 Pulse Width Modulation Driver");
MODULE_ALIAS("AB8500 PWM driver");
MODULE_LICENSE("GPL v2");
