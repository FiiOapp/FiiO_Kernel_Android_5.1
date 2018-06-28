/*
 * linux/drivers/leds-pwm.c
 *
 * simple PWM based LED control
 *
 * Copyright 2009 Luotao Fu @ Pengutronix (l.fu@pengutronix.de)
 *
 * based on leds-gpio.c by Raphael Assenat <raph@8d.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/slab.h>

#include <linux/clk.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>
#include <mach/board.h>
#include <plat/pwm.h>

#define PWM_DIV              PWM_DIV2
#define PWM_APB_PRE_DIV      5000
#define BL_STEP              (500)
#define MAX_BRIGHTNESS_CORRECT (50)

/*
 * Debug
 */
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif

#define read_pwm_reg(addr)              __raw_readl(pwm_base + addr)

static struct clk *pwm_clk;
static void __iomem *pwm_base;
static int suspend_flag = 0;




struct led_pwm_data {
	struct led_classdev	cdev;
	struct pwm_device	*pwm;
    struct rk30_pwmled_info *info;
	unsigned int 		active_low;
	unsigned int		period;
    int irq;
};

struct led_pwm_data*  rk30_led;

static void led_pwm_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct led_pwm_data *led_dat =
		container_of(led_cdev, struct led_pwm_data, cdev);
	unsigned int max = led_dat->cdev.max_brightness;
	unsigned int period =  led_dat->period;

	if (brightness == 0) {
		pwm_config(led_dat->pwm, 0, period);
		pwm_disable(led_dat->pwm);
	} else {
		pwm_config(led_dat->pwm, brightness * period / max, period);
		pwm_enable(led_dat->pwm);
	}
}

static int led_update_status(struct led_pwm_data* led)
{

    static u32   brightness =5;
    static u8   flag_inc=1;
    u8 step=2;

	u32 id = led->info->pwm_id;
	u32 ref = led->info->bl_ref;
	u32 divh,div_total;

	div_total = read_pwm_reg(PWM_REG_LRC);
	if (ref) {
		divh = div_total*brightness/BL_STEP;
	} else {
		divh = div_total*(BL_STEP-brightness)/BL_STEP;
	}
	rk_pwm_setup(id, PWM_DIV, divh, div_total);
    /* printk("id: %d %d %d %d \n",id,PWM_DIV,divh,div_total); */

    // -- 
    if(flag_inc){

        if((brightness+=step) >= 9*BL_STEP/10){
            flag_inc=0;
        }

    }else{

        if((brightness-=step) <= 1*BL_STEP/10){
            flag_inc=1;
        }

    }
}

static void rk30_led_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(rk30_led_work, rk30_led_work_func);
static void rk30_led_work_func(struct work_struct *work)
{
	/* rk29_bl->props.state &= ~BL_CORE_DRIVER2; */
	led_update_status(rk30_led);

	schedule_delayed_work(&rk30_led_work, msecs_to_jiffies(rk30_led->info->delay_ms));
}
static int led_pwm_probe(struct platform_device *pdev)
{
	struct rk30_pwmled_info *led_info= pdev->dev.platform_data;
	struct led_pwm *cur_led;
	/* struct led_pwm_data *leds_data, *led_dat; */
    int irq;
	unsigned long pwm_clk_rate;
	int i, ret = 0;
	int pre_div = PWM_APB_PRE_DIV;
	u32 id  ;  led_info->pwm_id;
	u32 divh, div_total;

    printk(" led pwm init %d \n",__LINE__);
	if (!led_info)
		return -EBUSY;

	id  =  led_info->pwm_id;
	rk30_led= kzalloc(sizeof(struct led_pwm_data) * led_info->num_leds,
				GFP_KERNEL);
	if (!rk30_led)
		return -ENOMEM;



    irq = platform_get_irq(pdev, 0);
    if (ret < 0) {
        dev_err(&pdev->dev, "cannot find IRQ\n");
        return ret; 
    }
    rk30_led->irq = irq;

    rk30_led->info=led_info;
    if(led_info->io_init){

        led_info->io_init();
    }


	pwm_base = rk_pwm_get_base(id);
	pwm_clk = rk_pwm_get_clk(id);
	if(IS_ERR(pwm_clk) || !pwm_clk) {
		printk(KERN_ERR "failed to get pwm clock source\n");
		return -ENODEV;
	}

    pwm_clk_rate = clk_get_rate(pwm_clk);
    div_total = pwm_clk_rate / pre_div;

    div_total >>= (1 + (PWM_DIV >> 9));
    div_total = (div_total) ? div_total : 1;
    if(led_info->bl_ref) {
        divh = 0;
    } else {
        divh = div_total;
    }

    clk_enable(pwm_clk);
    rk_pwm_setup(id, PWM_DIV, divh, div_total);


	schedule_delayed_work(&rk30_led_work, msecs_to_jiffies(led_info->delay_ms));

    printk(" led pwm init pwm rate %d divh %d div_total %d io: 0x%x \n",pwm_clk_rate ,divh,div_total,rk_pwm_get_base(3));
#if 0
	for (i = 0; i < pdata->num_leds; i++) {
		cur_led = &pdata->leds[i];
		led_dat = &leds_data[i];

		led_dat->pwm = pwm_request(cur_led->pwm_id,
				cur_led->name);
		if (IS_ERR(led_dat->pwm)) {
			ret = PTR_ERR(led_dat->pwm);
			dev_err(&pdev->dev, "unable to request PWM %d\n",
					cur_led->pwm_id);
			goto err;
		}

		led_dat->cdev.name = cur_led->name;
		led_dat->cdev.default_trigger = cur_led->default_trigger;
		led_dat->active_low = cur_led->active_low;
		led_dat->period = cur_led->pwm_period_ns;
		led_dat->cdev.brightness_set = led_pwm_set;
		led_dat->cdev.brightness = LED_OFF;
		led_dat->cdev.max_brightness = cur_led->max_brightness;
		led_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;

		ret = led_classdev_register(&pdev->dev, &led_dat->cdev);
		if (ret < 0) {
			pwm_free(led_dat->pwm);
			goto err;
		}
	}
#endif

	platform_set_drvdata(pdev, rk30_led);

	return 0;

err:
	kfree(rk30_led);

	return ret;
}

static int __devexit led_pwm_remove(struct platform_device *pdev)
{
	int i;
	struct led_pwm_platform_data *pdata = pdev->dev.platform_data;
	struct led_pwm_data *leds_data;

	leds_data = platform_get_drvdata(pdev);

	kfree(leds_data);

	return 0;
}

static struct platform_driver led_pwm_driver = {
	.probe		= led_pwm_probe,
	.remove		= __devexit_p(led_pwm_remove),
	.driver		= {
		.name	= "rk30_pwm_leds",
		.owner	= THIS_MODULE,
	},
};

static int __init led_pwm_init(void)
{
	return platform_driver_register(&led_pwm_driver);
}

static void __exit led_pwm_exit(void)
{
	platform_driver_unregister(&led_pwm_driver);
}

module_init(led_pwm_init);
module_exit(led_pwm_exit);

MODULE_AUTHOR("Luotao Fu <l.fu@pengutronix.de>");
MODULE_DESCRIPTION("PWM LED driver for PXA");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-pwm");
