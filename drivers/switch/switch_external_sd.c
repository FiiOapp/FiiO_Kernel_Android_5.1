/*
 *  drivers/switch/switch_external_sd.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/wakelock.h>
#include <linux/fs.h>

#if 0
#define DBG(fmt, args...)	dprintk(0,"*** " fmt, ## args)
#else
#define DBG(fmt, args...)	do{}while(0)
#endif

static int debug=5;
module_param(debug, int, S_IRUGO|S_IWUSR);
#define dprintk(level, fmt, arg...) do {			\
	if (debug >= level) 					\
	    printk("USB_MMC " fmt, ## arg); } while (0)



#define  SD2_DET_PIN     RK30_PIN1_PB7
#define  SD2_POWER_EN    RK30_PIN2_PB1
#define  SD2_DET_EN      RK30_PIN2_PD5

#define DELAY_TIME 1000

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend usb_mmc_early_suspend;
#endif

static struct delayed_work external_sd_work;

static int sd_irq;
static struct work_struct sdcard_work;
static irqreturn_t sdcard_detect_irq_handler(int irq, void *dev_id);

static void set_sdcard_power_onoff(bool state);
static struct class *external_sd_class;
static struct device *external_sd_dev;
int external_sd_status = 0;
static bool gsdcard_power=false;
static spinlock_t gpower_lock;
static struct wake_lock usb_sdmmc_lock;
void send_sdcard_status_uevent(char *);

static void external_sd_work_func(struct work_struct *data)
{
        dprintk(0,"%s:  enter !\n", __func__);
        set_sdcard_power_onoff(0);
}

static bool is_sdcard_insert(void)
{
        return gpio_get_value(SD2_DET_PIN) == 0 ? true : false;
}

static void set_sdcard_power_onoff(bool state)
{
       gsdcard_power=state;
       if (state) {
           //open external sdcard power
           dprintk(0,"open  external  sdcard  power !!!\n");
           gpio_set_value(SD2_POWER_EN, 1);
           msleep(200);
           gpio_set_value(SD2_DET_EN, 0);
       } else {
           //close external sdcard power
           dprintk(0,"close  external  sdcard  power !!!\n");
           gpio_set_value(SD2_DET_EN, 1);
           msleep(200);
           gpio_set_value(SD2_POWER_EN, 0);
       }
}

static void init_sdcard_power(void)
{
       bool state;

       state = is_sdcard_insert();
       set_sdcard_power_onoff(state);
}

extern void rk28_send_wakeup_key(void);
static void sdcard_detect_work(struct work_struct *work)
{
       int irq_flag;

       dprintk(0,"%s: enter !!!\n", __func__);

       // delay at least one jeffy for debounce
       msleep(5);
       external_sd_status = 0;

       //send sd insert or pull out uevent
       if(is_sdcard_insert())
           send_sdcard_status_uevent("insert");
       else
           send_sdcard_status_uevent("pullout");

       //wake on the screen
       rk28_send_wakeup_key();

       init_sdcard_power();

       irq_flag = gpio_get_value(SD2_DET_PIN) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
       dprintk(0,"irq  %s \n",irq_flag==IRQF_TRIGGER_RISING? "FALLING" : "RISING");
       dprintk(0,"\n");
       irq_set_irq_type(sd_irq, irq_flag);
       enable_irq(sd_irq);
}

static irqreturn_t sdcard_detect_irq_handler(int irq, void *dev_id)
{
        disable_irq_nosync(irq);
	schedule_work(&sdcard_work);

        dprintk(0,"usb sdcard detect \n");
	return IRQ_HANDLED;
}

void set_external_sd_power_off(void)
{
        external_sd_status = 1;
        schedule_delayed_work(&external_sd_work, msecs_to_jiffies(0));
}
EXPORT_SYMBOL_GPL(set_external_sd_power_off);


//usb mmc power off on external_sd_suspend()
// power on in usb_mmc_resume() if need 
static void usb_mmc_suspend(struct early_suspend *h)
{

}

static void usb_mmc_resume(struct early_suspend *h)
{
    //dprintk(0,KERN_WARNING " external_sd_status  ===   %d\n", external_sd_status);
    if(!gsdcard_power && external_sd_status != 1){
        //disable wake irq
        disable_irq_wake(sd_irq);
        //open sdcard power
        if (is_sdcard_insert()){

        	wake_lock_timeout(&usb_sdmmc_lock,5*HZ);
            set_sdcard_power_onoff(1);
        }


    }else{
        dprintk(0,KERN_WARNING "USB MMC power has opened\n");
    }
}

static void external_sd_gpio_init(void)
{
        int ret;

        ret = gpio_request(SD2_POWER_EN, "sd2_power");
        if (ret) {
            dprintk(0,KERN_ERR "%s:  request gpio fail !!!\n", __func__);
            gpio_free(SD2_POWER_EN);
        }

        gpio_direction_output(SD2_POWER_EN, GPIO_LOW);
        gpio_set_value(SD2_POWER_EN, GPIO_HIGH);
        DBG("%s:  sdcard2  poweren  value  is   %d \n", __func__, gpio_get_value(SD2_POWER_EN));

        ret = gpio_request(SD2_DET_EN, "sd2_det_en");
        if (ret) {
            dprintk(0,KERN_ERR "%s:  request sdcard2_det gpio fail !!!\n", __func__);
            gpio_free(SD2_DET_EN);
        }

        gpio_direction_output(SD2_DET_EN, GPIO_LOW);
        gpio_set_value(SD2_DET_EN, GPIO_LOW);
        DBG("%s:  sdcard2_det  value  is   %d \n", __func__, gpio_get_value(SD2_DET_EN));
}

void send_sdcard_status_uevent(char *status)
{
        char *sd_empty[2]   = { "SD_STATUS=SD_EMPTY", NULL };
        char *sd_insert[2]   = { "SD_STATUS=SD_INSERT", NULL };
        char *sd_normal[2]   = { "SD_STATUS=SD_NORMAL", NULL };
        char *sd_pullout[2]   = { "SD_STATUS=SD_PULLOUT", NULL };
        char **sdcard_envp = NULL;

        if (strcmp(status, "empty") == 0) {
                sdcard_envp = sd_empty;
        } else if (strcmp(status, "insert") == 0){
                sdcard_envp = sd_insert;
        } else if (strcmp(status, "normal") == 0){
                sdcard_envp = sd_normal;
        } else if (strcmp(status, "pullout") == 0){
                sdcard_envp = sd_pullout;
        } else {
                sdcard_envp = NULL;
        }

        if (sdcard_envp) {
                kobject_uevent_env(&external_sd_dev->kobj, KOBJ_CHANGE, sdcard_envp);
                pr_info("sdcard2: status changed, send event !!!\n");
        }
}
EXPORT_SYMBOL_GPL(send_sdcard_status_uevent);

static ssize_t new_sd_status_show(struct device *dev, struct device_attribute *attr,
               char *buf)
{
        if (external_sd_status == 1) {
            strcpy(buf, "sd_null\n");
            return strlen(buf);
        }
   
        if(is_sdcard_insert()) {
            strcpy(buf, "sd_insert\n");
        } else {
            strcpy(buf, "sd_none\n");
        }

        return strlen(buf);
}

static DEVICE_ATTR(new_sd_status, S_IRUGO | S_IWUSR, new_sd_status_show, NULL);

static int new_sd_status_init(void)
{
        int ret;

        if (!external_sd_class) {
               external_sd_class = class_create(THIS_MODULE, "sdcard_status");
               if (IS_ERR(external_sd_class))
                       return PTR_ERR(external_sd_class);
        }

        external_sd_dev = device_create(external_sd_class, NULL, MKDEV(0, 0), NULL, "external_sd");
        if (IS_ERR(external_sd_dev)) {
            dprintk(0,KERN_ERR "%s: external_sd_dev init failed !!!  \n", __func__);
            return PTR_ERR(external_sd_dev);
        }

        ret = device_create_file(external_sd_dev, &dev_attr_new_sd_status);
        if (ret < 0) {
               dprintk(0,KERN_ERR "%s: device_create_file init failed !!!  \n", __func__);
               device_destroy(external_sd_class, MKDEV(0, 0));
               return ret;
        }

        return 0;
}

static int external_sd_probe(struct platform_device *pdev)
{
        int ret = 0;
        int flags;

        //init extern sdcard control pin
        external_sd_gpio_init();

        //add sdcard2 uevent
        ret = new_sd_status_init();
        if(ret < 0) {
            dprintk(0,KERN_ERR "%s: external_sd_class init failed !!!  \n", __func__);
            goto err;
        }
        //add end

        //add for sdcard2 detect test
	ret = gpio_request(SD2_DET_PIN, "sdcard2_det");
	if (ret < 0) {
		gpio_free(SD2_DET_PIN);
                dprintk(0,KERN_ERR "%s:  sdcard2_det  request  fail !!!  \n", __func__);
        }

	ret = gpio_direction_input(SD2_DET_PIN);
	if (ret < 0) {
		gpio_free(SD2_DET_PIN);
                dprintk(0,KERN_ERR "%s:  sdcard2_det  gpio_direction_input  fail !!!  \n", __func__);
        }


    spin_lock_init(&gpower_lock);          // 2.6.39 later
        //gpio_pull_updown(SD2_DET_PIN, GPIOPullUp);
        //gpio_pull_updown(SD2_DET_PIN, PullDisable);
	INIT_WORK(&sdcard_work, sdcard_detect_work);
    wake_lock_init(&usb_sdmmc_lock, WAKE_LOCK_SUSPEND, "usb_sdmmc_lock"); 
	sd_irq = gpio_to_irq(SD2_DET_PIN);
	if (sd_irq < 0) {
		ret = sd_irq;
		gpio_free(SD2_DET_PIN);
        dprintk(0,KERN_ERR "%s:  sdcard2_det  gpio_to_irq  fail !!!  \n", __func__);
	}

        DBG("%s: gpio_get_value(SD2_DET_PIN)  is   %d \n", __func__, gpio_get_value(SD2_DET_PIN));
        flags = gpio_get_value(SD2_DET_PIN) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	ret = request_irq(sd_irq, sdcard_detect_irq_handler,
                      flags , "sdcard_det", NULL);
	if (ret) {
		//gpio_free(SD2_DET_PIN);
                dprintk(0,KERN_ERR "%s:  sdcard2_det  request_irq  fail !!!  \n", __func__);
                goto irq_err;
	}
        init_sdcard_power();
        //add end

        INIT_DELAYED_WORK(&external_sd_work, external_sd_work_func);

#ifdef CONFIG_HAS_EARLYSUSPEND
   usb_mmc_early_suspend.suspend = NULL;
   usb_mmc_early_suspend.resume  = usb_mmc_resume;
   usb_mmc_early_suspend.level = 0x2;
   register_early_suspend(&usb_mmc_early_suspend);
#endif


        return ret;

irq_err:
        gpio_free(SD2_DET_PIN);
        cancel_work_sync(&sdcard_work);

err:
        class_destroy(external_sd_class);
        external_sd_class = NULL;
/*
        gpio_free(SD2_DET_PIN);
        cancel_work_sync(&sdcard_work);
        cancel_delayed_work_sync(&external_sd_work);
*/
}

static int __devexit external_sd_remove(struct platform_device *pdev)
{
        cancel_work_sync(&sdcard_work);
        cancel_delayed_work_sync(&external_sd_work);

        device_remove_file(external_sd_dev, &dev_attr_new_sd_status);
        device_destroy(external_sd_class, MKDEV(0, 0));
        class_destroy(external_sd_class);
        external_sd_class = NULL;

        return 0;
}

static int external_sd_shutdown(struct platform_device *pdev)
{
       dprintk(0,"%s: enter !!!\n", __func__);
       cancel_work_sync(&sdcard_work);                                                                                                                                                                          
       cancel_delayed_work_sync(&external_sd_work);  
       //close sdcard power
       set_sdcard_power_onoff(0);

       return 0;
}

#ifdef CONFIG_PM
static int external_sd_suspend()
{
        dprintk(0,"%s: enter !!!\n", __func__);

        //external_sd_status = 1;
        cancel_work_sync(&sdcard_work);                                                                                                                                                                          
        cancel_delayed_work_sync(&external_sd_work); 
        //enable wake irq
        enable_irq_wake(sd_irq); 
        //close sdcard power
        set_sdcard_power_onoff(0);
        return 0;
}

static int external_sd_resume()
{
#if 0
        //disable wake irq
        disable_irq_wake(sd_irq);

        //open sdcard power
        if (is_sdcard_insert())
                set_sdcard_power_onoff(1);

        return 0;
#endif
        return 0;
}
#else
#define external_sd_suspend	NULL
#define external_sd_resume	NULL
#endif


static struct platform_driver external_sd_driver = {
        .shutdown   = external_sd_shutdown,
	.suspend    = external_sd_suspend,
	.resume     = external_sd_resume,
        .probe      = external_sd_probe,
	.remove		= __exit_p(external_sd_remove),
	.driver		= {
		.name		= "external_sd",
                .owner	        = THIS_MODULE,
	},
};

extern int act8846_sdcard_power(char on);
static int __init external_sd_init(void)
{
        int ret;

        dprintk(0,"%s: enter !!!\n", __func__);

        //open sdcard power
        //act8846_sdcard_power(1);

        ret = platform_driver_register(&external_sd_driver);
        if (ret < 0) {
                dprintk(0,KERN_ERR "%s:  probe error !!!\n", __func__);
        }
	return ret;
}

static void __exit external_sd_exit(void)
{
	platform_driver_unregister(&external_sd_driver);
}

module_init(external_sd_init);
module_exit(external_sd_exit);

