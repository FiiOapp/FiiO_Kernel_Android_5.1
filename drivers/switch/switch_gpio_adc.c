/*
 *  drivers/switch/switch_gpio.c
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
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
//#include <mach/irqs.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/delay.h>
/* #include <sound/hp_switch.h> */
#include <linux/adc.h>

#include "../../drivers/regulator/axp_power/axp-mfd.h"
#include "../../sound/soc/codecs/es9018.h"
#include<linux/i2c.h>

#if 0
#define DBG(fmt, args...)	printk("*** " fmt, ## args)
#else
#define DBG(fmt, args...)	do{}while(0)
#endif

#define HP_LEVEL    204
//#define LINE_LEVEL  410
#define LINE_LEVEL_IN   740
#define LINE_LEVEL_OUT  960

#define DC_DETECT_LEVEL_MIN     284//340
#define DC_DETECT_LEVEL_MAX     910//680

#define AM2_BOOST_GPIO  RK30_PIN0_PB3
#define GAIN_GPIO       RK30_PIN0_PC1
#define AMP_ID1_GPIO    RK30_PIN1_PA5
#define AMP_ID2_GPIO    RK30_PIN0_PA2
#define AMP_ID3_GPIO    RK30_PIN0_PA6

#define OP_DET_PIN      RK30_PIN0_PC0
extern int adc1_level;

static int set_amp_card_gpio_value(int val);
bool dac_power_hold = false;
static bool dock_restart_flag = false;

struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
        int l_count;
	struct work_struct work;
        bool bool_dc_detect;

        //add for line_det
        struct timer_list timer;
        struct adc_client *client;
        struct switch_dev headset;
        int line_count;
        int h2w_count;
        
        //add for amp_card detect
        struct switch_dev amp_card;

        //add for dc-detect
        struct timer_list dc_detect_timer;
        struct adc_client *dc_detect_client;
        struct switch_dev dc_detect;
        int dc_count;

        struct timer_list line_control_timer;
        struct switch_dev line_control;
        
        struct switch_dev accessory_dev;
};

static struct gpio_switch_data *gSwitchData = NULL;

static uint8_t Am_Type;
static bool amp_flag = false;
static int last_state;
static irqreturn_t gpio_irq_handler(int irq, void *dev_id);
static int amp_card_gpio_value(struct gpio_switch_data *);

extern void rk29_send_pause_key(int);
extern void rk29_send_prev_key(int);
extern void rk29_send_next_key(int);
extern void exAudioLinkChange(char);

static void gpio_switch_work(struct work_struct *work)
{
	int state;
	struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);
    DBG("%s enter!!!++++++++++the line is %d\n", __FUNCTION__, __LINE__);

    //switch_set_state(&data->sdev, last_state);
    free_irq(data->irq,data);
    msleep(50);//msleep(800);
    state = gpio_get_value(data->gpio);
    if(last_state != state) {
        last_state = state;
    }

#ifdef CONFIG_MFD_RK616
    if(state == 0)
        hp_switch_on();
    else
        hp_switch_off();
#endif
	//switch_set_state(&data->sdev, state);

    //data->irq = gpio_to_irq(data->gpio);
    //free_irq(data->irq,data);
    //msleep(50);
    request_irq(data->irq, gpio_irq_handler, (state==0) ? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING , "h2w",data);
    DBG("Switch the headset status : %d\n",state);

    if (state == 0 && data->line_control.status == 1 && data->headset.state == 1) {
        printk("lipf_debug:  timer test    %d   ***\n", timer_pending(&data->line_control_timer));
        if (timer_pending(&data->line_control_timer)) {
            data->l_count++;
        }

        mod_timer(&data->line_control_timer, jiffies + msecs_to_jiffies(300));
    }
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;
        disable_irq_nosync(irq);
	schedule_work(&switch_data->work);
    //set_irq_type(switch_data->irq,gpio_get_value(switch_data->gpio)? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING);
        DBG("Headset interrupt \n");
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

extern int gAndroid_boot_completed;
extern void kernel_restart(char *cmd);
extern void usb20otg_power_enable(int enable);
extern int axp_usb_det();
#define OTG_DRV_VBUS  RK30_PIN3_PD5

static void line_adc_callback(struct adc_client *client, void *client_param, int result)
{
        int level = result;
        struct gpio_switch_data *switch_data = (struct gpio_switch_data *)client_param;
        uint8_t val;
        int amp_val;

        DBG("lipf_debug: %s enter!\n", __FUNCTION__);
        DBG("line_adc_callback read adc value: %d\n",level);

        if (switch_data->amp_card.status == 1 && amp_flag)
        {
            DBG("lipf_debug: set_amp_card_gpio_value to  high  !!!\n ");
            amp_flag = false;
            set_amp_card_gpio_value(1);
        } else if (switch_data->amp_card.status == 0 && !amp_flag) {
            DBG("lipf_debug: set_amp_card_gpio_value to  low  !!!\n ");
            amp_flag = true;
            set_amp_card_gpio_value(0);
        }

        /* read AXP228 NTC pin ADC_value, judge the headset in or out */
        axp_read(&axp->dev, 0x58, &val);

        if (val < 0)
        {
            printk("%s:get AXP adc value err == %d\n", __FUNCTION__, val);
            return;
        }
         
        /* printk(" axp dac %d\n",val); */
        if (val >=0 && val < HP_LEVEL)
        {
                //headset is insert
                if(switch_data->headset.state != 1) {
                    switch_data->h2w_count++;

                    if (switch_data->h2w_count > 5) {
                        exAudioLinkChange(EVENT_HP_IN);

                        switch_set_state(&switch_data->headset, 1);
                        switch_data->h2w_count = 5;
                    }
                }
        } 
        else
        {
                //headset is out
                if(switch_data->headset.state != 0) {
                    exAudioLinkChange(EVENT_HP_OUT);
                    switch_set_state(&switch_data->headset, 0);
                }
                switch_data->h2w_count = 0;
        }
 
        /* read adc0 value, judge line in or out */
	if(level < 0)
	{
		printk("%s:get adc level err = %d!\n",__FUNCTION__, level);
		return;
	}

        if(level >=0 && level < LINE_LEVEL_IN)
        {
                //line is insert
                if(switch_data->sdev.state != 1) {
                    switch_data->line_count++;

                    if (switch_data->line_count > 5) {
                        exAudioLinkChange(EVENT_LINEOUT_IN);

                        switch_set_state(&switch_data->sdev, 1);
                        switch_data->line_count = 5;
                    }
                }
        }
        else if (level > LINE_LEVEL_OUT)
        {
                //line is out
                if(switch_data->sdev.state != 0) {
                    exAudioLinkChange(EVENT_LINEOUT_OUT);
                    switch_set_state(&switch_data->sdev, 0);
                }
                switch_data->line_count = 0;
        }
        
        if (gpio_get_value(OP_DET_PIN) == 1) {
            if (switch_data->accessory_dev.state != 0) {
                printk("acessory is  out  !!!\n");
                exAudioLinkChange(EVENT_LINEOUT_OUT);
#if defined(CONFIG_LIDA_MACH_X7)
                usb20otg_power_enable(0);
#endif
                dac_power_hold = false;
            }

            if (dock_restart_flag)
                dock_restart_flag = false;

            switch_set_state(&switch_data->accessory_dev, 0);
        } else {
            //only add for DK5
            if (!dock_restart_flag && switch_data->accessory_dev.status == 1 && adc1_level < 950) 
                kernel_restart(NULL);
            //add end

            if (!dock_restart_flag)
                dock_restart_flag = true;

            if (gAndroid_boot_completed == 1) {            
            if (/*adc1_level > 200 &&*/!axp_usb_det() && adc1_level < 800)
            {
#if defined(CONFIG_LIDA_MACH_X7)
                if (gpio_get_value(OTG_DRV_VBUS) != 1)
                    usb20otg_power_enable(1);
#endif
            } else {
#if defined(CONFIG_LIDA_MACH_X7)
                if (gpio_get_value(OTG_DRV_VBUS) != 0)
                    usb20otg_power_enable(0);
#endif
                if (switch_data->accessory_dev.state != 1) {
                    dac_power_hold = true;
                }
            }

            if (switch_data->accessory_dev.state != 1) {
                exAudioLinkChange(EVENT_LINEOUT_IN);
            }
            
            switch_set_state(&switch_data->accessory_dev, 1);
        }            
        }

        //add amp_card detect
        amp_val = switch_data->amp_card.state;
        if (amp_val != amp_card_gpio_value(switch_data)) {
            switch_set_state(&switch_data->amp_card, amp_card_gpio_value(switch_data));
        }
        //add end       
}

extern int set_dac_power(char on);
extern int getLinkPlaybackStatus(void);

#define AM_POWER_EN    RK30_PIN3_PD7
int get_am_power_status(void)
{
    int val;

    gpio_request(AM_POWER_EN, NULL);
    val = gpio_get_value(AM_POWER_EN);
    gpio_free(AM_POWER_EN);
    
    return val;
}

static void dc_detect_adc_callback(struct adc_client *client, void *client_param, int result)
{
        int level = result;
        struct gpio_switch_data *switch_data = (struct gpio_switch_data *)client_param;

        DBG("lipf_debug: %s enter!\n", __FUNCTION__);
        DBG("line_adc_callback read adc value: %d\n", level);

        if (level < 0)
        {
            printk("%s:get dc-detect adc2 value err == %d\n", __FUNCTION__, level);
            return;
        }

        if (switch_data->sdev.state == 0) {
        if (level >= DC_DETECT_LEVEL_MIN && level <= DC_DETECT_LEVEL_MAX)
        {
            //直流检测正常, 不关闭音频相关的电源
            if (switch_data->dc_detect.state != 0 && 1 == switch_data->headset.state) 
            {
                //set_dac_power(1);
                //exAudioLinkChange(EVENT_HP_IN);
            }

            switch_set_state(&switch_data->dc_detect, 0);
            switch_data->dc_count = 0;
            //switch_data->bool_dc_detect = false;
        } else {
            //直流检测异常, 关闭音频相关的电源
            if (1 == switch_data->headset.state && getLinkPlaybackStatus() == 1) {
                if (switch_data->dc_detect.state != 1 && get_am_power_status() == 1) 
                {
                    printk("%s:  get_am_power_status()  is   %d\n", __func__, get_am_power_status());
                    switch_data->dc_count++;
                    if (switch_data->dc_count > 5) {
                        printk("%s: disable dac power !\n", __func__);
                        exAudioLinkChange(EVENT_HP_OUT);
                        //exAudioLinkChange(EVENT_PLAYBACK_OFF);//
                        set_dac_power(0);
                        switch_data->dc_count = 5;

                        switch_set_state(&switch_data->dc_detect, 1);
                    }
                }
            }
        }
        }
}

static void line_adc_timer(unsigned long _data)
{
	struct gpio_switch_data *switch_data = (struct gpio_switch_data *)_data;

        DBG("lipf_debug: %s enter!\n", __FUNCTION__);

	adc_async_read(switch_data->client);
	mod_timer(&switch_data->timer, jiffies + msecs_to_jiffies(100));
}

static void dc_detect_adc_timer(unsigned long _data)
{
	struct gpio_switch_data *switch_data = (struct gpio_switch_data *)_data;

	adc_async_read(switch_data->dc_detect_client);
	mod_timer(&switch_data->dc_detect_timer, jiffies + msecs_to_jiffies(300));
}

static void line_control_timer(unsigned long _data)
{
	struct gpio_switch_data *switch_data = (struct gpio_switch_data *)_data;
        int state = 0;
        state = gpio_get_value(switch_data->gpio);

        if (state == 1) {
            if (switch_data->l_count == 0) {
                DBG("%s:  l_count  0000 \n", __FUNCTION__);
                rk29_send_pause_key(state);
                rk29_send_pause_key(!state);
            } else if (switch_data->l_count == 1) {
                DBG("%s:  l_count  1111 \n", __FUNCTION__);
                rk29_send_next_key(state);
                rk29_send_next_key(!state);
            } else if (switch_data->l_count == 2) {
                DBG("%s:  l_count  2222 \n", __FUNCTION__);
                rk29_send_prev_key(state);
                rk29_send_prev_key(!state);
            }
        }
        switch_data->l_count = 0;
}

static void headset_det_in(void)
{
        uint8_t val;

        // set TS pin as external input for ADC and do not affect the charge.
        // add by YunxiZhang
        axp_set_bits(&axp->dev, 0x84, 0x04);
        // enable TS ADC
        axp_set_bits(&axp->dev, 0x82, 0x01);
        axp_read(&axp->dev, 0x82, &val);
        printk("lipf_debug: read AXP 0x82 reg value is (%x)!\n", val);

}

//add by lipf for amp_card detect

static void init_amp_card_gpio(void)
{
        int ret;

        //amp card request and init gpio
        ret = gpio_request(AMP_ID1_GPIO, NULL);
        if (ret != 0) {
            DBG("%s:  id1_gpio  request failed !!!\n", __FUNCTION__);
        }

        gpio_pull_updown(AMP_ID1_GPIO, GPIOPullUp);
        ret = gpio_direction_input(AMP_ID1_GPIO);
        if (ret) {
            DBG("failed to set AMP_ID1_GPIO input\n");
        }

        ret = gpio_request(AMP_ID2_GPIO, NULL);
        if (ret != 0) {
            DBG("%s:  id2_gpio  request failed !!!\n", __FUNCTION__);
        }

        gpio_pull_updown(AMP_ID2_GPIO, GPIOPullUp);
        ret = gpio_direction_input(AMP_ID2_GPIO);
        if (ret) {
            DBG("failed to set AMP_ID2_GPIO input\n");
        }

        ret = gpio_request(AMP_ID3_GPIO, "id3_gpio");
        if (ret != 0) {
            DBG("%s:  id3_gpio  request failed !!!\n", __FUNCTION__);
        }

        gpio_pull_updown(AMP_ID3_GPIO, GPIOPullUp);
        ret = gpio_direction_input(AMP_ID3_GPIO);
        if (ret) {
            DBG("failed to set AMP_ID3_GPIO input\n");
        }

        ret = gpio_request(AM2_BOOST_GPIO, "am2_gpio");
        if (ret != 0) {
            gpio_free(AM2_BOOST_GPIO);
            printk("%s:  am2_gpio  request failed !!!\n", __FUNCTION__);
        }
        gpio_direction_output(AM2_BOOST_GPIO, 0);
        gpio_set_value(AM2_BOOST_GPIO, GPIO_HIGH);

        ret = gpio_request(GAIN_GPIO, "gain_gpio");
        if (ret != 0) {
            gpio_free(GAIN_GPIO);
            printk("%s:  gain_gpio  request failed !!!\n", __FUNCTION__);
        }
        gpio_direction_output(GAIN_GPIO, 0);
        gpio_set_value(GAIN_GPIO, GPIO_LOW);
}

static int set_amp_card_gpio_value(int val)
{
       gpio_direction_output(AM2_BOOST_GPIO, 1);
       gpio_set_value(AM2_BOOST_GPIO, !val);  

       gpio_direction_output(GAIN_GPIO, 0);
       gpio_set_value(GAIN_GPIO, val);  

       DBG("%s: AM2_BOOST_GPIO value  is  %d \n", __func__, gpio_get_value(AM2_BOOST_GPIO));   
       DBG("%s: GAIN_GPIO value  is  %d \n", __func__, gpio_get_value(GAIN_GPIO));

       return 0; 
}

static int amp_card_gpio_value(struct gpio_switch_data *switch_data)
{
       int value;
       int ID1, ID2, ID3;

       DBG("amp_card_gpio_value  enter !!!\n");

       ID1 = gpio_get_value(AMP_ID1_GPIO);
       DBG("the gpio value of ID1 is   %d  ***\n\n", ID1);

       ID2 = gpio_get_value(AMP_ID2_GPIO);
       DBG("the gpio value of ID2 is   %d  ***\n\n", ID2);

       ID3 = gpio_get_value(AMP_ID3_GPIO);
       DBG("the gpio value of ID3 is   %d  ***\n\n", ID3);

       if (ID1 == 1 && ID2 == 1 && ID3 == 1) {
           value = 7;
       } else if (ID1 == 1 && ID2 == 1 && ID3 == 0){
           value = 6;
       } else if (ID1 == 1 && ID2 == 0 && ID3 == 1){
           value = 5;
       } else if (ID1 == 1 && ID2 == 0 && ID3 == 0){
           value = 4;
       } else if (ID1 == 0 && ID2 == 1 && ID3 == 1){
           value = 3;
       } else if (ID1 == 0 && ID2 == 1 && ID3 == 0){
           value = 2;
       } else if (ID1 == 0 && ID2 == 0 && ID3 == 1){
           value = 1;
       } else if (ID1 == 0 && ID2 == 0 && ID3 == 0){
           value = 0;
       } else {
           value = 7;
       }
       DBG("the value is    %d   ***\n", value);

       Am_Type=(uint8_t)value;
       return value;
}


#define AM0 0
#define AM1 1
#define AM2 2
#define AM3 3
#define AM4 4
#define AM5 5
#define AM6 6
#define AM7 7
#define AM8 8

uint8_t audio_get_am_type()
{
    uint8_t am;
    switch(Am_Type){
    case 7: am=AM1;break;
    case 6: am=AM7;break;
    case 5: am=AM6;break;
    case 4: am=AM0;break;
    case 3: am=AM5;break;
    case 2: am=AM3;break;
    case 1: am=AM8;break;
    case 0: am=AM2;break;
    default:am=AM1;
    }
    return am;
}
EXPORT_SYMBOL_GPL(audio_get_am_type);

static void init_amp_card_gpio_value(struct gpio_switch_data *switch_data)
{
       int value;

       value = amp_card_gpio_value(switch_data);

       switch_set_state(&switch_data->amp_card, value);
}

int audio_get_amp_status(void)
{
       if (gSwitchData)
           return gSwitchData->amp_card.status;

       return 0;
}
EXPORT_SYMBOL_GPL(audio_get_amp_status);
//add end

//add for X7 accessory_dev DK1/K5
static void accessory_dev_init(void)
{
        int ret = 0;
        
	ret = gpio_request(OP_DET_PIN, "op_det");
	if (ret < 0) {
	        printk("%s: op_det gpio request failed !!!\n", __FUNCTION__);
		gpio_free(OP_DET_PIN);
        }
}

static int gpio_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	int ret = 0;
        int flags = 0;

        printk("register switch gpio \n");

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_gpio_print_state;

        //Init headsed (add by lipf)
        switch_data->headset.name = "h2w";
        ret = switch_dev_register(&switch_data->headset);
	if(ret < 0){
    	        printk(">>rk30 headset register fail!");
    	        return -ENOMEM;
	}
        //end

        //add for amp_card detect
        init_amp_card_gpio();
        switch_data->amp_card.name = "amp_card";
        ret = switch_dev_register(&switch_data->amp_card);
        if (ret < 0){
    	        printk(">>rk30 amp_card det register fail!");
    	        return -ENOMEM;
	}
        init_amp_card_gpio_value(switch_data);
        //add end

        //add for dc-detect
        switch_data->dc_detect.name = "dc_detect";
        ret = switch_dev_register(&switch_data->dc_detect);
        if (ret < 0){
    	        printk(">>rk30 dc-detect register fail!");
    	        return -ENOMEM;
	}
        //add end

        //add for line-control
        switch_data->line_control.name = "line_control";
        ret = switch_dev_register(&switch_data->line_control);
        if (ret < 0){
    	        printk(">>rk30 line_control register fail!");
    	        return -ENOMEM;
	}
        //add end
        
        //add for DK1/K5 detect
        switch_data->accessory_dev.name = "accessory_dev";
        ret = switch_dev_register(&switch_data->accessory_dev);
        if (ret < 0){
    	        printk(">>rk30 accessory_dev register fail!");
    	        return -ENOMEM;
	}     
	accessory_dev_init();   
        //add end

        ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = gpio_request(switch_data->gpio, pdev->name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

        gpio_pull_updown(switch_data->gpio, PullDisable);//GPIO_HIGH);
	INIT_WORK(&switch_data->work, gpio_switch_work);

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

        last_state = !gpio_get_value(switch_data->gpio);

        flags = gpio_get_value(switch_data->gpio) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	ret = request_irq(switch_data->irq, gpio_irq_handler,
                /*IRQF_TRIGGER_RISING*/flags , pdev->name, switch_data);
	if (ret < 0)
		goto err_request_irq;

	/* Perform initial detection */
	gpio_switch_work(&switch_data->work);

        /* init and register switch class for headset */
        headset_det_in();

        //add for line det
        setup_timer(&switch_data->timer, line_adc_timer, (unsigned long)switch_data);
        switch_data->client = adc_register(0, line_adc_callback, (void *)switch_data);
	if (!switch_data->client) {
		printk("line-det adc register error\n");;
		return -EINVAL;
	}
	mod_timer(&switch_data->timer, jiffies + msecs_to_jiffies(100));

        //add for dc detect by lipf
        setup_timer(&switch_data->dc_detect_timer, dc_detect_adc_timer, (unsigned long)switch_data);
        switch_data->dc_detect_client = adc_register(2, dc_detect_adc_callback, (void *)switch_data);
	if (!switch_data->dc_detect_client) {
		printk("dc-detect adc register error\n");;
		return -EINVAL;
	}
	mod_timer(&switch_data->dc_detect_timer, jiffies + msecs_to_jiffies(300));
        //add end

        //add for line-control
        setup_timer(&switch_data->line_control_timer, line_control_timer, (unsigned long)switch_data);

        gSwitchData = switch_data;
       
        printk("SWITCH-GPIO end!!!!!!\n");
	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
    switch_dev_unregister(&switch_data->headset);
    switch_dev_unregister(&switch_data->amp_card);
    switch_dev_unregister(&switch_data->dc_detect);
    switch_dev_unregister(&switch_data->line_control);
    switch_dev_unregister(&switch_data->accessory_dev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
        switch_dev_unregister(&switch_data->sdev);
        switch_dev_unregister(&switch_data->headset);
        switch_dev_unregister(&switch_data->amp_card);
        switch_dev_unregister(&switch_data->dc_detect);
        switch_dev_unregister(&switch_data->line_control);
        switch_dev_unregister(&switch_data->accessory_dev);

        del_timer_sync(&switch_data->timer);
        del_timer_sync(&switch_data->dc_detect_timer);
        del_timer_sync(&switch_data->line_control_timer);

        if (switch_data->client)
            adc_unregister(switch_data->client);

        if (switch_data->dc_detect_client)
            adc_unregister(switch_data->dc_detect_client);

	kfree(switch_data);

	return 0;
}

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= __devexit_p(gpio_switch_remove),
	.driver		= {
		.name	= "switch-gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init gpio_switch_init(void)
{
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

module_init(gpio_switch_init);
module_exit(gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
