/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/adc.h>
#include <linux/ctype.h>

#include <asm/gpio.h>
#include <mach/board.h>
#include <plat/key.h>

#define EMPTY_ADVALUE					950
#if defined(CONFIG_LIDA_MACH_X5) || defined(CONFIG_LIDA_MACH_X7II)
#define DRIFT_ADVALUE					20
#else
#define DRIFT_ADVALUE					70
#endif
#define INVALID_ADVALUE 				-1
#define EV_MENU					KEY_F1


#if 1
#define key_dbg(bdata, format, arg...)		\
	dev_printk(KERN_INFO , &bdata->input->dev , format , ## arg)
#else
#define key_dbg(bdata, format, arg...)	
#endif

int adc1_level;

struct rk29_button_data {
	int state;
	int long_press_count;
	struct rk29_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
        struct rk29_keys_drvdata *ddata;
};

struct rk29_keys_drvdata {
	int nbuttons;
	int result;
	bool in_suspend;	/* Flag to indicate if we're suspending/resuming */
	struct input_dev *input;
	struct adc_client *client;
	struct timer_list timer;
	struct rk29_button_data data[0];

        int ctrl;
};

int ctrl_value = 0;
static int lock_key_enable = 0;
static int lock_key_status = 0;
extern bool uac_enable_flag();

static struct input_dev *input_dev;
struct rk29_keys_Arrary {
	char keyArrary[20];
};

static ssize_t rk29key_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct rk29_keys_platform_data *pdata = dev_get_platdata(dev);
	int i,j,start,end;
	char rk29keyArrary[400];
	struct rk29_keys_Arrary Arrary[]={
                {
                        .keyArrary = {"menu"},
                },
                {
                        .keyArrary = {"home"},
                },
                {
                        .keyArrary = {"esc"},
                },
                {
                        .keyArrary = {"sensor"},
                },
                {
                        .keyArrary = {"play"},
                },
                {
                        .keyArrary = {"vol+"},
                },
                {
                        .keyArrary = {"vol-"},
                },
                {
                        .keyArrary = {"next"},
                },
                {
                        .keyArrary = {"pause"},
                },
                {
                        .keyArrary = {"pre"},
                },
        }; 
	char *p;
	  
	for(i=0;i<10;i++)
	{
		
		p = strstr(buf,Arrary[i].keyArrary);
		if(p==0)
              {
                   dev_dbg(dev," rk29key_set p == 0 error ...............\n");
                   continue;
              }
		start = strcspn(p,":");
		
		if(i<8)
			end = strcspn(p,",");
		else
			end = strcspn(p,"}");
	
		memset(rk29keyArrary,0,sizeof(rk29keyArrary));
		
		strncpy(rk29keyArrary,p+start+1,end-start-1);
							 		
		for(j=0;j<10;j++)
		{		
			if(strcmp(pdata->buttons[j].desc,Arrary[i].keyArrary)==0)
			{
				if(strcmp(rk29keyArrary,"MENU")==0)
					pdata->buttons[j].code = EV_MENU;
				else if(strcmp(rk29keyArrary,"HOME")==0)
					pdata->buttons[j].code = KEY_HOME;
				else if(strcmp(rk29keyArrary,"ESC")==0)
					pdata->buttons[j].code = KEY_BACK;
				else if(strcmp(rk29keyArrary,"sensor")==0)
					pdata->buttons[j].code = KEY_CAMERA;
				else if(strcmp(rk29keyArrary,"PLAY")==0)
					pdata->buttons[j].code = KEY_POWER;
				else if(strcmp(rk29keyArrary,"VOLUP")==0)
					pdata->buttons[j].code = KEY_VOLUMEUP;
				else if(strcmp(rk29keyArrary,"VOLDOWN")==0)
					pdata->buttons[j].code = KEY_VOLUMEDOWN;
				else if(strcmp(rk29keyArrary,"NEXT")==0)
					pdata->buttons[j].code = KEY_NEXTSONG;
				else if(strcmp(rk29keyArrary,"PAUSE")==0)
					pdata->buttons[j].code = KEY_PLAYPAUSE;
				else if(strcmp(rk29keyArrary,"PRE")==0)
					pdata->buttons[j].code = KEY_PREVIOUSSONG;
				else
				     continue;
		 	}

		}
			
   	}

	for(i=0;i<10;i++)
		dev_dbg(dev, "desc=%s, code=%d\n",pdata->buttons[i].desc,pdata->buttons[i].code);
	return 0; 

}

static DEVICE_ATTR(rk29key,0660, NULL, rk29key_set);

//add by lipf for line-control
//播放/暂停
void rk29_send_pause_key(int state)
{
    //printk("lipf_debug: enter rk29_send_pause_key  !!!\n");

    if (!input_dev)
        return;
    if(state)
    {
        input_report_key(input_dev, KEY_PLAYPAUSE, 1);
	input_sync(input_dev);
    }
    else
    {
        input_report_key(input_dev, KEY_PLAYPAUSE, 0);
        input_sync(input_dev);
    }
}

//上一首
void rk29_send_prev_key(int state)
{
    //printk("lipf_debug: enter rk29_send_prev_key  !!!\n");

    if (!input_dev)
        return;
    if(state)
    {
        input_report_key(input_dev, KEY_PREVIOUSSONG, 1);
	input_sync(input_dev);
    }
    else
    {
        input_report_key(input_dev, KEY_PREVIOUSSONG, 0);
        input_sync(input_dev);
    }
}

//下一首
void rk29_send_next_key(int state)
{
    //printk("lipf_debug: enter rk29_send_next_key  !!!\n");

    if (!input_dev)
        return;
    if(state)
    {
        input_report_key(input_dev, KEY_NEXTSONG, 1);
	input_sync(input_dev);
    }
    else
    {
        input_report_key(input_dev, KEY_NEXTSONG, 0);
        input_sync(input_dev);
    }
}
//add end

//音量+
void rk29_send_volumeup_key(int state)
{
    //printk("lipf_debug: enter rk29_send_next_key  !!!\n");

    if (!input_dev)
        return;

#if defined(CONFIG_LIDA_MACH_X5) || defined(CONFIG_LIDA_MACH_X7II)
    if (!lock_key_enable && (lock_key_status==2 || lock_key_status==3
        || lock_key_status==6 || lock_key_status==7))
        return;
#endif

    if(state)
    {
        input_report_key(input_dev, KEY_VOLUMEUP, 1);
	input_sync(input_dev);
    }
    else
    {
        input_report_key(input_dev, KEY_VOLUMEUP, 0);
        input_sync(input_dev);
    }
}
//add end

//音量-
void rk29_send_volumedown_key(int state)
{
    //printk("lipf_debug: enter rk29_send_next_key  !!!\n");

    if (!input_dev)
        return;

#if defined(CONFIG_LIDA_MACH_X5) || defined(CONFIG_LIDA_MACH_X7II)
    if (!lock_key_enable && (lock_key_status==2 || lock_key_status==3
        || lock_key_status==6 || lock_key_status==7))
        return;
#endif

    if(state)
    {
        input_report_key(input_dev, KEY_VOLUMEDOWN, 1);
	input_sync(input_dev);
    }
    else
    {
        input_report_key(input_dev, KEY_VOLUMEDOWN, 0);
        input_sync(input_dev);
    }
}
//add end

void rk29_send_power_key(int state)
{
	if (!input_dev)
		return;
	if(state)
	{
		input_report_key(input_dev, KEY_POWER, 1);
		input_sync(input_dev);
	}
	else
	{
		input_report_key(input_dev, KEY_POWER, 0);
		input_sync(input_dev);
	}
}

void rk28_send_wakeup_key(void)
{
	if (!input_dev)
		return;

	input_report_key(input_dev, KEY_WAKEUP, 1);
	input_sync(input_dev);
	input_report_key(input_dev, KEY_WAKEUP, 0);
	input_sync(input_dev);
}

static void keys_long_press_timer(unsigned long _data)
{
	int state;
	struct rk29_button_data *bdata = (struct rk29_button_data *)_data;
	struct rk29_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = EV_KEY;
	if(button->gpio != INVALID_GPIO )
		state = !!((gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low);
	else
		state = !!button->adc_state;
	if(state) {
		if(bdata->long_press_count != 0) {
			if(bdata->long_press_count % (LONG_PRESS_COUNT+ONE_SEC_COUNT) == 0){
				key_dbg(bdata, "%skey[%s]: report ev[%d] state[0]\n", 
					(button->gpio == INVALID_GPIO)?"ad":"io", button->desc, button->code_long_press);
				input_event(input, type, button->code_long_press, 0);
				input_sync(input);
			}
			else if(bdata->long_press_count%LONG_PRESS_COUNT == 0) {
				key_dbg(bdata, "%skey[%s]: report ev[%d] state[1]\n", 
					(button->gpio == INVALID_GPIO)?"ad":"io", button->desc, button->code_long_press);
				input_event(input, type, button->code_long_press, 1);
				input_sync(input);
			}
		}
		bdata->long_press_count++;
		mod_timer(&bdata->timer,
				jiffies + msecs_to_jiffies(DEFAULT_DEBOUNCE_INTERVAL));
	}
	else {
		if(bdata->long_press_count <= LONG_PRESS_COUNT) {
			bdata->long_press_count = 0;
			key_dbg(bdata, "%skey[%s]: report ev[%d] state[1], report ev[%d] state[0]\n", 
					(button->gpio == INVALID_GPIO)?"ad":"io", button->desc, button->code, button->code);
			input_event(input, type, button->code, 1);
			input_sync(input);
			input_event(input, type, button->code, 0);
			input_sync(input);
		}
		else if(bdata->state != state) {
			key_dbg(bdata, "%skey[%s]: report ev[%d] state[0]\n", 
			(button->gpio == INVALID_GPIO)?"ad":"io", button->desc, button->code_long_press);
			input_event(input, type, button->code_long_press, 0);
			input_sync(input);
		}
	}
	bdata->state = state;
}

static bool key_ctrl_now = false;
void setKeyCtrl(struct rk29_keys_button *button)
{
        if(strcmp(button->desc, "play")==0) {
            if (key_ctrl_now)
	            button->code = KEY_PLAYPAUSE;
            else
                    button->code = KEY_POWER;
        }
	else if(strcmp(button->desc,"vol+")==0) {
            if (key_ctrl_now)
	            button->code = KEY_PREVIOUSSONG;
            else
		    button->code = KEY_VOLUMEUP;
        }
	else if(strcmp(button->desc,"vol-")==0) {
            if (key_ctrl_now)
	            button->code = KEY_NEXTSONG;
            else
		    button->code = KEY_VOLUMEDOWN;
        }
	else if(strcmp(button->desc,"next")==0) {
            if (key_ctrl_now)
	            button->code = KEY_VOLUMEDOWN;
            else
	            button->code = KEY_NEXTSONG;
        }
	else if(strcmp(button->desc,"pause")==0) {
            if (key_ctrl_now)
		    button->code = KEY_POWER;
            else
		    button->code = KEY_PLAYPAUSE;
        }
	else if(strcmp(button->desc,"pre")==0) {
            if (key_ctrl_now)
	            button->code = KEY_VOLUMEUP;
            else
		    button->code = KEY_PREVIOUSSONG;
        }

}

static bool isLockKeyDisable(struct rk29_button_data *data)
{
        struct rk29_button_data *bdata = data;
        struct rk29_keys_button *button = bdata->button;
        int status;
        bool disable;

        if (lock_key_enable == 1)
            return false;
   
        status = lock_key_status;
        switch (status){
                case 0:
                    disable = false;
                    break;
                case 1:
                    if (button->code == KEY_PREVIOUSSONG || button->code == KEY_NEXTSONG)
                        disable = true;
                    else
                        disable = false;
                    break;
                case 2:
                    if (button->code == KEY_VOLUMEUP || button->code == KEY_VOLUMEDOWN)
                        disable = true;
                    else
                        disable = false;
                    break;
                case 3:
                    if (button->code == KEY_VOLUMEUP || button->code == KEY_VOLUMEDOWN
                        || button->code == KEY_PREVIOUSSONG || button->code == KEY_NEXTSONG)
                        disable = true;
                    else
                        disable = false;
                    break;
                case 4:
                    if (button->code == KEY_PLAYPAUSE)
                        disable = true;
                    else
                        disable = false;
                    break;
                case 5:
                    if (button->code == KEY_PREVIOUSSONG || button->code == KEY_NEXTSONG
                        || button->code == KEY_PLAYPAUSE)
                        disable = true;
                    else
                        disable = false;
                    break;
                case 6:
                    if (button->code == KEY_VOLUMEUP || button->code == KEY_VOLUMEDOWN
                        || button->code == KEY_PLAYPAUSE)
                        disable = true;
                    else
                        disable = false;
                    break;
                case 7:
                    if (button->code == KEY_PREVIOUSSONG || button->code == KEY_NEXTSONG
                        || button->code == KEY_VOLUMEUP || button->code == KEY_VOLUMEDOWN
                        || button->code == KEY_PLAYPAUSE)
                        disable = true;
                    else
                        disable = false;
                    break; 

                default:
                    disable = false;
                    break;
        }

        //key_dbg(bdata, "%skey[%s]: report ev[%d] state[%d]\n", 
	//	(button->gpio == INVALID_GPIO)?"ad":"io", button->desc, button->code, bdata->state);
        //key_dbg(bdata, "%s: status == %d, disable == %d\n", __func__, status, disable);

        return disable;
}

static void keys_timer(unsigned long _data)
{
	int state;
	struct rk29_button_data *bdata = (struct rk29_button_data *)_data;
	struct rk29_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = EV_KEY;
        struct rk29_keys_drvdata *ddata = bdata->ddata;
	
	if(button->gpio != INVALID_GPIO)
		state = !!((gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low);
	else
		state = !!button->adc_state;
	if(bdata->state != state) {
		bdata->state = state;
		key_dbg(bdata, "%skey[%s]: report ev[%d] state[%d]\n", 
			(button->gpio == INVALID_GPIO)?"ad":"io", button->desc, button->code, bdata->state);
                //key_dbg(bdata, " ctrl_value  is   %d \n", ctrl_value);
                if (ctrl_value)
                    key_ctrl_now = true;
                else
                    key_ctrl_now = false;
                setKeyCtrl(button);
     
                //disable pre_song, next_song and play_pause, when enter usb audio class mode !
#if defined(CONFIG_LIDA_MACH_X5) || defined(CONFIG_LIDA_MACH_X7II)
                if (!isLockKeyDisable(bdata) && !(uac_enable_flag() && (button->code == KEY_PREVIOUSSONG
                        || button->code == KEY_NEXTSONG || button->code == KEY_PLAYPAUSE))) 
#else                 
                if (!(uac_enable_flag() && (button->code == KEY_PREVIOUSSONG
                        || button->code == KEY_NEXTSONG || button->code == KEY_PLAYPAUSE))) 
#endif         
                {
		    input_event(input, type, button->code, bdata->state);
		    input_sync(input);
                    //key_dbg(bdata, "%s: uac_enable_flag == %d\n", __func__, uac_enable_flag());
                }
	}
	if(state)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(DEFAULT_DEBOUNCE_INTERVAL));
}

static irqreturn_t keys_isr(int irq, void *dev_id)
{
	struct rk29_button_data *bdata = dev_id;
	struct rk29_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = EV_KEY;
	BUG_ON(irq != gpio_to_irq(button->gpio));

        if(button->wakeup == 1 && bdata->ddata->in_suspend == true){
		bdata->state = 1;
		key_dbg(bdata, "wakeup: %skey[%s]: report ev[%d] state[%d]\n", 
			(button->gpio == INVALID_GPIO)?"ad":"io", button->desc, button->code, bdata->state);
                if (button->code == KEY_PLAYPAUSE)
                {
                    rk28_send_wakeup_key();
                }
		input_event(input, type, button->code, bdata->state);
		input_sync(input);
        }
	bdata->long_press_count = 0;
	mod_timer(&bdata->timer,
				jiffies + msecs_to_jiffies(DEFAULT_DEBOUNCE_INTERVAL));
	return IRQ_HANDLED;
}

static void keys_adc_callback(struct adc_client *client, void *client_param, int result)
{
	struct rk29_keys_drvdata *ddata = (struct rk29_keys_drvdata *)client_param;
	int i;
	adc1_level = result;
	
	if(result > INVALID_ADVALUE && result < EMPTY_ADVALUE)
		ddata->result = result;
	for (i = 0; i < ddata->nbuttons; i++) {
		struct rk29_button_data *bdata = &ddata->data[i];
		struct rk29_keys_button *button = bdata->button;
		if(!button->adc_value)
			continue;
		if(result < button->adc_value + DRIFT_ADVALUE &&
			result > button->adc_value - DRIFT_ADVALUE) {
			button->adc_state = 1;
                        key_dbg(bdata, "%s: button->code ==  %d\n", __func__, button->code);
		} else
			button->adc_state = 0;
		if(bdata->state != button->adc_state)
			mod_timer(&bdata->timer,
				jiffies + msecs_to_jiffies(DEFAULT_DEBOUNCE_INTERVAL));
	}
	return;
}

static void keys_adc_timer(unsigned long _data)
{
	struct rk29_keys_drvdata *ddata = (struct rk29_keys_drvdata *)_data;

	if (!ddata->in_suspend)
		adc_async_read(ddata->client);
	mod_timer(&ddata->timer, jiffies + msecs_to_jiffies(ADC_SAMPLE_TIME));
}

static ssize_t adc_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rk29_keys_drvdata *ddata = dev_get_drvdata(dev);
	
	return sprintf(buf, "adc_value: %d\n", ddata->result);
}

static ssize_t key_ctrl_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
        struct rk29_keys_drvdata *ddata = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ddata->ctrl);
}

static ssize_t key_ctrl_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
        struct rk29_keys_drvdata *ddata = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long res = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
        
	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
                ddata->ctrl = res;
                ctrl_value = res;
	}

	return ret;
}

static ssize_t lock_key_enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", lock_key_enable);
}

static ssize_t lock_key_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long res = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
        
	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;

		if (!uac_enable_flag())
                	lock_key_enable = res;
	}

	return ret;
}

static ssize_t lock_key_status_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", lock_key_status);
}

static ssize_t lock_key_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long res = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
        
	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
                lock_key_status = res;
	}

	return ret;
}

static DEVICE_ATTR(get_adc_value, S_IRUGO | S_IWUSR, adc_value_show, NULL);
static DEVICE_ATTR(key_ctrl, 0664, key_ctrl_show, key_ctrl_store);
static DEVICE_ATTR(lock_key_enable, 0664, lock_key_enable_show, lock_key_enable_store);
static DEVICE_ATTR(lock_key_status, 0664, lock_key_status_show, lock_key_status_store);

static int __devinit keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk29_keys_platform_data *pdata = dev_get_platdata(dev);
	struct rk29_keys_drvdata *ddata;
	struct input_dev *input;
	int i, error = 0;
	int wakeup = 0;

	if(!pdata) 
		return -EINVAL;
	
	ddata = kzalloc(sizeof(struct rk29_keys_drvdata) +
			pdata->nbuttons * sizeof(struct rk29_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		error = -ENOMEM;
		goto fail0;
	}

	platform_set_drvdata(pdev, ddata);

	input->name = pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);
	ddata->nbuttons = pdata->nbuttons;
	ddata->input = input;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct rk29_keys_button *button = &pdata->buttons[i];
		struct rk29_button_data *bdata = &ddata->data[i];

		bdata->input = input;
		bdata->button = button;
                bdata->ddata = ddata;

		if (button->code_long_press)
			setup_timer(&bdata->timer,
			    	keys_long_press_timer, (unsigned long)bdata);
		else if (button->code)
			setup_timer(&bdata->timer,
			    	keys_timer, (unsigned long)bdata);

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, EV_KEY, button->code);
	};

	if (pdata->chn >= 0) {
		setup_timer(&ddata->timer, keys_adc_timer, (unsigned long)ddata);
		ddata->client = adc_register(pdata->chn, keys_adc_callback, (void *)ddata);
		if (!ddata->client) {
			error = -EINVAL;
			goto fail1;
		}
		mod_timer(&ddata->timer, jiffies + msecs_to_jiffies(100));
	}

	for (i = 0; i < pdata->nbuttons; i++) {
		struct rk29_keys_button *button = &pdata->buttons[i];
		struct rk29_button_data *bdata = &ddata->data[i];
		int irq;

		if(button->gpio != INVALID_GPIO) {
			error = gpio_request(button->gpio, button->desc ?: "keys");
			if (error < 0) {
				pr_err("gpio-keys: failed to request GPIO %d,"
					" error %d\n", button->gpio, error);
				goto fail2;
			}

			error = gpio_direction_input(button->gpio);
			if (error < 0) {
				pr_err("gpio-keys: failed to configure input"
					" direction for GPIO %d, error %d\n",
					button->gpio, error);
				gpio_free(button->gpio);
				goto fail2;
			}

			irq = gpio_to_irq(button->gpio);
			if (irq < 0) {
				error = irq;
				pr_err("gpio-keys: Unable to get irq number"
					" for GPIO %d, error %d\n",
					button->gpio, error);
				gpio_free(button->gpio);
				goto fail2;
			}

			error = request_irq(irq, keys_isr,
					    (button->active_low)?IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING,
					    button->desc ? button->desc : "keys",
					    bdata);
			if (error) {
				pr_err("gpio-keys: Unable to claim irq %d; error %d\n",
					irq, error);
				gpio_free(button->gpio);
				goto fail2;
			}
		}
	}

	input_set_capability(input, EV_KEY, KEY_WAKEUP);

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, "
			"error: %d\n", error);
		goto fail2;
	}

	device_init_wakeup(dev, wakeup);
	error = device_create_file(dev, &dev_attr_get_adc_value);
	error = device_create_file(dev, &dev_attr_key_ctrl);
	error = device_create_file(dev, &dev_attr_lock_key_enable);
	error = device_create_file(dev, &dev_attr_lock_key_status);
	error = device_create_file(dev, &dev_attr_rk29key);
	if(error )
	{
		pr_err("failed to create key file error: %d\n", error);
	}


	input_dev = input;
	return error;

 fail2:
	while (--i >= 0) {
		free_irq(gpio_to_irq(pdata->buttons[i].gpio), &ddata->data[i]);
		del_timer_sync(&ddata->data[i].timer);
		gpio_free(pdata->buttons[i].gpio);
	}
	if(pdata->chn >= 0 && ddata->client);
		adc_unregister(ddata->client);
	if(pdata->chn >= 0)
	        del_timer_sync(&ddata->timer);
 fail1:
 	platform_set_drvdata(pdev, NULL);
 fail0:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit keys_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk29_keys_platform_data *pdata = dev_get_platdata(dev);
	struct rk29_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i;

	input_dev = NULL;
	device_init_wakeup(dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, &ddata->data[i]);
		del_timer_sync(&ddata->data[i].timer);
		gpio_free(pdata->buttons[i].gpio);
	}
	if(pdata->chn >= 0 && ddata->client);
		adc_unregister(ddata->client);
	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int keys_suspend(struct device *dev)
{
	struct rk29_keys_platform_data *pdata = dev_get_platdata(dev);
	struct rk29_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	ddata->in_suspend = true;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct rk29_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = gpio_to_irq(button->gpio);
				enable_irq_wake(irq);
			}
		}
	}

	return 0;
}

static int keys_resume(struct device *dev)
{
	struct rk29_keys_platform_data *pdata = dev_get_platdata(dev);
	struct rk29_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct rk29_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = gpio_to_irq(button->gpio);
				disable_irq_wake(irq);
			}
		}
		preempt_disable();
		if (local_softirq_pending())
			do_softirq(); // for call resend_irqs, which may call keys_isr
		preempt_enable_no_resched();
	}

	ddata->in_suspend = false;

	return 0;
}

static const struct dev_pm_ops keys_pm_ops = {
	.suspend	= keys_suspend,
	.resume		= keys_resume,
};
#endif

static struct platform_driver keys_device_driver = {
	.probe		= keys_probe,
	.remove		= __devexit_p(keys_remove),
	.driver		= {
		.name	= "rk29-keypad",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &keys_pm_ops,
#endif
	}
};

static int __init keys_init(void)
{
	return platform_driver_register(&keys_device_driver);
}

static void __exit keys_exit(void)
{
	platform_driver_unregister(&keys_device_driver);
}

module_init(keys_init);
module_exit(keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys");
