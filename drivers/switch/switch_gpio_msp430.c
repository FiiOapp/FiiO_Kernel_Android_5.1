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
#include<linux/i2c.h>
#include <linux/switch_gpio_msp430.h>
#include <linux/regulator/machine.h>
#include<linux/err.h>
#include <mach/board.h>

#if 0
#define DBG(fmt, args...)	printk("*** " fmt, ## args)
#else
#define DBG(fmt, args...)	do{}while(0)
#endif

#if 1
#define DBGP(fmt, args...)	printk("MSP430 " fmt, ## args)
#else
#define DBGP(fmt, args...)	do{}while(0)
#endif

#if defined(CONFIG_LIDA_MACH_X5) && defined(CONFIG_SND_RK29_SOC_AK4490)
extern void ak4490_set_am_power(int on);
#endif

#define  MSP430_SPEED    100 * 1000

#define  ADC2_LEVEL      300//400//200
#define  ADC2_DK5_LEVEL  540
#define  ADC0_LEVEL_IN   20

#define ENABLE_AUTO_UPGRADE 1

#define SC_INFO_ERASED              5
#define SC_INFO_STORED              6

enum SYS_STATUS {
	SYS_STATUS_SUSPEND = 0,	// 0x01
	SYS_STATUS_VIDEO,	// 0x02
	SYS_STATUS_VIDEO_720P,       // 0x04
	SYS_STATUS_VIDEO_1080P,       // 0x08
	SYS_STATUS_GPU,		// 0x10
	SYS_STATUS_RGA,		// 0x20
	SYS_STATUS_CIF0,	// 0x40
	SYS_STATUS_CIF1,	// 0x80
	SYS_STATUS_REBOOT,	// 0x100
	SYS_STATUS_LCDC0,	// 0x200
	SYS_STATUS_LCDC1,	// 0x400
	SYS_STATUS_WIFIDISPLAY,
	SYS_STATUS_AUDIO,
	SYS_STATUS_USB,
	SYS_STATUS_MAX,
};

extern int cpufreq_set_policy(char gov,int min,int max);
extern noinline void ddrfreq_set_sys_status(enum SYS_STATUS status);
extern noinline void ddrfreq_clear_sys_status(enum SYS_STATUS status);
static struct workqueue_struct *mcu_wq;
extern int board_boot_mode(void);
static DEFINE_SPINLOCK(update_fw_lock);
static int fw_update_flag=0;
enum fw_flag{
    FW_UNKNOW,
    FW_UPDATING,
    FW_UPDATE_FAILED,
    FW_UPDATE_SUCCESS,
    FW_NEWEST,
};
enum fw_info{
    FW_INFO_UNKNOW,
    FW_INFO_ERASED,
    FW_INFO_STORED,
};
static int fw_ok=FW_UNKNOW;
module_param_named(FW_FLAG, fw_ok, int, 0644);

static char fw_info[32];
module_param_string(FW_INFO_FLAG, fw_info,32, 0644);

extern int Read_info(void);
extern void ak4490_set_lo_mute(int mute);
struct gpio_switch_data*  gswitch=NULL;
static char buf[8] = {0};
static int last_state;
static irqreturn_t gpio_irq_handler(int irq, void *dev_id);
static int key_irq;

static int mcu_start();
int link_status_update(struct gpio_switch_data* switch_data);
int ti_sbw_hardware_init();
void ti_sbw_reset_mcu(void);
int updateFirmware(void);
char get_mcu_ver(void);

static bool dock_status = false;

extern void jackline_power(int on);

static void msp430_power(int on)
{
  struct  regulator* ldo6;
    ldo6=regulator_get(NULL,"act_ldo6");
    if(ldo6 == NULL || IS_ERR(ldo6)){

        printk("act ldo6 is not available \n");
    }else{

        if(on){
            regulator_set_voltage(ldo6,3300000,3300000);
            regulator_enable(ldo6);
            regulator_put(ldo6);
        }else{

            regulator_disable(ldo6);
            regulator_put(ldo6);
        }

    }
    
}

bool dock_is_intert(void)
{
        return dock_status;
}
EXPORT_SYMBOL_GPL(dock_is_intert);

static void rk3188_adc0_callback(struct adc_client *client, void *client_param, int result)
{
        int level = result;
        struct gpio_switch_data *switch_data = (struct gpio_switch_data *)client_param;

        DBG("lipf_debug: %s enter!\n", __FUNCTION__);
        DBG("%s: read adc value: %d\n", __func__, level);
 
        /* read adc0 value, judge usb dock in or out */
        switch_data->adc0_value = level;
}

static void rk3188_adc0_timer(unsigned long _data)
{
	struct gpio_switch_data *switch_data = (struct gpio_switch_data *)_data;

        DBG("%s enter!\n", __FUNCTION__);

    adc_async_read(switch_data->adc0_client);
	mod_timer(&switch_data->adc0_timer, jiffies + msecs_to_jiffies(200));
}

static void rk3188_adc2_callback(struct adc_client *client, void *client_param, int result)
{
        int level = result;
        struct gpio_switch_data *switch_data = (struct gpio_switch_data *)client_param;

        /* DBG("lipf_debug: %s enter!\n", __FUNCTION__); */
        /* DBG("%s: read adc2 value: %d\n", __func__, level); */
        /* DBG("%s: read adc0 value: %d\n", __func__, switch_data->adc0_value); */
 
        /* read adc2 value, judge usb dock in or out */
	if(level < 0)
	{
		printk("%s:get adc level err = %d!\n",__FUNCTION__, level);
		return;
	}

        if (switch_data->adc0_value < ADC0_LEVEL_IN) {
                if(level >=0 && level < ADC2_LEVEL) {
                    //DK1 wuyuan is insert, host-power-en ?
                }

                if(level >=0 && level < ADC2_LEVEL || level > ADC2_DK5_LEVEL) {
                        
                        if(switch_data->accessory_dev.state != 1) {
                                switch_data->dock_count++;

                                if (switch_data->dock_count > 5) {
                                        switch_set_state(&switch_data->accessory_dev, 1);
                                        switch_data->dock_count = 5;
                                        link_set_status(LINK_DOCK_ENABLE);
                                        dock_status = true;
                                }
                        }
                }
                switch_data->dock_out_count = 0;
        } else {
                //usb dock is out
                if(switch_data->accessory_dev.state != 0) {
                    switch_data->dock_out_count++;

                    if (switch_data->dock_out_count > 5) {
                        switch_set_state(&switch_data->accessory_dev, 0);
                        switch_data->dock_out_count = 5;
                        link_set_status(LINK_DOCK_DISABLE);
                        dock_status = false;
                    }
                }
                switch_data->dock_count = 0;
        }     
}

static void rk3188_adc2_timer(unsigned long _data)
{
	struct gpio_switch_data *switch_data = (struct gpio_switch_data *)_data;

        DBG("%s enter!\n", __FUNCTION__);

        /* link_status_update(gswitch); */
	adc_async_read(switch_data->adc2_client);
	mod_timer(&switch_data->adc2_timer, jiffies + msecs_to_jiffies(200));
}

static int msp430_recv_data(struct i2c_client *client, char *rxData, int length)
{
	int ret = -1;
	//ret = i2c_master_reg8_recv(client, reg, rxData, length, MSP430_SPEED);

    /* if(spin_trylock(&update_fw_lock)){ */
    if(!fw_update_flag){
        ret = i2c_master_recv(client, rxData, length);
        /* spin_unlock(&update_fw_lock); */
    }else
        printk("msp430 get lock failed receive\n");


	return (ret > 0)? 0 : ret;
}

static int msp430_send_data(struct i2c_client *client, char *txData, int length)
{
    int ret = -1;
    char reg = txData[0];
    /* if(spin_trylock(&update_fw_lock)){ */
        /* ret = i2c_master_reg8_send(client, reg, &txData[1], length-1, MSP430_SPEED); */
        printk(KERN_WARNING "%s  length %d \n",__func__,length);
        if(!fw_update_flag){
        ret=i2c_master_send(client , txData, length);
        }else
            printk("msp430 get lock failed send\n");
        /* spin_unlock(&update_fw_lock); */
    
	return (ret > 0)? 0 : ret;
}

static int msp430_get_data(struct i2c_client *client)
{
        //char buf[4] = {0};
        int ret;
        int count = 5;
        /* return -1; */

        do {
            //memset(buf, 0, 4);
            ret = msp430_recv_data(client, buf, 8);
            if (ret < 0) {
                printk("%s:  recv data error !!!\n", __func__);
                //return ret;
            }
            count--;
        } while(count > 0);

        printk("%s:  buf[0-3]  is  0x%x  0x%x  0x%x  0x%x \n", __func__, buf[0], buf[1], buf[2], buf[3]);
        printk("%s:  buf[4-7]  is  0x%x  0x%x  0x%x  0x%x \n", __func__, buf[4], buf[5], buf[6], buf[7]);

        return ret;
}

extern void rk29_send_pause_key(int);
extern void rk29_send_prev_key(int);
extern void rk29_send_next_key(int);
extern void rk29_send_volumeup_key(int);
extern void rk29_send_volumedown_key(int);

static int set_line_control(void)
{
       char val;
       static bool vol_up_press_flags;
       static bool vol_down_press_flags;

       val = buf[4] & LINE_VOL_UP;
       if (val == LINE_VOL_UP) {
           rk29_send_volumeup_key(1);
           rk29_send_volumeup_key(0);
           goto ctl_ok;
       }

       val = buf[4] & LINE_VOL_DOWN;
       if (val == LINE_VOL_DOWN) {
           rk29_send_volumedown_key(1);
           rk29_send_volumedown_key(0);
           goto ctl_ok;
       }

       val = buf[4] & LINE_VOL_PAUSE;
       if (val == LINE_VOL_PAUSE) {
           rk29_send_pause_key(1);
           rk29_send_pause_key(0);
           goto ctl_ok;
       }

       val = buf[4] & LINE_VOL_NEXT;
       if (val == LINE_VOL_NEXT) {
           rk29_send_next_key(1);
           rk29_send_next_key(0);
           goto ctl_ok;
       }

       val = buf[4] & LINE_VOL_PREV;
       if (val == LINE_VOL_PREV) {
           rk29_send_prev_key(1);
           rk29_send_prev_key(0);
           goto ctl_ok;
       }

       val = buf[4] & LINE_VOL_UP_PRESS;
       if (val == LINE_VOL_UP_PRESS) {
           rk29_send_volumeup_key(1);
           vol_up_press_flags = true;
       }

       val = buf[5] & LINE_VOL_DOWN_PRESS;
       if (val == LINE_VOL_DOWN_PRESS) {
           rk29_send_volumedown_key(1);
           vol_down_press_flags = true;
       }

       val = buf[5] & LINE_VOL_RELEASE;
       if (val == LINE_VOL_RELEASE) {
           if (vol_up_press_flags) {
               rk29_send_volumeup_key(0);
               vol_up_press_flags = false;
           } else if(vol_down_press_flags) {
               rk29_send_volumedown_key(0);
               vol_down_press_flags = false;
           }
           goto ctl_ok;
       }

       return 0;

ctl_ok:
       printk("%s:  ok,  val  is  0x%x \n", __func__, val);
       return 0;
}

static void set_dev_state(struct gpio_switch_data *switch_data)
{
       char val;

       val = buf[0] & LINK_PO_MASK;
       printk("%s:  val  is  0x%x\n", __func__, val);
       if (val == LINK_PO_DET) {
           if(switch_data->headset.state != 1) {
               switch_set_state(&switch_data->headset, 1);

	           schedule_work(&switch_data->power_work);
           }
       } else {
           if(switch_data->headset.state != 0) {
               switch_set_state(&switch_data->headset, 0);

               link_set_status(LINK_WIRE_CTL_DISABLE);
               jackline_power(0);

           }
       }

       val = buf[0] & LINK_LO_MASK;
       printk("%s:  val  is  0x%x\n", __func__, val);
       if (val == LINK_LO_DET) {
           if(switch_data->sdev.state != 1) {
               switch_set_state(&switch_data->sdev, 1);
           }
       } else {
           if(switch_data->sdev.state != 0) {
               switch_set_state(&switch_data->sdev, 0);
           }
       }

       val = buf[1] & LINK_BAH_MASK;
       printk("%s:  val  is  0x%x\n", __func__, val);
       if (val == LINK_BAH_DET) {
           if(switch_data->balance_dev.state != 1) {
               switch_set_state(&switch_data->balance_dev, 1);
           }
       } else {
           if(switch_data->balance_dev.state != 0) {
               switch_set_state(&switch_data->balance_dev, 0);
           }
       }

       val = buf[5] & LINK_LO_MUTE_MASK;
       printk("%s:  val  is  0x%x\n", __func__, val);
       if (val == LINK_LO_MUTE_ON) {
            ak4490_set_lo_mute(1);
       } else if(val == LINK_LO_MUTE_OFF) {
            ak4490_set_lo_mute(0);
       }else{

       }

       val = buf[1] & LINK_DC_MASK;
       printk("%s:  val  is  0x%x\n", __func__, val);
       if (val == LINK_DC_DET) {
           if(switch_data->dc_detect.state != 1) {
               switch_set_state(&switch_data->dc_detect, 1);
           }
       } else {
           if(switch_data->dc_detect.state != 0) {
               switch_set_state(&switch_data->dc_detect, 0);
           }
       }

       //add for wire control, when PO is intert!
       if(switch_data->headset.state == 1 && switch_data->line_control.status == 1)
           set_line_control();
}

static void gpio_switch_work(struct work_struct *work)
{
       struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);
       struct i2c_client *client = data->client;

       printk("%s: enter !!!\n", __func__);

       if (msp430_get_data(client) < 0){
           printk("%s:  msp430_get_data  fail !!!\n", __func__);
           return;
       }

       enable_irq(client->irq);

       set_dev_state(data);
}

static void power_work(struct work_struct *work)
{
       struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);
       struct i2c_client *client = data->client;

       printk("%s: enter !!!\n", __func__);

       msleep(1500);
       jackline_power(1);

       link_set_status(LINK_WIRE_CTL_ENABLE);
}


static void extract_link_state(struct gpio_switch_data *switch_data)
{

       switch_data->analog_power=buf[0]&0x80? 1:0;
       switch_data->mcu_working=buf[0]&0x20? 1:0;
       switch_data->first_play =buf[1]&0x20? 1:0;
       switch_data->dc_error=buf[1]&0x40? 1:0;
       switch_data->mute=buf[1]&0x08? 1:0;
       switch_data->dac_en=buf[1]&0x10? 1:0;
       switch_data->dock_en=buf[0]&0x40? 1:0;

       switch_data->mcu_firmware_version=buf[3];

       DBGP("mcuworking: %d analog_power: %d first_play %d dc_error %d  mute %d\n \tbuf: 0x%x  0x%x 0x%x 0x%x \n",
               (uint32_t) switch_data->analog_power,
               (uint32_t) switch_data->mcu_working,
               (uint32_t) switch_data->first_play ,
               (uint32_t) switch_data->dc_error,
               (uint32_t) switch_data->mute,
                buf[0],buf[1],buf[2],buf[3]);
}



int link_status_update(struct gpio_switch_data* switch_data)
{
    int ret=0;

    if(switch_data){

        if (msp430_get_data(switch_data->client) < 0){
            printk("%s:  msp430_get_data  fail !!!\n", __func__);
            ret= -EAGAIN;
        }

        extract_link_state(switch_data);

    }else{
        printk("%s:  switch_data is null !!!\n", __func__);
        ret = -ENXIO;
    }

    return ret;
}
EXPORT_SYMBOL_GPL(link_status_update);

int link_set_status(enum link_status status)
{
    int ret=-1;
    uint8_t dat[4]={0,0,0,0};

    DBGP("%s  status 0x%x \n",__func__,(uint32_t)status);



    switch(status){
    case LINK_FORCE_MCU_UPGRADE:
            fw_ok=FW_UNKNOW;
            link_stop(0);
            gswitch->mcu_force_upgrade=1;
            /* schedule_work(&gswitch->mcu_update_work); */
            queue_work(mcu_wq, &gswitch->mcu_update_work);
            return 0;

    default: ;
    }


    switch(status){
    case LINK_MCU_WORK: dat[0]|=0x80;break;

    case LINK_MCU_STOP: dat[0]|=0x10;break;

    case LINK_ANALOG_POWER: dat[0]|=0x40;break;

    case LINK_FIRST_PLAY: dat[0]|=0x40;break;

    case LINK_STOP_PLAY: dat[0]|=0x20;break;

    case LINK_DAC_POWER_ON: dat[1]|=0x08;break;

    case LINK_DAC_POWER_OFF: dat[1]|=0x04;break;

    case LINK_MUTE_ON: dat[1]|=0x02;break;
    case LINK_MUTE_OFF: dat[1]|=0x01;break;

    case LINK_DOCK_ENABLE: dat[1]|=0x20;break;
    case LINK_DOCK_DISABLE: dat[1]|=0x10;break;

    case LINK_WIRE_CTL_ENABLE: dat[1]|=0x80;break;
    case LINK_WIRE_CTL_DISABLE: dat[1]|=0x40;break;

    case LINK_AUTO_POWER_ON: dat[2]|=0x80;break;
    case LINK_AUTO_POWER_OFF:dat[2]|=0x40;break;
    case LINK_FORCE_POWER_ON:dat[2]|=0x20;break;



    default:
         printk(KERN_WARNING "link_set_status status error  %d \n",status);

    }

    if(*(uint32_t*)dat)
       ret= msp430_send_data(gswitch->client, dat,sizeof(dat));

    return ret;

}
EXPORT_SYMBOL_GPL(link_set_status);

void link_set_force_poweron()
{
    link_set_status(LINK_FORCE_POWER_ON);
#if defined(CONFIG_LIDA_MACH_X5) && defined(CONFIG_SND_RK29_SOC_AK4490)
     ak4490_set_am_power(1);
#endif
}

void link_set_force_upgrade()
{

     link_set_status(LINK_FORCE_MCU_UPGRADE);
}

bool link_get_status(enum link_status status)
{

    int ret=0,count=1;
    if((ret=link_status_update(gswitch))<0){
        printk(KERN_ERR "audio link status update Failed!! use old status\n");
        /* return ret; */
    }

    switch(status){
    case LINK_MCU_WORK: return gswitch->mcu_working;

    case LINK_ANALOG_POWER: return gswitch->analog_power;

    case LINK_FIRST_PLAY: return gswitch->first_play;

    case LINK_DAC_POWER_ON: return gswitch->dac_en;

    case LINK_MUTE_ON: return gswitch->mute;
    default:
                       printk(KERN_WARNING "link_get_status status error  %d \n",status);

    }


}
EXPORT_SYMBOL_GPL(link_get_status);

u8 mcu_get_fw_ver()
{
    link_get_status(LINK_MCU_WORK);
    return gswitch->mcu_firmware_version;
}

static int mcu_start()
{

    int ret=0,count=1;

    printk(KERN_WARNING "mcu_start \n");
    if((ret=link_status_update(gswitch))<0){
        printk(KERN_ERR "audio link status update Failed!!\n");
        return ret;
    }


    if(!gswitch->mcu_working){

        link_set_status(LINK_MCU_WORK);

        do{
            /* msleep(5); */
            count--;
            ret=link_status_update(gswitch);

        }while(!gswitch->mcu_working && count>0);

    }else
        printk(KERN_WARNING "LINK_MCU_WORK in statues,ret \n");


    if(count<0){
        printk(KERN_WARNING "LINK_MCU_WORK error,ret %d \n",ret);
        return ret==0? -ETIMEDOUT:-EAGAIN;
    }

    return ret;

}

static int mcu_stop()
{

    int ret=0,count=10;

    if((ret=link_status_update(gswitch))<0){
        printk(KERN_ERR "audio link status update Failed!!\n");
        return ret;
    }


    if(!gswitch->mcu_working){

        link_set_status(LINK_MCU_STOP);

        do{
            msleep(5);
            count--;
            ret=link_status_update(gswitch);

        }while(!gswitch->mcu_working && count>0);

    }
    if(count<0){
        printk(KERN_WARNING "LINK_MCU_STOP error,ret %d \n",ret);
        return ret==0? -ETIMEDOUT:-EAGAIN;
    }

}

int link_start()
{

    int ret=0,count=10;

    if((ret=link_status_update(gswitch))<0){
        printk(KERN_ERR "audio link status update Failed!!\n");
        return ret;
    }


    if(!gswitch->mcu_working){

        link_set_status(LINK_MCU_WORK);

        do{
            msleep(5);
            count--;
            ret=link_status_update(gswitch);

        }while(!gswitch->mcu_working && count>0);

    }
    if(count<=0){
        printk(KERN_WARNING "LINK_MCU_WORK error,ret %d \n",ret);
        return ret==0? -ETIMEDOUT:-EAGAIN;
    }




#if 0 // digital mute will handle this
    count=10;
    if(gswitch->mute){

        link_set_status(LINK_MUTE_OFF);

        do{
            msleep(100);
            count--;
            ret=link_status_update(gswitch);

        }while(gswitch->mute && count>0);

    }
    if(count<=0){
        printk(KERN_WARNING "LINK_MUTE_OFF error,ret %d \n",ret);
        return ret==0? -ETIMEDOUT:-EAGAIN;
    }


#endif
    // 10*200=2000ms
        DBGP("%s %d \n",__func__,__LINE__);
    count=10;
    if(!gswitch->first_play){
        DBGP("%s %d \n",__func__,__LINE__);
        link_set_status(LINK_FIRST_PLAY);
        do{
            msleep(200);//200ms at least
            count--;
            ret=link_status_update(gswitch);

        }while(!gswitch->first_play && count>0);
    }
    if(count<=0){
        if(gswitch->first_play==0 && gswitch->dc_error==1){
            printk(KERN_ERR"\n**************************\n");
            printk(KERN_ERR "LINK status found DC error\n");
            printk(KERN_ERR"**************************\n\n");
        }

        printk(KERN_WARNING "LINK_FIRST_PLAY error,ret %d \n",ret);
        return ret==0? -ETIMEDOUT:-EAGAIN;
    }

    DBGP("%s %d success\n",__func__,__LINE__);
    return ret;
}
EXPORT_SYMBOL_GPL(link_start);

int link_stop(int tosleep)
{

    int ret=0,count=10;

    if((ret=link_status_update(gswitch))<0){
        printk(KERN_ERR "audio link status update Failed!!\n");
        return ret;
    }


    if(tosleep){
        if(gswitch->first_play){

            link_set_status(LINK_MCU_STOP);

            do{
                msleep(200);
                count--;
                ret=link_status_update(gswitch);

            }while(gswitch->mcu_working && count>0);
        }
        if(count<0){
            printk(KERN_WARNING "LINK_MCU_STOP error,ret %d \n",ret);
            return ret==0? -ETIMEDOUT:-EAGAIN;
        }
    }else{
        if(gswitch->first_play){

            link_set_status(LINK_STOP_PLAY);

            do{
                msleep(200);
                count--;
                ret=link_status_update(gswitch);

            }while(gswitch->first_play&& count>0);
        }
        if(count<0){
            printk(KERN_WARNING "LINK_STOP_PLAY error,ret %d \n",ret);
            return ret==0? -ETIMEDOUT:-EAGAIN;
        }

    }

    return ret;

}
EXPORT_SYMBOL_GPL(link_stop);

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;

        disable_irq_nosync(irq);
	schedule_work(&switch_data->work);

        printk("Headset interrupt \n");
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

static void mcu_work(struct work_struct *work)
{
    struct gpio_switch_data	*data =
        container_of(work, struct gpio_switch_data, work);
    int irq_flag,ret,count=10,K;
    int info;

    DBGP("%s: enter !!!\n", __func__);

    cpufreq_set_policy(1,1008000,1008000);
    ddrfreq_set_sys_status(SYS_STATUS_AUDIO);
    spin_lock(&update_fw_lock);
    info=Read_info();

    spin_unlock(&update_fw_lock);
    ddrfreq_clear_sys_status(SYS_STATUS_AUDIO);
    cpufreq_set_policy(0,0,1500000);
    if(info == SC_INFO_ERASED)
        memcpy(fw_info,"info erased",12);
    else if(info == SC_INFO_STORED)
        memcpy(fw_info,"info stored",12);
    else
        memcpy(fw_info,"info read err",14);


    ti_sbw_hardware_init();
    ti_sbw_reset_mcu();
    ret=link_status_update(gswitch);
    printk("mcu fw_info %s ret:%d mcu_ver: 0x%x fw_ver: 0x%x \n",fw_info,ret,gswitch->mcu_firmware_version , get_mcu_ver());
#if ENABLE_AUTO_UPGRADE
    if(ret<0 || (gswitch->mcu_force_upgrade)|| (gswitch->mcu_firmware_version < get_mcu_ver()) || get_mcu_ver()>0xF0){
#else
    if(ret<0 || (gswitch->mcu_force_upgrade) || get_mcu_ver()>0xF0){
#endif
    /* if(1){ */

        if(ret<0)
            printk("msp430 update read data failed\n");
        else if(gswitch->mcu_force_upgrade)
            printk("\n*************MSP430 FORCE UPGRADE!!************\n");
        else;

        DBGP("%s: enter !!! current mcu version 0x%x update to 0x%x\n", __func__,gswitch->mcu_firmware_version,get_mcu_ver());

        if(!ti_sbw_hardware_init()){
            DBGP("%s sbw hardware(gpio) init success, update firmware !!! \n",__func__);

            fw_update_flag=1;
            do{

                cpufreq_set_policy(1,1008000,1008000);
                ddrfreq_set_sys_status(SYS_STATUS_AUDIO);
                spin_lock(&update_fw_lock);
                if(ret=updateFirmware()){

                    DBGP("%s SBW MCU FW Update failed! error: %d count: %d \n",__func__,ret,count);
                }else{
                    DBGP("%s SBW MCU FW Update success!  %d\n",__func__,31-count);
                }

                spin_unlock(&update_fw_lock);
                ddrfreq_clear_sys_status(SYS_STATUS_AUDIO);
                cpufreq_set_policy(0,0,1500000);

                count--;
                msleep(200);
            /* }while(count>0); */
            }while(ret && count>0);

            fw_update_flag=0;
            if(ret && (count <=0)){

                fw_ok=FW_UPDATE_FAILED;
            }else{
                fw_ok=FW_UPDATE_SUCCESS;
            }

        }else
            DBGP("%s sbw hardware(gpio) init failed stop update firmware !!! \n",__func__);
    }else{
          fw_ok=FW_NEWEST;

    }

    mcu_start();
    link_status_update(gswitch);
    DBGP("%s SBW MCU current FW 0x%x \n",__func__,gswitch->mcu_firmware_version);
}


#define VOLUME_KEY RK30_PIN0_PC2

static void test_key_work(struct work_struct *work)
{
       struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);
       int irq_flag;

       printk("%s: enter !!!\n", __func__);

       printk("%s:  vol+ is   %d \n", __func__, gpio_get_value(RK30_PIN0_PC2));
       printk("%s:  vol- is   %d \n", __func__, gpio_get_value(RK30_PIN3_PA0));
       if (gpio_get_value(RK30_PIN0_PC2) == 0 && gpio_get_value(RK30_PIN3_PA0) == 1)
       {
           rk29_send_volumeup_key(1);
           rk29_send_volumeup_key(0);
       }

       if (gpio_get_value(RK30_PIN0_PC2) == 0 && gpio_get_value(RK30_PIN3_PA0) == 0)
       {
           rk29_send_volumedown_key(1);
           rk29_send_volumedown_key(0);
       }

       irq_flag = gpio_get_value(VOLUME_KEY) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
       irq_set_irq_type(key_irq, irq_flag);
       enable_irq(key_irq);
}

static irqreturn_t key_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;

        disable_irq_nosync(irq);
	schedule_work(&switch_data->key_work);

        printk("vol+ key \n");
	return IRQ_HANDLED;
}

static int init_volume_key(struct gpio_switch_data *switch_data)
{
        int ret;
        int flags;

	ret = gpio_request(VOLUME_KEY, "volume+");
	if (ret < 0) {
		gpio_free(VOLUME_KEY);
                printk("%s:  vol+  request  fail !!!  \n", __func__);
        }

	ret = gpio_direction_input(VOLUME_KEY);
	if (ret < 0) {
		gpio_free(VOLUME_KEY);
                printk("%s:  vol+  gpio_direction_input  fail !!!  \n", __func__);
        }

	key_irq = gpio_to_irq(VOLUME_KEY);
	if (key_irq < 0) {
		ret = key_irq;
		gpio_free(VOLUME_KEY);
                printk("%s:  vol+  gpio_to_irq  fail !!!  \n", __func__);
	}

        printk("%s: gpio_get_value(VOLUME_KEY)  is   %d \n", __func__, gpio_get_value(VOLUME_KEY));

        flags = gpio_get_value(VOLUME_KEY) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	ret = request_irq(key_irq, key_irq_handler,
                      flags , "vol_key", switch_data);
	if (ret) {
		gpio_free(VOLUME_KEY);
                printk("%s:  vol+  request_irq  fail !!!  \n", __func__);
	}

	INIT_WORK(&switch_data->key_work, test_key_work);

        return ret;
}

static int gpio_switch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gpio_switch_platform_data *pdata = client->dev.platform_data;
	struct gpio_switch_data *switch_data;
	int ret = 0;
    int flags = 0;
    int wait=0;

        printk("register switch gpio \n");

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;
    gswitch=switch_data;
	switch_data->sdev.name = pdata->name;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_gpio_print_state;

        switch_data->client = client;
        i2c_set_clientdata(client, switch_data);

        //Init headsed (add by lipf)
        switch_data->headset.name = "h2w";
        ret = switch_dev_register(&switch_data->headset);
	if(ret < 0){
    	        printk(">>rk30 headset register fail!");
    	        return -ENOMEM;
	}
        //end

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
        //add end

        //add for balance detect
        switch_data->balance_dev.name = "balance_dev";
        ret = switch_dev_register(&switch_data->balance_dev);
        if (ret < 0){
    	        printk(">>rk30 balance_dev register fail!");
    	        return -ENOMEM;
	}      
        //add end

        ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = gpio_request(client->irq, "msp430_irq");
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(client->irq);
	if (ret < 0)
		goto err_set_gpio_input;

        gpio_pull_updown(client->irq, GPIOPullUp);
        //gpio_pull_updown(client->irq, PullDisable);
	INIT_WORK(&switch_data->work, gpio_switch_work);
	INIT_WORK(&switch_data->power_work, power_work);

	switch_data->irq = gpio_to_irq(client->irq);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

        printk("%s: gpio_get_value(client->irq)  is   %d \n", __func__, gpio_get_value(client->irq));
        flags = gpio_get_value(client->irq) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	ret = request_irq(switch_data->irq, gpio_irq_handler,
                      flags , client->dev.driver->name, switch_data);
	if (ret < 0)
		goto err_request_irq;

#if 0
        ret = msp430_get_data(client);
        if (ret < 0) {
            printk("%s: msp430_get_data  fail !!!\n", __func__);
        //    return ret;
        }
#endif

	/* Perform initial detection */


        ti_sbw_hardware_init();
#if 0
        // make mcu exit low power mode
        mcu_start();

        /* link_set_status(LINK_DAC_POWER_ON); */
        msleep(50);
        link_status_update(gswitch);

        ti_sbw_hardware_init();
        ret=updateFirmware();
        printk(KERN_WARNING "SBW updateFirmware ret %d \n",ret);
        mcu_start();
#else
        /* cancel_work_sync(&switch_data->work); */
        /* cancel_work_sync(&switch_data->sdcard_work); */
        /* mcu_work(&switch_data->mcu_update_work); */
        /* schedule_work(&switch_data->mcu_update_work); */

        mcu_wq = create_singlethread_workqueue("mcu upgrade work");
        if (!mcu_wq)
        {
            printk("Creat workqueue failed. mcu upgrade work");
            return -ENOMEM;
        }
        INIT_WORK(&switch_data->mcu_update_work, mcu_work);

        // disable fw update when enter recovery
        if(board_boot_mode() != BOOT_MODE_RECOVERY)
            /* schedule_work(&switch_data->mcu_update_work); */
            queue_work(mcu_wq, &switch_data->mcu_update_work);
#endif
        /* return 0; */

        mcu_start();
        gpio_switch_work(&switch_data->work);
        //add for adc0
        setup_timer(&switch_data->adc0_timer, rk3188_adc0_timer, (unsigned long)switch_data);
        switch_data->adc0_client = adc_register(0, rk3188_adc0_callback, (void *)switch_data);
	if (!switch_data->adc0_client) {
		printk("rk3188 adc0 register error\n");;
		return -EINVAL;
	}
	mod_timer(&switch_data->adc0_timer, jiffies + msecs_to_jiffies(200));

        //add for adc2
        setup_timer(&switch_data->adc2_timer, rk3188_adc2_timer, (unsigned long)switch_data);
        switch_data->adc2_client = adc_register(2, rk3188_adc2_callback, (void *)switch_data);
	if (!switch_data->adc2_client) {
		printk("rk3188 adc2 register error\n");;
		return -EINVAL;
	}

        //add for volume key +/-
    init_volume_key(switch_data);
	mod_timer(&switch_data->adc2_timer, jiffies + msecs_to_jiffies(200));
       
    // wait muc upgrade finished
    do{
        msleep(200);
        wait+=200;
    }while(wait<20001 && !((fw_ok==FW_UPDATE_SUCCESS) || (fw_ok==FW_NEWEST)));


    printk("SWITCH-GPIO end!!!!!! wait: %d\n",wait);
    return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(client->irq);
err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
    switch_dev_unregister(&switch_data->headset);
    switch_dev_unregister(&switch_data->dc_detect);
    switch_dev_unregister(&switch_data->line_control);
    switch_dev_unregister(&switch_data->accessory_dev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit gpio_switch_remove(struct i2c_client *client)
{
	struct gpio_switch_data *switch_data = i2c_get_clientdata(client);

	cancel_work_sync(&switch_data->work);
	cancel_work_sync(&switch_data->sdcard_work);
        switch_dev_unregister(&switch_data->sdev);
        switch_dev_unregister(&switch_data->headset);
        switch_dev_unregister(&switch_data->dc_detect);
        switch_dev_unregister(&switch_data->line_control);
        switch_dev_unregister(&switch_data->accessory_dev);

	kfree(switch_data);
        kfree(i2c_get_clientdata(client));

	return 0;
}

static const struct i2c_device_id msp430_i2c_id[] = {
	{ "msp430", 0 },
	{ }
};

static struct i2c_driver msp430_i2c_driver = {
	.driver = {
		.name = "msp430",
		.owner = THIS_MODULE,
	},
	.probe = gpio_switch_probe,
	.remove = __devexit_p(gpio_switch_remove),
	.id_table = msp430_i2c_id,
};

static int __init gpio_switch_init(void)
{
	return i2c_add_driver(&msp430_i2c_driver);
}

static void __exit gpio_switch_exit(void)
{
	i2c_del_driver(&msp430_i2c_driver);
}

module_init(gpio_switch_init);
/* late_initcall(gpio_switch_init); */
module_exit(gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
