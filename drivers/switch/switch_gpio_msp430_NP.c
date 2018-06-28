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
/* #include <linux/switch_gpio_msp430.h> */
#include <linux/switch_gpio_msp430_NP.h>
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

#if 1
#define DBGCMD(fmt, args...)	printk("MSP430_CMD " fmt, ## args)
#else
#define DBGCMD(fmt, args...)	do{}while(0)
#endif

#if 1
#define DBGEVT(fmt, args...)	printk("MSP430_EVT " fmt, ## args)
#else
#define DBGEVT(fmt, args...)	do{}while(0)
#endif



#define  MSP430_SPEED    100 * 1000

#define  ADC2_LEVEL      300//400//200
#define  ADC2_DK5_LEVEL  540
#define  ADC0_LEVEL_IN   20

#define ENABLE_AUTO_UPGRADE 1

#define SC_INFO_ERASED              5
#define SC_INFO_STORED              6

#define NORM(k) ((k)>0? 1:0)

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

extern void audio_bridge_onoff(int onoff);
extern int cpufreq_set_policy(char gov,int min,int max);
extern noinline void ddrfreq_set_sys_status(enum SYS_STATUS status);
extern noinline void ddrfreq_clear_sys_status(enum SYS_STATUS status);
static struct workqueue_struct *mcu_wq;
extern int board_boot_mode(void);
static DEFINE_SPINLOCK(update_fw_lock);
static volatile int fw_update_flag=0;
enum fw_flag{
    FW_UNKNOW,
    FW_UPDATING,
    FW_UPDATE_FAILED,
    FW_UPDATE_SUCCESS,
    FW_NEWEST,
};
/* enum FW_INFO{ */
    /* FW_INFO_UNKNOW, */
    /* FW_INFO_ERASED, */
    /* FW_INFO_STORED, */
/* }; */
static int fw_ok=FW_UNKNOW;
module_param_named(FW_FLAG, fw_ok, int, 0644);

static char fw_info[32];
module_param_string(FW_INFO_FLAG, fw_info,32, 0644);

extern int Read_info(void);
extern void ak4490_set_lo_mute(int mute);
struct gpio_switch_data*  gswitch=NULL;
static char buf[8] = {0};
static irqreturn_t gpio_irq_handler(int irq, void *dev_id);
static int key_irq;
static int key_irq2;
static volatile int key2_irq_cnt;
static int key_start;

#define VOL_NULL 0
#define VOL_UP   1
#define VOL_DOWN 2

static int mcu_start(void);
int link_status_update(struct gpio_switch_data* switch_data);
int ti_sbw_hardware_init(void);
void ti_sbw_reset_mcu(void);
int updateFirmware(void);
char get_mcu_ver(void);

static bool dock_status = false;

extern void jackline_power(int on);

#if 0
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
#endif

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

                if(level >=0 && (level < ADC2_LEVEL || level > ADC2_DK5_LEVEL)) {
                        
                        if(switch_data->accessory_dev.state != 1) {
                                switch_data->dock_count++;

                                if (switch_data->dock_count > 5) {
                                        switch_set_state(&switch_data->accessory_dev, 1);
                                        switch_data->dock_count = 5;
                                        link_set_status(LINK_EN_DOCK_OUT);
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
                        link_set_status(LINK_UN_DOCK_OUT);
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
    int i;
    char reg=rxData[0];

    /* WARN_ON(1); */
    if(!fw_update_flag){
        for(i=0;i<length;i++){

            ret=i2c_master_reg8_recv(client,reg+i,rxData+i,1,MSP430_SPEED);
        }
    }else{
        printk("msp430 get lock failed receive\n");
    }

    /* printk("MSP430 %d: 0x%x  0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \n",ret, */
           /* rxData[0],rxData[1],rxData[2],rxData[3], */
           /* rxData[4],rxData[5],rxData[6],rxData[7]); */


    return (ret == 1)?  length : ret;
    /* return (ret > 0)? 0 : ret; */
}

static int msp430_send_data(struct i2c_client *client, char *txData, int length)
{
    int ret = -1;
    /* char reg = txData[0]; */
        /* ret = i2c_master_reg8_send(client, reg, &txData[1], length-1, MSP430_SPEED); */
        printk(KERN_WARNING "%s  length %d \n",__func__,length);
        if(!fw_update_flag){
        /* ret=i2c_master_send(client , txData, length); */
        i2c_master_reg8_send(client,txData[0],&txData[1],length,MSP430_SPEED);
        }else
            printk("msp430 get lock failed send\n");
        /* spin_unlock(&update_fw_lock); */
    
	return (ret > 0)? 0 : ret;
}

static int msp430_get_data(struct i2c_client *client)
{
        int ret;
        int count =5;

        do {
            memset(buf, 0, 8);
            ret = msp430_recv_data(client, buf, 8);
            if (ret < 0) {
                printk("%s:  recv data error !!!\n", __func__);
                count--;
            }else{
                break;
            }
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

static int set_line_control(u8 event)
{
       static bool vol_up_press_flags;
       static bool vol_down_press_flags;


       printk("%s:  ok,  event is  0x%x \n", __func__, event);



       switch(event){

       case LINE_VOL_UP:
           rk29_send_volumeup_key(1);
           rk29_send_volumeup_key(0);
            DBGEVT(" line volume up \n");
           break;

       case LINE_VOL_UP_PRESS:
           rk29_send_volumeup_key(1);
           vol_up_press_flags = true;
            DBGEVT(" line volume up Press \n");
           break;

       case LINE_VOL_DOWN:
           rk29_send_volumedown_key(1);
           rk29_send_volumedown_key(0);
            DBGEVT(" line volume Down \n");
           break;

       case LINE_VOL_DOWN_PRESS:
           rk29_send_volumedown_key(1);
           vol_down_press_flags = true;
            DBGEVT(" line volume Down Press\n");
           break;

       case LINE_PLAY_PAUSE:
           rk29_send_pause_key(1);
           rk29_send_pause_key(0);
            DBGEVT(" line pause\n");
           break;

       case LINE_PLAY_NEXT:
           rk29_send_next_key(1);
           rk29_send_next_key(0);
            DBGEVT(" line play next\n");
           break;

       case LINE_PLAY_PREV:
           rk29_send_prev_key(1);
           rk29_send_prev_key(0);
            DBGEVT(" line play preve\n");
           break;

       case LINE_STATUS_RELEASE:
       default:

           if (vol_up_press_flags) {
               rk29_send_volumeup_key(0);
               vol_up_press_flags = false;
               DBGEVT(" line release vol up Press\n");
           }else if(vol_down_press_flags) {
               rk29_send_volumedown_key(0);
               vol_down_press_flags = false;
               DBGEVT(" line release vol down Press\n");
           }

            DBGEVT(" line release: 0x%02x\n",event);
       }

       return 0;

}

static void set_dev_state(struct gpio_switch_data *switch_data)
{

    // force read fw version
    switch_data->mcu_firmware_version=buf[REG_ID];

    if(buf[REG_EVENT] & REG02_STATUS_MASK){


        if(switch_data->mcu_working != NORM(buf[REG_STATUS] & REG02_MCU_WORKING)){
            switch_data->mcu_working =!switch_data->mcu_working;
            DBGEVT("mcu_working\n");

            switch_data->mcu_firmware_version=buf[REG_ID];
        }
        if(switch_data->single_dc_error != NORM(buf[REG_STATUS] & REG02_SINGLE_DC_ERROR)){
        /* if(NORM(buf[REG_STATUS] & REG02_SINGLE_DC_ERROR)){ */
            if(NORM(buf[REG_STATUS] & REG02_SINGLE_DC_ERROR)){

                switch_set_state(&switch_data->dc_detect, 1);
                DBGEVT("detect single_dc_error\n");

            }else{

                switch_set_state(&switch_data->dc_detect, 0);
                DBGEVT("release single_dc_error\n");

            }
            switch_data->single_dc_error=!switch_data->single_dc_error;
        }
        /* if(switch_data->bah_dc_error != NORM(buf[REG_STATUS] & REG02_BAH_DC_ERROR)){ */
        /* switch_data->bah_dc_error=!switch_data->bah_dc_error; */
        /* DBGEVT("bah_dc_error\n"); */
        /* } */
        if(switch_data->po_mute != NORM(buf[REG_STATUS] & REG02_MUTE_PO_EN)){
            switch_data->po_mute=!switch_data->po_mute;
            DBGEVT("LINK PO mute\n");
        }
        if(switch_data->mcu_busy != NORM(buf[REG_STATUS] & REG02_MCU_BUSY)){
            switch_data->mcu_busy=!switch_data->mcu_busy;
            DBGEVT("mcu_busy\n");
        }

        if(switch_data->lo_mute != NORM(buf[REG_STATUS] & REG02_MUTE_LO_EN)){
            switch_data->lo_mute=!switch_data->lo_mute;
            DBGEVT("LINK LO mute\n");
        }
    }else{

    }

    if(buf[REG_EVENT] & REG03_INSERT_MASK){


        if(switch_data->headset.state != NORM(buf[REG_INSERT] & REG03_PO_DETECT)){
            /* switch_data->headset.state=!headset.state; */

            //po insert
            if(NORM(buf[REG_INSERT] & REG03_PO_DETECT)){

                switch_set_state(&switch_data->headset, 1);

                schedule_work(&switch_data->power_work);

                DBGEVT("PO insert\n");
                //po pullout 
            }else{

                switch_set_state(&switch_data->headset, 0);

                link_set_status(LINK_WIRE_CTL_DISABLE);

                jackline_power(0);
                DBGEVT("PO pullout\n");

            }
        }

        if(switch_data->sdev.state != NORM(buf[REG_INSERT] & REG03_LINE_DETECT)){
            if(NORM(buf[REG_INSERT] & REG03_LINE_DETECT)){
                switch_set_state(&switch_data->sdev, 1);
                DBGEVT("LO inset\n");
            }else{
                switch_set_state(&switch_data->sdev, 0);
                DBGEVT("LO pullout\n");
            }
        }
    }


    if(buf[REG_EVENT] & REG04_OUT_STATUS_MASK){

        if(switch_data->phone_out != NORM(buf[REG_OUT_PORT] & REG04_PHONE_OUT)){
            switch_data->phone_out = !switch_data->phone_out;
            DBGEVT(" link phone out\n");
        }
        if(switch_data->line_out != NORM(buf[REG_OUT_PORT] & REG04_LINE_OUT)){
            switch_data->line_out = !switch_data->line_out;
            DBGEVT(" link line out\n");
        }
        if(switch_data->bah_out != NORM(buf[REG_OUT_PORT] & REG04_BAH_OUT)){

            switch_data->bah_out = !switch_data->bah_out;
            DBGEVT(" link bah out\n");
        }
        if(switch_data->opt_out != NORM(buf[REG_OUT_PORT] & REG04_OPT_OUT)){
            switch_data->opt_out = !switch_data->opt_out;
            DBGEVT(" link opt out\n");
        }

        if(switch_data->coax_out != NORM(buf[REG_OUT_PORT] & REG04_COAX_OUT)){
            switch_data->coax_out = !switch_data->coax_out;
            DBGEVT(" link coax out\n");
        }

        if(switch_data->analog_power != NORM(buf[REG_OUT_PORT] & REG04_ANALOG_POWER)){
            switch_data->analog_power= !switch_data->analog_power;

            // use dac_power to indicate power down 
            // if jack not insert analog_power will not opened
            if(!switch_data->analog_power)// indicate power down finished
                switch_data->is_power_process =0;

            DBGEVT(" link analog_power %s\n",switch_data->analog_power? "on":"off");
        }
        if(switch_data->dac_power != NORM(buf[REG_OUT_PORT] & REG04_DAC_POWER)){
            switch_data->dac_power= !switch_data->dac_power;

            if(!switch_data->dac_power && !switch_data->analog_power)
                switch_data->is_power_process =0;

            DBGEVT(" link dac_power %s\n",switch_data->dac_power? "on":"off");
        }
    }


    if(buf[REG_EVENT] & REG05_AMP_MODULE_MASK){
        switch_data->am_module=buf[REG_AM_MODULE] & REG05_AM_MODULE;
        DBGEVT(" link new module: %d\n",switch_data->am_module);
        switch_set_state(&switch_data->amp_card,switch_data->am_module);

    }


    if(buf[REG_EVENT] & REG06_KEY_MASK){

        set_line_control((buf[REG_KEY_EVENT]&REG06_KEY_EVENT));

    }
}

static void gpio_switch_work(struct work_struct *work)
{
       struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);
       struct i2c_client *client = data->client;

       printk("%s: enter !!!\n", __func__);

       if (msp430_get_data(client) < 0){
           printk("%s:  msp430_get_data  fail !!!\n", __func__);
           enable_irq(client->irq);
           return;
       }

       enable_irq(client->irq);

       set_dev_state(data);
}

static void gpio_switch_force_read(struct gpio_switch_data	*sdata)
{
       struct gpio_switch_data	*data =sdata;
       struct i2c_client *client = data->client;

       printk("%s: enter !!!\n", __func__);

       if (msp430_get_data(client) < 0){
           printk("%s:  msp430_get_data  fail !!!\n", __func__);
           return;
       }

       set_dev_state(data);
}


static void power_work(struct work_struct *work)
{
    struct gpio_switch_data	*sdata =
        container_of(work, struct gpio_switch_data, power_work);

    printk("%s: enter !!! %d\n", __func__,sdata->line_control.status);

    msleep(1500);

    // open power if po or bal insert
    jackline_power(1);
    if(sdata->line_control.status){
        /* jackline_power(1); */
        link_set_status(LINK_WIRE_CTL_ENABLE);
    }else{
        /* jackline_power(0); */
        link_set_status(LINK_WIRE_CTL_DISABLE);
    }
}


static void extract_link_state(struct gpio_switch_data *switch_data)
{
#if 0
       switch_data->analog_power=1;
       switch_data->mcu_working=1;
       switch_data->first_play =1;
       switch_data->dc_error=0;
       switch_data->mute=0;
       switch_data->dac_en=1;
       switch_data->dock_en=0;

       return ;

 
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
#endif
}



int link_status_update(struct gpio_switch_data* switch_data)
{
    int ret=0;


    /* mcu_start(); */
    gpio_switch_force_read(switch_data);
    return 0;
    extract_link_state(switch_data);
    return 0;
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
    int cmd;
    /* uint8_t dat[4]={0,0,0,0}; */
    uint8_t xreg=0x20;
    uint8_t reg1=0;
    uint8_t reg2=0;
    uint8_t sreg[4];

    /* DBGP("%s  status 0x%x \n",__func__,(uint32_t)status); */

    if( status == LINK_EN_APOWER_DOWN ){
        gswitch->is_power_process =1;
    }
    cmd=(int)status;
    if(status < LINK_REG20_MAX){
        xreg=0x20;
    }else if(status < LINK_REG21_MAX){
        xreg=0x21;
    }else if(status < LINK_REG22_MAX){
        xreg=0x22;
    }else{
        printk(KERN_WARNING "link_set_status status is not in enum %d \n",(int)status);
        goto enum_error;
    }


    reg1=xreg;
    ret = msp430_recv_data(gswitch->client, &reg1, 1);
    if(ret<0)
        goto i2c_error;

    reg2=reg1;

    switch(status){
    case LINK_FORCE_MCU_UPGRADE:
            fw_ok=FW_UNKNOW;
            link_stop(0,1);
            gswitch->mcu_force_upgrade=1;
            /* schedule_work(&gswitch->mcu_update_work); */
            queue_work(mcu_wq, &gswitch->mcu_update_work);
            return 0;

    default: ;
    }


    switch(status){
    case LINK_MCU_WAKEUP:  reg1|=0x80;break;
    case LINK_MCU_STANDBY: reg1&=~0x80;break;

    case LINK_WIRE_CTL_ENABLE:
    case LINK_EN_DBW:      reg1|=0x40;break;
    case LINK_WIRE_CTL_DISABLE:
    case LINK_UN_DBW:      reg1&=~0x40;break;



    case LINK_EN_OTG:      reg1|=0x80;break;
    case LINK_UN_OTG:      reg1&=~0x80;break;

    case LINK_EN_DAC:      reg1|=0x40;break;
    case LINK_UN_DAC:      reg1&=~0x40;break;

    case LINK_EN_AUPOWER:  reg1|=0x20;break;
    case LINK_UN_AUPOWER:  reg1&=~0x20;break;

    case LINK_EN_APOWER_DOWN:  reg1|=0x10;
                               reg1&=~0x01;break;

    case LINK_UN_APOWER_DOWN:  reg1&=~0x10;break;

    case LINK_AM_GAIN_LOW:   reg1&=~0x08;break;
    case LINK_AM_GAIN_HIGH:  reg1|=0x08;break;

    case LINK_MUTE_LO:  reg1|=0x04;break;
    case LINK_UNMUTE_LO:  reg1&=~0x04;break;

    case LINK_MUTE_PO:  reg1|=0x02;break;
    case LINK_UNMUTE_PO:  reg1&=~0x02; break;

    case LINK_PLAY_START:  reg1|=0x01;break;



    case LINK_EN_PHONE_OUT:  reg1|=0x80;break;
    case LINK_UN_PHONE_OUT:  reg1&=~0x80;break;

    case LINK_EN_LINE_OUT:   reg1&=~0x18; reg1 |= 0x08;break;
    case LINK_EN_OPT_OUT:    reg1&=~0x18; reg1 |= 0x10;break;
    case LINK_EN_COAX_OUT:   reg1&=~0x18; reg1 |= 0x18;break;

    case LINK_UN_LINE_OUT:
    case LINK_UN_OPT_OUT:
    case LINK_UN_COAX_OUT:   reg1 &= ~0x18;break;

    case LINK_EN_DOCK_OUT:   reg1|=0x04;break;
    case LINK_UN_DOCK_OUT:   reg1&=~0x04;break;

    case LINK_FORCE_POWER_ON:   reg1|=0x01;break;

    default:
         printk(KERN_ERR"link_set_status status not in enum %d \n",status);
    }


    if(cmd ==  cs[cmd].cmd)
        DBGCMD(" %s reg[%02x]: 0x%02x to 0x%02x\n",cs[cmd].name,xreg,reg2,reg1);
    else
        DBGCMD("LINK CMD table is not corret: cmd %d cs.cmd %d cs.name %s\n",cmd,cs[cmd].cmd,cs[cmd].name);

    if(reg2==reg1)
         printk("link_set_status status[%d] have been set,reg[0x%x]=0x%x \n",status,xreg,reg1);

    sreg[0]=xreg;
    sreg[1]=reg1;
    ret= msp430_send_data(gswitch->client, sreg,1);

    return ret;


enum_error:

    return -EINVAL;
i2c_error:
    return -ENXIO;

}

EXPORT_SYMBOL_GPL(link_set_status);

void link_set_force_poweron(void)
{
    link_set_status(LINK_FORCE_POWER_ON);
}
void link_set_force_upgrade(void)
{
    link_set_status(LINK_FORCE_MCU_UPGRADE);
}
bool link_get_status(enum link_status status)
{

#if 0
    int ret=0,count=1;
    if((ret=link_status_update(gswitch))<0){
        printk(KERN_ERR "audio link status update Failed!! use old status\n");
        /* return ret; */
    }
#endif

    switch(status){
    case LINK_REPORT_MCU_WORK: return gswitch->mcu_working;

    case LINK_REPORT_ANALOG_PWR: return gswitch->analog_power;

    case LINK_REPORT_PLAYING: return gswitch->start_play;

    case LINK_REPORT_DAC_PWR: return gswitch->dac_power;

    case LINK_REPORT_MUTE_ON: return gswitch->po_mute;

    case LINK_REPORT_POWER_PROCESS: return gswitch->is_power_process;
    default:
         printk(KERN_WARNING "link_get_status status error  %d \n",status);
         return false;

    }


}
EXPORT_SYMBOL_GPL(link_get_status);

u8 mcu_get_fw_ver(void)
{
    link_get_status(LINK_MCU_STANDBY);
    return gswitch->mcu_firmware_version;
}

bool has_jack_insert(void)
{
    if(gswitch){
        if( gswitch->headset.state || gswitch->sdev.state)
            return true;
        else
            return false;
    }else
        return false;

}

static int mcu_start()
{

    int ret=0,count=10;


    if(!gswitch->mcu_working){

        link_set_status(LINK_MCU_WAKEUP);

        do{
            msleep(5);
            count--;
            /* ret=link_status_update(gswitch); */

        }while(!gswitch->mcu_working && count>0);

    }else
        printk(KERN_WARNING "LINK_MCU_WORK in statues,ret \n");


    if(count<=0){
        printk(KERN_WARNING "LINK_MCU_WAKEUP error,ret %d \n",ret);
        return ret==0? -ETIMEDOUT:-EAGAIN;
    }

    return ret;
}

static int mcu_stop(void)
{

    int ret=0,count=10;

    if(gswitch->mcu_working){

        link_set_status(LINK_MCU_STANDBY);

        do{
            msleep(5);
            count--;
        }while(!gswitch->mcu_working && count>0);

    }
    if(count<=0){
        printk(KERN_WARNING "LINK_MCU_STANDBY error,ret %d \n",ret);
        ret = -1;
    }
    return ret==0? -ETIMEDOUT:-EAGAIN;
}

int link_start(void)
{

    int ret=0,count=50,wcount=20;
    enum link_status status=LINK_REPORT_ANALOG_PWR;

    while(link_get_status(LINK_REPORT_POWER_PROCESS) && wcount ){

        wcount--;
        printk("MSP430 wait power down process \n  ");
        msleep(200);
    }


    audio_bridge_onoff(1);
    mcu_start();


    if(!link_get_status(LINK_REPORT_PLAYING)){

        link_set_status(LINK_PLAY_START);
        gswitch->start_play=1;
        if(!has_jack_insert())
            status=LINK_REPORT_DAC_PWR;

        do{
            msleep(50);
            count--;

        }while(!link_get_status(status) && count);
    }
    DBGP("%s %d ms success\n",__func__,(50-count)*50);
    return ret;
}
EXPORT_SYMBOL_GPL(link_start);

int link_stop(int tosleep,int fpga_off)
{

    int ret=0;

    if(fpga_off)
        audio_bridge_onoff(0);

    gswitch->start_play=0;
    link_set_status(LINK_EN_APOWER_DOWN);
    if(tosleep){
        /* link_set_status(LINK_MCU_STANDBY); */
        mcu_stop();
    }

    DBGP("%s  success\n",__func__);
    return ret;

}
EXPORT_SYMBOL_GPL(link_stop);

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    struct gpio_switch_data *switch_data =
        (struct gpio_switch_data *)dev_id;

    disable_irq_nosync(irq);
    schedule_work(&switch_data->work);

    printk("MSP430 interrupt \n");
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

static int line_control_process(struct switch_dev * sdev)
{

    printk(" sdev %s %d \n",sdev->name,sdev->status);
    if(sdev->status){
        // power control by power_work
        /* jackline_power(1); */
        link_set_status(LINK_WIRE_CTL_ENABLE);
    }else{
        /* jackline_power(0); */
        link_set_status(LINK_WIRE_CTL_DISABLE);
    }
    return 0;

}

static int am_gain_process(struct switch_dev * sdev)
{

    DBGP(" sdev %s %d \n",sdev->name,sdev->status);
    if(sdev->status){
        link_set_status(LINK_AM_GAIN_HIGH);
    }else{
        link_set_status(LINK_AM_GAIN_LOW);
    }
    return 0;

}

static void mcu_work(struct work_struct *work)
{
    /* struct gpio_switch_data	*data = */
        /* container_of(work, struct gpio_switch_data, work); */
    int ret,count=10;
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
        memcpy(fw_info,"info erased\0",12);
    else if(info == SC_INFO_STORED)
        memcpy(fw_info,"info stored\0",12);
    else
        memcpy(fw_info,"info read err\0",14);


    printk("mcu fw_info %s \n",fw_info);
    ti_sbw_hardware_init();
    ti_sbw_reset_mcu();
    ret=link_status_update(gswitch);
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
                if((ret=updateFirmware())!=0){

                    DBGP("%s SBW MCU FW Update failed! error: %d count: %d \n",__func__,ret,count);
                }else{
                    DBGP("%s SBW MCU FW Update success!  %d\n",__func__,31-count);
                }

                spin_unlock(&update_fw_lock);
                ddrfreq_clear_sys_status(SYS_STATUS_AUDIO);
                cpufreq_set_policy(0,0,1500000);

                count--;
                msleep(200);
            }while(ret && count>0);

            fw_update_flag=0;
            if(ret && (count <=0)){

                fw_ok=FW_UPDATE_FAILED;
            }else{
                fw_ok=FW_UPDATE_SUCCESS;

                link_status_update(gswitch);
                DBGP("%s SBW MCU current FW 0x%x \n",__func__,gswitch->mcu_firmware_version);
            }

        }else
            DBGP("%s sbw hardware(gpio) init failed stop update firmware !!! \n",__func__);
    }else{
          fw_ok=FW_NEWEST;
          DBGP("%s SBW MCU current FW 0x%x is the newest \n",__func__,gswitch->mcu_firmware_version);
    }
}


#define VOLUME_KEY RK30_PIN0_PC2
#define VKEY       RK30_PIN3_PA0

static void test_key_work(struct work_struct *work)
{
       /* struct gpio_switch_data	*data = */
		/* container_of(work, struct gpio_switch_data, work); */
       int irq_flag;

       /* printk("%s: enter !!!\n", __func__); */

       /* printk("%s:  vol+ is   %d \n", __func__, gpio_get_value(RK30_PIN0_PC2)); */
       /* printk("%s:  vol- is   %d \n", __func__, gpio_get_value(RK30_PIN3_PA0)); */

#if 0
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

#else

       udelay(20);
       if (gpio_get_value(RK30_PIN0_PC2) == 0 ){
           if(gpio_get_value(RK30_PIN3_PA0) == 1 ){

               key_start=VOL_UP;

               key2_irq_cnt=0;
               DBG("-----------VOL_UP start    --------\n");
           }else{

               key_start=VOL_DOWN;
               key2_irq_cnt=0;
               DBG("-----------VOL_DOWN start  --------\n");
           }

       }else{

           if(key2_irq_cnt == 1 ){

               if(key_start == VOL_UP ){

                   rk29_send_volumeup_key(1);
                   rk29_send_volumeup_key(0);

                   DBG("*********** VOL_UP sync    ********\n\n");
               }else if(key_start == VOL_DOWN ){

                   rk29_send_volumedown_key(1);
                   rk29_send_volumedown_key(0);
                   DBG("*********** VOL_DOWN sync  *******\n\n");
               }else{

                   printk(KERN_ERR "%s: enter , sync error status of key cnt %d start %d\n\n", __func__,key2_irq_cnt ,key_start);
               }

           }else{
               printk(KERN_ERR "%s: enter , sync error status of key cnt %d start %d\n\n", __func__,key2_irq_cnt ,key_start);

           }


           key2_irq_cnt=0;
           key_start=VOL_NULL ;
       }
#endif

       irq_flag = gpio_get_value(VOLUME_KEY) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
       irq_set_irq_type(key_irq, irq_flag);
       enable_irq(key_irq);
}

static irqreturn_t key_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;

        disable_irq_nosync(irq);
	    schedule_delayed_work(&switch_data->key_work,0);

        DBG("vol+ key \n");
	return IRQ_HANDLED;
}
static irqreturn_t key_irq_handler2(int irq, void *dev_id)
{
	/* struct gpio_switch_data *switch_data = */
		/* (struct gpio_switch_data *)dev_id; */

    key2_irq_cnt++;
    DBG("vol- key \n");
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

	INIT_DELAYED_WORK(&switch_data->key_work, test_key_work);


    ////// volume key 2
    ret = gpio_request(VKEY, "volume-");
    if (ret < 0) {
        gpio_free(VKEY);
        printk("%s:  vol-  request  fail !!!  \n", __func__);
    }

	gpio_direction_input(VKEY);

	key_irq2 = gpio_to_irq(VKEY);

	ret = request_irq(key_irq2, key_irq_handler2,
                      IRQF_TRIGGER_RISING, "vol_key2", switch_data);
	if (ret) {
		gpio_free(VKEY);
        printk("%s:  vol-  request_irq  fail !!!  \n", __func__);
	}

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
        switch_data->line_control.status_store_process= line_control_process;
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

    switch_data->amp_card.name = "amp_card";
    switch_data->amp_card.status_store_process= am_gain_process;
    ret = switch_dev_register(&switch_data->amp_card);
    if (ret < 0){
         printk(">>rk30 amp_card det register fail!");
         return -ENOMEM;
    }

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

    disable_irq(switch_data->irq);

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

    printk("wait %d fw_ok %d \n",wait,fw_ok);

    enable_irq(client->irq);
    ti_sbw_reset_mcu();
    /* msleep(100); */
    /* mcu_start(); */
    /* gpio_switch_work(&switch_data->work); */
    printk("SWITCH-GPIO end!!!!!! wait: %d\n",wait);
    return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(client->irq);
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

static int __devexit gpio_switch_remove(struct i2c_client *client)
{
	struct gpio_switch_data *switch_data = i2c_get_clientdata(client);

	cancel_work_sync(&switch_data->work);
	cancel_work_sync(&switch_data->sdcard_work);
        switch_dev_unregister(&switch_data->sdev);
        switch_dev_unregister(&switch_data->headset);
        switch_dev_unregister(&switch_data->amp_card);
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
