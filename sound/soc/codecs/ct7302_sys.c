/*
 * BQ2589x battery charging debug sys
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
/* #include <linux/power/bq2589x_reg.h> */
#include <mach/board.h>
#include <linux/device.h>

#include "ak4490.h"

const static char AUDIO_BRIDGE_NAME[]="CT730X";
#define DEVICE_NAME  "CT7302"
/* #define DEVICE_OBJ_NAME(NAME)  (NAME##_debug) */
#define DEVICE_OBJ_NAME  "audio_bridge"

extern int gct7302_write_i2c(u8 reg, u8 data);
extern int gct7302_read_i2c(u8 reg, u8* data);
extern int gct7302_read_i2c_blk(u8 reg, u8* data,u8 len);
extern int ak4490_read_i2c_blk(u8 reg, u8* data,u8 len);
extern int ak4490_write_i2c_lr(u8 lr,u8 reg,u8 data);

#define REG_BITS      8
#define REG_DTYPE     u8
#define REG_MAX       0x97 // 0x00 - 0x14
#define REG_MAX_AK4490   AK4490_MAX_REGISTERS
typedef REG_DTYPE reg_t ;

#define DEVICE_REG_WRITE(reg,data) gct7302_write_i2c(reg,data)
#define DEVICE_REG_READ(reg,data) gct7302_read_i2c(reg,data)
#define DEVICE_REG_READ_BLK(reg,data,len) gct7302_read_i2c_blk(reg,data,len)
#define DEVICE_REG_READ_BLK_AK4490(reg,data,len) ak4490_read_i2c_blk(reg,data,len)
#define DEVICE_REG_WRITE_AK4490(lr,reg,data) ak4490_write_i2c_lr(lr,reg,data)

int gAndroid_boot_completed=0;
static struct kobject * device_kobj;

static ssize_t device_debug_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    int ret = 0;
#ifdef PRINT_INT_INFO
    debug_flage = !debug_flage;
    if(debug_flage)
        printk("elan debug switch open\n");
    else
        printk("elan debug switch close\n");
#endif
    return ret;
}
static DEVICE_ATTR(debug, S_IRUGO, device_debug_show, NULL);

struct sys_msg{
    u8 lr;
    u16 reg;
    reg_t value;
    u8 reg_type;
    u8 read_write;
    u16 num;

};
static struct sys_msg msg;
static ssize_t send_cmd_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t size)
{


    /* echo reg16 value32 reg-type[8,16,32] read_write[0,1] num > xxxx
     *
     */

    if (sscanf(buf, "0x%x 0x%x %d %d %d\n", \
               (unsigned int *)&msg.reg,(unsigned int *)&msg.value,\
               (unsigned int *)&msg.reg_type,(unsigned int *)&msg.read_write,\
               (unsigned int *)&msg.num) != 5){
        printk("device cmd format error\n");
        return -EINVAL;
    }
    printk("send cmd: reg 0x%x value 0x%x type %d read_write %d num:%d \n",
           msg.reg,msg.value,msg.reg_type,msg.read_write,msg.num);

    // write data
    if(!msg.read_write){
        DEVICE_REG_WRITE(msg.reg , msg.value);

        /* you can read after write reg */
        msg.read_write=1;


        return size;
    }
        return -EINVAL;
}

static ssize_t read_reg_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{

    ssize_t ret;
    u16 reg=0;
    char rbuf[PAGE_SIZE];
    char dbuf[256];

#if 0
    if(!msg.read_write){
        sprintf(buf,"TC READ: set read addr add read_write flag fist : %d\n",msg.read_write);
        ret = strlen(buf) + 1;
        return ret;
    }
#endif

    DEVICE_REG_READ_BLK(0,dbuf,REG_MAX);

    do{
        /* DEVICE_REG_READ(reg,&msg.value); */
        /* sprintf(rbuf,"READ:reg 0X%04X value 0X%08X\n",reg,msg.value ); */
        sprintf(rbuf,"READ:reg 0X%04X value 0X%08X\n",reg,dbuf[reg] );
        strcat(buf,rbuf);

        reg++;

    }while(reg<REG_MAX);

    ret = strlen(buf) + 1;
    return ret;

}
static DEVICE_ATTR(reg, S_IALLUGO, read_reg_show, send_cmd_store);

static ssize_t send_cmd_store_ak4490(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t size)
{


    /* echo reg16 value32 reg-type[8,16,32] read_write[0,1] num > xxxx
     *
     */

    if (sscanf(buf, "%c 0x%x 0x%x\n",&msg.lr,\
               (unsigned int *)&msg.reg,(unsigned int *)&msg.value) != 3){
        printk("device cmd format error : [LR] reg data,format is Hex \n");
        return -EINVAL;
    }
    printk("SEND CMD: DAC %c reg 0x%x value 0x%x \n", msg.lr, msg.reg,msg.value);

    DEVICE_REG_WRITE_AK4490(msg.lr, msg.reg, msg.value);


    return size;
}

static ssize_t read_reg_show_ak4490(struct device *dev,
                             struct device_attribute *attr, char *buf)
{

    ssize_t ret;
    u16 reg=0;
    char rbuf[PAGE_SIZE];
    char dbuf[256];

#if 0
    if(!msg.read_write){
        sprintf(buf,"TC READ: set read addr add read_write flag fist : %d\n",msg.read_write);
        ret = strlen(buf) + 1;
        return ret;
    }
#endif

    DEVICE_REG_READ_BLK_AK4490(0,dbuf,REG_MAX_AK4490);

    do{
        sprintf(rbuf,"READ:reg  0X%02X L 0X%02X R 0X%02X\n",reg,dbuf[reg] ,dbuf[reg+REG_MAX_AK4490]);
        strcat(buf,rbuf);

        reg++;

    }while(reg<REG_MAX_AK4490);

    ret = strlen(buf) + 1;
    return ret;

}

static DEVICE_ATTR(ak4490_reg, S_IALLUGO, read_reg_show_ak4490, send_cmd_store_ak4490);
static ssize_t sys_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    strcpy(buf,AUDIO_BRIDGE_NAME);

    return strlen(AUDIO_BRIDGE_NAME);
}
static DEVICE_ATTR(info, S_IRUGO, sys_info_show, NULL);

static ssize_t boot_completed_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t size)
{
    char cmd[size+1];
    if(size >0){
        memcpy(cmd,buf,size);
        cmd[size]='\0';
        if( strcmp(cmd,"boot_completed") == 0){
            gAndroid_boot_completed=1;
            printk("boot_completed_store get msg: <%s>\n",cmd);
        }else{
            printk("boot_completed_store unkown command: %s  %d\n",buf,size);
        }
    }

    return size;
}
static DEVICE_ATTR(boot, S_IRWXUGO,NULL, boot_completed_store);




static struct attribute *sysfs_attrs_ctrl[] = {
    &dev_attr_debug.attr,
    &dev_attr_info.attr,
    &dev_attr_boot.attr,
    &dev_attr_reg.attr,
    &dev_attr_ak4490_reg.attr,
    NULL
};
static struct attribute_group device_attribute_group[] = {
    {.attrs = sysfs_attrs_ctrl },
};

static void device_node_init(void)
{
    int ret ;
    /* struct elan_ts_data *ts = private_ts; */

    device_kobj= kobject_create_and_add(DEVICE_OBJ_NAME, NULL) ;
    if (device_kobj== NULL){
        printk(KERN_ERR " %s : kobject_create_and_add failed\n", __func__);
        return;
    }
    ret = sysfs_create_group(device_kobj, device_attribute_group);
    if (ret < 0) {
        printk(KERN_ERR "[%s] %s: sysfs_create_group failed\n",DEVICE_NAME , __func__);
    }

#ifdef ELAN_IAP_DEV
    ts->firmware.minor = MISC_DYNAMIC_MINOR;
    ts->firmware.name = "elan-iap";
    ts->firmware.fops = &elan_touch_fops;
    ts->firmware.mode = S_IFREG|S_IRWXUGO;

    if (misc_register(&ts->firmware) < 0)
        printk("[elan debug] misc_register failed!!\n");
    else
        printk("[elan debug] misc_register ok!!\n");

    ts->p = proc_create("elan-iap", 0666, NULL, &elan_touch_fops);
    if (ts->p == NULL){
        printk("[elan debug] proc_create failed!!\n");
    }
    else{
        printk("[elan debug] proc_create ok!!\n");
    }
#endif

    return;
}

static void device_node_deinit(void)
{
    if(device_kobj){
        sysfs_remove_group(device_kobj, device_attribute_group);
        kobject_put(device_kobj);
    }

#ifdef ELAN_IAP_DEV
    misc_deregister(&private_ts->firmware);
    remove_proc_entry("elan-iap", NULL);
#endif

}
late_initcall(device_node_init);
module_exit(device_node_deinit);
