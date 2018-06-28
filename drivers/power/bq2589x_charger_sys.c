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
#include <linux/power/bq2589x_reg.h>
#include <mach/board.h>
#include <linux/device.h>
#include <linux/ctype.h>


#define DEVICE_NAME  "BQ2589X"
/* #define DEVICE_OBJ_NAME(NAME)  (NAME##_debug) */
#define DEVICE_OBJ_NAME  "bq2589x_debug"



#define REG_BITS      8
#define REG_DTYPE     u8
#define REG_MAX       0x15 // 0x00 - 0x14
typedef REG_DTYPE reg_t ;

#define DEVICE_REG_WRITE(reg,data) gbq2589x_write_byte(reg,data)
#define DEVICE_REG_READ(reg,data) gbq2589x_read_byte(reg,data)

static struct kobject * device_kobj;
static int vehicle_charger = 0;

static ssize_t input_cur_limit_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_00, &val);
    val &= BQ2589X_IINLIM_MASK;
    val >>= BQ2589X_IINLIM_SHIFT;
    data = (int)val*BQ2589X_IINLIM_LSB + BQ2589X_IINLIM_BASE;

    sprintf(buf, "%d\n", data);
    ret = strlen(buf) + 1;

    return ret;
}
static ssize_t thermal_show(struct device *dev,	
				struct device_attribute *attr,	
				char *buf)	
{  
   char* thermal_arr[7]={
	   "LOW_SHUTDOWN",
	   "LOW_TEMP_STOP_CHARGING",
	   "LOW_TEMP_LIMIT_CURRENT",
	   "NORMAL",
	   "HIGH_TEMP_LIMIT_CURRENT",
	   "HIGH_TEMP_STOP_CHARGING",
	   "HIGH_SHUTDOWN",
   };
   int ret = 0;
   sprintf(buf, "%s\n", thermal_arr[thermal_s]);
   ret = strlen(buf) + 1;

   return ret;	
}  

static ssize_t en_charge_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_03, &val);
    val &= BQ2589X_CHG_CONFIG_MASK;
    val >>= BQ2589X_CHG_CONFIG_SHIFT;

    sprintf(buf, "%d\n", val);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t fast_charge_cur_limit_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_04, &val);
    val &= BQ2589X_ICHG_MASK;
    val >>= BQ2589X_ICHG_SHIFT;
    data = (int)val*BQ2589X_ICHG_LSB + BQ2589X_ICHG_BASE;

    sprintf(buf, "%d\n", data);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t charge_vol_limit_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_06, &val);
    val &= BQ2589X_VREG_MASK;
    val >>= BQ2589X_VREG_SHIFT;
    data = (int)val*BQ2589X_VREG_LSB + BQ2589X_VREG_BASE;

    sprintf(buf, "%d\n", data);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t vbus_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    char *type;

    gbq2589x_read_byte(BQ2589X_REG_0B, &val);
    val &= BQ2589X_VBUS_STAT_MASK;
    val >>= BQ2589X_VBUS_STAT_SHIFT;

    if (val == 0) {
        type = "NONE";
    } else if (val == 1) {
        type = "USB_SDP";
    } else if (val == 2) {
        type = "USB_CDP";
    } else if (val == 3) {
        type = "USB_DCP";
    } else if (val == 4) {
        type = "MAXC";
    } else if (val == 5) {
        type = "UNKNOWN";
    } else if (val == 6) {
        type = "NONSTAND";
    } else if (val == 7) {
        type = "OTG";
    }

    sprintf(buf, "%s\n", type);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t charge_state_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    char *type;

    gbq2589x_read_byte(BQ2589X_REG_0B, &val);
    val &= BQ2589X_CHRG_STAT_MASK;
    val >>= BQ2589X_CHRG_STAT_SHIFT;

    if (val == 0) {
        type = "discharging";
    } else if (val == 1) {
        type = "pre-charge";
    } else if (val == 2) {
        type = "fast-charging";
    } else if (val == 3) {
        type = "charge-done";
    }

    sprintf(buf, "%s\n", type);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t reg_0c_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;

    gbq2589x_read_byte(BQ2589X_REG_0C, &val);

    sprintf(buf, "0x%02x\n", val);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t input_vol_limit_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_0D, &val);
    val &= BQ2589X_VINDPM_MASK;
    val >>= BQ2589X_VINDPM_SHIFT;
    data = (int)val*BQ2589X_VINDPM_LSB + BQ2589X_VINDPM_BASE;

    sprintf(buf, "%d\n", data);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t battery_vol_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_0E, &val);
    val &= BQ2589X_BATV_MASK;
    val >>= BQ2589X_BATV_SHIFT;
    data = (int)val*BQ2589X_BATV_LSB + BQ2589X_BATV_BASE;

    sprintf(buf, "%d\n", data);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t sys_voltage_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_0F, &val);
    val &= BQ2589X_SYSV_MASK;
    val >>= BQ2589X_SYSV_SHIFT;
    data = (int)val*BQ2589X_SYSV_LSB + BQ2589X_SYSV_BASE;

    sprintf(buf, "%d\n", data);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t ts_percentage_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_10, &val);
    val &= BQ2589X_TSPCT_MASK;
    val >>= BQ2589X_TSPCT_SHIFT;
    data = (int)(val*BQ2589X_TSPCT_LSB)/1000 + BQ2589X_TSPCT_BASE;

    sprintf(buf, "%d\n", data);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t vbus_vol_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_11, &val);
    val &= BQ2589X_VBUSV_MASK;
    val >>= BQ2589X_VBUSV_SHIFT;
    data = (int)val*BQ2589X_VBUSV_LSB + BQ2589X_VBUSV_BASE;

    sprintf(buf, "%d\n", data);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t charge_limit_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_12, &val);
    val &= BQ2589X_ICHGR_MASK;
    val >>= BQ2589X_ICHGR_SHIFT;
    data = (int)val*BQ2589X_ICHGR_LSB + BQ2589X_ICHGR_BASE;

    sprintf(buf, "%d\n", data);
    ret = strlen(buf) + 1;

    return ret;
}

bool is_vehicle_mode(void)
{
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_0A, &val);
    val &= BQ2589X_RESERVED_MASK;
    val >>= BQ2589X_RESERVED_SHIFT;
    data = (int)val;

    if (data == 1)
        return true;

    return false;
}
EXPORT_SYMBOL_GPL(is_vehicle_mode);

static ssize_t vehicle_mode_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    u8 val;
    int data;

    gbq2589x_read_byte(BQ2589X_REG_0A, &val);
    val &= BQ2589X_RESERVED_MASK;
    val >>= BQ2589X_RESERVED_SHIFT;
    data = (int)val;

    sprintf(buf, "%d\n", data);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t vehicle_mode_store(struct device *dev, struct device_attribute *attr,
                              const char *buf, size_t size)
{
    ssize_t ret = -EINVAL;
    char *after;
    unsigned long state = simple_strtoul(buf, &after, 10);
    size_t count = after - buf;
    u8 val;

    if (isspace(*after))
	count++;

    if (count == size) {
	ret = count;

	if (state == 1) {
            gbq2589x_update_bits(BQ2589X_REG_0A, BQ2589X_RESERVED_MASK,
                    BQ2589X_RESERVED_ENABLE << BQ2589X_RESERVED_SHIFT);
        } else {
            gbq2589x_update_bits(BQ2589X_REG_0A, BQ2589X_RESERVED_MASK,
                    BQ2589X_RESERVED_DISABLE << BQ2589X_RESERVED_SHIFT);
        }
    }

    return ret;
}

static ssize_t vehicle_charger_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;

    sprintf(buf, "%d\n", vehicle_charger);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t vehicle_charger_store(struct device *dev, struct device_attribute *attr,
                              const char *buf, size_t size)
{
    ssize_t ret = -EINVAL;
    char *after;
    unsigned long state = simple_strtoul(buf, &after, 10);
    size_t count = after - buf;

    if (isspace(*after))
	count++;

    if (count == size) {
	ret = count;

	if (state == 1) {
            vehicle_charger = 1;
            gbq2589x_disable_charger(false);
        } else {
            vehicle_charger = 0;
            gbq2589x_disable_charger(true);
        }
    }

    return ret;
}

static DEVICE_ATTR(input_cur_limit, S_IRUGO, input_cur_limit_show, NULL);
static DEVICE_ATTR(en_charge, S_IRUGO, en_charge_show, NULL);
static DEVICE_ATTR(fast_charge_cur_limit, S_IRUGO, fast_charge_cur_limit_show, NULL);
static DEVICE_ATTR(charge_vol_limit, S_IRUGO, charge_vol_limit_show, NULL);
static DEVICE_ATTR(vbus, S_IRUGO, vbus_show, NULL);
static DEVICE_ATTR(charge_state, S_IRUGO, charge_state_show, NULL);
static DEVICE_ATTR(reg_0c, S_IRUGO, reg_0c_show, NULL);
static DEVICE_ATTR(input_vol_limit, S_IRUGO, input_vol_limit_show, NULL);
static DEVICE_ATTR(battery_vol, S_IRUGO, battery_vol_show, NULL);
static DEVICE_ATTR(sys_voltage, S_IRUGO, sys_voltage_show, NULL);
static DEVICE_ATTR(ts_percentage, S_IRUGO, ts_percentage_show, NULL);
static DEVICE_ATTR(vbus_vol, S_IRUGO, vbus_vol_show, NULL);
static DEVICE_ATTR(charge_limit, S_IRUGO, charge_limit_show, NULL);
static DEVICE_ATTR(vehicle_mode, 0664, vehicle_mode_show, vehicle_mode_store);
static DEVICE_ATTR(vehicle_charger, 0664, vehicle_charger_show, vehicle_charger_store);
static DEVICE_ATTR(thermal_debug,0664,thermal_show,NULL);  

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


static ssize_t device_info_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    return 0;
}
static DEVICE_ATTR(info, S_IRUGO, device_info_show, NULL);

struct sys_msg{
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

    if (sscanf(buf, "0x%x 0x%x %d %d %d\n",\
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
    char rbuf[256];

#if 0
    if(!msg.read_write){
        sprintf(buf,"TC READ: set read addr add read_write flag fist : %d\n",msg.read_write);
        ret = strlen(buf) + 1;
        return ret;
    }
#endif

    do{
        DEVICE_REG_READ(reg,&msg.value);
        sprintf(rbuf,"READ:reg 0X%04X value 0X%08X\n",reg,msg.value );
        strcat(buf,rbuf);

        reg++;

    }while(reg<REG_MAX);

    ret = strlen(buf) + 1;
    return ret;

}
static DEVICE_ATTR(reg, S_IALLUGO, read_reg_show, send_cmd_store);



static struct attribute *sysfs_attrs_ctrl[] = {
    &dev_attr_debug.attr,
    &dev_attr_info.attr,
    &dev_attr_reg.attr,
    &dev_attr_input_cur_limit.attr,
    &dev_attr_en_charge.attr,
    &dev_attr_fast_charge_cur_limit.attr,
    &dev_attr_charge_vol_limit.attr,
    &dev_attr_vbus.attr,
    &dev_attr_charge_state.attr,
    &dev_attr_reg_0c.attr,
    &dev_attr_input_vol_limit.attr,
    &dev_attr_battery_vol.attr,
    &dev_attr_sys_voltage.attr,
    &dev_attr_ts_percentage.attr,
    &dev_attr_vbus_vol.attr,
    &dev_attr_charge_limit.attr,
    &dev_attr_vehicle_mode.attr,
    &dev_attr_vehicle_charger.attr,
    &dev_attr_thermal_debug.attr,
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
