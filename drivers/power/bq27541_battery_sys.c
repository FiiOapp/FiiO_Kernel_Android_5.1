/*
 * BQ2754X battery charging debug sys
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

static struct kobject * bq27541_kobj;
extern int bq27541_read_reg(u8 reg);

static ssize_t unfiltered_soc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x04);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t temp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x06);
        value = (value - 2731);
	sprintf(buf, "%d.%d\n", value/10, value%10);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t voltage_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x08);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t flags_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x0A);
	sprintf(buf, "0x%04x\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t remaining_cap_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x10);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t fullcharge_cap_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x12);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t average_cur_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x14);
	if(value > 0x8000){
		value = 0xFFFF^(value - 1);
	}
	//value = value * 1000;

	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t fullcharge_cap_filtered_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x18);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t safety_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x1A);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t fullcharge_cap_unfiltered_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x1C);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t remaining_cap_unfiltered_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x20);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t remaining_cap_filtered_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x22);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t cycle_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x2A);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t state_of_charge_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x2C);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t state_of_health_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x2E);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t passed_charge_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x34);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t dod0_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x36);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t pack_config_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x3A);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t dodat_eoc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x62);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t q_start_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
        int value;
 
        value = bq27541_read_reg(0x64);
	sprintf(buf, "%d\n", value);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(unfiltered_soc, 0444, unfiltered_soc_show, NULL);
static DEVICE_ATTR(temp, 0444, temp_show, NULL);
static DEVICE_ATTR(voltage, 0444, voltage_show, NULL);
static DEVICE_ATTR(flags, 0444, flags_show, NULL);
static DEVICE_ATTR(remaining_cap, 0444, remaining_cap_show, NULL);
static DEVICE_ATTR(fullcharge_cap, 0444, fullcharge_cap_show, NULL);
static DEVICE_ATTR(average_cur, 0444, average_cur_show, NULL);
static DEVICE_ATTR(fullcharge_cap_filtered, 0444, fullcharge_cap_filtered_show, NULL);
static DEVICE_ATTR(safety_status, 0444, safety_status_show, NULL);
static DEVICE_ATTR(fullcharge_cap_unfiltered, 0444, fullcharge_cap_unfiltered_show, NULL);
static DEVICE_ATTR(remaining_cap_unfiltered, 0444, remaining_cap_unfiltered_show, NULL);
static DEVICE_ATTR(remaining_cap_filtered, 0444, remaining_cap_filtered_show, NULL);
static DEVICE_ATTR(cycle_count, 0444, cycle_count_show, NULL);
static DEVICE_ATTR(state_of_charge, 0444, state_of_charge_show, NULL);
static DEVICE_ATTR(state_of_health, 0444, state_of_health_show, NULL);
static DEVICE_ATTR(passed_charge, 0444, passed_charge_show, NULL);
static DEVICE_ATTR(dod0, 0444, dod0_show, NULL);
static DEVICE_ATTR(pack_config, 0444, pack_config_show, NULL);
static DEVICE_ATTR(dodat_eoc, 0444, dodat_eoc_show, NULL);
static DEVICE_ATTR(q_start, 0444, q_start_show, NULL);

static struct attribute *sysfs_attrs_ctrl[] = {
    &dev_attr_unfiltered_soc.attr,
    &dev_attr_temp.attr,
    &dev_attr_voltage.attr,
    &dev_attr_flags.attr,
    &dev_attr_remaining_cap.attr,
    &dev_attr_fullcharge_cap.attr,
    &dev_attr_average_cur.attr,
    &dev_attr_fullcharge_cap_filtered.attr,
    &dev_attr_safety_status.attr,
    &dev_attr_fullcharge_cap_unfiltered.attr,
    &dev_attr_remaining_cap_unfiltered.attr,
    &dev_attr_remaining_cap_filtered.attr,
    &dev_attr_cycle_count.attr,
    &dev_attr_state_of_charge.attr,
    &dev_attr_state_of_health.attr,
    &dev_attr_passed_charge.attr,
    &dev_attr_dod0.attr,
    &dev_attr_pack_config.attr,
    &dev_attr_dodat_eoc.attr,
    &dev_attr_q_start.attr,
    NULL
};

static struct attribute_group device_attribute_group[] = {
    {.attrs = sysfs_attrs_ctrl },
};

extern bool is_bq27541_flag;

static int bq27541_sys_init(void)
{
    int ret;

    if (!is_bq27541_flag) {
        printk(KERN_ERR " %s : bq27541 i2c trans failed, don`t create bq27541_debug class!\n", __func__);
        return -ENOMEM;
    }

    bq27541_kobj = kobject_create_and_add("bq27541_debug", NULL);
    if (bq27541_kobj == NULL){
        printk(KERN_ERR " %s : kobject_create_and_add failed\n", __func__);
        return -ENOMEM;
    }

    ret = sysfs_create_group(bq27541_kobj, device_attribute_group);
    if (ret < 0) {
        printk(KERN_ERR "%s: sysfs_create_group failed\n", __func__);
        goto err;
    }

    return 0;

err:
    kobject_del(bq27541_kobj);
}

static void bq27541_sys_exit(void)
{
    if(bq27541_kobj) {
        sysfs_remove_group(bq27541_kobj, device_attribute_group);
        kobject_put(bq27541_kobj);
    }
}

late_initcall(bq27541_sys_init);
module_exit(bq27541_sys_exit);

