/*
 * ct7302.c  --  audio driver for ct7302
 *
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <linux/gpio.h>
#include <linux/unaligned/le_byteshift.h>

#define BQ27_I2C_RATE   200*1000

#define CT7302_REG_VOLUM_L 0x09
#define CT7302_REG_VOLUM_R 0x0A
#define CT7302_REG_OUTPUT  0x10
#define CT7302_REG_SAMPLE  0x05

static int snd_soc_get_volsw_bridge(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
static int snd_soc_put_volsw_bridge(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_get_enum_double_bridge(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_put_enum_double_bridge(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
struct ct7302_priv {
	struct device 		*dev;
	struct i2c_client *i2c;
	int rstn;
	int rstn_tmp;
};
struct ct7302_priv ct7302_i2c;

struct reg_table {
    u8 reg;
    u8 data;
};

struct ct7302_priv * gct7302;

static struct reg_table ct7302_init_table[] ={

    {0x11,0x00},
    {0x13,0x00},
    {0x14,0x40},
    {0x30,0x23},
    {0x31,0x19},
    {0x39,0xF3},
    {0x3B,0xFF},
    {0x40,0x02},
    {0x45,0x00},
    {0x47,0xA4},
    {0x4D,0x07},//disable shift gain 0db
    {0x4E,0x72},
    {0x61,0x08},
    {0x62,0x01},
    {0x11,0x00},
    {0x06,0x48},
    /* {0x04,0x66}, */
    {0x04,0x36}, // mode 3
    /* {0x10,0xDA}, // enable pcm out */
    {0x10,0xD8}, // enable pcm out
    /* {0x10,0xD7}, // enable spdif out */

    {0x08,0x00},// Volume 0db
    {0x09,0x00},
    {0x0A,0x00},
    {0x5E,0x04},// pcm to dsd gain

    /* {0x08,0x5A}, */
};


static int ct7302_read_i2c(struct i2c_client *client, u8 reg, bool single)
{
	/* struct i2c_client *client = to_i2c_client(di->dev); */
	/* struct i2c_client *client = di->i2c; */
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
    msg[0].scl_rate=BQ27_I2C_RATE;
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
    msg[1].scl_rate=BQ27_I2C_RATE;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

    if (!single)
        ret = get_unaligned_le16(data);
    else
		ret = data[0];

	return ret;
}

int gct7302_read_i2c(u8 reg, u8* data)
{
    int ret;
    ret=ct7302_read_i2c(gct7302->i2c,reg,true);
    if(data)
        *data=ret;

    return ret;
}
EXPORT_SYMBOL_GPL(gct7302_read_i2c);

static int ct7302_write_i2c(struct i2c_client *client, u8 reg, int value, bool single)
{
	/* struct i2c_client *client = to_i2c_client(di->dev); */
	/* struct i2c_client *client = di->i2c; */
	struct i2c_msg msg;
	unsigned char data[4];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	data[0] = reg;
	if (single) {
		data[1] = (unsigned char)value;
		msg.len = 2;
	} else {
        put_unaligned_le16(value, &data[1]);
		msg.len = 3;
	}

	msg.buf = data;
	msg.addr = client->addr;
	msg.flags = 0;
    msg.scl_rate=BQ27_I2C_RATE;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return 0;
}

int gct7302_write_i2c(u8 reg, u8 data)
{
    int ret;
    ret=ct7302_write_i2c(gct7302->i2c,reg,data,true);

    return ret;
}
EXPORT_SYMBOL_GPL(gct7302_write_i2c);

static int ct7302_read_i2c_blk(struct i2c_client *client, u8 reg,
	u8 *data, u8 len)
{
	/* struct i2c_client *client = to_i2c_client(di->dev); */
	/* struct i2c_client *client = di->i2c; */
	struct i2c_msg msg[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = 1;
    msg[0].scl_rate=BQ27_I2C_RATE;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = len;
    msg[1].scl_rate=BQ27_I2C_RATE;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return ret;
}
int gct7302_read_i2c_blk(u8 reg, u8* data,u8 len)
{
    int ret;
    ret=ct7302_read_i2c_blk(gct7302->i2c,reg,data,len);

    return ret;
}
EXPORT_SYMBOL_GPL(gct7302_read_i2c_blk);


static int ct7302_write_i2c_blk(struct i2c_client *client, u8 reg,
	u8 *data, u8 sz)
{
	/* struct i2c_client *client = to_i2c_client(di->dev); */
	/* struct i2c_client *client = di->i2c; */
	struct i2c_msg msg;
	int ret;
	u8 buf[33];

	if (!client->adapter)
		return -ENODEV;

	buf[0] = reg;
	memcpy(&buf[1], data, sz);

	msg.buf = buf;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sz + 1;
    msg.scl_rate=BQ27_I2C_RATE;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return 0;
}

/**
 * snd_soc_update_bits - update codec register bits
 * @codec: audio codec
 * @reg: codec register
 * @mask: register mask
 * @value: new value
 *
 * Writes new register value.
 *
 * Returns 1 for change, 0 for no change, or negative error code.
 */
int ct7302_update_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned int mask, unsigned int value)
{
	int change;
	unsigned int old, new;
	int ret;

	/* ret = snd_soc_read(codec, reg); */
    ret = gct7302_read_i2c(reg,NULL);
	if (ret < 0)
		return ret;

	old = ret;
	new = (old & ~mask) | value;
	change = old != new;
	if (change) {
		/* ret = snd_soc_write(codec, reg, new); */
        ret = gct7302_write_i2c(reg,new);
		if (ret < 0)
			return ret;
	}

	return change;
}
EXPORT_SYMBOL_GPL(snd_soc_update_bits);


#define SOC_DOUBLE_TLV_CT7302(xname, xreg, shift_left, shift_right, xmax,\
			       xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw_bridge, \
	.put = snd_soc_put_volsw_bridge, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .shift = shift_left, .rshift = shift_right,\
		 .max = xmax, .invert = xinvert} }

#define SOC_SINGLE_TLV_CT7302(xname, xreg, xshift, xmax, xinvert, tlv_array) \
	SOC_DOUBLE_TLV_CT7302(xname, xreg, xshift, xshift, xmax, \
			       xinvert, tlv_array)

#define SOC_ENUM_CT7302(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_enum_double_bridge, .put = snd_soc_put_enum_double_bridge, \
	.private_value = (unsigned long)&xenum }


/*
 * Capture gain after the ADCs
 * from 0 dB to 31 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(digital_volume_tlv, -12750, 50, 0);

static const char *ct7302_spdif_out_enable[] = {"ON", "OFF"};
static const char *ct7302_pcm_out_enable[] = {"ON", "OFF"};

static const struct soc_enum ct7302_enum[] = {
	SOC_ENUM_SINGLE(CT7302_REG_OUTPUT, 3, 2, ct7302_spdif_out_enable),
	SOC_ENUM_SINGLE(CT7302_REG_OUTPUT, 2, 2, ct7302_pcm_out_enable),
};

static const struct snd_kcontrol_new ct7302_snd_controls[] = {

	SOC_SINGLE_TLV_CT7302("BRIDGE Playback Volume L",
		CT7302_REG_VOLUM_L, 0, 0XFF, 1, digital_volume_tlv),
	SOC_SINGLE_TLV_CT7302("BRIDGE Playback Volume R",
		CT7302_REG_VOLUM_R, 0, 0XFF, 1, digital_volume_tlv),



	SOC_ENUM_CT7302("BRIDGE AudioOutput SPDIF",ct7302_enum[0]),
	SOC_ENUM_CT7302("BRIDGE AudioOutput PCM"  ,ct7302_enum[1]),



};

struct snd_kcontrol_new* get_audiobirdge_control_table(int* num_controls)
{
    *num_controls=ARRAY_SIZE(ct7302_snd_controls);
    return ct7302_snd_controls;

}
EXPORT_SYMBOL_GPL(get_audiobirdge_control_table);


static int snd_soc_get_volsw_bridge(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;

	ucontrol->value.integer.value[0] =
        (ct7302_read_i2c(gct7302->i2c,reg,true)>> shift) & mask;
		/* (snd_soc_read(codec, reg) >> shift) & mask; */
	if (ucontrol->value.integer.value[0])
		ucontrol->value.integer.value[0] =
			max + 1 - ucontrol->value.integer.value[0];

	if (shift != rshift) {
		ucontrol->value.integer.value[1] =
            (ct7302_read_i2c(gct7302->i2c,reg,true)>> shift) & mask;
			/* (snd_soc_read(codec, reg) >> rshift) & mask; */
		if (ucontrol->value.integer.value[1])
			ucontrol->value.integer.value[1] =
				max + 1 - ucontrol->value.integer.value[1];
	}

	return 0;
}

static int snd_soc_put_volsw_bridge(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;
	unsigned short val, val2, val_mask;

	val = (ucontrol->value.integer.value[0] & mask);

	val_mask = mask << shift;
	if (val)
		val = max + 1 - val;
	val = val << shift;
	if (shift != rshift) {
		val2 = (ucontrol->value.integer.value[1] & mask);
		val_mask |= mask << rshift;
		if (val2)
			val2 = max + 1 - val2;
		val |= val2 << rshift;
	}
    return ct7302_update_bits(codec,reg,val_mask,val);
	/* return snd_soc_update_bits(codec, reg, val_mask, val); */
}

int snd_soc_get_enum_double_bridge(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val, bitmask;

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
    val=ct7302_read_i2c(gct7302->i2c,e->reg,true);
	/* val = snd_soc_read(codec, e->reg); */
	ucontrol->value.enumerated.item[0]
		= (val >> e->shift_l) & (bitmask - 1);
    printk("ct7302  reg %x val %x  value %x \n",e->reg,val,ucontrol->value.enumerated.item[0]);
	if (e->shift_l != e->shift_r)
		ucontrol->value.enumerated.item[1] =
			(val >> e->shift_r) & (bitmask - 1);

	return 0;
}

int snd_soc_put_enum_double_bridge(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val;
	unsigned int mask, bitmask;

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;
	val = ucontrol->value.enumerated.item[0] << e->shift_l;
	mask = (bitmask - 1) << e->shift_l;
	if (e->shift_l != e->shift_r) {
		if (ucontrol->value.enumerated.item[1] > e->max - 1)
			return -EINVAL;
		val |= ucontrol->value.enumerated.item[1] << e->shift_r;
		mask |= (bitmask - 1) << e->shift_r;
	}

    return ct7302_update_bits(codec,e->reg,mask,val);
	/* return snd_soc_update_bits_locked(codec, e->reg, mask, val); */
}

void ct7302_rate_set(int rate)
{

    int data;
    u8 dat;
    printk(KERN_WARNING "[CT7302] %s rate=%d\n", __func__, rate);
    data=gct7302_read_i2c(CT7302_REG_SAMPLE, NULL);
    dat=((u8)data) & 0xF0;
    if(data<0){
        printk(KERN_ERR "ct7302_i2c_read err %d set samplerate %d Failed\n",data,rate);
        return;
    }
    switch(rate) {
		case 384000: // 192K 
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x0C  );break;
		case 352800:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x0B  );break;
		case 256000:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x0A  );break;
		case 192000:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x09  );break;
		case 176400:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x08  );break;
		case 128000:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x07  );break;
		case 96000:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x06  );break;
		case 88200:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x05  );break;
		case 64000:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x04  );break;
		case 48000:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x03  );break;
		case 44100:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x02  );break;
		case 32000:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x01  );break;
		default:
            gct7302_write_i2c(CT7302_REG_SAMPLE, dat | 0x02  );break;
            printk(KERN_ERR "ct7302 can't set Sampling rate %d \n",rate);
			break;
	}
    printk("ct7302 reg 0x05 old 0x%x  new 0x%x \n",data,gct7302_read_i2c(CT7302_REG_SAMPLE, NULL));
	/* msleep(5); */
}

EXPORT_SYMBOL_GPL(ct7302_rate_set);

static int ct7302_config_init(struct i2c_client *client)
{

	int ret = 0,ret1=0,ret2=0;
    int i,tsize;
    /* ret1=ct7302_write_i2c(client, 0x08,0x5A,true); */


    /* ret2=ct7302_read_i2c(client,0x08,true); */

	/* printk(KERN_ERR "[LHS]%s ret1 = %0x,ret2 = %0x\n",__func__,ret1,ret2); */
    tsize=sizeof(ct7302_init_table)/sizeof(struct reg_table );
    printk(KERN_ERR "ct7302 tsize is %d \n",tsize);

    for(i=0;i<tsize;i++){
        printk(KERN_ERR "ct7302 tsize iss %d \n",tsize);
        ret=ct7302_write_i2c(client,ct7302_init_table[i].reg,ct7302_init_table[i].data,true);
        printk(KERN_ERR "ct7302 write size %d ret %d  reg 0x%02x  value 0x%02x\n",tsize,ret,ct7302_init_table[i].reg,ct7302_init_table[i].data);

    }

        printk(KERN_ERR "ct7302 tsize isss %d \n",tsize);
    for(i=0;i<tsize;i++){
       ret2=ct7302_read_i2c(client,ct7302_init_table[i].reg,true);
       if(ret2!=ct7302_init_table[i].data)
           printk(KERN_ERR "[LHS] reg 0x%x  write 0x%x read 0x%x",ct7302_init_table[i].reg,ct7302_init_table[i].data,ret2);
    }

#if 0

    msleep(50);

    ret1=ct7302_write_i2c(client, 0x04,0x66,true);
    ret2=ct7302_write_i2c(client, 0x06,0x48,true);

    

    i=10;
    do{
        msleep(50);

        ret1=ct7302_read_i2c(client,0x04,true);
        ret2=ct7302_read_i2c(client,0x06,true);
        printk(KERN_ERR "[LHS]%s ret1 = %0x,ret2 = %0x\n",__func__,ret1,ret2);

        ret1=ct7302_write_i2c(client, 0x06,0x48,true);
    }while(ret2!=0x48 && i-->0);

#endif

    /* ret1=ct7302_write_i2c(client, 0x10,0xDA,true); */


	/* ret = ct7302_i2c_write(client,0x00,0xe7); */

	/* ret1 = ct7302_i2c_read(client,0x00); */
	/* ret = ct7302_i2c_write(client,0x01,0x00); */

	/* ret2= ct7302_i2c_read(client,0x01); */
	return 0;

}

static int ct7302_i2c_test(struct i2c_client *i2c)
{
        int ret;
        int count = 5;

        do {
	    ret = ct7302_read_i2c(i2c, 0x11, true);
	    if (ret < 0) {
                dev_err(&i2c->dev, "%s:  error  !!!\n", __func__);
	    }
            count--;
        } while(count > 0);
	
        return ret;
}

static int ct7302_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	struct ct7302_priv *ct7302;
	int ret=0;

	pr_err("%s enter\n",__func__);
	printk(KERN_ERR "%s enter\n",__func__);

        ret = ct7302_i2c_test(i2c);
        if (ret < 0) {
            printk(KERN_ERR "%s error !!!\n", __func__);
            return ret;
        }

	ct7302 = kzalloc(sizeof(struct ct7302_priv), GFP_KERNEL);
	if (ct7302 == NULL) 
		return -ENOMEM;

    	ct7302->i2c = i2c;
	i2c_set_clientdata(i2c, ct7302);
	gct7302=ct7302;
#if 0
    ct7302->rstn = of_get_named_gpio(i2c->dev.of_node, 
					      "ct7302,rstn-gpio", 0);
	if (!gpio_is_valid(ct7302->rstn)){  
		pr_err("%s error ct7302 rstn ret=%d\n", __func__, ct7302->rstn);
		return -EINVAL;
	}

	ret = gpio_request(ct7302->rstn, "ct7302_rstn_pin");
	if (ret < 0) {
		pr_err("%s(): ct7302_rstn_pin request failed %d\n",
				__func__, ret);
		return ret;
	}

	gpio_direction_output(ct7302->rstn, 0);
	usleep_range(1000, 1005);
	gpio_direction_output(ct7302->rstn, 1);
    	pr_err("[LHS]%s(): reset success! \n",__func__);
#endif

#if 0
    ct7302->rstn_tmp = of_get_named_gpio(i2c->dev.of_node, 
					      "ct7302,rstn-gpio-tmp", 0);
	if (!gpio_is_valid(ct7302->rstn_tmp)){  
		pr_err("%s error ct7302 rstn_tmp ret=%d\n", __func__, ct7302->rstn_tmp);
		return -EINVAL;
	}

	ret = gpio_request(ct7302->rstn_tmp, "ct7302_rstn_pin_tmp");
	if (ret < 0) {
		pr_err("%s(): ct7302_rstn_pin_tmp request failed %d\n",
				__func__, ret);
		return ret;
	}

	gpio_direction_output(ct7302->rstn_tmp, 0);
	usleep_range(1000, 1005);
	gpio_direction_output(ct7302->rstn_tmp, 1);
    	pr_err("[LHS]%s(): reset tmp success! \n",__func__);
#endif

    ct7302_config_init(i2c);
    ct7302_i2c.i2c= i2c;
	ct7302_i2c.rstn=ct7302->rstn;
	ct7302_i2c.rstn_tmp=ct7302->rstn_tmp;
	return ret;
}


void ct7302_clock_open(void)
{
	int ret;
	pr_err("%s() !\n",__func__);
	/* ret=ct7302_i2c_write(ct7302_i2c.i2c,0x00,0x00); */
}

void ct7302_clock_close(void)
{
	int ret;
	pr_err("%s() !\n",__func__);
	/* ret = ct7302_i2c_write(ct7302_i2c.i2c,0x00,0xe7); */
}


static int ct7302_i2c_remove(struct i2c_client *i2c)
{
	kfree(i2c_get_clientdata(i2c));

	return 0;
}

static const struct i2c_device_id ct7302_i2c_id[] = {
	{ "ct7302", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ct7302_i2c_id);

static struct i2c_driver ct7302_i2c_driver = {
	.driver = {
		.name = "ct7302",
		.owner = THIS_MODULE,
	},
	.probe = ct7302_i2c_probe,
	.remove = ct7302_i2c_remove,
	.id_table = ct7302_i2c_id,
};

static int __init ct7302_init(void)
{

	pr_info("%s \n",__func__);

	return i2c_add_driver(&ct7302_i2c_driver);
}

module_init(ct7302_init);

static void __exit ct7302_exit(void)
{
	i2c_del_driver(&ct7302_i2c_driver);
}
module_exit(ct7302_exit);

MODULE_DESCRIPTION("ASoC ct7302 codec driver");
MODULE_LICENSE("GPL");

