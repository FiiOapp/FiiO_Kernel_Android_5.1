/*
 * es9028.c -- es9028 ALSA SoC audio driver
 *
 * Copyright 2009 Wolfson Microelectronics plc
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <mach/iomux.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <linux/wakelock.h>

//#include <linux/tchip_sysinf.h>

#include <linux/switch_gpio_msp430_NP.h>
#include "fpga_audio_interface.h"
#include "es9028.h"

#include <linux/proc_fs.h>
#include <linux/gpio.h>

#include <linux/interrupt.h>
#include <linux/irq.h>

#if 1
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif
#define alsa_dbg DBG

static int debug=3;
module_param(debug, int, S_IRUGO|S_IWUSR);
#define dprintk(level, fmt, arg...) do {			\
	if (debug >= level) 					\
	printk(KERN_WARNING "ES9028 %s-%d: " fmt ,__FUNCTION__, __LINE__, ## arg); } while (0)



int mutedealy=100;
module_param(mutedealy,int,0644);
extern int opt_cox_out_flag;

static struct snd_soc_codec *es9028_codec;
static fpga_fmt_t fpga_fmt;

/* static struct timer_list spk_timer; */
/* struct delayed_work spk_work; */
/* static bool last_is_spk = false; */

static struct wake_lock audiolink_wakelock;
static bool gReWriteSensitiveMsg=true;
#define AUDIO_LINK_SHUTDOWN_DELAY 5

#define DAC_RESET  RK30_PIN3_PD5 
//#define SPK_CTL             RK29_PIN6_PB6
//#define EAR_CON_PIN             RK29_PIN6_PB5
#undef EAR_CON_PIN

#ifndef es9028_DEF_VOL
#define es9028_DEF_VOL			0x1e
#endif

static int es9028_set_bias_level(struct snd_soc_codec *codec,enum snd_soc_bias_level level);
extern int es9028_dapm_pre_event(struct snd_soc_dapm_widget* widget, struct snd_kcontrol * null, int event);
extern int es9028_dapm_post_event(struct snd_soc_dapm_widget* widget, struct snd_kcontrol * null, int event);                                

extern void AudioBridge_enable(void);
extern void AudioBridge_disable(void);
extern int HW_tpye;

#define HW_TYPE_UNKNOWN -1
#define HW_TYPE_OSC_2IN 0
#define HW_TYPE_OSC_3IN 1
/*
 * es9028 register cache
 * We can't read the es9028 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
#define ES9028_CACHEREGNUM  32
static u8 es9028_reg[ES9028_CACHEREGNUM] = {

    0X00, 0X0C, 0X3C, 0X00,/*  0 */
    0X00, 0X68, 0X4A, 0X40,/*  4 */ 
    0X88, 0X88, 0X00, 0X00,/*  8 */ 
    0X5A, 0X20, 0X8A, 0X00,/* 12 */ 
    0X00, 0X00, 0X00, 0X00,/* 16 */ 
    0X00, 0X00, 0X00, 0X00,/* 20 */ 
    0XFF, 0XFF, 0XFF, 0X7F,/* 24 */ 
    0X00, 0X00, 0X00, 0X00,/* 28 */ 
      
};

/* codec private data */
struct es9028_priv {
    unsigned int sysclk;
    enum snd_soc_control_type control_type;
    struct snd_pcm_hw_constraint_list *sysclk_constraints;
    int is_startup;		// gModify.Add
    int is_biason;
};


#if 0
static unsigned int es9028_hw_read_reg_cache(struct snd_soc_codec *codec,
                                          unsigned int reg)
{
    //u16 *cache = codec->reg_cache;
    if (reg >= ARRAY_SIZE(es9028_reg))
        return -1;
    return es9028_reg[reg];
}
#endif

static inline unsigned int es9028_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 *cache = codec->reg_cache;
	if (reg >= ES9028_CACHEREGNUM)
		return -1;
	return cache[reg];
}

static inline void es9028_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u8 *cache = codec->reg_cache;
	if (reg >= ES9028_CACHEREGNUM)
		return;
	cache[reg] = value;
}
static unsigned int es9028_hw_read(struct snd_soc_codec *codec, unsigned int reg)
{
    u8 data[2];
    int ret;

    if(!link_get_status(LINK_REPORT_DAC_PWR)){
        return es9028_read_reg_cache(codec,reg);
    }


    BUG_ON(codec->volatile_register);
    data[0] = reg;
    ret=i2c_master_reg8_recv(codec->control_data,reg,data,1,400*1000);
    /* dprintk(2,"ret %d reg 0x%02x buf 0x%02x  \n",ret,reg,data[0]); */
    if (ret == 1)
        return data[0];
    if (ret < 0)
        return es9028_read_reg_cache(codec,reg);
        /* return ret; */
    else
        return -EIO;
}
static int es9028_hw_write(struct snd_soc_codec *codec, unsigned int reg,
                        unsigned int value)
{
    u8 data[2];
    int ret;


    es9028_write_reg_cache(codec,reg,value);
    if(!link_get_status(LINK_REPORT_DAC_PWR)){
        gReWriteSensitiveMsg=true;
        dprintk(0,"LINK DAC PWR is not opened!! reg: 0x%02x=0x%02x\n",reg,value); 
        return 0;
    }

    BUG_ON(codec->volatile_register);

    data[0] = reg;
    data[1] = value & 0x00ff;


    ret = codec->hw_write(codec->control_data, data, 2);
    if (ret == 2)
        return 0;
    if (ret < 0)
        return ret;
    else
        return -EIO;
}

int es9028_i2c_master_send(const struct i2c_client *client, const char *buf, int count)
{
    int ret;

   ret=i2c_master_send(client,buf,count);

   /* dprintk(2,"ret %d reg: 0x%02x data: 0x%02x \n",ret,buf[0],buf[1]); */
   return ret;
}

int es9028_update_bits(unsigned short reg, unsigned int mask, unsigned int value)
{
	int change;
	unsigned int old, new;
	int ret;

    ret=es9028_hw_read(es9028_codec,reg);
	if (ret < 0)
       ret=es9028_read_reg_cache(es9028_codec,reg);
		/* return ret; */

	old = ret;
	new = (old & ~mask) | value;
    dprintk(0,"reg 0x%x old %x new %x \n",reg,old,new);
	change = old != new;
	if (change) {
        ret=es9028_hw_write(es9028_codec,reg,new);
		if (ret < 0)
			return ret;
	}

	return change;
}



// for extern Function
int es9028_read_i2c_blk(u8 reg, u8* data,u8 len)
{
    int ret=0;
    int i;


    // not support continue read
    /* ret=i2c_master_reg8_recv(es9028_codec->control_data,reg,data,1,100*1000); */
    for(i=0;i<len;i++){
        ret=i2c_master_reg8_recv(es9028_codec->control_data,reg+i,data+i,1,400*1000);
    }

    return ret;

}

int es9028_write_i2c(u8 reg,u8 data)
{
    if(!es9028_codec)
        return -1;
    return es9028_hw_write(es9028_codec,reg,data);
}

//#define es9028_reset(c)	snd_soc_write(c, es9028_RESET, 0)
static int es9028_reset(struct snd_soc_codec *codec)
{
    int reg;
    // for mclk div,reg 0 is a special reg
    // soft reset will cause mclk div loss,cache before reset
    reg = es9028_read_reg_cache(es9028_codec,ES9028_CONTROL1);
    /* printk(" %s cache 0x%x \n",__FUNCTION__,reg); */
    /* return snd_soc_write(codec, ES9028_CONTROL1, 0x01); */
    snd_soc_update_bits(codec, ES9028_CONTROL1, 0x01,0x01);

    // restore reg0 to cache,this reg will write to hardware later.
    es9028_write_reg_cache(es9028_codec,ES9028_CONTROL1,reg);
    /* printk(" %s cache 0x%x \n",__FUNCTION__,es9028_read_reg_cache(es9028_codec,ES9028_CONTROL1)); */

    return 0;
}

void es9028_set_thdc2(u32 data)
{

    u8 c2_l=(u8)(data&0xFF);
    u8 c2_h=(u8)((data>>8)&0xFF);
    es9028_hw_write(es9028_codec,0x1c,c2_l);
    es9028_hw_write(es9028_codec,0x1d,c2_h);

}
void es9028_set_thdc3(u32 data)
{

    u8 c3_l=(u8)(data&0xFF);
    u8 c3_h=(u8)((data>>8)&0xFF);
    es9028_hw_write(es9028_codec,0x1e,c3_l);
    es9028_hw_write(es9028_codec,0x1f,c3_h);

}

u8 es9028_get_lpf(void)
{
    int ret;

    ret=es9028_hw_read(es9028_codec,ES9028_FILTER );
	if (ret < 0){
       ret=es9028_read_reg_cache(es9028_codec,ES9028_FILTER );
    }
    printk("%s lpf %d\n",__FUNCTION__,ret);
		return (u8)ret;
}
void es9028_set_lpf(u8 lpf)
{

    printk("%s lpf %d\n",__FUNCTION__,lpf);
    es9028_update_bits(ES9028_FILTER ,0xE0,lpf<<5);

}

static int es9028_get_lpf_control(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	/* struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec); */

    ucontrol->value.enumerated.item[0] = es9028_get_lpf();
    return 0;
}

static int es9028_set_lpf_control(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    es9028_set_lpf((u8)ucontrol->value.enumerated.item[0]);

    	return 0;
}
int snd_soc_get_volsw_lpf(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = es9028_get_lpf();

	return 0;
}

int snd_soc_put_volsw_lpf(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);


    es9028_set_lpf((u8)ucontrol->value.integer.value[0]);

	return 0;
}
#define SOC_ENUM_EXT_ES9028(xname, xenum, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_ext, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&xenum }

#define SOC_SINGLE_EXT_ES9028(xname, xreg, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmax, xinvert) }




static const char *es9028_line_texts[] = {
    "Line 1", "Line 2", "PGA"};

static const unsigned int es9028_line_values[] = {
    0, 1, 3};
static const char *es9028_pga_sel[] = {"Line 1", "Line 2", "Differential"};
static const char *stereo_3d_txt[] = {"No 3D  ", "Level 1","Level 2","Level 3","Level 4","Level 5","Level 6","Level 7"};
static const char *alc_func_txt[] = {"Off", "Right", "Left", "Stereo"};
static const char *ng_type_txt[] = {"Constant PGA Gain","Mute ADC Output"};
static const char *deemph_txt[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static const char *adcpol_txt[] = {"Normal", "L Invert", "R Invert","L + R Invert"};
static const char *es9028_mono_mux[] = {"Stereo", "Mono (Left)","Mono (Right)"};
static const char *es9028_diff_sel[] = {"Line 1", "Line 2"};

static const struct soc_enum es9028_enum__[]={	
    SOC_VALUE_ENUM_SINGLE(ES9028_DACCONTROL16, 3, 7, ARRAY_SIZE(es9028_line_texts), es9028_line_texts, es9028_line_values),/* LLINE */
    SOC_VALUE_ENUM_SINGLE(ES9028_DACCONTROL16, 0, 7, ARRAY_SIZE(es9028_line_texts), es9028_line_texts, es9028_line_values),/* rline	*/
    SOC_VALUE_ENUM_SINGLE(ES9028_ADCCONTROL2, 6, 3, ARRAY_SIZE(es9028_pga_sel), es9028_line_texts, es9028_line_values),/* Left PGA Mux */
    SOC_VALUE_ENUM_SINGLE(ES9028_ADCCONTROL2, 4, 3, ARRAY_SIZE(es9028_pga_sel), es9028_line_texts, es9028_line_values),/* Right PGA Mux */
    SOC_ENUM_SINGLE(ES9028_DACCONTROL7, 2, 8, stereo_3d_txt),/* stereo-3d */
    SOC_ENUM_SINGLE(ES9028_ADCCONTROL10, 6, 4, alc_func_txt),/*alc func*/
    SOC_ENUM_SINGLE(ES9028_ADCCONTROL14, 1, 2, ng_type_txt),/*noise gate type*/
    SOC_ENUM_SINGLE(ES9028_DACCONTROL6, 6, 4, deemph_txt),/*Playback De-emphasis*/
    SOC_ENUM_SINGLE(ES9028_ADCCONTROL6, 6, 4, adcpol_txt),
    SOC_ENUM_SINGLE(ES9028_ADCCONTROL3, 3, 3, es9028_mono_mux),
    SOC_ENUM_SINGLE(ES9028_ADCCONTROL3, 7, 2, es9028_diff_sel),
};






static const DECLARE_TLV_DB_SCALE(pga_tlv, 0, 300, 0);
static const DECLARE_TLV_DB_SCALE(adc_tlv, -9600, 50, 1);
/* static const DECLARE_TLV_DB_SCALE(dac_tlv, -9600, 50, 1); */
static const DECLARE_TLV_DB_SCALE(out_tlv, -4500, 150, 0);
static const DECLARE_TLV_DB_SCALE(bypass_tlv, -1500, 300, 0);



static const char *digital_filter_select[] = {"fast_roll-off_linear",
                                              "slow_roll-off_linear",
                                              "fast_roll-off_mini",
                                              "slow_roll-off_mini",
                                              "reserved",
                                              "apodizing_fast_roll-off_linear",
                                              "hybrid_fast_roll-off_mini"
                                              "brickwall"};

static const char *mclk_div_select[] = {"div_1",
                                        "div_2",
                                        "div_4",
                                        "div_8"};

static const struct soc_enum es9028_enum[]={	

	SOC_ENUM_SINGLE(ES9028_FILTER ,5,8, digital_filter_select),
	SOC_ENUM_SINGLE(ES9028_CONTROL1,2,4, mclk_div_select),
};

static const DECLARE_TLV_DB_SCALE(dac_tlv, -12750, 500, 0);
static const struct snd_kcontrol_new es9028_snd_controls[] = {

    SOC_SINGLE_TLV("DAC0 Playback Volume", ES9028_DAC0,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC1 Playback Volume", ES9028_DAC1,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC2 Playback Volume", ES9028_DAC2,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC3 Playback Volume", ES9028_DAC3,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC4 Playback Volume", ES9028_DAC4,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC5 Playback Volume", ES9028_DAC5,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC6 Playback Volume", ES9028_DAC6,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC7 Playback Volume", ES9028_DAC7,0, 255, 1, dac_tlv),


    /* SOC_ENUM("digital filter", es9028_enum[0]), */
	SOC_ENUM_EXT_ES9028("digital filter", es9028_enum[0], es9028_get_lpf_control, es9028_set_lpf_control),

    SOC_SINGLE_EXT_ES9028("DAC Digital Filter Mode",0x07,5,0x08,0,snd_soc_get_volsw_lpf,snd_soc_put_volsw_lpf),

    SOC_ENUM("DAC MCLK DIV", es9028_enum[1]),
    /* SOC_SINGLE("DAC MCLK DIV", ES9028_ADCCONTROL11, 4, 15, 0), */
#if 0
    SOC_ENUM("3D Mode", es9028_enum[4]),
    SOC_SINGLE("ALC Capture Target Volume", ES9028_ADCCONTROL11, 4, 15, 0),
    SOC_SINGLE("ALC Capture Max PGA", ES9028_ADCCONTROL10, 3, 7, 0),
    SOC_SINGLE("ALC Capture Min PGA", ES9028_ADCCONTROL10, 0, 7, 0),
    SOC_ENUM("ALC Capture Function", es9028_enum[5]),
    SOC_SINGLE("ALC Capture ZC Switch", ES9028_ADCCONTROL13, 6, 1, 0),
    SOC_SINGLE("ALC Capture Hold Time", ES9028_ADCCONTROL11, 0, 15, 0),
    SOC_SINGLE("ALC Capture Decay Time", ES9028_ADCCONTROL12, 4, 15, 0),
    SOC_SINGLE("ALC Capture Attack Time", ES9028_ADCCONTROL12, 0, 15, 0),
    SOC_SINGLE("ALC Capture NG Threshold", ES9028_ADCCONTROL14, 3, 31, 0),
    SOC_ENUM("ALC Capture NG Type",es9028_enum[6]),
    SOC_SINGLE("ALC Capture NG Switch", ES9028_ADCCONTROL14, 0, 1, 0),
    SOC_SINGLE("ZC Timeout Switch", ES9028_ADCCONTROL13, 6, 1, 0),
    SOC_DOUBLE_R_TLV("Capture Digital Volume", ES9028_ADCCONTROL8, ES9028_ADCCONTROL9,0, 255, 1, adc_tlv),		 
    SOC_SINGLE("Capture Mute", ES9028_ADCCONTROL7, 2, 1, 0),		
    SOC_SINGLE_TLV("Left Channel Capture Volume",	ES9028_ADCCONTROL1, 4, 15, 0, bypass_tlv),
    SOC_SINGLE_TLV("Right Channel Capture Volume",	ES9028_ADCCONTROL1, 0, 15, 0, bypass_tlv),
    SOC_ENUM("Playback De-emphasis", es9028_enum[7]),
    SOC_ENUM("Capture Polarity", es9028_enum[8]),
    SOC_DOUBLE_R_TLV("PCM Volume", ES9028_DACCONTROL4, ES9028_DACCONTROL5, 0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("Left Mixer Left Bypass Volume", ES9028_DACCONTROL17, 3, 7, 1, bypass_tlv),
    SOC_SINGLE_TLV("Right Mixer Right Bypass Volume", ES9028_DACCONTROL20, 3, 7, 1, bypass_tlv),
    SOC_DOUBLE_R_TLV("Output 1 Playback Volume", ES9028_DACCONTROL24, ES9028_DACCONTROL25, 0, 64, 0, out_tlv),
    SOC_DOUBLE_R_TLV("Output 2 Playback Volume", ES9028_DACCONTROL26, ES9028_DACCONTROL27, 0, 64, 0, out_tlv),
#endif
};


static const struct snd_kcontrol_new es9028_left_line_controls =
SOC_DAPM_VALUE_ENUM("Route", es9028_enum[0]);

static const struct snd_kcontrol_new es9028_right_line_controls =
SOC_DAPM_VALUE_ENUM("Route", es9028_enum[1]);

/* Left PGA Mux */
static const struct snd_kcontrol_new es9028_left_pga_controls =
SOC_DAPM_VALUE_ENUM("Route", es9028_enum[2]);
/* Right PGA Mux */
static const struct snd_kcontrol_new es9028_right_pga_controls =
SOC_DAPM_VALUE_ENUM("Route", es9028_enum[3]);

/* Left Mixer */
static const struct snd_kcontrol_new es9028_left_mixer_controls[] = {
    SOC_DAPM_SINGLE("Left Playback Switch", ES9028_DACCONTROL17, 7, 1, 0),
    SOC_DAPM_SINGLE("Left Bypass Switch", ES9028_DACCONTROL17, 6, 1, 0),	
};

/* Right Mixer */
static const struct snd_kcontrol_new es9028_right_mixer_controls[] = {
    SOC_DAPM_SINGLE("Right Playback Switch", ES9028_DACCONTROL20, 7, 1, 0),
    SOC_DAPM_SINGLE("Right Bypass Switch", ES9028_DACCONTROL20, 6, 1, 0),
};

/* Differential Mux */
//static const char *es9028_diff_sel[] = {"Line 1", "Line 2"};
static const struct snd_kcontrol_new es9028_diffmux_controls =
SOC_DAPM_ENUM("Route", es9028_enum[10]);

/* Mono ADC Mux */
static const struct snd_kcontrol_new es9028_monomux_controls =
SOC_DAPM_ENUM("Route", es9028_enum[9]);

static const struct snd_soc_dapm_widget es9028_dapm_widgets[] = {
#if 1
    SND_SOC_DAPM_INPUT("LINPUT1"),
    SND_SOC_DAPM_INPUT("LINPUT2"),
    SND_SOC_DAPM_INPUT("RINPUT1"),
    SND_SOC_DAPM_INPUT("RINPUT2"),

    SND_SOC_DAPM_MICBIAS("Mic Bias", ES9028_ADCPOWER, 3, 1),

    SND_SOC_DAPM_MUX("Differential Mux", SND_SOC_NOPM, 0, 0,
                     &es9028_diffmux_controls),

    SND_SOC_DAPM_MUX("Left ADC Mux", SND_SOC_NOPM, 0, 0,
                     &es9028_monomux_controls),
    SND_SOC_DAPM_MUX("Right ADC Mux", SND_SOC_NOPM, 0, 0,
                     &es9028_monomux_controls),

    SND_SOC_DAPM_MUX("Left PGA Mux", ES9028_ADCPOWER, 7, 1,
                     &es9028_left_pga_controls),
    SND_SOC_DAPM_MUX("Right PGA Mux", ES9028_ADCPOWER, 6, 1,
                     &es9028_right_pga_controls),

    SND_SOC_DAPM_MUX("Left Line Mux", SND_SOC_NOPM, 0, 0,
                     &es9028_left_line_controls),
    SND_SOC_DAPM_MUX("Right Line Mux", SND_SOC_NOPM, 0, 0,
                     &es9028_right_line_controls),

    SND_SOC_DAPM_ADC("Right ADC", "Right Capture", ES9028_ADCPOWER, 4, 1),
    SND_SOC_DAPM_ADC("Left ADC", "Left Capture", ES9028_ADCPOWER, 5, 1),

    /* gModify.Cmmt Implement when suspend/startup */
    SND_SOC_DAPM_DAC("Right DAC", "Right Playback", ES9028_DACPOWER, 7, 0),
    SND_SOC_DAPM_DAC("Left DAC", "Left Playback", ES9028_DACPOWER, 8, 0),

    SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
                       &es9028_left_mixer_controls[0],
                       ARRAY_SIZE(es9028_left_mixer_controls)),
    SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
                       &es9028_right_mixer_controls[0],
                       ARRAY_SIZE(es9028_right_mixer_controls)),

    SND_SOC_DAPM_PGA("Right Out 2", ES9028_DACPOWER, 2, 0, NULL, 0),
    SND_SOC_DAPM_PGA("Left Out 2", ES9028_DACPOWER, 3, 0, NULL, 0),
    SND_SOC_DAPM_PGA("Right Out 1", ES9028_DACPOWER, 4, 0, NULL, 0),
    SND_SOC_DAPM_PGA("Left Out 1", ES9028_DACPOWER, 5, 0, NULL, 0),
    SND_SOC_DAPM_PGA("LAMP", ES9028_ADCCONTROL1, 4, 0, NULL, 0),
    SND_SOC_DAPM_PGA("RAMP", ES9028_ADCCONTROL1, 0, 0, NULL, 0),

    SND_SOC_DAPM_OUTPUT("LOUT1"),
    SND_SOC_DAPM_OUTPUT("ROUT1"),
    SND_SOC_DAPM_OUTPUT("LOUT2"),
    SND_SOC_DAPM_OUTPUT("ROUT2"),
    SND_SOC_DAPM_OUTPUT("VREF"),

    SND_SOC_DAPM_PRE("PRE", es9028_dapm_pre_event),	
    SND_SOC_DAPM_POST("POST", es9028_dapm_post_event),
#endif
};

static const struct snd_soc_dapm_route audio_map[] = {

    { "Left Line Mux", "NULL", "LINPUT1" },
    { "Left Line Mux", "NULL", "LINPUT2" },
    { "Left Line Mux", "NULL", "Left PGA Mux" },

    { "Right Line Mux", "NULL", "RINPUT1" },
    { "Right Line Mux", "NULL", "RINPUT2" },
    { "Right Line Mux", "NULL", "Right PGA Mux" },	

    { "Left PGA Mux", "LAMP", "LINPUT1" },
    { "Left PGA Mux", "LAMP", "LINPUT2" },
    { "Left PGA Mux", "LAMP", "Differential Mux" },

    { "Right PGA Mux", "RAMP", "RINPUT1" },
    { "Right PGA Mux", "RAMP", "RINPUT2" },
    { "Right PGA Mux", "RAMP", "Differential Mux" },

    { "Differential Mux", "LAMP", "LINPUT1" },
    { "Differential Mux", "RAMP", "RINPUT1" },
    { "Differential Mux", "LAMP", "LINPUT2" },
    { "Differential Mux", "RAMP", "RINPUT2" },

    { "Left ADC Mux", "Stereo", "Left PGA Mux" },
    { "Left ADC Mux", "Mono (Left)", "Left PGA Mux" },
    //{ "Left ADC Mux", "Digital Mono", "Left PGA Mux" },

    { "Right ADC Mux", "Stereo", "Right PGA Mux" },
    { "Right ADC Mux", "Mono (Right)", "Right PGA Mux" },
    //{ "Right ADC Mux", "Digital Mono", "Right PGA Mux" },

    { "Left ADC", NULL, "Left ADC Mux" },
    { "Right ADC", NULL, "Right ADC Mux" },

    { "Left Line Mux", "LAMP", "LINPUT1" },
    { "Left Line Mux", "LAMP", "LINPUT2" },
    { "Left Line Mux", "LAMP", "Left PGA Mux" },

    { "Right Line Mux", "RAMP", "RINPUT1" },
    { "Right Line Mux", "RAMP", "RINPUT2" },
    { "Right Line Mux", "RAMP", "Right PGA Mux" },	

    { "Left Mixer", "Left Playback Switch", "Left DAC" },
    { "Left Mixer", "Left Bypass Switch", "Left Line Mux" },

    { "Right Mixer", "Right Playback Switch", "Right DAC" },
    { "Right Mixer", "Right Bypass Switch", "Right Line Mux" },

    { "Left Out 1", NULL, "Left Mixer" },
    { "LOUT1", NULL, "Left Out 1" },
    { "Right Out 1", NULL, "Right Mixer" },
    { "ROUT1", NULL, "Right Out 1" },

    { "Left Out 2", NULL, "Left Mixer" },
    { "LOUT2", NULL, "Left Out 2" },
    { "Right Out 2", NULL, "Right Mixer" },
    { "ROUT2", NULL, "Right Out 2" },
};

int es9028_dapm_pre_event(struct snd_soc_dapm_widget* widget, struct snd_kcontrol * null, int event)
{
    //	printk("fun:%s, event:%d\r\n", __FUNCTION__, event);
    if (event==1)
    { 
        widget->dapm->dev_power = 1;
        es9028_set_bias_level(widget->codec, SND_SOC_BIAS_PREPARE);
    }		
    return 0;
}
int es9028_dapm_post_event(struct snd_soc_dapm_widget* widget, struct snd_kcontrol * null, int event)
{
    //	printk("fun:%s, event:%d\r\n", __FUNCTION__, event);
    if (event==8)
    {
        widget->dapm->dev_power = 0;
        es9028_set_bias_level(widget->codec, SND_SOC_BIAS_STANDBY);
    }
    return 0;
}

struct _coeff_div {
    u32 mclk;
    u32 rate;
    u16 fs;
    u8 sr:4;
    u8 usb:1;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
    /* 8k */
    {12288000, 8000, 1536, 0xa, 0x0},
    {11289600, 8000, 1408, 0x9, 0x0},
    {18432000, 8000, 2304, 0xc, 0x0},
    {16934400, 8000, 2112, 0xb, 0x0},
    {12000000, 8000, 1500, 0xb, 0x1},

    /* 11.025k */
    {11289600, 11025, 1024, 0x7, 0x0},
    {16934400, 11025, 1536, 0xa, 0x0},
    {12000000, 11025, 1088, 0x9, 0x1},

    /* 16k */
    {12288000, 16000, 768, 0x6, 0x0},
    {18432000, 16000, 1152, 0x8, 0x0},
    {12000000, 16000, 750, 0x7, 0x1},

    /* 22.05k */
    {11289600, 22050, 512, 0x4, 0x0},
    {16934400, 22050, 768, 0x6, 0x0},
    {12000000, 22050, 544, 0x6, 0x1},

    /* 32k */
    {12288000, 32000, 384, 0x3, 0x0},
    {18432000, 32000, 576, 0x5, 0x0},
    {12000000, 32000, 375, 0x4, 0x1},

    /* 44.1k */
    {11289600, 44100, 256, 0x2, 0x0},
    {16934400, 44100, 384, 0x3, 0x0},
    {12000000, 44100, 272, 0x3, 0x1},

    /* 48k */
    {12288000, 48000, 256, 0x2, 0x0},
    {18432000, 48000, 384, 0x3, 0x0},
    {12000000, 48000, 250, 0x2, 0x1},

    /* 88.2k */
    {11289600, 88200, 128, 0x0, 0x0},
    {16934400, 88200, 192, 0x1, 0x0},
    {12000000, 88200, 136, 0x1, 0x1},

    /* 96k */
    {12288000, 96000, 128, 0x0, 0x0},
    {18432000, 96000, 192, 0x1, 0x0},
    {12000000, 96000, 125, 0x0, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
        if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
            return i;
    }

    return -EINVAL;
}

/* The set of rates we can generate from the above for each SYSCLK */

static unsigned int rates_12288[] = {
    8000, 12000, 16000, 24000, 24000, 32000, 48000, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12288 = {
    .count	= ARRAY_SIZE(rates_12288),
    .list	= rates_12288,
};

static unsigned int rates_112896[] = {
    8000, 11025, 22050, 44100,
};

static struct snd_pcm_hw_constraint_list constraints_112896 = {
    .count	= ARRAY_SIZE(rates_112896),
    .list	= rates_112896,
};

static unsigned int rates_12[] = {
    8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
    48000, 88235, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12 = {
    .count	= ARRAY_SIZE(rates_12),
    .list	= rates_12,
};


static void es9028_work_func(struct work_struct *work)
{

    printk(KERN_WARNING "9028_work_func \n");

    link_stop(0,1);
    wake_unlock(&audiolink_wakelock);
}

static DECLARE_DELAYED_WORK(es9028_delay_work, es9028_work_func);


/*
 * Note that this should be called from init rather than from hw_params.
 */
static int es9028_set_dai_sysclk(struct snd_soc_dai *codec_dai,
                                 int clk_id, unsigned int freq, int dir)
{
    struct snd_soc_codec *codec = codec_dai->codec;
    struct es9028_priv *es9028 = snd_soc_codec_get_drvdata(codec);

    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

    return 0;
    switch (freq) {
    case 11289600:
    case 18432000:
    case 22579200:
    case 36864000:
        es9028->sysclk_constraints = &constraints_112896;
        es9028->sysclk = freq;
        return 0;

    case 12288000:
    case 16934400:
    case 24576000:
    case 33868800:
        es9028->sysclk_constraints = &constraints_12288;
        es9028->sysclk = freq;
        return 0;

    case 12000000:
    case 24000000:
        es9028->sysclk_constraints = &constraints_12;
        es9028->sysclk = freq;
        return 0;
    }
    return -EINVAL;
}

static int es9028_set_dai_fmt(struct snd_soc_dai *codec_dai,
                              unsigned int fmt)
{
    /* struct snd_soc_codec *codec = codec_dai->codec; */
    /* int i; */
    alsa_dbg("%s----%d, fmt[%02x]\n",__FUNCTION__,__LINE__,fmt);

    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			/* format |= AK4490_DIF_I2S_MODE; */
            fpga_fmt=FMT_PCM;
            alsa_dbg("fmt FMT_PCM \n");
			/* if ( ak4490->nBickFreq == 1 ) 	format |= AK4490_DIF_32BIT_MODE; */
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			/* format |= AK4490_DIF_MSB_MODE; */
            fpga_fmt=FMT_PCM;
			/* if ( ak4490->nBickFreq == 1 ) 	format |= AK4490_DIF_32BIT_MODE; */
			break;
		case SND_SOC_DAIFMT_DSD:
            alsa_dbg("fmt FMT_DSD\n");
			/* format2 |= AK4490_DIF_DSD_MODE; */
            fpga_fmt=FMT_DSD;
			break;
		case SND_SOC_DAIFMT_SOP:
            alsa_dbg("fmt FMT_SOP\n");
            fpga_fmt=FMT_SOP;
            if(opt_cox_out_flag){
                /* link_set_status(LINK_UN_COAX_OUT); */
                /* msleep(50); */
                link_set_status(LINK_EN_OPT_OUT);
            }else{
                /* link_set_status(LINK_UN_OPT_OUT); */
                /* msleep(50); */
                link_set_status(LINK_EN_COAX_OUT);
            }
			break;
		default:
            alsa_dbg("fmt default\n");
            fpga_fmt=FMT_PCM;
			return -EINVAL;
	}



    return 0;
}

static int es9028_pcm_startup(struct snd_pcm_substream *substream,
                              struct snd_soc_dai *dai)
{
    int ret;
    int count=50,wcount=20;
    struct snd_soc_codec *codec = dai->codec;
    /* struct es9028_priv *es9028 = snd_soc_codec_get_drvdata(codec); */


    // u16 i;
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);


    ret=work_busy((struct work_struct*)&es9028_delay_work);
    if(ret&WORK_BUSY_PENDING){
        printk(KERN_WARNING "9028 work_func WORK_BUSY_PENDING cancel it\n");
        cancel_delayed_work_sync(&es9028_delay_work);
        wake_unlock(&audiolink_wakelock);
        /* return 0; */
    }
    if(ret&WORK_BUSY_RUNNING){

        printk(KERN_WARNING "9028 work_func WORK_BUSY_RUNNING wait it to finish\n");
        flush_delayed_work_sync(&es9028_delay_work);
    }




   while(link_get_status(LINK_REPORT_POWER_PROCESS) && wcount ){

        wcount--;
        printk("pcm_start wait power down process \n  ");
        msleep(200);
    }


    if(!link_get_status(LINK_REPORT_DAC_PWR)){
        gReWriteSensitiveMsg=true;
        printk(KERN_WARNING "DAC power off detected \n");
    }else{
        printk(KERN_WARNING "DAC power on detected \n");
        /* audioBridge_rate_set(48000,FMT_PCM); */
        /* msleep(50); */
        /* snd_soc_read(codec, ES9028_DACCONTROL3); */
    }

    link_start();


    while(!link_get_status(LINK_REPORT_DAC_PWR) && count) {

        gReWriteSensitiveMsg=true;
        msleep(100);
        count--;
    }

    if(!link_get_status(LINK_REPORT_DAC_PWR)){
            printk("LINK DAC PWR is not prepared !!! %d\n",count);
    }else{
            printk("LINK DAC PWR is prepared !!! %d\n",count);
    }


    /* gpio_direction_output(DAC_RESET,0); */
    msleep(5);

    return 0;
}
static void es9028_shutdown(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
    struct snd_soc_codec *codec = dai->codec;
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

    /* first_play=0; */
    wake_lock(&audiolink_wakelock);
    schedule_delayed_work(&es9028_delay_work,msecs_to_jiffies(AUDIO_LINK_SHUTDOWN_DELAY * 1000));
}

static int es9028_pcm_hw_params(struct snd_pcm_substream *substream,
                                struct snd_pcm_hw_params *params,
                                struct snd_soc_dai *dai)
{ 

    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_codec *codec = rtd->codec;
    /* struct es9028_priv *es9028 = snd_soc_codec_get_drvdata(codec); */
	u8 *cache = codec->reg_cache;
    int i;

    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

    DBG("params_rate=%d\n", params_rate(params));
    /* audioBridge_rate_set(params_rate(params),fpga_fmt); */

    /* es9028_update_bits(0x07,0x01,0x01); */


    /* AudioBridge_disable(); */
    /* msleep(mutedealy); */

    audioBridge_rate_set(params_rate(params),fpga_fmt);
    msleep(10);

    /* msleep(mutedealy); */

#if 1
    /* if(gReWriteSensitiveMsg){ */

        if(gReWriteSensitiveMsg ){
        /* if(0){ */
            
            msleep(10);
            es9028_reset(codec);
            msleep(50);
        }
        /* gpio_direction_output(DAC_RESET,1); */
        /* printk("LINK resend msg\n"); */

        if( fpga_fmt == FMT_DSD){
            snd_soc_write(codec, ES9028_CONTROL2 ,0xc3);
        }else if(fpga_fmt == FMT_PCM ){
            snd_soc_write(codec, ES9028_CONTROL2 ,0xc0);
        }else{

        }

        gReWriteSensitiveMsg=false;


        //update volume
        for(i=0;i<8;i++){
            snd_soc_write(codec, ES9028_DAC0+i,cache[ES9028_DAC0+i]);
            printk("LINK resend msg 0x%02x=0x%02x\n",ES9028_DAC0+i,cache[ES9028_DAC0+i]);
        }

        // filter  mode
        snd_soc_write(codec, ES9028_FILTER,cache[ES9028_FILTER]);

        //mclk div
        snd_soc_write(codec, ES9028_CONTROL1,cache[ES9028_CONTROL1]);
        // THD
        snd_soc_write(codec, ES9028_THDC2_L,cache[ES9028_THDC2_L]);
        snd_soc_write(codec, ES9028_THDC2_H,cache[ES9028_THDC2_H]);
        snd_soc_write(codec, ES9028_THDC3_L,cache[ES9028_THDC3_L]);
        snd_soc_write(codec, ES9028_THDC3_H,cache[ES9028_THDC3_H]);
        //stereo mode
        es9028_update_bits(0x0F,0x04,0x04);

    #if 0
        //power control : set mclk div
        if(params_rate(params) <=48000 ){

            es9028_update_bits(ES9028_CONTROL1,0x0C,0x08);
        }else if(params_rate(params) <=96000 ){
            es9028_update_bits(ES9028_CONTROL1,0x0C,0x04);
        }else{
            es9028_update_bits(ES9028_CONTROL1,0x0C,0x00);
        }
    #endif

        // for dac volume control
        /* msleep(100); */

    /* } */
#endif


    /* AudioBridge_enable(); */
    /* es9028_update_bits(0x07,0x01,0x00); */
    return 0;
}

static int es9028_mute(struct snd_soc_dai *dai, int mute)
{
    /* struct snd_soc_codec *codec = dai->codec; */
    // u16 mute_reg = snd_soc_read(codec, ES9028_DACCONTROL3) & 0xfb;

    DBG("Enter::%s----%d--mute=%d\n",__FUNCTION__,__LINE__,mute);
    /* return; */

    if( HW_tpye == HW_TYPE_OSC_3IN )
        return 0;

    if (mute)
    {

        link_set_status(LINK_MUTE_PO);
        /* msleep(50); */

    }
    else
    {

        /* msleep(50); */
        link_set_status(LINK_UNMUTE_PO);

    }

    return 0;
}

static int es9028_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{
    
   /* struct snd_soc_codec *codec = codec_dai->codec; */

    int ret=0;

    DBG("Enter::%s----%d \n",__FUNCTION__,__LINE__);

    /* WARN_ON(1); */
    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
        DBG("[ES9028] Trigger CMD SNDRV_PCM_TRIGGER_START \n");
        
        /* gpio_direction_output(RK30_PIN3_PD7,0); */
        /* udelay(200); */
        /* gpio_direction_output(RK30_PIN3_PD7,1); */
       break;
    case SNDRV_PCM_TRIGGER_STOP:

        DBG("[ES9028] Trigger CMD SNDRV_PCM_TRIGGER_STOP \n");
        /* gpio_request(RK30_PIN3_PD7,"ttt"); */
        /* gpio_direction_output(RK30_PIN3_PD7,0); */
        /* udelay(100); */
        /* gpio_direction_output(RK30_PIN3_PD7,1); */
        /* gpio_free(RK30_PIN3_PD7); */
        /* do something to stop the PCM engine */
        break;
    default:
        DBG("[ES9028] Trigger CMD %d \n",cmd);
        return -EINVAL;
    }

	return ret;
}


/////////////////////////////////////////////////////////////////
static int es9028_set_bias_level(struct snd_soc_codec *codec,
                                 enum snd_soc_bias_level level)
{
    /* struct es9028_priv *es9028 = snd_soc_codec_get_drvdata(codec); */
    // u16 OUT_VOL = snd_soc_read(codec, ES9028_LOUT1_VOL);
    // u16 i;

    DBG("Enter::%s----%d level =%d\n",__FUNCTION__,__LINE__,level);
    return 0;

}



/* #define es9028_RATES SNDRV_PCM_RATE_8000_384000   */

/* #define es9028_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\ */
                        /* SNDRV_PCM_FMTBIT_S24_LE) */

#define ES9028_RATES (SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000| \
                      SNDRV_PCM_RATE_24000 | SNDRV_PCM_RATE_128000| \
                      SNDRV_PCM_RATE_64000 | SNDRV_PCM_RATE_44100 | \
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
			SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 | \
            SNDRV_PCM_RATE_384000)

#define ES9028_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_SOP|\
        SNDRV_PCM_FMTBIT_DSD64_F32 |  SNDRV_PCM_FMTBIT_DSD128_F32  |SNDRV_PCM_FMTBIT_DSD256_F32 |\
        SNDRV_PCM_FMTBIT_DSD64_CT24 | SNDRV_PCM_FMTBIT_DSD128_CT24 |SNDRV_PCM_FMTBIT_DSD256_CT24)




static struct snd_soc_dai_ops es9028_ops = {
    .startup = es9028_pcm_startup,
    .set_fmt = es9028_set_dai_fmt,
    .set_sysclk = es9028_set_dai_sysclk,
    .hw_params = es9028_pcm_hw_params,
    .digital_mute = es9028_mute,
	.trigger = es9028_trigger,
    .shutdown   = es9028_shutdown,
};

static struct snd_soc_dai_driver es9028_dai = {
    .name = "ES9028 HiFi",
    .playback = {
        .stream_name = "Playback",
        .channels_min = 1,
        .channels_max = 2,
        .rates = ES9028_RATES,
        .formats = ES9028_FORMATS,
    },
#if 0
    .capture = {
        .stream_name = "Capture",
        .channels_min = 1,
        .channels_max = 2,
        .rates = es9028_RATES,
        .formats = es9028_FORMATS,
    },
#endif
    .ops = &es9028_ops,
    .symmetric_rates = 1,
};

static int es9028_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
    // u16 i;
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

#if 0
    snd_soc_write(codec, 0x19, 0x06);
    snd_soc_write(codec, 0x07, 0x7B);
    snd_soc_write(codec, 0x06, 0xFF);
    snd_soc_write(codec, 0x05, 0xFF);

    snd_soc_write(codec, 0x19, 0x06);
    snd_soc_write(codec, 0x30, 0x00);
    snd_soc_write(codec, 0x31, 0x00);
    snd_soc_write(codec, ES9028_ADCPOWER, 0xFF);					
    snd_soc_write(codec, ES9028_DACPOWER, 0xc0);  	
    snd_soc_write(codec, ES9028_CHIPPOWER, 0xF3);
    snd_soc_write(codec, 0x00, 0x00);
    snd_soc_write(codec, 0x01, 0x58);
    snd_soc_write(codec, 0x2b, 0x9c);	
    msleep(50);
    /* gpio_set_value(SPK_CON, 0); */
#endif
    return 0;
}

static int es9028_resume(struct snd_soc_codec *codec)
{
    // u16 i;
    // u8 data[2];
    // u16 *cache = codec->reg_cache;	
#if 0
    snd_soc_write(codec, 0x2b, 0x80);	
    snd_soc_write(codec, 0x01, 0x50);
    snd_soc_write(codec, 0x00, 0x32);
    snd_soc_write(codec, ES9028_CHIPPOWER, 0x00);	
    snd_soc_write(codec, ES9028_DACPOWER, 0x0c);	
    snd_soc_write(codec, ES9028_ADCPOWER, 0x59);
    snd_soc_write(codec, 0x31, es9028_DEF_VOL);
    snd_soc_write(codec, 0x30, es9028_DEF_VOL);
    snd_soc_write(codec, 0x19, 0x02);			
    gpio_set_value(SPK_CON, 1);
#endif
    return 0;
}

static u32 cur_reg=0;
#if 0
static int entry_read(char *page, char **start, off_t off,
                      int count, int *eof, void *data)
{
    int len;

    snd_soc_write(es9028_codec, ES9028_ADCPOWER, 0xff);
    snd_soc_write(es9028_codec, ES9028_DACPOWER, 0xf0);
    snd_soc_write(es9028_codec, ES9028_DACPOWER, 0xc0);
    snd_soc_write(es9028_codec, ES9028_CHIPPOWER, 0xf3);

    len = sprintf(page, "es9028 suspend...\n");

    return len ;
}

/* static DECLARE_DELAYED_WORK(wakeup_work, ); */
static void spk_work_handler(struct work_struct *work)
{

    static unsigned char i=0;
    i++;
    i=0xA5;
    snd_soc_write(es9028_codec,0x04,i);
    /* es9028_hw_write(es9028_codec,0x04,i); */
    msleep(5);

    snd_soc_read(es9028_codec,0x04);
    /* es9028_hw_read(es9028_codec,0x04); */
    schedule_delayed_work(&spk_work, HZ);
}
#endif

static int es9028_probe(struct snd_soc_codec *codec)
{
    // struct es9028_priv *es9028 = snd_soc_codec_get_drvdata(codec);
    /* struct snd_soc_dapm_context *dapm = &codec->dapm; */
    int ret = 0;
    int num_controls;
    struct snd_kcontrol_new *ctl=NULL;
    // u16 reg,i;

    printk("%s\n", __func__);

    ret = gpio_request(DAC_RESET, "DAC_RESET");
    if (ret != 0) {
        printk("%s request DAC_RESET error", __func__);
        return ret;
    }
    /* audioBridge_rate_set(48000,FMT_PCM); */
    /* msleep(5); */
    /* gpio_direction_output(DAC_RESET,0); */
    /* msleep(5); */
    gpio_direction_output(DAC_RESET,0);
    /* msleep(500); */


    if (codec == NULL) {
        dev_err(codec->dev, "Codec device not registered\n");
        return -ENODEV;
    }
    /* codec->read  = es9028_hw_read_reg_cache; */
    codec->read=   es9028_hw_read;
    codec->write = es9028_hw_write;
    codec->hw_write = (hw_write_t)es9028_i2c_master_send;
    /* codec->hw_read= es9028_hw_read; */
    codec->control_data = container_of(codec->dev, struct i2c_client, dev);

    es9028_codec = codec;

	/* INIT_DELAYED_WORK(&spk_work, spk_work_handler); */
    msleep(100);

#if 0
    snd_soc_read(es9028_codec,0x00);
    ret = es9028_reset(codec);
    if (ret < 0) {
        dev_err(codec->dev, "Failed to issue reset\n");
        /* return ret; */
    }
#endif

	/* schedule_delayed_work(&spk_work, HZ / 5); */

    wake_lock_init(&audiolink_wakelock, WAKE_LOCK_SUSPEND, "audiolink");

    snd_soc_add_controls(codec, es9028_snd_controls,
                         ARRAY_SIZE(es9028_snd_controls));
#if 0
    snd_soc_dapm_new_controls(dapm, es9028_dapm_widgets,
                              ARRAY_SIZE(es9028_dapm_widgets));
    snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

    create_proc_read_entry("es9028_suspend", 0644, NULL, entry_read, NULL);
#endif

    ctl=get_audiobirdge_control_table(&num_controls);
    if(ctl!=NULL && num_controls>0){

        snd_soc_add_controls(codec, ctl,num_controls);

    }


        gpio_request(RK30_PIN3_PD7,"ttt");
        gpio_direction_output(RK30_PIN3_PD7,0);
    gpio_direction_output(DAC_RESET,1);
    return 0;
}

static int es9028_remove(struct snd_soc_codec *codec)
{
    es9028_set_bias_level(codec, SND_SOC_BIAS_OFF);
    return 0;
}
static struct snd_soc_codec_driver soc_codec_dev_es9028 = {
    .probe =	es9028_probe,
    .remove =	es9028_remove,
    .suspend =	es9028_suspend,
    .resume =	es9028_resume,
    .set_bias_level = es9028_set_bias_level,
    .reg_cache_size = ARRAY_SIZE(es9028_reg),
    .reg_word_size = sizeof(u8),
    .reg_cache_default = es9028_reg,
    //------------------------------------------
    //.volatile_register = es9028_volatile_register,
    //.readable_register = es9028_hw_readable_register,
    .reg_cache_step = 1,
#if 0
    .controls = es9028_snd_controls,
    .num_controls = ARRAY_SIZE(es9028_snd_controls),	
    .dapm_routes = audio_map,  
    .num_dapm_routes = ARRAY_SIZE(audio_map), 
    .dapm_widgets = es9028_dapm_widgets,  
    .num_dapm_widgets = ARRAY_SIZE(es9028_dapm_widgets),   

#endif
    //--------------------------------------------------	
    /* .read	= es9028_hw_read_reg_cache, */
    /* .write = es9028_hw_write,	 */
};

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static ssize_t es9028_show(struct device *dev, struct device_attribute *attr, char *_buf)
{
    return sprintf(_buf, "%s(): get 0x%04x=0x%04x\n", __FUNCTION__, cur_reg, 
                   snd_soc_read(es9028_codec, cur_reg));
}

static u32 strtol(const char *nptr, int base)
{
    u32 ret;
    if(!nptr || (base!=16 && base!=10 && base!=8))
    {

        printk("%s(): NULL pointer input\n", __FUNCTION__);
        return -1;
    }
    for(ret=0; *nptr; nptr++)
    {


        if((base==16 && *nptr>='A' && *nptr<='F') || 
           (base==16 && *nptr>='a' && *nptr<='f') || 
           (base>=10 && *nptr>='0' && *nptr<='9') ||
           (base>=8 && *nptr>='0' && *nptr<='7') )
        {
            ret *= base;
            if(base==16 && *nptr>='A' && *nptr<='F')
                ret += *nptr-'A'+10;
            else if(base==16 && *nptr>='a' && *nptr<='f')
                ret += *nptr-'a'+10;
            else if(base>=10 && *nptr>='0' && *nptr<='9')
                ret += *nptr-'0';
            else if(base>=8 && *nptr>='0' && *nptr<='7')
                ret += *nptr-'0';
        }
        else
            return ret;
    }
    return ret;
}

static ssize_t es9028_store(struct device *dev,
                            struct device_attribute *attr,
                            const char *_buf, size_t _count)
{
    const char * p=_buf;
    u32 reg, val;

    if(!strncmp(_buf, "get", strlen("get")))
    {
        p+=strlen("get");
        cur_reg=(u32)strtol(p, 16);
        val=snd_soc_read(es9028_codec, cur_reg);
        printk("%s(): get 0x%04x=0x%04x\n", __FUNCTION__, cur_reg, val);
    }
    else if(!strncmp(_buf, "put", strlen("put")))
    {
        p+=strlen("put");
        reg=strtol(p, 16);
        p=strchr(_buf, '=');
        if(p)
        {
            ++ p;
            val=strtol(p, 16);
            snd_soc_write(es9028_codec, reg, val);
            printk("%s(): set 0x%04x=0x%04x\n", __FUNCTION__, reg, val);
        }
        else
            printk("%s(): Bad string format input!\n", __FUNCTION__);
    }
    else
        printk("%s(): Bad string format input!\n", __FUNCTION__);

    return _count;
} 

static struct device *es9028_dev = NULL;
static struct class *es9028_class = NULL;
static DEVICE_ATTR(es9028, 0664, es9028_show, es9028_store);
static __devinit int es9028_i2c_probe(struct i2c_client *i2c,
                                      const struct i2c_device_id *id)
{

    struct es9028_priv *es9028;
    int ret = -1;
    struct i2c_adapter *adapter = to_i2c_adapter(i2c->dev.parent);

        printk("es9028 probe i2c start\n");
    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
        dev_warn(&adapter->dev,
                 "I2C-Adapter doesn't support I2C_FUNC_I2C\n");
        return -EIO;
    }

    es9028 = kzalloc(sizeof(struct es9028_priv), GFP_KERNEL);
    if (es9028 == NULL)
        return -ENOMEM;

    i2c_set_clientdata(i2c, es9028);
    es9028->control_type = SND_SOC_I2C;

#if 0
    reg = ES9028_DACCONTROL18;
    ret = i2c_master_reg8_recv(i2c, reg, &tmp, 1 ,200 * 1000);
    //ret =i2c_master_reg8_recv(client, 0x00, buf, 2, 200*1000);//i2c_write_bytes(client, &test_data, 1);	//Test I2C connection.
    if (ret < 0){
        printk("es9028 probe error\n");
        kfree(es9028);
        return ret;
    }
#endif

    printk("es9028 probe i2c recv ok\n");

    ret =  snd_soc_register_codec(&i2c->dev,
                                  &soc_codec_dev_es9028, &es9028_dai, 1);
    if (ret < 0) {
        kfree(es9028);
        return ret;
    }
    es9028_class = class_create(THIS_MODULE, "es9028");
    if (IS_ERR(es9028_class))
    {
        printk("Create class audio_es9028.\n");
        return -ENOMEM;
    }
    es9028_dev = device_create(es9028_class, NULL, MKDEV(0, 1), NULL, "dev");
    ret = device_create_file(es9028_dev, &dev_attr_es9028);
    if (ret < 0)
        printk("failed to add dev_attr_es9028 file\n");
#ifdef CONFIG_MACH_RK_FAC              
    es9028_hdmi_ctrl=1;
#endif 

    return ret;
}

static __devexit int es9028_i2c_remove(struct i2c_client *client)
{
    snd_soc_unregister_codec(&client->dev);
    kfree(i2c_get_clientdata(client));
    return 0;
}

static const struct i2c_device_id es9028_i2c_id[] = {
    { "ES9028", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, es9028_i2c_id);

void es9028_i2c_shutdown(struct i2c_client *client)
{
    printk("Chenzy-------hkw-------%s\n", __func__);

    /* snd_soc_write(es9028_codec, ES9028_CONTROL1, 0x30);					 */
    /* snd_soc_write(es9028_codec, ES9028_CONTROL1, 0x34);					 */

    es9028_update_bits(0x07,0x01,0x01);
    mdelay(100);
    link_stop(1,0);
    msleep(3000);
}
#define  I2C_CLK_NAME  GPIO0B0_I2S8CHCLK_NAME
#define  I2C_CLK_GPIO_MODE  GPIO0B_GPIO0B0
#define  I2C_GPIO_OUTPUT  GPIO_LOW
#define  I2C_CLK_CLK_MODE   GPIO0B_I2S_8CH_CLK
#define  I2C_CLK_GPIO   RK30_PIN0_PB0

#define  I2C_MCLK_NAME  GPIO0B1_I2S8CHSCLK_NAME
#define  I2C_MCLK_GPIO_MODE  GPIO0B_GPIO0B1
#define  I2C_MGPIO_OUTPUT  GPIO_LOW
#define  I2C_MCLK_CLK_MODE   GPIO0B_I2S_8CH_SCLK
#define  I2C_MCLK_GPIO   RK30_PIN0_PB1
static int   es9028_i2c_suspend (struct i2c_client *client, pm_message_t mesg)
{
#if 0
    rk30_mux_api_set(I2C_CLK_NAME,I2C_CLK_GPIO_MODE);
    if (gpio_request(I2C_CLK_GPIO, NULL)) {
        printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
        return -1;
    }

    gpio_direction_output(I2C_CLK_GPIO,I2C_GPIO_OUTPUT);

    rk30_mux_api_set(I2C_MCLK_NAME,I2C_MCLK_GPIO_MODE);
    if (gpio_request(I2C_MCLK_GPIO, NULL)) {
        printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
        return -1;
    }

    gpio_direction_output(I2C_MCLK_GPIO,I2C_MGPIO_OUTPUT);
#endif

    #if 0
    iomux_set(GPIO1_C2);
    gpio_direction_input(RK30_PIN1_PC2);
    gpio_pull_updown(RK30_PIN1_PC2, PullDisable);

    iomux_set(GPIO1_C3);
    gpio_direction_input(RK30_PIN1_PC3);
    gpio_pull_updown(RK30_PIN1_PC3, PullDisable);

    iomux_set(GPIO1_C4);
    gpio_direction_input(RK30_PIN1_PC4);
    gpio_pull_updown(RK30_PIN1_PC4, PullDisable);

    iomux_set(GPIO1_C5);
    gpio_direction_input(RK30_PIN1_PC5);
    gpio_pull_updown(RK30_PIN1_PC5, PullDisable);
#endif

    return 0;
}

static int   es9028_i2c_resume(struct i2c_client *client)
{
#if 0
    gpio_free(I2C_MCLK_GPIO);
    gpio_free(I2C_CLK_GPIO);

    rk30_mux_api_set(I2C_MCLK_NAME,I2C_MCLK_CLK_MODE);
    rk30_mux_api_set(I2C_CLK_NAME,I2C_CLK_CLK_MODE);
#endif

    /* gpio_free(RK30_PIN1_PC1); */
    /* iomux_set(I2S0_SCLK); */

    return 0;
}

static struct i2c_driver es9028_i2c_driver = {
    .driver = {
        .name = "ES9028",
        .owner = THIS_MODULE,
    },
    .probe =    es9028_i2c_probe,
    .remove =   __devexit_p(es9028_i2c_remove),
    .shutdown = es9028_i2c_shutdown,
    .suspend  = es9028_i2c_suspend,
    .resume = es9028_i2c_resume,
    .id_table = es9028_i2c_id,
};
#endif

static int __init es9028_modinit(void)
{
    return i2c_add_driver(&es9028_i2c_driver);
}
module_init(es9028_modinit);

static void __exit es9028_exit(void)
{

    //	if(0 == tcsi_get_value(TCSI_CODEC_ES9028))
    //		return;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
    i2c_del_driver(&es9028_i2c_driver);
#endif
}
module_exit(es9028_exit);


MODULE_DESCRIPTION("ASoC es9028 driver");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");

