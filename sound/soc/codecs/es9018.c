/*
 * es9018 ALSA SoC Audio driver
 * Author: Kuangwenxu <kuangwenxu@163.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <linux/i2c.h>
#include <sound/asoundef.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <mach/iomux.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include "es9018.h"
#include "fpga_audio_interface.h"
#include <linux/regulator/consumer.h>
#include "../../../drivers/regulator/axp_power/axp-mfd.h"
#include <linux/reboot.h>

extern volatile uint32_t sys_reboot_cmd;
extern int board_boot_mode(void);
static int debug=0;
module_param(debug, int, S_IRUGO|S_IWUSR);
#define dprintk(level, fmt, arg...) do {			\
	if (debug >= level) 					\
	printk(KERN_WARNING "ES9018 %s-%d: " fmt ,__FUNCTION__, __LINE__, ## arg); } while (0)

#define CODEC_TR(format, ...) printk(KERN_ERR format, ## __VA_ARGS__)
#define CODEC_DG(format, ...) dprintk(1, format, ## __VA_ARGS__)

#define ES9018_RESET_PIN		  RK30_PIN0_PB4   //low:enable
#define ES9018_POWER_PIN		  RK30_PIN0_PA1   //high:enable
#define HOST_POWER_PIN		      RK30_PIN0_PA3   //high:enable  for +-5.0V
#define AM_POWER_PIN		      RK30_PIN3_PD7   //high:enable
#define AM_OP_EN                  RK30_PIN0_PA0   //high:enable
#define AM_MUTE_EN                RK30_PIN1_PA7   //high:enable
#define LINE_MUTE_EN              RK30_PIN1_PA6   //high:enable
#define OSC2_PIN_I2S_MOD          I2S0_SDI
#define OSC2_PIN                  RK30_PIN1_PC4

#define OSC3_PIN_I2S_MOD          I2S0_LRCKRX
#define OSC3_PIN                  RK30_PIN1_PC2

#define OSC_48KX_PIN              OSC2_PIN
#define OSC_44P1KX_PIN            OSC3_PIN

#define ENABLE_OSC_48K            1
#define ENABLE_OSC_44P1K          2

#define FPGA_ALWAYS_ON
/* #define DAC_POWER_ALWAYS_ON 1 */

#define AUDIO_LINK_SHUTDOWN_DELAY 5
#define AUDIO_LINK_HP_DELAY 20

#define WAKE_LOCK_TIMEOUT	((AUDIO_LINK_SHUTDOWN_DELAY+1) * HZ)
static struct wake_lock audiolink_wakelock;
extern int gAndroid_boot_completed;
static u16 gfpga_format=0x80;
extern bool is_otg_mode(void);

extern int audio_get_amp_status();
extern uint8_t audio_get_am_type();
extern void AudioBridge_disable();
extern void AudioBridge_enable();
/* audio hardware link status*/
struct audio_hw_link {

    // hw status
    char playback;
    char headphone;
    char linein;

    // power status
    char fpga_on;
    char dac_on;
    char am_on;
    char am_op_on;
    char am_mute;
    char line_mute;

    char vol;
    bool vol_mute;//indicate is mute function call do not keep Volume to vol
    bool mute; // indicate DAC mute status
};

struct audio_hw_link *gpAudioHwLink=NULL;
/* codec private data */
struct es9018_priv {
	enum snd_soc_control_type control_type;
	void *control_data;
	unsigned int sysclk;
};

#define GPIO_HIGH 1
#define GPIO_LOW 0

static struct snd_soc_codec *es9018_codec;
struct reg_default{
    int reg;
    int value;
};

struct work_data 
{
	int		reset_gpio;
	int		last_point_num;
	struct work_struct 	action_event_work;
	struct workqueue_struct *directc_workqueue;
};



static int set_lineout_mute(char );
/* int set_es9018_power_off(void); */
/* int set_es9018_power_on(void); */
int set_am_power_sequence(struct audio_hw_link* ,char );

static void es9018_all_write(struct snd_soc_codec *codec);
void exAudioLinkChange(char);

#define REG_CONFIG_MAX  8
static struct reg_default es9018_reg_defaults[] = {
	{  0, 0xE0 },     /* R0  - DAC1 Attenuation */
	{  1, 0xE0 },     /* R1  - DAC2 Attenuation */
	{  2, 0xE0 },     /* R2  - DAC3 Attenuation */
	{  3, 0xE0 },     /* R3  - DAC4 Attenuation */
	{  4, 0xE0 },     /* R4  - DAC5 Attenuation */
	{  5, 0xE0 },     /* R5  - DAC6 Attenuationl */
	{  6, 0xE0 },     /* R6  - DAC7 Attenuation */
	{  7, 0xE0 },     /* R7  - DAC8 Attenuation */
	{  8, 0x68 },     /* R8  - AUTO MUTE */
	{  9, 0x04 },     /* R9  - AUTO MUTE TIME */
	{  10, 0xCE },     /* R10  - Mode Control 1 */
	{  11, 0x85 },     /* R11  - Mode Control 2 */
	{  12, 0x20 },     /* R12  - Mode Control 3 */
	{  13, 0x00 },     /* R13  - DAC POL */
	{  14, 0x0B },     /* R14  - DAC3478 */
	{  15, 0x00 },     /* R15  - Mode Control 4 */
	{  16, 0x08 },     /* R16  - AutoMuteLoopBack */
	{  17, 0x5C },     /* R17  - Mode Control 5 */
	{  18, 0x01 },     /* R18  - SPDIF SRC DATAx */
	{  19, 0x00 },     /* R19  - DACB POL */
	{  24, 0x30 },     /* R24  - PHASE SHIFT */
	{  25, 0x02 },     /* R25  - DPLL MODE */
};

/* static bool es9018_readable(struct device *dev, unsigned int reg) */
/* { */
	/* switch (reg) { */
		/* case ES9018_DAC0: */
		/* case ES9018_DAC1: */
		/* case ES9018_DAC2: */
		/* case ES9018_DAC3: */
		/* case ES9018_DAC4: */
		/* case ES9018_DAC5: */
		/* case ES9018_DAC6: */
		/* case ES9018_DAC7: */
		/* case ES9018_AUTOMUTE_LEV: */
		/* case ES9018_AUTOMUTE_TIME: */
		/* case ES9018_MODE_CONTROL_1: */
		/* case ES9018_MODE_CONTROL_2: */
		/* case ES9018_MODE_CONTROL_3: */
		/* case ES9018_DAC_POL: */
		/* case ES9018_DAC_SRC_3478: */
		/* case ES9018_MODE_CONTROL_4: */
		/* case ES9018_AUTOMUTE_LPBACK: */
		/* case ES9018_MODE_CONTROL_5: */
		/* case ES9018_SPDIF_SRC: */
		/* case ES9018_DACB_POL: */
		/* case ES9018_PHASE_SHIFT: */
		/* case ES9018_DPLL_MODE: */
		/* case ES9018_STATUS: */
		/* case ES9018_PROG_ENABLE: */
		/* case ES9018_SPDIF_STS_00: */
		/* case ES9018_SPDIF_STS_01: */
		/* case ES9018_SPDIF_STS_02: */
		/* case ES9018_SPDIF_STS_03: */
		/* case ES9018_SPDIF_STS_04: */
		/* case ES9018_SPDIF_STS_22: */
		/* case ES9018_SPDIF_STS_23: */

		/* return true; */
	/* default: */
		/* return false; */
	/* } */
/* } */

extern bool dac_power_hold;
static void es9018_work_func(struct work_struct *work)
{

        printk(KERN_WARNING "9018_work_func \n");
    if (!dac_power_hold) {
        exAudioLinkChange(EVENT_PLAYBACK_OFF);
        wake_unlock(&audiolink_wakelock);
    }

}

static DECLARE_DELAYED_WORK(es9018_delay_work, es9018_work_func);

static bool isDSDFormat(snd_pcm_format_t fmt)
{
    switch(fmt){
    case SNDRV_PCM_FORMAT_DSD64_F32:   
    case SNDRV_PCM_FORMAT_DSD128_F32:  
    case SNDRV_PCM_FORMAT_DSD256_F32:  
    case SNDRV_PCM_FORMAT_SOP:  
        return true;
    default:
        return false;
    }

}

static int es9018_reset(struct snd_soc_codec *codec)
{
	return 0;
}

static int es9018_dac_mute_put(struct snd_kcontrol *kcontrol, 
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	/* struct snd_soc_dapm_context *dapm = &codec->dapm; */
	/* struct snd_soc_dapm_widget *w; */
	struct rt5625_priv *rt5625 = snd_soc_codec_get_drvdata(codec);
    int i;
    u8 data;

    if(ucontrol->value.integer.value[0] ){
        data = snd_soc_read(codec, ES9018_DAC0);
        gpAudioHwLink->vol = data;

        for(i=0;i<=7;i++){
            gpAudioHwLink->vol_mute=true;
            snd_soc_write(codec, ES9018_DAC0+i,255);
            //gpAudioHwLink->vol_mute=false;
            gpAudioHwLink->mute=true;
        }
    }else{

        for(i=0;i<=7;i++){
            snd_soc_write(codec, ES9018_DAC0+i,gpAudioHwLink->vol);
        }
        gpAudioHwLink->mute=false;
    }



	return 0;
}

static int es9018_dac_mute_get(struct snd_kcontrol *kcontrol, 
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    u8 data;
    data= snd_soc_read(codec, ES9018_DAC0);
    //gpAudioHwLink->mute ??
    if(data==0xFF)
	    ucontrol->value.integer.value[0] = 1;
    else
	    ucontrol->value.integer.value[0] = 0;

	return 0;
}


static const DECLARE_TLV_DB_SCALE(dac_tlv, -12750, 500, 0);


static const struct snd_kcontrol_new es9018_snd_controls[] = {
    SOC_SINGLE_TLV("DAC0 Playback Volume", ES9018_DAC0,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC1 Playback Volume", ES9018_DAC1,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC2 Playback Volume", ES9018_DAC2,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC3 Playback Volume", ES9018_DAC3,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC4 Playback Volume", ES9018_DAC4,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC5 Playback Volume", ES9018_DAC5,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC6 Playback Volume", ES9018_DAC6,0, 255, 1, dac_tlv),
    SOC_SINGLE_TLV("DAC7 Playback Volume", ES9018_DAC7,0, 255, 1, dac_tlv),

    // DAC mute is not works well ,do not use
    SOC_SINGLE("DAC Mute", ES9018_MODE_CONTROL_1, 0, 1, 0), 

    SOC_SINGLE("DAC ramp", ES9018_AUTOMUTE_LPBACK, 3, 1, 0), 

	SOC_SINGLE_EXT("DAC Mute Mode", 0xFF, 0, 1, 0,es9018_dac_mute_get,es9018_dac_mute_put),

    //add for digital filter mode, slow or fast, default is fast
    SOC_SINGLE("DAC Digital Filter Mode", ES9018_DAC_SRC_3478, 0, 1, 0),
};

static int es9018_startup(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
    unsigned int ret;
    struct snd_soc_codec *codec = dai->codec;
    CODEC_TR("%s %d\n",__func__,__LINE__);
    printk(" codec  reg_size %d cache_only %d sync %d init %d\n",codec->reg_size,codec->cache_only,codec->cache_sync,codec->cache_init);
    printk(KERN_WARNING "%s %d\n",__func__,__LINE__);
    /* set_am_power_sequence(1); */


    ret=work_busy((struct work_struct*)&es9018_delay_work);
    if(ret&WORK_BUSY_PENDING){
        printk(KERN_WARNING "9018 work_func WORK_BUSY_PENDING cancel it\n");
        cancel_delayed_work_sync(&es9018_delay_work);
        wake_unlock(&audiolink_wakelock);
        /* return 0; */
    }
    if(ret&WORK_BUSY_RUNNING){

        printk(KERN_WARNING "9018 work_func WORK_BUSY_RUNNING wait it to finish\n");
        flush_delayed_work_sync(&es9018_delay_work);
    }
    exAudioLinkChange(EVENT_PLAYBACK_START);

    /* es9018_all_write(codec); */

#if 0
    audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_44_1K16);
    //volume bypass
    codec->cache_only=0;
    // force hw write
    codec->cache_sync=1;
    snd_soc_cache_sync(codec);
#endif
    /* audio_fpga_bridge_sendCommand( FPGA_SUSPEND16); */
    /* codec->cache_only=1; */
	/* struct es9018_priv *es9018 = snd_soc_codec_get_drvdata(codec); */

	/* The set of sample rates that can be supported depends on the
	 * MCLK supplied to the CODEC - enforce this.
	 */
	return 0;
}

static int es9018_shutdown(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
    struct snd_soc_codec *codec = dai->codec;
    CODEC_TR("%s %d\n",__func__,__LINE__);
    /* printk(KERN_WARNING "%s %d\n",__func__,__LINE__); */
    /* set_am_power_sequence(0); */

    /* codec->cache_only=1; */
    wake_lock(&audiolink_wakelock);
    schedule_delayed_work(&es9018_delay_work,msecs_to_jiffies(AUDIO_LINK_SHUTDOWN_DELAY * 1000));
    CODEC_TR("AUDIO_LINK: schedule es9018_delay_work\n");
    /* exAudioLinkChange(EVENT_PLAYBACK_OFF); */
	/* struct snd_soc_codec *codec = dai->codec; */
	/* struct es9018_priv *es9018 = snd_soc_codec_get_drvdata(codec); */

	/* The set of sample rates that can be supported depends on the
	 * MCLK supplied to the CODEC - enforce this.
	 */
	return 0;
}
static int es9018_reg_cache_write(struct snd_soc_codec *codec,unsigned int reg,
		unsigned int value)
{

    int i;
    BUG_ON(codec->volatile_register);


    for (i=0;i< ARRAY_SIZE(es9018_reg_defaults);i++){
         if(es9018_reg_defaults[i].reg == reg)
            es9018_reg_defaults[i].value=value;
         }

}
static int es9018_reg_cache_read(struct snd_soc_codec *codec,unsigned int reg,
		unsigned int *value)
{

    int i;
    BUG_ON(codec->volatile_register);


    for (i=0;i< ARRAY_SIZE(es9018_reg_defaults);i++){
         if(es9018_reg_defaults[i].reg == reg)
            *value=es9018_reg_defaults[i].value;
         }

}


static int es9018_reg_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
	u8 data[2];
	int ret;

	/* BUG_ON(codec->volatile_register); */
	/* BUG_ON(1); */

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
static int es9018_reg_write_and_cache(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
    return es9018_reg_write(codec,reg,value);
}


/* write the register space */
static void es9018_all_write(struct snd_soc_codec *codec)
{
	int ret, val, i;
	val = 0;

	for(i=0; i<ARRAY_SIZE(es9018_reg_defaults); i++)
	{
		ret = es9018_reg_write(codec, es9018_reg_defaults[i].reg, es9018_reg_defaults[i].value);
		if (ret < 0)
			printk("%s-%d: write reg%2d failed!\n", __FUNCTION__, __LINE__, i);
	}

}


static int es9018_hw_params(struct snd_pcm_substream *substream,
                            struct snd_pcm_hw_params *params,
                            struct snd_soc_dai *dai)
{
    struct snd_soc_codec *codec = dai->codec;
    /* struct es9018_priv *es9018 = snd_soc_codec_get_drvdata(codec); */
    u8 iface,i;

    CODEC_TR("%s %d\n",__func__,__LINE__);
    /* iface= snd_soc_read(codec, ES9018_MODE_CONTROL_2); */
    CODEC_DG("params_rate=%d\n", params_rate(params));
    gfpga_format=AUDIO_SAMPLERATE_352_8K16;

    /*  SPDIF OVER PCM --> SOP format */
    if(params_format(params)==SNDRV_PCM_FORMAT_SOP) {
        switch (params_rate(params)) {
        case 352800:
            audio_fpga_bridge_sendCommand( AUDIO_FORMAT_SOP_352_8K);
            gfpga_format=AUDIO_FORMAT_SOP_352_8K;
            break;
        case 176400:
            audio_fpga_bridge_sendCommand( AUDIO_FORMAT_SOP_176_4K);
            gfpga_format=AUDIO_FORMAT_SOP_176_4K;
            break;
        case 88200:
            audio_fpga_bridge_sendCommand( AUDIO_FORMAT_SOP_88_2K);
            gfpga_format=AUDIO_FORMAT_SOP_88_2K;
            break;
        case 384000:
            audio_fpga_bridge_sendCommand( AUDIO_FORMAT_SOP_384K);
            gfpga_format=AUDIO_FORMAT_SOP_384K;
            break;
        case 192000:
            audio_fpga_bridge_sendCommand( AUDIO_FORMAT_SOP_192K);
            gfpga_format=AUDIO_FORMAT_SOP_192K;
            break;
        case 96000:
            audio_fpga_bridge_sendCommand( AUDIO_FORMAT_SOP_96K);
            gfpga_format=AUDIO_FORMAT_SOP_96K;
            break;
        default:
            dev_err(codec->dev, "unsupported sop sampling rate %d\n",params_rate(params));
            return -EINVAL;
        }

    /*  DSD OVER PCM --> DOP format */
    }else if(isDSDFormat(params_format(params))){

        switch (params_format(params)) {
        // default S32_LE is work 
        // ES9018_MODE_CONTROL_1 format control bit 
        // don't need to config
        case SNDRV_PCM_FORMAT_S16_LE:
            iface |= 0x80; 
            break;
        case SNDRV_PCM_FORMAT_S20_3LE:
            iface |= 0x60;
            break;
        case SNDRV_PCM_FORMAT_S24_LE:
            iface |= 0x00;
            break;
        case SNDRV_PCM_FORMAT_S32_LE:
            iface |= 0xC0;
            break;
        case SNDRV_PCM_FORMAT_DSD64_F32:
            audio_fpga_bridge_sendCommand(AUDIO_FORMAT_DOP_64);
            CODEC_DG("AUDIO_FORMAT_DOP_64\n");
            break;
        case SNDRV_PCM_FORMAT_DSD128_F32:
            audio_fpga_bridge_sendCommand(AUDIO_FORMAT_DOP_128);
            CODEC_DG("AUDIO_FORMAT_DOP_128\n");
            break;
        case SNDRV_PCM_FORMAT_DSD256_F32:
            audio_fpga_bridge_sendCommand(AUDIO_FORMAT_DOP_256);
            CODEC_DG("AUDIO_FORMAT_DOP_256\n");
        default:
            dev_dbg(codec->dev, "es9018_hw_params:    Unsupported bit size param = %d",
                    params_format(params));
            return -EINVAL;
        }


    }else{/* PCM format */

        switch (params_rate(params)) {
        case 352800:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_352_8K16);
            gfpga_format=AUDIO_SAMPLERATE_352_8K16;
            break;
        case 176400:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_176_4K16);
            gfpga_format=AUDIO_SAMPLERATE_176_4K16;
            break;
        case 88200:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_88_2K16);
            gfpga_format=AUDIO_SAMPLERATE_88_2K16;
            break;
        case 44100:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_44_1K16);
            gfpga_format=AUDIO_SAMPLERATE_44_1K16;
            break;
        case 22050:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_22_05K16);
            gfpga_format=AUDIO_SAMPLERATE_22_05K16;
            break;
        case 384000:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_384K16);
            gfpga_format=AUDIO_SAMPLERATE_384K16;
            break;
        case 192000:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_192K16);
            gfpga_format=AUDIO_SAMPLERATE_192K16;
            break;
        case 96000:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_96K16);
            gfpga_format=AUDIO_SAMPLERATE_96K16;
            break;
        case 48000:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_48K16);
            gfpga_format=AUDIO_SAMPLERATE_48K16;
            break;
        case 24000:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_24K16);
            gfpga_format=AUDIO_SAMPLERATE_24K16;
            break;
        case 32000:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_32K16);
            gfpga_format=AUDIO_SAMPLERATE_32K16;
            break;
        case 64000:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_64K16);
            gfpga_format=AUDIO_SAMPLERATE_64K16;
            break;
        case 128000:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_128K16);
            gfpga_format=AUDIO_SAMPLERATE_128K16;
            break;
        case 8000:
            audio_fpga_bridge_sendCommand( AUDIO_SAMPLERATE_48K16);
        default:
            dev_err(codec->dev, "unsupported sampling rate\n");
            return -EINVAL;
        }
    }
    //DAC need to init 

    for (i=0;i<=100;i++){

        msleep(5);
        iface= snd_soc_read(codec, ES9018_MODE_CONTROL_1);
        if(iface>0)
            break;

    }
    if(i>100)
        dev_err(codec->dev, "DAC i2c error\n");
    /* msleep(350); */
	return 0;
}

static int es9018_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	/* struct es9018_priv *es9018 = snd_soc_codec_get_drvdata(codec); */

    CODEC_TR("%s %d\n",__func__,__LINE__);
	dev_dbg(codec->dev, "es9018_set_dai_sysclk info: freq=%dHz\n", freq);
	
	return 0;
}

static int es9018_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
    // YunxiZhang
    // do not to set fmt
    CODEC_TR("%s %d\n",__func__,__LINE__);
	return 0;
	/* u8 iface = snd_soc_read(codec, ES9018_MODE_CONTROL_1) & 0xCF; */
u8 iface;
	CODEC_DG("%s ----fmt=%x  dai %x \n", __FUNCTION__, fmt,fmt & SND_SOC_DAIFMT_MASTER_MASK);
	/* check master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
        printk(" codec slave");
		break;
    case SND_SOC_DAIFMT_CBM_CFM:
        printk("fpga_master, codec slave");
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x00;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface |= 0x20;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x10;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
		case SND_SOC_DAIFMT_IB_IF:
		case SND_SOC_DAIFMT_IB_NF:
		case SND_SOC_DAIFMT_NB_IF:
			break;
	}

	CODEC_DG("es9018_set_dai_fmt:    Format=%x, Clock Inv=%x reg 10=%x\n",
				fmt & SND_SOC_DAIFMT_FORMAT_MASK,
				((fmt & SND_SOC_DAIFMT_INV_MASK)),iface);

	/* snd_soc_write(codec, ES9018_MODE_CONTROL_1, iface); */
	return 0;
}

static int es9018_digital_mute(struct snd_soc_dai *dai, int mute)
{
    u8 reg=0,i;
	struct snd_soc_codec *codec = dai->codec;

    /* WARN_ON(1); */
    CODEC_TR("%s %d\n",__func__,__LINE__);
    /* reg= snd_soc_read(codec, ES9018_MODE_CONTROL_1) ; */

    /* CODEC_TR("%s  %s 0x0A 0x%x\n",__func__,mute? "mute":"unmute",reg); */

#if 0

    if(mute)
        snd_soc_write(codec,ES9018_MODE_CONTROL_1,reg|0x01);
    else
        snd_soc_write(codec,ES9018_MODE_CONTROL_1,reg&0xFE);

#endif

    if(gpAudioHwLink->mute){

        for(i=0;i<=7;i++){
            snd_soc_write(codec, ES9018_DAC0+i,gpAudioHwLink->vol);
        }
        gpAudioHwLink->mute=false;
    }

    if(!mute)
	    snd_soc_write(es9018_codec, ES9018_MODE_CONTROL_5, 0x14);
	/* reg= snd_soc_read(codec, ES9018_MODE_CONTROL_1) ; */
    /* CODEC_TR("%s  %s 0x0A 0x%x\n",__func__,mute? "mute":"unmute",reg); */
    return 0;

}
static int es9018_trigger(struct snd_pcm_substream *sub, int cmd,
		struct snd_soc_dai *dai){
    u8 reg=0;
	struct snd_soc_codec *codec = dai->codec;


    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:

            AudioBridge_enable();
        CODEC_TR("trigger cmd SNDRV_PCM_TRIGGER_START \n");
        /* do something to start the PCM engine */
        break;
    case SNDRV_PCM_TRIGGER_STOP:
            AudioBridge_disable();
        CODEC_TR("trigger cmd SNDRV_PCM_TRIGGER_STOP \n");
        /* do something to stop the PCM engine */
        break;
    default:
        CODEC_TR("trigger cmd %d \n",cmd);
        return -EINVAL;
    }

    CODEC_TR("%s %d\n",__func__,__LINE__);
    /* WARN_ON(1); */
    return 0;
}

	
#define ES9018_RATES (SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000| \
                      SNDRV_PCM_RATE_24000 | SNDRV_PCM_RATE_128000| \
                      SNDRV_PCM_RATE_64000 | SNDRV_PCM_RATE_44100 | \
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
			SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 | \
            SNDRV_PCM_RATE_384000)

#define ES9018_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_SOP|\
        SNDRV_PCM_FMTBIT_DSD64_F32 |  SNDRV_PCM_FMTBIT_DSD128_F32  |SNDRV_PCM_FMTBIT_DSD256_F32 |\
        SNDRV_PCM_FMTBIT_DSD64_CT24 | SNDRV_PCM_FMTBIT_DSD128_CT24 |SNDRV_PCM_FMTBIT_DSD256_CT24)

static const struct snd_soc_dai_ops es9018_dai_ops = {
	.startup	= es9018_startup,
    .shutdown   = es9018_shutdown,
	.hw_params	= es9018_hw_params,
	.set_sysclk	= es9018_set_dai_sysclk,
	.set_fmt	= es9018_set_dai_fmt,
    .digital_mute=es9018_digital_mute,
    .trigger=es9018_trigger,
};

static struct snd_soc_dai_driver es9018_dai = {
	.name = "ES9018 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,  /* Mono modes not yet supported */
		.channels_max = 2,
		.rates = ES9018_RATES,
		.formats = ES9018_FORMATS,
	},
	.ops = &es9018_dai_ops,
};

/* 音频相关的电源管理的接口函数
 * 增加音频相关的电源管理处理函数: set_es9018_power_off, set_es9018_power_on
 * set_es9018_power_off: 在关闭音频相关电源时调用
 * set_es9018_power_on:  在打开音频相关电源时调用
 */


int set_fpga_power(char on)
{
    struct regulator *ldo_6;
    struct regulator *ldo_io1;

    ldo_6 = regulator_get(NULL, "axp22_dldo2");	//FPGA OSC-3V3
    if(ldo_6 == NULL || IS_ERR(ldo_6)){
        CODEC_TR("fpga power dldo2 set error %s %d\n",__func__,__LINE__);
        return -1;
    }
    else{
        if(on){
            if(regulator_is_enabled(ldo_6)==0){
                regulator_set_voltage(ldo_6,3300000, 3300000);
                regulator_enable(ldo_6);
                printk("AUDIO_LINK: BRIDGE power on IO_3V3\n");
            }else{
                printk("AUDIO_LINK: BRIDGE power IO_3V3 have opened\n");
            }
        }else{
            if(regulator_is_enabled(ldo_6)>=1){
                regulator_disable(ldo_6);
                printk("AUDIO_LINK: BRIDGE power off IO_3V3\n");
            }else{
                printk("AUDIO_LINK: BRIDGE power IO_3V3 have closed\n");
            }
            regulator_put(ldo_6);
        }
    }

    ldo_io1= regulator_get(NULL, "axp22_ldoio1");	//FPGA OSC-1V5
    if(ldo_io1== NULL || IS_ERR(ldo_io1)){
        CODEC_TR("fpga power ldoio1 set error %s %d\n",__func__,__LINE__);
        return -1;
    }else{
        if(on){
            if(regulator_is_enabled(ldo_io1)==0){
                regulator_set_voltage(ldo_io1,1500000, 1500000);
                regulator_enable(ldo_io1);
                gpAudioHwLink->fpga_on=1;
                printk("AUDIO_LINK: BRIDGE power on IO_1V5\n");
            }else{
                printk("AUDIO_LINK: BRIDGE power IO_1V5 have opened\n");
            }
        }
        else{
            if(regulator_is_enabled(ldo_io1)>=1){
                regulator_disable(ldo_io1);
                gpAudioHwLink->fpga_on=0;
                printk("AUDIO_LINK: BRIDGE power off IO_1V5\n");
            }else{
                printk("AUDIO_LINK: BRIDGE power IO_1V5 have closed\n");
            }
        }
        regulator_put(ldo_io1);
    }

    return 0;
}


int set_dac_power(char on)
{


    if(on){
        gpio_set_value(HOST_POWER_PIN, 1);
        gpio_direction_output(ES9018_POWER_PIN, 1);
        mdelay(5);
        gpio_direction_output(ES9018_RESET_PIN,1);
        mdelay(5);
        gpio_direction_output(ES9018_RESET_PIN,0);
        gpAudioHwLink->dac_on=1;
        printk("AUDIO_LINK: DAC power on\n");
    }else{
        gpio_direction_output(ES9018_POWER_PIN, 0);
        gpio_direction_output(ES9018_RESET_PIN,0);

        //when usb is otg mode, don`t disable HOST_POWER_PIN
        if (!is_otg_mode())
            gpio_set_value(HOST_POWER_PIN, 0);
        gpAudioHwLink->dac_on=0;
        printk("AUDIO_LINK: DAC power off\n");
    }

    return 0;
}

static int set_am_power(char on)
{
        gpio_set_value(AM_POWER_PIN, !!on);
        gpAudioHwLink->am_on=on;
        printk("AUDIO_LINK: AM power %s\n",on?"on":"off");
        return 0;
}

static int set_am_op_power(char on)
{
    
        gpio_set_value(AM_OP_EN, !!on);
        gpAudioHwLink->am_op_on=on;
        printk("AUDIO_LINK: AM-OP power %s\n",on?"on":"off");
        return 0;
}

static int set_am_mute(char on)
{
        if (audio_get_am_type() == AM6) {
            gpio_set_value(AM_MUTE_EN, !!!on);
        } else {
            gpio_set_value(AM_MUTE_EN, !!on);
        }
        gpAudioHwLink->am_mute=on;
        printk("AUDIO_LINK: AM %s\n",on?"mute":"unmute");

        return 0;
}

static int set_lineout_mute(char on)
{
        gpio_set_value(LINE_MUTE_EN, !!on);
        gpAudioHwLink->line_mute=on;
        printk("AUDIO_LINK: LINEOUT %s\n",on?"mute":"unmute");
        return 0;
}

static int set_amp_gain(char on)
{
        int status;

        status = audio_get_amp_status();
        if (status) {
                gpio_set_value(RK30_PIN0_PC1, !!on);
        }
        printk("AUDIO_AMP: gain status ==== %s\n", status ? "high":"low");
        printk("AUDIO_AMP: gain gpio level ==== %d\n", gpio_get_value(RK30_PIN0_PC1));

        return 0;
}

static lineout_pre_mute()
{
    set_lineout_mute(1);
}

static lineout_post_mute()
{
    set_lineout_mute(0);
}

// poweron 1: poweron sequence
// poweron 0: poweroff sequence
static int am_pre_mute(char poweron)
{
    if(poweron){
        set_am_op_power(0);
        msleep(50);
        set_am_power(1);
        mdelay(1);
        set_am_mute(1);

        msleep(500);
        set_am_op_power(1);
    }else{

        set_am_power(1);
        set_am_mute(1);
        msleep(50);
        set_am_op_power(0);
    }
}

// poweron 1: poweron sequence
// poweron 0: poweroff sequence
static int am_post_mute(char poweron)
{

    if(poweron){
        /* msleep(1250); */
        set_am_mute(0);
    }else{
        set_am_mute(0);
        set_am_power(0);
    }
}

static int fpga_poweron_init(char on)
{
#ifdef FPGA_ALWAYS_ON
    if(on){
        set_fpga_power(1);
        mdelay(1);
        //set FPGA format latest
        audio_fpga_bridge_sendCommand( FPGA_PRE_CMD);
    }else{

        set_fpga_power(0);
    }
#endif
}

static int dac_power(char on)
{
    if(on){
        gpio_direction_output(ES9018_RESET_PIN,1);
        gpio_set_value(HOST_POWER_PIN, 1);
        msleep(50);
        gpio_direction_output(ES9018_POWER_PIN, 1);
        msleep(50);
        gpio_direction_output(ES9018_RESET_PIN,0);
        gpAudioHwLink->dac_on=1;
        printk("AUDIO_LINK: DAC power on\n");
    }else{
        gpio_direction_output(ES9018_RESET_PIN,1);
        msleep(5);
        gpio_direction_output(ES9018_POWER_PIN, 0);
        msleep(5);

        //when usb is otg mode, don`t disable HOST_POWER_PIN
        if (!is_otg_mode())
            gpio_set_value(HOST_POWER_PIN, 0);
        gpAudioHwLink->dac_on=0;
        printk("AUDIO_LINK: DAC power off\n");
    }

}

//for usb otg detect
int get_dac_status(void) 
{
    return gpAudioHwLink->dac_on;
}

int getLinkPlaybackStatus(void)
{
    return gpAudioHwLink->playback;
}

static int DacPoweronRegInit()
{

    unsigned int ret,count=100;
    char buf[2]={0,0};
    if(es9018_codec){

        do{
            ret=i2c_master_recv(es9018_codec->control_data, buf, 1);
            count--;
            if(ret!=1)
                msleep(5);//10ms

        }while(ret!=1 && count>=0);

        if(count <0)
            CODEC_DG(KERN_ERR "%s i2c error\n",__func__);
        else
           CODEC_DG(KERN_ERR "%s DAC init  %d\n",__func__,count);

        es9018_codec->cache_sync=1;
        snd_soc_cache_sync(es9018_codec);

        snd_soc_write(es9018_codec, ES9018_AUTOMUTE_LEV, 0x68);
        /* snd_soc_write(es9018_codec, ES9018_MODE_CONTROL_1, 0x8e); */
        /* snd_soc_write(es9018_codec, ES9018_DAC_SRC_3478, 0x09); // dac src 1-3 2-4 */
        /* snd_soc_write(es9018_codec, ES9018_SPDIF_SRC, 0x01); // spdif src data1 */
        snd_soc_write(es9018_codec, ES9018_MODE_CONTROL_5, 0x1C); // OSF 8x
        /* snd_soc_write(es9018_codec, ES9018_AUTOMUTE_LPBACK , 0x08);// enable AutoMuteLoopBack */
    }
}

static codec_power_onoff(char poweron)
{
    if(poweron){
        fpga_poweron_init(1);
        dac_power(1);
        DacPoweronRegInit();
    }else{
        fpga_poweron_init(0);
        dac_power(0);
    }
}
/* int set_host_power(char on) */
/* { */
        /* gpio_set_value(HOST_POWER_PIN, !!on); */
        /* return 0; */
/* } */



int set_dac_power_squence(char on)
{

    set_dac_power(on);
    return 0;
}

static int set_AM6_power_ctl(char on)
{
    if (on) {
        set_am_power(1);
        mdelay(1);
        set_am_op_power(1);
        msleep(300);
        set_am_mute(0);
    } else {
        set_am_op_power(0);
        mdelay(1);
        set_am_mute(1);
        msleep(300);
        set_am_power(0);
    }
}

static void set_AM6_power_off(void)
{
    set_am_mute(1);
    msleep(300);
    set_am_op_power(0);
    msleep(100);
    set_am_power(0);
    //msleep(1200);
}

int audioLinkChange(struct audio_hw_link* link,char event)
{

    unsigned int ret,reg;
    char buf[2]={0,0};

    // FPGA power always on
    switch(event){
    case EVENT_PLAYBACK_START:


        CODEC_TR("%s EVENT_PLAYBACK_START\n",__func__);

        //set amp gain, add by lipf
        set_amp_gain(1);
        //add end

        if(link->linein  ){

            if( !link->fpga_on){
                lineout_pre_mute();
                /* am_pre_mute(0); */

                codec_power_onoff(1);

                msleep(1200);
                lineout_post_mute();
                /* am_post_mute(0); */
            }

            if(!link->dac_on){
                lineout_pre_mute();

                codec_power_onoff(1);

                msleep(1200);
                lineout_post_mute();
            }
        }else if(link->headphone ){

            if(!link->dac_on){
                lineout_pre_mute();

                if (audio_get_am_type() == AM6) {
                    set_AM6_power_ctl(0);
                } else {
                    am_pre_mute(1);
                }

                codec_power_onoff(1);
                msleep(1200);

                if (audio_get_am_type() == AM6) {
                    set_AM6_power_ctl(1);
                } else {
                    am_post_mute(1);
                }
                
                lineout_post_mute();
            }


            // not device inseted
        }else{

            codec_power_onoff(1);

        }
    link->playback=1;
    /* msleep(1000); */

        break;

    case EVENT_PLAYBACK_OFF:
        CODEC_TR("%s EVENT_PLAYBACK_OFF\n",__func__);

        //set amp gain, add by lipf
        set_amp_gain(0);
        //add end

        if(link->headphone){
            if (audio_get_am_type() == AM6) {
                set_AM6_power_off();
            } else {
                am_pre_mute(0);
            }
        }

        if(link->linein)
            set_lineout_mute(1);

        codec_power_onoff(0);
        msleep(1200);

        if(link->headphone){
            if (audio_get_am_type() != AM6) {
                am_post_mute(0);
            }
        }

        if(link->linein)
            set_lineout_mute(0);

        link->playback=0;
        break;

    case EVENT_HP_IN:
        // no matter playback is off open power
        link->headphone=1;

        if(!link->playback){
            set_amp_gain(0);
        }

        if(!gAndroid_boot_completed)
            break;
        CODEC_TR("%s EVENT_HP_IN  fpga %d\n",__func__,link->fpga_on);

        if(link->linein)// diable am output if lineout insert
            break;

        if(audio_get_am_type() == AM6)// diable am output if am_type is AM6
        {
            CODEC_TR("%s Current AM tpye is AM6, break...\n",__func__);
            break;
        }

        am_pre_mute(1);

        if(!link->fpga_on){
            codec_power_onoff(1);
        }

        if(!link->dac_on){
            codec_power_onoff(1);
        }

        msleep(1500);
        am_post_mute(1);

        if(!link->playback){
            wake_lock(&audiolink_wakelock);
            schedule_delayed_work(&es9018_delay_work,msecs_to_jiffies(AUDIO_LINK_HP_DELAY* 1000));
            printk("AUDIO_LINK: schedule es9018_delay_work\n");
        }

        break;

    case EVENT_HP_OUT:
        link->headphone=0;
        if(!gAndroid_boot_completed)
            break;

        ret=work_busy((struct work_struct*)&es9018_delay_work);
        if(ret&WORK_BUSY_RUNNING){

            printk(KERN_WARNING "9018 work_func WORK_BUSY_RUNNING wait it to finish\n");
            flush_delayed_work_sync(&es9018_delay_work);
            break;
        }
        if(ret&WORK_BUSY_PENDING){
            printk(KERN_WARNING "9018 work_func WORK_BUSY_PENDING cancel it\n");
            cancel_delayed_work_sync(&es9018_delay_work);
            wake_unlock(&audiolink_wakelock);
        }


        if(link->linein)
            set_lineout_mute(1);


        am_pre_mute(0);

        //keep poweron if playback
        if(!link->playback)
            codec_power_onoff(0);

        msleep(1500);
        if(link->linein)
            set_lineout_mute(0);

        am_post_mute(0);

        break;


    case EVENT_LINEOUT_IN:
        link->linein=1;
        CODEC_TR("%s EVENT_LINEOUT_IN\n",__func__);
        if(!gAndroid_boot_completed)
            break;

        set_lineout_mute(1);
        if(!link->fpga_on){
            codec_power_onoff(1);
        }

        if(!link->dac_on){
            codec_power_onoff(1);
        }

        // no matter headphone in shutdown
        if(link->am_on){
            am_pre_mute(0);
            msleep(1600);
            am_post_mute(0);
            set_lineout_mute(0);
        }else{
            msleep(1000);
            set_lineout_mute(0);
        }


        link->linein=1;

        if(!link->playback){
            wake_lock(&audiolink_wakelock);
            schedule_delayed_work(&es9018_delay_work,msecs_to_jiffies(AUDIO_LINK_HP_DELAY* 1000));
            printk("AUDIO_LINK: schedule es9018_delay_work\n");
        }


        break;

    case EVENT_LINEOUT_OUT:
        link->linein=0;
        if(!gAndroid_boot_completed)
            break;
        CODEC_TR("%s EVENT_LINEOUT_OUT\n",__func__);

        ret=work_busy((struct work_struct*)&es9018_delay_work);
        if(ret&WORK_BUSY_RUNNING){

            printk(KERN_WARNING "9018 work_func WORK_BUSY_RUNNING wait it to finish\n");
            flush_delayed_work_sync(&es9018_delay_work);
            break;
        }
        if(ret&WORK_BUSY_PENDING){
            printk(KERN_WARNING "9018 work_func WORK_BUSY_PENDING cancel it\n");
            cancel_delayed_work_sync(&es9018_delay_work);
            wake_unlock(&audiolink_wakelock);
        }


        if(link->headphone && link->playback){

            set_lineout_mute(1);
            am_pre_mute(1);
            msleep(1300);
            am_post_mute(1);
            set_lineout_mute(0);
        }else{
            set_lineout_mute(1);
            msleep(500);
            set_lineout_mute(0);
        }

        //keep poweron if don`t playback
        if(!link->playback)
            codec_power_onoff(0);

        link->linein=0;

        break;
    }

   CODEC_TR("link->playback %d\nlink->headphone %d\nlink->linein %d\n",
            link->playback,link->headphone,link->linein);
   CODEC_TR("link->fpga_on %d\nlink->dac_on %d\nlink->line_mute %d\n",
            link->fpga_on,link->dac_on,link->line_mute);
   CODEC_TR("link->am_on %d\nlink->am_op_on %d\nlink->am_mute %d\n",
            link->am_on,link->am_op_on,link->am_mute);
}



void exAudioLinkChange(char event)
{
    if( gpAudioHwLink)
        audioLinkChange(gpAudioHwLink,event);
    else
        CODEC_TR("gpAudioHwLink is NULL !!\n");
}
EXPORT_SYMBOL_GPL(exAudioLinkChange);




int set_am_power_init(char on)
{



    if(on){

        set_am_power(1);
        set_am_op_power(1);
        printk("AUDIO_LINK: AM power on\n");

    }else{

        set_am_op_power(0);
        set_am_power(0);
        printk("AUDIO_LINK: AM power off\n");
    }

    return 0;
}

//
int set_audiolink_power_on(void)
{


        set_fpga_power(1);
        set_dac_power_squence(1);
        set_am_power_init(1);

        return 0;
}

//user for driver shutdown when reboot
int set_audiolink_power_off(void)
{

    // when reboot dac poweron ,so mute am befor 
    // shutdown power.
    if(audio_get_am_type() == AM1){
        printk(KERN_ERR "poweroff AM1\n");
        am_pre_mute(0);
        msleep(100);
        /* set_fpga_power(0); */
        AudioBridge_disable();
        set_dac_power_squence(0);
        /* set_am_power_init(0); */
        set_am_op_power(0);
        msleep(300);
        am_post_mute(0);

    }else if(audio_get_am_type() == AM2){
        /****************************************************
         * open mute power and op power befor close am power
         * 
         * so that the power stored in capacity will discharge 
         * completely, Otherwise the AM_OP_EN will set to 
         * hight level  when reboot that will cause popo noise.
         * **************************************************/
        printk(KERN_ERR "poweroff AM2\n");
        set_am_mute(1);
        set_am_op_power(1);
        msleep(100);
        /* set_fpga_power(0); //diable audio output */
        AudioBridge_disable();
        set_dac_power_squence(0);
        msleep(20);
        set_am_op_power(0);
        msleep(720);

    /* if(board_boot_mode() != BOOT_MODE_RECOVERY) */
        //reboot mode
        //if( sys_reboot_cmd == LINUX_REBOOT_CMD_RESTART2)
        //    set_am_power(1);
        //else 
            set_am_power(0);
        msleep(500);
        /* set_am_op_power(0); */
        set_am_mute(0);
    }else if (audio_get_am_type() == AM6){
        printk(KERN_ERR "poweroff AM6\n");
        set_am_mute(1);
        msleep(20);
        set_am_op_power(1);

        /* set_fpga_power(0); //diable audio output */
        AudioBridge_disable();
        set_am_op_power(0);
        msleep(500);
        //set_am_power(0);
        /* set_am_op_power(0); */
        set_am_power(0);
        msleep(500);
        //set_am_mute(0);
        //msleep(500);
    } else {
        printk(KERN_ERR "poweroff AMx %d",audio_get_am_type());
        /* set_fpga_power(0); //diable audio output */
        AudioBridge_disable();
        set_am_mute(1);
        msleep(20);
        set_am_op_power(1);
        msleep(720);
        set_am_power(0);
        msleep(500);
        set_am_op_power(0);
    }

  return 0;
}
int set_am_power_sequence(struct audio_hw_link* link,char on)
{



    if(on){

        /* set_am_op_power(0); */
        set_am_power(1);
        set_am_mute(1);
        msleep(50);

        set_am_op_power(1);
        msleep(850);// power to  stable state
        set_am_mute(0);
        link->am_on=1;
        printk("AUDIO_LINK: AM power on\n");

    }else{

        set_am_mute(1);
        msleep(100);
        set_am_op_power(0);
        msleep(800);

        msleep(50);
        /* set_am_power(0); */
        link->am_on=0;
        printk("AUDIO_LINK: AM power off\n");
    }

    return 0;
}

#if defined(CONFIG_KP_AXP22)
#define  AXP_DLDO2_REG    0x12
#define  AXP_LDOIO1_REG   0x92
static void init_osc_power(void)
{
        //disable dldo2 for OSC_3V3
        axp_clr_bits(&axp->dev, AXP_DLDO2_REG, 0x10);

        //disable ldoio1 for OSC_1V5
        axp_write(&axp->dev, AXP_LDOIO1_REG, 0x04);
}
#endif




#ifdef CONFIG_PM

static int es9018_suspend(struct snd_soc_codec *codec, pm_message_t state)
{	
        //关闭音频相关电源
        /* set_es9018_power_off(); */

	return 0;
}

static int es9018_resume(struct snd_soc_codec *codec)
{
        //打开音频相关电源
        /* set_es9018_power_on(); */

	/* snd_soc_cache_sync(codec); */
	return 0;
}
#else
#define es9018_resume NULL
#endif

/* write the register to cache*/
static int es9018_cache_reg_init(struct snd_soc_codec *codec)
{
	int ret,  i;

	for(i=0; i<ARRAY_SIZE(es9018_reg_defaults); i++)
	{
		ret = snd_soc_cache_write(codec, es9018_reg_defaults[i].reg, es9018_reg_defaults[i].value);
		if (ret < 0)
			printk("%s-%d: write reg%2d failed!\n", __FUNCTION__, __LINE__, i);
	}
}


int es9018_i2c_master_send(const struct i2c_client *client, const char *buf, int count)
{
    CODEC_DG("es9018_i2c_master_send reg:%d data:%d %d dac %d\n",buf[0],buf[1],count,gpAudioHwLink->dac_on);
    if(!gpAudioHwLink->dac_on){
        printk("DAC is poweroff  reg[%d] not accesable\n",buf[0]);
        return count;
    }
    //if(buf[0]==0 && !gpAudioHwLink->vol_mute)
    //    gpAudioHwLink->vol=buf[1];

   return i2c_master_send(client,buf,count);
}
struct snd_soc_cache_ops es9018_cache_ops={
	.name="9018_cache",
	.id=SND_SOC_FLAT_COMPRESSION,
	/* int (*init)(struct snd_soc_codec *codec); */
	/* int (*exit)(struct snd_soc_codec *codec); */
	/* int (*read)(struct snd_soc_codec *codec, unsigned int reg, */
		/* unsigned int *value); */
	/* int (*write)(struct snd_soc_codec *codec, unsigned int reg, */
		/* unsigned int value); */
	/* int (*sync)(struct snd_soc_codec *codec); */
};


static int es9018_probe(struct snd_soc_codec *codec)
{
	struct es9018_priv *es9018 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	
    ret = snd_soc_codec_set_cache_io(codec, 8, 8, es9018->control_type);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		goto err_enable;
	}

	/* codec->hw_write = (hw_write_t)i2c_master_send; */
	codec->hw_write = (hw_write_t)es9018_i2c_master_send;
	/* codec->hw_read= (hw_write_t)i2c_master_recv; */
	codec->control_data = container_of(codec->dev, struct i2c_client, dev);
    /* codec->cache_only=1; */
    /* codec->compress_type */
	
    
#if 0
    set_am_power(1);
    set_am_op_power(1);
    msleep(50);
    set_am_mute(1);
    msleep(50);
    set_es9018_power_on();
    audio_fpga_bridge_sendCommand(AUDIO_SAMPLERATE_44_1K16);
    msleep(100);
    set_am_mute(0);
#else

    /* set_am_power(0); */
    /* set_am_op_power(0); */
    /* audio_fpga_bridge_sendCommand(AUDIO_SAMPLERATE_44_1K16); */
    /* set_es9018_power_on(); */

#endif

    // when reboot ,am power maybe enabled
    set_am_mute(1);
    msleep(10);
    set_am_op_power(0);
    msleep(500);
    set_am_power(0);
    // op gpio in high level when SOC poweron
    /* msleep(10); */
    /* set_audiolink_power_off(); */

#if defined(CONFIG_KP_AXP22)
        init_osc_power();
#endif

#ifdef FPGA_ALWAYS_ON
        set_fpga_power(1);
        mdelay(1);
        audio_fpga_bridge_sendCommand(AUDIO_SAMPLERATE_44_1K16);
#endif
	ret = es9018_reset(codec);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to issue reset\n");
		goto err_enable;
	}
	/* snd_soc_write(codec, ES9018_AUTOMUTE_LEV, 0x68); */
	/* snd_soc_write(codec, ES9018_MODE_CONTROL_1, 0x8e); */
	/* snd_soc_write(codec, ES9018_DAC_SRC_3478, 0x09); // dac src 1-3 2-4 */
	/* snd_soc_write(codec, ES9018_SPDIF_SRC, 0x01); // spdif src data1 */
	//snd_soc_write(codec, ES9018_MODE_CONTROL_5, 0x5C); // OSF 8x
	/* snd_soc_write(codec, ES9018_AUTOMUTE_LPBACK , 0x08);// enable AutoMuteLoopBack */
	es9018_codec = codec;
	ret = 0;
	dev_dbg(codec->dev, "Successful registration\n");

    es9018_cache_reg_init(es9018_codec);
    wake_lock_init(&audiolink_wakelock, WAKE_LOCK_SUSPEND, "audiolink");

        //Init FPGA end and power off FPGA, DAC
        //wake_lock(&audiolink_wakelock);
        //schedule_delayed_work(&es9018_delay_work,msecs_to_jiffies(AUDIO_LINK_SHUTDOWN_DELAY * 1000));

	return ret;

err_enable:
	return ret;
}

static int es9018_remove(struct snd_soc_codec *codec)
{
	/* struct es9018_priv *es9018 = snd_soc_codec_get_drvdata(codec); */
	return 0;
}
static int es9018_volatile_reg(struct snd_soc_codec *codec, unsigned int reg)
{
    if(reg<REG_CONFIG_MAX) 
        return 0;
    else
        return 1;
}
static int es9018_readable_reg(struct snd_soc_codec *codec, unsigned int reg)
{
    if(reg<REG_CONFIG_MAX) 
        return 1;
    else
        return 0;
}
static int es9018_writable_reg(struct snd_soc_codec *codec, unsigned int reg)
{
    if(reg<REG_CONFIG_MAX) 
        return 1;
    else
        return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es9018 = {
	.probe =	es9018_probe,
	.remove =	es9018_remove,
        .suspend =	es9018_suspend,
	.resume =	es9018_resume,

    .controls = es9018_snd_controls,
    .num_controls = ARRAY_SIZE(es9018_snd_controls),
    .volatile_register=es9018_volatile_reg,
    .readable_register=es9018_readable_reg,
    /* .writable_register=es9018_writable_reg, */
    .reg_cache_size=REG_CONFIG_MAX,
    .reg_word_size=1,
};

/* static const struct of_device_id es9018_of_match[] = { */
	/* { .compatible = "ess,es9018", }, */
	/* { } */
/* }; */
/* MODULE_DEVICE_TABLE(of, es9018_of_match); */

/* static const struct regmap_config es9018_regmap = { */
	/* .reg_bits = 8, */
	/* .val_bits = 8, */
	/* .max_register = ES9018_MAX_REGISTER, */

	/* .reg_defaults = es9018_reg_defaults, */
	/* .num_reg_defaults = ARRAY_SIZE(es9018_reg_defaults), */
	/* .cache_type = REGCACHE_RBTREE, */

	/* .readable_reg = es9018_readable, */
/* }; */

#if defined(CONFIG_I2C)
static int es9018_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
    struct es9018_priv *es9018;
	int ret;
	

   ret=gpio_request(AM_POWER_PIN,"AM power");
    if (ret != 0) {
        printk(">>>>gpio request Am power faile !!\n");
    }
 
    ret=gpio_request(AM_OP_EN,"AM op power EN");
    if (ret != 0) {
        printk(">>>>gpio request AM op power EN faile !!\n");
    }

    ret=gpio_request(AM_MUTE_EN,"AM mute EN");
    if (ret != 0) {
        printk(">>>>gpio request am mute EN faile !!\n");
    }

    ret=gpio_request(ES9018_POWER_PIN,"ES9018 power");
    if (ret != 0) {
        printk(">>>>gpio request ES9018 power faile !!\n");
    }

    ret=gpio_request(ES9018_RESET_PIN,"ES9018 reset");
    if (ret != 0) {
        printk(">>>>gpio request ES9018 reset faile !!\n");
    }
    ret=gpio_request(LINE_MUTE_EN,"ES9018 line mute");
    if (ret != 0) {
        printk(">>>>gpio request ES9018 line mute faile !!\n");
    }
    ret=gpio_request(HOST_POWER_PIN,"HOST_POWER_PIN");
    if (ret != 0) {
        printk(">>>>gpio request HOST_POWER_PIN faile !!\n");
    }
    
    /* gpio_direction_output(AM_OP_EN,0); */
    gpio_direction_output(AM_MUTE_EN,0);
    /* gpio_direction_output(AM_POWER_PIN,0); */

    /* gpio_direction_output(LINE_MUTE_EN,1); */
    /* gpio_direction_output(ES9018_RESET_PIN,0); */
    /* gpio_direction_output(ES9018_POWER_PIN,0); */

    // HOST_POWER always on for usbhost
    gpio_direction_output(HOST_POWER_PIN,1);

	gpAudioHwLink= kzalloc(sizeof(struct audio_hw_link ), GFP_KERNEL);
	if (gpAudioHwLink== NULL)
		return -ENOMEM;
 
	es9018 = kzalloc(sizeof(struct es9018_priv), GFP_KERNEL);
	if (es9018 == NULL)
		return -ENOMEM;

	printk("%s-%d: step\n", __FUNCTION__, __LINE__);


	
	
	i2c_set_clientdata(i2c, es9018);
	es9018->control_type = SND_SOC_I2C;

	ret = snd_soc_register_codec(&i2c->dev,
				     &soc_codec_dev_es9018, &es9018_dai, 1);

    /* printk("reg_cache %p",es9018_codec->reg_cache); */
    /* msleep(50); */
    /* es9018_cache_reg_init(es9018_codec); */

	if (ret < 0) {
		kfree(es9018);
		return ret;
	}

	printk("%s-%d: step\n", __FUNCTION__, __LINE__);
	return ret;
}

static int es9018_i2c_remove(struct i2c_client *client)
{
	printk(KERN_ERR"%s-%d: step\n", __FUNCTION__, __LINE__);
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static int es9018_i2c_shutdown(struct i2c_client *client)
{

    if(gpAudioHwLink->am_on)
        set_audiolink_power_off();
	printk(KERN_ERR"%s-%d: step\n", __FUNCTION__, __LINE__);


}
static const struct i2c_device_id es9018_i2c_id[] = {
	{ "es9018", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es9018_i2c_id);

static struct i2c_driver es9018_i2c_driver = {
	.driver = {
		.name = "ES9018",
		.owner = THIS_MODULE,
		/* .of_match_table = es9018_of_match, */
	},
	.probe =    es9018_i2c_probe,
	.remove =   es9018_i2c_remove,
    .shutdown = es9018_i2c_shutdown,
	.id_table = es9018_i2c_id,
};
#endif

static int __init es9018_modinit(void)
{
	int ret = 0;

#if defined(CONFIG_I2C)
	ret = i2c_add_driver(&es9018_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register ES9018 I2C driver: %d\n", ret);
#endif
	return ret;
}
module_init(es9018_modinit);

static void __exit es9018_exit(void)
{
#if defined(CONFIG_I2C)
	i2c_del_driver(&es9018_i2c_driver);
#endif
}
module_exit(es9018_exit);

MODULE_DESCRIPTION("ASoC es9018 driver");
MODULE_AUTHOR("kuangwenxu <kuangwenxu@163.com>");
MODULE_LICENSE("GPL");

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

int es9018_get_volume(unsigned char dac)
{
	return snd_soc_read(es9018_codec, dac);
}

void es9018_set_volume(int flag)
{
	unsigned char reg,val;
	switch(flag){
		case 0:
		case 1:
			reg = ES9018_DAC0;
			break;
		case 2:
		case 3:
			reg = ES9018_DAC1;
			break;
		case 4:
		case 5:
			reg = ES9018_DAC2;
			break;
		case 6:
		case 7:
			reg = ES9018_DAC3;
			break;
		default:
			return;		
	}
	val = es9018_get_volume(reg);
	if((flag%2)==0)
	{
		if(val>=250)
			return;
		val +=10;	//reduce 5 dB
		snd_soc_write(es9018_codec, reg, val);	
	}
	else
	{
		if(val<=0)
			return;
		val -= 10;	//plus 5dB
		snd_soc_write(es9018_codec, reg, val);	
	}
}

int es9018_get_mode_ctl1(int param)
{
	unsigned char val;
	val = snd_soc_read(es9018_codec, ES9018_MODE_CONTROL_1);
	if(param == 1)	//	de-emphasis
	{
		val &= 0x2;
		val >>= 1;
		return val;
	}
	else if(param == 2)	// jitter
	{
		val &= 0x4;
		val >>= 2;
		return val;
	}
	return -1;
}

void es9018_set_mode_ctl1(int param, int param2)
{
	unsigned char val;
	val = snd_soc_read(es9018_codec, ES9018_MODE_CONTROL_1);
	switch(param){
		case 1:		//	default: 1 by de-emphasis	
			val &= (~0x2);
			val |= ((param2<<1)&0x2);
			break;
		case 2:		// default: 1 use jitter filter
			val &= (~0x4);
			val |= ((param2<<2)&0x4);
			break;
		default:
			return;	
	}
	snd_soc_write(es9018_codec, ES9018_MODE_CONTROL_1, val);
}

int es9018_get_mode_ctl2(int param)
{
	unsigned char val;
	val = snd_soc_read(es9018_codec, ES9018_MODE_CONTROL_2);
	if(param == 1)	// de-emphasis delect
	{
		val &= 0x3;
		return val;
	}
	else if(param == 2)	// dpll bandwidth
	{
		val &= 0x1c;
		val >>= 2;
		return val;
	}
	return -1;
}

void es9018_set_mode_ctl2(int param, int param2)
{
	unsigned char val;
	val = snd_soc_read(es9018_codec, ES9018_MODE_CONTROL_2);
	switch(param){
		case 1:		// de-emphasis delect default 44.1k b01
			val &= (~0x3);
			val |= (param2&0x3);
			break;
		case 2:		// dpll bandwidth default lowest b001
			val &= (~0x1c);
			val |= ((param2<<2)&0x1c);
			break;
		default:
			return;	
	}
	snd_soc_write(es9018_codec, ES9018_MODE_CONTROL_2, val);
}

int es9018_get_iir_fir(int param)
{
	unsigned char val;
	val = snd_soc_read(es9018_codec, ES9018_DAC_SRC_3478);
	if(param == 1)	// fir rolloff speed default fast
	{
		val &= 0x1;
		return val;
	}
	else if(param == 2)	// iir bandwidth default 50k
	{
		val &= 0x6;
		val >>= 1;
		return val;
	}
	return -1;
}

void es9018_set_iir_fir(int param, int param2)
{
	unsigned char val;
	val = snd_soc_read(es9018_codec, ES9018_DAC_SRC_3478);
	switch(param){
		case 1:		// fir rolloff speed default fast
			val &= (~0x1);
			val |= (param2&0x1);
			break;
		case 2:		// iir bandwidth default 50k
			val &= (~0x6);
			val |= ((param2<<1)&0x6);
			break;
		default:
			return;	
	}
	snd_soc_write(es9018_codec, ES9018_DAC_SRC_3478, val);
}

int es9018_get_dacsrc(int param)
{
	unsigned char val;
	val = snd_soc_read(es9018_codec, ES9018_DAC_SRC_3478);
	if(param == 3)	// 
	{
		val &= 0x10;
		val >>= 4;
		return val;
	}
	else if(param == 4)	//
	{
		val &= 0x20;
		val >>= 5;
		return val;
	}
	else if(param == 7)	// 
	{
		val &= 0x40;
		val >>= 6;
		return val;
	}
	else if(param == 8)	// 
	{
		val &= 0x80;
		val >>= 7;
		return val;
	}
	return -1;
}

void es9018_set_dacsrc(int param, int param2)
{
	unsigned char val;
	val = snd_soc_read(es9018_codec, ES9018_DAC_SRC_3478);
	switch(param){
		case 3:		// fir rolloff speed default fast
			val &= (~0x10);
			val |= ((param2<<4)&0x10);
			break;
		case 4:		// iir bandwidth default 50k
			val &= (~0x20);
			val |= ((param2<<5)&0x20);
			break;
		case 7:		// iir bandwidth default 50k
			val &= (~0x40);
			val |= ((param2<<6)&0x40);
			break;
		case 8:		// iir bandwidth default 50k
			val &= (~0x80);
			val |= ((param2<<7)&0x80);
			break;
		default:
			return;	
	}
	snd_soc_write(es9018_codec, ES9018_DAC_SRC_3478, val);
}

int es9018_get_mode_ctl5(int param)
{
	unsigned char val;
	val = snd_soc_read(es9018_codec, ES9018_MODE_CONTROL_5);
	if(param == 1)	// osf bypass
	{
		val &= 0x40;
		val >>= 6;
		return val;
	}
	return -1;
}

void es9018_set_mode_ctl5(int param, int param2)
{
	unsigned char val;
	val = snd_soc_read(es9018_codec, ES9018_MODE_CONTROL_5);
	switch(param){
		case 1:		// fir rolloff speed default fast
			val &= (~0x40);
			val |= ((param2<<6)&0x40);
			break;
		default:
			return;	
	}
	snd_soc_write(es9018_codec, ES9018_MODE_CONTROL_5, val);
}

/* static int proc_dac_show(struct seq_file *s, void *v) */
/* {      */
	/* return 0; */
/* } */

//ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
static ssize_t proc_dac_read(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	char* temp,*p;
	int r = 0,i,num=0;
	u8 reg, val;
	
	if(*ppos){
		return 0;
	}

	temp  = (char*)__get_free_page(GFP_KERNEL);
	if(!temp) { 				 
		printk(KERN_ALERT"Failed to alloc page.\n");  
		return -ENOMEM;  
	}	

	r = snprintf(temp, PAGE_SIZE, "set_deemphasis=%d set_jitter=%d \
		set_dpdl=%d set_dpllbw=%d set_firsp=%d set_iirbw=%d \
		set_dac3_src=%d set_dac4_src=%d set_dac7_src=%d \
		set_dac8_src=%d set_osf=%d\n", 
		es9018_get_mode_ctl1(1),
		es9018_get_mode_ctl1(2),
		es9018_get_mode_ctl2(1),
		es9018_get_mode_ctl2(2),
		es9018_get_iir_fir(1),
		es9018_get_iir_fir(2),
		es9018_get_dacsrc(3),
		es9018_get_dacsrc(4),
		es9018_get_dacsrc(7),
		es9018_get_dacsrc(8),
		es9018_get_mode_ctl5(1)
		);

    p = temp+strlen(temp);
    printk(KERN_ERR "zzdts %p  %p %d\n",p,temp,strlen(temp));
	for(i = 1; i <= 72/*ARRAY_SIZE(es9018_reg)*/; i++)
	{
		reg = i-1;
		if( reg%4 == 0)
		{
			num+= sprintf(p + num, "/* %02d */   ", reg);
		}
		i2c_master_send(es9018_codec->control_data, &reg, 1);
		i2c_master_recv(es9018_codec->control_data, &val, 1);
        num+= sprintf(p + num, "0x%02x,", val);
		if( i%4 == 0)
		{
			num+= sprintf(p + num, "\n", val);
		}
	}

    r=strlen(temp);

	if(copy_to_user(user_buf,temp,r+1))
	{
		printk(KERN_ALERT"Failed to read.\n");
		r = -EFAULT;
	}
  
	free_page((unsigned long)temp);
	*ppos += r;
	return r;
}

//ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
static ssize_t proc_dac_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	int err = 0, v = 0;  
    static int reg=0;
	char *page = NULL;	
	char *q = NULL, *name = NULL, *value = NULL;

	if(count > PAGE_SIZE) {  
		printk(KERN_ALERT"The buff is too large: %u.\n", count);  
		return -EFAULT;  
	}  

	page = (char*)__get_free_page(GFP_KERNEL);	
	if(!page) { 				 
		printk(KERN_ALERT"Failed to alloc page.\n");  
		return -ENOMEM;  
	}		   

	if(copy_from_user(page, user_buf, count)) {  
		printk(KERN_ALERT"Faidac to copy buff from user.\n");				   
		err = -EFAULT;	
		goto out;  
	} 
	page[count] = 0; 

	q = page;
	while (true){
		value = strsep(&q, " ");
		name = strsep(&value, "=");
		if (name && value){
			printk("%s=%s\n", name, value);
			if (strcmp(name, "set_vol")==0){
				sscanf(value, "%d", &v);
				es9018_set_volume(v);
			}else if (strcmp(name, "set_deemphasis")==0){
				sscanf(value, "%d", &v);
				es9018_set_mode_ctl1(1,v);
			}else if (strcmp(name, "set_jitter")==0){
				sscanf(value, "%d", &v);
				es9018_set_mode_ctl1(2,v);
			}else if (strcmp(name, "set_dpdl")==0){
				sscanf(value, "%d", &v);
				es9018_set_mode_ctl2(1,v);
			}else if (strcmp(name, "set_dpllbw")==0){
				sscanf(value, "%d", &v);
				es9018_set_mode_ctl2(2,v);
			}else if (strcmp(name, "set_firsp")==0){
				sscanf(value, "%d", &v);
				es9018_set_iir_fir(1,v);
			}else if (strcmp(name, "set_iirbw")==0){
				sscanf(value, "%d", &v);
				es9018_set_iir_fir(2,v);
			}else if (strcmp(name, "set_dac3_src")==0){
				sscanf(value, "%d", &v);
				es9018_set_dacsrc(3,v);
			}else if (strcmp(name, "set_dac4_src")==0){
				sscanf(value, "%d", &v);
				es9018_set_dacsrc(4,v);
			}else if (strcmp(name, "set_dac7_src")==0){
				sscanf(value, "%d", &v);
				es9018_set_dacsrc(7,v);
			}else if (strcmp(name, "set_dac8_src")==0){
				sscanf(value, "%d", &v);
				es9018_set_dacsrc(8,v);
			}else if (strcmp(name, "set_osf")==0){
				sscanf(value, "%d", &v);
				es9018_set_mode_ctl5(1,v);
			}
			else if (strcmp(name, "reg")==0){
				sscanf(value, "%x", &reg);
			}
			else if (strcmp(name, "data")==0){
				sscanf(value, "%x", &v);
                if(reg<=71)
                    es9018_reg_write(es9018_codec,reg,v);
			}
		}
		if (q==NULL)
			break;
	}
	   
	err = count;

	out:  
	free_page((unsigned long)page);  
	return count; 
}

/* static int proc_dac_open(struct inode *inode, struct file *file) */
/* { */
	/* return single_open(file, proc_dac_show, NULL); */
/* } */

static const struct file_operations proc_dac_fops = {
	//.open		= proc_dac_open,
	.read		= proc_dac_read,
	.write		= proc_dac_write,
};

static int __init create_dac_proc(void)
{
	proc_create("es9018-dac", 0666, NULL, &proc_dac_fops);
	return 0;
}
late_initcall(create_dac_proc);
