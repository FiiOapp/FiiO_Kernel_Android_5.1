/*
 * rk29_ak4490.c  --  SoC audio for rockchip
 *
 * Driver for rockchip ak4490 audio
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 */

#include <linux/module.h>

#define DEBUG
#include <linux/device.h>
#define DEBUG
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/io.h>
#include <mach/hardware.h>
//#include <mach/rk29_iomap.h>
/* #include <linux/tchip_sysinf.h> */
#include "rk29_pcm.h"
#include "rk29_i2s.h"
#include <linux/delay.h>
#include <sound/pcm_params.h>

#include <mach/iomux.h>
#include <mach/gpio.h>

#if 1
#define	DBG(x...)	printk(x)
#else
#define	DBG(x...)
#endif

extern void ct7302_rate_set(int rate);
//static void *rk29_speaker = NULL;

static bool isI2sFormat_LJF(snd_pcm_format_t fmt)
{
    switch(fmt){
    case SNDRV_PCM_FORMAT_DSD64_F32:   
    case SNDRV_PCM_FORMAT_DSD128_F32:  
    case SNDRV_PCM_FORMAT_DSD256_F32:  
    case SNDRV_PCM_FORMAT_SOP:  
        return true;
        break;
    default:
        return false;

                                  
    /* case SNDRV_PCM_FORMAT_DSD64_CT24:  */
    /* case SNDRV_PCM_FORMAT_DSD128_CT24: */
    /* case SNDRV_PCM_FORMAT_DSD256_CT24: */

    }

}
static bool isDSDFormat(snd_pcm_format_t fmt)
{
    switch(fmt){
    case SNDRV_PCM_FORMAT_DSD64_F32:   
    case SNDRV_PCM_FORMAT_DSD128_F32:  
    case SNDRV_PCM_FORMAT_DSD256_F32:  
    /* case SNDRV_PCM_FORMAT_SOP:   */
        return true;
    default:
        return false;
    }

}


static int rk29_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;
	unsigned int pll_out = 0; 
	int div_bclk,div_mclk;
//	struct clk	*general_pll;
	
	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);    
	/*by Vincent Hsiung for EQ Vol Change*/
	#define HW_PARAMS_FLAG_EQVOL_ON 0x21
	#define HW_PARAMS_FLAG_EQVOL_OFF 0x22
	if ((params->flags == HW_PARAMS_FLAG_EQVOL_ON)||(params->flags == HW_PARAMS_FLAG_EQVOL_OFF))
	{
		ret = codec_dai->driver->ops->hw_params(substream, params, codec_dai); //by Vincent
	}
	else
	{
		/* set codec DAI configuration */
		DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
		#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE) 
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS); 
		#elif defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 



        if(isDSDFormat((params_format(params)))){

		    ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSD|
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);

        }else if(params_format(params)==SNDRV_PCM_FORMAT_SOP){

		    ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_SOP|
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);

        } else{
		    ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM ); 
        }
		#endif
		if (ret < 0)
			return ret; 



		/* set cpu DAI configuration */
		DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
		#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE) 
        ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
            SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
		#elif defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 

        if(isI2sFormat_LJF((params_format(params)))){

            if(params_format(params) == SNDRV_PCM_FORMAT_SOP)
		        ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_LSB_LEFT_J|SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS ); // spdif LSB first
            else
		        ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_LSB_LEFT_J |SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS); //dsd MSB first

        } else{
            ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                                     SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);	
        }
		#endif		
		if (ret < 0)
			return ret;
	}

		DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	switch(params_rate(params)) {
        case 8000:
        case 16000:
        case 24000:
        case 32000:
        case 48000:
        case 64000:
        case 96000:
            pll_out = 12288000*2;
            break;
        case 11025:
        case 22050:
        case 44100:
        case 88200:
            pll_out = 11289600*2;
            break;
        case 176400:
			pll_out = 11289600*2;
        	break;
        case 192000:
        	pll_out = 12288000*2;
        	break;
        case 352800:
        case 384000:
            pll_out = 12288000*2;
            break;
        default:
            DBG("Enter:%s, %d, Error rate=%d\n",__FUNCTION__,__LINE__,params_rate(params));
            return -EINVAL;
            break;
	}
	DBG("Enter:%s, %d, rate=%d\n",__FUNCTION__,__LINE__,params_rate(params));
    snd_soc_dai_set_sysclk(codec_dai, 0, pll_out, SND_SOC_CLOCK_IN);
	
//	#if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 	
//		snd_soc_dai_set_sysclk(cpu_dai, 0, pll_out, 0);
//	#endif	
	#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE)
        ct7302_rate_set(params_rate(params));
		div_bclk = 63;
		div_mclk = pll_out/(params_rate(params)*64) - 1;
		
		DBG("func is%s,pll_out=%d,div_mclk=%d div_bclk\n",
				__FUNCTION__,pll_out,div_mclk, div_bclk);
        snd_soc_dai_set_sysclk(cpu_dai, 0, pll_out, 0);
		snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK,div_bclk);
		snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, div_mclk);
//		DBG("Enter:%s, %d, LRCK=%d\n",__FUNCTION__,__LINE__,(pll_out/4)/params_rate(params));		
	#endif

#if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER)
    /* ct7302_rate_set(params_rate(params)); */
    /* audioBridge_rate_set(params_rate(params)); */
#endif
    return 0;
}


static int rk29_hw_params__(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int pll_out = 0; 
	int ret;

	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);    
	/*by Vincent Hsiung for EQ Vol Change*/
#define HW_PARAMS_FLAG_EQVOL_ON 0x21
#define HW_PARAMS_FLAG_EQVOL_OFF 0x22
	if ((params->flags == HW_PARAMS_FLAG_EQVOL_ON)||(params->flags == HW_PARAMS_FLAG_EQVOL_OFF))
	{
		ret = codec_dai->driver->ops->hw_params(substream, params, codec_dai); //by Vincent
		DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	}
	else
	{
		DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
		/* set codec DAI configuration */
#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE) 
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS); 
		//printk("CONFIG_SND_RK29_CODEC_SOC_SLAVE=====work in slave!\n");
#endif

#if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM ); 
#endif
		if (ret < 0)
			return ret; 

		/* set cpu DAI configuration */
#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE) 
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
		//printk("CONFIG_SND_RK29_CODEC_SOC_SLAVE=====work in slave!\n");
#endif	
#if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 
        if(isI2sFormat_LJF((params_format(params)))){

            if(params_format(params) == SNDRV_PCM_FORMAT_SOP)
		        ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_LSB_LEFT_J ); // spdif LSB first
            else
		        ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_MSB_LEFT_J ); //dsd MSB first

        } else{
            ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                                     SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);	
        }
#endif		
    }
		if (ret < 0)
			return ret;

	switch(params_rate(params)) {
		case 8000:
		case 16000:
		case 24000:
		case 32000:
		case 64000:
		case 48000:
			pll_out = 12288000;
			break;
		case 11025:
		case 22050:
		case 44100:
			pll_out = 11289600;
			DBG("pll_out=%d----%d\n",pll_out,__LINE__);
			break;
		case 88200:
		case 176400:
		case 352800:
			pll_out = 11289600*2;
			break;
		case 96000:
		case 192000:
		case 384000:
			pll_out = 12288000*2;
			break;
		default:
			DBG("Enter:%s, %d, Error rate=%d\n",__FUNCTION__,__LINE__,params_rate(params));
			return -EINVAL;
			break;
	}
	DBG("Enter:%s, %d, rate=%d pll %d\n",__FUNCTION__,__LINE__,params_rate(params),pll_out );

#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE__)
snd_soc_dai_set_sysclk(cpu_dai, 0, pll_out, 0);
snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK, (pll_out/4)/params_rate(params)-1);
snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, 3);
#endif
#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE)
    /* snd_soc_dai_set_sysclk(cpu_dai, 0,12000000, 0); */
    /* msleep(5); */
	snd_soc_dai_set_sysclk(cpu_dai, 0, pll_out, 0);
	//snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK, (pll_out/4)/params_rate(params)-1);
	//snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, 3);
	        snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK, (2*32-1));

        switch(params_rate(params)){
		case 384000:
		case 352800:
                DBG("MCLK== 1 BCLK \n");
                snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK,0);
                break;
        case 192000:
        case 176400:
                DBG("MCLK== 2 BCLK \n");
                snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK,1);
               snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK, (pll_out/2)/params_rate(params)-1);

                break;
        case 96000:
        case 88200:
                DBG("MCLK== 4 BCLK \n");
                snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK,3);
                snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK, (pll_out/2)/params_rate(params)-1);
                break;
        default :
                DBG("MCLK ==4BCLK \n");
                snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, 3);
                snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK, (pll_out/4)/params_rate(params)-1);
                break;
        }
        printk("Audio Sampling rate %d \n",params_rate(params) );
        /* snd_soc_dai_set_sysclk(codec_dai,0,pll_out,SND_SOC_CLOCK_IN); */

#endif

	/* DBG("Enter:%s, %d, LRCK=%d\n",__FUNCTION__,__LINE__,(pll_out/4)/params_rate(params)); */
	return 0;
}

static const struct snd_soc_dapm_widget rk29_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Audio Out", NULL),
	SND_SOC_DAPM_LINE("Line in", NULL),
	SND_SOC_DAPM_MIC("Micn", NULL),
	SND_SOC_DAPM_MIC("Micp", NULL),
};

static const struct snd_soc_dapm_route audio_map[]= {

	{"Audio Out", NULL, "LOUT1"},
	{"Audio Out", NULL, "ROUT1"},
	{"Line in", NULL, "RINPUT1"},
	{"Line in", NULL, "LINPUT1"},
	{"Micn", NULL, "RINPUT2"},
	{"Micp", NULL, "LINPUT2"}, 
};

/*
 * Logic for a ak4490 as connected on a rockchip board.
 */
static int rk29_ak4490_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	printk("Enter rk29_ak4490_init!\n");  
	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

	/* ret = snd_soc_dai_set_sysclk(codec_dai, 0, */
			/* 11289600, SND_SOC_CLOCK_IN); */
	if (ret < 0) {
		printk(KERN_ERR "Failed to set ak4490 SYSCLK: %d\n", ret);
		return ret;
	}
	printk("snd_soc_dai_set_sysclk is ok=======ret=%d\n", ret);
	/* Add specific widgets */
	//snd_soc_dapm_new_controls(dapm, rk29_dapm_widgets,
	//			  ARRAY_SIZE(rk29_dapm_widgets));
	//DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

	/* Set up specific audio path audio_mapnects */
	//snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));
	//DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	//snd_soc_dapm_nc_pin(codec, "LOUT1");
	//snd_soc_dapm_nc_pin(codec, "ROUT1");
	//snd_soc_dapm_sync(dapm);
	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

	return 0;
}

static struct snd_soc_ops rk29_ops = {
	.hw_params = rk29_hw_params,
};

static struct snd_soc_dai_link rk29_dai = {
	.name = "ak4490",
	.stream_name = "ak4490 PCM",
	.codec_name = "ak4490.3-0048",
	.platform_name = "rockchip-audio",
#if defined(CONFIG_SND_RK29_SOC_I2S_8CH)	
	.cpu_dai_name = "rk29_i2s.0",
#elif defined(CONFIG_SND_RK29_SOC_I2S_2CH)
	.cpu_dai_name = "rk29_i2s.1",
#else
	.cpu_dai_name = "rk29_i2s.2",
#endif
	.codec_dai_name = "ak4490 HiFi",
	.init = rk29_ak4490_init,
	.ops = &rk29_ops,
};

static struct snd_soc_card snd_soc_card_rk29 = {
	.name = "RK29_AK4490",
	.dai_link = &rk29_dai,
	.num_links = 1,
};

static struct platform_device *rk29_snd_device;

static int __init audio_card_init(void)
{
	int ret =0;	
	DBG("ak4490 audio_card_init\n");

	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

	rk29_snd_device = platform_device_alloc("soc-audio", -1);
	if (!rk29_snd_device) {
		printk("platform device allocation failed\n");
		ret = -ENOMEM;
		return ret;
	}

	platform_set_drvdata(rk29_snd_device, &snd_soc_card_rk29);

	ret = platform_device_add(rk29_snd_device);
	if (ret) {
		printk("platform device add failed\n");
		platform_device_put(rk29_snd_device);
		return ret;
	}

	return ret;
}
static void __exit audio_card_exit(void)
{
	platform_device_unregister(rk29_snd_device);
	//rk29_speaker_deinit(rk29_speaker);	
}

module_init(audio_card_init);
module_exit(audio_card_exit);
/* Module information */
MODULE_AUTHOR("rockchip");
MODULE_DESCRIPTION("ROCKCHIP i2s ASoC Interface");
MODULE_LICENSE("GPL");
