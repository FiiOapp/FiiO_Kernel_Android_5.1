/*drivers/fiio/fpga/fpga_audio_interface.c 
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/miscdevice.h>
#include <asm/dma.h>
#include <linux/preempt.h>
/* #include "rk29_spim.h" */
#include <linux/spi/spi.h>
#include <mach/board.h>
#include "fpga_audio_interface.h"
#include <mach/iomux.h>
#include <linux/regulator/consumer.h>

#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#define MAX_SPI_BUS_NUM 2

void gpio_sequence(u8 gpioconfig,u32 delay);
/* int audio_fpga_bridge_sendCommand( u8 cmd); */
int audio_fpga_bridge_sendCommand( u16 cmd);
#include "fpga_interface_sys.c"

#define FPGA_RESET      RK30_PIN0_PB0
#define FPGA_SPI_CS     RK30_PIN3_PB5
#define FPGA_SPI_CLK    RK30_PIN3_PB4
#define FPGA_SPI_MOSI   RK30_PIN1_PA4

#define    BRIDGE_REGULATOR_VOL 3000000
#define    BRIDGE_REGULATOR_NAME "axp22_dldo2"

#if defined(CONFIG_LIDA_MACH_X5)
    #undef  FPGA_RESET
    #define FPGA_RESET RK30_PIN3_PD7

    #undef  BRIDGE_REGULATOR_NAME
    #define BRIDGE_REGULATOR_NAME "act_ldo2"

#elif defined(CONFIG_LIDA_MACH_X7II)
    #undef  FPGA_RESET
    #define FPGA_RESET RK30_PIN0_PD7

    #undef  BRIDGE_REGULATOR_NAME
    #define BRIDGE_REGULATOR_NAME "act_ldo2"

    #define OSC_FS  1024


#endif

#define USE_DEFINED_SPI_INTERFACE

#define USE_GPIO_STATUS

#if defined USE_DEFINED_SPI_INTERFACE
struct spi_io {
    unsigned clk;
    unsigned cs;
    unsigned mosi;
    unsigned miso;
    unsigned bits;
};

static u16 PreCmd=AUDIO_SAMPLERATE_48K_S512FS;


#define CLK_RISING_EDGE    1
#define SPI_SET_CLK(spi)  gpio_set_value(spi->clk,CLK_RISING_EDGE);
#define SPI_UNSET_CLK(spi)  gpio_set_value(spi->clk,!CLK_RISING_EDGE);

#define SPI_CS       0
#define SPI_SET_CS(spi)  gpio_set_value(spi->cs,SPI_CS);
#define SPI_UNSET_CS(spi)  gpio_set_value(spi->cs,!SPI_CS);

#define SPI_SET_MOSI(spi,value)  gpio_set_value(spi->mosi,!!(value));



#define HW_TYPE_UNKNOWN -1
#define HW_TYPE_OSC_2IN 0
#define HW_TYPE_OSC_3IN 1
extern int HW_tpye;



void spi_write_sample(struct spi_io* spi,void* cmd)
{
    /* u8 cmd8,reg8; */
    u16 cmd16=0;
    int i;

    if(spi->bits <= 8){

        cmd16=*(u8*)cmd;
    }
    else if (spi->bits <= 16){
        cmd16=*(u16*)cmd;

    }

    SPI_UNSET_CLK(spi);
    SPI_SET_CS(spi);
    udelay(100);
    for(i=0;i<spi->bits;i++){

        SPI_UNSET_CLK(spi);
        SPI_SET_MOSI(spi,(cmd16<<i & 0x01<<(spi->bits-1)) );
        SPI_SET_CLK(spi);
        udelay(1);
        /* printk("spi set cmd16 0x%x  %x %x \n", */
               /* cmd16,cmd16>>i & 0x01,!!(cmd16>>i & 0x01) ); */
    }
    udelay(10);
    SPI_UNSET_CS(spi);
    SPI_SET_MOSI(spi,0 );
    SPI_SET_CLK(spi);

}

#endif
struct spi_test_data {
	struct device	*dev;
	struct spi_device	*spi;	
	char *rx_buf;
	int rx_len; 
	char *tx_buf;
	int tx_len; 
};



#if 0
static struct spi_test_data *g_spi_test_data[MAX_SPI_BUS_NUM];
static ssize_t spi_test_write(struct file *file, 
			const char __user *buf, size_t count, loff_t *offset)
{
	char nr_buf[8];
	int nr = 0, ret;
	int i = 0;
	struct spi_device *spi = NULL;
	char txbuf[256],rxbuf[256];

	printk("%s:0:bus=0,cs=0; 1:bus=0,cs=1; 2:bus=1,cs=0; 3:bus=1,cs=1\n",__func__);

	if(count > 3)
	    return -EFAULT;
	ret = copy_from_user(nr_buf, buf, count);
	if(ret < 0)
	    return -EFAULT;

	sscanf(nr_buf, "%d", &nr);
	if(nr >= 4 || nr < 0)
	{
		printk("%s:cmd is error\n",__func__);
	    return -EFAULT;
	}
	
	for(i=0; i<256; i++)
	txbuf[i] = i;


#if !defined(CONFIG_SPIM0_RK29)
	if((nr == 0) || (nr == 1))
	{
		printk("%s:error SPIM0 need selected\n",__func__);	
		return -EFAULT;
	}
#endif

#if !defined(CONFIG_SPIM1_RK29)
	if((nr == 2) || (nr == 3))
	{
		printk("%s:error SPIM1 need selected\n",__func__);	
		return -EFAULT;
	}
#endif

	switch(nr)
	{
		case 0:	
			if(!g_spi_test_data[0]->spi)		
			return -EFAULT;
			spi = g_spi_test_data[0]->spi;
			spi->chip_select = 0;
			break;
		case 1:	
			if(!g_spi_test_data[0]->spi)		
			return -EFAULT;
			spi = g_spi_test_data[0]->spi;
			spi->chip_select = 1;
			break;
		case 2:	
			if(!g_spi_test_data[1]->spi)		
			return -EFAULT;
			spi = g_spi_test_data[1]->spi;
			spi->chip_select = 0;
			break;
		case 3:	
			if(!g_spi_test_data[1]->spi)		
			return -EFAULT;
			spi = g_spi_test_data[1]->spi;
			spi->chip_select = 1;
			break;
		
		default:
			break;
	}

	for(i=0; i<10; i++)
	{
		ret = spi_write(spi, txbuf, 256);
		ret = spi_read(spi, rxbuf, 256);
		ret = spi_write_then_read(spi,txbuf,256,rxbuf,256);
		printk("%s:test %d times\n\n",__func__,i+1);
	}
	
	if(!ret)
	printk("%s:bus_num=%d,chip_select=%d,ok\n",__func__,spi->master->bus_num, spi->chip_select);
	else
	printk("%s:bus_num=%d,chip_select=%d,error\n",__func__,spi->master->bus_num, spi->chip_select);
	
	return count;
}
#endif

struct spi_data{
	struct device	*dev;
	struct spi_device	*spi;	
	char *rx_buf;
	int rx_len; 
	char *tx_buf;
	int tx_len; 
    int fpga_reset;
};

static struct spi_data *gp_spi_data;
static struct spi_io*  gp_spi_io;
int fpga_reset_pint_init(int gpio)
{
    int ret;
    ret=gpio_request(gpio,"fpga reset");
	if(ret < 0){
		printk("gpio request fpga reset error!\n");
	}
	else{

        gpio_direction_output(gpio, GPIO_HIGH);
    }

	return ret;
}

int fpga_enable_status(struct spi_data * pspi)
{
   return gpio_get_value(pspi->fpga_reset);
}

void fpga_enable(struct spi_data * pspi,int delayus)
{
    gpio_set_value(pspi->fpga_reset,GPIO_HIGH);
    udelay(delayus);
}

void fpga_disable(struct spi_data * pspi)
{
    gpio_set_value(pspi->fpga_reset,GPIO_LOW);
}

void AudioBridge_disable()
{

        fpga_disable(gp_spi_data);
        printk("FPGA Bridge Disable \n");

}
EXPORT_SYMBOL_GPL(AudioBridge_disable);
void AudioBridge_enable()
{

        fpga_enable(gp_spi_data,0);
        printk("FPGA Bridge Enable\n");

}
EXPORT_SYMBOL_GPL(AudioBridge_enable);

int AudioBridge_enable_status(void)
{

    return fpga_enable_status(gp_spi_data);

}

/* return >=1, power has opened , 0 not opened before
 * -1 means error
 *
 */


int AudioBridge_power(int onoff)
{

    int ret;
	struct regulator *ldo;
	ldo =regulator_get(NULL, BRIDGE_REGULATOR_NAME);

    printk("%s onoff %d \n",__func__,onoff);
    if(ldo == NULL || IS_ERR(ldo)){
        printk("FPGA power ldo get error %s %d\n",__func__,__LINE__);
        return -1;
    }

    ret=regulator_is_enabled(ldo);
    if(ret >0 && onoff ){

        printk("%s onoff %d  has enabled\n",__func__,onoff);
        return ret;
    }

    if(onoff){
        /* if(!regulator_is_enabled(ldo)){ */
        if(1){
            regulator_set_voltage(ldo,BRIDGE_REGULATOR_VOL,BRIDGE_REGULATOR_VOL);
            regulator_enable(ldo);
            regulator_put(ldo);
            printk("%s set power on\n",__func__);
        }
        /* else */
            /* printk("%s onoff %d  has enabled\n",__func__,onoff); */
    }else{

        printk("%s set power off\n",__func__);
        regulator_disable(ldo);
        regulator_put(ldo);
    }

        return ret;
}

void audio_bridge_onoff(int onoff)
{

    int ret;
    if(onoff){
        ret=AudioBridge_power(onoff);
#if defined(CONFIG_LIDA_MACH_X7II)
        //ret == 0 power has opend
        if(( HW_tpye == HW_TYPE_OSC_3IN) && (ret == 0) ){
            msleep(5);
            printk(" FPGA power on first ,send AUDIO_SAMPLERATE_SUSPEND16\n");
            audio_fpga_bridge_sendCommand(AUDIO_SAMPLERATE_SUSPEND16);
        }
#endif
    }else{

        audio_fpga_bridge_sendCommand(FPGA_SUSPEND16);
        AudioBridge_power(onoff);
    }

}
#if 0
static void spi_gpio_setMode(void)
{

	rk30_mux_api_set("SPI_MOSI",GPIO1_A4);
	rk30_mux_api_set("SPI_CS",GPIO3_B5);
	rk30_mux_api_set("SPI_CLK",GPIO3_B4);

    gpio_direction_output(RK30_PIN3_PB5,GPIO_HIGH);
    gpio_direction_output(RK30_PIN3_PB4,GPIO_HIGH);
    gpio_direction_output(RK30_PIN1_PA4,GPIO_HIGH);
}
#endif

void gpio_test(void)
{

    gpio_set_value(RK30_PIN3_PB5,GPIO_LOW);
    gpio_set_value(RK30_PIN3_PB4,GPIO_LOW);
    gpio_set_value(RK30_PIN1_PA4,GPIO_LOW);

    gpio_set_value(RK30_PIN3_PB5,GPIO_HIGH);
    gpio_set_value(RK30_PIN3_PB4,GPIO_HIGH);
    gpio_set_value(RK30_PIN1_PA4,GPIO_HIGH);

    gpio_set_value(RK30_PIN3_PB5,GPIO_LOW);
    gpio_set_value(RK30_PIN3_PB4,GPIO_LOW);
    gpio_set_value(RK30_PIN1_PA4,GPIO_LOW);

    gpio_set_value(RK30_PIN3_PB5,GPIO_HIGH);
    gpio_set_value(RK30_PIN3_PB4,GPIO_HIGH);
    gpio_set_value(RK30_PIN1_PA4,GPIO_HIGH);
}
void sequence_delay(u8 delay)
{
    if(delay&0x80){
        if((delay&0x7F) <10)
            mdelay(delay&0x7F);
        else
            msleep(delay&0x7F);
    }else{
        if(delay)
            udelay(delay);
    }
}

void gpio_sequence(u8 gpioconfig,u32 delay)
{
    /* printk("gpioconfig %x delay %x \n",gpioconfig , delay); */
    //reset first
    if(gpioconfig&0x80){
        gpio_set_value(FPGA_RESET,!!(gpioconfig&0x08));
        sequence_delay((delay&0xFF000000) >> 24);
    }

    if(gpioconfig&0x10){
        gpio_set_value(FPGA_SPI_CLK,!!(gpioconfig&0x01));
        sequence_delay(delay&0x00FF);
               /* printk("gpio spi clk set %d \n",!!(gpioconfig&0x01)); */
    }

    if(gpioconfig&0x20){
        gpio_set_value(FPGA_SPI_CS,!!(gpioconfig&0x02));
        sequence_delay((delay&0x00FF00) >> 8);
    }

    if(gpioconfig&0x40){
        gpio_set_value(FPGA_SPI_MOSI,!!(gpioconfig&0x04));
        sequence_delay((delay&0xFF0000) >> 16);
    }

    

}

bool audio_fpga_current_mode(fpga_fmt_t mod)
{
    if((PreCmd>>7) == mod)
        return true;
    else
        return false;
}
int audio_fpga_bridge_sendCommandPre()
{
    return audio_fpga_bridge_sendCommand(PreCmd);
}

int audio_fpga_bridge_sendCommand( u16 cmd)
{

    u16 data[1]={cmd};
    PreCmd=cmd;
    if (gp_spi_io == NULL){
        printk("%s gp_spi_io is null\n",__func__);
        return -1;
    }
    switch(cmd){
    case FPGA_SPI_INITSQ:

            gpio_set_value(gp_spi_io->mosi,0);
            gpio_set_value(gp_spi_io->mosi,1);
            gpio_set_value(gp_spi_io->mosi,0);

            gpio_set_value(gp_spi_io->clk,1);
            gpio_set_value(gp_spi_io->clk,0);

            gpio_set_value(gp_spi_io->cs,1);

            mdelay(5);
            fpga_enable(gp_spi_data,0);
            return 0;
    case FPGA_PRE_CMD:             
            data[0]=PreCmd;

    }
#if defined USE_DEFINED_SPI_INTERFACE
if(gp_spi_io){
        /* fpga_disable(gp_spi_data); */

        fpga_disable(gp_spi_data);

        /* udelay(200); */
        /* fpga_enable(gp_spi_data,10); */
        udelay(20);
        spi_write_sample(gp_spi_io,(void*) data);
        udelay(50);
        fpga_enable(gp_spi_data,0);
        printk("AAUDIO_LINK: BRIDGE CMD %x \n",data[0]);
    }
    else
        printk("AUDIO_LINK: BRIDGE gp_spi_data is null\n");


#else
    if(gp_spi_data){
        fpga_disable(gp_spi_data);
        ret = spi_write(gp_spi_data->spi, data, 1);
        fpga_enable(gp_spi_data,10);
        printk("AUDIO_LINK: BRIDGE CMD %x ret:%d  mosi %x\n",data[0],ret,RK30_PIN1_PA4);
    }
    else
        printk("AUDIO_LINK: BRIDGE gp_spi_data is null\n");

#endif
    return 0;
}
EXPORT_SYMBOL_GPL(audio_fpga_bridge_sendCommand);


#if defined(CONFIG_LIDA_MACH_X5)
void audioBridge_rate_set(int rate,fpga_fmt_t fmt)
{
    u16 cmd=AUDIO_SAMPLERATE_44_1K_S512FS;

    switch(rate) {
		case 192000:
            cmd= AUDIO_SAMPLERATE_192K_S512FS;break;
		case 96000:
            cmd= AUDIO_SAMPLERATE_96K_S512FS;break;
		case 48000:
            cmd= AUDIO_SAMPLERATE_48K_S512FS;break;
		case 64000:
            cmd= AUDIO_SAMPLERATE_64K_S512FS;break;
		case 32000:
            cmd= AUDIO_SAMPLERATE_32K_S512FS;break;

		case 176400:
            cmd= AUDIO_SAMPLERATE_176_4K_S512FS;break;
		case 88200:
            cmd= AUDIO_SAMPLERATE_88_2K_S512FS;break;
		case 44100:
            cmd= AUDIO_SAMPLERATE_44_1K_S512FS;break;

        // for spdif
		case 352800:
            cmd= AUDIO_SAMPLERATE_352_8K_S512FS;break;
		case 384000:
            cmd= AUDIO_SAMPLERATE_384K_S512FS;break;

		case 256000:
		case 128000:
		default:
            printk(KERN_ERR "Audio Bridge can't set Sampling rate %d \n",rate);
			break;
	}

    audio_fpga_bridge_sendCommand(FPGA_SUSPEND16);

    // fiio fpga support new protocol  for spdif & dsd
    if( (fmt == FMT_SOP) && (rate >=352800)){
        fmt  = FMT_FIIO;
        cmd=(cmd &0x7f) | (((u16)fmt) <<7);
    }else{
        cmd=(cmd &0x7f) | (((u16)fmt) <<7);
    }

    audio_fpga_bridge_sendCommand(cmd);
}
EXPORT_SYMBOL_GPL(audioBridge_rate_set);
#elif defined(CONFIG_LIDA_MACH_X7II)

void audioBridge_rate_set_osc2(int rate,fpga_fmt_t fmt)
{
    u16 cmd=AUDIO_SAMPLERATE_44_1K_S1024FS;

    switch(rate) {
		case 384000:
            cmd= AUDIO_SAMPLERATE_384K_S1024FS;break;
		case 192000:
            cmd= AUDIO_SAMPLERATE_192K_S1024FS;break;
		case 96000:
            cmd= AUDIO_SAMPLERATE_96K_S1024FS;break;
		case 48000:
            cmd= AUDIO_SAMPLERATE_48K_S1024FS;break;
		case 64000:
            cmd= AUDIO_SAMPLERATE_64K_S1024FS;break;
		case 32000:
            cmd= AUDIO_SAMPLERATE_32K_S1024FS;break;

		case 352800:
            cmd= AUDIO_SAMPLERATE_352_8K_S1024FS;break;
		case 176400:
            cmd= AUDIO_SAMPLERATE_176_4K_S1024FS;break;
		case 88200:
            cmd= AUDIO_SAMPLERATE_88_2K_S1024FS;break;
		case 44100:
            cmd= AUDIO_SAMPLERATE_44_1K_S1024FS;break;

		/* case 384000: */
		/* case 352800: */
		case 256000:
		case 128000:
		default:
            printk(KERN_ERR "Audio Bridge can't set Sampling rate %d \n",rate);
			break;
	}
    cmd=(cmd &0x7f) | (((u16)fmt) <<7);
    audio_fpga_bridge_sendCommand(cmd);
}


#define FS384K_MCLK_100M
void audioBridge_rate_set_osc3(int rate,fpga_fmt_t fmt)
{
    u16 cmd=AUDIO_SAMPLERATE_44_1K;
    u8 dis_2div=0;

    switch(rate) {
		case 384000:
            cmd= AUDIO_SAMPLERATE_384K;dis_2div =1;
            #if defined(FS384K_MCLK_100M )
            if(fmt  ==  FMT_PCM )
                cmd = cmd | 0x80;
            #endif
            break;
		case 192000:
            cmd= AUDIO_SAMPLERATE_192K;break;
		case 96000:
            cmd= AUDIO_SAMPLERATE_96K;break;
		case 48000:
            cmd= AUDIO_SAMPLERATE_48K;break;

		case 128000:
            cmd= AUDIO_SAMPLERATE_128K;break;
		case 64000:
            cmd= AUDIO_SAMPLERATE_64K;break;
		case 32000:
            cmd= AUDIO_SAMPLERATE_32K;break;

		case 352800:
            cmd= AUDIO_SAMPLERATE_352_8K;dis_2div =1;
            #if defined(FS384K_MCLK_100M )
            if(fmt  ==  FMT_PCM )
                cmd = cmd | 0x80;
            #endif

            break;
		case 176400:
            cmd= AUDIO_SAMPLERATE_176_4K;break;
		case 88200:
            cmd= AUDIO_SAMPLERATE_88_2K;break;
		case 44100:
            cmd= AUDIO_SAMPLERATE_44_1K;break;

		default:
            printk(KERN_ERR "Audio Bridge can't set Sampling rate %d \n",rate);
			break;
	}

    /* clkdiv={spicmd[4],spicmd[3],spicmd[2],spicmd[1],spicmd[0]};
     * clksel={spicmd[7],spicmd[6],spicmd[5]};
     * wmode = {spicmd[9],spicmd[8]};
     */

    /* cmd=(cmd &0x7f) | (((u16)fmt) <<7); */
    cmd=cmd & 0xEF;// clean pre 2 div bits

    if(OSC_FS == 1024 ) // set pre 2 div bits
        cmd=cmd | 0x10;

    if( dis_2div )    // clean pre 2 div bit
        cmd = cmd & 0xEF;


    cmd=((cmd &0xFF) | (((u16)fmt) <<8))<<4;
    audio_fpga_bridge_sendCommand(cmd);
}

void audioBridge_rate_set(int rate,fpga_fmt_t fmt)
{
    if( HW_tpye == HW_TYPE_OSC_3IN)
        audioBridge_rate_set_osc3(rate,fmt);
    else
        audioBridge_rate_set_osc2(rate,fmt);

}
EXPORT_SYMBOL_GPL(audioBridge_rate_set);



#else

#endif

#define SOC_ENUM_FPGA(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_enum_double_fpga, .put = snd_soc_put_enum_double_fpga, \
	.private_value = (unsigned long)&xenum }

int snd_soc_get_enum_double_fpga(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	/* struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol); */
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int  bitmask;

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;

    switch(e->reg){
    case 0xFF:
	    ucontrol->value.enumerated.item[0] = AudioBridge_enable_status()? 0x01:0x00;
        break;
    default:
        ucontrol->value.enumerated.item[0] =0;
        printk(KERN_ERR "FPGA contol error reg 0x%x not available\n",e->reg);

    }
    
	/* ucontrol->value.enumerated.item[0] */
		/* = (val >> e->shift_l) & (bitmask - 1); */
	/* if (e->shift_l != e->shift_r) */
		/* ucontrol->value.enumerated.item[1] = */
			/* (val >> e->shift_r) & (bitmask - 1); */

	return 0;
}

int snd_soc_put_enum_double_fpga(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	/* struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol); */
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

    switch(e->reg){
    case 0xFF:
        if(val){
            printk(" control enable\n");
            AudioBridge_enable();
        }
        else{
            printk(" control disable \n");
            AudioBridge_disable();
        }
        break;

    default:
        printk(KERN_ERR "FPGA contol error reg 0x%x not available\n",e->reg);

    }
 
    return 0;
}


static const char *fpga_out_enable[] = {"OFF","ON"};
static const struct soc_enum fpga_enum[] = {
	SOC_ENUM_SINGLE(0xFF, 0, ARRAY_SIZE(fpga_out_enable), fpga_out_enable), // virtual reg
};

static const struct snd_kcontrol_new fpga_snd_controls[] = {


	SOC_ENUM_FPGA("BRIDGE AudioOutput ENABLE",fpga_enum[0]),

};

const struct snd_kcontrol_new* get_audiobirdge_control_table(int* num_controls)
{
    *num_controls=ARRAY_SIZE(fpga_snd_controls);
    return fpga_snd_controls;
    /* return NULL; */

}
EXPORT_SYMBOL_GPL(get_audiobirdge_control_table);


static int __devinit fpga_spi_probe(struct spi_device *spi)
{	
	struct spi_data *pspi_data;
	struct spi_io *pspi_io;
    struct fpga_interface_platform_data* pdata;
	int ret;

    printk("%s %d\n",__func__,__LINE__);
	if(!spi)	
	return -ENOMEM;

	if((spi->master->bus_num >= MAX_SPI_BUS_NUM) || (spi->master->bus_num < 0))
	{
		printk("%s:error:bus_num=%d\n",__func__, spi->master->bus_num);	
		return -ENOMEM;
	}
	
	pspi_io= (struct spi_io*)kzalloc(sizeof(struct spi_io), GFP_KERNEL);
	if(!pspi_io){
		dev_err(&spi->dev, "ERR: no memory for pspi_io\n");
		return -ENOMEM;
	}

    pspi_io->clk=FPGA_SPI_CLK; 
    pspi_io->cs=FPGA_SPI_CS; 
    pspi_io->mosi=FPGA_SPI_MOSI; 

#if defined (CONFIG_LIDA_MACH_X7II)
    pspi_io->bits=16;
#else
    pspi_io->bits=9; 
#endif
    gp_spi_io = pspi_io;

	pspi_data = (struct spi_data *)kzalloc(sizeof(struct spi_data), GFP_KERNEL);
	if(!pspi_data){
		dev_err(&spi->dev, "ERR: no memory for pspi_data\n");
		return -ENOMEM;
	}

	spi->bits_per_word = 9;
	
	pspi_data->spi = spi;
	pspi_data->dev = &spi->dev;
    pdata=(struct fpga_interface_platform_data*)spi->dev.platform_data;
    pspi_data->fpga_reset=INVALID_GPIO;
    pspi_data->fpga_reset=pdata->reset_pin;
    fpga_reset_pint_init(pspi_data->fpga_reset);

#ifndef USE_DEFINED_SPI_INTERFACE
	ret = spi_setup(spi);
	if (ret < 0){
		dev_err(pspi_data->dev, "ERR: fail to setup spi\n");
		return -1;
	}	
#endif

	gp_spi_data= pspi_data;
    audio_fpga_bridge_sendCommandPre();

/* #if defined (CONFIG_LIDA_MACH_X7II) */
    /* AudioBridge_power(0); */
/* #endif */
    printk("%s %d ret %d\n",__func__,__LINE__,ret);
	return ret;

}

/* static const struct spi_device_id spi_test_id[] = {		 */
	/* {"spi_test_bus0", 0}, */
	/* {"spi_test_bus1", 1}, */
	/* {}, */
/* }; */


static struct spi_driver fpga_spi_driver= {
	.driver = {
		.name		= "FPGA_AUDIO_BRIDGA",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},
	/* .id_table = spi_test_id, */

	.probe		= fpga_spi_probe,
};

static int __init fpga_sys_init(void)
{	
	printk("%s\n",__func__);
    sys_node_init();
	/* spi_register_board_info(board_spi_test_devices, ARRAY_SIZE(board_spi_test_devices)); */
	/* misc_register(&spi_test_misc); */
	return spi_register_driver(&fpga_spi_driver);
}

static void __exit fpga_sys_exit(void)
{
        /* misc_deregister(&spi_test_misc); */
	return spi_unregister_driver(&fpga_spi_driver);
}
module_init(fpga_sys_init);
module_exit(fpga_sys_exit);

