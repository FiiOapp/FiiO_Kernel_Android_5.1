/*
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@openedhand.com>
 *
 * Based on ES9028.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _ES9028_H
#define _ES9028_H

#define CONFIG_HHTECH_MINIPMP	1

/* ES9028 register space */

#define ES9028_DAC0 0x10
#define ES9028_DAC1 0x11
#define ES9028_DAC2 0x12
#define ES9028_DAC3 0x13
#define ES9028_DAC4 0x14
#define ES9028_DAC5 0x15
#define ES9028_DAC6 0x16
#define ES9028_DAC7 0x17
#define ES9028_FILTER 0x07
#define ES9028_THDC2_L 0x1C
#define ES9028_THDC2_H 0x1D
#define ES9028_THDC3_L 0x1E
#define ES9028_THDC3_H 0x1F




#define ES9028_CONTROL1         0x00
#define ES9028_CONTROL2         0x01
#define ES9028_CHIPPOWER        0x02
#define ES9028_ADCPOWER         0x03
#define ES9028_DACPOWER         0x04
#define ES9028_CHIPLOPOW1       0x05
#define ES9028_CHIPLOPOW2       0x06
#define ES9028_ANAVOLMANAG      0x07
#define ES9028_MASTERMODE       0x08
#define ES9028_ADCCONTROL1      0x09
#define ES9028_ADCCONTROL2      0x0a
#define ES9028_ADCCONTROL3      0x0b
#define ES9028_ADCCONTROL4      0x0c
#define ES9028_ADCCONTROL5      0x0d
#define ES9028_ADCCONTROL6      0x0e
#define ES9028_ADCCONTROL7      0x0f
#define ES9028_ADCCONTROL8      0x10
#define ES9028_ADCCONTROL9      0x11
#define ES9028_ADCCONTROL10     0x12
#define ES9028_ADCCONTROL11     0x13
#define ES9028_ADCCONTROL12     0x14
#define ES9028_ADCCONTROL13     0x15
#define ES9028_ADCCONTROL14     0x16

#define ES9028_DACCONTROL1      0x17
#define ES9028_DACCONTROL2      0x18
#define ES9028_DACCONTROL3      0x19
#define ES9028_DACCONTROL4      0x1a
#define ES9028_DACCONTROL5      0x1b
#define ES9028_DACCONTROL6      0x1c
#define ES9028_DACCONTROL7      0x1d
#define ES9028_DACCONTROL8      0x1e
#define ES9028_DACCONTROL9      0x1f
#define ES9028_DACCONTROL10     0x20
#define ES9028_DACCONTROL11     0x21
#define ES9028_DACCONTROL12     0x22
#define ES9028_DACCONTROL13     0x23
#define ES9028_DACCONTROL14     0x24
#define ES9028_DACCONTROL15     0x25
#define ES9028_DACCONTROL16     0x26
#define ES9028_DACCONTROL17     0x27
#define ES9028_DACCONTROL18     0x28
#define ES9028_DACCONTROL19     0x29
#define ES9028_DACCONTROL20     0x2a
#define ES9028_DACCONTROL21     0x2b
#define ES9028_DACCONTROL22     0x2c
#define ES9028_DACCONTROL23     0x2d
#define ES9028_DACCONTROL24     0x2e
#define ES9028_DACCONTROL25     0x2f
#define ES9028_DACCONTROL26     0x30
#define ES9028_DACCONTROL27     0x31
#define ES9028_DACCONTROL28     0x32
#define ES9028_DACCONTROL29     0x33
#define ES9028_DACCONTROL30     0x34

#define ES9028_LADC_VOL         ES9028_ADCCONTROL8
#define ES9028_RADC_VOL         ES9028_ADCCONTROL9

#define ES9028_LDAC_VOL         ES9028_DACCONTROL4
#define ES9028_RDAC_VOL         ES9028_DACCONTROL5

#define ES9028_LOUT1_VOL        ES9028_DACCONTROL24
#define ES9028_ROUT1_VOL        ES9028_DACCONTROL25
#define ES9028_LOUT2_VOL        ES9028_DACCONTROL26
#define ES9028_ROUT2_VOL        ES9028_DACCONTROL27

#define ES9028_ADC_MUTE         ES9028_ADCCONTROL7
#define ES9028_DAC_MUTE         ES9028_DACCONTROL3



#define ES9028_IFACE            ES9028_MASTERMODE

#define ES9028_ADC_IFACE        ES9028_ADCCONTROL4
#define ES9028_ADC_SRATE        ES9028_ADCCONTROL5

#define ES9028_DAC_IFACE        ES9028_DACCONTROL1
#define ES9028_DAC_SRATE        ES9028_DACCONTROL2



#define ES9028_SYSCLK	        0

struct es9028_setup_data {
	int i2c_bus;	
	unsigned short i2c_address;
};

#if 1 //lzcx
#define ES9028_PLL1			0
#define ES9028_PLL2			1

/* clock inputs */
#define ES9028_MCLK		0
#define ES9028_PCMCLK		1

/* clock divider id's */
#define ES9028_PCMDIV		0
#define ES9028_BCLKDIV		1
#define ES9028_VXCLKDIV		2

/* PCM clock dividers */
#define ES9028_PCM_DIV_1	(0 << 6)
#define ES9028_PCM_DIV_3	(2 << 6)
#define ES9028_PCM_DIV_5_5	(3 << 6)
#define ES9028_PCM_DIV_2	(4 << 6)
#define ES9028_PCM_DIV_4	(5 << 6)
#define ES9028_PCM_DIV_6	(6 << 6)
#define ES9028_PCM_DIV_8	(7 << 6)

/* BCLK clock dividers */
#define ES9028_BCLK_DIV_1	(0 << 7)
#define ES9028_BCLK_DIV_2	(1 << 7)
#define ES9028_BCLK_DIV_4	(2 << 7)
#define ES9028_BCLK_DIV_8	(3 << 7)

/* VXCLK clock dividers */
#define ES9028_VXCLK_DIV_1	(0 << 6)
#define ES9028_VXCLK_DIV_2	(1 << 6)
#define ES9028_VXCLK_DIV_4	(2 << 6)
#define ES9028_VXCLK_DIV_8	(3 << 6)
#define ES9028_VXCLK_DIV_16	(4 << 6)

#define ES9028_DAI_HIFI		0
#define ES9028_DAI_VOICE		1

#define ES9028_1536FS 1536
#define ES9028_1024FS	1024
#define ES9028_768FS	768
#define ES9028_512FS	512
#define ES9028_384FS	384
#define ES9028_256FS	256
#define ES9028_128FS	128
#endif


extern int es9028_read_i2c_blk(u8 reg, u8* data,u8 len);
extern int es9028_write_i2c(u8 reg,u8 data);
#endif
