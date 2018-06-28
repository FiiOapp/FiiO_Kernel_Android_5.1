/*
 * ak4490.h  --  audio driver for ak4490
 *
 * Copyright (C) 2014 Asahi Kasei Microdevices Corporation
 *  Author                Date        Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      14/08/11	   1.0
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */


// #define SND_SOC_DAIFMT_DSD		   0x101

#ifndef _AK4490_H
#define _AK4490_H

#define AK4490_00_CONTROL1			0x00
#define AK4490_01_CONTROL2			0x01
#define AK4490_02_CONTROL3			0x02
#define AK4490_03_LCHATT			0x03
#define AK4490_04_RCHATT			0x04
#define AK4490_05_CONTROL4			0x05
#define AK4490_06_CONTROL5			0x06
#define AK4490_07_CONTROL6			0x07
#define AK4490_08_CONTROL7			0x08
#define AK4490_09_CONTROL8			0x09

#define AK4490_MAX_REGISTERS	(AK4490_09_CONTROL8 + 1)

/* Bitfield Definitions */

/* AK4490_00_CONTROL1 (0x00) Fields */
#define AK4490_DIF					0x0E
#define AK4490_DIF_MSB_MODE	    (2 << 1)
#define AK4490_DIF_I2S_MODE     (3 << 1)
#define AK4490_DIF_32BIT_MODE	(4 << 1)

/* AK4490_02_CONTROL3 (0x02) Fields */
#define AK4490_DIF_DSD				0x80
#define AK4490_DIF_DSD_MODE	    (1 << 7)


/* AK4490_01_CONTROL2 (0x01) Fields */
/* AK4490_05_CONTROL4 (0x05) Fields */
#define AK4490_DFS				0x18
#define AK4490_DFS_48KHZ		(0x0 << 3)  //  30kHz to 54kHz
#define AK4490_DFS_96KHZ		(0x1 << 3)  //  54kHz to 108kHz
#define AK4490_DFS_192KHZ		(0x2 << 3)  //  120kHz  to 216kHz
#define AK4490_DFS_384KHZ		(0x0 << 3)
#define AK4490_DFS_768KHZ		(0x1 << 3)

#define AK4490_DFS2				0x2
#define AK4490_DFS2_48KHZ		(0x0 << 1)  //  30kHz to 216kHz
#define AK4490_DFS2_384KHZ		(0x1 << 1)  //  384kHz, 768kHkHz to 108kHz

#define AK4490_MONO_MODE        (0x01<<3)




#define SOC_DOUBLE_TLV_AK4490(xname, xreg, shift_left, shift_right, xmax,\
			       xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw_ak4490, \
	.put = snd_soc_put_volsw_ak4490, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .shift = shift_left, .rshift = shift_right,\
		 .max = xmax, .invert = xinvert} }

#define SOC_SINGLE_TLV_AK4490(xname, xreg, xshift, xmax, xinvert, tlv_array) \
	SOC_DOUBLE_TLV_CT7302(xname, xreg, xshift, xshift, xmax, \
			       xinvert, tlv_array)

#define SOC_ENUM_AK4490(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_enum_double_ak4490, .put = snd_soc_put_enum_double_ak4490, \
	.private_value = (unsigned long)&xenum }

#define SOC_ENUM_EXT_AK4490(xname, xenum, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_ext, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&xenum }



#define SOC_DOUBLE_R_EXT_TLV_AK4490(xname, reg_left, reg_right, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		 SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw_2r, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, .rreg = reg_right, .shift = xshift, \
		.max = xmax, .platform_max = xmax, .invert = xinvert} }

#define SOC_SINGLE_EXT_AK4490(xname, xreg, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmax, xinvert) }


enum lpf_type {
    LPF_SHARP_ROLL_OFF=0,
    LPF_SLOW_ROLL_OFF,
    LPF_DELAY_SHARP_ROLL_OFF,
    LPF_DELAY_SLOW_ROLL_OFF,
    LPF_SUPPER_SLOW_ROLL_OFF,
};

extern enum lpf_type gak4490_defalult_lpf;
extern void ak4490_set_lpf(enum lpf_type lpf);
#endif
