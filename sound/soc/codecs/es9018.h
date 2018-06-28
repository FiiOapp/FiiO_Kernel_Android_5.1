/*
 * wm2000.h  --  WM2000 Soc Audio driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ES9018_H
#define _ES9018_H


#define DRV_NAME "ES9018"

/*ES9018_MODE_CONTROL_2   
 * [1:0] : DE-EMPHASIS DELECT
 * 2â€™b00 = 32kHz
 * 2â€™b01 = 44.1kHz
 * 2â€™b10 = 48kHz
 * 2â€™b11 = RESERVED
 */
#define ES9018_MODE_CONTROL2_DEEMPHASIS_MASK	(1<<1 | 1<<0)
#define ES9018_MODE_CONTROL2_DEEMPHASIS_32K		(0<<1 | 0<<0)
#define ES9018_MODE_CONTROL2_DEEMPHASIS_44K		(0<<1 | 1<<0)
#define ES9018_MODE_CONTROL2_DEEMPHASIS_48K		(1<<1 | 0<<0)

/*
 * Register values.
 */
#define ES9018_DAC0				0x00	// -REG_VALUE/2 -->DAC1
#define ES9018_DAC1             0x01
#define ES9018_DAC2             0x02
#define ES9018_DAC3             0x03
#define ES9018_DAC4				0x04
#define ES9018_DAC5             0x05
#define ES9018_DAC6             0x06
#define ES9018_DAC7             0x07
#define ES9018_AUTOMUTE_LEV     0x08	// 1'b0,7'd104
#define ES9018_AUTOMUTE_TIME	0x09	// 8'd4
#define ES9018_MODE_CONTROL_1   0x0A	// 8'b11001110
#define ES9018_MODE_CONTROL_2   0x0B	// 8'b10000101
#define ES9018_MODE_CONTROL_3   0x0C	// 8'b00100000
#define ES9018_DAC_POL   		0x0D	// 8'b00000000
#define ES9018_DAC_SRC_3478		0x0E	// 8'b00001011
#define ES9018_MODE_CONTROL_4   0x0F	// 8'b00000000
#define ES9018_AUTOMUTE_LPBACK  0x10	// 8'b00000000
#define ES9018_MODE_CONTROL_5   0x11	// 8'b00011100
#define ES9018_SPDIF_SRC		0x12	// 8'd1
#define ES9018_DACB_POL			0x13	// 8'b00000000
#define ES9018_PHASE_SHIFT		0x18	// 8'b00110000
#define ES9018_DPLL_MODE	   	0x19	// 8'b00000010
#define ES9018_STATUS   		0x1B	// [3]b1:DSD 0:I2S/SPDIF
#define ES9018_PROG_ENABLE		0x25	// 8'b00000000
#define ES9018_SPDIF_STS_00   	0x30	// reg48
#define ES9018_SPDIF_STS_01   	0x31
#define ES9018_SPDIF_STS_02   	0x32
#define ES9018_SPDIF_STS_03   	0x33
#define ES9018_SPDIF_STS_04   	0x34
#define ES9018_SPDIF_STS_22   	0x46	// professional reliablility flags
#define ES9018_SPDIF_STS_23   	0x47	// professional CRCC

#define ES9018_REGISTER_COUNT                   22
#define ES9018_MAX_REGISTER                     0x48

/*
 * Field Definitions.
 */

/*
 * R0 (0x00) - DAC1_ATTENUATION
 * R1 (0x01) - DAC2_ATTENUATION
 * R2 (0x02) - DAC3_ATTENUATION
 * R3 (0x03) - DAC4_ATTENUATION
 * R4 (0x04) - DAC5_ATTENUATION
 * R5 (0x05) - DAC6_ATTENUATION
 * R6 (0x06) - DAC7_ATTENUATION
 * R7 (0x07) - DAC8_ATTENUATION
 default 0x0
 Reg[7:0] 8b00000000:0dB 8b11111111:-127.5dB
 dB's = - REG_VALUE/2
 */

/*
 *R8 (0x08) - ES9018_AUTOMUTE_LEV default 0x68 { b[7]0,b[6:0]110 1000}
*/ 
#define ES9018_SPDIF_ENALBE       0x80  /* 7b1:USE SPDIF INPUT , 7b0:USE I2S/DSD */
#define ES9018_SPDIF_MASK         0x80  /* SPDIF I2S/DSD */
#define ES9018_SPDIF_SHIFT        7  	  /* SPDIF I2S/DSD */
#define ES9018_SPDIF_WIDTH        1       /* SPDIF I2S/DSD */
#define ES9018_AUTOMUTE			  0x7F	/* automute trigger point in dB's = -REG_VALE*/
#define ES9018_AUTOMUTE_MASK      0x7F  /* Mute - [6:0] */
#define ES9018_AUTOMUTE_SHIFT     0
#define ES9018_AUTOMUTE_WIDTH     7

/*
 * R9 (0x09) - ES9018_AUTOMUTE_TIME default 0x4
 * Time in Seconds = 2096896/(REG_VALUE*DATA_CLK)
*/

/*
 * R10 (0x0A) - ES9018_MODE_CONTROL_1 default 0xCE 8'b11001110
 */
#define ES9018_PARAMFMT			  	 0xC0  /* [7:6] 2'b00=24bit, 2'b01=20bit, 2'b10=16bit, 2'b11=32bit */
#define ES9018_PARAMFMT_MASK         0xC0
#define ES9018_PARAMFMT_SHIFT        6
#define ES9018_PARAMFMT_WIDTH        2
#define ES9018_IFFMT            	 0x30  /* [5:4] 2'b00=I2s, 2'b01=LJ, 2'b10=RJ, 2'b11=I2S */
#define ES9018_IFFMT_MASK			 0x30
#define ES9018_IFFMT_SHIFT           4
#define ES9018_IFFMT_WIDTH       	 2
#define ES9018_MODE1_RES             0x08  /* must be set 1'b1 for normal operation */
#define ES9018_MODE1_RES_MASK        0x08
#define ES9018_MODE1_RES_SHIFT 		 3
#define ES9018_MODE1_RES_WIDTH       1
#define ES9018_JITTER                0x04  /* 1'b1=use jitter_reduction, 1'b0=bypass and stop jitter_reduction */
#define ES9018_JITTER_MASK        	 0x04
#define ES9018_JITTER_SHIFT 		 2
#define ES9018_JITTER_WIDTH          1
#define ES9018_BYPASS             0x02  /* 1'b1=bypass de-emphasize filter, 1'b0=use de-emphasize filter */
#define ES9018_BYPASS_MASK        0x02
#define ES9018_BYPASS_SHIFT 		 1
#define ES9018_BYPASS_WIDTH       	1
#define ES9018_MUTE_DAC             0x01  /* 1'b1=mute all dac's, 1'b0 = unmute all dac's */
#define ES9018_MUTE_DAC_MASK        0x01
#define ES9018_MUTE_DAC_SHIFT 		 0
#define ES9018_MUTE_DAC_WIDTH       1

/*
 * R11 (0x0B) - ES9018_MODE_CONTROL_2 default 0x85 8'b10000101
 */
#define ES9018_MODE2_RES             0xE0  /* must be set 3'b100 for normal operation */
#define ES9018_MODE2_RES_MASK        0xE0  
#define ES9018_MODE2_RES_SHIFT       5  
#define ES9018_MODE2_RES_WIDTH       3  
#define ES9018_DPLL_BANDW            0x1C 
									/* 3'b000=No Bandwidth */
									/* 3'b001=Lowest Bandwidth */
									/* 3'b010=Low Bandwidth */
									/* 3'b011=Med-Low Bandwidth */
									/* 3'b100=Medium Bandwidth */
									/* 3'b101=Med-High Bandwidth */
									/* 3'b110=High Bandwidth */
									/* 3'b111=Highest Bandwidth */
#define ES9018_DPLL_BANDW_MASK       0x1C 
#define ES9018_DPLL_BANDW_SHIFT      2
#define ES9018_DPLL_BANDW_WIDTH      3
#define ES9018_DE_EMPHASIS           0x03  /* 2'b00=32kHz, 2'b01=44.1kHz, 2'b10=48kHz */
#define ES9018_DE_EMPHASIS_MASK      0x03
#define ES9018_DE_EMPHASIS_SHIFT     0
#define ES9018_DE_EMPHASIS_WIDTH     2

/*
 * R12 (0x0C) - ES9018_MODE_CONTROL_3 default 0x20 8'b00100000
 */

/*
 * R13 (0x0D) - ES9018_DAC_POL default 0x0 8'b00000000
 */
#define ES9018_POL_DAC8              0x80  /* 1'b1=Anti-Phase, 1'b0=In-Phase */
#define ES9018_POL_DAC8_MASK         0x80
#define ES9018_POL_DAC8_SHIFT        7
#define ES9018_POL_DAC8_WIDTH        1
#define ES9018_POL_DAC7              0x40  /* 1'b1=Anti-Phase, 1'b0=In-Phase */
#define ES9018_POL_DAC7_MASK         0x40
#define ES9018_POL_DAC7_SHIFT        6
#define ES9018_POL_DAC7_WIDTH        1
#define ES9018_POL_DAC6              0x20  /* 1'b1=Anti-Phase, 1'b0=In-Phase */
#define ES9018_POL_DAC6_MASK         0x20
#define ES9018_POL_DAC6_SHIFT        5
#define ES9018_POL_DAC6_WIDTH        1
#define ES9018_POL_DAC5              0x10  /* 1'b1=Anti-Phase, 1'b0=In-Phase */
#define ES9018_POL_DAC5_MASK         0x10
#define ES9018_POL_DAC5_SHIFT        4
#define ES9018_POL_DAC5_WIDTH        1
#define ES9018_POL_DAC4              0x08  /* 1'b1=Anti-Phase, 1'b0=In-Phase */
#define ES9018_POL_DAC4_MASK         0x08
#define ES9018_POL_DAC4_SHIFT        3
#define ES9018_POL_DAC4_WIDTH        1
#define ES9018_POL_DAC3              0x04  /* 1'b1=Anti-Phase, 1'b0=In-Phase */
#define ES9018_POL_DAC3_MASK         0x04
#define ES9018_POL_DAC3_SHIFT        2
#define ES9018_POL_DAC3_WIDTH        1
#define ES9018_POL_DAC2              0x02  /* 1'b1=Anti-Phase, 1'b0=In-Phase */
#define ES9018_POL_DAC2_MASK         0x02
#define ES9018_POL_DAC2_SHIFT        1
#define ES9018_POL_DAC2_WIDTH        1
#define ES9018_POL_DAC1              0x01  /* 1'b1=Anti-Phase, 1'b0=In-Phase */
#define ES9018_POL_DAC1_MASK         0x01
#define ES9018_POL_DAC1_SHIFT        0
#define ES9018_POL_DAC1_WIDTH        1

/*
 * R14 (0x0E) - ES9018_DAC_SRC_3478 default 0x0B 8'b00001011
 */
#define ES9018_SRC_DAC8              0x80  /* 1'b1=DAC6, 1'b0=DAC8 */
#define ES9018_SRC_DAC8_MASK         0x80
#define ES9018_SRC_DAC8_SHIFT        7
#define ES9018_SRC_DAC8_WIDTH        1
#define ES9018_SRC_DAC7              0x40  /* 1'b1=DAC5, 1'b0=DAC7 */
#define ES9018_SRC_DAC7_MASK         0x40
#define ES9018_SRC_DAC7_SHIFT        6
#define ES9018_SRC_DAC7_WIDTH        1
#define ES9018_SRC_DAC4              0x20  /* 1'b1=DAC2, 1'b0=DAC4 */
#define ES9018_SRC_DAC4_MASK         0x20
#define ES9018_SRC_DAC4_SHIFT        5
#define ES9018_SRC_DAC4_WIDTH        1
#define ES9018_SRC_DAC3              0x10  /* 1'b1=DAC1, 1'b0=DAC3 */
#define ES9018_SRC_DAC3_MASK         0x10
#define ES9018_SRC_DAC3_SHIFT        4
#define ES9018_SRC_DAC3_WIDTH        1
#define ES9018_SRC_RES              0x08  /* must be set 1'b1 */
#define ES9018_SRC_RES_MASK         0x08
#define ES9018_SRC_RES_SHIFT        3
#define ES9018_SRC_RES_WIDTH        1
#define ES9018_IIR_BANDW              0x06  /* 2'b00=Normal,2'b01=50k,2'b10=60k,2'b11=70k */
#define ES9018_IIR_BANDW_MASK         0x06
#define ES9018_IIR_BANDW_SHIFT        1
#define ES9018_IIR_BANDW_WIDTH        2
#define ES9018_FIR_ROLLOFF              0x01  /* 1'b1=Fast Rolloff, 1'b0=Slow Rolloff */
#define ES9018_FIR_ROLLOFF_MASK         0x01
#define ES9018_FIR_ROLLOFF_SHIFT        0
#define ES9018_FIR_ROLLOFF_WIDTH        1

/*
 * R15 (0x0F) - ES9018_MODE_CONTROL_4 default 0x0 8'b00000000
 */

/*
 * R16 (0x10) - ES9018_AUTOMUTE_LPBACK default 0x0 8'b00000000
 */

/*
 * R17 (0x11) - ES9018_MODE_CONTROL_5 default 0x1C 8'b00011100
 */

/*
 * R18 (0x12) - ES9018_SPDIF_SRC default 0x01 8'b00000001 spdif=data1
 */


/*
 * R19 (0x13) - ES9018_DACB_POL default 0x0 8'b00000000
 */
#define ES9018_POL_DACB8              0x80  /* 1'b0=Anti-Phase, 1'b1=In-Phase */
#define ES9018_POL_DACB8_MASK         0x80
#define ES9018_POL_DACB8_SHIFT        7
#define ES9018_POL_DACB8_WIDTH        1
#define ES9018_POL_DACB7              0x40  /* 1'b0=Anti-Phase, 1'b1=In-Phase */
#define ES9018_POL_DACB7_MASK         0x40
#define ES9018_POL_DACB7_SHIFT        6
#define ES9018_POL_DACB7_WIDTH        1
#define ES9018_POL_DACB6              0x20  /* 1'b0=Anti-Phase, 1'b1=In-Phase */
#define ES9018_POL_DACB6_MASK         0x20
#define ES9018_POL_DACB6_SHIFT        5
#define ES9018_POL_DACB6_WIDTH        1
#define ES9018_POL_DACB5              0x10  /* 1'b0=Anti-Phase, 1'b1=In-Phase */
#define ES9018_POL_DACB5_MASK         0x10
#define ES9018_POL_DACB5_SHIFT        4
#define ES9018_POL_DACB5_WIDTH        1
#define ES9018_POL_DACB4              0x08  /* 1'b0=Anti-Phase, 1'b1=In-Phase */
#define ES9018_POL_DACB4_MASK         0x08
#define ES9018_POL_DACB4_SHIFT        3
#define ES9018_POL_DACB4_WIDTH        1
#define ES9018_POL_DACB3              0x04  /* 1'b0=Anti-Phase, 1'b1=In-Phase */
#define ES9018_POL_DACB3_MASK         0x04
#define ES9018_POL_DACB3_SHIFT        2
#define ES9018_POL_DACB3_WIDTH        1
#define ES9018_POL_DACB2              0x02  /* 1'b0=Anti-Phase, 1'b1=In-Phase */
#define ES9018_POL_DACB2_MASK         0x02
#define ES9018_POL_DACB2_SHIFT        1
#define ES9018_POL_DACB2_WIDTH        1
#define ES9018_POL_DACB1              0x01  /* 1'b0=Anti-Phase, 1'b0=In-Phase */
#define ES9018_POL_DACB1_MASK         0x01
#define ES9018_POL_DACB1_SHIFT        0
#define ES9018_POL_DACB1_WIDTH        1

/*
 * R20~23 (0x14~0x17) - ES9018_MASTR_TRIM default 0x7fffffff
 */

/*
 * R24 (0x18) - ES9018_PHASE_SHIFT default 0x30 8'b00110000
 */
#define ES9018_PHASESHIFT              0x0F  /* 4'd0=default, delay time = default REG_VALUE/clk delay */
#define ES9018_PHASESHIFT_MASK         0x0F
#define ES9018_PHASESHIFT_SHIFT        0
#define ES9018_PHASESHIFT_WIDTH        4

/*
 * R25 (0x19) - ES9018_DPLL_MODE default 0x02 8'b00000010
 */
#define ES9018_DPLL_BW				0x02	/* 1'b1=Use the best DPLL bandwidth settins, 1'b0=Allow all settings */
#define ES9018_DPLL_BW_MASK         0x02
#define ES9018_DPLL_BW_SHIFT        1
#define ES9018_DPLL_BW_WIDTH        1
#define ES9018_DPLL_BW_128X				0x01	/* 1'b1=Multiply dpll bw by 128, 1'b0=use dpll bw settings */
#define ES9018_DPLL_BW_128X_MASK         0x01
#define ES9018_DPLL_BW_128X_SHIFT        0
#define ES9018_DPLL_BW_128X_WIDTH        1

/*
 * R27 (0x1B) - ES9018_STATUS  Read only
*/
#define ES9018_STS_DSD_PCM			0x08	/* 1'b1=DSD, 1'b0=I2S/SPDIF */
#define ES9018_STS_SPDIF_VALID		0x04	/* 1'b1=Spdif data is valid, 1'b0=Spdif data is invalid */
#define ES9018_STS_SPDIF_EN			0x02	/* 1'b1=Spdif mode is enable -> R8, 1'b0=Spdif mode is disabled */
#define ES9018_STS_LOCK				0x01	/* 1'b1=The Jitter Eliminator is locked to an incoming signal */
											/* 1'b0=not locked */

/*
 * R28~31 (0x1C~0x1F) - ES9018_DPLL_NUM read only Fin = DPLL_NUM*2^32/Fcrystal
 */

/*
 * R37 (0x25) - ES9018_PROG_ENABLE default 0x0
*/

/*
 * R48~71 (0x30~0x47) - ES9018_SPDIF_STS_00~23
*/

#define ES9018_NUM_RATES 6


#define  EVENT_PLAYBACK_START 0
#define  EVENT_PLAYBACK_OFF   1
#define  EVENT_LINEOUT_IN     2
#define  EVENT_LINEOUT_OUT    3
#define  EVENT_HP_IN          4
#define  EVENT_HP_OUT         5




#define AM1 1
#define AM2 2
#define AM3 3
#define AM4 4
#define AM5 5
#define AM6 6
#define AM7 7
#define AM8 8
#endif	/*_ES9018_H*/
