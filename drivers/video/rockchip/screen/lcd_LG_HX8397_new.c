
#ifndef __LCD_LG_HX8397__
#define __LCD_LG_HX8397__

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/i2c.h>

#include "lcd_X7II_define.h"

/* Base */
#define SCREEN_TYPE_LG		SCREEN_RGB
#define OUT_FACE_LG		OUT_D888_P666//OUT_P888
#define LVDS_FORMAT_LG       	LVDS_8BIT_1
#define DCLK_LG			26*1000*1000	//***27
#define LCDC_ACLK_LG       	300000000     //29 lcdc axi DMA Æµï¿œï¿œ           //rk29

/* Timing */
#define H_PW_LG			4 //8Ç°ÏûÓ°
#define H_BP_LG			8//6
#define H_VD_LG			480//320	//***800 
#define H_FP_LG			8//60

#define V_PW_LG			4//12
#define V_BP_LG			8// 4
#define V_VD_LG			800//480	//***480
#define V_FP_LG			8//40

#define LCD_WIDTH_LG            54//57    //lcd size *mm
#define LCD_HEIGHT_LG           88//94

/* Other */
#define DCLK_POL_LG		1//0 
#define DEN_POL_LG		0
#define VSYNC_POL_LG		0
#define HSYNC_POL_LG		0

#define SWAP_RB_LG		0
#define SWAP_RG_LG		0
#define SWAP_GB_LG		0

static void HX8397_core_init(void)
{
        printk("%s: debug !!!\n", __FUNCTION__);

Write_LCD_CMD(0xB9);
Write_LCD_DATA(0xFF);
Write_LCD_DATA(0x83);
Write_LCD_DATA(0x79);

Write_LCD_CMD(0xB1);
Write_LCD_DATA(0x44);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x31);
Write_LCD_DATA(0x51);
Write_LCD_DATA(0x50);
Write_LCD_DATA(0xDC);
Write_LCD_DATA(0xEE);
Write_LCD_DATA(0x94);
Write_LCD_DATA(0x80);
Write_LCD_DATA(0x38);
Write_LCD_DATA(0x38);
Write_LCD_DATA(0xF8);
Write_LCD_DATA(0x33);
Write_LCD_DATA(0x32);
Write_LCD_DATA(0x22);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x80);
Write_LCD_DATA(0x30);
Write_LCD_DATA(0x00);


Write_LCD_CMD(0xB2);
Write_LCD_DATA(0x80);
Write_LCD_DATA(0x3C);
Write_LCD_DATA(0x0C);
Write_LCD_DATA(0x03);
Write_LCD_DATA(0x30);
Write_LCD_DATA(0x50);
Write_LCD_DATA(0x11);
Write_LCD_DATA(0x42);
Write_LCD_DATA(0x1D);

Write_LCD_CMD(0xB4);
Write_LCD_DATA(0x01);
Write_LCD_DATA(0x78);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x7A);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x7A);
Write_LCD_DATA(0x0F);
Write_LCD_DATA(0x7E);
Write_LCD_DATA(0x0F);
Write_LCD_DATA(0x7E);

Write_LCD_CMD(0xD2);
Write_LCD_DATA(0x33);

Write_LCD_CMD(0xD3);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x01);
Write_LCD_DATA(0x01);
Write_LCD_DATA(0x08);
Write_LCD_DATA(0x08);
Write_LCD_DATA(0x32);
Write_LCD_DATA(0x10);
Write_LCD_DATA(0x01);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x01);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x37);
Write_LCD_DATA(0x83);
Write_LCD_DATA(0x05);
Write_LCD_DATA(0x05);
Write_LCD_DATA(0x37);
Write_LCD_DATA(0x04);
Write_LCD_DATA(0x04);
Write_LCD_DATA(0x37);
Write_LCD_DATA(0x0A);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x0A);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x10);

Write_LCD_CMD(0xCC);
Write_LCD_DATA(0x02);

Write_LCD_CMD(0xD5);
Write_LCD_DATA(0x19);
Write_LCD_DATA(0x19);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x1B);
Write_LCD_DATA(0x1B);
Write_LCD_DATA(0x1A);
Write_LCD_DATA(0x1A);
Write_LCD_DATA(0x02);
Write_LCD_DATA(0x03);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x01);
Write_LCD_DATA(0x06);
Write_LCD_DATA(0x07);
Write_LCD_DATA(0x04);
Write_LCD_DATA(0x05);
Write_LCD_DATA(0x21);
Write_LCD_DATA(0x20);
Write_LCD_DATA(0x23);
Write_LCD_DATA(0x22);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);

Write_LCD_CMD(0xD6);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x19);
Write_LCD_DATA(0x19);
Write_LCD_DATA(0x1A);
Write_LCD_DATA(0x1A);
Write_LCD_DATA(0x1B);
Write_LCD_DATA(0x1B);
Write_LCD_DATA(0x05);
Write_LCD_DATA(0x04);
Write_LCD_DATA(0x07);
Write_LCD_DATA(0x06);
Write_LCD_DATA(0x01);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x03);
Write_LCD_DATA(0x02);
Write_LCD_DATA(0x22);
Write_LCD_DATA(0x23);
Write_LCD_DATA(0x20);
Write_LCD_DATA(0x21);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x18);

Write_LCD_CMD(0xE0);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x05);
Write_LCD_DATA(0x0B);
Write_LCD_DATA(0x2D);
Write_LCD_DATA(0x35);
Write_LCD_DATA(0x3F);
Write_LCD_DATA(0x28);
Write_LCD_DATA(0x47);
Write_LCD_DATA(0x08);
Write_LCD_DATA(0x0B);
Write_LCD_DATA(0x0D);
Write_LCD_DATA(0x17);
Write_LCD_DATA(0x0D);
Write_LCD_DATA(0x11);
Write_LCD_DATA(0x13);
Write_LCD_DATA(0x10);
Write_LCD_DATA(0x13);
Write_LCD_DATA(0x07);
Write_LCD_DATA(0x11);
Write_LCD_DATA(0x14);
Write_LCD_DATA(0x18);
Write_LCD_DATA(0x00);
Write_LCD_DATA(0x06);
Write_LCD_DATA(0x0A);
Write_LCD_DATA(0x2D);
Write_LCD_DATA(0x34);
Write_LCD_DATA(0x3F);
Write_LCD_DATA(0x27);
Write_LCD_DATA(0x48);
Write_LCD_DATA(0x08);
Write_LCD_DATA(0x0B);
Write_LCD_DATA(0x0D);
Write_LCD_DATA(0x17);
Write_LCD_DATA(0x0D);
Write_LCD_DATA(0x11);
Write_LCD_DATA(0x13);
Write_LCD_DATA(0x11);
Write_LCD_DATA(0x13);
Write_LCD_DATA(0x06);
Write_LCD_DATA(0x11);
Write_LCD_DATA(0x12);
Write_LCD_DATA(0x18);

Write_LCD_CMD(0xB6);
Write_LCD_DATA(0x82);
Write_LCD_DATA(0x82);

 
Write_LCD_CMD(0x3A);
Write_LCD_DATA(0x66);

Write_LCD_CMD(0xE4);
Write_LCD_DATA(0x82);

Write_LCD_CMD(0x11);
mdelay(150);
  
Write_LCD_CMD(0x29);
mdelay(100);
}

static void init_HX8397_IO(void)
{
        LCD_RST_OUT();
        LCD_RST(1);
        mdelay(30);
	LCD_RST(0);
	mdelay(100);
	LCD_RST(1);
	mdelay(100);
        gpio_free(LCD_RESET_PORT);

        HX8397_core_init();
}

static void lcd_LG_suspend(void)
{
	Write_LCD_CMD(0x28);
	mdelay(50);
	Write_LCD_CMD(0x10);
	mdelay(50);
}

static void lcd_LG_resume(void)
{
	Write_LCD_CMD(0x11);
	mdelay(150);
	Write_LCD_CMD(0x29);
	mdelay(50);
}

#endif
