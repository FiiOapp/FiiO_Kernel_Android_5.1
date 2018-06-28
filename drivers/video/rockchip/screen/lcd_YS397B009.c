
#ifndef __LCD_YS397B009__
#define __LCD_YS397B009__

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/i2c.h>
#include "../../../regulator/axp_power/axp-mfd.h"

/* BOE PARAM*/
#define SCREEN_TYPE_BOE		SCREEN_RGB
#define OUT_FACE_BOE		OUT_D888_P666
#define DCLK_BOE			26*1000*1000
#define LCDC_ACLK_BOE       300000000

#define H_PW_BOE             8
#define H_BP_BOE             6
#define H_VD_BOE             480
#define H_FP_BOE             8

#define V_PW_BOE			 8
#define V_BP_BOE			 8
#define V_VD_BOE			 800
#define V_FP_BOE			 8

#define LCD_WIDTH_BOE        52
#define LCD_HEIGHT_BOE       88

#define DCLK_POL_BOE		    1 
#define DEN_POL_BOE			0
#define VSYNC_POL_BOE		0
#define HSYNC_POL_BOE		0

#define SWAP_RB_BOE			0
#define SWAP_RG_BOE			0
#define SWAP_GB_BOE			0

/* BOE_STST7701S PARAM*/
#define SCREEN_TYPE_BOE_ST		SCREEN_RGB
#define OUT_FACE_BOE_ST		OUT_D888_P666
#define DCLK_BOE_ST			26*1000*1000
#define LCDC_ACLK_BOE_ST       300000000

#define H_PW_BOE_ST             8
#define H_BP_BOE_ST             6
#define H_VD_BOE_ST             480
#define H_FP_BOE_ST             8

#define V_PW_BOE_ST			 8
#define V_BP_BOE_ST			 8
#define V_VD_BOE_ST			 800
#define V_FP_BOE_ST			 8

#define LCD_WIDTH_BOE_ST        52
#define LCD_HEIGHT_BOE_ST       88

#define DCLK_POL_BOE_ST		    1 
#define DEN_POL_BOE_ST			0
#define VSYNC_POL_BOE_ST		0
#define HSYNC_POL_BOE_ST		0

#define SWAP_RB_BOE_ST			1
#define SWAP_RG_BOE_ST			0
#define SWAP_GB_BOE_ST			0



/*TM PARAM*/
#define SCREEN_TYPE_TM      SCREEN_RGB
#define OUT_FACE_TM         OUT_D888_P666
#define DCLK_TM             27*1000*1000
#define LCDC_ACLK_TM        300000000 

#define H_PW_TM             22
#define H_BP_TM             12
#define H_VD_TM             480
#define H_FP_TM             8

#define V_PW_TM			    28
#define V_BP_TM			    18
#define V_VD_TM			    800
#define V_FP_TM			    16

#define LCD_WIDTH_TM        52
#define LCD_HEIGHT_TM       88

#define DCLK_POL_TM		    1 
#define DEN_POL_TM			0
#define VSYNC_POL_TM		0
#define HSYNC_POL_TM		0

#define SWAP_RB_TM			0
#define SWAP_RG_TM			0
#define SWAP_GB_TM			0


/*LG PARAM*/
#define SCREEN_TYPE_LG      SCREEN_RGB
#define OUT_FACE_LG         OUT_D888_P666//OUT_P888
#define DCLK_LG             26*1000*1000	//***27
#define LCDC_ACLK_LG        300000000     //29 lcdc axi DMA Æµï¿œï¿œ           //rk29
#define H_PW_LG             8
#define H_BP_LG             6
#define H_VD_LG             480
#define H_FP_LG             8

#define V_PW_LG			    8
#define V_BP_LG			    8
#define V_VD_LG			    800
#define V_FP_LG			    8

#define LCD_WIDTH_LG        52
#define LCD_HEIGHT_LG       88

#define DCLK_POL_LG		    1 
#define DEN_POL_LG			0
#define VSYNC_POL_LG		0
#define HSYNC_POL_LG		0

#define SWAP_RB_LG			0
#define SWAP_RG_LG			0
#define SWAP_GB_LG			0

/*BOE_SCREEN_ILI9806E PARAM*/
#define SCREEN_TYPE_BOE_ILI9806E      SCREEN_RGB
#define OUT_FACE_BOE_ILI9806E         OUT_D888_P666
#define DCLK_BOE_ILI9806E             27*1000*1000
#define LCDC_ACLK_BOE_ILI9806E        300000000 

#define H_PW_BOE_ILI9806E             22
#define H_BP_BOE_ILI9806E             12
#define H_VD_BOE_ILI9806E             480
#define H_FP_BOE_ILI9806E             8

#define V_PW_BOE_ILI9806E			    28
#define V_BP_BOE_ILI9806E			    18
#define V_VD_BOE_ILI9806E			    800
#define V_FP_BOE_ILI9806E			    16

#define LCD_WIDTH_BOE_ILI9806E        52
#define LCD_HEIGHT_BOE_ILI9806E       88

#define DCLK_POL_BOE_ILI9806E		    1 
#define DEN_POL_BOE_ILI9806E			0
#define VSYNC_POL_BOE_ILI9806E		0
#define HSYNC_POL_BOE_ILI9806E		0

#define SWAP_RB_BOE_ILI9806E			0
#define SWAP_RG_BOE_ILI9806E			0
#define SWAP_GB_BOE_ILI9806E			0




#define RK_SCREEN_INIT 	1

#if defined(RK_SCREEN_INIT)

#define LCD_RESET_PORT         RK30_PIN2_PD4

static struct rk29lcd_info *gLcd_info = NULL;
static struct rk29fb_screen *gLcd_screen = NULL;
static unsigned char reg_data[3] = {0};
static bool isLGScreen = false;
static bool isTMILI9806= false;
static bool isBOEScreen= false;
static bool isBOEScreenILI9806E= false;

static int gLcd_type=-1;

enum screen_type{
    LG_SCREEN,
    TM_SCREEN,
    BOE_SCREEN,
    BOE_SCREEN_ILI9806E,
};

int rk_lcd_init(void);
int rk_lcd_standby(u8 enable);
int boe_ili9806e_init();

#define TXD_PORT        gLcd_info->txd_pin
#define CLK_PORT        gLcd_info->clk_pin
#define CS_PORT         gLcd_info->cs_pin

#define CS_OUT()        gpio_direction_output(CS_PORT, 1)
#define CS_SET()        gpio_set_value(CS_PORT, GPIO_HIGH)
#define CS_CLR()        gpio_set_value(CS_PORT, GPIO_LOW)
#define CLK_OUT()       gpio_direction_output(CLK_PORT, 0) 
#define CLK_SET()       gpio_set_value(CLK_PORT, GPIO_HIGH)
#define CLK_CLR()       gpio_set_value(CLK_PORT, GPIO_LOW)
#define TXD_OUT()       gpio_direction_output(TXD_PORT, 1) 
#define TXD_SET()       gpio_set_value(TXD_PORT, GPIO_HIGH)
#define TXD_CLR()       gpio_set_value(TXD_PORT, GPIO_LOW)
#define TXD_IN()        gpio_direction_input(TXD_PORT)
#define TXD_GET()       gpio_get_value(TXD_PORT)

#define LCD_RST_OUT()   gpio_direction_output(LCD_RESET_PORT, 0)
#define LCD_RST(i)      gpio_set_value(LCD_RESET_PORT, i)

#define DRVmsleepUs(i)   umsleep(i*4)
#define delay_us(i)      udelay(i)

#define TFT_DET RK30_PIN3_PB3

#if 1
void WriteCommand( int  Command)
{
	unsigned char i,count1, count2,count3,count4;
	count1= Command>>8;
	count2= Command;
	count3=0x20;//00100000   //ÐŽÃüÁîžßÎ»
	count4=0x00;//00000000   //ÐŽÃüÁîµÍÎ»======ŸßÌåÇë¿ŽICµÄDatasheet
	CS_CLR();
	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count3 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count3<<=1;
	}

	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count1 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count1<<=1;
	}

	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count4 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count4<<=1;
	}
	
	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count2 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count2<<=1;
	}

	CS_SET();

}



void WriteParameter(char DH)
{
	unsigned char i, count1, count2,count3,count4;
	count1=DH>>8;
	count2=DH;
	count3=0x60;//ÐŽÊýŸÝžßÎ»
	count4=0x40;//ÐŽÊýŸÝµÍÎ»

	CS_CLR();

	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count4 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count4<<=1;
	}

	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count2 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count2<<=1;
	}

	CS_SET();

}
#endif

void Write_Index_Data(int  Command, char DH)
{
	unsigned char i,count1, count2,count3,count4;
	count1= Command>>8;
	count2= Command;
	count3=0x20;//00100000   //ÐŽÃüÁîžßÎ»
	count4=0x00;//00000000   //ÐŽÃüÁîµÍÎ»======ŸßÌåÇë¿ŽICµÄDatasheet
	CS_CLR();
	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count3 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count3<<=1;
	}

	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count1 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count1<<=1;
	}

	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count4 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count4<<=1;
	}
	
	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count2 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count2<<=1;
	}

	CS_SET();

	count1=DH>>8;
	count2=DH;
	count3=0x60;//ÐŽÊýŸÝžßÎ»
	count4=0x40;//ÐŽÊýŸÝµÍÎ»

	CS_CLR();

	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count4 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count4<<=1;
	}

	for(i=0;i<8;i++)
	{
		CLK_CLR();
		if (count2 & 0x80) TXD_SET();
		else             TXD_CLR();
		CLK_SET();
		count2<<=1;
	}

	CS_SET();


}

#define UDELAY_TIME     1
void Write_LCD_CMD(unsigned char index)
{
	int  j;
	CS_CLR();
	TXD_CLR();	//0
	udelay(UDELAY_TIME);
	  	
	CLK_CLR();
	udelay(3);//
        
	CLK_SET();        
	udelay(UDELAY_TIME);

	TXD_CLR();
	CLK_CLR();
		
	for(j=0;j<8;j++)
	{
		if(index&0x80)
		{
			TXD_SET();	
		}
		else
		{
			TXD_CLR();
		}
		index<<=1;	
			
		CLK_CLR();     
		udelay(UDELAY_TIME);
		CLK_SET();
		udelay(UDELAY_TIME);	
	}
	CS_SET();	
}

void Write_LCD_DATA(unsigned char data)
{
	int j;
	CS_CLR();
	TXD_SET();	
	udelay(UDELAY_TIME);
	  	
	CLK_CLR();
	udelay(3);
        
	CLK_SET();
	udelay(UDELAY_TIME);
		
	TXD_CLR();
	CLK_CLR();
		
	for(j=0;j<8;j++)
	{
		if(data&0x80)
		{
			TXD_SET();	
		}
		else
		{
			TXD_CLR();
		}
		data<<=1;	
			
		CLK_CLR();     
		udelay(UDELAY_TIME);
		CLK_SET();
		udelay(UDELAY_TIME);	
	}
	CS_SET();
}

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
Write_LCD_DATA(0x2F);
Write_LCD_DATA(0x2F);
Write_LCD_DATA(0x50);
Write_LCD_DATA(0xDC);
Write_LCD_DATA(0xE2);
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
Write_LCD_DATA(0x10);
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

static void init_YS397B009(void)
{

Write_Index_Data(0xF000,0x55);
Write_Index_Data(0xF001,0xAA);
Write_Index_Data(0xF002,0x52);
Write_Index_Data(0xF003,0x08);
Write_Index_Data(0xF004,0x02);
Write_Index_Data(0xF600,0x60);
Write_Index_Data(0xF601,0x40);
Write_Index_Data(0xFE00,0x01);
Write_Index_Data(0xFE01,0x80);
Write_Index_Data(0xFE02,0x09);
Write_Index_Data(0xFE03,0x09);
msleep(200);
Write_Index_Data(0xF000,0x55);
Write_Index_Data(0xF001,0xAA);
Write_Index_Data(0xF002,0x52);
Write_Index_Data(0xF003,0x08);
Write_Index_Data(0xF004,0x01);
Write_Index_Data(0xB000,0x0D);
Write_Index_Data(0xB001,0x0D);
Write_Index_Data(0xB002,0x0D);
Write_Index_Data(0xB100,0x0D);
Write_Index_Data(0xB101,0x0D);
Write_Index_Data(0xB102,0x0D);
Write_Index_Data(0xB200,0x00);
Write_Index_Data(0xB500,0x08);
Write_Index_Data(0xB501,0x08);
Write_Index_Data(0xB502,0x08);
Write_Index_Data(0xB600,0x34);
Write_Index_Data(0xB601,0x34);
Write_Index_Data(0xB602,0x34);
Write_Index_Data(0xB700,0x44);
Write_Index_Data(0xB701,0x44);
Write_Index_Data(0xB702,0x44);
Write_Index_Data(0xB800,0x24);
Write_Index_Data(0xB801,0x24);
Write_Index_Data(0xB802,0x24);
Write_Index_Data(0xB900,0x34);
Write_Index_Data(0xB901,0x34);
Write_Index_Data(0xB902,0x34);
Write_Index_Data(0xBA00,0x14);
Write_Index_Data(0xBA01,0x14);
Write_Index_Data(0xBA02,0x14);
Write_Index_Data(0xBC00,0x00);
Write_Index_Data(0xBC01,0x78);
Write_Index_Data(0xBC02,0x00);
Write_Index_Data(0xBD00,0x00);
Write_Index_Data(0xBD01,0x78);
Write_Index_Data(0xBD02,0x00);
Write_Index_Data(0xBE00,0x00);
Write_Index_Data(0xBE01,0x33);
Write_Index_Data(0xD100,0x00);
Write_Index_Data(0xD101,0x00);
Write_Index_Data(0xD102,0x00);
Write_Index_Data(0xD103,0x11);
Write_Index_Data(0xD104,0x00);
Write_Index_Data(0xD105,0x2E);
Write_Index_Data(0xD106,0x00);
Write_Index_Data(0xD107,0x47);
Write_Index_Data(0xD108,0x00);
Write_Index_Data(0xD109,0x5C);
Write_Index_Data(0xD10A,0x00);
Write_Index_Data(0xD10B,0x7E);
Write_Index_Data(0xD10C,0x00);
Write_Index_Data(0xD10D,0x9C);
Write_Index_Data(0xD10E,0x00);
Write_Index_Data(0xD10F,0xCD);
Write_Index_Data(0xD110,0x00);
Write_Index_Data(0xD111,0xF6);
Write_Index_Data(0xD112,0x01);
Write_Index_Data(0xD113,0x36);
Write_Index_Data(0xD114,0x01);
Write_Index_Data(0xD115,0x6A);
Write_Index_Data(0xD116,0x01);
Write_Index_Data(0xD117,0xBD);
Write_Index_Data(0xD118,0x01);
Write_Index_Data(0xD119,0xFF);
Write_Index_Data(0xD11A,0x02);
Write_Index_Data(0xD11B,0x01);
Write_Index_Data(0xD11C,0x02);
Write_Index_Data(0xD11D,0x3B);
Write_Index_Data(0xD11E,0x02);
Write_Index_Data(0xD11F,0x75);
Write_Index_Data(0xD120,0x02);
Write_Index_Data(0xD121,0x99);
Write_Index_Data(0xD122,0x02);
Write_Index_Data(0xD123,0xC8);
Write_Index_Data(0xD124,0x02);
Write_Index_Data(0xD125,0xE8);
Write_Index_Data(0xD126,0x03);
Write_Index_Data(0xD127,0x11);
Write_Index_Data(0xD128,0x03);
Write_Index_Data(0xD129,0x2B);
Write_Index_Data(0xD12A,0x03);
Write_Index_Data(0xD12B,0x48);
Write_Index_Data(0xD12C,0x03);
Write_Index_Data(0xD12D,0x59);
Write_Index_Data(0xD12E,0x03);
Write_Index_Data(0xD12F,0x65);
Write_Index_Data(0xD130,0x03);
Write_Index_Data(0xD131,0x71);
Write_Index_Data(0xD132,0x03);
Write_Index_Data(0xD133,0xFF);
Write_Index_Data(0xD200,0x00);
Write_Index_Data(0xD201,0x00);
Write_Index_Data(0xD202,0x00);
Write_Index_Data(0xD203,0x11);
Write_Index_Data(0xD204,0x00);
Write_Index_Data(0xD205,0x2E);
Write_Index_Data(0xD206,0x00);
Write_Index_Data(0xD207,0x47);
Write_Index_Data(0xD208,0x00);
Write_Index_Data(0xD209,0x5C);
Write_Index_Data(0xD20A,0x00);
Write_Index_Data(0xD20B,0x7E);
Write_Index_Data(0xD20C,0x00);
Write_Index_Data(0xD20D,0x9C);
Write_Index_Data(0xD20E,0x00);
Write_Index_Data(0xD20F,0xCD);
Write_Index_Data(0xD210,0x00);
Write_Index_Data(0xD211,0xF6);
Write_Index_Data(0xD212,0x01);
Write_Index_Data(0xD213,0x36);
Write_Index_Data(0xD214,0x01);
Write_Index_Data(0xD215,0x6A);
Write_Index_Data(0xD216,0x01);
Write_Index_Data(0xD217,0xBD);
Write_Index_Data(0xD218,0x01);
Write_Index_Data(0xD219,0xFF);
Write_Index_Data(0xD21A,0x02);
Write_Index_Data(0xD21B,0x01);
Write_Index_Data(0xD21C,0x02);
Write_Index_Data(0xD21D,0x3B);
Write_Index_Data(0xD21E,0x02);
Write_Index_Data(0xD21F,0x75);
Write_Index_Data(0xD220,0x02);
Write_Index_Data(0xD221,0x99);
Write_Index_Data(0xD222,0x02);
Write_Index_Data(0xD223,0xC8);
Write_Index_Data(0xD224,0x02);
Write_Index_Data(0xD225,0xE8);
Write_Index_Data(0xD226,0x03);
Write_Index_Data(0xD227,0x11);
Write_Index_Data(0xD228,0x03);
Write_Index_Data(0xD229,0x2B);
Write_Index_Data(0xD22A,0x03);
Write_Index_Data(0xD22B,0x48);
Write_Index_Data(0xD22C,0x03);
Write_Index_Data(0xD22D,0x59);
Write_Index_Data(0xD22E,0x03);
Write_Index_Data(0xD22F,0x65);
Write_Index_Data(0xD230,0x03);
Write_Index_Data(0xD231,0x71);
Write_Index_Data(0xD232,0x03);
Write_Index_Data(0xD233,0xFF);
Write_Index_Data(0xD300,0x00);
Write_Index_Data(0xD301,0x00);
Write_Index_Data(0xD302,0x00);
Write_Index_Data(0xD303,0x11);
Write_Index_Data(0xD304,0x00);
Write_Index_Data(0xD305,0x2E);
Write_Index_Data(0xD306,0x00);
Write_Index_Data(0xD307,0x47);
Write_Index_Data(0xD308,0x00);
Write_Index_Data(0xD309,0x5C);
Write_Index_Data(0xD30A,0x00);
Write_Index_Data(0xD30B,0x7E);
Write_Index_Data(0xD30C,0x00);
Write_Index_Data(0xD30D,0x9C);
Write_Index_Data(0xD30E,0x00);
Write_Index_Data(0xD30F,0xCD);
Write_Index_Data(0xD310,0x00);
Write_Index_Data(0xD311,0xF6);
Write_Index_Data(0xD312,0x01);
Write_Index_Data(0xD313,0x36);
Write_Index_Data(0xD314,0x01);
Write_Index_Data(0xD315,0x6A);
Write_Index_Data(0xD316,0x01);
Write_Index_Data(0xD317,0xBD);
Write_Index_Data(0xD318,0x01);
Write_Index_Data(0xD319,0xFF);
Write_Index_Data(0xD31A,0x02);
Write_Index_Data(0xD31B,0x01);
Write_Index_Data(0xD31C,0x02);
Write_Index_Data(0xD31D,0x3B);
Write_Index_Data(0xD31E,0x02);
Write_Index_Data(0xD31F,0x75);
Write_Index_Data(0xD320,0x02);
Write_Index_Data(0xD321,0x99);
Write_Index_Data(0xD322,0x02);
Write_Index_Data(0xD323,0xC8);
Write_Index_Data(0xD324,0x02);
Write_Index_Data(0xD325,0xE8);
Write_Index_Data(0xD326,0x03);
Write_Index_Data(0xD327,0x11);
Write_Index_Data(0xD328,0x03);
Write_Index_Data(0xD329,0x2B);
Write_Index_Data(0xD32A,0x03);
Write_Index_Data(0xD32B,0x48);
Write_Index_Data(0xD32C,0x03);
Write_Index_Data(0xD32D,0x59);
Write_Index_Data(0xD32E,0x03);
Write_Index_Data(0xD32F,0x65);
Write_Index_Data(0xD330,0x03);
Write_Index_Data(0xD331,0x71);
Write_Index_Data(0xD332,0x03);
Write_Index_Data(0xD333,0xFF);
Write_Index_Data(0xD400,0x00);
Write_Index_Data(0xD401,0x00);
Write_Index_Data(0xD402,0x00);
Write_Index_Data(0xD403,0x11);
Write_Index_Data(0xD404,0x00);
Write_Index_Data(0xD405,0x2E);
Write_Index_Data(0xD406,0x00);
Write_Index_Data(0xD407,0x47);
Write_Index_Data(0xD408,0x00);
Write_Index_Data(0xD409,0x5C);
Write_Index_Data(0xD40A,0x00);
Write_Index_Data(0xD40B,0x7E);
Write_Index_Data(0xD40C,0x00);
Write_Index_Data(0xD40D,0x9C);
Write_Index_Data(0xD40E,0x00);
Write_Index_Data(0xD40F,0xCD);
Write_Index_Data(0xD410,0x00);
Write_Index_Data(0xD411,0xF6);
Write_Index_Data(0xD412,0x01);
Write_Index_Data(0xD413,0x36);
Write_Index_Data(0xD414,0x01);
Write_Index_Data(0xD415,0x6A);
Write_Index_Data(0xD416,0x01);
Write_Index_Data(0xD417,0xBD);
Write_Index_Data(0xD418,0x01);
Write_Index_Data(0xD419,0xFF);
Write_Index_Data(0xD41A,0x02);
Write_Index_Data(0xD41B,0x01);
Write_Index_Data(0xD41C,0x02);
Write_Index_Data(0xD41D,0x3B);
Write_Index_Data(0xD41E,0x02);
Write_Index_Data(0xD41F,0x75);
Write_Index_Data(0xD420,0x02);
Write_Index_Data(0xD421,0x99);
Write_Index_Data(0xD422,0x02);
Write_Index_Data(0xD423,0xC8);
Write_Index_Data(0xD424,0x02);
Write_Index_Data(0xD425,0xE8);
Write_Index_Data(0xD426,0x03);
Write_Index_Data(0xD427,0x11);
Write_Index_Data(0xD428,0x03);
Write_Index_Data(0xD429,0x2B);
Write_Index_Data(0xD42A,0x03);
Write_Index_Data(0xD42B,0x48);
Write_Index_Data(0xD42C,0x03);
Write_Index_Data(0xD42D,0x59);
Write_Index_Data(0xD42E,0x03);
Write_Index_Data(0xD42F,0x65);
Write_Index_Data(0xD430,0x03);
Write_Index_Data(0xD431,0x71);
Write_Index_Data(0xD432,0x03);
Write_Index_Data(0xD433,0xFF);
Write_Index_Data(0xD500,0x00);
Write_Index_Data(0xD501,0x00);
Write_Index_Data(0xD502,0x00);
Write_Index_Data(0xD503,0x11);
Write_Index_Data(0xD504,0x00);
Write_Index_Data(0xD505,0x2E);
Write_Index_Data(0xD506,0x00);
Write_Index_Data(0xD507,0x47);
Write_Index_Data(0xD508,0x00);
Write_Index_Data(0xD509,0x5C);
Write_Index_Data(0xD50A,0x00);
Write_Index_Data(0xD50B,0x7E);
Write_Index_Data(0xD50C,0x00);
Write_Index_Data(0xD50D,0x9C);
Write_Index_Data(0xD50E,0x00);
Write_Index_Data(0xD50F,0xCD);
Write_Index_Data(0xD510,0x00);
Write_Index_Data(0xD511,0xF6);
Write_Index_Data(0xD512,0x01);
Write_Index_Data(0xD513,0x36);
Write_Index_Data(0xD514,0x01);
Write_Index_Data(0xD515,0x6A);
Write_Index_Data(0xD516,0x01);
Write_Index_Data(0xD517,0xBD);
Write_Index_Data(0xD518,0x01);
Write_Index_Data(0xD519,0xFF);
Write_Index_Data(0xD51A,0x02);
Write_Index_Data(0xD51B,0x01);
Write_Index_Data(0xD51C,0x02);
Write_Index_Data(0xD51D,0x3B);
Write_Index_Data(0xD51E,0x02);
Write_Index_Data(0xD51F,0x75);
Write_Index_Data(0xD520,0x02);
Write_Index_Data(0xD521,0x99);
Write_Index_Data(0xD522,0x02);
Write_Index_Data(0xD523,0xC8);
Write_Index_Data(0xD524,0x02);
Write_Index_Data(0xD525,0xE8);
Write_Index_Data(0xD526,0x03);
Write_Index_Data(0xD527,0x11);
Write_Index_Data(0xD528,0x03);
Write_Index_Data(0xD529,0x2B);
Write_Index_Data(0xD52A,0x03);
Write_Index_Data(0xD52B,0x48);
Write_Index_Data(0xD52C,0x03);
Write_Index_Data(0xD52D,0x59);
Write_Index_Data(0xD52E,0x03);
Write_Index_Data(0xD52F,0x65);
Write_Index_Data(0xD530,0x03);
Write_Index_Data(0xD531,0x71);
Write_Index_Data(0xD532,0x03);
Write_Index_Data(0xD533,0xFF);
Write_Index_Data(0xD600,0x00);
Write_Index_Data(0xD601,0x00);
Write_Index_Data(0xD602,0x00);
Write_Index_Data(0xD603,0x11);
Write_Index_Data(0xD604,0x00);
Write_Index_Data(0xD605,0x2E);
Write_Index_Data(0xD606,0x00);
Write_Index_Data(0xD607,0x47);
Write_Index_Data(0xD608,0x00);
Write_Index_Data(0xD609,0x5C);
Write_Index_Data(0xD60A,0x00);
Write_Index_Data(0xD60B,0x7E);
Write_Index_Data(0xD60C,0x00);
Write_Index_Data(0xD60D,0x9C);
Write_Index_Data(0xD60E,0x00);
Write_Index_Data(0xD60F,0xCD);
Write_Index_Data(0xD610,0x00);
Write_Index_Data(0xD611,0xF6);
Write_Index_Data(0xD612,0x01);
Write_Index_Data(0xD613,0x36);
Write_Index_Data(0xD614,0x01);
Write_Index_Data(0xD615,0x6A);
Write_Index_Data(0xD616,0x01);
Write_Index_Data(0xD617,0xBD);
Write_Index_Data(0xD618,0x01);
Write_Index_Data(0xD619,0xFF);
Write_Index_Data(0xD61A,0x02);
Write_Index_Data(0xD61B,0x01);
Write_Index_Data(0xD61C,0x02);
Write_Index_Data(0xD61D,0x3B);
Write_Index_Data(0xD61E,0x02);
Write_Index_Data(0xD61F,0x75);
Write_Index_Data(0xD620,0x02);
Write_Index_Data(0xD621,0x99);
Write_Index_Data(0xD622,0x02);
Write_Index_Data(0xD623,0xC8);
Write_Index_Data(0xD624,0x02);
Write_Index_Data(0xD625,0xE8);
Write_Index_Data(0xD626,0x03);
Write_Index_Data(0xD627,0x11);
Write_Index_Data(0xD628,0x03);
Write_Index_Data(0xD629,0x2B);
Write_Index_Data(0xD62A,0x03);
Write_Index_Data(0xD62B,0x48);
Write_Index_Data(0xD62C,0x03);
Write_Index_Data(0xD62D,0x59);
Write_Index_Data(0xD62E,0x03);
Write_Index_Data(0xD62F,0x65);
Write_Index_Data(0xD630,0x03);
Write_Index_Data(0xD631,0x71);
Write_Index_Data(0xD632,0x03);
Write_Index_Data(0xD633,0xFF);
msleep(200);
Write_Index_Data(0xF000,0x55);
Write_Index_Data(0xF001,0xAA);
Write_Index_Data(0xF002,0x52);
Write_Index_Data(0xF003,0x08);
Write_Index_Data(0xF004,0x03);
Write_Index_Data(0xB000,0x03);
Write_Index_Data(0xB001,0x15);
Write_Index_Data(0xB002,0xFA);
Write_Index_Data(0xB003,0x00);
Write_Index_Data(0xB004,0x00);
Write_Index_Data(0xB005,0x00);
Write_Index_Data(0xB006,0x00);
Write_Index_Data(0xB200,0xFB);
Write_Index_Data(0xB201,0xFC);
Write_Index_Data(0xB202,0xFD);
Write_Index_Data(0xB203,0xFE);
Write_Index_Data(0xB204,0xF0);
Write_Index_Data(0xB205,0x10);
Write_Index_Data(0xB206,0x00);
Write_Index_Data(0xB207,0x83);
Write_Index_Data(0xB208,0x04);
Write_Index_Data(0xB300,0x5B);
Write_Index_Data(0xB301,0x00);
Write_Index_Data(0xB302,0xFB);
Write_Index_Data(0xB303,0x21);
Write_Index_Data(0xB304,0x23);
Write_Index_Data(0xB305,0x0C);
Write_Index_Data(0xB400,0x00);
Write_Index_Data(0xB401,0x00);
Write_Index_Data(0xB402,0x00);
Write_Index_Data(0xB403,0x00);
Write_Index_Data(0xB404,0x00);
Write_Index_Data(0xB405,0x00);
Write_Index_Data(0xB406,0x00);
Write_Index_Data(0xB407,0x00);
Write_Index_Data(0xB408,0x00);
Write_Index_Data(0xB500,0x00);
Write_Index_Data(0xB501,0x00);
Write_Index_Data(0xB502,0x00);
Write_Index_Data(0xB503,0x00);
Write_Index_Data(0xB504,0x00);
Write_Index_Data(0xB505,0x00);
Write_Index_Data(0xB506,0x00);
Write_Index_Data(0xB507,0x00);
Write_Index_Data(0xB508,0x00);
Write_Index_Data(0xB509,0x00);
Write_Index_Data(0xB50A,0x44);
Write_Index_Data(0xB600,0x00);
Write_Index_Data(0xB601,0x00);
Write_Index_Data(0xB602,0x00);
Write_Index_Data(0xB603,0x00);
Write_Index_Data(0xB604,0x00);
Write_Index_Data(0xB605,0x00);
Write_Index_Data(0xB606,0x00);
Write_Index_Data(0xB700,0x00);
Write_Index_Data(0xB701,0x00);
Write_Index_Data(0xB702,0x20);
Write_Index_Data(0xB703,0x20);
Write_Index_Data(0xB704,0x20);
Write_Index_Data(0xB705,0x20);
Write_Index_Data(0xB706,0x00);
Write_Index_Data(0xB707,0x00);
Write_Index_Data(0xB800,0x00);
Write_Index_Data(0xB801,0x00);
Write_Index_Data(0xB802,0x00);
Write_Index_Data(0xB900,0x82);
Write_Index_Data(0xBA00,0x45);
Write_Index_Data(0xBA01,0x8A);
Write_Index_Data(0xBA02,0x04);
Write_Index_Data(0xBA03,0x5F);
Write_Index_Data(0xBA04,0xFF);
Write_Index_Data(0xBA05,0xFF);
Write_Index_Data(0xBA06,0xFF);
Write_Index_Data(0xBA07,0xFF);
Write_Index_Data(0xBA08,0xFF);
Write_Index_Data(0xBA09,0xFF);
Write_Index_Data(0xBA0A,0xFF);
Write_Index_Data(0xBA0B,0xFF);
Write_Index_Data(0xBA0C,0xF5);
Write_Index_Data(0xBA0D,0x41);
Write_Index_Data(0xBA0E,0xB9);
Write_Index_Data(0xBA0F,0x54);
Write_Index_Data(0xBB00,0x54);
Write_Index_Data(0xBB01,0xB9);
Write_Index_Data(0xBB02,0x15);
Write_Index_Data(0xBB03,0x4F);
Write_Index_Data(0xBB04,0xFF);
Write_Index_Data(0xBB05,0xFF);
Write_Index_Data(0xBB06,0xFF);
Write_Index_Data(0xBB07,0xFF);
Write_Index_Data(0xBB08,0xFF);
Write_Index_Data(0xBB09,0xFF);
Write_Index_Data(0xBB0A,0xFF);
Write_Index_Data(0xBB0B,0xFF);
Write_Index_Data(0xBB0C,0xF4);
Write_Index_Data(0xBB0D,0x50);
Write_Index_Data(0xBB0E,0x8A);
Write_Index_Data(0xBB0F,0x45);
Write_Index_Data(0xBC00,0xC7);
Write_Index_Data(0xBC01,0xFF);
Write_Index_Data(0xBC02,0xFF);
Write_Index_Data(0xBC03,0xE3);
Write_Index_Data(0xBD00,0xC7);
Write_Index_Data(0xBD01,0xFF);
Write_Index_Data(0xBD02,0xFF);
Write_Index_Data(0xBD03,0xE3);
Write_Index_Data(0xC000,0x00);
Write_Index_Data(0xC001,0x01);
Write_Index_Data(0xC002,0xFA);
Write_Index_Data(0xC003,0x00);
msleep(100);
Write_Index_Data(0xF000,0x55);
Write_Index_Data(0xF001,0xAA);
Write_Index_Data(0xF002,0x52);
Write_Index_Data(0xF003,0x08);
Write_Index_Data(0xF004,0x00);
Write_Index_Data(0xB000,0x00);
Write_Index_Data(0xB001,0x10);
Write_Index_Data(0xB100,0xFC);
msleep(100);
Write_Index_Data(0xB400,0x10);
Write_Index_Data(0xBA00,0x01);
Write_Index_Data(0xBC00,0x02);
Write_Index_Data(0xBC01,0x02);
Write_Index_Data(0xBC02,0x02);
Write_Index_Data(0x3500,0x00);
Write_Index_Data(0x3A00,0x66);
//Sleep out
WriteCommand(0X1100); 
msleep(120);
//Display on
WriteCommand(0X2900); 


}

static void ILI9806_core_init(void)
{
Write_LCD_CMD(0xFF);
Write_LCD_DATA(0xFF);
Write_LCD_DATA(0x98);
Write_LCD_DATA(0x06);
Write_LCD_DATA(0x04);
Write_LCD_DATA(0x01);

Write_LCD_CMD(0x08);
Write_LCD_DATA(0x10); 

Write_LCD_CMD(0x21);
Write_LCD_DATA(0x01); 

Write_LCD_CMD(0x30);
Write_LCD_DATA(0x02);
               
Write_LCD_CMD(0x31);
Write_LCD_DATA(0x00); 

Write_LCD_CMD(0x40);
Write_LCD_DATA(0x16);
             
Write_LCD_CMD(0x41);
Write_LCD_DATA(0x33);   // DVDDH DVDDL clamp    
          
Write_LCD_CMD(0x42);
Write_LCD_DATA(0x02); 
                
Write_LCD_CMD(0x43);
Write_LCD_DATA(0x09);   // VGH/VGL 
                
Write_LCD_CMD(0x44);
Write_LCD_DATA(0x09);   // VGH/VGL             


Write_LCD_CMD(0x50);
Write_LCD_DATA(0x78);   // VGMP  
              
Write_LCD_CMD(0x51);
Write_LCD_DATA(0x78);   // VGMN
           
Write_LCD_CMD(0x52);
Write_LCD_DATA(0x00);   //Flicker 
                  
Write_LCD_CMD(0x53);
Write_LCD_DATA(0x5e);   //Flicker


Write_LCD_CMD(0x60);
Write_LCD_DATA(0x07); 
                
Write_LCD_CMD(0x61);
Write_LCD_DATA(0x00);
               
Write_LCD_CMD(0x62);
Write_LCD_DATA(0x08);
                
Write_LCD_CMD(0x63);
Write_LCD_DATA(0x00);

Write_LCD_CMD(0xFF);
Write_LCD_DATA(0xFF);
Write_LCD_DATA(0x98);
Write_LCD_DATA(0x06);
Write_LCD_DATA(0x04);
Write_LCD_DATA(0x01);

Write_LCD_CMD(0xA0);Write_LCD_DATA(0x00);  // Gamma 0 
Write_LCD_CMD(0xA1);Write_LCD_DATA(0x1B);  // Gamma 4 
Write_LCD_CMD(0xA2);Write_LCD_DATA(0x24);  // Gamma 8
Write_LCD_CMD(0xA3);Write_LCD_DATA(0x11);  // Gamma 16
Write_LCD_CMD(0xA4);Write_LCD_DATA(0x07);  // Gamma 24
Write_LCD_CMD(0xA5);Write_LCD_DATA(0x0C);  // Gamma 52
Write_LCD_CMD(0xA6);Write_LCD_DATA(0x08);  // Gamma 80
Write_LCD_CMD(0xA7);Write_LCD_DATA(0x05);  // Gamma 108
Write_LCD_CMD(0xA8);Write_LCD_DATA(0x06);  // Gamma 147
Write_LCD_CMD(0xA9);Write_LCD_DATA(0x0b);  // Gamma 175
Write_LCD_CMD(0xAA);Write_LCD_DATA(0x0E);  // Gamma 203
Write_LCD_CMD(0xAB);Write_LCD_DATA(0x07);  // Gamma 231
Write_LCD_CMD(0xAC);Write_LCD_DATA(0x0e);  // Gamma 239
Write_LCD_CMD(0xAD);Write_LCD_DATA(0x12);  // Gamma 247
Write_LCD_CMD(0xAE);Write_LCD_DATA(0x0C);  // Gamma 251
Write_LCD_CMD(0xAF);Write_LCD_DATA(0x00);  // Gamma 255

/* ==================================== */
/* Nagitive */

Write_LCD_CMD(0xC0);Write_LCD_DATA(0x00);  // Gamma 0
Write_LCD_CMD(0xC1);Write_LCD_DATA(0x1c);  // Gamma 4
Write_LCD_CMD(0xC2);Write_LCD_DATA(0x24);  // Gamma 8
Write_LCD_CMD(0xC3);Write_LCD_DATA(0x11);  // Gamma 16
Write_LCD_CMD(0xC4);Write_LCD_DATA(0x07);  // Gamma 24
Write_LCD_CMD(0xC5);Write_LCD_DATA(0x0C); // Gamma 52
Write_LCD_CMD(0xC6);Write_LCD_DATA(0x08);  // Gamma 80
Write_LCD_CMD(0xC7);Write_LCD_DATA(0x06);  // Gamma 108
Write_LCD_CMD(0xC8);Write_LCD_DATA(0x07);  // Gamma 147
Write_LCD_CMD(0xC9);Write_LCD_DATA(0x0A);  // Gamma 175
Write_LCD_CMD(0xCA);Write_LCD_DATA(0x0E);  // Gamma 203
Write_LCD_CMD(0xCB);Write_LCD_DATA(0x07);  // Gamma 231
Write_LCD_CMD(0xCC);Write_LCD_DATA(0x0d);  // Gamma 239
Write_LCD_CMD(0xCD);Write_LCD_DATA(0x11);  // Gamma 247
Write_LCD_CMD(0xCE);Write_LCD_DATA(0x0C);  // Gamma 251
Write_LCD_CMD(0xCF);Write_LCD_DATA(0x00);  // Gamma 255

//=====================================
//Page 6 Command 

Write_LCD_CMD(0xFF);
Write_LCD_DATA(0xFF);
Write_LCD_DATA(0x98);
Write_LCD_DATA(0x06);
Write_LCD_DATA(0x04);
Write_LCD_DATA(0x06);// Change to Page 6
Write_LCD_CMD(0x00);Write_LCD_DATA(0x20);
Write_LCD_CMD(0x01);Write_LCD_DATA(0x04);
Write_LCD_CMD(0x02);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x03);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x04);Write_LCD_DATA(0x16);
Write_LCD_CMD(0x05);Write_LCD_DATA(0x16);
Write_LCD_CMD(0x06);Write_LCD_DATA(0x88);
Write_LCD_CMD(0x07);Write_LCD_DATA(0x02);
Write_LCD_CMD(0x08);Write_LCD_DATA(0x02);
Write_LCD_CMD(0x09);Write_LCD_DATA(0x00); 
Write_LCD_CMD(0x0A);Write_LCD_DATA(0x00); 
Write_LCD_CMD(0x0B);Write_LCD_DATA(0x00);    
Write_LCD_CMD(0x0C);Write_LCD_DATA(0x16);
Write_LCD_CMD(0x0D);Write_LCD_DATA(0x16);
Write_LCD_CMD(0x0E);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x0F);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x10);Write_LCD_DATA(0x50);
Write_LCD_CMD(0x11);Write_LCD_DATA(0x52);
Write_LCD_CMD(0x12);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x13);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x14);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x15);Write_LCD_DATA(0x43);
Write_LCD_CMD(0x16);Write_LCD_DATA(0x0b);
Write_LCD_CMD(0x17);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x18);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x19);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x1A);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x1B);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x1C);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x1D);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x20);Write_LCD_DATA(0x01);
Write_LCD_CMD(0x21);Write_LCD_DATA(0x23);
Write_LCD_CMD(0x22);Write_LCD_DATA(0x45);
Write_LCD_CMD(0x23);Write_LCD_DATA(0x67);
Write_LCD_CMD(0x24);Write_LCD_DATA(0x01);
Write_LCD_CMD(0x25);Write_LCD_DATA(0x23);
Write_LCD_CMD(0x26);Write_LCD_DATA(0x45);
Write_LCD_CMD(0x27);Write_LCD_DATA(0x67);
Write_LCD_CMD(0x30);Write_LCD_DATA(0x13);
Write_LCD_CMD(0x31);Write_LCD_DATA(0x11);
Write_LCD_CMD(0x32);Write_LCD_DATA(0x00);
Write_LCD_CMD(0x33);Write_LCD_DATA(0x22);
Write_LCD_CMD(0x34);Write_LCD_DATA(0x22);
Write_LCD_CMD(0x35);Write_LCD_DATA(0x22);
Write_LCD_CMD(0x36);Write_LCD_DATA(0x22);
Write_LCD_CMD(0x37);Write_LCD_DATA(0xAa);
Write_LCD_CMD(0x38);Write_LCD_DATA(0xBb);
Write_LCD_CMD(0x39);Write_LCD_DATA(0x66);
Write_LCD_CMD(0x3A);Write_LCD_DATA(0x22);
Write_LCD_CMD(0x3B);Write_LCD_DATA(0x22);
Write_LCD_CMD(0x3C);Write_LCD_DATA(0x22);
Write_LCD_CMD(0x3D);Write_LCD_DATA(0x22);
Write_LCD_CMD(0x3E);Write_LCD_DATA(0x22);
Write_LCD_CMD(0x3F);Write_LCD_DATA(0x22);
Write_LCD_CMD(0x40);Write_LCD_DATA(0x22);

Write_LCD_CMD(0xFF);
Write_LCD_DATA(0xFF);
Write_LCD_DATA(0x98);
Write_LCD_DATA(0x06);
Write_LCD_DATA(0x04);
Write_LCD_DATA(0x07);

Write_LCD_CMD(0x17);
Write_LCD_DATA(0x12); // 
             
Write_LCD_CMD(0x02);
Write_LCD_DATA(0x77); 


Write_LCD_CMD(0xFF);
Write_LCD_DATA(0xFF);
Write_LCD_DATA(0x98);
Write_LCD_DATA(0x06);
Write_LCD_DATA(0x04);
Write_LCD_DATA(0x00);

Write_LCD_CMD(0x3A);
Write_LCD_DATA(0x66);

Write_LCD_CMD(0x36);
Write_LCD_DATA(0x00); 
            
Write_LCD_CMD(0x11);
/* Write_LCD_DATA(0x00); */

msleep(120);

Write_LCD_CMD(0x29);
/* Write_LCD_DATA(0x00);  */

msleep(10);
}
static void Read_LG_HX8397_ID(void)
{
        int i;
        unsigned char tmp;
        int reg = 0x04;

	CS_CLR();
	TXD_CLR();	//0
	udelay(UDELAY_TIME);
	  	
	CLK_CLR();
	udelay(3);//
        
	CLK_SET();        
	udelay(UDELAY_TIME);

	TXD_CLR();
	CLK_CLR();
		
	for(i=0;i<8;i++)
	{
		if(reg&0x80)
			TXD_SET();	
		else
			TXD_CLR();
		reg<<=1;	
			
		CLK_CLR();     
		udelay(UDELAY_TIME);
		CLK_SET();
		udelay(UDELAY_TIME);	
	}

	TXD_CLR();
	TXD_IN();
	for(i=0;i<8;i++)
	{
                CLK_CLR();
		delay_us(4);

                tmp = TXD_GET();
                reg_data[0] += (tmp<<(7-i));
                CLK_SET();
		delay_us(4);
                //printk("%s: tmp == %d, data_l == %d\n", __func__, tmp, reg_data[0]);
	}
	CLK_CLR();
	TXD_CLR();

	for(i=0;i<8;i++)
	{
                CLK_CLR();
		delay_us(4);

                tmp = TXD_GET();
                reg_data[1] += (tmp<<(7-i));
                CLK_SET();
		delay_us(4);
                //printk("%s: tmp == %d, data_l == %d\n", __func__, tmp, reg_data[1]);
	}
	CLK_CLR();
	TXD_CLR();

	for(i=0;i<8;i++)
	{
                CLK_CLR();
		delay_us(4);

                tmp = TXD_GET();
                reg_data[2] += (tmp<<(7-i));
                CLK_SET();
		delay_us(4);
                //printk("%s: tmp == %d, data_l == %d\n", __func__, tmp, reg_data[2]);
	}
	CLK_CLR();
	TXD_CLR();

	delay_us(4);
	CS_SET();
}

static void read_LG_HX8397_ID(void)
{
    int err;
    int det_pu,det_pd;
        err=gpio_request(TFT_DET,"TFT_DET");
        if (err) {
            printk( "failed to request TFT_DET GPIO%d\n", TFT_DET);
        }

        gpio_direction_input(TFT_DET);
        gpio_pull_updown(TFT_DET,GPIOPullDown);
        msleep(100);
        det_pd=gpio_get_value(TFT_DET);

        gpio_pull_updown(TFT_DET,GPIOPullUp);
        msleep(100);
        det_pu=gpio_get_value(TFT_DET);

        printk("TFT_DET io pull up dwon value : %d  %d\n",det_pu,det_pd);
        Read_LG_HX8397_ID();

        if (reg_data[0] == 131 && reg_data[1] == 121 && reg_data[2] == 12) {
            printk("%s: current is LG HX8397 screen !\n", __func__);
            isLGScreen = true;
        }else if(det_pu && det_pd ){
            isTMILI9806=true;

        } else {
            printk("%s: current is not LG HX8397 screen !\n", __func__);
            isBOEScreen= true;
        }
}
static void init_BOEILI9806E_IO(void)
{
    LCD_RST_OUT();
    LCD_RST(1);
    mdelay(30);
    LCD_RST(0);
    mdelay(100);
    LCD_RST(1);
    mdelay(100);
    /* gpio_free(LCD_RESET_PORT); */

    boe_ili9806e_init();
}

static void init_TMILL9806_IO(void)
{
    LCD_RST_OUT();
    LCD_RST(1);
    mdelay(30);
    LCD_RST(0);
    mdelay(100);
    LCD_RST(1);
    mdelay(100);
    /* gpio_free(LCD_RESET_PORT); */

    ILI9806_core_init();
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

static DEFINE_MUTEX(lcd_mutex);
extern void rk29_lcd_spim_spin_lock(void);
extern void rk29_lcd_spim_spin_unlock(void);

static void lcd_resume(struct work_struct *work)
{
    mutex_lock(&lcd_mutex);
    rk29_lcd_spim_spin_lock();
    if(gLcd_info)
        gLcd_info->io_init();

    gpio_request(LCD_RESET_PORT, NULL);
    if (isLGScreen) {
        printk("%s: current screen is LG HX8397 \n", __func__);
        init_HX8397_IO();
    }else if(isTMILI9806){

        printk("%s: current screen is TM ILI9806\n", __func__);
        init_TMILL9806_IO();
    }else if(isBOEScreen){

        printk("%s: current screen is BOE YS397B009\n", __func__);
        gpio_direction_output(LCD_RESET_PORT, 0);
        mdelay(2);
        gpio_set_value(LCD_RESET_PORT, 1);
        mdelay(10);
        gpio_free(LCD_RESET_PORT);

        init_YS397B009();
    }else if(isBOEScreenILI9806E){

        printk("%s: current screen is BOE_SCREEN_ILI9806E \n", __func__);
        init_BOEILI9806E_IO();


    }else
        printk("%s: current screen is unknow\n", __func__);

    printk(KERN_DEBUG "%s\n",__FUNCTION__);

    if(gLcd_info)
        gLcd_info->io_deinit();

    rk29_lcd_spim_spin_unlock();
    mutex_unlock(&lcd_mutex);
}

static DECLARE_WORK(lcd_resume_work, lcd_resume);
static struct workqueue_struct *lcd_resume_wq;

static void lcd_late_resume(struct early_suspend *h)
{
	queue_work(lcd_resume_wq, &lcd_resume_work);
}

static struct early_suspend lcd_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1, // before fb resume
	.resume = lcd_late_resume,
};

int boe_lcd_init(void)
{ 
    volatile u32 data;
    printk("BOE lcd init...\n");
    if(gLcd_info)
        gLcd_info->io_init();

    gpio_request(LCD_RESET_PORT, NULL);
    gpio_request(TXD_PORT, NULL);
    gpio_request(CLK_PORT, NULL);
    gpio_request(CS_PORT , NULL);

    TXD_OUT();
    CLK_OUT();
    CS_OUT();
    CS_SET();
    TXD_SET();
    CLK_SET();

    printk("%s: current screen is BOE YS397B009\n", __func__);
    gpio_direction_output(LCD_RESET_PORT, 0);
    mdelay(2);
    gpio_set_value(LCD_RESET_PORT, 1);
    mdelay(10);
    gpio_free(LCD_RESET_PORT);
    init_YS397B009();

    if(gLcd_info)
        gLcd_info->io_deinit();

    lcd_resume_wq = create_singlethread_workqueue("lcd");
    register_early_suspend(&lcd_early_suspend_desc);

    return 0;
}

int boe_lcd_standby(u8 enable)	//***enable =1 means suspend, 0 means resume 
{
    if (enable) {
        mutex_lock(&lcd_mutex);
        rk29_lcd_spim_spin_lock();
        if(gLcd_info)
            gLcd_info->io_init();

        gpio_request(LCD_RESET_PORT, NULL);
        gpio_direction_output(LCD_RESET_PORT, 0);
        mdelay(2);
        gpio_set_value(LCD_RESET_PORT, 1);
        mdelay(10);
        gpio_free(LCD_RESET_PORT);

        WriteCommand(0X2800); 
        WriteCommand(0X1100); 
        msleep(5);
        WriteCommand(0X4f00); 
        WriteParameter(0x01);

        if(gLcd_info)
            gLcd_info->io_deinit();

        rk29_lcd_spim_spin_unlock();
        mutex_unlock(&lcd_mutex);

        //disable lcd power
        // x5 user vcc_io 
    } else {
        //open lcd power
        flush_workqueue(lcd_resume_wq);

    }

    return 0;
}

int boe_lcd_ili9806e_init(void)
{ 
    volatile u32 data;
    printk("BOE lcd ILI9806E init...\n");
    if(gLcd_info)
        gLcd_info->io_init();

    gpio_request(LCD_RESET_PORT, NULL);
    gpio_request(TXD_PORT, NULL);
    gpio_request(CLK_PORT, NULL);
    gpio_request(CS_PORT , NULL);

    TXD_OUT();
    CLK_OUT();
    CS_OUT();
    CS_SET();
    TXD_SET();
    CLK_SET();

    printk("%s: current screen is BOE ILI9806E\n", __func__);
    gpio_direction_output(LCD_RESET_PORT, 0);
    mdelay(2);
    gpio_set_value(LCD_RESET_PORT, 1);
    mdelay(10);
    gpio_free(LCD_RESET_PORT);

    boe_ili9806e_init();

    if(gLcd_info)
        gLcd_info->io_deinit();

    lcd_resume_wq = create_singlethread_workqueue("lcd");
    register_early_suspend(&lcd_early_suspend_desc);

    return 0;
}

int boe_lcd_ili9806e_standby(u8 enable)	//***enable =1 means suspend, 0 means resume 
{
    if (enable) {
        mutex_lock(&lcd_mutex);
        rk29_lcd_spim_spin_lock();
        if(gLcd_info)
            gLcd_info->io_init();

        gpio_request(LCD_RESET_PORT, NULL);
        gpio_direction_output(LCD_RESET_PORT, 0);
        mdelay(2);
        gpio_set_value(LCD_RESET_PORT, 1);
        mdelay(10);
        gpio_free(LCD_RESET_PORT);

        WriteCommand(0X2800); 
        WriteCommand(0X1100); 
        msleep(5);
        WriteCommand(0X4f00); 
        WriteParameter(0x01);

        if(gLcd_info)
            gLcd_info->io_deinit();

        rk29_lcd_spim_spin_unlock();
        mutex_unlock(&lcd_mutex);

        //disable lcd power
        // x5 user vcc_io 
    } else {
        //open lcd power
        flush_workqueue(lcd_resume_wq);

    }

    return 0;
}

#define wr_com8080(cmd) Write_LCD_CMD(cmd)
#define write_data(data) Write_LCD_DATA(data) 
int boe_ili9806e_init()
{
//****************************************************************************//
//****************************** Page 1 Command ******************************//
//****************************************************************************//
Write_LCD_CMD(0xFF);Write_LCD_DATA(0xFF);Write_LCD_DATA(0x98);Write_LCD_DATA(0x06);Write_LCD_DATA(0x04);Write_LCD_DATA(0x01);	  // Change to Page 1
//msleep(10);
Write_LCD_CMD(0x08);Write_LCD_DATA(0x10);				  // output SDA
//msleep(10);
Write_LCD_CMD(0x21);Write_LCD_DATA(0x01);				  // DE = 1 Active
//msleep(10);
Write_LCD_CMD(0x30);Write_LCD_DATA(0x02);				  // 480 X 800
//msleep(10);
Write_LCD_CMD(0x31);Write_LCD_DATA(0x00);				  // 2-dot Inversion
//msleep(10);

Write_LCD_CMD(0x40);Write_LCD_DATA(0x16);				 // BT	+2.5/-3 pump for DDVDH-L
//msleep(10);
Write_LCD_CMD(0x41);Write_LCD_DATA(0x33);				  // DVDDH DVDDL clamp	
//msleep(10);
Write_LCD_CMD(0x42);Write_LCD_DATA(0x11);				 // VGH/VGL 
//msleep(10);
Write_LCD_CMD(0x43);Write_LCD_DATA(0x85);				 //  
//msleep(10);
Write_LCD_CMD(0x44);Write_LCD_DATA(0x8B);				  // VGH/VGL 
//msleep(10);
Write_LCD_CMD(0x45);Write_LCD_DATA(0x1B);				 // VGH/VGL=1B 
//msleep(10);
Write_LCD_CMD(0x50);Write_LCD_DATA(0x78);				  // VGMP  78 
//msleep(10);
Write_LCD_CMD(0x51);Write_LCD_DATA(0x78);				  // VGMN  78
//msleep(10);
Write_LCD_CMD(0x52);Write_LCD_DATA(0x00);					//Flicker
//msleep(10);
Write_LCD_CMD(0x53);Write_LCD_DATA(0x32);	//42				//Flicker  
//msleep(10);
Write_LCD_CMD(0x57);Write_LCD_DATA(0x60); //50					//Flicker  
//msleep(10);
Write_LCD_CMD(0x60);Write_LCD_DATA(0x07);				  // SDTI
//msleep(10);
Write_LCD_CMD(0x61);Write_LCD_DATA(0x00);				// CRTI
//msleep(10);
Write_LCD_CMD(0x62);Write_LCD_DATA(0x07);				 // EQTI
//msleep(10);
Write_LCD_CMD(0x63);Write_LCD_DATA(0x00);			   // PCTI
//msleep(10);


//++++++++++++++++++ Gamma Setting ++++++++++++++++++//

Write_LCD_CMD(0xA0);Write_LCD_DATA(0x00);// Gamma 255  
//msleep(10);
Write_LCD_CMD(0xA1);Write_LCD_DATA(0x0b);// Gamma 251  
//msleep(10);
Write_LCD_CMD(0xA2);Write_LCD_DATA(0x12);// Gamma 247 
//msleep(10);
Write_LCD_CMD(0xA3);Write_LCD_DATA(0x0c);// Gamma 239
//msleep(10);
Write_LCD_CMD(0xA4);Write_LCD_DATA(0x05);// Gamma 231  
//msleep(10);
Write_LCD_CMD(0xA5);Write_LCD_DATA(0x0c);// Gamma 203	
//msleep(10);
Write_LCD_CMD(0xA6);Write_LCD_DATA(0x07);// Gamma 175  
//msleep(10);
Write_LCD_CMD(0xA7);Write_LCD_DATA(0x16);// Gamma 147  
//msleep(10);
Write_LCD_CMD(0xA8);Write_LCD_DATA(0x06);// Gamma 108	
//msleep(10);
Write_LCD_CMD(0xA9);Write_LCD_DATA(0x0a);// Gamma 80 
//msleep(10);
Write_LCD_CMD(0xAA);Write_LCD_DATA(0x0F);// Gamma 52   
//msleep(10);
Write_LCD_CMD(0xAB);Write_LCD_DATA(0x06);// Gamma 24
//msleep(10);
Write_LCD_CMD(0xAC);Write_LCD_DATA(0x0e);// Gamma 16   
//msleep(10);
Write_LCD_CMD(0xAD);Write_LCD_DATA(0x1a);// Gamma 8   
//msleep(10);
Write_LCD_CMD(0xAE);Write_LCD_DATA(0x12);// Gamma 4   
//msleep(10);
Write_LCD_CMD(0xAF);Write_LCD_DATA(0x00);// Gamma 0
//msleep(10);
///==============Nagitive
Write_LCD_CMD(0xC0);Write_LCD_DATA(0x00);// Gamma 255  
//msleep(10);
Write_LCD_CMD(0xC1);Write_LCD_DATA(0x0b);// Gamma 251  
//msleep(10);
Write_LCD_CMD(0xC2);Write_LCD_DATA(0x12);// Gamma 247 
//msleep(10);
Write_LCD_CMD(0xC3);Write_LCD_DATA(0x0c);// Gamma 239
//msleep(10);
Write_LCD_CMD(0xC4);Write_LCD_DATA(0x05);// Gamma 231  
//msleep(10);
Write_LCD_CMD(0xC5);Write_LCD_DATA(0x0c);// Gamma 203	
//msleep(10);
Write_LCD_CMD(0xC6);Write_LCD_DATA(0x07);// Gamma 175  
//msleep(10);
Write_LCD_CMD(0xC7);Write_LCD_DATA(0x16);// Gamma 147  
//msleep(10);
Write_LCD_CMD(0xC8);Write_LCD_DATA(0x06);// Gamma 108	
//msleep(10);
Write_LCD_CMD(0xC9);Write_LCD_DATA(0x0a);// Gamma 80 
//msleep(10);
Write_LCD_CMD(0xCA);Write_LCD_DATA(0x0F);// Gamma 52   
//msleep(10);
Write_LCD_CMD(0xCB);Write_LCD_DATA(0x06);// Gamma 24
//msleep(10);
Write_LCD_CMD(0xCC);Write_LCD_DATA(0x0e);// Gamma 16   
//msleep(10);
Write_LCD_CMD(0xCD);Write_LCD_DATA(0x1a);// Gamma 8   
//msleep(10);
Write_LCD_CMD(0xCE);Write_LCD_DATA(0x12);// Gamma 4   
//msleep(10);
Write_LCD_CMD(0xCF);Write_LCD_DATA(0x00);// Gamma 0
//msleep(10);

//****************************************************************************//
//****************************** Page 6 Command ******************************//
//****************************************************************************//
Write_LCD_CMD(0xFF);Write_LCD_DATA(0xFF);Write_LCD_DATA(0x98);Write_LCD_DATA(0x06);Write_LCD_DATA(0x04);Write_LCD_DATA(0x06);	  // Change to Page 6
//msleep(10);
Write_LCD_CMD(0x00);Write_LCD_DATA(0xA0);  //1
//msleep(10);
Write_LCD_CMD(0x01);Write_LCD_DATA(0x05);  //2
//msleep(10);
Write_LCD_CMD(0x02);Write_LCD_DATA(0x00);  //3	
//msleep(10);
Write_LCD_CMD(0x03);Write_LCD_DATA(0x00);  //4
//msleep(10);
Write_LCD_CMD(0x04);Write_LCD_DATA(0x01);  //5
//msleep(10);
Write_LCD_CMD(0x05);Write_LCD_DATA(0x01);  //6
//msleep(10);
Write_LCD_CMD(0x06);Write_LCD_DATA(0x88);  //7	
//msleep(10);
Write_LCD_CMD(0x07);Write_LCD_DATA(0x04);  //8
//msleep(10);
Write_LCD_CMD(0x08);Write_LCD_DATA(0x01);
//msleep(10);
Write_LCD_CMD(0x09);Write_LCD_DATA(0x90);	 
//msleep(10);
Write_LCD_CMD(0x0A);Write_LCD_DATA(0x04);	 
//msleep(10);
Write_LCD_CMD(0x0B);Write_LCD_DATA(0x01);	 
//msleep(10);
Write_LCD_CMD(0x0C);Write_LCD_DATA(0x01);
//msleep(10);
Write_LCD_CMD(0x0D);Write_LCD_DATA(0x01);
//msleep(10);
Write_LCD_CMD(0x0E);Write_LCD_DATA(0x00);
//msleep(10);
Write_LCD_CMD(0x0F);Write_LCD_DATA(0x00);
//msleep(10);
Write_LCD_CMD(0x10);Write_LCD_DATA(0x55);
//msleep(10);
Write_LCD_CMD(0x11);Write_LCD_DATA(0x50);
//msleep(10);
Write_LCD_CMD(0x12);Write_LCD_DATA(0x01);
//msleep(10);
Write_LCD_CMD(0x13);Write_LCD_DATA(0x85);
//msleep(10);
Write_LCD_CMD(0x14);Write_LCD_DATA(0x85);
//msleep(10);
Write_LCD_CMD(0x15);Write_LCD_DATA(0xC0);
//msleep(10);
Write_LCD_CMD(0x16);Write_LCD_DATA(0x0B);
//msleep(10);
Write_LCD_CMD(0x17);Write_LCD_DATA(0x00);
//msleep(10);
Write_LCD_CMD(0x18);Write_LCD_DATA(0x00);
//msleep(10);
Write_LCD_CMD(0x19);Write_LCD_DATA(0x00);
//msleep(10);
Write_LCD_CMD(0x1A);Write_LCD_DATA(0x00);
//msleep(10);
Write_LCD_CMD(0x1B);Write_LCD_DATA(0x00);
//msleep(10);
Write_LCD_CMD(0x1C);Write_LCD_DATA(0x00);
//msleep(10);
Write_LCD_CMD(0x1D);Write_LCD_DATA(0x00);
//msleep(10);

Write_LCD_CMD(0x20);Write_LCD_DATA(0x01);
//msleep(10);
Write_LCD_CMD(0x21);Write_LCD_DATA(0x23);
//msleep(10);
Write_LCD_CMD(0x22);Write_LCD_DATA(0x45);
//msleep(10);
Write_LCD_CMD(0x23);Write_LCD_DATA(0x67);
//msleep(10);
Write_LCD_CMD(0x24);Write_LCD_DATA(0x01);
//msleep(10);
Write_LCD_CMD(0x25);Write_LCD_DATA(0x23);
//msleep(10);
Write_LCD_CMD(0x26);Write_LCD_DATA(0x45);
//msleep(10);
Write_LCD_CMD(0x27);Write_LCD_DATA(0x67);
//msleep(10);

Write_LCD_CMD(0x30);Write_LCD_DATA(0x02);
//msleep(10);
Write_LCD_CMD(0x31);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x32);Write_LCD_DATA(0x11);
//msleep(10);
Write_LCD_CMD(0x33);Write_LCD_DATA(0xAA);
//msleep(10);
Write_LCD_CMD(0x34);Write_LCD_DATA(0xBB);
//msleep(10);
Write_LCD_CMD(0x35);Write_LCD_DATA(0x66);
//msleep(10);
Write_LCD_CMD(0x36);Write_LCD_DATA(0x00);
//msleep(10);
Write_LCD_CMD(0x37);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x38);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x39);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x3A);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x3B);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x3C);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x3D);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x3E);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x3F);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x40);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x52);Write_LCD_DATA(0x10);
//msleep(10);
Write_LCD_CMD(0x53);Write_LCD_DATA(0x10);
//msleep(10);

//****************************************************************************//
//****************************** Page 7 Command ******************************//
//****************************************************************************//

Write_LCD_CMD(0xFF);Write_LCD_DATA(0xFF);Write_LCD_DATA(0x98);Write_LCD_DATA(0x06);Write_LCD_DATA(0x04);Write_LCD_DATA(0x07);	  // Change to Page 7
//msleep(10);
Write_LCD_CMD(0x17);Write_LCD_DATA(0x22);
//msleep(10);
Write_LCD_CMD(0x02);Write_LCD_DATA(0x77);
//msleep(10);




//****************************************************************************//
//****************************** Page 0 Command ******************************//
//****************************************************************************//

Write_LCD_CMD(0xFF);Write_LCD_DATA(0xFF);Write_LCD_DATA(0x98);Write_LCD_DATA(0x06);Write_LCD_DATA(0x04);Write_LCD_DATA(0x00);	 // Change to Page 0
Write_LCD_CMD(0x36);Write_LCD_DATA(0x08);  //03


 Write_LCD_CMD(0x3a);Write_LCD_DATA(0x66);
//msleep(10);
Write_LCD_CMD(0x11);Write_LCD_DATA(0x00);				  // Sleep-Out
msleep(120);
Write_LCD_CMD(0x29);Write_LCD_DATA(0x00);				  // Display On
msleep(10);
}


int tm_lcd_init(void)
{ 
    volatile u32 data;
    printk("TM lcd init...\n");
    if(gLcd_info)
        gLcd_info->io_init();

    gpio_request(LCD_RESET_PORT, NULL);
    gpio_request(TXD_PORT, NULL);
    gpio_request(CLK_PORT, NULL);
    gpio_request(CS_PORT , NULL);

    TXD_OUT();
    CLK_OUT();
    CS_OUT();
    CS_SET();
    TXD_SET();
    CLK_SET();

    printk("%s: current screen is TM ILI9806\n", __func__);
    init_TMILL9806_IO();
    if(gLcd_info)
        gLcd_info->io_deinit();

    lcd_resume_wq = create_singlethread_workqueue("lcd");
    register_early_suspend(&lcd_early_suspend_desc);

    return 0;
}


int tm_lcd_standby(u8 enable)	//***enable =1 means suspend, 0 means resume 
{
    if (enable) {
        mutex_lock(&lcd_mutex);
        rk29_lcd_spim_spin_lock();
        if(gLcd_info)
            gLcd_info->io_init();

        gpio_request(LCD_RESET_PORT, NULL);
        gpio_direction_output(LCD_RESET_PORT, 0);
        mdelay(2);
        gpio_set_value(LCD_RESET_PORT, 1);
        mdelay(10);
        gpio_free(LCD_RESET_PORT);

        WriteCommand(0X2800); 
        WriteCommand(0X1100); 
        msleep(5);
        WriteCommand(0X4f00); 
        WriteParameter(0x01);

        if(gLcd_info)
            gLcd_info->io_deinit();

        rk29_lcd_spim_spin_unlock();
        mutex_unlock(&lcd_mutex);

        //disable lcd power
        // x5 user vcc_io 
    } else {
        //open lcd power

        flush_workqueue(lcd_resume_wq);

        if (isLGScreen) {
            lcd_LG_resume();
        }
    }

    return 0;
}


int lg_lcd_init(void)
{ 
    volatile u32 data;
    printk("LG lcd init...\n");
    if(gLcd_info)
        gLcd_info->io_init();

    gpio_request(LCD_RESET_PORT, NULL);
    gpio_request(TXD_PORT, NULL);
    gpio_request(CLK_PORT, NULL);
    gpio_request(CS_PORT , NULL);

    TXD_OUT();
    CLK_OUT();
    CS_OUT();
    CS_SET();
    TXD_SET();
    CLK_SET();

    printk("%s: current screen is LG HX8397 \n", __func__);
    init_HX8397_IO();

    if(gLcd_info)
        gLcd_info->io_deinit();

    lcd_resume_wq = create_singlethread_workqueue("lcd");
    register_early_suspend(&lcd_early_suspend_desc);

    return 0;
}


int lg_lcd_standby(u8 enable)	//***enable =1 means suspend, 0 means resume 
{
    if (enable) {
        mutex_lock(&lcd_mutex);
        rk29_lcd_spim_spin_lock();
        if(gLcd_info)
            gLcd_info->io_init();

        lcd_LG_suspend();
        if(gLcd_info)
            gLcd_info->io_deinit();

        rk29_lcd_spim_spin_unlock();
        mutex_unlock(&lcd_mutex);

        //disable lcd power
        // x5 user vcc_io 
    } else {
        //open lcd power
        flush_workqueue(lcd_resume_wq);

        if (isLGScreen) {
            lcd_LG_resume();
        }
    }

    return 0;
}


static int lcd_get_id(void)
{
    int id = -1;
    int ts_id = -1;
    int err;
    int det_pu,det_pd;

    volatile u32 data;
    printk("lcd init...\n");
    if(gLcd_info)
        gLcd_info->io_init();

    gpio_request(LCD_RESET_PORT, NULL);
    gpio_request(TXD_PORT, NULL);
    gpio_request(CLK_PORT, NULL);
    gpio_request(CS_PORT , NULL);
    err=gpio_request(TFT_DET,"TFT_DET");
    if (err) {
        printk( "failed to request TFT_DET GPIO%d\n", TFT_DET);
    }

    TXD_OUT();
    CLK_OUT();
    CS_OUT();
    CS_SET();
    TXD_SET();
    CLK_SET();



    gpio_direction_input(TFT_DET);
    gpio_pull_updown(TFT_DET,GPIOPullDown);
    msleep(100);
    det_pd=gpio_get_value(TFT_DET);

    gpio_pull_updown(TFT_DET,GPIOPullUp);
    msleep(100);
    det_pu=gpio_get_value(TFT_DET);

    printk("TFT_DET io pull up dwon value : %d  %d reg %x %x %x\n",det_pu,det_pd,reg_data[0],reg_data[1],reg_data[2]);
    Read_LG_HX8397_ID();

    // TM_SCREEN pullUp
    // BOE_SCREEN Z/Float
    // BOE_SCREEN_ILI9806E pullDown
    /* det_pu=0; */
    /* det_pd=0; */
    if (reg_data[0] == 131 && reg_data[1] == 121 && reg_data[2] == 12) {
        printk("%s: current is LG HX8397 screen !\n", __func__);
        isLGScreen = true;
        gLcd_type = LG_SCREEN;
    }else if(det_pu && det_pd ){
        isTMILI9806=true;
        gLcd_type = TM_SCREEN;

    }else if(det_pu && !det_pd ){
        printk("%s: current is BOE screen !\n", __func__);
        isBOEScreen= true;
        gLcd_type = BOE_SCREEN;

    }else if(!det_pu && !det_pd ){
        printk("%s: current is BOE screen ILI9806E!\n", __func__);
        isBOEScreenILI9806E= true;
        gLcd_type = BOE_SCREEN_ILI9806E;
    } else {
        printk("%s: current is BOE screen !\n", __func__);
        isBOEScreen= true;
        gLcd_type = BOE_SCREEN;
    }


    return gLcd_type;
}

#define RK_USE_SCREEN_ID

#if defined(RK_USE_SCREEN_ID)
void set_lcd_info_by_id(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info )
{
	int id;
    if(lcd_info)
        gLcd_info = lcd_info;

    if(screen)
        gLcd_screen = screen;


	id = lcd_get_id();

	switch(id)
	{
	case BOE_SCREEN:
			
        printk("lcd set BOE PARAM \n");
		/* screen type & face */
		screen->type = SCREEN_TYPE_BOE;
		screen->face = OUT_FACE_BOE;

		/* Screen size */
		screen->x_res = H_VD_BOE;
		screen->y_res = V_VD_BOE;

		screen->width = LCD_WIDTH_BOE;
		screen->height = LCD_HEIGHT_BOE;

		/* Timing */
		screen->lcdc_aclk = LCDC_ACLK_BOE;
		screen->pixclock = DCLK_BOE;
		screen->left_margin = H_BP_BOE;
		screen->right_margin = H_FP_BOE;
		screen->hsync_len = H_PW_BOE;
		screen->upper_margin = V_BP_BOE;
		screen->lower_margin = V_FP_BOE;
		screen->vsync_len = V_PW_BOE;

		/* Pin polarity */
		screen->pin_hsync = HSYNC_POL_BOE;
		screen->pin_vsync = VSYNC_POL_BOE;
		screen->pin_den = DEN_POL_BOE;
		screen->pin_dclk = DCLK_POL_BOE;

		/* Swap rule */
		screen->swap_rb = SWAP_RB_BOE;
		screen->swap_rg = SWAP_RG_BOE;
		screen->swap_gb = SWAP_GB_BOE;
		screen->swap_delta = 0;
		screen->swap_dumy = 0;

		/* Operation function*/
		screen->init = boe_lcd_init;
		screen->standby = boe_lcd_standby;

		break;

	case BOE_SCREEN_ILI9806E:
			
       printk("lcd set BOE_ILI9806E PARAM \n");
		/* screen type & face */
		screen->type = SCREEN_TYPE_BOE_ILI9806E;
		screen->face = OUT_FACE_BOE_ILI9806E;

		/* Screen size */
		screen->x_res = H_VD_BOE_ILI9806E;
		screen->y_res = V_VD_BOE_ILI9806E;

		screen->width = LCD_WIDTH_BOE_ILI9806E;
		screen->height = LCD_HEIGHT_BOE_ILI9806E;

		/* Timing */
		screen->lcdc_aclk = LCDC_ACLK_BOE_ILI9806E;
		screen->pixclock = DCLK_BOE_ILI9806E;
		screen->left_margin = H_BP_BOE_ILI9806E;
		screen->right_margin = H_FP_BOE_ILI9806E;
		screen->hsync_len = H_PW_BOE_ILI9806E;
		screen->upper_margin = V_BP_BOE_ILI9806E;
		screen->lower_margin = V_FP_BOE_ILI9806E;
		screen->vsync_len = V_PW_BOE_ILI9806E;

		/* Pin polarity */
		screen->pin_hsync = HSYNC_POL_BOE_ILI9806E;
		screen->pin_vsync = VSYNC_POL_BOE_ILI9806E;
		screen->pin_den = DEN_POL_BOE_ILI9806E;
		screen->pin_dclk = DCLK_POL_BOE_ILI9806E;

		/* Swap rule */
		screen->swap_rb = SWAP_RB_BOE_ILI9806E;
		screen->swap_rg = SWAP_RG_BOE_ILI9806E;
		screen->swap_gb = SWAP_GB_BOE_ILI9806E;
		screen->swap_delta = 0;
		screen->swap_dumy = 0;

		/* Operation function*/
		screen->init = boe_lcd_ili9806e_init;
		screen->standby = boe_lcd_ili9806e_standby;

		break;


	case LG_SCREEN:
			
        printk("lcd set LG PARAM \n");
		/* screen type & face */
		screen->type = SCREEN_TYPE_LG;
		screen->face = OUT_FACE_LG;

		/* Screen size */
		screen->x_res = H_VD_LG;
		screen->y_res = V_VD_LG;

		screen->width = LCD_WIDTH_LG;
		screen->height = LCD_HEIGHT_LG;

		/* Timing */
		screen->lcdc_aclk = LCDC_ACLK_LG;
		screen->pixclock = DCLK_LG;
		screen->left_margin = H_BP_LG;
		screen->right_margin = H_FP_LG;
		screen->hsync_len = H_PW_LG;
		screen->upper_margin = V_BP_LG;
		screen->lower_margin = V_FP_LG;
		screen->vsync_len = V_PW_LG;

		/* Pin polarity */
		screen->pin_hsync = HSYNC_POL_LG;
		screen->pin_vsync = VSYNC_POL_LG;
		screen->pin_den = DEN_POL_LG;
		screen->pin_dclk = DCLK_POL_LG;

		/* Swap rule */
		screen->swap_rb = SWAP_RB_LG;
		screen->swap_rg = SWAP_RG_LG;
		screen->swap_gb = SWAP_GB_LG;
		screen->swap_delta = 0;
		screen->swap_dumy = 0;

		/* Operation function*/
		screen->init = lg_lcd_init;
		screen->standby = lg_lcd_standby;

		break;

    case TM_SCREEN:
			
        printk("lcd set TM PARAM \n");
		/* screen type & face */
		screen->type = SCREEN_TYPE_TM;
		screen->face = OUT_FACE_TM;

		/* Screen size */
		screen->x_res = H_VD_TM;
		screen->y_res = V_VD_TM;

		screen->width = LCD_WIDTH_TM;
		screen->height = LCD_HEIGHT_TM;

		/* Timing */
		screen->lcdc_aclk = LCDC_ACLK_TM;
		screen->pixclock = DCLK_TM;
		screen->left_margin = H_BP_TM;
		screen->right_margin = H_FP_TM;
		screen->hsync_len = H_PW_TM;
		screen->upper_margin = V_BP_TM;
		screen->lower_margin = V_FP_TM;
		screen->vsync_len = V_PW_TM;

		/* Pin polarity */
		screen->pin_hsync = HSYNC_POL_TM;
		screen->pin_vsync = VSYNC_POL_TM;
		screen->pin_den = DEN_POL_TM;
		screen->pin_dclk = DCLK_POL_TM;

		/* Swap rule */
		screen->swap_rb = SWAP_RB_TM;
		screen->swap_rg = SWAP_RG_TM;
		screen->swap_gb = SWAP_GB_TM;
		screen->swap_delta = 0;
		screen->swap_dumy = 0;

		/* Operation function*/
		screen->init = tm_lcd_init;
		screen->standby = tm_lcd_standby;

		break;

    default:
        printk(KERN_ERR "LCD type is not support %d \n",id);


    }

}


#endif

#endif
#endif
