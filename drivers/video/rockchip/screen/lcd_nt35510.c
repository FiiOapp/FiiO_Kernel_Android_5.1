
#ifndef __LCD_NT35510__
#define __LCD_NT35510__

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/i2c.h>
#include "../../../regulator/axp_power/axp-mfd.h"

#define TM_ID_PIN RK30_PIN1_PC2

static int isTMlcd = 0;
static inline bool is_gt9xx_ornot();

#define IS_GT9XX_MODE     is_gt9xx_ornot()

/* Base */
#define SCREEN_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_D888_P666//OUT_P888
#define LVDS_FORMAT       	LVDS_8BIT_1
#define DCLK			26*1000*1000	//***27
#define LCDC_ACLK       	300000000     //29 lcdc axi DMA Æµï¿œï¿œ           //rk29

#ifdef IS_GT9XX_MODE
/* Timing */
#define H_PW			10
#define H_BP			20
#define H_VD			480 
#define H_FP			20

#define V_PW			4
#define V_BP			6
#define V_VD			800
#define V_FP			10

#define LCD_WIDTH       52//57    //lcd size *mm
#define LCD_HEIGHT      86//94

#else

/* Timing */
#define H_PW			4 //8Ç°ÏûÓ°
#define H_BP			8//6
#define H_VD			480//320	//***800 
#define H_FP			8//60

#define V_PW			4//12
#define V_BP			8// 4
#define V_VD			800//480	//***480
#define V_FP			8//40

#define LCD_WIDTH       54//57    //lcd size *mm
#define LCD_HEIGHT      88//94

#endif

/* Other */
#define DCLK_POL		1//0 
#define DEN_POL			0
#define VSYNC_POL		0
#define HSYNC_POL		0

#define SWAP_RB			0
#define SWAP_RG			0
#define SWAP_GB			0

#define RK_SCREEN_INIT 	1

#if defined(RK_SCREEN_INIT)

#define LCD_RESET_PORT         RK30_PIN2_PD4

static struct rk29lcd_info *gLcd_info = NULL;

int rk_lcd_init(void);
int rk_lcd_standby(u8 enable);
extern bool is_gt9xx;

static inline bool is_gt9xx_ornot()
{
    return is_gt9xx;
}

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

#define LCD_RST_OUT()   gpio_direction_output(LCD_RESET_PORT, 0)
#define LCD_RST(i)      gpio_set_value(LCD_RESET_PORT, i)


#define DRVDelayUs(i)   udelay(i*4)

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
static void TM_init()
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
		Write_LCD_DATA(0x33);	// DVDDH DVDDL clamp	
				  
		Write_LCD_CMD(0x42);
		Write_LCD_DATA(0x02); 
						
		Write_LCD_CMD(0x43);
		Write_LCD_DATA(0x09);	// VGH/VGL 
						
		Write_LCD_CMD(0x44);
		Write_LCD_DATA(0x09);	// VGH/VGL			   
		
		
		Write_LCD_CMD(0x50);
		Write_LCD_DATA(0x78);	// VGMP  
					  
		Write_LCD_CMD(0x51);
		Write_LCD_DATA(0x78);	// VGMN
				   
		Write_LCD_CMD(0x52);
		Write_LCD_DATA(0x00);	//Flicker 
						  
		Write_LCD_CMD(0x53);
		Write_LCD_DATA(0x5e);	//Flicker
		
		
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

    TM_init();
}

static void JD9161_core_init(void)
{
        printk("%s: debug !!!\n", __FUNCTION__);

	Write_LCD_CMD(0xBf);
	Write_LCD_DATA(0x91);
	Write_LCD_DATA(0x61); 
	Write_LCD_DATA(0xF2);

	Write_LCD_CMD(0xb3);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x6E);

	Write_LCD_CMD(0xb4);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x6E);

	Write_LCD_CMD(0xB8);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0xBF);
	Write_LCD_DATA(0x01);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0xBF);
	Write_LCD_DATA(0x01);

	Write_LCD_CMD(0xBA);
	Write_LCD_DATA(0x34);
	Write_LCD_DATA(0x23);
	Write_LCD_DATA(0x00);

	Write_LCD_CMD(0xC3);
	Write_LCD_DATA(0x04);

	Write_LCD_CMD(0xC4);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x64);

	Write_LCD_CMD(0xC7);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x01);
	Write_LCD_DATA(0x31);
	Write_LCD_DATA(0x0A);
	Write_LCD_DATA(0x6A);
	Write_LCD_DATA(0x2C);
	Write_LCD_DATA(0x13);
	Write_LCD_DATA(0xA5);
	Write_LCD_DATA(0xAC);

	Write_LCD_CMD(0xC8);
	Write_LCD_DATA(0x7E);
	Write_LCD_DATA(0x61);
	Write_LCD_DATA(0x4F);
	Write_LCD_DATA(0x41);
	Write_LCD_DATA(0x3C);
	Write_LCD_DATA(0x2D);
	Write_LCD_DATA(0x30);
	Write_LCD_DATA(0x19);
	Write_LCD_DATA(0x31);
	Write_LCD_DATA(0x2E);
	Write_LCD_DATA(0x2E);
	Write_LCD_DATA(0x4D);
	Write_LCD_DATA(0x3C);
	Write_LCD_DATA(0x43);
	Write_LCD_DATA(0x34);
	Write_LCD_DATA(0x2F);
	Write_LCD_DATA(0x21);
	Write_LCD_DATA(0x0E);
	Write_LCD_DATA(0x01);
	Write_LCD_DATA(0x7E);
	Write_LCD_DATA(0x61);
	Write_LCD_DATA(0x4F);
	Write_LCD_DATA(0x41);
	Write_LCD_DATA(0x3C);
	Write_LCD_DATA(0x2D);
	Write_LCD_DATA(0x30);
	Write_LCD_DATA(0x19);
	Write_LCD_DATA(0x31);
	Write_LCD_DATA(0x2E);
	Write_LCD_DATA(0x2E);
	Write_LCD_DATA(0x4D);
	Write_LCD_DATA(0x3C);
	Write_LCD_DATA(0x43);
	Write_LCD_DATA(0x34);
	Write_LCD_DATA(0x2F);
	Write_LCD_DATA(0x21);
	Write_LCD_DATA(0x0E);
	Write_LCD_DATA(0x01);

	Write_LCD_CMD(0xD4);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1E);
	Write_LCD_DATA(0x05);
	Write_LCD_DATA(0x07);
	Write_LCD_DATA(0x01);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);

	Write_LCD_CMD(0xD5);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1E);
	Write_LCD_DATA(0x04);
	Write_LCD_DATA(0x06);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);

	Write_LCD_CMD(0xD6);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x06);
	Write_LCD_DATA(0x04);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x1E);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);

	Write_LCD_CMD(0xD7);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x07);
	Write_LCD_DATA(0x05);
	Write_LCD_DATA(0x01);
	Write_LCD_DATA(0x1E);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x1F);

	Write_LCD_CMD(0xD8);
	Write_LCD_DATA(0x20);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x10);
	Write_LCD_DATA(0x03);
	Write_LCD_DATA(0x20);
	Write_LCD_DATA(0x01);
	Write_LCD_DATA(0x02);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x01);
	Write_LCD_DATA(0x02);
	Write_LCD_DATA(0x5F);
	Write_LCD_DATA(0x5F);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x32);
	Write_LCD_DATA(0x04);
	Write_LCD_DATA(0x5F);
	Write_LCD_DATA(0x5F);
	Write_LCD_DATA(0x08);

	Write_LCD_CMD(0xD9);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x0A);
	Write_LCD_DATA(0x0A);
	Write_LCD_DATA(0x88);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x06);
	Write_LCD_DATA(0x7B);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x3B);
	Write_LCD_DATA(0x2F);
	Write_LCD_DATA(0x1F);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x03);
	Write_LCD_DATA(0x7B);

#if 0
	Write_LCD_CMD(0x36);
	Write_LCD_DATA(0x02);
#endif

	Write_LCD_CMD(0xBE);
	Write_LCD_DATA(0x01);
	//ESD 
	Write_LCD_CMD(0xCC);
	Write_LCD_DATA(0x34);
	Write_LCD_DATA(0x20);
	Write_LCD_DATA(0x38);
	Write_LCD_DATA(0x60);
	Write_LCD_DATA(0x11);
	Write_LCD_DATA(0x91);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x40);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0xDD);
	Write_LCD_DATA(0x21);

	Write_LCD_CMD(0xDD);
	Write_LCD_DATA(0x21);
	
	Write_LCD_CMD(0xBE);
	Write_LCD_DATA(0x00);
	
	Write_LCD_CMD(0x11);
	msleep(150);

	Write_LCD_CMD(0x29);
	msleep(20);
	Write_LCD_CMD(0x2C);
}

void init_nt35510(void)
{
//#Enable Page1
Write_Index_Data(0xF000,0x55);
Write_Index_Data(0xF001,0xAA);
Write_Index_Data(0xF002,0x52);
Write_Index_Data(0xF003,0x08);
Write_Index_Data(0xF004,0x01);

//#Default ?šŠ05  6v,0x09 -- 5.6V
Write_Index_Data(0xB000,0x09);
Write_Index_Data(0xB001,0x09);
Write_Index_Data(0xB002,0x09);

//# AVDD: manual, 6V (0x44: 2.5xVCI0)
Write_Index_Data(0xB600,0x34);
Write_Index_Data(0xB601,0x34);
Write_Index_Data(0xB602,0x34);

//#Default ?šŠ05  -6v,0x0C -- 5.3V     
Write_Index_Data(0xB100,0x09);
Write_Index_Data(0xB101,0x09);
Write_Index_Data(0xB102,0x09);

//# AVEE: manual, -6V (0x34: -2.5xVCI)
Write_Index_Data(0xB700,0x24);
Write_Index_Data(0xB701,0x24);
Write_Index_Data(0xB702,0x24);

//#12
Write_Index_Data(0xB300,0x05);
Write_Index_Data(0xB301,0x05);
Write_Index_Data(0xB302,0x05);  

//# VGH: Clamp Enable, 2*AVDD-AVEE, 11V(0x00,0x34,0x0B)
Write_Index_Data(0xB900,0x24);
Write_Index_Data(0xB901,0x24);
Write_Index_Data(0xB902,0x24); 

Write_Index_Data(0xbf00,0x01);

//# VGL_REG(VGLO):-13V
Write_Index_Data(0xB500,0x0b);
Write_Index_Data(0xB501,0x0b);
Write_Index_Data(0xB502,0x0b);

//# VGL(LVGL):
Write_Index_Data(0xBA00,0x34);
Write_Index_Data(0xBA01,0x24);
Write_Index_Data(0xBA02,0x24);

//# VGMP/VGSP:     4.8v //80=128x0.0125+3=4.6v if Setting 5.0v,BC01H be Set A0H
Write_Index_Data(0xBC00,0x00);
Write_Index_Data(0xBC01,0xA3);
Write_Index_Data(0xBC02,0X00); 

//# VGMN/VGSN  -4.8v
Write_Index_Data(0xBD00,0x00);
Write_Index_Data(0xBD01,0xA3);
Write_Index_Data(0xBD02,0x00);

//# VCOM=-0.1
Write_Index_Data(0xBE00,0x00);
Write_Index_Data(0xBE01,0x50);//ŠÌ¡Â??2???

//#R+                            
Write_Index_Data(0xD100,0x00);
Write_Index_Data(0xD101,0x37);
Write_Index_Data(0xD102,0x00);
Write_Index_Data(0xD103,0x52);
Write_Index_Data(0xD104,0x00);
Write_Index_Data(0xD105,0x7B);
Write_Index_Data(0xD106,0x00);
Write_Index_Data(0xD107,0x99);
Write_Index_Data(0xD108,0x00);
Write_Index_Data(0xD109,0xB1);
Write_Index_Data(0xD10A,0x00);
Write_Index_Data(0xD10B,0xD2);
Write_Index_Data(0xD10C,0x00);
Write_Index_Data(0xD10D,0xF6);
Write_Index_Data(0xD10E,0x01);
Write_Index_Data(0xD10F,0x27);
Write_Index_Data(0xD110,0x01);
Write_Index_Data(0xD111,0x4E);
Write_Index_Data(0xD112,0x01);
Write_Index_Data(0xD113,0x8C);
Write_Index_Data(0xD114,0x01);
Write_Index_Data(0xD115,0xBE);
Write_Index_Data(0xD116,0x02);
Write_Index_Data(0xD117,0x0B);
Write_Index_Data(0xD118,0x02);
Write_Index_Data(0xD119,0x48);
Write_Index_Data(0xD11A,0x02);
Write_Index_Data(0xD11B,0x4A);
Write_Index_Data(0xD11C,0x02);
Write_Index_Data(0xD11D,0x7E);
Write_Index_Data(0xD11E,0x02);
Write_Index_Data(0xD11F,0xBC);
Write_Index_Data(0xD120,0x02);
Write_Index_Data(0xD121,0xE1);
Write_Index_Data(0xD122,0x03);
Write_Index_Data(0xD123,0x10);
Write_Index_Data(0xD124,0x03);
Write_Index_Data(0xD125,0x31);
Write_Index_Data(0xD126,0x03);
Write_Index_Data(0xD127,0x5A);
Write_Index_Data(0xD128,0x03);
Write_Index_Data(0xD129,0x73);
Write_Index_Data(0xD12A,0x03);
Write_Index_Data(0xD12B,0x94);
Write_Index_Data(0xD12C,0x03);
Write_Index_Data(0xD12D,0x9F);
Write_Index_Data(0xD12E,0x03);
Write_Index_Data(0xD12F,0xB3);
Write_Index_Data(0xD130,0x03);
Write_Index_Data(0xD131,0xB9);
Write_Index_Data(0xD132,0x03);
Write_Index_Data(0xD133,0xC1);  

Write_Index_Data(0xD200,0x00);
Write_Index_Data(0xD201,0x37);
Write_Index_Data(0xD202,0x00);
Write_Index_Data(0xD203,0x52);
Write_Index_Data(0xD204,0x00);
Write_Index_Data(0xD205,0x7B);
Write_Index_Data(0xD206,0x00);
Write_Index_Data(0xD207,0x99);
Write_Index_Data(0xD208,0x00);
Write_Index_Data(0xD209,0xB1);
Write_Index_Data(0xD20A,0x00);
Write_Index_Data(0xD20B,0xD2);
Write_Index_Data(0xD20C,0x00);
Write_Index_Data(0xD20D,0xF6);
Write_Index_Data(0xD20E,0x01);
Write_Index_Data(0xD20F,0x27);
Write_Index_Data(0xD210,0x01);
Write_Index_Data(0xD211,0x4E);
Write_Index_Data(0xD212,0x01);
Write_Index_Data(0xD213,0x8C);
Write_Index_Data(0xD214,0x01);
Write_Index_Data(0xD215,0xBE);
Write_Index_Data(0xD216,0x02);
Write_Index_Data(0xD217,0x0B);
Write_Index_Data(0xD218,0x02);
Write_Index_Data(0xD219,0x48);
Write_Index_Data(0xD21A,0x02);
Write_Index_Data(0xD21B,0x4A);
Write_Index_Data(0xD21C,0x02);
Write_Index_Data(0xD21D,0x7E);
Write_Index_Data(0xD21E,0x02);
Write_Index_Data(0xD21F,0xBC);
Write_Index_Data(0xD220,0x02);
Write_Index_Data(0xD221,0xE1);
Write_Index_Data(0xD222,0x03);
Write_Index_Data(0xD223,0x10);
Write_Index_Data(0xD224,0x03);
Write_Index_Data(0xD225,0x31);
Write_Index_Data(0xD226,0x03);
Write_Index_Data(0xD227,0x5A);
Write_Index_Data(0xD228,0x03);
Write_Index_Data(0xD229,0x73);
Write_Index_Data(0xD22A,0x03);
Write_Index_Data(0xD22B,0x94);
Write_Index_Data(0xD22C,0x03);
Write_Index_Data(0xD22D,0x9F);
Write_Index_Data(0xD22E,0x03);
Write_Index_Data(0xD22F,0xB3);
Write_Index_Data(0xD230,0x03);
Write_Index_Data(0xD231,0xB9);
Write_Index_Data(0xD232,0x03);
Write_Index_Data(0xD233,0xC1);
//B+
Write_Index_Data(0xD300,0x00);
Write_Index_Data(0xD301,0x37);
Write_Index_Data(0xD302,0x00);
Write_Index_Data(0xD303,0x52);
Write_Index_Data(0xD304,0x00);
Write_Index_Data(0xD305,0x7B);
Write_Index_Data(0xD306,0x00);
Write_Index_Data(0xD307,0x99);
Write_Index_Data(0xD308,0x00);
Write_Index_Data(0xD309,0xB1);
Write_Index_Data(0xD30A,0x00);
Write_Index_Data(0xD30B,0xD2);
Write_Index_Data(0xD30C,0x00);
Write_Index_Data(0xD30D,0xF6);
Write_Index_Data(0xD30E,0x01);
Write_Index_Data(0xD30F,0x27);
Write_Index_Data(0xD310,0x01);
Write_Index_Data(0xD311,0x4E);
Write_Index_Data(0xD312,0x01);
Write_Index_Data(0xD313,0x8C);
Write_Index_Data(0xD314,0x01);
Write_Index_Data(0xD315,0xBE);
Write_Index_Data(0xD316,0x02);
Write_Index_Data(0xD317,0x0B);
Write_Index_Data(0xD318,0x02);
Write_Index_Data(0xD319,0x48);
Write_Index_Data(0xD31A,0x02);
Write_Index_Data(0xD31B,0x4A);
Write_Index_Data(0xD31C,0x02);
Write_Index_Data(0xD31D,0x7E);
Write_Index_Data(0xD31E,0x02);
Write_Index_Data(0xD31F,0xBC);
Write_Index_Data(0xD320,0x02);
Write_Index_Data(0xD321,0xE1);
Write_Index_Data(0xD322,0x03);
Write_Index_Data(0xD323,0x10);
Write_Index_Data(0xD324,0x03);
Write_Index_Data(0xD325,0x31);
Write_Index_Data(0xD326,0x03);
Write_Index_Data(0xD327,0x5A);
Write_Index_Data(0xD328,0x03);
Write_Index_Data(0xD329,0x73);
Write_Index_Data(0xD32A,0x03);
Write_Index_Data(0xD32B,0x94);
Write_Index_Data(0xD32C,0x03);
Write_Index_Data(0xD32D,0x9F);
Write_Index_Data(0xD32E,0x03);
Write_Index_Data(0xD32F,0xB3);
Write_Index_Data(0xD330,0x03);
Write_Index_Data(0xD331,0xB9);
Write_Index_Data(0xD332,0x03);
Write_Index_Data(0xD333,0xC1); 

Write_Index_Data(0xD400,0x00);
Write_Index_Data(0xD401,0x37);
Write_Index_Data(0xD402,0x00);
Write_Index_Data(0xD403,0x52);
Write_Index_Data(0xD404,0x00);
Write_Index_Data(0xD405,0x7B);
Write_Index_Data(0xD406,0x00);
Write_Index_Data(0xD407,0x99);
Write_Index_Data(0xD408,0x00);
Write_Index_Data(0xD409,0xB1);
Write_Index_Data(0xD40A,0x00);
Write_Index_Data(0xD40B,0xD2);
Write_Index_Data(0xD40C,0x00);
Write_Index_Data(0xD40D,0xF6);
Write_Index_Data(0xD40E,0x01);
Write_Index_Data(0xD40F,0x27);
Write_Index_Data(0xD410,0x01);
Write_Index_Data(0xD411,0x4E);
Write_Index_Data(0xD412,0x01);
Write_Index_Data(0xD413,0x8C);
Write_Index_Data(0xD414,0x01);
Write_Index_Data(0xD415,0xBE);
Write_Index_Data(0xD416,0x02);
Write_Index_Data(0xD417,0x0B);
Write_Index_Data(0xD418,0x02);
Write_Index_Data(0xD419,0x48);
Write_Index_Data(0xD41A,0x02);
Write_Index_Data(0xD41B,0x4A);
Write_Index_Data(0xD41C,0x02);
Write_Index_Data(0xD41D,0x7E);
Write_Index_Data(0xD41E,0x02);
Write_Index_Data(0xD41F,0xBC);
Write_Index_Data(0xD420,0x02);
Write_Index_Data(0xD421,0xE1);
Write_Index_Data(0xD422,0x03);
Write_Index_Data(0xD423,0x10);
Write_Index_Data(0xD424,0x03);
Write_Index_Data(0xD425,0x31);
Write_Index_Data(0xD426,0x03);
Write_Index_Data(0xD427,0x5A);
Write_Index_Data(0xD428,0x03);
Write_Index_Data(0xD429,0x73);
Write_Index_Data(0xD42A,0x03);
Write_Index_Data(0xD42B,0x94);
Write_Index_Data(0xD42C,0x03);
Write_Index_Data(0xD42D,0x9F);
Write_Index_Data(0xD42E,0x03);
Write_Index_Data(0xD42F,0xB3);
Write_Index_Data(0xD430,0x03);
Write_Index_Data(0xD431,0xB9);
Write_Index_Data(0xD432,0x03);
Write_Index_Data(0xD433,0xC1); 

Write_Index_Data(0xD500,0x00);
Write_Index_Data(0xD501,0x37);
Write_Index_Data(0xD502,0x00);
Write_Index_Data(0xD503,0x52);
Write_Index_Data(0xD504,0x00);
Write_Index_Data(0xD505,0x7B);
Write_Index_Data(0xD506,0x00);
Write_Index_Data(0xD507,0x99);
Write_Index_Data(0xD508,0x00);
Write_Index_Data(0xD509,0xB1);
Write_Index_Data(0xD50A,0x00);
Write_Index_Data(0xD50B,0xD2);
Write_Index_Data(0xD50C,0x00);
Write_Index_Data(0xD50D,0xF6);
Write_Index_Data(0xD50E,0x01);
Write_Index_Data(0xD50F,0x27);
Write_Index_Data(0xD510,0x01);
Write_Index_Data(0xD511,0x4E);
Write_Index_Data(0xD512,0x01);
Write_Index_Data(0xD513,0x8C);
Write_Index_Data(0xD514,0x01);
Write_Index_Data(0xD515,0xBE);
Write_Index_Data(0xD516,0x02);
Write_Index_Data(0xD517,0x0B);
Write_Index_Data(0xD518,0x02);
Write_Index_Data(0xD519,0x48);
Write_Index_Data(0xD51A,0x02);
Write_Index_Data(0xD51B,0x4A);
Write_Index_Data(0xD51C,0x02);
Write_Index_Data(0xD51D,0x7E);
Write_Index_Data(0xD51E,0x02);
Write_Index_Data(0xD51F,0xBC);
Write_Index_Data(0xD520,0x02);
Write_Index_Data(0xD521,0xE1);
Write_Index_Data(0xD522,0x03);
Write_Index_Data(0xD523,0x10);
Write_Index_Data(0xD524,0x03);
Write_Index_Data(0xD525,0x31);
Write_Index_Data(0xD526,0x03);
Write_Index_Data(0xD527,0x5A);
Write_Index_Data(0xD528,0x03);
Write_Index_Data(0xD529,0x73);
Write_Index_Data(0xD52A,0x03);
Write_Index_Data(0xD52B,0x94);
Write_Index_Data(0xD52C,0x03);
Write_Index_Data(0xD52D,0x9F);
Write_Index_Data(0xD52E,0x03);
Write_Index_Data(0xD52F,0xB3);
Write_Index_Data(0xD530,0x03);
Write_Index_Data(0xD531,0xB9);
Write_Index_Data(0xD532,0x03);
Write_Index_Data(0xD533,0xC1); 

Write_Index_Data(0xD600,0x00);
Write_Index_Data(0xD601,0x37);
Write_Index_Data(0xD602,0x00);
Write_Index_Data(0xD603,0x52);
Write_Index_Data(0xD604,0x00);
Write_Index_Data(0xD605,0x7B);
Write_Index_Data(0xD606,0x00);
Write_Index_Data(0xD607,0x99);
Write_Index_Data(0xD608,0x00);
Write_Index_Data(0xD609,0xB1);
Write_Index_Data(0xD60A,0x00);
Write_Index_Data(0xD60B,0xD2);
Write_Index_Data(0xD60C,0x00);
Write_Index_Data(0xD60D,0xF6);
Write_Index_Data(0xD60E,0x01);
Write_Index_Data(0xD60F,0x27);
Write_Index_Data(0xD610,0x01);
Write_Index_Data(0xD611,0x4E);
Write_Index_Data(0xD612,0x01);
Write_Index_Data(0xD613,0x8C);
Write_Index_Data(0xD614,0x01);
Write_Index_Data(0xD615,0xBE);
Write_Index_Data(0xD616,0x02);
Write_Index_Data(0xD617,0x0B);
Write_Index_Data(0xD618,0x02);
Write_Index_Data(0xD619,0x48);
Write_Index_Data(0xD61A,0x02);
Write_Index_Data(0xD61B,0x4A);
Write_Index_Data(0xD61C,0x02);
Write_Index_Data(0xD61D,0x7E);
Write_Index_Data(0xD61E,0x02);
Write_Index_Data(0xD61F,0xBC);
Write_Index_Data(0xD620,0x02);
Write_Index_Data(0xD621,0xE1);
Write_Index_Data(0xD622,0x03);
Write_Index_Data(0xD623,0x10);
Write_Index_Data(0xD624,0x03);
Write_Index_Data(0xD625,0x31);
Write_Index_Data(0xD626,0x03);
Write_Index_Data(0xD627,0x5A);
Write_Index_Data(0xD628,0x03);
Write_Index_Data(0xD629,0x73);
Write_Index_Data(0xD62A,0x03);
Write_Index_Data(0xD62B,0x94);
Write_Index_Data(0xD62C,0x03);
Write_Index_Data(0xD62D,0x9F);
Write_Index_Data(0xD62E,0x03);
Write_Index_Data(0xD62F,0xB3);
Write_Index_Data(0xD630,0x03);
Write_Index_Data(0xD631,0xB9);
Write_Index_Data(0xD632,0x03);
Write_Index_Data(0xD633,0xC1); 

//#Enable Page0
Write_Index_Data(0xF000,0x55);
Write_Index_Data(0xF001,0xAA);
Write_Index_Data(0xF002,0x52);
Write_Index_Data(0xF003,0x08);
Write_Index_Data(0xF004,0x00);
 
//#//# SDT:
Write_Index_Data(0xB600,0x0a);

//#//# Gate EQ:
Write_Index_Data(0xB700,0x00);
Write_Index_Data(0xB701,0x00);

//#//# Source EQ: 
Write_Index_Data(0xB800,0x01);
Write_Index_Data(0xB801,0x05);
Write_Index_Data(0xB802,0x05);
Write_Index_Data(0xB803,0x05);

Write_Index_Data(0xBa00,0x01);

//# Inversion: Column inversion (NVT)
Write_Index_Data(0xBC00,0x00);
Write_Index_Data(0xBC01,0x00);//¶Ô±È¶È
Write_Index_Data(0xBC02,0x00);

//# Display Timing:
Write_Index_Data(0xBD00,0x01);
Write_Index_Data(0xBD01,0x84);//??¡Àšš?šš
Write_Index_Data(0xBD02,0x07);
Write_Index_Data(0xBD03,0x31);
Write_Index_Data(0xBD04,0x00);

//# Display Timing:
Write_Index_Data(0xBE00,0x01);
Write_Index_Data(0xBE01,0x84);
Write_Index_Data(0xBE02,0x07);
Write_Index_Data(0xBE03,0x31);
Write_Index_Data(0xBE04,0x00);

//# Display Timing:
Write_Index_Data(0xBF00,0x01);
Write_Index_Data(0xBF01,0x84);
Write_Index_Data(0xBF02,0x07);
Write_Index_Data(0xBF03,0x31);
Write_Index_Data(0xBF04,0x00);

//# BOE's Setting (default)
Write_Index_Data(0xCC00,0x03);
Write_Index_Data(0xCC01,0x00);
Write_Index_Data(0xCC02,0x00);

Write_Index_Data(0xB100,0xF8);
Write_Index_Data(0xB101,0x00);

Write_Index_Data(0x3500,0x00);
Write_Index_Data(0x3a00,0x66);//16Bit  0x66 18Bit 0x77 24Bit
Write_Index_Data(0x3600,0x00);//0xc0 ·ŽÏòÉšÃè
 	  
WriteCommand(0x1100);	
msleep(120);
	
WriteCommand(0x2900);
}

static DEFINE_MUTEX(lcd_mutex);
extern void rk29_lcd_spim_spin_lock(void);
extern void rk29_lcd_spim_spin_unlock(void);

static void init_JD9161_io(void)
{
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

        printk("%s:  TXD_PORT  is  %d \n", __FUNCTION__, gpio_get_value(TXD_PORT));
        printk("%s:  CLK_PORT  is  %d \n", __FUNCTION__, gpio_get_value(CLK_PORT));
        printk("%s:  CS_PORT  is  %d \n", __FUNCTION__, gpio_get_value(CS_PORT));

        LCD_RST_OUT();
        LCD_RST(1);
        mdelay(30);
	LCD_RST(0);
	mdelay(100);
	LCD_RST(1);
	mdelay(100);
        gpio_free(LCD_RESET_PORT);

        JD9161_core_init();
}

static void lcd_resume(struct work_struct *work)
{
	mutex_lock(&lcd_mutex);
	rk29_lcd_spim_spin_lock();
	if(gLcd_info)
		gLcd_info->io_init();
		if(isTMlcd){
			init_TMILL9806_IO();
		}else if (is_gt9xx_ornot()) {
            gpio_request(LCD_RESET_PORT, NULL);
            LCD_RST_OUT();
            LCD_RST(1);
            mdelay(30);
	    LCD_RST(0);
	    mdelay(100);
	    LCD_RST(1);
	    mdelay(100);
            gpio_free(LCD_RESET_PORT);

            //may be fail to wake up LCD some time,so change to init lcd again
            JD9161_core_init();
        } else {
            gpio_request(LCD_RESET_PORT, NULL);
            gpio_direction_output(LCD_RESET_PORT, 0);
            LCD_RST(1);
            mdelay(30);
	    LCD_RST(0);
	    mdelay(100);
	    LCD_RST(1);
	    mdelay(100);
            //mdelay(2);
            //gpio_set_value(LCD_RESET_PORT, 1);
            //mdelay(10);
            gpio_free(LCD_RESET_PORT);

	     init_nt35510();
        }
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

int rk_lcd_init(void)
{ 
	volatile u32 data;
	printk("lcd init...\n");
	if(gLcd_info)
	gLcd_info->io_init();

	gpio_request(TM_ID_PIN, NULL);
    gpio_direction_input(TM_ID_PIN);
    gpio_pull_updown(TM_ID_PIN,GPIOPullUp);
	isTMlcd = !gpio_get_value(TM_ID_PIN);

	if(isTMlcd){
		printk("screen is TM \n");
		init_TMILL9806_IO();
	}else if (is_gt9xx_ornot()) {
        printk("%s: the screen is DiJing screen and TP is gt9xx !\n", __func__);
        init_JD9161_io();
    } else {
        gpio_request(LCD_RESET_PORT, NULL);
        gpio_direction_output(LCD_RESET_PORT, 0);
        mdelay(2);
        gpio_set_value(LCD_RESET_PORT, 1);
        mdelay(10);
        gpio_free(LCD_RESET_PORT);
    	init_nt35510();
    }

	if(gLcd_info)
	gLcd_info->io_deinit();
//printk("\nlipf+++++++++++++++debug+++++++++++screen!!!!!!!rk_lcd_init!!!!1111\n\n");
	lcd_resume_wq = create_singlethread_workqueue("lcd");
	register_early_suspend(&lcd_early_suspend_desc);
//printk("\nlipf+++++++++++++++debug+++++++++++screen!!!!!!!rk_lcd_init!!!!2222\n\n");
    return 0;
}

static void lcd_DiJing_suspend(void)
{
	Write_LCD_CMD(0xC6);
	Write_LCD_DATA(0x01);
	mdelay(200);
	Write_LCD_CMD(0x28);

	mdelay(200);
	Write_LCD_CMD(0x10);

	mdelay(100);
}

static void TM_suspend(void)
{
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

}

int rk_lcd_standby(u8 enable)	//***enable =1 means suspend, 0 means resume 
{
	if (enable) {
//printk("\nlipf+++++++++debug++++++++++lcd_nt35510!!!!!!!!(enable suspend)\n\n");
		mutex_lock(&lcd_mutex);
		rk29_lcd_spim_spin_lock();
		if(gLcd_info)
			gLcd_info->io_init();
				if(isTMlcd){
					TM_suspend();
				}else if (is_gt9xx_ornot()) {
                    lcd_DiJing_suspend();
                } else {
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
                }

		if(gLcd_info)
			gLcd_info->io_deinit();

		rk29_lcd_spim_spin_unlock();
		mutex_unlock(&lcd_mutex);

                axp_clr_bits(&axp->dev, 0x12, 0x01);
	} else {
//printk("\nlipf+++++++++debug++++++++++lcd_nt35510!!!!!!!!(enable resume)\n\n");
                axp_set_bits(&axp->dev, 0x12, 0x01);
		flush_workqueue(lcd_resume_wq);
                //rk_lcd_init();
	}
	return 0;
}

#endif

#endif
