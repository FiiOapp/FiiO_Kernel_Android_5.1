
#ifndef __LCD_LG_HX8397__
#define __LCD_LG_HX8397__

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/i2c.h>

/* Base */
#define SCREEN_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_D888_P666//OUT_P888
#define LVDS_FORMAT       	LVDS_8BIT_1
#define DCLK			26*1000*1000	//***27
#define LCDC_ACLK       	300000000     //29 lcdc axi DMA Æµï¿œï¿œ           //rk29

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

static DEFINE_MUTEX(lcd_mutex);
extern void rk29_lcd_spim_spin_lock(void);
extern void rk29_lcd_spim_spin_unlock(void);

static void init_HX8397_io(void)
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

        HX8397_core_init();
}

static void lcd_resume(struct work_struct *work)
{
	mutex_lock(&lcd_mutex);
	rk29_lcd_spim_spin_lock();
	if(gLcd_info)
		gLcd_info->io_init();

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
            HX8397_core_init();

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

        init_HX8397_io();

	if(gLcd_info)
	gLcd_info->io_deinit();
//printk("\nlipf+++++++++++++++debug+++++++++++screen!!!!!!!rk_lcd_init!!!!1111\n\n");
	lcd_resume_wq = create_singlethread_workqueue("lcd");
	register_early_suspend(&lcd_early_suspend_desc);
//printk("\nlipf+++++++++++++++debug+++++++++++screen!!!!!!!rk_lcd_init!!!!2222\n\n");
    return 0;
}

static void lcd_LG_suspend(void)
{
	//Write_LCD_CMD(0xC6);
	//Write_LCD_DATA(0x01);
	//mdelay(200);
	Write_LCD_CMD(0x28);

	mdelay(50);
	Write_LCD_CMD(0x10);

	mdelay(50);
}

static void lcd_LG_resume(void)
{
	//Write_LCD_CMD(0xC6);
	//Write_LCD_DATA(0x01);
	//mdelay(200);
	Write_LCD_CMD(0x11);

	mdelay(150);
	Write_LCD_CMD(0x29);

	mdelay(50);
}

int rk_lcd_standby(u8 enable)	//***enable =1 means suspend, 0 means resume 
{
	if (enable) {
//printk("\nlipf+++++++++debug++++++++++lcd_nt35510!!!!!!!!(enable suspend)\n\n");
		mutex_lock(&lcd_mutex);
		rk29_lcd_spim_spin_lock();
		if(gLcd_info)
			gLcd_info->io_init();

                lcd_LG_suspend();
#if 0                
                    //lcd_DiJing_suspend();
                 
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
               
#endif
		if(gLcd_info)
			gLcd_info->io_deinit();

		rk29_lcd_spim_spin_unlock();
		mutex_unlock(&lcd_mutex);

	} else {
		flush_workqueue(lcd_resume_wq);
                //rk_lcd_init();

                lcd_LG_resume();
	}
	return 0;
}

#endif

#endif
