
#ifndef __LCD_LG4571__
#define __LCD_LG4571__

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
#define H_PW			8//4 //8Ç°ÏûÓ°
#define H_BP			14//6
#define H_VD			480//320	//***800 
#define H_FP			8//60

#define V_PW			2//4//12
#define V_BP			8// 4
#define V_VD			800//480	//***480
#define V_FP			18//8//40

#define LCD_WIDTH       42//45//54//57    //lcd size *mm
#define LCD_HEIGHT      70//74//88//94

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
#define TXD_IN()        gpio_direction_input(TXD_PORT)
#define TXD_GET()       gpio_get_value(TXD_PORT)

#define LCD_RST_OUT()   gpio_direction_output(LCD_RESET_PORT, 0)
#define LCD_RST(i)      gpio_set_value(LCD_RESET_PORT, i)


#define DRVDelayUs(i)   udelay(i*2)
#define UDELAY_TIME     1//2

static void SPI_SendData(unsigned char data)
{  
    unsigned char i;
   
    for(i=0; i<8; i++)			
    {
        if(data & 0x80)
            TXD_SET();
      	else
            TXD_CLR();
        data <<= 1;

        CLK_CLR();
        //udelay(UDELAY_TIME);
        CLK_SET();
        //udelay(UDELAY_TIME);
     }
}

static void Write_LCD_CMD(unsigned char cmd)
{
    CS_CLR();

    pr_info("%s: cmd>>8:0x%x,  cmd:0x%x \n", __func__, cmd>>8, cmd);

    SPI_SendData(0x7C);
    SPI_SendData(cmd>>8);
    SPI_SendData(cmd);

    CS_SET();
}

void Write_LCD_DATA(unsigned char data)
{
    CS_CLR();

    pr_info("%s: data>>8:0x%x,  data:0x%x \n", __func__, data>>8, data);

    SPI_SendData(0x7E);
    SPI_SendData(data>>8);
    SPI_SendData(data);

    CS_SET();
}

#if 1
static void LG4571_core_init(void)
{
        printk("%s: debug !!!\n", __FUNCTION__);

	Write_LCD_CMD(0x20);
	//Write_LCD_CMD(0x29);

	Write_LCD_CMD(0x3A);
	Write_LCD_DATA(0x60); //0x70:24bit, 0x60:18bit
	
	//Write_LCD_CMD(0x36);
	//Write_LCD_DATA(0x04);
	
	Write_LCD_CMD(0xB1);
	Write_LCD_DATA(0x10);//0x10
	Write_LCD_DATA(0x16);//39//0x16
	Write_LCD_DATA(0x08);//08
	
	Write_LCD_CMD(0xB2);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0xC8);	//0XC8

	Write_LCD_CMD(0xB3);
	Write_LCD_DATA(0x11);
	Write_LCD_DATA(0xff);

	Write_LCD_CMD(0xB4); //24bit to 18bit, 0x00: enable, 0x10: disable
	Write_LCD_DATA(0x10); //0x10

	Write_LCD_CMD(0xB6);
	Write_LCD_DATA(0x0E);//0X38
	Write_LCD_DATA(0x18);//18
	Write_LCD_DATA(0x3a);//3a
	Write_LCD_DATA(0x40);//40
	Write_LCD_DATA(0x10);//10


	Write_LCD_CMD(0xB7);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x10);
	Write_LCD_DATA(0x01);


	Write_LCD_CMD(0xC3);
	Write_LCD_DATA(0x01);
	Write_LCD_DATA(0x02);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x07);
	Write_LCD_DATA(0x01);

	Write_LCD_CMD(0xC4);
	Write_LCD_DATA(0x33);
	Write_LCD_DATA(0x03);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x1b);
	Write_LCD_DATA(0x1b);
	Write_LCD_DATA(0x05);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x04);

	Write_LCD_CMD(0xC5);
	Write_LCD_DATA(0x6f);
	//Write_LCD_DATA(0x07);

	Write_LCD_CMD(0xC6);
	Write_LCD_DATA(0x23);
	//Write_LCD_DATA(0x00);
	//Write_LCD_DATA(0x00);
	
	Write_LCD_CMD(0xC8);
	Write_LCD_DATA(0x01);//0x01
	Write_LCD_DATA(0x01);
	Write_LCD_DATA(0x03);
	Write_LCD_DATA(0xff);//0xff
	

	Write_LCD_CMD(0xD0);     
	Write_LCD_DATA(0x00);//0x00
	Write_LCD_DATA(0x55);//0x55
	Write_LCD_DATA(0x75);//0x75
	Write_LCD_DATA(0x05);
	Write_LCD_DATA(0x08);//0x08
	Write_LCD_DATA(0x07);
	Write_LCD_DATA(0x65);
	Write_LCD_DATA(0x24);
	Write_LCD_DATA(0x03);

	Write_LCD_CMD(0xD1);     
	Write_LCD_DATA(0x40);//0x40
	Write_LCD_DATA(0x45);//0x45
	Write_LCD_DATA(0x77);
	Write_LCD_DATA(0x27);
	Write_LCD_DATA(0x1C);//0x1c
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x57);
	Write_LCD_DATA(0x43);
	Write_LCD_DATA(0x04);

	Write_LCD_CMD(0xD2);     
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x55);//0x55
	Write_LCD_DATA(0x75);//0x75
	Write_LCD_DATA(0x05);
	Write_LCD_DATA(0x08);//0x08
	Write_LCD_DATA(0x07);
	Write_LCD_DATA(0x65);
	Write_LCD_DATA(0x24);
	Write_LCD_DATA(0x03);

	Write_LCD_CMD(0xD3);
	Write_LCD_DATA(0x40);//0x40
	Write_LCD_DATA(0x45);//0x45
	Write_LCD_DATA(0x77);//0x77
	Write_LCD_DATA(0x27);
	Write_LCD_DATA(0x1C);//0x1C
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x57);
	Write_LCD_DATA(0x43);
	Write_LCD_DATA(0x04);
	

	//Write_LCD_CMD(0x11);

	Write_LCD_CMD(0xC2);
	Write_LCD_DATA(0x08);
	mdelay(10);
	Write_LCD_CMD(0xC2);
	Write_LCD_DATA(0x18);
	mdelay(10);
	Write_LCD_CMD(0xC2);
	Write_LCD_DATA(0xB8);
	mdelay(20);
	
	Write_LCD_CMD(0xB5);
	Write_LCD_DATA(0x01);
	mdelay(50);

	Write_LCD_CMD(0x11);
	msleep(120);
	Write_LCD_CMD(0x29);
	msleep(150);
}
#endif

#if 0
static void LG4571_core_init(void)
{
        printk("%s: debug !!!\n", __FUNCTION__);

	Write_LCD_CMD(0x20);
	//Write_LCD_CMD(0x29);

	Write_LCD_CMD(0x3A);
	Write_LCD_DATA(0x70); //0x70:24bit, 0x60:18bit
	
	//Write_LCD_CMD(0x36);
	//Write_LCD_DATA(0x04);
	
	Write_LCD_CMD(0xB1);
	Write_LCD_DATA(0x10);//0x10
	Write_LCD_DATA(0x16);//39//0x16
	Write_LCD_DATA(0x08);//08
	
	Write_LCD_CMD(0xB2);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0xC8);	//0XC8

	Write_LCD_CMD(0xB3);
	Write_LCD_DATA(0x11);
	Write_LCD_DATA(0xff);

	Write_LCD_CMD(0xB4); //24bit to 18bit, 0x00: enable, 0x10: disable
	Write_LCD_DATA(0x10); //0x10

	Write_LCD_CMD(0xB6);
	Write_LCD_DATA(0x0E);//0X38
	Write_LCD_DATA(0x18);//18
	Write_LCD_DATA(0x3a);//3a
	Write_LCD_DATA(0x40);//40
	Write_LCD_DATA(0x10);//10


	Write_LCD_CMD(0xB7);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x10);
	Write_LCD_DATA(0x01);


	Write_LCD_CMD(0xC3);
	Write_LCD_DATA(0x01);
	Write_LCD_DATA(0x02);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x07);
	Write_LCD_DATA(0x01);

	Write_LCD_CMD(0xC4);
	Write_LCD_DATA(0x33);
	Write_LCD_DATA(0x03);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x1b);
	Write_LCD_DATA(0x1b);
	Write_LCD_DATA(0x05);
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x04);

	Write_LCD_CMD(0xC5);
	Write_LCD_DATA(0x6f);
	//Write_LCD_DATA(0x07);

	Write_LCD_CMD(0xC6);
	Write_LCD_DATA(0x23);
	//Write_LCD_DATA(0x00);
	//Write_LCD_DATA(0x00);
	
	Write_LCD_CMD(0xC8);
	Write_LCD_DATA(0x01);//0x01
	Write_LCD_DATA(0x01);
	Write_LCD_DATA(0x03);
	Write_LCD_DATA(0xff);//0xff
	

	Write_LCD_CMD(0xD0);     
	Write_LCD_DATA(0x00);//0x00
	Write_LCD_DATA(0x55);//0x55
	Write_LCD_DATA(0x75);//0x75
	Write_LCD_DATA(0x05);
	Write_LCD_DATA(0x08);//0x08
	Write_LCD_DATA(0x07);
	Write_LCD_DATA(0x65);
	Write_LCD_DATA(0x24);
	Write_LCD_DATA(0x03);

	Write_LCD_CMD(0xD1);     
	Write_LCD_DATA(0x40);//0x40
	Write_LCD_DATA(0x45);//0x45
	Write_LCD_DATA(0x77);
	Write_LCD_DATA(0x27);
	Write_LCD_DATA(0x1C);//0x1c
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x57);
	Write_LCD_DATA(0x43);
	Write_LCD_DATA(0x04);

	Write_LCD_CMD(0xD2);     
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x55);//0x55
	Write_LCD_DATA(0x75);//0x75
	Write_LCD_DATA(0x05);
	Write_LCD_DATA(0x08);//0x08
	Write_LCD_DATA(0x07);
	Write_LCD_DATA(0x65);
	Write_LCD_DATA(0x24);
	Write_LCD_DATA(0x03);

	Write_LCD_CMD(0xD3);
	Write_LCD_DATA(0x40);//0x40
	Write_LCD_DATA(0x45);//0x45
	Write_LCD_DATA(0x77);//0x77
	Write_LCD_DATA(0x27);
	Write_LCD_DATA(0x1C);//0x1C
	Write_LCD_DATA(0x00);
	Write_LCD_DATA(0x57);
	Write_LCD_DATA(0x43);
	Write_LCD_DATA(0x04);
	

	Write_LCD_CMD(0x11);

	Write_LCD_CMD(0xC2);
	Write_LCD_DATA(0x08);
	mdelay(10);
	Write_LCD_CMD(0xC2);
	Write_LCD_DATA(0x18);
	mdelay(10);
	Write_LCD_CMD(0xC2);
	Write_LCD_DATA(0xB8);
	mdelay(200);
	
	Write_LCD_CMD(0xB5);
	Write_LCD_DATA(0x01);
	mdelay(100);

	Write_LCD_CMD(0x29);
	
	mdelay(500);
}
#endif

static DEFINE_MUTEX(lcd_mutex);
extern void rk29_lcd_spim_spin_lock(void);
extern void rk29_lcd_spim_spin_unlock(void);

static void init_LG4571_io(void)
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
        //CS_CLR();
        printk("%s:  TXD_PORT  is  %d \n", __FUNCTION__, gpio_get_value(TXD_PORT));
        printk("%s:  CLK_PORT  is  %d \n", __FUNCTION__, gpio_get_value(CLK_PORT));
        printk("%s:  CS_PORT  is  %d \n", __FUNCTION__, gpio_get_value(CS_PORT));

        LCD_RST_OUT();
        LCD_RST(1);
        mdelay(10);
	LCD_RST(0);
	mdelay(50);
	LCD_RST(1);
	mdelay(30);
        gpio_free(LCD_RESET_PORT);
        printk("%s:  LCD_RESET_PORT  is  %d \n", __FUNCTION__, gpio_get_value(LCD_RESET_PORT));

        LG4571_core_init();
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
            mdelay(10);
	    LCD_RST(0);
	    mdelay(50);
	    LCD_RST(1);
	    mdelay(30);
            gpio_free(LCD_RESET_PORT);

            //may be fail to wake up LCD some time,so change to init lcd again
            LG4571_core_init();

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

        init_LG4571_io();

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
	Write_LCD_CMD(0x28);
	mdelay(50);
	Write_LCD_CMD(0x10);
	mdelay(50);

        gpio_request(LCD_RESET_PORT, NULL);
        LCD_RST_OUT();
	LCD_RST(0);
	msleep(100);
        gpio_free(LCD_RESET_PORT);
}

static void lcd_LG_resume(void)
{
	Write_LCD_CMD(0x11);
	msleep(150);
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
