
#ifndef __LCD_X7II_SCREEN__
#define __LCD_X7II_SCREEN__

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/i2c.h>

#include "lcd_X7II_define.h"
#include "lcd_LG_HX8397_new.c"
#include "lcd_TM_ITL9806.c"

#define RK_SCREEN_INIT 	1

#if defined(RK_SCREEN_INIT)

static struct rk29fb_screen *gLcd_screen = NULL;
static unsigned char reg_data[3] = {0};
static bool isLGScreen = false;
static bool isTMILI9806= false;

static int gLcd_type=-1;

enum screen_type{
    LG_SCREEN,
    TM_SCREEN,
};

int rk_lcd_init(void);
int rk_lcd_standby(u8 enable);

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

static DEFINE_MUTEX(lcd_mutex);
extern void rk29_lcd_spim_spin_lock(void);
extern void rk29_lcd_spim_spin_unlock(void);

static void lcd_resume(struct work_struct *work)
{
	mutex_lock(&lcd_mutex);
	rk29_lcd_spim_spin_lock();
	if(gLcd_info)
		gLcd_info->io_init();

	printk(KERN_DEBUG "%s\n",__FUNCTION__);

        gpio_request(LCD_RESET_PORT, NULL);
        if (isLGScreen) {
            printk("%s: current screen is LG HX8397 \n", __func__);
            init_HX8397_IO();
        }else if(isTMILI9806){
            printk("%s: current screen is TM ILI9806\n", __func__);
            init_TMILL9806_IO();
        } else {

        }

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
    } else {
        //open lcd power
        flush_workqueue(lcd_resume_wq);

        if (isLGScreen) {
            lcd_LG_resume();
        }
    }

    return 0;
}

int rk_lcd_standby(u8 enable)	//***enable =1 means suspend, 0 means resume 
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

	} else {
		flush_workqueue(lcd_resume_wq);

                if (isLGScreen) {
                    lcd_LG_resume();
                }
	}
	return 0;
}

#if defined(CONFIG_LIDA_MACH_X7II)
extern int act8846_sdcard_power(char on);
#endif

static int lcd_get_id(void)
{
    int id = -1;
    int ts_id = -1;
    int err;
    int det_pu,det_pd;

    #if defined(CONFIG_LIDA_MACH_X7II)
    act8846_sdcard_power(0);
    #endif

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

    printk("TFT_DET io pull up dwon value : %d  %d\n",det_pu,det_pd);
    Read_LG_HX8397_ID();

    if (reg_data[0] == 131 && reg_data[1] == 121 && reg_data[2] == 12) {
        printk("%s: current is LG HX8397 screen !\n", __func__);
        isLGScreen = true;
        gLcd_type = LG_SCREEN;
    }else if(!det_pu && !det_pd ){
        printk("%s: current is TM ILI9860 screen !\n", __func__);
        isTMILI9806=true;
        gLcd_type = TM_SCREEN;
    } else {
        isTMILI9806=true;
        gLcd_type = TM_SCREEN;
        printk("%s: current is BOE screen !\n", __func__);
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
