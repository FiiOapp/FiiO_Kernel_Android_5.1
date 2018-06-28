
#ifndef __LCD_TM_ILI9806__
#define __LCD_TM_ILI9806__

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/i2c.h>

#include "lcd_X7II_define.h"

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

#endif
