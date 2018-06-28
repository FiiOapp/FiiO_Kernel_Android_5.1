#ifndef __LCD_X7II_DEFINE__
#define __LCD_X7II_DEFINE__

static struct rk29lcd_info *gLcd_info = NULL;

#define TFT_DET         RK30_PIN3_PB3
#define LCD_RESET_PORT  RK30_PIN2_PD4
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


#define DRVDelayUs(i)   udelay(i*4)
#define delay_us(i)      udelay(i)

void WriteCommand(int);
void WriteParameter(char);
void Write_Index_Data(int, char);
void Write_LCD_CMD(unsigned char);
void Write_LCD_DATA(unsigned char);

#endif
