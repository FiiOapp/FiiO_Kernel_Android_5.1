//*****************************************************************************
// gpio_if.c
//
// GPIO interface APIs.
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

/* #include <stdio.h> */
/* #include <string.h> */
/* #include <stdint.h> */
/* #include <stdbool.h> */
/* #include <stdlib.h> */
#include "gpio_if.h"
#include "pinmux.h"

//****************************************************************************
//                      GLOBAL VARIABLES                                   
//****************************************************************************
char cmdString[100];

void loop(int n)
{
    unsigned int i;
    if(n)
        for(i=0;i<=10*n;i++){

            __asm("    nop");

        }

}

#ifdef ROCKCHIP
extern inline void rk30_gpio_bit_op(void __iomem *regbase, unsigned int offset, u32 bit, unsigned char flag);

int pintable[]={RK30_PIN1_PC2,RK30_PIN1_PA5};
struct gpio_data  iobase[2];
static int gpio_init(int gpio, struct sram_gpio_data *data)
{
       unsigned index;

       if(gpio == INVALID_GPIO)
               return -EINVAL;
       index = gpio - PIN_BASE;
       if(index/NUM_GROUP >= ARRAY_SIZE(ggpio_base))
               return -EINVAL;

       data->base = ggpio_base[index/NUM_GROUP];
       printk("MSG SBW base 0x%x offset 0x%x \n",data->base,index%NUM_GROUP);
       if(data->base == 0)
	       return -EINVAL;

       data->offset = 1<<(index%NUM_GROUP);

       /* data.offset = (1<<data.offset); */
       return 0;
}

int ti_sbw_hardware_init()
{
    int ret=0;
    static int init=0;


    if(init)
        return 0;

#ifdef USE_IOMEM

    iomux_set(RESET_PIN_GPIO_MODE);
    iomux_set(TEST_PIN_GPIO_MODE);
    ret = gpio_request(pintable[1], "ti sbw test pin");
    if (ret) {
        printk("ti sbw init failed TEST_PIN \n");
        return -EBUSY;
    }

    ret = gpio_request(pintable[0], "ti sbw reset pin");
    if (ret) {
        printk("ti sbw init failed RESET_PIN \n");
        return -EBUSY;
    }

    gpio_direction_output(pintable[1],GPIO_LOW);
    msleep(2000);
    /* gpio_pull_updown(pintable[0],PullDisable); */
    gpio_direction_input(pintable[0]);
    gpio_direction_input(pintable[1]);

    /* gpio_direction_output(pintable[0],GPIO_HIGH); */

    gpio_init(pintable[0],&iobase[0]);
    gpio_init(pintable[1],&iobase[1]);
#else
    iomux_set(RESET_PIN_GPIO_MODE);
    iomux_set(TEST_PIN_GPIO_MODE);
    ret = gpio_request(TEST_PIN, "ti sbw test pin");
    if (ret) {
        printk("ti sbw init failed TEST_PIN \n");
        return -EBUSY;
    }

    ret = gpio_request(RESET_PIN, "ti sbw reset pin");
    if (ret) {
        printk("ti sbw init failed RESET_PIN \n");
        return -EBUSY;
    }
    gpio_pull_updown(RESET_PIN,PullDisable);
    gpio_direction_input(TEST_PIN);
    gpio_direction_input(RESET_PIN);
    /* gpio_direction_output(RESET_PIN,1); */

#endif
    init=1;
    return ret;
}

void ti_sbw_reset_mcu(void)
{

    gpio_direction_output(RESET_PIN,0);
    mdelay(5);
    gpio_direction_output(RESET_PIN,1);
}

//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                          
//****************************************************************************

//*****************************************************************************
//
//! GPIO Enable & Configuration
//!
//! \param  gpio
//!
//! \return None
//
//*************
void
GPIO_IF_setAsOutputPin(int gpio)
{
    /* gpio_direction_output(gpio,GPIO_HIGH); */
}
//*****************************************************************************
//
//! GPIO Enable & Configuration
//!
//! \param  gpio
//!
//! \return None
//
//*
void
GPIO_IF_setAsInputPin(int gpio)
{
#ifndef USE_IOMEM_
    gpio_direction_input(pintable[gpio]);
#else

    /* printk("MSG base 0x%x %d ",iobase[gpio].base,gpio); */
	rk30_gpio_bit_op(iobase[gpio].base, GPIO_SWPORT_DDR, iobase[gpio].offset, 0);
#endif
}

//*****************************************************************************
//
//! Turn gpio On
//!
//! \param  gpio
//!
//! \return none
//!
//! \brief  Turns a specific gpio On
//
void
GPIO_IF_setOutputHighOnPin(int gpio)
{
 #ifndef USE_IOMEM
   gpio_direction_output(gpio,GPIO_HIGH);
#else

	rk30_gpio_bit_op(iobase[gpio].base, GPIO_SWPORT_DDR, iobase[gpio].offset, 1);
	rk30_gpio_bit_op(iobase[gpio].base, GPIO_SWPORT_DR, iobase[gpio].offset, GPIO_HIGH);
#endif
}

//*****************************************************************************
//
//! Turn gpio Off
//!
//! \param  gpio
//!
//! \return none
//!
//! \brief  Turns a specific gpio Off
//
//****************************************************************************
void
GPIO_IF_setOutputLowOnPin(int gpio)
{

#ifndef USE_IOMEM
    gpio_direction_output(gpio,GPIO_LOW);
#else

	rk30_gpio_bit_op(iobase[gpio].base, GPIO_SWPORT_DDR, iobase[gpio].offset, 1);
	rk30_gpio_bit_op(iobase[gpio].base, GPIO_SWPORT_DR, iobase[gpio].offset, GPIO_LOW);
#endif
}

//*****************************************************************************
//
//!  \brief This function returns gpio current Status
//!
//!  \param[in] gpio
//!
//!  \return 1: ON, 0: OFF
//
//*****************************************************************************
uint8_t
GPIO_IF_getPinStatus(int gpio)
{

#ifndef USE_IOMEM
    int ret;
    ret=gpio_get_value(gpio);

    return ret? 1:0;
    
#else

	return ((__raw_readl(iobase[gpio].base+ GPIO_EXT_PORT) & iobase[gpio].offset) != 0);
#endif
}
//*****************************************************************************
//
//! Toggle the gpio state
//!
//! \param  gpio is the LED Number
//!
//! \return none
//!
//! \brief  Toggles a board gpio
//
//*****************************************************************************
void
GPIO_IF_toggleOutputOnPin(int gpio)
{
	uint8_t gpioStatus = GPIO_IF_getPinStatus(gpio);

	if(gpioStatus == 1)
	{
		GPIO_IF_setOutputLowOnPin(gpio);
	}
	else
	{
		GPIO_IF_setOutputHighOnPin(gpio);
	}
}


#else
//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                          
//****************************************************************************

//*****************************************************************************
//
//! GPIO Enable & Configuration
//!
//! \param  gpio
//!
//! \return None
//
//*****************************************************************************
void
GPIO_IF_setAsOutputPin(gpioEnum_t gpio)
{
    // Get direction control register contents
    unsigned int creg = *(gpio_port + GPIO_OE);

	switch(gpio)
	{
	  case LED1:
	  case LED2:
	  case LED3:
	  case LED4:
	  case RESET_PIN:
	  case TEST_PIN:
		  creg = creg & (~(1<<gpio));
		break;
	  default:
		  break;
	}
    // Set new direction control register contents
    *(gpio_port + GPIO_OE) = creg;
}

//*****************************************************************************
//
//! GPIO Enable & Configuration
//!
//! \param  gpio
//!
//! \return None
//
//*****************************************************************************
void
GPIO_IF_setAsInputPin(gpioEnum_t gpio)
{
	unsigned int creg = *(gpio_port + GPIO_OE);

	switch(gpio)
	{
	  case LED1:
	  case LED2:
	  case LED3:
	  case LED4:
	  case RESET_PIN:
	  case TEST_PIN:
		  creg = creg | (1<<gpio);
		break;
	  default:
		  break;
	}
    *(gpio_port + GPIO_OE) = creg;
}

//*****************************************************************************
//
//! Turn gpio On
//!
//! \param  gpio
//!
//! \return none
//!
//! \brief  Turns a specific gpio On
//
//*****************************************************************************
void
GPIO_IF_setOutputHighOnPin(gpioEnum_t gpio)
{
	switch(gpio)
	{
	  case LED1:
	  case LED2:
	  case LED3:
	  case LED4:
	  case RESET_PIN:
	  case TEST_PIN:
		  *(gpio_port + GPIO_OUT) = *(gpio_port + GPIO_OUT) | (1<<gpio);
		break;
	  default:
		  break;
	}
}

//*****************************************************************************
//
//! Turn gpio Off
//!
//! \param  gpio
//!
//! \return none
//!
//! \brief  Turns a specific gpio Off
//
//*****************************************************************************
void
GPIO_IF_setOutputLowOnPin(gpioEnum_t gpio)
{
	switch(gpio)
	{
	  case LED1:
	  case LED2:
	  case LED3:
	  case LED4:
	  case RESET_PIN:
	  case TEST_PIN:
		  *(gpio_port + GPIO_OUT) = *(gpio_port + GPIO_OUT) & (~(1<<gpio));
		break;
	  default:
		  break;
	}
}

//*****************************************************************************
//
//!  \brief This function returns gpio current Status
//!
//!  \param[in] gpio
//!
//!  \return 1: ON, 0: OFF
//
//*****************************************************************************
uint8_t
GPIO_IF_getPinStatus(gpioEnum_t gpio)
{
	uint8_t gpioStatus = 0;

	switch(gpio)
	{
	  case LED1:
	  case LED2:
	  case LED3:
	  case LED4:
	  case RESET_PIN:
	  case TEST_PIN:
		  if ((*(gpio_port + GPIO_IN) & (1<<gpio)) == (1<<gpio))
		  {
			  gpioStatus = 1;
		  }
		break;
	  default:
		  break;
	}
	return (gpioStatus);
}

//*****************************************************************************
//
//! Toggle the gpio state
//!
//! \param  gpio is the LED Number
//!
//! \return none
//!
//! \brief  Toggles a board gpio
//
//*****************************************************************************
void
GPIO_IF_toggleOutputOnPin(gpioEnum_t gpio)
{
	uint8_t gpioStatus = GPIO_IF_getPinStatus(gpio);

	if(gpioStatus == 1)
	{
		GPIO_IF_setOutputLowOnPin(gpio);
	}
	else
	{
		GPIO_IF_setOutputHighOnPin(gpio);
	}
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif
