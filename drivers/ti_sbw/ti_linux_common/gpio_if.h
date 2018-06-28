//*****************************************************************************
// gpio_if.h
//
// Defines and Macros for the GPIO interface.
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

#ifndef __GPIOIF_H__
#define __GPIOIF_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#define ROCKCHIP
#define USE_IOMEM
// BeagleBone User LEDs
#ifdef ROCKCHIP
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/iomux.h>

#include <mach/io.h>

static void __iomem *ggpio_base[] = {RK30_GPIO0_BASE, RK30_GPIO1_BASE, RK30_GPIO2_BASE, RK30_GPIO3_BASE};
struct gpio_data {
       void __iomem *base;
       unsigned int offset;
};

#define    LED1  INVALID_GPIO
#define    LED2  INVALID_GPIO
#define    LED3  INVALID_GPIO
#define    LED4  INVALID_GPIO

#ifdef USE_IOMEM

#define	RESET_PIN  0
#define	TEST_PIN   1

#define     TEST_PIN_GPIO_MODE GPIO1_A5
#define     RESET_PIN_GPIO_MODE GPIO1_C2
#else
/* SBW PINS */
#define	RESET_PIN  RK30_PIN1_PC2
#define	TEST_PIN   RK30_PIN1_PA5

#define     TEST_PIN_GPIO_MODE GPIO1_A5
#define     RESET_PIN_GPIO_MODE GPIO1_C2
#endif



#else

typedef enum
{
    LED1 = 21,  /* gpio53 User 0 - GPIO1_21 */
    LED2 = 22,  /* gpio54 User 1 - GPIO1_22 */
    LED3 = 23,  /* gpio55 User 2 - GPIO1_23 */
    LED4 = 24,  /* gpio56 User 3 - GPIO1_24 */

    /* SBW PINS */
	RESET_PIN = 18, /* gpio50 Reset - GPIO1_18 */
	TEST_PIN = 19   /* gpio51 Test  - GPIO1_19 */
} gpioEnum_t;
#endif
// OE: 0 is output, 1 is input
#define GPIO_OE  0x14d
#define GPIO_IN  0x14e
#define GPIO_OUT 0x14f

extern char cmdString[100];

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
#ifdef ROCKCHIP
extern void GPIO_IF_setAsOutputPin(int gpio);
extern void GPIO_IF_setAsInputPin(int gpio);
extern void GPIO_IF_setOutputHighOnPin(int gpio);
extern void GPIO_IF_setOutputLowOnPin(int gpio);
extern uint8_t GPIO_IF_getPinStatus(int gpio);
extern void loop(int n);
#else
extern void GPIO_IF_setAsOutputPin(gpioEnum_t gpio);
extern void GPIO_IF_setAsInputPin(gpioEnum_t gpio);
extern void GPIO_IF_setOutputHighOnPin(gpioEnum_t gpio);
extern void GPIO_IF_setOutputLowOnPin(gpioEnum_t gpio);
extern uint8_t GPIO_IF_getPinStatus(gpioEnum_t gpio);
extern void GPIO_IF_toggleOutputOnPin(gpioEnum_t gpio);
#endif

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  __GPIOIF_H__

