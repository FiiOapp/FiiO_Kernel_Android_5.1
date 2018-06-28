//*****************************************************************************
// main.c
//
// Reference code to demonstrate controlling of GPIO. In this case LED blinky.
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

//****************************************************************************
//
//! \addtogroup blinky
//! @{
//
//****************************************************************************
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include "pinmux.h"
#include "gpio_if.h"
#include "dcdc_sbw.h"

//*****************************************************************************
//                      GLOBAL VARIABLES
//*****************************************************************************

/****************************************************************************/
/*                      LOCAL FUNCTION PROTOTYPES                           */
/****************************************************************************/
void LEDBlinkyRoutine();
static void BoardInit(void);
/****************************************************************************/
/*                      LOCAL FUNCTION DEFINITIONS                          */
/****************************************************************************/

//****************************************************************************
//
//! Configures the pins as GPIOs and periodically toggles the lines
//!
//! \param None
//!
//! This function
//!    1. Configures 4 lines connected to LEDs as GPIO
//!    2. Sets up the GPIO pins as output
//!    3. Periodically toggles each LED one by one by toggling the GPIO line
//!
//! \return None
//
//****************************************************************************
void LEDBlinkyRoutine()
{
    //
    // Toggle the lines initially to turn off the LEDs.
    // The values driven are as required by the LEDs on the LP.
    //
    while(1)
    {
        //
        // Alternately toggle hi-low each of the GPIOs
        // to switch the corresponding LED on/off.
        //
    	sleep(1);
    	GPIO_IF_setOutputHighOnPin(LED1);
        sleep(1);
        GPIO_IF_setOutputLowOnPin(LED1);
        sleep(1);
        GPIO_IF_setOutputHighOnPin(LED2);
        sleep(1);
        GPIO_IF_setOutputLowOnPin(LED2);
        sleep(1);
        GPIO_IF_setOutputHighOnPin(LED3);
        sleep(1);
        GPIO_IF_setOutputLowOnPin(LED3);
    	sleep(1);
    	GPIO_IF_setOutputHighOnPin(LED4);
        sleep(1);
        GPIO_IF_setOutputLowOnPin(LED4);
    }
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
}

//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//! This function
//!    1. Invokes the LEDBlinkyTask
//!
//! \return None.
//
//****************************************************************************
int error;
int
main()
{
//	uint8_t pinStatus;
    /*------------------------------------------------------------------------------------------------------*/
	/*  1. | Initialize Board */
    /*------------------------------------------------------------------------------------------------------*/
    BoardInit();
    PinMuxConfig();

	/*------------------------------------------------------------------------------------------------------*/
	/*  2. | Program the MSP430 */
	/*------------------------------------------------------------------------------------------------------*/
    error = updateFirmware();

    if (error != 0)
    {
    	printf("Error %d\n",error);
    }
    else
    {
    	printf("MSP430 Programmed Successfully\n",error);
    }
    //
    // Start the LEDBlinkyRoutine
    //
    GPIO_IF_setAsOutputPin(LED1);
    GPIO_IF_setAsOutputPin(LED2);
    GPIO_IF_setAsOutputPin(LED3);
    GPIO_IF_setAsOutputPin(LED4);

    GPIO_IF_setOutputLowOnPin(LED1);
    GPIO_IF_setOutputLowOnPin(LED2);
    GPIO_IF_setOutputLowOnPin(LED3);
    GPIO_IF_setOutputLowOnPin(LED4);

	LEDBlinkyRoutine();
    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
