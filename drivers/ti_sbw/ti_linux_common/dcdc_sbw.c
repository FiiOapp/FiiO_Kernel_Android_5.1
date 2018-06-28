//****************************************************************************
//
// dcdc_sbw.c -
//
// Copyright (c) 2014 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//****************************************************************************

// #include "msp430x22x4_image.h"
/* #include "msp430x20x3_image.h" */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/iomux.h>

#include "dcdc_sbw.h"
#include "gpio_if.h"

#if defined (CONFIG_LIDA_MACH_X5)
    #include "msp430g2443_image.h"
#elif defined (CONFIG_LIDA_MACH_X7II)
    #include "msp430g2443_image_X7II.h"
#endif

#if 1
    #define MSG(fmt, args...) printk("SBW " fmt, ## args)
#else

    #define MSG(fmt, arg...) do {} while (0)
#endif


#define SC_ERR_NONE                 0
#define SC_ERR_ET_DCDC_IDCODE       1
#define SC_ERR_ET_DCDC_DEVID        2
#define SC_ERR_ET_DCDC_CPU_STATE    3
#define SC_ERR_ET_DCDC_FLASH_VERI   4
#define SC_INFO_ERASED              5
#define SC_INFO_STORED              6

#define    SBWDATO          RESET_PIN
#define    SBWDATI          RESET_PIN
#define    SBWCLK           TEST_PIN

// NOP
#define _NOP() __asm("    nop")
/* #define   SBW_DELAY udelay(1); */
#define   SBW_DELAY ;
/* #define   SBW_DELAY loop(15); //11  */
#define   SBW_IDELAY ndelay(100);

// Low level macros for toggling pins
#define    ClrTST()    GPIO_IF_setOutputLowOnPin(TEST_PIN)
#define    SetTST()    GPIO_IF_setOutputHighOnPin(TEST_PIN)
#define    ClrRST()    GPIO_IF_setOutputLowOnPin(RESET_PIN)
#define    SetRST()    GPIO_IF_setOutputHighOnPin(RESET_PIN)

#define    SwitchInput()    GPIO_IF_setAsInputPin(RESET_PIN)
#define    SwitchOutput()   GPIO_IF_setAsOutputPin(RESET_PIN)

#define    GetTCLK()        GPIO_IF_getPinStatus(SBWDATO) == 0 ? 0 : 1

// SBW macros
#define   SetSBWTDIO()    GPIO_IF_setOutputHighOnPin(SBWDATO)
#define   ClrSBWTDIO()    GPIO_IF_setOutputLowOnPin(SBWDATO)
#define   SetSBWTCK()     GPIO_IF_setOutputHighOnPin(SBWCLK)
#define   ClrSBWTCK()     GPIO_IF_setOutputLowOnPin(SBWCLK)

#define   GetTDO()        GPIO_IF_getPinStatus(SBWDATO) == 0 ? 0 : 1

#if 0
#define   TMSH    SetSBWTDIO(); SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTCK();
#define   TMSL    ClrSBWTDIO(); SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTCK();
#define   TMSLDH  ClrSBWTDIO(); SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTDIO(); SetSBWTCK(); // TMS = 0, then TCLK(TDI) immediately = 1

#define   TDIH    SetSBWTDIO();  SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTCK();
#define   TDIL    ClrSBWTDIO();  SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTCK();

#define   TDOsbw  SwitchInput(); SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTCK(); SwitchOutput();
#define   TDO_RD  SwitchInput(); SBW_DELAY; ClrSBWTCK(); SBW_DELAY; tdo_bit = GetTDO();  SetSBWTCK(); SwitchOutput();
/* #define   TDO_RD  SwitchInput(); SBW_IDELAY; ClrSBWTCK(); SBW_RDELAY; tdo_bit = GetTDO(); SBW_DELAY; SetSBWTCK(); SwitchOutput(); */
#else
#define   TMSH    SetSBWTDIO();  SBW_DELAY;ClrSBWTCK(); SBW_DELAY; SetSBWTCK();
#define   TMSL    ClrSBWTDIO();  SBW_DELAY;ClrSBWTCK(); SBW_DELAY; SetSBWTCK();
#define   TMSLDH  ClrSBWTDIO();  SBW_DELAY;ClrSBWTCK(); SBW_DELAY; SetSBWTDIO(); SetSBWTCK(); // TMS = 0, then TCLK(TDI) immediately = 1

#define   TDIH    SetSBWTDIO();  SBW_DELAY;ClrSBWTCK();SBW_DELAY;SetSBWTCK();SwitchInput();
#define   TDIL    ClrSBWTDIO();  SBW_DELAY;ClrSBWTCK();SBW_DELAY;SetSBWTCK();SwitchInput();

#define   TDOsbw  SBW_DELAY;ClrSBWTCK();  SBW_DELAY;SetSBWTCK();
#define   TDO_RD  SBW_DELAY;ClrSBWTCK();  SBW_DELAY;tdo_bit = GetTDO(); SetSBWTCK();
#endif


// Constants for data formats, dedicated addresses
#define F_BYTE                     8
#define F_WORD                     16
#define V_RESET                    0xFFFE

// Bypass instruction
#define IR_BYPASS                  0xFF   // 0xFF
// Instructions for the JTAG data register
#define IR_DATA_16BIT              0x82   // 0x41
#define IR_CNTRL_SIG_16BIT         0xC8   // 0x13 original values
// Instructions for the JTAG address register
#define IR_ADDR_16BIT              0xC1   // 0x83
#define IR_ADDR_CAPTURE            0x21   // 0x84
#define IR_DATA_TO_ADDR            0xA1   // 0x85
// Instructions for the JTAG control signal register
#define IR_CNTRL_SIG_16BIT         0xC8   // 0x13 original values
#define IR_CNTRL_SIG_CAPTURE       0x28   // 0x14
#define IR_CNTRL_SIG_RELEASE       0xA8   // 0x15
// Instructions for the JTAG PSA mode
#define IR_DATA_PSA                0x22   // 0x44
#define IR_SHIFT_OUT_PSA           0x62   // 0x46

// JTAG identification value for all existing Flash-based MSP430 devices
#define JTAG_ID                    0x89

// Flash commands
#define ERASE_GLOB                 0xA50E // main & info of ALL      mem arrays
#define ERASE_ALLMAIN              0xA50C // main        of ALL      mem arrays
#define ERASE_MASS                 0xA506 // main & info of SELECTED mem arrays
#define ERASE_MAIN                 0xA504 // main        of SELECTED mem arrays
#define ERASE_SGMT                 0xA502 // SELECTED segment

#define MAX_ENTRY_TRY 7
#define TRUE          1
#define FALSE         0

// Local globals
uint8_t tdo_bit;               //holds the value of TDO-bit
uint8_t TCLK_saved;  // holds the last value of TCLK before entering a JTAG sequence
uint8_t JtagId = 0;
int io=0;
int tck=0;
int crc_error=0;
struct tsDeviceFeatures {
    uint16_t Id;
    bool TestPin;
    bool CpuX;
    bool DataQuick;
    bool FastFlash;
    bool EnhVerify;
    bool JTAG;
    bool SpyBiWire;
    uint16_t RamStart;
    uint16_t RamEnd;
    uint16_t MainStart;
};

static const struct tsDeviceFeatures sDeviceFeatures[] =
{
//                      TestPin      DataQuick      EnhVerify     SpyBiWire        RamEnd
//                Id       |    CpuX     |  FastFlash  |    JTAG     |    RamStart    |   MainStart
//                 |       |      |      |      |      |     |       |        |       |       |
/* G2xx3 */     { 0x2553, TRUE,  FALSE, TRUE , TRUE,  FALSE, TRUE , TRUE  , 0x0200, 0x02FF, 0xFC00 }, // MSP430G2553
/* G2433 */     { 0x2553, TRUE,  FALSE, TRUE , TRUE,  FALSE,  TRUE , TRUE  , 0x0200, 0x03FF, 0xE000 }, // MSP430G2433
/* F11x(1)(A)*/ { 0xF112, TRUE,  FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xF000 }, // MSP430F1121A
/* F11x2 */     { 0x1132, TRUE,  FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430F1132
/* F12x(A) */   { 0xF123, TRUE,  FALSE, FALSE, FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430F123
/* F12x2 */     { 0x1232, TRUE,  FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430F1232
/* F13x
   F14x  */     { 0xF149, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x09FF, 0x1100 }, // MSP430F149
/* F15x
   F16x  */     { 0xF169, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x09FF, 0x1100 }, // MSP430F169
/* F161x */     { 0xF16C, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x1100, 0x24FF, 0x8000 }, // MSP430F1610
/* F20xx */     { 0xF201, TRUE,  FALSE, TRUE , TRUE,  FALSE, TRUE , TRUE  , 0x0200, 0x027F, 0xF800 }, // MSP430F2013
/* F21x1
   F21x2 */     { 0xF213, TRUE,  FALSE, TRUE , TRUE,  FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430F2131
/* F22x2
   F22x4 */     { 0xF227, TRUE,  FALSE, TRUE , TRUE,  TRUE,  TRUE , TRUE  , 0x0200, 0x05FF, 0x8000 }, // MSP430F2274
/* F23x0 */     { 0xF237, TRUE,  FALSE, TRUE , TRUE,  TRUE,  TRUE , FALSE , 0x0200, 0x09FF, 0x8000 }, // MSP430F2370
/* F23x
   F24x
   F24x1
   F2410 */     { 0xF249, FALSE, FALSE, TRUE , TRUE,  TRUE,  TRUE , FALSE , 0x0200, 0x09FF, 0x1100 }, // MSP430F249
/* F241x
   F261x */     { 0xF26F, FALSE, TRUE,  TRUE , TRUE,  TRUE,  TRUE , FALSE , 0x1100, 0x20FF, 0x2100 }, // MSP430F2619
/* F41x */      { 0xF413, FALSE, FALSE, FALSE, FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430F413
/* F42x(x) */   { 0xF427, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430FW427
/* F43x 80p */  { 0xF437, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x05FF, 0xA000 }, // MSP430F437
/* FG43x */     { 0xF439, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x09FF, 0x1100 }, // MSP430FG439
/* F44x
   F43x 100p */ { 0xF449, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x09FF, 0x1100 }, // MSP430F449
/* FG461x */    { 0xF46F, FALSE, TRUE,  TRUE , TRUE,  TRUE,  TRUE , FALSE , 0x1100, 0x20FF, 0x2100 }, // MSP430FG4619
/* G2452 */     { 0x2452, TRUE,  FALSE, TRUE , TRUE,  FALSE, TRUE , TRUE  , 0x0200, 0x03FF, 0xC000 }, // MSP430G2452
/* GENERIC */   { 0xFFFF, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }
};

static uint32_t DeviceIdx = 0;

void delayInit();
void MsDelay(uint32_t time);
void usDelay(uint32_t time);

char get_mcu_ver(void)
{

    return get_fw_ver();

}

//*****************************************************************************
//
// Get the device index from the device description list
//
//*****************************************************************************
void
SetDevice (uint16_t wDeviceId)
{
    uint32_t device_num=(sizeof(sDeviceFeatures)/sizeof(*sDeviceFeatures));
    MSG("%s %d Id 0x%x\n" ,__func__,__LINE__ ,wDeviceId);
    for(DeviceIdx = 0; DeviceIdx < (sizeof(sDeviceFeatures)/sizeof(*sDeviceFeatures)); DeviceIdx++)
    {
        if(sDeviceFeatures[DeviceIdx].Id == wDeviceId)
        {
              MSG(" %s %d get device ID:0x%x  num:%d \n",__func__,__LINE__,wDeviceId,DeviceIdx);
              break;
        }
    }

    if(DeviceIdx>=device_num)
              MSG(" %s %d get device NULL no id 0x%x found in table \n",__func__,__LINE__,wDeviceId);

    MSG("%s %d Id 0x%x idx %d id_num%d\n" ,__func__,__LINE__ ,wDeviceId,DeviceIdx,device_num);
}

//*****************************************************************************
//
// Tristate the SBW lines
//
//*****************************************************************************
void
IO_3state()
{
    JtagId = 0;
    GPIO_IF_setAsInputPin(TEST_PIN);
    GPIO_IF_setAsInputPin(RESET_PIN);
}

//*****************************************************************************
//
// Drive the SBW signals
//
//*****************************************************************************
void
DrvSignals( void )
{
    // tristate SBW lines
    IO_3state();

    // enable SBW lines
    GPIO_IF_setAsOutputPin(TEST_PIN);
    GPIO_IF_setAsOutputPin(RESET_PIN);

    // Set the pin output to high
    GPIO_IF_setOutputHighOnPin(TEST_PIN);
    GPIO_IF_setOutputHighOnPin(RESET_PIN);
}
void Device_reset()
{

    GPIO_IF_setAsInputPin(TEST_PIN);

    GPIO_IF_setOutputHighOnPin(RESET_PIN);
    mdelay(5);
    GPIO_IF_setOutputLowOnPin(RESET_PIN);
    mdelay(5);
    GPIO_IF_setOutputHighOnPin(RESET_PIN);

    GPIO_IF_setAsInputPin(RESET_PIN);
}

//*****************************************************************************
//
// Initialize delay timer
//
//*****************************************************************************
void
delayInit()
{
}

//*****************************************************************************
//
// Get delay in milli seconds
//
//*****************************************************************************
void MsDelay(uint32_t time)
{
    if(time>=8)
		/* msleep(time); */
	    mdelay(time);
    else
	    mdelay(time);
}

//*****************************************************************************
//
// Get delay in micro seconds
//
//*****************************************************************************
void usDelay(uint32_t time)
{
	udelay(time);
}

void TMSH_TDIH(void)
{
    TMSH 
    TDIH  
    TDOsbw
}

void TMSL_TDIH(void)
{
    TMSL
    TDIH
    TDOsbw
}

void TMSL_TDIH_TDOrd(void)
{
    TMSL  TDIH  TDO_RD
}

void TMSL_TDIL_TDOrd(void)
{
    TMSL  TDIL  TDO_RD
}

void TMSH_TDIH_TDOrd(void)
{
    TMSH  TDIH  TDO_RD
}

void TMSH_TDIL_TDOrd(void)
{
    TMSH  TDIL  TDO_RD
}

void TMSL_TDIL(void)
{
    TMSL  TDIL  TDOsbw
}

void TMSH_TDIL(void)
{
    TMSH  TDIL  TDOsbw
}

//
// Enters with TCLK_saved and exits with TCLK = 0
//
void ClrTCLK(void)
{
    if (TCLK_saved)
    {
        TMSLDH
    }
    else
    {
        TMSL
    }

    ClrSBWTDIO();

    TDIL TDOsbw    //ExitTCLK
    TCLK_saved = 0;
}

//
// Enters with TCLK_saved and exits with TCLK = 1
//
void SetTCLK(void)
{
   if (TCLK_saved)
   {
        TMSLDH
   }
   else
   {
        TMSL
   }

   SetSBWTDIO();

   TDIH TDOsbw    //ExitTCLK
   TCLK_saved = 1;
}

//*****************************************************************************
//
// Reset SBW TAP controller
//
//*****************************************************************************
void ResetTAP(void)
{
    uint32_t i;
    // Now fuse is checked, Reset JTAG FSM
    for (i = 6; i > 0; i--)      // 6 is nominal
    {
        TMSH_TDIH();
    }
    // JTAG FSM is now in Test-Logic-Reset
    TMSL_TDIH();                 // now in Run/Test Idle

    // Fuse check
    TMSH_TDIH();
    TMSL_TDIH();
    TMSH_TDIH();
    TMSL_TDIH();
    TMSH_TDIH();
    // In every TDI slot a TCK for the JTAG machine is generated.
    // Thus we need to get TAP in Run/Test Idle state back again.
    TMSH_TDIH();
    TMSL_TDIH();                // now in Run/Test Idle
}

//*****************************************************************************
//
// Shift bits
//
//*****************************************************************************
uint32_t Shift( uint32_t Format, uint32_t Data)
{
   uint32_t TDOword = 0x0000;
   uint32_t MSB = 0x0000;
   uint32_t i;

   if(io) gpio_set_value(RK30_PIN3_PD4,GPIO_HIGH);
   (Format == F_WORD) ? (MSB = 0x8000) : (MSB = 0x80);
   for (i = Format; i > 0; i--)
   {
        if (i == 1)                     // last bit requires TMS=1; TDO one bit before TDI
        {
          ((Data & MSB) == 0) ? TMSH_TDIL_TDOrd() : TMSH_TDIH_TDOrd();
        }
        else
        {
          ((Data & MSB) == 0) ? TMSL_TDIL_TDOrd() : TMSL_TDIH_TDOrd();
        }
        Data <<= 1;
        if (tdo_bit)
            TDOword++;
        if (i > 1)
            TDOword <<= 1;               // TDO could be any port pin
   }
    if(io) gpio_set_value(RK30_PIN3_PD4,GPIO_LOW);
   TMSH_TDIH();                         // update IR
   if (TCLK_saved)
   {
        TMSL_TDIH();
   }
   else
   {
        TMSL_TDIL();
   }

   return(TDOword);
}

//*****************************************************************************
//
// IR scan
//
//*****************************************************************************
uint32_t IR_Shift(uint8_t instruction)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved)
    {
        TMSH_TDIH();
    }
    else
    {
        TMSH_TDIL();
    }
    // JTAG FSM state = Select DR-Scan
    TMSH_TDIH();

    // JTAG FSM state = Select IR-Scan
    TMSL_TDIH();
    // JTAG FSM state = Capture-IR
    TMSL_TDIH();
    // JTAG FSM state = Shift-IR, Shift in TDI (8-bit)
    return(Shift(F_BYTE, instruction));
    // JTAG FSM state = Run-Test/Idle
}

//*****************************************************************************
//
// 16 bit DR scan
//
//*****************************************************************************
uint16_t DR_Shift16(uint16_t data)
{
  
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved)
    {
        TMSH_TDIH();
    }
    else
    {
        TMSH_TDIL();
    }
    // JTAG FSM state = Select DR-Scan
    TMSL_TDIH();
    // JTAG FSM state = Capture-DR
    TMSL_TDIH();    

    // JTAG FSM state = Shift-DR, Shift in TDI (16-bit)
    return(Shift(F_WORD, data));
    // JTAG FSM state = Run-Test/Idle
}

//*****************************************************************************
//
// Start SBW and get Jtag Id
//
//*****************************************************************************
void StartSBW(void)
{
   // drive JTAG/TEST signals
    {
      DrvSignals();
      MsDelay(10);             // delay 10ms
    }
    // added by FB  
    ClrTST();
    usDelay(800);              // delay min 800us - clr SBW controller
    SetTST();
    usDelay(50); 

    // SpyBiWire entry sequence      
    // Reset Test logic
    ClrSBWTDIO();                   // put device in normal operation: Reset = 0
    ClrSBWTCK();                    // TEST pin = 0
    MsDelay(1);                     // wait 1ms (minimum: 100us)

    // SpyBiWire entry sequence
    SetSBWTDIO();                   // Reset = 1
    SetSBWTCK();                    // TEST pin = 1
                                    // initial 1 SBWCLKs to enter sbw-mode
    ClrSBWTCK();
    SetSBWTCK();

    // SpyBiWire mode is active now
    ResetTAP();  // reset TAP state machine -> Run-Test/Idle

    //TRIGGER;
    JtagId = (uint8_t)IR_Shift(IR_BYPASS);
}

//*****************************************************************************
//
// Issue SBW scan
//
//*****************************************************************************
uint16_t SetInstrFetch(void)
{
    uint16_t i;

    IR_Shift(IR_CNTRL_SIG_CAPTURE);

    // Wait until CPU is in instr. fetch state, timeout after limited attempts
    for (i = 50; i > 0; i--)
    {
        if (DR_Shift16(0x0000) & 0x0080)
        {
            return SC_ERR_NONE;
        }
        ClrTCLK();
        SetTCLK();
    }
    return SC_ERR_ET_DCDC_CPU_STATE;
}

//*****************************************************************************
//
// Halt CPU
//
//*****************************************************************************
void HaltCPU(void)
{
    SetInstrFetch();              // Set CPU into instruction fetch mode

    IR_Shift(IR_DATA_16BIT);
    DR_Shift16(0x3FFF);           // Send JMP $ instruction
    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2409);           // Set JTAG_HALT bit
    SetTCLK();
}

void TCLKstrobes(uint16_t Amount) // enters with TCLK_saved and exits with TCLK = 1
{
   uint16_t i;

   if (TCLK_saved)
   {
        TMSLDH
   }                         // TDI = 1 with rising sbwclk
   else
   {
        TMSL
   }

    // This implementation has 30 body cycles! -> 400kHz
    // DO NOT MODIFY IT !

   for (i = Amount; i > 0; i--)
   {
        ClrSBWTDIO();
        /* udelay(1); */
        SetSBWTDIO();
        /* udelay(1); */

   }
   TDIH TDOsbw    //ExitTCLK
   TCLK_saved = 1;
}

//*****************************************************************************
//
// Release CPU from halted state
//
//*****************************************************************************
void ReleaseCPU(void)
{
    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2401);           // Clear the HALT_JTAG bit
    IR_Shift(IR_ADDR_CAPTURE);
    SetTCLK();
}

//*****************************************************************************
//
// Write blocks of flash
//
//*****************************************************************************
void WriteFLASH(uint16_t StartAddr, uint16_t Length, const uint16_t *DataArray)
{
    uint16_t i;                     // Loop counter
    uint16_t addr = StartAddr;      // Address counter
    uint16_t FCTL3_val = 0xA500;    // ok for all devices; if Info-Seg. A on F2xxx should not be programmed
//  word FCTL3_val = 0xA540;    // only if Info-Seg. A on F2xxx should be programmed
    tck=35;

    HaltCPU();

    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2408);         // Set RW to write
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x0128);         // FCTL1 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(0xA540);         // Enable FLASH write
    SetTCLK();

    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012A);         // FCTL2 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(0xA540);         // Select MCLK as source, DIV=1
    SetTCLK();

    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012C);         // FCTL3 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(FCTL3_val);      // Clear FCTL3; F2xxx: Unlock Info-Seg.
                                // A by toggling LOCKA-Bit if required,
    SetTCLK();

    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);

    for (i = 0; i < Length; i++, addr += 2)
    {
        DR_Shift16(0x2408);             // Set RW to write
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(addr);               // Set address
        IR_Shift(IR_DATA_TO_ADDR);
    gpio_set_value(RK30_PIN3_PD4,GPIO_HIGH);
        DR_Shift16((uint16_t)DataArray[i]);       // Set data
    gpio_set_value(RK30_PIN3_PD4,GPIO_LOW);
        SetTCLK();
        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2409);             // Set RW to read

        //zzdts
        /* TCLKstrobes(35);        // Provide TCLKs, min. 33 for F149 and F449 */
        TCLKstrobes(tck);        // Provide TCLKs, min. 33 for F149 and F449

                                // F2xxx: 29 are ok
    }

    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2408);         // Set RW to write
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x0128);         // FCTL1 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(0xA500);         // Disable FLASH write
    SetTCLK();

    // set LOCK-Bits again
    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012C);         // FCTL3 address
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(FCTL3_val);      // Lock Inf-Seg. A by toggling LOCKA and set LOCK again
    SetTCLK();

    ReleaseCPU();
}

//*****************************************************************************
//
// Release SBW
//
//*****************************************************************************

void StopSBW(void)
{
    // release JTAG/TEST signals
    IO_3state();
    MsDelay(10);             // delay 10ms
}

//*****************************************************************************
//
// Read CPU memory
//
//*****************************************************************************
uint16_t ReadMem(uint16_t Format, uint16_t Addr)
{
    uint16_t TDOword;

    HaltCPU();

    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    if  (Format == F_WORD)
    {
        DR_Shift16(0x2409);         // Set word read
    }
    else
    {
        DR_Shift16(0x2419);         // Set byte read
    }
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(Addr);               // Set address
    IR_Shift(IR_DATA_TO_ADDR);
    SetTCLK();

    ClrTCLK();
    TDOword = DR_Shift16(0x0000);   // Shift out 16 bits

    ReleaseCPU();
    return(Format == F_WORD ? TDOword : TDOword & 0x00FF);
}

//*****************************************************************************
//
// Write CPU memory
//
//*****************************************************************************
void WriteMem(uint16_t Format, uint16_t Addr, uint16_t Data)
{
    HaltCPU();

    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    if  (Format == F_WORD)
    {
        DR_Shift16(0x2408);     // Set word write
    }
    else
    {
        DR_Shift16(0x2418);     // Set byte write
    }
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(Addr);           // Set addr
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(Data);           // Shift in 16 bits
    SetTCLK();

    ReleaseCPU();
}

int ExecutePOR(void)
{
    uint16_t JtagVersion;

    // Perform Reset
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2C01);                 // Apply Reset
    DR_Shift16(0x2401);                 // Remove Reset

    ClrTCLK();
    SetTCLK();
    ClrTCLK();
    SetTCLK();
    ClrTCLK();
    JtagVersion = IR_Shift(IR_ADDR_CAPTURE); // read JTAG ID, checked at function end
    SetTCLK();

    WriteMem(F_WORD, 0x0120, 0x5A80);   // Disable Watchdog on target device

    if (JtagVersion != JTAG_ID)
    {
        return SC_ERR_ET_DCDC_IDCODE;
    }

    return SC_ERR_NONE;
}

int GetDevice(void)
{
    uint16_t i;
    delayInit();

    for (i = 0; i < MAX_ENTRY_TRY; i++)
    {
      JtagId = 0;    // initialize JtagId with an invalid value
      StopSBW();    // release JTAG/TEST signals
      StartSBW();   // establish the physical connection to the JTAG interface
      if(JtagId == JTAG_ID) // break if a valid JTAG ID is being returned
      {
        break;
      }
    }


    if(i >= MAX_ENTRY_TRY)
    {
      return SC_ERR_ET_DCDC_IDCODE;
    }

    ResetTAP();                          // Reset JTAG state machine, check fuse HW

    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2401);                  // Set device into JTAG mode + read
    int ir = IR_Shift(IR_CNTRL_SIG_CAPTURE);
    if (ir != JTAG_ID)
    {
        return SC_ERR_ET_DCDC_IDCODE;
    }

    // Wait until CPU is synchronized, timeout after a limited # of attempts
    for (i = 50; i > 0; i--)
    {
        if (DR_Shift16(0x0000) & 0x0200)
        {
            uint16_t DeviceId;
    /* gpio_set_value(RK30_PIN3_PD4,GPIO_HIGH); */
    /* gpio_set_value(RK30_PIN3_PD4,GPIO_LOW); */
    /* gpio_set_value(RK30_PIN3_PD4,GPIO_HIGH); */
            DeviceId = ReadMem(F_WORD, 0x0FF0);// Get target device type
    /* gpio_set_value(RK30_PIN3_PD4,GPIO_LOW); */
                                               //(bytes are interchanged)
            DeviceId = (DeviceId << 8) + (DeviceId >> 8); // swop bytes
            //Set Device index, which is used by functions in Device.c
            SetDevice(DeviceId);
            break;
        }
        else
        {
            if (i == 1)
            {
                return SC_ERR_ET_DCDC_DEVID;      // Timeout reached, return false
            }
        }
    }
    return ExecutePOR();                     // Perform PUC, Includes
}

bool DeviceHas_FastFlash(void)
{
    return (sDeviceFeatures[DeviceIdx].FastFlash);
}

bool DeviceHas_CpuX(void)
{
    return (sDeviceFeatures[DeviceIdx].CpuX);
}

bool DeviceHas_EnhVerify(void)
{
    return (sDeviceFeatures[DeviceIdx].EnhVerify);
}

void SetPC(uint16_t Addr)
{
    SetInstrFetch();              // Set CPU into instruction fetch mode, TCLK=1

    // Load PC with address
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x3401);           // CPU has control of RW & BYTE.
    IR_Shift(IR_DATA_16BIT);
    DR_Shift16(0x4030);           // "mov #addr,PC" instruction
    ClrTCLK();
    SetTCLK();                    // F2xxx
    DR_Shift16(Addr);             // "mov #addr,PC" instruction
    ClrTCLK();
    IR_Shift(IR_ADDR_CAPTURE);
    SetTCLK();
    ClrTCLK();                    // Now the PC should be on Addr
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2401);           // JTAG has control of RW & BYTE.
}

//*****************************************************************************
//
// Erase Flash
//
//*****************************************************************************
void EraseFLASH(uint16_t EraseMode, uint16_t EraseAddr)
{
    uint16_t StrobeAmount = 4820;       // default for Segment Erase
    uint16_t i, loopcount = 1;          // erase cycle repeating for Mass Erase
    /* uint16_t FCTL3_val = 0xA500;        // ok for all devices; if Info-Seg. A on F2xxx should not be erased */
    uint16_t FCTL3_val = 0xA500;        // only if Info-Seg. A on F2xxx should be erased

    if ((EraseMode == ERASE_MASS) || (EraseMode == ERASE_MAIN))
    {
        if(DeviceHas_FastFlash())
        {
            StrobeAmount = 10600;        // Larger Flash memories require
        }
        else
        {
            StrobeAmount = 5300;        // Larger Flash memories require
            loopcount = 19;             // additional cycles for erase.
        }
    }
    HaltCPU();

    for (i = loopcount; i > 0; i--)
    {
        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2408);         // set RW to write
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x0128);         // FCTL1 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(EraseMode);      // Enable erase mode
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x012A);         // FCTL2 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(0xA540);         // MCLK is source, DIV=1
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x012C);         // FCTL3 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(FCTL3_val);      // Clear FCTL3; F2xxx: Unlock Info-Seg. A by toggling LOCKA-Bit if required,
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(EraseAddr);      // Set erase address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(0x55AA);         // Dummy write to start erase
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2409);         // Set RW to read
        TCLKstrobes(StrobeAmount);  // Provide TCLKs
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2408);         // Set RW to write
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x0128);         // FCTL1 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(0xA500);         // Disable erase
        SetTCLK();
    }
    // set LOCK-Bits again
    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012C);         // FCTL3 address
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(FCTL3_val);      // Lock Inf-Seg. A by toggling LOCKA (F2xxx) and set LOCK again
    SetTCLK();

    ReleaseCPU();
}

//*****************************************************************************
//
// Verify flash
//
//*****************************************************************************
int VerifyPSA(uint16_t StartAddr, uint16_t Length, const uint16_t *DataArray)
{
    uint16_t TDOword, i;
    uint16_t POLY = 0x0805;           // Polynom value for PSA calculation
    uint16_t PSA_CRC = StartAddr-2;   // Start value for PSA calculation

    ExecutePOR();

    if(DeviceHas_EnhVerify())
    {
        SetPC(StartAddr-4);
        HaltCPU();
        ClrTCLK();
        IR_Shift(IR_DATA_16BIT);
        DR_Shift16(StartAddr-2);
    }
    else
    {
        SetPC(StartAddr-2);
        SetTCLK();
        ClrTCLK();
    }
    IR_Shift(IR_DATA_PSA);

    for (i = 0; i < Length; i++)
    {
        // Calculate the PSA (Pseudo Signature Analysis) value
        if ((PSA_CRC & 0x8000) == 0x8000)
        {
            PSA_CRC ^= POLY;
            PSA_CRC <<= 1;
            PSA_CRC |= 0x0001;
        }
        else
        {
            PSA_CRC <<= 1;
        }
        // if pointer is 0 then use erase check mask, otherwise data
        if (DataArray == 0)
        {
            PSA_CRC ^= 0xFFFF;
        }
        else
        {
            PSA_CRC ^= DataArray[i];
        }
        // Clock through the PSA
        SetTCLK();

        TMSH_TDIH();
        TMSL_TDIH();
        TMSL_TDIH();
        TMSH_TDIH();
        TMSH_TDIH();
        TMSL_TDIH();

        ClrTCLK();
    }
    IR_Shift(IR_SHIFT_OUT_PSA);
    /* gpio_set_value(RK30_PIN3_PD4,GPIO_HIGH); */
    io=0;
    TDOword = DR_Shift16(0x0000);   // Read out the PSA value
    io=0;
    /* gpio_set_value(RK30_PIN3_PD4,GPIO_LOW); */
    SetTCLK();

    if(DeviceHas_EnhVerify())
    {
       ReleaseCPU();
    }

    ExecutePOR();

    if(TDOword != PSA_CRC)
        crc_error+=1;
    /* MSG(" TDOword 0x%04X PSA_CRC 0x%X \n",TDOword,PSA_CRC); */
    /* WARN_ON(1); */
    return((TDOword == PSA_CRC) ? 0 : -1);
}
//*****************************************************************************
//
// Perform blank check
//
//*****************************************************************************
int EraseCheck(uint16_t StartAddr, uint16_t Length)
{
    return (VerifyPSA(StartAddr, Length, 0));
}

void ReleaseDevice(uint16_t Addr)
{
    if (Addr == V_RESET)
    {
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2C01);         // Perform a reset
        DR_Shift16(0x2401);
    }
    else
    {
        SetPC(Addr);                // Set target CPU's PC
    }
    IR_Shift(IR_CNTRL_SIG_RELEASE);
}

int VerifyMem(uint16_t StartAddr, uint16_t Length, const uint16_t *DataArray)
{
    return (VerifyPSA(StartAddr, Length, DataArray));
}

#if 0
int WriteFLASHallSections(const uint16_t *data, const uint32_t *address, const uint32_t *length_of_sections, const uint32_t sections)
{
    int i, init = 1;
    int k;

    for(i = 0; i < sections; i++)
    {
        // Write/Verify(PSA) one FLASH section
        WriteFLASH((uint16_t)address[i], (uint16_t)length_of_sections[i], &data[init-1]);
        for(k=0;k<=10;k++){
            if (VerifyMem(address[i], length_of_sections[i], &data[init-1]))
            {
                /* MSG("WriteFLASH VerifyMem error\n"); */
                /* return SC_ERR_ET_DCDC_FLASH_VERI; */
            }
        }
        /* MSG("WriteFLASH VerifyMem ok\n"); */
        init += length_of_sections[i];      
    }

    return SC_ERR_NONE;
}
#else
#define WRITE_VERIFY_LENGTH (0x00000010)
#define RETRIES             (300)

int WriteFLASHallSections(const uint16_t *data, const uint32_t *address, const uint32_t *length_of_sections, const uint32_t sections)
{
    int i, init = 1;
    uint32_t sectionLength;
    uint16_t sectionAddress;
    int error = SC_ERR_NONE;
    int retryCounter = RETRIES;

    for(i = 0; i < sections; i++)
    {
    	if (length_of_sections[i] > WRITE_VERIFY_LENGTH)
    	{
//    		printk("length_of_sections 0x%x\n",length_of_sections[i]);

    		sectionLength = length_of_sections[i];
    		sectionAddress = 0;
    		while(sectionLength > WRITE_VERIFY_LENGTH)
    		{
//    			printk("init 0x%x\n",init);
//        		printk("sectionAddress 0x%x\n",(sectionAddress));
//        		printk("address 0x%x\n",(address[i] + (sectionAddress*2)));
//        		printk("data_array 0x%x\n",data[init-1+sectionAddress]);

        		while(retryCounter)
            	{
        			// Write/Verify(PSA) one FLASH section
        			WriteFLASH((uint16_t)(address[i]+(sectionAddress*2)),
        					(uint16_t)WRITE_VERIFY_LENGTH,
        					&data[init-1+sectionAddress]);
        			if (VerifyMem((address[i]+(sectionAddress*2)), WRITE_VERIFY_LENGTH,
        					&data[init-1+sectionAddress]))
        			{
//        				printk("1.Retry #%d\n",retryCounter);
        				retryCounter--;
        				error = SC_ERR_ET_DCDC_FLASH_VERI;
        			}
        			else
        			{
            			retryCounter = 0;
            			error = SC_ERR_NONE;
        			}
            	}

                if (error != SC_ERR_NONE)
                {
                   MSG("1. Retries exceed!!\n");
                   MSG("init 0x%x\n",init);
                   MSG("sectionAddress 0x%x\n",(sectionAddress));
                   MSG("address 0x%x\n",(address[i] + (sectionAddress*2)));
                   MSG("data_array 0x%x\n",data[init-1+sectionAddress]);
                    /* exit(1); */
                    return error;
                }
                sectionLength -= WRITE_VERIFY_LENGTH;
                sectionAddress += WRITE_VERIFY_LENGTH;
                retryCounter = RETRIES;
    		}

    		retryCounter = RETRIES;
    		while(retryCounter)
        	{
    			// Write/Verify(PSA) one FLASH section
    			WriteFLASH((uint16_t)(address[i]+(sectionAddress*2)),
    					(uint16_t)sectionLength, &data[init-1+sectionAddress]);
    			if (VerifyMem(address[i]+(sectionAddress*2), sectionLength,
    					&data[init-1+sectionAddress]))
    			{
//    				printk("2.Retry #%d\n",retryCounter);
    				retryCounter--;
    				error = SC_ERR_ET_DCDC_FLASH_VERI;
    			}
    			else
    			{
        			retryCounter = 0;
        			error = SC_ERR_NONE;
    			}
        	}
            if (error != SC_ERR_NONE)
            {
            	MSG("2. Retries exceed!!\n");
                /* exit(1); */
                return error;
            }
    	}
    	else
    	{
    		retryCounter = RETRIES;
    		while(retryCounter)
        	{
    			// Write/Verify(PSA) one FLASH section
    			WriteFLASH((uint16_t)address[i], (uint16_t)length_of_sections[i], &data[init-1]);
    			if (VerifyMem(address[i], length_of_sections[i], &data[init-1]))
    			{
//    				printk("3.Retry #%d\n",retryCounter);
    				retryCounter--;
    				error = SC_ERR_ET_DCDC_FLASH_VERI;
    			}
    			else
    			{
        			retryCounter = 0;
        			error = SC_ERR_NONE;
    			}
        	}

            if (error != SC_ERR_NONE)
            {
            	MSG("3. Retries exceed!!\n");
                /* exit(1); */
                return error;
            }
    	}
        init += length_of_sections[i];      
    }
    
    return error;
}

int VerifyFLASHallSections(const uint16_t *data, const uint32_t *address, const uint32_t *length_of_sections, const uint32_t sections)
{
    int i, init = 1;
    int error = SC_ERR_NONE;
    int retryCounter = 5;

    for(i = 0; i < sections; i++)
    {
    	while(retryCounter)
    	{
    		if (VerifyMem(address[i], length_of_sections[i], &data[init-1]))
    		{
    			printk("address 0x%x\n",address[i]);
    			printk("section length 0x%x\n",length_of_sections[i]);
    			printk("ERROR Section %d\n",i);
    			printk("Retry #%d\n",retryCounter);
    			retryCounter--;
    			error = SC_ERR_ET_DCDC_FLASH_VERI;
    		}
    		else
    		{
    			retryCounter = 0;
    			error = SC_ERR_NONE;
    		}
    	}
        init += length_of_sections[i];
		retryCounter = 5;
    }

    return error;
}
#endif

int Read_info(void)
{

    uint16_t info[32];
    int i,info_err;

     info_err= GetDevice();
    if (SC_ERR_NONE != info_err)
    {
        printk("GetdeviceERROR %d\n",info_err);
        goto exit;
    }

    info_err=SC_INFO_ERASED;

    for(i=0;i<32;i++){

        info[i] = ReadMem(F_WORD,0x10c0+i*2);
        /* info[i] = 0xFFFF; */

    }

    for(i=0;i<32;i+=8){

        MSG("info %04x %04x %04x %04x %04x %04x %04x %04x\n",info[0+i],info[1+i],info[2+i],info[3+i],info[4+i],info[5+i],info[6+i],info[7+i]);
    }

    for(i=0;i<32;i++){

        if(info[i] != 0xFFFF)
            info_err=SC_INFO_STORED;
    }


    MSG(" msp430_fflash  info_err  %s \n",info_err==SC_INFO_ERASED? "info erased":"info not erased");


exit:

    ReleaseDevice(V_RESET);               // Perform Reset, release CPU from JTAG control

    // Tristate SBW lines
    IO_3state();

    Device_reset();

    return info_err;
}
//*****************************************************************************
//
// Flash DCDC MCU
//
//*****************************************************************************
int
Program_DCDC_MSP(void)
{
    int count=10;
    // Get the MSP device type
    int error = GetDevice();
    uint16_t info[32];
    uint16_t flash[32];
    int i,info_err=1;
    crc_error=0;
    MSG(" [%s] %d \n",__func__,__LINE__);
    if (SC_ERR_NONE != error)
    {
    	printk("GetdeviceERROR %d\n",error);
        goto exit;
        /* return error; */
    }

    MSG(" [%s] %d \n",__func__,__LINE__);
    // Erase Flash
    do{
        if (DeviceHas_CpuX())
        {
            /* EraseFLASH(ERASE_GLOB, 0xFE00);     // Global-Erase Flash */
            /* printk("EraseFLASH ERASE_GLOB\n"); */
        }                                       // (for all devices with CPU-X)
        else
        {
            /* EraseFLASH(ERASE_ALLMAIN, 0xE000);     // Mass-Erase Flash (all types) */
            printk("EraseFLASH ERASE_ALLMAIN\n");
            /* EraseFLASH(ERASE_MASS, 0xFE00);     // Mass-Erase Flash (all types) */
            /* EraseFLASH(ERASE_SGMT, 0x1000);     // Segment-Erase Flash (all types) */
            EraseFLASH(ERASE_ALLMAIN, 0xE000);     // Mass-Erase Flash (all types)
            /* EraseFLASH(ERASE_SGMT, 0x1000);     // Segment-Erase Flash (all types) */
        }

        // Perform Blank check
        error = EraseCheck(0xE000, 0x1000);        // Check main memory erasure (Fxx2..9)
    }while(error && count-->0);


    if (SC_ERR_NONE != error)
    {
        goto exit;
        /* return error; */
    }

    for(i=0;i<32;i++){

       info[i] = ReadMem(F_WORD,0x10c0+i*2);// Get target device type

    }
    for(i=0;i<32;i++){

       flash[i] = ReadMem(F_WORD,0xE000+i*2);// Get target device type

    }


    for(i=0;i<32;i+=8){

        MSG("info %04x %04x %04x %04x %04x %04x %04x %04x\n",info[0+i],info[1+i],info[2+i],info[3+i],info[4+i],info[5+i],info[6+i],info[7+i]);
    }
    for(i=0;i<32;i+=8){

        MSG("flash %04x %04x %04x %04x %04x %04x %04x %04x\n",flash[0+i],flash[1+i],flash[2+i],flash[3+i],flash[4+i],flash[5+i],flash[6+i],flash[7+i]);
    }

    for(i=0;i<32;i++){

        if(info[i] != 0xFFFF)
            info_err=0;

    }

    MSG(" msp430_fflash  info_err  %s \n",info_err? "info erased":"info not erased");
    crc_error=0;
    MSG(" [%s] %d \n",__func__,__LINE__);
    // Write/Verify(PSA)
    /* for(tck=20;tck<=40;tck++){ */
    error = WriteFLASHallSections(&eprom[0], &eprom_address[0], &eprom_length_of_sections[0], eprom_sections);
    if (SC_ERR_NONE != error)
    {
        goto exit;
        /* return error; */
    }
    /* } */

    for(i=0;i<32;i++){

       flash[i] = ReadMem(F_WORD,0xE000+i*2);// Get target device type

    }
    for(i=0;i<32;i+=8){

        MSG("flash %04x %04x %04x %04x %04x %04x %04x %04x\n",flash[0+i],flash[1+i],flash[2+i],flash[3+i],flash[4+i],flash[5+i],flash[6+i],flash[7+i]);
    }

exit:
    MSG(" msp430_fflash  MCU FW CRC_ERROR %d\n",crc_error);
    /* MSG(" msp430_flash  [%s] %d  MCU FW CRC_ERROR %d\n",__func__,__LINE__,crc_error); */
    // Reset device
    ReleaseDevice(V_RESET);               // Perform Reset, release CPU from JTAG control

    // Tristate SBW lines
    IO_3state();

    Device_reset();

    return error;


}

//*****************************************************************************
//
// update the firmware if there is a version mismatch
//
//*****************************************************************************
int updateFirmware()
{
    int error = SC_ERR_NONE;

    // Read firmware
//dl    error = getMSPFirmwareVersion();
    /* Read_info(); */
    /* return error; */

    if (SC_ERR_NONE == error)
    {
    	error = Program_DCDC_MSP();
    }
    return error;
}
