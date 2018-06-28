/* ************************************************************************ */
/*                                                                          */
/*  DirectC         Copyright (C) Microsemi Corporation 2015                */
/*  Version 3.2     Release date  October 30, 2015                          */
/*                                                                          */
/* ************************************************************************ */
/*                                                                          */
/*  Module:         dpfrom.c                                                */
/*                                                                          */
/*  Description:    Contains initialization and data checking functions.    */
/*                                                                          */
/* ************************************************************************ */
/* ************ MICROSEMI SOC CORP. DIRECTC LICENSE AGREEMENT ***************/
/* 
PLEASE READ: BEFORE INSTALLING THIS SOFTWARE, CAREFULLY READ THE FOLLOWING 
MICROSEMI SOC CORP LICENSE AGREEMENT REGARDING THE USE OF THIS SOFTWARE. 
INSTALLING THIS SOFTWARE INDICATES THAT YOU ACCEPT AND UNDERSTAND THIS AGREEMENT 
AND WILL ABIDE BY IT. 

Note: This license agreement (“License”) only includes the following software: 
DirectC. DirectC is licensed under the following terms and conditions.

Hereinafter, Microsemi SoC Corp. shall be referred to as “Licensor” or “Author,” 
whereas the other party to this License shall be referred to as “Licensee.” Each 
party to this License shall be referred to, singularly, as a “Party,” or, 
collectively, as the “Parties.”

Permission to use, copy, modify, and/or distribute DirectC for any purpose, with
or without fee, is hereby granted by Licensor to Licensee, provided that the 
above Copyright notice and this permission notice appear in all copies, 
modifications and/or distributions of DirectC.

DIRECTC IS PROVIDED "AS IS" AND THE AUTHOR/LICENSOR DISCLAIMS ALL WARRANTIES 
WITH REGARD TO DIRECTC INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND 
FITNESS. IN NO EVENT SHALL AUTHOR/LICENSOR BE LIABLE TO LICENSEE FOR ANY DAMAGES, 
INCLUDING SPECIAL, DIRECT,INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES 
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF 
CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION 
WITH THE USE OR PERFORMANCE OF DIRECTC.

Export Control: Information furnished to Licensee may include United States 
origin technical data. Accordingly, Licensee is responsible for complying with, 
and warrants to Licensor that it will comply with, all U.S. export control laws 
and regulations, including the provisions of the Export Administration Act of 
1979 and the Export Administration Regulations promulgated thereunder, the Arms 
Export Control Act, and the sanctions laws administered by the Office of Foreign 
Assets Control including any other U.S. Government regulation applicable to the 
export, re-export, or disclosure of such controlled technical data (or the 
products thereof) to Foreign Nationals, whether within or without the U.S., 
including those employed by, or otherwise associated with, Licensee. Licensee 
shall obtain Licensor’s written consent prior to submitting any request for 
authority to export any such technical data.

ADR: Any dispute between the Parties arising from or related to this License or 
the subject matter hereof, including its validity, construction or performance 
thereunder, shall be exclusively resolved through arbitration by a mutually 
acceptable impartial and neutral arbitrator appointed by the Judicial 
Arbitration and Mediation Services (JAMS) in accordance with its rules and 
procedures. If the Parties are not able to agree on an arbitrator within 10 days 
of the date of request for mediation is served, then JAMS shall appoint an 
arbitrator. Notice of arbitration shall be served and filed with the JAMS main 
offices in Irvine, California. Each Party shall be responsible for all costs 
associated with the preparation and representation by attorneys, or any other 
persons retained thereby, to assist in connection with any such Arbitration. 
However, all costs charged by the mutually agreed upon Arbitration entity shall 
be equally shared by the Parties. The Party seeking Mediation and/or Arbitration 
as provided herein agrees that the venue for any such Mediation and Arbitration 
shall be selected by the other Party and that such venue must be Los Angeles, 
California; New York, New York; or Chicago, Illinois; whereby the applicable law 
and provisions of the Evidence Code of the State selected thereby shall be 
applicable and shall govern the validity, construction and performance of this 
License.

Governing Law: This license will be governed by the laws of the State of 
California, without regard to its conflict of law provisions.

Entire Agreement: This document constitutes the entire agreement between the 
Parties with respect to the subject matter herein and supersedes all other 
communications whether written or oral.
*/

#include "dpuser.h"
#ifdef ENABLE_G3_SUPPORT

#include "dputil.h"
#include "dpalg.h"
#include "dpG3alg.h"
#include "dpfrom.h"
#include "dpsecurity.h"
#include "dpcom.h"
#include "dpjtag.h"

DPCHAR FromRowNumber;
#ifdef FROM_SUPPORT
DPUCHAR ucFRomAddressMask;

/************************************************************************************************/
/*  FROM Action Functions                                                                       */
/************************************************************************************************/
#ifndef DISABLE_FROM_SPECIFIC_ACTIONS
void dp_erase_from_action(void)
{
    device_security_flags |= IS_ERASE_ONLY;
    device_security_flags |= IS_RESTORE_DESIGN;
    dp_erase_from();
    return;
}

void dp_program_from_action(void)
{
    dp_erase_from();
    
    /* Encryption support */
    #ifdef FROM_ENCRYPT
    if (error_code == DPE_SUCCESS)
    {
        if (dat_support_status & FROM_DAT_ENCRYPTION_BIT)
        {
            dp_enc_program_from();
        }
    }
    #endif
    #ifdef FROM_PLAIN
    /* Plain text support */
    if (error_code == DPE_SUCCESS)
    {
        if ((dat_support_status & FROM_DAT_ENCRYPTION_BIT) == 0U)
        {
            dp_program_from();
            if (error_code == DPE_SUCCESS)
            {
                dp_verify_from();
            }
        }
    }
    #endif
    return;
}

void dp_verify_from_action(void)
{
    #ifdef FROM_PLAIN    
    /* Plain text support */
    if ((dat_support_status & FROM_DAT_ENCRYPTION_BIT) == 0U)
    {
        dp_verify_from();
    }
    #endif
    return;
}

/************************************************************************************************/

/************************************************************************************************/
/* Common Functions                                                                             */
/************************************************************************************************/
void dp_erase_from(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nErase FlashROM...");
    #endif
    dp_flush_global_buf1();
    global_buf1[0] = UROW_ERASE_BITS_BYTE0;
    global_buf1[1] = UROW_ERASE_BITS_BYTE1;
    global_buf1[2] = UROW_ERASE_BITS_BYTE2;
    
    /* This is for FROM erase.  Need to get which bits are set to erase from the data file. */
    
    global_uchar1 = (DPUCHAR) dp_get_bytes(FRomAddressMask_ID,0U,1U);
    if (global_uchar1 & 0x1U)
    {
        global_buf1[1]|=0x80U;
    }
    global_buf1[2] |= (DPUCHAR)(global_uchar1 >> 1U);
    
    dp_exe_erase();
    return;
}
#endif

/************************************************************************************************/
/*  FROM Plain Text Programming Functions                                                       */
/************************************************************************************************/
#ifdef FROM_PLAIN
void dp_program_from(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nProgramming FlashROM...");
    #endif
    DataIndex=0U;
    global_uint1 = 0x80U; /* global_uchar1 could be used in place if FromAddressMaskIndex */
    
    ucFRomAddressMask = (DPUCHAR) dp_get_bytes(FRomAddressMask_ID,0U,1U);
    /* Since RowNumber could be an 8 bit variable or 16 bit, it will wrap around */
    for (FromRowNumber = 7; FromRowNumber >= 0 ;FromRowNumber--)
    {
        if (ucFRomAddressMask & global_uint1)
        {
            global_uchar1 = (DPUCHAR)FromRowNumber;
            opcode = ISC_UFROM_ADDR_SHIFT;
            IRSCAN_in();
            DRSCAN_in(0u,3u,&global_uchar1);
            goto_jtag_state(JTAG_UPDATE_DR,0u);		
            
            opcode = ISC_PROGRAM_UFROM;
            IRSCAN_in();
            dp_get_and_DRSCAN_in(FRomStream_ID, FROM_ROW_BIT_LENGTH, DataIndex);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_PROGRAM_UFROM_CYCLES);
            DataIndex = DataIndex + FROM_ROW_BIT_LENGTH;
            
            dp_poll_device();
            if (error_code != DPE_SUCCESS)
            {
                break;
            }
        }
        global_uint1>>=1;
    }
    return;
}

void dp_verify_from(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nVerifying FlashROM...");
    #endif
    dp_vnr();
    
    DataIndex=0U;
    global_uint1=0x80U; /* global_uchar1 could be used in place if FromAddressMaskIndex */
    
    ucFRomAddressMask = (DPUCHAR) dp_get_bytes(FRomAddressMask_ID,0U,1U);
    for (FromRowNumber=7; FromRowNumber >= 0 ;FromRowNumber--)
    {
        if (ucFRomAddressMask & global_uint1)
        {
            global_uchar1 = (DPUCHAR) FromRowNumber;
            opcode = ISC_UFROM_ADDR_SHIFT;
            IRSCAN_in();
            DRSCAN_in(0U,3U,&global_uchar1);
            goto_jtag_state(JTAG_UPDATE_DR,0u);
            
            
            opcode = ISC_VERIFY_UFROM;
            IRSCAN_in();
            dp_get_and_DRSCAN_in(FRomStream_ID, FROM_ROW_BIT_LENGTH, DataIndex);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_VERIFY_UFROM_CYCLES);
            dp_delay(ISC_VERIFY_UFROM_DELAY);
            
            dp_poll_device();
            if (error_code == DPE_SUCCESS)
            {
                opcode = ISC_VERIFY_UFROM;
                IRSCAN_in();
                DRSCAN_out(FROM_ROW_BIT_LENGTH,(DPUCHAR*)DPNULL,global_buf1);
                
                if ((global_buf1[0]&0x3U) != 0x3U)
                {
                    error_code = DPE_FROM_VERIFY_ERROR;
                    break;
                }
                DataIndex = DataIndex + FROM_ROW_BIT_LENGTH;
            }
            else 
            {
                FromRowNumber = -1;
            }
        }
        global_uint1>>=1;
    }
    return;
}
#endif

#ifdef FROM_ENCRYPT
/*********************** ENCRYPTION **************************/
void dp_enc_program_from(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nProgramming FlashROM...");
    #endif
    if (device_family & DUAL_KEY_BIT)
    {
        global_uchar1 = 0U;
        dp_set_aes_mode();
    }
    
    DataIndex=0U;
    global_uint1 = 0x1U; /* global_uint1 could be used in place if FromAddressMaskIndex */
    
    ucFRomAddressMask = (DPUCHAR) dp_get_bytes(FRomAddressMask_ID,0U,1U);
    
    for (FromRowNumber = 1;FromRowNumber <= 8 ;FromRowNumber++)
    {
        if (ucFRomAddressMask & global_uint1)
        {
            dp_init_aes();
            
            opcode = ISC_DESCRAMBLE_UFROM;
            IRSCAN_in();
            dp_get_and_DRSCAN_in(FRomStream_ID, FROM_ROW_BIT_LENGTH, DataIndex);
            
            goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_DESCRAMBLE_UFROM_CYCLES);
            dp_delay(ISC_DESCRAMBLE_UFROM_DELAY);
            
            DataIndex = DataIndex + FROM_ROW_BIT_LENGTH;
            
            opcode = ISC_PROGRAM_ENC_UFROM;
            IRSCAN_in();
            dp_get_and_DRSCAN_in(FRomStream_ID, FROM_ROW_BIT_LENGTH, DataIndex);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_PROGRAM_ENC_UFROM_CYCLES);
            
            DataIndex = DataIndex + FROM_ROW_BIT_LENGTH;
            dp_poll_device();
            if (error_code != DPE_SUCCESS)
            {
                break;
            }
        }
        global_uint1<<=1;
    }
    return;
}
#endif
#endif

#ifdef ENABLE_DISPLAY
void dp_read_from(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\n\r\nFlashROM Information: ");
    #endif
    dp_vnr();
    
    
    
    for (FromRowNumber=7; FromRowNumber >= 0 ;FromRowNumber--)
    {
        global_uchar1 = (DPUCHAR) FromRowNumber;
        opcode = ISC_UFROM_ADDR_SHIFT;
        IRSCAN_in();
        DRSCAN_in(0u, 3u, &global_uchar1);
        goto_jtag_state(JTAG_UPDATE_DR,0u);
        
        opcode = ISC_READ_UFROM;
        IRSCAN_in();
        DRSCAN_in(0u,FROM_ROW_BIT_LENGTH,(DPUCHAR*)DPNULL);
        goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_READ_UFROM_CYCLES);
        dp_delay(ISC_READ_UFROM_DELAY);
        
        
        DRSCAN_out(FROM_ROW_BIT_LENGTH,(DPUCHAR*)DPNULL,global_buf1);
        dp_display_text("\r\n");
        dp_display_array(global_buf1,FROM_ROW_BYTE_LENGTH,HEX);
    }
    dp_display_text("\r\n==================================================");
    
    return;
}
#endif

#endif /* End of ENABLE_G3_SUPPORT */
