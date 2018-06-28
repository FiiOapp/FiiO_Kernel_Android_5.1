/* ************************************************************************ */
/*                                                                          */
/*  DirectC         Copyright (C) Microsemi Corporation 2015                */
/*  Version 3.2     Release date  October 30, 2015                          */
/*                                                                          */
/* ************************************************************************ */
/*                                                                          */
/*  Module:         dpcore.c                                                */
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
#include "dpcore.h"
#include "dpnvm.h"
#include "dpsecurity.h"
#include "dpcom.h"
#include "dpjtag.h"


DPUINT cycle_count;
#ifdef CORE_SUPPORT
DPUCHAR bol_eol_verify;
DPUCHAR SDNumber;
DPINT RowNumber;


/************************************************************************************************/
/*  Core Action Functions                                                                       */
/************************************************************************************************/
#ifndef DISABLE_CORE_SPECIFIC_ACTIONS
void dp_erase_array_action(void)
{
    device_security_flags |= IS_ERASE_ONLY;
    dp_erase_array();
    return;
}

void dp_program_array_action(void)
{
    #ifdef CORE_ENCRYPT
    if (dat_support_status & CORE_DAT_ENCRYPTION_BIT)
    {
        dp_enc_data_authentication();
    }
    #endif
    if (error_code == DPE_SUCCESS)
    {
        dp_erase_array();
    }
    /* Array Programming */
    #ifdef CORE_ENCRYPT
    if (error_code == DPE_SUCCESS)
    {
        if (dat_support_status & CORE_DAT_ENCRYPTION_BIT)
        {
            dp_enc_program_array();
            if (error_code == DPE_SUCCESS)
            {
                bol_eol_verify = BOL;
                dp_enc_verify_array();
                if (error_code == DPE_SUCCESS)
                {
                    dp_enc_program_rlock();
                    if (error_code == DPE_SUCCESS)
                    {
                        dp_is_core_configured();
                    }
                }
            }
        }
    }
    #endif
    #ifdef CORE_PLAIN
    /* Plain text support */
    if (error_code == DPE_SUCCESS)
    {
        if ((dat_support_status & CORE_DAT_ENCRYPTION_BIT) == 0U)
        {
            dp_program_array();
            if (error_code == DPE_SUCCESS)
            {
                bol_eol_verify = BOL;
                dp_verify_array();
                if (error_code == DPE_SUCCESS)
                {
                    dp_program_rlock();
                    if (error_code == DPE_SUCCESS)
                    {
                        dp_is_core_configured();
                    }
                }
            }
        }
    }
    #endif
    
    
    #ifdef NVM_SUPPORT
    if (error_code == DPE_SUCCESS)
    {
        if (
        ((device_family & SFS_BIT) == SFS_BIT) &&  
        (dat_support_status | (NVM0_DAT_SUPPORT_BIT | NVM1_DAT_SUPPORT_BIT | NVM2_DAT_SUPPORT_BIT | NVM3_DAT_SUPPORT_BIT))
        )
        {
            if (hardware_interface == IAP_SEL)
            {
                dp_initialize_access_nvm();
            }
        }
        
        if (dat_support_status & NVM0_DAT_SUPPORT_BIT)
        {
            #ifdef NVM_ENCRYPT
            /* Encryption support */
            if (dat_support_status & NVM0_DAT_ENCRYPTION_BIT)
            {
                if (
                ((device_family & SFS_BIT) == SFS_BIT) && 
                ( (hardware_interface == GPIO_SEL) || (enable_mss_support) )
                )
                {
                    dp_enc_program_nvm_block(PRIVATE_CLIENT_PHANTOM_BLOCK);
                }
            }
            #endif
            #ifdef NVM_PLAIN
            /* Plain text support */
            if ((dat_support_status & NVM0_DAT_ENCRYPTION_BIT) == 0U)
            {
                if (
                ((device_family & SFS_BIT) == SFS_BIT) && 
                ( (hardware_interface == GPIO_SEL) || (enable_mss_support) )
                )
                {
                    dp_program_nvm_block(PRIVATE_CLIENT_PHANTOM_BLOCK);
                    if (error_code == DPE_SUCCESS)
                    {
                        dp_verify_nvm_block(PRIVATE_CLIENT_PHANTOM_BLOCK);
                    }
                }
            }
            #endif
        }
    }
    #endif
    return;
}

void dp_verify_array_action(void)
{
    /* Array verification */
    #ifdef CORE_ENCRYPT
    if (dat_support_status & CORE_DAT_ENCRYPTION_BIT)
    {
        bol_eol_verify = EOL;
        dp_enc_verify_array();
    }
    #endif
    #ifdef CORE_PLAIN
    /* Plain text support */
    if ((dat_support_status & CORE_DAT_ENCRYPTION_BIT) == 0U)
    {
        bol_eol_verify = EOL;
        dp_verify_array();
    }
    #endif
    if (error_code == DPE_SUCCESS)
    {
        dp_is_core_configured();
    }
    return;
}

#ifdef CORE_ENCRYPT
void dp_enc_data_authentication_action(void)
{
    dp_enc_data_authentication();
    return;
}
#endif
/************************************************************************************************/


/************************************************************************************************/
/* Common Functions                                                                             */
/************************************************************************************************/
void dp_erase_array(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nErase FPGA Array...");
    #endif
    
    #ifdef CORE_PLAIN
    if ((dat_support_status & CORE_DAT_ENCRYPTION_BIT) == 0u)
    {
        dp_disable_rlock();
    }
    #endif
    #ifdef CORE_ENCRYPT
    if (dat_support_status & CORE_DAT_ENCRYPTION_BIT)
    {
        dp_enc_disable_rlock();
    }
    #endif
    if (error_code == DPE_SUCCESS)
    {
        dp_flush_global_buf1();
        global_buf1[0] = UROW_ERASE_BITS_BYTE0 | CORE_ERASE_BITS_BYTE0;
        global_buf1[1] = UROW_ERASE_BITS_BYTE1 | CORE_ERASE_BITS_BYTE1;
        global_buf1[2] = UROW_ERASE_BITS_BYTE2 | CORE_ERASE_BITS_BYTE2;
        
        dp_exe_erase();
    }
    return;
}
#endif
void dp_exe_program(void)
{
    /* PROGRAM  */
    opcode = ISC_PROGRAM;
    IRSCAN_in();
    goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_PROGRAM_CYCLES);
    dp_poll_device();
    
    return;
}

void dp_exe_verify(void)
{
    /* Verify0 */
    opcode = ISC_VERIFY0;
    IRSCAN_in();
    DRSCAN_in(0u, 2u, &bol_eol_verify);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_VERIFY0_CYCLES);
    dp_delay(ISC_VERIFY0_DELAY);
    
    dp_poll_device();
    if (error_code != DPE_SUCCESS)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nVerify 0 failed\r\nRow Number : "); 
        dp_display_value(((DPULONG)RowNumber - 1u), DEC);
        #endif
        error_code = DPE_CORE_VERIFY_ERROR;
    }
    if (error_code == DPE_SUCCESS)
    {
        opcode = ISC_VERIFY0;
        IRSCAN_in();
        DRSCAN_in(0u, 2u, &bol_eol_verify);
        
        DRSCAN_out(2u, &bol_eol_verify, &global_uchar1);
        if ((global_uchar1 & 0x3U) != 0U)
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nVerify 0 failed\r\nRow Number : "); 
            dp_display_value((DPULONG)RowNumber - 1U, DEC);
            #endif
            error_code = DPE_CORE_VERIFY_ERROR;
        }
    }
    
    /* Verify1 */
    if (error_code == DPE_SUCCESS)
    {
        opcode = ISC_VERIFY1;
        IRSCAN_in();
        DRSCAN_in(0u, 2u, &bol_eol_verify);
        goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_VERIFY1_CYCLES);
        dp_delay(ISC_VERIFY1_DELAY);
        
        dp_poll_device();
        if (error_code != DPE_SUCCESS)
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nVerify 1 failed\r\nRow Number : "); 
            dp_display_value((DPULONG)RowNumber - 1U,DEC);
            #endif
            error_code = DPE_CORE_VERIFY_ERROR;
        }
    }
    
    if (error_code == DPE_SUCCESS)
    {
        opcode = ISC_VERIFY1;
        IRSCAN_in();
        DRSCAN_in(0u, 2u, &bol_eol_verify);
        
        DRSCAN_out(2u, &bol_eol_verify, &global_uchar1);
        if ((global_uchar1 & 0x3U) != 0U)
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nVerify 1 failed\r\nRow Number : "); 
            dp_display_value((DPULONG)RowNumber - 1u, DEC);
            #endif
            error_code = DPE_CORE_VERIFY_ERROR;
        }
    }
    return;
}

void dp_reset_address(void)
{
    opcode = ISC_INCREMENT;
    IRSCAN_in();
    global_uchar1 = 2u;
    DRSCAN_in(0u, 2u, &global_uchar1);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_INCREMENT_CYCLES);
    return;
}

void dp_increment_address(void)
{
    opcode = ISC_INCREMENT;
    IRSCAN_in();
    
    global_uchar1 = 3u;
    DRSCAN_in(0u, 2u, &global_uchar1);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_INCREMENT_CYCLES);
    return;
}
#ifdef ENABLE_DAS_SUPPORT
void dp_load_row_address(void)
{
    DPUCHAR Tiles;
    DPUINT TileSize[132] = {
        1U, 48U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U,
        44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 
        44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U,
        44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U,
        44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U,  1U,  1U, 44U, 44U, 44U, 44U,
        44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U,
        44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U,
        44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 
        44U, 44U, 44U, 44U, 44U ,44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U, 44U,
        44U, 44U, 44U,44U, 48U, 1U
    };
    
    DPINT LastSumOfTileRows = 0;
    DPINT SumOfTileRows = 0;
    DPUCHAR Address[24]= { 
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
    };
    DPUINT IO = 0U;
    DPUCHAR RP = 0U;
    DPUCHAR LH = 0U;
    DPUINT BitsToShift;
    DPUCHAR AddressIndex;
    /* Calculate which tile the row belongs to and what row within the tile we are programming*/
    for(Tiles = 0U; Tiles < 132U; Tiles ++)
    {
        SumOfTileRows += (DPINT) TileSize[Tiles];
        if (RowNumber < SumOfTileRows)
        {
            break;
        }
        else
        {
            LastSumOfTileRows = SumOfTileRows;
        }
    }
    if ((RowNumber - LastSumOfTileRows) >= 38)
    {
        BitsToShift = (TileSize[Tiles] - 1U) - ((DPUINT)RowNumber - (DPUINT)LastSumOfTileRows);
        IO = 0x200u;
        IO >>= BitsToShift;
        RP = (DPUCHAR)(0x20U >> BitsToShift);
        LH = (DPUCHAR)(0x20U >> BitsToShift);
        
        Address[22] |= (DPUCHAR) (IO << 6U);
        Address[23] |= (DPUCHAR) (IO >> 2U);
        Address[22] |= RP;
        Address[21] |= (DPUCHAR) (LH << 2U);
    }
    /* Setting row select bit */
    if ((RowNumber - LastSumOfTileRows) < 38)
    {
        BitsToShift = ((DPUINT)RowNumber - (DPUINT)LastSumOfTileRows) + 4u;
        AddressIndex = (DPUCHAR)(BitsToShift / 8U);
        BitsToShift -= ((DPUINT)AddressIndex * 8U);
        AddressIndex += 16U;
        Address[AddressIndex] |= (1U << BitsToShift);
    }
    /* Setting tile bit */
    AddressIndex = Tiles / 8U;
    BitsToShift = (DPUINT)Tiles - (DPUINT)AddressIndex * 8U;
    Address[AddressIndex] |= (1U << BitsToShift);
    
    /* Shift the address */
    opcode = ISC_ADDRESS_SHIFT;
    IRSCAN_in();
    DRSCAN_in(0u, 192u, Address);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_ADDRESS_SHIFT_CYCLES);
    
    return;
}
#endif

#ifdef CORE_PLAIN
/************************************************************************************************/
/* Plain text array programming functions                                                       */
/************************************************************************************************/
void dp_program_array(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nProgramming FPGA Array...");
    #endif
    dp_reset_address();
    DataIndex = 0U;
    for ( RowNumber = (DPINT)device_rows - 1; RowNumber >= 0; RowNumber-- )
    {
        #ifdef ENABLE_DISPLAY
        if ((RowNumber % 100) == 0)
        {
            dp_display_text(".");
        }
        #endif
        #ifdef ENABLE_DAS_SUPPORT
        if ((device_ID & AXXE1500X_ID_MASK) == (AXXE1500X_ID & AXXE1500X_ID_MASK))
        {
            if (device_family & DAS_BIT)
            {
                dp_load_row_address();
            }
        }
        #endif
        dp_load_row_data();
        dp_exe_program();
        if (error_code != DPE_SUCCESS)
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nFailed to program FPGA Array at row ");
            dp_display_value((DPULONG)RowNumber,DEC);
            #endif
            error_code = DPE_CORE_PROGRAM_ERROR;
            break;
        }
        dp_increment_address();
    }
    return;
}


void dp_verify_array(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nVerifying FPGA Array...");
    #endif
    
    dp_reset_address();
    DataIndex = 0U;
    for ( RowNumber = (DPINT)device_rows - 1; RowNumber >= 0; RowNumber-- )
    {
        #ifdef ENABLE_DISPLAY
        if ((RowNumber % 100) == 0)
        {
            dp_display_text(".");
        }
        #endif
        #ifdef ENABLE_DAS_SUPPORT
        if ((device_ID & AXXE1500X_ID_MASK) == (AXXE1500X_ID & AXXE1500X_ID_MASK))
        {
            if (device_family & DAS_BIT)
            {
                dp_load_row_address();
            }
        }
        #endif
        dp_load_row_data();
        dp_exe_verify();
        if (error_code != DPE_SUCCESS)
        {
            break;
        }
        dp_increment_address();
    }
    return;
}


/****************************************************************************/
/*  Address and Data Loading                                                */
/****************************************************************************/
void dp_load_row_data(void)
{
    /* Load one row of FPGA Array data  */
    opcode = ISC_DATA_SHIFT;
    IRSCAN_in();
    
    for ( SDNumber = 1u; SDNumber <= device_SD; SDNumber++ )
    {
        for ( global_ulong1 = 1u; global_ulong1 <= 8u; global_ulong1++ )
        {
            dp_get_and_DRSCAN_in(datastream_ID, ARRAY_ROW_LENGTH, DataIndex);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_DATA_SHIFT_CYCLES);
            DataIndex = DataIndex + ARRAY_ROW_LENGTH;
        }
    }
    return;
}

void dp_program_rlock(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nProgramming RLOCK...");
    #endif
    
    DataIndex = 0u;
    opcode = ISC_DATA_SHIFT;
    IRSCAN_in();
    
    for ( SDNumber = 1u; SDNumber <= device_SD; SDNumber++ )
    {
        for ( global_ulong1 = 1u; global_ulong1 <= 8u; global_ulong1++ )
        {
            dp_get_and_DRSCAN_in(rlock_ID, ARRAY_ROW_LENGTH, DataIndex);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_DATA_SHIFT_CYCLES);
            DataIndex = DataIndex + ARRAY_ROW_LENGTH;
        }
    }
    
    opcode = ISC_PROGRAM_RLOCK;
    IRSCAN_in();
    goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_PROGRAM_RLOCK_CYCLES);
    dp_poll_device();
    if (error_code != DPE_SUCCESS)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nFailed to program RLOCK ");
        #endif
        error_code = DPE_PROGRAM_RLOCK_ERROR;
    }
    
    return;
}

void dp_disable_rlock(void)
{
    
    opcode = ISC_DATA_SHIFT;
    IRSCAN_in();
    
    for ( SDNumber = 1u; SDNumber <= device_SD; SDNumber++ )
    {
        for ( global_ulong1 = 1u; global_ulong1 <= 8u; global_ulong1++ )
        {
            DRSCAN_in(0u, ARRAY_ROW_LENGTH, DPNULL);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_DATA_SHIFT_CYCLES);
        }
    }
    
    opcode = ISC_PROGRAM_RLOCK;
    IRSCAN_in();
    goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_PROGRAM_RLOCK_CYCLES);
    dp_poll_device();
    if (error_code != DPE_SUCCESS)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nFailed to disable RLOCK ");
        #endif
        error_code = DPE_PROGRAM_RLOCK_ERROR;
    }
    
    return;
}
#endif


#ifdef CORE_ENCRYPT
/************************************************************************************************/
/* Encryption array programming functions                                                       */
/************************************************************************************************/
void dp_enc_program_array(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nProgramming FPGA Array...");
    #endif
    if (device_family & DUAL_KEY_BIT)
    {
        global_uchar1 = AES_mode_value;
        dp_set_aes_mode();
    }
    dp_init_aes();
    dp_reset_address();
    
    DataIndex = 0U;
    for ( RowNumber = (DPINT)device_rows - 1; RowNumber >= 0; RowNumber-- )
    {
        #ifdef ENABLE_DISPLAY
        if ((RowNumber % 100) == 0)
        {
            dp_display_text(".");
        }
        #endif
        #ifdef ENABLE_DAS_SUPPORT
        if ((device_ID & AXXE1500X_ID_MASK) == (AXXE1500X_ID & AXXE1500X_ID_MASK))
        {
            if (device_family & DAS_BIT)
            {
                dp_load_row_address();
            }
        }
        #endif
        dp_load_enc_row_data();
        dp_exe_program();
        if (error_code != DPE_SUCCESS)
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nFailed to program FPGA Array at row ");
            dp_display_value((DPULONG)RowNumber, DEC);
            #endif
            error_code = DPE_CORE_PROGRAM_ERROR;
            break;
        }
        dp_increment_address();
    }
    return;
}


void dp_enc_verify_array(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nVerifying FPGA Array...");
    #endif
    if (device_family & DUAL_KEY_BIT)
    {
        global_uchar1 = AES_mode_value;
        dp_set_aes_mode();
    }
    dp_init_aes();
    dp_reset_address();
    
    DataIndex = 0u;
    for ( RowNumber = (DPINT)device_rows - 1; RowNumber >= 0; RowNumber-- )
    {
        #ifdef ENABLE_DISPLAY
        if ((RowNumber % 100) == 0)
        {
            dp_display_text(".");
        }
        #endif
        #ifdef ENABLE_DAS_SUPPORT
        if ((device_ID & AXXE1500X_ID_MASK) == (AXXE1500X_ID & AXXE1500X_ID_MASK))
        {
            if (device_family & DAS_BIT)
            {
                dp_load_row_address();
            }
        }
        #endif
        dp_load_enc_row_data();
        dp_exe_verify();
        if (error_code != DPE_SUCCESS)
        {
            break;
        }
        dp_increment_address();
    }
    return;
}

void dp_load_enc_row_data(void)
{
    opcode = DESCRAMBLE;
    IRSCAN_in();
    
    /* Load one row of FPGA Array data  */
    for ( SDNumber = 1u; SDNumber <= device_SD; SDNumber++ )
    {
        
        for ( global_ulong1 = 0u; global_ulong1 <= 1u; global_ulong1++ )
        {
            dp_get_and_DRSCAN_in(datastream_ID, AES_BLOCK_LENGTH, DataIndex);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,DESCRAMBLE_CYCLES);
            dp_delay(DESCRAMBLE_DELAY);
            DataIndex = DataIndex + AES_BLOCK_LENGTH;
        }
    }
    return;
}

void dp_enc_program_rlock(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nProgramming Rlock...");
    #endif
    
    DataIndex = 0u;
    opcode = DESCRAMBLE;
    IRSCAN_in();
    
    for ( SDNumber = 1u; SDNumber <= device_SD; SDNumber++ )
    {
        for ( global_ulong1 = 0u; global_ulong1 <= 1u; global_ulong1++ )
        {
            dp_get_and_DRSCAN_in(rlock_ID, AES_BLOCK_LENGTH, DataIndex);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,DESCRAMBLE_CYCLES);
            dp_delay(DESCRAMBLE_DELAY);
            DataIndex = DataIndex + AES_BLOCK_LENGTH;
        }
    }	
    
    opcode = ISC_PROGRAM_RDLC;
    IRSCAN_in();
    goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_PROGRAM_RDLC_CYCLES);
    dp_poll_device();
    
    return;
}

void dp_enc_disable_rlock(void)
{
    dp_get_data(rlockDisable_ID,1);
    if (return_bytes != 0u)
    {
        dp_init_aes();
        dp_reset_address();
        
        DataIndex = 0u;
        opcode = DESCRAMBLE;
        IRSCAN_in();
        
        for ( SDNumber = 1u; SDNumber <= device_SD; SDNumber++ )
        {
            for ( global_ulong1 = 0u; global_ulong1 <= 1u; global_ulong1++ )
            {
                dp_get_and_DRSCAN_in(rlockDisable_ID, AES_BLOCK_LENGTH, DataIndex);
                goto_jtag_state(JTAG_RUN_TEST_IDLE,DESCRAMBLE_CYCLES);
                dp_delay(DESCRAMBLE_DELAY);
                DataIndex = DataIndex + AES_BLOCK_LENGTH;
            }
        }	
        
        opcode = ISC_PROGRAM_RDLC;
        IRSCAN_in();
        goto_jtag_state(JTAG_RUN_TEST_IDLE,ISC_PROGRAM_RDLC_CYCLES);
        dp_poll_device();
    }
    
    return;
}

void dp_enc_data_authentication(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nPerforming Data Authentication...");
    #endif
    
    if (device_family & DUAL_KEY_BIT)
    {
        global_uchar1 = AES_mode_value;
        dp_set_aes_mode();
    }
    
    dp_init_aes();
    dp_reset_address();
    
    DataIndex = 0u;
    for ( RowNumber = (DPINT)device_rows - 1 ; RowNumber >= 0; RowNumber-- )
    {
        #ifdef ENABLE_DISPLAY
        if ((RowNumber % 100) == 0)
        {
            dp_display_text(".");
        }
        #endif
        
        dp_load_enc_row_data();
        dp_exe_authentication();
        if (error_code != DPE_SUCCESS)
        {
            break;
        }
    }
    return;
}

#endif /* End of CORE_ENCRYPT */

#endif /* End of CORE_SUPPORT */
#endif /* End of ENABLE_G3_SUPPORT */

