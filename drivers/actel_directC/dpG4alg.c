/* ************************************************************************ */
/*                                                                          */
/*  DirectC         Copyright (C) Microsemi Corporation 2015                */
/*  Version 3.2     Release date  October 30, 2015                          */
/*                                                                          */
/* ************************************************************************ */
/*                                                                          */
/*  Module:         dpG4alg.c                                               */
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
#ifdef ENABLE_G4_SUPPORT
#include "dputil.h"
#include "dpcom.h"
#include "dpalg.h"
#include "dpG4alg.h"
#include "dpjtag.h"

DPUCHAR pgmmode;
DPUCHAR pgmmode_flag;
DPUCHAR envm_only_flag;
DPUCHAR sec_ul;
DPUCHAR shared_buf[48]; // Maximum of 768
DPUCHAR poll_buf[17];
DPULONG poll_index;
DPUCHAR SYSREG_TEMP[4];

DPUCHAR security_queried;

/****************************************************************************
* Purpose: main entry function                                              
*  This function needs to be called from the main application function with 
*  the approppriate action code set to intiate the desired action.
****************************************************************************/
DPUCHAR dp_top_g4 (void) 
{
    dp_init_vars();
    dp_init_G4_vars();
    goto_jtag_state(JTAG_TEST_LOGIC_RESET,0u);
    dp_check_G4_action();
    if (error_code == DPE_SUCCESS)
    {
        dp_perform_G4_action();
    }
    
    return error_code;
}

void dp_init_G4_vars(void)
{
    envm_only_flag = FALSE;
    pgmmode = 0u;
    pgmmode_flag = FALSE;
    security_queried = FALSE;
    sec_ul = 0u;
    return;
}

void dp_check_G4_action(void)
{
    if ( (Action_code == DP_READ_IDCODE_ACTION_CODE) || (Action_code == DP_DEVICE_INFO_ACTION_CODE) )
    {
        #ifndef ENABLE_DISPLAY
        error_code = DPE_CODE_NOT_ENABLED;
        #endif 
    }
    else if (! (
    (Action_code == DP_ERASE_ACTION_CODE) ||
    (Action_code == DP_PROGRAM_ACTION_CODE) ||
    (Action_code == DP_VERIFY_ACTION_CODE) ||
    (Action_code == DP_ENC_DATA_AUTHENTICATION_ACTION_CODE)
    ))
    {
        error_code = DPE_ACTION_NOT_SUPPORTED;
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nInvalid action.");
        #endif
    }
    return;
}

void dp_perform_G4_action (void)
{
    #ifdef ENABLE_DISPLAY
    if (Action_code == DP_READ_IDCODE_ACTION_CODE)
    {
        dp_read_idcode_action();
        Action_done = TRUE;
    }
    else if (Action_code == DP_DEVICE_INFO_ACTION_CODE)
    {
        dp_G4M_device_info_action();
        Action_done = TRUE;
    }
    #endif
    if (Action_done == FALSE)
    {
        dp_check_image_crc();
        if (error_code == DPE_SUCCESS)
        {
            dp_check_G4_device_ID();
            if (error_code == DPE_SUCCESS)
            {
                switch (Action_code) 
                {
                    case DP_ERASE_ACTION_CODE: 
                    dp_G4M_erase_action();
                    break;         
                    case DP_PROGRAM_ACTION_CODE: 
                    dp_G4M_program_action();
                    break;
                    case DP_VERIFY_ACTION_CODE: 
                    dp_G4M_verify_action();
                    break;
                    case DP_ENC_DATA_AUTHENTICATION_ACTION_CODE: 
                    dp_G4M_enc_data_authentication_action();
                    break;
                }
            }
        }
    }
    dp_G4M_exit();
    return;
}


void dp_G4M_erase_action(void)
{
    dp_G4M_initialize();
    if (error_code == DPE_SUCCESS)
    {
        pgmmode = 0x1u;
        dp_set_mode();
        
        if (error_code == DPE_SUCCESS)
        {
            /* Global unit1 is used to hold the number of components */
            global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_NUMOFCOMPONENT_OFFSET,G4M_NUMOFCOMPONENT_BYTE_LENGTH);
            global_uint2 = global_uint1 - ((DPUINT)dp_get_bytes(Header_ID,G4M_ERASEDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) - 1u);
            
            dp_G4M_process_data(G4M_erasedatastream_ID);
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_ERASE_ERROR;
            }
            
        }
    }
    return;
}

void dp_G4M_program_action(void)
{
    dp_G4M_initialize();
    
    
    if ((error_code == DPE_SUCCESS) && (sec_ul & 0x2u))
    {
        pgmmode = 0x1u;
        dp_set_mode();
        if (error_code == DPE_SUCCESS)
        {
            global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_NUMOFCOMPONENT_OFFSET,G4M_NUMOFCOMPONENT_BYTE_LENGTH);
            global_uint2 = global_uint1 - ((DPUINT)dp_get_bytes(Header_ID,G4M_ERASEDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) - 1u);
            dp_G4M_process_data(G4M_erasedatastream_ID);
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_ERASE_ERROR;
            }
        }
    }
    if (error_code == DPE_SUCCESS)
    {
        /* Global unit1 is used to hold the number of components */
        global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
        global_uint2 = 1u;
        dp_G4M_setup_eNVM(G4M_datastream_ID);
        
        if (error_code == DPE_SUCCESS)
        {
            pgmmode = 0x1u;
            dp_set_mode();
            if (error_code == DPE_SUCCESS)
            {
                global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
                global_uint2 = 1u;
                dp_G4M_process_data(G4M_datastream_ID);
                if(error_code != DPE_SUCCESS)
                {
                    error_code = DPE_CORE_PROGRAM_ERROR;
                }
                else
                {
                    dp_G4M_post_setup_eNVM();
                }
            }
        }
    }
    
    return;
}

void dp_G4M_verify_action(void)
{
    dp_G4M_initialize();
    
    global_uint2 = 1u;
    global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
    dp_G4M_setup_eNVM(G4M_datastream_ID);
    
    if (error_code == DPE_SUCCESS)
    {
        pgmmode = 0x2u;
        dp_set_mode();
        
        if (error_code == DPE_SUCCESS)
        {
            /* Global unit1 is used to hold the number of components */
            
            if ((DPUINT)dp_get_bytes(Header_ID,G4M_VERIFYDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) != 0u)
            {
                /* Both global_unit1 and global_unit2 have DATASIZE, ENVMDATASIZE and ENVMVERIFYDATASIZE; */
                global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) +
                (DPUINT)dp_get_bytes(Header_ID,G4M_ENVMDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) +
                (DPUINT)dp_get_bytes(Header_ID,G4M_ENVMVERIFYDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
                global_uint2 = global_uint1 + 1u;
                global_uint1 += (DPUINT)dp_get_bytes(Header_ID,G4M_VERIFYDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
                dp_G4M_process_data(G4M_VerifyDataStream_ID);
            }
            else
            {
                global_uint2 = 1u;
                global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
                dp_G4M_process_data(G4M_datastream_ID);
            }
            
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_CORE_VERIFY_ERROR;
            }
            else
            {
                dp_G4M_post_setup_eNVM();
            }
        }
    }
    
    return;
}

void dp_G4M_enc_data_authentication_action(void)
{
    dp_G4M_initialize();
    if (error_code == DPE_SUCCESS)
    {
        pgmmode = 0x0u;
        dp_set_mode();
        
        if (error_code == DPE_SUCCESS)
        {
            /* Global unit1 is used to hold the number of components */
            global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
            global_uint2 = 1u;
            
            dp_G4M_process_data(G4M_datastream_ID);
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_AUTHENTICATION_FAILURE;
            }
            
        }
    }
    return;
}
void dp_G4M_check_core_status(void)
{
    
    opcode = G4M_ISC_NOOP;
    IRSCAN_out(&global_uchar1);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,1u);
    
    #ifdef ENABLE_DISPLAY
    if ((global_uchar1 & 0x4u) == 0x4u)
    {
        dp_display_text("\r\nFPGA Array is programmed and enabled.");
    }
    else
    {
        dp_display_text("\r\nFPGA Array is not enabled.");
    }
    #endif
    
}


#ifdef ENABLE_DISPLAY
void dp_G4M_device_info_action(void)
{
    dp_G4M_check_core_status();
    dp_G4M_read_design_info();
    dp_G4M_read_prog_info();
    dp_G4M_read_fsn();
    dp_G4M_read_security();
    
    return;
}

void dp_G4M_read_design_info(void)
{
    opcode = G4M_READ_DESIGN_INFO;
    IRSCAN_in();
    DRSCAN_in(0u, G4M_STATUS_REGISTER_BIT_LENGTH, (DPUCHAR*)(DPUCHAR*)DPNULL);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    opcode = G4M_READ_DESIGN_INFO;
    dp_G4M_device_poll(8u, 7u);
    if (error_code == DPE_SUCCESS)
    {
        dp_read_shared_buffer(3u);
        if (error_code == DPE_SUCCESS)
        {
            dp_display_text("\r\nDesign Name: ");
            
            
            for (global_uchar1 = 2u; global_uchar1 < 32u; global_uchar1++)
            {
                dp_display_value(shared_buf[global_uchar1],CHR);
            }
            dp_display_text("\r\nChecksum: ");
            dp_display_array(shared_buf,2u,HEX);
            dp_display_text("\r\nDesign Info: ");
            dp_display_array(shared_buf,34u,HEX);
            dp_display_text("\r\nDESIGNVER: ");
            dp_display_array(&shared_buf[32],2u,HEX);
            dp_display_text("\r\nBACKLEVEL: ");
            dp_display_array(&shared_buf[34],2u,HEX);
        }
    }
    
    return;
}

void dp_G4M_read_prog_info(void)
{
    opcode = G4M_READ_PROG_INFO;
    IRSCAN_in();
    DRSCAN_in(0u, G4M_STATUS_REGISTER_BIT_LENGTH, (DPUCHAR*)(DPUCHAR*)DPNULL);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    
    opcode = G4M_READ_PROG_INFO;
    dp_G4M_device_poll(128u, 127u);
    if (error_code == DPE_SUCCESS)
    {
        dp_display_text("\r\nPROG_INFO: ");
        dp_display_array(poll_buf,16u,HEX);
        if ((poll_buf[6] & 0x1) == 0u)
        {
            dp_display_text("\r\nVCC was programmed at 1.2V");
        }
        else
        {
            dp_display_text("\r\nVCC was programmed at 1.0v");
        }
        if  ( ((poll_buf[8] & 0x3f) != 0u) && ((poll_buf[8] & 0x3f) != 0x3fu) )
        {
            dp_display_text("\r\nAlgorithm Version: ");
            dp_display_value((poll_buf[8] & 0x3f),DEC);
        }
        
        
        global_uchar1 = ((poll_buf[8] >> 6) | (poll_buf[9] << 2)) & 0xfu;
        dp_display_text("\r\nProgrammer code: ");
        dp_display_value(global_uchar1, DEC);
        
        global_uchar1 = ((poll_buf[10] >> 1)) & 0x3fu;
        dp_display_text("\r\nSoftware version code: ");
        dp_display_value(global_uchar1, DEC);
        
        global_uchar1 = ((poll_buf[10] >> 7) | (poll_buf[11] << 1)) & 0x7u;
        dp_display_text("\r\nProgramming Software code: ");
        dp_display_value(global_uchar1, DEC);
        
        global_uchar1 = ((poll_buf[11] >> 2)) & 0x7u;
        dp_display_text("\r\nProgramming Interface Protocol code: ");
        dp_display_value(global_uchar1, DEC);
        
        global_uchar1 = ((poll_buf[11] >> 5)) & 0x7u;
        dp_display_text("\r\nProgramming File Type code: ");
        dp_display_value(global_uchar1, DEC);
        
    }
    
    return;
}

void dp_G4M_read_fsn(void)
{
    opcode = G4M_READ_FSN;
    IRSCAN_in();
    DRSCAN_in(0u, G4M_STATUS_REGISTER_BIT_LENGTH, (DPUCHAR*)(DPUCHAR*)DPNULL);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    opcode = G4M_READ_FSN;
    dp_G4M_device_poll(129u, 128u);
    dp_display_text("\r\n=====================================================================");
    dp_display_text("\r\nDSN: ");
    dp_display_array(poll_buf, 16u, HEX);
    dp_display_text("\r\n=====================================================================");
    
    return;
}
#endif

/* Checking device ID function.  ID is already read in dpalg.c */
void dp_check_G4_device_ID (void)
{
    /* DataIndex is a variable used for loading the array data but not used now.  
    * Therefore, it can be used to store the Data file ID for */
    DataIndex = dp_get_bytes(Header_ID,G4M_ID_OFFSET,G4M_ID_BYTE_LENGTH);
    
    global_ulong1 = dp_get_bytes(Header_ID,G4M_ID_MASK_OFFSET,4U);
    
    device_ID &= global_ulong1;
    DataIndex &= global_ulong1;
    /* Identifying target device and setting its parms */
    
    if ( (DataIndex & 0xfff) == MICROSEMI_ID)
    {
        if (device_ID == DataIndex )
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nActID = ");
            dp_display_value(device_ID,HEX);
            dp_display_text(" ExpID = ");
            dp_display_value(DataIndex,HEX);
            dp_display_text("\r\nDevice Rev = ");
            dp_display_value(device_rev,HEX);
            #endif
            device_family = (DPUCHAR) dp_get_bytes(Header_ID,G4M_DEVICE_FAMILY_OFFSET,G4M_DEVICE_FAMILY_BYTE_LENGTH);
        }
        else
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text(" ExpID = ");
            dp_display_value(DataIndex,HEX);
            #endif
            error_code = DPE_IDCODE_ERROR;
        }
    }
    else
    {
        error_code = DPE_IDCODE_ERROR;
    }
    
    return;
}

/* Check if system controller is ready to enter programming mode */
void dp_G4M_device_poll(DPUCHAR bits_to_shift, DPUCHAR Busy_bit)
{
    for ( poll_index = 0U; poll_index <= G4M_MAX_CONTROLLER_POLL; poll_index++ )
    {
        IRSCAN_in();
        DRSCAN_out(bits_to_shift, (DPUCHAR*)DPNULL, poll_buf);
        dp_delay(G4M_STANDARD_DELAY);
        if ( ((poll_buf[Busy_bit/8] & (1 << (Busy_bit % 8))) == 0x0u))
        {
            break;
        }
    }
    if(poll_index > G4M_MAX_CONTROLLER_POLL)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nDevice polling failed.");
        #endif
        error_code = DPE_POLL_ERROR;
    }
    
    return;
}

void dp_G4M_device_shift_and_poll(DPUCHAR bits_to_shift, DPUCHAR Busy_bit,DPUCHAR Variable_ID,DPULONG start_bit_index)
{
    for ( poll_index = 0U; poll_index <= G4M_MAX_CONTROLLER_POLL; poll_index++ )
    {
        IRSCAN_in();
        dp_get_and_DRSCAN_in_out(Variable_ID, bits_to_shift, start_bit_index, poll_buf);
        //DRSCAN_out(bits_to_shift, (DPUCHAR*)DPNULL, poll_buf);
        dp_delay(G4M_STANDARD_DELAY);
        if ( ((poll_buf[Busy_bit/8] & (1 << (Busy_bit % 8))) == 0x0u))
        {
            break;
        }
    }
    if(poll_index > G4M_MAX_CONTROLLER_POLL)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nDevice polling failed.");
        #endif
        error_code = DPE_POLL_ERROR;
    }
    
    return;
}

void dp_read_shared_buffer(DPUCHAR ucNumOfBlocks)
{
    
    dp_flush_global_buf1();
    for (global_uchar1 = 0u; global_uchar1 < ucNumOfBlocks; global_uchar1++)
    {
        global_buf1[0] = (global_uchar1 << 1u);
        opcode = G4M_READ_BUFFER;
        IRSCAN_in();
        DRSCAN_in(0u, G4M_FRAME_STATUS_BIT_LENGTH, global_buf1);
        goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
        dp_delay(G4M_STANDARD_DELAY);
        opcode = G4M_READ_BUFFER;
        dp_G4M_device_poll(129u, 128u);
        for (global_uchar2 = 0;global_uchar2 < 16u; global_uchar2++)
        {
            shared_buf[global_uchar1*16u + global_uchar2] = poll_buf[global_uchar2];
        }
    }
    
    
    
    return;
}

void dp_G4M_poll_device_ready(void)
{
    opcode = G4M_ISC_NOOP;
    for ( poll_index = 0U; poll_index <= G4M_MAX_CONTROLLER_POLL; poll_index++ )
    {
        IRSCAN_in();
        goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
        dp_delay(G4M_STANDARD_DELAY);
        DRSCAN_out(8u, (DPUCHAR*)DPNULL, poll_buf);
        
        if ((poll_buf[0] & 0x80u) == 0x0u)
        {
            break;
        }
    }
    if(poll_index > G4M_MAX_CONTROLLER_POLL)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nDevice polling failed.");
        #endif
        error_code = DPE_POLL_ERROR;
    }
    
    return;
}

void dp_set_pgm_mode(void)
{
    opcode = G4M_MODE;
    IRSCAN_in();
    DRSCAN_in(0u, G4M_STATUS_REGISTER_BIT_LENGTH, (DPUCHAR*)(DPUCHAR*)DPNULL);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    opcode = G4M_MODE;
    dp_G4M_device_poll(8u, 7u);
    
    return;
}

/****************************************************************************
* Purpose: Loads the BSR regsiter with data specified in the data file.  
*   If BSR_SAMPLE is enabled, the data is not loaded.  Instead, the last known
*   State of the IOs is maintained by stepping through DRCapture JTAG state.
****************************************************************************/
void dp_G4M_load_bsr(void)
{
    global_uint1 = (DPUINT) dp_get_bytes(G4M_Header_ID,G4M_NUMOFBSRBITS_OFFSET,G4M_NUMOFBSRBITS_BYTE_LENGTH);
    
    dp_G4M_check_core_status();
    opcode = ISC_SAMPLE;
    IRSCAN_in();
    
    #ifdef BSR_SAMPLE
    /* Capturing the last known state of the IOs is only valid if the core
    was programmed.  Otherwise, load the BSR with what is in the data file. */
    if ((global_uchar1 & 0x4u) != 0x4u)
    {
        dp_get_bytes(G4M_BsrPattern_ID,0u,1u);
        if (return_bytes)
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nWarning: FPGA array is not programmed. Loading BSR register...");
            #endif
            dp_get_and_DRSCAN_in(G4M_BsrPattern_ID, global_uint1, 0u);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,0u);
        }
    }
    else 
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nMaintaining last known IO states...");
        #endif
        goto_jtag_state(JTAG_CAPTURE_DR,0u);
        goto_jtag_state(JTAG_RUN_TEST_IDLE,0u);
    }
    #else
    dp_get_bytes(G4M_BsrPattern_ID,0u,1u);
    if (return_bytes)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nLoading BSR...");
        #endif
        dp_get_and_DRSCAN_in(G4M_BsrPattern_ID, global_uint1, 0u);
        goto_jtag_state(JTAG_RUN_TEST_IDLE,0u);
    }
    #endif
    
    return;
}

void dp_G4M_perform_isc_enable(void)
{
    pgmmode_flag = TRUE;
    dp_flush_global_buf1();
    global_buf1[0] |= (G4M_ALGO_VERSION & 0x3fu);
    global_buf1[2] |= (G4M_DIRECTC_VERSION & 0x3fu) << 1u;
    global_buf1[2] |= (DIRECTC_PROGRAMMING & 0x7u) << 7u;
    global_buf1[3] |= (DIRECTC_PROGRAMMING & 0x7u) >> 1u;
    global_buf1[3] |= (JTAG_PROGRAMMING_PROTOCOL & 0x7u) << 2u;
    
    opcode = ISC_ENABLE;
    IRSCAN_in();
    DRSCAN_in(0u, ISC_STATUS_REGISTER_BIT_LENGTH, global_buf1);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    
    opcode = ISC_ENABLE;
    dp_G4M_device_poll(32u, 31u);
    
    
    if ( (error_code != DPE_SUCCESS) || ((poll_buf[0] & 0x1u) == 1u)	)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nFailed to enter programming mode.");
        #endif
        error_code = DPE_INIT_FAILURE;
    }
    
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nISC_ENABLE_RESULT: ");
    dp_display_array(poll_buf,4u,HEX);
    
    /* Display CRCERR */
    global_uchar1 = poll_buf[0] & 0x1u;
    dp_display_text("\r\nCRCERR: ");
    dp_display_value(global_uchar1,HEX);
    
    /* Display EDCERR */
    global_uchar1 = (poll_buf[0] & 0x2u) >> 1u;
    dp_display_text("\r\nEDCERR: ");
    dp_display_value(global_uchar1,HEX);
    
    /* Display TEMPRANGE */
    global_uchar1 = (poll_buf[0] >> 2) & 0x7u;
    dp_display_text("\r\nTEMPRANGE:");
    dp_display_value(global_uchar1,HEX);
    if (global_uchar1 == 0u)
    {
        dp_display_text("\r\nTEMPRANGE: COLD");
    }
    else if (global_uchar1 == 1u)
    {
        dp_display_text("\r\nTEMPRANGE: ROOM");
    }
    if (global_uchar1 == 2u)
    {
        dp_display_text("\r\nTEMPRANGE: HOT");
    }
    
    
    /* Display VPPRANGE */
    global_uchar1 = (poll_buf[0] >> 5) & 0x7u;
    dp_display_text("\r\nVPPRANGE:");
    dp_display_value(global_uchar1,HEX);
    if (global_uchar1 == 0u)
    {
        dp_display_text("\r\nVPPRANGE: LOW");
    }
    else if (global_uchar1 == 1u)
    {
        dp_display_text("\r\nVPPRANGE: NOMINAL");
    }
    if (global_uchar1 == 2u)
    {
        dp_display_text("\r\nVPPRANGE: HIGH");
    }
    
    /* Display TEMP */
    dp_display_text("\r\nTEMP:");
    dp_display_value(poll_buf[1],HEX);
    
    /* Display VPP */
    dp_display_text("\r\nVPP:");
    dp_display_value(poll_buf[2],HEX);
    
    #endif
    
    
    return;
}
/* Enter programming mode */
void dp_G4M_initialize(void)
{
    dp_G4M_poll_device_ready();
    if (error_code == DPE_SUCCESS)
    {
        dp_set_pgm_mode();
        if (error_code == DPE_SUCCESS)
        {
            dp_G4M_read_security();
            if ((error_code == DPE_SUCCESS) && (sec_ul & 0x2))
            {
                dp_G4M_unlock_upk1();
                if (error_code == DPE_SUCCESS)
                {
                    dp_G4M_unlock_upk2();
                }
            }
            
            dp_G4M_load_bsr();
            if (error_code == DPE_SUCCESS)
            {
                dp_G4M_perform_isc_enable();
            }
        }
    }
    
    return;
}


/* Function is used to exit programming mode */
void dp_G4M_exit(void)
{
    
    if (pgmmode_flag == TRUE)
    {
        opcode = ISC_DISABLE;
        IRSCAN_in();
        goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
        dp_delay(G4M_STANDARD_DELAY);
        
        opcode = ISC_DISABLE;
        dp_G4M_device_poll(32u, 31u);
        #ifdef ENABLE_DISPLAY
        if (error_code != DPE_SUCCESS)
        {
            dp_display_text("\r\nFailed to disable programming mode.");
        }
        #endif
    }
    #ifdef ENABLE_DISPLAY
    dp_G4M_read_fsn();
    #endif
    goto_jtag_state(JTAG_TEST_LOGIC_RESET,0u);
    return;
}

void dp_set_mode(void)
{
    
    opcode = G4M_FRAME_INIT;
    IRSCAN_in();
    DRSCAN_in(0u, G4M_STATUS_REGISTER_BIT_LENGTH, &pgmmode);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);	
    dp_delay(G4M_STANDARD_DELAY);
    dp_G4M_device_poll(8u, 7u);
    #ifdef ENABLE_DISPLAY
    if (error_code != DPE_SUCCESS)
    {
        dp_display_text("r\nFailed to set programming mode.");
    }
    #endif
    
    return;
}

void dp_G4M_setup_eNVM(DPUCHAR BlockID)
{
    envm_only_flag = FALSE;
    
    DataIndex = 0u;
    for (; global_uint2 <= global_uint1; global_uint2++)
    {
        /* get the number of blocks */
        /* Global ulong1 is used to hold the number of blocks within the components */
        global_ulong1 = dp_get_bytes(G4M_NUMBER_OF_BLOCKS_ID,(DPULONG)(((global_uint2 - 1u) * 22u) / 8u),4u);
        global_ulong1 >>= ((global_uint2 - 1U)* 22u) % 8u;
        global_ulong1 &= 0x3FFFFFu;
        
        /* Determine component types in bitstream */
        global_uchar1 = (DPUCHAR) dp_get_bytes(BlockID,48u+DataIndex/8u,1u);
        if ( (global_uchar1 == G4M_FPGA) || (global_uchar1 == G4M_KEYS) )
        {
            envm_only_flag = FALSE;
            break;
        }
        else if(global_uchar1 == G4M_ENVM)
        {
            envm_only_flag = TRUE;
        }
        DataIndex += (G4M_FRAME_BIT_LENGTH * global_ulong1);
    }
    
    if (envm_only_flag == TRUE)
    {
        dp_MSS_ADDR_CONFIG();
        if (error_code == DPE_SUCCESS)
        {
            dp_MSS_RD_DATA_SETUP();
            if (error_code == DPE_SUCCESS)
            {
                dp_MSS_ADDR_CONFIG();
                if (error_code == DPE_SUCCESS)
                {
                    dp_MSS_WR_DATA_SETUP();
                }
            }
        }
    }
    return;
}

void dp_G4M_post_setup_eNVM(void)
{
    
    if (envm_only_flag == TRUE)
    {
        dp_MSS_ADDR_CONFIG();
        if (error_code == DPE_SUCCESS)
        {
            dp_MSS_WR_DATA();
        }
    }
    return;
}

void dp_MSS_ADDR_CONFIG(void)
{
    dp_flush_global_buf1();
    
    global_buf1[0] = 0x0cu;
    global_buf1[1] = 0x80u;
    global_buf1[2] = 0x03u;
    global_buf1[3] = 0x40u;
    global_buf1[4] = 0x02u;
    global_buf1[5] = 0x00u;
    global_buf1[6] = 0x00u;
    global_buf1[7] = 0x00u;
    
    opcode = G4M_MSSADDR;
    IRSCAN_in();
    DRSCAN_in(0u, G4M_MSSADDR_BIT_LENGTH, global_buf1);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    
    opcode = G4M_MSSADDR;
    dp_G4M_device_poll(64u, 63u);
    
    return;
}
void dp_MSS_RD_DATA_SETUP(void)
{
    dp_flush_global_buf1();
    global_buf1[0] = 0x04u;
    
    opcode = G4M_MSSRD;
    IRSCAN_in();
    DRSCAN_in(0u, G4M_MSSRD_BIT_LENGTH, global_buf1);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    
    opcode = G4M_MSSRD;
    dp_G4M_device_poll(16u, 15u);
    #ifndef ENABLE_DISPLAY
    if ( (poll_buf[0] & 0x2u) == 1)
    {
        dp_display_text("\r\nMSS_RD_DATA_SETUP: msserr");
    }
    if ( (poll_buf[0] & 0x1u) == 1)
    {
        dp_display_text("\r\nMSS_RD_DATA_SETUP: secerr");
    }
    #endif
    dp_read_shared_buffer(1u);
    
    for (global_uchar1 = 0u; global_uchar1 < 4u; global_uchar1++)
    {
        SYSREG_TEMP[global_uchar1] = shared_buf[global_uchar1];
    }
    
    
    return;
}

void dp_MSS_WR_DATA_SETUP(void)
{
    dp_flush_global_buf1();
    for(global_uchar1 = 0u; global_uchar1 < 4u; global_uchar1++)
    {
        global_buf1[global_uchar1] = SYSREG_TEMP[global_uchar1];
    }
    global_buf1[0] |= 0xe0u;
    global_buf1[1] |= 0x1fu;
    
    opcode = G4M_MSSWR;
    IRSCAN_in();
    DRSCAN_in(0u, G4M_MSSWR_BIT_LENGTH, global_buf1);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    
    opcode = G4M_MSSWR;
    dp_G4M_device_poll(32u, 31u);
    #ifndef ENABLE_DISPLAY
    if ( (poll_buf[0] & 0x2u) == 1)
    {
        dp_display_text("\r\nMSS_WR_DATA_SETUP: msserr");
    }
    if ( (poll_buf[0] & 0x1u) == 1)
    {
        dp_display_text("\r\nMSS_WR_DATA_SETUP: secerr");
    }
    #endif
    
    
    
    return;
}

void dp_MSS_WR_DATA(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nRestore user FREQRNG setting ");
    dp_display_array(SYSREG_TEMP,4u,HEX);
    dp_display_text("\r\n");
    #endif
    
    opcode = G4M_MSSWR;
    IRSCAN_in();
    DRSCAN_in(0u, G4M_MSSWR_BIT_LENGTH, SYSREG_TEMP);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    
    opcode = G4M_MSSWR;
    dp_G4M_device_poll(32u, 31u);
    #ifndef ENABLE_DISPLAY
    if ( (poll_buf[0] & 0x2u) == 1)
    {
        dp_display_text("\r\nMSS_WR_DATA: msserr");
    }
    if ( (poll_buf[0] & 0x1u) == 1)
    {
        dp_display_text("\r\nMSS_WR_DATA: secerr");
    }
    #endif
    
    
    
    return;
}




void dp_G4M_process_data(DPUCHAR BlockID)
{
    
    DataIndex = 0u;  
    /* Global unit1 is used to hold the number of components */
    /* Loop through the number of components */
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\n");
    #endif
    
    for (; global_uint2 <= global_uint1; global_uint2++)
    {
        /* get the number of blocks */
        /* Global ulong1 is used to hold the number of blocks within the components */
        global_ulong1 = dp_get_bytes(G4M_NUMBER_OF_BLOCKS_ID,(DPULONG)(((global_uint2 - 1u) * 22u) / 8u),4u);
        global_ulong1 >>= ((global_uint2 - 1U)* 22u) % 8u;
        global_ulong1 &= 0x3FFFFFu;
        
        opcode = G4M_FRAME_DATA;
        IRSCAN_in();
        dp_get_and_DRSCAN_in(BlockID, G4M_FRAME_BIT_LENGTH, DataIndex);
        
        for (global_ulong2 = 1u; global_ulong2 <= global_ulong1; global_ulong2++)
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\rProcessing frame: ");
            dp_display_value(global_ulong2,DEC);
            #endif
            
            goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
            dp_delay(G4M_STANDARD_DELAY);
            
            opcode = G4M_FRAME_DATA;
            if (global_ulong2 == global_ulong1)
            {
                dp_G4M_device_poll(128u, 127u);
            }
            else
            {
                dp_G4M_device_shift_and_poll(128u, 127u, BlockID, DataIndex + G4M_FRAME_BIT_LENGTH);
            }
            if ( ((error_code != DPE_SUCCESS) || ((poll_buf[0] & 0x18u) != 0u)) )
            {
                dp_G4M_get_data_status();
                #ifdef ENABLE_DISPLAY
                dp_display_text("\r\ncomponentNo: ");
                dp_display_value(global_uint2, DEC);
                dp_display_text("\r\nblockNo: ");
                dp_display_value(global_ulong2, DEC);
                dp_display_text("\r\nDATA_STATUS_RESULT: ");
                dp_display_array(poll_buf,4u,HEX);
                dp_display_text("\r\nERRORCODE: ");
                dp_display_value((poll_buf[0]>>3u) & 0x1fu,HEX);
                dp_display_text("\r\nAUTHERRCODE: ");
                dp_display_value(poll_buf[1],HEX);
                #endif
                error_code = DPE_PROCESS_DATA_ERROR;
                global_uint2 = global_uint1;
                break;
            }
            
            DataIndex += G4M_FRAME_BIT_LENGTH;
        }
    }
    
    return;
}

void dp_G4M_get_data_status(void)
{
    opcode = G4M_FRAME_STATUS;
    IRSCAN_in();
    DRSCAN_in(0u, ISC_STATUS_REGISTER_BIT_LENGTH, (DPUCHAR*)(DPUCHAR*)DPNULL);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    
    opcode = G4M_FRAME_STATUS;
    dp_G4M_device_poll(32u, 31u);
    
    return;
}

void dp_G4M_read_security(void)
{
    dp_G4M_query_security();
    if ((error_code == DPE_SUCCESS) && (security_queried == FALSE) )
    {
        dp_G4M_unlock_dpk();
        if (error_code == DPE_SUCCESS)
        {
            dp_G4M_query_security();
            if ((error_code == DPE_SUCCESS) && (security_queried == FALSE) )
            {
                #ifdef ENABLE_DISPLAY
                dp_display_text("\r\nWarning: Security cannot be read even after unlocking debug pass key.");	
                #endif
            }
        }
    }
    sec_ul = shared_buf[4] >> 2u;
    return;
}
void dp_G4M_query_security(void)
{
    opcode = G4M_QUERY_SECURITY;
    IRSCAN_in();
    DRSCAN_in(0u, G4M_SECURITY_STATUS_REGISTER_BIT_LENGTH, (DPUCHAR*)(DPUCHAR*)DPNULL);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    opcode = G4M_QUERY_SECURITY;
    dp_G4M_device_poll(16u, 15u);
    if (error_code != DPE_SUCCESS)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nFailed to query security information.");	
        #endif
    }
    else
    {
        dp_read_shared_buffer(3u);
        for (poll_index = 0u;poll_index < 40u; poll_index++)
        {
            if (shared_buf[poll_index] != 0u)
            {
                security_queried = TRUE;
                break;
            }
        }
        
        #ifdef ENABLE_DISPLAY
        if (security_queried == TRUE)
        {
            dp_display_text("\r\n--- Security locks and configuration settings ---\r\n");	
            dp_display_array(shared_buf,42u,HEX);
        }
        #endif
    }
    return;
}

void dp_G4M_unlock_dpk(void)
{
    dp_get_data(G4M_DPK_ID,0u);
    if (return_bytes == 0u)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nWarning: DPK data is missing.");
        #endif
    }
    else
    {
        dp_G4M_load_dpk();
        if (error_code == DPE_SUCCESS)
        {
            opcode = G4M_UNLOCK_DEBUG_PASSCODE;
            IRSCAN_in();
            DRSCAN_in(0u, G4M_STATUS_REGISTER_BIT_LENGTH, (DPUCHAR*)(DPUCHAR*)DPNULL);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
            dp_delay(G4M_STANDARD_DELAY);
        }
        dp_G4M_device_poll(8u, 7u);
        if ((error_code != DPE_SUCCESS) || ((poll_buf[0] & 0x3u) != 0x1u) )
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nFailed to unlock debug pass key.");
            #endif
            error_code = DPE_MATCH_ERROR;
        }
        else
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nDebug security (DPK) is unlocked.");
            #endif
        }
    }
    return;
}

void dp_G4M_unlock_upk1(void)
{
    dp_get_data(G4M_UPK1_ID,0u);
    if (return_bytes == 0u)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nWarning: UPK1 data is missing.");
        #endif
    }
    else
    {
        dp_G4M_load_upk1();
        if (error_code == DPE_SUCCESS)
        {
            opcode = G4M_UNLOCK_USER_PASSCODE;
            IRSCAN_in();
            DRSCAN_in(0u, G4M_STATUS_REGISTER_BIT_LENGTH, (DPUCHAR*)(DPUCHAR*)DPNULL);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
            dp_delay(G4M_STANDARD_DELAY);
        }
        dp_G4M_device_poll(8u, 7u);
        if ((error_code != DPE_SUCCESS) || ((poll_buf[0] & 0x3u) != 0x1u) )
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nFailed to unlock user pass key 1.");
            #endif
            error_code = DPE_MATCH_ERROR;
        }
        else
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nUser security (DPK1) is unlocked.");
            #endif
        }
    }
    return;
}

void dp_G4M_unlock_upk2(void)
{
    dp_get_data(G4M_UPK2_ID,0u);
    if (return_bytes == 0u)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nWarning: UPK2 data is missing.");
        #endif
    }
    else
    {
        dp_G4M_load_upk2();
        if (error_code == DPE_SUCCESS)
        {
            opcode = G4M_UNLOCK_VENDOR_PASSCODE;
            IRSCAN_in();
            DRSCAN_in(0u, G4M_STATUS_REGISTER_BIT_LENGTH, (DPUCHAR*)(DPUCHAR*)DPNULL);
            goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
            dp_delay(G4M_STANDARD_DELAY);
        }
        dp_G4M_device_poll(8u, 7u);
        if ((error_code != DPE_SUCCESS) || ((poll_buf[0] & 0x3u) != 0x1u) )
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nFailed to unlock user pass key 2.");
            #endif
            error_code = DPE_MATCH_ERROR;
        }
        else
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nUser security (DPK2) is unlocked.");
            #endif
        }
    }
    return;
}

void dp_G4M_load_dpk(void)
{
    opcode = G4M_KEYLO;
    IRSCAN_in();
    dp_get_and_DRSCAN_in(G4M_DPK_ID, G4M_FRAME_BIT_LENGTH, 0u);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    dp_G4M_device_poll(128u, 127u);
    if (error_code != DPE_SUCCESS)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nFailed to load keylo. \r\nkeylo_result: ");
        dp_display_array(poll_buf,G4M_FRAME_BYTE_LENGTH,HEX);
        #endif
        error_code = DPE_MATCH_ERROR;
    }
    else
    {
        opcode = G4M_KEYHI;
        IRSCAN_in();
        dp_get_and_DRSCAN_in(G4M_DPK_ID, G4M_FRAME_BIT_LENGTH, G4M_FRAME_BIT_LENGTH);
        goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
        dp_delay(G4M_STANDARD_DELAY);
        dp_G4M_device_poll(128u, 127u);
        if (error_code != DPE_SUCCESS)
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nFailed to load keyhi. \r\nkeyhi_result: ");
            dp_display_array(poll_buf,G4M_FRAME_BYTE_LENGTH,HEX);
            #endif
            error_code = DPE_MATCH_ERROR;
        }
    }
    
    return;
}

void dp_G4M_load_upk1(void)
{
    opcode = G4M_KEYLO;
    IRSCAN_in();
    dp_get_and_DRSCAN_in(G4M_UPK1_ID, G4M_FRAME_BIT_LENGTH, 0u);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    dp_G4M_device_poll(128u, 127u);
    if (error_code != DPE_SUCCESS)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nFailed to load keylo. \r\nkeylo_result: ");
        dp_display_array(poll_buf,G4M_FRAME_BYTE_LENGTH,HEX);
        #endif
        error_code = DPE_MATCH_ERROR;
    }
    else
    {
        opcode = G4M_KEYHI;
        IRSCAN_in();
        dp_get_and_DRSCAN_in(G4M_UPK1_ID, G4M_FRAME_BIT_LENGTH, G4M_FRAME_BIT_LENGTH);
        goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
        dp_delay(G4M_STANDARD_DELAY);
        dp_G4M_device_poll(128u, 127u);
        if (error_code != DPE_SUCCESS)
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nFailed to load keyhi. \r\nkeyhi_result: ");
            dp_display_array(poll_buf,G4M_FRAME_BYTE_LENGTH,HEX);
            #endif
            error_code = DPE_MATCH_ERROR;
        }
    }
    
    return;
}

void dp_G4M_load_upk2(void)
{
    opcode = G4M_KEYLO;
    IRSCAN_in();
    dp_get_and_DRSCAN_in(G4M_UPK2_ID, G4M_FRAME_BIT_LENGTH, 0u);
    goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
    dp_delay(G4M_STANDARD_DELAY);
    dp_G4M_device_poll(128u, 127u);
    if (error_code != DPE_SUCCESS)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nFailed to load keylo. \r\nkeylo_result: ");
        dp_display_array(poll_buf,G4M_FRAME_BYTE_LENGTH,HEX);
        #endif
        error_code = DPE_MATCH_ERROR;
    }
    else
    {
        opcode = G4M_KEYHI;
        IRSCAN_in();
        dp_get_and_DRSCAN_in(G4M_UPK2_ID, G4M_FRAME_BIT_LENGTH, G4M_FRAME_BIT_LENGTH);
        goto_jtag_state(JTAG_RUN_TEST_IDLE,G4M_STANDARD_CYCLES);
        dp_delay(G4M_STANDARD_DELAY);
        dp_G4M_device_poll(128u, 127u);
        if (error_code != DPE_SUCCESS)
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nFailed to load keyhi. \r\nkeyhi_result: ");
            dp_display_array(poll_buf,G4M_FRAME_BYTE_LENGTH,HEX);
            #endif
            error_code = DPE_MATCH_ERROR;
        }
    }
    
    return;
}


#endif /* ENABLE_G4_SUPPORT */

