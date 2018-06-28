/* ************************************************************************ */
/*                                                                          */
/*  DirectC         Copyright (C) Microsemi Corporation 2015                */
/*  Version 3.2     Release date  October 30, 2015                          */
/*                                                                          */
/* ************************************************************************ */
/*                                                                          */
/*  Module:         dpjtag.h                                                */
/*                                                                          */
/*  Description:    Definitions of JTAG constants, types, and functions     */
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

#ifndef INC_DPJTAG_H
#define INC_DPJTAG_H

/****************************************************************************/
/* JTAG states codes used to identify current and target JTAG states        */
/****************************************************************************/
#define JTAG_TEST_LOGIC_RESET	1u
#define JTAG_RUN_TEST_IDLE		2u
#define JTAG_SHIFT_DR		    3u
#define JTAG_SHIFT_IR		    4u
#define JTAG_EXIT1_DR		    5u
#define JTAG_EXIT1_IR		    6u
#define JTAG_PAUSE_DR		    7u
#define JTAG_PAUSE_IR		    8u
#define JTAG_UPDATE_DR		    9u
#define JTAG_UPDATE_IR		    10u
#define JTAG_CAPTURE_DR		    11u

/****************************************************************************/
/* Function prototypes                                                      */
/****************************************************************************/
void dp_wait_cycles(DPUCHAR cycles);
void goto_jtag_state(DPUCHAR target_state,DPUCHAR cycles);
void dp_shift_in(DPULONG start_bit, DPUINT num_bits, DPUCHAR tdi_data[],DPUCHAR terminate);
void dp_shift_in_out(DPUINT num_bits, DPUCHAR tdi_data[],DPUCHAR tdo_data[]);
void dp_get_and_shift_in(DPUCHAR Variable_ID,DPUINT total_bits_to_shift, DPULONG start_bit_index);
void dp_get_and_shift_in_out(DPUCHAR Variable_ID,DPUCHAR total_bits_to_shift, DPULONG start_bit_index,DPUCHAR* tdo_data);
void IRSCAN_in(void);
void IRSCAN_out(unsigned char* outbuf);
void DRSCAN_out(unsigned int bits_to_shift,unsigned char* inbuf,unsigned char* outbuf);
void DRSCAN_in(unsigned long start_bit_index, unsigned int bits_to_shift,unsigned char* inbuf);
void dp_get_and_DRSCAN_in(DPUCHAR Variable_ID,DPUINT total_bits_to_shift, DPULONG start_bit_index);
void dp_get_and_DRSCAN_in_out(DPUCHAR Variable_ID,DPUCHAR total_bits_to_shift, DPULONG start_bit_index,DPUCHAR* tdo_data);

extern DPUCHAR global_jtag_i;
extern DPUCHAR current_jtag_state;
extern DPUCHAR idx;
extern DPUCHAR data_buf;
extern DPUCHAR bit_buf;
extern DPUCHAR error_code;


#endif /* INC_DPJTAG_H */

