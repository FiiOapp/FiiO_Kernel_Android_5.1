/* ************************************************************************ */
/*                                                                          */
/*  DirectC         Copyright (C) Microsemi Corporation 2015                */
/*  Version 3.2     Release date  October 30, 2015                          */
/*                                                                          */
/* ************************************************************************ */
/*                                                                          */
/*  Module:         dpsecurity.h                                            */
/*                                                                          */
/*  Description:    Contains the function prototypes.                       */
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

#ifndef INC_dpsecurity_H
#define INC_dpsecurity_H

/****************************************************************************/
/* Security Opcodes                                                         */
/****************************************************************************/
#define ISC_PROGRAM_UKEY        0x8BU
#define ISC_PROGRAM_DMK         0x91U
#define ISC_VERIFY_DMK          0x0AU
#define AES_INIT                0xDDU
#define ISC_MATCH_UKEY          0x92U
#define ISC_PROGRAM_SECURITY    0xA3U
#define ISC_QUERY_SECURITY      0xA4U
#define AES_MODE                0xACU

/****************************************************************************/
/* Security Bit Locations for AFS devices.                                  */
/****************************************************************************/
#define ULNR0               0x00000001U  /* NVM Block 0 Verify pass key bit */
#define ULNW0               0x00000002U  /* NVM Block 0 write pass key bit */
#define ULNC0               0x00000004U  /* NVM Block 0 programming encryption bit */
#define ULNR1               0x00000008U  /* NVM Block 1 Verify pass key bit */
#define ULNW1               0x00000010U  /* NVM Block 1 write pass key bit */
#define ULNC1               0x00000020U  /* NVM Block 1 programming encryption bit */
#define ULNR2               0x00000040U  /* NVM Block 2 Verify pass key bit */
#define ULNW2               0x00000080U  /* NVM Block 2 write pass key bit */
#define ULNC2               0x00000100U  /* NVM Block 2 programming encryption bit */
#define ULNR3               0x00000200U  /* NVM Block 3 Verify pass key bit */
#define ULNW3               0x00000400U  /* NVM Block 3 write pass key bit */ 
#define ULNC3               0x00000800U  /* NVM Block 3 programming encryption bit */
#define ULARD               0x00001000U  /* Array verification pass key bit */
#define ULAWE               0x00002000U  /* Array erase and programming pass key bit */
#define ULULR               0x00004000U  /* DMK / user security erase and programming protection */
#define ULFLR               0x00008000U  /* Factory key disable bit */
#define ULUFJ               0x00010000U  /* FROM verification pass key bit */
#define ULUFP               0x00020000U  /* FROM erase and programming pass key bit */
#define ULUFE               0x00040000U  /* FROM programming encyption bit */
#define ULUPC               0x00080000U  /* Pass key bit */
#define ULARE               0x00100000U  /* Array programming encryption bit */
#define ULUWE               0x00200000U  /* UROW Erase and programming protection */ 
#define ULOPT0              0x00400000U
#define ULOPT1              0x00800000U
#define SEC_KEY_OK          0x01000000U
#define PERM_LOCK_BIT       0x02000000U
#define IS_ERASE_ONLY       0x04000000U
#define IS_RESTORE_DESIGN   0x08000000U
#define SET_ERASE_SEC       0x10000000U
#define M7_DEVICE           0x20000000U
#define M1_DEVICE           0x40000000U
#define P1_DEVICE           0x80000000U
/****************************************************************************/
/* NVM Register length and parameters                                       */
/****************************************************************************/
#define UKEY_BIT_LENGTH		     128U
#define DMK_BIT_LENGTH		     128U
#define AES_BIT_LENGTH		     128U
#define AES_MODE_BIT_LENGTH      3U
#define ULOCK_A3P_AGL_BIT_LENGTH 44U
#define ULOCK_AFS_BIT_LENGTH	 56U
#define SILGIG_BIT_LENGTH	     32U

/* Security delay and cycle parameters */
#define ISC_PROGRAM_UKEY_CYCLES        3u
#define ISC_PROGRAM_DMK_CYCLES         15u
#define ISC_PROGRAM_SECURITY_CYCLES    3u
#define ISC_QUERY_SECURITY_CYCLES      3u
#define ISC_MATCH_UKEY_CYCLES          3u
#define ISC_MATCH_UKEY_DELAY           1438u
#define ISC_VERIFY_DMK_CYCLES          3u
#define ISC_VERIFY_DMK_DELAY           104u


/****************************************************************************/
/* Function prototypes                                                      */
/****************************************************************************/
void dp_program_security(void);
void dp_write_sec_key(void);
void dp_write_enc_key(void);
void dp_match_security(void);
void dp_program_ulock(void);
void dp_read_device_security(void);
void dp_program_silsig(void);
void dp_erase_security_action(void);
void dp_erase_security(void);
void dp_program_security_action(void);
void dp_check_dual_key(void);
void dp_verify_id_dmk(void);
void dp_verify_p1_dmk(void);
void dp_verify_m1_dmk(void);
void dp_verify_m7_dmk(void);
void dp_verify_fc_dmk(void);
#endif

