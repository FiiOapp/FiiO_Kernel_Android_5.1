/* ************************************************************************ */
/*                                                                          */
/*  DirectC         Copyright (C) Microsemi Corporation 2015                */
/*  Version 3.2     Release date  October 30, 2015                          */
/*                                                                          */
/* ************************************************************************ */
/*                                                                          */
/*  Module:         dpG3alg.h                                               */
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

#ifndef INC_DPG3ALG_H
#define INC_DPG3ALG_H

#ifndef ENABLE_CODE_SPACE_OPTIMIZATION
extern DPUCHAR dat_version;
#define V85_DAT 1u
#endif

/* G3 specific compile options */
#define CORE_SUPPORT
#define CORE_PLAIN
#define CORE_ENCRYPT
#define FROM_SUPPORT
#define FROM_PLAIN
#define FROM_ENCRYPT
#define NVM_SUPPORT
#define NVM_PLAIN
#define NVM_ENCRYPT
#define SECURITY_SUPPORT
#define SILSIG_SUPPORT
#define ENABLE_DAS_SUPPORT

/* Code optimization specific compile switches */
/* #define ENABLE_CODE_SPACE_OPTIMIZATION */
/* #define DISABLE_CORE_SPECIFIC_ACTIONS */
/* #define DISABLE_FROM_SPECIFIC_ACTIONS */
/* #define DISABLE_NVM_SPECIFIC_ACTIONS */
/* #define DISABLE_SEC_SPECIFIC_ACTIONS */

/* This option could be used for performing progam_nvm_active_array and 
verify_nvm_active_array actions. 
Set FORCE_NVM_ACCESS to 1 to force NVM access */
#define FORCE_NVM_ACCESS 0u



#define BOL 0u
#define EOL 3u

/* 
* Data block ID definitions
*/
#define ACT_UROW_DESIGN_NAME_ID   1u
#define BsrPattern_ID             2u  
#define SILSIG_ID                 3u		
#define CHECKSUM_ID               4u
#define datastream_ID             5u
#define rlock_ID                  6u
#define FRomAddressMask_ID        7u
#define FRomStream_ID             8u

/* These defintions are the same as NVM block zoro.  They are defined to aviod confusion when pogram and verify NVM functions are called. */
#define NVM_OFFSET                5u
#define NvmParSize_ID             9u 
#define NumOfPart_ID             10u
#define NvmAddr_ID               11u
#define NvmData_ID               12u
#define NvmProtect_ID            13u


#define NvmParSize_0_ID           9u 
#define NumOfPart_0_ID           10u
#define NvmAddr_0_ID             11u
#define NvmData_0_ID             12u
#define NvmProtect_0_ID          13u
#define NvmParSize_1_ID          14u
#define NumOfPart_1_ID           15u
#define NvmAddr_1_ID             16u
#define NvmData_1_ID             17u
#define NvmProtect_1_ID          18u
#define NvmParSize_2_ID          19u
#define NumOfPart_2_ID           20u
#define NvmAddr_2_ID             21u
#define NvmData_2_ID             22u
#define NvmProtect_2_ID          23u
#define NvmParSize_3_ID          24u
#define NumOfPart_3_ID           25u
#define NvmAddr_3_ID             26u
#define NvmData_3_ID             27u
#define NvmProtect_3_ID          28u
#define UKEY_ID                  29u
#define DMK_ID                   30u
#define KDATA_ID                 31u
#define ULOCK_ID                 32u
#define NvmParSizePriv_all_ID    33u
#define NumOfPartPriv_all_ID     34u
#define NvmAddrPriv_all_ID       35u
#define NvmDataPriv_all_ID       36u
#define NvmProtectPriv_all_ID    37u
#define rlockDisable_ID          38u

/*
* Location of special variables needed in the header section of the image file
*/
#define M_DEVICE_OFFSET          30u
#define ID_OFFSET                31u

#define DEVICE_FAMILY_OFFSET     55u
#define DEVICE_NAME_OFFSET       56u
#define ID_MASK_OFFSET           57u
#define SD_TILES_OFFSET          61u
#define MAP_ROWS_OFFSET          62u
#define BSR_LENGTH_OFFSET        64u
#define SE_WAIT_OFFSET           66u
#define DUAL_KEY_SUPPORT_OFFSET  67u




#define AXX015_DEVICE   1u
#define AXX030_DEVICE   2u

#if ((!defined ENABLE_CODE_SPACE_OPTIMIZATION) || defined ENABLE_DAS_SUPPORT)
/************************************************************/
/* Device specific parameters                               */
/************************************************************/
/* Supported A3PE/A3PEL/AGLE     */
#define AXXE600X_ID                 0x023261CFu
#define AXXE600X_ID_MASK            0x03FFFFFFu
#define AXXE600X_SD                 6u
#define AXXE600X_ROWS               3444u
#define AXXE600X_COLS               1184u
#define AXXE600X_BSR_BIT_LENGTH     1056u

#define AXXE1500X_ID                0x0253A1CFu
#define AXXE1500X_ID_MASK           0x03FFFFFFu
#define AXXE1500X_SD                10u
#define AXXE1500X_ROWS              5644u
#define AXXE1500X_COLS              1956u
#define AXXE1500X_BSR_BIT_LENGTH    1740u

#define AXXE3000X_ID                0x0274E1CFu
#define AXXE3000X_ID_MASK           0x03FFFFFFu
#define AXXE3000X_SD                14u
#define AXXE3000X_ROWS              7844u
#define AXXE3000X_COLS              2728u
#define AXXE3000X_BSR_BIT_LENGTH    2424u

/* Supported A3P/A3PL/AGL     */
#define AXX030X_ID                  0x049011CFu
#define AXX030X_ID_MASK             0x07FFFFFFu
#define AXX030X_SD                  2u
#define AXX030X_ROWS                625u
#define AXX030X_COLS                412u
#define AXX030X_BSR_BIT_LENGTH      288u

#define AXX060X_ID                  0x029121CFu
#define AXX060X_ID_MASK             0x06FFFFFFu
#define AXX060X_SD                  2u
#define AXX060X_ROWS                1244u
#define AXX060X_COLS                412u
#define AXX060X_BSR_BIT_LENGTH      372u

#define AXX125X_ID                  0x02A121CFu
#define AXX125X_ID_MASK             0x06FFFFFFu
#define AXX125X_SD                  4u
#define AXX125X_ROWS                1244u
#define AXX125X_COLS                798u
#define AXX125X_BSR_BIT_LENGTH      564u

#define AXX250X_ID                  0x02A141CFu
#define AXX250X_ID_MASK             0x06FFFFFFu
#define AXX250X_SD                  4u
#define AXX250X_ROWS                2300u
#define AXX250X_COLS                798u
#define AXX250X_BSR_BIT_LENGTH      708u

#define AXX400X_ID                  0x02B141CFu
#define AXX400X_ID_MASK             0x06FFFFFFu
#define AXX400X_SD                  6u
#define AXX400X_ROWS                2300u
#define AXX400X_COLS                1184u
#define AXX400X_BSR_BIT_LENGTH      900u

#define	AXX600X_ID                 0x02b261CFu
#define	AXX600X_ID_MASK            0x06FFFFFFu
#define	AXX600X_SD                 6u
#define	AXX600X_ROWS               3444u
#define	AXX600X_COLS               1184u
#define AXX600X_BSR_BIT_LENGTH      1056u

#define AXX1000X_ID                 0x12C281CFu
#define AXX1000X_ID_MASK            0x06FFFFFFu
#define AXX1000X_SD                 8u
#define AXX1000X_ROWS               4500u
#define AXX1000X_COLS               1570u
#define AXX1000X_BSR_BIT_LENGTH     1392u

/* Supported AGLP     */
#define AGLP030X_ID                 0x0E1011CFu
#define AGLP030X_ID_MASK            0x0FFFFFFFu
#define AGLP030X_SD                 2u
#define AGLP030X_ROWS               625u
#define AGLP030X_COLS               412u
#define AGLP030X_BSR_BIT_LENGTH     288u

#define AGLP060X_ID                 0x0E1121CFu
#define AGLP060X_ID_MASK            0x0EFFFFFFu
#define AGLP060X_SD                 2u
#define AGLP060X_ROWS               1244u
#define AGLP060X_COLS               412u
#define AGLP060X_BSR_BIT_LENGTH     372u

#define AGLP125X_ID                 0x0E2121CFu
#define AGLP125X_ID_MASK            0x0EFFFFFFu
#define AGLP125X_SD                 4u
#define AGLP125X_ROWS               1244u
#define AGLP125X_COLS               798u
#define AGLP125X_BSR_BIT_LENGTH     564u

#define AXXN010X_ID                 0x069041CFu
#define AXXN020X_ID                 0x069081CFu
#define AXXN060X_ID                 0x039521CFu
#define AXXN125X_ID                 0x03A521CFu
#define AXXN250X_ID                 0x03A541CFu

/* Supported AFS Devices */
#define AFS090_ID                   0x031921CFu
#define AFS090_ID_MASK              0x0BFFFFFFu
#define AFS090_SD                   3u
#define AFS090_ROWS                 1244u
#define AFS090_COLS                 605u
#define AFS090_BSR_BIT_LENGTH       468u

#define AFS250_ID                   0x032141CFu
#define AFS250_ID_MASK              0x0FFFFFFFu
#define AFS250_SD                   4u
#define AFS250_ROWS                 2300u
#define AFS250_COLS                 798u
#define AFS250_BSR_BIT_LENGTH       708u

#define AFS600_ID                   0x033261CFu
#define AFS600_ID_MASK              0x0FFFFFFFu
#define AFS600_SD                   6u
#define AFS600_ROWS                 3444u
#define AFS600_COLS                 1184u
#define AFS600_BSR_BIT_LENGTH       1056u

#define AFS1500_ID                  0x0353A1CFu
#define AFS1500_ID_MASK             0x0FFFFFFFu
#define AFS1500_SD                  10u
#define AFS1500_ROWS                5644u
#define AFS1500_COLS                1956u
#define AFS1500_BSR_BIT_LENGTH      1740u
#else
#define AFS600_ID                   0x033261CFu
#define AFS1500_ID                  0x0353A1CFu
#endif
#define A2F200_ID                   0x05A131CFu


#define LDVPROP_LENGTH  6u

/************************************************************/
/* Instruction Set                                         */
/************************************************************/
/* General opcodes */
#define ISC_NOOP                 0x84u
#define DESELECT_ALL_TILES       0xC0u
/* Erase opcodes */              
#define ISC_ERASE                0x85u

/* UROW opcodes */               
#define ISC_PROGRAM_UROW         0xA7u
#define ISC_READ_UROW            0xA8u
#define ISC_PROGRAM_RLOCK        0x8Cu

#define HIGHZ                    0x07u
#define BYPASS                   0xFFu
#define USERCODE                 0x0Eu
#define LDVPROP                  0xB4u
/* Factory row opcodes */
#define FACTORY_ADDRESS_SHIFT    0xF9u
#define READ_FACTORY             0xBFu

/* UROW Data */
#define GPIO_PROGRAMMING_METHOD  0x2u
#define IAP_PROGRAMMING_METHOD   0x5u
/* This is equivalent to FP90 */
#define DIRECTC_VERSION          0x30u
#define DC_PROGRAMMING_SW        0x5u
#define ALGO_VERSION             0x14u

#define DC_SOFTWARE_VERSION      26u
#define IEEE1532_PM              0u
#define STAPL_PM                 1u
#define DIRECTC_PM               2u
#define PDB_PM                   3u
#define SVF_PM                   4u
#define IAP_PM                   5u

/* Programmer */
#define FP                       0u
#define FPLITE                   1u
#define FP3                      2u
#define SCULPTW                  3u
#define BPW                      4u
#define DIRECTCP                 5u
#define STP                      6u
#define FP4                      7u

/************************************************************/
/* Block Support status bits                                */
/************************************************************/
#define DAT_SUPPORT_STATUS_OFFSET   39u
#define CORE_DAT_SUPPORT_BIT        0x0001u
#define FROM_DAT_SUPPORT_BIT        0x0002u
#define NVM_DAT_SUPPORT_BIT         0x0004u 
#define NVM0_DAT_SUPPORT_BIT        0x0008u
#define NVM1_DAT_SUPPORT_BIT        0x0010u
#define NVM2_DAT_SUPPORT_BIT        0x0020u
#define NVM3_DAT_SUPPORT_BIT        0x0040u
#define NVM_DAT_VERIFY_SUPPORT_BIT  0x0080u
#define SEC_DAT_SUPPORT_BIT         0x0100u
#define AES_DAT_SUPPORT_BIT         0x0200u
#define CORE_DAT_ENCRYPTION_BIT     0x0400u
#define FROM_DAT_ENCRYPTION_BIT     0x0800u
#define	NVM0_DAT_ENCRYPTION_BIT     0x1000u
#define	NVM1_DAT_ENCRYPTION_BIT     0x2000u
#define	NVM2_DAT_ENCRYPTION_BIT     0x4000u
#define	NVM3_DAT_ENCRYPTION_BIT     0x8000u

/************************************************************/
/* Register Length                                          */
/************************************************************/
#define MAX_ERASE_POLL           262140u
#define MAX_PROGRAM_POLL         16380u
#define FACTORY_ROW_BIT_LENGTH   128u

/************************************************************/
/* Erase Bits Definitions                                   */
/************************************************************/
#define CORE_ERASE_BITS_BYTE0        0x1u   /*Bit 0 */
#define CORE_ERASE_BITS_BYTE1        0x0u   /*Bit 0 */
#define CORE_ERASE_BITS_BYTE2        0x0u   /*Bit 0 */
#define ULOCK_ERASE_BITS_BYTE0       0x2u   /*Bit 1 */
#define ULOCK_ERASE_BITS_BYTE1       0x0u   /*Bit 1 */
#define ULOCK_ERASE_BITS_BYTE2       0x0u   /*Bit 1 */
#define DMK_ERASE_BITS_BYTE0         0x4u   /*Bit 2 */
#define DMK_ERASE_BITS_BYTE1         0x0u   /*Bit 2 */
#define DMK_ERASE_BITS_BYTE2         0x0u   /*Bit 2 */
#define UKEY_ERASE_BITS_BYTE0        0x8u   /*Bit 3 */
#define UKEY_ERASE_BITS_BYTE1        0x0u   /*Bit 3 */
#define UKEY_ERASE_BITS_BYTE2        0x0u   /*Bit 3 */
#define FLOCK_ERASE_BITS_BYTE0      0x10u   /*Bit 4 */
#define FLOCK_ERASE_BITS_BYTE1       0x0u   /*Bit 4 */
#define FLOCK_ERASE_BITS_BYTE2       0x0u   /*Bit 4 */
#define FPRM_ERASE_BITS_BYTE0       0xE0u   /*Bits 5-10*/
#define FPRM_ERASE_BITS_BYTE1        0x7u   /*Bits 5-10*/
#define FPRM_ERASE_BITS_BYTE2        0x0u   /*Bits 5-10*/
#define FKEY_ERASE_BITS_BYTE0        0x0u   /*Bit 11*/
#define FKEY_ERASE_BITS_BYTE1        0x8u   /*Bit 11*/
#define FKEY_ERASE_BITS_BYTE2        0x0u   /*Bit 11*/
#define SECEN_ERASE_BITS_BYTE0       0x0u   /*Bit 12*/
#define SECEN_ERASE_BITS_BYTE1      0x10u   /*Bit 12*/
#define SECEN_ERASE_BITS_BYTE2       0x0u   /*Bit 12*/
#define VIRREF_ERASE_BITS_BYTE0      0x0u   /*Bit 13*/
#define VIRREF_ERASE_BITS_BYTE1     0x20u   /*Bit 13*/
#define VIRREF_ERASE_BITS_BYTE2      0x0u   /*Bit 13*/
#define UROW_ERASE_BITS_BYTE0        0x0u   /*Bit 14*/
#define UROW_ERASE_BITS_BYTE1       0x40u   /*Bit 14*/
#define UROW_ERASE_BITS_BYTE2        0x0u   /*Bit 14*/
#define FROM_ERASE_BITS_BYTE0        0x0u   /*Bit 14*/
#define FROM_ERASE_BITS_BYTE1       0x80u   /*Bit 14*/
#define FORM_ERASE_BITS_BYTE2      0x07Fu   /*Bit 14*/

#ifndef ENABLE_CODE_SPACE_OPTIMIZATION
#define M7KDATA0      0x45u
#define M7KDATA1      0x49u
#define M7KDATA2      0x66u
#define M7KDATA3      0x73u
#define M7KDATA4      0x3Fu
#define M7KDATA5      0x5Fu
#define M7KDATA6      0x01u
#define M7KDATA7      0x26u
#define M7KDATA8      0x11u
#define M7KDATA9      0xE9u
#define M7KDATA10     0xEEu
#define M7KDATA11     0x2Eu
#define M7KDATA12     0x3Au
#define M7KDATA13     0x62u
#define M7KDATA14     0x37u
#define M7KDATA15     0xE1u

#define M1KDATA0      0x77u
#define M1KDATA1      0x50u
#define M1KDATA2      0xE9u
#define M1KDATA3      0x8Fu
#define M1KDATA4      0xB1u
#define M1KDATA5      0x1Eu
#define M1KDATA6      0x29u
#define M1KDATA7      0x3Eu
#define M1KDATA8      0x86u
#define M1KDATA9      0x88u
#define M1KDATA10     0xB4u
#define M1KDATA11     0xCCu
#define M1KDATA12     0x48u
#define M1KDATA13     0x65u
#define M1KDATA14     0xDDu
#define M1KDATA15     0xACu

#define P1KDATA0      0x15u
#define P1KDATA1      0x7du
#define P1KDATA2      0x69u
#define P1KDATA3      0x38u
#define P1KDATA4      0xaeu
#define P1KDATA5      0x09u
#define P1KDATA6      0x5fu
#define P1KDATA7      0x5eu
#define P1KDATA8      0x17u
#define P1KDATA9      0x4eu
#define P1KDATA10     0x5au
#define P1KDATA11     0x37u
#define P1KDATA12     0x14u
#define P1KDATA13     0xe5u
#define P1KDATA14     0xa9u
#define P1KDATA15     0xe7u

#define FCBYTE0       0x86u
#define FCBYTE1       0x00u
#define FCBYTE2       0x50u
#define FCBYTE3       0x43u
#define FCBYTE4       0x64u
#define FCBYTE5       0x9Cu
#define FCBYTE6       0x52u
#define FCBYTE7       0x40u
#define FCBYTE8       0xC6u
#define FCBYTE9       0x73u
#define FCBYTE10      0xB0u
#define FCBYTE11      0xFBu
#define FCBYTE12      0x75u
#define FCBYTE13      0xE7u
#define FCBYTE14      0xFFu
#define FCBYTE15      0xFDu
#endif

/* General delay and cycle parameters */
#define ISC_ENABLE_CYCLES              3u
#define ISC_ENABLE_DELAY               2157u
#define ISC_DISABLE_DELAY              288u
#define ISC_NOOP_CYCLES                3u
#define ISC_NOOP_DELAY                 264u
#define BYPASS_DELAY                   288u
#define POLL_DELAY                     100u
#define ISC_ERASE_CYCLES               3u
// #define ERASE_POLL_DELAY               1000u
#define ERASE_POLL_DELAY               100u
#define HIGHZ_CYCLES                   1u                                       
#define DESELECT_ALL_TILES_CYCLES      1u
#define AES_INIT_CYCLES                3u                                      
#define AES_INIT_DELAY                 190u
#define AES_MODE_CYCLES                1u
#define USERCODE_CYCLES                3u                                       

/* UROW delay and cycle parameters */
#define ISC_READ_UROW_CYCLES           3u
#define ISC_READ_UROW_DELAY            330u
#define ISC_PROGRAM_UROW_CYCLES        15u

/* Core code enable flags */
#if (defined CORE_SUPPORT && defined CORE_PLAIN)
#define CORE_CODE_PLAIN_SUPPORT_BIT 0x0001u
#else
#define CORE_CODE_PLAIN_SUPPORT_BIT 0x0000u
#endif

#if (defined CORE_SUPPORT && defined CORE_ENCRYPT)
#define CORE_CODE_ENCRYPTION_BIT 0x0400u
#else
#define CORE_CODE_ENCRYPTION_BIT 0x0000u
#endif

/* FROM code enable flags */
#if (defined FROM_SUPPORT && defined FROM_PLAIN)
#define FROM_CODE_PLAIN_SUPPORT_BIT 0x0002u
#else
#define FROM_CODE_PLAIN_SUPPORT_BIT 0x0000u
#endif

#if (defined FROM_SUPPORT && defined FROM_ENCRYPT)
#define FROM_CODE_ENCRYPTION_BIT 0x0800u
#else
#define FROM_CODE_ENCRYPTION_BIT 0x0000u
#endif

/* NVM code enable flags */
#if (defined NVM_SUPPORT && defined NVM_PLAIN)
#define NVM_CODE_PLAIN_SUPPORT_BITS 0x003Cu
#else
#define NVM_CODE_PLAIN_SUPPORT_BITS 0x0000u
#endif

#if (defined NVM_SUPPORT && defined NVM_ENCRYPT)
#define NVM_CODE_ENCRYPTION_BITS 0xF000u
#else
#define NVM_CODE_ENCRYPTION_BITS 0x0000u
#endif

/* Security code enable flags */
#ifdef SECURITY_SUPPORT 
#define SEC_CODE_SUPPORT_BIT 0x0100u
#else 
#define SEC_CODE_SUPPORT_BIT 0x0000u
#endif

/************************************************************/
/* Data file Tags to indicate M1, M3 or M7 data             */
/************************************************************/
#ifndef ENABLE_CODE_SPACE_OPTIMIZATION
#define M7 7u
#define M1 1u
#define P1 3u
#endif

#define CALIBRATION_BIT 0x10u
#define DUAL_KEY_BIT    0x20u
#define DAS_BIT         0x40u


extern DPUCHAR device_SD; /* Device specific number of SD tiles	*/
extern DPUINT device_rows; /* Device specific number of rows */

extern DPUINT dat_support_status;
extern DPUCHAR device_se_wait_cycle;
extern DPULONG device_security_flags;
extern DPUINT device_bsr_bit_length;
extern DPUCHAR AES_mode_value;

extern DPCHAR fpga_enabled; 
extern DPULONG fpga_silsig; 
/*
main entry function : user calls this function from their main function
*/
DPUCHAR dp_top_g3(void);
void dp_init_G3_vars(void);
void dp_get_dat_support_status(void);
void dp_check_dat_support_version(void);

void dp_check_action(void);
void dp_perform_action (void);
/* Action Function Definitions */
void dp_erase_action(void);
void dp_program_action(void);
void dp_verify_action(void);

void dp_check_device_ID(void);
void dp_check_device_ID_V85_DAT(void);
void dp_read_usercode_action(void);
void dp_test_reg_015_030_check(void);
void dp_frow_015_030_check(void);
void dp_initialize(void);
void dp_exit(void);
void dp_check_security_settings(void);
void dp_get_dat_dual_key_flag(void);
void dp_is_core_configured(void);
void dp_exe_authentication(void);
void dp_verify_enc_key(void);

void dp_erase(void);
void dp_exe_erase(void);
void dp_poll_erase(void);
void dp_poll_device(void);
void dp_read_urow(void);
void dp_program_urow(void);
void dp_init_aes(void);
void dp_set_aes_mode(void);
void dp_vnr(void);
void dp_load_bsr(void);
void dp_read_factory_row(void);
void dp_das_check(void);

#ifdef ENABLE_DISPLAY
void dp_device_info_action(void);
void dp_verify_device_info_action(void);
void dp_read_silsig(void);
void dp_output_security(void);
void dp_display_urow(void);
#endif
#endif /* INC_DPALG_H */


