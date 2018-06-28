/* ************************************************************************ */
/*                                                                          */
/*  DirectC         Copyright (C) Microsemi Corporation 2015                */
/*  Version 3.2     Release date  October 30, 2015                          */
/*                                                                          */
/* ************************************************************************ */
/*                                                                          */
/*  Module:         dpuser.c												*/
/*                                                                          */
/*  Description:    user specific file containing JTAG interface functions  */
/*                  and delay function                                      */
/*                                                                          */
/****************************************************************************/
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
#include "dpalg.h"
#include "dputil.h"

/* 
 * User attention:
 * Include files needed to support hardware JTAG interface operations.
 * 
*/
/* This variable is used to select external programming vs IAP programming */
DPUCHAR hardware_interface = GPIO_SEL;
DPUCHAR enable_mss_support = FALSE;

/*
 * User attention:
 * jtag_port_reg: 	8 bit Static variable to keep track of the state of all the JTAG pins 
 * 					at all times during the programming operation.
 * Note: User check the variable size to make sure it can cover the hardware IO register. 
 * 
*/
static DPUCHAR jtag_port_reg;
/*
 * User attention: 
 * Module: jtag_inp
 * 		purpose: report the logic state of tdo jtag pin
 * Arguments: None
 * Return value: 8 bit value
 * 		0, 0x80
 * 
*/
DPUCHAR jtag_inp(void)
{
    DPUCHAR tdo = 0u;
    DPUCHAR ret = 0x80u;
    //zzdts
    tdo=gpio_get_value(GPIO_TDO)==1? TDO:0;

	/* User Specific Code 
    .
    .
    */
    /* For Parallel global_buf1er board, the logic is reversed. */
	if ((tdo&TDO) == 0u)
        ret = 0;
    return ret;
}
/*
 * User attention: 
 * Module: jtag_outp
 * 		purpose: Set the JTAG port (all JTAG pins)
 * Arguments: 8 bit value containing the new state of all the JTAG pins
 * Return value: None
 * 
*/
void jtag_outp(DPUCHAR outdata)
{
        gpio_set_value(GPIO_TRST,!!(outdata & TRST));
        gpio_set_value(GPIO_TMS,!!(outdata & TMS));
        gpio_set_value(GPIO_TDI,!!(outdata & TDI));
        gpio_set_value(GPIO_TCK,!!(outdata & TCK));
	/* User Specific Code 
    .
    .
    */
}

/*
 * No need to change this function
 * Module: dp_jtag_init
 * 		purpose: Set tck and trstb pins to logic level one
 * Arguments:
 * 		None
 * Return value: None
 * 
*/
void dp_jtag_init(void)
{
	jtag_port_reg = TCK | TRST; 
	jtag_outp(jtag_port_reg);
}

/*
 * No need to change this function
 * Module: dp_jtag_tms
 * 		purpose: Set tms pin to a logic level one or zero and pulse tck.
 * Arguments: 
 * 		tms: 8 bit value containing the new state of tms
 * Return value: None
 * Constraints: Since jtag_outp function sets all the jtag pins, jtag_port_reg is used 
 * 				to modify the required jtag pins and preseve the reset.
 * 
*/
void dp_jtag_tms(DPUCHAR tms)		 
{	
	jtag_port_reg &= ~(TMS | TCK);
    jtag_port_reg |= (tms ? TMS : 0u);
	jtag_outp(jtag_port_reg);
	jtag_port_reg |= TCK;
	jtag_outp(jtag_port_reg);
}

/*
 * No need to change this function
 * Module: dp_jtag_tms_tdi
 * 		purpose: Set tms amd tdi pins to a logic level one or zero and pulse tck.
 * Arguments: 
 * 		tms: 8 bit value containing the new state of tms
 * 		tdi: 8 bit value containing the new state of tdi
 * Return value: None
 * Constraints: Since jtag_outp function sets all the jtag pins, jtag_port_reg is used 
 * 				to modify the required jtag pins and preseve the reset.
 * 
*/
void dp_jtag_tms_tdi(DPUCHAR tms, DPUCHAR tdi)
{
	jtag_port_reg &= ~(TMS | TCK | TDI);
    jtag_port_reg |= ((tms ? TMS : 0u) | (tdi ? TDI : 0u));
	jtag_outp(jtag_port_reg);
	jtag_port_reg |= TCK;
	jtag_outp(jtag_port_reg);
}

/*
 * No need to change this function
 * Module: dp_jtag_tms_tdi_tdo
 * 		purpose: Set tms amd tdi pins to a logic level one or zero, 
 * 				 pulse tck and return tdi level
 * Arguments: 
 * 		tms: 8 bit value containing the new state of tms
 * 		tdi: 8 bit value containing the new state of tdi
 * Return value: 
 * 		ret: 8 bit variable ontaining the state of tdo.
 * Valid return values: 
 * 		0x80: indicating a logic level high on tdo
 * 		0: indicating a logic level zero on tdo
 * Constraints: Since jtag_outp function sets all the jtag pins, jtag_port_reg is used 
 * 				to modify the required jtag pins and preseve the reset.
 * 
*/
DPUCHAR dp_jtag_tms_tdi_tdo(DPUCHAR tms, DPUCHAR tdi)
{
    DPUCHAR ret = 0x80u;
	jtag_port_reg &= ~(TMS | TCK | TDI);
    jtag_port_reg |= ((tms ? TMS : 0u) | (tdi ? TDI : 0u));
	jtag_outp(jtag_port_reg);
	ret = jtag_inp() ;
	jtag_port_reg |= TCK;
	jtag_outp(jtag_port_reg);
	return ret;
}

/*
 * User attention: 
 * Module: dp_delay
 * 		purpose: Execute a time delay for a specified amount of time.
 * Arguments: 
 * 		microseconeds: 32 bit value containing the amount of wait time in microseconds.
  * Return value: None
 * 
*/
void dp_delay(DPULONG microseconds)
{

    static long long delay=0;
    DPULONG mdelay =microseconds;
    microseconds=microseconds/20;
    if(!microseconds)
        microseconds=1;
    delay+=microseconds;
#if 0
    /* printk(KERN_WARNING	"JTAG Msg de_delay: %d mdelay %u all%lld\n",microseconds,mdelay,delay); */
    if(microseconds <10)
        mdelay(microseconds);
    else
        msleep(microseconds);
#else
    udelay(microseconds*10);
#endif
	/* User Specific Code 
    .
    .
    */
}

#ifdef ENABLE_DISPLAY
void dp_display_text(DPCHAR *text)
{
    printk(KERN_WARNING	"JTAG Msg: %s \n",text);
    /* User Specific Code 
    .
    .
    */
}

void dp_display_value(DPULONG value,DPUINT descriptive)
{
    switch(descriptive){
    case HEX:
        printk(KERN_WARNING	"JTAG value HEX: 0x%x \n",value);
        break;
    case DEC:
        printk(KERN_WARNING	"JTAG value DEC: %d \n",value);
        break;
    case CHR:
        printk(KERN_WARNING	"JTAG value CHR: %c \n",value);
        break;
    default:
        ;
    }
	/* User Specific Code 
    .
    .
    */
}
void dp_display_array(DPUCHAR *value,DPUINT bytes, DPUINT descriptive)
{
    int i;
    int m,n;
    m=bytes/8;
    n=bytes%8;
    static char str[1024];
    char * pstr=str;
    DPUCHAR *pvalue=value;



    switch(descriptive){
    case HEX:
        /* printk(KERN_WARNING	"JTAG array HEX: \n\t\t"); */
        sprintf(pstr,"HEX:\n");
        pstr+=strlen(pstr);
        for(;m>0;m--){
            sprintf(pstr,"\t%02x %02x %02x %02x %02x %02x %02x %02x\n",
                    pvalue[0],pvalue[1],pvalue[2],pvalue[3],pvalue[4],pvalue[5],pvalue[6],pvalue[7]);
            pvalue+=8;
            pstr+=strlen(pstr);
        }
        sprintf(pstr,"\t");
        pstr+=strlen(pstr);
        for(;n>0;n--){
            sprintf(pstr,"%02x ",pvalue[0]);
            pvalue++;
            pstr+=strlen(pstr);
        }
            printk(KERN_WARNING"%s",str);
            
            

        /* for(i=0;i<bytes;i++){ */
            /* printk(KERN_WARNING	"0x%x",value[i]); */
            /* if((i+1)%8 == 0) */
                /* printk(KERN_WARNING	"\n \t\t"); */
        /* } */
        break;
    case DEC:
        /* printk(KERN_WARNING	"JTAG array DEC: \n\t\t"); */
        sprintf(pstr,"DEC:\n");
        pstr+=strlen(pstr);
        for(;m>0;m--){
            sprintf(pstr,"\t%02d %02d %02d %02d %02d %02d %02d %02d\n",
                    pvalue[0],pvalue[1],pvalue[2],pvalue[3],pvalue[4],pvalue[5],pvalue[6],pvalue[7]);
            pvalue+=8;
            pstr+=strlen(pstr);
        }
        sprintf(pstr,"\t");
        pstr+=strlen(pstr);
        for(;n>0;n--){
            sprintf(pstr,"%02d ",pvalue[0]);
            pvalue++;
            pstr+=strlen(pstr);
        }
            printk(KERN_WARNING"%s",str);
         
        /* for(i=0;i<bytes;i++){ */
            /* printk(KERN_WARNING	"%d",value[i]); */
            /* if((i+1)%8 == 0) */
                /* printk(KERN_WARNING	"\n \t\t"); */
        /* } */
        break;
    case CHR:
        /* printk(KERN_WARNING	"JTAG array CHR: \n\t\t"); */
        sprintf(pstr,"CHR:\n");
        pstr+=strlen(pstr);
        for(;m>0;m--){
            sprintf(pstr,"\t%c %c %c %c %c %c %c %c\n",
                    pvalue[0],pvalue[1],pvalue[2],pvalue[3],pvalue[4],pvalue[5],pvalue[6],pvalue[7]);
            pvalue+=8;
            pstr+=strlen(pstr);
        }
        sprintf(pstr,"\t");
        pstr+=strlen(pstr);
        for(;n>0;n--){
            sprintf(pstr,"%c ",pvalue[0]);
            pvalue++;
            pstr+=strlen(pstr);
        }
            printk(KERN_WARNING"%s",str);
 
        /* for(i=0;i<bytes;i++){ */
            /* printk(KERN_WARNING	"%c",value[i]); */
            /* if((i+1)%8 == 0) */
                /* printk(KERN_WARNING	"\n \t\t"); */
        /* } */
        break;
    default:
        ;
    }
	/* User Specific Code 
    .
    .
    */
}
#endif
