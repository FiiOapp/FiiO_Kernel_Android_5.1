/* ************************************************************************ */
/*                                                                          */
/*  DirectC         Copyright (C) Microsemi Corporation 2015                */
/*  Version 3.2     Release date  October 30, 2015                          */
/*                                                                          */
/* ************************************************************************ */
/*                                                                          */
/*  Module:         dpalg.c                                                 */
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
#include "dputil.h"
#include "dpalg.h"
#include "dpG3alg.h"
#include "dpG4alg.h"
#include "dpjtag.h"
#include <mach/board.h>

#include "a3p_pdat_X5.h"
#include "a3p_pdat_X7II_O3.h"
#include "a3p_pdat_md.h"
#include <linux/err.h>

extern int gBoardType;
extern int board_boot_mode(void);
DPUCHAR Action_code; /* used to hold the action codes as defined in dpalg.h */
DPUCHAR Action_done; /* used to hold the action codes as defined in dpalg.h */
DPUCHAR opcode; /* Holds the opcode value of the IR register prior to loading */

DPULONG device_ID;  /* Holds the device ID */
DPUCHAR device_rev; /* Holds the device revision */
DPUCHAR device_family = 0U;    /* Read from the data file AFS, or G3 */
DPUINT device_bsr_bit_length; /* Holds the bit length of the BSR register */

/* DataIndex variable is used to keep track of the position of the data 
* loaded in the file 
*/
DPULONG DataIndex;   

/* error_code holds the error code that could be set in the programming 
* functions 
*/
DPUCHAR error_code; 
extern DPULONG dp_get_silsig();

#define HW_TYPE_UNKNOWN -1
#define HW_TYPE_OSC_2IN 0
#define HW_TYPE_OSC_3IN 1
int HW_tpye=HW_TYPE_OSC_2IN;





struct directc_data 
{
	int		reset_gpio;
	int		last_point_num;
	struct work_struct 	action_event_work;
	struct workqueue_struct *directc_workqueue;
};

static struct directc_data *gpDirectC;

#include "../../../sound/soc/codecs/fpga_audio_interface.h"
extern int audio_fpga_bridge_sendCommand( u16 cmd);
extern void dp_exe_erase(void);
extern void AudioBridge_power(int onoff);
extern int audio_fpga_bridge_sendCommandPre();

DPUCHAR dp_top (void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nIdentifying device...");
    #endif
    
    goto_jtag_state(JTAG_TEST_LOGIC_RESET,0u);
    error_code = DPE_CODE_NOT_ENABLED;
    Action_done = FALSE;
    
    #ifdef ENABLE_G3_SUPPORT
    error_code = DPE_SUCCESS;
    dp_read_idcode();
    dp_check_device_ID();
    if (error_code == DPE_SUCCESS)
    {
        dp_top_g3();
        Action_done = TRUE;
    }	
    #endif
    
    #ifdef ENABLE_G4_SUPPORT
    if (Action_done == FALSE)
    {
        error_code = DPE_SUCCESS;
        dp_read_idcode();
        dp_check_G4_device_ID();
        if (error_code == DPE_SUCCESS)
        {
            dp_top_g4 ();
            Action_done = TRUE;
        }
    }
    #endif
    
    return error_code;
}

void dp_read_idcode(void)
{
    opcode = IDCODE;
    IRSCAN_in();
    goto_jtag_state(JTAG_RUN_TEST_IDLE,0u);
    DRSCAN_out(IDCODE_LENGTH, (DPUCHAR*)DPNULL, global_buf1);
    device_ID = (DPULONG)global_buf1[0] | (DPULONG)global_buf1[1] << 8u | 
    (DPULONG)global_buf1[2] << 16u | (DPULONG)global_buf1[3] << 24u;
    device_rev = (DPUCHAR) (device_ID >> 28);
    
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nActID = ");
    dp_display_value(device_ID,HEX);
    #endif
    
    
    return;
}

#ifdef ENABLE_DISPLAY
void dp_read_idcode_action(void)
{
    return;
}
#endif

static int actel_directc_gpio_init(void)
{
	int	ret;
    ret=gpio_request(GPIO_TCK,"actel jtag tck");
    if(ret){
        gpio_free(GPIO_TCK);
        printk(KERN_ERR " actel jtag tck request failed!!\n");
        return ret;
    }
    ret=gpio_request(GPIO_TDI,"actel jtag tdi");
    if(ret){
        gpio_free(GPIO_TDI);
        printk(KERN_ERR " actel jtag tdi request failed!!\n");
        return ret;
    }
    ret=gpio_request(GPIO_TDO,"actel jtag tdo");
    if(ret){
        gpio_free(GPIO_TDO);
        printk(KERN_ERR " actel jtag tdo request failed!!\n");
        return ret;
    }
    ret=gpio_request(GPIO_TMS,"actel jtag tms");
    if(ret){
        gpio_free(GPIO_TMS);
        printk(KERN_ERR " actel jtag tms request failed!!\n");
        return ret;
    }
    ret=gpio_request(GPIO_TRST,"actel jtag trst");
    if(ret){
        gpio_free(GPIO_TRST);
        printk(KERN_ERR " actel jtag trst request failed!!\n");
        return ret;
    }


    gpio_direction_input(GPIO_TDO);
    gpio_direction_output(GPIO_TCK,1);
    gpio_direction_output(GPIO_TMS,1);
    gpio_direction_output(GPIO_TRST,1);
    gpio_direction_output(GPIO_TDI,0);

#if 0
    gpio_direction_input(GPIO_TDO);
    gpio_direction_input(GPIO_TCK);
    gpio_direction_input(GPIO_TMS);
    gpio_direction_input(GPIO_TRST);
    gpio_direction_input(GPIO_TDI);
#endif

    printk("actel_directc_gpio_init success\n");

    return 0;


}

static char* actel_get_hw_fw(void)
{
	int	ret,vp,vd;

    if(gBoardType == BOARD_X7II){
        ret=gpio_request(GPIO_HW,"actel gpio hw type");
        if(ret){
            gpio_free(GPIO_HW);
            printk(KERN_ERR " actel gpio hw type gpio request failed!!\n");
            return ERR_PTR( -EBUSY);
        }

        gpio_direction_input(GPIO_HW);

        gpio_pull_updown(GPIO_HW,GPIOPullUp);
        msleep(5);
        vp=gpio_get_value(GPIO_HW);

        gpio_pull_updown(GPIO_HW,GPIOPullDown);
        msleep(5);
        vd=gpio_get_value(GPIO_HW);

        if( (vp == 1) && (vd == 0) ){
            HW_tpye=HW_TYPE_OSC_2IN;
        }
        else if( (vp == 0) && (vd == 0) ){
            HW_tpye=HW_TYPE_OSC_3IN;
        }else{

            HW_tpye=HW_TYPE_UNKNOWN;

            printk(KERN_WARNING "\n*****************************\n");
            printk(KERN_WARNING "Board FPGA type unknow !!!!!!! vp %d , vd %d\n",vp,vd);
            printk(KERN_WARNING "*****************************\n\n");
        }
    }




// get fw
    if(gBoardType == BOARD_X7){

        printk("FPGA FW : BOARD_X7 a3p_pdat_md \n");
        return a3p_pdat_md;

    }else if(gBoardType == BOARD_X5III){

        printk("FPGA FW : BOARD_X5 a3p_pdat_x5 \n");
        return a3p_pdat_x5;

    }else if(gBoardType == BOARD_X7II){



        switch(HW_tpye){
        case HW_TYPE_OSC_2IN:

            printk("FPGA FW : BOARD_X7II a3p_pdat_md \n");
            return a3p_pdat_md;

        case HW_TYPE_OSC_3IN:
            printk("FPGA FW : BOARD_X7II a3p_pdat_X7II_O3\n");
            return a3p_pdat_X7II_O3;

        case HW_TYPE_UNKNOWN:

            printk("FPGA FW : BOARD_X7II unknow defined fw !!! \n");
            return NULL;

        }


    }else{
            return NULL;
    }
}


static void  directc_delaywork_func(struct work_struct *work)
{

	struct directc_data* pdirectc= container_of(work, struct directc_data, action_event_work);
    int ret;
    bool upgrade=false;
    DPUCHAR action_back;
    DPULONG fw_silsig;

#ifdef CONFIG_LIDA_MACH_X7II
    AudioBridge_power(1);
#endif
    fw_silsig=dp_get_silsig();

    // silsig is a 4 Byte data
    // example 0xF1530101
    // bit[28:31] F: mean this fw is generated by fiio
    // bit[24:27] 1: mean this fw is built for 'X' series product
    // bit[16:23] 53 mean the fw is built for five service third generation  product
    // bit[8:15]  01: indicate the protocol verion 01
    // bit[0:7]   01: indicate the firmware verion 01
    // directc will confirm the right FPGA type

    action_back=Action_code;
    printk(KERN_WARNING"\n*************************************\n");
    printk(KERN_WARNING"actel_directc  action  info\n");
    Action_code=DP_DEVICE_INFO_ACTION_CODE;
    ret=dp_top();
    if(!fpga_silsig && fw_silsig){

        upgrade=true;
        printk(KERN_WARNING "dp upgrade, upgrade,fpga silsig %x  fw silsig %x\n",fpga_silsig,fw_silsig);
    }else if((fpga_silsig & 0xFFFF0000) != (fw_silsig & 0xFFFF0000)){

        upgrade=false;
        /* upgrade=true; */
        printk(KERN_WARNING "dp upgrade ,  upgrade ,fpga silsig %x   fw silsig %x\n",fpga_silsig,fw_silsig);

    }else if(fpga_silsig < fw_silsig){

        upgrade=true;
        printk(KERN_WARNING "dp upgrade, upgrade,fpga silsig %x  fw silsig %x\n",fpga_silsig,fw_silsig);
    }else if(fpga_silsig == fw_silsig){
        if((fpga_silsig & 0xFFFF) == 0xFFFF){
            upgrade=true;
            printk(KERN_WARNING "dp upgrade, debug fw,fpga silsig %x  fw silsig %x\n",fpga_silsig,fw_silsig);
        }else
            printk(KERN_WARNING "dp upgrade,fw is the newest fpga silsig %x  fw silsig %x\n",fpga_silsig,fw_silsig);
    }else{

        printk(KERN_WARNING "dp upgrade,not upgrade,  fpga silsig %x  fw silsig %x\n",fpga_silsig,fw_silsig);
    }


    Action_code=action_back;
    switch(Action_code){

    case DP_ACTION_X5_INFO:

        printk(KERN_WARNING"\n*************************************\n");
        printk(KERN_WARNING"actel_directc  action  info\n");
        Action_code=DP_DEVICE_INFO_ACTION_CODE;
        ret=dp_top();
        printk(KERN_WARNING "dp_top end with %x\n",ret);

        printk(KERN_WARNING"\n*************************************\n");
        printk(KERN_WARNING"actel_directc  action  usercode\n");
        Action_code=DP_READ_USERCODE_ACTION_CODE;
        ret=dp_top();
        printk(KERN_WARNING "dp_top end with %x\n",ret);

        break;
    case DP_ACTION_X5_UPGRADE:
    case DP_ACTION_X7II_UPGRADE:


       if((ret==0) &&((fpga_enabled == 0) || upgrade )){


            printk(KERN_WARNING"actel_directc action program \n");
            Action_code=DP_PROGRAM_ACTION_CODE;
            dp_top();

            printk(KERN_WARNING"*************************************\n");
            printk(KERN_WARNING"actel_directc action info reread\n");
            Action_code=DP_DEVICE_INFO_ACTION_CODE;
            ret=dp_top();
        }
        break;
    default:
        printk(KERN_WARNING"*************************************\n");
        printk(KERN_WARNING"actel_directc  action %x\n",Action_code);
        ret=dp_top();
        printk(KERN_WARNING "dp_top end with %x\n",ret);

    }


    printk(KERN_WARNING"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
    printk(KERN_WARNING"actel_directc action end\n");

#ifdef CONFIG_LIDA_MACH_X7II
    AudioBridge_power(0);
#endif
    /* audio_fpga_bridge_sendCommand(AUDIO_SAMPLERATE_48K_S512FS); */
    audio_fpga_bridge_sendCommandPre();


}
void actel_fpga_action(char* fw,unsigned char option_code)
{
    if(fw)
        image_buffer=fw;
    else if( image_buffer ==NULL)
       image_buffer= a3p_pdat_md;


    Action_code=option_code;
    queue_work(gpDirectC->directc_workqueue, &gpDirectC->action_event_work);

}
EXPORT_SYMBOL_GPL(actel_fpga_action);

/* static char imgs[256]; */
static int __init actel_directc_probe(struct platform_device *pdev)
{
    int ret;
    static struct directc_data *pdirectc;

    printk("actel_directc_probe \n");
    ret=actel_directc_gpio_init();

    pdirectc = kzalloc(sizeof(struct directc_data),GFP_KERNEL);
    if(!pdirectc){
        printk(KERN_ERR"directc :alloc data failed.\n");
        ret = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    INIT_WORK(&pdirectc->action_event_work, directc_delaywork_func);
    pdirectc->directc_workqueue= create_singlethread_workqueue("directc");
    if (!pdirectc->directc_workqueue) {
        ret= -ESRCH;
        goto exit_init_workqueue_failed;
    }

    gpDirectC=pdirectc;

    image_buffer=actel_get_hw_fw();
    if(IS_ERR_OR_NULL(image_buffer)) {

        printk("actel fpga action fw error \n");

        return 0;
    }


#ifdef CONFIG_LIDA_MACH_X5
    if(board_boot_mode() != BOOT_MODE_RECOVERY)
        actel_fpga_action(NULL,DP_ACTION_X5_UPGRADE);
#endif
#ifdef CONFIG_LIDA_MACH_X7II
    if(board_boot_mode() != BOOT_MODE_RECOVERY)
        actel_fpga_action(NULL,DP_ACTION_X7II_UPGRADE);
#endif

    printk("actel_directc_probe  end\n");
    return  0;

exit_init_workqueue_failed:
    kfree(pdirectc);
exit_alloc_data_failed:

    return ret;

}


static int __exit actel_directc_remove(struct platform_device *pdev)
{
	int				status=0;

	gpio_free(GPIO_TCK);
	gpio_free(GPIO_TMS);
	gpio_free(GPIO_TDO);
	gpio_free(GPIO_TDI);
	gpio_free(GPIO_TRST);

    gpDirectC=NULL;
	return status;
}

/* MODULE_ALIAS("platform:" DRIVER_NAME); */

static struct platform_driver actel_directc_driver = {
	.driver.name	="actel_fgpa_jtag",
	.driver.owner	= THIS_MODULE,
	.remove		= __exit_p(actel_directc_remove),
};

static int __init actel_directc_init(void)
{
	return platform_driver_probe(&actel_directc_driver , actel_directc_probe);
}

late_initcall(actel_directc_init);

static void __exit actel_directc_exit(void)
{
	platform_driver_unregister(&actel_directc_driver);
}
module_exit(actel_directc_exit);


MODULE_DESCRIPTION("Actel directc driver");
MODULE_AUTHOR("actel");
MODULE_LICENSE("GPL");
