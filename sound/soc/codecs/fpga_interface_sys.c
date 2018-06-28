
/* #include "../../../drivers/actel_directC/dpalg.h" */
/* Action Names -- match actions function */
/* These codes are passed to the main entry function "dp_top" to indicate 
* which action to perform */
#define DP_NO_ACTION_FOUND                      0u
#define DP_DEVICE_INFO_ACTION_CODE              1u
#define DP_READ_IDCODE_ACTION_CODE              2u
#define DP_ERASE_ACTION_CODE                    3u
#define DP_PROGRAM_ACTION_CODE                  5u
#define DP_VERIFY_ACTION_CODE                   6u
/* Array only actions */
#define DP_ENC_DATA_AUTHENTICATION_ACTION_CODE  7u
#define DP_ERASE_ARRAY_ACTION_CODE              8u
#define DP_PROGRAM_ARRAY_ACTION_CODE            9u
#define DP_VERIFY_ARRAY_ACTION_CODE             10u
/* FROM only actions */
#define DP_ERASE_FROM_ACTION_CODE               11u
#define DP_PROGRAM_FROM_ACTION_CODE             12u
#define DP_VERIFY_FROM_ACTION_CODE              13u
/* Security only actions */
#define DP_ERASE_SECURITY_ACTION_CODE           14u
#define DP_PROGRAM_SECURITY_ACTION_CODE         15u
/* NVM only actions */
#define DP_PROGRAM_NVM_ACTION_CODE              16u
#define DP_VERIFY_NVM_ACTION_CODE               17u
#define DP_VERIFY_DEVICE_INFO_ACTION_CODE       18u
#define DP_READ_USERCODE_ACTION_CODE            19u
/* For P1 device, The following two actions are only supported with data files
* generated form V86 or later.  ENABLE_V85_DAT_SUPPORT must be disabled */
#define DP_PROGRAM_NVM_ACTIVE_ARRAY_ACTION_CODE 20u
#define DP_VERIFY_NVM_ACTIVE_ARRAY_ACTION_CODE  21u
#define DP_IS_CORE_CONFIGURED_ACTION_CODE       22u
#define DP_PROGRAM_PRIVATE_CLIENTS_ACTION_CODE  23u
#define DP_VERIFY_PRIVATE_CLIENTS_ACTION_CODE   24u
#define DP_PROGRAM_PRIVATE_CLIENTS_ACTIVE_ARRAY_ACTION_CODE  25u
#define DP_VERIFY_PRIVATE_CLIENTS_ACTIVE_ARRAY_ACTION_CODE   26u


#ifdef CONFIG_LIDA_MACH_X5
#include <linux/switch_gpio_msp430.h>
#endif

#ifdef CONFIG_LIDA_MACH_X7II
#include <linux/switch_gpio_msp430_NP.h>
extern void es9028_set_thdc2(u32 data);
extern void es9028_set_thdc3(u32 data);
#endif

#ifdef CONFIG_SND_RK29_SOC_AK4490
#include "ak4490.h"
extern void ak4490_set_am_power(int on);
extern int ak4490_read_i2c_blk(u8 reg, u8* data,u8 len);
extern int ak4490_write_i2c_lr(u8 lr,u8 reg,u8 data);
#define DEVICE_REG_READ_BLK_AK4490(reg,data,len) ak4490_read_i2c_blk(reg,data,len)
#define DEVICE_REG_WRITE_AK4490(lr,reg,data) ak4490_write_i2c_lr(lr,reg,data)

#endif

#ifdef CONFIG_SND_RK29_SOC_ES9028
#include "es9028.h"
extern int es9028_read_i2c_blk(u8 reg, u8* data,u8 len);
extern int es9028_write_i2c(u8 reg,u8 data);
#define DEVICE_REG_READ_BLK_ES9028(reg,data,len) es9028_read_i2c_blk(reg,data,len)
#define DEVICE_REG_WRITE_ES9028(reg,data) es9028_write_i2c(reg,data)

#define REG_MAX_ES9028 116
#endif

#define REG_BITS      8
#define REG_DTYPE     u8
#define REG_MAX_AK4490   AK4490_MAX_REGISTERS
typedef REG_DTYPE reg_t ;

const static char AUDIO_BRIDGE_NAME[]="FPGA_ODM";
int gAndroid_boot_completed=0;

// 0 cox out
// 1 opt out
int opt_cox_out_flag=0;

struct sys_msg{
u32 gpiolevel; // 4 group bit 0-3 level bit 7-4 mask
u32 delay[4]; // us

/* u8 spi_data; */
u16 spi_data;
u8 spi_delay;
u16 flag;

    u8 lr;
    u16 reg;
    reg_t value;
    u8 reg_type;
    u8 read_write;
    u16 num;

    uint32_t dp_cmd;
};
static struct sys_msg msg;

static char  *Fw=NULL,*pbuf;
unsigned char action=0;
char valid_entry=0;
extern void actel_fpga_action(char* fw,unsigned char option_code) __attribute__((weak));
extern bool get_fpga_status(void);
extern void AudioBridge_enable(void);
extern void AudioBridge_disable(void);
static ssize_t sys_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
#ifdef PRINT_INT_INFO
    debug_flage = !debug_flage;
    if(debug_flage)
        printk("elan debug switch open\n");
    else
        printk("elan debug switch close\n");
#endif

    if(valid_entry)
        printk(KERN_WARNING "actel debug sent action\n");
        if(pbuf==Fw)
            actel_fpga_action(NULL,action);
        else
            actel_fpga_action(Fw,action);

    pbuf=Fw;
    valid_entry=0;

    return ret;
}

static ssize_t sys_debug_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t size)
{

    printk(KERN_WARNING "debug store get fw size %d\n",size);
    /* if(fw[0]==0x44 && fw[1]==0x65 && fw[2]==0x73 && fw[3]==0x69){ */
    // set Action_code
    if(size == 6){
        if(buf[0]=='c' && buf[1] == 'm' && buf[2] == 'd'){
            action=(buf[3]-48)*10+(buf[4]-48);
            if(action>=0 && action<27){
                valid_entry = 1;
                printk(KERN_WARNING"debug set action %d\n",action);
            }
            else
                printk(KERN_WARNING"debug set action %d not supported\n",action);

        }
    }
    else if (size == 4){
        printk("%x %X %x %x",buf[0],buf[1],buf[2],buf[3]);
        if(strcmp(buf,"fwo\n") == 0)
            printk(KERN_WARNING "set fw to ori");
        /* Fw=1; */
    }
    else if(size == 3){
        if(buf[0]=='c' && buf[1] == 'l' ){  
            action=0;
            valid_entry=0;
            pbuf=Fw;
        }else{
        }

    }else if(size){
        memcpy(pbuf,buf,size);
        pbuf+=size;
    } else{
        printk(KERN_WARNING "debug store get fw wrong %x %x %x %x \n",Fw[0],Fw[1],Fw[2],Fw[3]);
    }
    

    return size;
    


}
static DEVICE_ATTR(debug, S_IALLUGO, sys_debug_show,sys_debug_store);


static ssize_t sys_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    strcpy(buf,AUDIO_BRIDGE_NAME);

    return strlen(AUDIO_BRIDGE_NAME);
}
static DEVICE_ATTR(info, S_IRUGO, sys_info_show, NULL);

static ssize_t sys_fpga_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    if(get_fpga_status()){
        strcpy(buf,"fpga enabled");
    }else
        strcpy(buf,"fpga not enabled");

    return strlen(buf);
}
static DEVICE_ATTR(fpga_en, S_IRUGO, sys_fpga_show, NULL);


static ssize_t store_fpga_upgrade(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    char ss[8]={"NULL"};

    if (sscanf(buf, "%c%c 0x%x\n",&ss[0],&ss[1],&msg.dp_cmd) !=3){
        printk("DP cmd format error! \n");
        return -EINVAL;
    }
    ss[2]='\0';
    if(!strcmp(ss,"DP")){
        printk("cmd head : %s \n",ss);
        actel_fpga_action(NULL,msg.dp_cmd);
    }
    else
        printk("cmd head error: %s \n",ss);

    return count;
}
static DEVICE_ATTR(fpga, S_IRWXUGO,NULL,store_fpga_upgrade);


static ssize_t boot_completed_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t size)
{
    char cmd[size+1];
    if(size >0){
        memcpy(cmd,buf,size);
        cmd[size]='\0';
        if( strcmp(cmd,"boot_completed") == 0){
            gAndroid_boot_completed=1;
            printk("boot_completed_store get msg: <%s>\n",cmd);
        }else{
            printk("boot_completed_store unkown command: %s  %d\n",buf,size);
        }
    }

    return size;
}
static DEVICE_ATTR(boot, S_IRWXUGO,NULL, boot_completed_store);


static ssize_t send_cmd_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t size)
{
#if 0
    if (sscanf(buf, "0x%x %x %x %x %x %x %x %x\n",\
               (unsigned int *)&msg.gpiolevel,(unsigned int *)&msg.delay[0],\
               (unsigned int *)&msg.delay[1],(unsigned int *)&msg.delay[2],\
               (unsigned int *)&msg.delay[3],(unsigned int *)&msg.spi_data,\
               (unsigned int *)&msg.spi_delay,(unsigned int *)&msg.flag) != 8){
        printk("sys cmd format error\n");
        return -EINVAL;
    }
    printk("send cmd: gpiolevel 0x%x delay 0x%x 0x%x 0x%x 0x%x spi_data %x spi_delay 0x%x  flag %x\n",
           msg.gpiolevel,msg.delay[0],msg.delay[1],msg.delay[2],msg.delay[3],msg.spi_data,msg.spi_delay,msg.flag);
    /* elan_ts_send_cmd(private_ts->client, cmd, 4); */

    for(i=0;i<=3;i++){
        if((msg.flag >>i) & 0x01){
            gpio_sequence(msg.gpiolevel>>(i<<3) & 0x00FF,msg.delay[i]);
            }
    }
    if(msg.flag&0x10){
        /* spi_set(msg.spi_data); */
        audio_fpga_bridge_sendCommand(msg.spi_data);
    }
#endif

    if(sscanf(buf,"0x%x\n",(unsigned int *)&msg.spi_data)==1){

            switch(msg.spi_data){
            case 0xFFF:
                AudioBridge_enable();break;
            case 0xFF0:
                AudioBridge_disable();break;
            default:
                audio_fpga_bridge_sendCommand(msg.spi_data);

            }


    }


    /* return -EINVAL; */


    return size;
}

static ssize_t read_reg_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{

    ssize_t ret=0;
    /* u16 reg=0; */
    /* u8 i=0; */
    /* char rbuf[256]; */


    /* if(!msg.read_write){ */
        /* sprintf(buf,"TC READ: set read addr add read_write flag fist : %d\n",msg.read_write); */
        /* ret = strlen(buf) + 1; */
        /* return ret; */
    /* } */

    /* do{ */

        /* if(msg.reg_type == 32){ */

            /* reg value is not 32 bit,all regs are 8 bits */
            /* reg=msg.reg + (i<<2); */
            /* sys_regr32(sys_client, reg , &msg.value); */

        /* }else if(msg.reg_type==16){ */

            /* reg=msg.reg + (i<<1); */
            /* sys_regr16(sys_client, reg, &msg.value); */

        /* }else if(msg.reg_type==8){ */

            /* reg=msg.reg + i ; */
            /* sys_regr8(sys_client, reg, &msg.value); */

        /* }else{ */

        /* } */

        /* sprintf(rbuf,"TC READ:reg 0X%04X value 0X%08X\n",reg,msg.value ); */
        /* strcat(buf,rbuf); */

        /* i++; */
        /* msg.num--; */

    /* }while(msg.num!=0); */

    /* ret = strlen(buf) + 1; */
    return ret;

}
static DEVICE_ATTR(send_cmd, S_IALLUGO, read_reg_show, send_cmd_store);
#ifdef CONFIG_SND_RK29_SOC_AK4490
static ssize_t send_cmd_store_ak4490(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t size)
{


    /* echo reg16 value32 reg-type[8,16,32] read_write[0,1] num > xxxx
     *
     */

    if (sscanf(buf, "%c 0x%x 0x%x\n",&msg.lr,\
               (unsigned int *)&msg.reg,(unsigned int *)&msg.value) != 3){
        printk("device cmd format error : [LR] reg data,format is Hex \n");
        return -EINVAL;
    }
    printk("SEND CMD: DAC %c reg 0x%x value 0x%x \n", msg.lr, msg.reg,msg.value);

    DEVICE_REG_WRITE_AK4490(msg.lr, msg.reg, msg.value);


    return size;
}

static ssize_t read_reg_show_ak4490(struct device *dev,
                             struct device_attribute *attr, char *buf)
{

    ssize_t ret;
    u16 reg=0;
    char rbuf[PAGE_SIZE];
    char dbuf[256];

#if 0
    if(!msg.read_write){
        sprintf(buf,"TC READ: set read addr add read_write flag fist : %d\n",msg.read_write);
        ret = strlen(buf) + 1;
        return ret;
    }
#endif

    DEVICE_REG_READ_BLK_AK4490(0,dbuf,REG_MAX_AK4490);

    do{
        sprintf(rbuf,"READ:reg  0X%02X L 0X%02X R 0X%02X\n",reg,dbuf[reg] ,dbuf[reg+REG_MAX_AK4490]);
        strcat(buf,rbuf);

        reg++;

    }while(reg<REG_MAX_AK4490);

    ret = strlen(buf) + 1;
    return ret;

}

static DEVICE_ATTR(ak4490_reg, S_IALLUGO, read_reg_show_ak4490, send_cmd_store_ak4490);
static ssize_t store_ak4490_lpf(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t size)
{


    /* echo reg16 value32 reg-type[8,16,32] read_write[0,1] num > xxxx
     *
     */

    if (sscanf(buf, "0x%x\n",&msg.lr) != 1){
        printk("device cmd format error format is Hex [0x00 0x01 0x02 0x03] \n");
        return -EINVAL;
    }

    gak4490_defalult_lpf=msg.lr;
    ak4490_set_lpf(msg.lr);

    return size;
}

static ssize_t show_ak4490_lpf(struct device *dev,
                             struct device_attribute *attr, char *buf)
{

    ssize_t ret;
    u16 reg=0;
    char rbuf[PAGE_SIZE];
    char dbuf[256];

#if 0

    DEVICE_REG_READ_BLK_AK4490(0,dbuf,REG_MAX_AK4490);

    do{
        sprintf(rbuf,"READ:reg  0X%02X L 0X%02X R 0X%02X\n",reg,dbuf[reg] ,dbuf[reg+REG_MAX_AK4490]);
        strcat(buf,rbuf);

        reg++;

    }while(reg<REG_MAX_AK4490);

    ret = strlen(buf) + 1;
#endif
    return 0;

}

static DEVICE_ATTR(ak4490_lpf, S_IALLUGO, show_ak4490_lpf, store_ak4490_lpf);

#endif
#ifdef CONFIG_SND_RK29_SOC_ES9028
static ssize_t send_cmd_store_es9028(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t size)
{


    /* echo reg16 value32 reg-type[8,16,32] read_write[0,1] num > xxxx
     *
     */

    if (sscanf(buf, "0x%x=0x%x\n",(unsigned int *)&msg.reg,(unsigned int *)&msg.value) != 2){
        printk("device cmd format error : [LR] reg data,format is Hex \n");
        return -EINVAL;
    }
    printk("SEND CMD: DAC reg 0x%x set 0x%x \n",msg.reg,msg.value);

    DEVICE_REG_WRITE_ES9028(msg.reg, msg.value);


    return size;
}

static ssize_t read_reg_show_es9028(struct device *dev,
                             struct device_attribute *attr, char *buf)
{

    ssize_t ret;
    u16 reg=0;
    static char rbuf[PAGE_SIZE];
    char dbuf[256];

#if 0
    if(!msg.read_write){
        sprintf(buf,"TC READ: set read addr add read_write flag fist : %d\n",msg.read_write);
        ret = strlen(buf) + 1;
        return ret;
    }
#endif

    DEVICE_REG_READ_BLK_ES9028(0,dbuf,REG_MAX_ES9028);

    do{
        sprintf(rbuf,"READ:reg  %04X:  0X%02X  0X%02X 0X%02X  0X%02X\n",
                reg,dbuf[reg] ,dbuf[reg+1],dbuf[reg+2],dbuf[reg+3]);
        strcat(buf,rbuf);

        reg+=4;

    }while(reg<REG_MAX_ES9028);

    ret = strlen(buf) + 1;
    return ret;

}

static DEVICE_ATTR(es9028_reg, S_IALLUGO, read_reg_show_es9028, send_cmd_store_es9028);
#endif


#if defined(CONFIG_LIDA_MACH_X5) || defined(CONFIG_LIDA_MACH_X7II)
static ssize_t sys_mcu_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 ver=0;

    ver=mcu_get_fw_ver();
    sprintf(buf,"0x%02x",ver);

    return strlen(buf);
}
static DEVICE_ATTR(mcuver, S_IRUGO, sys_mcu_show, NULL);

static ssize_t msp430_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t size)
{


    uint32_t cmd;
    static uint8_t cmd_en=0;
    char str[128];

    str[0]='\n';
    if(sscanf(buf, "%s",str)){

        printk("store get string %s\n",str);
        if(!strcmp(str,"link_set_cmd")){

            cmd_en=1;
            return size;
        }

        if(!strcmp(str,"link_power_on")){
            link_set_force_poweron();
            return size;
        }

        if(!strcmp(str,"link_force_upgrade")){
            link_set_force_poweron();
            return size;
        }
    }


    if (sscanf(buf, "0x%x\n",&cmd) != 1){
        printk("device cmd format error format is Hex [0x00 0x01 0x02 0x03] \n");
        return -EINVAL;
    }

    if(cmd_en)
        link_set_status(cmd);
    return size;
}

static DEVICE_ATTR(link_cmd, S_IWUGO, NULL, msp430_store);
#endif

#if defined(CONFIG_LIDA_MACH_X7II)
static ssize_t thd_store(struct device *dev,
                         struct device_attribute *attr,
                         const char *buf, size_t size)
{

    int cmd =0;
    u8 c;
    u32 data;

    if(sscanf(buf, "0x%x",&cmd)){
        printk("thd store get cmd 0x%x\n",cmd);
    }else{
        printk("thd store get wrong cmd %s\n",buf);

    }
    // d3 d2 d1 d0
    // d2 ==2 set thd c2 
    // d2 ==3 set thd c3 
    // d1 d0 =u16 reg data

    c=(cmd>>16)&0xFF;
    data=cmd & 0xFFFF;

    switch(c){
    case 2:
            es9028_set_thdc2(data);
            printk("thd store set thdc2 %x\n",data);
        break;

    case 3:

            es9028_set_thdc3(data);
            printk("thd store set thdc3 %x\n",data);

        break;
    default:
            printk("thd store params error 0x%x\n",c);
        ;
    }
    return size;
}

static DEVICE_ATTR(dac_thd, S_IALLUGO, NULL, thd_store);


static ssize_t opt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    /* ver=mcu_get_fw_ver(); */
    sprintf(buf,"%d %s\n",opt_cox_out_flag,opt_cox_out_flag? "opt":"coax");

    return strlen(buf);
}

static ssize_t opt_store(struct device *dev,
                         struct device_attribute *attr,
                         const char *buf, size_t size)
{

    int cmd =0;
    if(sscanf(buf, "%d",&cmd)){
        printk("opt store get cmd %d\n",cmd);
    }else{

        printk("store get wrong cmd %d\n",cmd);

    }
    switch(cmd){
    case 0:
        opt_cox_out_flag=0;
        if(link_get_status(LINK_REPORT_PLAYING)){
            printk("opt store coax out \n");
            /* link_set_status(LINK_UN_OPT_OUT); */
            /* msleep(50); */
            link_set_status(LINK_EN_COAX_OUT);
        }else
            printk("opt store coax out link not playing\n");
        break;

    case 1:
        opt_cox_out_flag=1;
        if(link_get_status(LINK_REPORT_PLAYING)){
            printk("opt store opt out \n");
            /* link_set_status(LINK_UN_COAX_OUT); */
            /* msleep(50); */
            link_set_status(LINK_EN_OPT_OUT);
        }else
            printk("opt store opt out link not playing\n");

        break;
    default:
            printk("opt store params error %d\n",cmd);
        ;
    }
    return size;
}

static DEVICE_ATTR(link_opt, S_IALLUGO, opt_show, opt_store);

static ssize_t am_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    sprintf(buf,"%s\n","NULL");

    return strlen(buf);
}

static ssize_t am_gain_store(struct device *dev,
                         struct device_attribute *attr,
                         const char *buf, size_t size)
{

    int cmd =0;
    if(sscanf(buf, "%d",&cmd)){
        printk("am gain store get cmd %d\n",cmd);
    }else{

        printk("am gain store get wrong cmd %d\n",cmd);

    }
    switch(cmd){
    case 0:
            link_set_status(LINK_AM_GAIN_LOW);
        break;

    case 1:
            link_set_status(LINK_AM_GAIN_HIGH);
        break;
    default:
            printk("am gain store params error %d\n",cmd);
        ;
    }
    return size;
}

static DEVICE_ATTR(link_am_gain, S_IALLUGO, am_gain_show, am_gain_store);
#endif


static struct attribute *sysfs_attrs_ctrl[] = {
&dev_attr_debug.attr,
&dev_attr_info.attr,
&dev_attr_fpga_en.attr,
&dev_attr_boot.attr,
&dev_attr_send_cmd.attr,
&dev_attr_fpga.attr,
#ifdef CONFIG_SND_RK29_SOC_AK4490
&dev_attr_ak4490_reg.attr,
&dev_attr_ak4490_lpf.attr,
#endif

#ifdef CONFIG_SND_RK29_SOC_ES9028
&dev_attr_es9028_reg.attr,
#endif

#if defined(CONFIG_LIDA_MACH_X5) || defined(CONFIG_LIDA_MACH_X7II)
&dev_attr_mcuver.attr,
&dev_attr_link_cmd.attr,
#endif
#if defined(CONFIG_LIDA_MACH_X7II)
&dev_attr_link_opt.attr,
&dev_attr_link_am_gain.attr,
&dev_attr_dac_thd.attr,

#endif
NULL
};
static struct attribute_group sys_attribute_group[] = {
    {.attrs = sysfs_attrs_ctrl },
};

struct kobject* sys_debug_kobj=NULL;
static void sys_node_init(void)
{
    int ret ;
    /* struct elan_ts_data *ts = private_ts; */

    sys_debug_kobj= kobject_create_and_add("audio_bridge", NULL) ;
    if (sys_debug_kobj== NULL){
        printk(KERN_ERR "[sys_debug]%s: kobject_create_and_add failed\n", __func__);
        return;
    }
    ret = sysfs_create_group(sys_debug_kobj, sys_attribute_group);
    if (ret < 0) {
        printk(KERN_ERR "[sys_debug]%s: sysfs_create_group failed\n", __func__);
    }

    Fw=kzalloc(1024*12,GFP_KERNEL);
    pbuf=Fw;
   if(!Fw){
       printk("fpga_interface ERROR no mem\n");
       return ;
   }

}

#if 0
static void sys_node_deinit(void)
{
    if(sys_debug_kobj){
        sysfs_remove_group(sys_debug_kobj, sys_attribute_group);
        kobject_put(sys_debug_kobj);
    }
}
#endif
