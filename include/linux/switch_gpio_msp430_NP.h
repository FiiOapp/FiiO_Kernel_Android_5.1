#ifndef __SWITCH_GPIO_MSP430__
#define  __SWITCH_GPIO_MSP430__

#include <linux/switch.h>

struct gpio_switch_data {
    struct switch_dev sdev;
    unsigned gpio;
    const char *name_on;
    const char *name_off;
    const char *state_on;
    const char *state_off;
    int irq;
    int l_count;
    struct work_struct work;
    struct work_struct power_work;
    bool bool_dc_detect;
    struct i2c_client *client;

    struct switch_dev headset;
    struct switch_dev lineout;
    struct switch_dev dc_detect;
    struct switch_dev line_control;
    struct switch_dev accessory_dev;
    struct switch_dev balance_dev;
    struct switch_dev amp_card;

    struct work_struct sdcard_work;
    // struct work_struct key_work;
    struct delayed_work  key_work;

    struct work_struct mcu_update_work;
    //add for adc
    struct timer_list adc0_timer;
    struct adc_client *adc0_client;
    struct timer_list adc2_timer;
    struct adc_client *adc2_client;
    int dock_count;
    int adc0_value;
    int dock_out_count;
    int am_gain;

    // current output device
    u8 phone_out;
    u8 line_out;
    u8 bah_out;
    u8 opt_out;
    u8 coax_out;
    u8 dock_out;

    // u8 am module
    u8 am_module;

    //add for audio link status
    u8 mcu_working;
    u8 mcu_busy;
    u8 analog_power;
    u8 dac_power;
    u8 start_play;

    u8 single_dc_error;
    u8 bah_dc_error;
    u8 po_mute;
    u8 lo_mute;
    u8 mcu_firmware_version;
    u8 dock_en;
    u8 mcu_force_upgrade;

    u8 is_power_process
};

enum link_status{

    LINK_FORCE_MCU_UPGRADE,

    LINK_EN_PHONE_OUT,
    LINK_UN_PHONE_OUT,
    LINK_EN_LINE_OUT,
    LINK_UN_LINE_OUT,
    LINK_EN_OPT_OUT,
    LINK_UN_OPT_OUT,
    LINK_EN_COAX_OUT,
    LINK_UN_COAX_OUT,
    LINK_EN_DOCK_OUT, // detect dock in
    LINK_UN_DOCK_OUT, // detect dock out
    LINK_FORCE_POWER_ON,
    LINK_REG20_MAX, // out en 


    LINK_EN_OTG, // 0x0b
    LINK_UN_OTG,
    LINK_EN_DAC,
    LINK_UN_DAC,
    LINK_EN_AUPOWER,    //auto poweron  0x0F
    LINK_UN_AUPOWER,    //auto poweron
    LINK_EN_APOWER_DOWN, // analog_power down 
    LINK_UN_APOWER_DOWN, // analog_power down
    LINK_AM_GAIN_LOW,
    LINK_AM_GAIN_HIGH,
    LINK_MUTE_LO,
    LINK_UNMUTE_LO,
    LINK_MUTE_PO,
    LINK_UNMUTE_PO,
    LINK_PLAY_START,  // prepare to playing only set to 1 is available
    LINK_REG21_MAX,   // power && mute control

    LINK_MCU_WAKEUP,
    LINK_MCU_STANDBY,
    LINK_EN_DBW,     //enable headset line control
    LINK_UN_DBW,     //disable headset line control
    LINK_WIRE_CTL_DISABLE,
    LINK_WIRE_CTL_ENABLE,
    LINK_REG22_MAX,

    // get link status enum

    LINK_REPORT_MCU_WORK,
    LINK_REPORT_DC_ERROR,
    LINK_REPORT_MUTE_ON,
    LINK_REPORT_MCU_BUSY,
    LINK_REPORT_DAC_PWR,
    LINK_REPORT_ANALOG_PWR,
    LINK_REPORT_PLAYING,
    LINK_REPORT_POWER_PROCESS,

    LINK_ANALOG_POWER,
    LINK_DAC_POWER_ON,
    LINK_FIRST_PLAY,
};


#define CSNAME(cmd) {cmd,#cmd}
struct cmdstring{
    int cmd;
    char name[64];
};

static struct cmdstring cs[]={

    CSNAME(LINK_FORCE_MCU_UPGRADE),
    CSNAME(LINK_EN_PHONE_OUT),
    CSNAME(LINK_UN_PHONE_OUT),
    CSNAME(LINK_EN_LINE_OUT),
    CSNAME(LINK_UN_LINE_OUT),
    CSNAME(LINK_EN_OPT_OUT),
    CSNAME(LINK_UN_OPT_OUT),
    CSNAME(LINK_EN_COAX_OUT),
    CSNAME(LINK_UN_COAX_OUT),
    CSNAME(LINK_EN_DOCK_OUT),
    CSNAME(LINK_UN_DOCK_OUT),
    CSNAME(LINK_FORCE_POWER_ON),
    CSNAME(LINK_REG20_MAX),


    CSNAME(LINK_EN_OTG),
    CSNAME(LINK_UN_OTG),
    CSNAME(LINK_EN_DAC),
    CSNAME(LINK_UN_DAC),
    CSNAME(LINK_EN_AUPOWER),
    CSNAME(LINK_UN_AUPOWER),
    CSNAME(LINK_EN_APOWER_DOWN),
    CSNAME(LINK_UN_APOWER_DOWN),
    CSNAME(LINK_AM_GAIN_LOW),
    CSNAME(LINK_AM_GAIN_HIGH),
    CSNAME(LINK_MUTE_LO),
    CSNAME(LINK_UNMUTE_LO),
    CSNAME(LINK_MUTE_PO),
    CSNAME(LINK_UNMUTE_PO),
    CSNAME(LINK_PLAY_START),
    CSNAME(LINK_REG21_MAX),

    CSNAME(LINK_MCU_WAKEUP),
    CSNAME(LINK_MCU_STANDBY),
    CSNAME(LINK_EN_DBW),
    CSNAME(LINK_UN_DBW),
    CSNAME(LINK_WIRE_CTL_DISABLE),
    CSNAME(LINK_WIRE_CTL_ENABLE),
    CSNAME(LINK_REG22_MAX),


    CSNAME(LINK_REPORT_MCU_WORK),
    CSNAME(LINK_REPORT_DC_ERROR),
    CSNAME(LINK_REPORT_MUTE_ON),
    CSNAME(LINK_REPORT_MCU_BUSY),
    CSNAME(LINK_REPORT_DAC_PWR),
    CSNAME(LINK_REPORT_ANALOG_PWR),
    CSNAME(LINK_REPORT_PLAYING),

    CSNAME(LINK_ANALOG_POWER),
    CSNAME(LINK_DAC_POWER_ON),
    CSNAME(LINK_FIRST_PLAY),
};

#define REG_ID          0x00
#define REG_EVENT       0x01
#define REG_STATUS      0x02
#define REG_INSERT      0x03
#define REG_OUT_PORT    0x04
#define REG_AM_MODULE   0x05
#define REG_KEY_EVENT   0x06

#define REG02_STATUS_MASK 0x80
#define REG03_INSERT_MASK 0x40
#define REG04_OUT_STATUS_MASK 0x20
#define REG05_AMP_MODULE_MASK 0x10
#define REG06_KEY_MASK 0x08
#define REG07_APP_MASK 0x04

//REG02
#define REG02_MCU_WORKING       0x80
#define REG02_SINGLE_DC_ERROR   0x40
#define REG02_BAH_DC_ERROR      0x20
#define REG02_MUTE_PO_EN        0x10
#define REG02_MCU_BUSY          0x08
#define REG02_MUTE_LO_EN        0x04

//REG03
#define REG03_PO_DETECT         0x80
#define REG03_LINE_DETECT       0x40

//REG04
#define REG04_PHONE_OUT         0x80
#define REG04_LINE_OUT          0x40
#define REG04_BAH_OUT           0x20
#define REG04_OPT_OUT           0x10
#define REG04_COAX_OUT          0x08
#define REG04_DOCK_OUT          0x04
#define REG04_ANALOG_POWER      0x02
#define REG04_DAC_POWER         0x01


//REG05
#define REG05_AM_MODULE         0x0F

//REG06
#define REG06_KEY_EVENT         0xF0
#define LINE_STATUS_RELEASE     0x00
#define LINE_VOL_UP             0x10
#define LINE_VOL_UP_PRESS       0x20
#define LINE_VOL_DOWN           0x30
#define LINE_VOL_DOWN_PRESS     0x40
#define LINE_PLAY_PAUSE         0x50
#define LINE_PLAY_NEXT          0x60
#define LINE_PLAY_PREV          0x70


#if 0
#define LINK_PO_DET    0x02
#define LINK_LO_DET    0x01
#define LINK_BAH_DET   0x80
#define LINK_DC_DET    0x40

#define LINK_PO_MASK   0x02
#define LINK_LO_MASK   0x01
#define LINK_BAH_MASK  0x80
#define LINK_DC_MASK   0x40

#define LINK_LO_MUTE_MASK 0x0C
#define LINK_LO_MUTE_ON   0x08
#define LINK_LO_MUTE_OFF  0x04

#define LINE_VOL_UP    0x40
#define LINE_VOL_DOWN  0x20
#define LINE_VOL_PAUSE 0x10
#define LINE_VOL_NEXT  0x08
#define LINE_VOL_PREV  0x04

#define LINE_VOL_UP_PRESS      0x01
#define LINE_VOL_DOWN_PRESS    0x80
#define LINE_VOL_RELEASE       0x10

#endif


extern struct gpio_switch_data*  gswitch;
extern int link_status_upate(struct gpio_switch_data* switch_data);
extern int link_set_status(enum link_status status);
extern bool link_get_status(enum link_status status);
extern int link_start(void);
extern int link_stop(int tosleep,int fpga_off);
extern u8 mcu_get_fw_ver(void);
extern void link_set_force_poweron(void);
extern void link_set_force_upgrade(void);
#endif
