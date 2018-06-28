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
        struct switch_dev dc_detect;
        struct switch_dev line_control;
        struct switch_dev accessory_dev;
        struct switch_dev balance_dev;

	struct work_struct sdcard_work;
	struct work_struct key_work;

	struct work_struct mcu_update_work;
        //add for adc
        struct timer_list adc0_timer;
        struct adc_client *adc0_client;
        struct timer_list adc2_timer;
        struct adc_client *adc2_client;
        int dock_count;
        int adc0_value;
        int dock_out_count;

        //add for audio link status
        u8 mcu_working;
        u8 analog_power;
        u8 first_play;
        u8 dc_error;
        u8 mute;
        u8 dac_en;
        u8 mcu_firmware_version;
        u8 dock_en;
        u8 mcu_force_upgrade;
};

enum link_status{

    LINK_MCU_WORK,
    LINK_FIRST_PLAY,
    LINK_ANALOG_POWER,
    LINK_STOP_PLAY,
    LINK_MCU_STOP,
    LINK_DAC_POWER_ON,
    LINK_DAC_POWER_OFF,
    LINK_MUTE_ON,
    LINK_MUTE_OFF,
    LINK_DOCK_ENABLE,
    LINK_DOCK_DISABLE,
    LINK_WIRE_CTL_ENABLE,
    LINK_WIRE_CTL_DISABLE,
    LINK_AUTO_POWER_ON,
    LINK_AUTO_POWER_OFF,
    LINK_FORCE_POWER_ON,
    LINK_FORCE_MCU_UPGRADE,
};

extern struct gpio_switch_data*  gswitch;
extern int link_status_upate(struct gpio_switch_data* switch_data);
extern int link_set_status(enum link_status status);
extern bool link_get_status(enum link_status status);
extern int link_start();
extern int link_stop(int tosleep);
extern u8 mcu_get_fw_ver();
extern void link_set_force_poweron();
extern void link_set_force_upgrade();

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
