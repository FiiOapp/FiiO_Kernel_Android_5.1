#include <linux/regulator/machine.h>
#include <linux/regulator/act8846.h>
#include <mach/sram.h>
#include <linux/platform_device.h>

#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>

#ifdef CONFIG_REGULATOR_ACT8846

extern int act8846_set_bits(struct act8846 *act8846, u8 reg, u16 mask, u16 val);
extern struct act8846 *g_act8846;

void jackline_power(int on)
{
#if 0
    ldo9=regulator_get(NULL,"act_ldo9");
    if(ldo9 == NULL || IS_ERR(ldo9)){

        printk("act ldo9 is not available \n");
    }else{

        if(on)
            regulator_enable(ldo9);
        else
            regulator_disable(ldo9);

        regulator_put(ldo9);

    }
#else
	printk("%s,line=%d %d\n", __func__,__LINE__,on);
    if(on)
	    act8846_set_bits(g_act8846 , 0xb1,0x80, 0x80);
    else
	    act8846_set_bits(g_act8846 , 0xb1,0x80, 0);

#endif
}

static int act8846_set_init(struct act8846 *act8846)
{
	struct regulator *dcdc;
	struct regulator *ldo;
	int i = 0;
	printk("%s,line=%d\n", __func__,__LINE__);

	#ifndef CONFIG_RK_CONFIG
	g_pmic_type = PMIC_TYPE_ACT8846;
	#endif
	printk("%s:g_pmic_type=%d\n",__func__,g_pmic_type);
	
	for(i = 0; i < ARRAY_SIZE(act8846_dcdc_info); i++)
	{

                if(act8846_dcdc_info[i].min_uv == 0 && act8846_dcdc_info[i].max_uv == 0)
                        continue;
	        dcdc =regulator_get(NULL, act8846_dcdc_info[i].name);
	        regulator_set_voltage(dcdc, act8846_dcdc_info[i].min_uv, act8846_dcdc_info[i].max_uv);
		 regulator_set_suspend_voltage(dcdc, act8846_dcdc_info[i].suspend_vol);
	        regulator_enable(dcdc);
	        printk("%s  %s =%dmV end\n", __func__,act8846_dcdc_info[i].name, regulator_get_voltage(dcdc));
	        regulator_put(dcdc);
	        udelay(100);
	}
	
	for(i = 0; i < ARRAY_SIZE(act8846_ldo_info); i++)
	{
                if(act8846_ldo_info[i].min_uv == 0 && act8846_ldo_info[i].max_uv == 0){
                    ldo =regulator_get(NULL, act8846_ldo_info[i].name);
                    /* regulator_enable(ldo); */
                    /* regulator_disable(ldo); */
					/* regulator_put(ldo); */
	                printk("%s  %s  is default disabled!!\n", __func__,act8846_ldo_info[i].name);
                    continue;
                }
	        ldo =regulator_get(NULL, act8846_ldo_info[i].name);
	        regulator_set_voltage(ldo, act8846_ldo_info[i].min_uv, act8846_ldo_info[i].max_uv);
	        regulator_enable(ldo);
	        printk("%s  %s =%dmV end\n", __func__,act8846_ldo_info[i].name, regulator_get_voltage(ldo));
	        regulator_put(ldo);
	}


#if defined(CONFIG_LIDA_MACH_X5) || defined(CONFIG_LIDA_MACH_X7II)
    // disable ldo9(jack line power) output
    // call regulator_disable won't disable power,because this power 
    // enabled default by hardware not enalbed by regulator_enable.


    //set out13 vol
	act8846_set_bits(g_act8846 , 0xb1,0x10, 0x10);
	act8846_set_bits(g_act8846 , 0xb0,0xFF, 0x28);
	act8846_set_bits(g_act8846 , 0xb1,0x10, 0x00);

    jackline_power(0);
#endif

	#ifdef CONFIG_RK_CONFIG
	if(sram_gpio_init(get_port_config(pmic_slp).gpio, &pmic_sleep) < 0){
		printk(KERN_ERR "sram_gpio_init failed\n");
		return -EINVAL;
	}
	if(port_output_init(pmic_slp, 0, "pmic_slp") < 0){
		printk(KERN_ERR "port_output_init failed\n");
		return -EINVAL;
	}
	#else
    /*------------------------------------------------------------------*/
	/* if(sram_gpio_init(PMU_POWER_SLEEP, &pmic_sleep) < 0){ */
		/* printk(KERN_ERR "sram_gpio_init failed\n"); */
		/* return -EINVAL; */
	/* } */
	/* gpio_request(PMU_POWER_SLEEP, "NULL"); */
	/* gpio_direction_output(PMU_POWER_SLEEP, GPIO_LOW); */
    /*------------------------------------------------------------------*/

	// 3188 don't need to  change pmu clock from 24M to 32k 
    // add by yunxiZhang
    // 2016,05,17 16:57:10 CST
#if defined(CONFIG_SOC_RK3066) && defined(CONFIG_CLK_SWITCH_TO_32K)
    if(sram_gpio_init(PMU_POWER_SLEEP, &pmic_sleep) < 0){
        printk(KERN_ERR "sram_gpio_init failed\n");
        return -EINVAL;
    }
    gpio_request(PMU_POWER_SLEEP, "pmu power sleep");
    gpio_direction_output(PMU_POWER_SLEEP, GPIO_LOW);
#endif
    /*------------------------------------------------------------------*/
	#ifdef CONFIG_ACT8846_SUPPORT_RESET
	if(sram_gpio_init(PMU_VSEL, &pmic_vsel) < 0){
		printk(KERN_ERR "sram_gpio_init failed\n");
		return -EINVAL;
	}
//	rk30_mux_api_set(GPIO3D3_PWM0_NAME,GPIO3D_GPIO3D3);
	gpio_request(PMU_VSEL, "pmu vsel");
	gpio_direction_output(PMU_VSEL, GPIO_HIGH);
	#endif
	
	#endif


    /* iomux_set(GPIO0_C4); */
    /* gpio_request(PMU_POWEREN,"pmu pwern"); */
    /* gpio_direction_output(PMU_POWEREN,GPIO_HIGH); */

	printk("%s,line=%d END\n", __func__,__LINE__);
	
	
	return 0;
}

static struct regulator_consumer_supply act8846_buck1_supply[] = {
	{
		.supply = "act_dcdc1",
	},

};
static struct regulator_consumer_supply act8846_buck2_supply[] = {
	{
		.supply = "act_dcdc2",
	},
	{
		.supply = "vdd_core",
	},
	
};
static struct regulator_consumer_supply act8846_buck3_supply[] = {
	{
		.supply = "act_dcdc3",
	},
	{
		.supply = "vdd_cpu",
	},
};

static struct regulator_consumer_supply act8846_buck4_supply[] = {
	{
		.supply = "act_dcdc4",
	},

};

static struct regulator_consumer_supply act8846_ldo1_supply[] = {
	{
		.supply = "act_ldo1",
	},
};
static struct regulator_consumer_supply act8846_ldo2_supply[] = {
	{
		.supply = "act_ldo2",
	},
};

static struct regulator_consumer_supply act8846_ldo3_supply[] = {
	{
		.supply = "act_ldo3",
	},
};
static struct regulator_consumer_supply act8846_ldo4_supply[] = {
	{
		.supply = "act_ldo4",
	},
};
static struct regulator_consumer_supply act8846_ldo5_supply[] = {
	{
		.supply = "act_ldo5",
	},
};
static struct regulator_consumer_supply act8846_ldo6_supply[] = {
	{
		.supply = "act_ldo6",
	},
};

static struct regulator_consumer_supply act8846_ldo7_supply[] = {
	{
		.supply = "act_ldo7",
	},
};
static struct regulator_consumer_supply act8846_ldo8_supply[] = {
	{
		.supply = "act_ldo8",
	},
};
static struct regulator_consumer_supply act8846_ldo9_supply[] = {
	{
		.supply = "act_ldo9",
	},
};


static struct regulator_init_data act8846_buck1 = {
	.constraints = {
		.name           = "ACT_DCDC1",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		.always_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_buck1_supply),
	.consumer_supplies =  act8846_buck1_supply,
};

/* */
static struct regulator_init_data act8846_buck2 = {
	.constraints = {
		.name           = "ACT_DCDC2",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		.always_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_buck2_supply),
	.consumer_supplies =  act8846_buck2_supply,
};

/* */
static struct regulator_init_data act8846_buck3 = {
	.constraints = {
		.name           = "ACT_DCDC3",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		.always_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_buck3_supply),
	.consumer_supplies =  act8846_buck3_supply,
};

static struct regulator_init_data act8846_buck4 = {
	.constraints = {
		.name           = "ACT_DCDC4",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		.always_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_buck4_supply),
	.consumer_supplies =  act8846_buck4_supply,
};

static struct regulator_init_data act8846_ldo1 = {
	.constraints = {
		.name           = "ACT_LDO1",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_ldo1_supply),
	.consumer_supplies =  act8846_ldo1_supply,
};

/* */
static struct regulator_init_data act8846_ldo2 = {
	.constraints = {
		.name           = "ACT_LDO2",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_ldo2_supply),
	.consumer_supplies =  act8846_ldo2_supply,
};

/* */
static struct regulator_init_data act8846_ldo3 = {
	.constraints = {
		.name           = "ACT_LDO3",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_ldo3_supply),
	.consumer_supplies =  act8846_ldo3_supply,
};

/* */
static struct regulator_init_data act8846_ldo4 = {
	.constraints = {
		.name           = "ACT_LDO4",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_ldo4_supply),
	.consumer_supplies =  act8846_ldo4_supply,
};

static struct regulator_init_data act8846_ldo5 = {
	.constraints = {
		.name           = "ACT_LDO5",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_ldo5_supply),
	.consumer_supplies =  act8846_ldo5_supply,
};

static struct regulator_init_data act8846_ldo6 = {
	.constraints = {
		.name           = "ACT_LDO6",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_ldo6_supply),
	.consumer_supplies =  act8846_ldo6_supply,
};

static struct regulator_init_data act8846_ldo7 = {
	.constraints = {
		.name           = "ACT_LDO7",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_ldo7_supply),
	.consumer_supplies =  act8846_ldo7_supply,
};

static struct regulator_init_data act8846_ldo8 = {
	.constraints = {
		.name           = "ACT_LDO8",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_ldo8_supply),
	.consumer_supplies =  act8846_ldo8_supply,
};

static struct regulator_init_data act8846_ldo9 = {
	.constraints = {
		.name           = "ACT_LDO9",
		.min_uV			= 600000,
		.max_uV			= 3900000,
		.apply_uV		= 1,
		
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(act8846_ldo9_supply),
	.consumer_supplies =  act8846_ldo9_supply,
};

struct act8846_regulator_subdev act8846_regulator_subdev_id[] = {
	{
		.id=0,
		.initdata=&act8846_buck1,		
	 },

	{
		.id=1,
		.initdata=&act8846_buck2,		
	 },
	{
		.id=2,
		.initdata=&act8846_buck3,		
	 },
        {
		.id=3,
		.initdata=&act8846_buck4,		
	 },

	{
		.id=4,
		.initdata=&act8846_ldo1,		
	 },

	{
		.id=5,
		.initdata=&act8846_ldo2,		
	 },

	{
		.id=6,
		.initdata=&act8846_ldo3,		
	 },

	{
		.id=7,
		.initdata=&act8846_ldo4,		
	 },

	{
		.id=8,
		.initdata=&act8846_ldo5,		
	 },

	{
		.id=9,
		.initdata=&act8846_ldo6,		
	 },

	{
		.id=10,
		.initdata=&act8846_ldo7,		
	 },

	{
		.id=11,
		.initdata=&act8846_ldo8,		
	 },
#if 1
	{
		.id=12,
		.initdata=&act8846_ldo9,		
	 },
#endif
};

static struct act8846_platform_data act8846_data={
	.set_init=act8846_set_init,
	.num_regulators=13,
	.regulators=act8846_regulator_subdev_id,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
void act8846_early_suspend(struct early_suspend *h)
{
}

void act8846_late_resume(struct early_suspend *h)
{
}
#endif

#ifdef CONFIG_PM
int __sramdata vdd_cpu_vol ,vdd_core_vol;
void act8846_device_suspend(void)
{		
	struct regulator *dcdc;
	#ifdef CONFIG_ACT8846_SUPPORT_RESET_
	/* sram_gpio_set_value(pmic_vsel, GPIO_HIGH);   */
	
	dcdc =dvfs_get_regulator( "vdd_cpu");
	vdd_cpu_vol = regulator_get_voltage(dcdc);
	regulator_set_voltage(dcdc, 900000, 900000);
	udelay(100);

	dcdc =dvfs_get_regulator( "vdd_core");
	vdd_core_vol = regulator_get_voltage(dcdc);
	regulator_set_voltage(dcdc, 900000, 900000);
	udelay(100);

	dcdc =regulator_get(NULL, "act_dcdc4");
	regulator_set_voltage(dcdc, 2800000, 2800000);
	regulator_put(dcdc);
	udelay(100);

	#endif
}

void act8846_device_resume(void)
{
	struct regulator *dcdc;
	#ifdef CONFIG_ACT8846_SUPPORT_RESET_

	dcdc =dvfs_get_regulator( "vdd_cpu");
	regulator_set_voltage(dcdc, vdd_cpu_vol, vdd_cpu_vol);
	udelay(100);

	dcdc =dvfs_get_regulator( "vdd_core");
	regulator_set_voltage(dcdc, vdd_core_vol, vdd_core_vol);
	udelay(100);

	dcdc =regulator_get(NULL, "act_dcdc4");
	regulator_set_voltage(dcdc, 3000000, 3000000);
	regulator_put(dcdc);
	udelay(100);
	
	/* sram_gpio_set_value(pmic_vsel, GPIO_LOW);   */
	
	#endif
	
}
#else
void act8846_device_suspend(void)
{		
}
void act8846_device_resume(void)
{
}
#endif

void __sramfunc board_pmu_act8846_suspend(void)
{	
	#ifdef CONFIG_CLK_SWITCH_TO_32K
	 sram_gpio_set_value(pmic_sleep, GPIO_HIGH);  
	#endif
}
void __sramfunc board_pmu_act8846_resume(void)
{
	#ifdef CONFIG_CLK_SWITCH_TO_32K
 	sram_gpio_set_value(pmic_sleep, GPIO_LOW);  
	sram_32k_udelay(10000);
	#endif
}
void __sramfunc board_act8846_set_suspend_vol(void)
{	
#ifdef CONFIG_ACT8846_SUPPORT_RESET
	/* sram_gpio_set_value(pmic_vsel, GPIO_HIGH);  */
#endif
}
void __sramfunc board_act8846_set_resume_vol(void)
{
#ifdef CONFIG_ACT8846_SUPPORT_RESET
	/* sram_gpio_set_value(pmic_vsel, GPIO_LOW);   */
	sram_32k_udelay(1000);
#endif
}


#endif




