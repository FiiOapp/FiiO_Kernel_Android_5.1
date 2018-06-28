#ifndef __LINUX_AXP_CFG_H_
#define __LINUX_AXP_CFG_H_
#include "axp-mfd.h"

/*设备地址*/
/*
	一般不改变：
	AXP22:0x34
*/
#define	AXP_DEVICES_ADDR	(0x68 >> 1)
/*i2c控制器的设备号:具体看所使用平台硬件的连接*/
#define	AXP_I2CBUS			1//2
/*电源芯片对应的中断号：具体看所使用的平台硬件的连接，
中断线nmi连接cpu的哪路irq或者gpio*/
#define AXP_IRQNO			64

/*初始化各路输出，单位mV，0都为关闭*/
/*
	ldo1：
		由硬件决定输出电压，软件改不了，只是软件的显示电压：
*/
#define AXP_LDO1_VALUE			3000
/*
	ldo2：
		AXP22:700~3300,100/step
*/
#define AXP_LDO2_VALUE		3000
/*
	ldo3：
		AXP22:700~3300,100/step
*/
#define AXP_LDO3_VALUE		1000
/*
	ldo4：
		AXP22:700~3300,100/step
*/
#define AXP_LDO4_VALUE		1800
/*
	ldo5：
		AXP22:700~3300,100/step
*/
#define AXP_LDO5_VALUE		3000
/*
	ldo6：
		AXP22:700~3300,100/step
*/
#define AXP_LDO6_VALUE		3300//700 //OSC-3V3
/*
	ldo7：
		AXP22:700~3300,100/step
*/
#define AXP_LDO7_VALUE		2800//VCC_TP 700
/*
	ldo8：
		AXP22:700~3300,100/step
*/
#define AXP_LDO8_VALUE		3000//700
/*
	ldo9：
		AXP22:700~3300,100/step
*/
#define AXP_LDO9_VALUE		3000//700
/*
	ldo10：
		AXP22:700~3300,100/step
*/
#define AXP_LDO10_VALUE		3000//700 VCC_WL
/*
	ldo11：
		AXP22:700~3300,100/step
*/
#define AXP_LDO11_VALUE		3300//700
/*
	ldo12：
		AXP22:700~1400,100/step
*/
#define AXP_LDO12_VALUE		1200//1100

/*
	DCDC1:
		AXP22:1600~3400,100/setp
*/
#define AXP_DCDC1_VALUE		3000
/*
	DCDC2：
		AXP22:600~1540，20/step
*/
#define AXP_DCDC2_VALUE		1260//1200
/*
	DCDC3：
		AXP22:600~1860，20/step
*/
#define AXP_DCDC3_VALUE		1260
/*
	DCDC4：
		AXP22:600~1540，20/step
*/
#define AXP_DCDC4_VALUE		1200
/*
	DCDC5：
		AXP22:1000~2550，50/step
*/
#define AXP_DCDC5_VALUE		1500

#define AXP_LDOIO0_VALUE        2500//VCC_GS

#define AXP_LDOIO1_VALUE        1500//OSC-1V5

/*电池容量，mAh：根据实际电池容量来定义，对库仑计方法来说
这个参数很重要，必须配置*/
#define BATCAP				3773//4000

/*初始化电池内阻，mΩ：一般在100~200之间，不过最好根据实际
测试出来的确定，方法是打开打印信息，不接电池烧好固件后，
上电池，不接充电器，开机，开机1分钟后，接上充电器，充
1~2分钟，看打印信息中的rdc值，填入这里*/
#define BATRDC				89
/*开路电压方法中的电池电压的缓存*/
#define AXP_VOL_MAX			1
/*
	充电功能使能：
        AXP22:0-关闭，1-打开
*/
#define CHGEN       1

/*
	充电电流设置，uA，0为关闭：
		AXP22:300~2550,100/step
*/
/*开机充电电流，uA*/
#define STACHGCUR			1300*1000//900*1000
/*关屏充电电流，uA*/
#define EARCHGCUR			1300*1000//900*1000
/*休眠充电电流，uA*/
#define SUSCHGCUR			1300*1000//1500*1000
/*关机充电电流，uA*/
#define CLSCHGCUR			1300*1000//1500*1000

/*目标充电电压，mV*/
/*
	AXP22:4100/4220/4200/4240
*/
#define CHGVOL				4200*1000
/*充电电流小于设置电流的ENDCHGRATE%时，停止充电，%*/
/*
	AXP22:10\15
*/
#define ENDCHGRATE			10
/*关机电压，mV*/
/*
	系统设计的关机过后的电池端电压，需要与关机百分比、
	开路电压对应百分比表及低电警告电压相互配合才会有作用
*/
#define SHUTDOWNVOL			3300

/*adc采样率设置，Hz*/
/*
	AXP22:100\200\400\800
*/
#define ADCFREQ				100
/*预充电超时时间，min*/
/*
	AXP22:40\50\60\70
*/
#define CHGPRETIME			50
/*恒流充电超时时间，min*/
/*
	AXP22:360\480\600\720
*/
#define CHGCSTTIME			480


/*pek开机时间，ms*/
/*
	按power键硬件开机时间：
		AXP22:128/1000/2000/3000
*/
#define PEKOPEN				1000
/*pek长按时间，ms*/
/*
	按power键发长按中断的时间，短于此时间是短按，发短按键irq，
	长于此时间是长按，发长按键irq：
		AXP22:1000/1500/2000/2500
*/
#define PEKLONG				1500
/*pek长按关机使能*/
/*
	按power键超过关机时长硬件关机功能使能：
		AXP22:0-不关，1-关机
*/
#define PEKOFFEN			1
/*pek长按关机使能后开机选择*/
/*
	按power键超过关机时长硬件关机还是重启选择:
		AXP22:0-只关机不重启，1-关机后重启
*/
#define PEKOFFRESTART			0
/*pekpwr延迟时间，ms*/
/*
	开机后powerok信号的延迟时间：
		AXP20:8/16/32/64
*/
#define PEKDELAY			32
/*pek长按关机时间，ms*/
/*
	按power键的关机时长：
		AXP22:4000/6000/8000/10000
*/
#define PEKOFF				6000
/*过温关机使能*/
/*
	AXP内部温度过高硬件关机功能使能：
		AXP22:0-不关，1-关机
*/
#define OTPOFFEN			0
/* 充电电压限制使能*/
/*
	AXP22:0-关闭，1-打开
*/
#define USBVOLLIMEN		1
/*  充电限压，mV，0为不限制*/
/*
	AXP22:4000~4700，100/step
*/
#define USBVOLLIM			4500
/*  USB充电限压，mV，0为不限制*/
/*
	AXP22:4000~4700，100/step
*/
#define USBVOLLIMPC			4600

/* 充电电流限制使能*/
/*
	AXP22:0-关闭，1-打开
*/
#define USBCURLIMEN		0 // only control AC current
/* 充电限流，mA，0为不限制*/
/*
	AXP22:500/900
*/
#define USBCURLIM			1600
/* usb 充电限流，mA，0为不限制*/
/*
	AXP22:500/900
*/
#define USBCURLIMPC			500
/* PMU 中断触发唤醒使能*/
/*
	AXP22:0-不唤醒，1-唤醒
*/
#define IRQWAKEUP			0
/* N_VBUSEN PIN 功能控制*/
/*
	AXP22:0-输出，驱动OTG升压模块，1-输入，控制VBUS通路
*/
#define VBUSEN			1
/* ACIN/VBUS In-short 功能设置*/
/*
	AXP22:0-AC VBUS分开，1-使用VBUS当AC,无单独AC
*/
#define VBUSACINSHORT			0
/* CHGLED 管脚控制设置*/
/*
	AXP22:0-驱动马达，1-由充电功能控制
*/
#define CHGLEDFUN			0
/* CHGLED LED 类型设置*/
/*
	AXP22:0-充电时led长亮，1-充电时led闪烁
*/
#define CHGLEDTYPE			0
/* 电池总容量校正使能*/
/*
	AXP22:0-关闭，1-打开
*/
#define BATCAPCORRENT			1
/* 充电完成后，充电输出使能*/
/*
	AXP22:0-关闭，1-打开
*/
#define BATREGUEN			0
/* 电池检测功能使能*/
/*
	AXP22:0-关闭，1-打开
*/
#define BATDET		1
/* PMU重置使能*/
/*
	AXP22:0-关闭，1-打开按电源键16秒重置PMU功能
*/
#define PMURESET		0
/*低电警告电压1，%*/
/*
	根据系统设计来定：
	AXP22:5%~20%
*/
#define BATLOWLV1    15
/*低电警告电压2，%*/
/*
	根据系统设计来定：
	AXP22:0%~15%
*/
#define BATLOWLV2    0

#define ABS(x)				((x) >0 ? (x) : -(x) )

#ifdef	CONFIG_KP_AXP22
/*AXP GPIO start NUM,根据平台实际情况定义*/
#define AXP22_NR_BASE 100

/*AXP GPIO NUM,包括马达驱动、LCD power以及VBUS driver pin*/
#define AXP22_NR 5

/*初始化开路电压对应百分比表*/
/*
	可以使用默认值，但是最好根据实际测试的电池来确定每级
	对应的剩余百分比，特别需要注意，关机电压SHUTDOWNVOL和电池
	容量开始校准剩余容量百分比BATCAPCORRATE这两级的准确性
	AXP22适用
*/
#define OCVREG0				0	            //3.13V
#define OCVREG1				0	            //3.27V
#define OCVREG2				0	            //3.34V
#define OCVREG3				1	            //3.41V
#define OCVREG4				1	            //3.48V
#define OCVREG5				2	            //3.52V
#define OCVREG6				2	            //3.55V
#define OCVREG7				3	            //3.57V
#define OCVREG8				3	            //3.59V
#define OCVREG9				4	            //3.61V
#define OCVREGA				5	            //3.63V
#define OCVREGB				7	            //3.64V
#define OCVREGC				11	            //3.66V
#define OCVREGD				16	            //3.7V
#define OCVREGE				28	            //3.73V
#define OCVREGF				42	            //3.77V
#define OCVREG10		 	47              //3.78V
#define OCVREG11		 	52              //3.8V
#define OCVREG12		 	55              //3.82V
#define OCVREG13		 	59              //3.84V
#define OCVREG14		 	62              //3.85V
#define OCVREG15		 	64              //3.87V
#define OCVREG16		 	70              //3.91V
#define OCVREG17		 	74              //3.94V
#define OCVREG18		 	79              //3.98V
#define OCVREG19		 	82              //4.01V
#define OCVREG1A		 	87              //4.05V
#define OCVREG1B		 	90              //4.08V
#define OCVREG1C		 	93              //4.1V
#define OCVREG1D		 	96              //4.12V
#define OCVREG1E		 	98              //4.14V
#define OCVREG1F		 	100             //4.15V

/*  AXP中断*/
#define AXP_IRQ_USBLO		AXP22_IRQ_USBLO	//usb 低电
#define AXP_IRQ_USBRE		AXP22_IRQ_USBRE	//usb 拔出
#define AXP_IRQ_USBIN		AXP22_IRQ_USBIN	//usb 插入
#define AXP_IRQ_USBOV		AXP22_IRQ_USBOV	//usb 过压
#define AXP_IRQ_ACRE			AXP22_IRQ_ACRE	//ac  拔出
#define AXP_IRQ_ACIN			AXP22_IRQ_ACIN	//ac  插入
#define AXP_IRQ_ACOV		AXP22_IRQ_ACOV //ac  过压
#define AXP_IRQ_TEMLO		AXP22_IRQ_TEMLO //电池低温
#define AXP_IRQ_TEMOV		AXP22_IRQ_TEMOV //电池过温
#define AXP_IRQ_CHAOV		AXP22_IRQ_CHAOV //电池充电结束
#define AXP_IRQ_CHAST		AXP22_IRQ_CHAST //电池充电开始
#define AXP_IRQ_BATATOU	AXP22_IRQ_BATATOU //电池退出激活模式
#define AXP_IRQ_BATATIN		AXP22_IRQ_BATATIN //电池进入激活模式
#define AXP_IRQ_BATRE		AXP22_IRQ_BATRE //电池拔出
#define AXP_IRQ_BATIN		AXP22_IRQ_BATIN //电池插入
#define AXP_IRQ_PEKLO		AXP22_IRQ_POKLO //power键长按
#define AXP_IRQ_PEKSH		AXP22_IRQ_POKSH //power键短按

#define AXP_IRQ_CHACURLO	AXP22_IRQ_CHACURLO //充电电流小于设置值
#define AXP_IRQ_ICTEMOV		AXP22_IRQ_ICTEMOV //AXP芯片内部过温
#define AXP_IRQ_EXTLOWARN2	AXP22_IRQ_EXTLOWARN2 //APS低压警告电压2
#define AXP_IRQ_EXTLOWARN1	AXP22_IRQ_EXTLOWARN1 //APS低压警告电压1

#define AXP_IRQ_GPIO0TG		AXP22_IRQ_GPIO0TG //GPIO0输入边沿触发
#define AXP_IRQ_GPIO1TG		AXP22_IRQ_GPIO1TG //GPIO1输入边沿触发

#define AXP_IRQ_PEKFE		AXP22_IRQ_PEKFE //power键下降沿触发
#define AXP_IRQ_PEKRE		AXP22_IRQ_PEKRE //power键上升沿触发
#define AXP_IRQ_TIMER		AXP22_IRQ_TIMER //计时器超时

#endif

/*选择需要打开的中断使能*/
static const uint64_t AXP22_NOTIFIER_ON = (AXP_IRQ_USBIN |AXP_IRQ_USBRE |
				       		                            AXP_IRQ_ACIN |AXP_IRQ_ACRE |
				       		                            AXP_IRQ_BATIN |AXP_IRQ_BATRE |
				       		                            AXP_IRQ_CHAST |AXP_IRQ_CHAOV);

/* 需要做插入火牛、usb关机重启进boot时power_start设置为1，否则为0*/
#define POWER_START 1


#endif
