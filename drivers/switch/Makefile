# Switch Class Driver
obj-$(CONFIG_SWITCH)		+= switch_class.o
obj-$(CONFIG_EXTERNAL_SD)	+= switch_external_sd.o
ifeq ($(CONFIG_LIDA_MACH_X7),y)
obj-$(CONFIG_SWITCH_GPIO)	+= switch_gpio_adc.o
else ifeq ($(CONFIG_LIDA_MACH_X5),y)
obj-$(CONFIG_SWITCH_GPIO)	+= switch_gpio_msp430.o
else ifeq ($(CONFIG_LIDA_MACH_X7II),y)
obj-$(CONFIG_SWITCH_GPIO)	+= switch_gpio_msp430_NP.o
else
obj-$(CONFIG_SWITCH_GPIO)	+= switch_gpio.o
endif

