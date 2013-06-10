/*
 * cypress_touchkey.h - Platform data for cypress touchkey driver
 *
 * Copyright (C) 2011 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __LINUX_CYPRESS_TOUCHKEY_H
#define __LINUX_CYPRESS_TOUCHKEY_H
extern struct class *sec_class;
extern int ISSP_main(void);

/* DVFS feature : TOUCH BOOSTER */
#define TSP_BOOSTER
#ifdef TSP_BOOSTER
#include <linux/cpufreq.h>

#define TOUCH_BOOSTER_OFF_TIME	300
#define TOUCH_BOOSTER_CHG_TIME	200
#endif

#if defined(CONFIG_GLOVE_TOUCH)
#define TK_BIT_GLOVE 0x40
#endif

/*#define TK_HOME_ENABLE*/

/* Flip cover*/
#define TKEY_FLIP_MODE

#ifdef TKEY_FLIP_MODE
#define TK_BIT_FLIP		0x08
#endif

#define TK_INFORM_CHARGER
#define TK_BIT_TA_ON		0x10

#define CYPRESS_55_IC_MASK	0x20
#define CYPRESS_65_IC_MASK	0x04
/*
u32 ic_fw_id;
*/
struct touchkey_callbacks {
	void (*inform_charger)(struct touchkey_callbacks *, bool);
};

struct cypress_touchkey_platform_data {
	unsigned	gpio_int;
	unsigned	gpio_led_en;
	const u8	*touchkey_keycode;
	void	(*power_onoff) (int);
	bool	skip_fw_update;
	bool	touchkey_order;
	void	(*register_cb)(void *);

};

void touchkey_charger_infom(bool en);


#define PM8921_IRQ_BASE			(NR_MSM_IRQS + NR_GPIO_IRQS)
#define IRQ_TOUCHKEY_INT PM8921_GPIO_IRQ(PMIC8058_IRQ_BASE, (PM8058_GPIO(31)))

#if defined(CONFIG_MACH_JACTIVE_EUR)
#define GPIO_TOUCHKEY_SDA	6
#define GPIO_TOUCHKEY_SCL	34
#define GPIO_TOUCHKEY_SCL_2	26
#define PMIC_GPIO_TKEY_INT	32
#define PMIC_GPIO_TKEY_EN	32
#else
#define GPIO_TOUCHKEY_SDA	33
#define GPIO_TOUCHKEY_SCL	34
#define GPIO_TOUCHKEY_SCL_2	26
#define PMIC_GPIO_TKEY_INT	31
#define PMIC_GPIO_TKEY_EN	32
#endif


#ifdef CONFIG_KEYBOARD_CYPRESS_TOUCH_236
#if defined(CONFIG_MACH_EXPRESS)
#define CYPRESS_DIFF_MENU      0x0A
#define CYPRESS_DIFF_BACK      0x0C
#define CYPRESS_DIFF_HOME      0
#define CYPRESS_DIFF_RECENT    0
#define CYPRESS_RAW_DATA_MENU  0x0E
#define CYPRESS_RAW_DATA_BACK  0x10
#define CYPRESS_RAW_DATA_HOME  0
#define CYPRESS_RAW_DATA_RECENT    0
#elif defined(CONFIG_MACH_AEGIS2)
#define CYPRESS_DIFF_BACK      0x10
#define CYPRESS_DIFF_HOME      0x0E
#define CYPRESS_DIFF_RECENT    0x0C
#define CYPRESS_DIFF_MENU      0x0A

#define CYPRESS_RAW_DATA_BACK  0x18
#define CYPRESS_RAW_DATA_HOME  0x16
#define CYPRESS_RAW_DATA_RECENT 0x14
#define CYPRESS_RAW_DATA_MENU  0x12


#endif
#endif

#endif /* __LINUX_CYPRESS_TOUCHKEY_H */
