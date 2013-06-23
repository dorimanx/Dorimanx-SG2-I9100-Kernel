/*
* Copyright (C) 2011 Google, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/keyreset.h>
#include <linux/gpio_event.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <asm/mach-types.h>
#include <linux/regulator/consumer.h>
#include "board-8064.h"
#include <linux/delay.h>
#include <mach/apq8064-gpio.h>

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI
#include <linux/i2c/synaptics_rmi.h>
#endif


#define NEW_GPIO_TOUCH_IRQ	67
#define GPIO_TOUCH_IRQ		57

/* touch is on i2c3 */
#define GPIO_TOUCH_SDA		8
#define GPIO_TOUCH_SCL		9

/*
 * sec_tsp_reboot_mode(char *mode) used
 * only recovery mode && factory binary
 */
#ifdef CONFIG_TOUCHSCREEN_FACTORY_PLATFORM
static bool recovery_mode_enter;
static int __init sec_tsp_reboot_mode(char *mode)
{
	int ret;

	ret = strncmp(mode, "1", 1);
	if (ret == 0)
		recovery_mode_enter = true;
	else
		recovery_mode_enter = false;

	printk(KERN_ERR "%s: %d, %s\n", __func__, ret, mode);

	return 1;
}
__setup("androidboot.check_recovery_condition=0x", sec_tsp_reboot_mode);
#endif

/*
 * TSP IC power supplsy
 * ATMEL using 3.3V(L17), 3.0V(L16), 1.8V(L22).
 * SYNAP using 3.3V(L17), 1.8V(L22).
 */

void synaptics_tsp_register_callback(struct synaptics_rmi_callbacks *cb)
{
	charger_callbacks = cb;
	pr_info("%s: [synaptics] charger callback!\n", __func__);
}

static int touch_sleep_time;

void synaptics_power_onoff(bool enable)
{
	int ret = 0;
	/* 3.3V */
	static struct regulator *reg_l17;

	/* 1.8V */
	static struct regulator *reg_l22;

	if (!reg_l17) {
		reg_l17 = regulator_get(NULL, "8921_l17");
		if (IS_ERR(reg_l17)) {
			printk(KERN_ERR "%s: could not get 8921_l17, rc = %ld\n",
					__func__, PTR_ERR(reg_l17));
			return;
		}
		ret = regulator_set_voltage(reg_l17, 3300000, 3300000);
		if (ret) {
			printk(KERN_ERR "%s: unable to set ldo17 voltage to 3.3V\n",
					__func__);
			return;
		}
	}

	if (!reg_l22) {
		reg_l22 = regulator_get(NULL, "8921_l22");
		if (IS_ERR(reg_l22)) {
			printk(KERN_ERR "%s: could not get 8921_l22, rc = %ld\n",
					__func__, PTR_ERR(reg_l22));
			return;
		}
	}
	ret = regulator_set_voltage(reg_l22, 1800000, 1800000);
	if (ret) {
		printk(KERN_ERR"%s: unable to set ldo22 voltage to 1.8V\n",
				__func__);
		return;
	}

	if (enable) {
		if (!regulator_is_enabled(reg_l17)) {
			ret = regulator_enable(reg_l17);
			if (ret)
				printk(KERN_ERR "%s: enable l17 failed, rc=%d\n",
						__func__, ret);
			else
				printk(KERN_INFO "%s: enable l17\n", __func__);
		}

		if (!regulator_is_enabled(reg_l22)) {
			ret = regulator_enable(reg_l22);
			if (ret)
				printk(KERN_ERR "%s: enable l22 failed, rc=%d\n",
						__func__, ret);
			else
				printk(KERN_INFO "%s: enable l22\n", __func__);
		}

		msleep(touch_sleep_time);

	} else {
		if (regulator_is_enabled(reg_l17)) {
			ret = regulator_disable(reg_l17);
			if (ret)
				printk(KERN_ERR "%s: disable l17 failed, rc=%d\n",
						__func__, ret);
			else
				printk(KERN_INFO "%s: disable l17\n", __func__);
		}

		if (regulator_is_enabled(reg_l22)) {
			ret = regulator_disable(reg_l22);
			if (ret)
				printk(KERN_ERR "%s: disable l22 failed, rc=%d\n",
						__func__, ret);
			else
				printk(KERN_INFO "%s: disable l22\n", __func__);
		}
	}

}

static unsigned char tm1940_f1a_button_codes[] = {KEY_MENU, KEY_BACK};

static struct synaptics_rmi_f1a_button_map tm1940_f1a_button_map = {
	.nbuttons = ARRAY_SIZE(tm1940_f1a_button_codes),
	.map = tm1940_f1a_button_codes,
};

#define SYNAPTICS_MAX_X_SIZE	1079
#define SYNAPTICS_MAX_Y_SIZE	1919
#define SYNAPTICS_MAX_WIDTH	SYNAPTICS_MAX_Y_SIZE
#define NUM_RX	28
#define NUM_TX	16

static struct synaptics_rmi4_platform_data rmi4_platformdata = {
	/*.irq_type = IRQF_TRIGGER_FALLING,*/
	/*Direct IRQ for secure input*/
	.irq_type = IRQF_TRIGGER_RISING | IRQF_ONESHOT,
#ifdef CONFIG_TOUCHSCREEN_FACTORY_PLATFORM
	.factory_flatform = true,
#else
	.factory_flatform = false,
#endif
	.gpio = GPIO_TOUCH_IRQ,
	.max_x = SYNAPTICS_MAX_X_SIZE,
	.max_y = SYNAPTICS_MAX_Y_SIZE,
	.max_width = SYNAPTICS_MAX_WIDTH,
	.num_of_rx = NUM_RX,
	.num_of_tx = NUM_TX,
	/*.gpio_config = synaptics_gpio_setup,*/
	.f1a_button_map = &tm1940_f1a_button_map,
	.power = synaptics_power_onoff,
	.register_cb = synaptics_tsp_register_callback,
#ifdef CONFIG_FB_MSM_MIPI_SAMSUNG_OCTA_VIDEO_FULL_HD_PT_PANEL
	.tout1_on = touch_tout1_on,
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_PREVENT_HSYNC_LEAKAGE)
	.hsync_onoff = lcd_hsync_onoff,
#endif
};

static struct i2c_board_info bus2_i2c_devices[] = {
     /* SYNAPTICS I2C */
	{
	I2C_BOARD_INFO("synaptics_rmi4_i2c", 0x20),
	.platform_data = &rmi4_platformdata,
	/*.irq = MSM_GPIO_TO_INT(GPIO_TOUCH_IRQ),*/
	/*Direct IRQ for secure input*/
	.irq = TLMM_MSM_DIR_CONN_IRQ_0,
	},
};

static struct i2c_board_info new_bus2_i2c_devices[] = {
     /* SYNAPTICS I2C */
	{
	I2C_BOARD_INFO("synaptics_rmi4_i2c", 0x20),
	.platform_data = &rmi4_platformdata,
	/*.irq = MSM_GPIO_TO_INT(NEW_GPIO_TOUCH_IRQ),*/
	/*Direct IRQ for secure input*/
	.irq = TLMM_MSM_DIR_CONN_IRQ_0,
	},
};

/* DBh(ID2) of LDI version represents touch IC type and EL type
 * So we need to verify this infomation(which panel is attatched)
 * to load proper firmware and display it on factory app.
 *
 * 7(Encap) 6~4(Touch ID) 3~2(DCDC) 1~0(EL)
 *
 * Touch ID defines
 * 0 = Atmel mxt540s_revG
 * 1 = Synaptic IC
 * 2 = Atmel mxt540s_revS Boot only
 * 3 = Atmel mxt540s_revS Pre alpha
 * 4 = Synaptic IC
 * 5 = Synaptic IC(Remove SUS, connect GND Ring-Guard)
 *
 * EL defines
 * 0 = M4
 * 1 = M4+
 */

void __init S5000_tsp_input_init(int version)
{
	int ret;
	int touch_type, el_type;
	printk(KERN_DEBUG "[TSP]START %s\n", __func__);

	touch_type = (version >> 12) & 0xF;
	el_type = (version >> 8) & 0x1;

	/* IF TSP IS is A1, B0 version : ID2 value is 40
	 * IF TSP IS is B0 version : ID2 value is more than 40
	 */
	if ((touch_type << 4 | el_type) > 0x40)
		touch_sleep_time = SYNAPTICS_HW_RESET_TIME_B0;
	else
		touch_sleep_time = SYNAPTICS_HW_RESET_TIME;

	if (touch_type < 5) {
		if (el_type)
			rmi4_platformdata.panel_revision = OCTA_PANEL_REVISION_40;
		else
			rmi4_platformdata.panel_revision = OCTA_PANEL_REVISION_34;
	} else if (touch_type < 8 && touch_type >= 5) {
		rmi4_platformdata.panel_revision = OCTA_PANEL_REVISION_43;
	} else {
		rmi4_platformdata.panel_revision = OCTA_PANEL_REVISION_51;
	}

	rmi4_platformdata.panel_touch_type = touch_type;
#ifdef CONFIG_TOUCHSCREEN_FACTORY_PLATFORM
	rmi4_platformdata.recovery_mode = recovery_mode_enter;
#else
	rmi4_platformdata.recovery_mode = false;
#endif
	printk(KERN_INFO "%s: synaptics : el_type : %x, touch_type = %x[%s]\n",
			__func__, el_type, touch_type,
			rmi4_platformdata.recovery_mode ? "recovery mode" : "normal mode");

#if defined(CONFIG_MACH_JF_ATT) || defined(CONFIG_MACH_JF_TMO) || defined(CONFIG_MACH_JF_EUR)
	if(system_rev >= BOARD_REV08)  {
		rmi4_platformdata.gpio = NEW_GPIO_TOUCH_IRQ;
		ret = gpio_request(NEW_GPIO_TOUCH_IRQ, "tsp_int");
		if (ret != 0) {
			printk(KERN_ERR"tsp int request failed, ret=%d", ret);
			return ;
		}
		gpio_tlmm_config(GPIO_CFG(NEW_GPIO_TOUCH_IRQ, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
		i2c_register_board_info(APQ_8064_GSBI3_QUP_I2C_BUS_ID, new_bus2_i2c_devices,
			ARRAY_SIZE(bus2_i2c_devices));
	}
	else {
		rmi4_platformdata.gpio = GPIO_TOUCH_IRQ;
		ret = gpio_request(GPIO_TOUCH_IRQ, "tsp_int");
		if (ret != 0) {
			printk(KERN_ERR"tsp int request failed, ret=%d", ret);
			return ;
		}
		gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_IRQ, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
		i2c_register_board_info(APQ_8064_GSBI3_QUP_I2C_BUS_ID, bus2_i2c_devices,
			ARRAY_SIZE(bus2_i2c_devices));
	}
#elif defined(CONFIG_MACH_JF_DCM)
        rmi4_platformdata.gpio = NEW_GPIO_TOUCH_IRQ;
		ret = gpio_request(NEW_GPIO_TOUCH_IRQ, "tsp_int");
		if (ret != 0) {
			printk(KERN_ERR"tsp int request failed, ret=%d", ret);
			return ;
		}
		gpio_tlmm_config(GPIO_CFG(NEW_GPIO_TOUCH_IRQ, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
		i2c_register_board_info(APQ_8064_GSBI3_QUP_I2C_BUS_ID, new_bus2_i2c_devices,
			ARRAY_SIZE(bus2_i2c_devices));
#elif defined(CONFIG_MACH_JACTIVE_ATT)
        rmi4_platformdata.gpio = NEW_GPIO_TOUCH_IRQ;
		ret = gpio_request(NEW_GPIO_TOUCH_IRQ, "tsp_int");
		if (ret != 0) {
			printk(KERN_ERR"tsp int request failed, ret=%d", ret);
			return ;
		}
		gpio_tlmm_config(GPIO_CFG(NEW_GPIO_TOUCH_IRQ, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
		i2c_register_board_info(APQ_8064_GSBI3_QUP_I2C_BUS_ID, new_bus2_i2c_devices,
			ARRAY_SIZE(bus2_i2c_devices));
#else
	if(system_rev >= BOARD_REV09) {
		rmi4_platformdata.gpio = NEW_GPIO_TOUCH_IRQ;
		ret = gpio_request(NEW_GPIO_TOUCH_IRQ, "tsp_int");
		if (ret != 0) {
			printk(KERN_ERR"tsp int request failed, ret=%d", ret);
			return ;
		}
		gpio_tlmm_config(GPIO_CFG(NEW_GPIO_TOUCH_IRQ, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
		i2c_register_board_info(APQ_8064_GSBI3_QUP_I2C_BUS_ID, new_bus2_i2c_devices,
			ARRAY_SIZE(bus2_i2c_devices));
	}
	else {
		rmi4_platformdata.gpio = GPIO_TOUCH_IRQ;
		ret = gpio_request(GPIO_TOUCH_IRQ, "tsp_int");
		if (ret != 0) {
			printk(KERN_ERR"tsp int request failed, ret=%d", ret);
			return ;
		}
		gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_IRQ, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
		i2c_register_board_info(APQ_8064_GSBI3_QUP_I2C_BUS_ID, bus2_i2c_devices,
			ARRAY_SIZE(bus2_i2c_devices));
	}
#endif
	printk(KERN_DEBUG "[TSP]END %s\n", __func__);
	return ;
}
