/*
 * Copyright (C) 2012 Samsung Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>

#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/apq8064-gpio.h>

#include "devices-msm8x60.h"
#include "board-8064.h"

#if defined(CONFIG_BATTERY_SAMSUNG)
#include <linux/battery/sec_battery.h>
#include <linux/battery/sec_fuelgauge.h>
#include <linux/battery/sec_charger.h>

#define SEC_BATTERY_PMIC_NAME ""

static unsigned int sec_bat_recovery_mode;
static sec_charging_current_t charging_current_table[] = {
	{1900,	1600,	200,	40*60},
	{460,	0,	0,	0},
	{460,	460,	200,	40*60},
	{1900,	1600,	200,	40*60},
	{460,	460,	200,	40*60},
	{1000,	1000,	200,	40*60},
	{1000,	1000,	200,	40*60},
	{460,	460,	200,	40*60},
	{1700,	1600,	200,	40*60},
	{0,	0,	0,	0},
#ifdef CONFIG_WIRELESS_CHARGER
	{650,	700,	200,	40*60},
#endif
	{1900,	1600,	200,	40*60},
	{0,	0,	0,	0},
	{0,	0,	0,	0},
};

static bool sec_bat_adc_none_init(
		struct platform_device *pdev) {return true; }
static bool sec_bat_adc_none_exit(void) {return true; }
static int sec_bat_adc_none_read(unsigned int channel) {return 0; }

static bool sec_bat_adc_ap_init(
		struct platform_device *pdev) {return true; }
static bool sec_bat_adc_ap_exit(void) {return true; }
static int sec_bat_adc_ap_read(unsigned int channel)
{
	int rc = -1, data = -1;
	struct pm8xxx_adc_chan_result result;

	switch (channel) {
	case SEC_BAT_ADC_CHANNEL_TEMP:
		rc = pm8xxx_adc_mpp_config_read(
			PM8XXX_AMUX_MPP_7, ADC_MPP_1_AMUX6, &result);
		if (rc) {
			pr_err("error reading mpp %d, rc = %d\n",
				PM8XXX_AMUX_MPP_7, rc);
			return rc;
		}

		/* use measurement, no need to scale */
		data = (int)result.measurement;
		break;
	case SEC_BAT_ADC_CHANNEL_TEMP_AMBIENT:
		data = 33000;
		break;

	case SEC_BAT_ADC_CHANNEL_BAT_CHECK:
		rc = pm8xxx_adc_read(
			ADC_MPP_1_AMUX8, &result);
		if (rc) {
			pr_err("error reading mpp %d, rc = %d\n",
				PM8XXX_AMUX_MPP_8, rc);
			return rc;
		}

		/* use measurement, no need to scale */
		data = (int)result.measurement/1000;
		break;

	default:
		pr_err("Invalid adc channel: %d\n", channel);
		break;
	}

	return data;
}

static bool sec_bat_adc_ic_init(
		struct platform_device *pdev) {return true; }
static bool sec_bat_adc_ic_exit(void) {return true; }
static int sec_bat_adc_ic_read(unsigned int channel) {return 0; }

static bool sec_bat_gpio_init(void)
{
	return true;
}

static struct i2c_gpio_platform_data gpio_i2c_data_fgchg = {
	.sda_pin = GPIO_FUELGAUGE_I2C_SDA,
	.scl_pin = GPIO_FUELGAUGE_I2C_SCL,
};

static bool sec_fg_gpio_init(void)
{
	struct pm_gpio param = {
		.direction     = PM_GPIO_DIR_IN,
		.pull          = PM_GPIO_PULL_NO,
		.vin_sel       = PM_GPIO_VIN_S4,
		.function      = PM_GPIO_FUNC_NORMAL,
	};

	/* FUEL_ALERT Setting */
	if (system_rev == 0x0) {
		sec_battery_pdata.fg_irq = PM8921_GPIO_IRQ(PM8921_IRQ_BASE,
				GPIO_FUEL_INT_04A);
		pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(GPIO_FUEL_INT_04A),
						&param);
#if defined(CONFIG_MACH_JF_ATT) || defined(CONFIG_MACH_JF_TMO)
	} else if (system_rev >= 8) {
#else
	} else if (system_rev >= 9) {
#endif
		/* FUEL_ALERT Registration */
		struct pm8xxx_mpp_config_data fuel_alert_mppcfg = {
			.type = PM8XXX_MPP_TYPE_D_INPUT,
			.level = PM8921_MPP_DIG_LEVEL_S4,
			.control = PM8XXX_MPP_DOUT_CTRL_LOW,
		};

		sec_battery_pdata.fg_irq = PM8921_MPP_IRQ(PM8921_IRQ_BASE, 6);
		pm8xxx_mpp_config(PM8921_MPP_PM_TO_SYS(6),
				&fuel_alert_mppcfg);
	}
	else
		gpio_tlmm_config(GPIO_CFG(GPIO_FUEL_INT,  0, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	gpio_tlmm_config(GPIO_CFG(gpio_i2c_data_fgchg.scl_pin, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	gpio_tlmm_config(GPIO_CFG(gpio_i2c_data_fgchg.sda_pin,  0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	gpio_set_value(gpio_i2c_data_fgchg.scl_pin, 1);
	gpio_set_value(gpio_i2c_data_fgchg.sda_pin, 1);

	return true;
}

static bool sec_chg_gpio_init(void)
{
	return true;
}

static bool sec_bat_is_lpm(void) {return (bool)poweroff_charging; }

int extended_cable_type;

static void sec_bat_initial_check(void)
{
	union power_supply_propval value;

	if (POWER_SUPPLY_TYPE_BATTERY < current_cable_type) {
		value.intval = current_cable_type<<ONLINE_TYPE_MAIN_SHIFT;
		psy_do_property("battery", set,
				POWER_SUPPLY_PROP_ONLINE, value);
	} else {
		psy_do_property("sec-charger", get,
				POWER_SUPPLY_PROP_ONLINE, value);
		if (value.intval == POWER_SUPPLY_TYPE_WIRELESS) {
			value.intval =
			POWER_SUPPLY_TYPE_WIRELESS<<ONLINE_TYPE_MAIN_SHIFT;
			psy_do_property("battery", set,
					POWER_SUPPLY_PROP_ONLINE, value);
		}
	}
}

static bool sec_bat_check_jig_status(void)
{
	return (current_cable_type == POWER_SUPPLY_TYPE_UARTOFF);
}
static bool sec_bat_switch_to_check(void) {return true; }
static bool sec_bat_switch_to_normal(void) {return true; }

static int sec_bat_check_cable_callback(void)
{
	return current_cable_type;
}

static int sec_bat_get_cable_from_extended_cable_type(
	int input_extended_cable_type)
{
	int cable_main, cable_sub, cable_power;
	int cable_type = POWER_SUPPLY_TYPE_UNKNOWN;
	union power_supply_propval value;
	int charge_current_max = 0, charge_current = 0;

	cable_main = GET_MAIN_CABLE_TYPE(input_extended_cable_type);
	if (cable_main != POWER_SUPPLY_TYPE_UNKNOWN)
		extended_cable_type = (extended_cable_type &
			~(int)ONLINE_TYPE_MAIN_MASK) |
			(cable_main << ONLINE_TYPE_MAIN_SHIFT);
	cable_sub = GET_SUB_CABLE_TYPE(input_extended_cable_type);
	if (cable_sub != ONLINE_SUB_TYPE_UNKNOWN)
		extended_cable_type = (extended_cable_type &
			~(int)ONLINE_TYPE_SUB_MASK) |
			(cable_sub << ONLINE_TYPE_SUB_SHIFT);
	cable_power = GET_POWER_CABLE_TYPE(input_extended_cable_type);
	if (cable_power != ONLINE_POWER_TYPE_UNKNOWN)
		extended_cable_type = (extended_cable_type &
			~(int)ONLINE_TYPE_PWR_MASK) |
			(cable_power << ONLINE_TYPE_PWR_SHIFT);

	switch (cable_main) {
	case POWER_SUPPLY_TYPE_CARDOCK:
		switch (cable_power) {
		case ONLINE_POWER_TYPE_BATTERY:
			cable_type = POWER_SUPPLY_TYPE_BATTERY;
			break;
		case ONLINE_POWER_TYPE_TA:
			switch (cable_sub) {
			case ONLINE_SUB_TYPE_MHL:
				cable_type = POWER_SUPPLY_TYPE_USB;
				break;
			case ONLINE_SUB_TYPE_AUDIO:
			case ONLINE_SUB_TYPE_DESK:
			case ONLINE_SUB_TYPE_SMART_NOTG:
			case ONLINE_SUB_TYPE_KBD:
				cable_type = POWER_SUPPLY_TYPE_MAINS;
				break;
			case ONLINE_SUB_TYPE_SMART_OTG:
				cable_type = POWER_SUPPLY_TYPE_CARDOCK;
				break;
			}
			break;
		case ONLINE_POWER_TYPE_USB:
			cable_type = POWER_SUPPLY_TYPE_USB;
			break;
		default:
			cable_type = current_cable_type;
			break;
		}
		break;
	case POWER_SUPPLY_TYPE_MISC:
		switch (cable_sub) {
		case ONLINE_SUB_TYPE_MHL:
			switch (cable_power) {
			case ONLINE_POWER_TYPE_BATTERY:
				cable_type = POWER_SUPPLY_TYPE_BATTERY;
				break;
			case ONLINE_POWER_TYPE_MHL_500:
				cable_type = POWER_SUPPLY_TYPE_MISC;
				charge_current_max = 400;
				charge_current = 400;
				break;
			case ONLINE_POWER_TYPE_MHL_900:
				cable_type = POWER_SUPPLY_TYPE_MISC;
				charge_current_max = 700;
				charge_current = 700;
				break;
			case ONLINE_POWER_TYPE_MHL_1500:
				cable_type = POWER_SUPPLY_TYPE_MISC;
				charge_current_max = 1300;
				charge_current = 1300;
				break;
			case ONLINE_POWER_TYPE_USB:
				cable_type = POWER_SUPPLY_TYPE_USB;
				charge_current_max = 300;
				charge_current = 300;
				break;
			default:
				cable_type = cable_main;
			}
			break;
		default:
			cable_type = cable_main;
			break;
		}
		break;
	default:
		cable_type = cable_main;
		break;
	}

	if (charge_current_max == 0) {
		charge_current_max =
			charging_current_table[cable_type].input_current_limit;
		charge_current =
			charging_current_table[cable_type].
			fast_charging_current;
	}
	value.intval = charge_current_max;
	psy_do_property(sec_battery_pdata.charger_name, set,
			POWER_SUPPLY_PROP_CURRENT_MAX, value);
	value.intval = charge_current;
	psy_do_property(sec_battery_pdata.charger_name, set,
			POWER_SUPPLY_PROP_CURRENT_AVG, value);
	return cable_type;
}

static bool sec_bat_check_cable_result_callback(
				int cable_type)
{
	struct regulator *l29;
	current_cable_type = cable_type;

	if(system_rev >= 0x8)
	{
		if (current_cable_type == POWER_SUPPLY_TYPE_BATTERY)
		{
			pr_info("%s set ldo off\n", __func__);
			l29 = regulator_get(NULL, "8921_l29");
			if(l29 > 0)
			{
				regulator_disable(l29);
			}
		}
		else
		{
			pr_info("%s set ldo on\n", __func__);
			l29 = regulator_get(NULL, "8921_l29");
			if(l29 > 0)
			{
				regulator_enable(l29);
			}
		}
	}
	return true;
}

/* callback for battery check
 * return : bool
 * true - battery detected, false battery NOT detected
 */
static bool sec_bat_check_callback(void)
{
	struct power_supply *psy;
	union power_supply_propval value;

	psy = get_power_supply_by_name(("sec-charger"));
	if (!psy) {
		pr_err("%s: Fail to get psy (%s)\n",
			__func__, "sec_charger");
		value.intval = 1;
	} else {
		int ret;
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &(value));
		if (ret < 0) {
			pr_err("%s: Fail to sec-charger get_property (%d=>%d)\n",
				__func__, POWER_SUPPLY_PROP_PRESENT, ret);
			value.intval = 1;
		}
	}

	return value.intval;
}
static bool sec_bat_check_result_callback(void) {return true; }

/* callback for OVP/UVLO check
 * return : int
 * battery health
 */
static int sec_bat_ovp_uvlo_callback(void)
{
	int health;
	health = POWER_SUPPLY_HEALTH_GOOD;

	return health;
}

static bool sec_bat_ovp_uvlo_result_callback(int health) {return true; }

/*
 * val.intval : temperature
 */
static bool sec_bat_get_temperature_callback(
		enum power_supply_property psp,
		union power_supply_propval *val) {return true; }
static bool sec_fg_fuelalert_process(bool is_fuel_alerted) {return true; }

#if defined(CONFIG_MACH_JF_DCM)
static const sec_bat_adc_table_data_t temp_table[] = {
	{27120,	700},
	{27585,	650},
	{28110,	600},
	{28655,	550},
	{29315,	500},
	{30075,	450},
	{30780,	400},
	{31780,	350},
	{32713,	300},
	{33870,	250},
	{35000,	200},
	{36075,	150},
	{37125,	100},
	{38110,	50},
	{38995,	0},
	{39710,	-50},
	{40480,	-100},
	{41040,	-150},
	{41510,	-200},
	{41934,	-250},
};
#elif defined(CONFIG_MACH_JACTIVE_ATT)
static const sec_bat_adc_table_data_t temp_table[] = {
	{27039,	700},
	{27264,	670},
	{27435,	650},
	{27645,	630},
	{27982,	600},
	{28561,	550},
	{29202,	500},
	{29946,	450},
	{30318,	430},
	{30845,	400},
	{31630,	350},
	{32871,	300},
	{33975,	250},
	{35091,	200},
	{36174,	150},
	{37243,	100},
	{38198,	50},
	{39157,	0},
	{39676,	-30},
	{39974,	-50},
	{40690,	-100},
	{41183,	-150},
	{41685,	-200},
	{42089,	-250},
	{42370,	-300},
};
#else
static const sec_bat_adc_table_data_t temp_table[] = {
	{27188,	700},
	{27605,	650},
	{28182,	600},
	{28748,	550},
	{29265,	500},
	{29900,	450},
	{30981,	400},
	{31896,	350},
	{32820,	300},
	{33956,	250},
	{35037,	200},
	{36083,	150},
	{37180,	100},
	{38267,	50},
	{39148,	0},
	{39916,	-50},
	{40586,	-100},
	{41222,	-150},
	{41573,	-200},
	{41943, -250},
};
#endif

/* ADC region should be exclusive */
static sec_bat_adc_region_t cable_adc_value_table[] = {
	{0,	0},
	{0,	0},
	{0,	0},
	{0,	0},
	{0,	0},
	{0,	0},
	{0,	0},
	{0,	0},
	{0,	0},
	{0,	0},
	{0,	0},
};

static int polling_time_table[] = {
	10,	/* BASIC */
	30,	/* CHARGING */
	30,	/* DISCHARGING */
	30,	/* NOT_CHARGING */
	60 * 60,	/* SLEEP */
};

#if defined(CONFIG_MACH_JF_ATT) || defined(CONFIG_MACH_JF_TMO) || \
	defined(CONFIG_MACH_JF_SPR) || defined(CONFIG_MACH_JF_USC) || \
	defined(CONFIG_MACH_JF_VZW)
/* for MAX17048 */
static struct battery_data_t fusion3_battery_data[] = {
	/* SDI battery data (High voltage 4.35V) */
	{
		.RCOMP0 = 0x73,
		.RCOMP_charging = 0x70,
		.temp_cohot = -700,
		.temp_cocold = -4875,
		.is_using_model_data = true,
		.type_str = "SDI",
	}
};
#else
/* for MAX17048 */
static struct battery_data_t fusion3_battery_data[] = {
	/* SDI battery data (High voltage 4.35V) */
	{
		.RCOMP0 = 0x76,
		.RCOMP_charging = 0x76,
		.temp_cohot = -700,
		.temp_cocold = -4875,
		.is_using_model_data = true,
		.type_str = "SDI",
	}
};
#endif

sec_battery_platform_data_t sec_battery_pdata = {
	/* NO NEED TO BE CHANGED */
	.initial_check = sec_bat_initial_check,
	.bat_gpio_init = sec_bat_gpio_init,
	.fg_gpio_init = sec_fg_gpio_init,
	.chg_gpio_init = sec_chg_gpio_init,

	.is_lpm = sec_bat_is_lpm,
	.check_jig_status = sec_bat_check_jig_status,
	.check_cable_callback =
		sec_bat_check_cable_callback,
	.get_cable_from_extended_cable_type =
		sec_bat_get_cable_from_extended_cable_type,
	.cable_switch_check = sec_bat_switch_to_check,
	.cable_switch_normal = sec_bat_switch_to_normal,
	.check_cable_result_callback =
		sec_bat_check_cable_result_callback,
	.check_battery_callback =
		sec_bat_check_callback,
	.check_battery_result_callback =
		sec_bat_check_result_callback,
	.ovp_uvlo_callback = sec_bat_ovp_uvlo_callback,
	.ovp_uvlo_result_callback =
		sec_bat_ovp_uvlo_result_callback,
	.fuelalert_process = sec_fg_fuelalert_process,
	.get_temperature_callback =
		sec_bat_get_temperature_callback,

	.adc_api[SEC_BATTERY_ADC_TYPE_NONE] = {
		.init = sec_bat_adc_none_init,
		.exit = sec_bat_adc_none_exit,
		.read = sec_bat_adc_none_read
		},
	.adc_api[SEC_BATTERY_ADC_TYPE_AP] = {
		.init = sec_bat_adc_ap_init,
		.exit = sec_bat_adc_ap_exit,
		.read = sec_bat_adc_ap_read
		},
	.adc_api[SEC_BATTERY_ADC_TYPE_IC] = {
		.init = sec_bat_adc_ic_init,
		.exit = sec_bat_adc_ic_exit,
		.read = sec_bat_adc_ic_read
		},
	.cable_adc_value = cable_adc_value_table,
	.charging_current = charging_current_table,
	.polling_time = polling_time_table,
	/* NO NEED TO BE CHANGED */

	.pmic_name = SEC_BATTERY_PMIC_NAME,

	.adc_check_count = 6,
	.adc_type = {
		SEC_BATTERY_ADC_TYPE_NONE,	/* CABLE_CHECK */
		SEC_BATTERY_ADC_TYPE_AP,	/* BAT_CHECK */
		SEC_BATTERY_ADC_TYPE_AP,	/* TEMP */
		SEC_BATTERY_ADC_TYPE_AP,	/* TEMP_AMB */
		SEC_BATTERY_ADC_TYPE_AP,	/* FULL_CHECK */
	},

	/* Battery */
	.vendor = "SDI SDI",
	.technology = POWER_SUPPLY_TECHNOLOGY_LION,
	.battery_data = (void *)fusion3_battery_data,
	.bat_gpio_ta_nconnected = 0,
	.bat_polarity_ta_nconnected = 0,
	.bat_irq = IF_PMIC_IRQ_BASE + MAX77693_CHG_IRQ_BATP_I,
	.bat_irq_attr = IRQF_TRIGGER_FALLING,
#if defined(CONFIG_MACH_JF_VZW)
	.cable_check_type =
		SEC_BATTERY_CABLE_CHECK_PSY |
		SEC_BATTERY_CABLE_CHECK_NOINCOMPATIBLECHARGE,
#else
	.cable_check_type =
		SEC_BATTERY_CABLE_CHECK_PSY,
#endif
	.cable_source_type =
		SEC_BATTERY_CABLE_SOURCE_EXTERNAL |
		SEC_BATTERY_CABLE_SOURCE_EXTENDED,

	.event_check = true,
	.event_waiting_time = 600,

	/* Monitor setting */
	.polling_type = SEC_BATTERY_MONITOR_ALARM,
	.monitor_initial_count = 3,

	/* Battery check */
	.battery_check_type = SEC_BATTERY_CHECK_INT,
	.check_count = 0,
	/* Battery check by ADC */
	.check_adc_max = 1440,
	.check_adc_min = 0,

	/* OVP/UVLO check */
	.ovp_uvlo_check_type = SEC_BATTERY_OVP_UVLO_CHGPOLLING,

	/* Temperature check */
	.thermal_source = SEC_BATTERY_THERMAL_SOURCE_ADC,
	.temp_adc_table = temp_table,
	.temp_adc_table_size =
		sizeof(temp_table)/sizeof(sec_bat_adc_table_data_t),
	.temp_amb_adc_table = temp_table,
	.temp_amb_adc_table_size =
		sizeof(temp_table)/sizeof(sec_bat_adc_table_data_t),

	.temp_check_type = SEC_BATTERY_TEMP_CHECK_TEMP,
	.temp_check_count = 1,

#if defined(CONFIG_MACH_JF_USC)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 450,
	.temp_low_threshold_event = -50,
	.temp_low_recovery_event = 0,

	.temp_high_threshold_normal = 600,
	.temp_high_recovery_normal = 420,
	.temp_low_threshold_normal = -50,
	.temp_low_recovery_normal = 0,

	.temp_high_threshold_lpm = 600,
	.temp_high_recovery_lpm = 420,
	.temp_low_threshold_lpm = -50,
	.temp_low_recovery_lpm = 0,
#elif defined(CONFIG_MACH_JACTIVE_ATT)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 400,
	.temp_low_threshold_event = -30,
	.temp_low_recovery_event = 0,

	.temp_high_threshold_normal = 470,
	.temp_high_recovery_normal = 400,
	.temp_low_threshold_normal = -30,
	.temp_low_recovery_normal = 0,

	.temp_high_threshold_lpm = 500,
	.temp_high_recovery_lpm = 430,
	.temp_low_threshold_lpm = -30,
	.temp_low_recovery_lpm = 0,
#elif defined(CONFIG_MACH_JF_CRI)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 450,
	.temp_low_threshold_event = -50,
	.temp_low_recovery_event = 0,

	.temp_high_threshold_normal = 500,
	.temp_high_recovery_normal = 420,
	.temp_low_threshold_normal = -50,
	.temp_low_recovery_normal = 0,

	.temp_high_threshold_lpm = 460,
	.temp_high_recovery_lpm = 420,
	.temp_low_threshold_lpm = -50,
	.temp_low_recovery_lpm = 0,
#elif defined(CONFIG_MACH_JF_EUR)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 400,
	.temp_low_threshold_event = -50,
	.temp_low_recovery_event = 0,

	.temp_high_threshold_normal = 600,
	.temp_high_recovery_normal = 400,
	.temp_low_threshold_normal = -50,
	.temp_low_recovery_normal = 0,

	.temp_high_threshold_lpm = 600,
	.temp_high_recovery_lpm = 400,
	.temp_low_threshold_lpm = -50,
	.temp_low_recovery_lpm = 0,
#elif defined(CONFIG_MACH_JF_SPR)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 400,
	.temp_low_threshold_event = -30,
	.temp_low_recovery_event = 0,

	.temp_high_threshold_normal = 500,
	.temp_high_recovery_normal = 430,
	.temp_low_threshold_normal = -30,
	.temp_low_recovery_normal = 0,

	.temp_high_threshold_lpm = 473,
	.temp_high_recovery_lpm = 448,
	.temp_low_threshold_lpm = -10,
	.temp_low_recovery_lpm = 20,
#elif defined(CONFIG_MACH_JF_VZW)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 400,
	.temp_low_threshold_event = -30,
	.temp_low_recovery_event = 0,

	.temp_high_threshold_normal = 520,
	.temp_high_recovery_normal = 460,
	.temp_low_threshold_normal = -30,
	.temp_low_recovery_normal = 40,

	.temp_high_threshold_lpm = 480,
	.temp_high_recovery_lpm = 450,
	.temp_low_threshold_lpm = -30,
	.temp_low_recovery_lpm = 0,
#elif defined(CONFIG_MACH_JF_TMO)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 400,
	.temp_low_threshold_event = -30,
	.temp_low_recovery_event = 0,

	.temp_high_threshold_normal = 480,
	.temp_high_recovery_normal = 400,
	.temp_low_threshold_normal = -30,
	.temp_low_recovery_normal = 0,

	.temp_high_threshold_lpm = 480,
	.temp_high_recovery_lpm = 430,
	.temp_low_threshold_lpm = -30,
	.temp_low_recovery_lpm = 0,
#elif defined(CONFIG_MACH_JF_ATT)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 400,
	.temp_low_threshold_event = -30,
	.temp_low_recovery_event = 0,

	.temp_high_threshold_normal = 480,
	.temp_high_recovery_normal = 400,
	.temp_low_threshold_normal = -30,
	.temp_low_recovery_normal = 0,

	.temp_high_threshold_lpm = 480,
	.temp_high_recovery_lpm = 430,
	.temp_low_threshold_lpm = -30,
	.temp_low_recovery_lpm = 0,
#elif defined(CONFIG_MACH_JF_SKT)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 420,
	.temp_low_threshold_event = -34,
	.temp_low_recovery_event = -1,

	.temp_high_threshold_normal = 600,
	.temp_high_recovery_normal = 420,
	.temp_low_threshold_normal = -34,
	.temp_low_recovery_normal = -1,

	.temp_high_threshold_lpm = 600,
	.temp_high_recovery_lpm = 420,
	.temp_low_threshold_lpm = -34,
	.temp_low_recovery_lpm = -1,
#elif defined(CONFIG_MACH_JF_KTT)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 420,
	.temp_low_threshold_event = -34,
	.temp_low_recovery_event = -1,

	.temp_high_threshold_normal = 600,
	.temp_high_recovery_normal = 420,
	.temp_low_threshold_normal = -34,
	.temp_low_recovery_normal = -1,

	.temp_high_threshold_lpm = 600,
	.temp_high_recovery_lpm = 420,
	.temp_low_threshold_lpm = -34,
	.temp_low_recovery_lpm = -1,
#elif defined(CONFIG_MACH_JF_LGT)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 420,
	.temp_low_threshold_event = -34,
	.temp_low_recovery_event = -1,

	.temp_high_threshold_normal = 600,
	.temp_high_recovery_normal = 420,
	.temp_low_threshold_normal = -34,
	.temp_low_recovery_normal = -1,

	.temp_high_threshold_lpm = 600,
	.temp_high_recovery_lpm = 420,
	.temp_low_threshold_lpm = -34,
	.temp_low_recovery_lpm = -1,
#elif defined(CONFIG_MACH_JF_DCM)
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 400,
	.temp_low_threshold_event = -50,
	.temp_low_recovery_event = 0,

	.temp_high_threshold_normal = 600,
	.temp_high_recovery_normal = 400,
	.temp_low_threshold_normal = -50,
	.temp_low_recovery_normal = 0,

	.temp_high_threshold_lpm = 600,
	.temp_high_recovery_lpm = 400,
	.temp_low_threshold_lpm = -50,
	.temp_low_recovery_lpm = 0,
#else
	.temp_high_threshold_event = 600,
	.temp_high_recovery_event = 400,
	.temp_low_threshold_event = -30,
	.temp_low_recovery_event = 0,

	.temp_high_threshold_normal = 450,
	.temp_high_recovery_normal = 400,
	.temp_low_threshold_normal = -30,
	.temp_low_recovery_normal = 0,

	.temp_high_threshold_lpm = 450,
	.temp_high_recovery_lpm = 400,
	.temp_low_threshold_lpm = -30,
	.temp_low_recovery_lpm = 0,
#endif

	.full_check_type = SEC_BATTERY_FULLCHARGED_CHGPSY,
	.full_check_type_2nd = SEC_BATTERY_FULLCHARGED_TIME,
	.full_check_count = 1,
	.chg_gpio_full_check = 0,
	.chg_polarity_full_check = 1,
	.full_condition_type = SEC_BATTERY_FULL_CONDITION_SOC |
		SEC_BATTERY_FULL_CONDITION_NOTIMEFULL |
		SEC_BATTERY_FULL_CONDITION_VCELL,
	.full_condition_soc = 97,
	.full_condition_vcell = 4300,

	.recharge_check_count = 2,
	.recharge_condition_type =
		SEC_BATTERY_RECHARGE_CONDITION_VCELL,
	.recharge_condition_soc = 98,
	.recharge_condition_vcell = 4300,

	.charging_total_time = 6 * 60 * 60,
	.recharging_total_time = 90 * 60,
	.charging_reset_time = 0,

	/* Fuel Gauge */
	.fg_irq = MSM_GPIO_TO_INT(GPIO_FUEL_INT),
	.fg_irq_attr =
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
	.fuel_alert_soc = 2,
	.repeated_fuelalert = false,
	.capacity_calculation_type =
		SEC_FUELGAUGE_CAPACITY_TYPE_RAW |
		SEC_FUELGAUGE_CAPACITY_TYPE_SCALE |
		SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE |
		SEC_FUELGAUGE_CAPACITY_TYPE_ATOMIC,
	.capacity_max = 1000,
	.capacity_max_margin = 50,
	.capacity_min = 0,

	/* Charger */
	.charger_name = "sec-charger",
	.chg_gpio_en = 0,
	.chg_polarity_en = 0,
	.chg_gpio_status = 0,
	.chg_polarity_status = 0,
	.chg_irq = 0,
	.chg_irq_attr = IRQF_TRIGGER_FALLING,
	.chg_float_voltage = 4350,
};

static struct platform_device sec_device_battery = {
	.name = "sec-battery",
	.id = -1,
	.dev.platform_data = &sec_battery_pdata,
};

struct platform_device sec_device_fgchg = {
	.name = "i2c-gpio",
	.id = MSM_FUELGAUGE_I2C_BUS_ID,
	.dev.platform_data = &gpio_i2c_data_fgchg,
};

static struct i2c_board_info sec_brdinfo_fgchg[] __initdata = {
#if !defined(CONFIG_MFD_MAX77693)
	{
		I2C_BOARD_INFO("sec-charger",
			SEC_CHARGER_I2C_SLAVEADDR),
		.platform_data	= &sec_battery_pdata,
	},
#endif
	{
		I2C_BOARD_INFO("sec-fuelgauge",
			SEC_FUELGAUGE_I2C_SLAVEADDR),
		.platform_data	= &sec_battery_pdata,
	},
};

static struct platform_device *msm8960_battery_devices[] __initdata = {
	&sec_device_fgchg,
	&sec_device_battery,
};

static int __init sec_bat_current_boot_mode(char *mode)
{
	/*
	*	1 is recovery booting
	*	0 is normal booting
	*/

	if (strncmp(mode, "1", 1) == 0)
		sec_bat_recovery_mode = 1;
	else
		sec_bat_recovery_mode = 0;

	pr_info("%s : %s", __func__, sec_bat_recovery_mode == 1 ?
				"recovery" : "normal");

	return 1;
}
__setup("androidboot.boot_recovery=", sec_bat_current_boot_mode);

void __init msm8960_init_battery(void)
{
	/* FUEL_SDA/SCL setting */
	if ((system_rev > 0) && (system_rev < 6)) {
		gpio_i2c_data_fgchg.sda_pin = GPIO_FUELGAUGE_I2C_SDA_OLD;
		gpio_i2c_data_fgchg.scl_pin = GPIO_FUELGAUGE_I2C_SCL_OLD;
	}
	platform_add_devices(
		msm8960_battery_devices,
		ARRAY_SIZE(msm8960_battery_devices));

	i2c_register_board_info(
		MSM_FUELGAUGE_I2C_BUS_ID,
		sec_brdinfo_fgchg,
		ARRAY_SIZE(sec_brdinfo_fgchg));
}

#endif
