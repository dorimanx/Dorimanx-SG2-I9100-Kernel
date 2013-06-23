/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>

#include <mach/camera.h>
#include <mach/msm_bus_board.h>
#include <mach/gpiomux.h>

#if defined(CONFIG_MACH_JACTIVE_ATT)
#include <mach/pmic.h>
#include "devices-msm8x60.h"
#endif

#include "devices.h"
#include "board-8064.h"
#include <linux/spi/spi.h>

#define GPIO_CAM_CORE_EN	PM8921_GPIO_PM_TO_SYS(4)
#define GPIO_CAM_A_EN	PM8921_GPIO_PM_TO_SYS(5)
#define GPIO_CAM_A_EN2	PM8921_GPIO_PM_TO_SYS(32)/* >= REV06 */
#define GPIO_CAM_AF_EN	PM8921_GPIO_PM_TO_SYS(15)
#define GPIO_CAM_ISP_INT	PM8921_GPIO_PM_TO_SYS(24)
#define GPIO_13M_CAM_RESET	PM8921_GPIO_PM_TO_SYS(6)
#define GPIO_CAM_SENSOR_DET	PM8921_GPIO_PM_TO_SYS(12)
#define GPIO_CAM_VT_EN	PM8921_GPIO_PM_TO_SYS(26)
#define GPIO_CAM_MCLK	5

#if defined CONFIG_CAMERA_USE_GSBI7
#define GPIO_CAM_SDA	84
#define GPIO_CAM_SCL	85
#else
#define GPIO_CAM_SDA	12
#define GPIO_CAM_SCL	13
#endif

/* Front Camera */
#define GPIO_VT_CAM_STBY PM8921_GPIO_PM_TO_SYS(17)
#define GPIO_VT_CAM_RESET PM8921_GPIO_PM_TO_SYS(23)
#define GPIO_VT_CAM_ID PM8921_GPIO_PM_TO_SYS(1)
#define GPIO_VT_CAM_MCLK	2
#ifdef CONFIG_CAMERA_SW_I2C
#define GPIO_VT_CAM_SCL	70
#define GPIO_VT_CAM_SDA	71
#else
#define GPIO_VT_CAM_SDA	84
#define GPIO_VT_CAM_SCL	85
#endif
#ifdef CONFIG_CAMERA_SW_I2C_ACT
#define GPIO_CAM_AF_SCL	70
#define GPIO_CAM_AF_SDA	71
#endif


/* Concurrent camera */
static int gpio_ref_count;


#ifdef CONFIG_MSM_CAMERA

static struct gpiomux_setting cam_settings[] = {
	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},

	{
		.func = GPIOMUX_FUNC_1, /*active 1*/
#if defined(CONFIG_MACH_JACTIVE_ATT)			
		.drv = GPIOMUX_DRV_4MA,
#else
		.drv = GPIOMUX_DRV_2MA,
#endif
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 2*/
		.drv = GPIOMUX_DRV_2MA,
#if defined(CONFIG_MACH_JACTIVE_ATT) || defined(CONFIG_MACH_JACTIVE_EUR)
		.pull = GPIOMUX_PULL_DOWN,
#else
		.pull = GPIOMUX_PULL_NONE,
#endif
	},

	{
		.func = GPIOMUX_FUNC_2, /*active 3*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_5, /*active 4*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_6, /*active 5*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_2, /*active 6*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_3, /*active 7*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*i2c suspend*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_KEEPER,
	},

	{
		.func = GPIOMUX_FUNC_9, /*active 9*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_A, /*active 10*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_6, /*active 11*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_4, /*active 12*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},
	{
		.func = GPIOMUX_FUNC_2, /*active 13*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend (no pull) 14*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},
	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend (pull down) 15*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},
};

static struct msm_gpiomux_config apq8064_cam_common_configs[] = {
	{
		.gpio = GPIO_CAM_MCLK,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[1],
			[GPIOMUX_SUSPENDED] = &cam_settings[14],
		},
	},
#if defined (CONFIG_CAMERA_USE_GSBI7)
	{
		.gpio = GPIO_CAM_SDA,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[13],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = GPIO_CAM_SCL,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[7],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
#else
	{
		.gpio = GPIO_CAM_SDA,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[11],
			[GPIOMUX_SUSPENDED] = &cam_settings[15],
		},
	},
	{
		.gpio = GPIO_CAM_SCL,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[11],
			[GPIOMUX_SUSPENDED] = &cam_settings[15],
		},
	},
#endif
	{
		.gpio = GPIO_VT_CAM_MCLK,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[12],
			[GPIOMUX_SUSPENDED] = &cam_settings[14],
		},
	},
#if defined (CONFIG_CAMERA_SW_I2C)
	{
		.gpio = GPIO_VT_CAM_SDA,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = GPIO_VT_CAM_SCL,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	}
#else
	{
		.gpio = GPIO_VT_CAM_SDA,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[13],
			[GPIOMUX_SUSPENDED] = &cam_settings[15],
		},
	},
	{
		.gpio = GPIO_VT_CAM_SCL,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[7],
			[GPIOMUX_SUSPENDED] = &cam_settings[15],
		},
	},
#endif
#if defined (CONFIG_CAMERA_SW_I2C_ACT)
	{
		.gpio = GPIO_CAM_AF_SDA, // 71
		.settings = {
			[GPIOMUX_ACTIVE]	= &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = GPIO_CAM_AF_SCL, // 70
		.settings = {
			[GPIOMUX_ACTIVE]	= &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	}
#endif
};

#define VFE_CAMIF_TIMER1_GPIO 3
#define VFE_CAMIF_TIMER2_GPIO 1

static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_EXT,
	._fsrc.ext_driver_src.led_en = VFE_CAMIF_TIMER1_GPIO,
	._fsrc.ext_driver_src.led_flash_en = VFE_CAMIF_TIMER2_GPIO,
	._fsrc.ext_driver_src.flash_id = MAM_CAMERA_EXT_LED_FLASH_SC628A,
};

static struct msm_gpiomux_config apq8064_cam_2d_configs[] = {
};


static struct regulator *lvs5, *l21, *l28, *l35 ;
#if !defined(CONFIG_MACH_JACTIVE_ATT) && !defined(CONFIG_MACH_JACTIVE_EUR)
static struct regulator *smps_s8;
#endif
struct pm_gpio gpio_off_param = {
	.direction = PM_GPIO_DIR_OUT,
	.output_buffer = PM_GPIO_OUT_BUF_CMOS,
	.output_value = 0,
	.pull = PM_GPIO_PULL_NO,
	.vin_sel = 2,
	.out_strength = PM_GPIO_STRENGTH_HIGH,
	.function = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol = 0,
	.disable_pin = 0,
};

struct pm_gpio gpio_on_param = {
	.direction = PM_GPIO_DIR_OUT,
	.output_buffer = PM_GPIO_OUT_BUF_CMOS,
	.output_value = 1,
	.pull = PM_GPIO_PULL_NO,
	.vin_sel = 2,
	.out_strength = PM_GPIO_STRENGTH_HIGH,
	.function = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol = 0,
	.disable_pin = 0,
};

struct pm_gpio gpio_in_param = {
	.direction = PM_GPIO_DIR_IN,
	.output_value = 0,
	.pull = PM_GPIO_PULL_NO,
	.vin_sel = 2,
	.function = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol = 0,
};

struct pm_gpio gpio_in_off_param = {
	.direction = PM_GPIO_DIR_IN,
	.output_value = 0,
	.pull = PM_GPIO_PULL_DN,
	.out_strength = PM_GPIO_STRENGTH_HIGH,
	.vin_sel = PM_GPIO_VIN_S4,
	.function = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol = 0,
};

static void pmic_gpio_ctrl(int num, int onoff)
{
	int rc = 0;

	if (onoff == 1) {
		rc = pm8xxx_gpio_config(num, &gpio_on_param);
		if (rc) {
			printk(KERN_INFO
				"gpio_config %d failed , rc=%d\n", num, rc);
		}
	} else if (onoff == 0) {
		rc = pm8xxx_gpio_config(num, &gpio_off_param);
		if (rc) {
			printk(KERN_INFO
				"gpio_config %d failed , rc=%d\n", num, rc);
		}
	} else if (onoff == 2) {
		rc = pm8xxx_gpio_config(num, &gpio_in_param);
		if (rc) {
			printk(KERN_INFO
				"gpio_config %d failed , rc=%d\n", num, rc);
		}
	}  else if (onoff == 3) {
		rc = pm8xxx_gpio_config(num, &gpio_in_off_param);
		if (rc) {
			printk(KERN_INFO
				"gpio_config %d failed , rc=%d\n", num, rc);
		}
	}
}

static int get_system_rev(void)
{
	return system_rev;
}
#if !defined(CONFIG_MACH_JACTIVE_ATT) && !defined(CONFIG_MACH_JACTIVE_EUR)

static void cam_ldo_power_on(void)
{
	int ret = 0;
	int cam_type = 0;
	
	printk(KERN_DEBUG "[JC] %s: In\n", __func__);

	printk(KERN_DEBUG "[JC] %s: system_rev=%d\n", __func__, system_rev);

	/* CAM_ISP_CORE_1.2V*/
	if (system_rev < 3) {
		pmic_gpio_ctrl(GPIO_CAM_CORE_EN, 1);
	} else {
		if (!smps_s8) {
			smps_s8 = regulator_get(NULL, "8921_s8");
			if (IS_ERR(smps_s8)) {
				printk(KERN_DEBUG "could not get smps_s8, rc = %ld\n",
				PTR_ERR(smps_s8));
				smps_s8 = NULL;
				return;
			}
			ret = regulator_set_voltage(smps_s8, 1200000, 1200000);
			if (ret) {
				printk(KERN_DEBUG "set_voltage failed"\
					"for smps_s8, rc=%d\n",
					ret);
				return;
			}
		}
		ret = regulator_enable(smps_s8);
		if (ret)
			printk(KERN_DEBUG "error enabling regulator smps_s8\n");
	}
	usleep(1*1000);

	/* CAM_SENSOR_2.8V (CIS 2.8V)*/
	if (system_rev < 6) {
		if (gpio_ref_count == 0)
			pmic_gpio_ctrl(GPIO_CAM_A_EN, 1);

		gpio_ref_count++;
		printk(KERN_DEBUG "[JC] %s: gpio ref cnt %d\n",
			__func__, gpio_ref_count);
	} else {
		pmic_gpio_ctrl(GPIO_CAM_A_EN2, 1);
		printk(KERN_DEBUG "[JC] %s: GPIO_CAM_A_EN2: 1\n",
			__func__);
	}

	cam_type = gpio_get_value(GPIO_CAM_SENSOR_DET);

	printk(KERN_DEBUG "[JC] %s: SENSOR TYPE = %d\n", __func__, cam_type);

	/* CAM_DVDD1.1V_1.2V*/
	l28 = regulator_get(NULL, "8921_l28");

	if (cam_type == 1) {
		printk(KERN_DEBUG "[JC] %s: Sony Sensor 1.1V", __func__);
		regulator_set_voltage(l28, 1100000, 1100000);
	}
	else {
		printk(KERN_DEBUG "[JC] %s: LSI Sensor 1.2V", __func__);
		regulator_set_voltage(l28, 1200000, 1200000);	
	}
	
	ret = regulator_enable(l28);
	if (ret)
		printk(KERN_DEBUG "error enabling regulator 8921_l28\n");

	/* CAM_AF_2.8V */
	pmic_gpio_ctrl(GPIO_CAM_AF_EN, 1);

	usleep(2*1000);

	/* CAM_HOST_1.8V*/
	if (system_rev == 0) {
		lvs5 = regulator_get(NULL, "8921_lvs5");
		ret = regulator_enable(lvs5);
		if (ret)
			printk(KERN_DEBUG "error enabling regulator 8921_lvs5\n");
	} else {
		l35 = regulator_get(NULL, "8917_l35");
#ifdef CONFIG_MACH_JF_DCM
		regulator_set_voltage(l35, 1850000, 1850000);
#else
		regulator_set_voltage(l35, 1800000, 1800000);
#endif
		ret = regulator_enable(l35);
		if (ret)
			printk(KERN_DEBUG "error enabling regulator 8917_l35\n");
	}
	usleep(2*1000);
}

static void cam_ldo_power_off(void)
{
	int ret = 0;

	printk(KERN_DEBUG "[JC] %s: In\n", __func__);

	/* CAM_HOST_1.8V*/
	if (system_rev == 0) {
		if (lvs5) {
			ret = regulator_disable(lvs5);
			if (ret)
				printk(KERN_DEBUG "error disabling regulator 8921_lvs5\n");
		}
	} else {
		if (l35) {
			ret = regulator_disable(l35);
			if (ret)
				printk(KERN_DEBUG "error disabling regulator 8917_l35\n");
		}
	}

	/* CAM_DVDD1.1V_1.2V*/
	if (l28) {
		ret = regulator_disable(l28);
		if (ret)
			printk(KERN_DEBUG "error disabling" \
				" regulator 8921_l28\n");
	}

	if (system_rev < 6) {
		if (gpio_ref_count == 0) {
			/* CAM_SENSOR_2.8V (CIS 2.8V)*/
			printk(KERN_DEBUG "[JC] %s: Turning off GPIO_CAM_A_EN\n",
				__func__);
			pmic_gpio_ctrl(GPIO_CAM_A_EN, 0);
		} else {
			pr_err("[JC] %s: gpio %d, dont turn off GPIO_CAM_A_EN,"\
				"other camera is in use!!",
				__func__, gpio_ref_count);
		}
	} else {
		pmic_gpio_ctrl(GPIO_CAM_A_EN2, 0);
		printk(KERN_DEBUG "[JC] %s: GPIO_CAM_A_EN2: 0\n",
			__func__);
	}

	/* CAM_ISP_CORE_1.2V*/
	if (system_rev < 3) {
		pmic_gpio_ctrl(GPIO_CAM_CORE_EN, 0);
	} else {
		if (smps_s8) {
			ret = regulator_disable((smps_s8));
			if (ret)
				printk(KERN_DEBUG "error disabling regulator smps_s8\n");
		}
	}
}

#else
static void cam_ldo_power_on_sub(void)
{
	int ret = 0;
	printk(KERN_DEBUG "[FORTIUS] %s: Sub On\n", __func__);
	printk(KERN_DEBUG "[FORTIUS] %s: system_rev=%d\n", __func__, system_rev);

	pmic_gpio_ctrl(GPIO_CAM_A_EN2, 1);
	usleep(1*1000);

	l28 = regulator_get(NULL, "8921_l28");
	regulator_set_voltage(l28, 1200000, 1200000);
	ret = regulator_enable(l28);
	if (ret)
		printk(KERN_DEBUG "error enabling regulator 8921_l28 \n");

	usleep(1*1000);
}

static void cam_ldo_power_on(void)
{
	int ret = 0;
	printk(KERN_DEBUG "[FORTIUS] %s: On\n", __func__);
	pmic_gpio_ctrl(GPIO_CAM_AF_EN, 1);
	usleep(2*1000);

	if (system_rev == 0) {
		lvs5 = regulator_get(NULL, "8921_lvs5");
		ret = regulator_enable(lvs5);
		if (ret)
			printk(KERN_DEBUG "error enabling regulator 8921_lvs5\n");
	} else {
		l35 = regulator_get(NULL, "8917_l35");
		regulator_set_voltage(l35, 1800000, 1800000);
		ret = regulator_enable(l35);
		if (ret)
			printk(KERN_DEBUG "error enabling regulator 8917_l35\n");
	}
}

static void cam_ldo_power_off_sub(void)
{
	int ret = 0;
	printk(KERN_DEBUG "[Fortius] %s: Sub Off\n", __func__);

	pmic_gpio_ctrl(GPIO_CAM_AF_EN, 0);
	udelay(1*1000);

	if (l35) {
		ret = regulator_disable(l35);
		if (ret)
			printk(KERN_DEBUG "error disabling regulator 8917_l35\n");
	}
}

static void cam_ldo_power_off(void)
{
	int ret = 0;
	printk(KERN_DEBUG "[Fortius] %s: Off\n", __func__);

	if(l28){ 
		ret = regulator_disable(l28);
		if (ret)
			printk(KERN_DEBUG "error disabling regulator 8921_l28 \n");;
	}
	udelay(1000);

	gpio_set_value_cansleep(GPIO_CAM_A_EN2, 0);
	ret = gpio_get_value(GPIO_CAM_A_EN);
	if (ret)
		printk("check GPIO_CAM_A_EN : %d\n", ret);
}
#endif

static void cam_ldo_af_power_off(void)
{
	/* CAM_AF_2.8V */
	if (system_rev < 6) {
		gpio_ref_count--;
		printk(KERN_DEBUG "[JC] %s: gpio ref cnt %d\n",
			__func__, gpio_ref_count);
	}
	printk(KERN_DEBUG "[JC] %s: Turning off GPIO_CAM_AF_EN\n",
		__func__);
	pmic_gpio_ctrl(GPIO_CAM_AF_EN, 0);

	usleep(10);
}

static void cam_ldo_vddio_power_off(void)
{
	int ret = 0;
	
	/* CAM_HOST_1.8V*/
	if (system_rev == 0) {
		if (lvs5) {
			ret = regulator_disable(lvs5);
			if (ret)
				printk(KERN_DEBUG "error disabling regulator 8921_lvs5\n");
		}
	} else {
		if (l35) {
			ret = regulator_disable(l35);
			if (ret)
				printk(KERN_DEBUG "error disabling regulator 8917_l35\n");
		}
	}
}


static void vt_cam_ldo_power_on(void)
{
	int ret = 0;
	printk(KERN_DEBUG "[S5K6B2YX] %s: In\n", __func__);

#ifdef CONFIG_CAMERA_SW_I2C
	gpio_set_value_cansleep(GPIO_VT_CAM_SDA, 1);
	gpio_set_value_cansleep(GPIO_VT_CAM_SCL, 1);
#endif

	/* CAM_SENSOR_2.8V (CIS 2.8V)*/
	if (system_rev < 5) {
		printk(KERN_DEBUG "[S5K6B2YX] %s: GPIO_CAM_A_EN: 1\n",
			__func__);
		if (gpio_ref_count == 0)
			pmic_gpio_ctrl(GPIO_CAM_A_EN, 1);

		gpio_ref_count++;
		printk(KERN_DEBUG "[S5K6B2YX] %s: gpio ref cnt %d\n",
			__func__, gpio_ref_count);
	} else {
		pmic_gpio_ctrl(GPIO_CAM_VT_EN, 1);
		printk(KERN_DEBUG "[S5K6B2YX] %s: GPIO_CAM_VT_EN: 1\n",
			__func__);
	}

	/* VT_CAM_1P8 */
	l21 = regulator_get(NULL, "8921_l21");
	regulator_set_voltage(l21, 1800000, 1800000);
	ret = regulator_enable(l21);
	if (ret)
		printk(KERN_DEBUG "error enabling regulator 8921_l21\n");
}

static void vt_cam_ldo_power_off(void)
{
	printk(KERN_DEBUG "[S5K6B2YX] %s: In\n", __func__);

	/* VT_CAM_1P8 */
	if (l21) {
		int ret = regulator_disable(l21);
		if (ret)
			printk(KERN_DEBUG "error disabling regulator 8921_l21\n");
	}
	
	usleep(400);

	/* CAM_SENSOR_2.8V (CIS 2.8V)*/
	if (system_rev < 5) {
		printk(KERN_DEBUG "[S5K6B2YX] %s: GPIO_CAM_A_EN: 0\n",
			__func__);
		gpio_ref_count--;
		printk(KERN_DEBUG "[S5K6B2YX] %s: gpio ref cnt %d\n",
			__func__, gpio_ref_count);
		if (gpio_ref_count == 0) {
			printk(KERN_DEBUG "[S5K6B2YX] %s: Turning off GPIO_CAM_A_EN\n",
				__func__);
			pmic_gpio_ctrl(GPIO_CAM_A_EN, 0);
		} else {
			pr_err("[S5K6B2YX] %s: gpio %d, dont turn off"\
				" GPIO_CAM_A_EN, other camera is in use!!",
				 __func__, gpio_ref_count);
		}
	} else {
		pmic_gpio_ctrl(GPIO_CAM_VT_EN, 0);
		printk(KERN_DEBUG "[S5K6B2YX] %s: GPIO_CAM_VT_EN: 0\n",
			__func__);
	}
}

static int jc_config_isp_irq(void)
{
	printk(KERN_ERR "jc_config_isp_irq~~~~~~~~~~\n");
	return 0;
}

static struct msm_bus_vectors cam_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_low_power_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 1451520,
		.ib  = 3870720,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

#if defined(CONFIG_MACH_JACTIVE_ATT)
static struct msm_bus_vectors cam_preview_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 300000000,//27648000,
		.ib  = 2656000000UL,	/*110592000,*/
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 600000000,
		.ib  = 2656000000UL,	/*561807360,*/
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 800000000,//600000000,	/*274423680,*/
		.ib  = 2656000000UL,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
};

static struct msm_bus_vectors cam_zsl_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		/*.ab  = 302071680,*/
		.ab  = 800000000,
		.ib  = 4264000000UL, /*1208286720,*/
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 1350000000,
	},
};
#else
static struct msm_bus_vectors cam_preview_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 27648000,
		.ib  = 110592000,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 274406400,
		.ib  = 561807360,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 274423680,
		.ib  = 1097694720,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
};

static struct msm_bus_vectors cam_zsl_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 302071680,
		.ib  = 1208286720,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
};
#endif

static struct msm_bus_vectors cam_video_ls_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 348192000,
		.ib  = 617103360,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
};

static struct msm_bus_vectors cam_dual_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 348192000,
		.ib  = 4294000000UL,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 1750000000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1750000000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};


static struct msm_bus_paths cam_bus_client_config[] = {
	{
		ARRAY_SIZE(cam_init_vectors),
		cam_init_vectors,
	},
	{
		ARRAY_SIZE(cam_preview_vectors),
		cam_preview_vectors,
	},
	{
		ARRAY_SIZE(cam_video_vectors),
		cam_video_vectors,
	},
	{
		ARRAY_SIZE(cam_snapshot_vectors),
		cam_snapshot_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_video_ls_vectors),
		cam_video_ls_vectors,
	},
	{
		ARRAY_SIZE(cam_dual_vectors),
		cam_dual_vectors,
	},
	{
		ARRAY_SIZE(cam_low_power_vectors),
		cam_low_power_vectors,
	},

};

static struct msm_bus_scale_pdata cam_bus_client_pdata = {
		cam_bus_client_config,
		ARRAY_SIZE(cam_bus_client_config),
		.name = "msm_camera",
};

static struct msm_camera_device_platform_data msm_camera_csi_device_data[] = {
	{
		.csid_core = 0,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
	{
		.csid_core = 1,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
};

static struct camera_vreg_t apq_8064_cam_vreg[] = {
	{"cam_vdig", REG_LDO, 1200000, 1200000, 105000},
	{"cam_vio", REG_VS, 0, 0, 0},
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vaf", REG_LDO, 2800000, 2850000, 300000},
};

#define CAML_RSTN PM8921_GPIO_PM_TO_SYS(28)
#define CAMR_RSTN 34
#define CAMR_STANDBY PM8921_GPIO_PM_TO_SYS(41)

static struct gpio apq8064_common_cam_gpio[] = {
};

static struct gpio apq8064_back_cam_gpio[] = {
	{GPIO_CAM_MCLK, GPIOF_DIR_IN, "CAMIF_MCLK"},
	{GPIO_13M_CAM_RESET, GPIOF_DIR_OUT, "CAM_RESET"},
};

static struct msm_gpio_set_tbl apq8064_back_cam_gpio_set_tbl[] = {
	{GPIO_13M_CAM_RESET, GPIOF_OUT_INIT_LOW, 10000},
	{GPIO_13M_CAM_RESET, GPIOF_OUT_INIT_HIGH, 10000},
};

static struct msm_camera_gpio_conf apq8064_back_cam_gpio_conf = {
	.cam_gpiomux_conf_tbl = apq8064_cam_2d_configs,
	.cam_gpiomux_conf_tbl_size = ARRAY_SIZE(apq8064_cam_2d_configs),
	.cam_gpio_common_tbl = apq8064_common_cam_gpio,
	.cam_gpio_common_tbl_size = ARRAY_SIZE(apq8064_common_cam_gpio),
	.cam_gpio_req_tbl = apq8064_back_cam_gpio,
	.cam_gpio_req_tbl_size = ARRAY_SIZE(apq8064_back_cam_gpio),
	.cam_gpio_set_tbl = apq8064_back_cam_gpio_set_tbl,
	.cam_gpio_set_tbl_size = ARRAY_SIZE(apq8064_back_cam_gpio_set_tbl),
};

static struct gpio apq8064_front_cam_gpio[] = {
	{4, GPIOF_DIR_IN, "CAMIF_MCLK"},
	{12, GPIOF_DIR_IN, "CAMIF_I2C_DATA"},
	{13, GPIOF_DIR_IN, "CAMIF_I2C_CLK"},
	{CAMR_RSTN, GPIOF_DIR_OUT, "CAM_RESET"},
};

static struct gpio apq8064_s5k6b2yx_cam_gpio[] = {
	{GPIO_VT_CAM_MCLK, GPIOF_DIR_IN, "VT_CAMIF_MCLK"},
};

static struct msm_gpio_set_tbl apq8064_front_cam_gpio_set_tbl[] = {
	{CAMR_RSTN, GPIOF_OUT_INIT_LOW, 10000},
	{CAMR_RSTN, GPIOF_OUT_INIT_HIGH, 10000},
};

static struct msm_gpio_set_tbl apq8064_s5k6b2yx_cam_gpio_set_tbl[] = {
	{GPIO_VT_CAM_STBY, GPIOF_OUT_INIT_LOW, 10000},
	{GPIO_VT_CAM_RESET, GPIOF_OUT_INIT_LOW, 10000},
	{GPIO_VT_CAM_RESET, GPIOF_OUT_INIT_HIGH, 10000},
};

static struct msm_camera_gpio_conf apq8064_front_cam_gpio_conf = {
	.cam_gpiomux_conf_tbl = apq8064_cam_2d_configs,
	.cam_gpiomux_conf_tbl_size = ARRAY_SIZE(apq8064_cam_2d_configs),
	.cam_gpio_common_tbl = apq8064_common_cam_gpio,
	.cam_gpio_common_tbl_size = ARRAY_SIZE(apq8064_common_cam_gpio),
	.cam_gpio_req_tbl = apq8064_front_cam_gpio,
	.cam_gpio_req_tbl_size = ARRAY_SIZE(apq8064_front_cam_gpio),
	.cam_gpio_set_tbl = apq8064_front_cam_gpio_set_tbl,
	.cam_gpio_set_tbl_size = ARRAY_SIZE(apq8064_front_cam_gpio_set_tbl),
};

static struct msm_camera_gpio_conf apq8064_s5k6b2yx_cam_gpio_conf = {
	.cam_gpiomux_conf_tbl = apq8064_cam_2d_configs,
	.cam_gpiomux_conf_tbl_size = ARRAY_SIZE(apq8064_cam_2d_configs),
	.cam_gpio_common_tbl = apq8064_common_cam_gpio,
	.cam_gpio_common_tbl_size = ARRAY_SIZE(apq8064_common_cam_gpio),
	.cam_gpio_req_tbl = apq8064_s5k6b2yx_cam_gpio,
	.cam_gpio_req_tbl_size = ARRAY_SIZE(apq8064_s5k6b2yx_cam_gpio),
	.cam_gpio_set_tbl = apq8064_s5k6b2yx_cam_gpio_set_tbl,
	.cam_gpio_set_tbl_size = ARRAY_SIZE(apq8064_s5k6b2yx_cam_gpio_set_tbl),
};

#ifdef CONFIG_CAMERA_USE_GSBI7
static struct msm_camera_i2c_conf apq8064_back_cam_i2c_conf = {
	.use_i2c_mux = 0,
	.mux_dev = &apq8064_device_qup_i2c_gsbi7,
	.i2c_mux_mode = MODE_R,
};
#else
static struct msm_camera_i2c_conf apq8064_back_cam_i2c_conf = {
	.use_i2c_mux = 1,
	.mux_dev = &msm8960_device_i2c_mux_gsbi4,
	.i2c_mux_mode = MODE_R,
};
#endif

static struct i2c_board_info msm_act_main_cam_i2c_info = {
	I2C_BOARD_INFO("msm_actuator", 0x11),
};

static struct msm_actuator_info msm_act_main_cam_0_info = {
	.board_info     = &msm_act_main_cam_i2c_info,
	.cam_name   = MSM_ACTUATOR_MAIN_CAM_0,
#ifdef CONFIG_CAMERA_USE_GSBI7
	.bus_id         = APQ_8064_GSBI7_QUP_I2C_BUS_ID,
#else
	.bus_id         = APQ_8064_GSBI4_QUP_I2C_BUS_ID,
#endif
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};

static struct i2c_board_info msm_act_main_cam1_i2c_info = {
	I2C_BOARD_INFO("msm_actuator", 0x18),
};

static struct msm_actuator_info msm_act_main_cam_1_info = {
	.board_info     = &msm_act_main_cam1_i2c_info,
	.cam_name       = MSM_ACTUATOR_MAIN_CAM_1,
#ifdef CONFIG_CAMERA_USE_GSBI7
	.bus_id			= APQ_8064_GSBI7_QUP_I2C_BUS_ID,
#else
	.bus_id			= APQ_8064_GSBI4_QUP_I2C_BUS_ID,
#endif
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};

#if defined(CONFIG_MACH_JACTIVE_ATT)
static struct i2c_board_info msm_act_main_cam2_i2c_info = {
	I2C_BOARD_INFO("msm_actuator", 0x50),
};
static struct msm_actuator_info msm_act_main_cam_2_info = {
	.board_info     = &msm_act_main_cam2_i2c_info,
	.cam_name       = MSM_ACTUATOR_MAIN_CAM_2,
	.bus_id			= MSM_CAMERA_SW_I2C_BUS_ID, // 27
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};
#endif
static struct msm_camera_i2c_conf apq8064_front_cam_i2c_conf = {
	.use_i2c_mux = 0,
};

static struct msm_camera_sensor_flash_data flash_imx074 = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
	.flash_src	= &msm_flash_src
};

static struct msm_camera_csi_lane_params imx074_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct msm_camera_sensor_platform_info sensor_board_info_imx074 = {
	.mount_angle	= 90,
	.cam_vreg = apq_8064_cam_vreg,
	.num_vreg = ARRAY_SIZE(apq_8064_cam_vreg),
	.gpio_conf = &apq8064_back_cam_gpio_conf,
	.i2c_conf = &apq8064_back_cam_i2c_conf,
	.csi_lane_params = &imx074_csi_lane_params,
};

static struct i2c_board_info imx074_eeprom_i2c_info = {
	I2C_BOARD_INFO("imx074_eeprom", 0x34 << 1),
};

static struct msm_eeprom_info imx074_eeprom_info = {
	.board_info     = &imx074_eeprom_i2c_info,
	.bus_id         = APQ_8064_GSBI4_QUP_I2C_BUS_ID,
};

static struct msm_camera_sensor_info msm_camera_sensor_imx074_data = {
	.sensor_name	= "imx074",
	.pdata	= &msm_camera_csi_device_data[0],
	.flash_data	= &flash_imx074,
	.sensor_platform_info = &sensor_board_info_imx074,
	.csi_if	= 1,
	.camera_type = BACK_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
	.actuator_info = &msm_act_main_cam_0_info,
	.eeprom_info = &imx074_eeprom_info,
};

static struct msm_camera_csi_lane_params imx091_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct msm_camera_sensor_flash_data flash_imx091 = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_sensor_platform_info sensor_board_info_imx091 = {
	.mount_angle	= 0,
	.cam_vreg = apq_8064_cam_vreg,
	.num_vreg = ARRAY_SIZE(apq_8064_cam_vreg),
	.gpio_conf = &apq8064_back_cam_gpio_conf,
	.i2c_conf = &apq8064_back_cam_i2c_conf,
	.csi_lane_params = &imx091_csi_lane_params,
};

static struct i2c_board_info imx091_eeprom_i2c_info = {
	I2C_BOARD_INFO("imx091_eeprom", 0x21),
};

static struct msm_eeprom_info imx091_eeprom_info = {
	.board_info     = &imx091_eeprom_i2c_info,
	.bus_id         = APQ_8064_GSBI4_QUP_I2C_BUS_ID,
};

static struct msm_camera_sensor_info msm_camera_sensor_imx091_data = {
	.sensor_name	= "imx091",
	.pdata	= &msm_camera_csi_device_data[0],
	.flash_data	= &flash_imx091,
	.sensor_platform_info = &sensor_board_info_imx091,
	.csi_if	= 1,
	.camera_type = BACK_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
	.actuator_info = &msm_act_main_cam_1_info,
	.eeprom_info = &imx091_eeprom_info,
};

static struct msm_camera_sensor_flash_data flash_s5k3l1yx = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_csi_lane_params s5k3l1yx_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct msm_camera_sensor_platform_info sensor_board_info_s5k3l1yx = {
	.mount_angle	= 90,
	.cam_vreg = apq_8064_cam_vreg,
	.num_vreg = ARRAY_SIZE(apq_8064_cam_vreg),
	.gpio_conf = &apq8064_back_cam_gpio_conf,
	.i2c_conf = &apq8064_back_cam_i2c_conf,
	.csi_lane_params = &s5k3l1yx_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3l1yx_data = {
	.sensor_name	= "s5k3l1yx",
	.pdata	= &msm_camera_csi_device_data[0],
	.flash_data	= &flash_s5k3l1yx,
	.sensor_platform_info = &sensor_board_info_s5k3l1yx,
	.csi_if	= 1,
	.camera_type = BACK_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
};
static struct camera_vreg_t msm_8064_s5k3h5xa_vreg[] = {
	{"cam_vdig", REG_LDO, 1200000, 1200000, 105000},
	{"cam_vio", REG_VS, 0, 0, 0},
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vaf", REG_LDO, 2800000, 2800000, 300000},
};
#if defined(CONFIG_MACH_JACTIVE_ATT)
static struct msm_eeprom_info imx175_eeprom_info = {
        .type = MSM_EEPROM_SPI,
};
#endif
static struct msm_camera_csi_lane_params s5k3h5xa_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

#if defined(CONFIG_MACH_JACTIVE_ATT)
static int pmic_set_func(uint8_t pmic_gpio, uint8_t onoff)
{
	pmic_gpio_ctrl(PM8921_GPIO_PM_TO_SYS(pmic_gpio), onoff);
		
	return 0;
}

static struct msm_camera_sensor_flash_src msm_flash_src_s5k3h5xa = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC_GPIO,
	._fsrc.pmic_gpio_src.led_src_1 = 33, /* emitting until coming LOW signal */
	._fsrc.pmic_gpio_src.led_src_2 = 24, /* flash for a short time */
	._fsrc.pmic_gpio_src.pmic_set_func = pmic_set_func
};
static struct msm_camera_sensor_flash_data flash_s5k3h5xa = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
	.flash_src	= &msm_flash_src_s5k3h5xa
};

#else
static struct msm_camera_sensor_flash_data flash_s5k3h5xa = {
	.flash_type = MSM_CAMERA_FLASH_NONE
};
#endif

static struct msm_camera_sensor_platform_info sensor_board_info_s5k3h5xa = {
	.mount_angle = 90,
	.cam_vreg = msm_8064_s5k3h5xa_vreg,
	.num_vreg = ARRAY_SIZE(msm_8064_s5k3h5xa_vreg),
	.gpio_conf = &apq8064_back_cam_gpio_conf,
	.i2c_conf = &apq8064_back_cam_i2c_conf,
	.csi_lane_params = &s5k3h5xa_csi_lane_params,
	.sensor_power_on = cam_ldo_power_on,
	.sensor_power_off = cam_ldo_power_off,
#if defined(CONFIG_MACH_JACTIVE_ATT) || defined(CONFIG_MACH_JACTIVE_EUR)
	.sensor_power_on_sub = cam_ldo_power_on_sub,	
	.sensor_power_off_sub = cam_ldo_power_off_sub,	
#endif	
	.sensor_pmic_gpio_ctrl = pmic_gpio_ctrl,
	.reset = GPIO_13M_CAM_RESET,
};
static struct msm_camera_sensor_info msm_camera_sensor_s5k3h5xa_data = {
	.sensor_name = "s5k3h5xa",
	.pdata = &msm_camera_csi_device_data[0],
	.flash_data = &flash_s5k3h5xa,
	.sensor_platform_info = &sensor_board_info_s5k3h5xa,
	.csi_if = 1,
	.camera_type = BACK_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
#if defined(CONFIG_MACH_JACTIVE_ATT)	
	.actuator_info = &msm_act_main_cam_2_info,
	.eeprom_info = &imx175_eeprom_info,
#endif	
};

static struct msm_camera_sensor_flash_data flash_mt9m114 = {
	.flash_type = MSM_CAMERA_FLASH_NONE
};

static struct msm_camera_csi_lane_params mt9m114_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x1,
};

static struct msm_camera_sensor_platform_info sensor_board_info_mt9m114 = {
	.mount_angle = 90,
	.cam_vreg = apq_8064_cam_vreg,
	.num_vreg = ARRAY_SIZE(apq_8064_cam_vreg),
	.gpio_conf = &apq8064_front_cam_gpio_conf,
	.i2c_conf = &apq8064_front_cam_i2c_conf,
	.csi_lane_params = &mt9m114_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9m114_data = {
	.sensor_name = "mt9m114",
	.pdata = &msm_camera_csi_device_data[1],
	.flash_data = &flash_mt9m114,
	.sensor_platform_info = &sensor_board_info_mt9m114,
	.csi_if = 1,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = YUV_SENSOR,
};

static struct msm_camera_sensor_flash_data flash_ov2720 = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_csi_lane_params ov2720_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x3,
};

static struct msm_camera_sensor_platform_info sensor_board_info_ov2720 = {
	.mount_angle	= 0,
	.cam_vreg = apq_8064_cam_vreg,
	.num_vreg = ARRAY_SIZE(apq_8064_cam_vreg),
	.gpio_conf = &apq8064_front_cam_gpio_conf,
	.i2c_conf = &apq8064_front_cam_i2c_conf,
	.csi_lane_params = &ov2720_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_ov2720_data = {
	.sensor_name	= "ov2720",
	.pdata	= &msm_camera_csi_device_data[1],
	.flash_data	= &flash_ov2720,
	.sensor_platform_info = &sensor_board_info_ov2720,
	.csi_if	= 1,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
};


static struct msm_camera_sensor_flash_data flash_s5k6b2yx = {
	.flash_type     = MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_csi_lane_params s5k6b2yx_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x1,
};

static struct msm_camera_sensor_platform_info sensor_board_info_s5k6b2yx = {
	.mount_angle = 270,
	.cam_vreg = apq_8064_cam_vreg,
	.num_vreg = ARRAY_SIZE(apq_8064_cam_vreg),
	.gpio_conf = &apq8064_s5k6b2yx_cam_gpio_conf,
	.i2c_conf = &apq8064_front_cam_i2c_conf,
	.csi_lane_params = &s5k6b2yx_csi_lane_params,
	.sensor_power_on = vt_cam_ldo_power_on,
	.sensor_power_off = vt_cam_ldo_power_off,
	.sensor_pmic_gpio_ctrl = pmic_gpio_ctrl,
	.reset = GPIO_VT_CAM_RESET,
	.stby = GPIO_VT_CAM_STBY,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k6b2yx_data = {
	.sensor_name    = "s5k6b2yx",
	.pdata  = &msm_camera_csi_device_data[1],
	.flash_data     = &flash_s5k6b2yx,
	.sensor_platform_info = &sensor_board_info_s5k6b2yx,
	.csi_if = 1,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
};

static struct platform_device msm_camera_server = {
	.name = "msm_cam_server",
	.id = 0,
};

static struct camera_vreg_t apq_8064_jc_vreg[] = {
	{"cam_vio", REG_VS, 0, 0, 0},
	{"cam_vdig", REG_LDO, 1200000, 1200000, 105000},
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vaf", REG_LDO, 2800000, 2850000, 300000},
};

static struct msm_camera_sensor_flash_data flash_jc = {
	.flash_type = MSM_CAMERA_FLASH_LED
};

static struct msm_camera_csi_lane_params jc_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct msm_camera_sensor_platform_info sensor_board_info_jc = {
	.mount_angle = 90,
	.cam_vreg = apq_8064_jc_vreg,
	.num_vreg = ARRAY_SIZE(apq_8064_jc_vreg),
	.gpio_conf = &apq8064_back_cam_gpio_conf,
	.i2c_conf = &apq8064_back_cam_i2c_conf,
	.csi_lane_params = &jc_csi_lane_params,
	.sensor_power_on = cam_ldo_power_on,
	.sensor_power_off = cam_ldo_power_off,
	.sensor_af_power_off = cam_ldo_af_power_off,
	.sensor_vddio_power_off = cam_ldo_vddio_power_off,
	.sensor_pmic_gpio_ctrl = pmic_gpio_ctrl,
	.config_isp_irq = jc_config_isp_irq,
	.irq = PM8921_GPIO_IRQ(PM8921_IRQ_BASE, 24),
	.irq_gpio = GPIO_CAM_ISP_INT,
	.reset = GPIO_13M_CAM_RESET,
	.sys_rev = get_system_rev,
};

static struct msm_camera_sensor_info msm_camera_sensor_jc_data = {
	.sensor_name = "jc",
	.pdata = &msm_camera_csi_device_data[0],
	.flash_data = &flash_jc,
	.sensor_platform_info = &sensor_board_info_jc,
	.csi_if = 1,
	.camera_type = BACK_CAMERA_2D,
	.sensor_type = YUV_SENSOR,
};

static struct spi_board_info jc_spi_info[] __initdata = {
	{
		.modalias		= "jc_spi",
		.mode			= SPI_MODE_0 | SPI_CS_HIGH,
		.bus_num		= 0,
		.chip_select	= 0,
		.max_speed_hz	= 11000000,//10000000,
	}
};

struct pm_gpio cam_init_in_cfg = {
	.direction = PM_GPIO_DIR_IN,
	.pull = PM_GPIO_PULL_DN,
	.out_strength = PM_GPIO_STRENGTH_HIGH,
	.vin_sel = PM_GPIO_VIN_S4,
	.function = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol = 0,
	.output_value = 0,
};

	struct pm_gpio cam_init_out_cfg = {
	.direction = PM_GPIO_DIR_OUT,
	.pull = PM_GPIO_PULL_NO,
	.out_strength = PM_GPIO_STRENGTH_HIGH,
	.vin_sel = PM_GPIO_VIN_S4,
	.function = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol = 0,
	.output_value = 0,
};

struct pm_gpio cam_rear_det = {
		.direction		= PM_GPIO_DIR_IN,
		.pull			= PM_GPIO_PULL_NO,
		.out_strength		= PM_GPIO_STRENGTH_LOW,
		.function		= PM_GPIO_FUNC_NORMAL,
		.inv_int_pol		= 0,
		.vin_sel		= PM_GPIO_VIN_S4,
		.output_buffer		= PM_GPIO_OUT_BUF_CMOS,
		.output_value		= 0,
};

void __init apq8064_init_cam(void)
{
	printk(KERN_DEBUG "[JC] %s: In\n", __func__);

	msm_gpiomux_install(apq8064_cam_common_configs,
			ARRAY_SIZE(apq8064_cam_common_configs));

	/*config gpio*/
	/*pm gpio config for sleep state*/
	pm8xxx_gpio_config(GPIO_VT_CAM_ID, &cam_init_in_cfg);
	pm8xxx_gpio_config(GPIO_13M_CAM_RESET, &cam_init_out_cfg);
	pm8xxx_gpio_config(GPIO_CAM_AF_EN, &cam_init_out_cfg);
	pm8xxx_gpio_config(GPIO_VT_CAM_STBY, &cam_init_out_cfg);
#if defined(CONFIG_MACH_JACTIVE_ATT)
#else
	pm8xxx_gpio_config(GPIO_CAM_ISP_INT, &cam_init_in_cfg);
#endif

	pm8xxx_gpio_config(GPIO_CAM_A_EN2, &cam_init_out_cfg);
	pm8xxx_gpio_config(GPIO_CAM_SENSOR_DET, &cam_rear_det);

	/* temp: need to set low because bootloader make high signal. */
	pmic_gpio_ctrl(GPIO_CAM_VT_EN, 0);
	printk(KERN_DEBUG "[S5K6B2YX] %s: GPIO_CAM_VT_EN: 0\n", __func__);

	if (machine_is_apq8064_cdp()) {
		sensor_board_info_imx074.mount_angle = 0;
		sensor_board_info_mt9m114.mount_angle = 0;
	} else if (machine_is_apq8064_liquid())
		sensor_board_info_imx074.mount_angle = 180;
	spi_register_board_info(jc_spi_info,
		ARRAY_SIZE(jc_spi_info));

	platform_device_register(&msm_camera_server);
#if !defined (CONFIG_CAMERA_USE_GSBI7)
	platform_device_register(&msm8960_device_i2c_mux_gsbi4);
#endif
	platform_device_register(&msm8960_device_csiphy0);
	platform_device_register(&msm8960_device_csiphy1);
	platform_device_register(&msm8960_device_csid0);
	platform_device_register(&msm8960_device_csid1);
	platform_device_register(&msm8960_device_ispif);
	platform_device_register(&msm8960_device_vfe);
	platform_device_register(&msm8960_device_vpe);
}

#ifdef CONFIG_I2C
static struct i2c_board_info apq8064_camera_i2c_boardinfo[] = {
	{
	I2C_BOARD_INFO("imx074", 0x10),
	.platform_data = &msm_camera_sensor_imx074_data,
	},
	{
	I2C_BOARD_INFO("mt9m114", 0x48),
	.platform_data = &msm_camera_sensor_mt9m114_data,
	},
	{
	I2C_BOARD_INFO("ov2720", 0x6C),
	.platform_data = &msm_camera_sensor_ov2720_data,
	},
	{
	I2C_BOARD_INFO("sc628a", 0x6E),
	},
	{
	I2C_BOARD_INFO("imx091", 0x34),
	.platform_data = &msm_camera_sensor_imx091_data,
	},
	{
	I2C_BOARD_INFO("s5k3l1yx", 0x34),
	.platform_data = &msm_camera_sensor_s5k3l1yx_data,
	},
	{
	I2C_BOARD_INFO("jc", 0x3E),
	.platform_data = &msm_camera_sensor_jc_data,
	},
	{
	I2C_BOARD_INFO("s5k3h5xa", 0x2D),
	.platform_data = &msm_camera_sensor_s5k3h5xa_data,
	},
};

static struct i2c_board_info apq8064_front_camera_i2c_boardinfo[] = {
	{
	I2C_BOARD_INFO("s5k6b2yx", 0x6A),
	.platform_data = &msm_camera_sensor_s5k6b2yx_data,
	},
};


struct msm_camera_board_info apq8064_camera_board_info = {
	.board_info = apq8064_camera_i2c_boardinfo,
	.num_i2c_board_info = ARRAY_SIZE(apq8064_camera_i2c_boardinfo),
};

struct msm_camera_board_info apq8064_front_camera_board_info = {
	.board_info = apq8064_front_camera_i2c_boardinfo,
	.num_i2c_board_info = ARRAY_SIZE(apq8064_front_camera_i2c_boardinfo),
};
#endif
#endif
