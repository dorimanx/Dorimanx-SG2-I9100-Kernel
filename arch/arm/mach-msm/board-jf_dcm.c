/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/smb349.h>
#include <linux/i2c/sx150x.h>
#include <linux/i2c-gpio.h>
#include <linux/slimbus/slimbus.h>
#include <linux/mfd/wcd9xxx/core.h>
#include <linux/mfd/wcd9xxx/pdata.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/msm_ssbi.h>
#include <linux/spi/spi.h>
#include <linux/dma-contiguous.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/qcom_crypto_device.h>
#include <linux/msm_ion.h>
#include <linux/memory.h>
#include <linux/memblock.h>
#include <linux/msm_thermal.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/cyttsp-qc.h>
#ifdef CONFIG_HAPTIC_ISA1200
#include <linux/i2c/isa1200.h>
#endif
#include <linux/gpio_keys.h>
#include <linux/epm_adc.h>
#include <linux/i2c/sx150x.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach/mmc.h>
#include <linux/platform_data/qcom_wcnss_device.h>
#include <linux/gpio.h>
#ifdef CONFIG_VIBETONZ
#include <linux/vibrator.h>
#endif

#include <linux/power_supply.h>
#ifdef CONFIG_SND_SOC_ES325
#include <linux/i2c/esxxx.h>
#endif
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/ion.h>
#include <linux/usb/msm_hsusb.h>
#include <linux/usb/android.h>
#include <mach/socinfo.h>
#include <mach/msm_spi.h>
#include "timer.h"
#include "devices.h"
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/rpm.h>
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif
#include <mach/msm_memtypes.h>
#include <linux/bootmem.h>
#include <asm/setup.h>
#include <mach/dma.h>
#include <mach/msm_dsps.h>
#include <mach/msm_bus_board.h>
#include <mach/cpuidle.h>
#include <mach/mdm2.h>
#include <linux/msm_tsens.h>
#include <mach/msm_xo.h>
#include <mach/msm_rtb.h>
#include <sound/cs8427.h>
#include <media/gpio-ir-recv.h>
#include <linux/fmem.h>
#include <mach/msm_pcie.h>
#include <mach/restart.h>
#include <mach/msm_iomap.h>
#include <linux/sec_jack.h>
#include "clock.h"

#include <mach/apq8064-gpio.h>

#ifdef CONFIG_KEYBOARD_CYPRESS_TOUCH_236
#include <linux/i2c/cypress_touchkey.h>
#endif

#include <linux/persistent_ram.h>
#include <linux/platform_data/ram_console.h>

#include "msm_watchdog.h"
#include "board-8064.h"
#include "spm.h"
#include <mach/mpm.h>
#include "rpm_resources.h"
#include "pm.h"
#include "pm-boot.h"
#include "devices-msm8x60.h"
#include "smd_private.h"

#ifdef CONFIG_SEC_DEBUG
#include <mach/sec_debug.h>
#endif
#if defined(CONFIG_VIDEO_MHL_V2)
#include <linux/sii8240.h>
#endif
#ifdef CONFIG_SENSORS_SSP
#include <linux/ssp_platformdata.h>
#endif
#ifdef CONFIG_BCM2079X_NFC_I2C
#include <linux/nfc/bcm2079x.h>
#endif
#ifdef CONFIG_SENSORS_SSP_C12SD
#include <linux/sensor/guva_c12sd.h>
#endif
#ifdef CONFIG_MFD_MAX77693
#include <linux/mfd/max77693.h>
#include <linux/mfd/max77693-private.h>
#endif

#ifdef CONFIG_SEC_FPGA
#include <linux/barcode_emul.h>
#endif

#ifdef CONFIG_LEDS_MAX77693
#include <linux/leds-max77693.h>
#endif

#ifdef CONFIG_PROC_AVC
#include <linux/proc_avc.h>
#endif

#ifdef CONFIG_SEC_THERMISTOR
#include <mach/sec_thermistor.h>
#include <mach/fusion3-thermistor.h>
#endif

#if defined(CONFIG_TMM_CHG_CTRL)
#include <mach/isdbtmm_pdata.h>

#define TMM_CHG_TEST_LOG 0
#if TMM_CHG_TEST_LOG
#define tmm_chg_log printk
#else
#define tmm_chg_log(A, ...) ;
#endif

#endif

#if defined(CONFIG_SENSORS_SSP)
enum {
	SNS_PWR_OFF,
	SNS_PWR_ON,
	SNS_PWR_KEEP
};
#endif

#if defined(CONFIG_SENSORS_SSP)
static void sensor_power_on_vdd(int, int);
#endif

#define MSM_PMEM_ADSP_SIZE         0x7800000
#define MSM_PMEM_AUDIO_SIZE        0x4CF000
#ifdef CONFIG_FB_MSM_HDMI_AS_PRIMARY
#define MSM_PMEM_SIZE 0x4000000 /* 64 Mbytes */
#else
#define MSM_PMEM_SIZE 0x4000000 /* 64 Mbytes */
#endif

#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
#define HOLE_SIZE		0x20000
#define MSM_CONTIG_MEM_SIZE  0x65000
#ifdef CONFIG_MSM_IOMMU
#define MSM_ION_MM_SIZE		0x6600000    /* 56MB(0x3800000) -> 98MB -> 102MB */
#define MSM_ION_SF_SIZE		0
#define MSM_ION_QSECOM_SIZE	0x1700000    /* 7.5MB(0x780000) -> 23MB */
#define MSM_ION_HEAP_NUM	7
#else
#define MSM_ION_MM_SIZE		MSM_PMEM_ADSP_SIZE
#define MSM_ION_SF_SIZE		MSM_PMEM_SIZE
#define MSM_ION_QSECOM_SIZE	0x600000 /* (6MB) */
#define MSM_ION_HEAP_NUM	8
#endif
#define MSM_ION_MM_FW_SIZE	(0x200000 - HOLE_SIZE) /* (2MB - 128KB) */
#define MSM_ION_MFC_SIZE	SZ_8K
#define MSM_ION_AUDIO_SIZE	MSM_PMEM_AUDIO_SIZE
#else
#define MSM_CONTIG_MEM_SIZE  0x110C000
#define MSM_ION_HEAP_NUM	1
#endif

#define APQ8064_FIXED_AREA_START (0xa0000000 - (MSM_ION_MM_FW_SIZE + \
							HOLE_SIZE))
#define MAX_FIXED_AREA_SIZE	0x10000000
#define MSM_MM_FW_SIZE		(0x200000 - HOLE_SIZE)
#define APQ8064_FW_START	APQ8064_FIXED_AREA_START

#define QFPROM_RAW_FEAT_CONFIG_ROW0_MSB     (MSM_QFPROM_BASE + 0x23c)
#define QFPROM_RAW_OEM_CONFIG_ROW0_LSB      (MSM_QFPROM_BASE + 0x220)

/* PCIE AXI address space */
#define PCIE_AXI_BAR_PHYS   0x08000000
#define PCIE_AXI_BAR_SIZE   SZ_128M

/* PCIe pmic gpios */
#define PCIE_WAKE_N_PMIC_GPIO 12
#define PCIE_PWR_EN_PMIC_GPIO 13
#define PCIE_RST_N_PMIC_MPP 1

static int sec_tsp_synaptics_mode;
static int lcd_tsp_panel_version;

static int atoi(char *str)
{
	int ret = 0;
	int count = 0;

	if (str == NULL)
		return -EINVAL;

	while (str[count] != (int)NULL && str[count] >= '0'
			&& str[count] <= 'z') {
		ret = ret * 0x10 + str[count] - '0';
		++count;
	}
	return ret;
}

static int __init sec_tsp_mode(char *mode)
{
	int ret, ret1;

	ret = atoi(mode);
	ret1 = ((ret >> 12) & 0X0F);

	lcd_tsp_panel_version = ret;

	printk(KERN_ERR "%s: LCD_ID = 0x%s, val: 0X%x, ret1: 0x%x",
			__func__, mode, ret, ret1);

	if (ret1 == 0x02 || ret1 == 0x03 || ret1 == 0x00)
		sec_tsp_synaptics_mode = 0;
	else
		sec_tsp_synaptics_mode = 1;

	if (ret == 0)
		sec_tsp_synaptics_mode = 1;

	pr_info("%s : %s", __func__, sec_tsp_synaptics_mode ?
				"synaptics" : "atmel");

	return 1;
}
__setup("lcd_id=0x", sec_tsp_mode);


#if defined(CONFIG_VIBETONZ) && defined(CONFIG_MOTOR_DRV_MAX77693)
static struct max77693_haptic_platform_data max77693_haptic_pdata = {
	.max_timeout = 10000,
	.duty = 44000,
	.period = 44642,
	.reg2 = MOTOR_LRA | EXT_PWM | DIVIDER_256,
	.init_hw = NULL,
	.motor_en = NULL,
	.pwm_id = 1,
	.regulator_name = NULL,
};

static void max77693_haptic_power_onoff(int onoff)
{
	int ret;
	static struct regulator *reg_l8;

	if (!reg_l8) {
		reg_l8 = regulator_get(NULL, "8921_l8");
		ret = regulator_set_voltage(reg_l8, 1800000, 3000000);

		if (IS_ERR(reg_l8)) {
			printk(KERN_ERR"could not get 8921_l8, rc = %ld\n",
				PTR_ERR(reg_l8));
			return;
		}
	}

	if (onoff) {
		ret = regulator_enable(reg_l8);
		if (ret) {
			printk(KERN_ERR"enable l8 failed, rc=%d\n", ret);
			return;
		}
		printk(KERN_DEBUG"haptic power_on is finished.\n");
	} else {
		if (regulator_is_enabled(reg_l8)) {
			ret = regulator_disable(reg_l8);
			if (ret) {
				printk(KERN_ERR"disable l8 failed, rc=%d\n",
									ret);
				return;
			}
		}
		printk(KERN_DEBUG"haptic power_off is finished.\n");
	}
}
#endif

#ifdef CONFIG_LEDS_MAX77693
static struct max77693_led_platform_data max77693_led_pdata = {
	.num_leds = 2,

	.leds[0].name = "leds-sec1",
	.leds[0].id = MAX77693_FLASH_LED_1,
	.leds[0].timer = MAX77693_FLASH_TIME_187P5MS,
	.leds[0].timer_mode = MAX77693_TIMER_MODE_MAX_TIMER,
	.leds[0].cntrl_mode = MAX77693_LED_CTRL_BY_FLASHSTB,
	.leds[0].brightness = 0x3D,

	.leds[1].name = "torch-sec1",
	.leds[1].id = MAX77693_TORCH_LED_1,
	.leds[1].cntrl_mode = MAX77693_LED_CTRL_BY_FLASHSTB,
	.leds[1].brightness = 0x06,
};
#endif

#ifdef CONFIG_MFD_MAX77693
struct max77693_platform_data msm8960_max77693_info = {
	.irq_base       = IF_PMIC_IRQ_BASE,
	.irq_gpio       = GPIO_IF_PMIC_IRQ,
	.wakeup         = 1,
	.wc_irq_gpio	= 0,
	.muic           = &max77693_muic,
#if defined(CONFIG_REGULATOR_MAX77693)
	.regulators     = max77693_regulators,
	.num_regulators = MAX77693_REG_MAX,
#endif
#if defined(CONFIG_VIBETONZ) && defined(CONFIG_MOTOR_DRV_MAX77693)
	.haptic_data = &max77693_haptic_pdata,
#endif
#ifdef CONFIG_LEDS_MAX77693
	.led_data = &max77693_led_pdata,
#endif
#ifdef CONFIG_CHARGER_MAX77693
	.charger_data = &sec_battery_pdata,
#endif
};

static struct i2c_gpio_platform_data max77693_i2c_gpio_data = {
	.sda_pin                = GPIO_IF_PMIC_SDA,
	.scl_pin                = GPIO_IF_PMIC_SCL,
	.udelay                 = 2,
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

static struct platform_device max77693_i2c_gpio_device = {
	.name                   = "i2c-gpio",
	.id                     = MSM_FSA9485_I2C_BUS_ID,
	.dev.platform_data      = &max77693_i2c_gpio_data,
};

static struct i2c_board_info max77693_i2c_board_info[] = {

	{
		I2C_BOARD_INFO("max77693",  (0xCC >> 1)),
		.platform_data  = &msm8960_max77693_info,
		.irq    = MSM_GPIO_TO_INT(GPIO_IF_PMIC_IRQ),
	},

};
#endif
#if defined(CONFIG_IR_REMOCON_FPGA)
static void irda_wake_en(bool onoff)
{
	gpio_direction_output(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_IRDA_WAKE),
				onoff);
	printk(KERN_ERR "%s: irda_wake_en :%d\n", __func__, onoff);
}

static void irda_device_init(void)
{
	struct pm_gpio irda_irq = {
		.direction		= PM_GPIO_DIR_IN,
		.pull			= PM_GPIO_PULL_UP_31P5,
		.out_strength		= PM_GPIO_STRENGTH_HIGH,
		.function		= PM_GPIO_FUNC_NORMAL,
		.inv_int_pol		= 0,
		.vin_sel		= PM_GPIO_VIN_S4,
		.output_buffer		= PM_GPIO_OUT_BUF_CMOS,
		.output_value		= 0,
	};
	printk(KERN_ERR "%s called!\n", __func__);
	gpio_request(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_IRDA_WAKE), "irda_wake");
	gpio_direction_output(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_IRDA_WAKE), 0);
	pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(	\
			PMIC_GPIO_IRDA_IRQ), &irda_irq);
	gpio_request(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_IRDA_IRQ), "irda_irq");
	gpio_direction_input(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_IRDA_IRQ));
}

static void irda_vdd_onoff(bool onoff)
{
	int ret = 0;
	static struct regulator *irda_vreg_1p8;

	if (!irda_vreg_1p8) {
		irda_vreg_1p8 = regulator_get(NULL, "8917_l36");
		if (IS_ERR(irda_vreg_1p8)) {
			pr_err("could not get 8921_l36, rc = %ld\n",
				PTR_ERR(irda_vreg_1p8));
			return;
		}
	}

	if (onoff) {
		ret = regulator_enable(irda_vreg_1p8);
		if (ret) {
			pr_err("enable l36 failed, rc=%d\n", ret);
			return;
		}
		pr_info("%s irda_vreg 1.8V on is finished.\n", __func__);
	} else {
		if (regulator_is_enabled(irda_vreg_1p8))
			ret = regulator_disable(irda_vreg_1p8);

		if (ret) {
			pr_err("disable l36 failed, rc=%d\n", ret);
			return;
		}
		pr_info("%s irda_vreg 1.8V off is finished.\n", __func__);
	}
}
#endif

#ifdef CONFIG_KERNEL_MSM_CONTIG_MEM_REGION
static unsigned msm_contig_mem_size = MSM_CONTIG_MEM_SIZE;
static int __init msm_contig_mem_size_setup(char *p)
{
	msm_contig_mem_size = memparse(p, NULL);
	return 0;
}
early_param("msm_contig_mem_size", msm_contig_mem_size_setup);
#endif

#ifdef CONFIG_ANDROID_PMEM
static unsigned pmem_size = MSM_PMEM_SIZE;
static int __init pmem_size_setup(char *p)
{
	pmem_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_size", pmem_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;

static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;

static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);
#endif

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device apq8064_android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {.platform_data = &android_pmem_pdata},
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device apq8064_android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device apq8064_android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};
#endif /* CONFIG_MSM_MULTIMEDIA_USE_ION */
#endif /* CONFIG_ANDROID_PMEM */

struct fmem_platform_data apq8064_fmem_pdata = {
};

static struct memtype_reserve apq8064_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init reserve_rtb_memory(void)
{
#if defined(CONFIG_MSM_RTB)
	apq8064_reserve_table[MEMTYPE_EBI1].size += apq8064_rtb_pdata.size;
#endif
}


static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_size;
	android_pmem_audio_pdata.size = MSM_PMEM_AUDIO_SIZE;
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/
}

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	apq8064_reserve_table[p->memory_type].size += p->size;
}
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
	apq8064_reserve_table[MEMTYPE_EBI1].size += msm_contig_mem_size;
#endif /*CONFIG_ANDROID_PMEM*/
}

static int apq8064_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

#define FMEM_ENABLED 0

#ifdef CONFIG_ION_MSM
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct ion_cp_heap_pdata cp_mm_apq8064_ion_pdata = {
	.permission_type = IPT_TYPE_MM_CARVEOUT,
	.align = PAGE_SIZE,
	.reusable = FMEM_ENABLED,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_MIDDLE,
	.is_cma = 0,
	.no_nonsecure_alloc = 1,
};

static struct ion_cp_heap_pdata cp_mfc_apq8064_ion_pdata = {
	.permission_type = IPT_TYPE_MFC_SHAREDMEM,
	.align = PAGE_SIZE,
	.reusable = 0,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_HIGH,
	.no_nonsecure_alloc = 1,
};

static struct ion_co_heap_pdata co_apq8064_ion_pdata = {
	.adjacent_mem_id = INVALID_HEAP_ID,
	.align = PAGE_SIZE,
	.mem_is_fmem = 0,
};

static struct ion_co_heap_pdata fw_co_apq8064_ion_pdata = {
	.adjacent_mem_id = ION_CP_MM_HEAP_ID,
	.align = SZ_128K,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_LOW,
};
#endif

static u64 msm_dmamask = DMA_BIT_MASK(32);

static struct platform_device ion_mm_heap_device = {
	.name = "ion-mm-heap-device",
	.id = -1,
	.dev = {
		.dma_mask = &msm_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};

/**
 * These heaps are listed in the order they will be allocated. Due to
 * video hardware restrictions and content protection the FW heap has to
 * be allocated adjacent (below) the MM heap and the MFC heap has to be
 * allocated after the MM heap to ensure MFC heap is not more than 256MB
 * away from the base address of the FW heap.
 * However, the order of FW heap and MM heap doesn't matter since these
 * two heaps are taken care of by separate code to ensure they are adjacent
 * to each other.
 * Don't swap the order unless you know what you are doing!
 */
struct ion_platform_heap apq8064_heaps[] = {
		{
			.id	= ION_SYSTEM_HEAP_ID,
			.type	= ION_HEAP_TYPE_SYSTEM,
			.name	= ION_VMALLOC_HEAP_NAME,
		},
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		{
			.id	= ION_CP_MM_HEAP_ID,
			.type	= ION_HEAP_TYPE_CP,
			.name	= ION_MM_HEAP_NAME,
			.size	= MSM_ION_MM_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &cp_mm_apq8064_ion_pdata,
			.priv	= &ion_mm_heap_device.dev
		},
		{
			.id	= ION_MM_FIRMWARE_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_MM_FIRMWARE_HEAP_NAME,
			.size	= MSM_ION_MM_FW_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &fw_co_apq8064_ion_pdata,
		},
		{
			.id	= ION_CP_MFC_HEAP_ID,
			.type	= ION_HEAP_TYPE_CP,
			.name	= ION_MFC_HEAP_NAME,
			.size	= MSM_ION_MFC_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &cp_mfc_apq8064_ion_pdata,
		},
#ifndef CONFIG_MSM_IOMMU
		{
			.id	= ION_SF_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_SF_HEAP_NAME,
			.size	= MSM_ION_SF_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_apq8064_ion_pdata,
		},
#endif
		{
			.id	= ION_IOMMU_HEAP_ID,
			.type	= ION_HEAP_TYPE_IOMMU,
			.name	= ION_IOMMU_HEAP_NAME,
		},
		{
			.id	= ION_QSECOM_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_QSECOM_HEAP_NAME,
			.size	= MSM_ION_QSECOM_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_apq8064_ion_pdata,
		},
		{
			.id	= ION_AUDIO_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_AUDIO_HEAP_NAME,
			.size	= MSM_ION_AUDIO_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_apq8064_ion_pdata,
		},
#endif
};

static struct ion_platform_data apq8064_ion_pdata = {
	.nr = MSM_ION_HEAP_NUM,
	.heaps = apq8064_heaps,
};

static struct platform_device apq8064_ion_dev = {
	.name = "ion-msm",
	.id = 1,
	.dev = { .platform_data = &apq8064_ion_pdata },
};
#endif

static struct platform_device apq8064_fmem_device = {
	.name = "fmem",
	.id = 1,
	.dev = { .platform_data = &apq8064_fmem_pdata },
};

static void __init reserve_mem_for_ion(enum ion_memory_types mem_type,
				      unsigned long size)
{
	apq8064_reserve_table[mem_type].size += size;
}

static void __init apq8064_reserve_fixed_area(unsigned long fixed_area_size)
{
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	int ret;

	if (fixed_area_size > MAX_FIXED_AREA_SIZE)
		panic("fixed area size is larger than %dM\n",
			MAX_FIXED_AREA_SIZE >> 20);

	reserve_info->fixed_area_size = fixed_area_size;
	reserve_info->fixed_area_start = APQ8064_FW_START;

	ret = memblock_remove(reserve_info->fixed_area_start,
		reserve_info->fixed_area_size);
	BUG_ON(ret);
#endif
}

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

static void samsung_sys_class_init(void)
{
       pr_info("samsung sys class init.\n");

       sec_class = class_create(THIS_MODULE, "sec");

       if (IS_ERR(sec_class)) {
	       pr_err("Failed to create class(sec)!\n");
	       return;
       }

       pr_info("samsung sys class end.\n");
};



/**
 * Reserve memory for ION and calculate amount of reusable memory for fmem.
 * We only reserve memory for heaps that are not reusable. However, we only
 * support one reusable heap at the moment so we ignore the reusable flag for
 * other than the first heap with reusable flag set. Also handle special case
 * for video heaps (MM,FW, and MFC). Video requires heaps MM and MFC to be
 * at a higher address than FW in addition to not more than 256MB away from the
 * base address of the firmware. This means that if MM is reusable the other
 * two heaps must be allocated in the same region as FW. This is handled by the
 * mem_is_fmem flag in the platform data. In addition the MM heap must be
 * adjacent to the FW heap for content protection purposes.
 */
static void __init reserve_ion_memory(void)
{
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	unsigned int i;
	unsigned int ret;
	unsigned int fixed_size = 0;
	unsigned int fixed_low_size, fixed_middle_size, fixed_high_size;
	unsigned long fixed_low_start, fixed_middle_start, fixed_high_start;
	unsigned long cma_alignment;
	unsigned int low_use_cma = 0;
	unsigned int middle_use_cma = 0;
	unsigned int high_use_cma = 0;


	fixed_low_size = 0;
	fixed_middle_size = 0;
	fixed_high_size = 0;

	cma_alignment = PAGE_SIZE << max(MAX_ORDER, pageblock_order);

	for (i = 0; i < apq8064_ion_pdata.nr; ++i) {
		struct ion_platform_heap *heap =
			&(apq8064_ion_pdata.heaps[i]);
		int use_cma = 0;


		if (heap->extra_data) {
			int fixed_position = NOT_FIXED;

			switch ((int)heap->type) {
			case ION_HEAP_TYPE_CP:
				if (((struct ion_cp_heap_pdata *)
					heap->extra_data)->is_cma) {
					heap->size = ALIGN(heap->size,
						cma_alignment);
					use_cma = 1;
				}
				fixed_position = ((struct ion_cp_heap_pdata *)
					heap->extra_data)->fixed_position;
				break;
			case ION_HEAP_TYPE_DMA:
				use_cma = 1;
				/* Purposely fall through here */
			case ION_HEAP_TYPE_CARVEOUT:
				fixed_position = ((struct ion_co_heap_pdata *)
					heap->extra_data)->fixed_position;
				break;
			default:
				break;
			}

			if (fixed_position != NOT_FIXED)
				fixed_size += heap->size;
			else
				reserve_mem_for_ion(MEMTYPE_EBI1, heap->size);

			if (fixed_position == FIXED_LOW) {
				fixed_low_size += heap->size;
				low_use_cma = use_cma;
			} else if (fixed_position == FIXED_MIDDLE) {
				fixed_middle_size += heap->size;
				middle_use_cma = use_cma;
			} else if (fixed_position == FIXED_HIGH) {
				fixed_high_size += heap->size;
				high_use_cma = use_cma;
			} else if (use_cma) {
				/*
				 * Heaps that use CMA but are not part of the
				 * fixed set. Create wherever.
				 */
				dma_declare_contiguous(
					heap->priv,
					heap->size,
					0,
					0xb0000000);

			}
		}
	}

	if (!fixed_size)
		return;

	/*
	 * Given the setup for the fixed area, we can't round up all sizes.
	 * Some sizes must be set up exactly and aligned correctly. Incorrect
	 * alignments are considered a configuration issue
	 */

	fixed_low_start = APQ8064_FIXED_AREA_START;
	if (low_use_cma) {
		BUG_ON(!IS_ALIGNED(fixed_low_size + HOLE_SIZE, cma_alignment));
		BUG_ON(!IS_ALIGNED(fixed_low_start, cma_alignment));
	} else {
		BUG_ON(!IS_ALIGNED(fixed_low_size + HOLE_SIZE, SECTION_SIZE));
		ret = memblock_remove(fixed_low_start,
				      fixed_low_size + HOLE_SIZE);
		BUG_ON(ret);
	}

	fixed_middle_start = fixed_low_start + fixed_low_size + HOLE_SIZE;
	if (middle_use_cma) {
		BUG_ON(!IS_ALIGNED(fixed_middle_start, cma_alignment));
		BUG_ON(!IS_ALIGNED(fixed_middle_size, cma_alignment));
	} else {
		BUG_ON(!IS_ALIGNED(fixed_middle_size, SECTION_SIZE));
		ret = memblock_remove(fixed_middle_start, fixed_middle_size);
		BUG_ON(ret);
	}

	fixed_high_start = fixed_middle_start + fixed_middle_size;
	if (high_use_cma) {
		fixed_high_size = ALIGN(fixed_high_size, cma_alignment);
		BUG_ON(!IS_ALIGNED(fixed_high_start, cma_alignment));
	} else {
		/* This is the end of the fixed area so it's okay to round up */
		fixed_high_size = ALIGN(fixed_high_size, SECTION_SIZE);
		ret = memblock_remove(fixed_high_start, fixed_high_size);
		BUG_ON(ret);
	}

	for (i = 0; i < apq8064_ion_pdata.nr; ++i) {
		struct ion_platform_heap *heap = &(apq8064_ion_pdata.heaps[i]);

		if (heap->extra_data) {
			int fixed_position = NOT_FIXED;
			struct ion_cp_heap_pdata *pdata = NULL;

			switch ((int) heap->type) {
			case ION_HEAP_TYPE_CP:
				pdata =
				(struct ion_cp_heap_pdata *)heap->extra_data;
				fixed_position = pdata->fixed_position;
				break;
			case ION_HEAP_TYPE_CARVEOUT:
			case ION_HEAP_TYPE_DMA:
				fixed_position = ((struct ion_co_heap_pdata *)
					heap->extra_data)->fixed_position;
				break;
			default:
				break;
			}

			switch (fixed_position) {
			case FIXED_LOW:
				heap->base = fixed_low_start;
				break;
			case FIXED_MIDDLE:
				heap->base = fixed_middle_start;
				if (middle_use_cma) {
					ret = dma_declare_contiguous(
						heap->priv,
						heap->size,
						fixed_middle_start,
						0xa0000000);
					WARN_ON(ret);
				}
				pdata->secure_base = fixed_middle_start
								- HOLE_SIZE;
				pdata->secure_size = HOLE_SIZE + heap->size;
				break;
			case FIXED_HIGH:
				heap->base = fixed_high_start;
				break;
			default:
				break;
			}
		}
	}
#endif
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct ram_console_platform_data ram_console_pdata = {
	.bootinfo = NULL,
};

static struct platform_device ram_console_device = {
       .name = "ram_console",
       .id = -1,
       .dev    = {
               .platform_data = &ram_console_pdata,
       },
};
#endif

#ifdef CONFIG_ANDROID_PERSISTENT_RAM
#define RAMCONSOLE_PHYS_ADDR 0x8FD00000
static struct persistent_ram_descriptor per_ram_descs[] __initdata = {
       {
               .name = "ram_console",
               .size = SZ_1M,
       }
};

static struct persistent_ram per_ram __initdata = {
       .descs = per_ram_descs,
       .num_descs = ARRAY_SIZE(per_ram_descs),
       .start = RAMCONSOLE_PHYS_ADDR,
       .size = SZ_1M
};
#endif

static void __init reserve_mdp_memory(void)
{
	apq8064_mdp_writeback(apq8064_reserve_table);
}

static void __init reserve_cache_dump_memory(void)
{
#ifdef CONFIG_MSM_CACHE_DUMP
	unsigned int total;

	total = apq8064_cache_dump_pdata.l1_size +
		apq8064_cache_dump_pdata.l2_size;
	apq8064_reserve_table[MEMTYPE_EBI1].size += total;
#endif
}

static void __init reserve_mpdcvs_memory(void)
{
	apq8064_reserve_table[MEMTYPE_EBI1].size += SZ_32K;
}

static void __init apq8064_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
	reserve_ion_memory();
	reserve_mdp_memory();
	reserve_rtb_memory();
	reserve_cache_dump_memory();
	reserve_mpdcvs_memory();
}

static struct reserve_info apq8064_reserve_info __initdata = {
	.memtype_reserve_table = apq8064_reserve_table,
	.calculate_reserve_sizes = apq8064_calculate_reserve_sizes,
	.reserve_fixed_area = apq8064_reserve_fixed_area,
	.paddr_to_memtype = apq8064_paddr_to_memtype,
};

static int apq8064_memory_bank_size(void)
{
	return 1<<29;
}

static void __init locate_unstable_memory(void)
{
	struct membank *mb = &meminfo.bank[meminfo.nr_banks - 1];
	unsigned long bank_size;
	unsigned long low, high;

	bank_size = apq8064_memory_bank_size();
	low = meminfo.bank[0].start;
	high = mb->start + mb->size;

	/* Check if 32 bit overflow occured */
	if (high < mb->start)
		high = -PAGE_SIZE;

	low &= ~(bank_size - 1);

	if (high - low <= bank_size)
		goto no_dmm;

#ifdef CONFIG_ENABLE_DMM
	apq8064_reserve_info.low_unstable_address = mb->start -
					MIN_MEMORY_BLOCK_SIZE + mb->size;
	apq8064_reserve_info.max_unstable_size = MIN_MEMORY_BLOCK_SIZE;

	apq8064_reserve_info.bank_size = bank_size;
	pr_info("low unstable address %lx max size %lx bank size %lx\n",
		apq8064_reserve_info.low_unstable_address,
		apq8064_reserve_info.max_unstable_size,
		apq8064_reserve_info.bank_size);
	return;
#endif
no_dmm:
	apq8064_reserve_info.low_unstable_address = high;
	apq8064_reserve_info.max_unstable_size = 0;
}

#ifdef CONFIG_KEYBOARD_CYPRESS_TOUCH_236

static struct touchkey_callbacks *tk_charger_callbacks;

void touchkey_charger_infom(bool en)
{
	if (tk_charger_callbacks && tk_charger_callbacks->inform_charger)
		tk_charger_callbacks->inform_charger(tk_charger_callbacks, en);
}

static void touchkey_register_callback(void *cb)
{
	tk_charger_callbacks = cb;
}

#define GPIO_TOUCH_KEY_INT		PM8921_GPIO_PM_TO_SYS(31)

struct pm_gpio tkey_int = {
	.direction	= PM_GPIO_DIR_IN,
	.pull		= PM_GPIO_PULL_NO,
	.vin_sel	= 2,
	.function	= PM_GPIO_FUNC_NORMAL,
	.inv_int_pol	= 0,
};

static void cypress_power_onoff(int onoff)
{
	int ret;
	static struct regulator *reg_l23;
	static struct regulator *reg_l11;


	if (!reg_l23) {
		reg_l23 = regulator_get(NULL, "8921_l23");
		ret = regulator_set_voltage(reg_l23, 1800000, 1800000);

		if (IS_ERR(reg_l23)) {
			printk(KERN_ERR"could not get 8921_l23, rc = %ld\n",
				PTR_ERR(reg_l23));
			return;
		}
	}

	if (!reg_l11) {
		reg_l11 = regulator_get(NULL, "8921_l11");
		ret = regulator_set_voltage(reg_l11, 3300000, 3300000);

		if (IS_ERR(reg_l11)) {
			printk(KERN_ERR"could not get 8921_l11, rc = %ld\n",
				PTR_ERR(reg_l11));
			return;
		}
	}

	if (onoff) {
		ret = regulator_enable(reg_l23);
		if (ret) {
			printk(KERN_ERR"enable l23 failed, rc=%d\n", ret);
			return;
		}
		ret = regulator_enable(reg_l11);
		if (ret) {
			printk(KERN_ERR"enable l11 failed, rc=%d\n", ret);
			return;
		}
		printk(KERN_DEBUG"cypress_power_on is finished.\n");
	} else {
		ret = regulator_disable(reg_l23);
		if (ret) {
			printk(KERN_ERR"disable l23 failed, rc=%d\n", ret);
			return;
		}

		ret = regulator_disable(reg_l11);
		if (ret) {
			printk(KERN_ERR"disable l11 failed, rc=%d\n", ret);
			return;
		}
		printk(KERN_DEBUG"cypress_power_off is finished.\n");
	}
}

static u8 touchkey_keycode[] = {KEY_MENU, KEY_BACK};

static struct cypress_touchkey_platform_data cypress_touchkey_pdata = {
	.gpio_int = GPIO_TOUCH_KEY_INT,
	.touchkey_keycode = touchkey_keycode,
	.power_onoff = cypress_power_onoff,
	.register_cb = touchkey_register_callback,
};

static struct i2c_board_info touchkey_i2c_devices_info[] __initdata = {
	{
		I2C_BOARD_INFO("cypress_touchkey", 0x20),
		.platform_data = &cypress_touchkey_pdata,
		.irq = PM8921_GPIO_IRQ(PM8921_IRQ_BASE, 31),
	},
};


static struct i2c_gpio_platform_data  cypress_touchkey_i2c_gpio_data = {
	.sda_pin		= GPIO_TOUCHKEY_SDA,
	.scl_pin		= GPIO_TOUCHKEY_SCL,
	.udelay			= 0,
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};
static struct platform_device touchkey_i2c_gpio_device = {
	.name			= "i2c-gpio",
	.id			= MSM_TOUCHKEY_I2C_BUS_ID,
	.dev.platform_data	= &cypress_touchkey_i2c_gpio_data,
};

static struct i2c_gpio_platform_data  cypress_touchkey_i2c_gpio_data_2 = {
	.sda_pin		= GPIO_TOUCHKEY_SDA,	
	.scl_pin		= GPIO_TOUCHKEY_SCL_2,
	.udelay			= 0,
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};
static struct platform_device touchkey_i2c_gpio_device_2 = {
	.name			= "i2c-gpio",
	.id			= MSM_TOUCHKEY_I2C_BUS_ID,
	.dev.platform_data	= &cypress_touchkey_i2c_gpio_data_2,
};

#endif

static int apq8064_change_memory_power(u64 start, u64 size,
	int change_type)
{
	return soc_change_memory_power(start, size, change_type);
}

static char prim_panel_name[PANEL_NAME_MAX_LEN];
static char ext_panel_name[PANEL_NAME_MAX_LEN];

static int ext_resolution;

static int __init prim_display_setup(char *param)
{
	if (strnlen(param, PANEL_NAME_MAX_LEN))
		strlcpy(prim_panel_name, param, PANEL_NAME_MAX_LEN);
	return 0;
}
early_param("prim_display", prim_display_setup);

static int __init ext_display_setup(char *param)
{
	if (strnlen(param, PANEL_NAME_MAX_LEN))
		strlcpy(ext_panel_name, param, PANEL_NAME_MAX_LEN);
	return 0;
}
early_param("ext_display", ext_display_setup);

static int __init hdmi_resulution_setup(char *param)
{
	int ret;
	ret = kstrtoint(param, 10, &ext_resolution);
	return ret;
}
early_param("ext_resolution", hdmi_resulution_setup);

static void __init apq8064_reserve(void)
{
	apq8064_set_display_params(prim_panel_name, ext_panel_name,
		ext_resolution);
	msm_reserve();
#ifdef CONFIG_ANDROID_PERSISTENT_RAM
	persistent_ram_early_init(&per_ram);
#endif
}

static void __init place_movable_zone(void)
{
#ifdef CONFIG_ENABLE_DMM
	movable_reserved_start = apq8064_reserve_info.low_unstable_address;
	movable_reserved_size = apq8064_reserve_info.max_unstable_size;
	pr_info("movable zone start %lx size %lx\n",
		movable_reserved_start, movable_reserved_size);
#endif
}

static void __init apq8064_early_reserve(void)
{
	reserve_info = &apq8064_reserve_info;
	locate_unstable_memory();
	place_movable_zone();

}
#ifdef CONFIG_USB_EHCI_MSM_HSIC
/* Bandwidth requests (zero) if no vote placed */
static struct msm_bus_vectors hsic_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ab = 0,
		.ib = 0,
	},
};

/* Bus bandwidth requests in Bytes/sec */
static struct msm_bus_vectors hsic_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ab = 0,
		.ib = 256000000, /*vote for 32Mhz dfab clk rate*/
	},
};

static struct msm_bus_paths hsic_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(hsic_init_vectors),
		hsic_init_vectors,
	},
	{
		ARRAY_SIZE(hsic_max_vectors),
		hsic_max_vectors,
	},
};

static struct msm_bus_scale_pdata hsic_bus_scale_pdata = {
	hsic_bus_scale_usecases,
	ARRAY_SIZE(hsic_bus_scale_usecases),
	.name = "hsic",
};

static struct msm_hsic_host_platform_data msm_hsic_pdata = {
	.strobe			= 88,
	.data			= 89,
	.bus_scale_table	= &hsic_bus_scale_pdata,
};
#else
static struct msm_hsic_host_platform_data msm_hsic_pdata;
#endif
#if defined(CONFIG_VIDEO_MHL_V2)
static void muic77693_mhl_cb(bool otg_enable, int charger)
{
	union power_supply_propval value;
	int i, ret = 0;
	struct power_supply *psy;
	int current_cable_type = POWER_SUPPLY_TYPE_MISC;
	int sub_type = ONLINE_SUB_TYPE_MHL;
	int power_type = ONLINE_POWER_TYPE_UNKNOWN;
	int muic_cable_type = max77693_muic_get_charging_type();

	pr_info("%s: muic cable_type = %d, charger: %d\n",
			__func__, muic_cable_type, charger);
	switch (muic_cable_type) {
		case CABLE_TYPE_SMARTDOCK_MUIC:
		case CABLE_TYPE_SMARTDOCK_TA_MUIC:
		case CABLE_TYPE_SMARTDOCK_USB_MUIC:
			return;
		default:
			 break;
	}
	if (charger == 0x00) {
		pr_info("TA charger\n");
		power_type = ONLINE_POWER_TYPE_MHL_500;
	} else if (charger == 0x01) {
		pr_info("TA powered charger\n");
		power_type = ONLINE_POWER_TYPE_MHL_900;
	} else if (charger == 0x02) {
		pr_info("TA powered charger\n");
		power_type = ONLINE_POWER_TYPE_MHL_1500;
	} else if (charger == 0x03) {
		pr_info("USB charger\n");
		power_type = ONLINE_POWER_TYPE_USB;
	} else
		current_cable_type = POWER_SUPPLY_TYPE_BATTERY;

	if (otg_enable) {
		psy_do_property("sec-charger", get,
				POWER_SUPPLY_PROP_ONLINE, value);
		pr_info("sec-charger : %d\n", value.intval);
		if (value.intval == POWER_SUPPLY_TYPE_BATTERY
				|| value.intval == POWER_SUPPLY_TYPE_WIRELESS) {
			if (!poweroff_charging) {
				otg_control(true);
				power_type = ONLINE_POWER_TYPE_UNKNOWN;
				current_cable_type = POWER_SUPPLY_TYPE_BATTERY;
			}
		}
	} else
		otg_control(false);

	for (i = 0; i < 10; i++) {
		psy = power_supply_get_by_name("battery");
		if (psy)
			break;
	}
	if (i == 10) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return;
	}

	value.intval = current_cable_type<<ONLINE_TYPE_MAIN_SHIFT
               | sub_type<<ONLINE_TYPE_SUB_SHIFT
               | power_type<<ONLINE_TYPE_PWR_SHIFT;
	ret = psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	if (ret) {
		pr_err("%s: fail to set power_suppy ONLINE property(%d)\n",
			__func__, ret);
	}
}

struct pm_gpio pmic_mhl_en_parm = {
	.direction = PM_GPIO_DIR_OUT,
	.output_value = 0,
	.pull = PM_GPIO_PULL_NO,
	.vin_sel = 2,
	.out_strength = PM_GPIO_STRENGTH_HIGH,
	.function = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol = 0,
	.disable_pin = 0,
};
struct pm_gpio mhl_int = {
	.direction      = PM_GPIO_DIR_IN,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel                = 2,
	.function               = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol    = 0,
};
struct pm_gpio pmic_mhl_int_parm = {
	.direction = PM_GPIO_DIR_OUT,
	.pull = PM_GPIO_PULL_DN,
	.vin_sel = 2,
	.function = PM_GPIO_FUNC_NORMAL,
};
static void msm8960_mhl_gpio_init(void)
{
	int ret;

	if (system_rev >= 6) {
		ice_gpiox_set(FPGA_GPIO_MHL_RST, 0);
		ice_gpiox_set(FPGA_VSIL_A_1P2_EN, 0);
	} else if (system_rev < 6) {
		ret = gpio_request(GPIO_MHL_RST, "mhl_rst");
		if (ret < 0) {
			pr_err("mhl_rst gpio_request is failed\n");
			return;
		}
	}
	if (system_rev >= 4 && system_rev < 6) {
		ret = gpio_request(GPIO_MHL_VSIL, "mhl_vsil");
		if (ret < 0) {
			pr_err("mhl_vsil gpio_request is failed\n");
			return;
		}
	}
	gpio_tlmm_config(GPIO_CFG(GPIO_MHL_INT, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 1);
}

static void mhl_gpio_config(int data)
{
	int ret;
	if (system_rev < 6)
		gpio_tlmm_config(GPIO_CFG(GPIO_MHL_RST, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 1);
	if (data) {
		ret = gpio_request(GPIO_MHL_INT, "mhl_int");
		if (ret < 0) {
			pr_err("mhl_vsil gpio_request is failed\n");
			return;
		}
		gpio_tlmm_config(GPIO_CFG(GPIO_MHL_INT, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 1);
	} else
		gpio_free(GPIO_MHL_INT);
}
static int get_mhl_int_irq(void)
{
	return  MSM_GPIO_TO_INT(GPIO_MHL_INT);
}

static struct regulator *mhl_l12;
static struct regulator *mhl_l31;
static struct regulator *mhl_l32;
static int mhl_hw_onoff;

static void sii8240_hw_onoff(bool onoff)
{
	int rc = 0;
	/*VPH_PWR : mhl_power_source
	VMHL_3.3V, VSIL_A_1.2V, VMHL_1.8V
	just power control with HDMI_EN pin or control Regulator12*/
	pr_info("%s: onoff =%d\n", __func__, onoff);
	if (mhl_hw_onoff == onoff) {
		pr_info("mhl_hw_onoff already %d\n", onoff);
		return;
	}
	mhl_hw_onoff = onoff;

	if (onoff) {
		if (system_rev >= 4 && system_rev < 6)
			pm8xxx_gpio_config(GPIO_MHL_VSIL, &pmic_mhl_en_parm);
		if (system_rev < 4) {
			if (mhl_l12 == NULL) {
				mhl_l12 = regulator_get(NULL, "8921_l12");
				if (IS_ERR(mhl_l12))
					return ;
				rc = regulator_set_voltage(mhl_l12,
							1200000, 1200000);
				if (rc)
					pr_err("error: setting for mhl_l12\n");
			}
			rc = regulator_enable(mhl_l12);
			if (rc)
				pr_err("error enabling regulator\n");
		}
		if (mhl_l31 == NULL) {
			mhl_l31 = regulator_get(NULL, "8917_l31");
			if (IS_ERR(mhl_l31))
				return ;
		}
		rc = regulator_enable(mhl_l31);
		if (rc)
			pr_err("error enabling regulator\n");
		if (mhl_l32 == NULL) {
			mhl_l32 = regulator_get(NULL, "8917_l32");
			if (IS_ERR(mhl_l32))
				return ;
			rc = regulator_set_voltage(mhl_l32,
				3300000, 3300000);
		}
		rc = regulator_enable(mhl_l32);
		if (rc)
			pr_err("error enabling regulator\n");

		usleep(1*1000);
		if (system_rev >= 6)
			ice_gpiox_set(FPGA_VSIL_A_1P2_EN, 1);
		if (system_rev >= 4 && system_rev < 6)
			gpio_direction_output(PM8921_GPIO_PM_TO_SYS(32), 1);

	} else {
		if (system_rev >= 6)
			ice_gpiox_set(FPGA_VSIL_A_1P2_EN, 0);
		if (system_rev >= 4 && system_rev < 6)
			gpio_direction_output(PM8921_GPIO_PM_TO_SYS(32), 0);
		if (system_rev < 4) {
			if (mhl_l12) {
				rc = regulator_disable(mhl_l12);
				if (rc)
					pr_err("error disabling regulator\n");
			}
		}
		if (mhl_l31) {
			rc =  regulator_disable(mhl_l31);
			if (rc)
				pr_err("error: disable mhl_l31\n");
		}
		if (mhl_l32) {
			rc = regulator_disable(mhl_l32);
			if (rc)
				pr_err("error disable mhl_l32\n");
		}

		usleep_range(10000, 20000);

		if (system_rev >= 6)
			ice_gpiox_set(FPGA_GPIO_MHL_RST, 0);
		if (system_rev < 6)
			gpio_direction_output(GPIO_MHL_RST, 0);
		if (system_rev >= 4 && system_rev < 6)
			pm8xxx_gpio_config(GPIO_MHL_VSIL, &pmic_mhl_en_parm);
	}

	return;
}

static void sii8240_hw_reset(void)
{
	usleep_range(10000, 20000);
	if (system_rev >= 6) {
		if (ice_gpiox_set(FPGA_GPIO_MHL_RST, 1))
			pr_info("%s error in making GPIO_MHL_RST HIGH\n",
			__func__);
		usleep_range(5000, 20000);
		if (ice_gpiox_set(FPGA_GPIO_MHL_RST, 0))
			pr_err("%s error in making GPIO_MHL_RST Low\n",
			__func__);
		usleep_range(10000, 20000);
		if (ice_gpiox_set(FPGA_GPIO_MHL_RST, 1))
			pr_err("%s error in making GPIO_MHL_RST HIGH\n",
			__func__);
	}
	if (system_rev < 6) {
		if (gpio_direction_output(GPIO_MHL_RST, 1))
			pr_err("%s error in making GPIO_MHL_RST HIGH\n",
			__func__);

		usleep_range(5000, 20000);
		if (gpio_direction_output(GPIO_MHL_RST, 0))
			pr_err("%s error in making GPIO_MHL_RST Low\n",
			__func__);

		usleep_range(10000, 20000);
		if (gpio_direction_output(GPIO_MHL_RST, 1))
			pr_err("%s error in making GPIO_MHL_RST HIGH\n",
			__func__);
	}
	msleep(30);
}
struct sii8240_platform_data sii8240_pdata = {
	.get_irq = get_mhl_int_irq,
	.power = sii8240_hw_onoff,
	.hw_reset = sii8240_hw_reset,
	.gpio_cfg = mhl_gpio_config,
	.swing_level = 0x36,
	.vbus_present = muic77693_mhl_cb,
};

static struct i2c_board_info mhl_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("sii8240_tmds", 0x72>>1),
		.platform_data = &sii8240_pdata,
	},
	{
		I2C_BOARD_INFO("sii8240_hdmi", 0x92>>1),
		.platform_data = &sii8240_pdata,
	},
	{
		I2C_BOARD_INFO("sii8240_disc", 0x9A>>1),
		.platform_data = &sii8240_pdata,
	},
	{
		I2C_BOARD_INFO("sii8240_tpi", 0xB2>>1),
		.platform_data = &sii8240_pdata,
	},
	{
		I2C_BOARD_INFO("sii8240_cbus", 0xC8>>1),
		.platform_data = &sii8240_pdata,
	},
};
#endif

#if defined(CONFIG_VIDEO_MHL_V2) || defined(CONFIG_LEDS_AN30259A)
static struct i2c_gpio_platform_data shared_i2c_gpio_data = {
	.sda_pin = GPIO_SHARED_I2C_SDA,
	.scl_pin = GPIO_SHARED_I2C_SCL,
	.udelay = 4,
};

struct platform_device shared_i2c_gpio_device = {
	.name = "i2c-gpio",
	.id = MSM_MHL_I2C_BUS_ID,
	.dev.platform_data = &shared_i2c_gpio_data,
};
#endif

#define PID_MAGIC_ID		0x71432909
#define SERIAL_NUM_MAGIC_ID	0x61945374
#define SERIAL_NUMBER_LENGTH	127
#define DLOAD_USB_BASE_ADD	0x2A03F0C8

struct magic_num_struct {
	uint32_t pid;
	uint32_t serial_num;
};

struct dload_struct {
	uint32_t	reserved1;
	uint32_t	reserved2;
	uint32_t	reserved3;
	uint16_t	reserved4;
	uint16_t	pid;
	char		serial_number[SERIAL_NUMBER_LENGTH];
	uint16_t	reserved5;
	struct magic_num_struct magic_struct;
};

static int usb_diag_update_pid_and_serial_num(uint32_t pid, const char *snum)
{
	struct dload_struct __iomem *dload = 0;

	dload = ioremap(DLOAD_USB_BASE_ADD, sizeof(*dload));
	if (!dload) {
		pr_err("%s: cannot remap I/O memory region: %08x\n",
					__func__, DLOAD_USB_BASE_ADD);
		return -ENXIO;
	}

	pr_debug("%s: dload:%p pid:%x serial_num:%s\n",
				__func__, dload, pid, snum);
	/* update pid */
	dload->magic_struct.pid = PID_MAGIC_ID;
	dload->pid = pid;

	/* update serial number */
	dload->magic_struct.serial_num = 0;
	if (!snum) {
		memset(dload->serial_number, 0, SERIAL_NUMBER_LENGTH);
		goto out;
	}

	dload->magic_struct.serial_num = SERIAL_NUM_MAGIC_ID;
	strlcpy(dload->serial_number, snum, SERIAL_NUMBER_LENGTH);
out:
	iounmap(dload);
	return 0;
}

static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
	.cdrom = false,
	.nluns = 0,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

/* Bandwidth requests (zero) if no vote placed */
static struct msm_bus_vectors usb_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

/* Bus bandwidth requests in Bytes/sec */
static struct msm_bus_vectors usb_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 60000000,		/* At least 480Mbps on bus. */
		.ib = 960000000,	/* MAX bursts rate */
	},
};

static struct msm_bus_paths usb_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(usb_init_vectors),
		usb_init_vectors,
	},
	{
		ARRAY_SIZE(usb_max_vectors),
		usb_max_vectors,
	},
};

static struct msm_bus_scale_pdata usb_bus_scale_pdata = {
	usb_bus_scale_usecases,
	ARRAY_SIZE(usb_bus_scale_usecases),
	.name = "usb",
};

static bool vbus_is_on;
#ifdef CONFIG_CHARGER_BQ24157
static int msm_hsusb_vbus_power_bq24157(bool on)
{
	struct power_supply *psy = power_supply_get_by_name("battery");
	union power_supply_propval value;
	int ret = 0;

	pr_info("%s, attached %d, vbus_is_on %d\n", __func__, on, vbus_is_on);

	/* If VBUS is already on (or off), do nothing. */
	if (vbus_is_on == on)
		return -EBUSY;

	if (on)
		value.intval = POWER_SUPPLY_TYPE_OTG;
	else
		value.intval = POWER_SUPPLY_TYPE_BATTERY;

	if (psy && psy->set_property) {
		ret = psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
		if (ret) {
			pr_err("%s: fail to set power_suppy otg property(%d)\n",
				__func__, ret);
			return -EBUSY;
		}
#ifdef CONFIG_USB_SWITCH_FSA9485
		if (!on)
			fsa9485_otg_detach();
#endif
		vbus_is_on = on;
	} else {
		pr_err("%s : psy is null!\n", __func__);
		return -EBUSY;
	}
	return 0;
}
#endif

#ifdef CONFIG_MFD_MAX77693
static int msm_hsusb_vbus_power_max77693(bool on)
{
	pr_info("%s, attached %d, vbus_is_on %d\n", __func__, on, vbus_is_on);

	/* If VBUS is already on (or off), do nothing. */
	if (vbus_is_on == on)
		return -EBUSY;

	otg_control(on);
	vbus_is_on = on;
	return 0;
}
#endif

static int msm_hsusb_vbus_power(bool on)
{
#ifdef CONFIG_MFD_MAX77693
		return msm_hsusb_vbus_power_max77693(on);
#endif
#ifdef CONFIG_CHARGER_BQ24157
		return msm_hsusb_vbus_power_bq24157(on);
#endif
	return 0;
}

static int phy_init_seq[] = {
	0x44, 0x80, /* set VBUS valid threshold
			and disconnect valid threshold */
	0x6F, 0x81, /* update DC voltage level */
	0x3C, 0x82, /* set preemphasis and rise/fall time */
	0x17, 0x83, /* set source impedance adjusment */
	-1};

#define PMIC_GPIO_DP		27    /* PMIC GPIO for D+ change */
#define PMIC_GPIO_DP_IRQ	PM8921_GPIO_IRQ(PM8921_IRQ_BASE, PMIC_GPIO_DP)
#define MSM_MPM_PIN_USB1_OTGSESSVLD	40

static struct msm_otg_platform_data msm_otg_pdata = {
	.mode			= USB_OTG,
	.otg_control		= OTG_PMIC_CONTROL,
	.phy_type		= SNPS_28NM_INTEGRATED_PHY,
	.pmic_id_irq		= PM8921_USB_ID_IN_IRQ(PM8921_IRQ_BASE),
	.power_budget		= 750,
	.bus_scale_table	= &usb_bus_scale_pdata,
	.phy_init_seq		= phy_init_seq,
	.mpm_otgsessvld_int	= MSM_MPM_PIN_USB1_OTGSESSVLD,
	.vbus_power		= msm_hsusb_vbus_power,
};

static struct msm_usb_host_platform_data msm_ehci_host_pdata3 = {
	.power_budget = 500,
};

#ifdef CONFIG_USB_EHCI_MSM_HOST4
static struct msm_usb_host_platform_data msm_ehci_host_pdata4;
#endif

#ifdef CONFIG_SENSORS_SSP
static int wakeup_mcu(void);
static int check_mcu_busy(void);
static int check_mcu_ready(void);
static int set_mcu_reset(int on);
static int check_ap_rev(void);
static int ssp_read_chg(void);
static int ssp_check_changes(void);
static void ssp_get_positions(int *acc, int *mag);

static struct ssp_platform_data ssp_pdata = {
	.wakeup_mcu = wakeup_mcu,
	.check_mcu_busy = check_mcu_busy,
	.check_mcu_ready = check_mcu_ready,
	.set_mcu_reset = set_mcu_reset,
	.check_ap_rev = check_ap_rev,
	.read_chg = ssp_read_chg,
	.check_changes = ssp_check_changes,
	.get_positions = ssp_get_positions,
};

static struct i2c_board_info ssp_info[] __initdata = {
	{
		I2C_BOARD_INFO("ssp", 0x18),
		.platform_data = &ssp_pdata,
		.irq = GPIO_MCU_AP_INT,
	},
};

static struct regulator *vsensor_2p85, *vsensor_1p8, *gesture_sensor_1p8;
static int sensor_power_2p85_cnt, sensor_power_1p8_cnt;

static void sensor_power_on_vdd(int onoff_l9, int onoff_lvs4)
{
	int ret;

	pr_info("[SSP]%s\n", __func__);
	if (vsensor_2p85 == NULL) {
		vsensor_2p85 = regulator_get(NULL, "8921_l9");
		if (IS_ERR(vsensor_2p85))
			return ;

		ret = regulator_set_voltage(vsensor_2p85, 2850000, 2850000);
		if (ret)
			pr_err("%s: error vsensor_2p85 setting voltage ret=%d\n",
				__func__, ret);
	}
	if (vsensor_1p8 == NULL) {
		vsensor_1p8 = regulator_get(NULL, "8921_lvs4");
		gesture_sensor_1p8 = regulator_get(NULL, "8917_l36");
		if (IS_ERR(vsensor_1p8) || IS_ERR(gesture_sensor_1p8))
			return ;
	}

	if (onoff_l9 == SNS_PWR_ON) {
		sensor_power_2p85_cnt++;
		ret = regulator_enable(vsensor_2p85);
		if (ret)
			pr_err("%s: error enabling regulator\n", __func__);
	} else if ((onoff_l9 == SNS_PWR_OFF)) {
		sensor_power_2p85_cnt--;
		if (regulator_is_enabled(vsensor_2p85)) {
			ret = regulator_disable(vsensor_2p85);
			if (ret)
				pr_err("%s: error vsensor_2p85 enabling regulator\n",
				__func__);
		}
	}
	if (onoff_lvs4 == SNS_PWR_ON) {
		sensor_power_1p8_cnt++;
		ret = regulator_enable(vsensor_1p8);
		ret |= regulator_enable(gesture_sensor_1p8);
		if (ret)
			pr_err("%s: error enabling regulator\n", __func__);
	} else if ((onoff_lvs4 == SNS_PWR_OFF)) {
		sensor_power_1p8_cnt--;
		if (regulator_is_enabled(vsensor_1p8)) {
			ret = regulator_disable(vsensor_1p8);
			if (ret)
				pr_err("%s: error vsensor_1p8 enabling regulator\n",
				__func__);
		}
		if (regulator_is_enabled(gesture_sensor_1p8)) {
			ret = regulator_disable(gesture_sensor_1p8);
			if (ret)
				pr_err("%s: error gesture_sensor_1p8 enabling regulator\n",
				__func__);
		}
	}
	pr_err("%s: done\n", __func__);
}

struct pm8xxx_mpp_config_data mpp4_cfg = {
		.type = PM8XXX_MPP_TYPE_D_OUTPUT,
		.level = PM8921_MPP_DIG_LEVEL_S4,
		.control = PM8XXX_MPP_DOUT_CTRL_LOW,
};

#define MPP_MCU_NRST PM8921_MPP_PM_TO_SYS(4)

static void clear_ssp_gpio(void)
{
	struct pm_gpio ap_mcu_int_cfg = {
		.direction = PM_GPIO_DIR_IN,
		.pull = PM_GPIO_PULL_DN,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};
	struct pm_gpio mcu_ap_int_2_cfg = {
		.direction = PM_GPIO_DIR_IN,
		.pull = PM_GPIO_PULL_DN,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};
	struct pm_gpio mcu_ap_int_cfg = {
		.direction = PM_GPIO_DIR_IN,
		.pull = PM_GPIO_PULL_DN,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};
	struct pm_gpio ap_mcu_nrst_cfg = {
		.direction = PM_GPIO_DIR_OUT,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
		.out_strength = PM_GPIO_STRENGTH_HIGH,
	};

	pm8xxx_gpio_config(GPIO_AP_MCU_INT, &ap_mcu_int_cfg);
	pm8xxx_gpio_config(GPIO_MCU_AP_INT, &mcu_ap_int_cfg);
	pm8xxx_gpio_config(GPIO_MCU_AP_INT_2, &mcu_ap_int_2_cfg);
	if (system_rev >= 5)
		pm8xxx_gpio_config(GPIO_MCU_NRST, &ap_mcu_nrst_cfg);
	gpio_set_value_cansleep(GPIO_MCU_NRST, 0);
	mdelay(1);
	pr_info("[SSP] %s done\n", __func__);
}

static int initialize_ssp_gpio(void)
{
	int err;
	struct pm_gpio ap_mcu_int_cfg = {
		.direction = PM_GPIO_DIR_OUT,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};
	struct pm_gpio mcu_ap_int_2_cfg = {
		.direction = PM_GPIO_DIR_IN,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};

	struct pm_gpio mcu_ap_int_cfg = {
		.direction = PM_GPIO_DIR_IN,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};
	struct pm_gpio ap_mcu_nrst_cfg = {
		.direction = PM_GPIO_DIR_OUT,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
		.out_strength = PM_GPIO_STRENGTH_HIGH,
	};

	pr_info("[SSP]%s\n", __func__);
	pm8xxx_gpio_config(GPIO_AP_MCU_INT, &ap_mcu_int_cfg);
	err = gpio_request(GPIO_AP_MCU_INT, "AP_MCU_INT");
	if (err)
		printk(KERN_ERR "failed to request AP_MCU_INT for SSP\n");
	gpio_set_value_cansleep(GPIO_AP_MCU_INT, 1);

	pm8xxx_gpio_config(GPIO_MCU_AP_INT, &mcu_ap_int_cfg);
	err = gpio_request(GPIO_MCU_AP_INT, "MCU_AP_INT");
	if (err)
		printk(KERN_ERR "failed to request MCU_AP_INT for SSP\n");
	else {
		gpio_direction_input(GPIO_MCU_AP_INT);
		gpio_free(GPIO_MCU_AP_INT);
	}

	pm8xxx_gpio_config(GPIO_MCU_AP_INT_2, &mcu_ap_int_2_cfg);
	err = gpio_request(GPIO_MCU_AP_INT_2, "MCU_AP_INT_2");
	if (err)
		printk(KERN_ERR "failed to request MCU_AP_INT_2 for SSP\n");

	if (system_rev >= 5) {
		pm8xxx_gpio_config(GPIO_MCU_NRST, &ap_mcu_nrst_cfg);
		err = gpio_request(GPIO_MCU_NRST, "MCU_NRST");
		if (err)
			printk(KERN_ERR
				"failed to request MCU_NRST for SSP\n");
		gpio_set_value_cansleep(GPIO_MCU_NRST, 1);
	} else {
		pm8xxx_mpp_config(MPP_MCU_NRST, &mpp4_cfg);
		err = gpio_request(MPP_MCU_NRST, "MCU_NRST");
		if (err)
			printk(KERN_ERR
				"failed to request MCU_NRST for SSP\n");
		gpio_set_value_cansleep(MPP_MCU_NRST, 1);
	}

	if (system_rev >= BOARD_REV03) {
		gpio_tlmm_config(GPIO_CFG(GPIO_MCU_CHG, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
		err = gpio_request(GPIO_MCU_CHG, "GPIO_MCU_CHG");
		if (err)
			pr_err("%s, failed to request GPIO_MCU_CHG for SSP\n",
				__func__);
	}
	return 0;
}

static int wakeup_mcu(void)
{
	gpio_set_value_cansleep(GPIO_AP_MCU_INT, 0);
	udelay(20);
	gpio_set_value_cansleep(GPIO_AP_MCU_INT, 1);

	return 0;
}

static int set_mcu_reset(int on)
{
	if (system_rev >= 5)
		gpio_set_value_cansleep(GPIO_MCU_NRST, on);
	else
		gpio_set_value_cansleep(MPP_MCU_NRST, on);

	return 0;
}

static int check_mcu_busy(void)
{
	return gpio_get_value_cansleep(GPIO_MCU_AP_INT);
}

static int check_mcu_ready(void)
{
	return gpio_get_value_cansleep(GPIO_MCU_AP_INT_2);
}

static int check_ap_rev(void)
{
	return system_rev;
}

static int ssp_read_chg()
{
	if (system_rev >= BOARD_REV03)
		return gpio_get_value(GPIO_MCU_CHG);
	else
		return 0;
}

static int ssp_check_changes(void)
{
	int changes = 0;
/* 0 : MPU6050, MCU L0
 * 1 : MPU6500, MCU L5 */

	if (system_rev < BOARD_REV03)
		changes = SSP_MCU_L0;
	else
		changes = SSP_MCU_L5;

	return changes;
}

/* MCU
 * 0,     1,      2,      3,      4,     5,      6,      7
 * PXPYPZ, PYNXPZ, NXNYPZ, NYPXPZ, NXPYNZ, PYPXNZ, PXNYNZ, NYNXNZ
 * *******************************************************
 * Sensors
 * top/upper-left => top/upper-right ... ... =>	bottom/lower-left
 * 1. K330
 * NYPXPZ, PXPYPZ, PYNXPZ, NXNYPZ, NYPXNZ, PXPYNZ, NYNXNZ, NXPYNZ
 * 2. AK8963C
 * NXNYPZ, NYPXPZ, PXPYPZ, PYNXPZ, PXNYNZ, PYPXNZ, NXPYNZ, NYNXNZ
 * 3. LSM330DLC
 * NXNYPZ, NYPXPZ, PXPYPZ, PYNXPZ, PXNYNZ, PYPXNZ, NXPYNZ, NYNXNZ
*/
static void ssp_get_positions(int *acc, int *mag)
{
	if (system_rev >= BOARD_REV11)
		*acc = K330_TOP_LEFT_UPPER;
	else if (system_rev > BOARD_REV04)
		*acc = MPU6500_TOP_RIGHT_UPPER;
	else
		*acc = MPU6500_BOTTOM_RIGHT_UPPER;

	if (system_rev > BOARD_REV06)
		*mag = YAS532_BOTTOM_RIGHT_LOWER;
	else if (system_rev > BOARD_REV03)
		*mag = YAS532_TOP_RIGHT_LOWER;
	else if (system_rev == BOARD_REV03)
		*mag = YAS532_BOTTOM_RIGHT_LOWER;
	else
		*mag = YAS532_TOP_LEFT_UPPER;

	pr_info("%s, position acc : %d, mag = %d\n", __func__, *acc, *mag);
}
#endif /* CONFIG_SENSORS_SSP */

#ifdef CONFIG_FELICA
static int fpga_felica_status = 0;

void set_fpga_felica_flag(int on)
{
	fpga_felica_status = on;
	return;
}

static int get_fpga_felica_flag(void)
{
	return fpga_felica_status;
}

static int __init felica_init(void)
{
	struct pm_gpio felica_irq_cfg = {
		.direction = PM_GPIO_DIR_IN,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};
	struct pm_gpio nfc_irq_cfg = {
		.direction = PM_GPIO_DIR_IN,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};
	struct pm_gpio felica_rfs_cfg = {
		.direction = PM_GPIO_DIR_IN,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};

	pm8xxx_gpio_config(GPIO_FELICA_INT, &felica_irq_cfg);

	/* FELICA_INTU GPIO is changed */
	if(system_rev > 0 && system_rev < 15) {
		pm8xxx_gpio_config(GPIO_FELICA_INTU, 
			&nfc_irq_cfg);
	}
	else {
		gpio_tlmm_config(GPIO_CFG(GPIO_FELICA_INTU_REV06, 
			GPIOMUX_FUNC_GPIO, GPIO_CFG_INPUT, 
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	}
	
	pm8xxx_gpio_config(GPIO_FELICA_RFS, &felica_rfs_cfg);

	if(ice_gpiox_set(FPGA_GPIO_FELICA_PON, 0))
	{
		pr_err(KERN_ERR " %s: FeliCa init failed in setting PON=0\n",__func__);
	};
	if(ice_gpiox_set(FPGA_GPIO_FELICA_HSEL, 0))
	{
		pr_err(KERN_ERR "%s: FeliCa init failed in setting HSEL=0\n",__func__);
	};
	
	pr_info("[FELICA] %s done\n", __func__);
	return 0;
}
#endif /* CONFIG_FELICA */

#define NFC_SW_I2C
/* if undef NFC_SW_I2C
 * have to add gpiomux_setting gsbi4
 * in board_fusion3_xxx_gpiomux.c
 */

#ifdef CONFIG_BCM2079X_NFC_I2C
#ifdef NFC_SW_I2C
static void bcm2079x_sw_i2c_config(void)
{
	pr_info("%s is called\n", __func__);
	gpio_tlmm_config(GPIO_CFG(10, 0, GPIO_CFG_INPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	gpio_tlmm_config(GPIO_CFG(11, 0, GPIO_CFG_INPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
}
#endif
static int __init bcm2079x_init(void)
{
	struct pm_gpio nfc_irq_cfg = {
		.direction = PM_GPIO_DIR_IN,
		.pull = PM_GPIO_PULL_DN,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};
	struct pm_gpio nfc_en_cfg = {
		.direction = PM_GPIO_DIR_OUT,
		.pull = PM_GPIO_PULL_DN,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};
	struct pm_gpio nfc_firmware_cfg = {
		.direction = PM_GPIO_DIR_OUT,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};
	pm8xxx_gpio_config(GPIO_NFC_IRQ, &nfc_irq_cfg);
	pm8xxx_gpio_config(GPIO_NFC_EN, &nfc_en_cfg);
	pm8xxx_gpio_config(GPIO_NFC_FIRMWARE, &nfc_firmware_cfg);
#ifdef NFC_SW_I2C
	bcm2079x_sw_i2c_config();
#endif
	pr_info("[NFC] %s done\n", __func__);
	return 0;
}

#ifdef NFC_SW_I2C
static struct i2c_gpio_platform_data bcm2079x_i2c_gpio_data = {
	.sda_pin = 10,
	.scl_pin = 11,
};

static struct platform_device bcm2079x_i2c_gpio_device = {
	.name = "i2c-gpio",
	.id = MSM_NFC_I2C_BUS_ID,
	.dev = {
		.platform_data  = &bcm2079x_i2c_gpio_data,
	},
};
#endif
static struct bcm2079x_platform_data bcm2079x_i2c_pdata = {
	.irq_gpio = GPIO_NFC_IRQ,
	.en_gpio = GPIO_NFC_EN,
	.wake_gpio = GPIO_NFC_FIRMWARE,
};

static struct i2c_board_info nfc_bcm2079x_info[] __initdata = {
	{
		I2C_BOARD_INFO("bcm2079x-i2c", 0x77),
		.irq = GPIO_NFC_IRQ,
		.platform_data = &bcm2079x_i2c_pdata,
	},
};
#endif /* CONFIG_BCM2079X_NFC_I2C */

#ifdef CONFIG_CAMERA_SW_I2C
static struct i2c_gpio_platform_data s5k6b2yx_i2c_gpio_data = {
	.sda_pin = 71,
	.scl_pin = 70,
	.udelay = 1,
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

struct platform_device s5k6b2yx_i2c_gpio_device = {
	.name = "i2c-gpio",
	.id = MSM_CAMERA_SW_I2C_BUS_ID,
	.dev = {
		.platform_data  = &s5k6b2yx_i2c_gpio_data,
	},
};
#endif

#ifdef CONFIG_SEC_FPGA
static struct regulator *barcode_vreg_2p85, *barcode_vreg_l33;
static struct regulator *barcode_vreg_1p8;
static struct msm_xo_voter *fpga_xo;
static void irda_led_poweron(int onoff)
{
	int ret;
	static struct regulator *reg_l10;
	if (system_rev >= BOARD_REV09) {
		if(onoff) {
			if (!reg_l10) {
				reg_l10 = regulator_get(NULL, "8921_l10");
				ret = regulator_set_voltage(reg_l10, 3300000, 3300000);

				if (IS_ERR(reg_l10)) {
					printk(KERN_ERR"could not get 8921_l10, rc = %ld\n",
							PTR_ERR(reg_l10));
				}
			}
			ret = regulator_enable(reg_l10);
				if (ret)
					printk(KERN_ERR"enable l10 failed, rc=%d\n", ret);
		} else {
			if (regulator_is_enabled(reg_l10)) {
				ret = regulator_disable(reg_l10);
				if (ret)
					printk(KERN_ERR"disable l10 failed, rc=%d\n", ret);
			}
		}
	}
}

static int ice4_clock_en(int onoff)
{
	if (system_rev < BOARD_REV04)
		return 0;

	if (system_rev >= BOARD_REV04 && system_rev <= BOARD_REV08) {
	if (onoff)
		msm_xo_mode_vote(fpga_xo, MSM_XO_MODE_ON);
	else
		msm_xo_mode_vote(fpga_xo, MSM_XO_MODE_OFF);
	} else if (system_rev >= BOARD_REV09) {
		static struct clk *fpga_main_clk;
		if (!fpga_main_clk)
			fpga_main_clk = clk_get(NULL, "gp0_clk");

		if (onoff) {
			clk_set_rate(fpga_main_clk, 24000000);
			clk_prepare_enable(fpga_main_clk);
		} else {
		       clk_disable_unprepare(fpga_main_clk);
		       clk_put(fpga_main_clk);
		       fpga_main_clk = NULL;
		}
	}
	return 0;
}
static void barcode_emul_poweron(int onoff)
{
	int ret;

	pr_info("%s onoff : %d\n", __func__, onoff);

	if (barcode_vreg_2p85 == NULL) {
		barcode_vreg_2p85 = regulator_get(NULL, "8921_l9");
		if (IS_ERR(barcode_vreg_2p85))
			return ;

		ret = regulator_set_voltage(barcode_vreg_2p85,
				2850000, 2850000);
		if (ret)
			pr_err("%s: error vreg_2p85 set voltage ret=%d\n",
				__func__, ret);
	}
	if (barcode_vreg_l33 == NULL) {
			barcode_vreg_l33 = regulator_get(NULL,
								"8917_l33");
		if (IS_ERR(barcode_vreg_l33))
			return ;
		ret = regulator_set_voltage(barcode_vreg_l33,
						1200000, 1200000);
		if (ret)
			pr_err("%s: error vreg_l33 set voltage ret=%d\n",
							__func__, ret);
	}
	if (barcode_vreg_1p8 == NULL) {
		barcode_vreg_1p8 = regulator_get(NULL, "8921_lvs4");
		if (IS_ERR(barcode_vreg_1p8))
			return ;
	}

	if (onoff) {
		ret = regulator_enable(barcode_vreg_l33);
		if (ret)
			pr_err("%s: error enabling regulator\n",
								__func__);
			pr_info("%s gpio switch on\n", __func__);
		ret = regulator_enable(barcode_vreg_2p85);
		if (ret)
			pr_err("%s: error enabling regulator\n", __func__);
		ret = regulator_enable(barcode_vreg_1p8);
		if (ret)
			pr_err("%s: error enabling regulator\n", __func__);
		if (system_rev > BOARD_REV03) {
			fpga_xo = msm_xo_get(MSM_XO_TCXO_A0, "ice4_fpga");
			if (IS_ERR(fpga_xo)) {
				printk(KERN_ERR "%s: Couldn't get TCXO_A0 vote for ice4_fpga\n",
				__func__);
			}
		}
	} else {
		if (system_rev > BOARD_REV03)
			msm_xo_put(fpga_xo);
		if (regulator_is_enabled(barcode_vreg_2p85)) {
			ret = regulator_disable(barcode_vreg_2p85);
			if (ret)
				pr_err("%s: error disabling regulator\n",
				__func__);
		}
		if (regulator_is_enabled(barcode_vreg_1p8)) {
			ret = regulator_disable(barcode_vreg_1p8);
			if (ret)
				pr_err("%s: error disabling regulator\n",
				__func__);
		}
		if (regulator_is_enabled(barcode_vreg_l33)) {
			ret = regulator_disable(barcode_vreg_l33);
			if (ret)
				pr_err("%s: error disabling regulator\n",
						__func__);
		}
	}
}

static void barcode_gpio_config(void);

struct barcode_emul_platform_data barcode_emul_info = {
	.spi_en  = GPIO_FPGA_SPI_EN,
	.cresetb = GPIO_FPGA_CRESET_B,
	.rst_n   = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_FPGA_RST_N),
	.cdone   = GPIO_FPGA_CDONE,
	.clock_en = ice4_clock_en,
	.poweron = barcode_emul_poweron,
	.gpioconfig = barcode_gpio_config,
#if defined(CONFIG_IR_REMOCON_FPGA)
	.ir_remote_init = irda_device_init,
	.irda_wake = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_IRDA_WAKE),
	.irda_irq = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_IRDA_IRQ),
	.ir_wake_en = irda_wake_en,
	.ir_vdd_onoff = irda_vdd_onoff,
	.ir_led_poweron = irda_led_poweron,
#endif
	.get_fpga_felica_flag = get_fpga_felica_flag,
};

static void barcode_gpio_config(void)
{
	struct pm8xxx_mpp_config_data barcode_mppcfg = {
			.type = PM8XXX_MPP_TYPE_D_OUTPUT,
			.level = PM8921_MPP_DIG_LEVEL_S4,
			.control = PM8XXX_MPP_DOUT_CTRL_LOW,
	};

	struct pm_gpio creset_b = {
		.direction		= PM_GPIO_DIR_OUT,
		.pull			= PM_GPIO_PULL_NO,
		.out_strength		= PM_GPIO_STRENGTH_HIGH,
		.function		= PM_GPIO_FUNC_NORMAL,
		.inv_int_pol		= 0,
		.vin_sel		= PM_GPIO_VIN_S4,
		.output_buffer		= PM_GPIO_OUT_BUF_CMOS,
		.output_value		= 0,
	};

	pr_info("%s\n", __func__);
	pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(	\
			PMIC_GPIO_FPGA_RST_N), &creset_b);

	gpio_request_one(GPIO_FPGA_CDONE, GPIOF_IN, "FPGA_CDONE");
	if (system_rev < BOARD_REV06)
		gpio_request_one(GPIO_FPGA_SPI_EN, GPIOF_OUT_INIT_LOW,
							"FPGA_SPI_EN");
	if (system_rev > BOARD_REV05) {
		pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(	\
				PMIC_GPIO_FPGA_CRESET_B), &creset_b);
	} else {
		gpio_request_one(GPIO_FPGA_CRESET_B, GPIOF_OUT_INIT_LOW,
							"FPGA_CRESET_B");
	}
	if (system_rev > BOARD_REV03 && system_rev < BOARD_REV06)
		barcode_emul_info.fw_type = ICE_19M;
	else if (system_rev > BOARD_REV05 && system_rev < BOARD_REV09)
		barcode_emul_info.fw_type = ICE_I2C;
	else if (system_rev >= BOARD_REV09)
		barcode_emul_info.fw_type = ICE_24M;

	pm8xxx_mpp_config(PM8921_MPP_PM_TO_SYS(PMIC_MPP_FPGA_SPI_CLK),
						&barcode_mppcfg);
	pm8xxx_mpp_config(PM8921_MPP_PM_TO_SYS(PMIC_MPP_FPGA_SPI_SI),
						&barcode_mppcfg);

	if (system_rev >= BOARD_REV09) {
			gpio_tlmm_config(GPIO_CFG(GPIO_FPGA_MAIN_CLK, \
						2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	}
}

static struct i2c_gpio_platform_data barcode_i2c_gpio_data = {
#if defined(CONFIG_IR_REMOCON_FPGA)
	.udelay = 2,
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
#endif
};

struct platform_device barcode_i2c_gpio_device = {
	.name = "i2c-gpio",
	.id = MSM_SEC_FPGA_I2C_BUS_ID,
	.dev.platform_data = &barcode_i2c_gpio_data,
};

static struct i2c_board_info ice4_fpga_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("ice4", (0x6c)),
		.platform_data = &barcode_emul_info,
	},
};
#endif

static void __init apq8064_ehci_host_init(void)
{
	if (machine_is_apq8064_liquid() || machine_is_mpq8064_cdp() ||
		machine_is_mpq8064_hrd() || machine_is_mpq8064_dtv()) {
		if (machine_is_apq8064_liquid())
			msm_ehci_host_pdata3.dock_connect_irq =
					PM8921_MPP_IRQ(PM8921_IRQ_BASE, 9);
		else
			msm_ehci_host_pdata3.pmic_gpio_dp_irq =
							PMIC_GPIO_DP_IRQ;

		apq8064_device_ehci_host3.dev.platform_data =
				&msm_ehci_host_pdata3;
		platform_device_register(&apq8064_device_ehci_host3);

#ifdef CONFIG_USB_EHCI_MSM_HOST4
		apq8064_device_ehci_host4.dev.platform_data =
				&msm_ehci_host_pdata4;
		platform_device_register(&apq8064_device_ehci_host4);
#endif
	}
}

static struct smb349_platform_data smb349_data __initdata = {
	.en_n_gpio		= PM8921_GPIO_PM_TO_SYS(37),
	.chg_susp_gpio		= PM8921_GPIO_PM_TO_SYS(30),
	.chg_current_ma		= 2200,
};

static struct i2c_board_info smb349_charger_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO(SMB349_NAME, 0x1B),
		.platform_data	= &smb349_data,
	},
};

struct sx150x_platform_data apq8064_sx150x_data[] = {
	[SX150X_EPM] = {
		.gpio_base	= GPIO_EPM_EXPANDER_BASE,
		.oscio_is_gpo	= false,
		.io_pullup_ena	= 0x0,
		.io_pulldn_ena	= 0x0,
		.io_open_drain_ena = 0x0,
		.io_polarity	= 0,
		.irq_summary	= -1,
	},
};

static struct epm_chan_properties ads_adc_channel_data[] = {
	{10, 100}, {1000, 1}, {10, 100}, {1000, 1},
	{10, 100}, {1000, 1}, {10, 100}, {1000, 1},
	{10, 100}, {20, 100}, {500, 100}, {5, 100},
	{1000, 1}, {200, 100}, {50, 100}, {10, 100},
	{510, 100}, {50, 100}, {20, 100}, {100, 100},
	{510, 100}, {20, 100}, {50, 100}, {200, 100},
	{10, 100}, {20, 100}, {1000, 1}, {10, 100},
	{200, 100}, {510, 100}, {1000, 100}, {200, 100},
};

static struct epm_adc_platform_data epm_adc_pdata = {
	.channel		= ads_adc_channel_data,
	.bus_id	= 0x0,
	.epm_i2c_board_info = {
		.type	= "sx1509q",
		.addr = 0x3e,
		.platform_data = &apq8064_sx150x_data[SX150X_EPM],
	},
	.gpio_expander_base_addr = GPIO_EPM_EXPANDER_BASE,
};

static struct platform_device epm_adc_device = {
	.name   = "epm_adc",
	.id = -1,
	.dev = {
		.platform_data = &epm_adc_pdata,
	},
};

static void __init apq8064_epm_adc_init(void)
{
	epm_adc_pdata.num_channels = 32;
	epm_adc_pdata.num_adc = 2;
	epm_adc_pdata.chan_per_adc = 16;
	epm_adc_pdata.chan_per_mux = 8;
};

/* Micbias setting is based on 8660 CDP/MTP/FLUID requirement
 * 4 micbiases are used to power various analog and digital
 * microphones operating at 1800 mV. Technically, all micbiases
 * can source from single cfilter since all microphones operate
 * at the same voltage level. The arrangement below is to make
 * sure all cfilters are exercised. LDO_H regulator ouput level
 * does not need to be as high as 2.85V. It is choosen for
 * microphone sensitivity purpose.
 */
static struct wcd9xxx_pdata apq8064_tabla_platform_data = {
	.slimbus_slave_device = {
		.name = "tabla-slave",
		.e_addr = {0, 0, 0x10, 0, 0x17, 2},
	},
	.irq = MSM_GPIO_TO_INT(42),
	.irq_base = TABLA_INTERRUPT_BASE,
	.num_irqs = NR_WCD9XXX_IRQS,
	.reset_gpio = PM8921_GPIO_PM_TO_SYS(38),
	.micbias = {
		.ldoh_v = TABLA_LDOH_2P85_V,
		.cfilt1_mv = 1800,
		.cfilt2_mv = 2700,
		.cfilt3_mv = 1800,
		.bias1_cfilt_sel = TABLA_CFILT1_SEL,
		.bias2_cfilt_sel = TABLA_CFILT2_SEL,
		.bias3_cfilt_sel = TABLA_CFILT3_SEL,
		.bias4_cfilt_sel = TABLA_CFILT3_SEL,
	},
	.regulator = {
	{
		.name = "CDC_VDD_CP",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_CP_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_RX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_RX_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_TX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_TX_CUR_MAX,
	},
	{
		.name = "VDDIO_CDC",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_VDDIO_CDC_CUR_MAX,
	},
	{
		.name = "VDDD_CDC_D",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_D_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_A_1P2V",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_A_CUR_MAX,
	},
	},
};

static struct slim_device apq8064_slim_tabla = {
	.name = "tabla-slim",
	.e_addr = {0, 1, 0x10, 0, 0x17, 2},
	.dev = {
		.platform_data = &apq8064_tabla_platform_data,
	},
};

static struct wcd9xxx_pdata apq8064_tabla20_platform_data = {
	.slimbus_slave_device = {
		.name = "tabla-slave",
		.e_addr = {0, 0, 0x60, 0, 0x17, 2},
	},
	.irq = MSM_GPIO_TO_INT(42),
	.irq_base = TABLA_INTERRUPT_BASE,
	.num_irqs = NR_WCD9XXX_IRQS,
	.reset_gpio = PM8921_GPIO_PM_TO_SYS(38),
	.micbias = {
		.ldoh_v = TABLA_LDOH_2P85_V,
		.cfilt1_mv = 1800,
		.cfilt2_mv = 2700,
		.cfilt3_mv = 1800,
		.bias1_cfilt_sel = TABLA_CFILT1_SEL,
		.bias2_cfilt_sel = TABLA_CFILT2_SEL,
		.bias3_cfilt_sel = TABLA_CFILT3_SEL,
		.bias4_cfilt_sel = TABLA_CFILT3_SEL,
	},
	.regulator = {
	{
		.name = "CDC_VDD_CP",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_CP_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_RX",
		.min_uV = 2200000,
		.max_uV = 2200000,
		.optimum_uA = WCD9XXX_CDC_VDDA_RX_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_TX",
		.min_uV = 2200000,
		.max_uV = 2200000,
		.optimum_uA = WCD9XXX_CDC_VDDA_TX_CUR_MAX,
	},
	{
		.name = "VDDIO_CDC",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_VDDIO_CDC_CUR_MAX,
	},
	{
		.name = "VDDD_CDC_D",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_D_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_A_1P2V",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_A_CUR_MAX,
	},
	},
};

static struct slim_device apq8064_slim_tabla20 = {
	.name = "tabla2x-slim",
	.e_addr = {0, 1, 0x60, 0, 0x17, 2},
	.dev = {
		.platform_data = &apq8064_tabla20_platform_data,
	},
};

/* enable the level shifter for cs8427 to make sure the I2C
 * clock is running at 100KHz and voltage levels are at 3.3
 * and 5 volts
 */
static int enable_100KHz_ls(int enable)
{
	int ret = 0;
	if (enable) {
		ret = gpio_request(SX150X_GPIO(1, 10),
					"cs8427_100KHZ_ENABLE");
		if (ret) {
			pr_err("%s: Failed to request gpio %d\n", __func__,
				SX150X_GPIO(1, 10));
			return ret;
		}
		gpio_direction_output(SX150X_GPIO(1, 10), 1);
	} else {
		gpio_direction_output(SX150X_GPIO(1, 10), 0);
		gpio_free(SX150X_GPIO(1, 10));
	}
	return ret;
}

static struct cs8427_platform_data cs8427_i2c_platform_data = {
	.irq = SX150X_GPIO(1, 4),
	.reset_gpio = SX150X_GPIO(1, 6),
	.enable = enable_100KHz_ls,
};

static struct i2c_board_info cs8427_device_info[] __initdata = {
	{
		I2C_BOARD_INFO("cs8427", CS8427_ADDR4),
		.platform_data = &cs8427_i2c_platform_data,
	},
};

#ifdef CONFIG_HAPTIC_ISA1200
#define HAP_SHIFT_LVL_OE_GPIO		PM8921_MPP_PM_TO_SYS(8)
#define ISA1200_HAP_EN_GPIO		PM8921_GPIO_PM_TO_SYS(33)
#define ISA1200_HAP_LEN_GPIO		PM8921_GPIO_PM_TO_SYS(20)
#define ISA1200_HAP_CLK_PM8921		PM8921_GPIO_PM_TO_SYS(44)
#define ISA1200_HAP_CLK_PM8917		PM8921_GPIO_PM_TO_SYS(38)

static int isa1200_clk_enable(bool on)
{
	unsigned int gpio = ISA1200_HAP_CLK_PM8921;
	int rc = 0;

	if (socinfo_get_pmic_model() == PMIC_MODEL_PM8917)
		gpio = ISA1200_HAP_CLK_PM8917;

	gpio_set_value_cansleep(gpio, on);

	if (on) {
		rc = pm8xxx_aux_clk_control(CLK_MP3_2, XO_DIV_1, true);
		if (rc) {
			pr_err("%s: unable to write aux clock register(%d)\n",
				__func__, rc);
			goto err_gpio_dis;
		}
	} else {
		rc = pm8xxx_aux_clk_control(CLK_MP3_2, XO_DIV_NONE, true);
		if (rc)
			pr_err("%s: unable to write aux clock register(%d)\n",
				__func__, rc);
	}

	return rc;

err_gpio_dis:
	gpio_set_value_cansleep(gpio, !on);
	return rc;
}

static int isa1200_dev_setup(bool enable)
{
	unsigned int gpio = ISA1200_HAP_CLK_PM8921;
	int rc = 0;

	if (socinfo_get_pmic_model() == PMIC_MODEL_PM8917)
		gpio = ISA1200_HAP_CLK_PM8917;

	if (!enable)
		goto free_gpio;

	rc = gpio_request(gpio, "haptics_clk");
	if (rc) {
		pr_err("%s: unable to request gpio %d config(%d)\n",
			__func__, gpio, rc);
		return rc;
	}

	rc = gpio_direction_output(gpio, 0);
	if (rc) {
		pr_err("%s: unable to set direction\n", __func__);
		goto free_gpio;
	}

	return 0;

free_gpio:
	gpio_free(gpio);
	return rc;
}

static struct isa1200_regulator isa1200_reg_data[] = {
	{
		.name = "vddp",
		.min_uV = ISA_I2C_VTG_MIN_UV,
		.max_uV = ISA_I2C_VTG_MAX_UV,
		.load_uA = ISA_I2C_CURR_UA,
	},
};

static struct isa1200_platform_data isa1200_1_pdata = {
	.name = "vibrator",
	.dev_setup = isa1200_dev_setup,
	.clk_enable = isa1200_clk_enable,
	.need_pwm_clk = true,
	.hap_en_gpio = ISA1200_HAP_EN_GPIO,
	.hap_len_gpio = ISA1200_HAP_LEN_GPIO,
	.max_timeout = 15000,
	.mode_ctrl = PWM_GEN_MODE,
	.pwm_fd = {
		.pwm_div = 256,
	},
	.is_erm = false,
	.smart_en = true,
	.ext_clk_en = true,
	.chip_en = 1,
	.regulator_info = isa1200_reg_data,
	.num_regulators = ARRAY_SIZE(isa1200_reg_data),
};

static struct i2c_board_info isa1200_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("isa1200_1", 0x90>>1),
		.platform_data = &isa1200_1_pdata,
	},
};
#endif

#define MSM_WCNSS_PHYS	0x03000000
#define MSM_WCNSS_SIZE	0x280000

static struct resource resources_wcnss_wlan[] = {
	{
		.start	= RIVA_APPS_WLAN_RX_DATA_AVAIL_IRQ,
		.end	= RIVA_APPS_WLAN_RX_DATA_AVAIL_IRQ,
		.name	= "wcnss_wlanrx_irq",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= RIVA_APPS_WLAN_DATA_XFER_DONE_IRQ,
		.end	= RIVA_APPS_WLAN_DATA_XFER_DONE_IRQ,
		.name	= "wcnss_wlantx_irq",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_WCNSS_PHYS,
		.end	= MSM_WCNSS_PHYS + MSM_WCNSS_SIZE - 1,
		.name	= "wcnss_mmio",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 64,
		.end	= 68,
		.name	= "wcnss_gpios_5wire",
		.flags	= IORESOURCE_IO,
	},
};

static struct qcom_wcnss_opts qcom_wcnss_pdata = {
	.has_48mhz_xo	= 1,
};

static struct platform_device msm_device_wcnss_wlan = {
	.name		= "wcnss_wlan",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_wcnss_wlan),
	.resource	= resources_wcnss_wlan,
	.dev		= {.platform_data = &qcom_wcnss_pdata},
};

static struct platform_device msm_device_iris_fm __devinitdata = {
	.name = "iris_fm",
	.id   = -1,
};
#if defined(CONFIG_LEDS_AN30259A)
static struct i2c_board_info leds_i2c_devs[] __initdata = {
	{
		I2C_BOARD_INFO("an30259a", 0x30),
	},
};
#endif
#if defined(CONFIG_FELICA)
static struct i2c_board_info felica_i2c_devs[] __initdata = {
	{
		I2C_BOARD_INFO("felica_i2c", (0x56 >> 1)),
	},
};
#endif /* CONFIG_FELICA */
#ifdef CONFIG_QSEECOM
/* qseecom bus scaling */
static struct msm_bus_vectors qseecom_clks_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = 0,
		.ab = 0,
	},
};

static struct msm_bus_vectors qseecom_enable_dfab_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 70000000UL,
		.ib = 70000000UL,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 2480000000UL,
		.ib = 2480000000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = 0,
		.ab = 0,
	},
};

static struct msm_bus_vectors qseecom_enable_sfpb_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = (64 * 8) * 1000000UL,
		.ab = (64 * 8) *  100000UL,
	},
};

static struct msm_bus_vectors qseecom_enable_dfab_sfpb_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 70000000UL,
		.ib = 70000000UL,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 2480000000UL,
		.ib = 2480000000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = (64 * 8) * 1000000UL,
		.ab = (64 * 8) *  100000UL,
	},
};

static struct msm_bus_paths qseecom_hw_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(qseecom_clks_init_vectors),
		qseecom_clks_init_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_dfab_vectors),
		qseecom_enable_dfab_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_sfpb_vectors),
		qseecom_enable_sfpb_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_dfab_sfpb_vectors),
		qseecom_enable_dfab_sfpb_vectors,
	},
};

static struct msm_bus_scale_pdata qseecom_bus_pdata = {
	qseecom_hw_bus_scale_usecases,
	ARRAY_SIZE(qseecom_hw_bus_scale_usecases),
	.name = "qsee",
};

static struct platform_device qseecom_device = {
	.name		= "qseecom",
	.id		= 0,
	.dev		= {
		.platform_data = &qseecom_bus_pdata,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0x11000000

#define QCE_HW_KEY_SUPPORT	0
#define QCE_SHA_HMAC_SUPPORT	1
#define QCE_SHARE_CE_RESOURCE	3
#define QCE_CE_SHARED		0

static struct resource qcrypto_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV8064_CE_IN_CHAN,
		.end = DMOV8064_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV8064_CE_IN_CRCI,
		.end = DMOV8064_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV8064_CE_OUT_CRCI,
		.end = DMOV8064_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

static struct resource qcedev_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV8064_CE_IN_CHAN,
		.end = DMOV8064_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV8064_CE_IN_CRCI,
		.end = DMOV8064_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV8064_CE_OUT_CRCI,
		.end = DMOV8064_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)

static struct msm_ce_hw_support qcrypto_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	.bus_scale_table = NULL,
};

static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcrypto_resources),
	.resource	= qcrypto_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcrypto_ce_hw_suppport,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

static struct msm_ce_hw_support qcedev_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	.bus_scale_table = NULL,
};

static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcedev_resources),
	.resource	= qcedev_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcedev_ce_hw_suppport,
	},
};
#endif

#ifdef CONFIG_ISDBTMM
#define SPI_DMA_BASE_PHYS		0x16300000
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *tmm_base = 0;

	tmm_base = ioremap(SPI_DMA_BASE_PHYS, PAGE_SIZE);
	if (!tmm_base) {
		pr_err("%s: Could not remap %x\n", __func__, SPI_DMA_BASE_PHYS);
		pr_err("spi tmm remap failed\n");
		return -ENOMEM;
	}
	iounmap(tmm_base);
	
	return 0;
}

/* I2C25 */
static struct i2c_gpio_platform_data gpio_i2c_tmm25 = {
	.scl_pin = GPIO_TMM_I2C_SCL,
	.sda_pin = GPIO_TMM_I2C_SDA,
};

struct platform_device smtej113_device_i2c25 = {
	.name = "i2c-gpio",
	.id = 25,
	.dev.platform_data = &gpio_i2c_tmm25,
};

static struct i2c_board_info i2c_smtej113_tmm[] __initdata = {
	{
		I2C_BOARD_INFO("smtej113_main1", 0xC2 >> 1),
	},
	{
		I2C_BOARD_INFO("smtej113_main2", 0xC4 >> 1),
	},
	{
		I2C_BOARD_INFO("smtej113_sub", 0xC0 >> 1),
	},
};

static struct msm_spi_platform_data apq8064_qup_spi_gsbi4_pdata = {
	.max_clock_speed = 27000000,
	.dma_config      = msm_qsd_spi_dma_config,
};

static struct spi_board_info spi1_board_info[] __initdata = {
	{
		.modalias			= "tmmspidev",
		.max_speed_hz		= 27 * 1000 * 1000,
		.bus_num			= 1,
		.chip_select		= 0,
		.mode				= SPI_MODE_0,
	}
};

static struct platform_device tmm_spi_device = {
	.name			= "tmmspi",
	.id 			= -1,
};

#if defined(CONFIG_TMM_CHG_CTRL)
#define TMM_CHG_INPUT_CURRENT_LIMIT 900

static void tmm_reduce_chg_curr(void)
{
	union power_supply_propval value;
	int cable_type;
	int input_curr_limit;
	
	tmm_chg_log(KERN_ALERT"%s: start!\n", __func__);

	psy_do_property("battery", get, POWER_SUPPLY_PROP_ONLINE, value);

	cable_type = value.intval;

	if((cable_type != POWER_SUPPLY_TYPE_UNKNOWN) && (cable_type != POWER_SUPPLY_TYPE_BATTERY))
	{
		/* charging on! */
		tmm_chg_log(KERN_ALERT"%s: charging on! cable_type=%d\n", __func__, cable_type);

		psy_do_property("sec-charger", get, POWER_SUPPLY_PROP_CURRENT_NOW, value);

		input_curr_limit = value.intval;

		tmm_chg_log(KERN_ALERT"%s: input_curr_limit=%d\n", __func__, input_curr_limit);

		if(input_curr_limit > TMM_CHG_INPUT_CURRENT_LIMIT)
		{
			value.intval = TMM_CHG_INPUT_CURRENT_LIMIT;

			psy_do_property("sec-charger", set, POWER_SUPPLY_PROP_POWER_NOW, value);

			tmm_chg_log(KERN_ALERT"%s: set input current limit %dmA!\n", __func__, value.intval);
		}
	}

	return;

};

static void tmm_recover_chg_curr(void)
{
	union power_supply_propval value;
	int cable_type;

	tmm_chg_log(KERN_ALERT"%s: start!\n", __func__);
	
	psy_do_property("battery", get, POWER_SUPPLY_PROP_ONLINE, value);

	cable_type = value.intval;

	tmm_chg_log(KERN_ALERT"%s: cable_type=%d\n", __func__, cable_type);

	value.intval = cable_type;

	psy_do_property("sec-charger", set, POWER_SUPPLY_PROP_ONLINE, value);
	
	tmm_chg_log(KERN_ALERT"%s: POWER_SUPPLY_PROP_ONLINE!\n", __func__);

	return;
};

static struct isdbtmm_platform_data isdbtmm_pdata = {
	.tmm_reduce_chg_curr = tmm_reduce_chg_curr,
	.tmm_recover_chg_curr = tmm_recover_chg_curr,
};
#endif

static struct platform_device tmm_i2c_device = {
	.name			= "tmmi2c",
	.id 			= -1,
#if defined(CONFIG_TMM_CHG_CTRL)
	.dev.platform_data = &isdbtmm_pdata,
#endif
};

void __init tmm_dev_init(void)
{
	int ret;
	
	ret = platform_device_register(&tmm_i2c_device);
	
	if (ret < 0) {
		pr_err("%s: i2c platform_device_register returned error ret = %d\n", __func__, ret);
		return;
	}
	
	ret = platform_device_register(&tmm_spi_device);
	
	if (ret < 0) {
		pr_err("%s: spi platform_device_register returned error ret = %d\n", __func__, ret);
		return;
	}
	
	i2c_register_board_info(25, i2c_smtej113_tmm,
							ARRAY_SIZE(i2c_smtej113_tmm));

	ret = spi_register_board_info(spi1_board_info,
									ARRAY_SIZE(spi1_board_info));
		
	if (ret < 0)
		pr_err("%s: spi_register_board_info returned error ret = %d\n", __func__, ret);

	return;
}
#endif

static struct mdm_vddmin_resource mdm_vddmin_rscs = {
	.rpm_id = MSM_RPM_ID_VDDMIN_GPIO,
	.ap2mdm_vddmin_gpio = 30,
	.modes  = 0x03,
	.drive_strength = 8,
	.mdm2ap_vddmin_gpio = 80,
};

static struct gpiomux_setting mdm2ap_status_gpio_run_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct mdm_platform_data mdm_platform_data = {
	.mdm_version = "3.0",
	.ramdump_delay_ms = 2000,
	.early_power_on = 1,
	.sfr_query = 1,
	.vddmin_resource = &mdm_vddmin_rscs,
	.peripheral_platform_device = &apq8064_device_hsic_host,
	.ramdump_timeout_ms = 120000,
	.mdm2ap_status_gpio_run_cfg = &mdm2ap_status_gpio_run_cfg,
};

static struct tsens_platform_data apq_tsens_pdata  = {
		.tsens_factor		= 1000,
		.hw_type		= APQ_8064,
		.tsens_num_sensor	= 11,
		.slope = {1176, 1176, 1154, 1176, 1111,
			1132, 1132, 1199, 1132, 1199, 1132},
};

static struct platform_device msm_tsens_device = {
	.name   = "apq8064-tmu",
	.id = -1,
};

static struct msm_thermal_data msm_thermal_pdata = {
	.sensor_id = 7,
	.poll_ms = 250,
	.limit_temp_degC = 60,
	.temp_hysteresis_degC = 10,
	.freq_step = 2,
};

#define MSM_SHARED_RAM_PHYS 0x80000000
static void __init apq8064_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_apq8064_io();
	if (socinfo_init() < 0)
		pr_err("socinfo_init() failed!\n");
#ifdef CONFIG_SEC_DEBUG
	sec_getlog_supply_meminfo(0x40000000, 0x80000000, 0x00, 0x00);
#endif
}

static void __init apq8064_init_irq(void)
{
	struct msm_mpm_device_data *data = NULL;

#ifdef CONFIG_MSM_MPM
	data = &apq8064_mpm_dev_data;
#endif

	msm_mpm_irq_extn_init(data);
	gic_init(0, GIC_PPI_START, MSM_QGIC_DIST_BASE,
						(void *)MSM_QGIC_CPU_BASE);
}

static struct platform_device msm8064_device_saw_regulator_core0 = {
	.name	= "saw-regulator",
	.id	= 0,
	.dev	= {
		.platform_data = &msm8064_saw_regulator_pdata_8921_s5,
	},
};

static struct platform_device msm8064_device_saw_regulator_core1 = {
	.name	= "saw-regulator",
	.id	= 1,
	.dev	= {
		.platform_data = &msm8064_saw_regulator_pdata_8921_s6,
	},
};

static struct platform_device msm8064_device_saw_regulator_core2 = {
	.name	= "saw-regulator",
	.id	= 2,
	.dev	= {
		.platform_data = &msm8064_saw_regulator_pdata_8821_s0,
	},
};

static struct platform_device msm8064_device_saw_regulator_core3 = {
	.name	= "saw-regulator",
	.id	= 3,
	.dev	= {
		.platform_data = &msm8064_saw_regulator_pdata_8821_s1,

	},
};

static struct msm_rpmrs_level msm_rpmrs_levels[] = {
	{
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		1, 784, 180000, 100,
	},

	{
		MSM_PM_SLEEP_MODE_RETENTION,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		415, 715, 340827, 475,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		1300, 228, 1200000, 2000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(ON, GDHS, MAX, ACTIVE),
		false,
		2000, 138, 1208400, 3200,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(ON, HSFS_OPEN, ACTIVE, RET_HIGH),
		false,
		6000, 119, 1850300, 9000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, GDHS, MAX, ACTIVE),
		false,
		9200, 68, 2839200, 16400,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, MAX, ACTIVE),
		false,
		10300, 63, 3128000, 18200,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, ACTIVE, RET_HIGH),
		false,
		18000, 10, 4602600, 27000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, RET_HIGH, RET_LOW),
		false,
		20000, 2, 5752000, 32000,
	},
};

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_TZ,
};

static struct msm_rpmrs_platform_data msm_rpmrs_data __initdata = {
	.levels = &msm_rpmrs_levels[0],
	.num_levels = ARRAY_SIZE(msm_rpmrs_levels),
	.vdd_mem_levels  = {
		[MSM_RPMRS_VDD_MEM_RET_LOW]	= 750000,
		[MSM_RPMRS_VDD_MEM_RET_HIGH]	= 750000,
		[MSM_RPMRS_VDD_MEM_ACTIVE]	= 1050000,
		[MSM_RPMRS_VDD_MEM_MAX]		= 1150000,
	},
	.vdd_dig_levels = {
		[MSM_RPMRS_VDD_DIG_RET_LOW]	= 500000,
		[MSM_RPMRS_VDD_DIG_RET_HIGH]	= 750000,
		[MSM_RPMRS_VDD_DIG_ACTIVE]	= 950000,
		[MSM_RPMRS_VDD_DIG_MAX]		= 1150000,
	},
	.vdd_mask = 0x7FFFFF,
	.rpmrs_target_id = {
		[MSM_RPMRS_ID_PXO_CLK]		= MSM_RPM_ID_PXO_CLK,
		[MSM_RPMRS_ID_L2_CACHE_CTL]	= MSM_RPM_ID_LAST,
		[MSM_RPMRS_ID_VDD_DIG_0]	= MSM_RPM_ID_PM8921_S3_0,
		[MSM_RPMRS_ID_VDD_DIG_1]	= MSM_RPM_ID_PM8921_S3_1,
		[MSM_RPMRS_ID_VDD_MEM_0]	= MSM_RPM_ID_PM8921_L24_0,
		[MSM_RPMRS_ID_VDD_MEM_1]	= MSM_RPM_ID_PM8921_L24_1,
		[MSM_RPMRS_ID_RPM_CTL]		= MSM_RPM_ID_RPM_CTL,
	},
};

static uint8_t spm_wfi_cmd_sequence[] __initdata = {
	0x03, 0x0f,
};

static uint8_t spm_power_collapse_without_rpm[] __initdata = {
	0x00, 0x24, 0x54, 0x10,
	0x09, 0x03, 0x01,
	0x10, 0x54, 0x30, 0x0C,
	0x24, 0x30, 0x0f,
};

static uint8_t spm_retention_cmd_sequence[] __initdata = {
	0x00, 0x05, 0x03, 0x0D,
	0x0B, 0x00, 0x0f,
};

static uint8_t spm_power_collapse_with_rpm[] __initdata = {
	0x00, 0x24, 0x54, 0x10,
	0x09, 0x07, 0x01, 0x0B,
	0x10, 0x54, 0x30, 0x0C,
	0x24, 0x30, 0x0f,
};

static struct msm_spm_seq_entry msm_spm_boot_cpu_seq_list[] __initdata = {
	[0] = {
		.mode = MSM_SPM_MODE_CLOCK_GATING,
		.notify_rpm = false,
		.cmd = spm_wfi_cmd_sequence,
	},
	[1] = {
		.mode = MSM_SPM_MODE_POWER_RETENTION,
		.notify_rpm = false,
		.cmd = spm_retention_cmd_sequence,
	},
	[2] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = false,
		.cmd = spm_power_collapse_without_rpm,
	},
	[3] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = true,
		.cmd = spm_power_collapse_with_rpm,
	},
};
static struct msm_spm_seq_entry msm_spm_nonboot_cpu_seq_list[] __initdata = {
	[0] = {
		.mode = MSM_SPM_MODE_CLOCK_GATING,
		.notify_rpm = false,
		.cmd = spm_wfi_cmd_sequence,
	},
	[1] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = false,
		.cmd = spm_power_collapse_without_rpm,
	},
	[2] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = true,
		.cmd = spm_power_collapse_with_rpm,
	},
};

static uint8_t l2_spm_wfi_cmd_sequence[] __initdata = {
	0x00, 0x20, 0x03, 0x20,
	0x00, 0x0f,
};

static uint8_t l2_spm_gdhs_cmd_sequence[] __initdata = {
	0x00, 0x20, 0x34, 0x64,
	0x48, 0x07, 0x48, 0x20,
	0x50, 0x64, 0x04, 0x34,
	0x50, 0x0f,
};
static uint8_t l2_spm_power_off_cmd_sequence[] __initdata = {
	0x00, 0x10, 0x34, 0x64,
	0x48, 0x07, 0x48, 0x10,
	0x50, 0x64, 0x04, 0x34,
	0x50, 0x0F,
};

static struct msm_spm_seq_entry msm_spm_l2_seq_list[] __initdata = {
	[0] = {
		.mode = MSM_SPM_L2_MODE_RETENTION,
		.notify_rpm = false,
		.cmd = l2_spm_wfi_cmd_sequence,
	},
	[1] = {
		.mode = MSM_SPM_L2_MODE_GDHS,
		.notify_rpm = true,
		.cmd = l2_spm_gdhs_cmd_sequence,
	},
	[2] = {
		.mode = MSM_SPM_L2_MODE_POWER_COLLAPSE,
		.notify_rpm = true,
		.cmd = l2_spm_power_off_cmd_sequence,
	},
};


static struct msm_spm_platform_data msm_spm_l2_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW_L2_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x02020204,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x00A000AE,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x00A00020,
		.modes = msm_spm_l2_seq_list,
		.num_modes = ARRAY_SIZE(msm_spm_l2_seq_list),
	},
};

static struct msm_spm_platform_data msm_spm_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x03020004,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0084009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x00A4001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_boot_cpu_seq_list),
		.modes = msm_spm_boot_cpu_seq_list,
	},
	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x02020204,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0060009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x0000001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_nonboot_cpu_seq_list),
		.modes = msm_spm_nonboot_cpu_seq_list,
	},
	[2] = {
		.reg_base_addr = MSM_SAW2_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x02020204,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0060009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x0000001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_nonboot_cpu_seq_list),
		.modes = msm_spm_nonboot_cpu_seq_list,
	},
	[3] = {
		.reg_base_addr = MSM_SAW3_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x02020204,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0060009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x0000001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_nonboot_cpu_seq_list),
		.modes = msm_spm_nonboot_cpu_seq_list,
	},
};

static void __init apq8064_init_buses(void)
{
	msm_bus_rpm_set_mt_mask();
	msm_bus_8064_apps_fabric_pdata.rpm_enabled = 1;
	msm_bus_8064_sys_fabric_pdata.rpm_enabled = 1;
	msm_bus_8064_mm_fabric_pdata.rpm_enabled = 1;
	msm_bus_8064_apps_fabric.dev.platform_data =
		&msm_bus_8064_apps_fabric_pdata;
	msm_bus_8064_sys_fabric.dev.platform_data =
		&msm_bus_8064_sys_fabric_pdata;
	msm_bus_8064_mm_fabric.dev.platform_data =
		&msm_bus_8064_mm_fabric_pdata;
	msm_bus_8064_sys_fpb.dev.platform_data = &msm_bus_8064_sys_fpb_pdata;
	msm_bus_8064_cpss_fpb.dev.platform_data = &msm_bus_8064_cpss_fpb_pdata;
}

/* PCIe gpios */
static struct msm_pcie_gpio_info_t msm_pcie_gpio_info[MSM_PCIE_MAX_GPIO] = {
	{"rst_n", PM8921_MPP_PM_TO_SYS(PCIE_RST_N_PMIC_MPP), 0},
	{"pwr_en", PM8921_GPIO_PM_TO_SYS(PCIE_PWR_EN_PMIC_GPIO), 1},
};

static struct msm_pcie_platform msm_pcie_platform_data = {
	.gpio = msm_pcie_gpio_info,
	.axi_addr = PCIE_AXI_BAR_PHYS,
	.axi_size = PCIE_AXI_BAR_SIZE,
	.wake_n = PM8921_GPIO_IRQ(PM8921_IRQ_BASE, PCIE_WAKE_N_PMIC_GPIO),
};

static int __init mpq8064_pcie_enabled(void)
{
	return !((readl_relaxed(QFPROM_RAW_FEAT_CONFIG_ROW0_MSB) & BIT(21)) ||
		(readl_relaxed(QFPROM_RAW_OEM_CONFIG_ROW0_LSB) & BIT(4)));
}

static void __init mpq8064_pcie_init(void)
{
	if (mpq8064_pcie_enabled()) {
		msm_device_pcie.dev.platform_data = &msm_pcie_platform_data;
		platform_device_register(&msm_device_pcie);
	}
}

static struct platform_device apq8064_device_ext_5v_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= PM8921_MPP_PM_TO_SYS(7),
	.dev	= {
		.platform_data
			= &apq8064_gpio_regulator_pdata[GPIO_VREG_ID_EXT_5V],
	},
};

static struct platform_device apq8064_device_ext_mpp8_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= PM8921_MPP_PM_TO_SYS(8),
	.dev	= {
		.platform_data
			= &apq8064_gpio_regulator_pdata[GPIO_VREG_ID_EXT_MPP8],
	},
};

static struct platform_device apq8064_device_ext_3p3v_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= APQ8064_EXT_3P3V_REG_EN_GPIO,
	.dev	= {
		.platform_data =
			&apq8064_gpio_regulator_pdata[GPIO_VREG_ID_EXT_3P3V],
	},
};

static struct platform_device apq8064_device_ext_ts_sw_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= PM8921_GPIO_PM_TO_SYS(23),
	.dev	= {
		.platform_data
			= &apq8064_gpio_regulator_pdata[GPIO_VREG_ID_EXT_TS_SW],
	},
};

static struct platform_device apq8064_device_rpm_regulator __devinitdata = {
	.name	= "rpm-regulator",
	.id	= 0,
	.dev	= {
		.platform_data = &apq8064_rpm_regulator_pdata,
	},
};

static struct platform_device
apq8064_pm8921_device_rpm_regulator __devinitdata = {
	.name	= "rpm-regulator",
	.id	= 1,
	.dev	= {
		.platform_data = &apq8064_rpm_regulator_pm8921_pdata,
	},
};

static struct gpio_ir_recv_platform_data gpio_ir_recv_pdata = {
	.gpio_nr = 88,
	.active_low = 1,
};

static struct platform_device gpio_ir_recv_pdev = {
	.name = "gpio-rc-recv",
	.dev = {
		.platform_data = &gpio_ir_recv_pdata,
	},
};

static struct sec_jack_zone jack_zones[] = {
	[0] = {
		.adc_high	= 3,
		.delay_us	= 10000,
		.check_count	= 10,
		.jack_type	= SEC_HEADSET_3POLE,
	},
	[1] = {
		.adc_high	= 690,
		.delay_us	= 10000,
		.check_count	= 10,
		.jack_type	= SEC_HEADSET_3POLE,
	},
	[2] = {
		.adc_high	= 1679,
		.delay_us	= 10000,
		.check_count	= 10,
		.jack_type	= SEC_HEADSET_4POLE,
	},
	[3] = {
		.adc_high	= 9999,
		.delay_us	= 10000,
		.check_count	= 10,
		.jack_type	= SEC_HEADSET_4POLE,
	},
};

static struct sec_jack_zone jack_zones_rev7[] = {
	[0] = {
		.adc_high	= 3,
		.delay_us	= 10000,
		.check_count	= 10,
		.jack_type	= SEC_HEADSET_3POLE,
	},
	[1] = {
		.adc_high	= 690,
		.delay_us	= 10000,
		.check_count	= 10,
		.jack_type	= SEC_HEADSET_3POLE,
	},
	[2] = {
		.adc_high	= 1679,
		.delay_us	= 10000,
		.check_count	= 10,
		.jack_type	= SEC_HEADSET_4POLE,
	},
	[3] = {
		.adc_high	= 9999,
		.delay_us	= 10000,
		.check_count	= 10,
		.jack_type	= SEC_HEADSET_4POLE,
	},
};

/* To support 3-buttons earjack */
static struct sec_jack_buttons_zone jack_buttons_zones[] = {
	{
		.code		= KEY_MEDIA,
		.adc_low	= 0,
		.adc_high	= 225,
	},
	{
		.code		= KEY_VOLUMEUP,
		.adc_low	= 226,
		.adc_high	= 317,
	},
	{
		.code		= KEY_VOLUMEDOWN,
		.adc_low	= 318,
		.adc_high	= 505,
	},
};

static struct sec_jack_buttons_zone jack_buttons_zones_rev7[] = {
	{
		.code		= KEY_MEDIA,
		.adc_low	= 0,
		.adc_high	= 225,
	},
	{
		.code		= KEY_VOLUMEUP,
		.adc_low	= 226,
		.adc_high	= 317,
	},
	{
		.code		= KEY_VOLUMEDOWN,
		.adc_low	= 318,
		.adc_high	= 505,
	},
};

#if defined(CONFIG_SAMSUNG_JACK_GNDLDET)
static int get_sec_l_jack_state(void)
{
	if (system_rev >= BOARD_REV07)
		return (gpio_get_value_cansleep(PM8921_GPIO_PM_TO_SYS(GPIO_EAR_GND_DET)) ^ 1);
	else
		return 1;
}
#endif

static void set_sec_micbias_state(bool state)
{
	int retry = 0;

	do {
		if(gpio_get_value(GPIO_FPGA_CDONE)) {
			ice_gpiox_set(FPGA_GPIO_EAR_MICBIAS_EN, state);

			pr_info("%s: done\n", __func__);
			
			break;
		}
		else {
			++retry;

			pr_info("%s: failed becasue of FPGA unset. try count = %d\n", __func__, retry);

			usleep(1000000);
		}
	}while(retry < 15);
}

static int sec_jack_get_adc_value(void)
{
	int rc = 0;
	int retVal = 0;

	int retry = 0;

	struct pm8xxx_adc_chan_result result;

	do {
		if(gpio_get_value(GPIO_FPGA_CDONE)) {
			rc = pm8xxx_adc_mpp_config_read(
					PM8XXX_AMUX_MPP_3,
					ADC_MPP_1_AMUX6_SCALE_DEFAULT,
					&result);
			if (rc) {
				pr_err("%s : error reading mpp %d, rc = %d\n",
					__func__, PM8XXX_AMUX_MPP_3, rc);
				return rc;
			}
			retVal = ((int)result.physical)/1000;

			pr_info("%s: done\n", __func__);

			break;
		}
		else {
			++retry;

			pr_info("%s: failed becasue of FPGA unset. try count = %d\n", __func__, retry);

			usleep(1000000);
		}
	}while(retry < 15);

	return retVal;
}

static struct sec_jack_platform_data sec_jack_data = {
#if defined(CONFIG_SAMSUNG_JACK_GNDLDET)
	.get_l_jack_state	= get_sec_l_jack_state,
#endif
	.set_micbias_state	= set_sec_micbias_state,
	.get_adc_value		= sec_jack_get_adc_value,
	.zones			= jack_zones,
	.num_zones		= ARRAY_SIZE(jack_zones),
	.buttons_zones		= jack_buttons_zones,
	.num_buttons_zones	= ARRAY_SIZE(jack_buttons_zones),
	.det_gpio		= GPIO_EAR_DET,
	.send_end_gpio		= GPIO_SHORT_SENDEND,
	.send_end_active_high	= false,
};

static struct platform_device sec_device_jack = {
	.name           = "sec_jack",
	.id             = -1,
	.dev            = {
		.platform_data  = &sec_jack_data,
	},
};
#ifdef CONFIG_VIBETONZ
static struct vibrator_platform_data msm_8064_vibrator_pdata = {
	.vib_model = HAPTIC_PWM,
	.vib_pwm_gpio = GPIO_VIB_PWM,
	.is_pmic_vib_en = 0,
	.is_pmic_haptic_pwr_en = 0,
	.is_pmic_vib_pwm = 0,
#ifdef CONFIG_MOTOR_DRV_MAX77693
	.power_onoff = max77693_haptic_power_onoff,
#else
	.power_onoff = NULL,
#endif
};
static struct platform_device vibetonz_device = {
	.name = "tspdrv",
	.id = -1,
	.dev = {
		.platform_data = &msm_8064_vibrator_pdata,
	},
};
#endif /* CONFIG_VIBETONZ */
#ifdef CONFIG_SENSORS_SSP_C12SD
static int uv_get_adc_value(void)
{
	int rc = 0;
	int retVal = 0;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_mpp_config_read(
		PM8XXX_AMUX_MPP_1,
		ADC_MPP_2_AMUX6,
		&result);
	if (rc) {
		pr_err("%s : error reading mpp %d, rc = %d\n",
			__func__, PM8XXX_AMUX_MPP_1, rc);
		return rc;
	}
	retVal = (int)result.measurement;

	return retVal;
}

static struct regulator *uv_vcc_1p8;
static void uv_opamp_power_on(int onoff_l29)
{
	int ret;

	if (uv_vcc_1p8 == NULL) {
		uv_vcc_1p8 = regulator_get(NULL, "8921_l29");
		if (IS_ERR(uv_vcc_1p8))
			return ;

		ret = regulator_set_voltage(uv_vcc_1p8, 1800000, 1800000);
		if (ret)
			pr_err("%s: error uv_vcc_1p8 setting voltage ret=%d\n",
				__func__, ret);
	}

	if (onoff_l29 == SNS_PWR_ON) {
		ret = regulator_enable(uv_vcc_1p8);
		if (ret)
			pr_err("%s: error enabling regulator\n", __func__);
		else
			pr_err("%s: uv_vcc_1p8 enabled\n", __func__);
	} else if ((onoff_l29 == SNS_PWR_OFF)) {
		if (regulator_is_enabled(uv_vcc_1p8)) {
			ret = regulator_disable(uv_vcc_1p8);
			if (ret)
				pr_err("%s: error uv_vcc_1p8 enabling regulator\n",
				__func__);
			else
				pr_err("%s: uv_vcc_1p8 disabled\n", __func__);
		}
	}
}

static struct uv_platform_data uv_data = {
	.get_adc_value = uv_get_adc_value,
	.power_on = (void *) uv_opamp_power_on,
};

static struct platform_device uv_device = {
	.name           = "uv_sensor",
	.id             = -1,
	.dev            = {
		.platform_data  = &uv_data,
	},
};
#endif
static struct platform_device *common_not_mpq_devices[] __initdata = {
	&apq8064_device_qup_i2c_gsbi1,
	&apq8064_device_qup_i2c_gsbi3,

#ifndef CONFIG_ISDBTMM
	&apq8064_device_qup_i2c_gsbi4,
#endif
};

static struct platform_device *early_common_devices[] __initdata = {
	&apq8064_device_acpuclk,
	&apq8064_device_dmov,
	&apq8064_device_qup_spi_gsbi5,
};

static struct platform_device *pm8921_common_devices[] __initdata = {
	&apq8064_device_ext_5v_vreg,
	&apq8064_device_ext_mpp8_vreg,
	&apq8064_device_ext_3p3v_vreg,
	&apq8064_device_ssbi_pmic1,
	&apq8064_device_ssbi_pmic2,
	&apq8064_device_ext_ts_sw_vreg,
};

static struct platform_device *pm8917_common_devices[] __initdata = {
	&apq8064_device_ext_mpp8_vreg,
	&apq8064_device_ext_3p3v_vreg,
	&apq8064_device_ssbi_pmic1,
	&apq8064_device_ssbi_pmic2,
	&apq8064_device_ext_ts_sw_vreg,
};

static struct platform_device *common_devices[] __initdata = {
	&msm_device_smd_apq8064,
#ifdef CONFIG_MFD_MAX77693
	&max77693_i2c_gpio_device,
#endif
	&apq8064_device_otg,
	&apq8064_device_gadget_peripheral,
	&apq8064_device_hsusb_host,
	&android_usb_device,
	&msm_device_wcnss_wlan,
	&msm_device_iris_fm,
	&apq8064_fmem_device,
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	&apq8064_android_pmem_device,
	&apq8064_android_pmem_adsp_device,
	&apq8064_android_pmem_audio_device,
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/
#ifdef CONFIG_ION_MSM
	&apq8064_ion_dev,
#endif
	&msm8064_device_watchdog,
	&msm8064_device_saw_regulator_core0,
	&msm8064_device_saw_regulator_core1,
	&msm8064_device_saw_regulator_core2,
	&msm8064_device_saw_regulator_core3,
#if defined(CONFIG_QSEECOM)
	&qseecom_device,
#endif

	&msm_8064_device_tsif[0],
	&msm_8064_device_tsif[1],

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif

#ifdef CONFIG_HW_RANDOM_MSM
	&apq8064_device_rng,
#endif
	&apq_pcm,
	&apq_pcm_routing,
	&apq_cpudai0,
	&apq_cpudai1,
	&mpq_cpudai_sec_i2s_rx,
	&mpq_cpudai_mi2s_tx,
	&apq_cpudai_hdmi_rx,
	&apq_cpudai_bt_rx,
	&apq_cpudai_bt_tx,
	&apq_cpudai_fm_rx,
	&apq_cpudai_fm_tx,
	&apq_cpu_fe,
	&apq_stub_codec,
	&apq_voice,
	&apq_voip,
	&apq_lpa_pcm,
	&apq_compr_dsp,
	&apq_multi_ch_pcm,
	&apq_lowlatency_pcm,
	&apq_pcm_hostless,
	&apq_cpudai_afe_01_rx,
	&apq_cpudai_afe_01_tx,
	&apq_cpudai_afe_02_rx,
	&apq_cpudai_afe_02_tx,
	&apq_pcm_afe,
	&apq_cpudai_auxpcm_rx,
	&apq_cpudai_auxpcm_tx,
	&apq_cpudai_stub,
	&apq_cpudai_slimbus_1_rx,
	&apq_cpudai_slimbus_1_tx,
	&apq_cpudai_slimbus_2_rx,
	&apq_cpudai_slimbus_2_tx,
	&apq_cpudai_slimbus_3_rx,
	&apq_cpudai_slimbus_3_tx,
	&apq8064_rpm_device,
	&apq8064_rpm_log_device,
	&apq8064_rpm_stat_device,
	&apq_device_tz_log,
	&msm_bus_8064_apps_fabric,
	&msm_bus_8064_sys_fabric,
	&msm_bus_8064_mm_fabric,
	&msm_bus_8064_sys_fpb,
	&msm_bus_8064_cpss_fpb,
	&apq8064_msm_device_vidc,
	&msm_pil_dsps,
	&msm_8960_q6_lpass,
	&msm_pil_vidc,
	&msm_gss,
	&apq8064_rtb_device,
	&apq8064_dcvs_device,
	&apq8064_msm_gov_device,
	&apq8064_device_cache_erp,
	&msm8960_device_ebi1_ch0_erp,
	&msm8960_device_ebi1_ch1_erp,
	&epm_adc_device,
	&coresight_tpiu_device,
#ifdef CONFIG_CAMERA_SW_I2C
	&s5k6b2yx_i2c_gpio_device,
#endif
	&apq8064_device_qup_i2c_gsbi7,
	&apq8064_device_qup_i2c_gsbi2,
	&coresight_etb_device,
	&apq8064_coresight_funnel_device,
	&coresight_etm0_device,
	&coresight_etm1_device,
	&coresight_etm2_device,
	&coresight_etm3_device,
	&apq_cpudai_slim_4_rx,
	&apq_cpudai_slim_4_tx,
#ifdef CONFIG_MSM_GEMINI
	&msm8960_gemini_device,
#endif
	&apq8064_iommu_domain_device,
	&msm_tsens_device,
	&apq8064_cache_dump_device,
	&msm_8064_device_tspp,
#ifdef CONFIG_SEC_FPGA
	&barcode_i2c_gpio_device,
#endif
#ifdef CONFIG_BATTERY_BCL
	&battery_bcl_device,
#endif
#ifdef CONFIG_VIBETONZ
	&vibetonz_device,
#endif /* CONFIG_VIBETONZ */
	&apq8064_msm_mpd_device,
#ifdef CONFIG_BCM2079X_NFC_I2C
#ifdef NFC_SW_I2C
	&bcm2079x_i2c_gpio_device,
#endif
#endif
#if defined(CONFIG_VIDEO_MHL_V2) || defined(CONFIG_LEDS_AN30259A)
	&shared_i2c_gpio_device,
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ram_console_device,
#endif

#ifdef CONFIG_ISDBTMM
	&apq8064_device_qup_spi_gsbi4,
	&smtej113_device_i2c25,
#endif

};

static struct platform_device *cdp_devices[] __initdata = {
	&apq8064_device_uart_gsbi1,
	&apq8064_device_uart_gsbi7,
#ifdef CONFIG_FELICA
	&apq8064_device_uart_gsbi3,
#endif /* CONFIG_FELICA */
	&msm_device_sps_apq8064,
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm8064_pc_cntr,
	&sec_device_jack,
#ifdef CONFIG_SENSORS_SSP_C12SD
	&uv_device,
#endif
#ifdef CONFIG_SEC_THERMISTOR
	&sec_device_thermistor,
#endif
};

static struct platform_device
mpq8064_device_ext_1p2_buck_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= SX150X_GPIO(4, 2),
	.dev	= {
		.platform_data =
		 &mpq8064_gpio_regulator_pdata[GPIO_VREG_ID_AVC_1P2V],
	},
};

static struct platform_device
mpq8064_device_ext_1p8_buck_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= SX150X_GPIO(4, 4),
	.dev	= {
		.platform_data =
		&mpq8064_gpio_regulator_pdata[GPIO_VREG_ID_AVC_1P8V],
	},
};

static struct platform_device
mpq8064_device_ext_2p2_buck_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= SX150X_GPIO(4, 14),
	.dev	= {
		.platform_data =
		&mpq8064_gpio_regulator_pdata[GPIO_VREG_ID_AVC_2P2V],
	},
};

static struct platform_device
mpq8064_device_ext_5v_buck_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= SX150X_GPIO(4, 3),
	.dev	= {
		.platform_data =
		 &mpq8064_gpio_regulator_pdata[GPIO_VREG_ID_AVC_5V],
	},
};

static struct platform_device
mpq8064_device_ext_3p3v_ldo_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= SX150X_GPIO(4, 15),
	.dev	= {
		.platform_data =
		&mpq8064_gpio_regulator_pdata[GPIO_VREG_ID_AVC_3P3V],
	},
};

static struct platform_device rc_input_loopback_pdev = {
	.name	= "rc-user-input",
	.id	= -1,
};

static int rf4ce_gpio_init(void)
{
	if (!machine_is_mpq8064_cdp() &&
		!machine_is_mpq8064_hrd() &&
			!machine_is_mpq8064_dtv())
		return -EINVAL;

	/* CC2533 SRDY Input */
	if (!gpio_request(SX150X_GPIO(4, 6), "rf4ce_srdy")) {
		gpio_direction_input(SX150X_GPIO(4, 6));
		gpio_export(SX150X_GPIO(4, 6), true);
	}

	/* CC2533 MRDY Output */
	if (!gpio_request(SX150X_GPIO(4, 5), "rf4ce_mrdy")) {
		gpio_direction_output(SX150X_GPIO(4, 5), 1);
		gpio_export(SX150X_GPIO(4, 5), true);
	}

	/* CC2533 Reset Output */
	if (!gpio_request(SX150X_GPIO(4, 7), "rf4ce_reset")) {
		gpio_direction_output(SX150X_GPIO(4, 7), 0);
		gpio_export(SX150X_GPIO(4, 7), true);
	}

	return 0;
}
late_initcall(rf4ce_gpio_init);

static struct platform_device *mpq_devices[] __initdata = {
	&msm_device_sps_apq8064,
	&mpq8064_device_qup_i2c_gsbi5,
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&gpio_ir_recv_pdev,
	&mpq8064_device_ext_1p2_buck_vreg,
	&mpq8064_device_ext_1p8_buck_vreg,
	&mpq8064_device_ext_2p2_buck_vreg,
	&mpq8064_device_ext_5v_buck_vreg,
	&mpq8064_device_ext_3p3v_ldo_vreg,
#ifdef CONFIG_MSM_VCAP
	&msm8064_device_vcap,
#endif
	&rc_input_loopback_pdev,
};

static struct msm_spi_platform_data apq8064_qup_spi_gsbi5_pdata = {
	.max_clock_speed = 15060000,
};

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
#define KS8851_IRQ_GPIO		43

static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias               = "ks8851",
		.irq                    = MSM_GPIO_TO_INT(KS8851_IRQ_GPIO),
		.max_speed_hz           = 19200000,
		.bus_num                = 0,
		.chip_select            = 2,
		.mode                   = SPI_MODE_0,
	},
	{
		.modalias		= "epm_adc",
		.max_speed_hz		= 1100000,
		.bus_num		= 0,
		.chip_select		= 3,
		.mode			= SPI_MODE_0,
	},
};
#endif

#ifdef CONFIG_SND_SOC_ES325
static int  es325_enable_VDD_CORE(void)
{
	static struct regulator *l18;
	int ret;

	l18 = regulator_get(NULL, "8921_l18");
	if (IS_ERR(l18)) {
		pr_err("%s: error regulator_get\n", __func__);
		return -1;
	}
	ret = regulator_set_voltage(l18, 1100000, 1100000);
	if (ret)
		pr_err("%s: error set voltage ret=%d\n", __func__, ret);
	ret = regulator_enable(l18);
	if (ret) {
		pr_err("%s: error enable l18 ret=%d\n", __func__, ret);
		return -1;
	}
	return 0;
}

static void es325_2MIC_RST_init(void)
{
	struct pm_gpio es325_2MIC_RST = {
		.direction  = PM_GPIO_DIR_OUT,
		.pull   = PM_GPIO_PULL_NO,
		.out_strength  = PM_GPIO_STRENGTH_HIGH,
		.function  = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol  = 0,
		.vin_sel  = PM_GPIO_VIN_S4,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value  = 0,
	};

	struct pm_gpio es325_2MIC_WAKE = {
		.direction  = PM_GPIO_DIR_OUT,
		.pull   = PM_GPIO_PULL_NO,
		.out_strength  = PM_GPIO_STRENGTH_HIGH,
		.function  = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol  = 0,
		.vin_sel  = PM_GPIO_VIN_S4,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value  = 0,
	};

	 pm8xxx_gpio_config(
		PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_2MIC_RST), &es325_2MIC_RST);

	gpio_set_value_cansleep(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_2MIC_RST), 1);

	 pm8xxx_gpio_config(
		PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_2MIC_WAKE), &es325_2MIC_WAKE);

	gpio_set_value_cansleep(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_2MIC_WAKE), 1);
}

void es325_start_api(void)
{
	es325_enable_VDD_CORE();
	mdelay(50);
	es325_2MIC_RST_init();
	pr_info("=[ES325]=%s=====\n",__func__);
}

static struct platform_device msm_es325_mclk_dev = {
	.name = "es325_mclk_dev_pdev",
	.id = -1,
	.dev = {
		.init_name = "es325_mclk_dev",
	},
};

int es325_enable_ext_clk(int enable)
{
	int r = 0;
	static struct clk *es325_codec_clk;
	pr_info("%s: enable=%d\n", __func__, enable);

	if (!es325_codec_clk) {
		pr_info("%s: Clk_Get osr_clk\n", __func__);
		es325_codec_clk = clk_get(&msm_es325_mclk_dev.dev, "osr_clk");
	}

	if (enable) {
		clk_set_rate(es325_codec_clk, 12288000);
		clk_prepare_enable(es325_codec_clk);
	} else {
		clk_disable_unprepare(es325_codec_clk);
		clk_put(es325_codec_clk);
		es325_codec_clk  = NULL;
	}

	return r;
}

static struct esxxx_platform_data esxxx_platform_data = {
	.reset_gpio	= PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_2MIC_RST),
	.wakeup_gpio	= PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_2MIC_WAKE),
	.gpioa_gpio	= 2,
	.es325_clk_cb	= es325_enable_ext_clk,
	.gpiob_gpio	= 3
};

static struct slim_device es325_slim_intf = {
	.name		= "es325-codec-intf",
	.e_addr		= {0, 0, 0x2, 0, 0xbe, 2 },
	.laddr = 0xf0,
	.dev = {
		.platform_data = &esxxx_platform_data,
	},
};

static struct slim_device es325_slim_gen0 = {
	.name		= "es325-codec-gen0",
	.e_addr		= { 0, 1, 2, 0, 0xbe, 2 },
	.laddr = 0xf0,
	.dev = {
		.platform_data = &esxxx_platform_data,
	},
};
#endif

static struct slim_boardinfo apq8064_slim_devices[] = {
	{
		.bus_num = 1,
		.slim_slave = &apq8064_slim_tabla,
	},
	{
		.bus_num = 1,
		.slim_slave = &apq8064_slim_tabla20,
	},
#ifdef CONFIG_SND_SOC_ES325
	{
		.bus_num = 1,
		.slim_slave = &es325_slim_intf,
	},
	{
		.bus_num = 1,
		.slim_slave = &es325_slim_gen0,
	},
#endif
	/* add more slimbus slaves as needed */
};

static struct msm_i2c_platform_data apq8064_i2c_qup_gsbi1_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
};

static struct msm_i2c_platform_data apq8064_i2c_qup_gsbi3_pdata = {
	.clk_freq = 384000,
	.src_clk_rate = 24000000,
};

#ifndef CONFIG_ISDBTMM
static struct msm_i2c_platform_data apq8064_i2c_qup_gsbi4_pdata = {
	.clk_freq = 400000,
	.src_clk_rate = 24000000,
};
#endif
static struct msm_i2c_platform_data mpq8064_i2c_qup_gsbi5_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
};

static struct msm_i2c_platform_data apq8064_i2c_qup_gsbi2_pdata = {
	.clk_freq = 375000,
	.src_clk_rate = 24000000,
};

static struct msm_i2c_platform_data apq8064_i2c_qup_gsbi7_pdata = {
	.clk_freq = 384000,
	.src_clk_rate = 24000000,
	.use_gsbi_shared_mode = 1,
};

#define GSBI_DUAL_MODE_CODE 0x60
#define MSM_GSBI1_PHYS		0x12440000
static void __init apq8064_i2c_init(void)
{
	void __iomem *gsbi_mem;

	apq8064_device_qup_i2c_gsbi1.dev.platform_data =
					&apq8064_i2c_qup_gsbi1_pdata;
	gsbi_mem = ioremap_nocache(MSM_GSBI1_PHYS, 4);
	writel_relaxed(GSBI_DUAL_MODE_CODE, gsbi_mem);
	/* Ensure protocol code is written before proceeding */
	wmb();
	iounmap(gsbi_mem);
	apq8064_i2c_qup_gsbi1_pdata.use_gsbi_shared_mode = 1;
	apq8064_device_qup_i2c_gsbi3.dev.platform_data =
					&apq8064_i2c_qup_gsbi3_pdata;
	apq8064_device_qup_i2c_gsbi1.dev.platform_data =
					&apq8064_i2c_qup_gsbi1_pdata;
#ifndef CONFIG_ISDBTMM
	apq8064_device_qup_i2c_gsbi4.dev.platform_data =
					&apq8064_i2c_qup_gsbi4_pdata;
#endif
	mpq8064_device_qup_i2c_gsbi5.dev.platform_data =
					&mpq8064_i2c_qup_gsbi5_pdata;
	apq8064_device_qup_i2c_gsbi2.dev.platform_data =
					&apq8064_i2c_qup_gsbi2_pdata;
	apq8064_device_qup_i2c_gsbi7.dev.platform_data =
					&apq8064_i2c_qup_gsbi7_pdata;
}

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
static int ethernet_init(void)
{
	int ret;
	ret = gpio_request(KS8851_IRQ_GPIO, "ks8851_irq");
	if (ret) {
		pr_err("ks8851 gpio_request failed: %d\n", ret);
		goto fail;
	}

	return 0;
fail:
	return ret;
}
#else
static int ethernet_init(void)
{
	return 0;
}
#endif

#ifdef CONFIG_MACH_JF
#define GPIO_KEY_VOLUME_UP	PM8921_GPIO_PM_TO_SYS(35)
#define GPIO_KEY_VOLUME_DOWN	PM8921_GPIO_PM_TO_SYS(37)


static struct gpio_keys_button gpio_keys_button[] = {
	{
		.code           = KEY_VOLUMEUP,
		.gpio           = GPIO_KEY_VOLUME_UP,
		.desc           = "volume_up_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 0,
		.debounce_interval = 5,
	},
	{
		.code           = KEY_VOLUMEDOWN,
		.gpio           = GPIO_KEY_VOLUME_DOWN,
		.desc           = "volume_down_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 0,
		.debounce_interval = 5,
	},
	{
		.code           = KEY_HOMEPAGE,
		.gpio           = GPIO_KEY_HOME,
		.desc           = "home_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 5,
	},
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons        = gpio_keys_button,
	.nbuttons       = ARRAY_SIZE(gpio_keys_button),
#ifdef CONFIG_SENSORS_HALL
	.gpio_flip_cover = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_HALL_SENSOR_INT),
#endif
};

static struct platform_device gpio_kp_pdev = {
	.name           = "gpio-keys",
	.id             = -1,
	.dev            = {
		.platform_data  = &gpio_keys_data,
	},
};

#else

#define GPIO_KEY_HOME		PM8921_GPIO_PM_TO_SYS(27)
#define GPIO_KEY_VOLUME_UP	PM8921_GPIO_PM_TO_SYS(35)
#define GPIO_KEY_VOLUME_DOWN_PM8921	PM8921_GPIO_PM_TO_SYS(38)
#define GPIO_KEY_VOLUME_DOWN_PM8917	PM8921_GPIO_PM_TO_SYS(30)
#define GPIO_KEY_CAM_FOCUS		PM8921_GPIO_PM_TO_SYS(3)
#define GPIO_KEY_CAM_SNAP		PM8921_GPIO_PM_TO_SYS(4)
#define GPIO_KEY_ROTATION_PM8921	PM8921_GPIO_PM_TO_SYS(42)
#define GPIO_KEY_ROTATION_PM8917	PM8921_GPIO_PM_TO_SYS(8)

static struct gpio_keys_button cdp_keys_pm8921[] = {
	{
		.code           = KEY_HOME,
		.gpio           = GPIO_KEY_HOME,
		.desc           = "home_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
	{
		.code           = KEY_VOLUMEUP,
		.gpio           = GPIO_KEY_VOLUME_UP,
		.desc           = "volume_up_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
	{
		.code           = KEY_VOLUMEDOWN,
		.gpio           = GPIO_KEY_VOLUME_DOWN_PM8921,
		.desc           = "volume_down_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
	{
		.code           = SW_ROTATE_LOCK,
		.gpio           = GPIO_KEY_ROTATION_PM8921,
		.desc           = "rotate_key",
		.active_low     = 1,
		.type		= EV_SW,
		.debounce_interval = 15,
	},
};

static struct gpio_keys_button cdp_keys_pm8917[] = {
	{
		.code           = KEY_HOME,
		.gpio           = GPIO_KEY_HOME,
		.desc           = "home_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
	{
		.code           = KEY_VOLUMEUP,
		.gpio           = GPIO_KEY_VOLUME_UP,
		.desc           = "volume_up_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
	{
		.code           = KEY_VOLUMEDOWN,
		.gpio           = GPIO_KEY_VOLUME_DOWN_PM8917,
		.desc           = "volume_down_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
	{
		.code           = SW_ROTATE_LOCK,
		.gpio           = GPIO_KEY_ROTATION_PM8917,
		.desc           = "rotate_key",
		.active_low     = 1,
		.type		= EV_SW,
		.debounce_interval = 15,
	},
};

static struct gpio_keys_platform_data cdp_keys_data = {
	.buttons        = cdp_keys_pm8921,
	.nbuttons       = ARRAY_SIZE(cdp_keys_pm8921),
};

static struct platform_device cdp_kp_pdev = {
	.name           = "gpio-keys",
	.id             = -1,
	.dev            = {
		.platform_data  = &cdp_keys_data,
	},
};

static struct gpio_keys_button mtp_keys[] = {
	{
		.code           = KEY_CAMERA_FOCUS,
		.gpio           = GPIO_KEY_CAM_FOCUS,
		.desc           = "cam_focus_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
	{
		.code           = KEY_VOLUMEUP,
		.gpio           = GPIO_KEY_VOLUME_UP,
		.desc           = "volume_up_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
	{
		.code           = KEY_VOLUMEDOWN,
		.gpio           = GPIO_KEY_VOLUME_DOWN_PM8921,
		.desc           = "volume_down_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
	{
		.code           = KEY_CAMERA_SNAPSHOT,
		.gpio           = GPIO_KEY_CAM_SNAP,
		.desc           = "cam_snap_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.debounce_interval = 15,
	},
};

static struct gpio_keys_platform_data mtp_keys_data = {
	.buttons        = mtp_keys,
	.nbuttons       = ARRAY_SIZE(mtp_keys),
};

static struct platform_device mtp_kp_pdev = {
	.name           = "gpio-keys",
	.id             = -1,
	.dev            = {
		.platform_data  = &mtp_keys_data,
	},
};

static struct gpio_keys_button mpq_keys[] = {
	{
		.code           = KEY_VOLUMEDOWN,
		.gpio           = GPIO_KEY_VOLUME_DOWN_PM8921,
		.desc           = "volume_down_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
	{
		.code           = KEY_VOLUMEUP,
		.gpio           = GPIO_KEY_VOLUME_UP,
		.desc           = "volume_up_key",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
};

static struct gpio_keys_platform_data mpq_keys_data = {
	.buttons        = mpq_keys,
	.nbuttons       = ARRAY_SIZE(mpq_keys),
};

static struct platform_device mpq_gpio_keys_pdev = {
	.name           = "gpio-keys",
	.id             = -1,
	.dev            = {
		.platform_data  = &mpq_keys_data,
	},
};

#define MPQ_KP_ROW_BASE		SX150X_EXP2_GPIO_BASE
#define MPQ_KP_COL_BASE		(SX150X_EXP2_GPIO_BASE + 4)

static unsigned int mpq_row_gpios[] = {MPQ_KP_ROW_BASE, MPQ_KP_ROW_BASE + 1,
				MPQ_KP_ROW_BASE + 2, MPQ_KP_ROW_BASE + 3};
static unsigned int mpq_col_gpios[] = {MPQ_KP_COL_BASE, MPQ_KP_COL_BASE + 1,
				MPQ_KP_COL_BASE + 2};

static const unsigned int mpq_keymap[] = {
	KEY(0, 0, KEY_UP),
	KEY(0, 1, KEY_ENTER),
	KEY(0, 2, KEY_3),

	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_EXIT),
	KEY(1, 2, KEY_4),

	KEY(2, 0, KEY_LEFT),
	KEY(2, 1, KEY_1),
	KEY(2, 2, KEY_5),

	KEY(3, 0, KEY_RIGHT),
	KEY(3, 1, KEY_2),
	KEY(3, 2, KEY_6),
};

static struct matrix_keymap_data mpq_keymap_data = {
	.keymap_size	= ARRAY_SIZE(mpq_keymap),
	.keymap		= mpq_keymap,
};

static struct matrix_keypad_platform_data mpq_keypad_data = {
	.keymap_data		= &mpq_keymap_data,
	.row_gpios		= mpq_row_gpios,
	.col_gpios		= mpq_col_gpios,
	.num_row_gpios		= ARRAY_SIZE(mpq_row_gpios),
	.num_col_gpios		= ARRAY_SIZE(mpq_col_gpios),
	.col_scan_delay_us	= 32000,
	.debounce_ms		= 20,
	.wakeup			= 1,
	.active_low		= 1,
	.no_autorepeat		= 1,
};

static struct platform_device mpq_keypad_device = {
	.name           = "matrix-keypad",
	.id             = -1,
	.dev            = {
		.platform_data  = &mpq_keypad_data,
	},
};
#endif

/* Sensors DSPS platform data */
#define DSPS_PIL_GENERIC_NAME		"dsps"
static void __init apq8064_init_dsps(void)
{
	struct msm_dsps_platform_data *pdata =
		msm_dsps_device_8064.dev.platform_data;
	pdata->pil_name = DSPS_PIL_GENERIC_NAME;
	pdata->gpios = NULL;
	pdata->gpios_num = 0;

	platform_device_register(&msm_dsps_device_8064);
}

#define I2C_SURF 1
#define I2C_FFA  (1 << 1)
#define I2C_RUMI (1 << 2)
#define I2C_SIM  (1 << 3)
#define I2C_LIQUID (1 << 4)
#ifdef CONFIG_KEYBOARD_CYPRESS_TOUCH_236
#define I2C_FLUID (1 << 4)
#endif
#define I2C_MPQ_CDP	BIT(5)
#define I2C_MPQ_HRD	BIT(6)
#define I2C_MPQ_DTV	BIT(7)

struct i2c_registry {
	u8                     machs;
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

static struct i2c_registry apq8064_i2c_devices[] __initdata = {
	{
		I2C_LIQUID,
		APQ_8064_GSBI1_QUP_I2C_BUS_ID,
		smb349_charger_i2c_info,
		ARRAY_SIZE(smb349_charger_i2c_info)
	},
#ifdef CONFIG_HAPTIC_ISA1200
	{
		I2C_FFA | I2C_LIQUID,
		APQ_8064_GSBI1_QUP_I2C_BUS_ID,
		isa1200_board_info,
		ARRAY_SIZE(isa1200_board_info),
	},
#endif
	{
		I2C_MPQ_CDP,
		APQ_8064_GSBI5_QUP_I2C_BUS_ID,
		cs8427_device_info,
		ARRAY_SIZE(cs8427_device_info),
	},
#ifdef CONFIG_KEYBOARD_CYPRESS_TOUCH_236
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_TOUCHKEY_I2C_BUS_ID,
		touchkey_i2c_devices_info,
		ARRAY_SIZE(touchkey_i2c_devices_info),
	},
#endif
#ifdef CONFIG_SENSORS_SSP
	{
		I2C_SURF | I2C_FFA,
		APQ_8064_GSBI2_QUP_I2C_BUS_ID,
		ssp_info,
		ARRAY_SIZE(ssp_info),
	},
#endif
#ifdef CONFIG_BCM2079X_NFC_I2C
#ifdef NFC_SW_I2C
	{
		I2C_SURF | I2C_FFA,
		MSM_NFC_I2C_BUS_ID,
		nfc_bcm2079x_info,
		ARRAY_SIZE(nfc_bcm2079x_info),
	},
#else
	{
		I2C_SURF | I2C_FFA,
		APQ_8064_GSBI4_QUP_I2C_BUS_ID,
		nfc_bcm2079x_info,
		ARRAY_SIZE(nfc_bcm2079x_info),
	},
#endif
#endif
#ifdef CONFIG_MFD_MAX77693
	{
		I2C_FFA,
		MSM_FSA9485_I2C_BUS_ID,
		max77693_i2c_board_info,
		ARRAY_SIZE(max77693_i2c_board_info),
	},
#endif
#if defined(CONFIG_VIDEO_MHL_V1) || defined(CONFIG_VIDEO_MHL_V2)
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_MHL_I2C_BUS_ID,
		mhl_i2c_board_info,
		ARRAY_SIZE(mhl_i2c_board_info),
	},
#endif
#if defined(CONFIG_LEDS_AN30259A)
	{
		I2C_SURF | I2C_FFA | I2C_LIQUID,
		MSM_MHL_I2C_BUS_ID,
		leds_i2c_devs,
		ARRAY_SIZE(leds_i2c_devs),
	},
#endif
#ifdef CONFIG_SEC_FPGA
	{
		I2C_FFA,
		MSM_SEC_FPGA_I2C_BUS_ID,
		ice4_fpga_i2c_board_info,
		ARRAY_SIZE(ice4_fpga_i2c_board_info),
	},
#endif
#if defined(CONFIG_FELICA)
	{
		I2C_SURF | I2C_FFA | I2C_LIQUID,
		MSM_MHL_I2C_BUS_ID,
		felica_i2c_devs,
		ARRAY_SIZE(felica_i2c_devs),
	},
#endif /* CONFIG_FELICA */
};

#define SX150X_EXP1_INT_N	PM8921_MPP_IRQ(PM8921_IRQ_BASE, 9)
#define SX150X_EXP2_INT_N	MSM_GPIO_TO_INT(81)

struct sx150x_platform_data mpq8064_sx150x_pdata[] = {
	[SX150X_EXP1] = {
		.gpio_base	= SX150X_EXP1_GPIO_BASE,
		.oscio_is_gpo	= false,
		.io_pullup_ena	= 0x0,
		.io_pulldn_ena	= 0x0,
		.io_open_drain_ena = 0x0,
		.io_polarity	= 0,
		.irq_summary	= SX150X_EXP1_INT_N,
		.irq_base	= SX150X_EXP1_IRQ_BASE,
	},
	[SX150X_EXP2] = {
		.gpio_base	= SX150X_EXP2_GPIO_BASE,
		.oscio_is_gpo	= false,
		.io_pullup_ena	= 0x0f,
		.io_pulldn_ena	= 0x70,
		.io_open_drain_ena = 0x0,
		.io_polarity	= 0,
		.irq_summary	= SX150X_EXP2_INT_N,
		.irq_base	= SX150X_EXP2_IRQ_BASE,
	},
	[SX150X_EXP3] = {
		.gpio_base	= SX150X_EXP3_GPIO_BASE,
		.oscio_is_gpo	= false,
		.io_pullup_ena	= 0x0,
		.io_pulldn_ena	= 0x0,
		.io_open_drain_ena = 0x0,
		.io_polarity	= 0,
		.irq_summary	= -1,
	},
	[SX150X_EXP4] = {
		.gpio_base	= SX150X_EXP4_GPIO_BASE,
		.oscio_is_gpo	= false,
		.io_pullup_ena	= 0x0,
		.io_pulldn_ena	= 0x0,
		.io_open_drain_ena = 0x0,
		.io_polarity	= 0,
		.irq_summary	= -1,
	},
};

static struct i2c_board_info sx150x_gpio_exp_info[] = {
	{
		I2C_BOARD_INFO("sx1509q", 0x70),
		.platform_data = &mpq8064_sx150x_pdata[SX150X_EXP1],
	},
	{
		I2C_BOARD_INFO("sx1508q", 0x23),
		.platform_data = &mpq8064_sx150x_pdata[SX150X_EXP2],
	},
	{
		I2C_BOARD_INFO("sx1508q", 0x22),
		.platform_data = &mpq8064_sx150x_pdata[SX150X_EXP3],
	},
	{
		I2C_BOARD_INFO("sx1509q", 0x3E),
		.platform_data = &mpq8064_sx150x_pdata[SX150X_EXP4],
	},
};

#define MPQ8064_I2C_GSBI5_BUS_ID	5

static struct i2c_registry mpq8064_i2c_devices[] __initdata = {
	{
		I2C_MPQ_CDP,
		MPQ8064_I2C_GSBI5_BUS_ID,
		sx150x_gpio_exp_info,
		ARRAY_SIZE(sx150x_gpio_exp_info),
	},
};

static void __init register_i2c_devices(void)
{
	u8 mach_mask = 0;
	int i;

#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_CAMERA_USE_GSBI7
	struct i2c_registry apq8064_camera_i2c_devices = {
		I2C_SURF | I2C_FFA | I2C_LIQUID | I2C_RUMI,
		APQ_8064_GSBI7_QUP_I2C_BUS_ID,
		apq8064_camera_board_info.board_info,
		apq8064_camera_board_info.num_i2c_board_info,
	};
#else
	struct i2c_registry apq8064_camera_i2c_devices = {
		I2C_SURF | I2C_FFA | I2C_LIQUID | I2C_RUMI,
		APQ_8064_GSBI4_QUP_I2C_BUS_ID,
		apq8064_camera_board_info.board_info,
		apq8064_camera_board_info.num_i2c_board_info,
	};
#endif
#ifdef CONFIG_CAMERA_SW_I2C
	struct i2c_registry apq8064_front_camera_i2c_devices = {
		I2C_SURF | I2C_FFA | I2C_LIQUID | I2C_RUMI,
		MSM_CAMERA_SW_I2C_BUS_ID,
		apq8064_front_camera_board_info.board_info,
		apq8064_front_camera_board_info.num_i2c_board_info,
	};
#else
	struct i2c_registry apq8064_front_camera_i2c_devices = {
		I2C_SURF | I2C_FFA | I2C_LIQUID | I2C_RUMI,
		APQ_8064_GSBI7_QUP_I2C_BUS_ID,
		apq8064_front_camera_board_info.board_info,
		apq8064_front_camera_board_info.num_i2c_board_info,
	};
#endif
#endif
	/* Build the matching 'supported_machs' bitmask */
	if (machine_is_apq8064_cdp())
		mach_mask = I2C_SURF;
	else if (machine_is_apq8064_mtp()|| machine_is_JF())
		mach_mask = I2C_FFA;
	else if (machine_is_apq8064_liquid())
		mach_mask = I2C_LIQUID;
	else if (PLATFORM_IS_MPQ8064())
		mach_mask = I2C_MPQ_CDP;
	else
		pr_err("unmatched machine ID in register_i2c_devices\n");

	/* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(apq8064_i2c_devices); ++i) {
		if (apq8064_i2c_devices[i].machs & mach_mask)
			i2c_register_board_info(apq8064_i2c_devices[i].bus,
						apq8064_i2c_devices[i].info,
						apq8064_i2c_devices[i].len);
	}
#ifdef CONFIG_MSM_CAMERA
	if (apq8064_camera_i2c_devices.machs & mach_mask)
		i2c_register_board_info(apq8064_camera_i2c_devices.bus,
			apq8064_camera_i2c_devices.info,
			apq8064_camera_i2c_devices.len);

	if (apq8064_front_camera_i2c_devices.machs & mach_mask)
		i2c_register_board_info(apq8064_front_camera_i2c_devices.bus,
			apq8064_front_camera_i2c_devices.info,
			apq8064_front_camera_i2c_devices.len);
#endif

	for (i = 0; i < ARRAY_SIZE(mpq8064_i2c_devices); ++i) {
		if (mpq8064_i2c_devices[i].machs & mach_mask)
			i2c_register_board_info(
					mpq8064_i2c_devices[i].bus,
					mpq8064_i2c_devices[i].info,
					mpq8064_i2c_devices[i].len);
	}
}

static void enable_ddr3_regulator(void)
{
	static struct regulator *ext_ddr3;

	/* Use MPP7 output state as a flag for PCDDR3 presence. */
	if (gpio_get_value_cansleep(PM8921_MPP_PM_TO_SYS(7)) > 0) {
		ext_ddr3 = regulator_get(NULL, "ext_ddr3");
		if (IS_ERR(ext_ddr3) || ext_ddr3 == NULL)
			pr_err("Could not get MPP7 regulator\n");
		else
			regulator_enable(ext_ddr3);
	}
}

static void enable_avc_i2c_bus(void)
{
	int avc_i2c_en_mpp = PM8921_MPP_PM_TO_SYS(8);
	int rc;

	rc = gpio_request(avc_i2c_en_mpp, "avc_i2c_en");
	if (rc)
		pr_err("request for avc_i2c_en mpp failed,"
						 "rc=%d\n", rc);
	else
		gpio_set_value_cansleep(avc_i2c_en_mpp, 1);
}

static void main_mic_bias_init(void)
{
	return;
}

static void __init gpio_rev_init(void)
{
#ifdef CONFIG_SEC_FPGA
		barcode_i2c_gpio_data.sda_pin =
			PM8921_MPP_PM_TO_SYS(PMIC_MPP_FPGA_SPI_SI);
		barcode_i2c_gpio_data.scl_pin =
			PM8921_MPP_PM_TO_SYS(PMIC_MPP_FPGA_SPI_CLK);
		barcode_emul_info.spi_en  = -1;
		barcode_emul_info.cresetb =
		PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_FPGA_CRESET_B);

	barcode_emul_info.spi_clk =
			PM8921_MPP_PM_TO_SYS(PMIC_MPP_FPGA_SPI_CLK);
	barcode_emul_info.spi_si  =
			PM8921_MPP_PM_TO_SYS(PMIC_MPP_FPGA_SPI_SI);
#endif
}
static void sec_jack_init(void)
{

#if defined(CONFIG_SAMSUNG_JACK_GNDLDET)
	static struct pm_gpio gnd_ear_det = {
		.direction		= PM_GPIO_DIR_IN,
		.pull			= PM_GPIO_PULL_NO,
		.out_strength	= PM_GPIO_STRENGTH_HIGH,
		.function		= PM_GPIO_FUNC_NORMAL,
		.inv_int_pol	= 0,		
		.vin_sel		= PM_GPIO_VIN_S4,
		.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
		.output_value	= 0,		
	};	
#endif

static struct pm_gpio fsa8048_en = {
	.direction		= PM_GPIO_DIR_OUT,
	.pull			= PM_GPIO_PULL_NO,
	.out_strength	= PM_GPIO_STRENGTH_HIGH,
	.function		= PM_GPIO_FUNC_NORMAL,
	.inv_int_pol	= 0,		
	.vin_sel		= PM_GPIO_VIN_S4,
	.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
	.output_value	= 1,		
};

#if defined(CONFIG_SAMSUNG_JACK_GNDLDET)
	pm8xxx_gpio_config(
			PM8921_GPIO_PM_TO_SYS(GPIO_EAR_GND_DET),
			&gnd_ear_det);
#endif

	if (system_rev >= BOARD_REV07) {
		sec_jack_data.send_end_active_high = true;
		sec_jack_data.zones = jack_zones_rev7;
		sec_jack_data.buttons_zones = jack_buttons_zones_rev7;		
	}

	if (system_rev >= BOARD_REV10) {
		pm8xxx_gpio_config(
			PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_FSA8048_EN),
			&fsa8048_en);
	}
}

static void vps_sound_init(void)
{
	int ret;
	struct pm_gpio vps_sound_en = {
		.direction		= PM_GPIO_DIR_OUT,
		.pull			= PM_GPIO_PULL_NO,
		.out_strength		= PM_GPIO_STRENGTH_HIGH,
		.function		= PM_GPIO_FUNC_NORMAL,
		.inv_int_pol		= 0,
		.vin_sel		= PM_GPIO_VIN_S4,
		.output_buffer		= PM_GPIO_OUT_BUF_CMOS,
		.output_value		= 0,
	};

	ret = gpio_request(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_VPS_SOUND_EN),
		"vps_sound_en");
	if (ret) {
		pr_err("%s : gpio_request failed for %d\n", __func__,
			PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_VPS_SOUND_EN));
		return;
	}

	pm8xxx_gpio_config(
			PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_VPS_SOUND_EN),
			&vps_sound_en);

}
static void __init apq8064_common_init(void)
{
	u32 platform_version;

#ifdef CONFIG_KEYBOARD_CYPRESS_TOUCH_236
	int ret;
#endif

#ifndef CONFIG_MACH_JF
	if (socinfo_get_pmic_model() == PMIC_MODEL_PM8917)
		apq8064_pm8917_pdata_fixup();
#endif
	platform_device_register(&msm_gpio_device);
	msm_tsens_early_init(&apq_tsens_pdata);
	msm_thermal_init(&msm_thermal_pdata);
	if (socinfo_init() < 0)
		pr_err("socinfo_init() failed!\n");
	BUG_ON(msm_rpm_init(&apq8064_rpm_data));
	BUG_ON(msm_rpmrs_levels_init(&msm_rpmrs_data));
	regulator_suppress_info_printing();
	gpio_rev_init();
	if (socinfo_get_pmic_model() == PMIC_MODEL_PM8917)
		configure_apq8064_pm8917_power_grid();
	platform_device_register(&apq8064_device_rpm_regulator);
	if (socinfo_get_pmic_model() != PMIC_MODEL_PM8917)
		platform_device_register(&apq8064_pm8921_device_rpm_regulator);
	if (msm_xo_init())
		pr_err("Failed to initialize XO votes\n");
	msm_clock_init(&apq8064_clock_init_data);
	apq8064_init_gpiomux();
	apq8064_i2c_init();
	register_i2c_devices();

	apq8064_device_qup_spi_gsbi5.dev.platform_data =
						&apq8064_qup_spi_gsbi5_pdata;
	apq8064_init_pmic();
	if (machine_is_apq8064_liquid())
		msm_otg_pdata.mhl_enable = true;

	android_usb_pdata.swfi_latency =
		msm_rpmrs_levels[0].latency_us;

	apq8064_device_otg.dev.platform_data = &msm_otg_pdata;
	apq8064_ehci_host_init();
	apq8064_init_buses();

	platform_add_devices(early_common_devices,
				ARRAY_SIZE(early_common_devices));
	if (socinfo_get_pmic_model() != PMIC_MODEL_PM8917)
		platform_add_devices(pm8921_common_devices,
					ARRAY_SIZE(pm8921_common_devices));
	else
		platform_add_devices(pm8917_common_devices,
					ARRAY_SIZE(pm8917_common_devices));
	platform_add_devices(common_devices, ARRAY_SIZE(common_devices));
	if (!(machine_is_mpq8064_cdp() || machine_is_mpq8064_hrd() ||
			machine_is_mpq8064_dtv()))
		platform_add_devices(common_not_mpq_devices,
			ARRAY_SIZE(common_not_mpq_devices));
	
#ifdef CONFIG_KEYBOARD_CYPRESS_TOUCH_236
	if (system_rev < 9)
		platform_device_register(&touchkey_i2c_gpio_device);
	else
		platform_device_register(&touchkey_i2c_gpio_device_2);
#endif

	enable_ddr3_regulator();
	msm_hsic_pdata.swfi_latency =
		msm_rpmrs_levels[0].latency_us;
	if (machine_is_apq8064_mtp() || machine_is_JF()) {
		msm_hsic_pdata.log2_irq_thresh = 5,
		apq8064_device_hsic_host.dev.platform_data = &msm_hsic_pdata;
		device_initialize(&apq8064_device_hsic_host.dev);
	}
	apq8064_pm8xxx_gpio_mpp_init();
	apq8064_init_mmc();

	if (machine_is_apq8064_mtp()|| machine_is_JF()) {
		mdm_8064_device.dev.platform_data = &mdm_platform_data;
		platform_version = socinfo_get_platform_version();
		if (SOCINFO_VERSION_MINOR(platform_version) == 1) {
			i2s_mdm_8064_device.dev.platform_data =
				&mdm_platform_data;
			platform_device_register(&i2s_mdm_8064_device);
		} else {
			mdm_8064_device.dev.platform_data = &mdm_platform_data;
			platform_device_register(&mdm_8064_device);
		}
	}

	if (!poweroff_charging) {
		printk(KERN_DEBUG"[slimbus] starting init set up : %d %d\n",
				system_rev, poweroff_charging);
	        platform_device_register(&apq8064_slim_ctrl);
        	slim_register_board_info(apq8064_slim_devices,
	        	ARRAY_SIZE(apq8064_slim_devices));
    }

	if (!PLATFORM_IS_MPQ8064()) {
		apq8064_init_dsps();
		platform_device_register(&msm_8960_riva);
	}
	msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));
	msm_spm_l2_init(msm_spm_l2_data);
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));
	apq8064_epm_adc_init();
	msm_pm_set_tz_retention_flag(1);
	samsung_sys_class_init();
	
#ifdef CONFIG_MFD_MAX77693
	gpio_tlmm_config(GPIO_CFG(GPIO_IF_PMIC_IRQ,  0, GPIO_CFG_INPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_DISABLE);
#endif
#if defined(CONFIG_BATTERY_SAMSUNG)
	msm8960_init_battery();
#endif

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI) || defined(CONFIG_TOUCHSCREEN_ATMEL_MXTS)
	printk(KERN_DEBUG"[TSP] System revision, LPM mode : %d %d\n",
				system_rev, poweroff_charging);
	if (!poweroff_charging) {
		if (sec_tsp_synaptics_mode)
			S5000_tsp_input_init(lcd_tsp_panel_version);
		else
			mxt540s_tsp_input_init();
		}
#endif

#if defined(CONFIG_VIDEO_MHL_V2)
	msm8960_mhl_gpio_init();
#endif
#ifdef CONFIG_KEYBOARD_CYPRESS_TOUCH_236
	/*cypress_gpio_init();*/
	ret = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_TKEY_INT),
							&tkey_int);
	if (ret)
		pr_err("%s PMIC_GPIO_TKEY_INT config failed\n", __func__);
#endif
#ifdef CONFIG_BT_BCM4335
	apq8064_bt_init();
#endif

#ifdef CONFIG_ISDBTMM
	apq8064_device_qup_spi_gsbi4.dev.platform_data =
				&apq8064_qup_spi_gsbi4_pdata;
    tmm_dev_init();
#endif
}

static void __init apq8064_allocate_memory_regions(void)
{
	apq8064_allocate_fb_region();
}

static void __init apq8064_gpio_keys_init(void)
{
	int ret;
	struct pm_gpio param = {
		.direction     = PM_GPIO_DIR_IN,
		.pull          = PM_GPIO_PULL_UP_31P5,
		.out_strength  = PM_GPIO_STRENGTH_HIGH,
		.vin_sel       = PM_GPIO_VIN_S4,
		.function      = PM_GPIO_FUNC_NORMAL,
	};
#ifdef CONFIG_SENSORS_HALL
	struct pm_gpio param_hall_ic = {
		.direction     = PM_GPIO_DIR_IN,
		.pull          = PM_GPIO_PULL_NO,
		.vin_sel       = PM_GPIO_VIN_S4,
		.function      = PM_GPIO_FUNC_NORMAL,
	};
	ret = gpio_request(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_HALL_SENSOR_INT),
						"GPIO_HALL_SENSOR_INT");
	if (ret)
		pr_err("%s : gpio_request failed for %d\n", __func__,
			PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_HALL_SENSOR_INT));
	pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_HALL_SENSOR_INT),
						&param_hall_ic);
#endif
	pm8xxx_gpio_config(GPIO_KEY_VOLUME_UP, &param);
	pm8xxx_gpio_config(GPIO_KEY_VOLUME_DOWN, &param);
	pm8xxx_gpio_config(GPIO_KEY_HOME, &param);
}

static void __init samsung_jf_init(void)
{
#ifdef CONFIG_SEC_DEBUG
	sec_debug_init();
#endif
#ifdef CONFIG_PROC_AVC
	sec_avc_log_init();
#endif
	if (meminfo_init(SYS_MEMORY, SZ_256M) < 0)
		pr_err("meminfo_init() failed!\n");
	/*
	if (machine_is_apq8064_mtp() &&
		SOCINFO_VERSION_MINOR(socinfo_get_platform_version()) == 1)
			cyttsp_pdata.sleep_gpio = CYTTSP_TS_GPIO_SLEEP_ALT;
	*/
	apq8064_common_init();
	if (machine_is_mpq8064_cdp() || machine_is_mpq8064_hrd() ||
		machine_is_mpq8064_dtv()) {
		enable_avc_i2c_bus();
                msm_rotator_set_split_iommu_domain();
		platform_add_devices(mpq_devices, ARRAY_SIZE(mpq_devices));
		mpq8064_pcie_init();
	} else {
		ethernet_init();
                msm_rotator_set_split_iommu_domain();
		platform_add_devices(cdp_devices, ARRAY_SIZE(cdp_devices));
#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
		spi_register_board_info(spi_board_info,
						ARRAY_SIZE(spi_board_info));
#endif
	}
	apq8064_init_fb();
	apq8064_init_gpu();
	platform_add_devices(apq8064_footswitch, apq8064_num_footswitch);
	apq8064_gpio_keys_init();
#ifdef CONFIG_MSM_CAMERA
	apq8064_init_cam();
#endif
#ifdef CONFIG_SENSORS_SSP
	clear_ssp_gpio();
	sensor_power_on_vdd(SNS_PWR_ON, SNS_PWR_ON);
	initialize_ssp_gpio();
#endif
#ifdef CONFIG_MACH_JF
	platform_device_register(&gpio_kp_pdev);
#else
	if (machine_is_apq8064_cdp() || machine_is_apq8064_liquid())
		platform_device_register(&cdp_kp_pdev);

	if (machine_is_apq8064_mtp())
		platform_device_register(&mtp_kp_pdev);
#endif
#ifdef CONFIG_BCM2079X_NFC_I2C
	bcm2079x_init();
#endif
	change_memory_power = &apq8064_change_memory_power;

#ifndef CONFIG_MACH_JF
	if (machine_is_mpq8064_cdp()) {
		platform_device_register(&mpq_gpio_keys_pdev);
		platform_device_register(&mpq_keypad_device);
	}
#endif
	main_mic_bias_init();
	sec_jack_init();
	vps_sound_init();
#ifdef CONFIG_SND_SOC_ES325
	es325_start_api();
#endif
#if defined(CONFIG_BCM4335) || defined(CONFIG_BCM4335_MODULE)
	brcm_wlan_init();
#endif
#ifdef CONFIG_FELICA
	felica_init();
#endif /* CONFIG_FELICA */
}

MACHINE_START(JF, "SAMSUNG JF")
	.map_io = apq8064_map_io,
	.reserve = apq8064_reserve,
	.init_irq = apq8064_init_irq,
	.handle_irq = gic_handle_irq,
	.timer = &msm_timer,
	.init_machine = samsung_jf_init,
	.init_early = apq8064_allocate_memory_regions,
	.init_very_early = apq8064_early_reserve,
	.restart = msm_restart,
MACHINE_END
