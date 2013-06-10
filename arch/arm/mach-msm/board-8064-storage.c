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

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/bootmem.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>
#include <mach/msm_bus_board.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>
#include <mach/apq8064-gpio.h>
#include "devices.h"
#include "board-8064.h"
#include "board-storage-common-a.h"


/* APQ8064 has 4 SDCC controllers */
enum sdcc_controllers {
	SDCC1,
	SDCC2,
	SDCC3,
	SDCC4,
	MAX_SDCC_CONTROLLER
};

/* All SDCC controllers require VDD/VCC voltage */
static struct msm_mmc_reg_data mmc_vdd_reg_data[MAX_SDCC_CONTROLLER] = {
	/* SDCC1 : eMMC card connected */
	[SDCC1] = {
		.name = "sdc_vdd",
		.high_vol_level = 2950000,
		.low_vol_level = 2950000,
		.always_on = 1,
		.lpm_sup = 1,
		.lpm_uA = 9000,
		.hpm_uA = 200000, /* 200mA */
	},
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT	
	/* SDCC2 : External card slot connected after system_rev 08 */
	[SDCC2] = {
			.name = "sdc_vdd",
			.high_vol_level = 2950000,
			.low_vol_level = 2950000,
			.hpm_uA = 800000, /* 800mA */
	},
#endif
	/* SDCC3 : External card slot connected */
	[SDCC3] = {
#if defined(CONFIG_BCM4335) || defined(CONFIG_BCM4335_MODULE)
		.name = "sdc_vdd",
		.high_vol_level = 1800000,
		.low_vol_level = 1800000,
		.lpm_sup = 1,
		.hpm_uA = 16000,
		.lpm_uA = 2000,
#else
		.name = "sdc_vdd",
		.high_vol_level = 2950000,
		.low_vol_level = 2950000,
		.hpm_uA = 800000, /* 800mA */
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	},
	/* SDCC4 : External card slot connected after system_rev 05 */
	[SDCC4] = {
		.name = "sdc_vdd",
		.high_vol_level = 2950000,
		.low_vol_level = 2950000,
		.hpm_uA = 800000, /* 800mA */
#endif
	}
};

/* SDCC controllers may require voting for VDD IO voltage */
static struct msm_mmc_reg_data mmc_vdd_io_reg_data[MAX_SDCC_CONTROLLER] = {
	/* SDCC1 : eMMC card connected */
	[SDCC1] = {
		.name = "sdc_vdd_io",
		.always_on = 1,
		.high_vol_level = 1800000,
		.low_vol_level = 1800000,
		.hpm_uA = 200000, /* 200mA */
	},
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	/* SDCC2 : External card slot connected after system_rev 08 */
	[SDCC2] = {
		.name = "sdc_vdd_io",
		.high_vol_level = 1800000,
		.low_vol_level = 1800000,
		.always_on = 1,
		.lpm_sup = 1,
		/* Max. Active current required is 16 mA */
		.hpm_uA = 16000,
		/*
		 * Sleep current required is ~300 uA. But min. vote can be
		 * in terms of mA (min. 1 mA). So let's vote for 2 mA
		 * during sleep.
		 */
		.lpm_uA = 2000,
	},
#endif
	/* SDCC3 : External card slot connected */
	[SDCC3] = {
#if defined(CONFIG_BCM4335) || defined(CONFIG_BCM4335_MODULE)
		.name = "sdc_vdd_io",
		.high_vol_level = 1800000,
		.low_vol_level = 1800000,
		.always_on = 1,
		.lpm_sup = 1,
		.hpm_uA = 16000,
		.lpm_uA = 2000,
#else
		.name = "sdc_vdd_io",
		.high_vol_level = 2950000,
		.low_vol_level = 1850000,
		.always_on = 1,
		.lpm_sup = 1,
		/* Max. Active current required is 16 mA */
		.hpm_uA = 16000,
		/*
		 * Sleep current required is ~300 uA. But min. vote can be
		 * in terms of mA (min. 1 mA). So let's vote for 2 mA
		 * during sleep.
		 */
		.lpm_uA = 2000,
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	},
	/* SDCC4 : External card slot connected after system_rev 05 */
	[SDCC4] = {
		.name = "sdc_vdd_io",
		.high_vol_level = 1800000,
		.low_vol_level = 1800000,
		.always_on = 1,
		.lpm_sup = 1,
		/* Max. Active current required is 16 mA */
		.hpm_uA = 16000,
		/*
		 * Sleep current required is ~300 uA. But min. vote can be
		 * in terms of mA (min. 1 mA). So let's vote for 2 mA
		 * during sleep.
		 */
		.lpm_uA = 2000,
#endif
	}
};

static struct msm_mmc_slot_reg_data mmc_slot_vreg_data[MAX_SDCC_CONTROLLER] = {
	/* SDCC1 : eMMC card connected */
	[SDCC1] = {
		.vdd_data = &mmc_vdd_reg_data[SDCC1],
		.vdd_io_data = &mmc_vdd_io_reg_data[SDCC1],
	},
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	/* SDCC2 : External card slot connected after system_rev 08 */
	[SDCC2] = {
		.vdd_data = &mmc_vdd_reg_data[SDCC2],
		.vdd_io_data = &mmc_vdd_io_reg_data[SDCC2],
	},
#endif
	/* SDCC3 : External card slot connected */
	[SDCC3] = {
		.vdd_data = &mmc_vdd_reg_data[SDCC3],
		.vdd_io_data = &mmc_vdd_io_reg_data[SDCC3],
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	},
	/* SDCC4 : External card slot connected after system_rev 05 */
	[SDCC4] = {
		.vdd_data = &mmc_vdd_reg_data[SDCC4],
		.vdd_io_data = &mmc_vdd_io_reg_data[SDCC4],
#endif
	}
};

/* SDC1 pad data */
static struct msm_mmc_pad_drv sdc1_pad_drv_on_cfg[] = {
	{TLMM_HDRV_SDC1_CLK, GPIO_CFG_16MA},
	{TLMM_HDRV_SDC1_CMD, GPIO_CFG_10MA},
	{TLMM_HDRV_SDC1_DATA, GPIO_CFG_10MA}
};

static struct msm_mmc_pad_drv sdc1_pad_drv_off_cfg[] = {
	{TLMM_HDRV_SDC1_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC1_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC1_DATA, GPIO_CFG_2MA}
};

static struct msm_mmc_pad_pull sdc1_pad_pull_on_cfg[] = {
	{TLMM_PULL_SDC1_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC1_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC1_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_mmc_pad_pull sdc1_pad_pull_off_cfg[] = {
	{TLMM_PULL_SDC1_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC1_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC1_DATA, GPIO_CFG_PULL_UP}
};

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
/* SDC2 pad data */
static struct msm_mmc_pad_drv sdc2_pad_drv_on_cfg[] = {
	{TLMM_HDRV_SDC2_CLK, GPIO_CFG_6MA},
	{TLMM_HDRV_SDC2_CMD, GPIO_CFG_6MA},
	{TLMM_HDRV_SDC2_DATA, GPIO_CFG_6MA}
};

static struct msm_mmc_pad_drv sdc2_pad_drv_off_cfg[] = {
	{TLMM_HDRV_SDC2_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC2_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC2_DATA, GPIO_CFG_2MA}
};

static struct msm_mmc_pad_pull sdc2_pad_pull_on_cfg[] = {
	{TLMM_PULL_SDC2_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC2_CMD, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC2_DATA, GPIO_CFG_NO_PULL}
};

static struct msm_mmc_pad_pull sdc2_pad_pull_off_cfg[] = {
	{TLMM_PULL_SDC2_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC2_CMD, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC2_DATA, GPIO_CFG_NO_PULL}
};
#endif

/* SDC3 pad data */
static struct msm_mmc_pad_drv sdc3_pad_drv_on_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_10MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_6MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_6MA}
};

static struct msm_mmc_pad_drv sdc3_pad_drv_off_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_2MA}
};

static struct msm_mmc_pad_pull sdc3_pad_pull_on_cfg[] = {
	{TLMM_PULL_SDC3_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_mmc_pad_pull sdc3_pad_pull_off_cfg[] = {
	{TLMM_PULL_SDC3_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_UP}
};

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
/* SDC4 pad data */
static struct msm_mmc_pad_drv sdc4_pad_drv_on_cfg[] = {
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_6MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_6MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_6MA}
};

static struct msm_mmc_pad_drv sdc4_pad_drv_off_cfg[] = {
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_2MA}
};

static struct msm_mmc_pad_pull sdc4_pad_pull_on_cfg[] = {
	{TLMM_PULL_SDC4_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_mmc_pad_pull sdc4_pad_pull_off_cfg[] = {
	{TLMM_PULL_SDC4_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_UP}
};
#endif

static struct msm_mmc_pad_pull_data mmc_pad_pull_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.on = sdc1_pad_pull_on_cfg,
		.off = sdc1_pad_pull_off_cfg,
		.size = ARRAY_SIZE(sdc1_pad_pull_on_cfg)
	},
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	[SDCC2] = {
		.on = sdc2_pad_pull_on_cfg,
		.off = sdc2_pad_pull_off_cfg,
		.size = ARRAY_SIZE(sdc2_pad_pull_on_cfg)
	},
#endif
	[SDCC3] = {
		.on = sdc3_pad_pull_on_cfg,
		.off = sdc3_pad_pull_off_cfg,
		.size = ARRAY_SIZE(sdc3_pad_pull_on_cfg)
	},
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	[SDCC4] = {
		.on = sdc4_pad_pull_on_cfg,
		.off = sdc4_pad_pull_off_cfg,
		.size = ARRAY_SIZE(sdc4_pad_pull_on_cfg)
	},
#endif
};

static struct msm_mmc_pad_drv_data mmc_pad_drv_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.on = sdc1_pad_drv_on_cfg,
		.off = sdc1_pad_drv_off_cfg,
		.size = ARRAY_SIZE(sdc1_pad_drv_on_cfg)
	},
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	[SDCC2] = {
		.on = sdc2_pad_drv_on_cfg,
		.off = sdc2_pad_drv_off_cfg,
		.size = ARRAY_SIZE(sdc2_pad_drv_on_cfg)
	},
#endif
	[SDCC3] = {
		.on = sdc3_pad_drv_on_cfg,
		.off = sdc3_pad_drv_off_cfg,
		.size = ARRAY_SIZE(sdc3_pad_drv_on_cfg)
	},
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	[SDCC4] = {
		.on = sdc4_pad_drv_on_cfg,
		.off = sdc4_pad_drv_off_cfg,
		.size = ARRAY_SIZE(sdc4_pad_drv_on_cfg)
	},
#endif
};

static struct msm_mmc_pad_data mmc_pad_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.pull = &mmc_pad_pull_data[SDCC1],
		.drv = &mmc_pad_drv_data[SDCC1]
	},
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	[SDCC2] = {
		.pull = &mmc_pad_pull_data[SDCC2],
		.drv = &mmc_pad_drv_data[SDCC2]
	},
#endif
	[SDCC3] = {
		.pull = &mmc_pad_pull_data[SDCC3],
		.drv = &mmc_pad_drv_data[SDCC3]
	},
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	[SDCC4] = {
		.pull = &mmc_pad_pull_data[SDCC4],
		.drv = &mmc_pad_drv_data[SDCC4]
	},
#endif
};

static struct msm_mmc_gpio sdc2_gpio[] = {
	{59, "sdc2_clk"},
	{57, "sdc2_cmd"},
	{62, "sdc2_dat_0"},
	{61, "sdc2_dat_1"},
	{60, "sdc2_dat_2"},
	{58, "sdc2_dat_3"},
};

static struct msm_mmc_gpio sdc4_gpio[] = {
	{68, "sdc4_clk"},
	{67, "sdc4_cmd"},
	{66, "sdc4_dat_0"},
	{65, "sdc4_dat_1"},
	{64, "sdc4_dat_2"},
	{63, "sdc4_dat_3"},
};

static struct msm_mmc_gpio_data mmc_gpio_data[MAX_SDCC_CONTROLLER] = {
	[SDCC2] = {
		.gpio = sdc2_gpio,
		.size = ARRAY_SIZE(sdc2_gpio),
	},
	[SDCC4] = {
		.gpio = sdc4_gpio,
		.size = ARRAY_SIZE(sdc4_gpio),
	}
};

static struct msm_mmc_pin_data mmc_slot_pin_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.pad_data = &mmc_pad_data[SDCC1],
	},
	[SDCC2] = {
		.is_gpio = 1,
		.gpio_data = &mmc_gpio_data[SDCC2],
	},
	[SDCC3] = {
		.pad_data = &mmc_pad_data[SDCC3],
	},
	[SDCC4] = {
		.is_gpio = 1,
		.gpio_data = &mmc_gpio_data[SDCC4],
	},
};

#define MSM_MPM_PIN_SDC1_DAT1	17
#define MSM_MPM_PIN_SDC2_DAT1	19
#define MSM_MPM_PIN_SDC3_DAT1	21
#define MSM_MPM_PIN_SDC4_DAT1	23

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static unsigned int sdc1_sup_clk_rates[] = {
	400000, 24000000, 48000000, 96000000
};

static unsigned int sdc1_sup_clk_rates_all[] = {
	400000, 24000000, 48000000, 96000000, 192000000
};

static struct mmc_platform_data sdc1_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.sup_clk_table	= sdc1_sup_clk_rates,
	.sup_clk_cnt	= ARRAY_SIZE(sdc1_sup_clk_rates),
	.nonremovable	= 1,
	.pin_data	= &mmc_slot_pin_data[SDCC1],
	.vreg_data	= &mmc_slot_vreg_data[SDCC1],
	.uhs_caps	= MMC_CAP_1_8V_DDR | MMC_CAP_UHS_DDR50,
	.uhs_caps2	= MMC_CAP2_ADAPT_PACKED | MMC_CAP2_HS200_1_8V_SDR | MMC_CAP2_CACHE_CTRL,
	.packed_write	= MMC_CAP2_PACKED_WR | MMC_CAP2_PACKED_WR_CONTROL,
	.mpm_sdiowakeup_int = MSM_MPM_PIN_SDC1_DAT1,
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
};
static struct mmc_platform_data *apq8064_sdc1_pdata = &sdc1_data;
#else
static struct mmc_platform_data *apq8064_sdc1_pdata;
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static unsigned int sdc2_sup_clk_rates[] = {
	400000, 24000000, 48000000
};

static struct mmc_platform_data sdc2_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.sup_clk_table	= sdc2_sup_clk_rates,
	.sup_clk_cnt = ARRAY_SIZE(sdc2_sup_clk_rates),
	.pin_data	= &mmc_slot_pin_data[SDCC2],
	.vreg_data	= &mmc_slot_vreg_data[SDCC2],
	.wpswitch_gpio	= 0,
	.is_wpswitch_active_low = false,
	.status_gpio	= 26,
	.status_irq = MSM_GPIO_TO_INT(26),
	.irq_flags	= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.is_status_gpio_active_low = 1,
	.xpc_cap = 1,
	.mpm_sdiowakeup_int = MSM_MPM_PIN_SDC2_DAT1,
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
};
static struct mmc_platform_data *apq8064_sdc2_pdata = &sdc2_data;
#else
static struct mmc_platform_data *apq8064_sdc2_pdata;
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static unsigned int sdc3_sup_clk_rates[] = {
	400000, 24000000, 48000000, 96000000, 192000000
};

static struct mmc_platform_data sdc3_data = {
#if defined(CONFIG_BCM4335) || defined(CONFIG_BCM4335_MODULE)
	.built_in = 1,
	.ocr_mask       = MMC_VDD_165_195 | MMC_VDD_27_28 | MMC_VDD_28_29,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.sup_clk_table	= sdc3_sup_clk_rates,
	.sup_clk_cnt	= ARRAY_SIZE(sdc3_sup_clk_rates),
	.pin_data	= &mmc_slot_pin_data[SDCC3],
	.vreg_data	= &mmc_slot_vreg_data[SDCC3],
	.status = brcm_wifi_status,
	.uhs_caps	= (MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
			MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_DDR50 |
			MMC_CAP_UHS_SDR104 | MMC_CAP_MAX_CURRENT_800),
	.register_status_notify	= brcm_wifi_status_register,
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
#else
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.sup_clk_table	= sdc3_sup_clk_rates,
	.sup_clk_cnt	= ARRAY_SIZE(sdc3_sup_clk_rates),
	.pin_data	= &mmc_slot_pin_data[SDCC3],
	.vreg_data	= &mmc_slot_vreg_data[SDCC3],
	.wpswitch_gpio	= PM8921_GPIO_PM_TO_SYS(17),
	.is_wpswitch_active_low = true,
	.status_gpio	= 26,
	.status_irq	= MSM_GPIO_TO_INT(26),
	.irq_flags	= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.is_status_gpio_active_low = 1,
	.xpc_cap	= 1,
	.uhs_caps	= (MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
			MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_DDR50 |
			MMC_CAP_UHS_SDR104 | MMC_CAP_MAX_CURRENT_800),
	.mpm_sdiowakeup_int = MSM_MPM_PIN_SDC3_DAT1,
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
#endif
};
static struct mmc_platform_data *apq8064_sdc3_pdata = &sdc3_data;
#else
static struct mmc_platform_data *apq8064_sdc3_pdata;
#endif


#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static unsigned int sdc4_sup_clk_rates[] = {
	400000, 24000000, 48000000
};

static struct mmc_platform_data sdc4_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.sup_clk_table	= sdc4_sup_clk_rates,
	.sup_clk_cnt = ARRAY_SIZE(sdc4_sup_clk_rates),
	.pin_data	= &mmc_slot_pin_data[SDCC4],
	.vreg_data	= &mmc_slot_vreg_data[SDCC4],
	.wpswitch_gpio	= 0,
	.is_wpswitch_active_low = false,
	.status_gpio	= 26,
	.status_irq = MSM_GPIO_TO_INT(26),
	.irq_flags	= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.is_status_gpio_active_low = 1,
	.xpc_cap = 1,
	.mpm_sdiowakeup_int = MSM_MPM_PIN_SDC4_DAT1,
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
};
static struct mmc_platform_data *apq8064_sdc4_pdata = &sdc4_data;
#else
static struct mmc_platform_data *apq8064_sdc4_pdata;
#endif

void __init apq8064_init_mmc(void)
{
	if (apq8064_sdc1_pdata) {
		/* 8064 v2 supports upto 200MHz clock on SDC1 slot */
		if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) >= 2) {
			apq8064_sdc1_pdata->sup_clk_table =
					sdc1_sup_clk_rates_all;
			apq8064_sdc1_pdata->sup_clk_cnt	=
					ARRAY_SIZE(sdc1_sup_clk_rates_all);
		}
		apq8064_add_sdcc(1, apq8064_sdc1_pdata);
	}
/*
	if (apq8064_sdc2_pdata)
		apq8064_add_sdcc(2, apq8064_sdc2_pdata);
*/
/* system_rev 0x05 or more
 * SDC3 is used for WIFI
    SDC4 is used for External memory Card
 */
	if (apq8064_sdc3_pdata) {

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
		mmc_vdd_reg_data[SDCC3].high_vol_level = 1800000;
		mmc_vdd_reg_data[SDCC3].low_vol_level = 1800000;
		mmc_slot_vreg_data[SDCC3].vdd_io_data = NULL;
		apq8064_sdc3_pdata->status_irq = 0;
		apq8064_sdc3_pdata->status_gpio = 0;
#endif
#if !defined(CONFIG_BCM4335) && !defined(CONFIG_BCM4335_MODULE)
		if (!machine_is_apq8064_cdp()) {
			apq8064_sdc3_pdata->wpswitch_gpio = 0;
			apq8064_sdc3_pdata->is_wpswitch_active_low = false;
		}
		if (machine_is_mpq8064_cdp() || machine_is_mpq8064_hrd() ||
			machine_is_mpq8064_dtv()) {
			int rc;
			struct pm_gpio sd_card_det_init_cfg = {
				.direction      = PM_GPIO_DIR_IN,
				.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
				.pull           = PM_GPIO_PULL_UP_30,
				.vin_sel        = PM_GPIO_VIN_S4,
				.out_strength   = PM_GPIO_STRENGTH_NO,
				.function       = PM_GPIO_FUNC_NORMAL,
			};

			apq8064_sdc3_pdata->status_gpio =
				PM8921_GPIO_PM_TO_SYS(31);
			apq8064_sdc3_pdata->status_irq =
				PM8921_GPIO_IRQ(PM8921_IRQ_BASE, 31);
			rc = pm8xxx_gpio_config(apq8064_sdc3_pdata->status_gpio,
					&sd_card_det_init_cfg);
			if (rc) {
				pr_info("%s: SD_CARD_DET GPIO%d config "
					"failed(%d)\n", __func__,
					apq8064_sdc3_pdata->status_gpio, rc);
				apq8064_sdc3_pdata->status_gpio = 0;
				apq8064_sdc3_pdata->status_irq = 0;
			}
		}
		if (machine_is_apq8064_cdp()) {
			int i;

			for (i = 0;
			     i < apq8064_sdc3_pdata->pin_data->pad_data->\
				 drv->size;
			     i++)
				apq8064_sdc3_pdata->pin_data->pad_data->\
					drv->on[i].val = GPIO_CFG_10MA;
		}
#endif
		apq8064_add_sdcc(3, apq8064_sdc3_pdata);
	}

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	pr_info("%s: SDC4 is for Externel SD in system_rev %d\n",
			__func__, system_rev);
	/*gpio_direction_output(60, 0); */ /* TFLASH_LS_EN */
#endif

#if defined(CONFIG_MACH_JF_ATT) || defined(CONFIG_MACH_JF_TMO) || defined(CONFIG_MACH_JF_EUR) || \
	defined(CONFIG_MACH_JF_DCM)
	if (system_rev >= BOARD_REV09 && apq8064_sdc2_pdata) {
#elif defined(CONFIG_MACH_JACTIVE_EUR) || defined(CONFIG_MACH_JACTIVE_ATT)
	if (system_rev < BOARD_REV05 && apq8064_sdc2_pdata) {
#else /* VZW/SPT/USCC */
	if (system_rev >= BOARD_REV10 && apq8064_sdc2_pdata) {
#endif
		apq8064_sdc2_pdata->status_gpio = PM8921_GPIO_PM_TO_SYS(33);
		apq8064_sdc2_pdata->status_irq	= PM8921_GPIO_IRQ(PM8921_IRQ_BASE, 33);
	}

#if defined(CONFIG_MACH_JF_ATT) || defined(CONFIG_MACH_JF_TMO) || defined(CONFIG_MACH_JF_EUR) || \
	defined(CONFIG_MACH_JACTIVE_EUR) || defined(CONFIG_MACH_JACTIVE_ATT)
	if (system_rev < BOARD_REV08 && apq8064_sdc4_pdata)
#else /* VZW/SPT/USCC */
	if (system_rev < BOARD_REV09 && apq8064_sdc4_pdata)
#endif
		apq8064_add_sdcc(4, apq8064_sdc4_pdata);
	else if (apq8064_sdc2_pdata)
		apq8064_add_sdcc(2, apq8064_sdc2_pdata);
}
