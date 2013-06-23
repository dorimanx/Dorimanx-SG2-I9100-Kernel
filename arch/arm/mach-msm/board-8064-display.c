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

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/msm_ion.h>
#include <asm/mach-types.h>
#include <mach/msm_memtypes.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/ion.h>
#include <mach/msm_bus_board.h>
#include <mach/socinfo.h>

#include "devices.h"
#include "board-8064.h"

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#if defined(CONFIG_FB_MSM_MIPI_SAMSUNG_OLED_VIDEO_HD_PT_PANEL)
/* prim = 1280 x 736 x 3(bpp) x 3(pages) */
#define MSM_FB_PRIM_BUF_SIZE roundup(1100 * 1920 * 4 * 3, 0x10000)
#else
/* prim = 1366 x 768 x 3(bpp) x 3(pages) */
#define MSM_FB_PRIM_BUF_SIZE roundup(1100 * 1920 * 4 * 3, 0x10000)
#endif
#else
#if defined(CONFIG_FB_MSM_MIPI_SAMSUNG_OLED_VIDEO_HD_PT_PANEL)
/* prim = 1280 x 736 x 3(bpp) x 3(pages) */
#define MSM_FB_PRIM_BUF_SIZE roundup(1100 * 1920 * 4 * 2, 0x10000)
#else
/* prim = 1366 x 768 x 3(bpp) x 2(pages) */
#define MSM_FB_PRIM_BUF_SIZE roundup(1100 * 1920 * 4 * 2, 0x10000)
#endif
#endif

#if defined(CONFIG_FB_MSM_MIPI_DSI)
#if defined(CONFIG_FB_MSM_MIPI_SAMSUNG_OLED_VIDEO_WVGA_PT_PANEL)
/* 480 x 800 x 3 x 2 */
#define MIPI_DSI_WRITEBACK_SIZE (480 * 800 * 3 * 2)
#elif defined(CONFIG_FB_MSM_MIPI_SAMSUNG_OLED_CMD_QHD_PT_PANEL)
/* 540 x 960 x 3 x 2 */
#define MIPI_DSI_WRITEBACK_SIZE (544 * 960 * 3 * 2)
#elif defined(CONFIG_FB_MSM_MIPI_SAMSUNG_OLED_VIDEO_HD_PT_PANEL)
/*1280 x732 x 3 x 2 */
#define MIPI_DSI_WRITEBACK_SIZE (1080 * 1920 * 3 * 2)
#endif
#else
#define MIPI_DSI_WRITEBACK_SIZE 0
#endif

#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE, 4096)

#ifdef CONFIG_FB_MSM_OVERLAY0_WRITEBACK
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE roundup((1920 * 1088 * 3 * 2), 4096)
#else
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE (0)
#endif  /* CONFIG_FB_MSM_OVERLAY0_WRITEBACK */

#ifdef CONFIG_FB_MSM_OVERLAY1_WRITEBACK
#define MSM_FB_OVERLAY1_WRITEBACK_SIZE roundup((1920 * 1088 * 3 * 2), 4096)
#else
#define MSM_FB_OVERLAY1_WRITEBACK_SIZE (0)
#endif  /* CONFIG_FB_MSM_OVERLAY1_WRITEBACK */


static struct resource msm_fb_resources[] = {
	{
		.flags = IORESOURCE_DMA,
	}
};

#define LVDS_CHIMEI_PANEL_NAME "lvds_chimei_wxga"
#define LVDS_FRC_PANEL_NAME "lvds_frc_fhd"
#define MIPI_VIDEO_TOSHIBA_WSVGA_PANEL_NAME "mipi_video_toshiba_wsvga"
#define MIPI_VIDEO_CHIMEI_WXGA_PANEL_NAME "mipi_video_chimei_wxga"
#define HDMI_PANEL_NAME "hdmi_msm"
#define MHL_PANEL_NAME "hdmi_msm,mhl_8334"
#define TVOUT_PANEL_NAME "tvout_msm"

#define LVDS_PIXEL_MAP_PATTERN_1	1
#define LVDS_PIXEL_MAP_PATTERN_2	2

#ifdef CONFIG_FB_MSM_HDMI_AS_PRIMARY
static unsigned char hdmi_is_primary = 1;
#else
static unsigned char hdmi_is_primary;
#endif

static unsigned char mhl_display_enabled;

unsigned char apq8064_hdmi_as_primary_selected(void)
{
	return hdmi_is_primary;
}

unsigned char apq8064_mhl_display_enabled(void)
{
	return mhl_display_enabled;
}

static void set_mdp_clocks_for_wuxga(void);

static int msm_fb_detect_panel(const char *name)
{
	u32 version;
	if (machine_is_apq8064_liquid()) {
		version = socinfo_get_platform_version();
		if ((SOCINFO_VERSION_MAJOR(version) == 1) &&
				(SOCINFO_VERSION_MINOR(version) == 1)) {
			if (!strncmp(name, MIPI_VIDEO_CHIMEI_WXGA_PANEL_NAME,
				strnlen(MIPI_VIDEO_CHIMEI_WXGA_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
				return 0;
		} else {
			if (!strncmp(name, LVDS_CHIMEI_PANEL_NAME,
				strnlen(LVDS_CHIMEI_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
				return 0;
		}
	} else if (machine_is_apq8064_mtp() || machine_is_JF()) {
		if (!strncmp(name, MIPI_VIDEO_TOSHIBA_WSVGA_PANEL_NAME,
				strnlen(MIPI_VIDEO_TOSHIBA_WSVGA_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
			return 0;
	} else if (machine_is_apq8064_cdp()) {
		if (!strncmp(name, LVDS_CHIMEI_PANEL_NAME,
					strnlen(LVDS_CHIMEI_PANEL_NAME,
						PANEL_NAME_MAX_LEN)))
			return 0;
	} else if (machine_is_mpq8064_dtv()) {
		if (!strncmp(name, LVDS_FRC_PANEL_NAME,
			strnlen(LVDS_FRC_PANEL_NAME,
			PANEL_NAME_MAX_LEN))) {
			set_mdp_clocks_for_wuxga();
			return 0;
		}
	}

	if (!strncmp(name, HDMI_PANEL_NAME,
				strnlen(HDMI_PANEL_NAME,
					PANEL_NAME_MAX_LEN))) {
		if (apq8064_hdmi_as_primary_selected())
			set_mdp_clocks_for_wuxga();
		return 0;
	}


	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name              = "msm_fb",
	.id                = 0,
	.num_resources     = ARRAY_SIZE(msm_fb_resources),
	.resource          = msm_fb_resources,
	.dev.platform_data = &msm_fb_pdata,
};

void __init apq8064_allocate_fb_region(void)
{
	void *addr;
	unsigned long size;

	size = MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
			size, addr, __pa(addr));
}

#define MDP_VSYNC_GPIO 0

static struct msm_bus_vectors mdp_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_ui_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 525312000 * 2,
		.ib = 656640000 * 2,
	},
};

static struct msm_bus_vectors mdp_vga_vectors[] = {
	/* VGA and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 525312000 * 2,
		.ib = 656640000 * 2,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 580608000 * 2,
		.ib = 725760000 * 2,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 407808000 * 4,
		.ib = 2159040000u,
	},
};

static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_ui_vectors),
		mdp_ui_vectors,
	},
	{
		ARRAY_SIZE(mdp_ui_vectors),
		mdp_ui_vectors,
	},
	{
		ARRAY_SIZE(mdp_vga_vectors),
		mdp_vga_vectors,
	},
	{
		ARRAY_SIZE(mdp_720p_vectors),
		mdp_720p_vectors,
	},
	{
		ARRAY_SIZE(mdp_1080p_vectors),
		mdp_1080p_vectors,
	},
};

static struct msm_bus_scale_pdata mdp_bus_scale_pdata = {
	mdp_bus_scale_usecases,
	ARRAY_SIZE(mdp_bus_scale_usecases),
	.name = "mdp",
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = MDP_VSYNC_GPIO,
	.mdp_max_clk = 266667000,
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
	.mdp_rev = MDP_REV_44,
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	.mem_hid = BIT(ION_CP_MM_HEAP_ID),
#else
	.mem_hid = MEMTYPE_EBI1,
#endif
	.mdp_iommu_split_domain = 1,
	.cont_splash_enabled = 0x01,
};

static char mipi_dsi_splash_is_enabled(void)
{
	return mdp_pdata.cont_splash_enabled;
}

void __init apq8064_mdp_writeback(struct memtype_reserve* reserve_table)
{
	mdp_pdata.ov0_wb_size = MSM_FB_OVERLAY0_WRITEBACK_SIZE;
	mdp_pdata.ov1_wb_size = MSM_FB_OVERLAY1_WRITEBACK_SIZE;
#if defined(CONFIG_ANDROID_PMEM) && !defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	reserve_table[mdp_pdata.mem_hid].size +=
		mdp_pdata.ov0_wb_size;
	reserve_table[mdp_pdata.mem_hid].size +=
		mdp_pdata.ov1_wb_size;
#endif
}

static struct resource hdmi_msm_resources[] = {
	{
		.name  = "hdmi_msm_qfprom_addr",
		.start = 0x00700000,
		.end   = 0x007060FF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_hdmi_addr",
		.start = 0x04A00000,
		.end   = 0x04A00FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_irq",
		.start = HDMI_IRQ,
		.end   = HDMI_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static int hdmi_enable_5v(int on);
static int hdmi_core_power(int on, int show);
static int hdmi_cec_power(int on);
static int hdmi_gpio_config(int on);
static int hdmi_panel_power(int on);

static struct msm_hdmi_platform_data hdmi_msm_data = {
	.irq = HDMI_IRQ,
	.enable_5v = hdmi_enable_5v,
	.core_power = hdmi_core_power,
	.cec_power = hdmi_cec_power,
	.panel_power = hdmi_panel_power,
	.gpio_config = hdmi_gpio_config,
};

static struct platform_device hdmi_msm_device = {
	.name = "hdmi_msm",
	.id = 0,
	.num_resources = ARRAY_SIZE(hdmi_msm_resources),
	.resource = hdmi_msm_resources,
	.dev.platform_data = &hdmi_msm_data,
};

static char wfd_check_mdp_iommu_split_domain(void)
{
	return mdp_pdata.mdp_iommu_split_domain;
}

#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
static struct msm_wfd_platform_data wfd_pdata = {
	.wfd_check_mdp_iommu_split = wfd_check_mdp_iommu_split_domain,
};

static struct platform_device wfd_panel_device = {
	.name = "wfd_panel",
	.id = 0,
	.dev.platform_data = NULL,
};

static struct platform_device wfd_device = {
	.name          = "msm_wfd",
	.id            = -1,
	.dev.platform_data = &wfd_pdata,
};
#endif

/* HDMI related GPIOs */
#define HDMI_CEC_VAR_GPIO	31
#if !defined(CONFIG_SAMSUNG_MHL_8240)
#define HDMI_DDC_CLK_GPIO	70
#define HDMI_DDC_DATA_GPIO	71
#endif
#define HDMI_HPD_GPIO		72

static bool dsi_power_on;
static bool oled_power_on;

/* [junesok] Power on for samsung oled */
#if defined(CONFIG_MACH_JACTIVE_EUR)
#define LCD_22V_EN	33
#define LCD_22V_EN_2	20
#define PMIC_GPIO_LED_DRIVER 31
#elif defined(CONFIG_MACH_JACTIVE_ATT)
#define LCD_22V_EN	33
#define LCD_22V_EN_2	20
#define PMIC_GPIO_LED_DRIVER_REV00 28
#define PMIC_GPIO_LED_DRIVER_REV10 31
#else
#define LCD_22V_EN	69
#define PMIC_GPIO_LED_DRIVER 27
#endif
#define PMIC_GPIO_LCD_RST 43

enum DSI_BLOCK_POWER {
	MIPI_DSI_BLOCK_ON,
	MIPI_DSI_BLOCK_OFF,
	MIPI_DSI_BLOCK_NOTHING,
};

enum PANEL_BLOCK_POWR {
	MIPI_PANEL_BLOCK_ON,
	MIPI_PANEL_BLOCK_OFF,
	MIPI_PANEL_BLOCK_NOTHING,
};

void pull_ldi_reset_down(void)
{
	int gpio43 = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_LCD_RST);
	gpio_direction_output(gpio43, 0); /*RESETB*/
}

void pull_ldi_reset_up(void)
{
	int gpio43 = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_LCD_RST);
	gpio_direction_output(gpio43, 1); /*RESETB*/
}

#if defined(CONFIG_SUPPORT_DISPLAY_OCTA_TFT)
static int gpio43;	/* MLCD_RST */
static int gpio27;	/* LED_DRIVER */
#if defined(CONFIG_MACH_JACTIVE_ATT)
static int gpio33;
#endif
static struct regulator *reg_L15;
#if defined(CONFIG_MACH_JACTIVE_EUR) || defined(CONFIG_MACH_JACTIVE_ATT)
static struct regulator *reg_L16;
#endif
#if defined(CONFIG_MACH_JACTIVE_EUR) || defined(CONFIG_MACH_JACTIVE)
static struct regulator *reg_LVS1;
#endif
#else
static struct regulator *reg_L30;
#endif

static struct regulator *reg_l2;
#ifdef CONFIG_LCD_VDD3_BY_PMGPIO
int pmic_gpio4;
#else
static struct regulator *reg_L15;
#endif

#define MLCD_RST_MPP2 2
struct pm_gpio gpio43_param = {
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

static struct pm8xxx_mpp_config_data MLCD_RESET_HIGH_CONFIG = {
	.type		= PM8XXX_MPP_TYPE_D_OUTPUT,
	.level		= PM8XXX_MPP_DIG_LEVEL_VIO_1,
	.control		= PM8XXX_MPP_DOUT_CTRL_HIGH,
};

static struct pm8xxx_mpp_config_data MLCD_RESET_LOW_CONFIG = {
	.type		= PM8XXX_MPP_TYPE_D_OUTPUT,
	.level		= PM8XXX_MPP_DIG_LEVEL_VIO_1,
	.control		= PM8XXX_MPP_DOUT_CTRL_LOW,
};

static int mipi_dsi_power(int enable)
{
	int rc = 0;

	if (enable) {

		pr_info("[lcd] DSI ON\n");
		rc = regulator_set_optimum_mode(reg_l2, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode L2 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_enable(reg_l2);
		if (rc) {
			pr_err("enable L2 failed, rc=%d\n", rc);
			return -ENODEV;
		}
	} else {

		pr_info("[lcd] DSI OFF\n");
		rc = regulator_set_optimum_mode(reg_l2, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode L2 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_disable(reg_l2);
		if (rc) {
			pr_err("disable reg_L2 failed, rc=%d\n", rc);
			return -ENODEV;
		}
	}

	return rc;

}

#if defined(CONFIG_SUPPORT_DISPLAY_OCTA_TFT)
static int mipi_dsi_power_tft_request(void)
{
	int rc = 0;
#if defined(CONFIG_MACH_JACTIVE_ATT) 
	if (system_rev < 10){
		gpio33 = PM8921_GPIO_PM_TO_SYS(LCD_22V_EN);
		rc = gpio_request(gpio33, "led_dirver");
		if (rc) {
			pr_err("request gpio lcd_22v_en failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = pm8xxx_gpio_config(gpio33, &gpio43_param);
		if (rc) {
			pr_err("gpio_config lcd_22v_en failed (3), rc=%d\n", rc);
			return -EINVAL;
		}
	}
	else{
		pr_info("[lcd] request gpio lcd_22v_en\n");
		rc = gpio_request(LCD_22V_EN, "lcd_22v_en");

		if (rc) {
			gpio_free(LCD_22V_EN);
			rc = gpio_request(LCD_22V_EN, "lcd_22v_en");
			if(rc){
				pr_err("request gpio lcd_22v_en failed, rc=%d\n", rc);
				return -ENODEV;
			}
		}

		pr_info("[lcd] configure LCD_22V_EN\n");
		gpio_tlmm_config(GPIO_CFG(LCD_22V_EN,  0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

		if( system_rev >= 13 ) // rev0.5 + 8
		{
			pr_info("[lcd] request gpio lcd_22v_en_2\n");
			rc = gpio_request(LCD_22V_EN_2, "lcd_22v_en_2");
			if (rc) {
				gpio_free(LCD_22V_EN_2);
				rc = gpio_request(LCD_22V_EN_2, "lcd_22v_en_2");
				if(rc){
					pr_err("request gpio lcd_22v_en_2 failed, rc=%d\n", rc);
					return -ENODEV;
				}
			}
			pr_info("[lcd] configure LCD_22V_EN_2\n");
			gpio_tlmm_config(GPIO_CFG(LCD_22V_EN_2,  0, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		}
	}
#else
	pr_info("[lcd] request gpio lcd_22v_en\n");
	rc = gpio_request(LCD_22V_EN, "lcd_22v_en");

	if (rc) {
		pr_err("request gpio lcd_22v_en failed, rc=%d\n", rc);
		return -ENODEV;
	}

	pr_info("[lcd] configure LCD_22V_EN\n");
	gpio_tlmm_config(GPIO_CFG(LCD_22V_EN,  0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	if( system_rev >= 16 ) // rev0.6 + 10
	{
		pr_info("[lcd] request gpio lcd_22v_en_2\n");
		rc = gpio_request(LCD_22V_EN_2, "lcd_22v_en_2");
		if (rc) {
			pr_err("request gpio lcd_22v_en_2 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		pr_info("[lcd] configure LCD_22V_EN_2\n");
		gpio_tlmm_config(GPIO_CFG(LCD_22V_EN_2,  0, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	}
#endif

	if (system_rev == 0) {
		gpio43 = PM8921_GPIO_PM_TO_SYS(
						PMIC_GPIO_LCD_RST);

		rc = gpio_request(gpio43, "mlcd_rst");
		if (rc) {
			pr_err("request gpio mlcd_rst failed, rc=%d\n", rc);
			return -ENODEV;
		}

		rc = pm8xxx_gpio_config(gpio43, &gpio43_param);
		if (rc) {
			pr_err("gpio_config mlcd_rst failed (3), rc=%d\n", rc);
			return -EINVAL;
		}
	}
#if defined(CONFIG_MACH_JACTIVE_ATT)
	if (system_rev < 10)
		gpio27 = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_LED_DRIVER_REV00);
	else
		gpio27 = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_LED_DRIVER_REV10);
#else
	gpio27 = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_LED_DRIVER);
#endif
		

	rc = gpio_request(gpio27, "led_dirver");
	if (rc) {
		pr_err("request gpio led_dirver failed, rc=%d\n", rc);
		return -ENODEV;
	}

	rc = pm8xxx_gpio_config(gpio27, &gpio43_param);
	if (rc) {
		pr_err("gpio_config led_dirver failed (3), rc=%d\n", rc);
		return -EINVAL;
	}
#if defined(CONFIG_MACH_JACTIVE_ATT)
	if(system_rev < 10)
		gpio_direction_output(gpio33, 0);
	else
	{
		gpio_direction_output(LCD_22V_EN, 0);
		if( system_rev >= 13 ) // rev0.5 + 8
		{
			mdelay(10);
			gpio_direction_output(LCD_22V_EN_2, 0);
		}
	}
#else
	gpio_direction_output(LCD_22V_EN, 0);
	if( system_rev >= 16 ) // rev0.6 + 10
	{
		mdelay(10);
		gpio_direction_output(LCD_22V_EN_2, 0);
	}
#endif
	if (system_rev == 0)
		gpio_direction_output(gpio43, 0);
	else
		pm8xxx_mpp_config(
			PM8921_MPP_PM_TO_SYS(MLCD_RST_MPP2),
			&MLCD_RESET_LOW_CONFIG);

	msleep(1000);

	gpio_direction_output(gpio27, 0);

	return rc;
}

static int mipi_panel_power_tft(int enable)
{
	int rc = 0;

	pr_info("%s %d", __func__, enable);
	if (enable) {
#if defined(CONFIG_MACH_JACTIVE_EUR)
		rc = regulator_set_optimum_mode(reg_LVS1, 100000); /*IOVDD */
		if (rc < 0) {
			pr_err("set_optimum_mode LVS1 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_enable(reg_LVS1);
		if (rc) {
			pr_err("enable LVS1 failed, rc=%d\n", rc);
			return -ENODEV;
		}
#elif defined(CONFIG_MACH_JACTIVE)
		if (system_rev >= 11) {
			rc = regulator_set_optimum_mode(reg_LVS1, 100000); /*IOVDD */
			if (rc < 0) {
				pr_err("set_optimum_mode LVS1 failed, rc=%d\n", rc);
				return -EINVAL;
			}
			rc = regulator_enable(reg_LVS1);
			if (rc) {
				pr_err("enable LVS1 failed, rc=%d\n", rc);
				return -ENODEV;
			}
		}
#endif

		rc = regulator_set_optimum_mode(reg_L15, 100000); /*IOVDD */
		if (rc < 0) {
			pr_err("set_optimum_mode L15 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_enable(reg_L15);
		if (rc) {
			pr_err("enable L15 failed, rc=%d\n", rc);
			return -ENODEV;
		}

#if defined(CONFIG_MACH_JACTIVE_ATT)
	if(system_rev < 10)
		gpio_direction_output(gpio33, 1);
	else
	{
		gpio_direction_output(LCD_22V_EN, 1);
		if( system_rev >= 13 ) // rev0.5 + 8
		{
			mdelay(10);
			gpio_direction_output(LCD_22V_EN_2, 1);
		}
	}
#else
		gpio_direction_output(LCD_22V_EN, 1);
		if( system_rev >= 16 ) // rev0.6 + 10
		{
			mdelay(10);
			gpio_direction_output(LCD_22V_EN_2, 1);
		}
#endif

		msleep(20);

#if defined(CONFIG_MACH_JACTIVE_EUR)
		if( system_rev >= 15 ) // rev0.5 + 10
		{
			rc = regulator_set_optimum_mode(reg_L16, 100000); /*IOVDD */
			if (rc < 0) {
				pr_err("set_optimum_mode L16 failed, rc=%d\n", rc);
				return -EINVAL;
			}
			rc = regulator_enable(reg_L16);
			if (rc) {
				pr_err("enable L16 failed, rc=%d\n", rc);
				return -ENODEV;
			}
			
			msleep ( 10 );
		}
#elif defined(CONFIG_MACH_JACTIVE_ATT)
		if( system_rev >= 11 ) // rev0.3 + 8
		{
			rc = regulator_set_optimum_mode(reg_L16, 100000); /*IOVDD */
			if (rc < 0) {
				pr_err("set_optimum_mode L16 failed, rc=%d\n", rc);
				return -EINVAL;
			}
			rc = regulator_enable(reg_L16);
			if (rc) {
				pr_err("enable L16 failed, rc=%d\n", rc);
				return -ENODEV;
			}
			
			msleep ( 10 );
		}
#endif
		gpio_direction_output(gpio27, 1);/*LED_DRIVER(gpio27);*/
	} else {
		if (system_rev == 0)
			gpio_direction_output(gpio43, 0);
		else
			pm8xxx_mpp_config(
				PM8921_MPP_PM_TO_SYS(MLCD_RST_MPP2),
				&MLCD_RESET_LOW_CONFIG);
#if defined(CONFIG_MACH_JACTIVE_ATT)
		if(system_rev < 10)
			gpio_direction_output(gpio33, 0);
		else
		{
			if( system_rev >= 13 ) // rev0.5 + 8
			{
				gpio_direction_output(LCD_22V_EN_2, 0);
				mdelay(10);
			}
			gpio_direction_output(LCD_22V_EN, 0);
		}
#else
		if( system_rev >= 16 ) // rev0.6 + 10
		{
			gpio_direction_output(LCD_22V_EN_2, 0);
			mdelay(10);
		}
		gpio_direction_output(LCD_22V_EN, 0);
#endif
		usleep(2000); /*1ms delay(minimum) required between VDD off and AVDD off*/

		rc = regulator_set_optimum_mode(reg_L15, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode L15 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_disable(reg_L15);
		if (rc) {
			pr_err("disable reg_L15 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		gpio_direction_output(gpio27, 0);/*LED_DRIVER(gpio27);*/

#if defined(CONFIG_MACH_JACTIVE_EUR)
		if( system_rev >= 15 ) // rev0.5 + 10
		{
			msleep ( 10 );
			rc = regulator_set_optimum_mode(reg_L16, 100);
			if (rc < 0) {
				pr_err("set_optimum_mode L16 failed, rc=%d\n", rc);
				return -EINVAL;
			}
			
			rc = regulator_disable(reg_L16);
			if (rc) {
				pr_err("disable reg_L16 failed, rc=%d\n", rc);
				return -ENODEV;
			}
		}
#elif defined(CONFIG_MACH_JACTIVE_ATT)
		if( system_rev >= 11 ) // rev0.3 + 8
		{
			msleep ( 10 );
			rc = regulator_set_optimum_mode(reg_L16, 100);
			if (rc < 0) {
				pr_err("set_optimum_mode L16 failed, rc=%d\n", rc);
				return -EINVAL;
			}
			
			rc = regulator_disable(reg_L16);
			if (rc) {
				pr_err("disable reg_L16 failed, rc=%d\n", rc);
				return -ENODEV;
			}
		}
#endif

#if defined(CONFIG_MACH_JACTIVE_EUR)
		rc = regulator_set_optimum_mode(reg_LVS1, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode LVS1 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_disable(reg_LVS1);
		if (rc) {
			pr_err("disable reg_LVS1 failed, rc=%d\n", rc);
			return -ENODEV;
		}
#elif defined(CONFIG_MACH_JACTIVE)
		if (system_rev >= 11) {
			rc = regulator_set_optimum_mode(reg_LVS1, 100);
			if (rc < 0) {
				pr_err("set_optimum_mode LVS1 failed, rc=%d\n", rc);
				return -EINVAL;
			}
			rc = regulator_disable(reg_LVS1);
			if (rc) {
				pr_err("disable reg_LVS1 failed, rc=%d\n", rc);
				return -ENODEV;
			}
		}
#endif
		msleep(20);
	}

	return rc;
}
#else
static int mipi_dsi_power_octa_request(void)
{
	int rc = 0;

	reg_L30 = regulator_get(&msm_mipi_dsi1_device.dev,
				"3000mV_LCD");
	if (IS_ERR_OR_NULL(reg_L30)) {
		pr_err("could not get 8917_L30, rc = %ld\n",
				PTR_ERR(reg_L30));
		return -ENODEV;
	}

	rc = regulator_set_voltage(reg_L30, 3000000, 3000000);
	if (rc) {
		pr_err("set_voltage L30 failed, rc=%d\n", rc);
		return -EINVAL;
	}

	pm8xxx_mpp_config(PM8921_MPP_PM_TO_SYS(MLCD_RST_MPP2),
			&MLCD_RESET_LOW_CONFIG);

	msleep(100);

	return rc;
}

static int mipi_panel_power_oled(int enable)
{
	int rc = 0;

	if (enable) {

		pr_info("[lcd] PANEL ON\n");

		/* 3000mv VCI(ANALOG) */
		rc = regulator_set_optimum_mode(reg_L30, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode l30 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_enable(reg_L30);
		if (rc) {
			pr_err("enable l30 failed, rc=%d\n", rc);
			return -ENODEV;
		}

#ifdef CONFIG_LCD_VDD3_BY_PMGPIO
		gpio_set_value_cansleep(pmic_gpio4, 1);
#else
		/* 1800mv VDD3(IO) */
		rc = regulator_set_optimum_mode(reg_L15, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode L15 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_enable(reg_L15);
		if (rc) {
			pr_err("enable L15 failed, rc=%d\n", rc);
			return -ENODEV;
		}
#endif
	} else {

		pr_info("[lcd] PANEL OFF\n");

#ifdef CONFIG_LCD_VDD3_BY_PMGPIO
		gpio_set_value_cansleep(pmic_gpio4, 0);
#else
		rc = regulator_set_optimum_mode(reg_L15, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode L15 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_disable(reg_L15);
		if (rc) {
			pr_err("disable reg_L15 failed, rc=%d\n", rc);
			return -ENODEV;
		}
#endif
		rc = regulator_set_optimum_mode(reg_L30, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode L30 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_disable(reg_L30);
		if (rc) {
			pr_err("disable reg_L30 failed, rc=%d\n", rc);
			return -ENODEV;
		}
	}

	return rc;

}
#endif

void active_reset(int high)
{
	if (high) {
		pm8xxx_mpp_config(
			PM8921_MPP_PM_TO_SYS(MLCD_RST_MPP2),
			&MLCD_RESET_HIGH_CONFIG);
	} else {
		pm8xxx_mpp_config(
			PM8921_MPP_PM_TO_SYS(MLCD_RST_MPP2),
			&MLCD_RESET_LOW_CONFIG);
	}
}

static int mipi_dsi_power_set(void)
{
	int rc = 0;

	if (!dsi_power_on) {

		pr_info("[lcd] DSI REGULATOR SET ...\n");

		reg_l2 = regulator_get(&msm_mipi_dsi1_device.dev,
				"dsi1_pll_vdda");
		if (IS_ERR_OR_NULL(reg_l2)) {
			pr_err("could not get 8917_L2, rc = %ld\n",
					PTR_ERR(reg_l2));
			return -ENODEV;
		}

		rc = regulator_set_voltage(reg_l2, 1200000, 1200000);
		if (rc) {
			pr_err("set_voltage L2 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		dsi_power_on = true;
	}

	return 0;
}

static int mipi_oled_power_set(void)
{
	int rc = 0;

	if (!oled_power_on) {

		pr_info("[lcd] PANEL REGULATOR SET ...\n");

#ifdef CONFIG_LCD_VDD3_BY_PMGPIO
		pmic_gpio4 = PM8921_GPIO_PM_TO_SYS(4);
		rc = gpio_request(pmic_gpio4, "vdd3_IO_1800mV");
		if (rc) {
			pr_err("request APPMIC gpio 4 failed, rc=%d\n", rc);
			return -ENODEV;
		}
#else
		/* VDD3 */
		reg_L15 = regulator_get(&msm_mipi_dsi1_device.dev,
				"1800mV_LCD");
		if (IS_ERR_OR_NULL(reg_L15)) {
			pr_err("could not get 8917_L15, rc = %ld\n",
					PTR_ERR(reg_L15));
			return -ENODEV;
		}

		rc = regulator_set_voltage(reg_L15, 1800000, 1800000);
		if (rc) {
			pr_err("set_voltage L15 failed, rc=%d\n", rc);
			return -EINVAL;
		}

#if defined(CONFIG_MACH_JACTIVE_EUR)
		if( system_rev >= 15 ) // rev0.5 + 10
		{
			reg_L16 = regulator_get(NULL, "8921_l16");
			if (IS_ERR_OR_NULL(reg_L16)) {
				pr_err("could not get 8921_L16, rc = %ld\n",
						PTR_ERR(reg_L16));
				return -ENODEV;
			}
			
			rc = regulator_set_voltage(reg_L16, 3000000, 3000000);
			if (rc) {
				pr_err("set_voltage L16 failed, rc=%d\n", rc);
				return -EINVAL;
			}
		}
#elif defined(CONFIG_MACH_JACTIVE_ATT)
		if( system_rev >= 11 ) // rev0.3 + 8
		{
			reg_L16 = regulator_get(NULL, "8921_l16");
			if (IS_ERR_OR_NULL(reg_L16)) {
				pr_err("could not get 8921_L16, rc = %ld\n",
						PTR_ERR(reg_L16));
				return -ENODEV;
			}
			
			rc = regulator_set_voltage(reg_L16, 3000000, 3000000);
			if (rc) {
				pr_err("set_voltage L16 failed, rc=%d\n", rc);
				return -EINVAL;
			}
		}
#endif

#if defined(CONFIG_MACH_JACTIVE_EUR)
		reg_LVS1 = regulator_get(NULL, "8921_lvs1");
		if (IS_ERR_OR_NULL(reg_LVS1)) {
			pr_err("could not get 8917_LVS1, rc = %ld\n",
					PTR_ERR(reg_LVS1));
			return -ENODEV;
		}
#elif defined(CONFIG_MACH_JACTIVE)
		if (system_rev >=11) {
			reg_LVS1 = regulator_get(NULL, "8921_lvs1");
			if (IS_ERR_OR_NULL(reg_LVS1)) {
				pr_err("could not get 8917_LVS1, rc = %ld\n",
						PTR_ERR(reg_LVS1));
				return -ENODEV;
			}
		}
#endif
#endif

#if defined(CONFIG_SUPPORT_DISPLAY_OCTA_TFT)
		mipi_dsi_power_tft_request();
#else
		mipi_dsi_power_octa_request();
#endif

		oled_power_on = true;
	}

	return 0;
}

static int mipi_power_samsung_common(void)
{
	int ret = 0;

	ret = mipi_dsi_power_set();
	if (ret) {
		pr_err("mipi_dsi_power_set error!\n");
		return 1;
	}

	ret = mipi_oled_power_set();
	if (ret) {
		pr_err("mipi_dsi_power_set error!\n");
		return 1;
	}

	return 0;
}

static int mipi_dsi_power_samsung(int on)
{
	if (on)
		mipi_dsi_power(1);
	else
		mipi_dsi_power(0);

	return 0;
}

#if defined(CONFIG_SUPPORT_SECOND_POWER)
static int mipi_panel_power_samsung(int on)
{
	int rc;
	rc = 0;

#ifdef CONFIG_SUPPORT_DISPLAY_OCTA_TFT
	if (on)
		rc = mipi_panel_power_tft(1);
	else
		rc = mipi_panel_power_tft(0);
#else
	if (on)
		rc = mipi_panel_power_oled(1);
	else
		rc = mipi_panel_power_oled(0);
	
#endif
	return rc;
}
#endif

static char mipi_dsi_splash_is_enabled(void);

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.power_common = mipi_power_samsung_common,
	.dsi_power_save = mipi_dsi_power_samsung,
#if defined(CONFIG_SUPPORT_SECOND_POWER)
	.panel_power_save = mipi_panel_power_samsung,
#endif
	.splash_is_enabled = mipi_dsi_splash_is_enabled,
	.active_reset = active_reset,
};

static struct platform_device mipi_dsi_samsung_full_hd_panel_device = {
	.name = "mipi_samsung_full_hd",
	.id = 0,
	.dev.platform_data = &mipi_dsi_pdata,
};

static bool lvds_power_on;
static int lvds_panel_power(int on)
{
	static struct regulator *reg_lvs7, *reg_l2, *reg_ext_3p3v;
	static int gpio36, gpio26, mpp3;
	int rc;

	pr_debug("%s: on=%d\n", __func__, on);

	if (!lvds_power_on) {
		reg_lvs7 = regulator_get(&msm_lvds_device.dev,
				"lvds_vdda");
		if (IS_ERR_OR_NULL(reg_lvs7)) {
			pr_err("could not get 8921_lvs7, rc = %ld\n",
					PTR_ERR(reg_lvs7));
			return -ENODEV;
		}

		reg_l2 = regulator_get(&msm_lvds_device.dev,
				"lvds_pll_vdda");
		if (IS_ERR_OR_NULL(reg_l2)) {
			pr_err("could not get 8921_l2, rc = %ld\n",
					PTR_ERR(reg_l2));
			return -ENODEV;
		}

		rc = regulator_set_voltage(reg_l2, 1200000, 1200000);
		if (rc) {
			pr_err("set_voltage l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		reg_ext_3p3v = regulator_get(&msm_lvds_device.dev,
				"lvds_vccs_3p3v");
		if (IS_ERR_OR_NULL(reg_ext_3p3v)) {
			pr_err("could not get reg_ext_3p3v, rc = %ld\n",
					PTR_ERR(reg_ext_3p3v));
			return -ENODEV;
		}

		gpio26 = PM8921_GPIO_PM_TO_SYS(26);
		rc = gpio_request(gpio26, "pwm_backlight_ctrl");
		if (rc) {
			pr_err("request gpio 26 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		gpio36 = PM8921_GPIO_PM_TO_SYS(36); /* lcd1_pwr_en_n */
		rc = gpio_request(gpio36, "lcd1_pwr_en_n");
		if (rc) {
			pr_err("request gpio 36 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		mpp3 = PM8921_MPP_PM_TO_SYS(3);
		rc = gpio_request(mpp3, "backlight_en");
		if (rc) {
			pr_err("request mpp3 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		lvds_power_on = true;
	}

	if (on) {
		rc = regulator_enable(reg_lvs7);
		if (rc) {
			pr_err("enable lvs7 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		rc = regulator_set_optimum_mode(reg_l2, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_enable(reg_l2);
		if (rc) {
			pr_err("enable l2 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		rc = regulator_enable(reg_ext_3p3v);
		if (rc) {
			pr_err("enable reg_ext_3p3v failed, rc=%d\n", rc);
			return -ENODEV;
		}

		gpio_set_value_cansleep(gpio36, 0);
		gpio_set_value_cansleep(mpp3, 1);
		if (socinfo_get_pmic_model() == PMIC_MODEL_PM8917)
			gpio_set_value_cansleep(gpio26, 1);
	} else {
		if (socinfo_get_pmic_model() == PMIC_MODEL_PM8917)
			gpio_set_value_cansleep(gpio26, 0);
		gpio_set_value_cansleep(mpp3, 0);
		gpio_set_value_cansleep(gpio36, 1);

		rc = regulator_disable(reg_lvs7);
		if (rc) {
			pr_err("disable reg_lvs7 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_l2);
		if (rc) {
			pr_err("disable reg_l2 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_ext_3p3v);
		if (rc) {
			pr_err("disable reg_ext_3p3v failed, rc=%d\n", rc);
			return -ENODEV;
		}
	}

	return 0;
}

static int lvds_pixel_remap(void)
{
	u32 ver = socinfo_get_version();

	if (machine_is_apq8064_cdp() ||
			machine_is_apq8064_liquid()) {
		if ((SOCINFO_VERSION_MAJOR(ver) == 1) &&
				(SOCINFO_VERSION_MINOR(ver) == 0))
			return LVDS_PIXEL_MAP_PATTERN_1;
	} else if (machine_is_mpq8064_dtv()) {
		if ((SOCINFO_VERSION_MAJOR(ver) == 1) &&
		    (SOCINFO_VERSION_MINOR(ver) == 0))
			return LVDS_PIXEL_MAP_PATTERN_2;
	}
	return 0;
}

static struct lcdc_platform_data lvds_pdata = {
	.lcdc_power_save = lvds_panel_power,
	.lvds_pixel_remap = lvds_pixel_remap
};

#define LPM_CHANNEL 2
static int lvds_chimei_gpio[] = {LPM_CHANNEL};

static struct lvds_panel_platform_data lvds_chimei_pdata = {
	.gpio = lvds_chimei_gpio,
};

static struct platform_device lvds_chimei_panel_device = {
	.name = "lvds_chimei_wxga",
	.id = 0,
	.dev = {
		.platform_data = &lvds_chimei_pdata,
	}
};

#define FRC_GPIO_UPDATE	(SX150X_EXP4_GPIO_BASE + 8)
#define FRC_GPIO_RESET	(SX150X_EXP4_GPIO_BASE + 9)
#define FRC_GPIO_PWR	(SX150X_EXP4_GPIO_BASE + 10)

static int lvds_frc_gpio[] = {FRC_GPIO_UPDATE, FRC_GPIO_RESET, FRC_GPIO_PWR};
static struct lvds_panel_platform_data lvds_frc_pdata = {
	.gpio = lvds_frc_gpio,
};

static struct platform_device lvds_frc_panel_device = {
	.name = "lvds_frc_fhd",
	.id = 0,
	.dev = {
		.platform_data = &lvds_frc_pdata,
	}
};

static int dsi2lvds_gpio[2] = {
	LPM_CHANNEL,/* Backlight PWM-ID=0 for PMIC-GPIO#24 */
	0x1F08 /* DSI2LVDS Bridge GPIO Output, mask=0x1f, out=0x08 */
};
static struct msm_panel_common_pdata mipi_dsi2lvds_pdata = {
	.gpio_num = dsi2lvds_gpio,
};

static struct platform_device mipi_dsi2lvds_bridge_device = {
	.name = "mipi_tc358764",
	.id = 0,
	.dev.platform_data = &mipi_dsi2lvds_pdata,
};

static int toshiba_gpio[] = {LPM_CHANNEL};
static struct mipi_dsi_panel_platform_data toshiba_pdata = {
	.gpio = toshiba_gpio,
};

static struct platform_device mipi_dsi_toshiba_panel_device = {
	.name = "mipi_toshiba",
	.id = 0,
	.dev = {
		.platform_data = &toshiba_pdata,
	}
};

static struct msm_bus_vectors dtv_bus_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors dtv_bus_def_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 566092800 * 2,
		.ib = 707616000 * 2,
	},
};

#if defined(CONFIG_MACH_JACTIVE_ATT) || defined(CONFIG_MACH_JACTIVE_EUR)
static struct msm_bus_vectors dtv_bus_cam_override_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 707616000 * 2,
	},
};
#endif

#if defined(CONFIG_MACH_JACTIVE_ATT) || defined(CONFIG_MACH_JACTIVE_EUR)
static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_cam_override_vectors),
		dtv_bus_cam_override_vectors,
	},
};
#else
static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
};
#endif

static struct msm_bus_scale_pdata dtv_bus_scale_pdata = {
	dtv_bus_scale_usecases,
	ARRAY_SIZE(dtv_bus_scale_usecases),
	.name = "dtv",
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
	.lcdc_power_save = hdmi_panel_power,
};

static int hdmi_panel_power(int on)
{
	int rc;

	pr_debug("%s: HDMI Core: %s\n", __func__, (on ? "ON" : "OFF"));
	rc = hdmi_core_power(on, 1);
	if (rc)
		rc = hdmi_cec_power(on);

	pr_debug("%s: HDMI Core: %s Success\n", __func__, (on ? "ON" : "OFF"));
	return rc;
}

static int hdmi_enable_5v(int on)
{
	/* TBD: PM8921 regulator instead of 8901 */
	static struct regulator *reg_8921_hdmi_mvs;	/* HDMI_5V */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8921_hdmi_mvs) {
		reg_8921_hdmi_mvs = regulator_get(&hdmi_msm_device.dev,
				"hdmi_mvs");
		if (IS_ERR(reg_8921_hdmi_mvs)) {
			pr_err("could not get reg_8921_hdmi_mvs, rc = %ld\n",
					PTR_ERR(reg_8921_hdmi_mvs));
			reg_8921_hdmi_mvs = NULL;
			return -ENODEV;
		}
	}

	if (on) {
		rc = regulator_enable(reg_8921_hdmi_mvs);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
					"8921_hdmi_mvs", rc);
			return rc;
		}
		pr_debug("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8921_hdmi_mvs);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
					"8921_hdmi_mvs", rc);
		pr_debug("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
}

static int hdmi_core_power(int on, int show)
{
	static struct regulator *reg_8921_lvs7, *reg_8921_s4, *reg_ext_3p3v;
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	/* TBD: PM8921 regulator instead of 8901 */
	if (!reg_ext_3p3v) {
		reg_ext_3p3v = regulator_get(&hdmi_msm_device.dev,
				"hdmi_mux_vdd");
		if (IS_ERR_OR_NULL(reg_ext_3p3v)) {
			pr_err("could not get reg_ext_3p3v, rc = %ld\n",
					PTR_ERR(reg_ext_3p3v));
			reg_ext_3p3v = NULL;
			return -ENODEV;
		}
	}

	if (!reg_8921_lvs7) {
		reg_8921_lvs7 = regulator_get(&hdmi_msm_device.dev,
				"hdmi_vdda");
		if (IS_ERR(reg_8921_lvs7)) {
			pr_err("could not get reg_8921_lvs7, rc = %ld\n",
					PTR_ERR(reg_8921_lvs7));
			reg_8921_lvs7 = NULL;
			return -ENODEV;
		}
	}
	if (!reg_8921_s4) {
		reg_8921_s4 = regulator_get(&hdmi_msm_device.dev,
				"hdmi_lvl_tsl");
		if (IS_ERR(reg_8921_s4)) {
			pr_err("could not get reg_8921_s4, rc = %ld\n",
					PTR_ERR(reg_8921_s4));
			reg_8921_s4 = NULL;
			return -ENODEV;
		}
		rc = regulator_set_voltage(reg_8921_s4, 1800000, 1800000);
		if (rc) {
			pr_err("set_voltage failed for 8921_s4, rc=%d\n", rc);
			return -EINVAL;
		}
	}

	if (on) {
		/*
		 * Configure 3P3V_BOOST_EN as GPIO, 8mA drive strength,
		 * pull none, out-high
		 */
		rc = regulator_set_optimum_mode(reg_ext_3p3v, 290000);
		if (rc < 0) {
			pr_err("set_optimum_mode ext_3p3v failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_enable(reg_ext_3p3v);
		if (rc) {
			pr_err("enable reg_ext_3p3v failed, rc=%d\n", rc);
			return rc;
		}
		rc = regulator_enable(reg_8921_lvs7);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
					"hdmi_vdda", rc);
			goto error1;
		}
		rc = regulator_enable(reg_8921_s4);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
					"hdmi_lvl_tsl", rc);
			goto error2;
		}
		pr_debug("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_ext_3p3v);
		if (rc) {
			pr_err("disable reg_ext_3p3v failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_8921_lvs7);
		if (rc) {
			pr_err("disable reg_8921_l23 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_8921_s4);
		if (rc) {
			pr_err("disable reg_8921_s4 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		pr_debug("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;

error2:
	regulator_disable(reg_8921_lvs7);
error1:
	regulator_disable(reg_ext_3p3v);
	return rc;
}

static int hdmi_gpio_config(int on)
{
	int rc = 0;
	static int prev_on;
	int pmic_gpio14 = PM8921_GPIO_PM_TO_SYS(14);

	if (on == prev_on)
		return 0;

	if (on) {
#if !defined(CONFIG_SAMSUNG_MHL_8240)
		rc = gpio_request(HDMI_DDC_CLK_GPIO, "HDMI_DDC_CLK");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_DDC_CLK", HDMI_DDC_CLK_GPIO, rc);
			goto error1;
		}
		rc = gpio_request(HDMI_DDC_DATA_GPIO, "HDMI_DDC_DATA");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_DDC_DATA", HDMI_DDC_DATA_GPIO, rc);
			goto error2;
		}
#endif
		rc = gpio_request(HDMI_HPD_GPIO, "HDMI_HPD");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
					"HDMI_HPD", HDMI_HPD_GPIO, rc);
#if !defined(CONFIG_SAMSUNG_MHL_8240)
			goto error3;
#else
			goto error1;
#endif
		}
		if (machine_is_apq8064_liquid()) {
			rc = gpio_request(pmic_gpio14, "PMIC_HDMI_MUX_SEL");
			if (rc) {
				pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
						"PMIC_HDMI_MUX_SEL", 14, rc);
				goto error4;
			}
			gpio_set_value_cansleep(pmic_gpio14, 0);
		}
		pr_debug("%s(on): success\n", __func__);
	} else {
#if !defined(CONFIG_SAMSUNG_MHL_8240)
		gpio_free(HDMI_DDC_CLK_GPIO);
		gpio_free(HDMI_DDC_DATA_GPIO);
#endif
		gpio_free(HDMI_HPD_GPIO);

		if (machine_is_apq8064_liquid()) {
			gpio_set_value_cansleep(pmic_gpio14, 1);
			gpio_free(pmic_gpio14);
		}
		pr_debug("%s(off): success\n", __func__);
	}

	prev_on = on;
	return 0;

error4:
	gpio_free(HDMI_HPD_GPIO);
#if !defined(CONFIG_SAMSUNG_MHL_8240)
error3:
	gpio_free(HDMI_DDC_DATA_GPIO);
error2:
	gpio_free(HDMI_DDC_CLK_GPIO);
#endif
error1:
	return rc;
}

static int hdmi_cec_power(int on)
{
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (on) {
		rc = gpio_request(HDMI_CEC_VAR_GPIO, "HDMI_CEC_VAR");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
					"HDMI_CEC_VAR", HDMI_CEC_VAR_GPIO, rc);
			goto error;
		}
		pr_debug("%s(on): success\n", __func__);
	} else {
		gpio_free(HDMI_CEC_VAR_GPIO);
		pr_debug("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
error:
	return rc;
}

void __init apq8064_init_fb(void)
{
	printk(KERN_INFO "[lcd] apq8064_init_fb\n");

	platform_device_register(&msm_fb_device);
	platform_device_register(&lvds_chimei_panel_device);

#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
	platform_device_register(&wfd_panel_device);
	platform_device_register(&wfd_device);
#endif

	if (machine_is_apq8064_liquid())
		platform_device_register(&mipi_dsi2lvds_bridge_device);
	if (machine_is_apq8064_mtp() || machine_is_JF())
		platform_device_register(&mipi_dsi_toshiba_panel_device);
	if (machine_is_mpq8064_dtv())
		platform_device_register(&lvds_frc_panel_device);

	/* [LCD team] device register for samsung full hd panel  */
	platform_device_register(&mipi_dsi_samsung_full_hd_panel_device);
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("lvds", &lvds_pdata);
	msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
	platform_device_register(&hdmi_msm_device);
	msm_fb_register_device("dtv", &dtv_pdata);
}

/**
 * Set MDP clocks to high frequency to avoid DSI underflow
 * when using high resolution 1200x1920 WUXGA panels
 */
static void set_mdp_clocks_for_wuxga(void)
{
	mdp_ui_vectors[0].ab = 2000000000;
	mdp_ui_vectors[0].ib = 2000000000;
	mdp_vga_vectors[0].ab = 2000000000;
	mdp_vga_vectors[0].ib = 2000000000;
	mdp_720p_vectors[0].ab = 2000000000;
	mdp_720p_vectors[0].ib = 2000000000;
	mdp_1080p_vectors[0].ab = 2000000000;
	mdp_1080p_vectors[0].ib = 2000000000;
	if (apq8064_hdmi_as_primary_selected()) {
		dtv_bus_def_vectors[0].ab = 2000000000;
		dtv_bus_def_vectors[0].ib = 2000000000;
	}
}

void __init apq8064_set_display_params(char *prim_panel, char *ext_panel,
		unsigned char resolution)
{
	/*
	 * For certain MPQ boards, HDMI should be set as primary display
	 * by default, with the flexibility to specify any other panel
	 * as a primary panel through boot parameters.
	 */
	if (machine_is_mpq8064_hrd() || machine_is_mpq8064_cdp()) {
		pr_debug("HDMI is the primary display by default for MPQ\n");
		if (!strnlen(prim_panel, PANEL_NAME_MAX_LEN))
			strlcpy(msm_fb_pdata.prim_panel_name, HDMI_PANEL_NAME,
				PANEL_NAME_MAX_LEN);
	}

	if (strnlen(prim_panel, PANEL_NAME_MAX_LEN)) {
		strlcpy(msm_fb_pdata.prim_panel_name, prim_panel,
				PANEL_NAME_MAX_LEN);
		pr_debug("msm_fb_pdata.prim_panel_name %s\n",
				msm_fb_pdata.prim_panel_name);

		if (!strncmp((char *)msm_fb_pdata.prim_panel_name,
				HDMI_PANEL_NAME, strnlen(HDMI_PANEL_NAME,
					PANEL_NAME_MAX_LEN))) {
			pr_debug("HDMI is the primary display by"
					" boot parameter\n");
			hdmi_is_primary = 1;
			set_mdp_clocks_for_wuxga();
		}
	}
	if (strnlen(ext_panel, PANEL_NAME_MAX_LEN)) {
		strlcpy(msm_fb_pdata.ext_panel_name, ext_panel,
				PANEL_NAME_MAX_LEN);
		pr_debug("msm_fb_pdata.ext_panel_name %s\n",
				msm_fb_pdata.ext_panel_name);

		if (!strncmp((char *)msm_fb_pdata.ext_panel_name,
			MHL_PANEL_NAME, strnlen(MHL_PANEL_NAME,
				PANEL_NAME_MAX_LEN))) {
			pr_debug("MHL is external display by boot parameter\n");
			mhl_display_enabled = 1;
		}
	}

	msm_fb_pdata.ext_resolution = resolution;
	hdmi_msm_data.is_mhl_enabled = mhl_display_enabled;
}
