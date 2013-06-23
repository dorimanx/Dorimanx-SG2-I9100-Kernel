/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#include "msm_fb.h"
#include "msm_fb_panel.h"
#include "mipi_dsi.h"
#include "mipi_samsung_tft.h"

static struct msm_panel_info pinfo;
static struct mipi_panel_data mipi_pd;

enum {
	GAMMA_0CD	=	1,
	GAMMA_5CD	=	2,
	GAMMA_10CD	=	4, // MIN 10 from platform
	GAMMA_15CD	=	7,
	GAMMA_20CD	=	10,
	GAMMA_25CD	=	13,
	GAMMA_30CD	=	16,
	GAMMA_35CD	=	19,
	GAMMA_40CD	=	22,
	GAMMA_45CD	=	25,
	GAMMA_50CD	=	28,
	GAMMA_55CD	=	31,
	GAMMA_60CD	=	34,
	GAMMA_65CD	=	37,
	GAMMA_70CD	=	40,
	GAMMA_75CD	=	43,
	GAMMA_80CD	=	46,
	GAMMA_85CD	=	49,
	GAMMA_90CD	=	52,
	GAMMA_95CD	=	55,
	GAMMA_100CD	=	58,
	GAMMA_105CD	=	61,
	GAMMA_110CD =	64,
	GAMMA_115CD =	67,
	GAMMA_120CD	=	69,
	GAMMA_125CD	=	71,
	GAMMA_130CD =	73,
	GAMMA_135CD =	75,
	GAMMA_140CD	=	77,
	GAMMA_145CD	=	79,
	GAMMA_150CD =	81, // DEF 150 from platform
	GAMMA_155CD =	86,
	GAMMA_160CD	=	91,
	GAMMA_165CD	=	96,
	GAMMA_170CD =	101,
	GAMMA_175CD =	106,
	GAMMA_180CD	=	111,
	GAMMA_185CD	=	116,
	GAMMA_190CD =	121,
	GAMMA_195CD =	126,
	GAMMA_200CD	=	131,
	GAMMA_205CD	=	136,
	GAMMA_210CD =	141,
	GAMMA_215CD =	146,
	GAMMA_220CD	=	151,
	GAMMA_225CD	=	156,
	GAMMA_230CD =	161,
	GAMMA_235CD =	166,
	GAMMA_240CD =	170,
	GAMMA_245CD =	174,
	GAMMA_250CD =	178,
	GAMMA_255CD =	182, // MAX 255 from platform
};


static char renesas_manufacture_cmd_access_off[] = {
	0xB0,
	0x04,
};

static char renesas_nop[] = {
	0x0, 0x0, 0x0, 0x0,
};

static char renesas_manufacture_cmd_access_on[] = {
	0xB0,
	0x03,
};

static char renesas_brightness_setting[] = {
	0x51,
	0xB6, /*0xFF,*/
};

static char renesas_brightness_packet[] = {
	0x51,
	0xB6, /*0xFF, */
};

static char renesas_cabc_control[] = {
	0x55,
	0x00,
};

#if defined(AUTO_BRIGHTNESS_CABC_FUNCTION)
static char renesas_cabc_enable_control[] = {
	0x55,
	0x01,
};

static char renesas_cabc_disable_control[] = {
	0x55,
	0x00,
};
#endif

static char renesas_backlight_control[] = {
	0x53,
	0x2C,
};

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_PREVENT_HSYNC_LEAKAGE)
static char renesas_teon_control[] = {
	0x35,
	0x01,
};

static char renesas_teoff_control[] = {
	0x35,
	0x00,
};
#endif

static char renesas_memory_access_control[] = {
	0x36,
	0xC0,
};

static char renesas_F0_control[] = {
	0xF0,
	0x5A,0x5A,
};

static char renesas_FA_control[] = {
	0xFA,
	0x18,0x3F,0x0A,0x08,0x0D,0x0E,0x0E,0x10,0x11,0x11,
	0x10,0x0C,0x08,0x07,0x0A,0x0E,0x16,0x16,0x15,0x13,
	0x0F,0x02,0x02,0x0F,0x3F,0x0C,0x0D,0x12,0x15,0x16,
	0x18,0x1A,0x19,0x18,0x13,0x0D,0x0A,0x0C,0x10,0x17,
	0x17,0x16,0x13,0x0F,0x03,0x03,0x00,0x3F,0x0F,0x11,
	0x19,0x1D,0x20,0x22,0x24,0x23,0x22,0x1C,0x14,0x10,
	0x12,0x15,0x1B,0x1A,0x18,0x15,0x10,0x05,0x06,
};

static char renesas_FB_control[] = {
	0xFB,
	0x18,0x3F,0x0A,0x08,0x0D,0x0E,0x0E,0x10,0x11,0x11,
	0x10,0x0C,0x08,0x07,0x0A,0x0E,0x16,0x16,0x15,0x13,
	0x0F,0x02,0x02,0x0F,0x3F,0x0C,0x0D,0x12,0x15,0x16,
	0x18,0x1A,0x19,0x18,0x13,0x0D,0x0A,0x0C,0x10,0x17,
	0x17,0x16,0x13,0x0F,0x03,0x03,0x00,0x3F,0x0F,0x11,
	0x19,0x1D,0x20,0x22,0x24,0x23,0x22,0x1C,0x14,0x10,
	0x12,0x15,0x1B,0x1A,0x18,0x15,0x10,0x05,0x06,
};

static char renesas_display_on[] = { 0x29, /* no param */ };
static char renesas_display_off[] = { 0x28, /* no param */ };
static char renesas_sleep_in[] = { 0x10, /* no param */ };
static char renesas_sleep_out[] = { 0x11, /* no param */ };

static struct dsi_cmd_desc renesas_ready_to_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_brightness_setting), renesas_brightness_setting},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_cabc_control), renesas_cabc_control},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_backlight_control), renesas_backlight_control},

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_PREVENT_HSYNC_LEAKAGE)
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_teon_control), renesas_teon_control},
#endif

	{DTYPE_DCS_LWRITE, 1, 0, 0, 120,
		sizeof(renesas_sleep_out), renesas_sleep_out},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(renesas_F0_control), renesas_F0_control},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(renesas_FA_control), renesas_FA_control},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(renesas_FB_control), renesas_FB_control},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_display_on), renesas_display_on},
};

static struct dsi_cmd_desc panel_off_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_display_off), renesas_display_off},
};

static struct dsi_cmd_desc panel_late_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5,
		sizeof(renesas_display_on), renesas_display_on},
};

static struct dsi_cmd_desc panel_early_off_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_sleep_in), renesas_sleep_in},
};

static struct dsi_cmd_desc panel_mtp_enable_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_manufacture_cmd_access_off),
					renesas_manufacture_cmd_access_off},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_nop), renesas_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_nop), renesas_nop},
};

static struct dsi_cmd_desc panel_mtp_disable_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_manufacture_cmd_access_on),
					renesas_manufacture_cmd_access_on},
};

static struct dsi_cmd_desc panel_need_flip_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_memory_access_control),
					renesas_memory_access_control},
};

static struct dsi_cmd_desc brightness_packet[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_brightness_packet), renesas_brightness_packet},
};

#if defined(AUTO_BRIGHTNESS_CABC_FUNCTION)
static struct dsi_cmd_desc panel_cabc_enable_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_cabc_enable_control), renesas_cabc_enable_control},
};

static struct dsi_cmd_desc panel_cabc_disable_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_cabc_disable_control), renesas_cabc_disable_control},
};
#endif

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_PREVENT_HSYNC_LEAKAGE)
static struct dsi_cmd_desc panel_hsync_on_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_teon_control), renesas_teon_control},
};

static struct dsi_cmd_desc panel_hsync_off_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(renesas_teoff_control), renesas_teoff_control},
};
#endif
static int get_candela_index(int bl_level)
{
	int backlightlevel;

	/* brightness setting from platform is from 0 to 255
	 * But in this driver, brightness is only supported from 0 to 24 */

	switch (bl_level) {
	case 0 ... 4:
		backlightlevel = GAMMA_0CD;
		break;
	case 5 ... 9:
		backlightlevel = GAMMA_5CD;
		break;
	case 10 ... 14:
		backlightlevel = GAMMA_10CD;
		break;
	case 15 ... 19:
		backlightlevel = GAMMA_15CD;
		break;
	case 20 ... 24:
		backlightlevel = GAMMA_20CD;
		break;
	case 25 ... 29:
		backlightlevel = GAMMA_25CD;
		break;
	case 30 ... 34:
		backlightlevel = GAMMA_30CD;
		break;
	case 35 ... 39:
		backlightlevel = GAMMA_35CD;
		break;
	case 40 ... 44:
		backlightlevel = GAMMA_40CD;
		break;
	case 45 ... 49:
		backlightlevel = GAMMA_45CD;
		break;
	case 50 ... 54:
		backlightlevel = GAMMA_50CD;
		break;
	case 55 ... 59:
		backlightlevel = GAMMA_55CD;
		break;
	case 60 ... 64:
		backlightlevel = GAMMA_60CD;
		break;
	case 65 ... 69:
		backlightlevel = GAMMA_65CD;
		break;
	case 70 ... 74:
		backlightlevel = GAMMA_70CD;
		break;
	case 75 ... 79:
		backlightlevel = GAMMA_75CD;
		break;
	case 80 ... 84:
		backlightlevel = GAMMA_80CD;
		break;
	case 85 ... 89:
		backlightlevel = GAMMA_85CD;
		break;
	case 90 ... 94:
		backlightlevel = GAMMA_90CD;
		break;
	case 95 ... 99:
		backlightlevel = GAMMA_95CD;
		break;
	case 100 ... 104:
		backlightlevel = GAMMA_100CD;
		break;
	case 105 ... 109:
		backlightlevel = GAMMA_105CD;
		break;
	case 110 ... 114:
		backlightlevel = GAMMA_110CD;
		break;
	case 115 ... 119:
		backlightlevel = GAMMA_115CD;
		break;
	case 120 ... 124:
		backlightlevel = GAMMA_120CD;
		break;
	case 125 ... 129:
		backlightlevel = GAMMA_125CD;
		break;
	case 130 ... 134:
		backlightlevel = GAMMA_130CD;
		break;
	case 135 ... 139:
		backlightlevel = GAMMA_135CD;
		break;
	case 140 ... 144:
		backlightlevel = GAMMA_140CD;
		break;
	case 145 ... 149:
		backlightlevel = GAMMA_145CD;
		break;
	case 150 ... 154:
		backlightlevel = GAMMA_150CD;
		break;
	case 155 ... 159:
		backlightlevel = GAMMA_155CD;
		break;
	case 160 ... 164:
		backlightlevel = GAMMA_160CD;
		break;
	case 165 ... 169:
		backlightlevel = GAMMA_165CD;
		break;
	case 170 ... 174:
		backlightlevel = GAMMA_170CD;
		break;
	case 175 ... 179:
		backlightlevel = GAMMA_175CD;
		break;
	case 180 ... 184:
		backlightlevel = GAMMA_180CD;
		break;
	case 185 ... 189:
		backlightlevel = GAMMA_185CD;
		break;
	case 190 ... 194:
		backlightlevel = GAMMA_190CD;
		break;
	case 195 ... 199:
		backlightlevel = GAMMA_195CD;
		break;
	case 200 ... 204:
		backlightlevel = GAMMA_200CD;
		break;
	case 205 ... 209:
		backlightlevel = GAMMA_205CD;
		break;
	case 210 ... 214:
		backlightlevel = GAMMA_210CD;
		break;
	case 215 ... 219:
		backlightlevel = GAMMA_215CD;
		break;
	case 220 ... 224:
		backlightlevel = GAMMA_220CD;
		break;
	case 225 ... 229:
		backlightlevel = GAMMA_225CD;
		break;
	case 230 ... 234:
		backlightlevel = GAMMA_230CD;
		break;
	case 235 ... 239:
		backlightlevel = GAMMA_235CD;
		break;
	case 240 ... 244:
		backlightlevel = GAMMA_240CD;
		break;
	case 245 ... 249:
		backlightlevel = GAMMA_245CD;
		break;
	case 250 ... 254:
		backlightlevel = GAMMA_250CD;
		break;
	case 255:
		backlightlevel = GAMMA_255CD;
		break;
	default:
		backlightlevel = GAMMA_150CD;
		break;
	}
	return backlightlevel;
}

static int brightness_control(int bl_level)
{
	int level = get_candela_index(bl_level);
	int compensation = 500;

	/* 0x0 ~ 0x0FFF,  0 ~ 4095*/
	int value = 178 * level + compensation;

	if (value > 0xFFF)
		value = 0xFFF;

	if (system_rev == 0) {
		renesas_brightness_packet[1] = value & 0xFF;
		renesas_brightness_packet[1] = (value >> 8) & 0xFF;
	} else
	{
		renesas_brightness_packet[1] =  level & 0xFF;
		renesas_brightness_setting[1] = level & 0xFF;
	}

	return 0;
}

static struct mipi_panel_data mipi_pd = {
	.panel_name	= "JDI_ACX454AKM\n",
	.on		= {renesas_ready_to_on_cmds
				, ARRAY_SIZE(renesas_ready_to_on_cmds)},
	.off		= {panel_off_cmds
				, ARRAY_SIZE(panel_off_cmds)},
	.late_on		= {panel_late_on_cmds
				, ARRAY_SIZE(panel_late_on_cmds)},
	.early_off	= {panel_early_off_cmds
				, ARRAY_SIZE(panel_early_off_cmds)},

	.brightness	= {brightness_packet
				, ARRAY_SIZE(brightness_packet)},
	.backlight_control = brightness_control,

	.mtp_enable	= {panel_mtp_enable_cmds
				, ARRAY_SIZE(panel_mtp_enable_cmds)},
	.mtp_disable	= {panel_mtp_disable_cmds
				, ARRAY_SIZE(panel_mtp_disable_cmds)},
	.need_flip	= {panel_need_flip_cmds
				, ARRAY_SIZE(panel_mtp_disable_cmds)},
#if defined(AUTO_BRIGHTNESS_CABC_FUNCTION)
	.cabc_enable = {panel_cabc_enable_cmds
				, ARRAY_SIZE(panel_cabc_enable_cmds)},
	.cabc_disable	= {panel_cabc_disable_cmds
				, ARRAY_SIZE(panel_cabc_disable_cmds)},
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_PREVENT_HSYNC_LEAKAGE)
	.hsync_on = {panel_hsync_on_cmds
				, ARRAY_SIZE(panel_hsync_on_cmds)},
	.hsync_off	= {panel_hsync_off_cmds
				, ARRAY_SIZE(panel_hsync_off_cmds)},
#endif
};

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	/* 1920*1200, RGB888, 4 Lane 60 fps video mode */
	/* regulator */
	.regulator = {0x03, 0x0a, 0x04, 0x00, 0x20},
	/* timing */
	.timing = {0xD9, 0x40, 0x3C, 0x00, 0x52, 0x5E, 0x32, 0x40,
	0x3C, 0x03, 0x04, 0xa0},
	/* phy ctrl */
	.ctrl = {0x5f, 0x00, 0x00, 0x10},
	/* strength */
	.strength = {0xff, 0x00, 0x06, 0x00},
	/* pll control */
	.pll = {0x0, 0x7f, 0x31, 0xda, 0x00, 0x50, 0x48, 0x63,
	0x41, 0x0f, 0x03,
	0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
};

static int __init mipi_cmd_samsung_tft_full_hd_pt_init(void)
{
	int ret;

	printk(KERN_DEBUG "[lcd] mipi_cmd_samsung_tft_full_hd_pt_init start\n");

#ifdef CONFIG_SUPPORT_DISPLAY_OCTA_TFT
#if 0
	if (check_display_type() != SAMSUNG_TFT_FULL_HD) {
		pr_err("[lcd] error : this rev(%d) is not for TFT, return!!\n",
			system_rev);
		return 1;
	}
#endif
#endif

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_cmd_samsung_oled_qhd"))
		return 0;
#endif
	pinfo.xres = 1080;
	pinfo.yres = 1920;
	pinfo.height = 111;
	pinfo.width = 62;

	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;

	pinfo.lcdc.h_back_porch = 100;
	pinfo.lcdc.h_front_porch = 97;
	pinfo.lcdc.h_pulse_width = 14;

	pinfo.lcdc.v_back_porch = 6;
	pinfo.lcdc.v_front_porch = 8;
	pinfo.lcdc.v_pulse_width = 2;

	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0x000000;	/* blue */
	pinfo.lcdc.hsync_skew = 0;

	pinfo.bl_max = 255;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

	pinfo.clk_rate = 898000000;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = FALSE;
	pinfo.mipi.hfp_power_stop = TRUE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = FALSE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_BURST_MODE;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;
	pinfo.mipi.t_clk_post = 0x04;
	pinfo.mipi.t_clk_pre = 0x1c;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.force_clk_lane_hs = 1;
	pinfo.mipi.esc_byte_ratio = 1;

	ret = mipi_samsung_tft_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_FULL_HD_PT,
						&mipi_pd);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	printk(KERN_DEBUG "[lcd] mipi_cmd_samsung_tft_full_hd_pt_init end\n");

	return ret;
}
module_init(mipi_cmd_samsung_tft_full_hd_pt_init);
