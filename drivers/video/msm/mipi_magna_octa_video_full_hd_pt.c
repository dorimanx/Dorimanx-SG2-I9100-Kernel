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
#include "mipi_samsung_octa.h"

#define MAGNA_MANUAL_OTP_LOADING

static struct msm_panel_info pinfo;
static struct mipi_panel_data mipi_pd;

static int lux_tbl[] = {
	10, 11, 12, 13, 14,
	15, 16, 17, 19, 20,
	21, 22, 24, 25, 27,
	29, 30, 32, 34, 37,
	39, 41, 44, 47, 50,
	53, 56, 60, 64, 68,
	72, 77, 82, 87, 93,
	98, 105, 111, 119, 126, 
	134, 143, 152, 162, 172,
	183, 195, 207, 220, 234,
	249, 265, 282, 300,
};

static char magna_nop[] = {
	0x0, 0x0, 0x0, 0x0,
};

static char magna_test_key_on1[] = {
	0xF0,
	0x5A, 0x5A,
};

static char magna_test_key_on2[] = {
	0xF1,
	0x5A, 0x5A,
};

static char magna_test_key_on3[] = {
	0xFC,
	0x5A, 0x5A,
};

static char magna_test_key_off1[] = {
	0xF0,
	0xA5, 0xA5,
};

static char magna_test_key_off2[] = {
	0xF1,
	0xA5, 0xA5,
};

static char magna_test_key_off3[] = {
	0xFC,
	0xA5, 0xA5,
};

/* To use manual OTP loading because of MTP reading operation*/
static char magna_manual_bootsing[] = {
	0xF3,
	0x00, 0x01,
};

static char mgana_aid_control[] = {
	0xB2,
	0x00, 0x00, 0x00, 0x08,
};

static char magna_source_control[] = {
	0xB9,
	0x31, 0x15,
};

static char magna_pentile[] = {
	0xC1,
	0x00, 0x2B,
};

static char magna_power_control[] = {
	0xF4,
	0x20, 0x10,
};

static char magna_ss_control[] = {
	0x36,
	0x02,
};

static char magna_TSP_control[] = {
	0xFF,
	0x00, 0x26,
};

static char magna_ACL_control[] = {
	0xB4,
	0x00,
};

static char magna_brightness_write_als[] = {
	0xE3,
	0xff, 0xff, 0xff, 0xff,
};

static char magna_psre_mtp_pre[] = {
	0x00,
	0x00,
};

static char magna_psre_mtp_ref[] = {
	0xBB,
	0x00,
};

static char magna_brightness_psre_cont[] = {
	0xBC,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static char hbm_mode_cont_pre[] = {
	0x00,
	0x00,
};

static char hbm_mode_cont_ref[] = {
	0x53,
	0x00,
};

static char magna_temperature_compensation[] = {
	0xB8,
	0x00, 0x5A, 0x74, 0x03, 0x00,
	0x00, 0x19,
};

static char magna_ltps_dynaimc_control[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x99,
	0x00, 0x00, 0x0B, 0x78, 0x00,
	0x90, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x99, 0x0A, 0x5E,
	0x13, 0x13, 0x71, 0x71, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0xAE, 0xCB, 0xED, 0x0B, 0x78,
};

static char magna_ltps_60[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x99,
	0x00, 0x00, 0x0B, 0x78, 0x00,
	0x90, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x99, 0x0A, 0x5E,
	0x13, 0x13, 0x71, 0x71, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x0B, 0x78 
};

static char magna_ltps_59[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x96,
	0x00, 0x00, 0x0B, 0x76, 0x00,
	0x8E, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x96, 0x09, 0x5C,
	0x13, 0x13, 0x6F, 0x6F, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x0B, 0x76 
};

static char magna_ltps_58[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x94,
	0x00, 0x00, 0x0B, 0x74, 0x00,
	0x8B, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x94, 0x09, 0x5B,
	0x12, 0x12, 0x6D, 0x6D, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x0B, 0x74 
};

static char magna_ltps_57[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x91,
	0x00, 0x00, 0x0B, 0x72, 0x00,
	0x89, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x91, 0x09, 0x59,
	0x12, 0x12, 0x6B, 0x6B, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x0B, 0x72 
};

static char magna_ltps_56[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x8F,
	0x00, 0x00, 0x0B, 0x70, 0x00,
	0x86, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x8F, 0x09, 0x58,
	0x12, 0x12, 0x69, 0x69, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x0B, 0x70 
};

static char magna_ltps_55[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x8C,
	0x00, 0x00, 0x0B, 0x6E, 0x00,
	0x84, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x8C, 0x09, 0x56,
	0x11, 0x11, 0x67, 0x67, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x0B, 0x6E 
};

static char magna_ltps_54[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x8A,
	0x00, 0x00, 0x0A, 0x6C, 0x00,
	0x82, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x8A, 0x09, 0x54,
	0x11, 0x11, 0x65, 0x65, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x0A, 0x6C
};

static char magna_ltps_53[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x87,
	0x00, 0x00, 0x0A, 0x6A, 0x00,
	0x7F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x87, 0x09, 0x53,
	0x11, 0x11, 0x63, 0x63, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x0A, 0x6A 
};

static char magna_ltps_52[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x85,
	0x00, 0x00, 0x0A, 0x68, 0x00,
	0x7D, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x85, 0x08, 0x51,
	0x10, 0x10, 0x62, 0x62, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x0A, 0x68 
};

static char magna_ltps_51[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x82,
	0x00, 0x00, 0x0A, 0x66, 0x00,
	0x7A, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x82, 0x08, 0x50,
	0x10, 0x10, 0x60, 0x60, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x0A, 0x66 
};

static char magna_ltps_50[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x7F,
	0x00, 0x00, 0x0A, 0x64, 0x00,
	0x78, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x7F, 0x08, 0x4E,
	0x10, 0x10, 0x5E, 0x5E, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x0A, 0x64 
};

static char magna_ltps_49[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x7D,
	0x00, 0x00, 0x09, 0x62, 0x00,
	0x76, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x7D, 0x08, 0x4D,
	0x10, 0x10, 0x5C, 0x5C, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x09, 0x62 
};

static char magna_ltps_48[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x7A,
	0x00, 0x00, 0x09, 0x60, 0x00,
	0x73, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x7A, 0x08, 0x4B,
	0x0F, 0x0F, 0x5A, 0x5A, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x09, 0x60 
};

static char magna_ltps_47[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x78,
	0x00, 0x00, 0x09, 0x5E, 0x00,
	0x71, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x78, 0x08, 0x49,
	0x0F, 0x0F, 0x58, 0x58, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x09, 0x5E 
};

static char magna_ltps_46[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x75,
	0x00, 0x00, 0x09, 0x5C, 0x00,
	0x6E, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x75, 0x07, 0x48,
	0x0F, 0x0F, 0x56, 0x56, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x09, 0x5C 
};

static char magna_ltps_45[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x73,
	0x00, 0x00, 0x09, 0x5A, 0x00,
	0x6C, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x73, 0x07, 0x46,
	0x0E, 0x0E, 0x55, 0x55, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x09, 0x5A 
};

static char magna_ltps_44[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x70,
	0x00, 0x00, 0x08, 0x58, 0x00,
	0x6A, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x70, 0x07, 0x45,
	0x0E, 0x0E, 0x53, 0x53, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x08, 0x59 
};

static char magna_ltps_43[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x6E,
	0x00, 0x00, 0x08, 0x56, 0x00,
	0x67, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x6E, 0x07, 0x43,
	0x0E, 0x0E, 0x51, 0x51, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x08, 0x56 
};

static char magna_ltps_42[] = {
	0xCB,
	0xA0, 0x13, 0xA0, 0x08, 0x01,
	0xAB, 0x29, 0x18, 0x04, 0xC4,
	0xFF, 0xFC, 0x10, 0x00, 0x6B,
	0x00, 0x00, 0x08, 0x54, 0x00,
	0x65, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x6B, 0x07, 0x42,
	0x0D, 0x0D, 0x4F, 0x4F, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x56, 0x31, 0x32, 0x13,
	0x7E, 0xCB, 0xED, 0x08, 0x54 
};

static char magna_brightness_gamma[] = {
	0xCA,
	0x01, 0x00, 0x01, 0x00,
	0x01, 0x00, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x00, 0x00,
	0x00,
};

static char magna_brightness_gamma_hbm[] = {
	0xCA,
	0x01, 0x00, 0x01, 0x00,
	0x01, 0x00,
};

static char magna_gamma_update[] = {
	0xF7,
	0x01,
};

static char magna_brightness_aor_condition[] = {
	0xB2,
	0x00, 0x00, 0x00, 0x06
};

static char magna_elvss_condition[] = {
	0xB6,
	0x2F,
};

static char magna_acl_condition[] = {
	0x55,
	0x00,
};



/*To make packed packet */
static char magna_brightness_aor_default[] = {
	0xB2,
	0x00, 0x00, 0x00, 0x06
};

static char magna_brightness_aor_ref[] = {
	0xB2,
	0x00, 0x00, 0x00, 0x06
};

static char magna_brightness_aor_pre[] = {
	0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00,
};


static char magna_brightness_elvss_default[] = {
	0xB6,
	0x2B,
};

static char magna_brightness_elvss_ref[] = {
	0xB6,
	0x2B,
};

static char magna_brightness_elvss_pre[] = {
	0x00,
	0x00,
};


static char magna_brightness_acl_default[] = {
	0x55,
	0x00,
};

static char magna_brightness_acl_ref[] = {
	0x55,
	0x00,
};

static char magna_brightness_acl_pre[] = {
	0x00,
	0x00,
};

static char magna_display_on[] = { 0x29, /* no param */ };
static char magna_display_off[] = { 0x28, /* no param */ };
static char magna_sleep_in[] = { 0x10, /* no param */ };
static char magna_sleep_out[] = { 0x11, /* no param */ };
static char magna_soft_reset[] = { 0x01, /* no param */ };

static struct dsi_cmd_desc magna_on_cmds_revB[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on1), magna_test_key_on1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on3), magna_test_key_on3},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_source_control), magna_source_control},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_pentile), magna_pentile},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_power_control), magna_power_control},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_ss_control), magna_ss_control},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_TSP_control), magna_TSP_control},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_ACL_control), magna_ACL_control},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_brightness_gamma), magna_brightness_gamma},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(mgana_aid_control), mgana_aid_control},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_acl_condition), magna_acl_condition},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_gamma_update), magna_gamma_update},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_off1), magna_test_key_off1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_off2), magna_test_key_off2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_off3), magna_test_key_off3},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 120,
		sizeof(magna_sleep_out), magna_sleep_out},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_display_on), magna_display_on},
};

static struct dsi_cmd_desc magna_on_cmds_revF[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on1), magna_test_key_on1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on3), magna_test_key_on3},

	/* initial condition set */	
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(mgana_aid_control), mgana_aid_control},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_source_control), magna_source_control},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_pentile), magna_pentile},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_power_control), magna_power_control},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_ss_control), magna_ss_control},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_ACL_control), magna_ACL_control},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_temperature_compensation), magna_temperature_compensation},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 120,
		sizeof(magna_sleep_out), magna_sleep_out},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on1), magna_test_key_on1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on2), magna_test_key_on2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on3), magna_test_key_on3},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_brightness_gamma), magna_brightness_gamma},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(mgana_aid_control), mgana_aid_control},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_acl_condition), magna_acl_condition},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_gamma_update), magna_gamma_update},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_TSP_control), magna_TSP_control},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_off1), magna_test_key_off1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_off2), magna_test_key_off2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 20,
		sizeof(magna_test_key_off3), magna_test_key_off3},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_display_on), magna_display_on},
};

static struct dsi_cmd_desc panel_off_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 40,
		sizeof(magna_display_off), magna_display_off},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 120,
		sizeof(magna_sleep_in), magna_sleep_in},
};

static struct dsi_cmd_desc panel_late_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5,
		sizeof(magna_display_on), magna_display_on},
};

static struct dsi_cmd_desc panel_early_off_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_sleep_in), magna_sleep_in},
};

/* MAGNA LDI loads OTP data after sleep_out cmd.
That's why panel_mtp_enable_cmds structure has lots of cmds.
*/
static struct dsi_cmd_desc panel_mtp_enable_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on1), magna_test_key_on1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on3), magna_test_key_on3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_manual_bootsing), magna_manual_bootsing},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 20,
		sizeof(magna_sleep_out), magna_sleep_out},
		
#if !defined(MAGNA_MANUAL_OTP_LOADING)
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on1), magna_test_key_on1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on2), magna_test_key_on2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_on3), magna_test_key_on3},
#endif
};
/* MAGNA LDI loads OTP data after sleep_out cmd and need soft_reset cmd after reading register.
That's why panel_mtp_disable_cmds structure has lots of cmds.
*/
static struct dsi_cmd_desc panel_mtp_disable_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 20,
		sizeof(magna_soft_reset), magna_soft_reset},

#if !defined(MAGNA_MANUAL_OTP_LOADING)
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_off1), magna_test_key_off1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_off2), magna_test_key_off2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_test_key_off3), magna_test_key_off3},
#endif
};

static struct dsi_cmd_desc panel_need_flip_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_nop), magna_nop},
};

static struct dsi_cmd_desc panel_lpts[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(magna_ltps_dynaimc_control), magna_ltps_dynaimc_control},
};

static struct dsi_cmd_desc brightness_packet[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(magna_nop), magna_nop},
};

static struct dsi_cmd_desc panel_acl_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
	sizeof(magna_brightness_acl_ref), magna_brightness_acl_ref},
};

static char magna_brightness_aor_0[] = {
	0xB2,
	0x00, 0x00, 0x00, 0x08
};

static char magna_brightness_aor_40[] = {
	0xB2,
	0x00, 0x00, 0x03, 0x06
};

static char magna_brightness_aor_1p7[] = {
	0xB2,
	0x00, 0x00, 0x00, 0x20
};

static char magna_brightness_aor_8p5[] = {
	0xB2,
	0x00, 0x00, 0x00, 0xA5
};

static char magna_brightness_aor_13p4[] = {
	0xB2,
	0x00, 0x00, 0x01, 0x03
};

static char magna_brightness_aor_19p1[] = {
	0xB2,
	0x00, 0x00, 0x01, 0x72
};

static char magna_brightness_aor_23p8[] = {
	0xB2,
	0x00, 0x00, 0x01, 0xCC
};

static char magna_brightness_aor_28p5[] = {
	0xB2,
	0x00, 0x00, 0x02, 0x27
};

static char magna_brightness_aor_33p2[] = {
	0xB2,
	0x00, 0x00, 0x02, 0x83
};

static char magna_brightness_aor_37p1[] = {
	0xB2,
	0x00, 0x00, 0x02, 0xCE
};

static char magna_brightness_aor_41p1[] = {
	0xB2,
	0x00, 0x00, 0x03, 0x1B
};

static char magna_brightness_aor_45p1[] = {
	0xB2,
	0x00, 0x00, 0x03, 0x69
};

static char magna_brightness_aor_48p7[] = {
	0xB2,
	0x00, 0x00, 0x03, 0xAE
};

static char magna_brightness_aor_51p4[] = {
	0xB2,
	0x00, 0x00, 0x03, 0xE2
};

static char magna_brightness_aor_54p1[] = {
	0xB2,
	0x00, 0x00, 0x04, 0x16
};

static char magna_brightness_aor_57p0[] = {
	0xB2,
	0x00, 0x00, 0x04, 0x4E
};

static char magna_brightness_aor_59p9[] = {
	0xB2,
	0x00, 0x00, 0x04, 0x86
};

static char magna_brightness_aor_62p8[] = {
	0xB2,
	0x00, 0x00, 0x04, 0xBF
};

static char magna_brightness_aor_64p7[] = {
	0xB2,
	0x00, 0x00, 0x04, 0xE3
};

static char magna_brightness_aor_66p5[] = {
	0xB2,
	0x00, 0x00, 0x05, 0x06
};

static char magna_brightness_aor_69p2[] = {
	0xB2,
	0x00, 0x00, 0x05, 0x3B
};

static char magna_brightness_aor_71p0[] = {
	0xB2,
	0x00, 0x00, 0x05, 0x5E
};

static char magna_brightness_aor_72p9[] = {
	0xB2,
	0x00, 0x00, 0x05, 0x82
};
static char magna_brightness_aor_73p7[] = {
	0xB2,
	0x00, 0x00, 0x05, 0x92
};
static char magna_brightness_aor_75p4[] = {
	0xB2,
	0x00, 0x00, 0x05, 0xB3
};
static char magna_brightness_aor_77p1[] = {
	0xB2,
	0x00, 0x00, 0x05, 0xD4
};
static char magna_brightness_aor_78p0[] = {
	0xB2,
	0x00, 0x00, 0x05, 0xE5
};
static char magna_brightness_aor_79p7[] = {
	0xB2,
	0x00, 0x00, 0x06, 0x06
};
static char magna_brightness_aor_80p6[] = {
	0xB2,
	0x00, 0x00, 0x06, 0x16
};
static char magna_brightness_aor_81p4[] = {
	0xB2,
	0x00, 0x00, 0x06, 0x27
};
static char magna_brightness_aor_82p3[] = {
	0xB2,
	0x00, 0x00, 0x06, 0x38
};
static char magna_brightness_aor_84p1[] = {
	0xB2,
	0x00, 0x00, 0x06, 0x5B
};
static char magna_brightness_aor_85p0[] = {
	0xB2,
	0x00, 0x00, 0x06, 0x6C
};
static char magna_brightness_aor_85p9[] = {
	0xB2,
	0x00, 0x00, 0x06, 0x7D
};
static char magna_brightness_aor_86p8[] = {
	0xB2,
	0x00, 0x00, 0x06, 0x8F
};
static char magna_brightness_aor_87p7[] = {
	0xB2,
	0x00, 0x00, 0x06, 0xA0
};
static char magna_brightness_aor_88p6[] = {
	0xB2,
	0x00, 0x00, 0x06, 0xB2
};
static char magna_brightness_aor_89p5[] = {
	0xB2,
	0x00, 0x00, 0x06, 0xC3
};
static char magna_brightness_aor_90p4[] = {
	0xB2,
	0x00, 0x00, 0x06, 0xD5
};

enum {
	GAMMA_10CD,
	GAMMA_11CD,
	GAMMA_12CD,
	GAMMA_13CD,
	GAMMA_14CD,
	GAMMA_15CD,
	GAMMA_16CD,
	GAMMA_17CD,
	GAMMA_19CD,
	GAMMA_20CD,
	GAMMA_21CD,
	GAMMA_22CD,
	GAMMA_24CD,
	GAMMA_25CD,
	GAMMA_27CD,
	GAMMA_29CD,
	GAMMA_30CD,
	GAMMA_32CD,
	GAMMA_34CD,
	GAMMA_37CD,
	GAMMA_39CD,
	GAMMA_41CD,
	GAMMA_44CD,
	GAMMA_47CD,
	GAMMA_50CD,
	GAMMA_53CD,
	GAMMA_56CD,
	GAMMA_60CD,
	GAMMA_64CD,
	GAMMA_68CD,
	GAMMA_72CD,
	GAMMA_77CD,
	GAMMA_82CD,
	GAMMA_87CD,
	GAMMA_93CD,
	GAMMA_98CD,
	GAMMA_105CD,
	GAMMA_111CD,
	GAMMA_119CD,
	GAMMA_126CD,
	GAMMA_134CD,
	GAMMA_143CD,
	GAMMA_152CD,
	GAMMA_162CD,
	GAMMA_172CD,
	GAMMA_183CD,
	GAMMA_195CD,
	GAMMA_207CD,
	GAMMA_220CD,
	GAMMA_234CD,
	GAMMA_249CD,
	GAMMA_265CD,
	GAMMA_282CD,
	GAMMA_300CD,
};

static int get_candela_index(int bl_level)
{
	int backlightlevel;

	/* brightness setting from platform is from 0 to 255
	 * But in this driver, brightness is only supported from 0 to 24 */

	backlightlevel = 0;

	switch (bl_level) {
	case 0 ... 10:
		backlightlevel = GAMMA_10CD;
		break;
	case 11:
		backlightlevel = GAMMA_11CD;
		break;
	case 12:
		backlightlevel = GAMMA_12CD;
		break;
	case 13:
		backlightlevel = GAMMA_13CD;
		break;
	case 14:
		backlightlevel = GAMMA_14CD;
		break;
	case 15:
		backlightlevel = GAMMA_15CD;
		break;
	case 16:
		backlightlevel = GAMMA_16CD;
		break;
	case 17 ... 18:
		backlightlevel = GAMMA_17CD;
		break;
	case 19:
		backlightlevel = GAMMA_19CD;
		break;
	case 20:
		backlightlevel = GAMMA_20CD;
		break;
	case 21:
		backlightlevel = GAMMA_21CD;
		break;
	case 22 ... 23:
		backlightlevel = GAMMA_22CD;
		break;
	case 24:
		backlightlevel = GAMMA_24CD;
		break;
	case 25 ... 26:
		backlightlevel = GAMMA_25CD;
		break;
	case 27 ... 28:
		backlightlevel = GAMMA_27CD;
		break;
	case 29:
		backlightlevel = GAMMA_29CD;
		break;
	case 30 ... 31:
		backlightlevel = GAMMA_30CD;
		break;
	case 32:
		backlightlevel = GAMMA_32CD;
		break;
	case 33 ... 36:
		backlightlevel = GAMMA_34CD;
		break;
	case 37 ... 38:
		backlightlevel = GAMMA_37CD;
		break;
	case 39 ... 40:
		backlightlevel = GAMMA_39CD;
		break;
	case 41 ... 43:
		backlightlevel = GAMMA_41CD;
		break;
	case 44 ... 46:
		backlightlevel = GAMMA_44CD;
		break;
	case 47 ... 49:
		backlightlevel = GAMMA_47CD;
		break;
	case 50 ... 52:
		backlightlevel = GAMMA_50CD;
		break;
	case 53 ... 55:
		backlightlevel = GAMMA_53CD;
		break;
	case 56 ... 59:
		backlightlevel = GAMMA_56CD;
		break;
	case 60 ... 63:
		backlightlevel = GAMMA_60CD;
		break;
	case 64 ... 67:
		backlightlevel = GAMMA_64CD;
		break;
	case 68 ... 71:
		backlightlevel = GAMMA_68CD;
		break;
	case 72 ... 76:
		backlightlevel = GAMMA_72CD;
		break;
	case 77 ... 81:
		backlightlevel = GAMMA_77CD;
		break;
	case 82 ... 86:
		backlightlevel = GAMMA_82CD;
		break;
	case 87 ... 92:
		backlightlevel = GAMMA_87CD;
		break;
	case 93 ... 97:
		backlightlevel = GAMMA_93CD;
		break;
	case 98 ... 104:
		backlightlevel = GAMMA_98CD;
		break;
	case 105 ... 110:
		backlightlevel = GAMMA_105CD;
		break;
	case 111 ... 118:
		backlightlevel = GAMMA_111CD;
		break;
	case 119 ... 125:
		backlightlevel = GAMMA_119CD;
		break;
	case 126 ... 133:
		backlightlevel = GAMMA_126CD;
		break;
	case 134 ... 142:
		backlightlevel = GAMMA_134CD;
		break;
	case 143 ... 149:
		backlightlevel = GAMMA_143CD;
		break;
	case 150 ... 161:
		backlightlevel = GAMMA_152CD;
		break;
	case 162 ... 171:
		backlightlevel = GAMMA_162CD;
		break;
	case 172 ... 182:
		backlightlevel = GAMMA_172CD;
		break;
	case 183 ... 194:
		backlightlevel = GAMMA_183CD;
		break;
	case 195 ... 206:
		backlightlevel = GAMMA_195CD;
		break;
	case 207 ... 219:
		backlightlevel = GAMMA_207CD;
		break;
	case 220 ... 232:
		backlightlevel = GAMMA_220CD;
		break;
	case 233 ... 248:
		backlightlevel = GAMMA_234CD;
		break;
	case 249:
		backlightlevel = GAMMA_249CD;
		break;
	case 250 ... 251:
		backlightlevel = GAMMA_265CD;
		break;
	case 252 ... 253:
		backlightlevel = GAMMA_282CD;
		break;
	case 254 ... 255:
		backlightlevel = GAMMA_300CD;
		break;
	default:
		pr_info("%s lcd error bl_level : %d", __func__, bl_level);
		backlightlevel = GAMMA_152CD;
		break;
	}

	return backlightlevel;
}

#define DEFAULT_ELVSS 0x2B
#define ELVSS_LSL 2
static int get_elvss_value(int candela, int id3)
{
	int elvss_value;

	elvss_value = 0;

	switch (candela) {
	case 0 ... 105:
		elvss_value = DEFAULT_ELVSS + (0x0F << ELVSS_LSL);
		break;
	case 106 ... 111:
		elvss_value = DEFAULT_ELVSS + (0x0B << ELVSS_LSL);
		break;
	case 112 ... 119:
		elvss_value = DEFAULT_ELVSS + (0x0A << ELVSS_LSL);
		break;
	case 120 ... 126:
		elvss_value = DEFAULT_ELVSS + (0x09 << ELVSS_LSL);
		break;
	case 127 ... 134:
		elvss_value = DEFAULT_ELVSS + (0x08 << ELVSS_LSL);
		break;
	case 135 ... 143:
		elvss_value = DEFAULT_ELVSS + (0x07 << ELVSS_LSL);
		break;
	case 144 ... 152:
		elvss_value = DEFAULT_ELVSS + (0x06 << ELVSS_LSL);
		break;
	case 153 ... 162:
		elvss_value = DEFAULT_ELVSS + (0x05 << ELVSS_LSL);
		break;
	case 163 ... 172:
		elvss_value = DEFAULT_ELVSS + (0x05 << ELVSS_LSL);
		break;
	case 173 ... 183:
		elvss_value = DEFAULT_ELVSS + (0x09 << ELVSS_LSL);
		break;
	case 184 ... 195:
		elvss_value = DEFAULT_ELVSS + (0x08 << ELVSS_LSL);
		break;
	case 196 ... 207:
		elvss_value = DEFAULT_ELVSS + (0x07 << ELVSS_LSL);
		break;
	case 208 ... 220:
		elvss_value = DEFAULT_ELVSS + (0x06 << ELVSS_LSL);
		break;
	case 221 ... 234:
		elvss_value = DEFAULT_ELVSS + (0x05 << ELVSS_LSL);
		break;
	case 235 ... 249:
		elvss_value = DEFAULT_ELVSS + (0x04 << ELVSS_LSL);
		break;
	case 250 ... 265:
		elvss_value = DEFAULT_ELVSS + (0x03 << ELVSS_LSL);
		break;
	case 266 ... 282:
		elvss_value = DEFAULT_ELVSS + (0x01 << ELVSS_LSL);
		break;
	case 283 ... 300:
		elvss_value = DEFAULT_ELVSS + 0x0;
		break;
	default :
		pr_info("%s lcd error bl_level : %d", __func__, candela);
		elvss_value = DEFAULT_ELVSS + 0x0;
		break;
	}

	return elvss_value;
}

static void aor_copy(int candela)
{
	if (candela >= 183) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_0,
					sizeof(magna_brightness_aor_ref));
	} else if (candela >= 111) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_40,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 105) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_1p7,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 98) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_8p5,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 93) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_13p4,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 87) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_19p1,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 82) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_23p8,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 77) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_28p5,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 72) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_33p2,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 68) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_37p1,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 64) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_41p1,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 60) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_45p1,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 56) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_48p7,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 53) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_51p4,
					sizeof(magna_brightness_aor_ref));	
	} else if (candela == 50) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_54p1,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 47) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_57p0,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 44) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_59p9,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 41) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_62p8,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 39) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_64p7,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 37) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_66p5,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 34) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_69p2,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 32) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_71p0,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 30) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_72p9,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 29) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_73p7,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 27) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_75p4,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 25) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_77p1,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 24) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_78p0,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 22) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_79p7,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 21) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_80p6,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 20) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_81p4,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 19) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_82p3,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 17) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_84p1,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 16) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_85p0,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 15) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_85p9,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 14) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_86p8,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 13) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_87p7,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 12) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_88p6,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 11) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_89p5,
					sizeof(magna_brightness_aor_ref));
	} else if (candela == 10) {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_90p4,
					sizeof(magna_brightness_aor_ref));
	} else {
		memcpy(magna_brightness_aor_ref, magna_brightness_aor_ref,
					sizeof(magna_brightness_aor_ref));
	}
}

static int brightness_control(int bl_level)
{
	int id1, id2, id3;
	int cmd_size;
	char elvss_value;
	int candela;

	int i;
	char* b1_reg;
	char* b6_reg_magna;
	b1_reg = get_b1_reg();
	b6_reg_magna = get_b6_reg_magna();

	id1 = (mipi_pd.manufacture_id & 0x00FF0000) >> 16;
	id2 = (mipi_pd.manufacture_id & 0x0000FF00) >> 8;
	id3 = mipi_pd.manufacture_id & 0xFF;

	if (bl_level < 10)
		bl_level = 10;

	candela = lux_tbl[get_candela_index(bl_level)];

	pr_info("%s brightness_level : %d, candela : %d, auto : %d\n", 
					__func__, mipi_pd.brightness_level, candela, get_auto_brightness());
	cmd_size = 0;

	/* LEVEL KEY ON */
	brightness_packet[cmd_size].payload = magna_test_key_on1;
	brightness_packet[cmd_size].dlen = sizeof(magna_test_key_on1);
	cmd_size++;

	brightness_packet[cmd_size].payload = magna_test_key_on2;
	brightness_packet[cmd_size].dlen = sizeof(magna_test_key_on2);
	cmd_size++;

	/* aor ****************************************************************************/
	/* 0xB2 setting */
	memcpy(magna_brightness_aor_pre, magna_brightness_aor_ref,
					sizeof(magna_brightness_aor_ref));

	aor_copy(candela);

	if (memcmp(magna_brightness_aor_pre, magna_brightness_aor_ref,
			sizeof(magna_brightness_aor_ref))) {
		brightness_packet[cmd_size].payload =
					magna_brightness_aor_ref;
		brightness_packet[cmd_size].dlen =
					sizeof(magna_brightness_aor_ref);
		cmd_size++;
	}

	/* elvss ****************************************************************************/
	/* 0xB6 setting */
	memcpy(magna_brightness_elvss_pre, magna_brightness_elvss_ref,
					sizeof(magna_brightness_elvss_ref));

	elvss_value = get_elvss_value(candela, id3);

	magna_brightness_elvss_ref[1] = elvss_value;

	// ELVSS lOW TEMPERATURE 
	if (get_auto_brightness() != 6 && mipi_pd.need_update)
		if (mipi_pd.temperature <= -20)
			magna_brightness_elvss_ref[1] -= 0x04;

	if (memcmp(magna_brightness_elvss_pre, magna_brightness_elvss_ref,
				sizeof(magna_brightness_elvss_ref))) {
		brightness_packet[cmd_size].payload =
					magna_brightness_elvss_ref;
		brightness_packet[cmd_size].dlen =
					sizeof(magna_brightness_elvss_ref);
		cmd_size++;
	}

	// ELVSS lOW TEMPERATURE
	if (mipi_pd.need_update) {
		magna_temperature_compensation[7] = mipi_pd.temperature_value;
		brightness_packet[cmd_size].payload = magna_temperature_compensation;
		brightness_packet[cmd_size].dlen = sizeof(magna_temperature_compensation);
		cmd_size++;
	}

	/* write als *************************************************************************/
	/* 0xE3 setting */
	if (mipi_pd.first_bl_hbm_psre  && (get_auto_brightness() >= 6)) {
		brightness_packet[cmd_size].payload =
				magna_brightness_write_als;
		brightness_packet[cmd_size].dlen =
				sizeof(magna_brightness_write_als);
		cmd_size++;
	}

	/* PSRE MTP *************************************************************************/
	/* 0xBB setting */
	memcpy(magna_psre_mtp_pre, magna_psre_mtp_ref,
						sizeof(magna_psre_mtp_ref));

	if (get_auto_brightness() == 6) {
		magna_psre_mtp_ref[1] = 0x04; // RE ON
	} else {
		magna_psre_mtp_ref[1] = 0x84; // RE OFF
	}

	if (memcmp(magna_psre_mtp_pre, magna_psre_mtp_ref,
						sizeof(magna_psre_mtp_ref))) {
		brightness_packet[cmd_size].payload =
				magna_psre_mtp_ref;
		brightness_packet[cmd_size].dlen =
				sizeof(magna_psre_mtp_ref);
		cmd_size++;
	}

	/* PSRE control 1 *************************************************************************/
	/* 0xBC setting */
	if (magna_psre_mtp_ref[1] == 0x04) {
		brightness_packet[cmd_size].payload =
				magna_brightness_psre_cont;
		brightness_packet[cmd_size].dlen =
				sizeof(magna_brightness_psre_cont);
		cmd_size++;
	}

	/* acl *****************************************************************/
	/* 0x55 setting */
	memcpy(magna_brightness_acl_pre, magna_brightness_acl_ref,
					sizeof(magna_brightness_acl_ref));

	if (get_auto_brightness() == 6) {
		magna_brightness_acl_ref[1] = 0x42; /*RE low, ACL 40%*/
	} else {
		if (mipi_pd.acl_status) 
			magna_brightness_acl_ref[1] = 0x02; /*ACL 40%*/
		else
			magna_brightness_acl_ref[1] = 0x00; /*ACL off*/
	}

	if (memcmp(magna_brightness_acl_pre, magna_brightness_acl_ref,
				sizeof(magna_brightness_acl_ref))) {
		brightness_packet[cmd_size].payload =
				magna_brightness_acl_ref;
		brightness_packet[cmd_size].dlen =
				sizeof(magna_brightness_acl_ref);
		cmd_size++;
	}

	/* gamma ******************************************************************************/
	/* 0xCA setting */
	mipi_pd.smart_se6e8fa.brightness_level = candela;

	generate_gamma(&mipi_pd.smart_se6e8fa,
			&(magna_brightness_gamma[1]), GAMMA_SET_MAX);

	if (get_auto_brightness() == 6) {
		for(i=1; i<=6; i++)
			magna_brightness_gamma[i] = magna_brightness_gamma_hbm[i];
		for(i=7; i<=12; i++)
			magna_brightness_gamma[i] = *(b1_reg+(i-7));
		for(i=13; i<=15; i++)
			magna_brightness_gamma[i] = *(b6_reg_magna+(i-13));
		for(i=16; i<=18; i++)
			magna_brightness_gamma[i] = *(b6_reg_magna+(i-1));
		for(i=19; i<=21; i++)
			magna_brightness_gamma[i] = *(b6_reg_magna+(i+1));
	}

	/*for debug*/
	printk("%s CA[1~21] : ",__func__);
	for (i=1; i<=21; i++)
		printk("(%x)",magna_brightness_gamma[i]);
	printk("\n");

	brightness_packet[cmd_size].payload = magna_brightness_gamma;
	brightness_packet[cmd_size].dlen = sizeof(magna_brightness_gamma);
	cmd_size++;

	/* hbm mode control ******************************************************************************/
	/* 0x53 setting */
	memcpy(hbm_mode_cont_pre, hbm_mode_cont_ref, sizeof(hbm_mode_cont_ref));

	if (get_auto_brightness() == 6) {
		hbm_mode_cont_ref[1] = 0xA0; /* HBM Mid on (400nit) */
	} else {
		hbm_mode_cont_ref[1] = 0x00; /* HBM off */
	}

	if (memcmp(hbm_mode_cont_pre, hbm_mode_cont_ref, sizeof(hbm_mode_cont_ref))) {
		brightness_packet[cmd_size].payload = hbm_mode_cont_ref;
		brightness_packet[cmd_size].dlen = sizeof(hbm_mode_cont_ref);
		cmd_size++;
	}

	/* gamma update ***********************************************************************/
	/* 0xF7 setting */
	brightness_packet[cmd_size].payload = magna_gamma_update;
	brightness_packet[cmd_size].dlen = sizeof(magna_gamma_update);
	cmd_size++;


	/* LEVEL KEY OFF */
	brightness_packet[cmd_size].payload = magna_test_key_off1;
	brightness_packet[cmd_size].dlen = sizeof(magna_test_key_off1);
	cmd_size++;

	brightness_packet[cmd_size].payload = magna_test_key_off2;
	brightness_packet[cmd_size].dlen = sizeof(magna_test_key_off2);
	cmd_size++;

	mipi_pd.brightness.size = cmd_size;
	mipi_pd.brightness_level = candela;

	mipi_pd.need_update = 0;

	return 1;
}

static int acl_control(int bl_level)
{

	if (get_auto_brightness() == 6) {
		magna_brightness_acl_ref[1] = 0x42; /*RE low, ACL 40%*/
	} else {
		if (mipi_pd.acl_status) 
			magna_brightness_acl_ref[1] = 0x02; /*ACL 40%*/
		else
			magna_brightness_acl_ref[1] = 0x00; /*ACL off*/
	}

	return 1;
}

static int cmd_set_change(int cmd_set, int panel_id)
{
	int id1, id2, id3;
	id1 = (panel_id & 0x00FF0000) >> 16;
	id2 = (panel_id & 0x0000FF00) >> 8;
	id3 = panel_id & 0xFF;

	switch (cmd_set) {
	case PANEL_ON:
		if (id3 == MAGNA_C_FIRST_ID || id3 == MAGNA_C_SECOND_ID) {
			mipi_pd.on.cmd = magna_on_cmds_revB;
			mipi_pd.on.size =
				ARRAY_SIZE(magna_on_cmds_revB);
		} else {
			mipi_pd.on.cmd = magna_on_cmds_revF;
			mipi_pd.on.size =
				ARRAY_SIZE(magna_on_cmds_revF);
		}
		break;
	default:
		;
	}

	return 0;
}

static void reset_bl_level(void)
{
	/* copy current brightness level */
	if (memcmp(magna_brightness_aor_ref, magna_brightness_aor_default,
				sizeof(magna_brightness_aor_ref))) {
		memcpy(magna_brightness_aor_condition,
				magna_brightness_aor_ref,
				sizeof(magna_brightness_aor_condition));
	}

	if (memcmp(magna_brightness_elvss_ref, magna_brightness_elvss_default,
				sizeof(magna_brightness_elvss_ref))) {
		memcpy(magna_elvss_condition,
					magna_brightness_elvss_ref,
					sizeof(magna_elvss_condition));
	}


	/* reset brightness change value */
	memcpy(magna_brightness_aor_ref, magna_brightness_aor_default,
					sizeof(magna_brightness_aor_ref));
	memcpy(magna_brightness_acl_ref, magna_brightness_acl_default,
					sizeof(magna_brightness_acl_ref));
	memcpy(magna_brightness_elvss_ref, magna_brightness_elvss_default,
					sizeof(magna_brightness_elvss_ref));

}

static int lpts_set_change(int fps)
{
	if (fps == 60)
		panel_lpts[0].payload = magna_ltps_60;
	else if (fps == 59)
		panel_lpts[0].payload = magna_ltps_59;
	else if (fps == 58)
		panel_lpts[0].payload = magna_ltps_58;
	else if (fps == 57)
		panel_lpts[0].payload = magna_ltps_57;
	else if (fps == 56)
		panel_lpts[0].payload = magna_ltps_56;
	else if (fps == 55)
		panel_lpts[0].payload = magna_ltps_55;
	else if (fps == 54)
		panel_lpts[0].payload = magna_ltps_54;
	else if (fps == 53)
		panel_lpts[0].payload = magna_ltps_53;
	else if (fps == 52)
		panel_lpts[0].payload = magna_ltps_52;
	else if (fps == 51)
		panel_lpts[0].payload = magna_ltps_51;
	else if (fps == 50)
		panel_lpts[0].payload = magna_ltps_50;
	else if (fps == 49)
		panel_lpts[0].payload = magna_ltps_49;
	else if (fps == 48)
		panel_lpts[0].payload = magna_ltps_48;
	else if (fps == 47)
		panel_lpts[0].payload = magna_ltps_47;
	else if (fps == 46)
		panel_lpts[0].payload = magna_ltps_46;
	else if (fps == 45)
		panel_lpts[0].payload = magna_ltps_45;
	else if (fps == 44)
		panel_lpts[0].payload = magna_ltps_44;
	else if (fps == 43)
		panel_lpts[0].payload = magna_ltps_43;
	else if (fps == 42)
		panel_lpts[0].payload = magna_ltps_42;
	else
		panel_lpts[0].payload = magna_ltps_60;

	return 1;
}

static struct mipi_panel_data mipi_pd = {
	.panel_name	= "AMS499QP01-0\n",
	.on		= {magna_on_cmds_revB
				, ARRAY_SIZE(magna_on_cmds_revB)},
	.off		= {panel_off_cmds
				, ARRAY_SIZE(panel_off_cmds)},
	.late_on		= {panel_late_on_cmds
				, ARRAY_SIZE(panel_late_on_cmds)},
	.early_off	= {panel_early_off_cmds
				, ARRAY_SIZE(panel_early_off_cmds)},

	.brightness	= {brightness_packet
				, ARRAY_SIZE(brightness_packet)},
	.backlight_control = brightness_control,

	.reset_bl_level = reset_bl_level,

	.mtp_enable	= {panel_mtp_enable_cmds
				, ARRAY_SIZE(panel_mtp_enable_cmds)},
	.mtp_disable	= {panel_mtp_disable_cmds
				, ARRAY_SIZE(panel_mtp_disable_cmds)},
	.need_flip	= {panel_need_flip_cmds
				, ARRAY_SIZE(panel_need_flip_cmds)},
	.lpts		= {panel_lpts
				,ARRAY_SIZE(panel_lpts)},

	.cmd_set_change = cmd_set_change,

	.lux_table = lux_tbl,
	.lux_table_max_cnt = ARRAY_SIZE(lux_tbl),

	.acl_control = acl_control,
	.acl_cmds = {panel_acl_cmds
				, ARRAY_SIZE(panel_acl_cmds)},

	.lpts_set_change = lpts_set_change,
};

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	/* 1920*1200, RGB888, 4 Lane 60 fps video mode */
	/* regulator */
	.regulator = {0x03, 0x0a, 0x04, 0x00, 0x20},
	/* timing */
	.timing = {0xE0, 0x48, 0x25, 0x00, 0x66, 0x6D, 0x2A, 0x4B,
	0x3F, 0x03, 0x04, 0xa0},
	/* phy ctrl */
	.ctrl = {0x5f, 0x00, 0x00, 0x10},
	/* strength */
	.strength = {0xff, 0x00, 0x06, 0x00},
	/* pll control */
	.pll = {0x0, 0x7f, 0x31, 0xda, 0x00, 0x50, 0x48, 0x63,
	0x71, 0x0f, 0x03,
	0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
};

static int __init mipi_video_magna_octa_full_hd_pt_init(void)
{
	int ret;

	printk(KERN_DEBUG "[lcd] mipi_video_magna_octa_full_hd_pt_init start\n");

	/*
	* LSI 1, MAGNA 0
	*/
	pr_info("get_ldi_chip : %d %s", get_ldi_chip(),
				get_ldi_chip() > 0 ? "LSI" : "MAGNA");

	if (get_ldi_chip() != LDI_MAGNA)
		return 1;

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_cmd_magna_oled_qhd"))
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

	pinfo.lcdc.h_back_porch = 50; // hbp + hpw = 400ns
	pinfo.lcdc.h_front_porch = 145;
	pinfo.lcdc.h_pulse_width = 10;

	pinfo.lcdc.v_back_porch = 4;
	pinfo.lcdc.v_front_porch = 10;
	pinfo.lcdc.v_pulse_width = 2;

	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
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
	pinfo.mipi.esc_byte_ratio = 2;
	pinfo.lcd.blt_ctrl = 1; //OVERLAY_BLT_SWITCH_TG_OFF;

	ret = mipi_samsung_octa_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_FULL_HD_PT,
						&mipi_pd);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	printk(KERN_DEBUG "%s: get_lcd_attached(%d)!\n",
				__func__, get_lcd_attached());
	if (get_lcd_attached() == 0)
		return -ENODEV;

	printk(KERN_DEBUG "[lcd] mipi_video_magna_octa_full_hd_pt_init end\n");

	return ret;
}
module_init(mipi_video_magna_octa_full_hd_pt_init);

