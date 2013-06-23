/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef MIPI_SAMSUNG_OLED_H
#define MIPI_SAMSUNG_OLED_H

#define AUTO_BRIGHTNESS_CABC_FUNCTION

//#include "smart_dimming.h"

enum mipi_samsung_cmd_list {

	PANEL_READY_TO_ON,
	PANEL_READY_TO_OFF,
	PANEL_ON,
	PANEL_OFF,
	PANEL_LATE_ON,
	PANEL_EARLY_OFF,
	PANEL_BRIGHT_CTRL,
	PANEL_MTP_ENABLE,
	PANEL_MTP_DISABLE,
	PANEL_NEED_FLIP,
#if defined(AUTO_BRIGHTNESS_CABC_FUNCTION)
	PANEL_CABC_ENABLE,
	PANEL_CABC_DISABLE,
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_PREVENT_HSYNC_LEAKAGE)
	PANEL_HSYNC_ON,
	PANEL_HSYNC_OFF,
#endif
};

enum {
	MIPI_SUSPEND_STATE,
	MIPI_RESUME_STATE,
};

struct cmd_set {
	struct dsi_cmd_desc *cmd;
	int size;
};


struct display_status {
	unsigned char auto_brightness;
};

struct mipi_samsung_driver_data {
	struct dsi_buf samsung_tx_buf;
	struct dsi_buf samsung_rx_buf;
	struct msm_panel_common_pdata *mipi_samsung_disp_pdata;
	struct mipi_panel_data *mpd;
	struct display_status dstat;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_LCD_CLASS_DEVICE)
	struct platform_device *msm_pdev;
#endif
};

struct mipi_panel_data {
	const char panel_name[20];
	struct cmd_set on;
	struct cmd_set off;
	struct cmd_set late_on;
	struct cmd_set early_off;
	struct cmd_set mtp_enable;
	struct cmd_set mtp_disable;
	struct cmd_set need_flip;
	struct cmd_set brightness;
#if defined(AUTO_BRIGHTNESS_CABC_FUNCTION)
	struct cmd_set cabc_enable;
	struct cmd_set cabc_disable;
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_PREVENT_HSYNC_LEAKAGE)
	struct cmd_set hsync_on;
	struct cmd_set hsync_off;
#endif

	unsigned int manufacture_id;
	struct mipi_samsung_driver_data *msd;

	int (*backlight_control)(int bl_level);

	int siop_status;
};

int mipi_samsung_cabc_onoff ( int enable );
int mipi_samsung_tft_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel,
					struct mipi_panel_data *mpd);

extern struct mutex dsi_tx_mutex;
#endif  /* MIPI_SAMSUNG_OLED_H */
