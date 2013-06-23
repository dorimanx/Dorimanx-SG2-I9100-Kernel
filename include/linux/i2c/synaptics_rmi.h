/*
 * Copyright (c) 2011, 2012 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#ifndef _SYNAPTICS_RMI4_GENERIC_H_
#define _SYNAPTICS_RMI4_GENERIC_H_

#define SYNAPTICS_HW_RESET_TIME	400
#define SYNAPTICS_HW_RESET_TIME_B0	100

#define OCTA_PANEL_REVISION_51	0x03
#define OCTA_PANEL_REVISION_43	0x02
#define OCTA_PANEL_REVISION_40	0x01
#define OCTA_PANEL_REVISION_34	0x00

struct synaptics_rmi_f1a_button_map {
	unsigned char nbuttons;
	unsigned char *map;
};

extern struct synaptics_rmi_callbacks *charger_callbacks;
struct synaptics_rmi_callbacks {
	void (*inform_charger)(struct synaptics_rmi_callbacks *, int);
};

/**
 * struct synaptics_rmi4_platform_data - rmi4 platform data
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @regulator_en: regulator enable flag
 * @gpio: attention interrupt gpio
 * @irq_type: irq type
 * @gpio_config: pointer to gpio configuration function
 * @f1a_button_map: pointer to 0d button map
 */
struct synaptics_rmi4_platform_data {
	bool x_flip;
	bool y_flip;
	int panel_revision;	/* to identify panel info */
	int panel_touch_type;	/* to display panel touch type */
	bool regulator_en;
	bool factory_flatform;
	bool recovery_mode;
	unsigned gpio;
	int irq_type;
	int max_x;
	int max_y;
	int max_width;
    int num_of_rx;
    int num_of_tx;
	struct synaptics_rmi_f1a_button_map *f1a_button_map;
	int (*gpio_config)(unsigned interrupt_gpio, bool configure);
	void (*power)(bool enable);
	void (*register_cb)(struct synaptics_rmi_callbacks *);
#ifdef CONFIG_FB_MSM_MIPI_SAMSUNG_OCTA_VIDEO_FULL_HD_PT_PANEL
	int (*tout1_on)(void);
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_PREVENT_HSYNC_LEAKAGE)
	void (*hsync_onoff)(bool onoff);
#endif
};

#ifdef CONFIG_FB_MSM_MIPI_SAMSUNG_OCTA_VIDEO_FULL_HD_PT_PANEL
extern int touch_tout1_on(void);
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_PREVENT_HSYNC_LEAKAGE)
extern void lcd_hsync_onoff(bool onoff);
#endif
#endif
