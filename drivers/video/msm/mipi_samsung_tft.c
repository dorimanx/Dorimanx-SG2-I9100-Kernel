/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/lcd.h>

#include "msm_fb.h"
#include "msm_fb_panel.h"
#include "mipi_dsi.h"
#include "mipi_samsung_tft.h"
#include "mdp4.h"

#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/pm8821.h>
#include "../../../arch/arm/mach-msm/board-8064.h"
#include <linux/gpio.h>

#if defined(CONFIG_FB_MDP4_ENHANCE)
#include "mdp4_video_enhance.h"
#elif defined(CONFIG_MDNIE_LITE_TUNING)
#include "mdnie_lite_tuning.h"
#endif

#define DDI_VIDEO_ENHANCE_TUNING
#if defined(DDI_VIDEO_ENHANCE_TUNING)
#include <linux/syscalls.h>
#endif

static struct mipi_samsung_driver_data msd;
static int lcd_attached = 1;
struct mutex dsi_tx_mutex;

#if defined(AUTO_BRIGHTNESS_CABC_FUNCTION)
static int is_disp_on = 0;
static int is_cabc_delayed = 0;
#endif

#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
struct work_struct  err_fg_work;
#define PMIC_GPIO_ERR_FG 8
static int err_fg_gpio;	/* PM_GPIO8 */
struct pm_gpio gpio_get_param = {
	.direction	= PM_GPIO_DIR_IN,
	.pull		= PM_GPIO_PULL_NO,
	.vin_sel		= 2,
	.function	= PM_GPIO_FUNC_NORMAL,
	.inv_int_pol	= 0,
};
static int err_fg_working;
#endif

#if defined(RUMTIME_MIPI_CLK_CHANGE)
static int current_fps = 0;
#endif

static int atoi(const char *name)
{
	int val = 0;

	for (;; name++) {
		switch (*name) {
		case '0' ... '9':
			val = 10*val+(*name-'0');
			break;
		default:
			return val;
		}
	}
}

static int mipi_samsung_disp_send_cmd(struct msm_fb_data_type *mfd,
		enum mipi_samsung_cmd_list cmd,
		unsigned char lock)
{
	struct dsi_cmd_desc *cmd_desc;
	int cmd_size = 0;

	if (mfd->panel.type == MIPI_VIDEO_PANEL)
		mutex_lock(&dsi_tx_mutex);
	else {
		if (lock)
			mutex_lock(&mfd->dma->ov_mutex);
	}

		switch (cmd) {
		case PANEL_ON:
			cmd_desc = msd.mpd->on.cmd;
			cmd_size = msd.mpd->on.size;
			break;
		case PANEL_OFF:
			cmd_desc = msd.mpd->off.cmd;
			cmd_size = msd.mpd->off.size;
			break;
		case PANEL_LATE_ON:
			cmd_desc = msd.mpd->late_on.cmd;
			cmd_size = msd.mpd->late_on.size;
			break;
		case PANEL_EARLY_OFF:
			cmd_desc = msd.mpd->early_off.cmd;
			cmd_size = msd.mpd->early_off.size;
			break;
		case PANEL_BRIGHT_CTRL:
			cmd_desc = msd.mpd->brightness.cmd;
			cmd_size = msd.mpd->brightness.size;
			break;
		case PANEL_MTP_ENABLE:
			cmd_desc = msd.mpd->mtp_enable.cmd;
			cmd_size = msd.mpd->mtp_enable.size;
			break;
		case PANEL_MTP_DISABLE:
			cmd_desc = msd.mpd->mtp_disable.cmd;
			cmd_size = msd.mpd->mtp_disable.size;
			break;
		case PANEL_NEED_FLIP:
			cmd_desc = msd.mpd->need_flip.cmd;
			cmd_size = msd.mpd->need_flip.size;
			break;
#if defined(AUTO_BRIGHTNESS_CABC_FUNCTION)
		case PANEL_CABC_ENABLE:
			cmd_desc = msd.mpd->cabc_enable.cmd;
			cmd_size = msd.mpd->cabc_enable.size;
			break;
		case PANEL_CABC_DISABLE:
			cmd_desc = msd.mpd->cabc_disable.cmd;
			cmd_size = msd.mpd->cabc_disable.size;
			break;
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_PREVENT_HSYNC_LEAKAGE)
		case PANEL_HSYNC_ON:
			cmd_desc = msd.mpd->hsync_on.cmd;
			cmd_size = msd.mpd->hsync_on.size;
			break;
		case PANEL_HSYNC_OFF:
			cmd_desc = msd.mpd->hsync_off.cmd;
			cmd_size = msd.mpd->hsync_off.size;
			break;
#endif
		default:
			goto unknown_command;
			;
	}

	if (!cmd_size)
		goto unknown_command;

	if (mfd->panel_info.type == MIPI_CMD_PANEL) {
		mipi_dsi_mdp_busy_wait();
	}

	mipi_dsi_cmds_tx(&msd.samsung_tx_buf, cmd_desc, cmd_size);

	
	if (mfd->panel.type == MIPI_VIDEO_PANEL)
		mutex_unlock(&dsi_tx_mutex);
	else {
		if (lock)
			mutex_unlock(&mfd->dma->ov_mutex);
	}

	return 0;

unknown_command:
	if (mfd->panel.type == MIPI_VIDEO_PANEL)
		mutex_unlock(&dsi_tx_mutex);
	else {
		if (lock)
			mutex_unlock(&mfd->dma->ov_mutex);
	}

	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_PREVENT_HSYNC_LEAKAGE)
void lcd_hsync_onoff(bool onoff)
{
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(msd.msm_pdev);

	if (unlikely(!mfd)) { pr_err("%s : panel no mfd",__func__); return;}
//		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY)) { pr_err("%s : panel mfd invlaid",__func__); return;}
//		return -EINVAL;

#if defined(CONFIG_MACH_JACTIVE_EUR) /* HW DEFECT under REV 0.5 */
	if( system_rev > 15 )
		return;
#endif

	if (mfd->panel_power_on == TRUE)
		{ 
			if( onoff )
				{
				msleep(30);
				mipi_samsung_disp_send_cmd(mfd, PANEL_HSYNC_ON, true);
				pr_info("%s : HSYNC On\n",__func__);
				}
			else
				{
				mipi_samsung_disp_send_cmd(mfd, PANEL_HSYNC_OFF, true);
				msleep(10); /* Need time to discharging by capacitance*/
				pr_info("%s : HSYNC Off\n",__func__);
				}
		}
	else
		pr_err("%s : panel power off\n",__func__);
	
return;
}
#endif
static char manufacture_id1[2] = {0xDA, 0x00}; /* DTYPE_DCS_READ */
static char manufacture_id2[2] = {0xDB, 0x00}; /* DTYPE_DCS_READ */
static char manufacture_id3[2] = {0xDC, 0x00}; /* DTYPE_DCS_READ */
static char manufacture_id4[2] = {0xC1, 0x00}; /* DTYPE_DCS_READ */

static struct dsi_cmd_desc samsung_manufacture_id1_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id1), manufacture_id1};
static struct dsi_cmd_desc samsung_manufacture_id2_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id2), manufacture_id2};
static struct dsi_cmd_desc samsung_manufacture_id3_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id3), manufacture_id3};
static struct dsi_cmd_desc samsung_manufacture_id4_cmd = {
	DTYPE_GEN_READ, 1, 0, 1, 5, sizeof(manufacture_id4), manufacture_id4};

static uint32 mipi_samsung_manufacture_id(struct msm_fb_data_type *mfd)
{
	struct dsi_buf *rp, *tp;
	struct dsi_cmd_desc *cmd;
	uint32 id = 0;

	tp = &msd.samsung_tx_buf;
	rp = &msd.samsung_rx_buf;
	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);

	if (system_rev == 0) {
		pr_err("[LCD] %s, temp return! for rev00\n", __func__);
		return 0;
	}

	cmd = &samsung_manufacture_id1_cmd;
	mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 1);
	pr_info("%s: manufacture_id1=%x\n", __func__, *rp->data);
	id = *rp->data & 0xFF;
	id <<= 8;

	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);
	cmd = &samsung_manufacture_id2_cmd;
	mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 1);
	pr_info("%s: manufacture_id2=%x\n", __func__, *rp->data);
	id |= *rp->data & 0xFF;
	id <<= 8;

	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);
	cmd = &samsung_manufacture_id3_cmd;
	mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 1);
	pr_info("%s: manufacture_id3=%x\n", __func__, *rp->data);
	id |= *rp->data & 0xFF;
	id <<= 8;

	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);
	cmd = &samsung_manufacture_id4_cmd;
	mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 1);
	pr_info("%s: manufacture_id4=%x\n", __func__, *rp->data);
	id |= *rp->data & 0xFF;

	pr_info("%s: manufacture_id=%x\n", __func__, id);

	return id;
}

static void execute_panel_init(struct msm_fb_data_type *mfd)
{
	/* read LID ID */
	mipi_samsung_disp_send_cmd(mfd, PANEL_MTP_ENABLE, false);

	msd.mpd->manufacture_id = mipi_samsung_manufacture_id(mfd);

	mipi_samsung_disp_send_cmd(mfd, PANEL_MTP_DISABLE, false);
}


static int mipi_samsung_disp_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd = NULL;
	struct mipi_panel_info *mipi = NULL;
	static int first_boot_on = 0;

	printk(KERN_INFO "[lcd] mipi_samsung_disp_on start\n");

	mfd = platform_get_drvdata(pdev);
	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	mipi = &mfd->panel_info.mipi;

	if (system_rev == 0) {
		if (!first_boot_on) {
			execute_panel_init(mfd);
			first_boot_on = 1;
		}

		if ((msd.mpd->manufacture_id & 0xFF) == 0x84 || \
			(msd.mpd->manufacture_id & 0xFF) == 0x4)
			mipi_samsung_disp_send_cmd(mfd,
						PANEL_NEED_FLIP, false);
	}

	mipi_samsung_disp_send_cmd(mfd, PANEL_ON, false);
	mfd->resume_state = MIPI_RESUME_STATE;

#if	defined(CONFIG_MDNIE_LITE_TUNING)
	if (!first_boot_on)
	{
		mfd->resume_state = MIPI_RESUME_STATE;
		first_boot_on = 1;
	}
#endif

#if defined(RUMTIME_MIPI_CLK_CHANGE)
	current_fps = mfd->panel_info.mipi.frame_rate;
#endif

#if defined(AUTO_BRIGHTNESS_CABC_FUNCTION)
	is_disp_on = 1;

	if( (is_cabc_delayed == 1) || (msd.mpd->siop_status == true) )
	{
		mipi_samsung_disp_send_cmd(mfd, PANEL_CABC_ENABLE, false);
		is_cabc_delayed = 0;
		printk ( KERN_ERR "%s-PANEL_CABC_ENABLE(delayed)\n", __func__ );
	}
#endif

	printk(KERN_INFO "[lcd] mipi_samsung_disp_on end %d\n", gpio_get_value(err_fg_gpio));

#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	enable_irq(PM8921_GPIO_IRQ(PM8921_IRQ_BASE, PMIC_GPIO_ERR_FG));
#endif
	return 0;
}


static int mipi_samsung_disp_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd = NULL;

	mfd = platform_get_drvdata(pdev);
	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	if (!err_fg_working) {
		disable_irq_nosync(PM8921_GPIO_IRQ(PM8921_IRQ_BASE, PMIC_GPIO_ERR_FG));
		cancel_work_sync(&err_fg_work);
	}
#endif
	mfd->resume_state = MIPI_SUSPEND_STATE;
	mipi_samsung_disp_send_cmd(mfd, PANEL_OFF, false);

#if defined(RUMTIME_MIPI_CLK_CHANGE)
	if (mfd->panel_info.mipi.frame_rate != current_fps)
		mipi_runtime_clk_change(mfd->panel_info.mipi.frame_rate);
#endif

#if defined(AUTO_BRIGHTNESS_CABC_FUNCTION)
	is_disp_on = 0;
#endif

	return 0;
}

static void __devinit mipi_samsung_disp_shutdown(struct platform_device *pdev)
{
	static struct mipi_dsi_platform_data *mipi_dsi_pdata = NULL;

	if (pdev->id != 0)
		return;

	mipi_dsi_pdata = pdev->dev.platform_data;
	if (mipi_dsi_pdata == NULL) {
		pr_err("LCD Power off failure: No Platform Data\n");
		return;
	}


	if (mipi_dsi_pdata && mipi_dsi_pdata->active_reset)
		mipi_dsi_pdata->active_reset(0); /* low */

	usleep(2000); /*1ms delay(minimum) required between reset low and AVDD off*/

#if defined(CONFIG_MACH_JACTIVE_EUR)
	msleep ( 10 ); // need more delay for POWER OFF SIGNAL
#endif

	if (mipi_dsi_pdata && mipi_dsi_pdata->panel_power_save)
		mipi_dsi_pdata->panel_power_save(0);

	if (mipi_dsi_pdata && mipi_dsi_pdata->dsi_power_save)
		mipi_dsi_pdata->dsi_power_save(0);
}

static void mipi_samsung_disp_backlight(struct msm_fb_data_type *mfd)
{
	msd.mpd->backlight_control(mfd->bl_level);
	mipi_samsung_disp_send_cmd(mfd, PANEL_BRIGHT_CTRL, true);
	pr_info("mipi_samsung_disp_backlight %d\n", mfd->bl_level);

}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void mipi_samsung_disp_early_suspend(struct early_suspend *h)
{
	struct msm_fb_data_type *mfd = NULL;
	pr_info("[lcd] %s\n", __func__);

	mfd = platform_get_drvdata(msd.msm_pdev);
	if (unlikely(!mfd)) {
		pr_info("%s NO PDEV.\n", __func__);
		return;
	}
	if (unlikely(mfd->key != MFD_KEY)) {
		pr_info("%s MFD_KEY is not matched.\n", __func__);
		return;
	}
}

static void mipi_samsung_disp_late_resume(struct early_suspend *h)
{
	struct msm_fb_data_type *mfd = NULL;
	mfd = platform_get_drvdata(msd.msm_pdev);
	if (unlikely(!mfd)) {
		pr_info("%s NO PDEV.\n", __func__);
		return;
	}
	if (unlikely(mfd->key != MFD_KEY)) {
		pr_info("%s MFD_KEY is not matched.\n", __func__);
		return;
	}

	pr_info("[lcd] %s\n", __func__);
}
#endif

#if defined(CONFIG_LCD_CLASS_DEVICE)
static ssize_t mipi_samsung_disp_get_power(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct msm_fb_data_type *mfd = NULL;
	int rc;

	mfd = platform_get_drvdata(msd.msm_pdev);
	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	rc = snprintf((char *)buf, sizeof(buf), "%d\n", mfd->panel_power_on);
	pr_info("mipi_samsung_disp_get_power(%d)\n", mfd->panel_power_on);

	return rc;
}

static ssize_t mipi_samsung_disp_set_power(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct msm_fb_data_type *mfd = NULL;
	unsigned int power;

	mfd = platform_get_drvdata(msd.msm_pdev);
	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	if (sscanf(buf, "%u", &power) != 1)
		return -EINVAL;

	if (power == mfd->panel_power_on)
		return 0;

	if (power) {
		mfd->fbi->fbops->fb_blank(FB_BLANK_UNBLANK, mfd->fbi);
		mfd->fbi->fbops->fb_pan_display(&mfd->fbi->var, mfd->fbi);
		mipi_samsung_disp_send_cmd(mfd, PANEL_LATE_ON, true);
		mipi_samsung_disp_backlight(mfd);
	} else {
		mfd->fbi->fbops->fb_blank(FB_BLANK_POWERDOWN, mfd->fbi);
	}

	pr_info("mipi_samsung_disp_set_power\n");

	return size;
}
#endif

int mipi_samsung_cabc_onoff ( int enable )
{
	struct msm_fb_data_type *mfd = NULL;

	mfd = platform_get_drvdata(msd.msm_pdev);

	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	if( is_disp_on == 1 )
	{
		if( enable )
		{
			mipi_samsung_disp_send_cmd(mfd, PANEL_CABC_ENABLE, false);
			printk ( KERN_ERR "@@@@@%s-PANEL_CABC_ENABLE\n", __func__ );
		}
		else
		{
			mipi_samsung_disp_send_cmd(mfd, PANEL_CABC_DISABLE, false);
			printk ( KERN_ERR "@@@@@%s-PANEL_CABC_DISABLE\n", __func__ );
		}
	}

	return 0;
}

static ssize_t mipi_samsung_disp_lcdtype_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char temp[20];

	snprintf(temp, strnlen(msd.mpd->panel_name, 20) + 1,
			msd.mpd->panel_name);
	strlcat(buf, temp, 20);
	return strnlen(buf, 20);
}

static ssize_t mipi_samsung_auto_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf((char *)buf, sizeof(buf), "%d\n",
			msd.dstat.auto_brightness);
	pr_info("auot_brightness: %d\n", *buf);

	return rc;
}

static ssize_t mipi_samsung_auto_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
#if defined(AUTO_BRIGHTNESS_CABC_FUNCTION)
	struct msm_fb_data_type *mfd = NULL;

	mfd = platform_get_drvdata(msd.msm_pdev);
	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	// 0, 5, 6     => CABC OFF
	// 1, 2, 3, 4 => CABC ON
	if (sysfs_streq(buf, "0"))
		msd.dstat.auto_brightness = 0;
	else if (sysfs_streq(buf, "1"))
		msd.dstat.auto_brightness = 1;
	else if (sysfs_streq(buf, "2"))
		msd.dstat.auto_brightness = 2;
	else if (sysfs_streq(buf, "3"))
		msd.dstat.auto_brightness = 3;
	else if (sysfs_streq(buf, "4"))
		msd.dstat.auto_brightness = 4;
	else if (sysfs_streq(buf, "5"))
		msd.dstat.auto_brightness = 5;
	else if (sysfs_streq(buf, "6"))
		msd.dstat.auto_brightness = 6;
	else
		pr_info("%s: Invalid argument!!", __func__);

	if( (msd.dstat.auto_brightness >= 1) && (msd.dstat.auto_brightness <= 4) )
	{
		if( is_disp_on == 1 )
			mipi_samsung_cabc_onoff ( 1 );
		else
			is_cabc_delayed = 1;
	}
	else
	{
		if( msd.mpd->siop_status == false )
			mipi_samsung_cabc_onoff ( 0 );
	}

#else
	if (sysfs_streq(buf, "1"))
		msd.dstat.auto_brightness = 1;
	else if (sysfs_streq(buf, "0"))
		msd.dstat.auto_brightness = 0;
	else
		pr_info("%s: Invalid argument!!", __func__);
#endif

	return size;
}

static ssize_t mipi_samsung_disp_backlight_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int rc;
	struct msm_fb_data_type *mfd = NULL;
	mfd = platform_get_drvdata(msd.msm_pdev);
	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	rc = snprintf((char *)buf, sizeof(buf), "%d\n", mfd->bl_level);

	return rc;
}

static ssize_t mipi_samsung_disp_backlight_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct msm_fb_data_type *mfd = NULL;
	int level = atoi(buf);

	mfd = platform_get_drvdata(msd.msm_pdev);
	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	mfd->bl_level = level;
	
	if (mfd->resume_state == MIPI_RESUME_STATE) {
		mipi_samsung_disp_backlight(mfd);
		pr_info("%s : level (%d)\n",__func__,level);
	} else {
		pr_info("%s : panel is off state!!\n", __func__);
	}
	return size;
}

static struct lcd_ops mipi_samsung_disp_props = {
	.get_power = NULL,
	.set_power = NULL,
};

#if defined(RUMTIME_MIPI_CLK_CHANGE)
static ssize_t mipi_samsung_fps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf((char *)buf, 20, "%d\n", current_fps);

	return rc;
}

static ssize_t mipi_samsung_fps_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct msm_fb_data_type *mfd = NULL;
	int goal_fps;
	int level = atoi(buf);

	mfd = platform_get_drvdata(msd.msm_pdev);
	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	if (mfd->panel_power_on == FALSE) {
		pr_err("%s fps set error, panel power off 1", __func__);
		return size;
	}

	if (level == 0)
		goal_fps = 60;
	else if (level == 1)
		goal_fps = 42;
	else if (level == 2)
		goal_fps = 51;
	else {
		pr_info("%s fps set error : invalid level %d", __func__, level);
		return size;
	}

	if (current_fps != goal_fps)
		current_fps = goal_fps;
	else
		return size;

	mutex_lock(&dsi_tx_mutex);

	if (mfd->panel_power_on == FALSE) {
		mutex_unlock(&dsi_tx_mutex);
		pr_info("%s fps set error, panel power off 2", __func__);
		return size;
	} else {
		mipi_runtime_clk_change(current_fps);
		mutex_unlock(&dsi_tx_mutex);
	}

	pr_info("%s goal_fps : %d", __func__, goal_fps);

	return size;
}
#endif

static ssize_t mipi_samsung_disp_siop_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf((char *)buf, sizeof(buf), "%d\n", msd.mpd->siop_status);
	pr_info("siop status: %d\n", *buf);

	return rc;
}

static ssize_t mipi_samsung_disp_siop_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	if (sysfs_streq(buf, "1"))
	{
		msd.mpd->siop_status = true;
		mipi_samsung_cabc_onoff ( 1 );
	}
	else if (sysfs_streq(buf, "0"))
	{
		msd.mpd->siop_status = false;
		if( msd.dstat.auto_brightness == false )
			mipi_samsung_cabc_onoff ( 0 );
	}
	else {
		pr_info("%s: Invalid argument!!", __func__);
		return size;
	}

	pr_info("%s : auto_brightness (%d) siop_status (%d)",
			__func__, msd.dstat.auto_brightness, msd.mpd->siop_status);

	return size;
}

static DEVICE_ATTR(lcd_power, S_IRUGO | S_IWUSR,
		mipi_samsung_disp_get_power,
		mipi_samsung_disp_set_power);
static DEVICE_ATTR(lcd_type, S_IRUGO, mipi_samsung_disp_lcdtype_show, NULL);
static DEVICE_ATTR(auto_brightness, S_IRUGO | S_IWUSR | S_IWGRP,
		mipi_samsung_auto_brightness_show,
		mipi_samsung_auto_brightness_store);
static DEVICE_ATTR(backlight, S_IRUGO | S_IWUSR | S_IWGRP,
			mipi_samsung_disp_backlight_show,
			mipi_samsung_disp_backlight_store);
#if defined(RUMTIME_MIPI_CLK_CHANGE)
static DEVICE_ATTR(fps_change, S_IRUGO | S_IWUSR | S_IWGRP,
		mipi_samsung_fps_show,
		mipi_samsung_fps_store);
#endif
static DEVICE_ATTR(siop_enable, S_IRUGO | S_IWUSR | S_IWGRP,
		mipi_samsung_disp_siop_show,
		mipi_samsung_disp_siop_store);

#ifdef DDI_VIDEO_ENHANCE_TUNING
#define MAX_FILE_NAME 128
#define TUNING_FILE_PATH "/sdcard/"
#define TUNE_FIRST_SIZE 5
#define TUNE_SECOND_SIZE 108
static char tuning_file[MAX_FILE_NAME];

static char mdni_tuning1[TUNE_FIRST_SIZE];
static char mdni_tuning2[TUNE_SECOND_SIZE];

static char level1_key[] = {
	0xF0,
	0x5A, 0x5A,
};

static char level2_key[] = {
	0xF1,
	0x5A, 0x5A,
};

static struct dsi_cmd_desc mdni_tune_cmd[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(level1_key), level1_key},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(level2_key), level2_key},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(mdni_tuning1), mdni_tuning1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(mdni_tuning2), mdni_tuning2},
};

static char char_to_dec(char data1, char data2)
{
	char dec;

	dec = 0;

	if (data1 >= 'a') {
		data1 -= 'a';
		data1 += 10;
	} else if (data1 >= 'A') {
		data1 -= 'A';
		data1 += 10;
	} else
		data1 -= '0';

	dec = data1 << 4;

	if (data2 >= 'a') {
		data2 -= 'a';
		data2 += 10;
	} else if (data2 >= 'A') {
		data2 -= 'A';
		data2 += 10;
	} else
		data2 -= '0';

	dec |= data2;

	return dec;
}
static void sending_tune_cmd(char *src, int len)
{
	struct msm_fb_data_type *mfd = NULL;

	int data_pos;
	int cmd_step;
	int cmd_pos;

	cmd_step = 0;
	cmd_pos = 0;

	for (data_pos = 0; data_pos < len;) {
		if (*(src + data_pos) == '0') {
			if (*(src + data_pos + 1) == 'x') {
				if (!cmd_step) {
					mdni_tuning1[cmd_pos] =
					char_to_dec(*(src + data_pos + 2),
							*(src + data_pos + 3));
				} else {
					mdni_tuning2[cmd_pos] =
					char_to_dec(*(src + data_pos + 2),
							*(src + data_pos + 3));
				}

				data_pos += 3;
				cmd_pos++;

				if (cmd_pos == TUNE_FIRST_SIZE && !cmd_step) {
					cmd_pos = 0;
					cmd_step = 1;
				}
			} else
				data_pos++;
		} else {
			data_pos++;
		}
	}

/*
	printk(KERN_INFO "\n");
	for (data_pos = 0; data_pos < TUNE_FIRST_SIZE ; data_pos++)
		printk(KERN_INFO "0x%x ", mdni_tuning1[data_pos]);
	printk(KERN_INFO "\n");
	for (data_pos = 0; data_pos < TUNE_SECOND_SIZE ; data_pos++)
		printk(KERN_INFO"0x%x ", mdni_tuning2[data_pos]);
	printk(KERN_INFO "\n");
*/
	mfd = platform_get_drvdata(msd.msm_pdev);
	if (unlikely(!mfd))
		return;
	if (unlikely(mfd->key != MFD_KEY))
		return;

	mutex_lock(&dsi_tx_mutex);

	mipi_dsi_mdp_busy_wait();
	mipi_dsi_cmds_tx(&msd.samsung_tx_buf, mdni_tune_cmd,
						ARRAY_SIZE(mdni_tune_cmd));
	mutex_unlock(&dsi_tx_mutex);
}

static void load_tuning_file(char *filename)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret;
	mm_segment_t fs;

	pr_info("%s called loading file name : [%s]\n", __func__,
	       filename);

	fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		printk(KERN_ERR "%s File open failed\n", __func__);
		return;
	}

	l = filp->f_path.dentry->d_inode->i_size;
	pr_info("%s Loading File Size : %ld(bytes)", __func__, l);

	dp = kmalloc(l + 10, GFP_KERNEL);
	if (dp == NULL) {
		pr_info("Can't not alloc memory for tuning file load\n");
		filp_close(filp, current->files);
		return;
	}
	pos = 0;
	memset(dp, 0, l);

	pr_info("%s before vfs_read()\n", __func__);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	pr_info("%s after vfs_read()\n", __func__);

	if (ret != l) {
		pr_info("vfs_read() filed ret : %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return;
	}

	filp_close(filp, current->files);

	set_fs(fs);

	sending_tune_cmd(dp, l);

	kfree(dp);
}


static ssize_t tuning_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = snprintf(buf, MAX_FILE_NAME, "Tunned File Name : %s\n",
								tuning_file);

	return ret;
}

static ssize_t tuning_store(struct device *dev,
			    struct device_attribute *attr, const char *buf,
			    size_t size)
{
	char *pt;
	memset(tuning_file, 0, sizeof(tuning_file));
	snprintf(tuning_file, MAX_FILE_NAME, "%s%s", TUNING_FILE_PATH, buf);

	pt = tuning_file;
	while (*pt) {
		if (*pt == '\r' || *pt == '\n') {
			*pt = 0;
			break;
		}
		pt++;
	}

	pr_info("%s:%s\n", __func__, tuning_file);

	load_tuning_file(tuning_file);

	return size;
}

static DEVICE_ATTR(tuning, 0664, tuning_show, tuning_store);
#endif

#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
static irqreturn_t err_fg_irq_handler(int irq, void *handle)
{
	pr_info("%s  esd start", __func__);
	disable_irq_nosync(PM8921_GPIO_IRQ(PM8921_IRQ_BASE, PMIC_GPIO_ERR_FG));
	schedule_work(&err_fg_work);

	return IRQ_HANDLED;
}

static void err_fg_work_func(struct work_struct *work)
{
	err_fg_working = 1;
	esd_recovery();
	err_fg_working = 0;
	pr_info("%s esd end", __func__);
	return;
}
#endif
static int __devinit mipi_samsung_disp_probe(struct platform_device *pdev)
{
	int ret;
	struct platform_device *msm_fb_added_dev;
	struct lcd_device *lcd_device;
	struct backlight_device *bd = NULL;

	printk(KERN_INFO "[lcd] mipi_samsung_disp_probe start\n");

	if (pdev->id == 0) {
		msd.mipi_samsung_disp_pdata = pdev->dev.platform_data;

		printk(KERN_INFO
		"[lcd] pdev->id =%d,  pdev-name = %s\n", pdev->id, pdev->name);

		printk(KERN_INFO "[lcd] mipi_samsung_disp_probe end since pdev-id is 0\n");

		return 0;
	}

	printk(KERN_INFO "[lcd] msm_fb_add_device : %s\n", pdev->name);

	msm_fb_added_dev = msm_fb_add_device(pdev);

	mutex_init(&dsi_tx_mutex);

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_LCD_CLASS_DEVICE)
	msd.msm_pdev = msm_fb_added_dev;
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
	msd.early_suspend.suspend = mipi_samsung_disp_early_suspend;
	msd.early_suspend.resume = mipi_samsung_disp_late_resume;
	msd.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&msd.early_suspend);
#endif

#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	INIT_WORK(&err_fg_work, err_fg_work_func);

	err_fg_gpio = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_ERR_FG);

	ret = gpio_request(err_fg_gpio, "err_fg");

	if (ret) {
		pr_err("request gpio err_fg failed, rc=%d\n", ret);
		return -ENODEV;
	}

	ret = pm8xxx_gpio_config(err_fg_gpio, &gpio_get_param);

	if (ret) {
		pr_err("gpio_config err_fg_gpio failed (3), rc=%d\n", ret);
		return -EINVAL;
	}

	ret = request_threaded_irq(PM8921_GPIO_IRQ(PM8921_IRQ_BASE, PMIC_GPIO_ERR_FG), 
		NULL, err_fg_irq_handler,  IRQF_TRIGGER_RISING | IRQF_ONESHOT, "esd_detect", NULL);
	if (ret) {
		pr_err("%s : Failed to request_irq.:ret=%d", __func__, ret);
	}

	disable_irq(PM8921_GPIO_IRQ(PM8921_IRQ_BASE, PMIC_GPIO_ERR_FG));
#endif

#if defined(CONFIG_LCD_CLASS_DEVICE)
	printk(KERN_INFO "[lcd] lcd_device_register for panel start\n");

	lcd_device = lcd_device_register("panel", &pdev->dev, NULL,
			&mipi_samsung_disp_props);

	if (IS_ERR(lcd_device)) {
		ret = PTR_ERR(lcd_device);
		printk(KERN_ERR "lcd : failed to register device\n");
		return ret;
	}

	sysfs_remove_file(&lcd_device->dev.kobj,
			&dev_attr_lcd_power.attr);

	ret = sysfs_create_file(&lcd_device->dev.kobj,
			&dev_attr_lcd_power.attr);
	if (ret) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_lcd_power.attr.name);
	}

	ret = sysfs_create_file(&lcd_device->dev.kobj,
			&dev_attr_lcd_type.attr);
	if (ret) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_lcd_type.attr.name);
	}

	ret = sysfs_create_file(&lcd_device->dev.kobj,
					&dev_attr_backlight.attr);
	if (ret) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_backlight.attr.name);
	}

#if defined(RUMTIME_MIPI_CLK_CHANGE)
	ret = sysfs_create_file(&lcd_device->dev.kobj,
			&dev_attr_fps_change.attr);
	if (ret) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_fps_change.attr.name);
	}
#endif

	ret = sysfs_create_file(&lcd_device->dev.kobj,
					&dev_attr_siop_enable.attr);
	if (ret) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_siop_enable.attr.name);
	}

	printk(KERN_INFO "[lcd] backlight_device_register for panel start\n");

	bd = backlight_device_register("panel", &lcd_device->dev,
			NULL, NULL, NULL);
	if (IS_ERR(bd)) {
		ret = PTR_ERR(bd);
		pr_info("backlight : failed to register device\n");
		return ret;
	}

	ret = sysfs_create_file(&bd->dev.kobj,
			&dev_attr_auto_brightness.attr);
	if (ret) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_auto_brightness.attr.name);
	}
#endif

#if defined(CONFIG_FB_MDP4_ENHANCE)
	init_mdnie_class();
#elif defined(CONFIG_MDNIE_LITE_TUNING)
	pr_info("[%s] CONFIG_MDNIE_LITE_TUNING ok ! init class called!\n",
		__func__);
	mdnie_lite_tuning_init();
#endif

#if defined(DDI_VIDEO_ENHANCE_TUNING)
	ret = sysfs_create_file(&lcd_device->dev.kobj,
			&dev_attr_tuning.attr);
	if (ret) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_tuning.attr.name);
	}
#endif

	printk(KERN_INFO "[lcd] mipi_samsung_disp_probe end\n");

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_samsung_disp_probe,
	.driver = {
		.name   = "mipi_samsung_full_hd",
	},
	.shutdown = mipi_samsung_disp_shutdown
};

static struct msm_fb_panel_data samsung_panel_data = {
	.on		= mipi_samsung_disp_on,
	.off		= mipi_samsung_disp_off,
	.set_backlight	= mipi_samsung_disp_backlight,
};

static int ch_used[3];

int mipi_samsung_tft_device_register(struct msm_panel_info *pinfo,
		u32 channel, u32 panel,
		struct mipi_panel_data *mpd)
{
	struct platform_device *pdev = NULL;
	int ret = 0;

	printk(KERN_INFO "[lcd] mipi_samsung_device_register start\n");

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_samsung_full_hd",
			(panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	samsung_panel_data.panel_info = *pinfo;
	msd.mpd = mpd;
	if (!msd.mpd) {
		printk(KERN_ERR
			"%s: get mipi_panel_data failed!\n", __func__);
		goto err_device_put;
	}
	mpd->msd = &msd;
	ret = platform_device_add_data(pdev, &samsung_panel_data,
			sizeof(samsung_panel_data));
	if (ret) {
		printk(KERN_ERR
			"%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
			"%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	printk(KERN_INFO "[lcd] mipi_samsung_device_register end\n");

	return ret;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

int get_lcd_attached(void)
{
	return lcd_attached;
}
EXPORT_SYMBOL(get_lcd_attached);

static int __init lcd_attached_status(char *mode)
{
	/*
	*	1 is lcd attached
	*	0 is lcd detached
	*/

	if (strncmp(mode, "1", 1) == 0)
		lcd_attached = 1;
	else
		lcd_attached = 0;

	pr_info("%s %s", __func__, lcd_attached == 1 ?
				"lcd_attached" : "lcd_detached");
	return 1;
}
__setup("lcd_attached=", lcd_attached_status);

static int __init mipi_samsung_disp_init(void)
{
	printk(KERN_INFO "[lcd] mipi_samsung_disp_init start\n");

	mipi_dsi_buf_alloc(&msd.samsung_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&msd.samsung_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}
module_init(mipi_samsung_disp_init);
