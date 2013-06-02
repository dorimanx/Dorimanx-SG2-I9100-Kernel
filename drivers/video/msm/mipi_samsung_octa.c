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
#include "mipi_samsung_octa.h"
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

static int pm_gpio8;	/* ERR_FG */
static int pm_gpio5;	/* LDI_CHIP_SELECT */
#define PMIC_GPIO_ERR_FG 8
#define PMIC_GPIO_LDI 5
struct pm_gpio gpio_get_param = {
	.direction	= PM_GPIO_DIR_IN,
	.pull		= PM_GPIO_PULL_NO,
	.vin_sel		= 2,
	.function	= PM_GPIO_FUNC_NORMAL,
	.inv_int_pol	= 0,
};

static struct mipi_samsung_driver_data msd;
static int lcd_attached = 1;
struct mutex dsi_tx_mutex;
int touch_display_status;
DEFINE_MUTEX(brightness_mutex);

#if defined(RUMTIME_MIPI_CLK_CHANGE)
static int current_fps;
#endif

static int ldi_manupacture;
static int ldi_chip(void)
{
	int rc;
	
	gpio_get_param.pull = PM_GPIO_PULL_UP_1P5_30;

	rc = pm8xxx_gpio_config(pm_gpio5, &gpio_get_param);

	if (rc) {
		pr_err("gpio_config pm_gpio5 failed, rc=%d\n", rc);
	}

	ldi_manupacture = gpio_get_value(pm_gpio5);

	return ldi_manupacture;
}

int get_ldi_chip(void)
{
	return ldi_manupacture;
}

static int mipi_samsung_disp_send_cmd(struct msm_fb_data_type *mfd,
		enum mipi_samsung_cmd_list cmd,
		unsigned char lock)
{
	struct dsi_cmd_desc *cmd_desc;
	struct dcs_cmd_req cmdreq;
	int cmd_size = 0;
#ifdef CMD_DEBUG
	int i,j;
#endif

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
		case PANEL_ACL_CONTROL:
			cmd_desc = msd.mpd->acl_cmds.cmd;
			cmd_size = msd.mpd->acl_cmds.size;
			break;
		case PANLE_TOUCH_KEY:
			cmd_desc = msd.mpd->touch_key.cmd;
			cmd_size = msd.mpd->touch_key.size;
			break;
		case PANEL_LPTS:
			cmd_desc = msd.mpd->lpts.cmd;
			cmd_size = msd.mpd->lpts.size;
			break;
		default:
			pr_info("%s UNKNOW CMD", __func__);
			goto unknown_command;
			;
	}
		
#ifdef CMD_DEBUG
	if (cmd == PANEL_BRIGHT_CTRL || cmd == PANEL_ACL_CONTROL) {
		pr_info("+++ cmd_size = %d\n",cmd_size);
		for(i=0; i<cmd_size; i++){
			printk("cmd[%d] : ",i);
			for(j=0; j<cmd_desc[i].dlen; j++)
				printk("%x ",cmd_desc[i].payload[j]);
			printk("\n");
		}
		pr_info("--- end\n");
	}
#endif

	if (!cmd_size)
		goto unknown_command;

	cmdreq.cmds = cmd_desc;
	cmdreq.cmds_cnt = cmd_size;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);

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

static char manufacture_id1[2] = {0xD7, 0x00}; /* DTYPE_DCS_READ */

static struct dsi_cmd_desc samsung_manufacture_id1_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id1), manufacture_id1};

static uint32 mipi_samsung_manufacture_id(struct msm_fb_data_type *mfd)
{
	struct dsi_buf *rp, *tp;
	struct dsi_cmd_desc *cmd;
	uint32 id_size;
	char id[3];

	mutex_lock(&dsi_tx_mutex);

	id_size = 3;
	tp = &msd.samsung_tx_buf;
	rp = &msd.samsung_rx_buf;
	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);

	cmd = &samsung_manufacture_id1_cmd;
	mipi_dsi_cmds_rx(mfd, tp, rp, cmd, id_size);

	id[0] = *rp->data;
	id[1] = *(rp->data + 1);
	id[2] = *(rp->data + 2);

	mutex_unlock(&dsi_tx_mutex);

	pr_info("%s id0:  0x%x, id1: 0x%x, id2: 0x%x", __func__,
		id[0], id[1], id[2]);

	return id[0] << 16 | id[1] << 8 | id[2] ;
}

#if 0
static void read_mtp(char srcReg, int srcLength, char *destBuffer,
				struct msm_fb_data_type *pMFD)
{
	struct dcs_cmd_req cmdreq;
	int one_read_size = 6;
	int loop_limit = (srcLength / one_read_size) + 1;

	/* first byte = read-register */
	char read_reg[2] = { 0xFF, 0x00 };
	struct dsi_cmd_desc read_reg_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(read_reg), read_reg };

	/* first byte is size of Register */
	char packet_size[] = { 0x00, 0x00 };
	struct dsi_cmd_desc packet_size_cmd = {
	DTYPE_MAX_PKTSIZE, 1, 0, 0, 0, sizeof(packet_size), packet_size };

	/* second byte is Read-position */
	char reg_read_pos[] = { 0xB0, 0x00 };
	struct dsi_cmd_desc read_pos_cmd = {
	DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(reg_read_pos), reg_read_pos };

	int read_pos;
	int readed_size;
	int show_cnt;

	int read_size, read_loop;
	char show_buffer[256] = {0,};
	int show_buffer_pos = 0;

	mutex_lock(&dsi_tx_mutex);

	read_reg[0] = srcReg;

	show_buffer_pos +=
	    snprintf(show_buffer, 256, "read_reg : %X[%d] : ",
		 srcReg, srcLength);

	read_pos = 0;
	readed_size = 0;

	packet_size[0] = (char)srcLength;
	mipi_dsi_buf_init(&msd.samsung_tx_buf);

	cmdreq.cmds = &packet_size_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);

	show_cnt = 0;
	for (read_loop = 0; read_loop < loop_limit; read_loop++) {
		reg_read_pos[1] = read_pos;

		mipi_dsi_buf_init(&msd.samsung_tx_buf);

		cmdreq.cmds = &read_pos_cmd;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		if (mipi_dsi_cmdlist_put(&cmdreq) < 1) {
			show_buffer_pos +=
			    snprintf(show_buffer + show_buffer_pos, 256,
				    "Tx command FAILED");
			break;
		}

		mipi_dsi_buf_init(&msd.samsung_tx_buf);
		mipi_dsi_buf_init(&msd.samsung_rx_buf);
		readed_size =
		    mipi_dsi_cmds_rx(pMFD, &msd.samsung_tx_buf,
				     &msd.samsung_rx_buf, &read_reg_cmd,
				     one_read_size);
		for (read_size = 0; read_size < readed_size;
						read_size++, show_cnt++) {
			show_buffer_pos +=
			 snprintf(show_buffer + show_buffer_pos, 256, "%02x ",
				    msd.samsung_rx_buf.data[read_size]);
			if (destBuffer != NULL && show_cnt < srcLength) {
				destBuffer[show_cnt] =
				    msd.samsung_rx_buf.data[read_size];
			}
		}

		show_buffer_pos += snprintf(show_buffer +
			show_buffer_pos, 256, ".");
		read_pos += readed_size;

		if (read_pos >= srcLength)
			break;
	}

	mutex_unlock(&dsi_tx_mutex);

	if (read_loop == loop_limit)
		show_buffer_pos +=
		    snprintf(show_buffer + show_buffer_pos, 256, "Overrun");

	pr_info("%s\n", show_buffer);
}
#else
static void read_mtp(char srcReg, int readFrom, int srcLength, char *destBuffer,
				struct msm_fb_data_type *pMFD)
{
	struct dcs_cmd_req cmdreq;
	int one_read_size = 6;
	int loop_limit = (srcLength / one_read_size) + 1;

	/* first byte = read-register */
	char read_reg[2] = { 0xFF, 0x00 };
	struct dsi_cmd_desc read_reg_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(read_reg), read_reg };

	/* first byte is size of Register */
	char packet_size[] = { 0x00, 0x00 };
	struct dsi_cmd_desc packet_size_cmd = {
	DTYPE_MAX_PKTSIZE, 1, 0, 0, 0, sizeof(packet_size), packet_size };

	/* second byte is Read-position */
	char reg_read_pos[] = { 0xB0, 0x00 };
	struct dsi_cmd_desc read_pos_cmd = {
	DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(reg_read_pos), reg_read_pos };

	int read_pos;
	int readed_size;
	int show_cnt;

	int read_size, read_loop;
	char show_buffer[256] = {0,};
	int show_buffer_pos = 0;

	mutex_lock(&dsi_tx_mutex);

	read_reg[0] = srcReg;

	show_buffer_pos +=
	    snprintf(show_buffer, 256, "read_reg : %X[%d] : ",
		 srcReg, srcLength);

	read_pos = readFrom;
	readed_size = 0;

	packet_size[0] = (char)srcLength;
	mipi_dsi_buf_init(&msd.samsung_tx_buf);

	cmdreq.cmds = &packet_size_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);

	show_cnt = 0;
	for (read_loop = 0; read_loop < loop_limit; read_loop++) {
		reg_read_pos[1] = read_pos;

		mipi_dsi_buf_init(&msd.samsung_tx_buf);

		cmdreq.cmds = &read_pos_cmd;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		if (mipi_dsi_cmdlist_put(&cmdreq) < 1) {
			show_buffer_pos +=
			    snprintf(show_buffer + show_buffer_pos, 256,
				    "Tx command FAILED");
			break;
		}

		mipi_dsi_buf_init(&msd.samsung_tx_buf);
		mipi_dsi_buf_init(&msd.samsung_rx_buf);
		readed_size =
		    mipi_dsi_cmds_rx(pMFD, &msd.samsung_tx_buf,
				     &msd.samsung_rx_buf, &read_reg_cmd,
				     one_read_size);
		for (read_size = 0; read_size < readed_size;
						read_size++, show_cnt++) {
			show_buffer_pos +=
			 snprintf(show_buffer + show_buffer_pos, 256, "%02x ",
				    msd.samsung_rx_buf.data[read_size]);
			if (destBuffer != NULL && show_cnt < srcLength) {
				pr_debug("show_cnt(%d),srcLength(%d), msd.samsung_rx_buf.data[%d] = %x \n",
					show_cnt,srcLength,read_size,msd.samsung_rx_buf.data[read_size]);
				destBuffer[show_cnt] =
				    msd.samsung_rx_buf.data[read_size];
			}
		}

		show_buffer_pos += snprintf(show_buffer +
			show_buffer_pos, 256, ".");

		read_pos += readed_size;

		if ((read_pos - readFrom) >= srcLength)
			break;
	}

	mutex_unlock(&dsi_tx_mutex);

	if (read_loop == loop_limit)
		show_buffer_pos +=
		    snprintf(show_buffer + show_buffer_pos, 256, "Overrun");

	pr_info("%s\n", show_buffer);
}
#endif

static void execute_panel_init(struct msm_fb_data_type *mfd)
{
	int i;
	struct SMART_DIM *psmart = &(msd.mpd->smart_se6e8fa);
	char *mtp_buffer = (char *)&(msd.mpd->smart_se6e8fa.MTP_ORIGN);
	char *mtp_buffer1 = (char *)&(msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_1);
	/* LSI */
	char *mtp_buffer2 = (char *)&(msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_2);
	char *mtp_buffer3 = (char *)&(msd.mpd->smart_se6e8fa.hbm_reg.b6_reg_lsi);
	char temp[4];
	char C8_reg[3];
	/* MAGNA */
	char *mtp_buffer4 = (char *)&(msd.mpd->smart_se6e8fa.hbm_reg.b1_reg);
	char *mtp_buffer5 = (char *)&(msd.mpd->smart_se6e8fa.hbm_reg.b6_reg_magna);

	mipi_samsung_disp_send_cmd(mfd, PANEL_MTP_ENABLE, false);

	/* read LDi ID */
	msd.mpd->manufacture_id = mipi_samsung_manufacture_id(mfd);
	msd.mpd->cmd_set_change(PANEL_ON, msd.mpd->manufacture_id);

	/* smart dimming & AID*/
	psmart->plux_table = msd.mpd->lux_table;
	psmart->lux_table_max = msd.mpd->lux_table_max_cnt;
	psmart->ldi_revision = msd.mpd->manufacture_id;

	read_mtp(MTP_START_ADDR, 0, GAMMA_SET_MAX, mtp_buffer, mfd);

	if (get_ldi_chip() == LDI_LSI) {
		pr_info("#### LSI panel!!\n");
		if (msd.mpd->ldi_rev == 'H') {
			read_mtp(0xCB, 0, ARRAY_SIZE(C8_reg), C8_reg, mfd); // hover_flicker..
			msd.mpd->hover_flicker_change(C8_reg[2]);
		}
		read_mtp(MTP_START_ADDR, 34-1, 7, mtp_buffer1, mfd); // read C8h 34~40th
		if (msd.mpd->ldi_rev == 'I')
			read_mtp(MTP_START_ADDR, 73-1, 15, mtp_buffer2, mfd); // read C8h 73~87th
		else
			read_mtp(0xb6, 17-1, 1, mtp_buffer3, mfd); // recover original's ELVSS offset b6's 17th
	} else if (get_ldi_chip() == LDI_MAGNA) {
		pr_info("#### MAGNA panel!!\n");
		read_mtp(0xB1, 10-1, 6, mtp_buffer4, mfd); // read B1h 10~18th
		read_mtp(0xB6, 4-1, 23, mtp_buffer5, mfd); // read B6h 4~26th
	}

	i = 0;
#ifdef CONFIG_HBM_PSRE_DEBUG
	pr_info("%s c8[34~40](%x)(%x)(%x)(%x)(%x)(%x)(%x)", __func__,
		msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_1[0],
		msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_1[1], 
		msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_1[2],
		msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_1[3], 
		msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_1[4],
		msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_1[5],
		msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_1[6]);
	pr_info("\n");

	printk("c8_reg_1 : ");
	for(i=0; i<7; i++)
		printk("%x ",msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_1[i]);
	pr_info("\n");

	if (msd.mpd->ldi_rev == 'I') {
		printk("c8_reg_2 : ");
		for(i=0; i<15; i++)
			printk("%x ",msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_2[i]);
		pr_info("\n");
	} else
		printk("b6_reg : %x \n",msd.mpd->smart_se6e8fa.hbm_reg.b6_reg[0]);

#endif

#if defined(CONFIG_MDNIE_LITE_TUNING)
	read_mtp(LDI_COORDINATE_REG, 0,  ARRAY_SIZE(temp), temp, mfd);
	msd.mpd->coordinate[0] =  temp[0] << 8 | temp[1];	/* X */
	msd.mpd->coordinate[1] = temp[2] << 8 | temp[3];	/* Y */
	coordinate_tunning(msd.mpd->coordinate[0], msd.mpd->coordinate[1]);
#endif

	/* TO set default temperature value 25 degree*/
	msd.mpd->temperature_value = 25;

	mipi_samsung_disp_send_cmd(mfd, PANEL_MTP_DISABLE, false);
	smart_dimming_init(&(msd.mpd->smart_se6e8fa));

}

static int mipi_samsung_disp_on_in_video_engine(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);
	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	mipi_samsung_disp_send_cmd(mfd, PANEL_ON, false);
	mfd->resume_state = MIPI_RESUME_STATE;
	touch_display_status = MIPI_RESUME_STATE;

	if ((msd.mpd->manufacture_id & 0xFF) == 0)
		mipi_samsung_disp_send_cmd(mfd, PANEL_NEED_FLIP, false);

#if defined(RUMTIME_MIPI_CLK_CHANGE)
	current_fps = mfd->panel_info.mipi.frame_rate;
#endif

	/* To setting gpio_configuration under wake-up condition */
	if (get_ldi_chip() == LDI_LSI) {
		gpio_get_param.pull = PM_GPIO_PULL_NO;
		pm8xxx_gpio_config(pm_gpio8, &gpio_get_param); /* ERR_FG */
		gpio_get_param.pull = PM_GPIO_PULL_UP_1P5_30;
		pm8xxx_gpio_config(pm_gpio5, &gpio_get_param); /* LDI DET */
	} else {
		gpio_get_param.pull = PM_GPIO_PULL_NO;
		pm8xxx_gpio_config(pm_gpio8, &gpio_get_param);
		pm8xxx_gpio_config(pm_gpio5, &gpio_get_param);
	}

	pr_info("[%s] ID : 0x%x LDI : %s", __func__, msd.mpd->manufacture_id,
		ldi_manupacture == LDI_MAGNA ? "LDI_MAGNA" : "LDI_LSI");

	return 0;
}

static int mipi_samsung_disp_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	static int first_boot_on;

	mfd = platform_get_drvdata(pdev);
	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	mipi = &mfd->panel_info.mipi;

	if (!first_boot_on) {
		execute_panel_init(mfd);
		first_boot_on = 1;
	}

	if (get_auto_brightness() >= 6)
		msd.mpd->first_bl_hbm_psre = 1;

	sec_debug_mdp_reset_value();

	pr_info("[%s]\n", __func__);

	return 0;
}

int touch_tout1_on(void) 
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(msd.msm_pdev);

	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	if (mfd->panel_power_on == TRUE) { 
		mipi_samsung_disp_send_cmd(mfd, PANLE_TOUCH_KEY, true);
		pr_info("%s", __func__);
	}

	return 0;
}

static int mipi_samsung_disp_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);
	if (unlikely(!mfd))
		return -ENODEV;
	if (unlikely(mfd->key != MFD_KEY))
		return -EINVAL;

	mfd->resume_state = MIPI_SUSPEND_STATE;
	mipi_samsung_disp_send_cmd(mfd, PANEL_OFF, false);
	touch_display_status = MIPI_SUSPEND_STATE;

#if defined(RUMTIME_MIPI_CLK_CHANGE)
	if (mfd->panel_info.mipi.frame_rate != current_fps)
		mipi_runtime_clk_change(mfd->panel_info.mipi.frame_rate);
#endif

	if (msd.mpd->reset_bl_level != NULL)
		msd.mpd->reset_bl_level();

	/* To setting gpio_configuration under sleep condition */
	gpio_get_param.pull = PM_GPIO_PULL_DN;
	pm8xxx_gpio_config(pm_gpio5, &gpio_get_param);
	pm8xxx_gpio_config(pm_gpio8, &gpio_get_param);

	pr_info("[lcd] %s\n", __func__);

	return 0;
}

static void __devinit mipi_samsung_disp_shutdown(struct platform_device *pdev)
{
	static struct mipi_dsi_platform_data *mipi_dsi_pdata;
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (pdev->id != 0)
		return;

	mipi_dsi_pdata = pdev->dev.platform_data;

	if (mipi_dsi_pdata == NULL) {
		pr_err("LCD Power off failure: No Platform Data\n");
		return;
	}

	if (mfd)
		mipi_samsung_disp_send_cmd(mfd, PANEL_OFF, false);
}


static void mipi_samsung_disp_backlight(struct msm_fb_data_type *mfd)
{
	mutex_lock(&brightness_mutex);

	if (mfd->resume_state == MIPI_RESUME_STATE) {		
		if (msd.mpd->backlight_control(mfd->bl_level)) {
			mipi_samsung_disp_send_cmd(mfd, PANEL_BRIGHT_CTRL, true);
			pr_info("mipi_samsung_disp_backlight %d\n", mfd->bl_level);
		}
		msd.mpd->first_bl_hbm_psre = 0;
	} else {
		msd.mpd->first_bl_hbm_psre = 0;
		pr_info("%s : panel is off state!!\n", __func__);
	}
	
	mutex_unlock(&brightness_mutex);
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void mipi_samsung_disp_early_suspend(struct early_suspend *h)
{
	struct msm_fb_data_type *mfd;
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
	struct msm_fb_data_type *mfd;

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
	struct msm_fb_data_type *mfd;
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
	struct msm_fb_data_type *mfd;
	unsigned int power;

	mfd = platform_get_drvdata(msd.msm_pdev);

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

static ssize_t mipi_samsung_disp_lcdtype_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char temp[20];

	snprintf(temp, strnlen(msd.mpd->panel_name, 20) + 1,
			msd.mpd->panel_name);
	strlcat(buf, temp, 20);
	return strnlen(buf, 20);
}

static ssize_t mipi_samsung_disp_windowtype_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char temp[15];
	int id1, id2, id3;
	id1 = (msd.mpd->manufacture_id & 0x00FF0000) >> 16;
	id2 = (msd.mpd->manufacture_id & 0x0000FF00) >> 8;
	id3 = msd.mpd->manufacture_id & 0xFF;

	snprintf(temp, sizeof(temp), "%x %x %x\n",	id1, id2, id3);
	strlcat(buf, temp, 15);
	return strnlen(buf, 15);
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

unsigned char get_auto_brightness(void)
{
	return  msd.dstat.auto_brightness;
}
char* get_c8_reg_1(void)
{
	return msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_1;
}
char* get_c8_reg_2(void)
{
	return msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_2;
}
char get_elvss_400cd(void)
{
	return msd.mpd->smart_se6e8fa.hbm_reg.c8_reg_1[6];
}		
char get_elvss_offset(void)
{
	return msd.mpd->smart_se6e8fa.hbm_reg.b6_reg_lsi[0];
}
char* get_b1_reg(void)
{
	return msd.mpd->smart_se6e8fa.hbm_reg.b1_reg;
}
char* get_b6_reg_magna(void)
{
	return msd.mpd->smart_se6e8fa.hbm_reg.b6_reg_magna;
}

static ssize_t mipi_samsung_auto_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	static int first_auto_br;
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(msd.msm_pdev);

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
	else if (sysfs_streq(buf, "6")) // HBM mode (HBM + PSRE) will be changed to // leve 6 : no HBM , RE
		msd.dstat.auto_brightness = 6;
	else if (sysfs_streq(buf, "7")) // HBM mode (HBM + PSRE)
		msd.dstat.auto_brightness = 7;
	else
		pr_info("%s: Invalid argument!!", __func__);

	if (!first_auto_br) {
		pr_info("%s : skip first auto brightness store (%d) (%d)!!\n", 
				__func__, msd.dstat.auto_brightness, mfd->bl_level);
		first_auto_br++;
		return size;
	}

	if (mfd->resume_state == MIPI_RESUME_STATE) {
		msd.mpd->first_bl_hbm_psre = 1;
		mipi_samsung_disp_backlight(mfd);
		pr_info("%s : %d\n",__func__,msd.dstat.auto_brightness);
	} else {
		msd.mpd->first_bl_hbm_psre = 0;
		pr_info("%s : panel is off state!!\n", __func__);
	}

	return size;
}


static struct lcd_ops mipi_samsung_disp_props = {
	.get_power = NULL,
	.set_power = NULL,
};

static ssize_t mipi_samsung_disp_acl_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf((char *)buf, sizeof(buf), "%d\n", msd.mpd->acl_status);
	pr_info("acl status: %d\n", *buf);

	return rc;
}

static ssize_t mipi_samsung_disp_acl_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(msd.msm_pdev);

	if (sysfs_streq(buf, "1"))
		msd.mpd->acl_status = true;
	else if (sysfs_streq(buf, "0"))
		msd.mpd->acl_status = false;
	else {
		pr_info("%s: Invalid argument!!", __func__);
		return size;
	}

	if (mfd->panel_power_on) {
		if (msd.mpd->acl_control(mfd->bl_level))
			mipi_samsung_disp_send_cmd(mfd,
						PANEL_ACL_CONTROL, true);
	} else
		pr_info("%s : panel is off state. updating state value.\n", __func__);

	pr_info("%s : acl_status (%d) siop_status (%d)",
			__func__, msd.mpd->acl_status, msd.mpd->siop_status);

	return size;
}

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
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(msd.msm_pdev);

	if (sysfs_streq(buf, "1"))
		msd.mpd->siop_status = true;
	else if (sysfs_streq(buf, "0"))
		msd.mpd->siop_status = false;
	else {
		pr_info("%s: Invalid argument!!", __func__);
		return size;
	}

	if (mfd->panel_power_on) {
		if (msd.mpd->acl_control(mfd->bl_level))
			mipi_samsung_disp_send_cmd(mfd,
						PANEL_ACL_CONTROL, true);
	} else
		pr_info("%s : panel is off state. updating state value.\n", __func__);

	pr_info("%s : acl_status (%d) siop_status (%d)",
			__func__, msd.mpd->acl_status, msd.mpd->siop_status);

	return size;
}

#if defined(RUMTIME_MIPI_CLK_CHANGE)
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
	struct msm_fb_data_type *mfd;
	int goal_fps;
	int level = atoi(buf);

	mfd = platform_get_drvdata(msd.msm_pdev);

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
		if (get_ldi_chip() == LDI_LSI) {
			mipi_runtime_clk_change(current_fps);
			mutex_unlock(&dsi_tx_mutex);
		} else
			mutex_unlock(&dsi_tx_mutex);
	}

	pr_info("%s goal_fps : %d", __func__, goal_fps);

	return size;
}

#endif

static ssize_t mipi_samsung_disp_backlight_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int rc;
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(msd.msm_pdev);

	rc = snprintf((char *)buf, sizeof(buf), "%d\n", mfd->bl_level);

	return rc;
}

static ssize_t mipi_samsung_disp_backlight_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct msm_fb_data_type *mfd;
	int level = atoi(buf);

	mfd = platform_get_drvdata(msd.msm_pdev);

	mfd->bl_level = level;
	
	if (mfd->resume_state == MIPI_RESUME_STATE) {
		mipi_samsung_disp_backlight(mfd);
		pr_info("%s : level (%d)\n",__func__,level);
	} else {
		pr_info("%s : panel is off state!!\n", __func__);
	}

	return size;
}

static ssize_t mipi_samsung_temperature_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc;
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(msd.msm_pdev);

	rc = snprintf((char *)buf, 40,"-20, -19, 0, 1, 30, 40\n");

	pr_info("%s msd.mpd->temperature : %d msd.mpd->temperature_value : 0x%x", __func__,
				msd.mpd->temperature, msd.mpd->temperature_value);

	return rc;
}

static ssize_t mipi_samsung_temperature_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct msm_fb_data_type *mfd;
	int temp;

	mfd = platform_get_drvdata(msd.msm_pdev);

	sscanf(buf, "%d" , &msd.mpd->temperature);

	temp = msd.mpd->temperature;

	if (temp > 0)
		msd.mpd->temperature_value = (char)temp;
	else {
		temp *= -1;
		msd.mpd->temperature_value = (char)temp;
		msd.mpd->temperature_value |=0x80;
	}

	msd.mpd->need_update = 1;

	if (mfd->resume_state == MIPI_RESUME_STATE) {
		if (msd.mpd->backlight_control(mfd->bl_level)) {
			mipi_samsung_disp_send_cmd(mfd, PANEL_BRIGHT_CTRL, true);
			pr_info("mipi_samsung_temperature_store %d\n", mfd->bl_level);
		}
		
		pr_info("%s msd.mpd->temperature : %d msd.mpd->temperature_value : 0x%x", __func__,
						msd.mpd->temperature, msd.mpd->temperature_value);
	} else {
		pr_info("%s : panel is off state!!\n", __func__);
	}

	return size;
}

static DEVICE_ATTR(lcd_power, S_IRUGO | S_IWUSR,
		mipi_samsung_disp_get_power,
		mipi_samsung_disp_set_power);
static DEVICE_ATTR(lcd_type, S_IRUGO, mipi_samsung_disp_lcdtype_show, NULL);
static DEVICE_ATTR(window_type, S_IRUGO,
			mipi_samsung_disp_windowtype_show, NULL);

static DEVICE_ATTR(auto_brightness, S_IRUGO | S_IWUSR | S_IWGRP,
		mipi_samsung_auto_brightness_show,
		mipi_samsung_auto_brightness_store);
static DEVICE_ATTR(power_reduce, S_IRUGO | S_IWUSR | S_IWGRP,
			mipi_samsung_disp_acl_show,
			mipi_samsung_disp_acl_store);

static DEVICE_ATTR(siop_enable, S_IRUGO | S_IWUSR | S_IWGRP,
			mipi_samsung_disp_siop_show,
			mipi_samsung_disp_siop_store);

static DEVICE_ATTR(backlight, S_IRUGO | S_IWUSR | S_IWGRP,
			mipi_samsung_disp_backlight_show,
			mipi_samsung_disp_backlight_store);

#if defined(RUMTIME_MIPI_CLK_CHANGE)
static DEVICE_ATTR(fps_change, S_IRUGO | S_IWUSR | S_IWGRP,
			mipi_samsung_fps_show,
			mipi_samsung_fps_store);
#endif

static DEVICE_ATTR(temperature, S_IRUGO | S_IWUSR | S_IWGRP,
			mipi_samsung_temperature_show,
			mipi_samsung_temperature_store);

#ifdef DDI_VIDEO_ENHANCE_TUNING
#define MAX_FILE_NAME 128
#define TUNING_FILE_PATH "/sdcard/"
#define TUNE_FIRST_SIZE 5
#define TUNE_SECOND_SIZE 108
static char tuning_file[MAX_FILE_NAME];

static char mdni_tuning1[TUNE_FIRST_SIZE];
static char mdni_tuning2[TUNE_SECOND_SIZE];

static struct dsi_cmd_desc mdni_tune_cmd[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(mdni_tuning2), mdni_tuning2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(mdni_tuning1), mdni_tuning1},
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
	struct msm_fb_data_type *mfd;
	struct dcs_cmd_req cmdreq;

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

	printk(KERN_INFO "\n");
	for (data_pos = 0; data_pos < TUNE_FIRST_SIZE ; data_pos++)
		printk(KERN_INFO "0x%x ", mdni_tuning1[data_pos]);
	printk(KERN_INFO "\n");
	for (data_pos = 0; data_pos < TUNE_SECOND_SIZE ; data_pos++)
		printk(KERN_INFO"0x%x ", mdni_tuning2[data_pos]);
	printk(KERN_INFO "\n");

	mfd = platform_get_drvdata(msd.msm_pdev);

	mutex_lock(&dsi_tx_mutex);

	cmdreq.cmds = mdni_tune_cmd;
	cmdreq.cmds_cnt = ARRAY_SIZE(mdni_tune_cmd);
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);

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

static int __devinit mipi_samsung_disp_probe(struct platform_device *pdev)
{
	int ret, rc;
	struct platform_device *msm_fb_added_dev;
	struct lcd_device *lcd_device;
	struct backlight_device *bd = NULL;

	printk(KERN_INFO "[lcd] mipi_samsung_disp_probe start\n");
	if (pdev->id == 0) {
		msd.mipi_samsung_disp_pdata = pdev->dev.platform_data;

		printk(KERN_INFO
		"[lcd] pdev->id =%d,  pdev-name = %s\n", pdev->id, pdev->name);
		sec_debug_mdp_init();
		printk(KERN_INFO "[lcd] mipi_samsung_disp_probe end since pdev-id is 0\n");

		return 0;
	}

	printk(KERN_INFO "[lcd] msm_fb_add_device : %s\n", pdev->name);

	msm_fb_added_dev = msm_fb_add_device(pdev);

	mutex_init(&dsi_tx_mutex);

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_LCD_CLASS_DEVICE)
	msd.msm_pdev = msm_fb_added_dev;
#endif

	pm_gpio8 = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_ERR_FG);

	rc = gpio_request(pm_gpio8, "err_fg");

	if (rc) {
		pr_err("request gpio err_fg failed, rc=%d\n", rc);
		return -ENODEV;
	}

	rc = pm8xxx_gpio_config(pm_gpio8, &gpio_get_param);

	if (rc) {
		pr_err("gpio_config mlcd_rst failed (3), rc=%d\n", rc);
		return -EINVAL;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	msd.early_suspend.suspend = mipi_samsung_disp_early_suspend;
	msd.early_suspend.resume = mipi_samsung_disp_late_resume;
	msd.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&msd.early_suspend);
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
			&dev_attr_window_type.attr);
	if (ret) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_window_type.attr.name);
	}

	ret = sysfs_create_file(&lcd_device->dev.kobj,
					&dev_attr_power_reduce.attr);
	if (ret) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_power_reduce.attr.name);
	}

	ret = sysfs_create_file(&lcd_device->dev.kobj,
					&dev_attr_siop_enable.attr);
	if (ret) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_siop_enable.attr.name);
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
						&dev_attr_temperature.attr);
	if (ret) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_temperature.attr.name);
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

#if defined(CONFIG_MDNIE_LITE_TUNING)
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
	.late_init = mipi_samsung_disp_on_in_video_engine,
	.on		= mipi_samsung_disp_on,
	.off		= mipi_samsung_disp_off,
	.set_backlight	= mipi_samsung_disp_backlight,
};

static int ch_used[3];

int mipi_samsung_octa_device_register(struct msm_panel_info *pinfo,
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
	int rc;
	printk(KERN_INFO "[lcd] mipi_samsung_disp_init start\n");

#ifdef CONFIG_SUPPORT_DISPLAY_OCTA_TFT
	if (check_display_type() != SAMSUNG_OCTA_FULL_HD) {
		pr_err("[lcd] error : this rev(%d) is not for OCTA, return!!\n",
			system_rev);
		return 1;
	}
#endif

	pm_gpio5 = PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_LDI);

	rc = gpio_request(pm_gpio5, "LDI_SELECT");

	if (rc) {
		pr_err("request gpio LDI_SELECT failed, rc=%d\n", rc);
	}

	ldi_chip();

	mipi_dsi_buf_alloc(&msd.samsung_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&msd.samsung_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}
module_init(mipi_samsung_disp_init);
