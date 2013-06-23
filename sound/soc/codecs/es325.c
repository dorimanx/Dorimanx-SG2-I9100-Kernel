/*
 * es325.c  --  Audience eS325 ALSA SoC Audio driver
 *
 * Copyright 2011 Audience, Inc.
 *
 * Author: Greg Clemson <gclemson@audience.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* #define DEBUG */
#define BUS_TRANSACTIONS
#define FIXED_CONFIG
#define ES325_SLEEP /* To support sleep/wakeup for ES325 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/slimbus/slimbus.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/kthread.h>
#ifdef ES325_SLEEP
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slimbus/slimbus.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#endif
#include <linux/vmalloc.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <generated/utsrelease.h>
#include <linux/i2c/esxxx.h> /* TODO: common location for i2c and slimbus */
#include "es325.h"
#include "es325-export.h"

#define ES325_CMD_ACCESS_WR_MAX 4
#define ES325_CMD_ACCESS_RD_MAX 4
struct es325_cmd_access {
	u8 read_msg[ES325_CMD_ACCESS_RD_MAX];
	unsigned int read_msg_len;
	u8 write_msg[ES325_CMD_ACCESS_WR_MAX];
	unsigned int write_msg_len;
	unsigned int val_shift;
	unsigned int val_max;
};

#include "es325-access.h"

#define ES325_SLIM_CH_RX_OFFSET		152
#define ES325_SLIM_CH_TX_OFFSET		156
/* #define ES325_SLIM_RX_PORTS		10 */
#define ES325_SLIM_RX_PORTS		6
#define ES325_SLIM_TX_PORTS		6

#define ES325_SLIM_1_PB		1
#define ES325_SLIM_1_CAP	2
#define ES325_SLIM_2_PB		3
#define ES325_SLIM_2_CAP	4
#define ES325_SLIM_3_PB		5
#define ES325_SLIM_3_CAP	6

#define ES325_SLIM_1_PB_MAX_CHANS	2
#define ES325_SLIM_1_CAP_MAX_CHANS	2
#define ES325_SLIM_2_PB_MAX_CHANS	2
#define ES325_SLIM_2_CAP_MAX_CHANS	2
#define ES325_SLIM_3_PB_MAX_CHANS	2
#define ES325_SLIM_3_CAP_MAX_CHANS	2

#define ES325_SLIM_1_PB_OFFSET	0
#define ES325_SLIM_2_PB_OFFSET	2
#define ES325_SLIM_3_PB_OFFSET	4
#define ES325_SLIM_1_CAP_OFFSET	0
#define ES325_SLIM_2_CAP_OFFSET	2
#define ES325_SLIM_3_CAP_OFFSET	4

#define NARROW_BAND 0
#define WIDE_BAND 1
#define NETWORK_OFFSET 21
#define NS_OFFSET 2

#define ES325_NUM_CODEC_SLIM_DAIS	6

struct es325_slim_dai_data {
	unsigned int rate;
	unsigned int *ch_num;
	unsigned int ch_act;
	unsigned int ch_tot;
#ifdef ES325_SLEEP
	unsigned int ch_wakeup;
#endif
};

struct es325_slim_ch {
	u32	sph;
	u32	ch_num;
	u16	ch_h;
	u16	grph;
};

// add canada feature for adding WB firmware in ATT based kernel
#if defined(CONFIG_MACH_JF_TMO) || defined(CONFIG_MACH_JF_CAN)
#define FIRMWARE_NAME "audience-es325-fw-tmo.bin"
#elif defined(CONFIG_MACH_JF_VZW)
#define FIRMWARE_NAME "audience-es325-fw-vzw.bin"
#elif defined(CONFIG_MACH_JF_CMCC)
#define FIRMWARE_NAME "audience-es325-fw-cmcc.bin"
#elif defined(CONFIG_MACH_JF_EUR)
#define FIRMWARE_NAME "audience-es325-fw-eur.bin"
#elif defined(CONFIG_MACH_JACTIVE_EUR)
#define FIRMWARE_NAME "audience-es325-fw-jactive-eur.bin"
#elif defined(CONFIG_MACH_JACTIVE_ATT)
#define FIRMWARE_NAME "audience-es325-fw-jactive_att.bin"
#define FIRMWARE_NAME_REV02 "audience-es325-fw-jactive_att_rev02.bin"
#elif defined(CONFIG_MACH_JF_SKT) || defined(CONFIG_MACH_JF_KTT)\
	|| defined(CONFIG_MACH_JF_LGT)
#define FIRMWARE_NAME "audience-es325-fw-kor.bin"
#else
#define FIRMWARE_NAME "audience-es325-fw.bin"
#endif

extern unsigned int system_rev;

#define ES325_MAX_INVALID_VEQ 0xFFFF
#define ES325_MAX_INVALID_BWE 0xFFFF
#define ES325_MAX_INVALID_TX_NS 0xFFFF

static unsigned int es325_VEQ_enable = ES325_MAX_INVALID_VEQ;
static unsigned int es325_VEQ_enable_new = ES325_MAX_INVALID_VEQ;
static unsigned int es325_BWE_enable = ES325_MAX_INVALID_BWE;
static unsigned int es325_BWE_enable_new = ES325_MAX_INVALID_BWE;
static unsigned int es325_Tx_NS = ES325_MAX_INVALID_TX_NS;
static unsigned int es325_Tx_NS_new = ES325_MAX_INVALID_TX_NS;

/* codec private data */
struct es325_priv {
	struct snd_soc_codec *codec;
	struct firmware *fw;

	struct esxxx_platform_data *pdata;

	struct i2c_client *this_client;
	struct slim_device *intf_client;
	struct slim_device *gen0_client;
	struct es325_slim_dai_data dai[ES325_NUM_CODEC_SLIM_DAIS];
	struct es325_slim_ch slim_rx[ES325_SLIM_RX_PORTS];
	struct es325_slim_ch slim_tx[ES325_SLIM_TX_PORTS];
#ifdef ES325_SLEEP
	struct mutex pm_mutex; /* Mutex for protecting data structure */
	int wakeup_cnt; /* sleep and wakeup count, when reached to 0, start sleep timer */
	int clock_on; /* es325 clock status */
	int internal_route_config; /* last configured route config */
	int new_internal_route_config; /* new received route config via KControl, it is written when hw param is received */
	int wq_active; /* sleep timer status */
#endif
	bool rx_is_muted;
} es325_priv;

static unsigned int es325_ap_tx1_ch_cnt = 1;
unsigned int es325_tx1_route_enable;
unsigned int es325_rx1_route_enable;
unsigned int es325_rx2_route_enable;

static int es325_slim_rx_port_to_ch[ES325_SLIM_RX_PORTS] = {
	152, 153, 154, 155, 134, 135
};
static int es325_slim_tx_port_to_ch[ES325_SLIM_TX_PORTS] = {
	156, 157, 138, 139, 143, 144
};
static int es325_slim_be_id[ES325_NUM_CODEC_SLIM_DAIS] = {
	ES325_SLIM_2_CAP, /* for ES325_SLIM_1_PB tx from es325 */
	ES325_SLIM_3_PB, /* for ES325_SLIM_1_CAP rx to es325 */
	ES325_SLIM_3_CAP, /* for ES325_SLIM_2_PB tx from es325 */
	-1, /* for ES325_SLIM_2_CAP */
	-1, /* for ES325_SLIM_3_PB */
	-1, /* for ES325_SLIM_3_CAP */
};

#define ES325_INTERNAL_ROUTE_MAX 26
static long es325_internal_route_num;
static int es325_network_type = NARROW_BAND;

static bool es325_2mic_enable = false;

/* Audio playback, 2 channels */
static u8 es325_internal_route_audio_playback_2ch[10] = {
	0x90, 0x31, 0x00, 0x08, /* 2 Ch playback 4 Ch pass */
	0x90, 0x31, 0x00, 0x50, /* Algo Preset for 2 Ch playback 4 Ch pass */
	0xff		/* terminate */
};

/* 1-mic Headset NB (SW bypss)*/
static u8 es325_internal_route_1mic_headset[10] = {
	0x90, 0x31, 0x00, 0x01, /* 1 Mic 2 FEOUT w UITone MD */
	0x90, 0x31, 0x00, 0x84, /* Algo Preset: 1-mic CT NB */
	0xff		/* terminate */
};

/* 1-mic Headset WB (SW bypss)*/
static u8 es325_internal_route_1mic_headset_WB[10] = {
	0x90, 0x31, 0x00, 0x01, /* 1 Mic 2 FEOUT w UITone MD */
	0x90, 0x31, 0x00, 0x85, /* Algo Preset: 1-mic CT WB */
	0xff		/* terminate */
};

#if defined(CONFIG_MACH_JACTIVE_ATT)
/* 1-mic Speaker NB (1-mic FT)(NS off)(SW bypss) */
static u8 es325_internal_route_1mic_speaker[10] = {
	0x90, 0x31, 0x00, 0x0d, /* 1 Mic 1 FEOUT */
	0x90, 0x31, 0x00, 0x82, /* Algo Preset: 1-mic CT NB */
	0xff		/* terminate */
};
#else
/* 1-mic Speaker NB (1-mic FT)(NS off)(SW bypss) */
static u8 es325_internal_route_1mic_speaker[10] = {		
	0x90, 0x31, 0x00, 0x0c, /* 1 Mic 1 FEOUT */
	0x90, 0x31, 0x00, 0x82, /* Algo Preset: 1-mic CT NB */
	0xff		/* terminate */
};
#endif

/* 1-mic Speaker WB (1-mic FT)(NS off)(SW bypss) */
static u8 es325_internal_route_1mic_speaker_WB[10] = {		
	0x90, 0x31, 0x00, 0x0c, /* 1 Mic 1 FEOUT */
	0x90, 0x31, 0x00, 0x83, /* Algo Preset: 1-mic CT WB */
	0xff		/* terminate */
};

/* 2-mic Speaker NB (2-mic FT)(NS on) */
static u8 es325_internal_route_2mic_speaker[10] = {	
	0x90, 0x31, 0x00, 0x02, /* 2 Mic 1 FEOUT w UITone CT */
	0x90, 0x31, 0x00, 0x16, /* Algo Preset for 2 Mic FT NB */
	0xff		/* terminate */
};

/* 2-mic Speaker WB (2-mic FT)(NS off) */
static u8 es325_internal_route_2mic_speaker_WB[10] = {		
	0x90, 0x31, 0x00, 0x02, /* 2 Mic 1 FEOUT w UITone CT */
	0x90, 0x31, 0x00, 0x17, /* Algo Preset for 2 Mic FT WB */
	0xff		/* terminate */
};

#if defined(CONFIG_MACH_JF_SKT) || defined(CONFIG_MACH_JF_KTT)\
	|| defined(CONFIG_MACH_JF_LGT)
/* 1-mic Handset NB (1-mic CT)(NS off) */ 
static u8 es325_internal_route_1mic_handset[10] = {
	0x90, 0x31, 0x00, 0x04, /* 2 Mic 1 FEOUT w UITone CT */
	0x90, 0x31, 0x00, 0x18, /* Algo Preset for 2 Mic CT NB */
	0xff		/* terminate */
};
#else
/* 1-mic Handset NB (1-mic CT)(NS off) */ 
static u8 es325_internal_route_1mic_handset[10] = {
	0x90, 0x31, 0x00, 0x02, /* 2 Mic 1 FEOUT w UITone CT */
	0x90, 0x31, 0x00, 0x18, /* Algo Preset for 2 Mic CT NB */
	0xff		/* terminate */
};
#endif

#if defined(CONFIG_MACH_JF_SKT) || defined(CONFIG_MACH_JF_KTT)\
	|| defined(CONFIG_MACH_JF_LGT)
/* 1-mic Handset WB (1-mic CT)(NS off) */ 
static u8 es325_internal_route_1mic_handset_WB[10] = {
	0x90, 0x31, 0x00, 0x04, /* 2 Mic 1 FEOUT w UITone CT */
	0x90, 0x31, 0x00, 0x19, /* Algo Preset for 2 Mic CT WB */
	0xff		/* terminate */
};
#else
/* 1-mic Handset WB (1-mic CT)(NS off) */ 
static u8 es325_internal_route_1mic_handset_WB[10] = {
	0x90, 0x31, 0x00, 0x02, /* 2 Mic 1 FEOUT w UITone CT */
	0x90, 0x31, 0x00, 0x19, /* Algo Preset for 2 Mic CT WB */
	0xff		/* terminate */
};
#endif

/* 2-mic Handset NB (2-mic CT)(NS on) */
static u8 es325_internal_route_2mic_handset[10] = {
	0x90, 0x31, 0x00, 0x02, /* 2 Mic 1 FEOUT w UITone CT */
	0x90, 0x31, 0x00, 0x14, /* Algo Preset for 2 Mic CT NB */
	0xff		/* terminate */
};

/* 2-mic Handset WB (2-mic CT)(NS on) */
static u8 es325_internal_route_2mic_handset_WB[10] = {
	0x90, 0x31, 0x00, 0x02, /* 2 Mic 1 FEOUT w UITone CT */
	0x90, 0x31, 0x00, 0x15, /* Algo Preset for 2 Mic CT WB */
	0xff		/* terminate */
};

/* 1-mic 1-output for Loopback (SW bypss) */
static u8 es325_internal_route_1mic_1out_loopback[10] = {		
	0x90, 0x31, 0x00, 0x0d, /* 1 Mic 1 FEOUT */
	0x90, 0x31, 0x00, 0x82, /* Algo Preset: 1-mic CT NB */
	0xff		/* terminate */
};

/* 1-mic 2-output for Loopback (SW bypss) */
static u8 es325_internal_route_1mic_2out_loopback[10] = {		
	0x90, 0x31, 0x00, 0x01, /* 1 Mic 2 FEOUT w UITone MD */
	0x90, 0x31, 0x00, 0x84, /* Algo Preset: 1-mic CT NB */
	0xff		/* terminate */
};

/* 2-mic ASR */
static u8 es325_internal_route_2mic_ASR[10] = {		
	0x90, 0x31, 0x00, 0x6E, /* 2 Mic ASR WB */
	0x90, 0x31, 0x00, 0x70, /* Algo Preset for 2 Mic ASR WB */
	0xff        /* terminate */
};

/* TTY HCO - Rx:Handset(1ch), Tx: Main mic(1ch)(SW bypass)*/
static u8 es325_internal_route_TTY_HCO[10] = {	
	0x90, 0x31, 0x00, 0x0d, /* 1 Mic 1 FEOUT */
	0x90, 0x31, 0x00, 0x84, /* Algo Preset: 1-mic CT NB */
	0xff		/* terminate */
};

/* TTY VCO - Rx:Headset(2ch), Tx: Main mic(1ch)(SW bypass)*/
static u8 es325_internal_route_TTY_VCO[10] = {	
	0x90, 0x31, 0x00, 0x01, /* 1 Mic 2 FEOUT w UITone MD */
	0x90, 0x31, 0x00, 0x84, /* Algo Preset: 1-mic CT NB */
	0xff		/* terminate */
};

static u8 dummy[10] = {
	0x90, 0x31, 0x00, 0x07, /* 1 Ch playback 4 Ch pass */
	0x90, 0x31, 0x00, 0x46, /* Algo Preset for 1 Ch playback 4 Ch pass */
	0xff		/* terminate */
};

static u8* es325_internal_route_configs[ES325_INTERNAL_ROUTE_MAX] = {
	es325_internal_route_1mic_headset, 		/* [0]: 1-mic Headset NB (SW bypss) */
	es325_internal_route_2mic_speaker, 		/* [1]: 2-mic Speaker NB (2-mic FT)(NS on)*/
	es325_internal_route_2mic_handset, 		/* [2]: 2-mic Handset NB (2-mic CT)(NS on) */
	es325_internal_route_1mic_speaker, 		/* [3]: 1-mic Speaker NB (1-mic FT)(NS off)(SW bypss) */	
	es325_internal_route_1mic_handset, 		/* [4]: 1-mic Handset NB (1-mic CT)(NS off) */
	dummy, /* [5]: TBD */
	es325_internal_route_audio_playback_2ch,/* [6]: Audio playback, 2 channels */
	dummy, /* [7]: TBD */
	dummy, /* [8]: TBD */
	dummy, /* [9]: TBD */
	dummy, /* [10]: TBD */
	es325_internal_route_TTY_VCO, 			/* [11]: TTY VCO - Rx:Headset(2ch), Tx: Main mic(1ch)(SW bypass) */
	es325_internal_route_TTY_HCO, 			/* [12]: TTY HCO - Rx:Handset(1ch), Tx: Main mic(1ch)(SW bypass) */
	es325_internal_route_2mic_ASR, 			/* [13]: 2-mic ASR */
	dummy, /* [14]: TBD */
	dummy, /* [15]: TBD */
	dummy, /* [16]: TBD */
	dummy, /* [17]: TBD */
	es325_internal_route_1mic_1out_loopback,/* [18]: 1-mic 1-output for Loopback (SW bypss) */	
	es325_internal_route_1mic_2out_loopback,/* [19]: 1-mic 2-output for Loopback (SW bypss) */
	dummy, /* [20]: TBD */
	es325_internal_route_1mic_headset_WB, 	/* [21]: 1-mic Headset WB (SW bypss) */
	es325_internal_route_2mic_speaker_WB,	/* [22]: 2-mic Speaker WB (2-mic FT)(NS on) */
	es325_internal_route_2mic_handset_WB, 	/* [23]: 2-mic Handset WB (2-mic CT)(NS on) */
	es325_internal_route_1mic_speaker_WB, 	/* [24]: 1-mic Speaker WB (1-mic FT)(NS off)(SW bypss) */
	es325_internal_route_1mic_handset_WB,   /* [25]: 1-mic Handset WB (1-mic CT)(NS off) */
};

#ifdef NOT_USED_CONFIG
	/* [3]: 1-mic VOIP (CT) */
	{
		0x90, 0x31, 0x00, 0x03, /* 1 Mic 1 FEOUT */
		0x90, 0x31, 0x00, 0x1E, /* Algo Preset for 1 Mic VOIP DV SWB */
		0xff		/* terminate */
	},

	/* [4]: 2-mic VOIP (CT) */
	{
		0x90, 0x31, 0x00, 0x04, /* 2 Mic 1 FEOUT */
		0x90, 0x31, 0x00, 0x29, /* Algo Preset for 2 Mic VOIP CT WB */
		0xff		/* terminate */
	},

	/* [5]: Audio playback, 1 channel */
	{
		0x90, 0x31, 0x00, 0x07, /* 1 Ch playback 4 Ch pass */
		0x90, 0x31, 0x00, 0x46, /* Algo Preset for 1 Ch playback 4 Ch pass */
		0xff		/* terminate */
	},

	/* [7]: Audio record, 1 channel */
	{
		0x90, 0x31, 0x00, 0x09, /* 1 Ch Record 4 Ch pass */
		0x90, 0x31, 0x00, 0x5A, /* Algo Preset for 1 Ch Record 4 Ch pass */
		0xff		/* terminate */
	},

	/* [8]: Audio record, 2 channels */
	{
		0x90, 0x31, 0x00, 0x65, /* 2 Ch Record 4 Ch pass */
		0x90, 0x31, 0x00, 0x64, /* Algo Preset for 2 Ch Record 4 Ch pass */
		0xff		/* terminate */
	},

	/* [9]: 2-mic CT ASR */
	{
		0x90, 0x31, 0x00, 0x06, /* 2 Mic ASR */
		0x90, 0x31, 0x00, 0x3C, /* Algo Preset for 2 Mic ASR NB */
		0xff		/* terminate */
	},

	/* [10]: 1-mic External ASR */
	{
		0x90, 0x31, 0x00, 0x05, /* 1 Mic ASR */
		0x90, 0x31, 0x00, 0x32, /* Algo Preset for 1 Mic ASR NB */
		0xff		/* terminate */
	},
	
	/* [11]: VOIP Headset(CT) */
	{
		0x90, 0x31, 0x00, 0x01, /* 1 Mic Headset */
		0x90, 0x31, 0x00, 0x0b, /* Algo Preset for WB */
		0xff                    /* terminate */
	},

	/* [14]: 1-mic DV ASR WB */
	{
		0x90, 0x31, 0x00, 0x71, /* 1 Mic DV ASR WB */
		0x90, 0x31, 0x00, 0x73, /* Algo Preset for 1 Mic ASR DV WB */
		0xff                        /* terminate */
	},
	
	/* [15]: 1-mic FT */
	{
		0x90, 0x31, 0x00, 0x0c, /* 1 Mic FT */
		0x90, 0x31, 0x00, 0x16, /* Algo Preset for 2 Mic FT */
					/* TODO Switch to Algo Preset for 1 Mic FT after testing */
		0xff                        /* terminate */
	},

	/* [16]: 1-mic FT WB */
	{
		0x90, 0x31, 0x00, 0x0c, /* 1 Mic FT WB */
		0x90, 0x31, 0x00, 0x17, /* Algo Preset for 2 Mic FT WB */
					/* TODO Switch to Algo Preset for 1 Mic FT after testing */
		0xff                        /* terminate */
	},
#endif

static struct snd_soc_dai_driver es325_dai[];

#ifdef ES325_SLEEP
 void es325_wrapper_sleep_internal(struct work_struct *dummy);
#endif

static void es325_fixed_config(struct es325_priv *es325);
/* static void es325_ping(struct es325_priv *es325); */
static int es325_alloc_slim_rx_chan(struct slim_device *sbdev);
static int es325_alloc_slim_tx_chan(struct slim_device *sbdev);
static int es325_cfg_slim_rx(struct slim_device *sbdev, unsigned int *ch_num,
			     unsigned int ch_cnt, unsigned int rate, bool commit);
static int es325_cfg_slim_tx(struct slim_device *sbdev, unsigned int *ch_num,
			     unsigned int ch_cnt, unsigned int rate, bool commit);
static int es325_close_slim_rx(struct slim_device *sbdev, unsigned int *ch_num,
			       unsigned int ch_cnt);
static int es325_close_slim_tx(struct slim_device *sbdev, unsigned int *ch_num,
			       unsigned int ch_cnt);
static int es325_rx_ch_num_to_idx(int ch_num);
static int es325_tx_ch_num_to_idx(int ch_num);

static int es325_rx_ch_num_to_idx(int ch_num)
{
	int i;
	int idx = -1;

	for (i = 0; i < ES325_SLIM_RX_PORTS; i++) {
		if (ch_num == es325_slim_rx_port_to_ch[i]) {
			idx = i;
			break;
		}
	}

	return idx;
}

static int es325_tx_ch_num_to_idx(int ch_num)
{
	int i;
	int idx = -1;

	for (i = 0; i < ES325_SLIM_TX_PORTS; i++) {
		if (ch_num == es325_slim_tx_port_to_ch[i]) {
			idx = i;
			break;
		}
	}

	return idx;
}

/* es325 -> codec - alsa playback function */
static int es325_codec_cfg_slim_tx(struct es325_priv *es325, int dai_id)
{
	int rc = 0;

	/* start slim channels associated with id */
	rc = es325_cfg_slim_tx(es325->gen0_client,
			       es325->dai[dai_id - 1].ch_num,
			       es325->dai[dai_id - 1].ch_tot,
			       es325->dai[dai_id - 1].rate, true);

	return rc;
}

/* es325 <- codec - alsa capture function */
static int es325_codec_cfg_slim_rx(struct es325_priv *es325, int dai_id)
{
	int rc = 0;

	/* start slim channels associated with id */
	rc = es325_cfg_slim_rx(es325->gen0_client,
			       es325->dai[dai_id - 1].ch_num,
			       es325->dai[dai_id - 1].ch_tot,
			       es325->dai[dai_id - 1].rate, true);

	return rc;
}

/* es325 -> codec - alsa playback function */
static int es325_codec_close_slim_tx(struct es325_priv *es325, int dai_id)
{
	int rc = 0;

	/* close slim channels associated with id */
	rc = es325_close_slim_tx(es325->gen0_client,
				 es325->dai[dai_id - 1].ch_num,
				 es325->dai[dai_id - 1].ch_tot);

	return rc;
}

/* es325 <- codec - alsa capture function */
static int es325_codec_close_slim_rx(struct es325_priv *es325, int dai_id)
{
	int rc = 0;

	/* close slim channels associated with id */
	rc = es325_close_slim_rx(es325->gen0_client,
				 es325->dai[dai_id - 1].ch_num,
				 es325->dai[dai_id - 1].ch_tot);

	return rc;
}

static int es325_alloc_slim_rx_chan(struct slim_device *sbdev)
{
	struct es325_priv *es325_priv = slim_get_devicedata(sbdev);
	struct es325_slim_ch *rx = es325_priv->slim_rx;
	int i;
	int port_id;
	int rc = 0;

	dev_dbg(&sbdev->dev, "%s(): entry\n", __func__);

	for (i = 0; i < ES325_SLIM_RX_PORTS; i++) {
		port_id = i;
		rx[i].ch_num = es325_slim_rx_port_to_ch[i];
		slim_get_slaveport(sbdev->laddr, port_id, &rx[i].sph,
				   SLIM_SINK);
		slim_query_ch(sbdev, rx[i].ch_num, &rx[i].ch_h);
		dev_dbg(&sbdev->dev,
			"%s(): port_id = %d\n", __func__, port_id);
		dev_dbg(&sbdev->dev,
			"%s(): ch_num = %d\n", __func__, rx[i].ch_num);
		dev_dbg(&sbdev->dev,
			"%s(): sph = 0x%08x\n", __func__, rx[i].sph);
	}

	return rc;
}

static int es325_alloc_slim_tx_chan(struct slim_device *sbdev)
{
	struct es325_priv *es325_priv = slim_get_devicedata(sbdev);
	struct es325_slim_ch *tx = es325_priv->slim_tx;
	int i;
	int port_id;
	int rc = 0;

	dev_dbg(&sbdev->dev, "%s(): entry\n", __func__);

	for (i = 0; i < ES325_SLIM_TX_PORTS; i++) {
		port_id = i + 10; /* ES325_SLIM_RX_PORTS; */
		tx[i].ch_num = es325_slim_tx_port_to_ch[i];
		slim_get_slaveport(sbdev->laddr, port_id, &tx[i].sph,
				   SLIM_SRC);
		slim_query_ch(sbdev, tx[i].ch_num, &tx[i].ch_h);
		dev_dbg(&sbdev->dev,
			"%s(): port_id = %d\n", __func__, port_id);
		dev_dbg(&sbdev->dev,
			"%s(): ch_num = %d\n", __func__, tx[i].ch_num);
		dev_dbg(&sbdev->dev,
			"%s(): sph = 0x%08x\n", __func__, tx[i].sph);
	}

	return rc;
}

static int es325_cfg_slim_rx(struct slim_device *sbdev, unsigned int *ch_num,
			     unsigned int ch_cnt, unsigned int rate, bool commit)
{
	struct es325_priv *es325_priv = slim_get_devicedata(sbdev);
	struct es325_slim_ch *rx = es325_priv->slim_rx;
	u16 grph;
	u32 sph[ES325_SLIM_RX_PORTS] = {0};
	u16 ch_h[ES325_SLIM_RX_PORTS] = {0};
	struct slim_ch prop;
	int i;
	int idx = 0;
	int ret = 0;

	dev_dbg(&sbdev->dev, "+%s()==\n", __func__);
	dev_dbg(&sbdev->dev, "%s(): ch_cnt = %d\n", __func__, ch_cnt);
	dev_dbg(&sbdev->dev, "%s(): rate = %d\n", __func__, rate);

	for (i = 0; i < ch_cnt; i++) {
		idx = es325_rx_ch_num_to_idx(ch_num[i]);
		ch_h[i] = rx[idx].ch_h;
		sph[i] = rx[idx].sph;

		dev_dbg(&sbdev->dev, "%s(): idx = %d\n", __func__, idx);
		dev_dbg(&sbdev->dev, "%s(): ch_num[i] = %d\n", __func__, ch_num[i]);
		dev_dbg(&sbdev->dev, "%s(): ch_h[i] = %d\n", __func__, ch_h[i]);
		dev_dbg(&sbdev->dev, "%s(): sph[i] = 0x%08x\n", __func__, sph[i]);
	}

	prop.prot = SLIM_AUTO_ISO;
	prop.baser = SLIM_RATE_4000HZ;
	prop.dataf = SLIM_CH_DATAF_NOT_DEFINED;
	prop.auxf = SLIM_CH_AUXF_NOT_APPLICABLE;
	prop.ratem = (rate/4000);
	prop.sampleszbits = 16;

	ret = slim_define_ch(sbdev, &prop, ch_h, ch_cnt, true, &grph);
	if (ret < 0) {
		dev_err(&sbdev->dev, "%s(): slim_define_ch() failed: %d\n",
			__func__, ret);
		goto slim_define_ch_error;
	}
	for (i = 0; i < ch_cnt; i++) {
		ret = slim_connect_sink(sbdev, &sph[i], 1, ch_h[i]);
		if (ret < 0) {
			dev_err(&sbdev->dev,
				"%s(): slim_connect_sink() failed: %d\n",
				__func__, ret);
			goto slim_connect_sink_error;
		}
	}
	ret = slim_control_ch(sbdev, grph, SLIM_CH_ACTIVATE, commit);
	if (ret < 0) {
		dev_err(&sbdev->dev,
			"%s(): slim_control_ch() failed: %d\n",
			__func__, ret);
		goto slim_control_ch_error;
	}
	for (i = 0; i < ch_cnt; i++) {
		idx = es325_rx_ch_num_to_idx(ch_num[i]);
		dev_dbg(&sbdev->dev, "%s(): idx = %d\n", __func__, idx);
		rx[idx].grph = grph;
	}
	dev_dbg(&sbdev->dev, "-%s()==\n", __func__);
	return 0;
slim_control_ch_error:
slim_connect_sink_error:
	es325_close_slim_rx(sbdev, ch_num, ch_cnt);
slim_define_ch_error:
	return ret;
}

static int es325_cfg_slim_tx(struct slim_device *sbdev, unsigned int *ch_num,
			     unsigned int ch_cnt, unsigned int rate, bool commit)
{
	struct es325_priv *es325_priv = slim_get_devicedata(sbdev);
	struct es325_slim_ch *tx = es325_priv->slim_tx;
	u16 grph;
	u32 sph[ES325_SLIM_TX_PORTS] = {0};
	u16 ch_h[ES325_SLIM_TX_PORTS] = {0};
	struct slim_ch prop;
	int i;
	int idx = 0;
	int ret = 0;

	dev_dbg(&sbdev->dev, "+%s()==\n", __func__);
	dev_dbg(&sbdev->dev, "%s(): ch_cnt = %d\n", __func__, ch_cnt);
	dev_dbg(&sbdev->dev, "%s(): rate = %d\n", __func__, rate);

	for (i = 0; i < ch_cnt; i++) {
		idx = es325_tx_ch_num_to_idx(ch_num[i]);
		ch_h[i] = tx[idx].ch_h;
		sph[i] = tx[idx].sph;
		dev_dbg(&sbdev->dev, "%s(): idx = %d\n", __func__, idx);
		dev_dbg(&sbdev->dev, "%s(): ch_num[i] = %d\n", __func__, ch_num[i]);
		dev_dbg(&sbdev->dev, "%s(): ch_h[i] = %d\n", __func__, ch_h[i]);
		dev_dbg(&sbdev->dev, "%s(): sph[i] = 0x%08x\n", __func__, sph[i]);
	}

	prop.prot = SLIM_AUTO_ISO;
	prop.baser = SLIM_RATE_4000HZ;
	prop.dataf = SLIM_CH_DATAF_NOT_DEFINED;
	prop.auxf = SLIM_CH_AUXF_NOT_APPLICABLE;
	prop.ratem = (rate/4000);
	prop.sampleszbits = 16;

	ret = slim_define_ch(sbdev, &prop, ch_h, ch_cnt, true, &grph);
	if (ret < 0) {
		dev_err(&sbdev->dev, "%s(): slim_define_ch() failed: %d\n",
			__func__, ret);
		goto slim_define_ch_error;
	}
	for (i = 0; i < ch_cnt; i++) {
		ret = slim_connect_src(sbdev, sph[i], ch_h[i]);
		if (ret < 0) {
			dev_err(&sbdev->dev,
				"%s(): slim_connect_src() failed: %d\n",
				__func__, ret);
			dev_err(&sbdev->dev,
				"%s(): ch_num[0] = %d\n",
				__func__, ch_num[0]);
			goto slim_connect_src_error;
		}
	}
	ret = slim_control_ch(sbdev, grph, SLIM_CH_ACTIVATE, commit);
	if (ret < 0) {
		dev_err(&sbdev->dev,
			"%s(): slim_control_ch() failed: %d\n",
			__func__, ret);
		goto slim_control_ch_error;
	}
	for (i = 0; i < ch_cnt; i++) {
		idx = es325_tx_ch_num_to_idx(ch_num[i]);
		dev_dbg(&sbdev->dev, "%s(): idx = %d\n", __func__, idx);
		tx[idx].grph = grph;
	}
	dev_dbg(&sbdev->dev, "-%s()==\n", __func__);
	return 0;
slim_control_ch_error:
slim_connect_src_error:
	es325_close_slim_tx(sbdev, ch_num, ch_cnt);
slim_define_ch_error:
	return ret;
}

static int es325_close_slim_rx(struct slim_device *sbdev, unsigned int *ch_num,
			       unsigned int ch_cnt)
{
	struct es325_priv *es325_priv = slim_get_devicedata(sbdev);
	struct es325_slim_ch *rx = es325_priv->slim_rx;
	u16 grph = 0;
	u32 sph[ES325_SLIM_RX_PORTS] = {0};
	int i;
	int idx = 0;
	int ret = 0;

	dev_dbg(&sbdev->dev, "+%s()\n", __func__);

	for (i = 0; i < ch_cnt; i++) {
		idx = es325_rx_ch_num_to_idx(ch_num[i]);
		sph[i] = rx[idx].sph;
		grph = rx[idx].grph;
	}

	ret = slim_control_ch(sbdev, grph, SLIM_CH_REMOVE, true);
	if (ret < 0) {
		dev_err(&sbdev->dev,
			"%s(): slim_control_ch() failed: %d\n",
			__func__, ret);
		goto slim_control_ch_error;
	}
	for (i = 0; i < ch_cnt; i++) {
		idx = es325_rx_ch_num_to_idx(ch_num[i]);
		dev_dbg(&sbdev->dev,"%s(): idx = %d\n", __func__, idx);
		rx[idx].grph = 0;
	}
	ret = slim_disconnect_ports(sbdev, sph, ch_cnt);
	if (ret < 0) {
		dev_err(&sbdev->dev,
			"%s(): slim_disconnect_ports() failed: %d\n",
			__func__, ret);
	}
	
slim_control_ch_error:
	printk("=[ES325]=%s=close RX channel",__func__);
	for (i = 0; i < ch_cnt; i++)
		printk("[%d]",ch_num[i]);
	printk(" \n");
	
	dev_dbg(&sbdev->dev, "-%s()\n", __func__);
	return ret;
}

static int es325_close_slim_tx(struct slim_device *sbdev, unsigned int *ch_num,
			       unsigned int ch_cnt)
{
	struct es325_priv *es325_priv = slim_get_devicedata(sbdev);
	struct es325_slim_ch *tx = es325_priv->slim_tx;
	u16 grph = 0;
	u32 sph[ES325_SLIM_TX_PORTS] = {0};
	int i;
	int idx = 0;
	int ret = 0;

	dev_dbg(&sbdev->dev, "+%s()\n", __func__);

	for (i = 0; i < ch_cnt; i++) {
		idx = es325_tx_ch_num_to_idx(ch_num[i]);
		sph[i] = tx[idx].sph;
		if(i == 0)
			grph = tx[idx].grph;
	}

	ret = slim_control_ch(sbdev, grph, SLIM_CH_REMOVE, true);
	if (ret < 0) {
		dev_err(&sbdev->dev,
			"%s(): slim_control_ch() failed: %d\n",
			__func__, ret);
		goto slim_control_ch_error;
	}
	for (i = 0; i < ch_cnt; i++) {
		idx = es325_tx_ch_num_to_idx(ch_num[i]);
		dev_dbg(&sbdev->dev, "%s(): idx = %d\n", __func__, idx);
		tx[idx].grph = 0;
	}
	ret = slim_disconnect_ports(sbdev, sph, ch_cnt);
	if (ret < 0) {
		dev_err(&sbdev->dev,
			"%s(): slim_disconnect_ports() failed: %d\n",
			__func__, ret);
	}

slim_control_ch_error:
	printk("=[ES325]=%s=close TX channel",__func__);
	for (i = 0; i < ch_cnt; i++)
		printk("[%d]",ch_num[i]);
	printk(" \n");
	
	dev_dbg(&sbdev->dev, "-%s()\n", __func__);
	return ret;
}

int es325_remote_cfg_slim_rx(int dai_id)
{
	struct es325_priv *es325 = &es325_priv;
	int be_id;
	int rc = 0;

	if (dai_id != ES325_SLIM_1_PB
	    && dai_id != ES325_SLIM_2_PB)
		return 0;

	/* This is for defending ch_tot is not reset */
	if ((es325_rx1_route_enable == 0) && (es325_rx2_route_enable == 0)) {
		return 0;
	}

	if (es325->dai[dai_id - 1].ch_tot != 0) {
		/* start slim channels associated with id */
		rc = es325_cfg_slim_rx(es325->gen0_client,
				       es325->dai[dai_id - 1].ch_num,
				       es325->dai[dai_id - 1].ch_tot,
				       es325->dai[dai_id - 1].rate, false);

		be_id = es325_slim_be_id[dai_id - 1];
		es325->dai[be_id - 1].ch_tot = es325->dai[dai_id - 1].ch_tot;
		es325->dai[be_id - 1].rate = es325->dai[dai_id - 1].rate;
		if (be_id == ES325_SLIM_2_CAP) {
			es325->dai[be_id - 1].ch_num[0] = 138;
			es325->dai[be_id - 1].ch_num[1] = 139;
		} else if (be_id == ES325_SLIM_3_CAP) {
			es325->dai[be_id - 1].ch_num[0] = 143;
			es325->dai[be_id - 1].ch_num[1] = 144;
		}
		rc = es325_codec_cfg_slim_tx(es325, be_id);
		pr_info("=[ES325]=%s: MDM->>>[%d][%d]ES325[%d][%d]->>>WCD channel mapping\n", __func__,
			es325->dai[dai_id - 1].ch_num[0],
			es325->dai[dai_id -1].ch_tot == 1 ? 0 : es325->dai[dai_id - 1].ch_num[1],
			es325->dai[be_id - 1].ch_num[0],
			es325->dai[be_id - 1].ch_tot == 1 ? 0 : es325->dai[be_id - 1].ch_num[1]);
	}

	return rc;
}
EXPORT_SYMBOL_GPL(es325_remote_cfg_slim_rx);

int es325_remote_cfg_slim_tx(int dai_id)
{
	struct es325_priv *es325 = &es325_priv;
	int be_id;
	int ch_cnt;
	int rc = 0;

	if (dai_id != ES325_SLIM_1_CAP)
		return 0;

	/* This is for defending ch_tot is not reset */
	if (es325_tx1_route_enable == 0) {
		return 0;
	}

	if (es325->dai[dai_id - 1].ch_tot != 0) {
		/* start slim channels associated with id */
		if (dai_id == ES325_SLIM_1_CAP) {
			ch_cnt = es325_ap_tx1_ch_cnt;
		}
		rc = es325_cfg_slim_tx(es325->gen0_client,
				       es325->dai[dai_id - 1].ch_num,
				       ch_cnt,
				       es325->dai[dai_id - 1].rate, false);

		be_id = es325_slim_be_id[dai_id - 1];
		es325->dai[be_id - 1].ch_tot = es325->dai[dai_id - 1].ch_tot;
		es325->dai[dai_id - 1].ch_tot = ch_cnt;
		es325->dai[be_id - 1].rate = es325->dai[dai_id - 1].rate;
		if (be_id == ES325_SLIM_3_PB) {
			es325->dai[be_id - 1].ch_num[0] = 134;
			es325->dai[be_id - 1].ch_num[1] = 135;
		}
		rc = es325_codec_cfg_slim_rx(es325, be_id);
		pr_info("=[ES325]=%s: MDM<<<-[%d][%d]ES325[%d][%d]<<<-WCD channel mapping\n", __func__,
			es325->dai[dai_id - 1].ch_num[0],
			es325->dai[dai_id -1].ch_tot == 1 ? 0 : es325->dai[dai_id - 1].ch_num[1],
			es325->dai[be_id - 1].ch_num[0],
			es325->dai[be_id - 1].ch_tot == 1 ? 0 : es325->dai[be_id - 1].ch_num[1]);
	}

	return rc;
}
EXPORT_SYMBOL_GPL(es325_remote_cfg_slim_tx);

int es325_remote_close_slim_rx(int dai_id)
{
	struct es325_priv *es325 = &es325_priv;
	struct slim_device *sbdev = es325->gen0_client;
	int be_id;
	int rc = 0;

	dev_dbg(&sbdev->dev, "=[ES325]=%s(dai_id = %d)(ch_tot = %d)\n",
			__func__, dai_id,es325->dai[dai_id - 1].ch_tot);
			
	if (dai_id != ES325_SLIM_1_PB
		&& dai_id != ES325_SLIM_2_PB)
		return 0;

	if (es325->dai[dai_id - 1].ch_tot != 0) {
		es325_close_slim_rx(es325->gen0_client,
				    es325->dai[dai_id - 1].ch_num,
				    es325->dai[dai_id - 1].ch_tot);

		be_id = es325_slim_be_id[dai_id - 1];
		rc = es325_codec_close_slim_tx(es325, be_id);

		es325->dai[dai_id - 1].ch_tot = 0;
#ifdef ES325_SLEEP
		es325->dai[be_id - 1].ch_tot = 0;
#endif
	}

	return rc;
}
EXPORT_SYMBOL_GPL(es325_remote_close_slim_rx);

int es325_remote_close_slim_tx(int dai_id)
{
	struct es325_priv *es325 = &es325_priv;
	struct slim_device *sbdev = es325->gen0_client;
	int be_id;
	int rc = 0;

	dev_dbg(&sbdev->dev, "=[ES325]=%s(dai_id = %d)(ch_tot = %d)\n",
			__func__, dai_id,es325->dai[dai_id - 1].ch_tot);

	if (dai_id != ES325_SLIM_1_CAP)
		return 0;

	if (es325->dai[dai_id - 1].ch_tot != 0) {
		es325_close_slim_tx(es325->gen0_client,
					es325->dai[dai_id - 1].ch_num,
					es325->dai[dai_id - 1].ch_tot);

		be_id = es325_slim_be_id[dai_id - 1];
		rc = es325_codec_close_slim_rx(es325, be_id);

		es325->dai[dai_id - 1].ch_tot = 0;
#ifdef ES325_SLEEP
		es325->dai[be_id - 1].ch_tot = 0;
#endif
	}

	return rc;
}
EXPORT_SYMBOL_GPL(es325_remote_close_slim_tx);

static int es325_init_slim_slave(struct slim_device *sbdev)
{
	int rc = 0;

	dev_dbg(&sbdev->dev, "%s(): entry\n", __func__);

	rc = es325_alloc_slim_rx_chan(sbdev);
	rc = es325_alloc_slim_tx_chan(sbdev);

	return rc;
}

static void msg_to_bus_order(char *msg, int msg_len)
{
	char tmp;

	for (; msg_len > 0; msg_len -= 4, msg += 4) {
		tmp = *(msg + 3);
		*(msg + 3) = *(msg);
		*(msg) = tmp;
		tmp = *(msg + 2);
		*(msg + 2) = *(msg + 1);
		*(msg + 1) = tmp;
	}
}

/* Maximum number of attempts to read a VE in the presence of a "not
 * ready" response. */
#define ES325_RD_POLL_MAX 5
/* Interval between attemps to read a VE in milliseconds. */
#define ES325_RD_POLL_INTV 29

#ifdef BUS_TRANSACTIONS
#if defined(CONFIG_SND_SOC_ES325_I2C)
static int es325_i2c_read(struct es325_priv *es325, char *buf, int len);
static int es325_i2c_write(struct es325_priv *es325, char *buf, int len);
#define ES325_BUS_READ(x_es325, x_offset, x_width, x_buf, x_len, x_bus_order) \
	es325_i2c_read(x_es325, x_buf, x_len)
#define ES325_BUS_WRITE(x_es325, x_offset, x_width, x_buf, x_len, x_bus_order) \
	es325_i2c_write(x_es325, x_buf, x_len)
#elif defined(CONFIG_SND_SOC_ES325_SLIM)
static int es325_slim_read(struct es325_priv *es325, unsigned int offset,
			   unsigned int width, char *buf, int len, int bus_order);
static int es325_slim_write(struct es325_priv *es325, unsigned int offset,
			    unsigned int width, char *buf, int len, int bus_order);
#define ES325_BUS_READ(x_es325, x_offset, x_width, x_buf, x_len, x_bus_order) \
	es325_slim_read(x_es325, x_offset, x_width, x_buf, x_len, x_bus_order)
#define ES325_BUS_WRITE(x_es325, x_offset, x_width, x_buf, x_len, x_bus_order) \
	es325_slim_write(x_es325, x_offset, x_width, x_buf, x_len, x_bus_order)
#else
#error "es325.c - bus infrastructure not defined"
#endif
#else
/* Pretend all read and write operations on the bus are successful -
 * when no bus is available. */
#define ES325_BUS_READ(e, o, w, b, l, r) (0)
#define ES325_BUS_WRITE(e, o, w, b, l, r) (0)
#endif

#ifdef FIXED_CONFIG
static void es325_fixed_config(struct es325_priv *es325)
{
	u8 *msg_ptr = es325_internal_route_configs[6];
	while (*msg_ptr != 0xff) {
		u8 msg[4];
		memcpy(msg, msg_ptr, 4);
		es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				ES325_WRITE_VE_WIDTH, msg, 4, 1);
		pr_info("GAC:%s(): msg = %02x%02x%02x%02x\n", __func__,
			msg[0], msg[1], msg[2], msg[3]);
		msg_ptr += 4;
		usleep_range(20000, 20000);
	}
}
#endif

/*
static void es325_ping(struct es325_priv *es325)
{
	u8 req_msg[] = {
		0x80, 0x4f, 0x00, 0x00,
	};
	u8 ack_msg[4] = {
		0x00, 0x00, 0x00, 0x00,
	};

	es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
			 ES325_WRITE_VE_WIDTH, req_msg, 4, 1);
	usleep_range(20000, 20000);
	es325_slim_read(es325, ES325_READ_VE_OFFSET,
			ES325_READ_VE_WIDTH, ack_msg, 4, 1);
	pr_info("GAC:%s(): ping ack = %02x%02x%02x%02x\n", __func__,
		ack_msg[0], ack_msg[1], ack_msg[2], ack_msg[3]);
}
*/

#if defined(CONFIG_SND_SOC_ES325_I2C)
static int es325_i2c_read(struct es325_priv *es325, char *buf, int len)
{
	struct i2c_msg msg[] = {
		{
			.addr = es325->this_client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int rc = 0;

	pr_info("%s(): entry\n", __func__);

	/*
	rc = i2c_transfer(es325->this_client->adapter, msg, 1);
	if (rc < 0) {
		pr_err("%s(): i2c_transfer() failed, rc = %d", __func__, rc);
		return rc;
	}

	{
		int i;
		pr_info("%s(): i2c msg:\n", __func__);
		for (i = 0; i < len; ++i)
			pr_info("\t[%d] = 0x%02x\n", i, buf[i]);
	}

	pr_info("%s(): exit\n", __func__);
	*/
	return rc;
}

static int es325_i2c_write(struct es325_priv *es325, char *buf, int len)
{
	struct i2c_msg msg[] = {
		{
			.addr = es325->this_client->addr,
			.flags = 0,
			.len = len,
			.buf = buf,
		},
	};
	int rc = 0;

	/*
	pr_info("%s(): entry\n", __func__);

	{
		int i;

		pr_info("%s(): i2c msg:\n", __func__);
		for (i = 0; i < len; ++i)
			pr_info("\t[%d] = 0x%02x\n", i, buf[i]);
	}

	rc = i2c_transfer(es325->this_client->adapter, msg, 1);
	if (rc < 0) {
		pr_err("%s(): i2c_transfer() failed, rc = %d",
		__func__, rc);
		return rc;
	}

	pr_info("%s(): exit\n", __func__);
	*/
	return rc;
}
#else
static int es325_slim_read(struct es325_priv *es325, unsigned int offset,
			   unsigned int width, char *buf, int len,
			   int bus_order)
{
	char notready[] = { 0, 0, 0, 0 };
	struct slim_device *sbdev = es325->gen0_client;
	DECLARE_COMPLETION_ONSTACK(read_done);
	struct slim_ele_access msg = {
		.start_offset = offset,
		.num_bytes = width,
		/* .comp = &read_done, */
		.comp = NULL,
	};
	int rc = 0;
	int try = 0;
	for (try = 0; try < ES325_RD_POLL_MAX; try++) {
		rc = slim_request_val_element(sbdev, &msg, buf, len);
		if (rc != 0)
			break;
		if (memcmp(buf, notready, 4) != 0)
			break;
		usleep_range(ES325_RD_POLL_INTV, ES325_RD_POLL_INTV);
	}
	if (try >= ES325_RD_POLL_MAX && memcmp(buf, notready, 4) == 0) {
		pr_err("%s: failed not ready after %d tries\n", __func__, try);
		rc = -EIO;
	}

	if (bus_order)
		msg_to_bus_order(buf, len);
	if (rc != 0)
		pr_err("%s: read failed rc=%d\n", __func__, rc);
	return rc;
}

static int es325_slim_write(struct es325_priv *es325, unsigned int offset,
			    unsigned int width, char *buf, int len, int bus_order)
{
	struct slim_device *sbdev = es325->gen0_client;
	struct slim_ele_access msg = {
		.start_offset = offset,
		.num_bytes = width,
		.comp = NULL,
	};
	int rc = 0;

	if (bus_order)
		msg_to_bus_order(buf, len);
	rc = slim_change_val_element(sbdev, &msg, buf, len);
	if (rc != 0)
		pr_info("%s: rc=%d\n", __func__, rc);
	return rc;
}
#endif

static int es325_build_algo_read_msg(char *msg, int *msg_len,
				     unsigned int reg)
{
	unsigned int index = reg & ES325_ADDR_MASK;
	unsigned int paramid;

	if (index > ARRAY_SIZE(es325_algo_paramid))
		return -EINVAL;

	paramid = es325_algo_paramid[index];

	/* ES325_GET_ALGO_PARAM */
	*msg++ = (ES325_GET_ALGO_PARAM >> 8) & 0x00ff;
	*msg++ = ES325_GET_ALGO_PARAM & 0x00ff;

	/* PARAM ID */
	*msg++ = (paramid >> 8) & 0x00ff;
	*msg++ = paramid & 0x00ff;
	*msg_len = 4;

	return 0;
}

static int es325_build_algo_write_msg(char *msg, int *msg_len,
				      unsigned int reg, unsigned int value)
{
	unsigned int index = reg & ES325_ADDR_MASK;
	unsigned int cmd;
	unsigned int paramid;

	if (index > ARRAY_SIZE(es325_algo_paramid))
		return -EINVAL;

	paramid = es325_algo_paramid[index];

	/* ES325_SET_ALGO_PARAMID */
	cmd = ES325_SET_ALGO_PARAMID;
	if (reg & ES325_STAGED_CMD)
		cmd |= ES325_STAGED_MSG_BIT;
	*msg++ = (cmd >> 8) & 0x00ff;
	*msg++ = cmd & 0x00ff;

	/* PARAM ID */
	*msg++ = (paramid >> 8) & 0x00ff;
	*msg++ = paramid & 0x00ff;

	/* ES325_SET_ALGO_PARAM */
	cmd = ES325_SET_ALGO_PARAM;
	if (reg & ES325_STAGED_CMD)
		cmd |= ES325_STAGED_MSG_BIT;
	*msg++ = (cmd >> 8) & 0x00ff;
	*msg++ = cmd & 0x00ff;

	/* value */
	*msg++ = (value >> 8) & 0x00ff;
	*msg++ = value & 0x00ff;
	*msg_len = 8;

	return 0;
}

static int es325_build_dev_read_msg(char *msg, int *msg_len,
				    unsigned int reg)
{
	unsigned int index = reg & ES325_ADDR_MASK;
	unsigned int paramid;

	if (index > ARRAY_SIZE(es325_dev_paramid))
		return -EINVAL;

	paramid = es325_dev_paramid[index];

	/* ES325_GET_DEV_PARAM */
	*msg++ = (ES325_GET_DEV_PARAM >> 8) & 0x00ff;
	*msg++ = ES325_GET_DEV_PARAM & 0x00ff;

	/* PARAM ID */
	*msg++ = (paramid >> 8) & 0x00ff;
	*msg++ = paramid & 0x00ff;
	*msg_len = 4;

	return 0;
}

static int es325_build_dev_write_msg(char *msg, int *msg_len,
				     unsigned int reg, unsigned int value)
{
	unsigned int index = reg & ES325_ADDR_MASK;
	unsigned int cmd;
	unsigned int paramid;

	if (index > ARRAY_SIZE(es325_dev_paramid))
		return -EINVAL;

	paramid = es325_dev_paramid[index];

	/* ES325_SET_DEV_PARAMID */
	cmd = ES325_SET_DEV_PARAMID;
	if (reg & ES325_STAGED_CMD)
		cmd |= ES325_STAGED_MSG_BIT;
	*msg++ = (cmd >> 8) & 0x00ff;
	*msg++ = cmd & 0x00ff;

	/* PARAM ID */
	*msg++ = (paramid >> 8) & 0x00ff;
	*msg++ = paramid & 0x00ff;

	/* ES325_SET_DEV_PARAM */
	cmd = ES325_SET_DEV_PARAM;
	if (reg & ES325_STAGED_CMD)
		cmd |= ES325_STAGED_MSG_BIT;
	*msg++ = (cmd >> 8) & 0x00ff;
	*msg++ = cmd & 0x00ff;

	/* value */
	*msg++ = (value >> 8) & 0x00ff;
	*msg++ = value & 0x00ff;
	*msg_len = 8;

	return 0;
}

static int es325_build_cmd_read_msg(char *msg, int *msg_len,
				    unsigned int reg)
{
	unsigned int index = reg & ES325_ADDR_MASK;
	struct es325_cmd_access *cmd_access;

	if (index > ARRAY_SIZE(es325_cmd_access))
		return -EINVAL;
	cmd_access = es325_cmd_access + index;

	*msg_len = cmd_access->read_msg_len;
	memcpy(msg, &cmd_access->read_msg, *msg_len);

	return 0;
}

static int es325_build_cmd_write_msg(char *msg, int *msg_len,
				     unsigned int reg, unsigned int value)
{
	unsigned int index = reg & ES325_ADDR_MASK;
	struct es325_cmd_access *cmd_access;

	if (index > ARRAY_SIZE(es325_cmd_access))
		return -EINVAL;
	cmd_access = es325_cmd_access + index;

	*msg_len = cmd_access->write_msg_len;
	memcpy(msg, &cmd_access->write_msg, *msg_len);
	if (reg & ES325_STAGED_CMD)
		*msg |= (1 << 5);

	return 0;
}

static unsigned int es325_read(struct snd_soc_codec *codec,
			       unsigned int reg)
{
	struct es325_priv *es325 = &es325_priv;
	unsigned int access = reg & ES325_ACCESS_MASK;
	char req_msg[16];
	char ack_msg[16];
	unsigned int msg_len;
	unsigned int value;
	int rc;

	switch (access) {
	case ES325_ALGO_ACCESS:
		rc = es325_build_algo_read_msg(req_msg, &msg_len, reg);
		break;
	case ES325_DEV_ACCESS:
		rc = es325_build_dev_read_msg(req_msg, &msg_len, reg);
		break;
	case ES325_CMD_ACCESS:
		rc = es325_build_cmd_read_msg(req_msg, &msg_len, reg);
		break;
	case ES325_OTHER_ACCESS:
		return 0;
	default:
		rc = -EINVAL;
		break;
	}
	if (rc) {
		pr_err("%s(): failed to build read message for address = 0x%04x\n",
			__func__, reg);
		return rc;
	}

	rc = ES325_BUS_WRITE(es325, ES325_WRITE_VE_OFFSET,
			     ES325_WRITE_VE_WIDTH, req_msg, msg_len, 1);
	if (rc < 0) {
		pr_err("%s(): ES325_BUS_WRITE_ERROR = %d", __func__,rc);
		return rc;
	}
	usleep_range(20000, 20000);
	
	rc = ES325_BUS_READ(es325, ES325_READ_VE_OFFSET,
			    ES325_READ_VE_WIDTH, ack_msg, 4, 1);
	if (rc < 0) {
		pr_err("%s(): ES325_BUS_READ_ERROR = %d", __func__,rc);
		return rc;
	}
	value = ack_msg[2] << 8 | ack_msg[3];

	return value;
}

static int es325_write(struct snd_soc_codec *codec, unsigned int reg,
		       unsigned int value)
{
	struct es325_priv *es325 = &es325_priv;
	unsigned int access = reg & ES325_ACCESS_MASK;
	char msg[16];
	char *msg_ptr;
	int msg_len = 0;
	int i;
	int rc;

	switch (access) {
	case ES325_ALGO_ACCESS:
		rc = es325_build_algo_write_msg(msg, &msg_len, reg, value);
		break;
	case ES325_DEV_ACCESS:
		rc = es325_build_dev_write_msg(msg, &msg_len, reg, value);
		break;
	case ES325_CMD_ACCESS:
		rc = es325_build_cmd_write_msg(msg, &msg_len, reg, value);
		break;
	case ES325_OTHER_ACCESS:
		return 0;
	default:
		rc = -EINVAL;
		break;
	}
	if (rc) {
		pr_err("%s(): failed to build write message for address = 0x%04x\n",
			__func__, reg);
		return rc;
	}

	msg_ptr = msg;
	for (i = msg_len; i > 0; i -= 4) {
		rc = ES325_BUS_WRITE(es325, ES325_WRITE_VE_OFFSET,
				     ES325_WRITE_VE_WIDTH, msg_ptr, 4, 1);
		if (rc < 0) {
			pr_err("%s(): ES325_BUS_WRITE_ERROR = %d", __func__,rc);
			return rc;
		}
		usleep_range(20000, 20000);
		
		rc = ES325_BUS_READ(es325, ES325_READ_VE_OFFSET,
				    ES325_READ_VE_WIDTH, msg_ptr, 4, 1);
		if (rc < 0) {
			pr_err("%s(): ES325_BUS_READ_ERROR = %d", __func__,rc);
			return rc;
		}
		msg_ptr += 4;
	}

	return rc;
}

static ssize_t es325_route_status_show(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    int i = 0;
    int idx;
    ssize_t rc = 0;
    struct es325_priv *es325 = &es325_priv;
    int active = 0;
    char *route_st_names[] = {
        "ACTIVE",
        "MUTING",
        "SWITCHING",
        "UNMUTING",
        "INACTIVE",
        "es325 NO RESPONSE"
    };
   
    char *status_name[] = {
        "Route Status",

        "Slimbus Ports Raw Status",
        "Slimbus Active Rx Ports",
        "Slimbus Active Tx Ports",
    };
    u8 route_st_req_msg[] = {
        /* Route Status */
        0x80, 0x4f, 0x00, 0x00,
    };
   
    u8 port_st_req_msg[] = {
        /* SBUS port  State read */
        0x80, 0x0B, 0x09, 0x12,
    };

    u8 ack_msg[4];
    
    /* Read route status */
    es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
            ES325_WRITE_VE_WIDTH, route_st_req_msg, 4, 1);

    msleep(20);
   
   
    es325_slim_read(es325, ES325_READ_VE_OFFSET,
        ES325_READ_VE_WIDTH, ack_msg, 4, 1);

    pr_info("GAC:%s(): ping ack = %02x%02x%02x%02x\n", __func__,
        ack_msg[0], ack_msg[1], ack_msg[2], ack_msg[3]);

    /* Successful response? */
    if ((ack_msg[0] == 0x80) && (ack_msg[1] == 0x4f)) {
        rc = rc + snprintf(buf+rc, PAGE_SIZE - rc,
            "%s = 0x%02x%02x%02x%02x - %s\n",
            status_name[0],
            ack_msg[0], ack_msg[1],
            ack_msg[2], ack_msg[3],
            route_st_names[ack_msg[3]] );   

        /* Read Port status */
        es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
                ES325_WRITE_VE_WIDTH, port_st_req_msg, 4, 1);

        msleep(20);

        es325_slim_read(es325, ES325_READ_VE_OFFSET,
            ES325_READ_VE_WIDTH, ack_msg, 4, 1);

        pr_info("GAC:%s(): ping ack = %02x%02x%02x%02x\n", __func__,
            ack_msg[0], ack_msg[1], ack_msg[2], ack_msg[3]);
       
        if ((ack_msg[0] == 0x80) && (ack_msg[1] == 0x0B)) {
            u16 port_status = (ack_msg[2]  << 8) | ack_msg[3];
            rc = rc + snprintf(buf+rc, PAGE_SIZE - rc,
                    "%s = 0x%02x%02x%02x%02x \n",
                    status_name[1],
                    ack_msg[0], ack_msg[1],
                    ack_msg[2], ack_msg[3]);   
            
            rc = rc + snprintf(buf+rc, PAGE_SIZE - rc,
                    "%s = ",
                    status_name[2]);
            
            active = 0;
            for (i = 0; i < 10; i++){
                idx = (port_status & (1 << i)) >> i;
        
                if (idx == 1) {
                    active = 1;
                
                    rc = rc + snprintf(buf+rc, PAGE_SIZE - rc,
                            "%d ",
                            i );
                }
        
            }
            if (active == 0) {
                rc = rc + snprintf(buf+rc, PAGE_SIZE - rc,
                    "None\n%s = ",
                    status_name[3]);
            
            } else {
                
                rc = rc + snprintf(buf+rc, PAGE_SIZE - rc,
                    "\n%s = ",
                    status_name[3]);                
            }
            
            active = 0;
            for (i = 10; i < 16; i++){
                idx = (port_status & (1 << i)) >> i;
        
                if (idx == 1) {
                
                    active = 1;
                    rc = rc + snprintf(buf+rc, PAGE_SIZE - rc,
                            "%d ",
                            (i-10) );
                }
        
            }
            
            if (active == 0) {
                rc = rc + snprintf(buf+rc, PAGE_SIZE - rc,
                            "None\n" );
            
            } else {
                
                rc = rc + snprintf(buf+rc, PAGE_SIZE - rc,
                            "\n" );                
            }
            
            
            
            
            
        } else {
            rc = rc + snprintf(buf+rc, PAGE_SIZE - rc,
                    "%s = 0x%02x%02x%02x%02x - %s\n",
                    status_name[1],
                    ack_msg[0], ack_msg[1],
                    ack_msg[2], ack_msg[3],
                    "Cannot Read!" );   
        }
    } else {
        rc = rc + snprintf(buf+rc, PAGE_SIZE - rc,
                    "%s = 0x%02x%02x%02x%02x - %s\n",
                    status_name[0],
                    ack_msg[0], ack_msg[1],
                    ack_msg[2], ack_msg[3],
                    route_st_names[5] );   
    }
    return rc;
}
static DEVICE_ATTR(route_status, 0644, es325_route_status_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/es325-codec-gen0/route_status */

static ssize_t es325_route_config_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	pr_info("GAC:%s(): route=%ld\n", __func__,
		es325_internal_route_num);
	return snprintf(buf, PAGE_SIZE, "route=%ld\n",
		       es325_internal_route_num);
}

#ifndef ES325_SLEEP
static void es325_switch_route(long route_index)
{
	struct es325_priv *es325 = &es325_priv;
	u8 msg[4];
	u8 *msg_ptr;
	int ret;

	if (route_index > ES325_INTERNAL_ROUTE_MAX) {
		pr_info("%s(): new es325_internal_route = %ld is out of range\n",
			__func__, route_index);
		return;
	}

	pr_info("=[ES325]=%s():switch current es325_internal_route = %ld to new route = %ld\n",
		__func__, es325_internal_route_num, route_index);
	es325_internal_route_num = route_index;

	if (es325_network_type != NARROW_BAND) {
		if (es325_internal_route_num >= 0 &&
			es325_internal_route_num < 5)
				es325_internal_route_num += NETWORK_OFFSET;
	}

	msg_ptr = &es325_internal_route_configs[es325_internal_route_num][0];
	while (*msg_ptr != 0xff) {
		memcpy(msg, msg_ptr, 4);
		ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				       ES325_WRITE_VE_WIDTH, msg, 4, 1);
		msg_ptr += 4;
		usleep_range(5000, 5000);
	}
}
#else
static void es325_switch_route(void)
{
	struct es325_priv *es325 = &es325_priv;
	u8 msg[4];
	u8 *msg_ptr;
	int ret;


	if (es325_network_type == WIDE_BAND) {
		if (es325->new_internal_route_config >= 0 &&
			es325->new_internal_route_config < 5) {
			es325->new_internal_route_config += NETWORK_OFFSET;
			pr_info("=[ES325]=%s() adjust wideband offset\n", __func__);
		}
	}

	if (es325_network_type == NARROW_BAND) {
		if (es325->new_internal_route_config >= 0 + NETWORK_OFFSET &&
			es325->new_internal_route_config < 5 + NETWORK_OFFSET) {
			es325->new_internal_route_config -= NETWORK_OFFSET;
			pr_info("=[ES325]=%s() adjust narrowband offset\n", __func__);
		}
	}

	if (es325_2mic_enable == false) {
		if ((es325->new_internal_route_config >= 1 &&
			es325->new_internal_route_config < 3) ||
			(es325->new_internal_route_config >= 1 + NETWORK_OFFSET &&
			es325->new_internal_route_config < 3 + NETWORK_OFFSET)) {
			
			es325->new_internal_route_config += NS_OFFSET;
			pr_info("=[ES325]=%s() adjust 2mic_enable offset\n", __func__);
		}
	} else {
		if ((es325->new_internal_route_config >= 1 + NS_OFFSET &&
			es325->new_internal_route_config < 3 + NS_OFFSET) ||
			(es325->new_internal_route_config >= 1 + NETWORK_OFFSET + NS_OFFSET &&
			es325->new_internal_route_config < 3 + NETWORK_OFFSET + NS_OFFSET)) {
			
			es325->new_internal_route_config -= NS_OFFSET;
			pr_info("=[ES325]=%s() adjust 2mic_enable offset\n", __func__);
		}
	}

	pr_info("=[ES325]=%s() switch current es325_internal_route = %d to new route = %d\n",
		__func__, es325->internal_route_config, es325->new_internal_route_config);

	if (es325->new_internal_route_config > ES325_INTERNAL_ROUTE_MAX) {
		pr_info("=[ES325]=%s(): new es325_internal_route = %d is out of range\n",
			__func__, es325->new_internal_route_config);
		return;
	}

	if (es325->internal_route_config == es325->new_internal_route_config &&
		es325->rx_is_muted == false) {
		pr_info("=[ES325]=%s(): skip to set internal_route_config\n", __func__);
		return;
	}

	es325_internal_route_num = es325->new_internal_route_config;
	pr_info("=[ES325]=%s(): final es325_internal_route_num = %ld\n",
		__func__, es325_internal_route_num);
	msg_ptr = &es325_internal_route_configs[es325_internal_route_num][0];
	while (*msg_ptr != 0xff) {
		memcpy(msg, msg_ptr, 4);
		ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				       ES325_WRITE_VE_WIDTH, msg, 4, 1);
		msg_ptr += 4;
		usleep_range(5000, 5000);
	}
	es325->internal_route_config = es325->new_internal_route_config;
	es325->rx_is_muted = false;

}
#endif

static ssize_t es325_route_config_set(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	long route_index;
	int rc;

	pr_info("GAC:%s():buf = %s\n", __func__, buf);
	rc = kstrtol(buf, 10, &route_index);
#ifndef ES325_SLEEP
	es325_switch_route(route_index);
#else
	es325_priv.new_internal_route_config = route_index;
	pr_info("[HHJI] %s wakeup_cnt=%d\n", __func__, es325_priv.wakeup_cnt);
	mutex_lock(&es325_priv.pm_mutex);
	if (es325_priv.wakeup_cnt) {
		es325_switch_route();
	}
	mutex_unlock(&es325_priv.pm_mutex);
#endif
	return count;
}

static DEVICE_ATTR(route_config, 0644, es325_route_config_show,
		   es325_route_config_set);
/* /sys/devices/platform/msm_slim_ctrl.1/es325-codec-gen0/route_config */

#define SIZE_OF_VERBUF 256

static unsigned char first_char_msg[4] = {0x80, 0x20, 0x00, 0x00};
static unsigned char next_char_msg[4] = {0x80, 0x21, 0x00, 0x00};

static ssize_t es325_fw_version_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct es325_priv *es325 = &es325_priv;
	int rc, idx = 0;
	unsigned int imsg;
	unsigned char bmsg[4];
	char versionbuffer[SIZE_OF_VERBUF];
	char *verbuf = versionbuffer;
	unsigned char cmd[4];

	memset(verbuf,0,SIZE_OF_VERBUF);
	idx = 0;
	imsg = 1; /* force first loop */
	rc = 0;
	memcpy(cmd, first_char_msg, 4);
	while ((rc >= 0) &&
		(idx < (SIZE_OF_VERBUF-1)) &&
		(imsg & 0xFF)){
		rc = ES325_BUS_WRITE(es325, ES325_WRITE_VE_OFFSET,
				ES325_WRITE_VE_WIDTH, cmd, 4, 1);
		if (rc < 0){
			pr_err("%s: ES325_BUS_WRITE_ERROR = %d\n", __func__, rc);
			return snprintf(buf, PAGE_SIZE, "FW Version = Cannot Retrieve\n");
		}
		else{
			usleep_range(20000, 20000);
			rc = ES325_BUS_READ(es325, ES325_READ_VE_OFFSET,
				    ES325_READ_VE_WIDTH, bmsg, 4, 1);
			pr_info("%s() : get fw read %x %x %x %x\n", __func__, bmsg[0], bmsg[1], bmsg[2], bmsg[3]);
			if (rc < 0) {
				pr_err("%s: ES325_BUS_READ_ERROR = %d\n", __func__, rc);
				return snprintf(buf, PAGE_SIZE, "FW Version = Cannot Retrieve\n");
			}
			imsg = bmsg[3];
			if ((bmsg[0] == 0xFF) &&  (bmsg[1] == 0xFF)){
				pr_err("No version API on Audience\n");
				rc = -1;
			}
			else{
				verbuf[idx++] = (char) (bmsg[3]);
			}
		}
		memcpy(cmd, next_char_msg, 4);
	}
	/* Null terminate the string*/
	verbuf[idx] = '\0';
	pr_info("Audience fw ver %s\n", verbuf);
	return snprintf(buf, PAGE_SIZE, "FW Version = %s\n",verbuf);
}

static DEVICE_ATTR(fw_version, 0644, es325_fw_version_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/es325-codec-gen0/fw_version */

static ssize_t es325_txhex_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	pr_info("%s called\n", __func__);
	return 0;
}

static ssize_t es325_txhex_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct es325_priv *es325 = &es325_priv;
	u8 cmd[128];
	int cmdlen;
	int offset;
	u8 resp[4];
	int rc;

	pr_info("%s called\n", __func__);
	pr_info("%s count=%i\n", __func__, count);

	/* No command sequences larger than 128 bytes. */
	BUG_ON(count > (128 * 2) + 1);
	/* Expect a even number of hexadecimal digits terminated by a
	 * newline. */
	BUG_ON(!(count & 1));

	rc = hex2bin(cmd, buf, count / 2);
	BUG_ON(rc != 0);
	pr_info("%s rc==%i\n", __func__, rc);
	cmdlen = count / 2;
	offset = 0;
	pr_info("%s cmdlen=%i\n", __func__, cmdlen);
	while (offset < cmdlen) {
		/* Commands must be written in 4 byte blocks. */
		int wrsize = (cmdlen - offset > 4) ? 4 : cmdlen - offset;
		es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				 ES325_WRITE_VE_WIDTH, &cmd[offset], wrsize, 1);
		usleep_range(10000, 10000);
		es325_slim_read(es325, ES325_READ_VE_OFFSET,
				ES325_READ_VE_WIDTH, resp, 4, 1);
		pr_info("%s: %02x%02x%02x%02x\n", __func__, resp[0], resp[1], resp[2], resp[3]);
		offset += wrsize;
	}

	return count;
}
static DEVICE_ATTR(txhex, 0644, es325_txhex_show, es325_txhex_set);

static ssize_t es325_clock_on_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	char status[4];

	pr_debug("%s\n", __func__);
	if(es325_priv.clock_on)
		snprintf(status, sizeof("on"), "%s", "on");
	else
		snprintf(status, sizeof("off"), "%s", "off");

	return snprintf(buf, PAGE_SIZE, "clk_status: %s\n", status);
}
static DEVICE_ATTR(clock_on, 0644, es325_clock_on_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/es325-codec-gen0/clock_on_status */

static ssize_t es325_slim_ch_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct es325_priv *priv = &es325_priv;
	struct es325_slim_dai_data* dai = priv->dai;
	int length = 0;
	int i, j;
         
	for(i = 0; i < ES325_NUM_CODEC_SLIM_DAIS; i++) {
		length += sprintf(buf+length,"=dai[%d]=rate[%d]=ch_num=",i, dai[i].rate);
		for(j = 0; j < dai[i].ch_tot; j++)
	        length += sprintf(buf+length,"[%d]",dai[i].ch_num[j]);
		length += sprintf(buf+length,"%c\n",'=');
	}

	return length;
}
static DEVICE_ATTR(slim_ch_status, 0644, es325_slim_ch_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/es325-codec-gen0/slim_ch_status */

static ssize_t es325_reg_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	int length = 0;
	int i;
	int size = 0;

	length += sprintf(buf+length,"es325_reg : algo\n");  
	size = sizeof(es325_algo_paramid)/sizeof(unsigned short); /* 127 items */
	for(i = ES325_MIC_CONFIG; i < size; i++)
		length += sprintf(buf+length,"0x%04x : 0x%04x\n", i, es325_read(NULL, i));

	length += sprintf(buf+length,"\nes325_reg : dev\n");
	size = sizeof(es325_dev_paramid)/sizeof(unsigned short); /* 49 items */
	for(i = ES325_PORTA_WORD_LEN; i < (size + ES325_PORTA_WORD_LEN); i++)
		length += sprintf(buf+length,"0x%04x : 0x%04x\n", i, es325_read(NULL, i));

	return length;
}

static ssize_t es325_reg_write(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	char tempbuf[32];
	char *start = tempbuf;
	unsigned long reg, value;

	memcpy(tempbuf, buf, size);
	tempbuf[size] = 0;

	while (*start == ' ')
		start++;
	reg = simple_strtoul(start, &start, 16);
	while (*start == ' ')
		start++;
	if (strict_strtoul(start, 16, &value))
		return -EINVAL;

	es325_write(NULL, reg, value);
	
	return size;
}
static DEVICE_ATTR(es325_reg, 0644, es325_reg_show, es325_reg_write);
/* /sys/devices/platform/msm_slim_ctrl.1/es325-codec-gen0/es325_reg */

static ssize_t es325_cmd_reg_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	int length = 0;
	int i;
	int size = 0;

	/* removed 0x2001(first), 0x20d4(end) register read, because of error */
	size = sizeof(es325_cmd_access)/sizeof(struct es325_cmd_access); /* 213 items */
	for(i = ES325_POWER_STATE + 1; i < (size + ES325_POWER_STATE -1); i++)
		length += sprintf(buf+length,"0x%04x : 0x%04x\n", i, es325_read(NULL, i));

	return length;
}
static DEVICE_ATTR(es325_cmd_reg, 0644, es325_cmd_reg_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/es325-codec-gen0/es325_cmd_reg */

#define ES325_FW_LOAD_BUF_SZ 4
static int es325_bootup(struct es325_priv *es325)
{
	static char sync_ok[] = { 0x80, 0, 0, 0 };
	char msg[16];
	unsigned int buf_frames;
	char *buf_ptr;
	int rc;

	pr_info("%s(): entry\n", __func__);

	/* gpio_set_value(es325->pdata->reset_gpio, 0); */
	/* mdelay(1); */
	/* gpio_set_value(es325->pdata->reset_gpio, 1); */
	/* mdelay(50); */
	usleep_range(50000, 50000);
	pr_info("%s(): write ES325_BOOT_CMD\n", __func__);
	memset(msg, 0, 16);
	msg[0] = ES325_BOOT_CMD & 0x00ff;
	msg[1] = ES325_BOOT_CMD >> 8;
	pr_info("%s(): msg[0] = 0x%02x\n", __func__, msg[0]);
	pr_info("%s(): msg[1] = 0x%02x\n", __func__, msg[1]);
	pr_info("%s(): msg[2] = 0x%02x\n", __func__, msg[2]);
	pr_info("%s(): msg[3] = 0x%02x\n", __func__, msg[3]);
	rc = ES325_BUS_WRITE(es325, ES325_WRITE_VE_OFFSET,
			     ES325_WRITE_VE_WIDTH, msg, 4, 0);
	if (rc < 0) {
		pr_err("%s(): ES325_BUS_WRITE_ERROR = %d\n", __func__,rc);
		return	rc;
	}
	usleep_range(50000, 50000);
	pr_info("%s(): read boot cmd ack\n", __func__);
	memset(msg, 0, 16);
	rc = ES325_BUS_READ(es325, ES325_READ_VE_OFFSET, ES325_READ_VE_WIDTH,
			    msg, 4, 0);
	pr_info("%s(): msg[0] = 0x%02x\n", __func__, msg[0]);
	pr_info("%s(): msg[1] = 0x%02x\n", __func__, msg[1]);
	pr_info("%s(): msg[2] = 0x%02x\n", __func__, msg[2]);
	pr_info("%s(): msg[3] = 0x%02x\n", __func__, msg[3]);
	if (rc < 0) {
		pr_err("%s(): ES325_BUS_READ_ERROR = %d\n", __func__,rc);
		pr_err("%s(): firmware load failed boot ack\n", __func__);
		return	rc;
	}
	if ((msg[0] != (ES325_BOOT_ACK >> 8)) || (msg[1] != (ES325_BOOT_ACK & 0x00ff))) {
		pr_err("%s(): firmware load failed boot ack pattern", __func__);
		return	-EIO;
	}
	usleep_range(50000, 50000);

	pr_info("%s(): write firmware image\n", __func__);
	/* send image */
	buf_frames = es325->fw->size / ES325_FW_LOAD_BUF_SZ;
	pr_info("%s(): buf_frames = %d", __func__, buf_frames);
	buf_ptr = (char *)es325->fw->data;
	for ( ; buf_frames; --buf_frames, buf_ptr += ES325_FW_LOAD_BUF_SZ) {
		rc = ES325_BUS_WRITE(es325, ES325_WRITE_VE_OFFSET,
				     ES325_WRITE_VE_WIDTH, buf_ptr,
				     ES325_FW_LOAD_BUF_SZ, 0);
		if (rc < 0) {
			pr_err("%s(): ES325_BUS_WRITE_ERROR = %d\n", __func__,rc);
			pr_err("%s(): firmware load failed\n", __func__);
			return -EIO;
		}
	}
	if (es325->fw->size % ES325_FW_LOAD_BUF_SZ) {
		rc = ES325_BUS_WRITE(es325, ES325_WRITE_VE_OFFSET,
				     ES325_WRITE_VE_WIDTH, buf_ptr,
				     es325->fw->size % ES325_FW_LOAD_BUF_SZ, 0);
		if (rc < 0) {
			pr_err("%s(): ES325_BUS_WRITE_ERROR = %d\n", __func__,rc);
			pr_err("%s(): firmware load failed\n", __func__);
			return -EIO;
		}
	}

	/* Give the chip some time to become ready after firmware
	 * download. */
	usleep_range(50000, 50000);
	pr_info("%s(): write ES325_SYNC_CMD\n", __func__);
	memset(msg, 0, 16);
	msg[0] = ES325_SYNC_CMD >> 8;
	msg[1] = ES325_SYNC_CMD & 0x00ff;
	msg[2] = ES325_SYNC_POLLING >> 8;
	msg[3] = ES325_SYNC_POLLING & 0x00ff;
	pr_info("%s(): msg[0] = 0x%02x\n", __func__, msg[0]);
	pr_info("%s(): msg[1] = 0x%02x\n", __func__, msg[1]);
	pr_info("%s(): msg[2] = 0x%02x\n", __func__, msg[2]);
	pr_info("%s(): msg[3] = 0x%02x\n", __func__, msg[3]);
	rc = ES325_BUS_WRITE(es325, ES325_WRITE_VE_OFFSET,
			     ES325_WRITE_VE_WIDTH, msg, 4, 1);
	if (rc < 0) {
		pr_err("%s(): ES325_BUS_WRITE_ERROR = %d\n", __func__,rc);
		pr_err("%s(): firmware load failed sync write\n", __func__);
		return rc;
	}
	usleep_range(50000, 50000);
	pr_info("%s(): read sync cmd ack\n", __func__);
	memset(msg, 0, 16);
	rc = ES325_BUS_READ(es325, ES325_READ_VE_OFFSET, ES325_READ_VE_WIDTH,
			    msg, 4, 1);
	pr_info("%s(): msg[0] = 0x%02x\n", __func__, msg[0]);
	pr_info("%s(): msg[1] = 0x%02x\n", __func__, msg[1]);
	pr_info("%s(): msg[2] = 0x%02x\n", __func__, msg[2]);
	pr_info("%s(): msg[3] = 0x%02x\n", __func__, msg[3]);
	if (rc < 0) {
		pr_err("%s(): ES325_BUS_READ_ERROR = %d\n", __func__,rc);
		pr_err("%s(): error reading firmware sync ack rc=%d\n",
		       __func__, rc);
		return rc;
	}
	if (memcmp(msg, sync_ok, 4) == 0) {
		pr_info("%s(): firmware sync ack good=0x%02x%02x%02x%02x\n",
			__func__, msg[0], msg[1], msg[2], msg[3]);
	} else {
		pr_err("%s(): firmware sync ack failed=0x%02x%02x%02x%02x\n",
		       __func__, msg[0], msg[1], msg[2], msg[3]);
		return -EIO;
	}

	pr_info("%s(): exit\n", __func__);
	return 0;
}

static int register_snd_soc(struct es325_priv *priv);

static int fw_download(void *arg)
{
	struct es325_priv *priv = (struct es325_priv *)arg;
	int rc;

	pr_info("%s(): called\n", __func__);
	pr_info("%s(): es325 gen0 LA=%d\n", __func__, priv->gen0_client->laddr);
#ifdef BUS_TRANSACTIONS
	rc = es325_bootup(priv);
#endif
	pr_info("%s(): bootup rc=%d\n", __func__, rc);
	pr_info("%s(): would release firmware here...\n", __func__);
	release_firmware(priv->fw);

	pr_info("%s(): registering sound soc\n", __func__);
	rc = register_snd_soc(priv);
	pr_info("%s(): register_snd_soc rc=%d\n", __func__, rc);

#ifdef FIXED_CONFIG
	es325_fixed_config(priv);
#endif

	pr_info("%s(): release module\n", __func__);
	module_put(THIS_MODULE);
	pr_info("%s(): exiting fw_download\n", __func__);
	return 0;
}

static const char *fw_path[] = {
       "/data"
};

/* Don't inline this: 'struct kstat' is biggish */
static noinline_for_stack long fw_file_size(struct file *file)
{
       struct kstat st;
       if (vfs_getattr(file->f_path.mnt, file->f_path.dentry, &st))
               return -1;
       if (!S_ISREG(st.mode))
               return -1;
       if (st.size != (long)st.size)
               return -1;
       return st.size;
}

static bool fw_read_file_contents(struct file *file, struct firmware *fw)
{
       long size;
       char *buf;

       size = fw_file_size(file);
       if (size < 0)
               return false;
       buf = vmalloc(size);
       if (!buf)
               return false;
       if (kernel_read(file, 0, buf, size) != size) {
           vfree(buf);
           return false;
       }
       fw->data = buf;
       fw->size = size;
       return true;
}

static bool fw_get_filesystem_firmware(struct firmware *fw, const char *name)
{
       int i;
       bool success = false;
       char *path = __getname();

       for (i = 0; i < ARRAY_SIZE(fw_path); i++) {
               struct file *file;
               snprintf(path, PATH_MAX, "%s/%s", fw_path[i], name);

               file = filp_open(path, O_RDONLY, 0);
               if (IS_ERR(file))
                       continue;
               success = fw_read_file_contents(file, fw);
               fput(file);
               if (success)
                       break;
       }
       __putname(path);
       return success;
}

static int es325_sleep(struct es325_priv *es325);
static int es325_wakeup(struct es325_priv *es325);

static ssize_t
es325_firmware_store(struct device *dev, struct device_attribute *attr,
                  const char *buf, size_t count)
{
    unsigned long val;
    int err;
	const char *name = FIRMWARE_NAME;
#if defined(CONFIG_MACH_JACTIVE_ATT)
	if (system_rev >= 12) /* HW REV02 */
		name = FIRMWARE_NAME_REV02;
#endif

    err = kstrtoul(buf, 10, &val);
    if (err) {
		pr_err("%s err %d\n", __func__, err);
        return err;
	}

	if(val == 1) {
		fw_get_filesystem_firmware(es325_priv.fw, name);
		if(es325_priv.fw->data == NULL){
			pr_err("there is no firmware found\n");
			return count;
		}
	    es325_priv.pdata->es325_clk_cb(1);
	    usleep_range(1000, 1100);
		es325_wakeup(&es325_priv);
	    es325_priv.clock_on = 1;
		msleep(100);

		/* reset the audience chip */
		gpio_set_value(es325_priv.pdata->reset_gpio, 0);
		mdelay(1);
		gpio_set_value(es325_priv.pdata->reset_gpio, 1);
		msleep(50);
		gpio_set_value(es325_priv.pdata->wakeup_gpio, 1);
		msleep(50);

		es325_bootup(&es325_priv);
		es325_fixed_config(&es325_priv);
		msleep(100);
		
		es325_sleep(&es325_priv);
		es325_priv.pdata->es325_clk_cb(0);
		es325_priv.clock_on = 0;
		msleep(100);
	} else {
		return -EINVAL;
	}

    return count;
}
static DEVICE_ATTR(firmware, 0644, NULL, es325_firmware_store);

static int es325_sleep(struct es325_priv *es325)
{
	char pwr_cmd[] = {
		0x90, 0x10, 0, 1
	};
	int rc;
	int remain = 3;

	pr_info("+[ES325]=%s()=\n", __func__);

	/* FIXME: check host vs bus ordering in the write. Why is this
	 * still so confused?
	 */
LOOP:
	rc = ES325_BUS_WRITE(es325, ES325_WRITE_VE_OFFSET,
			     ES325_WRITE_VE_WIDTH, pwr_cmd, 4, 1);
	if (rc < 0) {
		if (--remain > 0) {
			pr_info("=[ES325]= wrapper %s sleep command failed remain count %d\n",
				__func__, remain);
			usleep_range(1000, 1100);
			goto LOOP;
		}
	}

	if (rc < 0) {
		pr_err("=[ES325]= error sending sleep cmd rc=%d\n", rc);
		return rc;
	}
	usleep_range(20000, 20000);

	pr_info("-[ES325]=%s()=\n", __func__);
	return 0;
}

static int es325_wakeup(struct es325_priv *es325)
{
	char sync_cmd[] = {
		0x90, 0, 0, 1 /* 0x80,0,0,1 */
	};
	int rc;
	int i, remain = 3;

	/* FIXME: hack to wakeup slimbus on apq. */
	pr_info("+[ES325]%s()=\n", __func__);

	/* usleep_range(1000, 1200); */

	/* Assert wakeup signal H->L, and give the chip time to
	 * respond. */
	gpio_set_value(es325->pdata->wakeup_gpio, 0);
	msleep(30);

	/* The chip slimbus interface needs time to report_present and
	 * then be re-assigned a logical address. Wait an arbitrary
	 * period here as the slimbus stack does not support
	 * REPORT_ABSENT. */

	/* FIXME: check host vs bus ordering in the write. Why is this
	 * still so confused?
	 */
	for (i = 0; i < remain; i++) {
		rc = ES325_BUS_WRITE(es325, ES325_WRITE_VE_OFFSET,
			     ES325_WRITE_VE_WIDTH, sync_cmd, 4, 1);
		if (rc >= 0) {
			pr_info("=[ES325]= sending sync cmd success rc=%d\n",
				rc);
			break;
		} else {
			pr_info("=[ES325]= %s wakeup command failed rc=%d remain count %d\n",
				__func__, rc, i);
			usleep_range(1000, 1100);
		}
	}
	usleep_range(1000, 1100);
	/* Deassert wakeup signal L->H. */
	gpio_set_value(es325->pdata->wakeup_gpio, 1);

	pr_info("-[ES325]%s()=\n", __func__);
	return 0;
}

/* static const char *es325_algo_sample_rate_texts[] = { */
/*      "Narrow band", "Wide band", "Super wide band" */
/* }; */
/* static const struct soc_enum es325_algo_sample_rate_enum = */
/*      SOC_ENUM_SINGLE(ES325_ALGO_SAMPLE_RATE, 0, */
/*                      ARRAY_SIZE(es325_algo_sample_rate_texts), */
/*                      es325_algo_sample_rate_texts); */

/* static const char *es325_change_status_texts[] = { */
/*      "Active", "Muting", "Switching", "Unmuting", "Inactive" */
/* }; */
/* static const struct soc_enum es325_change_status_enum = */
/*      SOC_ENUM_SINGLE(ES325_CHANGE_STATUS, 0, */
/*                      ARRAY_SIZE(es325_change_status_texts), */
/*                      es325_change_status_texts); */

/* static const char *es325_rx_tx_mbc_texts[] = { */
/*      "Off", "On voice", "On", */
/* }; */
/* static const struct soc_enum es325_tx_mbc_enum = */
/*      SOC_ENUM_SINGLE(ES325_TX_MBC, 0, */
/*                      ARRAY_SIZE(es325_rx_tx_mbc_texts), */
/*                      es325_rx_tx_mbc_texts); */
/* static const struct soc_enum es325_rx_mbc_enum = */
/*      SOC_ENUM_SINGLE(ES325_RX_MBC, 0, */
/*                      ARRAY_SIZE(es325_rx_tx_mbc_texts), */
/*                      es325_rx_tx_mbc_texts); */

/* static const char *es325_rx_ns_adaptation_speed_texts[] = { */
/*      "Reserved", "Normal", "Fast", */
/* }; */
/* static const struct soc_enum es325_rx_ns_adaptation_speed_enum = */
/*      SOC_ENUM_SINGLE(ES325_RX_NS_ADAPTATION_SPEED, 0, */
/*                      ARRAY_SIZE(es325_rx_ns_adaptation_speed_texts), */
/*                      es325_rx_ns_adaptation_speed_texts); */

/* static const char *es325_avalon_av_config_texts[] = { */
/*      "AV all", "AV ISP", "AV ESP", "AV HPH", "AV REC", "AV DIG", "AV EAR" */
/* }; */
/* static const struct soc_enum es325_avalon_av_config_enum = */
/*      SOC_ENUM_SINGLE(ES325_AVALON_AV_CONFIG, 0, */
/*                      ARRAY_SIZE(es325_avalon_av_config_texts), */
/*                      es325_avalon_av_config_texts); */

/* static const char *es325_avalon_eq_preset_texts[] = { */
/*      "Flat", "Jazz", "Pop", "Classic", "Rock", "Bass boost", "Treble boost", */
/*      "R and B", "Hip hop", "Vocal boost", "Latin", "Piano", "New age", */
/*      "Country", "Blues" */
/* }; */
/* static const struct soc_enum es325_avalon_eq_preset_enum = */
/*      SOC_ENUM_SINGLE(ES325_AVALON_EQ_PRESET, 0, */
/*                      ARRAY_SIZE(es325_avalon_eq_preset_texts), */
/*                      es325_avalon_eq_preset_texts); */

/* static const char *es325_avalon_stat_ns_adap_speed_texts[] = { */
/*      "Reserved", "Slow", "Medium", "Fast", */
/* }; */
/* static const struct soc_enum es325_avalon_stat_ns_adap_speed_enum = */
/*      SOC_ENUM_SINGLE(ES325_AVALON_EQ_PRESET, 0, */
/*                      ARRAY_SIZE(es325_avalon_eq_preset_texts), */
/*                      es325_avalon_eq_preset_texts); */

/* static const char *es325_mode_config_texts[] = { */
/*      "Close talk", "Hand-held speakerphone", "Desktop speakerphone", */
/*      "1Mic headset", "1Mic bluetooth", "ASR assist close talk", */
/*      "ARS assist hand-held speakerphone" */
/* }; */
/* static const struct soc_enum es325_mode_config_enum = */
/*      SOC_ENUM_SINGLE(ES325_MIC_CONFIG, 0, */
/*                      ARRAY_SIZE(es325_mode_config_texts), */
/*                      es325_mode_config_texts); */

/* static const char *es325_audio_config_texts[] = { */
/*      "Reserved00", "Reserved01", "Headphone playback", "Audio recording" */
/* }; */
/* static const struct soc_enum es325_audio_config_enum = */
/*      SOC_ENUM_SINGLE(ES325_AVALON_AV_CONFIG, 0, */
/*                      ARRAY_SIZE(es325_audio_config_texts), */
/*                      es325_audio_config_texts); */

/* static const char *es325_parametric_eq_texts[] = { */
/*      "Off", "Jazz", "Pop", "Classic", "Rock", "Custom1", "Custom2", */
/*      "R and B", "Hip hop" */
/* }; */
/* static const struct soc_enum es325_parametric_eq_enum = */
/*      SOC_ENUM_SINGLE(ES325_AVALON_EQ_PRESET, 0, */
/*                      ARRAY_SIZE(es325_parametric_eq_texts), */
/*                      es325_parametric_eq_texts); */

/* static const char *es325_stereo_widening_width_texts[] = { */
/*      "Narrow", "Normal", "Wide" */
/* }; */
/* static const struct soc_enum es325_stereo_widening_width_enum = */
/*      SOC_ENUM_SINGLE(ES325_AVALON_STEREO_WIDTH, 0, */
/*                      ARRAY_SIZE(es325_stereo_widening_width_texts), */
/*                      es325_stereo_widening_width_texts); */

static int es325_put_control_value(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	/* struct snd_soc_codec *codec = es325_priv.codec; */
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	/* unsigned int shift = mc->shift; */
	/* unsigned int max = mc->max; */
	/* unsigned int invert = mc->invert; */
	unsigned int value;
	int rc = 0;

	/* dev_dbg(codec->dev, "%s(): reg = %d\n", __func__, reg); */
	/* dev_dbg(codec->dev, "%s(): shift = %d\n", __func__, shift); */
	/* dev_dbg(codec->dev, "%s(): max = %d\n", __func__, max); */
	/* dev_dbg(codec->dev, "%s(): invert = %d\n", __func__, invert); */
	/* dev_dbg(codec->dev, "%s(): value = %ld\n", __func__, */
	/*	ucontrol->value.integer.value[0]); */
	value = ucontrol->value.integer.value[0];
	/* rc = snd_soc_write(codec, reg, value); */
	rc = es325_write(NULL, reg, value);

	return 0;
}

static int es325_get_control_value(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	/* struct snd_soc_codec *codec = es325_priv.codec; */
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	/* unsigned int shift = mc->shift; */
	/* unsigned int max = mc->max; */
	/* unsigned int invert = mc->invert; */
	unsigned int value;

	/* dev_dbg(codec->dev, "%s(): reg = %d\n", __func__, reg); */
	/* dev_dbg(codec->dev, "%s(): shift = %d\n", __func__, shift); */
	/* dev_dbg(codec->dev, "%s(): max = %d\n", __func__, max); */
	/* dev_dbg(codec->dev, "%s(): invert = %d\n", __func__, invert); */
	/* value = snd_soc_read(codec, reg); */
	value = es325_read(NULL, reg);
	/* dev_dbg(codec->dev, "%s(): value = %d\n", __func__, value); */
	ucontrol->value.integer.value[0] = value;
	/* dev_dbg(codec->dev, "%s(): value = %ld\n", __func__, */
	/*	ucontrol->value.integer.value[0]); */

	return 0;
}

static int es325_put_control_enum(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int max = e->max;
	unsigned int value;
	int rc = 0;

	pr_debug("%s(): reg = %d\n", __func__, reg);
	pr_debug("%s(): max = %d\n", __func__, max);
	value = ucontrol->value.enumerated.item[0];
	rc = es325_write(NULL, reg, value);

	return 0;
}

static int es325_get_control_enum(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int max = e->max;
	unsigned int value;

	pr_debug("%s(): reg = %d\n", __func__, reg);
	pr_debug("%s(): max = %d\n", __func__, max);
	value = es325_read(NULL, reg);
	ucontrol->value.enumerated.item[0] = value;

	return 0;
}

static void es325_update_VEQ_enable(void)
{
	struct es325_priv *es325 = &es325_priv;
	char msg[4];
	char cmd_str[8] = {0xb0, 0x17, 0x00, 0x09, 0x90, 0x18, 0x00, 0x00}; /*VEQ Disable */
	int ret;

	if (es325_VEQ_enable == es325_VEQ_enable_new)
		return;
	if (es325_priv.wakeup_cnt) {
		if (es325_VEQ_enable_new) {
			cmd_str[7] = 0x01; /* VEQ Enable */
		}
		
		memcpy(msg, cmd_str, 4);
		pr_debug("%s: write=0x%x, 0x%x, 0x%x, 0x%x\n", __func__, msg[0], msg[1], msg[2], msg[3]);
		ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				ES325_WRITE_VE_WIDTH, msg, 4, 1);

		memcpy(msg, cmd_str+4, 4);
		pr_debug("%s: write=0x%x, 0x%x, 0x%x, 0x%x\n", __func__, msg[0], msg[1], msg[2], msg[3]);
		ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				ES325_WRITE_VE_WIDTH, msg, 4, 1);
		es325_VEQ_enable = es325_VEQ_enable_new;
		pr_info("=[ES325]=%s VEQ enable value=%d =\n",__func__,es325_VEQ_enable);
	}
}

static int es325_put_VEQ_enable_control(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	es325_VEQ_enable_new = ucontrol->value.integer.value[0];
	es325_update_VEQ_enable();
	return 0;
}

static int es325_get_VEQ_enable_control(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = es325_VEQ_enable_new;
	return 0;
}

static void es325_update_BWE_enable(void)
{
	struct es325_priv *es325 = &es325_priv;
	char msg[4];
	char cmd_str[8] = {0xb0, 0x17, 0x00, 0x4f, 0x90, 0x18, 0x00, 0x00}; /* BWE Off */
	int ret;
	pr_info("[ES325] %s: curr=%d, new=%d\n", __func__, es325_BWE_enable, es325_BWE_enable_new);

	if (es325_BWE_enable == es325_BWE_enable_new)
		return;

	if (es325_priv.wakeup_cnt) {
		if (es325_BWE_enable_new) {
			cmd_str[7] = 0x01; /* BWE On */
		}
		
		memcpy(msg, cmd_str, 4);
		pr_debug("%s: write=0x%x, 0x%x, 0x%x, 0x%x\n", __func__, msg[0], msg[1], msg[2], msg[3]);
		ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				ES325_WRITE_VE_WIDTH, msg, 4, 1);

		memcpy(msg, cmd_str+4, 4);
		pr_debug("%s: write=0x%x, 0x%x, 0x%x, 0x%x\n", __func__, msg[0], msg[1], msg[2], msg[3]);
		ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				ES325_WRITE_VE_WIDTH, msg, 4, 1);

		es325_BWE_enable = es325_BWE_enable_new;
	}
}

static int es325_put_BWE_enable_control(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	es325_BWE_enable_new = ucontrol->value.integer.value[0];
	pr_info("%s: value=%d\n", __func__, es325_BWE_enable_new);
	es325_update_BWE_enable();
	return 0;
}

static int es325_get_BWE_enable_control(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = es325_BWE_enable_new;
	return 0;
}

static void es325_update_Tx_NS(void)
{
	struct es325_priv *es325 = &es325_priv;
	char msg[4];
	char cmd_str[8] = {0xb0, 0x17, 0x00, 0x4b, 0x90, 0x18, 0x00, 0x00}; /* Tx NS level */
	int ret;

	if (es325_Tx_NS == es325_Tx_NS_new)
		return;

	if (es325_priv.wakeup_cnt) {
		cmd_str[7] = es325_Tx_NS_new;
		memcpy(msg, cmd_str, 4);
		pr_debug("%s: write=0x%x, 0x%x, 0x%x, 0x%x\n", __func__, msg[0], msg[1], msg[2], msg[3]);
		ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				ES325_WRITE_VE_WIDTH, msg, 4, 1);

		memcpy(msg, cmd_str+4, 4);
		pr_debug("%s: write=0x%x, 0x%x, 0x%x, 0x%x\n", __func__, msg[0], msg[1], msg[2], msg[3]);
		ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				ES325_WRITE_VE_WIDTH, msg, 4, 1);
		es325_Tx_NS = es325_Tx_NS_new;
		pr_info("=[ES325]=%s Tx Ns value=%d =\n",__func__,es325_Tx_NS);
	}
}

static int es325_put_Tx_NS_control(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	es325_Tx_NS_new = ucontrol->value.integer.value[0];
	es325_update_Tx_NS();
	return 0;
}

static int es325_get_Tx_NS_control(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = es325_Tx_NS_new;
	return 0;
}

static int es325_get_rx1_route_enable_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = es325_rx1_route_enable;
	return 0;
}

static int es325_put_rx1_route_enable_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	es325_rx1_route_enable = ucontrol->value.integer.value[0];
	return 0;
}

static int es325_get_tx1_route_enable_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = es325_tx1_route_enable;
	return 0;
}

static int es325_put_tx1_route_enable_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	es325_tx1_route_enable = ucontrol->value.integer.value[0];
	return 0;
}

static int es325_get_rx2_route_enable_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = es325_rx2_route_enable;
	return 0;
}

static int es325_put_rx2_route_enable_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	es325_rx2_route_enable = ucontrol->value.integer.value[0];
	return 0;
}

int es325_remote_route_enable(struct snd_soc_dai *dai)
{
	switch (dai->id) {
	case ES325_SLIM_1_PB:
		return es325_rx1_route_enable;
	case ES325_SLIM_1_CAP:
		return es325_tx1_route_enable;
	case ES325_SLIM_2_PB:
		return es325_rx2_route_enable;
	default:
		return 0;
	}
}
EXPORT_SYMBOL_GPL(es325_remote_route_enable);

static int es325_put_internal_route_config(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
#ifdef ES325_SLEEP
	struct es325_priv *es325 = &es325_priv;
	es325->new_internal_route_config = ucontrol->value.integer.value[0];
#else
	es325_switch_route(ucontrol->value.integer.value[0]);
#endif
	return 0;
}

static int es325_get_internal_route_config(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
#ifndef ES325_SLEEP
	ucontrol->value.integer.value[0] = es325_internal_route_num;
	return 0;
#else
	struct es325_priv *es325 = &es325_priv;
	ucontrol->value.integer.value[0] = es325->new_internal_route_config;
	return 0;
#endif
}

static int es325_put_network_type(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int network = ucontrol->value.integer.value[0];
	pr_info("GAC:%s():new network type = %d\n", __func__,
					network);

	if (network != NARROW_BAND)
		es325_network_type = WIDE_BAND;
	else
		es325_network_type = NARROW_BAND;

	pr_info("%s: wakeup_cnt=%d\n", __func__, es325_priv.wakeup_cnt);
	mutex_lock(&es325_priv.pm_mutex);
	if (es325_priv.wakeup_cnt) {
		es325_switch_route();
	}
	mutex_unlock(&es325_priv.pm_mutex);

	return 0;
}

static int es325_get_network_type(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = es325_network_type;
	pr_debug("%s(): es325 network type = %d\n", __func__,
		es325_network_type);

	return 0;
}

static int es325_put_2mic_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int enable = ucontrol->value.integer.value[0];
	pr_info("GAC:%s():new 2mic_enable = %d, current_route_config = %ld\n", __func__,
					enable, es325_internal_route_num);

	if (enable == 0)
		es325_2mic_enable = false;
	else
		es325_2mic_enable = true;

	pr_info("%s: wakeup_cnt=%d\n", __func__, es325_priv.wakeup_cnt);
	mutex_lock(&es325_priv.pm_mutex);
	if (es325_priv.wakeup_cnt) {
		if ((es325_internal_route_num >= 1 && 
			es325_internal_route_num < 3 + NS_OFFSET) ||
			(es325_internal_route_num >= 1 + NETWORK_OFFSET &&
			es325_internal_route_num < 3 + + NS_OFFSET + NETWORK_OFFSET)) {

			es325_switch_route();
		}
	}
	mutex_unlock(&es325_priv.pm_mutex);

	return 0;
}

static int es325_get_2mic_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = es325_2mic_enable;
	pr_debug("%s(): es325 2mic enable = %d\n", __func__,
		es325_2mic_enable);

	return 0;
}

int es325_set_VEQ_max_gain(int volume)
{
	struct es325_priv *es325 = &es325_priv;
	char msg[4];
	static char cmd_str[8] = {0xb0, 0x17, 0x00, 0x00, 0x90, 0x18, 0x00, 0x00};
#if defined(CONFIG_MACH_JF_VZW) || defined(CONFIG_MACH_JF_USC)
	/* 8 level Voice Rx volume for VZW and USC */
	static char VEQ_max_gain[8] = {3, 3, 3, 3, 3, 3, 3, 3}; /* index 0 means max volume */
	static char VEQ_adj_gain[8] = {23, 2, 2, 2, 2, 2, 2, 2};
#elif defined(CONFIG_MACH_JF_SPR)
	/* SPR */
	static char VEQ_max_gain[6] = {4, 6, 9, 12, 9, 6}; /* index 0 means max volume */
	static char VEQ_adj_gain[6] = {23, 23, 23, 23, 23, 23};
#else
	/* ATT, TMO and others */
	static char VEQ_max_gain[6] = {3, 0, 0, 0, 0, 0}; /* index 0 means max volume */
	static char VEQ_adj_gain[6] = {23, 0, 0, 0, 0, 0};
#endif
	int ret;
	int i;

	pr_info("%s: volume=%d\n", __func__, volume);

	cmd_str[3] = 0x3d; /* VEQ Max Gain ID */
	cmd_str[7] = VEQ_max_gain[volume];
	for (i = 0; i < sizeof(cmd_str)/4; i++) {
		memcpy(msg, cmd_str + (4 * i), 4);
		pr_debug("%s: write=0x%x, 0x%x, 0x%x, 0x%x\n", __func__, msg[0], msg[1], msg[2], msg[3]);
		ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				ES325_WRITE_VE_WIDTH, msg, 4, 1);
	}

	cmd_str[3] = 0x25; /* VEQ Noise Estimate Adj */
	cmd_str[7] = VEQ_adj_gain[volume];
	for (i = 0; i < sizeof(cmd_str)/4; i++) {
		memcpy(msg, cmd_str + (4 * i), 4);
		pr_debug("%s: write=0x%x, 0x%x, 0x%x, 0x%x\n", __func__, msg[0], msg[1], msg[2], msg[3]);
		ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
				ES325_WRITE_VE_WIDTH, msg, 4, 1);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(es325_set_VEQ_max_gain);

static int es325_ap_put_tx1_ch_cnt(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	es325_ap_tx1_ch_cnt = ucontrol->value.enumerated.item[0] + 1;
	pr_debug("GAC:%s():tx1 ch cnt = %d\n", __func__,
		es325_ap_tx1_ch_cnt);
	return 0;
}

static int es325_ap_get_tx1_ch_cnt(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("GAC:%s(): tx1 ch cnt = %d\n", __func__,
		es325_ap_tx1_ch_cnt);
	ucontrol->value.enumerated.item[0] = es325_internal_route_num - 1;
	return 0;
}

static const char * const es325_ap_tx1_ch_cnt_texts[] = {
	"One", "Two"
};
static const struct soc_enum es325_ap_tx1_ch_cnt_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(es325_ap_tx1_ch_cnt_texts),
			es325_ap_tx1_ch_cnt_texts);

/* generic gain translation */
static int es325_index_to_gain(int min, int step, int index)
{
	return	min + (step * index);
}
static int es325_gain_to_index(int min, int step, int gain)
{
	return	(gain - min) / step;
}

/* dereverb gain */
static int es325_put_dereverb_gain_value(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;
	int rc = 0;

	if (ucontrol->value.integer.value[0] <= 12) {
		pr_info("GAC:%s() ucontrol = %ld\n", __func__,
			ucontrol->value.integer.value[0]);
		value = es325_index_to_gain(-12, 1, ucontrol->value.integer.value[0]);
		pr_info("GAC:%s() value = %d\n", __func__, value);
		rc = es325_write(NULL, reg, value);
	}

	return 0;
}

static int es325_get_dereverb_gain_value(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;

	value = es325_read(NULL, reg);
	pr_info("GAC:%s() value = %d\n", __func__, value);
	ucontrol->value.integer.value[0] = es325_gain_to_index(-12, 1, value);
	pr_info("GAC:%s() ucontrol = %ld\n", __func__,
		ucontrol->value.integer.value[0]);

	return 0;
}

/* bwe high band gain */
static int es325_put_bwe_high_band_gain_value(struct snd_kcontrol *kcontrol,
					      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;
	int rc = 0;

	if (ucontrol->value.integer.value[0] <= 30) {
		pr_info("GAC:%s() ucontrol = %ld\n", __func__,
			ucontrol->value.integer.value[0]);
		value = es325_index_to_gain(-10, 1, ucontrol->value.integer.value[0]);
		pr_info("GAC:%s() value = %d\n", __func__,
			value);
		rc = es325_write(NULL, reg, value);
	}

	return 0;
}

static int es325_get_bwe_high_band_gain_value(struct snd_kcontrol *kcontrol,
					      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;

	value = es325_read(NULL, reg);
	pr_info("GAC:%s() value = %d\n", __func__, value);
	ucontrol->value.integer.value[0] = es325_gain_to_index(-10, 1, value);
	pr_info("GAC:%s() ucontrol = %ld\n", __func__,
		ucontrol->value.integer.value[0]);

	return 0;
}

/* bwe max snr */
static int es325_put_bwe_max_snr_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;
	int rc = 0;

	if (ucontrol->value.integer.value[0] <= 70) {
		pr_info("GAC:%s() ucontrol = %ld\n", __func__,
			ucontrol->value.integer.value[0]);
		value = es325_index_to_gain(-20, 1, ucontrol->value.integer.value[0]);
		pr_info("GAC:%s() value = %d\n", __func__,
			value);
		rc = es325_write(NULL, reg, value);
	}

	return 0;
}

static int es325_get_bwe_max_snr_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;

	value = es325_read(NULL, reg);
	pr_info("GAC:%s() value = %d\n", __func__, value);
	ucontrol->value.integer.value[0] = es325_gain_to_index(-20, 1, value);
	pr_info("GAC:%s() ucontrol = %ld\n", __func__,
		ucontrol->value.integer.value[0]);

	return 0;
}

/* csout gain */
static int es325_put_csout_gain_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct es325_priv *es325 = &es325_priv;
	char msg[4];
	static char cmd_str[4] = {0xa0, 0x15, 0x05, 0xA6}; /* FEIN  gain */
//	static char cmd_str[4] = {0xb0, 0x15, 0x10, 0xA6}; /* CS out gain */
//	static char cmd_str1[4] = {0xb0, 0x15, 0x11, 0xA6}; /* FEOUT1 gain */
//	static char cmd_str2[4] = {0xb0, 0x15, 0x12, 0xA6}; /* FEOUT2 gain */
	int ret;

	if (es325_priv.wakeup_cnt <= 0)
		return 0;

	memcpy(msg, cmd_str, 4);
	pr_debug("%s: write=0x%x, 0x%x, 0x%x, 0x%x\n", __func__, msg[0], msg[1], msg[2], msg[3]);
	ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
			ES325_WRITE_VE_WIDTH, msg, 4, 1);
	
/*	usleep_range(100, 100);
	memcpy(msg, cmd_str1, 4);
	pr_debug("%s: write=0x%x, 0x%x, 0x%x, 0x%x\n", __func__, msg[0], msg[1], msg[2], msg[3]);
	ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
			ES325_WRITE_VE_WIDTH, msg, 4, 1);*/
/*
	usleep_range(100, 100);
	memcpy(msg, cmd_str2, 4);
	pr_debug("%s: write=0x%x, 0x%x, 0x%x, 0x%x\n", __func__, msg[0], msg[1], msg[2], msg[3]);
	ret = es325_slim_write(es325, ES325_WRITE_VE_OFFSET,
			ES325_WRITE_VE_WIDTH, msg, 4, 1);*/
	es325->rx_is_muted = true;
	pr_info("=[ES325]=CS out gain set 0");		

	return 0;
}

static int es325_get_csout_gain_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;

	value = es325_read(NULL, reg);
	pr_info("GAC:%s() value = %d\n", __func__, value);
	ucontrol->value.integer.value[0] = es325_gain_to_index(-20, 1, value);
	pr_info("GAC:%s() ucontrol = %ld\n", __func__,
		ucontrol->value.integer.value[0]);

	return 0;
}

static const char * const es325_mic_config_texts[] = {
	"CT 2-mic", "FT 2-mic", "DV 1-mic", "EXT 1-mic", "BT 1-mic",
	"CT ASR 2-mic", "FT ASR 2-mic", "EXT ASR 1-mic", "FT ASR 1-mic",
};
static const struct soc_enum es325_mic_config_enum =
	SOC_ENUM_SINGLE(ES325_MIC_CONFIG, 0,
			ARRAY_SIZE(es325_mic_config_texts),
			es325_mic_config_texts);

static const char * const es325_aec_mode_texts[] = {
	"Off", "On", "rsvrd2", "rsvrd3", "rsvrd4", "On half-duplex"
};
static const struct soc_enum es325_aec_mode_enum =
	SOC_ENUM_SINGLE(ES325_AEC_MODE, 0, ARRAY_SIZE(es325_aec_mode_texts),
			es325_aec_mode_texts);

static const char * const es325_algo_rates_text[] = {
	"fs=8khz", "fs=16khz", "fs=24khz", "fs=48khz", "fs=96khz", "fs=192khz"
};
static const struct soc_enum es325_algo_sample_rate_enum =
	SOC_ENUM_SINGLE(ES325_ALGO_SAMPLE_RATE, 0,
			ARRAY_SIZE(es325_algo_rates_text),
			es325_algo_rates_text);
static const struct soc_enum es325_algo_mix_rate_enum =
	SOC_ENUM_SINGLE(ES325_MIX_SAMPLE_RATE, 0,
			ARRAY_SIZE(es325_algo_rates_text),
			es325_algo_rates_text);

static const char * const es325_algorithms_text[] = {
	"None", "VP", "Two CHREC", "AUDIO", "Four CHPASS"
};
static const struct soc_enum es325_algorithms_enum =
	SOC_ENUM_SINGLE(ES325_ALGO_SAMPLE_RATE, 0,
			ARRAY_SIZE(es325_algorithms_text),
			es325_algorithms_text);

static const char * const es325_off_on_texts[] = {
	"Off", "On"
};
static const struct soc_enum es325_veq_enable_enum =
	SOC_ENUM_SINGLE(ES325_VEQ_ENABLE, 0, ARRAY_SIZE(es325_off_on_texts),
			es325_off_on_texts);
static const struct soc_enum es325_dereverb_enable_enum =
	SOC_ENUM_SINGLE(ES325_DEREVERB_ENABLE, 0,
			ARRAY_SIZE(es325_off_on_texts),
			es325_off_on_texts);
static const struct soc_enum es325_bwe_enable_enum =
	SOC_ENUM_SINGLE(ES325_BWE_ENABLE, 0, ARRAY_SIZE(es325_off_on_texts),
			es325_off_on_texts);
static const struct soc_enum es325_bwe_post_eq_enable_enum =
	SOC_ENUM_SINGLE(ES325_BWE_POST_EQ_ENABLE, 0,
			ARRAY_SIZE(es325_off_on_texts),
			es325_off_on_texts);
static const struct soc_enum es325_algo_processing_enable_enum =
	SOC_ENUM_SINGLE(ES325_ALGO_PROCESSING, 0,
			ARRAY_SIZE(es325_off_on_texts),
			es325_off_on_texts);

static unsigned int es325_power_state;
static int es325_put_power_state_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s():ucontrol = %d\n", __func__,
		ucontrol->value.enumerated.item[0]);
	pr_info("GAC:%s():power state= %d\n", __func__,
		es325_power_state);

	if (es325_power_state == ucontrol->value.enumerated.item[0]) {
		pr_info("GAC:%s():no power state change\n", __func__);
		return 0;
	}

	es325_power_state = ucontrol->value.enumerated.item[0];
	if (es325_power_state)
		es325_wrapper_wakeup(0);
	else
		es325_wrapper_sleep(0);

	return 0;
}

static int es325_get_power_state_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): power state = %d\n", __func__,
		es325_power_state);
	ucontrol->value.enumerated.item[0] = es325_power_state;
	pr_info("%s(): ucontrol = %d\n", __func__,
		ucontrol->value.enumerated.item[0]);

	return 0;
}
static const char * const es325_power_state_texts[] = {
	"Sleep", "Active"
};
static const struct soc_enum es325_power_state_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(es325_power_state_texts),
			es325_power_state_texts);

static struct snd_kcontrol_new es325_digital_ext_snd_controls[] = {
	/* commit controls */
	SOC_SINGLE_EXT("ES325 RX1 Enable", SND_SOC_NOPM, 0, 1, 0,
		       es325_get_rx1_route_enable_value,
		       es325_put_rx1_route_enable_value),
	SOC_SINGLE_EXT("ES325 TX1 Enable", SND_SOC_NOPM, 0, 1, 0,
		       es325_get_tx1_route_enable_value,
		       es325_put_tx1_route_enable_value),
	SOC_SINGLE_EXT("ES325 RX2 Enable", SND_SOC_NOPM, 0, 1, 0,
		       es325_get_rx2_route_enable_value,
		       es325_put_rx2_route_enable_value),
	SOC_ENUM_EXT("Mic Config", es325_mic_config_enum,
		     es325_get_control_enum, es325_put_control_enum),
	SOC_ENUM_EXT("AEC Mode", es325_aec_mode_enum,
		     es325_get_control_enum, es325_put_control_enum),
	SOC_SINGLE_EXT("VEQ Enable", SND_SOC_NOPM,0,1,0,
			es325_get_VEQ_enable_control, es325_put_VEQ_enable_control),
	SOC_SINGLE_EXT("BWE Enable", SND_SOC_NOPM,0,1,0,
			es325_get_BWE_enable_control, es325_put_BWE_enable_control),
	SOC_SINGLE_EXT("ES325 Tx NS", SND_SOC_NOPM, 0, 100, 0,
				es325_get_Tx_NS_control, es325_put_Tx_NS_control),
	SOC_ENUM_EXT("Dereverb Enable", es325_dereverb_enable_enum,
		     es325_get_control_enum, es325_put_control_enum),
	SOC_SINGLE_EXT("Dereverb Gain",
		       ES325_DEREVERB_GAIN, 0, 100, 0,
		       es325_get_dereverb_gain_value, es325_put_dereverb_gain_value),
//	SOC_ENUM_EXT("BWE Enable", es325_bwe_enable_enum,
//		     es325_get_control_enum, es325_put_control_enum),
	SOC_SINGLE_EXT("BWE High Band Gain",
		       ES325_BWE_HIGH_BAND_GAIN, 0, 100, 0,
		       es325_get_bwe_high_band_gain_value,
		       es325_put_bwe_high_band_gain_value),
	SOC_SINGLE_EXT("BWE Max SNR",
		       ES325_BWE_MAX_SNR, 0, 100, 0,
		       es325_get_bwe_max_snr_value, es325_put_bwe_max_snr_value),
	SOC_ENUM_EXT("BWE Post EQ Enable", es325_bwe_post_eq_enable_enum,
		     es325_get_control_enum, es325_put_control_enum),
	SOC_SINGLE_EXT("SLIMbus Link Multi Channel",
		       ES325_SLIMBUS_LINK_MULTI_CHANNEL, 0, 65535, 0,
		       es325_get_control_value, es325_put_control_value),
	SOC_ENUM_EXT("Set Power State", es325_power_state_enum,
		       es325_get_power_state_enum, es325_put_power_state_enum),
	SOC_ENUM_EXT("Algorithm Processing", es325_algo_processing_enable_enum,
		     es325_get_control_enum, es325_put_control_enum),
	SOC_ENUM_EXT("Algorithm Sample Rate", es325_algo_sample_rate_enum,
		     es325_get_control_enum, es325_put_control_enum),
	SOC_ENUM_EXT("Algorithm", es325_algorithms_enum,
		     es325_get_control_enum, es325_put_control_enum),
	SOC_ENUM_EXT("Mix Sample Rate", es325_algo_mix_rate_enum,
		     es325_get_control_enum, es325_put_control_enum),
	SOC_SINGLE_EXT("Internal Route Config",
		       SND_SOC_NOPM, 0, 100, 0, es325_get_internal_route_config,
		       es325_put_internal_route_config),
	SOC_ENUM_EXT("ES325-AP Tx Channels", es325_ap_tx1_ch_cnt_enum,
		     es325_ap_get_tx1_ch_cnt, es325_ap_put_tx1_ch_cnt),
	SOC_SINGLE_EXT("ES325 Network Type",
		SND_SOC_NOPM, 0, 1, 0, es325_get_network_type,
		es325_put_network_type),
	SOC_SINGLE_EXT("CS out gain",
		       ES325_DIGITAL_GAIN_CSOUT, 0, 100, 0,
		       es325_get_csout_gain_value, es325_put_csout_gain_value),
	SOC_SINGLE_EXT("ES325 2Mic Enable",
		SND_SOC_NOPM, 0, 1, 0, es325_get_2mic_enable,
		es325_put_2mic_enable)
};

/*
static struct snd_kcontrol_new es325_digital_snd_controls[] = {
	\/\* commit controls \*\/
	SOC_SINGLE("Mic Config",
		   ES325_MIC_CONFIG, 0, 8, 0),
	SOC_SINGLE("AEC Mode",
		   ES325_AEC_MODE, 0, 6, 0),
	SOC_SINGLE("TX AGC",
		   ES325_TX_AGC, 0, 65535, 0),
	SOC_SINGLE("TX AGC Target Level",
		   ES325_TX_AGC_TARGET_LEVEL, 0, 65535, 0),
	SOC_SINGLE("TX AGC Noise Floor",
		   ES325_TX_AGC_NOISE_FLOOR, 0, 65535, 0),
	SOC_SINGLE("TX AGC SNR Improvement",
		   ES325_TX_AGC_SNR_IMPROVEMENT, 0, 4, 0),
	SOC_SINGLE("VEQ Enable",
		   ES325_VEQ_ENABLE, 0, 65535, 0),
	SOC_SINGLE("RX Out Limiter Max Level",
		   ES325_RX_OUT_LIMITER_MAX_LEVEL, 0, 65535, 0),
	SOC_SINGLE("RX Noise Suppress",
		   ES325_RX_NOISE_SUPPRESS, 0, 65535, 0),
	SOC_SINGLE("RX STS",
		   ES325_RX_STS, 0, 65535, 0),
	SOC_SINGLE("RX STS Rate",
		   ES325_RX_STS_RATE, 0, 65535, 0),
	SOC_SINGLE("AEC Speaker Volume",
		   ES325_AEC_SPEAKER_VOLUME, 0, 65535, 0),
	SOC_SINGLE("Sidetone",
		   ES325_SIDETONE, 0, 2, 0),
	SOC_SINGLE("Sidetone Gain",
		   ES325_SIDETONE_GAIN, 0, 65535, 0),
	SOC_SINGLE("TX Comfort Noise",
		   ES325_TX_COMFORT_NOISE, 0, 1, 0),
	SOC_SINGLE("TX Comfort Noise Level",
		   ES325_TX_COMFORT_NOISE_LEVEL, 0, 65535, 0),
	SOC_SINGLE("Algorithm Reset",
		   ES325_ALGORITHM_RESET, 0, 31, 0),
	SOC_SINGLE("RX Post EQ",
		   ES325_RX_POST_EQ, 0, 65535, 0),
	SOC_SINGLE("TX Post EQ",
		   ES325_TX_POST_EQ, 0, 65535, 0),
	SOC_SINGLE("AEC CNG",
		   ES325_AEC_CNG, 0, 1, 0),
	SOC_SINGLE("VEQ Nosie Est Adjustment",
		   ES325_VEQ_NOISE_ESTIMATION_ADJUSTMENT, 0, 65535, 0),
	SOC_SINGLE("TX AGC Slew Rate Up",
		   ES325_TX_AGC_SLEW_RATE_UP, 0, 65535, 0),
	SOC_SINGLE("TX AGC Slew Rate Down",
		   ES325_TX_AGC_SLEW_RATE_DOWN, 0, 65535, 0),
	SOC_SINGLE("RX AGC",
		   ES325_RX_AGC, 0, 65535, 0),
	SOC_SINGLE("RX AGC Target Level",
		   ES325_RX_AGC_TARGET_LEVEL, 0, 65535, 0),
	SOC_SINGLE("RX AGC Noise Floor",
		   ES325_RX_AGC_NOISE_FLOOR, 0, 65535, 0),
	SOC_SINGLE("RX AGC SNR Improvement",
		   ES325_RX_AGC_SNR_IMPROVEMENT, 0, 4, 0),
	SOC_SINGLE("RX AGC Slew Rate Up",
		   ES325_RX_AGC_SLEW_RATE_UP, 0, 65535, 0),
	SOC_SINGLE("RX AGC Slew Rate Down",
		   ES325_RX_AGC_SLEW_RATE_DOWN, 0, 65535, 0),
	SOC_SINGLE("AEC CNG Gain",
		   ES325_AEC_CNG_GAIN, 0, 65535, 0),
	SOC_SINGLE("TX MBC",
		   ES325_TX_MBC, 0, 2, 0),
	SOC_SINGLE("RX MBC",
		   ES325_RX_MBC, 0, 2, 0),
	SOC_SINGLE("AEC ESE",
		   ES325_AEC_ESE, 0, 65535, 0),
	SOC_SINGLE("TX NS Adaptation Speed",
		   ES325_TX_NS_ADAPTATION_SPEED, 0, 2, 0),
	SOC_SINGLE("TX SNR Est",
		   ES325_TX_SNR_ESTIMATE, 0, 1, 0),
	SOC_SINGLE("VEQ Max Gain",
		   ES325_VEQ_MAX_GAIN, 0, 15, 0),
	SOC_SINGLE("TX AGC Guardband",
		   ES325_TX_AGC_GUARDBAND, 0, 15, 0),
	SOC_SINGLE("RX AGC Guardband",
		   ES325_RX_AGC_GUARDBAND, 0, 15, 0),
	SOC_SINGLE("TX Out Limiter Max Level",
		   ES325_TX_OUT_LIMITER_MAX_LEVEL, 0, 65535, 0),
	SOC_SINGLE("TX In Limiter Max Level",
		   ES325_TX_IN_LIMITER_MAX_LEVEL, 0, 65535, 0),
	SOC_SINGLE("RX NS Adaptation Speed",
		   ES325_RX_NS_ADAPTATION_SPEED, 0, 2, 0),
	SOC_SINGLE("AEC Variable Echo Ref Delay",
		   ES325_AEC_VARIABLE_ECHO_REF_DELAY, 0, 65535, 0),
	SOC_SINGLE("TX Noise Suppress level",
		   ES325_TX_NOISE_SUPPRESS_LEVEL, 0, 65535, 0),
	SOC_SINGLE("RX Noise Suppress level",
		   ES325_RX_NOISE_SUPPRESS_LEVEL, 0, 65535, 0),
	SOC_SINGLE("RX CNG",
		   ES325_RX_CNG, 0, 1, 0),
	SOC_SINGLE("RX CNG Gain",
		   ES325_RX_CNG_GAIN, 0, 65535, 0),
	SOC_SINGLE("TX AGC Max Gain",
		   ES325_TX_AGC_MAX_GAIN, 0, 65535, 0),
	SOC_SINGLE("RX AGC Max Gain",
		   ES325_RX_AGC_MAX_GAIN, 0, 45, 0),
	SOC_SINGLE("Avalon API Version Lo",
		   ES325_AVALON_API_VERSION_LO, 0, 65535, 0),
	SOC_SINGLE("Avalon API Version Hi",
		   ES325_AVALON_API_VERSION_HI, 0, 65535, 0),
	SOC_SINGLE("Avalon Enable AV Processor",
		   ES325_AVALON_AV_PROCESSOR, 0, 1, 0),
	SOC_SINGLE("Avalon AV Config",
		   ES325_AVALON_AV_CONFIG, 0, 6, 0),
	SOC_SINGLE("Avalon EQ Preset",
		   ES325_AVALON_EQ_PRESET, 0, 14, 0),
	SOC_SINGLE("Avalon Stereo Width",
		   ES325_AVALON_STEREO_WIDTH, 0, 65535, 0),
	SOC_SINGLE("Avalon AV Digital Out Gain",
		   ES325_AVALON_AV_DIGITAL_OUT_GAIN, 0, 65535, 0),
	SOC_SINGLE("Avalon Enable TdMBC",
		   ES325_AVALON_TDMBC, 0, 1, 0),
	SOC_SINGLE("Avalong AV Out Limit",
		   ES325_AVALON_AV_OUT_LIMIT, 0, 65535, 0),
	SOC_SINGLE("Avalon Enable Stereo Widening",
		   ES325_AVALON_STEREO_WIDENING, 0, 1, 0),
	SOC_SINGLE("Avalon Enable stat NS",
		   ES325_AVALON_STAT_NS, 0, 1, 0),
	SOC_SINGLE("Avalon Stat NS Suppress",
		   ES325_AVALON_STAT_NS_SUPPRESS, 0, 65535, 0),
	SOC_SINGLE("Avalon Stat NS Adaptation Speed",
		   ES325_AVALON_STAT_NS_ADAP_SPEED, 0, 3, 0),
	SOC_SINGLE("Avalon Stat NS Mode",
		   ES325_AVALON_STAT_NS_MODE, 0, 3, 0),
	SOC_SINGLE("Avalon Stat NS Max Noise Energy",
		   ES325_AVLALON_STAT_NS_MAX_NOISE_ENERGY, 0, 65535, 0),
	SOC_SINGLE("Avalon Enable VBB",
		   ES325_AVALON_VBB, 0, 1, 0),
	SOC_SINGLE("Avalon VBB Strength",
		   ES325_AVALON_VBB_STRENGTH, 0, 100, 0),
	SOC_SINGLE("Avalon EQ Mode",
		   ES325_AVALON_EQ_MODE, 0, 2, 0),
	SOC_SINGLE("Avalon EQ Graphic Band 1 Gain",
		   ES325_AVALON_EQ_GRAPHIC_BAND1_GAIN, 0, 65535, 0),
	SOC_SINGLE("Avalon EQ Graphic Band 2 Gain",
		   ES325_AVALON_EQ_GRAPHIC_BAND2_GAIN, 0, 65535, 0),
	SOC_SINGLE("Avalon EQ Graphic Band 3 Gain",
		   ES325_AVALON_EQ_GRAPHIC_BAND3_GAIN, 0, 65535, 0),
	SOC_SINGLE("Avalon EQ Graphic Band 4 Gain",
		   ES325_AVALON_EQ_GRAPHIC_BAND4_GAIN, 0, 65535, 0),
	SOC_SINGLE("Avalon EQ Graphic Band 5 Gain",
		   ES325_AVALON_EQ_GRAPHIC_BAND5_GAIN, 0, 65535, 0),
	SOC_SINGLE("Avalon EQ Graphic Band 6 Gain",
		   ES325_AVALON_EQ_GRAPHIC_BAND6_GAIN, 0, 65535, 0),
	SOC_SINGLE("Avalon EQ Graphic Band 7 Gain",
		   ES325_AVALON_EQ_GRAPHIC_BAND7_GAIN, 0, 65535, 0),
	SOC_SINGLE("Avalon EQ Graphic Band 8 Gain",
		   ES325_AVALON_EQ_GRAPHIC_BAND8_GAIN, 0, 65535, 0),
	SOC_SINGLE("Avalon EQ Graphic Band 9 Gain",
		   ES325_AVALON_EQ_GRAPHIC_BAND9_GAIN, 0, 65535, 0),
	SOC_SINGLE("Avalon EQ Graphic Band 10 Gain",
		   ES325_AVALON_EQ_GRAPHIC_BAND10_GAIN, 0, 65535, 0),
	SOC_SINGLE("Avalon TdDRC Enable",
		   ES325_AVALON_TDDRC, 0, 1, 0),
	SOC_SINGLE("Avalon TdDRC Strength",
		   ES325_AVALON_TDDRC_STRENGTH, 0, 100, 0),
	SOC_SINGLE("Avalon Enable Limiter",
		   ES325_AVALON_LIMITER, 0, 1, 0),
	SOC_SINGLE("Avalon Enable EQ",
		   ES325_AVALON_EQ, 0, 1, 0),
	SOC_SINGLE("DirAC Enable",
		   ES325_DIRAC, 0, 1, 0),
	SOC_SINGLE("DirAC Out Headroom Limiter",
		   ES325_DIRAC_OUT_HEADROOM_LIMITER, 0, 65535, 0),
	SOC_SINGLE("DirAC Mode",
		   ES325_DIRAC_MODE, 0, 2, 0),
	SOC_SINGLE("DirAC In Headroom Limiter",
		   ES325_DIRAC_IN_HEADROOM_LIMITER, 0, 65535, 0),
	SOC_SINGLE("DirAC Comfort Noise ",
		   ES325_DIRAC_COMFORT_NOISE, 0, 1, 0),
	SOC_SINGLE("DirAC Comfort Noise level",
		   ES325_DIRAC_COMFORT_NOISE_LEVEL, 0, 65535, 0),
	SOC_SINGLE("DirAC Narrator VQOS",
		   ES325_DIRAC_NARRATOR_VQOS, 0, 15, 0),
	SOC_SINGLE("DirAC Narrator Position Suppress",
		   ES325_DIRAC_NARRATOR_POSITION_SUPPRESS, 0, 7, 0),
	SOC_SINGLE("DirAC Narrator Use AGC Out",
		   ES325_DIRAC_NARRATOR_AGC_OUT, 0, 1, 0),
	SOC_SINGLE("DirAC Narrator AGC Speech Target",
		   ES325_DIRAC_NARRATOR_AGC_SPEECH_TARGET, 0, 65535, 0),
	SOC_SINGLE("DirAC Narrator AGC SNR Improve",
		   ES325_DIRAC_NARRATOR_AGC_SNR_IMPROVE, 0, 4, 0),
	SOC_SINGLE("DirAC Narrator AGC Noise Floor",
		   ES325_DIRAC_NARRATOR_AGC_NOISE_FLOOR, 0, 65535, 0),
	SOC_SINGLE("DirAC Narrator AGC Max Gain",
		   ES325_DIRAC_NARRATOR_AGC_MAX_GAIN, 0, 45, 0),
	SOC_SINGLE("DirAC Narrator AGC Up Rate",
		   ES325_DIRAC_NARRATOR_AGC_UP_RATE, 0, 50, 0),
	SOC_SINGLE("DirAC Narrator AGC Down Rate",
		   ES325_DIRAC_NARRATOR_AGC_DOWN_RATE, 0, 50, 0),
	SOC_SINGLE("DirAC Narrator AGC Guardband",
		   ES325_DIRAC_NARRATOR_AGC_GUARDBAND, 0, 15, 0),
	SOC_SINGLE("DirAC Narrator Post EQ Mode",
		   ES325_DIRAC_NARRATOR_POST_EQ_MODE, 0, 1, 0),
	SOC_SINGLE("DirAC Narrator MBC Mode",
		   ES325_DIRAC_NARRATOR_MBC_MODE, 0, 1, 0),
	SOC_SINGLE("DirAC Scene Beam Width Deg",
		   ES325_DIRAC_SCENE_BEAM_WIDTH, 0, 90, 0),
	SOC_SINGLE("DirAC Scene Use AGC Out",
		   ES325_DIRAC_SCENE_AGC_OUT, 0, 1, 0),
	SOC_SINGLE("DirAC Scene AGC Speech Target",
		   ES325_DIRAC_SCENE_AGC_SPEECH_TARGET, 0, 65535, 0),
	SOC_SINGLE("DirAC Scene AGC SNR Improve",
		   ES325_DIRAC_SCENE_AGC_SNR_IMPROVE, 0, 4, 0),
	SOC_SINGLE("DirAC Scene AGC Noise Floor",
		   ES325_DIRAC_SCENE_AGC_NOISE_FLOOR, 0, 65535, 0),
	SOC_SINGLE("DirAC Scene AGC Max Gain",
		   ES325_DIRAC_SCENE_AGC_MAX_GAIN, 0, 45, 0),
	SOC_SINGLE("DirAC Scene AGC Up Rate",
		   ES325_DIRAC_SCENE_AGC_UP_RATE, 0, 50, 0),
	SOC_SINGLE("DirAC Scene AGC Down Rate",
		   ES325_DIRAC_SCENE_AGC_DOWN_RATE, 0, 50, 0),
	SOC_SINGLE("DirAC Scene AGC Guardband",
		   ES325_DIRAC_SCENE_AGC_GUARDBAND, 0, 15, 0),
	SOC_SINGLE("DirAC Scene VQOS",
		   ES325_DIRAC_SCENE_VQOS, 0, 15, 0),
	SOC_SINGLE("DirACC Scene Post EQ Mode",
		   ES325_DIRAC_SCENE_POST_EQ_MODE, 0, 1, 0),
	SOC_SINGLE("DirAC Scene MBC Mode",
		   ES325_DIRAC_SCENE_MBC_MODE, 0, 1, 0),
	SOC_SINGLE("Tone Param API Version Lo",
		   ES325_TONE_PARAM_API_VERSION_LO, 0, 65535, 0),
	SOC_SINGLE("Tone Param API Version Hi",
		   ES325_TONE_PARAM_API_VERSION_HI, 0, 65535, 0),
	SOC_SINGLE("Tone Param Enable Beep Sys",
		   ES325_TONE_PARAM_ENABLE_BEEP_SYS, 0, 1, 0),
	SOC_SINGLE("Tone Param Enable Gen Beep",
		   ES325_TONE_PARAM_ENABLE_GEN_BEEP, 0, 1, 0),
	SOC_SINGLE("Tone Param Gen Beep On",
		   ES325_TONE_PARAM_GEN_BEEP_ON, 0, 1, 0),
	SOC_SINGLE("Tone Param Gen Beep Freq1",
		   ES325_TONE_PARAM_GEN_BEEP_FREQ1, 0, 20000, 0),
	SOC_SINGLE("Tone Param Gen Beep Freq2",
		   ES325_TONE_PARAM_GEN_BEEP_FREQ2, 0, 20000, 0),
	SOC_SINGLE("Tone Param Gen Beep Pan LR",
		   ES325_TONE_PARAM_GEN_BEEP_PAN_LR, 0, 65535, 0),
	SOC_SINGLE("Tone Param Gen Beep Gain",
		   ES325_TONE_PARAM_GEN_BEEP_GAIN, 0, 65535, 0),
	SOC_SINGLE("Dereverb Enable",
		   ES325_DEREVERB_ENABLE, 0, 1, 0),
	SOC_SINGLE("Dereverb Gain",
		   ES325_DEREVERB_GAIN, 0, 65535, 0),
	SOC_SINGLE("BWE Enable",
		   ES325_BWE_ENABLE, 0, 1, 0),
	SOC_SINGLE("BWE High Band Gain",
		   ES325_BWE_HIGH_BAND_GAIN, 0, 65535, 0),
	SOC_SINGLE("BWE Max SNR",
		   ES325_BWE_MAX_SNR, 0, 65535, 0),
	SOC_SINGLE("BWE Post EQ Enable",
		   ES325_BWE_MAX_SNR, 0, 1, 0),
	SOC_SINGLE("PortA Word Length",
		   ES325_PORTA_WORD_LEN, 0, 31, 0),
	SOC_SINGLE("PortA TDM Slots Per Frame",
		   ES325_PORTA_TDM_SLOTS_PER_FRAME, 0, 7, 0),
	SOC_SINGLE("PortA Delay From FS TX",
		   ES325_PORTA_TX_DELAY_FROM_FS, 0, 31, 0),
	SOC_SINGLE("PortA Delay From FS RX",
		   ES325_PORTA_RX_DELAY_FROM_FS, 0, 31, 0),
	SOC_SINGLE("PortA Latch Edge",
		   ES325_PORTA_LATCH_EDGE, 0, 3, 0),
	SOC_SINGLE("PortA Endian",
		   ES325_PORTA_ENDIAN, 0, 1, 0),
	SOC_SINGLE("PortA Tristate Enable",
		   ES325_PORTA_TRISTATE, 0, 1, 0),
	SOC_SINGLE("PortA Audio Port Mode",
		   ES325_PORTA_AUDIO_PORT_MODE, 0, 1, 0),
	SOC_SINGLE("PortA TDM Enabled",
		   ES325_PORTA_TDM_ENABLED, 0, 1, 0),
	SOC_SINGLE("PortA Clock Control",
		   ES325_PORTA_CLOCK_CONTROL, 0, 12736, 0),
	SOC_SINGLE("PortA Data Justification",
		   ES325_PORTA_DATA_JUSTIFICATION, 0, 1, 0),
	SOC_SINGLE("PortA FS Duration",
		   ES325_PORTA_FS_DURATION, 0, 255, 0),
	SOC_SINGLE("PortB Word Length",
		   ES325_PORTB_WORD_LEN, 0, 31, 0),
	SOC_SINGLE("PortB TDM Slots Per Frame",
		   ES325_PORTB_TDM_SLOTS_PER_FRAME, 0, 7, 0),
	SOC_SINGLE("PortB Delay From FS TX",
		   ES325_PORTB_TX_DELAY_FROM_FS, 0, 31, 0),
	SOC_SINGLE("PortB Delay From FS RX",
		   ES325_PORTB_RX_DELAY_FROM_FS, 0, 31, 0),
	SOC_SINGLE("PortB Latch Edge",
		   ES325_PORTB_LATCH_EDGE, 0, 3, 0),
	SOC_SINGLE("PortB Endian",
		   ES325_PORTB_ENDIAN, 0, 1, 0),
	SOC_SINGLE("PortB Tristate Enable",
		   ES325_PORTB_TRISTATE, 0, 1, 0),
	SOC_SINGLE("PortB Audio Port Mode",
		   ES325_PORTB_AUDIO_PORT_MODE, 0, 1, 0),
	SOC_SINGLE("PortB TDM Enabled",
		   ES325_PORTB_TDM_ENABLED, 0, 1, 0),
	SOC_SINGLE("PortB Clock Control",
		   ES325_PORTB_CLOCK_CONTROL, 0, 12736, 0),
	SOC_SINGLE("PortB Data Justification",
		   ES325_PORTB_DATA_JUSTIFICATION, 0, 1, 0),
	SOC_SINGLE("PortB FS Duration",
		   ES325_PORTB_FS_DURATION, 0, 255, 0),
	SOC_SINGLE("PortC Word Length",
		   ES325_PORTC_WORD_LEN, 0, 31, 0),
	SOC_SINGLE("PortC TDM Slots Per Frame",
		   ES325_PORTC_TDM_SLOTS_PER_FRAME, 0, 7, 0),
	SOC_SINGLE("PortC Delay From FS TX",
		   ES325_PORTC_TX_DELAY_FROM_FS, 0, 31, 0),
	SOC_SINGLE("PortC Delay From FS RX",
		   ES325_PORTC_RX_DELAY_FROM_FS, 0, 31, 0),
	SOC_SINGLE("PortC Latch Edge",
		   ES325_PORTC_LATCH_EDGE, 0, 3, 0),
	SOC_SINGLE("PortC Endian",
		   ES325_PORTC_ENDIAN, 0, 1, 0),
	SOC_SINGLE("PortC Tristate Enable",
		   ES325_PORTC_TRISTATE, 0, 1, 0),
	SOC_SINGLE("PortC Audio Port Mode",
		   ES325_PORTC_AUDIO_PORT_MODE, 0, 1, 0),
	SOC_SINGLE("PortC TDM Enabled",
		   ES325_PORTC_TDM_ENABLED, 0, 1, 0),
	SOC_SINGLE("PortC Clock Control",
		   ES325_PORTC_CLOCK_CONTROL, 0, 12736, 0),
	SOC_SINGLE("PortC Data Justification",
		   ES325_PORTC_DATA_JUSTIFICATION, 0, 1, 0),
	SOC_SINGLE("PortC FS Duration",
		   ES325_PORTC_FS_DURATION, 0, 255, 0),
	SOC_SINGLE("PortD Word Length",
		   ES325_PORTD_WORD_LEN, 0, 31, 0),
	SOC_SINGLE("PortD TDM Slots Per Frame",
		   ES325_PORTD_TDM_SLOTS_PER_FRAME, 0, 7, 0),
	SOC_SINGLE("PortD Delay From FS TX",
		   ES325_PORTD_TX_DELAY_FROM_FS, 0, 31, 0),
	SOC_SINGLE("PortD Delay From FS RX",
		   ES325_PORTD_RX_DELAY_FROM_FS, 0, 31, 0),
	SOC_SINGLE("PortD Latch Edge",
		   ES325_PORTD_LATCH_EDGE, 0, 3, 0),
	SOC_SINGLE("PortD Endian",
		   ES325_PORTD_ENDIAN, 0, 1, 0),
	SOC_SINGLE("PortD Tristate Enable",
		   ES325_PORTD_TRISTATE, 0, 1, 0),
	SOC_SINGLE("PortD Audio Port Mode",
		   ES325_PORTD_AUDIO_PORT_MODE, 0, 1, 0),
	SOC_SINGLE("PortD TDM Enabled",
		   ES325_PORTD_TDM_ENABLED, 0, 1, 0),
	SOC_SINGLE("PortD Clock Control",
		   ES325_PORTD_CLOCK_CONTROL, 0, 12736, 0),
	SOC_SINGLE("PortD Data Justification",
		   ES325_PORTD_DATA_JUSTIFICATION, 0, 1, 0),
	SOC_SINGLE("PortD FS Duration",
		   ES325_PORTD_FS_DURATION, 0, 255, 0),
	SOC_SINGLE("SLIMbus Link Multi Channel",
		   ES325_SLIMBUS_LINK_MULTI_CHANNEL, 0, 65535, 0),
	\/\* SOC_SINGLE("Set Power State", \*\/
	\/\*	ES325_POWER_STATE, 0, 1, 0), \*\/
	SOC_SINGLE("Streaming",
		   ES325_STREAMING, 0, 5, 0),
	SOC_SINGLE("Far End Streaming",
		   ES325_FE_STREAMING, 0, 8, 0),
	\/\* SOC_SINGLE("Preset", \*\/
	\/\*	ES325_PRESET, 0, 15, 0), \*\/
	SOC_SINGLE("Algorithm Statistics",
		   ES325_ALGO_STATS, 0, 15, 0),
	SOC_SINGLE("Algorithm Processing",
		   ES325_ALGO_PROCESSING, 0, 1, 0),
	SOC_SINGLE("Algorithm Sample Rate",
		   ES325_ALGO_SAMPLE_RATE, 0, 5, 0),
	SOC_SINGLE("Smooth Rate",
		   ES325_SMOOTH_RATE, 0, 65535, 0),
	SOC_SINGLE("Change Status",
		   ES325_CHANGE_STATUS, 0, 4, 0),
	SOC_SINGLE("Digital Pass Through",
		   ES325_DIGITAL_PASS_THROUGH, 0, 65535, 0),
	SOC_SINGLE("Data Path",
		   ES325_DATA_PATH, 0, 65535, 0),
	SOC_SINGLE("Algorithm",
		   ES325_ALGORITHM, 0, 4, 0),
	SOC_SINGLE("Mix Sample Rate",
		   ES325_MIX_SAMPLE_RATE, 0, 5, 0),
	SOC_SINGLE("Sig RMS PortA DIN Left",
		   ES325_SIGNAL_RMS_PORTA_DIN_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortA DIN Right",
		   ES325_SIGNAL_RMS_PORTA_DIN_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortA DOUT Left",
		   ES325_SIGNAL_RMS_PORTA_DOUT_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortA DOUT Right",
		   ES325_SIGNAL_RMS_PORTA_DOUT_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortB DIN Left",
		   ES325_SIGNAL_RMS_PORTB_DIN_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortB DIN Right",
		   ES325_SIGNAL_RMS_PORTB_DIN_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortB DOUT Left",
		   ES325_SIGNAL_RMS_PORTB_DOUT_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortB DOUT Right",
		   ES325_SIGNAL_RMS_PORTB_DOUT_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortC DIN Left",
		   ES325_SIGNAL_RMS_PORTC_DIN_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortC DIN Right",
		   ES325_SIGNAL_RMS_PORTC_DIN_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortC DOUT Left",
		   ES325_SIGNAL_RMS_PORTC_DOUT_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortC DOUT Right",
		   ES325_SIGNAL_RMS_PORTC_DOUT_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortD DIN Left",
		   ES325_SIGNAL_RMS_PORTD_DIN_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortD DIN Right",
		   ES325_SIGNAL_RMS_PORTD_DIN_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortD DOUT Left",
		   ES325_SIGNAL_RMS_PORTD_DOUT_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig RMS PortD DOUT Right",
		   ES325_SIGNAL_RMS_PORTD_DOUT_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortA DIN Left",
		   ES325_SIGNAL_PEAK_PORTA_DIN_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortA DIN Right",
		   ES325_SIGNAL_PEAK_PORTA_DIN_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortA DOUT Left",
		   ES325_SIGNAL_PEAK_PORTA_DOUT_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortA DOUT Right",
		   ES325_SIGNAL_PEAK_PORTA_DOUT_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortB DIN Left",
		   ES325_SIGNAL_PEAK_PORTB_DIN_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortB DIN Right",
		   ES325_SIGNAL_PEAK_PORTB_DIN_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortB DOUT Left",
		   ES325_SIGNAL_PEAK_PORTB_DOUT_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortB DOUT Right",
		   ES325_SIGNAL_PEAK_PORTB_DOUT_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortC DIN Left",
		   ES325_SIGNAL_PEAK_PORTC_DIN_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortC DIN Right",
		   ES325_SIGNAL_PEAK_PORTC_DIN_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortC DOUT Left",
		   ES325_SIGNAL_PEAK_PORTC_DOUT_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortC DOUT Right",
		   ES325_SIGNAL_PEAK_PORTC_DOUT_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortD DIN Left",
		   ES325_SIGNAL_PEAK_PORTD_DIN_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortD DIN Right",
		   ES325_SIGNAL_PEAK_PORTD_DIN_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortD DOUT Left",
		   ES325_SIGNAL_PEAK_PORTD_DOUT_LEFT, 0, 65535, 0),
	SOC_SINGLE("Sig Peak PortD DOUT Right",
		   ES325_SIGNAL_PEAK_PORTD_DOUT_RIGHT, 0, 65535, 0),
	SOC_SINGLE("Digital Gain Primary Volume",
		   ES325_DIGITAL_GAIN_PRIMARY, 0, 255, 0),
	SOC_SINGLE("Digital Gain Secondary Volume",
		   ES325_DIGITAL_GAIN_SECONDARY, 0, 255, 0),
	SOC_SINGLE("Digital Gain Tertiary Volume",
		   ES325_DIGITAL_GAIN_TERTIARY, 0, 255, 0),
	SOC_SINGLE("Digital Gain Quad Volume",
		   ES325_DIGITAL_GAIN_QUAD, 0, 255, 0),
	SOC_SINGLE("Digital Gain FEIN Volume",
		   ES325_DIGITAL_GAIN_FEIN, 0, 255, 0),
	SOC_SINGLE("Digital Gain AUDIN1 Volume",
		   ES325_DIGITAL_GAIN_AUDIN1, 0, 255, 0),
	SOC_SINGLE("Digital Gain AUDIN2 Volume",
		   ES325_DIGITAL_GAIN_AUDIN2, 0, 255, 0),
	SOC_SINGLE("Digital Gain AUDIN3 Volume",
		   ES325_DIGITAL_GAIN_AUDIN3, 0, 255, 0),
	SOC_SINGLE("Digital Gain AUDIN4 Volume",
		   ES325_DIGITAL_GAIN_AUDIN4, 0, 255, 0),
	SOC_SINGLE("Digital Gain UITONE1 Volume",
		   ES325_DIGITAL_GAIN_UITONE1, 0, 255, 0),
	SOC_SINGLE("Digital Gain UITONE2 Volume",
		   ES325_DIGITAL_GAIN_UITONE2, 0, 255, 0),
	SOC_SINGLE("Digital Gain CSOUT Volume",
		   ES325_DIGITAL_GAIN_CSOUT, 0, 255, 0),
	SOC_SINGLE("Digital Gain FEOUT1 Volume",
		   ES325_DIGITAL_GAIN_FEOUT1, 0, 255, 0),
	SOC_SINGLE("Digital Gain FEOUT2 Volume",
		   ES325_DIGITAL_GAIN_FEOUT2, 0, 255, 0),
	SOC_SINGLE("Digital Gain AUDOUT1 Volume",
		   ES325_DIGITAL_GAIN_AUDOUT1, 0, 255, 0),
	SOC_SINGLE("Digital Gain AUDOUT2 Volume",
		   ES325_DIGITAL_GAIN_AUDOUT2, 0, 255, 0),
	SOC_SINGLE("Digital Gain AUDOUT3 Volume",
		   ES325_DIGITAL_GAIN_AUDOUT3, 0, 255, 0),
	SOC_SINGLE("Digital Gain AUDOUT4 Volume",
		   ES325_DIGITAL_GAIN_AUDOUT4, 0, 255, 0),
	SOC_SINGLE("PortA timing",
		   ES325_PORTA_TIMING, 0, 15, 0),
	SOC_SINGLE("PortB timing",
		   ES325_PORTB_TIMING, 0, 15, 0),
	SOC_SINGLE("PortC timing",
		   ES325_PORTC_TIMING, 0, 15, 0),
	SOC_SINGLE("PortD timing",
		   ES325_PORTD_TIMING, 0, 15, 0),
	SOC_SINGLE("Flush",
		   ES325_FLUSH, 0, 0, 0),

	SOC_SINGLE_EXT("Internal Route Config",
		       SND_SOC_NOPM, 0, 0, 6, es325_put_internal_route_config,
		       es325_get_internal_route_config)
};
*/

static int es325_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	int ret = 0;

	/* dev_dbg(codec->dev, "%s(codec, level = 0x%04x): entry\n", __func__, level); */

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		break;

	case SND_SOC_BIAS_OFF:
		break;
	}
	codec->dapm.bias_level = level;

	/* dev_dbg(codec->dev, "%s(): exit\n", __func__); */
	return ret;
}

#if defined(CONFIG_SND_SOC_ES325_I2S)
static int es325_i2s_set_sysclk(struct snd_soc_dai *dai, int clk_id,
				unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}

static int es325_i2s_set_pll(struct snd_soc_dai *dai, int pll_id,
			     int source, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}

static int es325_i2s_set_clkdiv(struct snd_soc_dai *dai, int div_id,
				int div)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}

static int es325_i2s_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}

static int es325_i2s_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
				  unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}

static int es325_i2s_set_channel_map(struct snd_soc_dai *dai,
	unsigned int tx_num, unsigned int *tx_slot, unsigned int rx_num,
	unsigned int *rx_slot)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}

static int es325_i2s_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int paramid = 0;
	unsigned int val = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	switch (dai->id) {
	case 0:
		break;
	case 1:
		break;
	default:
		return -EINVAL;
	}

	if (tristate)
		val = 0x0001;
	else
		val = 0x0000;

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return snd_soc_write(codec, paramid, val);
}

static int es325_i2s_port_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int paramid = 0;
	unsigned int val = 0;

	/* Is this valid since DACs are not statically mapped to DAIs? */
	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);
	switch (dai->id) {
	case 0:
		break;
	case 1:
		break;
	default:
		return -EINVAL;
	}

	if (mute)
		val = 0x0000;
	else
		val = 0x0001;

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return snd_soc_write(codec, paramid, val);
}

static int es325_i2s_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);


	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}

static void es325_i2s_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
}

static int es325_i2s_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	/* struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec); */
	int bits_per_sample = 0;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);
	switch (dai->id) {
	case 0:
		dev_dbg(codec->dev, "%s(): ES325_PORTA_PARAMID\n", __func__);
		break;
	case 1:
		dev_dbg(codec->dev, "%s(): ES325_PORTB_PARAMID\n", __func__);
		break;
	default:
		dev_dbg(codec->dev, "%s(): unknown port\n", __func__);
		dev_dbg(codec->dev, "%s(): exit\n", __func__);
		return -EINVAL;
	}

	dev_dbg(codec->dev, "%s(): params_channels(params) = %d\n", __func__,
		params_channels(params));
	switch (params_channels(params)) {
	case 1:
		dev_dbg(codec->dev, "%s(): 1 channel\n", __func__);
		break;
	case 2:
		dev_dbg(codec->dev, "%s(): 2 channels\n", __func__);
		break;
	case 4:
		dev_dbg(codec->dev, "%s(): 4 channels\n", __func__);
		break;
	default:
		dev_dbg(codec->dev, "%s(): unsupported number of channels\n",
			__func__);
		dev_dbg(codec->dev, "%s(): exit\n", __func__);
		return -EINVAL;
	}

	dev_dbg(codec->dev, "%s(): params_rate(params) = %d\n", __func__,
		params_rate(params));
	switch (params_rate(params)) {
	case 8000:
		dev_dbg(codec->dev, "%s(): 8000Hz\n", __func__);
		break;
	case 11025:
		dev_dbg(codec->dev, "%s(): 11025\n", __func__);
		break;
	case 16000:
		dev_dbg(codec->dev, "%s(): 16000\n", __func__);
		break;
	case 22050:
		dev_dbg(codec->dev, "%s(): 22050\n", __func__);
		break;
	case 32000:
		dev_dbg(codec->dev, "%s(): 32000\n", __func__);
		break;
	case 48000:
		dev_dbg(codec->dev, "%s(): 48000\n", __func__);
		break;
	case 96000:
		dev_dbg(codec->dev, "%s(): 96000\n", __func__);
		break;
	case 192000:
		dev_dbg(codec->dev, "%s(): 96000\n", __func__);
		break;
	default:
		dev_dbg(codec->dev, "%s(): unsupported rate = %d\n", __func__,
			params_rate(params));
		dev_dbg(codec->dev, "%s(): exit\n", __func__);
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		dev_dbg(codec->dev, "%s(): S16_LE\n", __func__);
		bits_per_sample = 16;
		break;
	case SNDRV_PCM_FORMAT_S16_BE:
		dev_dbg(codec->dev, "%s(): S16_BE\n", __func__);
		bits_per_sample = 16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		dev_dbg(codec->dev, "%s(): S20_3LE\n", __func__);
		bits_per_sample = 20;
		break;
	case SNDRV_PCM_FORMAT_S20_3BE:
		dev_dbg(codec->dev, "%s(): S20_3BE\n", __func__);
		bits_per_sample = 20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		dev_dbg(codec->dev, "%s(): S24_LE\n", __func__);
		bits_per_sample = 24;
		break;
	case SNDRV_PCM_FORMAT_S24_BE:
		dev_dbg(codec->dev, "%s(): S24_BE\n", __func__);
		bits_per_sample = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		dev_dbg(codec->dev, "%s(): S32_LE\n", __func__);
		bits_per_sample = 32;
		break;
	case SNDRV_PCM_FORMAT_S32_BE:
		dev_dbg(codec->dev, "%s(): S32_BE\n", __func__);
		bits_per_sample = 32;
		break;
	default:
		dev_dbg(codec->dev, "%s(): unknown format\n", __func__);
		dev_dbg(codec->dev, "%s(): exit\n", __func__);
		return -EINVAL;
	}
	if (ret) {
		dev_dbg(codec->dev, "%s(): snd_soc_update_bits() failed\n",
			__func__);
		dev_dbg(codec->dev, "%s(): exit\n", __func__);
		return ret;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dev_dbg(codec->dev, "%s(): PLAYBACK\n", __func__);
	else
		dev_dbg(codec->dev, "%s(): CAPTURE\n", __func__);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}

static int es325_i2s_hw_free(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dev_dbg(codec->dev, "%s(): PLAYBACK\n", __func__);
	else
		dev_dbg(codec->dev, "%s(): CAPTURE\n", __func__);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}

static int es325_i2s_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}

static int es325_i2s_trigger(struct snd_pcm_substream *substream,
			     int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);
	dev_dbg(codec->dev, "%s(): cmd = %d\n", __func__, cmd);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}
#endif

static int es325_slim_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/*
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
	*/
	return 0;
}

int es325_slim_set_channel_map(struct snd_soc_dai *dai, unsigned int tx_num,
			       unsigned int *tx_slot, unsigned int rx_num, unsigned int *rx_slot)
{
	struct snd_soc_codec *codec = dai->codec;
	/* local codec access */
	/* struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec); */
	/* remote codec access */
	struct es325_priv *es325 = &es325_priv;
	int id = dai->id;
	int i;
	int ret = 0;

	dev_dbg(codec->dev, "+%s()==\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	if (id == ES325_SLIM_1_PB ||
	    id == ES325_SLIM_2_PB ||
	    id == ES325_SLIM_3_PB) {
		es325->dai[id - 1].ch_tot = rx_num;
		es325->dai[id - 1].ch_act = 0;
		dev_dbg(codec->dev, "%s() id = %d\n", __func__, id);
		dev_dbg(codec->dev, "%s() ch_tot = %d\n", __func__, rx_num);
		for (i = 0; i < rx_num; i++) {
			es325->dai[id - 1].ch_num[i] = rx_slot[i];
			dev_dbg(codec->dev, "%s() rx_slot[] = %d\n", __func__, rx_slot[i]);
        }
	} else if (id == ES325_SLIM_1_CAP ||
		 id == ES325_SLIM_2_CAP ||
		 id == ES325_SLIM_3_CAP) {
		es325->dai[id - 1].ch_tot = tx_num;
		es325->dai[id - 1].ch_act = 0;
		dev_dbg(codec->dev, "%s() id = %d\n", __func__, id);
		dev_dbg(codec->dev, "%s() ch_tot = %d\n", __func__, tx_num);
		for (i = 0; i < tx_num; i++) {
			es325->dai[id - 1].ch_num[i] = tx_slot[i];
			dev_dbg(codec->dev, "%s() tx_slot[] = %d\n", __func__, tx_slot[i]);
		}
	}

	dev_dbg(codec->dev, "-%s()==\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(es325_slim_set_channel_map);

int es325_slim_get_channel_map(struct snd_soc_dai *dai,
			       unsigned int *tx_num, unsigned int *tx_slot,
			       unsigned int *rx_num, unsigned int *rx_slot)
{
	struct snd_soc_codec *codec = dai->codec;
	/* local codec access */
	/* struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec); */
	/* remote codec access */
	struct es325_priv *es325 = &es325_priv;
	struct es325_slim_ch *rx = es325->slim_rx;
	struct es325_slim_ch *tx = es325->slim_tx;
	int id = dai->id;
	int i;
	int ret = 0;

	dev_dbg(codec->dev, "=[ES325]=%s(): dai->id = %d\n", __func__, dai->id);

	if (id == ES325_SLIM_1_PB) {
		*rx_num = es325_dai[id - 1].playback.channels_max;
		dev_dbg(codec->dev, "=[ES325]=%s(): *rx_num = %d\n", __func__,
			*rx_num);
		for (i = 0; i < *rx_num; i++) {
			rx_slot[i] = rx[ES325_SLIM_1_PB_OFFSET + i].ch_num;
			dev_dbg(codec->dev, "=[ES325]=%s(): rx_slot[] = %d\n",
				__func__, rx_slot[i]);
		}
	} else if (id == ES325_SLIM_2_PB) {
		*rx_num = es325_dai[id - 1].playback.channels_max;
		dev_dbg(codec->dev, "=[ES325]=%s(): *rx_num = %d\n", __func__,
			*rx_num);
		for (i = 0; i < *rx_num; i++) {
			rx_slot[i] = rx[ES325_SLIM_2_PB_OFFSET + i].ch_num;
			dev_dbg(codec->dev, "=[ES325]=%s(): rx_slot[] = %d\n",
				__func__, rx_slot[i]);
		}
	} else if (id == ES325_SLIM_3_PB) {
		*rx_num = es325_dai[id - 1].playback.channels_max;
		dev_dbg(codec->dev, "=[ES325]=%s(): *rx_num = %d\n", __func__,
			*rx_num);
		for (i = 0; i < *rx_num; i++) {
			rx_slot[i] = rx[ES325_SLIM_3_PB_OFFSET + i].ch_num;
			dev_dbg(codec->dev, "=[ES325]=%s(): rx_slot[] = %d\n",
				__func__, rx_slot[i]);
		}
	} else if (id == ES325_SLIM_1_CAP) {
		*tx_num = es325_dai[id - 1].capture.channels_max;
		dev_dbg(codec->dev, "=[ES325]=%s(): *tx_num = %d\n", __func__,
			*tx_num);
		for (i = 0; i < *tx_num; i++) {
			tx_slot[i] = tx[ES325_SLIM_1_CAP_OFFSET + i].ch_num;
			dev_dbg(codec->dev, "=[ES325]=%s(): tx_slot[] = %d\n",
				__func__, tx_slot[i]);
		}
	} else if (id == ES325_SLIM_2_CAP) {
		*tx_num = es325_dai[id - 1].capture.channels_max;
		dev_dbg(codec->dev, "=[ES325]=%s(): *tx_num = %d\n", __func__,
			*tx_num);
		for (i = 0; i < *tx_num; i++) {
			tx_slot[i] = tx[ES325_SLIM_2_CAP_OFFSET + i].ch_num;
			dev_dbg(codec->dev, "=[ES325]=%s(): tx_slot[] = %d\n",
				__func__, tx_slot[i]);
		}
	} else if (id == ES325_SLIM_3_CAP) {
		*tx_num = es325_dai[id - 1].capture.channels_max;
		for (i = 0; i < *tx_num; i++) {
			tx_slot[i] = tx[ES325_SLIM_3_CAP_OFFSET + i].ch_num;
			dev_dbg(codec->dev, "=[ES325]=%s(): tx_slot[] = %d\n",
				__func__, tx_slot[i]);
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(es325_slim_get_channel_map);

static int es325_slim_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	/*
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
	*/
	return 0;
}

static int es325_slim_port_mute(struct snd_soc_dai *dai, int mute)
{
	/*
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	\/\* Is this valid since DACs are not statically mapped to DAIs? \*\/
	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): dai->id = %d\n", __func__, dai->id);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
	*/
	return 0;
}

static int es325_slim_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	/*
	struct snd_soc_codec *codec = dai->codec;
	struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
	*/
	return 0;
}

static void es325_slim_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	/*
	struct snd_soc_codec *codec = dai->codec;
	struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s(): entry\n", __func__);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	*/
}

int es325_slim_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params,
			 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	/* local codec access */
	/* struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec); */
	/* remote codec access */
	struct es325_priv *es325 = &es325_priv;
	int id = dai->id;
	int channels;
	int rate;
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);
	dev_dbg(codec->dev, "%s(): stream_name = %s\n", __func__,
		 es325_dai[id - 1].playback.stream_name);
	dev_dbg(codec->dev, "%s(): id = %d\n", __func__,
		 es325_dai[id - 1].id);

	channels = params_channels(params);
	switch (channels) {
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
		es325->dai[id - 1].ch_tot = channels;
		break;
	default:
		dev_err(codec->dev,
			"%s(): unsupported number of channels, %d\n",
			__func__, channels);
		return -EINVAL;
	}
	rate = params_rate(params);
	switch (rate) {
	case 8000:
	case 16000:
	case 32000:
	case 48000:
		es325->dai[id - 1].rate = rate;
		break;
	default:
		dev_err(codec->dev,
			"%s(): unsupported rate, %d\n",
			__func__, rate);
		return -EINVAL;
	}
#ifdef ES325_SLEEP
	mutex_lock(&es325->pm_mutex);
	if (es325->wakeup_cnt) {
		es325_switch_route();
		es325_update_VEQ_enable();
/*		es325_update_BWE_enable();*/
/*		es325_update_Tx_NS();*/
	}
	mutex_unlock(&es325->pm_mutex);
#endif

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(es325_slim_hw_params);

static int es325_slim_hw_free(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int rc = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return rc;
}

static int es325_slim_prepare(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	/*
	struct snd_soc_codec *codec = dai->codec;
	struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	dev_dbg(codec->dev, "%s(): entry\n", __func__);

	dev_dbg(codec->dev, "%s(): exit\n", __func__);
	return ret;
	*/
	return 0;
}

int es325_slim_trigger(struct snd_pcm_substream *substream,
		       int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	/* local codec access */
	/* struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec); */
	/* remote codec access */
	/* struct es325_priv *es325 = &es325_priv; */
	int id = dai->id;
	int ret = 0;

	dev_info(codec->dev, "%s(): entry\n", __func__);
	dev_info(codec->dev, "GAC:%s(): stream_name = %s\n",
		 __func__,
		 es325_dai[id - 1].playback.stream_name);
	dev_info(codec->dev, "GAC:%s(): id = %d\n",
		 __func__, es325_dai[id - 1].id);
	dev_info(codec->dev, "GAC:%s(): cmd = %d\n",
		 __func__, cmd);

	return ret;
}
EXPORT_SYMBOL_GPL(es325_slim_trigger);

#define ES325_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)
#define ES325_SLIMBUS_RATES (SNDRV_PCM_RATE_48000)

#define ES325_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE |\
			SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE |\
			SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE)
#define ES325_SLIMBUS_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S16_BE)

#if defined(CONFIG_SND_SOC_ES325_I2S)
static struct snd_soc_dai_ops es325_portx_dai_ops = {
	.set_sysclk	= es325_i2s_set_sysclk,
	.set_pll	= es325_i2s_set_pll,
	.set_clkdiv	= es325_i2s_set_clkdiv,
	.set_fmt	= es325_i2s_set_dai_fmt,
	.set_tdm_slot	= es325_i2s_set_tdm_slot,
	.set_channel_map	= es325_i2s_set_channel_map,
	.set_tristate	= es325_i2s_set_tristate,
	.digital_mute	= es325_i2s_port_mute,
	.startup	= es325_i2s_startup,
	.shutdown	= es325_i2s_shutdown,
	.hw_params	= es325_i2s_hw_params,
	.hw_free	= es325_i2s_hw_free,
	.prepare	= es325_i2s_prepare,
	.trigger	= es325_i2s_trigger,
};
#endif

#if defined(CONFIG_SND_SOC_ES325_SLIM)
static struct snd_soc_dai_ops es325_slim_port_dai_ops = {
	.set_fmt	= es325_slim_set_dai_fmt,
	.set_channel_map	= es325_slim_set_channel_map,
	.get_channel_map	= es325_slim_get_channel_map,
	.set_tristate	= es325_slim_set_tristate,
	.digital_mute	= es325_slim_port_mute,
	.startup	= es325_slim_startup,
	.shutdown	= es325_slim_shutdown,
	.hw_params	= es325_slim_hw_params,
	.hw_free	= es325_slim_hw_free,
	.prepare	= es325_slim_prepare,
	.trigger	= es325_slim_trigger,
};
#endif

static struct snd_soc_dai_driver es325_dai[] = {
#if defined(CONFIG_SND_SOC_ES325_I2S)
/* initial support is for slimbus, not i2s */
	{
		.name = "es325-porta",
		.playback = {
			.stream_name = "PORTA Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = ES325_RATES,
			.formats = ES325_FORMATS,
		},
		.capture = {
			.stream_name = "PORTA Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = ES325_RATES,
			.formats = ES325_FORMATS,
		},
		.ops = &es325_portx_dai_ops,
	},
	{
		.name = "es325-portb",
		.playback = {
			.stream_name = "PORTB Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = ES325_RATES,
			.formats = ES325_FORMATS,
		},
		.capture = {
			.stream_name = "PORTB Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = ES325_RATES,
			.formats = ES325_FORMATS,
		},
		.ops = &es325_portx_dai_ops,
	},
	{
		.name = "es325-portc",
		.playback = {
			.stream_name = "PORTC Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = ES325_RATES,
			.formats = ES325_FORMATS,
		},
		.capture = {
			.stream_name = "PORTC Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = ES325_RATES,
			.formats = ES325_FORMATS,
		},
		.ops = &es325_portx_dai_ops,
	},
#endif
#if defined(CONFIG_SND_SOC_ES325_SLIM)
	{
		.name = "es325-slim-rx1",
		.id = ES325_SLIM_1_PB,
		.playback = {
			.stream_name = "SLIM_PORT-1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES325_SLIMBUS_RATES,
			.formats = ES325_SLIMBUS_FORMATS,
		},
		.ops = &es325_slim_port_dai_ops,
	},
	{
		.name = "es325-slim-tx1",
		.id = ES325_SLIM_1_CAP,
		.capture = {
			.stream_name = "SLIM_PORT-1 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES325_SLIMBUS_RATES,
			.formats = ES325_SLIMBUS_FORMATS,
		},
		.ops = &es325_slim_port_dai_ops,
	},
	{
		.name = "es325-slim-rx2",
		.id = ES325_SLIM_2_PB,
		.playback = {
			.stream_name = "SLIM_PORT-2 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES325_SLIMBUS_RATES,
			.formats = ES325_SLIMBUS_FORMATS,
		},
		.ops = &es325_slim_port_dai_ops,
	},
	{
		.name = "es325-slim-tx2",
		.id = ES325_SLIM_2_CAP,
		.capture = {
			.stream_name = "SLIM_PORT-2 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES325_SLIMBUS_RATES,
			.formats = ES325_SLIMBUS_FORMATS,
		},
		.ops = &es325_slim_port_dai_ops,
	},
	{
		.name = "es325-slim-rx3",
		.id = ES325_SLIM_3_PB,
		.playback = {
			.stream_name = "SLIM_PORT-3 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES325_SLIMBUS_RATES,
			.formats = ES325_SLIMBUS_FORMATS,
		},
		.ops = &es325_slim_port_dai_ops,
	},
	{
		.name = "es325-slim-tx3",
		.id = ES325_SLIM_3_CAP,
		.capture = {
			.stream_name = "SLIM_PORT-3 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES325_SLIMBUS_RATES,
			.formats = ES325_SLIMBUS_FORMATS,
		},
		.ops = &es325_slim_port_dai_ops,
	},
#endif
};

#ifdef CONFIG_PM
static int es325_codec_suspend(struct snd_soc_codec *codec)
{
	struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec);

	es325_set_bias_level(codec, SND_SOC_BIAS_OFF);

	es325_sleep(es325);

	return 0;
}

static int es325_codec_resume(struct snd_soc_codec *codec)
{
	struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec);

	es325_wakeup(es325);

	es325_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}
#else
#define es325_codec_suspend NULL
#define es325_codec_resume NULL
#endif

int es325_remote_add_codec_controls(struct snd_soc_codec *codec)
{
	int rc;

	dev_info(codec->dev, "%s(): entry\n", __func__);
	dev_info(codec->dev, "%s(): codec->name = %s\n", __func__, codec->name);

	rc = snd_soc_add_codec_controls(codec, es325_digital_ext_snd_controls,
					ARRAY_SIZE(es325_digital_ext_snd_controls));
	if (rc)
		dev_err(codec->dev, "%s(): es325_digital_ext_snd_controls failed\n", __func__);

	return rc;
}

static int es325_codec_probe(struct snd_soc_codec *codec)
{
	struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "%s(): entry\n", __func__);
	dev_info(codec->dev, "%s(): codec->name = %s\n", __func__, codec->name);
	dev_info(codec->dev, "%s(): codec = 0x%08x\n", __func__,
		 (unsigned int)codec);
	dev_info(codec->dev, "%s(): es325 = 0x%08x\n", __func__,
		 (unsigned int)es325);
	es325->codec = codec;

	codec->control_data = snd_soc_codec_get_drvdata(codec);
	dev_info(codec->dev, "%s(): codec->control_data = 0x%08x\n", __func__,
		(unsigned int)codec->control_data);

	es325_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/*
	rc = snd_soc_add_codec_controls(codec, es325_digital_snd_controls,
					ARRAY_SIZE(es325_digital_snd_controls));
	if (rc)
		dev_err(codec->dev, "%s(): es325_digital_snd_controls failed\n", __func__);
	*/

	dev_info(codec->dev, "%s(): exit\n", __func__);
	return 0;
}

static int  es325_codec_remove(struct snd_soc_codec *codec)
{
	struct es325_priv *es325 = snd_soc_codec_get_drvdata(codec);

	es325_set_bias_level(codec, SND_SOC_BIAS_OFF);

	kfree(es325);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es325 = {
	.probe =	es325_codec_probe,
	.remove =	es325_codec_remove,
	.suspend =	es325_codec_suspend,
	.resume =	es325_codec_resume,
	.read =		es325_read,
	.write =	es325_write,
	.set_bias_level =	es325_set_bias_level,
};

#if defined(CONFIG_SND_SOC_ES325_I2C)
static int es325_i2c_probe(struct i2c_client *i2c,
			   const struct i2c_device_id *id)
{
	struct esxxx_platform_data *pdata = i2c->dev.platform_data;
	struct es325_priv *es325;
	const char *filename = FIRMWARE_NAME;
	int rc;

	dev_dbg(&i2c->dev, "%s(): entry\n", __func__);

	if (pdata == NULL) {
		dev_err(&i2c->dev, "%s(): pdata is NULL", __func__);
		rc = -EIO;
		goto pdata_error;
	}

	es325 = kzalloc(sizeof(struct es325_priv), GFP_KERNEL);
	if (es325 == NULL) {
		dev_err(&i2c->dev, "%s(): kzalloc failed", __func__);
		return -ENOMEM;
	}
	i2c_set_clientdata(i2c, es325);
	es325->this_client = i2c;

	rc = gpio_request(pdata->reset_gpio, "es325_reset");
	if (rc < 0) {
		dev_err(&i2c->dev, "%s(): es325_reset request failed",
			__func__);
		goto reset_gpio_request_error;
	}
	rc = gpio_direction_output(pdata->reset_gpio, 1);
	if (rc < 0) {
		dev_err(&i2c->dev, "%s(): es325_reset direction failed",
			__func__);
		goto reset_gpio_direction_error;
	}

	rc = gpio_request(pdata->wakeup_gpio, "es325_wakeup");
	if (rc < 0) {
		dev_err(&i2c->dev, "%s(): es325_wakeup request failed",
			__func__);
		goto wakeup_gpio_request_error;
	}
	rc = gpio_direction_output(pdata->wakeup_gpio, 1);
	if (rc < 0) {
		dev_err(&i2c->dev, "%s(): es325_wakeup direction failed",
			__func__);
		goto wakeup_gpio_direction_error;
	}

	dev_dbg(&i2c->dev, "%s(): initialize interrupt\n", __func__);
	dev_dbg(&i2c->dev, "%s(): TODO: interrupts\n", __func__);
	/*
	  connect 2 interrupts: accessory detect (accdet_gpio)
	  and int (int_gpio)

	  what interrupts are supported to the host?
	*/

	rc = gpio_request(pdata->gpioa_gpio, "es325_gpioa");
	if (rc < 0) {
		dev_err(&i2c->dev, "%s(): es325_gpioa request failed",
			__func__);
		goto gpioa_gpio_request_error;
	}
	rc = gpio_direction_input(pdata->gpioa_gpio);
	if (rc < 0) {
		dev_err(&i2c->dev, "%s(): es325_gpioa direction failed",
			__func__);
		goto gpioa_gpio_direction_error;
	}
	/*
	rc = request_irq(gpio_to_irq(pdata->gpioa_gpio), accdet_irq,
			 IRQF_DISABLED, "es325 accessory detect", i2c);
	if (rc) {
		dev_err(&i2c->dev, "%s(): es325_gpioa irq request failed\n",
			__func__);
		goto gpioa_request_irq_error;
	}
	*/

	rc = gpio_request(pdata->gpiob_gpio, "es325_gpiob");
	if (rc < 0) {
		dev_err(&i2c->dev, "%s(): es325_gpiob request failed",
			__func__);
		goto gpiob_gpio_request_error;
	}
	rc = gpio_direction_input(pdata->gpiob_gpio);
	if (rc < 0) {
		dev_err(&i2c->dev, "%s(): es325_gpiob direction failed",
			__func__);
		goto gpiob_gpio_direction_error;
	}

	/* rc = request_irq(gpio_to_irq(pdata->gpiob_gpio), es325_status_irq, */
	/*		 IRQF_DISABLED, "es325 status", i2c); */
	/* if (rc) { */
	/*	dev_err(&i2c->dev, "%s(): es325_gpiob irq request failed\n", */
	/*		__func__); */
	/*	goto gpiob_request_irq_error; */
	/* } */

	gpio_set_value(pdata->reset_gpio, 0);
	gpio_set_value(pdata->wakeup_gpio, 1);

	es325->pdata = pdata;

	rc = request_firmware(&es325->fw, filename, &i2c->dev);
	if (rc) {
		dev_err(&i2c->dev, "%s(): request_firmware(%s) failed %d\n",
			__func__, filename, rc);
		goto request_firmware_error;
	}
	rc = es325_bootup(es325);
	if (rc) {
		dev_err(&i2c->dev, "%s(): es325_bootup failed %d\n",
			__func__, rc);
		goto bootup_error;
	}
	release_firmware(es325->fw);

	rc = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_es325, es325_dai,
				    ARRAY_SIZE(es325_dai));
	dev_dbg(&i2c->dev, "%s(): rc = snd_soc_regsiter_codec() = %d\n",
		__func__, rc);

	dev_dbg(&i2c->dev, "%s(): exit\n", __func__);
	return rc;

bootup_error:
request_firmware_error:
gpiob_request_irq_error:
gpiob_gpio_direction_error:
	gpio_free(pdata->gpiob_gpio);
gpiob_gpio_request_error:
gpioa_request_irq_error:
gpioa_gpio_direction_error:
	gpio_free(pdata->gpioa_gpio);
gpioa_gpio_request_error:
wakeup_gpio_direction_error:
	gpio_free(pdata->wakeup_gpio);
wakeup_gpio_request_error:
reset_gpio_direction_error:
	gpio_free(pdata->reset_gpio);
reset_gpio_request_error:
pdata_error:
	dev_dbg(&i2c->dev, "%s(): exit with error\n", __func__);
	return rc;
}

static int es325_i2c_remove(struct i2c_client *i2c)
{
	struct esxxx_platform_data *pdata = i2c->dev.platform_data;

	gpio_free(pdata->reset_gpio);
	gpio_free(pdata->wakeup_gpio);
	gpio_free(pdata->gpioa_gpio);

	snd_soc_unregister_codec(&i2c->dev);

	kfree(i2c_get_clientdata(i2c));

	return 0;
}

static const struct i2c_device_id es325_i2c_id[] = {
	{ "es325", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, es325_i2c_id);

static struct i2c_driver es325_i2c_driver = {
	.driver = {
		.name = "es325-codec",
		.owner = THIS_MODULE,
	},
	.probe = es325_i2c_probe,
	.remove = es325_i2c_remove,
	.id_table = es325_i2c_id,
};
#else
static int es325_slim_device_up(struct slim_device *sbdev);
static int es325_slim_get_laddr(struct slim_device *sb,
		const u8 *e_addr, u8 e_len, u8 *laddr)
{
	int ret = 0;
	int count = 10;

	do {
		ret = slim_get_logical_addr(sb, e_addr, e_len, laddr);
		if (!ret)
			break;
		/* Give SLIMBUS time to report present and be ready. */
		usleep_range(1000, 1000);
		pr_info("[HHJI]%s: retyring get logical addr\n",
				__func__);
	} while (--count);
	return ret;
}
static int es325_fw_thread(void *priv)
{
	struct es325_priv *es325 = (struct es325_priv  *)priv;
	es325_slim_get_laddr(es325->gen0_client, es325->gen0_client->e_addr,
		6, &(es325->gen0_client->laddr));
	es325_slim_get_laddr(es325->intf_client, es325->intf_client->e_addr,
		6, &(es325->intf_client->laddr));
	es325_slim_device_up(es325->gen0_client);
	return 0;
}

static int es325_slim_probe(struct slim_device *sbdev)
{
	struct esxxx_platform_data *pdata = sbdev->dev.platform_data;
	const char *filename = FIRMWARE_NAME;
	int rc;
	static int clk_count;
	struct task_struct *thread = NULL;

#if defined(CONFIG_MACH_JACTIVE_ATT)
	if (system_rev >= 12) /* HW REV02 */
		filename = FIRMWARE_NAME_REV02;
#endif
	dev_dbg(&sbdev->dev, "%s(): entry\n", __func__);
	dev_dbg(&sbdev->dev, "%s(): sbdev->name = %s\n", __func__, sbdev->name);
	dev_dbg(&sbdev->dev, "%s(): es325_priv = 0x%08x\n", __func__,
		(unsigned int)&es325_priv);

	slim_set_clientdata(sbdev, &es325_priv);
	if (strcmp(sbdev->name, "es325-codec-intf") == 0) {
		dev_dbg(&sbdev->dev, "%s(): interface device probe\n",
			__func__);
		es325_priv.intf_client = sbdev;
		return 0;
	}

	if (strcmp(sbdev->name, "es325-codec-gen0") == 0) {
		dev_dbg(&sbdev->dev, "%s(): generic device probe\n", __func__);
		es325_priv.gen0_client = sbdev;
	}

	if (es325_priv.intf_client == NULL ||
	    es325_priv.gen0_client == NULL) {
		dev_dbg(&sbdev->dev, "%s() incomplete initialization\n",
			__func__);
		return 0;
	}

	if (pdata == NULL) {
		dev_err(&sbdev->dev, "%s(): pdata is NULL", __func__);
		rc = -EIO;
		goto pdata_error;
	}

	if (clk_count++ == 0) {
		pdata->es325_clk_cb(1);
	}

	rc = device_create_file(&sbdev->dev, &dev_attr_route_status);
	if (rc)
		dev_err(&sbdev->dev, "%s(): error file sysfs create\n",	__func__);
	rc = device_create_file(&sbdev->dev, &dev_attr_route_config);
	if (rc)
		dev_err(&sbdev->dev, "%s(): error file sysfs create\n",	__func__);
	rc = device_create_file(&sbdev->dev, &dev_attr_txhex);
	if (rc)
		dev_err(&sbdev->dev, "%s(): error file sysfs create\n", __func__);
	rc = device_create_file(&sbdev->dev, &dev_attr_fw_version);
	if (rc)
		dev_err(&sbdev->dev, "%s(): error file sysfs create\n", __func__);
	rc = device_create_file(&sbdev->dev, &dev_attr_clock_on);
	if (rc)
		dev_err(&sbdev->dev, "%s(): error file sysfs create\n",	__func__);
	rc = device_create_file(&sbdev->dev, &dev_attr_slim_ch_status);
	if (rc)
		dev_err(&sbdev->dev, "%s(): error file sysfs create\n",	__func__);
	rc = device_create_file(&sbdev->dev, &dev_attr_es325_reg);
	if (rc)
		dev_err(&sbdev->dev, "%s(): error file sysfs create\n",	__func__);
	rc = device_create_file(&sbdev->dev, &dev_attr_es325_cmd_reg);
	if (rc)
		dev_err(&sbdev->dev, "%s(): error file sysfs create\n",	__func__);			
	rc = device_create_file(&sbdev->dev, &dev_attr_firmware);
	if (rc)
		dev_err(&sbdev->dev, "%s(): error file firmware create\n",	__func__);

	dev_dbg(&sbdev->dev, "%s(): reset_gpio = %d\n", __func__,
		pdata->reset_gpio);
	/*
	rc = gpio_request(pdata->reset_gpio, "es325_reset");
	if (rc < 0) {
		dev_err(&sbdev->dev, "%s(): es325_reset request failed",
			__func__);
		goto reset_gpio_request_error;
	}
	rc = gpio_direction_output(pdata->reset_gpio, 1);
	if (rc < 0) {
		dev_err(&sbdev->dev, "%s(): es325_reset direction failed",
			__func__);
		goto reset_gpio_direction_error;
	}
	gpio_set_value(pdata->reset_gpio, 1);
	*/

	dev_dbg(&sbdev->dev, "%s(): wakeup_gpio = %d\n", __func__, pdata->wakeup_gpio);

	/*
	rc = gpio_request(pdata->wakeup_gpio, "es325_wakeup");
	if (rc < 0) {
		dev_err(&sbdev->dev, "%s(): es325_wakeup request failed",
			__func__);
		goto wakeup_gpio_request_error;
	}
	rc = gpio_direction_output(pdata->wakeup_gpio, 1);
	if (rc < 0) {
		dev_err(&sbdev->dev, "%s(): es325_wakeup direction failed",
			__func__);
		goto wakeup_gpio_direction_error;
	}
	*/

	dev_dbg(&sbdev->dev, "%s(): gpioa_gpio = %d\n", __func__, pdata->gpioa_gpio);
	/*
	rc = gpio_request(pdata->gpioa_gpio, "es325_gpioa");
	if (rc < 0) {
		dev_err(&sbdev->dev, "%s(): es325_gpioa request failed",
			__func__);
		goto gpioa_gpio_request_error;
	}
	rc = gpio_direction_input(pdata->gpioa_gpio);
	if (rc < 0) {
		dev_err(&sbdev->dev, "%s(): es325_gpioa direction failed",
			__func__);
		goto gpioa_gpio_direction_error;
	}
	*/

	dev_dbg(&sbdev->dev, "%s(): gpiob_gpio = %d\n", __func__, pdata->gpiob_gpio);
	/*
	rc = gpio_request(pdata->gpiob_gpio, "es325_gpiob");
	if (rc < 0) {
		dev_err(&sbdev->dev, "%s(): es325_gpiob request failed",
			__func__);
		goto gpiob_gpio_request_error;
	}
	rc = gpio_direction_input(pdata->gpiob_gpio);
	if (rc < 0) {
		dev_err(&sbdev->dev, "%s(): es325_gpiob direction failed",
			__func__);
		goto gpiob_gpio_direction_error;
	}
	*/

	es325_priv.pdata = pdata;
#ifdef ES325_SLEEP
	mutex_init(&es325_priv.pm_mutex);
	es325_priv.internal_route_config =  ES325_INTERNAL_ROUTE_MAX;
	es325_priv.new_internal_route_config = ES325_INTERNAL_ROUTE_MAX;
#endif
	pr_info("%s: system_rev=%d, firmware=%s\n", __func__, system_rev, filename);
	rc = request_firmware((const struct firmware **)&es325_priv.fw,
			      filename, &sbdev->dev);
	if (rc) {
		dev_err(&sbdev->dev, "%s(): request_firmware(%s) failed %d\n", __func__, filename, rc);
		goto request_firmware_error;
	}

	thread = kthread_run(es325_fw_thread, &es325_priv, "audience thread");
	if (IS_ERR(thread)) {
		dev_err(&sbdev->dev, "%s(): can't create es325 firmware thread = %p\n", __func__, thread);
		return -1;
	}
	return 0;

request_firmware_error:
/*
gpiob_gpio_direction_error:
	gpio_free(pdata->gpiob_gpio);
gpiob_gpio_request_error:
gpioa_gpio_direction_error:
	gpio_free(pdata->gpioa_gpio);
gpioa_gpio_request_error:
wakeup_gpio_direction_error:
	gpio_free(pdata->wakeup_gpio);
wakeup_gpio_request_error:
reset_gpio_direction_error:
	gpio_free(pdata->reset_gpio);
reset_gpio_request_error:
*/
pdata_error:
	dev_dbg(&sbdev->dev, "%s(): exit with error\n", __func__);
	return rc;
}

static int register_snd_soc(struct es325_priv *priv)
{
	int rc;
	int i;
	int ch_cnt;
	struct slim_device *sbdev = priv->gen0_client;

	rc = es325_init_slim_slave(sbdev);

	dev_dbg(&sbdev->dev, "%s(): name = %s\n", __func__, sbdev->name);
	rc = snd_soc_register_codec(&sbdev->dev, &soc_codec_dev_es325,
					es325_dai, ARRAY_SIZE(es325_dai));
	dev_dbg(&sbdev->dev, "%s(): rc = snd_soc_regsiter_codec() = %d\n",
		__func__, rc);

	/* allocate ch_num array for each DAI */
	for (i = 0; i < ARRAY_SIZE(es325_dai); i++) {
		switch (es325_dai[i].id) {
		case ES325_SLIM_1_PB:
		case ES325_SLIM_2_PB:
		case ES325_SLIM_3_PB:
			ch_cnt = es325_dai[i].playback.channels_max;
			break;
		case ES325_SLIM_1_CAP:
		case ES325_SLIM_2_CAP:
		case ES325_SLIM_3_CAP:
			ch_cnt = es325_dai[i].capture.channels_max;
			break;
		default:
			continue;
		}
		es325_priv.dai[i].ch_num =
			kzalloc((ch_cnt * sizeof(unsigned int)), GFP_KERNEL);
	}
	dev_dbg(&sbdev->dev, "%s(): exit\n", __func__);
	return rc;
}

static int es325_slim_remove(struct slim_device *sbdev)
{
	struct esxxx_platform_data *pdata = sbdev->dev.platform_data;

	dev_dbg(&sbdev->dev, "%s(): entry\n", __func__);
	dev_dbg(&sbdev->dev, "%s(): sbdev->name = %s\n", __func__, sbdev->name);

	gpio_free(pdata->reset_gpio);
	gpio_free(pdata->wakeup_gpio);
	gpio_free(pdata->gpioa_gpio);

	snd_soc_unregister_codec(&sbdev->dev);

	dev_dbg(&sbdev->dev, "%s(): exit\n", __func__);

	return 0;
}

#ifdef ES325_SLEEP

#define ES325_SLEEP_TIME	HZ
struct delayed_work es325_work;
struct workqueue_struct *es325_workqueue;
static int es325_schedule_sleep_workqueue(void)
{
	int rc = 0;
	struct es325_priv *es325 = &es325_priv;
	rc = queue_delayed_work(es325_workqueue, &es325_work, ES325_SLEEP_TIME);
	if (!rc) {
		pr_err("=[ES325]=%s delayed work queue failed\n", __func__);
		return -1;
	}
	es325->wq_active = 1;
	return 0;
}

static void es325_wrapper_wakeup_internal(void)
{
	struct es325_priv *es325 = &es325_priv;
	if (!es325->clock_on) {
		pr_info("=[ES325]=%s enable clock\n", __func__);
		es325->pdata->es325_clk_cb(1);
		usleep_range(1000, 1100);
		es325_wakeup(es325);
		es325->clock_on = 1;
	}
}
#endif

void es325_wrapper_wakeup(struct snd_soc_dai *dai)
{
#ifdef ES325_SLEEP
	int rc;
	struct es325_priv *es325 = &es325_priv;
	pr_debug("=[ES325]=%s dai_id=%d ch_wakeup=%d,wakeup_cnt=%d\n", __func__,
		dai->id, es325->dai[dai->id-1].ch_wakeup, es325->wakeup_cnt);
	if (!es325_remote_route_enable(dai)) {
		pr_info("=[ES325]=%s(dai->id = %d) es325 not activated %d %d %d\n",
			__func__, dai->id, es325_rx1_route_enable,
			es325_tx1_route_enable, es325_rx2_route_enable);
		return;
	}

	mutex_lock(&es325->pm_mutex);
	es325->dai[dai->id-1].ch_wakeup = 1;
	if (es325->wakeup_cnt) {
		es325->wakeup_cnt++;
	} else {
		es325->wakeup_cnt++;
		if (es325->wq_active) {
			pr_debug("=[ES325]=%s delete sleep timer wakeup count=%d\n", __func__, es325->wakeup_cnt);
			if (delayed_work_pending(&es325_work)) {
				rc = cancel_delayed_work_sync(&es325_work);
				pr_debug("=[ES325]=%s cancel work queue rc=%d\n", __func__, rc);
			}
			es325->wq_active = 0;
		}
		es325_wrapper_wakeup_internal();
	}
	mutex_unlock(&es325->pm_mutex);
#endif
}
EXPORT_SYMBOL_GPL(es325_wrapper_wakeup);

#ifdef ES325_SLEEP
 void es325_wrapper_sleep_internal(struct work_struct *dummy)
{
	struct es325_priv *es325 = &es325_priv;

	pr_debug("=[ES325]=%s clock_on=%d\n", __func__, es325->clock_on);
	mutex_lock(&es325->pm_mutex);
	es325->wq_active = 0;
	if (es325->wakeup_cnt) {
		pr_info("=[ES325]=%s wakeup needed before turning off clock[clock on==%d]\n", __func__, es325->clock_on);
	} else {
		if (es325->clock_on) {
			es325_sleep(es325);
		 	pr_info("=[ES325]=%s disable clock\n", __func__);
			es325->pdata->es325_clk_cb(0);
			es325->clock_on = 0;
		}
	}
	/* initialize variable to update ES325 settings when wake-up */
	es325->internal_route_config = ES325_INTERNAL_ROUTE_MAX;
	es325_VEQ_enable = ES325_MAX_INVALID_VEQ;
	es325_BWE_enable = ES325_MAX_INVALID_BWE;
	es325_Tx_NS = ES325_MAX_INVALID_TX_NS;
	es325_VEQ_enable_new = ES325_MAX_INVALID_VEQ;
	es325_BWE_enable_new = ES325_MAX_INVALID_BWE;
	es325_Tx_NS_new = ES325_MAX_INVALID_TX_NS;	
	mutex_unlock(&es325->pm_mutex);
}
#endif

void es325_wrapper_sleep(int dai_id)
{
#ifdef ES325_SLEEP
	struct es325_priv *es325 = &es325_priv;
	pr_debug("=[ES325]=%s dai_id=%d ch_wakeup=%d,wakeup_cnt=%d\n", __func__,
		dai_id, es325->dai[dai_id-1].ch_wakeup, es325->wakeup_cnt);
	mutex_lock(&es325->pm_mutex);
	/* For dai not using audience */
	if (es325->dai[dai_id-1].ch_wakeup == 0) {
		pr_debug("=[ES325]=%s dai_id=%d ch_wakeup=%d\n", __func__,
			dai_id, es325->dai[dai_id-1].ch_wakeup);
	} else {
		es325->dai[dai_id-1].ch_wakeup = 0;

		if (es325->wakeup_cnt) {
			es325->wakeup_cnt--;
		}
		if (es325->wakeup_cnt == 0) {
			if (!es325->wq_active)
				es325_schedule_sleep_workqueue();
		}
	}
	mutex_unlock(&es325->pm_mutex);
#endif
}
EXPORT_SYMBOL_GPL(es325_wrapper_sleep);

static int es325_slim_device_up(struct slim_device *sbdev)
{
	struct es325_priv *es325;
	int rc;

	dev_info(&sbdev->dev, "%s: name=%s\n", __func__, sbdev->name);
	dev_info(&sbdev->dev, "%s: laddr=%d\n", __func__, sbdev->laddr);

	/* Start the firmware download in the workqueue context. */
	es325 = slim_get_devicedata(sbdev);

	dev_info(&sbdev->dev, "%s: priv=%p\n", __func__, es325);

	if (strcmp(sbdev->name, "es325-codec-intf") == 0)
		return 0;

	rc = fw_download(es325);
	BUG_ON(rc != 0);

#ifdef ES325_SLEEP
	if (strcmp(sbdev->name, "es325-codec-gen0") == 0) {

		dev_info(&sbdev->dev, "wrapper %s(): es325-codec-gen0\n",
					__func__);
		es325->clock_on = 1;
		es325_wrapper_sleep_internal(NULL);

		dev_info(&sbdev->dev, "wrapper %s: es325 sleep default\n",
					__func__);
	}
#endif

	dev_info(&sbdev->dev, "wrapper %s: device up complete\n", __func__);

	return rc;
}

static const struct slim_device_id es325_slim_id[] = {
	{ "es325-codec", 0 },
	{ "es325-codec-intf", 0 },
	{ "es325-codec-gen0", 0 },
	{  }
};
MODULE_DEVICE_TABLE(slim, es325_slim_id);

static struct slim_driver es325_slim_driver = {
	.driver = {
		.name = "es325-codec",
		.owner = THIS_MODULE,
	},
	.probe = es325_slim_probe,
	.remove = es325_slim_remove,
	.id_table = es325_slim_id,
};
#endif /* CONFIG_SND_SOC_ES325_I2C */

static __init int es325_init(void)
{
	int ret = 0;
	pr_info("%s(): entry", __func__);
	memset(&es325_priv, 0, sizeof(es325_priv));
#ifdef ES325_SLEEP
	es325_workqueue = create_workqueue("ES325");
	if (!es325_workqueue) {
		pr_info("%s can't create workqueue\n", __func__);
		return -1;
	}
	INIT_DELAYED_WORK(&es325_work, es325_wrapper_sleep_internal);
#endif

#if defined(CONFIG_SND_SOC_ES325_I2C)
	ret = i2c_add_driver(&es325_i2c_driver);
	if (ret)
		pr_err("Failed to register Audience eS325 I2C driver: %d\n",
		       ret);
#else
	pr_info("%s(): slim_driver_register()", __func__);
	ret = slim_driver_register(&es325_slim_driver);
	if (ret)
		pr_err("Error registering Audience eS325 SLIMbus driver: %d\n",
		       ret);
#endif

	pr_info("%s(): exit\n", __func__);
	return ret;
}
module_init(es325_init);

static __exit void es325_exit(void)
{
	pr_info("%s(): entry\n", __func__);
#ifdef ES325_SLEEP
	if (es325_workqueue)
		destroy_workqueue(es325_workqueue);
	es325_workqueue = NULL;
#endif

#if defined(CONFIG_SND_SOC_ES325_I2C)
	i2c_del_driver(&es325_i2c_driver);
#else
	/* Samsung - no support from QCOM to unregister
	 * slim_driver_unregister(&es325_slim_driver);
	 */
#endif
	pr_info("%s(): exit\n", __func__);
}
module_exit(es325_exit);


MODULE_DESCRIPTION("ASoC ES325 driver");
MODULE_AUTHOR("Greg Clemson <gclemson@audience.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:es325-codec");
MODULE_FIRMWARE(FIRMWARE_NAME);
#if defined(CONFIG_MACH_JACTIVE_ATT)
MODULE_FIRMWARE(FIRMWARE_NAME_REV02);
#endif
