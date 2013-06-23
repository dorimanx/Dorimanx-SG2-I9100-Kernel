/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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

#include "msm_sensor.h"
#include "s5k6b2yx.h"

#define SENSOR_NAME "s5k6b2yx"
#define PLATFORM_DRIVER_NAME "msm_camera_s5k6b2yx"
#define s5k6b2yx_obj s5k6b2yx_##obj

#define VISION_MODE_TEST_PATTERN	0

#define VISION_MODE_AE_REG_ADDR		0x600a
#define VISION_MODE_AE_BACKLIGHT	0x9a
#define VISION_MODE_AE_NORMAL		0x2a

#define VISION_MODE_SET_FPS_ADDR         0x6027
#define VISION_MODE_SET_FPS_5           0x1
#define VISION_MODE_SET_FPS_10          0X2
#define VISION_MODE_FPS_5_VAL           0xD0
#define VISION_MODE_FPS_10_VAL          0x68

DEFINE_MUTEX(s5k6b2yx_mut);
static struct msm_sensor_ctrl_t s5k6b2yx_s_ctrl;
int32_t msm_sensor_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf);
int32_t msm_sensor_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf);

/*176x104 in vision mode*/
static struct msm_camera_i2c_reg_conf s5k6b2yx_vision_settings[] = {
	/* For 176 x 104 */
	{0x4345, 0x80},		/* x_addr_start_alv : 128 */
	{0x4348, 0x07},		/* x_addr_end_alv : 1887 */
	{0x4349, 0x5F},
	{0x434D, 0xB0},		/* x_output_size_alv : 176 */
	/*  patch size:  ((44-1) x (26-1)) */
	{0x6014, 0x2B},		/*  patch width */
	{0x6015, 0x19},		/*  patch height */
	{0x6026, 0x00},		/*  5fps */
//	{0x6027, 0xD0},		/*  5fps */
	/*  number of pixel : 176*104*24/64=6864 */
	{0x5030, 0x11},
	{0x5031, 0xE0},
	/* AE max shutter */
	{0x5014, 0x11},
	{0x5015, 0x00},
	/*  8bit mode */
	{0x7030, 0x0E},
	{0x7031, 0x2F},
	/*  Analog Tuning */
	{0x7067, 0x00},
	{0x7074, 0x22},
	{0x7402, 0x1F},
	{0x7403, 0xC0},
	{0x7247, 0x01},
	{0x7412, 0x09},
	{0x7413, 0xB9},
	{0x7430, 0x05},
	{0x7432, 0x02},
	{0x7433, 0x32},
	{0x7075, 0x3D},
	{0x7066, 0x09},
	{0x6000, 0x11},
	{0x6001, 0x11},
	{0x6002, 0x11},
	{0x6003, 0x11},
	{0x6004, 0x11},
	{0x6005, 0x11},
	{0x6006, 0x11},
	{0x6007, 0x11},
	/* Target */
	{0x600A, 0x2A},
	/*  Speed */
	{0x5034, 0x00},
	/*  Innner Target Tolerance */
	{0x503F, 0x03},
	/*  G + R Setting (20120813) */
	/*  Vision Senser Data = 0.5*Gr + 0.5*R */
	{0x6029, 0x02},	/*  [2:0] : 1bit integer, 2bit fraction */
	{0x602A, 0x02},	/*  [2:0] : 1bit integer, 2bit fraction */
	/*  For Analog Gain 16x (20120904) */
	{0x7018, 0xCF},
	{0x7019, 0xDB},
	{0x702A, 0x8D},
	{0x702B, 0x60},
	{0x5035, 0x02},	/*  analog gain max */
	/* BIT_RATE_MBPS_alv (584Mbps) */
	{0x7351, 0x02},
	{0x7352, 0x48},
	{0x7353, 0x00},
	{0x7354, 0x00},
#if VISION_MODE_TEST_PATTERN
	{0x7203, 0x42}, /* to enable test pattern */
#endif
	{0x7339, 0x03} /*  [2]dphy_en1, [1]dphy_en0, [0] dhpy_en_clk */
};

static struct msm_camera_i2c_reg_conf s5k6b2yx_prev_snap_settings[] = {
#if 0 //CHANGED BY jay because of sensor vendor recommendation

	{0x3067, 0x25},
	{0x307D, 0x08},
	{0x307E, 0x08},
	{0x307F, 0x08},
	{0x3080, 0x08},
	{0x3073, 0x73},
	{0x3074, 0x45},
	{0x3075, 0xD4},
	{0x3085, 0xF0},
	{0x3068, 0x55},
	{0x3069, 0x00},
	{0x3063, 0x08},
	{0x3064, 0x00},
	{0x3083, 0xFF},
	{0x3084, 0xFF},
	/*pll settings*/
	{0x0305, 0x06},
	{0x0306, 0x00},
	{0x0307, 0xb7},

	/*size settings*/
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x07},
	{0x0349, 0x8F},
	{0x034a, 0x04},
	{0x034b, 0x47},
	{0x034c, 0x07},
	{0x034d, 0x90},
	{0x034e, 0x04},
	{0x034f, 0x48},

	/*subsampling and fll*/
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},

	{0x0340, 0x04},
	{0x0341, 0x5C},
	{0x0202, 0x02},
	{0x0203, 0x2e},
	{0x0204, 0x00},
	{0x0205, 0x20},

	{0x3351, 0x02},
	{0x3352, 0xDC},
	{0x3353, 0x00},
	{0x3354, 0x00},
	{0x7339, 0x03}, /* [2]dphy_en1, [1]dphy_en0, [0] dhpy_en_clk */
#else
	{0x31D3, 0x01}, 	// efuse read en
	{0x3426, 0x3A}, 	// [4]corr_en, [3:2]gain_b_sel, [1:0]gain_r_sel
	{0x340D, 0x30}, 	// efuse clock off

	{0x3067, 0x25}, 	// adc_sat[mV]=617mV 
	{0x307D, 0x08}, 	// dbr_tune_tgs
	{0x307E, 0x08}, 	// dbr_tune_rg
	{0x307F, 0x08}, 	// dbr_tune_fdb
	{0x3080, 0x04}, 	// dbr_tune_ntg 
	{0x3073, 0x73}, 	// comp1_bias, comp2_bias
	{0x3074, 0x45}, 	// pix_bias, pix_bias_boost
	{0x3075, 0xD4}, 	// clp_lvl
	{0x3085, 0xF0}, 	// rdv_option; LOB_PLA enable
	{0x3068, 0x55}, 	// ms[15:8]; x4~
	{0x3069, 0x00}, 	// ms[7:0]; x1~x4
	{0x3063, 0x08}, 	// cds_option[15:8]; [11]ldb nmos sw enable=1
	{0x3064, 0x00}, 	// cds_option[7:0]; 
	{0x3010, 0x04}, 	// FD start 2->4 added by jay for low lux fluctuation

	{0x3247, 0x11},    // [4] fadlc_blst_en
	{0x3083, 0x00},    // blst_en_cintr = 16
	{0x3084, 0x10},

	{0x0306, 0x00}, //pll multiplier
	{0x0307, 0xb7},

	{0x3351, 0x02},
	{0x3352, 0xDC},
	{0x3353, 0x00},
	{0x3354, 0x00},

	{0x7339, 0x03}, // [2]dphy_en1, [1]dphy_en0, [0] dhpy_en_clk

	{0x0202, 0x04}, //added by Jay coarse int time
	{0x0203, 0x5c}, //6D}, //1116
	{0x0204, 0x00}, //added by Jay global gain		/*indoor index: 33ms, x1.059*/
	{0x0205, 0x22},

	{0x0344, 0x00}, 	// x_addr_start M{0xB
	{0x0345, 0x00}, 	// x_addr_start L{0xB
	{0x0346, 0x00}, 	// y_addr_start M{0xB
	{0x0347, 0x02}, 	// y_addr_start L{0xB

	{0x0348, 0x07}, 	// x_addr_end MSB
	{0x0349, 0x8F}, 	// x_addr_end LSB
	{0x034a, 0x04}, 	// y_addr_end MSB
	{0x034b, 0x43}, 	// y_addr_end LSB

	{0x034c, 0x07}, 	// x_output_size MSB
	{0x034d, 0x90}, 	// x_output_size LSB
	{0x034e, 0x04}, 	// y_output_size MSB
	{0x034f, 0x42}, 	// y_output_size LSB

#endif
};


static struct msm_camera_i2c_reg_conf s5k6b2yx_pip_prev_snap_settings[] = {
	{0x31D3, 0x01}, 	// efuse read en
	{0x3426, 0x3A}, 	// [4]corr_en, [3:2]gain_b_sel, [1:0]gain_r_sel
	{0x340D, 0x30}, 	// efuse clock off

	{0x3067, 0x25}, 	// adc_sat[mV]=617mV
	{0x307D, 0x08}, 	// dbr_tune_tgs
	{0x307E, 0x08}, 	// dbr_tune_rg
	{0x307F, 0x08}, 	// dbr_tune_fdb
	{0x3080, 0x04}, 	// dbr_tune_ntg
	{0x3073, 0x73}, 	// comp1_bias, comp2_bias
	{0x3074, 0x45}, 	// pix_bias, pix_bias_boost
	{0x3075, 0xD4}, 	// clp_lvl
	{0x3085, 0xF0}, 	// rdv_option; LOB_PLA enable
	{0x3068, 0x55}, 	// ms[15:8]; x4~
	{0x3069, 0x00}, 	// ms[7:0]; x1~x4
	{0x3063, 0x08}, 	// cds_option[15:8]; [11]ldb nmos sw enable=1
	{0x3064, 0x00}, 	// cds_option[7:0];
	{0x3010, 0x04}, 	// FD start 2->4 added by jay for low lux fluctuation

	{0x3247, 0x11},    // [4] fadlc_blst_en
	{0x3083, 0x00},    // blst_en_cintr = 16
	{0x3084, 0x10},

	{0x0306, 0x00}, //pll multiplier
	{0x0307, 0x92},

	{0x3351, 0x02},
	{0x3352, 0x48},
	{0x3353, 0x00},
	{0x3354, 0x00},

	{0x7339, 0x03}, // [2]dphy_en1, [1]dphy_en0, [0] dhpy_en_clk

	{0x0202, 0x03},//02}, //added by np coarse int time
	{0x0203, 0x88},//2E},
	{0x0204, 0x00}, //added by np global gain		/*indoor index: 33ms, x1.059*/
	{0x0205, 0x2A}, //22},//20},

	{0x0344, 0x00}, 	// x_addr_start M{0xB
	{0x0345, 0x00}, 	// x_addr_start L{0xB
	{0x0346, 0x00}, 	// y_addr_start M{0xB
	{0x0347, 0x02}, 	// y_addr_start L{0xB

	{0x0348, 0x07}, 	// x_addr_end MSB
	{0x0349, 0x8F}, 	// x_addr_end LSB
	{0x034a, 0x04}, 	// y_addr_end MSB
	{0x034b, 0x43}, 	// y_addr_end LSB

	{0x034c, 0x07}, 	// x_output_size MSB
	{0x034d, 0x90}, 	// x_output_size LSB
	{0x034e, 0x04}, 	// y_output_size MSB
	{0x034f, 0x42}, 	// y_output_size LSB

};

static struct msm_camera_i2c_reg_conf s5k6b2yx_pip_prev_snap_settings_rev0[] = {
	{0x3067, 0x25}, 	// adc_sat[mV]=617mV
	{0x307D, 0x08}, 	// dbr_tune_tgs
	{0x307E, 0x08}, 	// dbr_tune_rg
	{0x307F, 0x08}, 	// dbr_tune_fdb
	{0x3080, 0x04}, 	// dbr_tune_ntg
	{0x3073, 0x73}, 	// comp1_bias, comp2_bias
	{0x3074, 0x45}, 	// pix_bias, pix_bias_boost
	{0x3075, 0xD4}, 	// clp_lvl
	{0x3085, 0xF0}, 	// rdv_option; LOB_PLA enable
	{0x3068, 0x55}, 	// ms[15:8]; x4~
	{0x3069, 0x00}, 	// ms[7:0]; x1~x4
	{0x3063, 0x08}, 	// cds_option[15:8]; [11]ldb nmos sw enable=1
	{0x3064, 0x00}, 	// cds_option[7:0];
	{0x3010, 0x04}, 	// FD start 2->4 added by jay for low lux fluctuation

	{0x3247, 0x11},    // [4] fadlc_blst_en
	{0x3083, 0x00},    // blst_en_cintr = 16
	{0x3084, 0x10},

	{0x0306, 0x00}, //pll multiplier
	{0x0307, 0x92},

	{0x3351, 0x02},
	{0x3352, 0x48},
	{0x3353, 0x00},
	{0x3354, 0x00},

	{0x7339, 0x03}, // [2]dphy_en1, [1]dphy_en0, [0] dhpy_en_clk

	{0x0202, 0x03},//02}, //added by np coarse int time
	{0x0203, 0x88},//2E},
	{0x0204, 0x00}, //added by np global gain		/*indoor index: 33ms, x1.059*/
	{0x0205, 0x2A}, //22},//20},

	{0x0344, 0x00}, 	// x_addr_start M{0xB
	{0x0345, 0x00}, 	// x_addr_start L{0xB
	{0x0346, 0x00}, 	// y_addr_start M{0xB
	{0x0347, 0x02}, 	// y_addr_start L{0xB

	{0x0348, 0x07}, 	// x_addr_end MSB
	{0x0349, 0x8F}, 	// x_addr_end LSB
	{0x034a, 0x04}, 	// y_addr_end MSB
	{0x034b, 0x43}, 	// y_addr_end LSB

	{0x034c, 0x07}, 	// x_output_size MSB
	{0x034d, 0x90}, 	// x_output_size LSB
	{0x034e, 0x04}, 	// y_output_size MSB
	{0x034f, 0x42}, 	// y_output_size LSB

};
static struct msm_camera_i2c_reg_conf s5k6b2yx_prev_snap_settings_rev0[] = { //JAY add for rev 1.0 sensor

	{0x3067, 0x25}, 	// adc_sat[mV]=617mV 
	{0x307D, 0x08}, 	// dbr_tune_tgsl
	{0x307E, 0x08}, 	// dbr_tune_rg
	{0x307F, 0x08}, 	// dbr_tune_fdb
	{0x3080, 0x04}, 	// dbr_tune_ntg 
	{0x3073, 0x73}, 	// comp1_bias, comp2_bias
	{0x3074, 0x45}, 	// pix_bias, pix_bias_boost
	{0x3075, 0xD4}, 	// clp_lvl
	{0x3085, 0xF0}, 	// rdv_option; LOB_PLA enable
	{0x3068, 0x55}, 	// ms[15:8]; x4~
	{0x3069, 0x00}, 	// ms[7:0]; x1~x4
	{0x3063, 0x08}, 	// cd{0x_option[15:8]; [11]ldb nmos sw enable=1
	{0x3064, 0x00}, 	// cds_option[7:0]; 
	{0x3010, 0x04}, 	// FD start 2->4 added by jay for low lux fluctuation
		
	{0x3247, 0x11},   // [4] fadlc_blst_en
	{0x3083, 0x00},   // blst_en_cintr = 16
	{0x3084, 0x10},
	
	{0x0306, 0x00}, // pll multiplier
	{0x0307, 0xb7},

	{0x3351, 0x02},
	{0x3352, 0xDC},
	{0x3353, 0x00},
	{0x3354, 0x00},
		
	{0x7339, 0x03}, // [2]dphy_en1, [1]dphy_en0, [0] dhpy_en_clk

	{0x0202, 0x04}, //added by Jay coarse int time
	{0x0203, 0x5c}, //6D}, //1116
	{0x0204, 0x00}, //added by Jay global gain		/*indoor index: 33ms, x1.059*/
	{0x0205, 0x22},
	
	{0x0344, 0x00}, // x_addr_start MSB
	{0x0345, 0x00}, // x_addr_start LSB
	{0x0346, 0x00}, // y_addr_start MSB
	{0x0347, 0x02}, // y_addr_start LSB
	
	{0x0348, 0x07}, // x_addr_end MSB
	{0x0349, 0x8F}, // x_addr_end LSB
	{0x034a, 0x04}, // y_addr_end MSB
	{0x034b, 0x43}, // y_addr_end LSB
		
	{0x034c, 0x07}, // x_output_size MSB
	{0x034d, 0x90}, // x_output_size LSB
	{0x034e, 0x04}, // y_output_size MSB
	{0x034f, 0x42}, // y_output_size LSB
};

static struct msm_camera_i2c_reg_conf s5k6b2yx_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf s5k6b2yx_stop_settings[] = {
	{0x0100, 0x00},
};

static struct msm_camera_i2c_reg_conf s5k6b2yx_start_settings_vision[] = {
	{0x4100, 0x01},
};

static struct msm_camera_i2c_reg_conf s5k6b2yx_stop_settings_vision[] = {
	{0x4100, 0x00},
};

static struct msm_camera_i2c_reg_conf s5k6b2yx_groupon_settings[] = {
	{0x104, 0x01},
};

static struct msm_camera_i2c_reg_conf s5k6b2yx_groupoff_settings[] = {
	{0x104, 0x00},
};
static struct v4l2_subdev_info s5k6b2yx_subdev_info[] = {
	{
	.code = V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt = 1,
	.order = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array s5k6b2yx_confs[] = {
	{&s5k6b2yx_prev_snap_settings[0],
	ARRAY_SIZE(s5k6b2yx_prev_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},

	{&s5k6b2yx_vision_settings[0],
	ARRAY_SIZE(s5k6b2yx_vision_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},

	{&s5k6b2yx_pip_prev_snap_settings[0], //PIP 24 FPS setting for > 1.0
	ARRAY_SIZE(s5k6b2yx_pip_prev_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},

	{&s5k6b2yx_prev_snap_settings_rev0[0],	//JAY add for rev 1.0
	ARRAY_SIZE(s5k6b2yx_prev_snap_settings_rev0), 0, MSM_CAMERA_I2C_BYTE_DATA},

	{&s5k6b2yx_pip_prev_snap_settings_rev0[0], //PIP 24 FPS setting for rev 1.0
	ARRAY_SIZE(s5k6b2yx_pip_prev_snap_settings_rev0), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_sensor_output_info_t s5k6b2yx_dimensions[] = {
	{
#if 0 //CHANGED BY jay because of changing sensor vendor recommendation
		.x_output = 0x790,/*1936*/
		.y_output = 0x448,/*1096*/
		.line_length_pclk = 0x868,
		.frame_length_lines = 0x45C,
		.vt_pixel_clk = 71984400,
		.op_pixel_clk = 91500000,
		.binning_factor = 1,
#else
		.x_output = 0x790,/*1936*/
		.y_output = 0x442,/*1090*/
		.line_length_pclk = 0x868,
		.frame_length_lines = 0x45C,
		.vt_pixel_clk = 73200000,
		.op_pixel_clk = 73200000,
		.binning_factor = 1,
#endif
	},
/*176x104 vision*/
	{
		.x_output = 176,
		.y_output = 104,
		.line_length_pclk = 200,	/* arbitrary values   */
		.frame_length_lines = 120, 	/* currently not used */
		.vt_pixel_clk = 1500000,
		.op_pixel_clk = 1500000,
		.binning_factor = 1,
	},
/*PIP setting*/
	{
		.x_output = 0x790,/*1936*/
		.y_output = 0x442,/*1090*/
		.line_length_pclk = 0x868,
		.frame_length_lines = 0x45C,
		.vt_pixel_clk = 58400000,
		.op_pixel_clk = 58400000,
		.binning_factor = 1,
	},
};

static struct msm_sensor_output_reg_addr_t s5k6b2yx_reg_addr = {
	.x_output = 0x034c,
	.y_output = 0x034e,
	.line_length_pclk = 0x0342,
	.frame_length_lines = 0x0340,
};

static struct msm_sensor_id_info_t s5k6b2yx_id_info = {
	.sensor_id_reg_addr = 0x0000,
	.sensor_id = 0x6B20,
};

static struct msm_sensor_exp_gain_info_t s5k6b2yx_exp_gain_info = {
	.coarse_int_time_addr = 0x0202,
	.global_gain_addr = 0x0204,
	.vert_offset = 4,
};

static const struct i2c_device_id s5k6b2yx_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&s5k6b2yx_s_ctrl},
	{ }
};

static struct i2c_driver s5k6b2yx_i2c_driver = {
	.id_table = s5k6b2yx_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client s5k6b2yx_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static enum msm_camera_vreg_name_t s5k6b2yx_veg_seq[] = {
	CAM_VANA,
	CAM_VDIG,
	CAM_VIO,
};

static struct msm_cam_clk_info cam_clk_info[] = {
	{"cam_clk", MSM_SENSOR_MCLK_24HZ},
};

static int s5k6b2yx_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
/*	int err;
	u32 int_factor;
	int temp = 0; */
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	CAM_DEBUG("E");
	CAM_DEBUG("s5k6b2yx_sensor_power_up(1) : i2c_scl: %d, i2c_sda: %d\n",
		 gpio_get_value(85), gpio_get_value(84));


	rc = msm_camera_request_gpio_table(data, 1);
	if (rc < 0)
		pr_err("%s: request gpio failed\n", __func__);

	CAM_DEBUG("s5k6b2yx_sensor_power_up(2) : i2c_scl: %d, i2c_sda: %d\n",
		 gpio_get_value(85), gpio_get_value(84));

	/* Power on */
	data->sensor_platform_info->sensor_power_on();

	if (data->sensor_platform_info->i2c_conf &&
		data->sensor_platform_info->i2c_conf->use_i2c_mux)
		msm_sensor_enable_i2c_mux(data->sensor_platform_info->i2c_conf);

	usleep(1200);

	CAM_DEBUG("s5k6b2yx_sensor_power_up(3) : i2c_scl: %d, i2c_sda: %d\n",
		 gpio_get_value(85), gpio_get_value(84));

	/* VT_CAM_RESET */
	data->sensor_platform_info->
		sensor_pmic_gpio_ctrl(data->sensor_platform_info->reset, 1);

	usleep(50);

	/* MCLK */
	if (s_ctrl->clk_rate != 0)
		cam_clk_info->clk_rate = s_ctrl->clk_rate;

	rc = msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_clk_info, s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 1);
	if (rc < 0)
		pr_err("%s: clk enable failed\n", __func__);

	CAM_DEBUG("s5k6b2yx_sensor_power_up(4) : i2c_scl: %d, i2c_sda: %d\n",
		 gpio_get_value(85), gpio_get_value(84));

	usleep(5000);
	CAM_DEBUG("X");

	return rc;
}

static int s5k6b2yx_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	CAM_DEBUG("E");

	if (data->sensor_platform_info->i2c_conf &&
		data->sensor_platform_info->i2c_conf->use_i2c_mux)
		msm_sensor_disable_i2c_mux(
			data->sensor_platform_info->i2c_conf);

	/* MCLK */
	rc = msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_clk_info, s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 0);

	/* VT_CAM_RESET */
	data->sensor_platform_info->
		sensor_pmic_gpio_ctrl(data->sensor_platform_info->reset, 0);

	/* STBY */
	data->sensor_platform_info->
		sensor_pmic_gpio_ctrl(data->sensor_platform_info->stby, 0);

	/* Power off */
	data->sensor_platform_info->sensor_power_off();

	msm_camera_request_gpio_table(data, 0);
	s_ctrl->vision_mode_flag=0;

	CAM_DEBUG("X");

	return rc;
}

/* Switch to low power vision mode */
static int s5k6b2yx_sensor_set_streaming_mode(
		struct msm_sensor_ctrl_t *s_ctrl, int32_t vision_mode_enable) {
	int rc = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;

		CAM_DEBUG("stop streaming");
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);

	CAM_DEBUG("vision_mode_enable=%d: E", vision_mode_enable);
	if(vision_mode_enable) { /*switch from normal/dual to vision mode */
		CAM_DEBUG("set X_SHUTDOWN pin to low");
		data->sensor_platform_info->
			sensor_pmic_gpio_ctrl(data->sensor_platform_info->reset, 0);
		usleep(1050);
		CAM_DEBUG("set VIS_STBY pin to high");
		data->sensor_platform_info->
			sensor_pmic_gpio_ctrl(data->sensor_platform_info->stby, 1);

		CAM_DEBUG("change stream config arrays");
		s_ctrl->msm_sensor_reg->start_stream_conf = s5k6b2yx_start_settings_vision;
		s_ctrl->msm_sensor_reg->start_stream_conf_size = ARRAY_SIZE(s5k6b2yx_start_settings_vision);
		s_ctrl->msm_sensor_reg->stop_stream_conf = s5k6b2yx_stop_settings_vision;
		s_ctrl->msm_sensor_reg->stop_stream_conf_size = ARRAY_SIZE(s5k6b2yx_stop_settings_vision);
		s_ctrl->vision_mode_flag = 1;
                /*set FPS based on flag set from user space driver*/
                if (vision_mode_enable == VISION_MODE_SET_FPS_5)
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
							VISION_MODE_SET_FPS_ADDR,
							VISION_MODE_FPS_5_VAL,
							MSM_CAMERA_I2C_BYTE_DATA);
		else if (vision_mode_enable == VISION_MODE_SET_FPS_10)	
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
							VISION_MODE_SET_FPS_ADDR,
							VISION_MODE_FPS_10_VAL,
							MSM_CAMERA_I2C_BYTE_DATA);


	} else { /*switch from vision to normal/dual mode */
		CAM_DEBUG("set VIS_STBY pin to low");
		data->sensor_platform_info->
			sensor_pmic_gpio_ctrl(data->sensor_platform_info->stby, 0);
		usleep(1050);
		CAM_DEBUG("set X_SHUTDOWN pin to high");
		data->sensor_platform_info->
			sensor_pmic_gpio_ctrl(data->sensor_platform_info->reset, 1);

		CAM_DEBUG("change stream config arrays");
		s_ctrl->msm_sensor_reg->start_stream_conf = s5k6b2yx_start_settings;
		s_ctrl->msm_sensor_reg->start_stream_conf_size = ARRAY_SIZE(s5k6b2yx_start_settings);
		s_ctrl->msm_sensor_reg->stop_stream_conf = s5k6b2yx_stop_settings;
		s_ctrl->msm_sensor_reg->stop_stream_conf_size = ARRAY_SIZE(s5k6b2yx_stop_settings);
		s_ctrl->vision_mode_flag = 0;
	}
	CAM_DEBUG("rc=%d : X", rc);
	return rc;
}

/*
 * Change normal/backlight AEC for vision mode
 * aec_mode:
 * 	0 = normal
 * 	1 = backlight
 */
int s5k6b2yx_sensor_set_vision_ae_control(
				struct msm_sensor_ctrl_t *s_ctrl, int ae_mode) {

	if(s_ctrl->vision_mode_flag == 0) {
		cam_err("Error: sensor not in vision mode, cannot set AE.");
		return -1;
	}
	if(ae_mode == 0) {
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
							VISION_MODE_AE_REG_ADDR,
							VISION_MODE_AE_NORMAL,
							MSM_CAMERA_I2C_BYTE_DATA);
		CAM_DEBUG("normal mode AEC set");
	} else {
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
							VISION_MODE_AE_REG_ADDR,
							VISION_MODE_AE_BACKLIGHT,
							MSM_CAMERA_I2C_BYTE_DATA);
		CAM_DEBUG("backlight mode AEC set");
	}
	return 0;
}

static ssize_t s5k6b2yx_camera_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "S5K6B2YX");
}

static ssize_t s5k6b2yx_camera_fw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s %s\n", "S5K6B2YX", "S5K6B2YX");
}

static DEVICE_ATTR(front_camtype, S_IRUGO, s5k6b2yx_camera_type_show, NULL);
static DEVICE_ATTR(front_camfw, S_IRUGO, s5k6b2yx_camera_fw_show, NULL);

static int __init s5k6b2yx_sensor_init_module(void)
{
	struct device *cam_dev_front = NULL;

	cam_dev_front =
	device_create(camera_class, NULL, 0, NULL, "front");
	if (IS_ERR(cam_dev_front)) {
		cam_err("failed to create device cam_dev_front!\n");
		return 0;
	}

	if (device_create_file
	(cam_dev_front, &dev_attr_front_camtype) < 0) {
		cam_err("failed to create device file, %s\n",
		dev_attr_front_camtype.attr.name);
	}

	if (device_create_file
	(cam_dev_front, &dev_attr_front_camfw) < 0) {
		cam_err("failed to create device file, %s\n",
		dev_attr_front_camfw.attr.name);
	}

	return i2c_add_driver(&s5k6b2yx_i2c_driver);
}

static struct v4l2_subdev_core_ops s5k6b2yx_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops s5k6b2yx_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops s5k6b2yx_subdev_ops = {
	.core = &s5k6b2yx_subdev_core_ops,
	.video  = &s5k6b2yx_subdev_video_ops,
};

static struct msm_sensor_fn_t s5k6b2yx_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_setting = msm_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = s5k6b2yx_sensor_power_up,
	.sensor_power_down = s5k6b2yx_sensor_power_down,
	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines1,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_set_streaming_mode = s5k6b2yx_sensor_set_streaming_mode,
	.sensor_set_vision_ae_control = s5k6b2yx_sensor_set_vision_ae_control,
};

static struct msm_sensor_reg_t s5k6b2yx_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = s5k6b2yx_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(s5k6b2yx_start_settings),
	.stop_stream_conf = s5k6b2yx_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(s5k6b2yx_stop_settings),
	.group_hold_on_conf = s5k6b2yx_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(s5k6b2yx_groupon_settings),
	.group_hold_off_conf = s5k6b2yx_groupoff_settings,
	.group_hold_off_conf_size = ARRAY_SIZE(s5k6b2yx_groupoff_settings),
	/*init settings clubbed with prev-snap settings for normal power mode
	* because sensor needs to load two settings depending on mode */
	.init_settings = NULL,
	.init_size = 0,
	.mode_settings = &s5k6b2yx_confs[0],
	.output_settings = &s5k6b2yx_dimensions[0],
	.num_conf = ARRAY_SIZE(s5k6b2yx_confs),
};

static struct msm_sensor_ctrl_t s5k6b2yx_s_ctrl = {
	.msm_sensor_reg = &s5k6b2yx_regs,
	.sensor_i2c_client = &s5k6b2yx_sensor_i2c_client,
	.sensor_i2c_addr = 0x6A,
	.vreg_seq = s5k6b2yx_veg_seq,
	.num_vreg_seq = ARRAY_SIZE(s5k6b2yx_veg_seq),
	.sensor_output_reg_addr = &s5k6b2yx_reg_addr,
	.sensor_id_info = &s5k6b2yx_id_info,
	.sensor_exp_gain_info = &s5k6b2yx_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.msm_sensor_mutex = &s5k6b2yx_mut,
	.sensor_i2c_driver = &s5k6b2yx_i2c_driver,
	.sensor_v4l2_subdev_info = s5k6b2yx_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(s5k6b2yx_subdev_info),
	.sensor_v4l2_subdev_ops = &s5k6b2yx_subdev_ops,
	.func_tbl = &s5k6b2yx_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
	.vision_mode_flag = 0,
};

module_init(s5k6b2yx_sensor_init_module);
MODULE_DESCRIPTION("Samsung 2MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
