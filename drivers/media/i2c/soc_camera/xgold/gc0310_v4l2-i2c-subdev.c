/*
 * gc0310 sensor driver
 *
 * Copyright (C) 2012-2014 Intel Mobile Communications GmbH
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Note:
 *    07/01/2014: new implementation using v4l2-subdev
 *                        instead of v4l2-int-device.
 */
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include "gc_camera_module.h"

#define GC0310_DRIVER_NAME "gc0310"


#define GC0310_AEC_ANALOG_GAIN_REG 0x71	/* Bit 8 */
#define GC0310_AEC_ANALOG_GAIN_COL_REG 0x48


#define GC0310_AEC_EXPO_COARSE_2ND_REG 0x03	/* Exposure Bits 8-15 */
#define GC0310_AEC_EXPO_COARSE_1ST_REG 0x04	/* Exposure Bits 0-7 */


#define GC0310_FETCH_2ND_BYTE_EXP(VAL) ((VAL >> 8) & 0x0F)	/* 4 Bits */
#define GC0310_FETCH_1ST_BYTE_EXP(VAL) (VAL & 0x000000FF)	/* 8 Bits */

#define GC0310_PID_H_ADDR     0xf0
#define GC0310_PID_L_ADDR     0xf1

#define GC0310_DUMMY_REG 0x05
#define GC0310_HORIZONTAL_DUMMY_LOW_REG 0x06
#define GC0310_VERTICAL_DUMMY_HIGH_REG 0x07
#define GC0310_VERTICAL_DUMMY_LOW_REG 0x08
#define GC0310_SH_DELAY_REG 0x11
#define GC0310_TIMING_FRAME_LENGTH_LINES_HIGH_REG 0x0d
#define GC0310_TIMING_FRAME_LENGTH_LINES_LOW_REG 0x0e
#define GC0310_TIMING_LINE_LENGTH_PCKL_HIGH_REG 0xf
#define GC0310_TIMING_LINE_LENGTH_PCKL_LOW_REG 0x10
#define GC0310_COARSE_INTEGRATION_TIME_MIN 1
#define GC0310_COARSE_INTEGRATION_TIME_MAX_MARGIN 6
#define GC0310_FINE_INTEGRATION_TIME_MIN 0
#define GC0310_FINE_INTEGRATION_TIME_MAX_MARGIN 0


#define GC0310_HORIZONTAL_START_REG 0x0c
#define GC0310_VERTICAL_START_REG 0x0a

#define GC0310_EXT_CLK 26000000


/* product ID */
#define GC0310_PID_MAGIC 0xa310

#define GC0310_ANALOG_GAIN_1 64  /* 1.00x*/
#define GC0310_ANALOG_GAIN_2 90  /* 1.40625x*/
#define GC0310_ANALOG_GAIN_3 128  /* 2.00x*/
#define GC0310_ANALOG_GAIN_4 162  /* 2.53125x*/
#define GC0310_ANALOG_GAIN_5 232  /* 3.625x*/
#define GC0310_ANALOG_GAIN_6 355  /* 5.60x*/
#define GC0310_ANALOG_GAIN_7 482  /* 8.00x*/


/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */
static const struct gc_camera_module_reg gc0310_init_tab_vga_30fps[] = {
	#if 1
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0xf0},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0xf0},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfc, 0x0e},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfc, 0x0e},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xf2, 0x80},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xf3, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xf7, 0x33},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xf8, 0x04},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xf9, 0x0e},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfa, 0x11},

	/*MIPI*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x03},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x01, 0x03},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x02, 0x11},/*VOD*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x03, 0x94},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x04, 0x10},/*low bit(0~7)*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x05, 0x00},/*high bit(8,9)*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x06, 0x80},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x11, 0x2a},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x12, 0x90},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x13, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x15, 0x11},/*discontinuous by line*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x17, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x40, 0x08},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x41, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x42, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x43, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x21, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x22, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x23, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x26, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x29, 0x00},/*prepare*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2a, 0x03},/*zero*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2b, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x00},

	/*CISCTL reg*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x00, 0x2f},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x01, 0x0f},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x02, 0x04},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x4f, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x03, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x04, 0x40},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x05, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x06, 0x26},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x07, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x08, 0x3c},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x09, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0a, 0x04},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0b, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0c, 0x04},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0d, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0e, 0xf2},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0f, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x10, 0x94},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x17, 0x14},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x18, 0x1a},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x19, 0x14},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x1b, 0x48},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x1c, 0x6c},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x1e, 0x6b},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x1f, 0x28},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x20, 0x8b},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x21, 0x49},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x22, 0xd0},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x23, 0x04},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x24, 0x16},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x34, 0x20},

	/*BLK*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x26, 0x23},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x28, 0xff},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x29, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x33, 0x18},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x37, 0x20},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2a, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2b, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2c, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2d, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2e, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2f, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x30, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x31, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x32, 0x04},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x47, 0x80},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x4e, 0x66},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xa8, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xa9, 0x80},

	/*ISP reg*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x40, 0x06}, /* dn & dd */
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x41, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x42, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x44, 0x18}, /* dndd out */
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x46, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x49, 0x03},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x4c, 0x20}, /*test pattern 24*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x50, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x55, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x56, 0xf0},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x57, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x58, 0x90},

	/*GAIN*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x70, 0x50},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x71, 0x20},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x72, 0x40},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x5a, 0x98},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x5b, 0xdc},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x5c, 0xfe},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x77, 0x40},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x78, 0x40},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x79, 0x40},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x48, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0a, 0x45},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x3e, 0x40},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x3f, 0x5c},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x40, 0x7b},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x41, 0xbd},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x42, 0xf6},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x43, 0x63},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x03, 0x60},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x44, 0x03},

	/*dark sum*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x45, 0xa4},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x46, 0xf0},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x48, 0x03},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x4f, 0x60},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x00}

#endif

};

/* ======================================================================== */

static struct gc_camera_module_config gc0310_configs[] = {
	{
		.name = "vga_30fps",
		.frm_fmt = {
			.width = 640,
			.height = 480,
			.code = V4L2_MBUS_FMT_SRGGB8_1X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 30
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)gc0310_init_tab_vga_30fps,
		.reg_table_num_entries =
			sizeof(gc0310_init_tab_vga_30fps)
			/
			sizeof(gc0310_init_tab_vga_30fps[0]),
		.v_blanking_time_us = 2000 /*empirically measured time*/
	},
};

/*--------------------------------------------------------------------------*/


static int gc0310_g_VTS(struct gc_camera_module *cam_mod, u32 *vts)
{
	u32 msb, lsb;
	int ret;

	ret = gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_TIMING_FRAME_LENGTH_LINES_HIGH_REG,
		&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_TIMING_FRAME_LENGTH_LINES_LOW_REG,
		&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	*vts = ((msb&0x01) << 8) | lsb;

	return 0;
err:
	gc_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc0310_auto_adjust_fps(struct gc_camera_module *cam_mod,
	u32 exp_time)
{
	int ret;
	u32 vts;

	if ((cam_mod->exp_config.exp_time +
		GC0310_COARSE_INTEGRATION_TIME_MAX_MARGIN) >
		cam_mod->vts_min) {
		vts = cam_mod->exp_config.exp_time +
			GC0310_COARSE_INTEGRATION_TIME_MAX_MARGIN;
	}	else {
		vts = cam_mod->vts_min;
	}
	gc_camera_module_pr_debug(cam_mod, "vts (%d)\n", vts);

	ret = gc_camera_module_write_reg(cam_mod,
		GC0310_VERTICAL_DUMMY_LOW_REG,
		vts & 0xFF);
	ret |= gc_camera_module_write_reg(cam_mod,
		GC0310_VERTICAL_DUMMY_HIGH_REG,
		(vts >> 8) & 0x0f);

	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_err(cam_mod,
				"failed with error (%d)\n", ret);
	else
		gc_camera_module_pr_debug(cam_mod,
				"vts = %d\n", vts);

	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc0310_write_aec(struct gc_camera_module *cam_mod)
{
	int ret = 0;
	int temp = 0;

	gc_camera_module_pr_debug(cam_mod,
		"exp_time = %d, gain = %d, flash_mode = %d\n",
		cam_mod->exp_config.exp_time,
		cam_mod->exp_config.gain,
		cam_mod->exp_config.flash_mode);

	/* if the sensor is already streaming, write to shadow registers,
		if the sensor is in SW standby, write to active registers,
		if the sensor is off/registers are not writeable, do nothing */
	if ((cam_mod->state == GC_CAMERA_MODULE_SW_STANDBY) ||
		(cam_mod->state == GC_CAMERA_MODULE_STREAMING)) {
		u32 a_gain = cam_mod->exp_config.gain;
		u32 exp_time = cam_mod->exp_config.exp_time;

		if (a_gain < 0x40)
			a_gain = 0x40;


		if ((GC0310_ANALOG_GAIN_1 <= a_gain) &&
			(a_gain < GC0310_ANALOG_GAIN_2)) {
			ret |= gc_camera_module_write_reg(cam_mod,
				GC0310_AEC_ANALOG_GAIN_COL_REG,
				0x00);
			temp = a_gain/2;
			ret |= gc_camera_module_write_reg(cam_mod,
				GC0310_AEC_ANALOG_GAIN_REG,
				temp);
		}	else if ((GC0310_ANALOG_GAIN_2 <= a_gain) &&
			(a_gain < GC0310_ANALOG_GAIN_3)) {
			ret |= gc_camera_module_write_reg(cam_mod,
				GC0310_AEC_ANALOG_GAIN_COL_REG,
				0x01);
			temp = 32 * a_gain / GC0310_ANALOG_GAIN_2 +
			(((32*a_gain % GC0310_ANALOG_GAIN_2) >=
			(GC0310_ANALOG_GAIN_2 / 2)) ? 1 : 0);
			ret |= gc_camera_module_write_reg(cam_mod,
				GC0310_AEC_ANALOG_GAIN_REG,
				temp);
		}	else if ((GC0310_ANALOG_GAIN_3 <= a_gain) &&
			(a_gain < GC0310_ANALOG_GAIN_4)) {
			ret |= gc_camera_module_write_reg(cam_mod,
				GC0310_AEC_ANALOG_GAIN_COL_REG,
				0x02);
			temp = 32 * a_gain / GC0310_ANALOG_GAIN_3 +
			(((32 * a_gain % GC0310_ANALOG_GAIN_3) >=
			(GC0310_ANALOG_GAIN_3 / 2)) ? 1 : 0);
			ret |= gc_camera_module_write_reg(cam_mod,
				GC0310_AEC_ANALOG_GAIN_REG,
				temp);
		}	else if ((GC0310_ANALOG_GAIN_4 <= a_gain) &&
			(a_gain < GC0310_ANALOG_GAIN_5)) {
			ret |= gc_camera_module_write_reg(cam_mod,
				GC0310_AEC_ANALOG_GAIN_COL_REG,
				0x03);
			temp = 32 * a_gain / GC0310_ANALOG_GAIN_4 +
			(((32 * a_gain % GC0310_ANALOG_GAIN_4) >=
			(GC0310_ANALOG_GAIN_4 / 2)) ? 1 : 0);
			ret |= gc_camera_module_write_reg(cam_mod,
				GC0310_AEC_ANALOG_GAIN_REG,
				temp);
		}	 else if (GC0310_ANALOG_GAIN_5 <= a_gain) {
			ret |= gc_camera_module_write_reg(cam_mod,
				GC0310_AEC_ANALOG_GAIN_COL_REG,
				0x04);
			temp = 32 * a_gain / GC0310_ANALOG_GAIN_5 +
			(((32 * a_gain % GC0310_ANALOG_GAIN_5) >=
			(GC0310_ANALOG_GAIN_5 / 2)) ? 1 : 0);
			ret |= gc_camera_module_write_reg(cam_mod,
				GC0310_AEC_ANALOG_GAIN_REG,
				temp);
		}



		ret |= gc_camera_module_write_reg(cam_mod,
			GC0310_AEC_EXPO_COARSE_2ND_REG,
			GC0310_FETCH_2ND_BYTE_EXP(exp_time));
		ret |= gc_camera_module_write_reg(cam_mod,
			GC0310_AEC_EXPO_COARSE_1ST_REG,
			GC0310_FETCH_1ST_BYTE_EXP(exp_time));

		if (!IS_ERR_VALUE(ret) && cam_mod->auto_adjust_fps)
			ret = gc0310_auto_adjust_fps(cam_mod,
				cam_mod->exp_config.exp_time);

	}

	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc0310_g_ctrl(struct gc_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	gc_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_FLASH_LED_MODE:
		/* nothing to be done here */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc0310_g_timings(struct gc_camera_module *cam_mod,
	struct gc_camera_module_timings *timings)
{
	int ret = 0;
	int reg_val;
	int temp = 0;


	if (IS_ERR_OR_NULL(cam_mod->active_config))
		goto err;


	timings->vt_pix_clk_freq_hz = GC0310_EXT_CLK / 2;


	ret = gc0310_g_VTS(cam_mod, &timings->frame_length_lines);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_TIMING_LINE_LENGTH_PCKL_HIGH_REG,
		&reg_val)))
		goto err;

	timings->line_length_pck = reg_val << 8;

	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_TIMING_LINE_LENGTH_PCKL_LOW_REG,
		&reg_val)))
		goto err;

	timings->line_length_pck |= reg_val;

	timings->coarse_integration_time_min =
	GC0310_COARSE_INTEGRATION_TIME_MIN;
	timings->coarse_integration_time_max_margin =
	GC0310_COARSE_INTEGRATION_TIME_MAX_MARGIN;


	timings->fine_integration_time_min =
	GC0310_FINE_INTEGRATION_TIME_MIN;
	timings->fine_integration_time_max_margin =
	GC0310_FINE_INTEGRATION_TIME_MAX_MARGIN;

	timings->binning_factor_x = 1;
	timings->binning_factor_y = 1;

	/* Get the cropping and output resolution to ISP for this mode. */
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_HORIZONTAL_START_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_start = reg_val;

	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_VERTICAL_START_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_start = reg_val;

	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_TIMING_LINE_LENGTH_PCKL_HIGH_REG,
		&reg_val)))
		goto err;
	temp = (reg_val << 8) & 0xff00;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_TIMING_LINE_LENGTH_PCKL_LOW_REG,
		&reg_val)))
		goto err;
	temp |= reg_val;

	timings->sensor_output_width = temp;
	timings->crop_horizontal_end = timings->crop_horizontal_start + temp;

	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_TIMING_FRAME_LENGTH_LINES_HIGH_REG,
		&reg_val)))
		goto err;
	temp = (reg_val << 8) & 0xff00;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_TIMING_FRAME_LENGTH_LINES_LOW_REG,
		&reg_val)))
		goto err;
	temp |= reg_val;

	timings->sensor_output_height = temp;
	timings->crop_vertical_end = timings->crop_vertical_start + temp;

	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_DUMMY_REG,
		&reg_val)))
		goto err;
	temp = (reg_val & 0x0f) << 8;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_HORIZONTAL_DUMMY_LOW_REG,
		&reg_val)))
		goto err;
	temp += reg_val;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_SH_DELAY_REG,
		&reg_val)))
		goto err;
	temp += reg_val;
	timings->line_length_pck = timings->sensor_output_width + temp + 4;

	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_DUMMY_REG,
		&reg_val)))
		goto err;
	temp = (reg_val & 0xf0) << 4;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(
		cam_mod,
		GC0310_VERTICAL_DUMMY_LOW_REG,
		&reg_val)))
		goto err;
	temp += reg_val;
	timings->frame_length_lines = timings->sensor_output_height + temp;


	return ret;
err:
	gc_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc0310_s_ctrl(struct gc_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	gc_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = gc0310_write_aec(cam_mod);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_debug(cam_mod,
			"failed with error (%d) 0x%x\n", ret, ctrl_id);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc0310_s_ext_ctrls(struct gc_camera_module *cam_mod,
				 struct gc_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if (ctrls->count == 1)
		ret = gc0310_s_ctrl(cam_mod, ctrls->ctrls[0].id);
	else if (ctrls->count == 2 &&
		((ctrls->ctrls[0].id == V4L2_CID_GAIN &&
		ctrls->ctrls[1].id == V4L2_CID_EXPOSURE) ||
		(ctrls->ctrls[1].id == V4L2_CID_GAIN &&
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE)))
		ret = gc0310_write_aec(cam_mod);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc0310_start_streaming(struct gc_camera_module *cam_mod)
{
	int ret = 0;

	gc_camera_module_pr_debug(cam_mod, "\n");

	ret = gc0310_write_aec(cam_mod);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (IS_ERR_VALUE(gc_camera_module_write_reg(cam_mod, 0xfe, 0x03)))
		goto err;
	/*10bit 0x92, 8 bit 0x94*/
	if (IS_ERR_VALUE(gc_camera_module_write_reg(cam_mod, 0x10, 0x94)))
		goto err;
	if (IS_ERR_VALUE(gc_camera_module_write_reg(cam_mod, 0x01, 0x03)))
		goto err;
	if (IS_ERR_VALUE(gc_camera_module_write_reg(cam_mod, 0xfe, 0x00)))
		goto err;


	return 0;
err:
	gc_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc0310_stop_streaming(struct gc_camera_module *cam_mod)
{
	int ret = 0;

	gc_camera_module_pr_debug(cam_mod, "\n");

	ret = gc_camera_module_write_reg(cam_mod, 0xfe, 0x03);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = gc_camera_module_write_reg(cam_mod, 0x10, 0x84);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = gc_camera_module_write_reg(cam_mod, 0x01, 0x00);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = gc_camera_module_write_reg(cam_mod, 0xfe, 0x00);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	gc_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc0310_check_camera_id(struct gc_camera_module *cam_mod)
{
	u32 val, pid = 0;
	int ret = 0;

	gc_camera_module_pr_debug(cam_mod, "\n");

	ret = gc_camera_module_read_reg(cam_mod, 1, GC0310_PID_H_ADDR, &val);
	gc_camera_module_pr_debug(cam_mod, "gc0310 id high = 0x%02x\n", val);
	pid = (val << 8) & 0xff00;
	ret = gc_camera_module_read_reg(cam_mod, 1, GC0310_PID_L_ADDR, &val);
	gc_camera_module_pr_debug(cam_mod, "gc0310 id low = 0x%02x\n", val);
	pid |= val;

	if (IS_ERR_VALUE(ret)) {
		gc_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if (pid == GC0310_PID_MAGIC) {
		gc_camera_module_pr_debug(cam_mod,
			"successfully detected camera ID 0x%02x\n",
			pid);
	}	else {
		gc_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x, detected 0x%02x\n",
			GC0310_PID_MAGIC, pid);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	gc_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops gc0310_camera_module_core_ops = {
	.g_ctrl = gc_camera_module_g_ctrl,
	.s_ctrl = gc_camera_module_s_ctrl,
	.s_ext_ctrls = gc_camera_module_s_ext_ctrls,
	.s_power = gc_camera_module_s_power,
	.ioctl = gc_camera_module_ioctl
};

static struct v4l2_subdev_video_ops gc0310_camera_module_video_ops = {
	.enum_frameintervals = gc_camera_module_enum_frameintervals,
	.s_mbus_fmt = gc_camera_module_s_fmt,
	.g_mbus_fmt = gc_camera_module_g_fmt,
	.try_mbus_fmt = gc_camera_module_try_fmt,
	.s_frame_interval = gc_camera_module_s_frame_interval,
	.s_stream = gc_camera_module_s_stream
};

static struct v4l2_subdev_ops gc0310_camera_module_ops = {
	.core = &gc0310_camera_module_core_ops,
	.video = &gc0310_camera_module_video_ops
};

static struct gc_camera_module gc0310;

static struct gc_camera_module_custom_config gc0310_custom_config = {
	.start_streaming = gc0310_start_streaming,
	.stop_streaming = gc0310_stop_streaming,
	.s_ctrl = gc0310_s_ctrl,
	.s_ext_ctrls = gc0310_s_ext_ctrls,
	.g_ctrl = gc0310_g_ctrl,
	.g_timings = gc0310_g_timings,
	.check_camera_id = gc0310_check_camera_id,
	.configs = gc0310_configs,
	.num_configs = sizeof(gc0310_configs) / sizeof(gc0310_configs[0]),
	.power_up_delays_ms = {0, 20, 0}
};

static int __init gc0310_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	dev_info(&client->dev, "probing...\n");

	v4l2_i2c_subdev_init(&gc0310.sd, client, &gc0310_camera_module_ops);
	ret = gc_camera_module_init(&gc0310,
			&gc0310_custom_config);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = gc_camera_module_s_power(&gc0310.sd, 1);
	if (IS_ERR_VALUE(ret))
		goto err;

	gc_camera_module_s_power(&gc0310.sd, 0);
	dev_info(&client->dev, "probing successful\n");
	return 0;
err:
	dev_err(&client->dev, "probing failed with error (%d)\n", ret);
	gc_camera_module_release(&gc0310);
	return ret;
}

/* ======================================================================== */

static int __exit gc0310_remove(
	struct i2c_client *client)
{
	struct gc_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	gc_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id gc0310_id[] = {
	{ GC0310_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id gc0310_of_match[] = {
	{.compatible = "galaxycore," GC0310_DRIVER_NAME "-v4l2-i2c-subdev",},
	{},
};

MODULE_DEVICE_TABLE(i2c, gc0310_id);

static struct i2c_driver gc0310_i2c_driver = {
	.driver = {
		.name = GC0310_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gc0310_of_match
	},
	.probe = gc0310_probe,
	.remove = __exit_p(gc0310_remove),
	.id_table = gc0310_id,
};

module_i2c_driver(gc0310_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for gc0310");
MODULE_AUTHOR("Clark Yu");
MODULE_LICENSE("GPL");

