/*
 * ov2680 sensor driver
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
 *    09/25/2014: new implementation using v4l2-subdev
 *                        instead of v4l2-int-device.
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include "ov_camera_module.h"

#define OV2680_DRIVER_NAME "ov2680"

#define OV2680_FETCH_LSB_GAIN(VAL) (VAL & 0x00FF)
#define OV2680_FETCH_MSB_GAIN(VAL) ((VAL >> 8) & 0x1)	/* Only 1 bit */
#define OV2680_AEC_PK_LONG_GAIN_HIGH_REG 0x350a	/* Bit 8 */
#define OV2680_AEC_PK_LONG_GAIN_LOW_REG	 0x350b	/* Bits 0 -7 */

#define OV2680_AEC_PK_LONG_EXPO_3RD_REG 0x3500	/* Exposure Bits 16-19 */
#define OV2680_AEC_PK_LONG_EXPO_2ND_REG 0x3501	/* Exposure Bits 8-15 */
#define OV2680_AEC_PK_LONG_EXPO_1ST_REG 0x3502	/* Exposure Bits 0-7 */

#define OV2680_AEC_GROUP_UPDATE_ADDRESS 0x3208
#define OV2680_AEC_GROUP_UPDATE_START_DATA 0x00
#define OV2680_AEC_GROUP_UPDATE_END_DATA 0x10
#define OV2680_AEC_GROUP_UPDATE_END_LAUNCH 0xA0

#define OV2680_FETCH_3RD_BYTE_EXP(VAL) ((VAL >> 16) & 0xF)	/* 4 Bits */
#define OV2680_FETCH_2ND_BYTE_EXP(VAL) ((VAL >> 8) & 0xFF)	/* 8 Bits */
#define OV2680_FETCH_1ST_BYTE_EXP(VAL) (VAL & 0x000000FF)	/* 8 Bits */

#define OV2680_PIDH_ADDR     0x300A
#define OV2680_PIDL_ADDR     0x300B

/* High byte of product ID */
#define OV2680_PIDH_MAGIC 0x26
/* Low byte of product ID  */
#define OV2680_PIDL_MAGIC 0x80

#define OV2680_EXT_CLK 26000000
#define OV2680_PLL_PREDIV0_REG 0x3088
#define OV2680_PLL_PREDIV_REG  0x3080
#define OV2680_PLL_MUL_HIGH_REG 0x3081
#define OV2680_PLL_MUL_LOW_REG 0x3082
#define OV2680_PLL_SPDIV_REG 0x3086
#define OV2680_PLL_DIVSYS_REG 0x3084
#define OV2680_TIMING_VTS_HIGH_REG 0x380e
#define OV2680_TIMING_VTS_LOW_REG 0x380f
#define OV2680_TIMING_HTS_HIGH_REG 0x380c
#define OV2680_TIMING_HTS_LOW_REG 0x380d
#define OV2680_INTEGRATION_TIME_MARGIN 0
#define OV2680_TIMING_X_INC		0x3814
#define OV2680_TIMING_Y_INC		0x3815
#define OV2680_HORIZONTAL_START_HIGH_REG 0x3800
#define OV2680_HORIZONTAL_START_LOW_REG 0x3801
#define OV2680_VERTICAL_START_HIGH_REG 0x3802
#define OV2680_VERTICAL_START_LOW_REG 0x3803
#define OV2680_HORIZONTAL_END_HIGH_REG 0x3804
#define OV2680_HORIZONTAL_END_LOW_REG 0x3805
#define OV2680_VERTICAL_END_HIGH_REG 0x3806
#define OV2680_VERTICAL_END_LOW_REG 0x3807
#define OV2680_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x3808
#define OV2680_HORIZONTAL_OUTPUT_SIZE_LOW_REG 0x3809
#define OV2680_VERTICAL_OUTPUT_SIZE_HIGH_REG 0x380a
#define OV2680_VERTICAL_OUTPUT_SIZE_LOW_REG 0x380b
#define OV2680_H_WIN_OFF_HIGH_REG 0x3810
#define OV2680_H_WIN_OFF_LOW_REG 0x3811
#define OV2680_V_WIN_OFF_HIGH_REG 0x3812
#define OV2680_V_WIN_OFF_LOW_REG 0x3813

/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */
static const struct ov_camera_module_reg ov2680_init_tab_1600_1200_30fps[] = {
	/* software reset */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x01},
	/* gpio0 input, vsync input, fsin input */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0x00},
	/* drive strength = 0x01, bypass latch of hs_enable */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3016, 0x1c},
	/* MIPI 10-bit mode */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3018, 0x44},
	/* output raw */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3020, 0x00},

	/* For 1600X1200@30fps, MIPI 1 lane data rate 663M */
	/* PLL CTRL, PLL prediv0 = 1 */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3088, 0x01},
	/* PLL prediv=2 */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3080, 0x02},
	/* PLL multiplier = 51 */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3081, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3082, 0x33},
	/* PLL sys clk div = 10 */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3084, 0x09},
	/* PLL dac div = 5 */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3085, 0x04},
	/* PLL sp div = 1 */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3086, 0x00},
	/* exposure M */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501, 0x26},
	/* exposure L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3502, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350b, 0x36},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3600, 0xb4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3603, 0x35},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3604, 0x24},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3605, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3620, 0x26},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0x37},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3622, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3628, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3705, 0x3c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370c, 0x50},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370d, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3718, 0x88},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3720, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3721, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3722, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3723, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3738, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370a, 0x23},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3717, 0x58},
	/* x address start, 0x3800:[15:8], 0x3801:[7:0] */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3800, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0x00},
	/* y address start, 0x3802:[15:8], 0x3803:[7:0] */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3802, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0x00},
	/* x address end, 0x3804:[15:8], 0x3805:[7:0] */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0x4f},
	/* y address end, 0x3806:[15:8], 0x3807:[7:0] */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0xbf},

	/* output size 1600x1200 */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0xB0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0xa4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0x0E},
	/*ISP WIN offset, 0x3810, 0x3811, 0x3812, 0x3813 */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3810, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3812, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x08},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x11},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815, 0x11},
	/* VSYNC ENG ROW */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3819, 0x04},

	/* 0x3820 and 0x3821 flip and mirror */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x00},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4000, 0x41},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4001, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4003, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4008, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x0A},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4602, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x481f, 0x26},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4825, 0x32},
	/* MIPI global timing */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4837, 0x1E},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5002, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5080, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5081, 0x41},
};

/* ======================================================================== */

static struct ov_camera_module_config ov2680_configs[] = {
	{
		.name = "1600x1200_30fps",
		.frm_fmt = {
			.width = 1600,
			.height = 1200,
			.code = V4L2_MBUS_FMT_SBGGR10_1X10
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
		.reg_table = (void *)ov2680_init_tab_1600_1200_30fps,
		.reg_table_num_entries =
			sizeof(ov2680_init_tab_1600_1200_30fps)
			/
			sizeof(ov2680_init_tab_1600_1200_30fps[0]),
	    .v_blanking_time_us = 3000
	},
};

/*--------------------------------------------------------------------------*/

static int ov2680_write_aec(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod,
		"exp_time = %d, gain = %d, flash_mode = %d\n",
		cam_mod->exp_config.exp_time,
		cam_mod->exp_config.gain,
		cam_mod->exp_config.flash_mode);

	/* if the sensor is already streaming, write to shadow registers,
		if the sensor is in SW standby, write to active registers,
		if the sensor is off/registers are not writeable, do nothing */
	if ((cam_mod->state == OV_CAMERA_MODULE_SW_STANDBY) ||
		(cam_mod->state == OV_CAMERA_MODULE_STREAMING)) {
		u32 a_gain = cam_mod->exp_config.gain;
		u32 exp_time = cam_mod->exp_config.exp_time << 4;
		if (cam_mod->state == OV_CAMERA_MODULE_STREAMING)
			ret = ov_camera_module_write_reg(cam_mod,
				OV2680_AEC_GROUP_UPDATE_ADDRESS,
				OV2680_AEC_GROUP_UPDATE_START_DATA);
		ret |= ov_camera_module_write_reg(cam_mod,
			OV2680_AEC_PK_LONG_GAIN_HIGH_REG,
			OV2680_FETCH_MSB_GAIN(a_gain));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV2680_AEC_PK_LONG_GAIN_LOW_REG,
			OV2680_FETCH_LSB_GAIN(a_gain));
		ret = ov_camera_module_write_reg(cam_mod,
			OV2680_AEC_PK_LONG_EXPO_3RD_REG,
			OV2680_FETCH_3RD_BYTE_EXP(exp_time));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV2680_AEC_PK_LONG_EXPO_2ND_REG,
			OV2680_FETCH_2ND_BYTE_EXP(exp_time));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV2680_AEC_PK_LONG_EXPO_1ST_REG,
			OV2680_FETCH_1ST_BYTE_EXP(exp_time));
		if (cam_mod->state == OV_CAMERA_MODULE_STREAMING) {
			ret = ov_camera_module_write_reg(cam_mod,
				OV2680_AEC_GROUP_UPDATE_ADDRESS,
				OV2680_AEC_GROUP_UPDATE_END_DATA);
			ret = ov_camera_module_write_reg(cam_mod,
				OV2680_AEC_GROUP_UPDATE_ADDRESS,
				OV2680_AEC_GROUP_UPDATE_END_LAUNCH);
		}
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int ov2680_g_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

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
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_g_timings(struct ov_camera_module *cam_mod,
	struct ov_camera_module_timings *timings)
{
	int ret = 0;
	u32 reg_val;
	u32 pll_prediv0;
	u32 pll_predivx2;
	u32 pll_multiplier;
	u32 pll_sp_div;
	u32 pll_sys_div;
	u32 win_off;

	if (IS_ERR_OR_NULL(cam_mod->active_config))
		goto err;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_PLL_PREDIV0_REG,
		&reg_val)))
		goto err;

	pll_prediv0 = (reg_val >> 4) & 0x1;

	if (pll_prediv0 == 0)
		pll_prediv0 = 1;
	else
		pll_prediv0 = 2;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_PLL_PREDIV_REG,
		&reg_val)))
		goto err;

	pll_predivx2 = reg_val & 0x7;

	switch (pll_predivx2) {
	case 0:
		pll_predivx2 = 2;
		break;
	case 1:
		pll_predivx2 = 3;
		break;
	case 2:
		pll_predivx2 = 4;
		break;
	case 3:
		pll_predivx2 = 5;
		break;
	case 4:
		pll_predivx2 = 6;
		break;
	case 5:
		pll_predivx2 = 8;
		break;
	case 6:
		pll_predivx2 = 12;
		break;
	case 7:
		pll_predivx2 = 16;
		break;
	}

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_PLL_MUL_HIGH_REG,
		&reg_val)))
		goto err;

	pll_multiplier = (reg_val & 0x1) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_PLL_MUL_LOW_REG,
		&reg_val)))
		goto err;

	pll_multiplier |= (reg_val & 0xFF);

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_PLL_SPDIV_REG,
		&reg_val)))
		goto err;

	pll_sp_div = 1 + (reg_val & 0xF);

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_PLL_DIVSYS_REG,
		&reg_val)))
		goto err;

	pll_sys_div = 1 + (reg_val & 0xF);

	timings->vt_pix_clk_freq_hz =
		((2 * OV2680_EXT_CLK) /
		(pll_prediv0 * pll_predivx2 *
		pll_sp_div * pll_sys_div)) *
		pll_multiplier;
	/*VTS*/
	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_VTS_HIGH_REG,
		&reg_val)))
		goto err;

	timings->frame_length_lines = reg_val <<  8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_VTS_LOW_REG,
		&reg_val)))
		goto err;

	timings->frame_length_lines |= reg_val;

	/*HTS*/
	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_HTS_HIGH_REG,
		&reg_val)))
		goto err;

	timings->line_length_pck = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_HTS_LOW_REG,
		&reg_val)))
		goto err;

	timings->line_length_pck |= reg_val;

	timings->coarse_integration_time_min = 0;
	timings->coarse_integration_time_max_margin =
		OV2680_INTEGRATION_TIME_MARGIN;

	/* OV Sensor do not use fine integration time. */
	timings->fine_integration_time_min = 0;
	timings->fine_integration_time_max_margin = 0;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_X_INC,
		&reg_val)))
		goto err;

	timings->binning_factor_x = ((reg_val >> 4) + 1) / 2;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_Y_INC,
		&reg_val)))
		goto err;

	timings->binning_factor_y = ((reg_val >> 4) + 1) / 2;

	/* Get the cropping and output resolution to ISP for this mode. */
	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_START_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_start = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_START_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_start |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_START_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_start = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_START_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_start |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_END_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_end = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_END_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_end |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_END_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_end = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_END_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_end |= reg_val;

	/* The sensor can do windowing within the cropped array.
	Take this into the cropping size reported. */
	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_H_WIN_OFF_HIGH_REG,
		&reg_val)))
		goto err;

	win_off = (reg_val & 0xf) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_H_WIN_OFF_LOW_REG,
		&reg_val)))
		goto err;

	win_off |= (reg_val & 0xff);

	timings->crop_horizontal_start += win_off;
	timings->crop_horizontal_end -= win_off;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_V_WIN_OFF_HIGH_REG,
		&reg_val)))
		goto err;

	win_off = (reg_val & 0xf) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_V_WIN_OFF_LOW_REG,
		&reg_val)))
		goto err;

	win_off |= (reg_val & 0xff);

	timings->crop_vertical_start += win_off;
	timings->crop_vertical_end -= win_off;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_OUTPUT_SIZE_HIGH_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_width = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_OUTPUT_SIZE_LOW_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_width |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_OUTPUT_SIZE_HIGH_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_height = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_OUTPUT_SIZE_LOW_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_height |= reg_val;

	return ret;
err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_s_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = ov2680_write_aec(cam_mod);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		/* nothing to be done here */
		break;
	case V4L2_CID_FOCUS_ABSOLUTE:
		/* todo*/
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_s_ext_ctrls(struct ov_camera_module *cam_mod,
				 struct ov_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if (ctrls->count == 1)
		ret = ov2680_s_ctrl(cam_mod, ctrls->ctrls[0].id);
	else if (ctrls->count == 2 &&
		((ctrls->ctrls[0].id == V4L2_CID_GAIN &&
		ctrls->ctrls[1].id == V4L2_CID_EXPOSURE) ||
		(ctrls->ctrls[1].id == V4L2_CID_GAIN &&
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE)))
		ret = ov2680_write_aec(cam_mod);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_start_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret = ov2680_write_aec(cam_mod);
	if (IS_ERR_VALUE(ret))
		goto err;
	if (IS_ERR_VALUE(ov_camera_module_write_reg(cam_mod, 0x0100, 1)))
		goto err;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_stop_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret = ov_camera_module_write_reg(cam_mod, 0x0100, 0);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_check_camera_id(struct ov_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret |= ov_camera_module_read_reg(cam_mod, 1, OV2680_PIDH_ADDR, &pidh);
	ret |= ov_camera_module_read_reg(cam_mod, 1, OV2680_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == OV2680_PIDH_MAGIC) && (pidl == OV2680_PIDL_MAGIC))
		ov_camera_module_pr_debug(cam_mod,
			"successfully detected camera ID 0x%02x%02x\n",
			pidh, pidl);
	else {
		ov_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			OV2680_PIDH_MAGIC, OV2680_PIDL_MAGIC, pidh, pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}


/* ======================================================================== */
int ov_camera_2680_module_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{

	return 0;
}

/* ======================================================================== */

int ov_camera_2680_module_s_ext_ctrls(
	struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls)
{

	return 0;
}


long ov_camera_2680_module_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd,
	void *arg)
{
	/*haven't work*/
	return 0;
}


/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops ov2680_camera_module_core_ops = {
	.g_ctrl = ov_camera_module_g_ctrl,
	.s_ctrl = ov_camera_module_s_ctrl,
	.s_ext_ctrls = ov_camera_module_s_ext_ctrls,
	.s_power = ov_camera_module_s_power,
	.ioctl = ov_camera_module_ioctl

};

static struct v4l2_subdev_video_ops ov2680_camera_module_video_ops = {
	.enum_frameintervals = ov_camera_module_enum_frameintervals,
	.s_mbus_fmt = ov_camera_module_s_fmt,
	.g_mbus_fmt = ov_camera_module_g_fmt,
	.try_mbus_fmt = ov_camera_module_try_fmt,
	.s_frame_interval = ov_camera_module_s_frame_interval,
	.s_stream = ov_camera_module_s_stream
};

static struct v4l2_subdev_ops ov2680_camera_module_ops = {
	.core = &ov2680_camera_module_core_ops,
	.video = &ov2680_camera_module_video_ops
};

static struct ov_camera_module ov2680;

static struct ov_camera_module_custom_config ov2680_custom_config = {
	.start_streaming = ov2680_start_streaming,
	.stop_streaming = ov2680_stop_streaming,
	.s_ctrl = ov2680_s_ctrl,
	.g_ctrl = ov2680_g_ctrl,
	.s_ext_ctrls = ov2680_s_ext_ctrls,
	.g_timings = ov2680_g_timings,
	.check_camera_id = ov2680_check_camera_id,
	.configs = ov2680_configs,
	.num_configs = sizeof(ov2680_configs) / sizeof(ov2680_configs[0]),
	.power_up_delays_ms = {5, 30, 0}
};

static int __init ov2680_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	dev_info(&client->dev, "probing...\n");

	v4l2_i2c_subdev_init(&ov2680.sd, client, &ov2680_camera_module_ops);
	ret = ov_camera_module_init(&ov2680,
			&ov2680_custom_config);
	if (IS_ERR_VALUE(ret))
		goto err;

	dev_info(&client->dev, "probing successful\n");
	return 0;
err:
	dev_err(&client->dev, "probing failed with error (%d)\n", ret);
	return ret;
}

/* ======================================================================== */

static int ov2680_remove(
	struct i2c_client *client)
{
	struct ov_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	ov_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id ov2680_id[] = {
	{ OV2680_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id ov2680_of_match[] = {
	{.compatible = "omnivision," OV2680_DRIVER_NAME "-v4l2-i2c-subdev",},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov2680_id);

static struct i2c_driver ov2680_i2c_driver = {
	.driver = {
		.name = OV2680_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ov2680_of_match
	},
	.probe = ov2680_probe,
	.remove = ov2680_remove,
	.id_table = ov2680_id,
};

module_i2c_driver(ov2680_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for ov2680");
MODULE_AUTHOR("Eike Grimpe");
MODULE_LICENSE("GPL");

