/*
 * gc2355 sensor driver
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
 *    16/04/2015: new implementation using v4l2-subdev
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
#define GC2355_DRIVER_NAME				"gc2355"
#define GC2355_AEC_Analog_Gain_REG			0xb6	/* Bit 8 */
#define GC2355_AEC_Global_Gain_REG			0xb0
#define GC2355_AEC_PreGain_HighBits_REG			0xb1
#define GC2355_AEC_PreGain_LowBits_REG			0xb2
#define GC2355_AEC_EXPO_COARSE_2ND_REG			0x03	/* Exposure Bits 8-15 */
#define GC2355_AEC_EXPO_COARSE_1ST_REG			0x04	/* Exposure Bits 0-7 */
#define GC2355_FETCH_2ND_BYTE_EXP(VAL)			((VAL >> 8) & 0x3F)	/* 4 Bits */
#define GC2355_FETCH_1ST_BYTE_EXP(VAL)			(VAL & 0xFF)	/* 8 Bits */
#define GC2355_PID_H_ADDR				0xf0
#define GC2355_PID_L_ADDR				0xf1
#define GC2355_HORIZONTAL_DUMMY_HIGH_REG		0x05
#define GC2355_HORIZONTAL_DUMMY_LOW_REG			0x06
#define GC2355_VERTICAL_DUMMY_HIGH_REG			0x07
#define GC2355_VERTICAL_DUMMY_LOW_REG			0x08
#define GC2355_SH_DELAY_REG				0x11
#define GC2355_TIMING_FRAME_LENGTH_LINES_HIGH_REG	0x95/*0x0d*/
#define GC2355_TIMING_FRAME_LENGTH_LINES_LOW_REG	0x96/*0x0e*/     /*window height*/
#define GC2355_TIMING_LINE_LENGTH_PCKL_HIGH_REG		0x97/*0x0f*/
#define GC2355_TIMING_LINE_LENGTH_PCKL_LOW_REG		0x98/*0x10*/	/*window width*/
#define GC2355_COARSE_INTEGRATION_TIME_MIN		1
#define GC2355_COARSE_INTEGRATION_TIME_MAX_MARGIN	0
#define GC2355_FINE_INTEGRATION_TIME_MIN		0
#define GC2355_FINE_INTEGRATION_TIME_MAX_MARGIN		0
#define GC2355_Output_height				1200
#define GC2355_Output_width				1600
#define GC2355_HBlank					0x125
#define GC2355_VBlank					0x70
#define GC2355_HORIZONTAL_START_REG			0x0c/*colstart*/
#define GC2355_VERTICAL_START_REG			0x0a/*rowstart*/
#define GC2355_EXT_CLK					26000000/*MCLK=26MHz*/
#define GC2355_Pixel_Clock				91000000/*Pixel Clk=91MHz*/
/*flip*/
#define GC2355_IMG_ORIENTATION				0x17
#define GC2355_VFLIP_BIT				2
#define GC2355_HFLIP_BIT				1
#define GC2355MIPI_2Lane /* TODO: Check DTS tree (csi-lanes) instead of hard-coding!*/
/* product ID */
#define GC2355_PID_MAGIC	0x2355
#define ANALOG_GAIN_1 64  /* 1.00x*/
#define ANALOG_GAIN_2 88  /* 1.375x*/
#define ANALOG_GAIN_3 122  /* 1.90x*/
#define ANALOG_GAIN_4 168  /* 2.625x*/
#define ANALOG_GAIN_5 239  /* 3.738x*/
#define ANALOG_GAIN_6 330  /* 5.163x*/
#define ANALOG_GAIN_7 470  /* 7.350x*/
/*static bool is_probed;*/
/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */
static const struct gc_camera_module_reg gc2355_init_tab_2MP_30fps[] = {
	/*SYS*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x80},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x80},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x80},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xf2, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xf6, 0x00},
#if defined(GC2355MIPI_2Lane)
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xf7, 0x31},
#else
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xf7, 0x19},
#endif
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xf8, 0x06}, /*Mclk:26Mhz@30fps*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xf9, 0x0e},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfa, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfc, 0x06},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x00},
	/*Analog & CISCTL reg*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x03, 0x07},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x04, 0x80},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x05, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x06, 0x25}, /*Mclk:26Mhz@30fps; 403 */
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x07, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x08, 0x70}, /*Mclk:26Mhz@30fps;*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0a, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0c, 0x04},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0d, 0x04},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0e, 0xc0},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x0f, 0x06},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x10, 0x50},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x17, 0x14},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x19, 0x0b},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x1b, 0x48},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x1c, 0x12},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x1d, 0x10},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x1e, 0xbc},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x1f, 0xc9},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x20, 0x71},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x21, 0x20},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x22, 0xa0},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x23, 0x51},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x24, 0x19},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x27, 0x20},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x28, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2b, 0x80},/* 0x81 20140926*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2c, 0x38},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2e, 0x16},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2f, 0x14},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x30, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x31, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x32, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x33, 0x03},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x34, 0x07},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x35, 0x0b},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x36, 0x0f},
	/*MIPI*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x03},
#if defined(GC2355MIPI_2Lane)
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x10, 0x81},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x01, 0x87},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x22, 0x03},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x23, 0x20},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x25, 0x10},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x29, 0x02},
#else
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x10, 0x80},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x01, 0x83},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x22, 0x05},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x23, 0x30},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x25, 0x15},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x29, 0x06},
#endif
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x02, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x03, 0x90},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x04, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x05, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x06, 0xa2},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x11, 0x2b},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x12, 0xd0},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x13, 0x07},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x15, 0x60},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x21, 0x10},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x24, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x26, 0x08},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x27, 0x06},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2a, 0x0a},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x2b, 0x08},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x40, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x41, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x42, 0x40},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x43, 0x06},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xfe, 0x00},
	/*GAIN*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xb0, 0x50},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xb1, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xb2, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xb3, 0x40},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xb4, 0x40},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xb5, 0x40},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xb6, 0x00},
	/*Crop*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x92, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x94, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x95, 0x04},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x96, 0xb0},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x97, 0x06},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x98, 0x40},
	/*BLK*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x18, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x1a, 0x01},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x40, 0x42},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x41, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x44, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x45, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x46, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x47, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x48, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x49, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x4a, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x4b, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x4e, 0x3c},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x4f, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x5e, 0x00},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x66, 0x20},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x6a, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x6b, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x6c, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x6d, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x6e, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x6f, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x70, 0x02},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x71, 0x02},
	/*dark sun*/
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0x87, 0x03},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xe0, 0xe7},
	{GC_CAMERA_MODULE_REG_TYPE_DATA, 0xe3, 0xc0},
};
/* ======================================================================== */
static struct gc_camera_module_config gc2355_configs[] = {
	{
		.name = "2MP_20fps",
		.frm_fmt = {
			.width = 1600,
			.height = 1200,
			.code =  V4L2_MBUS_FMT_SBGGR10_1X10/*RAW10*/
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
		.reg_table = (void *)gc2355_init_tab_2MP_30fps,
		.reg_table_num_entries =
			sizeof(gc2355_init_tab_2MP_30fps)
			/
			sizeof(gc2355_init_tab_2MP_30fps[0]),
		.v_blanking_time_us = 2000 /*empirically measured time*/
	},
};
/*--------------------------------------------------------------------------*/
#if 0
static int gc2355_g_VTS(struct gc_camera_module *cam_mod, u32 *vts)
{
	u32 msb, lsb;
	int ret;
	ret = gc_camera_module_read_reg_table(cam_mod, GC2355_TIMING_FRAME_LENGTH_LINES_HIGH_REG, &msb);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = gc_camera_module_read_reg_table(cam_mod, GC2355_TIMING_FRAME_LENGTH_LINES_LOW_REG, &lsb);
	if (IS_ERR_VALUE(ret))
		goto err;
	*vts = ((msb&0x01) << 8) | lsb;
	return 0;
err:
	gc_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}
/*--------------------------------------------------------------------------*/
static int gc2355_auto_adjust_fps(struct gc_camera_module *cam_mod, u32 exp_time)
{
	int ret;
	u32 vts;
	if ((cam_mod->exp_config.exp_time + GC2355_COARSE_INTEGRATION_TIME_MAX_MARGIN) > cam_mod->vts_min)
		vts = cam_mod->exp_config.exp_time + GC2355_COARSE_INTEGRATION_TIME_MAX_MARGIN;
	else
		vts = cam_mod->vts_min;
	gc_camera_module_pr_debug(cam_mod, "vts (%d)\n", vts);
	ret = gc_camera_module_write_reg(cam_mod, GC2355_VERTICAL_DUMMY_LOW_REG, vts & 0xFF);
	ret |= gc_camera_module_write_reg(cam_mod, GC2355_VERTICAL_DUMMY_HIGH_REG, (vts >> 8) & 0x0f);
	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	else
		gc_camera_module_pr_debug(cam_mod, "vts = %d\n", vts);
	return ret;
}
#endif
/*--------------------------------------------------------------------------*/
static int gc2355_write_aec(struct gc_camera_module *cam_mod)
{
	int ret = 0;
	int temp = 0;
    /* if the sensor is already streaming, write to shadow registers,
	 *  if the sensor is in SW standby, write to active registers,
	 *  if the sensor is off/registers are not writeable, do nothing */
	if ((cam_mod->state == GC_CAMERA_MODULE_SW_STANDBY) ||
		(cam_mod->state == GC_CAMERA_MODULE_STREAMING))	{
		u32 a_gain = cam_mod->exp_config.gain;
		u32 exp_time = cam_mod->exp_config.exp_time;
		if (a_gain < 0x40)
			a_gain = 0x40;
		if (a_gain > 512)
			a_gain = 512;
		if ((ANALOG_GAIN_1 <= a_gain) && (a_gain < ANALOG_GAIN_2)) {
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_Analog_Gain_REG, 0x00);
			temp = a_gain;
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_PreGain_HighBits_REG, temp>>6);
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_PreGain_LowBits_REG, (temp<<2)&0xfc);
		} else if ((ANALOG_GAIN_2 <= a_gain) && (a_gain < ANALOG_GAIN_3)) {
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_Analog_Gain_REG, 0x01);
			temp = 64*a_gain/ANALOG_GAIN_2;
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_PreGain_HighBits_REG, temp>>6);
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_PreGain_LowBits_REG, (temp<<2)&0xfc);
		} else if ((ANALOG_GAIN_3 <= a_gain) && (a_gain < ANALOG_GAIN_4)) {
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_Analog_Gain_REG, 0x02);
			temp = 64*a_gain/ANALOG_GAIN_3;
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_PreGain_HighBits_REG, temp>>6);
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_PreGain_LowBits_REG, (temp<<2)&0xfc);
		} else if (ANALOG_GAIN_4 <= a_gain) {
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_Analog_Gain_REG, 0x03);
			temp = 64*a_gain/ANALOG_GAIN_4;
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_PreGain_HighBits_REG, temp>>6);
			ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_PreGain_LowBits_REG, (temp<<2)&0xfc);
		}
		if (exp_time > 16383)
			exp_time = 16383;
		else if (exp_time < 8)
			exp_time = 8;
		ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_EXPO_COARSE_2ND_REG, GC2355_FETCH_2ND_BYTE_EXP(exp_time));
		ret |= gc_camera_module_write_reg(cam_mod, GC2355_AEC_EXPO_COARSE_1ST_REG, GC2355_FETCH_1ST_BYTE_EXP(exp_time));
		/*if (!IS_ERR_VALUE(ret) && cam_mod->auto_adjust_fps)*/
		/*	ret = gc2355_auto_adjust_fps(cam_mod, cam_mod->exp_config.exp_time);*/
/*
		mdelay(70);
		u32 exp_value_h = 0;
		u32 exp_value_l = 0;
		u32 exp_value = 0;
		ret = gc_camera_module_read_reg(cam_mod, 1,
			GC2355_AEC_EXPO_COARSE_2ND_REG, &exp_value_h);
		mdelay(1);
		ret = gc_camera_module_read_reg(cam_mod, 1,
			GC2355_AEC_EXPO_COARSE_1ST_REG, &exp_value_l);
		exp_value = ((exp_value_h & 0x003f) << 8) | (exp_value_l & 0x00ff);
*/
	}
	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}
/*------------------------------caotong_flip--------------------------------------*/
static int gc2355_flip(struct gc_camera_module *cam_mod)
{
	int ret = -EAGAIN;
	if (cam_mod->state == GC_CAMERA_MODULE_SW_STANDBY) {
		u32 reg_val;
		ret = gc_camera_module_read_reg(cam_mod, 1,
			GC2355_IMG_ORIENTATION, &reg_val);
		if (!IS_ERR_VALUE(ret)) {
			if (cam_mod->hflip)
				reg_val |= GC2355_HFLIP_BIT;
			else
				reg_val &= ~GC2355_HFLIP_BIT;
			if (cam_mod->vflip)
				reg_val |= GC2355_VFLIP_BIT;
			else
				reg_val &= ~GC2355_VFLIP_BIT;
			ret = gc_camera_module_write_reg(cam_mod,
				GC2355_IMG_ORIENTATION,
				reg_val);
		}
/*		mdelay(1);*/
/*
		ret = gc_camera_module_read_reg(cam_mod, 1,
			GC2355_IMG_ORIENTATION, &reg_val);
*/
		reg_val &= (GC2355_VFLIP_BIT|GC2355_HFLIP_BIT);
		switch (reg_val) {
		case 0:
			cam_mod->custom.configs[0].frm_fmt.code = V4L2_MBUS_FMT_SBGGR10_1X10;
			break;
		case 1:
			cam_mod->custom.configs[0].frm_fmt.code = V4L2_MBUS_FMT_SGBRG10_1X10;
			break;
		case 2:
			cam_mod->custom.configs[0].frm_fmt.code = V4L2_MBUS_FMT_SGRBG10_1X10;
			break;
		case 3:
			cam_mod->custom.configs[0].frm_fmt.code = V4L2_MBUS_FMT_SRGGB10_1X10;
			break;
		}
		/*cam_mod->frm_fmt.code = V4L2_MBUS_FMT_SBGGR10_1X10;*/
	}
	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}
/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
static int gc2355_g_ctrl(struct gc_camera_module *cam_mod, u32 ctrl_id)
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
		gc_camera_module_pr_debug(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}
/*--------------------------------------------------------------------------*/
static int gc2355_g_timings(struct gc_camera_module *cam_mod, struct gc_camera_module_timings *timings)
{
	int ret = 0;
	int reg_val;
	/*int temp = 0;*/
	if (IS_ERR_OR_NULL(cam_mod->active_config))
		goto err;
	/*---define Sensor Pixel Clock: GC2355 Pixel Clock= InMclk * (0xf8 + 1) /2---*/
	timings->vt_pix_clk_freq_hz = GC2355_EXT_CLK * (6 + 1) / 2;/*Pixel Clock*/
#if 0
	ret = gc2355_g_VTS(cam_mod, &timings->frame_length_lines);
	if (IS_ERR_VALUE(ret))
		goto err;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_TIMING_LINE_LENGTH_PCKL_HIGH_REG, &reg_val)))
		goto err;
	timings->line_length_pck = reg_val << 8;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_TIMING_LINE_LENGTH_PCKL_LOW_REG, &reg_val)))
		goto err;
	timings->line_length_pck = ((GC2355_Output_width + 16) / 2 + 14 + 4) * 2;
#endif
	timings->coarse_integration_time_min = GC2355_COARSE_INTEGRATION_TIME_MIN;
	timings->coarse_integration_time_max_margin = GC2355_COARSE_INTEGRATION_TIME_MAX_MARGIN;
	timings->fine_integration_time_min = GC2355_FINE_INTEGRATION_TIME_MIN;
	timings->fine_integration_time_max_margin = GC2355_FINE_INTEGRATION_TIME_MAX_MARGIN;
	timings->binning_factor_x = 1;
	timings->binning_factor_y = 1;
	/* Get the cropping and output resolution to ISP for this mode. */
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_HORIZONTAL_START_REG, &reg_val)))
		goto err;
	timings->crop_horizontal_start = reg_val;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_VERTICAL_START_REG, &reg_val)))
		goto err;
	timings->crop_vertical_start = reg_val;
#if 0
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_TIMING_LINE_LENGTH_PCKL_HIGH_REG, &reg_val)))
		goto err;
	temp = (reg_val << 8) & 0xff00;
	/*high bits*/
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_TIMING_LINE_LENGTH_PCKL_LOW_REG, &reg_val)))
		goto err;
	/*low bits*/
	temp |= reg_val;
#endif
	timings->sensor_output_width = GC2355_Output_width;
	timings->crop_horizontal_end = timings->crop_horizontal_start + GC2355_Output_width;
#if 0
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_TIMING_FRAME_LENGTH_LINES_HIGH_REG, &reg_val)))
		goto err;
	temp = (reg_val << 8) & 0xff00;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_TIMING_FRAME_LENGTH_LINES_LOW_REG, &reg_val)))
		goto err;
	temp |= reg_val;
#endif
	timings->sensor_output_height = GC2355_Output_height;
	timings->crop_vertical_end = timings->crop_vertical_start + GC2355_Output_height;
#if 0
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_HORIZONTAL_DUMMY_HIGH_REG, &reg_val)))
		goto err;
	temp = (reg_val & 0x0f) << 8;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_HORIZONTAL_DUMMY_LOW_REG, &reg_val)))
		goto err;
	temp += reg_val;
	temp = GC2355_HBlank;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_SH_DELAY_REG, &reg_val)))
		goto err;
	temp += reg_val;
	/*timings->line_length_pck = (timings->sensor_output_width + 16)/2 + 14 + temp + 4;*/
	/*timings->line_length_pck = 3540;*/
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_VERTICAL_DUMMY_HIGH_REG, &reg_val)))
		goto err;
	temp = (reg_val & 0xf0) << 4;
	if (IS_ERR_VALUE(gc_camera_module_read_reg_table(cam_mod, GC2355_VERTICAL_DUMMY_LOW_REG, &reg_val)))
		goto err;
	temp += reg_val;
#endif
	/*define full Line length*/
	timings->line_length_pck = ((timings->sensor_output_width + 16)/2 +
					GC2355_HBlank + 24 + 4) * 2;
	/*define full frame length*/
	timings->frame_length_lines = timings->sensor_output_height +
					GC2355_VBlank + 16 + 16;
	return ret;
err:
	gc_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}
/*--------------------------------------------------------------------------*/
static int gc2355_s_ctrl(struct gc_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;
	gc_camera_module_pr_debug(cam_mod, "\n");
	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = gc2355_write_aec(cam_mod);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = gc2355_flip(cam_mod);    /* caotong_flip*/
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_debug(cam_mod, "failed with error (%d) 0x%x\n", ret, ctrl_id);
	return ret;
}
/*--------------------------------------------------------------------------*/
static int gc2355_s_ext_ctrls(struct gc_camera_module *cam_mod, struct gc_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;
	/* Handles only exposure and gain together special case. */
	if (ctrls->count == 1)
		ret = gc2355_s_ctrl(cam_mod, ctrls->ctrls[0].id);
	else if (ctrls->count == 2 &&
		((ctrls->ctrls[0].id == V4L2_CID_GAIN && ctrls->ctrls[1].id == V4L2_CID_EXPOSURE) ||
		(ctrls->ctrls[1].id == V4L2_CID_GAIN && ctrls->ctrls[0].id == V4L2_CID_EXPOSURE)))
		ret = gc2355_write_aec(cam_mod);
	else
		ret = -EINVAL;
	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_debug(cam_mod, "failed with error (%d)\n", ret);
/*
	int i;
	if (ctrls->count == 0){
		return -1;
		}
	for (i = 0; i < ctrls->count; i++)
	{
//		struct v4l2_ext_control *ctrl;
//		ctrl = &ctrls->controls[i];
		switch (ctrls->ctrls[i].id)
		{
			case V4L2_CID_GAIN:
			case V4L2_CID_EXPOSURE:
				ret = gc2355_write_aec(cam_mod);
				break;
			case V4L2_CID_HFLIP:
			case V4L2_CID_VFLIP:
				ret = gc2355_flip(cam_mod);
				break;
			default:
				break;
		}
	}
		if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_debug(cam_mod, "failed with error (%d)\n", ret);
*/
	return ret;
}
/*--------------------------------------------------------------------------*/
static int gc2355_start_streaming(struct gc_camera_module *cam_mod)
{
	int ret = 0;
	/*u32 reg_val = 0;*/
	gc_camera_module_pr_debug(cam_mod, "\n");
	if (IS_ERR_VALUE(gc_camera_module_write_reg(cam_mod, 0xfe, 0x03)))
		goto err;
#if	defined(GC2355MIPI_2Lane)
	if (IS_ERR_VALUE(gc_camera_module_write_reg(cam_mod, 0x10, 0x91)))
		goto err;
#else
	if (IS_ERR_VALUE(gc_camera_module_write_reg(cam_mod, 0x10, 0x90)))
		goto err;
#endif
	if (IS_ERR_VALUE(gc_camera_module_write_reg(cam_mod, 0xfe, 0x00)))
		goto err;
/*
	ret = gc_camera_module_write_reg(cam_mod,GC2355_IMG_ORIENTATION,
				reg_val);
	reg_val &= (GC2355_VFLIP_BIT|GC2355_HFLIP_BIT);
	switch(reg_val)
	{
		case 0:
			cam_mod->custom.configs[0].frm_fmt.code = V4L2_MBUS_FMT_SBGGR10_1X10;
			break;
		case 1:
			cam_mod->custom.configs[0].frm_fmt.code = V4L2_MBUS_FMT_SGBRG10_1X10;
			break;
		case 2:
			cam_mod->custom.configs[0].frm_fmt.code = V4L2_MBUS_FMT_SGRBG10_1X10;
			break;
		case 3:
			cam_mod->custom.configs[0].frm_fmt.code = V4L2_MBUS_FMT_SRGGB10_1X10;
			break;
	}
*/
	ret = gc2355_write_aec(cam_mod);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = gc2355_flip(cam_mod);
	if (IS_ERR_VALUE(ret))
		goto err;
	return 0;
err:
	gc_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}
/*--------------------------------------------------------------------------*/
static int gc2355_stop_streaming(struct gc_camera_module *cam_mod)
{
	int ret = 0;
	gc_camera_module_pr_debug(cam_mod, "\n");
	ret = gc_camera_module_write_reg(cam_mod, 0xfe, 0x03);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = gc_camera_module_write_reg(cam_mod, 0x10, 0x81);
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
static int gc2355_check_camera_id(struct gc_camera_module *cam_mod)
{
	u32 val, pid = 0;
	int ret = 0;
	gc_camera_module_pr_debug(cam_mod, "\n");
	ret = gc_camera_module_read_reg(cam_mod, 1, GC2355_PID_H_ADDR, &val);
	gc_camera_module_pr_debug(cam_mod, "gc2355 id high = 0x%02x\n", val);
	pid = (val << 8) & 0xff00;
	ret = gc_camera_module_read_reg(cam_mod, 1, GC2355_PID_L_ADDR, &val);
	gc_camera_module_pr_debug(cam_mod, "gc2355 id low = 0x%02x\n", val);
	pid |= val;
	if (IS_ERR_VALUE(ret)) {
		gc_camera_module_pr_err(cam_mod, "register read failed, camera module powered off?\n");
		goto err;
	}
	if (pid == GC2355_PID_MAGIC) {
		/*gc_camera_module_pr_debug(cam_mod, "successfully detected camera ID 0x%02x\n", pid);*/
	} else {
		gc_camera_module_pr_err(cam_mod, "wrong camera ID, expected 0x%02x, detected 0x%02x\n",
					GC2355_PID_MAGIC, pid);
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
static struct v4l2_subdev_core_ops gc2355_camera_module_core_ops = {
	.g_ctrl = gc_camera_module_g_ctrl,
	.s_ctrl = gc_camera_module_s_ctrl,
	.s_ext_ctrls = gc_camera_module_s_ext_ctrls,
	.s_power = gc_camera_module_s_power,
	.ioctl = gc_camera_module_ioctl
};
static struct v4l2_subdev_video_ops gc2355_camera_module_video_ops = {
	.enum_frameintervals = gc_camera_module_enum_frameintervals,
	.s_mbus_fmt = gc_camera_module_s_fmt,
	.g_mbus_fmt = gc_camera_module_g_fmt,
	.try_mbus_fmt = gc_camera_module_try_fmt,
	.s_frame_interval = gc_camera_module_s_frame_interval,
	.s_stream = gc_camera_module_s_stream
};
static struct v4l2_subdev_ops gc2355_camera_module_ops = {
	.core = &gc2355_camera_module_core_ops,
	.video = &gc2355_camera_module_video_ops
};
static struct gc_camera_module gc2355;
static struct gc_camera_module_custom_config gc2355_custom_config = {
	.start_streaming = gc2355_start_streaming,
	.stop_streaming = gc2355_stop_streaming,
	.s_ctrl = gc2355_s_ctrl,
	.s_ext_ctrls = gc2355_s_ext_ctrls,
	.g_ctrl = gc2355_g_ctrl,
	.g_timings = gc2355_g_timings,
	.check_camera_id = gc2355_check_camera_id,
	.configs = gc2355_configs,
	.num_configs = sizeof(gc2355_configs) / sizeof(gc2355_configs[0]),
	.power_up_delays_ms = {20, 20, 20}
};
static int __init gc2355_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int ret = 0;
	dev_info(&client->dev, "probing...\n");
	v4l2_i2c_subdev_init(&gc2355.sd, client, &gc2355_camera_module_ops);
	ret = gc_camera_module_init(&gc2355, &gc2355_custom_config);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = gc_camera_module_s_power(&gc2355.sd, 1);
	/*GC2355_check_camera_id(&GC2355);*/
	if (IS_ERR_VALUE(ret))
		goto err;
	gc_camera_module_s_power(&gc2355.sd, 0);
	dev_info(&client->dev, "probing successful\n");
	return 0;
err:
	dev_err(&client->dev, "probing failed with error (%d)\n", ret);
	gc_camera_module_release(&gc2355);
	return ret;
}
/* ======================================================================== */
static int __exit gc2355_remove(struct i2c_client *client)
{
	struct gc_camera_module *cam_mod = i2c_get_clientdata(client);
	dev_info(&client->dev, "removing device...\n");
	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */
	gc_camera_module_release(cam_mod);
	dev_info(&client->dev, "removed\n");
	return 0;
}
static const struct i2c_device_id gc2355_id[] = {
	{ GC2355_DRIVER_NAME, 0 },
	{ }
};
static struct of_device_id gc2355_of_match[] = {
	{.compatible = "galaxycore," GC2355_DRIVER_NAME "-v4l2-i2c-subdev",},
	{},
};
MODULE_DEVICE_TABLE(i2c, gc2355_id);
static struct i2c_driver gc2355_i2c_driver = {
	.driver = {
		.name = GC2355_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gc2355_of_match
	},
	.probe = gc2355_probe,
	.remove = __exit_p(gc2355_remove),
	.id_table = gc2355_id,
};
module_i2c_driver(gc2355_i2c_driver);
MODULE_DESCRIPTION("RAW Camera Driver for gc2355");
MODULE_AUTHOR("Thomas");
MODULE_LICENSE("GPL");
