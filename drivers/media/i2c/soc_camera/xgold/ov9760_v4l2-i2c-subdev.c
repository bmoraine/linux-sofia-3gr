/*
 * ov9760 sensor driver
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
#include "ov_camera_module.h"

#define OV9760_DRIVER_NAME "ov9760"

#define OV9760_FETCH_LSB_GAIN(VAL) (VAL & 0x00FF)
#define OV9760_FETCH_MSB_GAIN(VAL) ((VAL >> 8) & 0x7)
#define OV9760_AEC_PK_LONG_GAIN_HIGH_REG 0x350a	/* Bit 8 */
#define OV9760_AEC_PK_LONG_GAIN_LOW_REG	 0x350b	/* Bits 0 -7 */

#define OV9760_AEC_PK_LONG_EXPO_3RD_REG 0x3500	/* Exposure Bits 16-19 */
#define OV9760_AEC_PK_LONG_EXPO_2ND_REG 0x3501	/* Exposure Bits 8-15 */
#define OV9760_AEC_PK_LONG_EXPO_1ST_REG 0x3502	/* Exposure Bits 0-7 */

#define OV9760_AEC_GROUP_UPDATE_ADDRESS 0x3208
#define OV9760_AEC_GROUP_UPDATE_START_DATA 0x00
#define OV9760_AEC_GROUP_UPDATE_END_DATA 0x10
#define OV9760_AEC_GROUP_UPDATE_END_LAUNCH 0xA0

#define OV9760_FETCH_3RD_BYTE_EXP(VAL) ((VAL >> 16) & 0xF)	/* 4 Bits */
#define OV9760_FETCH_2ND_BYTE_EXP(VAL) ((VAL >> 8) & 0xFF)	/* 8 Bits */
#define OV9760_FETCH_1ST_BYTE_EXP(VAL) (VAL & 0x000000FF)	/* 8 Bits */

#define OV9760_PIDH_ADDR     0x300A
#define OV9760_PIDL_ADDR     0x300B

#define OV9760_TIMING_FRAME_LENGTH_LINES_HIGH_REG 0x340
#define OV9760_TIMING_FRAME_LENGTH_LINES_LOW_REG 0x341
#define OV9760_TIMING_LINE_LENGTH_PCKL_HIGH_REG 0x342
#define OV9760_TIMING_LINE_LENGTH_PCKL_LOW_REG 0x343
#define OV9760_INTEGRATION_TIME_MARGIN 4
#define OV9760_TIMING_X_INC		0x3820
#define OV9760_TIMING_Y_INC		0x3821

#define OV9760_HORIZONTAL_START_HIGH_REG 0x344
#define OV9760_HORIZONTAL_START_LOW_REG 0x345
#define OV9760_VERTICAL_START_HIGH_REG 0x346
#define OV9760_VERTICAL_START_LOW_REG 0x347
#define OV9760_HORIZONTAL_END_HIGH_REG 0x348
#define OV9760_HORIZONTAL_END_LOW_REG 0x349
#define OV9760_VERTICAL_END_HIGH_REG 0x34a
#define OV9760_VERTICAL_END_LOW_REG 0x34b
#define OV9760_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x34c
#define OV9760_HORIZONTAL_OUTPUT_SIZE_LOW_REG 0x34d
#define OV9760_VERTICAL_OUTPUT_SIZE_HIGH_REG 0x34e
#define OV9760_VERTICAL_OUTPUT_SIZE_LOW_REG 0x34f
#define OV9760_EXT_CLK 26000000
#define OV9760_PLL2_PREDIV 0x3090
#define OV9760_PLL2_MUL 0x3091
#define OV9760_PLL2_DIV 0x3092
#define OV9760_PLL_SELD5 0x3093
#define OV9760_IMAGE_ORIENTATION_REG 0x0101

#define OV9760_VFLIP 0x2
#define OV9760_HFLIP 0x1

/* High byte of product ID */
#define OV9760_PIDH_MAGIC 0x97
/* Low byte of product ID  */
#define OV9760_PIDL_MAGIC 0x60


/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */
static const struct ov_camera_module_reg ov9760_init_tab_1_6MP_30fps[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_TIMEOUT, 0x0000, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0340, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0341, 0xb9},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0342, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0343, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0344, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0345, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0346, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0347, 0x50},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0348, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0349, 0xd7},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034a, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034b, 0x17},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034c, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034e, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034f, 0xd0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0383, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0387, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3660, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3680, 0xf4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3010, 0x81},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3012, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3014, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3022, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3023, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3080, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3090, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3091, 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3092, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3093, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3094, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3095, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3096, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3097, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3098, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3099, 0x12},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309a, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309c, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309e, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309f, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30a2, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b0, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b1, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b3, 0x32},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b4, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b5, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3509, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3600, 0x7c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0xb8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3622, 0x23},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3631, 0xe2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3634, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3662, 0x14},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366b, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3682, 0x82},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3705, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3708, 0x64},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x371b, 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3732, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3745, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3746, 0x18},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3780, 0x2a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3781, 0x8c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x378f, 0xf5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3823, 0x37},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x383d, 0x88},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4000, 0x23},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4001, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4002, 0x45},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4004, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4005, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4006, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x404F, 0x8F},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4058, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4101, 0x32},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4102, 0xa4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4520, 0xb0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4580, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4582, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4307, 0x30},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4605, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4608, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4609, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4801, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4819, 0xB6},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4837, 0x1E},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4906, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d00, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d01, 0x4b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d02, 0xfe},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d03, 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d04, 0x1e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d05, 0xb7},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a04, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a05, 0xa4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a06, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a07, 0xf8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x64},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x7c},
	/* Switch_to_1472x1104 */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0344, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0345, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0346, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0347, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0348, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0349, 0xdf},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034a, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034b, 0x63},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034c, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034d, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034e, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034f, 0x50},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0381, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0383, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0385, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0387, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3810, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3812, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x08},
	/* default exposure and gain: */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3500, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3502, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350b, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x320b, 0x00}
};

#if 0
/* we keep this for reference */
static const struct ov_camera_module_reg ov9760_init_tab_720p_30fps[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0340, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0341, 0xB9},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0342, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0343, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0344, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0345, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0346, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0347, 0x50},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0348, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0349, 0xd7},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034a, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034b, 0x17},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034c, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034e, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034f, 0xd0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0383, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0387, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3660, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3680, 0xf4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0101, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3010, 0x81},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3012, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3014, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3022, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3023, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3080, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3090, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3091, 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3092, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3093, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3094, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3095, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3096, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3097, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3098, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3099, 0x12},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309a, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309c, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309e, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309f, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30a2, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b0, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b1, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b3, 0x32},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b4, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b5, 0x00},
/*	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x27},*/
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3600, 0x7c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0xb8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3622, 0x23},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3631, 0xe2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3634, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3662, 0x14},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366b, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3682, 0x82},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3705, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3708, 0x64},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x371b, 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3732, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3745, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3746, 0x18},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3780, 0x2a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3781, 0x8c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x378f, 0xf5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3823, 0x37},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x383d, 0x88},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4000, 0x23},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4001, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4002, 0x45},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4004, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4005, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4006, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x404F, 0x8F},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4058, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4101, 0x32},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4102, 0xa4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4520, 0xb0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4580, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4582, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4307, 0x30},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4605, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4608, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4609, 0x00},
	/* {OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4800, 0x64}, */ /* gated clk */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4801, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4819, 0xB6},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4837, 0x1E},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4906, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d00, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d01, 0x4b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d02, 0xfe},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d03, 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d04, 0x1e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4d05, 0xb7},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a04, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a05, 0xa4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a06, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a07, 0xf8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x64},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x7c},
};
#endif

/* ======================================================================== */

static struct ov_camera_module_config ov9760_configs[] = {
	{
		.name = "1_6MP_30fps",
		.frm_fmt = {
			.width = 1472,
			.height = 1104,
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
		.reg_table = (void *)ov9760_init_tab_1_6MP_30fps,
		.reg_table_num_entries =
			sizeof(ov9760_init_tab_1_6MP_30fps)
			/
			sizeof(ov9760_init_tab_1_6MP_30fps[0]),
		.v_blanking_time_us = 2000 /*empirically measured time*/
	},
};

/*--------------------------------------------------------------------------*/

static int ov9760_g_VTS(struct ov_camera_module *cam_mod, u32 *vts)
{
	u32 msb, lsb;
	int ret;

	ret = ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_TIMING_FRAME_LENGTH_LINES_HIGH_REG,
		&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_TIMING_FRAME_LENGTH_LINES_LOW_REG,
		&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	*vts = (msb << 8) | lsb;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/
static int ov9760_flip(struct ov_camera_module *cam_mod)
{
	int ret = -EAGAIN;

	if (cam_mod->state == OV_CAMERA_MODULE_SW_STANDBY) {
		u32 reg_val = 0;

		if (cam_mod->hflip)
			reg_val |= OV9760_HFLIP;
		if (cam_mod->vflip)
			reg_val |= OV9760_VFLIP;

		ret = ov_camera_module_write_reg(cam_mod,
			OV9760_IMAGE_ORIENTATION_REG,
			reg_val);
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov9760_auto_adjust_fps(struct ov_camera_module *cam_mod,
	u32 exp_time)
{
	int ret;
	u32 vts;

	if ((cam_mod->exp_config.exp_time + OV9760_INTEGRATION_TIME_MARGIN) >
		cam_mod->vts_min)
		vts = cam_mod->exp_config.exp_time +
			OV9760_INTEGRATION_TIME_MARGIN;
	else
		vts = cam_mod->vts_min;
	ret = ov_camera_module_write_reg(cam_mod,
		OV9760_TIMING_FRAME_LENGTH_LINES_LOW_REG,
		vts & 0xFF);
	ret |= ov_camera_module_write_reg(cam_mod,
		OV9760_TIMING_FRAME_LENGTH_LINES_HIGH_REG,
		(vts >> 8) & 0xFF);

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
				"failed with error (%d)\n", ret);
	else
		ov_camera_module_pr_debug(cam_mod,
				"vts = %d\n", vts);

	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov9760_write_aec(struct ov_camera_module *cam_mod)
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
			OV9760_AEC_GROUP_UPDATE_ADDRESS,
			OV9760_AEC_GROUP_UPDATE_START_DATA);
		ret |= ov_camera_module_write_reg(cam_mod,
			OV9760_AEC_PK_LONG_GAIN_HIGH_REG,
			OV9760_FETCH_MSB_GAIN(a_gain));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV9760_AEC_PK_LONG_GAIN_LOW_REG,
			OV9760_FETCH_LSB_GAIN(a_gain));
		ret = ov_camera_module_write_reg(cam_mod,
			OV9760_AEC_PK_LONG_EXPO_3RD_REG,
			OV9760_FETCH_3RD_BYTE_EXP(exp_time));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV9760_AEC_PK_LONG_EXPO_2ND_REG,
			OV9760_FETCH_2ND_BYTE_EXP(exp_time));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV9760_AEC_PK_LONG_EXPO_1ST_REG,
			OV9760_FETCH_1ST_BYTE_EXP(exp_time));
		if (!IS_ERR_VALUE(ret) && cam_mod->auto_adjust_fps)
			ret = ov9760_auto_adjust_fps(cam_mod,
				cam_mod->exp_config.exp_time);
		if (cam_mod->state == OV_CAMERA_MODULE_STREAMING) {
			ret = ov_camera_module_write_reg(cam_mod,
				OV9760_AEC_GROUP_UPDATE_ADDRESS,
				OV9760_AEC_GROUP_UPDATE_END_DATA);
			ret = ov_camera_module_write_reg(cam_mod,
				OV9760_AEC_GROUP_UPDATE_ADDRESS,
				OV9760_AEC_GROUP_UPDATE_END_LAUNCH);
		}
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov9760_g_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
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

static int ov9760_g_timings(struct ov_camera_module *cam_mod,
	struct ov_camera_module_timings *timings)
{
	int ret = 0;
	int reg_val;
	u32 pll2_prediv;
	u32 pll2_multiplier;
	u32 pll2_divs;
	u32 pll_seld5x2;

	if (IS_ERR_OR_NULL(cam_mod->active_config))
		goto err;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_PLL2_PREDIV,
		&reg_val)))
		goto err;

	pll2_prediv = reg_val & 0x07;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_PLL2_MUL,
		&reg_val)))
		goto err;

	pll2_multiplier = reg_val & 0x3F;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_PLL2_DIV,
		&reg_val)))
		goto err;

	pll2_divs = reg_val & 0xF;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_PLL_SELD5,
		&reg_val)))
		goto err;

	pll_seld5x2 = reg_val & 0x03;

	if (pll2_prediv == 0 || pll2_multiplier == 0 ||
	    pll2_divs == 0)
		goto err;

	switch (pll_seld5x2) {
	case 0:
	case 1:
		pll_seld5x2 = 2;
		break;
	case 2:
		pll_seld5x2 = 4;
		break;
	case 3:
		pll_seld5x2 = 5;
		break;
	}

	timings->vt_pix_clk_freq_hz =
		((OV9760_EXT_CLK / pll2_prediv) * pll2_multiplier)
		/ (pll2_divs * pll_seld5x2);

	ret = ov9760_g_VTS(cam_mod, &timings->frame_length_lines);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_TIMING_LINE_LENGTH_PCKL_HIGH_REG,
		&reg_val)))
		goto err;

	timings->line_length_pck = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_TIMING_LINE_LENGTH_PCKL_LOW_REG,
		&reg_val)))
		goto err;

	timings->line_length_pck |= reg_val;

	timings->coarse_integration_time_min = 1;
	timings->coarse_integration_time_max_margin =
		OV9760_INTEGRATION_TIME_MARGIN;

	/* OV Sensor do not use fine integration time. */
	timings->fine_integration_time_min = 0;
	timings->fine_integration_time_max_margin = 0;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_TIMING_X_INC,
		&reg_val)))
		goto err;

	if (reg_val & 0x1)
		timings->binning_factor_x = 2;
	else
		timings->binning_factor_x = 1;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_TIMING_Y_INC,
		&reg_val)))
		goto err;

	if (reg_val & 0x1)
		timings->binning_factor_y = 2;
	else
		timings->binning_factor_y = 1;

	/* Get the cropping and output resolution to ISP for this mode. */
	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_HORIZONTAL_START_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_start = (reg_val & 0x15) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_HORIZONTAL_START_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_start |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_VERTICAL_START_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_start = (reg_val & 0x15) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_VERTICAL_START_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_start |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_HORIZONTAL_END_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_end = (reg_val & 0x15) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_HORIZONTAL_END_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_end |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_VERTICAL_END_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_end = (reg_val & 0x15) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_VERTICAL_END_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_end |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_HORIZONTAL_OUTPUT_SIZE_HIGH_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_width = (reg_val & 0x15) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_HORIZONTAL_OUTPUT_SIZE_LOW_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_width |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_VERTICAL_OUTPUT_SIZE_HIGH_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_height = (reg_val & 0xF) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV9760_VERTICAL_OUTPUT_SIZE_LOW_REG,
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

static int ov9760_s_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = ov9760_write_aec(cam_mod);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = ov9760_flip(cam_mod);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d) 0x%x\n", ret, ctrl_id);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov9760_s_ext_ctrls(struct ov_camera_module *cam_mod,
				 struct ov_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if (ctrls->count == 1)
		ret = ov9760_s_ctrl(cam_mod, ctrls->ctrls[0].id);
	else if (ctrls->count == 2 &&
		((ctrls->ctrls[0].id == V4L2_CID_GAIN &&
		ctrls->ctrls[1].id == V4L2_CID_EXPOSURE) ||
		(ctrls->ctrls[1].id == V4L2_CID_GAIN &&
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE)))
		ret = ov9760_write_aec(cam_mod);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov9760_start_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret = ov9760_g_VTS(cam_mod, &cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = ov9760_write_aec(cam_mod);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = ov9760_flip(cam_mod);
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

static int ov9760_stop_streaming(struct ov_camera_module *cam_mod)
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

static int ov9760_check_camera_id(struct ov_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret |= ov_camera_module_read_reg(cam_mod, 1, OV9760_PIDH_ADDR, &pidh);
	ret |= ov_camera_module_read_reg(cam_mod, 1, OV9760_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == OV9760_PIDH_MAGIC) && (pidl == OV9760_PIDL_MAGIC))
		ov_camera_module_pr_debug(cam_mod,
			"successfully detected camera ID 0x%02x%02x\n",
			pidh, pidl);
	else {
		ov_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			OV9760_PIDH_MAGIC, OV9760_PIDL_MAGIC, pidh, pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops ov9760_camera_module_core_ops = {
	.g_ctrl = ov_camera_module_g_ctrl,
	.s_ctrl = ov_camera_module_s_ctrl,
	.s_ext_ctrls = ov_camera_module_s_ext_ctrls,
	.s_power = ov_camera_module_s_power,
	.ioctl = ov_camera_module_ioctl
};

static struct v4l2_subdev_video_ops ov9760_camera_module_video_ops = {
	.enum_frameintervals = ov_camera_module_enum_frameintervals,
	.s_mbus_fmt = ov_camera_module_s_fmt,
	.g_mbus_fmt = ov_camera_module_g_fmt,
	.try_mbus_fmt = ov_camera_module_try_fmt,
	.s_frame_interval = ov_camera_module_s_frame_interval,
	.s_stream = ov_camera_module_s_stream
};

static struct v4l2_subdev_ops ov9760_camera_module_ops = {
	.core = &ov9760_camera_module_core_ops,
	.video = &ov9760_camera_module_video_ops
};

static struct ov_camera_module ov9760;

static struct ov_camera_module_custom_config ov9760_custom_config = {
	.start_streaming = ov9760_start_streaming,
	.stop_streaming = ov9760_stop_streaming,
	.s_ctrl = ov9760_s_ctrl,
	.s_ext_ctrls = ov9760_s_ext_ctrls,
	.g_ctrl = ov9760_g_ctrl,
	.g_timings = ov9760_g_timings,
	.check_camera_id = ov9760_check_camera_id,
	.configs = ov9760_configs,
	.num_configs = sizeof(ov9760_configs) / sizeof(ov9760_configs[0]),
	.power_up_delays_ms = {5, 20, 0}
};

static int __init ov9760_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	dev_info(&client->dev, "probing...\n");

	v4l2_i2c_subdev_init(&ov9760.sd, client, &ov9760_camera_module_ops);
	ret = ov_camera_module_init(&ov9760,
			&ov9760_custom_config);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov_camera_module_s_power(&ov9760.sd, 1);
	if (IS_ERR_VALUE(ret))
		goto err;

	ov_camera_module_s_power(&ov9760.sd, 0);
	dev_info(&client->dev, "probing successful\n");
	return 0;
err:
	dev_err(&client->dev, "probing failed with error (%d)\n", ret);
	ov_camera_module_release(&ov9760);
	return ret;
}

/* ======================================================================== */

static int __exit ov9760_remove(
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

static const struct i2c_device_id ov9760_id[] = {
	{ OV9760_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id ov9760_of_match[] = {
	{.compatible = "omnivision," OV9760_DRIVER_NAME "-v4l2-i2c-subdev",},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov9760_id);

static struct i2c_driver ov9760_i2c_driver = {
	.driver = {
		.name = OV9760_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ov9760_of_match
	},
	.probe = ov9760_probe,
	.remove = __exit_p(ov9760_remove),
	.id_table = ov9760_id,
};

module_i2c_driver(ov9760_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for ov9760");
MODULE_AUTHOR("Eike Grimpe");
MODULE_LICENSE("GPL");

