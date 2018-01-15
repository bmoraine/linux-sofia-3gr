/*
 * drivers/media/i2c/soc_camera/xgold/ov9740.c
 *
 * ov9740 sensor driver
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

#define OV9740_DRIVER_NAME "ov9740"
#define XVCLK	26000000 /*26MHz external clock input*/
#define COARSE_INTEGRATION_TIME_MARGIN	4

/* Register addresses */
#define OV9740_EXPL_ADDR 0x0203
#define OV9740_EXPH_ADDR 0x0202
#define OV9740_GAIN_ADDR 0x0205
#define OV9740_AGC_AEC_ENABLE_ADDR 0x3503
#define OV9740_AWB_ENABLE_ADDR 0x5001
#define OV9740_PIDH_ADDR 0x300A
#define OV9740_PIDL_ADDR 0x300B


/* Integration time registers. */
#define INTEGRATION_TIME_SUB_LINE_HIGH	0x0200 /*'Fine' integration time in unit
						of pixel period, high byte*/
#define INTEGRATION_TIME_SUB_LINE_LOW	0x0201 /*'Fine' integration time in unit
						of pixel period, low byte*/
#define INTEGRATION_TIME_LINE_HIGH	0x0202 /*'Corse' integration time in
						unit of line period, high byte*/
#define INTEGRATION_TIME_LINE_LOW	0x0203 /*'Corse' integration time in
						unit of line period, low byte*/

/* Clock configuration register */
#define PIXEL_CLK_DIVIDER_PLL1_HIGH	0x0300 /*PLL1 video timing pixel clock
						divider, high byte*/
#define PIXEL_CLK_DIVIDER_PLL1_LOW	0x0301 /*PLL1 video timing pixel clock
						divider, low byte*/
#define SYSTEM_CLK_DIVIDER_PLL1_HIGH	0x0302 /*PLL1 video timing system clock
						divider, high byte*/
#define SYSTEM_CLK_DIVIDER_PLL1_LOW	0x0303 /*PLL1 video timing system clock
						divider, low byte*/
#define PRE_PLL_CLK_DIVIDER_PLL1_HIGH	0x0304 /*PLL1 pre PLL clock divider
						value, high byte*/
#define PRE_PLL_CLK_DIVIDER_PLL1_LOW	0x0305 /*PLL1 pre PLL clock divider
						value, low byte*/
#define PLL_MULTIPLIER_PLL1_HIGH	0x0306 /*PLL1 multiplier value,
						high byte*/
#define PLL_MULTIPLIER_PLL1_LOW		0x0307 /*PLL1 multiplier value,
						low byte*/
#define SCALE_DIVIDER_PLL1		0x3010 /*PLL1 scale divider*/
#define PRE_PLL_CLK_DIVIDER_PLL2	0x300C /*PLL2 pre PLL clock divider
						value*/
#define PLL_MULTIPLIER_PLL2		0x300D /*PLL2 muliplier value*/
#define SYSTEM_CLK_DIVIDER_PLL2		0x300E /*PLL2 video timing system clock
						divider*/
#define PLL_RST_SW			0x3104 /*PLL switch / RESET*/

/* Frame timing registers */
#define VERTICAL_TOTAL_LENGTH_HIGH	0x0340 /*Frame length in unit of line
						period, high byte*/
#define VERTICAL_TOTAL_LENGTH_LOW	0x0341 /*Frame length in unit of line
						period, low byte*/
#define HORIZONTAL_TOTAL_LENGTH_HIGH	0x0342 /*Line length in unit of line
						period, high byte*/
#define HORIZONTAL_TOTAL_LENGTH_LOW	0x0343 /*Line length in unit of line
						period, low byte*/

/* Image size registers */
#define HORIZONTAL_START_HIGH	0x0344 /*X-address of the top left corner of the
					visible pixel data, high byte*/
#define HORIZONTAL_START_LOW	0x0345 /*X-address of the top left corner of the
					visible pixel data, low byte*/
#define VERTICAL_START_HIGH	0x0346 /*Y-address of the top left corner of the
					visible pixel data, high byte*/
#define VERTICAL_START_LOW	0x0347 /*Y-address of the top left corner of the
					visible pixel data, low byte*/
#define HORIZONTAL_END_HIGH	0x0348 /*X-address of the bottom right corner
					of the visible pixel data, high byte*/
#define HORIZONTAL_END_LOW	0x0349 /*X-address of the bottom right corner
					of the visible pixel data, low byte*/
#define VERTICAL_END_HIGH	0x034A /*Y-address of the bottom right corner
					of the visible pixel data, high byte*/
#define VERTICAL_END_LOW	0x034B /*Y-address of the bottom right corner
					of the visible pixel data, low byte*/
#define IMAGE_WIDTH_HIGH	0x034C /*Width of the image data output from
					sensor, high byte*/
#define IMAGE_WIDTH_LOW		0x034D /*Width of the image data output from
					sensor, low byte*/
#define IMAGE_HEIGHT_HIGH	0x034E /*Height of the image data output from
					sensor, high byte*/
#define IMAGE_HEIGHT_LOW	0x034F /*Height of the image data output from
					sensor, low byte*/

/* Bit masks */
#define OV9740_AWB_ENABLE_BIT_MSK ((u8)0x01)
#define OV9740_AEC_DISABLE_BIT_MSK ((u8)0x01)
#define OV9740_AGC_DISABLE_BIT_MSK ((u8)0x02)

#define OV9740_FETCH_3RD_BYTE_EXP(VAL) ((VAL >> 16) & 0xF)	/* 4 Bits */
#define OV9740_FETCH_2ND_BYTE_EXP(VAL) ((VAL >> 8) & 0xFF)	/* 8 Bits */
#define OV9740_FETCH_1ST_BYTE_EXP(VAL) (VAL & 0x000000FF)	/* 8 Bits */

/* High byte of product ID */
#define OV9740_PIDH_MAGIC 0x97
/* Low byte of product ID  */
#define OV9740_PIDL_MAGIC 0x40

/* =============================================== */
/* Base sensor configs */
/* =============================================== */

/* register initialization tables for ov8825 */
static const struct ov_camera_module_reg ov9740_manual_wb_incandescent[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3400, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3401, 0x1c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3402, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3403, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3404, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3405, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3406, 0x01},
};

static const struct ov_camera_module_reg ov9740_manual_wb_fluerescent[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3400, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3401, 0xa6},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3402, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3403, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3404, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3405, 0x4c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3406, 0x01},
};

static const struct ov_camera_module_reg ov9740_manual_wb_cloudy_daylight[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3400, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3401, 0xb5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3402, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3403, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3404, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3405, 0xe2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3406, 0x01},
};

static const struct ov_camera_module_reg ov9740_manual_wb_daylight[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3400, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3401, 0x87},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3402, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3403, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3404, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3405, 0xdc},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3406, 0x01},
};

static const struct ov_camera_module_reg ov9740_auto_wb[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3406, 0x00},
};

static const struct ov_camera_module_reg ov9740_init_tab_720p_24fps[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x01},
	/* orientation */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0101, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3104, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0305, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0307, 0x39},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0303, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0301, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3010, 0x01},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0340, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0341, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0342, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0343, 0x62},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0344, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0345, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0346, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0347, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0348, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0349, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034a, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034b, 0xd8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034c, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034e, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x034f, 0xd0},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3004, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3005, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3012, 0x70},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3013, 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3014, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301f, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3026, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3027, 0x00},
	/* analog*/
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3601, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3602, 0x16},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3603, 0xaa},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3604, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3610, 0xa1},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3612, 0x24},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3620, 0x66},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3622, 0x9f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3630, 0xd2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3631, 0x5e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3632, 0x27},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3633, 0x50},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3703, 0x42},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3704, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3705, 0x45},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3707, 0x11},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3817, 0x94},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3819, 0x6e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3831, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3833, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3835, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3837, 0x01},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a18, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a19, 0xB5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a1a, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a11, 0x90},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a1b, 0x4a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0f, 0x48},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a10, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a1e, 0x42},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a1f, 0x22},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a08, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a09, 0xe8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0e, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a14, 0x15},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a15, 0xc6},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0b, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0d, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a02, 0x18},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a03, 0x20},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c0a, 0x9c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c0b, 0x3f},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4002, 0x45},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4005, 0x18},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4601, 0x16},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x460e, 0x82},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4702, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4704, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4706, 0x08},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4800, 0x64},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4801, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4802, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4803, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4805, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4818, 0x0F},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4819, 0xFF},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4837, 0x20},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5000, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5001, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5003, 0xff},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5180, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5181, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5182, 0x41},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5183, 0x42},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5184, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5185, 0x68},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5186, 0x93},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5187, 0xa8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5188, 0x17},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5189, 0x45},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518a, 0x27},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518b, 0x41},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518c, 0x2d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518d, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518e, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518f, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5190, 0x0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5191, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5192, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5193, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5194, 0x00},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x529a, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x529b, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x529c, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x529d, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x529e, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x529f, 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52a0, 0x32},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52a2, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52a3, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52a4, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52a5, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52a6, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52a7, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52a8, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52a9, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52aa, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52ab, 0x38},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52ac, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52ad, 0x3c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52ae, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x52af, 0x4c},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x530d, 0x06},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5380, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5381, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5382, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5383, 0x0d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5384, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5385, 0x2f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5386, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5387, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5388, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5389, 0xd3},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x538a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x538b, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x538c, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x538d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x538e, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x538f, 0x32},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5390, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5391, 0x94},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5392, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5393, 0xa4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5394, 0x18},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5401, 0x2c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5403, 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5404, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5405, 0xe0},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5480, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5481, 0x12},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5482, 0x27},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5483, 0x49},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5484, 0x57},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5485, 0x66},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5486, 0x75},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5487, 0x81},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5488, 0x8c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5489, 0x95},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x548a, 0xa5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x548b, 0xb2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x548c, 0xc8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x548d, 0xd9},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x548e, 0xec},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5490, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5491, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5492, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5493, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5494, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5495, 0xe0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5496, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5497, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5498, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5499, 0xac},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x549a, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x549b, 0x75},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x549c, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x549d, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x549e, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x549f, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54a0, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54a1, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54a2, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54a3, 0xec},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54a4, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54a5, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54a6, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54a7, 0x9b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54a8, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54a9, 0x63},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54aa, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54ab, 0x2b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54ac, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x54ad, 0x22},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5501, 0x1c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5502, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5503, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5504, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5505, 0x80},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5800, 0x1c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5801, 0x16},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5802, 0x15},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5803, 0x16},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5804, 0x18},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5805, 0x1a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5806, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5807, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5808, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5809, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580a, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580b, 0x0b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580c, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580d, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580e, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580f, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5810, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5811, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5812, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5813, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5814, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5815, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5816, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5817, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5818, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5819, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x581a, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x581b, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x581c, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x581d, 0x0b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x581e, 0x15},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x581f, 0x14},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5820, 0x14},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5821, 0x13},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5822, 0x17},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5823, 0x16},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5824, 0x46},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5825, 0x4c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5826, 0x6c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5827, 0x4c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5828, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5829, 0x2e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x582a, 0x48},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x582b, 0x46},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x582c, 0x2a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x582d, 0x68},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x582e, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x582f, 0x26},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5830, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5831, 0x46},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5832, 0x62},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5833, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5834, 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5835, 0x46},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5836, 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5837, 0x88},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5838, 0x0e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5839, 0x0e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x583a, 0x2c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x583b, 0x2e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x583c, 0x46},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x583d, 0xca},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x583e, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5842, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5843, 0x5e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5844, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5845, 0x32},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5846, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5847, 0x29},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5848, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5849, 0xcc},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0105, 0x01}
};

/* =============================================== */

static struct ov_camera_module_config ov9740_configs[] = {
	{
		.name = "720p_24fps",
		.frm_fmt = {
			.width = 1280,
			.height = 720,
			.code = V4L2_MBUS_FMT_YUYV8_2X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 24
			}
		},
		.auto_exp_enabled = true,
		.auto_gain_enabled = true,
		.auto_wb_enabled = true,
		.reg_table = (void *)ov9740_init_tab_720p_24fps,
		.reg_table_num_entries =
			sizeof(ov9740_init_tab_720p_24fps)
			/
			sizeof(ov9740_init_tab_720p_24fps[0])
	},
};

static int ov9740_g_VTS(struct ov_camera_module *cam_mod, u32 *vts)
{
	u32 msb, lsb;
	int ret;

	ret = ov_camera_module_read_reg_table(
		cam_mod,
		VERTICAL_TOTAL_LENGTH_HIGH,
		&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov_camera_module_read_reg_table(
		cam_mod,
		VERTICAL_TOTAL_LENGTH_LOW,
		&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	*vts = (msb << 8) | lsb;

	ov_camera_module_pr_debug(cam_mod, "\n"
				" VERTICAL_TOTAL_LENGTH: 0x%04X\n", *vts);
	return 0;

err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int ov9740_write_wb_temperature(struct ov_camera_module *cam_mod)
{
	int err = 0;

	if ((cam_mod->state == OV_CAMERA_MODULE_SW_STANDBY) ||
		(cam_mod->state == OV_CAMERA_MODULE_STREAMING)) {
		if (cam_mod->wb_config.temperature >= 6500)
			err =
				ov_camera_module_write_reglist(cam_mod,
					ov9740_manual_wb_daylight,
					sizeof(ov9740_manual_wb_daylight)
					/
					sizeof(ov9740_manual_wb_daylight
					[0]));
		else if (cam_mod->wb_config.temperature < 6500 &&
			cam_mod->wb_config.temperature >= 5000)
			err =
				ov_camera_module_write_reglist(cam_mod,
					ov9740_manual_wb_cloudy_daylight,
					sizeof
					(ov9740_manual_wb_cloudy_daylight)
					/
					sizeof
					(ov9740_manual_wb_cloudy_daylight
					[0]));
		else if (cam_mod->wb_config.temperature < 5000 &&
			cam_mod->wb_config.temperature >= 4000)
			err =
				ov_camera_module_write_reglist(cam_mod,
					ov9740_manual_wb_fluerescent,
					sizeof
					(ov9740_manual_wb_fluerescent) /
					sizeof
					(ov9740_manual_wb_fluerescent
					[0]));
		else if (cam_mod->wb_config.temperature < 4000 &&
			cam_mod->wb_config.temperature >= 2500)
			err =
				ov_camera_module_write_reglist(cam_mod,
					ov9740_manual_wb_incandescent,
					sizeof
					(ov9740_manual_wb_incandescent) /
					sizeof
					(ov9740_manual_wb_incandescent
					[0]));
		else
			err = ov_camera_module_write_reglist(cam_mod,
				ov9740_auto_wb,
				sizeof(ov9740_auto_wb) /
				sizeof(ov9740_auto_wb[0]));
	} else {
		ov_camera_module_pr_warn(cam_mod,
			"camera module clock off, cannot write white balance config\n");
	}

	if (IS_ERR_VALUE(err))
		ov_camera_module_pr_err(cam_mod,
			"failed with error %d\n", err);
	return err;
}

/*--------------------------------------------------------------------------*/

static int ov9740_g_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;
	u32 val;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
		ret = ov_camera_module_read_reg(cam_mod, 1,
			OV9740_GAIN_ADDR, &val);
		if (!IS_ERR_VALUE(ret))
			cam_mod->exp_config.gain = val;
		break;
	case V4L2_CID_EXPOSURE:
		ret = ov_camera_module_read_reg(cam_mod, 1,
			OV9740_EXPL_ADDR, &val);
		if (!IS_ERR_VALUE(ret))
			cam_mod->exp_config.exp_time = (u32)val << 8;
		ret = ov_camera_module_read_reg(cam_mod, 1,
			OV9740_EXPH_ADDR, &val);
		if (!IS_ERR_VALUE(ret))
			cam_mod->exp_config.exp_time |= val;
		break;
	case V4L2_CID_AUTOGAIN:
	case V4L2_CID_EXPOSURE_AUTO:
	case V4L2_CID_AUTO_WHITE_BALANCE:
		/* nothing to be done here */
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

static int ov9740_g_timings(struct ov_camera_module *cam_mod,
		struct ov_camera_module_timings *timings)
{
	u32 msb = 0;
	u32 lsb = 0;
	u32 integration_time_line = 0;
	u32 integration_time_sub_line = 0;
	u32 pixel_clk_divider_pll1 = 0;
	u32 system_clk_divider_pll1 = 0;
	u32 pre_pll_clk_divider_pll1 = 0;
	u32 pll_multiplier_pll1 = 0;
	u32 scale_divider_pll1 = 0;
	u32 pre_pll_clk_divider_pll2 = 0;
	u32 pll_multiplier_pll2 = 0;
	u32 system_clk_divider_pll2 = 0;
	u32 pll_rst_sw = 0;
	u32 horizontal_total_length = 0;
	u32 horizontal_start = 0;
	u32 vertical_start = 0;
	u32 horizontal_end = 0;
	u32 vertical_end = 0;
	u32 image_width = 0;
	u32 image_height = 0;

	/*Read integration time registers*/
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			INTEGRATION_TIME_LINE_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			INTEGRATION_TIME_LINE_LOW, &lsb)))
		goto read_reg;
	integration_time_line = (msb << 8) | lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			INTEGRATION_TIME_SUB_LINE_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			INTEGRATION_TIME_SUB_LINE_LOW, &lsb)))
		goto read_reg;
	integration_time_sub_line = (msb << 8) | lsb;

	/*Read clock configuration registers*/
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			PIXEL_CLK_DIVIDER_PLL1_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			PIXEL_CLK_DIVIDER_PLL1_LOW, &lsb)))
		goto read_reg;
	pixel_clk_divider_pll1 = (msb << 8) | lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			SYSTEM_CLK_DIVIDER_PLL1_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			SYSTEM_CLK_DIVIDER_PLL1_LOW, &lsb)))
		goto read_reg;
	system_clk_divider_pll1 = (msb << 8) | lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			PRE_PLL_CLK_DIVIDER_PLL1_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			PRE_PLL_CLK_DIVIDER_PLL1_LOW, &lsb)))
		goto read_reg;
	pre_pll_clk_divider_pll1 = (msb << 8) | lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			PLL_MULTIPLIER_PLL1_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			PLL_MULTIPLIER_PLL1_LOW, &lsb)))
		goto read_reg;
	pll_multiplier_pll1 = (msb << 8) | lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			SCALE_DIVIDER_PLL1, &lsb)))
		goto read_reg;
	scale_divider_pll1 = lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			PRE_PLL_CLK_DIVIDER_PLL2, &lsb)))
		goto read_reg;
	pre_pll_clk_divider_pll2 = lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			PLL_MULTIPLIER_PLL2, &lsb)))
		goto read_reg;
	pll_multiplier_pll2 = lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			SYSTEM_CLK_DIVIDER_PLL2, &lsb)))
		goto read_reg;
	system_clk_divider_pll2 = lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			PLL_RST_SW, &lsb)))
		goto read_reg;
	pll_rst_sw = lsb;

	/* Read Frame timing registers. */
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			HORIZONTAL_TOTAL_LENGTH_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			HORIZONTAL_TOTAL_LENGTH_LOW, &lsb)))
		goto read_reg;
	horizontal_total_length = (msb << 8) | lsb;

	/* Read image size register. */
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			HORIZONTAL_START_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			HORIZONTAL_START_LOW, &lsb)))
		goto read_reg;
	horizontal_start = (msb << 8) | lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			VERTICAL_START_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			VERTICAL_START_LOW, &lsb)))
		goto read_reg;
	vertical_start = (msb << 8) | lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			HORIZONTAL_END_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			HORIZONTAL_END_LOW, &lsb)))
		goto read_reg;
	horizontal_end = (msb << 8) | lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			VERTICAL_END_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			VERTICAL_END_LOW, &lsb)))
		goto read_reg;
	vertical_end = (msb << 8) | lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			IMAGE_WIDTH_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			IMAGE_WIDTH_LOW, &lsb)))
		goto read_reg;
	image_width = (msb << 8) | lsb;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			IMAGE_HEIGHT_HIGH, &msb)))
		goto read_reg;
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
			IMAGE_HEIGHT_LOW, &lsb)))
		goto read_reg;
	image_height = (msb << 8) | lsb;

	/* Debug print. */
	ov_camera_module_pr_debug(cam_mod, "\n"
		"Integration time registers:\n"
		" INTEGRATION_TIME_LINE: 0x%04X\n"
		" INTEGRATION_TIME_SUB_LINE: 0x%04X\n"
		"Clock configuration registers:\n"
		" PIXEL_CLK_DIVIDER_PLL1: 0x%04X\n"
		" SYSTEM_CLK_DIVIDER_PLL1: 0x%04X\n"
		" PRE_PLL_CLK_DIVIDER_PLL1: 0x%04X\n"
		" PLL_MULTIPLIER_PLL1: 0x%04X\n"
		" SCALE_DIVIDER_PLL1: 0x%02X\n"
		" PRE_PLL_CLK_DIVIDER_PLL2: 0x%02X\n"
		" PLL_MULTIPLIER_PLL2: 0x%02X\n"
		" SYSTEM_CLK_DIVIDER_PLL2: 0x%02X\n"
		" PLL_RST_SW: 0x%02X\n"
		"Frame timing registers:\n"
		" HORIZONTAL_TOTAL_LENGTH: 0x%04X\n"
		"Image size registers:\n"
		" HORIZONTAL_START: 0x%04X\n"
		" VERTICAL_START: 0x%04X\n"
		" HORIZONTAL_END: 0x%04X\n"
		" VERTICAL_END: 0x%04X\n"
		" IMAGE_WIDTH: 0x%04X\n"
		" IMAGE_HEIGHT: 0x%04X\n"
		, integration_time_line, integration_time_sub_line
		, pixel_clk_divider_pll1, system_clk_divider_pll1
		, pre_pll_clk_divider_pll1, pll_multiplier_pll1
		, scale_divider_pll1, pre_pll_clk_divider_pll2
		, pll_multiplier_pll2, system_clk_divider_pll2, pll_rst_sw
		, horizontal_total_length
		, horizontal_start, vertical_start
		, horizontal_end, vertical_end
		, image_width, image_height);

	/* Populate timings structure. */
	timings->coarse_integration_time_min = integration_time_line;
	timings->coarse_integration_time_max_margin =
			COARSE_INTEGRATION_TIME_MARGIN;
	timings->fine_integration_time_min = integration_time_sub_line;
	timings->fine_integration_time_max_margin = 0;
	if (IS_ERR_VALUE(ov9740_g_VTS(cam_mod, &timings->frame_length_lines)))
		goto read_reg;
	timings->line_length_pck = horizontal_total_length;
	if (pre_pll_clk_divider_pll1 == 0 ||
			system_clk_divider_pll1 == 0 ||
			pixel_clk_divider_pll1 == 0){
		ov_camera_module_pr_err(cam_mod, "Avoid devision by ZERO.\n");
		goto error;
	} else {
		timings->vt_pix_clk_freq_hz = (XVCLK / pre_pll_clk_divider_pll1)
				* pll_multiplier_pll1 / system_clk_divider_pll1
				/ pixel_clk_divider_pll1;
	}
	timings->sensor_output_width = image_width;
	timings->sensor_output_height = image_height;
	timings->crop_horizontal_start = horizontal_start;
	timings->crop_vertical_start = vertical_start;
	timings->crop_horizontal_end = horizontal_end;
	timings->crop_vertical_end = vertical_end;
	timings->binning_factor_x = 1;
	timings->binning_factor_y = 1;

	/* Debug print. */
	ov_camera_module_pr_debug(cam_mod, "\n"
		"Timings:\n"
		" coarse_integration_time_min: %d\n"
		" coarse_integration_time_max_margin: %d\n"
		" fine_integration_time_min: %d\n"
		" fine_integration_time_max_margin: %d\n"
		" frame_length_lines: %d\n"
		" line_length_pck: %d\n"
		" vt_pix_clk_freq_hz: %d\n"
		" sensor_output_width: %d\n"
		" sensor_output_height: %d\n"
		" crop_horizontal_start: %d\n"
		" crop_vertical_start: %d\n"
		" crop_horizontal_end: %d\n"
		" crop_vertical_end: %d\n"
		" binning_factor_x: %d\n"
		" binning_factor_y: %d\n"
		, timings->coarse_integration_time_min
		, timings->coarse_integration_time_max_margin
		, timings->fine_integration_time_min
		, timings->fine_integration_time_max_margin
		, timings->frame_length_lines
		, timings->line_length_pck
		, timings->vt_pix_clk_freq_hz
		, timings->sensor_output_width
		, timings->sensor_output_height
		, timings->crop_horizontal_start
		, timings->crop_vertical_start
		, timings->crop_horizontal_end
		, timings->crop_vertical_end
		, timings->binning_factor_x
		, timings->binning_factor_y);

	return 0;

read_reg:
	ov_camera_module_pr_err(cam_mod, "Register read error.\n");
error:
	return -EINVAL;
}
/*--------------------------------------------------------------------------*/

static int ov9740_s_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;
	u32 val;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		ret = ov9740_write_wb_temperature(cam_mod);
		break;
	case V4L2_CID_GAIN:
		ret = ov_camera_module_write_reg(cam_mod, OV9740_GAIN_ADDR,
			OV9740_FETCH_1ST_BYTE_EXP(cam_mod->exp_config.
				gain));
		break;
	case V4L2_CID_EXPOSURE:
		ret |=
			ov_camera_module_write_reg(cam_mod, OV9740_EXPH_ADDR,
				OV9740_FETCH_2ND_BYTE_EXP(cam_mod->exp_config.
					exp_time));
		ret |=
			ov_camera_module_write_reg(cam_mod, OV9740_EXPL_ADDR,
				OV9740_FETCH_1ST_BYTE_EXP(cam_mod->exp_config.
					exp_time));
		break;
	case V4L2_CID_AUTOGAIN:
		ret |= ov_camera_module_read_reg(cam_mod, 1,
			OV9740_AGC_AEC_ENABLE_ADDR, &val);
		if (cam_mod->exp_config.auto_gain)
			val &= ~OV9740_AGC_DISABLE_BIT_MSK;
		else
			val |= OV9740_AGC_DISABLE_BIT_MSK;
		ret |= ov_camera_module_write_reg(cam_mod,
			OV9740_AGC_AEC_ENABLE_ADDR, (u8)val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret |= ov_camera_module_read_reg(cam_mod, 1,
			OV9740_AGC_AEC_ENABLE_ADDR, &val);
		if (cam_mod->exp_config.auto_exp)
			val &= ~OV9740_AEC_DISABLE_BIT_MSK;
		else
			val |= OV9740_AEC_DISABLE_BIT_MSK;
		ret |= ov_camera_module_write_reg(cam_mod,
			OV9740_AGC_AEC_ENABLE_ADDR, (u8)val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret |= ov_camera_module_read_reg(cam_mod, 1,
			OV9740_AWB_ENABLE_ADDR, &val);
		if (cam_mod->wb_config.auto_wb)
			val |= OV9740_AWB_ENABLE_BIT_MSK;
		else
			val &= ~OV9740_AWB_ENABLE_BIT_MSK;
		ret |= ov_camera_module_write_reg(cam_mod,
			OV9740_AWB_ENABLE_ADDR, (u8)val);
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

static int ov9740_s_ext_ctrls(struct ov_camera_module *cam_mod,
				 struct ov_camera_module_ext_ctrls *ctrls)
{
	int ret;

	if (ctrls->count == 1)
		ret = ov9740_s_ctrl(cam_mod, ctrls->ctrls[0].id);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov9740_start_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	if (cam_mod->ctrl_updt) {
		if (cam_mod->ctrl_updt & OV_CAMERA_MODULE_CTRL_UPDT_GAIN)
			ret |= ov9740_s_ctrl(cam_mod,
				V4L2_CID_GAIN);
		if (cam_mod->ctrl_updt & OV_CAMERA_MODULE_CTRL_UPDT_EXP_TIME)
			ret |= ov9740_s_ctrl(cam_mod,
				V4L2_CID_EXPOSURE);
		if (cam_mod->ctrl_updt &
			OV_CAMERA_MODULE_CTRL_UPDT_WB_TEMPERATURE)
			ret |= ov9740_s_ctrl(cam_mod,
				V4L2_CID_WHITE_BALANCE_TEMPERATURE);
		if (cam_mod->ctrl_updt & OV_CAMERA_MODULE_CTRL_UPDT_AUTO_GAIN)
			ret |= ov9740_s_ctrl(cam_mod,
				V4L2_CID_AUTOGAIN);
		if (cam_mod->ctrl_updt & OV_CAMERA_MODULE_CTRL_UPDT_AUTO_EXP)
			ret |= ov9740_s_ctrl(cam_mod,
				V4L2_CID_EXPOSURE_AUTO);
		if (cam_mod->ctrl_updt & OV_CAMERA_MODULE_CTRL_UPDT_AUTO_WB)
			ret |= ov9740_s_ctrl(cam_mod,
				V4L2_CID_AUTO_WHITE_BALANCE);
		if (IS_ERR_VALUE(ret)) {
			ov_camera_module_pr_err(cam_mod,
				"error while writing some registers\n");
			goto err;
		}
	}
	if (IS_ERR_VALUE(ov_camera_module_write_reg(cam_mod, 0x0100, 1)))
		goto err;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov9740_stop_streaming(struct ov_camera_module *cam_mod)
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

static int ov9740_check_camera_id(struct ov_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret |=
		ov_camera_module_read_reg(cam_mod, 1, OV9740_PIDH_ADDR, &pidh);
	ret |=
		ov_camera_module_read_reg(cam_mod, 1, OV9740_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == OV9740_PIDH_MAGIC) && (pidl == OV9740_PIDL_MAGIC))
		ov_camera_module_pr_debug(cam_mod,
			"successfully detected camera ID 0x%02x%02x\n",
			pidh, pidl);
	else {
		ov_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			OV9740_PIDH_MAGIC, OV9740_PIDL_MAGIC, pidh, pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/* =============================================== */
/* This part is platform dependent */
/* =============================================== */

static struct v4l2_subdev_core_ops ov9740_camera_module_core_ops = {
	.g_ctrl = ov_camera_module_g_ctrl,
	.s_ctrl = ov_camera_module_s_ctrl,
	.s_power = ov_camera_module_s_power
};

static struct v4l2_subdev_video_ops ov9740_camera_module_video_ops = {
	.enum_frameintervals = ov_camera_module_enum_frameintervals,
	.s_mbus_fmt = ov_camera_module_s_fmt,
	.try_mbus_fmt = ov_camera_module_try_fmt,
	.s_frame_interval = ov_camera_module_s_frame_interval,
	.s_stream = ov_camera_module_s_stream
};

static struct v4l2_subdev_ops ov9740_camera_module_ops = {
	.core = &ov9740_camera_module_core_ops,
	.video = &ov9740_camera_module_video_ops
};

static struct ov_camera_module ov9740;

static struct ov_camera_module_custom_config ov9740_custom_config = {
	.start_streaming = ov9740_start_streaming,
	.stop_streaming = ov9740_stop_streaming,
	.s_ctrl = ov9740_s_ctrl,
	.s_ext_ctrls = ov9740_s_ext_ctrls,
	.g_ctrl = ov9740_g_ctrl,
	.g_timings = ov9740_g_timings,
	.check_camera_id = ov9740_check_camera_id,
	.configs = ov9740_configs,
	.num_configs = sizeof(ov9740_configs) / sizeof(ov9740_configs[0]),
	.power_up_delays_ms = {5, 20, 0}
};

static int __init ov9740_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	dev_info(&client->dev, "probing...\n");

	v4l2_i2c_subdev_init(&ov9740.sd, client, &ov9740_camera_module_ops);
	ret = ov_camera_module_init(&ov9740, &ov9740_custom_config);
	if (IS_ERR_VALUE(ret))
		goto err;

	dev_info(&client->dev, "probing successful\n");
	return 0;
err:
	dev_err(&client->dev, "probing failed with error (%d)\n", ret);
	return ret;
}

/* =============================================== */

static int __exit ov9740_remove(
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

static const struct i2c_device_id ov9740_id[] = {
	{ OV9740_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id ov9740_of_match[] = {
	{.compatible = "omnivision," OV9740_DRIVER_NAME "-v4l2-i2c-subdev",},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov9740_id);

static struct i2c_driver ov9740_i2c_driver = {
	.driver = {
		.name = OV9740_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ov9740_of_match
	},
	.probe = ov9740_probe,
	.remove = __exit_p(ov9740_remove),
	.id_table = ov9740_id,
};

module_i2c_driver(ov9740_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for ov9740");
MODULE_AUTHOR("Eike Grimpe");
MODULE_LICENSE("GPL");

