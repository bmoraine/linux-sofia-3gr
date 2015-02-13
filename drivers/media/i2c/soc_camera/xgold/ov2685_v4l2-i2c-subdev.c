/*
 * ov2685 sensor driver
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
#define OV2685_DRIVER_NAME "ov2685"
/*{{{*/
#define OV2685_AEC_PK_LONG_GAIN_HIGH_REG 0x350a	/* bit[2:0] -> gain_pk[10:8] */
#define OV2685_AEC_PK_LONG_GAIN_LOW_REG	 0x350b	/* bit[7:0] -> gain_pk[7:0] */
#define OV2685_FETCH_MSB_GAIN(VAL) ((VAL >> 8) & 0x7)
#define OV2685_FETCH_LSB_GAIN(VAL) (VAL & 0x00FF)
/*}}}*/
/*{{{*/
#define OV2685_AEC_PK_LONG_EXPO_3RD_REG 0x3500	/* bit[3:0] -> expo_pk[15:12] */
#define OV2685_AEC_PK_LONG_EXPO_2ND_REG 0x3501	/* bit[7:0] -> expo_pk[11:4] */
#define OV2685_AEC_PK_LONG_EXPO_1ST_REG 0x3502	/* bit[7:4] -> expo_pk[3:0] */

#define OV2685_AEC_GROUP_UPDATE_ADDRESS 0x3208
#define OV2685_AEC_GROUP_UPDATE_START_DATA 0x00
#define OV2685_AEC_GROUP_UPDATE_END_DATA 0x10
#define OV2685_AEC_GROUP_UPDATE_END_LAUNCH 0xA0 /* group delay launch */

#define OV2685_FETCH_3RD_BYTE_EXP(VAL) ((VAL >> 12) & 0xF)	/* 4 Bits */
#define OV2685_FETCH_2ND_BYTE_EXP(VAL) ((VAL >> 4) & 0xFF)	/* 8 Bits */
#define OV2685_FETCH_1ST_BYTE_EXP(VAL) (VAL & 0x0000000F)	/* 4 Bits */
/*}}}*/
/*{{{*/
#define OV2685_PIDH_ADDR     0x300A
#define OV2685_PIDL_ADDR     0x300B
/*}}}*/

#define OV2685_TIMING_HTS_HIGH_REG 0x380c   /* bit[7:0] -> HTS[15:8] */
#define OV2685_TIMING_HTS_LOW_REG 0x380d    /* bit[7:0] -> HTS[7:0] */
#define OV2685_TIMING_VTS_HIGH_REG 0x380e
#define OV2685_TIMING_VTS_LOW_REG 0x380f

/*{{{*/
#define OV2685_INTEGRATION_TIME_MARGIN 0
#define OV2685_TIMING_X_INC		0x3814
#define OV2685_TIMING_Y_INC		0x3815
/*}}}*/
/*{{{*/
#define OV2685_HORIZONTAL_START_HIGH_REG 0x3800
#define OV2685_HORIZONTAL_START_LOW_REG 0x3801
#define OV2685_VERTICAL_START_HIGH_REG 0x3802
#define OV2685_VERTICAL_START_LOW_REG 0x3803
#define OV2685_HORIZONTAL_END_HIGH_REG 0x3804
#define OV2685_HORIZONTAL_END_LOW_REG 0x3805
#define OV2685_VERTICAL_END_HIGH_REG 0x3806
#define OV2685_VERTICAL_END_LOW_REG 0x3807
#define OV2685_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x3808
#define OV2685_HORIZONTAL_OUTPUT_SIZE_LOW_REG 0x3809
#define OV2685_VERTICAL_OUTPUT_SIZE_HIGH_REG 0x380a
#define OV2685_VERTICAL_OUTPUT_SIZE_LOW_REG 0x380b
/*}}}*/

/* NOTES */
#define OV2685_EXT_CLK 26000000
#define OV2685_PLL2_PREDIV 0x3080  /*bit[2:0]*/
#define OV2685_PLL2_MUL_HIGH 0x3081 /*bit[0]*/
#define OV2685_PLL2_MUL_LOW 0x3082  /*bit[7:0]*/
#define OV2685_PLL2_DIV 0x3092
#define OV2685_PLL_SELD5 0x3093

/*{{{*/
/* High byte of product ID */
#define OV2685_PIDH_MAGIC 0x26
/* Low byte of product ID  */
#define OV2685_PIDL_MAGIC 0x85
/*}}}*/

/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */
static const struct ov_camera_module_reg ov2685_initial_setting[] = {
	/*1lanes, 30fps*/
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0103 , 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_TIMEOUT, 0x0000, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3016 , 0x1c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3018 , 0x44},/*84,modify to 1lane*/
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301d , 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3020 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3082 , 0x29},/*mclk=19.2M; 26M 0x37*/
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3083 , 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3084 , 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3085 , 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3086 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3087 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501 , 0x4e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3502 , 0xe0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503 , 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350b , 0x36},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3600 , 0xb4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3603 , 0x35},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3604 , 0x24},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3605 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3620 , 0x24},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621 , 0x34},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3622 , 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3628 , 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3705 , 0x3c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370a , 0x21},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370c , 0x50},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370d , 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3717 , 0x58},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3718 , 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3720 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3721 , 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3722 , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3723 , 0x59},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3738 , 0x99},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3781 , 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3784 , 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3789 , 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3800 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3802 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804 , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805 , 0x4f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806 , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807 , 0xbf},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808 , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809 , 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b , 0xb0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d , 0xa4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e , 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f , 0x0e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3810 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811 , 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3812 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813 , 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814 , 0x11},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815 , 0x11},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3819 , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a06 , 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a07 , 0x84},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a08 , 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a09 , 0x43},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0a , 0x24},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0b , 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0c , 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0d , 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0e , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0f , 0x8c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a10 , 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a11 , 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4000 , 0x81},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4001 , 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4008 , 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009 , 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4300 , 0x30},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x430e , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4602 , 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4837 , 0x1e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5000 , 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5001 , 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5002 , 0x32},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5003 , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5004 , 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5005 , 0x12},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5180 , 0xf4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5181 , 0x11},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5182 , 0x41},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5183 , 0x42},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5184 , 0x78},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5185 , 0x58},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5186 , 0xb5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5187 , 0xb2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5188 , 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5189 , 0x0e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518a , 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518b , 0x4c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518c , 0x38},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518d , 0xf8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518e , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x518f , 0x7f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5190 , 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5191 , 0x5f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5192 , 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5193 , 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5194 , 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5195 , 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5196 , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5197 , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5198 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5199 , 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x519a , 0xd2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x519b , 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5200 , 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5201 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5202 , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5203 , 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5204 , 0x41},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5205 , 0x16},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5206 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5207 , 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x520b , 0x30},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x520c , 0x75},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x520d , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x520e , 0x30},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x520f , 0x75},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5210 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5280 , 0x14},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5281 , 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5282 , 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5283 , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5284 , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5285 , 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5286 , 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5287 , 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5300 , 0xc5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5301 , 0xa0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5302 , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5303 , 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5304 , 0x30},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5305 , 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5306 , 0x90},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5307 , 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5308 , 0x82},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5309 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x530a , 0x26},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x530b , 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x530c , 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x530d , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x530e , 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x530f , 0x14},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5310 , 0x1a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5311 , 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5312 , 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5313 , 0x4b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5380 , 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5381 , 0x52},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5382 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5383 , 0x4a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5384 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5385 , 0xb6},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5386 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5387 , 0x8d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5388 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5389 , 0x3a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x538a , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x538b , 0xa6},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x538c , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5400 , 0x0d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5401 , 0x18},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5402 , 0x31},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5403 , 0x5a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5404 , 0x65},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5405 , 0x6f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5406 , 0x77},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5407 , 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5408 , 0x87},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5409 , 0x8f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x540a , 0xa2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x540b , 0xb2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x540c , 0xcc},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x540d , 0xe4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x540e , 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x540f , 0xa0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5410 , 0x6e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5411 , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5480 , 0x19},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5481 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5482 , 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5483 , 0x12},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5484 , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5485 , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5486 , 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5487 , 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5488 , 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5489 , 0x18},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5500 , 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5501 , 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5502 , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5503 , 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5504 , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5505 , 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5506 , 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5600 , 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5603 , 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5604 , 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5609 , 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x560a , 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5800 , 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5801 , 0x24},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5802 , 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5803 , 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5804 , 0x34},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5805 , 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5806 , 0x12},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5807 , 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5808 , 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5809 , 0x3c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580a , 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580b , 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580c , 0x26},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580d , 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580e , 0x52},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x580f , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5810 , 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5811 , 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5812 , 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5813 , 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5814 , 0x24},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5815 , 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5816 , 0x42},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5817 , 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5818 , 0x0d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5819 , 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x581a , 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x581b , 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a03 , 0x4c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a04 , 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503 , 0x00},
};

/* @@ YCbCr 2M UXGA Capture 100 99 1600 1200 */
static const struct ov_camera_module_reg ov2685_init_tab_2MP_15fps[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3620, 0x24},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0x34},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3622, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370a, 0x21},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370d, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3718, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3721, 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3722, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3723, 0x59},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3738, 0x99},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0x4f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0xbf},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0xb0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0xa4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0x0e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x11},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815, 0x11},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x382a, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a00, 0x41},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a07, 0xc2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a09, 0xa1},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0a, 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0b, 0xda},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0c, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0d, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0e, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0f, 0x8c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a10, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a11, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4008, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x09},
};

static const struct ov_camera_module_reg ov2685_init_tab_720P_30fps[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3620, 0x26},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0x37},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3622, 0x4 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370a, 0x21},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370d, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3718, 0x88},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3721, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3722, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3723, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3738, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0xa0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0xf2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x5 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0xaf},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x3 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0xcd},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x5 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x2 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0xd0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x5 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0xa6},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x2 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0xf8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x8 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x6 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x11},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815, 0x11},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x382a, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a00, 0x41},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a07, 0xe4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a09, 0xbe},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0a, 0x15},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0b, 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0c, 0x17},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0d, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0e, 0x2 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0f, 0xac},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a10, 0x2 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a11, 0xf8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4008, 0x2 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x9 },
};

static const struct ov_camera_module_reg ov2685_init_tab_VGA_30fps[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3620, 0x26},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0x37},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3622, 0x4 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370a, 0x23},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370d, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3718, 0x88},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3721, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3722, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3723, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3738, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0xa0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0x78},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x5 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0xaf},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x4 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0x47},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x2 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x1 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0xe0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x6 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0xac},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x2 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0x84},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x4 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x4 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x31},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815, 0x31},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0xc2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x1 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x382a, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a00, 0x41},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a07, 0xc1},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a09, 0xa1},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0a, 0x12},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0b, 0x18},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0c, 0x14},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0d, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0e, 0x2 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0f, 0x43},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a10, 0x2 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a11, 0x84},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4008, 0x0 },
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x3 },
};

static const struct ov_camera_module_reg ov2685_init_tab_SVGA_30fps[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3620, 0x24},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0x34},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3622, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370a, 0x23},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3718, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3721, 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3722, 0x0b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3723, 0x48},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3738, 0x99},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0x4f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0xbf},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0x58},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0xac},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0x84},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x31},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815, 0x31},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0xc2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x382a, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a00, 0x43},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a07, 0xc1},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a09, 0xa1},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0a, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0b, 0x8a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0c, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0d, 0x8c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0e, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0f, 0x43},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a10, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a11, 0x84},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a13, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4008, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5003, 0x0c},
};

/* ======================================================================== */

static struct ov_camera_module_config ov2685_configs[] = {/*{{{*/
	{
		.name = "2MP_15fps",
		.frm_fmt = {
			.width = 1600,
			.height = 1200,
			.code = V4L2_MBUS_FMT_YUYV8_2X8,
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 15
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)ov2685_init_tab_2MP_15fps,
		.reg_table_num_entries =
			sizeof(ov2685_init_tab_2MP_15fps)
			/
			sizeof(ov2685_init_tab_2MP_15fps[0])
	},
	{
		.name = "720P_30fps",
		.frm_fmt = {
			.width = 1280,
			.height = 720,
			.code = V4L2_MBUS_FMT_YUYV8_2X8,
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
		.reg_table = (void *)ov2685_init_tab_720P_30fps,
		.reg_table_num_entries =
			sizeof(ov2685_init_tab_720P_30fps)
			/
			sizeof(ov2685_init_tab_720P_30fps[0])
	},
	{
		.name = "SVGA_30fps",
		.frm_fmt = {
			.width = 800,
			.height = 600,
			.code = V4L2_MBUS_FMT_YUYV8_2X8,
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
		.reg_table = (void *)ov2685_init_tab_SVGA_30fps,
		.reg_table_num_entries =
			sizeof(ov2685_init_tab_SVGA_30fps)
			/
			sizeof(ov2685_init_tab_SVGA_30fps[0])
	},
	{
		.name = "VGA_30fps",
		.frm_fmt = {
			.width = 640,
			.height = 480,
			.code = V4L2_MBUS_FMT_YUYV8_2X8,
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
		.reg_table = (void *)ov2685_init_tab_VGA_30fps,
		.reg_table_num_entries =
			sizeof(ov2685_init_tab_VGA_30fps)
			/
			sizeof(ov2685_init_tab_VGA_30fps[0])
	},
};/*}}}*/

/*--------------------------------------------------------------------------*/

static int ov2685_write_aec(struct ov_camera_module *cam_mod)
{/*{{{*/
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod,
			"exp_time = %d, gain = %d, flash_mode = %d\n",
			cam_mod->exp_config.exp_time,
			cam_mod->exp_config.gain,
			cam_mod->exp_config.flash_mode);
	return ret;
} /*}}}*/

/*--------------------------------------------------------------------------*/

static int ov2685_g_timings(struct ov_camera_module *cam_mod,
	struct ov_camera_module_timings *timings)
{
	int ret = 0;
	u32 reg_val;

	if (IS_ERR_OR_NULL(cam_mod->active_config))
		goto err;

	/*VTS*/
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
					OV2685_TIMING_VTS_HIGH_REG, &reg_val)))
		goto err;

	timings->frame_length_lines = reg_val <<  8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
		OV2685_TIMING_VTS_LOW_REG, &reg_val)))
		goto err;

	timings->frame_length_lines |= reg_val;

	/*HTS*/
	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
	OV2685_TIMING_HTS_HIGH_REG, &reg_val)))
		goto err;

	timings->line_length_pck = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg(cam_mod, 1,
	OV2685_TIMING_HTS_LOW_REG, &reg_val)))
		goto err;

	timings->line_length_pck |= reg_val;

	timings->coarse_integration_time_min = 0;
	timings->coarse_integration_time_max_margin =
		OV2685_INTEGRATION_TIME_MARGIN;

	/* OV Sensor do not use fine integration time. */
	timings->fine_integration_time_min = 0;
	timings->fine_integration_time_max_margin = 0;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_TIMING_X_INC,
		&reg_val)))
		goto err;

	timings->binning_factor_x = ((ret >> 4) + 1) / 2;

	if (timings->binning_factor_x == 0)
		timings->binning_factor_x = 1;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_TIMING_Y_INC,
		&reg_val)))
		goto err;

	timings->binning_factor_y = ((ret >> 4) + 1) / 2;

	if (timings->binning_factor_y == 0)
		timings->binning_factor_y = 1;

	/* Get the cropping and output resolution to ISP for this mode. */
	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_HORIZONTAL_START_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_start = (reg_val & 0x0FF) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_HORIZONTAL_START_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_start |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_VERTICAL_START_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_start = (reg_val & 0x0FF) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_VERTICAL_START_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_start |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_HORIZONTAL_END_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_end = (reg_val & 0x0FF) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_HORIZONTAL_END_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_end |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_VERTICAL_END_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_end = (reg_val & 0x0FF) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_VERTICAL_END_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_end |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_HORIZONTAL_OUTPUT_SIZE_HIGH_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_width = (reg_val & 0xFF) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_HORIZONTAL_OUTPUT_SIZE_LOW_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_width |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_VERTICAL_OUTPUT_SIZE_HIGH_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_height = (reg_val & 0xFF) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2685_VERTICAL_OUTPUT_SIZE_LOW_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_height |= reg_val;

	timings->vt_pix_clk_freq_hz =
		cam_mod->frm_intrvl.interval.denominator*
		timings->frame_length_lines * timings->line_length_pck;

	return ret;
err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int ov2685_s_wb(struct ov_camera_module *cam_mod)
{
	u32 value = cam_mod->wb_config.preset_id;
	switch (value) {
	case V4L2_WHITE_BALANCE_AUTO:
		ov_camera_module_write_reg(cam_mod, 0x3208, 0x00);
		ov_camera_module_write_reg(cam_mod, 0x5180, 0xf4);
		ov_camera_module_write_reg(cam_mod, 0x3208, 0x10);
		ov_camera_module_write_reg(cam_mod, 0x3208, 0xa0);
		break;

	case V4L2_WHITE_BALANCE_MANUAL:
	case V4L2_WHITE_BALANCE_INCANDESCENT:
		/* Sunny */
		/* start group 1 */
		ov_camera_module_write_reg(cam_mod, 0x3208, 0x00);
		ov_camera_module_write_reg(cam_mod, 0x5180, 0xf6);
		/* R Gain */
		ov_camera_module_write_reg(cam_mod, 0x5195, 0x07);
		ov_camera_module_write_reg(cam_mod, 0x5196, 0x9c);
		/* G Gain */
		ov_camera_module_write_reg(cam_mod, 0x5197, 0x04);
		ov_camera_module_write_reg(cam_mod, 0x5198, 0x00);
		/* B Gain */
		ov_camera_module_write_reg(cam_mod, 0x5199, 0x05);
		ov_camera_module_write_reg(cam_mod, 0x519a, 0xf3);
		/* end group 1 */
		ov_camera_module_write_reg(cam_mod, 0x3208, 0x10);
		/* launch group 1 */
		ov_camera_module_write_reg(cam_mod, 0x3208, 0xa0);
		break;
	case V4L2_WHITE_BALANCE_FLUORESCENT:
		/* Home */
		ov_camera_module_write_reg(cam_mod, 0x3208, 0x00);
		ov_camera_module_write_reg(cam_mod, 0x5180, 0xf6);
		ov_camera_module_write_reg(cam_mod, 0x5195, 0x04);
		ov_camera_module_write_reg(cam_mod, 0x5196, 0x90);
		ov_camera_module_write_reg(cam_mod, 0x5197, 0x04);
		ov_camera_module_write_reg(cam_mod, 0x5198, 0x00);
		ov_camera_module_write_reg(cam_mod, 0x5199, 0x09);
		ov_camera_module_write_reg(cam_mod, 0x519a, 0x20);
		ov_camera_module_write_reg(cam_mod, 0x3208, 0x10);
		ov_camera_module_write_reg(cam_mod, 0x3208, 0xa0);
		break;
	case V4L2_WHITE_BALANCE_FLUORESCENT_H:
	case V4L2_WHITE_BALANCE_HORIZON:
	case V4L2_WHITE_BALANCE_DAYLIGHT:
		/* Office */
		ov_camera_module_write_reg(cam_mod, 0x3208, 0x00);
		ov_camera_module_write_reg(cam_mod, 0x5180, 0xf6);
		ov_camera_module_write_reg(cam_mod, 0x5195, 0x06);
		ov_camera_module_write_reg(cam_mod, 0x5196, 0xb8);
		ov_camera_module_write_reg(cam_mod, 0x5197, 0x04);
		ov_camera_module_write_reg(cam_mod, 0x5198, 0x00);
		ov_camera_module_write_reg(cam_mod, 0x5199, 0x06);
		ov_camera_module_write_reg(cam_mod, 0x519a, 0x5f);
		ov_camera_module_write_reg(cam_mod, 0x3208, 0x10);
		ov_camera_module_write_reg(cam_mod, 0x3208, 0xa0);
		break;
	case V4L2_WHITE_BALANCE_FLASH:
	case V4L2_WHITE_BALANCE_CLOUDY:
		/* Cloudy */
		ov_camera_module_write_reg(cam_mod, 0x3208, 0x00);
		ov_camera_module_write_reg(cam_mod, 0x5180, 0xf6);
		ov_camera_module_write_reg(cam_mod, 0x5195, 0x07);
		ov_camera_module_write_reg(cam_mod, 0x5196, 0xdc);
		ov_camera_module_write_reg(cam_mod, 0x5197, 0x04);
		ov_camera_module_write_reg(cam_mod, 0x5198, 0x00);
		ov_camera_module_write_reg(cam_mod, 0x5199, 0x05);
		ov_camera_module_write_reg(cam_mod, 0x519a, 0xd3);
		ov_camera_module_write_reg(cam_mod, 0x3208, 0x10);
		ov_camera_module_write_reg(cam_mod, 0x3208, 0xa0);
		break;
	case V4L2_WHITE_BALANCE_SHADE:
	default:
		ov_camera_module_pr_debug(cam_mod,
				"unsupported v4l2 preset wb:%d\n", value);
	break;
	}
	return 0;
}

static int ov2685_g_exposure(struct ov_camera_module *cam_mod, s32 *value)
{
	u16 shutter;
	u32 expo_pk;

	if (IS_ERR_OR_NULL(cam_mod->active_config))
		return -1;

	/* read shutter, in number of line period */
	ov_camera_module_read_reg(cam_mod, 1,
			OV2685_AEC_PK_LONG_EXPO_3RD_REG, &expo_pk);
	shutter = expo_pk & 0xF;

	ov_camera_module_read_reg(cam_mod, 1,
			OV2685_AEC_PK_LONG_EXPO_2ND_REG, &expo_pk);
	shutter = (shutter << 8) + (expo_pk & 0xFF);

	ov_camera_module_read_reg(cam_mod, 1,
			OV2685_AEC_PK_LONG_EXPO_1ST_REG, &expo_pk);
	shutter = (shutter << 4) + ((expo_pk >> 4) & 0xF);

	/* T = shutter * 65us */
	*value = shutter * 65 / 1000;

	ov_camera_module_pr_debug(cam_mod,
			"exposure time: %dms shutter: %dus\n", *value, shutter);

	return 0;
}

static int ov2685_g_iso(struct ov_camera_module *cam_mod, s32 *value)
{
	u32 gain16, reg;

	ov_camera_module_read_reg(cam_mod, 1,
			OV2685_AEC_PK_LONG_GAIN_HIGH_REG, &reg);
	gain16 = reg & 0x07;
	ov_camera_module_read_reg(cam_mod, 1,
			OV2685_AEC_PK_LONG_GAIN_LOW_REG, &reg);
	gain16 = (gain16 << 8) + (reg & 0xFF);

	*value = gain16 / 16 * 100;
	ov_camera_module_pr_debug(cam_mod, "iso %d\n", *value);

	return 0;
}

static int ov2685_s_exposure(struct ov_camera_module *cam_mod)
{
	s32 value = cam_mod->exp_config.exp_time;
	switch (value) {
	case -2:
		ov_camera_module_write_reg(cam_mod, 0x3a03, 0x2a);
		ov_camera_module_write_reg(cam_mod, 0x3a04, 0x20);
		break;
	case -1:
		ov_camera_module_write_reg(cam_mod, 0x3a03, 0x3a);
		ov_camera_module_write_reg(cam_mod, 0x3a04, 0x30);
		break;
	case 0:
		ov_camera_module_write_reg(cam_mod, 0x3a03, 0x4e);
		ov_camera_module_write_reg(cam_mod, 0x3a04, 0x40);
		break;
	case 1:
		ov_camera_module_write_reg(cam_mod, 0x3a03, 0x5e);
		ov_camera_module_write_reg(cam_mod, 0x3a04, 0x50);
		break;

	case 2:
		ov_camera_module_write_reg(cam_mod, 0x3a03, 0x6a);
		ov_camera_module_write_reg(cam_mod, 0x3a04, 0x60);
		break;
	}
	return 0;
}


/*--------------------------------------------------------------------------*/
/*{{{*/
static int ov2685_s_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
		ret = ov2685_write_aec(cam_mod);
		break;
	case V4L2_CID_EXPOSURE:
		ret = ov2685_s_exposure(cam_mod);
		break;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		ret = ov2685_s_wb(cam_mod);
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

int ov2685_g_ctrl(struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int ret = 0;
	struct ov_camera_module *cam_mod =
		container_of(sd, struct ov_camera_module, sd);

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_FLASH_LED_MODE:
		/* nothing to be done here */
		break;
	case V4L2_CID_EXPOSURE:
		ret = ov2685_g_exposure(cam_mod, &ctrl->value);
		break;
	case V4L2_CID_ISO_SENSITIVITY:
		ret = ov2685_g_iso(cam_mod, &ctrl->value);
		break;
	default:
		break;
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int ov2685_get_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	return 0;
}

/*--------------------------------------------------------------------------*/

static int ov2685_s_ext_ctrls(struct ov_camera_module *cam_mod,
				 struct ov_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if (ctrls->count == 1)
		ret = ov2685_s_ctrl(cam_mod, ctrls->ctrls[0].id);
	else if (ctrls->count == 2 &&
		((ctrls->ctrls[0].id == V4L2_CID_GAIN &&
		ctrls->ctrls[1].id == V4L2_CID_EXPOSURE) ||
		(ctrls->ctrls[1].id == V4L2_CID_GAIN &&
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE))) {
		ret = ov2685_write_aec(cam_mod);
		ov2685_s_exposure(cam_mod);
	} else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2685_start_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret = ov_camera_module_write_reg(cam_mod, 0x0100, 1);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2685_stop_streaming(struct ov_camera_module *cam_mod)
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

static int ov2685_check_camera_id(struct ov_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	/* writing initialize registers */
	ret = ov_camera_module_write_reglist(cam_mod, ov2685_initial_setting,
			sizeof(ov2685_initial_setting)
			/ sizeof(ov2685_initial_setting[0]));
	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod,
			"write initial setting error\n");
		goto err;
	}

	ret |= ov_camera_module_read_reg(cam_mod, 1, OV2685_PIDH_ADDR, &pidh);
	ret |= ov_camera_module_read_reg(cam_mod, 1, OV2685_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == OV2685_PIDH_MAGIC) && (pidl == OV2685_PIDL_MAGIC))
		ov_camera_module_pr_info(cam_mod,
			"successfully detected camera ID 0x%02x%02x\n",
			pidh, pidl);
	else {
		ov_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			OV2685_PIDH_MAGIC, OV2685_PIDL_MAGIC, pidh, pidl);
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

static struct v4l2_subdev_core_ops ov2685_camera_module_core_ops = {
	.g_ctrl = ov2685_g_ctrl,
	.s_ctrl = ov_camera_module_s_ctrl,
	.s_ext_ctrls = ov_camera_module_s_ext_ctrls,
	.s_power = ov_camera_module_s_power,
	.ioctl = ov_camera_module_ioctl
};

static struct v4l2_subdev_video_ops ov2685_camera_module_video_ops = {
	.enum_frameintervals = ov_camera_module_enum_frameintervals,
	.s_mbus_fmt = ov_camera_module_s_fmt,
	.g_mbus_fmt = ov_camera_module_g_fmt,
	.try_mbus_fmt = ov_camera_module_try_fmt,
	.s_frame_interval = ov_camera_module_s_frame_interval,
	.s_stream = ov_camera_module_s_stream
};

static struct v4l2_subdev_ops ov2685_camera_module_ops = {
	.core = &ov2685_camera_module_core_ops,
	.video = &ov2685_camera_module_video_ops
};

static struct ov_camera_module ov2685;

static struct ov_camera_module_custom_config ov2685_custom_config = {/*{{{*/
	.start_streaming = ov2685_start_streaming,
	.stop_streaming = ov2685_stop_streaming,
	.s_ctrl = ov2685_s_ctrl,
	.s_ext_ctrls = ov2685_s_ext_ctrls,
	.g_ctrl = ov2685_get_ctrl,
	.g_timings = ov2685_g_timings,
	.check_camera_id = ov2685_check_camera_id,
	.configs = ov2685_configs,
	.num_configs = sizeof(ov2685_configs) / sizeof(ov2685_configs[0]),
	.power_up_delays_ms = {5, 20, 0}
};/*}}}*/

static int __init ov2685_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{/*{{{*/
	int ret = 0;

	dev_info(&client->dev, "probing...\n");

	v4l2_i2c_subdev_init(&ov2685.sd, client, &ov2685_camera_module_ops);
	ret = ov_camera_module_init(&ov2685,
			&ov2685_custom_config);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov_camera_module_s_power(&ov2685.sd, 1);
	if (IS_ERR_VALUE(ret))
		goto err;

	ov_camera_module_s_power(&ov2685.sd, 0);
	dev_info(&client->dev, "probing successful\n");
	return 0;
err:
	dev_err(&client->dev, "probing failed with error (%d)\n", ret);
	ov_camera_module_release(&ov2685);
	return ret;
} /*}}}*/

/* ======================================================================== */

static int __exit ov2685_remove(
	struct i2c_client *client)
{/*{{{*/
	struct ov_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	ov_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");
	return 0;
} /*}}}*/

static const struct i2c_device_id ov2685_id[] = {
	{ OV2685_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id ov2685_of_match[] = {
	{.compatible = "omnivision," OV2685_DRIVER_NAME "-v4l2-i2c-subdev",},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov2685_id);

static struct i2c_driver ov2685_i2c_driver = {
	.driver = {
		.name = OV2685_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ov2685_of_match
	},
	.probe = ov2685_probe,
	.remove = __exit_p(ov2685_remove),
	.id_table = ov2685_id,
};

module_i2c_driver(ov2685_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for ov2685");
MODULE_AUTHOR("Eike Grimpe");
MODULE_LICENSE("GPL");
/*}}}*/
