/*
 * drivers/media/i2c/soc_camera/xgold/ov5670.c
 *
 * ov5670 sensor driver
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
 *	11/14/2014: new implementation using v4l2-subdev
 *						instead of v4l2-int-device.
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include "ov_camera_module.h"

#define OV5670_DRIVER_NAME					   "ov5670"

/* High byte of product ID */
#define OV5670_PIDH_MAGIC					   0x56
/* Low byte of product ID */
#define OV5670_PIDL_MAGIC					   0x70

#define OV5670_EXT_CLK						   26000000

#define OV5670_FETCH_LSB_GAIN(VAL)			   (VAL & 0x00FF)
#define OV5670_FETCH_MSB_GAIN(VAL)			   ((VAL >> 8) & 0xff)

/* 4 Bits */
#define OV5670_FETCH_3RD_BYTE_EXP(VAL)		   ((VAL >> 16) & 0xF)
/* 8 Bits */
#define OV5670_FETCH_2ND_BYTE_EXP(VAL)		   ((VAL >> 8) & 0xFF)
/* 8 Bits */
#define OV5670_FETCH_1ST_BYTE_EXP(VAL)		   (VAL & 0xFF)

#define OV5670_PIDH_ADDR					   0x300B
#define OV5670_PIDL_ADDR					   0x300C

/* Bit 8 */
#define OV5670_AEC_PK_LONG_GAIN_HIGH_REG	   0x3508
/* Bits 0 -7 */
#define OV5670_AEC_PK_LONG_GAIN_LOW_REG		   0x3509

/* Exposure Bits 16-19 */
#define OV5670_AEC_PK_LONG_EXPO_3RD_REG		   0x3500
/* Exposure Bits 8-15 */
#define OV5670_AEC_PK_LONG_EXPO_2ND_REG		   0x3501
/* Exposure Bits 0-7 */
#define OV5670_AEC_PK_LONG_EXPO_1ST_REG		   0x3502

#define OV5670_AEC_GROUP_UPDATE_ADDRESS		   0x3208
#define OV5670_AEC_GROUP_ID_0				   0x00
#define OV5670_AEC_GROUP_ID_1				   0x01
#define OV5670_AEC_GROUP_UPDATE_START_DATA	   0x00
#define OV5670_AEC_GROUP_UPDATE_END_DATA	   0x10
#define OV5670_AEC_GROUP_UPDATE_END_LAUNCH	   0xA0

#define OV5670_TIMING_HTS_HIGH_REG			   0x380c
#define OV5670_TIMING_HTS_LOW_REG			   0x380d
#define OV5670_TIMING_VTS_HIGH_REG			   0x380e
#define OV5670_TIMING_VTS_LOW_REG			   0x380f

/* Change from 8 to 4, but what's that means? */
#define OV5670_INTEGRATION_TIME_MARGIN		   4

#define OV5670_HORIZONTAL_START_HIGH_REG	   0x3800
#define OV5670_HORIZONTAL_START_LOW_REG		   0x3801
#define OV5670_VERTICAL_START_HIGH_REG		   0x3802
#define OV5670_VERTICAL_START_LOW_REG		   0x3803
#define OV5670_HORIZONTAL_END_HIGH_REG		   0x3804
#define OV5670_HORIZONTAL_END_LOW_REG		   0x3805
#define OV5670_VERTICAL_END_HIGH_REG		   0x3806
#define OV5670_VERTICAL_END_LOW_REG			   0x3807
#define OV5670_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x3808
#define OV5670_HORIZONTAL_OUTPUT_SIZE_LOW_REG  0x3809
#define OV5670_VERTICAL_OUTPUT_SIZE_HIGH_REG   0x380a
#define OV5670_VERTICAL_OUTPUT_SIZE_LOW_REG	   0x380b

#define OV5670_H_WIN_OFF_HIGH_REG			   0x3810
#define OV5670_H_WIN_OFF_LOW_REG			   0x3811
#define OV5670_V_WIN_OFF_HIGH_REG			   0x3812
#define OV5670_V_WIN_OFF_LOW_REG			   0x3813

#define OV5670_TIMING_X_INC					   0x3814
#define OV5670_TIMING_Y_INC					   0x382a

#define OV5670_PLL2_CLKDIV_REG				   0x3106
#define OV5670_PLL2_CTRL_REG				   0x0312
#define OV5670_PLL2_PREDIV_REG				   0x030b
#define OV5670_PLL2_MUL_HIGH_REG			   0x030c
#define OV5670_PLL2_MUL_LOW_REG				   0x030d
#define OV5670_PLL2_SYS_PREDIV_REG			   0X030f
#define OV5670_PLL2_SYSDIV_REG				   0x030e

#define OV5670_BASE_GAIN					   128
#define OV5670_GAIN_MULTI_MAX				   16

#define OV5670_V_FLIP_REG					   0x3820
#define OV5670_H_MIRROR_REG					   0x3821

#define OV5670_FLIP_BIT_MASK				   0x6

#define OV5670_OTP_SUPPORT


static struct ov_camera_module ov5670;
static struct ov_camera_module_custom_config ov5670_custom_config;

static int flip_mode;

/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */

/* For 4:3 preview */
static const struct ov_camera_module_reg ov5670_1280x960_30fps[] = {
/* @@ov5670_1280x960_30fps */
/* ;;Sysclk=104M mipi=840.6667Mbps dac_clk=364Mhz */
/* ;;fps=30fps 1lane 10bit */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501, 0x3d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3623, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366e, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370b, 0x1b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0xc0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0x42},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0x50},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x90},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x47},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x382a, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x400a, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x400b, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4502, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4508, 0x55},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4509, 0x55},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x450a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4600, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4601, 0x81},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4017, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x01},
};

/* For 16:9 video */
static const struct ov_camera_module_reg ov5670_720P_30fps[] = {
/* @@ov5670_720P_30fps */
/* ;;Sysclk=104M mipi=840.6667Mbps dac_clk=364Mhz */
/* ;;fps=30 1lane 10bit */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501, 0x3d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3623, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366e, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370b, 0x1b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0xd0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x0d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0x18},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x90},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x47},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x382a, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x400a, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x400b, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4502, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4508, 0x55},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4509, 0x55},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x450a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4600, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4601, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4017, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x01},
};

/* FULL QSXGA (2592x1944) 15fps 1lane 10Bit*/
static const struct ov_camera_module_reg ov5670_5M_15fps[] = {
/* @@ov5670_2592x1944_15fps */
/* ;;Sysclk=104M mipi=840.6667Mbps dac_clk=364Mhz */
/* ;;fps=15fps 1lane 10bit */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501, 0x7b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3623, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366e, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370b, 0x1b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0x98},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x0d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0x3e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0xfd},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x46},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x382a, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x0d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x400a, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x400b, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4502, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4508, 0xaa},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4509, 0xaa},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x450a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4600, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4601, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4017, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x01},
};


/* ======================================================================== */
/* Init common sensor configs */
/* ======================================================================== */

static const struct ov_camera_module_reg ov5670_init_regs[] = {
	/* @@ov5670_init_regs */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0300, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0301, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0302, 0x61},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0303, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0304, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0305, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0306, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030b, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030c, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030d, 0x1c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030e, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030f, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0312, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3000, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0x21},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3005, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3007, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3015, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3018, 0x12},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301a, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301b, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301c, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301d, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301e, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3030, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3031, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x303c, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x303e, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3040, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3041, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3042, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3106, 0x11},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3500, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501, 0x7b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3502, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3504, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3505, 0x83},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3508, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3509, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350e, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350f, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3510, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3511, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3512, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3601, 0xc8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3610, 0x88},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3612, 0x48},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3614, 0x5b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3615, 0x96},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0xd0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3622, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3623, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3633, 0x13},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3634, 0x13},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3635, 0x13},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3636, 0x13},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3645, 0x13},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3646, 0x82},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3650, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3652, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3655, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3656, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x365a, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x365e, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3668, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366a, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366e, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366f, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3700, 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3701, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3702, 0x3a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3703, 0x19},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3704, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3705, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3706, 0x66},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3707, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3708, 0x34},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3709, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370a, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370b, 0x1b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3714, 0x24},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x371a, 0x3e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3733, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3734, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x373a, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x373b, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x373c, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x373f, 0xa0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3755, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3758, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x375b, 0x0e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3766, 0x5f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3768, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3769, 0x22},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3773, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3774, 0x1f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3776, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a0, 0x88},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a1, 0x5c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a7, 0x88},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a8, 0x70},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37aa, 0x88},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37ab, 0x48},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37b3, 0x66},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37c2, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37c5, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37c8, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3800, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3802, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0x33},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0xa3},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0x98},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0x8c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0xb8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3816, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3817, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3818, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3819, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x46},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3822, 0x48},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3826, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3827, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x382a, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x382b, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3830, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3836, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3837, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3838, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3841, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3846, 0x48},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3861, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3862, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3863, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a11, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a12, 0x78},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b00, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b02, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b03, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b04, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b05, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c00, 0x89},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c01, 0xab},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c02, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c03, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c04, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c05, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c06, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c07, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c0c, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c0d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c0e, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c0f, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c40, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c41, 0xa3},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c43, 0x7d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c45, 0xd7},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c47, 0xfc},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c50, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c52, 0xaa},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c54, 0x71},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c56, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3d85, 0x17},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f03, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f0a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f0b, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4001, 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x0d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4020, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4021, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4022, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4023, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4024, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4025, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4026, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4027, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4028, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4029, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x402a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x402b, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x402c, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x402d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x402e, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x402f, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4040, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4041, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4042, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4043, 0x7A},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4044, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4045, 0x7A},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4046, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4047, 0x7A},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4048, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4049, 0x7A},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4303, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4307, 0x30},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4500, 0x58},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4501, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4502, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4503, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4508, 0xaa},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4509, 0xaa},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x450a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x450b, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4600, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4601, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4700, 0xa4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4800, 0x4c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4816, 0x53},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x481f, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4837, 0x13},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5000, 0x56},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5001, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5002, 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5004, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5006, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5007, 0xe0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5008, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5009, 0xb0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5901, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5a01, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5a03, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5a04, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5a05, 0xe0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5a06, 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5a07, 0xb0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5a08, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5e00, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3734, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b00, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b01, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b02, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b03, 0xdb},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3d8c, 0x71},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3d8d, 0xea},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4017, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3618, 0x2a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5780, 0x3e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5781, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5782, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5783, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5784, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5785, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5786, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5787, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5788, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5789, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578a, 0xfd},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578b, 0xf5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578c, 0xf5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578d, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578e, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578f, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5790, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5791, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5792, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5793, 0x52},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5794, 0xa3},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0xfd},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0x61},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3010, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x300D, 0x00},
};

static struct ov_camera_module_config ov5670_configs[] = {
	/* For normal preview and 480P recording */
	{
		.name = "ov5670_1280x960_30fps",
		.frm_fmt = {
			.width  = 1280,
			.height = 960,
			.code   = V4L2_MBUS_FMT_SBGGR10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator	 = 1,
				.denominator = 30
			}
		},
		.auto_exp_enabled	   = false,
		.auto_gain_enabled	   = false,
		.auto_wb_enabled	   = false,
		.reg_table			   =
			(void *)ov5670_1280x960_30fps,
		.reg_table_num_entries =
			sizeof(ov5670_1280x960_30fps)
			/
			sizeof(ov5670_1280x960_30fps[0]),
		/* v_blanking_time_us =
		 * HTS * (VTS - HEIGHT_OUTPUT_SIZE) / PCLK */
		.v_blanking_time_us	   = 4344
	},
	/* For 720P recording and special preview in capturing 1080P image */
	{
		.name = "ov5670_720P_30fps",
		.frm_fmt = {
			.width  = 1280,
			.height = 720,
			.code   = V4L2_MBUS_FMT_SBGGR10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator	 = 1,
				.denominator = 30
			}
		},
		.auto_exp_enabled	   = false,
		.auto_gain_enabled	   = false,
		.auto_wb_enabled	   = false,
		.reg_table			   = (void *)ov5670_720P_30fps,
		.reg_table_num_entries =
			sizeof(ov5670_720P_30fps)
			/
			sizeof(ov5670_720P_30fps[0]),
		/* v_blanking_time_us =
		 * HTS * (VTS - HEIGHT_OUTPUT_SIZE) / PCLK */
		.v_blanking_time_us	   = 10120
	},
	/* For 4:3 ratio snapshot */
	{
		.name = "ov5670_5M_15fps",
		.frm_fmt = {
			.width  = 2592,
			.height = 1944,
			.code   = V4L2_MBUS_FMT_SBGGR10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator	   = 1,
				.denominator   = 15
			}
		},
		.auto_exp_enabled	   = false,
		.auto_gain_enabled	   = false,
		.auto_wb_enabled	   = false,
		.reg_table			   = (void *)ov5670_5M_15fps,
		.reg_table_num_entries =
			sizeof(ov5670_5M_15fps)
			/
			sizeof(ov5670_5M_15fps[0]),
		/* v_blanking_time_us =
		 * HTS * (VTS - HEIGHT_OUTPUT_SIZE) / PCLK */
		.v_blanking_time_us	   = 3292
	},
};

/*--------------------------------------------------------------------------*/

static int ov5670_g_VTS(struct ov_camera_module *cam_mod, u32 *vts)
{
	int ret = 0;
	u32 msb = 0;
	u32 lsb = 0;

	ret = ov_camera_module_read_reg_table(cam_mod,
			OV5670_TIMING_VTS_HIGH_REG,
			&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov_camera_module_read_reg_table(cam_mod,
			OV5670_TIMING_VTS_LOW_REG,
			&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	*vts = (msb << 8) | lsb;

	return 0;

err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n",
			ret);
	return ret;
}

static int ov5670_g_HTS(struct ov_camera_module *cam_mod, u32 *hts)
{
	int ret = 0;
	u32 msb = 0;
	u32 lsb = 0;

	ret = ov_camera_module_read_reg_table(cam_mod,
			OV5670_TIMING_HTS_HIGH_REG,
			&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov_camera_module_read_reg_table(cam_mod,
			OV5670_TIMING_HTS_LOW_REG,
			&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	*hts = (msb << 8) | lsb;

	return 0;

err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n",
			ret);
	return ret;
}

static int OV5670_get_SCLK(struct ov_camera_module *cam_mod, u32 *sysclk)
{
	int ret				   = 0;
	u32 reg_val			   = 0;
	u32 pll2_prediv0	   = 0;
	u32 pll2_prediv2x	   = 0;
	u32 pll2_mult		   = 0;
	u32 pll2_sys_prediv	   = 0;
	u32 pll2_sys_divider2x = 0;
	u32 sys_prediv		   = 0;
	u32 sclk_pdiv		   = 0;
	u32 VCO				   = 0;

	u32 pll2_prediv2x_map[]		 = {2, 3, 4, 5, 6, 8, 12, 16};
	u32 pll2_sys_divider2x_map[] = {2, 3, 4, 5, 6, 7, 8, 10};
	u32 sys_prediv_map[]		 = {1, 2, 4, 1};

	ret = ov_camera_module_read_reg_table(cam_mod,
			OV5670_PLL2_CTRL_REG,
			&reg_val);
	if (IS_ERR_VALUE(ret))
		return ret;
	reg_val	    &= 0x10;
	pll2_prediv0 = (10 == reg_val) ? 2 : 1;

	ret = ov_camera_module_read_reg_table(cam_mod,
			OV5670_PLL2_PREDIV_REG,
			&reg_val);
	if (IS_ERR_VALUE(ret))
		return ret;
	pll2_prediv2x = pll2_prediv2x_map[reg_val & 0x7];

	ret = ov_camera_module_read_reg_table(cam_mod,
			OV5670_PLL2_MUL_HIGH_REG,
			&reg_val);
	if (IS_ERR_VALUE(ret))
		return ret;
	pll2_mult = ((reg_val & 0x03) << 8);

	ret = ov_camera_module_read_reg_table(cam_mod,
			OV5670_PLL2_MUL_LOW_REG,
			&reg_val);
	if (IS_ERR_VALUE(ret))
		return ret;
	pll2_mult += reg_val;

	if (pll2_prediv0 && pll2_prediv2x && pll2_mult)
		VCO = OV5670_EXT_CLK * 2 / pll2_prediv0 /
			pll2_prediv2x * pll2_mult;
	else {
		ov_camera_module_pr_err(cam_mod,
				"%s(%d): someone of the denominators is zero!\n",
				__func__,
				__LINE__);
		return -EINVAL;
	}

	ret = ov_camera_module_read_reg_table(cam_mod,
			OV5670_PLL2_SYS_PREDIV_REG,
			&reg_val);
	if (IS_ERR_VALUE(ret))
		return ret;
	pll2_sys_prediv = (reg_val & 0xF) + 1;

	ret = ov_camera_module_read_reg_table(cam_mod,
			OV5670_PLL2_SYSDIV_REG,
			&reg_val);
	if (IS_ERR_VALUE(ret))
		return ret;
	pll2_sys_divider2x = pll2_sys_divider2x_map[reg_val & 0x7];

	ret = ov_camera_module_read_reg_table(cam_mod,
			OV5670_PLL2_CLKDIV_REG,
			&reg_val);
	if (IS_ERR_VALUE(ret))
		return ret;
	sys_prediv = sys_prediv_map[(reg_val >> 2) & 0x3];
	reg_val	   = (reg_val >> 4) & 0xF;
	sclk_pdiv  = (reg_val > 0) ? reg_val : 1;

	if (pll2_sys_prediv && pll2_sys_divider2x && sys_prediv && sclk_pdiv)
		/* 104000000 */
		*sysclk = VCO * 2 /
			pll2_sys_prediv /
			pll2_sys_divider2x /
			sys_prediv /
			sclk_pdiv;
	else {
		ov_camera_module_pr_err(cam_mod,
				"%s(%d): someone of the denominators is zero!\n",
				__func__,
				__LINE__);

		return -EINVAL;
	}

	ov_camera_module_pr_debug(cam_mod,
			"%s(%d): sysclk is %u,VCO is %u,pll2_sys_prediv is %u,pll2_sys_divider2x is %u,sys_prediv is %u,sclk_pdiv is %u.\n",
			__func__,
			__LINE__,
			*sysclk,
			VCO,
			pll2_sys_prediv,
			pll2_sys_divider2x,
			sys_prediv,
			sclk_pdiv);

	return ret;
}
/*--------------------------------------------------------------------------*/

static int ov5670_auto_adjust_fps(struct ov_camera_module *cam_mod,
		u32 exp_time)
{
	int ret = 0;
	u32 vts = 0;

	if ((cam_mod->exp_config.exp_time + OV5670_INTEGRATION_TIME_MARGIN) >
			cam_mod->vts_min)
		vts = cam_mod->exp_config.exp_time +
			OV5670_INTEGRATION_TIME_MARGIN;
	else
		vts = cam_mod->vts_min;

	ret = ov_camera_module_write_reg(cam_mod,
			OV5670_TIMING_VTS_LOW_REG,
			vts & 0xFF);

	ret |= ov_camera_module_write_reg(cam_mod,
			OV5670_TIMING_VTS_HIGH_REG,
			(vts >> 8) & 0xFF);

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
				"failed with error (%d)\n",
				ret);
	else
		ov_camera_module_pr_debug(cam_mod,
				"vts = %d\n",
				vts);

	return ret;
}

/*--------------------------------------------------------------------------*/
static int ov5670_s_gain(struct ov_camera_module *cam_mod, int gain)
{
	/* write gain, 128(0x80) = 1x, low 7 bits are fraction bits */
	/* gain[12:0] */
	gain &= 0x1fff;

	ov_camera_module_write_reg(cam_mod,
			OV5670_AEC_PK_LONG_GAIN_LOW_REG,
			gain & 0xff);
	ov_camera_module_write_reg(cam_mod,
			OV5670_AEC_PK_LONG_GAIN_HIGH_REG,
			(gain >> 8) & 0x1f);

	return 0;
}

static int ov5670_s_shutter(struct ov_camera_module *cam_mod, u32 shutter)
{
	/* write shutter, in number of line period */
	shutter &= 0xfffff;

	ov_camera_module_write_reg(cam_mod,
			OV5670_AEC_PK_LONG_EXPO_1ST_REG,
			OV5670_FETCH_1ST_BYTE_EXP(shutter));
	ov_camera_module_write_reg(cam_mod,
			OV5670_AEC_PK_LONG_EXPO_2ND_REG,
			OV5670_FETCH_2ND_BYTE_EXP(shutter));
	ov_camera_module_write_reg(cam_mod,
			OV5670_AEC_PK_LONG_EXPO_3RD_REG,
			OV5670_FETCH_3RD_BYTE_EXP(shutter));

	return 0;
}

static int ov5670_write_aec(struct ov_camera_module *cam_mod)
{
	int ret		   = 0;
	u32 a_gain	   = 0;
	u32 exp_time   = 0;
	u8 change_flag = 0x07;
	u16 multiple   = 1;

	ov_camera_module_pr_debug(cam_mod,
			"exp_time = %5d, gain = %d(0x%.4X), flash_mode = %2d\n",
			cam_mod->exp_config.exp_time,
			cam_mod->exp_config.gain,
			cam_mod->exp_config.gain,
			cam_mod->exp_config.flash_mode);

	/* if the sensor is already streaming, write to shadow registers,
	   if the sensor is in SW standby, write to active registers,
	   if the sensor is off/registers are not writeable, do nothing */
	if ((cam_mod->state == OV_CAMERA_MODULE_SW_STANDBY) ||
			(cam_mod->state == OV_CAMERA_MODULE_STREAMING)) {
		a_gain   = cam_mod->exp_config.gain;
		exp_time = cam_mod->exp_config.exp_time << 4;

		a_gain &= 0x1fff;
		/* Fixme */
		if ((a_gain < OV5670_BASE_GAIN) ||
				(a_gain > OV5670_GAIN_MULTI_MAX *
				 OV5670_BASE_GAIN)) {
			ov_camera_module_pr_err(cam_mod,
					"Error gain setting %d\n",
					a_gain);

			if (a_gain < OV5670_BASE_GAIN)
				a_gain = OV5670_BASE_GAIN;
			else if (a_gain > OV5670_GAIN_MULTI_MAX *
					OV5670_BASE_GAIN)
				a_gain = OV5670_GAIN_MULTI_MAX *
					OV5670_BASE_GAIN;
		}

		multiple = a_gain / OV5670_BASE_GAIN;
		if (multiple < 2)
			change_flag = 0x00;
		else if (multiple < 4)
			change_flag = 0x01;
		else if (multiple < 8)
			change_flag = 0x03;
		else
			change_flag = 0x07;

		/* Increase the VTS to match exposure + 8 */
		ov_camera_module_write_reg(cam_mod,
				0x301d,
				0xf0);
		ov_camera_module_write_reg(cam_mod,
				0x3209,
				0x00);
		ov_camera_module_write_reg(cam_mod,
				0x320a,
				0x01);

		if (cam_mod->state == OV_CAMERA_MODULE_STREAMING) {
			ov_camera_module_write_reg(cam_mod,
					OV5670_AEC_GROUP_UPDATE_ADDRESS,
					OV5670_AEC_GROUP_UPDATE_START_DATA |
					OV5670_AEC_GROUP_ID_0);

			ov_camera_module_write_reg(cam_mod,
				0x366a,
				change_flag);
		}

		/* Set VTS */
		if (cam_mod->auto_adjust_fps) {
			ret = ov5670_auto_adjust_fps(cam_mod,
					cam_mod->exp_config.exp_time);
			if (IS_ERR_VALUE(ret))
				ov_camera_module_pr_err(cam_mod,
						"%s(%d): auto adjust fps failed!\n",
						__func__,
						__LINE__);
		}

		/* Set AG */
		ret = ov5670_s_gain(cam_mod, a_gain);
		if (IS_ERR_VALUE(ret))
			ov_camera_module_pr_err(cam_mod,
					"%s(%d): set gain failed!\n",
					__func__,
					__LINE__);

		/* Set ET */
		ret = ov5670_s_shutter(cam_mod, exp_time);
		if (IS_ERR_VALUE(ret))
			ov_camera_module_pr_err(cam_mod,
					"%s(%d): set exposure time failed!\n",
					__func__,
					__LINE__);

		/* group lanch */
		if (cam_mod->state == OV_CAMERA_MODULE_STREAMING) {
			ov_camera_module_write_reg(cam_mod,
				OV5670_AEC_GROUP_UPDATE_ADDRESS,
				OV5670_AEC_GROUP_UPDATE_END_DATA |
				OV5670_AEC_GROUP_ID_0);

			ov_camera_module_write_reg(cam_mod,
					0x320B,
					0x0);
			ov_camera_module_write_reg(cam_mod,
					OV5670_AEC_GROUP_UPDATE_ADDRESS,
					OV5670_AEC_GROUP_UPDATE_END_LAUNCH |
					OV5670_AEC_GROUP_ID_0);
		}
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
				"failed with error (%d)\n",
				ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5670_g_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
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
				"failed with error (%d)\n",
				ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5670_g_timings(struct ov_camera_module *cam_mod,
		struct ov_camera_module_timings *timings)
{
	int ret	 = 0;
	u32 reg_val = 0;
	u32 win_off = 0;

	ret = OV5670_get_SCLK(cam_mod,
			&timings->vt_pix_clk_freq_hz);
	if (IS_ERR_VALUE(ret))
		goto err;

	/* VTS */
	ret = ov5670_g_VTS(cam_mod,
			&timings->frame_length_lines);
	if (IS_ERR_VALUE(ret))
		goto err;

	/* HTS */
	ret = ov5670_g_HTS(cam_mod,
			&timings->line_length_pck);
	if (IS_ERR_VALUE(ret))
		goto err;

	timings->coarse_integration_time_min		= 1;
	timings->coarse_integration_time_max_margin =
		OV5670_INTEGRATION_TIME_MARGIN;

	/* OV Sensor do not use fine integration time */
	timings->fine_integration_time_min			= 0;
	timings->fine_integration_time_max_margin   = 0;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_TIMING_X_INC,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->binning_factor_x = ((reg_val & 0x0F) + 1) / 2;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_TIMING_Y_INC,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->binning_factor_y = ((reg_val & 0x0F) + 1) / 2;

	/* Get the cropping and output resolution to ISP for this mode. */
	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_HORIZONTAL_START_HIGH_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->crop_horizontal_start = reg_val << 8;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_HORIZONTAL_START_LOW_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->crop_horizontal_start |= reg_val;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_VERTICAL_START_HIGH_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->crop_vertical_start = reg_val << 8;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_VERTICAL_START_LOW_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->crop_vertical_start |= reg_val;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_HORIZONTAL_END_HIGH_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->crop_horizontal_end = reg_val << 8;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_HORIZONTAL_END_LOW_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->crop_horizontal_end |= reg_val;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_VERTICAL_END_HIGH_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->crop_vertical_end = reg_val << 8;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_VERTICAL_END_LOW_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->crop_vertical_end |= reg_val;

	/* The sensor can do windowing within the cropped array.
	   Take this into the cropping size reported. */
	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_H_WIN_OFF_HIGH_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	win_off = (reg_val & 0xf) << 8;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_H_WIN_OFF_LOW_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	win_off |= (reg_val & 0xff);

	timings->crop_horizontal_start += win_off;
	timings->crop_horizontal_end   -= win_off;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_V_WIN_OFF_HIGH_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	win_off = (reg_val & 0xf) << 8;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_V_WIN_OFF_LOW_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	win_off |= (reg_val & 0xff);

	timings->crop_vertical_start += win_off;
	timings->crop_vertical_end   -= win_off;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_HORIZONTAL_OUTPUT_SIZE_HIGH_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->sensor_output_width = reg_val << 8;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_HORIZONTAL_OUTPUT_SIZE_LOW_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->sensor_output_width |= reg_val;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_VERTICAL_OUTPUT_SIZE_HIGH_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->sensor_output_height = reg_val << 8;

	ret = ov_camera_module_read_reg_table(cam_mod,
					OV5670_VERTICAL_OUTPUT_SIZE_LOW_REG,
					&reg_val);
	if (IS_ERR_VALUE(ret))
		goto err;
	timings->sensor_output_height |= reg_val;

	return ret;

err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n",
			ret);
	return ret;
}

static int ov5670_set_flip_reg(struct ov_camera_module *cam_mod)
{
	int ret		= -EAGAIN;
	u32 reg_val = 0;

	/* Fixme */
	if ((cam_mod->state == OV_CAMERA_MODULE_SW_STANDBY) ||
		(cam_mod->state == OV_CAMERA_MODULE_STREAMING)) {
		ret = ov_camera_module_read_reg(cam_mod,
				1,
				OV5670_V_FLIP_REG,
				&reg_val);

		if (!IS_ERR_VALUE(ret)) {
			if (cam_mod->vflip)
				reg_val |= OV5670_FLIP_BIT_MASK;
			else
				reg_val &= ~OV5670_FLIP_BIT_MASK;

			ret = ov_camera_module_write_reg(cam_mod,
					OV5670_V_FLIP_REG,
					reg_val);
		}

		ret = ov_camera_module_read_reg(cam_mod,
				1,
				OV5670_H_MIRROR_REG,
				&reg_val);

		if (!IS_ERR_VALUE(ret)) {
			if (cam_mod->hflip)
				reg_val |= OV5670_FLIP_BIT_MASK;
			else
				reg_val &= ~OV5670_FLIP_BIT_MASK;

			ret = ov_camera_module_write_reg(cam_mod,
					OV5670_H_MIRROR_REG,
					reg_val);
		}

		/* Fixme, should return the false value */
		if (IS_ERR_VALUE(ret))
			ov_camera_module_pr_err(cam_mod,
					"failed with error (%d)\n",
					ret);
	} else
		ret = 0;

	return ret;
}

static int ov5670_set_flip(struct ov_camera_module *cam_mod)
{
	int i, mode;

	if (cam_mod->vflip)
		mode |= 0x01;
	else
		mode &= ~0x01;

	if (cam_mod->hflip)
		mode |= 0x02;
	else
		mode &= ~0x02;

	ov_camera_module_pr_info(cam_mod,
			"%s(%d): flip_mode is %d, mode is %d.\n",
			__func__,
			__LINE__,
			flip_mode,
			mode);

	if (mode != flip_mode) {
		/* Reconfigure the compatible bayer order */
		switch (mode) {
		case 0:
			/* Change all the bayter order of registered configs */
			for (i = 0; i < ov5670_custom_config.num_configs; i++)
				ov5670_custom_config.configs[i].frm_fmt.code =
					V4L2_MBUS_FMT_SBGGR10_1X10;
			break;

		case 1:
			for (i = 0; i < ov5670_custom_config.num_configs; i++)
				ov5670_custom_config.configs[i].frm_fmt.code =
					V4L2_MBUS_FMT_SGBRG10_1X10;
			break;

		case 2:
			for (i = 0; i < ov5670_custom_config.num_configs; i++)
				ov5670_custom_config.configs[i].frm_fmt.code =
					V4L2_MBUS_FMT_SBGGR10_1X10;
			break;

		case 3:
			for (i = 0; i < ov5670_custom_config.num_configs; i++)
				ov5670_custom_config.configs[i].frm_fmt.code =
					V4L2_MBUS_FMT_SGBRG10_1X10;
			break;

		default:
			break;
		};
	}

	flip_mode = mode;

	return 0;
}
/*--------------------------------------------------------------------------*/

static int ov5670_s_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = ov5670_write_aec(cam_mod);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		/* nothing to be done here */
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		/* It seems unuseful */
		ret = ov5670_set_flip(cam_mod);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
				"failed with error (%d) 0x%x\n",
				ret,
				ctrl_id);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5670_s_ext_ctrls(struct ov_camera_module *cam_mod,
		struct ov_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if (ctrls->count == 1) {
		ret = ov5670_s_ctrl(cam_mod,
				ctrls->ctrls[0].id);
	} else if (ctrls->count == 2 &&
			((ctrls->ctrls[0].id == V4L2_CID_GAIN &&
			  ctrls->ctrls[1].id == V4L2_CID_EXPOSURE) ||
			 (ctrls->ctrls[1].id == V4L2_CID_GAIN &&
			  ctrls->ctrls[0].id == V4L2_CID_EXPOSURE))) {
		ret = ov5670_write_aec(cam_mod);
	} else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
				"failed with error (%d)\n",
				ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5670_start_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret = ov5670_g_VTS(cam_mod,
			&cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov5670_write_aec(cam_mod);
	if (IS_ERR_VALUE(ret))
		goto err;

	/* Before this time in calling
	 * the bayer order and flip_mode has been reconfigured;
	 * So in this moment
	 * write the flip control to the register truely */
	ret = ov5670_set_flip_reg(cam_mod);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (IS_ERR_VALUE(ov_camera_module_write_reg(cam_mod,
					0x0100,
					1)))
		goto err;

	return 0;

err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n",
			ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5670_stop_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret = ov_camera_module_write_reg(cam_mod,
			0x0100,
			0);

	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;

err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n",
			ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5670_check_camera_id(struct ov_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret |= ov_camera_module_read_reg(cam_mod,
			1,
			OV5670_PIDH_ADDR,
			&pidh);
	ret |= ov_camera_module_read_reg(cam_mod,
			1,
			OV5670_PIDL_ADDR,
			&pidl);
	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod,
				"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == OV5670_PIDH_MAGIC) && (pidl == OV5670_PIDL_MAGIC))
		ov_camera_module_pr_debug(cam_mod,
				"successfully detected camera ID 0x%02X%02X\n",
				pidh,
				pidl);
	else {
		ov_camera_module_pr_err(cam_mod,
				"wrong camera ID, expected 0x%02X%02X, detected 0x%02X%02X\n",
				OV5670_PIDH_MAGIC,
				OV5670_PIDL_MAGIC,
				pidh,
				pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;

err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n",
			ret);
	return ret;
}


/* Add for OTP, begin */
#ifdef OV5670_OTP_SUPPORT

/* Check with module vendor provider */
#define RG_RATIO_TYPICAL 0x117
#define BG_RATIO_TYPICAL 0x142

struct ov5670_otp_struct {
	int flag; /* bit[7]: info, bit[6]: wb */
	int module_integrator_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int rg_ratio;
	int bg_ratio;
	int r_gain;
	int b_gain;
	int g_gain;
} ov5670_otp;

static void ov5670_enable_mem_access(struct ov_camera_module *cam_mod)
{
	u32 reg_val = 0;

	ov_camera_module_read_reg_table(cam_mod,
			0x5002,
			&reg_val);
	ov_camera_module_write_reg(cam_mod,
			0x5002,
			reg_val & ~(1 << 3));
}

static void ov5670_disable_mem_access(struct ov_camera_module *cam_mod)
{
	u32 reg_val = 0;

	ov_camera_module_read_reg_table(cam_mod,
			0x5002,
			&reg_val);
	ov_camera_module_write_reg(cam_mod,
			0x5002,
			reg_val | (1 << 3));
}

static int ov5670_clear_otp_buffer(struct ov_camera_module *cam_mod)
{
	int i, ret;

	for (i = 0; i < 32; i++) {
		ret = ov_camera_module_write_reg(cam_mod,
				0x7010 + i,
				0x00);
		if (IS_ERR_VALUE(ret)) {
			ov_camera_module_pr_err(cam_mod,
					"%s(%d): failed with error (%d)\n",
					__func__,
					__LINE__,
					ret);
			return ret;
		}
	}
	return 0;
}

static void ov5670_set_otp_addr_range(struct ov_camera_module *cam_mod)
{
	/* otp start address, 0x7010 */
	ov_camera_module_write_reg(cam_mod,
			0x3d88,
			0x70);
	ov_camera_module_write_reg(cam_mod,
			0x3d89,
			0x10);

	/* otp end address, 0x7029 */
	ov_camera_module_write_reg(cam_mod,
			0x3d8a,
			0x70);
	ov_camera_module_write_reg(cam_mod,
			0x3d8b,
			0x29);
}

static int ov5670_check_otp(struct ov_camera_module *cam_mod)
{
	u16 otp_addr  = 0;
	int otp_group = -1;
	u32 otp_flag  = 0;

	/* Enable OTP memory access */
	ov5670_enable_mem_access(cam_mod);
	/* Clear otp buffer */
	ov5670_clear_otp_buffer(cam_mod);
	/* Enable partial otp write mode */
	ov_camera_module_write_reg(cam_mod,
			0x3d84,
			0xc0);
	/* Set address range of otp to be programmed */
	ov5670_set_otp_addr_range(cam_mod);
	/* Load otp data into buffer */
	ov_camera_module_write_reg(cam_mod,
			0x3d81,
			0x01);

	mdelay(10);

	/* Check flag to confirm if current group
	 * is empty, invalid or valid */
	otp_addr = 0;
	ov_camera_module_read_reg_table(cam_mod,
			0x7010,
			&otp_flag);
	if ((otp_flag & 0xc0) == 0x40) {
		/* base address of info group 1 */
		otp_addr  = 0x7011;
		otp_group = 1;
	} else if ((otp_flag & 0x30) == 0x10) {
		/* base address of info group 2 */
		otp_addr  = 0x7016;
		otp_group = 2;
	} else if ((otp_flag & 0x0c) == 0x04) {
		/* base address of info group 3 */
		otp_addr  = 0x701b;
		otp_group = 3;
	}

	ov_camera_module_pr_debug(cam_mod,
			"otp_flag is 0x%.4X,otp_addr is 0x%.4X,otp_group is %d.\n",
			otp_flag,
			otp_addr,
			otp_group);

	/* Read module information  */
	if (otp_addr != 0) {
		/* valid base info in otp */
		ov5670_otp.flag = 0x80;
		ov_camera_module_read_reg_table(cam_mod,
				otp_addr,
				&ov5670_otp.module_integrator_id);
		ov_camera_module_read_reg_table(cam_mod,
				otp_addr + 1,
				&ov5670_otp.lens_id);
		ov_camera_module_read_reg_table(cam_mod,
				otp_addr + 2,
				&ov5670_otp.production_year);
		ov_camera_module_read_reg_table(cam_mod,
				otp_addr + 3,
				&ov5670_otp.production_month);
		ov_camera_module_read_reg_table(cam_mod,
				otp_addr + 4,
				&ov5670_otp.production_day);
	} else {
		/* have no info in otp */
		ov5670_otp.flag					= 0x00;
		ov5670_otp.module_integrator_id = 0;
		ov5670_otp.lens_id				= 0;
		ov5670_otp.production_year		= 0;
		ov5670_otp.production_month		= 0;
		ov5670_otp.production_day		= 0;
	}

	/* Clear otp buffer */
	ov5670_clear_otp_buffer(cam_mod);
	/* Disable OTP memory access */
	ov5670_disable_mem_access(cam_mod);

	return ((ov5670_otp.flag == 0x80) ? otp_group : -EINVAL);
}

static int ov5670_read_otp(
		struct ov_camera_module *cam_mod,
		int otp_group)
{
	int ret;
	u32 reg_val  = 0;
	u32 reg_val1 = 0;
	u32 otp_flag = 0;
	u16 otp_addr = 0;

	/* Enable OTP memory access */
	ov5670_enable_mem_access(cam_mod);
	/* Enable partial otp write mode */
	ov_camera_module_write_reg(cam_mod,
			0x3d84,
			0xc0);
	/* Set address range of otp to be programmed */
	ov5670_set_otp_addr_range(cam_mod);
	/* Clear otp buffer */
	ov5670_clear_otp_buffer(cam_mod);
	/* load otp data into buffer */
	ov_camera_module_write_reg(cam_mod,
			0x3d81,
			0x01);

	mdelay(10);

	/* Read otp data */
	/* OTP WB calibration */
	otp_addr = 0;

	/* Fixme */
#if 0
	/* It seems unnecessary because we've read before */
	ov_camera_module_read_reg_table(cam_mod,
			0x7020,
			&otp_flag);
	if ((otp_flag & 0xc0) == 0x40) {
		/* base address of WB Calibration group 1 */
		otp_addr = 0x7021;
	} else if ((otp_flag & 0x30) == 0x10) {
		/* base address of WB Calibration group 2 */
		otp_addr = 0x7024;
	} else if ((otp_flag & 0x0c) == 0x04) {
		/* base address of WB Calibration group 3 */
		otp_addr = 0x7027;
	} else {
		ov_camera_module_pr_err(cam_mod,
				"otp_flag(%.2X) is invalid!\n",
				otp_flag);
		ret = -EINVAL;
		goto err;
	}
#endif

	switch (otp_group) {
	case 1:
		/* base address of WB Calibration group 1 */
		otp_addr = 0x7021;
		break;
	case 2:
		/* base address of WB Calibration group 2 */
		otp_addr = 0x7024;
		break;
	case 3:
		/* base address of WB Calibration group 3 */
		otp_addr = 0x7027;
		break;
	default:
		ov_camera_module_pr_err(cam_mod,
				"otp_flag(%.2X) is invalid!\n",
				otp_flag);
		ret = -EINVAL;
		goto err;
	}

	ov_camera_module_pr_debug(cam_mod,
			"otp_addr is 0x%.4X, otp_group is %d.\n",
			otp_addr,
			otp_group);

	if (otp_addr != 0) {
		ov5670_otp.flag	|= 0x40;

		ov_camera_module_read_reg_table(cam_mod,
				otp_addr + 2,
				&reg_val);

		ov_camera_module_read_reg_table(cam_mod,
				otp_addr,
				&reg_val1);
		ov5670_otp.rg_ratio =
			(reg_val1 << 2) + ((reg_val >> 6) & 0x03);

		ov_camera_module_read_reg_table(cam_mod,
				otp_addr + 1,
				&reg_val1);
		ov5670_otp.bg_ratio =
			(reg_val1 << 2) + ((reg_val >> 4) & 0x03);
	} else {
		ov5670_otp.rg_ratio = 0;
		ov5670_otp.bg_ratio = 0;
	}

	/* Clear otp buffer */
	ov5670_clear_otp_buffer(cam_mod);

	ret = 0;
err:
	/* Disable OTP memory access */
	ov5670_disable_mem_access(cam_mod);

	return ret;
}


static int ov5670_apply_otp(struct ov_camera_module *cam_mod)
{
	int ret;
	int rg, bg, base_gain;
	u32 reg_val = 0;

	/* Enable OTP memory access */
	ov5670_enable_mem_access(cam_mod);
	/* Clear otp buffer */
	ov5670_clear_otp_buffer(cam_mod);
	/* Enable partial otp write mode */
	ov_camera_module_write_reg(cam_mod,
			0x3d84,
			0x40);
	/* Set address range of otp to be programmed */
	ov5670_set_otp_addr_range(cam_mod);

	/* Update otp data */
	if (ov5670_otp.flag & 0x40) {
		rg = ov5670_otp.rg_ratio;
		bg = ov5670_otp.bg_ratio;

		/* Calculate G gain */
		ov5670_otp.r_gain = (RG_RATIO_TYPICAL * 1000) / rg;
		ov5670_otp.b_gain = (BG_RATIO_TYPICAL * 1000) / bg;
		ov5670_otp.g_gain = 1000;

		/* Choose the min gain's value to base_gain */
		if ((ov5670_otp.r_gain < 1000) ||
				(ov5670_otp.b_gain < 1000)) {
			if (ov5670_otp.r_gain < ov5670_otp.b_gain)
				base_gain = ov5670_otp.r_gain;
			else
				base_gain = ov5670_otp.b_gain;
		} else
			base_gain = ov5670_otp.g_gain;

		ov5670_otp.r_gain =
			(0x400 * ov5670_otp.r_gain) / base_gain;
		ov5670_otp.g_gain =
			(0x400 * ov5670_otp.g_gain) / base_gain;
		ov5670_otp.b_gain =
			(0x400 * ov5670_otp.b_gain) / base_gain;

		/* Update sensor WB gain */
		if (ov5670_otp.r_gain > 0x400) {
			ov_camera_module_write_reg(cam_mod,
					0x5032,
					ov5670_otp.r_gain >> 8);
			ov_camera_module_write_reg(cam_mod,
					0x5033,
					ov5670_otp.r_gain & 0xff);

			ov_camera_module_pr_debug(cam_mod,
					"r_gain is 0x%.4X.\n",
					ov5670_otp.r_gain);
		}

		if (ov5670_otp.g_gain > 0x400) {
			ov_camera_module_write_reg(cam_mod,
					0x5034,
					ov5670_otp.g_gain >> 8);
			ov_camera_module_write_reg(cam_mod,
					0x5035,
					ov5670_otp.g_gain & 0xff);

			ov_camera_module_pr_debug(cam_mod,
					"g_gain is 0x%.3X.\n",
					ov5670_otp.g_gain);
		}

		if (ov5670_otp.b_gain > 0x400) {
			ov_camera_module_write_reg(cam_mod,
					0x5036,
					ov5670_otp.b_gain >> 8);
			ov_camera_module_write_reg(cam_mod,
					0x5037,
					ov5670_otp.b_gain & 0xff);

			ov_camera_module_pr_debug(cam_mod,
					"b_gain is %.3X.\n",
					ov5670_otp.b_gain);
		}
	} else {
		ov_camera_module_pr_err(cam_mod,
				"ov5670_otp's value invalid!\n");
		ret = -EINVAL;
		goto err;
	}

	/* Enable otp write function */
	ov_camera_module_read_reg_table(cam_mod,
			0x3d80,
			&reg_val);
	ov_camera_module_write_reg(cam_mod,
			0x3d80,
			reg_val | 0x01);

	mdelay(20);

	/* Clear otp buffer */
	ov5670_clear_otp_buffer(cam_mod);

	ret = 0;
err:
	/* Disable OTP memory access */
	ov5670_disable_mem_access(cam_mod);

	return ret;
}

static void ov5670_print_otp_info(struct ov_camera_module *cam_mod)
{
	ov_camera_module_pr_debug(cam_mod,
			"OTP flag                 is 0x%.2X\n",
			ov5670_otp.flag);
	ov_camera_module_pr_debug(cam_mod,
			"OTP module_integrator_id is 0x%.2X\n",
			ov5670_otp.module_integrator_id);
	ov_camera_module_pr_debug(cam_mod,
			"OTP lens_id              is 0x%.2X\n",
			ov5670_otp.lens_id);
	ov_camera_module_pr_debug(cam_mod,
			"OTP production_year      is 0x%.2X\n",
			ov5670_otp.production_year);
	ov_camera_module_pr_debug(cam_mod,
			"OTP production_month     is 0x%.2X\n",
			ov5670_otp.production_month);
	ov_camera_module_pr_debug(cam_mod,
			"OTP production_day       is 0x%.2X\n",
			ov5670_otp.production_day);
	ov_camera_module_pr_debug(cam_mod,
			"OTP rg_ratio             is 0x%.2X\n",
			ov5670_otp.rg_ratio);
	ov_camera_module_pr_debug(cam_mod,
			"OTP bg_ratio             is 0x%.2X\n",
			ov5670_otp.bg_ratio);
	ov_camera_module_pr_debug(cam_mod,
			"OTP r_gain               is 0x%.2X\n",
			ov5670_otp.r_gain);
	ov_camera_module_pr_debug(cam_mod,
			"OTP g_gain               is 0x%.2X\n",
			ov5670_otp.g_gain);
	ov_camera_module_pr_debug(cam_mod,
			"OTP b_gain               is 0x%.2X\n",
			ov5670_otp.b_gain);
}

/* Called after the sensor initialization */
static int ov5670_update_otp(struct ov_camera_module *cam_mod)
{
	int ret;
	int otp_group;

	ov_camera_module_pr_info(cam_mod,
			"%s(%d): going to update otp!\n",
			__func__,
			__LINE__);

	if (!ov5670_otp.flag) {
		ret = ov5670_check_otp(cam_mod);
		if (ret <= 0) {
			ov_camera_module_pr_err(cam_mod,
					"%s(%d): check otp failed with error (%d)\n",
					__func__,
					__LINE__,
					ret);
			goto err;
		}
		otp_group = ret;

		ret = ov5670_read_otp(cam_mod,
				otp_group);
		if (ret != 0) {
			ov_camera_module_pr_err(cam_mod,
					"%s(%d): read otp failed with error (%d)\n",
					__func__,
					__LINE__,
					ret);
			goto err;
		}
	} else
		ov_camera_module_pr_info(cam_mod,
				"%s(%d): already read OTP data!\n",
				__func__,
				__LINE__);

	ret = ov5670_apply_otp(cam_mod);
	if (ret != 0) {
		ov_camera_module_pr_err(cam_mod,
				"%s(%d): apply otp failed with error (%d)\n",
				__func__,
				__LINE__,
				ret);
		goto err;
	}

	ov5670_print_otp_info(cam_mod);

	ov_camera_module_pr_info(cam_mod,
			"%s(%d): update otp success!\n",
			__func__,
			__LINE__);

	return 0;

err:
	return ret;
}
#endif


static int ov5670_init_common(struct ov_camera_module *cam_mod)
{
	ov_camera_module_pr_info(cam_mod, "%s().\n",
			__func__);

	return ov_camera_module_write_reglist(cam_mod,
			ov5670_init_regs,
			sizeof(ov5670_init_regs) / sizeof(ov5670_init_regs[0]));
}

/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops ov5670_camera_module_core_ops = {
	.g_ctrl				 = ov_camera_module_g_ctrl,
	.s_ctrl				 = ov_camera_module_s_ctrl,
	.s_ext_ctrls		 = ov_camera_module_s_ext_ctrls,
	.s_power			 = ov_camera_module_s_power,
	.ioctl				 = ov_camera_module_ioctl
};

static struct v4l2_subdev_video_ops ov5670_camera_module_video_ops = {
	.enum_frameintervals = ov_camera_module_enum_frameintervals,
	.s_mbus_fmt			 = ov_camera_module_s_fmt,
	.g_mbus_fmt			 = ov_camera_module_g_fmt,
	.try_mbus_fmt		 = ov_camera_module_try_fmt,
	.s_frame_interval	 = ov_camera_module_s_frame_interval,
	.s_stream			 = ov_camera_module_s_stream
};

static struct v4l2_subdev_ops ov5670_camera_module_ops = {
	.core				 = &ov5670_camera_module_core_ops,
	.video				 = &ov5670_camera_module_video_ops
};

static struct ov_camera_module_custom_config ov5670_custom_config = {
#if 1
	.set_flip			 = ov5670_set_flip,
#ifdef OV5670_OTP_SUPPORT
	.update_otp			 = ov5670_update_otp,
#endif
#endif
	.init_common		 = ov5670_init_common,
	.start_streaming	 = ov5670_start_streaming,
	.stop_streaming		 = ov5670_stop_streaming,
	.s_ctrl				 = ov5670_s_ctrl,
	.s_ext_ctrls		 = ov5670_s_ext_ctrls,
	.g_ctrl				 = ov5670_g_ctrl,
	.g_timings			 = ov5670_g_timings,
	.check_camera_id	 = ov5670_check_camera_id,
	.configs			 = ov5670_configs,
	.num_configs		 = sizeof(ov5670_configs) /
		sizeof(ov5670_configs[0]),
	.power_up_delays_ms  = {5, 20, 0}
};

static int __init ov5670_probe(
		struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;

	dev_info(&client->dev, "probing...\n");

	v4l2_i2c_subdev_init(&ov5670.sd,
			client,
			&ov5670_camera_module_ops);

	ret = ov_camera_module_init(&ov5670,
			&ov5670_custom_config);

	if (IS_ERR_VALUE(ret))
		goto err;

	dev_info(&client->dev, "probing successful\n");

	return 0;

err:
	dev_err(&client->dev,
			"probing failed with error (%d)\n",
			ret);
	return ret;
}

/* ======================================================================== */

static int __exit ov5670_remove(
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

static const struct i2c_device_id ov5670_id[] = {
	{OV5670_DRIVER_NAME, 0},
	{}
};

static struct of_device_id ov5670_of_match[] = {
	{.compatible = "omnivision," OV5670_DRIVER_NAME "-v4l2-i2c-subdev",},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov5670_id);

static struct i2c_driver ov5670_i2c_driver = {
	.driver = {
		.name			= OV5670_DRIVER_NAME,
		.owner			= THIS_MODULE,
		.of_match_table = ov5670_of_match
	},
	.probe				= ov5670_probe,
	.remove				= __exit_p(ov5670_remove),
	.id_table			= ov5670_id,
};

module_i2c_driver(ov5670_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for ov5670");
MODULE_AUTHOR("Kunkka Lu");
MODULE_LICENSE("GPL");
