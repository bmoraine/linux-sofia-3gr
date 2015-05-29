/*
 * Support for Himax HM2051 8M camera sensor.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 */

#ifndef __HM2051_H__
#define __HM2051_H__
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>

#include <linux/atomisp_platform.h>

#define HM2051_NAME		"hm2051"
#define HM2051_BAYER_ORDER V4L2_MBUS_FMT_SBGGR10_1X10

#define ULPM_PROCEDURE 1

/* Defines for register writes and register array processing */
#define I2C_MSG_LENGTH		0x2
#define I2C_RETRY_COUNT		5

#define HM2051_FOCAL_LENGTH_NUM	334	/*3.34mm*/
#define HM2051_FOCAL_LENGTH_DEM	100
#define HM2051_F_NUMBER_DEFAULT_NUM	24
#define HM2051_F_NUMBER_DEM	10

#define MAX_FMTS		1

#define HM2051_INTEGRATION_TIME_MARGIN	5

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define HM2051_FOCAL_LENGTH_DEFAULT 0x1B70064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define HM2051_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define HM2051_F_NUMBER_RANGE 0x180a180a
#define HM2051_ID	0x2051


#define HM2051_BIN_FACTOR_MAX 4
#define HM2051_FINE_INTG_TIME_MIN 0
#define HM2051_FINE_INTG_TIME_MAX_MARGIN 5
#define HM2051_COARSE_INTG_TIME_MIN 0
#define HM2051_COARSE_INTG_TIME_MAX_MARGIN 5/*(0xffff - 6) */
/*
 * HM2051 System control registers
 */
#define HM2051_SW_RESET					0x0022
#define HM2051_SW_STREAM				0x0005

#define HM2051_SC_CMMN_CHIP_ID_H		0x0001
#define HM2051_SC_CMMN_CHIP_ID_L		0x0002

#define HM2051_PRE_PLL_CLK_DIV			0x002A
#define HM2051_PLL_MULTIPLIER			  0x002B
/*#define HM2051_VT_PIX_CLK_DIV			0x0301 */
#define HM2051_VT_SYS_CLK_DIV			  0x0026


#define HM2051_COMMAND_UPDATE     0x0100

#define HM2051_EXPOSURE_H				0x0015
#define HM2051_EXPOSURE_L				0x0016
#define HM2051_AGAIN            0x0018
#define HM2051_DGAIN            0x001D
/*#define HM2051_AGC						  0x0205 */

#define HM2051_HORIZONTAL_START		0x0344
#define HM2051_VERTICAL_START			0x0346
#define HM2051_HORIZONTAL_END			0x0348
#define HM2051_VERTICAL_END				0x034a

/*#define HM2051_HORIZONTAL_OUTPUT_SIZE	0x034c */
/*#define HM2051_VERTICAL_OUTPUT_SIZE		0x034e */

/* Add for sofia blanking sync */
#define HM2051_IMG_OUT_CONF          0x000F
#define HM2051_FIXED_FRAME_RATE_MASK 0x08
#define HM2051_IMG_OUT_VALUE        0x08
/* End Add for sofia blanking sync */

#define HM2051_REG_BLANKING_ROW_H				0x0010
#define HM2051_REG_BLANKING_ROW_L				0x0011
#define HM2051_REG_BLANKING_COLUMN				0x0013

/*#define HM2051_DIGITAL_GAIN_GR		0x020e */
/*#define HM2051_DIGITAL_GAIN_R			0x020e */
/*#define HM2051_DIGITAL_GAIN_B			0x020e */
/*#define HM2051_DIGITAL_GAIN_GB		0x020e */

#define HM2051_START_STREAMING			0x03
#define HM2051_STOP_STREAMING		    0x02

#define HM2051_RDCFG_REG      0x0006
#define hm2051_IMAGE_ORIENTATION_REG HM2051_RDCFG_REG
#define HM2051_RDCFG_MODE_MASK 0x0C

enum hm2051_rdcfg_mode {
	RDCFG_MODE_1616_1216  = 0,
	RDCFG_MODE_1616_736  = 4,
	RDCFG_MODE_UNKNOW = 99
};

#define HM2051_FULL_FRAME_VTS_THRESHOLD 1264
#define HM2051_CROP_FRAME_VTS_THRESHOLD 784

struct regval_list {
	u16 reg_num;
	u8 value;
};

struct hm2051_resolution {
	u8 *desc;
	const struct hm2051_reg *regs;
	int res;
	int width;
	int height;
	int pix_clk_freq;
	int fps;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 bin_factor_x;
	u8 bin_factor_y;
	u8 bin_mode;
	bool used;
	int skip_frames;
	int horizontal_start;
	int horizontal_end;
	int vertical_start;
	int vertical_end;
};

struct hm2051_format {
	u8 *desc;
	u32 pixelformat;
	struct hm2051_reg *regs;
};

struct hm2051_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, s32 value);
};

/*
 * HM2051 device structure.
 */
struct hm2051_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct mutex input_lock;

	struct camera_sensor_platform_data *platform_data;
	int vt_pix_clk_freq_mhz;
	int fmt_idx;
	int run_mode;
	u8 res;
	u8 type;
	u16 coarse_itg;
	u16 fine_itg;
	u16 digital_gain;
	u16 gain;
	u16 sensor_blanking;  /* Add for sofia blanking sync */
};

enum hm2051_tok_type {
	HM2051_8BIT  = 0x0001,
	HM2051_16BIT = 0x0002,
	HM2051_32BIT = 0x0004,
	HM2051_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	HM2051_TOK_DELAY  = 0xfe00,	/* delay token for reg list */
	HM2051_TOK_MASK = 0xfff0
};

/**
 * struct hm2051_reg - MI sensor  register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct hm2051_reg {
	enum hm2051_tok_type type;
	u16 reg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

#define to_hm2051_sensor(x) container_of(x, struct hm2051_device, sd)

#define HM2051_MAX_WRITE_BUF_SIZE	30

struct hm2051_write_buffer {
	u16 addr;
	u8 data[HM2051_MAX_WRITE_BUF_SIZE];
};

struct hm2051_write_ctrl {
	int index;
	struct hm2051_write_buffer buffer;
};

#if ULPM_PROCEDURE
static struct hm2051_reg const hm2051_stream_on[] = {
	{HM2051_8BIT, 0x0000, 0x01},
	{HM2051_8BIT, 0x0100, 0x01},
	{HM2051_8BIT, 0x0101, 0x01},
	{HM2051_8BIT, 0x0005, 0x03},	/*Turn on rolling shutter */
	{HM2051_TOK_TERM, 0, 0}
};

static struct hm2051_reg const hm2051_stream_off[] = {
	{HM2051_8BIT, 0x0005, 0x02},	/*Turn off rolling shutter*/
	{HM2051_TOK_TERM, 0, 0}
};

#endif

static const struct i2c_device_id hm2051_id[] = {
	{HM2051_NAME, 0},
	{}
};

static struct hm2051_reg const hm2051_global_setting[] = {
	/*---------------------------------------------------*/
	/* Initial*/
	/*---------------------------------------------------*/
	{HM2051_8BIT, 0x0022, 0x00},/* RESET*/
	/*---------------------------------------------------*/
	/* CMU update*/
	/*---------------------------------------------------*/
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},
	{HM2051_8BIT, 0x0005, 0x02},/* power up*/
	{HM2051_TOK_DELAY, 0, 50},

	{HM2051_8BIT, 0x0026, 0x0B},/* PLL1, mipi pll_pre_div*/
	{HM2051_8BIT, 0x002A, 0x4F},
	/* PLL1, mipi pll_div (pclk=pktclk= 002A + 1)*/

	/*---------------------------------------------------*/
	/* Digital function*/
	/*---------------------------------------------------*/
	{HM2051_8BIT, 0x0006, 0x00},/* [1] hmirror, [0] vflip*/
	{HM2051_8BIT, 0x000F, 0x00},/* [1] fixed frame rate mode,*/
	/*[0] non fixed frame rate mode */
	{HM2051_8BIT, 0x0010, 0x00},/* just for VBI (high byte) test */
	{HM2051_8BIT, 0x0011, 0x33},/* just for VBI (Low  byte) 3ms test */
	{HM2051_8BIT, 0x0024, 0x40},/* mipi enable*/
	{HM2051_8BIT, 0x0027, 0x23},
	/* OUTFMT, after BPC, [7] pre_vsync_en*/
	{HM2051_8BIT, 0x0065, 0x01},/* Hscan_delay (default=1)*/
	{HM2051_8BIT, 0x0074, 0x13},/* disable output black rows*/
	/*---------------------------------------------------*/
	/* Analog*/
	/*---------------------------------------------------*/
	{HM2051_8BIT, 0x002B, 0x00},/*  clk divider*/
	{HM2051_8BIT, 0x002C, 0x06},/* PLL cfg: CP, LPF, use PLL clk*/

	{HM2051_8BIT, 0x0040, 0x00}, /* BLC  //0A->00*/
	{HM2051_8BIT, 0x0044, 0x03},/* enable BLC, enable BLC IIR*/
	{HM2051_8BIT, 0x0045, 0x63},
	/* CFPN cfg, 0x65=repeat 32 times, 0x55=repeat 16 times*/
	{HM2051_8BIT, 0x0046, 0x5F},
	/* CFPN cfg, enable IIR, weight=1/4 new, CFPN applied to BLC*/
	{HM2051_8BIT, 0x0049, 0xC0},/*improve BLC_hunting*/
	{HM2051_8BIT, 0x004B, 0x03},/*improve BLC_hunting*/

	{HM2051_8BIT, 0x0070, 0x2F},/* ADD  0923*/
	{HM2051_8BIT, 0x0072, 0xFB},/*8F -> FB  0923*/
	{HM2051_8BIT, 0x0073, 0x77},/* ADD  0923 for 30 fps*/
	{HM2051_8BIT, 0x0075, 0x40},
	/* Negative CP is controlled by INT*/
	{HM2051_8BIT, 0x0078, 0x65},/* ADD  0923 for 30 fps*/

	{HM2051_8BIT, 0x0080, 0x98},
	/* fd_clamp_en_d=1, tg_boost_en_d=1  //90 -> 98  0923*/
	{HM2051_8BIT, 0x0082, 0x09},
	/* fd_clamp_en_d=1, tg_boost_en_d=1*/
	{HM2051_8BIT, 0x0083, 0x3C},
	/* VRPP=avdd+1.36V, VRNP=-1V, w/ lowest pump clk freq.*/
	{HM2051_8BIT, 0x0087, 0x41},
	/* disable dsun clamp first  31 -> 41 0923*/
	{HM2051_8BIT, 0x008D, 0x20},/* pix_disc_diff_en_d=1*/
	{HM2051_8BIT, 0x008E, 0x30},

	{HM2051_8BIT, 0x009D, 0x11},/* Nramp_rst1,2*/
	{HM2051_8BIT, 0x009E, 0x12},/* Nramp_rst3,4*/

	{HM2051_8BIT, 0x0090, 0x00},/* gain table*/
	{HM2051_8BIT, 0x0091, 0x01},/* gain table*/
	{HM2051_8BIT, 0x0092, 0x02},/* gain table*/
	{HM2051_8BIT, 0x0093, 0x03},/* gain table*/


	{HM2051_8BIT, 0x00C0, 0x64},/* col_ldo setting*/
	{HM2051_8BIT, 0x00C1, 0x15},
	/* pvdd_refsel=5h(for power noise), pvdd_lorefsel*/
	{HM2051_8BIT, 0x00C2, 0x00},/* ramp power consumption*/

	{HM2051_8BIT, 0x00C3, 0x02},/* comp1,2,3_bias_sel*/
	{HM2051_8BIT, 0x00C4, 0x0B},/* column ADC cfg*/
	{HM2051_8BIT, 0x00C6, 0x83},
	/*sf_srcdr_shrt_en_right_d=1,sf_always_on_d=0(improve noise)*/
	{HM2051_8BIT, 0x00C7, 0x02},
	/* sr_sel_sh_d (reduce CFPN @ high AVDD) ADD 0923*/
	{HM2051_8BIT, 0x00CC, 0x00},/* mipi skew[5:0]*/

	{HM2051_8BIT, 0x4B20, 0xDE},/*dis continue mode*/
	{HM2051_8BIT, 0x4B3B, 0x12},/* MIPI analog setting*/
	{HM2051_8BIT, 0x4B41, 0x10},
	/* HS0_D=1,prevent enter test mode(clk lane=0)*/

	/*Star of BPC setting*/
	{HM2051_8BIT, 0x0165, 0x03},/*[1:0]0:24 1:32, 2:48, 3:80*/
	{HM2051_8BIT, 0x018C, 0x00},/*[7:6]Dark_raw_enable*/

	{HM2051_8BIT, 0x0195, 0x06},/*X_offset[2:0]*/
	{HM2051_8BIT, 0x0196, 0x4F},/*X_offset[7:0]*/
	{HM2051_8BIT, 0x0197, 0x04},/*Y_offset[2:0]*/
	{HM2051_8BIT, 0x0198, 0xBF},/*Y_offset[7:0]*/

	{HM2051_8BIT, 0x0144, 0x10},
	/*BPC_HOT_TH[8],[1]Median Filter with current pixel*/
	{HM2051_8BIT, 0x0140, 0x20},/*BPC_HOT_TH[7:0]*/
	{HM2051_8BIT, 0x015A, 0x30},/*BPC_HOT_2*/
	{HM2051_8BIT, 0x015D, 0xFF},/*BPC_HOT_3*/
	{HM2051_8BIT, 0x0160, 0x61},
	/*[0]hot_replace[1]cold_replace[3:2]Max1_Max2*/
	/*[4]correct_all[5]Dynamic[6]Static[7]no write back */

	{HM2051_8BIT, 0x0025, 0x00},

	{HM2051_TOK_TERM, 0, 0}
};

static struct hm2051_reg const hm2051_SUB2_800x600_56fps[] = {
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_TOK_DELAY, 0, 100},

	{HM2051_8BIT, 0x0006, 0x00},
	{HM2051_8BIT, 0x000D, 0x01},
	{HM2051_8BIT, 0x000E, 0x01},
	{HM2051_8BIT, 0x0660, 0x00},
	{HM2051_8BIT, 0x0661, 0x04},
	{HM2051_8BIT, 0x0662, 0x03},
	{HM2051_8BIT, 0x0663, 0x23},
	{HM2051_8BIT, 0x0664, 0x00},
	{HM2051_8BIT, 0x0665, 0x04},
	{HM2051_8BIT, 0x0666, 0x02},
	{HM2051_8BIT, 0x0667, 0x5B},
	{HM2051_8BIT, 0x4B50, 0x08},
	{HM2051_8BIT, 0x4B51, 0xE6},
	{HM2051_8BIT, 0x4B0A, 0x03},
	{HM2051_8BIT, 0x4B0B, 0x20},

	/* --------------------------------------------------- */
	/* CMU update */
	/* --------------------------------------------------- */
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},

	/*{HM2051_8BIT, 0x0005, 0x03}, */

	{HM2051_TOK_TERM, 0, 0}
};

static struct hm2051_reg const hm2051_1280x720_45fps[] = {
	{HM2051_8BIT, 0x0005, 0x02},/* power up */
	{HM2051_TOK_DELAY, 0, 100}, /* 10FPS  delay one frame */

	{HM2051_8BIT, 0x0006, 0x04},
	{HM2051_8BIT, 0x000D, 0x00},
	{HM2051_8BIT, 0x000E, 0x00},

	{HM2051_8BIT, 0x0660, 0x00},
	{HM2051_8BIT, 0x0661, 0xA8},
	{HM2051_8BIT, 0x0662, 0x05},
	{HM2051_8BIT, 0x0663, 0xA7},
	{HM2051_8BIT, 0x0664, 0x00},
	{HM2051_8BIT, 0x0665, 0x08},
	{HM2051_8BIT, 0x0666, 0x02},
	{HM2051_8BIT, 0x0667, 0xD7},

	{HM2051_8BIT, 0x4B50, 0x00},
	{HM2051_8BIT, 0x4B51, 0xAF},

	{HM2051_8BIT, 0x4B0A, 0x05},
	{HM2051_8BIT, 0x4B0B, 0x00},
	{HM2051_8BIT, 0x4B30, 0x0E},
	{HM2051_8BIT, 0x4B30, 0x0F},
	/* --------------------------------------------------- */
	/* CMU update */
	/* --------------------------------------------------- */
	{HM2051_8BIT, 0x0000, 0x00},/* */
	{HM2051_8BIT, 0x0100, 0x00},/* */
	{HM2051_8BIT, 0x0101, 0x00},/* */

	/*{HM2051_8BIT, 0x0005, 0x03},// */
	{HM2051_TOK_TERM, 0, 0}

};

static struct hm2051_reg const hm2051_1600x900_30fps[] = {

/*--------------------------------------------------- */
/* Initial */
/*--------------------------------------------------- */

	/*{HM2051_8BIT, 0x0005, 0x02},// power up*/
	/*{HM2051_TOK_DELAY,0,100}, // 10FPS  delay one frame */
	/*--------------------------------------------------- */
	/* Digital function */
	/*--------------------------------------------------- */
	{HM2051_8BIT, 0x0006, 0x00},/* [1] hmirror, [0] vflip */
	{HM2051_8BIT, 0x000D, 0x00},
	{HM2051_8BIT, 0x000E, 0x00},
	/* ---------------------------------------------------/ */
	/* mipi-tx settings */
	/* --------------------------------------------------- */
	{HM2051_8BIT, 0x0123, 0xD5},
	{HM2051_8BIT, 0x0660, 0x00},  /*win x_st Hb */
	{HM2051_8BIT, 0x0661, 0x00},  /*win x_st Lb */
	{HM2051_8BIT, 0x0662, 0x06},  /*win x_ed Hb */
	{HM2051_8BIT, 0x0663, 0x4F},  /*win x_ed Lb */
	{HM2051_8BIT, 0x0664, 0x00},  /*win y_st Hb */
	{HM2051_8BIT, 0x0665, 0x96},  /*win y_st Lb */
	{HM2051_8BIT, 0x0666, 0x04},  /*win y_ed Hb */
	{HM2051_8BIT, 0x0667, 0x29},  /*win y_ed Lb */
	{HM2051_8BIT, 0x4B50, 0x08},/* pre_h Hb 09->08 */
	{HM2051_8BIT, 0x4B51, 0xE2},/* pre_h Lb 22->B2  //B2 -> E2 0923 */
	{HM2051_8BIT, 0x4B0A, 0x06},
	{HM2051_8BIT, 0x4B0B, 0x40},
	{HM2051_8BIT, 0x4B20, 0xDE},/*discontinuous mode*/
	{HM2051_8BIT, 0x4B30, 0x0E},
	{HM2051_8BIT, 0x4B30, 0x0F},
	/* --------------------------------------------------- */
	/* CMU update */
	/* --------------------------------------------------- */
	{HM2051_8BIT, 0x0000, 0x00},/* */
	{HM2051_8BIT, 0x0100, 0x00},/* */
	{HM2051_8BIT, 0x0101, 0x00},/* */
	/*--------------------------------------------------- */
	/* Turn on rolling shutter */
	/*--------------------------------------------------- */
	/* {HM2051_8BIT, 0x0005, 0x03}, */
	{HM2051_TOK_TERM, 0, 0}
};

static struct hm2051_reg const hm2051_1600x1200_28fps[] = {

	/*{HM2051_8BIT, 0x0005, 0x02},// power up*/
	/*{HM2051_TOK_DELAY,0,100}, // 10FPS  delay one frame */

	{HM2051_8BIT, 0x0006, 0x00},
	{HM2051_8BIT, 0x000D, 0x00},
	{HM2051_8BIT, 0x000E, 0x00},
	{HM2051_8BIT, 0x000F, 0x00},
	/* [1] fixed frame rate mode, [0] non fixed frame rate mode  */
	{HM2051_8BIT, 0x0024, 0x40},/* mipi enable*/
	{HM2051_8BIT, 0x0027, 0x23},
	/* OUTFMT, after BPC, [7] pre_vsync_en*/
	{HM2051_8BIT, 0x0065, 0x01},/* Hscan_delay (default=1)*/
	{HM2051_8BIT, 0x0074, 0x13},/* disable output black rows*/

	/*static dymaic BPC*/
	{HM2051_8BIT, 0x0160, 0x65},  /*static BPC switch*/
	{HM2051_8BIT, 0x0183, 0x00},  /*static BPC windows offset*/
	{HM2051_8BIT, 0x0123, 0xD5},

	{HM2051_8BIT, 0x0660, 0x00},
	{HM2051_8BIT, 0x0661, 0x08},
	{HM2051_8BIT, 0x0662, 0x06},
	{HM2051_8BIT, 0x0663, 0x47},
	{HM2051_8BIT, 0x0664, 0x00},
	{HM2051_8BIT, 0x0665, 0x08},
	{HM2051_8BIT, 0x0666, 0x04},
	{HM2051_8BIT, 0x0667, 0xB7},

	{HM2051_8BIT, 0x4B50, 0x08},
	{HM2051_8BIT, 0x4B51, 0xEA},
	{HM2051_8BIT, 0x4B0A, 0x06},
	{HM2051_8BIT, 0x4B0B, 0x40},
	{HM2051_8BIT, 0x4B20, 0xDE},/*discontinuous mode*/
	{HM2051_8BIT, 0x4B07, 0xBD},
	{HM2051_8BIT, 0x4B30, 0x0E},
	{HM2051_8BIT, 0x4B30, 0x0F},

	/* ---------------------------------------------------*/
	/* CMU update */
	/* ---------------------------------------------------*/
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},
	/*---------------------------------------------------*/
	/* Turn on rolling shutter*/
	/*---------------------------------------------------*/
	/*{HM2051_8BIT, 0x0005, 0x02},*/

	{HM2051_TOK_TERM, 0, 0}
};

/*
preview: 16:9
capture: 16:9 4:3
video:   16:9 3:2
*/
/* */
struct hm2051_resolution hm2051_res_preview[] = {
#if 0
	{
		.desc = "THIS IS PREVIEW SUB2 :hm2051_800x600_56fps",
		.width = 800,
		.height = 600,
		.pix_clk_freq = 86666667,
		.fps = 56,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 658,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_SUB2_800x600_56fps,
		.skip_frames = 1,
		.horizontal_start = 0,
		.horizontal_end = 799,/*1615, */
		.vertical_start = 0,
		.vertical_end = 599,/*735, */
	},
	{
		.desc = "THIS IS PREVIEW:hm2051_1296x736_47fps",
		.width = 1280,
		.height = 720,
		.pix_clk_freq = 86666667,
		.fps = 48,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 786,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_1280x720_45fps,
		.skip_frames = 1,
		.horizontal_start = 0,
		.horizontal_end = 1279,
		.vertical_start = 0,
		.vertical_end = 719,
	},

	{
		.desc = "THIS IS PREVIEW:hm2051_1600x900_30fps",
		.width = 1600,
		.height = 900,
		.pix_clk_freq = 86666667,
		.fps = 29,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 1266,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_1600x900_30fps,
		.skip_frames = 1,
		.horizontal_start = 0,
		.horizontal_end = 1599,
		.vertical_start = 0,
		.vertical_end = 899,
	},
#endif
	{
		.desc = "THIS IS PREVIEW:hm2051_1600x1200_28fps",
		.width = 1600,
		.height = 1200,
		.pix_clk_freq = 86666667,
		.fps = 28,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 1266,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_1600x1200_28fps,
		.skip_frames = 1,
		.horizontal_start = 8,
		.horizontal_end = 1607,
		.vertical_start = 8,
		.vertical_end = 1207,
	},
};
#define N_RES_PREVIEW (ARRAY_SIZE(hm2051_res_preview))

struct hm2051_resolution hm2051_res_still[] = {
	{
		.desc = "THIS IS PREVIEW:hm2051_1600x1200_28fps",
		.width = 1600,
		.height = 1200,
		.pix_clk_freq = 86666667,
		.fps = 28,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 1266,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_1600x1200_28fps,
	  .skip_frames = 1,
	  .horizontal_start = 8,
	  .horizontal_end = 1607,
	  .vertical_start = 8,
	  .vertical_end = 1207,
	},
};
#define N_RES_STILL (ARRAY_SIZE(hm2051_res_still))

struct hm2051_resolution hm2051_res_video[] = {
	{
		.desc = "THIS IS VIDEO:hm2051_1296x736_45fps",
		.width = 1280,
		.height = 720,
		.pix_clk_freq = 86666667,
		.fps = 45,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 786,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_1280x720_45fps,
		.skip_frames = 1,
		.horizontal_start = 0,
		.horizontal_end = 1279,
		.vertical_start = 0,
		.vertical_end = 719,
	},
};
#define N_RES_VIDEO (ARRAY_SIZE(hm2051_res_video))

static struct hm2051_resolution *hm2051_res = hm2051_res_preview;
static int N_RES = N_RES_PREVIEW;
#endif
