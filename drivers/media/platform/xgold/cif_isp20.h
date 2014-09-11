/*
 ****************************************************************
 *
 *  Intel CIF ISP 2.0 driver
 *
 *  Copyright (C) 2011-2014 Intel Mobile Communications GmbH
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Note:
 *     03/08/2012: sensor related setting are moved to sensor file.
 *
 ****************************************************************
 */

#ifndef _CIF_ISP20_H
#define _CIF_ISP20_H

#include "cif_isp20_pltfrm.h"
#include "cif_isp20_img_src.h"
#include "cif_isp20_isp.h"
#include "cif_isp20_dma.h"

#include <media/v4l2-device.h>

/*****************************************************************************/


#define CONFIG_SENSOR_IF_MIPI
#define CONFIG_XGOLD_ISP

#define CONFIG_CIF_ISP_AUTO_UPD_CFG_BUG
#if defined(CONFIG_CIF_ISP_AUTO_UPD_CFG_BUG)
#define CIF_NULL_BUFF_SIZE 128
#endif

/* Definitions */

#define INTEL_VIDIOC_SENSOR_MODE_DATA \
	_IOR('v', BASE_VIDIOC_PRIVATE,\
	struct isp_supplemental_sensor_mode_data)

#define CIF_ISP20_NUM_CSI_INPUTS 2

#define cif_iowrite32(d, a)	iowrite32(d, a)
#define cif_ioread32(a)	ioread32(a)
#define cif_iowrite32OR(d, a)	iowrite32((ioread32(a)|d), a)
#define cif_iowrite32AND(d, a)	iowrite32((ioread32(a)&d), a)
#define cif_iowrite32FIELD(d, p, m, a)	\
	iowrite32((d << p) | (ioread32(a) & (~m)), a)

#define XGOLD_V4L2_ISR		(1<<5)
#define XGOLD_V4L2_QUEUE	(1<<4)
#define XGOLD_V4L2_FMT		(1<<3)
#define XGOLD_V4L2_ENTER	(1<<2)
#define XGOLD_V4L2_INFO		(1<<1)
#define XGOLD_V4L2_ERROR	(1<<0)

extern int xgold_v4l2_level;
#define xgold_v4l2_debug(level, fmt, arg...) \
	do { \
		if (XGOLD_V4L2_ERROR&level) \
			pr_err(fmt, ##arg); \
		else if (xgold_v4l2_level&level) \
			pr_debug(fmt, ##arg); \
	} while (0)

#define CIF_MRSZ_W_MAX	3264
#define CIF_MRSZ_H_MAX	2448
#define CIF_MRSZ_W_MIN	32
#define CIF_MRSZ_H_MIN	16

#define CIF_SRSZ_W_MAX	854
#define CIF_SRSZ_H_MAX	480
#define CIF_SRSZ_W_MIN	32
#define CIF_SRSZ_H_MIN	16

#define DRIVER_NAME "cif_isp20"

#define PREVIEW_WIDTH		854
#define PREVIEW_HEIGHT		480

#define MAGIC_XGOLD_MEM 0xCAFEBABE

/* FORMAT */
#define MAX_NB_FORMATS	30

#define XGOLD_MAX_WIDTH	3264
#define XGOLD_MAX_HEIGHT	2448
#define CONTRAST_DEF	0x80
#define BRIGHTNESS_DEF	0x0
#define HUE_DEF	0x0

#define MARVIN_MIPI_PRIORITY 1
#define MARVIN_FRAME_IN_PRIORITY 10
#define MARVIN_FRAME_OUT_PRIORITY 6
#define MARVIN_JPEG_PRIORITY      11

/* MARVIN-5MP */
#define MARVIN_HW_MAX_WIDTH_MP	3264
#define MARVIN_HW_MAX_HEIGHT_MP	2448
#define MARVIN_HW_MAX_WIDTH_SP	640
#define MARVIN_HW_MAX_HEIGHT_SP	480
#define MARVIN_HW_MAX_WIDTH			MARVIN_HW_MAX_WIDTH_MP
#define MARVIN_HW_MAX_HEIGHT		MARVIN_HW_MAX_HEIGHT_MP

#define MARVIN_HW_MIN_WIDTH		32
#define MARVIN_HW_MIN_HEIGHT	16

#define MARVIN_HW_MAX_ERRORS 50

/*
	MIPI CSI2.0
*/
#define CSI2_DT_YUV420_8b	(0x18)
#define CSI2_DT_YUV420_10b	(0x19)
#define CSI2_DT_YUV422_8b	(0x1E)
#define CSI2_DT_YUV422_10b	(0x1F)
#define CSI2_DT_RGB565	(0x22)
#define CSI2_DT_RGB666	(0x23)
#define CSI2_DT_RGB888	(0x24)
#define CSI2_DT_RAW8	(0x2A)
#define CSI2_DT_RAW10	(0x2B)
#define CSI2_DT_RAW12	(0x2C)

#define MARVIN_HW_ISR	(1<<4)
#define MARVIN_HW_DEBUG	(1<<3)
#define MARVIN_HW_ENTER	(1<<2)
#define MARVIN_HW_INFO	(1<<1)
#define MARVIN_HW_ERROR	(1<<0)

#define PREVIEW_MODE_CAP 0
#define PREVIEW_MODE_VID 1

#define		SOFIA_ES1_BU_PM_NATIVE	1
#ifdef SOFIA_ES1_BU_PM_NATIVE
#define OF_KERNEL_CLK			"clk_kernel"
#define OF_SLAVE_CLK			"clk_slave"
#define OF_MASTER_CLK			"clk_master"
#define OF_SENSOR_CLK			"clk_sensor"
#endif

enum cif_isp20_img_src_state {
	CIF_ISP20_IMG_SRC_STATE_OFF = 0,
	CIF_ISP20_IMG_SRC_STATE_SW_STNDBY = 1,
	CIF_ISP20_IMG_SRC_STATE_STREAMING = 2
};

enum cif_isp20_state {
	/* path not yet opened: */
	CIF_ISP20_STATE_DISABLED = 0,
	/* path opened but not yet configured: */
	CIF_ISP20_STATE_INACTIVE = 1,
	/* path opened and configured, ready for streaming: */
	CIF_ISP20_STATE_READY = 2,
	/* path is streaming: */
	CIF_ISP20_STATE_STREAMING = 3
};

enum cif_isp20_pm_state {
	CIF_ISP20_PM_STATE_OFF,
	CIF_ISP20_PM_STATE_SUSPENDED,
	CIF_ISP20_PM_STATE_SW_STNDBY,
	CIF_ISP20_PM_STATE_STREAMING
};

enum cif_isp20_inp {
	CIF_ISP20_INP_CSI_0 = 0,
	CIF_ISP20_INP_CSI_1 = 1,
	CIF_ISP20_INP_CPI = 2,
	CIF_ISP20_INP_DMA = 3,
};

enum cif_isp20_irq {
	CIF_ISP20_IRQ_MIPI,
	CIF_ISP20_IRQ_ISP,
	CIF_ISP20_IRQ_MI,
	CIF_ISP20_IRQ_JPE_STATUS,
	CIF_ISP20_IRQ_JPE_ERROR
};

enum cif_isp20_pinctrl_state {
	CIF_ISP20_PINCTRL_STATE_SLEEP,
	CIF_ISP20_PINCTRL_STATE_INACTIVE,
	CIF_ISP20_PINCTRL_STATE_DEFAULT,
	CIF_ISP20_PINCTRL_STATE_ACTIVE
};

enum cif_isp20_flash_mode {
	CIF_ISP20_FLASH_MODE_OFF,
	CIF_ISP20_FLASH_MODE_FLASH,
	CIF_ISP20_FLASH_MODE_TORCH,
};

enum cif_isp20_cid {
	CIF_ISP20_CID_FLASH_MODE = 0,
	CIF_ISP20_CID_EXPOSURE_TIME,
	CIF_ISP20_CID_ANALOG_GAIN,
	CIF_ISP20_CID_WB_TEMPERATURE,
	CIF_ISP20_CID_BLACK_LEVEL,
	CIF_ISP20_CID_AUTO_GAIN,
	CIF_ISP20_CID_AUTO_EXPOSURE,
	CIF_ISP20_CID_AUTO_WHITE_BALANCE,
	CIF_ISP20_CID_FOCUS_ABSOLUTE,
};

enum marvin_version {
	marvin_xg6310 = 12,	/* corresponds to bitfield value !!! */
};

enum marvin_channel_mode {
	marvin_no_mode = 0,	/* corresponds to bitfield value !!! */
	marvin_main_path = 1,	/* corresponds to bitfield value !!! */
	marvin_self_path = 2,	/* corresponds to bitfield value !!! */
	marvin_both_path = 3	/* corresponds to bitfield value !!! */
};

struct marvin_overlay_pos {
	unsigned int top;
	unsigned int left;
};

struct marvin_fb {
	unsigned int addr;
	unsigned int width;
	unsigned int height;
};

enum marvin_control_id {
	marvin_hflip,		/* SelfPicture SubModule */
	marvin_vflip,		/* SelfPicture SubModule */
	marvin_rotate,		/* SelfPicture SubModule */
	marvin_sepia,		/* ImageEffects SubModule */
	marvin_black_and_white,	/* ImageEffects SubModule */
	marvin_negative,	/* ImageEffects SubModule */
	marvin_color_selection,	/* ImageEffects SubModule */
	marvin_emboss,		/* ImageEffects SubModule */
	marvin_sketch,		/* ImageEffects SubModule */
	marvin_jpeg_quality,
	marvin_none_ie		/*ImageEffects SubModule */
};

struct marvin_control {
	enum marvin_control_id id;
	signed int val;		/* Note signedness */
};


#define CIF_ISP20_PIX_FMT_MASK					0xf0000000
#define CIF_ISP20_PIX_FMT_MASK_BPP				0x0003f000

#define CIF_ISP20_PIX_FMT_YUV_MASK_CPLANES	0x00000003
#define CIF_ISP20_PIX_FMT_YUV_MASK_UVSWAP		0x00000004
#define CIF_ISP20_PIX_FMT_YUV_MASK_YCSWAP		0x00000008
#define CIF_ISP20_PIX_FMT_YUV_MASK_X			0x00000f00
#define CIF_ISP20_PIX_FMT_YUV_MASK_Y			0x000000f0

#define CIF_ISP20_PIX_FMT_RGB_MASK_PAT			0x000000f0

#define CIF_ISP20_PIX_FMT_BAYER_MASK_PAT		0x000000f0

#define CIF_ISP20_PIX_FMT_GET_BPP(pix_fmt) \
	((pix_fmt & CIF_ISP20_PIX_FMT_MASK_BPP) >> 12)
#define cif_isp20_pix_fmt_set_bpp(pix_fmt, bpp) \
	{\
		pix_fmt = ((pix_fmt & ~CIF_ISP20_PIX_FMT_MASK_BPP) |\
			((bpp << 12) & CIF_ISP20_PIX_FMT_MASK_BPP));\
	}

#define CIF_ISP20_PIX_FMT_YUV_GET_NUM_CPLANES(pix_fmt) \
	(pix_fmt & CIF_ISP20_PIX_FMT_YUV_MASK_CPLANES)
#define CIF_ISP20_PIX_FMT_YUV_IS_YC_SWAPPED(pix_fmt) \
	(pix_fmt & CIF_ISP20_PIX_FMT_YUV_MASK_YCSWAP)
#define CIF_ISP20_PIX_FMT_YUV_IS_UV_SWAPPED(pix_fmt) \
		(pix_fmt & CIF_ISP20_PIX_FMT_YUV_MASK_UVSWAP)
#define CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(pix_fmt) \
	((pix_fmt & CIF_ISP20_PIX_FMT_YUV_MASK_X) >> 8)
#define CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(pix_fmt) \
	((pix_fmt & CIF_ISP20_PIX_FMT_YUV_MASK_Y) >> 4)
#define cif_isp20_pix_fmt_set_y_subs(pix_fmt, y_subs) \
	{\
		pix_fmt = ((pix_fmt & ~CIF_ISP20_PIX_FMT_YUV_MASK_Y) |\
			((y_subs << 4) & CIF_ISP20_PIX_FMT_YUV_MASK_Y));\
	}

#define CIF_ISP20_PIX_FMT_BAYER_PAT_IS_BGGR(pix_fmt) \
	((pix_fmt & CIF_ISP20_PIX_FMT_BAYER_MASK_PAT) == 0x0)
#define CIF_ISP20_PIX_FMT_BAYER_PAT_IS_GBRG(pix_fmt) \
	((pix_fmt & CIF_ISP20_PIX_FMT_BAYER_MASK_PAT) == 0x10)
#define CIF_ISP20_PIX_FMT_BAYER_PAT_IS_GRBG(pix_fmt) \
	((pix_fmt & CIF_ISP20_PIX_FMT_BAYER_MASK_PAT) == 0x20)
#define CIF_ISP20_PIX_FMT_BAYER_PAT_IS_RGGB(pix_fmt) \
	((pix_fmt & CIF_ISP20_PIX_FMT_BAYER_MASK_PAT) == 0x30)

#define CIF_ISP20_PIX_FMT_IS_YUV(pix_fmt) \
	((pix_fmt & CIF_ISP20_PIX_FMT_MASK) == 0x10000000)
#define CIF_ISP20_PIX_FMT_IS_RGB(pix_fmt) \
	((pix_fmt & CIF_ISP20_PIX_FMT_MASK) == 0x20000000)
#define CIF_ISP20_PIX_FMT_IS_RAW_BAYER(pix_fmt) \
	((pix_fmt & CIF_ISP20_PIX_FMT_MASK) == 0x30000000)
#define CIF_ISP20_PIX_FMT_IS_JPEG(pix_fmt) \
	((pix_fmt & CIF_ISP20_PIX_FMT_MASK) == 0x40000000)


enum cif_isp20_pix_fmt {
	/* YUV */
	CIF_YUV400				= 0x10008000,

	CIF_YUV420I				= 0x1000c200,
	CIF_YUV420SP			= 0x1000c201,	/* NV12 */
	CIF_YUV420P				= 0x1000c202,
	CIF_YVU420I				= 0x1000c204,
	CIF_YVU420SP			= 0x1000c205,	/* NV21 */
	CIF_YVU420P				= 0x1000c206,	/* YV12 */

	CIF_YUV422I				= 0x10010220,
	CIF_YUV422SP			= 0x10010221,
	CIF_YUV422P				= 0x10010222,
	CIF_YVU422I				= 0x10010224,
	CIF_YVU422SP			= 0x10010225,
	CIF_YVU422P				= 0x10010226,

	CIF_YUV444I				= 0x10018440,
	CIF_YUV444SP			= 0x10018441,
	CIF_YUV444P				= 0x10018442,
	CIF_YVU444I				= 0x10018444,
	CIF_YVU444SP			= 0x10018445,
	CIF_YVU444P				= 0x10018446,

	CIF_UYV400				= 0x10008008,

	CIF_UYV420I				= 0x1000c208,
	CIF_UYV420SP			= 0x1000c209,
	CIF_UYV420P				= 0x1000c20a,
	CIF_VYU420I				= 0x1000c20c,
	CIF_VYU420SP			= 0x1000c20d,
	CIF_VYU420P				= 0x1000c20e,

	CIF_UYV422I				= 0x10010228,
	CIF_UYV422SP			= 0x10010229,
	CIF_UYV422P				= 0x1001022a,
	CIF_VYU422I				= 0x1001022c,
	CIF_VYU422SP			= 0x1001022d,
	CIF_VYU422P				= 0x1001022e,

	CIF_UYV444I				= 0x10018448,
	CIF_UYV444SP			= 0x10018449,
	CIF_UYV444P				= 0x1001844a,
	CIF_VYU444I				= 0x1001844c,
	CIF_VYU444SP			= 0x1001844d,
	CIF_VYU444P				= 0x1001844e,

	/* RGB */
	CIF_RGB565				= 0x20010000,
	CIF_RGB666				= 0x20012000,
	CIF_RGB888				= 0x20018000,

	/* RAW Bayer */
	CIF_BAYER_SBGGR8		= 0x30008000,
	CIF_BAYER_SGBRG8		= 0x30008010,
	CIF_BAYER_SGRBG8		= 0x30008020,
	CIF_BAYER_SRGGB8		= 0x30008030,

	CIF_BAYER_SBGGR10		= 0x3000a000,
	CIF_BAYER_SGBRG10		= 0x3000a010,
	CIF_BAYER_SGRBG10		= 0x3000a020,
	CIF_BAYER_SRGGB10		= 0x3000a030,

	CIF_BAYER_SBGGR12		= 0x3000c000,
	CIF_BAYER_SGBRG12		= 0x3000c010,
	CIF_BAYER_SGRBG12		= 0x3000c020,
	CIF_BAYER_SRGGB12		= 0x3000c030,

	/* JPEG */
	CIF_JPEG					= 0x40008000,

	/* Data */
	CIF_DATA					= 0x70000000,

	CIF_UNKNOWN_FORMAT	= 0x80000000
};

enum cif_isp20_stream_id {
	CIF_ISP20_STREAM_SP	= 0x1,
	CIF_ISP20_STREAM_MP	= 0x2,
	CIF_ISP20_STREAM_DMA	= 0x4,
	CIF_ISP20_STREAM_ISP	= 0x8
};

struct cif_isp20_csi_config {
	u32 vc;
	u32 nb_lanes;
	u32 dphy1;
	u32 dphy2;
	u32 ana_bandgab_bias;
};

struct cif_isp20_frm_intrvl {
	u32 numerator;
	u32 denominator;
};

struct cif_isp20_frm_fmt {
	u32 width;
	u32 height;
	enum cif_isp20_pix_fmt pix_fmt;
};

struct cif_isp20_strm_fmt {
	struct cif_isp20_frm_fmt frm_fmt;
	struct cif_isp20_frm_intrvl frm_intrvl;
};

struct cif_isp20_strm_fmt_desc {
	bool discrete_frmsize;
	struct {
		u32 width;
		u32 height;
	} min_frmsize;
	struct {
		u32 width;
		u32 height;
	} max_frmsize;
	enum cif_isp20_pix_fmt pix_fmt;
	bool discrete_intrvl;
	struct cif_isp20_frm_intrvl min_intrvl;
	struct cif_isp20_frm_intrvl max_intrvl;
};

struct cif_isp20_rsz_config {
	struct cif_isp20_frm_fmt *input;
	struct cif_isp20_frm_fmt output;
};

struct cif_isp20_sp_config {
	struct cif_isp20_rsz_config rsz_config;
	bool updt_cfg;
};

struct cif_isp20_mp_config {
	struct cif_isp20_rsz_config rsz_config;
	bool updt_cfg;
};

#ifdef NO_YET
struct cif_isp20_buffer {
	struct list_head list;
	CIF_ISP20_PLTFRM_MEM_IO_ADDR dma_addr;
	u32 size;
};
#else
#define cif_isp20_buffer videobuf_buffer
#endif

struct cif_isp20_stream {
	enum cif_isp20_state state;
	enum cif_isp20_state saved_state;
	struct list_head buf_queue;
	struct videobuf_buffer *curr_buf;
	struct videobuf_buffer *next_buf;
	bool stall;
};

enum marvin_w_format {
	marvin_planar = 0,	/* corresponds to bitfield value !!! */
	marvin_semiplanar = 1,	/* corresponds to bitfield value !!! */
	marvin_interleaved = 2,	/* corresponds to bitfield value !!! */
};

struct marvin_picture {
	unsigned int inputwidth;
	unsigned int inputheight;
	unsigned int outputwidth;
	unsigned int outputheight;
	enum cif_isp20_pix_fmt inputformat;
	enum cif_isp20_pix_fmt outputformat;
	enum marvin_w_format writeformat;
	bool rotation;
};

struct tmarvin_qtable {
	const unsigned char *y;
	const unsigned char *uv;
};

enum marvin_jpeg_format {
	CIF_JPEG_YUV422,
	CIF_JPEG_YUV400
};

enum marvin_jpeg_header {
	CIF_JPE_JFIF,
	CIF_JPE_NOAPPN
};

struct marvin_jpeg {
	bool enable;
	bool busy;
	u32 ratio;
	enum marvin_jpeg_format format;
	enum marvin_jpeg_header header;
	unsigned int size;
};

struct marvin_isp {
	unsigned int in_sel;
	unsigned int vsync;
	unsigned int hsync;
	unsigned int sample_edge;
	unsigned field_sel;
	struct cif_isp20_frm_fmt *input;
	struct cif_isp20_frm_fmt output;
};

enum marvin_dmaport_fmt {
	DMAPORT_YUV400 = 0,	/* corresponds to bitfield value !!! */
	DMAPORT_YUV420 = 1,	/* corresponds to bitfield value !!! */
	DMAPORT_YUV422 = 2,	/* corresponds to bitfield value !!! */
	DMAPORT_YUV444 = 3,	/* corresponds to bitfield value !!! */
};

struct marvin_dmaport {
	bool enable;
	unsigned int y_pic;
	unsigned int width;
	unsigned int llength;
	unsigned int size;
	unsigned int cb_pic;
	unsigned int cr_pic;
	enum marvin_w_format w_format;
	enum marvin_dmaport_fmt format;
};

struct marvin_resolution_path {
	unsigned int input_width;
	unsigned int input_height;
	unsigned int output_width;
	unsigned int output_height;
};

struct marvin_mi_selfpicture {
	bool enable;
	bool hflip;		/* Horizontal Flipping */
	bool vflip;		/* Vertical Flipping */
	bool rotate;		/* Rotate 90 deg */
	unsigned int width;
	unsigned int height;
	unsigned int size;
};

struct marvin_mi_path {
	struct cif_isp20_frm_fmt *input;
	struct cif_isp20_frm_fmt output;
	u32 llength;
	unsigned int y_base;
	unsigned int cb_offs;
	unsigned int cr_offs;
	unsigned int y_size;
	unsigned int cb_size;
	unsigned int cr_size;
	struct marvin_mi_selfpicture selfpicture;
	bool configured;
};

struct marvin_mi {
	bool raw_enable;
	struct marvin_mi_path mp;	/* Memory Interface main path */
	struct marvin_mi_path sp;	/* Memory Interface self path */
#if defined(CONFIG_CIF_ISP_AUTO_UPD_CFG_BUG)
	void *null_buff;
#endif
	dma_addr_t null_buff_dma_addr;
};

struct marvin_mipi {
	u32 input_sel;
	struct cif_isp20_csi_config csi_config;
};

enum marvin_si_mode {
	si_bypass,
	si_overlay,
	si_color_keying
};

struct marvin_si {
	enum marvin_si_mode mode;
	unsigned int x;
	unsigned int y;
	unsigned int y_comp;
	unsigned int cb_comp;
	unsigned int cr_comp;
};

struct marvin_ei {
	unsigned int image_effect;
};

struct marvin_cbh {
	unsigned int contrast;
	signed int brightness;
	signed int hue;
};

struct marvinconfig {
	enum marvin_version marvin_version;
	CIF_ISP20_PLTFRM_MEM_IO_ADDR base_addr;	/* registers base address */
	unsigned int id;
	enum cif_isp20_flash_mode flash_mode;
	enum cif_isp20_inp input_sel;
	struct marvin_control control;	/* control */
	struct marvin_jpeg jpeg_config;	/* configuration of the JPEG encoder */
	struct marvin_isp isp_config;	/* isp configuration */
	struct marvin_dmaport dmaport_config;	/* DMA Port configuration */
	struct marvin_overlay_pos overlay_pos;
	struct marvin_fb framebuffer;
	/* Input picture resolution & Output picture resolution */
	struct marvin_mi mi_config;	/* data storage configuration */
	struct marvin_mipi mipi_config;	/* mipi configuration */
	struct marvin_si si_config;	/* superimopse configuration */
	struct marvin_ei ei_config;	/* image_effect configuration */
	struct marvin_cbh cbh_config;	/* Contrast, Brightnes and Hue */
	struct cif_isp20_strm_fmt img_src_output;
	struct cif_isp20_sp_config sp_config;
	struct cif_isp20_mp_config mp_config;
};

struct cif_isp20_mi_state {
	unsigned long flags;
	unsigned int isp_ctrl;
	unsigned int y_base_ad;
	unsigned int y_size;
	unsigned int y_offs_cnt;
	unsigned int cb_base_ad;
	unsigned int cb_size;
	unsigned int cb_offs_cnt;
	unsigned int cr_base_ad;
	unsigned int cr_size;
	unsigned int cr_offs_cnt;
};

/* Sensor resolution specific data for AE calculation.*/
struct isp_supplemental_sensor_mode_data {
	unsigned int coarse_integration_time_min;
	unsigned int coarse_integration_time_max_margin;
	unsigned int fine_integration_time_min;
	unsigned int fine_integration_time_max_margin;
	unsigned int frame_length_lines;
	unsigned int line_length_pck;
	unsigned int vt_pix_clk_freq_hz;
	unsigned int crop_horizontal_start; /* Sensor crop start cord. (x0,y0)*/
	unsigned int crop_vertical_start;
	unsigned int crop_horizontal_end; /* Sensor crop end cord. (x1,y1)*/
	unsigned int crop_vertical_end;
	unsigned int sensor_output_width; /* input size to ISP */
	unsigned int sensor_output_height;
	unsigned int isp_input_horizontal_start;
	unsigned int isp_input_vertical_start;
	unsigned int isp_input_width;
	unsigned int isp_input_height;
	unsigned char binning_factor_x; /* horizontal binning factor used */
	unsigned char binning_factor_y; /* vertical binning factor used */
	unsigned int sensor_type; /* 0 without ISP, 1 with ISP*/
};

/* ======================================================================== */

struct xgold_fmt {
	char *name;
	u32 fourcc;
	int flags;
	int depth;
	unsigned char rotation;
	unsigned char overlay;
};

/* ======================================================================== */

/* marvin_hw operators prototypes */
struct marvin_hw_ops {
	int (*open)(struct marvinconfig *);
	int (*close)(struct marvinconfig *);
	int (*control)(struct marvinconfig *);
};

/* ======================================================================== */

struct xgold_hardware {
	/* marvin lib configuration */
	struct marvinconfig marvin_config;
	/* current format */

	/* marvin operators */
	struct marvin_hw_ops hw_ops;

	/* DMA Address */
	unsigned int mp_dma_add;
	unsigned int sp_dma_add;
};

/* =============================================== */

struct cif_isp20_device {
	CIF_ISP20_PLTFRM_DEVICE dev;
	struct v4l2_device v4l2_dev;
	int mp_minor;
	int mp_major;
	int sp_major;
	int sp_minor;
	enum cif_isp20_pm_state pm_state;
	enum cif_isp20_img_src_state img_src_state;

	spinlock_t vbq_lock;	/* spinlock for videobuf queues */
	spinlock_t img_lock;	/* spinlock for image parameters */
	struct semaphore sem;

	struct cif_isp20_img_src *img_src;
	struct cif_isp20_img_src *img_src_array[CIF_ISP20_NUM_CSI_INPUTS];
	/* xgold_hw related structure */
	struct xgold_hardware xgold_hw;
	/* ISP device */
	struct xgold_isp_dev isp_dev;
	struct xgold_readback_path_dev rb_dev;
	struct cif_isp20_stream sp_stream;
	struct cif_isp20_stream mp_stream;
	struct cif_isp20_stream dma_stream;
#ifdef SOFIA_ES1_BU_PM_NATIVE
	struct clk *clk_kernel;
	struct clk *clk_slave;
	struct clk *clk_master;
	struct clk *clk_sensor;
	struct regulator *regulator_lmipi_ana;
	struct regulator *regulator_lmipi_bg;
	struct regulator *regulator_laux;
	struct regulator *regulator_laux1;
	struct regulator *regulator_lcif;
	struct regulator *regulator_lcsi1;
	struct regulator *regulator_lcsi2;
#endif
};

int marvin_lib_mi_sp(struct cif_isp20_device *dev);
unsigned int marvin_lib_start(struct cif_isp20_device *dev);
unsigned int marvin_lib_mi_address(struct marvinconfig *marvin_config,
				   int write_sp);
unsigned int marvin_lib_disable_dma_read_int(
	struct marvinconfig *marvin_config);
unsigned int marvin_lib_clear_frame_in_error(
	struct marvinconfig *marvin_config);
unsigned int marvin_lib_s_control(struct marvinconfig *marvin_config);
unsigned int marvin_lib_g_control(struct marvinconfig *marvin_config);
int marvin_lib_jpeg_config(struct marvinconfig *marvin_config);
int marvin_lib_program_jpeg_tables(
	struct marvinconfig *marvin_config);
int marvin_lib_select_jpeg_tables(struct marvinconfig *marvin_config);
unsigned int marvin_lib_dmaport(struct marvinconfig *marvin_config);
unsigned int marvin_lib_si_config(struct marvinconfig *marvin_config);
unsigned int marvin_lib_isp_shutter(struct marvinconfig *marvin_config);
unsigned int marvin_lib_isp_flash(struct marvinconfig *marvin_config);
int marvin_s_ctrl(struct cif_isp20_device *dev,
	struct v4l2_control *vc);

/* Config CIF*/
int config_cif(struct cif_isp20_device *dev);

struct xgold_fmt *get_xgold_output_format(int index);
int get_xgold_output_format_size(void);

struct v4l2_fmtdesc *get_xgold_output_format_desc(int index);
int get_xgold_output_format_desc_size(void);

/*Clean code starts from here*************************************************/

struct cif_isp20_device *cif_isp20_create(void);

int cif_isp20_release(
	struct cif_isp20_device *dev,
	bool release_sp,
	bool release_mp);

void cif_isp20_init_sp(
	struct cif_isp20_device *dev);

void cif_isp20_init_mp(
	struct cif_isp20_device *dev);

int cif_isp20_streamon(
	struct cif_isp20_device *dev,
	bool streamon_sp,
	bool streamon_mp);

int cif_isp20_streamoff(
	struct cif_isp20_device *dev,
	bool streamoff_sp,
	bool streamoff_mp);

int cif_isp20_s_input(
	struct cif_isp20_device *dev,
	enum cif_isp20_inp inp);

int cif_isp20_s_fmt_mp(
	struct cif_isp20_device *dev,
	struct cif_isp20_strm_fmt *strm_fmt,
	u32 stride);

int cif_isp20_s_fmt_sp(
	struct cif_isp20_device *dev,
	struct cif_isp20_strm_fmt *strm_fmt,
	u32 stride);

int cif_isp20_qbuf(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream,
	struct cif_isp20_buffer *buf);

int cif_isp20_calc_isp_cropping(
	struct cif_isp20_device *dev,
	u32 *width,
	u32 *height,
	u32 *h_offs,
	u32 *v_offs);

int cif_isp20_calc_min_out_buff_size(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream,
	u32 *size);

/*TBD: remove these declarations once the respective code has been moved
	to cif_isp20_v4l2.c */
int xgold_v4l2_core_open(
	struct file *file);

extern const struct v4l2_ioctl_ops cif_isp20_sp_ioctlops;
extern const struct v4l2_ioctl_ops cif_isp20_mp_ioctlops;
extern const struct v4l2_file_operations cif_isp20_sp_v4l2_fops;
extern const struct v4l2_file_operations cif_isp20_mp_v4l2_fops;


#endif
