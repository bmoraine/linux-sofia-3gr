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

#include <linux/platform_device.h>
#include "cif_isp20_pltfrm.h"
#include "cif_isp20_img_src.h"
#include "cif_isp20_isp.h"

#include <media/v4l2-device.h>

/*****************************************************************************/

#define CONFIG_CIF_ISP_AUTO_UPD_CFG_BUG
#if defined(CONFIG_CIF_ISP_AUTO_UPD_CFG_BUG)
#endif

/* Definitions */

#define CIF_ISP20_NUM_CSI_INPUTS 2

#define DRIVER_NAME "cif_isp20"

/* FORMAT */
#define MAX_NB_FORMATS	30

#define CONTRAST_DEF	0x80
#define BRIGHTNESS_DEF	0x0
#define HUE_DEF	0x0

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
	CIF_ISP20_INP_CSI_1,
	CIF_ISP20_INP_CPI,
	CIF_ISP20_INP_DMA, /* DMA -> ISP */
	CIF_ISP20_INP_DMA_IE, /* DMA -> IE */
	CIF_ISP20_INP_DMA_SP /* DMA -> SP */
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
	CIF_ISP20_CID_EXPOSURE_TIME = 1,
	CIF_ISP20_CID_ANALOG_GAIN = 2,
	CIF_ISP20_CID_WB_TEMPERATURE = 3,
	CIF_ISP20_CID_BLACK_LEVEL = 4,
	CIF_ISP20_CID_AUTO_GAIN = 5,
	CIF_ISP20_CID_AUTO_EXPOSURE = 6,
	CIF_ISP20_CID_AUTO_WHITE_BALANCE = 7,
	CIF_ISP20_CID_FOCUS_ABSOLUTE = 8,
	CIF_ISP20_CID_AUTO_N_PRESET_WHITE_BALANCE = 9,
	CIF_ISP20_CID_SCENE_MODE = 10,
	CIF_ISP20_CID_SUPER_IMPOSE = 11,
	CIF_ISP20_CID_JPEG_QUALITY = 12,
	CIF_ISP20_CID_IMAGE_EFFECT = 13,
	CIF_ISP20_CID_HFLIP = 14,
	CIF_ISP20_CID_VFLIP = 15,
	CIF_ISP20_CID_AUTO_FPS = 16,
	CIF_ISP20_CID_VBLANKING = 17
};

/* correspond to bit field values */
enum cif_isp20_image_effect {
	CIF_ISP20_IE_BW = 0,
	CIF_ISP20_IE_NEGATIVE = 1,
	CIF_ISP20_IE_SEPIA = 2,
	CIF_ISP20_IE_C_SEL = 3,
	CIF_ISP20_IE_EMBOSS = 4,
	CIF_ISP20_IE_SKETCH = 5,
	CIF_ISP20_IE_NONE /* not a bit field value */
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
	{ \
		pix_fmt = ((pix_fmt & ~CIF_ISP20_PIX_FMT_MASK_BPP) | \
			((bpp << 12) & CIF_ISP20_PIX_FMT_MASK_BPP)); \
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
	{ \
		pix_fmt = ((pix_fmt & ~CIF_ISP20_PIX_FMT_YUV_MASK_Y) | \
			((y_subs << 4) & CIF_ISP20_PIX_FMT_YUV_MASK_Y)); \
	}
#define cif_isp20_pix_fmt_set_x_subs(pix_fmt, x_subs) \
	{ \
		pix_fmt = ((pix_fmt & ~CIF_ISP20_PIX_FMT_YUV_MASK_X) | \
			((x_subs << 8) & CIF_ISP20_PIX_FMT_YUV_MASK_X)); \
	}
#define cif_isp20_pix_fmt_set_yc_swapped(pix_fmt, yc_swapped) \
	{ \
		pix_fmt = ((pix_fmt & ~CIF_ISP20_PIX_FMT_YUV_MASK_YCSWAP) | \
			((yc_swapped << 3) & \
			CIF_ISP20_PIX_FMT_YUV_MASK_YCSWAP)); \
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

#define CIF_ISP20_PIX_FMT_IS_INTERLEAVED(pix_fmt) \
	(!CIF_ISP20_PIX_FMT_IS_YUV(pix_fmt) ||\
	!CIF_ISP20_PIX_FMT_YUV_GET_NUM_CPLANES(pix_fmt))

enum cif_isp20_pix_fmt {
	/* YUV */
	CIF_YUV400				= 0x10008000,
	CIF_YVU400				= 0x10008004,

	CIF_YUV420I				= 0x1000c220,
	CIF_YUV420SP			= 0x1000c221,	/* NV12 */
	CIF_YUV420P				= 0x1000c222,
	CIF_YVU420I				= 0x1000c224,
	CIF_YVU420SP			= 0x1000c225,	/* NV21 */
	CIF_YVU420P				= 0x1000c226,	/* YV12 */

	CIF_YUV422I				= 0x10010240,
	CIF_YUV422SP			= 0x10010241,
	CIF_YUV422P				= 0x10010242,
	CIF_YVU422I				= 0x10010244,
	CIF_YVU422SP			= 0x10010245,
	CIF_YVU422P				= 0x10010246,

	CIF_YUV444I				= 0x10018440,
	CIF_YUV444SP			= 0x10018441,
	CIF_YUV444P				= 0x10018442,
	CIF_YVU444I				= 0x10018444,
	CIF_YVU444SP			= 0x10018445,
	CIF_YVU444P				= 0x10018446,

	CIF_UYV400				= 0x10008008,

	CIF_UYV420I				= 0x1000c228,
	CIF_UYV420SP			= 0x1000c229,
	CIF_UYV420P				= 0x1000c22a,
	CIF_VYU420I				= 0x1000c22c,
	CIF_VYU420SP			= 0x1000c22d,
	CIF_VYU420P				= 0x1000c22e,

	CIF_UYV422I				= 0x10010248,
	CIF_UYV422SP			= 0x10010249,
	CIF_UYV422P				= 0x1001024a,
	CIF_VYU422I				= 0x1001024c,
	CIF_VYU422SP			= 0x1001024d,
	CIF_VYU422P				= 0x1001024e,

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

#define CIF_ISP20_ALL_STREAMS \
	(CIF_ISP20_STREAM_SP | \
	CIF_ISP20_STREAM_MP | \
	CIF_ISP20_STREAM_DMA)

enum cif_isp20_buff_fmt {
	/* values correspond to bitfield values */
	CIF_ISP20_BUFF_FMT_PLANAR = 0,
	CIF_ISP20_BUFF_FMT_SEMIPLANAR = 1,
	CIF_ISP20_BUFF_FMT_INTERLEAVED = 2
};

enum cif_isp20_jpeg_header {
	CIF_ISP20_JPEG_HEADER_JFIF,
	CIF_ISP20_JPEG_HEADER_NONE
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
	bool ycflt_adjust;
};

struct cif_isp20_sp_config {
	struct cif_isp20_rsz_config rsz_config;
	bool inp_yc_filt;
};

struct cif_isp20_mp_config {
	struct cif_isp20_rsz_config rsz_config;
};

struct cif_isp20_mi_path_config {
	struct cif_isp20_frm_fmt *input;
	struct cif_isp20_frm_fmt output;
	u32 llength;
	u32 curr_buff_addr;
	u32 next_buff_addr;
	u32 cb_offs;
	u32 cr_offs;
	u32 y_size;
	u32 cb_size;
	u32 cr_size;
	bool busy;
};

struct cif_isp20_mi_config {
	bool raw_enable;
	bool async_updt;
	struct cif_isp20_mi_path_config mp;
	struct cif_isp20_mi_path_config sp;
	struct cif_isp20_mi_path_config dma;
};

struct cif_isp20_mipi_config {
	u32 input_sel;
	struct cif_isp20_csi_config csi_config;
};

#ifdef NO_YET
struct cif_isp20_buffer {
	struct list_head list;
	u32 dma_addr;
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
	bool updt_cfg;
	bool stall;
	bool first_frame;
};

struct cif_isp20_jpeg_config {
	bool enable;
	bool busy;
	u32 ratio;
	struct cif_isp20_frm_fmt *input;
	enum cif_isp20_jpeg_header header;
};

struct cif_isp20_ie_config {
	enum cif_isp20_image_effect effect;
};

struct cif_isp20_isp_config {
	bool si_enable;
	struct cif_isp20_ie_config ie_config;
	struct cif_isp20_frm_fmt *input;
	struct cif_isp20_frm_fmt output;
};

struct cif_isp20_config {
	CIF_ISP20_PLTFRM_MEM_IO_ADDR base_addr;
	enum cif_isp20_flash_mode flash_mode;
	enum cif_isp20_inp input_sel;
	struct cif_isp20_jpeg_config jpeg_config;
	struct cif_isp20_mi_config mi_config;
	struct cif_isp20_mipi_config mipi_config;
	struct cif_isp20_sp_config sp_config;
	struct cif_isp20_mp_config mp_config;
	struct cif_isp20_strm_fmt img_src_output;
	struct cif_isp20_isp_config isp_config;
};

struct cif_isp20_mi_state {
	unsigned long flags;
	unsigned int isp_ctrl;
	unsigned int y_base_ad;
	unsigned int y_size;
	unsigned int cb_base_ad;
	unsigned int cb_size;
	unsigned int cr_base_ad;
	unsigned int cr_size;
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

/* =============================================== */

struct cif_isp20_device {
	CIF_ISP20_PLTFRM_DEVICE dev;
	struct v4l2_device v4l2_dev;
	enum cif_isp20_pm_state pm_state;
	enum cif_isp20_img_src_state img_src_state;

	spinlock_t vbq_lock;	/* spinlock for videobuf queues */

	struct cif_isp20_img_src *img_src;
	struct cif_isp20_img_src *img_src_array[CIF_ISP20_NUM_CSI_INPUTS];
	struct cif_isp20_config config;
	struct xgold_isp_dev isp_dev;
	struct cif_isp20_stream sp_stream;
	struct cif_isp20_stream mp_stream;
	struct cif_isp20_stream dma_stream;
	bool stop_dma;
	CIF_ISP20_PLTFRM_EVENT dma_done;
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

struct xgold_fmt *get_xgold_output_format(int index);
int get_xgold_output_format_size(void);

struct v4l2_fmtdesc *get_xgold_output_format_desc(int index);
int get_xgold_output_format_desc_size(void);

/*Clean code starts from here*************************************************/

struct cif_isp20_device *cif_isp20_create(
	CIF_ISP20_PLTFRM_DEVICE pdev);

void cif_isp20_destroy(
	struct cif_isp20_device *dev);

int cif_isp20_init(
	struct cif_isp20_device *dev,
	u32 stream_ids);

int cif_isp20_release(
	struct cif_isp20_device *dev,
	int stream_ids);

int cif_isp20_streamon(
	struct cif_isp20_device *dev,
	u32 stream_ids);

int cif_isp20_streamoff(
	struct cif_isp20_device *dev,
	u32 stream_ids);

int cif_isp20_s_input(
	struct cif_isp20_device *dev,
	enum cif_isp20_inp inp);

int cif_isp20_s_fmt(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream_id,
	struct cif_isp20_strm_fmt *strm_fmt,
	u32 stride);

int cif_isp20_resume(
	struct cif_isp20_device *dev);

int cif_isp20_suspend(
	struct cif_isp20_device *dev);

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

const char *cif_isp20_g_input_name(
	struct cif_isp20_device *dev,
	enum cif_isp20_inp inp);

int cif_isp20_calc_min_out_buff_size(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream_id,
	u32 *size);

int cif_isp20_s_ctrl(
	struct cif_isp20_device *dev,
	const enum cif_isp20_cid id,
	int val);

#endif
