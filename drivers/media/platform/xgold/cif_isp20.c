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

#include <linux/videodev2.h>
#include <media/videobuf-dma-contig.h>
#include "cif_isp20_regs.h"
#include "cif_isp20.h"

static int marvin_mipi_isr(
	void *cntxt);
static int marvin_isp_isr(
	void *cntxt);
static void init_output_formats(void);

struct v4l2_fmtdesc output_formats[MAX_NB_FORMATS];

/*
	JPEG quantization tables for JPEG encoding
*/
/* DC luma table according to ISO/IEC 10918-1 annex K */
static const unsigned char dc_luma_table_annex_k[] = {
	0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b
};

/* DC chroma table according to ISO/IEC 10918-1 annex K */
static const unsigned char dc_chroma_table_annex_k[] = {
	0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b
};

/* AC luma table according to ISO/IEC 10918-1 annex K */
static const unsigned char ac_luma_table_annex_k[] = {
	0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03,
	0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d,
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
	0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
	0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
	0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
	0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa
};

/* AC Chroma table according to ISO/IEC 10918-1 annex K */
static const unsigned char ac_chroma_table_annex_k[] = {
	0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04,
	0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77,
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
	0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
	0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
	0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
	0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
	0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
	0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
	0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
	0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
	0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
	0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
	0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
	0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
	0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
	0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
	0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa
};

/* Standard JPEG quantization tables */
/* luma */
/* According to JPEG spec:
[
	16, 11, 10, 16,  24,  40,  51,  61,
	12, 12, 14, 19,  26,  58,  60,  55,
	14, 13, 16, 24,  40,  57,  69,  56,
	14, 17, 22, 29,  51,  87,  80,  62,
	18, 22, 37, 56,  68, 109, 103,  77,
	24, 35, 55, 64,  81, 104, 113,  92,
	49, 64, 78, 87, 103, 121, 120, 101,
	72, 92, 95, 98, 112, 100, 103,  99
]
*/

/* CIF needs it in zigzag order */
static const unsigned char yq_table_base_zigzag[] = {
	16, 11, 12, 14, 12, 10, 16, 14,
	13, 14, 18, 17, 16, 19, 24, 40,
	26, 24, 22, 22, 24, 49, 35, 37,
	29, 40, 58, 51, 61, 60, 57, 51,
	56, 55, 64, 72, 92, 78, 64, 68,
	87, 69, 55, 56, 80, 109, 81, 87,
	95, 98, 103, 104, 103, 62, 77, 113,
	121, 112, 100, 120, 92, 101, 103, 99
};

/* chroma */
/* According to JPEG spec:
[
	17, 18, 24, 47, 99, 99, 99, 99,
	18, 21, 26, 66, 99, 99, 99, 99,
	24, 26, 56, 99, 99, 99, 99, 99,
	47, 66, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99
]
*/

/* CIF needs it in zigzag order */
static const unsigned char uvq_table_base_zigzag[] = {
	17, 18, 18, 24, 21, 24, 47, 26,
	26, 47, 99, 66, 56, 66, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99
};

static struct xgold_fmt xgold_output_format[] = {
/* ************* YUV422 ************* */
{
	.name		= "YUV422-Interleaved",
	.fourcc	= V4L2_PIX_FMT_YUYV,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YUV422-Interleaved",
	.fourcc	= V4L2_PIX_FMT_YUYV,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YVU422-Interleaved",
	.fourcc	= V4L2_PIX_FMT_UYVY,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YUV422-Planar",
	.fourcc	= V4L2_PIX_FMT_YUV422P,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YUV422-Semi-Planar",
	.fourcc	= V4L2_PIX_FMT_NV16,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
/* ************* YUV420 ************* */
{
	.name		= "YUV420-Planar",
	.fourcc	= V4L2_PIX_FMT_YUV420,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YUV420-Planar",
	.fourcc	= V4L2_PIX_FMT_YUV420,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YVU420-Planar",
	.fourcc	= V4L2_PIX_FMT_YVU420,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YUV420-Semi-Planar",
	.fourcc	= V4L2_PIX_FMT_NV12,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YVU420-Semi-Planar",
	.fourcc	= V4L2_PIX_FMT_NV21,
	.flags	= 0,
	.depth	= 12,
	.rotation = false,
	.overlay = false,
},
/* ************* YUV400 ************* */
{
	.name		= "YVU400-Grey-Planar",
	.fourcc	= V4L2_PIX_FMT_GREY,
	.flags	= 0,
	.depth	= 8,
	.rotation = false,
	.overlay = false,
},
/* ************* YUV444 ************* */
{
	.name		= "YVU444-Planar",
	.fourcc	= V4L2_PIX_FMT_YUV444,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
{
	.name		= "YVU444-Semi-Planar",
	.fourcc	= V4L2_PIX_FMT_NV24,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
/* ************* JPEG ************* */
{
	.name		= "JPEG",
	.fourcc	= V4L2_PIX_FMT_JPEG,
	.flags	= 0,
	.depth	= 16,
	.rotation = false,
	.overlay = false,
},
/* ************ RGB565 *********** */
{
	.name       = "RGB565",
	.fourcc = V4L2_PIX_FMT_RGB565,
	.flags  = 0,
	.depth  = 16,
	.rotation = false,
	.overlay = false,
}
};

struct smarvin_hw_errors {
	char *name;
	unsigned int count;
	unsigned int mask;
	unsigned int type;	/*isp:0 ;mipi:1*/
};

static struct smarvin_hw_errors marvin_hw_errors[] = {
	{
	 .name = "isp_data_loss",
	 .count = 0,
	 .mask = (1 << 2),
	 .type = 0,
	 },
	{
	 .name = "isp_pic_size_err",
	 .count = 0,
	 .mask = (1 << 3),
	 .type = 0,
	 },
	{
	 .name = "mipi_fifo_err",
	 .count = 0,
	 .mask = (1 << 0),
	 .type = 1,
	 },
	{
	 .name = "dphy_err_sot",
	 .count = 0,
	 .mask = (3 << 4),
	 .type = 1,
	 },
	{
	 .name = "dphy_err_sot_sync",
	 .count = 0,
	 .mask = (3 << 8),
	 .type = 1,
	 },
	{
	 .name = "dphy_err_eot_sync",
	 .count = 0,
	 .mask = (3 << 12),
	 .type = 1,
	 },
	{
	 .name = "dphy_err_ctrl",
	 .count = 0,
	 .mask = (3 << 16),
	 .type = 1,
	 },
	{
	 .name = "csi_err_protocol",
	 .count = 0,
	 .mask = (1 << 20),
	 .type = 2,
	 },
	{
	 .name = "csi_err_ecc1",
	 .count = 0,
	 .mask = (1 << 21),
	 .type = 2,
	 },
	{
	 .name = "csi_err_ecc2",
	 .count = 0,
	 .mask = (1 << 22),
	 .type = 2,
	 },
	{
	 .name = "csi_err_cs",
	 .count = 0,
	 .mask = (1 << 23),
	 .type = 2,
	 },
	{
	 .name = "fifo_ovf",
	 .count = 0,
	 .mask = (3 << 0),
	 .type = 2,
	 },
	{
	 .name = "isp_outform",
	 .count = 0,
	 .mask = (1 << 2),
	 .type = 0,
	 },
	{
	 .name = "isp_stab",
	 .count = 0,
	 .mask = (1 << 1),
	 .type = 0,
	 },
	{
	 .name = "isp_inform",
	 .count = 0,
	 .mask = (1 << 0),
	 .type = 0,
	 }
};

static const struct {
	u32 input_width;
	u32 output_width;
} accetable_crop_widths[] = {
	{1280, 1220},
	{640, 592},
};


/**Defines********************************************************************/

#define CIF_ISP20_INVALID_BUFF_ADDR ((u32)~0)
#define CIF_ISP20_SP_YCFLT_INP (true)
#define CIF_ISP20_MI_IS_BUSY(dev)\
	(dev->config.mi_config.mp.busy ||\
	dev->config.mi_config.sp.busy ||\
	dev->config.mi_config.dma.busy)
enum {
	CIF_ISP20_ASYNC_JPEG = 0x1,
	CIF_ISP20_ASYNC_YCFLT = 0x2,
	CIF_ISP20_ASYNC_ISM = 0x4,
	CIF_ISP20_ASYNC_DMA = 0x8
};
#define CIF_ISP20_ALWAYS_ASYNC 0x80
#define CIF_ISP20_ALWAYS_STALL_ON_NO_BUFS (false)

#ifndef DIV_ROUND_UP
#define DIV_ROUND_UP(x, y) (((x) + (y) - 1) / (y))
#endif

#ifndef DIV_TRUNCATE
#define DIV_TRUNCATE(x, y) ((x) / (y))
#endif

/**Structures and Types*******************************************************/


/**Static Functions***********************************************************/

static const char *cif_isp20_img_src_state_string(
	enum cif_isp20_img_src_state state)
{
	switch (state) {
	case CIF_ISP20_IMG_SRC_STATE_OFF:
		return "OFF";
	case CIF_ISP20_IMG_SRC_STATE_SW_STNDBY:
		return "SW_STNDBY";
	case CIF_ISP20_IMG_SRC_STATE_STREAMING:
		return "STREAMING";
	default:
		return "UNKNOWN/UNSUPPORTED";
	}
}

static const char *cif_isp20_state_string(
	enum cif_isp20_state state)
{
	switch (state) {
	case CIF_ISP20_STATE_DISABLED:
		return "DISABLED";
	case CIF_ISP20_STATE_INACTIVE:
		return "INACTIVE";
	case CIF_ISP20_STATE_READY:
		return "READY";
	case CIF_ISP20_STATE_STREAMING:
		return "STREAMING";
	default:
		return "UNKNOWN/UNSUPPORTED";
	}
}

static const char *cif_isp20_pm_state_string(
	enum cif_isp20_pm_state pm_state)
{
	switch (pm_state) {
	case CIF_ISP20_PM_STATE_OFF:
		return "OFF";
	case CIF_ISP20_PM_STATE_SW_STNDBY:
		return "STANDBY";
	case CIF_ISP20_PM_STATE_SUSPENDED:
		return "SUSPENDED";
	case CIF_ISP20_PM_STATE_STREAMING:
		return "STREAMING";
	default:
		return "UNKNOWN/UNSUPPORTED";
	}
}

static const char *cif_isp20_stream_id_string(
	enum cif_isp20_stream_id stream_id)
{
	switch (stream_id) {
	case CIF_ISP20_STREAM_SP:
		return "SP";
	case CIF_ISP20_STREAM_MP:
		return "MP";
	case CIF_ISP20_STREAM_DMA:
		return "DMA";
	case CIF_ISP20_STREAM_ISP:
		return "ISP";
	default:
		return "UNKNOWN/UNSUPPORTED";
	}
}

static const char *cif_isp20_inp_string(
	enum cif_isp20_inp inp)
{
	switch (inp) {
	case CIF_ISP20_INP_CSI_0:
		return "CSI-0";
	case CIF_ISP20_INP_CSI_1:
		return "CSI-1";
	case CIF_ISP20_INP_CPI:
		return "CPI";
	case CIF_ISP20_INP_DMA:
		return "DMA";
	case CIF_ISP20_INP_DMA_IE:
		return "DMA(Image Effects)";
	case CIF_ISP20_INP_DMA_SP:
		return "DMA(SP)";
	default:
		return "UNKNOWN/UNSUPPORTED";
	}
}

static const char *cif_isp20_pix_fmt_string(int pixfmt)
{
	switch (pixfmt) {
	case CIF_YUV400:
		return "YUV400";
	case CIF_YUV420I:
		return "YUV420I";
	case CIF_YUV420SP:
		return "YUV420SP";
	case CIF_YUV420P:
		return "YUV420P";
	case CIF_YVU420I:
		return "YVU420I";
	case CIF_YVU420SP:
		return "YVU420SP";
	case CIF_YVU420P:
		return "YVU420P";
	case CIF_YUV422I:
		return "YUV422I";
	case CIF_YUV422SP:
		return "YUV422SP";
	case CIF_YUV422P:
		return "YUV422P";
	case CIF_YVU422I:
		return "YVU422I";
	case CIF_YVU422SP:
		return "YVU422SP";
	case CIF_YVU422P:
		return "YVU422P";
	case CIF_YUV444I:
		return "YUV444I";
	case CIF_YUV444SP:
		return "YUV444SP";
	case CIF_YUV444P:
		return "YUV444P";
	case CIF_YVU444I:
		return "YVU444I";
	case CIF_YVU444SP:
		return "YVU444SP";
	case CIF_YVU444P:
		return "YVU444SP";
	case CIF_UYV400:
		return "UYV400";
	case CIF_UYV420I:
		return "UYV420I";
	case CIF_UYV420SP:
		return "UYV420SP";
	case CIF_UYV420P:
		return "UYV420P";
	case CIF_VYU420I:
		return "VYU420I";
	case CIF_VYU420SP:
		return "VYU420SP";
	case CIF_VYU420P:
		return "VYU420P";
	case CIF_UYV422I:
		return "UYV422I";
	case CIF_UYV422SP:
		return "UYV422I";
	case CIF_UYV422P:
		return "UYV422P";
	case CIF_VYU422I:
		return "VYU422I";
	case CIF_VYU422SP:
		return "VYU422SP";
	case CIF_VYU422P:
		return "VYU422P";
	case CIF_UYV444I:
		return "UYV444I";
	case CIF_UYV444SP:
		return "UYV444SP";
	case CIF_UYV444P:
		return "UYV444P";
	case CIF_VYU444I:
		return "VYU444I";
	case CIF_VYU444SP:
		return "VYU444SP";
	case CIF_VYU444P:
		return "VYU444P";
	case CIF_RGB565:
		return "RGB565";
	case CIF_RGB666:
		return "RGB666";
	case CIF_RGB888:
		return "RGB888";
	case CIF_BAYER_SBGGR8:
		return "BAYER BGGR8";
	case CIF_BAYER_SGBRG8:
		return "BAYER GBRG8";
	case CIF_BAYER_SGRBG8:
		return "BAYER GRBG8";
	case CIF_BAYER_SRGGB8:
		return "BAYER RGGB8";
	case CIF_BAYER_SBGGR10:
		return "BAYER BGGR10";
	case CIF_BAYER_SGBRG10:
		return "BAYER GBRG10";
	case CIF_BAYER_SGRBG10:
		return "BAYER GRBG10";
	case CIF_BAYER_SRGGB10:
		return "BAYER RGGB10";
	case CIF_BAYER_SBGGR12:
		return "BAYER BGGR12";
	case CIF_BAYER_SGBRG12:
		return "BAYER GBRG12";
	case CIF_BAYER_SGRBG12:
		return "BAYER GRBG12";
	case CIF_BAYER_SRGGB12:
		return "BAYER RGGB12";
	case CIF_DATA:
		return "DATA";
	case CIF_JPEG:
		return "JPEG";
	default:
		return "unknown/unsupported";
	}
}

static void cif_isp20_debug_print_mi_sp(struct cif_isp20_device *dev)
{
	cif_isp20_pltfrm_pr_info(dev->dev,
		"\n  MI_CTRL 0x%08x/0x%08x\n"
		"  MI_STATUS 0x%08x\n"
		"  MI_RIS 0x%08x/0x%08x\n"
		"  MI_IMSC 0x%08x\n"
		"  MI_SP_Y_SIZE %d/%d\n"
		"  MI_SP_CB_SIZE %d/%d\n"
		"  MI_SP_CR_SIZE %d/%d\n"
		"  MI_SP_PIC_WIDTH %d\n"
		"  MI_SP_PIC_HEIGHT %d\n"
		"  MI_SP_PIC_LLENGTH %d\n"
		"  MI_SP_PIC_SIZE %d\n"
		"  MI_SP_Y_BASE_AD 0x%08x/0x%08x\n"
		"  MI_SP_Y_OFFS_CNT %d/%d\n"
		"  MI_SP_Y_OFFS_CNT_START %d\n"
		"  MI_SP_CB_OFFS_CNT %d/%d\n"
		"  MI_SP_CB_OFFS_CNT_START %d\n"
		"  MI_SP_CR_OFFS_CNT %d/%d\n"
		"  MI_SP_CR_OFFS_CNT_START %d\n",
		cif_ioread32(dev->config.base_addr +
			CIF_MI_CTRL),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_CTRL_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_STATUS),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_RIS),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MIS),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_IMSC),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_SIZE_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_SIZE_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_CB_SIZE_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_CB_SIZE_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_CR_SIZE_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_CR_SIZE_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_PIC_WIDTH),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_PIC_HEIGHT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_LLENGTH),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_PIC_SIZE),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_BASE_AD_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_BASE_AD_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_OFFS_CNT_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_OFFS_CNT_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_OFFS_CNT_START),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_CB_OFFS_CNT_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_CB_OFFS_CNT_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_CB_OFFS_CNT_START),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_CR_OFFS_CNT_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_CR_OFFS_CNT_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_CR_OFFS_CNT_START));
}

static void cif_isp20_debug_print_mi_mp(struct cif_isp20_device *dev)
{
	cif_isp20_pltfrm_pr_info(dev->dev,
		"\n  MI_CTRL 0x%08x/0x%08x\n"
		"  MI_STATUS 0x%08x\n"
		"  MI_BYTE_CNT %d\n"
		"  MI_RIS 0x%08x/0x%08x\n"
		"  MI_IMSC 0x%08x\n"
		"  MI_MP_Y_SIZE %d/%d\n"
		"  MI_MP_CB_SIZE %d/%d\n"
		"  MI_MP_CR_SIZE %d/%d\n"
		"  MI_MP_Y_BASE_AD 0x%08x/0x%08x\n"
		"  MI_MP_Y_OFFS_CNT %d/%d\n"
		"  MI_MP_Y_OFFS_CNT_START %d\n"
		"  MI_MP_CB_OFFS_CNT %d/%d\n"
		"  MI_MP_CB_OFFS_CNT_START %d\n"
		"  MI_MP_CR_OFFS_CNT %d/%d\n"
		"  MI_MP_CR_OFFS_CNT_START %d\n",
		cif_ioread32(dev->config.base_addr +
			CIF_MI_CTRL),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_CTRL_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_STATUS),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_BYTE_CNT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_RIS),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MIS),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_IMSC),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_Y_SIZE_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_Y_SIZE_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CB_SIZE_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CB_SIZE_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CR_SIZE_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CR_SIZE_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_Y_BASE_AD_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_Y_BASE_AD_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_Y_OFFS_CNT_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_Y_OFFS_CNT_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_Y_OFFS_CNT_START),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CB_OFFS_CNT_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CB_OFFS_CNT_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CB_OFFS_CNT_START),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CR_OFFS_CNT_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CR_OFFS_CNT_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CR_OFFS_CNT_START));
}

static void cif_isp20_debug_print_srsz(struct cif_isp20_device *dev)
{
	cif_isp20_pltfrm_pr_info(dev->dev,
		"\n  SRSZ_CTRL 0x%08x/0x%08x\n"
		"  SRSZ_SCALE_HY %d/%d\n"
		"  SRSZ_SCALE_HCB %d/%d\n"
		"  SRSZ_SCALE_HCR %d/%d\n"
		"  SRSZ_SCALE_VY %d/%d\n"
		"  SRSZ_SCALE_VC %d/%d\n"
		"  SRSZ_PHASE_HY %d/%d\n"
		"  SRSZ_PHASE_HC %d/%d\n"
		"  SRSZ_PHASE_VY %d/%d\n"
		"  SRSZ_PHASE_VC %d/%d\n",
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_CTRL),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_CTRL_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_SCALE_HY),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_SCALE_HY_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_SCALE_HCB),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_SCALE_HCB_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_SCALE_HCR),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_SCALE_HCR_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_SCALE_VY),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_SCALE_VY_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_SCALE_VC),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_SCALE_VC_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_PHASE_HY),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_PHASE_HY_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_PHASE_HC),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_PHASE_HC_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_PHASE_VY),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_PHASE_VY_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_PHASE_VC),
		cif_ioread32(dev->config.base_addr +
			CIF_SRSZ_PHASE_VC_SHD));
}

static void cif_isp20_debug_print_mrsz(struct cif_isp20_device *dev)
{
	cif_isp20_pltfrm_pr_info(dev->dev,
		"\n  MRSZ_CTRL 0x%08x/0x%08x\n"
		"  MRSZ_SCALE_HY %d/%d\n"
		"  MRSZ_SCALE_HCB %d/%d\n"
		"  MRSZ_SCALE_HCR %d/%d\n"
		"  MRSZ_SCALE_VY %d/%d\n"
		"  MRSZ_SCALE_VC %d/%d\n"
		"  MRSZ_PHASE_HY %d/%d\n"
		"  MRSZ_PHASE_HC %d/%d\n"
		"  MRSZ_PHASE_VY %d/%d\n"
		"  MRSZ_PHASE_VC %d/%d\n",
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_CTRL),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_CTRL_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_SCALE_HY),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_SCALE_HY_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_SCALE_HCB),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_SCALE_HCB_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_SCALE_HCR),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_SCALE_HCR_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_SCALE_VY),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_SCALE_VY_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_SCALE_VC),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_SCALE_VC_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_PHASE_HY),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_PHASE_HY_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_PHASE_HC),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_PHASE_HC_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_PHASE_VY),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_PHASE_VY_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_PHASE_VC),
		cif_ioread32(dev->config.base_addr +
			CIF_MRSZ_PHASE_VC_SHD));
}

static void cif_isp20_debug_print_block(
	struct cif_isp20_device *dev,
	const char *block_name)
{
	if (!strncmp(block_name, "all", 3)) {
		cif_isp20_debug_print_srsz(dev);
		cif_isp20_debug_print_mrsz(dev);
		cif_isp20_debug_print_mi_sp(dev);
		cif_isp20_debug_print_mi_mp(dev);
	} else if (!strncmp(block_name, "srsz", 4))
		cif_isp20_debug_print_srsz(dev);
	else if (!strncmp(block_name, "mrsz", 4))
		cif_isp20_debug_print_mrsz(dev);
	else if (!strncmp(block_name, "mi_sp", 5))
		cif_isp20_debug_print_mi_sp(dev);
	else if (!strncmp(block_name, "mi_mp", 5))
		cif_isp20_debug_print_mi_mp(dev);
	else
		cif_isp20_pltfrm_pr_err(dev->dev,
			"unknown block %s\n", block_name);
}

static u32 cif_isp20_calc_llength(
	u32 width,
	u32 stride,
	enum cif_isp20_pix_fmt pix_fmt)
{
	if (stride == 0)
		return width;

	if (CIF_ISP20_PIX_FMT_IS_YUV(pix_fmt)) {
		u32 num_cplanes =
			CIF_ISP20_PIX_FMT_YUV_GET_NUM_CPLANES(pix_fmt);
		if (num_cplanes == 0)
			return 8 * stride / CIF_ISP20_PIX_FMT_GET_BPP(pix_fmt);
		else
			return stride;
	} else if (CIF_ISP20_PIX_FMT_IS_RGB(pix_fmt))
		return 8 * stride / CIF_ISP20_PIX_FMT_GET_BPP(pix_fmt);
	else
		return width;
}

static int cif_isp20_set_pm_state(
	struct cif_isp20_device *dev,
	enum cif_isp20_pm_state pm_state)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "%s -> %s\n",
		cif_isp20_pm_state_string(dev->pm_state),
		cif_isp20_pm_state_string(pm_state));

	if (dev->pm_state == pm_state)
		return 0;

	ret = cif_isp20_pltfrm_pm_set_state(
		dev->dev,
		pm_state,
		&dev->config.img_src_output);
	if (IS_ERR_VALUE(ret))
		goto err;
	dev->pm_state = pm_state;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
}

static int cif_isp20_img_src_set_state(
	struct cif_isp20_device *dev,
	enum cif_isp20_img_src_state state)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "%s -> %s\n",
		cif_isp20_img_src_state_string(dev->img_src_state),
		cif_isp20_img_src_state_string(state));

	if (dev->img_src_state == state)
		return 0;

	switch (state) {
	case CIF_ISP20_IMG_SRC_STATE_OFF:
		ret = cif_isp20_img_src_s_power(dev->img_src, false);
		break;
	case CIF_ISP20_IMG_SRC_STATE_SW_STNDBY:
		if (dev->img_src_state == CIF_ISP20_IMG_SRC_STATE_STREAMING) {
			ret = cif_isp20_img_src_s_streaming(
				dev->img_src, false);
		} else
			ret = cif_isp20_img_src_s_power(dev->img_src, true);
		break;
	case CIF_ISP20_IMG_SRC_STATE_STREAMING:
		if (dev->config.flash_mode !=
			CIF_ISP20_FLASH_MODE_OFF)
			cif_isp20_img_src_s_ctrl(dev->img_src,
				CIF_ISP20_CID_FLASH_MODE,
				dev->config.flash_mode);
		ret = cif_isp20_img_src_s_streaming(dev->img_src, true);
		break;
	default:
		break;
	}

	if ((dev->config.flash_mode != CIF_ISP20_FLASH_MODE_OFF) &&
		(IS_ERR_VALUE(ret) ||
		(state == CIF_ISP20_IMG_SRC_STATE_OFF)))
		cif_isp20_img_src_s_ctrl(dev->img_src,
			CIF_ISP20_CID_FLASH_MODE,
			CIF_ISP20_FLASH_MODE_OFF);

	if (!IS_ERR_VALUE(ret))
		dev->img_src_state = state;
	else
		cif_isp20_pltfrm_pr_err(dev->dev,
			"failed with err %d\n", ret);

	return ret;
}

static int cif_isp20_img_srcs_init(
	struct cif_isp20_device *dev)
{
	int ret = 0;
	u32 i;
	CIF_ISP20_PLTFRM_DEVICE img_src_device;

	for (i = 0; i < CIF_ISP20_NUM_CSI_INPUTS; i++) {
		img_src_device =
			cif_isp20_pltfrm_get_img_src_device(dev->dev,
				i ? CIF_ISP20_INP_CSI_1 :
					CIF_ISP20_INP_CSI_0);
		if (IS_ERR_OR_NULL(img_src_device)) {
			if (PTR_ERR(img_src_device) == -EPROBE_DEFER) {
				ret = PTR_ERR(img_src_device);
				goto err;
			} else {
				cif_isp20_pltfrm_pr_warn(dev->dev,
					"nothing connected to CIF CSI-%d\n", i);
			}
		} else {
			dev->img_src_array[i] =
				cif_isp20_img_src_to_img_src(img_src_device);
			if (IS_ERR_OR_NULL(dev->img_src_array[i])) {
				dev->img_src_array[i] = NULL;
				ret = -EFAULT;
				goto err;
			}
			cif_isp20_pltfrm_pr_info(dev->dev,
				"device %s attached to CIF CSI-%d\n",
				cif_isp20_pltfrm_dev_string(img_src_device), i);
		}
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_img_src_select_strm_fmt(
	struct cif_isp20_device *dev)
{
	int ret = 0;
	u32 index;
	struct cif_isp20_strm_fmt_desc strm_fmt_desc;
	struct cif_isp20_strm_fmt request_strm_fmt;
	bool matching_format_found = false;
	bool better_match = false;
	u32 target_width, target_height;
	u32 img_src_width, img_src_height;
	u32 best_diff = ~0;
	int vblanking;

	if (IS_ERR_OR_NULL(dev->img_src)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"no image source selected as input (call s_input first)\n");
		ret = -EFAULT;
		goto err;
	}

	ret = cif_isp20_img_src_set_state(dev,
		CIF_ISP20_IMG_SRC_STATE_SW_STNDBY);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_isp20_get_target_frm_size(dev,
		&target_width, &target_height);
	if (IS_ERR_VALUE(ret))
		goto err;

	/* find the best matching format from the image source */
	/* TODO: frame interval and pixel format handling */
	for (index = 0;; index++) {
		if (IS_ERR_VALUE(cif_isp20_img_src_enum_strm_fmts(dev->img_src,
			index, &strm_fmt_desc)))
			break;
		if (!strm_fmt_desc.discrete_frmsize) {
			if (strm_fmt_desc.min_frmsize.width >= target_width)
				img_src_width = strm_fmt_desc.min_frmsize.width;
			else if (strm_fmt_desc.max_frmsize.width >=
				target_width)
				img_src_width = target_width;
			else
				img_src_width = strm_fmt_desc.max_frmsize.width;
			if (strm_fmt_desc.min_frmsize.height >= target_height)
				img_src_height =
					strm_fmt_desc.min_frmsize.height;
			else if (strm_fmt_desc.max_frmsize.height >=
				target_height)
				img_src_height = target_height;
			else
				img_src_height =
					strm_fmt_desc.max_frmsize.height;
		} else {
			img_src_width = strm_fmt_desc.min_frmsize.width;
			img_src_height = strm_fmt_desc.min_frmsize.height;
		}
		if ((img_src_width >= target_width) &&
			(img_src_height >= target_height)) {
			u32 diff = abs(
				target_height -
				(target_width * img_src_height
				/
				img_src_width));
			if (matching_format_found) {
				if (CIF_ISP20_PIX_FMT_IS_JPEG(
					dev->config.mi_config.
					mp.output.pix_fmt) &&
					((img_src_width >=
					request_strm_fmt.frm_fmt.width) &&
					(img_src_height >
					request_strm_fmt.frm_fmt.height)))
					/* for image capturing we try to
						maximize the size */
					better_match = true;
				else if (!CIF_ISP20_PIX_FMT_IS_JPEG(
					dev->config.mi_config.
					mp.output.pix_fmt) &&
					((strm_fmt_desc.min_intrvl.denominator
					/
					strm_fmt_desc.min_intrvl.numerator)
					>
					(request_strm_fmt.frm_intrvl.denominator
					/
					request_strm_fmt.frm_intrvl.numerator)))
					/* maximize fps */
					better_match = true;
				else if (!CIF_ISP20_PIX_FMT_IS_JPEG(
					dev->config.mi_config.
					mp.output.pix_fmt) &&
					((strm_fmt_desc.min_intrvl.denominator
					/
					strm_fmt_desc.min_intrvl.numerator)
					==
					(request_strm_fmt.frm_intrvl.denominator
					/
					request_strm_fmt.frm_intrvl.numerator))
					&&
					(diff < best_diff))
					/* chose better aspect ratio
						match if fps equal */
					better_match = true;
				else
					better_match = false;
			}
			if (!matching_format_found ||
				better_match) {
				request_strm_fmt.frm_fmt.width =
					strm_fmt_desc.min_frmsize.width;
				request_strm_fmt.frm_fmt.height =
					strm_fmt_desc.min_frmsize.height;
				request_strm_fmt.frm_fmt.pix_fmt =
					strm_fmt_desc.pix_fmt;
				request_strm_fmt.frm_intrvl.numerator =
					strm_fmt_desc.min_intrvl.numerator;
				request_strm_fmt.frm_intrvl.denominator =
					strm_fmt_desc.min_intrvl.denominator;
				best_diff = diff;
				matching_format_found = true;
			}
		}
	}

	if (!matching_format_found) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"no matching image source format (%dx%d) found\n",
			target_width, target_height);
		ret = -EINVAL;
		goto err;
	}

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"requesting format %s %dx%d@%d/%dfps from image source\n",
		cif_isp20_pix_fmt_string(request_strm_fmt.frm_fmt.pix_fmt),
		request_strm_fmt.frm_fmt.width,
		request_strm_fmt.frm_fmt.height,
		request_strm_fmt.frm_intrvl.denominator,
		request_strm_fmt.frm_intrvl.numerator);
	ret = cif_isp20_img_src_s_strm_fmt(dev->img_src, &request_strm_fmt);
	if (IS_ERR_VALUE(ret))
		goto err;

	dev->config.img_src_output = request_strm_fmt;

	ret = cif_isp20_img_src_g_ctrl(dev->img_src,
		CIF_ISP20_CID_VBLANKING, &vblanking);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (vblanking > 0)
		dev->isp_dev.v_blanking_us = vblanking;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
}

static void cif_isp20_rsz_ycflt_adjust(
	struct cif_isp20_device *dev,
	u32 *width,
	u32 *height)
{
	u32 ss_ctrl =
		cif_ioread32(dev->config.base_addr + CIF_YC_FLT_CHR_SS_CTRL);
	u32 ss_fac =
		cif_ioread32(dev->config.base_addr + CIF_YC_FLT_CHR_SS_FAC);
	u32 ss_offs =
		cif_ioread32(dev->config.base_addr + CIF_YC_FLT_CHR_SS_OFFS);
	u32 h_mode = (ss_ctrl >> 8) & 0x3;
	u32 v_mode = (ss_ctrl >> 4) & 0x3;
	bool h_en = (ss_ctrl >> 1) & 0x1;
	bool v_en = ss_ctrl & 0x1;
	u32 h_fac = (ss_fac >> 16) & 0xff;
	u32 v_fac = ss_fac & 0xff;
	u32 h_offs = (ss_offs >> 16) & 0xff;
	u32 v_offs = ss_offs & 0xff;
	u32 new_height;
	u32 new_width;
	u32 rem;

	cif_isp20_pltfrm_pr_dbg(NULL,
		"YC_FLT_CHR_SS_CTRL 0x%08x, YC_FLT_CHR_SS_FAC 0x%08x, YC_FLT_CHR_SS_OFFS 0x%08x\n",
		ss_ctrl, ss_fac, ss_offs);

	if (h_en) {
		if ((h_fac & 0x1) || (h_offs & 0x1) || !(h_mode & 0x1)) {
			cif_isp20_pltfrm_pr_err(NULL,
				"YC flt settings for horizontal ss are not even (h_fac %d, h_offs %d, h_mode %d)\n",
				h_fac, h_offs, h_mode);
			BUG();
		}
		h_offs >>= 1;
		h_fac >>= 1;
		new_width =
			(*width - h_offs)
			/
			(h_fac + 1);
		rem = (*width - h_offs) % (h_fac + 1);
		if (rem > 0)
			new_width += 1;
		if (h_mode < 2) /* skip */
			*width -= new_width;
		else /* pass */
			*width = new_width;
	}

	if (v_en) {
		new_height =
			(*height - v_offs)
			/
			(v_fac + 1 + (v_mode & 0x1));
		rem = (*height - v_offs) % (v_fac + 1 + (v_mode & 0x1));
		if (rem > 0)
			new_height += 1;
		if ((rem > 1) && (v_mode & 0x1))
			new_height += 1;
		if (v_mode < 2) /* skip */
			*height -= new_height;
		else /* pass */
			*height = new_height;
	}
}

#ifdef CONFIG_CIF_ISP20_TEST_YC_FLT
extern int cifisp_ycflt_enable(
	struct xgold_isp_dev *isp_dev,
		bool flag,
		__s32 *value);

static __s32 cif_isp20_yc_flt_enable;
static u32 cif_isp20_dbg_count;

static int cif_isp20_enable_yc_flt(
	struct cif_isp20_device *dev)
{
	dev->isp_dev.ycflt_config.ctrl = 0x71;
	dev->isp_dev.ycflt_config.chr_ss_ctrl = 0x323;
	dev->isp_dev.ycflt_config.chr_ss_fac = 0x20003;
	dev->isp_dev.ycflt_config.chr_ss_offs = 0x20002;

	/*
	cifisp_ycflt_enable(&dev->isp_dev, 1, &cif_isp20_yc_flt_enable);
	cif_isp20_yc_flt_enable = cif_isp20_yc_flt_enable ? 0 : 1;
	*/
	cif_isp20_yc_flt_enable = 1;
	cifisp_ycflt_enable(&dev->isp_dev, 1, &cif_isp20_yc_flt_enable);
	return 0;
}
#endif

/* it might be possible that certain values between
output_width - 80 and output_width are not causing any problem.
Create a look up table including only those exact setting
which won't cause problem.  */
static bool cif_isp20_crop_width_acceptable(
	struct cif_isp20_device *dev,
	u32 input,
	u32 output)
{
	u32 i = 0;
	for (i = 0; i < ARRAY_SIZE(accetable_crop_widths); i++) {
		if (input == accetable_crop_widths[i].input_width
			&& output == accetable_crop_widths[i].output_width)
			return true;
	}

	return false;
}

/* This should only be called when configuring CIF
	or at the frame end interrupt */
static void cif_isp20_config_ism(struct cif_isp20_device *dev, bool async)
{
	const struct cif_isp20_ism_config *pconfig =
		&(dev->config.isp_config.ism_config);

	/* some cropping settings cause the MI to hang */
	if ((dev->sp_stream.state >= CIF_ISP20_STATE_READY) &&
		(pconfig->ism_params.h_size <
			dev->config.mi_config.sp.output.width) &&
		((dev->config.mi_config.sp.output.width -
			pconfig->ism_params.h_size) < 80) &&
		!cif_isp20_crop_width_acceptable(dev,
			dev->config.mi_config.sp.output.width,
			(u32)pconfig->ism_params.h_size)) {
			cif_isp20_pltfrm_pr_dbg(dev->dev,
				"SP: Skipping problematic crop settings %dx%d->%dx%d\n",
				dev->config.mi_config.sp.output.width,
				dev->config.mi_config.sp.output.height,
				pconfig->ism_params.h_size,
				pconfig->ism_params.v_size);
			return;
		}
	if ((dev->mp_stream.state >= CIF_ISP20_STATE_READY) &&
		(pconfig->ism_params.h_size <
			dev->config.mi_config.mp.output.width) &&
		((dev->config.mi_config.mp.output.width -
			pconfig->ism_params.h_size) < 80) &&
		!cif_isp20_crop_width_acceptable(dev,
			dev->config.mi_config.mp.output.width,
			(u32)pconfig->ism_params.h_size)) {
			cif_isp20_pltfrm_pr_dbg(dev->dev,
				"MP: Skipping problematic crop settings %dx%d->%dx%d\n",
				dev->config.mi_config.mp.output.width,
				dev->config.mi_config.mp.output.height,
				pconfig->ism_params.h_size,
				pconfig->ism_params.v_size);
			return;
		}

	if (pconfig->ism_en) {
		cif_isp20_pltfrm_pr_dbg(dev->dev, "%dx%d -> %dx%d@(%d,%d)\n",
			dev->isp_dev.input_width,
			dev->isp_dev.input_height,
			pconfig->ism_params.h_size,
			pconfig->ism_params.v_size,
			pconfig->ism_params.h_offs,
			pconfig->ism_params.v_offs);
		cif_iowrite32(pconfig->ism_params.recenter,
			dev->config.base_addr + CIF_ISP_IS_RECENTER);
		cif_iowrite32(pconfig->ism_params.max_dx,
			dev->config.base_addr + CIF_ISP_IS_MAX_DX);
		cif_iowrite32(pconfig->ism_params.max_dy,
			dev->config.base_addr + CIF_ISP_IS_MAX_DY);
		cif_iowrite32(pconfig->ism_params.displace,
			dev->config.base_addr + CIF_ISP_IS_DISPLACE);
		cif_iowrite32(pconfig->ism_params.h_offs,
			dev->config.base_addr + CIF_ISP_IS_H_OFFS);
		cif_iowrite32(pconfig->ism_params.v_offs,
			dev->config.base_addr + CIF_ISP_IS_V_OFFS);
		cif_iowrite32(pconfig->ism_params.h_size,
			dev->config.base_addr + CIF_ISP_IS_H_SIZE);
		cif_iowrite32(pconfig->ism_params.v_size,
			dev->config.base_addr + CIF_ISP_IS_V_SIZE);
		cif_iowrite32OR(1,
			dev->config.base_addr + CIF_ISP_IS_CTRL);
		dev->config.isp_config.output.width =
			dev->config.isp_config.ism_config.ism_params.h_size;
		dev->config.isp_config.output.height =
			dev->config.isp_config.ism_config.ism_params.v_size;
	} else {
		cif_iowrite32(pconfig->ism_params.recenter,
			dev->config.base_addr + CIF_ISP_IS_RECENTER);
		cif_iowrite32(pconfig->ism_params.max_dx,
			dev->config.base_addr + CIF_ISP_IS_MAX_DX);
		cif_iowrite32(pconfig->ism_params.max_dy,
			dev->config.base_addr + CIF_ISP_IS_MAX_DY);
		cif_iowrite32(pconfig->ism_params.displace,
			dev->config.base_addr + CIF_ISP_IS_DISPLACE);
		cif_iowrite32(0,
			dev->config.base_addr + CIF_ISP_IS_H_OFFS);
		cif_iowrite32(0,
			dev->config.base_addr + CIF_ISP_IS_V_OFFS);
		cif_iowrite32(dev->isp_dev.input_width,
			dev->config.base_addr + CIF_ISP_IS_H_SIZE);
		cif_iowrite32(dev->isp_dev.input_height,
			dev->config.base_addr + CIF_ISP_IS_V_SIZE);
		/* cif_iowrite32AND(0,
			dev->config.base_addr + CIF_ISP_IS_CTRL); */
		cif_iowrite32(0,
			dev->config.base_addr + CIF_ISP_IS_CTRL);
		dev->config.isp_config.output.width =
			dev->isp_dev.input_width;
		dev->config.isp_config.output.height =
			dev->isp_dev.input_height;
	}

	if (async)
		cif_iowrite32OR(CIF_ISP_CTRL_ISP_CFG_UPD,
			dev->config.base_addr + CIF_ISP_CTRL);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"\n  ISP_IS_H_OFFS %d/%d\n"
		"  ISP_IS_V_OFFS %d/%d\n"
		"  ISP_IS_H_SIZE %d/%d\n"
		"  ISP_IS_V_SIZE %d/%d\n"
		"  ISP_IS_RECENTER 0x%08x\n"
		"  ISP_IS_MAX_DX %d\n"
		"  ISP_IS_MAX_DY %d\n"
		"  ISP_IS_DISPLACE 0x%08x\n"
		"  ISP_IS_CTRL 0x%08x\n",
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_H_OFFS),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_H_OFFS_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_V_OFFS),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_V_OFFS_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_H_SIZE),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_H_SIZE_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_V_SIZE),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_V_SIZE_SHD),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_RECENTER),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_MAX_DX),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_MAX_DY),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_DISPLACE),
		cif_ioread32(dev->config.base_addr +
			CIF_ISP_IS_CTRL));

	return;
}

static void cif_isp20_program_jpeg_tables(
	struct cif_isp20_device *dev)
{
	unsigned int ratio = dev->config.jpeg_config.ratio;
	unsigned int i = 0;
	unsigned int q, q_next, scale;

	cif_isp20_pltfrm_pr_dbg(NULL, "ratio %d\n", ratio);

	/* Y q-table 0 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_QUANT0,
		dev->config.base_addr + CIF_JPE_TABLE_ID);
	if (ratio != 50) {
		scale = (ratio < 50) ? 5000/ratio : 200 - (ratio << 1);
		for (i = 0; i < 32; i++) {
			q = yq_table_base_zigzag[i * 2];
			q_next = yq_table_base_zigzag[i * 2 + 1];
			q = (scale * q + 50) / 100;
			q = (q > 1) ? ((q < 255) ? q : 255) : 1;
			q_next = (scale * q_next + 50) / 100;
			q_next = (q_next > 1) ?
				((q_next < 255) ? q_next : 255) : 1;
			cif_iowrite32(q_next + (q << 8),
				dev->config.base_addr +
				CIF_JPE_TABLE_DATA);
		}
	} else {
		for (i = 0; i < 32; i++) {
			q = yq_table_base_zigzag[i * 2];
			q_next = yq_table_base_zigzag[i * 2 + 1];
			cif_iowrite32(q_next + (q << 8),
				dev->config.base_addr +
				CIF_JPE_TABLE_DATA);
		}
	}

	/* U/V q-table 0 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_QUANT1,
		dev->config.base_addr + CIF_JPE_TABLE_ID);
	if (ratio != 50) {
		for (i = 0; i < 32; i++) {
			q = uvq_table_base_zigzag[i * 2];
			q_next = uvq_table_base_zigzag[i * 2 + 1];
			q = (scale * q + 50) / 100;
			q = (q > 1) ? ((q < 255) ? q : 255) : 1;
			q_next = (scale * q_next + 50) / 100;
			q_next = (q_next > 1) ?
				((q_next < 255) ? q_next : 255) : 1;
			cif_iowrite32(q_next + (q << 8),
				dev->config.base_addr +
				CIF_JPE_TABLE_DATA);
		}
	} else {
		for (i = 0; i < 32; i++) {
			q = uvq_table_base_zigzag[i * 2];
			q_next = uvq_table_base_zigzag[i * 2 + 1];
			cif_iowrite32(q_next + (q << 8),
				dev->config.base_addr +
				CIF_JPE_TABLE_DATA);
		}
	}

	/* Y AC-table 0 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_HUFFAC0,
		dev->config.base_addr + CIF_JPE_TABLE_ID);
	cif_iowrite32(178, dev->config.base_addr + CIF_JPE_TAC0_LEN);
	for (i = 0; i < (178 / 2); i++) {
		cif_iowrite32(ac_luma_table_annex_k[i * 2 + 1] +
			(ac_luma_table_annex_k[i * 2] << 8),
			dev->config.base_addr +
			CIF_JPE_TABLE_DATA);
	}

	/* U/V AC-table 1 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_HUFFAC1,
		dev->config.base_addr + CIF_JPE_TABLE_ID);
	cif_iowrite32(178, dev->config.base_addr + CIF_JPE_TAC1_LEN);
	for (i = 0; i < (178 / 2); i++) {
		cif_iowrite32(ac_chroma_table_annex_k[i * 2 + 1] +
			(ac_chroma_table_annex_k[i * 2] << 8),
			dev->config.base_addr +
			CIF_JPE_TABLE_DATA);
	}

	/* Y DC-table 0 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_HUFFDC0,
		dev->config.base_addr + CIF_JPE_TABLE_ID);
	cif_iowrite32(28, dev->config.base_addr + CIF_JPE_TDC0_LEN);
	for (i = 0; i < (28 / 2); i++) {
		cif_iowrite32(dc_luma_table_annex_k[i * 2 + 1] +
			(dc_luma_table_annex_k[i * 2] << 8),
			dev->config.base_addr +
			CIF_JPE_TABLE_DATA);
	}

	/* U/V DC-table 1 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_HUFFDC1,
		dev->config.base_addr + CIF_JPE_TABLE_ID);
	cif_iowrite32(28, dev->config.base_addr + CIF_JPE_TDC1_LEN);
	for (i = 0; i < (28 / 2); i++) {
		cif_iowrite32(dc_chroma_table_annex_k[i * 2 + 1] +
		(dc_chroma_table_annex_k[i * 2] << 8),
		dev->config.base_addr +
		CIF_JPE_TABLE_DATA);
	}
}

static void cif_isp20_select_jpeg_tables(
	struct cif_isp20_device *dev)
{
	cif_isp20_pltfrm_pr_dbg(NULL, "\n");

	/* Selects quantization table for Y */
	cif_iowrite32(CIF_JPE_TQ_TAB0,
		dev->config.base_addr + CIF_JPE_TQ_Y_SELECT);
	/* Selects quantization table for U */
	cif_iowrite32(CIF_JPE_TQ_TAB1,
		dev->config.base_addr + CIF_JPE_TQ_U_SELECT);
	/* Selects quantization table for V */
	cif_iowrite32(CIF_JPE_TQ_TAB1,
		dev->config.base_addr + CIF_JPE_TQ_V_SELECT);
	/* Selects Huffman DC table */
	cif_iowrite32(CIF_DC_V_TABLE | CIF_DC_U_TABLE,
		dev->config.base_addr + CIF_JPE_DC_TABLE_SELECT);
	/* Selects Huffman AC table */
	cif_iowrite32(CIF_AC_V_TABLE | CIF_AC_U_TABLE,
		dev->config.base_addr + CIF_JPE_AC_TABLE_SELECT);

	cif_isp20_pltfrm_pr_dbg(NULL,
		"\n  JPE_TQ_Y_SELECT 0x%08x\n"
		"  JPE_TQ_U_SELECT 0x%08x\n"
		"  JPE_TQ_V_SELECT 0x%08x\n"
		"  JPE_DC_TABLE_SELECT 0x%08x\n"
		"  JPE_AC_TABLE_SELECT 0x%08x\n",
		cif_ioread32(dev->config.base_addr + CIF_JPE_TQ_Y_SELECT),
		cif_ioread32(dev->config.base_addr + CIF_JPE_TQ_U_SELECT),
		cif_ioread32(dev->config.base_addr + CIF_JPE_TQ_V_SELECT),
		cif_ioread32(dev->config.base_addr + CIF_JPE_DC_TABLE_SELECT),
		cif_ioread32(dev->config.base_addr + CIF_JPE_AC_TABLE_SELECT));
}

static int cif_isp20_config_img_src(
	struct cif_isp20_device *dev)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "\n");

	ret = cif_isp20_img_src_set_state(dev,
		CIF_ISP20_IMG_SRC_STATE_SW_STNDBY);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (!dev->sp_stream.updt_cfg &&
		!dev->mp_stream.updt_cfg)
		return 0;

	ret = cif_isp20_pltfrm_g_csi_config(dev->dev,
		dev->config.mipi_config.input_sel ?
			CIF_ISP20_INP_CSI_1 : CIF_ISP20_INP_CSI_0,
		&dev->config.img_src_output,
		&dev->config.mipi_config.csi_config);

	if (IS_ERR_VALUE(ret))
		goto err;
	ret = cif_isp20_pltfrm_write_cif_ana_bandgap_bias(dev->dev,
			dev->config.mipi_config.csi_config.ana_bandgab_bias);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_config_isp(
	struct cif_isp20_device *dev)
{
	int ret = 0;
	u32 input_width;
	u32 input_height;
	u32 h_offs;
	u32 v_offs;
	u32 yuv_seq = 0;
	u32 bpp;
	u32 isp_input_sel = 0;
	u32 isp_bayer_pat = 0;
	u32 acq_mult = 1;
	u32 irq_mask = 0;
	enum cif_isp20_pix_fmt in_pix_fmt;
	struct cif_isp20_frm_fmt *output;

	if (dev->config.input_sel == CIF_ISP20_INP_DMA_IE) {
		dev->config.isp_config.output =
			dev->config.mi_config.dma.output;
		cifisp_disable_isp(&dev->isp_dev);
		return 0;
	} else if (dev->config.input_sel == CIF_ISP20_INP_DMA_SP) {
		cif_iowrite32AND(~CIF_ICCL_ISP_CLK,
			dev->config.base_addr + CIF_ICCL);
		cif_isp20_pltfrm_pr_dbg(NULL,
			"ISP disabled\n");
		return 0;
	}
	cif_iowrite32OR(CIF_ICCL_ISP_CLK,
		dev->config.base_addr + CIF_ICCL);

	in_pix_fmt = dev->config.isp_config.input->pix_fmt;
	input_width = dev->config.isp_config.input->width;
	input_height = dev->config.isp_config.input->height;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "%s %dx%d\n",
		cif_isp20_pix_fmt_string(in_pix_fmt),
		input_width, input_height);

	output = &dev->config.isp_config.output;

	if (CIF_ISP20_PIX_FMT_IS_RAW_BAYER(in_pix_fmt)) {
		if (!CIF_ISP20_PIX_FMT_IS_RAW_BAYER(
			dev->config.mi_config.mp.output.pix_fmt)) {
			output->pix_fmt = CIF_YUV422I;
			cif_iowrite32(0xc,
				dev->config.base_addr + CIF_ISP_DEMOSAIC);
			cif_iowrite32(CIF_ISP_CTRL_ISP_MODE_BAYER_ITU601,
				dev->config.base_addr + CIF_ISP_CTRL);
		} else {
			output->pix_fmt = in_pix_fmt;
			cif_iowrite32(CIF_ISP_CTRL_ISP_MODE_RAW_PICT,
				dev->config.base_addr + CIF_ISP_CTRL);
		}

		bpp = CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt);
		if (bpp == 8)
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_8B_MSB;
		else if (bpp == 10)
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_10B_MSB;
		else if (bpp == 12)
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_12B;
		else {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"%d bits per pixel not supported\n", bpp);
			ret = -EINVAL;
			goto err;
		}
		if (CIF_ISP20_PIX_FMT_BAYER_PAT_IS_BGGR(in_pix_fmt))
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_BGGR;
		else if (CIF_ISP20_PIX_FMT_BAYER_PAT_IS_GBRG(in_pix_fmt))
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_GBRG;
		else if (CIF_ISP20_PIX_FMT_BAYER_PAT_IS_GRBG(in_pix_fmt))
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_GRBG;
		else if (CIF_ISP20_PIX_FMT_BAYER_PAT_IS_RGGB(in_pix_fmt))
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_RGGB;
		else {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"BAYER pattern not supported\n");
			ret = -EINVAL;
			goto err;
		}
	} else if (CIF_ISP20_PIX_FMT_IS_YUV(in_pix_fmt)) {
		output->pix_fmt = in_pix_fmt;
		acq_mult = 2;
		if (dev->config.input_sel == CIF_ISP20_INP_DMA) {
			bpp = CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt);
			bpp =
				bpp * 4
				/
				(4 + (CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(
					in_pix_fmt) *
				CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(
					in_pix_fmt) / 2));
			if (bpp == 8)
				isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_8B_MSB;
			else if (bpp == 10)
				isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_10B_MSB;
			else if (bpp == 12)
				isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_12B;
			else {
				cif_isp20_pltfrm_pr_err(dev->dev,
					"format %s not supported, invalid bpp %d\n",
					cif_isp20_pix_fmt_string(in_pix_fmt),
					bpp);
				ret = -EINVAL;
				goto err;
			}
			cif_iowrite32(CIF_ISP_CTRL_ISP_MODE_ITU601 |
				CIF_ISP_CTRL_ISP_YUV_FAST_MODE_DIS,
				dev->config.base_addr + CIF_ISP_CTRL);
		} else {
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_12B;
			cif_iowrite32(CIF_ISP_CTRL_ISP_MODE_ITU601,
				dev->config.base_addr + CIF_ISP_CTRL);
			/* ISP DATA LOSS is only meaningful
				when input is not DMA */
			irq_mask |= CIF_ISP_DATA_LOSS;
		}
		if (CIF_ISP20_PIX_FMT_YUV_IS_YC_SWAPPED(in_pix_fmt)) {
			yuv_seq = CIF_ISP_ACQ_PROP_CBYCRY;
			cif_isp20_pix_fmt_set_yc_swapped(output->pix_fmt, 0);
		} else if (CIF_ISP20_PIX_FMT_YUV_IS_UV_SWAPPED(in_pix_fmt))
			yuv_seq = CIF_ISP_ACQ_PROP_YCRYCB;
		else
			yuv_seq = CIF_ISP_ACQ_PROP_YCBYCR;
	} else {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"format %s not supported\n",
			cif_isp20_pix_fmt_string(in_pix_fmt));
		ret = -EINVAL;
		goto err;
	}

	/* Set up input acquisition properties*/
	cif_iowrite32(CIF_ISP_ACQ_PROP_NEG_EDGE |
		CIF_ISP_ACQ_PROP_HSYNC_HIGH |
		CIF_ISP_ACQ_PROP_VSYNC_HIGH |
		yuv_seq |
		CIF_ISP_ACQ_PROP_FIELD_SEL_ALL |
		isp_input_sel |
		isp_bayer_pat |
		(0 << 20),  /*input_selection_no_app */
		dev->config.base_addr + CIF_ISP_ACQ_PROP);
	cif_iowrite32(0,
		dev->config.base_addr + CIF_ISP_ACQ_NR_FRAMES);
	/* Acquisition Size */
	cif_iowrite32(acq_mult * input_width,
		dev->config.base_addr + CIF_ISP_ACQ_H_SIZE);
	cif_iowrite32(input_height,
		dev->config.base_addr + CIF_ISP_ACQ_V_SIZE);

	/* do cropping to match output aspect ratio */
	ret = cif_isp20_calc_isp_cropping(dev,
		&output->width, &output->height,
		&h_offs, &v_offs);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_iowrite32(v_offs,
		dev->config.base_addr + CIF_ISP_OUT_V_OFFS);
	cif_iowrite32(h_offs,
		dev->config.base_addr + CIF_ISP_OUT_H_OFFS);
	cif_iowrite32(output->width,
		dev->config.base_addr + CIF_ISP_OUT_H_SIZE);
	cif_iowrite32(output->height,
		dev->config.base_addr + CIF_ISP_OUT_V_SIZE);

	dev->isp_dev.input_width = output->width;
	dev->isp_dev.input_height = output->height;

	/* interrupt mask */
	irq_mask |=
		CIF_ISP_FRAME |
		CIF_ISP_PIC_SIZE_ERROR |
		CIF_ISP_FRAME_IN |
		CIF_ISP_V_START;
	cif_iowrite32(irq_mask,
		dev->config.base_addr + CIF_ISP_IMSC);

	if (!CIF_ISP20_PIX_FMT_IS_RAW_BAYER(
		dev->config.mi_config.mp.output.pix_fmt))
		cifisp_configure_isp(&dev->isp_dev,
			in_pix_fmt,
			CIF_ISP20_PIX_FMT_IS_JPEG(
				dev->config.mi_config.mp.output.pix_fmt) &&
			(dev->config.input_sel <= CIF_ISP20_INP_CPI));
	else
		cifisp_disable_isp(&dev->isp_dev);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"\n  ISP_CTRL 0x%08x\n"
		"  ISP_IMSC 0x%08x\n"
		"  ISP_ACQ_PROP 0x%08x\n"
		"  ISP_ACQ %dx%d@(%d,%d)\n"
		"  ISP_OUT %dx%d@(%d,%d)\n"
		"  ISP_IS %dx%d@(%d,%d)\n",
		cif_ioread32(dev->config.base_addr + CIF_ISP_CTRL),
		cif_ioread32(dev->config.base_addr + CIF_ISP_IMSC),
		cif_ioread32(dev->config.base_addr + CIF_ISP_ACQ_PROP),
		cif_ioread32(dev->config.base_addr + CIF_ISP_ACQ_H_SIZE),
		cif_ioread32(dev->config.base_addr + CIF_ISP_ACQ_V_SIZE),
		cif_ioread32(dev->config.base_addr + CIF_ISP_ACQ_H_OFFS),
		cif_ioread32(dev->config.base_addr + CIF_ISP_ACQ_V_OFFS),
		cif_ioread32(dev->config.base_addr + CIF_ISP_OUT_H_SIZE),
		cif_ioread32(dev->config.base_addr + CIF_ISP_OUT_V_SIZE),
		cif_ioread32(dev->config.base_addr + CIF_ISP_OUT_H_OFFS),
		cif_ioread32(dev->config.base_addr + CIF_ISP_OUT_V_OFFS),
		cif_ioread32(dev->config.base_addr + CIF_ISP_IS_H_SIZE),
		cif_ioread32(dev->config.base_addr + CIF_ISP_IS_V_SIZE),
		cif_ioread32(dev->config.base_addr + CIF_ISP_IS_H_OFFS),
		cif_ioread32(dev->config.base_addr + CIF_ISP_IS_V_OFFS));

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_config_mipi(
	struct cif_isp20_device *dev)
{
	int ret = 0;
	u32 data_type;
	u32 mipi_ctrl;
	u32 shutdown_lanes;
	u32 i;
	enum cif_isp20_pix_fmt in_pix_fmt;

	if (dev->config.input_sel > CIF_ISP20_INP_CSI_1) {
		cif_iowrite32AND(~CIF_ICCL_MIPI_CLK,
			dev->config.base_addr + CIF_ICCL);
		cif_isp20_pltfrm_pr_dbg(NULL,
			"MIPI disabled\n");
		return 0;
	}
	cif_iowrite32OR(CIF_ICCL_MIPI_CLK,
		dev->config.base_addr + CIF_ICCL);

	in_pix_fmt = dev->config.img_src_output.frm_fmt.pix_fmt;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"input %d, vc = %d, nb_lanes = %d, dphy1 = 0x%08x, dphy2 = 0x%02x, ana_bandgap_bias = %d\n",
		dev->config.mipi_config.input_sel,
		dev->config.mipi_config.csi_config.vc,
		dev->config.mipi_config.csi_config.nb_lanes,
		dev->config.mipi_config.csi_config.dphy1,
		dev->config.mipi_config.csi_config.dphy2,
		dev->config.mipi_config.csi_config.ana_bandgab_bias);

	if ((dev->config.mipi_config.csi_config.nb_lanes == 0) ||
		(dev->config.mipi_config.csi_config.nb_lanes > 4)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"invalid number (%d) of MIPI lanes, valid range is [1..4]\n",
			dev->config.mipi_config.csi_config.nb_lanes);
		ret = -EINVAL;
		goto err;
	}

	shutdown_lanes = 0xe;
	for (i = dev->config.mipi_config.csi_config.nb_lanes - 1; i; i--)
		shutdown_lanes &= (shutdown_lanes << 1);

	mipi_ctrl =
		CIF_MIPI_CTRL_NUM_LANES(
			dev->config.mipi_config.csi_config.nb_lanes - 1) |
		CIF_MIPI_CTRL_ERR_SOT_HS_SKIP |
		CIF_MIPI_CTRL_ERR_SOT_SYNC_HS_SKIP |
		CIF_MIPI_CTRL_SHUTDOWNLANES(shutdown_lanes) |
		CIF_MIPI_CTRL_ERR_SOT_SYNC_HS_ENA |
		CIF_MIPI_CTRL_ERR_SOT_HS_ENA |
		CIF_MIPI_CTRL_FORCE_EARLY_ENABLE;

	if (dev->config.mipi_config.input_sel == 0) {
		/* mipi_dphy */
		cif_iowrite32(dev->config.mipi_config.csi_config.dphy1,
			dev->config.base_addr + CIF_MIPI_DPHY1_1);
		cif_iowrite32(dev->config.mipi_config.csi_config.dphy2,
			dev->config.base_addr + CIF_MIPI_DPHY1_2);
		mipi_ctrl |= CIF_MIPI_CTRL_DPHY_SELECT_PRIMARY;
	} else if (dev->config.mipi_config.input_sel == 1) {
		/* mipi_dphy */
		cif_iowrite32(dev->config.mipi_config.csi_config.dphy1,
			dev->config.base_addr + CIF_MIPI_DPHY2_1);
		cif_iowrite32(dev->config.mipi_config.csi_config.dphy2,
			dev->config.base_addr + CIF_MIPI_DPHY2_2);
		mipi_ctrl |= CIF_MIPI_CTRL_DPHY_SELECT_SECONDARY;
	} else {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"invalid input %d, valid range is [0..1]\n",
			dev->config.mipi_config.input_sel);
		ret = -EINVAL;
		goto err;
	}
	cif_iowrite32(mipi_ctrl,
		dev->config.base_addr + CIF_MIPI_CTRL);

	/* Configure Data Type and Virtual Channel */
	if (CIF_ISP20_PIX_FMT_IS_YUV(in_pix_fmt)) {
		if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt) == 12))
			data_type = CSI2_DT_YUV420_8b;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt) == 15))
			data_type = CSI2_DT_YUV420_10b;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 4) &&
			(CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt) == 16))
			data_type = CSI2_DT_YUV422_8b;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 4) &&
			(CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt) == 20))
			data_type = CSI2_DT_YUV422_10b;
		else {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"unsupported format %s\n",
				cif_isp20_pix_fmt_string(in_pix_fmt));
			ret = -EINVAL;
			goto err;
		}
	} else if (CIF_ISP20_PIX_FMT_IS_RAW_BAYER(in_pix_fmt)) {
		if (CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt) == 8)
			data_type = CSI2_DT_RAW8;
		else if (CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt) == 10)
			data_type = CSI2_DT_RAW10;
		else if (CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt) == 12)
			data_type = CSI2_DT_RAW12;
		else {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"unsupported format %s\n",
				cif_isp20_pix_fmt_string(in_pix_fmt));
			ret = -EINVAL;
			goto err;
		}
	} else if (in_pix_fmt == CIF_RGB565) {
		data_type = CSI2_DT_RGB565;
	} else if (in_pix_fmt == CIF_RGB666) {
		data_type = CSI2_DT_RGB666;
	} else if (in_pix_fmt == CIF_RGB888) {
		data_type = CSI2_DT_RGB888;
	} else {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"unsupported format %s\n",
			cif_isp20_pix_fmt_string(in_pix_fmt));
		ret = -EINVAL;
		goto err;
	}

	cif_iowrite32(
		CIF_MIPI_DATA_SEL_DT(data_type) |
		CIF_MIPI_DATA_SEL_VC(
			dev->config.mipi_config.csi_config.vc),
		dev->config.base_addr + CIF_MIPI_IMG_DATA_SEL);

	/* Enable MIPI interrupts */
	cif_iowrite32(~0,
		dev->config.base_addr + CIF_MIPI_ICR);
	cif_iowrite32(
		CIF_MIPI_ERR_CSI |
		CIF_MIPI_ERR_DPHY |
		CIF_MIPI_SYNC_FIFO_OVFLW(3) |
		CIF_MIPI_ADD_DATA_OVFLW,
		dev->config.base_addr + CIF_MIPI_IMSC);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"\n  MIPI_CTRL 0x%08x\n"
		"  MIPI_IMG_DATA_SEL 0x%08x\n"
		"  MIPI_STATUS 0x%08x\n"
		"  MIPI_IMSC 0x%08x\n"
		"  MIPI_DPHY1_1 0x%08x\n"
		"  MIPI_DPHY1_2 0x%08x\n"
		"  MIPI_DPHY2_1 0x%08x\n"
		"  MIPI_DPHY2_2 0x%08x\n",
		cif_ioread32(dev->config.base_addr + CIF_MIPI_CTRL),
		cif_ioread32(dev->config.base_addr + CIF_MIPI_IMG_DATA_SEL),
		cif_ioread32(dev->config.base_addr + CIF_MIPI_STATUS),
		cif_ioread32(dev->config.base_addr + CIF_MIPI_IMSC),
		cif_ioread32(dev->config.base_addr + CIF_MIPI_DPHY1_1),
		cif_ioread32(dev->config.base_addr + CIF_MIPI_DPHY1_2),
		cif_ioread32(dev->config.base_addr + CIF_MIPI_DPHY2_1),
		cif_ioread32(dev->config.base_addr + CIF_MIPI_DPHY2_2));

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_config_mi_mp(
	struct cif_isp20_device *dev)
{
	enum cif_isp20_pix_fmt out_pix_fmt =
		dev->config.mi_config.mp.output.pix_fmt;
	u32 llength =
		dev->config.mi_config.mp.llength;
	u32 width =
		dev->config.mi_config.mp.output.width;
	u32 height =
		dev->config.mi_config.mp.output.height;
	u32 writeformat = CIF_ISP20_BUFF_FMT_PLANAR;
	u32 swap_cb_cr = 0;
	u32 bpp = CIF_ISP20_PIX_FMT_GET_BPP(out_pix_fmt);
	u32 size = llength * height * bpp / 8;
	u32 mi_ctrl;

	dev->config.mi_config.mp.input =
		&dev->config.mp_config.rsz_config.output;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s %dx%d, llength = %d\n",
		cif_isp20_pix_fmt_string(out_pix_fmt),
		width,
		height,
		llength);

	dev->config.mi_config.mp.y_size = size;
	dev->config.mi_config.mp.cb_size = 0;
	dev->config.mi_config.mp.cr_size = 0;
	if (CIF_ISP20_PIX_FMT_IS_YUV(out_pix_fmt)) {
		u32 num_cplanes =
			CIF_ISP20_PIX_FMT_YUV_GET_NUM_CPLANES(out_pix_fmt);
		if (num_cplanes == 0) {
			writeformat = CIF_ISP20_BUFF_FMT_INTERLEAVED;
		} else {
			dev->config.mi_config.mp.y_size =
				(dev->config.mi_config.mp.y_size * 4)
				/
				(4 + (CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(
					out_pix_fmt) *
				CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(
					out_pix_fmt) / 2));
			dev->config.mi_config.mp.cb_size =
				size -
				dev->config.mi_config.mp.y_size;
			if (num_cplanes == 1)
				writeformat = CIF_ISP20_BUFF_FMT_SEMIPLANAR;
			else if (num_cplanes == 2) {
				writeformat = CIF_ISP20_BUFF_FMT_PLANAR;
				dev->config.mi_config.mp.cb_size /= 2;
			}
			/* for U<->V swapping: */
			dev->config.mi_config.mp.cr_size =
				dev->config.mi_config.mp.cb_size;
		}
		if (CIF_ISP20_PIX_FMT_YUV_IS_UV_SWAPPED(out_pix_fmt))
			swap_cb_cr = CIF_MI_MP_CB_CR_SWAP;
	} else if (CIF_ISP20_PIX_FMT_IS_RAW_BAYER(out_pix_fmt) &&
		CIF_ISP20_PIX_FMT_GET_BPP(out_pix_fmt) > 8) {
		writeformat = CIF_ISP20_BUFF_FMT_SEMIPLANAR;
		dev->config.mi_config.mp.y_size = width * height;
		dev->config.mi_config.mp.cb_size =
			dev->config.mi_config.mp.y_size;
	}

	if (writeformat == CIF_ISP20_BUFF_FMT_SEMIPLANAR) {
		dev->config.mi_config.mp.cb_offs =
		    dev->config.mi_config.mp.y_size;
		dev->config.mi_config.mp.cr_offs =
		    dev->config.mi_config.mp.cb_offs;
	} else if (writeformat == CIF_ISP20_BUFF_FMT_PLANAR) {
		if (swap_cb_cr) {
			swap_cb_cr = 0;
			dev->config.mi_config.mp.cr_offs =
				dev->config.mi_config.mp.y_size;
			dev->config.mi_config.mp.cb_offs =
				dev->config.mi_config.mp.cr_offs +
				dev->config.mi_config.mp.cr_size;
		} else {
			dev->config.mi_config.mp.cb_offs =
				dev->config.mi_config.mp.y_size;
			dev->config.mi_config.mp.cr_offs =
				dev->config.mi_config.mp.cb_offs +
				dev->config.mi_config.mp.cb_size;
		}
	}

	cif_iowrite32_verify(dev->config.mi_config.mp.y_size,
		dev->config.base_addr + CIF_MI_MP_Y_SIZE_INIT, ~0x3);
	cif_iowrite32_verify(dev->config.mi_config.mp.cb_size,
		dev->config.base_addr + CIF_MI_MP_CB_SIZE_INIT, ~0x3);
	cif_iowrite32_verify(dev->config.mi_config.mp.cr_size,
		dev->config.base_addr + CIF_MI_MP_CR_SIZE_INIT, ~0x3);
	cif_iowrite32OR_verify(CIF_MI_MP_FRAME,
		dev->config.base_addr +
		CIF_MI_IMSC, ~0);
/*
	cif_iowrite32_verify(0x303030,
		dev->config.base_addr + CIF_MI_QOS_WRITE_MP, 0x3f3f3f);
*/

	mi_ctrl = cif_ioread32(dev->config.base_addr + CIF_MI_CTRL) |
		CIF_MI_CTRL_MP_WRITE_FMT(writeformat) |
		swap_cb_cr |
		CIF_MI_CTRL_BURST_LEN_LUM_64 |
		CIF_MI_CTRL_BURST_LEN_CHROM_64 |
		CIF_MI_CTRL_INIT_BASE_EN |
		CIF_MI_CTRL_INIT_OFFSET_EN;
	if (CIF_ISP20_PIX_FMT_IS_JPEG(
		dev->config.mi_config.mp.output.pix_fmt))
		mi_ctrl |= CIF_MI_CTRL_JPEG_ENABLE;

	cif_iowrite32_verify(mi_ctrl,
		dev->config.base_addr + CIF_MI_CTRL, ~0);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"\n  MI_CTRL 0x%08x\n"
		"  MI_STATUS 0x%08x\n"
		"  MI_MP_Y_SIZE %d\n"
		"  MI_MP_CB_SIZE %d\n"
		"  MI_MP_CR_SIZE %d\n",
		cif_ioread32(dev->config.base_addr +
			CIF_MI_CTRL),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_STATUS),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_Y_SIZE_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CB_SIZE_INIT),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_CR_SIZE_INIT));

	return 0;
}

static int cif_isp20_config_mi_sp(
	struct cif_isp20_device *dev)
{
	int ret = 0;
	enum cif_isp20_pix_fmt out_pix_fmt =
		dev->config.mi_config.sp.output.pix_fmt;
	enum cif_isp20_pix_fmt in_pix_fmt =
		dev->config.sp_config.rsz_config.output.pix_fmt;
	u32 llength =
		dev->config.mi_config.sp.llength;
	u32 width =
		dev->config.mi_config.sp.output.width;
	u32 height =
		dev->config.mi_config.sp.output.height;
	u32 writeformat = CIF_ISP20_BUFF_FMT_PLANAR;
	u32 swap_cb_cr = 0;
	u32 bpp = CIF_ISP20_PIX_FMT_GET_BPP(out_pix_fmt);
	u32 size = llength * height * bpp / 8;
	u32 input_format = 0;
	u32 output_format;
	u32 mi_ctrl;

	dev->config.mi_config.sp.input =
		&dev->config.sp_config.rsz_config.output;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s %dx%d, llength = %d\n",
		cif_isp20_pix_fmt_string(out_pix_fmt),
		width,
		height,
		llength);

	if (!CIF_ISP20_PIX_FMT_IS_YUV(in_pix_fmt)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"unsupported format %s (must be YUV)\n",
			cif_isp20_pix_fmt_string(in_pix_fmt));
		ret = -EINVAL;
		goto err;
	}

	dev->config.mi_config.sp.y_size = size;
	dev->config.mi_config.sp.cb_size = 0;
	dev->config.mi_config.sp.cr_size = 0;
	if (CIF_ISP20_PIX_FMT_IS_YUV(out_pix_fmt)) {
		u32 num_cplanes =
			CIF_ISP20_PIX_FMT_YUV_GET_NUM_CPLANES(out_pix_fmt);
		if (num_cplanes == 0) {
			writeformat = CIF_ISP20_BUFF_FMT_INTERLEAVED;
		} else {
			dev->config.mi_config.sp.y_size =
				(dev->config.mi_config.sp.y_size * 4)
				/
				(4 + (CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(
					out_pix_fmt) *
				CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(
					out_pix_fmt) / 2));
			dev->config.mi_config.sp.cb_size =
				size -
				dev->config.mi_config.sp.y_size;
			if (num_cplanes == 1)
				writeformat = CIF_ISP20_BUFF_FMT_SEMIPLANAR;
			else if (num_cplanes == 2) {
				writeformat = CIF_ISP20_BUFF_FMT_PLANAR;
				dev->config.mi_config.sp.cb_size /= 2;
			}
			/* for U<->V swapping: */
			dev->config.mi_config.sp.cr_size =
				dev->config.mi_config.sp.cb_size;
		}
		if (CIF_ISP20_PIX_FMT_YUV_IS_UV_SWAPPED(out_pix_fmt))
			swap_cb_cr = CIF_MI_SP_CB_CR_SWAP;

		if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt) == 0) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 0))
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_YUV400;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 2))
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_YUV420;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 4))
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_YUV422;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt) == 4) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 4))
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_YUV444;
		else {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"unsupported YUV output format %s\n",
				cif_isp20_pix_fmt_string(out_pix_fmt));
			ret = -EINVAL;
			goto err;
		}
	} else if (CIF_ISP20_PIX_FMT_IS_RGB(out_pix_fmt)) {
		if (out_pix_fmt == CIF_RGB565)
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_RGB565;
		else if (out_pix_fmt == CIF_RGB666)
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_RGB666;
		else if (out_pix_fmt == CIF_RGB888)
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_RGB888;
		else {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"unsupported RGB output format %s\n",
				cif_isp20_pix_fmt_string(out_pix_fmt));
			ret = -EINVAL;
			goto err;
		}
	} else {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"unsupported output format %s\n",
			cif_isp20_pix_fmt_string(out_pix_fmt));
		ret = -EINVAL;
		goto err;
	}

	if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 0) &&
		(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 0))
		input_format = CIF_MI_CTRL_SP_INPUT_FMT_YUV400;
	else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
		(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 2))
		input_format = CIF_MI_CTRL_SP_INPUT_FMT_YUV420;
	else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
		(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 4))
		input_format = CIF_MI_CTRL_SP_INPUT_FMT_YUV422;
	else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 4) &&
		(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 4))
		input_format = CIF_MI_CTRL_SP_INPUT_FMT_YUV444;
	else {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"unsupported YUV input format %s\n",
			cif_isp20_pix_fmt_string(in_pix_fmt));
		ret = -EINVAL;
		goto err;
	}

	if (writeformat == CIF_ISP20_BUFF_FMT_SEMIPLANAR) {
		dev->config.mi_config.sp.cb_offs =
		    dev->config.mi_config.sp.y_size;
		dev->config.mi_config.sp.cr_offs =
		    dev->config.mi_config.sp.cb_offs;
	} else if (writeformat == CIF_ISP20_BUFF_FMT_PLANAR) {
		if (swap_cb_cr) {
			swap_cb_cr = 0;
			dev->config.mi_config.sp.cr_offs =
				dev->config.mi_config.sp.y_size;
			dev->config.mi_config.sp.cb_offs =
				dev->config.mi_config.sp.cr_offs +
				dev->config.mi_config.sp.cr_size;
		} else {
			dev->config.mi_config.sp.cb_offs =
				dev->config.mi_config.sp.y_size;
			dev->config.mi_config.sp.cr_offs =
				dev->config.mi_config.sp.cb_offs +
				dev->config.mi_config.sp.cb_size;
		}
	}

	cif_iowrite32_verify(dev->config.mi_config.sp.y_size,
		dev->config.base_addr + CIF_MI_SP_Y_SIZE_INIT, ~0x3);
	cif_iowrite32_verify(dev->config.mi_config.sp.y_size,
		dev->config.base_addr + CIF_MI_SP_Y_PIC_SIZE, ~0x3);
	cif_iowrite32_verify(dev->config.mi_config.sp.cb_size,
		dev->config.base_addr + CIF_MI_SP_CB_SIZE_INIT, ~0x3);
	cif_iowrite32_verify(dev->config.mi_config.sp.cr_size,
		dev->config.base_addr + CIF_MI_SP_CR_SIZE_INIT, ~0x3);
	cif_iowrite32_verify(width,
		dev->config.base_addr + CIF_MI_SP_Y_PIC_WIDTH, ~0x3);
	cif_iowrite32_verify(height,
		dev->config.base_addr + CIF_MI_SP_Y_PIC_HEIGHT, ~0x3);
	cif_iowrite32_verify(llength,
		dev->config.base_addr + CIF_MI_SP_Y_LLENGTH, ~0x3);
	cif_iowrite32OR_verify(CIF_MI_SP_FRAME,
		dev->config.base_addr +
		CIF_MI_IMSC, ~0);
/*
	cif_iowrite32_verify(0x303030,
		dev->config.base_addr + CIF_MI_QOS_WRITE_SP, 0x3f3f3f);
*/

	cif_iowrite32_verify(0x303030,
		dev->config.base_addr + CIF_MI_QOS_WRITE_SP, 0x3f3f3f);


	mi_ctrl = cif_ioread32(dev->config.base_addr + CIF_MI_CTRL) |
		CIF_MI_CTRL_SP_WRITE_FMT(writeformat) |
		swap_cb_cr |
		input_format |
		output_format |
		CIF_MI_CTRL_BURST_LEN_LUM_64 |
		CIF_MI_CTRL_BURST_LEN_CHROM_64 |
		CIF_MI_CTRL_INIT_BASE_EN |
		CIF_MI_CTRL_INIT_OFFSET_EN;
	cif_iowrite32_verify(mi_ctrl,
		dev->config.base_addr + CIF_MI_CTRL, ~0);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"\n  MI_CTRL 0x%08x\n"
		"  MI_STATUS 0x%08x\n"
		"  MI_SP_Y_SIZE %d\n"
		"  MI_SP_CB_SIZE %d\n"
		"  MI_SP_CR_SIZE %d\n"
		"  MI_SP_PIC_WIDTH %d\n"
		"  MI_SP_PIC_HEIGHT %d\n"
		"  MI_SP_PIC_LLENGTH %d\n"
		"  MI_SP_PIC_SIZE %d\n",
		cif_ioread32(dev->config.base_addr + CIF_MI_CTRL),
		cif_ioread32(dev->config.base_addr + CIF_MI_STATUS),
		cif_ioread32(dev->config.base_addr + CIF_MI_SP_Y_SIZE_INIT),
		cif_ioread32(dev->config.base_addr + CIF_MI_SP_CB_SIZE_INIT),
		cif_ioread32(dev->config.base_addr + CIF_MI_SP_CR_SIZE_INIT),
		cif_ioread32(dev->config.base_addr + CIF_MI_SP_Y_PIC_WIDTH),
		cif_ioread32(dev->config.base_addr + CIF_MI_SP_Y_PIC_HEIGHT),
		cif_ioread32(dev->config.base_addr + CIF_MI_SP_Y_LLENGTH),
		cif_ioread32(dev->config.base_addr + CIF_MI_SP_Y_PIC_SIZE));

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_config_mi_dma(
	struct cif_isp20_device *dev)
{
	int ret = 0;
	enum cif_isp20_pix_fmt out_pix_fmt =
		dev->config.mi_config.dma.output.pix_fmt;
	u32 llength =
		dev->config.mi_config.dma.llength;
	u32 width =
		dev->config.mi_config.dma.output.width;
	u32 height =
		dev->config.mi_config.dma.output.height;
	u32 writeformat = CIF_ISP20_BUFF_FMT_PLANAR;
	u32 bpp = CIF_ISP20_PIX_FMT_GET_BPP(out_pix_fmt);
	u32 size = llength * height * bpp / 8;
	u32 output_format;
	u32 mi_ctrl;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s %dx%d, llength = %d\n",
		cif_isp20_pix_fmt_string(out_pix_fmt),
		width,
		height,
		llength);

	dev->config.mi_config.dma.y_size = size;
	dev->config.mi_config.dma.cb_size = 0;
	dev->config.mi_config.dma.cr_size = 0;
	if (CIF_ISP20_PIX_FMT_IS_YUV(out_pix_fmt)) {
		u32 num_cplanes =
			CIF_ISP20_PIX_FMT_YUV_GET_NUM_CPLANES(out_pix_fmt);
		if (num_cplanes == 0) {
			writeformat = CIF_ISP20_BUFF_FMT_INTERLEAVED;
		} else {
			dev->config.mi_config.dma.y_size =
				(dev->config.mi_config.dma.y_size * 4)
				/
				(4 + (CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(
					out_pix_fmt) *
				CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(
					out_pix_fmt) / 2));
			dev->config.mi_config.dma.cb_size =
				size -
				dev->config.mi_config.dma.y_size;
			if (num_cplanes == 1)
				writeformat = CIF_ISP20_BUFF_FMT_SEMIPLANAR;
			else if (num_cplanes == 2) {
				writeformat = CIF_ISP20_BUFF_FMT_PLANAR;
				dev->config.mi_config.dma.cb_size /= 2;
			}
			/* for U<->V swapping: */
			dev->config.mi_config.dma.cr_size =
				dev->config.mi_config.dma.cb_size;
		}

		if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt) == 0) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 0))
			output_format = CIF_MI_DMA_CTRL_FMT_YUV400;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 2))
			output_format = CIF_MI_DMA_CTRL_FMT_YUV420;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 4))
			output_format = CIF_MI_DMA_CTRL_FMT_YUV422;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt) == 4) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 4))
			output_format = CIF_MI_DMA_CTRL_FMT_YUV444;
		else {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"unsupported YUV output format %s\n",
				cif_isp20_pix_fmt_string(out_pix_fmt));
			ret = -EINVAL;
			goto err;
		}
	} else {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"unsupported output format %s\n",
			cif_isp20_pix_fmt_string(out_pix_fmt));
		ret = -EINVAL;
		goto err;
	}

	if (writeformat == CIF_ISP20_BUFF_FMT_SEMIPLANAR) {
		dev->config.mi_config.dma.cb_offs =
		    dev->config.mi_config.dma.y_size;
		dev->config.mi_config.dma.cr_offs =
		    dev->config.mi_config.dma.cb_offs;
	} else if (writeformat == CIF_ISP20_BUFF_FMT_PLANAR) {
		dev->config.mi_config.dma.cb_offs =
			dev->config.mi_config.dma.y_size;
		dev->config.mi_config.dma.cr_offs =
			dev->config.mi_config.dma.cb_offs +
			dev->config.mi_config.dma.cb_size;
	}

	cif_iowrite32_verify(dev->config.mi_config.dma.y_size,
		dev->config.base_addr + CIF_MI_DMA_Y_PIC_SIZE, ~0x3);
	cif_iowrite32_verify(width,
		dev->config.base_addr + CIF_MI_DMA_Y_PIC_WIDTH, ~0x3);
	cif_iowrite32_verify(llength,
		dev->config.base_addr + CIF_MI_DMA_Y_LLENGTH, ~0x3);

	mi_ctrl = cif_ioread32(dev->config.base_addr + CIF_MI_DMA_CTRL) |
		CIF_MI_DMA_CTRL_WRITE_FMT(writeformat) |
		output_format |
		CIF_MI_DMA_CTRL_BURST_LEN_LUM_64 |
		CIF_MI_DMA_CTRL_BURST_LEN_CHROM_64;
	cif_iowrite32_verify(mi_ctrl,
		dev->config.base_addr + CIF_MI_DMA_CTRL, ~0);

	cif_iowrite32OR_verify(CIF_MI_DMA_READY,
		dev->config.base_addr + CIF_MI_IMSC, ~0);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"\n  MI_DMA_CTRL 0x%08x\n"
		"  MI_DMA_STATUS 0x%08x\n"
		"  MI_DMA_Y_PIC_WIDTH %d\n"
		"  MI_DMA_Y_LLENGTH %d\n"
		"  MI_DMA_Y_PIC_SIZE %d\n"
		"  MI_DMA_Y_PIC_START_AD %d\n"
		"  MI_DMA_CB_PIC_START_AD %d\n"
		"  MI_DMA_CR_PIC_START_AD %d\n",
		cif_ioread32(dev->config.base_addr +
			CIF_MI_DMA_CTRL),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_DMA_STATUS),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_DMA_Y_PIC_WIDTH),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_DMA_Y_LLENGTH),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_DMA_Y_PIC_SIZE),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_DMA_Y_PIC_START_AD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_DMA_CB_PIC_START_AD),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_DMA_CR_PIC_START_AD));

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_config_jpeg_enc(
	struct cif_isp20_device *dev)
{
	struct cif_isp20_frm_fmt *inp_fmt =
		&dev->config.mp_config.rsz_config.output;
	dev->config.jpeg_config.input = inp_fmt;

	cif_isp20_pltfrm_pr_dbg(NULL,
		"%s %dx%d\n",
		cif_isp20_pix_fmt_string(inp_fmt->pix_fmt),
		inp_fmt->width, inp_fmt->height);

	/*
	   Reset JPEG-Encoder. In contrast to other software resets this
	   triggers the modules asynchronous reset resulting in loss of all data
	 */
	cif_iowrite32OR(CIF_IRCL_JPEG_SW_RST,
		dev->config.base_addr + CIF_IRCL);
	cif_iowrite32AND(~CIF_IRCL_JPEG_SW_RST,
		dev->config.base_addr + CIF_IRCL);

	cif_iowrite32(CIF_JPE_ERROR_MASK,
		dev->config.base_addr + CIF_JPE_ERROR_IMSC);

	/* Set configuration for the Jpeg capturing */
	cif_iowrite32(inp_fmt->width,
		dev->config.base_addr + CIF_JPE_ENC_HSIZE);
	cif_iowrite32(inp_fmt->height,
		dev->config.base_addr + CIF_JPE_ENC_VSIZE);

	if (dev->config.input_sel > CIF_ISP20_INP_CPI ||
		!CIF_ISP20_PIX_FMT_IS_RAW_BAYER(
		dev->config.isp_config.input->pix_fmt) ||
		cifisp_is_ie_active(&dev->isp_dev)) {
		/*
		upscaling of BT601 color space to full range 0..255
		The image effec block requires reduced range.
		TODO: input in full range from DMA or YUV sensor.
		*/
		cif_iowrite32(CIF_JPE_LUM_SCALE_ENABLE,
			dev->config.base_addr + CIF_JPE_Y_SCALE_EN);
		cif_iowrite32(CIF_JPE_CHROM_SCALE_ENABLE,
			dev->config.base_addr + CIF_JPE_CBCR_SCALE_EN);
	}

	switch (inp_fmt->pix_fmt) {
	case CIF_YUV422I:
	case CIF_YVU422I:
	case CIF_YUV422SP:
	case CIF_YVU422SP:
	case CIF_YUV422P:
	case CIF_YVU422P:
		cif_iowrite32(CIF_JPE_PIC_FORMAT_YUV422,
			dev->config.base_addr + CIF_JPE_PIC_FORMAT);
		break;
	case CIF_YUV400:
	case CIF_YVU400:
		cif_iowrite32(CIF_JPE_PIC_FORMAT_YUV400,
			dev->config.base_addr + CIF_JPE_PIC_FORMAT);
		break;
	default:
		cif_isp20_pltfrm_pr_err(NULL,
			"format %s not supported as input for JPEG encoder\n",
			cif_isp20_pix_fmt_string(inp_fmt->pix_fmt));
		BUG();
		break;
	}

	/* Set to normal operation (wait for encoded image data
	to fill output buffer) */
	cif_iowrite32(0, dev->config.base_addr + CIF_JPE_TABLE_FLUSH);

	/*
	   CIF Spec 4.7
	   3.14 JPEG Encoder Programming
	   Do not forget to re-program all AC and DC tables
	   after system reset as well as after
	   module software reset because after any reset
	   the internal RAM is filled with FFH which
	   is an illegal symbol. This filling takes
	   approximately 400 clock cycles. So do not start
	   any table programming during the first 400 clock
	   cycles after reset is de-asserted.
	   Note: depends on CIF clock setting
	   400 clock cycles at 312 Mhz CIF clock-> 1.3 us
	   400 clock cycles at 208 Mhz CIF clock-> 1.93 us
	   -> 2us ok for both
	 */
	udelay(2);

	/* Program JPEG tables */
	cif_isp20_program_jpeg_tables(dev);
	/* Select JPEG tables */
	cif_isp20_select_jpeg_tables(dev);

	switch (dev->config.jpeg_config.header) {
	case CIF_ISP20_JPEG_HEADER_JFIF:
		cif_isp20_pltfrm_pr_dbg(NULL,
			"generate JFIF header\n");
		cif_iowrite32(CIF_JPE_HEADER_MODE_JFIF,
			dev->config.base_addr +
			CIF_JPE_HEADER_MODE);
		break;
	case CIF_ISP20_JPEG_HEADER_NONE:
		cif_isp20_pltfrm_pr_dbg(NULL,
			"generate no JPEG header\n");
		cif_iowrite32(CIF_JPE_HEADER_MODE_NOAPPN,
			dev->config.base_addr +
			CIF_JPE_HEADER_MODE);
		break;
	default:
		cif_isp20_pltfrm_pr_err(NULL,
			"unkown/unsupport JPEG header type %d\n",
			dev->config.jpeg_config.header);
		BUG();
		break;
	}

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"\n  JPE_PIC_FORMAT 0x%08x\n"
		"  JPE_ENC_HSIZE %d\n"
		"  JPE_ENC_VSIZE %d\n"
		"  JPE_Y_SCALE_EN 0x%08x\n"
		"  JPE_CBCR_SCALE_EN 0x%08x\n"
		"  JPE_ERROR_RIS 0x%08x\n"
		"  JPE_ERROR_IMSC 0x%08x\n"
		"  JPE_STATUS_RIS 0x%08x\n"
		"  JPE_STATUS_IMSC 0x%08x\n"
		"  JPE_DEBUG 0x%08x\n",
		cif_ioread32(dev->config.base_addr + CIF_JPE_PIC_FORMAT),
		cif_ioread32(dev->config.base_addr + CIF_JPE_ENC_HSIZE),
		cif_ioread32(dev->config.base_addr + CIF_JPE_ENC_VSIZE),
		cif_ioread32(dev->config.base_addr + CIF_JPE_Y_SCALE_EN),
		cif_ioread32(dev->config.base_addr + CIF_JPE_CBCR_SCALE_EN),
		cif_ioread32(dev->config.base_addr + CIF_JPE_ERROR_RIS),
		cif_ioread32(dev->config.base_addr + CIF_JPE_ERROR_IMSC),
		cif_ioread32(dev->config.base_addr + CIF_JPE_STATUS_RIS),
		cif_ioread32(dev->config.base_addr + CIF_JPE_STATUS_IMSC),
		cif_ioread32(dev->config.base_addr + CIF_JPE_DEBUG));

	return 0;
}

static int cif_isp20_config_path(
	struct cif_isp20_device *dev,
	u32 stream_ids)
{
	u32 dpcl = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "\n");

	/* if_sel */
	if (dev->config.input_sel == CIF_ISP20_INP_DMA)
		dpcl |= CIF_VI_DPCL_IF_SEL_DMA | CIF_VI_DPCL_DMA_SW_ISP;
	else if (dev->config.input_sel == CIF_ISP20_INP_DMA_IE)
		dpcl |= CIF_VI_DPCL_IF_SEL_DMA | CIF_VI_DPCL_DMA_IE_MUX_DMA |
			CIF_VI_DPCL_DMA_SW_IE;
	else if (dev->config.input_sel == CIF_ISP20_INP_DMA_SP)
		dpcl |= CIF_VI_DPCL_DMA_SP_MUX_DMA;
	else {
		if (dev->config.input_sel < CIF_ISP20_INP_CPI)
			dpcl |= CIF_VI_DPCL_IF_SEL_MIPI;
		else
			dpcl |= CIF_VI_DPCL_IF_SEL_PARALLEL;
		if (dev->config.isp_config.si_enable)
			dpcl |= CIF_VI_DPCL_DMA_SW_SI;
	}

	/* chan_mode */
	if (stream_ids & CIF_ISP20_STREAM_SP) {
		dpcl |= CIF_VI_DPCL_CHAN_MODE_SP;
		if (dev->config.sp_config.inp_yc_filt)
			dpcl |= CIF_VI_DPCL_YC_SPMUX_FILT;
	}

	if ((stream_ids & CIF_ISP20_STREAM_MP) &&
		!(dev->config.input_sel == CIF_ISP20_INP_DMA_SP)) {
			dpcl |= CIF_VI_DPCL_CHAN_MODE_MP;
		/* mp_dmux */
		if (CIF_ISP20_PIX_FMT_IS_JPEG(
			dev->config.mi_config.mp.output.pix_fmt))
			dpcl |= CIF_VI_DPCL_MP_MUX_MRSZ_JPEG;
		else
			dpcl |= CIF_VI_DPCL_MP_MUX_MRSZ_MI;
	} else if (dpcl & CIF_VI_DPCL_YC_SPMUX_FILT)
		/* if SP mux is set to YC flt input also MP channel
			mode has to be configured in DPCL otherwsie
			the SP path will not produce frame end interrupts */
		dpcl |= CIF_VI_DPCL_MP_MUX_MRSZ_MI |
			CIF_VI_DPCL_CHAN_MODE_MP;

	cif_iowrite32(dpcl,
		dev->config.base_addr + CIF_VI_DPCL);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"CIF_DPCL 0x%08x\n", dpcl);

	return 0;
}

int cif_isp20_config_rsz(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream_id,
	bool async)
{
	int ret;
	u32 i;
	CIF_ISP20_PLTFRM_MEM_IO_ADDR scale_h_y_addr =
		dev->config.base_addr;
	CIF_ISP20_PLTFRM_MEM_IO_ADDR scale_h_cr_addr =
		dev->config.base_addr;
	CIF_ISP20_PLTFRM_MEM_IO_ADDR scale_h_cb_addr =
		dev->config.base_addr;
	CIF_ISP20_PLTFRM_MEM_IO_ADDR scale_v_y_addr =
		dev->config.base_addr;
	CIF_ISP20_PLTFRM_MEM_IO_ADDR scale_v_c_addr =
		dev->config.base_addr;
	CIF_ISP20_PLTFRM_MEM_IO_ADDR rsz_ctrl_addr =
		dev->config.base_addr;
	struct cif_isp20_frm_fmt *rsz_input;
	struct cif_isp20_frm_fmt *rsz_output;
	struct cif_isp20_frm_fmt *mi_output;
	u32 rsz_ctrl;
	u32 input_width_y;
	u32 output_width_y;
	u32 input_height_y;
	u32 output_height_y;
	u32 input_width_c;
	u32 output_width_c;
	u32 input_height_c;
	u32 output_height_c;
	u32 scale_h_c;
	bool inp_yc_filt = dev->isp_dev.ycflt_en;

	if (stream_id == CIF_ISP20_STREAM_MP) {
		rsz_ctrl_addr += CIF_MRSZ_CTRL;
		scale_h_y_addr += CIF_MRSZ_SCALE_HY;
		scale_v_y_addr += CIF_MRSZ_SCALE_VY;
		scale_h_cb_addr += CIF_MRSZ_SCALE_HCB;
		scale_h_cr_addr += CIF_MRSZ_SCALE_HCR;
		scale_v_c_addr += CIF_MRSZ_SCALE_VC;
		dev->config.mp_config.rsz_config.input =
			&dev->config.isp_config.output;
		rsz_input = dev->config.mp_config.rsz_config.input;
		rsz_output = &dev->config.mp_config.rsz_config.output;
		mi_output = &dev->config.mi_config.mp.output;
		/* No phase offset */
		cif_iowrite32(0, dev->config.base_addr + CIF_MRSZ_PHASE_HY);
		cif_iowrite32(0, dev->config.base_addr + CIF_MRSZ_PHASE_HC);
		cif_iowrite32(0, dev->config.base_addr + CIF_MRSZ_PHASE_VY);
		cif_iowrite32(0, dev->config.base_addr + CIF_MRSZ_PHASE_VC);
		/* Linear interpolation */
		for (i = 0; i < 64; i++) {
			cif_iowrite32(i,
				dev->config.base_addr +
					CIF_MRSZ_SCALE_LUT_ADDR);
			cif_iowrite32(i,
				dev->config.base_addr +
					CIF_MRSZ_SCALE_LUT);
		}
	} else {
		rsz_ctrl_addr += CIF_SRSZ_CTRL;
		scale_h_y_addr += CIF_SRSZ_SCALE_HY;
		scale_v_y_addr += CIF_SRSZ_SCALE_VY;
		scale_h_cb_addr += CIF_SRSZ_SCALE_HCB;
		scale_h_cr_addr += CIF_SRSZ_SCALE_HCR;
		scale_v_c_addr += CIF_SRSZ_SCALE_VC;
		if (dev->config.input_sel == CIF_ISP20_INP_DMA_SP)
			dev->config.sp_config.rsz_config.input =
				&dev->config.mi_config.dma.output;
		else
			dev->config.sp_config.rsz_config.input =
				&dev->config.isp_config.output;
		inp_yc_filt = inp_yc_filt &&
			dev->config.sp_config.inp_yc_filt;
		rsz_input = dev->config.sp_config.rsz_config.input;
		rsz_output = &dev->config.sp_config.rsz_config.output;
		mi_output = &dev->config.mi_config.sp.output;
		/* No phase offset */
		cif_iowrite32(0, dev->config.base_addr + CIF_SRSZ_PHASE_HY);
		cif_iowrite32(0, dev->config.base_addr + CIF_SRSZ_PHASE_HC);
		cif_iowrite32(0, dev->config.base_addr + CIF_SRSZ_PHASE_VY);
		cif_iowrite32(0, dev->config.base_addr + CIF_SRSZ_PHASE_VC);
		/* Linear interpolation */
		for (i = 0; i < 64; i++) {
			cif_iowrite32(i,
				dev->config.base_addr +
					CIF_SRSZ_SCALE_LUT_ADDR);
			cif_iowrite32(i,
				dev->config.base_addr +
					CIF_SRSZ_SCALE_LUT);
		}
	}

	/* set RSZ input and output */
	rsz_output->width = mi_output->width;
	rsz_output->height = mi_output->height;
	rsz_output->pix_fmt = rsz_input->pix_fmt;
	if (CIF_ISP20_PIX_FMT_IS_YUV(mi_output->pix_fmt)) {
		cif_isp20_pix_fmt_set_y_subs(
			rsz_output->pix_fmt,
			CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(mi_output->pix_fmt));
		cif_isp20_pix_fmt_set_x_subs(
			rsz_output->pix_fmt,
			CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(mi_output->pix_fmt));
		cif_isp20_pix_fmt_set_bpp(
			rsz_output->pix_fmt,
			CIF_ISP20_PIX_FMT_GET_BPP(mi_output->pix_fmt));
	} else if (CIF_ISP20_PIX_FMT_IS_JPEG(mi_output->pix_fmt)) {
		cif_isp20_pix_fmt_set_y_subs(
			rsz_output->pix_fmt, 4);
		cif_isp20_pix_fmt_set_x_subs(
			rsz_output->pix_fmt, 2);
		cif_isp20_pix_fmt_set_bpp(
			rsz_output->pix_fmt, 16);
	}

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s %s %dx%d -> %s %dx%d\n",
		cif_isp20_stream_id_string(stream_id),
		cif_isp20_pix_fmt_string(rsz_input->pix_fmt),
		rsz_input->width,
		rsz_input->height,
		cif_isp20_pix_fmt_string(rsz_output->pix_fmt),
		rsz_output->width,
		rsz_output->height);

	/* set input and output sizes for scale calculation */
	input_width_y = rsz_input->width;
	output_width_y = rsz_output->width;
	input_height_y = rsz_input->height;
	output_height_y = rsz_output->height;
	input_width_c = input_width_y;
	output_width_c = output_width_y;
	input_height_c = input_height_y;
	output_height_c = output_height_y;

	if (CIF_ISP20_PIX_FMT_IS_YUV(rsz_output->pix_fmt)) {
		input_width_c = (input_width_c *
			CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(
				rsz_input->pix_fmt)) / 4;
		input_height_c = (input_height_c *
			CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(
				rsz_input->pix_fmt)) / 4;
		output_width_c = (output_width_c *
			CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(
				rsz_output->pix_fmt)) / 4;
		output_height_c = (output_height_c *
			CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(
				rsz_output->pix_fmt)) / 4;

		if (inp_yc_filt)
			cif_isp20_rsz_ycflt_adjust(dev,
				&input_width_c, &input_height_c);
		cif_isp20_pltfrm_pr_dbg(NULL,
			"chroma scaling %dx%d -> %dx%d\n",
			input_width_c, input_height_c,
			output_width_c, output_height_c);

		if (((input_width_c == 0) && (output_width_c > 0)) ||
			((input_height_c == 0) && (output_height_c > 0))) {
			cif_isp20_pltfrm_pr_err(NULL,
				"input is black and white, cannot output colour\n");
			ret = -EINVAL;
			goto err;
		}
	} else {
		if ((input_width_y != output_width_y) ||
			(input_height_y != output_height_y)) {
			cif_isp20_pltfrm_pr_err(NULL,
				"can only scale YUV input\n");
			ret = -EINVAL;
			goto err;
		}
	}

	/* calculate and set scale */
	rsz_ctrl = 0;
	if (input_width_y < output_width_y) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_HY_ENABLE |
			CIF_RSZ_CTRL_SCALE_HY_UP;
		cif_iowrite32(
			DIV_TRUNCATE((input_width_y - 1) * 16384,
				output_width_y - 1),
			scale_h_y_addr);
	} else if (input_width_y > output_width_y) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_HY_ENABLE;
		cif_iowrite32(
			DIV_TRUNCATE((output_width_y - 1) * 16384,
				input_width_y - 1) + 1,
			scale_h_y_addr);
	}
	if (input_width_c < output_width_c) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_HC_ENABLE |
			CIF_RSZ_CTRL_SCALE_HC_UP;
		scale_h_c = DIV_TRUNCATE((input_width_c - 1) * 16384,
			output_width_c - 1);
		cif_iowrite32(scale_h_c, scale_h_cb_addr);
		cif_iowrite32(scale_h_c, scale_h_cr_addr);
	} else if (input_width_c > output_width_c) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_HC_ENABLE;
		scale_h_c = DIV_TRUNCATE((output_width_c - 1) * 16384,
			input_width_c - 1) + 1;
		cif_iowrite32(scale_h_c, scale_h_cb_addr);
		cif_iowrite32(scale_h_c, scale_h_cr_addr);
	}

	if (input_height_y < output_height_y) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_VY_ENABLE |
			CIF_RSZ_CTRL_SCALE_VY_UP;
		cif_iowrite32(
			DIV_TRUNCATE((input_height_y - 1) * 16384,
				output_height_y - 1),
			scale_v_y_addr);
	} else if (input_height_y > output_height_y) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_VY_ENABLE;
		cif_iowrite32(
			DIV_TRUNCATE((output_height_y - 1) * 16384,
				input_height_y - 1) + 1,
			scale_v_y_addr);
	}

	if (input_height_c < output_height_c) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_VC_ENABLE |
			CIF_RSZ_CTRL_SCALE_VC_UP;
		cif_iowrite32(
			DIV_TRUNCATE((input_height_c - 1) * 16384,
				output_height_c - 1),
			scale_v_c_addr);
	} else if (input_height_c > output_height_c) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_VC_ENABLE;
		cif_iowrite32(
			DIV_TRUNCATE((output_height_c - 1) * 16384,
				input_height_c - 1) + 1,
			scale_v_c_addr);
	}

	cif_iowrite32(rsz_ctrl, rsz_ctrl_addr);

	if (stream_id == CIF_ISP20_STREAM_MP) {
		if (async)
			cif_iowrite32OR(CIF_RSZ_CTRL_CFG_UPD,
				dev->config.base_addr + CIF_MRSZ_CTRL);
		dev->config.mp_config.rsz_config.ycflt_adjust = false;
		dev->config.mp_config.rsz_config.ism_adjust = false;
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"\n  MRSZ_CTRL 0x%08x/0x%08x\n"
			"  MRSZ_SCALE_HY %d/%d\n"
			"  MRSZ_SCALE_HCB %d/%d\n"
			"  MRSZ_SCALE_HCR %d/%d\n"
			"  MRSZ_SCALE_VY %d/%d\n"
			"  MRSZ_SCALE_VC %d/%d\n"
			"  MRSZ_PHASE_HY %d/%d\n"
			"  MRSZ_PHASE_HC %d/%d\n"
			"  MRSZ_PHASE_VY %d/%d\n"
			"  MRSZ_PHASE_VC %d/%d\n",
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_CTRL),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_CTRL_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_SCALE_HY),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_SCALE_HY_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_SCALE_HCB),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_SCALE_HCB_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_SCALE_HCR),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_SCALE_HCR_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_SCALE_VY),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_SCALE_VY_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_SCALE_VC),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_SCALE_VC_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_PHASE_HY),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_PHASE_HY_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_PHASE_HC),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_PHASE_HC_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_PHASE_VY),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_PHASE_VY_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_PHASE_VC),
			cif_ioread32(dev->config.base_addr +
				CIF_MRSZ_PHASE_VC_SHD));
	} else {
		if (async)
			cif_iowrite32OR(CIF_RSZ_CTRL_CFG_UPD,
				dev->config.base_addr + CIF_SRSZ_CTRL);
		dev->config.sp_config.rsz_config.ycflt_adjust = false;
		dev->config.sp_config.rsz_config.ism_adjust = false;
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"\n  SRSZ_CTRL 0x%08x/0x%08x\n"
			"  SRSZ_SCALE_HY %d/%d\n"
			"  SRSZ_SCALE_HCB %d/%d\n"
			"  SRSZ_SCALE_HCR %d/%d\n"
			"  SRSZ_SCALE_VY %d/%d\n"
			"  SRSZ_SCALE_VC %d/%d\n"
			"  SRSZ_PHASE_HY %d/%d\n"
			"  SRSZ_PHASE_HC %d/%d\n"
			"  SRSZ_PHASE_VY %d/%d\n"
			"  SRSZ_PHASE_VC %d/%d\n",
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_CTRL),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_CTRL_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_SCALE_HY),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_SCALE_HY_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_SCALE_HCB),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_SCALE_HCB_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_SCALE_HCR),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_SCALE_HCR_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_SCALE_VY),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_SCALE_VY_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_SCALE_VC),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_SCALE_VC_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_PHASE_HY),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_PHASE_HY_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_PHASE_HC),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_PHASE_HC_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_PHASE_VY),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_PHASE_VY_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_PHASE_VC),
			cif_ioread32(dev->config.base_addr +
				CIF_SRSZ_PHASE_VC_SHD));
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
}

static int cif_isp20_config_sp(
	struct cif_isp20_device *dev)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(NULL, "\n");

	ret = cif_isp20_config_rsz(dev, CIF_ISP20_STREAM_SP, true);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_isp20_config_mi_sp(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	dev->sp_stream.updt_cfg = false;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_config_mp(
	struct cif_isp20_device *dev)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(NULL, "\n");

	ret = cif_isp20_config_rsz(dev, CIF_ISP20_STREAM_MP, true);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_isp20_config_mi_mp(dev);
	if (IS_ERR_VALUE(ret))
		goto err;
	if (CIF_ISP20_PIX_FMT_IS_JPEG(
		dev->config.mi_config.mp.output.pix_fmt)) {
		ret = cif_isp20_config_jpeg_enc(dev);
		if (IS_ERR_VALUE(ret))
			goto err;
		dev->config.jpeg_config.busy = false;
	}

	dev->mp_stream.updt_cfg = false;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static void cif_isp20_config_clk(
	struct cif_isp20_device *dev)
{
	cif_iowrite32(CIF_CCL_CIF_CLK_ENA,
		dev->config.base_addr + CIF_CCL);
	cif_iowrite32(0x0000187B, dev->config.base_addr + CIF_ICCL);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"\n  CIF_CCL 0x%08x\n"
		"  CIF_ICCL 0x%08x\n",
		cif_ioread32(dev->config.base_addr + CIF_CCL),
		cif_ioread32(dev->config.base_addr + CIF_ICCL));
}

static int cif_isp20_config_cif(
	struct cif_isp20_device *dev,
	u32 stream_ids)
{
	int ret = 0;
	u32 cif_id;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"config MP = %d, config SP = %d, img_src state = %s, PM state = %s, SP state = %s, MP state = %s\n",
		(stream_ids & CIF_ISP20_STREAM_MP) == CIF_ISP20_STREAM_MP,
		(stream_ids & CIF_ISP20_STREAM_SP) == CIF_ISP20_STREAM_SP,
		cif_isp20_img_src_state_string(dev->img_src_state),
		cif_isp20_pm_state_string(dev->pm_state),
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state));

	cif_isp20_pltfrm_rtrace_printf(NULL,
		"start configuring CIF...\n");

	if ((stream_ids & CIF_ISP20_STREAM_MP) ||
		(stream_ids & CIF_ISP20_STREAM_SP)) {
		if (dev->config.input_sel < CIF_ISP20_INP_DMA) {
			/* configure sensor */
			ret = cif_isp20_config_img_src(dev);
			if (IS_ERR_VALUE(ret))
				goto err;
		}

		ret = cif_isp20_set_pm_state(dev,
			CIF_ISP20_PM_STATE_SW_STNDBY);
		if (IS_ERR_VALUE(ret))
			goto err;

		cif_id = cif_ioread32(dev->config.base_addr + CIF_VI_ID);
		if ((cif_id & 0xffff) >= 0x4C0)
			dev->config.out_of_buffer_stall =
				CIF_ISP20_ALWAYS_STALL_ON_NO_BUFS;
		else
			dev->config.out_of_buffer_stall = true;

		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"CIF_ID 0x%08x\n", cif_id);

		cif_iowrite32(CIF_IRCL_CIF_SW_RST,
			dev->config.base_addr + CIF_IRCL);

		cif_isp20_config_clk(dev);

#ifdef CONFIG_CIF_ISP20_TEST_YC_FLT
		cif_isp20_enable_yc_flt(dev);
#endif

		/* Decide when to switch to asynchronous mode */
		/* TODO: remove dev->isp_dev.ycflt_en check for
			HW with the scaler fix. */
		dev->config.mi_config.async_updt = CIF_ISP20_ALWAYS_ASYNC;
		if (dev->config.input_sel > CIF_ISP20_INP_CPI) {
			dev->config.mi_config.async_updt |= CIF_ISP20_ASYNC_DMA;
			ret = cif_isp20_config_mi_dma(dev);
			if (IS_ERR_VALUE(ret))
				goto err;
		}
		if ((stream_ids & CIF_ISP20_STREAM_MP) &&
			CIF_ISP20_PIX_FMT_IS_JPEG(
				dev->config.mi_config.mp.output.pix_fmt))
			dev->config.mi_config.async_updt |=
				CIF_ISP20_ASYNC_JPEG;
		if (dev->isp_dev.ycflt_en &&
			(dev->isp_dev.ycflt_config.chr_ss_ctrl & 0x3))
			dev->config.mi_config.async_updt |=
				CIF_ISP20_ASYNC_YCFLT;
		if (dev->config.isp_config.ism_config.ism_en)
			dev->config.mi_config.async_updt |=
				CIF_ISP20_ASYNC_ISM;

		ret = cif_isp20_config_mipi(dev);
		if (IS_ERR_VALUE(ret))
			goto err;

		ret = cif_isp20_config_isp(dev);
		if (IS_ERR_VALUE(ret))
			goto err;

		cif_isp20_config_ism(dev, true);
		dev->config.isp_config.ism_config.ism_update_needed = false;
		if (stream_ids & CIF_ISP20_STREAM_SP)
			dev->config.sp_config.rsz_config.ism_adjust = true;
		if (stream_ids & CIF_ISP20_STREAM_MP)
			dev->config.mp_config.rsz_config.ism_adjust = true;

		/* YC filter enabled in secondary path causes sync fifo
			overflows for interleaved output */
		if ((!dev->isp_dev.ycflt_en) ||
		     ((stream_ids & CIF_ISP20_STREAM_SP) &&
			CIF_ISP20_PIX_FMT_IS_INTERLEAVED(
			dev->config.mi_config.sp.output.pix_fmt)) ||
		     (dev->config.input_sel == CIF_ISP20_INP_DMA_SP))
			dev->config.sp_config.inp_yc_filt = false;
		else
			dev->config.sp_config.inp_yc_filt =
				CIF_ISP20_SP_YCFLT_INP;

		if (stream_ids & CIF_ISP20_STREAM_SP) {
			ret = cif_isp20_config_sp(dev);
			if (IS_ERR_VALUE(ret))
				goto err;
		}
		if (stream_ids & CIF_ISP20_STREAM_MP) {
			ret = cif_isp20_config_mp(dev);
			if (IS_ERR_VALUE(ret))
				goto err;
		}
		ret = cif_isp20_config_path(dev, stream_ids);
		if (IS_ERR_VALUE(ret))
			goto err;
	}

	/* Turn off XNR vertical subsampling when ism cropping is enabled */
	if (dev->config.isp_config.ism_config.ism_en) {
		if (dev->isp_dev.cif_ism_cropping == false) {
			dev->isp_dev.cif_ism_cropping = true;
			dev->isp_dev.ycflt_update = true;
		}
	} else {
		if (dev->isp_dev.cif_ism_cropping == true) {
			dev->isp_dev.cif_ism_cropping = false;
			dev->isp_dev.ycflt_update = true;
		}
	}

	if ((dev->sp_stream.state != CIF_ISP20_STATE_STREAMING) &&
		(dev->mp_stream.state != CIF_ISP20_STATE_STREAMING) &&
		dev->isp_dev.ycflt_update) {

		if (dev->isp_dev.ycflt_en) {
			cifisp_ycflt_config(&dev->isp_dev);
			cifisp_ycflt_en(&dev->isp_dev);
		} else
			cifisp_ycflt_end(&dev->isp_dev);
		dev->isp_dev.ycflt_update = false;
		dev->config.mp_config.rsz_config.ycflt_adjust = true;
		if (dev->config.sp_config.inp_yc_filt)
			dev->config.sp_config.rsz_config.ycflt_adjust = true;
	}

	if (dev->config.sp_config.rsz_config.ycflt_adjust ||
		dev->config.sp_config.rsz_config.ism_adjust) {
		if (dev->sp_stream.state == CIF_ISP20_STATE_READY) {
			ret = cif_isp20_config_rsz(dev,
				CIF_ISP20_STREAM_SP, true);
			if (IS_ERR_VALUE(ret))
				goto err;
		} else {
			/* Disable SRSZ if SP is not used */
			cif_iowrite32(0, dev->config.base_addr + CIF_SRSZ_CTRL);
			cif_iowrite32OR(CIF_RSZ_CTRL_CFG_UPD,
				dev->config.base_addr + CIF_SRSZ_CTRL);
			dev->config.sp_config.rsz_config.ycflt_adjust = false;
			dev->config.sp_config.rsz_config.ism_adjust = false;
		}
	}

	if (dev->config.mp_config.rsz_config.ycflt_adjust ||
		dev->config.mp_config.rsz_config.ism_adjust) {
		if (dev->mp_stream.state == CIF_ISP20_STATE_READY) {
			ret = cif_isp20_config_rsz(dev,
				CIF_ISP20_STREAM_MP, true);
			if (IS_ERR_VALUE(ret))
				goto err;
		} else {
			/* Disable MRSZ if MP is not used */
			cif_iowrite32(0, dev->config.base_addr + CIF_MRSZ_CTRL);
			cif_iowrite32OR(CIF_RSZ_CTRL_CFG_UPD,
				dev->config.base_addr + CIF_MRSZ_CTRL);
			dev->config.mp_config.rsz_config.ycflt_adjust = false;
			dev->config.mp_config.rsz_config.ism_adjust = false;
		}
	}

	if (dev->config.mi_config.async_updt)
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"CIF in asynchronous mode (0x%08x)\n",
			dev->config.mi_config.async_updt);

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static void cif_isp20_init_stream(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream_id)
{
	struct cif_isp20_stream *stream;

	switch (stream_id) {
	case CIF_ISP20_STREAM_SP:
		stream = &dev->sp_stream;
		dev->config.sp_config.rsz_config.ycflt_adjust = false;
		dev->config.sp_config.rsz_config.ism_adjust = false;
		dev->config.mi_config.sp.busy = false;
		dev->config.mi_config.sp.output.pix_fmt = CIF_UNKNOWN_FORMAT;
		break;
	case CIF_ISP20_STREAM_MP:
		stream = &dev->mp_stream;
		dev->config.jpeg_config.ratio = 50;
		dev->config.jpeg_config.header =
			CIF_ISP20_JPEG_HEADER_JFIF;
		dev->config.mp_config.rsz_config.ycflt_adjust = false;
		dev->config.mp_config.rsz_config.ism_adjust = false;
		dev->config.mi_config.mp.busy = false;
		dev->config.mi_config.mp.output.pix_fmt = CIF_UNKNOWN_FORMAT;
		break;
	case CIF_ISP20_STREAM_DMA:
		stream = &dev->dma_stream;
		dev->config.mi_config.dma.busy = false;
		dev->config.mi_config.dma.output.pix_fmt = CIF_UNKNOWN_FORMAT;
		break;
	default:
		cif_isp20_pltfrm_pr_err(NULL,
			"unknown/unsupported stream ID %d\n", stream_id);
		BUG();
		break;
	}

	INIT_LIST_HEAD(&stream->buf_queue);
	stream->next_buf = NULL;
	stream->curr_buf = NULL;
	stream->updt_cfg = false;
	stream->stop = false;
	stream->stall = false;

	cif_isp20_pltfrm_event_clear(dev->dev, &stream->done);
	stream->state = CIF_ISP20_STATE_INACTIVE;
}

static int cif_isp20_jpeg_gen_header(
	struct cif_isp20_device *dev)
{
	unsigned int timeout = 10000;

	cif_isp20_pltfrm_pr_dbg(NULL, "\n");

	cif_iowrite32(CIF_JPE_GEN_HEADER_ENABLE,
		dev->config.base_addr + CIF_JPE_GEN_HEADER);

	while (timeout--) {
		if (cif_ioread32(dev->config.base_addr +
			CIF_JPE_STATUS_RIS) &
			CIF_JPE_STATUS_GENHEADER_DONE) {
			cif_isp20_pltfrm_pr_dbg(NULL,
				"JPEG header generated\n");
			cif_iowrite32(CIF_JPE_STATUS_GENHEADER_DONE,
				dev->config.base_addr + CIF_JPE_STATUS_ICR);
			break;
		}
	}

	if (!timeout) {
		cif_isp20_pltfrm_pr_err(NULL,
			"JPEG header generation timeout\n");
		cif_isp20_pltfrm_pr_err(NULL,
			"failed with error %d\n", -ETIMEDOUT);
		return -ETIMEDOUT;
	}

#ifdef CIF_ISP20_VERIFY_JPEG_HEADER
	{
		u32 *buff = (u32 *)phys_to_virt(
			dev->config.mi_config.mp.curr_buff_addr);
		if (buff[0] != 0xe0ffd8ff)
			cif_isp20_pltfrm_pr_err(NULL,
				"JPEG HEADER WRONG: 0x%08x\n"
				"curr_buff_addr 0x%08x\n"
				"MI_MP_Y_SIZE_SHD 0x%08x\n"
				"MI_MP_Y_BASE_AD_SHD 0x%08x\n",
				buff[0],
				dev->config.mi_config.mp.curr_buff_addr,
				cif_ioread32(dev->config.base_addr +
					CIF_MI_MP_Y_SIZE_SHD),
				cif_ioread32(dev->config.base_addr +
					CIF_MI_MP_Y_BASE_AD_SHD));
	}
#endif

	return 0;
}

static void cif_isp20_mi_update_buff_addr(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id strm_id)
{
	if (strm_id == CIF_ISP20_STREAM_SP) {
		if (dev->config.mi_config.sp.next_buff_addr ==
			CIF_ISP20_INVALID_BUFF_ADDR) {
			return;
		}
		cif_iowrite32_verify(dev->config.mi_config.sp.next_buff_addr,
			dev->config.base_addr +
			CIF_MI_SP_Y_BASE_AD_INIT, ~0x3);
		cif_iowrite32_verify(dev->config.mi_config.sp.next_buff_addr +
			dev->config.mi_config.sp.cb_offs,
			dev->config.base_addr +
			CIF_MI_SP_CB_BASE_AD_INIT, ~0x3);
		cif_iowrite32_verify(dev->config.mi_config.sp.next_buff_addr +
			dev->config.mi_config.sp.cr_offs,
			dev->config.base_addr +
			CIF_MI_SP_CR_BASE_AD_INIT, ~0x3);
		/* There have bee repeatedly issues with
			the offset registers, it is safer to write
			them each time, even though it is always
			0 and even though that is the
			register's default value */
		cif_iowrite32_verify(0,
			dev->config.base_addr +
			CIF_MI_SP_Y_OFFS_CNT_INIT, ~0x3);
		cif_iowrite32_verify(0,
			dev->config.base_addr +
			CIF_MI_SP_CB_OFFS_CNT_INIT, ~0x3);
		cif_iowrite32_verify(0,
			dev->config.base_addr +
			CIF_MI_SP_CR_OFFS_CNT_INIT, ~0x3);
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"\n  MI_SP_Y_BASE_AD 0x%08x/0x%08x\n"
			"  MI_SP_CB_BASE_AD 0x%08x/0x%08x\n"
			"  MI_SP_CR_BASE_AD 0x%08x/0x%08x\n",
			cif_ioread32(dev->config.base_addr +
				CIF_MI_SP_Y_BASE_AD_INIT),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_SP_Y_BASE_AD_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_SP_CB_BASE_AD_INIT),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_SP_CB_BASE_AD_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_SP_CR_BASE_AD_INIT),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_SP_CR_BASE_AD_SHD));
	} else if (strm_id == CIF_ISP20_STREAM_MP) {
		if (dev->config.mi_config.mp.next_buff_addr ==
			CIF_ISP20_INVALID_BUFF_ADDR) {
			return;
		}
		cif_iowrite32_verify(dev->config.mi_config.mp.next_buff_addr,
			dev->config.base_addr +
			CIF_MI_MP_Y_BASE_AD_INIT, ~0x3);
		cif_iowrite32_verify(dev->config.mi_config.mp.next_buff_addr +
			dev->config.mi_config.mp.cb_offs,
			dev->config.base_addr +
			CIF_MI_MP_CB_BASE_AD_INIT, ~0x3);
		cif_iowrite32_verify(dev->config.mi_config.mp.next_buff_addr +
			dev->config.mi_config.mp.cr_offs,
			dev->config.base_addr +
			CIF_MI_MP_CR_BASE_AD_INIT, ~0x3);
		/* There have bee repeatedly issues with
			the offset registers, it is safer to write
			them each time, even though it is always
			0 and even though that is the
			register's default value */
		cif_iowrite32_verify(0,
			dev->config.base_addr +
			CIF_MI_MP_Y_OFFS_CNT_INIT, ~0x3);
		cif_iowrite32_verify(0,
			dev->config.base_addr +
			CIF_MI_MP_CB_OFFS_CNT_INIT, ~0x3);
		cif_iowrite32_verify(0,
			dev->config.base_addr +
			CIF_MI_MP_CR_OFFS_CNT_INIT, ~0x3);
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"\n  MI_MP_Y_BASE_AD 0x%08x/0x%08x\n"
			"  MI_MP_CB_BASE_AD 0x%08x/0x%08x\n"
			"  MI_MP_CR_BASE_AD 0x%08x/0x%08x\n",
			cif_ioread32(dev->config.base_addr +
				CIF_MI_MP_Y_BASE_AD_INIT),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_MP_Y_BASE_AD_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_MP_CB_BASE_AD_INIT),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_MP_CB_BASE_AD_SHD),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_MP_CR_BASE_AD_INIT),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_MP_CR_BASE_AD_SHD));
	} else { /* DMA */
		cif_iowrite32_verify(dev->config.mi_config.dma.next_buff_addr,
			dev->config.base_addr +
			CIF_MI_DMA_Y_PIC_START_AD, ~0x3);
		cif_iowrite32_verify(dev->config.mi_config.dma.next_buff_addr +
			dev->config.mi_config.dma.cb_offs,
			dev->config.base_addr +
			CIF_MI_DMA_CB_PIC_START_AD, ~0x3);
		cif_iowrite32_verify(dev->config.mi_config.dma.next_buff_addr +
			dev->config.mi_config.dma.cr_offs,
			dev->config.base_addr +
			CIF_MI_DMA_CR_PIC_START_AD, ~0x3);
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"\n  MI_DMA_Y_PIC_START_AD 0x%08x\n"
			"  MI_DMA_CB_PIC_START_AD 0x%08x\n"
			"  MI_DMA_CR_PIC_START_AD 0x%08x\n",
			cif_ioread32(dev->config.base_addr +
				CIF_MI_DMA_Y_PIC_START_AD),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_DMA_CB_PIC_START_AD),
			cif_ioread32(dev->config.base_addr +
				CIF_MI_DMA_CR_PIC_START_AD));
	}
}

static int cif_isp20_update_mi_mp(
	struct cif_isp20_device *dev)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(NULL,
		"curr 0x%08x next 0x%08x\n",
		dev->config.mi_config.mp.curr_buff_addr,
		dev->config.mi_config.mp.next_buff_addr);

	if (CIF_ISP20_PIX_FMT_IS_JPEG(
		dev->config.mi_config.mp.output.pix_fmt)) {
		/* in case of jpeg encoding, we don't have to disable the
		   MI, because the encoding
		   anyway has to be started explicitely */
		if (!dev->config.jpeg_config.busy) {
			if (dev->config.mi_config.mp.next_buff_addr !=
				CIF_ISP20_INVALID_BUFF_ADDR)
				cif_isp20_mi_update_buff_addr(dev,
					CIF_ISP20_STREAM_MP);
			if ((dev->config.mi_config.mp.curr_buff_addr !=
				dev->config.mi_config.mp.next_buff_addr) &&
				(dev->config.mi_config.mp.curr_buff_addr !=
				CIF_ISP20_INVALID_BUFF_ADDR)) {
				ret = cif_isp20_jpeg_gen_header(dev);
				if (IS_ERR_VALUE(ret))
					goto err;
				cif_isp20_pltfrm_pr_dbg(NULL,
					"Starting JPEG encoding\n");
				cif_isp20_pltfrm_rtrace_printf(dev->dev,
					"Starting JPEG encoding\n");
				cif_iowrite32(CIF_JPE_ENCODE_ENABLE,
					dev->config.base_addr + CIF_JPE_ENCODE);
				cif_iowrite32(CIF_JPE_INIT_ENABLE,
					dev->config.base_addr +
					CIF_JPE_INIT);
				dev->config.jpeg_config.busy = true;
			}
			dev->config.mi_config.mp.curr_buff_addr =
				dev->config.mi_config.mp.next_buff_addr;
		}
	} else {
		if (dev->config.mi_config.mp.next_buff_addr !=
			dev->config.mi_config.mp.curr_buff_addr) {
			if (dev->config.mi_config.mp.next_buff_addr ==
				CIF_ISP20_INVALID_BUFF_ADDR) {
				/* disable MI MP */
				cif_isp20_pltfrm_pr_dbg(NULL,
					"disabling MP MI\n");
				cif_iowrite32AND_verify(~CIF_MI_CTRL_MP_ENABLE,
					dev->config.base_addr + CIF_MI_CTRL,
					~0);
				cif_isp20_mi_update_buff_addr(dev,
					CIF_ISP20_STREAM_MP);
			} else if (dev->config.mi_config.mp.curr_buff_addr ==
				CIF_ISP20_INVALID_BUFF_ADDR) {
				cif_isp20_mi_update_buff_addr(dev,
					CIF_ISP20_STREAM_MP);
				/* re-enable MI MP */
				cif_isp20_pltfrm_pr_dbg(NULL,
					"enabling MP MI\n");
				cif_iowrite32(CIF_MI_MP_FRAME,
					dev->config.base_addr + CIF_MI_ICR);
				cif_iowrite32OR_verify(CIF_MI_CTRL_MP_ENABLE,
					dev->config.base_addr + CIF_MI_CTRL,
					~0);
			} else
				cif_isp20_mi_update_buff_addr(dev,
					CIF_ISP20_STREAM_MP);

			dev->config.mi_config.mp.curr_buff_addr =
				dev->config.mi_config.mp.next_buff_addr;
		}
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
}

static int cif_isp20_update_mi_sp(
	struct cif_isp20_device *dev)
{
	cif_isp20_pltfrm_pr_dbg(NULL,
		"curr 0x%08x next 0x%08x\n",
		dev->config.mi_config.sp.curr_buff_addr,
		dev->config.mi_config.sp.next_buff_addr);

	if (dev->config.mi_config.sp.next_buff_addr !=
		dev->config.mi_config.sp.curr_buff_addr) {
		if (dev->config.mi_config.sp.next_buff_addr ==
			CIF_ISP20_INVALID_BUFF_ADDR) {
			/* disable MI SP */
			cif_isp20_pltfrm_pr_dbg(NULL, "disabling SP MI\n");
			/* 'switch off' MI interface */
			cif_iowrite32AND_verify(~CIF_MI_CTRL_SP_ENABLE,
				dev->config.base_addr + CIF_MI_CTRL, ~0);
			cif_isp20_mi_update_buff_addr(dev,
				CIF_ISP20_STREAM_SP);
		} else if (dev->config.mi_config.sp.curr_buff_addr ==
			CIF_ISP20_INVALID_BUFF_ADDR) {
			cif_isp20_mi_update_buff_addr(dev,
				CIF_ISP20_STREAM_SP);
			/* re-enable MI SP */
			cif_isp20_pltfrm_pr_dbg(NULL, "enabling SP MI\n");
			cif_iowrite32(CIF_MI_SP_FRAME,
				dev->config.base_addr + CIF_MI_ICR);
			cif_iowrite32OR_verify(CIF_MI_CTRL_SP_ENABLE,
				dev->config.base_addr + CIF_MI_CTRL, ~0);
		} else
			cif_isp20_mi_update_buff_addr(dev,
				CIF_ISP20_STREAM_SP);
		dev->config.mi_config.sp.curr_buff_addr =
			dev->config.mi_config.sp.next_buff_addr;
	}

	return 0;
}

static int cif_isp20_s_fmt_mp(
	struct cif_isp20_device *dev,
	struct cif_isp20_strm_fmt *strm_fmt,
	u32 stride)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s %dx%d@%d/%dfps, stride = %d\n",
		cif_isp20_pix_fmt_string(strm_fmt->frm_fmt.pix_fmt),
		strm_fmt->frm_fmt.width,
		strm_fmt->frm_fmt.height,
		strm_fmt->frm_intrvl.numerator,
		strm_fmt->frm_intrvl.denominator,
		stride);

	/* TBD: check whether format is a valid format for MP */

	if (CIF_ISP20_PIX_FMT_IS_RAW_BAYER(strm_fmt->frm_fmt.pix_fmt)) {
		if ((dev->sp_stream.state == CIF_ISP20_STATE_READY) ||
			(dev->sp_stream.state == CIF_ISP20_STATE_STREAMING))
			cif_isp20_pltfrm_pr_warn(dev->dev,
				"cannot output RAW data when SP is active, you will not be able to (re-)start streaming\n");
	}

	dev->config.mi_config.mp.output = strm_fmt->frm_fmt;

	dev->config.mi_config.mp.llength =
		cif_isp20_calc_llength(
			strm_fmt->frm_fmt.width,
			stride,
			strm_fmt->frm_fmt.pix_fmt);

	dev->mp_stream.updt_cfg = true;
	dev->mp_stream.state = CIF_ISP20_STATE_READY;

	if (dev->config.input_sel < CIF_ISP20_INP_DMA) {
		ret = cif_isp20_img_src_select_strm_fmt(dev);
		if (IS_ERR_VALUE(ret)) {
			dev->mp_stream.updt_cfg = false;
			dev->mp_stream.state = CIF_ISP20_STATE_INACTIVE;
			goto err;
		}
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_s_fmt_sp(
	struct cif_isp20_device *dev,
	struct cif_isp20_strm_fmt *strm_fmt,
	u32 stride)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s %dx%d@%d/%dfps, stride = %d\n",
		cif_isp20_pix_fmt_string(strm_fmt->frm_fmt.pix_fmt),
		strm_fmt->frm_fmt.width,
		strm_fmt->frm_fmt.height,
		strm_fmt->frm_intrvl.numerator,
		strm_fmt->frm_intrvl.denominator,
		stride);

	if (CIF_ISP20_PIX_FMT_IS_RAW_BAYER(
		dev->config.mi_config.mp.output.pix_fmt))
		cif_isp20_pltfrm_pr_warn(dev->dev,
			"cannot activate SP when MP is set to RAW data output, you will not be able to (re-)start streaming\n");

	/* TBD: more detailed check whether format is a valid format for SP */
	/* TBD: remove the mode stuff */
	if (!CIF_ISP20_PIX_FMT_IS_YUV(strm_fmt->frm_fmt.pix_fmt) &&
		!CIF_ISP20_PIX_FMT_IS_RGB(strm_fmt->frm_fmt.pix_fmt)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"format %s %dx%d@%d/%dfps, stride = %d not supported on SP\n",
			cif_isp20_pix_fmt_string(strm_fmt->frm_fmt.pix_fmt),
			strm_fmt->frm_fmt.width,
			strm_fmt->frm_fmt.height,
			strm_fmt->frm_intrvl.numerator,
			strm_fmt->frm_intrvl.denominator,
			stride);
		ret = -EINVAL;
		goto err;
	}

	dev->config.mi_config.sp.output = strm_fmt->frm_fmt;
	dev->config.mi_config.sp.llength =
		cif_isp20_calc_llength(
		strm_fmt->frm_fmt.width,
		stride,
		strm_fmt->frm_fmt.pix_fmt);

	dev->sp_stream.updt_cfg = true;
	dev->sp_stream.state = CIF_ISP20_STATE_READY;

	if (dev->config.input_sel < CIF_ISP20_INP_DMA) {
		ret = cif_isp20_img_src_select_strm_fmt(dev);
		if (IS_ERR_VALUE(ret)) {
			dev->sp_stream.updt_cfg = false;
			dev->sp_stream.state = CIF_ISP20_STATE_INACTIVE;
			goto err;
		}
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_s_fmt_dma(
	struct cif_isp20_device *dev,
	struct cif_isp20_strm_fmt *strm_fmt,
	u32 stride)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s %dx%d@%d/%dfps, stride = %d\n",
		cif_isp20_pix_fmt_string(strm_fmt->frm_fmt.pix_fmt),
		strm_fmt->frm_fmt.width,
		strm_fmt->frm_fmt.height,
		strm_fmt->frm_intrvl.numerator,
		strm_fmt->frm_intrvl.denominator,
		stride);

	if (!CIF_ISP20_PIX_FMT_IS_YUV(strm_fmt->frm_fmt.pix_fmt) &&
		!CIF_ISP20_PIX_FMT_IS_RAW_BAYER(strm_fmt->frm_fmt.pix_fmt)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"format %s %dx%d@%d/%dfps, stride = %d not supported for DMA\n",
			cif_isp20_pix_fmt_string(strm_fmt->frm_fmt.pix_fmt),
			strm_fmt->frm_fmt.width,
			strm_fmt->frm_fmt.height,
			strm_fmt->frm_intrvl.numerator,
			strm_fmt->frm_intrvl.denominator,
			stride);
		ret = -EINVAL;
		goto err;
	}

	dev->config.mi_config.dma.output = strm_fmt->frm_fmt;
	dev->config.mi_config.dma.llength =
		cif_isp20_calc_llength(
		strm_fmt->frm_fmt.width,
		stride,
		strm_fmt->frm_fmt.pix_fmt);

	dev->dma_stream.updt_cfg = true;
	dev->dma_stream.state = CIF_ISP20_STATE_READY;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static void cif_isp20_dma_next_buff(
	struct cif_isp20_device *dev)
{
	cif_isp20_pltfrm_pr_dbg(NULL, "\n");

	if (!list_empty(&dev->dma_stream.buf_queue) &&
		!dev->dma_stream.stop) {
		if (dev->dma_stream.curr_buf != NULL)
			BUG();
		dev->dma_stream.curr_buf =
			list_first_entry(&dev->dma_stream.buf_queue,
				struct videobuf_buffer, queue);
		list_del(&dev->dma_stream.curr_buf->queue);
		dev->dma_stream.curr_buf->state = VIDEOBUF_ACTIVE;
		dev->config.mi_config.dma.next_buff_addr =
			videobuf_to_dma_contig(
				dev->dma_stream.curr_buf);
		cif_isp20_mi_update_buff_addr(dev,
			CIF_ISP20_STREAM_DMA);
		dev->config.mi_config.dma.busy = true;
		if ((dev->sp_stream.state == CIF_ISP20_STATE_STREAMING) &&
			dev->sp_stream.curr_buf)
			dev->config.mi_config.sp.busy = true;
		if ((dev->mp_stream.state == CIF_ISP20_STATE_STREAMING) &&
			dev->mp_stream.curr_buf)
			dev->config.mi_config.mp.busy = true;
		/* workaround for write register failure bug */
		do {
			cif_iowrite32(CIF_MI_DMA_START_ENABLE,
				dev->config.base_addr + CIF_MI_DMA_START);
			udelay(1);
		} while (!cif_ioread32(
			dev->config.base_addr + CIF_MI_DMA_STATUS));
	}

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"\n  MI_DMA_CTRL 0x%08x\n"
		"  MI_DMA_STATUS 0x%08x\n",
		cif_ioread32(dev->config.base_addr +
			CIF_MI_DMA_CTRL),
		cif_ioread32(dev->config.base_addr +
			CIF_MI_DMA_STATUS));
}

static void cif_isp20_dma_ready(
	struct cif_isp20_device *dev)
{
	cif_isp20_pltfrm_pr_dbg(NULL, "\n");

	cif_iowrite32(CIF_MI_DMA_READY,
		dev->config.base_addr + CIF_MI_ICR);
	dev->dma_stream.curr_buf->state = VIDEOBUF_DONE;
	wake_up(&dev->dma_stream.curr_buf->done);
	dev->dma_stream.curr_buf = NULL;
	dev->config.mi_config.dma.busy = false;
	cif_isp20_pltfrm_event_signal(dev->dev, &dev->dma_stream.done);
}

static int cif_isp20_mi_frame_end(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream_id)
{
	struct cif_isp20_stream *stream;
	u32 *next_buff_addr;
	CIF_ISP20_PLTFRM_MEM_IO_ADDR y_base_addr;
	int (*update_mi)(
		struct cif_isp20_device *dev);

	cif_isp20_pltfrm_pr_dbg(NULL, "%s\n",
		cif_isp20_stream_id_string(stream_id));

	if (stream_id == CIF_ISP20_STREAM_MP) {
		stream = &dev->mp_stream;
		y_base_addr =
			dev->config.base_addr + CIF_MI_MP_Y_BASE_AD_SHD;
		next_buff_addr = &dev->config.mi_config.mp.next_buff_addr;
		update_mi = cif_isp20_update_mi_mp;
		if (CIF_ISP20_PIX_FMT_IS_JPEG(
			dev->config.mi_config.mp.output.pix_fmt)) {
			unsigned int jpe_status =
				cif_ioread32(dev->config.base_addr +
					CIF_JPE_STATUS_RIS);
			if (jpe_status & CIF_JPE_STATUS_ENCODE_DONE) {
				cif_iowrite32(CIF_JPE_STATUS_ENCODE_DONE,
					dev->config.base_addr +
						CIF_JPE_STATUS_ICR);
				if (stream->curr_buf != NULL) {
					stream->curr_buf->size =
					cif_ioread32(dev->config.base_addr +
						CIF_MI_BYTE_CNT);
					cif_isp20_pltfrm_pr_dbg(NULL,
						"JPEG encoding done, size %lu\n",
						stream->curr_buf->size);
					if (cif_ioread32(dev->config.base_addr +
						CIF_MI_RIS) & CIF_MI_WRAP_MP_Y)
						cif_isp20_pltfrm_pr_err(NULL,
							"buffer wrap around detected, JPEG presumably corrupted (%d/%d/%lu)\n",
							dev->config.mi_config.
							mp.y_size,
							cif_ioread32(
							dev->config.base_addr +
							CIF_MI_MP_Y_SIZE_SHD),
							stream->curr_buf->size);
				}
			}
		}
	} else if (stream_id == CIF_ISP20_STREAM_SP) {
		stream = &dev->sp_stream;
		y_base_addr =
			dev->config.base_addr + CIF_MI_SP_Y_BASE_AD_SHD;
		next_buff_addr = &dev->config.mi_config.sp.next_buff_addr;
		update_mi = cif_isp20_update_mi_sp;
	} else
		BUG();

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s Y_BASE_AD_INIT/Y_BASE_AD_SHD (0x%08x/0x%08x)\n",
		cif_isp20_stream_id_string(stream_id),
		(stream_id & CIF_ISP20_STREAM_MP) ?
			cif_ioread32(dev->config.base_addr +
			CIF_MI_MP_Y_BASE_AD_INIT) :
			cif_ioread32(dev->config.base_addr +
			CIF_MI_SP_Y_BASE_AD_INIT),
		cif_ioread32(y_base_addr));

	if ((stream->next_buf == NULL) &&
		!(CIF_ISP20_PIX_FMT_IS_JPEG(
		dev->config.mi_config.mp.output.pix_fmt) &&
		(stream_id == CIF_ISP20_STREAM_MP))) {
		stream->stall = dev->config.out_of_buffer_stall;
	} else if ((stream->next_buf != NULL) &&
		(videobuf_to_dma_contig(stream->next_buf) !=
			cif_ioread32(y_base_addr))) {
		cif_isp20_pltfrm_pr_warn(dev->dev,
			"%s buffer queue is not advancing (0x%08x/0x%08x)\n",
			cif_isp20_stream_id_string(stream_id),
			(stream_id & CIF_ISP20_STREAM_MP) ?
				cif_ioread32(dev->config.base_addr +
				CIF_MI_MP_Y_BASE_AD_INIT) :
				cif_ioread32(dev->config.base_addr +
				CIF_MI_SP_Y_BASE_AD_INIT),
			cif_ioread32(y_base_addr));
		stream->stall = true;
	}

	if (!stream->stall) {
		if (stream->curr_buf != NULL) {
			cif_isp20_pltfrm_pr_dbg(NULL,
				"frame done\n");
			stream->curr_buf->field_count = dev->isp_dev.frame_id;
			stream->curr_buf->ts = dev->curr_frame_time;
			/*Inform the wait queue */
			stream->curr_buf->state = VIDEOBUF_DONE;
			wake_up(&stream->curr_buf->done);
			stream->curr_buf = NULL;
		}
		stream->curr_buf = stream->next_buf;
		stream->next_buf = NULL;
	}

	if (stream->next_buf == NULL) {
		/* in case of jpeg encoding, we are only programming
		a new buffer, if the jpeg header was generated, because
		we need the curent buffer for the jpeg encoding
		in the current frame period */
		if (!list_empty(&stream->buf_queue)) {
			stream->next_buf =
				list_first_entry(&stream->buf_queue,
					struct videobuf_buffer, queue);
			list_del(&stream->next_buf->queue);
			stream->next_buf->state = VIDEOBUF_ACTIVE;
			*next_buff_addr = videobuf_to_dma_contig(
				stream->next_buf);
		} else if (!dev->config.out_of_buffer_stall ||
			(CIF_ISP20_PIX_FMT_IS_JPEG(
				dev->config.mi_config.mp.output.pix_fmt) &&
			(stream_id == CIF_ISP20_STREAM_MP)))
			*next_buff_addr =
				CIF_ISP20_INVALID_BUFF_ADDR;
	}
	(void)update_mi(dev);

	stream->stall = false;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s curr_buff 0x%p, next_buf 0x%p, next_buff_addr = 0x%08x\n",
		cif_isp20_stream_id_string(stream_id),
		stream->curr_buf, stream->next_buf,
		*next_buff_addr);

	return 0;
}

static void cif_isp20_start_mi(
	struct cif_isp20_device *dev,
	bool start_mi_sp,
	bool start_mi_mp)
{
	cif_isp20_pltfrm_pr_dbg(dev->dev, "\n");

	if (start_mi_sp &&
		(dev->sp_stream.state == CIF_ISP20_STATE_STREAMING))
		start_mi_sp = false;
	if (start_mi_mp &&
		(dev->mp_stream.state == CIF_ISP20_STATE_STREAMING))
		start_mi_mp = false;
	if (!start_mi_sp && !start_mi_mp)
		return;

	if ((start_mi_sp &&
		(dev->mp_stream.state == CIF_ISP20_STATE_STREAMING)) ||
		(start_mi_mp &&
		(dev->sp_stream.state == CIF_ISP20_STATE_STREAMING)))
		BUG();

	cif_iowrite32OR_verify(CIF_MI_AHB_ERROR,
		dev->config.base_addr +
		CIF_MI_IMSC, ~0);

	if (start_mi_sp) {
		dev->config.mi_config.sp.next_buff_addr =
			CIF_ISP20_INVALID_BUFF_ADDR;
		dev->config.mi_config.sp.curr_buff_addr =
			CIF_ISP20_INVALID_BUFF_ADDR;
		spin_lock(&dev->vbq_lock);
		cif_isp20_mi_frame_end(dev, CIF_ISP20_STREAM_SP);
		spin_unlock(&dev->vbq_lock);
		dev->sp_stream.stall = false;
	}

	if (start_mi_mp) {
		dev->config.mi_config.mp.next_buff_addr =
			CIF_ISP20_INVALID_BUFF_ADDR;
		dev->config.mi_config.mp.curr_buff_addr =
			CIF_ISP20_INVALID_BUFF_ADDR;
		spin_lock(&dev->vbq_lock);
		cif_isp20_mi_frame_end(dev, CIF_ISP20_STREAM_MP);
		spin_unlock(&dev->vbq_lock);
		dev->mp_stream.stall = false;
	}

	cif_iowrite32(CIF_MI_INIT_SOFT_UPD,
		dev->config.base_addr + CIF_MI_INIT);
	cif_isp20_pltfrm_pr_dbg(NULL,
		"CIF_MI_INIT_SOFT_UPD\n");

	if (start_mi_sp) {
		spin_lock(&dev->vbq_lock);
		cif_isp20_mi_frame_end(dev, CIF_ISP20_STREAM_SP);
		spin_unlock(&dev->vbq_lock);
		if (dev->sp_stream.curr_buf &&
			(dev->config.input_sel < CIF_ISP20_INP_DMA))
			dev->config.mi_config.sp.busy = true;
	}

	if (start_mi_mp) {
		spin_lock(&dev->vbq_lock);
		cif_isp20_mi_frame_end(dev, CIF_ISP20_STREAM_MP);
		spin_unlock(&dev->vbq_lock);
		if (dev->mp_stream.curr_buf &&
			(dev->config.input_sel < CIF_ISP20_INP_DMA))
			dev->config.mi_config.mp.busy = true;
	}

	if (!dev->config.mi_config.async_updt)
		cif_iowrite32OR(CIF_ISP_CTRL_ISP_GEN_CFG_UPD,
			dev->config.base_addr + CIF_ISP_CTRL);

}

static void cif_isp20_stop_mi(
	struct cif_isp20_device *dev,
	bool stop_mi_sp,
	bool stop_mi_mp)
{
	cif_isp20_pltfrm_pr_dbg(dev->dev, "\n");

	if (stop_mi_sp &&
		(dev->sp_stream.state != CIF_ISP20_STATE_STREAMING))
		stop_mi_sp = false;
	if (stop_mi_mp &&
		(dev->mp_stream.state != CIF_ISP20_STATE_STREAMING))
		stop_mi_mp = false;

	if (!stop_mi_sp && !stop_mi_mp)
		return;

	if (stop_mi_sp && stop_mi_mp) {
		cif_iowrite32AND_verify(~(CIF_MI_SP_FRAME |
			CIF_MI_MP_FRAME |
			CIF_JPE_STATUS_ENCODE_DONE),
			dev->config.base_addr + CIF_MI_IMSC, ~0);
		cif_iowrite32(CIF_MI_SP_FRAME |
			CIF_MI_MP_FRAME |
			CIF_JPE_STATUS_ENCODE_DONE,
			dev->config.base_addr + CIF_MI_ICR);
		cif_iowrite32AND_verify(~CIF_MI_CTRL_SP_ENABLE,
			dev->config.base_addr + CIF_MI_CTRL, ~0);
		cif_iowrite32AND_verify(~(CIF_MI_CTRL_MP_ENABLE |
			CIF_MI_CTRL_SP_ENABLE |
			CIF_MI_CTRL_JPEG_ENABLE |
			CIF_MI_CTRL_RAW_ENABLE),
			dev->config.base_addr + CIF_MI_CTRL, ~0);
		cif_iowrite32(CIF_MI_INIT_SOFT_UPD,
			dev->config.base_addr + CIF_MI_INIT);
	} else if (stop_mi_sp) {
		cif_iowrite32(CIF_MI_SP_FRAME,
			dev->config.base_addr + CIF_MI_ICR);
		cif_iowrite32AND_verify(~CIF_MI_CTRL_SP_ENABLE,
			dev->config.base_addr + CIF_MI_CTRL, ~0);
	} else if (stop_mi_mp) {
		cif_iowrite32(CIF_MI_MP_FRAME |
			CIF_JPE_STATUS_ENCODE_DONE,
			dev->config.base_addr + CIF_MI_ICR);
		cif_iowrite32AND_verify(~(CIF_MI_CTRL_MP_ENABLE |
			CIF_MI_CTRL_JPEG_ENABLE |
			CIF_MI_CTRL_RAW_ENABLE),
			dev->config.base_addr + CIF_MI_CTRL, ~0);
	}
}

static void cif_isp20_requeue_bufs(
	struct cif_isp20_device *dev,
	struct cif_isp20_stream *stream)
{
	INIT_LIST_HEAD(&stream->buf_queue);
	stream->next_buf = NULL;
	stream->curr_buf = NULL;
	dev->requeue_bufs(stream->id);
	if (stream->id == CIF_ISP20_STREAM_SP) {
		dev->config.mi_config.sp.next_buff_addr =
			CIF_ISP20_INVALID_BUFF_ADDR;
		dev->config.mi_config.sp.curr_buff_addr =
			CIF_ISP20_INVALID_BUFF_ADDR;
	} else if (stream->id == CIF_ISP20_STREAM_MP) {
		dev->config.mi_config.mp.next_buff_addr =
			CIF_ISP20_INVALID_BUFF_ADDR;
		dev->config.mi_config.mp.curr_buff_addr =
			CIF_ISP20_INVALID_BUFF_ADDR;
	}
}

static void cif_isp20_stop_sp(
	struct cif_isp20_device *dev)
{
	int ret;
	if (dev->sp_stream.state ==
		CIF_ISP20_STATE_STREAMING) {
		dev->sp_stream.stop = true;
		ret = cif_isp20_pltfrm_event_wait_timeout(dev->dev,
			&dev->sp_stream.done,
			dev->sp_stream.state !=
		CIF_ISP20_STATE_STREAMING,
			1000000);
		dev->sp_stream.stop = false;
		if (IS_ERR_VALUE(ret)) {
			cif_isp20_pltfrm_pr_warn(NULL,
				"waiting on event returned with error %d\n",
				ret);
		}
		if (dev->config.mi_config.sp.busy)
			cif_isp20_pltfrm_pr_warn(NULL,
				"SP path still active while stopping it\n");
	}
}

static void cif_isp20_stop_mp(
	struct cif_isp20_device *dev)
{
	int ret;
	if (dev->mp_stream.state ==
		CIF_ISP20_STATE_STREAMING) {
		dev->mp_stream.stop = true;
		ret = cif_isp20_pltfrm_event_wait_timeout(dev->dev,
			&dev->mp_stream.done,
			dev->mp_stream.state !=
		CIF_ISP20_STATE_STREAMING,
			1000000);
		dev->mp_stream.stop = false;
		if (IS_ERR_VALUE(ret)) {
			cif_isp20_pltfrm_pr_warn(NULL,
				"waiting on event returned with error %d\n",
				ret);
		}
		if (dev->config.mi_config.mp.busy ||
			dev->config.jpeg_config.busy)
			cif_isp20_pltfrm_pr_warn(NULL,
				"MP path still active while stopping it\n");
	}
}

static void cif_isp20_stop_dma(
	struct cif_isp20_device *dev)
{
	unsigned long flags = 0;
	cif_isp20_pltfrm_pr_dbg(NULL, "\n");

	if (dev->dma_stream.state ==
		CIF_ISP20_STATE_STREAMING) {
		/* we should not stop during an active DMA transfer */
		dev->dma_stream.stop = true;
		(void)cif_isp20_pltfrm_event_wait_timeout(dev->dev,
			&dev->dma_stream.done,
			dev->dma_stream.state !=
			CIF_ISP20_STATE_STREAMING,
			50000);
		/* intentionally NOT checking dma.busy again */
		if (dev->config.mi_config.dma.busy)
			cif_isp20_pltfrm_pr_warn(NULL,
				"DMA transfer still active while stopping it\n");
		dev->dma_stream.state = CIF_ISP20_STATE_READY;
		spin_lock_irqsave(&dev->vbq_lock, flags);
		cif_isp20_requeue_bufs(dev, &dev->dma_stream);
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
	}
}

static int cif_isp20_stop(
	struct cif_isp20_device *dev,
	bool stop_sp,
	bool stop_mp)
{
	unsigned long flags = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, img_src state = %s, stop_sp = %d, stop_mp = %d\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_img_src_state_string(dev->img_src_state),
		stop_sp,
		stop_mp);

	if (!((stop_mp &&
		(dev->mp_stream.state == CIF_ISP20_STATE_STREAMING)) ||
		(stop_sp &&
		(dev->sp_stream.state == CIF_ISP20_STATE_STREAMING)))) {
		return 0;
	}

	if ((stop_mp && stop_sp) ||
		(stop_sp &&
		(dev->mp_stream.state != CIF_ISP20_STATE_STREAMING)) ||
		(stop_mp &&
		(dev->sp_stream.state != CIF_ISP20_STATE_STREAMING))) {

		cif_isp20_stop_dma(dev);

		local_irq_save(flags);
		/* stop and clear MI, MIPI, and ISP interrupts */
		cif_iowrite32(0, dev->config.base_addr + CIF_MIPI_IMSC);
		cif_iowrite32(~0, dev->config.base_addr + CIF_MIPI_ICR);

		cif_iowrite32(0, dev->config.base_addr + CIF_ISP_IMSC);
		cif_iowrite32(~0, dev->config.base_addr + CIF_ISP_ICR);

		cif_iowrite32_verify(0,
			dev->config.base_addr + CIF_MI_IMSC, ~0);
		cif_iowrite32(~0, dev->config.base_addr + CIF_MI_ICR);

		cif_iowrite32AND(~CIF_MIPI_CTRL_OUTPUT_ENA,
			dev->config.base_addr + CIF_MIPI_CTRL);

		/* stop MI, MIPI, and ISP */
		cif_iowrite32AND(~(CIF_ISP_CTRL_ISP_INFORM_ENABLE |
			CIF_ISP_CTRL_ISP_ENABLE),
			dev->config.base_addr + CIF_ISP_CTRL);
		cif_iowrite32OR(CIF_ISP_CTRL_ISP_CFG_UPD,
			dev->config.base_addr + CIF_ISP_CTRL);

		cif_isp20_stop_mi(dev, stop_sp, stop_mp);
		local_irq_restore(flags);

		if (dev->config.input_sel < CIF_ISP20_INP_DMA) {
			if (IS_ERR_VALUE(cif_isp20_img_src_set_state(dev,
				CIF_ISP20_IMG_SRC_STATE_SW_STNDBY)))
				cif_isp20_pltfrm_pr_dbg(dev->dev,
					"unable to put image source into standby\n");
		}
		if (IS_ERR_VALUE(cif_isp20_set_pm_state(dev,
			CIF_ISP20_PM_STATE_SW_STNDBY)))
			cif_isp20_pltfrm_pr_dbg(dev->dev,
			"unable to put CIF into standby\n");
	} else if (stop_sp) {
		if (!dev->config.mi_config.async_updt) {
			local_irq_save(flags);
			cif_isp20_stop_mi(dev, true, false);
			local_irq_restore(flags);
		}
		cif_isp20_stop_sp(dev);
		cif_iowrite32AND_verify(~CIF_MI_SP_FRAME,
			dev->config.base_addr + CIF_MI_IMSC, ~0);

	} else /* stop_mp */ {
		if (!dev->config.mi_config.async_updt) {
			local_irq_save(flags);
			cif_isp20_stop_mi(dev, false, true);
			local_irq_restore(flags);
		}
		cif_isp20_stop_mp(dev);
		cif_iowrite32AND_verify(~(CIF_MI_MP_FRAME |
			CIF_JPE_STATUS_ENCODE_DONE),
			dev->config.base_addr + CIF_MI_IMSC, ~0);

	}

	if (stop_mp && (dev->mp_stream.state == CIF_ISP20_STATE_STREAMING))
		dev->mp_stream.state = CIF_ISP20_STATE_READY;

	if (stop_sp && (dev->sp_stream.state == CIF_ISP20_STATE_STREAMING))
		dev->sp_stream.state = CIF_ISP20_STATE_READY;

	spin_lock(&dev->vbq_lock);
	if (stop_sp) {
		dev->config.mi_config.sp.busy = false;
		cif_isp20_requeue_bufs(dev, &dev->sp_stream);
	}
	if (stop_mp) {
		dev->config.mi_config.mp.busy = false;
		cif_isp20_requeue_bufs(dev, &dev->mp_stream);
	}
	spin_unlock(&dev->vbq_lock);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, DMA state = %s, img_src state = %s\n"
		"  MI_CTRL 0x%08x\n"
		"  ISP_CTRL 0x%08x\n"
		"  MIPI_CTRL 0x%08x\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_state_string(dev->dma_stream.state),
		cif_isp20_img_src_state_string(dev->img_src_state),
		cif_ioread32(dev->config.base_addr + CIF_MI_CTRL),
		cif_ioread32(dev->config.base_addr + CIF_ISP_CTRL),
		cif_ioread32(dev->config.base_addr + CIF_MIPI_CTRL));

	return 0;
}

static int cif_isp20_start(
	struct cif_isp20_device *dev,
	bool start_sp,
	bool start_mp)
{
	unsigned int ret;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, DMA state = %s, img_src state = %s, start_sp = %d, start_mp = %d\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_state_string(dev->dma_stream.state),
		cif_isp20_img_src_state_string(dev->img_src_state),
		start_sp,
		start_mp);

	if (!((start_mp &&
		(dev->mp_stream.state != CIF_ISP20_STATE_STREAMING)) ||
		(start_sp &&
		(dev->sp_stream.state != CIF_ISP20_STATE_STREAMING))))
		return 0;

	if ((dev->config.input_sel > CIF_ISP20_INP_CPI) &&
		(dev->dma_stream.state < CIF_ISP20_STATE_READY)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"cannot start streaming, input source (DMA) not ready\n");
		ret = -EFAULT;
		goto err;
	}

	/* Activate MI */
	cif_isp20_start_mi(dev, start_sp, start_mp);

	if ((dev->sp_stream.state != CIF_ISP20_STATE_STREAMING) &&
		(dev->mp_stream.state != CIF_ISP20_STATE_STREAMING)) {
		/* Activate MIPI */
		if (dev->config.input_sel < CIF_ISP20_INP_DMA)
			cif_iowrite32OR(CIF_MIPI_CTRL_OUTPUT_ENA,
				dev->config.base_addr + CIF_MIPI_CTRL);

		/* Activate ISP ! */
		if (dev->config.input_sel < CIF_ISP20_INP_DMA_IE)
			cif_iowrite32OR(CIF_ISP_CTRL_ISP_CFG_UPD |
				CIF_ISP_CTRL_ISP_INFORM_ENABLE |
				CIF_ISP_CTRL_ISP_ENABLE,
				dev->config.base_addr + CIF_ISP_CTRL);
	}

	if (start_sp &&
		(dev->sp_stream.state != CIF_ISP20_STATE_STREAMING)) {
		dev->sp_stream.state = CIF_ISP20_STATE_STREAMING;
	}
	if (start_mp &&
		(dev->mp_stream.state != CIF_ISP20_STATE_STREAMING)) {
		dev->mp_stream.state = CIF_ISP20_STATE_STREAMING;
	}
	ret = cif_isp20_set_pm_state(dev,
		CIF_ISP20_PM_STATE_STREAMING);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (dev->config.input_sel < CIF_ISP20_INP_DMA) {
		/* CIF spec says to wait for sufficient time after enabling
			the MIPI interface and before starting the
			sensor output. */
		mdelay(1);
		/* start sensor output! */
		dev->isp_dev.frame_id = 0;
		cif_isp20_pltfrm_rtrace_printf(dev->dev,
			"starting image source...\n");
		ret = cif_isp20_img_src_set_state(dev,
			CIF_ISP20_IMG_SRC_STATE_STREAMING);
		if (IS_ERR_VALUE(ret))
			goto err;
	} else {
		cif_isp20_pltfrm_rtrace_printf(dev->dev,
			"starting DMA...\n");
		dev->dma_stream.state = CIF_ISP20_STATE_STREAMING;
		dev->dma_stream.stop = false;
		cif_isp20_dma_next_buff(dev);
	}

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, DMA state = %s, img_src state = %s\n"
		"  MI_CTRL 0x%08x\n"
		"  ISP_CTRL 0x%08x\n"
		"  MIPI_CTRL 0x%08x\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_state_string(dev->dma_stream.state),
		cif_isp20_img_src_state_string(dev->img_src_state),
		cif_ioread32(dev->config.base_addr + CIF_MI_CTRL),
		cif_ioread32(dev->config.base_addr + CIF_ISP_CTRL),
		cif_ioread32(dev->config.base_addr + CIF_MIPI_CTRL));

	return 0;
err:
	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, DMA state = %s, img_src state = %s\n"
		"  MI_CTRL 0x%08x\n"
		"  ISP_CTRL 0x%08x\n"
		"  MIPI_CTRL 0x%08x\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_state_string(dev->dma_stream.state),
		cif_isp20_img_src_state_string(dev->img_src_state),
		cif_ioread32(dev->config.base_addr + CIF_MI_CTRL),
		cif_ioread32(dev->config.base_addr + CIF_ISP_CTRL),
		cif_ioread32(dev->config.base_addr + CIF_MIPI_CTRL));
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
}

static inline bool cif_isp20_check_for_ovs(
		struct cif_isp20_device *dev, int line)
{
	if (cif_ioread32(dev->config.base_addr + CIF_ISP_RIS)
			& CIF_ISP_V_START) {
		cif_isp20_pltfrm_pr_warn(dev->dev,
			"(%d) Overlapping V_SYNC\n", line);
		return true;
	}
	return false;
}

/* Function to be called inside ISR to update CIF ISM/YCFLT/RSZ */
static int cif_isp20_update_ism_ycflt_rsz(
	struct cif_isp20_device *dev)
{
	int ret = 0;
	unsigned int dpcl;
	unsigned int mi_ctrl_shd;

	bool mi_mp_off = false;

	if (cif_isp20_check_for_ovs(dev, __LINE__))
		return 0;

	dpcl = cif_ioread32(dev->config.base_addr + CIF_VI_DPCL);
	if ((dpcl & CIF_VI_DPCL_CHAN_MODE_MP) &&
	    (dpcl & CIF_VI_DPCL_CHAN_MODE_SP) &&
	    (dpcl & CIF_VI_DPCL_MP_MUX_MRSZ_JPEG)) {
		mi_ctrl_shd = cif_ioread32(dev->config.base_addr
			+ CIF_MI_CTRL_SHD);
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"mi_ctrl=0x%08x, dpcl=0x%08x\n",
			mi_ctrl_shd, dpcl);

		if (!(mi_ctrl_shd & CIF_MI_CTRL_SHD_MP_ENABLE) &&
			!(mi_ctrl_shd & CIF_MI_CTRL_SHD_JPEG_ENABLE) &&
			!(mi_ctrl_shd & CIF_MI_CTRL_SHD_RAW_ENABLE)) {
			mi_mp_off = true;
		}
	}

	if (mi_mp_off) {
		/* Branch for partial stop on MP */
		/* Turn off MRSZ since it is not needed */
		cif_iowrite32(0, dev->config.base_addr + CIF_MRSZ_CTRL);
		cif_iowrite32OR(CIF_RSZ_CTRL_CFG_UPD,
			dev->config.base_addr + CIF_MRSZ_CTRL);

		/* Also need to turn off XNR SS if it is ON and not
		connected to SP, otherwise there are
		MIPI_SYNC_OVERFLOW issues.  Possible explanation
		is that in our capture usecase, DPCL is set to sp_mp
		and when we can not change it to sp here without
		stopping the whole CIF, so the output of YCFLT has
		nowhere to go. */
		/* We must turn off the whole YCFLT in this
		case. Only XNR SS is not enough */
		cifisp_ycflt_end(&dev->isp_dev);
		dev->isp_dev.ycflt_en = false;

		/* SRSZ might still need to be updated because we might
		haved changed YCFLT XNR SS */
		dev->config.sp_config.rsz_config.ycflt_adjust = true;
		cif_isp20_pltfrm_pr_dbg(dev->dev, "mp stop seq\n");

	} else {
		/* Branch for normal update of YCFLT/ISM */
		if (dev->isp_dev.ycflt_update ||
			dev->config.isp_config.ism_config.ism_update_needed) {

			if (dev->config.isp_config.ism_config.ism_en) {
				if (dev->isp_dev.cif_ism_cropping == false) {
					dev->isp_dev.cif_ism_cropping = true;
					dev->isp_dev.ycflt_update = true;
				}
			} else {
				if (dev->isp_dev.cif_ism_cropping == true) {
					dev->isp_dev.cif_ism_cropping = false;
					dev->isp_dev.ycflt_update = true;
				}
			}
		}

		/* Update YCFLT */
		if (dev->isp_dev.ycflt_update) {
			if (dev->isp_dev.ycflt_en) {
				cifisp_ycflt_config(&dev->isp_dev);
				cifisp_ycflt_en(&dev->isp_dev);
			} else
				cifisp_ycflt_end(&dev->isp_dev);

			dev->isp_dev.ycflt_update = false;

			if (dev->mp_stream.state ==
					CIF_ISP20_STATE_STREAMING)
				dev->config.mp_config.rsz_config.ycflt_adjust
					= true;
			if (dev->config.sp_config.inp_yc_filt &&
				(dev->sp_stream.state ==
					CIF_ISP20_STATE_STREAMING))
				dev->config.sp_config.rsz_config.ycflt_adjust
					= true;
		}

		/* Update ISM, cif_isp20_config_ism() changes the
		output size of isp, so it must be called before
		cif_isp20_config_rsz() */
		if (dev->config.isp_config.ism_config.ism_update_needed) {
			cif_isp20_config_ism(dev, false);
			if (dev->mp_stream.state == CIF_ISP20_STATE_STREAMING)
				dev->config.mp_config.rsz_config.
					ism_adjust = true;
			if (dev->sp_stream.state == CIF_ISP20_STATE_STREAMING)
				dev->config.sp_config.rsz_config.
					ism_adjust = true;
			dev->config.isp_config.ism_config.
				ism_update_needed = false;
			cif_iowrite32OR(CIF_ISP_CTRL_ISP_CFG_UPD,
				dev->config.base_addr + CIF_ISP_CTRL);

			if (dev->config.isp_config.ism_config.ism_en)
				dev->config.mi_config.async_updt
					|= CIF_ISP20_ASYNC_ISM;
		}
	}

	/* Update RSZ */
	if ((dev->config.mp_config.rsz_config.ycflt_adjust ||
		dev->config.mp_config.rsz_config.ism_adjust)) {
		ret = cif_isp20_config_rsz(dev,	CIF_ISP20_STREAM_MP, true);
		if (IS_ERR_VALUE(ret))
			goto err;
	}
	if ((dev->config.sp_config.rsz_config.ycflt_adjust ||
		dev->config.sp_config.rsz_config.ism_adjust)) {
		ret = cif_isp20_config_rsz(dev, CIF_ISP20_STREAM_SP, true);
		if (IS_ERR_VALUE(ret))
			goto err;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
}

static int cif_isp20_mi_isr(void *cntxt)
{
	struct cif_isp20_device *dev = cntxt;
	unsigned int mi_mis;

	mi_mis = cif_ioread32(dev->config.base_addr + CIF_MI_MIS);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"\n  MI_RIS 0x%08x\n"
		"  MI_IMSC 0x%08x\n"
		"  MI_MIS 0x%08x\n",
		cif_ioread32(dev->config.base_addr + CIF_MI_RIS),
		cif_ioread32(dev->config.base_addr + CIF_MI_IMSC),
		mi_mis);

	cif_isp20_pltfrm_rtrace_printf(dev->dev,
		"MI_MIS %08x, MI_RIS %08x, MI_IMSC %08x\n",
		mi_mis,
		cif_ioread32(dev->config.base_addr + CIF_MI_RIS),
		cif_ioread32(dev->config.base_addr + CIF_MI_IMSC));

	if (mi_mis & CIF_MI_AHB_ERROR) {
		cif_isp20_pltfrm_pr_warn(dev->dev, "AHB error\n");
		cif_iowrite32(CIF_MI_AHB_ERROR,
			dev->config.base_addr + CIF_MI_ICR);
	}

	if (mi_mis & CIF_MI_SP_FRAME) {
		dev->config.mi_config.sp.busy = false;
		cif_iowrite32(CIF_MI_SP_FRAME,
			dev->config.base_addr + CIF_MI_ICR);
	}
	if (mi_mis & CIF_MI_MP_FRAME) {
		dev->config.mi_config.mp.busy = false;
		cif_iowrite32(CIF_MI_MP_FRAME,
			dev->config.base_addr + CIF_MI_ICR);
	}
	if (mi_mis & CIF_MI_DMA_READY)
		(void)cif_isp20_dma_ready(dev);
	if (CIF_ISP20_PIX_FMT_IS_JPEG(
		dev->config.mi_config.mp.output.pix_fmt) &&
		(cif_ioread32(dev->config.base_addr +
			CIF_JPE_STATUS_RIS) & CIF_JPE_STATUS_ENCODE_DONE))
		dev->config.jpeg_config.busy = false;

	if (!CIF_ISP20_MI_IS_BUSY(dev) &&
		!dev->config.jpeg_config.busy) {

		if (dev->config.mi_config.async_updt) {
			u32 mp_y_off_cnt_shd =
				cif_ioread32(dev->config.base_addr +
				CIF_MI_MP_Y_OFFS_CNT_SHD);
			u32 sp_y_off_cnt_shd =
				cif_ioread32(dev->config.base_addr +
				CIF_MI_SP_Y_OFFS_CNT_SHD);
			cif_iowrite32(CIF_MI_INIT_SOFT_UPD,
				dev->config.base_addr + CIF_MI_INIT);
			cif_isp20_pltfrm_pr_dbg(NULL,
				"CIF_MI_INIT_SOFT_UPD\n");
			if (!dev->config.isp_config.ism_config.ism_en &&
				(dev->config.mi_config.async_updt &
				CIF_ISP20_ASYNC_ISM))
				dev->config.mi_config.async_updt &=
					~CIF_ISP20_ASYNC_ISM;
			if (sp_y_off_cnt_shd != 0)
				cif_isp20_requeue_bufs(dev, &dev->sp_stream);
			if ((mp_y_off_cnt_shd != 0) &&
				!CIF_ISP20_PIX_FMT_IS_JPEG(
					dev->config.mi_config.
					mp.output.pix_fmt))
				cif_isp20_requeue_bufs(dev, &dev->mp_stream);
			if (((mp_y_off_cnt_shd != 0) &&
				!CIF_ISP20_PIX_FMT_IS_JPEG(
					dev->config.mi_config.
					mp.output.pix_fmt)) ||
				(sp_y_off_cnt_shd != 0)) {
				cif_isp20_pltfrm_pr_dbg(dev->dev,
					"soft update too late (SP offset %d, MP offset %d)\n",
					sp_y_off_cnt_shd, mp_y_off_cnt_shd);
			}
		}

		if (dev->mp_stream.stop &&
			(dev->mp_stream.state == CIF_ISP20_STATE_STREAMING)) {
			cif_isp20_stop_mi(dev, false, true);
			cif_iowrite32(CIF_MI_INIT_SOFT_UPD,
				dev->config.base_addr + CIF_MI_INIT);
			dev->mp_stream.state = CIF_ISP20_STATE_READY;
			dev->mp_stream.stop = false;

			cif_isp20_pltfrm_pr_dbg(NULL,
				"MP has stopped\n");
			cif_isp20_pltfrm_event_signal(dev->dev,
				&dev->mp_stream.done);
		}
		if (dev->sp_stream.stop &&
			(dev->sp_stream.state == CIF_ISP20_STATE_STREAMING)) {
			cif_isp20_stop_mi(dev, true, false);
			cif_iowrite32(CIF_MI_INIT_SOFT_UPD,
				dev->config.base_addr + CIF_MI_INIT);
			dev->sp_stream.state = CIF_ISP20_STATE_READY;
			dev->sp_stream.stop = false;

			cif_isp20_pltfrm_pr_dbg(NULL,
				"SP has stopped\n");
			cif_isp20_pltfrm_event_signal(dev->dev,
				&dev->sp_stream.done);
		}

		if (dev->sp_stream.state == CIF_ISP20_STATE_STREAMING)
			(void)cif_isp20_mi_frame_end(dev,
				CIF_ISP20_STREAM_SP);
		if (dev->mp_stream.state == CIF_ISP20_STATE_STREAMING) {
			(void)cif_isp20_mi_frame_end(dev,
				CIF_ISP20_STREAM_MP);
#ifdef CONFIG_CIF_ISP20_TEST_YC_FLT
			if (0 && !(++cif_isp20_dbg_count % 150))
				cif_isp20_enable_yc_flt(dev);
#endif
		}

		dev->b_mi_frame_end = true;

		if (dev->dma_stream.state == CIF_ISP20_STATE_STREAMING) {
			cif_isp20_dma_next_buff(dev);
		} else {
			if ((dev->sp_stream.state ==
				CIF_ISP20_STATE_STREAMING) &&
				dev->sp_stream.curr_buf)
				dev->config.mi_config.sp.busy = true;
			if ((dev->mp_stream.state ==
				CIF_ISP20_STATE_STREAMING) &&
				dev->mp_stream.curr_buf)
				dev->config.mi_config.mp.busy = true;
		}

		if (dev->b_isp_frame_in)
			cif_isp20_update_ism_ycflt_rsz(dev);

	}

	cif_iowrite32(~(CIF_MI_MP_FRAME |
		CIF_MI_SP_FRAME | CIF_MI_DMA_READY),
		dev->config.base_addr + CIF_MI_ICR);

	return 0;
}

static int cif_isp20_isr(void *cntxt)
{
	struct cif_isp20_device *dev = cntxt;

	spin_lock(&dev->irq_lock);

	if ((dev->config.input_sel < CIF_ISP20_INP_CPI) &&
		cif_ioread32(dev->config.base_addr + CIF_MIPI_MIS))
		(void)marvin_mipi_isr(cntxt);
	if (cif_ioread32(dev->config.base_addr + CIF_ISP_MIS))
		(void)marvin_isp_isr(cntxt);
	if (cif_ioread32(dev->config.base_addr + CIF_MI_MIS))
		(void)cif_isp20_mi_isr(cntxt);

	spin_unlock(&dev->irq_lock);

	return 0;
}

static int cif_isp20_register_isrs(struct cif_isp20_device *dev)
{
	int ret = 0;

	spin_lock_init(&dev->irq_lock);

	cif_isp20_pltfrm_irq_register_isr(
		dev->dev,
		CIF_ISP20_IRQ_ISP,
		cif_isp20_isr,
		dev);
	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_warn(dev->dev,
			"unable to register ISP ISR, some processing errors may go unnoticed\n");

	cif_isp20_pltfrm_irq_register_isr(
		dev->dev,
		CIF_ISP20_IRQ_MIPI,
		cif_isp20_isr,
		dev);
	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_warn(dev->dev,
			"unable to register MIPI ISR, MIPI errors may go unnoticed\n");

	ret = cif_isp20_pltfrm_irq_register_isr(
		dev->dev,
		CIF_ISP20_IRQ_MI,
		cif_isp20_isr,
		dev);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"unable to register MI ISR, aborting\n");
		goto err;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev, "failed with error %d", ret);
	return ret;
}

/**Public Functions***********************************************************/

int cif_isp20_streamon(
	struct cif_isp20_device *dev,
	u32 stream_ids)
{
	int ret = 0;
	bool streamon_sp = stream_ids & CIF_ISP20_STREAM_SP;
	bool streamon_mp = stream_ids & CIF_ISP20_STREAM_MP;
	bool streamon_dma = stream_ids & CIF_ISP20_STREAM_DMA;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, DMA state = %s, streamon SP = %d, streamon MP = %d, streamon DMA = %d\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_state_string(dev->dma_stream.state),
		streamon_sp, streamon_mp, streamon_dma);

	if (!((streamon_sp &&
		(dev->sp_stream.state != CIF_ISP20_STATE_STREAMING)) ||
		(streamon_mp &&
		(dev->mp_stream.state != CIF_ISP20_STATE_STREAMING))))
		return 0;

	if (streamon_sp &&
		(dev->sp_stream.state != CIF_ISP20_STATE_READY)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"cannot start streaming on SP path, path not yet enabled\n");
		ret = -EFAULT;
		goto err;
	}

	if (streamon_mp && (dev->mp_stream.state != CIF_ISP20_STATE_READY)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"cannot start streaming on MP path, path not yet enabled\n");
		ret = -EFAULT;
		goto err;
	}

	if (streamon_sp &&
		CIF_ISP20_PIX_FMT_IS_RAW_BAYER(
			dev->config.mi_config.mp.output.pix_fmt) &&
		(streamon_mp ||
		(dev->mp_stream.state == CIF_ISP20_STATE_STREAMING))) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"cannot start streaming on SP path when MP is active and set to RAW output\n");
		ret = -EBUSY;
		goto err;
	}

	if (streamon_mp &&
		(dev->sp_stream.state == CIF_ISP20_STATE_STREAMING))
		dev->mp_stream.updt_cfg = true;
	if (streamon_sp &&
		(dev->mp_stream.state == CIF_ISP20_STATE_STREAMING))
		dev->sp_stream.updt_cfg = true;

	if (streamon_sp && dev->sp_stream.updt_cfg &&
		(dev->mp_stream.state == CIF_ISP20_STATE_STREAMING)) {
		ret = cif_isp20_stop(dev, false, true);
		if (IS_ERR_VALUE(ret))
			goto err;
		streamon_mp = true;
		dev->mp_stream.updt_cfg = true;
	}
#ifdef CONFIG_CIF_ISP20_TEST_YC_FLT
	if (streamon_mp) {
		cif_isp20_dbg_count = 0;
		cif_isp20_yc_flt_enable = 1;
	}
#endif
	if (streamon_mp && dev->mp_stream.updt_cfg &&
		(dev->sp_stream.state == CIF_ISP20_STATE_STREAMING)) {
		ret = cif_isp20_stop(dev, true, false);
		if (IS_ERR_VALUE(ret))
			goto err;

		streamon_sp = true;
		dev->sp_stream.updt_cfg = true;
	}

	stream_ids = 0;
	if (streamon_mp && dev->mp_stream.updt_cfg)
		stream_ids |= CIF_ISP20_STREAM_MP;
	if (streamon_sp && dev->sp_stream.updt_cfg)
		stream_ids |= CIF_ISP20_STREAM_SP;

	ret = cif_isp20_config_cif(dev, stream_ids);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_isp20_start(dev, streamon_sp, streamon_mp);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, DMA state = %s\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_state_string(dev->dma_stream.state));

	return 0;
err:
	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, DMA state = %s\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_state_string(dev->dma_stream.state));
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

int cif_isp20_streamoff(
	struct cif_isp20_device *dev,
	u32 stream_ids)
{
	int ret = 0;
	bool streamoff_sp = stream_ids & CIF_ISP20_STREAM_SP;
	bool streamoff_mp = stream_ids & CIF_ISP20_STREAM_MP;
	bool streamoff_dma = stream_ids & CIF_ISP20_STREAM_DMA;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, DMA state = %s, streamoff SP = %d, streamoff MP = %d, streamoff DMA = %d\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_state_string(dev->dma_stream.state),
		streamoff_sp,
		streamoff_mp,
		streamoff_dma);

	if (dev->config.flash_mode != CIF_ISP20_FLASH_MODE_OFF &&
		((streamoff_sp &&
		(dev->mp_stream.state == CIF_ISP20_STATE_INACTIVE)) ||
		(streamoff_mp &&
		(dev->sp_stream.state == CIF_ISP20_STATE_INACTIVE))))
		cif_isp20_img_src_s_ctrl(dev->img_src,
			CIF_ISP20_CID_FLASH_MODE,
			CIF_ISP20_FLASH_MODE_OFF);

	ret = cif_isp20_stop(dev, streamoff_sp, streamoff_mp);
	if (IS_ERR_VALUE(ret))
		goto err;
	if ((streamoff_sp) &&
		(dev->sp_stream.state == CIF_ISP20_STATE_READY))
		dev->sp_stream.state = CIF_ISP20_STATE_INACTIVE;
	if (streamoff_mp) {
		dev->config.mi_config.mp.output.width = 0;
		dev->config.mi_config.mp.output.height = 0;
		dev->config.mi_config.mp.output.pix_fmt =
			CIF_UNKNOWN_FORMAT;
		if (dev->mp_stream.state == CIF_ISP20_STATE_READY)
			dev->mp_stream.state = CIF_ISP20_STATE_INACTIVE;
	}
	if (streamoff_dma) {
		cif_isp20_stop_dma(dev);
		if (dev->dma_stream.state == CIF_ISP20_STATE_READY)
			dev->dma_stream.state = CIF_ISP20_STATE_INACTIVE;
	}
	if ((dev->dma_stream.state <= CIF_ISP20_STATE_INACTIVE) &&
		(dev->mp_stream.state <= CIF_ISP20_STATE_INACTIVE) &&
		(dev->sp_stream.state <= CIF_ISP20_STATE_INACTIVE)) {
		dev->isp_dev.input_width = 0;
		dev->isp_dev.input_height = 0;
		dev->config.isp_config.ism_config.ism_en = 0;
	}

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, DMA state = %s, # frames received = %d\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_state_string(dev->dma_stream.state),
		dev->isp_dev.frame_id >> 1);

	return 0;
err:
	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, DMA state = %s\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_state_string(dev->dma_stream.state));
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

int cif_isp20_suspend(
	struct cif_isp20_device *dev)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state));

	if ((dev->pm_state == CIF_ISP20_PM_STATE_SUSPENDED) ||
		(dev->pm_state == CIF_ISP20_PM_STATE_OFF))
		return 0;

	dev->sp_stream.saved_state = dev->sp_stream.state;
	dev->mp_stream.saved_state = dev->mp_stream.state;
	ret = cif_isp20_stop(dev, true, true);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = cif_isp20_set_pm_state(dev, CIF_ISP20_PM_STATE_SUSPENDED);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = cif_isp20_img_src_set_state(dev, CIF_ISP20_IMG_SRC_STATE_OFF);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

int cif_isp20_resume(
	struct cif_isp20_device *dev)
{
	u32 stream_ids = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state));

	if ((dev->sp_stream.saved_state == CIF_ISP20_STATE_READY) ||
		(dev->sp_stream.saved_state == CIF_ISP20_STATE_STREAMING)) {
		dev->sp_stream.updt_cfg = true;
		dev->sp_stream.state = CIF_ISP20_STATE_READY;
		if (dev->sp_stream.saved_state == CIF_ISP20_STATE_STREAMING)
			stream_ids |= CIF_ISP20_STREAM_SP;
	}
	if ((dev->mp_stream.saved_state == CIF_ISP20_STATE_READY) ||
		(dev->mp_stream.saved_state == CIF_ISP20_STATE_STREAMING)) {
		dev->mp_stream.updt_cfg = true;
		dev->mp_stream.state = CIF_ISP20_STATE_READY;
		if (dev->mp_stream.saved_state == CIF_ISP20_STATE_STREAMING)
			stream_ids |= CIF_ISP20_STREAM_MP;
	}

	if ((dev->dma_stream.saved_state == CIF_ISP20_STATE_READY) ||
		(dev->dma_stream.saved_state == CIF_ISP20_STATE_STREAMING)) {
		dev->dma_stream.state = CIF_ISP20_STATE_READY;
		if (dev->dma_stream.saved_state == CIF_ISP20_STATE_STREAMING)
			stream_ids |= CIF_ISP20_STREAM_DMA;
	}

	return cif_isp20_streamon(dev, stream_ids);
}

int cif_isp20_s_fmt(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream_id,
	struct cif_isp20_strm_fmt *strm_fmt,
	u32 stride)
{
	int ret;
	cif_isp20_pltfrm_pr_dbg(NULL, "%s\n",
		cif_isp20_stream_id_string(stream_id));

	switch (stream_id) {
	case CIF_ISP20_STREAM_SP:
		return cif_isp20_s_fmt_sp(dev, strm_fmt, stride);
	case CIF_ISP20_STREAM_MP:
		return cif_isp20_s_fmt_mp(dev, strm_fmt, stride);
	case CIF_ISP20_STREAM_DMA:
		return cif_isp20_s_fmt_dma(dev, strm_fmt, stride);
	default:
		cif_isp20_pltfrm_pr_err(NULL,
			"unknown/unsupported stream ID %d\n", stream_id);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(NULL,
		"failed with error %d\n", ret);
	return ret;
}

int cif_isp20_init(
	struct cif_isp20_device *dev,
	u32 stream_ids)
{
	int ret;

	cif_isp20_pltfrm_pr_dbg(NULL, "0x%08x\n", stream_ids);

	if (stream_ids & ~(CIF_ISP20_ALL_STREAMS)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"unknown/unsupported stream IDs 0x%08x\n",
			stream_ids);
		ret = -EINVAL;
		goto err;
	}

	/* set default input, failure is not fatal here */
	if ((dev->sp_stream.state == CIF_ISP20_STATE_DISABLED) &&
		(dev->mp_stream.state == CIF_ISP20_STATE_DISABLED)) {
		(void)cif_isp20_s_input(dev, CIF_ISP20_INP_CSI_0);
		dev->config.isp_config.si_enable = false;
		dev->config.isp_config.ie_config.effect =
			CIF_ISP20_IE_NONE;
	}

	if (stream_ids & CIF_ISP20_STREAM_SP)
		cif_isp20_init_stream(dev, CIF_ISP20_STREAM_SP);
	if (stream_ids & CIF_ISP20_STREAM_MP)
		cif_isp20_init_stream(dev, CIF_ISP20_STREAM_MP);
	if (stream_ids & CIF_ISP20_STREAM_DMA)
		cif_isp20_init_stream(dev, CIF_ISP20_STREAM_DMA);

	return 0;
err:
	cif_isp20_pltfrm_pr_err(NULL,
		"failed with error %d\n", ret);
	return ret;
}

int cif_isp20_release(
	struct cif_isp20_device *dev,
	int stream_ids)
{
	int ret;

	cif_isp20_pltfrm_pr_dbg(NULL, "0x%08x\n", stream_ids);

	if ((dev->sp_stream.state == CIF_ISP20_STATE_DISABLED) &&
		(dev->mp_stream.state == CIF_ISP20_STATE_DISABLED) &&
		(dev->dma_stream.state == CIF_ISP20_STATE_DISABLED))
		return 0;

	if (stream_ids & ~(CIF_ISP20_ALL_STREAMS)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"unknown/unsupported stream IDs 0x%08x\n",
			stream_ids);
		ret = -EINVAL;
		goto err;
	}

	if (stream_ids & CIF_ISP20_STREAM_SP) {
		if (dev->sp_stream.state == CIF_ISP20_STATE_STREAMING) {
			cif_isp20_pltfrm_pr_warn(dev->dev,
			"CIF SP in streaming state, should be stopped before release, trying to stop it\n");
			ret = cif_isp20_stop(dev, true, false);
			if (IS_ERR_VALUE(ret))
				goto err;
		}
		dev->sp_stream.state = CIF_ISP20_STATE_DISABLED;
	}
	if (stream_ids & CIF_ISP20_STREAM_MP) {
		if (dev->mp_stream.state == CIF_ISP20_STATE_STREAMING) {
			cif_isp20_pltfrm_pr_warn(dev->dev,
			"CIF MP in streaming state, should be stopped before release, trying to stop it\n");
			ret = cif_isp20_stop(dev, false, true);
			if (IS_ERR_VALUE(ret))
				goto err;
		}
		dev->mp_stream.state = CIF_ISP20_STATE_DISABLED;
	}

	if ((dev->sp_stream.state == CIF_ISP20_STATE_DISABLED) &&
		(dev->mp_stream.state == CIF_ISP20_STATE_DISABLED)) {
		if (IS_ERR_VALUE(cif_isp20_set_pm_state(dev,
			CIF_ISP20_PM_STATE_OFF)))
			cif_isp20_pltfrm_pr_warn(dev->dev,
			"CIF power off failed\n");
		if (dev->img_src != NULL) {
			if (IS_ERR_VALUE(cif_isp20_img_src_set_state(dev,
				CIF_ISP20_IMG_SRC_STATE_OFF)))
				cif_isp20_pltfrm_pr_warn(dev->dev,
					"image source power off failed\n");
			dev->img_src = NULL;
		}
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

struct cif_isp20_device *cif_isp20_create(
	CIF_ISP20_PLTFRM_DEVICE pdev,
	void (*sof_event)(__u32 frame_sequence),
	void (*requeue_bufs)(enum cif_isp20_stream_id stream_id))
{
	int ret;
	struct cif_isp20_device *dev;

	cif_isp20_pltfrm_pr_dbg(NULL, "\n");

	/* Allocate needed structures */
	dev = kzalloc(sizeof(struct cif_isp20_device), GFP_KERNEL);
	if (NULL == dev) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"memory allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}
	dev->sof_event = sof_event;
	dev->requeue_bufs = requeue_bufs;

	ret = cif_isp20_pltfrm_dev_init(dev,
		&pdev, &dev->config.base_addr);
	if (IS_ERR_VALUE(ret))
		goto err;
	cif_isp20_pltfrm_debug_register_print_cb(
		dev->dev,
		(void (*)(void *, const char *))cif_isp20_debug_print_block,
		dev);

	ret = cif_isp20_img_srcs_init(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_isp20_register_isrs(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	(void)cif_isp20_init(dev, CIF_ISP20_ALL_STREAMS);
	dev->pm_state = CIF_ISP20_PM_STATE_OFF;
	dev->sp_stream.state = CIF_ISP20_STATE_DISABLED;
	dev->sp_stream.id = CIF_ISP20_STREAM_SP;
	dev->mp_stream.state = CIF_ISP20_STATE_DISABLED;
	dev->mp_stream.id = CIF_ISP20_STREAM_MP;
	dev->dma_stream.state = CIF_ISP20_STATE_DISABLED;
	dev->dma_stream.id = CIF_ISP20_STREAM_DMA;
	dev->config.mi_config.async_updt = 0;
	cif_isp20_pltfrm_event_init(dev->dev, &dev->dma_stream.done);
	cif_isp20_pltfrm_event_init(dev->dev, &dev->sp_stream.done);
	cif_isp20_pltfrm_event_init(dev->dev, &dev->mp_stream.done);

	/* TBD: clean this up */
	init_output_formats();

	return dev;
err:
	cif_isp20_pltfrm_pr_err(NULL,
		"failed with error %d\n", ret);
	if (!IS_ERR_OR_NULL(dev))
		kfree(dev);
	return ERR_PTR(ret);
}

void cif_isp20_destroy(
	struct cif_isp20_device *dev)
{
	cif_isp20_pltfrm_pr_dbg(NULL, "\n");
	if (!IS_ERR_OR_NULL(dev))
		kfree(dev);
}

int cif_isp20_s_input(
	struct cif_isp20_device *dev,
	enum cif_isp20_inp inp)
{
	int ret;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"setting input to %s\n",
		cif_isp20_inp_string(inp));

	if (inp > CIF_ISP20_INP_DMA_SP) {
		cif_isp20_pltfrm_pr_err(NULL,
			"invalid input %d\n", inp);
		ret = -EINVAL;
		goto err;
	}

	dev->img_src = NULL;

	/* DMA -> ISP or DMA -> IE */
	if ((inp == CIF_ISP20_INP_DMA) || (inp == CIF_ISP20_INP_DMA_IE))
		dev->config.isp_config.input =
			&dev->config.mi_config.dma.output;
	else if (inp < CIF_ISP20_INP_DMA) {
		if (inp == CIF_ISP20_INP_CSI_0) {
			if (NULL == dev->img_src_array[0]) {
				cif_isp20_pltfrm_pr_err(dev->dev,
					"no image source connected to CSI-0\n");
				ret = -EINVAL;
				goto err;
			}
			dev->img_src = dev->img_src_array[0];
			dev->config.mipi_config.input_sel = 0;
		} else if (inp == CIF_ISP20_INP_CSI_1) {
			if (NULL == dev->img_src_array[1]) {
				cif_isp20_pltfrm_pr_err(dev->dev,
					"no image source connected to CSI-1\n");
				ret = -EINVAL;
				goto err;
			}
			dev->img_src = dev->img_src_array[1];
			dev->config.mipi_config.input_sel = 1;
		} else if (inp == CIF_ISP20_INP_CPI) {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"parallel input currently not supported\n");
			ret = -EINVAL;
			goto err;
		}
		dev->config.isp_config.input =
			&dev->config.img_src_output.frm_fmt;
	}
	dev->config.input_sel = inp;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(NULL,
		"failed with error %d\n", ret);
	return ret;
}

const char *cif_isp20_g_input_name(
	struct cif_isp20_device *dev,
	enum cif_isp20_inp inp)
{
	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"get name of input source %s(0x%08x)\n",
		cif_isp20_inp_string(inp), inp);

	if (inp < CIF_ISP20_INP_CPI) {
		if (inp == CIF_ISP20_INP_CSI_0)
			return cif_isp20_img_src_g_name(dev->img_src_array[0]);
		else
			return cif_isp20_img_src_g_name(dev->img_src_array[1]);
	} else
		return cif_isp20_inp_string(inp);
}

int cif_isp20_qbuf(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream,
	struct cif_isp20_buffer *buf)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s\n",
		cif_isp20_stream_id_string(stream));

	switch (stream) {
	case CIF_ISP20_STREAM_SP:
		list_add_tail(&buf->queue, &dev->sp_stream.buf_queue);
		break;
	case CIF_ISP20_STREAM_MP:
		list_add_tail(&buf->queue, &dev->mp_stream.buf_queue);
		break;
	case CIF_ISP20_STREAM_DMA:
		list_add_tail(&buf->queue, &dev->dma_stream.buf_queue);
		if ((dev->dma_stream.state == CIF_ISP20_STATE_STREAMING) &&
			!CIF_ISP20_MI_IS_BUSY(dev))
			cif_isp20_dma_next_buff(dev);
		break;
	case CIF_ISP20_STREAM_ISP:
		BUG();
		break;
	default:
		cif_isp20_pltfrm_pr_err(dev->dev,
			"unknown stream %d\n", stream);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
}

int cif_isp20_get_target_frm_size(
	struct cif_isp20_device *dev,
	u32 *target_width,
	u32 *target_height)
{
	if (dev->sp_stream.state >= CIF_ISP20_STATE_READY) {
		if (dev->mp_stream.state >= CIF_ISP20_STATE_READY) {
			u32 mp_width, mp_height, sp_width, sp_height;
			u32 input_width, input_height;

			input_width = dev->config.isp_config.input->width;
			input_height = dev->config.isp_config.input->height;
			mp_width = dev->config.mi_config.mp.output.width;
			mp_height = dev->config.mi_config.mp.output.height;
			sp_width = dev->config.mi_config.sp.output.width;
			sp_height = dev->config.mi_config.sp.output.height;

			*target_height = mp_height;
			*target_width = mp_width;
			/* keep MP's aspect ratio and create envelope */
			if (mp_height < sp_height) {
				*target_height = sp_height;
				*target_width = (*target_height * mp_width /
					mp_height + 1) & (~1);
			}
			if (*target_width < sp_width) {
				*target_width = sp_width;
				*target_height = (*target_width * mp_height /
					mp_width + 1) & (~1);
			}
			/* target size must be smaller or equal to input size */
			if (*target_height > input_height) {
				*target_height = input_height;
				*target_width = (*target_height * mp_width /
					mp_height + 1) & (~1);
			}
			if (*target_width > input_width) {
				*target_width = input_width;
				*target_height = (*target_width * mp_height /
					mp_width + 1) & (~1);
			}

			cif_isp20_pltfrm_pr_dbg(dev->dev,
				"ISP %dx%d, MP %dx%d, SP %dx%d, target %dx%d\n",
				input_width, input_height,
				mp_width, mp_height,
				sp_width, sp_height,
				*target_width, *target_height);
		} else {
			*target_width =
				dev->config.mi_config.sp.output.width;
			*target_height =
				dev->config.mi_config.sp.output.height;
			cif_isp20_pltfrm_pr_dbg(dev->dev,
				"SP %dx%d, target %dx%d\n",
				dev->config.mi_config.sp.output.width,
				dev->config.mi_config.sp.output.height,
				*target_width, *target_height);
		}
	} else if (dev->mp_stream.state >= CIF_ISP20_STATE_READY) {
		*target_width = dev->config.mi_config.mp.output.width;
		*target_height = dev->config.mi_config.mp.output.height;
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"MP %dx%d, target %dx%d\n",
			dev->config.mi_config.mp.output.width,
			dev->config.mi_config.mp.output.height,
			*target_width, *target_height);
	} else {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"cannot get target frame size, no path ready\n");
		return -EFAULT;
	}

	return 0;
}

int cif_isp20_calc_isp_cropping(
	struct cif_isp20_device *dev,
	u32 *width,
	u32 *height,
	u32 *h_offs,
	u32 *v_offs)
{
	int ret = 0;
	u32 input_width;
	u32 input_height;
	u32 target_width;
	u32 target_height;

	if (IS_ERR_OR_NULL(dev->config.isp_config.input)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"no input selected for ISP\n");
		ret = -EFAULT;
		goto err;
	}

	input_width = dev->config.isp_config.input->width;
	input_height = dev->config.isp_config.input->height;

	ret = cif_isp20_get_target_frm_size(dev,
		&target_width, &target_height);
	if (IS_ERR_VALUE(ret))
		goto err;

	*width = input_width;
	*height = input_width * target_height / target_width;
	*v_offs = 0;
	*h_offs = 0;
	*height &= ~1;
	if (*height < input_height)
		/* vertical cropping */
		*v_offs = (input_height - *height) >> 1;
	else if (*height > input_height) {
		/* horizontal cropping */
		*height = input_height;
		*width = input_height * target_width / target_height;
		*width &= ~1;
		*h_offs = (input_width - *width) >> 1;
	}

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%dx%d -> %dx%d@(%d,%d)\n",
		input_width, input_height,
		*width, *height,
		*h_offs, *v_offs);

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
}

int cif_isp20_calc_min_out_buff_size(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream_id,
	u32 *size)
{
	int ret = 0;
	enum cif_isp20_pix_fmt pix_fmt;
	u32 llength;
	u32 height;
	u32 bpp;
	struct cif_isp20_mi_path_config *mi_path;
	struct cif_isp20_stream *stream;

	cif_isp20_pltfrm_pr_dbg(NULL,
		"%s\n",
		cif_isp20_stream_id_string(stream_id));

	if (stream_id == CIF_ISP20_STREAM_SP) {
		mi_path = &dev->config.mi_config.sp;
		stream = &dev->sp_stream;
	} else if (stream_id == CIF_ISP20_STREAM_MP) {
		mi_path = &dev->config.mi_config.mp;
		stream = &dev->mp_stream;
	} else if (stream_id == CIF_ISP20_STREAM_DMA) {
		mi_path = &dev->config.mi_config.dma;
		stream = &dev->dma_stream;
	} else {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"cannot calculate buffer size for this stream (%s)\n",
			cif_isp20_stream_id_string(stream_id));
		ret = -EINVAL;
		goto err;
	}

	if (stream->state < CIF_ISP20_STATE_READY) {
		cif_isp20_pltfrm_pr_err(NULL,
			"cannot calculate buffer size, %s stream not ready\n",
			cif_isp20_stream_id_string(stream_id));
		ret = -EINVAL;
		goto err;
	}
	pix_fmt = mi_path->output.pix_fmt;
	llength = mi_path->llength;
	height = mi_path->output.height;

	if (CIF_ISP20_PIX_FMT_IS_RAW_BAYER(pix_fmt) &&
		CIF_ISP20_PIX_FMT_GET_BPP(pix_fmt) > 8)
		/* RAW input > 8BPP is stored with 16BPP by MI */
		bpp = 16;
	else
		bpp = CIF_ISP20_PIX_FMT_GET_BPP(pix_fmt);
	*size = llength * height * bpp / 8;

	cif_isp20_pltfrm_pr_dbg(NULL,
		"calculated buffer size: %d\n",
		*size);

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
}

int cif_isp20_s_ctrl(
	struct cif_isp20_device *dev,
	const enum cif_isp20_cid id,
	int val)
{
	cif_isp20_pltfrm_pr_dbg(NULL,
		"id %d, val %d\n",
		id, val);

	switch (id) {
	case CIF_ISP20_CID_SUPER_IMPOSE:
		dev->config.isp_config.si_enable = val;
		break;
	case CIF_ISP20_CID_IMAGE_EFFECT:
		if ((u32)val > CIF_ISP20_IE_NONE) {
			cif_isp20_pltfrm_pr_err(NULL,
				"unknown/unsupported image effect %d\n", val);
			return -EINVAL;
		}
		dev->config.isp_config.ie_config.effect = val;
		break;
	case CIF_ISP20_CID_JPEG_QUALITY:
		if ((u32)val > 100) {
			cif_isp20_pltfrm_pr_err(NULL,
				"JPEG quality (%d) must be in [1..100]\n", val);
			return -EINVAL;
		}
		dev->config.jpeg_config.ratio = val;
		break;
	case CIF_ISP20_CID_FLASH_MODE:
		if ((u32)val > CIF_ISP20_FLASH_MODE_TORCH) {
			cif_isp20_pltfrm_pr_err(NULL,
				"unknown/unsupported flash mode (%d)\n", val);
			return -EINVAL;
		}
		dev->config.flash_mode = val;
		if (dev->img_src_state == CIF_ISP20_IMG_SRC_STATE_STREAMING)
			cif_isp20_img_src_s_ctrl(dev->img_src,
				CIF_ISP20_CID_FLASH_MODE,
				dev->config.flash_mode);
		break;
	case CIF_ISP20_CID_WB_TEMPERATURE:
	case CIF_ISP20_CID_ANALOG_GAIN:
	case CIF_ISP20_CID_EXPOSURE_TIME:
	case CIF_ISP20_CID_BLACK_LEVEL:
	case CIF_ISP20_CID_FOCUS_ABSOLUTE:
	case CIF_ISP20_CID_AUTO_N_PRESET_WHITE_BALANCE:
	case CIF_ISP20_CID_SCENE_MODE:
	case CIF_ISP20_CID_AUTO_FPS:
	case CIF_ISP20_CID_HFLIP:
	case CIF_ISP20_CID_VFLIP:
	case CIF_ISP20_CID_3A_LOCK:
		return cif_isp20_img_src_s_ctrl(dev->img_src,
			id, val);
	default:
		cif_isp20_pltfrm_pr_err(dev->dev,
			"unknown/unsupported control %d\n", id);
		return -EINVAL;
	}

	return 0;
}

/* ======================================================================== */

enum {
	isp_data_loss = 0,
	isp_pic_size_err,
	mipi_fifo_err,
	dphy_err_sot,
	dphy_err_sot_sync,
	dphy_err_eot_sync,
	dphy_err_ctrl,
	csi_err_protocol,
	csi_ecc1_err,
	csi_ecc2_err,
	csi_cs_err,
};

static void marvin_hw_restart(struct cif_isp20_device *dev)
{
	cif_isp20_pltfrm_pr_dbg(NULL, "\n");

	marvin_hw_errors[isp_pic_size_err].count = 0;
	marvin_hw_errors[isp_data_loss].count = 0;
	marvin_hw_errors[csi_err_protocol].count = 0;
	marvin_hw_errors[csi_ecc1_err].count = 0;
	marvin_hw_errors[csi_ecc2_err].count = 0;
	marvin_hw_errors[csi_cs_err].count = 0;
	cif_iowrite32(0x00000841, dev->config.base_addr + CIF_IRCL);
	cif_iowrite32(0x0, dev->config.base_addr + CIF_IRCL);

	/* enable csi protocol errors interrupts*/
	cif_iowrite32OR(CIF_MIPI_ERR_CSI,
			dev->config.base_addr + CIF_MIPI_IMSC);
	/* enable dphy errors interrupts*/
	cif_iowrite32OR(CIF_MIPI_ERR_DPHY,
			dev->config.base_addr + CIF_MIPI_IMSC);
	/* add fifo error*/
	cif_iowrite32OR(CIF_MIPI_SYNC_FIFO_OVFLW(3),
			dev->config.base_addr + CIF_MIPI_IMSC);
	/* add data overflow_error*/
	cif_iowrite32OR(CIF_MIPI_ADD_DATA_OVFLW,
			dev->config.base_addr + CIF_MIPI_IMSC);

	cif_iowrite32(0x0,
		      dev->config.base_addr + CIF_MI_MP_Y_OFFS_CNT_INIT);
	cif_iowrite32(0x0,
		      dev->config.base_addr +
		      CIF_MI_MP_CR_OFFS_CNT_INIT);
	cif_iowrite32(0x0,
		      dev->config.base_addr +
		      CIF_MI_MP_CB_OFFS_CNT_INIT);
	cif_iowrite32(0x0,
		      dev->config.base_addr + CIF_MI_SP_Y_OFFS_CNT_INIT);
	cif_iowrite32(0x0,
		      dev->config.base_addr +
		      CIF_MI_SP_CR_OFFS_CNT_INIT);
	cif_iowrite32(0x0,
		      dev->config.base_addr +
		      CIF_MI_SP_CB_OFFS_CNT_INIT);
	cif_iowrite32OR(CIF_MI_CTRL_INIT_OFFSET_EN,
			dev->config.base_addr + CIF_MI_CTRL);

	/* Enable ISP ! */
	cif_iowrite32OR(CIF_ISP_CTRL_ISP_CFG_UPD |
			CIF_ISP_CTRL_ISP_INFORM_ENABLE |
			CIF_ISP_CTRL_ISP_ENABLE,
			dev->config.base_addr + CIF_ISP_CTRL);
	/*enable MIPI */
	cif_iowrite32OR(CIF_MIPI_CTRL_OUTPUT_ENA,
			dev->config.base_addr + CIF_MIPI_CTRL);
}

int marvin_mipi_isr(void *cntxt)
{
	struct cif_isp20_device *dev =
	    (struct cif_isp20_device *)cntxt;
	unsigned int mipi_mis = 0;
	unsigned int i = 0;

	mipi_mis =
	    cif_ioread32(dev->config.base_addr + CIF_MIPI_MIS);

	cif_isp20_pltfrm_rtrace_printf(dev->dev,
		"MIPI_MIS %08x, MIPI_RIS %08x, MIPI_IMSC %08x\n",
		mipi_mis,
		cif_ioread32(dev->config.base_addr + CIF_MIPI_RIS),
		cif_ioread32(dev->config.base_addr + CIF_MIPI_IMSC));

	if (mipi_mis & CIF_MIPI_ERR_DPHY) {
		unsigned int mipi_ris =
			cif_ioread32(dev->config.base_addr +
						CIF_MIPI_RIS);
		/* clear_mipi_dphy_error*/
		cif_iowrite32(CIF_MIPI_ERR_DPHY,
			      dev->config.base_addr + CIF_MIPI_ICR);

		for (i = 0;
		     i <
		     (sizeof(marvin_hw_errors) /
		      sizeof(struct smarvin_hw_errors)); i++) {
			if (marvin_hw_errors[i].type != 1)
				continue;	/*skip if not mipi dphy error*/

			if (marvin_hw_errors[i].
			    mask & (mipi_mis & CIF_MIPI_ERR_DPHY)) {
				marvin_hw_errors[i].count++;
				cif_isp20_pltfrm_pr_err(dev->dev,
					"CIF_MIPI_ERR_DPHY 0x%x\n", mipi_ris);
				break;
			}
		}
	}
	if (mipi_mis & CIF_MIPI_ERR_CSI) {
		unsigned int mipi_ris =
			cif_ioread32(dev->config.base_addr +
						CIF_MIPI_RIS);
		/*clear_mipi_csi_error*/
		cif_iowrite32(CIF_MIPI_ERR_CSI,
			      dev->config.base_addr + CIF_MIPI_ICR);

		for (i = 0;
		     i <
		     (sizeof(marvin_hw_errors) /
		      sizeof(struct smarvin_hw_errors)); i++) {
			if (marvin_hw_errors[i].type != 2)
				continue;	/*skip if not mipi csi error*/

			if (marvin_hw_errors[i].
			    mask & (mipi_mis & CIF_MIPI_ERR_CSI)) {
				marvin_hw_errors[i].count++;
				cif_isp20_pltfrm_pr_err(dev->dev,
					"CIF_MIPI_ERR_CSI 0x%x\n", mipi_ris);
				break;
			}
		}
	}
	if ((mipi_mis & CIF_MIPI_SYNC_FIFO_OVFLW(3))) {

		/* clear_mipi_fifo_error*/
		cif_iowrite32(CIF_MIPI_SYNC_FIFO_OVFLW(3),
			      dev->config.base_addr + CIF_MIPI_ICR);
		cif_iowrite32OR(CIF_MIPI_CTRL_OUTPUT_ENA,
				dev->config.base_addr + CIF_MIPI_CTRL);
		for (i = 0;
		     i <
		     (sizeof(marvin_hw_errors) /
		      sizeof(struct smarvin_hw_errors)); i++) {
			if (marvin_hw_errors[i].type != 2)
				continue;	/*skip if not mipi error*/

			if (marvin_hw_errors[i].
			    mask & (mipi_mis & CIF_MIPI_SYNC_FIFO_OVFLW(3))) {
				u32 mi_status =
					cif_ioread32(dev->config.base_addr +
					CIF_MI_STATUS);
				marvin_hw_errors[i].count++;
				if (mi_status &
					(CIF_MI_STATUS_MP_Y_FIFO_FULL |
					CIF_MI_STATUS_SP_Y_FIFO_FULL))
					cif_isp20_pltfrm_pr_err(dev->dev,
						"CIF_MIPI_SYNC_FIFO_OVFLW, backpressure (0x%08x)\n",
						mi_status);
				else
					cif_isp20_pltfrm_pr_err(dev->dev,
						"CIF_MIPI_SYNC_FIFO_OVFLW 0x%x\n",
						cif_ioread32(
							dev->config.base_addr +
							CIF_MIPI_RIS));
				break;
			}
		}
	}
	if (mipi_mis & CIF_MIPI_ADD_DATA_OVFLW) {
		/* clear_mipi_fifo_error*/
		cif_iowrite32(CIF_MIPI_ADD_DATA_OVFLW,
				  dev->config.base_addr + CIF_MIPI_ICR);

		for (i = 0;
		     i <
		     (sizeof(marvin_hw_errors) /
		      sizeof(struct smarvin_hw_errors)); i++) {
			if (marvin_hw_errors[i].type != 1)
				continue;	/*skip if not mipi error*/

			if (marvin_hw_errors[i].
			    mask & (mipi_mis & CIF_MIPI_ADD_DATA_OVFLW)) {
				marvin_hw_errors[i].count++;
				cif_isp20_pltfrm_pr_err(dev->dev,
					"CIF_MIPI_ADD_DATA_OVFLW");
				break;
			}
		}
	}

	return 0;
}

/* ======================================================================== */
int marvin_isp_isr(void *cntxt)
{
	struct cif_isp20_device *dev =
	    (struct cif_isp20_device *)cntxt;
	unsigned int isp_mis = 0;
	unsigned int isp_err = 0;

	cif_isp20_pltfrm_pr_dbg(NULL, "\n");

	isp_mis = cif_ioread32(dev->config.base_addr + CIF_ISP_MIS);

	cif_isp20_pltfrm_rtrace_printf(dev->dev,
		"ISP_MIS 0x%08x, ISP_RIS 0x%08x, ISP_IMSC 0x%08x\n",
		isp_mis,
		cif_ioread32(dev->config.base_addr + CIF_ISP_RIS),
		cif_ioread32(dev->config.base_addr + CIF_ISP_IMSC));

	if ((isp_mis & (CIF_ISP_DATA_LOSS | CIF_ISP_PIC_SIZE_ERROR))) {
		dev->sp_stream.stall = true;
		dev->mp_stream.stall = true;
		if ((isp_mis & CIF_ISP_PIC_SIZE_ERROR)) {
			/* Clear pic_size_error */
			cif_iowrite32(CIF_ISP_PIC_SIZE_ERROR,
				dev->config.base_addr +
				CIF_ISP_ICR);
			marvin_hw_errors[isp_pic_size_err].count++;
			isp_err =
			    cif_ioread32(dev->config.base_addr +
					 CIF_ISP_ERR);
			dev_err(dev->dev,
				"CIF_ISP_PIC_SIZE_ERROR (0x%08x)",
				isp_err);
			cif_iowrite32(isp_err,
				dev->config.base_addr +
				CIF_ISP_ERR_CLR);
		} else if ((isp_mis & CIF_ISP_DATA_LOSS)) {
			/* Clear data_loss */
			cif_iowrite32(CIF_ISP_DATA_LOSS,
				dev->config.base_addr +
				CIF_ISP_ICR);
			marvin_hw_errors[isp_data_loss].count++;
			dev_err(dev->dev,
				"CIF_ISP_DATA_LOSS\n");
			cif_iowrite32(CIF_ISP_DATA_LOSS,
				      dev->config.base_addr +
				      CIF_ISP_ICR);
		}
		/* Stop ISP */
		cif_iowrite32AND(~CIF_ISP_CTRL_ISP_INFORM_ENABLE &
				~CIF_ISP_CTRL_ISP_ENABLE,
				dev->config.base_addr + CIF_ISP_CTRL);
		/*   isp_update */
		cif_iowrite32OR(CIF_ISP_CTRL_ISP_CFG_UPD,
				dev->config.base_addr + CIF_ISP_CTRL);
		marvin_hw_restart(dev);
	}

	if (isp_mis & CIF_ISP_V_START) {
		do_gettimeofday(&dev->curr_frame_time);
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"ISR:VS\n");
		dev->b_isp_frame_in = false;
		dev->b_mi_frame_end = false;
		cifisp_v_start(&dev->isp_dev, &dev->curr_frame_time);
		cif_iowrite32(~0,
		      dev->config.base_addr + CIF_ISP_ICR);
		cif_iowrite32(~0,
		      dev->config.base_addr + CIF_MI_ICR);
		cif_iowrite32AND(~(CIF_MI_SP_FRAME | CIF_MI_MP_FRAME),
		      dev->config.base_addr + CIF_MI_IMSC);
		if (!dev->config.mi_config.async_updt) {
			cif_iowrite32OR(CIF_ISP_CTRL_ISP_GEN_CFG_UPD,
				dev->config.base_addr + CIF_ISP_CTRL);
			cif_isp20_pltfrm_pr_dbg(NULL,
				"CIF_ISP_CTRL_ISP_GEN_CFG_UPD\n");
		}
		if (dev->sof_event)
			dev->sof_event(dev->isp_dev.frame_id >> 1);
		isp_mis = 0;
	}

	if (isp_mis & CIF_ISP_FRAME_IN) {
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"ISR:FI\n");
		if (dev->b_mi_frame_end)
			cif_isp20_update_ism_ycflt_rsz(dev);
		cif_iowrite32(CIF_ISP_FRAME_IN,
			dev->config.base_addr + CIF_ISP_ICR);
		dev->b_isp_frame_in = true;
	}

	if (isp_mis & CIF_ISP_FRAME) {
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"ISR:FO\n");
		/* Clear Frame In (ISP) */
		cif_iowrite32(CIF_ISP_FRAME,
			dev->config.base_addr + CIF_ISP_ICR);

		cif_iowrite32OR(CIF_MI_SP_FRAME | CIF_MI_MP_FRAME,
				dev->config.base_addr + CIF_MI_IMSC);
		/* restart MI if CIF has run out of buffers */
		if ((dev->config.input_sel < CIF_ISP20_INP_DMA) &&
			!CIF_ISP20_MI_IS_BUSY(dev) &&
			!dev->config.jpeg_config.busy &&
			(dev->config.mi_config.async_updt ||
			(!dev->sp_stream.next_buf &&
			!dev->mp_stream.next_buf))) {
			u32 mi_isr = 0;
			if (dev->sp_stream.state == CIF_ISP20_STATE_STREAMING)
				mi_isr |= CIF_MI_SP_FRAME;
			if (dev->mp_stream.state == CIF_ISP20_STATE_STREAMING)
				mi_isr |= CIF_MI_MP_FRAME;
			cif_iowrite32(mi_isr,
				dev->config.base_addr + CIF_MI_ISR);
		}
	}

	if (cif_isp20_check_for_ovs(dev, __LINE__))
		return 0;
	cifisp_isp_isr(&dev->isp_dev, isp_mis);

	return 0;
}

/* ======================================================================== */

void init_output_formats(void)
{
	unsigned int i = 0;
	int ret = 0;		/* RF*/
	int xgold_num_format = 0;	/*RF*/

	xgold_num_format =
	    (sizeof(xgold_output_format) / sizeof(struct xgold_fmt));

	for (i = 0; i < xgold_num_format; i++) {
		struct v4l2_fmtdesc fmtdesc;
		memset(&fmtdesc, 0, sizeof(fmtdesc));
		fmtdesc.index = i;
		fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		strlcpy((&fmtdesc)->description,
			xgold_output_format[(&fmtdesc)->index].name,
			sizeof((&fmtdesc)->description));
		(&fmtdesc)->pixelformat =
		    xgold_output_format[(&fmtdesc)->index].fourcc;
		(&fmtdesc)->flags =
		    xgold_output_format[(&fmtdesc)->index].flags;

		if (ret < 0)
			break;

		output_formats[i] = fmtdesc;
	}
}

int get_xgold_output_format_size(void)
{
	return sizeof(xgold_output_format) / sizeof(struct xgold_fmt);
}

struct xgold_fmt *get_xgold_output_format(int index)
{
	struct xgold_fmt *fmt = NULL;

	if ((index >= 0) && (index < get_xgold_output_format_size()))
		fmt = &xgold_output_format[index];

	return fmt;
}

struct v4l2_fmtdesc *get_xgold_output_format_desc(int index)
{
	struct v4l2_fmtdesc *desc = NULL;

	if ((index >= 0) && (index < get_xgold_output_format_desc_size()))
		desc = &output_formats[index];

	return desc;
}

int get_xgold_output_format_desc_size(void)
{
	return ARRAY_SIZE(xgold_output_format);
}

/* TODO: add support for native PM, but not in this file */
#ifdef PM_LEGACY_TO_BE_CLEANED_UP

#include <linux/of.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/platform_device_pm.h>
#endif

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int xgold_v4l2_set_pm_native_regulators(struct device *dev)
{
	int ret = 0;

#if 0
	xgold_v4l2->regulator_laux1 =
	    regulator_get(&pdev->dev, "ciflaux1");
	if (ret) {
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF can't set laux1 voltage\n",
			DRIVER_NAME, __func__);
		return ret;
	}
	if (IS_ERR(xgold_v4l2->regulator_laux1)) {
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF can't get cif laux1 supply\n",
			DRIVER_NAME, __func__);
		xgold_v4l2->regulator_laux1 = NULL;
	} else {
		ret = regulator_enable(xgold_v4l2->regulator_laux1);
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF %s regulator enabled\n",
			DRIVER_NAME, __func__, "ciflaux1");
	}
	xgold_v4l2->regulator_laux =
	    regulator_get(&pdev->dev, "cifexternal");
	if (IS_ERR(xgold_v4l2->regulator_laux)) {
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF can't get cif external supply\n",
			DRIVER_NAME, __func__);
		xgold_v4l2->regulator_laux = NULL;
	} else {
		ret = regulator_enable(xgold_v4l2->regulator_laux);
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF %s regulator enabled\n",
			DRIVER_NAME, __func__, "cifexternal");
	}
	xgold_v4l2->regulator_lmipi_ana = regulator_get(&pdev->dev,
							"cifmipi");
	if (IS_ERR(xgold_v4l2->regulator_lmipi_ana)) {
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF can't get cif mipi_ana supply\n",
			DRIVER_NAME, __func__);
		xgold_v4l2->regulator_lmipi_ana = NULL;
	} else {
		ret = regulator_enable(xgold_v4l2->regulator_lmipi_ana);
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF %s regulator enabled\n",
			DRIVER_NAME, __func__, "cifmipi");
	}
#endif
	xgold_v4l2->regulator_lcif = regulator_get(xgold_v4l2->dev, "cif");
	if (IS_ERR(xgold_v4l2->regulator_lcif)) {
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF can't get cif supply\n",
			DRIVER_NAME, __func__);
		xgold_v4l2->regulator_lcif = NULL;
	} else {
		ret = regulator_enable(xgold_v4l2->regulator_lcif);
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF %s regulator enabled\n",
			DRIVER_NAME, __func__, "cif");
	}
	xgold_v4l2->regulator_lcsi1 = regulator_get(xgold_v4l2->dev,
						    "cifcsi1");
	if (IS_ERR(xgold_v4l2->regulator_lcsi1)) {
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF can't get cif csi1 supply\n",
			DRIVER_NAME, __func__);
		xgold_v4l2->regulator_lcsi1 = NULL;
	} else {
		ret = regulator_enable(xgold_v4l2->regulator_lcsi1);
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF %s regulator enabled\n",
			DRIVER_NAME, __func__, "cifcsi1");
	}
	xgold_v4l2->regulator_lcsi2 = regulator_get(xgold_v4l2->dev,
						    "cifcsi2");
	if (IS_ERR(xgold_v4l2->regulator_lcsi2)) {
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF can't get cif csi2 supply\n",
			DRIVER_NAME, __func__);
		xgold_v4l2->regulator_lcsi2 = NULL;
	} else {
		ret = regulator_enable(xgold_v4l2->regulator_lcsi2);
		xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			" %s: %s:  CIF %s regulator enabled\n",
			DRIVER_NAME, __func__, "cifcsi2");
	}
	return ret;
}
static int xgold_v4l2_set_pm_native_clocks(struct device *dev)
{
	int ret = 0;
	unsigned long rate = 0;
	clk_prepare_enable(xgold_v4l2->clk_master);
	rate = clk_get_rate(xgold_v4l2->clk_master);
	xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			 " %s: %s:  CIF %s = %ld\n",
			 DRIVER_NAME, __func__, OF_MASTER_CLK, rate);
	clk_prepare_enable(xgold_v4l2->clk_kernel);
	rate = clk_get_rate(xgold_v4l2->clk_kernel);
	xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			 " %s: %s:  CIF %s = %ld\n",
			 DRIVER_NAME, __func__, OF_KERNEL_CLK, rate);
	clk_prepare_enable(xgold_v4l2->clk_slave);
	rate = clk_get_rate(xgold_v4l2->clk_slave);
	xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			 " %s: %s:  CIF %s = %ld\n",
			 DRIVER_NAME, __func__, OF_SLAVE_CLK, rate);
	clk_prepare_enable(xgold_v4l2->clk_sensor);
	rate = clk_get_rate(xgold_v4l2->clk_sensor);
	xgold_v4l2_debug(XGOLD_V4L2_ERROR,
			 " %s: %s:  CIF %s = %ld\n",
			 DRIVER_NAME, __func__, OF_SENSOR_CLK, rate);
	return ret;
}

static int xgold_cif_set_pm_state(struct device *_dev,
				struct device_state_pm_state *state)
{
	int id = device_state_pm_get_state_id(_dev, state->name);

	switch (id) {
	case PM_STATE_D0:
		/*FIXME: The PM native code is absolutely NOT correct
		 * The regulator and clocks have to be parsed ONCE
		 * in the probe, then their handlers to be stored
		 * in the CIF platform data....
		 * Reda Fenjiro MUST fix this mess
		 * */
		dev_err(_dev, " Reda Fenjiro must fix the PM native support\n");
		BUG();
/*
		xgold_v4l2_set_pm_native_regulators(dev);
		xgold_v4l2_set_pm_native_clocks(dev);
*/
		break;

	case PM_STATE_D3:
		/* TODO */
		break;
	};

}

struct device_state_pm_state cif_pm_states[] = {
	       {.name = "high_perf", }, /* D0 */
	       {.name = "disable", } /* D3 */
};

struct device_state_pm_state *xgold_cif_get_initial_state(struct device *dev)
{
	return &cif_pm_states[PM_STATE_D3];
}

static struct device_state_pm_ops cif_pm_ops = {
	.set_state = xgold_cif_set_pm_state,
	.get_initial_state = xgold_cif_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(cif);

#ifdef SOFIA_ES1_BU_PM_NATIVE
#define OF_KERNEL_CLK			"clk_kernel"
#define OF_SLAVE_CLK			"clk_slave"
#define OF_MASTER_CLK			"clk_master"
#define OF_SENSOR_CLK			"clk_sensor"

int xgold_v4l2_parse_clock_dt(struct platform_device *pdev,
			struct device_node *v4l2_cif_node) {
	struct cif_isp20_device *cam = platform_get_drvdata(pdev);
	cam->clk_kernel = of_clk_get_by_name(v4l2_cif_node, OF_KERNEL_CLK);
	if (IS_ERR(cam->clk_kernel)) {
		xgold_v4l2_debug(XGOLD_V4L2_ERROR, "Clk %s not found\n",
				 OF_KERNEL_CLK);
		return -EINVAL;
	}
	cam->clk_slave = of_clk_get_by_name(v4l2_cif_node, OF_SLAVE_CLK);
	if (IS_ERR(cam->clk_slave)) {
		xgold_v4l2_debug(XGOLD_V4L2_ERROR, "Clk %s not found\n",
				 OF_SLAVE_CLK);
		return -EINVAL;
	}
	cam->clk_sensor = of_clk_get_by_name(v4l2_cif_node, OF_SENSOR_CLK);
	if (IS_ERR(cam->clk_sensor)) {
		xgold_v4l2_debug(XGOLD_V4L2_ERROR, "Clk %s not found\n",
				 OF_SENSOR_CLK);
		return -EINVAL;
	}
	cam->clk_master = of_clk_get_by_name(v4l2_cif_node, OF_MASTER_CLK);
	if (IS_ERR(cam->clk_master)) {
		xgold_v4l2_debug(XGOLD_V4L2_ERROR, "Clk %s not found\n",
				 OF_MASTER_CLK);
	}
		return -EINVAL;
	return 0;
}
#endif
#endif
#endif

