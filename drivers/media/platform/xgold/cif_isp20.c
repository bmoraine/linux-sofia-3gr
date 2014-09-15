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

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/videodev2.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-dma-contig.h>
#include <linux/mm.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "cif_isp20_regs.h"
#include "cif_isp20.h"

#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/platform_device_pm.h>
#endif

#if !defined(CONFIG_CIF_ISP_AUTO_UPD_CFG_BUG)
/* With the CIF ISP bug we were not yet able to test the code for the
    synchronous disabling/enabling of the MI. Once fixed hardware is available
    all the code that is actived when !defined(CONFIG_CIF_ISP_AUTO_UPD_CFG_BUG)
    must be tested first! */
#error "This would cause currently untested code to be activated. Test first!"
#endif

/*
#define MEASURE_VERTICAL_BLANKING
*/

static int update_mi_sp(
	struct cif_isp20_device *dev);
static int update_mi_mp(
	struct cif_isp20_device *dev);
static int marvin_mipi_isr(
	void *cntxt);
static int marvin_isp_isr(
	void *cntxt);
static int marvin_lib_program_jpeg_tables(
	struct cif_isp20_device *dev);
static int marvin_lib_select_jpeg_tables(
	struct cif_isp20_device *dev);
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
static const unsigned char yq_table_base[] = {
	16, 11, 10, 16, 24, 40, 51, 61,
	12, 12, 14, 19, 26, 58, 60, 55,
	14, 13, 16, 24, 40, 57, 69, 56,
	14, 17, 22, 29, 51, 87, 80, 62,
	18, 22, 37, 56, 68, 109, 103, 77,
	24, 35, 55, 64, 81, 104, 113, 92,
	49, 64, 78, 87, 103, 121, 120, 101,
	72, 92, 95, 98, 112, 100, 103, 99
};

/* chroma */
static const unsigned char uvq_table_base[] = {
	17, 18, 24, 47, 99, 99, 99, 99,
	18, 21, 26, 66, 99, 99, 99, 99,
	24, 26, 56, 99, 99, 99, 99, 99,
	47, 66, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99
};

#if 0
static struct xgold_fmt xgold_input_format[] = {
{
	.name		= "YUV422-Interleaved",
	.fourcc	= V4L2_PIX_FMT_YUYV,
	.flags	= 0,
	.depth	= 16,
},
{
	.name   = "BAYER-RGB8",
	.fourcc  = V4L2_PIX_FMT_SGRBG8,
	.flags	    = 0,
	.depth  = 8,
},
{
	.name	= "BAYER-RGB10",
	.fourcc        = V4L2_PIX_FMT_SGRBG10,
	.flags		= 0,
	.depth	= 16,
}
};
#endif

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


/**Defines********************************************************************/

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
	enum cif_isp20_state pm_state)
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
	switch (inp & ~CIF_ISP20_INP_SI) {
	case CIF_ISP20_INP_CSI_0:
		if (inp & CIF_ISP20_INP_SI)
			return "CSI-0 + Superimpose";
		else
			return "CSI-0";
	case CIF_ISP20_INP_CSI_1:
		if (inp & CIF_ISP20_INP_SI)
			return "CSI-1 + Superimpose";
		else
			return "CSI-1";
	case CIF_ISP20_INP_CPI:
		if (inp & CIF_ISP20_INP_SI)
			return "CPI + Superimpose";
		else
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

static int cif_isp20_get_target_frm_size(
	struct cif_isp20_device *dev,
	u32 *target_width,
	u32 *target_height)
{
	if (dev->sp_stream.state >= CIF_ISP20_STATE_READY) {
		if ((dev->mp_stream.state >= CIF_ISP20_STATE_READY) &&
			(dev->config.mi_config.mp.output.width >
			dev->config.mi_config.sp.output.width))
			*target_width =
				dev->config.mi_config.mp.output.width;
		else
			*target_width =
				dev->config.mi_config.sp.output.width;
		if ((dev->mp_stream.state >= CIF_ISP20_STATE_READY) &&
			(dev->config.mi_config.mp.output.height >
			dev->config.mi_config.sp.output.height))
			*target_height =
				dev->config.mi_config.mp.output.height;
		else
			*target_height =
				dev->config.mi_config.sp.output.height;
	} else if (dev->mp_stream.state >= CIF_ISP20_STATE_READY) {
		*target_width = dev->config.mi_config.mp.output.width;
		*target_height = dev->config.mi_config.mp.output.height;
	} else {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"cannot get target frame size, no path ready\n");
		return -EFAULT;
	}
	return 0;
}

static void cif_isp20_save_mi_sp(
	struct cif_isp20_device *dev,
	struct cif_isp20_mi_state *state_storage)
{
	CIF_ISP20_PLTFRM_MEM_IO_ADDR base_addr =
		dev->config.base_addr;

	if (!in_irq())
		spin_lock_irqsave(&dev->vbq_lock, state_storage->flags);

	state_storage->isp_ctrl =
		cif_ioread32(base_addr + CIF_ISP_CTRL);
	cif_iowrite32(
		state_storage->isp_ctrl & ~CIF_ISP_CTRL_ISP_CFG_UPD,
		base_addr + CIF_ISP_CTRL);

	/* save the values of the non-shadow registers */
	state_storage->y_base_ad =
		cif_ioread32(base_addr +
			CIF_MI_SP_Y_BASE_AD_INIT);
	state_storage->y_size =
		cif_ioread32(base_addr +
			CIF_MI_SP_Y_SIZE_INIT);
	state_storage->y_offs_cnt =
		cif_ioread32(base_addr +
			CIF_MI_SP_Y_OFFS_CNT_INIT);
	state_storage->cb_base_ad =
		cif_ioread32(base_addr +
			CIF_MI_SP_CB_BASE_AD_INIT);
	state_storage->cb_size =
		cif_ioread32(base_addr +
			CIF_MI_SP_CB_SIZE_INIT);
	state_storage->cb_offs_cnt =
		cif_ioread32(base_addr +
			CIF_MI_SP_CB_OFFS_CNT_INIT);
	state_storage->cr_base_ad =
		cif_ioread32(base_addr +
			CIF_MI_SP_CR_BASE_AD_INIT);
	state_storage->cr_size =
		cif_ioread32(base_addr +
			CIF_MI_SP_CR_SIZE_INIT);
	state_storage->cr_offs_cnt =
		cif_ioread32(base_addr +
			CIF_MI_SP_CR_OFFS_CNT_INIT);

	/* write the values of the shadow (active) registers
		into the non-shadow registers, so that they are
		essentially written back when an asynchronous
		config update is done*/
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_SP_Y_BASE_AD_SHD),
		base_addr + CIF_MI_SP_Y_BASE_AD_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_SP_Y_SIZE_SHD),
		base_addr + CIF_MI_SP_Y_SIZE_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_SP_Y_OFFS_CNT_SHD),
		base_addr + CIF_MI_SP_Y_OFFS_CNT_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_SP_CB_BASE_AD_SHD),
		base_addr + CIF_MI_SP_CB_BASE_AD_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_SP_CB_SIZE_SHD),
		base_addr + CIF_MI_SP_CB_SIZE_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_SP_CB_OFFS_CNT_SHD),
		base_addr + CIF_MI_SP_CB_OFFS_CNT_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_SP_CR_BASE_AD_SHD),
		base_addr + CIF_MI_SP_CR_BASE_AD_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_SP_CR_SIZE_SHD),
		base_addr + CIF_MI_SP_CR_SIZE_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_SP_CR_OFFS_CNT_INIT),
		base_addr + CIF_MI_SP_CR_OFFS_CNT_INIT);
}

/* make static once the mainpath code has been moved here */
static void cif_isp20_save_mi_mp(
	struct cif_isp20_device *dev,
	struct cif_isp20_mi_state *state_storage)
{
	CIF_ISP20_PLTFRM_MEM_IO_ADDR base_addr =
		dev->config.base_addr;

	if (!in_irq())
		spin_lock_irqsave(&dev->vbq_lock, state_storage->flags);

	state_storage->isp_ctrl =
		cif_ioread32(base_addr + CIF_ISP_CTRL);
	cif_iowrite32(
		state_storage->isp_ctrl & ~CIF_ISP_CTRL_ISP_CFG_UPD,
		base_addr + CIF_ISP_CTRL);

	/* save the values of the non-shadow registers */
	state_storage->y_base_ad =
		cif_ioread32(base_addr +
			CIF_MI_MP_Y_BASE_AD_INIT);
	state_storage->y_size =
		cif_ioread32(base_addr +
			CIF_MI_MP_Y_SIZE_INIT);
	state_storage->y_offs_cnt =
		cif_ioread32(base_addr +
			CIF_MI_MP_Y_OFFS_CNT_INIT);
	state_storage->cb_base_ad =
		cif_ioread32(base_addr +
			CIF_MI_MP_CB_BASE_AD_INIT);
	state_storage->cb_size =
		cif_ioread32(base_addr +
			CIF_MI_MP_CB_SIZE_INIT);
	state_storage->cb_offs_cnt =
		cif_ioread32(base_addr +
			CIF_MI_MP_CB_OFFS_CNT_INIT);
	state_storage->cr_base_ad =
		cif_ioread32(base_addr +
			CIF_MI_MP_CR_BASE_AD_INIT);
	state_storage->cr_size =
		cif_ioread32(base_addr +
			CIF_MI_MP_CR_SIZE_INIT);
	state_storage->cr_offs_cnt =
		cif_ioread32(base_addr +
			CIF_MI_MP_CR_OFFS_CNT_INIT);

	/* write the values of the shadow (active) registers
		into the non-shadow registers, so that they are
		essentially written back when an asynchronous
		config update is done*/
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_MP_Y_BASE_AD_SHD),
		base_addr + CIF_MI_MP_Y_BASE_AD_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_MP_Y_SIZE_SHD),
		base_addr + CIF_MI_MP_Y_SIZE_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_MP_Y_OFFS_CNT_SHD),
		base_addr + CIF_MI_MP_Y_OFFS_CNT_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_MP_CB_BASE_AD_SHD),
		base_addr + CIF_MI_MP_CB_BASE_AD_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_MP_CB_SIZE_SHD),
		base_addr + CIF_MI_MP_CB_SIZE_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_MP_CB_OFFS_CNT_SHD),
		base_addr + CIF_MI_MP_CB_OFFS_CNT_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_MP_CR_BASE_AD_SHD),
		base_addr + CIF_MI_MP_CR_BASE_AD_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_MP_CR_SIZE_SHD),
		base_addr + CIF_MI_MP_CR_SIZE_INIT);
	cif_iowrite32(cif_ioread32(base_addr +
		CIF_MI_MP_CR_OFFS_CNT_INIT),
		base_addr + CIF_MI_MP_CR_OFFS_CNT_INIT);
}

static void cif_isp20_restore_mi_sp(
	struct cif_isp20_device *dev,
	struct cif_isp20_mi_state *state_storage)
{
	CIF_ISP20_PLTFRM_MEM_IO_ADDR base_addr =
		dev->config.base_addr;

	cif_iowrite32(state_storage->y_base_ad,
		base_addr + CIF_MI_SP_Y_BASE_AD_INIT);
	cif_iowrite32(state_storage->y_size,
		base_addr + CIF_MI_SP_Y_SIZE_INIT);
	cif_iowrite32(state_storage->y_offs_cnt,
		base_addr + CIF_MI_SP_Y_OFFS_CNT_INIT);
	cif_iowrite32(state_storage->cb_base_ad,
		base_addr + CIF_MI_SP_CB_BASE_AD_INIT);
	cif_iowrite32(state_storage->cb_size,
		base_addr + CIF_MI_SP_CB_SIZE_INIT);
	cif_iowrite32(state_storage->cb_offs_cnt,
		base_addr + CIF_MI_SP_CB_OFFS_CNT_INIT);
	cif_iowrite32(state_storage->cr_base_ad,
		base_addr + CIF_MI_SP_CR_BASE_AD_INIT);
	cif_iowrite32(state_storage->cr_size,
		base_addr + CIF_MI_SP_CR_SIZE_INIT);
	cif_iowrite32(state_storage->cr_offs_cnt,
		base_addr + CIF_MI_SP_CR_OFFS_CNT_INIT);

	cif_iowrite32(
		state_storage->isp_ctrl, base_addr + CIF_ISP_CTRL);

	if (!in_irq())
		spin_unlock_irqrestore(&dev->vbq_lock, state_storage->flags);
}

static void cif_isp20_restore_mi_mp(
	struct cif_isp20_device *dev,
	struct cif_isp20_mi_state *state_storage)
{
	CIF_ISP20_PLTFRM_MEM_IO_ADDR base_addr =
		dev->config.base_addr;

	cif_iowrite32(state_storage->y_base_ad,
		base_addr + CIF_MI_MP_Y_BASE_AD_INIT);
	cif_iowrite32(state_storage->y_size,
		base_addr + CIF_MI_MP_Y_SIZE_INIT);
	cif_iowrite32(state_storage->y_offs_cnt,
		base_addr + CIF_MI_MP_Y_OFFS_CNT_INIT);
	cif_iowrite32(state_storage->cb_base_ad,
		base_addr + CIF_MI_MP_CB_BASE_AD_INIT);
	cif_iowrite32(state_storage->cb_size,
		base_addr + CIF_MI_MP_CB_SIZE_INIT);
	cif_iowrite32(state_storage->cb_offs_cnt,
		base_addr + CIF_MI_MP_CB_OFFS_CNT_INIT);
	cif_iowrite32(state_storage->cr_base_ad,
		base_addr + CIF_MI_MP_CR_BASE_AD_INIT);
	cif_iowrite32(state_storage->cr_size,
		base_addr + CIF_MI_MP_CR_SIZE_INIT);
	cif_iowrite32(state_storage->cr_offs_cnt,
		base_addr + CIF_MI_MP_CR_OFFS_CNT_INIT);

	cif_iowrite32(
		state_storage->isp_ctrl, base_addr + CIF_ISP_CTRL);

	if (!in_irq())
		spin_unlock_irqrestore(&dev->vbq_lock, state_storage->flags);
}

static void cif_isp20_config_clk(
	struct cif_isp20_device *dev)
{
	cif_isp20_pltfrm_pr_dbg(dev->dev, "\n");

	/* CIF HW INIT  Section 3.1.2 Marvin Spec */
	/* 1. BASIC INITIALIZATION*/
	/* Enable the main clock: Section 3.1.7.10 Marvin Spec*/
	cif_iowrite32(CIF_CCL_CIF_CLK_ENA,
		(dev->config.base_addr) + CIF_CCL);
	cif_iowrite32(0x0000187B, (dev->config.base_addr) + CIF_ICCL);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"\n  CIF_CCL 0x%08x\n"
		"  CIF_ICCL 0x%08x\n",
		cif_ioread32(dev->config.base_addr + CIF_CCL),
		cif_ioread32(dev->config.base_addr + CIF_ICCL));
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
			struct cif_isp20_frm_intrvl *frm_intrvl =
				&dev->config.img_src_output.frm_intrvl;
			ret = cif_isp20_img_src_s_streaming(
				dev->img_src, false);
			/* wait for a frame period to make sure that there is
				no pending frame left. */
			mdelay((1000 *
				frm_intrvl->numerator +
				frm_intrvl->denominator - 1) /
				frm_intrvl->denominator);
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
		(state != CIF_ISP20_IMG_SRC_STATE_STREAMING)))
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
			;
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
				if (dev->config.jpeg_config.enable &&
					((img_src_width >=
					request_strm_fmt.frm_fmt.width) &&
					(img_src_height >
					request_strm_fmt.frm_fmt.height)))
					/* for image capturing we try to
						maximize the size */
					better_match = true;
				else if (!dev->config.jpeg_config.enable &&
					(diff < best_diff))
					better_match = true;
				else if (!dev->config.jpeg_config.enable &&
					((strm_fmt_desc.min_intrvl.denominator
					/
					strm_fmt_desc.min_intrvl.numerator)
					>
					(request_strm_fmt.frm_intrvl.denominator
					/
					request_strm_fmt.frm_intrvl.numerator))
					&&
					(diff == best_diff))
					/* maximize fps */
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

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
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

	if (!dev->config.sp_config.updt_cfg &&
		!dev->config.mp_config.updt_cfg)
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
	u32 output_width;
	u32 output_height;
	u32 h_offs;
	u32 v_offs;
	u32 yuv_seq = 0;
	u32 bpp;
	u32 isp_input_sel = 0;
	u32 isp_bayer_pat = 0;
	u32 acq_mult = 1;
	enum cif_isp20_pix_fmt in_pix_fmt;

	dev->config.isp_config.output =
		*dev->config.isp_config.input;

	in_pix_fmt = dev->config.isp_config.input->pix_fmt;
	input_width = dev->config.isp_config.input->width;
	input_height = dev->config.isp_config.input->height;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "%s %dx%d\n",
		cif_isp20_pix_fmt_string(in_pix_fmt),
		input_width, input_height);

	if (CIF_ISP20_PIX_FMT_IS_RAW_BAYER(in_pix_fmt)) {
		if (!dev->config.mi_config.raw_enable) {
			dev->config.isp_config.output.pix_fmt = CIF_YUV422I;
			cif_iowrite32(0xc,
				dev->config.base_addr + CIF_ISP_DEMOSAIC);
			cif_iowrite32(CIF_ISP_CTRL_ISP_MODE_BAYER_ITU601,
				dev->config.base_addr + CIF_ISP_CTRL);
		} else {
			cif_iowrite32(CIF_ISP_CTRL_ISP_MODE_RAW_PICT,
				dev->config.base_addr + CIF_ISP_CTRL);
		}

		bpp = CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt);
		if (bpp == 8)
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_8b_4_0_LSB;
		else if (bpp == 10)
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_10b_2_MSB_LSB;
		else if (bpp == 12)
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_12b;
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
		acq_mult = 2;
		isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_12b;
		if (CIF_ISP20_PIX_FMT_YUV_IS_YC_SWAPPED(in_pix_fmt))
			yuv_seq = CIF_ISP_ACQ_PROP_CBYCRY;
		else
			yuv_seq = CIF_ISP_ACQ_PROP_YCBYCR;
		cif_iowrite32(CIF_ISP_CTRL_ISP_MODE_ITU601,
			(dev->config.base_addr) + CIF_ISP_CTRL);
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
		&output_width, &output_height,
		&h_offs, &v_offs);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_iowrite32(v_offs,
		dev->config.base_addr + CIF_ISP_OUT_V_OFFS);
	cif_iowrite32(h_offs,
		dev->config.base_addr + CIF_ISP_OUT_H_OFFS);
	cif_iowrite32(output_width,
		dev->config.base_addr + CIF_ISP_OUT_H_SIZE);
	cif_iowrite32(output_height,
		dev->config.base_addr + CIF_ISP_OUT_V_SIZE);

	dev->config.isp_config.output.width = output_width;
	dev->config.isp_config.output.height = output_height;

	dev->isp_dev.input_width = output_width;
	dev->isp_dev.input_height = output_height;

	/* interrupt mask */
	cif_iowrite32(
		CIF_ISP_FRAME |
		CIF_ISP_DATA_LOSS |
		CIF_ISP_PIC_SIZE_ERROR
#ifdef MEASURE_VERTICAL_BLANKING
		| CIF_ISP_V_START | CIF_ISP_FRAME_IN
#endif
		,
		dev->config.base_addr + CIF_ISP_IMSC);

	if (CIF_ISP20_PIX_FMT_IS_RAW_BAYER(
			dev->config.isp_config.input->pix_fmt) &&
		!dev->config.mi_config.raw_enable)
		cifisp_configure_isp(&dev->isp_dev,
			dev->config.jpeg_config.enable);
	else
		cifisp_disable_isp(&dev->isp_dev);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"  ISP_CTRL 0x%08x\n"
		"  ISP_IMSC 0x%08x\n"
		"  ISP_ACQ_PROP 0x%08x\n"
		"  ISP_ACQ %dx%d@(%d,%d)\n"
		"  ISP_OUT %dx%d@(%d,%d)\n"
		"  ISP_IS %dx%d@(%d,%d)\n",
		cif_ioread32((dev->config.base_addr) + CIF_ISP_CTRL),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_IMSC),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_ACQ_PROP),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_ACQ_H_SIZE),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_ACQ_V_SIZE),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_ACQ_H_OFFS),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_ACQ_V_OFFS),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_OUT_H_SIZE),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_OUT_V_SIZE),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_OUT_H_OFFS),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_OUT_V_OFFS),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_IS_H_SIZE),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_IS_V_SIZE),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_IS_H_OFFS),
		cif_ioread32((dev->config.base_addr) + CIF_ISP_IS_V_OFFS));

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
	enum cif_isp20_pix_fmt in_pix_fmt =
		dev->config.img_src_output.frm_fmt.pix_fmt;

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
		CIF_MIPI_CTRL_ERR_SOT_HS_ENA;

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
			(dev->config.base_addr) + CIF_MIPI_DPHY2_1);
		cif_iowrite32(dev->config.mipi_config.csi_config.dphy2,
			(dev->config.base_addr) + CIF_MIPI_DPHY2_2);
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
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 0) &&
			(CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt) == 12))
			data_type = CSI2_DT_YUV420_8b;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 0) &&
			(CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt) == 15))
			data_type = CSI2_DT_YUV420_10b;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_GET_BPP(in_pix_fmt) == 16))
			data_type = CSI2_DT_YUV422_8b;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 2) &&
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
		CIF_MIPI_FRAME_END |
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
			u32 mult = 1;

			if (CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt))
				mult = 2;
			dev->config.mi_config.mp.y_size =
				(dev->config.mi_config.mp.y_size * 4) /
				(4 + mult *
				CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt));
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

	cif_iowrite32(dev->config.mi_config.mp.y_size,
		dev->config.base_addr + CIF_MI_MP_Y_SIZE_INIT);
	cif_iowrite32(dev->config.mi_config.mp.cb_size,
		dev->config.base_addr + CIF_MI_MP_CB_SIZE_INIT);
	cif_iowrite32(dev->config.mi_config.mp.cr_size,
		dev->config.base_addr + CIF_MI_MP_CR_SIZE_INIT);

	cif_iowrite32OR(
		CIF_MI_CTRL_MP_WRITE_FMT(writeformat) |
		swap_cb_cr |
		CIF_MI_CTRL_BURST_LEN_LUM_64 |
		CIF_MI_CTRL_BURST_LEN_CHROM_64 |
		CIF_MI_CTRL_INIT_BASE_EN,
		dev->config.base_addr + CIF_MI_CTRL);

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

	dev->config.mi_config.sp.input =
		&dev->config.mp_config.rsz_config.output;

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
			u32 mult = 1;

			if (CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt))
				mult = 2;
			dev->config.mi_config.sp.y_size =
				(dev->config.mi_config.sp.y_size * 4) /
				(4 + mult *
				CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt));
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
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 0))
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_YUV420;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 2))
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
		(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 0))
		input_format = CIF_MI_CTRL_SP_INPUT_FMT_YUV420;
	else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
		(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 2))
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

	cif_iowrite32(dev->config.mi_config.sp.y_size,
		dev->config.base_addr + CIF_MI_SP_Y_SIZE_INIT);
	cif_iowrite32(dev->config.mi_config.sp.y_size,
		dev->config.base_addr + CIF_MI_SP_Y_PIC_SIZE);
	cif_iowrite32(dev->config.mi_config.sp.cb_size,
		dev->config.base_addr + CIF_MI_SP_CB_SIZE_INIT);
	cif_iowrite32(dev->config.mi_config.sp.cr_size,
		dev->config.base_addr + CIF_MI_SP_CR_SIZE_INIT);
	cif_iowrite32(width,
		dev->config.base_addr + CIF_MI_SP_Y_PIC_WIDTH);
	cif_iowrite32(height,
		dev->config.base_addr + CIF_MI_SP_Y_PIC_HEIGHT);
	cif_iowrite32(llength,
		dev->config.base_addr + CIF_MI_SP_Y_LLENGTH);

	cif_iowrite32OR(
		CIF_MI_CTRL_SP_WRITE_FMT(writeformat) |
		swap_cb_cr |
		input_format |
		output_format |
		CIF_MI_CTRL_BURST_LEN_LUM_64 |
		CIF_MI_CTRL_BURST_LEN_CHROM_64 |
		CIF_MI_CTRL_INIT_BASE_EN,
		dev->config.base_addr + CIF_MI_CTRL);

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
			u32 mult = 1;

			if (CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt))
				mult = 2;
			dev->config.mi_config.dma.y_size =
				(dev->config.mi_config.dma.y_size * 4) /
				(4 + mult *
				CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt));
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
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 0))
			output_format = CIF_MI_DMA_CTRL_FMT_YUV420;
		else if ((CIF_ISP20_PIX_FMT_YUV_GET_X_SUBS(out_pix_fmt) == 2) &&
			(CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(out_pix_fmt) == 2))
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

	cif_iowrite32(dev->config.mi_config.dma.y_size,
		dev->config.base_addr + CIF_MI_DMA_Y_PIC_SIZE);
	cif_iowrite32(width,
		dev->config.base_addr + CIF_MI_DMA_Y_PIC_WIDTH);
	cif_iowrite32(llength,
		dev->config.base_addr + CIF_MI_DMA_Y_LLENGTH);

	cif_iowrite32OR(
		CIF_MI_DMA_CTRL_WRITE_FMT(writeformat) |
		output_format |
		CIF_MI_DMA_CTRL_BURST_LEN_LUM_64 |
		CIF_MI_DMA_CTRL_BURST_LEN_CHROM_64,
		dev->config.base_addr + CIF_MI_DMA_CTRL);

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
	int ret;
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

	/* upscaling of BT601 color space to full range 0..255 */
	cif_iowrite32(CIF_JPE_LUM_SCALE_ENABLE,
		dev->config.base_addr + CIF_JPE_Y_SCALE_EN);
	cif_iowrite32(CIF_JPE_CHROM_SCALE_ENABLE,
		dev->config.base_addr + CIF_JPE_CBCR_SCALE_EN);

	switch (inp_fmt->pix_fmt) {
	case CIF_YUV422I:
	case CIF_YUV422SP:
	case CIF_YUV422P:
		cif_iowrite32(CIF_JPE_PIC_FORMAT_YUV422,
			dev->config.base_addr + CIF_JPE_PIC_FORMAT);
		break;
	case CIF_YUV400:
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

	/* Program tables */
	ret = marvin_lib_program_jpeg_tables(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	/* Select tables */
	ret = marvin_lib_select_jpeg_tables(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

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
err:
	cif_isp20_pltfrm_pr_err(NULL,
		"failed with error %d\n", ret);
	return ret;
}



static int cif_isp20_config_path(struct cif_isp20_device *dev)
{
	u32 dpcl = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "\n");

	/* TBD: yc_spmux */

	/* if_sel */
	if (dev->config.input_sel & CIF_ISP20_INP_DMA)
		dpcl |= CIF_VI_DPCL_IF_SEL_DMA;
	else if (dev->config.input_sel & CIF_ISP20_INP_DMA_IE)
		dpcl |= CIF_VI_DPCL_DMA_IE_MUX_DMA | CIF_VI_DPCL_DMA_SW_IE;
	else if (dev->config.input_sel & CIF_ISP20_INP_DMA_SP)
		dpcl |= CIF_VI_DPCL_DMA_SP_MUX_DMA;
	else {
		if ((dev->config.input_sel & CIF_ISP20_INP_CSI_0) ||
			(dev->config.input_sel & CIF_ISP20_INP_CSI_1)) {
			dpcl |= CIF_VI_DPCL_IF_SEL_MIPI;
		} else
			dpcl |= CIF_VI_DPCL_IF_SEL_PARALLEL;
		if (dev->config.input_sel & CIF_ISP20_INP_SI)
			dpcl |= CIF_VI_DPCL_DMA_SW_SI;
	}

	/* chan_mode */
	if (dev->sp_stream.state == CIF_ISP20_STATE_READY)
		dpcl |= CIF_VI_DPCL_CHAN_MODE_SP;

	if ((dev->mp_stream.state == CIF_ISP20_STATE_READY) &&
		!(dev->config.input_sel & CIF_ISP20_INP_DMA_SP)) {
			dpcl |= CIF_VI_DPCL_CHAN_MODE_MP;
		/* mp_dmux */
		if (dev->config.jpeg_config.enable == true)
			dpcl |= CIF_VI_DPCL_MP_MUX_MRSZ_JPEG;
		else
			dpcl |= CIF_VI_DPCL_MP_MUX_MRSZ_MI;
	}

	cif_iowrite32(dpcl,
		dev->config.base_addr + CIF_VI_DPCL);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"CIF_DPCL 0x%08x\n", dpcl);

	return 0;
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

	return 0;
}

static int cif_isp20_mi_frame_end(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream_id)
{
	int ret = 0;
	struct cif_isp20_stream *stream;
	bool frame_done = true;
	bool mp = false;
	CIF_ISP20_PLTFRM_MEM_IO_ADDR y_base_addr;

	cif_isp20_pltfrm_pr_dbg(NULL, "%s\n",
		cif_isp20_stream_id_string(stream_id));

	if (stream_id == CIF_ISP20_STREAM_MP) {
		mp = true;
		stream = &dev->mp_stream;
		y_base_addr =
			dev->config.base_addr + CIF_MI_MP_Y_BASE_AD_SHD;
		if (dev->config.jpeg_config.enable) {
			unsigned int jpe_status =
				cif_ioread32(dev->config.base_addr +
					CIF_JPE_STATUS_RIS);
			if (jpe_status & CIF_JPE_STATUS_ENCODE_DONE) {
				cif_iowrite32(CIF_JPE_STATUS_ENCODE_DONE,
					dev->config.base_addr +
						CIF_JPE_STATUS_ICR);
				dev->config.jpeg_config.busy = false;
				if (stream->curr_buf != NULL) {
					stream->curr_buf->size =
					cif_ioread32(dev->config.base_addr +
						CIF_MI_BYTE_CNT);
					if (stream->curr_buf->size >
						dev->config.mi_config.
						mp.y_size)
						cif_isp20_pltfrm_pr_err(NULL,
							"JPEG image too large for buffer, presumably corrupted\n");
				}
			} else {
				frame_done = false;
			}
		}
	} else if (stream_id == CIF_ISP20_STREAM_SP) {
		stream = &dev->sp_stream;
		y_base_addr =
			dev->config.base_addr + CIF_MI_SP_Y_BASE_AD_SHD;
	} else {
		BUG();
	}

	if (stream->next_buf == NULL)
		stream->stall = true;

	if (frame_done && (stream->curr_buf != NULL)) {
		if (!stream->stall ||
			(dev->config.jpeg_config.enable && mp)) {
			do_gettimeofday(&stream->curr_buf->ts);
			stream->curr_buf->field_count++;
			/*Inform the wait queue */
			stream->curr_buf->state = VIDEOBUF_DONE;
			wake_up(&stream->curr_buf->done);
			stream->curr_buf = NULL;
		}
	}
	stream->stall = false;

	if ((stream->next_buf != NULL) && (stream->curr_buf == NULL)) {
		if (videobuf_to_dma_contig(stream->next_buf) ==
			cif_ioread32(y_base_addr)) {
			stream->curr_buf = stream->next_buf;
			stream->next_buf = NULL;
		} else
			cif_isp20_pltfrm_pr_warn(dev->dev,
				"%s buffer queue is no advancing\n",
				mp ? "MP" : "SP");
	}

	if ((frame_done || (stream->curr_buf == NULL)) &&
		(stream->next_buf == NULL)) {
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
			if (mp)
				dev->config.mi_config.mp.next_buff_addr =
					videobuf_to_dma_contig(
						stream->next_buf);
			else
				dev->config.mi_config.sp.next_buff_addr =
					videobuf_to_dma_contig(
						stream->next_buf);
		} else if (stream->curr_buf == NULL) {
			if (mp)
				dev->config.mi_config.mp.next_buff_addr =
					dev->config.mi_config.
						null_buff_dma_addr;
			else
				dev->config.mi_config.sp.next_buff_addr =
					dev->config.mi_config.
						null_buff_dma_addr;
		}
	}

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"MP next_buff_addr = 0x%08x, SP next_buff_addr = 0x%08x\n",
		dev->config.mi_config.mp.next_buff_addr,
		dev->config.mi_config.sp.next_buff_addr);

	return ret;
}

static void cif_isp20_start_mi(
	struct cif_isp20_device *dev,
	bool start_mi_sp,
	bool start_mi_mp)
{
	struct cif_isp20_mi_state saved_mi_state;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "\n");

	if (start_mi_sp &&
		(dev->sp_stream.state == CIF_ISP20_STATE_STREAMING))
		start_mi_sp = false;
	if (start_mi_mp &&
		(dev->mp_stream.state == CIF_ISP20_STATE_STREAMING))
		start_mi_mp = false;
	if (!start_mi_sp && !start_mi_mp)
		return;

	if (start_mi_sp) {
		dev->config.mi_config.sp.next_buff_addr =
			dev->config.mi_config.null_buff_dma_addr;
		dev->config.mi_config.sp.curr_buff_addr =
			dev->config.mi_config.null_buff_dma_addr;
		spin_lock(&dev->vbq_lock);
		cif_isp20_mi_frame_end(dev, CIF_ISP20_STREAM_SP);
		spin_unlock(&dev->vbq_lock);
		if (dev->mp_stream.state == CIF_ISP20_STATE_STREAMING)
			cif_isp20_save_mi_mp(dev, &saved_mi_state);
		update_mi_sp(dev);
		dev->sp_stream.stall = false;
		cif_iowrite32OR(CIF_MI_CTRL_SP_ENABLE,
			dev->config.base_addr + CIF_MI_CTRL);
	}

	if (start_mi_mp) {
		dev->config.mi_config.mp.next_buff_addr =
			dev->config.mi_config.null_buff_dma_addr;
		dev->config.mi_config.mp.curr_buff_addr =
			dev->config.mi_config.null_buff_dma_addr;
		spin_lock(&dev->vbq_lock);
		cif_isp20_mi_frame_end(dev, CIF_ISP20_STREAM_MP);
		spin_unlock(&dev->vbq_lock);
		if (dev->sp_stream.state == CIF_ISP20_STATE_STREAMING)
			cif_isp20_save_mi_sp(dev, &saved_mi_state);
		update_mi_mp(dev);
		dev->mp_stream.stall = false;
		if (dev->config.jpeg_config.enable)
			cif_iowrite32OR(CIF_MI_CTRL_JPEG_ENABLE,
				dev->config.base_addr +
				CIF_MI_CTRL);
		else
			cif_iowrite32OR(CIF_MI_CTRL_MP_ENABLE,
				(dev->config.base_addr) +
				CIF_MI_CTRL);
	}

	cif_iowrite32(CIF_MI_INIT_SOFT_UPD,
		dev->config.base_addr + CIF_MI_INIT);

	if (start_mi_sp && (dev->mp_stream.state == CIF_ISP20_STATE_STREAMING))
		cif_isp20_restore_mi_mp(dev, &saved_mi_state);
	if (start_mi_mp && (dev->sp_stream.state == CIF_ISP20_STATE_STREAMING))
		cif_isp20_restore_mi_sp(dev, &saved_mi_state);

	/* this will start the JPEG encoding as early as possible: */
	if (start_mi_mp && dev->config.jpeg_config.enable) {
		cif_iowrite32OR(CIF_MI_MP_FRAME,
			dev->config.base_addr + CIF_MI_IMSC);
		spin_lock(&dev->vbq_lock);
		cif_isp20_mi_frame_end(dev, CIF_ISP20_STREAM_MP);
		update_mi_mp(dev);
		spin_unlock(&dev->vbq_lock);
	}
}

static void cif_isp20_stop_mi(
	struct cif_isp20_device *dev,
	bool stop_mi_sp,
	bool stop_mi_mp)
{
	struct cif_isp20_mi_state saved_mi_state;

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
		cif_iowrite32AND(~(CIF_MI_SP_FRAME |
			CIF_MI_MP_FRAME |
			CIF_JPE_STATUS_ENCODE_DONE),
			(dev->config.base_addr) + CIF_MI_IMSC);
		cif_iowrite32(CIF_MI_SP_FRAME |
			CIF_MI_MP_FRAME |
			CIF_JPE_STATUS_ENCODE_DONE,
			dev->config.base_addr + CIF_MI_ICR);
		cif_iowrite32AND(~CIF_MI_CTRL_SP_ENABLE,
			dev->config.base_addr + CIF_MI_CTRL);
		cif_iowrite32AND(~(CIF_MI_CTRL_MP_ENABLE |
			CIF_MI_CTRL_SP_ENABLE |
			CIF_MI_CTRL_JPEG_ENABLE |
			CIF_MI_CTRL_RAW_ENABLE),
			(dev->config.base_addr) + CIF_MI_CTRL);
		cif_iowrite32(CIF_MI_INIT_SOFT_UPD,
			dev->config.base_addr + CIF_MI_INIT);
	} else if (stop_mi_sp) {
		if (dev->mp_stream.state == CIF_ISP20_STATE_STREAMING)
			cif_isp20_save_mi_mp(dev, &saved_mi_state);
		cif_iowrite32AND(~CIF_MI_SP_FRAME,
			(dev->config.base_addr) + CIF_MI_IMSC);
		cif_iowrite32(CIF_MI_SP_FRAME,
			dev->config.base_addr + CIF_MI_ICR);
		cif_iowrite32AND(~CIF_MI_CTRL_SP_ENABLE,
			dev->config.base_addr + CIF_MI_CTRL);
		cif_iowrite32(CIF_MI_INIT_SOFT_UPD,
			dev->config.base_addr + CIF_MI_INIT);
		if (dev->mp_stream.state == CIF_ISP20_STATE_STREAMING)
			cif_isp20_restore_mi_mp(dev, &saved_mi_state);
	} else if (stop_mi_mp) {
		if (dev->sp_stream.state == CIF_ISP20_STATE_STREAMING)
			cif_isp20_save_mi_sp(dev, &saved_mi_state);
		cif_iowrite32AND(~(CIF_MI_MP_FRAME |
			CIF_JPE_STATUS_ENCODE_DONE),
			(dev->config.base_addr) + CIF_MI_IMSC);
		cif_iowrite32(CIF_MI_MP_FRAME |
			CIF_JPE_STATUS_ENCODE_DONE,
			dev->config.base_addr + CIF_MI_ICR);
		cif_iowrite32AND(~(CIF_MI_CTRL_MP_ENABLE |
			CIF_MI_CTRL_JPEG_ENABLE |
			CIF_MI_CTRL_RAW_ENABLE),
			(dev->config.base_addr) + CIF_MI_CTRL);
		if (dev->sp_stream.state == CIF_ISP20_STATE_STREAMING)
			cif_isp20_restore_mi_sp(dev, &saved_mi_state);
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
		(dev->sp_stream.state == CIF_ISP20_STATE_STREAMING))))
		return 0;

	if ((stop_mp && stop_sp) ||
		(stop_sp &&
		(dev->mp_stream.state != CIF_ISP20_STATE_STREAMING)) ||
		(stop_mp &&
		(dev->sp_stream.state != CIF_ISP20_STATE_STREAMING))) {
		/* stop and clear MI, MIPI, and ISP interrupts */
		cif_iowrite32(0, dev->config.base_addr + CIF_MIPI_IMSC);
		cif_iowrite32(~0, dev->config.base_addr + CIF_MIPI_ICR);

		cif_iowrite32(0, dev->config.base_addr + CIF_ISP_IMSC);
		cif_iowrite32(~0, dev->config.base_addr + CIF_ISP_ICR);

		cif_iowrite32(0, dev->config.base_addr + CIF_MI_IMSC);
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

		if (IS_ERR_VALUE(cif_isp20_img_src_set_state(dev,
			CIF_ISP20_IMG_SRC_STATE_SW_STNDBY)))
			cif_isp20_pltfrm_pr_dbg(dev->dev,
			"unable to put image source into standby\n");
		if (IS_ERR_VALUE(cif_isp20_set_pm_state(dev,
			CIF_ISP20_PM_STATE_SW_STNDBY)))
			cif_isp20_pltfrm_pr_dbg(dev->dev,
			"unable to put CIF into standby\n");
	} else if (stop_sp) {
		cif_isp20_stop_mi(dev, true, false);
	} else /* stop_mp */ {
		cif_isp20_stop_mi(dev, false, true);
	}

	if (stop_mp && (dev->mp_stream.state == CIF_ISP20_STATE_STREAMING))
		dev->mp_stream.state = CIF_ISP20_STATE_READY;

	if (stop_sp && (dev->sp_stream.state == CIF_ISP20_STATE_STREAMING))
		dev->sp_stream.state = CIF_ISP20_STATE_READY;

	spin_lock_irqsave(&dev->vbq_lock, flags);
	if (stop_sp) {
		if (dev->sp_stream.next_buf != NULL) {
			list_add(&dev->sp_stream.next_buf->queue,
				&dev->sp_stream.buf_queue);
			dev->sp_stream.next_buf->state = VIDEOBUF_QUEUED;
			dev->sp_stream.next_buf = NULL;
		}
		if (dev->sp_stream.curr_buf != NULL) {
			list_add(&dev->sp_stream.curr_buf->queue,
				&dev->sp_stream.buf_queue);
			dev->sp_stream.curr_buf->state = VIDEOBUF_QUEUED;
			dev->sp_stream.curr_buf = NULL;
		}
	}
	if (stop_mp) {
		if (dev->mp_stream.next_buf != NULL) {
			list_add(&dev->mp_stream.next_buf->queue,
				&dev->mp_stream.buf_queue);
			dev->mp_stream.next_buf->state = VIDEOBUF_QUEUED;
			dev->mp_stream.next_buf = NULL;
		}
		if (dev->mp_stream.curr_buf != NULL) {
			list_add(&dev->mp_stream.curr_buf->queue,
				&dev->mp_stream.buf_queue);
			dev->mp_stream.curr_buf->state = VIDEOBUF_QUEUED;
			dev->mp_stream.curr_buf = NULL;
		}
	}
	spin_unlock_irqrestore(&dev->vbq_lock, flags);

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, img_src state = %s\n"
		"  MI_CTRL 0x%08x\n"
		"  ISP_CTRL 0x%08x\n"
		"  MIPI_CTRL 0x%08x\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
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
	unsigned int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, img_src state = %s, start_sp = %d, start_mp = %d\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		cif_isp20_img_src_state_string(dev->img_src_state),
		start_sp,
		start_mp);

	if (!((start_mp &&
		(dev->mp_stream.state != CIF_ISP20_STATE_STREAMING)) ||
		(start_sp &&
		(dev->sp_stream.state != CIF_ISP20_STATE_STREAMING))))
		return 0;

	/* Activate MI */
	cif_isp20_start_mi(dev, start_sp, start_mp);

	if ((dev->sp_stream.state != CIF_ISP20_STATE_STREAMING) &&
		(dev->mp_stream.state != CIF_ISP20_STATE_STREAMING)) {
		/* Activate MIPI */
		cif_iowrite32OR(CIF_MIPI_CTRL_OUTPUT_ENA,
			(dev->config.base_addr) + CIF_MIPI_CTRL);

		/* Activate ISP ! */
		cif_iowrite32OR(CIF_ISP_CTRL_ISP_CFG_UPD |
			CIF_ISP_CTRL_ISP_INFORM_ENABLE |
			CIF_ISP_CTRL_ISP_ENABLE,
			(dev->config.base_addr) + CIF_ISP_CTRL);
	}

	if (start_sp)
		dev->sp_stream.state = CIF_ISP20_STATE_STREAMING;
	if (start_mp)
		dev->mp_stream.state = CIF_ISP20_STATE_STREAMING;

	ret = cif_isp20_set_pm_state(dev,
		CIF_ISP20_PM_STATE_STREAMING);
	if (!IS_ERR_VALUE(ret)) {
		/* CIF spec says to wait for sufficient time after enabling
			the MIPI interface and before starting the
			sensor output. */
		mdelay(1);
		/* start sensor output! */
		ret = cif_isp20_img_src_set_state(dev,
			CIF_ISP20_IMG_SRC_STATE_STREAMING);
	}

	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_err(dev->dev,
			"image source start streaming failed with %d\n",
			ret);
	else
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"SP state = %s, MP state = %s, img_src state = %s\n"
			"  MI_CTRL 0x%08x\n"
			"  ISP_CTRL 0x%08x\n"
			"  MIPI_CTRL 0x%08x\n",
			cif_isp20_state_string(dev->sp_stream.state),
			cif_isp20_state_string(dev->mp_stream.state),
			cif_isp20_img_src_state_string(dev->img_src_state),
			cif_ioread32(dev->config.base_addr + CIF_MI_CTRL),
			cif_ioread32(dev->config.base_addr + CIF_ISP_CTRL),
			cif_ioread32(dev->config.base_addr + CIF_MIPI_CTRL));

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

	if (mi_mis & CIF_MI_AHB_ERROR) {
		cif_isp20_pltfrm_pr_warn(dev->dev, "AHB error\n");
		cif_iowrite32(CIF_MI_AHB_ERROR,
			dev->config.base_addr + CIF_MI_ICR);
	}

	if (mi_mis & CIF_MI_SP_FRAME) {
		cif_isp20_mi_frame_end(dev, CIF_ISP20_STREAM_SP);
		update_mi_sp(dev);
		cif_iowrite32(CIF_MI_SP_FRAME,
			(dev->config.base_addr) + CIF_MI_ICR);
	}
	if (mi_mis & CIF_MI_MP_FRAME) {
		cif_isp20_mi_frame_end(dev, CIF_ISP20_STREAM_MP);
		update_mi_mp(dev);
		cif_iowrite32(CIF_MI_MP_FRAME,
			(dev->config.base_addr) + CIF_MI_ICR);
	}

	cif_iowrite32(~(CIF_MI_MP_FRAME | CIF_MI_SP_FRAME),
		(dev->config.base_addr) + CIF_MI_ICR);

	return 0;
}

static int cif_isp20_register_isrs(struct cif_isp20_device *dev)
{
	int ret = 0;

	cif_isp20_pltfrm_irq_register_isr(
		dev->dev,
		CIF_ISP20_IRQ_ISP,
		marvin_isp_isr,
		dev);
	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_warn(dev->dev,
			"unable to register ISP ISR, some processing errors may go unnoticed\n");

	cif_isp20_pltfrm_irq_register_isr(
		dev->dev,
		CIF_ISP20_IRQ_MIPI,
		marvin_mipi_isr,
		dev);
	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_warn(dev->dev,
			"unable to register MIPI ISR, MIPI errors may go unnoticed\n");

	ret = cif_isp20_pltfrm_irq_register_isr(
		dev->dev,
		CIF_ISP20_IRQ_MI,
		cif_isp20_mi_isr,
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
	bool streamon_sp,
	bool streamon_mp)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, streamon SP = %d, streamon MP = %d\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		streamon_sp,
		streamon_mp);

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

	if (streamon_sp && dev->config.mi_config.raw_enable &&
		(streamon_mp ||
		(dev->mp_stream.state == CIF_ISP20_STATE_STREAMING))) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"cannot start streaming on SP path when MP is active and set to RAW output\n");
		ret = -EBUSY;
		goto err;
	}

	if (streamon_sp && dev->config.sp_config.updt_cfg &&
		(dev->mp_stream.state == CIF_ISP20_STATE_STREAMING)) {
		ret = cif_isp20_stop(dev, false, true);
		if (IS_ERR_VALUE(ret))
			goto err;
		streamon_mp = true;
		dev->config.mp_config.updt_cfg = true;
	}
	if (streamon_mp && dev->config.mp_config.updt_cfg &&
		(dev->sp_stream.state == CIF_ISP20_STATE_STREAMING)) {
		ret = cif_isp20_stop(dev, true, false);
		if (IS_ERR_VALUE(ret))
			goto err;
		streamon_sp = true;
		dev->config.sp_config.updt_cfg = true;
	}

	ret = config_cif(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_isp20_start(dev, streamon_sp, streamon_mp);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

int cif_isp20_streamoff(
	struct cif_isp20_device *dev,
	bool streamoff_sp,
	bool streamoff_mp)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s, streamoff SP = %d, streamoff MP = %d\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state),
		streamoff_sp,
		streamoff_mp);

	ret = cif_isp20_stop(dev, streamoff_sp, streamoff_mp);
	if (IS_ERR_VALUE(ret))
		goto err;
	if (streamoff_sp) {
		if (dev->sp_stream.state == CIF_ISP20_STATE_READY)
			dev->sp_stream.state = CIF_ISP20_STATE_INACTIVE;
	}
	if (streamoff_mp) {
		dev->config.jpeg_config.enable = false;
		dev->config.mi_config.raw_enable = false;
		if (dev->mp_stream.state == CIF_ISP20_STATE_READY)
			dev->mp_stream.state = CIF_ISP20_STATE_INACTIVE;
	}

	return 0;
err:
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
	bool streamon_sp = false;
	bool streamon_mp = false;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"SP state = %s, MP state = %s\n",
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state));

	if ((dev->sp_stream.saved_state == CIF_ISP20_STATE_READY) ||
		(dev->sp_stream.saved_state == CIF_ISP20_STATE_STREAMING)) {
		dev->config.sp_config.updt_cfg = true;
		dev->sp_stream.state = CIF_ISP20_STATE_READY;
		if (dev->sp_stream.saved_state == CIF_ISP20_STATE_STREAMING)
			streamon_sp = true;
	}
	if ((dev->mp_stream.saved_state == CIF_ISP20_STATE_READY) ||
		(dev->mp_stream.saved_state == CIF_ISP20_STATE_STREAMING)) {
		dev->config.mp_config.updt_cfg = true;
		dev->mp_stream.state = CIF_ISP20_STATE_READY;
		if (dev->mp_stream.saved_state == CIF_ISP20_STATE_STREAMING)
			streamon_mp = true;
	}

	return cif_isp20_streamon(dev, streamon_sp, streamon_mp);
}

int cif_isp20_release(
	struct cif_isp20_device *dev,
	bool release_sp,
	bool release_mp)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"release_sp = %d, release_mp = %d\n",
		release_sp, release_mp);

	if ((dev->sp_stream.state == CIF_ISP20_STATE_DISABLED) &&
		(dev->mp_stream.state == CIF_ISP20_STATE_DISABLED))
		return 0;

	if (release_sp) {
		if (dev->sp_stream.state == CIF_ISP20_STATE_STREAMING) {
			cif_isp20_pltfrm_pr_warn(dev->dev,
			"CIF SP in streaming state, should be stopped before release, trying to stop it\n");
			ret = cif_isp20_stop(dev, true, false);
			if (IS_ERR_VALUE(ret))
				goto err;
		}
		dev->sp_stream.state = CIF_ISP20_STATE_DISABLED;
	}
	if (release_mp) {
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
		if (IS_ERR_VALUE(cif_isp20_img_src_set_state(dev,
			CIF_ISP20_IMG_SRC_STATE_OFF)))
			cif_isp20_pltfrm_pr_warn(dev->dev,
				"image source power off failed\n");
		dev->img_src = NULL;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

int cif_isp20_s_fmt_mp(
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

	if (CIF_ISP20_PIX_FMT_IS_JPEG(strm_fmt->frm_fmt.pix_fmt))
		dev->config.jpeg_config.enable = true;
	else if (CIF_ISP20_PIX_FMT_IS_RAW_BAYER(strm_fmt->frm_fmt.pix_fmt)) {
		if ((dev->sp_stream.state == CIF_ISP20_STATE_READY) ||
			(dev->sp_stream.state == CIF_ISP20_STATE_STREAMING))
			cif_isp20_pltfrm_pr_warn(dev->dev,
				"cannot output RAW data when SP is active, you will not be able to (re-)start streaming\n");
		dev->config.mi_config.raw_enable = true;
	}

	dev->config.mi_config.mp.output = strm_fmt->frm_fmt;

	dev->config.mi_config.mp.llength =
		cif_isp20_calc_llength(
			strm_fmt->frm_fmt.width,
			stride,
			strm_fmt->frm_fmt.pix_fmt);

	dev->config.mp_config.updt_cfg = true;
	dev->mp_stream.state = CIF_ISP20_STATE_READY;

	ret = cif_isp20_img_src_select_strm_fmt(dev);
	if (IS_ERR_VALUE(ret)) {
		dev->config.mp_config.updt_cfg = false;
		dev->mp_stream.state = CIF_ISP20_STATE_INACTIVE;
		goto err;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

int cif_isp20_s_fmt_sp(
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

	if (dev->config.mi_config.raw_enable)
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

	dev->config.sp_config.updt_cfg = true;
	dev->sp_stream.state = CIF_ISP20_STATE_READY;

	ret = cif_isp20_img_src_select_strm_fmt(dev);
	if (IS_ERR_VALUE(ret)) {
		dev->config.sp_config.updt_cfg = false;
		dev->sp_stream.state = CIF_ISP20_STATE_INACTIVE;
		goto err;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

int cif_isp20_s_fmt_dma(
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

	if ((!CIF_ISP20_PIX_FMT_IS_YUV(strm_fmt->frm_fmt.pix_fmt) &&
		!CIF_ISP20_PIX_FMT_IS_RAW_BAYER(strm_fmt->frm_fmt.pix_fmt)) ||
		(CIF_ISP20_PIX_FMT_IS_YUV(strm_fmt->frm_fmt.pix_fmt) &&
		CIF_ISP20_PIX_FMT_YUV_IS_UV_SWAPPED(
			strm_fmt->frm_fmt.pix_fmt))) {
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

	dev->config.dma_config.updt_cfg = true;
	dev->dma_stream.state = CIF_ISP20_STATE_READY;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

void cif_isp20_init_sp(struct cif_isp20_device *dev)
{
	dev->config.sp_config.updt_cfg = false;
	INIT_LIST_HEAD(&dev->sp_stream.buf_queue);
	dev->sp_stream.next_buf = NULL;
	dev->sp_stream.curr_buf = NULL;

	/* default */
	if (dev->img_src == NULL)
		cif_isp20_s_input(dev, CIF_ISP20_INP_CSI_0);

	dev->sp_stream.state = CIF_ISP20_STATE_INACTIVE;
}

void cif_isp20_init_mp(
	struct cif_isp20_device *dev)
{
	dev->config.jpeg_config.ratio = 50;
	dev->config.jpeg_config.header =
		CIF_ISP20_JPEG_HEADER_JFIF;
	dev->config.jpeg_config.enable = false;
	dev->config.mi_config.raw_enable = false;
	dev->config.mp_config.updt_cfg = false;
	INIT_LIST_HEAD(&dev->mp_stream.buf_queue);
	dev->mp_stream.next_buf = NULL;
	dev->mp_stream.curr_buf = NULL;

	/* default */
	if (dev->img_src == NULL)
		cif_isp20_s_input(dev, CIF_ISP20_INP_CSI_0);

	dev->mp_stream.state = CIF_ISP20_STATE_INACTIVE;
}

struct cif_isp20_device *cif_isp20_create(
	CIF_ISP20_PLTFRM_DEVICE pdev)
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

	ret = cif_isp20_pltfrm_dev_init(dev,
		&pdev, &dev->config.base_addr);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_isp20_img_srcs_init(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_isp20_register_isrs(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_isp20_init_sp(dev);
	cif_isp20_init_mp(dev);
	dev->pm_state = CIF_ISP20_PM_STATE_OFF;
	dev->sp_stream.state = CIF_ISP20_STATE_DISABLED;
	dev->mp_stream.state = CIF_ISP20_STATE_DISABLED;

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
	if (IS_ERR_OR_NULL(dev))
		kfree(dev);
}

int cif_isp20_s_input(
	struct cif_isp20_device *dev,
	enum cif_isp20_inp inp)
{
	int ret;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"setting input to %s(0x%08x)\n",
		cif_isp20_inp_string(inp), inp);

	if (inp & CIF_ISP20_INP_CSI_0) {
		if (NULL == dev->img_src_array[0]) {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"no image source connected to CSI-0\n");
			ret = -EINVAL;
			goto err;
		}
		dev->img_src = dev->img_src_array[0];
		dev->config.mipi_config.input_sel = 0;
	} else if (inp & CIF_ISP20_INP_CSI_1) {
		if (NULL == dev->img_src_array[1]) {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"no image source connected to CSI-1\n");
			ret = -EINVAL;
			goto err;
		}
		dev->img_src = dev->img_src_array[1];
		dev->config.mipi_config.input_sel = 1;
	} else if (inp & CIF_ISP20_INP_CPI) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"parallel input currently not supported\n");
		ret = -EINVAL;
		goto err;
	} else /* DMA */ {
		if (inp & CIF_ISP20_INP_SI) {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"cannot do super impose on DMA input\n");
			ret = -EINVAL;
			goto err;
		}
		dev->img_src = NULL;
	}

	/* TODO: handle other possible input sources, e.g. readback path */
	dev->config.isp_config.input =
		&dev->config.img_src_output.frm_fmt;
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

	if ((inp & ~CIF_ISP20_INP_SI) <= CIF_ISP20_INP_CPI)
		return cif_isp20_img_src_g_name(dev->img_src);
	else
		return cif_isp20_inp_string(inp);
}

int cif_isp20_qbuf(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream,
	struct cif_isp20_buffer *buf)
{
	int ret = 0;

	switch (stream) {
	case CIF_ISP20_STREAM_SP:
		list_add_tail(&buf->queue, &dev->sp_stream.buf_queue);
		if ((dev->sp_stream.state == CIF_ISP20_STATE_STREAMING) &&
			(dev->sp_stream.next_buf == NULL)) {
			cif_iowrite32(CIF_MI_SP_FRAME,
				dev->config.base_addr +
				CIF_MI_ICR);
			cif_iowrite32OR(CIF_MI_SP_FRAME,
				dev->config.base_addr +
				CIF_MI_IMSC);
		}
		break;
	case CIF_ISP20_STREAM_MP:
		list_add_tail(&buf->queue, &dev->mp_stream.buf_queue);
		if ((dev->mp_stream.state == CIF_ISP20_STATE_STREAMING) &&
			(dev->mp_stream.next_buf == NULL)) {
			cif_iowrite32(CIF_MI_MP_FRAME,
				dev->config.base_addr +
				CIF_MI_ICR);
			cif_iowrite32OR(CIF_MI_MP_FRAME,
				dev->config.base_addr +
				CIF_MI_IMSC);
		}
		break;
	case CIF_ISP20_STREAM_DMA:
		list_add_tail(&buf->queue, &dev->dma_stream.buf_queue);
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
	if (*height < input_height)
		/* vertical cropping */
		*v_offs = (input_height - *height) >> 1;
	else if (*height > input_height) {
		/* horizontal cropping */
		*width = input_height * target_width / target_height;
		*h_offs = (input_width - *width) >> 1;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
}

int cif_isp20_calc_min_out_buff_size(
	struct cif_isp20_device *dev,
	enum cif_isp20_stream_id stream,
	u32 *size)
{
	int ret = 0;
	enum cif_isp20_pix_fmt pix_fmt;
	u32 llength;
	u32 height;
	u32 bpp;

	if (stream == CIF_ISP20_STREAM_SP) {
		if (dev->sp_stream.state < CIF_ISP20_STATE_READY) {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"cannot calculate buffer size, SP stream not ready\n");
			ret = -EINVAL;
			goto err;
		}
		pix_fmt = dev->config.mi_config.sp.output.pix_fmt;
		llength = dev->config.mi_config.sp.llength;
		height = dev->config.mi_config.sp.output.height;
	} else if (stream == CIF_ISP20_STREAM_MP) {
		if (dev->mp_stream.state < CIF_ISP20_STATE_READY) {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"cannot calculate buffer size, MP stream not ready\n");
			ret = -EINVAL;
			goto err;
		}
		pix_fmt = dev->config.mi_config.mp.output.pix_fmt;
		llength = dev->config.mi_config.mp.llength;
		height = dev->config.mi_config.mp.output.height;
	} else {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"cannot calculate buffer size for this stream (%d)\n",
			stream);
		ret = -EINVAL;
		goto err;
	}

	if (CIF_ISP20_PIX_FMT_IS_RAW_BAYER(pix_fmt) &&
		CIF_ISP20_PIX_FMT_GET_BPP(pix_fmt) > 8)
		/* RAW input > 8BPP is stored with 16BPP by MI */
		bpp = 16;
	else
		bpp = CIF_ISP20_PIX_FMT_GET_BPP(pix_fmt);
	*size = llength * height * bpp / 8;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with err %d\n", ret);
	return ret;
}

/*****************************************************************************/

int xgold_v4l2_level = 0x0f;

#define MARVIN_LIB_REG		(1<<4)
#define MARVIN_LIB_ISR		(1<<3)
#define MARVIN_LIB_ENTER	(1<<2)
#define MARVIN_LIB_INFO		(1<<1)
#define MARVIN_LIB_ERROR	(1<<0)

#define marvin_lib_level 0x1f
#define marvin_lib_debug(level, fmt, arg...) \
	do { \
		if (MARVIN_LIB_ERROR&level) \
			pr_err(fmt, ##arg); \
		else if (marvin_lib_level&level) \
			pr_debug(fmt, ##arg); \
	} while (0)

/****************************************************************************/


#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int xgold_cif_set_pm_state(struct device *,
		struct device_state_pm_state *);

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
#endif

/* ==================================================================== */

/* ================================================= */

static int marvin_lib_sp_scaler(struct cif_isp20_device *dev)
{
	unsigned int i = 0;
	unsigned int size_in_h;
	unsigned int size_in_v;
	unsigned int size_out_h;
	unsigned int size_out_v;
	enum cif_isp20_pix_fmt format_in;
	enum cif_isp20_pix_fmt format_out;
	unsigned int ret = 0;
	bool colour_downsampling = false;

	dev->config.sp_config.rsz_config.input =
		&dev->config.isp_config.output;
	dev->config.sp_config.rsz_config.output =
		dev->config.mi_config.sp.output;
	dev->config.sp_config.rsz_config.output.pix_fmt =
		dev->config.sp_config.rsz_config.input->pix_fmt;

	format_in = dev->config.sp_config.rsz_config.input->pix_fmt;
	size_in_h = dev->config.sp_config.rsz_config.input->width;
	size_in_v = dev->config.sp_config.rsz_config.input->height;
	format_out = dev->config.mi_config.sp.output.pix_fmt;
	size_out_h = dev->config.mi_config.sp.output.width;
	size_out_v = dev->config.mi_config.sp.output.height;

	if (CIF_ISP20_PIX_FMT_IS_YUV(format_out)) {
		if (CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(format_out) <
			CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(format_in))
			colour_downsampling = true;
		cif_isp20_pix_fmt_set_y_subs(
			dev->config.sp_config.rsz_config.output.pix_fmt,
			CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(format_out));
		cif_isp20_pix_fmt_set_bpp(
			dev->config.sp_config.rsz_config.output.pix_fmt,
			CIF_ISP20_PIX_FMT_GET_BPP(format_out));
	}

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s %dx%d -> %s %dx%d\n",
		cif_isp20_pix_fmt_string(
			dev->config.sp_config.rsz_config.input->pix_fmt),
		size_in_h, size_in_v,
		cif_isp20_pix_fmt_string(
			dev->config.sp_config.rsz_config.output.pix_fmt),
		size_out_h, size_out_v);

	/* Linear interpolation */
	for (i = 0; i < 64; i++) {
		cif_iowrite32(i,
		dev->config.base_addr +
		CIF_SRSZ_SCALE_LUT_ADDR);
		cif_iowrite32(i,
			dev->config.base_addr +
			CIF_SRSZ_SCALE_LUT);
	}

	if (size_out_h == size_in_h) {
		cif_iowrite32AND(~(CIF_SRSZ_CTRL_SCALE_HY_ENABLE |
			CIF_SRSZ_CTRL_SCALE_HC_ENABLE |
			CIF_SRSZ_CTRL_SCALE_HY_UP |
			CIF_SRSZ_CTRL_SCALE_HC_UP),
			dev->config.base_addr + CIF_SRSZ_CTRL);

		cif_iowrite32(0x0,
			dev->config.base_addr + CIF_SRSZ_SCALE_HY);
		cif_iowrite32(0x0,
			dev->config.base_addr +
			CIF_SRSZ_SCALE_HCB);
		cif_iowrite32(0x0,
			dev->config.base_addr +
			CIF_SRSZ_SCALE_HCR);
	} else if (size_out_h > size_in_h) {
		cif_iowrite32OR(CIF_SRSZ_CTRL_SCALE_HY_ENABLE |
				CIF_SRSZ_CTRL_SCALE_HC_ENABLE |
				CIF_SRSZ_CTRL_SCALE_HY_UP |
				CIF_SRSZ_CTRL_SCALE_HC_UP,
				dev->config.base_addr + CIF_SRSZ_CTRL);

		cif_iowrite32(((size_in_h - 1) * 16384) / (size_out_h - 1) + 1,
			dev->config.base_addr + CIF_SRSZ_SCALE_HY);
		cif_iowrite32(
			((size_in_h / 2 - 1) * 16384)
			/
			(size_out_h / 2 - 1) + 1,
			dev->config.base_addr +
			CIF_SRSZ_SCALE_HCB);
		cif_iowrite32(
			((size_in_h / 2 - 1) * 16384)
			/
			(size_out_h / 2 - 1) + 1,
			dev->config.base_addr +
			CIF_SRSZ_SCALE_HCR);
	} else {
		cif_iowrite32OR(CIF_SRSZ_CTRL_SCALE_HY_ENABLE |
			CIF_SRSZ_CTRL_SCALE_HC_ENABLE,
			dev->config.base_addr + CIF_SRSZ_CTRL);

		cif_iowrite32(
			((size_out_h - 1) * 16384)
			/
			(size_in_h - 1) + 1,
			dev->config.base_addr + CIF_SRSZ_SCALE_HY);
		cif_iowrite32(
			((size_out_h / 2 - 1) * 16384)
			/
			(size_in_h / 2 - 1) + 1,
			dev->config.base_addr +
			CIF_SRSZ_SCALE_HCB);
		cif_iowrite32(
			((size_out_h / 2 - 1) * 16384)
			/
			(size_in_h / 2 - 1) + 1,
			dev->config.base_addr +
			CIF_SRSZ_SCALE_HCR);
	}

	if (size_out_v == size_in_v) {
		cif_iowrite32AND(~(CIF_SRSZ_CTRL_SCALE_VY_ENABLE |
			CIF_SRSZ_CTRL_SCALE_VC_ENABLE |
			CIF_SRSZ_CTRL_SCALE_VY_UP |
			CIF_SRSZ_CTRL_SCALE_VC_UP),
			dev->config.base_addr + CIF_SRSZ_CTRL);

		if (colour_downsampling) {
			cif_iowrite32OR(CIF_SRSZ_CTRL_SCALE_VC_ENABLE,
				dev->config.base_addr +
				CIF_SRSZ_CTRL);

			cif_iowrite32AND(~(CIF_SRSZ_CTRL_SCALE_VC_UP),
				dev->config.base_addr +
				CIF_SRSZ_CTRL);

			cif_iowrite32((
				((size_out_v / 2 - 1) * 16384)
				/
				(size_in_v - 1) + 1),
				dev->config.base_addr +
				CIF_SRSZ_SCALE_VC);
		} else {
			cif_iowrite32(0x0,
				dev->config.base_addr +
				CIF_SRSZ_SCALE_VC);
		}

		cif_iowrite32(0x0,
			(dev->config.base_addr) + CIF_SRSZ_SCALE_VY);
	} else if (size_out_v > size_in_v) {
		cif_iowrite32OR(CIF_SRSZ_CTRL_SCALE_VY_ENABLE |
			CIF_SRSZ_CTRL_SCALE_VC_ENABLE |
			CIF_SRSZ_CTRL_SCALE_VY_UP |
			CIF_SRSZ_CTRL_SCALE_VC_UP,
			dev->config.base_addr + CIF_SRSZ_CTRL);

		cif_iowrite32(((size_in_v - 1) * 16384) / (size_out_v - 1) + 1,
			dev->config.base_addr + CIF_SRSZ_SCALE_VY);
		cif_iowrite32(((size_in_v - 1) * 16384) / (size_out_v - 1) + 1,
			dev->config.base_addr + CIF_SRSZ_SCALE_VC);
	} else {
		cif_iowrite32OR(CIF_SRSZ_CTRL_SCALE_VY_ENABLE |
			CIF_SRSZ_CTRL_SCALE_VC_ENABLE,
			(dev->config.base_addr) + CIF_SRSZ_CTRL);
		cif_iowrite32(((size_out_v - 1) * 16384) / (size_in_v - 1) + 1,
			dev->config.base_addr + CIF_SRSZ_SCALE_VY);
		if (colour_downsampling) {
			cif_iowrite32((
				((size_out_v / 2 - 1) * 16384)
				/
				(size_in_v - 1) + 1),
				dev->config.base_addr +
				CIF_SRSZ_SCALE_VC);
		} else {
			cif_iowrite32(
				((size_out_v - 1) * 16384)
				/
				(size_in_v - 1) + 1,
				(dev->config.base_addr) +
				CIF_SRSZ_SCALE_VC);
		}
	}

	/* No phase offset */
	cif_iowrite32(0, dev->config.base_addr + CIF_SRSZ_PHASE_HY);
	cif_iowrite32(0, dev->config.base_addr + CIF_SRSZ_PHASE_HC);
	cif_iowrite32(0, dev->config.base_addr + CIF_SRSZ_PHASE_VY);
	cif_iowrite32(0, dev->config.base_addr + CIF_SRSZ_PHASE_VC);

	/* SW update MRSZ */
	cif_iowrite32OR(CIF_SRSZ_CTRL_CFG_UPD,
		dev->config.base_addr + CIF_SRSZ_CTRL);

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
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_CTRL),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_CTRL_SHD),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_SCALE_HY),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_SCALE_HY_SHD),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_SCALE_HCB),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_SCALE_HCB_SHD),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_SCALE_HCR),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_SCALE_HCR_SHD),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_SCALE_VY),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_SCALE_VY_SHD),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_SCALE_VC),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_SCALE_VC_SHD),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_PHASE_HY),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_PHASE_HY_SHD),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_PHASE_HC),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_PHASE_HC_SHD),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_PHASE_VY),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_PHASE_VY_SHD),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_PHASE_VC),
		cif_ioread32(dev->config.base_addr + CIF_SRSZ_PHASE_VC_SHD));

	return ret;
}

static int marvin_lib_mp_scaler(struct cif_isp20_device *dev)
{
	unsigned int i = 0;
	unsigned int size_out_h = 0;
	unsigned int size_in_h = 0;
	unsigned int size_out_v = 0;
	unsigned int size_in_v = 0;
	enum cif_isp20_pix_fmt format_out;
	enum cif_isp20_pix_fmt format_in;
	unsigned int ret = 0;
	bool colour_downsampling = false;

	dev->config.mp_config.rsz_config.input =
		&dev->config.isp_config.output;
	dev->config.mp_config.rsz_config.output =
		dev->config.mi_config.mp.output;
	dev->config.mp_config.rsz_config.output.pix_fmt =
		dev->config.mp_config.rsz_config.input->pix_fmt;

	format_in = dev->config.mp_config.rsz_config.input->pix_fmt;
	size_in_h = dev->config.mp_config.rsz_config.input->width;
	size_in_v = dev->config.mp_config.rsz_config.input->height;
	format_out = dev->config.mi_config.mp.output.pix_fmt;
	size_out_h = dev->config.mi_config.mp.output.width;
	size_out_v = dev->config.mi_config.mp.output.height;

	if (CIF_ISP20_PIX_FMT_IS_YUV(format_out)) {
		if (CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(format_out) <
			CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(format_in))
			colour_downsampling = true;
		cif_isp20_pix_fmt_set_y_subs(
			dev->config.mp_config.rsz_config.output.pix_fmt,
			CIF_ISP20_PIX_FMT_YUV_GET_Y_SUBS(format_out));
		cif_isp20_pix_fmt_set_bpp(
			dev->config.mp_config.rsz_config.output.pix_fmt,
			CIF_ISP20_PIX_FMT_GET_BPP(format_out));
	}

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"%s %dx%d -> %s %dx%d\n",
		cif_isp20_pix_fmt_string(
			dev->config.mp_config.rsz_config.input->pix_fmt),
		size_in_h, size_in_v,
		cif_isp20_pix_fmt_string(
			dev->config.mp_config.rsz_config.output.pix_fmt),
		size_out_h, size_out_v);

	if (dev->config.mi_config.raw_enable) {
		cif_iowrite32(0,
			dev->config.base_addr + CIF_MRSZ_CTRL);
	} else {
		/* Linear interpolation */
		for (i = 0; i < 64; i++) {
			cif_iowrite32(i,
				dev->config.base_addr +
				CIF_MRSZ_SCALE_LUT_ADDR);
			cif_iowrite32(i,
				dev->config.base_addr +
				CIF_MRSZ_SCALE_LUT);
		}

		if (size_out_h == size_in_h) {
			/* No Horizontal scaling required */
			cif_iowrite32AND(~(CIF_MRSZ_CTRL_SCALE_HY_ENABLE |
				CIF_MRSZ_CTRL_SCALE_HC_ENABLE |
				CIF_MRSZ_CTRL_SCALE_HY_UP |
				CIF_MRSZ_CTRL_SCALE_HC_UP),
				dev->config.base_addr + CIF_MRSZ_CTRL);

			cif_iowrite32(0x0,
				dev->config.base_addr + CIF_MRSZ_SCALE_HY);
			cif_iowrite32(0x0,
				dev->config.base_addr +
				CIF_MRSZ_SCALE_HCB);
			cif_iowrite32(0x0,
				dev->config.base_addr +
				CIF_MRSZ_SCALE_HCR);

		} else if (size_out_h > size_in_h) {
			cif_iowrite32OR(CIF_MRSZ_CTRL_SCALE_HY_ENABLE |
				CIF_MRSZ_CTRL_SCALE_HC_ENABLE |
				CIF_MRSZ_CTRL_SCALE_HY_UP |
				CIF_MRSZ_CTRL_SCALE_HC_UP,
				dev->config.base_addr + CIF_MRSZ_CTRL);

			cif_iowrite32(
				((size_in_h - 1) * 16384) / (size_out_h - 1),
				dev->config.base_addr + CIF_MRSZ_SCALE_HY);
			cif_iowrite32(
				((size_in_h / 2 - 1) * 16384)
				/
				(size_out_h / 2 - 1),
				dev->config.base_addr +
				CIF_MRSZ_SCALE_HCB);
			cif_iowrite32(
				((size_in_h / 2 - 1) * 16384)
				/
				(size_out_h / 2 - 1),
				dev->config.base_addr +
				CIF_MRSZ_SCALE_HCR);

		} else {
			cif_iowrite32OR(CIF_MRSZ_CTRL_SCALE_HY_ENABLE |
				CIF_MRSZ_CTRL_SCALE_HC_ENABLE,
				dev->config.base_addr + CIF_MRSZ_CTRL);

			cif_iowrite32(
				((size_out_h - 1) * 16384)
				/
				(size_in_h - 1) + 1,
				dev->config.base_addr + CIF_MRSZ_SCALE_HY);
			cif_iowrite32(
				((size_out_h / 2 - 1) * 16384)
				/
				(size_in_h / 2 - 1) + 1,
				dev->config.base_addr +
				CIF_MRSZ_SCALE_HCB);
			cif_iowrite32(
				((size_out_h / 2 - 1) * 16384)
				/
				(size_in_h / 2 - 1) + 1,
				dev->config.base_addr +
				CIF_MRSZ_SCALE_HCR);
		}

		if (size_out_v == size_in_v) {
			cif_iowrite32AND(~(CIF_MRSZ_CTRL_SCALE_VY_ENABLE |
				CIF_MRSZ_CTRL_SCALE_VC_ENABLE |
				CIF_MRSZ_CTRL_SCALE_VY_UP |
				CIF_MRSZ_CTRL_SCALE_VC_UP),
				dev->config.base_addr + CIF_MRSZ_CTRL);

			if (colour_downsampling) {
				cif_iowrite32OR(CIF_MRSZ_CTRL_SCALE_VC_ENABLE,
					dev->config.base_addr +
					CIF_MRSZ_CTRL);

				cif_iowrite32AND(~(CIF_MRSZ_CTRL_SCALE_VC_UP),
					dev->config.base_addr +
					CIF_MRSZ_CTRL);

				cif_iowrite32((
					((size_out_v / 2 - 1) * 16384)
					/
					(size_in_v - 1) + 1),
					dev->config.base_addr +
					CIF_MRSZ_SCALE_VC);
			} else {
				cif_iowrite32(0x0,
					dev->config.base_addr +
					CIF_MRSZ_SCALE_VC);
			}

			cif_iowrite32(0x0,
				dev->config.base_addr + CIF_MRSZ_SCALE_VY);
		} else if (size_out_v > size_in_v) {
			cif_iowrite32OR(CIF_MRSZ_CTRL_SCALE_VY_ENABLE |
				CIF_MRSZ_CTRL_SCALE_VC_ENABLE |
				CIF_MRSZ_CTRL_SCALE_VY_UP |
				CIF_MRSZ_CTRL_SCALE_VC_UP,
				dev->config.base_addr + CIF_MRSZ_CTRL);

			cif_iowrite32(
				((size_in_v - 1) * 16384) / (size_out_v - 1),
				dev->config.base_addr + CIF_MRSZ_SCALE_VY);
			cif_iowrite32(
				((size_in_v - 1) * 16384) / (size_out_v - 1),
				dev->config.base_addr + CIF_MRSZ_SCALE_VC);
		} else {
			cif_iowrite32OR(CIF_MRSZ_CTRL_SCALE_VY_ENABLE |
				CIF_MRSZ_CTRL_SCALE_VC_ENABLE,
				dev->config.base_addr + CIF_MRSZ_CTRL);

			cif_iowrite32(
				((size_out_v - 1) * 16384)
				/
				(size_in_v - 1) + 1,
				dev->config.base_addr + CIF_MRSZ_SCALE_VY);

			if (colour_downsampling) {
				cif_iowrite32((
					((size_out_v / 2 - 1) * 16384)
					/
					(size_in_v - 1) + 1),
					dev->config.base_addr +
					CIF_MRSZ_SCALE_VC);
			} else {
				cif_iowrite32(
					((size_out_v - 1) * 16384)
					/
					(size_in_v - 1) + 1,
					dev->config.base_addr +
					CIF_MRSZ_SCALE_VC);
			}
		}
	}

	/* No phase offset */
	cif_iowrite32(0, dev->config.base_addr + CIF_MRSZ_PHASE_HY);
	cif_iowrite32(0, dev->config.base_addr + CIF_MRSZ_PHASE_HC);
	cif_iowrite32(0, dev->config.base_addr + CIF_MRSZ_PHASE_VY);
	cif_iowrite32(0, dev->config.base_addr + CIF_MRSZ_PHASE_VC);

	cif_iowrite32OR(CIF_MRSZ_CTRL_CFG_UPD,
			dev->config.base_addr + CIF_MRSZ_CTRL);

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
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_CTRL),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_CTRL_SHD),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_SCALE_HY),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_SCALE_HY_SHD),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_SCALE_HCB),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_SCALE_HCB_SHD),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_SCALE_HCR),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_SCALE_HCR_SHD),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_SCALE_VY),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_SCALE_VY_SHD),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_SCALE_VC),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_SCALE_VC_SHD),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_PHASE_HY),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_PHASE_HY_SHD),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_PHASE_HC),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_PHASE_HC_SHD),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_PHASE_VY),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_PHASE_VY_SHD),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_PHASE_VC),
		cif_ioread32(dev->config.base_addr + CIF_MRSZ_PHASE_VC_SHD));

	return ret;
}

static int config_sp(
		    struct cif_isp20_device *dev)
{
	int ret = 0;
	struct cif_isp20_mi_state saved_mi_sp_state;

	xgold_v4l2_debug(XGOLD_V4L2_INFO,
		"%s: %s\n", DRIVER_NAME, __func__);

	ret = marvin_lib_sp_scaler(dev);
	if (IS_ERR_VALUE(ret))
		goto err;
	/* Work-around for SMS04982237
		See 'config_mp' for more details.
	*/
	cif_isp20_save_mi_sp(dev, &saved_mi_sp_state);
	cif_isp20_restore_mi_sp(dev, &saved_mi_sp_state);
	ret = cif_isp20_config_mi_sp(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	dev->config.sp_config.updt_cfg = false;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int config_mp(
		    struct cif_isp20_device *dev)
{
	int ret = 0;
	struct cif_isp20_mi_state saved_mi_mp_state;

	xgold_v4l2_debug(XGOLD_V4L2_INFO,
			    " %s: %s\n",
			    DRIVER_NAME, __func__);

	ret = marvin_lib_mp_scaler(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	/* Work-around for SMS04982237
		The write to CIF_MI_MP_Y_SIZE_INIT in cif_isp20_config_mi_mp()
		fails from time to time. We found adding a register read before
		the write can temporarily fix the issue. Since we are not sure
		whether it is a general issue or only for this specific
		register, we read all the MI MP registers here using the
		save_mi_mp() functions. Strange thing is, we need to
		put these two functions just before marvin_lib_mi_mp(),
		otherwise the register write still fails.
	*/
	cif_isp20_save_mi_mp(dev, &saved_mi_mp_state);
	cif_isp20_restore_mi_mp(dev, &saved_mi_mp_state);
	ret = cif_isp20_config_mi_mp(dev);
	if (IS_ERR_VALUE(ret))
		goto err;
	if (dev->config.jpeg_config.enable) {
		ret = cif_isp20_config_jpeg_enc(dev);
		if (IS_ERR_VALUE(ret))
			goto err;
		dev->config.jpeg_config.busy = false;
	}

	dev->config.mp_config.updt_cfg = false;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int ie_configure(struct cif_isp20_device *dev)
{
/*set image effect*/
	cif_iowrite32OR(0x00000100, (dev->config.base_addr) + CIF_ICCL);
	dev->config.control.val = 1;
	dev->config.control.id = dev->config.ei_config.image_effect;
	marvin_lib_s_control(dev);
	return 0;
}

int config_cif(struct cif_isp20_device *dev)
{
	int ret = 0;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"config MP = %d, config SP = %d, img_src state = %s, PM state = %s, SP state = %s, MP state = %s\n",
		dev->config.mp_config.updt_cfg,
		dev->config.sp_config.updt_cfg,
		cif_isp20_img_src_state_string(dev->img_src_state),
		cif_isp20_pm_state_string(dev->pm_state),
		cif_isp20_state_string(dev->sp_stream.state),
		cif_isp20_state_string(dev->mp_stream.state));

	/* configure sensor */
	if (!dev->config.mp_config.updt_cfg &&
		!dev->config.sp_config.updt_cfg)
		return 0;

	if ((dev->config.input_sel & ~CIF_ISP20_INP_SI) <=
		CIF_ISP20_INP_CPI) {
		ret = cif_isp20_config_img_src(dev);
		if (IS_ERR_VALUE(ret))
			goto err;
	}

	ret = cif_isp20_set_pm_state(dev,
		CIF_ISP20_PM_STATE_SW_STNDBY);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"CIF_ID 0x%08x\n",
		cif_ioread32(dev->config.base_addr + CIF_VI_ID));

	cif_iowrite32(CIF_IRCL_CIF_SW_RST,
		(dev->config.base_addr) + CIF_IRCL);

	cif_isp20_config_clk(dev);

	ret = cif_isp20_config_path(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = cif_isp20_config_mipi(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ie_configure(dev);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = cif_isp20_config_isp(dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (dev->config.dma_config.updt_cfg) {
		ret = cif_isp20_config_mi_dma(dev);
		if (IS_ERR_VALUE(ret))
			goto err;
	}

	if (dev->config.sp_config.updt_cfg) {
		ret = config_sp(dev);
		if (IS_ERR_VALUE(ret))
			goto err;
	}
	if (dev->config.mp_config.updt_cfg) {
		ret = config_mp(dev);
		if (IS_ERR_VALUE(ret))
			goto err;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

/* =============================================== */

/* ==================================================== */

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

/* ======================================================================= */

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
#if 0
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
#endif

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
#endif

/* ======================================================================= */

#ifdef SOFIA_ES1_BU_PM_NATIVE
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

/* ======================================================================== */

/* ===============================DEBUG==================================== */

/***************************************************************************/
/*!
********************************************************************
*	\function:  marvin_lib_mi_address\n
*
*	\par Description: set mi base addresses (called under isr)
*
*	\param struct cif_isp20_config *marvin_config\n
*
*	\return Status value\n
*
*	\par HISTORY (ascending):
*
*********************************************************************/
unsigned int marvin_lib_mi_address(struct cif_isp20_device *dev,
	int write_sp)
{
	if (write_sp) {
		cif_iowrite32(dev->config.mi_config.sp.curr_buff_addr,
			      (dev->config.base_addr) +
			      CIF_MI_SP_Y_BASE_AD_INIT);
		if (dev->config.mi_config.sp.curr_buff_addr ==
		    dev->config.mi_config.null_buff_dma_addr) {
			cif_iowrite32(dev->config.mi_config.sp.curr_buff_addr +
				      3 * CIF_NULL_BUFF_SIZE,
				      (dev->config.base_addr) +
				      CIF_MI_SP_Y_BASE_AD_INIT);
			cif_iowrite32(dev->config.mi_config.
				      null_buff_dma_addr +
				      4 * CIF_NULL_BUFF_SIZE,
				      (dev->config.base_addr) +
				      CIF_MI_SP_CB_BASE_AD_INIT);
			cif_iowrite32(dev->config.mi_config.
				      null_buff_dma_addr +
				      5 * CIF_NULL_BUFF_SIZE,
				      (dev->config.base_addr) +
				      CIF_MI_SP_CR_BASE_AD_INIT);
		} else {
			cif_iowrite32(dev->config.mi_config.sp.curr_buff_addr +
				      dev->config.mi_config.sp.cb_offs,
				      (dev->config.base_addr) +
				      CIF_MI_SP_CB_BASE_AD_INIT);
			cif_iowrite32(dev->config.mi_config.sp.curr_buff_addr +
				      dev->config.mi_config.sp.cr_offs,
				      (dev->config.base_addr) +
				      CIF_MI_SP_CR_BASE_AD_INIT);
		}
	} else {
		cif_iowrite32(dev->config.mi_config.mp.curr_buff_addr,
			      (dev->config.base_addr) +
			      CIF_MI_MP_Y_BASE_AD_INIT);
		if (dev->config.mi_config.mp.curr_buff_addr ==
		    dev->config.mi_config.null_buff_dma_addr) {
			cif_iowrite32(dev->config.mi_config.
				      null_buff_dma_addr + CIF_NULL_BUFF_SIZE,
				      (dev->config.base_addr) +
				      CIF_MI_MP_CB_BASE_AD_INIT);
			cif_iowrite32(dev->config.mi_config.
				      null_buff_dma_addr +
				      2 * CIF_NULL_BUFF_SIZE,
				      (dev->config.base_addr) +
				      CIF_MI_MP_CR_BASE_AD_INIT);
		} else {
			cif_iowrite32(dev->config.mi_config.mp.curr_buff_addr +
				      dev->config.mi_config.mp.cb_offs,
				      (dev->config.base_addr) +
				      CIF_MI_MP_CB_BASE_AD_INIT);
			cif_iowrite32(dev->config.mi_config.mp.curr_buff_addr +
				      dev->config.mi_config.mp.cr_offs,
				      (dev->config.base_addr) +
				      CIF_MI_MP_CR_BASE_AD_INIT);
		}
	}

	return 0;
}

/********************************************************************
*\function:  marvin_lib_control\n
*
*	\par Description: marvin_lib_s_control
*
*	\param   none\n
*
*	\return Status\n
*
*	\par HISTORY (ascending):
*
*********************************************************************/
unsigned int marvin_lib_s_control(struct cif_isp20_device *dev)
{
	unsigned int id = dev->config.control.id;
	unsigned int val = dev->config.control.val;
	unsigned int ret = 0;

   /* these controls do not need the CIF powered up */
	if (id == marvin_jpeg_quality) {
		dev->config.jpeg_config.ratio = val;
		return ret;
	}

	if ((dev->pm_state != CIF_ISP20_PM_STATE_SW_STNDBY) &&
		(dev->pm_state != CIF_ISP20_PM_STATE_STREAMING))
		return 0;

	switch (id) {

	case marvin_hflip:
		if (val) {
			/* hflip was enabled so disabled it */
			cif_iowrite32AND(~CIF_MI_CTRL_HFLIP,
					 (dev->config.base_addr) +
					 CIF_MI_CTRL);
		} else {
			/* hflip was disabled so enabled it */
			cif_iowrite32OR(CIF_MI_CTRL_HFLIP,
					(dev->config.base_addr) +
					CIF_MI_CTRL);
		}
		/* mi_update*/
		cif_iowrite32OR(CIF_MI_INIT_SOFT_UPD,
				(dev->config.base_addr) + CIF_MI_INIT);

		/* udpate marvin structure (avoid overwrite) */
		break;
	case marvin_vflip:
		if (val) {
			/* hflip was enabled so disabled it */
			cif_iowrite32AND(~CIF_MI_CTRL_VFLIP,
					 (dev->config.base_addr) +
					 CIF_MI_CTRL);
		} else {
			/* hflip was disabled so enabled it */
			cif_iowrite32OR(CIF_MI_CTRL_VFLIP,
					(dev->config.base_addr) +
					CIF_MI_CTRL);
		}
		/* mi_update*/
		cif_iowrite32OR(CIF_MI_INIT_SOFT_UPD,
				(dev->config.base_addr) + CIF_MI_INIT);
		/* udpate marvin structure (avoid overwrite) */
		break;

	case marvin_rotate:
		/* NOTE: Is it realistic to handle rotation as v4l2 control? */
		BUG();
		break;
	case marvin_sepia:
		if (val) {
			cif_iowrite32(CIF_IMG_EFF_CTRL_ENABLE |
				      CIF_IMG_EFF_CTRL_MODE_SEPIA,
				      (dev->config.base_addr) +
				      CIF_IMG_EFF_CTRL);
		} else {
			cif_iowrite32AND(~(CIF_IMG_EFF_CTRL_ENABLE |
					   CIF_IMG_EFF_CTRL_MODE_SEPIA),
					 (dev->config.base_addr) +
					 CIF_IMG_EFF_CTRL);
		}
		cif_iowrite32OR(CIF_IMG_EFF_CTRL_CFG_UPD,
				(dev->config.base_addr) +
				CIF_IMG_EFF_CTRL);
		break;
	case marvin_black_and_white:
		if (val) {
			cif_iowrite32(CIF_IMG_EFF_CTRL_ENABLE |
				      CIF_IMG_EFF_CTRL_MODE_BLACKWHITE,
				      (dev->config.base_addr) +
				      CIF_IMG_EFF_CTRL);
		} else {
			cif_iowrite32AND(~(CIF_IMG_EFF_CTRL_ENABLE |
					   CIF_IMG_EFF_CTRL_MODE_BLACKWHITE),
					 (dev->config.base_addr) +
					 CIF_IMG_EFF_CTRL);
		}
		cif_iowrite32OR(CIF_IMG_EFF_CTRL_CFG_UPD,
				(dev->config.base_addr) +
				CIF_IMG_EFF_CTRL);
		break;
	case marvin_negative:
		if (val) {
			cif_iowrite32(CIF_IMG_EFF_CTRL_ENABLE |
				      CIF_IMG_EFF_CTRL_MODE_NEGATIVE,
				      (dev->config.base_addr) +
				      CIF_IMG_EFF_CTRL);
		} else {
			cif_iowrite32AND(~(CIF_IMG_EFF_CTRL_ENABLE |
					   CIF_IMG_EFF_CTRL_MODE_NEGATIVE),
					 (dev->config.base_addr) +
					 CIF_IMG_EFF_CTRL);
		}
		cif_iowrite32OR(CIF_IMG_EFF_CTRL_CFG_UPD,
				(dev->config.base_addr) +
				CIF_IMG_EFF_CTRL);
		break;
	case marvin_none_ie:
		cif_iowrite32AND(~(CIF_IMG_EFF_CTRL_ENABLE),
				 (dev->config.base_addr) +
				 CIF_IMG_EFF_CTRL);
		cif_iowrite32OR(CIF_IMG_EFF_CTRL_CFG_UPD,
				(dev->config.base_addr) +
				CIF_IMG_EFF_CTRL);
		break;
	case marvin_color_selection:
		if (val) {
			cif_iowrite32(CIF_IMG_EFF_CTRL_ENABLE |
				      CIF_IMG_EFF_CTRL_MODE_COLOR_SEL,
				      (dev->config.base_addr) +
				      CIF_IMG_EFF_CTRL);
		} else {
			cif_iowrite32AND(~(CIF_IMG_EFF_CTRL_ENABLE |
					   CIF_IMG_EFF_CTRL_MODE_COLOR_SEL),
					 (dev->config.base_addr) +
					 CIF_IMG_EFF_CTRL);
		}
		cif_iowrite32(0x4004,
			      (dev->config.base_addr) +
			      CIF_IMG_EFF_COLOR_SEL);
		cif_iowrite32OR(CIF_IMG_EFF_CTRL_CFG_UPD,
				(dev->config.base_addr) +
				CIF_IMG_EFF_CTRL);
		break;
	case marvin_emboss:
		if (val) {
			cif_iowrite32(CIF_IMG_EFF_CTRL_ENABLE |
				      CIF_IMG_EFF_CTRL_MODE_EMBOSS,
				      (dev->config.base_addr) +
				      CIF_IMG_EFF_CTRL);
		} else {
			cif_iowrite32AND(~(CIF_IMG_EFF_CTRL_ENABLE |
					   CIF_IMG_EFF_CTRL_MODE_EMBOSS),
					 (dev->config.base_addr) +
					 CIF_IMG_EFF_CTRL);
		}
		cif_iowrite32OR(CIF_IMG_EFF_CTRL_CFG_UPD,
				(dev->config.base_addr) +
				CIF_IMG_EFF_CTRL);
		break;
	case marvin_sketch:
		if (val) {
			cif_iowrite32(CIF_IMG_EFF_CTRL_ENABLE |
				      CIF_IMG_EFF_CTRL_MODE_SKETCH,
				      (dev->config.base_addr) +
				      CIF_IMG_EFF_CTRL);
		} else {
			cif_iowrite32AND(~(CIF_IMG_EFF_CTRL_ENABLE |
					   CIF_IMG_EFF_CTRL_MODE_SKETCH),
					 (dev->config.base_addr) +
					 CIF_IMG_EFF_CTRL);
		}
		cif_iowrite32OR(CIF_IMG_EFF_CTRL_CFG_UPD,
				(dev->config.base_addr) +
				CIF_IMG_EFF_CTRL);
		break;
	default:
		BUG();
		break;
	}
	return ret;
}

/********************************************************************
*\function:  marvin_lib_program_jpeg_tables\n
*
*	\par Description: progam jpeg tables
*
*	\param   none\n
*
*	\return Status\n
*
*	\par HISTORY (ascending):
*
*********************************************************************/
int marvin_lib_program_jpeg_tables(struct cif_isp20_device *dev)
{

	unsigned int ratio = dev->config.jpeg_config.ratio;
	unsigned int i = 0;
	unsigned int ret = 0;
	unsigned int q, q_next, scale;
	BUG_ON(!(dev->config.base_addr));

	marvin_lib_debug(MARVIN_LIB_ENTER, " %s: %s: Enter...\n", DRIVER_NAME,
			 __func__);
	marvin_lib_debug(MARVIN_LIB_INFO, " %s: %s: ratio: %d\n", DRIVER_NAME,
			 __func__, ratio);

	/* Y q-table 0 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_QUANT0,
		      (dev->config.base_addr) + CIF_JPE_TABLE_ID);
	for (i = 0; i < 64 / 2; i++) {
		q = yq_table_base[i * 2];
		q_next = yq_table_base[i * 2 + 1];
		if (ratio != 50) {
			scale = (ratio < 50) ? 5000/ratio : 200 - (ratio << 1);
			q = (scale * q + 50) / 100;
			q_next = (scale * q_next + 50) / 100;
		}
		cif_iowrite32(q_next + (q << 8),
			      (dev->config.base_addr) +
			      CIF_JPE_TABLE_DATA);
	}

	/* U/V q-table 0 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_QUANT1,
		      (dev->config.base_addr) + CIF_JPE_TABLE_ID);
	for (i = 0; i < 64 / 2; i++) {
		q = uvq_table_base[i * 2];
		q_next = uvq_table_base[i * 2 + 1];
		if (ratio != 50) {
			scale = (ratio < 50) ? 5000/ratio : 200 - (ratio << 1);
			q = (scale * q + 50) / 100;
			q_next = (scale * q_next + 50) / 100;
		}
		cif_iowrite32(q_next + (q << 8),
			      (dev->config.base_addr) +
			      CIF_JPE_TABLE_DATA);
	}

	/* Y AC-table 0 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_HUFFAC0,
		      (dev->config.base_addr) + CIF_JPE_TABLE_ID);
	cif_iowrite32(178, (dev->config.base_addr) + CIF_JPE_TAC0_LEN);
	for (i = 0; i < (178 / 2); i++) {
		cif_iowrite32(ac_luma_table_annex_k[i * 2 + 1] +
			      (ac_luma_table_annex_k[i * 2] << 8),
			      (dev->config.base_addr) +
			      CIF_JPE_TABLE_DATA);
	}

	/* U/V AC-table 1 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_HUFFAC1,
		      (dev->config.base_addr) + CIF_JPE_TABLE_ID);
	cif_iowrite32(178, (dev->config.base_addr) + CIF_JPE_TAC1_LEN);
	for (i = 0; i < (178 / 2); i++) {
		cif_iowrite32(ac_chroma_table_annex_k[i * 2 + 1] +
			      (ac_chroma_table_annex_k[i * 2] << 8),
			      (dev->config.base_addr) +
			      CIF_JPE_TABLE_DATA);
	}

	/* Y DC-table 0 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_HUFFDC0,
		      (dev->config.base_addr) + CIF_JPE_TABLE_ID);
	cif_iowrite32(28, (dev->config.base_addr) + CIF_JPE_TDC0_LEN);
	for (i = 0; i < (28 / 2); i++) {
		cif_iowrite32(dc_luma_table_annex_k[i * 2 + 1] +
			      (dc_luma_table_annex_k[i * 2] << 8),
			      (dev->config.base_addr) +
			      CIF_JPE_TABLE_DATA);
	}

	/* U/V DC-table 1 programming */
	cif_iowrite32(CIF_JPE_TAB_ID_HUFFDC1,
		      (dev->config.base_addr) + CIF_JPE_TABLE_ID);
	cif_iowrite32(28, (dev->config.base_addr) + CIF_JPE_TDC1_LEN);
	for (i = 0; i < (28 / 2); i++) {
		cif_iowrite32(dc_chroma_table_annex_k[i * 2 + 1] +
			      (dc_chroma_table_annex_k[i * 2] << 8),
			      (dev->config.base_addr) +
			      CIF_JPE_TABLE_DATA);
	}

	return ret;
}

/********************************************************************
*\function:  marvin_lib_select_jpeg_tables\n
*
*	\par Description: select jpeg tables
*
*	\param   none\n
*
*	\return Status\n
*
*	\par HISTORY (ascending):
*
*********************************************************************/
int marvin_lib_select_jpeg_tables(struct cif_isp20_device *dev)
{
	unsigned int ret = 0;

	marvin_lib_debug(MARVIN_LIB_ENTER, " %s: %s: Enter...\n", DRIVER_NAME,
			 __func__);
	BUG_ON(!(dev->config.base_addr));

	/* Selects quantization table for Y */
	cif_iowrite32(CIF_JPE_TQ_TAB0,
		      (dev->config.base_addr) + CIF_JPE_TQ_Y_SELECT);
	/* Selects quantization table for U */
	cif_iowrite32(CIF_JPE_TQ_TAB1,
		      (dev->config.base_addr) + CIF_JPE_TQ_U_SELECT);
	/* Selects quantization table for V */
	cif_iowrite32(CIF_JPE_TQ_TAB1,
		      (dev->config.base_addr) + CIF_JPE_TQ_V_SELECT);
	/* Selects Huffman DC table */
	cif_iowrite32(CIF_DC_V_TABLE | CIF_DC_U_TABLE,
		      (dev->config.base_addr) + CIF_JPE_DC_TABLE_SELECT);
	/* Selects Huffman AC table */
	cif_iowrite32(CIF_AC_V_TABLE | CIF_AC_U_TABLE,
		      (dev->config.base_addr) + CIF_JPE_AC_TABLE_SELECT);

	marvin_lib_debug(MARVIN_LIB_REG,
			 " %s: %s: CIF_JPE_TQ_Y_SELECT = 0x%x\n", DRIVER_NAME,
			 __func__,
			 cif_ioread32((dev->config.base_addr) +
				      CIF_JPE_TQ_Y_SELECT));
	marvin_lib_debug(MARVIN_LIB_REG,
			 " %s: %s: CIF_JPE_TQ_U_SELECT = 0x%x\n", DRIVER_NAME,
			 __func__,
			 cif_ioread32((dev->config.base_addr) +
				      CIF_JPE_TQ_U_SELECT));
	marvin_lib_debug(MARVIN_LIB_REG,
			 " %s: %s: CIF_JPE_TQ_V_SELECT = 0x%x\n", DRIVER_NAME,
			 __func__,
			 cif_ioread32((dev->config.base_addr) +
				      CIF_JPE_TQ_V_SELECT));
	marvin_lib_debug(MARVIN_LIB_REG,
			 " %s: %s: CIF_JPE_DC_TABLE_SELECT = 0x%x\n",
			 DRIVER_NAME, __func__,
			 cif_ioread32((dev->config.base_addr) +
				      CIF_JPE_DC_TABLE_SELECT));
	marvin_lib_debug(MARVIN_LIB_REG,
			 " %s: %s: CIF_JPE_AC_TABLE_SELECT = 0x%x\n",
			 DRIVER_NAME, __func__,
			 cif_ioread32((dev->config.base_addr) +
				      CIF_JPE_AC_TABLE_SELECT));

	return ret;
}

/* ======================================================================== */
static void marvin_hw_restart(struct cif_isp20_device *dev)
{
	xgold_v4l2_debug(XGOLD_V4L2_ISR,
			 "%s: %s\n",
			 DRIVER_NAME, __func__);

	marvin_hw_errors[isp_pic_size_err].count = 0;
	marvin_hw_errors[isp_data_loss].count = 0;
	marvin_hw_errors[csi_err_protocol].count = 0;
	marvin_hw_errors[csi_ecc1_err].count = 0;
	marvin_hw_errors[csi_ecc2_err].count = 0;
	marvin_hw_errors[csi_cs_err].count = 0;
	cif_iowrite32(0x00000841, (dev->config.base_addr) + CIF_IRCL);
	cif_iowrite32(0x0, (dev->config.base_addr) + CIF_IRCL);

	/* enable mipi frame end interrupt*/
	cif_iowrite32(CIF_MIPI_FRAME_END,
		      (dev->config.base_addr) + CIF_MIPI_IMSC);
	/* enable csi protocol errors interrupts*/
	cif_iowrite32OR(CIF_MIPI_ERR_CSI,
			(dev->config.base_addr) + CIF_MIPI_IMSC);
	/* enable dphy errors interrupts*/
	cif_iowrite32OR(CIF_MIPI_ERR_DPHY,
			(dev->config.base_addr) + CIF_MIPI_IMSC);
	/* add fifo error*/
	cif_iowrite32OR(CIF_MIPI_SYNC_FIFO_OVFLW(3),
			(dev->config.base_addr) + CIF_MIPI_IMSC);
	/* add data overflow_error*/
	cif_iowrite32OR(CIF_MIPI_ADD_DATA_OVFLW,
			(dev->config.base_addr) + CIF_MIPI_IMSC);

	cif_iowrite32(0x0,
		      (dev->config.base_addr) + CIF_MI_MP_Y_OFFS_CNT_INIT);
	cif_iowrite32(0x0,
		      (dev->config.base_addr) +
		      CIF_MI_MP_CR_OFFS_CNT_INIT);
	cif_iowrite32(0x0,
		      (dev->config.base_addr) +
		      CIF_MI_MP_CB_OFFS_CNT_INIT);
	cif_iowrite32(0x0,
		      (dev->config.base_addr) + CIF_MI_SP_Y_OFFS_CNT_INIT);
	cif_iowrite32(0x0,
		      (dev->config.base_addr) +
		      CIF_MI_SP_CR_OFFS_CNT_INIT);
	cif_iowrite32(0x0,
		      (dev->config.base_addr) +
		      CIF_MI_SP_CB_OFFS_CNT_INIT);
	cif_iowrite32OR(CIF_MI_CTRL_INIT_OFFSET_EN,
			(dev->config.base_addr) + CIF_MI_CTRL);

	/* Enable ISP ! */
	cif_iowrite32OR(CIF_ISP_CTRL_ISP_CFG_UPD |
			CIF_ISP_CTRL_ISP_INFORM_ENABLE |
			CIF_ISP_CTRL_ISP_ENABLE,
			(dev->config.base_addr) + CIF_ISP_CTRL);
	/*enable MIPI */
	cif_iowrite32OR(CIF_MIPI_CTRL_OUTPUT_ENA,
			(dev->config.base_addr) + CIF_MIPI_CTRL);
}

int marvin_mipi_isr(void *cntxt)
{
	struct cif_isp20_device *dev =
	    (struct cif_isp20_device *)cntxt;
	unsigned int mipi_mis = 0;
	unsigned int i = 0;

	mipi_mis =
	    cif_ioread32((dev->config.base_addr) + CIF_MIPI_MIS);

	if (mipi_mis & CIF_MIPI_ERR_DPHY) {
		/* clear_mipi_dphy_error*/
		cif_iowrite32(CIF_MIPI_ERR_DPHY,
			      (dev->config.base_addr) + CIF_MIPI_ICR);

		for (i = 0;
		     i <
		     (sizeof(marvin_hw_errors) /
		      sizeof(struct smarvin_hw_errors)); i++) {
			if (marvin_hw_errors[i].type != 1)
				continue;	/*skip if not mipi dphy error*/

			if (marvin_hw_errors[i].
			    mask & (mipi_mis & CIF_MIPI_ERR_DPHY)) {
				marvin_hw_errors[i].count++;
				dev_err(dev->dev,
					"CIF_MIPI_ERR_DPHY");
				break;
			}
		}
	} else if (mipi_mis & CIF_MIPI_ERR_CSI) {
		/*clear_mipi_csi_error*/
		cif_iowrite32(CIF_MIPI_ERR_CSI,
			      (dev->config.base_addr) + CIF_MIPI_ICR);

		for (i = 0;
		     i <
		     (sizeof(marvin_hw_errors) /
		      sizeof(struct smarvin_hw_errors)); i++) {
			if (marvin_hw_errors[i].type != 2)
				continue;	/*skip if not mipi csi error*/

			if (marvin_hw_errors[i].
			    mask & (mipi_mis & CIF_MIPI_ERR_CSI)) {
				marvin_hw_errors[i].count++;
				dev_err(dev->dev,
					"CIF_MIPI_ERR_CSI %x",
					mipi_mis);
				break;
			}
		}
	} else if ((mipi_mis & CIF_MIPI_SYNC_FIFO_OVFLW(3))) {

		/* clear_mipi_fifo_error*/
		cif_iowrite32(CIF_MIPI_SYNC_FIFO_OVFLW(3),
			      (dev->config.base_addr) + CIF_MIPI_ICR);
		cif_iowrite32OR(CIF_MIPI_CTRL_OUTPUT_ENA,
				(dev->config.base_addr) + CIF_MIPI_CTRL);
		for (i = 0;
		     i <
		     (sizeof(marvin_hw_errors) /
		      sizeof(struct smarvin_hw_errors)); i++) {
			if (marvin_hw_errors[i].type != 2)
				continue;	/*skip if not mipi error*/

			if (marvin_hw_errors[i].
			    mask & (mipi_mis & CIF_MIPI_SYNC_FIFO_OVFLW(3))) {
				marvin_hw_errors[i].count++;
				dev_err(dev->dev,
					"CIF_MIPI_SYNC_FIFO_OVFLW\n");
				break;
			}
		}
	} else if (mipi_mis & CIF_MIPI_ADD_DATA_OVFLW) {
		/* clear_mipi_fifo_error*/
		cif_iowrite32(CIF_MIPI_ADD_DATA_OVFLW,
				  (dev->config.base_addr) + CIF_MIPI_ICR);

		for (i = 0;
		     i <
		     (sizeof(marvin_hw_errors) /
		      sizeof(struct smarvin_hw_errors)); i++) {
			if (marvin_hw_errors[i].type != 1)
				continue;	/*skip if not mipi error*/

			if (marvin_hw_errors[i].
			    mask & (mipi_mis & CIF_MIPI_ADD_DATA_OVFLW)) {
				marvin_hw_errors[i].count++;
				dev_err(dev->dev,
					"CIF_MIPI_ADD_DATA_OVFLW");
				break;
			}
		}
	}

	if (mipi_mis & CIF_MIPI_FRAME_END) {
		cif_iowrite32(CIF_MIPI_FRAME_END,
			      (dev->config.base_addr) + CIF_MIPI_ICR);
	}

	return 0;
}

static int update_mi_mp(struct cif_isp20_device *dev)
{
	int ret = 0;

	if (dev->config.jpeg_config.enable) {
		/* in case of jpeg encoding, we don't have to disable the
		   MI, because the encoding
		   anyway has to be started explicitely */
		if (!dev->config.jpeg_config.busy) {
			if ((dev->config.mi_config.mp.next_buff_addr ==
				 dev->config.mi_config.mp.curr_buff_addr)
				 && (dev->config.mi_config.mp.next_buff_addr !=
				dev->config.mi_config.null_buff_dma_addr)) {
				ret = cif_isp20_jpeg_gen_header(dev);
				cif_iowrite32(CIF_JPE_ENCODE_ENABLE,
					      (dev->config.base_addr) +
					      CIF_JPE_ENCODE);
				dev->config.jpeg_config.busy = true;
			}
		}

		dev->config.mi_config.mp.curr_buff_addr =
			dev->config.mi_config.mp.next_buff_addr;
		ret += marvin_lib_mi_address(dev, 0);
		cif_iowrite32OR(CIF_ISP_CTRL_ISP_GEN_CFG_UPD,
				(dev->config.base_addr) + CIF_ISP_CTRL);
	} else {
		if (dev->config.mi_config.mp.next_buff_addr !=
			dev->config.mi_config.mp.curr_buff_addr) {
			if (dev->config.mi_config.mp.next_buff_addr ==
				dev->config.mi_config.null_buff_dma_addr) {
				BUG();
				/* disable MI MP */
				xgold_v4l2_debug(XGOLD_V4L2_ISR,
						 "\n %s: %s: disabling MP MI\n",
						 DRIVER_NAME, __func__);
#if defined(CONFIG_CIF_ISP_AUTO_UPD_CFG_BUG)
	/* 'switch off' MI interface */
	/* synchronous switching on/off of the MI(xx_enable)
	is currently not possible because of a bug in
	the CIF ISP hardware. This is a workaround
	which writes to 'dev/null' instead */
				cif_iowrite32(CIF_NULL_BUFF_SIZE,
					      (dev->config.base_addr) +
					      CIF_MI_MP_Y_SIZE_INIT);
				cif_iowrite32(CIF_NULL_BUFF_SIZE,
					      (dev->config.base_addr) +
					      CIF_MI_MP_CB_SIZE_INIT);
				cif_iowrite32(CIF_NULL_BUFF_SIZE,
					      (dev->config.base_addr) +
					      CIF_MI_MP_CR_SIZE_INIT);
#else
				cif_iowrite32AND(~CIF_MI_CTRL_MP_ENABLE,
						 (dev->config.base_addr) +
						 CIF_MI_CTRL);
#endif
			} else if (dev->config.mi_config.mp.curr_buff_addr ==
				   dev->config.mi_config.
				   null_buff_dma_addr) {
				/* re-enable MI MP */
				xgold_v4l2_debug(XGOLD_V4L2_ISR,
						 "\n %s: %s: enabling MP MI\n",
						 DRIVER_NAME, __func__);
#if defined(CONFIG_CIF_ISP_AUTO_UPD_CFG_BUG)
				cif_iowrite32(CIF_MI_MP_FRAME,
					      (dev->config.base_addr) +
					      CIF_MI_ICR);
				cif_iowrite32OR(CIF_MI_MP_FRAME |
						CIF_MI_AHB_ERROR,
						(dev->config.base_addr) +
						CIF_MI_IMSC);
				cif_iowrite32(dev->config.mi_config.mp.
					      y_size,
					      (dev->config.base_addr) +
					      CIF_MI_MP_Y_SIZE_INIT);
				cif_iowrite32(dev->config.mi_config.mp.
					      cb_size,
					      (dev->config.base_addr) +
					      CIF_MI_MP_CB_SIZE_INIT);
				cif_iowrite32(dev->config.mi_config.mp.
					      cr_size,
					      (dev->config.base_addr) +
					      CIF_MI_MP_CR_SIZE_INIT);
#else
				cif_iowrite32OR(CIF_MI_CTRL_MP_ENABLE,
						(dev->config.base_addr) +
						CIF_MI_CTRL);
#endif
			}
			dev->config.mi_config.mp.curr_buff_addr =
				dev->config.mi_config.mp.next_buff_addr;
			ret += marvin_lib_mi_address(dev, 0);
			cif_iowrite32OR(CIF_ISP_CTRL_ISP_GEN_CFG_UPD,
					(dev->config.base_addr) +
					CIF_ISP_CTRL);
		} else if (dev->config.mi_config.mp.next_buff_addr ==
			   dev->config.mi_config.null_buff_dma_addr) {
			cif_iowrite32AND(~CIF_MI_MP_FRAME,
					 (dev->config.base_addr) +
					 CIF_MI_IMSC);
		}
	}

	return ret;
}

static int update_mi_sp(struct cif_isp20_device *dev)
{
	int ret = 0;

	if (dev->config.mi_config.sp.next_buff_addr !=
		dev->config.mi_config.sp.curr_buff_addr) {
		if (dev->config.mi_config.sp.next_buff_addr ==
		    dev->config.mi_config.null_buff_dma_addr) {
			/* disable MI SP */
			BUG();
			xgold_v4l2_debug(XGOLD_V4L2_ISR,
					 "\n %s: %s: disabling SP MI\n",
					 DRIVER_NAME, __func__);
#if defined(CONFIG_CIF_ISP_AUTO_UPD_CFG_BUG)
			/* 'switch off' MI interface */
			/* synchronous switching on/off of the MI(xx_enable)
			is currently not possible because of a bug in
			the CIF ISP hardware. This is
			a workaround which writes to 'dev/null' instead */
			cif_iowrite32(CIF_NULL_BUFF_SIZE,
				      (dev->config.base_addr) +
				      CIF_MI_SP_Y_SIZE_INIT);
			cif_iowrite32(CIF_NULL_BUFF_SIZE,
				      (dev->config.base_addr) +
				      CIF_MI_SP_CB_SIZE_INIT);
			cif_iowrite32(CIF_NULL_BUFF_SIZE,
				      (dev->config.base_addr) +
				      CIF_MI_SP_CR_SIZE_INIT);
#else
			cif_iowrite32AND(~CIF_MI_CTRL_SP_ENABLE,
					 (dev->config.base_addr) +
					 CIF_MI_CTRL);
#endif
		} else if (dev->config.mi_config.sp.curr_buff_addr ==
			   dev->config.mi_config.null_buff_dma_addr) {
			/* re-enable MI SP */
			xgold_v4l2_debug(XGOLD_V4L2_ISR,
					 "\n %s: %s: enabling SP MI\n",
					 DRIVER_NAME, __func__);
#if defined(CONFIG_CIF_ISP_AUTO_UPD_CFG_BUG)
			cif_iowrite32(CIF_MI_SP_FRAME,
				      (dev->config.base_addr) +
				      CIF_MI_ICR);
			cif_iowrite32OR(CIF_MI_SP_FRAME | CIF_MI_AHB_ERROR,
					(dev->config.base_addr) +
					CIF_MI_IMSC);
			cif_iowrite32(dev->config.mi_config.sp.y_size,
				      (dev->config.base_addr) +
				      CIF_MI_SP_Y_SIZE_INIT);
			cif_iowrite32(dev->config.mi_config.sp.cb_size,
				      (dev->config.base_addr) +
				      CIF_MI_SP_CB_SIZE_INIT);
			cif_iowrite32(dev->config.mi_config.sp.cr_size,
				      (dev->config.base_addr) +
				      CIF_MI_SP_CR_SIZE_INIT);
#else
			cif_iowrite32OR(CIF_MI_CTRL_SP_ENABLE,
					(dev->config.base_addr) +
					CIF_MI_CTRL);
#endif
		}
		dev->config.mi_config.sp.curr_buff_addr =
			dev->config.mi_config.sp.next_buff_addr;
		ret += marvin_lib_mi_address(dev, 1);
		cif_iowrite32OR(CIF_ISP_CTRL_ISP_GEN_CFG_UPD,
				(dev->config.base_addr) + CIF_ISP_CTRL);
	} else
	    if ((dev->config.mi_config.sp.next_buff_addr ==
		 dev->config.mi_config.null_buff_dma_addr)
		&& (!dev->config.jpeg_config.enable)) {
		/* if JPEG encoding is enabled we are currently
		using the frame interrupt
		of the secondary path to poll the JPEG encoding status
		and determine when the JPEG encoding has been done */
		cif_iowrite32AND(~CIF_MI_SP_FRAME,
				 (dev->config.base_addr) + CIF_MI_IMSC);
	}
	return ret;
}

/* ======================================================================== */
int marvin_isp_isr(void *cntxt)
{
	struct cif_isp20_device *dev =
	    (struct cif_isp20_device *)cntxt;
	unsigned int isp_mis = 0;
	unsigned int isp_err = 0;

	xgold_v4l2_debug(XGOLD_V4L2_ISR,
			     "\n %s: %s:\n", DRIVER_NAME, __func__);

	isp_mis = cif_ioread32((dev->config.base_addr) + CIF_ISP_MIS);

#ifdef MEASURE_VERTICAL_BLANKING
	if (isp_mis & CIF_ISP_V_START) {
		pr_info("ISP_INT:VS\n");
		cif_iowrite32(CIF_ISP_V_START,
			      (dev->config.base_addr) + CIF_ISP_ICR);
	}
	if (isp_mis & CIF_ISP_FRAME_IN) {
		pr_info("ISP_INT:FI\n");
		cif_iowrite32(CIF_ISP_FRAME_IN,
			      (dev->config.base_addr) + CIF_ISP_ICR);
	}
#endif

	if ((isp_mis & (CIF_ISP_DATA_LOSS | CIF_ISP_PIC_SIZE_ERROR))) {
		dev->sp_stream.stall = true;
		dev->mp_stream.stall = true;
		if ((isp_mis & CIF_ISP_PIC_SIZE_ERROR)) {
			/* Clear pic_size_error */
			cif_iowrite32(CIF_ISP_PIC_SIZE_ERROR,
				(dev->config.base_addr) +
				CIF_ISP_ICR);
			marvin_hw_errors[isp_pic_size_err].count++;
			dev_err(dev->dev,
				"CIF_ISP_PIC_SIZE_ERROR");
			isp_err =
			    cif_ioread32((dev->config.base_addr) +
					 CIF_ISP_ERR);
			cif_iowrite32(isp_err,
				      (dev->config.base_addr) +
				      CIF_ISP_ERR_CLR);
		} else if ((isp_mis & CIF_ISP_DATA_LOSS)) {
			/* Clear data_loss */
			cif_iowrite32(CIF_ISP_DATA_LOSS,
				(dev->config.base_addr) +
				CIF_ISP_ICR);
			marvin_hw_errors[isp_data_loss].count++;
			dev_err(dev->dev,
				"CIF_ISP_DATA_LOSS\n");
			cif_iowrite32(CIF_ISP_DATA_LOSS,
				      (dev->config.base_addr) +
				      CIF_ISP_ICR);
		}

		/* Stop ISP */
		cif_iowrite32AND(~CIF_ISP_CTRL_ISP_INFORM_ENABLE &
				 ~CIF_ISP_CTRL_ISP_ENABLE,
				 (dev->config.base_addr) + CIF_ISP_CTRL);
		/*   isp_update */
		cif_iowrite32OR(CIF_ISP_CTRL_ISP_CFG_UPD,
				(dev->config.base_addr) + CIF_ISP_CTRL);
		marvin_hw_restart(dev);
	}

	cifisp_isp_isr(&dev->isp_dev, isp_mis);

	if (isp_mis & CIF_ISP_FRAME) {

		/* Clear Frame In (ISP) */
		cif_iowrite32(CIF_ISP_FRAME
			      | CIF_ISP_FRAME_IN
			      | CIF_ISP_V_START
			      | CIF_ISP_H_START,
			      (dev->config.base_addr) + CIF_ISP_ICR);

	}
	return 0;
}

/* ======================================================================== */

int marvin_s_ctrl(struct cif_isp20_device *dev, struct v4l2_control *vc)
{
	int ret = 0;

	xgold_v4l2_debug(XGOLD_V4L2_ENTER,
		"%s: %s: Enter...\n", DRIVER_NAME, __func__);

	switch (vc->id) {
		/* MEMORY INTERFACE FEATURES */
	case V4L2_CID_HFLIP:
		dev->config.control.val = vc->value;
		dev->config.control.id = marvin_hflip;
		ret = marvin_lib_s_control(dev);
		break;

	case V4L2_CID_VFLIP:
		dev->config.control.val = vc->value;
		dev->config.control.id = marvin_vflip;
		ret = marvin_lib_s_control(dev);
		break;
		/* END OF MEMORY INTERFACE FEATURES */

		/* IMAGE EFFECTS */
	case V4L2_CID_COLORFX:
		dev->config.control.val = 1;
	if (vc->value == V4L2_COLORFX_SEPIA) {
		dev->config.control.id = marvin_sepia;
		dev->config.ei_config.image_effect = marvin_sepia;
	} else if (vc->value == V4L2_COLORFX_BW) {
		dev->config.control.id = marvin_black_and_white;
		dev->config.ei_config.image_effect = marvin_black_and_white;
	} else if (vc->value == V4L2_COLORFX_NEGATIVE) {
		dev->config.control.id = marvin_negative;
		dev->config.ei_config.image_effect = marvin_negative;
	} else if (vc->value == V4L2_COLORFX_NONE) {
		dev->config.control.id = marvin_none_ie;
		dev->config.ei_config.image_effect = marvin_none_ie;
	} else if (vc->value == V4L2_COLORFX_EMBOSS) {
		dev->config.control.id = marvin_emboss;
		dev->config.ei_config.image_effect = marvin_emboss;
	} else if (vc->value == V4L2_COLORFX_SKETCH) {
		dev->config.control.id = marvin_sketch;
		dev->config.ei_config.image_effect = marvin_sketch;
	} else {
	    ret = -EINVAL;
	    break;
	}
	/* Color selection not implemented */
	ret = marvin_lib_s_control(dev);
		break;
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		dev->config.control.val = vc->value;
		dev->config.control.id = marvin_jpeg_quality;
		ret += marvin_lib_s_control(dev);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		if (vc->value == V4L2_FLASH_LED_MODE_NONE)
			dev->config.flash_mode = CIF_ISP20_FLASH_MODE_OFF;
		else if (vc->value == V4L2_FLASH_LED_MODE_FLASH)
			dev->config.flash_mode = CIF_ISP20_FLASH_MODE_FLASH;
		else if (vc->value == V4L2_FLASH_LED_MODE_TORCH)
			dev->config.flash_mode = CIF_ISP20_FLASH_MODE_TORCH;
		else
			ret = -EINVAL;
		if (dev->img_src_state == CIF_ISP20_IMG_SRC_STATE_STREAMING)
			cif_isp20_img_src_s_ctrl(dev->img_src,
				CIF_ISP20_CID_FLASH_MODE,
				dev->config.flash_mode);
		break;
	default:
		/* not supported !! */
		BUG();
		break;
	}

	return ret;
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
