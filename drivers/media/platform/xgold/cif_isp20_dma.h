/*
 ****************************************************************
 *
 *  Intel CIF ISP 2.0 driver - DMA related functionality
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
 ****************************************************************
 */

#ifndef _CIF_ISP20_DMA_H
#define _CIF_ISP20_DMA_H

/* Linux header files */
#include <linux/types.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <linux/videodev2.h>
#include <media/videobuf-core.h>
#include <media/videobuf-vmalloc.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/mm.h>
#include <linux/irq.h>

/* Xgold header files */
#include "cif_isp20_regs.h"

#define _GET_ 0
#define _SET_ 1

#define RB_PATH_DEV_NAME "READBACK_PATH_ISP"

/* Readback path debug Level */
#define CIF_RBPATH_DEBUG_ISR	(1<<5)
#define CIF_RBPATH_DEBUG_QUEUE	(1<<4)
#define CIF_RBPATH_DEBUG_FMT	(1<<3)
#define CIF_RBPATH_DEBUG_ENTER	(1<<2)
#define CIF_RBPATH_DEBUG_INFO	(1<<1)
#define CIF_RBPATH_DEBUG_ERROR	(1<<0)

#define CIFISP_MAX_RB_BUFFER 4

/****************************************************************************
*  Read back path datas structs
****************************************************************************/
#define V4L2_CID_CIF_RB_MODE (V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_CIF_RB_PATH (V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_CIF_RB_DATA_SRC (V4L2_CID_PRIVATE_BASE + 2)

/*Borrowed from XGOLD, will have to replace with RB specific ones.*/
#define CIF_RBPATH_DEBUG_ISR        (1<<5)
#define CIF_RBPATH_DEBUG_QUEUE  (1<<4)
#define CIF_RBPATH_DEBUG_FMT        (1<<3)
#define CIF_RBPATH_DEBUG_ENTER  (1<<2)
#define CIF_RBPATH_DEBUG_INFO       (1<<1)
#define CIF_RBPATH_DEBUG_ERROR  (1<<0)

/** Enumeration defining the various modes supported for Read back mode*/
enum cif_rb_mode {
	/* Image is read once from the system memory */
	CAMERA_RB_MODE_SINGLE = 0,
	/* Image is read continuously from the system memory */
	CAMERA_RB_MODE_CONTINIOUS,
};

/** Enumerations to select various data paths for read back data */
enum cif_rb_path {
	/* Disable self path */
	CAMERA_RB_PATH_DISABLE = 0,
	/*  From system memory through Self path resizer   */
	CAMERA_RB_PATH_SP,
	/*  From system memory through Super Impose block  */
	CAMERA_RB_PATH_SI,
	/*  From system memory through Image Effects block */
	CAMERA_RB_PATH_IE,
	/*  Raw data from system memory through ISP */
	CAMERA_RB_PATH_ISP
};

/** Enumeration defining the various data sources supported with CIF */
enum cif_rb_data_src {
	/* Data source is camera sensor */
	CAMERA_RB_DATA_SRC_SENSOR = 0,
	/* Data source is system memory */
	CAMERA_RB_DATA_SRC_RB,
	/* Both source are active, this used to enable super-impose */
	CAMERA_RB_DATA_SRC_BOTH,
};

/* Structure defining image size (in width and height) */
struct cif_rb_img_size {
	unsigned int width;	/* Width of the image in pixels */
	unsigned int height;	/* Height of the image in pixels */
};

/* Structure to hold readback parameters */
struct xgold_camera_readback_params {
	/* Color format of the image in memory */
	struct v4l2_pix_format rb_img_format;
	/* Size of the image in system buffer */
	struct cif_rb_img_size rb_image_size;
	/* Buffer pointer to fetch the image data from system memory */
	unsigned char *p_rb_buffer;
	/* Readback data source */
	enum cif_rb_data_src rb_data_src;
	/* Specifies read back mode */
	enum cif_rb_mode rb_mode;
	/* This specifies where readback data is fed to CIF */
	enum cif_rb_path rb_path;
};

struct xgold_readback_path_dev {

	struct video_device *vdev_cifrbpath;
	struct videobuf_queue vbq_stat;
	struct list_head stat;
	bool is_queue_init;
	spinlock_t irq_lock;
	void *rb_vaddr[CIFISP_MAX_RB_BUFFER];

	/* Purpose of mutex is to protect and
	serialize use of selfpath data structure and CIF API calls. */
	struct mutex mutex;

	/* Read back path parameters */
	struct xgold_camera_readback_params rb_params;
	void __iomem *base_addr;    /* registers base address */
};

/* Function definitions ***********************************/
/**********************************************************/
#endif /* end of _CIF_READBACK_PATH_H */
