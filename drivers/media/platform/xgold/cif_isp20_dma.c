
/*
 ****************************************************************
 *
 *  Intel CIF ISP 2.0 driver - DMA related functionality
 *
 *  Copyright (C) 2013-2014 Intel Mobile Communications GmbH
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

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-core.h>
#include <media/videobuf2-core.h>
#include <media/videobuf-dma-contig.h>

#include "cif_isp20_dma.h"
#include "cif_isp20.h"

static int rbpath_dbg_level = 0x0F;
static int rbpath_minor = -1;
static int rbpath_nr = -1;

#define CIF_RBPATH_DPRINT(level, fmt, arg...) { if (rbpath_dbg_level&level)\
	pr_debug(fmt, ##arg); }

/*
 * Fun name : rbpath_get_set_rb_mode
 *
 * Description : This function allows to get and
 * set read back path mode.
 * */

int rbpath_get_set_rb_mode(struct xgold_readback_path_dev *rb_dev, bool flag,
			   __s32 *value)
{
	pr_info("rbpath_get_set_rb_mode value = %d\n", *value);
	if (flag == _GET_) {
		*value = (int)rb_dev->rb_params.rb_mode;
		return 0;
	} else if (flag == _SET_) {
		rb_dev->rb_params.rb_mode = (enum cif_rb_mode) *value;
	} else {
		return -ENODEV;
	}

	return 0;
}

/*
 * Fun name : rbpath_get_set_rb_path
 *
 * Description : This function allows to get and set
 *               where the read back path data
 *               has to be fed to CIF HW.
 *
 * */
int rbpath_get_set_rb_path(struct xgold_readback_path_dev *rb_dev, bool flag,
			   __s32 *value)
{
	if (flag == _GET_) {
		*(enum cif_rb_path *) value = (int)rb_dev->rb_params.rb_path;
		return 0;
	} else if (flag == _SET_) {
		rb_dev->rb_params.rb_path = (enum cif_rb_path) *value;
	} else {
		return -ENODEV;
	}

	return 0;
}

/*
 * Fun name : rbpath_get_set_rb_data_src
 *
 * Description : This function allows to get and
 * set data source for read back path
 *
 * */
int rbpath_get_set_rb_data_src(struct xgold_readback_path_dev *rb_dev,
			       bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = (int)rb_dev->rb_params.rb_mode;
		return 0;
	} else if (flag == _SET_) {
		rb_dev->rb_params.rb_data_src = (enum cif_rb_data_src) *value;
	} else {
		return -ENODEV;
	}
	return 0;
}

/* =================================================== */
/*========== Read back path IOCTL implementation ===== */
/* =================================================== */

static int rbpath_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *p)
{
	struct videobuf_queue *q;
	struct video_device *vdev = video_devdata(file);
	struct xgold_readback_path_dev *rbpath_dev = video_get_drvdata(vdev);
	int ret = 0;

	q = &rbpath_dev->vbq_stat;

	if ((p == NULL) || (q == NULL)) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  "Null pointer p= %x, q = %x in %s\n",
				  (unsigned int)p, (unsigned int)q, __func__);
		return -ENODEV;
	}

	ret = videobuf_reqbufs(q, p);
	if (ret) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  " %s: in %s: Error when reqbufs Readback path buffers\n",
				  RB_PATH_DEV_NAME, __func__);
		return -ENODEV;
	}
	return ret;
}

static int rbpath_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct videobuf_queue *q;
	struct video_device *vdev = video_devdata(file);
	struct xgold_readback_path_dev *rbpath_dev = video_get_drvdata(vdev);
	int ret = 0;

	q = &rbpath_dev->vbq_stat;
	if ((p == NULL) || (q == NULL)) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  "Null pointer p= %x, q = %x in %s\n ",
				  (unsigned int)p, (unsigned int)q, __func__);
		return -ENODEV;
	}

	ret += videobuf_querybuf(q, p);
	if (ret) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  " %s: %s: Error when querying Readback path buffer\n",
				  RB_PATH_DEV_NAME, __func__);
		return -ENODEV;
	}
	return ret;
}

static int rbpath_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct videobuf_queue *q;
	struct video_device *vdev = video_devdata(file);
	struct xgold_readback_path_dev *rbpath_dev = video_get_drvdata(vdev);
	int ret = 0;

	q = &rbpath_dev->vbq_stat;
	CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR, "in rbpath_qbuf function");

	if ((p == NULL) || (q == NULL)) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  "Null pointer p= %x, q = %x in %s\n",
				  (unsigned int)p, (unsigned int)q, __func__);
		return -ENODEV;
	}

	ret += videobuf_qbuf(q, p);
	if (ret) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  " %s: in %s: Error when qbuf Readback path buffer\n",
				  RB_PATH_DEV_NAME, __func__);
		return -ENODEV;
	}

	return ret;
}

/* ================================================= */

static int rbpath_dqbuf(struct file *file, void *priv,
	struct v4l2_buffer *p)
{
	struct videobuf_queue *q;
	struct video_device *vdev = video_devdata(file);
	struct xgold_readback_path_dev *rbpath_dev = video_get_drvdata(vdev);
	int ret = 0;

	CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR, "in rbpath_qbuf function");

	q = &rbpath_dev->vbq_stat;
	if ((p == NULL) || (q == NULL)) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  "Null pointer p= %x, q = %x in %s\n ",
				  (unsigned int)p, (unsigned int)q, __func__);
		return -ENODEV;
	}

	ret += videobuf_dqbuf(q, p, file->f_flags & O_NONBLOCK);
	if (ret) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  " %s: in %s: Error when dqbuf Main path buffer\n",
				  RB_PATH_DEV_NAME, __func__);
		return -ENODEV;
	}
	return ret;
}

static int rbpath_addr_update(struct xgold_readback_path_dev *rbpath_dev)
{
	unsigned int rb_dma_add = 0;
	struct list_head *ptr = NULL;
	struct videobuf_buffer *vb_queue = NULL, *vb = NULL;
	list_for_each(ptr, &rbpath_dev->stat) {
		vb = list_entry(ptr, struct videobuf_buffer, queue);
		if ((vb->state == VIDEOBUF_QUEUED) && !vb_queue)
			vb_queue = vb;
	}
	if (vb_queue) {
		rb_dma_add = videobuf_to_dma_contig(vb_queue);
		vb_queue->state = VIDEOBUF_ACTIVE;
	} else
		pr_info("nothing to queue for RB\n");

	cif_iowrite32(rb_dma_add,
		      (rbpath_dev->base_addr) + CIF_MI_DMA_Y_PIC_START_AD);
	cif_iowrite32(rbpath_dev->rb_params.rb_image_size.width *
		      rbpath_dev->rb_params.rb_image_size.height,
		      (rbpath_dev->base_addr) + CIF_MI_DMA_Y_PIC_SIZE);
	cif_iowrite32(rbpath_dev->rb_params.rb_image_size.width,
		      (rbpath_dev->base_addr) + CIF_MI_DMA_Y_LLENGTH);
	cif_iowrite32(rbpath_dev->rb_params.rb_image_size.width,
		      (rbpath_dev->base_addr) + CIF_MI_DMA_Y_PIC_WIDTH);
	cif_iowrite32(rb_dma_add +
		      rbpath_dev->rb_params.rb_image_size.width *
		      rbpath_dev->rb_params.rb_image_size.height,
		      (rbpath_dev->base_addr) + CIF_MI_DMA_CB_PIC_START_AD);
	cif_iowrite32(rb_dma_add +
		      (rbpath_dev->rb_params.rb_image_size.width *
		       rbpath_dev->rb_params.rb_image_size.height) / 2,
		      (rbpath_dev->base_addr) + CIF_MI_DMA_CR_PIC_START_AD);
	return 0;
}

static int rbpath_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct videobuf_queue *q;
	struct video_device *vdev = video_devdata(file);
	struct xgold_readback_path_dev *rbpath_dev = video_get_drvdata(vdev);
	int ret = 0;

	q = &rbpath_dev->vbq_stat;
	if (q == NULL) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  "Null pointer q = %x in %s\n ",
				  (unsigned int)q, __func__);
		return -ENODEV;
	}
	ret = videobuf_streamon(q);
	if (ret) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  " %s: in %s: Error when dqbuf Main path buffer\n",
				  RB_PATH_DEV_NAME, __func__);
		return -ENODEV;
	}

	return ret;
}

/* =========================================================== */
static int rbpath_streamoff(struct file *file, void *priv,
	enum v4l2_buf_type i)
{
	struct videobuf_queue *q;
	struct video_device *vdev = video_devdata(file);
	struct xgold_readback_path_dev *rbpath_dev = video_get_drvdata(vdev);
	int ret = 0;

	q = &rbpath_dev->vbq_stat;
	if (q == NULL) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  "Null pointer q = %x in %s\n ",
				  (unsigned int)q, __func__);
		return -ENODEV;
	}

	videobuf_mmap_free(q);
	ret = videobuf_streamoff(q);
	if (ret) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  " %s: in %s: Error when dqbuf Main path buffer\n",
				  RB_PATH_DEV_NAME, __func__);
		return -ENODEV;
	}
	rbpath_dev->rb_params.rb_path = CAMERA_RB_PATH_DISABLE;
	return ret;
}

/*
 * Fun name : rbpath_g_ctrl
 *
 * Description : This is a IOCTL function,
 * allows to get read back path control parameters.
 *
 * */

int rbpath_g_ctrl(struct file *file, void *priv, struct v4l2_control *vc)
{
	struct video_device *vdev = video_devdata(file);
	struct xgold_readback_path_dev *rbpath_dev = video_get_drvdata(vdev);
	int ret = -EINVAL;

	if (vc == NULL)
		return -EINVAL;

	mutex_lock(&rbpath_dev->mutex);
	switch (vc->id) {
	case V4L2_CID_CIF_RB_MODE:
		ret = rbpath_get_set_rb_mode(rbpath_dev, _GET_, &vc->value);
		break;

	case V4L2_CID_CIF_RB_PATH:
		ret = rbpath_get_set_rb_path(rbpath_dev, _GET_, &vc->value);
		break;

	case V4L2_CID_CIF_RB_DATA_SRC:
		ret = rbpath_get_set_rb_data_src(rbpath_dev, _GET_, &vc->value);
		break;

	default:
		mutex_unlock(&rbpath_dev->mutex);
		return -EINVAL;
	}
	mutex_unlock(&rbpath_dev->mutex);
	return ret;
}

/*
 * Fun name : rbpath_s_ctrl
 *
 * Description : This is a IOCTL function,
 * allows to set read back path control parameters.
 * */
int rbpath_s_ctrl(struct file *file, void *priv, struct v4l2_control *vc)
{
	struct video_device *vdev = video_devdata(file);
	struct xgold_readback_path_dev *rbpath_dev = video_get_drvdata(vdev);

	int ret = -EINVAL;

	if (vc == NULL)
		return -EINVAL;

	pr_info("rbpath_s_ctrl vc-id = %x vc-value = %x\n", vc->id, vc->value);

	mutex_lock(&rbpath_dev->mutex);
	switch (vc->id) {
	case V4L2_CID_CIF_RB_MODE:
		ret = rbpath_get_set_rb_mode(rbpath_dev, _SET_, &vc->value);
		break;
	case V4L2_CID_CIF_RB_PATH:
		ret = rbpath_get_set_rb_path(rbpath_dev, _SET_, &vc->value);
		break;
	case V4L2_CID_CIF_RB_DATA_SRC:
		ret = rbpath_get_set_rb_data_src(rbpath_dev, _SET_, &vc->value);
		break;
	default:
		mutex_unlock(&rbpath_dev->mutex);
		return -EINVAL;
	}
	mutex_unlock(&rbpath_dev->mutex);
	return ret;
}

static long rbpath_ioctl_default(struct file *file, void *fh,
				 bool valid_prio, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct xgold_readback_path_dev *rb_dev = video_get_drvdata(vdev);

	mutex_lock(&rb_dev->mutex);
	switch (cmd) {
	default:
		mutex_unlock(&rb_dev->mutex);
		return -EINVAL;
	}
	mutex_unlock(&rb_dev->mutex);
	return 0;
}

static int rbpath_s_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct xgold_readback_path_dev *rb_dev = video_get_drvdata(vdev);
	if (NULL == f)
		return -EINVAL;

	rb_dev->rb_params.rb_img_format.pixelformat = f->fmt.pix.pixelformat;
	rb_dev->rb_params.rb_img_format.width = f->fmt.pix.width;
	rb_dev->rb_params.rb_img_format.height = f->fmt.pix.height;
	rb_dev->rb_params.rb_image_size.width = f->fmt.pix.width;
	rb_dev->rb_params.rb_image_size.height = f->fmt.pix.height;
	return 0;
}

static int rbpath_try_fmt_cap(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	return 0;
}

static int rbpath_g_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	return 0;
}

static int rbpath_enum_fmt_cap(struct file *file, void *fh,
			       struct v4l2_fmtdesc *f)
{
	return 0;
}

int rb_configure(struct xgold_readback_path_dev *rbpath_dev)
{
	struct xgold_camera_readback_params *p_rb_param;
	unsigned int mi_dma_ctrl = 0, tmp_reg_val = 0;

	if (NULL == rbpath_dev)
		return -EINVAL;

	p_rb_param = &rbpath_dev->rb_params;
	if (CAMERA_RB_PATH_DISABLE == p_rb_param->rb_path) {
		/* Clear the memory DMA interrupt */
		cif_iowrite32OR(CIF_MI_DMA_READY, ((rbpath_dev->base_addr) +
				CIF_MI_ICR));
		/* Disable the memory DMA interrupt */
		cif_iowrite32AND(~CIF_MI_DMA_READY, (rbpath_dev->base_addr) +
				CIF_MI_IMSC);
		/* Disable RB DMA */
		cif_iowrite32(0, (rbpath_dev->base_addr) + CIF_MI_DMA_START);
	} else {
		/* Configure the DMA read format here */
		switch (p_rb_param->rb_img_format.pixelformat) {
		case V4L2_PIX_FMT_YUV420M:	/* 420 planar */
		case V4L2_PIX_FMT_YUV422P:	/* 420 planar */
			mi_dma_ctrl |= CIF_MI_DMA_CTRL_READ_FMT_PLANAR;
			break;

		case V4L2_PIX_FMT_NV12:	/* 420 Semiplanar */
		case V4L2_PIX_FMT_NV21:	/* 420 Semiplanar */
		case V4L2_PIX_FMT_NV12M:/* 420 Semiplanar */
		case V4L2_PIX_FMT_NV12MT:/* 420 Semiplanar */
		case V4L2_PIX_FMT_NV16:	/* 422 Semiplanar */
		case V4L2_PIX_FMT_NV61:	/* 422 Semiplanar */
			/* Enable the DMA read format bits */
			mi_dma_ctrl |= CIF_MI_DMA_CTRL_READ_FMT_SPLANAR;
			break;

		case V4L2_PIX_FMT_YUYV:	/* 422 interleaved */
		case V4L2_PIX_FMT_UYVY:	/* 422 interleaved */
		case V4L2_PIX_FMT_YYUV:	/* 422 interleaved */
		case V4L2_PIX_FMT_YVYU:	/* 422 interleaved */
		case V4L2_PIX_FMT_VYUY:	/* 422 interleaved */
		case V4L2_PIX_FMT_YUV444:	/*  444 interleaved */
		/* RAW-BAYER-10Bits, format 2 is applicable to this. */
		case V4L2_PIX_FMT_SGRBG10:
		/* RAW-BAYER-10Bits format 2 is applicable to this. */
		case V4L2_PIX_FMT_SGRBG8:
			/* Enable the DMA read format bits */
			mi_dma_ctrl |= CIF_MI_DMA_CTRL_READ_FMT_PACKED;
			break;
		default:
			return -EINVAL;

		}
		/* Configure DMA inout format */
		switch (p_rb_param->rb_img_format.pixelformat) {
			/* Did not find any YUV 400 color format in V4L2,
			so did not add  CAMERA_RB_DMA_INOUT_FORMAT_0 */
		case V4L2_PIX_FMT_YUV420M:	/* 420 planar */
		case V4L2_PIX_FMT_NV12:	/* 420 Semiplanar */
		case V4L2_PIX_FMT_NV21:	/* 420 Semiplanar */
		case V4L2_PIX_FMT_NV12M:	/* 420 Semiplanar */
		case V4L2_PIX_FMT_NV12MT:	/* 420 Semiplanar */
			/* Enable the DMA inout format bits */
			mi_dma_ctrl |= CIF_MI_DMA_CTRL_FMT_YUV420;
			break;

		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUYV:	/* 422 interleaved */
		case V4L2_PIX_FMT_UYVY:	/* 422 interleaved */
		case V4L2_PIX_FMT_YYUV:	/* 422 interleaved */
		case V4L2_PIX_FMT_YVYU:	/* 422 interleaved */
		case V4L2_PIX_FMT_VYUY:	/* 422 interleaved */

		case V4L2_PIX_FMT_NV16:	/* 422 Semiplanar */
		case V4L2_PIX_FMT_NV61:	/* 422 Semiplanar */

		/* RAW-BAYER-10Bits, format for 422 is applicable here */
		case V4L2_PIX_FMT_SGRBG10:
		/* RAW-BAYER-10Bits format for 422 is applicable here */
		case V4L2_PIX_FMT_SGRBG8:
			/* Enable the DMA inout format bits */
			mi_dma_ctrl |= CIF_MI_DMA_CTRL_FMT_YUV422;
			break;

		case V4L2_PIX_FMT_YUV444:	/*  444 interleaved */
			/* Enable the DMA inpout format bits */
			mi_dma_ctrl |= CIF_MI_DMA_CTRL_FMT_YUV444;
			break;

		}
		/* Configure DMA Lunimance burst length */
		mi_dma_ctrl |= CIF_MI_DMA_CTRL_BURST_LEN_LUM_64;
		/* Configure DMA Chrominance burst length */
		mi_dma_ctrl |= CIF_MI_DMA_CTRL_BURST_LEN_CHROM_64;
		/* Configure bytes swap */
		mi_dma_ctrl |= CIF_MI_DMA_CTRL_NO_BYTE_SWAP;

		/* Configure DMA mode(Single/continous) */
		if (p_rb_param->rb_mode == CAMERA_RB_MODE_SINGLE)
			mi_dma_ctrl |= CIF_MI_DMA_CTRL_CONTINOUS_DIS;
		else
			mi_dma_ctrl |= CIF_MI_DMA_CTRL_CONTINOUS_ENA;

		/* All configuration in MI_DMA_CTRL is ready, write to HW */
		cif_iowrite32OR(mi_dma_ctrl,
				(rbpath_dev->base_addr) + CIF_MI_DMA_CTRL);

		/* Program the DMA address and size information */
		pr_info("\n CIF_MI_DMA_DMA_CTRL = 0x%x",
		       cif_ioread32((rbpath_dev->base_addr) +
				    CIF_MI_DMA_CTRL));
		/*update the DMA address */
		rbpath_addr_update(rbpath_dev);
		/* Enable Read Back path DMA */
		tmp_reg_val =
		    cif_ioread32((rbpath_dev->base_addr) + CIF_MI_ICR);
		tmp_reg_val = tmp_reg_val | CIF_MI_DMA_READY;
		/* Clear the RB DMA interrupt */
		cif_iowrite32(tmp_reg_val,
			((rbpath_dev->base_addr) + CIF_MI_ICR));

		tmp_reg_val =
		    cif_ioread32((rbpath_dev->base_addr) + CIF_MI_IMSC);
		tmp_reg_val = tmp_reg_val | CIF_MI_DMA_READY;
		/* Enable the RB DMA interrupt */
		cif_iowrite32(tmp_reg_val,
			(rbpath_dev->base_addr) + CIF_MI_IMSC);

		tmp_reg_val =
		    cif_ioread32((rbpath_dev->base_addr) + CIF_VI_DPCL);
		switch (p_rb_param->rb_path) {
		case CAMERA_RB_PATH_SP:
			tmp_reg_val = tmp_reg_val | CIF_VI_DPCL_DMA_SW_SPMUX;
			break;
		case CAMERA_RB_PATH_SI:
			tmp_reg_val = tmp_reg_val | CIF_VI_DPCL_DMA_SW_SI;
			break;
		case CAMERA_RB_PATH_IE:
			tmp_reg_val = tmp_reg_val | CIF_VI_DPCL_DMA_SW_IE;
			break;
		case CAMERA_RB_PATH_ISP:
			tmp_reg_val = tmp_reg_val | CIF_VI_DPCL_DMA_SW_ISP;
			break;
		default:
			return -EINVAL;
		}

		/* Write DMA path selector */
		cif_iowrite32(tmp_reg_val,
			(rbpath_dev->base_addr) + CIF_VI_DPCL);

	}
	return 0;
}

/* ================================QUEUE OPS ============= */
static int cif_rb_vbq_setup(struct videobuf_queue *vq,
			    unsigned int *cnt, unsigned int *size)
{
	struct xgold_readback_path_dev *rb_dev = vq->priv_data;
	if (*cnt <= 0 || *cnt > CIFISP_MAX_RB_BUFFER)
		*cnt = CIFISP_MAX_RB_BUFFER;/* Supply a default num of buffers*/

	pr_info("cif_rb_vbq_setup cnt = %d", *cnt);
	*size =
	    rb_dev->rb_params.rb_image_size.width *
	    rb_dev->rb_params.rb_image_size.height * 2;

	return 0;
}

static void cif_rb_vbq_release(struct videobuf_queue *vq,
			       struct videobuf_buffer *vb)
{
	videobuf_waiton(vq, vb, 0, 0);
	videobuf_dma_contig_free(vq, vb);

}

static int cif_rb_vbq_prepare(struct videobuf_queue *vq,
			      struct videobuf_buffer *vb, enum v4l2_field field)
{
	int err = 0;
	struct xgold_readback_path_dev *rb_dev = vq->priv_data;

	vb->size =
	    rb_dev->rb_params.rb_image_size.width *
	    rb_dev->rb_params.rb_image_size.height * 2;
	vb->width = rb_dev->rb_params.rb_image_size.width;
	vb->height = rb_dev->rb_params.rb_image_size.height;

	if (vb->state == VIDEOBUF_NEEDS_INIT)
		err = videobuf_iolock(vq, vb, NULL);

	if (!err)
		vb->state = VIDEOBUF_PREPARED;
	else
		cif_rb_vbq_release(vq, vb);

	return err;
}

static void cif_rb_vbq_queue(struct videobuf_queue *vq,
			     struct videobuf_buffer *vb)
{
	struct xgold_readback_path_dev *rb_dev = vq->priv_data;

	vb->state = VIDEOBUF_QUEUED;

	list_add_tail(&vb->queue, &rb_dev->stat);
}

static struct videobuf_queue_ops rbpath_queue_ops = {
	.buf_setup = cif_rb_vbq_setup,
	.buf_prepare = cif_rb_vbq_prepare,
	.buf_queue = cif_rb_vbq_queue,
	.buf_release = cif_rb_vbq_release,
};

/* Main path video device IOCTLs*/
static const struct v4l2_ioctl_ops rbpath_ioctl = {
	.vidioc_reqbufs = rbpath_reqbufs,
	.vidioc_querybuf = rbpath_querybuf,
	.vidioc_qbuf = rbpath_qbuf,
	.vidioc_dqbuf = rbpath_dqbuf,
	.vidioc_streamon = rbpath_streamon,
	.vidioc_streamoff = rbpath_streamoff,
	.vidioc_g_ctrl = rbpath_g_ctrl,
	.vidioc_s_ctrl = rbpath_s_ctrl,
	.vidioc_default = rbpath_ioctl_default,
	.vidioc_enum_fmt_vid_cap = rbpath_enum_fmt_cap,
	.vidioc_g_fmt_vid_cap = rbpath_g_fmt_cap,
	.vidioc_s_fmt_vid_cap = rbpath_s_fmt_cap,
	.vidioc_try_fmt_vid_cap = rbpath_try_fmt_cap,

};

void rbpath_init_statq(struct xgold_readback_path_dev *rbpath_dev)
{
	/*This function may be called multiple times from Open.
	Hence the check*/
	if (rbpath_dev->is_queue_init)
		return;

	/* Init lock */
	spin_lock_init(&rbpath_dev->irq_lock);
	INIT_LIST_HEAD(&rbpath_dev->stat);

	videobuf_queue_dma_contig_init(
		&rbpath_dev->vbq_stat,
		&rbpath_queue_ops,
		NULL,
		&rbpath_dev->irq_lock,
		V4L2_BUF_TYPE_VIDEO_CAPTURE,
		V4L2_FIELD_NONE,
		sizeof(struct videobuf_buffer),
		rbpath_dev,
		NULL);	/* ext_lock: NULL */

	if (!rbpath_dev->is_queue_init)
		rbpath_dev->is_queue_init = true;
}

/******************************ISP video device Fops *********************/
static int rbpath_open(struct file *file)
{
	struct video_device *vdev;
	struct xgold_readback_path_dev *rbpath_dev;

	unsigned int ret = 0;
	int minor = video_devdata(file)->minor;
	if (file != NULL) {
		vdev = video_devdata(file);
		rbpath_dev = video_get_drvdata(vdev);
		minor = video_devdata(file)->minor;
	} else {
		return -ENODEV;
	}
	if (minor == rbpath_minor) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_INFO,
				  " %s: %s: open video minor=%d\n",
				  RB_PATH_DEV_NAME, __func__, minor);
		rbpath_init_statq(rbpath_dev);
	} else {
		ret = -ENODEV;
	}
	return ret;
}

/* ============================================================ */
static int rbpath_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *vdev;
	struct xgold_readback_path_dev *rbpath_dev;
	struct videobuf_queue *q;
	struct videobuf_buffer *vb;
	static int stat_index;
	int ret = 0;
	if (file != NULL) {
		vdev = video_devdata(file);
		if (vdev == NULL)
			return -EINVAL;
		rbpath_dev = video_get_drvdata(vdev);
		if (rbpath_dev == NULL)
			return -EINVAL;
		q = &(rbpath_dev->vbq_stat);
		if (q == NULL)
			return -EINVAL;
	} else {
		return -ENODEV;
	}
	ret = videobuf_mmap_mapper(q, vma);
	vb = q->bufs[stat_index];
	rbpath_dev->rb_vaddr[stat_index++] = (void *)videobuf_to_dma_contig(vb);
	if (stat_index == CIFISP_MAX_RB_BUFFER)
		stat_index = 0;
	return ret;
}

/* ============================================================ */

static int rbpath_release(struct file *file)
{
	return 0;
}

/* ============================================================ */

static unsigned int rbpath_poll(struct file *file,
				struct poll_table_struct *wait)
{
	struct video_device *vdev;
	struct xgold_readback_path_dev *rbpath_dev = NULL;
	struct videobuf_queue *vq = NULL;
	int ret;
	struct list_head *ptr = NULL;
	struct videobuf_buffer *vb_active = NULL, *vb = NULL;

	CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
			  " %s: %s: Polling on stat buffer!!\n",
			  RB_PATH_DEV_NAME, __func__);

	if (!file || !wait) {
		CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_ERROR,
				  "File pointer = %x, Wait = %x\n",
				  (unsigned int)file, (unsigned int)wait);
	}
	if (file != NULL) {
		vdev = video_devdata(file);
		if (vdev != NULL)
			rbpath_dev = video_get_drvdata(vdev);
		else
			return -EINVAL;
		if (rbpath_dev == NULL)
			return -EINVAL;
	} else
		return -EINVAL;

	list_for_each(ptr, &rbpath_dev->stat) {
		vb = list_entry(ptr, struct videobuf_buffer, queue);
		if (vb->state == VIDEOBUF_ACTIVE)
			vb_active = vb;
	}
	if (vb_active) {
		do_gettimeofday(&vb_active->ts);
		vb_active->state = VIDEOBUF_DONE;
		list_del(&vb_active->queue);
		/*Inform the wait queue */
		wake_up(&vb_active->done);
	}

	vq = &rbpath_dev->vbq_stat;
	ret = videobuf_poll_stream(file, vq, wait);

	return ret;
}

struct v4l2_file_operations rbpath_fops = {
	.open = rbpath_open,
	.mmap = rbpath_mmap,
	.ioctl = video_ioctl2,
	.release = rbpath_release,
	.poll = rbpath_poll,
};

/********************************************************************/
int register_rbpath_device(struct xgold_readback_path_dev *rbpath_dev,
	void __iomem *cif_reg_baseaddress)
{
	int ret = 0;
	struct video_device *vdev_cifrbpath = NULL;
	struct cif_isp20_device *xgold_v4l2 =
		container_of(rbpath_dev, struct cif_isp20_device, rb_dev);


	/* Assign the register base address,
	which we have already obtained from probe. */
	rbpath_dev->base_addr = cif_reg_baseaddress;
	BUG_ON(!(rbpath_dev->base_addr));
	rbpath_dev->rb_params.rb_path = CAMERA_RB_PATH_DISABLE;
	vdev_cifrbpath = rbpath_dev->vdev_cifrbpath;
	vdev_cifrbpath = video_device_alloc();
	if (!vdev_cifrbpath) {
		dev_err(&(vdev_cifrbpath->dev),
			"could not allocate video device for main path\n");
		ret = -ENOMEM;
		goto err;
	}

	vdev_cifrbpath->release = video_device_release;
	strlcpy(vdev_cifrbpath->name, RB_PATH_DEV_NAME,
		sizeof(vdev_cifrbpath->name));
	vdev_cifrbpath->vfl_type = V4L2_CAP_VIDEO_CAPTURE;
	vdev_cifrbpath->fops = &rbpath_fops;
	video_set_drvdata(vdev_cifrbpath, rbpath_dev);
	vdev_cifrbpath->minor = -1;
	vdev_cifrbpath->ioctl_ops = &rbpath_ioctl;
	vdev_cifrbpath->v4l2_dev = &xgold_v4l2->v4l2_dev;

	if (video_register_device(vdev_cifrbpath, VFL_TYPE_GRABBER, rbpath_nr) <
	    0) {
		dev_err(&(vdev_cifrbpath->dev),
			"could not register main path video note\n");
		ret = -ENODEV;
		goto err;
	}

	CIF_RBPATH_DPRINT(CIF_RBPATH_DEBUG_INFO,
			  " %s: %s: CIFISP vdev minor =  %d\n",
			  RB_PATH_DEV_NAME, __func__, vdev_cifrbpath->minor);

	if (rbpath_minor == -1)
		rbpath_minor = vdev_cifrbpath->minor;

	mutex_init(&rbpath_dev->mutex);

	return 0;

err:
	/*kfree(NULL) is safe, so no check is required */
	kfree(vdev_cifrbpath);
	return ret;
}
