/*
 ****************************************************************
 *
 *  Intel CIF ISP 2.0 driver - V4L2 compliant interface
 *
 *  Copyright (C) 2014 Intel Mobile GmbH
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
 *     07/07/2014: initial version.
 *
 ****************************************************************
 */

#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-controls_intel.h>
#include <media/videobuf-dma-contig.h>
#include "cif_isp20.h"
#include <linux/module.h>

#define CIIF_ISP20_V4L2_SP_DEV_MAJOR 0
#define CIIF_ISP20_V4L2_ISP_DEV_MAJOR 1
#define CIIF_ISP20_V4L2_MP_DEV_MAJOR 2
#define CIIF_ISP20_V4L2_DMA_DEV_MAJOR 3

#define SP_DEV 0
#define MP_DEV 1
#define DMA_DEV 2
#define ISP_DEV 3

enum isp_camera_port {
	ISP_CAMERA_PORT_SECONDARY,
	ISP_CAMERA_PORT_PRIMARY,
	ISP_CAMERA_PORT_TERTIARY,
	ISP_CAMERA_NR_PORTS
};

/* One structure per open file handle */
struct cif_isp20_v4l2_fh {
	enum cif_isp20_stream_id stream_id;
	struct v4l2_fh fh;
};

/* One structure per video node */
struct cif_isp20_v4l2_node {
	struct videobuf_queue buf_queue;
	struct video_device vdev;
	int users;
	struct cif_isp20_v4l2_fh *owner;
};

/* One structure per device */
struct cif_isp20_v4l2_device {
	struct cif_isp20_device cif_isp20_dev;
	struct cif_isp20_v4l2_node node[4];
};

/* TODO: make this a dynamically allocated variable */
static struct cif_isp20_v4l2_device cif_isp20_v4l2_dev;

static struct cif_isp20_v4l2_fh *to_fh(struct file *file)
{
	if (file == NULL || file->private_data == NULL)
		return NULL;

	return container_of(file->private_data, struct cif_isp20_v4l2_fh, fh);
}

static struct cif_isp20_v4l2_node *to_node(struct cif_isp20_v4l2_fh *fh)
{
	struct video_device *vdev = fh ? fh->fh.vdev : NULL;

	if (fh == NULL || vdev == NULL)
		return NULL;

	return container_of(vdev, struct cif_isp20_v4l2_node, vdev);
}

static struct videobuf_queue *to_videobuf_queue(
	struct file *file)
{
	struct cif_isp20_v4l2_fh *fh = to_fh(file);
	struct video_device *vdev = fh ? fh->fh.vdev : NULL;
	struct cif_isp20_v4l2_node *node = to_node(fh);
	struct videobuf_queue *q;

	if (unlikely(!node)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"node is NULL\n");
		BUG();
	}

	if (unlikely(vdev == NULL)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"vdev is NULL\n");
		BUG();
	}
	q = &node->buf_queue;
	if (unlikely(NULL == q)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"buffer queue is NULL\n");
		BUG();
	}

	return q;
}

static enum cif_isp20_stream_id to_stream_id(
	struct file *file)
{
	struct cif_isp20_v4l2_fh *fh;

	if (unlikely(NULL == file)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"NULL file handle\n");
		BUG();
	}
	fh = to_fh(file);
	if (unlikely(NULL == fh)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"fh is NULL\n");
		BUG();
	}

	return fh->stream_id;
}

static struct cif_isp20_device *to_cif_isp20_device(
	struct videobuf_queue *queue)
{
	return queue->priv_data;
}

static enum cif_isp20_stream_id to_cif_isp20_stream_id(
	enum v4l2_buf_type buf_type)
{
	if (buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return CIF_ISP20_STREAM_MP;
	else if (buf_type == V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return CIF_ISP20_STREAM_SP;
	else if (buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return CIF_ISP20_STREAM_DMA;
	else {
		cif_isp20_pltfrm_pr_err(NULL,
			"unsupported/unknown buffer type %d\n", buf_type);
		return -EINVAL;
	}
}

static const char *cif_isp20_v4l2_buf_type_string(
	enum v4l2_buf_type buf_type)
{
	switch (buf_type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return "VIDEO_CAPTURE";
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		return "VIDEO_OVERLAY";
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return "VIDEO_OUTPUT";
	default:
		break;
	}
	return "UNKNOWN/UNSUPPORTED";
}

const char *cif_isp20_v4l2_pix_fmt_string(
	int pix_fmt)
{
	switch (pix_fmt) {
	case V4L2_PIX_FMT_RGB332:
		return "V4L2-RGB332";
	case V4L2_PIX_FMT_RGB555:
		return "V4L2-RGB555";
	case V4L2_PIX_FMT_RGB565:
		return "V4L2-RGB565";
	case V4L2_PIX_FMT_RGB555X:
		return "V4L2-RGB555X";
	case V4L2_PIX_FMT_RGB565X:
		return "V4L2-RGB565X";
	case V4L2_PIX_FMT_BGR24:
		return "V4L2-BGR24";
	case V4L2_PIX_FMT_RGB24:
		return "V4L2-RGB24";
	case V4L2_PIX_FMT_BGR32:
		return "V4L2-BGR32";
	case V4L2_PIX_FMT_RGB32:
		return "V4L2-RGB32";
	case V4L2_PIX_FMT_GREY:
		return "V4L2-GREY";
	case V4L2_PIX_FMT_YVU410:
		return "V4L2-YVU410";
	case V4L2_PIX_FMT_YVU420:
		return "V4L2-YVU420";
	case V4L2_PIX_FMT_YUYV:
		return "V4L2-YUYV";
	case V4L2_PIX_FMT_UYVY:
		return "V4L2-UYVY";
	case V4L2_PIX_FMT_YUV422P:
		return "V4L2-YUV422P";
	case V4L2_PIX_FMT_YUV411P:
		return "V4L2-YUV411P";
	case V4L2_PIX_FMT_Y41P:
		return "V4L2-Y41P";
	case V4L2_PIX_FMT_NV12:
		return "V4L2-NV12";
	case V4L2_PIX_FMT_NV21:
		return "V4L2-NV21";
	case V4L2_PIX_FMT_YUV410:
		return "V4L2-YUV410";
	case V4L2_PIX_FMT_YUV420:
		return "V4L2--YUV420";
	case V4L2_PIX_FMT_YYUV:
		return "V4L2-YYUV";
	case V4L2_PIX_FMT_HI240:
		return "V4L2-HI240";
	case V4L2_PIX_FMT_WNVA:
		return "V4L2-WNVA";
	case V4L2_PIX_FMT_NV16:
		return "V4L2-NV16";
	case V4L2_PIX_FMT_YUV444:
		return "V4L2-YUV444P";
	case V4L2_PIX_FMT_NV24:
		return "M5-YUV444SP";
	case V4L2_PIX_FMT_JPEG:
		return "V4L2-JPEG";
	case V4L2_PIX_FMT_SGRBG10:
		return "RAW-BAYER-10Bits";
	case V4L2_PIX_FMT_SGRBG8:
		return "RAW-BAYER-8Bits";
	}
	return "UNKNOWN/UNSUPPORTED";
}

static enum cif_isp20_inp cif_isp20_v4l2_inp2cif_isp20_inp(
	unsigned int i)
{
	enum cif_isp20_inp inp;

	if (0 == i)
		inp = CIF_ISP20_INP_CSI_0;
	else if (1 == i)
		inp = CIF_ISP20_INP_CSI_1;
	else if (2 == i)
		inp = CIF_ISP20_INP_CPI;
	else if (3 == i)
		inp = CIF_ISP20_INP_DMA;
	else if (4 == i)
		inp = CIF_ISP20_INP_DMA_IE;
	else if (5 == i)
		inp = CIF_ISP20_INP_DMA_SP;
	else {
		cif_isp20_pltfrm_pr_err(NULL,
			"index %d out of bounds\n", i);
		return -EINVAL;
	}

	return inp;
}

static int cif_isp20_v4l2_cid2cif_isp20_cid(u32 v4l2_cid)
{
	switch (v4l2_cid) {
	case V4L2_CID_FLASH_LED_MODE:
		return CIF_ISP20_CID_FLASH_MODE;
	case V4L2_CID_AUTOGAIN:
		return CIF_ISP20_CID_AUTO_GAIN;
	case V4L2_CID_EXPOSURE_AUTO:
		return CIF_ISP20_CID_AUTO_EXPOSURE;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return CIF_ISP20_CID_AUTO_WHITE_BALANCE;
	case V4L2_CID_BLACK_LEVEL:
		return CIF_ISP20_CID_BLACK_LEVEL;
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		return CIF_ISP20_CID_WB_TEMPERATURE;
	case V4L2_CID_EXPOSURE:
		return CIF_ISP20_CID_EXPOSURE_TIME;
	case V4L2_CID_GAIN:
		return CIF_ISP20_CID_ANALOG_GAIN;
	case V4L2_CID_FOCUS_ABSOLUTE:
		return CIF_ISP20_CID_FOCUS_ABSOLUTE;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return CIF_ISP20_CID_AUTO_N_PRESET_WHITE_BALANCE;
	case V4L2_CID_SCENE_MODE:
		return CIF_ISP20_CID_SCENE_MODE;
	case V4L2_CID_COLORFX:
		return CIF_ISP20_CID_IMAGE_EFFECT;
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		return CIF_ISP20_CID_JPEG_QUALITY;
	case V4L2_CID_HFLIP:
		return CIF_ISP20_CID_HFLIP;
	case V4L2_CID_VFLIP:
		return CIF_ISP20_CID_VFLIP;
	case V4L2_CID_ISO_SENSITIVITY:
		return CIF_ISP20_CID_ISO_SENSITIVITY;
	case V4L2_CID_3A_LOCK:
		return CIF_ISP20_CID_3A_LOCK;
	case INTEL_V4L2_CID_AUTO_FPS:
		return CIF_ISP20_CID_AUTO_FPS;
	default:
		cif_isp20_pltfrm_pr_err(NULL,
			"unknown/unsupported V4L2 CID 0x%x\n",
			v4l2_cid);
		break;
	}
	return -EINVAL;
}

static enum cif_isp20_image_effect cif_isp20_v4l2_colorfx2cif_isp20_ie(
	u32 v4l2_colorfx)
{
	switch (v4l2_colorfx) {
	case V4L2_COLORFX_SEPIA:
		return CIF_ISP20_IE_SEPIA;
	case V4L2_COLORFX_BW:
		return CIF_ISP20_IE_BW;
	case V4L2_COLORFX_NEGATIVE:
		return CIF_ISP20_IE_NEGATIVE;
	case V4L2_COLORFX_EMBOSS:
		return CIF_ISP20_IE_EMBOSS;
	case V4L2_COLORFX_SKETCH:
		return CIF_ISP20_IE_SKETCH;
	case V4L2_COLORFX_NONE:
		return CIF_ISP20_IE_NONE;
	default:
		cif_isp20_pltfrm_pr_err(NULL,
			"unknown/unsupported V4L2 COLORFX %d\n",
			v4l2_colorfx);
		break;
	}
	return -EINVAL;
}

static enum cif_isp20_pix_fmt cif_isp20_v4l2_pix_fmt2cif_isp20_pix_fmt(
	u32 v4l2_pix_fmt)
{
	switch (v4l2_pix_fmt) {
	case V4L2_PIX_FMT_GREY:
		return CIF_YUV400;
	case V4L2_PIX_FMT_YUV420:
		return CIF_YUV420P;
	case V4L2_PIX_FMT_YVU420:
		return CIF_YVU420P;
	case V4L2_PIX_FMT_NV12:
		return CIF_YUV420SP;
	case V4L2_PIX_FMT_NV21:
		return CIF_YVU420SP;
	case V4L2_PIX_FMT_YUYV:
		return CIF_YUV422I;
	case V4L2_PIX_FMT_UYVY:
		return CIF_UYV422I;
	case V4L2_PIX_FMT_YUV422P:
		return CIF_YUV422P;
	case V4L2_PIX_FMT_NV16:
		return CIF_YUV422SP;
	case V4L2_PIX_FMT_YUV444:
		return CIF_YUV444P;
	case V4L2_PIX_FMT_NV24:
		return CIF_YUV444SP;
	case V4L2_PIX_FMT_RGB565:
		return CIF_RGB565;
	case V4L2_PIX_FMT_RGB24:
		return CIF_RGB888;
	case V4L2_PIX_FMT_SBGGR8:
		return CIF_BAYER_SBGGR8;
	case V4L2_PIX_FMT_SGBRG8:
		return CIF_BAYER_SGBRG8;
	case V4L2_PIX_FMT_SGRBG8:
		return CIF_BAYER_SGRBG8;
	case V4L2_PIX_FMT_SRGGB8:
		return CIF_BAYER_SRGGB8;
	case V4L2_PIX_FMT_SBGGR10:
		return CIF_BAYER_SBGGR10;
	case V4L2_PIX_FMT_SGBRG10:
		return CIF_BAYER_SGBRG10;
	case V4L2_PIX_FMT_SGRBG10:
		return CIF_BAYER_SGRBG10;
	case V4L2_PIX_FMT_SRGGB10:
		return CIF_BAYER_SRGGB10;
	case V4L2_PIX_FMT_SBGGR12:
		return CIF_BAYER_SBGGR12;
	case V4L2_PIX_FMT_SGBRG12:
		return CIF_BAYER_SGBRG12;
	case V4L2_PIX_FMT_SGRBG12:
		return CIF_BAYER_SGRBG12;
	case V4L2_PIX_FMT_SRGGB12:
		return CIF_BAYER_SRGGB12;
	case V4L2_PIX_FMT_JPEG:
		return CIF_JPEG;
	default:
		cif_isp20_pltfrm_pr_err(NULL,
			"unknown or unsupported V4L2 pixel format %c%c%c%c\n",
			(u8)(v4l2_pix_fmt & 0xff),
			(u8)((v4l2_pix_fmt >> 8) & 0xff),
			(u8)((v4l2_pix_fmt >> 16) & 0xff),
			(u8)((v4l2_pix_fmt >> 24) & 0xff));
		return CIF_UNKNOWN_FORMAT;
	}
}

static int cif_isp20_v4l2_register_video_device(
	struct cif_isp20_device *dev,
	struct video_device *vdev,
	const char *name,
	int qtype,
	int major,
	const struct v4l2_file_operations *fops,
	const struct v4l2_ioctl_ops *ioctl_ops)
{
	int ret;

	vdev->release = video_device_release;
	strlcpy(vdev->name, name, sizeof(vdev->name));
	vdev->vfl_type = qtype;
	vdev->fops = fops;
	video_set_drvdata(vdev, dev);
	vdev->minor = -1;
	vdev->ioctl_ops = ioctl_ops;
	vdev->v4l2_dev = &dev->v4l2_dev;
	if (qtype == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		vdev->vfl_dir = VFL_DIR_TX;
	else
		vdev->vfl_dir = VFL_DIR_RX;

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, major);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"video_register_device failed with error %d\n", ret);
		goto err;
	}

	cif_isp20_pltfrm_pr_info(NULL,
		"video device video%d.%d (%s) successfully registered\n",
		major, vdev->minor, name);

	return 0;
err:
	video_device_release(vdev);
	cif_isp20_pltfrm_pr_err(NULL,
		"failed with err %d\n", ret);
	return ret;
}


static int cif_isp20_v4l2_streamon(
	struct file *file,
	void *priv,
	enum v4l2_buf_type buf_type)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	static u32 streamon_cnt_sp;
	static u32 streamon_cnt_mp;
	static u32 streamon_cnt_dma;
	struct cif_isp20_v4l2_fh *fh = to_fh(file);
	struct cif_isp20_v4l2_node *node = to_node(fh);
	u32 stream_ids = to_stream_id(file);

	if (likely(node) && (node->owner != fh))
		return -EBUSY;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "%s(%d)\n",
		cif_isp20_v4l2_buf_type_string(queue->type),
		(stream_ids & CIF_ISP20_STREAM_MP) ? ++streamon_cnt_mp :
		((stream_ids & CIF_ISP20_STREAM_SP) ? ++streamon_cnt_sp :
		++streamon_cnt_dma));

	ret = videobuf_streamon(queue);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"videobuf_streamon failed\n");
		goto err;
	}

	ret = cif_isp20_streamon(dev, stream_ids);
	if (IS_ERR_VALUE(ret)) {
		videobuf_queue_cancel(queue);
		goto err;
	}

	return 0;
err:
	(void)videobuf_mmap_free(queue);
	cif_isp20_pltfrm_pr_err(dev->dev, "failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_v4l2_do_streamoff(
	struct file *file)
{
	int ret = 0;
	int err;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	struct cif_isp20_v4l2_fh *fh = to_fh(file);
	struct cif_isp20_v4l2_node *node = to_node(fh);
	u32 stream_ids = to_stream_id(file);

	cif_isp20_pltfrm_pr_dbg(dev->dev, "%s\n",
		cif_isp20_v4l2_buf_type_string(queue->type));

	if (likely(node) && (node->owner != fh))
		return -EBUSY;

	err = cif_isp20_streamoff(dev, stream_ids);
	if (IS_ERR_VALUE(err))
		ret = -EFAULT;

	videobuf_stop(queue);

	err = videobuf_mmap_free(queue);
	if (IS_ERR_VALUE(err)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"videobuf_mmap_free failed with error %d\n", err);
		ret = -EFAULT;
	}

	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_err(dev->dev,
			"failed with error %d\n", ret);

	return ret;
}

static int cif_isp20_v4l2_streamoff(
	struct file *file,
	void *priv,
	enum v4l2_buf_type buf_type)
{
	int ret = cif_isp20_v4l2_do_streamoff(file);
	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_err(NULL,
			"failed with error %d\n", ret);

	return ret;
}

static int cif_isp20_v4l2_qbuf(
	struct file *file,
	void *priv,
	struct v4l2_buffer *buf)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_v4l2_fh *fh = to_fh(file);
	struct cif_isp20_v4l2_node *node = to_node(fh);

	cif_isp20_pltfrm_pr_dbg(NULL,
		"%s buffer type %s, index %d\n",
		cif_isp20_v4l2_buf_type_string(queue->type),
		cif_isp20_v4l2_buf_type_string(buf->type),
		buf->index);

	if (likely(node) && (node->owner != fh))
		return -EBUSY;

	ret = videobuf_qbuf(queue, buf);
	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_err(NULL,
			"videobuf_qbuf failed with error %d\n", ret);

	return ret;
}

static int cif_isp20_v4l2_dqbuf(
	struct file *file,
	void *priv,
	struct v4l2_buffer *buf)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_v4l2_fh *fh = to_fh(file);
	struct cif_isp20_v4l2_node *node = to_node(fh);

	cif_isp20_pltfrm_pr_dbg(NULL, "%s\n",
		cif_isp20_v4l2_buf_type_string(queue->type));

	if (likely(node) && (node->owner != fh))
		return -EBUSY;

	ret = videobuf_dqbuf(queue, buf, file->f_flags & O_NONBLOCK);
	if (IS_ERR_VALUE(ret) && (ret != -EAGAIN))
		cif_isp20_pltfrm_pr_err(NULL,
			"videobuf_dqbuf failed with error %d\n", ret);
	else
		cif_isp20_pltfrm_pr_dbg(NULL,
			"dequeued buffer %d, size %d\n",
			buf->index, buf->length);

	return ret;
}

static void cif_isp20_v4l2_buf_release(
	struct videobuf_queue *queue,
	struct videobuf_buffer *buf)
{
	cif_isp20_pltfrm_pr_dbg(NULL,
		"%s\n",
		cif_isp20_v4l2_buf_type_string(queue->type));

	if (in_interrupt())
		BUG();

	videobuf_dma_contig_free(queue, buf);

	buf->state = VIDEOBUF_NEEDS_INIT;
}

static void cif_isp20_v4l2_buf_queue(
	struct videobuf_queue *queue,
	struct videobuf_buffer *buf)
{
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	enum cif_isp20_stream_id strm = to_cif_isp20_stream_id(queue->type);

	cif_isp20_pltfrm_pr_dbg(NULL,
		"%s %dx%d, size %lu, bytesperline %d\n",
		cif_isp20_v4l2_buf_type_string(queue->type),
		buf->width, buf->height, buf->size, buf->bytesperline);

	if (!IS_ERR_VALUE(cif_isp20_qbuf(dev, strm, buf)))
		buf->state = VIDEOBUF_QUEUED;
	else
		cif_isp20_pltfrm_pr_err(NULL, "failed\n");
}

static int cif_isp20_v4l2_buf_setup(
	struct videobuf_queue *queue,
	unsigned int *cnt,
	unsigned int *size)
{
	int ret;
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	enum cif_isp20_stream_id strm = to_cif_isp20_stream_id(queue->type);

	cif_isp20_pltfrm_pr_dbg(NULL, "%s count %d, size %d\n",
		cif_isp20_v4l2_buf_type_string(queue->type),
		*cnt, *size);

	ret = cif_isp20_calc_min_out_buff_size(
		dev, strm, size);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
		return ret;
	}

	cif_isp20_pltfrm_pr_dbg(NULL, "%s count %d, size %d\n",
		cif_isp20_v4l2_buf_type_string(queue->type),
		*cnt, *size);

	return 0;
}

static int cif_isp20_v4l2_buf_prepare(
	struct videobuf_queue *queue,
	struct videobuf_buffer *buf,
	enum v4l2_field field)
{
	int ret;
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	enum cif_isp20_stream_id strm = to_cif_isp20_stream_id(queue->type);
	u32 size;

	cif_isp20_pltfrm_pr_dbg(NULL, "%s\n",
		cif_isp20_v4l2_buf_type_string(queue->type));

	ret = cif_isp20_calc_min_out_buff_size(
		dev, strm, &size);
	if (IS_ERR_VALUE(ret))
		goto err;
	buf->size = size;
	if (strm == CIF_ISP20_STREAM_SP) {
		buf->width =
			dev->config.mi_config.sp.output.width;
		buf->height =
			dev->config.mi_config.sp.output.height;
	} else if (strm == CIF_ISP20_STREAM_MP) {
		buf->width =
			dev->config.mi_config.mp.output.width;
		buf->height =
			dev->config.mi_config.mp.output.height;
	} else if (strm == CIF_ISP20_STREAM_DMA) {
		buf->width =
			dev->config.mi_config.dma.output.width;
		buf->height =
			dev->config.mi_config.dma.output.height;
	} else {
		cif_isp20_pltfrm_pr_err(NULL,
			"wrong buffer queue %d\n", queue->type);
		ret = -EINVAL;
		goto err;
	}
	buf->field = field;

	cif_isp20_pltfrm_pr_dbg(NULL, "%s buffer prepared %dx%d, size %d\n",
		cif_isp20_v4l2_buf_type_string(queue->type),
		buf->width, buf->height, size);

	if (buf->state == VIDEOBUF_NEEDS_INIT) {
		ret = videobuf_iolock(queue, buf, NULL);
		if (IS_ERR_VALUE(ret)) {
			cif_isp20_pltfrm_pr_err(NULL,
				"videobuf_iolock failed with error %d\n", ret);
			goto err;
		}
	}
	buf->state = VIDEOBUF_PREPARED;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	cif_isp20_v4l2_buf_release(queue, buf);
	return ret;
}

static int cif_isp20_v4l2_reqbufs(
	struct file *file,
	void *priv,
	struct v4l2_requestbuffers *req)
{
	struct cif_isp20_v4l2_fh *fh = to_fh(file);
	struct cif_isp20_v4l2_node *node = to_node(fh);
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);

	cif_isp20_pltfrm_pr_dbg(NULL,
		"%s requested type %s, count %d\n",
		cif_isp20_v4l2_buf_type_string(queue->type),
		cif_isp20_v4l2_buf_type_string(req->type),
		req->count);

	if (likely(node)) {
		if (node->owner && (node->owner != fh))
			return -EBUSY;
		node->owner = fh;
	}

	if (req->count == 0) {
		cif_isp20_v4l2_do_streamoff(file);
		return 0;
	}

	ret = videobuf_reqbufs(queue, req);
	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_err(NULL,
			"videobuf_reqbufs failed with error %d\n", ret);

	return ret;
}

static int cif_isp20_v4l2_querybuf(
	struct file *file,
	void *priv,
	struct v4l2_buffer *buf)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);

	cif_isp20_pltfrm_pr_dbg(NULL,
		"%s, index %d\n",
		cif_isp20_v4l2_buf_type_string(queue->type), buf->index);

	ret = videobuf_querybuf(queue, buf);
	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_err(NULL,
			"videobuf_querybuf failed with error %d\n", ret);

	return ret;
}

static int cif_isp20_v4l2_s_ctrl(
	struct file *file,
	void *priv,
	struct v4l2_control *vc)
{
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	enum cif_isp20_cid id =
		cif_isp20_v4l2_cid2cif_isp20_cid(vc->id);
	int val = vc->value;

	if (IS_ERR_VALUE(id))
		return id;

	switch (vc->id) {
	case V4L2_CID_COLORFX:
		val = cif_isp20_v4l2_colorfx2cif_isp20_ie(val);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		if (vc->value == V4L2_FLASH_LED_MODE_NONE)
			val = CIF_ISP20_FLASH_MODE_OFF;
		else if (vc->value == V4L2_FLASH_LED_MODE_FLASH)
			val = CIF_ISP20_FLASH_MODE_FLASH;
		else if (vc->value == V4L2_FLASH_LED_MODE_TORCH)
			val = CIF_ISP20_FLASH_MODE_TORCH;
		else
			val = -EINVAL;
		break;
	default:
		break;
	}

	return cif_isp20_s_ctrl(dev, id, val);
}

static int cif_isp20_v4l2_s_fmt(
	struct file *file,
	void *priv,
	struct v4l2_format *f)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	struct cif_isp20_v4l2_fh *fh = to_fh(file);
	struct cif_isp20_v4l2_node *node = to_node(fh);
	struct cif_isp20_strm_fmt strm_fmt;

	cif_isp20_pltfrm_pr_dbg(NULL,
		"%s\n",
		cif_isp20_v4l2_buf_type_string(queue->type));

	if (likely(node) && node->owner && (node->owner != fh))
		return -EBUSY;

	strm_fmt.frm_fmt.pix_fmt =
		cif_isp20_v4l2_pix_fmt2cif_isp20_pix_fmt(
			f->fmt.pix.pixelformat);
	strm_fmt.frm_fmt.width = f->fmt.pix.width;
	strm_fmt.frm_fmt.height = f->fmt.pix.height;

	ret = cif_isp20_s_fmt(dev,
		to_stream_id(file),
		&strm_fmt,
		f->fmt.pix.bytesperline);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(NULL,
		"failed with error %d\n", ret);
	return ret;
}

/* existence of this function is checked by V4L2 */
static int cif_isp20_v4l2_g_fmt(
	struct file *file,
	void *priv,
	struct v4l2_format *f)
{
	return -EFAULT;
}

static int cif_isp20_v4l2_s_input(
	struct file *file,
	void *priv,
	unsigned int i)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);

	cif_isp20_pltfrm_pr_dbg(dev->dev, "setting input to %d\n", i);

	if (i & 8) {
		ret = cif_isp20_s_ctrl(dev,
			CIF_ISP20_CID_SUPER_IMPOSE, true);
		if (IS_ERR_VALUE(ret))
			goto err;
		i -= 8;
	} else
		ret = cif_isp20_s_ctrl(dev,
			CIF_ISP20_CID_SUPER_IMPOSE, false);

	ret = cif_isp20_s_input(dev,
		cif_isp20_v4l2_inp2cif_isp20_inp(i));
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(NULL,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_v4l2_enum_framesizes(
	struct file *file,
	void *priv,
	struct v4l2_frmsizeenum *fsize)
{
	/* THIS FUNCTION IS UNDER CONSTRUCTION */
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);

	if (IS_ERR_OR_NULL(dev->img_src)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"input has not yet been selected, cannot enumerate formats\n");
		ret = -ENODEV;
		goto err;
	}

	return -EINVAL;
err:
	cif_isp20_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	return ret;
}

/* fops **********************************************************************/

const struct videobuf_queue_ops cif_isp20_qops = {
	.buf_setup = cif_isp20_v4l2_buf_setup,
	.buf_prepare = cif_isp20_v4l2_buf_prepare,
	.buf_queue = cif_isp20_v4l2_buf_queue,
	.buf_release = cif_isp20_v4l2_buf_release,
};

static int cif_isp20_v4l2_open(
	struct file *file)
{
	int ret;
	struct video_device *vdev = video_devdata(file);
	struct cif_isp20_device *dev = video_get_drvdata(vdev);
	struct cif_isp20_v4l2_fh *fh;
	struct cif_isp20_v4l2_node *node;
	enum v4l2_buf_type buf_type;
	enum cif_isp20_stream_id stream_id;

	cif_isp20_pltfrm_pr_dbg(NULL,
		"video device video%d.%d (%s)\n",
		vdev->num, vdev->minor, vdev->name);

	if (vdev->minor == cif_isp20_v4l2_dev.node[SP_DEV].vdev.minor) {
		buf_type = V4L2_BUF_TYPE_VIDEO_OVERLAY;
		stream_id = CIF_ISP20_STREAM_SP;
	} else if (vdev->minor == cif_isp20_v4l2_dev.node[MP_DEV].vdev.minor) {
		buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		stream_id = CIF_ISP20_STREAM_MP;
	} else if (vdev->minor ==
				cif_isp20_v4l2_dev.node[DMA_DEV].vdev.minor) {
		buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		stream_id = CIF_ISP20_STREAM_DMA;
	} else {
		cif_isp20_pltfrm_pr_err(NULL,
			"invalid video device video%d.%d (%s)\n",
			vdev->num, vdev->minor, vdev->name);
		ret = -EINVAL;
		goto err;
	}

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (NULL == fh) {
		cif_isp20_pltfrm_pr_err(NULL,
			"memory allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}
	fh->stream_id = stream_id;

	file->private_data = &fh->fh;
	v4l2_fh_init(&fh->fh, vdev);
	v4l2_fh_add(&fh->fh);

	node = to_node(fh);
	if (!node) {
		ret = -ENXIO;
		goto err_free;
	}
	if (likely(node)) {
		if (++node->users > 1)
			return 0;

		/* First open of the device, so initialize everything */
		node->owner = NULL;
	}

	videobuf_queue_dma_contig_init(
		to_videobuf_queue(file),
		&cif_isp20_qops,
		dev->dev,
		&dev->vbq_lock,
		buf_type,
		V4L2_FIELD_NONE,
		sizeof(struct videobuf_buffer),
		dev, NULL);

	ret = cif_isp20_init(dev, to_stream_id(file));
	if (IS_ERR_VALUE(ret)) {
		if (likely(node))
			node->users--;
		goto err_free;
	}

	return 0;

err_free:
	v4l2_fh_del(&fh->fh);
	v4l2_fh_exit(&fh->fh);
	kfree(fh);
err:
	cif_isp20_pltfrm_pr_err(NULL,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_v4l2_release(struct file *file)
{
	int ret = 0;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	struct cif_isp20_v4l2_fh *fh = to_fh(file);
	struct cif_isp20_v4l2_node *node = to_node(fh);
	enum cif_isp20_stream_id stream_id = to_stream_id(file);

	cif_isp20_pltfrm_pr_dbg(dev->dev, "%s\n",
		cif_isp20_v4l2_buf_type_string(queue->type));

	if (likely(node) && node->users)
		--node->users;
	else if (likely(node)) {
		cif_isp20_pltfrm_pr_warn(dev->dev,
			"number of users for this device is already 0\n");
		return 0;
	}

	if (unlikely(!node) || !node->users) {
		if (queue->streaming)
			if (IS_ERR_VALUE(cif_isp20_v4l2_do_streamoff(file)))
				cif_isp20_pltfrm_pr_warn(dev->dev,
					"streamoff failed\n");

		/* Last close, so uninitialize hardware */
		ret = cif_isp20_release(dev, stream_id);
	}

	if (likely(node) && (node->owner == fh))
		node->owner = NULL;

	v4l2_fh_del(&fh->fh);
	v4l2_fh_exit(&fh->fh);
	kfree(fh);

	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_err(dev->dev,
			"failed with error %d\n", ret);
	return ret;
}

static unsigned int cif_isp20_v4l2_poll(
	struct file *file,
	struct poll_table_struct *wait)
{
	struct cif_isp20_v4l2_fh *fh = to_fh(file);
	int ret = 0;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	unsigned long req_events = poll_requested_events(wait);

	cif_isp20_pltfrm_pr_dbg(NULL, "%s\n",
		cif_isp20_v4l2_buf_type_string(queue->type));

	if (v4l2_event_pending(&fh->fh))
		ret = POLLPRI;
	else if (req_events & POLLPRI)
		poll_wait(file, &fh->fh.wait, wait);

	if (!(req_events & (POLLIN | POLLRDNORM)))
		return ret;

	ret |= videobuf_poll_stream(file, queue, wait);
	if (ret & POLLERR) {
		cif_isp20_pltfrm_pr_err(NULL,
			"videobuf_poll_stream failed with error 0x%x\n", ret);
	}
	return ret;
}

const struct v4l2_file_operations cif_isp20_v4l2_fops = {
	.open = cif_isp20_v4l2_open,
	.unlocked_ioctl = video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = video_ioctl2,
#endif
	.release = cif_isp20_v4l2_release,
	.poll = cif_isp20_v4l2_poll,
};

/*TBD: clean up code below this line******************************************/

static int v4l2_querycap(struct file *file,
			 void *priv, struct v4l2_capability *cap)
{
	int ret = 0;
	strcpy(cap->driver, DRIVER_NAME);
	strcpy(cap->card, "xgold-cif");
	cap->version = 21;
	cap->capabilities =
	    V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OVERLAY |
	    V4L2_CAP_STREAMING;
	return ret;
}

static int cif_isp20_v4l2_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;
	return v4l2_event_subscribe(fh, sub, 16, NULL);
}

static int cif_isp20_v4l2_unsubscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

static void cif_isp20_v4l2_event(__u32 frame_sequence)
{
	struct v4l2_event ev;

	memset(&ev, 0, sizeof(ev));
	ev.type = V4L2_EVENT_FRAME_SYNC;
	ev.u.frame_sync.frame_sequence = frame_sequence;
	v4l2_event_queue(&cif_isp20_v4l2_dev.node[SP_DEV].vdev, &ev);
}

static void cif_isp20_v4l2_requeue_bufs(
	enum cif_isp20_stream_id stream_id)
{
	struct videobuf_buffer *buf;
	struct videobuf_queue *q;

	if (stream_id == CIF_ISP20_STREAM_SP)
		q = &cif_isp20_v4l2_dev.node[SP_DEV].buf_queue;
	else if (stream_id == CIF_ISP20_STREAM_MP)
		q = &cif_isp20_v4l2_dev.node[MP_DEV].buf_queue;
	else if (stream_id == CIF_ISP20_STREAM_DMA)
		q = &cif_isp20_v4l2_dev.node[DMA_DEV].buf_queue;
	else
		BUG();

	list_for_each_entry(buf, &q->stream, stream) {
		if (!IS_ERR_VALUE(cif_isp20_qbuf(
			to_cif_isp20_device(q), stream_id, buf)))
			buf->state = VIDEOBUF_QUEUED;
		else
			cif_isp20_pltfrm_pr_err(NULL,
				"failed for buffer %d\n", buf->i);
	}
}


static long v4l2_default_ioctl(struct file *file, void *fh,
			       bool valid_prio, unsigned int cmd, void *arg)
{
	int ret = -EINVAL;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);

	if (arg == NULL) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"NULL Pointer Violation from IOCTL arg:0x%x\n",
			(unsigned int)arg);
		return ret;
	}

	if (cmd == INTEL_VIDIOC_SENSOR_MODE_DATA) {
		struct isp_supplemental_sensor_mode_data *p_mode_data =
		(struct isp_supplemental_sensor_mode_data *)arg;

		ret = (int)cif_isp20_img_src_ioctl(dev->img_src,
			INTEL_VIDIOC_SENSOR_MODE_DATA, p_mode_data);

		if (ret < 0) {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"failed to get sensor mode data\n");
			return ret;
		}

		ret = cif_isp20_calc_isp_cropping(dev,
			&p_mode_data->isp_input_width,
			&p_mode_data->isp_input_height,
			&p_mode_data->isp_input_horizontal_start,
			&p_mode_data->isp_input_vertical_start);

		if (ret < 0) {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"failed to get isp input info\n");
			return ret;
		}
	} else
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"ignored unknown cmd 0x%x from %s\n", cmd, current->comm);

	return ret;
}

static int v4l2_s_parm(
	struct file *file,
	void *priv,
	struct v4l2_streamparm *a)
{
	return 0;
}

static enum isp_camera_port isp_camera_port_from_inp(enum cif_isp20_inp inp)
{
	switch (inp) {
	case CIF_ISP20_INP_CSI_0:
		return ISP_CAMERA_PORT_PRIMARY;
	case CIF_ISP20_INP_CSI_1:
		return ISP_CAMERA_PORT_SECONDARY;
	default:
		break;
	}

	return ISP_CAMERA_NR_PORTS; // Unknown
}

static int v4l2_enum_input(struct file *file, void *priv,
			   struct v4l2_input *input)
{
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	const char *inp_name;
	enum cif_isp20_inp inp;

	if ((queue->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
		(queue->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"wrong buffer queue %d\n", queue->type);
		return -EINVAL;
	}

	if (input->index >= CIF_ISP20_INP_CPI) {
		cif_isp20_pltfrm_pr_err(NULL,
			"index %d out of bounds\n",
			input->index);
		return -EINVAL;
	}

	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = V4L2_STD_UNKNOWN;
	inp = cif_isp20_v4l2_inp2cif_isp20_inp(input->index);
	if (IS_ERR_VALUE(inp))
		return inp;
	input->reserved[1] = isp_camera_port_from_inp(inp);

	inp_name = cif_isp20_g_input_name(dev, inp);
	if (IS_ERR(inp_name))
		return -ENODEV;

	strlcpy(input->name, inp_name, sizeof(input->name));

	return 0;
}

/* ================================================================= */

static int mainpath_g_ctrl(
	struct file *file,
	void *priv,
	struct v4l2_control *vc)
{
	int ret = -EINVAL;

	switch (vc->id) {
	default:
		return -EINVAL;
	}
	return ret;
}

#ifdef NOT_YET
static int mainpath_try_fmt_cap(struct v4l2_format *f)
{
	int ifmt = 0;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	cif_isp20_pltfrm_pr_dbg(NULL, "\n");

	for (ifmt = 0; ifmt < get_xgold_output_format_size(); ifmt++) {
		if (pix->pixelformat == get_xgold_output_format(ifmt)->fourcc)
			break;
	}

	if (ifmt == get_xgold_output_format_size())
		ifmt = 0;

	pix->bytesperline = pix->width *
		get_xgold_output_format(ifmt)->depth / 8;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_YUV444:
	case V4L2_PIX_FMT_NV24:
	case V4L2_PIX_FMT_JPEG:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_SGRBG10:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	default:
		BUG();
		break;
	}

	return 0;
}
#endif

static int v4l2_enum_fmt_cap(struct file *file, void *fh,
			     struct v4l2_fmtdesc *f)
{
	int ret = 0;
	int xgold_num_format = 0;

	xgold_num_format = get_xgold_output_format_desc_size();
	if ((f->index >= xgold_num_format)
	    || (get_xgold_output_format_desc(f->index)->pixelformat == 0)) {

		cif_isp20_pltfrm_pr_err(NULL, "index %d\n", f->index);
		return -EINVAL;
	}
	strlcpy(f->description,
		get_xgold_output_format_desc(f->index)->description,
			sizeof(f->description));
	f->pixelformat = get_xgold_output_format_desc(f->index)->pixelformat;
	f->flags = get_xgold_output_format_desc(f->index)->flags;

	return ret;
}

static int v4l2_g_ctrl(struct file *file, void *priv,
	struct v4l2_control *vc)
{
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	enum cif_isp20_cid id =
		cif_isp20_v4l2_cid2cif_isp20_cid(vc->id);

	return cif_isp20_img_src_g_ctrl(dev->img_src,
		id, &vc->value);
}

static int v4l2_s_ext_ctrls(struct file *file, void *priv,
	struct v4l2_ext_controls *vc_ext)
{
	struct cif_isp20_img_src_ctrl *ctrls;
	struct cif_isp20_img_src_ext_ctrl ctrl;
	int i;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	int ret = -EINVAL;

	/* The only use-case is gain and exposure to sensor. Thus no check if
	 this shall go to img_src or not as of now.*/

	cif_isp20_pltfrm_pr_dbg(dev->dev, "count %d\n",
		vc_ext->count);

	if (vc_ext->count == 0)
		return ret;

	ctrls = kmalloc(vc_ext->count *
		sizeof(struct cif_isp20_img_src_ctrl), GFP_KERNEL);

	if (!ctrls)
		return -ENOMEM;

	for (i = 0; i < vc_ext->count; i++) {
		ctrls[i].id = vc_ext->controls[i].id;
		ctrls[i].val = vc_ext->controls[i].value;
	}

	ctrl.cnt = vc_ext->count;
	ctrl.class = vc_ext->ctrl_class;
	ctrl.ctrls = ctrls;

	if (dev->img_src != NULL)
		ret = cif_isp20_img_src_s_ext_ctrls(dev->img_src, &ctrl);
	else
		cif_isp20_pltfrm_pr_err(dev->dev, "dev->img_src is NULL\n");

	kfree(ctrls);

	return ret;
}

#ifdef CALC_CIF_ZOOMING_CAPACITY
#define CIF_FREQ 442
#define ISP_PIX_CLK_FREQ (CIF_FREQ/2)
#endif
int cif_isp20_v4l2_cropcap(
	struct file *file,
	void *fh,
	struct v4l2_cropcap *a)
{
	int ret = 0;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	u32 target_width, target_height;
	u32 h_offs, v_offs;
#ifdef CALC_CIF_ZOOMING_CAPACITY
	struct isp_supplemental_sensor_mode_data mode_data;
	u32 thresh_zoom;
#endif

	if ((dev->config.input_sel == CIF_ISP20_INP_DMA) ||
		(dev->config.input_sel == CIF_ISP20_INP_DMA_IE)) {

		/* calculate cropping for aspect ratio */
		ret = cif_isp20_calc_isp_cropping(dev,
			&dev->isp_dev.input_width, &dev->isp_dev.input_height,
			&h_offs, &v_offs);

		/* Get output size */
		ret = cif_isp20_get_target_frm_size(dev,
			&target_width, &target_height);
		if (ret < 0) {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"failed to get target frame size\n");
			return ret;
		}

		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"CIF_IN_W=%d, CIF_IN_H=%d, ISP_IN_W=%d, ISP_IN_H=%d, target_width=%d, target_height=%d\n",
			dev->config.isp_config.input->width,
			dev->config.isp_config.input->height,
			dev->isp_dev.input_width,
			dev->isp_dev.input_height,
			target_width,
			target_height);

		/* This is the input to Bayer after input formatter cropping */
		a->defrect.top = 0;
		a->defrect.left = 0;
		a->defrect.width = dev->isp_dev.input_width;
		a->defrect.height = dev->isp_dev.input_height;
		/* This is the minimum cropping window for the IS module */
		a->bounds.width = 2;
		a->bounds.height = 2;
		a->bounds.top = (a->defrect.height - a->bounds.height) / 2;
		a->bounds.left = (a->defrect.width - a->bounds.width) / 2;

		a->type = V4L2_BUF_TYPE_VIDEO_OVERLAY;
	} else if ((dev->config.input_sel == CIF_ISP20_INP_CSI_0) ||
		(dev->config.input_sel == CIF_ISP20_INP_CSI_1)) {
		/* calculate cropping for aspect ratio */
		ret = cif_isp20_calc_isp_cropping(dev,
			&dev->isp_dev.input_width, &dev->isp_dev.input_height,
			&h_offs, &v_offs);

		/* Get output size */
		ret = cif_isp20_get_target_frm_size(dev,
			&target_width, &target_height);
		if (ret < 0) {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"failed to get target frame size\n");
			return ret;
		}

		/* This is the input to Bayer after input formatter cropping */
		a->defrect.top = 0;
		a->defrect.left = 0;
		a->defrect.width = dev->isp_dev.input_width;
		a->defrect.height = dev->isp_dev.input_height;


		/* The chapter "On-The-Fly Zooming" in CIF 4.12 spec. gives
		some fomular to estimate the HW zooming capacity of CIF.
		However, it is not accurate enough, so here we don't
		estimate it until reasonable description is given.
		The v4l2_cropcap.bounds is set to the minimum */
#ifdef CALC_CIF_ZOOMING_CAPACITY
		/* Get sensor timings */
		ret = (int)cif_isp20_img_src_ioctl(dev->img_src,
			INTEL_VIDIOC_SENSOR_MODE_DATA, &mode_data);

		if (ret < 0) {
			cif_isp20_pltfrm_pr_err(dev->dev,
				"failed to get sensor mode data\n");
			return ret;
		}

		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"VTS=%d, HTS=%d, PCLK=%d, CIF_IN_W=%d, CIF_IN_H=%d, ISP_IN_W=%d, ISP_IN_H=%d, target_width=%d, target_height=%d\n",
			mode_data.frame_length_lines, /*VTS*/
			mode_data.line_length_pck, /*HTS*/
			mode_data.vt_pix_clk_freq_hz,
			mode_data.sensor_output_width, /*Sensor_width*/
			mode_data.sensor_output_height, /*Sensor_height*/
			dev->isp_dev.input_width,
			dev->isp_dev.input_height,
			target_width,
			target_height);

		thresh_zoom =
			100*(mode_data.line_length_pck*dev->isp_dev.input_width
			/
			target_width)
			/
			target_width;
		thresh_zoom =
			thresh_zoom * ISP_PIX_CLK_FREQ
			/
			(mode_data.vt_pix_clk_freq_hz/1000000);
		cif_isp20_pltfrm_pr_dbg(dev->dev,
			"th_zoom_ratio(mid): %d\n", thresh_zoom);

		/* This is the minimum cropping window for the IS module */
		a->bounds.width = a->defrect.width * 100 / thresh_zoom;
		a->bounds.width &= ~1;
		a->bounds.height = a->defrect.height * 100 / thresh_zoom;
		a->bounds.height &= ~1;
#else
		a->bounds.width = 2;
		a->bounds.height = 2;
#endif

		a->bounds.top = (a->defrect.height - a->bounds.height) / 2;
		a->bounds.left = (a->defrect.width - a->bounds.width) / 2;

		a->type = V4L2_BUF_TYPE_VIDEO_OVERLAY;
	} else {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"cif_isp20_v4l2_cropcap: invalid input\n");
	}

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"v4l2_cropcap: defrect(%d,%d,%d,%d) bounds(%d,%d,%d,%d)\n",
		a->defrect.width,
		a->defrect.height,
		a->defrect.left,
		a->defrect.top,
		a->bounds.width,
		a->bounds.height,
		a->bounds.left,
		a->bounds.top);

	return ret;
}

int cif_isp20_v4l2_g_crop(struct file *file, void *fh, struct v4l2_crop *a)
{
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);

	a->c.width = dev->config.isp_config.ism_config.ism_params.h_size;
	a->c.height = dev->config.isp_config.ism_config.ism_params.v_size;
	a->c.left = dev->config.isp_config.ism_config.ism_params.h_offs;
	a->c.top = dev->config.isp_config.ism_config.ism_params.v_offs;

	return 0;
}

/*
	This is a write only function, so the upper layer
	will ignore the changes to 'a'. So don't use 'a' to pass
	the actual cropping parameters, the upper layer
	should call g_crop to get the actual window.
*/
int cif_isp20_v4l2_s_crop(
	struct file *file,
	void *fh,
	const struct v4l2_crop *a)
{
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	unsigned long flags = 0;
	struct cif_isp20_ism_params *ism_params =
		&(dev->config.isp_config.ism_config.ism_params);

	/* Return if no change*/
	if ((ism_params->h_size == a->c.width) &&
			(ism_params->v_size == a->c.height) &&
			(ism_params->h_offs == a->c.left) &&
			(ism_params->v_offs == a->c.top))
		return 0;

	local_irq_save(flags);

	/* check legal range*/
	if ((a->c.left < 0) ||
		((a->c.left + a->c.width) > dev->isp_dev.input_width) ||
		(a->c.top < 0) ||
		((a->c.top + a->c.height) > dev->isp_dev.input_height)) {
		local_irq_restore(flags);
		cif_isp20_pltfrm_pr_err(dev->dev,
			"Invalid cropping window %dx%d@(%d,%d)\n",
			a->c.width,
			a->c.height,
			a->c.left,
			a->c.top);
		return -EINVAL;
	}

	dev->config.isp_config.ism_config.ism_update_needed = false;
	ism_params->recenter = 0;
	ism_params->displace = 0;
	ism_params->max_dx = 0;
	ism_params->max_dy = 0;

	if ((a->c.width == dev->isp_dev.input_width) &&
		(a->c.height == dev->isp_dev.input_height)) {
		dev->config.isp_config.ism_config.ism_en = 0;
		ism_params->h_size = a->c.width;
		ism_params->v_size = a->c.height;
		ism_params->h_offs = 0;
		ism_params->v_offs = 0;
	} else {
		dev->config.isp_config.ism_config.ism_en = 1;
		ism_params->h_size = a->c.width;
		ism_params->v_size = a->c.height;
		ism_params->h_offs = a->c.left;
		ism_params->v_offs = a->c.top;
	}

	dev->config.isp_config.ism_config.ism_update_needed = true;

	cif_isp20_pltfrm_pr_dbg(dev->dev,
		"crop window= %dx%d@(%d,%d)\n",
		ism_params->h_size,
		ism_params->v_size,
		ism_params->h_offs,
		ism_params->v_offs);

	local_irq_restore(flags);
	return 0;
}

const struct v4l2_ioctl_ops cif_isp20_v4l2_sp_ioctlops = {
	.vidioc_reqbufs = cif_isp20_v4l2_reqbufs,
	.vidioc_querybuf = cif_isp20_v4l2_querybuf,
	.vidioc_qbuf = cif_isp20_v4l2_qbuf,
	.vidioc_dqbuf = cif_isp20_v4l2_dqbuf,
	.vidioc_streamon = cif_isp20_v4l2_streamon,
	.vidioc_streamoff = cif_isp20_v4l2_streamoff,
	.vidioc_s_input = cif_isp20_v4l2_s_input,
	.vidioc_enum_input = v4l2_enum_input,
	.vidioc_g_ctrl = v4l2_g_ctrl,
	.vidioc_s_ctrl = cif_isp20_v4l2_s_ctrl,
	.vidioc_s_fmt_vid_overlay = cif_isp20_v4l2_s_fmt,
	.vidioc_g_fmt_vid_overlay = cif_isp20_v4l2_g_fmt,
	.vidioc_s_ext_ctrls = v4l2_s_ext_ctrls,
	.vidioc_querycap = v4l2_querycap,
	.vidioc_cropcap = cif_isp20_v4l2_cropcap,
	.vidioc_s_crop = cif_isp20_v4l2_s_crop,
	.vidioc_g_crop = cif_isp20_v4l2_g_crop,
	.vidioc_subscribe_event = cif_isp20_v4l2_subscribe_event,
	.vidioc_unsubscribe_event = cif_isp20_v4l2_unsubscribe_event,
	.vidioc_default = v4l2_default_ioctl,
};

const struct v4l2_ioctl_ops cif_isp20_v4l2_mp_ioctlops = {
	.vidioc_reqbufs = cif_isp20_v4l2_reqbufs,
	.vidioc_querybuf = cif_isp20_v4l2_querybuf,
	.vidioc_qbuf = cif_isp20_v4l2_qbuf,
	.vidioc_dqbuf = cif_isp20_v4l2_dqbuf,
	.vidioc_streamon = cif_isp20_v4l2_streamon,
	.vidioc_streamoff = cif_isp20_v4l2_streamoff,
	.vidioc_s_input = cif_isp20_v4l2_s_input,
	.vidioc_enum_input = v4l2_enum_input,
	.vidioc_g_ctrl = mainpath_g_ctrl,
	.vidioc_s_ctrl = cif_isp20_v4l2_s_ctrl,
	.vidioc_s_fmt_vid_cap = cif_isp20_v4l2_s_fmt,
	.vidioc_g_fmt_vid_cap = cif_isp20_v4l2_g_fmt,
	.vidioc_enum_fmt_vid_cap = v4l2_enum_fmt_cap,
	.vidioc_enum_framesizes = cif_isp20_v4l2_enum_framesizes,
	.vidioc_s_parm = v4l2_s_parm,
	.vidioc_cropcap = cif_isp20_v4l2_cropcap,
	.vidioc_s_crop = cif_isp20_v4l2_s_crop,
	.vidioc_g_crop = cif_isp20_v4l2_g_crop,
};

const struct v4l2_ioctl_ops cif_isp20_v4l2_dma_ioctlops = {
	.vidioc_reqbufs = cif_isp20_v4l2_reqbufs,
	.vidioc_querybuf = cif_isp20_v4l2_querybuf,
	.vidioc_qbuf = cif_isp20_v4l2_qbuf,
	.vidioc_dqbuf = cif_isp20_v4l2_dqbuf,
	.vidioc_streamon = cif_isp20_v4l2_streamon,
	.vidioc_streamoff = cif_isp20_v4l2_streamoff,
	.vidioc_s_fmt_vid_out = cif_isp20_v4l2_s_fmt,
	.vidioc_g_fmt_vid_out = cif_isp20_v4l2_g_fmt,
	.vidioc_cropcap = cif_isp20_v4l2_cropcap,
	.vidioc_s_crop = cif_isp20_v4l2_s_crop,
	.vidioc_g_crop = cif_isp20_v4l2_g_crop,
};

static int xgold_v4l2_drv_probe(struct platform_device *pdev)
{
	struct cif_isp20_device *dev = NULL;
	int ret;

	cif_isp20_pltfrm_pr_info(NULL, "probing...\n");

	memset(&cif_isp20_v4l2_dev, 0, sizeof(cif_isp20_v4l2_dev));

	dev = cif_isp20_create(&pdev->dev, cif_isp20_v4l2_event,
		cif_isp20_v4l2_requeue_bufs);
	if (IS_ERR_OR_NULL(dev)) {
		ret = -ENODEV;
		goto err;
	}

	spin_lock_init(&dev->vbq_lock);

	ret = v4l2_device_register(dev->dev, &dev->v4l2_dev);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"V4L2 device registration failed\n");
		goto err;
	}

	ret = cif_isp20_v4l2_register_video_device(
		dev,
		&cif_isp20_v4l2_dev.node[SP_DEV].vdev,
		"CIF ISP20 SP",
		V4L2_CAP_VIDEO_OVERLAY,
		CIIF_ISP20_V4L2_SP_DEV_MAJOR,
		&cif_isp20_v4l2_fops,
		&cif_isp20_v4l2_sp_ioctlops);
	if (ret)
		goto err;

	ret = register_cifisp_device(&dev->isp_dev,
		&cif_isp20_v4l2_dev.node[ISP_DEV].vdev,
		&dev->v4l2_dev,
		dev->config.base_addr);
	if (ret)
		goto err;

	ret = cif_isp20_v4l2_register_video_device(
		dev,
		&cif_isp20_v4l2_dev.node[MP_DEV].vdev,
		"CIF ISP20 MP",
		V4L2_CAP_VIDEO_CAPTURE,
		CIIF_ISP20_V4L2_MP_DEV_MAJOR,
		&cif_isp20_v4l2_fops,
		&cif_isp20_v4l2_mp_ioctlops);
	if (ret)
		goto err;

	ret = cif_isp20_v4l2_register_video_device(
		dev,
		&cif_isp20_v4l2_dev.node[DMA_DEV].vdev,
		"CIF ISP20 DMA",
		V4L2_CAP_VIDEO_OUTPUT,
		CIIF_ISP20_V4L2_DMA_DEV_MAJOR,
		&cif_isp20_v4l2_fops,
		&cif_isp20_v4l2_dma_ioctlops);
	if (ret)
		goto err;

	return 0;
err:
	cif_isp20_destroy(dev);
	return ret;
}

/* ======================================================================== */

static int xgold_v4l2_drv_remove(struct platform_device *pdev)
{
	struct cif_isp20_device *cif_isp20_dev =
		(struct cif_isp20_device *)platform_get_drvdata(pdev);

	if (IS_ERR_VALUE(cif_isp20_release(cif_isp20_dev,
		CIF_ISP20_ALL_STREAMS)))
		cif_isp20_pltfrm_pr_warn(cif_isp20_dev->dev,
			"CIF power off failed\n");

	video_unregister_device(&cif_isp20_v4l2_dev.node[SP_DEV].vdev);
	video_unregister_device(&cif_isp20_v4l2_dev.node[MP_DEV].vdev);
	video_unregister_device(&cif_isp20_v4l2_dev.node[DMA_DEV].vdev);
	unregister_cifisp_device(&cif_isp20_v4l2_dev.node[ISP_DEV].vdev);
	v4l2_device_unregister(&cif_isp20_dev->v4l2_dev);
	cif_isp20_pltfrm_dev_release(&pdev->dev);
	cif_isp20_destroy(cif_isp20_dev);

	return 0;
}

static int xgold_v4l2_drv_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	int ret = 0;
	struct cif_isp20_device *cif_isp20_dev =
		(struct cif_isp20_device *)platform_get_drvdata(pdev);

	cif_isp20_pltfrm_pr_dbg(cif_isp20_dev->dev, "\n");

	ret = cif_isp20_suspend(cif_isp20_dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_isp20_pltfrm_pinctrl_set_state(&pdev->dev,
		CIF_ISP20_PINCTRL_STATE_SLEEP);

	return 0;
err:
	cif_isp20_pltfrm_pr_err(cif_isp20_dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int xgold_v4l2_drv_resume(struct platform_device *pdev)
{
	int ret = 0;
	struct cif_isp20_device *cif_isp20_dev =
		(struct cif_isp20_device *)platform_get_drvdata(pdev);

	cif_isp20_pltfrm_pr_dbg(cif_isp20_dev->dev, "\n");

	ret = cif_isp20_resume(cif_isp20_dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	cif_isp20_pltfrm_pinctrl_set_state(&pdev->dev,
		CIF_ISP20_PINCTRL_STATE_DEFAULT);

	return 0;
err:
	cif_isp20_pltfrm_pr_err(cif_isp20_dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static struct of_device_id xgold_v4l2_of_match[] = {
	{.compatible = "intel," DRIVER_NAME,},
	{},
};

static struct platform_driver xgold_v4l2_plat_drv = {
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = xgold_v4l2_of_match,
		   },
	.probe = xgold_v4l2_drv_probe,
	.remove = xgold_v4l2_drv_remove,
	.suspend = xgold_v4l2_drv_suspend,
	.resume = xgold_v4l2_drv_resume,
};

/* ======================================================================== */
static int xgold_v4l2_init(void)
{
	int ret = platform_driver_register(&xgold_v4l2_plat_drv);

	if (ret) {
		cif_isp20_pltfrm_pr_err(NULL,
			"cannot register platfrom driver, failed with %d\n",
			ret);
		return -ENODEV;
	}
	return ret;
}

/* ======================================================================== */
static void __exit xgold_v4l2_exit(void)
{
	platform_driver_unregister(&xgold_v4l2_plat_drv);
}

late_initcall(xgold_v4l2_init);
module_exit(xgold_v4l2_exit);

MODULE_DESCRIPTION("V4L2 interface for CIF ISP20 driver");
MODULE_AUTHOR("Eike Grimpe");
MODULE_LICENSE("GPL");
