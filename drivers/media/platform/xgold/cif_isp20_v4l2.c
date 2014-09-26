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
#include <media/v4l2-ioctl.h>
#include <media/videobuf-dma-contig.h>
#include "cif_isp20.h"

#define CIIF_ISP20_V4L2_SP_DEV_MAJOR 0
#define CIIF_ISP20_V4L2_ISP_DEV_MAJOR 1
#define CIIF_ISP20_V4L2_MP_DEV_MAJOR 2
#define CIIF_ISP20_V4L2_DMA_DEV_MAJOR 3

struct cif_isp20_v4l2_device {
	struct videobuf_queue buf_queues[4];
	struct cif_isp20_device cif_isp20_dev;
	struct video_device *sp_dev;
	struct video_device *mp_dev;
	struct video_device *dma_dev;
};

/* TODO: make this a dynamically allocated variable */
static struct cif_isp20_v4l2_device cif_isp20_v4l2_dev;

struct cif_isp20_v4l2_fh {
	struct videobuf_queue buf_queue;
	enum cif_isp20_stream_id stream_id;
};

static struct videobuf_queue *to_videobuf_queue(
	struct file *file)
{
	struct cif_isp20_v4l2_fh *fh;
	struct videobuf_queue *q;

	if (unlikely(NULL == file)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"NULL file handle\n");
		BUG();
	}
	fh = file->private_data;
	if (unlikely(NULL == fh)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"fh is NULL\n");
		BUG();
	}
	q = &fh->buf_queue;
	if (unlikely(NULL == q)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"buffer queue is NULL\n");
		BUG();
	}

	if (unlikely(((q->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
		(q->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)))) {
		cif_isp20_pltfrm_pr_err(NULL,
			"wrong video buffer queue\n");
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
	fh = file->private_data;
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

	if (i & 8) {
		inp |= CIF_ISP20_INP_SI;
		inp -= 8;
	}
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
		cif_isp20_pltfrm_pr_err(dev->dev,
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
	case V4L2_EXPOSURE_AUTO:
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
	default:
		cif_isp20_pltfrm_pr_err(NULL,
			"unknown/unsupported V4L2 CID 0x%x\n",
			v4l2_cid);
		break;
	}
	return -EINVAL;
}

#ifdef NOT_YET
static int cif_isp20_v4l2_cid2v4l2_cid(u32 cif_isp20_cid)
{
	switch (cif_isp20_cid) {
	case CIF_ISP20_CID_FLASH_MODE:
		return V4L2_CID_FLASH_LED_MODE;
	case CIF_ISP20_CID_AUTO_GAIN:
		return V4L2_CID_AUTOGAIN;
	case CIF_ISP20_CID_AUTO_EXPOSURE:
		return V4L2_EXPOSURE_AUTO;
	case CIF_ISP20_CID_AUTO_WHITE_BALANCE:
		return V4L2_CID_AUTO_WHITE_BALANCE;
	case CIF_ISP20_CID_BLACK_LEVEL:
		return V4L2_CID_BLACK_LEVEL;
	case CIF_ISP20_CID_WB_TEMPERATURE:
		return V4L2_CID_WHITE_BALANCE_TEMPERATURE;
	case CIF_ISP20_CID_EXPOSURE_TIME:
		return V4L2_CID_EXPOSURE;
	case CIF_ISP20_CID_ANALOG_GAIN:
		return V4L2_CID_GAIN;
	case CIF_ISP20_CID_FOCUS_ABSOLUTE:
		return V4L2_CID_FOCUS_ABSOLUTE;
	case CIF_ISP20_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE;
	case CIF_ISP20_CID_SCENE_MODE:
		return V4L2_CID_SCENE_MODE;
	default:
		cif_isp20_pltfrm_pr_err(NULL,
			"unknown/unsupported CIF ISP20 ID %d\n",
			cif_isp20_cid);
		break;
	}
	return -EINVAL;
}
#endif

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

static struct video_device *cif_isp20_v4l2_register_video_device(
	struct cif_isp20_device *dev,
	const char *name,
	int qtype,
	int major,
	const struct v4l2_file_operations *fops,
	const struct v4l2_ioctl_ops *ioctl_ops)
{
	int ret;
	struct video_device *vdev;

	vdev = video_device_alloc();
	if (!vdev) {
		cif_isp20_pltfrm_pr_err(NULL,
			"could not allocate video device %s\n", name);
		ret = -ENOMEM;
		goto err;
	}

	vdev->release = video_device_release;
	strlcpy(vdev->name, name, sizeof(vdev->name));
	vdev->vfl_type = qtype;
	vdev->fops = fops;
	video_set_drvdata(vdev, dev);
	vdev->minor = -1;
	vdev->ioctl_ops = ioctl_ops;
	vdev->v4l2_dev = &dev->v4l2_dev;

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, major);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"video_register_device failed with error %d\n", ret);
		goto err;
	}

	cif_isp20_pltfrm_pr_info(NULL,
		"video device video%d.%d (%s) successfully registered\n",
		major, vdev->minor, name);

	return vdev;
err:
	video_device_release(vdev);
	cif_isp20_pltfrm_pr_err(NULL,
		"failed with err %d\n", ret);
	return ERR_PTR(ret);
}


static int cif_isp20_v4l2_streamon(
	struct file *file,
	void *priv,
	enum v4l2_buf_type buf_type)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	bool mp = queue->type == V4L2_BUF_TYPE_VIDEO_CAPTURE;
	static u32 streamon_cnt_sp;
	static u32 streamon_cnt_mp;
	u32 stream_ids;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "%s(%d)\n",
		cif_isp20_v4l2_buf_type_string(queue->type),
		mp ? ++streamon_cnt_mp : ++streamon_cnt_sp);

	ret = videobuf_streamon(queue);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"videobuf_streamon failed\n");
		goto err;
	}

	if (mp)
		stream_ids = CIF_ISP20_STREAM_MP;
	else
		stream_ids = CIF_ISP20_STREAM_SP;

	ret = cif_isp20_streamon(dev, stream_ids);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev, "failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_v4l2_streamoff(
	struct file *file,
	void *priv,
	enum v4l2_buf_type buf_type)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	bool mp = queue->type == V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "%s\n",
		cif_isp20_v4l2_buf_type_string(queue->type));

	ret = cif_isp20_streamoff(dev, !mp, mp);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = videobuf_streamoff(queue);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"videobuf_streamoff failed\n");
		goto err;
	}
	ret = videobuf_mmap_free(queue);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"videobuf_mmap_free failed\n");
		goto err;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev->dev, "failed with error %d\n", ret);
	return ret;
}

static int cif_isp20_v4l2_qbuf(
	struct file *file,
	void *priv,
	struct v4l2_buffer *buf)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);

	cif_isp20_pltfrm_pr_dbg(NULL,
		"%s buffer type %s, index %d\n",
		cif_isp20_v4l2_buf_type_string(queue->type),
		cif_isp20_v4l2_buf_type_string(buf->type),
		buf->index);

	ret = videobuf_qbuf(queue, buf);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(NULL, "videobuf_qbuf failed\n");
		cif_isp20_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	}

	return ret;
}

static int cif_isp20_v4l2_dqbuf(
	struct file *file,
	void *priv,
	struct v4l2_buffer *buf)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);

	cif_isp20_pltfrm_pr_dbg(NULL, "%s\n",
		cif_isp20_v4l2_buf_type_string(queue->type));

	ret = videobuf_dqbuf(queue, buf, file->f_flags & O_NONBLOCK);
	if (IS_ERR_VALUE(ret) && (ret != -EAGAIN)) {
		cif_isp20_pltfrm_pr_err(NULL, "videobuf_dqbuf failed\n");
		cif_isp20_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	} else
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
		/* TBD */
		BUG();
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
				"videobuf_iolock failed\n");
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
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);

	cif_isp20_pltfrm_pr_dbg(NULL,
		"%s requested type %s, count %d\n",
		cif_isp20_v4l2_buf_type_string(queue->type),
		cif_isp20_v4l2_buf_type_string(req->type),
		req->count);

	ret = videobuf_reqbufs(queue, req);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(NULL, "videobuf_reqbufs failed\n");
		cif_isp20_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	}

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
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(NULL, "videobuf_querybuf failed\n");
		cif_isp20_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	}

	return ret;
}

static int cif_isp20_v4l2_s_fmt(
	struct file *file,
	void *priv,
	struct v4l2_format *f)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	struct cif_isp20_strm_fmt strm_fmt;

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
	struct cif_isp20_v4l2_fh *fh = NULL;
	enum v4l2_buf_type buf_type;
	enum cif_isp20_stream_id stream_id;

	cif_isp20_pltfrm_pr_dbg(NULL,
		"video device video%d.%d (%s)\n",
		vdev->num, vdev->minor, vdev->name);

	if (vdev->minor == cif_isp20_v4l2_dev.sp_dev->minor) {
		buf_type = V4L2_BUF_TYPE_VIDEO_OVERLAY;
		stream_id = CIF_ISP20_STREAM_SP;
	} else if (vdev->minor == cif_isp20_v4l2_dev.mp_dev->minor) {
		buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		stream_id = CIF_ISP20_STREAM_MP;
	} else if (vdev->minor == cif_isp20_v4l2_dev.dma_dev->minor) {
		buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		stream_id = CIF_ISP20_STREAM_DMA;
	} else {
		cif_isp20_pltfrm_pr_err(NULL,
			"invalid video device video%d.%d (%s)\n",
			vdev->num, vdev->minor, vdev->name);
		ret = -EINVAL;
		goto err;
	}

	fh = kmalloc(sizeof(*fh), GFP_KERNEL);
	if (NULL == fh) {
		cif_isp20_pltfrm_pr_err(NULL,
			"memory allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}
	fh->stream_id = stream_id;
	file->private_data = fh;

	videobuf_queue_dma_contig_init(
		&fh->buf_queue,
		&cif_isp20_qops,
		dev->dev,
		&dev->vbq_lock,
		buf_type,
		V4L2_FIELD_NONE,
		sizeof(struct videobuf_buffer),
		dev, NULL);

	ret = cif_isp20_init(dev, stream_id);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(NULL,
		"failed with error %d\n", ret);
	kfree(fh);
	return ret;
}

static int cif_isp20_v4l2_release(struct file *file)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);

	cif_isp20_pltfrm_pr_dbg(dev->dev, "%s\n",
		cif_isp20_v4l2_buf_type_string(queue->type));
	ret = cif_isp20_release(dev, to_stream_id(file));
	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_err(dev->dev,
			"failed with error %d\n", ret);
	return ret;
}

static unsigned int cif_isp20_v4l2_poll(
	struct file *file,
	struct poll_table_struct *wait)
{
	int ret;
	struct videobuf_queue *queue = to_videobuf_queue(file);

	cif_isp20_pltfrm_pr_dbg(NULL, "%s\n",
		cif_isp20_v4l2_buf_type_string(queue->type));
	ret = videobuf_poll_stream(file, queue, wait);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(NULL,
			"videobuf_poll_stream failed\n");
		cif_isp20_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	}
	return ret;
}

const struct v4l2_file_operations cif_isp20_v4l2_fops = {
	.open = cif_isp20_v4l2_open,
	.ioctl = video_ioctl2,
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

static long v4l2_default_ioctl(struct file *file, void *fh,
			       bool valid_prio, unsigned int cmd, void *arg)
{
	int ret = -EINVAL;
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	int has_auto_gain;

	if (arg == NULL) {
		cif_isp20_pltfrm_pr_err(dev->dev,
			"NULL Pointer Violation from IOCTL arg:0x%x\n",
			(unsigned int)arg);
		return ret;
	}

	if (cmd == INTEL_VIDIOC_SENSOR_MODE_DATA) {
		struct isp_supplemental_sensor_mode_data *p_mode_data =
		(struct isp_supplemental_sensor_mode_data *)arg;

		if (IS_ERR_VALUE(cif_isp20_img_src_g_ctrl(
			dev->img_src, CIF_ISP20_CID_AUTO_GAIN, &has_auto_gain))
			||
			!has_auto_gain)
			p_mode_data->sensor_type = 0;
		else
			p_mode_data->sensor_type = 1;

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
		cif_isp20_pltfrm_pr_err(dev->dev,
			"unknown cmd 0x%x ignored\n", cmd);

	return ret;
}

static int v4l2_g_fmt_overlay(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	return -EFAULT;
}


static int v4l2_s_parm(
	struct file *file,
	void *priv,
	struct v4l2_streamparm *a)
{
	return 0;
}

static int v4l2_s_input(struct file *file, void *priv, unsigned int i)
{
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	enum cif_isp20_inp inp;

	cif_isp20_pltfrm_pr_dbg(dev->dev, "setting input to %d\n", i);

	inp = cif_isp20_v4l2_inp2cif_isp20_inp(i);
	if (IS_ERR_VALUE(inp))
		return inp;

	return cif_isp20_s_input(dev, inp);
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
	inp_name = cif_isp20_g_input_name(dev, inp);
	strcpy(input->name, inp_name);

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

static int v4l2_g_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	return -EFAULT;
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

static int v4l2_s_ctrl(struct file *file, void *priv,
	struct v4l2_control *vc)
{
	struct videobuf_queue *queue = to_videobuf_queue(file);
	struct cif_isp20_device *dev = to_cif_isp20_device(queue);
	int ret;

	switch (vc->id) {
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	case V4L2_CID_COLORFX:
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
	case V4L2_CID_FLASH_LED_MODE:
		ret = marvin_s_ctrl(dev, vc);
		break;
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_BLACK_LEVEL:
	case V4L2_CID_FOCUS_ABSOLUTE:
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
	case V4L2_CID_SCENE_MODE:
		{
			enum cif_isp20_cid id =
				cif_isp20_v4l2_cid2cif_isp20_cid(vc->id);
			ret = cif_isp20_img_src_s_ctrl(dev->img_src,
				id, vc->value);
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
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

const struct v4l2_ioctl_ops cif_isp20_v4l2_sp_ioctlops = {
	.vidioc_reqbufs = cif_isp20_v4l2_reqbufs,
	.vidioc_querybuf = cif_isp20_v4l2_querybuf,
	.vidioc_qbuf = cif_isp20_v4l2_qbuf,
	.vidioc_dqbuf = cif_isp20_v4l2_dqbuf,
	.vidioc_streamon = cif_isp20_v4l2_streamon,
	.vidioc_streamoff = cif_isp20_v4l2_streamoff,
	.vidioc_s_input = v4l2_s_input,
	.vidioc_enum_input = v4l2_enum_input,
	.vidioc_g_ctrl = v4l2_g_ctrl,
	.vidioc_s_ctrl = v4l2_s_ctrl,
	.vidioc_s_fmt_vid_overlay = cif_isp20_v4l2_s_fmt,
	.vidioc_g_fmt_vid_overlay = v4l2_g_fmt_overlay,
	.vidioc_s_ext_ctrls = v4l2_s_ext_ctrls,
	.vidioc_querycap = v4l2_querycap,
	.vidioc_default = v4l2_default_ioctl,
};

const struct v4l2_ioctl_ops cif_isp20_v4l2_mp_ioctlops = {
	.vidioc_reqbufs = cif_isp20_v4l2_reqbufs,
	.vidioc_querybuf = cif_isp20_v4l2_querybuf,
	.vidioc_qbuf = cif_isp20_v4l2_qbuf,
	.vidioc_dqbuf = cif_isp20_v4l2_dqbuf,
	.vidioc_streamon = cif_isp20_v4l2_streamon,
	.vidioc_streamoff = cif_isp20_v4l2_streamoff,
	.vidioc_s_input = v4l2_s_input,
	.vidioc_enum_input = v4l2_enum_input,
	.vidioc_g_ctrl = mainpath_g_ctrl,
	.vidioc_s_ctrl = v4l2_s_ctrl,
	.vidioc_s_fmt_vid_cap = cif_isp20_v4l2_s_fmt,
	.vidioc_g_fmt_vid_cap = v4l2_g_fmt_cap,
	.vidioc_enum_fmt_vid_cap = v4l2_enum_fmt_cap,
	.vidioc_enum_framesizes = cif_isp20_v4l2_enum_framesizes,
	.vidioc_s_parm = v4l2_s_parm
};

const struct v4l2_ioctl_ops cif_isp20_v4l2_dma_ioctlops = {
	.vidioc_reqbufs = cif_isp20_v4l2_reqbufs,
	.vidioc_querybuf = cif_isp20_v4l2_querybuf,
	.vidioc_qbuf = cif_isp20_v4l2_qbuf,
	.vidioc_dqbuf = cif_isp20_v4l2_dqbuf,
	.vidioc_streamon = cif_isp20_v4l2_streamon,
	.vidioc_streamoff = cif_isp20_v4l2_streamoff,
	.vidioc_s_fmt_vid_out = cif_isp20_v4l2_s_fmt,
};

static int xgold_v4l2_drv_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct cif_isp20_device *dev = NULL;
	struct video_device *vdev;

	cif_isp20_pltfrm_pr_info(NULL, "probing...\n");

	dev = cif_isp20_create(&pdev->dev);
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

	vdev = cif_isp20_v4l2_register_video_device(
		dev,
		"CIF ISP20 SP",
		V4L2_CAP_VIDEO_OVERLAY,
		CIIF_ISP20_V4L2_SP_DEV_MAJOR,
		&cif_isp20_v4l2_fops,
		&cif_isp20_v4l2_sp_ioctlops);
	if (IS_ERR(vdev)) {
		ret = PTR_ERR(vdev);
		goto err;
	}
	cif_isp20_v4l2_dev.sp_dev = vdev;

	ret = register_cifisp_device(&dev->isp_dev,
		&dev->v4l2_dev,
		dev->config.base_addr);
	if (IS_ERR_VALUE(ret))
		goto err;

	vdev = cif_isp20_v4l2_register_video_device(
		dev,
		"CIF ISP20 MP",
		V4L2_CAP_VIDEO_CAPTURE,
		CIIF_ISP20_V4L2_MP_DEV_MAJOR,
		&cif_isp20_v4l2_fops,
		&cif_isp20_v4l2_mp_ioctlops);
	if (IS_ERR(vdev)) {
		ret = PTR_ERR(vdev);
		goto err;
	}
	cif_isp20_v4l2_dev.mp_dev = vdev;

	vdev = cif_isp20_v4l2_register_video_device(
		dev,
		"CIF ISP20 DMA",
		V4L2_CAP_VIDEO_OUTPUT,
		CIIF_ISP20_V4L2_DMA_DEV_MAJOR,
		&cif_isp20_v4l2_fops,
		&cif_isp20_v4l2_dma_ioctlops);
	if (IS_ERR(vdev)) {
		ret = PTR_ERR(vdev);
		goto err;
	}
	cif_isp20_v4l2_dev.dma_dev = vdev;

	return 0;
err:
	cif_isp20_destroy(dev);
	return ret;
}

/* ======================================================================== */

static int xgold_v4l2_drv_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct cif_isp20_device *cif_isp20_dev =
		(struct cif_isp20_device *)platform_get_drvdata(pdev);

	if (IS_ERR_VALUE(cif_isp20_release(cif_isp20_dev,
		CIF_ISP20_ALL_STREAMS)))
		cif_isp20_pltfrm_pr_warn(cif_isp20_dev->dev,
			"CIF power off failed\n");
	return ret;
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
	/* TO DO */
}

late_initcall(xgold_v4l2_init);
module_exit(xgold_v4l2_exit);


