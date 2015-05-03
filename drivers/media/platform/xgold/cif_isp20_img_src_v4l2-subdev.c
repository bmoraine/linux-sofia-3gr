/*
 ****************************************************************
 *
 *  Intel CIF ISP 2.0 driver - Image source implementation based on
 *  V4L2-subdev.
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

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-controls_intel.h>
#include "cif_isp20.h"
#include <media/v4l2-controls_intel.h>

/* ===================== */
/* Image Source */
/* ===================== */
void *cif_isp20_img_src_v4l2_i2c_subdev_to_img_src(
	struct device *dev)
{
	int ret = 0;
	struct i2c_client *client;

	client = i2c_verify_client(dev);
	if (IS_ERR_OR_NULL(client)) {
		cif_isp20_pltfrm_pr_err(dev,
			"not an I2C device\n");
		ret = -EINVAL;
		goto err;
	}

	return i2c_get_clientdata(client);
err:
	cif_isp20_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	return ERR_PTR(ret);
}

static enum cif_isp20_pix_fmt img_src_v4l2_subdev_pix_fmt2cif_isp20_pix_fmt(
	int img_src_pix_fmt)
{
	switch (img_src_pix_fmt) {
	case V4L2_MBUS_FMT_YUYV8_1_5X8:
	case V4L2_MBUS_FMT_YUYV8_2X8:
	case V4L2_MBUS_FMT_YUYV10_2X10:
	case V4L2_MBUS_FMT_YUYV8_1X16:
	case V4L2_MBUS_FMT_YUYV10_1X20:
		return CIF_YUV422I;
	case V4L2_MBUS_FMT_UYVY8_1_5X8:
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_UYVY8_1X16:
		return CIF_UYV422I;
	case V4L2_MBUS_FMT_RGB565_2X8_BE:
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
		return CIF_RGB565;
	case V4L2_MBUS_FMT_RGB666_1X18:
		return CIF_RGB666;
	case V4L2_MBUS_FMT_RGB888_1X24:
	case V4L2_MBUS_FMT_RGB888_2X12_BE:
	case V4L2_MBUS_FMT_RGB888_2X12_LE:
		return CIF_RGB888;
	case V4L2_MBUS_FMT_SBGGR8_1X8:
		return CIF_BAYER_SBGGR8;
	case V4L2_MBUS_FMT_SGBRG8_1X8:
		return CIF_BAYER_SGBRG8;
	case V4L2_MBUS_FMT_SGRBG8_1X8:
		return CIF_BAYER_SGRBG8;
	case V4L2_MBUS_FMT_SRGGB8_1X8:
		return CIF_BAYER_SRGGB8;
	case V4L2_MBUS_FMT_SBGGR10_ALAW8_1X8:
	case V4L2_MBUS_FMT_SBGGR10_DPCM8_1X8:
	case V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_BE:
	case V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_LE:
	case V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_BE:
	case V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_LE:
	case V4L2_MBUS_FMT_SBGGR10_1X10:
		return CIF_BAYER_SBGGR10;
	case V4L2_MBUS_FMT_SGBRG10_ALAW8_1X8:
	case V4L2_MBUS_FMT_SGBRG10_DPCM8_1X8:
	case V4L2_MBUS_FMT_SGBRG10_1X10:
		return CIF_BAYER_SGBRG10;
	case V4L2_MBUS_FMT_SGRBG10_ALAW8_1X8:
	case V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8:
	case V4L2_MBUS_FMT_SGRBG10_1X10:
		return CIF_BAYER_SGRBG10;
	case V4L2_MBUS_FMT_SRGGB10_ALAW8_1X8:
	case V4L2_MBUS_FMT_SRGGB10_DPCM8_1X8:
	case V4L2_MBUS_FMT_SRGGB10_1X10:
		return CIF_BAYER_SRGGB10;
	case V4L2_MBUS_FMT_SBGGR12_1X12:
		return CIF_BAYER_SBGGR12;
	case V4L2_MBUS_FMT_SGBRG12_1X12:
		return CIF_BAYER_SGBRG12;
	case V4L2_MBUS_FMT_SGRBG12_1X12:
		return CIF_BAYER_SGRBG12;
	case V4L2_MBUS_FMT_SRGGB12_1X12:
		return CIF_BAYER_SRGGB12;
	case V4L2_MBUS_FMT_JPEG_1X8:
		return CIF_JPEG;
	default:
		return CIF_UNKNOWN_FORMAT;
	}
}

static int cif_isp20_pix_fmt2img_src_v4l2_subdev_pix_fmt(
	enum cif_isp20_pix_fmt cif_isp20_pix_fmt)
{
	switch (cif_isp20_pix_fmt) {
	case CIF_YUV422I:
		return V4L2_MBUS_FMT_YUYV8_2X8;
	case CIF_UYV422I:
		return V4L2_MBUS_FMT_UYVY8_2X8;
	case CIF_RGB565:
		return V4L2_MBUS_FMT_RGB565_2X8_LE;
	case CIF_RGB666:
		return V4L2_MBUS_FMT_RGB666_1X18;
	case CIF_RGB888:
		return V4L2_MBUS_FMT_RGB888_1X24;
	case CIF_BAYER_SBGGR8:
		return V4L2_MBUS_FMT_SBGGR8_1X8;
	case CIF_BAYER_SGBRG8:
		return V4L2_MBUS_FMT_SGBRG8_1X8;
	case CIF_BAYER_SGRBG8:
		return V4L2_MBUS_FMT_SGRBG8_1X8;
	case CIF_BAYER_SRGGB8:
		return V4L2_MBUS_FMT_SRGGB8_1X8;
	case CIF_BAYER_SBGGR10:
		return V4L2_MBUS_FMT_SBGGR10_1X10;
	case CIF_BAYER_SGBRG10:
		return V4L2_MBUS_FMT_SGBRG10_1X10;
	case CIF_BAYER_SGRBG10:
		return V4L2_MBUS_FMT_SGRBG10_1X10;
	case CIF_BAYER_SRGGB10:
		return V4L2_MBUS_FMT_SRGGB10_1X10;
	case CIF_BAYER_SBGGR12:
		return V4L2_MBUS_FMT_SBGGR12_1X12;
	case CIF_BAYER_SGBRG12:
		return V4L2_MBUS_FMT_SGBRG12_1X12;
	case CIF_BAYER_SGRBG12:
		return V4L2_MBUS_FMT_SGRBG12_1X12;
	case CIF_BAYER_SRGGB12:
		return V4L2_MBUS_FMT_SRGGB12_1X12;
	case CIF_JPEG:
		return V4L2_MBUS_FMT_JPEG_1X8;
	default:
		return -EINVAL;
	}
}

static int cif_isp20_v4l2_cid2v4l2_cid(u32 cif_isp20_cid)
{
	switch (cif_isp20_cid) {
	case CIF_ISP20_CID_FLASH_MODE:
		return V4L2_CID_FLASH_LED_MODE;
	case CIF_ISP20_CID_AUTO_GAIN:
		return V4L2_CID_AUTOGAIN;
	case CIF_ISP20_CID_AUTO_EXPOSURE:
		return V4L2_CID_EXPOSURE_AUTO;
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
	case CIF_ISP20_CID_ISO_SENSITIVITY:
		return V4L2_CID_ISO_SENSITIVITY;
	case CIF_ISP20_CID_AUTO_FPS:
		return INTEL_V4L2_CID_AUTO_FPS;
	case CIF_ISP20_CID_VBLANKING:
		return INTEL_V4L2_CID_VBLANKING;
	case CIF_ISP20_CID_HFLIP:
		return V4L2_CID_HFLIP;
	case CIF_ISP20_CID_VFLIP:
		return V4L2_CID_VFLIP;
	case CIF_ISP20_CID_3A_LOCK:
		return V4L2_CID_3A_LOCK;
	default:
		cif_isp20_pltfrm_pr_err(NULL,
			"unknown/unsupported CIF ISP20 ID %d\n",
			cif_isp20_cid);
		break;
	}
	return -EINVAL;
}

int cif_isp20_img_src_v4l2_subdev_s_streaming(
	void *img_src,
	bool enable)
{
	struct v4l2_subdev *subdev = img_src;

	if (enable)
		return v4l2_subdev_call(subdev, video, s_stream, 1);
	else
		return v4l2_subdev_call(subdev, video, s_stream, 0);
}

int cif_isp20_img_src_v4l2_subdev_s_power(
	void *img_src,
	bool on)
{
	struct v4l2_subdev *subdev = img_src;

	if (on)
		return v4l2_subdev_call(subdev, core, s_power, 1);
	else
		return v4l2_subdev_call(subdev, core, s_power, 0);
}

int cif_isp20_img_src_v4l2_subdev_enum_strm_fmts(
	void *img_src,
	u32 index,
	struct cif_isp20_strm_fmt_desc *strm_fmt_desc)
{
	int ret;
	struct v4l2_subdev *subdev = img_src;
	struct v4l2_frmivalenum fival = {.index = index};

	ret = v4l2_subdev_call(subdev, video,
		enum_frameintervals, &fival);
	if (!IS_ERR_VALUE(ret)) {
		strm_fmt_desc->discrete_intrvl = (fival.type ==
			V4L2_FRMSIZE_TYPE_DISCRETE);
		if (fival.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
			strm_fmt_desc->min_intrvl.numerator =
				fival.discrete.numerator;
			strm_fmt_desc->min_intrvl.denominator =
				fival.discrete.denominator;
		} else {
			strm_fmt_desc->min_intrvl.numerator =
				fival.stepwise.min.numerator;
			strm_fmt_desc->max_intrvl.numerator =
				fival.stepwise.max.numerator;
			strm_fmt_desc->min_intrvl.denominator =
				fival.stepwise.min.denominator;
			strm_fmt_desc->max_intrvl.denominator =
				fival.stepwise.max.denominator;
		}
		strm_fmt_desc->discrete_frmsize = true;
		strm_fmt_desc->min_frmsize.width = fival.width;
		strm_fmt_desc->min_frmsize.height = fival.height;
		strm_fmt_desc->pix_fmt =
			img_src_v4l2_subdev_pix_fmt2cif_isp20_pix_fmt(
				fival.pixel_format);
	}
	return ret;
}

int cif_isp20_img_src_v4l2_subdev_s_strm_fmt(
	void *img_src,
	struct cif_isp20_strm_fmt *strm_fmt)
{
	int ret = 0;
	struct v4l2_subdev *subdev = img_src;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_subdev_frame_interval intrvl;

	fmt.code = cif_isp20_pix_fmt2img_src_v4l2_subdev_pix_fmt(
		strm_fmt->frm_fmt.pix_fmt);
	fmt.width = strm_fmt->frm_fmt.width;
	fmt.height = strm_fmt->frm_fmt.height;
	ret = v4l2_subdev_call(subdev, video, s_mbus_fmt, &fmt);
	if (IS_ERR_VALUE(ret))
		goto err;
	intrvl.interval.numerator = strm_fmt->frm_intrvl.numerator;
	intrvl.interval.denominator = strm_fmt->frm_intrvl.denominator;
	ret = v4l2_subdev_call(subdev, video, s_frame_interval, &intrvl);
	if (IS_ERR_VALUE(ret))
		goto err;
	return 0;
err:
	pr_err("img_src.%s ERR: failed with error %d\n", __func__, ret);
	return ret;
}

int cif_isp20_img_src_v4l2_subdev_g_ctrl(
	void *img_src,
	int id,
	int *val)
{
	struct v4l2_control ctrl;
	int ret;
	struct v4l2_subdev *subdev = img_src;

	ctrl.id = cif_isp20_v4l2_cid2v4l2_cid(id);

	if (IS_ERR_VALUE(ctrl.id))
		return (int)ctrl.id;

	ret = v4l2_subdev_call(subdev, core, g_ctrl, &ctrl);
	if (!IS_ERR_VALUE(ret)) {
		if (id == CIF_ISP20_CID_FLASH_MODE) {
			if (ctrl.value == V4L2_FLASH_LED_MODE_NONE)
				ctrl.value = CIF_ISP20_FLASH_MODE_OFF;
			else if (ctrl.value == V4L2_FLASH_LED_MODE_FLASH)
				ctrl.value = CIF_ISP20_FLASH_MODE_FLASH;
			else if (ctrl.value == V4L2_FLASH_LED_MODE_TORCH)
				ctrl.value = CIF_ISP20_FLASH_MODE_TORCH;
			else {
				cif_isp20_pltfrm_pr_err(NULL,
					"unknown/unsupported value %d for control ID 0x%x\n",
					ctrl.value, id);
				return -EINVAL;
			}
		}
		*val = ctrl.value;
	}
	return ret;
}

int cif_isp20_img_src_v4l2_subdev_s_ctrl(
	void *img_src,
	int id,
	int val)
{
	struct v4l2_control ctrl;
	struct v4l2_subdev *subdev = img_src;

	ctrl.value = val;
	ctrl.id = cif_isp20_v4l2_cid2v4l2_cid(id);

	if (IS_ERR_VALUE(ctrl.id))
		return (int)ctrl.id;
	else if (id == CIF_ISP20_CID_FLASH_MODE) {
		if (val == CIF_ISP20_FLASH_MODE_OFF)
			ctrl.value = V4L2_FLASH_LED_MODE_NONE;
		else if (val == CIF_ISP20_FLASH_MODE_FLASH)
			ctrl.value = V4L2_FLASH_LED_MODE_FLASH;
		else if (val == CIF_ISP20_FLASH_MODE_TORCH)
			ctrl.value = V4L2_FLASH_LED_MODE_TORCH;
		else {
			cif_isp20_pltfrm_pr_err(NULL,
				"unknown/unsupported value %d for control ID %d\n",
				val, id);
			return -EINVAL;
		}
	}
	return v4l2_subdev_call(subdev, core, s_ctrl, &ctrl);
}

const char *cif_isp20_img_src_v4l2_subdev_g_name(
	void *img_src)
{
	struct v4l2_subdev *subdev = img_src;

	return dev_driver_string(subdev->dev);
}

int cif_isp20_img_src_v4l2_subdev_s_ext_ctrls(
	void *img_src,
	struct cif_isp20_img_src_ext_ctrl *ctrl)
{
	struct v4l2_ext_controls ctrls;
	struct v4l2_ext_control *controls;
	int i;
	int ret;
	struct v4l2_subdev *subdev = img_src;

	if (ctrl->cnt == 0)
		return -EINVAL;

	controls = kmalloc(ctrl->cnt * sizeof(struct v4l2_ext_control),
		GFP_KERNEL);

	if (!controls)
		return -ENOMEM;

	for (i = 0; i < ctrl->cnt; i++) {
		controls[i].id = ctrl->ctrls[i].id;
		controls[i].value = ctrl->ctrls[i].val;
	}

	ctrls.count = ctrl->cnt;
	ctrls.controls = controls;
	ctrls.ctrl_class = ctrl->class;
	ctrls.reserved[0] = 0;
	ctrls.reserved[1] = 0;

	ret = v4l2_subdev_call(subdev,
		core, s_ext_ctrls, &ctrls);

	kfree(controls);

	return ret;
}

long cif_isp20_img_src_v4l2_subdev_ioctl(
	void *img_src,
	unsigned int cmd,
	void *arg)
{
	struct v4l2_subdev *subdev = img_src;

	if (cmd == INTEL_VIDIOC_SENSOR_MODE_DATA) {
		long ret;
		ret = v4l2_subdev_call(subdev,
			core,
			ioctl,
			INTEL_VIDIOC_SENSOR_MODE_DATA,
			arg);

		if (IS_ERR_VALUE(ret))
			pr_err("img_src.%s subdev call failed with error %ld\n",
			__func__, ret);

		return ret;
	} else {
		return -EINVAL;
	}
}
