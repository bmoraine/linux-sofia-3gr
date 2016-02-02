/*
 ****************************************************************
 *
 *  Intel CIF ISP 2.0 driver - Image source implementation based on
 *  V4L2-subdev.
 * Copyright (C) 2014-2015 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Note:
 *     07/07/2014: initial version.
 *
 ****************************************************************
 */

#ifndef _CIF_ISP20_IMG_SRC_V4L2_SUBDEV_H
#define _CIF_ISP20_IMG_SRC_V4L2_SUBDEV_H

#define CIF_ISP20_IMG_SRC_V4L2_I2C_SUBDEV  "v4l2-i2c-subdev"

void *cif_isp20_img_src_v4l2_i2c_subdev_to_img_src(
	CIF_ISP20_PLTFRM_DEVICE dev);

int cif_isp20_img_src_v4l2_subdev_s_streaming(
	void *img_src,
	bool enable);

int cif_isp20_img_src_v4l2_subdev_s_power(
	void *img_src,
	bool on);

int cif_isp20_img_src_v4l2_subdev_enum_strm_fmts(
	void *img_src,
	u32 index,
	struct cif_isp20_strm_fmt_desc *strm_fmt_desc);

int cif_isp20_img_src_v4l2_subdev_s_strm_fmt(
	void *img_src,
	struct cif_isp20_strm_fmt *strm_fmt);

int cif_isp20_img_src_v4l2_subdev_g_ctrl(
	void *img_src,
	int id,
	int *val);

int cif_isp20_img_src_v4l2_subdev_s_ctrl(
	void *img_src,
	int id,
	int val);

const char *cif_isp20_img_src_v4l2_subdev_g_name(
	void *img_src);

int cif_isp20_img_src_v4l2_subdev_s_ext_ctrls(
	void *img_src,
	struct cif_isp20_img_src_ext_ctrl *ctrl);

long cif_isp20_img_src_v4l2_subdev_ioctl(
	void *img_src,
	unsigned int cmd,
	void *arg);

#endif
