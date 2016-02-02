/*
 ****************************************************************
 *
 * Intel CIF ISP 2.0 driver - Image source interface
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

#ifndef _CIF_ISP20_IMG_SRC_H
#define _CIF_ISP20_IMG_SRC_H

struct cif_isp20_img_src;
struct cif_isp20_strm_fmt_desc;
struct cif_isp20_strm_fmt;
struct cif_isp20_csi_config;
enum cif_isp20_pix_fmt;

struct cif_isp20_img_src;

struct cif_isp20_img_src_ctrl {
	unsigned int id;
	int val;
};

struct cif_isp20_img_src_ext_ctrl {
	int cnt;
	unsigned int class;
	struct cif_isp20_img_src_ctrl *ctrls;
};

struct cif_isp20_img_src *cif_isp20_img_src_to_img_src(
	CIF_ISP20_PLTFRM_DEVICE img_src_dev);

int cif_isp20_img_src_s_streaming(
	struct cif_isp20_img_src *img_src,
	bool enable);

int cif_isp20_img_src_s_power(
	struct cif_isp20_img_src *img_src,
	bool on);

int cif_isp20_img_src_enum_strm_fmts(
	struct cif_isp20_img_src *img_src,
	u32 index,
	struct cif_isp20_strm_fmt_desc *strm_fmt_desc);

int cif_isp20_img_src_s_strm_fmt(
	struct cif_isp20_img_src *img_src,
	struct cif_isp20_strm_fmt *strm_fmt);

int cif_isp20_img_src_g_ctrl(
	struct cif_isp20_img_src *img_src,
	int id,
	int *val);

int cif_isp20_img_src_s_ctrl(
	struct cif_isp20_img_src *img_src,
	int id,
	int val);

const char *cif_isp20_img_src_g_name(
	struct cif_isp20_img_src *img_src);

int cif_isp20_img_src_s_ext_ctrls(
	struct cif_isp20_img_src *img_src,
	struct cif_isp20_img_src_ext_ctrl *ctrls);

long cif_isp20_img_src_ioctl(
	struct cif_isp20_img_src *img_src,
	unsigned int cmd,
	void *arg);

#endif
