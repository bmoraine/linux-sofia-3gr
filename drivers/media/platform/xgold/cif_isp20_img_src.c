/*
 ****************************************************************
 *
 *  Intel CIF ISP 2.0 driver - Image source interface
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
#include "cif_isp20.h"
#include "cif_isp20_img_src_ops.h"

struct cif_isp20_img_src {
	void *img_src;
	const struct cif_isp20_img_src_ops *ops;
};

struct cif_isp20_img_src *cif_isp20_img_src_to_img_src(
	CIF_ISP20_PLTFRM_DEVICE dev)
{
	int ret = 0;
	int i;
	const char *device_type;
	struct cif_isp20_img_src *img_src;

	img_src = devm_kzalloc(dev, sizeof(*img_src), GFP_KERNEL);
	if (NULL == img_src) {
		ret = -ENOMEM;
		goto err;
	}

	device_type = cif_isp20_pltfrm_get_device_type(dev);

	img_src->ops = NULL;
	for (i = 0; i < ARRAY_SIZE(cif_isp20_img_src_ops); i++) {
		if (!strcmp(device_type, cif_isp20_img_src_ops->device_type)) {
			img_src->ops = &cif_isp20_img_src_ops[i].ops;
			break;
		}
	}
	if (NULL == img_src->ops) {
		cif_isp20_pltfrm_pr_err(NULL,
			"unsupported device type %s\n",
			device_type);
		ret = -EINVAL;
		goto err;
	}

	BUG_ON(NULL == img_src->ops->to_img_src);
	BUG_ON(NULL == img_src->ops->s_streaming);
	BUG_ON(NULL == img_src->ops->s_power);
	BUG_ON(NULL == img_src->ops->enum_strm_fmts);
	BUG_ON(NULL == img_src->ops->s_strm_fmt);
	BUG_ON(NULL == img_src->ops->g_ctrl);
	BUG_ON(NULL == img_src->ops->s_ctrl);

	img_src->img_src = img_src->ops->to_img_src(dev);
	if (IS_ERR_OR_NULL(img_src->img_src)) {
		ret = -EFAULT;
		goto err;
	}

	return img_src;
err:
	cif_isp20_pltfrm_pr_err(NULL, "failed with error %d\n",
		ret);
	if (!IS_ERR_OR_NULL(img_src))
		devm_kfree(dev, img_src);

	return ERR_PTR(ret);
}

int cif_isp20_img_src_s_streaming(
	struct cif_isp20_img_src *img_src,
	bool enable)
{
	return img_src->ops->s_streaming(img_src->img_src, enable);
}

int cif_isp20_img_src_s_power(
	struct cif_isp20_img_src *img_src,
	bool on)
{
	return img_src->ops->s_power(img_src->img_src, on);
}

int cif_isp20_img_src_enum_strm_fmts(
	struct cif_isp20_img_src *img_src,
	u32 index,
	struct cif_isp20_strm_fmt_desc *strm_fmt_desc)
{
	return img_src->ops->enum_strm_fmts(img_src->img_src,
		index, strm_fmt_desc);
}

int cif_isp20_img_src_s_strm_fmt(
	struct cif_isp20_img_src *img_src,
	struct cif_isp20_strm_fmt *strm_fmt)
{
	if (NULL == img_src) {
		cif_isp20_pltfrm_pr_err(NULL, "img_src is NULL\n");
		return -EINVAL;
	}
	return img_src->ops->s_strm_fmt(img_src->img_src, strm_fmt);
}

int cif_isp20_img_src_g_ctrl(
	struct cif_isp20_img_src *img_src,
	int id,
	int *val)
{
	if (NULL == img_src) {
		cif_isp20_pltfrm_pr_err(NULL, "img_src is NULL\n");
		return -EINVAL;
	}
	return img_src->ops->g_ctrl(img_src->img_src, id, val);
}

int cif_isp20_img_src_s_ctrl(
	struct cif_isp20_img_src *img_src,
	int id,
	int val)
{
	if (NULL == img_src) {
		cif_isp20_pltfrm_pr_err(NULL, "img_src is NULL\n");
		return -EINVAL;
	}
	return img_src->ops->s_ctrl(img_src->img_src, id, val);
}

int cif_isp20_img_src_s_ext_ctrls(
	struct cif_isp20_img_src *img_src,
	struct cif_isp20_img_src_ext_ctrl *ctrl)
{
	if (NULL == img_src) {
		cif_isp20_pltfrm_pr_err(NULL, "img_src is NULL\n");
		return -EINVAL;
	}
	return img_src->ops->s_ext_ctrls(img_src->img_src, ctrl);
}

long cif_isp20_img_src_ioctl(
	struct cif_isp20_img_src *img_src,
	unsigned int cmd,
	void *arg)
{
	if (NULL == img_src) {
		cif_isp20_pltfrm_pr_err(NULL, "img_src is NULL\n");
		return -EINVAL;
	}
	return img_src->ops->ioctl(img_src->img_src, cmd, arg);
}

const char *cif_isp20_img_src_g_name(
	struct cif_isp20_img_src *img_src)
{
	if (NULL == img_src) {
		cif_isp20_pltfrm_pr_err(NULL, "img_src is NULL\n");
		return ERR_PTR(-EINVAL);
	}
	return img_src->ops->g_name(img_src->img_src);
}

