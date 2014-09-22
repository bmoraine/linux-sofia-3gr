/*
 *
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-device.h>
#include <media/xgold-isp-ioctl.h>
#include <linux/platform_data/platform_camera_module.h>
#include "gc.h"

/* I2C functions */
int gc_read_reg(struct i2c_client *client, u8 len, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	unsigned char data[GC_SHORT_MAX];
	int err, i;

	if (len > GC_BYTE_MAX)
		return -EINVAL;

	memset(msg, 0 , sizeof(msg));
	memset(data, 0 , sizeof(data));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */

	data[0] = reg;

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		dev_err(&client->dev, "[gcdriver]: i2c read failed\n");
		if (err >= 0)
			err = -EIO;
		goto error;
	}

	/* high byte comes first */
	if (len == GC_8BIT) {
		*val = (u8)data[0];
	} else {
		/* 16-bit access is default when len > 1 */
		for (i = 0; i < (len >> 1); i++)
			val[i] = be16_to_cpu(data[i]);
	}

	return 0;

error:
	return err;
}

int gc_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);
	return ret == num_msg ? 0 : -EIO;

	/* END */
}

int gc_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[2] = {0};
	const u16 len = data_length + 1; /* address + data */
	/* struct v4l2_subdev *sd = i2c_get_clientdata(client); */

	/* high byte goes out first */
	data[0] = reg;
	data[1] = (u8) val;

	ret = gc_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev, "[gcdriver]: i2c write failed\n");

	/* pltfrm_camera_module_pr_info(sd,
		"%s %d %d: %d\n", client->name, reg, val, ret); */
	return ret;
}

/* __verify_register_write
 *	Verify that register write is properly updated.
 *	GC seems to require somewhere around 4ms for
 *	write to register to be reflected back when it
 *	is read out.
 */
int __verify_register_write(struct i2c_client *client,
			u8 reg, u8 expect_val)
{
	int count = 0;
	int err = 0;
	u8 new_val;

	while (1) {
		count++;
		mdelay(4);
		err = gc_read_reg(client, GC_8BIT, reg, &new_val);

		if (err)
			break;

		/* Success */
		if (new_val == expect_val)
			break;

		if (count >= GC_REG_UPDATE_RETRY_LIMIT) {
			dev_err(&client->dev, "Register update failed for GC register!!\n");
			err = -ENODEV;
			break;
		}
	}

	return err;
}


int gc_write_reg_array(struct i2c_client *client,
			const struct gc_register *reglist)
{
	const struct gc_register *next = reglist;
	u8 tmp_val;

	int err;

	for (; next->type != GC_TOK_TERM; next++) {

		switch (next->type) {
		case GC_8BIT:
			err = gc_write_reg(client,
				GC_8BIT, next->sreg, (u16) next->val);
			break;

		case GC_8BIT_RMW_AND:
			err = gc_read_reg(client,
				GC_8BIT, next->sreg, &tmp_val);

			if (err)
				break;

			tmp_val = tmp_val & next->val;

			err = gc_write_reg(client,
				GC_8BIT, next->sreg, tmp_val);

			if (err)
				break;

			err = __verify_register_write(client,
				next->sreg, tmp_val);

			break;

		case GC_8BIT_RMW_OR:
			err = gc_read_reg(client,
				GC_8BIT, next->sreg, &tmp_val);

			if (err)
				break;

			tmp_val = tmp_val | next->val;

			err = gc_write_reg(client,
				GC_8BIT, next->sreg, tmp_val);

			if (err)
				break;

			err = __verify_register_write(client,
				next->sreg, tmp_val);

			break;

		default:
			break;

		}

	}

	return 0;
}


/* End of I2C functions */

/************************************************************/

/* Helper function */
struct gc_table_info *__get_register_table_info(struct gc_device *dev,
					enum gc_setting_enum setting_id)
{
	int i;

	for (i = 0; i < GC_NUM_SETTINGS; i++) {
		if (dev->product_info->settings_tables[i].setting_id ==
			setting_id)
			return &dev->product_info->settings_tables[i];
	}

	return NULL;
}

int __gc_program_ctrl_table(struct v4l2_subdev *sd,
			enum gc_setting_enum setting_id, s32 value)
{
	struct gc_device *dev = to_gc_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc_table_info *table_info;
	int ret;
	int i;

	table_info = __get_register_table_info(dev, setting_id);

	if (!table_info)
		return -EINVAL;

	for (i = 0; i < table_info->num_tables; i++) {
		if (table_info->tables[i].lookup_value == value) {
			ret = gc_write_reg_array(client,
				table_info->tables[i].reg_table);
			break;
		}
	}

	return 0;
}

long gc_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	/* TODO: add support for debug register read/write ioctls */
	return 0;
}

int gc_detect(struct i2c_client *client, u16 *id)
{
	struct i2c_adapter *adapter = client->adapter;
	u8 id_l, id_h;

	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;


	/* check sensor chip ID	 */
	if (gc_read_reg(client, GC_8BIT, GC_REG_SENSOR_ID_HIGH_BIT, &id_h))
		return -ENODEV;


	*id = (u16)(id_h << 0x8);

	if (gc_read_reg(client, GC_8BIT, GC_REG_SENSOR_ID_LOW_BIT, &id_l))
		return -ENODEV;

	*id = *id + id_l;

	return 0;
}

int gc_set_streaming(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct gc_device *dev = to_gc_sensor(sd);

	pltfrm_camera_module_pr_info(sd, "power:%d, streaming:%d, on:%d\n",
				dev->power, dev->streaming, enable);

	if ((0 == dev->power) && (1 == dev->streaming)) {
		pltfrm_camera_module_pr_err(sd,
			"gc2155 can't stream on when powered off.\n");
		return -EINVAL;
	}

	if (enable) {
		if (0 == dev->streaming) {
			pltfrm_camera_module_pr_info(sd,
				"Started writing stream-on regs.\n");
			ret = __gc_program_ctrl_table(sd, GC_SETTING_STREAM, 1);
			dev->streaming = 1;
			pltfrm_camera_module_pr_info(sd,
				"Writing stream-on regs done.\n");

		} else {
			ret = 0;
		}
	} else {
		if (0 == dev->streaming) {
			ret = 0;
		} else {
			pltfrm_camera_module_pr_info(sd,
				"Started writing stream-off regs.\n");
			ret = __gc_program_ctrl_table(sd, GC_SETTING_STREAM, 0);
			dev->streaming = 0;
			pltfrm_camera_module_pr_info(sd,
				"Writing stream-off regs done.\n");
		}
	}

	return ret;

}

int gc_init_common(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc_device *dev = to_gc_sensor(sd);
	int ret = 0;

	if (!dev->product_info)
		return -ENODEV;

	if (!dev->product_info->mode_info)
		return -ENODEV;

	if (!dev->product_info->mode_info->init_settings) {
		/* Not an error as some sensor might not have init sequence */
		return 0;
	}

	pltfrm_camera_module_pr_info(sd, "Started writing init_settings.\n");

	ret = gc_write_reg_array(client,
		dev->product_info->mode_info->init_settings);

	pltfrm_camera_module_pr_info(sd, "Writing init_settings done.\n");

	return 0;
}

int get_resolution_index(struct v4l2_subdev *sd, int w, int h)
{
	int i;
	struct gc_device *dev = to_gc_sensor(sd);

	for (i = 0; i < dev->entries_curr_table; i++) {
		if (w != dev->curr_res_table[i].width)
			continue;
		if (h != dev->curr_res_table[i].height)
			continue;

		return i;
	}

	return -1;
}


int gc_try_mbus_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	struct gc_device *dev = to_gc_sensor(sd);
	int idx = 0;
	const struct gc_resolution *tmp_res = NULL;

	mutex_lock(&dev->input_lock);

	if ((fmt->width > dev->product_info->max_res[0].res_max_width) ||
		(fmt->height > dev->product_info->max_res[0].res_max_height)) {
		fmt->width =  dev->product_info->max_res[0].res_max_width;
		fmt->height = dev->product_info->max_res[0].res_max_height;

	} else {
		idx = get_resolution_index(sd, fmt->width, fmt->height);

		if (idx < 0) {
			for (idx = 0; idx < dev->entries_curr_table; idx++) {
				tmp_res = &dev->curr_res_table[idx];
				if ((tmp_res->width >= fmt->width) &&
					(tmp_res->height >= fmt->height))
					break;
			 }
		}

		if (idx == dev->entries_curr_table)
			idx = dev->entries_curr_table - 1;

		fmt->width = dev->curr_res_table[idx].width;
		fmt->height = dev->curr_res_table[idx].height;

	}

	mutex_unlock(&dev->input_lock);

	return 0;
}

/* Find the product specific info about the format */
int gc_find_mbus_fmt_index(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *fmt)
{
	struct gc_device *dev = to_gc_sensor(sd);
	int i;

	for (i = 0; i < dev->product_info->num_mbus_formats; i++) {
		if (dev->product_info->mbus_formats[i].code == fmt->code)
			return i;
	}

	return 0;
}

int gc_s_mbus_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	struct gc_device *dev = to_gc_sensor(sd);
	const struct gc_register *gc_mode_reg_table;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	int mbus_fmt_idx;

	pltfrm_camera_module_pr_info(sd, "start\n");
	pltfrm_camera_module_pr_info(sd,
		"fmt->width:%d fmt->height:%d fmt->code:0x%x\n",
		fmt->width, fmt->height, fmt->code);

	ret = gc_try_mbus_fmt(sd, fmt);
	if (ret) {
		pltfrm_camera_module_pr_info(sd, "gc_try_mbus_fmt: %d\n", ret);
		return ret;
	}

	mutex_lock(&dev->input_lock);

	dev->fmt_idx = get_resolution_index(sd, fmt->width, fmt->height);
	/* Sanity check */
	if (unlikely(dev->fmt_idx == -1)) {
		pltfrm_camera_module_pr_info(sd,
			"get_resolution_index: %d\n", dev->fmt_idx);
		ret = -EINVAL;
		goto out;
	}

	gc_mode_reg_table = dev->curr_res_table[dev->fmt_idx].regs;

	pltfrm_camera_module_pr_info(sd, "Started writing res regs.\n");
	ret = gc_write_reg_array(client, gc_mode_reg_table);
	pltfrm_camera_module_pr_info(sd, "Writing res regs done.\n");

	if (ret)
		goto out;

	dev->fps = dev->curr_res_table[dev->fmt_idx].fps;

	dev->pixels_per_line =
		dev->curr_res_table[dev->fmt_idx].pixels_per_line;
	dev->lines_per_frame =
		dev->curr_res_table[dev->fmt_idx].lines_per_frame;

	mbus_fmt_idx = gc_find_mbus_fmt_index(sd, fmt);

	if (mbus_fmt_idx == -1) {
		ret = -EINVAL;
		goto out;
	}

	dev->format.code = fmt->code;

	gc_mode_reg_table = dev->product_info->mbus_formats[mbus_fmt_idx].regs;

	pltfrm_camera_module_pr_info(sd, "Started writing format regs.\n");
	ret = gc_write_reg_array(client, gc_mode_reg_table);
	pltfrm_camera_module_pr_info(sd, "Writing format regs done.\n");

	if (ret)
		goto out;

out:
	mutex_unlock(&dev->input_lock);

	return ret;
}

int gc_g_mbus_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	struct gc_device *dev = to_gc_sensor(sd);

	if (!fmt)
		return -EINVAL;

	fmt->width = dev->curr_res_table[dev->fmt_idx].width;
	fmt->height = dev->curr_res_table[dev->fmt_idx].height;

	fmt->code = V4L2_MBUS_FMT_UYVY8_2X8;

	return 0;
}

/*
 * gc stream on/off
 */
int gc_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct gc_device *dev = to_gc_sensor(sd);


	mutex_lock(&dev->input_lock);
	if (enable) {
		ret = gc_set_streaming(sd, 1);
		dev->streaming = 1;
	} else {
		ret = gc_set_streaming(sd, 0);
		dev->streaming = 0;
		dev->fps_index = 0;
	}
	mutex_unlock(&dev->input_lock);

	return 0;
}

int gc_enum_framesizes(struct v4l2_subdev *sd,
			struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;
	struct gc_device *dev = to_gc_sensor(sd);

	pltfrm_camera_module_pr_info(sd, "\n");


	if (index >= dev->entries_curr_table)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = dev->curr_res_table[index].width;
	fsize->discrete.height = dev->curr_res_table[index].height;
	fsize->reserved[0] = dev->curr_res_table[index].used;

	return 0;
}

int gc_enum_frameintervals(struct v4l2_subdev *sd,
			struct v4l2_frmivalenum *fival)
{
	int i;
	struct gc_device *dev = to_gc_sensor(sd);
#if 0
	const struct gc_resolution *tmp_res = NULL;
#endif

	pltfrm_camera_module_pr_info(sd,
		"fival->width: %d, fival->height:%d\n",
		fival->width, fival->height);
#if 0
	for (i = 0; i < dev->entries_curr_table; i++) {
		tmp_res = &dev->curr_res_table[i];

		if ((tmp_res->width >= fival->width) &&
			 (tmp_res->height >= fival->height))
			break;
	}

	if (i == dev->entries_curr_table)
		i--;
#endif

	if (fival->index >= dev->entries_curr_table)
		return -EINVAL;

	i = fival->index;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->width = dev->curr_res_table[i].width;
	fival->height = dev->curr_res_table[i].height;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = dev->curr_res_table[i].fps;

	/* [WR] Tell CIF this sensor's supported pixel format */
	fival->pixel_format = V4L2_MBUS_FMT_UYVY8_2X8;

	pltfrm_camera_module_pr_info(sd,
		"type: %d width: %d, height:%d, numerator:%d, denominator: %d\n",
		fival->type, fival->width, fival->height,
		fival->discrete.numerator, fival->discrete.denominator);

	return 0;
}

int gc_s_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct gc_device *dev = to_gc_sensor(sd);
	int ret = 0;

	pltfrm_camera_module_pr_info(sd, "\n");

	if ((30 != interval->interval.denominator) ||
	(1 != interval->interval.numerator)) {
		pltfrm_camera_module_pr_err(&(dev->sd),
			"invalid frame interval %d/%d\n",
			interval->interval.numerator,
			interval->interval.denominator);
		ret = -EINVAL;
		goto err;
	}

	dev->fps = interval->interval.denominator;

	return 0;
err:
	pltfrm_camera_module_pr_err(sd,
		"failed with error %d\n", ret);
	return ret;
}


int gc_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_UYVY8_2X8;

	return 0;
}

int gc_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;
	struct gc_device *dev = to_gc_sensor(sd);


	if (index >= dev->entries_curr_table)
		return -EINVAL;

	fse->min_width = dev->curr_res_table[index].width;
	fse->min_height = dev->curr_res_table[index].height;
	fse->max_width = dev->curr_res_table[index].width;
	fse->max_height = dev->curr_res_table[index].height;

	return 0;
}

int gc_get_pad_format(struct v4l2_subdev *sd,
			struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_format *fmt)
{
	struct gc_device *dev = to_gc_sensor(sd);

	switch (fmt->which) {
#if defined(CONFIG_VIDEO_V4L2_SUBDEV_API)
	case V4L2_SUBDEV_FORMAT_TRY:
		fmt->format = *v4l2_subdev_get_try_format(fh, fmt->pad);
		break;
#endif
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		fmt->format = dev->format;
	}

		return 0;
}

int gc_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_format *fmt)
{
	struct gc_device *dev = to_gc_sensor(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		dev->format = fmt->format;

	return 0;
}


int gc_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct gc_device *dev = to_gc_sensor(sd);

	if (!param || param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	dev->run_mode = param->parm.capture.capturemode;

	memset(param, 0, sizeof(*param));
	param->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

int gc_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct gc_device *dev = to_gc_sensor(sd);
	dev->run_mode = param->parm.capture.capturemode;


	mutex_lock(&dev->input_lock);
	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		dev->curr_res_table =
			dev->product_info->mode_info->res_video;
		dev->entries_curr_table =
			dev->product_info->mode_info->n_res_video;
		break;

	case CI_MODE_STILL_CAPTURE:
		dev->curr_res_table =
			dev->product_info->mode_info->res_still;
		dev->entries_curr_table =
			dev->product_info->mode_info->n_res_still;
		break;

	default:
		dev->curr_res_table =
			dev->product_info->mode_info->res_preview;
		dev->entries_curr_table =
			dev->product_info->mode_info->n_res_preview;
	}
	mutex_unlock(&dev->input_lock);

	return 0;
}

int gc_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct gc_device *dev = to_gc_sensor(sd);

	mutex_lock(&dev->input_lock);
	*frames = dev->curr_res_table[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);
	return 0;
}

int gc_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *control)
{
	pltfrm_camera_module_pr_info(sd, "\n");
	return 0;
}

int gc_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *control)
{
	pltfrm_camera_module_pr_info(sd, "\n");
	return 0;
}
