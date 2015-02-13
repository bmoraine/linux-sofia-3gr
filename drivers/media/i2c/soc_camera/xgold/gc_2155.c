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
#include "gc_2155.h"

#define GC_BIN_FACTOR_MAX	4
#define GC_MAX_FOCUS_POS	1023
#define GC_MAX_FOCUS_NEG	(-1023)
#define GC_VCM_SLEW_STEP_MAX	0x3f
#define GC_VCM_SLEW_TIME_MAX	0x1f
#define GC_EXPOSURE_MIN		-4
#define GC_EXPOSURE_MAX		4

static int gc2155_power_down(struct v4l2_subdev *sd)
{
	struct gc_device *dev = to_gc_sensor(sd);
	int ret = 0;

	if (dev->streaming)
		ret = gc_set_streaming(sd, 0);

	if (ret) {
		pltfrm_camera_module_pr_err(sd,
			"stop stream failed with error %d\n", ret);
		return ret;
	}

	ret = pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_PD,
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);

	msleep(20);

	ret |= pltfrm_camera_module_set_pin_state(sd,
		"intel,rst-gpio",
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
	if (ret) {
		pltfrm_camera_module_pr_err(sd,
			"pin setting failed with error %d\n", ret);
		return ret;
	}

	msleep(20);

	ret = pltfrm_camera_module_s_power(sd, 0);
	if (ret) {
		pltfrm_camera_module_pr_err(sd,
			"power-off failed with error %d\n", ret);
	}

	/*according to DS, 20ms is needed after power down*/
	msleep(20);

	return ret;
}

static int gc2155_power_up(struct v4l2_subdev *sd)
{
	int ret;

	ret = pltfrm_camera_module_s_power(sd, 1);
	if (ret) {
		pltfrm_camera_module_pr_err(sd,
			"failed with error %d\n", ret);
		goto fail_power;
	}

	msleep(20);

	ret = pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_PD,
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);

	ret |= pltfrm_camera_module_set_pin_state(sd,
		"intel,rst-gpio",
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);

	msleep(20);

	ret |= pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_PD,
		PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);

	msleep(20);

	ret |= pltfrm_camera_module_set_pin_state(sd,
		"intel,rst-gpio",
		PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);

	if (ret) {
		pltfrm_camera_module_pr_err(sd, "failed with error %d\n", ret);
		goto fail_power;
	}

	msleep(20);
	pltfrm_camera_module_pr_debug(sd, "gc2155 power-up done.\n");
	return ret;

fail_power:
	gc2155_power_down(sd);
	pltfrm_camera_module_pr_err(sd, "gc2155 power-up failed\n");

	return ret;
}

static int __gc_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc_device *dev = to_gc_sensor(sd);
	int ret = 0;

	pltfrm_camera_module_pr_debug(sd, "power:%d, streaming:%d, on:%d\n",
	dev->power, dev->streaming, on);

#ifdef WR_GC_NEVER_POWER_DOWN
	if ((1 == on) && (0 == dev->power)) {
		ret = gc2155_power_up(sd);
		ret |= gc_init_common(sd);
		if (!ret)
			dev->power = 1;
		return ret;
	}
	return 0;
#else
	if (on == 0) {
		if (0 == dev->power) {
			return 0;
		} else {
			if (0 == dev->streaming) {
				ret = gc2155_power_down(sd);
				dev->power = 0;
				return ret;
			} else {
				ret = gc_set_streaming(sd, 0);
				ret |= gc2155_power_down(sd);
				dev->power = 0;
				return ret;
			}
		}
	} else {
		if (0 == dev->power) {
			ret = gc2155_power_up(sd);
			if (!ret) {
				dev->power = 1;
				return gc_init_common(sd);
			} else {
				gc2155_power_down(sd);
				return ret;
			}
		} else {
			return 0;
		}
	}
#endif
}

int gc_s_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
	u16 sensor_id = 0x0;
	struct gc_device *dev = to_gc_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	mutex_lock(&dev->input_lock);

	ret = __gc_s_power(sd, on);
	if (ret) {
		pltfrm_camera_module_pr_err(sd,
			"Failed to power up %s: %d.\n", client->name, ret);
	}

	if (on && !ret) {
		/* config & detect sensor */
		ret = gc_detect(client, &sensor_id);
		if (ret) {
			v4l2_err(client, "gc_detect err.\n");
			__gc_s_power(sd, 0);
		} else if (dev->product_info->sensor_id != sensor_id) {
			v4l2_err(client, "sensor id didn't match expected\n");
			__gc_s_power(sd, 0);
			ret = -EINVAL;
		}

		pltfrm_camera_module_pr_info(sd, "%s Sensor ID: 0x%x.\n",
				dev->product_info->name, sensor_id);
	}

	mutex_unlock(&dev->input_lock);

	return ret;
}

static const struct v4l2_subdev_sensor_ops gc_sensor_ops = {
	.g_skip_frames = gc_g_skip_frames,
};

static const struct v4l2_subdev_video_ops gc_video_ops = {
	.s_stream = gc_s_stream,
	.enum_framesizes = gc_enum_framesizes,
	.enum_frameintervals = gc_enum_frameintervals,
	.s_frame_interval = gc_s_frame_interval,
	.try_mbus_fmt = gc_try_mbus_fmt,
	.g_mbus_fmt = gc_g_mbus_fmt,
	.s_mbus_fmt = gc_s_mbus_fmt,
	.s_parm = gc_s_parm,
	.g_parm = gc_g_parm,
};

static const struct v4l2_subdev_core_ops gc_core_ops = {
	.s_power = gc_s_power,
	.ioctl = gc_ioctl,
	.queryctrl = v4l2_subdev_queryctrl,
	.s_ctrl = gc_s_ctrl,
	.g_ctrl = gc_g_ctrl,
};

static const struct v4l2_subdev_pad_ops gc_pad_ops = {
	.enum_mbus_code = gc_enum_mbus_code,
	.enum_frame_size = gc_enum_frame_size,
	.get_fmt = gc_get_pad_format,
	.set_fmt = gc_set_pad_format,
};

static const struct v4l2_subdev_ops gc_ops = {
	.core = &gc_core_ops,
	.video = &gc_video_ops,
	.pad = &gc_pad_ops,
	.sensor = &gc_sensor_ops,
};

static int gc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct gc_device *dev;
	int ret;

	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;

	v4l2_info(client, "i2c name: %s\n", client->name);

	dev->product_info = (struct gc_product_info *)&gc2155_product_info;

	if (dev->product_info == NULL) {
		v4l2_err(client, "product_info was null\n");
		ret = -ENODEV;
		goto out_free;
	}

	/* [WR]Set default res table */
	dev->curr_res_table = dev->product_info->mode_info->res_video;
	dev->entries_curr_table = dev->product_info->mode_info->n_res_video;

	dev->streaming = 0;
	dev->power = 0;

	v4l2_i2c_subdev_init(&(dev->sd), client, &gc_ops);

	ret = pltfrm_camera_module_init(&(dev->sd), &(dev->pltfrm_data));
	if (ret) {
		pltfrm_camera_module_pr_err(&(dev->sd),
			"failed with error %d\n", ret);
		goto out_pltfrm_data_free;
	}

	pltfrm_camera_module_pr_debug(&(dev->sd), "%s %d-%04x\n",
		dev->product_info->name,
		i2c_adapter_id(client->adapter), client->addr);

	ret = gc_s_power(&dev->sd, 1);
	if (ret != 0) {
		pltfrm_camera_module_pr_err(&(dev->sd),
				"failed with error %d\n", ret);
		goto power_up_error;
	}

	/* Query device specifics */
	ret = query_device_specifics(dev);
	if (ret != 0) {
		pltfrm_camera_module_pr_err(&(dev->sd),
				"failed with error %d\n", ret);
		goto power_up_error;
	}

	gc_s_power(&dev->sd, 0);
#if 0
	/* Initialize v4l2 control handler */
	ret = __gc_init_ctrl_handler(dev);
	if (ret)
		goto out_ctrl_handler_free;

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = V4L2_MBUS_FMT_UYVY8_2X8;
#if defined(CONFIG_MEDIA_CONTROLLER)
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
#endif
	if (ret)
		gc_remove(client);
#endif

	return ret;
#if 0
out_ctrl_handler_free:
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
#endif
power_up_error:
	pltfrm_camera_module_release(&dev->sd);
out_pltfrm_data_free:
	devm_kfree(&(client->dev), dev->pltfrm_data);

out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);

	return ret;
}

static struct of_device_id gc_of_match[] = {
	{.compatible = "galaxycore," GC2155_NAME "-v4l2-i2c-subdev"},
};


static const struct i2c_device_id gc_ids[] = {
	{GC2155_NAME, (kernel_ulong_t)&gc2155_product_info },
};

MODULE_DEVICE_TABLE(i2c, gc_ids);

static struct i2c_driver gc_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = GC2155_NAME,
		.of_match_table = gc_of_match
	},
	.probe = gc_probe,
	.remove = __exit_p(gc_remove),
	.id_table = gc_ids,
};

static __init int init_gc(void)
{
	int rc;

	rc = i2c_add_driver(&gc_i2c_driver);

	return rc;
}

static __exit void exit_gc(void)
{
	i2c_del_driver(&gc_i2c_driver);
}

module_init(init_gc);
module_exit(exit_gc);


MODULE_DESCRIPTION("Class driver for GalaxyCore sensors");
MODULE_AUTHOR("Shamim Begum <shamim.begum@intel.com>");
MODULE_LICENSE("GPL");
