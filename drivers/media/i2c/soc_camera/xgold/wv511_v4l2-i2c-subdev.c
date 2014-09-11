/*
 * drivers/media/i2c/soc_camera/xgold/ad5820.c
 *
 * AD5820 auto focus controller driver
 *
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Note:
 *    09/04/2014: initial version
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>

#define WV511_DRIVER_NAME "wv511"

/* ======================================================================== */

struct wv511_dev {
	u16 current_lens_pos;
	struct v4l2_subdev sd;
};

/* ======================================================================== */

int wv511_write_msg(
	struct i2c_client *client,
	u8 msb, u8 lsb)
{
	int ret = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retries;

	if (!client->adapter) {
		dev_err(&client->dev, "client->adapter NULL\n");
		return -ENODEV;
	}

	for (retries = 0; retries < 5; retries++) {
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 2;
		msg->buf = data;

		data[0] = msb;
		data[1] = lsb;

		ret = i2c_transfer(client->adapter, msg, 1);
		udelay(50);

		if (ret == 1)
			return 0;

		dev_dbg(&client->dev,
			"retrying I2C... %d\n", retries);
		retries++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
	}
	dev_err(&client->dev,
		"i2c write to failed with error %d\n", ret);
	return ret;
}

/* ======================================================================== */

static int wv511_g_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct wv511_dev *dev = container_of(sd, struct wv511_dev, sd);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		ctrl->value = dev->current_lens_pos;
		dev_dbg(&client->dev,
			"V4L2_CID_FOCUS_ABSOLUTE %d\n", ctrl->value);
		return 0;
	}

	return -EINVAL;
}

/* ======================================================================== */

static int wv511_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int ret;
	u8 lsb;
	u8 msb;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct wv511_dev *dev = container_of(sd, struct wv511_dev, sd);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		dev_dbg(&client->dev,
			"V4L2_CID_FOCUS_ABSOLUTE %d\n", ctrl->value);
		if (ctrl->value > 1023) {
			dev_err(&client->dev,
				"value out of range, must be in [0..1023]\n");
			ret = -ERANGE;
			goto err;
		}
		dev->current_lens_pos = ctrl->value;
		msb = (dev->current_lens_pos >> 4) & 0xff;
		lsb = ((dev->current_lens_pos << 4) | 0x0) & 0xff;

		ret = wv511_write_msg(client, msb, lsb);
		if (IS_ERR_VALUE(ret))
			goto err;
	} else {
		dev_dbg(&client->dev,
			"ctrl ID %d not supported\n", ctrl->id);
		return -EINVAL;
	}

	return 0;
err:
	dev_err(&client->dev,
		"failed with error %d\n", ret);
	return ret;
}

/* ======================================================================== */

static struct v4l2_subdev_core_ops wv511_core_ops = {
	.g_ctrl = wv511_g_ctrl,
	.s_ctrl = wv511_s_ctrl,
};

static struct v4l2_subdev_ops wv511_ops = {
	.core = &wv511_core_ops,
};

static int __init wv511_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct wv511_dev *dev;
	dev_info(&client->dev, "probing...\n");

	dev = devm_kzalloc(&client->dev, sizeof(struct wv511_dev), GFP_KERNEL);
	if (NULL == dev) {
		dev_info(&client->dev, "memory allocation failed\n");
		return -ENOMEM;
	}
	v4l2_i2c_subdev_init(&dev->sd, client, &wv511_ops);
	dev_info(&client->dev, "probing successful\n");

	return 0;
}

/* ======================================================================== */

static int __exit wv511_remove(
	struct i2c_client *client)
{
	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id wv511_id[] = {
	{ WV511_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id wv511_of_match[] = {
	{.compatible = "whitus vision," WV511_DRIVER_NAME "-v4l2-i2c-subdev"}
};

MODULE_DEVICE_TABLE(i2c, wv511_id);

static struct i2c_driver wv511_i2c_driver = {
	.driver = {
		.name = WV511_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = wv511_of_match
	},
	.probe = wv511_probe,
	.remove = __exit_p(wv511_remove),
	.id_table = wv511_id,
};

module_i2c_driver(wv511_i2c_driver);

MODULE_DESCRIPTION("WV511 auto focus controller driver");
MODULE_AUTHOR("Eike Grimpe");
MODULE_LICENSE("GPL");

