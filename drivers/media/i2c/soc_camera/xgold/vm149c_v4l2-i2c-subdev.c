/*
 * drivers/media/i2c/soc_camera/xgold/vm149c.c
 *
 * vm149c auto focus controller driver
 *
 * Copyright (C) 2014
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

#define VM149C_DRIVER_NAME    "vm149c"
/* ======================================================================== */

struct vm149c_dev {
	u16 current_lens_pos;
	struct v4l2_subdev sd;
};

/* ======================================================================== */

int vm149c_write_msg(
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
		usleep_range(50, 100);

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

static int vm149c_g_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vm149c_dev *dev = container_of(sd, struct vm149c_dev, sd);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		ctrl->value = dev->current_lens_pos;
		dev_dbg(&client->dev,
			"V4L2_CID_FOCUS_ABSOLUTE %d\n", ctrl->value);
		return 0;
	}

	return -EINVAL;
}

/* ======================================================================== */

static int vm149c_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int ret;
	u8 lsb;
	u8 msb;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vm149c_dev *dev = container_of(sd, struct vm149c_dev, sd);

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
		msb = (0x00U | ((dev->current_lens_pos & 0x3F0U) >> 4U));
		lsb = (((dev->current_lens_pos & 0x0FU) << 4U) | 0x04);
		ret = vm149c_write_msg(client, msb, lsb);
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

static struct v4l2_subdev_core_ops vm149c_core_ops = {
	.g_ctrl = vm149c_g_ctrl,
	.s_ctrl = vm149c_s_ctrl,
};

static struct v4l2_subdev_ops vm149c_ops = {
	.core = &vm149c_core_ops,
};

static int vm149c_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct vm149c_dev *dev;

	dev_info(&client->dev, "probing...\n");

	dev = devm_kzalloc(&client->dev, sizeof(struct vm149c_dev), GFP_KERNEL);
	if (NULL == dev)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&dev->sd, client, &vm149c_ops);
	dev_info(&client->dev, "probing successful\n");

	return 0;
}

/* ======================================================================== */

static int __exit vm149c_remove(
	struct i2c_client *client)
{
	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id vm149c_id[] = {
	{ VM149C_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id vm149c_of_match[] = {
	{.compatible = "silicon touch,vm149c-v4l2-i2c-subdev"},
	{ }
};

MODULE_DEVICE_TABLE(i2c, vm149c_id);

static struct i2c_driver vm149c_i2c_driver = {
	.driver = {
		.name = VM149C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vm149c_of_match
	},
	.probe = vm149c_probe,
	.remove = __exit_p(vm149c_remove),
	.id_table = vm149c_id,
};

module_i2c_driver(vm149c_i2c_driver);

MODULE_DESCRIPTION("vm149c auto focus controller driver");
MODULE_AUTHOR("George");
MODULE_LICENSE("GPL");
