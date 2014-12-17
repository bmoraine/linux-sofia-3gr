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

#define AD5823_DRIVER_NAME "ad5823"

#define AD5823_RESET_ADDR 0x0
#define AD5823_MODE_ADDR 0x2
#define AD5823_VCM_DAC_MSB_ADDR 0x4
#define AD5823_VCM_DAC_LSB_ADDR 0x5

/* ======================================================================== */

int ad5823_read_reg(
	struct i2c_client *client,
	u8 reg,
	u8 *val)
{
	int ret = 0;
	struct i2c_msg msg[1];
	u8 data[4] = { 0, 0, 0, 0 };

	if (!client->adapter) {
		dev_err(&client->dev, "client->adapter NULL\n");
		return -ENODEV;
	}

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	/* High byte goes out first */
	data[0] = (u8) (reg & 0xff);

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret >= 0) {
		mdelay(3);
		msg->flags = I2C_M_RD;
		msg->len = 1;
		ret = i2c_transfer(client->adapter, msg, 1);
	}
	if (ret >= 0) {
		*val = data[0];
		return 0;
	}
	dev_err(&client->dev,
		"i2c read from offset 0x%08x failed with error %d\n", reg, ret);
	return ret;
}

/* ======================================================================== */

int ad5823_write_reg(
	struct i2c_client *client,
	u8 reg, u8 val)
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

		data[0] = reg;
		data[1] = val;

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
		"i2c write to offset 0x%08x failed with error %d\n", reg, ret);
	return ret;
}

/* ======================================================================== */

static int ad5823_g_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int ret;
	u8 val;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		ret = ad5823_read_reg(client,
			AD5823_VCM_DAC_MSB_ADDR, &val);
		if (!IS_ERR_VALUE(ret)) {
			ctrl->value = (u16)(val & 0x3) << 8;
			ret = ad5823_read_reg(client,
				AD5823_VCM_DAC_LSB_ADDR, &val);
			ctrl->value |= (val & 0xff);
		}
		dev_dbg(&client->dev,
			"V4L2_CID_FOCUS_ABSOLUTE %d\n", ctrl->value);
		return ret;
	}

	return -EINVAL;
}

/* ======================================================================== */

static int ad5823_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int ret;
	u8 val;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		dev_dbg(&client->dev,
			"V4L2_CID_FOCUS_ABSOLUTE %d\n", ctrl->value);
		if (ctrl->value > 1023) {
			dev_err(&client->dev,
				"value out of range, must be in [0..1023]\n");
			ret = -ERANGE;
			goto err;
		}
		ret = ad5823_read_reg(client,
			AD5823_VCM_DAC_MSB_ADDR, &val);
		if (IS_ERR_VALUE(ret))
			goto err;
		val = (val & 0xc) | ((ctrl->value >> 8) & 0x3);
		ret = ad5823_write_reg(client,
			AD5823_VCM_DAC_MSB_ADDR, val);
		if (IS_ERR_VALUE(ret))
			goto err;
		ret = ad5823_write_reg(client,
			AD5823_VCM_DAC_LSB_ADDR,
			ctrl->value & 0xff);
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

static struct v4l2_subdev_core_ops ad5823_core_ops = {
	.g_ctrl = ad5823_g_ctrl,
	.s_ctrl = ad5823_s_ctrl,
};

static struct v4l2_subdev_ops ad5823_ops = {
	.core = &ad5823_core_ops,
};

static int __init ad5823_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	dev_info(&client->dev, "probing...\n");

	sd = devm_kzalloc(&client->dev, sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;
	v4l2_i2c_subdev_init(sd, client, &ad5823_ops);

	dev_info(&client->dev, "probing successful\n");

	return 0;
}

/* ======================================================================== */

static int __exit ad5823_remove(
	struct i2c_client *client)
{
	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id ad5823_id[] = {
	{ AD5823_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id ad5823_of_match[] = {
	{.compatible = "analog devices," AD5823_DRIVER_NAME "-v4l2-i2c-subdev"}
};

MODULE_DEVICE_TABLE(i2c, ad5823_id);

static struct i2c_driver ad5823_i2c_driver = {
	.driver = {
		.name = AD5823_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ad5823_of_match
	},
	.probe = ad5823_probe,
	.remove = __exit_p(ad5823_remove),
	.id_table = ad5823_id,
};

module_i2c_driver(ad5823_i2c_driver);

MODULE_DESCRIPTION("AD5823 auto focus controller driver");
MODULE_AUTHOR("Eike Grimpe");
MODULE_LICENSE("GPL");

