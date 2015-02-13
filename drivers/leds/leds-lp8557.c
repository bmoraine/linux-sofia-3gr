/*
 * TI LP8557 6 channel LED Driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * Author: Milo(Woogyom) Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_data/leds-lp55xx.h>
#include <linux/init.h>
#include <linux/slab.h>
#include "leds-lp55xx-common.h"

#define LP8557_REG_COMMAND	0x00
#define LP8557_REG_STATUS	0x01
#define LP8557_REG_BRTLO	0x03
#define LP8557_REG_BRTHI	0x04
#define LP8557_REG_CONFIG	0x10
#define LP8557_REG_CURRENT	0x11
#define LP8557_REG_PGEN	0x12
#define LP8557_REG_BOOST	0x13
#define LP8557_REG_LEDEN	0x14
#define LP8557_REG_STEP	0x15

static int send_i2c_command(struct i2c_client *client, int addr, int value)
{
	int ret;
	int i = 0;
	for (i = 0; i < 3; i++) {
		ret = i2c_smbus_write_byte_data(client, addr, value);
		if (ret < 0)
			continue;
		else
			return 0;
	}
	return -1;
}

static int lp8557_post_init_device(struct i2c_client *client)
{
	int ret;
	ret = send_i2c_command(client, 0x00, 0x00);
	if (ret < 0)
		return -1;
	ret = send_i2c_command(client, 0x10, 0x84);
	ret = send_i2c_command(client, 0x11, 0x05);
	ret = send_i2c_command(client, 0x12, 0x2c);
	ret = send_i2c_command(client, 0x13, 0x03);
	ret = send_i2c_command(client, 0x00, 0x01);
	usleep_range(1000, 2000);
	return 0;
}

static int lp8557_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lp55xx_chip *chip;
	int ret;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);

	if (!chip)
		return -ENOMEM;


	chip->cl = client;
	mutex_init(&chip->lock);

	ret = lp8557_post_init_device(client);
	if (ret < 0)
		return -1;
	else
		return 0;
}

static int lp8557_remove(struct i2c_client *client)
{
	struct lp55xx_led *led = i2c_get_clientdata(client);
	struct lp55xx_chip *chip = led->chip;

	lp55xx_unregister_leds(led, chip);
	lp55xx_deinit_device(chip);

	return 0;
}

static int lp8557_resume(struct i2c_client *client)
{
	int ret;
	ret = i2c_smbus_write_byte_data(client, 0x00, 0x80);
	lp8557_post_init_device(client);
	return 0;
}

static const struct i2c_device_id lp8557_id[] = {
	{ "lp8557",  0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lp8557_id);


static const struct of_device_id of_lp8557_leds_match[] = {
	{ .compatible = "ti,lp8557", },
	{},
};

MODULE_DEVICE_TABLE(of, of_lp8557_leds_match);


static struct i2c_driver lp8557_driver = {
	.driver = {
		.name = "lp8557",
		.of_match_table = of_match_ptr(of_lp8557_leds_match),
	},
	.probe = lp8557_probe,
	.remove = lp8557_remove,
	.resume = lp8557_resume,
	.id_table = lp8557_id,
};
module_i2c_driver(lp8557_driver);

MODULE_DESCRIPTION("Texas Instruments LP8501 LED drvier");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL");
