/*
 * -------------------------------------------------------------------------
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/pinctrl/consumer.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>

#include <linux/time.h>
#include <linux/wakelock.h>

#include "fan54x_charger.h"
#include <linux/power/charger_debug.h>


static int fan54x_i2c_read_reg(
			struct i2c_client *client, u8 reg_addr, u8 *data);

static int fan54x_i2c_write_reg(
			struct i2c_client *client, u8 reg_addr, u8 data);


/**
 * fan54x_i2c_read_reg - function for reading fan54x registers
 * @client		[in] pointer i2c client structure
 * @reg_addr		[in] register address
 * @data		[out] value read from register
 *
 * Returns '0' on success
 */
static int fan54x_i2c_read_reg(struct i2c_client *client, u8 reg_addr,
								u8 *data)
{
	int ret, cnt = MAX_NR_OF_I2C_RETRIES;
	struct i2c_msg msgs[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 1,
			.buf    = &reg_addr,
		},
		{
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 1,
			.buf    = data,
		},
	};

	do {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2) {
			CHARGER_DEBUG_READ_REG(chrgr_dbg, reg_addr, *data);
			return 0;
		}
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_READ_ERROR, ret,
								reg_addr);
	} while (cnt--);

	return ret;
}

/**
 * fan54x_i2c_write_reg - function for writing to fan54x registers
 * @client		[in] pointer i2c client structure
 * @reg_addr		[in] register address
 * @data		[in] value to be written to register
 *
 * Returns '0' on success.
 */
static int fan54x_i2c_write_reg(struct i2c_client *client, u8 reg_addr,
								u8 data)
{
	int ret, cnt = MAX_NR_OF_I2C_RETRIES;
	u8 out_buf[2];
	struct i2c_msg msgs[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = out_buf,
		},
	};
	out_buf[0] = reg_addr;
	out_buf[1] = data;

	do {
		ret = i2c_transfer(client->adapter, msgs, 1);
		CHARGER_DEBUG_WRITE_REG(chrgr_dbg, reg_addr, data);
		if (ret == 1)
			return 0;

		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_WRITE_ERROR, ret,
								reg_addr);
	} while (cnt--);

	return ret;
}

int fan54x_i2c_update_reg(struct i2c_client *client, u8 reg_addr,
					u8 mask, int shift, u8 data)
{
	int ret;
	u8 val;

	ret = fan54x_i2c_read_reg(client, reg_addr, &val);
	if (ret)
		return ret;

	val &= ~(mask << shift);
	val |= ((data & mask) << shift);

	if (reg_addr == 0x4)
		val &= ~BIT(7);

	ret = fan54x_i2c_write_reg(client, reg_addr, val);

	return ret;
}

int fan54x_attr_write(struct i2c_client *client,
				int attr,
				u8 val)
{
	int ret;
	struct fan54x_charger *chrgr = i2c_get_clientdata(client);
	struct charger_attrmap *map = chrgr->attrmap + attr;

	if (!map || !map->rpt)
		return -EINVAL;

	if (map->type == BITS)
		ret = fan54x_i2c_update_reg(client, map->reg_addr,
					map->mask, map->shift, val);
	else
		ret = fan54x_i2c_write_reg(client, map->reg_addr, val);

	return ret;
}

int fan54x_attr_read(struct i2c_client *client,
				int attr,
				u8 *val)
{
	int ret;
	struct fan54x_charger *chrgr = i2c_get_clientdata(client);
	struct charger_attrmap *map = chrgr->attrmap + attr;

	if (!map || !map->rpt)
		return -EINVAL;

	ret = fan54x_i2c_read_reg(client, map->reg_addr, val);
	if (ret)
		return ret;

	if (map->type == BITS)
		*val = (*val >> map->shift) & map->mask;

	return 0;
}

