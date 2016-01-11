/**
 * Driver for Goodix GT911 touchscreen.
 *
 * Copyright (c) 2014 Intel Corporation
 *
 * Based on Goodix GT9xx driver:
 *	(c) 2010 - 2013 Goodix Technology.
 *	Version: 2.0
 *	Authors: andrew@goodix.com, meta@goodix.com
 *	Release Date: 2013/04/25
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/irq.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/interrupt.h>
#include <linux/input/mt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#ifdef CONFIG_PM
#include <linux/power_hal_sysfs.h>
#include <linux/device_pm_data.h>
#endif

#define GT9XX_MAX_TOUCHES		5
#define GT9XX_REG_CONFIG_DATA		0x8047
#define GT9XX_CONFIG_LENGTH		186
#define GT9XX_MAX_NUM_BUTTONS		8
#define GT9XX_FW_FILE_NAME		"gt9xx_firmware.fw"
#define GT9XX_FW_HEAD_LEN		14

#define GT9XX_SS51_BLOCK_ADDR		0xc000
#define GT9XX_DSP_BLOCK_ADDR		0xc000
#define GT9XX_SRAM_BANK_ADDR		0x4048
#define GT9XX_SW_WDT_ADDR		0x8041
#define GT9XX_MEM_CD_EN_ADDR		0x4049
#define GT9XX_CACHE_EN_ADDR		0x404B
#define GT9XX_TMR0_EN_ADDR		0x40B0
#define GT9XX_SWRST_B0_ADDR		0x4180
#define GT9XX_CPU_SWRST_PULSE_ADDR	0x4184
#define GT9XX_FW_REL_ADDR		0x4180
#define GT9XX_BOOTCTL_B0_ADDR		0x4190
#define GT9XX_BOOT_OPT_B0_ADDR		0x4218
#define GT9XX_BOOT_CTL_ADDR		0x5094
#define GT9XX_DSP_CLK_ADDR		0x4010

#define GT9X5_SS51_BLOCK_ADDR1		0xc000
#define GT9X5_SS51_BLOCK_ADDR2		0xe000
#define GT9X5_DSP_BLOCK_ADDR		0x9000
#define GT9X5_DSP_ISP_BLOCK_ADDR	0xc000
#define GT9X5_BOOT_BLOCK_ADDR		0x9000
#define GT9X5_BOOT_ISP_BLOCK_ADDR	0x9000
#define GT9X5_GFWLINK_BLOCK_ADDR	0x9000
#define GT9X5_GWAKE_BLOCK_ADDR		0x9000
#define GT9X5_DSP_ISP_BLOCK_ADDR	0xc000

#define GT9XX_SS51_SECTION_LEN		0x2000 /* total 4 sections */
#define GT9XX_DSP_SECTION_LEN		0x1000

#define GT9X5_SS51_SECTION_LEN		0x2000 /* total 4 sections */
#define GT9X5_DSP_SECTION_LEN		0x1000
#define GT9X5_DSP_ISP_SECTION_LEN	0x1000
#define GT9X5_BOOT_SECTION_LEN		0x800
#define GT9X5_BOOT_ISP_SECTION_LEN	0x800
#define GT9X5_GFWLINK_SECTION_LEN	0x2000
#define GT9X5_GFWLINK_LEN		0x3000
#define GT9X5_GWAKE_SECTION_LEN		0x2000 /* total 4 section */
#define GT9X5_GWAKE_LEN			(GT9X5_GWAKE_SECTION_LEN * 4)

#define GT9XX_SS51_START_INDEX		(0)
#define GT9XX_DSP_START_INDEX		(GT9XX_SS51_SECTION_LEN * 4)

#define GT9X5_SS51_START_INDEX		(0)
#define GT9X5_DSP_START_INDEX		(GT9XX_SS51_SECTION_LEN * 4)
#define GT9X5_BOOT_START_INDEX		(GT9XX_DSP_START_INDEX + \
					 GT9XX_DSP_SECTION_LEN)
#define GT9X5_BOOT_ISP_START_INDEX	(GT9X5_BOOT_START_INDEX + \
					 GT9X5_BOOT_SECTION_LEN)
#define GT9X5_GFWLINK_START_INDEX	(GT9X5_BOOT_ISP_START_INDEX + \
					 GT9X5_BOOT_ISP_SECTION_LEN)
#define GT9X5_GWAKE_START_INDEX		(GT9X5_GFWLINK_START_INDEX + \
					 GT9X5_GFWLINK_LEN)
#define GT9X5_DSP_ISP_START_INDEX	(GT9X5_DSP_ISP_SECTION_LEN)

#define GT9XX_SS51_BANK_INDEX0		0
#define GT9XX_SS51_BANK_INDEX1		1
#define GT9XX_DSP_BANK_INDEX0		2

#define GT9XX_FW_CHK_SIZE		1024
#define GT9XX_FW_CHK_RETRY		40
#define GT9XX_FW_DOWNLOAD_RETRY		5
#define GT9XX_FW_HOLD_RETRY		200

#define GT9XX_CFG_DN_RETRY		3

#define GT9XX_SW_WDT_DEF_VAL		0xaa
#define GT9XX_SWRST_B0_DEF_VAL		0x0c
#define GT9XX_SWRST_B0_REL_SS51_DSP	0x04

#define GT9X5_SWRST_REL_DSP_HOLD_SS51	0x04
#define GT9X5_SWRST_REL_SS51_HOLD_DSP	0x08

#define GT9X5_FLASH_PACK_LEN		256

#define GT9XX_BOOTCTL_B0_SRAM		0x02

#define GT9XX_BUTTONS_PROPERTY		"goodix,buttons"
#define GT9XX_FIRMWARE_UPDATE_PROPERTY	"goodix,firmware-update"
#define GT9XX_FW_NAME_PROPERTY		"goodix,firmware-name"
#define GT9XX_COMPAT_MODE_PROPERTY	"goodix,compat-mode"

enum gt9xx_status_bits {
	/* bits 0 .. GT9XX_MAX_TOUCHES - 1 are use to track touches */
	GT9XX_STATUS_SLEEP_BIT = GT9XX_MAX_TOUCHES,
	GT9XX_STATUS_BITS,
};

struct gt9xx_fw_head {
	u8 hw_info[4]; /* hardware info */
	u8 pid[8]; /* product id */
	u16 vid; /* vendor id */
};

struct gt9xx_ts {
	struct i2c_client *client;
	struct input_dev *input;
	char phys[32];

	struct pinctrl *pinctrl;
	struct pinctrl_state *ctrl_state;
	struct pinctrl_state *int_state;

	struct gpio_desc *gpiod_int;
	struct gpio_desc *gpiod_rst;
	int irq_type;
	int compat_mode;

	u16 max_x;
	u16 max_y;

	const char *fw_name;
	const struct firmware *fw;
	struct gt9xx_fw_head *fw_head;
	int num_buttons;
	unsigned int button_codes[GT9XX_MAX_NUM_BUTTONS];

	DECLARE_BITMAP(status, GT9XX_STATUS_BITS);

	struct device_pm_platdata *pm_platdata;
};

/* Registers define */
#define GT9XX_REG_CMD			0x8040
#define GT9XX_REG_CONFIG		0x8047
#define GT9XX_REG_ID			0x8140
#define GT9XX_REG_STATUS		0x814E
#define GT9XX_REG_DATA			0x814F

static int gt9xx_i2c_read(struct i2c_client *client, u16 addr,
			  void *buf, unsigned len)
{
	u8 addr_buf[2];
	struct i2c_msg msgs[2];
	int ret, retries = 0;

	addr_buf[0] = addr >> 8;
	addr_buf[1] = addr & 0xFF;

	msgs[0].flags = 0;
	msgs[0].addr = client->addr;
	msgs[0].buf = addr_buf;
	msgs[0].len = sizeof(addr_buf);

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].buf = buf;
	msgs[1].len = len;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}

	if (retries >= 5) {
		dev_err(&client->dev, "I2C read @0x%04X (%d) failed: %d", addr,
			len, ret);
		return ret;
	}

	return ret;

}

static int gt9xx_i2c_write(struct i2c_client *client, u16 addr, void *buf,
			  unsigned len)
{
	u8 *addr_buf;
	struct i2c_msg msg;
	int ret = 0, retries = 0;

	addr_buf = kmalloc(len + 2, GFP_KERNEL);
	if (!addr_buf)
		return -ENOMEM;

	addr_buf[0] = addr >> 8;
	addr_buf[1] = addr & 0xFF;

	memcpy(&addr_buf[2], buf, len);

	msg.flags = 0;
	msg.addr = client->addr;
	msg.buf = addr_buf;
	msg.len = len + 2;

	while (retries < 5 ) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}

	kfree(addr_buf);

	if (retries >= 5) {
		dev_err(&client->dev, "I2C write @0x%04X (%d) failed: %d", addr,
			len, ret);
		return ret;
	}

	return ret;
}

static int gt9xx_i2c_write_u8(struct i2c_client *client, u16 addr, u8 value)
{
	return gt9xx_i2c_write(client, addr, &value, sizeof(value));
}

static void gt9xx_irq_disable(struct gt9xx_ts *ts, bool no_sync)
{
	if (no_sync)
		disable_irq_nosync(ts->client->irq);
	else
		disable_irq(ts->client->irq);
	gpiod_unlock_as_irq(ts->gpiod_int);
}

static void gt9xx_irq_enable(struct gt9xx_ts *ts)
{
	gpiod_lock_as_irq(ts->gpiod_int);
	enable_irq(ts->client->irq);
}

static int gt9xx_send_cfg(struct gt9xx_ts *ts, u8 *cfg_data, size_t cfg_size)
{
	int ret, i, retry = 0;
	size_t raw_cfg_len;
	u8 check_sum = 0;

	if (cfg_size != GT9XX_CONFIG_LENGTH) {
		dev_err(&ts->client->dev, "The length of the config buffer array is not correct");
		return -EINVAL;
	}

	raw_cfg_len = cfg_size - 2;
	for (i = 0; i < raw_cfg_len; i++)
		check_sum += cfg_data[i];
	check_sum = (~check_sum) + 1;
	if (check_sum != cfg_data[raw_cfg_len]) {
		dev_err(&ts->client->dev, "The checksum of the config buffer array is not correct");
		return -EINVAL;
	}

	if (cfg_data[raw_cfg_len + 1] != 1) {
		dev_err(&ts->client->dev, "The Config_Fresh register needs to be set");
		return -EINVAL;
	}

again:
	ret = gt9xx_i2c_write(ts->client, GT9XX_REG_CONFIG_DATA,
				cfg_data, GT9XX_CONFIG_LENGTH);
	if ((ret < 0) && (retry++ < GT9XX_CFG_DN_RETRY)) {
		dev_err(&ts->client->dev, "Config send failed, retry %d",
				retry);
		goto again;
	}
	if (retry >= GT9XX_CFG_DN_RETRY) {
		dev_err(&ts->client->dev, "Config send failed, err: %d", ret);
		return ret;
	}
	dev_info(&ts->client->dev, "Config settings sent sucessfully");

	/* Let the firmware reconfigure itself, so sleep for 10ms */
	usleep_range(10000, 11000);
	return ret;
}

#define GT9XX_STATUS_REG_MASK_TOUCHES		0x0F
#define GT9XX_STATUS_REG_MASK_VALID		0x80

static irqreturn_t gt9xx_thread_handler(int irq, void *arg)
{
	struct gt9xx_ts *ts = arg;
	struct gt9xx_touch_data {
		u8 id;
		__le16 x;
		__le16 y;
		__le16 witdh;
		u8 reserved;
	} __packed data[GT9XX_MAX_TOUCHES];
	int touches;
	DECLARE_BITMAP(active_touches, GT9XX_MAX_TOUCHES);
	u8 status;
	int ret;
	int i;
	int retries = 5;

	while (--retries) {
		ret = gt9xx_i2c_read(ts->client, GT9XX_REG_STATUS, &status, 1);
		if (ret <= 0)
			goto out;

		if (status & GT9XX_STATUS_REG_MASK_VALID)
			break;

		/*
		 * If we reach this place, it means the interrupt may be out of
		 * sync with GT9XX_STATUS_REG_MASK_VALID bit. In other words,
		 * the interrupt event is coming too early while the buffer of
		 * events is not ready yet.
		 * Our tests show this bit will (almost) always be set if user
		 * is still pressing screen and we wait a bit. So we'll wait
		 * and retry few times before discard the buffer for good.
		 */
		usleep_range(500, 1000);
	}

	if (!retries) {
		/* GT9XX_STATUS_REG_MASK_VALID was never set. Bad data? */
		dev_dbg(&ts->client->dev, "buffer status was never set\n");
		goto out;
	}

	touches = status & GT9XX_STATUS_REG_MASK_TOUCHES;
	if (touches > GT9XX_MAX_TOUCHES) {
		dev_err(&ts->client->dev, "invalid number of touches");
		goto out;
	}

	if (touches) {
		int len = touches * sizeof(struct gt9xx_touch_data);

		ret = gt9xx_i2c_read(ts->client, GT9XX_REG_DATA, data, len);
		if (ret <= 0)
			goto out;
	}

	if (ts->num_buttons > 0) {
		u8 key_values;
		int offset = GT9XX_REG_DATA +
				touches * sizeof(struct gt9xx_touch_data);
		int i;
		ret = gt9xx_i2c_read(ts->client, offset,
					&key_values, sizeof(key_values));
		if (ret <= 0)
			goto out;

		for (i = 0; i < ts->num_buttons; i++) {
			int value = key_values & (1 << i) ? 1 : 0;
			input_report_key(ts->input,
					ts->button_codes[i], value);
		}
	}

	bitmap_clear(active_touches, 0, GT9XX_MAX_TOUCHES);

	input_report_key(ts->input, BTN_TOUCH, touches);

	/* generate touch down events */
	for (i = 0; i < touches; i++) {
		int id = data[i].id;
		int x = le16_to_cpu(data[i].x);
		int y = le16_to_cpu(data[i].y);
		int w = le16_to_cpu(data[i].witdh);

		set_bit(id, active_touches);

		input_mt_slot(ts->input, id);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		input_report_abs(ts->input, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, w);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, w);
	}

	/* generate touch up events */
	for (i = 0; i < GT9XX_MAX_TOUCHES; i++) {
		if (test_bit(i, active_touches)) {
			set_bit(i, ts->status);
			continue;
		} else {
			if (!test_and_clear_bit(i, ts->status))
				continue;
		}

		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}

	input_sync(ts->input);

out:
	gt9xx_i2c_write_u8(ts->client, GT9XX_REG_STATUS, 0);

	return IRQ_HANDLED;
}

static int gt9xx_i2c_test(struct i2c_client *client)
{
	u8 test;

	return gt9xx_i2c_read(client, GT9XX_REG_CONFIG, &test, sizeof(test));
}

static int gt9xx_get_info(struct gt9xx_ts *ts)
{
	struct gt9xx_config {
		u8 version;
		__le16 max_x;
		__le16 max_y;
		u8 reserved, touch_no:4;
		u8 reserved2:2, stretch_rank:2, x2y:1, sito:1, int_trigger:2;
		u8 data[182];
	} __packed cfg;
	const int irq_table[] = {
		IRQ_TYPE_EDGE_RISING,
		IRQ_TYPE_EDGE_FALLING,
		IRQ_TYPE_LEVEL_LOW,
		IRQ_TYPE_LEVEL_HIGH,
	};
	struct {
		u8 id[4];	/* may not be NULL terminated */
		__le16 fw_version;
	} __packed id;
	char id_str[5];
	int ret;

	ret = gt9xx_i2c_read(ts->client, GT9XX_REG_ID, &id, sizeof(id));
	if (ret <= 0) {
		dev_err(&ts->client->dev, "read id failed");
		return ret;
	}

	memcpy(id_str, id.id, 4);
	id_str[4] = 0;
	if (kstrtou16(id_str, 10, &ts->input->id.product))
		ts->input->id.product = 0;
	ts->input->id.version = le16_to_cpu(id.fw_version);

	dev_info(&ts->client->dev, "version: %d_%04x", ts->input->id.product,
		 ts->input->id.version);

	ret = gt9xx_i2c_read(ts->client, GT9XX_REG_CONFIG, &cfg, sizeof(cfg));
	if (ret <= 0)
		return ret;

	ts->max_x = le16_to_cpu(cfg.max_x);
	ts->max_y = le16_to_cpu(cfg.max_y);
	ts->irq_type = irq_table[cfg.int_trigger];

	dev_info(&ts->client->dev, "max_x = %d, max_y = %d, irq_type = 0x%02x",
		 ts->max_x, ts->max_y, ts->irq_type);

	return ret;
}



static void gt9xx_int_sync(struct gt9xx_ts *ts)
{
	struct pinctrl *pinctrl = ts->pinctrl;

	gpiod_direction_output(ts->gpiod_int, 0);
	msleep(50);
	gpiod_direction_input(ts->gpiod_int);

	if (pinctrl)
		pinctrl_select_state(pinctrl, ts->int_state);
}

static void gt9xx_reset(struct gt9xx_ts *ts)
{
	struct pinctrl *pinctrl = ts->pinctrl;

	if (pinctrl)
		pinctrl_select_state(pinctrl, ts->ctrl_state);

	/* begin select I2C slave addr */
	gpiod_direction_output(ts->gpiod_rst, 0);
	msleep(20);				/* T2: > 10ms */
	/* HIGH: 0x28/0x29, LOW: 0xBA/0xBB */
	gpiod_direction_output(ts->gpiod_int, ts->client->addr == 0x14);
	msleep(2);				/* T3: > 100us */
	gpiod_direction_output(ts->gpiod_rst, 1);
	msleep(6);				/* T4: > 5ms */
	/* end select I2C slave addr */
	gpiod_direction_input(ts->gpiod_rst);

	gt9xx_int_sync(ts);
}

#ifdef CONFIG_ACPI
static int gt9xx_get_and_send_cfg(struct gt9xx_ts *ts)
{
	int ret;
	struct device *dev = &ts->client->dev;
	struct acpi_buffer buf = {ACPI_ALLOCATE_BUFFER, NULL};
	union acpi_object *out;
	acpi_handle handle;
	acpi_status err;

	handle = ACPI_HANDLE(dev);
	if (!handle) {
		dev_err(&ts->client->dev, "Cannot get ACPI handle");
		return -EINVAL;
	}

	err = acpi_evaluate_object(handle, "_DSM", NULL, &buf);
	out = buf.pointer;
	if (ACPI_FAILURE(err) || !(out->type == ACPI_TYPE_BUFFER)) {
		dev_err(&ts->client->dev, "Cannot get ACPI config buffer array");
		return -EINVAL;
	}

	ret = gt9xx_send_cfg(ts, out->buffer.pointer, out->buffer.length);
	if (ret > 0)
		ret = 0;

	kfree(out);

	return ret;
}
#else
static inline int gt9xx_get_and_send_cfg(struct gt9xx_ts *ts)
{
	u8 *config;
	int ret;

	config = kzalloc(sizeof(*config)*GT9XX_CONFIG_LENGTH, GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	ret = of_property_read_u8_array(ts->client->dev.of_node,
				"goodix,config", config, GT9XX_CONFIG_LENGTH);
	if (ret) {
		dev_err(&ts->client->dev,
			"Cannot get OF config buffer array. Err: %d", ret);
		goto out;
	}

	ret = gt9xx_send_cfg(ts, config, GT9XX_CONFIG_LENGTH);
	if (ret > 0)
		ret = 0;

out:
	kfree(config);
	return ret;
}
#endif

static int gt9xx_fw_check_and_repair(struct i2c_client *client, u16 start_addr,
				     u8 *comp_buf, u32 len)
{
	int i = 0, ret = 0, retry = 0, comp_len, index = 0;
	u8 *buf;
	bool check_failed = false;

	buf = kmalloc(GT9XX_FW_CHK_SIZE, GFP_KERNEL);

	while (index < len && retry < GT9XX_FW_CHK_RETRY) {
		comp_len = ((len - index) < GT9XX_FW_CHK_SIZE) ? (len - index) :
				GT9XX_FW_CHK_SIZE;

		ret = gt9xx_i2c_read(client, start_addr + index, buf, comp_len);
		if (ret <= 0)
			break;

		for (i = 0; i < comp_len; i++) {
			if (buf[i] != comp_buf[index + i]) {
				gt9xx_i2c_write(client, start_addr + index + i,
						&comp_buf[index + i],
						comp_len - i);
				retry++;
				check_failed = true;
				break;
			}
		}
		if (!check_failed)
			index += comp_len;

		check_failed = false;
	}

	kfree(buf);

	if ((len - index) > 0)
		return -EIO;

	return ret;
}

static int gt9xx_fw_flash_verify(struct i2c_client *client, u8 bank, u16 addr,
				 u8 *data, u32 len)
{
	int ret;

	ret = gt9xx_i2c_write_u8(client, GT9XX_SRAM_BANK_ADDR, bank);
	if (ret <= 0)
		return ret;

	ret = gt9xx_i2c_write(client, addr, data, len);
	if (ret <= 0)
		return ret;

	return gt9xx_fw_check_and_repair(client, addr, data, len);
}

static int gt9x5_flash_and_check(struct i2c_client *client,
				u16 addr, u8 *data, u32 tot_len)
{
	u16 cur_addr = addr;
	u32 cur_len = 0, pack_len = 0;
	int ret = 0, retry;
	u8 *cur_data = data;
	u8 rd_buf[GT9X5_FLASH_PACK_LEN];

	dev_dbg(&client->dev,
		"Begin flash %dk data to addr 0x%x\n", (tot_len/1024),
		cur_addr);
	while (cur_len < tot_len) {
		pack_len = tot_len - cur_len;
		dev_dbg(&client->dev, "B/T:%04d/%04d\n", cur_len, tot_len);
		if (pack_len > GT9X5_FLASH_PACK_LEN)
			pack_len = GT9X5_FLASH_PACK_LEN;
		for (retry = 0; retry < 5; retry++) {
			ret = gt9xx_i2c_write(client, cur_addr, cur_data,
					      pack_len);
			if (ret <= 0) {
				dev_err(&client->dev,
					"write packet data i2c err\n");
				continue;
			}
			ret = gt9xx_i2c_read(client, cur_addr, rd_buf,
					     pack_len);
			if (ret <= 0) {
				dev_err(&client->dev,
					"read packet data i2c err\n");
				continue;
			}
			if (memcmp(cur_data, rd_buf, pack_len)) {
				dev_err(&client->dev,
					"check pack data failed, not equal");
				continue;
			} else
				break;
		}
		if (retry >= 5) {
			dev_err(&client->dev, "flash data time out, exit\n");
			return -EINVAL;
		}
		cur_len += pack_len;
		cur_addr += pack_len;
		cur_data += pack_len;
	}

	return 1;
}

static int gt9x5_fw_flash_ss51_section(struct i2c_client *client, u8 bank,
				       u16 addr, u8 *data, u32 len)
{
	int ret, val = 0, retry = 0;

	dev_dbg(&client->dev,
		"ss51 fw flash section bank:%d addr:%04x len:%d\n",
		bank, addr, len);

	/* select and hold ss51 & dsp */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9XX_SWRST_B0_DEF_VAL);
	if (ret <= 0)
		return ret;

	/* set scramble */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_OPT_B0_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* select bank */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SRAM_BANK_ADDR,
				 (bank >> 4) & 0x0f);
	if (ret <= 0)
		return ret;

	/* enable accessing mode */
	ret = gt9xx_i2c_write_u8(client, GT9XX_MEM_CD_EN_ADDR, 1);
	if (ret <= 0)
		return ret;

	/* write the data */
	ret = gt9x5_flash_and_check(client, addr, data, len);
	if (ret <= 0)
		return ret;

	/* release dsp & hold ss51 */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9X5_SWRST_REL_DSP_HOLD_SS51);
	if (ret <= 0)
		return ret;

	/* must sleep for a millisecond */
	usleep_range(1000, 2000);

	/* send burn command to move data to flash from sram */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_CTL_ADDR, bank & 0x0f);
	if (ret <= 0)
		return ret;

	/* wait for flash burn to complete */
	do {
		ret = gt9xx_i2c_read(client, GT9XX_BOOT_CTL_ADDR, &val, 1);
		if (ret <= 0)
			return ret;
		/* wait for 10 ms, value based on reference driver*/
		usleep_range(10000, 11000);
	} while (val && retry++ < len);

	dev_dbg(&client->dev,
		"ss51 section burn command complete val:%d retry:%d len:%d\n",
		 val, retry, len);

	/* select bank */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SRAM_BANK_ADDR,
				 (bank >> 4) & 0x0f);
	if (ret <= 0)
		return ret;

	/* enable accessing mode */
	ret = gt9xx_i2c_write_u8(client, GT9XX_MEM_CD_EN_ADDR, 1);
	if (ret <= 0)
		return ret;

	ret = gt9xx_fw_check_and_repair(client, addr, data, len);
	if (ret <= 0)
		return ret;

	/* disable accessing mode */
	return gt9xx_i2c_write_u8(client, GT9XX_MEM_CD_EN_ADDR, 0);
}

static int gt9x5_fw_flash_block_ss51(struct gt9xx_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret;
	const u8 *data;
	u8 *temp_data;

	temp_data = kmalloc((GT9XX_SS51_SECTION_LEN + 1), GFP_KERNEL);

	/* move head to section 1 address*/
	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + GT9XX_SS51_START_INDEX];

	memset(temp_data, 0xff, GT9XX_SS51_SECTION_LEN);

	/* clear control flag */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_CTL_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* flash ss51 firmware section 1*/
	ret =  gt9x5_fw_flash_ss51_section(client, 1, GT9X5_SS51_BLOCK_ADDR1,
					   (u8 *)temp_data,
					   GT9XX_SS51_SECTION_LEN);
	kfree(temp_data);
	if (ret <= 0)
		return ret;

	/* move head to section 2 address */
	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + GT9XX_SS51_SECTION_LEN];

	/* flash ss51 firmware section 2*/
	ret =  gt9x5_fw_flash_ss51_section(client, 2, GT9X5_SS51_BLOCK_ADDR2,
					   (u8 *)data, GT9XX_SS51_SECTION_LEN);
	if (ret <= 0)
		return ret;

	/* move head to section 3 address */
	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + 2 * GT9XX_SS51_SECTION_LEN];

	/* flash ss51 firmware section 3*/
	ret =  gt9x5_fw_flash_ss51_section(client, 0x13, GT9X5_SS51_BLOCK_ADDR1,
					   (u8 *)data, GT9XX_SS51_SECTION_LEN);
	if (ret <= 0)
		return ret;

	/* move head to section 4 address */
	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + 3 * GT9XX_SS51_SECTION_LEN];

	/* flash ss51 firmware section 4*/
	ret =  gt9x5_fw_flash_ss51_section(client, 0x14, GT9X5_SS51_BLOCK_ADDR2,
					   (u8 *)data, GT9XX_SS51_SECTION_LEN);

	dev_info(&ts->client->dev, "ss51 fw flash sucess\n");

	return ret;
}

static int gt9x5_fw_flash_block_dsp(struct gt9xx_ts *ts)
{
	struct i2c_client *client = ts->client;
	const u8 *data;
	int val = 0, ret, retry = 0;

	/* move head to dsp section 1 address*/
	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + GT9XX_DSP_START_INDEX];

	/* select bank 3*/
	ret = gt9xx_i2c_write_u8(client, GT9XX_SRAM_BANK_ADDR, 0x03);
	if (ret <= 0)
		return ret;

	/* select and hold ss51 & dsp */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9XX_SWRST_B0_DEF_VAL);
	if (ret <= 0)
		return ret;

	/* set scramble */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_OPT_B0_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* hold ss51 & release dsp */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9X5_SWRST_REL_DSP_HOLD_SS51);
	if (ret <= 0)
		return ret;

	/* must sleep for a millisecond */
	usleep_range(1000, 2000);

	/* write the data & verify */
	ret = gt9x5_flash_and_check(client, GT9X5_DSP_BLOCK_ADDR, (u8 *)data,
			      GT9XX_DSP_SECTION_LEN);
	if (ret <= 0)
		return ret;

	/* send burn command to move data to flash from sram */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_CTL_ADDR, 0x05);
	if (ret <= 0)
		return ret;

	/* wait for flash burn to complete */
	do {
		ret = gt9xx_i2c_read(client, GT9XX_BOOT_CTL_ADDR, &val, 1);
		if (ret <= 0)
			return ret;
		/* wait for 10 ms, value based on reference driver*/
		usleep_range(10000, 11000);
	} while (val && retry++ < GT9XX_DSP_SECTION_LEN);

	dev_dbg(&client->dev,
		"dsp section flash send burn command done %d %d %d\n",
		val, retry, GT9XX_DSP_SECTION_LEN);

	ret = gt9xx_fw_check_and_repair(client, GT9X5_DSP_BLOCK_ADDR,
					(u8 *)data, GT9XX_DSP_SECTION_LEN);

	dev_info(&ts->client->dev, "dsp fw flash sucess\n");

	return ret;
}

static int gt9x5_fw_flash_block_boot(struct gt9xx_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret, val = 0, retry = 0;
	const u8 *data;

	/* move head to boot section 1 address*/
	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + GT9X5_BOOT_START_INDEX];

	/* select and hold ss51 & dsp */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9XX_SWRST_B0_DEF_VAL);
	if (ret <= 0)
		return ret;

	/* set scramble */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_OPT_B0_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* release ss51 & release dsp */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9XX_SWRST_B0_REL_SS51_DSP);
	if (ret <= 0)
		return ret;

	/* must sleep for a millisecond */
	usleep_range(1000, 2000);

	/* select bank 3*/
	ret = gt9xx_i2c_write_u8(client, GT9XX_SRAM_BANK_ADDR, 0x03);
	if (ret <= 0)
		return ret;

	/* write the data & verify */
	ret = gt9x5_flash_and_check(client, GT9X5_BOOT_BLOCK_ADDR, (u8 *)data,
			      GT9X5_BOOT_SECTION_LEN);
	if (ret <= 0)
		return ret;

	/* send burn command to move data to flash from sram */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_CTL_ADDR, 0x06);
	if (ret <= 0)
		return ret;

	/* wait for flash burn to complete */
	do {
		ret = gt9xx_i2c_read(client, GT9XX_BOOT_CTL_ADDR, &val, 1);
		if (ret <= 0)
			return ret;
		/* wait for 10 ms, value based on reference driver*/
		usleep_range(10000, 11000);
	} while (val && retry++ < GT9X5_BOOT_SECTION_LEN);
	dev_dbg(&client->dev,
		"boot section flash send burn cmd complete %d %d %d\n",
		val, retry, GT9X5_BOOT_SECTION_LEN);

	ret = gt9xx_fw_check_and_repair(client, GT9X5_BOOT_BLOCK_ADDR,
					(u8 *)data, GT9X5_BOOT_SECTION_LEN);
	if (ret <= 0)
		return ret;

	dev_info(&ts->client->dev, "boot fw flash sucess\n");

	return ret;
}

static int gt9x5_fw_flash_block_boot_isp(struct gt9xx_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret, val = 0, retry = 0;
	const u8 *data;

	/* move head to boot section 1 address*/
	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + GT9X5_BOOT_ISP_START_INDEX];

	/* select and hold ss51 & dsp */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9XX_SWRST_B0_DEF_VAL);
	if (ret <= 0)
		return ret;

	/* set scramble */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_OPT_B0_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* release ss51 & release dsp */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9XX_SWRST_B0_REL_SS51_DSP);
	if (ret <= 0)
		return ret;

	/* must sleep for a millisecond */
	usleep_range(1000, 2000);

	/* select bank 3*/
	ret = gt9xx_i2c_write_u8(client, GT9XX_SRAM_BANK_ADDR, 0x03);
	if (ret <= 0)
		return ret;

	/* write the data & verify */
	ret = gt9x5_flash_and_check(client, GT9X5_BOOT_ISP_BLOCK_ADDR,
				    (u8 *)data, GT9X5_BOOT_ISP_SECTION_LEN);
	if (ret <= 0)
		return ret;

	/* send burn command to move data to flash from sram */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_CTL_ADDR, 0x07);
	if (ret <= 0)
		return ret;

	/* wait for flash burn to complete */
	do {
		ret = gt9xx_i2c_read(client, GT9XX_BOOT_CTL_ADDR, &val, 1);
		if (ret <= 0)
			return ret;
		/* wait for 10 ms, value based on reference driver*/
		usleep_range(10000, 11000);
	} while (val && retry++ < GT9X5_BOOT_ISP_SECTION_LEN);
	dev_dbg(&client->dev,
		"boot isp section flash send burn cmd complete %d %d %d\n",
		val, retry, GT9X5_BOOT_ISP_SECTION_LEN);

	ret = gt9xx_fw_check_and_repair(client, GT9X5_BOOT_ISP_BLOCK_ADDR,
					(u8 *)data, GT9X5_BOOT_ISP_SECTION_LEN);
	if (ret <= 0)
		return ret;

	dev_info(&ts->client->dev, "boot isp fw flash sucess\n");

	return ret;
}

static int gt9x5_fw_flash_g_section(struct i2c_client *client, u8 bank,
				       u16 addr, u8 *data, u32 len)
{
	int ret, val = 0, retry = 0;

	dev_dbg(&client->dev,
		"fw flash g section bank:%d addr:%04x len:%d\n",
		bank, addr, len);

	/* select and hold ss51 & dsp */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9XX_SWRST_B0_DEF_VAL);
	if (ret <= 0)
		return ret;

	/* set scramble */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_OPT_B0_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* release dsp & hold ss51 */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9X5_SWRST_REL_DSP_HOLD_SS51);
	if (ret <= 0)
		return ret;

	/* must sleep for a millisecond */
	usleep_range(1000, 2000);

	/* select bank */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SRAM_BANK_ADDR,
				 (bank >> 4) & 0x0f);
	if (ret <= 0)
		return ret;

	/* write the data */
	ret = gt9x5_flash_and_check(client, addr, data, len);
	if (ret <= 0)
		return ret;

	/* send burn command to move data to flash from sram */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_CTL_ADDR, bank & 0x0f);
	if (ret <= 0)
		return ret;

	/* wait for flash burn to complete */
	do {
		ret = gt9xx_i2c_read(client, GT9XX_BOOT_CTL_ADDR, &val, 1);
		if (ret <= 0)
			return ret;
		/* wait for 10 ms, value based on reference driver*/
		usleep_range(10000, 11000);
	} while (val && retry++ < len);

	dev_dbg(&client->dev,
		"g section burn command complete val:%d retry:%d len:%d\n",
		val, retry, len);

	ret = gt9xx_fw_check_and_repair(client, addr, data, len);

	return ret;
}


static int gt9x5_fw_flash_block_gfwlink(struct gt9xx_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret, newlen;
	const u8 *data;

	/* move head to g section 1 address*/
	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + GT9X5_GFWLINK_START_INDEX];

	/* flash g firmware section 1*/
	ret =  gt9x5_fw_flash_g_section(client, 0x38, GT9X5_GFWLINK_BLOCK_ADDR,
					   (u8 *)data,
					   GT9X5_GFWLINK_SECTION_LEN);
	if (ret <= 0)
		return ret;

	/* move head to g section 2 address */
	data = data + GT9X5_GFWLINK_SECTION_LEN;

	newlen = GT9X5_GFWLINK_LEN - GT9X5_GFWLINK_SECTION_LEN;

	/* flash firmware g section 2*/
	ret =  gt9x5_fw_flash_g_section(client, 0x39, GT9X5_GFWLINK_BLOCK_ADDR,
					   (u8 *)data, newlen);
	if (ret <= 0)
		return ret;

	dev_info(&ts->client->dev, "gfwlink flash sucess\n");

	return ret;
}

static int gt9x5_fw_flash_block_gwake(struct gt9xx_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret;
	const u8 *data;

	/* move head to g section 1 address*/
	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + GT9X5_GWAKE_START_INDEX];

	/* flash g firmware section 1*/
	ret =  gt9x5_fw_flash_g_section(client, 0x3a, GT9X5_GWAKE_BLOCK_ADDR,
					   (u8 *)data, GT9X5_GWAKE_SECTION_LEN);
	if (ret <= 0)
		return ret;

	/* move head to g section 2 address */
	data = data + GT9X5_GWAKE_SECTION_LEN;

	/* flash firmware g section 2*/
	ret =  gt9x5_fw_flash_g_section(client, 0x3b, GT9X5_GWAKE_BLOCK_ADDR,
					   (u8 *)data, GT9X5_GWAKE_SECTION_LEN);
	if (ret <= 0)
		return ret;

	/* move head to g section 3 address */
	data = data + GT9X5_GWAKE_SECTION_LEN;

	/* flash firmware g section 3*/
	ret =  gt9x5_fw_flash_g_section(client, 0x3c, GT9X5_GWAKE_BLOCK_ADDR,
					   (u8 *)data, GT9X5_GWAKE_SECTION_LEN);
	if (ret <= 0)
		return ret;

	/* move head to g section 4 address */
	data = data + GT9X5_GWAKE_SECTION_LEN;

	/* flash firmware g section 4*/
	ret =  gt9x5_fw_flash_g_section(client, 0x3d, GT9X5_GWAKE_BLOCK_ADDR,
					   (u8 *)data, GT9X5_GWAKE_SECTION_LEN);
	if (ret <= 0)
		return ret;

	dev_info(&ts->client->dev, "gwake flash sucess\n");

	return ret;
}

static int gt9x5_fw_flash_block_dsp_isp(struct gt9xx_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret;
	const u8 *data;

	/* move head to dsp isp address*/
	data = &ts->fw->data[ts->fw->size - GT9X5_DSP_ISP_START_INDEX];

	/* disable wdt */
	ret = gt9xx_i2c_write_u8(client, GT9XX_TMR0_EN_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* clear cache */
	ret = gt9xx_i2c_write_u8(client, GT9XX_CACHE_EN_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* select and hold ss51 & dsp */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9XX_SWRST_B0_DEF_VAL);
	if (ret <= 0)
		return ret;

	/* set boot from SRAM */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOTCTL_B0_ADDR,
				 GT9XX_BOOTCTL_B0_SRAM);
	if (ret <= 0)
		return ret;

	/* software reset */
	ret = gt9xx_i2c_write_u8(client, GT9XX_CPU_SWRST_PULSE_ADDR, 1);
	if (ret <= 0)
		return ret;

	/* select bank 2*/
	ret = gt9xx_i2c_write_u8(client, GT9XX_SRAM_BANK_ADDR, 0x02);
	if (ret <= 0)
		return ret;

	/* enable accessing mode */
	ret = gt9xx_i2c_write_u8(client, GT9XX_MEM_CD_EN_ADDR, 1);
	if (ret <= 0)
		return ret;

	/* write the data & verify */
	ret = gt9x5_flash_and_check(client, GT9X5_DSP_ISP_BLOCK_ADDR,
				    (u8 *)data, GT9X5_DSP_ISP_SECTION_LEN);
	if (ret <= 0)
		return ret;

	ret = gt9xx_fw_check_and_repair(client, GT9X5_DSP_ISP_BLOCK_ADDR,
					(u8 *)data, GT9X5_DSP_ISP_SECTION_LEN);
	if (ret <= 0)
		return ret;

	/* set scramble */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_OPT_B0_ADDR, 0);

	dev_info(&ts->client->dev, "dsp isp fw flash sucess\n");

	return ret;
}

static int gt9x5_fw_flash_finish(struct gt9xx_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret;
	const u8 *data;

	/* move head to section 1 address*/
	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + GT9XX_SS51_START_INDEX];

	/* clear control flag */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_CTL_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* flash ss51 firmware section 1*/
	ret =  gt9x5_fw_flash_ss51_section(client, 1, GT9X5_SS51_BLOCK_ADDR1,
					   (u8 *)data,
					   GT9XX_SS51_SECTION_LEN);
	if (ret <= 0)
		return ret;

	/* enable download DSP code */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_CTL_ADDR, 0x99);
	if (ret <= 0)
		return ret;

	/* release ss51 & hold dsp */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
				 GT9X5_SWRST_REL_SS51_HOLD_DSP);

	dev_info(&ts->client->dev, "dsp fw flash finish sucess\n");

	return ret;
}

static int gt9xx_fw_flash_block_dsp(struct gt9xx_ts *ts)
{
	struct i2c_client *client = ts->client;
	const u8 *data;

	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + GT9XX_DSP_START_INDEX];

	return gt9xx_fw_flash_verify(client, GT9XX_DSP_BANK_INDEX0,
				     GT9XX_DSP_BLOCK_ADDR, (u8 *)data,
				     GT9XX_DSP_SECTION_LEN);
}

static int gt9xx_fw_flash_block_ss51(struct gt9xx_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret;
	const u8 *data;

	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + GT9XX_SS51_START_INDEX];

	/* flash first sections */
	ret = gt9xx_fw_flash_verify(client, GT9XX_SS51_BANK_INDEX0,
				    GT9XX_SS51_BLOCK_ADDR, (u8 *)data,
				    GT9XX_SS51_SECTION_LEN * 2);
	if (ret <= 0)
		return ret;

	data = &ts->fw->data[GT9XX_FW_HEAD_LEN + 2 * GT9XX_SS51_SECTION_LEN];

	/* flash next 2 sections */
	return gt9xx_fw_flash_verify(client, GT9XX_SS51_BANK_INDEX1,
				     GT9XX_SS51_BLOCK_ADDR, (u8 *)data,
				     GT9XX_SS51_SECTION_LEN * 2);
}

/* If the version of current fw in tp is not older than the
 * target updated fw, return 0, otherwise return 1.
 */
static int gt9xx_fw_ver_check(struct gt9xx_ts *ts)
{
	/* perform checksum calculation */
	ts->fw_head = (struct gt9xx_fw_head *)ts->fw->data;

	ts->fw_head->vid = (u16)be16_to_cpu(ts->fw_head->vid);

	if (ts->input->id.version >= ts->fw_head->vid)
		return 0;

	return 1;
}

static int gt9xx_fw_checksum(struct gt9xx_ts *ts)
{
	int i, checksum = 0;

	for (i = GT9XX_FW_HEAD_LEN; i < ts->fw->size; i += 2)
		checksum += (ts->fw->data[i] << 8) + ts->fw->data[i+1];

	if (checksum & 0xffff)
		return -EINVAL;

	return 0;
}

static int gt9xx_fw_download_enabled(struct gt9xx_ts *ts)
{
	unsigned long long load_firmware = 0;
	struct device_node *of_np = ts->client->dev.of_node;
#ifdef CONFIG_ACPI
	acpi_handle handle;
	acpi_status status;

	handle = ACPI_HANDLE(&ts->client->dev);
	if (!handle)
		return load_firmware;

	status = acpi_evaluate_integer(handle, "_FRM", NULL, &load_firmware);
	if (ACPI_SUCCESS(status))
		dev_info(&ts->client->dev, "_FRM=%llu\n", load_firmware);
#endif

#ifdef CONFIG_OF
	if (of_get_property(of_np, GT9XX_FIRMWARE_UPDATE_PROPERTY, NULL)) {
		dev_info(&ts->client->dev, "of firmware-udpate enabled\n");
		load_firmware = 1;
	}
#endif

	return load_firmware;
}

static int gt9xx_fw_is_valid(struct gt9xx_ts *ts)
{
	struct {
		u8 id[4];	/* may not be NULL terminated */
		__le16 fw_version;
	} __packed id;
	char id_str[5];
	int ret = 0;

	if (!gt9xx_fw_download_enabled(ts)) {
		dev_warn(&ts->client->dev, "gt9xx fw download not enabled\n");
		return 0;
	}

	/* Get the current version in tp ic */
	ret = gt9xx_i2c_read(ts->client, GT9XX_REG_ID, &id, sizeof(id));
	if (ret <= 0) {
		dev_err(&ts->client->dev, "[%s] read id failed", __func__);
		return ret;
	}

	memcpy(id_str, id.id, 4);
	id_str[4] = 0;
	if (kstrtou16(id_str, 10, &ts->input->id.product))
		ts->input->id.product = 0;
	ts->input->id.version = le16_to_cpu(id.fw_version);

	dev_info(&ts->client->dev, "[%s] current version: %d_%04x", __func__,
		ts->input->id.product, ts->input->id.version);

	/* Get the version info of the firmware file */
	ret = request_firmware(&ts->fw, ts->fw_name, &ts->client->dev);
	if (ret < 0 || !ts->fw) {
		dev_err(&ts->client->dev, "[%s] gt9xx request firmware failed\n",
			__func__);
		return ret;
	}

	if (ts->fw->size < (GT9XX_SS51_SECTION_LEN * 4 +
				GT9XX_DSP_SECTION_LEN)) {
		dev_err(&ts->client->dev, "[%s] Invalid FW size", __func__,
				ts->fw->size);
		goto fw_err;
	}

	if (gt9xx_fw_checksum(ts)) {
		dev_err(&ts->client->dev, "[%s] fw checksum error", __func__);
		goto fw_err;
	}

	return 1;

fw_err:
	release_firmware(ts->fw);
	return 0;
}

static int gt9x5_enter_fw_burn_mode(struct gt9xx_ts *ts)
{
	int retry = 0, ret;
	struct i2c_client *client = ts->client;
	u8 val;

	/* Step1: RST output low last at least 2ms */
	gpiod_direction_output(ts->gpiod_rst, 0);
	gpiod_set_value(ts->gpiod_rst, 0);
	usleep_range(2000, 2200);

	/* Step2: select I2C slave addr,INT:0--0xBA;1--0x28 */
	gpiod_direction_output(ts->gpiod_int, (ts->client->addr == 0x14));
	gpiod_set_value(ts->gpiod_rst, (ts->client->addr == 0x14));
	usleep_range(2000, 2200);

	/* Step3: RST output high reset guitar */
	gpiod_direction_output(ts->gpiod_rst, 1);
	gpiod_set_value(ts->gpiod_rst, 1);
	usleep_range(6000, 6200);

	/* select and hold ss51 & dsp */
	while (retry++ < GT9XX_FW_HOLD_RETRY) {

		ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
					 GT9XX_SWRST_B0_DEF_VAL);
		if (ret <= 0)
			continue;

		ret = gt9xx_i2c_read(client, GT9XX_SWRST_B0_ADDR, &val, 1);
		if (ret <= 0)
			continue;

		if (val == GT9XX_SWRST_B0_DEF_VAL)
			break;
	}

	if (retry >= GT9XX_FW_HOLD_RETRY)
		return -ENODEV;

	/* dsp clock power on*/
	ret = gt9xx_i2c_write_u8(client, GT9XX_DSP_CLK_ADDR, 0);

	return ret;
}

static int gt9xx_enter_fw_burn_mode(struct gt9xx_ts *ts)
{
	int retry = 0, ret;
	struct i2c_client *client = ts->client;
	u8 val;

	/* Step1: RST output low last at least 2ms */
	gpiod_direction_output(ts->gpiod_rst, 0);
	usleep_range(2000, 2200);

	/* Step2: select I2C slave addr,INT:0--0xBA;1--0x28 */
	gpiod_direction_output(ts->gpiod_int, ts->client->addr == 0x14);
	usleep_range(2000, 2200);

	/* Step3: RST output high reset guitar */
	gpiod_direction_output(ts->gpiod_rst, 1);
	usleep_range(6000, 6200);

	/* select and hold ss51 & dsp */
	while (retry++ < GT9XX_FW_HOLD_RETRY) {

		ret = gt9xx_i2c_write_u8(client, GT9XX_SWRST_B0_ADDR,
					 GT9XX_SWRST_B0_DEF_VAL);
		if (ret <= 0)
			continue;

		ret = gt9xx_i2c_read(client, GT9XX_SWRST_B0_ADDR, &val, 1);
		if (ret <= 0)
			continue;

		if (val == GT9XX_SWRST_B0_DEF_VAL)
			break;
	}

	if (retry >= GT9XX_FW_HOLD_RETRY)
		return -ENODEV;

	/* dsp clock power on*/
	ret = gt9xx_i2c_write_u8(client, GT9XX_DSP_CLK_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* disable wdt */
	ret = gt9xx_i2c_write_u8(client, GT9XX_TMR0_EN_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* clear cache */
	ret = gt9xx_i2c_write_u8(client, GT9XX_CACHE_EN_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* set boot from SRAM */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOTCTL_B0_ADDR,
				 GT9XX_BOOTCTL_B0_SRAM);
	if (ret <= 0)
		return ret;

	/* software reset */
	ret = gt9xx_i2c_write_u8(client, GT9XX_CPU_SWRST_PULSE_ADDR, 1);
	if (ret <= 0)
		return ret;

	/* clear control flag */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_CTL_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* set scramble */
	ret = gt9xx_i2c_write_u8(client, GT9XX_BOOT_OPT_B0_ADDR, 0);
	if (ret <= 0)
		return ret;

	/* enable accessing mode */
	ret = gt9xx_i2c_write_u8(client, GT9XX_MEM_CD_EN_ADDR, 1);

	return ret;
}

static int gt9xx_fw_start(struct gt9xx_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret;
	u8 val;

	if (!ts->compat_mode)
		return 1;

	/* init sw WDT */
	ret = gt9xx_i2c_write_u8(client, GT9XX_SW_WDT_ADDR,
				 GT9XX_SW_WDT_DEF_VAL);
	if (ret <= 0)
		return ret;

	/* release SS51 & DSP */
	ret = gt9xx_i2c_write_u8(client, GT9XX_FW_REL_ADDR, 0x00);
	if (ret <= 0)
		return ret;

	gt9xx_int_sync(ts);

	/* Check fw run status by verifying WDT default */
	ret = gt9xx_i2c_read(client, GT9XX_SW_WDT_ADDR, &val, sizeof(u8));
	if (ret <= 0)
		return ret;

	if (val == GT9XX_SW_WDT_DEF_VAL) {
		dev_err(&client->dev, "fw_start failed\n");
		return -ENODEV;
	}

	/* on success init sw WDT again*/
	ret = gt9xx_i2c_write_u8(client, GT9XX_SW_WDT_ADDR,
				 GT9XX_SW_WDT_DEF_VAL);
	return ret;
}

static int gt9xx_flash_fw(struct gt9xx_ts *ts)
{
	int ret, retry = 0;

	gt9xx_irq_disable(ts, false);

	ret = gt9xx_enter_fw_burn_mode(ts);
	if (ret <= 0) {
		dev_err(&ts->client->dev, "gt9xx fw burn mode set failed\n");
		return ret;
	}

	while (retry++ < GT9XX_FW_DOWNLOAD_RETRY) {
		ret = gt9xx_fw_flash_block_ss51(ts);
		if (ret <= 0)
			continue;

		ret = gt9xx_fw_flash_block_dsp(ts);
		if (ret <= 0)
			continue;
		break;
	}

	if (retry >= GT9XX_FW_DOWNLOAD_RETRY)
		dev_info(&ts->client->dev, "gt9xx exceeds max retry count\n");

	return ret;
}

static int gt9x5_flash_fw(struct gt9xx_ts *ts)
{
	int ret, retry = 0;

	gt9xx_irq_disable(ts, true);

	ret = gt9x5_enter_fw_burn_mode(ts);
	if (ret <= 0) {
		dev_err(&ts->client->dev, "gt9xx fw burn mode set failed\n");
		return ret;
	}

	while (retry++ < GT9XX_FW_DOWNLOAD_RETRY) {
		ret = gt9x5_fw_flash_block_dsp_isp(ts);
		if (ret <= 0) {
			dev_info(&ts->client->dev,
				 "dsp isp fw flash failed, retry:%d\n", retry);
			continue;
		}

		ret = gt9x5_fw_flash_block_ss51(ts);
		if (ret <= 0) {
			dev_info(&ts->client->dev,
				 "ss51 fw flash failed, retry:%d\n", retry);
			continue;
		}

		ret = gt9x5_fw_flash_block_dsp(ts);
		if (ret <= 0) {
			dev_info(&ts->client->dev,
				 "dsp fw flash failed, retry:%d\n", retry);
			continue;
		}

		ret = gt9x5_fw_flash_block_boot(ts);
		if (ret <= 0) {
			dev_info(&ts->client->dev,
				 "boot fw flash failed, retry:%d\n", retry);
			continue;
		}

		ret = gt9x5_fw_flash_block_boot_isp(ts);
		if (ret <= 0) {
			dev_info(&ts->client->dev,
				 "boot isp fw flash failed, retry:%d\n", retry);
			continue;
		}

		ret = gt9x5_fw_flash_block_gfwlink(ts);
		if (ret <= 0) {
			dev_info(&ts->client->dev,
				 "gfwlink fw flash failed, retry:%d\n", retry);
			continue;
		}

		ret = gt9x5_fw_flash_block_gwake(ts);
		if (ret <= 0) {
			dev_info(&ts->client->dev,
				 "gwake fw flash failed, retry:%d\n", retry);
			continue;
		}

		ret = gt9x5_fw_flash_finish(ts);
		if (ret <= 0) {
			dev_info(&ts->client->dev,
				 "fw finish flash failed, retry:%d\n", retry);
			continue;
		}

		break;
	}

	if (retry >= GT9XX_FW_DOWNLOAD_RETRY)
		dev_info(&ts->client->dev, "gt9xx exceeds max retry count\n");

	return ret;
}

static int gt9xx_fw_download(struct gt9xx_ts *ts)
{
	int ret;

	dev_warn(&ts->client->dev, "firmware name %s\n", ts->fw_name);

	if (!ts->fw) {
		dev_err(&ts->client->dev, "gt9xx invalid firmware\n");
		return -EINVAL;
	}

	ret = gt9xx_fw_ver_check(ts);
	if (0 == ret) {
		dev_warn(&ts->client->dev, "[%s] target version is not new: %04x",
				__func__, ts->fw_head->vid);
		goto fw_err;
	}

	if (ts->compat_mode) {
		ret = gt9xx_flash_fw(ts);
		if (ret < 0) {
			dev_err(&ts->client->dev, "gt9xx fw flash failed\n");
			goto fw_err;
		}
	} else {
		ret = gt9x5_flash_fw(ts);
		if (ret < 0) {
			dev_err(&ts->client->dev, "gt9x5 fw flash failed\n");
			goto fw_err;
		}
	}

	dev_info(&ts->client->dev, "gt9xx download fw sucessfull\n");

fw_err:
	release_firmware(ts->fw);
	gt9xx_fw_start(ts);
	/* reset the controller */
	gt9xx_reset(ts);
	/* send the _DSM ts configuration to the firmware */
#ifdef CONFIG_ACPI
	gt9xx_send_cfg(ts);
#else
	gt9xx_get_and_send_cfg(ts);
#endif
	gt9xx_int_sync(ts);
	gt9xx_irq_enable(ts);
	return ret;
}

static int gt9xx_set_device_data(struct gt9xx_ts *ts)
{
	struct device *dev = &ts->client->dev;
	struct gpio_desc *gpiod;
	struct pinctrl_state *state;
	struct pinctrl *pinctrl;
	struct device_node *of_np = ts->client->dev.of_node;
	int ret;

	if (of_np) {
		struct property *prop = of_find_property(
						of_np,
						GT9XX_BUTTONS_PROPERTY,
						NULL);
		if (prop) {
			size_t code_size = sizeof(ts->button_codes[0]);

			/* The button property size to be an
			 * even number of u32's and bigger than zero
			 */
			ts->num_buttons = prop->length / code_size;
			if (prop->length % code_size ||
			     ts->num_buttons == 0 ||
			     ts->num_buttons > GT9XX_MAX_NUM_BUTTONS) {
				dev_err(&ts->client->dev,
					"button property size is incorrect\n");
				return -EINVAL;
			}

			ret = of_property_read_u32_array(
					ts->client->dev.of_node,
					GT9XX_BUTTONS_PROPERTY,
					ts->button_codes,
					ts->num_buttons);
			if (ret) {
				dev_err(&ts->client->dev,
					"failed to read button property\n");
				return ret;
			}
		}
		if (!of_property_read_string(of_np, GT9XX_FW_NAME_PROPERTY,
				    &ts->fw_name))
			dev_info(&ts->client->dev, "found new firmware %s\n",
				 ts->fw_name);
		if (of_get_property(of_np, GT9XX_COMPAT_MODE_PROPERTY,
				    NULL)) {
			dev_info(&ts->client->dev, "of compatibility mode enabled\n");
			ts->compat_mode = 1;
		}
	}

	pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ts->pinctrl))
		ts->pinctrl = pinctrl;

	if (ts->pinctrl) {
		state = pinctrl_lookup_state(pinctrl, "interrupt");
		if (IS_ERR(state)) {
			dev_err(&ts->client->dev, "could not get interrupt pin state!\n");
			return PTR_ERR(state);
		}
		ts->int_state = state;

		state = pinctrl_lookup_state(pinctrl, "control");
		if (IS_ERR(state)) {
			dev_err(&ts->client->dev, "could not get control pin state!\n");
			return PTR_ERR(state);
		}
		ts->ctrl_state = state;
	}

	/* Get interrupt GPIO pin number */
	if (ACPI_HANDLE(dev))
		gpiod = devm_gpiod_get_index(dev, "gt9xx_gpio_int", 0);
	else
		gpiod = devm_gpiod_get(dev, "irq");

	if (IS_ERR(gpiod)) {
		int err = PTR_ERR(gpiod);

		dev_err(dev, "get gt9xx_gpio_int failed: %d\n", err);
		return err;
	}

	gpiod_direction_input(gpiod);
	ts->gpiod_int = gpiod;

	if (!ts->client->irq)
		ts->client->irq = gpiod_to_irq(gpiod);

	/* get the reset line GPIO pin number */
	if (ACPI_HANDLE(dev))
		gpiod = devm_gpiod_get_index(dev, "gt9xx_gpio_rst", 1);
	else
		gpiod = devm_gpiod_get(dev, "reset");

	if (IS_ERR(gpiod)) {
		int err = PTR_ERR(gpiod);

		dev_err(dev, "get gt9xx_gpio_rst failed: %d\n", err);
		return err;
	}

	gpiod_direction_input(gpiod);
	ts->gpiod_rst = gpiod;

#ifdef CONFIG_PM
	if (of_np) {
		ts->pm_platdata = of_device_state_pm_setup(of_np);
		if (IS_ERR(ts->pm_platdata)) {
			dev_err(dev, "error during device state pm init");
			ret = PTR_ERR(ts->pm_platdata);
			return ret;
		}

		ret = device_state_pm_set_class(dev,
			ts->pm_platdata->pm_user_name);
		if (ret) {
			dev_err(dev, "error while setting the pm class");
			return ret;
		}

		ret = device_state_pm_set_state_by_name(dev,
			ts->pm_platdata->pm_state_D0_name);
		if (ret) {
			dev_err(dev, "error while setting the pm state");
			return ret;
		}
	}
#endif
    /* reset the controller */
	gt9xx_reset(ts);

	return  gt9xx_get_and_send_cfg(ts);
}

#ifdef CONFIG_PM
static void gt9xx_sleep(struct gt9xx_ts *ts)
{
	int ret;
	struct pinctrl *pinctrl = ts->pinctrl;
	struct device *dev = &ts->client->dev;

	if (test_and_set_bit(GT9XX_STATUS_SLEEP_BIT, ts->status))
		return;

	gt9xx_irq_disable(ts, false);

	if (pinctrl)
		pinctrl_select_state(pinctrl, ts->ctrl_state);

	gpiod_direction_output(ts->gpiod_int, 0);
	msleep(5);

	ret = gt9xx_i2c_write_u8(ts->client, GT9XX_REG_CMD, 5);
	if (ret <= 0) {
		dev_err(dev, "sleep cmd failed");
	}

	ret = device_state_pm_set_state_by_name(dev,
		ts->pm_platdata->pm_state_D3_name);
	if (ret)
		dev_err(dev, "error while setting the pm state");

	dev_dbg(dev, "sleeping");
}

static void gt9xx_wakeup(struct gt9xx_ts *ts)
{
	int ret;
	struct device *dev = &ts->client->dev;

	if (!test_and_clear_bit(GT9XX_STATUS_SLEEP_BIT, ts->status))
		return;

	ret = device_state_pm_set_state_by_name(dev,
		ts->pm_platdata->pm_state_D0_name);
	if (ret)
		dev_err(dev, "error while setting the pm state");

	gt9xx_reset(ts);

	ret = gt9xx_i2c_test(ts->client);
	if (ret <= 0) {
		dev_err(dev, "i2c test after reset failed");
	}

	gt9xx_irq_enable(ts);

	dev_dbg(dev, "woke up");
}

static ssize_t gt9xx_power_hal_suspend_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct gt9xx_ts *ts = input_get_drvdata(input);
	static DEFINE_MUTEX(mutex);

	mutex_lock(&mutex);
	if (!strncmp(buf, POWER_HAL_SUSPEND_ON, POWER_HAL_SUSPEND_STATUS_LEN))
		gt9xx_sleep(ts);
	else
		gt9xx_wakeup(ts);
	mutex_unlock(&mutex);

	return count;
}

static DEVICE_POWER_HAL_SUSPEND_ATTR(gt9xx_power_hal_suspend_store);
#endif

static int gt9xx_ts_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret = -1;
	struct gt9xx_ts *ts;
	struct device *dev = &client->dev;
	int i;

	dev_info(dev, "probing GT911 @ 0x%02x", client->addr);

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	__set_bit(EV_SYN, ts->input->evbit);
	__set_bit(EV_ABS, ts->input->evbit);
	__set_bit(EV_KEY, ts->input->evbit);
	__set_bit(BTN_TOUCH, ts->input->keybit);

	input_mt_init_slots(ts->input, GT9XX_MAX_TOUCHES, 0);
	input_set_abs_params(ts->input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(dev));
	ts->input->name = "goodix_ts";
	ts->input->phys = ts->phys;
	ts->input->id.bustype = BUS_I2C;
	ts->input->dev.parent = dev;
	input_set_drvdata(ts->input, ts);

	ts->fw_name = GT9XX_FW_FILE_NAME;

	ret = gt9xx_set_device_data(ts);
	if (ret)
		return ret;

	ret = gt9xx_fw_is_valid(ts);
	if (ret > 0)
		ret = gt9xx_fw_download(ts);

	ret = gt9xx_get_info(ts);
	if (ret <= 0)
		return ret;

	for (i = 0; i < ts->num_buttons; i++)
		__set_bit(ts->button_codes[i], ts->input->keybit);

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);

	ret = input_register_device(ts->input);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					gt9xx_thread_handler,
					ts->irq_type | IRQF_ONESHOT,
					client->name, ts);
	if (ret) {
		dev_err(dev, "request IRQ failed: %d", ret);
		input_unregister_device(ts->input);
		return -1;
	}

#ifdef CONFIG_PM
	ret = device_create_file(dev, &dev_attr_power_HAL_suspend);
	if (ret < 0) {
		dev_err(dev, "unable to create suspend entry");
		goto out;
	}

	ret = register_power_hal_suspend_device(dev);
	if (ret < 0)
		dev_err(dev, "unable to register for power hal");
out:
#endif

	return 0;
}

static int gt9xx_ts_remove(struct i2c_client *client)
{
	struct gt9xx_ts *ts = i2c_get_clientdata(client);

#ifdef CONFIG_PM
	device_remove_file(&client->dev, &dev_attr_power_HAL_suspend);
	unregister_power_hal_suspend_device(&ts->input->dev);
#endif
	i2c_set_clientdata(client, NULL);
	gpiod_direction_input(ts->gpiod_int);
	input_unregister_device(ts->input);

	return 0;
}

static const struct i2c_device_id gt9xx_ts_id[] = {
	{ "GODX0911", 0 },
	{ "GODX0912", 0 },
	{ "GOOD9271", 0 },
	{ "GOOD9157", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, gt9xx_ts_id);

#ifdef CONFIG_ACPI
static struct acpi_device_id gt9xx_acpi_match[] = {
	{ "GODX0911", 0 },
	{ "GODX0912", 0 },
	{ "GOOD9271", 0 },
	{ "GOOD9157", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, gt9xx_acpi_match);
#endif

static struct i2c_driver gt9xx_ts_driver = {
	.probe      = gt9xx_ts_probe,
	.remove     = gt9xx_ts_remove,
	.id_table   = gt9xx_ts_id,
	.driver = {
		.name = "gt9xx_ts",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(gt9xx_acpi_match),
	},
};

module_i2c_driver(gt9xx_ts_driver);

MODULE_DESCRIPTION("Goodix GT911 Touchscreen Driver");
MODULE_AUTHOR("Octavian Purdila <octavian.purdila@intel.com>");
MODULE_LICENSE("GPL v2");
