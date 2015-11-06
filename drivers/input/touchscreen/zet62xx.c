/* -------------------------------------------------------------------------
 * Copyright (C) 2014-2015, Intel Corporation
 *
 * Derived from:
 *  zet62xx.c , Zeitec Semiconductor Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * ------------------------------------------------------------------------- */

#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/pm.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_PM
#include <linux/power_hal_sysfs.h>
#include <linux/device_pm_data.h>
#endif

#define PRESSURE_CONST			1
#define FINGER_NUMBER			10
#define MT_FINGER_NUMBER		10
#define KEY_NUMBER                      0
#define DEBOUNCE_NUMBER			1
#define TS_WAKEUP_LOW_PERIOD		10
#define TS_WAKEUP_HIGH_PERIOD		5

#define MAX_FLASH_BUF_SIZE		0x10000
#define FINGER_REPROT_DATA_HEADER	0x3C
#define FINGER_PACK_SIZE		4
#define FINGER_HEADER_SHIFT		3

#define TRUE				1
#define FALSE				0

#define X_MAX                           (1024)
#define Y_MAX                           (600)
#define P_MAX				(255)
#define FW_X_RESOLUTION			(1280)
#define FW_Y_RESOLUTION			(720)
#define MAX_KEY_NUMBER			(8)
#define MAX_FINGER_NUMBER		(16)
#define PRE_PRESSED_DEFAULT_VALUE       -1

#define ZET62XX_CMD_COORD		0xb2
#define ZET62XX_CMD_WRITE_PASSWORD	0x20
#define ZET62XX_CMD_PAGE_WRITE		0x22
#define ZET62XX_CMD_PAGE_ERASE		0x23
#define ZET62XX_CMD_MASS_ERASE		0x24
#define ZET62XX_CMD_PAGE_READ		0x25
#define ZET62XX_CMD_READ_CODE_OPTION	0x27
#define ZET62XX_CMD_READ_SFR		0x2c
#define ZET62XX_CMD_WRITE_SFR		0x2b
#define ZET62XX_CMD_CALC_CHKSUM		0x36
#define ZET62XX_CMD_READ_CHKSUM		0x37

#define ZET62XX_PASSWORD_HIBYTE		0xc5
#define ZET62XX_PASSWORD_LOBYTE		0x9d
#define ZET62XX_PASSWORD_1K_HIBYTE	0xb9
#define ZET62XX_PASSWORD_1K_LOBYTE	0xa3

#define ZET62XX_DYNAMIC_DATA_HEADER	0x3c

/* dynamic data length in bytes */
#define ZET62XX_PACKET_HEADER_LEN	0x03
#define ZET62XX_FINGER_DATA_LEN		0x04
#define ZET62XX_KEY_DATA_LEN		0x01
#define ZET62XX_MAX_PACKET_LEN		24
#define ZET62XX_MAX_FINGER_LEN		16

#define ZET62XX_CODE_OPTION_LEN		32
#define ZET62XX_SFR_DATA_LEN		16

#define ZET62XX_SFR_UNLOCK_FLASH	0x3d
#define ZET62XX_SFR_LOCK_FLASH		0x7d

#define ZET62XX_PCODE_MAX_COUNT		0x08

#define ZET62XX_FLASH_PAGE_LEN		128
#define ZET62XX_MAX_PAGE_LEN		256

#define ZET_TS_ID_NAME			"zet62xx_ts"

#define ZET6221_FW_NAME			"zet6221.fw"
#define ZET6223_FW_NAME			"zet6223.fw"
#define ZET6231_FW_NAME			"zet6231.fw"
#define ZET6251_FW_NAME			"zet6251.fw"
#define ZET6270_FW_NAME			"zet6270.fw"

#define ZET62XX_RESET_GPIO_NAME "reset-gpio"
#define ZET62XX_IRQ_GPIO_NAME "irq-gpio"

struct light_load_report_mode
{
	u32 pre_x;
	u32 pre_y;
	u32 pre_z;
	int pressed;
};

struct finger_coordinate_struct
{
	u32 report_x;
	u32 report_y;
	u32 report_z;
	u32 last_report_x;
	u32 last_report_y;
	u32 last_report_z;
	u32 coordinate_x;
	u32 coordinate_y;
	u32 last_coordinate_x;
	u32 last_coordinate_y;
	u32 predicted_coordinate_x;
	u32 predicted_coordinate_y;
	u32 last_distance;
	u8 valid;
};

enum zet62xx_chip_id {
	ZET6221,
	ZET6223,
	ZET6231,
	ZET6241,
	ZET6251,
	ZET6270,
	ZET_MAX_CHIP
};

enum zet62xx_rom_type {
	ZET62XX_ROM_TYPE_SRAM,
	ZET62XX_ROM_TYPE_FLASH,
	ZET62XX_ROM_TYPE_OTP,
	ZET62XX_ROM_TYPE_UNKNOWN,
};

struct zet62xx_chip_config {
	u8 chip_id;
	u16 x_max;
        u16 y_max;
	u8 max_finger;
	u8 max_key;
	u16 packet_len;
	u8 rom_type;
	u16 pcode[ZET62XX_PCODE_MAX_COUNT];
};

struct zet62xx_ts_data {
	struct i2c_client *client;
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_irq;
	struct input_dev *input_dev;
	struct zet62xx_chip_config *config;
	const struct firmware *fw;
	const char *fw_name;
	u8 *flash_buffer;
	unsigned long flash_tot_len;
	unsigned int gpio;
	unsigned int irq;
};


static u16 resolution_x	= X_MAX;
static u16 resolution_y	= Y_MAX;
static u16 TS_RST_GPIO = 32;
static u16 TS_INT_GPIO = 66;
static u16 finger_num;
static u16 key_num;
static int finger_packet_size;
static u16 finger_up_cnt;
static struct light_load_report_mode pre_event[MAX_FINGER_NUMBER];
static struct finger_coordinate_struct finger_report[MAX_FINGER_NUMBER];
static u8 finger_report_key;
static u8 hover_status	= 0;
static u8 key_menu_pressed = 0x00;
static u8 key_back_pressed = 0x00;
static u8 key_home_pressed = 0x00;

struct zet62xx_chip_config chip_config [] = {

	/* 6221 chip config */
	{
		.chip_id = ZET6221,
		.x_max = 1024,
		.y_max = 600,
		.max_finger = 0x05,
		.max_key = 0,
		.packet_len = 23,
		.rom_type = ZET62XX_ROM_TYPE_FLASH,
		.pcode = {
				0x3DF1, 0x3DF4,
				0x3DF7, 0x3DFA,
				0x3EF6, 0x3EF9,
				0x3EFC, 0x3EFF
			},
	},
	/* 6223 chip config */
	{
		.chip_id = ZET6223,
		.x_max = 1024,
		.y_max = 600,
		.max_finger = 0x05,
		.max_key = 0,
		.packet_len = 23,
		.rom_type = ZET62XX_ROM_TYPE_FLASH,
		.pcode = {
				0x7BFC, 0x7BFD,
				0x7BFE, 0x7BFF,
				0x7C04, 0x7C05,
				0x7C06, 0x7C07
			},

	},
	/* 6231 chip config */
	{
		.chip_id = ZET6231,
		.x_max = 1024,
		.y_max = 600,
		.max_finger = 0x05,
		.max_key = 0,
		.packet_len = 23,
		.rom_type = ZET62XX_ROM_TYPE_FLASH,
		.pcode = {
				0x3DF1, 0x3DF4,
				0x3DF7, 0x3DFA,
				0x3EF6, 0x3EF9,
				0x3EFC, 0x3EFF
			},
	},
	/* 6241 chip config */
	{
		.chip_id = ZET6241,
		.x_max = 1024,
		.y_max = 600,
		.max_finger = 0x05,
		.max_key = 0,
		.packet_len = 23,
		.rom_type = ZET62XX_ROM_TYPE_FLASH,
		.pcode = {
				0x3DF1, 0x3DF4,
				0x3DF7, 0x3DFA,
				0x3EF6, 0x3EF9,
				0x3EFC, 0x3EFF
			},
	},
	/* 6251 chip config */
	{
		.chip_id = ZET6251,
		.x_max = 1024,
		.y_max = 600,
		.max_finger = 0x0A,
		.max_key = 0,
		.packet_len = 43,
		.rom_type = ZET62XX_ROM_TYPE_SRAM,
		.pcode = {
				0x3DF1, 0x3DF4,
				0x3DF7, 0x3DFA,
				0x3EF6, 0x3EF9,
				0x3EFC, 0x3EFF
			},
	},
	/* 6270 chip config */
	{
		.chip_id = ZET6270,
		.x_max = 1024,
		.y_max = 600,
		.max_finger = 0x05,
		.max_key = 0,
		.packet_len = 23,
		.rom_type = ZET62XX_ROM_TYPE_SRAM,
		.pcode = {
				0x9BFC, 0x9BFD,
				0x9BFE, 0x9BFF,
				0x9C04, 0x9C05,
				0x9C06, 0x9C07
			},
	},
};

static void zet62xx_ts_wakeup(struct zet62xx_ts_data *data)
{
	gpiod_set_value_cansleep(data->gpio_reset, 0);
	mdelay(TS_WAKEUP_LOW_PERIOD);
	gpiod_set_value_cansleep(data->gpio_reset, 1);
	mdelay(TS_WAKEUP_HIGH_PERIOD);
}

#ifdef CONFIG_PM
static void zet62xx_ts_sleep(struct device *dev)
{
	int ret = 0;
	struct i2C_client *this_client = to_i2c_client(dev);
	ret = i2c_smbus_write_byte(this_client, 0xb1);
	return;
}

static ssize_t zet62xx_power_hal_suspend_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct zet62xx_ts_data *ts = input_get_drvdata(input);
	static DEFINE_MUTEX(mutex);

	mutex_lock(&mutex);
	if (!strncmp(buf, POWER_HAL_SUSPEND_ON, POWER_HAL_SUSPEND_STATUS_LEN))
		zet62xx_ts_sleep(dev);
	else
		zet62xx_ts_wakeup(ts);
	mutex_unlock(&mutex);

	return count;
}
static DEVICE_POWER_HAL_SUSPEND_ATTR(zet62xx_power_hal_suspend_store);
#endif


static void zet62xx_ts_reset(struct zet62xx_ts_data *data)
{
	gpiod_set_value_cansleep(data->gpio_reset, 1);
	msleep(1);
	gpiod_set_value_cansleep(data->gpio_reset, 0);
	msleep(2);
}

s32 zet622x_i2c_read_tsdata(struct i2c_client *client, u8 *data, u8 length)
{
	struct i2c_msg msg;
	msg.addr     = client->addr;
	msg.flags    = I2C_M_RD;
	msg.len      = length;
	msg.buf      = data;
	return i2c_transfer(client->adapter,&msg, 1);
}

static int zet62xx_cmd_password(struct i2c_client *client, u8 lo, u8 hi)
{
	u8 cmd[2];

	cmd[0] = hi;
	cmd[1] = lo;

	return i2c_smbus_write_i2c_block_data(client,
					      ZET62XX_CMD_WRITE_PASSWORD,
					      2, cmd);
}

static int zet62xx_load_fw(struct i2c_client *client)
{
	struct zet62xx_ts_data *data = i2c_get_clientdata(client);
	int ret;

	ret = request_firmware(&data->fw, data->fw_name, &client->dev);
	if (ret) {
		dev_err(&client->dev, "Firmware request error %d\n", ret);
		return ret;
	}

	data->flash_buffer = (u8 *) data->fw->data;

	pr_info("loading fw name:%s fw size:%x\n",
		data->fw_name, data->fw->size);

	return 0;
}

static void zet62xx_release_fw(struct i2c_client *client)
{
	struct zet62xx_ts_data *data = i2c_get_clientdata(client);

	data->flash_buffer = NULL;

	pr_info("releasing fw name:%s\n", data->fw_name);

	release_firmware(data->fw);
}

static int zet62xx_cmd_codeoption(struct i2c_client *client)
{
	struct zet62xx_ts_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	u8 buf[ZET62XX_CODE_OPTION_LEN];
	u16 model;
	int ret;

	ret = i2c_smbus_write_byte(client, ZET62XX_CMD_READ_CODE_OPTION);
	if (ret < 0)
		return ret;

	msleep(1);

	ret = i2c_smbus_read_i2c_block_data(client,
					    0,
					    ZET62XX_CODE_OPTION_LEN,
					    buf);
	if (ret < 0) {
		dev_err(dev, "read codeoption error\n");
		return ret;
	}

	model = buf[7] << 8 | buf[6];

	pr_info("zet device model %x\n",model);

	switch(model)
	{
	case 0xFFFF:
		data->config = &chip_config[ZET6221];
		data->flash_tot_len = 0x4000;
		data->fw_name = ZET6221_FW_NAME;
		break;
	case 0x6231:
		data->config = &chip_config[ZET6231];
		data->flash_tot_len = 0x8000;
		data->fw_name = ZET6231_FW_NAME;
		break;
	case 0x6223:
		data->config = &chip_config[ZET6223];
		data->flash_tot_len = 0x10000;
		data->fw_name = ZET6223_FW_NAME;
		break;
	case 0x6251:
		data->config = &chip_config[ZET6251];
		data->flash_tot_len = 0x8000;
		data->fw_name = ZET6251_FW_NAME;
		break;
	case 0x6270:
		data->config = &chip_config[ZET6270];
		data->flash_tot_len = 0xA000;
		data->fw_name = ZET6270_FW_NAME;
		break;
	default:
		data->config = &chip_config[ZET6223];
		data->flash_tot_len = 0x8000;
		data->fw_name = ZET6221_FW_NAME;
		break;
	}

	dev_info(&client->dev, "chip id:%x rom_type:%x\n",
		 data->config->chip_id, data->config->rom_type);

	return 0;
}


///**********************************************************************
///   [function]:  zet622x_ts_parse_dynamic_finger
///   [parameters]: i2c_client
///   [return]: void
///**********************************************************************
static void zet622x_ts_data_to_dynamic_finger(u8 *pData)
{
	int i;
	u16 valid;
	u8 pressed;
	valid = pData[1];
	valid = (valid << 8) | pData[2];
	/// parse the finger data
	/// parse the valid data to finger report data
	for(i = 0; i < finger_num; i++)
	{
		pressed = (valid >> (MAX_FINGER_NUMBER-i-1)) & 0x1;
		/// keep the last point data
		finger_report[i].last_report_x = finger_report[i].report_x;
		finger_report[i].last_report_y = finger_report[i].report_y;
		finger_report[i].last_report_z = finger_report[i].report_z;
		/// get the finger data
		finger_report[i].report_x = (u8)((pData[FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*i])>>4)*256 + (u8)pData[(FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*i)+1];
		finger_report[i].report_y = (u8)((pData[FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*i]) & 0x0f)*256 + (u8)pData[(FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*i)+2];
		finger_report[i].report_z = (u8)((pData[(FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*i)+3]) & 0xff);
		finger_report[i].valid = pressed;
	}
	//if key enable
	if(key_num > 0)
	{
		finger_report_key = pData[FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*finger_num];
	}
	//Hover Flag
	hover_status = (pData[6] >> 7);
}
///**********************************************************************
///   [function]:  zet622x_ts_parse_dynamic_finger
///   [parameters]: i2c_client
///   [return]: void
///**********************************************************************
static u8 zet622x_ts_parse_dynamic_finger(struct i2c_client *client)
{
	u8  ts_data[70];
	int ret;
	memset(ts_data,0,70);
	ret = zet622x_i2c_read_tsdata(client, &ts_data[0], finger_packet_size);
	if(ts_data[0] != FINGER_REPROT_DATA_HEADER)
	{
		return FALSE;
	}
	zet622x_ts_data_to_dynamic_finger(ts_data);
	return TRUE;
}
///**********************************************************************
///   [function]:  zet622x_ts_finger_up_report
///   [parameters]: ts,  index
///   [return]: void
///**********************************************************************
static void zet622x_ts_finger_up_report(struct zet62xx_ts_data *ts, int index)
{
	if(pre_event[index].pressed == FALSE)  ///< check the pre-finger status is up
	{
		return;
	}
	pre_event[index].pressed = FALSE;
	input_mt_slot(ts->input_dev, index);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
}
///**********************************************************************
///   [function]:  zet62xx_ts_finger_down_report
///   [parameters]: ts, index
///   [return]: void
///**********************************************************************
static void zet62xx_ts_finger_down_report( struct zet62xx_ts_data *ts, int index, struct finger_coordinate_struct* report_data)
{
#ifdef FEATURE_BTN_TOUCH
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif ///< for FEATURE_BTN_TOUCH
	/// check the pre-finger status is pressed and X,Y is same, than skip report to the host
	if((pre_event[index].pressed == TRUE) &&
	(pre_event[index].pre_x == report_data[index].report_x) &&
	(pre_event[index].pre_y == report_data[index].report_y))
	{
		return;
	}
	/// Send finger down status to host
	pre_event[index].pressed = TRUE;
	pre_event[index].pre_x = report_data[index].report_x;
	pre_event[index].pre_y =  report_data[index].report_y;
	pre_event[index].pre_z =  report_data[index].report_z;
	input_mt_slot(ts->input_dev, index);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	input_report_abs(ts->input_dev,ABS_MT_PRESSURE,  PRESSURE_CONST);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, index);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, PRESSURE_CONST);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X,  report_data[index].report_x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,  report_data[index].report_y);
}
///**********************************************************************
///   [function]:  zet622x_ts_key_report
///   [parameters]: ts, key index
///   [return]: void
///**********************************************************************
static void zet622x_ts_key_report(struct zet62xx_ts_data *ts, u8 ky)
{
	int i;
	u8 pressed;
	if(key_num <= 0)
	{
		return;
	}
	for(i = 0 ; i < KEY_NUMBER ; i++)
	{
		pressed = ky & ( 0x01 << i );
		switch(i)
		{
			case 0:
				if(pressed == 0x01)
				{
					if(!key_back_pressed)
					{
						input_report_key(ts->input_dev, KEY_BACK, 1);
						key_back_pressed = 0x1;
					}
				}
				else
				{
					if(key_back_pressed)
					{
						input_report_key(ts->input_dev, KEY_BACK, 0);
						key_back_pressed = 0x0;
					}
				}
				break;
			case 1:
				if(pressed == 0x02)
				{
					if(!key_home_pressed)
					{
						input_report_key(ts->input_dev, KEY_HOMEPAGE, 1);
						key_home_pressed = 0x1;
					}
				}
				else
				{
					if(key_home_pressed)
					{
						input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
						key_home_pressed = 0x0;
					}
				}
				break;
			case 2:
				if(pressed == 0x04)
				{
					if(!key_menu_pressed)
					{
						input_report_key(ts->input_dev, KEY_MENU, 1);
						key_menu_pressed = 0x1;
					}
				}
				else
				{
					if(key_menu_pressed)
					{
						input_report_key(ts->input_dev, KEY_MENU, 0);
						key_menu_pressed = 0x0;
					}
				}
				break;
		}
		input_sync(ts->input_dev); //jack.chen  20141112
	}
}
///**********************************************************************
///   [function]:  zet622x_ts_finger_report
///   [parameters]: work
///   [return]: void
///**********************************************************************
static void zet622x_ts_finger_report(struct zet62xx_ts_data *ts)
{
	int i;
	u8 finger_cnt = 0;
	u8 chk_finger = FALSE;
	u8 ky = finger_report_key;
	///-------------------------------------------///
	/// check have finger data
	///-------------------------------------------///
	for(i = 0; i < finger_num; i++)
	{
		if(finger_report[i].valid == TRUE)
		{
			chk_finger = TRUE;
			finger_cnt = finger_cnt + 1;
		}
	}
	///-------------------------------------------///
	/// all finger up report
	///-------------------------------------------///
	if(chk_finger == FALSE)
	{
		/// finger up debounce check
		finger_up_cnt++;
		if(finger_up_cnt >= DEBOUNCE_NUMBER)
		{
			finger_up_cnt = 0;
#ifdef FEATURE_BTN_TOUCH
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif ///< for FEATURE_BTN_TOUCH
			for(i = 0; i < finger_num; i++)
			{
				/// finger up setting
				zet622x_ts_finger_up_report(ts, i);
			}
			input_mt_report_pointer_emulation(ts->input_dev, true);
		}
	}
	else
	{
		///-------------------------------------------///
		/// parse finger report
		///-------------------------------------------///
		finger_up_cnt = 0;
		for(i = 0 ; i < finger_num ; i++)
		{
			if(finger_report[i].valid == TRUE)
			{
				/// finger down setting
				zet62xx_ts_finger_down_report(ts, i, finger_report);
			}
			else
			{
				/// finger up setting
				zet622x_ts_finger_up_report(ts, i);
			}
		}
		input_mt_report_pointer_emulation(ts->input_dev, true);
	}
	zet622x_ts_key_report(ts, ky);
	input_sync(ts->input_dev);
}

static irqreturn_t zet622x_ts_interrupt(int irq, void *dev_id)
{
	struct zet62xx_ts_data *data = dev_id;

	if (gpio_get_value(data->gpio) == 0) {
		if (zet622x_ts_parse_dynamic_finger(data->client) != TRUE)
			return IRQ_HANDLED;

		zet622x_ts_finger_report(data);
	}

	return IRQ_HANDLED;
}

static int zet62xx_cmd_sfrunlock(struct i2c_client *client)
{
	int ret;
	u8 buf[ZET62XX_SFR_DATA_LEN + 1];

	ret = i2c_smbus_read_i2c_block_data(client, ZET62XX_CMD_READ_SFR,
					    ZET62XX_SFR_DATA_LEN, buf);
	if (ret < 0)
		return ret;

	buf[ZET62XX_SFR_DATA_LEN - 2] = ZET62XX_SFR_UNLOCK_FLASH;

	ret = i2c_smbus_write_i2c_block_data(client, ZET62XX_CMD_WRITE_SFR,
					     ZET62XX_SFR_DATA_LEN, buf);
	if (ret < 0)
		return ret;

	return 0;
}

static int zet62xx_cmd_pageerase(struct i2c_client *client, int npage)
{
	struct zet62xx_ts_data *data = i2c_get_clientdata(client);

	switch(data->config->chip_id)
	{
	case ZET6221:
		return i2c_smbus_write_byte_data(client, ZET62XX_CMD_PAGE_ERASE,
						      npage);
	case ZET6223:
	case ZET6231:
	case ZET6251:
	case ZET6270:
	default:
		return i2c_smbus_write_word_data(client, ZET62XX_CMD_PAGE_ERASE,
						      npage);
	}

	return 0;
}

static int zet62xx_cmd_writepage(struct i2c_client *client,
				 int pageid, u8 *buf, int len)
{
	struct i2c_msg msg;
	struct zet62xx_ts_data *data = i2c_get_clientdata(client);
	u8 tx_buf[ZET62XX_MAX_PAGE_LEN], index;
	int i;

	switch(data->config->chip_id)
	{
	case ZET6221:
		tx_buf[0] = ZET62XX_CMD_PAGE_WRITE;
		tx_buf[1] = pageid;
		index = 2;
		break;
	case ZET6223:
	case ZET6231:
	case ZET6251:
	case ZET6270:
	default:
		tx_buf[0] = ZET62XX_CMD_PAGE_WRITE;
		tx_buf[1] = pageid & 0xff;
		tx_buf[2] = pageid >> 8;
		index = 3;
		break;
	}

	for (i = 0; i < len; i++)
		tx_buf[i + index] = buf[i];

	msg.addr     = client->addr;
	msg.flags    = 0;
	msg.len      = len + index;
	msg.buf      = tx_buf;

	return i2c_transfer(client->adapter,&msg, 1);

}

static int zet62xx_cmd_read_chksum(struct i2c_client *client, int pageid,
				   u8 *buf)
{
	int ret;

	ret = i2c_smbus_write_word_data(client, ZET62XX_CMD_CALC_CHKSUM,
					pageid);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte_data(client, ZET62XX_CMD_READ_CHKSUM);
	if (ret < 0)
		return ret;

	*buf = ret;

	return 0;
}

static int zet62xx_cmd_sram_chksum(struct i2c_client *client, int pageid,
				   u8 *buf, int len)
{
	int i, ret;
	u8 current_chksum, actual_chksum;

	if (!buf || len <= 0)
		return -EINVAL;

	current_chksum = buf[0];

	for (i = 1; i < len; i++)
		current_chksum = current_chksum ^ buf[i];

	ret = zet62xx_cmd_read_chksum(client, pageid, &actual_chksum);
	if (ret < 0)
		return ret;

	if (current_chksum != actual_chksum) {
		dev_err(&client->dev, "failed checksum page:%d cur:%d==act:%d\n",
			 pageid, current_chksum, actual_chksum);
		return -EIO;
	}
	else
		return 0;
}

static int zet62xx_fw_download_recover(struct zet62xx_ts_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	int ret;

	/* reset the device */
	zet62xx_ts_reset(data);

	/* 2. send password */
	ret = zet62xx_cmd_password(client, ZET62XX_PASSWORD_LOBYTE,
				   ZET62XX_PASSWORD_HIBYTE);
	if (ret < 0) {
		dev_err(dev, "fw send password failed\n");
		return ret;
	}

	if (data->config->rom_type == ZET62XX_ROM_TYPE_UNKNOWN)
		return ret;

	if (data->config->rom_type == ZET62XX_ROM_TYPE_FLASH) {
		ret = zet62xx_cmd_sfrunlock(client);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int zet622x_downloader( struct i2c_client *client)
{
	int ret;
	int offset;
	int flash_rest_len	= 0;
	int flash_page_id	= 0;
	struct zet62xx_ts_data *data = i2c_get_clientdata(client);

	/* 1. set the reset pin low */
	gpiod_set_value_cansleep(data->gpio_reset, 0);

	msleep(1);

	/* 2. send password */
	ret = zet62xx_cmd_password(client, ZET62XX_PASSWORD_LOBYTE,
				   ZET62XX_PASSWORD_HIBYTE);
	if (ret < 0) {
		dev_err(&client->dev, "fw send password failed %d\n", ret);
		goto exit_fw_download;
	}

	msleep(10);

	/* 3. read code option and assign flash buffer */
	ret = zet62xx_cmd_codeoption(client);
	if (ret < 0)
		goto exit_fw_download;

	/* 4 for 6223 need to unlock write protect of 0xFC00~0XFFF */
	if (data->config->chip_id == ZET6223) {
		ret = zet62xx_cmd_password(client, ZET62XX_PASSWORD_1K_LOBYTE,
					   ZET62XX_PASSWORD_1K_HIBYTE);
		if (ret < 0)
			goto exit_fw_download;
	}

	/*TODO: check the flash version and exit download if necessary*/
	/* 5. unlock the SFR */
	if (data->config->rom_type == ZET62XX_ROM_TYPE_FLASH) {
		ret = zet62xx_cmd_sfrunlock(client);
		if (ret < 0)
			goto exit_fw_download;

		/* flash erase */
		ret = i2c_smbus_write_byte(client, ZET62XX_CMD_MASS_ERASE);
		if (ret < 0)
			goto exit_fw_download;
	}

	ret = zet62xx_load_fw(client);
    if (ret < 0)
        goto exit_fw_download;

	flash_rest_len = data->flash_tot_len;

	while(flash_rest_len > 0)
	{
		/* 6. Erase flash page */
		if (data->config->rom_type == ZET62XX_ROM_TYPE_FLASH) {
			ret = zet62xx_cmd_pageerase(client, flash_page_id);
			if (ret < 0)
				goto exit_fw_download;
		}
		offset = flash_page_id * ZET62XX_FLASH_PAGE_LEN;

		/* 7. Write flash page */
		ret = zet62xx_cmd_writepage(client, flash_page_id,
					    &data->flash_buffer[offset],
					    ZET62XX_FLASH_PAGE_LEN);
		if (ret <= 0)
			goto exit_fw_recover;

		ret = zet62xx_cmd_sram_chksum(client, flash_page_id,
					      &data->flash_buffer[offset],
					      ZET62XX_FLASH_PAGE_LEN);
		if (ret < 0)
			goto exit_fw_download;

		flash_rest_len -= ZET62XX_FLASH_PAGE_LEN;
		flash_page_id += 1;
	}

	goto exit_fw_download;
exit_fw_recover:
	zet62xx_fw_download_recover(data);
exit_fw_download:
	zet62xx_release_fw(client);
	gpiod_set_value_cansleep(data->gpio_reset, 1);

	zet62xx_ts_wakeup(data);

	msleep(20);

	return ret;
}

static int zet62xx_ts_remove(struct i2c_client *client)
{
	struct zet62xx_ts_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_PM
	device_remove_file(&client->dev, &dev_attr_power_HAL_suspend);
	unregister_power_hal_suspend_device(&client->dev);
#endif

	pr_crit("[ZET] : ==zet62xx_ts_remove=\n");
	input_unregister_device(data->input_dev);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static int zet62xx_ts_gpio_init(struct i2c_client *client)
{
	struct zet62xx_ts_data *data = i2c_get_clientdata(client);
	int ret, gpio_pin;

	/* configure reset pin */
	gpio_pin = of_get_named_gpio_flags(client->dev.of_node,
					   ZET62XX_RESET_GPIO_NAME, 0, NULL);
	if (gpio_pin <= 0) {
		dev_err(&client->dev,
			"error getting gpio for %s\n", ZET62XX_RESET_GPIO_NAME);
		return -EINVAL;
	}

	TS_RST_GPIO = gpio_pin;
	data->gpio_reset = gpio_to_desc(gpio_pin);

	ret = gpio_request(gpio_pin, ZET62XX_RESET_GPIO_NAME);
	if(ret < 0)
		return ret;

	gpio_pin = of_get_named_gpio_flags(client->dev.of_node,
					   ZET62XX_IRQ_GPIO_NAME, 0, NULL);
	if (gpio_pin <= 0) {
		dev_err(&client->dev,
			"error getting gpio for %s\n", ZET62XX_IRQ_GPIO_NAME);
		return -EINVAL;
	}

	TS_INT_GPIO = gpio_pin;
	data->gpio_irq = gpio_to_desc(gpio_pin);

	pr_crit("ZET irq_pin = %d reset_pin = %d\n", TS_INT_GPIO, TS_RST_GPIO);

	return 0;
}

static int zet62xx_ts_pinctrl_setup(struct i2c_client *client)
{
	struct pinctrl *pinctrl;
	struct pinctrl_state *default_state;

	if (!client->dev.of_node)
		return 0;

	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl))
		return PTR_ERR(pinctrl);

	default_state = pinctrl_lookup_state(pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(default_state))
		return PTR_ERR(default_state);

	return pinctrl_select_state(pinctrl, default_state);
}

static  int zet62xx_ts_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int ret;
	struct input_dev *input_dev;
	struct zet62xx_ts_data *data;

	pr_crit("[ZET]: Probe Zet62xx\n");

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	data->config = &chip_config[ZET6221];
	i2c_set_clientdata(client, data);

	if (zet62xx_ts_pinctrl_setup(client) < 0) {
		dev_warn(&client->dev, "setting pin ctrl state failed\n");
	}

	ret = zet62xx_ts_gpio_init(client);
	if (ret < 0) {
		dev_err(&client->dev, "gpio init failed\n");
		return ret;
	}

	data->gpio = TS_INT_GPIO;

	/* input device init and registration */
	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev || !data)
	{
		return -ENOMEM;
	}
	input_dev->name       = ZET_TS_ID_NAME;
	input_dev->phys = "zet6221_touch/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor  = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;

	ret = zet622x_downloader(client);
	if (ret < 0) {
		dev_err(&client->dev, "firmware download failed...retrying\n");
		ret = zet622x_downloader(client);
		if (ret < 0) {
		dev_err(&client->dev, "firmware download retry failed\n");
			return ret;
		}
	}

	zet62xx_ts_wakeup(data);

	resolution_x = X_MAX;
	resolution_y = Y_MAX;
	finger_num   = FINGER_NUMBER;
	key_num      = KEY_NUMBER;

	if(key_num == 0)
		finger_packet_size  = 3 + 4*finger_num;
	else
		finger_packet_size  = 3 + 4*finger_num + 1;

	pr_crit( "[ZET] : resolution= (%d x %d ), finger_num=%d, key_num=%d\n",
		 resolution_x,resolution_y,finger_num,key_num);

	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	input_mt_init_slots(input_dev, MT_FINGER_NUMBER, 0);
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X,  input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y,  input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, P_MAX, 0, 0);

	/* set virtual key */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, resolution_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, resolution_y, 0, 0);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_HOMEPAGE, input_dev->keybit);
	set_bit(KEY_SEARCH, input_dev->keybit);
	input_dev->evbit[0] = BIT(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	ret = input_register_device(input_dev);
	if (ret < 0)
		return ret;

	data->input_dev = input_dev;
	input_set_drvdata(data->input_dev, data);

	data->irq = client->irq;
	pr_crit( "[ZET]: zet6221_ts_probe.gpid_to_irq [zet6221_ts->irq=%d]\n", data->irq);

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					zet622x_ts_interrupt,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					client->name, data);
	if (ret) {
		dev_err(&client->dev, "IRQ request failed %d\n", ret);
		return ret;
	}

#ifdef CONFIG_PM
	ret = device_create_file(&client->dev, &dev_attr_power_HAL_suspend);
	if (ret < 0) {
		dev_err(&client->dev, "unable to create suspend entry");
		goto out;
	}

	ret = register_power_hal_suspend_device(&client->dev);
	if (ret < 0)
		dev_err(&client->dev, "unable to register for power hal");
out:
#endif

	pr_crit("[ZET] : zet62xx probe ok........");

	return 0;
}

static const struct i2c_device_id zet62xx_ts_idtable[] = {
      { "ZET6221", ZET6221 },
      { }
};
MODULE_DEVICE_TABLE(i2c, zet62xx_ts_idtable);

static const struct acpi_device_id zet62xx_ts_acpi_match[] = {
	{ "ZET6221", ZET6221 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, zet62xx_ts_acpi_match);

static struct i2c_driver zet62xx_ts_driver = {
	.probe = zet62xx_ts_probe,
	.remove	= zet62xx_ts_remove,
	.id_table = zet62xx_ts_idtable,
	.driver = {
		.owner = THIS_MODULE,
		.name = ZET_TS_ID_NAME,
		.acpi_match_table = ACPI_PTR(zet62xx_ts_acpi_match),
	},

};
module_i2c_driver(zet62xx_ts_driver);
MODULE_DESCRIPTION("ZET6221 I2C Touch Screen driver");
MODULE_LICENSE("GPL v2");
