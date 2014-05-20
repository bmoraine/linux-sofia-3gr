/*
 * Copyright (C) 2013 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */


#ifndef _ACC_CNF_H
#define _ACC_CNF_H

#include <linux/switch.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/consumer.h>
#include <linux/iio/driver.h>

#define XGOLD_HEADSET_MODULE_NAME		"headset-xgold"


struct xgold_headset_device_data {
	struct gpio_switch_platform_data *headset_data;
	struct switch_dev sdev;
	struct delayed_work work;
	struct delayed_work hook_work;
	void __iomem *mmio_base;
	struct input_dev *input_dev;
	struct iio_channel *iio_client;
	struct hook_key_cfg *hook_cfg;
	int size_of_hook_cfg;
	struct wake_lock acc_lock;
	const struct xgold_headset_platform_data *pdata;
	unsigned irq_headset;
	unsigned irq_hook;
};

struct hook_key_cfg {
	int min_mv;
	int max_mv;
	int key_code;
	bool pressed;
};

/**
 * Different states of the Headset
 * XGOLD_DET_HEADSET_INS
 *			Nothing is Inserted
 *			Ready to detect Accessory Insertion
 *
 * XGOLD_DET_HEADSET_REM_HOOK_PRESS
 *			Accessory is Inserted
 *			Ready to detect Accessory Removal or Hook Press
 *
 * XGOLD_DET_HOOK_RELEASE
 *			Accessory is Inserted and Hook Key is Pressed
 *			Ready to detect Hook Key Release
**/
enum xgold_headset_config {
	XGOLD_DET_HEADSET_INS,
	XGOLD_DET_HEADSET_REM,
	XGOLD_DET_HEADSET_REM_HOOK_PRESS,
	XGOLD_DET_HOOK_RELEASE,
};

/**
 * Different sub-devices supported
**/
enum xgold_device {
	XGOLD_HEADSET,
	XGOLD_HOOK,
};

/**
 * Different VBIAS settings
**/
enum xgold_vbias {
	XGOLD_VBIAS_ENABLE,
	XGOLD_VBIAS_ULP_ON,
};

enum xgold_headset_type {
	XGOLD_HEADSET_REMOVED,
	XGOLD_HEAD_SET,
	XGOLD_HEAD_PHONE,
	XGOLD_INVALID,
};
/**
 * Platform Data Structure
**/
struct xgold_headset_platform_data {

	/* Confgure the sub-device to detect
	 * 1. Accessory Insertion/Removal
	 * 2. Hook Press/Release
	**/
	void (*xgold_configure_headset)(
			enum xgold_headset_config val,
			struct xgold_headset_device_data *headset_device_data);

	/* Read the sub-device state
	 * 1. Accessory Inserted/Removed
	 * 2. Hook Pressed/Released
	**/
	int (*xgold_read_state)(
			enum xgold_device dev,
			struct xgold_headset_device_data *headset_device_data,
			int volt);

	/* Call to AFE to change the VBIAS settings */
	void (*xgold_configure_vbias)(
			struct xgold_headset_device_data *headset_device_data,
			enum xgold_vbias state);


	/* Check and Scheduling Delayed work */
	void (*xgold_headset_keymap)(
		struct xgold_headset_device_data *headset_device_data);
};

extern struct xgold_headset_platform_data headset_platform_data;

#endif
