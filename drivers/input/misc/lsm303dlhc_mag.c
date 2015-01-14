/*
 *   Adaptations were made to original lsm303dlhc_mag.c with
 *   Copyright (C) 2012-2014 Intel Mobile Communications GmbH
 */
/******************** (C) COPYRIGHT 2010 STMicroelectronics *******************
*
* File Name	: lsm303dlhc_mag_sys.c
* Authors	: MSH - Motion Mems BU - Application Team
*		: Carmine Iascone (carmine.iascone@st.com)
*		: Matteo Dameno (matteo.dameno@st.com)
*		: Both authors are willing to be considered the contact
*		: and update points for the driver.*
* Version	: V.1.0.12
* Date		: 2012/Feb/29
* Description	: LSM303DLHC 6D module sensor device driver sysfs
*
*******************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************
 Revision 1.0.7: 2010/Nov/22
  corrects bug in enable/disable of polling polled device;
 Revision 1.0.9: 2011/May/23
SLEEP_MODE correction; update_odr func correct.; get/set_polling_rate f. cor.
 Revision 1.0.10: 2011/Aug/16
  introduces default_platform_data, i2c_read and i2c_write function rewritten,
  manages smbus beside i2c; sensitivities correction;
 Revision 1.0.11: 2012/Jan/09
  moved under input/misc
 Revision 1.0.12: 2012/Feb/29
  moved use_smbus inside status struct;
******************************************************************************/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input-polldev.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/input/lsm303dlhc.h>

/** Maximum polled-device-reported g value */
#define H_MAX			8100

/* Magnetometer registers */
#define CRA_REG_M		(0x00)	/* Configuration register A */
#define CRB_REG_M		(0x01)	/* Configuration register B */
#define MR_REG_M		(0x02)	/* Mode register */

/* resume state index */
#define RES_CRA_REG_M		0	/* Configuration register A */
#define RES_CRB_REG_M		1	/* Configuration register B */
#define RES_MR_REG_M		2	/* Mode register */

/* Output register start address*/
#define OUT_X_M			(0x03)

/* Magnetic Sensor Operation Mode */
#define NORMAL_MODE		(0x00)
#define POS_BIAS		(0x01)
#define NEG_BIAS		(0x02)
#define CC_MODE			(0x00)
#define SC_MODE			(0x01)
#define SLEEP_MODE		(0x03)

/* Magnetometer X-Y sensitivity  */
#define XY_SENSITIVITY_1_3	1100	/* XY sensitivity at 1.3G */
#define XY_SENSITIVITY_1_9	 855	/* XY sensitivity at 1.9G */
#define XY_SENSITIVITY_2_5	 670	/* XY sensitivity at 2.5G */
#define XY_SENSITIVITY_4_0	 450	/* XY sensitivity at 4.0G */
#define XY_SENSITIVITY_4_7	 400	/* XY sensitivity at 4.7G */
#define XY_SENSITIVITY_5_6	 330	/* XY sensitivity at 5.6G */
#define XY_SENSITIVITY_8_1	 230	/* XY sensitivity at 8.1G */

/* Magnetometer Z sensitivity  */
#define Z_SENSITIVITY_1_3	 980	/* Z sensitivity at 1.3G */
#define Z_SENSITIVITY_1_9	 760	/* Z sensitivity at 1.9G */
#define Z_SENSITIVITY_2_5	 600	/* Z sensitivity at 2.5G */
#define Z_SENSITIVITY_4_0	 400	/* Z sensitivity at 4.0G */
#define Z_SENSITIVITY_4_7	 355	/* Z sensitivity at 4.7G */
#define Z_SENSITIVITY_5_6	 295	/* Z sensitivity at 5.6G */
#define Z_SENSITIVITY_8_1	 205	/* Z sensitivity at 8.1G */

/* Magnetometer output data rate  */
#define LSM303DLHC_MAG_ODR_75		(0x00)	/* 0.75Hz output data rate */
#define LSM303DLHC_MAG_ODR1_5		(0x04)	/* 1.5Hz output data rate */
#define LSM303DLHC_MAG_ODR3_0		(0x08)	/* 3Hz output data rate */
#define LSM303DLHC_MAG_ODR7_5		(0x0C)	/* 7.5Hz output data rate */
#define LSM303DLHC_MAG_ODR15		(0x10)	/* 15Hz output data rate */
#define LSM303DLHC_MAG_ODR30		(0x14)	/* 30Hz output data rate */
#define LSM303DLHC_MAG_ODR75		(0x18)	/* 75Hz output data rate */
#define LSM303DLHC_MAG_ODR220		(0x1C)	/* 220Hz output data rate */

#define FUZZ			0
#define FLAT			0
#define	I2C_AUTO_INCREMENT		(0x80)

#define MS_TO_NS(value) ktime_set(value/1000, (value % 1000)*1000000)

/* #define DEBUG 0 */

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {

	{	LSM303DLHC_MAG_MIN_POLL_PERIOD_MS,	LSM303DLHC_MAG_ODR220},
	{	14,	LSM303DLHC_MAG_ODR75},
	{	34,	LSM303DLHC_MAG_ODR30},
	{	67,	LSM303DLHC_MAG_ODR15},
	{	134,	LSM303DLHC_MAG_ODR7_5},
	{	334,	LSM303DLHC_MAG_ODR3_0},
	{	667,	LSM303DLHC_MAG_ODR1_5},
	{	1334,	LSM303DLHC_MAG_ODR_75},
};


struct lsm303dlhc_mag_status {
	struct i2c_client *client;
	struct lsm303dlhc_mag_platform_data *pdata;

	struct mutex lock;
	struct hrtimer timer;
	struct work_struct input_work;
	struct workqueue_struct *input_work_queue;
	struct input_dev *input_dev;

	int hw_initialized;
	atomic_t enabled;
	int on_before_suspend;
	int use_smbus;

	u16 xy_sensitivity;
	u16 z_sensitivity;

	u8 reg_addr;
	u8 resume_state[3];
};

static const struct lsm303dlhc_mag_platform_data
	default_lsm303dlhc_mag_pdata = {
	.poll_interval = 100,
	.min_interval = LSM303DLHC_MAG_MIN_POLL_PERIOD_MS,
	.fs_range = LSM303DLHC_H_1_3G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
};

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
/* Magnetometer PM states & class */
static struct device_state_pm_state lsm303dlhc_mag_pm_states[] = {
	{ .name = "disable", }, /* D3 */
	{ .name = "enable", }, /* D0 */
};

static int lsm303dlhc_mag_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	/* FIXME */
	return 0;
}

static struct device_state_pm_state *lsm303dlhc_mag_get_initial_state(
		struct device *dev)
{
	return &lsm303dlhc_mag_pm_states[0];
}

static struct device_state_pm_ops lsm303dlhc_mag_pm_ops = {
	.set_state = lsm303dlhc_mag_set_pm_state,
	.get_initial_state = lsm303dlhc_mag_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(lsm303dlhc_mag);
#endif

#ifdef CONFIG_OF
int mag_power_on(struct device *dev)
{
	struct lsm303dlhc_mag_status *stat = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = stat->pdata->pm_platdata;
	int err = device_state_pm_set_state_by_name(dev,
			pm_platdata->pm_state_D0_name);
	if (err)
		dev_err(dev, "Power ON Failed: %d\n", err);

	return err;
}

int mag_power_off(struct device *dev)
{
	struct lsm303dlhc_mag_status *stat = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = stat->pdata->pm_platdata;
	int err = device_state_pm_set_state_by_name(dev,
			pm_platdata->pm_state_D3_name);

	if (err)
		dev_err(dev, "Power OFF Failed:\n");

	return err;
}

static int mag_init(struct device *dev)
{
	struct lsm303dlhc_mag_status *stat = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = stat->pdata->pm_platdata;

	if (!pm_platdata || !pm_platdata->pm_state_D0_name ||
			!pm_platdata->pm_state_D3_name)
		return -EINVAL;

	return device_state_pm_set_class(dev, pm_platdata->pm_user_name);
}

static void mag_exit(void)
{
	return;
}

#define OF_AXIS_MAP		"intel,axis-map"
#define OF_NEGATE		"intel,negate"
#define OF_POLL_INTERVAL	"intel,poll-interval"
#define OF_OFFSET		"intel,offset"

static struct lsm303dlhc_mag_platform_data *lsm303dlhc_mag_of_get_platdata(
		struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct lsm303dlhc_mag_platform_data *mag_pdata;
	u32 out_values[3];
	int ret = 0;

	mag_pdata = kzalloc(sizeof(*mag_pdata), GFP_KERNEL);

	if (!mag_pdata)
		return ERR_PTR(-ENOMEM);

	/* Axis map properties */
	if (of_property_read_u32_array(np, OF_AXIS_MAP, out_values, 3) < 0) {

		dev_err(&client->dev, "Error parsing %s property of node %s\n",
			OF_AXIS_MAP, np->name);
		goto out;
	}
	mag_pdata->axis_map_x = (u8)out_values[0];
	mag_pdata->axis_map_y = (u8)out_values[1];
	mag_pdata->axis_map_z = (u8)out_values[2];

	/* Negate properties */
	if (of_property_read_u32_array(np, OF_NEGATE, out_values, 3) < 0) {

		dev_err(&client->dev, "Error parsing %s property of node %s\n",
			OF_NEGATE, np->name);
		goto out;
	}
	mag_pdata->negate_x = (u8)out_values[0];
	mag_pdata->negate_y = (u8)out_values[1];
	mag_pdata->negate_z = (u8)out_values[2];

	/* Poll interval property */
	if (of_property_read_u32(np, OF_POLL_INTERVAL,
				&mag_pdata->poll_interval) < 0) {

		dev_err(&client->dev, "Error parsing %s property of node %s\n",
			OF_POLL_INTERVAL, np->name);
		goto out;
	}

	/* Offset properties */
	if (of_property_read_u32_array(np, OF_OFFSET, out_values, 3) < 0) {

		dev_err(&client->dev, "Error parsing %s property of node %s\n",
			OF_OFFSET, np->name);
		goto out;
	}
	mag_pdata->x_offset = (u8)out_values[0];
	mag_pdata->y_offset = (u8)out_values[1];
	mag_pdata->z_offset = (u8)out_values[2];

	/* device pm */
	mag_pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(mag_pdata->pm_platdata)) {
		dev_err(&client->dev, "Error during device state pm init\n");
		ret = -EINVAL;
		goto out;
	}

	mag_pdata->init = mag_init;
	mag_pdata->exit = mag_exit;
	mag_pdata->power_on = mag_power_on;
	mag_pdata->power_off = mag_power_off;

	/* FIXME: set default values */
	mag_pdata->fs_range = LSM303DLHC_H_4_0G;
	mag_pdata->min_interval = LSM303DLHC_MAG_MIN_POLL_PERIOD_MS;

	return mag_pdata;

out:
	kfree(mag_pdata);
	return ERR_PTR(ret);
}
#endif

static int lsm303dlhc_mag_i2c_read(struct lsm303dlhc_mag_status *stat,
				u8 *buf, int len)
{
	int ret, retries = 5;
	u8 cmd = buf[0];
	struct i2c_msg msgs[2];

	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | buf[0]);
	msgs[0].addr = stat->client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &cmd;
	msgs[1].addr = stat->client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

retry:
	ret = i2c_transfer(stat->client->adapter, msgs, 2);
	if (ret < 0) {
		if (retries) {
			retries--;
			pr_err("msg %s i2c read error: %d\n", __func__, ret);
			goto retry;
		}
	}
	return ret;
}

static int lsm303dlhc_mag_i2c_write(struct lsm303dlhc_mag_status *stat,
						u8 *buf, int len)
{
	int ret , retries = 5;
	struct i2c_msg msgs;

	if (len > 1)
		buf[0] = (I2C_AUTO_INCREMENT | buf[0]);
	msgs.addr = stat->client->addr;
	msgs.flags = 0;
	msgs.len = len + 1;
	msgs.buf = buf;

retry:
	ret = i2c_transfer(stat->client->adapter, &msgs, 1);
	if (ret < 0) {
		if (retries--) {
			pr_err("%s i2c write error: %d\n", __func__, ret);
			goto retry;
		}
	}
	return ret;
}

int lsm303dlhc_mag_update_fs_range(struct lsm303dlhc_mag_status *stat,
						u8 new_fs_range)
{
	int err = -1;
	u8 buf[2];

	switch (new_fs_range) {
	case LSM303DLHC_H_1_3G:
		stat->xy_sensitivity = XY_SENSITIVITY_1_3;
		stat->z_sensitivity = Z_SENSITIVITY_1_3;
		break;
	case LSM303DLHC_H_1_9G:
		stat->xy_sensitivity = XY_SENSITIVITY_1_9;
		stat->z_sensitivity = Z_SENSITIVITY_1_9;
		break;
	case LSM303DLHC_H_2_5G:
		stat->xy_sensitivity = XY_SENSITIVITY_2_5;
		stat->z_sensitivity = Z_SENSITIVITY_2_5;
		break;
	case LSM303DLHC_H_4_0G:
		stat->xy_sensitivity = XY_SENSITIVITY_4_0;
		stat->z_sensitivity = Z_SENSITIVITY_4_0;
		break;
	case LSM303DLHC_H_4_7G:
		stat->xy_sensitivity = XY_SENSITIVITY_4_7;
		stat->z_sensitivity = Z_SENSITIVITY_4_7;
		break;
	case LSM303DLHC_H_5_6G:
		stat->xy_sensitivity = XY_SENSITIVITY_5_6;
		stat->z_sensitivity = Z_SENSITIVITY_5_6;
		break;
	case LSM303DLHC_H_8_1G:
		stat->xy_sensitivity = XY_SENSITIVITY_8_1;
		stat->z_sensitivity = Z_SENSITIVITY_8_1;
		break;
	default:
		return -EINVAL;
	}

	if (atomic_read(&stat->enabled)) {

		buf[0] = CRB_REG_M;
		buf[1] = new_fs_range;
		err = lsm303dlhc_mag_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;
		stat->resume_state[RES_CRB_REG_M] = new_fs_range;
	}

	return 0;
}

int lsm303dlhc_mag_update_odr(struct lsm303dlhc_mag_status *stat,
							int poll_interval)
{
	int err = -1;
	int i;
	u8 config[2];

	for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
		if ((odr_table[i].poll_rate_ms <= poll_interval)
							|| (i == 0))
			break;
	}

	config[1] = odr_table[i].mask;
	config[1] |= NORMAL_MODE;

	if (atomic_read(&stat->enabled)) {
		config[0] = CRA_REG_M;
		err = lsm303dlhc_mag_i2c_write(stat, config, 1);
		if (err < 0)
			return err;
		stat->resume_state[RES_CRA_REG_M] = config[1];
	}

	return 0;
}

static int lsm303dlhc_mag_get_data(struct lsm303dlhc_mag_status *stat, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware HxL, HxH, HyL, HyH, HzL, HzH */
	u8 mag_data[6] = { 0 };
	/* x,y,z hardware data */
	int hw_d[3] = { 0 };

	mag_data[0] = OUT_X_M;
	err = lsm303dlhc_mag_i2c_read(stat, mag_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (int) (((mag_data[0]) << 8) | mag_data[1]);
	hw_d[1] = (int) (((mag_data[4]) << 8) | mag_data[5]);
	hw_d[2] = (int) (((mag_data[2]) << 8) | mag_data[3]);
	dev_dbg(&stat->client->dev, "z fresh = %d\n", hw_d[2]);

	hw_d[0] = (hw_d[0] & 0x8000) ? (hw_d[0] | 0xFFFF0000) : (hw_d[0]);
	hw_d[1] = (hw_d[1] & 0x8000) ? (hw_d[1] | 0xFFFF0000) : (hw_d[1]);
	hw_d[2] = (hw_d[2] & 0x8000) ? (hw_d[2] | 0xFFFF0000) : (hw_d[2]);
	dev_dbg(&stat->client->dev, "z = conditioned %d\n", hw_d[2]);

	hw_d[0] -= stat->pdata->x_offset;
	hw_d[1] -= stat->pdata->y_offset;
	hw_d[2] += stat->pdata->z_offset;
	dev_dbg(&stat->client->dev, "z offseted = %d\n", hw_d[2]);

	if (hw_d[0] != 0XF000)
		hw_d[0] = hw_d[0] * 1000 / stat->xy_sensitivity;
	else
		hw_d[0] = 0X8000;
	if (hw_d[1] != 0XF000)
		hw_d[1] = hw_d[1] * 1000 / stat->xy_sensitivity;
	else
		hw_d[1] = 0x8000;
	if (hw_d[2] != 0XF000)
		hw_d[2] = hw_d[2] * 1000 / stat->z_sensitivity;
	else
		hw_d[2] = 0x8000;

	if ((hw_d[stat->pdata->axis_map_x] != 0x8000) &&
			(stat->pdata->negate_x))
		xyz[0] = -hw_d[stat->pdata->axis_map_x];
	else
		xyz[0] = hw_d[stat->pdata->axis_map_x];

	if ((hw_d[stat->pdata->axis_map_y] != 0x8000) &&
			(stat->pdata->negate_y))
		xyz[1] = -hw_d[stat->pdata->axis_map_y];
	else
		xyz[1] = hw_d[stat->pdata->axis_map_y];

	if ((hw_d[stat->pdata->axis_map_z] != 0x8000) &&
			(stat->pdata->negate_z))
		xyz[2] = -hw_d[stat->pdata->axis_map_z];
	else
		xyz[2] = hw_d[stat->pdata->axis_map_z];

	dev_dbg(&stat->client->dev, "x =   %d   y =   %d   z =   %d\n",
			xyz[0], xyz[1], xyz[2]);

	return err;
}

static void lsm303dlhc_mag_report_values(struct lsm303dlhc_mag_status *stat,
					int *xyz)
{
	struct input_dev *input = stat->input_dev;
	input_report_abs(input, ABS_X, xyz[0]);
	input_report_abs(input, ABS_Y, xyz[1]);
	input_report_abs(input, ABS_Z, xyz[2]);
	input_sync(input);
}

static int lsm303dlhc_mag_hw_init(struct lsm303dlhc_mag_status *stat)
{
	int err = -1;
	u8 buf[4];

	buf[0] = CRA_REG_M;
	buf[1] = stat->resume_state[RES_CRA_REG_M];
	buf[2] = stat->resume_state[RES_CRB_REG_M];
	buf[3] = stat->resume_state[RES_MR_REG_M];
	err = lsm303dlhc_mag_i2c_write(stat, buf, 3);

	if (err < 0)
		return err;

	stat->hw_initialized = 1;

	return 0;
}

static void lsm303dlhc_mag_device_power_off(struct lsm303dlhc_mag_status *stat)
{
	int err;
	u8 buf[2] = { MR_REG_M, SLEEP_MODE };

	err = lsm303dlhc_mag_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "soft power off failed\n");

	if (stat->pdata->power_off) {
		stat->pdata->power_off(&stat->input_dev->dev);
		stat->hw_initialized = 0;
	}
}

static int lsm303dlhc_mag_device_power_on(struct lsm303dlhc_mag_status *stat)
{
	int err;
	u8 buf[2] = { MR_REG_M, NORMAL_MODE };

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on(&stat->input_dev->dev);
		if (err < 0)
			return err;
	}
	msleep(50);
	if (!stat->hw_initialized) {
		err = lsm303dlhc_mag_hw_init(stat);
		if (err < 0) {
			lsm303dlhc_mag_device_power_off(stat);
			return err;
		}
	} else {
		err = lsm303dlhc_mag_i2c_write(stat, buf, 1);
	}

	return 0;
}

static int lsm303dlhc_mag_enable(struct lsm303dlhc_mag_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {

		err = lsm303dlhc_mag_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
		hrtimer_start(&stat->timer,
				MS_TO_NS(stat->pdata->poll_interval),
				HRTIMER_MODE_REL);
	}

	return 0;
}

static int lsm303dlhc_mag_disable(struct lsm303dlhc_mag_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		hrtimer_cancel(&stat->timer);
		lsm303dlhc_mag_device_power_off(stat);
	}

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int val;
	struct lsm303dlhc_mag_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lsm303dlhc_mag_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max_t(unsigned int, interval_ms,
			stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	lsm303dlhc_mag_update_odr(stat, interval_ms);
	dev_dbg(&stat->client->dev, " interval_ms = %ld\n", interval_ms);
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303dlhc_mag_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm303dlhc_mag_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm303dlhc_mag_enable(stat);
	else
		lsm303dlhc_mag_disable(stat);

	dev_dbg(dev, "sensor %s\n", val ? "enable" : "disable");

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
			attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(enable_device, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
			attr_get_enable, attr_set_enable),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void lsm303dlhc_mag_input_poll_func(struct work_struct *work)
{
	struct lsm303dlhc_mag_status *stat = container_of(work,
			struct lsm303dlhc_mag_status, input_work);

	int xyz[3] = { 0 };

	int err;

	if (!atomic_read(&stat->enabled))
		return;

	mutex_lock(&stat->lock);
	err = lsm303dlhc_mag_get_data(stat, xyz);
	if (err < 0) {
		dev_err(&stat->client->dev, "get_magnetometer_data failed\n");
		mutex_unlock(&stat->lock);
		return;
	}

	lsm303dlhc_mag_report_values(stat, xyz);

	hrtimer_start(&stat->timer,
			MS_TO_NS(stat->pdata->poll_interval),
			HRTIMER_MODE_REL);
	mutex_unlock(&stat->lock);
}

static enum hrtimer_restart
		lsm303dlhc_mag_hrtimer_callback(struct hrtimer *timer)
{
	struct lsm303dlhc_mag_status *stat =
		container_of(timer, struct lsm303dlhc_mag_status, timer);
	queue_work(stat->input_work_queue, &stat->input_work);
	return HRTIMER_NORESTART;
}

int lsm303dlhc_mag_input_open(struct input_dev *input)
{
	struct lsm303dlhc_mag_status *stat = input_get_drvdata(input);

	dev_dbg(&stat->client->dev, "%s\n", __func__);
	return 0;
}

void lsm303dlhc_mag_input_close(struct input_dev *dev)
{
	struct lsm303dlhc_mag_status *stat = input_get_drvdata(dev);

	dev_dbg(&stat->client->dev, "%s\n", __func__);
	return;
}

static int lsm303dlhc_mag_validate_pdata(struct lsm303dlhc_mag_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int) LSM303DLHC_MAG_MIN_POLL_PERIOD_MS,
						stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
					stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 ||
	    stat->pdata->axis_map_y > 2 || stat->pdata->axis_map_z > 2) {
		dev_err(&stat->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			stat->pdata->axis_map_x, stat->pdata->axis_map_y,
			stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 || stat->pdata->negate_y > 1 ||
	    stat->pdata->negate_z > 1) {
		dev_err(&stat->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			stat->pdata->negate_x, stat->pdata->negate_y,
			stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(&stat->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm303dlhc_mag_input_init(struct lsm303dlhc_mag_status *stat)
{
	int err = -1;

	INIT_WORK(&stat->input_work, lsm303dlhc_mag_input_poll_func);
	stat->input_work_queue =
			create_singlethread_workqueue("lsm303_mag_wq");
	if (!stat->input_work_queue) {
		err = -ENOMEM;
		dev_err(&stat->client->dev,
				"cannot create work queue1: %d\n", err);
		goto err0;
	}
	hrtimer_init(&stat->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->timer.function = lsm303dlhc_mag_hrtimer_callback;
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev, "input device allocate failed\n");
		goto err1;
	}

	stat->input_dev->open = lsm303dlhc_mag_input_open;
	stat->input_dev->close = lsm303dlhc_mag_input_close;

	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);

	input_set_abs_params(stat->input_dev, ABS_X, -H_MAX, H_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Y, -H_MAX, H_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Z, -H_MAX, H_MAX, FUZZ, FLAT);

	stat->input_dev->name = LSM303DLHC_MAG_DEV_NAME;

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(&stat->client->dev,
			"unable to register input  device %s\n",
			stat->input_dev->name);
		goto err2;
	}

	return 0;

err2:
	input_free_device(stat->input_dev);
err1:
	destroy_workqueue(stat->input_work_queue);
err0:
	return err;
}

static void lsm303dlhc_mag_input_cleanup(struct lsm303dlhc_mag_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

static int lsm303dlhc_mag_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lsm303dlhc_mag_status *stat;

	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;

	int err = -1;

	stat = kzalloc(sizeof(struct lsm303dlhc_mag_status), GFP_KERNEL);
	if (stat == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for module data: %d\n", err);
		goto err0;
	}

	/* Support for both I2C and SMBUS adapter interfaces. */
	stat->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			goto err1;
		}
	}

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	stat->client = client;

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL)
		goto err1;

#ifdef CONFIG_OF
	client->dev.platform_data =
		lsm303dlhc_mag_of_get_platdata(client);
	if (IS_ERR(client->dev.platform_data)) {
		err = PTR_ERR(client->dev.platform_data);
		goto err1;
	}
#endif

	if (client->dev.platform_data == NULL) {
		memcpy(stat->pdata, &default_lsm303dlhc_mag_pdata,
							sizeof(*stat->pdata));
		dev_err(&client->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, client->dev.platform_data,
							sizeof(*stat->pdata));
	}

	err = lsm303dlhc_mag_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, stat);

	err = lsm303dlhc_mag_input_init(stat);
	if (err < 0)
		goto err2;

	if (stat->pdata->init) {
		err = stat->pdata->init(&stat->input_dev->dev);
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err2;
		}
	}

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CRA_REG_M] =
				LSM303DLHC_MAG_ODR15 |
				LSM303DLHC_MAG_NORMAL_MODE;
	stat->resume_state[RES_CRB_REG_M] = LSM303DLHC_H_1_3G;
	stat->resume_state[RES_MR_REG_M] = NORMAL_MODE;

	err = lsm303dlhc_mag_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err3;
	}

	atomic_set(&stat->enabled, 1);

	err = lsm303dlhc_mag_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto err4;
	}

	err = lsm303dlhc_mag_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err4;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev, "%s register failed\n",
						LSM303DLHC_MAG_DEV_NAME);
		goto err4;
	}

	lsm303dlhc_mag_device_power_off(stat);

	atomic_set(&stat->enabled, 0);

	mutex_unlock(&stat->lock);

	dev_info(&client->dev, "probed successfully\n");

	return 0;

err4:
	lsm303dlhc_mag_device_power_off(stat);
err3:
	if (stat->pdata->exit)
		stat->pdata->exit();
err2:
	lsm303dlhc_mag_input_cleanup(stat);
err1_1:
	mutex_unlock(&stat->lock);
	kfree(stat->pdata->pm_platdata);
	kfree(stat->pdata);
err1:
	kfree(stat);
err0:
	pr_err("%s: Driver Initialization failed\n", LSM303DLHC_MAG_DEV_NAME);
	return err;
}

static int lsm303dlhc_mag_remove(struct i2c_client *client)
{
	struct lsm303dlhc_mag_status *stat = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s\n", __func__);

	lsm303dlhc_mag_disable(stat);
	lsm303dlhc_mag_input_cleanup(stat);
	remove_sysfs_interfaces(&client->dev);

	if (stat->pdata->exit)
		stat->pdata->exit();

	kfree(stat->pdata);
	kfree(stat);
	return 0;
}

#ifdef CONFIG_PM
static int lsm303dlhc_mag_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_mag_status *stat = i2c_get_clientdata(client);

	dev_dbg(dev, "%s: suspend\n", LSM303DLHC_MAG_DEV_NAME);
	stat->on_before_suspend = atomic_read(&stat->enabled);
	return lsm303dlhc_mag_disable(stat);
}

static int lsm303dlhc_mag_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_mag_status *stat = i2c_get_clientdata(client);

	dev_dbg(dev, "%s: resume\n", LSM303DLHC_MAG_DEV_NAME);

	if (stat->on_before_suspend)
		return lsm303dlhc_mag_enable(stat);

	return 0;
}
#else
#define lsm303dlhc_mag_suspend	NULL
#define lsm303dlhc_mag_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm303dlhc_mag_id[] = {
	{ LSM303DLHC_MAG_DEV_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, lsm303dlhc_mag_id);

static const struct dev_pm_ops lsm303dlhc_mag_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm303dlhc_mag_suspend, lsm303dlhc_mag_resume)
};

static struct i2c_driver lsm303dlhc_mag_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LSM303DLHC_MAG_DEV_NAME,
		.pm = &lsm303dlhc_mag_pm,
	},
	.probe = lsm303dlhc_mag_probe,
	.remove = lsm303dlhc_mag_remove,
	.id_table = lsm303dlhc_mag_id,
};

static int __init lsm303dlhc_mag_init(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&lsm303dlhc_mag_pm_class);
	if (ret) {
		pr_err("%s: %s: ERROR adding %s pm class\n",
				LSM303DLHC_MAG_DEV_NAME, __func__,
				lsm303dlhc_mag_pm_class.name);
		return ret;
	}
#endif
	pr_info("%s magnetometer driver init\n", LSM303DLHC_MAG_DEV_NAME);
	return i2c_add_driver(&lsm303dlhc_mag_driver);
}

static void __exit lsm303dlhc_mag_exit(void)
{
	i2c_del_driver(&lsm303dlhc_mag_driver);
	return;
}

module_init(lsm303dlhc_mag_init);
module_exit(lsm303dlhc_mag_exit);

MODULE_DESCRIPTION("lsm303dlhc sysfs driver for the magnetometer section");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, STMicroelectronics");
MODULE_LICENSE("GPL");
