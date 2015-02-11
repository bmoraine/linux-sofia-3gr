/*
 *  Adaptations were made to original l3gd20_gyr.c with
 *   Copyright (C) 2012-2014 Intel Mobile Communications GmbH
 */
/******************** (C) COPYRIGHT 2012 STMicroelectronics *******************
*
* File Name		: l3gd20_gyr_sysfs.c
* Authors		: MEMS Motion Sensors Products Div- Application Team
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Denis Ciocca (denis.ciocca@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.2.1 sysfs
* Date			: 2012/Jul/10
* Description		: L3GD20 digital output gyroscope sensor API
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
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS	  | DESCRIPTION
* 1.0		| 2010/May/02	| Carmine Iascone | First Release
* 1.1.3		| 2011/Jun/24	| Matteo Dameno	  | Corrects ODR Bug
* 1.1.4		| 2011/Sep/02	| Matteo Dameno	  | SMB Bus Mng,
*		|		|		  | forces BDU setting
* 1.1.5		| 2011/Sep/24	| Matteo Dameno	  | Introduces FIFO Feat.
* 1.1.5.2	| 2011/Nov/11	| Matteo Dameno	  | enable gpio_int to be
*		|		|		  | passed as parameter at
*		|		|		  | module loading time;
*		|		|		  | corrects polling
*		|		|		  | bug at end of probing;
* 1.1.5.3	| 2011/Dec/20	| Matteo Dameno	  | corrects error in
*		|		|		  | I2C SADROOT; Modifies
*		|		|		  | resume suspend func.
* 1.1.5.4	| 2012/Jan/09	| Matteo Dameno	  | moved under input/misc;
* 1.1.5.5	| 2012/Mar/30	| Matteo Dameno	  | moved watermark, use_smbus,
*		|		|		  | fifomode @ struct foo_status
*		|		|		  | sysfs range input format
*		|		|		  | changed to decimal
* 1.2		| 2012/Jul/10	| Denis Ciocca	  | input_poll_dev removal
* 1.2.1		| 2012/Jul/10	| Denis Ciocca	  | added high resolution timers
*******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/stat.h>
#ifndef CONFIG_PINCTRL
#include <mach/gpio.h>
#endif
#include <linux/input/l3gd20.h>

/* Maximum polled-device-reported rot speed value value in dps */
#define FS_MAX		32768
#define MS_TO_NS(x)	(x*1000000L)

/* l3gd20 gyroscope registers */
#define WHO_AM_I	(0x0F)

#define SENSITIVITY_250_NUMERATOR      875              /* 875/100 mdps/LSB */
#define SENSITIVITY_500_NUMERATOR      175              /* 175/10  mdps/LSB */
#define SENSITIVITY_2000_NUMERATOR     70               /*  70/1   mdps/LSB */

#define SENSITIVITY_250_DENOMINATOR     100
#define SENSITIVITY_500_DENOMINATOR     10
#define SENSITIVITY_2000_DENOMINATOR    1

#define CTRL_REG1	(0x20)    /* CTRL REG1 */
#define CTRL_REG2	(0x21)    /* CTRL REG2 */
#define CTRL_REG3	(0x22)    /* CTRL_REG3 */
#define CTRL_REG4	(0x23)    /* CTRL_REG4 */
#define CTRL_REG5	(0x24)    /* CTRL_REG5 */
#define	REFERENCE	(0x25)    /* REFERENCE REG */
#define	FIFO_CTRL_REG	(0x2E)    /* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG	(0x2F)    /* FIFO SOURCE REGISTER */
#define	OUT_X_L		(0x28)    /* 1st AXIS OUT REG of 6 */
#define INT1_CFG		 0x30
#define INT1_THS_XH      0x32
#define INT1_THS_XL      0x33
#define INT1_THS_YH      0x34
#define INT1_THS_YL      0x35
#define INT1_THS_ZH      0x36
#define INT1_THS_ZL      0x37
#define INT1_DURATION    0x38

#define AXISDATA_REG	OUT_X_L

/* CTRL_REG1 */
#define ALL_ZEROES	(0x00)
#define PM_OFF		(0x00)
#define PM_NORMAL	(0x08)
#define ENABLE_ALL_AXES	(0x07)
#define ENABLE_NO_AXES	(0x00)
#define BW00		(0x00)
#define BW01		(0x10)
#define BW10		(0x20)
#define BW11		(0x30)
#define ODR095		(0x00)  /* ODR =  95Hz */
#define ODR190		(0x40)  /* ODR = 190Hz */
#define ODR380		(0x80)  /* ODR = 380Hz */
#define ODR760		(0xC0)  /* ODR = 760Hz */

/* CTRL_REG3 bits */
#define	I2_DRDY		(0x08)
#define	I2_WTM		(0x04)
#define	I2_OVRUN	(0x02)
#define	I2_EMPTY	(0x01)
#define	I2_NONE		(0x00)
#define	I2_MASK		(0x0F)

/* CTRL_REG4 bits */
#define	FS_MASK		(0x30)
#define	BDU_ENABLE	(0x80)

/* CTRL_REG5 bits */
#define	FIFO_ENABLE	(0x40)
#define HPF_ENALBE	(0x11)

/* FIFO_CTRL_REG bits */
#define	FIFO_MODE_MASK		(0xE0)
#define	FIFO_MODE_BYPASS	(0x00)
#define	FIFO_MODE_FIFO		(0x20)
#define	FIFO_MODE_STREAM	(0x40)
#define	FIFO_MODE_STR2FIFO	(0x60)
#define	FIFO_MODE_BYPASS2STR	(0x80)
#define	FIFO_WATERMARK_MASK	(0x1F)

#define FIFO_STORED_DATA_MASK	(0x1F)

#define I2C_AUTO_INCREMENT	(0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_FIFO_CTRL_REG	5
#define	RESUME_ENTRIES		6

#define INT1_ENABLE         0x80
#define INT1_DISABLE        (~INT1_ENABLE)

#define INT1_AND                 0x80
#define INT1_OR                  0x00
#define INT1_LIR_ENABLE          0x40
#define INT1_LIR_DISABLE         0x00
#define INT1_ZHIE_ENABLE         0x20
#define INT1_ZHIE_DISABLE        0x00
#define INT1_ZLIE_ENABLE         0x10
#define INT1_ZLIE_DISABLE        0x00
#define INT1_YHIE_ENABLE         0x08
#define INT1_YHIE_DISABLE        0x00
#define INT1_YLIE_ENABLE         0x04
#define INT1_YLIE_DISABLE        0x00
#define INT1_XHIE_ENABLE         0x02
#define INT1_XHIE_DISABLE        0x00
#define INT1_XLIE_ENABLE         0x01
#define INT1_XLIE_DISABLE        0x00

#define INT_THS_XH_VAL			 0x01
#define INT_THS_XL_VAL			 0x1F

#define INT_THS_YH_VAL			 0x01
#define INT_THS_YL_VAL			 0x1F

#define INT_THS_ZH_VAL			 0x01
#define INT_THS_ZL_VAL			 0x1F

#define INT1_DURATION_VAL		 0x02

/* Registers Contents */
#define WHOAMI_L3GD20_GYR	(0xD4)  /* Expected content for WAI register*/


static int int1_gpio = L3GD20_GYR_DEFAULT_INT1_GPIO;
static int int2_gpio = L3GD20_GYR_DEFAULT_INT2_GPIO;
/* module_param(int1_gpio, int, S_IRUGO); */
module_param(int2_gpio, int, S_IRUGO);

/*
 * L3GD20 gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * s32
 */

struct l3gd20_gyr_triple {
	s32	x,	/* x-axis angular rate data. */
		y,	/* y-axis angluar rate data. */
		z;	/* z-axis angular rate data. */
};

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {

	{	2,	ODR760|BW10},
	{	3,	ODR380|BW01},
	{	6,	ODR190|BW00},
	{	11,	ODR095|BW00},
};

static struct l3gd20_gyr_platform_data default_l3gd20_gyr_pdata = {
	.fs_range = L3GD20_GYR_FS_250DPS,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,

	.poll_interval = 100,
	.min_interval = L3GD20_GYR_MIN_POLL_PERIOD_MS, /* 2ms */

	.gpio_int1 = L3GD20_GYR_DEFAULT_INT1_GPIO,
	.gpio_int2 = L3GD20_GYR_DEFAULT_INT2_GPIO,	/* int for fifo */

};

struct workqueue_struct *l3gd20_gyr_workqueue;

struct l3gd20_gyr_status {
	struct i2c_client *client;
	struct l3gd20_gyr_platform_data *pdata;

	struct mutex lock;

	struct input_dev *input_dev;

	int hw_initialized;
	atomic_t enabled;
	int on_before_suspend;
	int use_smbus;

	u8 reg_addr;
	u8 resume_state[RESUME_ENTRIES];

	u32 sensitivity_numerator;
	u32 sensitivity_denominator;

	/* interrupt related */
	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;

	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	bool polling_enabled;
	/* fifo related */
	u8 watermark;
	u8 fifomode;

	struct hrtimer hr_timer;
	ktime_t ktime;
	struct work_struct polling_task;
};

static int l3gd20_gyr_i2c_read(struct l3gd20_gyr_status *stat, u8 *buf,
							int len)
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

static int l3gd20_gyr_i2c_write(struct l3gd20_gyr_status *stat, u8 *buf,
							int len)
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

static int l3gd20_gyr_register_write(struct l3gd20_gyr_status *stat,
		u8 *buf, u8 reg_address, u8 new_value)
{
	int err;

		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = l3gd20_gyr_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

	return err;
}

static int l3gd20_gyr_register_read(struct l3gd20_gyr_status *stat,
					u8 *buf, u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = l3gd20_gyr_i2c_read(stat, buf, 1);
	return err;
}

static int l3gd20_gyr_register_update(struct l3gd20_gyr_status *stat,
			u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = l3gd20_gyr_register_read(stat, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = l3gd20_gyr_register_write(stat, buf, reg_address,
				updated_val);
	}
	return err;
}

static int l3gd20_gyr_manage_int1settings(struct l3gd20_gyr_status *stat,
								u8 enable)
{
	int err;
	u8 buf[8];

	if (enable) {
		stat->resume_state[RES_CTRL_REG3] =
			stat->resume_state[RES_CTRL_REG3] | INT1_ENABLE;
		buf[0] = (CTRL_REG1);
		buf[1] = stat->resume_state[RES_CTRL_REG1];
		buf[2] = stat->resume_state[RES_CTRL_REG2];
		buf[3] = stat->resume_state[RES_CTRL_REG3];
		buf[4] = stat->resume_state[RES_CTRL_REG4];
		buf[5] = stat->resume_state[RES_CTRL_REG5];

		err = l3gd20_gyr_i2c_write(stat, buf, 5);
		if (err < 0)
			return err;

		buf[0] = (INT1_CFG);
		buf[1] = (INT1_OR | INT1_ZHIE_ENABLE | INT1_XHIE_ENABLE |
							INT1_YHIE_ENABLE);
		err = l3gd20_gyr_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

		buf[0] = (INT1_THS_XH);
		buf[1] = INT_THS_XH_VAL;
		buf[2] = INT_THS_XL_VAL;
		buf[3] = INT_THS_YH_VAL;
		buf[4] = INT_THS_YL_VAL;
		buf[5] = INT_THS_ZH_VAL;
		buf[6] = INT_THS_ZL_VAL;
		buf[7] = INT1_DURATION_VAL;
		err = l3gd20_gyr_i2c_write(stat, buf, 7);
		if (err < 0)
			return err;
	} else {
		stat->resume_state[RES_CTRL_REG3] =
			stat->resume_state[RES_CTRL_REG3] & INT1_DISABLE;
		buf[0] = (CTRL_REG3);
		buf[1] = stat->resume_state[RES_CTRL_REG3];
		err = l3gd20_gyr_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;
	}
	return 0;
}

static int l3gd20_gyr_update_fs_range(struct l3gd20_gyr_status *stat,
							u8 new_fs)
{
	int res;
	u8 buf[2];

	u32 sensitivity_numerator;
	u32 sensitivity_denominator;

	switch (new_fs) {
	case L3GD20_GYR_FS_250DPS:
		sensitivity_numerator = SENSITIVITY_250_NUMERATOR;
		sensitivity_denominator = SENSITIVITY_250_DENOMINATOR;
		break;
	case L3GD20_GYR_FS_500DPS:
		sensitivity_numerator = SENSITIVITY_500_NUMERATOR;
		sensitivity_denominator = SENSITIVITY_500_DENOMINATOR;
		break;
	case L3GD20_GYR_FS_2000DPS:
		sensitivity_numerator = SENSITIVITY_2000_NUMERATOR;
		sensitivity_denominator = SENSITIVITY_2000_DENOMINATOR;
		break;
	default:
		dev_err(&stat->client->dev,
				"invalid g range requested: %u\n", new_fs);
		return -EINVAL;
	}

	buf[0] = CTRL_REG4;

	res = l3gd20_gyr_register_update(stat, buf, CTRL_REG4, FS_MASK, new_fs);

	if (res < 0) {
		dev_err(&stat->client->dev, "%s: failed to update fs:0x%02x\n",
				__func__, new_fs);
		return res;
	}
	stat->resume_state[RES_CTRL_REG4] =
		((FS_MASK & new_fs) |
		(~FS_MASK & stat->resume_state[RES_CTRL_REG4]));

	stat->sensitivity_numerator = sensitivity_numerator;
	stat->sensitivity_denominator = sensitivity_denominator;
	dev_dbg(&stat->client->dev, "Sensitivity: %d/%d\n",
		sensitivity_numerator, sensitivity_denominator);

	return res;
}


static int l3gd20_gyr_update_odr(struct l3gd20_gyr_status *stat,
			unsigned int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
		if ((odr_table[i].poll_rate_ms <= poll_interval_ms)
					|| (i == 0))
			break;
	}

	config[1] = odr_table[i].mask;
	config[1] |= (ENABLE_ALL_AXES + PM_NORMAL);

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		config[0] = CTRL_REG1;
		err = l3gd20_gyr_i2c_write(stat, config, 1);
		if (err < 0)
			return err;
		stat->resume_state[RES_CTRL_REG1] = config[1];
		stat->ktime = ktime_set(0, MS_TO_NS(poll_interval_ms));
	}
	dev_dbg(&stat->client->dev, "ODR %x\n", config[1]);

	return err;
}

/* gyroscope data readout */
static int l3gd20_gyr_get_data(struct l3gd20_gyr_status *stat,
			     struct l3gd20_gyr_triple *data)
{
	int err;
	u8 gyro_out[6] = { 0 };
	/* y,p,r hardware data */
	s32 hw_d[3] = { 0 };

	gyro_out[0] = (AXISDATA_REG);

	err = l3gd20_gyr_i2c_read(stat, gyro_out, 6);

	if (err < 0)
		return err;

	hw_d[0] =  ((s16)((gyro_out[1]) << 8) | gyro_out[0]);
	hw_d[1] =  ((s16)((gyro_out[3]) << 8) | gyro_out[2]);
	hw_d[2] =  ((s16)((gyro_out[5]) << 8) | gyro_out[4]);
	dev_dbg(&stat->client->dev, "gyro_raw: x = %d, y = %d, z = %d\n",
			hw_d[0], hw_d[1], hw_d[2]);

	hw_d[0] = hw_d[0] *
		stat->sensitivity_numerator / stat->sensitivity_denominator;
	hw_d[1] = hw_d[1] *
		stat->sensitivity_numerator / stat->sensitivity_denominator;
	hw_d[2] = hw_d[2] *
		stat->sensitivity_numerator / stat->sensitivity_denominator;

	data->x = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
		   : (hw_d[stat->pdata->axis_map_x]));
	data->y = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
		   : (hw_d[stat->pdata->axis_map_y]));
	data->z = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
		   : (hw_d[stat->pdata->axis_map_z]));

	 dev_dbg(&stat->client->dev, "gyro_out: x = %d, y = %d, z = %d\n",
		data->x, data->y, data->z);
	return err;
}

static void l3gd20_gyr_report_values(struct l3gd20_gyr_status *stat,
					struct l3gd20_gyr_triple *data)
{
	input_report_abs(stat->input_dev, ABS_X, data->x);
	input_report_abs(stat->input_dev, ABS_Y, data->y);
	input_report_abs(stat->input_dev, ABS_Z, data->z);
	input_sync(stat->input_dev);
}

static int l3gd20_gyr_hw_init(struct l3gd20_gyr_status *stat)
{
	int err;
	u8 buf[6];

	dev_dbg(&stat->client->dev, "hw init\n");

	buf[0] = (CTRL_REG1);
	buf[1] = stat->resume_state[RES_CTRL_REG1];
	buf[2] = stat->resume_state[RES_CTRL_REG2];
	buf[3] = stat->resume_state[RES_CTRL_REG3];
	buf[4] = stat->resume_state[RES_CTRL_REG4];
	buf[5] = stat->resume_state[RES_CTRL_REG5];

	err = l3gd20_gyr_i2c_write(stat, buf, 5);
	if (err < 0)
		return err;

	buf[0] = (FIFO_CTRL_REG);
	buf[1] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = l3gd20_gyr_i2c_write(stat, buf, 1);
	if (err < 0)
			return err;

	stat->hw_initialized = 1;

	return err;
}

static void l3gd20_gyr_device_power_off(struct l3gd20_gyr_status *stat)
{
	int err;
	u8 buf[2];

	dev_dbg(&stat->client->dev, "power off\n");

	buf[0] = (CTRL_REG1);
	buf[1] = (PM_OFF);
	err = l3gd20_gyr_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "soft power off failed\n");

	if (stat->pdata->gpio_int1 >= 0)
		disable_irq_nosync(stat->irq1);

	if (stat->pdata->power_off)
		stat->pdata->power_off(&stat->input_dev->dev);

	stat->hw_initialized = 0;
}

static int l3gd20_gyr_device_power_on(struct l3gd20_gyr_status *stat)
{
	int err;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on(&stat->input_dev->dev);
		if (err < 0) {
			dev_err(&stat->client->dev,
					"power on: irq1 enabled\n");
			return err;
		}
	}
	msleep(50);
	if (stat->pdata->gpio_int1 >= 0)
		enable_irq(stat->irq1);

	if (!stat->hw_initialized) {
		err = l3gd20_gyr_hw_init(stat);
		if (err < 0) {
			l3gd20_gyr_device_power_off(stat);
			return err;
		}
	}

	return 0;
}

static int l3gd20_gyr_enable(struct l3gd20_gyr_status *stat)
{
	int err;

	dev_dbg(&stat->client->dev, "%s: stat->enabled = %d\n", __func__,
						atomic_read(&stat->enabled));

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = l3gd20_gyr_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
		err = l3gd20_gyr_manage_int1settings(stat, 1);
		if (err < 0)
			return err;

		if (stat->polling_enabled)
			hrtimer_start(&stat->hr_timer, stat->ktime,
						HRTIMER_MODE_REL);
	}

	return 0;
}

static int l3gd20_gyr_disable(struct l3gd20_gyr_status *stat)
{
	dev_dbg(&stat->client->dev, "%s: stat->enabled = %d\n", __func__,
						atomic_read(&stat->enabled));

	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		if (stat->polling_enabled) {
			hrtimer_cancel(&stat->hr_timer);
			dev_dbg(&stat->client->dev, "%s: cancel_hrtimer ",
					__func__);
		}

		l3gd20_gyr_manage_int1settings(stat, 0);
		l3gd20_gyr_device_power_off(stat);
	}
	return 0;
}

static ssize_t attr_polling_rate_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int err;
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	if (!interval_ms)
		return -EINVAL;

	mutex_lock(&stat->lock);
	err = l3gd20_gyr_update_odr(stat, interval_ms);
	if (err >= 0)
		stat->pdata->poll_interval = interval_ms;
	dev_dbg(&stat->client->dev, "interval_ms %ld\n", interval_ms);
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		l3gd20_gyr_enable(stat);
	else
		l3gd20_gyr_disable(stat);

	dev_dbg(dev, "sensor %s\n", val ? "enable" : "disable");

	return size;
}

static ssize_t attr_polling_mode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int val = 0;
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	if (stat->polling_enabled)
		val = 1;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	if (val) {
		if (!stat->polling_enabled) {
			stat->polling_enabled = true;
			if (atomic_read(&stat->enabled))
				hrtimer_start(&(stat->hr_timer), stat->ktime,
						HRTIMER_MODE_REL);
		}
		dev_info(dev, "polling mode enabled\n");
	} else {
		if (stat->polling_enabled) {
			hrtimer_cancel(&stat->hr_timer);
			stat->polling_enabled = false;
		}
		dev_info(dev, "polling mode disabled\n");
	}
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_reg_get(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	int i, n, err, ofst = 0;
	u8 i2c_buf[25];

	for (i = 0; i < 25; i++) {
		i2c_buf[i] = (0x20 + i);
		err = l3gd20_gyr_i2c_read(stat, &i2c_buf[i], 1);
		if (err < 0) {
			dev_err(dev, "Error in reading gyroscope reg\n");
			return -EINVAL;
		}
	}

	for (i = 0; i < 25; i++) {
		if ((i % 8) == 0) {
			n = sprintf(buf + ofst, "\n0x%02x:", (i + 0x20));
			if (n < 0)
				return 0;
			ofst += n;
		}
		n = sprintf(buf+ofst, "  0x%02x", i2c_buf[i]);
		if (n < 0)
			return 0;
		ofst += n;
	}

	n = sprintf(buf + ofst, "\n");
	if (n < 0)
		return 0;
	ofst += n;

	return ofst;
}

static ssize_t attr_reg_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	int ret;
	unsigned int val, regid;
	u8 i2c_buf[2];

	if (sscanf(buf, "%x", &val) == 0) {
		dev_err(&stat->client->dev,
			"Error in reading register setting!\n");
		return -EINVAL;
	}
	/*
		Format of input value:
			bit[31..16] = don't care
			bit[15..8]  = register address
			bit[8..0]   = value to write into register
	*/
	regid = (val >> 8) & 0xff;
	val &= 0xff;

	if (regid < CTRL_REG1 || regid > INT1_DURATION) {
		dev_err(&stat->client->dev,
			"Register address out of range!\n");
		return -EINVAL;
	}

	i2c_buf[0] = (u8) regid;
	i2c_buf[1] = (u8) val;
	ret = l3gd20_gyr_i2c_write(stat, i2c_buf, 1);
	if (ret < 0)
		dev_err(&stat->client->dev, "Write failed\n");

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
			attr_polling_rate_show,
			attr_polling_rate_store),
	__ATTR(enable_device, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
			attr_enable_show,
			attr_enable_store),
	__ATTR(enable_polling, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
			attr_polling_mode_show,
			attr_polling_mode_store),
	__ATTR(reg, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
			attr_reg_get,
			attr_reg_set),
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

static void l3gd20_gyr_report_triple(struct l3gd20_gyr_status *stat)
{
	int err;
	struct l3gd20_gyr_triple data_out;

	err = l3gd20_gyr_get_data(stat, &data_out);
	if (err < 0)
		dev_err(&stat->client->dev, "get_gyroscope_data failed\n");
	else
		l3gd20_gyr_report_values(stat, &data_out);
}

static irqreturn_t l3gd20_gyr_isr1(int irq, void *dev)
{
	struct l3gd20_gyr_status *stat = dev;
	disable_irq_nosync(irq);

	queue_work(stat->irq1_work_queue, &stat->irq1_work);
	return IRQ_HANDLED;
}

static void l3gd20_gyr_irq1_work_func(struct work_struct *work)
{

	struct l3gd20_gyr_status *stat =
		container_of(work, struct l3gd20_gyr_status, irq1_work);

	l3gd20_gyr_report_triple(stat);
	enable_irq(stat->irq1);
}

int l3gd20_gyr_input_open(struct input_dev *input)
{
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	dev_dbg(&stat->client->dev, "%s\n", __func__);
	return 0;
}

void l3gd20_gyr_input_close(struct input_dev *dev)
{
	struct l3gd20_gyr_status *stat = input_get_drvdata(dev);

	dev_dbg(&stat->client->dev, "%s\n", __func__);
	return;
}

static int l3gd20_gyr_validate_pdata(struct l3gd20_gyr_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int) L3GD20_GYR_MIN_POLL_PERIOD_MS,
						stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
			stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 ||
	    stat->pdata->axis_map_y > 2 ||
	    stat->pdata->axis_map_z > 2) {
		dev_err(&stat->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			stat->pdata->axis_map_x,
			stat->pdata->axis_map_y,
			stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 ||
	    stat->pdata->negate_y > 1 ||
	    stat->pdata->negate_z > 1) {
		dev_err(&stat->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			stat->pdata->negate_x,
			stat->pdata->negate_y,
			stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(&stat->client->dev,
			"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static int l3gd20_gyr_input_init(struct l3gd20_gyr_status *stat)
{
	int err = -1;

	dev_dbg(&stat->client->dev, "%s\n", __func__);

	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev,
			"input device allocation failed\n");
		goto err0;
	}

	stat->input_dev->open = l3gd20_gyr_input_open;
	stat->input_dev->close = l3gd20_gyr_input_close;
	stat->input_dev->name = L3GD20_GYR_DEV_NAME;

	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);

	input_set_abs_params(stat->input_dev, ABS_X, -FS_MAX-1, FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev, ABS_Y, -FS_MAX-1, FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev, ABS_Z, -FS_MAX-1, FS_MAX, 0, 0);

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(&stat->client->dev,
			"unable to register input polled device %s\n",
			stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);
err0:
	return err;
}

static void l3gd20_gyr_input_cleanup(struct l3gd20_gyr_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

static void poll_function_work(struct work_struct *polling_task)
{
	struct l3gd20_gyr_status *stat;
	struct l3gd20_gyr_triple data_out;
	int err;

	stat = container_of((struct work_struct *)polling_task,
				struct l3gd20_gyr_status, polling_task);

	if (!atomic_read(&stat->enabled))
		return;

	err = l3gd20_gyr_get_data(stat, &data_out);
	if (err < 0) {
		dev_err(&stat->client->dev, "get_rotation_data failed.\n");
		return;
	}

	l3gd20_gyr_report_values(stat, &data_out);
	hrtimer_start(&stat->hr_timer, stat->ktime, HRTIMER_MODE_REL);
}

enum hrtimer_restart poll_function_read(struct hrtimer *timer)
{
	struct l3gd20_gyr_status *stat;

	stat = container_of((struct hrtimer *)timer,
				struct l3gd20_gyr_status, hr_timer);

	queue_work(l3gd20_gyr_workqueue, &stat->polling_task);
	return HRTIMER_NORESTART;
}

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int l3gd20_set_pm_state(
		struct device *dev, struct device_state_pm_state *state)
{
	/* FIXME */
	return 0;
}

static struct device_state_pm_state *l3gd20_get_initial_state(
		struct device *);

static struct device_state_pm_ops l3gd20_pm_ops = {
	.set_state = l3gd20_set_pm_state,
	.get_initial_state = l3gd20_get_initial_state,
};

/* APDS990x PM states & class */
static struct device_state_pm_state l3gd20_pm_states[] = {
	{ .name = "disable", }, /* D3 */
	{ .name = "enable", }, /* D0 */
};

DECLARE_DEVICE_STATE_PM_CLASS(l3gd20);

static struct device_state_pm_state *l3gd20_get_initial_state(
		struct device *dev)
{
	return &l3gd20_pm_states[0];
}
#endif

#ifdef CONFIG_OF
#define L3GD20_PM_DISABLE	0
#define L3GD20_PM_ENABLE	1
#define L3GD20_PM_NOF_STATES	2

int gyro_power_on(struct device *dev)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = stat->pdata->pm_platdata;
	int ret = device_state_pm_set_state_by_name(dev,
			pm_platdata->pm_state_D0_name);

	if (ret)
		dev_err(dev, "Power ON Failed: %d\n", ret);

	return ret;
}

int gyro_power_off(struct device *dev)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = stat->pdata->pm_platdata;
	int ret = device_state_pm_set_state_by_name(dev,
			pm_platdata->pm_state_D3_name);

	if (ret)
		dev_err(dev, "Power OFF Failed: %d\n", ret);

	return ret;
}

static int gyro_init(struct device *dev)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = stat->pdata->pm_platdata;

	if (!pm_platdata || !pm_platdata->pm_state_D3_name
			|| !pm_platdata->pm_state_D0_name)
		return -EINVAL;

	return device_state_pm_set_class(dev, pm_platdata->pm_user_name);
}

static void gyro_exit(void)
{
	return;
}

#define OF_AXIS_MAP		"intel,axis-map"
#define OF_NEGATE		"intel,negate"
#define OF_POLL_INTERVAL	"intel,poll-interval"
#define OF_FS_RANGE		"intel,full-scale"

static struct l3gd20_gyr_platform_data *l3gd20_gyr_of_get_platdata(
		struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct l3gd20_gyr_platform_data *gyro_pdata;
	u32 out_values[3], val;
	int ret = 0;

	gyro_pdata = kzalloc(sizeof(*gyro_pdata), GFP_KERNEL);

	if (!gyro_pdata)
		return ERR_PTR(-ENOMEM);

	/* pinctrl */
	gyro_pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(gyro_pdata->pinctrl))
		goto skip_pinctrl;

	gyro_pdata->pins_default = pinctrl_lookup_state(gyro_pdata->pinctrl,
			PINCTRL_STATE_DEFAULT);
	if (IS_ERR(gyro_pdata->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	gyro_pdata->pins_sleep = pinctrl_lookup_state(gyro_pdata->pinctrl,
			PINCTRL_STATE_SLEEP);
	if (IS_ERR(gyro_pdata->pins_sleep))
		dev_err(dev, "could not get sleep pinstate\n");

	gyro_pdata->pins_inactive = pinctrl_lookup_state(gyro_pdata->pinctrl,
			"inactive");
	if (IS_ERR(gyro_pdata->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

skip_pinctrl:
	/* Axis map properties */
	if (of_property_read_u32_array(np, OF_AXIS_MAP, out_values, 3) < 0) {

		dev_err(dev, "Error parsing %s property of node %s\n",
			OF_AXIS_MAP, np->name);
		goto out;
	}
	gyro_pdata->axis_map_x = (u8)out_values[0];
	gyro_pdata->axis_map_y = (u8)out_values[1];
	gyro_pdata->axis_map_z = (u8)out_values[2];

	/* Negate properties */
	if (of_property_read_u32_array(np, OF_NEGATE, out_values, 3) < 0) {

		dev_err(dev, "Error parsing %s property of node %s\n",
			OF_NEGATE, np->name);
		goto out;
	}
	gyro_pdata->negate_x = (u8)out_values[0];
	gyro_pdata->negate_y = (u8)out_values[1];
	gyro_pdata->negate_z = (u8)out_values[2];

	/* Poll interval property */
	if (of_property_read_u32(np, OF_POLL_INTERVAL,
				&gyro_pdata->poll_interval) < 0) {

		dev_err(dev, "Error parsing %s property of node %s\n",
			OF_POLL_INTERVAL, np->name);
		goto out;
	}

	/* Full scales property */
	val = 0;
	of_property_read_u32(np, OF_FS_RANGE, &val);
	switch (val) {
	case 2000:
		gyro_pdata->fs_range = L3GD20_GYR_FS_2000DPS;
		break;
	case 500:
		gyro_pdata->fs_range = L3GD20_GYR_FS_500DPS;
		break;
	case 250:
		gyro_pdata->fs_range = L3GD20_GYR_FS_250DPS;
		break;
	default:
		dev_dbg(dev, "Error parsing %s property of node %s\n",
			OF_FS_RANGE, np->name);
		dev_dbg(dev, "Use default value %d\n", L3GD20_GYR_FS_250DPS);
		break;
	}

	/* device pm */
	gyro_pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(gyro_pdata->pm_platdata)) {
		dev_err(dev, "Error during device state pm init\n");
		ret = -EINVAL;
		goto out;
	}

	gyro_pdata->init = gyro_init;
	gyro_pdata->exit = gyro_exit;
	gyro_pdata->power_on = gyro_power_on;
	gyro_pdata->power_off = gyro_power_off;

	/*  */
	gyro_pdata->min_interval = L3GD20_GYR_MIN_POLL_PERIOD_MS; /* 2ms */
	gyro_pdata->gpio_int1 = 0;
	gyro_pdata->gpio_int2 = L3GD20_GYR_DEFAULT_INT2_GPIO; /* int for fifo */

	return gyro_pdata;

out:
	kfree(gyro_pdata);
	return ERR_PTR(ret);
}
#endif

static inline int l3gd20_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	struct l3gd20_gyr_platform_data *pdata = stat->pdata;
	int ret = 0;

	if (!pdata) {
		dev_err(dev, "Unable to retrieve l3gd20 platform data\n");
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	return ret;
}

static int l3gd20_gyr_probe(struct i2c_client *client,
					const struct i2c_device_id *devid)
{
	struct l3gd20_gyr_status *stat;
	struct l3gd20_gyr_platform_data *platdata;

	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;

	int err = -1;

	stat = kzalloc(sizeof(*stat), GFP_KERNEL);
	if (stat == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	/* Support for both I2C and SMBUS adapter interfaces. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			stat->use_smbus = 0;
			goto err0_1;
		}
	}

	l3gd20_gyr_workqueue = create_workqueue("l3gd20_gyr_workqueue");
	if (!l3gd20_gyr_workqueue) {
		err = -ENOMEM;
		dev_err(&client->dev, "create_workqueue failed\n");
		stat->use_smbus = 0;
		goto err0_1;
	}

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);
	stat->client = client;

#ifdef CONFIG_OF
	platdata = client->dev.platform_data =
		l3gd20_gyr_of_get_platdata(&client->dev);
	if (IS_ERR(platdata)) {
		err = PTR_ERR(platdata);
		goto err0_2;
	}
#endif

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err1;
	}

	if (client->dev.platform_data == NULL) {
		default_l3gd20_gyr_pdata.gpio_int1 = int1_gpio;
		default_l3gd20_gyr_pdata.gpio_int2 = int2_gpio;
		memcpy(stat->pdata, &default_l3gd20_gyr_pdata,
							sizeof(*stat->pdata));
		dev_dbg(&client->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, client->dev.platform_data,
						sizeof(*stat->pdata));
	}

	err = l3gd20_gyr_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, stat);

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL_REG1] = ALL_ZEROES | ENABLE_ALL_AXES
								| PM_NORMAL;
	stat->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG4] = ALL_ZEROES | BDU_ENABLE;
	stat->resume_state[RES_CTRL_REG5] = ALL_ZEROES;
	stat->resume_state[RES_FIFO_CTRL_REG] = ALL_ZEROES;

	if (client->irq > 0 || stat->pdata->gpio_int1 > 0)
		stat->polling_enabled = false;
	else {
		stat->polling_enabled = true;
		stat->pdata->gpio_int1 = -1;
	}

	if (stat->polling_enabled) {
		stat->pdata->gpio_int1 = -1;
		hrtimer_init(&stat->hr_timer,
				CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		stat->hr_timer.function = &poll_function_read;
		INIT_WORK(&stat->polling_task, poll_function_work);
	}

	if (stat->pdata->gpio_int1 >= 0) {
		INIT_WORK(&stat->irq1_work, l3gd20_gyr_irq1_work_func);
		stat->irq1_work_queue =
			create_singlethread_workqueue("l3gd20_gyr_irq1_wq");
		if (!stat->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err1_1;
		}
#ifndef CONFIG_OF
		err = gpio_request(stat->pdata->gpio_int1, "GYRO_INTR");
		if (err) {
			dev_err(&client->dev,
			"%s gpio request failed\n", L3GD20_GYR_DEV_NAME);
			goto err2;
		}

		stat->irq1 = gpio_to_irq(stat->pdata->gpio_int1);
		if (stat->irq1 < 0) {
			dev_err(&client->dev,
			"%s gpio to irq failed\n", L3GD20_GYR_DEV_NAME);
			goto err2;
		}
		dev_info(&client->dev,
			"%s: %s has set irq1 to irq: %d mapped on gpio:%d\n",
			L3GD20_GYR_DEV_NAME, __func__, stat->irq1,
			stat->pdata->gpio_int1);
#else
		/* FIXME: missing 2nd irq support */
		stat->irq1 = client->irq;
#endif
		err = request_irq(stat->irq1, l3gd20_gyr_isr1,
				IRQF_TRIGGER_HIGH, "l3gd20_gyr_irq1", stat);

		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err3;
		}

		disable_irq_nosync(stat->irq1);

		pr_debug("%s interrupt handler registered\n", __func__);
	}

	err = l3gd20_gyr_input_init(stat);
	if (err < 0)
		goto err4;

	/* pcl */
	l3gd20_set_pinctrl_state(&client->dev, stat->pdata->pins_default);

	if (stat->pdata->init) {
		err = stat->pdata->init(&stat->input_dev->dev);
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err5;
		}
	}

	err = l3gd20_gyr_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err6;
	}

	atomic_set(&stat->enabled, 1);

	err = l3gd20_gyr_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto err7;
	}

	err = l3gd20_gyr_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err7;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
			"%s device register failed\n", L3GD20_GYR_DEV_NAME);
		goto err7;
	}

	l3gd20_gyr_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	mutex_unlock(&stat->lock);

	dev_info(&client->dev, "%s probed successfully\n",
			L3GD20_GYR_DEV_NAME);

	return 0;

err7:
	l3gd20_gyr_device_power_off(stat);
err6:
	if (stat->pdata->exit)
		stat->pdata->exit();
err5:
	l3gd20_gyr_input_cleanup(stat);
err4:
	if (stat->pdata->gpio_int2 >= 0)
		free_irq(stat->irq2, stat);
err3:
#ifndef CONFIG_OF
	gpio_free(stat->pdata->gpio_int1);
err2:
#endif
	if (stat->pdata->gpio_int1 >= 0)
		destroy_workqueue(stat->irq1_work_queue);
err1_1:
	kfree(stat->pdata);
err1:
#ifdef CONFIG_OF
	kfree(platdata->pm_platdata);
	kfree(platdata);
err0_2:
#endif
	mutex_unlock(&stat->lock);
	destroy_workqueue(l3gd20_gyr_workqueue);
err0_1:
	kfree(stat);
err0:
	pr_err("%s: Driver Initialization failed\n", L3GD20_GYR_DEV_NAME);
	return err;
}

static int l3gd20_gyr_remove(struct i2c_client *client)
{
	struct l3gd20_gyr_status *stat = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s\n", __func__);
	l3gd20_set_pinctrl_state(&client->dev, stat->pdata->pins_inactive);
	l3gd20_gyr_disable(stat);
	cancel_work_sync(&stat->polling_task);
	if (l3gd20_gyr_workqueue) {
		flush_workqueue(l3gd20_gyr_workqueue);
		destroy_workqueue(l3gd20_gyr_workqueue);
	}

	if (stat->pdata->gpio_int1 >= 0) {
		free_irq(stat->irq1, stat);
		gpio_free(stat->pdata->gpio_int1);
		destroy_workqueue(stat->irq1_work_queue);
	}

	l3gd20_gyr_input_cleanup(stat);

	remove_sysfs_interfaces(&client->dev);

	if (stat->pdata->exit)
		stat->pdata->exit();

	kfree(stat->pdata);
	kfree(stat);
	return 0;
}

#ifdef CONFIG_PM
static int l3gd20_gyr_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_gyr_status *stat = i2c_get_clientdata(client);

	l3gd20_set_pinctrl_state(dev, stat->pdata->pins_sleep);
	stat->on_before_suspend = atomic_read(&stat->enabled);
	return l3gd20_gyr_disable(stat);
}

static int l3gd20_gyr_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_gyr_status *stat = i2c_get_clientdata(client);

	l3gd20_set_pinctrl_state(dev, stat->pdata->pins_default);
	if (stat->on_before_suspend)
		return l3gd20_gyr_enable(stat);

	return 0;
}

#else

#define l3gd20_gyr_suspend		NULL
#define l3gd20_gyr_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id l3gd20_gyr_id[] = {
	{ L3GD20_GYR_DEV_NAME , 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, l3gd20_gyr_id);

static const struct dev_pm_ops l3gd20_gyr_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(l3gd20_gyr_suspend, l3gd20_gyr_resume)
};

static struct i2c_driver l3gd20_gyr_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = L3GD20_GYR_DEV_NAME,
			.pm = &l3gd20_gyr_pm,
	},
	.probe = l3gd20_gyr_probe,
	.remove = l3gd20_gyr_remove,
	.id_table = l3gd20_gyr_id,

};

static int __init l3gd20_gyr_init(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&l3gd20_pm_class);
	if (ret) {
		pr_err("%s: %s: ERROR adding %s pm class\n",
				L3GD20_GYR_DEV_NAME, __func__,
				l3gd20_pm_class.name);
		return ret;
	}
#endif
	pr_info("%s: gyroscope sysfs driver init\n", L3GD20_GYR_DEV_NAME);
	return i2c_add_driver(&l3gd20_gyr_driver);
}

static void __exit l3gd20_gyr_exit(void)
{
	pr_info("%s exit\n", L3GD20_GYR_DEV_NAME);
	i2c_del_driver(&l3gd20_gyr_driver);
	return;
}

module_init(l3gd20_gyr_init);
module_exit(l3gd20_gyr_exit);

MODULE_DESCRIPTION("l3gd20 gyroscope driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");
