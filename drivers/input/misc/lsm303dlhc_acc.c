/*
 *   Adaptations were made to original lsm303dlhc_acc.c with
 *   Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 */
/******************** (C) COPYRIGHT 2011 STMicroelectronics ******************
 *
 * File Name          : lsm303dlhc_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Carmine Iascone (carmine.iascone@st.com)
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.12.1
 * Date               : 2012/May/29
 * Description        : LSM303DLHC accelerometer sensor API
 *
 ******************************************************************************
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
 ******************************************************************************
 Revision 1.0.6 15/11/2010
  first revision
  supports sysfs;
  no more support for ioctl;
 Revision 1.0.7 26/11/2010
  checks for availability of interrupts pins
  correction on FUZZ and FLAT values;
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7
 Revision 1.0.9: 2011/May/23
  update_odr func correction;
 Revision 1.0.10: 2011/Aug/16
  introduces default_platform_data, i2c_read and i2c_write function rewritten,
  manages smbus beside i2c
 Revision 1.0.11: 2012/Jan/09
  moved under input/misc
 Revision 1.0.12: 2012/Feb/29
  moved use_smbus inside status struct; modified:-update_fs_range;-set_range
  input format; allows gpio_intX to be passed as parameter at insmod time;
 Revision 1.0.12.1: 2012/May/29
  modified some defines;
 *****************************************************************************/

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/hrtimer.h>
#ifndef CONFIG_PINCTRL
#include <mach/gpio.h>
#else
#include <linux/pinctrl/consumer.h>
#endif
#include <linux/input/lsm303dlhc.h>

#define	G_MAX		16000


#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/

/* Accelerometer Sensor Operating Mode */
#define LSM303DLHC_ACC_ENABLE	(0x01)
#define LSM303DLHC_ACC_DISABLE	(0x00)

#define	HIGH_RESOLUTION		(0x08)

#define	AXISDATA_REG		(0x28)
#define WHOAMI_LSM303DLHC_ACC	(0x33)	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		(0x0F)	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		(0x1F)	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		(0x20)	/*	control reg 1		*/
#define	CTRL_REG2		(0x21)	/*	control reg 2		*/
#define	CTRL_REG3		(0x22)	/*	control reg 3		*/
#define	CTRL_REG4		(0x23)	/*	control reg 4		*/
#define	CTRL_REG5		(0x24)	/*	control reg 5		*/
#define	CTRL_REG6		(0x25)	/*	control reg 6		*/
#define REFERENCE_REG   (0x26)  /*  reference reg       */

#define	FIFO_CTRL_REG		(0x2E)	/*	FiFo control reg	*/

#define	INT_CFG1		(0x30)	/*	interrupt 1 config	*/
#define	INT_SRC1		(0x31)	/*	interrupt 1 source	*/
#define	INT_THS1		(0x32)	/*	interrupt 1 threshold	*/
#define	INT_DUR1		(0x33)	/*	interrupt 1 duration	*/

#define	TT_CFG			(0x38)	/*	tap config		*/
#define	TT_SRC			(0x39)	/*	tap source		*/
#define	TT_THS			(0x3A)	/*	tap threshold		*/
#define	TT_LIM			(0x3B)	/*	tap time limit		*/
#define	TT_TLAT			(0x3C)	/*	tap time latency	*/
#define	TT_TW			(0x3D)	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/

#define ENABLE_HIGH_RESOLUTION	1
#define ALL_ZEROES		(0x00)

#define LSM303DLHC_ACC_PM_OFF		(0x00)
#define LSM303DLHC_ACC_ENABLE_ALL_AXES	(0x07)

#define PMODE_MASK		(0x08)
#define ODR_MASK		(0XF0)

#define LSM303DLHC_ACC_ODR1	(0x10)  /* 1Hz output data rate */
#define LSM303DLHC_ACC_ODR10	(0x20)  /* 10Hz output data rate */
#define LSM303DLHC_ACC_ODR25	(0x30)  /* 25Hz output data rate */
#define LSM303DLHC_ACC_ODR50	(0x40)  /* 50Hz output data rate */
#define LSM303DLHC_ACC_ODR100	(0x50)  /* 100Hz output data rate */
#define LSM303DLHC_ACC_ODR200	(0x60)  /* 200Hz output data rate */
#define LSM303DLHC_ACC_ODR400	(0x70)  /* 400Hz output data rate */
#define LSM303DLHC_ACC_ODR1250	(0x90)  /* 1250Hz output data rate */

#define	IA			(0x40)
#define	ZH			(0x20)
#define	ZL			(0x10)
#define	YH			(0x08)
#define	YL			(0x04)
#define	XH			(0x02)
#define	XL			(0x01)
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	(0x40)
#define	CTRL_REG4_BDU_ENABLE	(0x80)
#define	CTRL_REG4_BDU_MASK	(0x80)
#define	CTRL_REG6_I2_TAPEN	(0x80)
#define	CTRL_REG6_HLACTIVE	(0x02)
/* */
#define NO_MASK			(0xFF)
#define INT1_DURATION_MASK	(0x7F)
#define	INT1_THRESHOLD_MASK	(0x7F)
#define TAP_CFG_MASK		(0x3F)
#define	TAP_THS_MASK		(0x7F)
#define	TAP_TLIM_MASK		(0x7F)
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK

/* TAP_SOURCE_REG BIT */
#define	DTAP			(0x20)
#define	STAP			(0x10)
#define	SIGNTAP			(0x08)
#define	ZTAP			(0x04)
#define	YTAP			(0x02)
#define	XTAZ			(0x01)

#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	(0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define	RESUME_ENTRIES		17
/* end RESUME STATE INDICES */

#define MS_TO_NS(value) ktime_set(value/1000, (value % 1000)*1000000)

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
	unsigned int int1_ths;
	int polling_threshold;
} lsm303dlhc_acc_odr_table[] = {
		{    1, LSM303DLHC_ACC_ODR1250, 2, 2 },
		{    3, LSM303DLHC_ACC_ODR400, 2, 16 },
		{    5, LSM303DLHC_ACC_ODR200, 2, 16 },
		{   10, LSM303DLHC_ACC_ODR100, 2, 16  },
		{   20, LSM303DLHC_ACC_ODR50, 3, 32 },
		{   40, LSM303DLHC_ACC_ODR25, 3, 32 },
		{  100, LSM303DLHC_ACC_ODR10, 3, 32 },
		{ 1000, LSM303DLHC_ACC_ODR1, 3, 32 },
};

static int int1_gpio = LSM303DLHC_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = LSM303DLHC_ACC_DEFAULT_INT2_GPIO;
module_param(int1_gpio, int, S_IRUGO);
module_param(int2_gpio, int, S_IRUGO);

struct lsm303dlhc_acc_status {
	struct i2c_client *client;
	struct lsm303dlhc_acc_platform_data *pdata;

	struct mutex lock;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	int use_smbus;
	int poll_count;
	int prev_xyz[3];
	bool polling_enabled;
	int polling_threshold;
	u8 sensitivity;
	u8 resume_state[RESUME_ENTRIES];
	int irq1;
	int timer_running;
	struct work_struct input_work;
	struct workqueue_struct *input_work_queue;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct hrtimer timer;
	struct workqueue_struct *irq2_work_queue;
	u8 reg_addr;
};

static struct lsm303dlhc_acc_platform_data
	default_lsm303dlhc_acc_pdata = {
	.fs_range = LSM303DLHC_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.intr_poll_count = 20,
	.min_interval = LSM303DLHC_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LSM303DLHC_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM303DLHC_ACC_DEFAULT_INT2_GPIO,
};

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
/* Accelerometer PM states & class */
static struct device_state_pm_state lsm303dlhc_acc_pm_states[] = {
	{ .name = "disable", }, /* D3 */
	{ .name = "enable", }, /* D0 */
};

static int lsm303dlhc_acc_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	/* FIXME */
	return 0;
}

static struct device_state_pm_state *lsm303dlhc_acc_get_initial_state(
		struct device *dev)
{
	return &lsm303dlhc_acc_pm_states[0];
}

static struct device_state_pm_ops lsm303dlhc_acc_pm_ops = {
	.set_state = lsm303dlhc_acc_set_pm_state,
	.get_initial_state = lsm303dlhc_acc_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(lsm303dlhc_acc);
#endif

#ifdef CONFIG_OF
int acc_power_on(struct device *dev)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = stat->pdata->pm_platdata;
	int err = device_state_pm_set_state_by_name(dev,
			pm_platdata->pm_state_D0_name);
	if (err)
		dev_err(dev, "Power ON Failed: %d\n", err);

	return err;
}

int acc_power_off(struct device *dev)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = stat->pdata->pm_platdata;
	int err = device_state_pm_set_state_by_name(dev,
			pm_platdata->pm_state_D3_name);

	if (err)
		dev_err(dev, "Power OFF Failed:\n");

	return err;
}

static int acc_init(struct device *dev)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = stat->pdata->pm_platdata;

	if (!pm_platdata || !pm_platdata->pm_state_D0_name ||
			!pm_platdata->pm_state_D3_name)
		return -EINVAL;

	return device_state_pm_set_class(dev, pm_platdata->pm_user_name);
}

static void acc_exit(void)
{
	return;
}

#define OF_AXIS_MAP		"intel,axis-map"
#define OF_NEGATE		"intel,negate"
#define OF_POLL_INTERVAL	"intel,poll-interval"

static struct lsm303dlhc_acc_platform_data *lsm303dlhc_acc_of_get_platdata(
		struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct lsm303dlhc_acc_platform_data *acc_pdata;
	u32 out_values[3];
	int ret = 0;

	acc_pdata = kzalloc(sizeof(*acc_pdata), GFP_KERNEL);

	if (!acc_pdata)
		return ERR_PTR(-ENOMEM);

	/* pinctrl */
	acc_pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(acc_pdata->pinctrl))
		goto skip_pinctrl;

	acc_pdata->pins_default = pinctrl_lookup_state(acc_pdata->pinctrl,
			PINCTRL_STATE_DEFAULT);
	if (IS_ERR(acc_pdata->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	acc_pdata->pins_sleep = pinctrl_lookup_state(acc_pdata->pinctrl,
			PINCTRL_STATE_SLEEP);
	if (IS_ERR(acc_pdata->pins_sleep))
		dev_err(dev, "could not get sleep pinstate\n");

	acc_pdata->pins_inactive = pinctrl_lookup_state(acc_pdata->pinctrl,
			"inactive");
	if (IS_ERR(acc_pdata->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

skip_pinctrl:
	/* Axis map properties */
	if (of_property_read_u32_array(np, OF_AXIS_MAP, out_values, 3) < 0) {

		dev_err(dev, "Error parsing %s property of node %s\n",
			OF_AXIS_MAP, np->name);
		goto out;
	}
	acc_pdata->axis_map_x = (u8)out_values[0];
	acc_pdata->axis_map_y = (u8)out_values[1];
	acc_pdata->axis_map_z = (u8)out_values[2];

	/* Negate properties */
	if (of_property_read_u32_array(np, OF_NEGATE, out_values, 3) < 0) {

		dev_err(dev, "Error parsing %s property of node %s\n",
			OF_NEGATE, np->name);
		goto out;
	}
	acc_pdata->negate_x = (u8)out_values[0];
	acc_pdata->negate_y = (u8)out_values[1];
	acc_pdata->negate_z = (u8)out_values[2];

	/* Poll interval property */
	if (of_property_read_u32(np, OF_POLL_INTERVAL,
				&acc_pdata->poll_interval) < 0) {

		dev_err(dev, "Error parsing %s property of node %s\n",
			OF_POLL_INTERVAL, np->name);
		goto out;
	}

	/* device pm */
	acc_pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(acc_pdata->pm_platdata)) {
		dev_err(dev, "Error during device state pm init\n");
		ret = -EINVAL;
		goto out;
	}

	acc_pdata->init = acc_init;
	acc_pdata->exit = acc_exit;
	acc_pdata->power_on = acc_power_on;
	acc_pdata->power_off = acc_power_off;

	/* FIXME: set default values */
	acc_pdata->fs_range = LSM303DLHC_ACC_G_2G;
	acc_pdata->min_interval = LSM303DLHC_ACC_MIN_POLL_PERIOD_MS; /* 2ms */
	acc_pdata->gpio_int1 =
		LSM303DLHC_ACC_DEFAULT_INT1_GPIO;
	acc_pdata->gpio_int2 =
		LSM303DLHC_ACC_DEFAULT_INT2_GPIO; /* int for fifo */
	acc_pdata->intr_poll_count = 5;

	return acc_pdata;

out:
	kfree(acc_pdata);
	return ERR_PTR(ret);
}
#endif

static inline int lsm303dlhc_acc_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	struct lsm303dlhc_acc_platform_data *pdata = stat->pdata;
	int ret = 0;

	if (!pdata) {
		dev_err(dev,
			"Unable to retrieve lsm303dlhc_acc platform data\n");
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	return ret;
}

static int lsm303dlhc_acc_get_acceleration_data(
				struct lsm303dlhc_acc_status *stat, int *xyz);
static void lsm303dlhc_acc_report_values(
				struct lsm303dlhc_acc_status *stat, int *xyz);

static int lsm303dlhc_acc_i2c_read(struct lsm303dlhc_acc_status *stat, u8 *buf,
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

static int lsm303dlhc_acc_i2c_write(struct lsm303dlhc_acc_status *stat,
				    u8 *buf, int len)
{
	int ret, retries = 5;
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

static int lsm303dlhc_acc_hw_init(struct lsm303dlhc_acc_status *stat)
{
	int err = -1;
	u8 buf[7];

	dev_dbg(&stat->client->dev, "%s: hw init start\n", __func__);

	buf[0] = WHO_AM_I;
	err = lsm303dlhc_acc_i2c_read(stat, buf, 1);
	if (err < 0) {
		dev_warn(&stat->client->dev,
			"Error reading WHO_AM_I: is device available?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf[0] != WHOAMI_LSM303DLHC_ACC) {
		dev_err(&stat->client->dev,
			"device unknown. Expected: 0x%02x, Replies: 0x%02x\n",
			WHOAMI_LSM303DLHC_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = CTRL_REG1;
	buf[1] = stat->resume_state[RES_CTRL_REG1];
	err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TEMP_CFG_REG;
	buf[1] = stat->resume_state[RES_TEMP_CFG_REG];
	err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = FIFO_CTRL_REG;
	buf[1] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TT_THS;
	buf[1] = stat->resume_state[RES_TT_THS];
	buf[2] = stat->resume_state[RES_TT_LIM];
	buf[3] = stat->resume_state[RES_TT_TLAT];
	buf[4] = stat->resume_state[RES_TT_TW];
	err = lsm303dlhc_acc_i2c_write(stat, buf, 4);
	if (err < 0)
		goto err_resume_state;
	buf[0] = TT_CFG;
	buf[1] = stat->resume_state[RES_TT_CFG];
	err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = INT_THS1;
	buf[1] = stat->resume_state[RES_INT_THS1];
	buf[2] = stat->resume_state[RES_INT_DUR1];
	err = lsm303dlhc_acc_i2c_write(stat, buf, 2);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = stat->resume_state[RES_INT_CFG1];
	err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = CTRL_REG2;
	buf[1] = stat->resume_state[RES_CTRL_REG2];
	buf[2] = stat->resume_state[RES_CTRL_REG3];
	buf[3] = stat->resume_state[RES_CTRL_REG4];
	buf[4] = stat->resume_state[RES_CTRL_REG5];
	buf[5] = stat->resume_state[RES_CTRL_REG6];
	err = lsm303dlhc_acc_i2c_write(stat, buf, 5);
	if (err < 0)
		goto err_resume_state;

	stat->prev_xyz[0] = 0;
	stat->prev_xyz[1] = 0;
	stat->prev_xyz[2] = 0;
	stat->poll_count = 0;
	stat->hw_initialized = 1;
	dev_dbg(&stat->client->dev, "%s: hw init done %x\n", __func__,
			stat->resume_state[RES_CTRL_REG1]);
	return 0;

err_firstread:
	stat->hw_working = 0;
err_unknown_device:
err_resume_state:
	stat->hw_initialized = 0;
	dev_err(&stat->client->dev, "hw init error 0x%02x,0x%02x: %d\n",
			 buf[0], buf[1], err);
	return err;
}

static int lsm303dlhc_acc_manage_int1settings(
				struct lsm303dlhc_acc_status *stat,
				u8 enable)
{
	int err;
	u8 buf[8];

	if (enable) {
		stat->resume_state[RES_CTRL_REG1] = 0x77;
		stat->resume_state[RES_CTRL_REG2] = 0x31;
		stat->resume_state[RES_CTRL_REG3] = 0x40;
		stat->resume_state[RES_CTRL_REG4] = 0x08;
		stat->resume_state[RES_CTRL_REG5] = 0x08;
		stat->resume_state[RES_CTRL_REG6] = 0x00;

		buf[0] = (CTRL_REG1);
		buf[1] = stat->resume_state[RES_CTRL_REG1];
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

		buf[0] = (CTRL_REG2);
		buf[1] = stat->resume_state[RES_CTRL_REG2];
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

		buf[0] = (CTRL_REG3);
		buf[1] = stat->resume_state[RES_CTRL_REG3];
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

		buf[0] = (CTRL_REG4);
		buf[1] = stat->resume_state[RES_CTRL_REG4];
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

		buf[0] = (CTRL_REG5);
		buf[1] = stat->resume_state[RES_CTRL_REG5];
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

		buf[0] = (CTRL_REG6);
		buf[1] = stat->resume_state[RES_CTRL_REG6];
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

		buf[0] = REFERENCE_REG;
		buf[1] = 0;
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

		buf[0] = FIFO_CTRL_REG;
		buf[1] = 0;
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

		buf[0] = (0x32);
		buf[1] = 0x03;
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

		buf[0] = (0x33);
		buf[1] = 0x0;
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

		buf[0] = (0x30);
		buf[1] = 0x2A;
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;
	} else {
		stat->resume_state[RES_CTRL_REG1] = 0;
		buf[0] = (CTRL_REG1);
		buf[1] = stat->resume_state[RES_CTRL_REG1];
		err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;
	}
	return 0;
}

static int lsm303dlhc_acc_update_fs_range(struct lsm303dlhc_acc_status *stat,
							u8 new_fs_range)
{
	int err = -1;
	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LSM303DLHC_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_fs_range) {
	case LSM303DLHC_ACC_G_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case LSM303DLHC_ACC_G_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case LSM303DLHC_ACC_G_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	case LSM303DLHC_ACC_G_16G:

		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&stat->client->dev, "invalid fs range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}

	/* Updates configuration register 4,
	 * which contains fs range setting */
	buf[0] = CTRL_REG4;
	err = lsm303dlhc_acc_i2c_read(stat, buf, 1);
	if (err < 0)
		goto error;
	init_val = buf[0];
	stat->resume_state[RES_CTRL_REG4] = init_val;
	new_val = new_fs_range | HIGH_RESOLUTION;
	updated_val = ((mask & new_val) | ((~mask) & init_val));
	buf[1] = updated_val;
	buf[0] = CTRL_REG4;
	err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;
	stat->resume_state[RES_CTRL_REG4] = updated_val;
	stat->sensitivity = sensitivity;

	dev_dbg(&stat->client->dev, "%s, %d\n", __func__, stat->sensitivity);
	return err;
error:
	dev_err(&stat->client->dev,
			"update fs range failed 0x%02x, 0x%02x: %d\n",
			buf[0], buf[1], err);

	return err;
}

static int lsm303dlhc_acc_update_odr(struct lsm303dlhc_acc_status *stat,
							int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lsm303dlhc_acc_odr_table) - 1; i >= 0; i--) {
		if ((lsm303dlhc_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
								|| (i == 0))
			break;
	}
	config[1] = lsm303dlhc_acc_odr_table[i].mask;
	config[1] |= LSM303DLHC_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		config[0] = CTRL_REG1;
		err = lsm303dlhc_acc_i2c_write(stat, config, 1);
		if (err < 0)
			goto error;
		stat->resume_state[RES_CTRL_REG1] = config[1];
		config[0] = INT_THS1;
		config[1] = lsm303dlhc_acc_odr_table[i].int1_ths;
		err = lsm303dlhc_acc_i2c_write(stat, config, 1);
		if (err < 0)
			goto error;
		stat->polling_threshold =
			lsm303dlhc_acc_odr_table[i].polling_threshold;
	}

	return err;

error:
	dev_err(&stat->client->dev, "update odr failed 0x%02x,0x%02x: %d\n",
			config[0], config[1], err);

	return err;
}

static int lsm303dlhc_acc_get_acceleration_data(
		struct lsm303dlhc_acc_status *stat, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6] = { 0 };
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	acc_data[0] = (AXISDATA_REG);
	err = lsm303dlhc_acc_i2c_read(stat, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] = hw_d[0] * stat->sensitivity;
	hw_d[1] = hw_d[1] * stat->sensitivity;
	hw_d[2] = hw_d[2] * stat->sensitivity;

	xyz[0] = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
		   : (hw_d[stat->pdata->axis_map_x]));
	xyz[1] = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
		   : (hw_d[stat->pdata->axis_map_y]));
	xyz[2] = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
		   : (hw_d[stat->pdata->axis_map_z]));

	return err;
}

static void lsm303dlhc_acc_report_values(struct lsm303dlhc_acc_status *stat,
					int *xyz)
{
	dev_dbg(&stat->client->dev, "%s read x=%d, y=%d, z=%d\n",
			LSM303DLHC_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	input_report_abs(stat->input_dev, ABS_X, xyz[0]);
	input_report_abs(stat->input_dev, ABS_Y, xyz[1]);
	input_report_abs(stat->input_dev, ABS_Z, xyz[2]);
	input_sync(stat->input_dev);
}

static void lsm303dlhc_acc_irq1_work_func(struct work_struct *work)
{
	struct lsm303dlhc_acc_status *stat = container_of(work,
			struct lsm303dlhc_acc_status, irq1_work);
	int xyz[3] = { 0 };
	int err, sum;
	u8 buf[2] = {0};

	dev_dbg(&stat->client->dev,
			"lsm303dlhc_acc_irq1_work_func --> Start\n");
	buf[0] = (0x31);
	err = lsm303dlhc_acc_i2c_read(stat, buf, 1);
	if (err < 0)
		return;

	err = lsm303dlhc_acc_get_acceleration_data(stat, xyz);
	if (err < 0)
		dev_err(&stat->client->dev, "get_acceleration_data failed\n");
	else
		lsm303dlhc_acc_report_values(stat, xyz);

	buf[0] = REFERENCE_REG;
	err = lsm303dlhc_acc_i2c_read(stat, buf, 1);
	if (err < 0)
		return;

	if (stat->poll_count < stat->pdata->intr_poll_count) {
		sum = (xyz[0] - stat->prev_xyz[0]) +
				(xyz[1] - stat->prev_xyz[1]) +
				(xyz[2] - stat->prev_xyz[2]);
		stat->prev_xyz[0] = xyz[0];
		stat->prev_xyz[1] = xyz[1];
		stat->prev_xyz[2] = xyz[2];
		if (sum < stat->polling_threshold) {
			hrtimer_start(&stat->timer,
					MS_TO_NS(stat->pdata->poll_interval),
					HRTIMER_MODE_REL);
			stat->poll_count++;
		} else {
			hrtimer_start(&stat->timer,
					MS_TO_NS(stat->pdata->poll_interval),
					HRTIMER_MODE_REL);
			stat->poll_count = 0;
		}
	} else {
		enable_irq(stat->irq1);
		stat->poll_count = 0;
	}
	dev_dbg(&stat->client->dev,
		"lsm303dlhc_acc_irq1_work_func --> Exit\n");
}

static enum hrtimer_restart
		lsm303dlhc_acc_hrtimer_callback(struct hrtimer *timer)
{
	struct lsm303dlhc_acc_status *stat =
		container_of(timer, struct lsm303dlhc_acc_status, timer);
	dev_dbg(&stat->client->dev, "lsm303dlhc_acc_hrtimer_callback\n");
	if (!stat->polling_enabled)
		queue_work(stat->irq1_work_queue, &stat->irq1_work);
	else
		queue_work(stat->input_work_queue, &stat->input_work);
	return HRTIMER_NORESTART;
}

static irqreturn_t lsm303dlhc_acc_isr1(int irq, void *dev)
{
	struct lsm303dlhc_acc_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq1_work_queue, &stat->irq1_work);
	return IRQ_HANDLED;
}

static void lsm303dlhc_acc_input_work_func(struct work_struct *work)
{
	struct lsm303dlhc_acc_status *stat;

	int xyz[3] = { 0 };
	int err;

	stat = container_of(work,
			struct lsm303dlhc_acc_status, input_work);

	if (!atomic_read(&stat->enabled))
		return;

	mutex_lock(&stat->lock);
	err = lsm303dlhc_acc_get_acceleration_data(stat, xyz);
	if (err < 0) {
		dev_err(&stat->client->dev, "get_acceleration_data failed\n");
		mutex_unlock(&stat->lock);
		return;
	}

	lsm303dlhc_acc_report_values(stat, xyz);

	hrtimer_start(&stat->timer, MS_TO_NS(stat->pdata->poll_interval),
						HRTIMER_MODE_REL);
	mutex_unlock(&stat->lock);
}

static void lsm303dlhc_acc_device_power_off(struct lsm303dlhc_acc_status *stat)
{
	int err;

	u8 buf[2] = { CTRL_REG1, LSM303DLHC_ACC_PM_OFF };

	err = lsm303dlhc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev,
			"soft power off failed: %d\n", err);

	if (stat->timer_running == 1)
		stat->timer_running = 0;
	else if (!stat->polling_enabled)
		disable_irq_nosync(stat->irq1);

	if (stat->pdata->power_off)
		stat->pdata->power_off(&stat->input_dev->dev);

	stat->hw_initialized = 0;
}

static int lsm303dlhc_acc_device_power_on(struct lsm303dlhc_acc_status *stat)
{
	int err = -1;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on(&stat->input_dev->dev);
		if (err < 0) {
			dev_err(&stat->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
	}
	msleep(20);
	if (!stat->polling_enabled)
		enable_irq(stat->irq1);

	if (!stat->hw_initialized) {
		err = lsm303dlhc_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lsm303dlhc_acc_device_power_off(stat);
			return err;
		}
	}
	return 0;
}

static int lsm303dlhc_acc_enable(struct lsm303dlhc_acc_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		stat->poll_count = 0;
		err = lsm303dlhc_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
		if (!stat->polling_enabled) {
			err = lsm303dlhc_acc_manage_int1settings(stat, 1);
			if (err < 0)
				return err;

			dev_dbg(&stat->client->dev, "Interrupt mode enabled\n");
		} else {
			dev_dbg(&stat->client->dev, "Polling mode enabled\n");
			hrtimer_start(&stat->timer,
					MS_TO_NS(stat->pdata->poll_interval),
					HRTIMER_MODE_REL);
		}
	}
	return 0;
}

static int lsm303dlhc_acc_disable(struct lsm303dlhc_acc_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		stat->timer_running = hrtimer_active(&stat->timer);
		hrtimer_cancel(&stat->timer);
		lsm303dlhc_acc_device_power_off(stat);
	}

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int val;
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_ms = max_t(unsigned int, interval_ms,
			stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	dev_dbg(&stat->client->dev, "interval_ms %ld\n", interval_ms);
	lsm303dlhc_acc_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm303dlhc_acc_enable(stat);
	else
		lsm303dlhc_acc_disable(stat);

	dev_dbg(dev, "sensor %s\n", val ? "enable" : "disable");

	return size;
}

static ssize_t attr_set_register(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
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

	if (regid < TEMP_CFG_REG || regid > TT_TW) {
		dev_err(&stat->client->dev,
			"Register address out of range!\n");
		return -EINVAL;
	}

	switch (regid) {
	case CTRL_REG1:
	case CTRL_REG2:
	case CTRL_REG3:
	case CTRL_REG4:
	case CTRL_REG5:
	case CTRL_REG6:
		stat->resume_state[RES_CTRL_REG1+(regid-CTRL_REG1)] = (u8) val;
		break;

	case INT_CFG1:
		stat->resume_state[RES_INT_CFG1] = (u8) val;
		break;

	case INT_THS1:
	case INT_DUR1:
		stat->resume_state[RES_INT_THS1+(regid-INT_THS1)] = (u8) val;
	break;

	case TT_CFG:
		stat->resume_state[RES_TT_CFG] = (u8) val;
		break;

	case TT_THS:
	case TT_LIM:
	case TT_TLAT:
	case TT_TW:
		stat->resume_state[RES_TT_THS+(regid-TT_THS)] = (u8) val;
		break;

	case TEMP_CFG_REG:
		stat->resume_state[RES_TEMP_CFG_REG] = (u8) val;
		break;

	case REFERENCE_REG:
		stat->resume_state[RES_REFERENCE_REG] = (u8) val;
		break;

	case FIFO_CTRL_REG:
		stat->resume_state[RES_FIFO_CTRL_REG] = (u8) val;
		break;

	default:
		dev_err(&stat->client->dev,
		"Write to register address 0x%02x not allowed!\n", regid);
		return -EINVAL;
	}
	i2c_buf[0] = (u8) regid;
	i2c_buf[1] = (u8) val;
	lsm303dlhc_acc_i2c_write(stat, i2c_buf, 1);

	return size;
}

static ssize_t attr_show_registers(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	int i, n, err, ofst = 0;
	u8 i2c_buf[28];

	for (i = 0; i < 28; i++) {
		i2c_buf[i] = (0x20+i);
		err = lsm303dlhc_acc_i2c_read(stat, &i2c_buf[i], 1);
		if (err < 0) {
			dev_err(dev, "Error while reading registers!\n");
			return -EINVAL;
		}
	}

	for (i = 0; i < 28; i++) {
		if ((i % 8) == 0) {
			n = sprintf(buf+ofst, "\n0x%02x:", (i+0x20));
			if (n < 0)
				return 0;
				ofst += n;
		}
		n = sprintf(buf+ofst, "  0x%02x", i2c_buf[i]);
		if (n < 0)
			return 0;
			ofst += n;
	}

	n = sprintf(buf+ofst, "\n");
	if (n < 0)
		return 0;
	ofst += n;

	return ofst;
}

static ssize_t attr_set_threshold(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long polling_threshold;

	if (kstrtoul(buf, 10, &polling_threshold))
		return -EINVAL;

	stat->polling_threshold = polling_threshold;
	dev_dbg(dev, "stat->polling_threshold %d\n",
			stat->polling_threshold);
	return size;
}


static ssize_t attr_get_threshold(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long polling_threshold;

	polling_threshold = stat->polling_threshold;
	dev_dbg(dev, "stat->polling_threshold %d\n",
				stat->polling_threshold);
	return sprintf(buf, "%ld\n", polling_threshold);
}

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
			attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(enable_device, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
			attr_get_enable, attr_set_enable),
	__ATTR(polling_threshold, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
			attr_get_threshold, attr_set_threshold),
	__ATTR(dev_register, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
			attr_show_registers, attr_set_register),
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

int lsm303dlhc_acc_input_open(struct input_dev *input)
{
	struct lsm303dlhc_acc_status *stat = input_get_drvdata(input);

	dev_dbg(&stat->client->dev, "%s\n", __func__);
	return 0;
}

void lsm303dlhc_acc_input_close(struct input_dev *dev)
{
	struct lsm303dlhc_acc_status *stat = input_get_drvdata(dev);

	dev_dbg(&stat->client->dev, "%s\n", __func__);
	return;
}

static int lsm303dlhc_acc_validate_pdata(struct lsm303dlhc_acc_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int)LSM303DLHC_ACC_MIN_POLL_PERIOD_MS,
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
	if (stat->pdata->negate_x > 1 || stat->pdata->negate_y > 1
	    || stat->pdata->negate_z > 1) {
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

static int lsm303dlhc_acc_input_init(struct lsm303dlhc_acc_status *stat)
{
	int err;

	hrtimer_init(&stat->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->timer.function = lsm303dlhc_acc_hrtimer_callback;
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev, "input device allocation failed\n");
		goto err0;
	}

	stat->input_dev->open = lsm303dlhc_acc_input_open;
	stat->input_dev->close = lsm303dlhc_acc_input_close;
	stat->input_dev->name = LSM303DLHC_ACC_DEV_NAME;
	/* stat->input_dev->name = "accelerometer"; */
	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, stat->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, stat->input_dev->absbit);

	input_set_abs_params(stat->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0,
		0);

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(&stat->client->dev,
				"unable to register input device %s\n",
				stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);
err0:
	return err;
}

static void lsm303dlhc_acc_input_cleanup(struct lsm303dlhc_acc_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

static int lsm303dlhc_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct lsm303dlhc_acc_status *stat;
	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA |
			I2C_FUNC_SMBUS_I2C_BLOCK;
	int err = -1;

	stat = kzalloc(sizeof(struct lsm303dlhc_acc_status), GFP_KERNEL);
	if (stat == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for module data: %d\n", err);
		goto exit_check_functionality_failed;
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
			goto err_freedata;
		}
	}

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	stat->client = client;
	i2c_set_clientdata(client, stat);

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

#ifdef CONFIG_OF
	client->dev.platform_data =
		lsm303dlhc_acc_of_get_platdata(&client->dev);
	if (IS_ERR(client->dev.platform_data)) {
		err = PTR_ERR(client->dev.platform_data);
		goto err_mutexunlock;
	}
#endif

	if (client->dev.platform_data == NULL) {
		default_lsm303dlhc_acc_pdata.gpio_int1 = int1_gpio;
		default_lsm303dlhc_acc_pdata.gpio_int2 = int2_gpio;
		memcpy(stat->pdata, &default_lsm303dlhc_acc_pdata,
							sizeof(*stat->pdata));
		dev_err(&client->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, client->dev.platform_data,
							sizeof(*stat->pdata));
	}

	err = lsm303dlhc_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	err = lsm303dlhc_acc_input_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto exit_kfree_pdata;
	}

	/* pcl */
	lsm303dlhc_acc_set_pinctrl_state(&client->dev,
			stat->pdata->pins_default);

	if (stat->pdata->init) {
		err = stat->pdata->init(&stat->input_dev->dev);
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_input_cleanup;
		}
	}

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL_REG1] = (ALL_ZEROES |
					     LSM303DLHC_ACC_ENABLE_ALL_AXES);
	stat->resume_state[RES_CTRL_REG4] = (ALL_ZEROES | CTRL_REG4_BDU_ENABLE);

	stat->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG4] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG5] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG6] = ALL_ZEROES;

	stat->resume_state[RES_TEMP_CFG_REG] = ALL_ZEROES;
	stat->resume_state[RES_FIFO_CTRL_REG] = ALL_ZEROES;
	stat->resume_state[RES_INT_CFG1] = ALL_ZEROES;
	stat->resume_state[RES_INT_THS1] = ALL_ZEROES;
	stat->resume_state[RES_INT_DUR1] = ALL_ZEROES;

	stat->resume_state[RES_TT_CFG] = ALL_ZEROES;
	stat->resume_state[RES_TT_THS] = ALL_ZEROES;
	stat->resume_state[RES_TT_LIM] = ALL_ZEROES;
	stat->resume_state[RES_TT_TLAT] = ALL_ZEROES;
	stat->resume_state[RES_TT_TW] = ALL_ZEROES;

	if (client->irq > 0 || stat->pdata->gpio_int1 > 0)
		stat->polling_enabled = false;
	else
		stat->polling_enabled = true;

	if (!stat->polling_enabled) {
		INIT_WORK(&stat->irq1_work, lsm303dlhc_acc_irq1_work_func);
		stat->irq1_work_queue =
		    create_singlethread_workqueue("lsm303dlhc_acc_wq1");
		if (!stat->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
				"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
#ifndef CONFIG_OF
		err = gpio_request(stat->pdata->gpio_int1, "ACC_INTR");
		if (err) {
			dev_err(&client->dev,
				"%s gpio request failed\n",
				LSM303DLHC_ACC_DEV_NAME);
		}
		stat->irq1 = gpio_to_irq(stat->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d, mapped on gpio:%d\n",
			LSM303DLHC_ACC_DEV_NAME, __func__, stat->irq1,
			stat->pdata->gpio_int1);
#else
		stat->irq1 = client->irq;
#endif
		err = request_irq(stat->irq1, lsm303dlhc_acc_isr1,
			IRQF_TRIGGER_HIGH, "lsm303dlhc_acc_irq1", stat);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destroy_work_queue;
		}
		disable_irq_nosync(stat->irq1);
	} else {
		INIT_WORK(&stat->input_work, lsm303dlhc_acc_input_work_func);
		stat->input_work_queue =
			create_singlethread_workqueue("lsm303_acc_input_wq");
		if (!stat->input_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
				"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
	}

	err = lsm303dlhc_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&stat->enabled, 1);

	err = lsm303dlhc_acc_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto err_power_off;
	}

	err = lsm303dlhc_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev, "sysfs register failed\n");
		goto err_power_off;
	}

	lsm303dlhc_acc_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	mutex_unlock(&stat->lock);

	dev_info(&client->dev, "probed successfully\n");

	return 0;

err_destroy_work_queue:
	if (stat->irq1_work_queue)
		destroy_workqueue(stat->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_power_off:
	lsm303dlhc_acc_device_power_off(stat);
err_pdata_init:
	if (stat->pdata->exit)
		stat->pdata->exit();
err_input_cleanup:
	lsm303dlhc_acc_input_cleanup(stat);
exit_kfree_pdata:
	kfree(stat->pdata->pm_platdata);
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);
err_freedata:
	kfree(stat);
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LSM303DLHC_ACC_DEV_NAME);
	return err;
}

static int lsm303dlhc_acc_remove(struct i2c_client *client)
{

	struct lsm303dlhc_acc_status *stat = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s\n", __func__);

	lsm303dlhc_acc_set_pinctrl_state(&client->dev,
			stat->pdata->pins_inactive);
	lsm303dlhc_acc_disable(stat);
	if (stat->pdata->gpio_int1 >= 0) {
		free_irq(stat->irq1, stat);
		gpio_free(stat->pdata->gpio_int1);
	}

	if (stat->pdata->gpio_int2 >= 0) {
		free_irq(stat->irq2, stat);
		gpio_free(stat->pdata->gpio_int2);
		destroy_workqueue(stat->irq2_work_queue);
	}

	lsm303dlhc_acc_input_cleanup(stat);
	remove_sysfs_interfaces(&client->dev);

	if (stat->pdata->exit)
		stat->pdata->exit();
	kfree(stat->pdata);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM
static int lsm303dlhc_acc_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_acc_status *stat = i2c_get_clientdata(client);

	dev_dbg(dev, "%s: resume\n", LSM303DLHC_ACC_DEV_NAME);
	lsm303dlhc_acc_set_pinctrl_state(dev,	stat->pdata->pins_default);

	if (stat->on_before_suspend)
		return lsm303dlhc_acc_enable(stat);

	return 0;
}

static int lsm303dlhc_acc_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_acc_status *stat = i2c_get_clientdata(client);

	dev_dbg(dev, "%s: suspend\n", LSM303DLHC_ACC_DEV_NAME);
	lsm303dlhc_acc_set_pinctrl_state(dev, stat->pdata->pins_sleep);
	stat->on_before_suspend = atomic_read(&stat->enabled);

	return lsm303dlhc_acc_disable(stat);
}
#else
#define lsm303dlhc_acc_suspend	NULL
#define lsm303dlhc_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm303dlhc_acc_id[] = {
	{ LSM303DLHC_ACC_DEV_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, lsm303dlhc_acc_id);

static const struct dev_pm_ops lsm303dlhc_acc_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm303dlhc_acc_suspend, lsm303dlhc_acc_resume)
};

static struct i2c_driver lsm303dlhc_acc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LSM303DLHC_ACC_DEV_NAME,
		.pm = &lsm303dlhc_acc_pm,
	},
	.probe = lsm303dlhc_acc_probe,
	.remove = lsm303dlhc_acc_remove,
	.id_table = lsm303dlhc_acc_id,
};

static int __init lsm303dlhc_acc_init(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&lsm303dlhc_acc_pm_class);
	if (ret) {
		pr_err("%s: %s: ERROR adding %s pm class\n",
				LSM303DLHC_ACC_DEV_NAME, __func__,
				lsm303dlhc_acc_pm_class.name);
		return ret;
	}
#endif
	pr_info("%s accelerometer driver init\n", LSM303DLHC_ACC_DEV_NAME);
	return i2c_add_driver(&lsm303dlhc_acc_driver);
}

static void __exit lsm303dlhc_acc_exit(void)
{

	pr_info("%s accelerometer driver exit\n", LSM303DLHC_ACC_DEV_NAME);

	i2c_del_driver(&lsm303dlhc_acc_driver);
	return;
}

module_init(lsm303dlhc_acc_init);
module_exit(lsm303dlhc_acc_exit);

MODULE_DESCRIPTION("lsm303dlhc accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, STMicroelectronics");
MODULE_LICENSE("GPL");
