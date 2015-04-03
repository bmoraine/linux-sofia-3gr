/*
 *  Adaptations were made to original apds990x.c with
 *  ltr559.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/input/ltr559.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <asm/setup.h>
#include <linux/version.h>

#define LTR559_DRV_NAME	"ltr559"
#define DRIVER_VERSION	"1.0.0"
#define LTR559_STARTUP_DELAY		5 /* ms */
#define LTR559_PS_DETECTION_THRESHOLD	20
#define LTR559_PS_HSYTERESIS_THRESHOLD	1


/*
 * Defines
 */

#define CMD_BYTE	0x80
#define CMD_WORD	0xA0
#define CMD_SPECIAL	0xE0

#define LTR559_WTIME_REG	0x03
#define LTR559_CONFIG_REG	0x0D

#define CMD_CLR_PS_INT	0xE5
#define CMD_CLR_ALS_INT	0xE6
#define CMD_CLR_PS_ALS_INT	0xE7


/* LTR-559 Registers */
#define LTR559_ALS_CONTR		0x80
#define LTR559_PS_CONTR			0x81
#define LTR559_PS_LED			0x82
#define LTR559_PS_N_PULSES		0x83
#define LTR559_PS_MEAS_RATE		0x84
#define LTR559_ALS_MEAS_RATE		0x85
#define LTR559_CHIP_PARD_ID			0x86
#define LTR559_MANUFACTURER_ID		0x87

#define LTR559_INTERRUPT		0x8F
#define LTR559_PS_THRES_UP_0		0x90
#define LTR559_PS_THRES_UP_1		0x91
#define LTR559_PS_THRES_LOW_0		0x92
#define LTR559_PS_THRES_LOW_1		0x93

#define LTR559_ALS_THRES_UP_0		0x97
#define LTR559_ALS_THRES_UP_1		0x98
#define LTR559_ALS_THRES_LOW_0		0x99
#define LTR559_ALS_THRES_LOW_1		0x9A
#define LTR559_INTERRUPT_PERSIST	0x9E

/* 559's Read Only Registers */
#define LTR559_ALS_DATA_CH1_0		0x88
#define LTR559_ALS_DATA_CH1_1		0x89
#define LTR559_ALS_DATA_CH0_0		0x8A
#define LTR559_ALS_DATA_CH0_1		0x8B
#define LTR559_ALS_PS_STATUS		0x8C
#define LTR559_PS_DATA_0		0x8D
#define LTR559_PS_DATA_1		0x8E


/*
 * Structs
 */

struct ltr559_data {
	struct i2c_client *client;
	struct mutex update_lock;
	spinlock_t lock;
	struct delayed_work	dwork;	/* for PS interrupt */
	struct delayed_work    als_dwork; /* for ALS polling */
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;
	struct als_platform_data  *pdata;

	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

	unsigned int enable_ps_sensor_pm;
	unsigned int enable_als_sensor_pm;

	/* PS parameters */
	unsigned int ps_threshold;
	/* always lower than ps_threshold */
	unsigned int ps_hysteresis_threshold;
	/* 0 = near-to-far; 1 = far-to-near */
	unsigned int ps_detection;
	/* to store PS data */
	unsigned int ps_data;

	/* ALS parameters */
	/* low threshold */
	unsigned int als_threshold_l;
	/* high threshold */
	unsigned int als_threshold_h;
	/* to store ALS data */
	unsigned int als_data;

	/* needed for Lux calculation */
	unsigned int als_gain;
	/* needed for light sensor polling : micro-second (us) */
	unsigned int als_poll_delay;
	/* storage for als integratiion time */
	unsigned int als_atime;

	struct device_pm_platdata *pm_platdata;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
};

/*
 * Global data
 */

static int ltr559_read_byte(struct i2c_client *client, u8 reg)
{
	int retries = 5;
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
retry:
	if (ret < 0)	{
		if (retries--)
			goto retry;
	}
	return ret;
}

static int ltr559_read_word(struct i2c_client *client, u8 reg)
{
	int retries = 5;
	s32 ret;

	ret = i2c_smbus_read_word_data(client, reg);
retry:
	if (ret < 0) {
		if (retries--)
			goto retry;
	}
	return ret;
}

static int ltr559_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	int retries = 5;
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, data);
retry:
	if (ret < 0) {
		if (retries--)
			goto retry;
	}
	return ret;
}

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int ltr559_set_pm_state(struct device *dev,
			struct device_state_pm_state *state)
{
	/* FIXME */
	return 0;
}

static struct device_state_pm_state *ltr559_get_initial_state(
		struct device *);

static struct device_state_pm_ops ltr559_pm_ops = {
	.set_state = ltr559_set_pm_state,
	.get_initial_state = ltr559_get_initial_state,
};

/* ltr559 PM states & class */
static struct device_state_pm_state ltr559_pm_states[] = {
	{ .name = "disable", }, /* D3 */
	{ .name = "enable", },  /* D0 */
};

DECLARE_DEVICE_STATE_PM_CLASS(ltr559);

static struct device_state_pm_state *ltr559_get_initial_state(
		struct device *dev)
{
	return &ltr559_pm_states[0];
}
#endif

#ifdef CONFIG_OF
static void ltr559_pm_power_on(struct i2c_client *client)
{
	int ret;
	struct ltr559_data *data = i2c_get_clientdata(client);

	ret = device_state_pm_set_state_by_name(&client->dev,
		data->pm_platdata->pm_state_D0_name);
	if (ret)
		dev_err(&client->dev, "%s: pm on failed\n", __func__);
}

static void ltr559_pm_power_off(struct i2c_client *client)
{
	int ret;
	struct ltr559_data *data = i2c_get_clientdata(client);

	ret = device_state_pm_set_state_by_name(&client->dev,
		data->pm_platdata->pm_state_D3_name);
	if (ret)
		dev_err(&client->dev, "%s: pm off failed\n", __func__);
}
#endif

static int ltr559_set_ps_enable(struct i2c_client *client, int enable)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);

	if (enable == 0)
		ret = ltr559_write_byte(client, LTR559_PS_CONTR, 0x00);
	else
		ret = ltr559_write_byte(client, LTR559_PS_CONTR, 0x02);

	mutex_unlock(&data->update_lock);
	data->enable = enable;

	return ret;
}

static int ltr559_set_als_enable(struct i2c_client *client, int enable)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);

	if (enable == 0)
		ret = ltr559_write_byte(client, LTR559_ALS_CONTR, 0x00);
	else
		ret = ltr559_write_byte(client, LTR559_ALS_CONTR, 0x19);

	mutex_unlock(&data->update_lock);
	data->enable = enable;

	return ret;
}




static int ltr559_set_atime(struct i2c_client *client, int atime)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = ltr559_write_byte(client, LTR559_ALS_MEAS_RATE, atime);
	mutex_unlock(&data->update_lock);
	data->atime = atime;

	return ret;
}

static int ltr559_set_ptime(struct i2c_client *client, int ptime)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = ltr559_write_byte(client, LTR559_PS_MEAS_RATE, ptime);
	mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	return ret;
}

static int ltr559_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = ltr559_write_byte(client, LTR559_PS_N_PULSES,
					ppcount);
	mutex_unlock(&data->update_lock);

	data->ppcount = ppcount;

	return ret;
}

static void ltr559_change_ps_threshold(struct i2c_client *client)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ps_low, ps_high;
	int ret;
	static int temp = 5;

	ps_low =  ltr559_read_byte(client, LTR559_PS_DATA_0);
	ps_high =  ltr559_read_byte(client, LTR559_PS_DATA_1);
	data->ps_data = (ps_high << 8) + ps_low;

	dev_info(&client->dev,
		"%s: ps_data=%d, ps_threshold=%d, ps_hysteresis=%d\n",
		__func__, data->ps_data, data->ps_threshold,
		data->ps_hysteresis_threshold);

	if (data->ps_data > data->ps_threshold) {
		/* far-to-near detected */
		data->ps_detection = 1;

		/* FAR-to-NEAR detection */
		temp = 0;
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);
		input_sync(data->input_dev_ps);

		ret = ltr559_write_byte(client, 0x90, 0xff);
		ret = ltr559_write_byte(client, 0x91, 0x07);
		ret = ltr559_write_byte(client, 0x92,
			data->ps_hysteresis_threshold & 0xff);
		ret = ltr559_write_byte(client, 0x93,
			(data->ps_hysteresis_threshold >> 8) & 0xff);

		dev_info(&client->dev,
			"%s far-to-near detected\n", LTR559_DEV_NAME);
	} else if (data->ps_data < data->ps_hysteresis_threshold) {
		/* near-to-far detected */
		data->ps_detection = 0;

		/* NEAR-to-FAR detection */
		temp = 5;
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 5);
		input_sync(data->input_dev_ps);

		ret = ltr559_write_byte(client, 0x90,
			data->ps_threshold & 0xff);
		ret = ltr559_write_byte(client, 0x91,
			(data->ps_threshold >> 8) & 0xff);
		ret = ltr559_write_byte(client, 0x92, 0);
		ret = ltr559_write_byte(client, 0x93, 0);

		dev_info(&client->dev,
			"%s near-to-far detected\n", LTR559_DEV_NAME);
	} else {
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, temp);
		input_sync(data->input_dev_ps);
	}
}

static void ltr559_change_als_threshold(struct i2c_client *client)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int cdata, irdata;
	int luxValue = 0;
	int alsval_ch0;
	int alsval_ch1;
	int ch1_co, ch0_co, ratio;

	irdata = ltr559_read_word(client, 0x88);
	cdata = ltr559_read_word(client, 0x8a);

	alsval_ch1 = irdata;
	alsval_ch0 = cdata;
		if ((alsval_ch0 + alsval_ch1) == 0)
			ratio = 1000;
		else
			ratio = alsval_ch1 * 1000 / (alsval_ch1 + alsval_ch0);

		if (ratio < 450) {
				ch0_co = 17743;
				ch1_co = -11059;
		} else if ((ratio >= 450) && (ratio < 640)) {
				ch0_co = 42785;
				ch1_co = 19548;
		} else if ((ratio >= 640) && (ratio < 850)) {
				ch0_co = 5926;
				ch1_co = -1185;
		} else if (ratio >= 850) {
				ch0_co = 0;
				ch1_co = 0;
		}
		luxValue = (alsval_ch0 * ch0_co - alsval_ch1 * ch1_co) / 10000;
		/* report the lux level */
		input_report_abs(data->input_dev_als, ABS_MISC, luxValue);
		input_sync(data->input_dev_als);
}

/*
 * Management functions
 */
static void ltr559_work_handler(struct work_struct *work)
{
	struct ltr559_data *data = container_of(work,
					struct ltr559_data, dwork.work);
	struct i2c_client *client = data->client;
	int	status;

	status = ltr559_read_byte(client, LTR559_ALS_PS_STATUS);

	if (((status & 0x03) == 0x03) || ((status & 0x0c) == 0x0c)) {
		/*for ps*/
		if ((status & 0x03) == 0x03)
			ltr559_change_ps_threshold(client);
		else
			ltr559_change_als_threshold(client);
	}
}

static void ltr559_reschedule_work(struct ltr559_data *data,
					  unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	__cancel_delayed_work(&data->dwork);
	schedule_delayed_work(&data->dwork, delay);

	spin_unlock_irqrestore(&data->lock, flags);
}

 /* assume this is ISR */
static irqreturn_t ltr559_interrupt(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct ltr559_data *data = i2c_get_clientdata(client);

	ltr559_reschedule_work(data, 0);
	/* enable_irq(vec); */

	return IRQ_HANDLED;
}

/*
 * SysFS support
 */

static ssize_t ltr559_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr559_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t ltr559_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr559_data *data = i2c_get_clientdata(client);
	int err;
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dev_info(&client->dev,
		"%s enable ps sensor\n", LTR559_DEV_NAME);
	if ((val != 0) && (val != 1)) {
		dev_err(&client->dev, "%s: enable als sensor=%ld\n",
					__func__, val);
		return count;
	}

	if (val == 1) {
		/* turn on ps sensor if need */
		if (data->enable_ps_sensor == 0) {
			data->enable_ps_sensor = 1;
#ifdef CONFIG_OF
			ltr559_pm_power_on(client);
#endif
			mdelay(10);
			err = ltr559_set_ps_enable(client, 1);
			if (err < 0) {
				dev_err(&client->dev, "%s: ps_enable on fail\n",
					__func__);
				return err;
			}

			if (0 == data->enable_als_sensor) {
				err = ltr559_set_als_enable(client, 0);
				if (err < 0) {
					dev_err(&client->dev, "%s: als_enable on fail\n",
						__func__);
					return err;
				}
			}
		}
	} else {
		/* turn off ps sensor if need */
		if (data->enable_ps_sensor == 1) {
			data->enable_ps_sensor = 0;
			err = ltr559_set_ps_enable(client, 0);
			mdelay(10);
#ifdef CONFIG_OF
			if ((data->enable_ps_sensor == 0) &&
				(data->enable_als_sensor == 0))
				ltr559_pm_power_off(client);
#endif
			if (err < 0) {
				dev_err(&client->dev, "%s: ps_enable off fail\n",
					__func__);
				return err;
			}
		}
	}

	return count;
}


static DEVICE_ATTR(enable_ps_sensor, 0666,
				ltr559_show_enable_ps_sensor,
				ltr559_store_enable_ps_sensor);

static ssize_t ltr559_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr559_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t ltr559_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr559_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int ret = 0;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dev_info(&client->dev,
		"%s enable als sensor\n", LTR559_DEV_NAME);
	if ((val != 0) && (val != 1)) {
		dev_err(&client->dev, "%s: enable als sensor=%ld\n",
					 __func__, val);
		return count;
	}

	if (val == 1) {
		/* turn on als sensor if need */
		if (data->enable_als_sensor == 0) {
			data->enable_als_sensor = 1;
#ifdef CONFIG_OF
			ltr559_pm_power_on(client);
#endif
			mdelay(10);
			/*set the interrutp condition for als*/
			ret = ltr559_write_byte(client,
				LTR559_ALS_THRES_UP_0, 0x00);
			ret = ltr559_write_byte(client,
				LTR559_ALS_THRES_UP_1, 0x00);

			ret = ltr559_set_als_enable(client, 1);
			if (ret < 0) {
				dev_err(&client->dev, "%s: als_enable on fail\n",
					__func__);
				return ret;
			}

			if (0 == data->enable_ps_sensor) {
				ret = ltr559_set_ps_enable(client, 0);
				if (ret < 0) {
					dev_err(&client->dev, "%s: ps_enable on fail\n",
						__func__);
					return ret;
				}
			}
		}
	} else {
		/* turn off als sensor if need */
		if (data->enable_als_sensor == 1) {
			data->enable_als_sensor = 0;
			ret = ltr559_write_byte(client,
					LTR559_ALS_THRES_UP_0, 0xff);
			ret = ltr559_write_byte(client,
					LTR559_ALS_THRES_UP_1, 0xff);
			ret = ltr559_set_als_enable(client, 0);
			mdelay(10);
#ifdef CONFIG_OF
			if ((data->enable_ps_sensor == 0) &&
				(data->enable_als_sensor == 0))
				ltr559_pm_power_off(client);
#endif
			if (ret < 0)
				dev_err(&client->dev, "%s: als_enable off fail\n",
					__func__);
		}
	}

	return count;
}

static DEVICE_ATTR(enable_als_sensor, 0666,
				ltr559_show_enable_als_sensor,
				ltr559_store_enable_als_sensor);

static int ltr559_detect(struct i2c_client *client)
{
	int  id;

	id = ltr559_read_byte(client, 0x86);
	if (id < 0) {
		dev_err(&client->dev, "ID read failed\n");
		return id;
	}
	dev_info(&client->dev,
		"%s chip id 0x%x\n", LTR559_DEV_NAME, id);

	return id;
}

static ssize_t ltr559_chip_id_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int id;

	id = ltr559_detect(client);
	return sprintf(buf, "%s %x\n", "LTR559 proximity and light sensor",
						id);
}

static DEVICE_ATTR(chip_id, S_IRUGO, ltr559_chip_id_show, NULL);

static ssize_t ltr559_reg_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	ret = ltr559_detect(client);
	if (ret < 0)
		return 0;

	ret = ltr559_read_byte(client, 0x80);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, " 0x80: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x81);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x81: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x82);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x82: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x83);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x83: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x84);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x84: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x85);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x85: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x86);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x86: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x87);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x87: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x88);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x88: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x89);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x89: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x8a);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x8a: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x8b);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x8b: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x8c);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x8c: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x8d);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x8d: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x8e);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x8e: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x8f);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x8f: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x90);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x90: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x91);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x91: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x92);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x92: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x93);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x93: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x94);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x94: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x95);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x95: 0x%x\n", ret);

	ret = ltr559_read_byte(client, 0x9e);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "0x9e: 0x%x\n", ret);

	return 0;
}

static ssize_t ltr559_reg_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret;
	unsigned int val;
	int regid;
	u8 i2c_buf[2];

	if (sscanf(buf, "%x", &val) == 0) {
		dev_err(&client->dev,
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

	if (regid < 0x80 || regid > 0x9f) {
		dev_err(&client->dev,
			"Register address out of range!\n");
		return -EINVAL;
	}

	i2c_buf[0] = (u8) regid;
	i2c_buf[1] = (u8) val;
	mutex_lock(&data->update_lock);
	ret = ltr559_write_byte(client, i2c_buf[0],
						i2c_buf[1]);
	if (ret < 0)
		dev_err(&client->dev, "Write failed\n");
	mutex_unlock(&data->update_lock);

	return size;
}

static DEVICE_ATTR(reg, S_IRUGO, ltr559_reg_show, ltr559_reg_set);

static struct attribute *ltr559_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_reg.attr,
	NULL,
};


static const struct attribute_group ltr559_attr_group = {
	.attrs = ltr559_attributes,
};

/*
 * Initialization function
 */

static int ltr559_init_chip(struct i2c_client *client)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int err;
	int id;
	int led_test;
	int ret;

	err = ltr559_set_ps_enable(client, 0);
	if (err < 0) {
		dev_err(&client->dev, "%s: ps_enable fail\n", __func__);
		return err;
	}

	err = ltr559_set_als_enable(client, 0);
	if (err < 0) {
		dev_err(&client->dev, "%s: als_enable fail\n", __func__);
		return err;
	}

	id = ltr559_read_byte(client, 0x86);
	if (id == 0x92) {
		dev_info(&client->dev,
		"%s LTR-ltr559\n", LTR559_DEV_NAME);
	} else {
		dev_info(&client->dev,
		"%s not LTR-ltr559\n", LTR559_DEV_NAME);
		return -EIO;
	}

	/* 100.64ms ALS integration time */
	ltr559_set_atime(client, 0x03);
	/* 2.72ms Prox integration time */
	ltr559_set_ptime(client, 0x0);
	/* 2.72ms Wait time */
	/*ltr559_set_wtime(client, 0xB6);*/

	/* 8-Pulse for proximity */
	ltr559_set_ppcount(client, 0x04);
	led_test = ltr559_read_byte(client, 0x83);

	ret = ltr559_write_byte(client, 0x82, 0x7b);
	ret = ltr559_write_byte(client, 0x8f, 0x3);
	ret = ltr559_write_byte(client, 0x9E, 0x22);

	/* init threshold for proximity */
	ret = ltr559_write_byte(client, 0x90,
			LTR559_PS_DETECTION_THRESHOLD & 0xff);
	ret = ltr559_write_byte(client, 0x91,
			(LTR559_PS_DETECTION_THRESHOLD >> 8) & 0xff);
	ret = ltr559_write_byte(client, 0x92,
			LTR559_PS_HSYTERESIS_THRESHOLD & 0xff);
	ret = ltr559_write_byte(client, 0x93,
		(LTR559_PS_HSYTERESIS_THRESHOLD >> 8) & 0xff);

	data->ps_threshold = LTR559_PS_DETECTION_THRESHOLD;
	data->ps_hysteresis_threshold = LTR559_PS_HSYTERESIS_THRESHOLD;

	ret = ltr559_write_byte(client, LTR559_ALS_MEAS_RATE,
			0x13);
	/* init threshold for als */
	/*ltr559_set_ailt(client, 0);*/
	/*ltr559_set_aiht(client, 0xFFFF);*/

	/* 2 consecutive Interrupt persistence */
	/*ltr559_set_pers(client, 0x22);*/

	/* sensor is in disabled mode but all the configurations are preset */

	return 0;
}


/*
 * I2C init/probing/exit functions
 */

static inline int ltr559_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	struct ltr559_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(data->pinctrl, state);
		if (ret)
			dev_err(dev, "%s: pinctrl_select_state fail\n",
				__func__);
	}
	return ret;
}

static struct i2c_driver ltr559_driver;
static int __init ltr559_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ltr559_data *data;
	struct device_node *np = NULL;
	int err = 0;

	dev_info(&client->dev, "%s: enter!\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct ltr559_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	data->client = client;
	i2c_set_clientdata(client, data);

	data->enable = 0;	/* default mode is standard */
	data->ps_threshold = 0;
	data->ps_hysteresis_threshold = 0;
	data->ps_detection = 0;	/* default to no detection */
	data->enable_als_sensor = 0;	/* default to 0 */
	data->enable_ps_sensor = 0;	/* default to 0 */
	data->enable_als_sensor_pm = 0;	/* default to 0 */
	data->enable_ps_sensor_pm = 0;	/* default to 0 */
	data->als_poll_delay = 100;	/* default to 100ms */
	/* work in conjuction with als_poll_delay */
	data->als_atime	= 0xdb;

	dev_info(&client->dev, "enable = %s\n",
			data->enable ? "1" : "0");

	/* lock init */
	mutex_init(&data->update_lock);
	spin_lock_init(&data->lock);

	if (request_irq(client->irq, ltr559_interrupt, IRQF_TRIGGER_FALLING,
		LTR559_DRV_NAME, (void *)client)) {
		dev_err(&client->dev,
			"ltr559.c: Could not allocate ltr559_INT !\n");
		goto exit_free_gpio;
	}

	disable_irq_nosync(client->irq);

	INIT_DELAYED_WORK(&data->dwork, ltr559_work_handler);

	/* Register to Input Device */
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		err = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device als\n");
		goto exit_free_irq;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device ps\n");
		goto exit_free_dev_als;
	}

	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 65535, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_als->name = "Liteon light sensor";
	data->input_dev_ps->name = "Liteon proximity sensor";

	err = input_register_device(data->input_dev_als);
	if (err) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"Unable to register input device als: %s\n",
				data->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"Unable to register input device ps: %s\n",
				data->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

#ifdef CONFIG_OF
		/* pinctrl */
		data->pinctrl = devm_pinctrl_get(&client->dev);
		if (IS_ERR(data->pinctrl))
			dev_err(&client->dev, "%s: devm_pinctrl_get err\n",
					__func__);

		data->pins_default = pinctrl_lookup_state(data->pinctrl,
			PINCTRL_STATE_DEFAULT);

		if (IS_ERR(data->pins_default))
			dev_err(&client->dev, "%s: could not get default pinstate\n",
					__func__);

		data->pins_sleep = pinctrl_lookup_state(data->pinctrl,
			PINCTRL_STATE_SLEEP);
		if (IS_ERR(data->pins_sleep))
			dev_err(&client->dev, "%s: could not get sleep pinstate\n",
					__func__);

		data->pins_inactive = pinctrl_lookup_state(data->pinctrl,
								"inactive");
		if (IS_ERR(data->pins_inactive))
			dev_err(&client->dev, "%s: could not get inactive pinstate\n",
					__func__);

		/* device pm */
		np = client->dev.of_node;
		if (np == NULL) {
			dev_err(&client->dev, "%s: np null\n", __func__);
		} else {
			data->pm_platdata = of_device_state_pm_setup(np);
			if (IS_ERR(data->pm_platdata)) {
				dev_err(&client->dev, "%s: state_pm_setup faield\n",
						__func__);
			}
		}

		/* attach this device driver to pm class */
		err = device_state_pm_set_class(&client->dev,
				data->pm_platdata->pm_user_name);
		if (err < 0)
			dev_err(&client->dev, "%s: state_pm_set_class fail\n",
					__func__);

		/* pm set state */
		ltr559_pm_power_on(client);

		/* set pins default */
#ifdef CONFIG_PM
		err = ltr559_set_pinctrl_state(&client->dev,
				data->pins_default);
		if (err)
			dev_err(&client->dev, "%s: set_pins_default fail\n",
					__func__);
#endif
#endif

	/* Initialize the LTR559 chip */
	err = ltr559_init_chip(client);
	if (err < 0) {
		dev_err(&client->dev, "ltr559 Device initialization Failed: %s\n",
		       data->input_dev_ps->name);
		goto exit_unregister_dev_ps;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &ltr559_attr_group);
	if (err)
		goto exit_unregister_dev_ps;

	enable_irq(client->irq);

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);
	dev_info(&client->dev, "%s: exit\n", __func__);

	return 0;

exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
exit_unregister_dev_als:
	input_unregister_device(data->input_dev_als);
exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
exit_free_dev_als:
	input_free_device(data->input_dev_als);
exit_free_irq:
	free_irq(client->irq, client);
exit_free_gpio:
	kfree(data);
exit:
	return err;
}

static int __exit ltr559_remove(struct i2c_client *client)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int err;

	/* Power down the device */
	err = ltr559_set_ps_enable(client, 0);
	if (err < 0)
		dev_err(&client->dev, "%s: ps_enable fail\n", __func__);

	err = ltr559_set_als_enable(client, 0);
	if (err < 0)
		dev_err(&client->dev, "%s: als_enable fail\n", __func__);

	input_unregister_device(data->input_dev_als);
	input_unregister_device(data->input_dev_ps);

	input_free_device(data->input_dev_als);
	input_free_device(data->input_dev_ps);

	free_irq(client->irq, client);

	sysfs_remove_group(&client->dev.kobj, &ltr559_attr_group);

	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int ltr559_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret = 0;

	ret = ltr559_set_pinctrl_state(dev, data->pins_sleep);
	if (ret) {
		dev_err(&client->dev, "%s: set_pins_sleep fail\n",
			__func__);
	}

	if (data->enable_ps_sensor) {
		data->enable_ps_sensor_pm = 1;
		ret = ltr559_set_ps_enable(client, 0);
		if (ret < 0) {
			dev_err(&client->dev, "suspend ps_enable failed: %s\n",
				data->input_dev_ps->name);
			goto out;
		}
	}

	if (data->enable_als_sensor) {
		data->enable_als_sensor_pm = 1;
		ret = ltr559_set_als_enable(client, 0);
		if (ret < 0) {
			dev_err(&client->dev, "suspend als_enable failed: %s\n",
				data->input_dev_als->name);
			goto out;
		}
	}
#ifdef CONFIG_OF
	ltr559_pm_power_off(client);
#endif
	return 0;
out:
	return ret;
}

static int ltr559_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret = 0;

#ifdef CONFIG_OF
	ltr559_pm_power_on(client);
#endif

	ret = ltr559_set_pinctrl_state(dev, data->pins_default);
	if (ret)
		dev_err(&client->dev, "%s: set_pins_default fail\n",
				__func__);

	if (data->enable_ps_sensor_pm) {
		ret = ltr559_set_ps_enable(client, 1);
		if (ret < 0) {
			dev_err(&client->dev, "resume ps_enable failed: %s\n",
				data->input_dev_ps->name);
			goto out;
		}
	}

	if (data->enable_als_sensor_pm) {
		ret = ltr559_set_als_enable(client, 1);
		if (ret < 0) {
			dev_err(&client->dev, "resume als_enable failed: %s\n",
				data->input_dev_als->name);
			goto out;
		}
	}

	data->enable_ps_sensor_pm = 0;
	data->enable_als_sensor_pm = 0;
	return 0;
out:
	return ret;
}

#else

#define ltr559_suspend	NULL
#define ltr559_resume	NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id ltr559_id[] = {
	{ "ltr559", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltr559_id);
static const struct dev_pm_ops ltr559_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(ltr559_suspend, ltr559_resume)
};

static struct i2c_driver ltr559_driver = {
	.driver = {
		.name	= LTR559_DRV_NAME,
		.owner	= THIS_MODULE,
		.pm = &ltr559_pm,
	},
	.probe	= ltr559_probe,
	.remove	= ltr559_remove,
	.id_table = ltr559_id,
};

static int __init ltr559_init(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&ltr559_pm_class);
	if (ret) {
		pr_err("%s: %s: ERROR adding %s pm class\n",
			LTR559_DEV_NAME, __func__,
			ltr559_pm_class.name);
		return ret;
	}
#endif

	pr_info("%s: ltr559 sysfs driver init\n", LTR559_DEV_NAME);
	return i2c_add_driver(&ltr559_driver);
}

static void __exit ltr559_exit(void)
{
	pr_info("%s exit\n", LTR559_DEV_NAME);
	i2c_del_driver(&ltr559_driver);
	return;
}

module_init(ltr559_init);
module_exit(ltr559_exit);

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("LTR559 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
