/*
 *  Adaptations were made to original apds990x.c with
 *  apds990x.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2012-2014 Intel Mobile Communications GmbH
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
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
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
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
#include <linux/platform_device.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#else
#include <mach/gpio.h>
#endif
#include <linux/input/apds_990x.h>

#define APDS990x_DRV_NAME	"apds990x"
#define DRIVER_VERSION		"1.0.4"

#define APDS990x_INT				IRQ_EINT20

#define APDS_STARTUP_DELAY			5 /* ms */
#define APDS990x_PS_DETECTION_THRESHOLD		970
#define APDS990x_PS_HSYTERESIS_THRESHOLD	950

#define APDS990x_ALS_THRESHOLD_HSYTERESIS	20	/* 20 = 20% */

/* Change History
 *
 * 1.0.1	Functions apds990x_show_rev(), apds990x_show_id() and
 *			apds990x_show_status() have missing CMD_BYTE in the
 *			apds990x_read_byte(). APDS-990x needs CMD_BYTE for
 *			i2c write/read byte transaction.
 *
 *
 * 1.0.2	Include PS switching threshold level when interrupt occurred
 *
 *
 * 1.0.3	Implemented ISR and delay_work, correct PS threshold storing
 *
 * 1.0.4	Added Input Report Event
 */

/*
 * Defines
 */

#define APDS990x_ENABLE_REG	0x00
#define APDS990x_ATIME_REG	0x01
#define APDS990x_PTIME_REG	0x02
#define APDS990x_WTIME_REG	0x03
#define APDS990x_AILTL_REG	0x04
#define APDS990x_AILTH_REG	0x05
#define APDS990x_AIHTL_REG	0x06
#define APDS990x_AIHTH_REG	0x07
#define APDS990x_PILTL_REG	0x08
#define APDS990x_PILTH_REG	0x09
#define APDS990x_PIHTL_REG	0x0A
#define APDS990x_PIHTH_REG	0x0B
#define APDS990x_PERS_REG	0x0C
#define APDS990x_CONFIG_REG	0x0D
#define APDS990x_PPCOUNT_REG	0x0E
#define APDS990x_CONTROL_REG	0x0F
#define APDS990x_REV_REG	0x11
#define APDS990x_ID_REG		0x12
#define APDS990x_STATUS_REG	0x13
#define APDS990x_CDATAL_REG	0x14
#define APDS990x_CDATAH_REG	0x15
#define APDS990x_IRDATAL_REG	0x16
#define APDS990x_IRDATAH_REG	0x17
#define APDS990x_PDATAL_REG	0x18
#define APDS990x_PDATAH_REG	0x19

#define CMD_BYTE	0x80
#define CMD_WORD	0xA0
#define CMD_SPECIAL	0xE0

#define CMD_CLR_PS_INT	0xE5
#define CMD_CLR_ALS_INT	0xE6
#define CMD_CLR_PS_ALS_INT	0xE7

/* 100mA, IR-diode, 1X PGAIN, 1X AGAIN */
#define VAL_CONTROL_REG_APDS990x  0x20
/* 100mA, IR-diode, 4X PGAIN, 16X AGAIN */
#define VAL_CONTROL_REG_APDS9930  0x2A

#define VAL_ALS_GAIN_DEFAULT       48    /* without glass factor */
#define VAL_ALS_COEFF_B_DEFAULT    223   /* without glass factor */
#define VAL_ALS_COEFF_C_DEFAULT    70    /* without glass factor */
#define VAL_ALS_COEFF_D_DEFAULT    142   /* without glass factor */

/*
 * Structs
 */

struct apds990x_data {
	struct i2c_client *client;
	struct mutex update_lock;
	spinlock_t lock;
	struct delayed_work dwork; /* for PS interrupt */
	struct delayed_work als_dwork; /* for ALS polling */
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;
	struct als_platform_data *pdata;

	unsigned int part_id;
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
	unsigned int control_default;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

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
	/* dump ADC value and lux for calibration */
	unsigned int lux;
	unsigned int ch0;
	unsigned int ch1;
};

static int apds990x_init_client(struct i2c_client *client);
/*
 * Global data
 */

static int apds990x_read_byte(struct i2c_client *client, u8 reg)
{
	int retries = 5;
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
retry:
	if (ret < 0) {
		if (retries--)
			goto retry;
	}

	return ret;
}

static int apds990x_read_word(struct i2c_client *client, u8 reg)
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

static int apds990x_write_byte(struct i2c_client *client, u8 reg, u8 data)
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

static int apds990x_write_word(struct i2c_client *client, u8 reg, u16 data)
{
	int retries = 5;
	s32 ret;

	ret = i2c_smbus_write_word_data(client, reg, data);
retry:
	if (ret < 0) {
		if (retries--)
			goto retry;
	}

	return ret;
}

static int apds990x_set_command(struct i2c_client *client, int command)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	int clear_int;

	if (command == 0)
		clear_int = CMD_CLR_PS_INT;
	else if (command == 1)
		clear_int = CMD_CLR_ALS_INT;
	else
		clear_int = CMD_CLR_PS_ALS_INT;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte(client, clear_int);
	mutex_unlock(&data->update_lock);

	return ret;
}

static int apds990x_set_enable(struct i2c_client *client, int enable)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_byte(client, CMD_BYTE | APDS990x_ENABLE_REG,
						enable);
	mutex_unlock(&data->update_lock);
	data->enable = enable;

	return ret;
}

static int apds990x_set_atime(struct i2c_client *client, int atime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_byte(client, CMD_BYTE|APDS990x_ATIME_REG, atime);
	mutex_unlock(&data->update_lock);
	data->atime = atime;

	return ret;
}

static int apds990x_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_byte(client, CMD_BYTE|APDS990x_PTIME_REG, ptime);
	mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	return ret;
}

static int apds990x_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_byte(client, CMD_BYTE|APDS990x_WTIME_REG, wtime);
	mutex_unlock(&data->update_lock);

	data->wtime = wtime;

	return ret;
}

static int apds990x_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_word(client, CMD_WORD | APDS990x_AILTL_REG,
			threshold);
	mutex_unlock(&data->update_lock);

	data->ailt = threshold;

	return ret;
}

static int apds990x_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_word(client, CMD_WORD | APDS990x_AIHTL_REG,
			threshold);
	mutex_unlock(&data->update_lock);

	data->aiht = threshold;

	return ret;
}

static int apds990x_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_word(client, CMD_WORD | APDS990x_PILTL_REG,
			threshold);
	mutex_unlock(&data->update_lock);

	data->pilt = threshold;

	return ret;
}

static int apds990x_set_piht(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_word(client, CMD_WORD | APDS990x_PIHTL_REG,
			threshold);
	mutex_unlock(&data->update_lock);

	data->piht = threshold;

	return ret;
}

static int apds990x_set_pers(struct i2c_client *client, int pers)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_byte(client, CMD_BYTE|APDS990x_PERS_REG, pers);
	mutex_unlock(&data->update_lock);

	data->pers = pers;

	return ret;
}

static int apds990x_set_config(struct i2c_client *client, int config)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_byte(client, CMD_BYTE | APDS990x_CONFIG_REG,
			config);
	mutex_unlock(&data->update_lock);

	data->config = config;

	return ret;
}

static int apds990x_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_byte(client, CMD_BYTE | APDS990x_PPCOUNT_REG,
			ppcount);
	mutex_unlock(&data->update_lock);

	data->ppcount = ppcount;

	return ret;
}

static int apds990x_set_control(struct i2c_client *client, int control)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = apds990x_write_byte(client, CMD_BYTE | APDS990x_CONTROL_REG,
			control);
	mutex_unlock(&data->update_lock);

	data->control = control;

	/* obtain ALS gain value */
	if ((data->control&0x03) == 0x00) /* 1X Gain */
		data->als_gain = 1;
	else if ((data->control&0x03) == 0x01) /* 8X Gain */
		data->als_gain = 8;
	else if ((data->control&0x03) == 0x02) /* 16X Gain */
		data->als_gain = 16;
	else  /* 120X Gain */
		data->als_gain = 120;

	return ret;
}


static int lux_calculation(struct i2c_client *client, int cdata, int irdata)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int lux_value = 0;
	unsigned int GA = data->pdata->als_gain;
	unsigned int COE_B = data->pdata->coeff_b;
	unsigned int COE_C = data->pdata->coeff_c;
	unsigned int COE_D = data->pdata->coeff_d;

	int IAC1 = 0;
	int IAC2 = 0;
	int IAC = 0;
	int DF = 52;

	/* re-adjust COE_B to avoid 2 decimal point */
	IAC1 = (cdata - (COE_B * irdata) / 100);

	/* re-adjust COE_C and COE_D to void 2 decimal point */
	IAC2 = ((COE_C * cdata) / 100 - (COE_D * irdata) / 100);

	if (IAC1 > IAC2)
		IAC = IAC1;
	else if (IAC1 <= IAC2)
		IAC = IAC2;
	else
		IAC = 0;

	lux_value = ((IAC * GA * DF) / 100);
	lux_value /= (((272 * (256 - data->atime)) / 100) * data->als_gain);

	data->lux = lux_value;
	data->ch0 = cdata;
	data->ch1 = irdata;

	return lux_value;
}

static void apds990x_change_ps_threshold(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);

	data->ps_data =
		apds990x_read_word(client, CMD_WORD | APDS990x_PDATAL_REG);

	if ((data->ps_data > data->pilt) && (data->ps_data >= data->piht)) {
		/* far-to-near detected */
		data->ps_detection = 1;

		/* FAR-to-NEAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);
		input_sync(data->input_dev_ps);

		apds990x_write_word(client, CMD_WORD | APDS990x_PILTL_REG,
				data->ps_hysteresis_threshold);
		apds990x_write_word(client, CMD_WORD | APDS990x_PIHTL_REG,
				1023);

		data->pilt = data->ps_hysteresis_threshold;
		data->piht = 1023;

		dev_dbg(&client->dev, "%s far-to-near detected\n",
				APDS_990X_DEV_NAME);
	} else if (data->ps_data <= data->pilt && data->ps_data < data->piht) {
		/* near-to-far detected */
		data->ps_detection = 0;

		/* NEAR-to-FAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 5);
		input_sync(data->input_dev_ps);

		apds990x_write_word(client, CMD_WORD | APDS990x_PILTL_REG, 0);
		apds990x_write_word(client, CMD_WORD | APDS990x_PIHTL_REG,
				data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		dev_dbg(&client->dev, "%s near-to-far detected\n",
				APDS_990X_DEV_NAME);
	}
}

static void apds990x_change_als_threshold(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int cdata, irdata;
	int lux_value = 0;

	cdata = apds990x_read_word(client, CMD_WORD | APDS990x_CDATAL_REG);
	irdata = apds990x_read_word(client, CMD_WORD | APDS990x_IRDATAL_REG);

	lux_value = lux_calculation(client, cdata, irdata);

	lux_value = lux_value > 0 ? lux_value : 0;
	lux_value = lux_value < 10000 ? lux_value : 10000;

	/* check PS under sunlight */
	if (data->ps_detection == 1 &&
			(cdata > (75 * (1024 * (256 - data->atime)))/100)) {
		/* need to inform input event as there will be no
		 * interrupt from the PS
		 * NEAR-to-FAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);
		input_sync(data->input_dev_ps);

		apds990x_write_word(client, CMD_WORD|APDS990x_PILTL_REG, 0);
		apds990x_write_word(client, CMD_WORD|APDS990x_PIHTL_REG,
				data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		/* near-to-far detected */
		data->ps_detection = 0;
	}

	dev_dbg(&client->dev, "%s LuxValue = %d\n",
			APDS_990X_DEV_NAME, lux_value);
	/* report the lux level */
	input_report_abs(data->input_dev_als, ABS_MISC, lux_value);
	input_sync(data->input_dev_als);

	data->als_data = cdata;

	data->als_threshold_l = (data->als_data *
			(100 - APDS990x_ALS_THRESHOLD_HSYTERESIS)) / 100;
	data->als_threshold_h = (data->als_data *
			(100 + APDS990x_ALS_THRESHOLD_HSYTERESIS)) / 100;

	if (data->als_threshold_h >= 65535)
		data->als_threshold_h = 65535;

	apds990x_write_word(client, CMD_WORD | APDS990x_AILTL_REG,
			data->als_threshold_l);

	apds990x_write_word(client, CMD_WORD | APDS990x_AIHTL_REG,
			data->als_threshold_h);
}

/*
 * Management functions
 */
static void apds990x_work_handler(struct work_struct *work)
{
	struct apds990x_data *data = container_of(work,
					struct apds990x_data, dwork.work);
	struct i2c_client *client = data->client;
	int status;
	int cdata;

	status = apds990x_read_byte(client, CMD_BYTE | APDS990x_STATUS_REG);
	/* disable 990x's ADC first */
	apds990x_write_byte(client, CMD_BYTE | APDS990x_ENABLE_REG, 1);

	if ((status & data->enable & 0x30) == 0x30) {
		/* both PS and ALS are interrupted */
		apds990x_change_als_threshold(client);

		cdata =	apds990x_read_word(client,
				CMD_WORD | APDS990x_CDATAL_REG);

		if (cdata < (75 * (1024 * (256 - data->atime))) / 100)
			apds990x_change_ps_threshold(client);
		else
			dev_dbg(&client->dev,
				"%s Triggered by background ambient noise\n",
				APDS_990X_DEV_NAME);

		/* FIXME: 2 = CMD_CLR_PS_ALS_INT */
		apds990x_set_command(client, 2);
	} else if ((status & data->enable & 0x20) == 0x20) {
		/* only PS is interrupted */

		/* check if this is triggered by background ambient noise */
		cdata = apds990x_read_word(client,
				CMD_WORD | APDS990x_CDATAL_REG);
		if (cdata < (75 * (1024 * (256 - data->atime))) / 100)
			apds990x_change_ps_threshold(client);
		else
			dev_dbg(&client->dev,
				"%s Triggered by background ambient noise\n",
				APDS_990X_DEV_NAME);

		/* FIXME: 0 = CMD_CLR_PS_INT */
		apds990x_set_command(client, 0);
	} else if ((status & data->enable & 0x10) == 0x10) {
		/* only ALS is interrupted */
		apds990x_change_als_threshold(client);

		/* 1 = CMD_CLR_ALS_INT */
		apds990x_set_command(client, 1);
	}

	apds990x_write_byte(client, CMD_BYTE | APDS990x_ENABLE_REG,
			data->enable);
}

static void apds990x_reschedule_work(struct apds990x_data *data,
					  unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&data->dwork);
	schedule_delayed_work(&data->dwork, delay);

	spin_unlock_irqrestore(&data->lock, flags);
}

 /* assume this is ISR */
static irqreturn_t apds990x_interrupt(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct apds990x_data *data = i2c_get_clientdata(client);

	disable_irq_nosync(vec);
	apds990x_reschedule_work(data, 0);
	enable_irq(vec);

	return IRQ_HANDLED;
}

static int apds990x_power_off(struct apds990x_data *data)
{
	struct device_pm_platdata *pm_platdata = data->pdata->pm_platdata;
	int ret;

	if (data->pdata->gpio_int >= 0)
		disable_irq_nosync(data->client->irq);

	ret = device_state_pm_set_state_by_name(&data->client->dev,
			pm_platdata->pm_state_D3_name);

	if (ret)
		dev_err(&data->client->dev, "Power OFF Failed: %d\n", ret);

	return ret;
}

static int apds990x_power_on(struct apds990x_data *data)
{
	struct device_pm_platdata *pm_platdata = data->pdata->pm_platdata;
	int ret;

	ret = device_state_pm_set_state_by_name(&data->client->dev,
			pm_platdata->pm_state_D0_name);

	if (ret) {
		dev_err(&data->client->dev, "Power ON Failed: %d\n", ret);
		return ret;
	}

	msleep(APDS_STARTUP_DELAY);
	if (data->pdata->gpio_int >= 0)
		enable_irq(data->client->irq);

	return 0;
}

/*
 * SysFS support
 */

static ssize_t apds990x_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds990x_store_enable_ps_sensor(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	int err = -1;
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if ((val != 0) && (val != 1)) {
		dev_err(&client->dev, "%s: enable als sensor=%ld\n",
				__func__, val);
		return count;
	}

	dev_dbg(&client->dev,
		"%s ps sensor\n", (val == 1) ? "enable" : "disable");

	if (val == 1) {
		/* turn on prox sensor */
		if (data->enable_ps_sensor == 0) {
			data->enable_ps_sensor = 1;
			if (!data->enable_als_sensor) {
				err = apds990x_power_on(data);
				if (err)
					return err;
			}
			apds990x_set_enable(client, 0); /* Power Off */
			apds990x_set_atime(client, 0xf6); /* 27.2ms */
			apds990x_set_ptime(client, 0xff); /* 2.72ms */

			apds990x_set_ppcount(client, 8); /* 8-pulse */

			apds990x_set_control(client, data->control_default);

			apds990x_set_pilt(client,
					data->ps_hysteresis_threshold);
			apds990x_set_piht(client, 1023);

			data->ps_threshold = APDS990x_PS_DETECTION_THRESHOLD;

			apds990x_set_ailt(client, 0xffff);
			apds990x_set_aiht(client, 0);

			/* 3 persistence */
			apds990x_set_pers(client, 0x33);

		    if (data->enable_als_sensor == 0) {
				/* only enable PS interrupt */
				apds990x_set_enable(client, 0x27);
			} else
				apds990x_set_enable(client, 0x37);
		}
	} else {
		/* turn off p sensor - kk 25 Apr 2011 .
		 * we can't turn off the entire sensor,
		 * the light sensor may be needed by HAL */
		if (data->enable_ps_sensor == 1) {
			data->enable_ps_sensor = 0;
			if (data->enable_als_sensor) {
				/* reconfigute light sensor setting */
				/* Power Off */
				apds990x_set_enable(client, 0);

				/* previous als poll delay */
				apds990x_set_atime(client, data->als_atime);

				apds990x_set_ailt(client, 0xffff);
				apds990x_set_aiht(client, 0);

				apds990x_set_control(client,
						data->control_default);
				/* 3 persistence */
				apds990x_set_pers(client, 0x33);

				/* only enable light sensor */
				apds990x_set_enable(client, 0x13);
			} else {
				apds990x_set_enable(client, 0);
				apds990x_power_off(data);
			}
		}
	}

	return count;
}

static DEVICE_ATTR(enable_ps_sensor, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		apds990x_show_enable_ps_sensor,
		apds990x_store_enable_ps_sensor);

static ssize_t apds990x_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t apds990x_store_enable_als_sensor(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err = -1;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if ((val != 0) && (val != 1)) {
		dev_err(&client->dev, "%s: enable als sensor=%ld\n",
				__func__, val);
		return count;
	}

	dev_dbg(&client->dev,
		"%s als sensor\n", (val == 1) ? "enable" : "disable");

	if (val == 1) {
		/* turn on light  sensor */
		if (data->enable_als_sensor == 0) {

			data->enable_als_sensor = 1;
			if (!data->enable_ps_sensor) {
				err = apds990x_power_on(data);
				if (err)
					return err;
			}
			apds990x_set_enable(client, 0); /* Power Off */

			/* 100.64ms */
			apds990x_set_atime(client, data->als_atime);

			apds990x_set_ailt(client, 0xffff);
			apds990x_set_aiht(client, 0);

			apds990x_set_control(client, data->control_default);
			apds990x_set_pers(client, 0x33); /* 3 persistence */

			if (data->enable_ps_sensor) {
				apds990x_set_ptime(client, 0xff); /* 2.72ms */

				apds990x_set_ppcount(client, 8); /* 8-pulse */

				apds990x_set_pilt(client,
						data->ps_hysteresis_threshold);
				apds990x_set_piht(client, 1023);

				data->ps_threshold =
					APDS990x_PS_DETECTION_THRESHOLD;

				/* if prox sensor was activated previously */
				apds990x_set_enable(client, 0x37);
			} else
				/* only enable light sensor */
				apds990x_set_enable(client, 0x13);
		}
	} else {
		/* turn off light sensor
		 * what if the p sensor is active?*/
		if (data->enable_als_sensor == 1) {
			data->enable_als_sensor = 0;

			if (data->enable_ps_sensor) {
				apds990x_set_enable(client, 0); /* Power Off */
				apds990x_set_atime(client, 0xf6);  /* 27.2ms */
				apds990x_set_ptime(client, 0xff); /* 2.72ms */
				apds990x_set_ppcount(client, 8); /* 8-pulse */

				apds990x_set_control(client,
						data->control_default);

				apds990x_set_pilt(client,
						data->ps_hysteresis_threshold);
				apds990x_set_piht(client, 1023);

				apds990x_set_ailt(client, 0);
				apds990x_set_aiht(client, 0xffff);
				/* 3 persistence */
				apds990x_set_pers(client, 0x33);
				/* only enable prox sensor with interrupt */
				apds990x_set_enable(client, 0x27);
			} else {
				apds990x_set_enable(client, 0);
				apds990x_power_off(data);
			}
		}
	}

	return count;
}

static DEVICE_ATTR(enable_als_sensor, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		apds990x_show_enable_als_sensor,
		apds990x_store_enable_als_sensor);

static int apds990x_detect(struct i2c_client *client)
{
	int  id;

	id = apds990x_read_byte(client, CMD_BYTE | APDS990x_ID_REG);
	if (id < 0) {
		dev_err(&client->dev, "ID read failed\n");
		return id;
	}
	dev_info(&client->dev, "%s chip id %x\n",
			APDS_990X_DEV_NAME, id);

	return id;
}

static ssize_t apds990x_chip_id_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int id;

	id = apds990x_detect(client);
	return sprintf(buf, "%s %x\n",
			"APDS990x proximity and light sensor", id);
}

static DEVICE_ATTR(chip_id, S_IRUSR | S_IRGRP | S_IROTH,
		apds990x_chip_id_show, NULL);

static ssize_t apds990x_reg_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	ret = apds990x_detect(client);
	if (ret < 0)
		return 0;

	ret = apds990x_read_byte(client, CMD_BYTE|APDS990x_ENABLE_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_ENABLE_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_ATIME_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_ATIME_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_PTIME_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_PTIME_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_WTIME_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_WTIME_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_AILTL_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_AILTL_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_AILTH_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_AILTH_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_AIHTL_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_AIHTL_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_AIHTH_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_AIHTH_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_PILTL_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_PILTL_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_PILTH_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_PILTH_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_PIHTL_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_PIHTL_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_PIHTH_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_PIHTH_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_PERS_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_PERS_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_CONFIG_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_CONFIG_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_PPCOUNT_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_PPCOUNT_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_CONTROL_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_CONTROL_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_REV_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_REV_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_ID_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_ID_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_STATUS_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_STATUS_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_CDATAL_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_CDATAL_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_CDATAH_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_CDATAH_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_IRDATAL_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_IRDATAL_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_IRDATAH_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_IRDATAH_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_PDATAL_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_PDATAL_REG %x\n", ret);

	ret = apds990x_read_byte(client, CMD_BYTE | APDS990x_PDATAH_REG);
	if (ret < 0)
		dev_err(&client->dev, "read failed\n");
	dev_info(&client->dev, "APDS990x_PDATAH_REG %x\n", ret);

	return 0;
}

static ssize_t apds990x_reg_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
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
	 * Format of input value:
	 * bit[31..16] = don't care
	 * bit[15..8]  = register address
	 * bit[8..0]   = value to write into register
	 */
	regid = (val >> 8) & 0xff;
	val &= 0xff;

	if (regid < APDS990x_ENABLE_REG || regid > APDS990x_PDATAH_REG) {
		dev_err(&client->dev, "Register address out of range!\n");
		return -EINVAL;
	}

	i2c_buf[0] = (u8) regid;
	i2c_buf[1] = (u8) val;
	mutex_lock(&data->update_lock);
	ret = apds990x_write_byte(client, CMD_BYTE | i2c_buf[0], i2c_buf[1]);
	if (ret < 0)
		dev_err(&client->dev, "Write failed\n");

	mutex_unlock(&data->update_lock);

	return size;
}

static DEVICE_ATTR(reg, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		apds990x_reg_show,
		apds990x_reg_set);

static ssize_t apds990x_lux_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "lux:%d,ch0:%d,ch1:%d,again:%d,atime:0x%x\n",
			data->lux, data->ch0, data->ch1,
			data->als_gain, data->atime);
}

static DEVICE_ATTR(lux, S_IRUSR | S_IRGRP | S_IROTH, apds990x_lux_show, NULL);

static struct attribute *apds990x_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_reg.attr,
	&dev_attr_lux.attr,
	NULL
};

static const struct attribute_group apds990x_attr_group = {
	.attrs = apds990x_attributes,
};

/*
 * Initialization function
 */

static int apds990x_init_client(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int err;
	int id;

	err = apds990x_set_enable(client, 0);

	if (err < 0)
		return err;

	id = apds990x_read_byte(client, CMD_BYTE|APDS990x_ID_REG);
	if (id == 0x20) {
		dev_info(&client->dev, "%s APDS-9901\n", APDS_990X_DEV_NAME);
		data->part_id = 0x20;
		data->control_default = VAL_CONTROL_REG_APDS990x;
	} else if (id == 0x29) {
		dev_info(&client->dev, "%s APDS-990x\n", APDS_990X_DEV_NAME);
		data->part_id = 0x29;
		data->control_default = VAL_CONTROL_REG_APDS990x;
	} else if (id == 0x39) {
		dev_info(&client->dev, "%s APDS-9930\n", APDS_990X_DEV_NAME);
		data->part_id = 0x39;
		data->control_default = VAL_CONTROL_REG_APDS9930;
	} else {
		dev_err(&client->dev,
			"%s Neither APDS-9901 nor APDS-990x nor ADPS-9930\n",
			APDS_990X_DEV_NAME);
		return -EIO;
	}

	/* 100.64ms ALS integration time */
	apds990x_set_atime(client, 0xDB);
	/* 2.72ms Prox integration time */
	apds990x_set_ptime(client, 0xFF);
	/* 2.72ms Wait time */
	apds990x_set_wtime(client, 0xB6);

	/* 8-Pulse for proximity */
	apds990x_set_ppcount(client, 0x04);
	/* no long wait */
	apds990x_set_config(client, 0);

	apds990x_set_control(client, data->control_default);

	/* init threshold for proximity */
	apds990x_set_pilt(client, 0);
	apds990x_set_piht(client, APDS990x_PS_DETECTION_THRESHOLD);

	data->ps_threshold = APDS990x_PS_DETECTION_THRESHOLD;
	data->ps_hysteresis_threshold = APDS990x_PS_HSYTERESIS_THRESHOLD;

	/* init threshold for als */
	apds990x_set_ailt(client, 0);
	apds990x_set_aiht(client, 0xFFFF);

	/* 2 consecutive Interrupt persistence */
	apds990x_set_pers(client, 0x33);

	/* sensor is in disabled mode but all the configurations are preset */

	return 0;
}

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int apds990x_set_pm_state(
		struct device *dev, struct device_state_pm_state *state)
{
	return 0;
}
static struct device_state_pm_state *apds990x_get_initial_state(
		struct device *);

static struct device_state_pm_ops apds990x_pm_ops = {
	.set_state = apds990x_set_pm_state,
	.get_initial_state = apds990x_get_initial_state,
};

/* APDS990x PM states & class */
static struct device_state_pm_state apds990x_pm_states[] = {
	{ .name = "disable", }, /* D3 */
	{ .name = "enable", }, /* D0 */
};

DECLARE_DEVICE_STATE_PM_CLASS(apds990x);

static struct device_state_pm_state *apds990x_get_initial_state(
		struct device *dev)
{
	return &apds990x_pm_states[0];
}
#endif

#ifdef CONFIG_OF

#define OF_ALS_GAIN   "intel,als-gain"
#define OF_COEFF_B    "intel,coeff-B"
#define OF_COEFF_C    "intel,coeff-C"
#define OF_COEFF_D    "intel,coeff-D"

static struct als_platform_data *apds990x_of_get_platdata(
		struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct als_platform_data *als_pdata;
	struct device_pm_platdata *pm_platdata;
	int ret = 0;

	als_pdata = kzalloc(sizeof(*als_pdata), GFP_KERNEL);

	if (!als_pdata)
		return ERR_PTR(-ENOMEM);

	if (of_property_read_u32(np, OF_ALS_GAIN,
				&als_pdata->als_gain) < 0) {
		/* allow absent */
		als_pdata->als_gain = VAL_ALS_GAIN_DEFAULT;
		dev_info(dev, "%s of node(%s) not available, set to default\n",
				OF_ALS_GAIN, np->name);
	}

	if (of_property_read_u32(np, OF_COEFF_B,
				&als_pdata->coeff_b) < 0) {
		als_pdata->coeff_b = VAL_ALS_COEFF_B_DEFAULT;
		dev_info(dev, "%s of node(%s) not available, set to default\n",
				OF_COEFF_B, np->name);
	}

	if (of_property_read_u32(np, OF_COEFF_C,
				&als_pdata->coeff_c) < 0) {
		als_pdata->coeff_c = VAL_ALS_COEFF_C_DEFAULT;
		dev_info(dev, "%s of node(%s) not available, set to default\n",
				OF_COEFF_C, np->name);
	}

	if (of_property_read_u32(np, OF_COEFF_D,
				&als_pdata->coeff_d) < 0) {
		als_pdata->coeff_d = VAL_ALS_COEFF_D_DEFAULT;
		dev_info(dev, "%s of node(%s) not available, set to default\n",
				OF_COEFF_D, np->name);
	}

	/* pinctrl */
	als_pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(als_pdata->pinctrl))
		goto skip_pinctrl;

	als_pdata->pins_default = pinctrl_lookup_state(als_pdata->pinctrl,
			PINCTRL_STATE_DEFAULT);
	if (IS_ERR(als_pdata->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	als_pdata->pins_sleep = pinctrl_lookup_state(als_pdata->pinctrl,
			PINCTRL_STATE_SLEEP);
	if (IS_ERR(als_pdata->pins_sleep))
		dev_err(dev, "could not get sleep pinstate\n");

	als_pdata->pins_inactive = pinctrl_lookup_state(als_pdata->pinctrl,
			"inactive");
	if (IS_ERR(als_pdata->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

skip_pinctrl:
	pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pm_platdata)) {
		dev_err(dev, "Error during device state pm init\n");
		ret = -EINVAL;
		goto out;
	}

	als_pdata->pm_platdata = pm_platdata;
	als_pdata->gpio_int = 0;

	return als_pdata;

out:
	kfree(als_pdata);
	return ERR_PTR(ret);
}
#endif

static inline int apds990x_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	struct apds990x_data *data = dev_get_drvdata(dev);
	struct als_platform_data *pdata = data->pdata;
	int ret = 0;

	if (!pdata) {
		dev_err(dev, "Unable to retrieve apds990x platform data\n");
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	return ret;
}

/*
 * I2C init/probing/exit functions
 */
static int __init apds990x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds990x_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct apds990x_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

#ifdef CONFIG_OF
	data->pdata = apds990x_of_get_platdata(&client->dev);
	if (IS_ERR(data->pdata)) {
		err = PTR_ERR(data->pdata);
		data->pdata = NULL;
	}
#else
	data->pdata = client->dev.platform_data;
#endif
	if (!data->pdata) {
		dev_err(&client->dev,
		"%s No Platform data\n", APDS_990X_DEV_NAME);
		goto exit_kfree;
	}
	data->client = client;
	i2c_set_clientdata(client, data);

	data->enable = 0;	/* default mode is standard */
	data->ps_threshold = 0;
	data->ps_hysteresis_threshold = 0;
	data->ps_detection = 0;	/* default to no detection */
	data->enable_als_sensor = 0;	/* default to 0 */
	data->enable_ps_sensor = 0;	/* default to 0 */
	data->als_poll_delay = 100;	/* default to 100ms */
	/* work in conjuction with als_poll_delay */
	data->als_atime	= 0xdb;

	mutex_init(&data->update_lock);
	spin_lock_init(&data->lock);

#ifndef CONFIG_OF
	err = gpio_request(XGOLD_GPIO(data->pdata->gpio_int), "ALS_INTR");
	if (err) {
		dev_err(&client->dev, "%s gpio request failed\n",
				APDS_990X_DEV_NAME);
		goto exit_kfree;
	}

	client->irq = gpio_to_irq(XGOLD_GPIO(data->pdata->gpio_int));
	if (client->irq < 0) {
		dev_err(&client->dev, "%s gpio to irq failed\n",
				APDS_990X_DEV_NAME);
		goto exit_free_gpio;
	}

	dev_info(&client->dev,
			"%s: %s has set irq to irq: %d mapped on gpio:%d\n",
			APDS_990X_DEV_NAME, __func__, client->irq,
			data->pdata->gpio_int);
#endif

	if (request_irq(client->irq, apds990x_interrupt, IRQF_TRIGGER_FALLING,
		APDS990x_DRV_NAME, (void *)client)) {
		dev_err(&client->dev,
			"apds990x.c: Could not allocate APDS990x_INT !\n");

		goto exit_free_gpio;
	}

	disable_irq_nosync(client->irq);

	INIT_DELAYED_WORK(&data->dwork, apds990x_work_handler);

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

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 10000, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_als->name = "Avago light sensor";
	data->input_dev_ps->name = "Avago proximity sensor";

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

	/* pcl */
	apds990x_set_pinctrl_state(&client->dev, data->pdata->pins_default);

	if (!data->pdata->pm_platdata ||
			!data->pdata->pm_platdata->pm_state_D0_name ||
			!data->pdata->pm_platdata->pm_state_D3_name) {
		err = -EINVAL;
		dev_err(&client->dev, "Missing PM platdata\n");
		goto exit_unregister_dev_ps;
	}

	err = device_state_pm_set_class(&client->dev,
			data->pdata->pm_platdata->pm_user_name);
	if (err < 0) {
		dev_err(&client->dev, "Device PM Init Failed\n");
		goto exit_unregister_dev_ps;
	}

	err = apds990x_power_on(data);
	if (err < 0) {
		dev_err(&client->dev, "Power ON Failed\n");
		goto exit_plat_exit;
	}

	/* Initialize the APDS990x chip */
	err = apds990x_init_client(client);
	if (err < 0) {
		dev_err(&client->dev, "Device initialization Failed: %s\n",
		       data->input_dev_ps->name);
		goto exit_power_off;
	}

	err = apds990x_power_off(data);
	if (err)
		goto exit_plat_exit;

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds990x_attr_group);
	if (err)
		goto exit_power_off;

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

exit_power_off:
	apds990x_power_off(data);
exit_plat_exit:
	device_state_pm_remove_device(&client->dev);
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
#ifndef CONFIG_OF
	gpio_free(data->pdata->gpio_int);
#else
	kfree(data->pdata);
#endif
exit_kfree:
	kfree(data);
exit:
	return err;
}

static int apds990x_remove(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);

	/* Power down the device */
	apds990x_set_pinctrl_state(&client->dev, data->pdata->pins_inactive);
	apds990x_set_enable(client, 0);
	apds990x_power_off(data);
	input_unregister_device(data->input_dev_als);
	input_unregister_device(data->input_dev_ps);

	input_free_device(data->input_dev_als);
	input_free_device(data->input_dev_ps);

	free_irq(client->irq, client);

	sysfs_remove_group(&client->dev.kobj, &apds990x_attr_group);

	device_state_pm_remove_device(&client->dev);

	kfree(data->pdata->pm_platdata);
	kfree(data->pdata);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int apds990x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	dev_dbg(dev, "%s: suspend\n", APDS_990X_DEV_NAME);

	apds990x_set_pinctrl_state(dev, data->pdata->pins_sleep);
	if (data->enable_als_sensor == 1  || data->enable_ps_sensor == 1) {
		apds990x_set_enable(client, 0);
		apds990x_power_off(data);
	}
	return 0;
}

static int apds990x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int err = 0;
	struct apds990x_data *data = i2c_get_clientdata(client);

	dev_dbg(dev, "%s: resume\n", APDS_990X_DEV_NAME);

	apds990x_set_pinctrl_state(dev, data->pdata->pins_default);
	if (data->enable_als_sensor == 1 || data->enable_ps_sensor == 1) {
		err = apds990x_power_on(data);
		if (err) {
			dev_err(&client->dev, "Power ON Failed: %s\n",
					data->input_dev_ps->name);
			return err;
		}
		err = apds990x_set_enable(client, 1);
		if (err < 0) {
			dev_err(&client->dev, "Enable Failed: %s\n",
					data->input_dev_ps->name);
			apds990x_power_off(data);
			return err;
		}
	}
	return err;
}

#else

#define apds990x_suspend	NULL
#define apds990x_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id apds990x_id[] = {
	{ "apds990x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds990x_id);

static const struct dev_pm_ops apds990x_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(apds990x_suspend, apds990x_resume)
};

static struct i2c_driver apds990x_driver = {
	.driver = {
		.name	= APDS990x_DRV_NAME,
		.owner	= THIS_MODULE,
		.pm = &apds990x_pm,
	},
	.probe	= apds990x_probe,
	.remove	= apds990x_remove,
	.id_table = apds990x_id,
};

static int __init apds990x_init(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&apds990x_pm_class);
	if (ret) {
		pr_err("%s: %s: ERROR adding %s pm class\n",
				APDS_990X_DEV_NAME, __func__,
				apds990x_pm_class.name);
		return ret;
	}
#endif
	pr_info("%s: apds sysfs driver init\n", APDS_990X_DEV_NAME);
	return i2c_add_driver(&apds990x_driver);
}

static void __exit apds990x_exit(void)
{
	pr_info("%s exit\n", APDS_990X_DEV_NAME);
	i2c_del_driver(&apds990x_driver);
	return;
}

module_init(apds990x_init);
module_exit(apds990x_exit);

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS990x ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
