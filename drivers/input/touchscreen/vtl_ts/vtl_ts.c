/*
 * VTL CTP driver
 *
 * Copyright (C) 2013 VTL Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 */
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>




#include "vtl_ts.h"
#include "chip.h"
#include "apk.h"


#define		TS_THREAD_PRIO		90
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static struct ts_config_info *vtl_ts_of_get_platdata(struct i2c_client *client);
static int vtl_request_irq(struct ts_info *ts);
static void vtl_irq_enable(struct ts_info *ts);
static void vtl_irq_disable(struct ts_info *ts);


static unsigned char thread_syn_flag;
struct ts_config_info *vtl_pdata_t;
spinlock_t irq_lock;



/****************************************************************************
 Globel or static variables
****************************************************************************/
static struct ts_driver	g_driver;

struct ts_info	g_ts = {
	.driver = &g_driver,
	.debug  = DEBUG_ENABLE,
};
struct ts_info	*pg_ts = &g_ts;





/****************************************************************************
 Function declaration
****************************************************************************/

static int vtl_ts_config(struct ts_info *ts)
{
	struct device *dev;
	int err;

	DEBUG();

	dev = &ts->driver->client->dev;
	/* ts config */
	ts->config_info->touch_point_number = TOUCH_POINT_NUM;

	/* IRQ config*/
	ts->config_info->irq_number =
		gpio_to_irq(ts->config_info->irq_gpio_number);

	err = gpio_request(ts->config_info->rst_gpio_number, "vtl_ts_rst");
	if (err)
		return -EIO;

	gpio_direction_output(ts->config_info->rst_gpio_number, 1);

	return 0;
}


struct ts_info *vtl_ts_get_object(void)
{
	DEBUG();

	return pg_ts;
}

void vtl_ts_hw_reset(void)
{
	struct ts_info *ts;
	ts = pg_ts;
	DEBUG();

	gpio_set_value(ts->config_info->rst_gpio_number, 0);
	msleep(50);
	gpio_set_value(ts->config_info->rst_gpio_number, 1);
	msleep(20);
	chip_solfware_reset(ts->driver->client);/*20140306*/
}

static void vtl_ts_wakeup(void)
{
	struct ts_info *ts;
	ts = pg_ts;
	DEBUG();

	gpio_set_value(ts->config_info->rst_gpio_number, 0);
	/*msleep(50);*/
	msleep(20);
	gpio_set_value(ts->config_info->rst_gpio_number, 1);
	msleep(20);
	chip_solfware_reset(ts->driver->client);/*20140306*/
}

static irqreturn_t vtl_ts_irq(int irq, void *dev)
{
	struct ts_info *ts;
	ts = pg_ts;

	DEBUG();

	vtl_irq_disable(ts);/* Disable ts interrupt*/
	thread_syn_flag = 1;
	wake_up_interruptible(&waiter);

	return IRQ_HANDLED;
}


static int vtl_ts_read_xy_data(struct ts_info *ts)
{
	struct i2c_client *client;
	struct i2c_msg msgs;
	int ret;

	DEBUG();
	client = ts->driver->client;

	msgs.addr = ts->driver->client->addr;
	msgs.flags = 0x01;  /* 0x00: write 0x01:read */
	msgs.len = sizeof(ts->xy_data.buf);
	msgs.buf = ts->xy_data.buf;
	ret = i2c_transfer(ts->driver->client->adapter, &msgs, 1);
	if (ret != 1) {
		pr_debug("___%s:i2c read xy_data err___\n", __func__);
		return -1;
	}
	return 0;
}

static void vtl_ts_report_xy_coord(struct ts_info *ts)
{

	int id;
	int sync;
	int x, y;
	unsigned int press;
	unsigned char touch_point_number;
	/*Static variables are initialised to 0 by GCC. */
	static unsigned int release;
	struct input_dev *input_dev;
	union ts_xy_data *xy_data;

	DEBUG();

	xy_data = &ts->xy_data;
	input_dev = ts->driver->input_dev;
	touch_point_number = ts->config_info->touch_point_number;

	if (((xy_data->point[0].status != 1) && (xy_data->point[0].status != 2)
		&& (xy_data->point[0].status != 3))) {/*20140221*/

		vtl_ts_hw_reset();
		for (id = 0; id < touch_point_number; id++) {
			input_mt_slot(input_dev, id);

			input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
		}
		press = 0;
		release = 0;
		return;
	}

	/* report points */
	sync = 0;  press = 0;
	/*down */
	for (id = 0; id < touch_point_number; id++) {
		if ((xy_data->point[id].xhi != 0xFF) &&
			(xy_data->point[id].yhi != 0xFF) &&
			((xy_data->point[id].status == 1) ||
			(xy_data->point[id].status == 2))) {

		#if (XY_SWAP_ENABLE)
			x = (xy_data->point[id].yhi<<4)|(
				xy_data->point[id].ylo&0xF);
			y = (xy_data->point[id].xhi<<4)|
				(xy_data->point[id].xlo&0xF);
		#else
			x = (xy_data->point[id].xhi<<4)|(
				xy_data->point[id].xlo&0xF);
			y = (xy_data->point[id].yhi<<4)|
				(xy_data->point[id].ylo&0xF);
		#endif
		#if (X_REVERSE_ENABLE)
			x = ts->config_info->screen_max_x - x;
		#endif
		#if (Y_REVERSE_ENABLE)
			y = ts->config_info->screen_max_y - y;
		#endif

		pr_debug("id = %d,status = %d,X = %d,Y = %d\n",
				xy_data->point[id].id,
				xy_data->point[id].status, x, y);
			input_mt_slot(input_dev, xy_data->point[id].id - 1);
			input_report_abs(input_dev, ABS_MT_TRACKING_ID,
				xy_data->point[id].id-1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 1);

			press |= 0x01 << (xy_data->point[id].id - 1);
			sync = 1;

		}

	}
	release &= (release ^ press);/*release point flag*/
	/*up*/
	for (id = 0; id < touch_point_number; id++) {
		if (release & (0x01<<id)) {
			pr_debug("release touch,id=%d", id);
			input_mt_slot(input_dev, id);
			input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
			sync = 1;
		}
	}

	release = press;
	if (sync)
		input_sync(input_dev);

}



int vtl_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ts_info *ts;
	unsigned char i;
	ts = pg_ts;

	DEBUG();
	if (ts->config_info->ctp_used) {
		vtl_irq_disable(ts);
		chip_enter_sleep_mode();

		for (i = 0; i < ts->config_info->touch_point_number; i++) {
			input_mt_slot(ts->driver->input_dev, i);
			input_report_abs(ts->driver->input_dev,
				ABS_MT_TRACKING_ID, -1);
		}
		input_sync(ts->driver->input_dev);
	}
	return 0;
}

int vtl_ts_resume(struct i2c_client *client)
{
	struct ts_info *ts;
	unsigned char i;
	ts = pg_ts;

	DEBUG();
	if (ts->config_info->ctp_used) {
		vtl_ts_wakeup();
		for (i = 0; i < ts->config_info->touch_point_number; i++) {
			input_mt_slot(ts->driver->input_dev, i);
			input_report_abs(ts->driver->input_dev,
				ABS_MT_TRACKING_ID, -1);
		}
		input_sync(ts->driver->input_dev);

		vtl_irq_enable(ts);
	}
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void vtl_ts_early_suspend(struct early_suspend *handler)
{
	struct ts_info *ts;
	ts = pg_ts;

	DEBUG();

	vtl_ts_suspend(ts->driver->client, PMSG_SUSPEND);
}

static void vtl_ts_early_resume(struct early_suspend *handler)
{
	struct ts_info *ts;
	ts = pg_ts;

	DEBUG();

	vtl_ts_resume(ts->driver->client);
}
#endif


static int  vtl_ts_remove(struct i2c_client *client)
{
	struct ts_info *ts;
	ts = pg_ts;

	DEBUG();

	free_irq(ts->config_info->irq_number, ts);
	gpio_free(ts->config_info->rst_gpio_number);
	/*vtl_ts_free_gpio(); */

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->driver->early_suspend);
	#endif
	if (ts->driver->input_dev != NULL) {
		input_unregister_device(ts->driver->input_dev);
		input_free_device(ts->driver->input_dev);
	}

	if (ts->driver->proc_entry != NULL)
		remove_proc_entry(DRIVER_NAME, NULL);

	if (ts->driver->ts_thread != NULL) {
		pr_debug("___kthread stop start___\n");
		thread_syn_flag = 1;
		wake_up_interruptible(&waiter);
		kthread_stop(ts->driver->ts_thread);
		ts->driver->ts_thread = NULL;
		pr_debug("___kthread stop end___\n");
	}
	return 0;
}

static int vtl_ts_init_input_dev(struct ts_info *ts)
{
	struct input_dev *input_dev;
	struct device *dev;
	int err;

	DEBUG();


	dev = &ts->driver->client->dev;

	/* allocate input device */
	ts->driver->input_dev = input_allocate_device();
	if (ts->driver->input_dev == NULL) {
		dev_err(dev,
		"Unable to allocate input device for device %s.\n",
			 DRIVER_NAME);
		return -1;
	}

	input_dev = ts->driver->input_dev;

	input_dev->name = DRIVER_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor  = 0xaaaa;
	input_dev->id.product = 0x5555;
	input_dev->id.version = 0x0001;

	/* config input device */
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);



	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, TOUCH_POINT_NUM, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
		0, ts->config_info->screen_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
		0, ts->config_info->screen_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
		0, ts->config_info->touch_point_number, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);


	/* register input device */
	err = input_register_device(input_dev);
	if (err) {
		input_free_device(ts->driver->input_dev);
		ts->driver->input_dev = NULL;
		dev_err(dev,
		"Unable to register input device for device %s.\n",
			DRIVER_NAME);
		return -1;
	}

	return 0;
}


static int vtl_ts_handler(void *data)
{
	int ret;
	struct device *dev;
	struct ts_info *ts;

	DEBUG();
	pr_debug("%s\n", __func__);

	ts = (struct ts_info *)data;
	dev = &ts->driver->client->dev;

	ret = vtl_ts_config(ts);
	if (ret) {

		dev_err(dev, "VTL touch screen config Failed.\n");
		goto ERR_TS_CONFIG;
	}

	vtl_ts_hw_reset();


	ret = chip_init();
	if (ret) {

		dev_err(dev, "vtl ts chip init failed.\n");
		goto ERR_CHIP_INIT;
	}

	/*init input dev*/
	ret = vtl_ts_init_input_dev(ts);
	if (ret) {

		dev_err(dev, "init input dev failed.\n");
		goto ERR_INIT_INPUT;
	}

	/* Create Proc Entry File */
	ts->driver->proc_entry = proc_create(DRIVER_NAME,
		0666/*S_IFREG | S_IRUGO | S_IWUSR*/, NULL, &apk_fops);

	/* register early suspend */
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->driver->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->driver->early_suspend.suspend = vtl_ts_early_suspend;
	ts->driver->early_suspend.resume = vtl_ts_early_resume;
	register_early_suspend(&ts->driver->early_suspend);
#endif

	/* Init irq */
	ret = vtl_request_irq(ts);
	if (ret) {
		dev_err(dev,
		"Unable to request irq for device %s.\n",
			DRIVER_NAME);
		goto ERR_IRQ_REQ;
	}

	ts->config_info->ctp_used = 1;
	while (!kthread_should_stop()) {
		wait_event_interruptible(waiter, thread_syn_flag);
		thread_syn_flag = 0;
		ret = vtl_ts_read_xy_data(ts);

		if (!ret)
			vtl_ts_report_xy_coord(ts);
		else
			pr_debug("____read xy_data error___\n");

		/* Enable ts interrupt */
		vtl_irq_enable(pg_ts);
	}

	pr_debug("vtl_ts_Kthread exit,%s(%d)\n", __func__, __LINE__);
	return 0;




ERR_IRQ_REQ:
	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->driver->early_suspend);
	#endif
	if (ts->driver->proc_entry) {
		remove_proc_entry(DRIVER_NAME, NULL);
		ts->driver->proc_entry = NULL;
	}

	/*ERR_PROC_ENTRY:*/
	if (ts->driver->input_dev) {
		input_unregister_device(ts->driver->input_dev);
		input_free_device(ts->driver->input_dev);
		ts->driver->input_dev = NULL;
	}
ERR_INIT_INPUT:
ERR_CHIP_INIT:
	gpio_free(ts->config_info->rst_gpio_number);
ERR_TS_CONFIG:
	ts->config_info->ctp_used = 0;
	pr_debug("vtl_ts_Kthread exit,%s(%d)\n", __func__, __LINE__);
	return 0;
}

#ifdef CONFIG_OF

#define OF_VTL_SUPPLY  "i2c"
#define OF_VTL_SUPPLY_A  "i2ca"
#define OF_VTL_PIN_RESET "intel,ts-gpio-reset"
#define OF_VTL_PIN_IRQ   "intel,ts-gpio-irq"

static struct ts_config_info *vtl_ts_of_get_platdata(
	struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct ts_config_info *vtl_pdata;
	struct regulator *pow_reg;
	int ret;
	vtl_pdata = devm_kzalloc(&client->dev,
		sizeof(struct  ts_config_info), GFP_KERNEL);
	if (!vtl_pdata)
		return ERR_PTR(-ENOMEM);


	/* regulator */

	pow_reg = regulator_get(&client->dev, OF_VTL_SUPPLY);
	vtl_pdata->power = pow_reg;
	if (IS_ERR(pow_reg)) {
		dev_warn(&client->dev, "%s can't get %s-supply handle\n",
			 np->name, OF_VTL_SUPPLY);
		vtl_pdata->power = NULL;
	}

	pow_reg = regulator_get(&client->dev, OF_VTL_SUPPLY_A);
	vtl_pdata->power2 = pow_reg;
	if (IS_ERR(pow_reg)) {
		dev_warn(&client->dev, "%s can't get %s-supply handle\n",
			 np->name, OF_VTL_SUPPLY_A);
		vtl_pdata->power2 = NULL;
	}

	/* pinctrl */
	vtl_pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(vtl_pdata->pinctrl)) {
		ret = PTR_ERR(vtl_pdata->pinctrl);
		goto out;
	}

	vtl_pdata->pins_default = pinctrl_lookup_state(
		vtl_pdata->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(vtl_pdata->pins_default))
		dev_err(&client->dev, "could not get default pinstate\n");

	vtl_pdata->pins_sleep = pinctrl_lookup_state(
		vtl_pdata->pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR(vtl_pdata->pins_sleep))
		dev_err(&client->dev, "could not get sleep pinstate\n");

	vtl_pdata->pins_inactive = pinctrl_lookup_state(
		vtl_pdata->pinctrl, "inactive");
	if (IS_ERR(vtl_pdata->pins_inactive))
		dev_err(&client->dev, "could not get inactive pinstate\n");

	/* gpio reset */
	vtl_pdata->rst_gpio_number =
		of_get_named_gpio_flags(client->dev.of_node,
			OF_VTL_PIN_RESET, 0, NULL);
	if (vtl_pdata->rst_gpio_number <= 0) {
		dev_err(&client->dev,
			"error getting gpio for %s\n",  OF_VTL_PIN_RESET);
		ret = -EINVAL;
		goto out;
	}
	/* gpio irq */
	vtl_pdata->irq_gpio_number =
		of_get_named_gpio_flags(client->dev.of_node,
			OF_VTL_PIN_IRQ, 0, NULL);
	if (vtl_pdata->irq_gpio_number <= 0) {
		dev_err(&client->dev,
			"error getting gpio for %s\n",  OF_VTL_PIN_IRQ);
		ret = -EINVAL;
		goto out;
	}

	/* interrupt mode */
	if (of_property_read_bool(np, "intel,polling-mode"))
		vtl_pdata->polling_mode = true;

	/* pm */
	vtl_pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(vtl_pdata->pm_platdata)) {
		dev_warn(&client->dev, "Error during device state pm init\n");
		ret = PTR_ERR(vtl_pdata->pm_platdata);
		goto out;
	}
	ret = of_property_read_u32(np, "intel,max-x",
		&vtl_pdata->screen_max_x);
	if (ret != 0) {
		pr_debug("get screen max x error\n");
		ret = -EINVAL;
		goto out;
	}
	ret = of_property_read_u32(np, "intel,max-y",
		&vtl_pdata->screen_max_y);
	if (ret != 0) {
		pr_debug("get screen max y error\n");
		ret = -EINVAL;
		goto out;
	}
	pr_debug("max_x = %d, max_y=%d\n",
		vtl_pdata->screen_max_x, vtl_pdata->screen_max_y);
	return vtl_pdata;

out:
	return ERR_PTR(ret);
}
#endif


int vtl_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = -1;
	struct ts_info *ts;
	struct device *dev;

	DEBUG();
	pr_debug("%s\n", __func__);
	ts = pg_ts;
	ts->driver->client = client;
	dev = &ts->driver->client->dev;


	/* Check I2C Functionality */
	err = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!err) {
		dev_err(dev, "Check I2C Functionality Failed.\n");
		return -ENODEV;
	}

	ts->config_info = vtl_ts_of_get_platdata(client);

	if (IS_ERR(ts->config_info)) {
		err = PTR_ERR(ts->config_info);
		return err;
	}

	spin_lock_init(&irq_lock);		  /* 2.6.39 later */
	ts->driver->ts_thread = kthread_run(vtl_ts_handler, ts, DRIVER_NAME);
	if (IS_ERR(ts->driver->ts_thread)) {
		err = PTR_ERR(ts->driver->ts_thread);
		ts->driver->ts_thread = NULL;
		dev_err(dev, "failed to create kernel thread: %d\n",  err);
		return -1;
	}

	pr_debug("___%s() end____\n", __func__);

	return 0;



}

static int vtl_request_irq(struct ts_info *ts)
{
	int ret = -1;

	ret = request_irq(ts->driver->client->irq,
				vtl_ts_irq,
				IRQF_TRIGGER_FALLING,
				DRIVER_NAME,
					ts);
	return ret;
}

void vtl_irq_disable(struct ts_info *ts)
{
	unsigned long irqflags;

	spin_lock_irqsave(&irq_lock, irqflags);
	disable_irq_nosync(ts->driver->client->irq);
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

void vtl_irq_enable(struct ts_info *ts)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&irq_lock, irqflags);
	enable_irq(ts->driver->client->irq);
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

#define Vtl_I2C_NAME  "VTL-TS"

static const struct i2c_device_id vtl_ts_id[] = {
	{ Vtl_I2C_NAME, 0 },
	{ }
};

struct i2c_driver vtl_ts_driver  = {

	.driver = {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME
	},
	.id_table	= vtl_ts_id,
	.probe		= vtl_ts_probe,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= vtl_ts_suspend,
	.resume		= vtl_ts_resume,
#endif
	.remove		= vtl_ts_remove,
};


int __init vtl_ts_init(void)
{
	DEBUG();
	return i2c_add_driver(&vtl_ts_driver);
}

void __exit vtl_ts_exit(void)
{
	DEBUG();
	i2c_del_driver(&vtl_ts_driver);
}

module_init(vtl_ts_init);
module_exit(vtl_ts_exit);

MODULE_AUTHOR("yangdechu@vtl.com.cn");
MODULE_DESCRIPTION("VTL touchscreen driver for rockchip,V1.0");
MODULE_LICENSE("GPL");
