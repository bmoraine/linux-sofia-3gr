/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/input/xgold_headset.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include "agold620-headset.h"

MODULE_DESCRIPTION("Headset Detection Driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL");

#define XGOLD_KEY_HOOK			233
#define VBIAS_SETTLING_TIME_MS	20

/* Confgure the sub-device to detect
 * 1. Accessory Insertion/Removal
 * 2. Hook Press/Release
**/
static void configure_headset(enum xgold_headset_config val,
			struct xgold_headset_device_data *headset_device_data)
{
	u32 set;

	dev_dbg(headset_device_data->sdev.dev, "Configure val = %d\n", val);

	switch (val) {
	case XGOLD_DET_HEADSET_INS:
		set = XGOLD_DETECT_INSERTION;
		break;
	case XGOLD_DET_HEADSET_REM:
		set = XGOLD_DETECT_REMOVAL;
		break;
	case XGOLD_DET_HEADSET_REM_HOOK_PRESS:
		set = XGOLD_DETECT_REMOVAL_HOOK;
		break;
	case XGOLD_DET_HOOK_RELEASE:
		set = XGOLD_DETECT_HOOK_RELEASE;
		break;
	default:
		set = XGOLD_DETECT_INSERTION;
		break;
	}

	dev_dbg(headset_device_data->sdev.dev,
			"Write ACD_BAT_DET = %x\n", set);
	iowrite32(set, headset_device_data->mmio_base);

	return;
}

/* Read the sub-device state
 * 1. Accessory Inserted/Removed
 * 2. Hook Pressed/Released
**/
static int read_state(enum xgold_device dev,
			struct xgold_headset_device_data *headset_device_data,
			int volt)
{
	int i;

	if (dev == XGOLD_HEADSET) {
		if ((volt >= AHJ_TYPE_MIN_MV) &&
			(volt <= AHJ_TYPE_MAX_MV))
			return XGOLD_HEAD_SET;
		else if ((volt >= HEADPHONE_MIN_MV) &&
			(volt <= HEADPHONE_MAX_MV))
			return XGOLD_HEAD_PHONE;
		else if ((volt > 1700))
			return XGOLD_HEADSET_REMOVED;
		else
			return XGOLD_INVALID;
	} else if (dev == XGOLD_HOOK) {
		for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
			if ((volt >= xgold_hs_keymap[i].min_mv) &&
				(volt <= xgold_hs_keymap[i].max_mv)) {
				xgold_hs_keymap[i].pressed = 1;
				dev_dbg(headset_device_data->sdev.dev,
					"index = %x\n", i);
				return i;
			}
		}
		return -1;
	} else
		return XGOLD_INVALID;
}

/* Call to AFE to change the VBIAS settings */
static void configure_vbias(
			struct xgold_headset_device_data *headset_device_data,
			enum xgold_vbias state)
{
	switch (state) {
	case XGOLD_VBIAS_ENABLE:
		ACC_VBIAS_ENABLE;
		break;
	case XGOLD_VBIAS_ULP_ON:
		ACC_VBIAS_ULP_ON;
		break;
	default:
		break;
	}
}

static void init_keymap(
			struct xgold_headset_device_data *headset_device_data)
{
	headset_device_data->hook_cfg = xgold_hs_keymap;
	headset_device_data->size_of_hook_cfg = ARRAY_SIZE(xgold_hs_keymap);
}

struct xgold_headset_platform_data headset_platform_data = {
	.xgold_configure_headset = &configure_headset,
	.xgold_read_state = &read_state,
	.xgold_configure_vbias = &configure_vbias,
	.xgold_headset_keymap = &init_keymap,
};

/**
 * Hook Key Interrupt Handler
**/
static irqreturn_t hook_det_lowisr(int irq, void *dev_id)
{
	struct xgold_headset_device_data *headset_device_data =
		(struct xgold_headset_device_data *)dev_id;

	dev_dbg(headset_device_data->sdev.dev,
			"Entering %s at %d\n", __func__ , __LINE__);

	if (!headset_device_data) {
		pr_err("Hook: Error headset_device_data = NULL\n");
		return IRQ_HANDLED;
	}

	schedule_delayed_work(&headset_device_data->hook_work, 0);

	dev_dbg(headset_device_data->sdev.dev,
			"Leaving %s at %d\n" , __func__ , __LINE__);
	return IRQ_HANDLED;
}

/**
 * Delayed Work to handle Hook key interrupt
**/
static void hook_input_work(struct work_struct *work)
{
	int hook_state;
	int volt, ret, i;
	enum xgold_headset_config val;
	struct xgold_headset_device_data *data =
		container_of(to_delayed_work(work),
			struct xgold_headset_device_data, hook_work);

	dev_dbg(data->sdev.dev, "Entering %s at %d\n", __func__ , __LINE__);

	/* Disable the Hook IRQ here */
	disable_irq_nosync(data->irq_hook);

	ret = iio_read_channel_raw(data->iio_client, &volt);
	if (ret < 0) {
		dev_err(data->sdev.dev, "Unable to read channel volt\n");
		return;
	}

	dev_dbg(data->sdev.dev, "hook key volt %d\n", volt);
	/* State will indicate if pressed or released */
	hook_state = data->pdata->xgold_read_state(XGOLD_HOOK, data, volt);
	if (hook_state >= 0)
		val = XGOLD_DET_HOOK_RELEASE;
	else {
		val = XGOLD_DET_HEADSET_REM_HOOK_PRESS;
		for (i = 0; i < data->size_of_hook_cfg; i++)
			if (data->hook_cfg[i].pressed) {
				data->hook_cfg[i].pressed = 0;
				hook_state = i;
				break;
			}
	}

	/* Report the state to the input subsystem */
	input_report_key(data->input_dev, data->hook_cfg[hook_state].key_code,
				data->hook_cfg[hook_state].pressed);
	input_sync(data->input_dev);

	/* Re-configure the hook settings to detect press/release
	 * as per current state */
	data->pdata->xgold_configure_headset(val, data);

	/* Re-enable the Hook IRQ here */
	enable_irq(data->irq_hook);

	dev_dbg(data->sdev.dev, "Leaving %s at %d\n" , __func__ , __LINE__);
}

/**
 * Accessory Interrupt Handler
**/
static irqreturn_t plug_det_lowisr(int irq, void *dev_id)
{
	struct xgold_headset_device_data *headset_device_data =
		(struct xgold_headset_device_data *)dev_id;

	dev_dbg(headset_device_data->sdev.dev,
			"Entering %s at %d\n", __func__ , __LINE__);

	if (!headset_device_data) {
		pr_err("Error headset_device_data = NULL\n");
		return IRQ_HANDLED;
	}

	schedule_delayed_work(&headset_device_data->work,
		msecs_to_jiffies(0));

	dev_dbg(headset_device_data->sdev.dev,
			"Leaving %s at %d\n" , __func__ , __LINE__);

	wake_lock(&headset_device_data->acc_lock);

	return IRQ_HANDLED;
}

static struct irqaction headset_irq = {
	.name = "headset_irq",
	.handler = plug_det_lowisr,
	.flags = IRQF_SHARED,
};

static struct irqaction hook_irq = {
	.name = "hook_irq",
	.handler = hook_det_lowisr,
	.flags = IRQF_SHARED | IRQF_DISABLED,
};

/**
 * Delayed Work to handle Accessory Interrupt
**/
static void headset_switch_work(struct work_struct *work)
{
	u32 state, old_state;
	int ret, volt;
	enum xgold_headset_config val;
	bool hook_enable;

	struct xgold_headset_device_data *data =
		container_of(to_delayed_work(work),
			struct xgold_headset_device_data, work);

	dev_dbg(data->sdev.dev, "Entering %s at %d\n", __func__ , __LINE__);

	/*  Calling abb_pmu_irq from lowisr is causing Kernel Crash.
		Hence disabling the interrupts here*/
	disable_irq_nosync(data->irq_headset);

	data->pdata->xgold_configure_vbias(data, XGOLD_VBIAS_ENABLE);

read_iio_volt:
	msleep(250);
	ret = iio_read_channel_raw(data->iio_client, &volt);
	if (ret < 0) {
		dev_err(data->sdev.dev, "Unable to read channel volt\n");
		wake_unlock(&data->acc_lock);
		return;
	}
	/* State will indicate if pressed or released */
	old_state = data->pdata->xgold_read_state(
							XGOLD_HEADSET,
							data,
							volt);
	dev_dbg(data->sdev.dev, "old_state = %d volt = %d\n",
				(u32)old_state, volt);
	msleep(250);
	ret = iio_read_channel_raw(data->iio_client, &volt);
	if (ret < 0) {
		dev_err(data->sdev.dev, "Unable to read channel volt\n");
		wake_unlock(&data->acc_lock);
		return;
	}
	/* State will indicate if pressed or released */
	state = data->pdata->xgold_read_state(
							XGOLD_HEADSET,
							data,
							volt);
	dev_dbg(data->sdev.dev, "state = %d volt = %d\n", (u32)state, volt);
	if (state != old_state)
		goto read_iio_volt;

	if (state == XGOLD_INVALID) {
		/* Invalid Head set*/
		/* Enable Accessory interrupts */
		enable_irq(data->irq_headset);
		wake_unlock(&data->acc_lock);
		return;
	}
	/* Report to switch class */
	switch_set_state(&data->sdev, state);

	if ((state == XGOLD_HEAD_SET) || (state == XGOLD_HEAD_PHONE)) {
		/* If Inserted, confgure to detect Removal */
		dev_dbg(data->sdev.dev, "INSERTED\n");

		/* Ask AFE to change the VBIAS settings */

		if (state == XGOLD_HEAD_PHONE) {
			hook_enable = 0;
			val = XGOLD_DET_HEADSET_REM;
			data->pdata->xgold_configure_vbias(data,
						XGOLD_VBIAS_ULP_ON);
		} else {
			hook_enable = 1;
			val = XGOLD_DET_HEADSET_REM_HOOK_PRESS;
			data->pdata->xgold_configure_vbias(data,
						XGOLD_VBIAS_ENABLE);
		}
	} else {
		/* If Removed, confgure to detect Insertion */
		dev_dbg(data->sdev.dev, "REMOVED\n");

		/* Ask AFE to change the VBIAS settings */
		data->pdata->xgold_configure_vbias(data, XGOLD_VBIAS_ULP_ON);
		hook_enable = 0;
		val = XGOLD_DET_HEADSET_INS;
	}

	/* Delay for the Biasing to stabilise */
	msleep(VBIAS_SETTLING_TIME_MS);

	/* Re-configure the Accessory settings to detect Insertion/Removal
	  * as per current state
	  */
	data->pdata->xgold_configure_headset(val, data);

	/* Configure Hook interrupts accordingly */
	if (hook_enable)
		enable_irq(data->irq_hook);
	else
		disable_irq_nosync(data->irq_hook);

	/* Enable Accessory interrupts */
	enable_irq(data->irq_headset);

	wake_unlock(&data->acc_lock);
	dev_dbg(data->sdev.dev, "Leaving %s at %d\n", __func__ , __LINE__);
}

/**
 * Print state to the appropriate Switch Class file
**/
static ssize_t headset_print_state(struct switch_dev *sdev, char *buf)
{
	dev_dbg(sdev->dev, "Entering %s at %d\n", __func__ , __LINE__);

	if ((sdev->state == XGOLD_HEAD_SET) ||
		(sdev->state == XGOLD_HEAD_PHONE))
		return sprintf(buf, "%d\n", 1);
	else
		return sprintf(buf, "%d\n", 0);

	dev_dbg(sdev->dev, "Leaving %s at %d\n", __func__ , __LINE__);
}

static int accessory_probe(struct platform_device *pdev)
{
	struct xgold_headset_device_data *headset_device_data;
	struct gpio_switch_platform_data *sw_pdata;
	struct resource *res;
	struct input_dev *input_dev;
	int ret = 0;
	struct xgold_headset_platform_data *pdata = &headset_platform_data;
	pdev->dev.platform_data = pdata;

	pr_devel("Accessory: Entering probe\n");

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("Accessory: failed to allocate the input device\n");
		return -ENOMEM;
	}

	headset_device_data = kzalloc(
				sizeof(struct xgold_headset_device_data),
				GFP_KERNEL);
	if (!headset_device_data) {
		pr_err("Accessory: Error headset_device_data = NULL\n");
		ret = -ENOMEM;
		goto failed_kzalloc_1;
	}

	sw_pdata = kzalloc(
				sizeof(struct gpio_switch_platform_data),
				GFP_KERNEL);
	if (!sw_pdata) {
		dev_err(headset_device_data->sdev.dev,
				"Error sw_pdata = NULL\n");
		ret = -ENOMEM;
		goto failed_kzalloc_2;
	}

	pdata->xgold_headset_keymap(headset_device_data);

	sw_pdata->name = "h2w";
	headset_device_data->pdata = pdata;
	headset_device_data->headset_data = sw_pdata;
	headset_device_data->sdev.name = sw_pdata->name;
	headset_device_data->sdev.print_state = headset_print_state;
	headset_device_data->input_dev = input_dev;

	input_dev->name = pdev->name;
	input_dev->id.bustype = BUS_VIRTUAL;
	input_dev->dev.parent = &pdev->dev;
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	__set_bit(KEY_FORWARDMAIL , input_dev->keybit);
	__set_bit(KEY_VOLUMEUP, input_dev->keybit);
	__set_bit(KEY_VOLUMEDOWN, input_dev->keybit);

	input_set_capability(input_dev, EV_MSC, MSC_SCAN);
	input_set_drvdata(input_dev, headset_device_data);

	INIT_DELAYED_WORK(&(headset_device_data->work), headset_switch_work);
	INIT_DELAYED_WORK(&(headset_device_data->hook_work), hook_input_work);

	dev_dbg(headset_device_data->sdev.dev,
			"Registering %s\n",
			headset_device_data->sdev.name);

	ret = switch_dev_register(&headset_device_data->sdev);
	if (ret < 0) {
		dev_err(headset_device_data->sdev.dev,
				"Unable to register Switch device\n");
		goto failed_switch_register;
	}

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(headset_device_data->sdev.dev,
				"failed to register input device\n");
		goto failed_input_register;
	}

	res = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "headset-registers");
	if (res == NULL) {
		dev_err(headset_device_data->sdev.dev,
				"no I/O memory defined in platform data\n");
		ret = -EINVAL;
		goto failed_platform_resource;
	}

	dev_dbg(headset_device_data->sdev.dev,
			"res->start %x\n", res->start);

	headset_device_data->mmio_base =
			ioremap(res->start, resource_size(res));

	dev_dbg(headset_device_data->sdev.dev,
			" ioremap %x\n",
		  (u32)headset_device_data->mmio_base);

	if (headset_device_data->mmio_base == NULL) {
		dev_err(headset_device_data->sdev.dev,
				"failed to remap I/O memory\n");
		ret = -ENOMEM;
		goto failed_ioremap;
	}

	/* Configure the Accessory settings to detect Insertion */
	pdata->xgold_configure_headset(
						XGOLD_DET_HEADSET_INS,
						headset_device_data);

	headset_device_data->irq_headset =
			platform_get_irq_byname(pdev, "acd1");
	if (headset_device_data->irq_headset < 0) {
		dev_err(&pdev->dev, "no irq_headset irq defined\n");
		return -EINVAL;
	}
	headset_irq.dev_id = headset_device_data;
	ret = setup_irq(headset_device_data->irq_headset, &headset_irq);
	if (ret)
		dev_err(&pdev->dev, "setup of irq headset failed!\n");


	headset_device_data->irq_hook =
			platform_get_irq_byname(pdev, "acd2");
	if (headset_device_data->irq_hook < 0) {
		dev_err(&pdev->dev, "no irq_hook irq defined\n");
		return -EINVAL;
	}

	hook_irq.dev_id = headset_device_data;
	ret = setup_irq(headset_device_data->irq_hook, &hook_irq);
	if (ret)
		dev_err(&pdev->dev, "setup of irq hook failed!\n");

	if (ret) {
		dev_err(headset_device_data->sdev.dev,
				"IRQ registration failed\n");
		goto failed_pmu_irq;
	}
	headset_device_data->iio_client = iio_channel_get(&pdev->dev,
							"ACCID_ADC");
	if (IS_ERR(headset_device_data->iio_client)) {
		dev_err(headset_device_data->sdev.dev,
				"CHP iio channel error\n");
		goto failed_pmu_irq;
	}

	wake_lock_init(&headset_device_data->acc_lock,
				WAKE_LOCK_SUSPEND, "acc_det");
	platform_set_drvdata(pdev, headset_device_data);

	dev_dbg(headset_device_data->sdev.dev,
			"Leaving %s at %d\n", __func__ , __LINE__);

	return 0;

failed_pmu_irq:
failed_ioremap:
failed_platform_resource:
	input_unregister_device(input_dev);
failed_input_register:
	switch_dev_unregister(&headset_device_data->sdev);
failed_switch_register:
	kfree(sw_pdata);
failed_kzalloc_2:
	kfree(headset_device_data);
failed_kzalloc_1:
	input_free_device(input_dev);

	return ret;
}

static int accessory_remove(struct platform_device *pdev)
{
	struct xgold_headset_device_data *headset_device_data;

	headset_device_data = platform_get_drvdata(pdev);

	if (!headset_device_data) {
		pr_err("Accessory: Error headset_device_data = NULL\n");
		return -EBUSY;
	}

	cancel_delayed_work_sync(&headset_device_data->work);
	cancel_delayed_work_sync(&headset_device_data->hook_work);
	input_unregister_device(headset_device_data->input_dev);
	switch_dev_unregister(&headset_device_data->sdev);
	input_free_device(headset_device_data->input_dev);
	iio_channel_release(headset_device_data->iio_client);
	kfree(headset_device_data->headset_data);
	kfree(headset_device_data);

	return 0;
}

static struct of_device_id xgold_headset_of_match[] = {
	{.compatible = "intel,headset",},
	{},
};

static struct platform_driver accessory_driver = {
	.probe  = accessory_probe,
	.remove	= accessory_remove,
	.driver = {
		.name   = XGOLD_HEADSET_MODULE_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = xgold_headset_of_match,
	},
};
MODULE_DEVICE_TABLE(of, xgold_headset_of_match);

static int __init headset_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&accessory_driver);

	return ret;
}

static void __exit headset_exit(void)
{
	platform_driver_unregister(&accessory_driver);
	return;
}

late_initcall(headset_init);
module_exit(headset_exit);
