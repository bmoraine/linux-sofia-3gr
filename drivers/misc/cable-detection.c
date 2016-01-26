/*
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>

#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/usb/phy-intel.h>

#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define PROP_CABLE_DETECTION "intel,cable_detection"
#define SYSFS_INPUT_VAL_LEN (1)

struct cable_detection_device {
	int irq;
	int irq_flags;
	int det_gpio;
	bool irq_wakeup;
	int fake_vbus;
	int pwr_en_pin;
	int vbus_err_pin;
	int enable_boost;
	struct semaphore prop_lock;
	struct usb_phy *otg_handle;
	struct delayed_work work;
	struct notifier_block otg_nb;
	struct platform_device *pdev;
};

static ssize_t cable_detection_fake_vbus_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cable_detection_device *data = dev_get_drvdata(dev);
	size_t size_copied;
	int value;

	value = data->fake_vbus;
	size_copied = sprintf(buf, "%d\n", value);

	return size_copied;
}

static ssize_t cable_detection_fake_vbus_store(struct device *dev,
				struct device_attribute *attr,const char *buf,
				size_t count)
{
	struct cable_detection_device *cab_det_device = dev_get_drvdata(dev);
	int sysfs_val;
	int ret;
	size_t size_to_cpy;
	char strvalue[SYSFS_INPUT_VAL_LEN + 1];

	size_to_cpy = (count > SYSFS_INPUT_VAL_LEN) ?
			SYSFS_INPUT_VAL_LEN : count;

	strncpy(strvalue, buf, size_to_cpy);
	strvalue[size_to_cpy] = '\0';

	ret = kstrtoint(strvalue, 10, &sysfs_val);
	if (ret != 0)
		return ret;
	sysfs_val = (sysfs_val == 0) ? 0 : 1;

	down(&cab_det_device->prop_lock);
	cab_det_device->fake_vbus = -1;
	if (sysfs_val == 0) {
		cab_det_device->fake_vbus = 0;
		atomic_notifier_call_chain(
				&cab_det_device->otg_handle->notifier,
				USB_EVENT_VBUS, &cab_det_device->fake_vbus);
		pr_info("%s:fake vbus removal sent\n", __func__);
	} else if (sysfs_val == 1) {
		cab_det_device->fake_vbus = 1;
		atomic_notifier_call_chain(
				&cab_det_device->otg_handle->notifier,
				USB_EVENT_VBUS, &cab_det_device->fake_vbus);
		pr_info("%s:fake vbus connection sent\n", __func__);
	}
	up(&cab_det_device->prop_lock);

	return count;
}

static struct device_attribute cable_detection_fake_vbus_attr = {
	.attr = {
		.name = "fake_vbus_event",
		.mode = S_IRUSR | S_IWUSR,
	},
	.show = cable_detection_fake_vbus_show,
	.store = cable_detection_fake_vbus_store,
};

static void cable_detection_setup_fake_vbus_sysfs_attr(
					struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	if (device_create_file(dev, &cable_detection_fake_vbus_attr))
		dev_err(dev, "Unable to create sysfs entry: '%s'\n",
	cable_detection_fake_vbus_attr.attr.name);
}

static void cable_detect_work(struct work_struct *work)
{
	int state;
	struct cable_detection_device *cab_det_device =
				container_of(to_delayed_work(work),
				struct cable_detection_device, work);

	state = gpio_get_value(cab_det_device->det_gpio);
	cab_det_device->fake_vbus = -1;

	down(&cab_det_device->prop_lock);
	if (!state) {  /* low active, disconnected */
		cab_det_device->fake_vbus = 0;
		atomic_notifier_call_chain(
				&cab_det_device->otg_handle->notifier,
				USB_EVENT_VBUS, &cab_det_device->fake_vbus);
		pr_info("%s:fake vbus connection sent\n", __func__);
	} else { /* connected */
		cab_det_device->fake_vbus = 1;
		atomic_notifier_call_chain(
				&cab_det_device->otg_handle->notifier,
				USB_EVENT_VBUS, &cab_det_device->fake_vbus);
		pr_info("%s:fake vbus removal sent\n", __func__);
	}
	up(&cab_det_device->prop_lock);
}

int cab_det_notification_handler(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct cable_detection_device *cab_det_dev =
		container_of(nb, struct cable_detection_device, otg_nb);

	printk ("%s: event=%d\n", __func__,event);
	switch (event) {
	case INTEL_USB_DRV_VBUS:
	{
		cab_det_dev->enable_boost = *((bool *)data);

		printk("%s: pwren-gpio: %d, enable_bootst\n", __func__,
			cab_det_dev->pwr_en_pin,cab_det_dev->enable_boost);

		if (cab_det_dev->enable_boost) {
			gpio_direction_output(cab_det_dev->pwr_en_pin, 1);
		} else {
			gpio_direction_output(cab_det_dev->pwr_en_pin, 0);
		}
		break;
	}
	default:
		break;
	}
	return NOTIFY_OK;
}


static irqreturn_t cable_detect_irq_handler(int irq, void *dev_id)
{
	struct cable_detection_device *data = dev_id;
	pr_info("%s\n", __func__);

	queue_delayed_work(system_power_efficient_wq, &data->work, 0);

	return IRQ_HANDLED;
}

static int cable_detection_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct cable_detection_device *cab_det_dev;
	struct usb_phy *otg_handle;
	int ret = 0;

	cab_det_dev = kzalloc(sizeof(struct cable_detection_device),
				GFP_KERNEL);
	if (!cab_det_dev) {
		dev_err(&pdev->dev, "not enough memory for driver data\n");
		return -ENOMEM;
	}

	cab_det_dev->det_gpio = of_get_named_gpio(np, "detect-gpio", 0);
	if (!gpio_is_valid(cab_det_dev->det_gpio)) {
		pr_err("%s: invalid detection gpio\n", __func__);
		return -EINVAL;
	}

	pr_info("%s: detect-gpio: %d\n", __func__, cab_det_dev->det_gpio);
	if (gpio_request(cab_det_dev->det_gpio, "detect-gpio")) {
		dev_err(dev, "%s: request gpio %d fail!\n", __func__,
				cab_det_dev->det_gpio);
		return -EINVAL;
	}

	cab_det_dev->irq_wakeup = 1;//device_property_read_bool(dev, "wakeup-source");
	cab_det_dev->irq_flags = (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
					IRQF_NO_SUSPEND);

	cab_det_dev->irq = irq_of_parse_and_map(np, 0);
	pr_info("%s: data->irq:%d\n", __func__, cab_det_dev->irq);
	if (!cab_det_dev->irq) {
		pr_err("%s: can't get detach irq\n", __func__);
	}

	sema_init(&cab_det_dev->prop_lock, 1);

	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg_handle == NULL) {
		pr_err("ERROR!: getting OTG transceiver failed\n");
		return -EINVAL;
	}
	cab_det_dev->otg_handle = otg_handle;

	INIT_DELAYED_WORK(&cab_det_dev->work, cable_detect_work);

	cab_det_dev->pdev = pdev;
	platform_set_drvdata(pdev, cab_det_dev);

	cab_det_dev->otg_nb.notifier_call = cab_det_notification_handler;

	cab_det_dev->pwr_en_pin = of_get_named_gpio(np, "pwren-gpio", 0);
	if (!gpio_is_valid(cab_det_dev->pwr_en_pin)) {
		pr_err("%s: invalid detection gpio\n", __func__);
	}

	if (gpio_request(cab_det_dev->pwr_en_pin, "pwren-gpio")) {
		pr_err("%s: request gpio %d failed!\n", __func__,
			cab_det_dev->pwr_en_pin);
	}
	ret = usb_register_notifier(cab_det_dev->otg_handle,
					&cab_det_dev->otg_nb);
	if (ret) {
		pr_err("ERROR!: registration for OTG notifications failed\n");
	}

	if (cab_det_dev->irq) {
		ret = request_irq(cab_det_dev->irq, cable_detect_irq_handler,
					cab_det_dev->irq_flags, pdev->name,
					cab_det_dev);
		if (ret != 0) {
			pr_err("%s: Failed to register irq, ret=%d\n",
				__func__, ret);
			return ret;
		}
	} else
		pr_err("%s: Failed to register irq, irq=0", __func__);

	cable_detection_setup_fake_vbus_sysfs_attr(pdev);

	/* Perform initial detection */
	cable_detect_work(&cab_det_dev->work.work);

	return ret;
}

static int cable_detection_remove(struct platform_device *pdev)
{
	struct cable_detection_device *cab_det_device =
					platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&cab_det_device->work);
	free_irq(cab_det_device->irq, cab_det_device);

	platform_set_drvdata(pdev, NULL);
	kfree(cab_det_device);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cable_detection_suspend(struct device *dev)
{
	struct cable_detection_device *data = dev_get_drvdata(dev);

	if (data->irq_wakeup)
		enable_irq_wake(data->irq);

	return 0;
}

static int cable_detection_resume(struct device *dev)
{
	struct cable_detection_device *data = dev_get_drvdata(dev);

	if (data->irq_wakeup)
		disable_irq_wake(data->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(cable_detection_pm_ops,
		cable_detection_suspend, cable_detection_resume);

static struct of_device_id cable_detection_match_table[] = {
	{ .compatible = "cable-detection" },
	{},
};

static struct platform_driver cable_detection_driver = {
	.probe  = cable_detection_probe,
	.remove = cable_detection_remove,
	.driver = {
		.name  = "CABLE-DETECT",
		.owner = THIS_MODULE,
		.pm    = &cable_detection_pm_ops,
		.of_match_table = cable_detection_match_table,
	},
};

static int __init cable_detection_init(void)
{
	int ret = 0;
	pr_debug("Intel cable_detection driver init\n");

	ret = platform_driver_register(&cable_detection_driver);
	if (ret < 0) {
		pr_err("%s:unable to add cable_detection driver\n", __func__);
		return -ENODEV;
	}

	return ret;
}

static void __exit cable_detection_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&cable_detection_driver);
}

late_initcall(cable_detection_init);
module_exit(cable_detection_exit);

MODULE_DESCRIPTION("Intel cable detection driver");
MODULE_LICENSE("GPL");
