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
	int active_low;
	int det_gpio;
	bool irq_wakeup;
	int fake_vbus;
	int pwr_en_pin;
	int vbus_err_pin;
	int enable_boost;
	/* for usb fault interrupt */
	int usb_fault_irq;
	int usb_fault_gpio;
	struct semaphore prop_lock;
	struct usb_phy *otg_handle;
	struct delayed_work work;
	struct notifier_block otg_nb;
	struct platform_device *pdev;
};

static ssize_t cable_detection_fake_vbus_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct cable_detection_device *data = dev_get_drvdata(dev);
	size_t size_copied;
	int value;

	value = data->fake_vbus;
	size_copied = sprintf(buf, "%d\n", value);

	return size_copied;
}

static ssize_t cable_detection_fake_vbus_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
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
	if (!sysfs_val) {
		cab_det_device->fake_vbus = 0;
		atomic_notifier_call_chain(&cab_det_device->otg_handle->
					   notifier, USB_EVENT_VBUS,
					   &cab_det_device->fake_vbus);
		pr_debug("%s:fake vbus removal sent\n", __func__);
	} else {
		cab_det_device->fake_vbus = 1;
		atomic_notifier_call_chain(&cab_det_device->otg_handle->
					   notifier, USB_EVENT_VBUS,
					   &cab_det_device->fake_vbus);
		pr_debug("%s:fake vbus connection sent\n", __func__);
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

static void cable_detection_setup_fake_vbus_sysfs_attr(struct platform_device
						       *pdev)
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
	if (!state) {
		/*
			det_gpio=low,
			if active_low=1, make connection else disconnect
			cab_det_device->fake_vbus = 0;
		*/
		cab_det_device->fake_vbus = cab_det_device->active_low;
		atomic_notifier_call_chain(&cab_det_device->otg_handle->
			notifier, USB_EVENT_VBUS,
			&cab_det_device->fake_vbus);
		if (cab_det_device->active_low)
			pr_debug("%s:fake vbus connection sent\n", __func__);
		else
			pr_debug("%s:fake vbus removal sent\n", __func__);
	} else {
		/*
			det_gpio=high,
			if active_low=1, make disconnection else connection
			cab_det_device->fake_vbus = 1;
		*/
		cab_det_device->fake_vbus = !cab_det_device->active_low;
		atomic_notifier_call_chain(&cab_det_device->otg_handle->
			notifier, USB_EVENT_VBUS,
			&cab_det_device->fake_vbus);

		if (!cab_det_device->active_low)
			pr_debug("%s:fake vbus connection sent\n", __func__);
		else
			pr_debug("%s:fake vbus removal sent\n", __func__);
	}
	up(&cab_det_device->prop_lock);
}

int cab_det_notification_handler(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct cable_detection_device *cab_det_dev =
	    container_of(nb, struct cable_detection_device, otg_nb);

	pr_debug("%s: event=%lu\n", __func__, event);
	switch (event) {
	case INTEL_USB_DRV_VBUS:
		{
			cab_det_dev->enable_boost = *((bool *) data);
			pr_debug("%s: pwren-gpio: %d, enable_bootst: %d\n",
				 __func__, cab_det_dev->pwr_en_pin,
				 cab_det_dev->enable_boost);

			if (cab_det_dev->enable_boost)
				gpio_direction_output(cab_det_dev->pwr_en_pin,
					1);
			else
				gpio_direction_output(cab_det_dev->pwr_en_pin,
					0);
			break;
		}
	default:
		pr_debug("%s: Unhandle USB phy event\n", __func__);
		break;
	}
	return NOTIFY_OK;
}

static irqreturn_t cable_detect_irq_handler(int irq, void *dev_id)
{
	struct cable_detection_device *data = dev_id;

	queue_delayed_work(system_power_efficient_wq, &data->work, 0);

	return IRQ_HANDLED;
}

static irqreturn_t usb_fault_irq_handler(int irq, void *dev_id)
{
	struct cable_detection_device *data = dev_id;
	int state;

	state = gpio_get_value(data->usb_fault_gpio);

	if (!state) {
		atomic_notifier_call_chain(&data->otg_handle->notifier,
			INTEL_USB_DRV_VBUS_ERR, 0);
		pr_debug("usb_fault irq\n");
		if (data->enable_boost)
			gpio_direction_output(data->pwr_en_pin, 0);
	} else {
		pr_debug("usb_fault irq ignore\n");
	}
	return IRQ_HANDLED;
}

static int cable_detection_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct cable_detection_device *cab_det_dev;
	struct usb_phy *otg_handle;
	int ret = 0, val = 0;

	cab_det_dev = kzalloc(sizeof(struct cable_detection_device),
		GFP_KERNEL);
	if (!cab_det_dev) {
		dev_err(&pdev->dev, "not enough memory for driver data\n");
		return -ENOMEM;
	}

	if (!of_property_read_u32(np, "active_low", &val)) {
		if (val)
			cab_det_dev->active_low = 1;
		else
			cab_det_dev->active_low = 0;
	} else
		cab_det_dev->active_low = 0;

	pr_debug("%s: active-low:%d\n", __func__, cab_det_dev->active_low);

	cab_det_dev->det_gpio = of_get_named_gpio(np, "detect-gpio", 0);
	pr_debug("%s: detect-gpio: %d\n", __func__, cab_det_dev->det_gpio);

	cab_det_dev->usb_fault_gpio = of_get_named_gpio(np, "usbfault-gpio", 0);
	pr_debug("%s: usb_fault_gpio: %d\n", __func__,
		 cab_det_dev->usb_fault_gpio);

	cab_det_dev->irq_wakeup = 1;
	cab_det_dev->irq_flags = (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				  IRQF_NO_SUSPEND);

	cab_det_dev->irq = platform_get_irq_byname(pdev, "cable_det");
	pr_debug("%s: data->irq:%d\n", __func__, cab_det_dev->irq);
	if (!cab_det_dev->irq) {
		pr_err("%s: can't get detach irq\n", __func__);
	}

	/* Register usb fault pin interrupt used to detect overcurrent */
	cab_det_dev->usb_fault_irq = platform_get_irq_byname(pdev, "usb_fault");
	pr_debug("%s: usb fault irq:%d\n", __func__,
		 cab_det_dev->usb_fault_irq);
	if (!cab_det_dev->usb_fault_irq)
		pr_err("%s: can't get usb_fault_irq\n", __func__);

	sema_init(&cab_det_dev->prop_lock, 1);

	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg_handle == NULL) {
		pr_err("ERROR!: getting OTG transceiver failed\n");
		ret = -EINVAL;
		goto err_kzalloc;
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

	usb_register_notifier(cab_det_dev->otg_handle, &cab_det_dev->otg_nb);

	if (cab_det_dev->irq) {
		ret = request_irq(cab_det_dev->irq, cable_detect_irq_handler,
				  cab_det_dev->irq_flags, pdev->name,
				  cab_det_dev);
		if (ret != 0) {
			pr_err("%s: Failed to register irq, ret=%d\n",
			       __func__, ret);
			goto err_kzalloc;
		}
	} else
		pr_err("%s: Failed to register irq, irq=0", __func__);

	if (cab_det_dev->usb_fault_irq) {
		ret =
		    request_irq(cab_det_dev->usb_fault_irq,
				usb_fault_irq_handler,
				(IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND),
				"usb_fault", cab_det_dev);
		if (ret != 0) {
			pr_err("%s: Failed to register usbfault irq, ret=%d\n",
			       __func__, ret);
			goto err_kzalloc;
		}
	}

	cable_detection_setup_fake_vbus_sysfs_attr(pdev);

	/* Perform initial detection */
	cable_detect_work(&cab_det_dev->work.work);

	return 0;

err_kzalloc:
	kfree(cab_det_dev);
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
	{.compatible = "cable-detection"},
	{},
};

static struct platform_driver cable_detection_driver = {
	.probe = cable_detection_probe,
	.remove = cable_detection_remove,
	.driver = {
		   .name = "CABLE-DETECT",
		   .owner = THIS_MODULE,
		   .pm = &cable_detection_pm_ops,
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
