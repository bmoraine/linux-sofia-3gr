/*
 * -------------------------------------------------------------------------
 *  Copyright (C) 2016 Intel Corporation
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
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

#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>

#define PROP_DUMMY_CHARGER "intel,dummy_charger"
#define SYSFS_INPUT_VAL_LEN (1)

#define CHARGER_CONTROL_O 0x0
#define CHARGER_CONTROL(_base) ((_base) + CHARGER_CONTROL_O)
#define CHARGER_CONTROL_CHGWSRC_O 7
#define CHARGER_CONTROL_CHGWSRC_M 0x2
#define CHARGER_CONTROL_CHGCIEN_O 23
#define CHARGER_CONTROL_CHGCIEN_M 0x1

#define CHARGER_CONTROL_WR_O 0x8
#define CHARGER_CONTROL_WR(_base) ((_base) + CHARGER_CONTROL_WR_O)
#define CHARGER_CONTROL_WR_WS_O 0
#define CHARGER_CONTROL_WR_WS_M 0x1

struct idi_device {
	struct idi_peripheral_device *ididev;
	struct resource *ctrl_io_res;
};

static struct idi_device idi_dummy_chrg;

struct dummy_charger_device {
	int irq;
	int irq_flags;
	int active_low;
	int det_gpio;
	bool irq_wakeup;
	int fake_vbus;
	int pwr_en_pin;
	int vbus_err_pin;
	int enable_boost;

	struct idi_device *idi;
	struct semaphore prop_lock;
	struct usb_phy *otg_handle;
	struct delayed_work work;
	struct notifier_block otg_nb;

	struct platform_device *pdev;
};

static ssize_t dummy_charger_fake_vbus_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dummy_charger_device *data = dev_get_drvdata(dev);
	size_t size_copied;
	int value;

	value = data->fake_vbus;
	size_copied = sprintf(buf, "%d\n", value);

	return size_copied;
}

static int dummy_charger_configure_pmu_regs(struct dummy_charger_device *dummy_chrg_dev)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	int ret;

	if (!dummy_chrg_dev->idi->ididev) {
		pr_err("%s: Invalid ididev\n", __func__);
		return -EINVAL;
	}

	pm_state_en =
		idi_peripheral_device_pm_get_state_handler(
                        dummy_chrg_dev->idi->ididev, "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return -EINVAL;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
				dummy_chrg_dev->idi->ididev, "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return -EINVAL;
	}

	pr_debug("%s: Getting PM state handlers: OK\n", __func__);

	ret = idi_set_power_state(dummy_chrg_dev->idi->ididev, pm_state_en, true);

	if (ret) {
		pr_err("Setting PM state '%s' failed!\n", pm_state_en->name);
		return -EIO;
	}

	ret |= idi_client_ioread(dummy_chrg_dev->idi->ididev,
			CHARGER_CONTROL(dummy_chrg_dev->idi->ctrl_io_res->start), &regval);

	/* CHGWSRC - Turn of CHGINT for ChargerDetecSource */
	regval &= ~(CHARGER_CONTROL_CHGWSRC_M << CHARGER_CONTROL_CHGWSRC_O);

	/* CIEN - Turn off chargerirq enable */
	regval &= ~(CHARGER_CONTROL_CHGCIEN_M << CHARGER_CONTROL_CHGCIEN_O);

	ret |= idi_client_iowrite(dummy_chrg_dev->idi->ididev,
			CHARGER_CONTROL(dummy_chrg_dev->idi->ctrl_io_res->start), regval);

	/* charger control WR strobe */
	ret |= idi_client_iowrite(dummy_chrg_dev->idi->ididev,
			CHARGER_CONTROL_WR(dummy_chrg_dev->idi->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

	/* Workaround the PMU not loaded the CHARGER_CONTROL with single write */
	ret |= idi_client_iowrite(dummy_chrg_dev->idi->ididev,
			CHARGER_CONTROL_WR(dummy_chrg_dev->idi->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

	if (ret) {
		pr_err("%s: IDI access failure!\n", __func__);
		return -EIO;
	}

	ret = idi_set_power_state(dummy_chrg_dev->idi->ididev, pm_state_dis, false);

	if (ret) {
		pr_err("Setting PM state '%s', failed!\n", pm_state_dis->name);
		return -EIO;
	}

	return ret;

}

static ssize_t dummy_charger_fake_vbus_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct dummy_charger_device *dummy_chrg_device = dev_get_drvdata(dev);
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

	down(&dummy_chrg_device->prop_lock);
	dummy_chrg_device->fake_vbus = -1;

	if (!sysfs_val) {
		dummy_chrg_device->fake_vbus = 0;
		atomic_notifier_call_chain(&dummy_chrg_device->otg_handle->notifier,
				USB_EVENT_VBUS, &dummy_chrg_device->fake_vbus);
		pr_debug("%s:fake vbus removal sent\n", __func__);

	} else {
		dummy_chrg_device->fake_vbus = 1;
		atomic_notifier_call_chain(&dummy_chrg_device->otg_handle->notifier,
				USB_EVENT_VBUS, &dummy_chrg_device->fake_vbus);
		pr_debug("%s:fake vbus connection sent\n", __func__);
	}

	up(&dummy_chrg_device->prop_lock);

	return count;
}

static struct device_attribute dummy_charger_fake_vbus_attr = {
	.attr = {
		.name = "fake_vbus_event",
		.mode = S_IRUSR | S_IWUSR,
	},
	.show = dummy_charger_fake_vbus_show,
	.store = dummy_charger_fake_vbus_store,
};

static void dummy_charger_setup_fake_vbus_sysfs_attr(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	if (device_create_file(dev, &dummy_charger_fake_vbus_attr))
		dev_err(dev, "Unable to create sysfs entry: '%s'\n",
				dummy_charger_fake_vbus_attr.attr.name);
}

static void dummy_chrg_work(struct work_struct *work)
{
	int state;
	struct dummy_charger_device	*dummy_chrg_device =
		container_of(to_delayed_work(work), struct dummy_charger_device,
				work);

	state = gpio_get_value(dummy_chrg_device->det_gpio);
	dummy_chrg_device->fake_vbus = -1;

	down(&dummy_chrg_device->prop_lock);
	if (!state) {  /* det_gpio=low, if active_low=1, make connection else disconnect */
		/* dummy_chrg_device->fake_vbus = 0; */
		dummy_chrg_device->fake_vbus = dummy_chrg_device->active_low;
		atomic_notifier_call_chain(&dummy_chrg_device->otg_handle->notifier,
				USB_EVENT_VBUS, &dummy_chrg_device->fake_vbus);
		if (dummy_chrg_device->active_low)
			pr_info("%s: Vbus connection sent\n", __func__);
		else
			pr_info("%s: Vbus removal sent\n", __func__);
	}
	else { /* det_gpio=high, if active_low=1, make disconnection else connection */
		/* dummy_chrg_device->fake_vbus = 1; */
		dummy_chrg_device->fake_vbus = !dummy_chrg_device->active_low;
		atomic_notifier_call_chain(&dummy_chrg_device->otg_handle->notifier,
				USB_EVENT_VBUS, &dummy_chrg_device->fake_vbus);

		if (!dummy_chrg_device->active_low)
			pr_info("%s: Vbus connection sent\n", __func__);
		else
			pr_info("%s: Vbus removal sent\n", __func__);
	}
	up(&dummy_chrg_device->prop_lock);
}

int dummy_chrg_notification_handler(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct dummy_charger_device *dummy_chrg_dev =
		container_of(nb, struct dummy_charger_device, otg_nb);

	pr_debug("%s: event=%lu\n", __func__, event);

	switch (event) {
	case INTEL_USB_DRV_VBUS:
		{
			dummy_chrg_dev->enable_boost = *((bool *)data);
			pr_debug("%s: pwren-gpio: %d, enable_bootst: %d\n", __func__,
					dummy_chrg_dev->pwr_en_pin,dummy_chrg_dev->enable_boost);

			if (dummy_chrg_dev->enable_boost) {
				gpio_direction_output(dummy_chrg_dev->pwr_en_pin, 1);
			} else {
				gpio_direction_output(dummy_chrg_dev->pwr_en_pin, 0);
			}
		}
		break;
	default:
		pr_debug("%s: Unhandle USB phy event\n", __func__);
		break;
	}

	return NOTIFY_OK;
}

static irqreturn_t dummy_chrg_irq_handler(int irq, void *dev_id)
{
	struct dummy_charger_device *data = dev_id;

	queue_delayed_work(system_power_efficient_wq, &data->work, 0);

	return IRQ_HANDLED;
}

static int dummy_charger_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct dummy_charger_device *dummy_chrg_dev;
	struct usb_phy *otg_handle;
	int ret=0, val=0;

	dummy_chrg_dev = kzalloc(sizeof(struct dummy_charger_device), GFP_KERNEL);
	if (!dummy_chrg_dev) {
		dev_err(&pdev->dev,
				"not enough memory for driver data\n");
		return -ENOMEM;
	}

	if (!of_property_read_u32(np, "active_low", &val)) {
		if (val)
			dummy_chrg_dev->active_low = 1;
		else
			dummy_chrg_dev->active_low = 0;
	} else
		dummy_chrg_dev->active_low = 0;

	pr_info("%s: active-low:%d\n", __func__, dummy_chrg_dev->active_low);

	dummy_chrg_dev->det_gpio = of_get_named_gpio(np, "detect-gpio", 0);
	pr_info("%s: detect-gpio: %d\n", __func__, dummy_chrg_dev->det_gpio);

	dummy_chrg_dev->irq_wakeup = 1; // device_property_read_bool(dev, "wakeup-source");
	dummy_chrg_dev->irq_flags = (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND);

	dummy_chrg_dev->irq = irq_of_parse_and_map(np, 0);
	pr_info("%s: data->irq:%d\n", __func__, dummy_chrg_dev->irq);

	if (!dummy_chrg_dev->irq) {
		pr_err("%s: can't get detach irq\n", __func__);
	}

	sema_init(&dummy_chrg_dev->prop_lock, 1);

	dummy_chrg_dev->idi = &idi_dummy_chrg;

	/* Setup the PMU registers. The charger control by default enable charger
	 * Disable the charger
	 */
	ret = dummy_charger_configure_pmu_regs(dummy_chrg_dev);
	if (ret != 0) {
		pr_err("%s: ERROR: Configure PMU registers failed\n", __func__);
	}

	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg_handle == NULL) {
		pr_err("%s: ERROR!: getting OTG transceiver failed\n", __func__);
		return -ENODEV;
	}
	dummy_chrg_dev->otg_handle = otg_handle;

	INIT_DELAYED_WORK(&dummy_chrg_dev->work, dummy_chrg_work);

	dummy_chrg_dev->pdev = pdev;
	platform_set_drvdata(pdev, dummy_chrg_dev);

	dummy_chrg_dev->otg_nb.notifier_call = dummy_chrg_notification_handler;

	dummy_chrg_dev->pwr_en_pin = of_get_named_gpio(np, "pwren-gpio", 0);
	if (!gpio_is_valid(dummy_chrg_dev->pwr_en_pin)) {
		pr_err("%s: invalid pwren gpio\n", __func__);
	}

	usb_register_notifier(dummy_chrg_dev->otg_handle, &dummy_chrg_dev->otg_nb);

	if (dummy_chrg_dev->irq) {
		ret = request_irq(dummy_chrg_dev->irq, dummy_chrg_irq_handler,
				dummy_chrg_dev->irq_flags, pdev->name, dummy_chrg_dev);
		if (ret != 0) {
			pr_err("%s: Failed to register irq, ret=%d\n", __func__, ret);
			return ret;
		}
	}

	dummy_charger_setup_fake_vbus_sysfs_attr(pdev);

	/* Perform initial detection */
	dummy_chrg_work(&dummy_chrg_dev->work.work);

	return ret;
}

static int dummy_charger_remove(struct platform_device *pdev)
{
	struct dummy_charger_device *dummy_chrg_device = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&dummy_chrg_device->work);
	free_irq(dummy_chrg_device->irq, dummy_chrg_device);

	platform_set_drvdata(pdev, NULL);
	kfree(dummy_chrg_device);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dummy_charger_suspend(struct device *dev)
{
	struct dummy_charger_device *data = dev_get_drvdata(dev);

	if (data->irq_wakeup)
		enable_irq_wake(data->irq);

	return 0;
}

static int dummy_charger_resume(struct device *dev)
{
	struct dummy_charger_device *data = dev_get_drvdata(dev);

	if (data->irq_wakeup)
		disable_irq_wake(data->irq);

	return 0;
}
#endif

static void idi_init(struct idi_device *idi,
		struct resource *ctrl_io_res,
		struct idi_peripheral_device *ididev)
{
	idi->ctrl_io_res = ctrl_io_res;
	idi->ididev =ididev;
}

static int dummy_chrg_idi_probe(struct idi_peripheral_device *ididev,
		const struct idi_device_id *id)
{
	struct resource *res;
	int ret = 0;

	res = idi_get_resource_byname(&ididev->resources,
			IORESOURCE_MEM, "registers");
	if (res == NULL) {
		pr_err("getting PMU's Charger registers resources failed!\n");
		return -EINVAL;
	}

	idi_init(&idi_dummy_chrg, res, ididev);

	ret = idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
				__func__);
		return ret;
	}

	return ret;
}

static int __exit dummy_chrg_idi_remove(struct idi_peripheral_device *ididev)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(dummy_charger_pm_ops,
	dummy_charger_suspend, dummy_charger_resume);

static struct of_device_id dummy_charger_match_table[] = {
	{.compatible = "dummy-charger" },
	{},
};

static struct platform_driver dummy_charger_driver = {
	.probe  = dummy_charger_probe,
	.remove = dummy_charger_remove,
	.driver = {
		.name  = "DUMMY-CHARGER",
		.owner = THIS_MODULE,
		.pm    = &dummy_charger_pm_ops,
		.of_match_table = dummy_charger_match_table,
	},
};

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_CHG,
	},

	{ /* end: all zeroes */},
};

static struct idi_peripheral_driver dummy_chrg_idi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "dummy_chrg_idi",
		.pm    = NULL,
	},
	.p_type = IDI_CHG,
	.id_table = idi_ids,
	.probe  = dummy_chrg_idi_probe,
	.remove = dummy_chrg_idi_remove,
};

static int __init dummy_charger_init(void)
{
	int ret = 0;
	pr_info("Intel Dummy Charger Driver Init\n");

	ret = idi_register_peripheral_driver(&dummy_chrg_idi_driver);
	if (ret) {
		pr_err("%s:unable to register idi driver\n", __func__);
		return -ENODEV;
	}

	ret = platform_driver_register(&dummy_charger_driver);
	if (ret < 0) {
		pr_err("%s:unable to add dummy_charger driver\n", __func__);
		return -ENODEV;
	}

	return ret;
}

static void __exit dummy_charger_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&dummy_charger_driver);
}

late_initcall(dummy_charger_init);
module_exit(dummy_charger_exit);

MODULE_DESCRIPTION("Intel Dummy Charger Driver");
MODULE_LICENSE("GPL");
