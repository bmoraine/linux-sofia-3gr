/*
* Copyright (C) 2014 Intel Mobile Communications GmbH
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <../drivers/staging/android/timed_output.h>
#include <linux/hrtimer.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <sofia/mv_svc_hypercalls.h>
#include "xgold-vibra.h"

#define XGOLD_VIBRA_MODULE_NAME		"vibra-xgold"

#define XGOLD_VIBRA_USE_SECURE_IO_ACCESS BIT(0)
#define XGOLD_VIBRA_USE_NATIVE_IO_ACCESS BIT(1)

/* Macros for power enable, disable */
#define XG_PM_DISABLE		0x0
#define XG_PM_ENABLE		0x1
#define XG_PM_NOF_STATES	0x2

#define MS_TO_NS(value) ktime_set(value/1000, (value % 1000)*1000000)

struct xgold_vibra_device {
	struct platform_device *pdev;
	void __iomem *mmio_base;
	phys_addr_t phys_io_addr;
	struct timed_output_dev vibrator;
	struct work_struct work;
	struct hrtimer timer;
	int vibra_on;
	spinlock_t lock;
	unsigned long flags;
};

static struct device_state_pm_state *vibra_pm_states[XG_PM_NOF_STATES];

static void vibra_write32(struct xgold_vibra_device *vib, u16 offset, u32 val)
{
	int ret;

	if (vib->flags & XGOLD_VIBRA_USE_SECURE_IO_ACCESS) {
		ret = mv_svc_reg_write(vib->phys_io_addr + offset,
							val, 0xFFFFFFFF);
		BUG_ON(ret);
	} else {
		iowrite32(val, vib->mmio_base + offset);
	}
}

static int vibra_read32(struct xgold_vibra_device *vib, u16 offset)
{
	int val, ret;

	if (vib->flags & XGOLD_VIBRA_USE_SECURE_IO_ACCESS) {
		ret = mv_svc_reg_read(vib->phys_io_addr + offset,
						&val, 0xFFFFFFFF);
		BUG_ON(ret);
	} else {
		val = ioread32(vib->mmio_base + offset);
	}
	return val;
}

static int xgold_set_clk_vibrator(struct device *dev,
					bool on)
{
	int ret;
	if (on) {
		ret = device_state_pm_set_state(dev,
				vibra_pm_states[XG_PM_ENABLE]);
		if (ret) {
			dev_err(dev, "%s Power On Failed\n", __func__);
			return ret;
		}
	} else {
		ret = device_state_pm_set_state(dev,
				vibra_pm_states[XG_PM_DISABLE]);
		if (ret) {
			dev_err(dev, "%s Power Off Failed\n", __func__);
			return ret;
		}
	}
	return 0;
}

static int xgold_set_vibrator(struct xgold_vibra_device *vib)
{
	int reg_val;
	unsigned long flags;
	spinlock_t *lock = &vib->lock;
	int intensity = vib->vibra_on;

	if (intensity) {
		reg_val = (VIBRA_UP) |
				(((((u32)intensity)*127)/100) & 0x7F);
		spin_lock_irqsave(lock, flags);
		vibra_write32(vib, VIBRA_CONTROL, VIBRA_DOWN);
		vibra_read32(vib, VIBRA_CONTROL);
		vibra_write32(vib, VIBRA_CONTROL, reg_val);
		vibra_read32(vib, VIBRA_CONTROL);
		spin_unlock_irqrestore(lock, flags);
	} else {
		spin_lock_irqsave(lock, flags);
		vibra_write32(vib, VIBRA_CONTROL, VIBRA_DOWN);
		vibra_read32(vib, VIBRA_CONTROL);
		spin_unlock_irqrestore(lock, flags);
	}

	return 0;
}

static int xgold_init_vibrator(struct device *dev)
{
	int ret;
	ret = device_state_pm_set_class(dev, "vib");
	vibra_pm_states[XG_PM_DISABLE] =
		device_state_pm_get_state_handler(dev, "disable");
	vibra_pm_states[XG_PM_ENABLE] =
		device_state_pm_get_state_handler(dev, "enable");
	if ((vibra_pm_states[XG_PM_DISABLE] == NULL) ||
		(vibra_pm_states[XG_PM_ENABLE] == NULL))
		ret = -1;
	return ret;
}

static void xgold_vibra_work(struct work_struct *work)
{
	struct xgold_vibra_device *vib =
			container_of(work, struct xgold_vibra_device, work);
	static int power_on; /* false at init as static */
	int ret = 0;

	if (vib->vibra_on) {
		if (power_on == false) {
			ret = xgold_set_clk_vibrator(vib->vibrator.dev, true);
			if (ret) {
				dev_err(vib->vibrator.dev,
				"%s Set Clk Failed\n", __func__);
				return;
			}
			power_on = true;
		}

		ret = xgold_set_vibrator(vib);
		if (ret) {
			dev_err(vib->vibrator.dev,
			"%s Set vibrator Failed\n", __func__);
			return;
		}
	} else {
		ret = xgold_set_vibrator(vib);
		if (ret) {
			dev_err(vib->vibrator.dev,
			"%s Set vibrator Failed\n", __func__);
			return;
		}
		if (power_on == true) {
			ret = xgold_set_clk_vibrator(vib->vibrator.dev, false);
			if (ret) {
				dev_err(vib->vibrator.dev,
				"%s Set Clk Failed\n", __func__);
				return;
			}
			power_on = false;
		}
	}
}

static void xgold_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct xgold_vibra_device *vib =
		container_of(dev, struct xgold_vibra_device, vibrator);
	unsigned long flags;

	if (value != 0 && value < 50)
		value += 50;
retry:
	spin_lock_irqsave(&vib->lock, flags);
	if (hrtimer_try_to_cancel(&vib->timer) < 0) {
		spin_unlock_irqrestore(&vib->lock, flags);
		goto retry;
	}

	if (value) {
		vib->vibra_on = 100;
		hrtimer_start(&vib->timer, MS_TO_NS(value), HRTIMER_MODE_REL);
	} else {
		vib->vibra_on = 0;
	}
	spin_unlock_irqrestore(&vib->lock, flags);
	schedule_work(&vib->work);
}

static enum hrtimer_restart xgold_hrtimer_callback(struct hrtimer *timer)
{
	struct xgold_vibra_device *vib =
		container_of(timer, struct xgold_vibra_device, timer);
	vib->vibra_on = 0;
	schedule_work(&vib->work);
	return HRTIMER_NORESTART;
}

static int xgold_vibrator_get_time(struct timed_output_dev *dev)
{
	return 0;
}

static int xgold_vibra_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct xgold_vibra_device *vib;
	struct device_node *np = pdev->dev.of_node;

	res = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "vibra-registers");
	if (res == NULL) {
		dev_err(&pdev->dev,
			"no I/O memory defined in platform data\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "%pR\n", res);

	vib = kzalloc(sizeof(struct xgold_vibra_device), GFP_KERNEL);
	if (!vib) {
		dev_err(&pdev->dev,
			"not enough memory for driver data\n");
		return -ENOMEM;
	}

	if (of_find_property(np, "intel,vmm-secured-access", NULL)) {
		dev_info(&pdev->dev, "AGold vibrator using secure access\n");
		vib->flags |= XGOLD_VIBRA_USE_SECURE_IO_ACCESS;
	} else {
		dev_info(&pdev->dev, "AGold vibrator using native access\n");
		vib->flags |= XGOLD_VIBRA_USE_NATIVE_IO_ACCESS;
	}

	vib->mmio_base = ioremap(res->start, resource_size(res));
	if (vib->mmio_base == NULL) {
		dev_err(&pdev->dev,
			" I/O remap failed\n");
		ret = -EINVAL;
		goto failed_free_mem;
	}

	vib->phys_io_addr = res->start;
	vib->pdev = pdev;
	INIT_WORK(&vib->work, xgold_vibra_work);
	spin_lock_init(&vib->lock);

	vib->vibrator.name = "vibrator";
	vib->vibrator.enable = xgold_vibrator_enable;
	vib->vibrator.get_time = xgold_vibrator_get_time;

	hrtimer_init(&vib->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->timer.function = xgold_hrtimer_callback;
	ret = timed_output_dev_register(&vib->vibrator);
	if (ret != 0) {
		dev_err(&pdev->dev,
			" unable to register with timed output\n");
		ret = -EINVAL;
		goto failed_unmap;
	}

	ret = xgold_init_vibrator(vib->vibrator.dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Plat Init Failed: %s\n",
						vib->vibrator.name);
		ret = -EINVAL;
		goto failed_unregister_timed_output;
	}

	platform_set_drvdata(pdev, vib);
	dev_err(&pdev->dev, "Vibrator driver probed\n");
	return 0;

failed_unregister_timed_output:
	timed_output_dev_unregister(&vib->vibrator);
failed_unmap:
	iounmap(vib->mmio_base);
failed_free_mem:
	kfree(vib);

	return ret;
}

static int xgold_vibra_remove(struct platform_device *pdev)
{
	struct xgold_vibra_device *vib = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	cancel_work_sync(&vib->work);
	timed_output_dev_unregister(&vib->vibrator);
	iounmap(vib->mmio_base);
	kfree(vib);
	return 0;
}

static struct of_device_id xgold_vibra_of_match[] = {
	{.compatible = "intel,vibra",},
	{},
};

static struct platform_driver xgold_vibra_driver = {
	.driver		= {
		.name	= XGOLD_VIBRA_MODULE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = xgold_vibra_of_match,
	},
	.probe		= xgold_vibra_probe,
	.remove		= xgold_vibra_remove,
};
MODULE_DEVICE_TABLE(of, xgold_vibra_of_match);

static int __init xgold_vibra_init(void)
{
	pr_info("xgold Vibrator init\n");
	return platform_driver_register(&xgold_vibra_driver);
}

static void __exit xgold_vibra_exit(void)
{
	pr_info("xgold Vibrator exit\n");
	platform_driver_unregister(&xgold_vibra_driver);
}

module_init(xgold_vibra_init);
module_exit(xgold_vibra_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("vibra driver for xgold");
