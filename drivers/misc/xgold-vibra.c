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
#include "xgold-vibra.h"

#define XGOLD_VIBRA_MODULE_NAME		"vibra-xgold"

/* Macros for power enable, disable */
#define XG_PM_DISABLE		0x0
#define XG_PM_ENABLE		0x1
#define XG_PM_NOF_STATES	0x2

#define MS_TO_NS(value) ktime_set(value/1000, (value % 1000)*1000000)

static struct device_state_pm_state *vibra_pm_states[XG_PM_NOF_STATES];

static void vibra_write32(void __iomem *mmio_base, u16 offset, u32 val)
{
	iowrite32(val, mmio_base + offset);
}

static int vibra_read32(void __iomem *mmio_base, u16 offset)
{
	int val = ioread32(mmio_base + offset);
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

static int xgold_set_vibrator(struct device *dev,
				void *mmio_base, spinlock_t *lock,
				int intensity)
{
	int reg_val;
	unsigned long flags;

	if (intensity) {
		reg_val = (VIBRA_UP) |
				(((((u32)intensity)*127)/100) & 0x7F);
		spin_lock_irqsave(lock, flags);
		vibra_write32(mmio_base, VIBRA_CONTROL, VIBRA_DOWN);
		vibra_read32(mmio_base, VIBRA_CONTROL);
		vibra_write32(mmio_base, VIBRA_CONTROL, reg_val);
		vibra_read32(mmio_base, VIBRA_CONTROL);
		spin_unlock_irqrestore(lock, flags);
	} else {
		spin_lock_irqsave(lock, flags);
		vibra_write32(mmio_base, VIBRA_CONTROL, VIBRA_DOWN);
		vibra_read32(mmio_base, VIBRA_CONTROL);
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

static void xgold_exit_vibrator(void)
{
	return;
}

static struct xgold_vibra_platform_data xgold_vibra_platform_data = {
	.init = xgold_init_vibrator,
	.exit = xgold_exit_vibrator,
	.set_clk = xgold_set_clk_vibrator,
	.set_vibrator = xgold_set_vibrator,
};

struct xgold_vibra_device {
	const struct xgold_vibra_platform_data *pdata;
	struct platform_device *pdev;
	void __iomem *mmio_base;
	struct timed_output_dev vibrator;
	struct work_struct work;
	struct hrtimer timer;
	int vibra_on;
	spinlock_t lock;
};

static void xgold_vibra_work(struct work_struct *work)
{
	struct xgold_vibra_device *vib =
			container_of(work, struct xgold_vibra_device, work);
	static int power_on; /* false at init as static */
	int ret = 0;

	if (vib->vibra_on) {
		if (power_on == false) {
			if (vib->pdata->set_clk) {
				ret = vib->pdata->set_clk(vib->vibrator.dev,
							true);
				if (ret) {
					dev_err(vib->vibrator.dev,
					"%s Set Clk Failed\n", __func__);
					return;
				}
			}
			power_on = true;
		}
		if (vib->pdata->set_vibrator) {
			ret = vib->pdata->set_vibrator(vib->vibrator.dev,
							vib->mmio_base,
							&vib->lock,
							vib->vibra_on);
			if (ret) {
				dev_err(vib->vibrator.dev,
				"%s Set vibrator Failed\n", __func__);
				return;
			}
		}
	} else {
		if (vib->pdata->set_vibrator) {
			ret = vib->pdata->set_vibrator(vib->vibrator.dev,
							vib->mmio_base,
							&vib->lock,
							vib->vibra_on);
			if (ret) {
				dev_err(vib->vibrator.dev,
				"%s Set vibrator Failed\n", __func__);
				return;
			}
		}
		if (power_on == true) {
			if (vib->pdata->set_clk) {
				ret = vib->pdata->set_clk(vib->vibrator.dev,
							false);
				if (ret) {
					dev_err(vib->vibrator.dev,
					"%s Set Clk Failed\n", __func__);
					return;
				}
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

	if (value < 50)
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
	struct xgold_vibra_platform_data *pdata = &xgold_vibra_platform_data;
	pdev->dev.platform_data = pdata;

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data\n");
		return -EINVAL;
	}

	res = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "vibra-registers");
	if (res == NULL) {
		dev_err(&pdev->dev,
			"no I/O memory defined in platform data\n");
		return -EINVAL;
	}
	dev_info(&pdev->dev, "res-start %#x\n", res->start);

	vib = kzalloc(sizeof(struct xgold_vibra_device), GFP_KERNEL);
	if (!vib) {
		dev_err(&pdev->dev,
			"not enough memory for driver data\n");
		return -ENOMEM;
	}

	vib->mmio_base = ioremap(res->start, resource_size(res));
	if (vib->mmio_base == NULL) {
		dev_err(&pdev->dev,
			" I/O remap failed\n");
		ret = -EINVAL;
		goto failed_free_mem;
	}
	vib->pdata = pdata;
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

	if (vib->pdata->init) {
		ret = vib->pdata->init(vib->vibrator.dev);
		if (ret < 0) {
			dev_err(&pdev->dev, "Plat Init Failed: %s\n",
					vib->vibrator.name);
			ret = -EINVAL;
			goto failed_unregister_timed_output;
		}
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
