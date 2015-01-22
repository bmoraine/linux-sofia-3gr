/*
* Copyright (C) 2015 Intel Mobile Communications GmbH
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
#include <sofia/vmm_pmic.h>

#define PMIC6832_VIBRA_MODULE_NAME		"vibra-pmic"

#define PMIC_DEV6_SLAVE_ADDR (0x6E)
#define PMIC_DEV6_ADDR (PMIC_DEV6_SLAVE_ADDR<<24)

#define VIBRA_PMIC_STATUS       0x00
#define VIBRA_PMIC_CONTROL      0x01
#define VIBRA_PMIC_PWM          0x02
#define VIBRA_PMIC_CFG          0x03
#define VIBRA_PMIC_OVC          0x04

#define VIBRA_DOWN              0x00
#define VIBRA_UP                0x01

#define MS_TO_NS(value) ktime_set(value/1000, (value % 1000)*1000000)

struct pmic6832_vibra_device {
	struct platform_device *pdev;
	struct timed_output_dev vibrator;
	struct work_struct work;
	struct hrtimer timer;
	int vibra_on;
	spinlock_t lock;
};


static int pmic6832_set_vibrator(struct pmic6832_vibra_device *vib)
{
	int reg_val;
	int intensity = vib->vibra_on;

	if (intensity) {
		reg_val = (((((u32)intensity)*127)/100) & 0x7F);
		vmm_pmic_reg_write(PMIC_DEV6_ADDR | VIBRA_PMIC_CONTROL, VIBRA_DOWN);
		vmm_pmic_reg_write(PMIC_DEV6_ADDR | VIBRA_PMIC_PWM, reg_val);
		vmm_pmic_reg_write(PMIC_DEV6_ADDR | VIBRA_PMIC_CONTROL, VIBRA_UP);
	} else {
		vmm_pmic_reg_write(PMIC_DEV6_ADDR | VIBRA_PMIC_CONTROL, VIBRA_DOWN);
	}

	return 0;
}

static void pmic6832_vibra_work(struct work_struct *work)
{
	struct pmic6832_vibra_device *vib =
			container_of(work, struct pmic6832_vibra_device, work);
	int ret = 0;

	if (vib->vibra_on) {
		ret = pmic6832_set_vibrator(vib);
		if (ret) {
			dev_err(vib->vibrator.dev,
			"%s Set vibrator Failed\n", __func__);
			return;
		}
	} else {
		ret = pmic6832_set_vibrator(vib);
		if (ret) {
			dev_err(vib->vibrator.dev,
			"%s Set vibrator Failed\n", __func__);
			return;
		}
	}
}

static void pmic6832_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct pmic6832_vibra_device *vib =
		container_of(dev, struct pmic6832_vibra_device, vibrator);
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

static enum hrtimer_restart pmic6832_hrtimer_callback(struct hrtimer *timer)
{
	struct pmic6832_vibra_device *vib =
		container_of(timer, struct pmic6832_vibra_device, timer);
	vib->vibra_on = 0;
	schedule_work(&vib->work);
	return HRTIMER_NORESTART;
}

static int pmic6832_vibrator_get_time(struct timed_output_dev *dev)
{
	return 0;
}


static int pmic6832_vibra_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pmic6832_vibra_device *vib;

	pr_info("%s: entry.\n", __func__);
	vib = kzalloc(sizeof(struct pmic6832_vibra_device), GFP_KERNEL);
	if (!vib) {
		dev_err(&pdev->dev,
			"not enough memory for driver data\n");
		return -ENOMEM;
	}

	vib->pdev = pdev;
	INIT_WORK(&vib->work, pmic6832_vibra_work);
	spin_lock_init(&vib->lock);

	vib->vibrator.name = "vibrator";
	vib->vibrator.enable = pmic6832_vibrator_enable;
	vib->vibrator.get_time = pmic6832_vibrator_get_time;

	hrtimer_init(&vib->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->timer.function = pmic6832_hrtimer_callback;
	ret = timed_output_dev_register(&vib->vibrator);
	if (ret != 0) {
		dev_err(&pdev->dev,
			" unable to register with timed output\n");
		ret = -EINVAL;
		goto failed_free_mem;
	}

	platform_set_drvdata(pdev, vib);
	dev_err(&pdev->dev, "Vibrator driver probed\n");

	return 0;

failed_free_mem:
	kfree(vib);

	return ret;
}

static int pmic6832_vibra_remove(struct platform_device *pdev)
{
	struct pmic6832_vibra_device *vib = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	cancel_work_sync(&vib->work);
	timed_output_dev_unregister(&vib->vibrator);
	kfree(vib);
	return 0;
}

static struct of_device_id pmic6832_vibra_of_match[] = {
	{.compatible = "intel,pmic-vibra",},
	{},
};

static struct platform_driver pmic6832_vibra_driver = {
	.driver		= {
		.name	= PMIC6832_VIBRA_MODULE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = pmic6832_vibra_of_match,
	},
	.probe		= pmic6832_vibra_probe,
	.remove		= pmic6832_vibra_remove,
};
MODULE_DEVICE_TABLE(of, pmic6832_vibra_of_match);

static int __init pmic6832_vibra_init(void)
{
	pr_info("pmic Vibrator init\n");
	return platform_driver_register(&pmic6832_vibra_driver);
}

static void __exit pmic6832_vibra_exit(void)
{
	pr_info("pmic Vibrator exit\n");
	platform_driver_unregister(&pmic6832_vibra_driver);
}

module_init(pmic6832_vibra_init);
module_exit(pmic6832_vibra_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("vibra driver for PMIC6832");
