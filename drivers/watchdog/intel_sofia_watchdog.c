/*
 *
 *      Copyright (C) 2013-2014 Intel Corporation. All rights reserved.
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of version 2 of the GNU General
 *      Public License as published by the Free Software Foundation.
 *
 *      This program is distributed in the hope that it will be
 *      useful, but WITHOUT ANY WARRANTY; without even the implied
 *      warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *      PURPOSE.  See the GNU General Public License for more details.
 *      The full GNU General Public License is included in this
 *      distribution in the file called COPYING.
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/nmi.h>
#include <sofia/mv_svc_hypercalls.h>

#define PFX "intel_sofia_watchdog: "

static bool disable_sofia_watchdog;
module_param(disable_sofia_watchdog, bool, S_IRUGO);
/* Disable sofia watchdog,"
	"Set to 0, watchdog started at boot"
	"and left running; Set to 1, watchdog"
	"is not started until user space"
	"watchdog daemon is started");
*/
MODULE_PARM_DESC(disable_sofia_watchdog, "Disable sofia watchdog");

struct intel_scu_watchdog_dev {
	struct miscdevice miscdev;
	struct mutex lock;
	unsigned int threshold;
	bool kernel_enable;
	bool scu_enable;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dfs_wd;
	struct dentry *dfs_scu_wd;
	struct dentry *dfs_scu_wd_enable;
	struct dentry *dfs_scu_wd_trigger;
	struct dentry *dfs_kwd;
	struct dentry *dfs_kwd_enable;
	struct dentry *dfs_kwd_trigger;
#endif /* CONFIG_DEBUG_FS */
} sofia_watchdog_device;

static int intel_sofia_watchdog_keepalive(void)
{
	pr_notice("%s\n", __func__);
	mv_svc_watchdog_pet();

	return 0;
}

/*
 * /dev/watchdog handling
 */
static int intel_sofia_watchdog_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static ssize_t intel_sofia_watchdog_write(struct file *file,
		char const *data,
		size_t len,
		loff_t *ppos)
{
	mutex_lock(&sofia_watchdog_device.lock);

	if (sofia_watchdog_device.kernel_enable)
		intel_sofia_watchdog_keepalive();

	mutex_unlock(&sofia_watchdog_device.lock);

	return len;
}

static long intel_sofia_watchdog_ioctl(struct file *file,
		unsigned int cmd,
		unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = 0;
	u32 __user *p = argp;
	u32 new_margin;

	mutex_lock(&sofia_watchdog_device.lock);

	switch (cmd) {
	case WDIOC_KEEPALIVE:
		if (sofia_watchdog_device.kernel_enable)
			ret = intel_sofia_watchdog_keepalive();
		break;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, p)) {
			ret = -EFAULT;
			break;
		}
		sofia_watchdog_device.threshold = new_margin;
		if (!disable_sofia_watchdog) {
			ret = mv_svc_watchdog_enable(new_margin);
			pr_debug("%s threshold = %d\n", __func__, new_margin);
			if (ret == 0)
				sofia_watchdog_device.kernel_enable = true;
		}
		break;
	case WDIOC_GETTIMEOUT:
		ret =  put_user(sofia_watchdog_device.threshold, p);
		break;
	default:
		ret =  -ENOTTY;
	}

	mutex_unlock(&sofia_watchdog_device.lock);

	return ret;
}

static int intel_sofia_watchdog_release(struct inode *inode, struct file *file)
{
	/* This watchdog should not be closed, here just return and wait
	watchdog timeout */

	return 0;
}

static const struct file_operations intel_sofia_watchdog_fops = {
	.owner          = THIS_MODULE,
	.llseek         = no_llseek,
	.write          = intel_sofia_watchdog_write,
	.unlocked_ioctl = intel_sofia_watchdog_ioctl,
	.open           = intel_sofia_watchdog_open,
	.release        = intel_sofia_watchdog_release,
};

#ifdef CONFIG_DEBUG_FS
static int intel_sofia_watchdog_enable_debugfs_show(void *data, uint64_t *value)
{
	mutex_lock(&sofia_watchdog_device.lock);
	*value = sofia_watchdog_device.kernel_enable;
	mutex_unlock(&sofia_watchdog_device.lock);

	return 0;
}

static int intel_sofia_watchdog_enable_debugfs_set(void *data, uint64_t value)
{

	mutex_lock(&sofia_watchdog_device.lock);

	if (value != 0 && !sofia_watchdog_device.kernel_enable)
		mv_svc_watchdog_enable(sofia_watchdog_device.threshold);

	else if (value == 0 && sofia_watchdog_device.kernel_enable)
		mv_svc_watchdog_disable();

	sofia_watchdog_device.kernel_enable = !!value;

	mutex_unlock(&sofia_watchdog_device.lock);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(intel_sofia_watchdog_enable_attr,
		intel_sofia_watchdog_enable_debugfs_show,
		intel_sofia_watchdog_enable_debugfs_set,
		"%llu\n");


static int intel_sofia_watchdog_trigger_debugfs_set(void *data, uint64_t value)
{
	pr_err(PFX "kwdt_trigger\n");
	/* set the enable = 0 but not stop kernel watchdog will trigger
	 * the kernel watchdog expire */

	sofia_watchdog_device.kernel_enable = 0;

	return 0;

}
DEFINE_SIMPLE_ATTRIBUTE(kwd_trigger_attr,
		NULL,
		intel_sofia_watchdog_trigger_debugfs_set,
		"%llu\n");

static int scu_watchdog_debugfs_show(void *data, uint64_t *value)
{
	*value = sofia_watchdog_device.scu_enable;

	return 0;
}

static int scu_watchdog_debugfs_set(void *data, uint64_t value)
{
	sofia_watchdog_device.scu_enable = !!value;
	mv_svc_scu_watchdog_switch(sofia_watchdog_device.scu_enable);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(scu_wdt_attr,
		scu_watchdog_debugfs_show,
		scu_watchdog_debugfs_set,
		"%llu\n");

static int intel_sofia_watchdog_scu_trigger_debugfs_set
		(void *data, uint64_t value)
{
	if (sofia_watchdog_device.scu_enable)
		mv_svc_scu_watchdog_switch(2);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(scu_trigger_attr,
		NULL,
		intel_sofia_watchdog_scu_trigger_debugfs_set,
		"%llu\n");

static int remove_debugfs_entries(void)
{
	struct intel_scu_watchdog_dev *dev = &sofia_watchdog_device;

	/* /sys/kernel/debug/watchdog */
	debugfs_remove_recursive(dev->dfs_wd);

	return 0;
}

static int create_debugfs_entries(void)
{
	struct intel_scu_watchdog_dev *dev = &sofia_watchdog_device;

	/* /sys/kernel/debug/watchdog */
	dev->dfs_wd = debugfs_create_dir("watchdog", NULL);
	if (!dev->dfs_wd) {
		pr_err(PFX "%s: Cannot create main dir\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/scu_watchdog */
	dev->dfs_scu_wd = debugfs_create_dir("scu_watchdog", dev->dfs_wd);
	if (!dev->dfs_scu_wd) {
		pr_err(PFX "%s: Cannot create scu dir\n", __func__);
		goto error;
	}
	/* /sys/kernel/debug/watchdog/scu_watchdog/scu_wdt_enable */
	dev->dfs_scu_wd_enable = debugfs_create_file("enable",
			0644, dev->dfs_scu_wd, NULL, &scu_wdt_attr);
	if (!dev->dfs_scu_wd_enable) {
		pr_err(PFX "%s: Cannot create scu_wdt_enable\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/scu_watchdog/trigger */
	dev->dfs_scu_wd_trigger = debugfs_create_file("trigger", 0220,
			dev->dfs_scu_wd, NULL, &scu_trigger_attr);

	if (!dev->dfs_scu_wd_trigger) {
		pr_err(PFX "%s: Cannot create vmm scu trigger file\n",
				__func__);
		goto error;
	}
	/* /sys/kernel/debug/watchdog/kernel_watchdog */
	dev->dfs_kwd = debugfs_create_dir("kernel_watchdog", dev->dfs_wd);
	if (!dev->dfs_kwd) {
		pr_err(PFX "%s: Cannot create kwd dir\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog/enable */
	dev->dfs_kwd_enable = debugfs_create_file("enable", 0644,
			dev->dfs_kwd, NULL, &intel_sofia_watchdog_enable_attr);
	if (!dev->dfs_kwd_enable) {
		pr_err(PFX "%s: Cannot create kwd enable file\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog/trigger */
	dev->dfs_kwd_trigger = debugfs_create_file("trigger", 0220,
			dev->dfs_kwd, NULL, &kwd_trigger_attr);

	if (!dev->dfs_kwd_trigger) {
		pr_err(PFX "%s: Cannot create kwd trigger file\n", __func__);
		goto error;
	}

	return 0;
error:
	remove_debugfs_entries();
	return 1;

}
#endif

static ssize_t intel_sofia_watchdog_disable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	if (!strtobool(buf, &disable_sofia_watchdog)) {
		if (disable_sofia_watchdog) {
			pr_err("%s: Disable sofia watchdog\n", __func__);

			sofia_watchdog_device.kernel_enable = false;
			ret = mv_svc_watchdog_disable();
			if (ret != 0)
				pr_err("cant disable kernel watchdog\n");

			sofia_watchdog_device.scu_enable = false;
			mv_svc_scu_watchdog_switch(
					sofia_watchdog_device.scu_enable);
		} else {
			sofia_watchdog_device.kernel_enable = true;
			mv_svc_watchdog_enable(
					sofia_watchdog_device.threshold);

			sofia_watchdog_device.scu_enable = true;
			mv_svc_scu_watchdog_switch(
					sofia_watchdog_device.scu_enable);
		}
	} else {
		pr_err("got invalid value\n");
		return -EINVAL;
	}

	return size;
}

static ssize_t intel_sofia_watchdog_disable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_debug("%s\n", __func__);
	if (disable_sofia_watchdog)
		return sprintf(buf, "1\n");

	return sprintf(buf, "0\n");
}

static DEVICE_ATTR(disable, S_IWUSR | S_IRUGO,
	intel_sofia_watchdog_disable_show, intel_sofia_watchdog_disable_store);

static irqreturn_t intel_sofia_watchdog_isr(int irq, void *dev)
{
	trigger_all_cpu_backtrace();
	panic("Kernel Watchdog timeout\n");

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM_SLEEP
static int intel_sofia_watchdog_suspend(struct device *dev)
{
      /*
	   * need to disabe watchdog when system suspend,
	   * in case vmm is still alive
	   */
	if (sofia_watchdog_device.kernel_enable) {
		pr_err(PFX "disable watchdog.\n");
		mv_svc_watchdog_disable();
	}
	return 0;
}

static int intel_sofia_watchdog_resume(struct device *dev)
{
	if (sofia_watchdog_device.kernel_enable) {
		pr_err(PFX "enable watchdog.\n");
		mv_svc_watchdog_enable(sofia_watchdog_device.threshold);
	}
	return 0;
}
#endif

static int intel_sofia_watchdog_probe(struct platform_device *pdev)
{
	int ret;
	int wdt_timeout_irq;

	sofia_watchdog_device.miscdev.minor = 0;
	sofia_watchdog_device.miscdev.name = "watchdog";
	sofia_watchdog_device.miscdev.fops = &intel_sofia_watchdog_fops;
	sofia_watchdog_device.kernel_enable = false;
	mutex_init(&sofia_watchdog_device.lock);

	pr_debug("%s\n", __func__);
	ret = misc_register(&sofia_watchdog_device.miscdev);
	if (ret) {
		pr_err("cannot register miscdev %d err =%d\n",
				WATCHDOG_MINOR, ret);
		return ret;
	}

	ret = device_create_file(sofia_watchdog_device.miscdev.this_device,
		&dev_attr_disable);
	if (ret) {
		pr_warn("cant register dev file for disable\n");
		misc_deregister(&sofia_watchdog_device.miscdev);
		return ret;
	}

#ifdef CONFIG_DEBUG_FS
	create_debugfs_entries();
#endif
	if (disable_sofia_watchdog) {
		pr_err("%s: Disable sofia watchdog\n", __func__);

		sofia_watchdog_device.kernel_enable = false;
		ret = mv_svc_watchdog_disable();
		if (ret != 0)
			pr_err("cant disable timer\n");

		sofia_watchdog_device.scu_enable = false;
		mv_svc_scu_watchdog_switch(sofia_watchdog_device.scu_enable);
	} else {
		/* the vmm watchdog enable default, no need switch vmm */
		sofia_watchdog_device.scu_enable = true;
	}

	wdt_timeout_irq = platform_get_irq_byname(pdev, "WDT_TIMEOUT");
	if (!IS_ERR_VALUE(wdt_timeout_irq))
		ret |= request_irq(wdt_timeout_irq, intel_sofia_watchdog_isr,
			    IRQF_SHARED, "wdt_timeout", &sofia_watchdog_device);

	return ret;
}

static int intel_sofia_watchdog_remove(struct platform_device *pdev)
{
	misc_deregister(&sofia_watchdog_device.miscdev);
#ifdef CONFIG_DEBUG_FS
	remove_debugfs_entries();
#endif
	return 0;
}

static const struct of_device_id intel_sofia_watchdog_of_match[] = {
	{
		.compatible = "intel,watchdog",
	},
	{},
};

MODULE_DEVICE_TABLE(of, intel_sofia_watchdog_of_match);

static const struct dev_pm_ops intel_sofia_watchdog_pm_ops = {
	.suspend_late = intel_sofia_watchdog_suspend,
	.resume_early = intel_sofia_watchdog_resume,
};

static struct platform_driver watchdog_driver = {
	.probe = intel_sofia_watchdog_probe,
	.remove =  intel_sofia_watchdog_remove,
	.driver = {
		.name = "intel_sofia_watchdog",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(intel_sofia_watchdog_of_match),
		.pm = &intel_sofia_watchdog_pm_ops,
	}
};

static int __init intel_sofia_watchdog_init(void)
{
	return platform_driver_register(&watchdog_driver);
}

static void __exit intel_sofia_watchdog_exit(void)
{
	platform_driver_unregister(&watchdog_driver);
}

late_initcall(intel_sofia_watchdog_init);
module_exit(intel_sofia_watchdog_exit);

MODULE_AUTHOR("linx.z.chen@intel.com");
MODULE_DESCRIPTION("Intel Sofia Watchdog Device Driver");
MODULE_LICENSE("GPL");
