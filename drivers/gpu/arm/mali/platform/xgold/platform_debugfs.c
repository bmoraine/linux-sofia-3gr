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
 *
 * Notes:
 * Nov 18 2014: IMC: Adaptions for Mali Utgard driver r5p0-01rel0
 *                   - New method to enable/disable DVFS via debugfs
 * Jul 16 2014: IMC: [OSS Scan] Add missing license type
 * Jun 02 2014: IMC: Add pm and debugfs support
 *                   Splitup platform adaption for better readability
 */

#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#if defined(CONFIG_PM_RUNTIME)
#include <linux/pm_runtime.h>
#endif

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/mali/mali_utgard.h>

#include "platform_intern.h"


static struct dentry *dev_gpu_debugfs_dir;
static struct mali_platform_pm *p_dev_pm;
static struct platform_device *p_dev;
static const struct dev_pm_ops *p_dev_pm_ops;

#define GPU_INITIAL_PM_STATE (p_dev_pm->pm_status_num-1)


static ssize_t dev_gpu_dvfs_read(struct file *filp, char __user *ubuf,
	size_t cnt, loff_t *ppos) {
	char output_buffer[3];
	size_t r;

	r = snprintf(output_buffer, 3, "%u\n", p_dev_pm->dvfs_off);
	return simple_read_from_buffer(ubuf, cnt, ppos, output_buffer, r);
}

static ssize_t dev_gpu_dvfs_write(struct file *filp, const char __user *ubuf,
	size_t cnt, loff_t *ppos)
{
	int ret;
	char control_buffer[2];
	unsigned long control_value;
	unsigned int prev_pm_state;

	/* Copy single character from buffer */
	if (copy_from_user(control_buffer, ubuf, sizeof(control_buffer)-1))
		return -EFAULT;

	/* Add trailing zero */
	control_buffer[sizeof(control_buffer)-1] = '\0';

	/* Convert to unsigned long */
	ret = kstrtoul(control_buffer, 10, &control_value);
	if (0 != ret)
		return ret;

	/* Call debug function based on received control_value */
	if (control_value == 1) {
		mali_dbg("DebugFS - Disable DVFS!\n");

		p_dev_pm->dvfs_off = true;
		flush_workqueue(p_dev_pm->dvfs_wq);

		/* When disabling DVFS switch to GPU_INITIAL_PM_STATE */
#if defined(CONFIG_PM_RUNTIME)
		if ((p_dev_pm->curr_pm_state != GPU_INITIAL_PM_STATE) &&
			!pm_runtime_suspended(&(p_dev->dev))) {
#else
		if (p_dev_pm->curr_pm_state != GPU_INITIAL_PM_STATE) {
#endif
			prev_pm_state = p_dev_pm->curr_pm_state;
			p_dev_pm->curr_pm_state = GPU_INITIAL_PM_STATE;
			mali_dbg("DebugFS - PM Setting power state to %d\n",
				p_dev_pm->curr_pm_state);
			mali_dev_pause();
			ret = platform_device_pm_set_state(p_dev,
				p_dev_pm->pm_states[
				p_dev_pm->curr_pm_state]);
			mali_dev_resume();
			if (ret != 0) {
				p_dev_pm->curr_pm_state = prev_pm_state;
				mali_err(
					"DebugFS - PM set state failed (%d)\n"
					, ret);
			}
		}
	} else {
		mali_dbg("DebugFS - Enable DVFS!\n");

		/* When reenabling DVFS reset resume_pm_state to default */
		p_dev_pm->resume_pm_state = GPU_INITIAL_PM_STATE;
		p_dev_pm->req_clock_index = p_dev_pm->resume_pm_state - 1;

		p_dev_pm->dvfs_off = false;
	}

	*ppos += cnt;

	return cnt;
}

static const struct file_operations dev_gpu_dvfs_fops = {
	.owner = THIS_MODULE,
	.read = dev_gpu_dvfs_read,
	.write = dev_gpu_dvfs_write,
};


static ssize_t dev_gpu_os_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret;
	char control_buffer[2];
	unsigned long control_value;

	/* Copy single character from buffer */
	if (copy_from_user(control_buffer, ubuf, sizeof(control_buffer)-1))
		return -EFAULT;

	/* Add trailing zero */
	control_buffer[sizeof(control_buffer)-1] = '\0';

	/* Convert to unsigned long */
	ret = kstrtoul(control_buffer, 10, &control_value);
	if (0 != ret)
		return ret;

	/* Call debug function based on received control_value */
	if (control_value == 1) {
		mali_dbg("DebugFS >>>suspend\n");
		(p_dev_pm_ops->suspend)(&(p_dev->dev));
		mali_dbg("DebugFS <<<suspend\n");
	} else {
		mali_dbg("DebugFS >>>resume\n");
		(p_dev_pm_ops->resume)(&(p_dev->dev));
		mali_dbg("DebugFS <<<resume\n");
	}

	*ppos += cnt;

	return cnt;
}

static const struct file_operations dev_gpu_os_fops = {
	.owner = THIS_MODULE,
	.write = dev_gpu_os_write,
};


static ssize_t dev_gpu_runtime_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret;
	char control_buffer[2];
	unsigned long control_value;

	/* Copy single character from buffer */
	if (copy_from_user(control_buffer, ubuf, sizeof(control_buffer)-1))
		return -EFAULT;

	/* Add trailing zero */
	control_buffer[sizeof(control_buffer)-1] = '\0';

	/* Convert to unsigned long */
	ret = kstrtoul(control_buffer, 10, &control_value);
	if (0 != ret)
		return ret;

	/* Call debug function based on received control_value */
	if (control_value == 1) {
		mali_dbg("DebugFS >>>runtime_suspend\n");
		p_dev_pm_ops->runtime_suspend(&(p_dev->dev));
		mali_dbg("DebugFS <<<runtime_suspend\n");
	} else {
		mali_dbg("DebugFS >>>runtime_resume\n");
		p_dev_pm_ops->runtime_resume(&(p_dev->dev));
		mali_dbg("DebugFS <<<runtime_resume\n");
	}

	*ppos += cnt;

	return cnt;
}

static const struct file_operations dev_gpu_runtime_fops = {
	.owner = THIS_MODULE,
	.write = dev_gpu_runtime_write,
};


static ssize_t dev_gpu_pm_read(struct file *filp, char __user *ubuf,
	size_t cnt, loff_t *ppos) {
	char output_buffer[3];
	size_t r;

	r = snprintf(output_buffer, 3, "%u\n", p_dev_pm->curr_pm_state);
	return simple_read_from_buffer(ubuf, cnt, ppos, output_buffer, r);
}

static ssize_t dev_gpu_pm_write(struct file *filp, const char __user *ubuf,
	size_t cnt, loff_t *ppos) {
	int ret;
	char control_buffer[2];
	unsigned long control_value;
	unsigned int prev_pm_state;

	/* Copy single character from buffer */
	if (copy_from_user(control_buffer, ubuf, sizeof(control_buffer)-1))
		return -EFAULT;

	/* Add trailing zero */
	control_buffer[sizeof(control_buffer)-1] = '\0';

	/* Convert to unsigned long */
	ret = kstrtoul(control_buffer, 10, &control_value);
	if (0 != ret)
		return ret;

	/*
	Manual setting of power level via debugfs is only allowed
	if DVFS is disabled.
	*/
	if (false == p_dev_pm->dvfs_off) {
		mali_dbg(
			"DebugFS - PM Can't set GPU power level when dvfs is enabled!\n");
		return -EFAULT;
	}

	/* Call debug function based on received control_value */
	if ((0 < control_value) && (p_dev_pm->pm_status_num > control_value)) {
#if defined(CONFIG_PM_RUNTIME)
		if (pm_runtime_suspended(&(p_dev->dev)))
			p_dev_pm->resume_pm_state = control_value;
		else {
#endif
			prev_pm_state = p_dev_pm->curr_pm_state;
			p_dev_pm->curr_pm_state = control_value;
			mali_dbg("DebugFS - PM Setting power state to %d\n",
				p_dev_pm->curr_pm_state);
			mali_dev_pause();
			ret = platform_device_pm_set_state(p_dev,
				p_dev_pm->pm_states[
					p_dev_pm->curr_pm_state]);
			mali_dev_resume();
			if (ret != 0) {
				p_dev_pm->curr_pm_state = prev_pm_state;
				mali_err(
					"DebugFS - PM set state failed (%d)\n"
					, ret);
			} else
				p_dev_pm->resume_pm_state =
					p_dev_pm->curr_pm_state;
#if defined(CONFIG_PM_RUNTIME)
		}
#endif
	} else {
		mali_dbg("DebugFS - PM Invalid power level requested!\n");
		return -EFAULT;
	}

	*ppos += cnt;

	return cnt;
}

static const struct file_operations dev_gpu_pm_fops = {
	.owner = THIS_MODULE,
	.read = dev_gpu_pm_read,
	.write = dev_gpu_pm_write,
};


int platform_debugfs_register(struct mali_platform_pm *pdev_pm,
	struct platform_device *pdev)
{
	if ((NULL == pdev_pm) || (NULL == pdev))
		return -EINVAL;

	p_dev_pm = pdev_pm;
	p_dev_pm_ops = pdev->dev.driver->pm;
	p_dev = pdev;

	dev_gpu_debugfs_dir = debugfs_create_dir("mali_platform", NULL);
	if (IS_ERR_OR_NULL(dev_gpu_debugfs_dir))
		return -EFAULT;

	debugfs_create_file("dvfs_off", 0600,
		dev_gpu_debugfs_dir, NULL,
		&dev_gpu_dvfs_fops);
	debugfs_create_file("os_suspend", 0200,
		dev_gpu_debugfs_dir, NULL,
		&dev_gpu_os_fops);
#if defined(CONFIG_PM_RUNTIME)
	debugfs_create_file("runtime_suspend", 0200,
		dev_gpu_debugfs_dir, NULL,
		&dev_gpu_runtime_fops);
#endif
	debugfs_create_file("pm_level", 0600,
		dev_gpu_debugfs_dir, NULL,
		&dev_gpu_pm_fops);

	return 0;
}
