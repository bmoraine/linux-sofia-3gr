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
 *
 * Notes:
 * March 11 2015: IMC: Add initial xgold platform adaptation code
 *
 */
#ifdef CONFIG_MALI_MIDGARD_RT_PM
#include <linux/pm_runtime.h>
#endif
#include <mali_kbase.h>
#include <linux/debugfs.h>
#include <platform/xgold/mali_kbase_platform_xgold.h>
#include <platform/xgold/mali_kbase_dvfs_xgold.h>


static struct dentry *dev_gpu_debugfs_dir;
static struct kbase_device *kbdev;


static ssize_t dev_gpu_dvfs_read(struct file *filp, char __user *ubuf,
	size_t cnt, loff_t *ppos) {
	char output_buffer[3];
	size_t r;
	struct xgold_platform_context *plf_context;
	plf_context = (struct xgold_platform_context *) kbdev->platform_context;

	mali_dbg("%s()\n", __func__);
	r = snprintf(output_buffer, 3, "%u\n", plf_context->dvfs_off);
	return simple_read_from_buffer(ubuf, cnt, ppos, output_buffer, r);
}

static ssize_t dev_gpu_dvfs_write(struct file *filp, const char __user *ubuf,
	size_t cnt, loff_t *ppos)
{
	int ret;
	char control_buffer[2];
	unsigned long control_value;
	struct xgold_platform_context *plf_context;
	plf_context = (struct xgold_platform_context *) kbdev->platform_context;

	mali_dbg("%s()\n", __func__);
	/* Copy single character from buffer */
	if (copy_from_user(control_buffer, ubuf, sizeof(control_buffer)-1))
		return -EFAULT;

	/* Add trailing zero */
	control_buffer[sizeof(control_buffer)-1] = '\0';

	/* Convert to unsigned long */
	ret = kstrtoul(control_buffer, 10, &control_value);
	if (0 != ret)
		return ret;

	if (control_value == 1) {
		mali_dbg("DebugFS - Disable DVFS!\n");

		plf_context->dvfs_off = MALI_TRUE;

		mutex_lock(&plf_context->pm_lock_mutex);

		kbase_platform_dvfs_enable(MALI_FALSE, kbdev);

#ifdef CONFIG_MALI_MIDGARD_RT_PM
		if ((plf_context->curr_pm_state != plf_context->resume_pm_state)
			&& !pm_runtime_suspended(kbdev->dev)) {
#else
		if (plf_context->curr_pm_state !=
			plf_context->resume_pm_state) {
#endif
			ret = kbase_platform_xgold_pm_control(kbdev,
					plf_context->resume_pm_state);
			if (ret < 0) {
				mali_err("%s() ret = %d\n", __func__, ret);
				mutex_unlock(&plf_context->pm_lock_mutex);
				return ret;
			}
		}
		mutex_unlock(&plf_context->pm_lock_mutex);

	} else {
		mali_dbg("DebugFS - Enable DVFS!\n");
		plf_context->dvfs_off = MALI_FALSE;
		mutex_lock(&plf_context->pm_lock_mutex);

		kbase_platform_dvfs_enable(MALI_TRUE, kbdev);

		mutex_unlock(&plf_context->pm_lock_mutex);
	}
	*ppos += cnt;

	return cnt;
}

static const struct file_operations dev_gpu_dvfs_fops = {
	.owner = THIS_MODULE,
	.read = dev_gpu_dvfs_read,
	.write = dev_gpu_dvfs_write,
};

int platform_debugfs_register(struct kbase_device *p_kbdev)
{
	mali_dbg("%s()\n", __func__);
	if (NULL == p_kbdev)
		return -EINVAL;
	kbdev = p_kbdev;

	dev_gpu_debugfs_dir = debugfs_create_dir("mali_platform", NULL);
	if (IS_ERR_OR_NULL(dev_gpu_debugfs_dir))
		return -EFAULT;

	debugfs_create_file("dvfs_off", 0600,
		dev_gpu_debugfs_dir, NULL,
		&dev_gpu_dvfs_fops);

	return 0;
}
