/*
 * Copyright (C) 2013, 2014, 2015 Intel Mobile Communications GmbH
 *
 * Notes:
 * Nov	5 2013: IMC: debug fs for hx280enc driver
 * Mar 13 2014: IMC: Review Comments & Clean up
 * May 20 2014: IMC: replace printk() with pr_*()
 * Oct 30 2014: IMC: include pingvm command
 * Mar 17 2015: IMC: VVPU only: remove code accessing HW
 */

/*
 * Decoder device driver - debug fs module
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *					     Boston, MA	 02110-1301, USA.
 *
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/fs.h>

#include <linux/platform_device.h>

#include "hx280enc.h"
#include <sofia/vvpu_vbpipe.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("hx280 DebugFS functionality");

static char *hx280_build_date = __DATE__ " " __TIME__;

/* command buffer length: use 4k */
#define HX280_CMD_LEN 4096
static char hx280_cmdBuffer[HX280_CMD_LEN];

static int hx280_fileValue;


static struct dentry *hx280_dir;
static struct dentry *hx280_ctrl;

static struct device	       *dev;
static struct platform_device  *pdev;

static const struct dev_pm_ops *dev_pm_ops;

/*
 * define comands understood by the ctrl interface
 */
enum _hx280_device_ctrl_cmds {
	_HX280_DEVICE_HELP,
	_HX280_DEVICE_PREPARE,
	_HX280_DEVICE_COMPLETE,
	_HX280_DEVICE_SUSPEND,
	_HX280_DEVICE_RESUME,
	_HX280_DEVICE_ON,
	_HX280_DEVICE_OFF,
	_HX280_DEVICE_SET_PWR_ON,
	_HX280_DEVICE_RESET,
	_HX280_DEVICE_STATE,
	_HX280_DEVICE_COVERAGE,
	_HX280_DEVICE_PING,
	_HX280_MAX_CMDS
};

static const char * const hx280_ctrl_cmds[_HX280_MAX_CMDS] = {
	[_HX280_DEVICE_HELP]	   = "help",
	[_HX280_DEVICE_PREPARE]	   = "prepare",
	[_HX280_DEVICE_COMPLETE]   = "complete",
	[_HX280_DEVICE_SUSPEND]	   = "suspend",
	[_HX280_DEVICE_RESUME]	   = "resume",
	[_HX280_DEVICE_ON]	   = "on",
	[_HX280_DEVICE_OFF]	   = "off",
	[_HX280_DEVICE_SET_PWR_ON] = "poweron",
	[_HX280_DEVICE_RESET]	   = "reset",
	[_HX280_DEVICE_STATE]	   = "state",
	[_HX280_DEVICE_COVERAGE]   = "coverage",
	[_HX280_DEVICE_PING]	   = "pingvm",
};


/*
 * internal functions
 */
static int hx280_print_state(char *p, ssize_t size);
static int hx280_set_power_on_handle(char *argp);


/*
 * open file operation
 */
int hx280_ctrlopen(struct inode *ip, struct file *fp)
{
	int ret = 0;

	pr_info("hx280dbg open(inode %p file %p) = %d\n",
		ip, fp, ret);

	return ret;
}

/*
 * release file operation
 */
int hx280_ctrlrelease(struct inode *ip, struct file *fp)
{
	int ret = 0;

	pr_info("hx280dbg close(inode %p file %p) = %d\n",
		ip, fp, ret);

	return ret;
}

/*
 * read file operation: provide information to user space
 */
static ssize_t hx280_ctrlread(struct file *fp, char __user *user_buffer,
	size_t count, loff_t *position)
{

	ssize_t ret = 0;


	ret = hx280_print_state(hx280_cmdBuffer, HX280_CMD_LEN);

	ret = simple_read_from_buffer(user_buffer,
		(ret <= count ? ret : count),
		position, hx280_cmdBuffer, HX280_CMD_LEN);

	pr_info("hx280dbg hx280_ctrlread(%p count %d pos %p) = %d\n",
		user_buffer, count, position, ret);

	return ret;
}

/*
 * write file operation: receive commands from user space
 */
static ssize_t hx280_ctrlwrite(struct file *fp, const char __user *user_buffer,
	size_t count, loff_t *position)
{
	int n = 0;
	ssize_t ret = 0;
	struct vpu_enc_device_t *vpu_enc_data = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = vpu_enc_data->pm_platdata;

	pr_info("hx280dbg hx280_pmwrite(%p len %d)\n",
		user_buffer, count);

	if (count >= HX280_CMD_LEN) {
		ret = -EINVAL;
		goto end;
	}

	if (dev == NULL) {
		pr_warn("hx280dbg dev = 0; PM calls not possible\n");
		ret = 0;
		goto end;
	}

	if (pdev == NULL) {
		pr_warn("hx280dbg platform dev = 0; PM calls not possible\n");
		ret = 0;
		goto end;
	}

	if (dev_pm_ops == NULL) {
		pr_warn("hx280dbg dev_pm_ops = 0; PM calls not possible\n");
		ret = 0;
		goto end;
	}

	ret = simple_write_to_buffer(hx280_cmdBuffer, HX280_CMD_LEN, position,
		user_buffer, count);
	hx280_cmdBuffer[count] = 0;

	if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_HELP],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_HELP]))) {

		pr_info("hx280dbg command list:\n");
		for (n = 0; n < _HX280_MAX_CMDS; ++n)
			pr_info("	%s\n", hx280_ctrl_cmds[n]);

	} else if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_STATE],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_STATE]))) {
		/*
		 * report VPU state
		 */

		n = hx280_print_state(hx280_cmdBuffer, HX280_CMD_LEN);
		pr_info("%s", hx280_cmdBuffer);

	} else if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_SET_PWR_ON],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_SET_PWR_ON]))) {
		/*
		 * set power on handle
		 */
		int n;

		n = hx280_set_power_on_handle(hx280_cmdBuffer +
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_SET_PWR_ON]));

		pr_info("hx280dbg set power on handle %d\n", n);

	} else if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_RESET],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_RESET]))) {
		/*
		 * reset VPU state
		 */
		pr_info("hx280dbg reset vpu encoder\n");

		n = platform_device_pm_set_state_by_name(pdev,
					pm_platdata->pm_state_D3_name);

		hx280enc_reset_power_state();

		/* other things to reset?? */

		pr_info("hx280dbg vpu encoder OFF (%d)\n", n);

	} else if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_ON],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_ON]))) {
		/*
		 * switch on device unconditionally
		 */
		n = platform_device_pm_set_state_by_name(pdev,
					pm_platdata->pm_state_D0_name);
		/*
		 * increase power state if successful
		 * do we really want this?
		 */
		if (n == 0)
			hx280enc_power_state(1);

		pr_info("hx280dbg vpu encoder ON (%d)\n", n);

	} else if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_OFF],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_OFF]))) {
		/*
		 * switch off device unconditionally
		 */
		n = platform_device_pm_set_state_by_name(pdev,
					pm_platdata->pm_state_D3_name);
		if (n == 0)
			hx280enc_power_state(-1);

		pr_info("hx280dbg vpu encoder OFF (%d)\n", n);

	} else if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_PREPARE],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_PREPARE]))) {
		/*
		 * call prepare
		 */
		if (dev_pm_ops->prepare != NULL) {
			int r;

			pr_info("hx280dbg device PREPARE\n");
			r = dev_pm_ops->prepare(dev);
			pr_info("hx280dbg device PREPARE = %d\n", r);
		} else
			pr_info("hx280dbg device PREPARE not supported\n");


	} else if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_COMPLETE],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_COMPLETE]))) {
		/*
		 * call complete
		 */
		if (dev_pm_ops->complete != NULL) {

			pr_info("hx280dbg device COMPLETE\n");
			dev_pm_ops->complete(dev);
			pr_info("hx280dbg device COMPLETE = void\n");
		} else
			pr_info("hx280dbg device COMPLETE not supported\n");


	} else if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_SUSPEND],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_SUSPEND]))) {
		/*
		 * call suspend function
		 */
		if (dev_pm_ops->suspend != NULL) {
			int r;

			pr_info("hx280dbg device SUSPEND\n");
			r = dev_pm_ops->suspend(dev);
			pr_info("hx280dbg device SUSPEND = %d\n", r);
		} else
			pr_info("hx280dbg device SUSPEND not supported\n");

	} else if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_RESUME],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_RESUME]))) {
		/*
		 * call resume function
		 */
		if (dev_pm_ops->resume != NULL) {
			int r;

			pr_info("hx280dbg device RESUME\n");
			r = dev_pm_ops->resume(dev);
			pr_info("hx280dbg device RESUME = %d\n", r);
		} else
			pr_info("hx280dbg device RESUME not supported\n");

	} else if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_COVERAGE],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_COVERAGE]))) {
		/*
		 * call function for coverage testing
		 */
		pr_info("hx280dbg coverage testing ... TBD\n");

	} else if (!strncmp(hx280_cmdBuffer,
			hx280_ctrl_cmds[_HX280_DEVICE_PING],
			strlen(hx280_ctrl_cmds[_HX280_DEVICE_PING]))) {
		/*
		 * call function for pinging the secure vm
		 */
		pr_info("hx280dbg ping secure vm\n");
		vvpu_ping(dev, 0xfac31355);

	} else {

		pr_info("hx280dbg command %s not supported\n",
			hx280_cmdBuffer);
	}

end:
	return ret;
}

static const struct file_operations fops_ctrl = {
	.open	 = hx280_ctrlopen,
	.release = hx280_ctrlrelease,
	.read = hx280_ctrlread,
	.write = hx280_ctrlwrite,
};

int hx280_init_debug(void)
{
	/* create a directory by the name dell in /sys/kernel/debugfs */
	hx280_dir = debugfs_create_dir(H1_DRIVER_NAME, NULL);

	/*
	   create ctrl node in the above directory
	   This requires read and write file operations
	*/
	hx280_ctrl = debugfs_create_file("ctrl", 0644, hx280_dir,
		&hx280_fileValue, &fops_ctrl);

	return 0;
}

int hx280_probe_debug(struct device *the_dev,
	struct platform_device	*the_p_dev,
	const struct dev_pm_ops *pm_ops)
{
	int ret = 0;

	dev	   = the_dev;
	pdev	   = the_p_dev;
	dev_pm_ops = pm_ops;

	pr_info("hx280dbg registered device and pm ops\n");

#if 0
	if (pm_ops != NULL) {
		pr_info("    ops->prepare	   = %p\n",
			pm_ops->prepare);
		pr_info("    ops->complete	   = %p\n",
			pm_ops->complete);
		pr_info("    ops->suspend	   = %p\n",
			pm_ops->suspend);
		pr_info("    ops->resume	   = %p\n",
			pm_ops->resume);
		pr_info("    ops->runtime_suspend = %p\n",
			pm_ops->runtime_suspend);
		pr_info("    ops->runtime_resume  = %p\n",
			pm_ops->runtime_resume);
		pr_info("    ops->runtime_idle	   = %p\n",
			pm_ops->runtime_idle);
	} else
		pr_info("    ops == NULL\n");
#endif

	return ret;
}

void hx280_release_debug(void)
{
	/* removing the directory recursively which
	   in turn cleans all the file */

	dev	   = NULL;
	dev_pm_ops = NULL;

	if (hx280_dir != NULL)
		debugfs_remove_recursive(hx280_dir);
}



static int hx280_print_state(char *p, ssize_t size)
{
	int n, i;
	ssize_t rest = size;

#define ADVANCE(j) do { p += (j); rest -= (j); } while (0)

	i = snprintf(p, rest, "hx280dbg build date: %s\n", hx280_build_date);
	ADVANCE(i);

	/*
	 * report VPU state
	 */
	i = snprintf(p, rest, "device state:\n");
	ADVANCE(i);

	i = snprintf(p, rest, " + vpu decoder is currently%s available",
		(hx280enc_pm_get_avail() ? "" : " not"));
	ADVANCE(i);

	n = hx280enc_power_state(0);
	if (n < 0) {
		i = snprintf(p, rest, ", cannot get power state\n");
		ADVANCE(i);
	} else {
		i = snprintf(p, rest, ", power is %s\n",
			(n > 0 ? "on" : "off"));
		ADVANCE(i);
	}

	i = snprintf(p, rest, "	 + power handles\n");

#define IS_POWER_ON_HDL(hdl) \
	((hdl) == (*xgold_vpu_enc_pm_state_on_p) ? " = power on" : "")

	ADVANCE(i);
	i = snprintf(p, rest, "	   + disable %p%s\n",
		xgold_vpu_enc_pm_state_disable,
		IS_POWER_ON_HDL(xgold_vpu_enc_pm_state_disable));
	ADVANCE(i);
	i = snprintf(p, rest, "	   + low %p%s\n",
		xgold_vpu_enc_pm_state_low_perf,
		IS_POWER_ON_HDL(xgold_vpu_enc_pm_state_low_perf));
	ADVANCE(i);
	i = snprintf(p, rest, "	   + mid %p%s\n",
		xgold_vpu_enc_pm_state_mid_perf,
		IS_POWER_ON_HDL(xgold_vpu_enc_pm_state_mid_perf));
	ADVANCE(i);
	i = snprintf(p, rest, "	   + high %p%s\n",
		xgold_vpu_enc_pm_state_high_perf,
		IS_POWER_ON_HDL(xgold_vpu_enc_pm_state_high_perf));
	ADVANCE(i);
	i = snprintf(p, rest, "	   + ultra high %p%s\n",
		xgold_vpu_enc_pm_state_ultra_high_perf,
		IS_POWER_ON_HDL(xgold_vpu_enc_pm_state_ultra_high_perf));
	ADVANCE(i);

	return size - rest;
}

/*
 * map power handled to strings
 */
struct {
	const char *name;
	struct platform_device_pm_state **p_handle;
	const char *note;
} hx280_power_on_handles[] = {
	{ "disable",
	  &xgold_vpu_enc_pm_state_disable,
	  "WARNING: disable handle!!" },
	{ "low",
	  &xgold_vpu_enc_pm_state_low_perf,
	  "low performance, CPU 104 MHz, Bus >= 26 MHz" },
	{ "mid",
	  &xgold_vpu_enc_pm_state_mid_perf,
	  "mid performance, CPU 156 MHz, Bus >= 26 MHz"},
	{ "high",
	  &xgold_vpu_enc_pm_state_high_perf,
	  "high performance, CPU 312 MHz, Bus >= 26 MHz"},
#ifdef HX280_USE_ULTRA_HIGH_PERF
	{ "ultra",
	  &xgold_vpu_enc_pm_state_ultra_high_perf,
	  "ultra high performance, CPU 420 MHz, Bus >= 26 MHz"},
#endif
	{ NULL, NULL, NULL }
};


/*
 * set power on handle
 */
static int hx280_set_power_on_handle(char *argp)
{
	int i;
	int ret = -1;

#define SKIP_BLANKS(p) { while (*(p) == ' ') (p)++; }

	SKIP_BLANKS(argp);

	for (i = 0; hx280_power_on_handles[i].name != NULL; i++) {
		if (!strncmp(argp, hx280_power_on_handles[i].name,
				strlen(hx280_power_on_handles[i].name))) {

			xgold_vpu_enc_pm_state_on_p
				= hx280_power_on_handles[i].p_handle;

			pr_info("hx280dbg set power on handle to %s\n",
				hx280_power_on_handles[i].name);
			pr_info("hx280dbg note: %s\n",
				hx280_power_on_handles[i].note);

			ret = i;
			break;
		}
	}

	if (ret < 0)
		pr_info("hx280dbg did not find power on handle for \"%s\"\n",
			argp);

	return ret;
}
