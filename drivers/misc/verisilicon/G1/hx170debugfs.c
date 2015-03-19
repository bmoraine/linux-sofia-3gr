/*
 * Copyright (C) 2013, 2014, 2015 Intel Mobile Communications GmbH
 *
 * Notes:
 * Nov	5 2013: IMC: debug fs for hx170dec driver
 * Mar 13 2014: IMC: Review Comments & Clean up
 * May 20 2014: IMC: replace printk() with pr_*()
 * Oct 30 2014: IMC: include pingvm command
 * Mar 16 2015: IMC: VVPU only: remove code accessing HW
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

#include "hx170dec.h"
#include <sofia/vvpu_vbpipe.h>


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("hx170 DebugFS functionality");

static char *hx170_build_date = __DATE__ " " __TIME__;

/* command buffer length: use 4k */
#define HX170_CMD_LEN 4096
static char hx170_cmdBuffer[HX170_CMD_LEN];

static int hx170_fileValue;


static struct dentry *hx170_dir;
static struct dentry *hx170_ctrl;

static struct device	       *dev;
static struct platform_device  *pdev;

static const struct dev_pm_ops *dev_pm_ops;

/*
 * define comands understood by the ctrl interface
 */
enum _hx170_device_ctrl_cmds {
	_HX170_DEVICE_HELP,
	_HX170_DEVICE_PREPARE,
	_HX170_DEVICE_COMPLETE,
	_HX170_DEVICE_SUSPEND,
	_HX170_DEVICE_RESUME,
	_HX170_DEVICE_ON,
	_HX170_DEVICE_OFF,
	_HX170_DEVICE_SET_PWR_ON,
	_HX170_DEVICE_RESET,
	_HX170_DEVICE_STATE,
	_HX170_DEVICE_COVERAGE,
	_HX170_DEVICE_PING,
	_HX170_MAX_CMDS
};

static const char * const hx170_ctrl_cmds[_HX170_MAX_CMDS] = {
	[_HX170_DEVICE_HELP]	   = "help",
	[_HX170_DEVICE_PREPARE]	   = "prepare",
	[_HX170_DEVICE_COMPLETE]   = "complete",
	[_HX170_DEVICE_SUSPEND]	   = "suspend",
	[_HX170_DEVICE_RESUME]	   = "resume",
	[_HX170_DEVICE_ON]	   = "on",
	[_HX170_DEVICE_OFF]	   = "off",
	[_HX170_DEVICE_SET_PWR_ON] = "poweron",
	[_HX170_DEVICE_RESET]	   = "reset",
	[_HX170_DEVICE_STATE]	   = "state",
	[_HX170_DEVICE_COVERAGE]   = "coverage",
	[_HX170_DEVICE_PING]	   = "pingvm",
};

/*
 * internal functions
 */
static int hx170_print_state(char *p, ssize_t size);
static int hx170_set_power_on_handle(char *argp);


/*
 * open file operation
 */
int hx170_ctrlopen(struct inode *ip, struct file *fp)
{
	int ret = 0;

	pr_info("hx170dbg open(inode %p file %p) = %d\n",
		ip, fp, ret);

	return ret;
}

/*
 * release file operation
 */
int hx170_ctrlrelease(struct inode *ip, struct file *fp)
{
	int ret = 0;

	pr_info("hx170dbg close(inode %p file %p) = %d\n",
		ip, fp, ret);

	return ret;
}

/*
 * read file operation: provide information to user space
 */
static ssize_t hx170_ctrlread(struct file *fp, char __user *user_buffer,
	size_t count, loff_t *position)
{

	ssize_t ret = 0;

	pr_info("hx170dbg hx170_ctrlread(%p count %d pos %p)\n",
		user_buffer, count, position);

	ret = hx170_print_state(hx170_cmdBuffer, HX170_CMD_LEN);

	ret = simple_read_from_buffer(user_buffer,
		(ret <= count ? ret : count),
		position, hx170_cmdBuffer, HX170_CMD_LEN);

	pr_info("hx170dbg hx170_ctrlread(%p count %d pos %p) = %d\n",
		user_buffer, count, position, ret);

	return ret;
}

/*
 * write file operation: receive commands from user space
 */
static ssize_t hx170_ctrlwrite(struct file *fp, const char __user *user_buffer,
	size_t count, loff_t *position)
{
	int n = 0;
	ssize_t ret = 0;
	struct vpu_dec_device_t *vpu_dec_data = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = vpu_dec_data->pm_platdata;

	pr_info("hx170dbg hx170_pmwrite(%p len %d)\n",
		user_buffer, count);

	if (count >= HX170_CMD_LEN) {
		ret = -EINVAL;
		goto end;
	}

	if (dev == NULL) {
		pr_warn("hx170dbg device = 0; PM calls not possible\n");
		ret = 0;
		goto end;
	}

	if (pdev == NULL) {
		pr_warn("hx280dbg platform dev = 0; PM calls not possible\n");
		ret = 0;
		goto end;
	}

	if (dev_pm_ops == NULL) {
		pr_warn("hx170dbg dev_pm_ops = 0; PM calls not possible\n");
		ret = 0;
		goto end;
	}

	ret = simple_write_to_buffer(hx170_cmdBuffer, HX170_CMD_LEN, position,
		user_buffer, count);
	hx170_cmdBuffer[count] = 0;

	if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_HELP],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_HELP]))) {

		pr_info("hx170dbg command list:\n");
		for (n = 0; n < _HX170_MAX_CMDS; ++n)
			pr_info("	%s\n", hx170_ctrl_cmds[n]);

	} else if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_STATE],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_STATE]))) {
		/*
		 * report VPU state
		 */

		n = hx170_print_state(hx170_cmdBuffer, HX170_CMD_LEN);
		pr_info("%s", hx170_cmdBuffer);

	} else if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_SET_PWR_ON],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_SET_PWR_ON]))) {
		/*
		 * set power on handle
		 */
		int n;

		n = hx170_set_power_on_handle(hx170_cmdBuffer +
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_SET_PWR_ON]));

		pr_info("hx170dbg set power on handle %d\n", n);

	} else if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_RESET],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_RESET]))) {
		/*
		 * reset VPU state
		 */
		pr_info("hx170dbg reset vpu decoder\n");

		n = platform_device_pm_set_state_by_name(pdev,
					pm_platdata->pm_state_D3_name);


		hx170dec_reset_power_state();

		/* other things to reset?? */

		pr_info("hx170dbg vpu decoder OFF (%d)\n", n);

	} else if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_ON],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_ON]))) {
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
			hx170dec_power_state(1);

		pr_info("hx170dbg vpu decoder ON (%d)\n", n);

	} else if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_OFF],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_OFF]))) {
		/*
		 * switch off device unconditionally
		 */
		n = platform_device_pm_set_state_by_name(pdev,
					pm_platdata->pm_state_D3_name);

		if (n == 0)
			hx170dec_power_state(-1);

		pr_info("hx170dbg vpu decoder OFF (%d)\n", n);

	} else if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_PREPARE],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_PREPARE]))) {
		/*
		 * call prepare
		 */
		if (dev_pm_ops->prepare != NULL) {
			int r;

			pr_info("hx170dbg device PREPARE\n");
			r = dev_pm_ops->prepare(dev);
			pr_info("hx170dbg device PREPARE = %d\n", r);
		} else
			pr_info("hx170dbg device PREPARE not supported\n");


	} else if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_COMPLETE],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_COMPLETE]))) {
		/*
		 * call complete
		 */
		if (dev_pm_ops->complete != NULL) {

			pr_info("hx170dbg device COMPLETE\n");
			dev_pm_ops->complete(dev);
			pr_info("hx170dbg device COMPLETE = void\n");
		} else
			pr_info("hx170dbg device COMPLETE not supported\n");


	} else if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_SUSPEND],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_SUSPEND]))) {
		/*
		 * call suspend function
		 */
		if (dev_pm_ops->suspend != NULL) {
			int r;

			pr_info("hx170dbg device SUSPEND\n");
			r = dev_pm_ops->suspend(dev);
			pr_info("hx170dbg device SUSPEND = %d\n", r);
		} else
			pr_info("hx170dbg device SUSPEND not supported\n");

	} else if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_RESUME],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_RESUME]))) {
		/*
		 * call resume function
		 */
		if (dev_pm_ops->resume != NULL) {
			int r;

			pr_info("hx170dbg device RESUME\n");
			r = dev_pm_ops->resume(dev);
			pr_info("hx170dbg device RESUME = %d\n", r);
		} else
			pr_info("hx170dbg device RESUME not supported\n");

	} else if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_COVERAGE],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_COVERAGE]))) {
		/*
		 * call function for coverage testing
		 */
		pr_info("hx170dbg coverage testing ... TBD\n");

	} else if (!strncmp(hx170_cmdBuffer,
			hx170_ctrl_cmds[_HX170_DEVICE_PING],
			strlen(hx170_ctrl_cmds[_HX170_DEVICE_PING]))) {
		/*
		 * call function for pinging the secure vm
		 */
		pr_info("hx170dbg ping secure vm\n");
		vvpu_ping(dev, 0x0b53553d);

	} else {

		pr_info("hx170dbg command %s not supported\n",
			hx170_cmdBuffer);
	}

end:
	return ret;
}

static const struct file_operations fops_ctrl = {
	.open	 = hx170_ctrlopen,
	.release = hx170_ctrlrelease,
	.read	 = hx170_ctrlread,
	.write	 = hx170_ctrlwrite,
};

int hx170_init_debug(void)
{
	/* create a directory by the name dell in /sys/kernel/debugfs */
	hx170_dir = debugfs_create_dir(G1_DRIVER_NAME, NULL);

	/*
	   create ctrl node in the above directory
	   This requires read and write file operations
	*/
	hx170_ctrl = debugfs_create_file("ctrl", 0644, hx170_dir,
		&hx170_fileValue, &fops_ctrl);

	return 0;
}

int hx170_probe_debug(struct device *the_dev,
	struct platform_device	*the_p_dev,
	const struct dev_pm_ops *pm_ops)
{
	int ret = 0;

	dev	   = the_dev;
	pdev	   = the_p_dev;
	dev_pm_ops = pm_ops;

	pr_info("hx170dbg registered device and pm ops\n");

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

void hx170_release_debug(void)
{
	/* removing the directory recursively which
	   in turn cleans all the file */

	dev	   = NULL;
	dev_pm_ops = NULL;

	if (hx170_dir != NULL)
		debugfs_remove_recursive(hx170_dir);
}


/*
 * print/format status of VPU decoder
 */
static int hx170_print_state(char *p, ssize_t size)
{
	int n, i;
	ssize_t rest = size;

#define ADVANCE(j) do { p += (j); rest -= (j); } while (0)

	i = snprintf(p, rest, "hx170dbg build date: %s\n", hx170_build_date);
	ADVANCE(i);

	/*
	 * report VPU state
	 */
	i = snprintf(p, rest, "device state:\n");
	ADVANCE(i);

	i = snprintf(p, rest, " + vpu decoder is currently%s available",
		(hx170dec_pm_get_avail() ? "" : " not"));
	ADVANCE(i);

	n = hx170dec_power_state(0);
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
	((hdl) == (*xgold_vpu_dec_pm_state_on_p) ? " = power on" : "")

	ADVANCE(i);
	i = snprintf(p, rest, "	   + disable %p%s\n",
		xgold_vpu_dec_pm_state_disable,
		IS_POWER_ON_HDL(xgold_vpu_dec_pm_state_disable));
	ADVANCE(i);
	i = snprintf(p, rest, "	   + low %p%s\n",
		xgold_vpu_dec_pm_state_low_perf,
		IS_POWER_ON_HDL(xgold_vpu_dec_pm_state_low_perf));
	ADVANCE(i);
	i = snprintf(p, rest, "	   + mid %p%s\n",
		xgold_vpu_dec_pm_state_mid_perf,
		IS_POWER_ON_HDL(xgold_vpu_dec_pm_state_mid_perf));
	ADVANCE(i);
	i = snprintf(p, rest, "	   + high %p%s\n",
		xgold_vpu_dec_pm_state_high_perf,
		IS_POWER_ON_HDL(xgold_vpu_dec_pm_state_high_perf));
	ADVANCE(i);
	i = snprintf(p, rest, "	   + ultra high %p%s\n",
		xgold_vpu_dec_pm_state_ultra_high_perf,
		IS_POWER_ON_HDL(xgold_vpu_dec_pm_state_ultra_high_perf));
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
} hx170_power_on_handles[] = {
	{ "disable",
	  &xgold_vpu_dec_pm_state_disable,
	  "WARNING: disable handle!!" },
	{ "low",
	  &xgold_vpu_dec_pm_state_low_perf,
	  "low performance, 104 MHz, Bus >= 26 MHz" },
	{ "mid",
	  &xgold_vpu_dec_pm_state_mid_perf,
	  "mid performance, 156 MHz, Bus >= 26 MHz"},
	{ "high",
	  &xgold_vpu_dec_pm_state_high_perf,
	  "high performance, 312 MHz, Bus >= 26 MHz"},
#ifdef HX170_USE_ULTRA_HIGH_PERF
	{ "ultra",
	  &xgold_vpu_dec_pm_state_ultra_high_perf,
	  "ultra high performance, 420 MHz, Bus >= 26 MHz"},
#endif
	{ NULL, NULL, NULL }
};


/*
 * set power on handle
 */
static int hx170_set_power_on_handle(char *argp)
{
	int i;
	int ret = -1;

#define SKIP_BLANKS(p) { while (*(p) == ' ') (p)++; }

	SKIP_BLANKS(argp);

	for (i = 0; hx170_power_on_handles[i].name != NULL; i++) {
		if (!strncmp(argp, hx170_power_on_handles[i].name,
				strlen(hx170_power_on_handles[i].name))) {

			xgold_vpu_dec_pm_state_on_p
				= hx170_power_on_handles[i].p_handle;

			pr_info("hx170dbg set power on handle to %s\n",
				hx170_power_on_handles[i].name);
			pr_info("hx170dbg note: %s\n",
				hx170_power_on_handles[i].note);

			ret = i;
			break;
		}
	}

	if (ret < 0)
		pr_info("hx170dbg did not find power on handle for \"%s\"\n",
			argp);

	return ret;
}
