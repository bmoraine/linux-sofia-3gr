/*
 * Copyright (C) 2013 - 2015 Intel Mobile Communications GmbH
 *
 * Notes:
 * Jan 15 2013: IMC: make it build
 * Feb 14 2013: IMC: rebase on Hantro's GPL version
 * May 23 2013: IMC: clean up things
 * May 28 2013: IMC: misc device support
 * Jun	1 2013: IMC: stubs for PM ioctl
 * Aug 29 2013: IMC: semaphore and ioctl for reserve/release HW
 * Sep	4 2013: IMC: implementation runtime PM
 * Oct	9 2013: IMC: fix kernel code formatting issues
 * Oct 24 2013: IMC: debugging for power management
 * Nov	5 2013: IMC: debug fs for hx170dec driver
 * Nov 14 2013: IMC: abstraction for power on-handle in ES2.xx
 * Nov 27 2013: IMC: remove ultra high perf from all builds
 * Nov 15 2013: IMC: read register before write not required for ES 2.xx
 * Mar 13 2014: IMC: Review Comments & Clean up
 * Apr 04 2014: IMC: Change decoder signal to SIGUSR2, Reset VPU
.*		     when changing from enc to dec
 *		     and vice versa.
 * Aug	7 2014: IMC: use pm handles to switch power
 * Jul 17 2014: IMC: secure vm rpc sync command list with decoder lib
 * Jul 24 2014: IMC: probe device in secure vm via rpc
 * Aug 19 2014: IMC: replace vprc by vbpipe
 * Aug 19 2014: IMC: if IO address present use traditional driver,
 *		     vvpu otherwise
 * Aug 22 2014: IMC: separate vvpu module to share with encoder
 * Mar 16 2015: IMC: VVPU only: remove code accessing HW
 */

/*
 * Decoder device driver (kernel module)
 *
 * Copyright (C) 2012 Google Finland Oy.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

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
 --------------------------------------------------------------------------*/

/*
 * debugging options:
 *  - power management: power always on
 *  - power management: desctivate power switching
 *
 * NOTE: set the same value in the hx280enc.c file!!!!!!!!!!!!!
 */
/*
 * leave power on after probe
 */
/** #define __POWER_ON_AFTER_PROBE__ **/

/*
 * do not call PM susbsystem
 */
/** #define __SKIP_POWER_ON_OFF__ **/
#if (defined(__SKIP_POWER_ON_OFF__) && !defined(__POWER_ON_AFTER_PROBE__))
#define __POWER_ON_AFTER_PROBE__
#endif


#include <linux/version.h>

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
/* needed for __init,__exit directives */
#include <linux/init.h>
/* needed for remap_pfn_range
   SetPageReserved
   ClearPageReserved
*/
#include <linux/mm.h>
/* obviously, for kmalloc */
#include <linux/slab.h>
/* for struct file_operations, register_chrdev() */
#include <linux/fs.h>
/* standard error codes */
#include <linux/errno.h>

#include <linux/moduleparam.h>
/* request_irq(), free_irq() */
#include <linux/interrupt.h>

#include <linux/delay.h>

#include <asm/irq.h>


#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>

#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include <android/sync.h>
#include <android/sw_sync.h>
#include <linux/file.h>

/* our own stuff */
#include "hx170dec.h"
#include "hx170ioctl.h"

#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE)

#include <sofia/vvpu_vbpipe.h>

#ifdef CONFIG_X86_INTEL_SOFIA
#include <linux/xgold_noc.h>
#endif

#endif /* CONFIG_MOBILEVISOR_VDRIVER_PIPE */

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Google Finland Oy");
MODULE_DESCRIPTION("Driver module for 8170/8190/G1 Hantro decoder/pp");


struct vpu_dec_device_t *vpu_dec_data;

/* supported HW IDs */
static const int dec_hw_id[] = { 0x8190, 0x8170, 0x9170, 0x9190, 0x6731 };


/*
 * PM related data:
 * - show whether the device may be accessed from a PM POV
 *   (incl spin lock)
 * - semaphore to restrict access during suspend state
 */
static int hx170dec_pm_avail;
DEFINE_SPINLOCK(hx170dec_pm_avail_lock);

#ifdef CONFIG_VERISILICON_G1_DEBUG_FS
static const struct dev_pm_ops hx170dec_pm;
#endif

/*
 * switch device on and off
 */
static int hx170dec_dev_pm(struct device *dev, int state);

static u32 hx170dec_req_counter;
DEFINE_SEMAPHORE(hx170dec_req_counter_lock);

#ifdef CONFIG_PM_RUNTIME
static bool hx170dec_runtime_suspended;
#endif


#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE)
static int check_vvpu_hw_id(long int hardware);
#endif

/*
 * misc device registration
 */
struct miscdevice hx170dec_miscdevice = { 0, };

static int hx170dec_miscdevice_register(struct platform_device *pdev);
static void hx170dec_miscdevice_unregister(void);


#ifdef CONFIG_PM_RUNTIME
/*
 * return number of current users of decoder and post processor
 */
static int hx170dec_users(void)
{
	int owners = 0;

	HX170_ENTER();

	HX170_LEAVE(owners);

	return owners;
}
#endif


#if defined(CONFIG_SW_SYNC_USER)
static int pphwc_fence_init(struct vpu_dec_device_t *pdata, void *instance)
{
	if (!pdata->pphwc_timeline) {
		pdata->pphwc_timeline = sw_sync_timeline_create("pphwc");
		if (!pdata->pphwc_timeline) {
			dev_err(pdata->dev,
				"%s: cannot create time line", __func__);
			return -ENOMEM;
		}
	}
	return 0;
}

static void pphwc_fence_release(struct vpu_dec_device_t *pdata,
	uint64_t instance)
{
	if (vpu_dec_data->pphwc_timeline) {
		sync_timeline_destroy(&vpu_dec_data->pphwc_timeline->obj);
		vpu_dec_data->pphwc_timeline = NULL;
	}
}

static void pphwc_fence_update(struct vpu_dec_device_t *pdata,
		struct pphwc_cmd *cmd)
{
	if (!pdata->pphwc_timeline)
		return;
	/* do not increase if the timeline is already past the given value */
	if (cmd->sync_value > pdata->pphwc_timeline->value) {
		sw_sync_timeline_inc(pdata->pphwc_timeline,
			cmd->sync_value - pdata->pphwc_timeline->value);
	}
}

static void pphwc_fence_create(struct vpu_dec_device_t *pdata,
		struct pphwc_cmd *cmd)
{
	struct sync_pt *point;
	struct sync_fence *fence = NULL;
	int fd = -1;

	cmd->release_fence_fd = -1;
	if (!pdata->pphwc_timeline)
		return;

	/* Create sync point */
	point = sw_sync_pt_create(pdata->pphwc_timeline,
			pdata->pphwc_timeline->value + 1);
	if (point == NULL) {
		cmd->release_fence_fd = -EINVAL;
		return;
	}

	/* Create fence */
	fence = sync_fence_create("dcc-fence", point);
	if (fence == NULL) {
		sync_pt_free(point);
		cmd->release_fence_fd = -EINVAL;
		return;
	}

	/* Create fd */
	fd = get_unused_fd();
	if (fd < 0) {
		dev_err(pdata->dev, "fence_fd not initialized\n");
		sync_fence_put(fence);
		sync_pt_free(point);
		cmd->release_fence_fd = -EINVAL;
		return;
	}

	/* Finally install the fence to that file descriptor */
	sync_fence_install(fence, fd);
	cmd->sync_value = pdata->pphwc_timeline->value + 1;
	cmd->release_fence_fd = fd;
}
#endif

/*--------------------------------------------------------------------------
  Function name	  : hx170dec_ioctl
  Description	  : communication method to/from the user space

  Return type	  : int
  --------------------------------------------------------------------------*/
static long hx170dec_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int ret = 0;

	struct vpu_dec_device_t *pdata = vpu_dec_data;
	struct device *dev = pdata->dev;
	struct device_pm_platdata *pm_platdata = pdata->pm_platdata;
	(void) pm_platdata; /* used for debug trace only */

	HX170_ENTER();
	HX170_DEBUG("ioctl magic 0x%08x cmd %d", (int)_IOC_TYPE(cmd),
		(int)_IOC_NR(cmd));

	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != HX170DEC_IOC_MAGIC) {
		ret = -ENOTTY;
		goto end;
	}

	if (_IOC_NR(cmd) > HX170DEC_IOC_MAXNR) {
		ret = -ENOTTY;
		goto end;
	}

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void *) arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void *) arg, _IOC_SIZE(cmd));

	if (err) {
		ret = -EFAULT;
		goto end;
	}

	HX170_DEBUG("ioctl cmd 0x%08ux params OK", cmd);

	switch (cmd) {
	default:
		dev_err(dev, "ioctl %d cmd is not supported! - ignored", cmd);
		break;

	case HX170DEC_IOC_PM_DISABLE:
	{
		HX170_DEBUG("HX170DEC_IOC_PM_DISABLE");
#ifdef __SKIP_POWER_ON_OFF__

		dev_info(dev,
		"ioctl: switch power off ignored for testing purposes");

#else /* __SKIP_POWER_ON_OFF__ */

		if (down_interruptible(&hx170dec_req_counter_lock)) {
			ret = -ERESTARTSYS;
			goto end;
		}

		if (hx170dec_req_counter == 1) {
			err = device_state_pm_set_state(dev,
				pm_platdata->pm_states[PM_STATE_D3]);

			HX170_DEBUG("set state \"%s\"",
				pm_platdata->pm_state_D3_name);

			if (err < 0) {
				dev_err(dev,
				"HX170DEC_IOC_PM_DISABLE failed:%d",
					err);
				ret = EAGAIN;
			} else
				hx170dec_req_counter--;
		} else
			if (hx170dec_req_counter > 0)
				hx170dec_req_counter--;

		up(&hx170dec_req_counter_lock);

#endif /* __SKIP_POWER_ON_OFF__ */
	}
	break;

	case HX170DEC_IOC_PM_ULTRA:
	{
		HX170_DEBUG("HX170DEC_IOC_PM_ULTRA");
#ifdef __SKIP_POWER_ON_OFF__

		dev_info(dev,
		"ioctl: switch power on ignored for testing purposes");

#else /* __SKIP_POWER_ON_OFF__ */

		if (down_interruptible(&hx170dec_req_counter_lock)) {
			ret = -ERESTARTSYS;
			goto end;
		}

		if (hx170dec_req_counter == 0) {
			err = device_state_pm_set_state(dev,
				pm_platdata->pm_states[PM_STATE_D0]);

			HX170_DEBUG("set state \"%s\"",
				pm_platdata->pm_state_D0_name);

			if (err < 0) {
				dev_err(dev,
				"HX170DEC_IOC_PM_ULTRA failed: %d",
					err);
				ret = EAGAIN;
			} else
				hx170dec_req_counter++;
		} else
			hx170dec_req_counter++;

		up(&hx170dec_req_counter_lock);

#endif /* __SKIP_POWER_ON_OFF__ */
	}
	break;

	case HX170DEC_IOCT_SECVM_CMD: {
#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE)
		/* IMC: send a VVPU command to secure VM */
		struct vvpu_secvm_cmd vvpu_cmd;

		if (copy_from_user(&vvpu_cmd, (void __user *)arg,
				sizeof(vvpu_cmd)))
			ret = -EFAULT;
		else {
			vvpu_call(dev, &vvpu_cmd);

			if (copy_to_user((void __user *)arg, &vvpu_cmd,
					sizeof(vvpu_cmd)))
				ret = -EFAULT;
		}
#else
		ret = -EFAULT;
#endif
		break;
	}

	case HX170DEC_IOCT_PPHWC_START: {
#if defined(CONFIG_SW_SYNC_USER)
		struct pphwc_cmd cmd;
		if (copy_from_user(&cmd, (void __user *)arg, sizeof(cmd))) {
			ret = -EFAULT;
			break;
		}
		/* set this as current instance, if it is not already */
		if (vpu_dec_data->ppwc_instance == 0)
			vpu_dec_data->ppwc_instance = cmd.instance;
		/* only one instance at a time */
		if (cmd.instance == 0 ||
			(cmd.instance != vpu_dec_data->ppwc_instance)) {
			ret = -EINVAL;
			dev_err(dev, "pphwc start err: %d", err);
			break;
		}
		pphwc_fence_create(vpu_dec_data, &cmd);
		/* return out values to user space */
		if (copy_to_user((void __user *)arg, &cmd, sizeof(cmd))) {
			ret = -EFAULT;
			break;
		}
#else
		ret = -EFAULT;
#endif
		break;
	}

	case HX170DEC_IOCT_PPHWC_DONE: {
#if defined(CONFIG_SW_SYNC_USER)
		struct pphwc_cmd cmd;
		if (copy_from_user(&cmd, (void __user *)arg, sizeof(cmd))) {
			ret = -EFAULT;
			break;
		}
		pphwc_fence_update(vpu_dec_data, &cmd);
#else
		ret = -EFAULT;
#endif
		break;
	}

	case HX170DEC_IOCT_PPHWC_RELEASE: {
#if defined(CONFIG_SW_SYNC_USER)
		struct pphwc_cmd cmd;
		/* if instance is not set yet, just ignore and return OK,
		 * it probably was never used */
		if (vpu_dec_data->ppwc_instance == 0)
			break;
		if (copy_from_user(&cmd, (void __user *)arg, sizeof(cmd))) {
			ret = -EFAULT;
			break;
		}
		if (cmd.instance == 0 ||
			(cmd.instance != vpu_dec_data->ppwc_instance)) {
			ret = -EINVAL;
			break;
		}
		vpu_dec_data->ppwc_instance = 0;
#else
		ret = -EFAULT;
#endif
		break;
	}
	}  /* switch(cmd) */
end:
	HX170_LEAVE(ret);

	return ret;
}

/*--------------------------------------------------------------------------
  Function name	  : hx170dec_open
  Description	  : open method

  Return type	  : int
  --------------------------------------------------------------------------*/

static int hx170dec_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	HX170_ENTER();

	/*
	 * cannot open if suspend/freeze is about to happen
	 */
	if (hx170dec_pm_get_avail() == 0) {
		pr_debug("hx170dec_open() = EAGAIN; suspend was announced");

		ret = -EAGAIN;
	}

	HX170_DEBUG("open(%p) = %d", (void *)filp, ret);

	HX170_LEAVE(ret);

	return ret;
}


/*---------------------------------------------------------------------------
  Function name	  : hx170dec_release
  Description	  : Release driver

  Return type	  : int
  ---------------------------------------------------------------------------*/

static int hx170dec_release(struct inode *inode, struct file *filp)
{
	HX170_ENTER();

	HX170_DEBUG("owners: dec %p pp %p", dec_owner, pp_owner);


	HX170_DEBUG("close(%p) = 0", (void *) filp);

	HX170_LEAVE(0);

	return 0;
}

/*
 * VFS methods
 */
static const struct file_operations hx170dec_fops = {
	.open		= hx170dec_open,
	.release	= hx170dec_release,
	.unlocked_ioctl = hx170dec_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= hx170dec_ioctl,
#endif
};


struct vpu_dec_pm_resources {
	struct regulator *reg_core;
	struct clk *clk_kernel;
	struct clk *clk_slave;
	struct clk *clk_master;
};

static struct of_device_id xgold_vpu_decoder_of_match[] = {
	{ .compatible = "intel,vpu_decoder",},
	{ },
};

#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
int xgold_of_parse_pm_vpu_dec(struct platform_device *pdev)
{
	struct vpu_dec_device_t *pdata = platform_get_drvdata(pdev);
	pdata->pm_platdata->priv = NULL;
	return 0;
}
#else

/* VPU ENC PM states index */
#define VPU_DEC_PM_STATE_D0	3
#define VPU_DEC_PM_STATE_D3	0
int xgold_vpu_dec_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	struct vpu_dec_device_t *vpu_dec_data = dev_get_drvdata(dev);
	int id = device_state_pm_get_state_id(dev, state->name);
	struct device_pm_platdata *pm_platdata;
	unsigned long rate = 0;
	struct vpu_dec_pm_resources *pm_pdata;
	int ret = 0;

	HX170_ENTER();

	if (!vpu_dec_data)
		return -EINVAL;
	if (!vpu_dec_data->pm_platdata)
		return -EINVAL;

	pm_platdata = vpu_dec_data->pm_platdata;
	pm_pdata = pm_platdata->priv;

	if (!pm_pdata)
		return -EINVAL;

	switch (id) {
	case VPU_DEC_PM_STATE_D0:
		clk_prepare_enable(pm_pdata->clk_kernel);
		rate = clk_get_rate(pm_pdata->clk_kernel);
		dev_err(dev, "kernel rate %ld",
				rate);

		clk_prepare_enable(pm_pdata->clk_slave);
		rate = clk_get_rate(pm_pdata->clk_slave);
		dev_err(dev, "slave rate %ld",
				rate);

		clk_prepare_enable(pm_pdata->clk_master);
		rate = clk_get_rate(pm_pdata->clk_master);
		dev_err(dev, "rate master %ld",
				rate);

		ret = regulator_enable(pm_pdata->reg_core);
		if (ret)
			dev_err(dev, "Error while enabling core regulator");

		break;
	case VPU_DEC_PM_STATE_D3:
		clk_disable_unprepare(pm_pdata->clk_kernel);
		clk_disable_unprepare(pm_pdata->clk_slave);
		clk_disable_unprepare(pm_pdata->clk_master);
		regulator_disable(pm_pdata->reg_core);
		break;
	default:
		return -EINVAL;
	}

	HX170_LEAVE(0);

	return 0;
}

struct device_state_pm_state vpu_dec_pm_states[] = {
	{ .name = "disable", },
	{ .name = "low_perf", },
	{ .name = "mid_perf", },
	{ .name = "high_perf", },
	{ .name = "ultra_high_perf", },
};

struct device_state_pm_state *xgold_vpu_dec_get_initial_state(
		struct device *dev)
{
	return &vpu_dec_pm_states[VPU_DEC_PM_STATE_D3];
}

/* USIF PM states & class & pm ops */
static struct device_state_pm_ops vpu_dec_pm_ops = {
	.set_state = xgold_vpu_dec_set_pm_state,
	.get_initial_state = xgold_vpu_dec_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(vpu_dec);

static int xgold_of_parse_pm_vpu_dec(struct platform_device *pdev)
{
	struct vpu_dec_device_t *pdata = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;
	struct vpu_dec_pm_resources *pm_pdata;

	if (!pdata)
		return -EINVAL;

	if (!pdata->pm_platdata)
		return -EINVAL;

	pm_pdata = kzalloc(sizeof(struct vpu_dec_pm_resources), GFP_KERNEL);
	if (!pm_pdata)
		return -ENOMEM;

	pdata->pm_platdata->priv = pm_pdata;

	pm_pdata->clk_master = of_clk_get_by_name(np, OF_MASTER_CLK);
	if (IS_ERR(pm_pdata->clk_master)) {
		pr_err("VPU DEC Clk %s not found", OF_MASTER_CLK);
		return -EINVAL;
	}

	pm_pdata->clk_slave = of_clk_get_by_name(np, OF_SLAVE_CLK);
	if (IS_ERR(pm_pdata->clk_slave)) {
		pr_err("VPU DEC Clk %s not found", OF_SLAVE_CLK);
		return -EINVAL;
	}

	pm_pdata->clk_kernel = of_clk_get_by_name(np, OF_KERNEL_CLK);
	if (IS_ERR(pm_pdata->clk_kernel)) {
		pr_err("VPU DEC Clk %s not found", OF_KERNEL_CLK);
		return -EINVAL;
	}

	pm_pdata->reg_core = regulator_get(&pdev->dev, OF_CORE_REG);
	if (IS_ERR(pm_pdata->reg_core)) {
		pr_err("VPU DEC can't get video supply");
		return -ENODEV;
	}

	return 0;
}
#endif

static int xgold_vpu_dec_parse_platform_data(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct vpu_dec_device_t *vpu_dec_data = platform_get_drvdata(pdev);
	struct device_pm_platdata *pm_platdata;
	int ret;

	if (!vpu_dec_data)
		return -EINVAL;

	pm_platdata  = of_device_state_pm_setup(np);
	if (!pm_platdata)
		return -EINVAL;

	vpu_dec_data->pm_platdata  = pm_platdata;

	ret = platform_device_pm_set_class(pdev, pm_platdata->pm_user_name);
	if (ret)
		return ret;

	return xgold_of_parse_pm_vpu_dec(pdev);
}

static int xgold_vpu_dec_probe(struct platform_device *pdev)
{
	int result = 0;
	int ret = 0;

	struct device *dev = &pdev->dev;
	struct device_pm_platdata *pm_platdata;

	dev_info(dev, "probing vvpu");

	/* Allocate driver data record */
	vpu_dec_data = kzalloc(sizeof(struct vpu_dec_device_t), GFP_KERNEL);

	if (vpu_dec_data == NULL)
		return -ENOMEM;

	/* link dev and pdev to vpu_dec_data */
	platform_set_drvdata(pdev, vpu_dec_data);
	vpu_dec_data->pdev = pdev;
	vpu_dec_data->dev  = dev;

	result = xgold_vpu_dec_parse_platform_data(pdev);
	if (result) {
		kfree(vpu_dec_data);

		return -EINVAL;
	}

	/*
	 * get handles for power management
	 */
	pm_platdata = vpu_dec_data->pm_platdata;
	if (pm_platdata != NULL) {
		pm_platdata->pm_states[PM_STATE_D3] =
			device_state_pm_get_state_handler(dev,
			pm_platdata->pm_state_D3_name);

		if (pm_platdata->pm_states[PM_STATE_D3] == NULL)
			dev_err(dev, "error getting D3 power handle");
		else
			dev_info(dev, "D3 state handle: %s",
				pm_platdata->pm_state_D3_name);

		pm_platdata->pm_states[PM_STATE_D0] =
			device_state_pm_get_state_handler(dev,
			pm_platdata->pm_state_D0_name);

		if (pm_platdata->pm_states[PM_STATE_D0] == NULL)
			dev_err(dev, "error getting D0 power handle");
		else
			dev_info(dev, "D0 state handle: %s",
				pm_platdata->pm_state_D0_name);

	} else
		dev_err(dev, "pm data is not present");


#ifdef CONFIG_PM_RUNTIME
	/*
	 * Runtime PM initialization
	 * Enable runtime PM helper functions to execute subsystem-level
	 */
	/* pm_runtime_enable(dev); */

	/*
	 * Increment the device's usage counter,
	 * triggers pm_runtime_resume(dev) and return its result
	 */
	pm_runtime_get_sync(dev);

	/*
	 * Do some other initialization.. if any ...
	 * call pm_runtime_put() put when done.
	 */
#else
	/*
	 * switch on vpu decoder ...
	 */
	result = hx170dec_dev_pm(dev, 1);
	if (result) {
		dev_err(dev, "Error powering up the interface");
		BUG();
	}
#endif /* CONFIG_PM_RUNTIME */

#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE)
	/*
	 * initlialize secure VM decoder: init pipe to secure vm
	 */
	dev_info(dev, "init SecureVM decoder");

	result = vvpu_vbpipe_init(dev);

	/*
	 * during system boot the vbpipe may not yet be accessible
	 * therefore ignore an error and init the pipe
	 * the first time it is used
	 */
	if (result != 0) {
		dev_warn(dev, "mvpipe open is postponed");

		result = 0;
	} else {
		struct vvpu_secvm_cmd cmd;
		int vvpu_ret;

		dev_info(dev, "probing secureVM decoder");

		/* set up init probe command */
		memset(&cmd, 0, sizeof(cmd));

		cmd.payload[0] = VVPU_VTYPE_DEC;
		cmd.payload[1] = VVPU_VOP_INIT_PROBE;

		/* do command */
		vvpu_ret = vvpu_call(dev, &cmd);

		if (vvpu_ret == 0)
			dev_err(dev, "error probing secureVM decoder");
		else
			dev_info(dev, "found hw id 0x%08x",
				cmd.payload[4]);

		check_vvpu_hw_id(cmd.payload[4]);
	}

#else
	dev_err(dev, "MOBILEVISOR_VDRIVER_PIPE is not available");
	ret = -ENODEV;

	goto end;
#endif

	result = hx170dec_miscdevice_register(pdev);
	if (result != 0) {
		ret = result;

		goto end;
	}

end:

#ifdef CONFIG_PM_RUNTIME
	/*
	 * triggers pm_request_idle(dev) and return its result
	 */
	pm_runtime_put(dev);
#else	/* CONFIG_PM_RUNTIME */

#ifdef __POWER_ON_AFTER_PROBE__

	/* this is for testing; leave the power on */
	dev_info(dev, "leave it on for the moment - tesing only");

#else	/* __POWER_ON_AFTER_PROBE__ */

	hx170dec_dev_pm(dev, 0);
	dev_info(dev, "switch it off for the moment");

#endif	/* __POWER_ON_AFTER_PROBE__ */

#endif	/* CONFIG_PM_RUNTIME */

	if (ret != 0) {
		dev_err(dev, "vvpu probe error");

		/* release resources */
		if (vpu_dec_data != NULL)
			kfree(vpu_dec_data);

		hx170dec_miscdevice_unregister();

		ret = -ENODEV;
	} else {
		/* show that the device may be used */
		hx170dec_pm_set_avail(1);

#ifdef CONFIG_VERISILICON_G1_DEBUG_FS
		hx170_probe_debug(dev, pdev, &hx170dec_pm);
#endif

		dev_info(dev, "vvpu probe OK (%d)", ret);
	}

#if defined(CONFIG_SW_SYNC_USER)
	pphwc_fence_init(vpu_dec_data, 0);
#endif

	return ret;
}


static int xgold_vpu_dec_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;

#ifdef CONFIG_PM_RUNTIME
	/*
	 * triggers pm_request_idle(dev) and return its result
	 */
	pm_runtime_put(dev);
#else	/* CONFIG_PM_RUNTIME */
	hx170dec_dev_pm(dev, 0);

	dev_info(dev, "switch it off");
#endif	/* CONFIG_PM_RUNTIME */

	/* deactivate runtime PM for the device */
	pm_runtime_disable(dev);

	/* show that the device may no longer be used */
	hx170dec_pm_set_avail(0);

#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE)
	vvpu_vbpipe_release(dev);
#endif

#if defined(CONFIG_SW_SYNC_USER)
	pphwc_fence_release(vpu_dec_data, 0);
#endif
	return ret;
}


/*
 * handles for power management: used for switching the device on and off
 */
struct platform_device_pm_state *xgold_vpu_dec_pm_state_disable;
struct platform_device_pm_state *xgold_vpu_dec_pm_state_low_perf;
struct platform_device_pm_state *xgold_vpu_dec_pm_state_mid_perf;
struct platform_device_pm_state *xgold_vpu_dec_pm_state_high_perf;
struct platform_device_pm_state *xgold_vpu_dec_pm_state_ultra_high_perf;

/*
 * currently don't use ultra high perf
 * - set power on handle
 */
struct platform_device_pm_state **xgold_vpu_dec_pm_state_on_p =
#ifdef HX170_USE_ULTRA_HIGH_PERF
	&xgold_vpu_dec_pm_state_ultra_high_perf;
#else
	&xgold_vpu_dec_pm_state_high_perf;
#endif


/*
 * aux functions to set and query availability from PM POV
 */
void hx170dec_pm_set_avail(int avail)
{
	unsigned long flags;

	spin_lock_irqsave(&hx170dec_pm_avail_lock, flags);
	hx170dec_pm_avail = avail;
	spin_unlock_irqrestore(&hx170dec_pm_avail_lock, flags);
}

int hx170dec_pm_get_avail(void)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&hx170dec_pm_avail_lock, flags);
	ret = hx170dec_pm_avail;
	spin_unlock_irqrestore(&hx170dec_pm_avail_lock, flags);

	return ret;
}


#ifdef CONFIG_VERISILICON_G1_DEBUG_FS
/*
 * aux functions; determine/reset power state
 */
int hx170dec_power_state(int change)
{
	int ret = 0;

	if (down_interruptible(&hx170dec_req_counter_lock))
		ret = -1;
	else {
		hx170dec_req_counter += change;
		ret = hx170dec_req_counter;
		up(&hx170dec_req_counter_lock);
	}

	return ret;
}

void hx170dec_reset_power_state(void)
{
	int ret = 0;

	if (down_interruptible(&hx170dec_req_counter_lock))
		ret = -1;
	else {
		hx170dec_req_counter = 0;
		up(&hx170dec_req_counter_lock);
	}
}

#endif /* CONFIG_VERISILICON_G1_DEBUG_FS */


/*
 * aux function; switch power, input is struct dev
 */
static int hx170dec_dev_pm(struct device *dev, int state)
{
	int ret = 0;

	struct device_pm_platdata *pm_platdata;
	struct device_state_pm_state *pm_state_h;
	const char *pm_state_s;
	struct vpu_dec_device_t *vpu_dec_data =
		(struct vpu_dec_device_t *)dev_get_drvdata(dev);

	HX170_ENTER();

	if (!vpu_dec_data)
		return -EINVAL;

	pm_platdata = vpu_dec_data->pm_platdata;
	if (!pm_platdata)
		return -EINVAL;

	if (state == 0) {
		pm_state_s = pm_platdata->pm_state_D3_name;
		pm_state_h = pm_platdata->pm_states[PM_STATE_D3];
	} else {
		pm_state_s = pm_platdata->pm_state_D0_name;
		pm_state_h = pm_platdata->pm_states[PM_STATE_D0];
	}

	/* switch on/off HW */
	ret = device_state_pm_set_state(dev, pm_state_h);
	HX170_DEBUG("set state \"%s\"", pm_state_s);

	if (ret < 0) {
		dev_err(dev, "Device PM set state %s failed(%d)",
			pm_state_s, ret);
	} else {
		dev_info(dev, "switch power to \"%s\"", pm_state_s);
	}

	HX170_LEAVE(ret);

	return ret;
}

#ifdef CONFIG_PM
/*
 * system sleep PM functions
 */

int hx170dec_prepare(struct device *dev)
{
	int ret = 0;

	HX170_ENTER();

	hx170dec_pm_set_avail(0);

	HX170_LEAVE(ret);

	return ret;
}

void hx170dec_complete(struct device *dev)
{

	HX170_ENTER();

	/*
	 * indicate, that device is available again
	 * (this might not be neccessary)
	 */
	hx170dec_pm_set_avail(1);

	HX170_LEAVE(0);
}

int hx170dec_suspend(struct device *dev)
{
#ifndef CONFIG_PM_RUNTIME
	struct device_pm_platdata *pm_platdata;
	struct vpu_dec_device_t *vpu_dec_data =
		(struct vpu_dec_device_t *)dev_get_drvdata(dev);
#endif
	int ret = 0;

	HX170_ENTER();


#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev)) {
		ret = pm_generic_runtime_suspend(dev);
		hx170dec_runtime_suspended = true;
	}
#else  /* CONFIG_PM_RUNTIME */

	if (!vpu_dec_data)
		return -EINVAL;

	pm_platdata = vpu_dec_data->pm_platdata;
	if (!pm_platdata)
		return -EINVAL;

	if (down_interruptible(&hx170dec_req_counter_lock))
		ret = -ERESTARTSYS;
	else {
		/*
		 * if the power is on; switch it off, but don't change
		 * the counter; it will be used in resume
		 */
		if (hx170dec_req_counter > 0) {
			int err = device_state_pm_set_state(dev,
				pm_platdata->pm_states[PM_STATE_D3]);

			HX170_DEBUG("suspend: set state \"%s\"",
					pm_platdata->pm_state_D3_name);

			if (err < 0) {
				dev_err(dev,
					"set state %s failed (%d)",
					pm_platdata->pm_state_D3_name, err);
				ret = -EAGAIN;
			}
		}

		up(&hx170dec_req_counter_lock);
	}
#endif	/* CONFIG_PM_RUNTIME */

	HX170_LEAVE(ret);

	return ret;
}

int hx170dec_resume(struct device *dev)
{
#ifndef CONFIG_PM_RUNTIME
	struct device_pm_platdata *pm_platdata;
	struct vpu_dec_device_t *vpu_dec_data =
		(struct vpu_dec_device_t *)dev_get_drvdata(dev);
#endif
	int ret = 0;

	HX170_ENTER();

#ifdef CONFIG_PM_RUNTIME
	if (hx170dec_runtime_suspended) {
		ret = pm_generic_runtime_resume(dev);
		hx170dec_runtime_suspended = false;
	}
	if (!ret)
		pm_runtime_enable(dev);
#else  /* CONFIG_PM_RUNTIME */
	if (!vpu_dec_data)
		return -EINVAL;

	pm_platdata = vpu_dec_data->pm_platdata;
	if (!pm_platdata)
		return -EINVAL;

	if (down_interruptible(&hx170dec_req_counter_lock))
		ret = -ERESTARTSYS;
	else {
		/*
		 * if the power was on; switch it oon, but don't change
		 * the counter
		 */

		if (hx170dec_req_counter > 0) {
			int err = device_state_pm_set_state(dev,
				pm_platdata->pm_states[PM_STATE_D0]);

			HX170_DEBUG("resume: set state \"%s\"",
					pm_platdata->pm_state_D0_name);

			if (err < 0) {
				dev_err(dev,
					"set state %s failed (%d)",
					pm_platdata->pm_state_D0_name, err);
				ret = -EAGAIN;
			}
			xgold_noc_qos_set("VPU");
		}
		up(&hx170dec_req_counter_lock);

	}
#endif	/* CONFIG_PM_RUNTIME */

	HX170_LEAVE(ret);

	return ret;
}


int hx170dec_suspend_noirq(struct device *dev)
{
	int ret = 0;

	HX170_ENTER();

	HX170_LEAVE(ret);

	return ret;
}

int hx170dec_resume_noirq(struct device *dev)
{
	int ret = 0;

	HX170_ENTER();

	HX170_LEAVE(ret);

	return ret;
}
#endif	/* CONFIG_PM */



#ifdef CONFIG_PM_RUNTIME
/*
 * runtime PM functions
 */

int hx170dec_runtime_suspend(struct device *dev)
{
	int ret = 0;

	HX170_ENTER();

	/* switch off device */
	ret = hx170dec_dev_pm(dev, 0);

	HX170_LEAVE(ret);

	return ret;
}

int hx170dec_runtime_resume(struct device *dev)
{
	int ret = 0;

	HX170_ENTER();

	dev_info(dev, "runtime_resume(%p)", (void *)dev);

	/* switch on device */
	ret = hx170dec_dev_pm(dev, 1);

	HX170_LEAVE(ret);

	return ret;
}

int hx170dec_runtime_idle(struct device *dev)
{
	int ret = 0;
	int owners = hx170dec_users();

	HX170_ENTER();

	/*
	 * cannot resume if dec and/or pp are in use
	 */
	if (owners != 0) {
		dev_info(dev, "runtime_idle(%p) %d users",
			(void *)dev, owners);
		ret = -EAGAIN;
	} else {
		pm_runtime_suspend(dev);
	}

	HX170_LEAVE(ret);

	return ret;
}

#endif /* CONFIG_PM_RUNTIME */


/*
 * device power management functions
 */
static const struct dev_pm_ops hx170dec_pm = {
#ifdef CONFIG_PM
	.prepare	 = hx170dec_prepare,
	.complete	 = hx170dec_complete,
	.suspend	 = hx170dec_suspend,
	.resume		 = hx170dec_resume,

	.freeze		 = 0,
	.thaw		 = 0,
	.poweroff	 = 0,
	.restore	 = 0,
	.suspend_noirq	 = hx170dec_suspend_noirq,
	.resume_noirq	 = hx170dec_resume_noirq,
	.freeze_noirq	 = 0,
	.thaw_noirq	 = 0,
	.poweroff_noirq	 = 0,
	.restore_noirq	 = 0,
#else
	.prepare	 = 0,
	.complete	 = 0,
	.suspend	 = 0,
	.resume		 = 0,
	.freeze		 = 0,
	.thaw		 = 0,
	.poweroff	 = 0,
	.restore	 = 0,
	.suspend_noirq	 = 0,
	.resume_noirq	 = 0,
	.freeze_noirq	 = 0,
	.thaw_noirq	 = 0,
	.poweroff_noirq	 = 0,
	.restore_noirq	 = 0,
#endif	/* CONFIG_PM */
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend = hx170dec_runtime_suspend,
	.runtime_resume	 = hx170dec_runtime_resume,
	.runtime_idle	 = hx170dec_runtime_idle,
#else
	.runtime_suspend = 0,
	.runtime_resume	 = 0,
	.runtime_idle	 = 0,
#endif /* CONFIG_PM_RUNTIME */
};

/*
 * the platform driver
 */
static struct platform_driver xgold_vpu_dec = {
	.probe	  = xgold_vpu_dec_probe,
	.remove	  = xgold_vpu_dec_remove,
	.shutdown = NULL,
	.driver = {
		.name	 = G1_DRIVER_NAME,
		.bus   = &platform_bus_type,
		.owner = THIS_MODULE,
		.pm    = &hx170dec_pm,
		.of_match_table = xgold_vpu_decoder_of_match,
	},
};

/*--------------------------------------------------------------------------
  Function name	  : hx170dec_init
  Description	  : Initialize the driver

  Return type	  : int
  --------------------------------------------------------------------------*/



int __init hx170dec_init(void)
{
	int ret = 0;

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	ret = device_state_pm_add_class(&vpu_dec_pm_class);
	if (ret)
		return ret;
#endif

	ret = platform_driver_register(&xgold_vpu_dec);
	if (ret) {
		pr_err("registering platform driver failed %d", ret);

		return -ENODEV;
	}

#ifdef CONFIG_VERISILICON_G1_DEBUG_FS
	hx170_init_debug();
#endif

	pr_info("hx170dec: registered platform driver");

	return ret;
}

/*--------------------------------------------------------------------------
  Function name	  : hx170dec_cleanup
  Description	  : clean up

  Return type	  : void
  --------------------------------------------------------------------------*/


void __exit hx170dec_cleanup(void)
{

#ifdef CONFIG_VERISILICON_G1_DEBUG_FS
	hx170_release_debug();
#endif

	hx170dec_miscdevice_unregister();

	pr_info("hx170dec: unregistered device");

	return;
}

module_init(hx170dec_init);
module_exit(hx170dec_cleanup);

#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE)
static int check_vvpu_hw_id(long int hardware)
{
	int ret = 1;
	long int hwid = hardware;

	size_t num_hw = sizeof(dec_hw_id) / sizeof(*dec_hw_id);

	HX170_DEBUG("VVPU HW ID=0x%08lx", hwid);

	hwid = (hwid >> 16) & 0xFFFF;	/* product version only */

	while (num_hw--) {
		if (hwid == dec_hw_id[num_hw]) {
			HX170_DEBUG("Compatible HW found 0x%08lx", hwid);

			ret = 0;
			break;
		}
	}

	if (ret != 0)
		pr_warn("hx170dec: No Compatible HW found 0x%08lx",
			hardware);

	return ret;
}
#endif

static int hx170dec_miscdevice_register(struct platform_device *pdev)
{
	int err;

	hx170dec_miscdevice.minor  = MISC_DYNAMIC_MINOR;
	hx170dec_miscdevice.name   = G1_DRIVER_NAME;
	hx170dec_miscdevice.fops   = &hx170dec_fops;
	hx170dec_miscdevice.parent = get_device(&pdev->dev);

	err = misc_register(&hx170dec_miscdevice);
	if (0 != err)
		pr_err("hx170dec: error %d registering misc device", err);

	return err;
}


static void hx170dec_miscdevice_unregister(void)
{
	misc_deregister(&hx170dec_miscdevice);
}
