/*
 * Copyright (C) 2013 - 2015 Intel Mobile Communications GmbH
 *
 * Notes:
 * Jan 15 2013: IMC: make it build
 * Feb 14 2013: IMC: rebase on Hantro GPL version
 * May 28 2013: IMC: misc device support
 * Jun	1 2013: IMC: stubs for PM ioctl
 * Sep	4 2013: IMC: implementation runtime PM
 * Oct	9 2013: IMC: fix kernel code formatting issues
 * Oct 24 2013: IMC: debugging for power management
 * Nov	8 2013: IMC: debug fs for hx280 encoder
 * Nov 14 2013: IMC: abstraction for power-on handle in ES2.xx
 * Nov 27 2013: IMC: remove ultra high perf from all builds
 * Nov 15 2013: IMC: read register before write not required for ES 2.xx
 * Mar 13 2014: IMC: Review Comments & Clean up
 * Apr 04 2014: IMC: Change decoder signal to SIGUSR2, Reset VPU
.*		     when changing from enc to dec
 *		     and vice versa.
 * Aug	7 2014: IMC: use pm handles to switch power
 * Aug 22 2014: IMC: introduce vvpu module
 * Sep	3 2014: IMC: rm CONFIG_PM_RUNTIME for ioctl power ops
 * Mar 16 2015: IMC: VVPU only: remove code accessing HW
 */

/*
 * Encoder device driver (kernel module)
 *
 * Copyright (C) 2011  On2 Technologies Finland Oy.
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
 *					  Boston, MA  02110-1301, USA.
 *
 ----------------------------------------------------------------------------
 --
 --
 --  Version control information, please leave untouched.
 --
 --  $RCSfile: hx280enc.c,v $
 --  $Date: 2011/03/10 14:05:42 $
 --  $Revision: 1.1 $
 --
 --------------------------------------------------------------------------*/

/*
 * debugging options:
 *  - power management: power always on
 *  - power management: desctivate power switching
 *
 * NOTE: set the same value in the hx170dec.c file!!!!!!!!!!!!!
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



#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
/* needed for __init,__exit directives */
#include <linux/init.h>
/* needed for remap_page_range
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

/* needed for virt_to_phys() */
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>

#include <asm/irq.h>

#include <linux/version.h>

/* our own stuff */
#include "hx280enc.h"
#include "hx280ioctl.h"

#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>

#include <linux/delay.h>

#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE)

#include <sofia/vvpu_vbpipe.h>

#ifdef CONFIG_X86_INTEL_SOFIA
#include <linux/xgold_noc.h>
#endif

#endif /* CONFIG_MOBILEVISOR_VDRIVER_PIPE */

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hantro Products Oy");
MODULE_DESCRIPTION("Hantro 6280/7280/8270 Encoder driver");


struct vpu_enc_device_t *vpu_enc_data;

/*
 * PM related data:
 * - show whether the device may be accessed from a PM POV
 *   (incl spin lock)
 * - semaphore to restrict access during suspend state
 */
static int hx280enc_pm_avail;
DEFINE_SPINLOCK(hx280enc_pm_avail_lock);

#ifdef CONFIG_VERISILICON_7280_DEBUG_FS
static const struct dev_pm_ops hx280enc_pm;
#endif

/*
 * switch device on and off
 */
static int hx280enc_dev_pm(struct device *dev, int state);

static u32 hx280enc_req_counter;
DEFINE_SEMAPHORE(hx280enc_req_counter_lock);

#ifdef CONFIG_PM_RUNTIME
static bool hx280enc_runtime_suspended;
#endif

#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE)
static int check_vvpu_hw_id(long int hardware);
#endif


/*
 * mis device registration stuff
 */
struct miscdevice hx280enc_miscdevice = { 0, };

static int hx280enc_miscdevice_register(struct platform_device *pdev);
static void hx280enc_miscdevice_unregister(void);


#ifdef CONFIG_PM_RUNTIME
/*
 * return number of current users of encoder units
 */
static int hx280enc_users(void)
{
	int owners = 0;

	HX280_ENTER();

	HX280_LEAVE(owners);

	return owners;
}
#endif



/*--------------------------------------------------------------------------
  Function name	  : hx280enc_ioctl
  Description	  : communication method to/from the user space

  Return type	  : int
  --------------------------------------------------------------------------*/
static long hx280enc_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	int err = 0;
	int ret = 0;

	struct vpu_enc_device_t *pdata = vpu_enc_data;
	struct device *dev = pdata->dev;
	struct device_pm_platdata *pm_platdata = pdata->pm_platdata;
	(void) pm_platdata; /* used for debug trace only */

	HX280_ENTER();
	HX280_DEBUG("ioctl magic 0x%08x cmd %d", (int)_IOC_TYPE(cmd),
		(int)_IOC_NR(cmd));

	/*
	 * extract the type and number bitfields, and don't encode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != HX280ENC_IOC_MAGIC) {
		ret = -ENOTTY;
		goto end;
	}

	if (_IOC_NR(cmd) > HX280ENC_IOC_MAXNR) {
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

	switch (cmd) {
	default:
		dev_err(dev, "ioctl %d cmd is not supported! - ignored", cmd);

		ret = -EFAULT;
		break;

	case HX280ENC_IOC_PM_DISABLE:
	{
		HX280_DEBUG("HX280ENC_IOC_PM_DISABLE");

#ifdef __SKIP_POWER_ON_OFF__

		dev_info(dev,
		"ioctl: switch power off ignored for testing purposes");

#else /* __SKIP_POWER_ON_OFF__ */

		if (down_interruptible(&hx280enc_req_counter_lock)) {
			ret = -ERESTARTSYS;
			goto end;
		}

		if (hx280enc_req_counter == 1) {
			err = device_state_pm_set_state(dev,
				pm_platdata->pm_states[PM_STATE_D3]);

			HX280_DEBUG("set state \"%s\"",
				pm_platdata->pm_state_D3_name);

			if (err < 0) {
				dev_err(dev,
					"HX280ENC_IOC_PM_DISABLE failed: %d",
					err);
				ret = -EAGAIN;
			} else
				hx280enc_req_counter--;
		} else
			if (hx280enc_req_counter > 0)
				hx280enc_req_counter--;

		up(&hx280enc_req_counter_lock);

#endif /* __SKIP_POWER_ON_OFF__ */
	}
	break;

	case HX280ENC_IOC_PM_ULTRA:
	{
		HX280_DEBUG("HX280ENC_IOC_PM_ULTRA");

#ifdef __SKIP_POWER_ON_OFF__

		dev_info(dev,
		"ioctl: switch power on ignored for testing purposes");

#else /* __SKIP_POWER_ON_OFF__ */

		if (down_interruptible(&hx280enc_req_counter_lock)) {
			ret = -ERESTARTSYS;
			goto end;
		}

		if (hx280enc_req_counter == 0) {
			err = device_state_pm_set_state(dev,
				pm_platdata->pm_states[PM_STATE_D0]);

			HX280_DEBUG("set state \"%s\"",
				pm_platdata->pm_state_D0_name);

			if (err < 0) {
				dev_err(dev,
					"HX280ENC_IOC_PM_ULTRA failed: %d",
					err);
				ret = -EAGAIN;
			} else
				hx280enc_req_counter++;
		} else
			hx280enc_req_counter++;

		xgold_noc_qos_set("VPU");
		up(&hx280enc_req_counter_lock);

#endif /* __SKIP_POWER_ON_OFF__ */
	}
	break;


	case HX280_IOCT_SECVM_CMD: {
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
	} /* switch(cmd) */

end:
	HX280_LEAVE(ret);

	return ret;
}

static int hx280enc_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	HX280_ENTER();

	/*
	 * cannot open if suspend/freeze is about to happen
	 */
	if (hx280enc_pm_get_avail() == 0) {
		pr_info("hx280enc_open() = EAGAIN; suspend was announced");

		ret = -EAGAIN;
	}


	HX280_DEBUG("open(%p) = %d", (void *)filp, ret);

	HX280_LEAVE(ret);

	return ret;
}


static int hx280enc_release(struct inode *inode, struct file *filp)
{
	HX280_ENTER();


	HX280_DEBUG("close(%p) = 0", (void *)filp);

	HX280_LEAVE(0);

	return 0;
}

/* VFS methods */
static const struct file_operations hx280enc_fops = {
	.open		= hx280enc_open,
	.release	= hx280enc_release,
	.unlocked_ioctl = hx280enc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= hx280enc_ioctl,
#endif
};


struct vpu_enc_pm_resources {
	struct regulator *reg_core;
	struct clk *clk_kernel;
	struct clk *clk_slave;
	struct clk *clk_master;
};

static struct of_device_id xgold_vpu_encoder_of_match[] = {
	{ .compatible = "intel,vpu_encoder",},
	{ },
};

#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int xgold_of_parse_pm_vpu_enc(struct platform_device *pdev)
{
	struct vpu_enc_device_t *pdata = platform_get_drvdata(pdev);
	pdata->pm_platdata->priv = NULL;
	return 0;
}
#else

/* VPU ENC PM states index */
#define VPU_ENC_PM_STATE_D0	3
#define VPU_ENC_PM_STATE_D3	0
static int xgold_vpu_enc_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	struct vpu_enc_device_t *vpu_enc_data = dev_get_drvdata(dev);
	int id = device_state_pm_get_state_id(dev, state->name);
	struct device_pm_platdata *pm_platdata;
	struct vpu_enc_pm_resources *pm_pdata;
	int ret = 0;

	if (!vpu_enc_data)
		return -EINVAL;
	if (!vpu_enc_data->pm_platdata)
		return -EINVAL;

	pm_platdata = vpu_enc_data->pm_platdata;
	pm_pdata = pm_platdata->priv;
	if (!pm_pdata)
		return -EINVAL;

	switch (id) {
	case VPU_ENC_PM_STATE_D0:
		clk_prepare_enable(pm_pdata->clk_kernel);
		clk_prepare_enable(pm_pdata->clk_slave);
		clk_prepare_enable(pm_pdata->clk_master);
		ret = regulator_enable(pm_pdata->reg_core);
		if (ret)
			dev_err(dev, "Error while enabling core regulator");
		break;
	case VPU_ENC_PM_STATE_D3:
		clk_disable_unprepare(pm_pdata->clk_kernel);
		clk_disable_unprepare(pm_pdata->clk_slave);
		clk_disable_unprepare(pm_pdata->clk_master);
		regulator_disable(pm_pdata->reg_core);
		break;
	default:
		return -EINVAL;
	}

	return 0;

}

static struct device_state_pm_state vpu_enc_pm_states[] = {
	{ .name = "disable", },
	{ .name = "low_perf", },
	{ .name = "mid_perf", },
	{ .name = "high_perf", },
	{ .name = "ultra_high_perf", },
};

static struct device_state_pm_state *xgold_vpu_enc_get_initial_state(
		struct device *dev)
{
	return &vpu_enc_pm_states[VPU_ENC_PM_STATE_D3];
}

/* USIF PM states & class & pm ops */
static struct device_state_pm_ops vpu_enc_pm_ops = {
	.set_state = xgold_vpu_enc_set_pm_state,
	.get_initial_state = xgold_vpu_enc_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(vpu_enc);

static int xgold_of_parse_pm_vpu_enc(struct platform_device *pdev)
{
	struct vpu_enc_device_t *pdata = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;
	struct vpu_enc_pm_resources *pm_pdata;

	if (!pdata)
		return -EINVAL;
	if (!pdata->pm_platdata)
		return -EINVAL;

	pm_pdata = kzalloc(sizeof(struct vpu_enc_pm_resources), GFP_KERNEL);
	if (!pm_pdata)
		return -ENOMEM;

	pdata->pm_platdata->priv = pm_pdata;

	pm_pdata->clk_master = of_clk_get_by_name(np, OF_MASTER_CLK);
	if (IS_ERR(pm_pdata->clk_master)) {
		pr_err("VPU ENC Clk %s not found", OF_MASTER_CLK);
		return -EINVAL;
	}

	pm_pdata->clk_slave = of_clk_get_by_name(np, OF_SLAVE_CLK);
	if (IS_ERR(pm_pdata->clk_slave)) {
		pr_err("VPU ENC Clk %s not found", OF_SLAVE_CLK);
		return -EINVAL;
	}

	pm_pdata->clk_kernel = of_clk_get_by_name(np, OF_KERNEL_CLK);
	if (IS_ERR(pm_pdata->clk_kernel)) {
		pr_err("VPU ENC Clk %s not found", OF_KERNEL_CLK);
		return -EINVAL;
	}

	pm_pdata->reg_core = regulator_get(&pdev->dev, OF_CORE_REG);
	if (IS_ERR(pm_pdata->reg_core)) {
		pr_err("VPU ENC can't get video supply");
		return -ENODEV;
	}

	return 0;
}
#endif

static int xgold_vpu_enc_parse_platform_data(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct vpu_enc_device_t *vpu_enc_data = platform_get_drvdata(pdev);
	struct device_pm_platdata *pm_platdata;
	int ret;

	if (!vpu_enc_data)
		return -EINVAL;

	pm_platdata  = of_device_state_pm_setup(np);
	if (!pm_platdata)
		return -EINVAL;

	vpu_enc_data->pm_platdata  = pm_platdata;

	ret = platform_device_pm_set_class(pdev, pm_platdata->pm_user_name);
	if (ret)
		return ret;

	return xgold_of_parse_pm_vpu_enc(pdev);
}

static int xgold_vpu_enc_probe(struct platform_device *pdev)
{
	int result = 0;
	int ret = 0;

	struct device *dev = &pdev->dev;
	struct device_pm_platdata *pm_platdata;

	dev_info(dev, "probing vvpu");

	/* Allocate driver data record */
	vpu_enc_data = kzalloc(sizeof(struct vpu_enc_device_t), GFP_KERNEL);

	if (vpu_enc_data == NULL)
		return -ENOMEM;

	/* link dev and pdev to vpu_dec_data */
	platform_set_drvdata(pdev, vpu_enc_data);
	vpu_enc_data->pdev = pdev;
	vpu_enc_data->dev  = dev;

	result = xgold_vpu_enc_parse_platform_data(pdev);
	if (result) {
		kfree(vpu_enc_data);

		return -EINVAL;
	}

	/*
	 * get handles for power management
	 */
	pm_platdata = vpu_enc_data->pm_platdata;
	if (pm_platdata != NULL) {
		pm_platdata->pm_states[PM_STATE_D3]  =
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
	 * switch on vpu encoder ...
	 */
	hx280enc_dev_pm(dev, 1);
	if (result) {
		dev_err(dev, "Error powering up the interface");
		BUG();
	}
#endif /* CONFIG_PM_RUNTIME */

#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE)
	/*
	 * initlialize secure VM decoder: init pipe to secure vm
	 */
	dev_info(dev, "init SecureVM encoder");

	result = vvpu_vbpipe_init(dev);

	/*
	 * during system boot the mvpipe may not yet be accessible
	 * therefore ignore an error and init the pipe
	 * the first time it is used
	 */
	if (result != 0) {
		dev_warn(dev, "mvpipe open is postponed");

		result = 0;
	} else {
		struct vvpu_secvm_cmd cmd;
		int vvpu_ret;

		dev_info(dev, "probing secureVM encoder");

		/* set up init probe command */
		memset(&cmd, 0, sizeof(cmd));

		cmd.payload[0] = VVPU_VTYPE_ENC;
		cmd.payload[1] = VVPU_VOP_INIT_PROBE;

		/* do command */
		vvpu_ret = vvpu_call(dev, &cmd);

		if (vvpu_ret == 0)
			dev_err(dev, "error probing secureVM encoder");
		else
			dev_info(dev, "found hw id 0x%08x",
				cmd.payload[4]);

		check_vvpu_hw_id(cmd.payload[4]);
	}

#else
	dev_err(dev, "MOBILEVISOR_VDRIVER_PIPE is not available");
	ret = -ENODEV

	goto end;
#endif

	result = hx280enc_miscdevice_register(pdev);
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
	dev_info(dev, "leave it on for the moment - testing only");

#else	/* __POWER_ON_AFTER_PROBE__ */
	hx280enc_dev_pm(dev, 0);

	dev_info(dev, "switch it off for the moment");

#endif	/* __POWER_ON_AFTER_PROBE__ */

#endif	/* CONFIG_PM_RUNTIME */

	if (ret != 0) {
		dev_err(dev, "vvpu probe error");

		/* release resources */
		if (vpu_enc_data != NULL)
			kfree(vpu_enc_data);

		hx280enc_miscdevice_unregister();

		ret = -ENODEV;
	} else {
		/* show that the device may be used */
		hx280enc_pm_set_avail(1);

#ifdef CONFIG_VERISILICON_7280_DEBUG_FS
		hx280_probe_debug(dev, pdev, &hx280enc_pm);
#endif

		dev_info(dev, "vvpu probe OK (%d)", ret);
	}

	return ret;
}

static int xgold_vpu_enc_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;

#ifdef CONFIG_PM_RUNTIME
	/*
	 * triggers pm_request_idle(dev) and return its result
	 */
	pm_runtime_put(dev);
#else	/* CONFIG_PM_RUNTIME */
	hx280enc_dev_pm(dev, 0);

	dev_info(dev, "switch it off");
#endif	/* CONFIG_PM_RUNTIME */

	/* deactivate runtime PM for the device */
	pm_runtime_disable(dev);

	/* show that the device may no longer be used */
	hx280enc_pm_set_avail(0);

#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE)
	vvpu_vbpipe_release(dev);
#endif

	return ret;
}


/*
 * handles for power management: used for switching the device on and off
 */
struct platform_device_pm_state *xgold_vpu_enc_pm_state_disable;
struct platform_device_pm_state *xgold_vpu_enc_pm_state_low_perf;
struct platform_device_pm_state *xgold_vpu_enc_pm_state_mid_perf;
struct platform_device_pm_state *xgold_vpu_enc_pm_state_high_perf;
struct platform_device_pm_state *xgold_vpu_enc_pm_state_ultra_high_perf;

/*
 * currently don't use ultra high perf
 * - set power on handle
 */
struct platform_device_pm_state **xgold_vpu_enc_pm_state_on_p =
#ifdef HX280_USE_ULTRA_HIGH_PERF
	&xgold_vpu_enc_pm_state_ultra_high_perf;
#else
	&xgold_vpu_enc_pm_state_high_perf;
#endif

/*
 * aux functions to set and query availability from PM POV
 */
void hx280enc_pm_set_avail(int avail)
{
	unsigned long flags;

	spin_lock_irqsave(&hx280enc_pm_avail_lock, flags);
	hx280enc_pm_avail = avail;
	spin_unlock_irqrestore(&hx280enc_pm_avail_lock, flags);
}

int hx280enc_pm_get_avail(void)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&hx280enc_pm_avail_lock, flags);
	ret = hx280enc_pm_avail;
	spin_unlock_irqrestore(&hx280enc_pm_avail_lock, flags);

	return ret;
}

#ifdef CONFIG_VERISILICON_7280_DEBUG_FS
/*
 * aux functions; determine/set power state
 */
int hx280enc_power_state(int change)
{
	int ret = 0;

	if (down_interruptible(&hx280enc_req_counter_lock))
		ret = -1;
	else {
		hx280enc_req_counter += change;
		ret = hx280enc_req_counter;
		up(&hx280enc_req_counter_lock);
	}

	return ret;
}

void hx280enc_reset_power_state(void)
{
	int ret = 0;

	if (down_interruptible(&hx280enc_req_counter_lock))
		ret = -1;
	else {
		hx280enc_req_counter = 0;
		up(&hx280enc_req_counter_lock);
	}
}

#endif /* CONFIG_VERISILICON_7280_DEBUG_FS */


/*
 * aux function; switch power, input is struct dev
 */
static int hx280enc_dev_pm(struct device *dev, int state)
{
	int ret = 0;

	struct device_pm_platdata *pm_platdata;
	struct device_state_pm_state *pm_state_h;
	const char *pm_state_s;
	struct vpu_enc_device_t *vpu_enc_data =
		(struct vpu_enc_device_t *)dev_get_drvdata(dev);

	HX280_ENTER();

	if (!vpu_enc_data)
		return -EINVAL;

	pm_platdata = vpu_enc_data->pm_platdata;
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
	HX280_DEBUG("set state \"%s\"", pm_state_s);

	if (ret < 0)
		dev_err(dev, "Device PM set state %s failed(%d)",
							pm_state_s, ret);
	else
		dev_info(dev, "switch power to \"%s\"", pm_state_s);

	HX280_LEAVE(ret);

	return ret;
}

#ifdef CONFIG_PM
/*
 * system sleep PM functions
 */

int hx280enc_prepare(struct device *dev)
{
	int ret = 0;

	HX280_ENTER();

	hx280enc_pm_set_avail(0);

	HX280_LEAVE(ret);

	return ret;
}

void hx280enc_complete(struct device *dev)
{
	HX280_ENTER();

	/*
	 * indicate, that device is available again
	 * (this might not be neccessary)
	 */
	hx280enc_pm_set_avail(1);

	HX280_LEAVE(0);
}

int hx280enc_suspend(struct device *dev)
{
#ifndef CONFIG_PM_RUNTIME
	struct device_pm_platdata *pm_platdata;
	struct vpu_enc_device_t *vpu_enc_data =
		(struct vpu_enc_device_t *)dev_get_drvdata(dev);
#endif
	int ret = 0;

	HX280_ENTER();

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev)) {
		ret = pm_generic_runtime_suspend(dev);
		hx280enc_runtime_suspended = true;
	}
#else

	if (!vpu_enc_data)
		return -EINVAL;

	pm_platdata = vpu_enc_data->pm_platdata;
	if (!pm_platdata)
		return -EINVAL;

	if (down_interruptible(&hx280enc_req_counter_lock))
		ret = -ERESTARTSYS;
	else {
		/*
		 * if the power is on; switch it off, but don't change
		 * the counter; it will be used in resume
		 */
		if (hx280enc_req_counter > 0) {
			int err = device_state_pm_set_state(dev,
				pm_platdata->pm_states[PM_STATE_D3]);

			HX280_DEBUG("suspend: set state \"%s\"",
					pm_platdata->pm_state_D3_name);

			if (err < 0) {
				dev_err(dev,
					"set state %s failed (%d)",
					pm_platdata->pm_state_D3_name, err);
				ret = -EAGAIN;
			}
		}

		up(&hx280enc_req_counter_lock);
	}
#endif

	HX280_LEAVE(ret);

	return ret;
}

int hx280enc_resume(struct device *dev)
{
#ifndef CONFIG_PM_RUNTIME
	struct device_pm_platdata *pm_platdata;
	struct vpu_enc_device_t *vpu_enc_data =
		(struct vpu_enc_device_t *)dev_get_drvdata(dev);
#endif
	int ret = 0;

	HX280_ENTER();

#ifdef CONFIG_PM_RUNTIME
	if (hx280enc_runtime_suspended) {
		ret = pm_generic_runtime_resume(dev);
		hx280enc_runtime_suspended = false;
	}
	if (!ret)
		pm_runtime_enable(dev);
#else
	if (!vpu_enc_data)
		return -EINVAL;

	pm_platdata = vpu_enc_data->pm_platdata;
	if (!pm_platdata)
		return -EINVAL;

	if (down_interruptible(&hx280enc_req_counter_lock))
		ret = -ERESTARTSYS;
	else {
		/*
		 * if the power was on; switch it oon, but don't change
		 * the counter
		 */
		if (hx280enc_req_counter > 0) {
			int err = device_state_pm_set_state(dev,
				pm_platdata->pm_states[PM_STATE_D0]);

			HX280_DEBUG("resume: set state \"%s\"",
					pm_platdata->pm_state_D0_name);

			if (err < 0) {
				dev_err(dev,
					"set state %s failed (%d)",
					pm_platdata->pm_state_D0_name, err);
				ret = -EAGAIN;
			}
		}

		up(&hx280enc_req_counter_lock);

	}
#endif

	HX280_LEAVE(ret);

	return ret;
}


int hx280enc_suspend_noirq(struct device *dev)
{
	int ret = 0;

	HX280_ENTER();

	HX280_LEAVE(ret);

	return ret;
}

int hx280enc_resume_noirq(struct device *dev)
{
	int ret = 0;

	HX280_ENTER();

	HX280_LEAVE(ret);

	return ret;
}
#endif	/* CONFIG_PM */


#ifdef CONFIG_PM_RUNTIME
/*
 * runtime PM functions
 */

int hx280enc_runtime_suspend(struct device *dev)
{
	int ret = 0;

	HX280_ENTER();

#ifdef CONFIG_PM_RUNTIME
	dev_info(dev, "suspend: pm_runtime_put(%p)", (void *) dev);
	pm_runtime_put(dev);
#else
	/* switch off device */
	hx280enc_dev_pm(dev, 0);
#endif

	HX280_LEAVE(ret);

	return ret;
}

int hx280enc_runtime_resume(struct device *dev)
{
	int ret = 0;

	HX280_ENTER();

	/* switch on device */
	hx280enc_dev_pm(dev, 1);

	HX280_LEAVE(ret);

	return ret;
}

int hx280enc_runtime_idle(struct device *dev)
{
	int ret = 0;
	int owners = hx280enc_users();

	HX280_ENTER();

	/*
	 * cannot resume if encoder is in use
	 */
	if (owners != 0) {
		dev_info(dev, "runtime_idle(%p) %d users",
			(void *)dev, owners);
		ret = -EAGAIN;
	} else {
		pm_runtime_suspend(dev);
	}

	HX280_LEAVE(ret);

	return ret;
}

#endif /* CONFIG_PM_RUNTIME */

/*
 * device power management functions
 */
static const struct dev_pm_ops hx280enc_pm = {
#ifdef CONFIG_PM
	.prepare	 = hx280enc_prepare,
	.complete	 = hx280enc_complete,
	.suspend	 = hx280enc_suspend,
	.resume		 = hx280enc_resume,
	.freeze		 = 0,
	.thaw		 = 0,
	.poweroff	 = 0,
	.restore	 = 0,
	.suspend_noirq	 = hx280enc_suspend_noirq,
	.resume_noirq	 = hx280enc_resume_noirq,
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
	.runtime_suspend = hx280enc_runtime_suspend,
	.runtime_resume	 = hx280enc_runtime_resume,
	.runtime_idle	 = hx280enc_runtime_idle,
#else
	.runtime_suspend = 0,
	.runtime_resume	 = 0,
	.runtime_idle	 = 0,
#endif /* CONFIG_PM_RUNTIME */
};


/*
 * the platform driver
 */
static struct platform_driver xgold_vpu_enc = {
	.probe	  = xgold_vpu_enc_probe,
	.remove	  = xgold_vpu_enc_remove,
	.shutdown = NULL,
	.driver = {
		.name	= H1_DRIVER_NAME,
		.bus = &platform_bus_type,
		.owner =  THIS_MODULE,
		.pm    = &hx280enc_pm,
		.of_match_table = xgold_vpu_encoder_of_match,
	},
};

int __init hx280enc_init(void)
{
	int ret = 0;

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	ret = device_state_pm_add_class(&vpu_enc_pm_class);
	if (ret)
		return ret;
#endif

	/*
	 * register platform driver ..
	 */
	ret = platform_driver_register(&xgold_vpu_enc);
	if (ret) {
		pr_err("hx280enc:registering platform driver failed %d",
			ret);

		return -ENODEV;
	}

#ifdef CONFIG_VERISILICON_7280_DEBUG_FS
	hx280_init_debug();
#endif

	pr_info("hx280enc: registered platform driver");

	return ret;
}

void __exit hx280enc_cleanup(void)
{

#ifdef CONFIG_VERISILICON_7280_DEBUG_FS
	hx280_release_debug();
#endif

	hx280enc_miscdevice_unregister();

	HX280_DEBUG("unregistered device");
	return;
}

module_init(hx280enc_init);
module_exit(hx280enc_cleanup);


#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE)
static int check_vvpu_hw_id(long int hwid)
{
	int ret = 0;

	HX280_DEBUG("HW ID=0x%08lx", hwid);

	/* check for encoder HW ID */
	if ((((hwid >> 16)&0xFFFF) != ((ENC_HW_ID1 >> 16)&0xFFFF)) &&
		(((hwid >> 16)&0xFFFF) != ((ENC_HW_ID2 >> 16)&0xFFFF)) &&
		(((hwid >> 16)&0xFFFF) != ((ENC_HW_ID3 >> 16)&0xFFFF))) {

		pr_warn("hx170dec: No Compatible HW found 0x%08lx",
			hwid);

		ret = -EBUSY;
	}

	return ret;
}
#endif


static int hx280enc_miscdevice_register(struct platform_device *pdev)
{
	int err;

	hx280enc_miscdevice.minor  = MISC_DYNAMIC_MINOR;
	hx280enc_miscdevice.name   = H1_DRIVER_NAME;
	hx280enc_miscdevice.fops   = &hx280enc_fops;
	hx280enc_miscdevice.parent = get_device(&pdev->dev);

	err = misc_register(&hx280enc_miscdevice);
	if (0 != err)
		pr_err("hx280enc: error %d registering misc device", err);

	return err;
}


static void hx280enc_miscdevice_unregister(void)
{
	misc_deregister(&hx280enc_miscdevice);
}
