/*
 * Copyright (C) 2013 - 2015 Intel Mobile Communications GmbH
 *
 * Notes:
 * May	2 2013: IMC: make platform device/driver
 * Jun	1 2013: IMC: moved ioctl definitions to hx170ioctl.h
 * Oct	9 2013: IMC: fix kernel code formatting issues
 * Nov	5 2013: IMC: debug fs for hx170dec driver
 * Nov 14 2013: IMC: abstraction for power-on handle in ES2.xx
 * Nov 27 2013: IMC: remove ultra high perf from all builds
 * Nov 29 2013: IMC: change string to register with
 *		     power management to "vpu_dec"
 * Mar 13 2014: IMC: Review Comments & Clean up
 * May 19 2014: IMC: reset vpu when switching between decoder and encoder
 * Aug 18 2014: IMC: support for vbpipe
 * Mar 16 2015: IMC: VVPU only: remove code accessing HW
 */

/*
 * Decoder device driver (kernel module headers)
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
 *					 Boston, MA  02110-1301, USA.
 *
 --------------------------------------------------------------------------*/

#ifndef _HX170DEC_H_
#define _HX170DEC_H_

#include <linux/ioctl.h>    /* needed for the _IOW etc stuff used later */
#include <linux/pm.h>
#include <linux/platform_device.h>

#include <linux/platform_device_pm.h>

#include <linux/semaphore.h>
#include <linux/reset.h>

/*
 * define to enable ultra high perf
 */
#undef HX170_USE_ULTRA_HIGH_PERF

/*
 * Macros to help debugging
 */

/** TODO: enable temp only **/
/*** #define HX170DEC_DUMP_REGS ***/

/*
 * define HX170DEC_DEBUG for verbose tracing
 * but be aware it creates load!
 */
/** #define HX170DEC_DEBUG **/
#ifdef HX170DEC_DEBUG
/* This one if debugging is on, and kernel space */
#define HX170_DEBUG(fmt, args...) pr_debug("hx170dec: " fmt, ## args)
#define HX170_ENTER()	pr_debug(">> %s()\n", __func__)
#define HX170_LEAVE(r)	pr_debug("<< %s() = %d\n", __func__, (int)(r))
#else
#define HX170_DEBUG(fmt, args...)
#define HX170_ENTER()
#define HX170_LEAVE(r)
#endif


#define G1_DRIVER_NAME		"vpu-dec"
#define G1_RESET_NAME		"vpu_dec"
#define OF_VPU_DEC_REG_SIZE	"reg"
#define OF_KERNEL_CLK		"clk_kernel"
#define OF_SLAVE_CLK		"clk_slave"
#define OF_MASTER_CLK		"clk_master"
#define PM_CLASS_VPU_DEC	"vpu_dec"


struct vpu_dec_device_t {
	struct platform_device *pdev;
	struct device *dev;		/* TODO: skip this one??	  */
	struct device_pm_platdata *pm_platdata;
#if 0
	struct reset_control *rstc;
#endif
#if defined(CONFIG_SW_SYNC_USER)
	struct sw_sync_timeline *pphwc_timeline;
	uint64_t ppwc_instance;
#endif
};



/* power management state handles ----------------------------------- */
extern struct platform_device_pm_state *xgold_vpu_dec_pm_state_disable;
extern struct platform_device_pm_state *xgold_vpu_dec_pm_state_low_perf;
extern struct platform_device_pm_state *xgold_vpu_dec_pm_state_mid_perf;
extern struct platform_device_pm_state *xgold_vpu_dec_pm_state_high_perf;
extern struct platform_device_pm_state *xgold_vpu_dec_pm_state_ultra_high_perf;

extern struct platform_device_pm_state **xgold_vpu_dec_pm_state_on_p;

/* power management call backs -------------------------------------- */
extern int xgold_vpu_dec_suspend(struct platform_device *pdev,
	pm_message_t state);
extern int xgold_vpu_dec_resume(struct platform_device *pdev);


/*
 * system sleep PM functions
 */
extern	int hx170dec_prepare(struct device *dev);
extern void hx170dec_complete(struct device *dev);
extern	int hx170dec_suspend(struct device *dev);
extern	int hx170dec_resume(struct device *dev);
extern	int hx170dec_suspend_noirq(struct device *dev);
extern	int hx170dec_resume_noirq(struct device *dev);


/*
 * runtime PM functions
 */
extern	int hx170dec_runtime_suspend(struct device *dev);
extern	int hx170dec_runtime_resume(struct device *dev);
extern	int hx170dec_runtime_idle(struct device *dev);

/*
 * set/get availability
 */
extern void hx170dec_pm_set_avail(int avail);
extern	int hx170dec_pm_get_avail(void);


#ifdef CONFIG_VERISILICON_G1_DEBUG_FS
/*
 * debug file system
 */
extern int hx170_init_debug(void);
extern int hx170_probe_debug(struct device *the_dev,
	struct platform_device	*the_p_dev,
	const struct dev_pm_ops *pm_ops);
extern void hx170_release_debug(void);

extern	int hx170dec_power_state(int change);
extern void hx170dec_reset_power_state(void);

#endif /* CONFIG_VERISILICON_G1_DEBUG_FS */

#endif /* !_HX170DEC_H_ */
