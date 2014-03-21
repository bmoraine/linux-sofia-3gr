/*
 * Copyright (C) 2013, 2014 Intel Mobile Communications GmbH
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
/* #define HX170DEC_DEBUG */
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

struct vpu_dec_resource {
	unsigned pbase;
	unsigned size;
	void *vbase;
};

struct vpu_dec_debug_t {
	int showfps;
	long long delay_max;	   /* max delay in us */
	struct timeval cmd_delay;  /*cmd delay measure */
	int level;
	unsigned int frame_update_number;
};

#define NB_MEAS 20
struct vpu_dec_meas_t {
	unsigned max;
	unsigned idx;
	struct timeval t[NB_MEAS];
	unsigned diffms[NB_MEAS];
};

struct vpu_dec_irq {
	int dec;
};

struct vpu_dec_device_t {
	struct platform_device *pdev;
	struct device *dev;		/* TODO: skip this one??	  */
	struct device_pm_platdata *pm_platdata;
	struct vpu_dec_resource reg;	/* register memory desc		  */
	struct vpu_dec_resource mem;	/* dedicated memory desc	  */
	struct vpu_dec_irq irq;
	struct reset_control *rstc;
};


#if 0 /* currently unused */
static inline struct vpu_dec_device_t *vpu_dec_get_drvdata(
	struct platform_device *pdev)
{
	struct vpu_dec_device_t *data =
		(struct vpu_dec_device_t *)platform_get_drvdata(pdev);

	if (!data) {
		struct device *dev = &pdev->dev;
		dev_err(dev, "drvdata is not yet set\n");
	}

	return data;
}
#endif


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
extern	int hx170dec_freeze(struct device *dev);
extern	int hx170dec_freeze(struct device *dev);
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

extern unsigned int hx170dec_n_dec_irq;
extern unsigned int hx170dec_n_pp_irq;
extern unsigned int hx170dec_n_spurious_irq;

#endif /* CONFIG_VERISILICON_G1_DEBUG_FS */

#endif /* !_HX170DEC_H_ */
