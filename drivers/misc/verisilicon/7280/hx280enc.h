/*
 * Copyright (C) 2013 - 2015 Intel Mobile Communications GmbH
 *
 * Notes:
 * May	6 2013: IMC: make platform device/driver
 * Jun	1 2013: IMC: moved ioctl definitions to hx280ioctl.h
 * Oct	9 2013: IMC: fix kernel code formatting issues
 * Nov 14 2013: IMC: abstraction for power-on handle in ES2.xx
 * Nov 27 2013: IMC: remove ultra high perf from all builds
 * Nov 29 2013: IMC: change string to register with
 *		     power management to "vpu_enc"
 * Mar 13 2014: IMC: Review Comments & Clean up
 * May 19 2014: IMC: reset vpu when switching between decoder and encoder
 * Mar 16 2015: IMC: VVPU only: remove code accessing HW
 */

/*
 * Encoder device driver (kernel module header)
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
 *					 Boston, MA  02110-1301, USA.
 *
 ----------------------------------------------------------------------------
 --
 --  Version control information, please leave untouched.
 --
 --  $RCSfile: hx280enc.h,v $
 --  $Date: 2011/03/10 14:05:43 $
 --  $Revision: 1.1 $
 --
 --------------------------------------------------------------------------*/

#ifndef _HX280ENC_H_
#define _HX280ENC_H_
#include <linux/ioctl.h>    /* needed for the _IOW etc stuff used later */

#include <linux/pm.h>
#include <linux/platform_device.h>

#include <linux/platform_device_pm.h>

#include <linux/semaphore.h>
#include <linux/reset.h>

/*
 * define to enable ultra high perf
 */
#undef HX280_USE_ULTRA_HIGH_PERF


/*
 * Macros to help debugging
 */

/** TODO: enable temp only **/
/*** #define HX280ENC_DUMP_REGS ***/

/*
 * define HX280ENC_DEBUG for verbose tracing
 * but be aware it creates load!
 */
/** #define HX280ENC_DEBUG **/
#ifdef HX280ENC_DEBUG
/* This one if debugging is on, and kernel space */
#define HX280_DEBUG(fmt, args...) pr_debug("hx280enc: " fmt, ## args)
#define HX280_ENTER()	pr_debug(">> %s()\n", __func__)
#define HX280_LEAVE(r)	pr_debug("<< %s() = %d\n", __func__, (int)(r))
#else
#define HX280_DEBUG(fmt, args...)
#define HX280_ENTER()
#define HX280_LEAVE(r)
#endif

#define H1_DRIVER_NAME		"vpu-enc"
#define H1_RESET_NAME		"vpu_enc"
#define OF_KERNEL_CLK		"clk_kernel"
#define OF_SLAVE_CLK		"clk_slave"
#define OF_MASTER_CLK		"clk_master"
#define OF_CORE_REG		"video"

#define ENC_HW_ID1		0x62800000
#define ENC_HW_ID2		0x72800000
#define ENC_HW_ID3		0x82700000

#if 0
struct vpu_enc_resource {
	unsigned pbase;
	unsigned size;
	phys_addr_t vbase;
};

struct vpu_enc_debug_t {
	int showfps;
	long long delay_max;	    /* max delay in us	 */
	struct timeval cmd_delay;   /* cmd delay measure */
	int level;
	unsigned int frame_update_number;
};

#define NB_MEAS 20
struct vpu_enc_meas_t {
	unsigned max;
	unsigned idx;
	struct timeval t[NB_MEAS];
	unsigned diffms[NB_MEAS];
};

struct vpu_enc_irq {
	int enc;
};

#endif

struct vpu_enc_device_t {
	struct platform_device *pdev;
	struct device *dev;

	struct device_pm_platdata *pm_platdata;
#if 0
	struct vpu_enc_resource reg;  /* register memory desc		    */
	struct vpu_enc_resource mem;  /* dedicated memory desc		    */
	struct vpu_enc_irq irq;
	struct reset_control *rstc;   /* reference to reset controller	    */
#endif
};

#if 0
extern struct vpu_common_sem {
	struct semaphore vpu_sem;
	int vpu_sem_owner;
	int cur_user; /*0--> no owner, 1 --> decoder,PP, 2 --> encoder*/
} vpu_com_sema;
#endif

#if 0 /* currently unused */
static inline struct vpu_enc_device_t *vpu_enc_get_drvdata(
	struct platform_device *pdev)
{
	struct vpu_enc_device_t *data = (struct vpu_enc_device_t *)
		platform_get_drvdata(pdev);

	if (!data) {
		struct device *dev = &pdev->dev;
		dev_err(dev, "drvdata is not yet set\n");
	}

	return data;
}
#endif

/* power management state handles ----------------------------------- */
extern struct platform_device_pm_state *xgold_vpu_enc_pm_state_disable;
extern struct platform_device_pm_state *xgold_vpu_enc_pm_state_low_perf;
extern struct platform_device_pm_state *xgold_vpu_enc_pm_state_mid_perf;
extern struct platform_device_pm_state *xgold_vpu_enc_pm_state_high_perf;
extern struct platform_device_pm_state *xgold_vpu_enc_pm_state_ultra_high_perf;

extern struct platform_device_pm_state **xgold_vpu_enc_pm_state_on_p;

/* power management call backs -------------------------------------- */
extern int xgold_vpu_enc_suspend(struct platform_device *pdev,
	pm_message_t state);
extern int xgold_vpu_enc_resume(struct platform_device *pdev);


/*
 * set/get availability
 */
extern void hx280enc_pm_set_avail(int avail);
extern	int hx280enc_pm_get_avail(void);

#ifdef CONFIG_VERISILICON_7280_DEBUG_FS
/*
 * debug file system
 */
extern int hx280_init_debug(void);
extern int hx280_probe_debug(struct device *the_dev,
	struct platform_device	*the_p_dev,
	const struct dev_pm_ops *pm_ops);
extern void hx280_release_debug(void);

extern	int hx280enc_power_state(int change);
extern void hx280enc_reset_power_state(void);

#endif /* CONFIG_VERISILICON_7280_DEBUG_FS */

#endif /* !_HX280ENC_H_ */
