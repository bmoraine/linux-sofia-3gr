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
 *                   - Cleanup Mali PMU function calls
 * Jul 16 2014: IMC: [OSS Scan] Add missing license type
 * Jun 02 2014: IMC: Add pm and debugfs support
 *                   Splitup platform adaption for better readability
 */

#if !defined(CONFIG_PLATFORM_DEVICE_PM)
/* To limit the number of ifdefs we always require CONFIG_PLATFORM_DEVICE_PM */
#error "XGold platform adaption for MALI400 requires CONFIG_PLATFORM_DEVICE_PM!"
#endif


#define MALI_PLF_NAME "Mali Platform"
#define mali_err(fmt, arg...)	pr_err(MALI_PLF_NAME" [ERROR]: " fmt, ##arg)
#define mali_info(fmt, arg...)	pr_info(MALI_PLF_NAME": " fmt, ##arg)
#define mali_warn(fmt, arg...)	pr_warn(MALI_PLF_NAME" [W]: " fmt, ##arg)
#define mali_dbg(fmt, arg...)	pr_debug(MALI_PLF_NAME" [D]: " fmt, ##arg)

/* ToDo: Should we get this in probe from DTS? */
#undef GPU_USE_ULTRA_HIGH_PERF /* Enable to use ultra_high_perf mode */
#if defined(GPU_USE_ULTRA_HIGH_PERF)
#define GPU_NUM_PM_STATES 5
#else
#define GPU_NUM_PM_STATES 4
#endif /* defined(GPU_USE_ULTRA_HIGH_PERF) */
#define GPU_MIN_PM_STATE 1
#define GPU_MAX_PM_STATE (GPU_NUM_PM_STATES - 1)
#define GPU_INITIAL_PM_STATE GPU_MAX_PM_STATE

/* PM states index */
#define MALI_PLF_PM_STATE_D3	0
#define MALI_PLF_PM_STATE_D0	GPU_MAX_PM_STATE


struct mali_platform_data {
#if !defined(CONFIG_PLATFORM_DEVICE_PM_VIRT)
	/* Directly access regulator and clock only on non virtualized kernel */
	struct regulator *regul;
	struct clk *clk_kernel;
	struct clk *clk_ahb;
#endif
	struct mali_gpu_device_data *gpu_data;
	struct device_pm_platdata *pm_platdata;
};

struct mali_platform_pm {
	struct platform_device_pm_state *pm_states[GPU_NUM_PM_STATES];
	unsigned int curr_pm_state;
	unsigned int resume_pm_state;
	struct workqueue_struct *dvfs_wq;
	struct mali_dev_dvfs_work_t *dvfs_work;
	bool dvfs_off;
	int req_clock_index;
	struct platform_device *pdev;
};

