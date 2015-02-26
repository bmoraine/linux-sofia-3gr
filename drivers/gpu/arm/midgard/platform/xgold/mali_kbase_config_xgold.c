/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * Notes:
 * Dec 17 2014: IMC: Add initial xgold platform configuration
 */

/*
 *
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 */

#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#ifdef CONFIG_MALI_MIDGARD_RT_PM
#include <linux/pm_runtime.h>
#endif
#include <linux/xgold_noc.h>

#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config.h>
#include "mali_kbase_config_platform.h"
#include "mali_kbase_platform_xgold.h"

#ifdef CONFIG_MALI_MIDGARD_RT_PM
#define RUNTIME_PM_DELAY_TIME 100
#endif

#ifdef CONFIG_MALI_MIDGARD_RT_PM
static void pm_callback_power_off(struct kbase_device *kbdev)
{
	struct device *dev = kbdev->dev;

	mali_dbg("%s()\n", __func__);

	pm_schedule_suspend(dev, RUNTIME_PM_DELAY_TIME);
}

static int pm_callback_power_on(struct kbase_device *kbdev)
{
	int ret;
	int ret_val;
	struct xgold_platform_context *plf_context;
	struct device *dev = kbdev->dev;

	mali_dbg("%s()\n", __func__);

	if (pm_runtime_status_suspended(dev))
		ret_val = 1;
	else
		ret_val = 0;

	plf_context = (struct xgold_platform_context *) kbdev->platform_context;
	if (!plf_context) {
		mali_err("platform context is NULL\n");
		return MALI_FALSE;
	}


	if (dev->power.disable_depth > 0) {
		if (plf_context->curr_pm_state == MALI_PLF_PM_STATE_D3) {
			ret = kbase_platform_xgold_pm_control(kbdev,
						plf_context->resume_pm_state);
			if (ret < 0) {
				mali_err("%s (%d)\n",
					"kbase_platform_xgold_pm_control",
					ret);
				return ret;
			}
			ret_val = 1;
		}
		return ret_val;
	}

	ret = pm_runtime_resume(dev);

	if (ret < 0 && ret == -EAGAIN) {
		if (plf_context->curr_pm_state == MALI_PLF_PM_STATE_D3) {
			ret = kbase_platform_xgold_pm_control(kbdev,
						plf_context->resume_pm_state);
			if (ret < 0) {
				mali_err("%s (%d)\n",
					"kbase_platform_xgold_pm_control",
					ret);
				return ret;
			}
		}
	} else if (ret < 0)
		mali_err("pm_runtime_get_sync failed (%d)\n", ret);

	return ret_val;
}

static mali_error pm_callback_runtime_init(struct kbase_device *kbdev)
{
	mali_dbg("%s()\n", __func__);

	pm_suspend_ignore_children(kbdev->dev, true);
	pm_runtime_enable(kbdev->dev);

	return MALI_ERROR_NONE;
}

static void pm_callback_runtime_term(struct kbase_device *kbdev)
{
	mali_dbg("%s()\n", __func__);

	pm_runtime_disable(kbdev->dev);
}

static void pm_callback_runtime_off(struct kbase_device *kbdev)
{
	int ret;
	struct xgold_platform_context *plf_context;

	mali_dbg("%s()\n", __func__);

	plf_context = (struct xgold_platform_context *) kbdev->platform_context;

	ret = kbase_platform_xgold_pm_control(kbdev,
				MALI_PLF_PM_STATE_D3);
	if (ret < 0)
		mali_err("kbase_platform_xgold_pm_control failed(%d)\n", ret);

}

static int pm_callback_runtime_on(struct kbase_device *kbdev)
{
	int ret;
	struct xgold_platform_context *plf_context;

	mali_dbg("%s()\n", __func__);

	plf_context = (struct xgold_platform_context *) kbdev->platform_context;

	ret = kbase_platform_xgold_pm_control(kbdev,
				plf_context->resume_pm_state);
	if (ret < 0)
		mali_err("kbase_platform_xgold_pm_control failed(%d)\n", ret);

	return ret;
}

static struct kbase_pm_callback_conf pm_callbacks = {
	.power_off_callback = pm_callback_power_off,
	.power_on_callback = pm_callback_power_on,
	/*
	Note: If suspend and resume callback are not defined,
	system will use power_off/on_callback instead
	*/
	.power_suspend_callback  = NULL,
	.power_resume_callback = NULL,
	.power_runtime_init_callback = pm_callback_runtime_init,
	.power_runtime_term_callback = pm_callback_runtime_term,
	.power_runtime_off_callback  = pm_callback_runtime_off,
	.power_runtime_on_callback   = pm_callback_runtime_on
};
#endif /* CONFIG_MALI_MIDGARD_RT_PM */

static mali_bool kbase_platform_xgold_init(struct kbase_device *kbdev)
{
	struct platform_device *pdev;

	mali_dbg("%s()\n", __func__);

	if (!kbdev) {
		mali_err("No kbase_device\n");
		return MALI_FALSE;
	}

	pdev = to_platform_device(kbdev->dev);
	if (IS_ERR_OR_NULL(pdev)) {
		mali_err("Could not get platform device\n");
		return MALI_FALSE;
	}

	if (MALI_ERROR_NONE == kbase_platform_init(kbdev))
		return MALI_TRUE;
	else
		return MALI_FALSE;
}

static void kbase_platform_xgold_term(struct kbase_device *kbdev)
{
	mali_dbg("%s()\n", __func__);
	kbase_platform_term(kbdev);
}

static struct kbase_platform_funcs_conf platform_funcs = {
	.platform_init_func = &kbase_platform_xgold_init,
	.platform_term_func = &kbase_platform_xgold_term,
};

static int cpu_speed_func(u32 *clock_speed)
{
	/*
	From ARM header file:
	Once called this will contain the current CPU clock speed in MHz.
	This is mainly used to implement OpenCL's clGetDeviceInfo()
	*/

	KBASE_DEBUG_ASSERT(NULL != clock_speed);

	*clock_speed = 832;

	return 0;
}

static int gpu_speed_func(u32 *clock_speed)
{
	/*
	From Arm header file:
	Once called this will contain the current GPU clock speed in MHz.
	If the system timer is not available then this function is required
	for the OpenCL queue profiling to return correct timing information
	*/

	KBASE_DEBUG_ASSERT(NULL != clock_speed);

	*clock_speed = GPU_FREQ_KHZ_MAX / 1000;

	return 0;

}

static struct kbase_attribute config_attributes[] = {
#ifdef CONFIG_MALI_MIDGARD_RT_PM
	{
	 KBASE_CONFIG_ATTR_POWER_MANAGEMENT_CALLBACKS,
	 (uintptr_t)&pm_callbacks
	},
#endif
	{
	 KBASE_CONFIG_ATTR_PLATFORM_FUNCS,
	 (uintptr_t)&platform_funcs
	},
	{
	 KBASE_CONFIG_ATTR_CPU_SPEED_FUNC,
	 (uintptr_t)&cpu_speed_func
	},
	{
	 KBASE_CONFIG_ATTR_GPU_SPEED_FUNC,
	 (uintptr_t)&gpu_speed_func
	},
#if 0
	{
	 KBASE_CONFIG_ATTR_PM_GPU_POWEROFF_TICK_NS,
	 500000 /* 500us */
	},
	{
	 KBASE_CONFIG_ATTR_PM_POWEROFF_TICK_SHADER,
	 2 /* 2*500us */
	},
	{
	 KBASE_CONFIG_ATTR_PM_POWEROFF_TICK_GPU,
	 2 /* 2*500us */
	},
#endif
	{
	 KBASE_CONFIG_ATTR_END,
	 0
	}
};

static struct kbase_platform_config xgold_platform_config = {
	.attributes = config_attributes,
#ifndef CONFIG_OF
	.io_resources = &io_resources
#endif
};

struct kbase_platform_config *kbase_get_platform_config(void)
{
	mali_dbg("%s()\n", __func__);

	return &xgold_platform_config;
}

int kbase_platform_early_init(void)
{
	mali_dbg("%s()\n", __func__);

	return 0;
}

int kbase_platform_dvfs_event(struct kbase_device *kbdev,
		u32 utilisation,
		u32 util_gl_share,
		u32 util_cl_share[2])
{
	return 0;
}

