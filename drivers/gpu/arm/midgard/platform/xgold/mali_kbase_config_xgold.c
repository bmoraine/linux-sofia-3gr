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
#if defined(CONFIG_PM_RUNTIME)
#include <linux/pm_runtime.h>
#endif
#include <linux/xgold_noc.h>

#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config.h>
#include "mali_kbase_config_platform.h"
#include "mali_kbase_platform_xgold.h"


static void pm_callback_power_off(struct kbase_device *kbdev)
{
	int ret = -1;
	struct xgold_platform_context *plf_context;
	struct platform_device *pdev;

	mali_dbg("%s()\n", __func__);

	if (!kbdev) {
		mali_err("No kbase_device\n");
		return;
	}

	plf_context = (struct xgold_platform_context *) kbdev->platform_context;

	/* Already powered off */
	if (MALI_PLF_PM_STATE_D3 == plf_context->curr_pm_state)
		return;

	pdev = to_platform_device(kbdev->dev);
	if (IS_ERR_OR_NULL(pdev)) {
		mali_err("Could not get platform device\n");
		return;
	}

	ret = platform_device_pm_set_state(pdev,
		plf_context->pm_states[MALI_PLF_PM_STATE_D3]);
	if (ret < 0) {
		mali_err("Device pm set state failed (%d)\n", ret);
		return;
	}

	plf_context->curr_pm_state = MALI_PLF_PM_STATE_D3;

	mali_dbg("powered off\n");
}

static int pm_callback_power_on(struct kbase_device *kbdev)
{
	int ret = -1;
	struct xgold_platform_context *plf_context;
	struct platform_device *pdev;

	mali_dbg("%s()\n", __func__);

	if (!kbdev) {
		mali_err("No kbase_device\n");
		return MALI_FALSE;
	}

	plf_context = (struct xgold_platform_context *) kbdev->platform_context;

	/* Already powered up */
	if (plf_context->resume_pm_state == plf_context->curr_pm_state)
		return 0;

	pdev = to_platform_device(kbdev->dev);
	if (IS_ERR_OR_NULL(pdev)) {
		mali_err("Could not get platform device\n");
		return MALI_FALSE;
	}

	ret = platform_device_pm_set_state(pdev,
		plf_context->pm_states[plf_context->resume_pm_state]);
	if (ret < 0) {
		mali_err("Device pm set state failed (%d)\n", ret);
		return ret;
	}

	plf_context->curr_pm_state = plf_context->resume_pm_state;

	/* ToDo: Only call once after boot-up and on system resume!!! */
	/* Need to set GPU QoS on power_on */
	xgold_noc_qos_set("GPU");
	mali_dbg("Set GPU QoS\n");

	mali_dbg("powered on\n");
	/* return 1 if the GPU state may have been lost, 0 otherwise */
	return 1;
}

#ifdef CONFIG_PM_RUNTIME
static mali_error pm_callback_runtime_init(struct kbase_device *kbdev)
{
	mali_dbg("%s()\n", __func__);

	return MALI_ERROR_NONE;
}

static void pm_callback_runtime_term(struct kbase_device *kbdev)
{
	mali_dbg("%s()\n", __func__);

}

static void pm_callback_runtime_off(struct kbase_device *kbdev)
{
	mali_dbg("%s()\n", __func__);

}

static int pm_callback_runtime_on(struct kbase_device *kbdev)
{
	mali_dbg("%s()\n", __func__);

	return 0;
}
#endif /* CONFIG_PM_RUNTIME */


static struct kbase_pm_callback_conf pm_callbacks = {
	.power_off_callback = pm_callback_power_off,
	.power_on_callback = pm_callback_power_on,
	/*
	Note: If suspend and resume callback are not defined,
	system will used power_off/on_callback instead
	*/
	.power_suspend_callback  = NULL,
	.power_resume_callback = NULL,
#ifdef CONFIG_PM_RUNTIME
	.power_runtime_init_callback = pm_callback_runtime_init,
	.power_runtime_term_callback = pm_callback_runtime_term,
	.power_runtime_off_callback  = pm_callback_runtime_off,
	.power_runtime_on_callback   = pm_callback_runtime_on
#else
	.power_runtime_init_callback = NULL,
	.power_runtime_term_callback = NULL,
	.power_runtime_off_callback  = NULL,
	.power_runtime_on_callback   = NULL
#endif
};

static mali_bool kbase_platform_xgold_init(struct kbase_device *kbdev)
{
	int ret = -1;
	struct xgold_platform_context *plf_context;
	struct device_node *np;
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

	plf_context = devm_kzalloc(kbdev->dev,
		sizeof(struct xgold_platform_context), GFP_KERNEL);
	if (!plf_context) {
		mali_err("Could not allocate platform context\n");
		return MALI_FALSE;
	}

	np = pdev->dev.of_node;
	plf_context->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR_OR_NULL(plf_context->pm_platdata)) {
		mali_err("Device state pm setup from devicetree\n");
		return MALI_FALSE;
	}

	ret = platform_device_pm_set_class(pdev,
			plf_context->pm_platdata->pm_user_name);
	if (ret) {
		mali_err("Device state pm set class\n");
		return MALI_FALSE;
	}

	/* Is this the right way to get the state handlers when you use DTS? */
	plf_context->pm_states[0] =
		platform_device_pm_get_state_handler(pdev,
		"disable");
	plf_context->pm_states[1] =
		platform_device_pm_get_state_handler(pdev,
		"low_perf");
	plf_context->pm_states[2] =
		platform_device_pm_get_state_handler(pdev,
		"mid_perf");
	plf_context->pm_states[3] =
		platform_device_pm_get_state_handler(pdev,
		"high_perf");
#if defined(GPU_USE_ULTRA_HIGH_PERF)
	plf_context->pm_states[4] =
		platform_device_pm_get_state_handler(pdev,
		"ultra_high_perf");
#endif /* defined(GPU_USE_ULTRA_HIGH_PERF) */
	if (plf_context->pm_states[0] == 0
		|| plf_context->pm_states[1] == 0
		|| plf_context->pm_states[2] == 0
		|| plf_context->pm_states[3] == 0
#if defined(GPU_USE_ULTRA_HIGH_PERF)
		|| plf_context->pm_states[4] == 0
#endif /* defined(GPU_USE_ULTRA_HIGH_PERF) */
		) {
		mali_err("Device pm unable to get state handler\n");
		return MALI_FALSE;
	}

	/* Currently powered off */
	plf_context->curr_pm_state = MALI_PLF_PM_STATE_D3;

	/* Initial power level (D1=312MHz / D0=456MHz) */
	plf_context->resume_pm_state = MALI_PLF_PM_STATE_D0;

	kbdev->platform_context = (void *) plf_context;
	mali_info("Initialized\n");

	return MALI_TRUE;
}

static void kbase_platform_xgold_term(struct kbase_device *kbdev)
{
	mali_dbg("%s()\n", __func__);

	devm_kfree(kbdev->dev, kbdev->platform_context);
	kbdev->platform_context = 0;
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
	{
	 KBASE_CONFIG_ATTR_POWER_MANAGEMENT_CALLBACKS,
	 (uintptr_t)&pm_callbacks
	},
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

