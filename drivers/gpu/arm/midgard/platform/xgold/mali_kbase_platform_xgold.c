/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * Notes:
 * Feb 25 2015: IMC: Add initial xgold platform adaptation code
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

#include <mali_kbase.h>
#include "mali_kbase_platform_xgold.h"
#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/xgold_noc.h>
#ifdef CONFIG_MALI_MIDGARD_DVFS
#include <platform/xgold/mali_kbase_dvfs_xgold.h>
#include <platform/xgold/mali_kbase_platform_debugfs.h>
#endif


static int kbase_platform_xgold_pm_init(struct kbase_device *kbdev)
{
	int ret = -1;
	struct xgold_platform_context *plf_context;
	struct device_node *np;
	struct platform_device *pdev;
	struct device *dev = kbdev->dev;

	mali_dbg("%s()\n", __func__);

	pdev = to_platform_device(dev);
	plf_context = (struct xgold_platform_context *) kbdev->platform_context;

	np = pdev->dev.of_node;
	plf_context->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR_OR_NULL(plf_context->pm_platdata)) {
		mali_err("Device state pm setup from devicetree\n");
		return -EPERM;
	}

	ret = platform_device_pm_set_class(pdev,
			plf_context->pm_platdata->pm_user_name);
	if (ret) {
		mali_err("Device state pm set class\n");
		return -EPERM;
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
		return -EPERM;
	}

	plf_context->curr_pm_state = MALI_PLF_PM_STATE_D3;
	plf_context->resume_pm_state = MALI_PLF_PM_STATE_D0;
#ifdef CONFIG_MALI_MIDGARD_DVFS
	spin_lock_init(&plf_context->pm_lock);
	plf_context->dvfs_off = MALI_FALSE;
	platform_debugfs_register(kbdev);
#endif
	mutex_init(&plf_context->pm_lock_mutex);

	return 0;
}

int kbase_platform_xgold_pm_control(struct kbase_device *kbdev,
							int req_pm_state)
{
	int ret = 0;
	struct xgold_platform_context *plf_context;
	struct device *dev = kbdev->dev;
	struct platform_device *pdev;

	mali_dbg("%s()\n", __func__);

	pdev = to_platform_device(dev);
	plf_context = (struct xgold_platform_context *) kbdev->platform_context;

	if (plf_context->curr_pm_state == req_pm_state)
		return ret;

	ret = platform_device_pm_set_state(pdev,
		plf_context->pm_states[req_pm_state]);
	if (ret < 0) {
		mali_err("Device pm set state failed (%d)\n", ret);
		return ret;
	}
	if (plf_context->curr_pm_state == MALI_PLF_PM_STATE_D3) {
		xgold_noc_qos_set("GPU");
		mali_dbg("Set GPU QoS\n");
	}
	plf_context->curr_pm_state = req_pm_state;

	mali_dbg("%s() GPU pm state set to %d\n", __func__, req_pm_state);

	return ret;
}

mali_error kbase_platform_init(struct kbase_device *kbdev)
{
	struct xgold_platform_context *plf_context;

	mali_dbg("%s()\n", __func__);

	plf_context = devm_kzalloc(kbdev->dev,
		sizeof(struct xgold_platform_context), GFP_KERNEL);
	if (!plf_context) {
		mali_err("Could not allocate platform context\n");
		return MALI_ERROR_OUT_OF_MEMORY;
	}

	kbdev->platform_context = (void *) plf_context;

	if (kbase_platform_xgold_pm_init(kbdev)) {
		mali_err("Power initialization failed");
		return MALI_ERROR_FUNCTION_FAILED;
	}

	kbase_platform_xgold_pm_control(kbdev, plf_context->resume_pm_state);
#ifdef CONFIG_MALI_MIDGARD_DVFS
	kbase_platform_dvfs_init(plf_context);
	plf_context->kbdev = kbdev;
#endif
	mali_info("Initialized\n");
	return MALI_ERROR_NONE;
}

void kbase_platform_term(struct kbase_device *kbdev)
{
	struct xgold_platform_context *plf_context;
	int ret;

	mali_dbg("%s()\n", __func__);

	plf_context = (struct xgold_platform_context *) kbdev->platform_context;

#ifdef CONFIG_MALI_MIDGARD_DVFS
	kbase_platform_dvfs_term(plf_context);
#endif

	mutex_lock(&plf_context->pm_lock_mutex);

	ret = kbase_platform_xgold_pm_control(kbdev, MALI_PLF_PM_STATE_D3);
	if (ret < 0)
		mali_err("kbase_platform_xgold_pm_control failed (%d)\n", ret);

	mutex_unlock(&plf_context->pm_lock_mutex);

	mutex_destroy(&plf_context->pm_lock_mutex);

	devm_kfree(kbdev->dev, kbdev->platform_context);
	kbdev->platform_context = 0;
}
