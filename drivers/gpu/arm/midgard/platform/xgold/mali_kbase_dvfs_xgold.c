/*
 * Copyright (C) 2015 Intel Mobile Communications GmbH
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
 * March 11 2015: IMC: Add initial xgold platform adaptation code
 *
 */
#include <mali_kbase.h>
#include <platform/xgold/mali_kbase_platform_xgold.h>
#include <platform/xgold/mali_kbase_dvfs_xgold.h>

static void mali_dvfs_wq(struct work_struct *ws)
{
	struct xgold_platform_context *plf_context = NULL;
	unsigned long flags;
	unsigned int temp_utilization;

	plf_context = map_to_xgold_platform_context(ws, dvfs_work);

	mutex_lock(&plf_context->pm_lock_mutex);

	mali_dbg("%s()", __func__);
	mali_dbg("current_state=%d\n", plf_context->curr_pm_state);

	if (plf_context->curr_pm_state == MALI_PLF_PM_STATE_D3)
		goto out;

	spin_lock_irqsave(&plf_context->pm_lock, flags);

	temp_utilization = plf_context->utilization;

	spin_unlock_irqrestore(&plf_context->pm_lock, flags);

	mali_dbg("%s()utilization=%d\n", __func__, temp_utilization);

	if (temp_utilization > MAX_UTILISATION) {
		mali_dbg("%s() stepping up\n", __func__);

		kbase_platform_xgold_pm_control(plf_context->kbdev,
			plf_context->resume_pm_state);
	} else if (temp_utilization < MIN_UTILISATION) {
		if (plf_context->curr_pm_state - 1) {

			mali_dbg("%s()stepping down\n", __func__);
			kbase_platform_xgold_pm_control(plf_context->kbdev,
			((plf_context->curr_pm_state)-1));
		}
	}

out:
	mutex_unlock(&plf_context->pm_lock_mutex);
	return;
}


int kbase_platform_dvfs_init(struct xgold_platform_context *plf_context)
{
	mali_dbg("%s() mali\n", __func__);
	INIT_WORK(&plf_context->dvfs_work, mali_dvfs_wq);
	plf_context->mali_dvfs_wq  = create_singlethread_workqueue("mali_dvfs");
	return 0;
}
void kbase_platform_dvfs_term(struct xgold_platform_context *plf_context)
{
	mali_dbg("%s() mali\n", __func__);
	flush_workqueue(plf_context->mali_dvfs_wq);
	destroy_workqueue(plf_context->mali_dvfs_wq);
	return;
}

int kbase_platform_dvfs_event(struct kbase_device *kbdev,
		u32 utilisation,
		u32 util_gl_share,
		u32 util_cl_share[2])
{
	struct xgold_platform_context *plf_context;
	unsigned long flags;

	plf_context = (struct xgold_platform_context *) kbdev->platform_context;

	spin_lock_irqsave(&plf_context->pm_lock, flags);

	plf_context->utilization = utilisation;

	spin_unlock_irqrestore(&plf_context->pm_lock, flags);

	queue_work(plf_context->mali_dvfs_wq, &plf_context->dvfs_work);

	return 0;
}

void kbase_platform_dvfs_enable(bool enable, struct kbase_device *kbdev)
{
	unsigned long flags;
	struct xgold_platform_context *plf_context;
	plf_context = (struct xgold_platform_context *) kbdev->platform_context;

	if (kbdev->pm.metrics.timer_active != enable) {
		if (enable) {
			mali_dbg("%s() timer_active =%d\n", __func__, enable);

			spin_lock_irqsave(&kbdev->pm.metrics.lock, flags);

			kbdev->pm.metrics.timer_active = MALI_TRUE;

			spin_unlock_irqrestore(&kbdev->pm.metrics.lock, flags);

			hrtimer_start(&kbdev->pm.metrics.timer,
			HR_TIMER_DELAY_MSEC(kbdev->pm.platform_dvfs_frequency),
			HRTIMER_MODE_REL);
		} else {
			mali_dbg("%s() timer_active =%d\n", __func__, enable);

			flush_workqueue(plf_context->mali_dvfs_wq);

			spin_lock_irqsave(&kbdev->pm.metrics.lock, flags);

			kbdev->pm.metrics.timer_active = MALI_FALSE;

			spin_unlock_irqrestore(&kbdev->pm.metrics.lock, flags);

			hrtimer_cancel(&kbdev->pm.metrics.timer);
		}
	}
	return;
}

