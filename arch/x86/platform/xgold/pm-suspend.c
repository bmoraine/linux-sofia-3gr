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
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/err.h>
#include <asm/x86_init.h>
#include <linux/of.h>
#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/vpower.h>
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>
#include <sofia/mv_gal.h>
#endif
#endif

int xgold_suspend_enter(suspend_state_t suspend_state)
{

	struct vmm_shared_data *data = mv_gal_get_shared_data();

	switch (suspend_state) {
	case PM_SUSPEND_MEM:
		if (x86_platform.save_sched_clock_state)
			x86_platform.save_sched_clock_state();
		mv_svc_vm_enter_idle(data->pal_shared_mem_data,
					PM_S3);
		native_halt();
		mv_svc_vm_enter_idle(data->pal_shared_mem_data,
					PM_S0);
		break;
	}
	return 0;
}

void xgold_suspend_finish(void)
{
	if (x86_platform.restore_sched_clock_state)
		x86_platform.restore_sched_clock_state();
}

static const struct platform_suspend_ops xgold_suspend_ops = {
	.enter		= xgold_suspend_enter,
	.finish		= xgold_suspend_finish,
	.valid		= suspend_valid_only_mem,
};

static int __init xgold_suspend_init(void)
{
	struct device_node *np = of_find_node_by_path("/xgold");
	if (!of_find_property(np, "intel,nodeepsleep", NULL))
			suspend_set_ops(&xgold_suspend_ops);
	return 0;
}
late_initcall(xgold_suspend_init);
