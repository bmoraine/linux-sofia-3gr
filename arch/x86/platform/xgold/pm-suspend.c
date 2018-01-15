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
#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
#ifdef CONFIG_X86_INTEL_SOFIA
#include <linux/vpower.h>
#include <sofia/pal_shared_data.h>
#include <sofia/vmm_platform_service.h>
#endif
#endif

int xgold_suspend_enter(suspend_state_t suspend_state)
{

	struct vmm_shared_data *data = get_vmm_shared_data();

	switch (suspend_state) {
	case PM_SUSPEND_MEM:
		vm_enter_idle(data->pal_shared_mem_data,
					PM_S3);
		native_halt();
		vm_enter_idle(data->pal_shared_mem_data,
					PM_S0);
		break;
	}
	return 0;
}

static const struct platform_suspend_ops xgold_suspend_ops = {
	.enter		= xgold_suspend_enter,
	.valid		= suspend_valid_only_mem,
};

static int __init xgold_suspend_init(void)
{
	suspend_set_ops(&xgold_suspend_ops);
	return 0;
}
late_initcall(xgold_suspend_init);
