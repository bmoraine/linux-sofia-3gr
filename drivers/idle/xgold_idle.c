/*
* cpuidle.c - platform specific driver
* Copyright (C) 2012-2013 Intel Mobile Communications GmbH
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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/cpuidle.h>
#ifdef CONFIG_X86_INTEL_XGOLD
#include <asm/irqflags.h>
#else
#include <asm/cpuidle.h>
#include <asm/proc-fns.h>
#endif
#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
#ifdef CONFIG_X86_INTEL_SOFIA
#include <linux/vpower.h>
#include <sofia/pal_shared_data.h>
#include <sofia/vmm_platform_service.h>
#else
#include <vlx/vpower_common.h>
#endif
#endif

/****
	1 - C1	- WFE
	2 - C2	- Dormant
	3 - C3	- Shutdown
	4 - S0i3- Shutdown with higher latency. MEX sleep allowed.
****/


static int xgold_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_driver *drv, int index)
{
#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
	struct vmm_shared_data *data = get_vmm_shared_data();
	switch (index) {
	case 0:
		vm_enter_idle(data->pal_shared_mem_data,
					PM_S0);
		native_safe_halt();
		break;
#if 0
	case 1:		/* go to Shutdown state */
		vm_enter_idle(data->pal_shared_mem_data,
					PM_S1);
		native_safe_halt();
		break;
	case 2:		/* Allow MEX to sleep */
		vm_enter_idle(data->pal_shared_mem_data,
					PM_S0i3);
		native_safe_halt();
		break;
#endif
	default:	/* should never get here */
		break;
		}
	/* back to active */
	vm_enter_idle(data->pal_shared_mem_data,
				PM_S0);
	/* inform MEX not to sleep */
#else
	switch (index) {
	case 0:
		native_safe_halt();
		break;
	case 1: /*dummy case for test */
		native_safe_halt();
		break;
	default:
		break;
	};
#endif
	local_irq_enable();
	return index;
}

static struct cpuidle_driver xgold_cpuidle_driver = {
	.name = "cpuidle_xgold",
	.owner = THIS_MODULE,
#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
	.states = {
		[0] = {
			.name = "S0",
			.desc = "WFE",
			.flags = CPUIDLE_FLAG_TIME_VALID,
			.exit_latency = 1,
			.power_usage = 100,
			.target_residency = 2,
			.enter = &xgold_enter_idle,
		},
#if 0
		[1] = {
			.name = "S1",
			.desc = "Shutdown",
			.flags = CPUIDLE_FLAG_TIME_VALID,
			.exit_latency = 100,
			.power_usage = 10,
			.target_residency = 100,
			.enter = xgold_enter_idle,
		},
		[2] = {
			.name = "S0i3",
			.desc = "Allow MEX sleep",
			.flags = CPUIDLE_FLAG_TIME_VALID,
			.exit_latency = 200,
			.power_usage = 5,
			.target_residency = 400,
			.enter = xgold_enter_idle,
		},
#endif
	},
	/*.state_count = 3,*/
	.state_count = 1,
#else
	.states = {
		[0] = {
			.name = "C1",
			.desc = "WFE",
			.flags = CPUIDLE_FLAG_TIME_VALID,
			.exit_latency = 1,
			.power_usage = 100,
			.target_residency = 2,
			.enter = &xgold_enter_idle,
		},
		[1] = {
			.name = "C2",
			.desc = "Shutdown",
			.flags = CPUIDLE_FLAG_TIME_VALID,
			.exit_latency = 100,
			.power_usage = 50,
			.target_residency = 100,
			.enter = &xgold_enter_idle,
		},
	},
	.state_count = 2,
#endif
};


static int __init xgold_cpuidle_init(void)
{
	return cpuidle_register(&xgold_cpuidle_driver, NULL);
}

device_initcall(xgold_cpuidle_init);
