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

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/err.h>
#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
#ifdef CONFIG_X86_INTEL_SOFIA
#include <linux/vpower.h>
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>
#endif
#endif

#define VCORE_MEX	0
#define VCORE_SECVM	3
unsigned long arch_scale_freq_power(struct sched_domain *sd, int cpu)
{
	int mex_load;
	int secvm_load;
	if (cpu != 0) {
		mex_load = xgold_cpu_load_get(VCORE_MEX);
		secvm_load = xgold_cpu_load_get(VCORE_SECVM);
		if (mex_load <= 0)
			mex_load = 30;
		if (secvm_load <= 0)
			secvm_load = 10;
		return (100 - mex_load - secvm_load) * SCHED_POWER_SCALE / 100;
	} else
		return SCHED_POWER_SCALE;
}


