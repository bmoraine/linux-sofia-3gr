/*
 ****************************************************************
 *
 *  Component: VLX virtual Power Management frontend driver
 *
 *  Copyright (C) 2011 - 2013 Intel Mobile Communications GmbH
 *  Copyright (C) 2011, Red Bend Ltd.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 ****************************************************************
 */

/*----- System header files -----*/

#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/stddef.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock_types.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/of_irq.h>
#include <linux/device_state_pm.h>

#include <sofia/nk_sofia_bridge.h>
#include <linux/vpower.h>
#include <sofia/mv_hypercalls.h>
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>

#include "vpower.h"

/*----- Local configuration -----*/
#if 1
#define VPOWER_DEBUG
#endif

/*----- Tracing -----*/
#ifdef VPOWER_DEBUG
#define DTRACE(format, args...)	\
	pr_debug("%s: " format, __func__, ##args)
#else
#define DTRACE(x...)
#endif

#define TRACE(x...)	pr_notice("VPOWER-FE: " x)
#define WTRACE(x...)	pr_warn("VPOWER-FE: " x)
#define ETRACE(x...)	pr_err("VPOWER-FE: " x)

extern struct vmm_shared_data *vmm_shared_data[];
extern cycle_t xgold_stm_clock_source_read(struct clocksource *cs);
static DEFINE_MUTEX(call_mutex);

struct vpower_data {
	/* Internal */
	struct pal_shared_data *shared_data;
};

struct vcore_load_info_t {
	uint64_t timec0_previous;
	uint64_t timetotal_previous;
	uint8_t  load_previous;
};

/*C0 residency states per user*/
#define VCORE_NUMBER_OF	6
struct vcore_load_info_t vcore_loads[VCORE_NUMBER_OF];


static DEFINE_PER_CPU(struct vpower_data, percpu_vpower);
static DECLARE_COMPLETION(prh_sync_complete);

static irqreturn_t vmm_prh_irq(int irq, void *dev_id)
{
	complete(&prh_sync_complete);
	return IRQ_HANDLED;
}
#define INTEL_PM_VPOWER "intel,vpower"
#define INTEL_PM_PRH_MODE "intel,prh_mode"
/* TODO: This parameter should be coming from the caller */
enum vmm_pm_opcode pm_opcode = PM_PRH_SET_PER_MODE;


static int of_vpower_parse(void)
{
	const char *vm_power_mode;
	int hirq_prh;
	struct device_node *dn;
	int ret;

	dn = of_find_compatible_node(NULL, NULL, INTEL_PM_VPOWER);
	if (!dn)
		return -EINVAL;

	hirq_prh = irq_of_parse_and_map(dn, 0);
	if (hirq_prh == 0) {
		pr_err("No PRH HIRQ found - going to die !\n");
		BUG();
	}

	ret = request_irq(hirq_prh, vmm_prh_irq, 0, "vmm hirq prh", NULL);
	if (ret) {
		pr_err("Installing HIRQ PRQ (%d) handler failed (%d)\n",
				hirq_prh, ret);
		BUG();
	}

	ret = of_property_read_string(dn, INTEL_PM_PRH_MODE, &vm_power_mode);
	if (!ret) {
		if (strcmp(vm_power_mode, "async") == 0)
			pm_opcode = PM_PRH_SET_PER_MODE_ASYNC;
		else if (strcmp(vm_power_mode, "sync") == 0)
			pm_opcode = PM_PRH_SET_PER_MODE;
		else
			pr_err("%s: Invalid (%s) vpower mode\n",
					__func__, vm_power_mode);

		pr_info("VPOWER: Running in %s mode\n", vm_power_mode);
	} else {
		pr_info("%s: %s is not defined - fallback to sync mode\n",
				__func__, INTEL_PM_PRH_MODE);
	}

	return 0;
}

static ePRH_RETURN_T vpower_init_prh(void)
{
	int i;
	int hirq_prh = 0;
	ePRH_RETURN_T retval;

	DTRACE("Initializing prh\n");
	for (i = 0; i < num_possible_cpus(); i++) {
		struct vpower_data *vpower;
		struct vmm_shared_data *vmmdata = vmm_shared_data[i];
		vpower = &per_cpu(percpu_vpower, i);
		vpower->shared_data =
			(struct pal_shared_data *) vmmdata->pal_shared_mem_data;
	}

	of_vpower_parse();
	/* FIXME: We likely want to question VMM about the mode to be used
	 * a simple 'prh_init' vmcall service should be enough */
	retval = mv_svc_pm_control(PM_PRH_INIT_SET_MODE, 4, 8, 0);
	if (retval != PRH_OK)
		return retval;

	retval = mv_svc_pm_control(PM_PRH_PRE_INIT_SET_MODE, 4, 8, 0);
	if (retval != PRH_OK)
		return retval;

	if (!hirq_prh)
		return retval;


	return retval;
}

ePRH_RETURN_T vpower_call_prh(uint32_t user_id,
			uint32_t per_id,
			uint32_t * const p_per_mode_info,
			uint32_t size)
{
	ePRH_RETURN_T retval;
	struct vpower_data *vpower;

#ifdef CONFIG_VPOWER_STUB
	return PRH_OK;
#endif

/* FIXME: No sense to check on hard coded int there
 * Find a smarter way
   if (size >= 20)
		return PRH_ERR_INV_MODE_INFO;
*/
	if (mutex_lock_interruptible(&call_mutex))
		return PRH_ERR_PER_IN_USE;

	vpower = &get_cpu_var(percpu_vpower);

	if ((vpower == NULL) || (vpower->shared_data == NULL)) {
		retval = PRH_ERR_INTERNAL;
		goto fail;
	}

	memset((&vpower->shared_data->pm_control_shared_data.prh_param), 0,
			PRH_MODE_INFO_SZ);
	memcpy((&vpower->shared_data->pm_control_shared_data.prh_param),
		  (void *) p_per_mode_info, size);

	reinit_completion(&prh_sync_complete);

	retval = mv_svc_pm_control(pm_opcode , user_id, per_id, 0);
	if (retval)
		pr_err("%s: Error(%i) arguments (%#x, %#x, %#x)\n",
			__func__, retval, PM_PRH_SET_PER_MODE, user_id, per_id);

fail:
	put_cpu_var(percpu_vpower);
	if ((retval == 0) && (pm_opcode == PM_PRH_SET_PER_MODE_ASYNC))
		wait_for_completion(&prh_sync_complete);
	mutex_unlock(&call_mutex);

	/* return the actual prh ret value from backend */
	return retval;
}

void vpower_get_C0(int cpu_id, uint64_t *timetotal_ptr, uint64_t *timeC0_ptr)
{
	struct vpower_data *vpower;
	struct clocksource *cs = NULL;
	uint64_t time = xgold_stm_clock_source_read(cs);
	ePRH_RETURN_T retval;

	vpower = &get_cpu_var(percpu_vpower);

	if ((vpower == NULL) || (vpower->shared_data == NULL))
		goto fail;

	if ((cpu_id >= VCORE_NUMBER_OF) ||
		(timetotal_ptr == NULL) ||
		(timeC0_ptr == NULL)) {
		ETRACE("Invalid parameters\n");
		goto fail;
	}
	/* vcpu_c0 struct {	u64 mex_c0;
	 *			u64 linux_c0[NOF_LINUX_VCPU];
	 *			u64 secvm_c0;};  */

	retval = mv_svc_pm_control(PM_GET_VCPU_C0, 0, 0, 0);

	*timeC0_ptr = (uint64_t)
		(vpower->shared_data->pm_control_shared_data.vcpu_c0[cpu_id*2]);
	*timetotal_ptr = time;
fail:
	put_cpu_var(percpu_vpower);
}

int xgold_cpu_load_get(int cpuid)
{
	struct vcore_load_info_t *vcore_l;
	uint64_t timec0_now = 0;
	uint64_t timetotal_now = 0;
	uint32_t load = 0, deltatimetotal, deltatimeC0;
	struct vpower_data *vpower;
	struct clocksource *cs = NULL;

	if (cpuid >= VCORE_NUMBER_OF) {
		ETRACE("Invalid parameters\n");
		return -1;
	}

	vpower = &get_cpu_var(percpu_vpower);
	if (vpower == NULL) {
		/* not enough info to compute the load, return 0% as load */
		put_cpu_var(percpu_vpower);
		return 0;
	}

	vcore_l = &vcore_loads[cpuid];

	if (vcore_l->timec0_previous != 0) {
		vpower_get_C0(cpuid, &timetotal_now, &timec0_now);
		DTRACE("timec0_now = %llu\n", timec0_now);
		DTRACE("timetotal_now = %llu\n", timetotal_now);
		if (vcore_l->timetotal_previous == timetotal_now)
			load = vcore_l->load_previous;
		else {
			deltatimeC0 = (timec0_now - vcore_l->
							timec0_previous) * 100;
			deltatimetotal = (timetotal_now - vcore_l->
							timetotal_previous);
			load = (deltatimeC0/deltatimetotal);
		}
	} else {
		timec0_now = xgold_stm_clock_source_read(cs);
		timetotal_now = xgold_stm_clock_source_read(cs);
	}

	vcore_l->load_previous = load;
	vcore_l->timec0_previous = timec0_now;
	vcore_l->timetotal_previous = timetotal_now;

	if (load > 100)
		load = 100;

	put_cpu_var(percpu_vpower);
	DTRACE("xgold load computed = %d, cpuid = %d\n", load, cpuid);
	return load;
}
EXPORT_SYMBOL(xgold_cpu_load_get);


void vpower_set_cpu_target_frequency(const int cpufreq)
{
	ePRH_RETURN_T retval;
	retval = mv_svc_pm_control(PM_OMP_SET_POLICY, 1, cpufreq, 0);
	DTRACE("======= ask vmm to change to %d, return = %x\n", cpufreq,
			retval);

}
EXPORT_SYMBOL(vpower_set_cpu_target_frequency);

uint32_t vpower_system_get_cpu_freq(void)
{
	struct vpower_data *vpower;
	uint32_t freq = 0;

	vpower = &get_cpu_var(percpu_vpower);

	if ((vpower == NULL) || (vpower->shared_data == NULL))
		goto fail;

	freq = vpower->shared_data->pm_control_shared_data.cpu_clk;

fail:
	put_cpu_var(percpu_vpower);

	/* TODO */
	return freq;
}


static int __init vpower_init(void)
{
	vpower_init_prh();

	return 0;
}
arch_initcall(vpower_init);

