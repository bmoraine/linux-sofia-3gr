/*
 ****************************************************************
 *
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
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

#include <sofia/vpower.h>
#include <sofia/mv_hypercalls.h>
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>

extern struct vmm_shared_data *vmm_shared_data[];
static DEFINE_MUTEX(call_mutex);

struct vpower_data {
	/* Internal */
	struct pal_shared_data *shared_data;
};


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

	ret = request_irq(hirq_prh, vmm_prh_irq,
			IRQF_NO_SUSPEND, "vmm hirq prh", NULL);
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
	mutex_lock(&call_mutex);

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
	if ((retval == 0) && (pm_opcode == PM_PRH_SET_PER_MODE_ASYNC)) {
		retval = wait_for_completion_timeout(&prh_sync_complete,
				msecs_to_jiffies(3000));
		if (retval == 0) {
			pr_err("%s: Timeout waiting for PRH async interrupt\n", __func__);
			BUG();
		} else {
			retval = (ePRH_RETURN_T) vpower->shared_data
			->pm_control_shared_data.prh_request_return_value;
		}
	}

	mutex_unlock(&call_mutex);

	/* return the actual prh ret value from backend */
	return retval;
}
static int __init vpower_init(void)
{
	vpower_init_prh();

	return 0;
}
arch_initcall(vpower_init);

