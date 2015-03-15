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
#include <linux/notifier.h>
#include <linux/clockchips.h>
#include <linux/of.h>
#include <asm/xgold.h>
#ifdef CONFIG_X86_INTEL_XGOLD
#include <asm/irqflags.h>
#else
#include <asm/cpuidle.h>
#include <asm/proc-fns.h>
#endif
#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/vpower.h>
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>
#include <sofia/mv_gal.h>
#else
#include <vlx/vpower_common.h>
#endif
#endif

struct xgold_cpuidle_state {
	unsigned id;
};

static struct xgold_cpuidle_state xgold_cpuidle_states[CPUIDLE_STATE_MAX] = {
	[0] = {
		.id = PM_S0,
	},
	[1] = {
		.id = PM_S1,
	},
};


#define XGOLD_CPUIDLE_ACTIVE xgold_cpuidle_states[0].id

#ifdef CONFIG_X86_INTEL_SOFIA
static int xgold_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_driver *drv, int index)
{
	struct xgold_cpuidle_state *xg_idle = &xgold_cpuidle_states[index];
	struct vmm_shared_data *data = mv_gal_get_shared_data();

	mv_svc_vm_enter_idle(data->pal_shared_mem_data, xg_idle->id);
	native_halt();
	/* back to active */
	mv_svc_vm_enter_idle(data->pal_shared_mem_data, XGOLD_CPUIDLE_ACTIVE);

	return index;
}
#else
static int xgold_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_driver *drv, int index)
	native_safe_halt();
	return index;
}
#endif


static struct cpuidle_driver xgold_cpuidle_driver = {
	.name = "cpuidle_xgold",
	.owner = THIS_MODULE,
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
		[1] = {
			.name = "S1",
			.desc = "Shutdown",
			.flags = CPUIDLE_FLAG_TIME_VALID,
			.exit_latency = 100,
			.power_usage = 10,
			.target_residency = 100,
			.enter = xgold_enter_idle,
		},
	},
	.state_count = 2,
};
static void __setup_broadcast_timer(void *arg)
{
	unsigned long reason = (unsigned long)arg;
	int cpu = smp_processor_id();

	WARN_ON(!xgold_platform_needs_broadcast_timer());

	reason = reason ?
		CLOCK_EVT_NOTIFY_BROADCAST_ON : CLOCK_EVT_NOTIFY_BROADCAST_OFF;

	clockevents_notify(reason, &cpu);
}

#define INTEL_IDLE_STATE_COMPAT "intel,sofia,idle-state"
#define INTEL_IDLE_STATE_DESC "desc"
#define INTEL_IDLE_STATE_RESIDENCY "target-residency"
#define INTEL_IDLE_STATE_LATENCY "exit-latency"
#define INTEL_IDLE_STATE_POWER "power-usage"
#define INTEL_IDLE_STATE_FLAGS "flags"
#define INTEL_IDLE_STATE_VMMID "vmm-id"

static void xgold_cpuidle_print_states(struct cpuidle_driver *idle_drv)
{
	unsigned i;

	pr_info("idle xgold: Name | latency | residency | power | flags | vmmid\n");
	for (i = 0; i < idle_drv->state_count; i++) {
		struct cpuidle_state *idle_state;
		struct xgold_cpuidle_state *xg_idle_state;
		idle_state = &idle_drv->states[i];
		xg_idle_state = &xgold_cpuidle_states[i];
		pr_info("\t%s [%s]\t %d\t %d\t %d\t %d %d\n",
				idle_state->name,
				idle_state->desc,
				idle_state->exit_latency,
				idle_state->target_residency,
				idle_state->power_usage,
				idle_state->flags,
				xg_idle_state->id);
	}
}

static int __init xgold_cpuidle_parse_dt(void)
{
	int ret;
	unsigned nr_state = 0;
	const char *desc;
	struct device_node *states, *state;
	struct cpuidle_driver *idle_drv = &xgold_cpuidle_driver;
	states = of_find_node_by_path("/cpus/idle-states");
	if (states == NULL) {
		pr_err("%s: No idle states defined in dts\n", __func__);
		return -EINVAL;
	}

	/* How Many states */
	for_each_child_of_node(states, state) {
		struct cpuidle_state *idle_state;
		struct xgold_cpuidle_state *xg_cpuidle_state;
		unsigned vmm_id;
		if (!(of_device_is_compatible(state, INTEL_IDLE_STATE_COMPAT)))
			continue;

		ret = of_property_read_u32(state, INTEL_IDLE_STATE_VMMID,
						&vmm_id);
		BUG_ON(ret);

		if (!xgold_platform_needs_broadcast_timer()
				&& (vmm_id == 0)) {
			pr_info("Skipping non sense state %s if no broadcast timer is needed\n",
					state->name);
			continue;
		}

		idle_state = &idle_drv->states[nr_state];
		xg_cpuidle_state = &xgold_cpuidle_states[nr_state];

		strncpy(idle_state->name, state->name, CPUIDLE_NAME_LEN);

		ret = of_property_read_string(state, INTEL_IDLE_STATE_DESC,
							&desc);
		BUG_ON(ret);

		strncpy(idle_state->desc, desc, CPUIDLE_DESC_LEN);

		ret = of_property_read_u32(state, INTEL_IDLE_STATE_FLAGS,
							&idle_state->flags);
		BUG_ON(ret);

		ret = of_property_read_u32(state, INTEL_IDLE_STATE_LATENCY,
						&idle_state->exit_latency);
		BUG_ON(ret);

		ret = of_property_read_u32(state, INTEL_IDLE_STATE_POWER,
						&idle_state->power_usage);
		BUG_ON(ret);

		ret = of_property_read_u32(state, INTEL_IDLE_STATE_RESIDENCY,
						&idle_state->target_residency);
		BUG_ON(ret);

		ret = of_property_read_u32(state, INTEL_IDLE_STATE_VMMID,
						&xg_cpuidle_state->id);
		BUG_ON(ret);

		idle_state->enter = xgold_enter_idle;

		nr_state++;
	}

	idle_drv->state_count = nr_state;

	return 0;
}

static int __init xgold_cpuidle_init(void)
{
	pr_info("%s: Initializing xgold cpuidle driver\n", __func__);

	if (xgold_platform_needs_broadcast_timer())
		on_each_cpu(__setup_broadcast_timer, (void *)true, 1);

	xgold_cpuidle_parse_dt();
	xgold_cpuidle_print_states(&xgold_cpuidle_driver);
	/* We want to boot a UP configuration,
	 * even with SMP kernel configuration
	 *  if cpumask is not defined, this will be initialized
	 *  with cpu_possible_mask that we don't want for UP */
	xgold_cpuidle_driver.cpumask = (struct cpumask *) cpu_online_mask;
	return cpuidle_register(&xgold_cpuidle_driver, NULL);
}

static void __exit xgold_cpuidle_exit(void)
{
	cpuidle_unregister(&xgold_cpuidle_driver);

	if (xgold_platform_needs_broadcast_timer())
		on_each_cpu(__setup_broadcast_timer, (void *)false, 1);

	return;
}

subsys_initcall(xgold_cpuidle_init);
