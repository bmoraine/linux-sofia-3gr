/* Copyright (C) 2014 Intel Mobile Communications GmbH
 * *
 * * This software is licensed under the terms of the GNU General Public
 * * License version 2, as published by the Free Software Foundation, and
 * * may be copied, distributed, and modified under those terms.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * * GNU General Public License for more details.
 * */


#include <sofia/mv_gal.h>
#include <sofia/mv_hypercalls.h>
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>
#include <sofia/cpu.h>

#include <linux/sched.h>
#include <linux/of.h>

struct vcore_load_info_t {
	uint64_t timestolen_previous;
	uint64_t timetotal_previous;
	uint8_t  load_previous;
};

/*C0 residency states per user*/
struct vcore_load_info_t vcore_loads;
static DEFINE_PER_CPU(struct vcore_load_info_t, percpu_vcore_load);

struct static_key paravirt_steal_enabled;
struct static_key paravirt_steal_rq_enabled;
struct static_key paravirt_steal_power_enabled;

int sofia_get_steal_time(uint64_t *total_time, uint64_t *active_stolen_time,
						uint64_t *idle_stolen_time)
{
	struct vmm_shared_data *mv_shm = mv_gal_get_shared_data();
	int ret = 0;
	u32 sampling_period;
	u64 active_stolen_again;
	BUG_ON(mv_shm == NULL);

	if ((total_time == NULL)
		|| (active_stolen_time == NULL)
		|| (idle_stolen_time == NULL)) {
			pr_err("%s: Invalid parameters\n", __func__);
			return -EINVAL;
	}

	/* vcpu_stolen_cpu_time {u64 active_stolen_cpu_count;
	 *			u64 idle_stolen_cpu_count;};  */
retry:

	/* FIXME: Should be provided from the hypercall */

	*active_stolen_time = (uint64_t)
			(mv_shm->stolen_cpu_time_stats.active_stolen_cpu_count);
	*idle_stolen_time = (uint64_t)
			(mv_shm->stolen_cpu_time_stats.idle_stolen_cpu_count);

	*total_time = sched_clock();
	active_stolen_again = (uint64_t)
			(mv_shm->stolen_cpu_time_stats.active_stolen_cpu_count);

	/* check function has not been preempted for time coherence */
	if (active_stolen_again != *active_stolen_time) {
		pr_debug("%s:Preempted while reading stolen time\n", __func__);
		goto retry;
	}

	if ((*active_stolen_time + *idle_stolen_time) != 0)
		pr_debug("Stolen time: now %llu ns, active %llu ns, idle %llu ns\n",
			*total_time,
			*active_stolen_time,
			*idle_stolen_time);

	sampling_period = (uint64_t)
			(mv_shm->stolen_cpu_time_stats.stolen_cpu_counter_period_nsec);

	BUG_ON(sampling_period == 0);
	*active_stolen_time *= sampling_period;
	*idle_stolen_time *= sampling_period;
	return ret;
}

static int sofia_cpu_get_steal_load(void)
{
	struct vcore_load_info_t *vcore_l;
	uint64_t idle_steal = 0;
	uint64_t active_steal = 0;
	uint64_t steal_now = 0;
	uint64_t timetotal_now = 0;
	uint32_t load = 0, deltatimetotal, deltastolen;

	vcore_l = &get_cpu_var(percpu_vcore_load);


	sofia_get_steal_time(&timetotal_now, &active_steal, &idle_steal);

	steal_now = active_steal + idle_steal;
	deltastolen = ((steal_now -
			vcore_l->timestolen_previous) * 100);
	deltatimetotal = (timetotal_now - vcore_l->
					timetotal_previous);
	load = (deltastolen/deltatimetotal);

	vcore_l->load_previous = load;
	vcore_l->timestolen_previous = steal_now;
	vcore_l->timetotal_previous = timetotal_now;

	put_cpu_var(percpu_vcore_load);
	/* FIXME */
	if (load > 100)
		load = 100;
	else if (load <= 0)
		load = 5;

	pr_debug("xgold load computed = %d\n", load);
	return load;
}

void sofia_set_cpu_frequency(const int cpufreq)
{
	u32 retval;
	pr_debug("%s: ask vmm to change to %d", __func__, cpufreq*1000);
	retval = mv_svc_pm_control(PM_REQ_FREQ_CHNG, 1, cpufreq, 0);

}
EXPORT_SYMBOL(sofia_set_cpu_frequency);

u32 sofia_get_cpu_nb_freq(void)
{
	pr_debug("%s: ask vmm the nuber of supported frequencies", __func__);

	return mv_svc_pm_control(PM_CPU_GET_NUM_FREQ_STEPS, 0, 0, 0);
}
EXPORT_SYMBOL(sofia_get_cpu_nb_freq);

u32 sofia_get_cpu_freq_table(int *freq_table, const int nb_freq)
{
	u32 freq_size = nb_freq * sizeof(u32);

	pr_debug("%s: ask vmm supported frequencies", __func__);

	return mv_svc_pm_control(PM_CPU_GET_FREQ_STEPS,
			(uint32_t)__pa(freq_table), freq_size, 0);

}
EXPORT_SYMBOL(sofia_get_cpu_freq_table);

void sofia_set_cpu_policy(const int freqmin, const int freqmax)
{
	u32 retval;
	u32 freq_requested = 0;

	pr_debug("%s: ask vmm to change policy to min %d kHz, max %d kHz",
			__func__, freqmin*1000, freqmax*1000);
	freq_requested = (freqmax<<16)|((u16)freqmin);
	retval = mv_svc_pm_control(PM_OMP_SET_POLICY, 1, freq_requested, 0);

}
EXPORT_SYMBOL(sofia_set_cpu_policy);

void sofia_thermal_set_cpu_policy(const int freqmin, const int freqmax)
{
	u32 retval;
	u32 freq_requested = 0;


	pr_debug("%s: ask vmm to change policy to min %d kHz, max %d kHz",
			__func__, freqmin*1000, freqmax*1000);
	freq_requested = (freqmax<<16)|((u16)freqmin);
	retval = mv_svc_pm_control(PM_OMP_SET_POLICY, 2, freq_requested, 0);
}
EXPORT_SYMBOL(sofia_thermal_set_cpu_policy);

void sofia_get_cpu_frequency(struct sofia_cpu_freq_t *sh_freqs)
{
	struct pal_shared_data *mv_shm = mv_svc_get_system_shared_data();

	sh_freqs->curfreq = mv_shm->pm_control_shared_data.cpu_clk;
	sh_freqs->minfreq = mv_shm->pm_control_shared_data.vcpu_c0[0];
	sh_freqs->maxfreq = mv_shm->pm_control_shared_data.vcpu_c0[1];
	pr_debug("%s: freq %d kHz, min is %d kHz, max is %d kHz\n", __func__,
		       sh_freqs->curfreq, sh_freqs->minfreq, sh_freqs->maxfreq);
}
EXPORT_SYMBOL(sofia_get_cpu_frequency);

unsigned long arch_scale_freq_power(struct sched_domain *sd, int cpu)
{
	int load;

	/* FIXME: Get the vcpu shared with another vm from bitmask */
	if (static_key_false((&paravirt_steal_power_enabled))
			&& (sofia_is_cpu_shared(cpu))) {
		load = sofia_cpu_get_steal_load();
		return (100 - load) * SCHED_POWER_SCALE / 100;
	} else
		return SCHED_POWER_SCALE;
}

static int __init sofia_cpu_steal_init(void)
{
	struct device_node *np = of_find_node_by_path("/xgold");
	if (!of_find_property(np, "intel,nosteal", NULL)) {
		static_key_slow_inc(&paravirt_steal_enabled);
		static_key_slow_inc(&paravirt_steal_rq_enabled);
		static_key_slow_inc(&paravirt_steal_power_enabled);
	}

	return 0;
}

arch_initcall(sofia_cpu_steal_init);

static struct cpumask _cpumask_shared = { CPU_BITS_NONE };
static struct cpumask *cpumask_shared = &_cpumask_shared;
static unsigned vcpu_map, shared_cpu_map, apic_map, num_of_cpus;
static unsigned apic_vcpu_map[CONFIG_NR_CPUS];

bool sofia_is_cpu_shared(unsigned cpu)
{
	if (cpumask_test_cpu(cpu, cpumask_shared))
		return true;
	else
		return false;
}
EXPORT_SYMBOL(sofia_is_cpu_shared);

bool sofia_is_cpu_exclusive(unsigned cpu)
{
	if (cpumask_test_cpu(cpu, cpumask_shared))
		return false;
	else
		return true;
}
EXPORT_SYMBOL(sofia_is_cpu_exclusive);

unsigned sofia_cpu_get_apicid(unsigned cpu)
{
	return apic_vcpu_map[cpu];
}
EXPORT_SYMBOL(sofia_cpu_get_apicid);

unsigned sofia_get_nr_vcpus(void)
{
	return num_of_cpus;
}
EXPORT_SYMBOL(sofia_get_nr_vcpus);

int sofia_cpu_mapping_init(void)
{
	unsigned vcpu;

	cpumask_clear(cpumask_shared);
	mv_get_cpu_map(&vcpu_map, &shared_cpu_map, &apic_map, &num_of_cpus);

	for (vcpu = 0; vcpu < num_of_cpus; vcpu++) {
		unsigned phys_cpu;
		bool shared;
		phys_cpu = (vcpu_map >> (vcpu * 4)) & 0xF;

		if (shared_cpu_map & BIT(phys_cpu))
			shared = true;
		else
			shared = false;

		apic_vcpu_map[vcpu] = (apic_map >> (phys_cpu * 4)) & 0xF;

		pr_info("vcpu mapping: shared:%s, vcpu:%x, phys:%x, apicid:%x\n",
				shared ? "yes" : "no", vcpu,
				phys_cpu, apic_vcpu_map[vcpu]);
		if (shared)
			cpumask_set_cpu(vcpu, cpumask_shared);
	}

	return 0;
}
EXPORT_SYMBOL(sofia_cpu_mapping_init);
