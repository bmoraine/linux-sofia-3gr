/*
 * Copyright (c) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/platform_device.h>

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/cpu.h>
#endif

/*------Notification function------*/
static void vcpufreq_notify(unsigned long param);

static DEFINE_MUTEX(cpufreq_lock);
static bool frequency_locked;

struct xgold_cpufreq_info {
	struct cpufreq_frequency_table *freq_table;
	int nr_freq;
	struct clk *core_clk;
	struct clk *mux_clk;
	struct clk *pll_clk;
	struct clk *bank_clk;
	struct cpufreq_freqs freqs;
	unsigned int irq;
	unsigned int latency;
	struct tasklet_struct tasklet_notify;
};

static struct xgold_cpufreq_info _xgold_cpufreq_info;
struct xgold_cpufreq_info *xgold_cpu_info = &_xgold_cpufreq_info;

static struct of_device_id xgold_cpufreq_ids[] = {
	{.compatible = "intel,xgold-cpufreq",},
	{},
};

#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
#ifdef CONFIG_PM
static int xgold_cpu_freq_pm_notifier(struct notifier_block *nb,
					unsigned long val, void *data);
static struct notifier_block xgold_cpufreq_pm_notifier = {
	.notifier_call = xgold_cpu_freq_pm_notifier
};
static int xgold_cpu_freq_notifier(struct notifier_block *nb,
					unsigned long val, void *data);
static struct notifier_block xgold_cpufreq_notifier = {
	.notifier_call = xgold_cpu_freq_notifier
};


static int xgold_cpu_freq_pm_notifier(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct cpufreq_policy *policy =	cpufreq_cpu_get(0);

	if (policy == NULL)
		return -EINVAL;

	if (val == PM_SUSPEND_PREPARE) {
		pr_debug("%s: callback notified\n", __func__);
		cpufreq_driver_target(policy, policy->user_policy.min, 0);
		frequency_locked = true;
	} else if (val == PM_POST_SUSPEND) {
		frequency_locked = false;
	}

	return 0;
}

static int xgold_cpu_freq_notifier(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	struct sofia_cpu_freq_t shmem_freq;
	int min, max;

	if (val == CPUFREQ_NOTIFY) {
		/* if min or max already in shared mem, do not notify, this
		 * means request comes from  other VMs */
		sofia_get_cpu_frequency(&shmem_freq);
		min = (policy->min == shmem_freq.minfreq * 1000) ? -1 :
			(policy->min / 1000);
		max = (policy->max == shmem_freq.maxfreq * 1000) ? -1 :
			(policy->max / 1000);
		/* value 1000/1000=1 means no change in prh */
		mutex_lock(&cpufreq_lock);
		sofia_thermal_set_cpu_policy(min, max);
		mutex_unlock(&cpufreq_lock);
	}
	return 0;
}
#endif

static void vcpufreq_notify(unsigned long param)
{
	struct cpufreq_freqs notif_freqs;
	struct cpufreq_freqs *freqs = &xgold_cpu_info->freqs;
	struct cpufreq_policy *policy =	(struct cpufreq_policy *)param;
	struct sofia_cpu_freq_t shmem_freq;

	pr_debug("cpufreq notify\n");
	/* prepare data for notification */
	notif_freqs.old = freqs->old;
	sofia_get_cpu_frequency(&shmem_freq);
	notif_freqs.new = shmem_freq.curfreq * 1000;
	notif_freqs.flags = 0;
	/* notify prechange only if change comes from other VM */
	if (notif_freqs.new != policy->cur) {
		cpufreq_notify_transition(policy,
				&notif_freqs,
				CPUFREQ_PRECHANGE);
	}
	cpufreq_notify_transition(policy,
				&notif_freqs,
				CPUFREQ_POSTCHANGE);

	}


static irqreturn_t xgold_cpufreq_isr(int irq, void *dev)
{
	struct cpufreq_policy *policy = dev;
	struct sofia_cpu_freq_t shmem_freq;

	tasklet_schedule(&xgold_cpu_info->tasklet_notify);
	sofia_get_cpu_frequency(&shmem_freq);
	pr_debug("cpufreq hirq: shmem_freq = %d, policy_cur = %d\n",
			shmem_freq.curfreq * 1000, policy->cur);
	/* if min or max change need to update policy */
	if ((policy->user_policy.min != (shmem_freq.minfreq * 1000))
		|| (policy->user_policy.max != (shmem_freq.maxfreq * 1000))) {
		pr_debug("other vm requested policy change, cpu is %d\n",
				policy->cpu);
		policy->user_policy.min = shmem_freq.minfreq * 1000;
		policy->user_policy.max = shmem_freq.maxfreq * 1000;
		schedule_work(&policy->update);
	} else if (shmem_freq.curfreq * 1000 == policy->cur) {
		pr_debug("%s: Spurious HIRQ detected!\n", __func__);
	}

	return IRQ_HANDLED;
}
#endif

static int xgold_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy,
				xgold_cpu_info->freq_table);
}

static unsigned int xgold_cpufreq_get(unsigned int cpu)
{
#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
	struct sofia_cpu_freq_t shmem_freq;
	sofia_get_cpu_frequency(&shmem_freq);
	pr_debug("cpufreq: get cpufre: %d\n", shmem_freq.curfreq*1000);
	return (shmem_freq.curfreq * 1000);

#else
	return clk_get_rate(xgold_cpu_info->core_clk) / 1000;
#endif
}

static int xgold_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
	int ret;
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	ret = clk_prepare(xgold_cpu_info->pll_clk);
	if (ret) {
		pr_err("Core clk prepare call failed !");
		return ret;
	}

	ret = clk_set_parent(xgold_cpu_info->mux_clk, xgold_cpu_info->pll_clk);
	if (ret) {
		pr_err("Setting mux parent failed !");
		return ret;
	}

	ret = clk_prepare_enable(xgold_cpu_info->core_clk);
	if (ret) {
		pr_err("Core clk prepare_enable call failed !");
		return ret;
	}

	if (xgold_cpu_info->bank_clk) {
		ret = clk_prepare_enable(xgold_cpu_info->bank_clk);
		if (ret) {
			pr_err("Bank clk prepare_enable call failed !");
			return ret;
		}
	}
#endif
	/*
	 * XGOLD multi cores shares the same clock,
	 * i.e. no individual clock settings possible
	 */
	if (num_online_cpus() == 1) {
		cpumask_copy(policy->related_cpus, cpu_possible_mask);
		cpumask_copy(policy->cpus, cpu_online_mask);
	} else {
		cpumask_setall(policy->cpus);
	}

	ret = cpufreq_generic_init(policy,
			xgold_cpu_info->freq_table, xgold_cpu_info->latency);
	/*
	 * XGOLD multi cores shares the same clock,
	 * i.e. no individual clock settings possible
	 */
	if (num_online_cpus() == 1) {
		cpumask_copy(policy->related_cpus, cpu_possible_mask);
		cpumask_copy(policy->cpus, cpu_online_mask);
	} else {
		cpumask_setall(policy->cpus);
	}


	policy->cur = xgold_cpufreq_get(policy->cpu);

	ret |= request_irq(xgold_cpu_info->irq, xgold_cpufreq_isr,
			    IRQF_SHARED, "cpu_clk_change", policy);

	tasklet_init(&xgold_cpu_info->tasklet_notify,
		vcpufreq_notify, (unsigned long)policy);
	return ret;
}



static int xgold_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int target_freq, unsigned int relation)
{
	unsigned int index;
	struct cpufreq_frequency_table *freq_table = xgold_cpu_info->freq_table;
	struct cpufreq_freqs *freqs = &xgold_cpu_info->freqs;
	int ret = 0;

	mutex_lock(&cpufreq_lock);
	if (frequency_locked)
		goto out;

	freqs->old = policy->cur;

	if (cpufreq_frequency_table_target(policy, freq_table,
					   target_freq, relation, &index)) {
		ret = -EINVAL;
		goto out;
	}

	freqs->new = freq_table[index].frequency;
	freqs->cpu = policy->cpu;

	pr_debug("cpufreq: intarget freqs->new %d freqs->old %d\n", freqs->new,
			freqs->old);
	if (freqs->new == freqs->old)
		goto out;

	cpufreq_notify_transition(policy, freqs, CPUFREQ_PRECHANGE);
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	clk_set_rate(xgold_cpu_info->pll_clk, (freqs->new * 1000));
	cpufreq_notify_transition(policy, freqs, CPUFREQ_POSTCHANGE);
#else
	pr_debug("cpufreq: ask vmm for %d frequency\n", freqs->new / 1000);
	sofia_set_cpu_frequency(freqs->new / 1000);
#endif
out:
	mutex_unlock(&cpufreq_lock);
	return ret;
}

int xgold_cpufreq_suspend(struct cpufreq_policy *policy)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	struct cpufreq_frequency_table *freq_table = xgold_cpu_info->freq_table;
	struct cpufreq_freqs *freqs = &xgold_cpu_info->freqs;

	/* Switch pll clock to lower freq */
	freqs->old = policy->cur;
	freqs->new = freq_table[0].frequency;
	freqs->cpu = policy->cpu;

	if (freqs->new == freqs->old)
		return 0;

	clk_set_rate(xgold_cpu_info->pll_clk, (freqs->new*1000));
/* FIXME: cannot disable PLL if cpu is using on it */
/*	clk_disable(cpu_info->pll_clk); */
/*	clk_unprepare(cpu_info->pll_clk); */
#endif
	return 0;
}

int xgold_cpufreq_resume(struct cpufreq_policy *policy)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	struct cpufreq_freqs *freqs = &xgold_cpu_info->freqs;

/*	clk_prepare(cpu_info->pll_clk); */
/*	clk_enable(cpu_info->pll_clk); */

	/* Switch pll clock to higher freq */
	freqs->new = freqs->old;
	freqs->cpu = policy->cpu;

	if (freqs->new == freqs->old)
		return 0;

	clk_set_rate(xgold_cpu_info->pll_clk, (freqs->new*1000));
#endif
	return 0;
}

static int xgold_cpufreq_cpu_exit(struct cpufreq_policy *policy)

{
	cpufreq_frequency_table_put_attr(policy->cpu);
	tasklet_kill(&xgold_cpu_info->tasklet_notify);
	return 0;
}

static ssize_t thermal_scaling_max_freq_store(struct cpufreq_policy *policy,
		const char *buf, size_t count)
{
	int ret = 0, i;
	unsigned int max;

	ret = sscanf(buf, "%u", &max);
	if (ret != 1)
		return -EINVAL;

	for (i = 0; i < xgold_cpu_info->nr_freq; i++) {
		if (max == xgold_cpu_info->freq_table[i].frequency)
			break;
	}

	if (i == xgold_cpu_info->nr_freq)
		return -EINVAL;

	/* if min or max change need to update policy */
	if (policy->user_policy.max != max) {
		mutex_lock(&cpufreq_lock);
		sofia_thermal_set_cpu_policy(policy->user_policy.min / 1000,
				max / 1000);
		mutex_unlock(&cpufreq_lock);
	}

	return count;
}

static ssize_t thermal_scaling_max_freq_show(struct cpufreq_policy *policy,
				char *buf)
{
	return sprintf(buf, "%u\n", policy->user_policy.max);
}

void set_thermal_scaling_max_freq_to_lowest(void)
{
	struct cpufreq_policy policy;
	unsigned int max;

	cpufreq_get_policy(&policy, 0);
	max = xgold_cpu_info->freq_table[0].frequency;
	pr_err("%s: set the max freq to lowest freq %d\n", __func__, max);

	if (policy.user_policy.max != max) {
		mutex_lock(&cpufreq_lock);
		sofia_thermal_set_cpu_policy(policy.user_policy.min / 1000,
				 max / 1000);
		mutex_unlock(&cpufreq_lock);
	}
}
static struct freq_attr cpufreq_freq_attr_thermal_scaling_max_freq =
	__ATTR_RW(thermal_scaling_max_freq);

static struct freq_attr *xgold_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	&cpufreq_freq_attr_thermal_scaling_max_freq,
	NULL,
};
static struct cpufreq_driver xgold_cpufreq_driver = {
	.flags = CPUFREQ_STICKY,
	.verify = xgold_cpufreq_verify,
	.target = xgold_cpufreq_target,
	.get = xgold_cpufreq_get,
	.init = xgold_cpufreq_cpu_init,
	.exit = xgold_cpufreq_cpu_exit,
	.suspend = xgold_cpufreq_suspend,
	.resume	= xgold_cpufreq_resume,
	.name = "xgold_cpufreq",
	.attr = xgold_cpufreq_attr,
};

static int xgold_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np;
	int ret = -EINVAL, def_len, nr_freq, i;
	unsigned *freq_table;
	unsigned latency;
	unsigned start_limits[2];
	struct cpufreq_frequency_table *cpufreq_table;
	struct property *prop;
	bool use_vmm_table = false;

	pr_debug("Entering xgold_cpufreq init");

	np = of_find_matching_node(NULL, xgold_cpufreq_ids);
	if (!np) {
		pr_err("xgold-cpufreq device node not found !");
		return -ENODEV;
	}

	if (of_find_property(np, "intel,table-from-vmm", NULL)) {
		/* frequency table is given by vmm */
		nr_freq = sofia_get_cpu_nb_freq();
		if (nr_freq <= 0)
			pr_err("can't read cpufreq table from vmm\n");
		else {
			use_vmm_table = true;
			goto skip_cpufreq_table;
		}
	}

	/* frequency table is coming from device tree */
	prop = of_find_property(np, "intel,cpufreq-table", &def_len);
	if (!prop) {
		pr_err("can't get cpufreq table from device tree\n");
		return -EINVAL;
	}
	nr_freq = def_len / sizeof(u32);

skip_cpufreq_table:
	xgold_cpu_info->nr_freq = nr_freq;
	freq_table = kzalloc(nr_freq, GFP_KERNEL);
	if (!freq_table) {
		pr_err("unable to allocate memory for frequencies table");
		return -ENOMEM;
	}

	if (use_vmm_table) {
		ret = sofia_get_cpu_freq_table(freq_table, nr_freq);

		for (i = 0; i < nr_freq; i++)
			freq_table[i] = freq_table[i] * 1000;
	} else
		ret = of_property_read_u32_array(np,
					 "intel,cpufreq-table",
					 (u32 *) freq_table, nr_freq);

	if (ret) {
		pr_err("unable to read frequencies table\n");
		kfree(freq_table);
		return -ENOMEM;
	}
	cpufreq_table = kzalloc((nr_freq + 1) *
			sizeof(struct cpufreq_frequency_table), GFP_KERNEL);
	if (!cpufreq_table) {
		pr_err("can't allocate cpu frequencies table\n");
		kfree(freq_table);
		ret = -ENOMEM;
		goto err_cpufreq3;
	}

	xgold_cpu_info->freq_table = cpufreq_table;
	for (i = 0; i < nr_freq; i++)
		cpufreq_table[i].frequency = freq_table[i];

	cpufreq_table[nr_freq].frequency = CPUFREQ_TABLE_END;

	kfree(freq_table);
	of_property_read_u32(np, "intel,clock_latency", &latency);
	xgold_cpu_info->latency = latency;

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	xgold_cpu_info->core_clk = of_clk_get_by_name(np, "core");
	if (IS_ERR(xgold_cpu_info->core_clk)) {
		pr_err("core clock not found in device tree !");
		ret = -ENODEV;
		goto err_cpufreq3;
	}

	xgold_cpu_info->mux_clk = of_clk_get_by_name(np, "mux");
	if (IS_ERR(xgold_cpu_info->mux_clk)) {
		pr_err("mux clock not found in device tree !");
		ret = -ENODEV;
		goto err_cpufreq1;
	}

	xgold_cpu_info->pll_clk = of_clk_get_by_name(np, "pll");
	if (IS_ERR(xgold_cpu_info->pll_clk)) {
		pr_err("pll clock not found in device tree !");
		ret = -ENODEV;
		goto err_cpufreq2;
	}

	xgold_cpu_info->bank_clk = of_clk_get_by_name(np, "bank");
	if (IS_ERR(xgold_cpu_info->bank_clk))
		xgold_cpu_info->bank_clk = NULL;
#else
	xgold_cpu_info->irq = platform_get_irq_byname(pdev, "CPU_CLK_CHANGE");
	pr_info("%s:platform_get_irq_byname(CPU_CLK_CHANGE) (virq:%d)\n",
			__func__, xgold_cpu_info->irq);
#endif
	if (cpufreq_register_driver(&xgold_cpufreq_driver)) {
		pr_err("Failed to register cpufreq driver\n");
		ret = -EINVAL;
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
		goto err_cpufreq;
#else
		goto err_cpufreq3;
#endif
	}
#ifdef CONFIG_PM
	register_pm_notifier(&xgold_cpufreq_pm_notifier);
#endif
	cpufreq_register_notifier(&xgold_cpufreq_notifier,
			CPUFREQ_POLICY_NOTIFIER);

	if (!of_property_read_u32_array(np, "intel,start-freq-limits",
				(u32 *)start_limits, 2)) {
		pr_info("limit start frequency to min: %d, max:%d\n",
				start_limits[0], start_limits[1]);
		mutex_lock(&cpufreq_lock);
		sofia_thermal_set_cpu_policy(start_limits[0] / 1000,
				start_limits[1] / 1000);
		mutex_unlock(&cpufreq_lock);
	}
	return 0;
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
err_cpufreq:
	if (xgold_cpu_info->bank_clk)
		clk_put(xgold_cpu_info->bank_clk);
	clk_put(xgold_cpu_info->pll_clk);
err_cpufreq2:
	clk_put(xgold_cpu_info->mux_clk);
err_cpufreq1:
	clk_put(xgold_cpu_info->core_clk);
#endif
err_cpufreq3:
	kfree(cpufreq_table);
	return ret;
}

static int xgold_cpufreq_remove(struct platform_device *pdev)
{
	return cpufreq_unregister_driver(&xgold_cpufreq_driver);
}

static const struct of_device_id xgold_cpufreq_of_match[] = {
	{
		.compatible = "intel,xgold-cpufreq"},
	{ }
};
MODULE_DEVICE_TABLE(of, xgold_cpudfreq_of_match);

static struct platform_driver xgold_cpufreq_platdrv = {
	.probe          = xgold_cpufreq_probe,
	.remove         = xgold_cpufreq_remove,
	.driver = {
		.name   = "xgold-cpufreq",
		.owner  = THIS_MODULE,
		.of_match_table = xgold_cpufreq_of_match,
	},
};

module_platform_driver(xgold_cpufreq_platdrv);

MODULE_DESCRIPTION("Cpufreq driver for xgold");
MODULE_LICENSE("GPL");

