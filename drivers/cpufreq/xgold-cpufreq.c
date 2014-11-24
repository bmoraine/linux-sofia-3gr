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
#include <linux/reboot.h>
#include <linux/platform_device.h>

#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
#ifdef CONFIG_X86_INTEL_SOFIA
#include <linux/vpower.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#else
#include <vlx/vpower_common.h>
#include <vlx/vpower_prh.h>
#endif
#endif

/*------Notification function------*/
static void vcpufreq_notify(struct work_struct *work);
static DECLARE_WORK(vcpufreq_pre, vcpufreq_notify);
static irqreturn_t process_cpufreq(int irq, void *dev);

static DEFINE_MUTEX(cpufreq_lock);
static bool frequency_locked;

struct xgold_cpufreq_info {
	struct cpufreq_frequency_table *freq_table;
	struct clk *core_clk;
	struct clk *mux_clk;
	struct clk *pll_clk;
	struct clk *bank_clk;
	struct cpufreq_freqs freqs;
	unsigned int irq;
};

static struct xgold_cpufreq_info _xgold_cpufreq_info;
struct xgold_cpufreq_info *xgold_cpu_info = &_xgold_cpufreq_info;

static struct of_device_id xgold_cpufreq_ids[] = {
	{.compatible = "intel,xgold-cpufreq",},
	{},
};

#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
static uint32_t cpufreq_old;
static uint32_t cpufreq_new;
struct cpufreq_policy *stat_policy;

static void vcpufreq_notify(struct work_struct *work)
{
	struct cpufreq_freqs notif_freqs;


	/* prepare data for notification */
	notif_freqs.old = cpufreq_old;
	notif_freqs.new = cpufreq_new;
	notif_freqs.flags = 0;
	cpufreq_notify_transition(stat_policy,
				&notif_freqs,
				CPUFREQ_PRECHANGE);
	cpufreq_notify_transition(stat_policy,
				&notif_freqs,
				CPUFREQ_POSTCHANGE);

	}


static irqreturn_t process_cpufreq(int irq, void *dev)
{
	struct cpufreq_policy *policy = dev;
	cpufreq_old = cpufreq_new;
	cpufreq_new = vpower_system_get_cpu_freq() * 1000;
	pr_debug("cpufreq: hirq, new freq is %d\n", cpufreq_new);
	stat_policy = policy;
	queue_work(system_nrt_wq, &vcpufreq_pre);
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
	pr_debug("cpufreq: get cpufre: %d\n", vpower_system_get_cpu_freq());
#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
	return vpower_system_get_cpu_freq() * 1000;

#else
	return clk_get_rate(xgold_cpu_info->core_clk) / 1000;
#endif
}

static int xgold_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
	int ret;

	cpufreq_frequency_table_get_attr(xgold_cpu_info->freq_table,
						policy->cpu);
	/* TODO: Should we get the bootloader/kernel init frequency ? */

	/*
	   Set the transition latency value
	   FIXME: Find out correct value
	 */
	policy->cpuinfo.transition_latency = 100000;

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
	policy->cur = policy->min = policy->max =
	    /* TODO : vmm to initialise shared mem variable */
		/*xgold_cpufreq_get(policy->cpu);*/
		1040000;
	cpufreq_old = policy->cur;
	cpufreq_new = policy->cur;

	ret = cpufreq_frequency_table_cpuinfo(policy,
				xgold_cpu_info->freq_table);

	ret |= request_irq(xgold_cpu_info->irq, process_cpufreq,
			    IRQF_SHARED, "cpu_clk_change", policy);

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

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	cpufreq_notify_transition(policy, freqs, CPUFREQ_PRECHANGE);
	clk_set_rate(xgold_cpu_info->pll_clk, (freqs->new * 1000));
	cpufreq_notify_transition(policy, freqs, CPUFREQ_POSTCHANGE);
#else
	pr_debug("cpufreq: ask vmm for %d frequency\n", freqs->new / 1000);
	vpower_set_cpu_target_frequency(freqs->new / 1000);
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
/**
 * @notifier
 * @pm_event
 * @v
 *
 * While frequency_locked == true, target() ignores every frequency but
 * locking_frequency. The locking_frequency value is the initial frequency,
 * which is set by the bootloader. In order to eliminate possible
 * inconsistency in clock values, we save and restore frequencies during
 * suspend and resume and block CPUFREQ activities. Note that the standard
 * suspend/resume cannot be used as they are too deep (syscore_ops) for
 * regulator actions.
 */
#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int xgold_cpufreq_pm_notifier(struct notifier_block *notifier,
				       unsigned long pm_event, void *v)
{


	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		mutex_lock(&cpufreq_lock);
		frequency_locked = true;
		mutex_unlock(&cpufreq_lock);


		break;

	case PM_POST_SUSPEND:
		mutex_lock(&cpufreq_lock);
		frequency_locked = false;
		mutex_unlock(&cpufreq_lock);
		break;
	default:
	break;
	}

	return NOTIFY_OK;
}

static struct notifier_block xgold_cpufreq_nb = {
	.notifier_call = xgold_cpufreq_pm_notifier,
};

static int xgold_cpufreq_reboot_notifier(struct notifier_block *this,
						unsigned long code, void *_cmd)
{

	mutex_lock(&cpufreq_lock);

	if (frequency_locked)
		goto out;
	frequency_locked = true;

out:
	mutex_unlock(&cpufreq_lock);
	return NOTIFY_DONE;
}

static struct notifier_block xgold_cpufreq_reboot_nb = {
	.notifier_call = xgold_cpufreq_reboot_notifier,
};
#endif

static int xgold_cpufreq_cpu_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);
	return 0;
}

static struct freq_attr *xgold_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
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
	struct cpufreq_frequency_table *cpufreq_table;
	struct property *prop;

	pr_debug("Entering xgold_cpufreq init");

	np = of_find_matching_node(NULL, xgold_cpufreq_ids);
	if (!np) {
		pr_err("xgold-cpufreq device node not found !");
		return -ENODEV;
	}

	prop = of_find_property(np, "intel,cpufreq-table", &def_len);
	if (prop == NULL) {
		pr_err("cpufreq table property not found in dts");
		return -ENODEV;
	}

	nr_freq = def_len / sizeof(u32);
	freq_table = kzalloc(def_len, GFP_KERNEL);
	if (!freq_table) {
		pr_err("unable to allocate memory for frequencies table");
		return -ENOMEM;
	}

	ret = of_property_read_u32_array(np,
					 "intel,cpufreq-table",
					 (u32 *) freq_table, nr_freq);
	if (ret) {
		pr_err("unable to read frequencies table from dts");
		kfree(freq_table);
		return -ENOMEM;
	}

	cpufreq_table =
	    kzalloc((nr_freq + 1) * sizeof(struct cpufreq_frequency_table),
		    GFP_KERNEL);
	if (!cpufreq_table) {
		pr_err
		    ("Unable to allocate memory for cpufreq frequencies table");
		kfree(freq_table);
		ret = -ENOMEM;
		goto err_cpufreq3;
	}

	xgold_cpu_info->freq_table = cpufreq_table;
	for (i = 0; i < nr_freq; i++)
		cpufreq_table[i].frequency = freq_table[i];

	cpufreq_table[nr_freq].frequency = CPUFREQ_TABLE_END;

	kfree(freq_table);

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
	register_pm_notifier(&xgold_cpufreq_nb);
	register_reboot_notifier(&xgold_cpufreq_reboot_nb);
#endif
	if (cpufreq_register_driver(&xgold_cpufreq_driver)) {
		pr_err("Failed to register cpufreq driver\n");
		ret = -EINVAL;
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
		goto err_cpufreq;
#else
		unregister_pm_notifier(&xgold_cpufreq_nb);
		goto err_cpufreq3;
#endif
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

