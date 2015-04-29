/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <asm/prom.h>
#include <sofia/mv_hypercalls.h>
#include <linux/irqchip/irq_xgold.h>

#include "irqchip.h"

static struct vpic_chip_data {
	struct irq_domain *domain;
	unsigned int nr_irqs;
	void __iomem *reg_base;
	void __iomem *entries_base;
} vpic_data __read_mostly;

static inline unsigned int vpic_irq(struct irq_data *d)
{
	return d->hwirq;
}

unsigned int sofia_irq_to_vector(unsigned int irq)
{
	return irq;
}

unsigned int sofia_vector_to_irq(unsigned int vector)
{
	return vector;
}

static DEFINE_RAW_SPINLOCK(vector_lock);

void lock_vector_lock(void)
{
	/* Used to the online set of cpus does not change
	 * during assign_irq_vector.
	 */
	raw_spin_lock(&vector_lock);
}

void unlock_vector_lock(void)
{
	raw_spin_unlock(&vector_lock);
}


static struct irq_cfg *alloc_irq_cfg(void)
{
	struct irq_cfg *cfg;

	cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		return NULL;
	if (!zalloc_cpumask_var(&cfg->domain, GFP_KERNEL))
		goto out_cfg;
	if (!zalloc_cpumask_var(&cfg->old_domain, GFP_KERNEL))
		goto out_domain;
	return cfg;
out_domain:
	free_cpumask_var(cfg->domain);
out_cfg:
	kfree(cfg);
	return NULL;
}

static int
__assign_irq_vector(int irq, struct irq_cfg *cfg, const struct cpumask *mask)
{
	int err;
	cpumask_var_t tmp_mask;

	if (cfg == NULL)
		BUG();

	if (cfg->move_in_progress)
		return -EBUSY;

	if (!alloc_cpumask_var(&tmp_mask, GFP_ATOMIC))
		return -ENOMEM;

	err = -ENOSPC;
#if 0
	/* Affinity changes  or first vector request ?*/
	if (cfg->vector == (u8)VECTOR_UNDEFINED) {
		/* First vector request */
		cfg->vector = sofia_irq_to_vector(irq);
		/*
		cpumask_clear(cfg->old_domain);
		cpumask_clear(cfg->domain);
		*/
		cpumask_and(tmp_mask, mask, cpu_online_mask);

		for_each_cpu(cpu, tmp_mask)
			per_cpu(vector_irq, cpu)[cfg->vector] = irq;

		cpumask_copy(cfg->domain, tmp_mask);
		cpumask_copy(cfg->old_domain, tmp_mask);

		err = 0;
	}

	free_cpumask_var(tmp_mask);
	return err;
#else
	/* Only try and allocate irqs on cpus that are present */
	cpumask_clear(cfg->old_domain);
/*	while (cpu < nr_cpu_ids) { */
	while (1) {
		int new_cpu, vector;
		unsigned core = smp_processor_id();

		apic->vector_allocation_domain(core, tmp_mask, mask);
		/* Check whether the vector is already installed,
		 * affinity domain correctly set, etc..*/
		if (cpumask_subset(tmp_mask, cfg->domain)) {
			err = 0;
			if (cpumask_equal(tmp_mask, cfg->domain))
				break;
			/*
			 * New cpumask using the vector is a proper subset of
			 * the current in use mask. So cleanup the vector
			 * allocation for the members that are not used anymore.
			 */
			cpumask_andnot(cfg->old_domain, cfg->domain, tmp_mask);
			cfg->move_in_progress =
			   cpumask_intersects(cfg->old_domain, cpu_online_mask);
			cpumask_and(cfg->domain, cfg->domain, tmp_mask);

			break;
		}
		/* */
		vector = sofia_irq_to_vector(irq);
#if 0
		vector = current_vector;
		offset = current_offset;
next:
		vector += 16;
		if (vector >= first_system_vector) {
			offset = (offset + 1) % 16;
			vector = FIRST_EXTERNAL_VECTOR + offset;
		}

		if (unlikely(current_vector == vector)) {
			cpumask_or(cfg->old_domain, cfg->old_domain, tmp_mask);
			cpumask_andnot(tmp_mask, mask, cfg->old_domain);
			cpu = cpumask_first_and(tmp_mask, cpu_online_mask);
			continue;
		}

		if (test_bit(vector, used_vectors))
			goto next;

		for_each_cpu_and(new_cpu, tmp_mask, cpu_online_mask) {
			if (per_cpu(vector_irq, new_cpu)[vector] > VECTOR_UNDEFINED)
				goto next;
		}
		/* Found one! */
		current_vector = vector;
		current_offset = offset;
#endif
		if (cfg->vector != (u8) VECTOR_UNDEFINED) {
			cpumask_copy(cfg->old_domain, cfg->domain);
			cfg->move_in_progress =
			   cpumask_intersects(cfg->old_domain, cpu_online_mask);
		}
		for_each_cpu_and(new_cpu, tmp_mask, cpu_online_mask)
			per_cpu(vector_irq, new_cpu)[vector] = irq;
		cfg->vector = vector;
		cpumask_copy(cfg->domain, tmp_mask);
		err = 0;
		break;
	}
	free_cpumask_var(tmp_mask);
	return err;
#endif
}


int assign_irq_vector(int irq, struct irq_cfg *cfg, const struct cpumask *mask)
{
	int err;
	unsigned long flags;

	raw_spin_lock_irqsave(&vector_lock, flags);
	err = __assign_irq_vector(irq, cfg, mask);
	raw_spin_unlock_irqrestore(&vector_lock, flags);
	return err;
}


#if 1

#ifdef CONFIG_SMP
void send_cleanup_vector(struct irq_cfg *cfg)
{
	cpumask_var_t cleanup_mask;

	if (unlikely(!alloc_cpumask_var(&cleanup_mask, GFP_ATOMIC))) {
		unsigned int i;
		for_each_cpu_and(i, cfg->old_domain, cpu_online_mask)
			apic->send_IPI_mask(cpumask_of(i), IRQ_MOVE_CLEANUP_VECTOR);
	} else {
		cpumask_and(cleanup_mask, cfg->old_domain, cpu_online_mask);
		apic->send_IPI_mask(cleanup_mask, IRQ_MOVE_CLEANUP_VECTOR);
		free_cpumask_var(cleanup_mask);
	}
	cfg->move_in_progress = 0;
}

static struct irq_cfg *irq_cfg(unsigned int irq)
{
	return irq_get_chip_data(irq);
}


asmlinkage void smp_irq_move_cleanup_interrupt(void)
{
	unsigned vector, me;

	ack_APIC_irq();
	irq_enter();
	exit_idle();

	me = smp_processor_id();
	for (vector = FIRST_EXTERNAL_VECTOR; vector < NR_VECTORS; vector++) {
		int irq;
		unsigned int irr;
		struct irq_desc *desc;
		struct irq_cfg *cfg;
		irq = __this_cpu_read(vector_irq[vector]);

		if (irq <= VECTOR_UNDEFINED)
			continue;

		desc = irq_to_desc(irq);
		if (!desc)
			continue;

		cfg = irq_cfg(irq);
		if (!cfg)
			continue;

		raw_spin_lock(&desc->lock);

		/*
		 * Check if the irq migration is in progress. If so, we
		 * haven't received the cleanup request yet for this irq.
		 */
		if (cfg->move_in_progress)
			goto unlock;

		if (vector == cfg->vector && cpumask_test_cpu(me, cfg->domain))
			goto unlock;

		irr = apic_read(APIC_IRR + (vector / 32 * 0x10));
		/*
		 * Check if the vector that needs to be cleanedup is
		 * registered at the cpu's IRR. If so, then this is not
		 * the best time to clean it up. Lets clean it up in the
		 * next attempt by sending another IRQ_MOVE_CLEANUP_VECTOR
		 * to myself.
		 */
		if (irr  & (1 << (vector % 32))) {
			apic->send_IPI_self(IRQ_MOVE_CLEANUP_VECTOR);
			goto unlock;
		}
		__this_cpu_write(vector_irq[vector], -1);
unlock:
		raw_spin_unlock(&desc->lock);
	}

	irq_exit();
}

static void __irq_complete_move(struct irq_cfg *cfg, unsigned vector)
{
	unsigned me;

	if (likely(!cfg->move_in_progress))
		return;

	me = smp_processor_id();

	if (vector == cfg->vector && cpumask_test_cpu(me, cfg->domain))
		send_cleanup_vector(cfg);
}
#if 0
static void irq_complete_move(struct irq_cfg *cfg)
{
	__irq_complete_move(cfg, ~get_irq_regs()->orig_ax);
}
#endif
void irq_force_complete_move(int irq)
{
	struct irq_cfg *cfg = irq_get_chip_data(irq);

	if (!cfg)
		return;

	__irq_complete_move(cfg, cfg->vector);
}
#else
static inline void irq_complete_move(struct irq_cfg *cfg) { }
#endif

/* Based on io_apic.c implementation */
void __setup_vector_irq(int cpu)
{
	/* Initialize vector_irq on a new cpu */
	int irq, vector;
	struct irq_cfg *cfg;

	/*
	 * vector_lock will make sure that we don't run into irq vector
	 * assignments that might be happening on another cpu in parallel,
	 * while we setup our initial vector to irq mappings.
	 */
	raw_spin_lock(&vector_lock);
	/* Mark the inuse vectors */
	for_each_active_irq(irq) {
		cfg = irq_get_chip_data(irq);
		if (!cfg)
			continue;

		if (!cpumask_test_cpu(cpu, cfg->domain))
			continue;
		vector = cfg->vector;
		per_cpu(vector_irq, cpu)[vector] = irq;
	}
	/* Mark the free vectors */
	for (vector = 0; vector < NR_VECTORS; ++vector) {
		irq = per_cpu(vector_irq, cpu)[vector];
		if (irq <= VECTOR_UNDEFINED)
			continue;

		cfg = irq_get_chip_data(irq);
		if (!cpumask_test_cpu(cpu, cfg->domain))
			per_cpu(vector_irq, cpu)[vector] = VECTOR_UNDEFINED;
	}
	raw_spin_unlock(&vector_lock);
}

/* void __setup_vector_irq(int cpu) { } */
#endif

static unsigned int sofia_vpic_irq_startup(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	struct irq_cfg *cfg;
	unsigned affinity;

	cfg = irq_get_chip_data(irq);
	if (!cfg)
		return -ENXIO;
	/* Vector assignement is fixed in sofia platform,
	 * but this will set correctly the domain mask
	 * */
	assign_irq_vector(irq, cfg, apic->target_cpus());
	affinity = cpumask_bits(cfg->domain)[0];
	mv_virq_request(cfg->vector, affinity);
	mv_virq_unmask(cfg->vector);
	return 0;
}

static void sofia_vpic_irq_shutdown(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	unsigned int vect = sofia_irq_to_vector(irq);

	/* TODO: Free the vector request */
	mv_virq_unmask(vect);
}

void sofia_vpic_irq_enable(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	unsigned int vect = sofia_irq_to_vector(irq);
	mv_virq_unmask(vect);
}

void sofia_vpic_irq_disable(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	unsigned int vect = sofia_irq_to_vector(irq);
	mv_virq_mask(vect);
}

void sofia_vpic_irq_mask(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	unsigned int vect = sofia_irq_to_vector(irq);
	mv_virq_mask(vect);
}

void sofia_vpic_irq_unmask(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	unsigned int vect = sofia_irq_to_vector(irq);
	mv_virq_unmask(vect);
}

void sofia_vpic_irq_eoi(struct irq_data *data)
{
	unsigned int irq =  vpic_irq(data);
	unsigned int vect = sofia_irq_to_vector(irq);
	mv_virq_eoi(vect);
}

#ifdef CONFIG_PM
static int sofia_vpic_set_wake(struct irq_data *data, unsigned on)
{
	unsigned int irq =  vpic_irq(data);
	return xgold_irq_set_wake(data, irq, on, WAKE_ID_DBB);
}
#endif

static int sofia_vpic_set_affinity(struct irq_data *data,
			       const struct cpumask *mask,
			       bool force)
{
	struct irq_cfg *cfg = data->chip_data;
	unsigned int irq = data->irq;
	unsigned int vect = sofia_irq_to_vector(irq);
	unsigned int dest_id;
	int err;

	if (!config_enabled(CONFIG_SMP))
		return -1;

	if (!cpumask_intersects(mask, cpu_online_mask))
		return -EINVAL;

	err = assign_irq_vector(irq, cfg, mask);
	if (err) {
		pr_debug("%s: Error while assigning vector %d\n",
				__func__, cfg->vector);
		return err;
	}

	err = apic->cpu_mask_to_apicid_and(mask, cfg->domain, &dest_id);
	if (err) {
		if (assign_irq_vector(irq, cfg, data->affinity))
			pr_debug("Failed to recover vector for irq %d\n", irq);
		return err;
	}
	cpumask_copy(data->affinity, mask);

	mv_virq_set_affinity(vect, dest_id);

	return IRQ_SET_MASK_OK_NOCOPY;
}

static struct irq_chip sofia_vpic_chip = {
	.name = "VPIC",
	.irq_startup = sofia_vpic_irq_startup,
	.irq_shutdown = sofia_vpic_irq_shutdown,
	.irq_mask = sofia_vpic_irq_mask,
	.irq_unmask = sofia_vpic_irq_unmask,
	.irq_enable = sofia_vpic_irq_enable,
	.irq_disable = sofia_vpic_irq_disable,
	.irq_eoi = sofia_vpic_irq_eoi,
	.irq_set_affinity   = sofia_vpic_set_affinity,
#ifdef CONFIG_PM
	.irq_set_wake = sofia_vpic_set_wake,
#endif
};

void __init sofia_vpic_fixup_affinity(void)
{
	int irq;
	struct irq_data *idata;
	struct irq_cfg *cfg;
	const struct cpumask *mask;

	for_each_active_irq(irq) {
		cfg = irq_get_chip_data(irq);
		if (cpumask_empty(cfg->domain))
			continue;

		idata = irq_get_irq_data(irq);
		if (!idata)
			return;
		/*
		 * Honour affinities which have been set in early boot
		 */
		if (!irqd_can_balance(idata) || irqd_affinity_was_set(idata))
			mask = idata->affinity;
		else
			mask = apic->target_cpus();
		sofia_vpic_set_affinity(idata, mask, false);
	}
}

static int sofia_vpic_irq_domain_map(struct irq_domain *d, unsigned int irq,
			      irq_hw_number_t hw)
{
	int vector, cpu;
	struct irq_cfg *cfg;

	cfg = alloc_irq_cfg();
	if (cfg == NULL)
		BUG();
	cfg->vector = VECTOR_UNDEFINED;

	irq_set_chip_data(irq, cfg);
	irq_clear_status_flags(irq, IRQ_NOREQUEST);
	irq_set_chip_and_handler(irq, &sofia_vpic_chip, handle_fasteoi_irq);

	if (irq > 31) {
		vector = sofia_irq_to_vector(irq);
		for_each_possible_cpu(cpu)
			per_cpu(vector_irq, cpu)[vector] = VECTOR_UNDEFINED;
	}
	return 0;
}

const struct irq_domain_ops sofia_vpic_irq_domain_ops = {
	.map = sofia_vpic_irq_domain_map,
	.xlate = irq_domain_xlate_onecell,
};

/* Return number of vpic irqs */
unsigned __init vpic_get_nr_of_irqs(struct device_node *np)
{
	unsigned vpic_irqs, vpic_irqs_default = 256;
	if (of_property_read_u32(np, "intel,vpic-irqs", &vpic_irqs)) {
		pr_warn("%s: vpic-irqs not specified in dts - set it to: %d\n",
				__func__, vpic_irqs_default);
		vpic_irqs = vpic_irqs_default;
	}
	return vpic_irqs;
}

int __init vpic_of_init(struct device_node *np, struct device_node *parent)
{
	struct irq_domain *id;
	unsigned int vpic_irqs;
	int ret;
	pr_info("%s: Initializing SoFIA vpic\n", __func__);
	vpic_data.nr_irqs = vpic_get_nr_of_irqs(np);
	vpic_irqs = vpic_data.nr_irqs;

	/* Map io even if not used - confortable to have it in perfile */
	vpic_data.reg_base = of_iomap(np, 0);
	vpic_data.entries_base = of_iomap(np, 1);
	pr_info("%s: xgold-vpic register remapping: Reg: %p, entries %p\n",
			__func__, vpic_data.reg_base, vpic_data.entries_base);

	id = irq_domain_add_linear(np, vpic_irqs, &sofia_vpic_irq_domain_ops,
			(void *)&vpic_data);

	BUG_ON(!id);
	vpic_data.domain  = id;

	ret = irq_create_strict_mappings(id, 0, 0, vpic_irqs);
	if (ret)
		pr_err("%s: Error mapping legacy IRQs: %d\n", __func__, ret);

	of_ioapic = 1;

	x86_init.irqs.fixup_affinity = sofia_vpic_fixup_affinity;

	return 0;
}

IRQCHIP_DECLARE(sofia_vpic, "intel,sofia-vpic", vpic_of_init);
