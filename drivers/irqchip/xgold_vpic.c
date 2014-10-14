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

#include "irqchip.h"
#include "irq-xgold.h"

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

void sofia_vpic_irq_enable(struct irq_data *data)
{
	unsigned int irq = vpic_irq(data);
	unsigned int vect = sofia_irq_to_vector(irq);
	pr_debug("%s: mv_virq_request(%d, 1)\n", __func__, vect);
	mv_virq_request(vect, 1);
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
	unsigned int irq = vpic_irq(data);
	unsigned int vect = sofia_irq_to_vector(irq);
	unsigned bit_mask = cpumask_bits(mask)[0];

	if (!config_enabled(CONFIG_SMP))
		return -1;

	mv_virq_set_affinity(vect, bit_mask);

	return 0;
}

static struct irq_chip sofia_vpic_chip = {
	.name = "SoFIA VPIC",
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

static int sofia_vpic_irq_domain_map(struct irq_domain *d, unsigned int irq,
			      irq_hw_number_t hw)
{
	int vector, cpu;
	irq_set_chip_and_handler(irq, &sofia_vpic_chip, handle_fasteoi_irq);
	irq_set_chip_data(irq, d->host_data);

	/* FIXME: Why are we filling this vector_irq table ? */
	if (irq > 31) {
		vector = sofia_irq_to_vector(irq);
		for_each_possible_cpu(cpu)
			per_cpu(vector_irq, cpu)[vector] = irq;
	}
	return 0;
}

static int sofia_vpic_irq_domain_xlate(struct irq_domain *d,
				struct device_node *controller,
				const u32 *intspec,
				unsigned int intsize,
				unsigned long *out_hwirq,
				unsigned int *out_type)
{
	pr_debug("%s: intspec.. [0]: %x, [1]: %x, [2]: %x\n",
			__func__, intspec[0], intspec[1], intspec[2]);
	/* Get the interrupt number */
	*out_hwirq = intspec[0];
	/* Type set as none - VMM is handling that */
	*out_type = IRQ_TYPE_NONE;
	return 0;
}

const struct irq_domain_ops sofia_vpic_irq_domain_ops = {
	.map = sofia_vpic_irq_domain_map,
	.xlate = sofia_vpic_irq_domain_xlate,
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
	return 0;
}

IRQCHIP_DECLARE(sofia_vpic, "intel,sofia-vpic", vpic_of_init);
