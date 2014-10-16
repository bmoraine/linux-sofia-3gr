/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/cpu_pm.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/irqchip/irq_xgold.h>

#include "irqchip.h"

#include <sofia/mv_gal.h>
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>

static DEFINE_SPINLOCK(hirq_lock);
static uint32_t irq_hirq_offset;

static inline uint32_t vpic_irq(struct irq_data *d)
{
	return d->hwirq;
}

static uint32_t sofia_irq_to_vector(uint32_t irq)
{
	return irq + irq_hirq_offset;
}

static inline void xgold_irq_hirq_unmask(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = sofia_irq_to_vector(irq);
	pr_debug("%s: mv_virq_unmask(%d) - hwirq=%d\n",
			__func__, vect, irq);
	spin_lock(&hirq_lock);
	mv_virq_unmask(vect);
	spin_unlock(&hirq_lock);
}

static inline void xgold_irq_hirq_mask(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = sofia_irq_to_vector(irq);
	pr_debug("%s: mv_virq_mask(%d) - hwirq=%d\n",
			__func__, vect, irq);
	spin_lock(&hirq_lock);
	mv_virq_mask(vect);
	spin_unlock(&hirq_lock);
}

static void xgold_irq_hirq_enable(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = sofia_irq_to_vector(irq);
	pr_debug("%s: mv_guest_request_virq(%d, 1) - hwirq=%d\n",
			__func__, vect, irq);
	spin_lock(&hirq_lock);
	mv_virq_request(vect, 1);
	pr_debug("%s: mv_virq_unmask(%d) - hwirq=%d\n",
			__func__, vect, irq);
	mv_virq_unmask(vect);
	spin_unlock(&hirq_lock);
}

static void xgold_irq_hirq_disable(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = sofia_irq_to_vector(irq);
	pr_debug("%s: mv_virq_mask(%d) - hwirq=%d\n",
			__func__, vect, irq);
	spin_lock(&hirq_lock);
	mv_virq_mask(vect);
	spin_unlock(&hirq_lock);
}

void xgold_irq_hirq_eoi(struct irq_data *data)
{
	uint32_t irq = vpic_irq(data);
	uint32_t vect = sofia_irq_to_vector(irq);
	pr_debug("%s: mv_virq_eoi(%d) - hwirq=%d\n",
			__func__, vect, irq);
	spin_lock(&hirq_lock);
	mv_virq_eoi(vect);
	spin_unlock(&hirq_lock);
}

static struct irq_chip xgold_irq_hirq_chip = {
	.name = "HIRQ",
	.irq_mask = xgold_irq_hirq_mask,
	.irq_unmask = xgold_irq_hirq_unmask,
	.irq_enable = xgold_irq_hirq_enable,
	.irq_disable = xgold_irq_hirq_disable,
	.irq_eoi = xgold_irq_hirq_eoi,
};

static struct irq_domain_ops xgold_irq_hirq_domain_ops = {
	.xlate = xgold_irq_domain_xlate,
	.map = xgold_irq_domain_map,
};

static uint32_t xgold_irq_hirq_find_mapping(uint32_t irq)
{
	uint32_t virq, index = 0;
	struct vmm_shared_data *pdata = mv_gal_get_shared_data();
	pr_debug("%s(%d)-->\n", __func__, irq);
	if (pdata) {
		virq = pdata->triggering_xirq;
		if (virq < irq_hirq_offset)
			pr_err("%s: vmm_shared_data:%d < offset:%d\n",
					__func__, virq, irq_hirq_offset);
		else {
			index = virq - irq_hirq_offset;
			pr_debug("%s: VIRQ%d from VMM shared data => index:%d\n",
					__func__, virq, index);
		}
	} else
		pr_err("%s: get_hirq_shared_data returns: %p\n",
				__func__, pdata);
	return index;
}

unsigned __init xgold_irq_get_hirq_offset(struct device_node *np)
{
	unsigned hirq_offset, hirq_offset_default = 512;
	if (of_property_read_u32(np, "intel,hirq_offset", &hirq_offset)) {
		pr_warn("%s: hirq_offset not specified in dts - set it to: %d\n",
				__func__, hirq_offset_default);
		hirq_offset = hirq_offset_default;
	}
	return hirq_offset;
}

/*
 * Entry point for HIRQ IRQ. called from of_irq_init
 */
static int32_t __init xgold_irq_hirq_of_init(struct device_node *np,
					struct device_node *parent)
{
	int32_t ret = 0;
	struct xgold_irq_chip_data *data;

	data = kzalloc(sizeof(struct xgold_irq_chip_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->chip = &xgold_irq_hirq_chip;
	data->type = XGOLD_IRQ_DOMAIN_CUST;
	data->find_mapping = xgold_irq_hirq_find_mapping;
	irq_hirq_offset = xgold_irq_get_hirq_offset(np);

	/* extract info form dt */
	xgold_irq_of_get(np, data);

	/* add linear domain */
	ret |= xgold_irq_domain_add_linear(np,
			data, &xgold_irq_hirq_domain_ops);

	/* Parse, Map and Cascade */
	if (parent)
		ret |= xgold_irq_parse_map_and_cascade(np, data);

	return ret;
}

IRQCHIP_DECLARE(xgold_hirq_vpic, "intel,sofia-hirq", xgold_irq_hirq_of_init);
