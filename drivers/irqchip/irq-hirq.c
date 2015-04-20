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
	.xlate = irq_domain_xlate_twocell,
	.map = xgold_irq_domain_map,
};

/*
 * Run software resend of hirq
 */
static void xgold_hirq_resend(unsigned long arg)
{
	struct xgold_irq_chip_data *data = (struct xgold_irq_chip_data *)arg;
	int irq = data->hirq;
	struct irq_desc *desc = irq_to_desc(irq);
	pr_debug("%s(%d)\n", __func__, irq);
	if (desc) {
		local_irq_disable();
		desc->handle_irq(irq, desc);
		local_irq_enable();
	}
}

static uint32_t xgold_irq_hirq_find_mapping(uint32_t irq)
{
	uint32_t index = 0;
	struct xgold_irq_chip_data *data = irq_get_handler_data(irq);
	struct vmm_shared_data *pdata = mv_gal_get_shared_data();
	struct virq_info_t *p_virq = &(pdata->virq_info);
	uint32_t level1, level2;
	pr_debug("%s(%d)-->\n", __func__, irq);

	if (p_virq->lvl1) {
		level1 = __ffs(p_virq->lvl1);
		if (level1 >= 16) {
			pr_err("%s: invalid virq detected\n", __func__);
			BUG();
		}

		if (p_virq->lvl2[level1] == 0) {
			pr_err("%s: error - lvl2 is null...\n", __func__);
			return 0;
		}

		level2 = __ffs(p_virq->lvl2[level1]);
		index = (level1 << 5) + level2;

		p_virq->lvl2[level1] &= ~(1 << level2);
		if (p_virq->lvl2[level1] == 0)
			p_virq->lvl1 &= ~(1 << level1);

		/* we need to re-trigger the irq handler
		   since there are still pending virqs */
		if (p_virq->lvl1)
			tasklet_schedule(&data->hirq_resend);
	} else
		pr_err("spurious virq detected\n");

	if (index >= irq_hirq_offset)
		BUG();

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
	data->flow_handler = handle_fasteoi_irq;
	irq_hirq_offset = xgold_irq_get_hirq_offset(np);

	/* extract info form dt */
	xgold_irq_of_get(np, data);

	if (data->hirq)
		tasklet_init(&data->hirq_resend, xgold_hirq_resend,
						(unsigned long)data);

	/* add linear domain */
	ret |= xgold_irq_domain_add_linear(np,
			data, &xgold_irq_hirq_domain_ops);

	/* Parse, Map and Cascade */
	if (parent)
		ret |= xgold_irq_parse_map_and_cascade(np, data);

	return ret;
}

IRQCHIP_DECLARE(xgold_hirq_vpic, "intel,sofia-hirq", xgold_irq_hirq_of_init);

static inline void xgold_irq_hirq_main_unmask(struct irq_data *data)
{
	return;
}

static inline void xgold_irq_hirq_main_mask(struct irq_data *data)
{
	return;
}

static void xgold_irq_hirq_main_enable(struct irq_data *data)
{
	return;
}

static void xgold_irq_hirq_main_disable(struct irq_data *data)
{
	return;
}

void xgold_irq_hirq_main_eoi(struct irq_data *data)
{
	return;
}

static struct irq_chip xgold_irq_hirq_main_chip = {
	.name = "HIRQ MAIN",
	.irq_mask = xgold_irq_hirq_main_mask,
	.irq_unmask = xgold_irq_hirq_main_unmask,
	.irq_enable = xgold_irq_hirq_main_enable,
	.irq_disable = xgold_irq_hirq_main_disable,
	.irq_eoi = xgold_irq_hirq_main_eoi,
};

static struct irq_domain_ops xgold_irq_hirq_main_domain_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = xgold_irq_domain_map,
};

static int32_t __init xgold_irq_hirq_main_of_init(struct device_node *np,
					struct device_node *parent)
{
	int32_t ret = 0;
	struct xgold_irq_chip_data *data;

	data = kzalloc(sizeof(struct xgold_irq_chip_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->chip = &xgold_irq_hirq_main_chip;
	data->type = XGOLD_IRQ_DOMAIN_N2N;

	/* extract info form dt */
	xgold_irq_of_get(np, data);

	/* add linear domain */
	ret |= xgold_irq_domain_add_linear(np,
			data, &xgold_irq_hirq_main_domain_ops);

	/* Parse, Map and Cascade */
	if (parent)
		ret |= xgold_irq_parse_map_and_cascade(np, data);

	return ret;
}

IRQCHIP_DECLARE(xgold_hirq_main_vpic, "intel,sofia-main-hirq",
		xgold_irq_hirq_main_of_init);
