/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip/irq_xgold.h>

#include "irqchip.h"

/* eint type */
#define XGOLD_IRQ_TYPE_EDGE_DISABLED		0
#define XGOLD_IRQ_TYPE_EDGE_RISING		1
#define XGOLD_IRQ_TYPE_EDGE_FALLING		2
#define XGOLD_IRQ_TYPE_EDGE_BOTH		3
#define XGOLD_IRQ_TYPE_LEVEL_DISABLED		0
#define XGOLD_IRQ_TYPE_LEVEL_LOW		2
#define XGOLD_IRQ_TYPE_LEVEL_HIGH		3


struct irq_domain *irq_eint_domain;
struct irq_domain *xgold_irq_eint_get_domain(void)
{
	return irq_eint_domain;
}
EXPORT_SYMBOL(xgold_irq_eint_get_domain);

static DEFINE_SPINLOCK(eint_lock);

static void xgold_irq_eint_ack(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	xgold_irq_write(chipdata, chipdata->ack[irq], 1, XGOLD_WO);
}

static inline void xgold_irq_eint_unmask(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	if (chipdata->mask[irq]) {
		spin_lock(&eint_lock);
		if (chipdata->preack[irq])
			xgold_irq_eint_ack(data);

		if (chipdata->unmask[irq])
			xgold_irq_write(chipdata,
				chipdata->mask[irq], 1, XGOLD_WO);
		else
			xgold_irq_write(chipdata,
				chipdata->mask[irq], 1, XGOLD_RW);
		spin_unlock(&eint_lock);
	}
}

static inline void xgold_irq_eint_mask(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	if (chipdata->mask[irq]) {
		spin_lock(&eint_lock);
		if (chipdata->unmask[irq])
			xgold_irq_write(chipdata,
				chipdata->unmask[irq], 1, XGOLD_WO);
		else
			xgold_irq_write(chipdata,
				chipdata->mask[irq], 0, XGOLD_RW);
		spin_unlock(&eint_lock);
	}
}

static void xgold_irq_eint_maskack(struct irq_data *data)
{
	xgold_irq_eint_mask(data);
	xgold_irq_eint_ack(data);
}

static int xgold_irq_eint_set_type(struct irq_data *data, unsigned int type)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	u32 edge = XGOLD_IRQ_TYPE_EDGE_DISABLED;
	u32 level = XGOLD_IRQ_TYPE_LEVEL_DISABLED;

	if ((!chipdata->edge[irq]) || (!chipdata->level[irq]))
		return 0;

	/* do not configure as !Linux interrupt */
	if (chipdata->virq[irq])
		return 0;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		pr_debug("%s: IRQ_TYPE_EDGE_RISING\n", __func__);
		edge = XGOLD_IRQ_TYPE_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		pr_debug("%s: IRQ_TYPE_EDGE_FALLING\n", __func__);
		edge = XGOLD_IRQ_TYPE_EDGE_FALLING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		pr_debug("%s: IRQ_TYPE_EDGE_BOTH\n", __func__);
		edge = XGOLD_IRQ_TYPE_EDGE_BOTH;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		pr_debug("%s: IRQ_TYPE_LEVEL_LOW\n", __func__);
		level = XGOLD_IRQ_TYPE_LEVEL_LOW;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		pr_debug("%s: IRQ_TYPE_LEVEL_HIGH\n", __func__);
		level = XGOLD_IRQ_TYPE_LEVEL_HIGH;
		break;

	default:
		pr_err("%s: No such irq type %d. exit...", __func__, type);
		return -EINVAL;
	}
	spin_lock(&eint_lock);
	xgold_irq_write(chipdata, chipdata->edge[irq], edge, XGOLD_RW);
	xgold_irq_write(chipdata, chipdata->level[irq], level, XGOLD_RW);
	spin_unlock(&eint_lock);

	return 0;
}

#ifdef CONFIG_PM
static int xgold_irq_eint_set_wake(struct irq_data *data, unsigned on)
{
	u32 irq = data->hwirq;
	return xgold_irq_set_wake(data, irq, on, WAKE_ID_DBB);
}
#endif

static struct irq_chip xgold_irq_eint_chip = {
	.name = "EINT",
	.irq_mask = xgold_irq_eint_mask,
	.irq_disable = xgold_irq_eint_mask,
	.irq_unmask = xgold_irq_eint_unmask,
	.irq_mask_ack = xgold_irq_eint_maskack,
	.irq_ack = xgold_irq_eint_ack,
	.irq_set_type = xgold_irq_eint_set_type,
#ifdef CONFIG_PM
	.irq_set_wake = xgold_irq_eint_set_wake,
#endif
};

static struct irq_domain_ops xgold_irq_eint_domain_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = xgold_irq_domain_map,
};

/*
 * Entry point for EINT. called from of_irq_init
 */
static int __init xgold_irq_eint_of_init(struct device_node *np,
					struct device_node *parent)
{
	int ret = 0;
	struct xgold_irq_chip_data *data;

	data = kzalloc(sizeof(struct xgold_irq_chip_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->chip = &xgold_irq_eint_chip;
	data->type = XGOLD_IRQ_DOMAIN_N2N;

	/* extract info form dt */
	xgold_irq_of_get(np, data);

	/* add linear domain */
	ret |= xgold_irq_domain_add_linear(np,
			data, &xgold_irq_eint_domain_ops);
	irq_eint_domain = data->domain;

	/* Parse, Map and Cascade */
	if (parent)
		ret |= xgold_irq_parse_map_and_cascade(np, data);

	return ret;
}

IRQCHIP_DECLARE(xgold_eint, "intel,xgold_eint", xgold_irq_eint_of_init);

/* EOF */
