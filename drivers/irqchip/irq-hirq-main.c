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

#include "irqchip.h"
#include "irq-xgold.h"

static inline void xgold_irq_hirq_main_unmask(struct irq_data *data)
{
	pr_debug("%s(%d): -->\n", __func__, (uint32_t)data->hwirq);
}

static inline void xgold_irq_hirq_main_mask(struct irq_data *data)
{
	pr_debug("%s(%d): -->\n", __func__, (uint32_t)data->hwirq);
}

static void xgold_irq_hirq_main_enable(struct irq_data *data)
{
	pr_debug("%s(%d): -->\n", __func__, (uint32_t)data->hwirq);
}

static void xgold_irq_hirq_main_disable(struct irq_data *data)
{
	pr_debug("%s(%d): -->\n", __func__, (uint32_t)data->hwirq);
}

void xgold_irq_hirq_main_eoi(struct irq_data *data)
{
	pr_debug("%s(%d): -->\n", __func__, (uint32_t)data->hwirq);
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
	.xlate = xgold_irq_domain_xlate,
	.map = xgold_irq_domain_map,
};

/*
 * Entry point for MAIN HIRQ IRQ. called from of_irq_init
 */
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
