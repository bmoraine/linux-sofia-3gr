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
#include <linux/irqchip/irq_xgold.h>

#include "irqchip.h"

static DEFINE_SPINLOCK(pmu_lock);

static inline void xgold_irq_pmu_unmask(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	spin_lock(&pmu_lock);
	xgold_irq_write(chipdata, chipdata->mask[irq], 1, XGOLD_RW);
	spin_unlock(&pmu_lock);
}

static inline void xgold_irq_pmu_mask(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	spin_lock(&pmu_lock);
	xgold_irq_write(chipdata, chipdata->mask[irq], 0, XGOLD_RW);
	spin_unlock(&pmu_lock);
}

static void xgold_irq_pmu_ack(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	xgold_irq_write(chipdata, chipdata->ack[irq], 1, XGOLD_WO);
}

static void xgold_irq_pmu_maskack(struct irq_data *data)
{
	xgold_irq_pmu_mask(data);
	xgold_irq_pmu_ack(data);
}

#ifdef CONFIG_PM
static int xgold_irq_pmu_set_wake(struct irq_data *data, unsigned on)
{
	u32 irq = data->hwirq;
	return xgold_irq_set_wake(data, irq, on, WAKE_ID_ABB);
}
#endif

static struct irq_chip xgold_irq_pmu_chip = {
	.name = "PMU",
	.irq_mask = xgold_irq_pmu_mask,
	.irq_disable = xgold_irq_pmu_mask,
	.irq_unmask = xgold_irq_pmu_unmask,
	.irq_mask_ack = xgold_irq_pmu_maskack,
	.irq_ack = xgold_irq_pmu_ack,
#ifdef CONFIG_PM
	.irq_set_wake = xgold_irq_pmu_set_wake,
#endif
};

static struct irq_domain_ops xgold_irq_pmu_domain_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = xgold_irq_domain_map,
};

static void xgold_handle_entry_pmu_irq(struct xgold_irq_chip_data *chipdata)
{
	pr_debug("%s\n", __func__);
	/* Mask main PMU Interrupt */
	xgold_irq_write(chipdata, chipdata->globalmask[0], 0, XGOLD_RW);
}

static void xgold_handle_exit_pmu_irq(struct xgold_irq_chip_data *chipdata)
{
	pr_debug("%s\n", __func__);
	/* unmask main PMU Interrupt */
	xgold_irq_write(chipdata, chipdata->globalmask[0], 1, XGOLD_RW);
}

/*
 * Entry point for PMU IRQ. called from of_irq_init
 */
static int __init xgold_irq_pmu_of_init(struct device_node *np,
					struct device_node *parent)
{
	int ret = 0;
	struct xgold_irq_chip_data *data;

	data = kzalloc(sizeof(struct xgold_irq_chip_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->chip = &xgold_irq_pmu_chip;
	data->type = XGOLD_IRQ_DOMAIN_N21;
	data->handle_entry = xgold_handle_entry_pmu_irq;
	data->handle_exit = xgold_handle_exit_pmu_irq;

	/* extract info form dt */
	xgold_irq_of_get(np, data);

	/* add linear domain */
	ret |= xgold_irq_domain_add_linear(np,
			data, &xgold_irq_pmu_domain_ops);

	/* Parse, Map and Cascade */
	if (parent)
		ret |= xgold_irq_parse_map_and_cascade(np, data);

	return ret;
}

IRQCHIP_DECLARE(xgold_pmu, "intel,xgold_pmu", xgold_irq_pmu_of_init);
/*
 * EOF
 */
