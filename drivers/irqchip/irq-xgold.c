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
#include <linux/of_irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip/irq_xgold.h>
#include "irqchip.h"

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>
#endif

/*
 * xgold irq write accessor done by Linux guest
 */
int32_t _xgold_irq_write(void __iomem *base, struct irq_reg *reg,
					uint32_t value, bool wr)
{
	uint32_t tmp = 0;
	if (!reg)
		return -1;

	if (wr == XGOLD_RW) {
		tmp = (ioread32(base + reg->base));
		pr_debug("%s: read %#x @ %p\n", __func__, tmp,
				base + reg->base);
		tmp &= ~(reg->mask);
	}
	tmp |= (value << reg->offset);
	pr_debug("%s: write %#x (%#x) @ %p\n", __func__, tmp,
		value << reg->offset, base + reg->base);
	iowrite32(tmp, base + reg->base);
	return 0;
};
/*
 * xgold irq read accessor done by Linux guest
 */
int32_t _xgold_irq_read(void __iomem *base, struct irq_reg *reg)
{
	uint32_t tmp = 0, tmp2 = 0;
	if (!reg)
		return -1;
	tmp = (ioread32(base + reg->base));
	tmp2 = tmp & (reg->mask);
	pr_debug("%s: read %#x (%#x) @ %p\n", __func__, tmp, tmp2,
				base + reg->base);
	return tmp2;
}
/*
 * xgold irq write accessor done by VMM
 */
int32_t _xgold_irq_write_vmm(uint32_t base, struct irq_reg *reg,
		uint32_t value, bool wr)
{
#ifdef CONFIG_X86_INTEL_SOFIA
	uint32_t tmp = 0;
	if (!reg)
		return -1;
	if (wr == XGOLD_RW) {
		if (mv_svc_reg_read(base + reg->base, &tmp, -1)) {
			pr_err("%s: mv_svc_reg_read_service fails @%#x\n",
					__func__, base + reg->base);
			return -1;
		} else {
			pr_debug("%s: read %#x @ %#x\n", __func__, tmp,
				(uint32_t)(base + reg->base));
			tmp &= ~(reg->mask);
		}
	}
	tmp |= (value << reg->offset);
	pr_debug("%s: write %#x (%#x) @ %#x\n", __func__, tmp,
			value << reg->offset, (uint32_t)(base + reg->base));
	if (mv_svc_reg_write(base + reg->base, tmp, -1)) {
		pr_err("%s: mv_svc_reg_write_service fails @%#x\n",
					__func__, base + reg->base);
		return -1;
	}
#endif
	return 0;
}
/*
 * xgold irq read accessor done by VMM
 */
int32_t _xgold_irq_read_vmm(uint32_t base, struct irq_reg *reg)
{
	uint32_t tmp2 = 0;
#ifdef CONFIG_X86_INTEL_SOFIA
	uint32_t tmp = 0;
	if (!reg)
		return -1;
	if (mv_svc_reg_read(base + reg->base, &tmp, -1)) {
		pr_err("%s: mv_svc_reg_read_service fails @%#x\n",
					__func__, base + reg->base);
		return -1;
	}
	tmp2 = tmp & (reg->mask);
	pr_debug("%s: read %#x (%#x) @ %#x\n", __func__, tmp, tmp2,
				(uint32_t)(base + reg->base));
#endif
	return tmp2;
}
/*
 * xgold irq write accessor
 */
int32_t xgold_irq_write(struct xgold_irq_chip_data *data, struct irq_reg *reg,
					uint32_t value, bool wr)
{
	if (data->io_master == IRQ_IO_ACCESS_BY_VMM)
		return _xgold_irq_write_vmm(data->base_phys, reg, value, wr);
	else if (data->io_master == IRQ_IO_ACCESS_BY_LNX)
		return _xgold_irq_write(data->base, reg, value, wr);
	else
		return -1;
}
/*
 * xgold irq read accessor
 */
int32_t xgold_irq_read(struct xgold_irq_chip_data *data, struct irq_reg *reg)
{
	if (data->io_master == IRQ_IO_ACCESS_BY_VMM) {
#ifndef CONFIG_X86_INTEL_SOFIA
		/* This is killing, no vmm calls available
		 * ensure the intel,io-access-guest property is there */
		pr_err("%s: vmm called there!\n", __func__);
		return -1;
#endif
		return _xgold_irq_read_vmm(data->base_phys, reg);
	} else if (data->io_master == IRQ_IO_ACCESS_BY_LNX)
		return _xgold_irq_read(data->base, reg);
	else
		return -1;
}
/*
 * xgold irq find mapping from a N to N domain
 */
static int xgold_irq_find_mapping_n2n(unsigned int irq)
{
	struct xgold_irq_chip_data *data = irq_get_handler_data(irq);
	auto int i;
	pr_debug("%s(%d)\n", __func__, irq);
	if (!data)
		return -1;
	for (i = 0; i < data->nr_int; i++) {
		if (data->table[i] == irq) {
			pr_debug("%s: mapping found for %d: %d\n",
							__func__, irq, i);
			return i;
		}
	}
	pr_debug("%s: mapping not found for %d\n", __func__, irq);
	return -1;
}
/*
 * xgold irq find mapping from a N to 1 domain
 */
static int xgold_irq_find_mapping_n21(unsigned int irq)
{
	struct xgold_irq_chip_data *data = irq_get_handler_data(irq);
	auto int i;
	uint32_t raised = 0, enabled = 0;
	pr_debug("%s(%d)\n", __func__, irq);
	if (!data)
		return -1;
	for (i = 0; i < data->nr_int; i++) {
		raised = xgold_irq_read(data, data->status[i]);
		enabled = xgold_irq_read(data, data->mask[i]);
		pr_debug("%s: status:%#x - mask:%#x\n", __func__,
				raised, enabled);
		if (raised && enabled) {
			pr_debug("%s: mapping found for %d: %d\n",
							__func__, irq, i);
			return i;
		}
	}
	pr_debug("%s: mapping not found for %d\n", __func__, irq);
	return -1;
}
/*
 * xgold irq find mapping custom
 */
static unsigned int xgold_irq_find_mapping_custom(unsigned int irq)
{
	struct xgold_irq_chip_data *data = irq_get_handler_data(irq);
	if (data && data->find_mapping)
		return data->find_mapping(irq);
	else
		return -2;
}
/*
 * xgold irq find mapping wrapper
 *	will return (-1) if no mapping is found. That's could happen if the
 *	IRQ is disabled while we just starting the ISR
 *	will return (-2) in case of custom handler with NULL callback - killing
 */
static int xgold_irq_find_mapping(unsigned int irq, unsigned int type)
{
	if (XGOLD_IRQ_DOMAIN_N21 == type)
		return xgold_irq_find_mapping_n21(irq);
	else if (XGOLD_IRQ_DOMAIN_N2N == type)
		return xgold_irq_find_mapping_n2n(irq);
	else
		return xgold_irq_find_mapping_custom(irq);
}
/*
 * xgold cascade handler entry
 */
void xgold_irq_handle_cascade_irq(unsigned int irq, struct irq_desc *desc)
{
	struct xgold_irq_chip_data *data = NULL;
	struct irq_domain *domain = NULL;
	struct irq_chip *chip = irq_get_chip(irq);
	uint32_t casc_irq = 0;
	int32_t domain_irq = 0;
	data = irq_get_handler_data(irq);
	if (!data || !chip)
		return;
	domain = data->domain;
	chained_irq_enter(chip, desc);
	domain_irq = xgold_irq_find_mapping(irq, data->type);
	if (domain_irq >= 0)
		casc_irq = irq_find_mapping(data->domain, domain_irq);
	if (data->handle_entry)
		data->handle_entry(data);
	if (casc_irq > 0)
		generic_handle_irq(casc_irq);
	if (data->handle_exit)
		data->handle_exit(data);
	chained_irq_exit(chip, desc);
	return;
}
/*
 * xgold map the irq
 */
int xgold_irq_domain_map(struct irq_domain *d, unsigned int virq,
				    irq_hw_number_t hw)
{
	struct xgold_irq_chip_data *data = d->host_data;
	pr_debug("%s: virq(%d) <=> hw(%d)\n", __func__, virq, (unsigned)hw);
	if (!data->flow_handler)
		data->flow_handler = handle_level_irq;

	irq_set_chip_and_handler(virq, data->chip, data->flow_handler);
	irq_set_chip_data(virq, data);
	/* set_irq_flags(virq, IRQF_VALID | IRQF_PROBE); */
	return 0;
}

/*
 * Mask'n'ack all interrupts before domain creation
 */
void xgold_irq_mask_ack_all(struct xgold_irq_chip_data *data)
{
	unsigned i;
	struct irq_data idata;
	pr_debug("%s: Mask and ack before creating domain %s\n",
			__func__, data->name);
	for (i = 0; i < data->nr_int; i++) {
		idata.hwirq = i;
		idata.chip_data = data;
		if (!data->virq[i] && data->chip)
			if (data->chip->irq_mask_ack)
				data->chip->irq_mask_ack(&idata);
	}
}

/*
 * add xgold irq linear irq domain
 * irq desc will be allocated dynamically when requesting the interrupt
 */
int __init xgold_irq_domain_add_linear(struct device_node *np,
		struct xgold_irq_chip_data *data, struct irq_domain_ops *ops)
{
	xgold_irq_mask_ack_all(data);
	data->domain = irq_domain_add_linear(np, data->nr_int, ops, data);
	if (WARN_ON(!data->domain)) {
		pr_err("%s: irq domain init failed. exit...\n", __func__);
		return -1;
	}
	pr_info("%s: new irq domain: %s - %d irqs\n", XGOLD_IRQ,
			data->name, data->nr_int);
	return 0;
}
/*
 * Cascading
 */
static void __init xgold_irq_cascade_irq(unsigned irq, void *chipdata,
		unsigned index)
{
	struct xgold_irq_chip_data *data =
		(struct xgold_irq_chip_data *)chipdata;
	pr_debug("%s: cascading %d irq\n", __func__, irq);
	if (irq_set_handler_data(irq, data) != 0)
		BUG();
	if (!data->virq || !data->virq[index])
		irq_set_chained_handler(irq, xgold_irq_handle_cascade_irq);
	else
		pr_debug("%s: Don't chain irq%d as it's VLX/VMM interrupt - index:%d\n",
				__func__, irq, index);
}
/*
 * Parse, Map and cascade interrupt from different IRQ domains
 */
int __init xgold_irq_parse_map_and_cascade(struct device_node *np,
					struct xgold_irq_chip_data *data)
{
	unsigned i, irq;

	if (!data || !data->nr_int) {
		pr_err("%s: cannot allocate empty table\n", __func__);
		return -1;
	}
	data->table = kzalloc(data->nr_int * sizeof(int), GFP_KERNEL);
	if (!data->table) {
		pr_err("%s: table allocation failed, exit...\n", __func__);
		return -1;
	}
	for (i = 0; i < data->nr_int; i++) {
		irq = irq_of_parse_and_map(np, i);
		data->table[i] = irq;
		if (!irq) {
			pr_err("%s:parse and map failed. exit...\n", __func__);
			return -1;
		}
		pr_debug("%s: parse&map %d => %d\n", __func__, i, irq);
		xgold_irq_cascade_irq(irq, data, i);
	}
	pr_debug("%s: parse, map and cascade %d irqs\n",
			__func__, data->nr_int);
	return 0;
}

/*
 * EOF
 */
