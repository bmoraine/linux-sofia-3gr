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
#include "irqchip.h"
#include "irq-xgold.h"

/*
 * Get intel,io-access property if any from dts
 */
bool xgold_irq_get_io_master(struct device_node *np)
{
	if (of_find_property(np, "intel,io-access-guest", NULL))
		return IRQ_IO_ACCESS_BY_LNX;
	return IRQ_IO_ACCESS_BY_VMM;
}
/*
 * fill irq reg
 */
int __init xgold_irq_of_parse(struct irq_reg **datareg,
			struct device_node *np, char *comp)
{
	unsigned ret = 0;
	unsigned int len = 0;
	unsigned int idx = 0;
	unsigned int j = 0;
	u32 *array;
	struct irq_reg *reg;

	if (of_find_property(np, comp, &len)) {
		len /= sizeof(u32);
		array = kzalloc(len * sizeof(u32), GFP_KERNEL);
		if (!array) {
			pr_err("array allocation failed\n");
			return -1;
		}
		ret = of_property_read_u32_array(np, comp, array, len);
		if (ret != 0)
			pr_err("read %s property failed: %d\n", comp, ret);

		reg = kzalloc(sizeof(struct irq_reg), GFP_KERNEL);
		if (!reg) {
			pr_err("reg allocation failed\n");
			kfree(array);
			return -1;
		}
		pr_debug("Parsing %s properties:\n", comp);
		for (idx = 0; idx + 2 < len; idx += 3) {
			reg->base = array[idx];
			reg->offset = array[idx+1];
			for (j = 0; j < array[idx+2]; j++)
				reg->mask |= 1 << j;

			reg->mask <<= array[idx+1];
			*datareg = reg;
			pr_debug("%s: %s: base=%#x, offset=%#x, mask=%#x\n",
					__func__, comp, reg->base,
							reg->offset, reg->mask);
		}
		kfree(array);
	}
	return 0;
}

static void xgold_display_irq_reg(struct irq_reg *p,
		uint32_t i, char *name)
{
	if (p)
		pr_debug("[%02d] %s\t %08x %08x %08x\n",
				i, name, p->base, p->offset, p->mask);
}

static void xgold_display_irq_regs(struct xgold_irq_chip_data *data)
{
	uint32_t i;
	pr_debug("%s: %s - %d interrupts\n", __func__,
			data->name, data->nr_int);
	pr_debug("[NR] name\t base     offset   mask\n");
	for (i = 0; i < data->nr_int; i++) {
		xgold_display_irq_reg(data->mask[i], i, "mask");
		xgold_display_irq_reg(data->unmask[i], i, "unmask");
		xgold_display_irq_reg(data->slmask[i], i, "slmask");
		xgold_display_irq_reg(data->ack[i], i, "ack ");
		xgold_display_irq_reg(data->edge[i], i, "edge");
		xgold_display_irq_reg(data->level[i], i, "level");
		xgold_display_irq_reg(data->status[i], i, "status");
	}
	xgold_display_irq_reg(data->globalmask[0], 0, "globalmask");
}

/*
 * parse xgold irq domain description
 */
int xgold_irq_of_get(struct device_node *np, void *chipdata)
{
	struct xgold_irq_chip_data *data =
		(struct xgold_irq_chip_data *)chipdata;
	unsigned i = 0;
	int cplen;
	const char *name = of_get_property(np, "compatible", &cplen);
	char comp[30];
	struct resource regs;
	data->np = np;
	pr_info("%s: Looking for %s in dts\n", XGOLD_IRQ, name);
	/* Get io resources */
	if (of_address_to_resource(np, 0, &regs)) {
		/* it's no killing, we are supported SW IRQ domains */
		pr_debug("%s: no resource found", __func__);
	} else {
		/* Get io access master */
		data->io_master = xgold_irq_get_io_master(np);
		data->base_phys = regs.start;
		data->base = of_iomap(np, 0);
		pr_info("%s: io:%s - v:%p - p:%#x\n", XGOLD_IRQ,
			data->io_master == IRQ_IO_ACCESS_BY_LNX ?
			"linux" : "vmm", data->base, data->base_phys);
	}
	/* Get NR of irqs */
	data->nr_int = of_irq_count(np);
	if (!data->nr_int) {
		/* This is killing */
		pr_err("%s: no interrupt found for %s\n", __func__, name);
		return -1;
	} else {
		pr_debug("%s: %d interrupts found\n",
				__func__, data->nr_int);
		data->mask =
			kzalloc(data->nr_int * sizeof(struct irq_req *),
					GFP_KERNEL);
		if (!data->mask) {
			pr_err("data->mask allocation failed\n");
			goto free_them_all;
		}
		data->unmask =
			kzalloc(data->nr_int * sizeof(struct irq_req *),
					GFP_KERNEL);
		if (!data->unmask) {
			pr_err("data->unmask allocation failed\n");
			goto free_them_all;
		}
		data->slmask =
			kzalloc(data->nr_int * sizeof(struct irq_req *),
					GFP_KERNEL);
		if (!data->slmask) {
			pr_err("data->slmask allocation failed\n");
			goto free_them_all;
		}
		data->ack =
			kzalloc(data->nr_int * sizeof(struct irq_req *),
					GFP_KERNEL);
		if (!data->ack) {
			pr_err("data->ack allocation failed\n");
			goto free_them_all;
		}
		data->edge =
			kzalloc(data->nr_int * sizeof(struct irq_req *),
					GFP_KERNEL);
		if (!data->edge) {
			pr_err("data->edge allocation failed\n");
			goto free_them_all;
		}
		data->level =
			kzalloc(data->nr_int * sizeof(struct irq_req *),
					GFP_KERNEL);
		if (!data->level) {
			pr_err("data->level allocation failed\n");
			goto free_them_all;
		}
		data->status =
			kzalloc(data->nr_int * sizeof(struct irq_req *),
					GFP_KERNEL);
		if (!data->status) {
			pr_err("data->status allocation failed\n");
			goto free_them_all;
		}
		data->globalmask
			= kzalloc(sizeof(struct irq_req *),
					GFP_KERNEL);
		if (!data->globalmask) {
			pr_err("data->globalmask allocation failed\n");
			goto free_them_all;
		}
		data->virq =
			kzalloc(data->nr_int * sizeof(unsigned),
					GFP_KERNEL);
		if (!data->virq) {
			pr_err("data->virq allocation failed\n");
			goto free_them_all;
		}
		data->preack =
			kzalloc(data->nr_int * sizeof(unsigned),
					GFP_KERNEL);
		if (!data->preack) {
			pr_err("data->preack allocation failed\n");
			goto free_them_all;
		}
	}

	for (i = 0; i < data->nr_int; i++) {
		sprintf(comp, "intel,mask,%d", i);
		xgold_irq_of_parse(&data->mask[i], np, comp);
		sprintf(comp, "intel,unmask,%d", i);
		xgold_irq_of_parse(&data->unmask[i], np, comp);
		sprintf(comp, "intel,slmask,%d", i);
		xgold_irq_of_parse(&data->slmask[i], np, comp);
		sprintf(comp, "intel,ack,%d", i);
		xgold_irq_of_parse(&data->ack[i], np, comp);
		sprintf(comp, "intel,edge,%d", i);
		xgold_irq_of_parse(&data->edge[i], np, comp);
		sprintf(comp, "intel,level,%d", i);
		xgold_irq_of_parse(&data->level[i], np, comp);
		sprintf(comp, "intel,status,%d", i);
		xgold_irq_of_parse(&data->status[i], np, comp);
		sprintf(comp, "intel,virq,%d", i);
		of_property_read_u32(np, comp, &data->virq[i]);
		if (data->virq[i])
			pr_debug("%s: virq[%d]=%d detected !!!\n",
					__func__, i, data->virq[i]);
	}
	sprintf(comp, "intel,globalmask");
	xgold_irq_of_parse(&data->globalmask[0], np, comp);
	strncpy(data->name, name, MAX_NAME_LENGTH);
	/* Display captured device tree data */
	xgold_display_irq_regs(data);
	return 0;

free_them_all:
	kfree(data->globalmask);
	kfree(data->mask);
	kfree(data->unmask);
	kfree(data->slmask);
	kfree(data->ack);
	kfree(data->edge);
	kfree(data->level);
	kfree(data->status);
	kfree(data->virq);
	kfree(data->preack);
	return -1;
}
/*
 * EOF
 */
