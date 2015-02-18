/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/syscore_ops.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/irqchip/irq_xgold.h>

#include "irqchip.h"

/* Define this as static for syscore resume callback */
static struct xgold_irq_chip_data *data;

static DEFINE_SPINLOCK(abb_lock);

/*
 * unmask all ABB interrupts, masking/unmasking will
 * be handle with slmask registers to avoid registers
 * sharing with the idi controller driver
 * Mask all interrupts in slmask (HW default is all ones)
 */
#ifndef XGOLD_HANDLE_ABB_MASK_IRQ
static void xgold_abb_unmask_all(struct xgold_irq_chip_data *data)
{
	u32 i;
	spin_lock(&abb_lock);
	for (i = 0; i < data->nr_int; i++) {
		xgold_irq_write(data, data->mask[i], 1, XGOLD_RW);
		if (!data->virq[i])
			xgold_irq_write(data, data->slmask[i], 0, XGOLD_RW);
	}
	spin_unlock(&abb_lock);
}
#endif

static inline void xgold_irq_abb_unmask(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	spin_lock(&abb_lock);
#ifdef XGOLD_HANDLE_ABB_MASK_IRQ
	xgold_irq_write(chipdata, chipdata->mask[irq], 1, XGOLD_RW);
#endif
	xgold_irq_write(chipdata, chipdata->slmask[irq], 1, XGOLD_RW);
	spin_unlock(&abb_lock);
}

static inline void xgold_irq_abb_mask(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	spin_lock(&abb_lock);
#ifdef XGOLD_HANDLE_ABB_MASK_IRQ
	xgold_irq_write(chipdata, chipdata->mask[irq], 0, XGOLD_RW);
#endif
	xgold_irq_write(chipdata, chipdata->slmask[irq], 0, XGOLD_RW);
	spin_unlock(&abb_lock);
}

static void xgold_irq_abb_ack(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	xgold_irq_write(chipdata, chipdata->ack[irq], 1, XGOLD_WO);
}

static void xgold_irq_abb_maskack(struct irq_data *data)
{
	xgold_irq_abb_mask(data);
	xgold_irq_abb_ack(data);
}

#ifdef CONFIG_PM
static int xgold_irq_abb_set_wake(struct irq_data *data, unsigned on)
{
	u32 irq = data->hwirq;
	return xgold_irq_set_wake(data, irq, on, WAKE_ID_DBB);
}
#endif

static struct irq_chip xgold_irq_abb_chip = {
	.name = "ABB",
	.irq_mask = xgold_irq_abb_mask,
	.irq_disable = xgold_irq_abb_mask,
	.irq_unmask = xgold_irq_abb_unmask,
	.irq_mask_ack = xgold_irq_abb_maskack,
	.irq_ack = xgold_irq_abb_ack,
#ifdef CONFIG_PM
	.irq_set_wake = xgold_irq_abb_set_wake,
#endif
};

static struct irq_domain_ops xgold_irq_abb_domain_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = xgold_irq_domain_map,
};

#define IMC_IDI_TXSTAT(_base)          ((_base) + 0x104)
#define IMC_IDI_TXSTAT_BSY_LOG      (1 << 1)
#define IMC_IDI_TXSTAT_BSY_LNK      (1 << 2)

#define IMC_IDI_RXCON(_base)           ((_base) + 0x138)
#define IMC_IDI_RXCON_EN_CH             (1 << 1)

static struct idi_standby_reg {
	unsigned offset;
	unsigned value;
} idi_regs[] = {
	{ .offset = 0x84, }, /* IMSC */
	{ .offset = 0xA4, }, /* IMSC1 */
	{ .offset = 0x168, }, /* SL_MASK */
	{ .offset = 0x138, }, /* RXCON */
	{ .offset = 0x108, }, /* TXCON */
	{ .offset = 0x160, }, /* ERR_CON */
	{ .offset = 0x15C, }, /* EXT_CON */
	{ .offset = 0x11C, }, /* TXMASK_CON */
	{ .offset = 0x14C, }, /* RXMASK_CON */
};

static int abb_irq_suspend(void)
{
	void __iomem *hw_base = data->base;
	unsigned int reg, i;

	/* IDI bus should be in idle */
	reg = ioread32(IMC_IDI_TXSTAT(hw_base));
	if (reg & (IMC_IDI_TXSTAT_BSY_LOG | IMC_IDI_TXSTAT_BSY_LNK))
		BUG();

	/* Pause channel management */
	reg = ioread32(IMC_IDI_RXCON(hw_base));
	iowrite32(reg & ~IMC_IDI_RXCON_EN_CH, IMC_IDI_RXCON(hw_base));

	/* Save IDI DBB registers */
	for (i = 0; i < ARRAY_SIZE(idi_regs); i++)
		idi_regs[i].value = ioread32(hw_base + idi_regs[i].offset);

	/* FIXME: Ack WUP_DBB in SCU ? */
	return 0;
}

static void abb_irq_resume(void)
{
	void __iomem *hw_base = data->base;
	unsigned int reg, i;

	/* TODO: Restore channels */

	/* Restore IDI DBB registers */
	for (i = 0; i < ARRAY_SIZE(idi_regs); i++)
		iowrite32(idi_regs[i].value, hw_base + idi_regs[i].offset);
	/* IDI bus should be in idle */
	reg = ioread32(IMC_IDI_TXSTAT(hw_base));
	if (reg & (IMC_IDI_TXSTAT_BSY_LOG | IMC_IDI_TXSTAT_BSY_LNK))
		BUG();

	/* Resume channel management */
	reg = ioread32(IMC_IDI_RXCON(hw_base));
	iowrite32(reg | IMC_IDI_RXCON_EN_CH, IMC_IDI_RXCON(hw_base));
}

static struct syscore_ops abb_irq_syscore_ops = {
	.suspend = abb_irq_suspend,
	.resume	= abb_irq_resume,
};

/*
 * Entry point for ABB IRQ. called from of_irq_init
 */
static int __init xgold_irq_abb_of_init(struct device_node *np,
					struct device_node *parent)
{
	int ret = 0;

	data = kzalloc(sizeof(struct xgold_irq_chip_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->chip = &xgold_irq_abb_chip;
	data->type = XGOLD_IRQ_DOMAIN_N2N;
	data->handle_entry = NULL;
	data->handle_exit = NULL;

	/* extract info form dt */
	xgold_irq_of_get(np, data);

	/* add linear domain */
	ret |= xgold_irq_domain_add_linear(np,
			data, &xgold_irq_abb_domain_ops);

	/* Parse, Map and Cascade */
	if (parent)
		ret |= xgold_irq_parse_map_and_cascade(np, data);

#ifndef XGOLD_HANDLE_ABB_MASK_IRQ
	xgold_abb_unmask_all(data);
#endif
/*
	 * Syscore resume callback is called during early resume of the chip.
	 * Kernel needs to set IDI mask again after a deep sleep in this callack
	 */

	if (of_property_read_bool(np, "intel,do-syscore-ops"))
		register_syscore_ops(&abb_irq_syscore_ops);

	return ret;
}

IRQCHIP_DECLARE(xgold_abb, "intel,xgold_abb", xgold_irq_abb_of_init);
/*
 * EOF
 */
