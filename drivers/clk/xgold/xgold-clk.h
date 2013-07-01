/*
* Copyright (C) 2014 Intel Mobile Communications GmbH
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
*/

#include <linux/delay.h>

#define CLK_XGOLD_NO_PARAMETER_INIT		BIT(31)
#define CLK_XGOLD_DIVIDER_ONE_BASED		BIT(30)
#define CLK_XGOLD_DIVIDER_POWER_OF_TWO		BIT(29)
#define CLK_XGOLD_DIVIDER_DIV_FIX		BIT(28)
#define CLK_XGOLD_DIVIDER_MULT_OF_TWO		BIT(27)
#define CLK_XGOLD_DIVIDER_ONLY_EVEN		BIT(26)
#define CLK_XGOLD_DIVIDER_ADD_ONE		BIT(25)
#define CLK_XGOLD_REQ_FORCE_RCOUT		BIT(24)

#define CLK_XGOLD_FLAGS_VREQ_MASK		0x000F0000
#define CLK_XGOLD_FLAGS_VREQ_CPU		BIT(16)

#define CLK_XGOLD_FLAGS_MASK			0xFFFFFF00
#define CLK_XGOLD_FLAGS_INHERIT_MASK		CLK_XGOLD_FLAGS_VREQ_CPU

#define XGOLD_CLK_TRACER	pr_debug(" %s: clk %s\n", __func__,\
						__clk_get_name(hw->clk))
#define xgold_reg_mask(d)	((1 << (d->width)) - 1)

#define ALLOC_FAILED "Allocation of xgold pll %s for clock %s failed!\n"
#define PROPERTY_MISSING "Property %s of clock %s missing!\n"

struct xgold_clk_reg {
	const char *name;
	void __iomem *reg;
	unsigned char shift;
	unsigned char width;
	spinlock_t *lock;
};

#define CGU_MAX_BANK 4
struct xgold_cgu_device {
	const char *name;
	void __iomem *hw_base;
	spinlock_t *reg_locks;
	void __iomem *phys_hw_base;
	unsigned hw_length;
	unsigned vreq_mapping[CGU_MAX_BANK];
	struct xgold_clk_reg *global_vreq;
	struct xgold_clk_reg *cpu_vreq;
	unsigned new_cpu_bank;
	unsigned new_global_bank;
	int (*get_bank)(struct xgold_cgu_device *cgu, struct clk_hw *hw);
};

extern void __iomem *cgu_hw_base;
extern int xgold_create_clk_alias_if_any(struct device_node *np);
extern void xgold_clk_reg_write(struct xgold_clk_reg *, unsigned int);
extern unsigned int xgold_clk_reg_read(struct xgold_clk_reg *);
extern int xgold_clk_init_reg(struct device_node *,
			      struct xgold_clk_reg *, const char *);
extern int xgold_clk_init_reg_array(struct device_node *,
				    struct xgold_clk_reg *,
				    const char *, unsigned);

extern unsigned long xgold_clk_get_flags(struct device_node *);

static inline int xgold_clk_wait_til_timeout(struct xgold_clk_reg *reg,
					     unsigned value, unsigned timeout)
{
	if (reg->reg == NULL)
		return -EINVAL;

	do {
		if (xgold_clk_reg_read(reg) == value)
			return 0;
		udelay(1);
	} while (timeout--);

	pr_err("Timeout while waiting for %s !\n", reg->name);
	return -ETIMEDOUT;
}

extern struct xgold_cgu_device *xgold_clk_get_cgu(struct clk_hw *hw);

extern int xgold_clk_prepare_bank(struct xgold_cgu_device *cgu,
				  unsigned long flags, unsigned bank);
extern int xgold_clk_set_bank(struct xgold_cgu_device *, unsigned long,
			      unsigned);
extern unsigned xgold_clk_get_bank(struct xgold_cgu_device *, unsigned long);
unsigned xgold_clk_get_new_bank(struct xgold_cgu_device *, unsigned long);
#define XGOLD_CLK_GET_NEW_BANK(_clk) \
	xgold_clk_get_new_bank(_clk->cgu,\
		_clk->flags & CLK_XGOLD_FLAGS_VREQ_MASK)
#define XGOLD_CLK_GET_BANK(_clk) \
	xgold_clk_get_bank(_clk->cgu,\
		_clk->flags & CLK_XGOLD_FLAGS_VREQ_MASK)
#define XGOLD_CLK_SET_BANK(_clk, _bank)\
	xgold_clk_set_bank(_clk->cgu,\
		_clk->flags & CLK_XGOLD_FLAGS_VREQ_MASK, _bank)
#define XGOLD_CLK_PREPARE_BANK(_clk, _bank) \
	xgold_clk_prepare_bank(_clk->cgu,\
			_clk->flags & CLK_XGOLD_FLAGS_VREQ_MASK, _bank)

/* xgold clock setup functions */
extern void of_xgold_clk_setup(struct device_node *);
extern void of_fixed_clk_setup(struct device_node *);
extern void of_xgold_pll_clk_setup(struct device_node *);
extern void of_xgold_phs_clk_setup(struct device_node *);
extern void of_xgold_fixed_divider_clk_setup(struct device_node *);
extern void of_xgold_divider_clk_setup(struct device_node *);
extern void of_xgold_mux_clk_setup(struct device_node *);
extern void of_xgold_clock_clk_setup(struct device_node *);
extern void of_xgold_req_clk_setup(struct device_node *);

