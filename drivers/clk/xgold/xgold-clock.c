/*
* Copyright (C) 2014 Intel Mobile Communications GmbH
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
*/

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/io.h>

#include "xgold-clk.h"

struct xgold_clk_clock_hw {
	unsigned freq_banks;
	unsigned is_divideable;
	unsigned is_gateable;
	const struct clk_div_table *table;
	struct xgold_clk_reg divider_status;
	struct xgold_clk_reg divider_activate;
	struct xgold_clk_reg *divider;
	struct xgold_clk_reg enable_status;
	struct xgold_clk_reg enable;
	struct xgold_clk_reg disable;

};

struct xgold_clk_clock {
	struct clk_hw hw;
	struct xgold_cgu_device *cgu;
	unsigned *def_divider;
	unsigned flags;
	struct xgold_clk_clock_hw *clock_hw;
};

#define to_xgold_clk_clock(_hw) container_of(_hw, struct xgold_clk_clock, hw)

static int xgold_clk_clock_enable(struct clk_hw *hw)
{
	struct xgold_clk_clock *clock = to_xgold_clk_clock(hw);
	struct xgold_clk_clock_hw *clock_hw = clock->clock_hw;

	XGOLD_CLK_TRACER;
	if (!(clock_hw->is_gateable))
		return 0;

	xgold_clk_reg_write(&clock_hw->enable, 1);

	return 0;
}

static void xgold_clk_clock_disable(struct clk_hw *hw)
{
	struct xgold_clk_clock *clock = to_xgold_clk_clock(hw);
	struct xgold_clk_clock_hw *clock_hw = clock->clock_hw;

	XGOLD_CLK_TRACER;
	if (!(clock_hw->is_gateable))
		return;

	if (clock_hw->disable.reg) {
		xgold_clk_reg_write(&clock_hw->disable, 1);
		return;
	}

	xgold_clk_reg_write(&clock_hw->enable, 0);

}

static int xgold_clk_clock_is_enabled(struct clk_hw *hw)
{
	struct xgold_clk_clock *clock = to_xgold_clk_clock(hw);
	struct xgold_clk_clock_hw *clock_hw = clock->clock_hw;

	XGOLD_CLK_TRACER;
	if (!(clock_hw->is_gateable))
		return 1;

	if (clock_hw->enable_status.reg)
		return xgold_clk_reg_read(&clock_hw->enable_status);

	return xgold_clk_reg_read(&clock_hw->enable);
}

static unsigned int _xgold_get_table_val(const struct clk_div_table *table,
					 unsigned int div)
{
	const struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->div == div)
			return clkt->val;
	return 0;
}

static void _xgold_set_table_val(struct clk_div_table *table,
				 unsigned int val, unsigned int div)
{
	struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->val == val)
			clkt->div = div;
}

#define div_mask(d)	((1 << (d->width)) - 1)
#define is_power_of_two(i)	(ffs(i) == fls(i))
#define is_even(i)	!(i & 1)

static unsigned int _xgold_get_table_maxdiv(const struct clk_div_table *table)
{
	unsigned int maxdiv = 0;
	const struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->div > maxdiv)
			maxdiv = clkt->div;
	return maxdiv;
}

static unsigned _xgold_get_maxdiv(struct xgold_clk_clock *clock)
{
	struct xgold_clk_clock_hw *clock_hw = clock->clock_hw;
	struct xgold_clk_reg *div_reg = &clock_hw->divider[0];

	if (clock->flags & CLK_XGOLD_DIVIDER_ONE_BASED)
		return xgold_reg_mask(div_reg);

	if (clock->flags &
	    (CLK_XGOLD_DIVIDER_POWER_OF_TWO | CLK_XGOLD_DIVIDER_MULT_OF_TWO)) {
		unsigned value = xgold_reg_mask(div_reg);

		if (clock->flags & CLK_XGOLD_DIVIDER_ADD_ONE)
			value += 1;

		if (clock->flags & CLK_XGOLD_DIVIDER_POWER_OF_TWO)
			return 1 << value;

		if (clock->flags & CLK_XGOLD_DIVIDER_MULT_OF_TWO)
			return value << 1;

	}

	if (clock_hw->table)
		return _xgold_get_table_maxdiv(clock_hw->table);

	return xgold_reg_mask(div_reg) + 1;
}

/* CLK_XGOLD_DIVIDER_ONE_BASED:
 *	the divider is the raw value read from the register,
 *      with the value of zero considered invalid
 * CLK_XGOLD_DIVIDER_POWER_OF_TWO:
 *	clock divisor is 2 raised to the value read from the hardware register
 * CLK_XGOLD_DIVIDER_ONLY_EVEN:
 *	only even divider supported ABB_CGUDIV_CHP
 * CLK_XGOLD_DIVIDER_MULT_OF_TWO:
 *	clock divisor is 2x the value read from the hardware
 * CLK_XGOLD_DIVIDER_ADD_ONE:
 *	clock divisor is 2x or power of two of
 *	the value read from the hardware + 1
 * CLK_XGOLD_DIVIDER_DIV_FIX:
 *	divider is 4(N+1) of parent DIF_FIX raw register
 * DEFAULT :
 *	by default the divisor is the value read from the register plus one.
*/

/* SelPclkdbgEff */
static const struct clk_div_table def_div_fix_table[] = {
	{.div = 1, .val = 7},
	{.div = 2, .val = 1},
	{.div = 3, .val = 2},
	{.div = 4, .val = 3},
	{.div = 6, .val = 4},
	{.div = 8, .val = 5},
	{.div = 0, .val = 6},
	{.div = 0, .val = 0},
};

/* SelPclkdbgEff */
static const struct clk_div_table div_table2[] = {
	{.div = 1, .val = 7},
	{.div = 2, .val = 1},
	{.div = 3, .val = 2},
	{.div = 4, .val = 3},
	{.div = 6, .val = 4},
	{.div = 8, .val = 5},
	{.div = 16, .val = 6},
	{.div = 0, .val = 0},
};

static unsigned int _xgold_get_table_div(const struct clk_div_table *table,
					 unsigned int val)
{
	const struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->val == val)
			return clkt->div;
	return 0;
}

static int _xgold_get_div(struct xgold_clk_clock *clock, int value)
{
	struct xgold_clk_clock_hw *clock_hw = clock->clock_hw;

	if (clock_hw->table)
		return _xgold_get_table_div(clock_hw->table, value);

	if (clock->flags &
	    (CLK_XGOLD_DIVIDER_POWER_OF_TWO | CLK_XGOLD_DIVIDER_MULT_OF_TWO)) {
		if (clock->flags & CLK_XGOLD_DIVIDER_ADD_ONE)
			value += 1;

		if (clock->flags & CLK_XGOLD_DIVIDER_POWER_OF_TWO)
			return 1 << value;

		if (clock->flags & CLK_XGOLD_DIVIDER_MULT_OF_TWO)
			return value << 1;
	}

	if (clock->flags & CLK_XGOLD_DIVIDER_ONE_BASED)
		return value;

	return value + 1;
}

static unsigned long xgold_clk_clock_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct xgold_clk_clock *clock = to_xgold_clk_clock(hw);
	struct xgold_clk_clock_hw *clock_hw = clock->clock_hw;
	unsigned long new_rate;
	int bank = 0, div, val;

	XGOLD_CLK_TRACER;

	if (clock_hw->freq_banks > 1)
		bank = XGOLD_CLK_GET_NEW_BANK(clock);

	val = xgold_clk_reg_read(&clock_hw->divider[bank]);
	div = _xgold_get_div(clock, val);
	if (!div) {
		WARN(1, "%s: Invalid divisor for clock %s\n", __func__,
		     __clk_get_name(hw->clk));
		return parent_rate;
	}

	new_rate = parent_rate / div;

	pr_debug("%s: new rate %lu prate %lu\n", __clk_get_name(hw->clk),
		 new_rate, parent_rate);
	return new_rate;
}

static bool _xgold_is_valid_table_div(const struct clk_div_table *table,
				      unsigned int div)
{
	const struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->div == div)
			return true;
	return false;
}

static bool _xgold_is_valid_div(struct xgold_clk_clock *clock, unsigned int div)
{

	struct xgold_clk_clock_hw *clock_hw = clock->clock_hw;

	if ((clock->flags & CLK_XGOLD_DIVIDER_ADD_ONE) && (div < 2))
		return false;

	if (clock->flags & CLK_XGOLD_DIVIDER_POWER_OF_TWO)
		return is_power_of_two(div);

	if (clock->flags & (CLK_XGOLD_DIVIDER_ONLY_EVEN
			    | CLK_XGOLD_DIVIDER_MULT_OF_TWO))
		return is_even(div);

	if (clock_hw->table)
		return _xgold_is_valid_table_div(clock_hw->table, div);

	return true;
}

/*
	from drivers/clk/clk-divider.c
*/

#define MULT_ROUND_UP(r, m) ((r) * (m) + (m) - 1)
static int _xgold_clk_divider_bestdiv(struct clk_hw *hw, unsigned long rate,
				      unsigned long *best_parent_rate)
{
	struct xgold_clk_clock *clock = to_xgold_clk_clock(hw);
	int i, bestdiv = 0;
	unsigned long parent_rate, best = 0, now, maxdiv;

	if (!rate)
		rate = 1;

	maxdiv = _xgold_get_maxdiv(clock);

#if 0
	if (!(__clk_get_flags(hw->clk) & CLK_SET_RATE_PARENT)) {
		parent_rate = *best_parent_rate;
		bestdiv = DIV_ROUND_UP(parent_rate, rate);
		bestdiv = bestdiv == 0 ? 1 : bestdiv;
		bestdiv = bestdiv > maxdiv ? maxdiv : bestdiv;
		return bestdiv;
	}
#endif
	/*
	 * The maximum divider we can use without overflowing
	 * unsigned long in rate * i below
	 */
	maxdiv = min(ULONG_MAX / rate, maxdiv);

	for (i = 1; i <= maxdiv; i++) {
		if (!_xgold_is_valid_div(clock, i))
			continue;
		parent_rate = __clk_round_rate(__clk_get_parent(hw->clk),
					       MULT_ROUND_UP(rate, i));
		now = parent_rate / i;
		if (now <= rate && now > best) {
			bestdiv = i;
			best = now;
			*best_parent_rate = parent_rate;
		}
	}

	if (!bestdiv) {
		bestdiv = _xgold_get_maxdiv(clock);
		*best_parent_rate =
		    __clk_round_rate(__clk_get_parent(hw->clk), 1);
	}

	return bestdiv;
}

static long xgold_clk_clock_round_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long *prate)
{
	int div;
	long round_rate;

	XGOLD_CLK_TRACER;
	div = _xgold_clk_divider_bestdiv(hw, rate, prate);

	round_rate = (*prate / div);
	pr_debug("%s:rate %lu,round rate %lu, prate %lu\n",
		 __clk_get_name(hw->clk), rate, round_rate, *prate);
	return round_rate;
}

static int _xgold_get_val(struct xgold_clk_clock *clock, int div)
{
	struct xgold_clk_clock_hw *clock_hw = clock->clock_hw;

	if (clock_hw->table)
		return _xgold_get_table_val(clock_hw->table, div);

	if (clock->flags &
	    (CLK_XGOLD_DIVIDER_POWER_OF_TWO | CLK_XGOLD_DIVIDER_MULT_OF_TWO)) {
		int value = 0;
		if (clock->flags & CLK_XGOLD_DIVIDER_POWER_OF_TWO)
			value = __ffs(div);

		if (clock->flags & CLK_XGOLD_DIVIDER_MULT_OF_TWO)
			value = div >> 1;

		if ((clock->flags & CLK_XGOLD_DIVIDER_ADD_ONE) && (value > 0))
			value -= 1;

		return value;
	}

	if (clock->flags & CLK_XGOLD_DIVIDER_ONE_BASED)
		return div;

	return div - 1;
}

static int xgold_clk_clock_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	struct xgold_clk_clock *clock = to_xgold_clk_clock(hw);
	struct xgold_clk_clock_hw *clock_hw = clock->clock_hw;
	unsigned int div, value, i;

	XGOLD_CLK_TRACER;
	if (clock->flags & CLK_XGOLD_FLAGS_VREQ_CPU)
		return 0;

	div = parent_rate / rate;
	value = _xgold_get_val(clock, div);

	for (i = 0; i < clock_hw->freq_banks; i++)
		xgold_clk_reg_write(&clock_hw->divider[i], value);

	if (clock_hw->divider_activate.reg)
		xgold_clk_reg_write(&clock_hw->divider_activate, 1);

	return 0;
}

int _xgold_clk_clock_set_dividers(struct xgold_clk_clock *clock)
{
	struct xgold_clk_clock_hw *clock_hw = clock->clock_hw;
	int i, val;

	if (!(clock_hw->is_divideable))
		return 0;

	if (clock->def_divider == NULL)
		return 0;

	for (i = 0; i < clock_hw->freq_banks; i++) {
		val = _xgold_get_val(clock, clock->def_divider[i]);
		xgold_clk_reg_write(&clock_hw->divider[i], val);
	}

	if (clock_hw->divider_activate.reg)
		xgold_clk_reg_write(&clock_hw->divider_activate, 1);

	return 0;
}

static const struct clk_ops xgold_clk_clock_ops = {
	.enable = xgold_clk_clock_enable,
	.disable = xgold_clk_clock_disable,
	.is_enabled = xgold_clk_clock_is_enabled,
	.recalc_rate = xgold_clk_clock_recalc_rate,
	.round_rate = xgold_clk_clock_round_rate,
	.set_rate = xgold_clk_clock_set_rate,
};

static const struct clk_ops xgold_clk_clock_ops_gateable = {
	.enable = xgold_clk_clock_enable,
	.disable = xgold_clk_clock_disable,
	.is_enabled = xgold_clk_clock_is_enabled,
};

static const struct clk_ops xgold_clk_clock_ops_divideable = {
	.recalc_rate = xgold_clk_clock_recalc_rate,
	.round_rate = xgold_clk_clock_round_rate,
	.set_rate = xgold_clk_clock_set_rate,
};

struct clk *xgold_clk_register_clock(struct device *dev, const char *name,
				     const char *parent_name,
				     unsigned long flags,
				     struct xgold_clk_clock_hw *clock_hw,
				     unsigned *def_div)
{
	struct xgold_clk_clock *clock;
	struct clk *clk;
	struct clk_init_data init;

	/* allocate the clock */
	clock = kzalloc(sizeof(struct xgold_clk_clock), GFP_KERNEL);
	if (!clock) {
		pr_err(ALLOC_FAILED, "xgold clock clk\n", name);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.flags = flags & ~CLK_XGOLD_FLAGS_MASK;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* Initialize xgold_clock struct */
	clock->clock_hw = clock_hw;
	clock->hw.init = &init;
	clock->def_divider = def_div;
	clock->flags = flags;

	if ((!(clock->flags & CLK_XGOLD_NO_PARAMETER_INIT)) &&
	    _xgold_clk_clock_set_dividers(clock)
	    )
		return ERR_PTR(-EINVAL);

	if (clock_hw->is_gateable && clock_hw->is_divideable)
		init.ops = &xgold_clk_clock_ops;
	else if (clock_hw->is_gateable)
		init.ops = &xgold_clk_clock_ops_gateable;
	else if (clock_hw->is_divideable)
		init.ops = &xgold_clk_clock_ops_divideable;
	else
		BUG();

	clock->cgu = xgold_clk_get_cgu(&clock->hw);

	/* Register the clock */
	clk = clk_register(dev, &clock->hw);

	if (IS_ERR(clk))
		kfree(clock);

	clk_register_clkdev(clk, name, NULL);

	return clk;
}

void __init of_xgold_clock_clk_setup(struct device_node *node)
{

	const char *clk_name = node->name;
	struct xgold_clk_clock_hw *clock_hw;
	struct clk *clk, *clkp;
	struct device_node *pnode;
	unsigned *def_div = NULL;
	unsigned long flags = 0;

	clock_hw = kzalloc(sizeof(struct xgold_clk_clock_hw), GFP_KERNEL);
	if (!clock_hw) {
		pr_err(ALLOC_FAILED, "xgold clock hw clk", clk_name);
		return;
	}

	if (of_property_read_bool(node, "intel,frequency-banks"))
		of_property_read_u32(node, "intel,frequency-banks",
				     &clock_hw->freq_banks);
	else
		clock_hw->freq_banks = 1;

	of_property_read_string(node, "clock-output-names", &clk_name);

	clkp = of_clk_get(node, 0);
	pnode = of_parse_phandle(node, "clocks", 0);

	flags |= xgold_clk_get_flags(node);
	flags |= (xgold_clk_get_flags(pnode) & CLK_XGOLD_FLAGS_INHERIT_MASK);

	/* Get the registers informations */
	if (of_property_read_bool(node, "intel,block-enable")) {
		xgold_clk_init_reg(node, &clock_hw->enable_status,
				   "intel,block-status-enable");
		xgold_clk_init_reg(node, &clock_hw->enable,
				   "intel,block-enable");
		xgold_clk_init_reg(node, &clock_hw->disable,
				   "intel,block-disable");

		clock_hw->is_gateable = 1;
	}

	if (of_property_read_bool(node, "intel,block-divider")) {
		clock_hw->divider = kcalloc(clock_hw->freq_banks,
						sizeof(struct xgold_clk_reg),
						GFP_KERNEL);
	       if (!(clock_hw->divider)) {
			pr_err(ALLOC_FAILED, "hw divider registers", clk_name);
			return;
		}
		if (xgold_clk_init_reg_array(node, clock_hw->divider,
					     "intel,block-divider",
					     clock_hw->freq_banks)) {
			pr_err(PROPERTY_MISSING, "intel,block-divider",
								clk_name);
			return;
		}
		xgold_clk_init_reg(node, &clock_hw->divider_activate,
				   "intel,block-divider-activate");
		xgold_clk_init_reg(node, &clock_hw->divider_status,
				   "intel,block-divider-status");

		if (!(flags & CLK_XGOLD_NO_PARAMETER_INIT)) {
			def_div = kcalloc(clock_hw->freq_banks,
						sizeof(unsigned),
						GFP_KERNEL);
			if (!def_div) {
				pr_err(ALLOC_FAILED, "divider table", clk_name);
				return;
			}

			if (of_property_read_u32_array
			    (node, "intel,block-divider-default",
			     (unsigned *)def_div, clock_hw->freq_banks)) {
				pr_err(PROPERTY_MISSING,
						"block-divider-default",
						clk_name);
				return;
			}
		}
		if (flags & CLK_XGOLD_DIVIDER_DIV_FIX) {
			clock_hw->table = def_div_fix_table;
			if (of_property_read_bool
			    (node, "intel,block-divider-default-div-fix")) {
				unsigned divfix = 1;
				struct clk_div_table *div_fix_table;
				of_property_read_u32(node,
						     "intel,block-divider-default-div-fix",
						     &divfix);
				div_fix_table =
				    kzalloc(sizeof(def_div_fix_table),
					    GFP_KERNEL);
				memcpy(div_fix_table, def_div_fix_table,
				       sizeof(def_div_fix_table));
				/*
				 *  6 is the selector value in the divider
				 *  which selects the div fix of the mux
				 */
				_xgold_set_table_val(div_fix_table, 6,
						     (divfix + 1) * 4);
				clock_hw->table = div_fix_table;
			}
		}

		if (of_property_read_bool(node, "intel,flags-divider_table1"))
			clock_hw->table = div_table2;

		clock_hw->is_divideable = 1;
	}

	if ((!clock_hw->is_divideable) && (!clock_hw->is_gateable))
		return;

	/* Finally, register the clock to the common clock framework  */
	clk = xgold_clk_register_clock(NULL, clk_name, __clk_get_name(clkp),
				       flags, clock_hw, def_div);

	if (clk)
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	xgold_create_clk_alias_if_any(node);

}
EXPORT_SYMBOL_GPL(of_xgold_clock_clk_setup);
