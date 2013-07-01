/*
 * Copyright (c) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/io.h>

#include "xgold-clk.h"

enum mux_type {
	natural = 0,
	power = 1,
	custom = 2,
};

struct xgold_clk_mux_hw {
	unsigned freq_banks;
	enum mux_type type;
	unsigned *mux_sels;
	struct xgold_clk_reg update;
	struct xgold_clk_reg status;
	struct xgold_clk_reg *select;
	struct xgold_clk_reg divfix;
	struct xgold_clk_reg xreq;
	struct xgold_clk_reg sleep;
};

struct xgold_clk_mux {
	struct clk_hw hw;
	struct xgold_cgu_device *cgu;
	unsigned *def_sels;
	unsigned flags;
	struct xgold_clk_mux_hw *mux_hw;
};

#define to_xgold_clk_mux(_hw) container_of(_hw, struct xgold_clk_mux, hw)

static u8 _xgold_clk_mux_get_index_from_selector(struct clk_hw *hw,
						 unsigned val)
{
	struct xgold_clk_mux *mux = to_xgold_clk_mux(hw);
	struct xgold_clk_mux_hw *mux_hw = mux->mux_hw;

	if (mux_hw->type == natural)
		return (u8) val;

	if (mux_hw->type == power)
		return ffs(val);

	if (mux_hw->type == custom) {
		int i;
		for (i = 0; i < __clk_get_num_parents(hw->clk); i++)
			if (mux_hw->mux_sels[i] == val)
				return (u8) i;
	}

	return -EINVAL;
}

static u8 _xgold_clk_mux_get_selector_from_index(struct clk_hw *hw, u8 index)
{
	struct xgold_clk_mux *mux = to_xgold_clk_mux(hw);
	struct xgold_clk_mux_hw *mux_hw = mux->mux_hw;

	if (mux_hw->type == natural)
		return index;

	if (mux_hw->type == power)
		return index ? (1 << (index - 1)) : 0;

	if (mux_hw->type == custom)
		return mux_hw->mux_sels[index];

	return 0;
}

static void _xgold_clk_mux_set_selector(struct xgold_clk_mux_hw *mux_hw,
					u8 index)
{
	int i;
	for (i = 0; i < mux_hw->freq_banks; i++)
		xgold_clk_reg_write(&mux_hw->select[i], index);

	if (mux_hw->update.reg)
		xgold_clk_reg_write(&mux_hw->update, 1);
}

static void _xgold_clk_mux_set_default(struct xgold_clk_mux *mux)
{
	int i;
	struct xgold_clk_mux_hw *mux_hw = mux->mux_hw;

	for (i = 0; i < mux_hw->freq_banks; i++)
		xgold_clk_reg_write(&mux_hw->select[i], mux->def_sels[i]);

	if (mux_hw->xreq.reg)
		xgold_clk_reg_write(&mux_hw->xreq, 1);

}

static u8 xgold_clk_mux_get_parent(struct clk_hw *hw)
{
	struct xgold_clk_mux *mux = to_xgold_clk_mux(hw);
	struct xgold_clk_mux_hw *mux_hw = mux->mux_hw;
	unsigned val, bank = 0;
	XGOLD_CLK_TRACER;

	if (mux_hw->freq_banks > 1)
		bank = XGOLD_CLK_GET_BANK(mux);

	if (mux_hw->status.reg)
		return xgold_clk_reg_read(&mux_hw->status);
	else
		val = xgold_clk_reg_read(&mux_hw->select[bank]);

	return _xgold_clk_mux_get_index_from_selector(hw, val);
}

static int xgold_clk_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct xgold_clk_mux *mux = to_xgold_clk_mux(hw);
	struct xgold_clk_mux_hw *mux_hw = mux->mux_hw;
	unsigned char mux_index;
	XGOLD_CLK_TRACER;

	mux_index = _xgold_clk_mux_get_selector_from_index(hw, index);
	_xgold_clk_mux_set_selector(mux_hw, mux_index);

	return 0;
}

const struct clk_ops xgold_clk_mux_ops = {
	.get_parent = xgold_clk_mux_get_parent,
	.set_parent = xgold_clk_mux_set_parent,
};

struct clk *xgold_clk_register_mux(struct device *dev, const char *name,
				   const char **parent_names, u8 num_parents,
				   unsigned long flags,
				   struct xgold_clk_mux_hw *mux_hw,
				   unsigned *def_sels, unsigned def_divfix)
{
	struct clk *clk;
	struct xgold_clk_mux *mux;
	struct clk_init_data init;

	/* allocate the mux */
	mux = kzalloc(sizeof(struct xgold_clk_mux), GFP_KERNEL);
	if (!mux) {
		pr_err(ALLOC_FAILED, "mux clk", name);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &xgold_clk_mux_ops;
	init.flags = (flags & ~CLK_XGOLD_FLAGS_MASK) | CLK_IS_BASIC;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	/* struct clk_mux assignments */
	mux->mux_hw = mux_hw;
	mux->hw.init = &init;
	mux->flags = flags;
	mux->def_sels = def_sels;

	if (!(mux->flags & CLK_XGOLD_NO_PARAMETER_INIT)) {
		_xgold_clk_mux_set_default(mux);
		if (mux_hw->divfix.reg)
			xgold_clk_reg_write(&mux_hw->divfix, def_divfix);
		if (mux_hw->update.reg)
			xgold_clk_reg_write(&mux_hw->update, 1);
	}

	mux->cgu = xgold_clk_get_cgu(&mux->hw);

	clk = clk_register(dev, &mux->hw);

	if (IS_ERR(clk))
		kfree(mux);

	clk_register_clkdev(clk, name, NULL);

	return clk;
}

void __init of_xgold_mux_clk_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct xgold_clk_mux_hw *mux_hw;
	struct clk *clk;
	unsigned *def_sels = NULL;
	unsigned long flags = 0;
	unsigned int num_parents = 0;
	unsigned divfix;
	int i;
	const char *mux_type;
	const char **parent_names;
	struct device_node *consumer_node = NULL;

	mux_hw = kzalloc(sizeof(struct xgold_clk_mux_hw), GFP_KERNEL);
	if (!mux_hw) {
		pr_err(ALLOC_FAILED, "mux hw clk", clk_name);
		return;
	}

	if (of_property_read_bool(node, "intel,frequency-banks"))
		of_property_read_u32(node, "intel,frequency-banks",
				     &mux_hw->freq_banks);
	else
		mux_hw->freq_banks = 1;

	of_property_read_string(node, "clock-output-names", &clk_name);

	/* Get the mux type */
	of_property_read_string(node, "intel,mux-type", &mux_type);

	mux_hw->type = natural;
	if (!strcmp(mux_type, "natural"))
		mux_hw->type = natural;
	else if (!strcmp(mux_type, "power"))
		mux_hw->type = power;
	else if (!strcmp(mux_type, "custom"))
		mux_hw->type = custom;

	for (num_parents = 0;
	     (consumer_node =
	      of_parse_phandle(node, "clocks", num_parents)) != NULL;
	     num_parents++) {
		of_node_put(consumer_node);
	}

	parent_names = kcalloc(num_parents, sizeof(char *), GFP_KERNEL);
	if (!parent_names)
		pr_err(ALLOC_FAILED, "parent names table", clk_name);

	for (i = 0; i < num_parents; i++) {
		struct clk *clkp = of_clk_get(node, i);
		if (IS_ERR(clkp)) {
			pr_err("Could not find %i th clock of mux %s inputs\n",
			       i, clk_name);
			parent_names[i] = "OFF";
		} else {
			parent_names[i] = __clk_get_name(clkp);
		}
	}

	flags |= xgold_clk_get_flags(node);
	mux_hw->select = kcalloc(mux_hw->freq_banks,
				sizeof(struct xgold_clk_reg), GFP_KERNEL);
	if (!mux_hw->select) {
		pr_err(ALLOC_FAILED, "mux hw select registers", clk_name);
		return;
	}

	if (!(flags & CLK_XGOLD_NO_PARAMETER_INIT)) {
		def_sels = kcalloc(mux_hw->freq_banks,
				sizeof(unsigned), GFP_KERNEL);
		if (!def_sels) {
			pr_err(ALLOC_FAILED, "default selection table",
								clk_name);
			return;
		}

		if (of_property_read_u32_array(node, "intel,mux-default",
					       (unsigned *)def_sels,
					       mux_hw->freq_banks)) {
			pr_err(PROPERTY_MISSING, "intel,mux-default", clk_name);
			return;
		}

		if (of_property_read_u32
		    (node, "intel,mux-default-div-fix", &divfix)) {
			divfix = 1;
		}
	}

	if (mux_hw->type == custom) {
		if (of_property_read_u32_array(node, "intel,mux-selectors",
					       mux_hw->mux_sels, num_parents)) {
			pr_err(PROPERTY_MISSING, "intel,mux-selectors",
								clk_name);
			return;
		}
	}

	/* Get hardware register informations */
	if (xgold_clk_init_reg_array(node, mux_hw->select,
				     "intel,mux-select", mux_hw->freq_banks)) {
		pr_err(PROPERTY_MISSING, "intel,mux-select", clk_name);
		return;
	}

	/* Optional register informations */
	xgold_clk_init_reg(node, &mux_hw->update, "intel,mux-update");
	xgold_clk_init_reg(node, &mux_hw->status, "intel,mux-status-select");
	xgold_clk_init_reg(node, &mux_hw->divfix, "intel,mux-divfix");
	xgold_clk_init_reg(node, &mux_hw->xreq, "intel,mux-xreq");
	xgold_clk_init_reg(node, &mux_hw->sleep, "intel,mux-sleep");

	clk = xgold_clk_register_mux(NULL, clk_name, parent_names,
				     num_parents, flags, mux_hw, def_sels,
				     divfix);
	if (clk)
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	xgold_create_clk_alias_if_any(node);
}
EXPORT_SYMBOL_GPL(of_xgold_mux_clk_setup);
