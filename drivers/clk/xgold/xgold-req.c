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

struct xgold_clk_req_hw {
	struct xgold_clk_reg *en;
	unsigned en_count;
	struct xgold_clk_reg *inv;
	unsigned inv_count;
	struct xgold_clk_reg *rcout_en;
	struct xgold_clk_reg *rcout_set;
};

struct xgold_clk_req {
	struct clk_hw hw;
	struct xgold_cgu_device *cgu;
	unsigned *def_inv;
	unsigned flags;
	struct xgold_clk_req_hw *req_hw;
};

#define to_xgold_clk_req(_hw) container_of(_hw, struct xgold_clk_req, hw)

static void _xgold_clk_req_set_default(struct xgold_clk_req *req)
{
	int i;
	struct xgold_clk_req_hw *req_hw = req->req_hw;

	for (i = 0; i < req_hw->inv_count; i++)
		xgold_clk_reg_write(&req_hw->inv[i], req->def_inv[i]);
}

int xgold_clk_req_init_reg_array(struct xgold_clk_reg *clk_reg, unsigned shift,
				 struct xgold_clk_reg *rrin_en,
				 const char *propname, unsigned nr)
{
	int i;

	for (i = 0; i < nr; i++) {
		/* Reuse same register address as RRINi_EN bit fields */
		clk_reg[i].reg = rrin_en[i].reg;
		clk_reg[i].shift = shift;
		clk_reg[i].width = 1;
		clk_reg[i].lock = rrin_en[i].lock;
	}

	return 0;

}

static void _xgold_clk_req_endisable(struct clk_hw *hw, int enable)
{
	struct xgold_clk_req *req = to_xgold_clk_req(hw);
	struct xgold_clk_req_hw *req_hw = req->req_hw;
	int i;

	for (i = 0; i < req_hw->en_count; i++) {
		xgold_clk_reg_write(&req_hw->en[i], enable);

		/*FIXME */
		/* Force RCOUT_EN bit field to 1 */
		xgold_clk_reg_write(&req_hw->rcout_en[i], 1);
		/* Force RCOUT_SET bit field */
		if (req->flags & CLK_XGOLD_REQ_FORCE_RCOUT)
			xgold_clk_reg_write(&req_hw->rcout_set[i], 1);
		else
			xgold_clk_reg_write(&req_hw->rcout_set[i], 0);
	}
}

static int xgold_clk_req_enable(struct clk_hw *hw)
{
	_xgold_clk_req_endisable(hw, 1);

	return 0;
}

static void xgold_clk_req_disable(struct clk_hw *hw)
{
	_xgold_clk_req_endisable(hw, 0);
}

static int xgold_clk_req_is_enabled(struct clk_hw *hw)
{
	struct xgold_clk_req *req = to_xgold_clk_req(hw);
	struct xgold_clk_req_hw *req_hw = req->req_hw;
	int en = 1, i;

	/* if at least, 1 EN bit field is 0, return clock disable */
	for (i = 0; i < req_hw->en_count; i++)
		en &= (xgold_clk_reg_read(&req_hw->en[i]) != 0);

	return en ? 1 : 0;
}

const struct clk_ops xgold_clk_req_ops = {
	.enable = xgold_clk_req_enable,
	.disable = xgold_clk_req_disable,
	.is_enabled = xgold_clk_req_is_enabled,
};

struct clk *xgold_clk_register_req(struct device *dev, const char *name,
				   const char **parent_names, u8 num_parents,
				   unsigned long flags,
				   struct xgold_clk_req_hw *req_hw,
				   unsigned *def_inv)
{
	struct clk *clk;
	struct xgold_clk_req *req;
	struct clk_init_data init;

	/* allocate the req */
	req = kzalloc(sizeof(struct xgold_clk_req), GFP_KERNEL);
	if (!req) {
		pr_err(ALLOC_FAILED, "xgold req clk", name);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &xgold_clk_req_ops;
	init.flags = (flags & ~CLK_XGOLD_FLAGS_MASK) | CLK_IS_BASIC;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	/* struct clk_req assignments */
	req->req_hw = req_hw;
	req->hw.init = &init;
	req->flags = flags;
	req->def_inv = def_inv;

	if (!(req->flags & CLK_XGOLD_NO_PARAMETER_INIT))
		_xgold_clk_req_set_default(req);

	req->cgu = xgold_clk_get_cgu(&req->hw);

	clk = clk_register(dev, &req->hw);

	if (IS_ERR(clk))
		kfree(req);

	clk_register_clkdev(clk, name, NULL);

	return clk;
}

void __init of_xgold_req_clk_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct xgold_clk_req_hw *req_hw;
	struct clk *clk;
	unsigned *def_inv = NULL;
	unsigned long flags = 0;
	const char **parent_names;
	struct device_node *consumer_node = NULL;
	struct clk *clkp;

	req_hw = kzalloc(sizeof(struct xgold_clk_req_hw), GFP_KERNEL);
	if (!req_hw) {
		pr_err(ALLOC_FAILED, "req hw clk", clk_name);
		return;
	}

	of_property_read_string(node, "clock-output-names", &clk_name);

	consumer_node = of_parse_phandle(node, "clocks", 0);
	of_node_put(consumer_node);

	parent_names = kzalloc(sizeof(char *), GFP_KERNEL);
	if (!parent_names)
		pr_err(ALLOC_FAILED, "parent names table", clk_name);

	clkp = of_clk_get(node, 0);
	if (IS_ERR(clkp)) {
		pr_err("Could not find parent clock of req %s inputs\n",
		       clk_name);
		parent_names[0] = "OFF";
	} else {
		parent_names[0] = __clk_get_name(clkp);
	}

	flags |= xgold_clk_get_flags(node);

	/* req hw EN registers information */
	of_find_property(node, "intel,req,en", &req_hw->en_count);
	/* Convert from u32, 3 param per bit field */
	req_hw->en_count /= 4 * 3;
	req_hw->en = kcalloc(req_hw->en_count,
				sizeof(struct xgold_clk_reg), GFP_KERNEL);
	if (!req_hw->en) {
		pr_err(ALLOC_FAILED, "req hw enable registers", clk_name);
		return;
	}

	if (xgold_clk_init_reg_array(node, req_hw->en,
				     "intel,req,en", req_hw->en_count)) {
		pr_err(PROPERTY_MISSING, "intel,req,en", clk_name);
		return;
	}
	/* Prepare RCOUT_EN bit fields */
	req_hw->rcout_en = kcalloc(req_hw->en_count,
				sizeof(struct xgold_clk_reg), GFP_KERNEL);
	if (!req_hw->rcout_en) {
		pr_err(ALLOC_FAILED, "req hw rcout_en registers", clk_name);
		return;
	}

	if (xgold_clk_req_init_reg_array
	    (req_hw->rcout_en, 1, req_hw->en, "intel,req,rcout_en",
	     req_hw->en_count)) {
		pr_err(PROPERTY_MISSING, "intel,req,rcout_en", clk_name);
		return;
	}
	/* Prepare RCOUT_SET bit fields */
	req_hw->rcout_set = kcalloc(req_hw->en_count,
				sizeof(struct xgold_clk_reg), GFP_KERNEL);
	if (!req_hw->rcout_set) {
		pr_err(ALLOC_FAILED, "req hw rcout_set registers", clk_name);
		return;
	}

	if (xgold_clk_req_init_reg_array
	    (req_hw->rcout_set, 0, req_hw->en, "intel,req,rcout_set",
	     req_hw->en_count)) {
		pr_err(PROPERTY_MISSING, "intel,req,rcout_set", clk_name);
		return;
	}

	/* req hw INV registers information */
	of_find_property(node, "intel,req,inv", &req_hw->inv_count);
	/* Convert from u32, 3 param per bit field */
	req_hw->inv_count /= 4 * 3;

	if (req_hw->inv_count > 0) {
		req_hw->inv = kcalloc(req_hw->inv_count,
				sizeof(struct xgold_clk_reg), GFP_KERNEL);
		if (!req_hw->inv) {
			pr_err(ALLOC_FAILED, "req hw invert registers",
								clk_name);
			return;
		}

		if (xgold_clk_init_reg_array(node, req_hw->inv,
					     "intel,req,inv",
					     req_hw->inv_count)) {
			pr_err(PROPERTY_MISSING, "intel,req,inv", clk_name);
			return;
		}

		def_inv = kcalloc(req_hw->inv_count,
					sizeof(unsigned), GFP_KERNEL);
		if (!def_inv) {
			pr_err(ALLOC_FAILED, "default invert reg table",
								clk_name);
			return;
		}

		if (of_property_read_u32_array(node, "intel,req,inv-def",
					       def_inv, req_hw->inv_count)) {
			pr_err(PROPERTY_MISSING, "intel,req,inv-def", clk_name);
			return;
		}
	}

	clk = xgold_clk_register_req(NULL, clk_name, parent_names,
				     1, flags, req_hw, def_inv);
	if (clk)
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	xgold_create_clk_alias_if_any(node);
}
EXPORT_SYMBOL_GPL(of_xgold_req_clk_setup);
