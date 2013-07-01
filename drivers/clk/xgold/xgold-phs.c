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

/*
	rate = prate * ( y + x/6)
*/

struct xgold_clk_phs_params {
	unsigned x;
	unsigned y;
};

struct xgold_clk_phs_hw {
	unsigned freq_banks;
	unsigned mux_banks;
	struct xgold_clk_reg *up;
	struct xgold_clk_reg update;
	struct xgold_clk_reg *mux;
	struct xgold_clk_reg status_mux;
	struct xgold_clk_reg status_up;
	struct xgold_clk_reg status_x;
	struct xgold_clk_reg status_y;
	struct xgold_clk_reg *x;
	struct xgold_clk_reg *y;

};

struct xgold_clk_phs {
	struct clk_hw hw;
	struct xgold_cgu_device *cgu;
	unsigned flags;
	struct xgold_clk_phs_hw *phs_hw;
	struct xgold_clk_phs_params *params;
};

#define to_xgold_clk_phs(_hw) container_of(_hw, struct xgold_clk_phs, hw)

static int _xgold_clk_phs_set_parameters(struct xgold_clk_phs *phs)
{
	struct xgold_clk_phs_params *params = phs->params;
	struct xgold_clk_phs_hw *phs_hw = phs->phs_hw;
	int i;

	for (i = 0; i < phs_hw->freq_banks; i++) {
		xgold_clk_reg_write(&phs_hw->x[i], params[i].x);
		xgold_clk_reg_write(&phs_hw->y[i], params[i].y);
	}

	/* Update phs parameters if any */
	if (phs_hw->update.reg)
		xgold_clk_reg_write(&phs_hw->update, 1);

	return 0;
}

static int xgold_clk_phs_prepare(struct clk_hw *hw)
{
	int ret = 0, i;
	struct xgold_clk_phs *phs = to_xgold_clk_phs(hw);
	struct xgold_clk_phs_hw *phs_hw = phs->phs_hw;
	XGOLD_CLK_TRACER;
	for (i = 0; i < phs_hw->mux_banks; i++)
		xgold_clk_reg_write(&phs_hw->mux[i], 0);
	_xgold_clk_phs_set_parameters(phs);
	for (i = 0; i < phs_hw->freq_banks; i++)
		xgold_clk_reg_write(&phs_hw->up[i], 0);

/*FIXME : Shall we wait ?*/
	return ret;
}

static void xgold_clk_phs_unprepare(struct clk_hw *hw)
{
	XGOLD_CLK_TRACER;
}

static int xgold_clk_phs_enable(struct clk_hw *hw)
{
	int ret = 0, i;
	struct xgold_clk_phs *phs = to_xgold_clk_phs(hw);
	struct xgold_clk_phs_hw *phs_hw = phs->phs_hw;

	XGOLD_CLK_TRACER;

	for (i = 0; i < phs_hw->freq_banks; i++)
		xgold_clk_reg_write(&phs_hw->up[i], 1);
	for (i = 0; i < phs_hw->mux_banks; i++)
		xgold_clk_reg_write(&phs_hw->mux[i], 1);

	return ret;
}

static void xgold_clk_phs_disable(struct clk_hw *hw)
{
	struct xgold_clk_phs *phs = to_xgold_clk_phs(hw);
	struct xgold_clk_phs_hw *phs_hw = phs->phs_hw;
	int i;

	XGOLD_CLK_TRACER;
	for (i = 0; i < phs_hw->freq_banks; i++)
		xgold_clk_reg_write(&phs_hw->up[i], 0);
}

static int _xgold_clk_phs_get_mux(struct xgold_clk_phs *phs)
{
	int ret, bank = 0;
	struct xgold_clk_phs_hw *phs_hw = phs->phs_hw;

	if (phs_hw->mux_banks > 1)
		bank = XGOLD_CLK_GET_BANK(phs);

	if (phs_hw->status_mux.reg)
		ret = xgold_clk_reg_read(&phs_hw->status_mux);
	else
		ret = xgold_clk_reg_read(&phs_hw->mux[bank]);

	return ret;
}

static int _xgold_clk_phs_get_up(struct xgold_clk_phs_hw *phs_hw)
{
	int ret, bank = 0;

	if (phs_hw->status_up.reg)
		ret = xgold_clk_reg_read(&phs_hw->status_up);
	else
		ret = xgold_clk_reg_read(&phs_hw->up[bank]);

	return ret;
}
static int xgold_clk_phs_is_enabled(struct clk_hw *hw)
{
	int ret = 0;
	struct xgold_clk_phs *phs = to_xgold_clk_phs(hw);
	struct xgold_clk_phs_hw *phs_hw = phs->phs_hw;

	XGOLD_CLK_TRACER;
	ret = _xgold_clk_phs_get_up(phs_hw);
	if (!ret)
		return 0;

	ret = _xgold_clk_phs_get_mux(phs);
	if (!ret)
		return 0;

	return 1;
}

static struct xgold_clk_phs_params _xgold_clk_phs_get_parameters(struct
								 xgold_clk_phs
								 *phs)
{
	struct xgold_clk_phs_hw *phs_hw = phs->phs_hw;
	struct xgold_clk_phs_params params;
	int bank = 0;

	if (phs_hw->freq_banks > 1)
		bank = XGOLD_CLK_GET_BANK(phs);

	if (phs_hw->status_x.reg)
		params.x = xgold_clk_reg_read(&phs_hw->status_x);

	else
		params.x = xgold_clk_reg_read(&phs_hw->x[bank]);

	if (phs_hw->status_y.reg)
		params.y = xgold_clk_reg_read(&phs_hw->status_y);

	else
		params.y = xgold_clk_reg_read(&phs_hw->y[bank]);

	return params;
}
static unsigned long _xgold_clk_phs_calc_rate(struct xgold_clk_phs_params
					      params, unsigned long prate)
{
	unsigned long rate;
	unsigned int tmp_div;

	rate = (6 * prate);
	tmp_div = (6 * params.y) + params.x;
	do_div(rate, tmp_div);

	return rate;
}

static unsigned long xgold_clk_phs_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct xgold_clk_phs *phs = to_xgold_clk_phs(hw);
	struct xgold_clk_phs_params params;

	XGOLD_CLK_TRACER;
	params = _xgold_clk_phs_get_parameters(phs);

	return _xgold_clk_phs_calc_rate(params, parent_rate);
}

static long xgold_clk_phs_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *prate)
{
	unsigned long y, tmp, tmp2;
	unsigned int x;
	struct xgold_clk_phs_params params;

	XGOLD_CLK_TRACER;
	if (rate > *prate)
		return *prate;

	y = rate;

	x = do_div(y, (unsigned int)*prate);

	tmp = *prate;

	do_div(tmp, 6);
	tmp2 = do_div(x, tmp);

	if (tmp2 > (tmp >> 1))
		x++;

	if (x == 6) {
		y++;
		x = 0;
	}

	params.x = x;
	params.y = (unsigned int)y;

	return _xgold_clk_phs_calc_rate(params, *prate);
}

static int xgold_clk_phs_set_rate(struct clk_hw *hw, unsigned long new_rate,
				  unsigned long prate)
{
	int ret = 0;

	XGOLD_CLK_TRACER;
	return ret;
}

/* FIXME: Not sure we should really declare prepare/unprepare callbacks
	as the phs enabling procedure does not need waiting cycles
*/
const struct clk_ops xgold_clk_phs_ops = {
	.prepare = xgold_clk_phs_prepare,
	.unprepare = xgold_clk_phs_unprepare,
	.enable = xgold_clk_phs_enable,
	.disable = xgold_clk_phs_disable,
	.is_enabled = xgold_clk_phs_is_enabled,
	.recalc_rate = xgold_clk_phs_recalc_rate,
	.round_rate = xgold_clk_phs_round_rate,
	.set_rate = xgold_clk_phs_set_rate,
};

static struct clk *xgold_clk_register_phs(struct device *dev, const char *name,
					  const char *parent_name,
					  unsigned long flags,
					  struct xgold_clk_phs_hw *phs_hw,
					  struct xgold_clk_phs_params *params)
{
	struct xgold_clk_phs *phs;
	struct clk *clk;
	struct clk_init_data init;

	/* allocate the phs clock */
	phs = kzalloc(sizeof(struct xgold_clk_phs), GFP_KERNEL);
	if (!phs) {
		pr_err(ALLOC_FAILED, "xgold phs clk", name);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &xgold_clk_phs_ops;
	init.flags = flags & ~CLK_XGOLD_FLAGS_MASK;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* Initialize xgold_phs struct */
	phs->phs_hw = phs_hw;
	phs->hw.init = &init;
	phs->params = params;
	phs->flags = flags;

	if ((!(phs->flags & CLK_XGOLD_NO_PARAMETER_INIT)) &&
	    _xgold_clk_phs_set_parameters(phs)
	    )
		return ERR_PTR(-EINVAL);

	phs->cgu = xgold_clk_get_cgu(&phs->hw);

	/* Register the clock */
	clk = clk_register(dev, &phs->hw);

	if (IS_ERR(clk))
		kfree(phs);

	clk_register_clkdev(clk, name, NULL);

	return clk;

}

void __init of_xgold_phs_clk_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct xgold_clk_phs_hw *phs_hw;
	struct xgold_clk_phs_params *phs_params;
	struct device_node *pnode;
	struct clk *clk;
	unsigned long flags = 0;

	phs_hw = kzalloc(sizeof(struct xgold_clk_phs_hw), GFP_KERNEL);
	if (!phs_hw) {
		pr_err(ALLOC_FAILED, "xgold phs hw clk", clk_name);
		return;
	}

	/* Check banks parameters is present, otherwise assume
	   the phs does not have bank support */

	if (of_property_read_bool(node, "intel,frequency-banks"))
		of_property_read_u32(node, "intel,frequency-banks",
				     &phs_hw->freq_banks);
	else
		phs_hw->freq_banks = 1;

	if (of_property_read_bool(node, "intel,mux-banks"))
		of_property_read_u32(node, "intel,mux-banks",
				     &phs_hw->mux_banks);
	else
		phs_hw->mux_banks = 1;

	pnode = of_parse_phandle(node, "clocks", 0);
	of_property_read_string(node, "clock-output-names", &clk_name);

	phs_hw->up = kcalloc(phs_hw->freq_banks,
			sizeof(struct xgold_clk_reg), GFP_KERNEL);
	if (!phs_hw->up) {
		pr_err(ALLOC_FAILED, "xgold phs hw up registers", clk_name);
		return;
	}

	phs_hw->mux = kcalloc(phs_hw->mux_banks,
			sizeof(struct xgold_clk_reg), GFP_KERNEL);
	if (!phs_hw->mux) {
		pr_err(ALLOC_FAILED, "xgold phs hw mux registers", clk_name);
		return;
	}

	phs_hw->x = kcalloc(phs_hw->freq_banks,
			sizeof(struct xgold_clk_reg), GFP_KERNEL);
	if (!phs_hw->x) {
		pr_err(ALLOC_FAILED, "xgold phs hw X registers", clk_name);
		return;
	}

	phs_hw->y = kcalloc(phs_hw->freq_banks,
			sizeof(struct xgold_clk_reg), GFP_KERNEL);
	if (!phs_hw->y) {
		pr_err(ALLOC_FAILED, "xgold phs hw Y registers", clk_name);
		return;
	}

	phs_params = kcalloc(phs_hw->freq_banks,
			sizeof(struct xgold_clk_phs_params), GFP_KERNEL);
	if (!phs_params) {
		pr_err(ALLOC_FAILED, "xgold phs clk parameters", clk_name);
		return;
	}

	if (of_property_read_u32_array(node, "intel,phs-parameters",
				       (unsigned *)phs_params,
				       ((sizeof(struct xgold_clk_phs_params) /
					 sizeof(unsigned))
					* phs_hw->freq_banks)
	    )) {
		pr_err(PROPERTY_MISSING, "intel,phs-parameters", clk_name);
		return;
	}

	flags |= xgold_clk_get_flags(node);

	/* Parse the mandatory properties */
	if (xgold_clk_init_reg_array
	    (node, phs_hw->up, "intel,phs-up", phs_hw->freq_banks)) {
		pr_err(PROPERTY_MISSING, "intel,phs-up", clk_name);
		return;
	}

	if (xgold_clk_init_reg_array
	    (node, phs_hw->mux, "intel,phs-mux", phs_hw->mux_banks)) {
		pr_err(PROPERTY_MISSING, "intel,phs-mux", clk_name);
		return;
	}

	if (xgold_clk_init_reg_array
	    (node, phs_hw->x, "intel,phs-X", phs_hw->freq_banks)) {
		pr_err(PROPERTY_MISSING, "intel,phs-X", clk_name);
		return;
	}

	if (xgold_clk_init_reg_array
	    (node, phs_hw->y, "intel,phs-Y", phs_hw->freq_banks)) {
		pr_err(PROPERTY_MISSING, "intel,phs-Y", clk_name);
		return;
	}

	/* Optional register informations */
	xgold_clk_init_reg(node, &phs_hw->update, "intel,phs-update");
	xgold_clk_init_reg(node, &phs_hw->status_mux, "intel,phs-status-mux");
	xgold_clk_init_reg(node, &phs_hw->status_up, "intel,phs-status-up");
	xgold_clk_init_reg(node, &phs_hw->status_x, "intel,phs-status-X");
	xgold_clk_init_reg(node, &phs_hw->status_y, "intel,phs-status-Y");

	clk =
	    xgold_clk_register_phs(NULL, clk_name, pnode->name, flags, phs_hw,
				   phs_params);
	if (clk)
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	xgold_create_clk_alias_if_any(node);
}
EXPORT_SYMBOL_GPL(of_xgold_phs_clk_setup);
