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

enum pll_type {
	analog = 0,
	digital = 1,
	fixed = 2,
	unknown = 3,
};

struct xgold_clk_pll_params {
	unsigned mult;
	unsigned div;
};

struct xgold_clk_pll_hw {
	unsigned freq_banks;
	unsigned mux_banks;
	unsigned long *freqs;
	struct xgold_clk_reg *up;
	struct xgold_clk_reg *shpu;
	struct xgold_clk_reg update;
	struct xgold_clk_reg *mux;
	struct xgold_clk_reg status_lock;
	struct xgold_clk_reg status_mux;
	struct xgold_clk_reg status_up;
	struct xgold_clk_reg *mult;
	struct xgold_clk_reg *div;
};

struct xgold_clk_pll {
	struct clk_hw hw;
	struct xgold_cgu_device *cgu;
	enum pll_type type;
	unsigned flags;
	unsigned long fixed_rate;
	struct xgold_clk_pll_hw *pll_hw;
	struct xgold_clk_pll_params *params;
	unsigned sd_div;
};

#define to_xgold_clk_pll(_hw) container_of(_hw, struct xgold_clk_pll, hw)

static unsigned long _xgold_clk_pll_get_rate(struct xgold_clk_pll *pll,
					     unsigned long parent_rate,
					     struct xgold_clk_pll_params
					     *params)
{
	unsigned long rate;

	if (pll->type == analog) {
		rate = ((parent_rate * (params->mult + 1)) / (params->div + 1));
	} else {
		u64 myrate;
		myrate = parent_rate;
		myrate *= (u64)pll->sd_div;
		myrate += ((u64) parent_rate) * params->mult;
		do_div(myrate, pll->sd_div);
		rate = (unsigned long)myrate;
	}

	return rate;
}

static void _xgold_clk_pll_update_freqs(struct xgold_clk_pll *pll,
					unsigned long prate)
{
	struct xgold_clk_pll_params *params = pll->params;
	struct xgold_clk_pll_hw *pll_hw = pll->pll_hw;
	int i;

	for (i = 0; i < pll_hw->freq_banks; i++)
		pll_hw->freqs[i] =
		    _xgold_clk_pll_get_rate(pll, prate, &params[i]);

}

static int _xgold_clk_pll_set_parameters(struct xgold_clk_pll *pll)
{
	struct xgold_clk_pll_params *params = pll->params;
	struct xgold_clk_pll_hw *pll_hw = pll->pll_hw;
	int i;

	if (pll->type == fixed)
		return 0;

	/* Update parameters */
	if (pll->type == analog) {
		for (i = 0; i < pll_hw->freq_banks; i++) {
			xgold_clk_reg_write(&pll_hw->mult[i], params[i].mult);
			xgold_clk_reg_write(&pll_hw->div[i], params[i].div);
		}
	} else {
		if (pll->type == digital) {
			for (i = 0; i < pll_hw->freq_banks; i++)
				xgold_clk_reg_write(&pll_hw->mult[i],
						    params[i].mult);
		} else {
			pr_err("Pll type unknown !");
			return -EINVAL;
		}
	}

	/* Update pll parameters if any */
	if (pll_hw->update.reg)
		xgold_clk_reg_write(&pll_hw->update, 1);

	return 0;
}

/* FIXME:
	The pll is enabled for all banks.
	We should consider that some banks disable the Pll
 */
static int xgold_clk_pll_prepare(struct clk_hw *hw)
{
	int ret = 0, i;
	struct xgold_clk_pll *pll = to_xgold_clk_pll(hw);
	struct xgold_clk_pll_hw *pll_hw = pll->pll_hw;

	XGOLD_CLK_TRACER;
	if (pll->type == fixed)
		goto lock;

	/* Set pll Output to clkin */
	for (i = 0; i < pll_hw->mux_banks; i++)
		xgold_clk_reg_write(&pll_hw->mux[i], 0);

	/* Switch off pll locking mechanism */
	if (pll_hw->shpu != NULL) {
		for (i = 0; i < pll_hw->freq_banks; i++)
			xgold_clk_reg_write(&pll_hw->shpu[i], 0);
	}
	for (i = 0; i < pll_hw->freq_banks; i++)
		xgold_clk_reg_write(&pll_hw->up[i], 0);

	ret = xgold_clk_wait_til_timeout(&pll_hw->status_mux, 0, 200);

	if (ret == -ETIMEDOUT)
		return ret;

	/* Update parameters */
	ret = _xgold_clk_pll_set_parameters(pll);
	if (ret)
		return ret;

lock:
	/* Switch on pll locking mechanism */
	for (i = 0; i < pll_hw->freq_banks; i++)
		xgold_clk_reg_write(&pll_hw->up[i], 1);

	if (pll_hw->shpu != NULL) {
		for (i = 0; i < pll_hw->freq_banks; i++)
			xgold_clk_reg_write(&pll_hw->shpu[i], 1);
	}

	/* Wait until pll is locked */
	ret = xgold_clk_wait_til_timeout(&pll_hw->status_lock, 1, 100000);

	return ret;
}

static void xgold_clk_pll_unprepare(struct clk_hw *hw)
{
	struct xgold_clk_pll *pll = to_xgold_clk_pll(hw);
	struct xgold_clk_pll_hw *pll_hw = pll->pll_hw;
	int i;

	XGOLD_CLK_TRACER;
	/* Switch off pll locking mechanism */
	for (i = 0; i < pll_hw->freq_banks; i++)
		xgold_clk_reg_write(&pll_hw->up[i], 0);
}

static int xgold_clk_pll_enable(struct clk_hw *hw)
{
	int i;
	struct xgold_clk_pll *pll = to_xgold_clk_pll(hw);
	struct xgold_clk_pll_hw *pll_hw = pll->pll_hw;

	XGOLD_CLK_TRACER;

	if (pll->type == fixed)
		return 0;

	/* Set pll Output to locked output */
	for (i = 0; i < pll_hw->mux_banks; i++)
		xgold_clk_reg_write(&pll_hw->mux[i], 1);

	return 0;
}

static void xgold_clk_pll_disable(struct clk_hw *hw)
{
	struct xgold_clk_pll *pll = to_xgold_clk_pll(hw);
	struct xgold_clk_pll_hw *pll_hw = pll->pll_hw;
	int i;

	XGOLD_CLK_TRACER;

	if (pll->type == fixed)
		return;

	/* Set pll Output to clkin */
	for (i = 0; i < pll_hw->mux_banks; i++)
		xgold_clk_reg_write(&pll_hw->mux[i], 0);

}

static int _xgold_clk_pll_get_mux(struct xgold_clk_pll *pll)
{
	struct xgold_clk_pll_hw *pll_hw = pll->pll_hw;
	int ret, bank = 0;

	if (pll_hw->mux_banks > 1)
		bank = XGOLD_CLK_GET_BANK(pll);

	ret = xgold_clk_reg_read(&pll_hw->mux[bank]);

	return ret;
}

static int xgold_clk_pll_is_enabled(struct clk_hw *hw)
{
	struct xgold_clk_pll *pll = to_xgold_clk_pll(hw);
	struct xgold_clk_pll_hw *pll_hw = pll->pll_hw;
	int ret = 0;

	XGOLD_CLK_TRACER;

	if (pll->type == fixed)
		goto locked;

	ret = _xgold_clk_pll_get_mux(pll);

	/* pll is disabled if mux does not select pll locked output */
	if (!ret)
		return ret;

locked:
	/* pll is disabled if not locked */
	return xgold_clk_reg_read(&pll_hw->status_lock);

}
static struct xgold_clk_pll_params _xgold_clk_pll_get_params(
						struct xgold_clk_pll *pll)
{
	struct xgold_clk_pll_hw *pll_hw = pll->pll_hw;
	struct xgold_clk_pll_params params;
	int bank = 0;

	if (pll_hw->freq_banks > 1)
		bank = XGOLD_CLK_GET_BANK(pll);

	params.mult = xgold_clk_reg_read(&pll_hw->mult[bank]);

	if (pll->type == analog)
		params.div = xgold_clk_reg_read(&pll_hw->div[bank]);
	else
		params.div = 1;

	return params;
}

static unsigned long xgold_clk_pll_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct xgold_clk_pll *pll = to_xgold_clk_pll(hw);
	struct xgold_clk_pll_params params;

	XGOLD_CLK_TRACER;
	if (pll->type == fixed)
		return pll->fixed_rate;
/*
	FIXME: rate should depends on the mux state
	but then we have to force a progation of the rate
	change after the enable call
*/
/*
	if(!xgold_clk_pll_is_enabled(hw))
		return parent_rate;
*/
	params = _xgold_clk_pll_get_params(pll);
	return _xgold_clk_pll_get_rate(pll, parent_rate, &params);
}

static int _xgold_clk_pll_find_out_bank(struct xgold_clk_pll *pll,
					unsigned long new_rate)
{
	int i, best_bank = 0;
	struct xgold_clk_pll_hw *pll_hw = pll->pll_hw;
	unsigned long rate_err;
	rate_err = new_rate;

	for (i = 0; i < pll_hw->freq_banks; i++) {
		signed long err = pll_hw->freqs[i] - new_rate;
		err = abs(err);
		if (rate_err > err) {
			rate_err = err;
			best_bank = i;
		}
	}
	return best_bank;
}

static long xgold_clk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *prate)
{
	struct xgold_clk_pll *pll = to_xgold_clk_pll(hw);
	int i, j;
	signed long rate_err = rate;
	unsigned long best_rate = 0;

	XGOLD_CLK_TRACER;
	if (pll->type == fixed)
		return pll->fixed_rate;

#define MAX_N 255
#define MAX_M 15

	if (pll->type == analog) {
		for (i = 0; i < MAX_N; i++) {
			for (j = 0; j < MAX_M; j++) {
				unsigned long myrate;
				myrate = (*prate * (i + 1)) / (j + 1);
				if (rate_err > abs(rate - myrate)) {
					rate_err = abs(rate - myrate);
					best_rate = myrate;
				}
			}
		}
	} else {
		u64 myrate = rate;
		unsigned remaining;
		myrate <<= 16;
		remaining = do_div(myrate, (unsigned int)*prate);
		if (remaining > (*prate) / 2)
			myrate++;
		myrate *= (*prate);
		best_rate = (myrate >> 16);
	}

	_xgold_clk_pll_update_freqs(pll, *prate);

	if (pll->flags & CLK_XGOLD_FLAGS_VREQ_CPU) {
		int best_bank;
		best_bank = _xgold_clk_pll_find_out_bank(pll, best_rate);
		XGOLD_CLK_PREPARE_BANK(pll, best_bank);
	}

	return best_rate;
}

static int xgold_clk_pll_set_rate(struct clk_hw *hw, unsigned long new_rate,
				  unsigned long prate)
{
	int ret = 0, best_bank = 0;
	struct xgold_clk_pll *pll = to_xgold_clk_pll(hw);

	XGOLD_CLK_TRACER;
/*TODO: Compute parameters. */

	if (pll->flags & CLK_XGOLD_FLAGS_VREQ_CPU) {
		best_bank = _xgold_clk_pll_find_out_bank(pll, new_rate);
		XGOLD_CLK_SET_BANK(pll, best_bank);
	}

	return ret;
}

const struct clk_ops xgold_clk_pll_ops = {
	.prepare = xgold_clk_pll_prepare,
	.unprepare = xgold_clk_pll_unprepare,
	.enable = xgold_clk_pll_enable,
	.disable = xgold_clk_pll_disable,
	.is_enabled = xgold_clk_pll_is_enabled,
	.recalc_rate = xgold_clk_pll_recalc_rate,
	.round_rate = xgold_clk_pll_round_rate,
	.set_rate = xgold_clk_pll_set_rate,
};

static struct clk *xgold_clk_register_pll(struct device *dev, const char *name,
					  const char *parent_name,
					  unsigned long flags,
					  struct xgold_clk_pll_hw *pll_hw,
					  struct xgold_clk_pll_params *params,
					  enum pll_type type,
					  unsigned long rate,
					  unsigned sd_div)
{
	struct xgold_clk_pll *pll;
	struct clk *clk;
	struct clk_init_data init;

	/* allocate the pll clock */
	pll = kzalloc(sizeof(struct xgold_clk_pll), GFP_KERNEL);
	if (!pll) {
		pr_err(ALLOC_FAILED, "xgold pll clk", name);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &xgold_clk_pll_ops;
	init.flags = flags & ~CLK_XGOLD_FLAGS_MASK;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* Initialize xgold_pll struct */
	pll->pll_hw = pll_hw;
	pll->hw.init = &init;
	pll->type = type;
	pll->flags = flags;
	pll->params = params;
	pll->fixed_rate = rate;
	pll->sd_div = sd_div;

	/* Update parameters */
	if ((!(pll->flags & CLK_XGOLD_NO_PARAMETER_INIT)) &&
	    _xgold_clk_pll_set_parameters(pll)
	    )
		return ERR_PTR(-EINVAL);

	pll->cgu = xgold_clk_get_cgu(&pll->hw);

	/* Register the clock */
	clk = clk_register(dev, &pll->hw);

	if (IS_ERR(clk))
		kfree(pll);

	clk_register_clkdev(clk, name, NULL);

	return clk;
}

void __init of_xgold_pll_clk_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct clk *clk;
	const char *pll_type;
	struct device_node *pnode;
	struct xgold_clk_pll_hw *pll_hw;
	struct xgold_clk_pll_params *pll_params = NULL;
	unsigned long flags = 0;
	unsigned rate;
	enum pll_type type = unknown;
	unsigned sd_div = 1 << 16; /* use 65536 as default divisor factor for
				      digital pll */

	pll_hw = kzalloc(sizeof(struct xgold_clk_pll_hw), GFP_KERNEL);
	if (!pll_hw) {
		pr_err(ALLOC_FAILED, "pll hw", clk_name);
		return;
	}

	/* Check banks parameters is present, otherwise assume
	   the pll does not have bank support */

	if (of_property_read_bool(node, "intel,frequency-banks"))
		of_property_read_u32(node, "intel,frequency-banks",
				     &pll_hw->freq_banks);
	else
		pll_hw->freq_banks = 1;

	if (of_property_read_bool(node, "intel,mux-banks"))
		of_property_read_u32(node, "intel,mux-banks",
				     &pll_hw->mux_banks);
	else
		pll_hw->mux_banks = 1;

	of_property_read_string(node, "clock-output-names", &clk_name);
	of_property_read_string(node, "intel,pll-type", &pll_type);

	if (!strcmp(pll_type, "analog"))
		type = analog;
	else if (!strcmp(pll_type, "digital"))
		type = digital;
	else if (!strcmp(pll_type, "fixed"))
		type = fixed;

	if (type == unknown) {
		pr_err("Pll type %s is unknown for clk %s!\n", pll_type,
		       clk_name);
		return;
	}

	if (of_property_read_u32(node, "clock-frequency", &rate)
	    && (type == fixed)) {
		pr_err("Fixed Pll %s requires a frequency", clk_name);
		return;
	}

	pnode = of_parse_phandle(node, "clocks", 0);

	flags |= xgold_clk_get_flags(node);

	pll_hw->up = kcalloc(pll_hw->freq_banks,
				sizeof(struct xgold_clk_reg), GFP_KERNEL);
	if (!pll_hw->up) {
		pr_err(ALLOC_FAILED, "hw up registers", clk_name);
		return;
	}

	pll_hw->shpu = kcalloc(pll_hw->freq_banks,
				sizeof(struct xgold_clk_reg),
		    GFP_KERNEL);
	if (!pll_hw->shpu) {
		pr_err(ALLOC_FAILED, "hw shup registers", clk_name);
		return;
	}

	if (type == fixed)
		goto pll_fixed;

	pll_params = kcalloc(pll_hw->freq_banks,
			sizeof(struct xgold_clk_pll_params), GFP_KERNEL);
	if (!pll_params) {
		pr_err(ALLOC_FAILED, "clk parameters", clk_name);
		return;
	}

	pll_hw->freqs = kcalloc(pll_hw->freq_banks,
				sizeof(unsigned long), GFP_KERNEL);
	if (!pll_hw->freqs) {
		pr_err(ALLOC_FAILED, "clk frequencies", clk_name);
		return;
	}

	pll_hw->mux = kcalloc(pll_hw->mux_banks,
				sizeof(struct xgold_clk_reg), GFP_KERNEL);
	if (!pll_hw->mux) {
		pr_err(ALLOC_FAILED, "hw mux registers", clk_name);
		return;
	}

	pll_hw->mult = kcalloc(pll_hw->freq_banks,
				sizeof(struct xgold_clk_reg), GFP_KERNEL);
	if (!pll_hw->mult) {
		pr_err(ALLOC_FAILED, "hw multipliers registers", clk_name);
		return;
	}

	if (type == analog) {
		pll_hw->div = kcalloc(pll_hw->freq_banks,
				sizeof(struct xgold_clk_reg), GFP_KERNEL);
		if (!pll_hw->div) {
			pr_err(ALLOC_FAILED, "hw dividers registers", clk_name);
			return;
		}
	}

	if (of_property_read_u32_array(node, "intel,pll-parameters",
				       (unsigned *)pll_params,
				       (sizeof(struct xgold_clk_pll_params) /
					sizeof(unsigned)
					* pll_hw->freq_banks)
	    )) {
		pr_err(PROPERTY_MISSING, "intel,pll-parameters", clk_name);
		return;
	}

	/* Parse the mandatory properties */

	if (xgold_clk_init_reg_array
	    (node, pll_hw->mux, "intel,pll-mux", pll_hw->mux_banks)) {
		pr_err(PROPERTY_MISSING, "intel,pll-mux", clk_name);
		return;
	}

	if (xgold_clk_init_reg_array
	    (node, pll_hw->mult, "intel,pll-mult", pll_hw->freq_banks)) {
		pr_err(PROPERTY_MISSING, "intel,pll-mult", clk_name);
		return;
	}

	if ((type == analog)
	    && xgold_clk_init_reg_array(node, pll_hw->div, "intel,pll-div",
					pll_hw->freq_banks)) {
		pr_err(PROPERTY_MISSING, "intel,pll-div", clk_name);
		return;
	}

pll_fixed:
	if (xgold_clk_init_reg_array
	    (node, pll_hw->up, "intel,pll-up", pll_hw->freq_banks)) {
		pr_err(PROPERTY_MISSING, "intel,pll-up", clk_name);
		return;
	}

	if (xgold_clk_init_reg
	    (node, &pll_hw->status_lock, "intel,pll-status-lock")) {
		pr_err(PROPERTY_MISSING, "intel,pll-status-lock", clk_name);
		return;
	}

	/* Optional register informations */
	xgold_clk_init_reg(node, &pll_hw->update, "intel,pll-update");

	if (xgold_clk_init_reg_array
	    (node, pll_hw->shpu, "intel,pll-shpu", pll_hw->freq_banks)) {
		kfree(pll_hw->shpu);
		pll_hw->shpu = NULL;
	}

	of_property_read_u32(node, "intel,pll-sd-div", &sd_div);

	clk =
	    xgold_clk_register_pll(NULL, clk_name, pnode->name, flags, pll_hw,
				   pll_params, type, rate, sd_div);
	if (clk)
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	xgold_create_clk_alias_if_any(node);
}
EXPORT_SYMBOL_GPL(of_xgold_pll_clk_setup);
