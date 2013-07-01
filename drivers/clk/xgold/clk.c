/*
* Copyright (C) 2014 Intel Mobile Communications GmbH
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
*/

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "xgold-clk.h"

#define XGOLD_MAX_CGU_DEVICES 2
static struct xgold_cgu_device xgold_cgu_devices[XGOLD_MAX_CGU_DEVICES];
static unsigned char cgu_in_use;

int xgold_create_clk_alias_if_any(struct device_node *np)
{
	int ret = 0, do_it = 0;
	const char *alias = NULL, *alias_dev = NULL;
	char *clk_id;

	ret = of_property_read_string(np, "clock-alias", &alias);
	if (!ret)
		do_it++;

	ret = of_property_read_string(np, "clock-alias-dev", &alias_dev);
	if ((ret) && (!do_it))
		return -ENODEV;

	clk_id = kstrdup(np->name, GFP_KERNEL);
	if (!clk_id)
		return -ENOMEM;

	ret = clk_add_alias(alias, alias_dev, clk_id, NULL);
	if (ret) {
		pr_err
		    ("Adding alias,dev \'%s,%s\' to clock \'%s\' failed!\n",
		     alias, alias_dev, clk_id);
		ret = -ENODEV;
		goto exit;
	}
	pr_debug("Alias,dev \'%s,%s\' to clock \'%s\' added.\n",
		 alias, alias_dev, clk_id);
	return 0;

exit:
	kfree(clk_id);
	return ret;
}

void xgold_clk_reg_write(struct xgold_clk_reg *clk_reg, unsigned int value)
{
	unsigned int val;
	unsigned long flags = 0;

	spin_lock_irqsave(clk_reg->lock, flags);

	if (value > xgold_reg_mask(clk_reg)) {
		pr_warn("%s clk reg value %d out of range\n",
			clk_reg->name, value);
	}

	val = readl(clk_reg->reg);
	val &= ~(xgold_reg_mask(clk_reg) << clk_reg->shift);
	val |= (value & xgold_reg_mask(clk_reg)) << clk_reg->shift;
	writel(val, clk_reg->reg);

	spin_unlock_irqrestore(clk_reg->lock, flags);
}

unsigned int xgold_clk_reg_read(struct xgold_clk_reg *clk_reg)
{
	unsigned int val;

	val = readl(clk_reg->reg);
	return (val >> clk_reg->shift) & xgold_reg_mask(clk_reg);
}

static spinlock_t *xgold_clk_get_reglock(unsigned offset)
{
	struct xgold_cgu_device *cgu = &xgold_cgu_devices[cgu_in_use - 1];
	if (offset > cgu->hw_length)
		return NULL;

	return &cgu->reg_locks[offset];
}

static void xgold_clk_print_clk_reg(struct xgold_clk_reg *reg)
{
	pr_debug("clk reg: %s @ offset %p, %d bits @ offset %d, spinlock %p\n",
		 reg->name, reg->reg, reg->width, reg->shift, reg->lock);
}

int _xgold_clk_init_reg(struct xgold_clk_reg *clk_reg,
			unsigned *values, const char *propname)
{
	struct xgold_cgu_device *cgu = &xgold_cgu_devices[cgu_in_use - 1];
	clk_reg->name = propname;

	clk_reg->reg = cgu->hw_base + values[0];
	clk_reg->shift = (unsigned char)values[1];
	clk_reg->width = (unsigned char)values[2];
	clk_reg->lock = xgold_clk_get_reglock(values[0] / 4);

	if (clk_reg->lock == NULL)
		return -EINVAL;

	xgold_clk_print_clk_reg(clk_reg);

	return 0;
}

int xgold_clk_init_reg_array(struct device_node *np,
			     struct xgold_clk_reg *clk_reg,
			     const char *propname, unsigned nr)
{
	int ret, i;
	unsigned out_values[nr * 3];

	ret = of_property_read_u32_array(np, propname, out_values, nr * 3);

	if (ret) {
		pr_devel("Could not find property %s\n", propname);
		for (i = 0; i < nr; i++)
			clk_reg[i].reg = NULL;

		return ret;
	}

	for (i = 0; i < nr; i++) {
		ret =
		    _xgold_clk_init_reg(&clk_reg[i], &out_values[i * 3],
					propname);
		if (ret)
			return ret;
	}

	return 0;

}

int xgold_clk_init_reg(struct device_node *np,
		       struct xgold_clk_reg *clk_reg, const char *propname)
{
	return xgold_clk_init_reg_array(np, clk_reg, propname, 1);
}

unsigned long xgold_clk_get_flags(struct device_node *node)
{
	unsigned long flags = 0;
	if (of_property_read_bool(node, "intel,flags-noinit"))
		flags |= CLK_XGOLD_NO_PARAMETER_INIT;

	if (of_property_read_bool(node, "intel,flags-ignore-unused"))
		flags |= CLK_IGNORE_UNUSED;

	if (of_property_read_bool(node, "intel,flags-set-rate-gate"))
		flags |= CLK_SET_RATE_GATE;

	if (of_property_read_bool(node, "intel,flags-set-parent-gate"))
		flags |= CLK_SET_PARENT_GATE;

	if (of_property_read_bool(node, "intel,flags-is-root"))
		flags |= CLK_IS_ROOT;

	if (of_property_read_bool(node, "intel,flags-is-basic"))
		flags |= CLK_IS_BASIC;

	if (of_property_read_bool(node, "intel,flags-get-rate-nocache"))
		flags |= CLK_GET_RATE_NOCACHE;

	if (of_property_read_bool(node, "intel,flags-divider_div_fix"))
		flags |= CLK_XGOLD_DIVIDER_DIV_FIX;

	if (of_property_read_bool(node, "intel,flags-divider_one_based"))
		flags |= CLK_XGOLD_DIVIDER_ONE_BASED;

	if (of_property_read_bool(node, "intel,flags-divider_power_of_two"))
		flags |= CLK_XGOLD_DIVIDER_POWER_OF_TWO;

	if (of_property_read_bool(node, "intel,flags-divider_mult_of_two"))
		flags |= CLK_XGOLD_DIVIDER_MULT_OF_TWO;

	if (of_property_read_bool(node, "intel,flags-divider_add_one"))
		flags |= CLK_XGOLD_DIVIDER_ADD_ONE;

	if (of_property_read_bool(node, "intel,flags-divider_only_even"))
		flags |= CLK_XGOLD_DIVIDER_ONLY_EVEN;

	if (of_property_read_bool(node, "intel,flags-req-force"))
		flags |= CLK_XGOLD_REQ_FORCE_RCOUT;

	if (of_property_read_bool(node, "intel,flags-cpu"))
		flags |= CLK_XGOLD_FLAGS_VREQ_CPU;

	return flags;
}

void __init of_xgold_fixed_divider_clk_setup(struct device_node *node)
{

	struct clk *clk;
	struct device_node *pnode;
	const char *clk_name = node->name;
	unsigned int mult, div;
	unsigned long flags = 0;
	pnode = of_parse_phandle(node, "clocks", 0);
	of_property_read_string(node, "clock-output-names", &clk_name);
	of_property_read_u32(node, "intel,mult", &mult);
	of_property_read_u32(node, "intel,divider", &div);

	clk = clk_register_fixed_factor(NULL, clk_name,
					pnode->name, flags, mult, div);

	if (clk)
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

}

void __init of_xgold_divider_clk_setup(struct device_node *node)
{

	struct clk *clk;
	const char *clk_name = node->name;
	const char *parent_name = of_clk_get_parent_name(node, 0);
	unsigned long flags, div_flags = 0, xgold_flags;
	struct xgold_clk_reg clk_reg;
	int ret;

	of_property_read_string(node, "clock-output-names", &clk_name);
	ret = xgold_clk_init_reg(node, &clk_reg, "intel,divider-div");
	if (ret)
		return;

	xgold_flags = xgold_clk_get_flags(node);

	flags = xgold_flags & ~CLK_XGOLD_FLAGS_MASK;

	if (xgold_flags & CLK_XGOLD_DIVIDER_ONE_BASED)
		div_flags |= CLK_DIVIDER_ONE_BASED;

	if (xgold_flags & CLK_XGOLD_DIVIDER_POWER_OF_TWO)
		div_flags |= CLK_DIVIDER_POWER_OF_TWO;

	clk =
	    clk_register_divider(NULL, clk_name, parent_name, flags,
				 clk_reg.reg, clk_reg.shift, clk_reg.width,
				 div_flags, clk_reg.lock);

	if (clk)
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}

struct xgold_cgu_device *xgold_clk_get_cgu(struct clk_hw *hw)
{
	return &xgold_cgu_devices[cgu_in_use - 1];
}

static unsigned char _xgold_clk_get_bank_from_vreq(unsigned *map, unsigned vreq)
{
	unsigned i;
	for (i = 0; i < CGU_MAX_BANK; i++) {
		if (map[i] == vreq)
			return i;
	}
	return 0;
}

static unsigned char _xgold_clk_get_vreq_from_bank(unsigned *map, unsigned bank)
{
	return map[bank];
}

static unsigned _xgold_clk_get_bank(struct xgold_cgu_device *cgu,
				    struct xgold_clk_reg *reg)
{
	unsigned vreq;
	vreq = xgold_clk_reg_read(reg);
	return _xgold_clk_get_bank_from_vreq(cgu->vreq_mapping, vreq);
}

static int _xgold_clk_set_bank(struct xgold_cgu_device *cgu,
			       struct xgold_clk_reg *reg, unsigned bank)
{

	unsigned char vreq =
	    _xgold_clk_get_vreq_from_bank(cgu->vreq_mapping, bank);

	xgold_clk_reg_write(reg, vreq);

	return 0;
}

unsigned xgold_clk_get_bank(struct xgold_cgu_device *cgu, unsigned long flags)
{
	unsigned bank = 0;

	if ((flags & CLK_XGOLD_FLAGS_VREQ_CPU) && (cgu->cpu_vreq != NULL)) {
		bank = _xgold_clk_get_bank(cgu, cgu->cpu_vreq);
	} else {
		if (cgu->global_vreq)
			bank = _xgold_clk_get_bank(cgu, cgu->global_vreq);
	}

	pr_debug("Get bank %d with flags %lx\n", bank, flags);

	return bank;
}

unsigned xgold_clk_get_new_bank(struct xgold_cgu_device *cgu,
				unsigned long flags)
{

	pr_debug("Get new bank cpu %d, global %d with flags %lx\n",
		 cgu->new_cpu_bank, cgu->new_global_bank, flags);

	if ((flags & CLK_XGOLD_FLAGS_VREQ_CPU) && (cgu->cpu_vreq != NULL))
		return cgu->new_cpu_bank;

	if (cgu->global_vreq)
		return cgu->new_global_bank;

	return 0;
}

int xgold_clk_set_bank(struct xgold_cgu_device *cgu, unsigned long flags,
		       unsigned bank)
{
	pr_debug("Set bank %d with flags %lx\n", bank, flags);

	if ((flags & CLK_XGOLD_FLAGS_VREQ_CPU) && (cgu->cpu_vreq != NULL)) {
		cgu->new_cpu_bank = bank;
		return _xgold_clk_set_bank(cgu, cgu->cpu_vreq, bank);
	}

	if (cgu->global_vreq) {
		cgu->new_global_bank = bank;
		return _xgold_clk_set_bank(cgu, cgu->global_vreq, bank);
	}

	return 0;
}

int xgold_clk_prepare_bank(struct xgold_cgu_device *cgu, unsigned long flags,
			   unsigned bank)
{

	pr_debug("Prepare bank %d with flags %lx\n", bank, flags);

	if ((flags & CLK_XGOLD_FLAGS_VREQ_CPU) && (cgu->cpu_vreq != NULL)) {
		cgu->new_cpu_bank = bank;
	} else {
		if (cgu->global_vreq)
			cgu->new_global_bank = bank;
	}

	return 0;
}

static void __init of_xgold_clk_spcu(struct device_node *np)
{
	int def_len, ret;
	struct property *prop;
	struct xgold_cgu_device *cgu = &xgold_cgu_devices[cgu_in_use - 1];

	prop = of_find_property(np, "intel,reg-defaults", &def_len);
	if (prop != NULL) {
		unsigned i;
		struct _def_values {
			unsigned offset;
			unsigned value;
		} *def_values;

		def_values = kzalloc(def_len, GFP_KERNEL);
		if (def_values == NULL) {
			pr_err("%s: Memory allocation failed!", __func__);
			BUG();
		}
		ret = of_property_read_u32_array(np,
						 "intel,reg-defaults",
						 (u32 *) def_values,
						 def_len / sizeof(unsigned));
		if (ret) {
			pr_err("%s: Could not read spcu defaults values",
			       __func__);
			BUG();
		}
		for (i = 0; i < (def_len / sizeof(struct _def_values)); i++) {
			iowrite32(def_values[i].value,
				  cgu->hw_base + def_values[i].offset);
		}
	}
	if (of_property_read_bool(np, "intel,global-vreq")) {

		cgu->global_vreq =
		    kzalloc(sizeof(struct xgold_clk_reg), GFP_KERNEL);
		if (xgold_clk_init_reg
		    (np, cgu->global_vreq, "intel,global-vreq")) {
			pr_err("%s: Global Vreq register init failed !",
			       __func__);
			BUG();
		}

		cgu->new_global_bank =
		    _xgold_clk_get_bank(cgu, cgu->global_vreq);
	}

	if (of_property_read_bool(np, "intel,cpu-vreq")) {

		cgu->cpu_vreq =
		    kzalloc(sizeof(struct xgold_clk_reg), GFP_KERNEL);
		if (xgold_clk_init_reg(np, cgu->cpu_vreq, "intel,cpu-vreq")) {
			pr_err("%s: Cpu Vreq register init failed !", __func__);
			BUG();
		}

		cgu->new_cpu_bank = _xgold_clk_get_bank(cgu, cgu->cpu_vreq);
	}

	/* FIXME: Vreq mapping should come from kmalloc */
	prop = of_find_property(np, "intel,vreq-mapping", &def_len);

	if (prop)
		of_property_read_u32_array(np, "intel,vreq-mapping",
					   cgu->vreq_mapping,
					   (def_len / sizeof(u32)));

	of_node_put(np);

}

void __init of_xgold_clk_setup(struct device_node *node)
{
	int i;
	struct device_node *cgu_node =
	    of_parse_phandle(node, "intel,reg-phys", 0);
	struct xgold_cgu_device *cgu = &xgold_cgu_devices[cgu_in_use];
	struct resource cgu_res;

	if (cgu_in_use == XGOLD_MAX_CGU_DEVICES) {
		pr_err("Two many CGU devices registered\n");
		BUG();
	}

	if (cgu_node == NULL) {
		pr_err("Could not find reg-phys information node\n");
		BUG();
	}

	cgu->hw_base = of_iomap(cgu_node, 0);
	if (cgu->hw_base == NULL) {
		pr_err("Could not find io remap cgu registers\n");
		BUG();
	}

	if (of_address_to_resource(cgu_node, 0, &cgu_res))
		BUG();

	cgu->phys_hw_base = (void __iomem *)cgu_res.start;
	cgu->hw_length = (unsigned)resource_size(&cgu_res);

	pr_debug("CGU remapped base address %p\n", cgu->hw_base);
	pr_debug("CGU HW base address %p, length %x\n", cgu->phys_hw_base,
		 cgu->hw_length);

	cgu->reg_locks = kcalloc(cgu->hw_length / 4, sizeof(spinlock_t),
				 GFP_KERNEL);
	if (cgu->reg_locks == NULL)
		BUG();

	for (i = 0; i < (cgu->hw_length / 4); i++)
		spin_lock_init(&cgu->reg_locks[i]);

	cgu_in_use++;
	of_xgold_clk_spcu(node);
}

static const struct of_device_id clk_match[] __initconst = {
	{
		.compatible = "intel,xgold-clock",
		.data = of_xgold_clk_setup,
	},
	{
		.compatible = "fixed-clock",
		.data = of_fixed_clk_setup,
	},
	{
		.compatible = "intel,xgold-pll",
		.data = of_xgold_pll_clk_setup,
	},
	{	.compatible = "intel,xgold-phs",
		.data = of_xgold_phs_clk_setup,
	},
	{
		.compatible = "intel,xgold-fixed-divider",
		.data = of_xgold_fixed_divider_clk_setup,
	},
	{
		.compatible = "intel,xgold-divider",
		.data = of_xgold_divider_clk_setup,
	},
	{
		.compatible = "intel,xgold-mux",
		.data = of_xgold_mux_clk_setup,
	},
	{
		.compatible = "intel,xgold-block-clock",
		.data = of_xgold_clock_clk_setup,
	},
	{
		.compatible = "intel,xgold-req",
		.data = of_xgold_req_clk_setup,
	},
	{}
};

void __init xgold_init_clocks(void)
{
	/* Register all the xgold clocks from DT */
	of_clk_init(clk_match);
}
