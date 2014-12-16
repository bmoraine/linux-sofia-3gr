/* ----------------------------------------------------------------------------
 *  Copyright (C) 2014 Intel Mobile Communications GmbH

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

  ---------------------------------------------------------------------------*/

#define pr_fmt(fmt) "pmic_reg_access: "fmt

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <sofia/vmm_pmic.h>
#include <linux/module.h>

#include <linux/slab.h>
#include <linux/string.h>

#include <linux/of.h>

#define PMIC_REG_ACCESS_NODE_NAME "vmm_pmic-ext"


struct pmic_reg_access {
	u32 *disallowed_regs;
	u32 disallowed_regs_len;
};

static struct pmic_reg_access pmic_access;


static DEFINE_MUTEX(setfield_lock);

int32_t pmic_reg_set_field(uint32_t reg_address,
				uint8_t mask, uint8_t data)
{
	uint32_t reg_val = 0, field_val;
	int ret, i, dis_arr_len = pmic_access.disallowed_regs_len;

	for (i = 0; i < dis_arr_len; ++i)
		if (reg_address == pmic_access.disallowed_regs[i])
			return -EINVAL;

	mutex_lock(&setfield_lock);

	ret = vmm_pmic_reg_read(reg_address, &reg_val);
	if (ret)
		goto out;

	reg_val &= (uint32_t) ~(mask);
	field_val = data & mask;
	reg_val |= field_val;

	ret = vmm_pmic_reg_write(reg_address, reg_val);
	if (ret)
		goto out;

out:
	mutex_unlock(&setfield_lock);
	return ret;
}
EXPORT_SYMBOL(pmic_reg_set_field);

static int32_t __init pmic_reg_access_init(void)
{
	struct device_node *np;
	struct property *prop;
	int size;

	np = of_find_node_by_name(NULL, PMIC_REG_ACCESS_NODE_NAME);
	if (!np) {
		pr_err("cannot find node '%s'\n", PMIC_REG_ACCESS_NODE_NAME);
		return -ENODEV;
	}

	prop = of_find_property(np,
		PMIC_REG_ACCESS_NODE_NAME",disallowed_regs", &size);
	if (!prop) {
		pr_err("cannot find property '%s'\n",
				PMIC_REG_ACCESS_NODE_NAME",disallowed_regs");
		return -ENODEV;
	}

	if (0 == size) {
		pr_info("property '%s' empty. All addresses allowed\n",
				PMIC_REG_ACCESS_NODE_NAME",disallowed_regs");
		goto out;
	}

	pmic_access.disallowed_regs = kmalloc(size, GFP_KERNEL);
	if (!pmic_access.disallowed_regs)
		return -ENOMEM;

	pmic_access.disallowed_regs_len = size / sizeof(u32);

	if (of_property_read_u32_array(np,
		PMIC_REG_ACCESS_NODE_NAME",disallowed_regs",
			pmic_access.disallowed_regs,
			pmic_access.disallowed_regs_len)) {

		kfree(pmic_access.disallowed_regs);

		pmic_access.disallowed_regs = NULL;
		pmic_access.disallowed_regs_len = 0;

		pr_err("error reading '%s' property\n",
			PMIC_REG_ACCESS_NODE_NAME",disallowed_regs");
		return -ENODEV;
	}

out:
	pr_info("init OK\n");

	return 0;
}

static void __exit pmic_reg_access_exit(void)
{
	kfree(pmic_access.disallowed_regs);
}

subsys_initcall(pmic_reg_access_init);
module_exit(pmic_reg_access_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("pmic access extension module");
