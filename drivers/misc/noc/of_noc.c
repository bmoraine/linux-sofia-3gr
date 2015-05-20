/*
 ****************************************************************
 *
 *  Copyright (C) 2011-2014 Intel Mobile Communications GmbH
 *
 *    This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 ****************************************************************
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/interrupt.h>
#include <linux/xgold_noc.h>

#include "noc.h"


void of_noc_qos_free(struct noc_qos_list *qos_root)
{
	struct noc_qos_list *qos;
	list_for_each_entry(qos, &qos_root->list, list) {
		kfree(qos);
	}
}
EXPORT_SYMBOL(of_noc_qos_free);

int of_noc_qos_populate(struct device *dev,
			struct device_node *np,
			struct noc_qos_list **qos_root)
{
	int def_len;
	struct property *prop;

	prop = of_find_property(np, "intel,qos-ports", &def_len);
	if (prop != NULL) {
		struct noc_qos_list *qos;
		int i, ncfg = of_property_count_strings(np, "intel,qos-ports");
		*qos_root = (struct noc_qos_list *)
			devm_kzalloc(dev,
				sizeof(struct noc_qos_list), GFP_KERNEL);
		if (*qos_root == NULL) {
			dev_err(dev, "%s: Memory allocation failed!",
					__func__);
			return -ENOMEM;
		}
		INIT_LIST_HEAD(&(*qos_root)->list);
		for (i = 0; i < ncfg; i++) {
			qos = (struct noc_qos_list *)
				devm_kzalloc(dev,
					sizeof(struct noc_qos_list),
					GFP_KERNEL);
			if (qos == NULL) {
				dev_err(dev, "%s: Memory allocation failed!",
						__func__);
				of_noc_qos_free(*qos_root);
				return -ENOMEM;
			}
			of_property_read_string_index(np,
					"intel,qos-ports", i,
					&qos->name);

			dev_info(dev, "QoS port %s added to list\n", qos->name);
			list_add_tail(&qos->list,
					&(*qos_root)->list);
		}
	}
	return 0;
}
EXPORT_SYMBOL(of_noc_qos_populate);


