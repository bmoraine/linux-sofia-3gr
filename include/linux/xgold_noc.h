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

#ifndef XGOLD_NOC_H_
#define XGOLD_NOC_H_

struct noc_qos_list {
	struct list_head list;
	const char *name;
};

extern void xgold_noc_qos_set(char *name);
extern int of_noc_qos_populate(struct device *dev,
			struct device_node *np,
			struct noc_qos_list **qos);
extern void of_noc_qos_free(struct noc_qos_list *qos_root);
#endif
