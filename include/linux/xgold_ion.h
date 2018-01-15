/*
 * include/linux/xgold_ion.h
 *
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2012 Intel Corp.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_XGOLD_ION_H
#define _LINUX_XGOLD_ION_H

#ifndef __KERNEL__
#include <linux/ion.h>
#endif
#include <linux/compat.h>


#ifdef __KERNEL__
extern struct ion_handle *ion_handle_get_by_id(struct ion_client *client,
						int id);
extern int xgold_ion_handler_init(struct device_node *node,
	struct ion_device *idev);
extern void xgold_ion_handler_exit(void);
#endif

struct xgold_ion_get_params_data {
	int handle;
	size_t size;
	unsigned long addr;
};

enum {
	XGOLD_ION_GET_PARAM = 0,
	XGOLD_ION_ALLOC_SECURE,
	XGOLD_ION_FREE_SECURE,
};

enum {
	ION_HEAP_TYPE_SECURE = ION_HEAP_TYPE_CUSTOM,
};

#define ION_HEAP_TYPE_SECURE_MASK	(1 << ION_HEAP_TYPE_SECURE)

#ifdef __KERNEL__
#ifdef CONFIG_COMPAT
struct compat_xgold_ion_get_params_data {
	compat_int_t handle;
	compat_size_t size;
	compat_long_t addr;
};

extern int compat_put_xgold_ion_custom_data(unsigned int arg, struct
		xgold_ion_get_params_data __user *data);
extern int compat_get_xgold_ion_custom_data(
			struct compat_xgold_ion_get_params_data __user *data32,
			struct xgold_ion_get_params_data __user *data);
extern struct xgold_ion_get_params_data __user
		*compat_xgold_ion_get_param(unsigned int arg);
#else
struct compat_xgold_ion_get_params_data;

static inline int compat_put_xgold_ion_custom_data(unsigned int arg, struct
		xgold_ion_get_params_data __user *data)
{
	return 0;
}

static inline struct xgold_ion_get_params_data __user
		*compat_xgold_ion_get_param(unsigned int arg)
{
	return NULL;
}
#endif
#endif

#endif /* _LINUX_XGOLD_ION_H */
