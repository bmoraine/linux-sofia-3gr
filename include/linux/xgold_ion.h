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
	ION_HEAP_TYPE_DISPLAY_CARVEOUT = ION_HEAP_TYPE_CUSTOM,
	ION_HEAP_TYPE_VIDEO_CARVEOUT,
};

#endif /* _LINUX_XGOLD_ION_H */
