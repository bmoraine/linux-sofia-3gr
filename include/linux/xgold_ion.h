/*
 * include/linux/xgold_ion.h
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_XGOLD_ION_H
#define _LINUX_XGOLD_ION_H

#include <linux/ion.h>

struct xgold_ion_get_params_data {
	int handle;
	size_t size;
	unsigned long addr;
};

enum {
	XGOLD_ION_GET_PARAM = 0,
};

enum {
	ION_HEAP_TYPE_DISPLAY_CARVEOUT = ION_HEAP_TYPE_CUSTOM,
	ION_HEAP_TYPE_VIDEO_CARVEOUT,
};

#endif /* _LINUX_XGOLD_ION_H */
