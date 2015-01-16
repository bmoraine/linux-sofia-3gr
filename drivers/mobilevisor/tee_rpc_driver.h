/*
 * Copyright (c) 2014, Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef __TEE_RPC_DRIVER_H__
#define __TEE_RPC_DRIVER_H__

#include <linux/cdev.h>
#include <linux/types.h>

struct tee_rpc_client_ctx {
	struct cdev cdev;
};

struct pvec {
	u32 phys_addr;
	void __user *user_addr;
	size_t len;
	bool is_contig;
};


#endif
