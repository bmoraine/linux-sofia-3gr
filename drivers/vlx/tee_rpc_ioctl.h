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

#ifndef __TEE_RPC_IOCTL_H__
#define __TEE_RPC_IOCTL_H__

#include <linux/fs.h>


long tee_rpc_ioctl(struct file *flip, unsigned int cmd, unsigned long arg);

long tee_rpc_compat_ioctl(struct file *flip, unsigned int cmd,
			  unsigned long arg);

#endif
