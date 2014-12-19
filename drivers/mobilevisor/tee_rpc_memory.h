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

#ifndef __TEE_RPC_MEMORY_H__
#define __TEE_RPC_MEMORY_H__

#include <linux/uio.h>

#include "tee_rpc_driver.h"
#include "tee_rpc_driver_abi.h"

/*!
 * \brief map_user_pages
 * Map the user provided iovec to an iovec containing kernel memory
 * \param user_msg [IN] The user message as received from the ioctl
 * \param num_v [OUT]The number of entries in the iovector list
 * \param kiov [OUT] The iovec that contains the physical addresses
 * \return 0 on success
 */
int tee_rpc_map_user_pages(struct tee_message __user *user_msg,
			   int *num_v, struct pvec **kiov,
			   u32 *service, int *cmd_id);

int tee_rpc_unmap_user_pages(int num_v, struct pvec *kiov);

void free_pvec(int num_vec, struct pvec *vector);


#endif
