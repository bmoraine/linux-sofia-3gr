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

#ifndef __TEE_RPC_SEC_VM_H__
#define __TEE_RPC_SEC_VM_H__

#include "tee_rpc_driver.h"
#include "vsec.h"

struct t_rpc_send_info {
	enum t_vsec_vm_id send_id; /* Who are you? */
	enum t_vsec_vm_id recv_id; /* Who do you want to call? */
	char vlink_name[8];      /* Which com path (vlink) should be used? */
};

enum t_rpc_if_grp {
	RPC_IF_SEC      = 0x00000000,
	RPC_IF_FUS,
	RPC_IF_CEU,
	RPC_IF_NVM,
	RPC_IF_SCU,
	RPC_ATA_SEC     = 0x00001000,
	RPC_TST_SEC     = 0x00002000,
	RPC_IF_GRP_END  = 0x7FFFFFFF
};

void rpc_handle_cmd(void *shared_mem);

int rpc_call(struct t_rpc_send_info *send_info, enum t_rpc_if_grp if_grp,
	     u32 opcode, u8 *io_data, u32 *io_data_len, u32 max_len);

#ifdef MEM_TEST_STUB
int stub_call_sec_vm(int num_vec, struct pvec *phys_iov, int *remote_ret);
#endif

#endif /*__TEE_RPC_SEC_VM_H__ */
