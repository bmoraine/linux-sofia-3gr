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

#define IBFS_OK 0
#define IBFS_TRC_BUF_LGT  40

/* Remember to set access LvL in secure_vm -> sec_rpc.c */
enum t_rpc_if_grp {
	RPC_IF_SEC      = 0x00000000,
	RPC_IF_FUS,
	RPC_IF_CEU,
	RPC_IF_NVM,
	RPC_IF_SCU,
	RPC_IF_KEYMASTER,
	RPC_IF_IBFS,
	RPC_IF_RPMB     = 0x00000103,
	RPC_ATA_SEC     = 0x00001000,
	RPC_ATA_CEU,
	RPC_TST_SEC     = 0x00002000,
	RPC_IF_VOUCHER  = 0x00003000,
	RPC_TST_TEE     = 1080,
	RPC_IF_GRP_END  = 0x7FFFFFFF
};

enum t_rpc_opcode_ibfs_if {
	rpc_op_ibfs_open,
	rpc_op_ibfs_read,
	rpc_op_ibfs_write,
	rpc_op_ibfs_erase,
	rpc_op_ibfs_alloc,
	rpc_op_ibfs_free,
	rpc_op_ibfs_trace,
	rpc_op_ibfs_close,
	rpc_op_ibfs_if_end = 0x7FFFFFFF
};

enum T_IBFS_TRC_TYPE {
	IBFS_TRC_INIT  = 1,
	IBFS_TRC_CLOSE = 2,
	IBFS_TRC_WRITE = 3,
	IBFS_TRC_READ  = 4,
	IBFS_TRC_ERASE = 5,
	IBFS_TRC_ALLOC = 6,
	IBFS_TRC_FREE  = 7
};

enum T_RPC_RESULT {
	RPC_SUCCESS    = 0,
	RPC_FAILURE    = 1,
	RPC_ENUM_SIZE  = 0x7FFFFFFF
};

struct T_IBFS_TRC_BUF_ELEM {
	enum T_IBFS_TRC_TYPE  trc_type;
	u32                   val0;
	u32                   val1;
};

struct T_IBFS_TRC_BUF {
	bool                        init;
	u32                         next_item;
	struct T_IBFS_TRC_BUF_ELEM buf[IBFS_TRC_BUF_LGT];
};

void rpc_handle_cmd(void *shared_mem);

int rpc_call(
	struct t_rpc_send_info *send_info,
	enum t_rpc_if_grp if_grp,
	u32 opcode,
	u8 *io_data,
	u32 *io_data_len,
	u32 max_len);

#ifdef MEM_TEST_STUB
int stub_call_sec_vm(int num_vec, struct pvec *phys_iov, int *remote_ret);
#endif

#endif /*__TEE_RPC_SEC_VM_H__ */
