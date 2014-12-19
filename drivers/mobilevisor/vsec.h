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

#ifndef __VSEC_H_
#define __VSEC_H_

#define VSEC_SUCCESS 0
#define VSEC_FAILURE 1
#define MAX_VLINK_NAME_SIZE 8

enum t_vsec_vm_id {
	VSEC_VM_MODEM   = 1,
	VSEC_VM_LINUX   = 2,
	VSEC_VM_SECURE  = 3,
	VSEC_VM_END     = 0x7FFFFFFF
};

u8 vsec_get_context_entry_id(char *vlink_name, enum t_vsec_vm_id peer_id);
void *vsec_get_shared_mem(u8 entry_id);
u32 vsec_get_shared_mem_size(u8 entry_id);
u32 vsec_call(u8 entry_id);
u32 vsec_init(void);

#endif /* VSEC_H */
