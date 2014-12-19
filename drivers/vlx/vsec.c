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

#include <linux/semaphore.h>

#include "vrpc.h"
#include "vsec.h"
#include "sec_rpc.h"

#define RPC_INDIRECT_CALL 0
#define RPC_DIRECT_CALL 1

struct t_vsec_ctx {
	char vlink_name[MAX_VLINK_NAME_SIZE];
	enum t_vsec_vm_id peer_id;
	bool is_server;
	struct vrpc_t *vrpc;
	struct semaphore call_sem;
};

static struct t_vsec_ctx vsec_entries[] = {
	{"vsec", VSEC_VM_SECURE, false, NULL},
	{"vsec", VSEC_VM_SECURE, true, NULL}
};


const u32 n_vsec_entries = sizeof(vsec_entries)/sizeof(struct t_vsec_ctx);


static struct t_vsec_ctx *vsec_get_context(u8 entry_id)
{
	return &vsec_entries[entry_id];
}

static void vsec_client_ready_cb(void *cookie)
{
	/* Release call semaphore after server has performed handshake */
	up((struct semaphore *)cookie);
	pr_debug("VRPC communication established.\n");
}

u8 vsec_get_context_entry_id(char *vlink_name, enum t_vsec_vm_id peer_id)
{
	u8 i;
	u8 entry_id = 0xFF;

	if (vlink_name == NULL)
		return entry_id;

	/* Run through VSEC members */
	for (i = 0; i < n_vsec_entries; i++) {
		/* Check if VSEC entry matches input specificiations */
		if (strncmp(vsec_entries[i].vlink_name, vlink_name,
			    MAX_VLINK_NAME_SIZE) == 0 &&
		    vsec_entries[i].peer_id == peer_id &&
		    vsec_entries[i].is_server == false) {
			/* Found matching VSEC entry */
			entry_id = i;
			break;
		}
	}

	return entry_id;
}

void *vsec_get_shared_mem(u8 entry_id)
{
	struct t_vsec_ctx *vsec = vsec_get_context(entry_id);
	return vrpc_data(vsec->vrpc);
}

u32 vsec_get_shared_mem_size(u8 entry_id)
{
	struct t_vsec_ctx *vsec = vsec_get_context(entry_id);
	return (u32)vrpc_maxsize(vsec->vrpc);
}

u32 vsec_call(u8 entry_id)
{
	u32 vsec_result = -1;
	u32 size;
	struct t_vsec_ctx *vsec = vsec_get_context(entry_id);

	/* Obtain call semaphore (only client calls) */
	if (!vsec->is_server) {
		if (down_interruptible(&vsec->call_sem)) {
			pr_err("Semaphore aquire interupted\n");
			vsec_result = -ERESTARTSYS;
			goto cleanup;
		}
	}

	/* Perform vRPC call */
	vsec_result = vrpc_call(vsec->vrpc, &size);

	/* Release call semaphore (only client calls) */
	if (!vsec->is_server)
		up(&vsec->call_sem);

cleanup:
	return vsec_result;
}

u32 vsec_server_dispatch_cb(void *cookie, u32 size)
{
	/* TODO SHOULD THIS BE ENABLED BY DEFAULT */
	pr_debug("Handle XIRQ request\n");
	rpc_handle_cmd(cookie);
	return 0;
}

bool vsec_setup_vrpc(struct t_vsec_ctx *vsec)
{
	bool result = false;

	/* Look up specified vRPC context */
	for (;;) {
		pr_debug("Perform %s lookup\n", vsec->is_server ?
			 "server" : "client");

		if (vsec->is_server)
			vsec->vrpc = vrpc_server_lookup(vsec->vlink_name,
							vsec->vrpc);
		else
			vsec->vrpc = vrpc_client_lookup(vsec->vlink_name,
							vsec->vrpc);

		pr_debug("Lookup returned 0x%08X\n", (u32)vsec->vrpc);

		if (vsec->vrpc == NULL) {
			pr_err("No VLINK found\n");
			goto cleanup;
		}

		/* Verify the peer id is as expected or else we
		   release it again */
		if (vrpc_peer_id(vsec->vrpc) == (u32)vsec->peer_id)
			break;
		else
			vrpc_release(vsec->vrpc);
	}


	if (vsec->is_server) {
		/* Open server connection */
		if (vrpc_server_open(vsec->vrpc, vsec_server_dispatch_cb,
				     vrpc_data(vsec->vrpc), RPC_INDIRECT_CALL))
			goto cleanup;
	} else {
		/* init semaphore to ensure that server has come online */
		sema_init(&vsec->call_sem, 1);

		/* Open client connection */
		if (vrpc_client_open(vsec->vrpc, vsec_client_ready_cb,
				     (void *)&vsec->call_sem))
			goto cleanup;
	}

	/* Everything went well */
	result = true;

cleanup:
	return result;
}

u32 vsec_init(void)
{
	u32 i;

	pr_debug("Initialize VSEC driver\n");

	for (i = 0; i < n_vsec_entries; i++) {
		if (vsec_setup_vrpc(&vsec_entries[i]) != true) {
			pr_err("Initialization failed at entry %d", i);
			return false;
		}
	}

	return true;
}
