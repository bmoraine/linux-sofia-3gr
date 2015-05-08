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
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/kthread.h>

#include <sofia/mv_gal.h>
#include <sofia/mv_ipc.h>
#include <sofia/mv_hypercalls.h>

#include "vsec.h"
#include "sec_rpc.h"

#define RPC_INDIRECT_CALL 0
#define RPC_DIRECT_CALL 1

enum vsec_status {
	VSEC_STATUS_CONNECT = 0,
	VSEC_STATUS_DISCONNECT
};

enum vsec_rpc_event {
	VSEC_RPC_EVENT_CLIENT = 0,
	VSEC_RPC_EVENT_SERVER,
};

struct vsec_rpc {
	uint32_t max_size;
	uint8_t *data;
};

struct vsec_share_ctx {
	uint32_t handshake;	
	uint8_t share_data[0];
};

struct t_vsec_ctx {
	uint32_t token;
	uint32_t share_mem_size;
	char *cmdline;
	enum vsec_status status;
	uint32_t server_index;
	wait_queue_head_t open_wq;

	uint32_t is_client_wait;
	//uint32_t server_responded;
	struct semaphore client_wq;
	struct mutex client_mutex;

	struct semaphore server_sem;

	struct vsec_rpc vsec_rpc_ctx[2];
	
	/* point to share memory */
	struct vsec_share_ctx *vsec_share;
};
 
static struct t_vsec_ctx vsec_ctx;

u8 vsec_get_context_entry_id(char *vlink_name, enum t_vsec_vm_id peer_id)
{
	return !vsec_ctx.server_index;
}

void *vsec_get_shared_mem(u8 entry_id)
{
	if(entry_id == vsec_ctx.server_index) {
		panic("invalid entry id for vsec\n");
	}
	
	return vsec_ctx.vsec_rpc_ctx[!vsec_ctx.server_index].data;
}

u32 vsec_get_shared_mem_size(u8 entry_id)
{
	if(entry_id == vsec_ctx.server_index) {
		panic("invalid entry id for vsec\n");
	}
	
	return vsec_ctx.vsec_rpc_ctx[!vsec_ctx.server_index].max_size;

}

u32 vsec_call(u8 entry_id)
{
	struct t_vsec_ctx *p_vsec_ctx = &vsec_ctx;

	while (p_vsec_ctx->status != VSEC_STATUS_CONNECT) {
		if(wait_event_interruptible(p_vsec_ctx->open_wq,
                            		p_vsec_ctx->status == VSEC_STATUS_CONNECT))
        	return VSEC_FAILURE;
	}

	if (entry_id != p_vsec_ctx->server_index) {

		/* Only one thread can perform RPC call */
		if (mutex_lock_interruptible(&p_vsec_ctx->client_mutex))
        	return 0;

		//p_vsec_ctx->server_responded = 0;

		/* Perform RPC call (post client event) */
		mv_ipc_mbox_post(p_vsec_ctx->token, VSEC_RPC_EVENT_CLIENT);

		/* Waiting for response */
		p_vsec_ctx->is_client_wait = 1;

		down_interruptible(&p_vsec_ctx->client_wq);

		/* Race condition protection */
		if (p_vsec_ctx->status != VSEC_STATUS_CONNECT) {
        	mutex_unlock(&p_vsec_ctx->client_mutex);
        	return VSEC_FAILURE;
		}

    /*
		if (wait_event_interruptible(p_vsec_ctx->client_wq,
			p_vsec_ctx->server_responded == 1)) {
			mutex_unlock(&p_vsec_ctx->client_mutex);
			return VSEC_FAILURE;
		}
      */
      
		p_vsec_ctx->is_client_wait = 0;

		 /* Disconnect exit */
		if (p_vsec_ctx->status != VSEC_STATUS_CONNECT) {
		    mutex_unlock(&p_vsec_ctx->client_mutex);
		    return VSEC_FAILURE;
		}

		mutex_unlock(&p_vsec_ctx->client_mutex);

	} else {
		return VSEC_FAILURE;
	}

	return VSEC_SUCCESS;
}

static int vsec_server_thread(void *cookie)
{
	struct t_vsec_ctx *p_vsec_ctx = (struct t_vsec_ctx *)cookie;

	while(1) {
		/* wait for server event */
		down_interruptible(&p_vsec_ctx->server_sem);

		if (p_vsec_ctx->status != VSEC_STATUS_CONNECT)
			continue;
		pr_debug("vsec server started %d\n",p_vsec_ctx->server_index);
		/* Dispatch */
		rpc_handle_cmd(p_vsec_ctx->vsec_rpc_ctx[p_vsec_ctx->server_index].data);
		pr_debug("vsec server stopped\n");
		
		mv_ipc_mbox_post(p_vsec_ctx->token, VSEC_RPC_EVENT_SERVER);
	}
	
	return 0;
}


static void vsec_on_connect(uint32_t token, void *cookie)
{
	struct t_vsec_ctx *p_vsec_ctx = (struct t_vsec_ctx *)cookie;
	pr_debug("vsec_on_connect\n");
	/* Do handshake */
	if (p_vsec_ctx->vsec_share->handshake == mv_gal_os_id()) {
		p_vsec_ctx->server_index = 1;
		pr_debug("vsec_on_connect server_idex 1\n");
	} else {
		pr_debug("vsec_on_connect server_idex 0\n");
		p_vsec_ctx->server_index = 0;
	}
	
	/* Set connect */
	p_vsec_ctx->status = VSEC_STATUS_CONNECT;

	wake_up_interruptible(&p_vsec_ctx->open_wq);
}

static void vsec_on_disconnect(uint32_t token, void *cookie)
{
	struct t_vsec_ctx *p_vsec_ctx = (struct t_vsec_ctx *)cookie;
	
	p_vsec_ctx->status = VSEC_STATUS_DISCONNECT;

	/* wake up client if still waiting*/
	if (p_vsec_ctx->is_client_wait)
		wake_up_interruptible(&p_vsec_ctx->client_wq);
}

static void vsec_on_event(uint32_t token, uint32_t event_id, void *cookie)
{
	struct t_vsec_ctx *p_vsec_ctx = (struct t_vsec_ctx *)cookie;
	switch (event_id) {
	case VSEC_RPC_EVENT_CLIENT:
		pr_debug("vsec client event\n");
		up(&p_vsec_ctx->server_sem);
		break;
	case VSEC_RPC_EVENT_SERVER:
	  //p_vsec_ctx->server_responded = 1;
		pr_debug("vsec server event\n");
		up(&p_vsec_ctx->client_wq);
		break;
	default:
		break;
	}
}

static struct mbox_ops vsec_ops = {
	.on_connect    = vsec_on_connect,
	.on_disconnect = vsec_on_disconnect,
	.on_event      = vsec_on_event
};

uint32_t vsec_initialised = 0;

u32 vsec_init(void)
{
	uint8_t *pshare_mem;
	struct t_vsec_ctx *p_vsec_ctx = &vsec_ctx;

	pr_debug("vsec_init\n");
	
	if(vsec_initialised)
		return true;
	
	
	p_vsec_ctx->token = mv_ipc_mbox_get_info("security", 
						"vsec_tee", 
						&vsec_ops,
						&pshare_mem,
						&(p_vsec_ctx->share_mem_size), 
						&(p_vsec_ctx->cmdline),
						(void *)p_vsec_ctx);

	p_vsec_ctx->status = VSEC_STATUS_DISCONNECT;
	p_vsec_ctx->vsec_share = (struct vsec_share_ctx *)pshare_mem;

	p_vsec_ctx->vsec_share->handshake = mv_gal_os_id();
	
	p_vsec_ctx->vsec_rpc_ctx[0].data = p_vsec_ctx->vsec_share->share_data;
	p_vsec_ctx->vsec_rpc_ctx[0].max_size = (p_vsec_ctx->share_mem_size - sizeof(struct vsec_share_ctx))/2;

	p_vsec_ctx->vsec_rpc_ctx[1].data = p_vsec_ctx->vsec_rpc_ctx[0].data + p_vsec_ctx->vsec_rpc_ctx[0].max_size;
	p_vsec_ctx->vsec_rpc_ctx[1].max_size = p_vsec_ctx->vsec_rpc_ctx[0].max_size;

	
	init_waitqueue_head(&p_vsec_ctx->open_wq);
	mutex_init(&p_vsec_ctx->client_mutex);
	sema_init(&p_vsec_ctx->server_sem, 0);
	sema_init(&p_vsec_ctx->client_wq, 0);
	p_vsec_ctx->is_client_wait = 0;

	p_vsec_ctx->server_index = 1;

	/* Create server thread */
	kthread_run(vsec_server_thread, (void *)p_vsec_ctx, "vsec");
	
	/* Set on line */
	mv_mbox_set_online(p_vsec_ctx->token);
	
	vsec_initialised = 1;
	
	return true;

}

