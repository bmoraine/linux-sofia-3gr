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

#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "sec_rpc.h"
#include "vrpc.h"

enum rpc_action {
	RPC_ALLOC,
	RPC_WRITE,
	RPC_CALL,
	RPC_READ,
	RPC_FREE,
	RPC_ALL,
	RPC_DONE,
	RPC_CMD_END = 0x7FFFFFFF
};

struct t_rpc_param {
	u32 id;     /* Reference to remote allocated buffer */
	u32 buf_len; /* alloc_len */
	u32 length;  /* Length to read or write in buffer */
	u32 offset;  /* Offset from where to read or write in buffer */
	enum t_rpc_if_grp if_grp; /* Which module do you want to dispatch to? */
	u32 opcode;      /* Which remote interface should be invoked? */
};

struct t_rpc_cmd {
	enum rpc_action action; /* The action to perform during this RPC call */
	u32 ctx_id;             /* The running context ID for current RPC call
				   (used for block-based handling) */
	struct t_rpc_param param;/* The params associated with this RPC call */
	u32 tx_size;         /* transfer data len = rpc command + return data */
	int result;             /* The result of this RPC call */
	struct t_rpc_send_info send_info;  /* The sender info during RPC call */
	u8 recv_ack;           /* Acknowledge flag from receiver
				  (so we can release transfer semaphore)*/
};

enum t_rpc_opcode_sec {
	rpc_op_sec_init_start,
	rpc_op_sec_init_done,
	rpc_op_sec_if_end = 0x7FFFFFFF
};

struct t_rpc_ctx {
	struct t_rpc_cmd cmd;   /* Caller defined parameters */
	struct semaphore sem;   /* Blocking semaphore during RPC call */
	u8 *data_out;           /* Where to store output data */
	u32 max_len;            /* Max allocated length */
	struct t_rpc_ctx *next;
};

struct t_rpc_ctx_list {
	u32 nof;
	u32 glob_ctx_id;
	struct t_rpc_ctx *first;
};

struct t_rpc_data {
	u8 *data;
	u32 length;
	u32 id;
	struct t_rpc_data *next;
};

struct t_rpc_data_list {
	u32 nof;
	struct t_rpc_data *first;
};

struct t_rpc_dispatch {
	struct task_struct *task;
	u8 *data;
	struct t_rpc_cmd cmd;
};


static struct t_rpc_ctx_list g_ctx_list;
static struct t_rpc_data_list g_data_list;

DEFINE_SEMAPHORE(ctx_list_sem);
DEFINE_SEMAPHORE(data_list_sem);
DEFINE_SEMAPHORE(shared_mem_mutex);
DEFINE_SEMAPHORE(sem_sec_mutex);
static u32 rpc_alloc_data(struct t_rpc_data_list *g_data_list, u32 length)
{
	struct t_rpc_data *rpc_data_elm;

	/* Allocate new data element */
	rpc_data_elm = kzalloc(sizeof(struct t_rpc_data), GFP_KERNEL);
	if (rpc_data_elm == NULL) {
		pr_err("Failed to allocate new data element (%d bytes)\n",
		       sizeof(struct t_rpc_data));
		return 0;
	}

	/* Allocate data area within current element */
	rpc_data_elm->data = kzalloc(length, GFP_KERNEL);
	if (rpc_data_elm->data == NULL) {
		pr_err("Failed to allocate data area (%d bytes)\n", length);
		kfree(rpc_data_elm);
		return 0;
	}

	/* Set data length and element ID */
	rpc_data_elm->length = length;
	rpc_data_elm->id = g_data_list->nof + 1;

	/* Inject element into data list and update number of elements */
	rpc_data_elm->next = g_data_list->first;
	g_data_list->first = rpc_data_elm;
	g_data_list->nof++;

	return rpc_data_elm->id;
}

static bool rpc_free_data(struct t_rpc_data_list *g_data_list, u32 id)
{
	struct t_rpc_data *rpc_data_elm;
	struct t_rpc_data *prev_rpc_data_elm = NULL;

	rpc_data_elm = g_data_list->first;

	while (rpc_data_elm != NULL) {
		if (rpc_data_elm->id == id) {
			if (prev_rpc_data_elm == NULL)
				g_data_list->first = rpc_data_elm->next;
			else
				prev_rpc_data_elm->next = rpc_data_elm->next;

			g_data_list->nof--;

			kfree(rpc_data_elm->data);
			kfree(rpc_data_elm);

			pr_debug("Cleared data element #%d\n", id);
			return true;
		}

		prev_rpc_data_elm = rpc_data_elm;
		rpc_data_elm = rpc_data_elm->next;
	}

	pr_debug("Data element #%d not found!\n", id);
	return false;
}

static struct t_rpc_data *rpc_get_data(struct t_rpc_data_list *g_data_list,
				       u32 id)
{
	struct t_rpc_data *rpc_data;

	rpc_data = g_data_list->first;

	while (rpc_data != NULL) {
		if (rpc_data->id == id)
			return rpc_data;

		rpc_data = rpc_data->next;
	}

	return NULL;
}


static int rpc_ctx_add(struct t_rpc_ctx *ctx)
{
	/* lock the list */
	if (down_interruptible(&ctx_list_sem)) {
		pr_err("Semaphore aquire interupted\n");
		return -ERESTARTSYS;
	}

	/*Increment and update current context with running context ID */
	g_ctx_list.glob_ctx_id++;
	ctx->cmd.ctx_id = g_ctx_list.glob_ctx_id;

	/*Inject current RPC request as first element in global request list */
	ctx->next = g_ctx_list.first;
	g_ctx_list.first = ctx;

	/* Update number of current contexts in list */
	g_ctx_list.nof++;

	/* release the mutex */
	up(&ctx_list_sem);
	return 0;
}

static struct t_rpc_ctx *rpc_ctx_del(u32 ctx_id)
{
	struct t_rpc_ctx *curr_ctx = NULL;
	struct t_rpc_ctx *prev_ctx = NULL;

	/* lock the list */
	if (down_interruptible(&ctx_list_sem)) {
		pr_err("Semaphore aquire interupted\n");
		return NULL;
	}

	curr_ctx = g_ctx_list.first;

	while (curr_ctx != NULL) {
		if (curr_ctx->cmd.ctx_id == ctx_id) {
			if (prev_ctx)
				prev_ctx->next = curr_ctx->next;
			else
				g_ctx_list.first = curr_ctx->next;

			/* Update number of current contexts in list */
			g_ctx_list.nof--;
			break;
		}

		prev_ctx = curr_ctx;
		curr_ctx = curr_ctx->next;
	}

	/* release the mutex */
	up(&ctx_list_sem);

	return curr_ctx;
}

static int rpc_thread_tx(struct t_rpc_cmd *cmd, u8 *inp_data, u32 inp_data_len,
			 u8 *out_data, u32 max_len)
{
	int result = -1;
	u8 entry_id;
	u8 *shared_mem = NULL;
	struct t_rpc_cmd *shared_cmd = NULL;
	u8 *shared_data = NULL;
	u32 shared_data_size;
	struct t_rpc_ctx *ctx = NULL;

	/* Validate input parameters */
	if (!cmd)
		goto cleanup;

	/* Get the VSEC context ID for this call */
	pr_debug("Look up entry ID based on VLINK: %s, Receiver: VM#%d\n",
		 cmd->send_info.vlink_name, (cmd->action != RPC_DONE) ?
		 cmd->send_info.recv_id : cmd->send_info.send_id);

	entry_id = vsec_get_context_entry_id(cmd->send_info.vlink_name,
					     (cmd->action != RPC_DONE) ?
					     cmd->send_info.recv_id :
					     cmd->send_info.send_id);
	if (entry_id == 0xFF) {
		pr_err("No mathing entry ID found\n");
		goto cleanup;
	}

	pr_debug("Found entry ID: %d\n", entry_id);

	/* Get pointer to shared mem and interpret it as RPC command structure*/
	shared_mem = (u8 *)vsec_get_shared_mem(entry_id);
	if (shared_mem == NULL)
		goto cleanup;

	/* Get pointers to shared rpc command and data + data size */
	shared_cmd = (struct t_rpc_cmd *)&shared_mem[0];
	shared_data = &shared_mem[sizeof(struct t_rpc_cmd)];
	shared_data_size = vsec_get_shared_mem_size(entry_id) -
		sizeof(struct t_rpc_cmd);

	if (cmd->action != RPC_DONE) {
		/* Allocate new RPC context element */
		ctx = kzalloc(sizeof(struct t_rpc_ctx), GFP_KERNEL);
		if (ctx == NULL) {
			pr_err("Out of memory for the ctx\n");
			goto cleanup;
		}

		/* Copy command structure and output information to context */
		memcpy(&ctx->cmd, cmd, sizeof(struct t_rpc_cmd));
		ctx->data_out = out_data;
		ctx->max_len = max_len;

		/* Add current context to global list */
		result = rpc_ctx_add(ctx);
		if (result)
			goto cleanup;

		/* Create already locked semaphore */
		sema_init(&ctx->sem, 1);
		if (down_interruptible(&ctx->sem)) {
			pr_err("Semaphore aquire interupted\n");
			result = -ERESTARTSYS;
			goto cleanup;
		}
	}


	/* lock the list */
	if (down_interruptible(&shared_mem_mutex)) {
		pr_err("Semaphore aquire interupted\n");
		result = -ERESTARTSYS;
		goto cleanup;
	}

	/* Copy command data to shared memory */
	if (cmd->action != RPC_DONE) {
		pr_err("Copy %d bytes from 0x%08X to 0x%08X\n",
		       sizeof(struct t_rpc_cmd), (u32)&ctx->cmd,
		       (u32)shared_cmd);
		memcpy(shared_cmd, &ctx->cmd, sizeof(struct t_rpc_cmd));
	} else {
		pr_err("Copy %d bytes from 0x%08X to 0x%08X\n",
		       sizeof(struct t_rpc_cmd), (u32)cmd, (u32)shared_cmd);
		memcpy(shared_cmd, cmd, sizeof(struct t_rpc_cmd));
	}


	/* Copy input data to shared memory */
	if (inp_data != NULL && inp_data_len < shared_data_size) {
		pr_debug("Transfer input data (%d bytes)\n", inp_data_len);
		memcpy(shared_data, inp_data, inp_data_len);
		shared_cmd->tx_size = inp_data_len;
	}


	result = vsec_call(entry_id);
	/* always unlock the mutex */
	up(&shared_mem_mutex);

	if (result) { /* check if the vsec call failed */
		pr_err("RPC call failed\n");
		goto cleanup;
	}

	if (cmd->action != RPC_DONE && ctx != NULL) {
		pr_debug("Wait for handler to release semaphore\n");

		/* wait for the handler to release the semaphore */
		if (down_interruptible(&ctx->sem)) {
			pr_err("Semaphore aquire interupted\n");
			result = -ERESTARTSYS;
			goto cleanup;
		}

		pr_debug("CTX (0x%08X) @ L:%d\n", (u32)ctx, __LINE__);

		/* Command handler should have updated current RPC context */
		result = ctx->cmd.result;
		memcpy(cmd, &ctx->cmd, sizeof(struct t_rpc_cmd));
	} else {
		result = 0;
	}

	pr_debug("RPC call finished with result: %d\n", result);

cleanup:
	kfree(ctx);
	return result;
}

static int rpc_dispatch_sec(u32 opcode, u8 *io_data, u32 *io_data_len)
{
	int rpc_result = -1;

	if (io_data == NULL || io_data_len == NULL) {
		pr_err("Invalid input : data(0x%08X),", (u32)io_data);
		pr_err("io_data_len(0x%08X)\n", (u32)io_data_len);
		goto cleanup;
	}

	/* Update output length */
	*io_data_len = 0;

	/* Switch on which security interface group/component to interact with*/
	switch (opcode) {
	case rpc_op_sec_init_start:
		up(&sem_sec_mutex);
		break;
	default:
		break;
	}

	rpc_result = 0;

cleanup:
	return rpc_result;
}

static int rpc_dispatch(enum t_rpc_if_grp if_grp, u32 opcode,
			u8 *io_data, u32 *io_data_len)
{
	int rpc_result = 0;

	/*Switch on which security interface group/component to interact with*/
	switch (if_grp) {
	case RPC_IF_SEC:
		rpc_result = rpc_dispatch_sec(opcode, io_data, io_data_len);
		break;
	default:
		break;
	}

	return rpc_result;
}

static int rpc_dispatcher_cmd(void *arg)
{
	struct t_rpc_dispatch *dispatch = (struct t_rpc_dispatch *)arg;
	struct t_rpc_cmd *cmd = (struct t_rpc_cmd *)&dispatch->cmd;
	struct t_rpc_param *param = (struct t_rpc_param *)&cmd->param;
	struct t_rpc_data *rpc_data;
	u8 *return_data = NULL;
	u32 return_data_len = 0;


	pr_debug("ACTION = %d\n", cmd->action);

	/* lock the list */
	if (down_interruptible(&data_list_sem)) {
		pr_err("Semaphore aquire interupted\n");
		return -ERESTARTSYS;
	}

	switch (cmd->action) {
	case RPC_ALLOC:
	{
		param->id = rpc_alloc_data(&g_data_list, param->length);
		if (param->id)
			cmd->result = 0;
		break;
	}
	case RPC_WRITE:
	{
		rpc_data = rpc_get_data(&g_data_list, param->id);
		if (rpc_data != NULL) {
			if ((param->length + param->offset) <=
			    rpc_data->length) {
				memcpy(&rpc_data->data[param->offset],
				       dispatch->data, param->length);
				cmd->result = 0;
			}
		}
		break;
	}
	case RPC_CALL:
	{
		rpc_data = rpc_get_data(&g_data_list, param->id);
		if (rpc_data != NULL)
			cmd->result = rpc_dispatch(param->if_grp,
						   param->opcode,
						   rpc_data->data,
						   &param->length);
		break;
	}
	case RPC_READ:
	{
		rpc_data = rpc_get_data(&g_data_list, param->id);
		if (rpc_data != NULL) {
			if (param->length + param->offset <= rpc_data->length) {
				return_data = &(rpc_data->data[param->offset]);
				return_data_len = param->length;
				cmd->result = 0;
			}
		}
		break;
	}
	case RPC_FREE:
	{
		cmd->result = rpc_free_data(&g_data_list, param->id);
		break;
	}
	case RPC_ALL:
	{
		cmd->result = rpc_dispatch(param->if_grp, param->opcode,
					   dispatch->data, &param->length);

		if (cmd->result == 0 && 0 < param->length &&
		    param->length <= param->buf_len) {
			return_data = dispatch->data;
			return_data_len = param->length;
		}
		break;
	}
	default:
	{
		break;
	}
	} /* end switch */

	up(&data_list_sem);

	cmd->action = RPC_DONE;
	rpc_thread_tx(cmd, return_data, return_data_len, NULL, 0);

	/* Free this thread and its associated data */
	kfree(dispatch->data);
	kfree(dispatch);

	return 0;
}

void rpc_handle_cmd(void *shared_mem)
{
	struct t_rpc_cmd *cmd = (struct t_rpc_cmd *)shared_mem;
	u8 *data = (u8 *)shared_mem + sizeof(struct t_rpc_cmd);
	struct t_rpc_ctx *ctx = NULL;
	struct t_rpc_dispatch *dispatch = NULL;

	pr_debug("ACTION = %d\n", cmd->action);

	switch (cmd->action) {
	case RPC_DONE:
	{
		/* Find already listed context for this RPC call */
		ctx = rpc_ctx_del(cmd->ctx_id);
		if (ctx == NULL)
			break;

		/* Copy entire shared cmd struct back to context list */
		memcpy(&ctx->cmd, cmd, sizeof(struct t_rpc_cmd));

		/* If return data is available, copy specified
		   length from transfer buffer */
		if ((ctx->data_out != NULL) && (0 < cmd->tx_size) &&
		    (cmd->tx_size <= ctx->max_len)) {
			pr_err("Transfer data (%d bytes) to 0x%08X\n",
			       cmd->tx_size, (uint32_t)ctx->data_out);
			memcpy(ctx->data_out, data, cmd->tx_size);
		}

		/* Release caller semaphore */
		pr_err("Release caller sem for RPC context ID#%d\n",
		       ctx->cmd.ctx_id);

		up(&ctx->sem);
		break;
	}
	case RPC_ALLOC:
	case RPC_CALL:
	case RPC_WRITE:
	case RPC_READ:
	case RPC_FREE:
	case RPC_ALL:
	{
		/* Allocate dispatcher information */
		dispatch = kzalloc(sizeof(struct t_rpc_dispatch), GFP_KERNEL);
		if (dispatch == NULL) {
			pr_err("Malloc error @ %d!\n", __LINE__);
			break; /* Handle ERROR properly */
		}

		/* Copy entire command structure to dispatch info */
		memcpy(&dispatch->cmd, cmd, sizeof(struct t_rpc_cmd));

		if (cmd->action == RPC_ALL) {
			/* Allocate buffer for IO data when we do
			   one-shot RPC calls.
			   Note: It is important that sender specifies
			   total buffer size in the command parameter
			   structure! */
			if (cmd->param.buf_len) {
				dispatch->data = kzalloc(cmd->param.buf_len,
							 GFP_KERNEL);
				if (dispatch->data == NULL) {
					pr_err("Malloc error!\n");
					kfree(dispatch);
					dispatch = NULL;
					break;
				}
			}

			/* If input data is available, copy specified
			   length from transfer buffer */
			if (0 < cmd->tx_size &&
			    cmd->tx_size <= cmd->param.buf_len) {
				pr_err("%d bytes: 0x%08X -> 0x%08X\n",
				       cmd->tx_size, (u32)data,
				       (u32)dispatch->data);
				memcpy(dispatch->data, data, cmd->tx_size);
			}
		} else {
			/* If input data is available, copy specified
			   length from transfer buffer */
			if (cmd->tx_size) {
				dispatch->data = kzalloc(cmd->tx_size,
							 GFP_KERNEL);
				if (dispatch->data == NULL) {
					pr_err("Malloc error @ %d!\n",
					       __LINE__);
					kfree(dispatch);
					dispatch = NULL;
					break;
				}
				memcpy(dispatch->data, data, cmd->tx_size);
			}
		}

		pr_debug("dispatch->data = 0x%08X\n", (u32)dispatch->data);
		pr_debug("dispatch->cmd.action = %d\n", dispatch->cmd.action);
		pr_debug("dispatch->cmd.ctx_id = %d\n", dispatch->cmd.ctx_id);
		pr_debug("dispatch->cmd.tx_size = %d\n", dispatch->cmd.tx_size);
		pr_debug("dispatch->cmd.param.id = %d\n",
			 dispatch->cmd.param.id);
		pr_debug("dispatch->cmd.param.buf_len = %d\n",
			 dispatch->cmd.param.buf_len);
		pr_debug("dispatch->cmd.param.length = %d\n",
			 dispatch->cmd.param.length);
		pr_debug("dispatch->cmd.param.if_grp = %d\n",
			 dispatch->cmd.param.if_grp);
		pr_debug("dispatch->cmd.param.opcode = %d\n",
			 dispatch->cmd.param.opcode);

		/* Start RPC dispatcher thread */
		pr_debug("Create thread context.\n");
		dispatch->task = kthread_run(&rpc_dispatcher_cmd,
					     (void *)dispatch,
					     "RPC_THREAD");
		break;
	}
	default:
		break;
	} /* end switch */

	return;
}

int rpc_call(struct t_rpc_send_info *send_info, enum t_rpc_if_grp if_grp,
	     u32 opcode, u8 *io_data, u32 *io_data_len, u32 max_len)
{
	int rpc_result = -1;
	struct t_rpc_cmd *cmd = NULL;
	u32 block_size;
	u8 entry_id;
	u32 curr_size;

	/* Validate input parameters */
	if (io_data == NULL && io_data_len == NULL)
		goto cleanup;

	/* Allocate RPC command */
	cmd = kzalloc(sizeof(struct t_rpc_cmd), GFP_KERNEL);
	if (cmd == NULL)
		goto cleanup;

	memcpy(&cmd->send_info, send_info, sizeof(struct t_rpc_send_info));

	/* Get the VSEC context ID for this call */
	entry_id = vsec_get_context_entry_id(cmd->send_info.vlink_name,
					     cmd->send_info.recv_id);
	if (entry_id == 0xFF) {
		pr_err("No matching entry ID found based on VLINK: %s,",
		       cmd->send_info.vlink_name);
		pr_err(" Receiver: VM#%d\n", cmd->send_info.recv_id);
		goto cleanup;
	}

	pr_err("Found entry ID: %d\n", entry_id);

	/* Get pointer to shared mem and interpret as RPC command structure */
	block_size = vsec_get_shared_mem_size(entry_id) -
		sizeof(struct t_rpc_cmd);

	pr_err("Shared memory block size: %d\n", block_size);

	/* Check if call should be made block based or not */
	if (block_size < max_len) {
		/* ======================================================
		 RPC_ALLOC
		 ====================================================== */
		cmd->action = RPC_ALLOC;
		cmd->param.buf_len = max_len;
		rpc_thread_tx(cmd, NULL, 0, NULL, 0);

		if (cmd->param.id == 0)
			goto cleanup;

		/* Get the data input length */
		curr_size = *io_data_len;

		/* ======================================================
		 RPC_WRITE
		 ====================================================== */
		cmd->action = RPC_WRITE;
		cmd->param.length = block_size;
		cmd->param.offset = 0;

		/* Write input data in block sizes supported by RPC channel */
		while (curr_size > block_size) {
			rpc_result = rpc_thread_tx(cmd, io_data,
						   curr_size, NULL, 0);
			if (rpc_result)
				goto cleanup;

			/* Add one block size to offset */
			cmd->param.offset += block_size;
			/* Subtract block size from current size left to be
			   transferred */
			curr_size -= block_size;
		}

		/* If more data need be written, do one last transfer */
		if (0 < curr_size && curr_size < block_size) {
			cmd->param.length = curr_size;

			rpc_result = rpc_thread_tx(cmd, io_data,
						   curr_size, NULL, 0);
			if (rpc_result)
				goto cleanup;
		}

		/* ======================================================
		 RPC_CALL
		 ====================================================== */
		cmd->action = RPC_CALL;
		cmd->param.length = *io_data_len;
		cmd->param.offset = 0;
		cmd->param.if_grp = if_grp;
		cmd->param.opcode = opcode;

		rpc_result = rpc_thread_tx(cmd, NULL, 0, NULL, 0);
		if (rpc_result)
			goto cleanup;

		/* Get the data output length after invoking interface */
		*io_data_len = cmd->param.length;
		curr_size = *io_data_len;

		/* ======================================================
		 RPC_READ
		 ====================================================== */
		cmd->action = RPC_READ;
		cmd->param.length = block_size;
		cmd->param.offset = 0;

		/* Read output data in block sizes supported by RPC channel */
		while (curr_size > block_size) {
			rpc_result = rpc_thread_tx(cmd, NULL, 0,
						   io_data, curr_size);
			if (rpc_result)
				goto cleanup;

			/* Add one block size to offset */
			cmd->param.offset += block_size;
			/* Subtract block size from current size left to
			   be transferred */
			curr_size -= block_size;
		}

		/* If more data need be read, do one last transfer */
		if (0 < curr_size && curr_size < block_size) {
			cmd->param.length = curr_size;

			rpc_result = rpc_thread_tx(cmd, NULL, 0,
						   io_data, curr_size);
			if (rpc_result)
				goto cleanup;
		}

		/* ======================================================
		 RPC_FREE
		 ====================================================== */
		cmd->action = RPC_FREE;
		rpc_result = rpc_thread_tx(cmd, NULL, 0, NULL, 0);

	} else {

		/* ======================================================
		 RPC_ALL
		 ====================================================== */
		cmd->action = RPC_ALL;
		cmd->param.id = 0;
		cmd->param.buf_len = max_len;
		cmd->param.length = *io_data_len;
		cmd->param.offset = 0;
		cmd->param.if_grp = if_grp;
		cmd->param.opcode = opcode;

		/* Call transfer function (RPC call) */
		rpc_result = rpc_thread_tx(cmd, io_data, *io_data_len,
					   io_data, max_len);
		if (rpc_result) {
			/* Update output length after RPC call */
			*io_data_len = cmd->param.length;
			pr_debug("RPC call success returned buffer length %d\n",
				 *io_data_len);
		}
	}

cleanup:
	kfree(cmd);
	return rpc_result;
}


#ifdef MEM_TEST_STUB

int stub_call_sec_vm(int num_vec, struct pvec *phys_iov, int *remote_ret)
{
	/* This is a hard coded test as a stub for the userspace application */

	if (phys_iov[0].len + phys_iov[1].len <= phys_iov[2].len) {
		memcpy(phys_iov[2].phys_addr, phys_iov[0].phys_addr,
		       phys_iov[0].len);
		memcpy(phys_iov[2].phys_addr + phys_iov[0].len,
		       phys_iov[1].phys_addr, phys_iov[1].len);
	}  else {
		pr_err("Invalid amount of space in out buf to concatinate\n");
	}

	memset(phys_iov[0].phys_addr, 'X', phys_iov[0].len);
	memset(phys_iov[1].phys_addr, 'Y', phys_iov[1].len);

	*remote_ret = 0;
	return 0;
}
#endif
