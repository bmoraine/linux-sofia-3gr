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
#include "sec_shared_mem.h"
#include "rpmb_rpc.h"

#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#define UI16(A) ((u16)(A))
#define UI32(A) ((u32)(A))

#define LIT_UINT16_TO_UCHARS(A, B)\
	{(A)[1] = (B)>>8&0xFF; (A)[0] = (B)&0xFF; }

#define LIT_UCHARS_TO_UINT16(A)\
	(UI16((A)[1])<<8|UI16((A)[0]))

#define LIT_UINT32_TO_UCHARS(A, B)\
	{(A)[3] = (B)>>24&0xFF; (A)[2] = (B)>>16&0xFF; \
	(A)[1] = (B)>>8&0xFF; (A)[0] = (B)&0xFF; }

#define LIT_UCHARS_TO_UINT32(A)\
	(UI32((A)[3])<<24|UI32((A)[2])<<16|UI32((A)[1])<<8|UI32((A)[0]))

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

static const char devname[] =
	"/dev/block/platform/soc0/e0000000.noc/by-name/ImcPartID010";
static struct file *fd_sec_rpc;
static struct t_rpc_ctx_list g_ctx_list;
static struct t_rpc_data_list g_data_list;


#if defined IBFS_TRC
static struct T_IBFS_TRC_BUF trc_buf_a = {0/*init*/};
#endif

DEFINE_SEMAPHORE(ctx_list_sem);
DEFINE_SEMAPHORE(data_list_sem);
DEFINE_SEMAPHORE(shared_mem_mutex);
DEFINE_SEMAPHORE(sem_sec_mutex);
DEFINE_SEMAPHORE(fd_sec_rpc_mutex);

static bool disable_sec_rpc;

static int __init disable_sec_rpc_driver(char *arg)
{
	disable_sec_rpc = true;
	return 0;
}
early_param("nosecvm", disable_sec_rpc_driver);

void *sec_rpc_contig_alloc(u32 sz)
{
	return sec_shared_mem_alloc(sz);
}

void sec_rpc_free(void *ptr)
{
	sec_shared_mem_free(ptr);
}


static u32 rpc_alloc_data(struct t_rpc_data_list *data_list, u32 length)
{
	struct t_rpc_data *rpc_data_elm;

	/* Allocate new data element */
	rpc_data_elm = (struct t_rpc_data *)
		sec_rpc_contig_alloc(sizeof(struct t_rpc_data));
	if (rpc_data_elm == NULL) {
		pr_err("[sec_rpc] rpc_alloc_data: Failed to allocate new data element (%zu bytes)\n",
		       sizeof(struct t_rpc_data));
		return 0;
	}

	/* Allocate data area within current element */
	rpc_data_elm->data = (u8 *)sec_rpc_contig_alloc(length);
	if (rpc_data_elm->data == NULL) {
		pr_err("[sec_rpc] rpc_alloc_data: Failed to allocate data area (%d bytes)\n",
			length);
		sec_rpc_free((void *)rpc_data_elm);
		return 0;
	}

	/* Set data length and element ID */
	rpc_data_elm->length = length;
	rpc_data_elm->id = data_list->nof + 1;

	/* Inject element into data list and update number of elements */
	rpc_data_elm->next = data_list->first;
	data_list->first = rpc_data_elm;
	data_list->nof++;

	return rpc_data_elm->id;
}

static bool rpc_free_data(struct t_rpc_data_list *data_list, u32 id)
{
	struct t_rpc_data *rpc_data_elm;
	struct t_rpc_data *prev_rpc_data_elm = NULL;

	rpc_data_elm = data_list->first;

	while (rpc_data_elm != NULL) {
		if (rpc_data_elm->id == id) {
			if (prev_rpc_data_elm == NULL)
				data_list->first = rpc_data_elm->next;
			else
				prev_rpc_data_elm->next = rpc_data_elm->next;

			data_list->nof--;

			if (rpc_data_elm != NULL) {
				if (rpc_data_elm->data != NULL) {
					sec_rpc_free(
						(void *)rpc_data_elm->data);
				}
				sec_rpc_free((void *)rpc_data_elm);
			}

			pr_debug(
				"[sec_rpc] rpc_free_data: Cleared data element #%d\n",
				id);
			return true;
		}

		prev_rpc_data_elm = rpc_data_elm;
		rpc_data_elm = rpc_data_elm->next;
	}

	pr_debug(
		"[sec_rpc] rpc_free_data: Data element #%d not found!\n",
		id);
	return false;
}

static struct t_rpc_data *rpc_get_data(struct t_rpc_data_list *data_list,
				       u32 id)
{
	struct t_rpc_data *rpc_data;

	rpc_data = data_list->first;

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
		pr_err("[sec_rpc] rpc_ctx_add: Semaphore aquire interupted\n");
		return RPC_FAILURE;
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
	return RPC_SUCCESS;
}

static struct t_rpc_ctx *rpc_ctx_del(u32 ctx_id)
{
	struct t_rpc_ctx *curr_ctx = NULL;
	struct t_rpc_ctx *prev_ctx = NULL;

	/* lock the list */
	if (down_interruptible(&ctx_list_sem)) {
		pr_err("[sec_rpc] rpc_ctx_del: Semaphore aquire interupted\n");
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
	int result = RPC_FAILURE;
	u8 entry_id;
	u8 *shared_mem = NULL;
	struct t_rpc_cmd *shared_cmd = NULL;
	u8 *shared_data = NULL;
	u32 shared_data_size;
	struct t_rpc_ctx *ctx = NULL;

	/* Validate input parameters */
	if (!cmd) {
		pr_err("[sec_rpc] rpc_thread_tx: no input command\n");
		goto cleanup;
	}

	/* Get the VSEC context ID for this call */
	pr_debug("[sec_rpc] rpc_thread_tx: look up entry ID based on VLINK: %s, Receiver: VM#%d\n",
		 cmd->send_info.vlink_name, (cmd->action != RPC_DONE) ?
		 cmd->send_info.recv_id : cmd->send_info.send_id);

	entry_id = vsec_get_context_entry_id(
		cmd->send_info.vlink_name,
		(cmd->action != RPC_DONE) ?
		cmd->send_info.recv_id :
		cmd->send_info.send_id);
	if (entry_id == 0xFF) {
		pr_err("[sec_rpc] rpc_thread_tx: No mathing entry ID found\n");
		goto cleanup;
	}

	pr_debug("[sec_rpc] rpc_thread_tx: Found entry ID: %d\n", entry_id);

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
		ctx = (struct t_rpc_ctx *)
			sec_rpc_contig_alloc(sizeof(struct t_rpc_ctx));
		if (ctx == NULL) {
			pr_err("[sec_rpc] rpc_thread_tx: out of memory for CTX\n");
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
		sema_init(&ctx->sem, 0);
		/*
		if (down_interruptible(&ctx->sem)) {
			pr_err("Semaphore aquire interupted\n");
			result = -ERESTARTSYS;
			goto cleanup;
		}
		*/
	}


	/* lock the list */
	if (down_interruptible(&shared_mem_mutex)) {
		pr_err("[sec_rpc] rpc_thread_tx: semaphore aquire interupted\n");
		result = -ERESTARTSYS;
		goto cleanup;
	}

	/* Copy command data to shared memory */
	if (cmd->action != RPC_DONE) {
		pr_debug("[sec_rpc] rpc_thread_tx: copy %zu bytes from 0x%p to 0x%p\n",
		       sizeof(struct t_rpc_cmd), &ctx->cmd, shared_cmd);
		memcpy(shared_cmd, &ctx->cmd, sizeof(struct t_rpc_cmd));
	} else {
		pr_debug("[sec_rpc] rpc_thread_tx: copy %zu bytes from 0x%p to 0x%p\n",
		       sizeof(struct t_rpc_cmd), cmd, shared_cmd);
		memcpy(shared_cmd, cmd, sizeof(struct t_rpc_cmd));
	}


	/* Copy input data to shared memory */
	if (inp_data != NULL && inp_data_len < shared_data_size) {
		pr_debug("[sec_rpc] rpc_thread_tx: transfer input data (%d bytes)\n",
			inp_data_len);
		memcpy(shared_data, inp_data, inp_data_len);
		shared_cmd->tx_size = inp_data_len;
	}


	result = vsec_call(entry_id);
	/* always unlock the mutex */
	up(&shared_mem_mutex);

	if (result) { /* check if the vsec call failed */
		pr_err("[sec_rpc] rpc_thread_tx: RPC call failed\n");
		goto cleanup;
	}

	if (cmd->action != RPC_DONE && ctx != NULL) {
		pr_debug("[sec_rpc] rpc_thread_tx: wait for handler to release semaphore\n");

		/* wait for the handler to release the semaphore */
		if (down_interruptible(&ctx->sem)) {
			pr_err("[sec_rpc] rpc_thread_tx: semaphore aquire interupted\n");
			result = -ERESTARTSYS;
			goto cleanup;
		}

		pr_debug("[sec_rpc] rpc_thread_tx: CTX (0x%p) @ L:%d\n",
			ctx, __LINE__);

		/* Command handler should have updated current RPC context */
		result = ctx->cmd.result;
		memcpy(cmd, &ctx->cmd, sizeof(struct t_rpc_cmd));
	} else {
		result = RPC_SUCCESS;
	}

	pr_debug("[sec_rpc] rpc_thread_tx: RPC call finished with result: %d\n",
		result);

cleanup:
	if (ctx != NULL)
		sec_rpc_free((void *)ctx);

	return result;
}

static int rpc_dispatch_sec(u32 opcode, u8 *io_data, u32 *io_data_len)
{
	int rpc_result = RPC_FAILURE;

	if (io_data == NULL || io_data_len == NULL) {
		pr_err("[sec_rpc] rpc_dispatch_sec: invalid input : data(0x%p),",
			io_data);
		pr_err("[sec_rpc] rpc_dispatch_sec: io_data_len(0x%p)\n",
			io_data_len);
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

	rpc_result = RPC_SUCCESS;

cleanup:
	return rpc_result;
}


#if defined IBFS_TRC
static void rpc_trace(enum T_IBFS_TRC_TYPE trc_type, u32 val0, u32 val1)
{
	if (!trc_buf_a.init) {
		memset(&trc_buf_a, 0, sizeof(struct T_IBFS_TRC_BUF));
		trc_buf_a.init = true;
	}

	trc_buf_a.buf[trc_buf_a.next_item].trc_type  = trc_type;
	trc_buf_a.buf[trc_buf_a.next_item].val0      = val0;
	trc_buf_a.buf[trc_buf_a.next_item].val1      = val1;

	if (trc_buf_a.next_item >= IBFS_TRC_BUF_LGT-1)
		trc_buf_a.next_item = 0;
	else
		trc_buf_a.next_item++;
}

static void rpc_trace_get(struct T_IBFS_TRC_BUF *p_trc_buf,
	u32 *p_nof_bytes_buf)
{
	*p_nof_bytes_buf = sizeof(struct T_IBFS_TRC_BUF);
	memcpy(p_trc_buf, &trc_buf_a, sizeof(struct T_IBFS_TRC_BUF));
}
#else
#define rpc_trace(trc_type, val0, val1)
#define rpc_trace_get(p_trc_buf, p_nof_bytes_buf)
#endif


static int rpc_write_ibfs(u8 *p_buf, u32 offset, u32 nof_bytes)
{
	int rpc_result = RPC_FAILURE;
	loff_t pos;
	ssize_t cnt;
	mm_segment_t old_fs;

	/* Lock fd */
	if (down_interruptible(&fd_sec_rpc_mutex)) {
		pr_err("Semaphore aquire interupted\n");
		return RPC_FAILURE;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = vfs_llseek(fd_sec_rpc, offset, SEEK_SET);

	if (pos < 0) {
		pr_err("[sec_rpc] rpc_write_ibfs: Seek %d failed\n", offset);
		goto cleanup;
	}

	if (pos != offset) {
		pr_err("[sec_rpc] rpc_write_ibfs: Seek %d Got %d\n",
			offset, (u32)pos);
		goto cleanup;
	}

	cnt = vfs_write(fd_sec_rpc, p_buf, nof_bytes, &pos);
	set_fs(old_fs);

	/* Release fd lock */
	up(&fd_sec_rpc_mutex);

	if (nof_bytes != cnt) {
		pr_err("[sec_rpc] rpc_write_ibfs: Wrt %d Got %d\n",
			nof_bytes, (u32)cnt);

		goto cleanup;
	}

	rpc_result = RPC_SUCCESS;

cleanup:
	return rpc_result;
}

static int rpc_dispatch_ibfs(u32 opcode, u8 *io_data, u32 *io_data_len)
{
	int rpc_result = RPC_FAILURE;
	loff_t pos;
	u32 offset;
	u32 nof_bytes;
	ssize_t cnt;
	mm_segment_t old_fs;
	u8 *p_buf = NULL;
	phys_addr_t p_buf_phys = 0;

	if (io_data == NULL || io_data_len == NULL) {
		pr_err("[sec_rpc] rpc_dispatch_ibfs: invalid input : data(0x%08X),",
					(u32)io_data);
		pr_err("[sec_rpc] rpc_dispatch_ibfs: io_data_len(0x%08X)\n",
		      (u32)io_data_len);
		goto cleanup;
	}

	/* Switch on which ibfs interface group/component to interact with*/
	switch (opcode) {
	/*********************** rpc_op_ibfs_open ***********************/
	case rpc_op_ibfs_open:
		/* Lock fd */
		if (down_interruptible(&fd_sec_rpc_mutex)) {
			pr_err("Semaphore aquire interupted\n");
			return -ERESTARTSYS;
		}
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fd_sec_rpc = filp_open(devname, (O_RDWR | O_SYNC), 0);

		if (unlikely(IS_ERR(fd_sec_rpc))) {
			pr_err("[sec_rpc] rpc_dispatch_ibfs->rpc_op_ibfs_open: Failed %s\n",
				devname);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1000);
			/* Release fd lock */
			up(&fd_sec_rpc_mutex);
			goto cleanup;
		}

		pos = vfs_llseek(fd_sec_rpc, 0, SEEK_END);
		set_fs(old_fs);

		/* Release fd lock */
		up(&fd_sec_rpc_mutex);

		if (pos < 0) {
			pr_err("[sec_rpc] rpc_dispatch_ibfs->rpc_op_ibfs_open: Seek end failed\n");
			goto cleanup;
		}

		if (pos == 0) {
			pr_err("[sec_rpc] rpc_dispatch_ibfs->rpc_op_ibfs_open: Empty %s\n",
				devname);
			goto cleanup;
		}

		/* Update output length */
		*io_data_len = 2*sizeof(u32);

		/* Return length of partition */
		LIT_UINT32_TO_UCHARS(&io_data[4], pos);

		/* Return result */
		LIT_UINT32_TO_UCHARS(&io_data[0], IBFS_OK);

		rpc_trace(IBFS_TRC_INIT, 0, pos);
		break;

	/*********************** rpc_op_ibfs_close ***********************/
	case rpc_op_ibfs_close:
		/* Lock fd */
		if (down_interruptible(&fd_sec_rpc_mutex)) {
			pr_err("Semaphore aquire interupted\n");
			return -ERESTARTSYS;
		}
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		filp_close(fd_sec_rpc, NULL);
		set_fs(old_fs);

		/* Release fd lock */
		up(&fd_sec_rpc_mutex);

		/* Update output length */
		*io_data_len = sizeof(u32);

		/* Return result */
		LIT_UINT32_TO_UCHARS(&io_data[0], IBFS_OK);

		rpc_trace(IBFS_TRC_CLOSE, 0, 0);
		break;

	/*********************** rpc_op_ibfs_read ***********************/
	case rpc_op_ibfs_read:
		offset = LIT_UCHARS_TO_UINT32(&(io_data[0]));
		nof_bytes = LIT_UCHARS_TO_UINT32(&(io_data[4]));
		p_buf = phys_to_virt((phys_addr_t)
			LIT_UCHARS_TO_UINT32(&(io_data[8])));

		if ((nof_bytes == 0) || p_buf == NULL || ((phys_addr_t)
				LIT_UCHARS_TO_UINT32(&(io_data[8])) == 0)) {
			pr_err("[sec_rpc] rpc_op_ibfs_read: parameters invalid\n");
			goto cleanup;
		}

		/* Lock fd */
		if (down_interruptible(&fd_sec_rpc_mutex)) {
			pr_err("Semaphore aquire interupted\n");
			return -ERESTARTSYS;
		}
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		pos = vfs_llseek(fd_sec_rpc, offset, SEEK_SET);
		if (pos < 0) {
			pr_err("[sec_rpc] rpc_dispatch_ibfs->rpc_op_ibfs_read: Failed %d\n",
				offset);
			goto cleanup;
		}

		if (pos != offset) {
			pr_err("[sec_rpc] rpc_dispatch_ibfs->rpc_op_ibfs_read: Seek %d Got %d\n",
				offset, (u32)pos);
			goto cleanup;
		}

		cnt = vfs_read(fd_sec_rpc, p_buf, nof_bytes, &pos);
		set_fs(old_fs);

		/* Release fd lock */
		up(&fd_sec_rpc_mutex);

		if (nof_bytes != cnt) {
			pr_err("[sec_rpc] rpc_dispatch_ibfs->rpc_op_ibfs_read: Read %d. Got %d\n",
				nof_bytes, (u32)cnt);
			goto cleanup;
		}

		/* Update output length */
		*io_data_len = sizeof(u32);

		/* Return result */
		LIT_UINT32_TO_UCHARS(&io_data[0], IBFS_OK);

		rpc_trace(IBFS_TRC_READ, offset, nof_bytes);
		break;

	/*********************** rpc_op_ibfs_write ***********************/
	case rpc_op_ibfs_write:
		offset = LIT_UCHARS_TO_UINT32(&(io_data[0]));
		nof_bytes = LIT_UCHARS_TO_UINT32(&(io_data[4]));
		p_buf = phys_to_virt(
			(phys_addr_t)LIT_UCHARS_TO_UINT32(&(io_data[8])));

		if ((nof_bytes == 0) || ((phys_addr_t)
				LIT_UCHARS_TO_UINT32(&(io_data[8])) == 0)) {
			pr_err("[sec_rpc] rpc_op_ibfs_write: parameters invalid\n");
			goto cleanup;
		}

		rpc_result = rpc_write_ibfs(p_buf, offset, nof_bytes);
		if (0 != rpc_result) {
			pr_err("[sec_rpc] rpc_dispatch_ibfs->rpc_op_ibfs_write: Write failed\n");
			goto cleanup;
		}

		/* Update output length */
		*io_data_len = sizeof(u32);

		/* Return result */
		LIT_UINT32_TO_UCHARS(&io_data[0], IBFS_OK);

		rpc_trace(IBFS_TRC_WRITE, offset, nof_bytes);
		break;

	/*********************** rpc_op_ibfs_erase ***********************/
	case rpc_op_ibfs_erase:
		offset    = LIT_UCHARS_TO_UINT32(&(io_data[0]));
		nof_bytes = LIT_UCHARS_TO_UINT32(&(io_data[4]));

		if (nof_bytes == 0) {
			pr_err("[sec_rpc] rpc_op_ibfs_erase: parameters invalid\n");
			goto cleanup;
		}
		p_buf = (u8 *)sec_rpc_contig_alloc(nof_bytes);
		if (NULL == p_buf) {
			pr_err("[sec_rpc] rpc_op_ibfs_erase: memory alloc failed for %d bytes\n",
				nof_bytes);
			goto cleanup;
		}

		memset(p_buf, 0xFF, nof_bytes);

		rpc_result = rpc_write_ibfs(p_buf, offset, nof_bytes);
		if (RPC_SUCCESS != rpc_result) {
			sec_rpc_free((void *)p_buf);
			pr_err("[sec_rpc] rpc_dispatch_ibfs->rpc_op_ibfs_erase: Write failed\n");
			goto cleanup;
		} else {
			sec_rpc_free((void *)p_buf);
		}

		/* Update output length */
		*io_data_len = sizeof(u32);

		/* Return result */
		LIT_UINT32_TO_UCHARS(&io_data[0], IBFS_OK);

		rpc_trace(IBFS_TRC_ERASE, offset, nof_bytes);
		break;

	/*********************** rpc_op_ibfs_alloc ***********************/
	case rpc_op_ibfs_alloc:
		nof_bytes = LIT_UCHARS_TO_UINT32(&(io_data[0]));

		if (nof_bytes == 0) {
			pr_err("[sec_rpc] rpc_op_ibfs_alloc: parameters invalid\n");
			goto cleanup;
		}
		p_buf = (u8 *)sec_rpc_contig_alloc(nof_bytes);
		if (NULL == p_buf) {
			pr_err("[sec_rpc] rpc_dispatch_ibfs->rpc_op_ibfs_alloc: memory alloc of %d bytes failed\n",
				nof_bytes);
			goto cleanup;
		}

		p_buf_phys = virt_to_phys(p_buf);

		if (p_buf_phys == 0) {
			pr_err(
			"[sec_rpc] rpc_op_ibfs_alloc: conversion from virt to phys failed!\n");
			sec_rpc_free((void *)p_buf);
			goto cleanup;
		}

		/* Update output length */
		*io_data_len = 2*sizeof(u32);

		/* Return pointer */
		LIT_UINT32_TO_UCHARS(&io_data[4], p_buf_phys);

		/* Return result */
		LIT_UINT32_TO_UCHARS(&io_data[0], IBFS_OK);

		rpc_trace(IBFS_TRC_ALLOC, p_buf_phys, nof_bytes);
		break;

	/*********************** rpc_op_ibfs_free ***********************/
	case rpc_op_ibfs_free:
		p_buf_phys = (phys_addr_t)
			LIT_UCHARS_TO_UINT32(&(io_data[0]));

		if (((phys_addr_t)
			  LIT_UCHARS_TO_UINT32(&(io_data[0]))) == 0) {
			pr_err(
				"[sec_rpc] rpc_op_ibfs_free: physical address is 0\n");
			goto cleanup;
		}

		p_buf = phys_to_virt(p_buf_phys);

		if (NULL == p_buf) {
			pr_err("[sec_rpc] rpc_dispatch_ibfs->rpc_op_ibfs_free: buf already freed\n");
			goto cleanup;
		}

		sec_rpc_free((void *)p_buf);

		/* Update output length */
		*io_data_len = sizeof(u32);

		/* Return result */
		LIT_UINT32_TO_UCHARS(&io_data[0], IBFS_OK);

		rpc_trace(IBFS_TRC_FREE, p_buf_phys, 0);
		break;

#if defined IBFS_TRC
	/*********************** rpc_op_ibfs_trace ***********************/
	case rpc_op_ibfs_trace:
		/* Update output length */
		nof_bytes = 0;

		rpc_trace_get((struct T_IBFS_TRC_BUF *)&io_data[4], &nof_bytes);

		*io_data_len = sizeof(u32) + nof_bytes;

		/* Return result */
		LIT_UINT32_TO_UCHARS(&io_data[0], IBFS_OK);
		break;
#endif

	default:
		pr_err("[sec_rpc] rpc_dispatch_ibfs: unknown opcode\n");
		break;
	}

	rpc_result = RPC_SUCCESS;

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
	case RPC_IF_RPMB:
		rpc_result =
			rpc_rpmb_dispatch(opcode, io_data, io_data_len);
		pr_debug("rpc_dispatch_rpmb\n");
		break;
	case RPC_IF_IBFS:
		rpc_result = rpc_dispatch_ibfs(opcode, io_data, io_data_len);
		pr_debug("[sec_rpc] rpc_dispatch_ibfs: opcode %d / result %d\n",
			opcode, rpc_result);
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
	u32 result = 0;

	pr_debug("[sec_rpc] rpc_dispatcher_cmd: ACTION = %d\n", cmd->action);

	/* lock the list */
	if (down_interruptible(&data_list_sem)) {
		pr_err("[sec_rpc] rpc_dispatcher_cmd: Semaphore aquire interupted\n");
		return -ERESTARTSYS;
	}

	switch (cmd->action) {
	case RPC_ALLOC:
	{
		param->id = rpc_alloc_data(&g_data_list, param->length);
		if (param->id)
			cmd->result = RPC_SUCCESS;
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
				cmd->result = RPC_SUCCESS;
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
				cmd->result = RPC_SUCCESS;
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

		if (cmd->result == RPC_SUCCESS && 0 < param->length &&
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

	if (cmd->result != RPC_SUCCESS) {
		pr_err("[sec_rpc] rpc_dispatcher_cmd: RPC cmd %d failed with result %d\n",
			cmd->action, cmd->result);
		cmd->result = RPC_FAILURE;
		result = -1;
	}

	cmd->action = RPC_DONE;
	result =
		(rpc_thread_tx(cmd, return_data, return_data_len, NULL, 0) ==
			RPC_SUCCESS ? 0 : -1);

	/* Free this thread and its associated data */
	if (dispatch != NULL) {
		if (dispatch->data != NULL)
			sec_rpc_free((void *)dispatch->data);
		sec_rpc_free((void *)dispatch);
	}

	return result;
}

void rpc_handle_cmd(void *shared_mem)
{
	struct t_rpc_cmd *cmd = (struct t_rpc_cmd *)shared_mem;
	u8 *data = (u8 *)shared_mem + sizeof(struct t_rpc_cmd);
	struct t_rpc_ctx *ctx = NULL;
	struct t_rpc_dispatch *dispatch = NULL;

	pr_debug("[sec_rpc] rpc_handle_cmd: ACTION = %d\n", cmd->action);

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
		    (cmd->tx_size <= ctx->max_len) &&
		    (RPC_SUCCESS == cmd->result)) {
			pr_debug("[sec_rpc] rpc_handle_cmd: Transfer data (%d bytes) to 0x%p\n",
			       cmd->tx_size, ctx->data_out);
			memcpy(ctx->data_out, data, cmd->tx_size);
		}

		/* Release caller semaphore */
		pr_err("[sec_rpc] rpc_handle_cmd: Release caller sem for RPC context ID#%d\n",
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
		dispatch = (struct t_rpc_dispatch *)
		    sec_rpc_contig_alloc(sizeof(struct t_rpc_dispatch));
		if (dispatch == NULL) {
			pr_err("[sec_rpc] rpc_handle_cmd: Malloc error @ %d!\n",
				__LINE__);
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
				dispatch->data = (u8 *)sec_rpc_contig_alloc(
					cmd->param.buf_len);
				if (dispatch->data == NULL) {
					pr_err("[sec_rpc] rpc_handle_cmd: no memory for dispatch->data\n");
					sec_rpc_free((void *)dispatch);
					break;
				}
			}

			/* If input data is available, copy specified
				length from transfer buffer */
			if (0 < cmd->tx_size &&
			    cmd->tx_size <= cmd->param.buf_len) {
				pr_debug("[sec_rpc] rpc_handle_cmd: cmd->action %d with %d bytes: 0x%p -> 0x%p\n",
					cmd->action,
					cmd->tx_size, data,
					dispatch->data);
				memcpy(dispatch->data, data, cmd->tx_size);
			}
		} else {
			/* If input data is available, copy specified
			   length from transfer buffer */
			if (cmd->tx_size) {
				dispatch->data =
					(u8 *)sec_rpc_contig_alloc(
					cmd->tx_size);
				if (dispatch->data == NULL) {
					pr_err("[sec_rpc] rpc_handle_cmd: Malloc error @ %d!\n",
						__LINE__);
					sec_rpc_free((void *)dispatch);
					break;
				}
				memcpy(dispatch->data, data, cmd->tx_size);
			}
		}

		pr_debug("[sec_rpc] rpc_handle_cmd: dispatch->data = 0x%p\n",
			dispatch->data);
		pr_debug("[sec_rpc] rpc_handle_cmd: dispatch->cmd.action = %d\n",
			dispatch->cmd.action);
		pr_debug("[sec_rpc] rpc_handle_cmd: dispatch->cmd.ctx_id = %d\n",
			dispatch->cmd.ctx_id);
		pr_debug("[sec_rpc] rpc_handle_cmd: dispatch->cmd.tx_size = %d\n",
			dispatch->cmd.tx_size);
		pr_debug("[sec_rpc] rpc_handle_cmd: dispatch->cmd.param.id = %d\n",
			dispatch->cmd.param.id);
		pr_debug("[sec_rpc] rpc_handle_cmd: dispatch->cmd.param.buf_len = %d\n",
			 dispatch->cmd.param.buf_len);
		pr_debug("[sec_rpc] rpc_handle_cmd: dispatch->cmd.param.length = %d\n",
			 dispatch->cmd.param.length);
		pr_debug("[sec_rpc] rpc_handle_cmd: dispatch->cmd.param.if_grp = %d\n",
			 dispatch->cmd.param.if_grp);
		pr_debug("[sec_rpc] rpc_handle_cmd: dispatch->cmd.param.opcode = %d\n",
			 dispatch->cmd.param.opcode);

		/* Start RPC dispatcher thread */
		pr_debug("[sec_rpc] rpc_handle_cmd: Create thread context.\n");
		kthread_run(&rpc_dispatcher_cmd,
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
	int rpc_result = RPC_FAILURE;
	struct t_rpc_cmd *cmd = NULL;
	u32 block_size;
	u8 entry_id;
	u32 curr_size;

	if (disable_sec_rpc) {
		pr_err("sec_rpc disabled, call rejected\n");
		return rpc_result;
	}

	/* Validate input parameters */
	if (io_data == NULL && io_data_len == NULL)
		goto cleanup;

	/* Allocate RPC command */
	cmd = (struct t_rpc_cmd *)sec_rpc_contig_alloc(
		sizeof(struct t_rpc_cmd));
	if (cmd == NULL) {
		pr_err("[sec_rpc (linux] rpc_call: no memory for cmd\n");
		goto cleanup;
	}

	memcpy(&cmd->send_info, send_info, sizeof(struct t_rpc_send_info));

	/* Get the VSEC context ID for this call */
	entry_id = vsec_get_context_entry_id(cmd->send_info.vlink_name,
					     cmd->send_info.recv_id);
	if (entry_id == 0xFF) {
		pr_err(
		  "No matching entry ID found based on VLINK: %s Receiver: VM#%d and sending as %d\n",
			cmd->send_info.vlink_name,
			cmd->send_info.recv_id,
			cmd->send_info.send_id);
		goto cleanup;
	}

	pr_err("[sec_rpc] rpc_call: Found entry ID: %d\n", entry_id);

	/* Get pointer to shared mem and interpret as RPC command structure */
	block_size = vsec_get_shared_mem_size(entry_id) -
		sizeof(struct t_rpc_cmd);

	pr_err("[sec_rpc] rpc_call: Shared memory block size: %d\n",
		block_size);

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
			pr_debug("[sec_rpc] rpc_call: RPC call success returned buffer length %d\n",
				 *io_data_len);
		}
	}

cleanup:
	if (cmd != NULL)
		sec_rpc_free((void *)cmd);

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
		pr_err("[sec_rpc] stub_call_sec_vm: Invalid amount of space in out buf to concatinate\n");
	}

	memset(phys_iov[0].phys_addr, 'X', phys_iov[0].len);
	memset(phys_iov[1].phys_addr, 'Y', phys_iov[1].len);

	*remote_ret = 0;
	return 0;
}
#endif
