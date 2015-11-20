/*
 ****************************************************************
 *
 *
 *  Copyright (C) 2012 - 2013 Intel Mobile Communications GmbH
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <sofia/mv_gal.h>
#include <sofia/mv_ipc.h>

#include "vnvm_common.h"

static LIST_HEAD(queue);
static DECLARE_COMPLETION(setup_done);
static DECLARE_COMPLETION(req_done);
static DEFINE_SPINLOCK(req_lock);
static struct task_struct *thread;
static DECLARE_WAIT_QUEUE_HEAD(vnvm_waitqueue);

#define VNVM_TRACE(x) pr_debug(x)

enum vnvm_msg {
	VNVM_MSG_CONNECT,
	VNVM_MSG_DISCONNECT,
	VNVM_MSG_RPC_CLIENT,
	VNVM_MSG_RPC_SERVER,
	VNVM_MSG_LAST = VNVM_MSG_RPC_SERVER,
};
#define VNVM_MAGIC_REQ 0xDEADBEEF
struct vnvm_msg_req {
	struct list_head node;
	enum vnvm_msg msg;
	unsigned magic;
};

static inline char *vnvm_msg_to_str(enum vnvm_msg msg)
{
	switch (msg) {
	case VNVM_MSG_CONNECT:
		return "NVM CONNECT";
	case VNVM_MSG_DISCONNECT:
		return "NVM DISCONNECT";
	case VNVM_MSG_RPC_CLIENT:
		return "NVM RPC CLIENT";
	case VNVM_MSG_RPC_SERVER:
		return "NVM RPC SERVER";
	default:
		return "INVALID";

	}
}
static void vnvm_queue_msg(enum vnvm_msg msg)
{
	struct vnvm_msg_req *req;
	unsigned long flags;

	BUG_ON(msg > VNVM_MSG_LAST);
	req = kzalloc(sizeof(struct vnvm_msg_req), GFP_ATOMIC);
	BUG_ON(req == NULL);
	req->msg = msg;
	req->magic = VNVM_MAGIC_REQ;
	pr_debug("%s:%d:--> message %s(%x)\n",
			__func__, __LINE__, vnvm_msg_to_str(msg), msg);
	spin_lock_irqsave(&req_lock, flags);
	list_add_tail(&req->node, &queue);
	spin_unlock_irqrestore(&req_lock, flags);

	wake_up_interruptible(&vnvm_waitqueue);
}

enum vnvm_rpc_event {
	VNVM_RPC_EVENT_CLIENT = 0,
	VNVM_RPC_EVENT_SERVER,
};

struct vnvm_rpc {
	uint32_t max_size;
	uint8_t *data;
};

struct vnvm_share_ctx {
	uint32_t handshake;
	uint32_t server_status;
	uint8_t share_data[0];
};

struct vnvm_ctx {
	uint32_t token;
	uint32_t share_mem_size;
	struct vnvm_rpc vnvm_rpc_ctx[2];
	char *cmdline;
	uint32_t server_index;
	uint32_t client_index;
	char devname[NVM_MAX_NB_PART][VNVM_DEV_NAME_LIMIT];
	uint32_t devid;
	uint32_t devsize[NVM_MAX_NB_PART];
	struct file *fd[NVM_MAX_NB_PART];
	struct vnvm_share_ctx *vnvm_share;
	uint32_t nb_part;
	const char *part_name[NVM_MAX_NB_PART];
};

static struct vnvm_ctx vnvm_ctx;

static struct wake_lock vnvm_be_suspend_lock;

static int vnvm_init_device(uint32_t is_retry)
{
	signed diag;
	loff_t pos;
	struct file *fd;
	mm_segment_t old_fs = get_fs();
	struct vnvm_ctx *be = &vnvm_ctx;
	unsigned i;

	for (i = 0; i < be->nb_part; i++) {
		set_fs(KERNEL_DS);
		fd = filp_open(be->devname[i], (O_RDWR | O_SYNC), 0);
		set_fs(old_fs);
		if (unlikely(IS_ERR(fd))) {
			/* Better use system events to get
			 * notified on device removal/creation */
			pr_err("open device %s failed, retrying\n",
					be->devname[i]);
			BUG();
		}

		be->fd[i] = fd;
		pos = vfs_llseek(fd, 0, SEEK_END);
		if (pos < 0) {
			pr_err("Seek to end of device failed\n");
			diag = -ESPIPE;
			goto dev_error;
		}

		if (pos == 0) {
			pr_err("device %s size 0 bytes\n", be->devname[i]);
			diag = -ESPIPE;
			goto dev_error;
		}

		be->devsize[i] = pos;
		pr_debug("opened device %s size %lld bytes\n",
				be->devname[i], pos);
	}

	return 0;

dev_error:
	/* No retry */
	pr_err("Not able to initialize NVM\n");
	BUG();

	return diag;
}

static int vvfs_device_write(loff_t offset, char *buf,
					size_t count, uint32_t part_id)
{
	ssize_t cnt;
	loff_t pos;
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;
	mm_segment_t old_fs = get_fs();

	pos = vfs_llseek(p_vnvm_ctx->fd[part_id], offset, SEEK_SET);
	if (pos < 0)
		return 0;
	set_fs(KERNEL_DS);
	cnt = vfs_write(p_vnvm_ctx->fd[part_id], buf, count, &pos);
	set_fs(old_fs);
	return cnt;
}

static int vvfs_device_read(loff_t offset, char *buf,
					size_t count, uint32_t part_id)
{
	ssize_t cnt;
	loff_t pos;
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;
	mm_segment_t old_fs = get_fs();
	pos = vfs_llseek(p_vnvm_ctx->fd[part_id], offset, SEEK_SET);
	if (pos < 0)
		return 0;
	set_fs(KERNEL_DS);
	cnt = vfs_read(p_vnvm_ctx->fd[part_id], buf, count, &pos);
	set_fs(old_fs);
	return cnt;
}

static unsigned vnvm_handle_fe_message(struct vnvm_ctx *p_vnvm_ctx,
							unsigned req_op)
{
	void *message;
	nvm_request *req;
	nvm_reply *reply;
	unsigned data_index;
	unsigned i;

	BUG_ON(p_vnvm_ctx == NULL);
	pr_debug("%s: Handle FE message\n", __func__);
	data_index = p_vnvm_ctx->client_index;
	message = p_vnvm_ctx->vnvm_rpc_ctx[data_index].data;

	BUG_ON(message == NULL);

	req = (nvm_request *)message;
	reply = (nvm_reply *)message;

	switch (req_op) {
	case VNVM_OP_DEVICE_INFO:
		vnvm_init_device(1);
		req->op = VNVM_OP_DEVICE_INFO;
		req->addr = p_vnvm_ctx->devid;
		for (i = 0; i < p_vnvm_ctx->nb_part; i++)
			req->devsize[i] = p_vnvm_ctx->devsize[i];
		break;
	default:
		BUG();
	}
	pr_debug("%s: Sending FE message %d\n", __func__, req_op);
	mv_ipc_mbox_post(p_vnvm_ctx->token, VNVM_RPC_EVENT_CLIENT);
	wait_for_completion_interruptible(&req_done);

	pr_debug("%s: FE message %d completed\n", __func__, req_op);

	return reply->retcode;
}

static void vnvm_handle_be_message(struct vnvm_ctx *p_vnvm_ctx)
{
	void *message;
	nvm_request *req;
	nvm_reply *reply;
	unsigned data_index;
	ssize_t cnt;

	BUG_ON(p_vnvm_ctx == NULL);

	data_index = p_vnvm_ctx->server_index;
	message = p_vnvm_ctx->vnvm_rpc_ctx[data_index].data;

	BUG_ON(message == NULL);

	req = (nvm_request *)message;
	reply = (nvm_reply *)message;

	switch (req->op) {
	case VNVM_OP_WRITE_DATA: {
		pr_debug("Write request: %x@%x, PartId = %d",
				req->size, req->addr, req->part_id);
		cnt = vvfs_device_write(req->addr, (char *)req->data, req->size,
				req->part_id - 0x16);
		pr_debug("Write Request: Count =%d", cnt);
		reply->retcode = cnt;
		break;
	}
	case VNVM_OP_READ_DATA: {
		ssize_t cnt;
		pr_debug("Read request: %x@%x, PartId = %d",
				req->size, req->addr, req->part_id);
		cnt = vvfs_device_read(req->addr, (char *)req->data, req->size,
				req->part_id - 0x16);
		pr_debug("Read Request: Count =%d", cnt);
		reply->retcode = cnt;
		break;
	}

	default: {
		panic("unknown nvm request!\n");
		break;
	}
	}

	pr_debug("%s: Sending BE message %d\n", __func__, req->op);
	mv_ipc_mbox_post(p_vnvm_ctx->token, VNVM_RPC_EVENT_SERVER);
	wake_unlock(&vnvm_be_suspend_lock);
}

static void vnvm_close(void)
{
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;
	mm_segment_t old_fs;
	unsigned i;

	for (i = 0; i < p_vnvm_ctx->nb_part; i++) {
		if (p_vnvm_ctx->fd[i]) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			filp_close(p_vnvm_ctx->fd[i], NULL);
			set_fs(old_fs);
			p_vnvm_ctx->fd[i] = NULL;
			p_vnvm_ctx->devsize[i] = 0;
		}
	}
}

static signed vnvm_probe_devid(void)
{
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;
	unsigned i;

	p_vnvm_ctx->devid = 0xB303;

	for (i = 0; i < p_vnvm_ctx->nb_part; i++)
		snprintf(p_vnvm_ctx->devname[i], VNVM_DEV_NAME_LIMIT,
			p_vnvm_ctx->part_name[i]);

	if (p_vnvm_ctx->devid == 0) {
		pr_err("Could not proceed further as no device configured\n");
		return -ENODEV;
	}
	for (i = 0; i < p_vnvm_ctx->nb_part; i++) {
		pr_info("device id (%u,%u) name %s\n",
				VNVM_DEVID_MAJOR(p_vnvm_ctx->devid),
				VNVM_DEVID_MINOR(p_vnvm_ctx->devid),
				p_vnvm_ctx->devname[i]);
	}

	return 0;
}

static void vnvm_on_connect(uint32_t token, void *cookie)
{
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;

	/* Do handshake */
	if (p_vnvm_ctx->vnvm_share->handshake == mv_gal_os_id()) {
		p_vnvm_ctx->server_index = 1;
		p_vnvm_ctx->client_index = 0;
	} else {
		p_vnvm_ctx->server_index = 0;
		p_vnvm_ctx->client_index = 1;
	}

	vnvm_queue_msg(VNVM_MSG_CONNECT);
}

static void vnvm_on_disconnect(uint32_t token, void *cookie)
{
	vnvm_queue_msg(VNVM_MSG_DISCONNECT);
}

static void vnvm_on_event(uint32_t token, uint32_t event_id, void *cookie)
{
	switch (event_id) {
	case VNVM_RPC_EVENT_CLIENT:
		wake_lock(&vnvm_be_suspend_lock);
		vnvm_queue_msg(VNVM_MSG_RPC_CLIENT);
		break;
	case VNVM_RPC_EVENT_SERVER:
		complete(&req_done);
		break;
	default:
		break;
	}
}

static struct mbox_ops vnvm_ops = {
	.on_connect    = vnvm_on_connect,
	.on_disconnect = vnvm_on_disconnect,
	.on_event      = vnvm_on_event
};

/* device tree parsing */
#define INTEL_NVM "intel,vnvm"
#define INTEL_NVM_PARTITION_NB "intel,nb-part"
#define INTEL_NVM_PARTITION_NAME "intel,part-names"
static int of_vnvm_parse(void)
{
	struct device_node *dn;
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;
	int i;
	dn = of_find_compatible_node(NULL, NULL, INTEL_NVM);
	if (!dn)
		return -EINVAL;

	if (of_property_read_u32(dn, INTEL_NVM_PARTITION_NB,
			&p_vnvm_ctx->nb_part)) {
		p_vnvm_ctx->nb_part = 0;
		pr_err("no partition defined...\n");
		return -EINVAL;
	} else {
		for (i = 0; i < p_vnvm_ctx->nb_part; i++) {
			if (of_property_read_string_index(dn,
				INTEL_NVM_PARTITION_NAME, i,
				&p_vnvm_ctx->part_name[i])) {
				pr_err("missing partition %d name\n", i);
				return -EINVAL;
			}
		}
	}
	pr_info("NVM: probe nb_partition =%d, name = %s\n",
			p_vnvm_ctx->nb_part, p_vnvm_ctx->part_name[0]);

	return 0;

}

static int vnvm_server_thread(void *cookie)
{
	uint8_t *pshare_mem;
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;
	unsigned long flags;

	if (of_vnvm_parse()) {
		pr_err("device tree parsing error !\n");
		BUG();
	}

	p_vnvm_ctx->token = mv_ipc_mbox_get_info("vnvm",
			    "vnvm",
			    &vnvm_ops,
			    &pshare_mem,
			    &(p_vnvm_ctx->share_mem_size),
			    &(p_vnvm_ctx->cmdline),
			    (void *)p_vnvm_ctx);

	p_vnvm_ctx->vnvm_share = (struct vnvm_share_ctx *)pshare_mem;

	p_vnvm_ctx->vnvm_share->handshake = mv_gal_os_id();

	p_vnvm_ctx->vnvm_rpc_ctx[0].data = p_vnvm_ctx->vnvm_share->share_data;
	p_vnvm_ctx->vnvm_rpc_ctx[0].max_size = (p_vnvm_ctx->share_mem_size
			- sizeof(struct vnvm_share_ctx)) / 2;

	p_vnvm_ctx->vnvm_rpc_ctx[1].data = p_vnvm_ctx->vnvm_rpc_ctx[0].data +
					   p_vnvm_ctx->vnvm_rpc_ctx[0].max_size;
	p_vnvm_ctx->vnvm_rpc_ctx[1].max_size = (p_vnvm_ctx->share_mem_size
			- sizeof(struct vnvm_share_ctx)) / 2;

	wake_lock_init(&vnvm_be_suspend_lock,
		       WAKE_LOCK_SUSPEND,
		       "VNVM_BE_EMMC_ACCESS_WAKE_LOCK");

	vnvm_probe_devid();
	complete(&setup_done);

	while (1) {
		pr_debug("Polling for NVM message");
		spin_lock_irqsave(&req_lock, flags);
		while (!list_empty(&queue)) {
			struct vnvm_msg_req *msg_req;
			enum vnvm_msg msg;
			msg_req = list_first_entry(&queue,
					struct vnvm_msg_req, node);
			pr_debug("%s:%d:--> Processing message %s(%x)\n",
				__func__, __LINE__,
				vnvm_msg_to_str(msg_req->msg), msg_req->msg);
			if (msg_req->magic != VNVM_MAGIC_REQ)
				BUG();
			list_del(&msg_req->node);
			msg = msg_req->msg;
			kfree(msg_req);
			spin_unlock_irqrestore(&req_lock, flags);
			switch (msg) {
			case VNVM_MSG_CONNECT:
				vnvm_handle_fe_message(p_vnvm_ctx,
						VNVM_OP_DEVICE_INFO);
				break;
			case VNVM_MSG_DISCONNECT:
				vnvm_close();
				break;
			case VNVM_MSG_RPC_CLIENT:
				vnvm_handle_be_message(p_vnvm_ctx);
				break;
			case VNVM_MSG_RPC_SERVER:
			default:
				pr_err("Invalid message: %x\n", msg);
			}
			spin_lock_irqsave(&req_lock, flags);
			pr_debug("Polling for ANOTHER NVM message");
		};
		spin_unlock_irqrestore(&req_lock, flags);
		pr_debug("VNVM: Sleeping...\n");
		__set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(vnvm_waitqueue, !list_empty(&queue));
		pr_debug("VNVM: Wake up...\n");
	}
}

/*******************************************************************************
* Function:... vnvm_init
*******************************************************************************/
static int __init vnvm_init(void)
{
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;

	/* Create server thread */
	thread = kthread_run(vnvm_server_thread, (void *)p_vnvm_ctx, "vnvm");
	if (!IS_ERR(thread))
		wait_for_completion(&setup_done);
	else
		BUG();

	/* Set on line */
	mv_mbox_set_online(p_vnvm_ctx->token);

	return 0;
}

static void vnvm_exit(void)
{
	kthread_stop(thread);

	wake_lock_destroy(&vnvm_be_suspend_lock);
}

module_init(vnvm_init);
module_exit(vnvm_exit);

/*----- Module description -----*/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("Virtual NVM backend driver");
