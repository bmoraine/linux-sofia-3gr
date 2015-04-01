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
#include <linux/stddef.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/semaphore.h>
#include <linux/fs.h>

#include <sofia/mv_gal.h>
#include <sofia/mv_ipc.h>

#include "vnvm_common.h"

//#define VNVM_DEBUG

#define TRACE(_f, _a...)  printk (KERN_INFO    "VNVM-BE: " _f, ## _a)
#define ETRACE(_f, _a...) printk (KERN_ERR     "VNVM-BE: Error: " _f, ## _a)

#ifdef VNVM_DEBUG
#define DTRACE(_f, _a...) \
        do { printk (KERN_ALERT "VNVM-BE: %s: " _f, __func__, ## _a); } while (0)
#else
#define DTRACE(_f, _a...) ((void)0)
#endif

#define VNVM_SUCCESS	0
#define VNVM_FAILURE	1

enum vnvm_status {
	VNVM_STATUS_CONNECT = 0,
	VNVM_STATUS_DISCONNECT
};

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
	enum vnvm_status status;
	uint32_t server_index;
	uint32_t client_index;
	wait_queue_head_t open_wq;

	uint32_t is_client_wait;
	uint32_t server_responded;
	wait_queue_head_t client_wq;
	struct mutex client_mutex;

	uint32_t is_client_request;
	uint32_t is_devinit;
	struct semaphore server_sem;

	uint32_t is_thread_aborted;

	uint32_t dev_active;
	char devname [VNVM_DEV_NAME_LIMIT];
	uint32_t devid;
	uint32_t devsize;
	struct file *fd;

	struct vnvm_share_ctx *vnvm_share;

};

static struct vnvm_ctx vnvm_ctx;

static char vnvm_erase_buf[32*1024];  /* 32kb */
static struct wake_lock vnvm_be_suspend_lock;

static void vnvm_exit_device (void);

/*******************************************************************************
* Function:... vnvm_call
*******************************************************************************/
uint32_t vnvm_call(void)
{
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;

	if(wait_event_interruptible(p_vnvm_ctx->open_wq,
				    p_vnvm_ctx->status == VNVM_STATUS_CONNECT))
		return VNVM_FAILURE;

	/* Only one thread can perform RPC call */
	if (mutex_lock_interruptible(&p_vnvm_ctx->client_mutex))
		return 0;

	p_vnvm_ctx->server_responded = 0;

	/* Perform RPC call (post client event) */
	mv_ipc_mbox_post(p_vnvm_ctx->token, VNVM_RPC_EVENT_CLIENT);

	/* Waiting for respond */
	p_vnvm_ctx->is_client_wait = 1;

	/* Race condition protection */
	if (p_vnvm_ctx->status != VNVM_STATUS_CONNECT) {
		mutex_unlock(&p_vnvm_ctx->client_mutex);
		return VNVM_FAILURE;
	}

	DTRACE("wait server event!\n");
	/* wait for server respond */
	if(wait_event_interruptible(p_vnvm_ctx->client_wq,
				    p_vnvm_ctx->server_responded == 1)) {
		DTRACE("wait server event error exit!\n");
		mutex_unlock(&p_vnvm_ctx->client_mutex);
		return VNVM_FAILURE;
	}

	DTRACE("wait server event success exit!\n");

	p_vnvm_ctx->is_client_wait = 0;

	/* Disconnect exit */
	if (p_vnvm_ctx->status != VNVM_STATUS_CONNECT) {
		mutex_unlock(&p_vnvm_ctx->client_mutex);
		return VNVM_FAILURE;
	}

	mutex_unlock(&p_vnvm_ctx->client_mutex);

	return VNVM_SUCCESS;
}

int vvfs_device_write(loff_t offset, char *buf, size_t count)
{
	ssize_t cnt;
	loff_t pos;
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;
	mm_segment_t old_fs = get_fs();

	if((pos = vfs_llseek(p_vnvm_ctx->fd, offset, SEEK_SET)) < 0)
		return 0;
	set_fs(KERNEL_DS);
	cnt = vfs_write(p_vnvm_ctx->fd, buf, count, &pos);
	set_fs(old_fs);
	return cnt;
}

void vnvm_handle_be_message(void *message)
{
	nvm_request *req = (nvm_request *)message;
	nvm_reply *reply = (nvm_reply *)message;

	switch(req->op) {
	case VNVM_OP_WRITE_DATA: {
		ssize_t cnt;
		cnt = vvfs_device_write(req->addr, (char *)req->data, req->size);
		reply->retcode = cnt;
		break;
	}
	default:
		panic("unknown nvm request!\n");
		break;
	}
}

uint8_t *vnvm_get_out_buffer (void)
{
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;
	return p_vnvm_ctx->vnvm_rpc_ctx[p_vnvm_ctx->client_index].data;
}

static signed vnvm_get_devid (void)
{
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;

	p_vnvm_ctx->devid = 0xB303;

	// we could have done better, but this is how it works today
	snprintf(p_vnvm_ctx->devname, sizeof(p_vnvm_ctx->devname),
		 "/dev/block/platform/soc0/e0000000.noc/by-name/ImcPartID022");

	return 0;
}

static signed vnvm_probe_devid (void)
{
	struct vnvm_ctx *be = &vnvm_ctx;

	DTRACE ("\n");
	if (vnvm_get_devid ())
		return -ENODEV;

	if (be->devid == 0) {
		ETRACE ("Could not proceed further as no device configured\n");
		return -ENODEV;
	}

	DTRACE ("device id (%u,%u) name %s\n", VNVM_DEVID_MAJOR (be->devid),
		VNVM_DEVID_MINOR (be->devid), be->devname);

	return 0;
}

static signed vnvm_set_devinfo (void)
{
	nvm_request *req;
	nvm_reply *reply;
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;

	DTRACE ("\n");
	req = (nvm_request *)vnvm_get_out_buffer();

	req->op = VNVM_OP_DEVICE_INFO;
	req->addr = p_vnvm_ctx->devid;
	req->size = p_vnvm_ctx->devsize;

	if(vnvm_call() != VNVM_SUCCESS)
		return -EBUSY;

	reply = (nvm_reply *)vnvm_get_out_buffer();
	return reply->retcode;
}

int vnvm_init_device (uint32_t is_retry)
{
	signed diag;
	loff_t pos;
	struct file *fd;
	mm_segment_t old_fs = get_fs();
	struct vnvm_ctx *be = &vnvm_ctx;

	if(be->dev_active) {
		TRACE ("device %s size %u bytes already opened\n", be->devname, be->devsize);
		if((diag = vnvm_set_devinfo()) != 0) {
			ETRACE("could not send device info\n");
			goto dev_error;
		}
		return 0;
	}

	set_fs(KERNEL_DS);
	fd = filp_open(be->devname, (O_RDWR | O_SYNC), 0);
	set_fs(old_fs);
	if(unlikely(IS_ERR(fd))) {
		/* Better use system events to get notified on device removal/creation */
		ETRACE("open device %s failed, retrying\n", be->devname);
		goto dev_retry;
	}

	be->fd = fd;
	if((pos = vfs_llseek(fd, 0,SEEK_END))< 0) {
		ETRACE("Seek to end of device failed\n");
		diag = -ESPIPE;
		goto dev_error;
	}

	if(pos == 0) {
		ETRACE("device %s size 0 bytes\n", be->devname);
		diag = -ESPIPE;
		goto dev_error;
	}

	/* VFS assumes that after erase flash content is 0xFF */
	memset(vnvm_erase_buf, 0xFF, sizeof vnvm_erase_buf);
	be->devsize = pos;
	be->dev_active = 1;
	TRACE ("opened device %s size %lld bytes\n", be->devname, pos);

	if((diag = vnvm_set_devinfo()) != 0) {
		ETRACE("could not send device info\n");
		goto dev_error;
	}

	return 0;

dev_error:
	/* No retry */
	vnvm_exit_device();
	return diag;

dev_retry:
	if (is_retry) {
		/* Waiting for the device before retry */
		set_current_state (TASK_INTERRUPTIBLE);
		schedule_timeout (1000);
		be->is_devinit = 1;
		up(&be->server_sem);
	}
	return -EAGAIN;
}

static int vnvm_server_thread(void *cookie)
{
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;

	DTRACE ("started\n");

	p_vnvm_ctx->vnvm_share->server_status = 0;

	while(!p_vnvm_ctx->is_thread_aborted) {
		/* wait for server event */
		DTRACE ("waiting for event\n");
		 p_vnvm_ctx->vnvm_share->server_status = 1;
		down_interruptible(&p_vnvm_ctx->server_sem);
		 p_vnvm_ctx->vnvm_share->server_status = 0;

		DTRACE ("wakeup%s%s%s\n",
			p_vnvm_ctx->is_thread_aborted ? " thread-aborted"  : "",
			p_vnvm_ctx->is_client_request ? " client-request"  : "",
			p_vnvm_ctx->is_devinit        ? " devinit"         : "");

		/* thread aborted */
		if(p_vnvm_ctx->is_thread_aborted)
			break;

		/* device init */
		if(p_vnvm_ctx->is_devinit) {
			p_vnvm_ctx->is_devinit = 0;
			vnvm_init_device (1);
			continue;
		}

		/* client request */
		if(p_vnvm_ctx->is_client_request) {
			p_vnvm_ctx->is_client_request = 0;
			vnvm_handle_be_message(p_vnvm_ctx->vnvm_rpc_ctx[p_vnvm_ctx->server_index].data);
			mv_ipc_mbox_post(p_vnvm_ctx->token, VNVM_RPC_EVENT_SERVER);
			wake_unlock(&vnvm_be_suspend_lock);
		}

	}

	DTRACE("exiting thread\n");

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

	/* Set connect */
	p_vnvm_ctx->status = VNVM_STATUS_CONNECT;

	wake_up_interruptible(&p_vnvm_ctx->open_wq);

	vnvm_probe_devid();

	if(p_vnvm_ctx->dev_active == 0) {
		p_vnvm_ctx->is_devinit = 1;
		up(&p_vnvm_ctx->server_sem);
	}
}

static void vnvm_on_disconnect(uint32_t token, void *cookie)
{
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;
	mm_segment_t old_fs;

	p_vnvm_ctx->dev_active = 0;
	p_vnvm_ctx->status = VNVM_STATUS_DISCONNECT;

	if (p_vnvm_ctx->fd) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		filp_close(p_vnvm_ctx->fd, NULL);
		p_vnvm_ctx->fd = NULL;
		set_fs(old_fs);
	}

	/* wake up client if still waiting*/
	if (p_vnvm_ctx->is_client_wait)
		wake_up_interruptible(&p_vnvm_ctx->client_wq);
}

static void vnvm_on_event(uint32_t token, uint32_t event_id, void *cookie)
{
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;

	switch (event_id) {
	case VNVM_RPC_EVENT_CLIENT:
		wake_lock(&vnvm_be_suspend_lock);
		p_vnvm_ctx->is_client_request = 1;
		up(&p_vnvm_ctx->server_sem);
		break;
	case VNVM_RPC_EVENT_SERVER:
		p_vnvm_ctx->server_responded = 1;
		wake_up_interruptible(&p_vnvm_ctx->client_wq);
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

static void vnvm_exit_device ()
{
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;
	mm_segment_t old_fs;

	if(p_vnvm_ctx->dev_active == 0) return;

	p_vnvm_ctx->dev_active = 0;

	if (p_vnvm_ctx->fd) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		filp_close(p_vnvm_ctx->fd, NULL);
		p_vnvm_ctx->fd = NULL;
		set_fs(old_fs);
	}

	p_vnvm_ctx->devsize = 0;
	return;
}

/*******************************************************************************
* Function:... vnvm_init
*******************************************************************************/
uint8_t vnvm_server_thread_stack[4096];
uint32_t vnvm_initialised = 0;

static int __init vnvm_init (void)
{
	uint8_t *pshare_mem;
	struct vnvm_ctx *p_vnvm_ctx = &vnvm_ctx;

	if(vnvm_initialised)
		return 1;

	DTRACE ("initializing\n");

	p_vnvm_ctx->token = mv_ipc_mbox_get_info("vnvm",
			    "vnvm",
			    &vnvm_ops,
			    &pshare_mem,
			    &(p_vnvm_ctx->share_mem_size),
			    &(p_vnvm_ctx->cmdline),
			    (void *)p_vnvm_ctx);

	p_vnvm_ctx->status = VNVM_STATUS_DISCONNECT;
	p_vnvm_ctx->vnvm_share = (struct vnvm_share_ctx *)pshare_mem;

	p_vnvm_ctx->vnvm_share->handshake = mv_gal_os_id();

	p_vnvm_ctx->vnvm_rpc_ctx[0].data = p_vnvm_ctx->vnvm_share->share_data;
	p_vnvm_ctx->vnvm_rpc_ctx[0].max_size = (p_vnvm_ctx->share_mem_size - sizeof(
			struct vnvm_share_ctx) ) / 2;

	p_vnvm_ctx->vnvm_rpc_ctx[1].data = p_vnvm_ctx->vnvm_rpc_ctx[0].data +
					   p_vnvm_ctx->vnvm_rpc_ctx[0].max_size;
	p_vnvm_ctx->vnvm_rpc_ctx[1].max_size = (p_vnvm_ctx->share_mem_size - sizeof(
			struct vnvm_share_ctx) ) / 2;

	init_waitqueue_head(&p_vnvm_ctx->open_wq);

	init_waitqueue_head(&p_vnvm_ctx->client_wq);
	p_vnvm_ctx->is_client_wait = 0;
	mutex_init(&p_vnvm_ctx->client_mutex);

	sema_init(&p_vnvm_ctx->server_sem, 0);

	wake_lock_init(&vnvm_be_suspend_lock,
		       WAKE_LOCK_SUSPEND,
		       "VNVM_BE_EMMC_ACCESS_WAKE_LOCK");

	/* Create server thread */
	kthread_run(vnvm_server_thread, (void *)p_vnvm_ctx, "vnvm");


	/* Set on line */
	mv_mbox_set_online(p_vnvm_ctx->token);

	vnvm_initialised = 1;

	return 0;
}

static void vnvm_exit (void)
{
	struct vnvm_ctx *be = &vnvm_ctx;

	DTRACE ("exiting\n");
	vnvm_exit_device ();

	be->is_thread_aborted = 1;
	up(&be->server_sem);

	wake_lock_destroy(&vnvm_be_suspend_lock);
}

module_init (vnvm_init);
module_exit (vnvm_exit);

/*----- Module description -----*/

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION ("Virtual NVM backend driver");
