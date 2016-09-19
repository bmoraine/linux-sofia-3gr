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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/uio.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include "sec_rpc.h"
#include "vsec.h"
#include "trusty_ipc.h"
#include <sofia/mv_ipc.h>

static void locked_or(uint32_t *dst, uint32_t bits_to_set)
{
	__asm__ __volatile__ (
		"lock orl %%edx, (%%ecx)\n"
		: : "c" (dst), "d" (bits_to_set)
		);
}

/* return old value */
static inline uint32_t locked_cmpxchg(uint32_t *dst,
				      uint32_t new_val,
				      uint32_t old_val)
{
	uint32_t ret;

	__asm__ __volatile__ (
		"lock cmpxchgl  %%edx,(%%ecx)\n"
		: "=a" (ret)
		: "c" (dst), "d" (new_val), "a" (old_val)
		);

	return ret;
}

struct tipc_mbox {
	uint32_t cmd;
	uint8_t data[0];
};

struct tipc_data {
	uint8_t connected;
	uint8_t tx; /* 0: rx (android to tee), 1: tx (tee to android) */
	uint16_t padding;
	uint32_t token;
	struct tipc_mbox *mbox;
	uint32_t mbox_size;
	uint32_t event;
	wait_queue_head_t event_queue;
	struct completion comp;
};

static struct tipc_data tipc_data[2]; /* tx and rx */
static int tipc_exit_flag;

#define EVENT_MBOX_AVAIL 0x1
#define EVENT_RET_AVAIL 0x2
#define EVENT_RET_INVALID 0x4

static void tipc_on_connect(uint32_t token, void *cookie)
{
	struct tipc_data *p_tipc_data = (struct tipc_data *)cookie;

	p_tipc_data->connected = 1;
	locked_or(&p_tipc_data->event, EVENT_MBOX_AVAIL);
	wake_up_interruptible(&p_tipc_data->event_queue);
}

static void tipc_on_disconnect(uint32_t token, void *cookie)
{
	struct tipc_data *p_tipc_data = (struct tipc_data *)cookie;

	p_tipc_data->connected = 0;
	p_tipc_data->event = 0;
}

static inline bool _has_event(struct tipc_data *p_tipc_data, uint32_t event)
{
	uint32_t old_val, new_val;

	old_val = p_tipc_data->event;
	while (old_val & event) {
		new_val = old_val & (~event);
		if (locked_cmpxchg(&p_tipc_data->event, new_val, old_val)
		    == old_val)
			return true;
		/* it is possible that other bits are changed,
		 * so, try again
		 */
		old_val = p_tipc_data->event;
	}
	return false;
}

static int rpc_dispatch_trusty(u32 opcode, u8 *io_data, u32 *io_data_len)
{
	int rpc_result = RPC_FAILURE;
	uint32_t ret = 0;

	if (io_data == NULL || io_data_len == NULL) {
		pr_err("%s: invalid input : data(0x%08X),",
		       __func__, (u32)io_data);
		pr_err("%s: io_data_len(0x%08X)\n",
		       __func__, (u32)io_data_len);
		goto cleanup;
	}

	ret = trusty_process_cmd(opcode,
				 io_data, *io_data_len,
				 io_data, io_data_len);

	rpc_result = RPC_SUCCESS;

cleanup:
	return rpc_result;
}

static void tipc_on_event(uint32_t token, uint32_t event_id, void *cookie)
{
	struct tipc_data *p_tipc_data = (struct tipc_data *)cookie;

	complete(&p_tipc_data->comp);
}

static int tipc_handle_event(void *cookie)
{
	struct tipc_data *p_tipc_data = (struct tipc_data *)cookie;
	uint32_t io_data_len;

	while (!tipc_exit_flag) {
		wait_for_completion_interruptible(&p_tipc_data->comp);
		if (p_tipc_data->tx) { /* tx, get response from Android */
			locked_or(&p_tipc_data->event, EVENT_RET_AVAIL);
			wake_up_interruptible(&p_tipc_data->event_queue);
			wait_event_interruptible(p_tipc_data->event_queue,
						 _has_event(p_tipc_data,
							    EVENT_RET_INVALID));
			locked_or(&p_tipc_data->event, EVENT_MBOX_AVAIL);
			wake_up_interruptible(&p_tipc_data->event_queue);
		} else {  /* rx, handle request from Android, need reponse */
			io_data_len = p_tipc_data->mbox_size;
			rpc_dispatch_trusty(p_tipc_data->mbox->cmd,
					    &(p_tipc_data->mbox->data[0]),
					    &io_data_len);
			mv_ipc_mbox_post(p_tipc_data->token, 0);
		}
	}

	return 0;
}

static struct mbox_ops tipc_ops = {
	.on_connect	= tipc_on_connect,
	.on_disconnect	= tipc_on_disconnect,
	.on_event	= tipc_on_event
};

static void tipc_init_mbox(struct tipc_data *p_tipc_data,
			   char *instance,
			   uint8_t tx)
{
	char *cmdline;

	p_tipc_data->connected = 0;
	p_tipc_data->tx = tx;
	init_waitqueue_head(&p_tipc_data->event_queue);
	p_tipc_data->token =
		mv_ipc_mbox_get_info("trusty",
				     instance,
				     &tipc_ops,
				     (unsigned char **)&(p_tipc_data->mbox),
				     &(p_tipc_data->mbox_size),
				     &cmdline,
				     (void *)p_tipc_data);
	if (p_tipc_data->token == (uint32_t)-1) {
		pr_err("ERROR in %s(): failed to init mbox for %s\n",
		       __func__, instance);
		return;
	}
	p_tipc_data->mbox_size -= sizeof(struct tipc_mbox);
	init_completion(&p_tipc_data->comp);
	kthread_run(tipc_handle_event, (void *)p_tipc_data, instance);
	mv_ipc_mbox_set_online(p_tipc_data->token);
}

static void tipc_ipc_init(void)
{
	tipc_init_mbox(&tipc_data[0], "t2a", 0);
	tipc_init_mbox(&tipc_data[1], "a2t", 1);
}

static uint32_t tipc_send_data(uint32_t cmd, uint8_t *data, uint32_t data_len)
{
	uint32_t ret;
	struct tipc_data *p_tipc_data = &tipc_data[1];

	if (p_tipc_data->connected == 0) {
		pr_err("ERROR in %s(): not connected yet\n", __func__);
		return -ECOMM;
	}

	if (data_len > p_tipc_data->mbox_size) {
		pr_err("ERROR in %s(): data too big (data_len=0x%x)\n",
		       __func__, data_len);
		return -ECOMM;
	}

	wait_event_interruptible(p_tipc_data->event_queue,
				 _has_event(p_tipc_data, EVENT_MBOX_AVAIL));
	barrier();
	p_tipc_data->event &= ~EVENT_RET_AVAIL;
	/* copy cmd and data to shared memory */
	p_tipc_data->mbox->cmd = cmd;
	memcpy(&p_tipc_data->mbox->data[0], data, data_len);
	/* signal peer */
	mv_ipc_mbox_post(p_tipc_data->token, 0);
	/* TODO: ret is only required for msg.
	 * for connect and close,
	 * not wait for ret to enhance performance */
	/* wait for ret */
	wait_event_interruptible(p_tipc_data->event_queue,
				 _has_event(p_tipc_data, EVENT_RET_AVAIL));
	/* get ret */
	ret = *(uint32_t *)&(p_tipc_data->mbox->data[0]);
	locked_or(&p_tipc_data->event, EVENT_RET_INVALID);
	wake_up_interruptible(&p_tipc_data->event_queue);
	return ret;
}

#define TIPC_IOC_MAGIC                  'r'
#define TIPC_IOC_CONNECT                _IOW(TIPC_IOC_MAGIC, 0x80, char *)

#define PORT_PATH_MAX 64
#define REPLY_TIMEOUT 5000

enum {
	TRUSTY_STATE_WORKING = 0x0,
	TRUSTY_STATE_LOCAL_CLOSED = 0x1,
	TRUSTY_STATE_REMOTE_CLOSED = 0x2,
	TRUSTY_STATE_BOTH_CLOSED = 0x3,
	TRUSTY_STATE_NOT_CONNECTED = 0x4,
	TRUSTY_STATE_CONNECTING = 0x8
};

enum {
	TRUSTY_CMD_CONN,
	TRUSTY_CMD_CONN_RSP, /* response */
	TRUSTY_CMD_CLOSE,
	TRUSTY_CMD_MSG
};

struct trusty_msg {
	struct list_head node;
	size_t size;
	uint8_t buf[0];
};

struct trusty_chan {
	uint32_t state;
	uint32_t peer_id;
	uint32_t buf_size;
	uint32_t ref_count;
	struct mutex lock;
	wait_queue_head_t readq;
	struct completion reply_comp;
	struct list_head rx_msg_queue;
};

void locked_inc(uint32_t *dst)
{
	__asm__ __volatile__ (
		"lock incl (%0)\n"
		: : "r" (dst)
		);
}

uint32_t locked_dec(uint32_t *dst)
{
	uint32_t old_val, new_val;

	while (1) {
		old_val = *dst;
		new_val = old_val - 1;
		if (old_val == locked_cmpxchg(dst, new_val, old_val))
			break;
		barrier();
	}
	return new_val;
}

static inline struct trusty_chan *tipc_get_chan(struct file *filp)
{
	struct trusty_chan *chan = filp->private_data;

	/* chan must exist, no need to check it */
	locked_inc(&chan->ref_count);
	return chan;
}

static void tipc_destroy_chan(struct trusty_chan *chan)
{
	struct trusty_msg *msg;

	if (chan->state & TRUSTY_STATE_REMOTE_CLOSED) {
		msg = list_first_entry_or_null(
			&(chan->rx_msg_queue), struct trusty_msg, node);
		while (msg) {
			list_del(&msg->node);
			kfree(msg);
			msg = list_first_entry_or_null(
				&(chan->rx_msg_queue), struct trusty_msg, node);
		}
		kfree(chan);
		pr_err("%s() called\n", __func__);
	}
}

static inline void tipc_put_chan(struct trusty_chan *chan)
{
	if (locked_dec(&chan->ref_count) == 0)
		tipc_destroy_chan(chan);
}

struct tipc_conn_rsp {
	uint64_t peer_id;
	uint32_t rc;
	uint32_t buf_size;
};

struct tipc_msg {
	uint64_t peer_id;
	uint32_t size;
	uint32_t pad;
	char buf[0];
};

static int tipc_open(struct inode *inode, struct file *filp)
{
	struct trusty_chan *chan;

	chan = kzalloc(sizeof(struct trusty_chan), GFP_KERNEL);
	if (!chan) {
		pr_err("%s(): malloc fail for channel\n", __func__);
		return -ENOMEM;
	}

	chan->state = TRUSTY_STATE_NOT_CONNECTED;
	chan->peer_id = (uint32_t)-1;
	chan->buf_size = 0;
	/* init as 1, which will be decreased in close() */
	chan->ref_count = 1;
	mutex_init(&chan->lock);
	init_waitqueue_head(&chan->readq);
	init_completion(&chan->reply_comp);
	INIT_LIST_HEAD(&chan->rx_msg_queue);

	filp->private_data = chan;
	return 0;
}

static long tipc_ioctl_connect(struct trusty_chan *chan, char __user *usr_name)
{
	long ret = 0;
	struct tipc_msg *msg = NULL;
	uint32_t io_data_len = 0;

	msg = kzalloc(sizeof(struct tipc_msg) + PORT_PATH_MAX, GFP_KERNEL);
	if (!msg) {
		pr_err("%s(): failed to malloc for msg\n", __func__);
		return -ENOMEM;
	}

	msg->peer_id = (uint64_t)(uint32_t)chan;

	mutex_lock(&chan->lock);
	if (chan->state == TRUSTY_STATE_NOT_CONNECTED)
		chan->state = TRUSTY_STATE_CONNECTING;
	else{
		pr_err("%s(): invalid state (=0x%x)\n",
		       __func__, chan->state);
		if (chan->state & TRUSTY_STATE_BOTH_CLOSED)
			ret = -EISCONN;
		else if (chan->state == TRUSTY_STATE_CONNECTING)
			ret = -EALREADY;
		else    /* >TRUSTY_STATE_CONNECTING */
			ret = -EINVAL;
	}
	mutex_unlock(&chan->lock);
	if (ret != 0)
		goto cleanup;

	ret = strncpy_from_user(msg->buf, usr_name, PORT_PATH_MAX);
	if (ret < 0) {
		pr_err("%s(): copy_from_user (%p) failed (%ld)\n",
		       __func__, usr_name, ret);
		goto cleanup;
	}
	if (ret == PORT_PATH_MAX) {
		pr_err("%s(): port path is too long\n", __func__);
		ret = -EINVAL;
		goto cleanup;
	}
	msg->size = ret + 1;
	io_data_len = sizeof(struct tipc_msg) + msg->size;
	tipc_send_data(TRUSTY_CMD_CONN, (uint8_t *)msg,
		       sizeof(struct tipc_msg) + msg->size);

	mutex_lock(&chan->lock);
	ret = wait_for_completion_interruptible_timeout(
		&chan->reply_comp,
		msecs_to_jiffies(REPLY_TIMEOUT));
	if (ret < 0) {
		pr_err("%s(): wait for response failed, err=%ld\n",
				__func__, ret);
		mutex_unlock(&chan->lock);
		goto cleanup;
	}
	if (ret == 0) { /* timeout */
		chan->state |= TRUSTY_STATE_REMOTE_CLOSED;
		chan->peer_id = -1;
		chan->buf_size = 0;
		mutex_unlock(&chan->lock);
		pr_err("%s(): timeout for response\n", __func__);
		ret = -ETIMEDOUT;
		goto cleanup;
	}
	if (chan->state == TRUSTY_STATE_WORKING)
		ret = 0;
	else
		ret = -ENOTCONN;

	mutex_unlock(&chan->lock);
cleanup:
	kfree(msg);
	return ret;
}

static void tipc_on_connect_rsp(struct trusty_chan *chan,
				int status,
				uint32_t buf_size)
{
	if (chan->state == TRUSTY_STATE_CONNECTING) {
		if (status >= 0) {
			chan->state = TRUSTY_STATE_WORKING;
			chan->peer_id = status;
			chan->buf_size = buf_size;
		} else {
			chan->state = TRUSTY_STATE_REMOTE_CLOSED;
			chan->peer_id = -1;
			chan->buf_size = 0;
		}
		complete(&chan->reply_comp);
	} else
		pr_err("%s(): called with state is 0x%x\n",
		       __func__, chan->state);
}

static long tipc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct trusty_chan *chan = tipc_get_chan(filp);
	long ret;

	switch (cmd) {
	case TIPC_IOC_CONNECT:
		ret = tipc_ioctl_connect(chan, (char __user *)arg);
		break;
	default:
		pr_warn("%s: Unhandled ioctl cmd: 0x%x\n",
			__func__, cmd);
		ret = -EINVAL;
	}
	tipc_put_chan(chan);
	return ret;
}

static int tipc_release(struct inode *inode, struct file *filp)
{
	struct trusty_chan *chan = tipc_get_chan(filp);
	uint32_t io_data_len = 0;

	mutex_lock(&chan->lock);
	chan->state |= TRUSTY_STATE_LOCAL_CLOSED;
	complete(&chan->reply_comp);
	wake_up_interruptible_all(&chan->readq);
	io_data_len = sizeof(uint32_t);
	if (chan->peer_id != -1)
		tipc_send_data(TRUSTY_CMD_CLOSE,
			       (uint8_t *)&(chan->peer_id),
			       sizeof(uint32_t));
	mutex_unlock(&chan->lock);
	tipc_put_chan(chan);
	tipc_put_chan(chan); /* put it twice to make ref_counf 0 */
	return 0;
}

static int tipc_on_msg(struct trusty_chan *chan, char *buf, uint32_t size)
{
	struct trusty_msg *msg;

	if (chan->state != TRUSTY_STATE_WORKING) {
		pr_err("%s(): invalid state (0x%x)\n", __func__, chan->state);
		return -1;
	}

	msg = kzalloc(sizeof(struct trusty_msg) + size, GFP_KERNEL);
	if (!msg) {
		pr_err("%s(): failed to malloc for msg, size=0x%x\n",
		       __func__, size);
		return -1;
	}

	memcpy(&msg->buf, buf, size);
	msg->size = size;
	mutex_lock(&chan->lock);
	list_add_tail(&msg->node, &chan->rx_msg_queue);
	wake_up_interruptible(&chan->readq);
	mutex_unlock(&chan->lock);
	return 0;
}

static inline bool _got_rx(struct trusty_chan *chan)
{
	if (chan->state != TRUSTY_STATE_WORKING)
		return true;

	if (!list_empty(&chan->rx_msg_queue))
		return true;

	return false;
}

static ssize_t tipc_read(struct file *filp, char __user *buf,
			 size_t count, loff_t *offp)
{
	struct trusty_chan *chan = tipc_get_chan(filp);
	struct trusty_msg *msg;
	long ret;

	while (1) {
		if (chan->state & TRUSTY_STATE_LOCAL_CLOSED) {
			pr_err("%s(): invalid state (0x%x)\n",
			       __func__, chan->state);
			ret = -EINVAL;
			break;
		}
		mutex_lock(&chan->lock);
		if (list_empty(&chan->rx_msg_queue)) {
			pr_warn("%s(): no message\n", __func__);
			ret = -ENOMSG;
		} else {
			msg = list_first_entry(&chan->rx_msg_queue,
					       struct trusty_msg, node);
			if (msg->size > count) {
				pr_err("%s():buf too small(0x%x),need 0x%x\n",
				       __func__, count, msg->size);
				ret = -EMSGSIZE;
			} else {
				ret = 0;
				list_del(&msg->node);
			}
		}
		mutex_unlock(&chan->lock);
		if (ret == -ENOMSG)
			if (wait_event_interruptible(chan->readq,
						     _got_rx(chan)))
				ret = -ERESTARTSYS;
		if (ret != -ENOMSG)
			break;
	}
	if (ret == 0) {
		copy_to_user(buf, &msg->buf[0], msg->size);
		ret = msg->size;
		kfree(msg);
	}

	tipc_put_chan(chan);
	return ret;
}

static ssize_t tipc_write(struct file *filp, const char __user *buf,
			  size_t count, loff_t *offp)
{
	struct trusty_chan *chan = tipc_get_chan(filp);
	long ret;
	struct tipc_msg *msg = NULL;
	uint32_t io_data_len = 0;

	if (chan->state != TRUSTY_STATE_WORKING) {
		pr_err("%s(): invalid state (0x%x)\n", __func__, chan->state);
		ret = -EINVAL;
		goto cleanup;
	}

	if (((uint32_t)count) > chan->buf_size) {
		pr_err("%s(): message too long (0x%x > 0x%x)\n",
		       __func__, (uint32_t)count, chan->buf_size);
		ret = -EMSGSIZE;
		goto cleanup;
	}

	msg = kzalloc(sizeof(struct tipc_msg) + count, GFP_KERNEL);
	if (!msg) {
		pr_err("%s(): failed to malloc for msg\n", __func__);
		ret = -ENOMEM;
		goto cleanup;
	}

	msg->peer_id = (uint64_t)chan->peer_id;
	msg->size = count;
	copy_from_user(msg->buf, buf, count);

	io_data_len = sizeof(struct tipc_msg) + msg->size;
	ret = tipc_send_data(TRUSTY_CMD_MSG,
			     (uint8_t *)msg,
			     sizeof(struct tipc_msg) + msg->size);
	if (ret != 0) {
		pr_err("%s(): ret from rpc = 0x%lx\n", __func__, ret);
		ret = -EIO;
		goto cleanup;
	}
	ret = count;
cleanup:
	kfree(msg);
	tipc_put_chan(chan);
	return ret;
}

static unsigned int tipc_poll(struct file *filp,
			      struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct trusty_chan *chan = tipc_get_chan(filp);

	mutex_lock(&chan->lock);

	poll_wait(filp, &chan->readq, wait);

	/* Writes always succeed for now */
	mask |= POLLOUT | POLLWRNORM;

	if (!list_empty(&chan->rx_msg_queue))
		mask |= POLLIN | POLLRDNORM;

	if (chan->state != TRUSTY_STATE_WORKING)
		mask |= POLLERR;

	mutex_unlock(&chan->lock);
	tipc_put_chan(chan);
	return mask;
}

static void tipc_on_close(struct trusty_chan *chan)
{
	mutex_lock(&chan->lock);
	chan->state |= TRUSTY_STATE_REMOTE_CLOSED;
	mutex_unlock(&chan->lock);
}

uint32_t trusty_process_cmd(uint32_t opcode, uint8_t *input_buffer,
			    uint32_t input_size, uint8_t *output_buffer,
			    uint32_t *output_size)
{
	struct trusty_chan *chan;
	struct tipc_conn_rsp *conn_rsp;
	struct tipc_msg *tmsg;

	switch (opcode) {
	case TRUSTY_CMD_CONN_RSP:
		if (input_size < sizeof(struct tipc_conn_rsp)) {
			pr_err("%s(): buffer too small (%d) for conn_rsp\n",
			       __func__, input_size);
			return RPC_FAILURE;
		}
		conn_rsp = (struct tipc_conn_rsp *)input_buffer;
		chan = (struct trusty_chan *)(uint32_t)conn_rsp->peer_id;
		locked_inc(&chan->ref_count);
		tipc_on_connect_rsp(chan, conn_rsp->rc, conn_rsp->buf_size);
		tipc_put_chan(chan);
		break;
	case TRUSTY_CMD_MSG:
		tmsg = (struct tipc_msg *)input_buffer;
		if ((input_size < sizeof(struct tipc_msg))  ||
		    (input_size < (tmsg->size + sizeof(struct tipc_msg)))) {
			pr_err("%s(): buf too small: tmsg->size=0x%x, input_size=0x%x\n",
			       __func__, tmsg->size, input_size);
			return RPC_FAILURE;
		}
		chan = (struct trusty_chan *)(uint32_t)tmsg->peer_id;
		locked_inc(&chan->ref_count);
		*(int *)output_buffer = tipc_on_msg(chan,
						    (char *)tmsg->buf,
						    tmsg->size);
		tipc_put_chan(chan);
		*output_size = sizeof(int);
		break;
	case TRUSTY_CMD_CLOSE:
		if (input_size < sizeof(struct trusty_chan *)) {
			pr_err("%s(): buf too small (%d) for close\n",
			       __func__, input_size);
			return RPC_FAILURE;
		}
		chan = *(struct trusty_chan **)input_buffer;
		locked_inc(&chan->ref_count);
		tipc_on_close(chan);
		tipc_put_chan(chan);
		*output_size = 0;
		break;
	default:
		pr_err("Invalid opcode %x\n", opcode);
		return RPC_FAILURE;
	}
	return RPC_SUCCESS;
}

const struct file_operations tipc_fops = {
	.open			= tipc_open,
	.release		= tipc_release,
	.read			= tipc_read,
	.write			= tipc_write,
	.poll			= tipc_poll,
	.unlocked_ioctl		= tipc_ioctl,
#if defined(CONFIG_COMPAT)
	.compat_ioctl		= tipc_ioctl,
#endif
	.owner			= THIS_MODULE,
};

#define DEVICE_NAME "trusty-ipc-dev"
dev_t dev = 0;
struct cdev chdev;
static struct class *drv_class; /* Device class */

static int __init tipc_init(void)
{
	int dev_created = 0;

	if (alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME) < 0) {
		pr_err("Failed alloc_chardev_region\n");
		dev = 0;
		goto error_exit;
	}

	drv_class = class_create(THIS_MODULE, "trusty-ipc-class");
	if (drv_class == NULL) {
		pr_err("Failed class_create\n");
		goto error_exit;
	}

	if (device_create(drv_class, NULL, dev, NULL, DEVICE_NAME) == NULL) {
		pr_err("Failed device_create\n");
		goto error_exit;
	}
	dev_created = 1;

	cdev_init(&chdev, &tipc_fops);
	if (cdev_add(&chdev, dev, 1) == -1) {
		pr_err("Failed cdev_add\n");
		goto error_exit;
	}

	tipc_exit_flag = 0;

	tipc_ipc_init();

	return 0;
error_exit:
	if (dev_created)
		device_destroy(drv_class, dev);
	if (drv_class)
		class_destroy(drv_class);
	if (dev)
		unregister_chrdev_region(dev, 1);
	return -1;
}

static void __exit tipc_exit(void)
{
	/* TODO: clean up for tipc_init() */
	tipc_exit_flag = 1;
	cdev_del(&chdev);
	device_destroy(drv_class, dev);
	class_destroy(drv_class);
	unregister_chrdev_region(dev, 1);
}

module_init(tipc_init);
module_exit(tipc_exit);

MODULE_DESCRIPTION("Trusty IPC driver");
MODULE_LICENSE("GPL v2");

