/*
 * Copyright (C) 2015 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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
#include <sofia/mv_ipc.h>
#include "sec_rpc.h"
#include "vsec.h"
#include "trusty_ipc.h"

#define DEVICE_NAME "trusty-log"
#define TRUSTY_LINE_BUFFER_SIZE 256

struct log_rb {
	uint32_t alloc;
	uint32_t put;
	uint32_t sz;
	char data[0];
} __packed;

struct trusty_log_state {
	struct log_rb *log;
	size_t rb_sz;
	size_t buf_sz;
	uint32_t get;
	uint32_t get_4_user;
	uint32_t token;
	uint8_t connected;
	wait_queue_head_t event_queue;
};

dev_t dev_log = 0;
static int trusty_log_exit_flag;
struct cdev chdev_log;
static struct class *drv_class;
static struct trusty_log_state state;

static int trusty_log_read_line(struct trusty_log_state *s, int put, int get,
				char *line_buffer)
{
	struct log_rb *log = s->log;
	int i;
	char c = '\0';
	size_t max_to_read = min((size_t)(put - get),
				(size_t)(TRUSTY_LINE_BUFFER_SIZE - 1));
	size_t mask = log->sz - 1;

	for (i = 0; i < max_to_read && c != '\n'; )
		line_buffer[i++] = c = log->data[get++ & mask];
	line_buffer[i] = '\0';

	return i;
}

static uint32_t lower_pow2(uint32_t v)
{
	return 1u << (31 - __builtin_clz(v));
}

static unsigned int trusty_log_poll(struct file *filp,
				struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct trusty_log_state *s = &state;

	poll_wait(filp, &s->event_queue, wait);

	/* Writes always succeed for now */
	mask |= POLLOUT | POLLWRNORM;

	if (s->get_4_user != s->log->put)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static ssize_t trusty_log_read(struct file *filp, char __user *buf,
				size_t count, loff_t *offp)
{
	struct trusty_log_state *s = &state;
	uint32_t get, put, alloc;
	int read_chars = 0;
	struct log_rb *log = s->log;
	char line_buffer[TRUSTY_LINE_BUFFER_SIZE];

	get = s->get_4_user;

	if (wait_event_interruptible(s->event_queue, get != log->put)) {
		return -ERESTARTSYS;
	}

	while ((put = log->put) != get) {
		/* Make sure that the read of put occurs
		 * before the read of log data
		 */
		rmb();

		/* Read a line from the log */
		read_chars = trusty_log_read_line(s, put, get, line_buffer);

		/* Force the loads from trusty_log_read_line to complete. */
		rmb();
		alloc = log->alloc;

		/*
		 * Discard the line that was just read if the data could
		 * have been corrupted by the producer.
		 */
		if (alloc - get > log->sz) {
			get = alloc - log->sz;
			continue;
		} else {
			copy_to_user(buf, line_buffer, read_chars);
			get += read_chars;
			break;
		}
	}
	s->get_4_user = get;

	return read_chars;
}

static void trusty_log_on_connect(uint32_t token, void *cookie)
{
	struct trusty_log_state *s = (struct trusty_log_state *)cookie;

	s->connected = 1;
}

static void trusty_log_on_disconnect(uint32_t token, void *cookie)
{
	struct trusty_log_state *s = (struct trusty_log_state *)cookie;

	s->connected = 0;
}

static void trusty_log_on_event(uint32_t token, uint32_t event_id, void *cookie)
{
	struct trusty_log_state *s = (struct trusty_log_state *)cookie;

	wake_up_interruptible(&s->event_queue);
}

static int trusty_log_dump_logs(struct trusty_log_state *s)
{
	struct log_rb *log = s->log;
	uint32_t get, put, alloc;
	int read_chars;
	char line_buffer[TRUSTY_LINE_BUFFER_SIZE];

	BUG_ON(!is_power_of_2(log->sz));

	get = s->get;
	if (wait_event_interruptible(s->event_queue, get != log->put)) {
		pr_err("trusty: %s wait event failed\n", __func__);
		return -ERESTARTSYS;
	}

	/*
	 * For this ring buffer, at any given point, alloc >= put >= get.
	 * The producer side of the buffer is not locked, so the put and alloc
	 * pointers must be read in a defined order (put before alloc) so
	 * that the above condition is maintained. A read barrier is needed
	 * to make sure the hardware and compiler keep the reads ordered.
	 */
	while ((put = log->put) != get) {
		/* Make sure that the read of put occurs before
		 * the read of log data
		 */
		rmb();

		/* Read a line from the log */
		read_chars = trusty_log_read_line(s, put, get, line_buffer);

		/* Force the loads from trusty_log_read_line to complete. */
		rmb();
		alloc = log->alloc;

		/*
		 * Discard the line that was just read if the data could
		 * have been corrupted by the producer.
		 */
		if (alloc - get > log->sz) {
			pr_info("trusty: log overflow.");
			get = alloc - log->sz;
			continue;
		}
		pr_info("trusty: %s", line_buffer);
		get += read_chars;
	}
	s->get = get;

	return 0;
}

static int trusty_log_handle_event(void *cookie)
{
	struct trusty_log_state *s = (struct trusty_log_state *)cookie;
	int ret;

	while (!trusty_log_exit_flag) {
		ret = trusty_log_dump_logs(s);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static struct mbox_ops trusty_log_ops = {
	.on_connect	= trusty_log_on_connect,
	.on_disconnect	= trusty_log_on_disconnect,
	.on_event	= trusty_log_on_event
};

static int trusty_log_init_mbox(struct trusty_log_state *s,
				char *instance,
				uint8_t tx)
{
	char *cmdline;

	s->connected = 0;
	s->token = mv_ipc_mbox_get_info("trusty",
					instance,
					&trusty_log_ops,
					(unsigned char **)&(s->log),
					&(s->buf_sz),
					&cmdline,
					(void *)s);
	if (s->token == (uint32_t)-1) {
		pr_err("ERROR in %s(): failed to init mbox for %s\n",
			__func__, instance);
		return -1;
	}
	s->buf_sz -= sizeof(struct log_rb);

	s->rb_sz = lower_pow2(s->buf_sz - offsetof(struct log_rb, data));
	pr_info("Log ring buffer size = %d\n", s->rb_sz);
	s->log->sz = s->rb_sz;

	kthread_run(trusty_log_handle_event, (void *)s, instance);
	mv_ipc_mbox_set_online(s->token);

	return 0;
}

static int trusty_log_state_init(void)
{
	struct trusty_log_state *s = &state;
	int ret;

	memset(s, 0, sizeof(struct trusty_log_state));
	init_waitqueue_head(&s->event_queue);

	ret = trusty_log_init_mbox(s, "trusty_log", 0);

	return ret;
}

static int trusty_log_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int trusty_log_release(struct inode *inode, struct file *filp)
{
	return 0;
}

const struct file_operations trusty_log_fops = {
	.open		= trusty_log_open,
	.release	= trusty_log_release,
	.read		= trusty_log_read,
	.poll		= trusty_log_poll,
	.owner		= THIS_MODULE,
};

static int __init trusty_log_init(void)
{
	int ret;
	int dev_created = 0;

	if (alloc_chrdev_region(&dev_log, 0, 1, DEVICE_NAME) < 0) {
		pr_err("Failed alloc_chardev_region\n");
		dev_log = 0;
		goto error_exit;
	}

	drv_class = class_create(THIS_MODULE, "trusty-log-class");
	if (drv_class == NULL) {
		pr_err("Failed class_create\n");
		goto error_exit;
	}

	if (device_create(drv_class, NULL, dev_log,
				NULL, DEVICE_NAME) == NULL) {
		pr_err("Failed device_create\n");
		goto error_exit;
	}
	dev_created = 1;

	cdev_init(&chdev_log, &trusty_log_fops);
	if (cdev_add(&chdev_log, dev_log, 1) == -1) {
		pr_err("Failed cdev_add\n");
		goto error_exit;
	}

	trusty_log_exit_flag = 0;

	ret = trusty_log_state_init();
	if (ret < 0) {
		pr_err("Failed to init mbox\n");
		goto error_exit;
	}

	return 0;
error_exit:
	if (dev_created)
		device_destroy(drv_class, dev_log);
	if (drv_class)
		class_destroy(drv_class);
	if (dev_log)
		unregister_chrdev_region(dev_log, 1);
	return -1;
}

static void __exit trusty_log_exit(void)
{
	trusty_log_exit_flag = 1;
	cdev_del(&chdev_log);
	device_destroy(drv_class, dev_log);
	class_destroy(drv_class);
	unregister_chrdev_region(dev_log, 1);
}

module_init(trusty_log_init);
module_exit(trusty_log_exit);

MODULE_DESCRIPTION("Trusty log driver");
MODULE_LICENSE("GPL v2");
