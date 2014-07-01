/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/stddef.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/init.h>

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/nk_sofia_bridge.h>
#endif

#include "vrpc.h"
#include "voemcrypto_common.h"

#if 1
#define VOEMCRYPTO_DEBUG
#endif

/*----- Tracing -----*/
#ifdef VOEMCRYPTO_DEBUG
#define DTRACE(format, args...)	\
	pr_debug("%s: " format, __func__, ##args)
#else
#define DTRACE(x...)
#endif

#define TRACE(x...)	pr_notice("VOEMCRYPTO-FE: " x)
#define WTRACE(x...)	pr_warn("VOEMCRYPTO-FE: " x)
#define ETRACE(x...)	pr_err("VOEMCRYPTO-FE: " x)


/**** Data/structure definitions *****/
enum voemcrypto_devstate_t {
	VOEMCRYPTO_DEVICE_IDLE,
	VOEMCRYPTO_DEVICE_BUSY,
	VOEMCRYPTO_DEVICE_WAITING_FOR_READ,
};

struct voemcrypto_t {
	/* Internal */
	struct vrpc_t *vrpc;
	void *vrpc_data;
	struct mutex call_mutex;
};

/**** Data declarations *****/
static struct voemcrypto_t *voemcrypto_data = (struct voemcrypto_t *)NULL;  /* pointer to struct */
static int device_open = (int)0;
static struct proc_dir_entry *proc_file_entry = (struct proc_dir_entry *)NULL;
static enum voemcrypto_devstate_t device_state = VOEMCRYPTO_DEVICE_IDLE;

/********** Local function declarations *******/
vrpc_size_t voemcrypto_call_ext(struct voemcrypto_t*, const unsigned int*,
	       const vrpc_size_t);

static int chr_dev_open(struct inode *inode, struct file *file)
{
	if (device_open)
		return -EBUSY;

	device_open = 1;

	TRACE("dev_open\n");
	return 0;
}

/* OEMCrypto API called in user space */
/* Read to be called after write op to get return data from pmem */
static ssize_t chr_dev_read(struct file *fl,
	char __user *buf, size_t size, loff_t *off)
{
	if (device_state == VOEMCRYPTO_DEVICE_IDLE)
		return -EAGAIN;

	if (device_state == VOEMCRYPTO_DEVICE_BUSY)
		return -EBUSY;

	if (size > vrpc_maxsize(voemcrypto_data->vrpc))
		return -EINVAL;

	/* Data is waiting in pmem and waiting to be read */
	memcpy(buf+(*off), voemcrypto_data->vrpc_data, size);

	device_state = VOEMCRYPTO_DEVICE_IDLE;

	return size;
}


/* OEMCrypto API called in user space */
static ssize_t chr_dev_write(struct file *fl,
	const char __user *buf, size_t size, loff_t *off)
{
	size_t ret_size;
	enum voemcrypto_cmd_t cmd;

    /* read and parse data in buf */
	cmd = (enum voemcrypto_cmd_t) *(buf+(*off));

	if ((cmd >= VOEMCRYPTO_CMD_MAX) ||
		(device_state != VOEMCRYPTO_DEVICE_IDLE)) {
		/* invalid command code */
		return 0;
	}

	device_state = VOEMCRYPTO_DEVICE_BUSY;

	/* make vrpc call to backend in SecVM */
	/* vrpc call is blocked by mutex until return from secure VM*/
	ret_size = voemcrypto_call_ext(voemcrypto_data,
				(unsigned int *)(buf+(*off)), size);

	device_state = VOEMCRYPTO_DEVICE_WAITING_FOR_READ;

	return ret_size;
}


/**** Char Device creation (interface to Android space) *****/
static const struct file_operations voemcrypto_fops = {
	.owner	= THIS_MODULE,
	.open	= chr_dev_open,
	.read	= chr_dev_read,
	.write  = chr_dev_write,
};


/****** Basic VRPC call function *******/
/* Need to add support for multiple arguments */
/* Calls need to be atomic to prevent corrupting the shared mem */
vrpc_size_t
voemcrypto_call_ext(struct voemcrypto_t *voemcrypto, const unsigned int *arg,
	       const vrpc_size_t size0)
{
	struct vrpc_t *vrpc = voemcrypto->vrpc;
	/* pointer to default pmem owned by vlink */
	struct voemcrypto_req_t *req = voemcrypto->vrpc_data;
	vrpc_size_t	size;

	/* check for invalid size */
	if (size0 > sizeof(struct voemcrypto_req_t))
		return 0;

	/* Acquire mutex */
	if (mutex_lock_interruptible(&voemcrypto->call_mutex))
		return 0;

	DTRACE("vcmd %x arg %x\n", arg[0], arg[1]);

	/* copy cmd and parameters to shared mem. */
	memcpy(req, arg, size0);
	/* Up to 11 arguments in OEMCrypto interfaces. */
	size = size0;

	for (;;) {
		if (!vrpc_call(vrpc, &size))
			break;

		ETRACE("Lost backend. Closing and reopening.\n");
		vrpc_close(vrpc);
		if (vrpc_client_open(vrpc, 0, 0))
			BUG();

		TRACE("Re-established backend link.\n");
	}
	/* at this stage, vrpc_call has returned with data in pmem */
	/* copy return results and data back to user. */

	mutex_unlock(&voemcrypto->call_mutex);

	return size;
}


static void voemcrypto_ready(void *cookie)
{
	struct voemcrypto_t *voemc = (struct voemcrypto_t *)cookie;
	struct voemcrypto_req_t *req = voemc->vrpc_data;

	/* clear shared mem */
	memset(req, 0, sizeof(struct voemcrypto_req_t));
	TRACE("voemcrypto_ready.\n");

	return;
}


static int __init voemcrypto_init(void)
{
	int ret;
	struct voemcrypto_t *voemcrypto;
	struct vrpc_t *vrpc = vrpc_client_lookup(VOEMCRYPTO_VRPC_NAME, 0);

	TRACE("Initializing.\n");

	if (!vrpc) {
		ETRACE("No vrpc link.\n");
		return -ENODEV;
	}

	voemcrypto = kzalloc(sizeof(struct voemcrypto_t), GFP_KERNEL);
	if (!voemcrypto) {
		ETRACE("Out of memory.\n");
		return -ENOMEM;
	}

	mutex_init(&voemcrypto->call_mutex);

	voemcrypto->vrpc = vrpc;
	voemcrypto->vrpc_data = vrpc_data(vrpc);

	if (vrpc_maxsize(vrpc) < sizeof(struct voemcrypto_req_t)) {
		ETRACE("vrpc_maxsize() too small.\n");
		ret = -EINVAL;
		goto error;
	}

	/* store the new allocated struct for later use. */
	voemcrypto_data = voemcrypto;

	ret = vrpc_client_open(vrpc, voemcrypto_ready, voemcrypto);
	if (ret) {
		ETRACE("Could not open VRPC client (%d).\n", ret);
		goto error;
	}

	/* create char device */
	proc_file_entry = proc_create("voemcrypto", 0666,
			NULL, &voemcrypto_fops);

	if (proc_file_entry == NULL) {
		ETRACE("Could not create proc device.\n");
		ret = -ENOMEM;
		goto error;
	}

	TRACE("Initialized.\n");
	return 0;
error:
	kfree(voemcrypto);
	return ret;
}

#if 0
static void __exit voemcrypto_exit(void)
{
	vrpc_close(voemcrypto_data->vrpc);
	vrpc_release(voemcrypto_data->vrpc);

	kfree(voemcrypto_data);
	if (proc_file_entry)
		proc_remove(proc_file_entry);
}
#endif

device_initcall(voemcrypto_init);
