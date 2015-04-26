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

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "tee_rpc_ioctl.h"
#include "tee_rpc_driver.h"
#include "vsec.h"
#include "rpmb_rpc.h"


static int tee_major;		/* default to dynamic major */
#define TEE_RPC_MAX_DEVS 1      /* The number of devices that we create */

static struct class *tee_rpc_class;
static struct tee_rpc_client_ctx *client_contexts;

static int tee_rpc_open(struct inode *node, struct file *flip)
{
	struct tee_rpc_client_ctx *dev;

	pr_err("opening device\n");

	dev = container_of(node->i_cdev, struct tee_rpc_client_ctx, cdev);

	flip->private_data = dev;

	return 0;
}

static int tee_rpc_release(struct inode *node, struct file *flip)
{
	return 0;
}

static ssize_t tee_rpc_read(struct file *flip, char __user *ubuf,
			    size_t size, loff_t *offset)
{
	static const char output[] = "Reading the test output!\n";

	pr_err("Writing something from open\n");
	if (output[*offset] == '\0') {
		pr_err("End of string, returning zero.\n");
		return 0;
	}

	copy_to_user(ubuf, &output[*offset], 1);
	*offset += 1;
	return 1;  /* return a single char */
}

static ssize_t tee_rpc_write(struct file *flip, const char __user *ubuf,
			     size_t size, loff_t *offset)
{
	return 0;
}

static const struct file_operations tee_rpc_fops = {
	.owner = THIS_MODULE,
	.open = tee_rpc_open,
	.release = tee_rpc_release,
	.read = tee_rpc_read,
	.write = tee_rpc_write,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tee_rpc_compat_ioctl,
#endif
	.unlocked_ioctl = tee_rpc_ioctl,
};

static void tee_rpc_cleanup(void)
{
	int i;

	rpc_rpmb_release();

	if (client_contexts) {
		for (i = 0; i < TEE_RPC_MAX_DEVS; i++)
			cdev_del(&client_contexts[i].cdev);

		kfree(client_contexts);
		client_contexts = NULL;
	}

	unregister_chrdev_region(MKDEV(tee_major, 0), TEE_RPC_MAX_DEVS);
}

static int __init tee_rpc_init(void)
{
	int ret;
	dev_t major;
	int i;

	tee_rpc_class = class_create(THIS_MODULE, "tee_rpc_ctl");

	ret = alloc_chrdev_region(&major, 0, TEE_RPC_MAX_DEVS, "tee_rpc");
	if (ret < 0) {
		pr_err("allocate_chrdev_region failed for tee_rpc\n");
		goto out_err;
	}

	tee_major = MAJOR(major);

	client_contexts = kzalloc(TEE_RPC_MAX_DEVS *
				  sizeof(struct tee_rpc_client_ctx),
				  GFP_KERNEL);
	if (!client_contexts) {
		ret = -ENOMEM;
		goto out_err;
	}

	for (i = 0; i < TEE_RPC_MAX_DEVS; i++) {

		cdev_init(&client_contexts[i].cdev, &tee_rpc_fops);
		client_contexts[i].cdev.owner = THIS_MODULE;
		client_contexts[i].cdev.ops = &tee_rpc_fops;

		major = MKDEV(tee_major, i);
		ret = cdev_add(&client_contexts[i].cdev, major, i + 1);
		if (ret) {
			pr_err("Failed to register device, minor: %d\n", i);
			goto out_err;
		}

		device_create(tee_rpc_class, NULL, major, NULL, "tee_rpc");
	}

	pr_err("TEE_RPC REGISTERED major : %d\n", tee_major);

	/* initialize the underlying vsec communication mechanism */
	if (!vsec_init())
		goto out_err;

	rpc_rpmb_init();

	return 0;

out_err:
	tee_rpc_cleanup();
	pr_err("TEE_RPC Failed to register\n");
	return ret;
}

static void __exit tee_rpc_exit(void) /* Destructor */
{
	tee_rpc_cleanup();
	pr_info("TEE RPC unregistered");
}

module_init(tee_rpc_init);
module_exit(tee_rpc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Brian McGillion <brian.mcgillion@intel.com>");
MODULE_DESCRIPTION("TEE RPC driver for Sofia sec_vm");
MODULE_VERSION("0.1");
