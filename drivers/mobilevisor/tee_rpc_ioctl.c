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

#include <linux/compat.h>
#include <linux/compiler.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uio.h>

#include "tee_rpc_driver.h"
#include "tee_rpc_ioctl.h"
#include "tee_rpc_driver_abi.h"
#include "tee_rpc_memory.h"
#include "sec_rpc.h"

typedef long rpc_ioctl_t(struct file *, unsigned long);


static long get_ver_major(struct file *filp, unsigned long arg)
{
	u32 __user *ver_p = (u32 __user *)arg;
	const u32 ver_major = TEE_RPC_VER_MAJOR;

	return put_user(ver_major, ver_p);
}

static long get_ver_minor(struct file *filp, unsigned long arg)
{
	u32 __user *ver_p = (u32 __user *)arg;
	const u32 ver_minor = TEE_RPC_VER_MINOR;

	return put_user(ver_minor, ver_p);
}

static long perform_rpc(struct file *flip, unsigned long arg)
{
	struct tee_message __user *user_msg = (struct tee_message __user *)arg;
	struct pvec *phys_iov = NULL;
	int ret = 0;
	int remote_ret;
	int num_vectors;
	u32 service;
	int cmd_id;
	u32 size;
	struct t_rpc_send_info s_info = {VSEC_VM_LINUX, VSEC_VM_SECURE, "vsec"};

	/* map the iov_base part of the iovec into memory that can be used in
	 * the kernel
	 */
	ret = tee_rpc_map_user_pages(user_msg, &num_vectors, &phys_iov,
				     &service, &cmd_id);
	if (ret)
		return ret;

#ifdef MEM_TEST_STUB
	ret = stub_call_sec_vm(num_vectors, phys_iov, &remote_ret);
	if (ret)
		goto cleanup_rpc_call;
#else

	size = sizeof(struct pvec) * num_vectors;

	ret = rpc_call(&s_info, service, cmd_id, (u8 *)phys_iov, &size, size);
	if (ret)
		goto cleanup_rpc_call;
	remote_ret = 0;
#endif

	ret = tee_rpc_unmap_user_pages(num_vectors, phys_iov);
	if (ret)
		goto cleanup_rpc_call;

	put_user(remote_ret, &(user_msg->remote_return));

cleanup_rpc_call:
	free_pvec(num_vectors, phys_iov);

	/*TODO CLEANUP ALL THE ALLOCATED MEMORY !!!! */
	return ret;
}

static rpc_ioctl_t *rpc_ioctls[] = {
	/* Version info. commands */
	[TEE_RPC_IOC_NR_GET_VER_MAJOR] = get_ver_major,
	[TEE_RPC_IOC_NR_GET_VER_MINOR] = get_ver_minor,
	[TEE_RPC_IOC_NR_CALL] = perform_rpc,
};

long tee_rpc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int nr = _IOC_NR(cmd);
	rpc_ioctl_t *callback = NULL;
	int ret = -ENOTTY;

	pr_debug("Calling the IOCTL %d\n", nr);

	/* Verify IOCTL command: magic */
	if (_IOC_TYPE(cmd) != TEE_IOC_MAGIC) {
		pr_err("Invalid IOCTL type=%u", _IOC_TYPE(cmd));
		return -ENOTTY;
	}

	/* Verify permissions on parameters pointer (arg) */
	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(ACCESS_WRITE,
				 (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(ACCESS_READ,
				 (void __user *)arg, _IOC_SIZE(cmd));
	if (ret)
		return -EFAULT;

	/* Find the correct handler for the ioctl */
	if (nr < ARRAY_SIZE(rpc_ioctls))
		callback = rpc_ioctls[nr];

	if (callback != NULL) {
		ret = (*callback) (filp, arg);
	} else {
		pr_err("Unknown command : %u", nr);
		ret = -EINVAL;
	}

	return ret;
}

static rpc_ioctl_t *rpc_compat_ioctls[] = {
	/* Version info. commands */
	[TEE_RPC_IOC_NR_GET_VER_MAJOR] = get_ver_major,
	[TEE_RPC_IOC_NR_GET_VER_MINOR] = get_ver_minor,
};

long tee_rpc_compat_ioctl(struct file *filp, unsigned int cmd,
			  unsigned long arg)
{
	unsigned int nr = _IOC_NR(cmd);
	rpc_ioctl_t *callback = NULL;
	int ret = -EINVAL;

	pr_debug("Calling the IOCTL compat %d\n", nr);

	if (nr < ARRAY_SIZE(rpc_compat_ioctls))
		callback = rpc_compat_ioctls[nr];

	if (callback != NULL)
		ret = (*callback) (filp, arg);

	return ret;
}
