/*
 * Intel RPMB access dispatcher code
 * Copyright (c) 2015, Intel Corporation.
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
#define pr_fmt(fmt) "rpmb_rpc: " fmt

#include <linux/errno.h>
#include <linux/types.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/rpmb.h>

#include "rpmb_rpc.h"
#include "sec_rpc.h"
#include "tee_rpc_driver_abi.h"

enum SEC_RPC_STATUS {
	SEC_RPC_SUCCESS = 0x0,
	SEC_RPC_NULL_PTR = 0x1,
	SEC_RPC_BUFFER_TOO_SMALL = 0x2,
	SEC_RPC_INVALID_PARAMS = 0x3,
	SEC_RPC_OS_CALL_FAILED = 0x4,
	SEC_RPC_UNKNOWN_OPCODE = 0x5,
	SEC_RPC_OUT_OF_RESOURCES = 0x6,
	SEC_RPC_OPERATION_NOT_SUPPORTED = 0x7,
	SEC_RPC_GENERAL_FAILURE = 0x8
};

struct sec_rpc_request {
	void *buffer;
	u32 buffer_size;
} __packed;

struct sec_rpc_response {
	enum SEC_RPC_STATUS status;
	u32 written_size;
} __packed;

enum RPMB_OPCODE {
	RPMB_OPCODE_HOST_START    = 0x0,    /* Host to SecVM */
	RPMB_OPCODE_SECURE_WRITE  = 0x1,    /* SecVM to Host */
	RPMB_OPCODE_SECURE_READ   = 0x2,    /* SecVM to Host */
	RPMB_OPCODE_PROVISION     = 0x3,    /* SecVM to Host */
	RPMB_OPCODE_GET_COUNTER   = 0x4,    /* SecVM to Host */
	RPMB_OPCODE_HOST_STOP     = 0x5,    /* Host to SecVM */

	RPMB_OPCODE_DBG_PROVISION = 0x1000, /* Host to SecVM */
	RPMB_OPCODE_DBG_RUN_UT    = 0x1001, /* Host to SecVM */

	RPMB_OPCODE_MAX = 0x7FFFFFFF
};

struct RPMB_REQUEST {
	u32 Reserved1[4];

	union {
		struct {
			u32 CountToWrite;
			u32 Reserved2[3];
			struct rpmb_frame ResultFrame;
			struct rpmb_frame FrameDataToWrite[0];
		} RPMB_WRITE_IO;

		struct {
			u32 CountToRead;
			u32 Reserved2[3];
			struct rpmb_frame RequestFrame;
			struct rpmb_frame ReturnedFrameData[0];

		} RPMB_READ_IO;

		struct {
			u32 Reserved2[4];
			struct rpmb_frame ResultFrame;
			struct rpmb_frame ProgramKeyFrame[0];
		} RPMB_PROGRAM_KEY_IO;

		struct {
			u32 Reserved2[4];
			struct rpmb_frame ResultFrame;
			struct rpmb_frame GetCounterFrame[0];
		} RPMB_GET_COUNTER_IO;
	};
};

#define RPMB_BUF_SIZE 4096
static struct RPMB_REQUEST *rpc_rpmb_req;
static struct rpmb_dev *rpc_rpmb_dev;
static DEFINE_MUTEX(rpc_rpmb_mutex);

/**
 * rpc_rpmb_dispatch - process request for RPMB operations
 *
 * @opcode: operation code
 * @io_data: pointer to IO buffer
 * @io_data_len: pointer to size of incoming and outgoing command
 *
 * Return: 0 if call processed, -1 otherwise
 */
int rpc_rpmb_dispatch(u32 opcode, u8 *io_data, u32 *io_data_len)
{
	struct sec_rpc_response *rsp = (struct sec_rpc_response *)io_data;
	enum SEC_RPC_STATUS status;
	struct rpmb_data data;
	int rpc_result = -1;
	int ret;

	if (io_data == NULL || io_data_len == NULL) {
		pr_err("Invalid input: data(0x%08X), data_len(0x%08X)\n",
			 (u32)io_data, (u32)io_data_len);
		goto cleanup;
	}
	if (*io_data_len < sizeof(struct sec_rpc_response)) {
		pr_err("io_data_len(%u) < sec_rpc_response(%zu)\n",
			*io_data_len, sizeof(struct sec_rpc_response));
		goto cleanup;
	}
	*io_data_len = sizeof(struct sec_rpc_response);

	mutex_lock(&rpc_rpmb_mutex);
	if (WARN_ON(!rpc_rpmb_req)) {
		pr_err("Not initialised\n");
		status = SEC_RPC_GENERAL_FAILURE;
		goto reply;
	}

	if (WARN_ON(!rpc_rpmb_dev)) {
		pr_err("RPMB device not found\n");
		status = SEC_RPC_GENERAL_FAILURE;
		goto reply;
	}

	switch (opcode) {
	case RPMB_OPCODE_SECURE_WRITE:
		data.req_type = RPMB_WRITE_DATA;
		data.block_count =
			rpc_rpmb_req->RPMB_WRITE_IO.CountToWrite;

		data.in_frames_cnt =
			rpc_rpmb_req->RPMB_WRITE_IO.CountToWrite;
		data.in_frames =
			rpc_rpmb_req->RPMB_WRITE_IO.FrameDataToWrite;
		data.out_frames_cnt = 1;
		data.out_frames =
			&rpc_rpmb_req->RPMB_WRITE_IO.ResultFrame;
		break;

	case RPMB_OPCODE_SECURE_READ:
		data.req_type = RPMB_READ_DATA;
		data.block_count =
			rpc_rpmb_req->RPMB_READ_IO.CountToRead;

		data.in_frames_cnt = 1;
		data.in_frames =
			&rpc_rpmb_req->RPMB_READ_IO.RequestFrame;
		data.out_frames_cnt =
			rpc_rpmb_req->RPMB_READ_IO.CountToRead;
		data.out_frames =
			rpc_rpmb_req->RPMB_READ_IO.ReturnedFrameData;
		break;

	case RPMB_OPCODE_GET_COUNTER:
		data.req_type = RPMB_GET_WRITE_COUNTER;
		data.block_count = 1;

		data.in_frames_cnt = 1;
		data.in_frames =
			rpc_rpmb_req->RPMB_GET_COUNTER_IO.GetCounterFrame;
		data.out_frames_cnt = 1;
		data.out_frames =
			&rpc_rpmb_req->RPMB_GET_COUNTER_IO.ResultFrame;
		break;

	case RPMB_OPCODE_PROVISION:
		pr_err("Opcode not supported: %d\n", opcode);
		status = SEC_RPC_OPERATION_NOT_SUPPORTED;
		goto reply;

	default:
		pr_err("Invalid opcode: %d\n", opcode);
		status = SEC_RPC_UNKNOWN_OPCODE;
		goto reply;
	}

	ret = rpmb_send_req(rpc_rpmb_dev, &data);
	if (ret) {
		pr_err("RPMB request failed ret = %d\n", ret);
		status = SEC_RPC_OS_CALL_FAILED;
		goto reply;
	}
	status = SEC_RPC_SUCCESS;

reply:
	mutex_unlock(&rpc_rpmb_mutex);
	pr_debug("Have reply with status: %d\n", status);
	rsp->status = status;
	rsp->written_size = 0;

	rpc_result = 0;

cleanup:
	return rpc_result;
}

static int rpc_rpmb_send_rpc_req(struct rpmb_dev *dev,
				     u32 opcode,
				     struct sec_rpc_request *req)
{
	struct t_rpc_send_info s_info = {
		VSEC_VM_LINUX,
		VSEC_VM_SECURE,
		"vsec" };
	u32 size = sizeof(struct sec_rpc_request);
	int ret;

	ret = rpc_call(&s_info, RPC_IF_RPMB, opcode,
		(u8 *)req, &size, size);
	if (ret)
		pr_err("Failed rpc_call ret = %d\n", ret);

	return ret;
}

static int rpc_rpmb_stop(struct rpmb_dev *dev)
{
	struct sec_rpc_request req;

	mutex_lock(&rpc_rpmb_mutex);
	if (WARN_ON(!rpc_rpmb_dev)) {
		pr_err("Not started\n");
		mutex_unlock(&rpc_rpmb_mutex);
		return -EFAULT;
	}
	if (dev != rpc_rpmb_dev) {
		pr_err("Wrong RPMB\n");
		mutex_unlock(&rpc_rpmb_mutex);
		return -EFAULT;
	}
	rpmb_dev_put(rpc_rpmb_dev);
	rpc_rpmb_dev = NULL;
	mutex_unlock(&rpc_rpmb_mutex);

	req.buffer = NULL;
	req.buffer_size = 0;
	return rpc_rpmb_send_rpc_req(dev, RPMB_OPCODE_HOST_STOP, &req);
}

static int rpc_rpmb_start(struct rpmb_dev *dev)
{
	struct sec_rpc_request req;

	mutex_lock(&rpc_rpmb_mutex);
	if (WARN_ON(!rpc_rpmb_req)) {
		pr_err("Not initialised\n");
		mutex_unlock(&rpc_rpmb_mutex);
		return -EFAULT;
	}

	if (WARN_ON(rpc_rpmb_dev)) {
		pr_err("Already have RPMB device\n");
		mutex_unlock(&rpc_rpmb_mutex);
		return -EFAULT;
	}
	rpc_rpmb_dev = rpmb_dev_get(dev);
	req.buffer = (void *)virt_to_phys(rpc_rpmb_req);
	req.buffer_size = RPMB_BUF_SIZE;
	mutex_unlock(&rpc_rpmb_mutex);

	return rpc_rpmb_send_rpc_req(dev, RPMB_OPCODE_HOST_START, &req);
}

static int rpc_rpmb_notify(struct notifier_block *nb,
				unsigned long val, void *bd)
{
	struct rpmb_dev *dev = (struct rpmb_dev *)bd;

	/* FIXME: verify identity */

	switch (val) {
	case RPMB_PART_ADD:
		pr_info("rpmb partition created\n");
		rpc_rpmb_start(dev);
		break;
	case RPMB_PART_REMOVE:
		pr_info("rpmb partition removed\n");
		rpc_rpmb_stop(dev);
		break;
	case RPMB_PART_SUSPEND:
		pr_info("rpmb partition suspended\n");
		break;
	case RPMB_PART_RESUME:
		pr_info("rpmb partition resumed\n");
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block rpmb_nb = {
	.notifier_call = rpc_rpmb_notify,
	.priority = 0,
};


/**
 * rpc_rpmb_init - RPMB processor initialization routine
 *
 * Return: 0 on sucess, <0 otherwise
 */
int rpc_rpmb_init(void)
{
	int ret;

	mutex_lock(&rpc_rpmb_mutex);
	rpc_rpmb_req = kmalloc(RPMB_BUF_SIZE, GFP_KERNEL);
	if (!rpc_rpmb_req) {
		ret = -ENOMEM;
		goto out;
	}

	ret = rpmb_register_notify(&rpmb_nb);

out:
	if (ret) {
		kfree(rpc_rpmb_req);
		rpc_rpmb_req = NULL;
	}
	mutex_unlock(&rpc_rpmb_mutex);
	return ret;
}

/**
 * rpc_rpmb_release - RPMB processor release routine
 *
 * Return: always 0
 */
int rpc_rpmb_release(void)
{
	rpmb_unregister_notify(&rpmb_nb);

	mutex_lock(&rpc_rpmb_mutex);
	kfree(rpc_rpmb_req);
	rpc_rpmb_req = NULL;
	mutex_unlock(&rpc_rpmb_mutex);
	return 0;
}
