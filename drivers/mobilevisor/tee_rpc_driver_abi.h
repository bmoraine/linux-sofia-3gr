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

#ifndef __TEE_RPC_DRIVER_ABI_H__
#define __TEE_RPC_DRIVER_ABI_H__

#define TEE_RPC_VER_MAJOR 0
#define TEE_RPC_VER_MINOR 1

/*!
 * \brief A message constuct that contains the contents that are being conveyed
 * to the TEE.
 */
struct tee_message {
	struct iovec *iov; /*!< The data vectors that are being transfered */
	uint32_t service;  /*!< The service to which the message is sent*/
	int command_id; /*!< The command that is being invoked */
	int num_vectors; /*!< The number of vecotors in iov */
	int remote_return;   /*!< The return value of the remote procedure */
};

/* This magic number appears free in Documentation/ioctl/ioctl-number.txt */
#define TEE_IOC_MAGIC 0xBB

/* IOCTL ordinal numbers */
/*(for backward compatibility, add new ones only at end of list!) */
enum rpc_ioc_nr {
	/* Version info. commands */
	TEE_RPC_IOC_NR_GET_VER_MAJOR = 0,
	TEE_RPC_IOC_NR_GET_VER_MINOR = 1,
	/* Context size queries */
	TEE_RPC_IOC_NR_CALL = 2,
};

/******************************/
/* IOCTL commands definitions */
/******************************/
/* Version info. commands */
#define TEE_RPC_IOC_GET_VER_MAJOR _IOR(TEE_IOC_MAGIC,			\
				       TEE_RPC_IOC_NR_GET_VER_MAJOR, u32)
#define TEE_RPC_IOC_GET_VER_MINOR _IOR(TEE_IOC_MAGIC,			\
				       TEE_RPC_IOC_NR_GET_VER_MINOR, u32)

/* Define a unique IOCTL command for the rpc call for reading and writing */
#define TEE_RPC_IOC_CALL _IOWR(TEE_IOC_MAGIC,				\
			       TEE_RPC_IOC_NR_CALL, struct tee_message)

#endif
