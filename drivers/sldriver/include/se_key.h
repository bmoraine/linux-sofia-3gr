/*
 * Copyright (c) 2015, Intel Corporation
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

#pragma once

#ifndef _SE_KEY_H_
#define _SE_KEY_H_

/*#include <stdint.h>*/
#include "se_attributes.h"

/* Key Name*/
#define KEYSELECT_LICENSE          0x0000
#define KEYSELECT_PROVISION        0x0001
#define KEYSELECT_PROVISION_SEAL   0x0002
#define KEYSELECT_REPORT           0x0003
#define KEYSELECT_SEAL             0x0004

/* Key Policy*/
/* Derive key using the enclave's ENCLAVE measurement register */
#define KEYPOLICY_MRENCLAVE        0x0001
/* Derive key using the enclave's SINGER measurement register */
#define KEYPOLICY_MRSIGNER         0x0002

#define KEY_ID_SIZE                8

#define CPU_SVN_SIZE               16

typedef uint8_t se_key_128bit_t[16];
typedef uint32_t se_key_id_t[KEY_ID_SIZE];
typedef uint8_t se_cpu_svn_t[CPU_SVN_SIZE];
typedef uint16_t se_isv_svn_t;

typedef struct _key_request_t {
	/* 000 Identifies the key required */
	uint16_t key_name;
	/* 002 Identifies which inputes should be used in thekey derivation */
	uint16_t key_policy;
	/* 004 Security Version of the Enclave */
	se_isv_svn_t isv_svn;
	/* 006 Must be 0 */
	uint16_t reserved;
	/* 008 Security Version of the CPU */
	se_cpu_svn_t cpu_svn;
	/* 024 Mask which ATTRIBUTES Seal keys should be bound to */
	se_attributes_t attribute_mask;
	/* 040 Value for key wear-out protection */
	se_key_id_t key_id;
} se_key_request_t;

#endif
