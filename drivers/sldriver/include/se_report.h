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

#ifndef _SE_REPORT_H_
#define _SE_REPORT_H_

#include "se_attributes.h"
#include "se_key.h"

/* SHA256 */
#define SE_HASH_SIZE        32
/* Message Authentication Code - 16 bytes */
#define SE_MAC_SIZE         16

#define REPORT_DATA_SIZE    64

typedef uint8_t se_measurement_t[SE_HASH_SIZE];
typedef uint8_t se_mac_t[SE_MAC_SIZE];
typedef uint8_t se_report_data_t[REPORT_DATA_SIZE];
typedef uint16_t se_prod_id_t;

typedef struct _targe_info_t {
	/* (  0) The MRENCLAVE of the target enclave */
	se_measurement_t mr_enclave;
	/* ( 32) The ATTRIBUTES field of the target enclave */
	se_attributes_t attributes;
} se_target_info_t;

typedef struct _report_body_t {
	/* (  0) Security Version of the CPU */
	se_cpu_svn_t cpu_svn;
	/* ( 16) */
	uint8_t reserved1[32];
	/* ( 48) Any special Capabilities the Enclave possess */
	se_attributes_t attributes;
	/* ( 64) The value of the enclave's ENCLAVE measurement */
	se_measurement_t mr_enclave;
	/* ( 96) */
	uint8_t reserved2[32];
	/* (128) The value of the enclave's SIGNER measurement */
	se_measurement_t mr_signer;
	/* (160) */
	uint8_t reserved3[32];
	/* (192) Reserved measurement for future use */
	se_measurement_t mr_reserved1;
	/* (224) Reserved measurement for future use */
	se_measurement_t mr_reserved2;
	/* (256) Product ID of the Enclave */
	se_prod_id_t isv_prod_id;
	/* (258) Security Version of the Enclave */
	se_isv_svn_t isv_svn;
	/* (260) Must be zero */
	uint8_t reserved4[60];
	/* (320) Data provided by the user */
	se_report_data_t report_data;
} se_report_body_t;

typedef struct _report_t {
	/* 432 bytes */
	se_report_body_t body;
	/* (384) KeyID used for diversifying the key tree */
	se_key_id_t key_id;
	/* (416) The Message Authentication Code over this structure. */
	se_mac_t mac;
} se_report_t;

#endif
