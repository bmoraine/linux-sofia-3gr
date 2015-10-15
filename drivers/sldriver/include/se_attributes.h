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

#ifndef _SE_ATTRIBUTES_H_
#define _SE_ATTRIBUTES_H_

/*#include <stdint.h>*/

/* Enclave Flags Bit Masks*/
	/* if set, then the enclave is initialized*/
#define SE_FLAGS_INITTED        0x0000000000000001ULL
	/* if set, then the enclave is debug*/
#define SE_FLAGS_DEBUG          0x0000000000000002ULL
	/* if set, then the enclave is 64 bit*/
#define SE_FLAGS_MODE64BIT      0x0000000000000004ULL
	/* if set, then the enclave has access to provision key*/
#define SE_FLAGS_PROVISION_KEY  0x0000000000000010ULL
	/* if set, then the enclave has access to License key*/
#define SE_FLAGS_LICENSE_KEY    0x0000000000000020ULL
#define SE_FLAGS_RESERVED       (~(SE_FLAGS_INITTED | \
				SE_FLAGS_DEBUG | \
				SE_FLAGS_MODE64BIT | \
				SE_FLAGS_PROVISION_KEY| \
				SE_FLAGS_LICENSE_KEY))

/* XSAVE Feature Request Mask*/
#define SE_XFRM_LEGACY          0x0000000000000003ULL
#define SE_XFRM_AVX             0x0000000000000006ULL
#define SE_XFRM_AVX3            0x00000000000000E6ULL
#define SE_XFRM_LWP             0x4000000000000000ULL
#define SE_XFRM_MPX             0x0000000000000018ULL
#define SE_XFRM_RESERVED        (~(SE_XFRM_LEGACY | \
				SE_XFRM_AVX | \
				SE_XFRM_AVX3 | \
				SE_XFRM_MPX))

typedef struct _se_attributes_t {
	uint64_t flags;
	uint64_t xfrm;
} se_attributes_t;

#endif /*_SE_ATTRIBUTES_H_*/
