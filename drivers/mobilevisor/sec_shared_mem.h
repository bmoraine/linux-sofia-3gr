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

#ifndef __SEC_SHARED_MEM_DEFINED_H__
#define __SEC_SHARED_MEM_DEFINED_H__

extern void *sec_shared_mem_alloc(unsigned int size);
extern void sec_shared_mem_free(void *ptr);

#define SEC_SHARED_MEM_ALLOC_ENABLED

#endif /*__SEC_SHARED_MEM_DEFINED_H__*/

