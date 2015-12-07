/*
 * The file implements ioctl interface for vIDT driver.
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
 */
#ifndef __VIDT_IOCTL__
#define __VIDT_IOCTL__

#include <linux/ioctl.h>
#include "sl_types.h"

struct entry_pid_viewid {
	unsigned long long int pid;
	unsigned long long int viewid;
};

struct enclave_create_param {
	uint64_t size;
	uint64_t base;
	uint32_t flags;
	uint32_t view_handle;
	uint64_t alloc_size;
} __packed;

struct enclave_add_param {
	uint64_t user_page;
	uint64_t rva;
	uint64_t flags;
	uint32_t view_handle;
	uint32_t type;
	uint32_t attr;
} __packed;

struct enclave_init_param {
	uint32_t view_handle;
	enclave_css_t enclave_css;
} __packed;

typedef struct ta_property_t {
	uint8_t uuid[16];
	uint8_t pub_key[SE_KEY_SIZE];
	uint16_t svn;
	uint8_t type;
} __packed ta_property_t;

typedef struct view_prop_t {
	ta_property_t prop;
	uint64_t view;
} __packed view_prop_t;

struct shm_map_info {
	uint64_t sharing_view;
	uint64_t shm_hnd;
	uint64_t start_addr;
	uint64_t size;
	uint64_t view_list;
	uint32_t num_entries;
} __packed;

typedef enum {
	MEM_TYPE_SHARED_UNTRUSTED = 0,
	MEM_TYPE_SHARED_TRUSTED = 1,
	MEM_TYPE_MALLOC_TRUSTED = 2,
	MEM_TYPE_UNSHARE_UNTRUSTED = 3,
	MEM_TYPE_UNSHARE_TRUSTED = 4,
	MEM_TYPE_FREE_TRUSTED = 5,
	MEM_TYPE_OWNED_EXCL = 6,
	MEM_TYPE_INVALID = 7,
} sl_mem_type_t;

typedef struct mem_area_t {
	uint64_t addr;
	uint64_t size;
} __packed range_t;

struct enclave_code_page_param {
	uint64_t code_page_addr;
	uint64_t view_id;
} __packed;

typedef struct vmm_view_info {
	uint64_t enclave_id;
	uint64_t vmm_view;
} vmm_view_info_t;

typedef struct vmm_view_type {
	uint32_t view_id;
	uint32_t ta_type;
} vmm_view_type_t;

typedef enum perm_t {
	NO_RIGHTS = 0,
	R_RIGHTS = 1,
	RW_RIGHTS = 3,
	X_RIGHTS = 4,
	RX_RIGHTS = 5,
	RWX_RIGHTS = 7,
	ALL_RIGHTS = 7
} sl_mem_permission_t;

typedef struct sl_view_info_t {
	uint64_t hnd;
	sl_mem_permission_t perms;
} __packed sl_view_info_t;

typedef struct view_list_array {
	sl_view_info_t *array;
	size_t size;
} sl_view_list_t;

typedef enum {
	no_rights = 0,
	r_rights = 1,
	w_rights = 2,
	x_rights = 4,
	rw_rights = 3,
	rx_rights = 5,
	rwx_rights = 7,
	all_rights = 7
} perm_t;

typedef struct view_list_entry {
	uint64_t hnd;
	perm_t perms;
} __packed v_node;

typedef struct mem_info {
	uint64_t start_addr;
	uint64_t size;
	sl_mem_type_t type;
	uint64_t view_list;
	uint32_t num_entries;
	uint64_t shm_hnd;
	range_t faulting_range;
} __packed sl_mem_info_t;

typedef struct vIDT_sign {
	uint64_t sign_blob;	/* again a pointer keeping as 64 bit */
	uint32_t sign_size;
} __packed vIDT_sign_t;

struct vmm_keepalive_info {
	uint32_t view_id;
	uint32_t ta_state;
};

#define VIDT_MAGIC_NUMBER   255

#define VIDT_PID_VIEW_MAP       _IOW(VIDT_MAGIC_NUMBER, 0, struct entry_pid_viewid)
#define VIDT_PID_VIEW_UNMAP     _IOW(VIDT_MAGIC_NUMBER, 1, struct entry_pid_viewid)
#define VIDT_REGISTER           _IO(VIDT_MAGIC_NUMBER, 2)
#define VIDT_REGISTER_HP_PID    _IOW(VIDT_MAGIC_NUMBER, 3, pid_t)
#define VIDT_CREATE_VIEW        _IOW(VIDT_MAGIC_NUMBER, 4, struct enclave_create_param)
#define VIDT_ADD_PAGE           _IOW(VIDT_MAGIC_NUMBER, 5, struct enclave_add_param)
#define VIDT_INIT_VIEW          _IOW(VIDT_MAGIC_NUMBER, 6, struct enclave_init_param)
#define VIDT_GET_TA_PROPERTIES  _IOW(VIDT_MAGIC_NUMBER, 7, struct view_prop_t)
#define VIDT_VERIFY_STATUS      _IO(VIDT_MAGIC_NUMBER, 8)
#define VIDT_MAP_SHM            _IOW(VIDT_MAGIC_NUMBER, 9, struct shm_map_info)
#define VIDT_CHANGE_MEM_PERM    _IOW(VIDT_MAGIC_NUMBER, 10, struct mem_info)
#define VIDT_GET_VMM_VIEWID     _IOW(VIDT_MAGIC_NUMBER, 11, struct vmm_view_info)
#define VIDT_UPDATE_PERM        _IOW(VIDT_MAGIC_NUMBER, 12, struct enclave_code_page_param)
#define VIDT_GET_INFO           _IOW(VIDT_MAGIC_NUMBER, 13, sl_info_t)
#define VIDT_REMOVE_VIEW        _IOW(VIDT_MAGIC_NUMBER, 14, uint64_t)
#define VIDT_GET_AVAIL_HEAP     _IOW(VIDT_MAGIC_NUMBER, 15, uint64_t)
#define VIDT_GET_TA_TYPE        _IOW(VIDT_MAGIC_NUMBER, 16, struct vmm_view_type)
#define VIDT_VERIFY             _IOW(VIDT_MAGIC_NUMBER, 17, struct vIDT_sign)
#define VIDT_GET_SL_AFFINITY    _IOW(VIDT_MAGIC_NUMBER, 18, uint32_t)
#define VIDT_ACTIVATE_KEEPALIVE_VIEW _IOW(VIDT_MAGIC_NUMBER, 19, \
					  struct vmm_keepalive_info)

#endif
