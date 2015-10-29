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

#ifndef VMM_HSYM_H__
#define VMM_HSYM_H__

#include "vmm_hsym_common.h"

#define ALIGN_DATA_SECT(x) __attribute__((aligned (4096), section (x)))
#define ALIGN_CODE_SECT(x) __attribute__((aligned (4096), section (x), noinline))

extern int verbose;

enum {
	CONFIG_MAX_CPUS = 64,
	EXPECTED_API_VERSION = 3,
	EXPECTED_API_REVISION = 0,
};

/* hypersec interface */
typedef enum perf_type_t {
	SL_CMD_HSEC_GET_INFO = 0x56583264,
	SL_CMD_HSEC_CONFIG = 0x56583265,
	SL_CMD_HSEC_START = 0x56583266,
	SL_CMD_HSEC_STOP = 0x56583267,
	SL_CMD_HSEC_REG_SECT = 0x56583268,
	SL_CMD_HSEC_CREATE_VIEW = 0x56583269,
	SL_CMD_HSEC_REMOVE_VIEW = 0x5658326A,
	SL_CMD_HSEC_ADD_PAGE = 0x5658326B,
	SL_CMD_HSEC_INIT_VIEW = 0x5658326C,
	SL_CMD_HSEC_CHANGE_MEM_PERM = 0x5658326D,
	SL_CMD_HSEC_CHK_ACCESS_RIGHT = 0x5658326E,
	SL_CMD_HSEC_REG_VIDT = 0x5658326F,
	SL_CMD_HSEC_REG_SL_INFO = 0x56583270,
	SL_CMD_HSEC_GET_CURR_VIEW = 0x56583271,
	SL_CMD_HSEC_UPDATE_PERM = 0x56583272,
	SL_CMD_HSEC_UUID = 0x56583273,
	SL_CMD_HSEC_MAP_SHM = 0x56583274,
	SL_CMD_HSEC_VIDT_VERIFY_STATUS = 0x56583275,
	SL_CMD_HSEC_GET_DEBUG_STATUS = 0x56583276,
	SL_CMD_HSEC_VERIFY_VIDT = 0x56583277,
	SL_CMD_HSEC_UNMAP_SHM = 0x56583278,
	SL_CMD_HSEC_GET_UUID_INSTANCE_COUNT = 0x56583279,
	SL_CMD_HSEC_GET_TA_PROPERTIES = 0x56583280,
	SL_CMD_HSEC_PSTA_GET_BOOT_INFO = 0x56583281,
	SL_CMD_HSEC_GET_VMM_VIEWID = 0x56583282,
	SL_CMD_HSEC_GET_AVAIL_HEAP = 0x56583283,
	SL_CMD_HSEC_GET_VIEW_STATS = 0x56583284,
	SL_CMD_HSEC_GET_VMEXIT_COUNT = 0x56583285,
	SL_CMD_HSEC_SET_KEEP_ALIVE = 0x56583286,
	SL_CMD_HSEC_GET_TA_TYPE = 0x56583287,
	SL_CMD_HSEC_ADD_TO_LOCK_LIST = 0x56583288,
	SL_CMD_HSEC_ADD_PAGE_TO_KERNTA = 0x5658328A,
	SL_CMD_HSEC_CREATE_KTA = 0x5658328B,
	SL_CMD_HSEC_MAP_KERNEL_RO_PAGE = 0x5658328C,
	SL_CMD_HSEC_REG_KTA_FUNCTION_TABLE = 0x5658328D,
	SL_CMD_HSEC_REG_KTA_FUNCTION_ENTRYPOINT = 0x5658328E,
	SL_CMD_HSEC_KTA_HEAP_INIT = 0x5658328F,
	SL_CMD_HSEC_GET_AFFINITY_MASK = 0x56583290,
	SL_CMD_HSEC_ACTIVATE_KEEPALIVE_VIEW = 0x56583299,
	SL_CMD_HSEC_MAX = 0x5658329F
} perf_type_t;

typedef enum {
	CMD_INFO = 0,
	CMD_CONFIG = 1,
	CMD_CONFIG_ID = 10,
	CMD_START = 2,
	CMD_STOP = 3,
	CMD_CLEAR = 4,
	CMD_GET = 5,
	CMD_REPLAY = 6,
	CMD_NEXT = 7,
	CMD_ANNOTATE = 8,
} perf_cmd_t;

extern bool hip_valid;

typedef struct {
	uint64_t size;
	uint64_t val[6];
} intlist_t;

perf_type_t cpuid_code(void);
void info(void);
void get(void);
void config(const intlist_t *);
void config2(const char *id, const intlist_t *);
perf_type_t cpuid_code_ex(uint32_t);
void reg_sections(hsec_register_t *);
bool get_hip(void);
void reg_vIDT(void *data);
void reg_sl_global_info(hsec_sl_param_t *sl_info);

#endif
