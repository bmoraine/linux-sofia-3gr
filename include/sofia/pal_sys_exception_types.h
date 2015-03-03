/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _PAL_SYS_EXCEPTION_TYPES_H
#define _PAL_SYS_EXCEPTION_TYPES_H

#define TRAP_DUMP_DATE_SIZE      11
#define TRAP_DUMP_TIME_SIZE      9
#define TRAP_DUMP_FILENAME_SIZE  128
#define TRAP_MAX_LOG_DATA_SIZE   255

#define TRAP_SW_GENERATED        0xDDDD

#define MAX_VCPU_PER_VM    4
#define MAX_VM  3

enum sys_exception {
	SYS_EXCEPTION_MEX = 0,
	SYS_EXCEPTION_LINUX = 1,
	SYS_EXCEPTION_VMM = 2,
	SYS_EXCEPTION_SECURITY = 3
};

/**
 * Structure for holding internal X86 general purpose registers
 */
struct x86_gp_regs {
	uint32_t eax;
	uint32_t ebx;
	uint32_t ecx;
	uint32_t edx;
	uint32_t esi;
	uint32_t edi;
	uint32_t ebp;
	uint32_t esp;
};

/**
 * Structure for holding internal X86 control registers
 */
struct x86_ctrl_regs {
	uint32_t cr0;
	uint32_t cr2;
	uint32_t cr3;
	uint32_t cr4;
};

/**
 * Structure for holding internal X86 EFLAGS register
 */
struct x86_eflags_reg {
	uint32_t eflags;
};

/**
 * Structure for holding internal X86 segment registers
 */
struct x86_segment_regs {
	uint16_t cs;
	uint16_t ds;
	uint16_t ss;
	uint16_t es;
	uint16_t fs;
	uint16_t gs;
};

/**
 * Structure for holding internal X86 debug registers
 */
struct x86_debug_regs {
	uint32_t dr0;
	uint32_t dr1;
	uint32_t dr2;
	uint32_t dr3;
	uint32_t dr4;
	uint32_t dr5;
	uint32_t dr6;
	uint32_t dr7;
};

/**
 * Structure for holding internal X86 Memory manager registers
 */
struct x86_mm_regs {
	uint32_t gdtb;
	uint32_t idtb;
	uint32_t ldtb;
	uint32_t tssb;
	uint16_t gdtl;
	uint16_t idtl;
	uint16_t ldtl;
	uint16_t tssl;
	uint16_t tr;
	uint16_t ldtr;
};

/**
 * Structure for holding internal X86 EIP register
 */
struct x86_eip_reg {
	uint32_t eip;
};

struct x86_cpu_regs {
	struct x86_gp_regs       gp_regs;
	struct x86_segment_regs  segment_regs;
	struct x86_eflags_reg    eflags_reg;
	struct x86_eip_reg         eip_reg;
	struct x86_ctrl_regs     ctrl_regs;
	struct x86_mm_regs       mm_regs;
	struct x86_debug_regs    debug_regs;
};

struct sys_trap {
	uint32_t exception_type;
	uint16_t trap_vector;
	char date[TRAP_DUMP_DATE_SIZE];
	char time[TRAP_DUMP_TIME_SIZE];
	char filename[TRAP_DUMP_FILENAME_SIZE];
	uint32_t line;
	char log_data[TRAP_MAX_LOG_DATA_SIZE];
	struct x86_cpu_regs regs;

	/* OS dependent */
	union {
		struct {
			char *kmsg;         /* kernel message */
			uint32_t kmsg_len;  /* kernel message length */
		} linux_log;
		/* other OS dependent log structure to be added here */
	} os;
};

struct sys_vm {
	uint32_t os_id;
	char no_of_vcpu;
	struct x86_cpu_regs vcpu_reg[MAX_VCPU_PER_VM];
};

struct sys_vm_dump {
	char no_of_vm;
	struct sys_vm  vm[MAX_VM];
};

#endif /* _PAL_SYS_EXCEPTION_TYPES_H */
