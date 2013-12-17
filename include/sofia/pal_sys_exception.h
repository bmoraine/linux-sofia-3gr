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

#ifndef _PAL_SYS_EXCEPTION_H
#define _PAL_SYS_EXCEPTION_H

#define TRAP_DUMP_DATE_SIZE      11
#define TRAP_DUMP_TIME_SIZE      9
#define TRAP_DUMP_FILENAME_SIZE  128
#define TRAP_DUMP_TASKNAME_SIZE  8
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
	unsigned int eax;
	unsigned int ebx;
	unsigned int ecx;
	unsigned int edx;
	unsigned int esi;
	unsigned int edi;
	unsigned int ebp;
	unsigned int esp;
};

/**
 * Structure for holding internal X86 control registers
 */
struct x86_ctrl_regs {
	unsigned int cr0;
	unsigned int cr2;
	unsigned int cr3;
	unsigned int cr4;
};

/**
 * Structure for holding internal X86 EFLAGS register
 */
struct x86_eflags_reg {
	unsigned int eflags;
};

/**
 * Structure for holding internal X86 segment registers
 */
struct x86_segment_regs {
	unsigned short cs;
	unsigned short ds;
	unsigned short ss;
	unsigned short es;
	unsigned short fs;
	unsigned short gs;
};

/**
 * Structure for holding internal X86 debug registers
 */
struct x86_debug_regs {
	unsigned int dr0;
	unsigned int dr1;
	unsigned int dr2;
	unsigned int dr3;
	unsigned int dr4;
	unsigned int dr5;
	unsigned int dr6;
	unsigned int dr7;
};

/**
 * Structure for holding internal X86 Memory manager registers
 */
struct x86_mm_regs {
	unsigned int gdtb;
	unsigned int idtb;
	unsigned int ldtb;
	unsigned int tssb;
	unsigned short gdtl;
	unsigned short idtl;
	unsigned short ldtl;
	unsigned short tssl;
	unsigned short tr;
	unsigned short ldtr;
};

/**
 * Structure for holding internal X86 EIP register
 */
struct x86_eip_reg {
	unsigned int eip;
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
	enum sys_exception exception_type;
	unsigned short trap_vector;
	char date[TRAP_DUMP_DATE_SIZE];
	char time[TRAP_DUMP_TIME_SIZE];
	char filename[TRAP_DUMP_FILENAME_SIZE];
	unsigned int line;
	char log_data[TRAP_MAX_LOG_DATA_SIZE];
	struct x86_cpu_regs regs;

	/* OS dependent */
	union {
		struct {
			char *kmsg;         /* kernel message */
			unsigned long kmsg_len;  /* kernel message length */
		} linux_log;
		/* other OS dependent log structure to be added here */
	} os;
};

struct sys_vm {
	unsigned int os_id;
	char no_of_vcpu;
	struct x86_cpu_regs vcpu_reg[MAX_VCPU_PER_VM];
};

struct sys_vm_dump {
	char no_of_vm;
	struct sys_vm vm[MAX_VM];
};

/* FUNCTION PROTOTYPES */

/*
    PAL system exception initialization
*/
void pal_sys_exception_init(void);

void pal_sys_trap(void *log_data, int log_size_bytes, int line, char *filename);

#endif /* _PAL_SYS_EXCEPTION_H*/
