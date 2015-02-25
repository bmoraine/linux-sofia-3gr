/* ----------------------------------------------------------------------------
   Copyright (C) 2014 Intel Mobile Communications GmbH

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

  ---------------------------------------------------------------------------*/

#ifndef _sysprofile_h_
#define _sysprofile_h_

#include <linux/io.h>
#include <linux/smp.h>
#include <linux/types.h>

#if defined(CONFIG_SYSTEM_PROFILING)
#include <sofia/vmcalls.h>
#endif

#ifdef CONFIG_SMP
#define SYSPROF_CORE_ID                 raw_smp_processor_id()
#else
#define SYSPROF_CORE_ID                 0
#endif

/* Event class definitions (up to 32 classes, used for filtering) */
/* Keep in sync with assembler defines in 'sysprofile.inc'! */
#define SYSPROF_EVENT_CLASS_RTOS                 0
#define SYSPROF_EVENT_CLASS_RTOS_EX              1
#define SYSPROF_EVENT_CLASS_IRQ_CTRL             2
#define SYSPROF_EVENT_CLASS_FUNCTIONS            3
#define SYSPROF_EVENT_CLASS_BW_COUNTS            4
#define SYSPROF_EVENT_CLASS_EVENTS               5
#define SYSPROF_EVENT_CLASS_PERF_COUNTS          6
#define SYSPROF_EVENT_CLASS_CYCLE_COUNT          7
#define SYSPROF_EVENT_CLASS_CLK_CTRL             8
#define SYSPROF_EVENT_CLASS_TIMING               9
#define SYSPROF_EVENT_CLASS_ECT                 10
#define SYSPROF_EVENT_CLASS_SYSMON              11
#define SYSPROF_EVENT_CLASS_SPIN                12
#define SYSPROF_EVENT_CLASS_DVFS                15
#define SYSPROF_NOF_EVENT_CLASSES               32

#define SYSPROF_LINUX_CTX_INST                  0x10

#if defined(CONFIG_SYSTEM_PROFILING)

#define SYS_PROF_IF(c)	sys_prof_if[SYSPROF_CORE_ID][SYSPROF_EVENT_CLASS_##c]

/* RTOS internal */
#define sysprof_sys_idle()		iowrite32((0x00000200 | \
						   SYSPROF_LINUX_CTX_INST), \
						  SYS_PROF_IF(RTOS))
#define sysprof_sys_active()		iowrite32((0x00000300 | \
						   SYSPROF_LINUX_CTX_INST), \
						  SYS_PROF_IF(RTOS))
#define sysprof_interrupt(line)		iowrite32((0x000E0000 | \
						   (line & 0xFFFF)), \
						  SYS_PROF_IF(RTOS))
#define sysprof_int_enter()		iowrite32((0x00000800), \
						  SYS_PROF_IF(RTOS))
#define sysprof_int_leave()		iowrite32((0x00000900), \
						  SYS_PROF_IF(RTOS))

/* RTOS extended */
#define sysprof_task_enter(tid)		iowrite32((0x00030000 | \
						   (tid  & 0xFFFF)), \
						  SYS_PROF_IF(RTOS_EX))
#define sysprof_task_leave(tid)		iowrite32((0x00040000 | \
						   (tid  & 0xFFFF)), \
						  SYS_PROF_IF(RTOS_EX))
#define sysprof_syscall_enter(scn)	iowrite32((0x00188000 | \
						   (scn  & 0x7FFF)), \
						  SYS_PROF_IF(RTOS_EX))
#define sysprof_syscall_leave(scn)	iowrite32((0x00198000 | \
						   (scn  & 0x7FFF)), \
						  SYS_PROF_IF(RTOS_EX))

/* Virtualization */
#define sysprof_xirq_to_mex(num)	iowrite32((0x00050000 | \
						   (num & 0xFFFF)), \
						  SYS_PROF_IF(RTOS))
#define sysprof_xirq_to_linux(num)	iowrite32((0x00060000 | \
						   (num & 0xFFFF)), \
						  SYS_PROF_IF(RTOS))
#define sysprof_hw_interrupt(line)	iowrite32((0x000F0000 | \
						   (line & 0xFFFF)), \
						  SYS_PROF_IF(RTOS))
#define sysprof_vmexit_start()		iowrite32((0x00000F01), \
						  SYS_PROF_IF(RTOS))
#define sysprof_vmentry_end()		iowrite32((0x00000F02), \
						  SYS_PROF_IF(RTOS))
#define sysprof_vmcall_entry(nr)	iowrite32((0x00001B00 | \
						   (nr & 0xFF)), \
						  SYS_PROF_IF(RTOS))
#define sysprof_platsvc_entry(svc, op)	iowrite32((0x003B0000 | \
						   ((svc & 0xFF) << 8) | \
						   (op & 0xFF)), \
						  SYS_PROF_IF(RTOS))

/* Bandwidth counters */

/* 
 * Use the current mapping from Modem side.
 * Workaround till Linux is capable of dumping its own static metadata
 */
#define eSP_BWC_IPC_MEM_ul_total    52
#define eSP_BWC_IPC_MEM_dl_total    54

#define sysprof_bw( name, size )                                                                                                                        \
{                                                                                                                                                       \
            unsigned int sz = (unsigned int)size;                                                                                                       \
            if((sz >> 16) > 0)                                                                                                                          \
            {                                                                                                                                           \
                iowrite32(((0x04 << 24) | ( (eSP_##name & 0xFF) << 16 ) | (sz >> 16)),SYS_PROF_IF(BW_COUNTS));                                          \
            }                                                                                                                                           \
            iowrite32(((0x01 << 24) | ( (eSP_##name & 0xFF) << 16 ) | (sz & 0xFFFF)),SYS_PROF_IF(BW_COUNTS));                                           \
}

/* DVFS */
#define sysprof_dvfs_vcore_load(data)       iowrite32((0x002C0000 | \
						   (data & 0xFFFF)), \
						  SYS_PROF_IF(DVFS))
#define sysprof_dvfs_vcore_load_bal(data)   iowrite32((0x002D0000 | \
						   (data & 0xFFFF)), \
						  SYS_PROF_IF(DVFS))
#define sysprof_dvfs_cpu_freq_notif(data)   iowrite32((0x002E0000 | \
						   (data & 0xFFFF)), \
						  SYS_PROF_IF(DVFS))

#define mv_guest_trace_vmcall_entry(nr, svc, op)	{ \
		if (nr != VMCALL_PLATFORM_SERVICE) \
			sysprof_vmcall_entry(nr); \
		else \
			sysprof_platsvc_entry(svc, op); \
	}

extern uint32_t __iomem
	*sys_prof_if[CONFIG_NR_CPUS][SYSPROF_NOF_EVENT_CLASSES];

#else

#define sysprof_sys_idle()
#define sysprof_sys_active()
#define sysprof_interrupt(line)
#define sysprof_int_enter()
#define sysprof_int_leave()
#define sysprof_task_enter(tid)
#define sysprof_task_leave(tid)
#define sysprof_syscall_enter(scn)
#define sysprof_syscall_leave(scn)
#define sysprof_xirq_to_mex(num)
#define sysprof_xirq_to_linux(num)
#define sysprof_hw_interrupt(line)
#define sysprof_vmexit_start()
#define sysprof_vmentry_end()
#define sysprof_vmcall_entry(nr)
#define sysprof_platsvc_entry(svc, op)

#define mv_guest_trace_vmcall_entry(nr, svc, op)

#endif

#define mv_guest_trace_vmcall_exit()		sysprof_vmentry_end()
#define mv_guest_trace_xirq_post(num)		sysprof_xirq_to_mex(num)
#define mv_guest_trace_ipi_post(num)		sysprof_xirq_to_linux(num)
#define mv_guest_trace_virq_mask(virq)
#define mv_guest_trace_virq_unmask(virq)

#endif
