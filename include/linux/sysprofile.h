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

#define SYSPROF_NOF_PHYSICAL_CORES      2

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

extern uint32_t __iomem
	*sys_prof_if[SYSPROF_NOF_PHYSICAL_CORES][SYSPROF_NOF_EVENT_CLASSES];

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

#endif

#endif
