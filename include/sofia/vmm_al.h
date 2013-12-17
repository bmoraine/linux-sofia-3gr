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

/*
 * NOTES:
 * 1) This source file is included in guests including Linux and purposely
 * kept in line with Linux kernel coding guidelines for ease of portability and
 * maintainability. See linux/Documentation/CodingStyle for coding guidelines.
 * Please check all further changes are compliant with Linux coding standard
 * and clear all commands reported by Linux checkpatch.pl
 * Use command: linux/scripts/checkpatch.pl -f <filename>
 * Clear ALL warnings and errors reported.
 *
*/

#ifndef _VMM_AL_H
#define _VMM_AL_H

#ifdef __KERNEL
#include <sofia/vmm_guest_api.h>
#else
#include "vmm_guest_api.h"
#endif

/*
 * VMM adaptation layer initialization. should be called once at system start up
 */
void vmm_al_init(struct vmm_shared_data *data);

/*
 * Returns OS ID of the guest
 */
uint32_t vmm_myid(void);

/*
 * VMM adaptation physical/virtual memory translation.
 */
void *vmm_ptov(vmm_paddr_t paddr);
vmm_paddr_t vmm_vtop(void *vaddr);

/*
 * VMM vlink lookup by name
 */
vmm_paddr_t vmm_vlink_lookup(const char *name, vmm_paddr_t plnk);

/*
 * attach/detach XIRQ callback. (XIRQ should have been allocated first.)
 */
void *vmm_register_xirq_callback(uint32_t xirq,
				void (*cb)(void *, uint32_t), void *);

void vmm_xirq_detach(void *id);

/*
 * attach/detach HIRQ callback.
 */
void *vmm_register_hirq_callback(uint32_t hirq,
				void (*cb)(void *, uint32_t), void *cookie);

void vmm_hirq_detach(void *id);

/*
 * Get per VCPU-Mobilevisor shared data
 */
extern struct vmm_shared_data *get_vmm_shared_data(void);
/*
 * print to vmm log amd loops
 */
void vmm_panic(char *panic_msg);

/*
 * print to vmm log
 */
#ifdef VM_DEBUG_ON
#define vmm_printk(format, s...) \
	do { \
		extern struct vmm_shared_data *get_vmm_shared_data(); \
		snprintf(get_vmm_shared_data()->vm_log_str, \
			sizeof(get_vmm_shared_data()->vm_log_str), \
			format, ##s); \
		vmm_guest_log(); \
	} while (0)
#else
#define vmm_printk(format, s...)
#endif
#endif /* _VMM_AL_H */
