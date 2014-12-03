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

#ifndef _MV_GAL_H
#define _MV_GAL_H

#ifdef __KERNEL
#include <sofia/mv_hypercalls.h>
#else
#include "mv_hypercalls.h"
#endif

/*
 * VMM adaptation layer initialization. should be called once at system start up
 */
void mv_gal_init(struct vmm_shared_data *data);

/*
 * VMM initialization for primary and secondary
*/
int sofia_vmm_init(void);
int sofia_vmm_init_secondary(void);

/*
 * Returns OS ID of the guest
 */
uint32_t mv_gal_os_id(void);

/*
 * VMM adaptation physical/virtual memory translation.
 */
void *mv_gal_ptov(vmm_paddr_t paddr);
vmm_paddr_t mv_gal_vtop(void *vaddr);

/*
 * VMM vlink lookup by name
 */
vmm_paddr_t mv_gal_vlink_lookup(const char *name, vmm_paddr_t plnk);

/*
 * attach/detach XIRQ callback. (XIRQ should have been allocated first.)
 */
void *mv_gal_register_xirq_callback(uint32_t xirq,
				void (*cb)(void *, uint32_t), void *);

void mv_gal_xirq_detach(void *id);

/*
 * attach/detach HIRQ callback.
 */
void *mv_gal_register_hirq_callback(uint32_t hirq,
				void (*cb)(void *, uint32_t), void *cookie);

void mv_gal_hirq_detach(void *id);

/*
 * Get per VCPU-Mobilevisor shared data
 */
extern struct vmm_shared_data *mv_gal_get_shared_data(void);
/*
 * print to vmm log amd loops
 */
void mv_gal_panic(char *panic_msg);

/*
 * print to vmm log
 */
#ifdef VM_DEBUG_ON
#define mv_gal_printk(format, s...) \
	do { \
		extern struct vmm_shared_data *mv_gal_get_shared_data(); \
		snprintf(mv_gal_get_shared_data()->vm_log_str, \
			sizeof(mv_gal_get_shared_data()->vm_log_str), \
			format, ##s); \
		mv_guest_log(); \
	} while (0)
#else
#define mv_gal_printk(format, s...)
#endif



#endif /* _MV_GAL_H */
