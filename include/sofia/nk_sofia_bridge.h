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
 * 2) Use only C99 fixed width types for definition as this header needs to be
 * both 32bit/64bit portable.
 * Avoid the use of pointers/enum in structure as that make the structure
 * variable size based on 32/64bit toolchain used.
*/

#ifndef _D_NK_SOFIA_BRIDGE_H
#define _D_NK_SOFIA_BRIDGE_H

#ifdef __KERNEL__
#include <sofia/mv_ipc.h>
#include <sofia/mv_gal.h>
#include <linux/smp.h>
#include <linux/percpu.h>
#else
#include "mv_ipc.h"
#include "mv_gal.h"
#endif

/* Use C99 portable fixed width types */
typedef uint8_t  nku8_f;
typedef uint16_t nku16_f;
typedef uint32_t nku32_f;
typedef uint64_t nku64_f;

#ifdef __KERNEL__
void sofia_stop_other_cpus(int wait);
void irq_force_complete_move(int irq);
#endif

typedef nku32_f NkPhAddr;	/* physical address */
typedef nku32_f NkPhSize;	/* physical size */

typedef nku32_f NkOsId;		/* OS identifier */
typedef nku32_f NkOsMask;	/* OS identifiers bit-mask */

typedef nku32_f NkXIrq;		/* cross interrupt request */
typedef nku32_f NkVex;		/* virtual exception number */

typedef nku32_f NkCpuId;	/* CPU identifier */
typedef nku32_f NkCpuMask;	/* CPU identifier bit-mask */

typedef void *NkXIrqId;
typedef nku32_f NkResourceId;

#define NK_DEV_VLINK_OFF        VLINK_OFF
#define NK_DEV_VLINK_RESET      VLINK_INIT
#define NK_DEV_VLINK_ON         VLINK_ON

#define NK_DEV_VLINK_NAME_LIMIT 16

#define NkDevVlink struct vmm_vlink
typedef void (*NkXIrqHandler)(void *cookie, NkXIrq xirq);

#define NK_XIRQ_SYSCONF 768

typedef enum NkVersion {
	NK_VERSION_8 = 8,	/* to be inline with 6321 nkddi */
} NkVersion;

typedef struct NkDevOps {

	NkVersion nk_version;	/* DDI version */
	/*
	 * Get self operating system ID
	 */
	 NkOsId(*nk_id_get) (void);

	/*
	 * Convert a physical address into a virtual one.
	 */
	void *(*nk_ptov) (NkPhAddr paddr);

	/*
	 * Convert a virtual address into a physical one.
	 */
	 NkPhAddr(*nk_vtop) (void *vaddr);

	/*
	 * Attach a handler to a given NanoKernel cross-interrupt.
	 * (0 returned on failure)
	 * Must be called from base level.
	 * The handler is called with ONLY masked cross interrupt source.
	 */
	 NkXIrqId
	    (*nk_xirq_attach) (NkXIrq xirq, NkXIrqHandler hdl, void *cookie);

	/*
	 * Mask a given cross-interrupt
	 */
	void
	 (*nk_xirq_mask) (NkXIrq xirq);

	/*
	 * Unmask a given cross-interrupt
	 */
	void
	 (*nk_xirq_unmask) (NkXIrq xirq);

	/*
	 * Detach a handler (previously attached with irq_attach())
	 *
	 * Must be called from base level
	 */
	void
	 (*nk_xirq_detach) (NkXIrqId id);

	/*
	 * Trigger a cross-interrupt to a given operating system.
	 *
	 * Must be called from base level
	 */
	void
	 (*nk_xirq_trigger) (NkXIrq xirq, NkOsId osId);

	/*
	 * Get high priority bit set in bit mask.
	 */
	 nku32_f(*nk_mask2bit) (nku32_f mask);

	/*
	 * Insert a bit of given priority into a mask.
	 */
	 nku32_f(*nk_bit2mask) (nku32_f bit);

	/*
	 * Atomic operation to clear bits within a bit field.
	 *
	 * The following logical operation: *mask &= ~clear
	 * is performed atomically.
	 */
	void
	 (*nk_atomic_clear) (volatile nku32_f *mask, nku32_f clear);
	/*
	 * Atomic operation to clear bits within a bit field.
	 *
	 * The following logical operation: *mask &= ~clear
	 * is performed atomically.
	 *
	 * Returns 0 if and only if the result is zero.
	 */
	 nku32_f(*nk_clear_and_test) (volatile nku32_f *mask, nku32_f clear);

	/*
	 * Atomic operation to set bits within a bit field.
	 *
	 * The following logical operation: *mask |= set
	 * is performed atomically
	 */
	void
	 (*nk_atomic_set) (volatile nku32_f *mask, nku32_f set);

	/*
	 * Atomic operation to substract value to a given memory location.
	 *
	 * The following logical operation: *ptr -= val
	 * is performed atomically.
	 */
	void
	 (*nk_atomic_sub) (volatile nku32_f *ptr, nku32_f val);

	/*
	 * Atomic operation to substract value to a given memory location.
	 *
	 * The following logical operation: *ptr -= val
	 * is performed atomically.
	 *
	 * Returns 0 if and only if the result is zero.
	 */
	 nku32_f(*nk_sub_and_test) (volatile nku32_f *ptr, nku32_f val);

	/*
	 * Atomic operation to add value to a given memory location.
	 *
	 * The following logical operation: *ptr += val
	 * is performed atomically
	 */
	void
	 (*nk_atomic_add) (volatile nku32_f *ptr, nku32_f val);

	/*
	 * Map a physical address range of "shared" memory
	 * into supervisor space (RAM from other systems).
	 *
	 * On error, NULL is returned.
	 */
	void *(*nk_mem_map) (NkPhAddr paddr, NkPhSize size);
	/*
	 * Unmap memory previously mapped using nk_mem_map.
	 */
	void
	 (*nk_mem_unmap) (void *vaddr, NkPhAddr paddr, NkPhSize size);

	/*
	 * Lookup first virtual communication link of given a class/name
	 * into NanoKernel repository.
	 *
	 * Return value:
	 *
	 * if <plnk> is zero the first intance of a virtual link with
	 * a required <name> is returned. Otherwise <plnk> must be an
	 * address returned by a previous call to nk_vlink_lookup().
	 * The next virtual link with required <name>, starting from <plnk>
	 * is returned in that case.
	 *
	 * NULL is returned, if no virtual link with required <name> is found.
	 */
	 NkPhAddr(*nk_vlink_lookup) (const char *name, NkPhAddr plnk);

	/*
	 * Allocate <size> bytes (rounded up to the nearest multiple of
	 * page size) of contiguous memory from the persistent communication
	 * memory pool.
	 *
	 * The allocated memory block is labelled using <vlink, id>.
	 * It is guaranteed that for a uniq label, a uniq memory block
	 * is allocated. Thus different calls with the same label always
	 * return the same result.
	 *
	 * Return the physical base address of the allocated memory block
	 * (aligned to a page boundary), or 0 on failure.
	 */
	 NkPhAddr
	    (*nk_pmem_alloc) (NkPhAddr vlink, NkResourceId id, NkPhSize size);
	/*
	 * Allocate <nb> contiguous persistent cross-interrupts.
	 *
	 * The allocated xirqs range is labelled using <vlink, id>.
	 * It is guaranteed that for a uniq label, a uniq xirqs range
	 * is allocated. Thus different calls with the same label always
	 * return the same result.
	 *
	 * Return the number of the first allocated xirq or 0 if not enough
	 * xirq are available.
	 */
	 NkXIrq
	    (*nk_pxirq_alloc) (NkPhAddr vlink, NkResourceId id, NkOsId osid,
			       int32_t nb);

	/*
	 * Set vCPUs affinity for a given cross-interrupt
	 */
	void
	 (*nk_xirq_affinity) (NkXIrq xirq, NkCpuMask cpus);

	/*
	 * Allocate <size> bytes of contiguous memory from the persistent
	 * device repository.
	 *
	 * The allocated memory block is labeled using <vlink, id>.
	 * It is guaranteed that for a unique label, a unique memory block
	 * is allocated. Thus different calls with the same label always
	 * return the same result.
	 *
	 * Return the physical base address of the allocated memory block,
	 * or 0 on failure.
	 */
	NkPhAddr
	    (*nk_pdev_alloc) (NkPhAddr vlink, NkResourceId id, NkPhSize size);

} NkDevOps;

extern NkDevOps nkops;

extern const char *vm_command_line;
#define vlx_command_line vm_command_line
#endif
