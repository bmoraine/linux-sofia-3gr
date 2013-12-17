/* ----------------------------------------------------------------------------
 *  Copyright (C) 2014 Intel Mobile Communications GmbH

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
 * 2) Use only NK defined types for definition as this header needs to be
 * both 32bit/64bit portable.
*/

#ifdef __KERNEL__
#include <linux/bitops.h>
#include <linux/atomic.h>
#include <linux/gfp.h>
#include <sofia/nk_sofia_bridge.h>
#include <sofia/vmm_al.h>
#include <asm/mpspec.h>
#include <asm/irq_vectors.h>
#else
#include "nk_sofia_bridge.h"
#include "vmm_al.h"
#endif


#define ASM_ATOMIC_OPERATIONS

#ifndef __KERNEL__

#ifdef ASM_ATOMIC_OPERATIONS
#define LOCK_PREFIX "\n\tlock;"

#define atomic_clear_mask(mask, addr) \
{ \
	__asm__ __volatile__( \
	LOCK_PREFIX "andl %0,%1" \
	: : "r" (~(mask)), "m" (*(addr)) \
	: "memory"); \
}

#define atomic_set_mask(mask, addr) \
{ \
	__asm__ __volatile__( \
	LOCK_PREFIX "orl %0,%1" \
	: : "r" ((unsigned)(mask)), "m" (*(addr))  \
	: "memory"); \
}
#endif

static inline nku32_f __ffs(nku32_f word)
{
#if defined(ASM_ATOMIC_OPERATIONS)
	__asm__ __volatile__("\n\tbsf %1,%0" : "=r"(word) : "rm"(word));
	return word;
#else
	nku32_f n = 31;

	if (word & 0x0000ffff) {
		n -= 16;
		word <<= 16;
	}
	if (word & 0x00ff0000) {
		n -= 8;
		word <<= 8;
	}
	if (word & 0x0f000000) {
		n -= 4;
		word <<= 4;
	}
	if (word & 0x30000000) {
		n -= 2;
		word <<= 2;
	}
	if (word & 0x40000000)
		n -= 1;

	return n;
#endif
}

#endif /* !defined(__KERNEL__) */

static inline void _intr_mask(void)
{
#if defined(__KERNEL__)
	__asm__ __volatile__("\tcli\n");
#else
#endif
}

static inline void _intr_unmask(void)
{
#if defined(__KERNEL__)
	__asm__ __volatile__("\tsti\n");
#else
#endif
}

/*
 * Find the first bit set in the given mask
 */
static nku32_f vmm_ffs(nku32_f mask)
{
	if (mask == 0)
		vmm_panic("vmm_ffs called with mask equal to 0\n");

	return __ffs(mask);
}

/*
 * Insert a bit of given priority into a mask.
 */
static inline nku32_f vmm_bit_mask(nku32_f _bit)
{
	return 1 << _bit;
}

/*
 * Atomic operation to clear bits within a bit field.
 */
static void vmm_atomic_clear(volatile nku32_f *addr, nku32_f data)
{
	if (!addr)
		return;

#ifndef ASM_ATOMIC_OPERATIONS
	/* TODO: locks needed.
	 * For now, we just want to get vbpipe to compile with basic functions
	 */

	_intr_mask();
	*addr &= ~data;
	_intr_unmask();
#else
	atomic_clear_mask(data, addr);
#endif
}

/*
 * Atomic operation to clear bits within a bit field.
 * Returns 0 if and only if the result is 0.
 */
static nku32_f vmm_clear_and_test(volatile nku32_f *addr, nku32_f data)
{
#ifndef ASM_ATOMIC_OPERATIONS
	/*TODO: locks needed.
	 * For now, we just want to get vbpipe to compile with basic functions
	 */

	if (!addr)
		return ~0;

	_intr_mask();
	*addr &= ~data;
	_intr_unmask();
	return (*addr == 0) ? 0 : 1;
#else
	nku32_f res = 0;

	if (!addr)
		return ~0;

	__asm__ __volatile__(LOCK_PREFIX "andl %2,%0; setnz %b1" : "+m"(*addr),
			     "=qm"(res)
			     : "r"(~(data))
			     : "memory");
	return res;
#endif
}

/*
 * Atomic operation to set bits within a bit field.
 */
static void vmm_atomic_set(volatile nku32_f *addr, nku32_f data)
{
	if (!addr)
		return;

#ifndef ASM_ATOMIC_OPERATIONS
	/* TODO: locks needed.
	 * For now, we just want to get vbpipe to compile with basic functions
	 */

	_intr_mask();
	*addr |= data;
	_intr_unmask();
#else
	atomic_set_mask(data, addr);
#endif
}

/*
 * Atomic operation to subtract value to a given memory location.
 */
static void vmm_atomic_sub(volatile nku32_f *addr, nku32_f data)
{
	if (!addr)
		return;

#ifndef ASM_ATOMIC_OPERATIONS
	/* TODO: locks needed.
	* For now, we just want to get vbpipe to compile with basic functions
	*/
	_intr_mask();
	*addr -= data;
	_intr_unmask();
#else
	__asm__ __volatile__(LOCK_PREFIX "subl %1,%0" : "+m"(*addr) :
			"ir"(data));
#endif
}

/*
 * Atomic operation to subtract value to a given memory location.
 * Returns 0 if and only if the result is 0.
 */
static nku32_f vmm_sub_and_test(volatile nku32_f *addr, nku32_f data)
{
#ifndef ASM_ATOMIC_OPERATIONS
	/* TODO: locks needed.
	 * For now, we just want to get vbpipe to compile with basic functions
	 */

	if (!addr)
		return ~0;

	_intr_mask();
	*addr -= data;
	_intr_unmask();
	return (*addr == 0) ? 0 : 1;
#else
	nku32_f res = 0;
	if (!addr)
		return ~0;

	__asm__ __volatile__(LOCK_PREFIX "subl %2, %0; setnz %b1" : "+m"(*addr),
			     "=q"(res)
			     : "ir"(data)
			     : "memory", "cc");
	return res;
#endif
}

/*
 * Atomic operation to add value to a given memory location.
 */
void vmm_atomic_add(volatile nku32_f *addr, nku32_f data)
{
	if (!addr)
		return;

#ifndef ASM_ATOMIC_OPERATIONS
	/*TODO: locks needed.
	 * For now, we just want to get vbpipe to compile with basic functions
	 */

	_intr_mask();
	*addr += data;
	_intr_unmask();
#else
	__asm__ __volatile__(LOCK_PREFIX "addl %1,%0" : "+m"(*addr)
			     : "ir"(data));
#endif
}

static void *vmm_mem_map(NkPhAddr paddr, NkPhSize size)
{
	return vmm_ptov(paddr);
}

static void vmm_mem_unmap(void *vaddr, NkPhAddr paddr, NkPhSize size)
{
}

NkDevOps nkops = {		/* NKDDI operations */
	NK_VERSION_8,
	vmm_myid,
	vmm_ptov,
	vmm_vtop,
	vmm_register_xirq_callback,
	vmm_virq_mask,
	vmm_virq_unmask,
	vmm_xirq_detach,
	vmm_xirq_post,
	vmm_ffs,
	vmm_bit_mask,
	vmm_atomic_clear,
	vmm_clear_and_test,
	vmm_atomic_set,
	vmm_atomic_sub,
	vmm_sub_and_test,
	vmm_atomic_add,
	vmm_mem_map,
	vmm_mem_unmap,
	vmm_vlink_lookup,
	vmm_shared_mem_alloc,
	vmm_xirq_alloc,
	vmm_set_vaffinity,
	vmm_shared_mem_alloc,
};

#ifdef __KERNEL__

void irq_force_complete_move(int irq)
{
	//TODO
}

#endif
