#ifndef _ASM_X86_CACHE_H
#define _ASM_X86_CACHE_H

#include <linux/linkage.h>

/* L1 cache line size */
#define L1_CACHE_SHIFT	(CONFIG_X86_L1_CACHE_SHIFT)
#define L1_CACHE_BYTES	(1 << L1_CACHE_SHIFT)

/*
 * Memory returned by kmalloc() may be used for DMA, so we must make
 * sure that all such allocations are cache aligned. Otherwise,
 * unrelated code may cause parts of the buffer to be read into the
 * cache before the transfer is done, causing old data to be seen by
 * the CPU.
 * This is specific to XGOLD architecture for x86, used to host
 * virtualized SOFIA platform
 */
#ifdef CONFIG_X86_INTEL_XGOLD
#define ARCH_DMA_MINALIGN       L1_CACHE_BYTES
#define ARCH_SLAB_MINALIGN	8
#endif

#define __read_mostly __attribute__((__section__(".data..read_mostly")))

#define INTERNODE_CACHE_SHIFT CONFIG_X86_INTERNODE_CACHE_SHIFT
#define INTERNODE_CACHE_BYTES (1 << INTERNODE_CACHE_SHIFT)

#ifdef CONFIG_X86_VSMP
#ifdef CONFIG_SMP
#define __cacheline_aligned_in_smp					\
	__attribute__((__aligned__(INTERNODE_CACHE_BYTES)))		\
	__page_aligned_data
#endif
#endif

#endif /* _ASM_X86_CACHE_H */
