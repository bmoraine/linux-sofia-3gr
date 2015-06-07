/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * parts of code are based on:
 *       arch/arm/mm/dma-mapping.c
 *
 * Copyright (C) 2000-2004 Russell King
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

#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/highmem.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/gfp.h>
#include <linux/pci.h>
#include <linux/mm.h>

#include <asm/processor.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>

static void dma_sync_cache_range(void *vaddr, unsigned int size,
					enum dma_data_direction dir)
{
	void *vend = vaddr + size - 1;

	for (; vaddr < vend; vaddr += boot_cpu_data.x86_clflush_size)
		clflush(vaddr);
	/*
	 * Flush any possible final partial cacheline:
	 */
	clflush(vend);

	if (dir != DMA_FROM_DEVICE)
		mb();

}

static void __dma_page_sync(struct page *page, unsigned long off,
	size_t size, enum dma_data_direction dir)
{
	unsigned long pfn;
	size_t left = size;

	pfn = page_to_pfn(page) + off / PAGE_SIZE;
	off %= PAGE_SIZE;

	do {
		size_t len = left;
		void *vaddr;
		page = pfn_to_page(pfn);

		if (PageHighMem(page)) {
			if (len + off > PAGE_SIZE)
				len = PAGE_SIZE - off;

			vaddr = kmap_atomic(page);
			dma_sync_cache_range(vaddr, len, dir);
			flush_write_buffers();
			kunmap_atomic(vaddr);
		} else {
			vaddr = page_address(page) + off;
			dma_sync_cache_range(vaddr, len, dir);
			flush_write_buffers();
		}
		off = 0;
		pfn++;
		left -= len;
	} while (left);

}

static void __dma_page_dev_to_cpu(struct page *page, unsigned long off,
	size_t size, enum dma_data_direction dir)
{
	__dma_page_sync(page, off, size, dir);
}

static void __dma_page_cpu_to_dev(struct page *page, unsigned long off,
	size_t size, enum dma_data_direction dir)
{
	__dma_page_sync(page, off, size, dir);
}


static dma_addr_t noncoherent_map_page(struct device *dev, struct page *page,
				 unsigned long offset, size_t size,
				 enum dma_data_direction dir,
				 struct dma_attrs *attrs)
{
	if (!dma_get_attr(DMA_ATTR_SKIP_CPU_SYNC, attrs))
		__dma_page_cpu_to_dev(page, offset, size, dir);

	return page_to_dma(dev, page) + offset;

}

static int noncoherent_map_sg(struct device *dev, struct scatterlist *sg,
			int nents, enum dma_data_direction dir,
			struct dma_attrs *attrs)
{
	const struct dma_map_ops *ops = get_dma_ops(dev);
	struct scatterlist *s;
	int i, j;

	for_each_sg(sg, s, nents, i) {
#ifdef CONFIG_NEED_SG_DMA_LENGTH
		s->dma_length = s->length;
#endif
		s->dma_address = ops->map_page(dev, sg_page(s), s->offset,
						sg_dma_len(s), dir, attrs);
		if (dma_mapping_error(dev, s->dma_address))
			goto bad_mapping;
	}
	return nents;

 bad_mapping:
	for_each_sg(sg, s, i, j)
		ops->unmap_page(dev, sg_dma_address(s),
				sg_dma_len(s), dir, attrs);
	return 0;
}


static void noncoherent_unmap_sg(struct device *dev,
			 struct scatterlist *sg, int nents,
			 enum dma_data_direction dir,
			 struct dma_attrs *attrs)
{
	const struct dma_map_ops *ops = get_dma_ops(dev);
	struct scatterlist *s;

	int i;

	for_each_sg(sg, s, nents, i)
		ops->unmap_page(dev, sg_dma_address(s),
					sg_dma_len(s), dir, attrs);

}

static void noncoherent_sync_single_for_device(struct device *dev,
			dma_addr_t handle, size_t size,
			enum dma_data_direction dir)
{
	unsigned int offset = handle & (PAGE_SIZE - 1);
	struct page *page = dma_to_page(dev, handle - offset);
	__dma_page_cpu_to_dev(page, offset, size, dir);
}

static void noncoherent_sync_sg_for_device(struct device *dev,
			struct scatterlist *sg, int nents,
			enum dma_data_direction dir)
{
	const struct dma_map_ops *ops = get_dma_ops(dev);
	struct scatterlist *s;
	int i;

	for_each_sg(sg, s, nents, i)
		ops->sync_single_for_device(dev, sg_dma_address(s),
					sg_dma_len(s), dir);

}

static void noncoherent_unmap_page(struct device *dev, dma_addr_t handle,
			   size_t size, enum dma_data_direction dir,
			   struct dma_attrs *attrs)
{
	if (!dma_get_attr(DMA_ATTR_SKIP_CPU_SYNC, attrs))
		__dma_page_dev_to_cpu(dma_to_page(dev, handle),
				      handle & ~PAGE_MASK, size, dir);

}

static void noncoherent_sync_single_for_cpu(struct device *dev,
				    dma_addr_t handle, size_t size,
				    enum dma_data_direction dir)
{
	unsigned int offset = handle & (PAGE_SIZE - 1);
	struct page *page = dma_to_page(dev, handle - offset);
	__dma_page_dev_to_cpu(page, offset, size, dir);

}

static void noncoherent_sync_sg_for_cpu(struct device *dev,
				struct scatterlist *sg, int nents,
				enum dma_data_direction dir)
{
	const struct dma_map_ops *ops = get_dma_ops(dev);
	struct scatterlist *s;
	int i;

	for_each_sg(sg, s, nents, i)
		ops->sync_single_for_cpu(dev, sg_dma_address(s),
							sg_dma_len(s), dir);
}


/* based on dma_common_mmap implementation */
static int noncoherent_mmap(struct device *dev,
		struct vm_area_struct *vma, void *cpu_addr,
	       dma_addr_t dma_addr, size_t size, struct dma_attrs *attrs)
{
	int ret = -ENXIO;
#ifdef CONFIG_MMU
	unsigned long user_count = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	unsigned long count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	unsigned long pfn = page_to_pfn(virt_to_page(cpu_addr));
	unsigned long off = vma->vm_pgoff;


	if (dma_get_attr(DMA_ATTR_WRITE_COMBINE, attrs))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	else if (!dma_get_attr(DMA_ATTR_NON_CONSISTENT, attrs)) {
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		if (dma_mmap_from_coherent(dev, vma, cpu_addr, size, &ret))
			return ret;
	}

	if (off < count && user_count <= (count - off)) {
		ret = remap_pfn_range(vma, vma->vm_start,
				      pfn + off,
				      user_count << PAGE_SHIFT,
				      vma->vm_page_prot);
	}
#endif	/* CONFIG_MMU */

	return ret;
}

static struct dma_map_ops noncoherent_dma_ops = {
	.alloc			= dma_generic_alloc_noncoherent,
	.free			= dma_generic_free_noncoherent,
	.mmap			= noncoherent_mmap,
	.map_sg			= noncoherent_map_sg,
	.map_page		= noncoherent_map_page,
	.unmap_page		= noncoherent_unmap_page,
	.unmap_sg		= noncoherent_unmap_sg,
	.sync_single_for_device = noncoherent_sync_single_for_device,
	.sync_sg_for_device	= noncoherent_sync_sg_for_device,
	.sync_single_for_cpu	= noncoherent_sync_single_for_cpu,
	.sync_sg_for_cpu	= noncoherent_sync_sg_for_cpu,
};

/*
 * FIXME: I should use notifiers to set archdata.dma_ops, triggered on device
 * creation, and use CONFIG_X86_DEV_DMA_OPS .
 *
 */
static __init int noncoherent_dma_init(void)
{
	pr_info("Initializing DMA ops for non coherent architecture\n");
	dma_ops = &noncoherent_dma_ops;
	return 0;
}

arch_initcall(noncoherent_dma_init);
