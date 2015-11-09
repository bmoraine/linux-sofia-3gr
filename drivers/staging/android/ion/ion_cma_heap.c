/*
 * drivers/staging/android/ion/ion_cma_heap.c
 *
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 * Copyright (C) Linaro 2012
 * Author: <benjamin.gaignard@linaro.org> for ST-Ericsson.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#ifdef CONFIG_ROCKCHIP_IOMMU
#include <linux/rockchip_iovmm.h>
#endif

#include <linux/dma-contiguous.h>

#include "ion.h"
#include "ion_priv.h"

#define ION_CMA_ALLOCATE_FAILED -1

struct ion_cma_heap {
	struct ion_heap heap;
	struct device *dev;
	struct cma *cma_area;
};

#define to_cma_heap(x) container_of(x, struct ion_cma_heap, heap)

struct ion_cma_buffer_info {
	void *cpu_addr;
	dma_addr_t handle;
	struct sg_table *table;
};

DEFINE_SEMAPHORE(ion_cma_sem);

/*
 * Three global variables to save the ion cma statitics, all in bytes
 *
 *	total_ion_cma:		total size of all cma heaps
 *	allocated_ion_cma:	total size of allocated cma buffers
 *	unused_ion_cma:		un-allocated cma buffers
 */
static int total_ion_cma;
static int allocated_ion_cma;
int unused_ion_cma;

DEFINE_SPINLOCK(ion_cma_lock);

/*
 * Create scatter-list for the already allocated DMA buffer.
 * This function could be replaced by dma_common_get_sgtable
 * as soon as it will avalaible.
 */
static int ion_cma_get_sgtable(struct device *dev, struct sg_table *sgt,
			       void *cpu_addr, dma_addr_t handle, size_t size)
{
	struct page *page = virt_to_page(cpu_addr);
	int ret;

	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (unlikely(ret))
		return ret;

	sg_set_page(sgt->sgl, page, PAGE_ALIGN(size), 0);
	return 0;
}

/* ION CMA heap operations functions */
static int ion_cma_allocate(struct ion_heap *heap, struct ion_buffer *buffer,
			    unsigned long len, unsigned long align,
			    unsigned long flags)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info;
	unsigned long irqflags;

	dev_dbg(dev, "Request buffer allocation len %ld\n", len);

	if (align > PAGE_SIZE)
		return -EINVAL;

	info = kzalloc(sizeof(struct ion_cma_buffer_info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "Can't allocate buffer info\n");
		return ION_CMA_ALLOCATE_FAILED;
	}

	/* set cma_area */
	down(&ion_cma_sem);
	dev_set_cma_area(dev, cma_heap->cma_area);

#ifdef CONFIG_X86
	if (buffer->flags & ION_FLAG_CACHED)
		info->cpu_addr = dma_alloc_cma_writeback(dev, len, &(info->handle),
						GFP_HIGHUSER | __GFP_ZERO);
	else
		info->cpu_addr = dma_alloc_cma_writecombine(dev, len,
					&(info->handle),
					GFP_HIGHUSER | __GFP_ZERO);
#else
	info->cpu_addr = dma_alloc_coherent(dev, len, &(info->handle),
						GFP_HIGHUSER | __GFP_ZERO);
#endif
	up(&ion_cma_sem);
	if (!info->cpu_addr) {
		dev_err(dev, "Fail to allocate buffer\n");
		goto err;
	}

	spin_lock_irqsave(&ion_cma_lock, irqflags);
	allocated_ion_cma  += len;
	unused_ion_cma -= len;
	spin_unlock_irqrestore(&ion_cma_lock, irqflags);

	info->table = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!info->table) {
		dev_err(dev, "Fail to allocate sg table\n");
		goto free_mem;
	}

	if (ion_cma_get_sgtable
	    (dev, info->table, info->cpu_addr, info->handle, len))
		goto free_table;
	/* keep this for memory release */
	buffer->priv_virt = info;
	dev_dbg(dev, "Allocate buffer %p\n", buffer);
	return 0;

free_table:
	kfree(info->table);
free_mem:
#ifdef CONFIG_X86
	dma_free_cma_writecombine(dev, len, info->cpu_addr, info->handle);
#else
	dma_free_coherent(dev, len, info->cpu_addr, info->handle);
#endif
err:
	kfree(info);
	return ION_CMA_ALLOCATE_FAILED;
}

static void ion_cma_free(struct ion_buffer *buffer)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(buffer->heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info = buffer->priv_virt;
	unsigned long flags;

	dev_dbg(dev, "Release buffer %p\n", buffer);
	/* set cma_area */
	down(&ion_cma_sem);
	dev_set_cma_area(dev, cma_heap->cma_area);
	/* release memory */
#ifdef CONFIG_X86
	dma_free_cma_writecombine(dev, buffer->size, info->cpu_addr, info->handle);
#else
	dma_free_coherent(dev, buffer->size, info->cpu_addr, info->handle);
#endif
	up(&ion_cma_sem);

	spin_lock_irqsave(&ion_cma_lock, flags);
	allocated_ion_cma  -= buffer->size;
	unused_ion_cma  += buffer->size;
	spin_unlock_irqrestore(&ion_cma_lock, flags);

	/* release sg table */
	sg_free_table(info->table);
	kfree(info->table);
	kfree(info);
}

/* return physical address in addr */
static int ion_cma_phys(struct ion_heap *heap, struct ion_buffer *buffer,
			ion_phys_addr_t *addr, size_t *len)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(buffer->heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info = buffer->priv_virt;

	dev_dbg(dev, "Return buffer %p physical address 0x%pa\n", buffer,
		&info->handle);

	*addr = info->handle;
	*len = buffer->size;

	return 0;
}

static struct sg_table *ion_cma_heap_map_dma(struct ion_heap *heap,
					     struct ion_buffer *buffer)
{
	struct ion_cma_buffer_info *info = buffer->priv_virt;

	return info->table;
}

static void ion_cma_heap_unmap_dma(struct ion_heap *heap,
				   struct ion_buffer *buffer)
{
	return;
}

static int ion_cma_mmap(struct ion_heap *mapper, struct ion_buffer *buffer,
			struct vm_area_struct *vma)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(buffer->heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info = buffer->priv_virt;
	int ret;

	/* set cma_area */
	down(&ion_cma_sem);
	dev_set_cma_area(dev, cma_heap->cma_area);
#ifdef CONFIG_X86
	if (buffer->flags & ION_FLAG_CACHED)
		ret = dma_mmap_writeback(dev, vma, info->cpu_addr,
				info->handle, buffer->size);
	else
		ret = dma_mmap_writecombine(dev, vma, info->cpu_addr,
				info->handle, buffer->size);
#else
	ret = dma_mmap_coherent(dev, vma, info->cpu_addr, info->handle,
				buffer->size);
#endif
	up(&ion_cma_sem);
	return ret;

}

static void *ion_cma_map_kernel(struct ion_heap *heap,
				struct ion_buffer *buffer)
{
	struct ion_cma_buffer_info *info = buffer->priv_virt;
	/* kernel memory mapping has been done at allocation time */
	return info->cpu_addr;
}

static void ion_cma_unmap_kernel(struct ion_heap *heap,
					struct ion_buffer *buffer)
{
}

#ifdef CONFIG_ROCKCHIP_IOMMU
static int ion_cma_map_iommu(struct ion_buffer *buffer,
			     struct device *iommu_dev,
			     struct ion_iommu_map *data,
			     unsigned long iova_length,
			     unsigned long flags)
{
	int ret = 0;
	struct ion_cma_buffer_info *info = buffer->priv_virt;

	data->iova_addr = rockchip_iovmm_map(iommu_dev,
					  info->table->sgl,
					  0,
					  iova_length);
	pr_debug("%s: map %x -> %lx\n", __func__, info->table->sgl->dma_address,
		 data->iova_addr);
	if (IS_ERR_VALUE(data->iova_addr)) {
		pr_err("%s: failed: %lx\n", __func__, data->iova_addr);
		ret = data->iova_addr;
		goto out;
	}

	data->mapped_size = iova_length;

out:
	return ret;
}

void ion_cma_unmap_iommu(struct device *iommu_dev, struct ion_iommu_map *data)
{
	pr_debug("%s: unmap %x@%lx\n",
		 __func__,
		 data->mapped_size,
		 data->iova_addr);
	rockchip_iovmm_unmap(iommu_dev, data->iova_addr);
}
#endif

static struct ion_heap_ops ion_cma_ops = {
	.allocate = ion_cma_allocate,
	.free = ion_cma_free,
	.map_dma = ion_cma_heap_map_dma,
	.unmap_dma = ion_cma_heap_unmap_dma,
	.phys = ion_cma_phys,
	.map_user = ion_cma_mmap,
	.map_kernel = ion_cma_map_kernel,
	.unmap_kernel = ion_cma_unmap_kernel,
#ifdef CONFIG_ROCKCHIP_IOMMU
	.map_iommu = ion_cma_map_iommu,
	.unmap_iommu = ion_cma_unmap_iommu,
#endif
};

struct ion_heap *ion_cma_heap_create(struct ion_platform_heap *data)
{
	struct ion_cma_heap *cma_heap;
	struct device *dev = NULL;

	cma_heap = kzalloc(sizeof(struct ion_cma_heap), GFP_KERNEL);

	if (!cma_heap)
		return ERR_PTR(-ENOMEM);

	cma_heap->heap.ops = &ion_cma_ops;
	/* get device from private heaps data, later it will be
	 * used to make the link with reserved CMA memory */
	cma_heap->dev = data->priv;
	dev = cma_heap->dev;

	/* register cma area linked to this heap */
	cma_heap->cma_area = data->priv2;

	cma_heap->heap.type = ION_HEAP_TYPE_DMA;

	if (cma_heap->cma_area) {
		unsigned long count, flags;
		spin_lock_irqsave(&ion_cma_lock, flags);
		count = cma_area_get_count(cma_heap->cma_area);
		total_ion_cma += count << PAGE_SHIFT;
		unused_ion_cma += count << PAGE_SHIFT;
		spin_unlock_irqrestore(&ion_cma_lock, flags);
	}

	return &cma_heap->heap;
}

void ion_cma_heap_destroy(struct ion_heap *heap)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(heap);

	if (cma_heap->cma_area) {
		unsigned long count, flags;
		spin_lock_irqsave(&ion_cma_lock, flags);
		count = cma_area_get_count(cma_heap->cma_area);
		total_ion_cma -= count << PAGE_SHIFT;
		unused_ion_cma -= count << PAGE_SHIFT;
		spin_unlock_irqrestore(&ion_cma_lock, flags);
	}

	kfree(cma_heap);
}
