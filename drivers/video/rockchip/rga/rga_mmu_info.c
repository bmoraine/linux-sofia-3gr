/*
 * rockchip RGA(2D raster graphic acceleration unit) hardware driver.
 *
 * Copyright (C) 2014 Rockchip Electronics Co., Ltd.
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

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/pagemap.h>
#include <linux/seq_file.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/memory.h>
#include <linux/dma-mapping.h>
#include <linux/atomic.h>
#include <asm/cacheflush.h>
#include "rga_mmu_info.h"

#define KERNEL_SPACE_VALID    0xc0000000
static int rga_mem_size_cal(uint32_t mem,
			    uint32_t memsize, uint32_t *startaddr)
{
	uint32_t start, end;
	uint32_t pagecount;

	end = (mem + (memsize + PAGE_SIZE - 1)) >> PAGE_SHIFT;
	start = mem >> PAGE_SHIFT;
	pagecount = end - start;
	*startaddr = start;
	return pagecount;
}
static int rga_mmu_buf_get(struct rga_mmu_buf_t *t,
							uint32_t size)
{
	mutex_lock(&rga_service.lock);
	t->front += size;
	mutex_unlock(&rga_service.lock);
	return 0;
}

static void print_info_mmu(struct rga_req *req)
{
	pr_info("s:y_ad=%.8x, uv_ad=%.8x, v_a=%.8x, format=%d\n",
		req->src.yrgb_addr, req->src.uv_addr,
		req->src.v_addr, req->src.format);
	pr_info("s:act_w=%d, act_h=%d, vir_w=%d, vir_h=%d\n",
		req->src.act_w, req->src.act_h, req->src.vir_w, req->src.vir_h);
	pr_info("s:x_off=%.8x y_off=%.8x\n",
		req->src.x_offset, req->src.y_offset);

	pr_info("d:y_ad=%.8x, uv_ad=%.8x, v_ad=%.8x, format=%d\n",
		req->dst.yrgb_addr, req->dst.uv_addr,
		req->dst.v_addr, req->dst.format);
	pr_info("d:x_off=%.8x y_off=%.8x\n",
		req->dst.x_offset, req->dst.y_offset);
	pr_info("d:act_w=%d, act_h=%d, vir_w=%d, vir_h=%d\n",
		req->dst.act_w, req->dst.act_h, req->dst.vir_w, req->dst.vir_h);

	pr_info("c.xmin=%d,c.xmax=%d,c.ymin=%d,c.ymax=%d\n",
		req->clip.xmin, req->clip.xmax, req->clip.ymin, req->clip.ymax);

	pr_info("mmu_flag=%.8x\n", req->mmu_info.mmu_flag);
}

static int rga_mmu_buf_get_try(struct rga_mmu_buf_t *t,
							uint32_t size)
{
	mutex_lock(&rga_service.lock);
	if ((t->back - t->front) > t->size) {
		if (t->front + size > t->back - t->size) {
			mutex_unlock(&rga_service.lock);
			return -1;
		}
	} else {
		if ((t->front + size) > t->back) {
			mutex_unlock(&rga_service.lock);
			return -1;
		}

		if (t->front + size > t->size) {
			if (size > (t->back - t->size)) {
				mutex_unlock(&rga_service.lock);
				return -1;
			}

			t->front = 0;
		}
	}
	mutex_unlock(&rga_service.lock);

	return 0;
}


static int rga_buf_size_cal(uint32_t yrgb_addr,
			    uint32_t uv_addr,
			      uint32_t v_addr,
			      int format, uint32_t w, uint32_t h,
			      uint32_t *startaddr)
{
	uint32_t size_yrgb = 0;
	uint32_t size_uv = 0;
	uint32_t size_v = 0;
	uint32_t stride = 0;
	uint32_t start, end;
	uint32_t pagecount;

	switch (format) {
	case RGA_FORMAT_RGBA_8888:
		stride = (w * 4 + 3) & (~3);
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		pagecount = (size_yrgb + PAGE_SIZE - 1) >> PAGE_SHIFT;
		break;
	case RGA_FORMAT_RGBX_8888:
		stride = (w * 4 + 3) & (~3);
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		pagecount = (size_yrgb + PAGE_SIZE - 1) >> PAGE_SHIFT;
		break;
	case RGA_FORMAT_RGB_888:
		stride = (w * 3 + 3) & (~3);
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		pagecount = (size_yrgb + PAGE_SIZE - 1) >> PAGE_SHIFT;
		break;
	case RGA_FORMAT_BGRA_8888:
		size_yrgb = w * h * 4;
		start = yrgb_addr >> PAGE_SHIFT;
		pagecount = (size_yrgb + PAGE_SIZE - 1) >> PAGE_SHIFT;
		break;
	case RGA_FORMAT_RGB_565:
		stride = (w * 2 + 3) & (~3);
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		pagecount = (size_yrgb + PAGE_SIZE - 1) >> PAGE_SHIFT;
		break;
	case RGA_FORMAT_RGBA_5551:
		stride = (w * 2 + 3) & (~3);
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		pagecount = (size_yrgb + PAGE_SIZE - 1) >> PAGE_SHIFT;
		break;
	case RGA_FORMAT_RGBA_4444:
		stride = (w * 2 + 3) & (~3);
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		pagecount = (size_yrgb + PAGE_SIZE - 1) >> PAGE_SHIFT;
		break;
	case RGA_FORMAT_BGR_888:
		stride = (w * 3 + 3) & (~3);
		size_yrgb = stride * h;
		start = yrgb_addr >> PAGE_SHIFT;
		pagecount = (size_yrgb + PAGE_SIZE - 1) >> PAGE_SHIFT;
		break;

		    /* YUV FORMAT */
	case RGA_FORMAT_YCBCR_422_SP:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = stride * h;
		start = MIN(yrgb_addr, uv_addr);
		start >>= PAGE_SHIFT;
		end = MAX((yrgb_addr + size_yrgb), (uv_addr + size_uv));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pagecount = end - start;
		break;
	case RGA_FORMAT_YCBCR_422_P:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = ((stride >> 1) * h);
		size_v = ((stride >> 1) * h);
		start = MIN(MIN(yrgb_addr, uv_addr), v_addr);
		start = start >> PAGE_SHIFT;
		end = MAX(MAX((yrgb_addr + size_yrgb),
			      (uv_addr + size_uv)), (v_addr + size_v));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pagecount = end - start;
		break;
	case RGA_FORMAT_YCBCR_420_SP:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = (stride * (h >> 1));
		start = MIN(yrgb_addr, uv_addr);
		start >>= PAGE_SHIFT;
		end = MAX((yrgb_addr + size_yrgb), (uv_addr + size_uv));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pagecount = end - start;
		break;
	case RGA_FORMAT_YCBCR_420_P:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = ((stride >> 1) * (h >> 1));
		size_v = ((stride >> 1) * (h >> 1));
		start = MIN(MIN(yrgb_addr, uv_addr), v_addr);
		start >>= PAGE_SHIFT;
		end = MAX(MAX((yrgb_addr + size_yrgb),
			      (uv_addr + size_uv)), (v_addr + size_v));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pagecount = end - start;
		break;
	case RGA_FORMAT_YCRCB_422_SP:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = stride * h;
		start = MIN(yrgb_addr, uv_addr);
		start >>= PAGE_SHIFT;
		end = MAX((yrgb_addr + size_yrgb), (uv_addr + size_uv));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pagecount = end - start;
		break;
	case RGA_FORMAT_YCRCB_422_P:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = ((stride >> 1) * h);
		size_v = ((stride >> 1) * h);
		start = MIN(MIN(yrgb_addr, uv_addr), v_addr);
		start >>= PAGE_SHIFT;
		end = MAX(MAX((yrgb_addr + size_yrgb),
			      (uv_addr + size_uv)), (v_addr + size_v));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pagecount = end - start;
		break;
	case RGA_FORMAT_YCRCB_420_SP:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = (stride * (h >> 1));
		start = MIN(yrgb_addr, uv_addr);
		start >>= PAGE_SHIFT;
		end = MAX((yrgb_addr + size_yrgb), (uv_addr + size_uv));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pagecount = end - start;
		break;
	case RGA_FORMAT_YCRCB_420_P:
		stride = (w + 3) & (~3);
		size_yrgb = stride * h;
		size_uv = ((stride >> 1) * (h >> 1));
		size_v = ((stride >> 1) * (h >> 1));
		start = MIN(MIN(yrgb_addr, uv_addr), v_addr);
		start >>= PAGE_SHIFT;
		end = MAX(MAX((yrgb_addr + size_yrgb),
			      (uv_addr + size_uv)), (v_addr + size_v));
		end = (end + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		pagecount = end - start;
		break;
	default:
		pagecount = 0;
		start = 0;
		break;
	}
	*startaddr = start;
	return pagecount;
}

static uint32_t rga_map_pte(uint32_t memory, uint32_t i)
{
	uint32_t address = 0;
	pte_t *pte;
    /* */
	spinlock_t *ptl;
	unsigned long pfn;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;

	do {
		pgd = pgd_offset(current->mm, (memory + i) <<
				PAGE_SHIFT);
		if (!pgd_present(*pgd))
			break;

		pud = pud_offset(pgd, (memory + i) << PAGE_SHIFT);
		if (!pud_present(*pud))
			break;

		pmd = pmd_offset(pud, (memory + i) <<
				PAGE_SHIFT);
		if (!pmd_present(*pmd))
			break;

		pte = pte_offset_map_lock(current->mm, pmd, (memory + i) <<
			 PAGE_SHIFT,
			 &ptl);
		if (!pte_present(*pte)) {
			pte_unmap_unlock
				(pte,
				 ptl);
			break;
		}
		pfn = pte_pfn(*pte);
		address = ((pfn << PAGE_SHIFT) |
				(((unsigned long)((memory + i) << PAGE_SHIFT)) &
				 ~PAGE_MASK));
		pte_unmap_unlock(pte, ptl);

	} while (0);

	return address;
}

static int rga_map_user_memory(struct page **pages, uint32_t *pagetable,
			       uint32_t memory, uint32_t page_count)
{
	int32_t result;
	uint32_t i;
	uint32_t status;
	uint32_t address;

	status = 0;
	address = 0;

	do {
		down_read(&current->mm->mmap_sem);
		result =
		    get_user_pages(current, current->mm, memory << PAGE_SHIFT,
				   page_count, 1, 0, pages, NULL);
		up_read(&current->mm->mmap_sem);
		if (result <= 0 || result < page_count) {
			struct vm_area_struct *vma;

			if (result > 0) {
				down_read(&current->mm->mmap_sem);
				for (i = 0; i < result; i++)
					put_page(pages[i]);
				up_read(&current->mm->mmap_sem);
			}
			for (i = 0; i < page_count; i++) {
				vma =
				    find_vma(current->mm,
					     (memory + i) << PAGE_SHIFT);
				if (vma) {
					address = rga_map_pte(memory, i);
					pagetable[i] = address;
				} else {
					status = RGA_OUT_OF_RESOURCES;
					break;
				}
			}
			return status;
		}

		    /* Fill the page table. */
		for (i = 0; i < page_count; i++)
			pagetable[i] = page_to_phys(pages[i]);

		down_read(&current->mm->mmap_sem);
		for (i = 0; i < result; i++)
			put_page(pages[i]);
		up_read(&current->mm->mmap_sem);
		return 0;
	} while (0);
	return status;
}

static int rga_map_ion(struct sg_table *sg, uint32_t *memory,
		       int32_t page_count)
{
	uint32_t i;
	uint32_t status;
	uint32_t address;
	uint32_t mapped_size = 0;
	uint32_t len;
	struct scatterlist *sgl = sg->sgl;
	uint32_t sg_num = 0;
	uint32_t break_flag = 0;

	status = 0;
	address = 0;

	do {
		len = sg_dma_len(sgl) >> PAGE_SHIFT;
		address = sg_phys(sgl);
		for (i = 0; i < len; i++) {
			if (mapped_size + i >= page_count) {
				break_flag = 1;
				break;
			}
			memory[mapped_size + i] = address + (i << PAGE_SHIFT);
		}
		if (break_flag)
			break;
		mapped_size += len;
		sg_num += 1;
		sgl = sg_next(sgl);
	} while ((sgl) && (mapped_size < page_count) && (sg_num < sg->nents));
	return 0;
}

static int rga_mmu_info_bitblt_mode(struct rga_reg *reg,
				    struct rga_req *req)
{
	int src_mem_size, dst_mem_size;
	uint32_t src_start, dst_start;
	uint32_t i;
	uint32_t all_size;
	uint32_t *mmu_base, *mmu_p, *mmu_base_phys;
	int ret;
	int status = 0;
	uint32_t uv_size, v_size;
	struct page **pages = NULL;

	mmu_base = NULL;
	src_mem_size = 0;
	dst_mem_size = 0;

	do {
		src_mem_size =
		    rga_buf_size_cal(req->src.yrgb_addr, req->src.uv_addr,
				     req->src.v_addr, req->src.format,
				     req->src.vir_w,
				     req->src.act_h + req->src.y_offset,
				     &src_start);
		if (src_mem_size == 0)
			return -EINVAL;
		dst_mem_size =
		    rga_buf_size_cal(req->dst.yrgb_addr, req->dst.uv_addr,
				     req->dst.v_addr, req->dst.format,
				     req->dst.vir_w, req->dst.vir_h,
				     &dst_start);
		if (dst_mem_size == 0)
			return -EINVAL;

		/* Cal out the needed mem size */
		src_mem_size = (src_mem_size + 15) & (~15);
		all_size = src_mem_size + dst_mem_size + 1;

		if (rga_mmu_buf_get_try(&rga_mmu_buf,
			(all_size + 15) & (~15))) {
			pr_err("bitblt mode RGA Get MMU mem failed\n");
			print_info_mmu(req);
			status = -1;
			break;
		}

		mutex_lock(&rga_service.lock);
		mmu_base = rga_mmu_buf.buf_virtual +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mmu_base_phys = rga_mmu_buf.buf +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mutex_unlock(&rga_service.lock);
		pages = rga_mmu_buf.pages;

		if ((req->mmu_info.mmu_flag >> 8) & 1) {
			if (req->sg_src) {
				ret =
				    rga_map_ion(req->sg_src, &mmu_base[0],
						src_mem_size);
			} else {
				ret =
				    rga_map_user_memory(&pages[0], &mmu_base[0],
							src_start,
							src_mem_size);
				if (ret < 0) {
					pr_err("rga map src memory failed\n");
					status = ret;
					break;
				}
			}
		} else {
			mmu_p = mmu_base;
			if (req->src.yrgb_addr ==
			      (uint32_t)rga_service.pre_scale_buf) {
				for (i = 0; i < src_mem_size; i++)
					mmu_p[i] =
					    rga_service.pre_scale_buf[i];
			} else {
				for (i = 0; i < src_mem_size; i++)
					mmu_p[i] =
					    (uint32_t)((src_start +
							 i) << PAGE_SHIFT);
			}
		}
		if ((req->mmu_info.mmu_flag >> 10) & 1) {
			if (req->sg_dst) {
				ret =
				    rga_map_ion(req->sg_dst,
						&mmu_base[src_mem_size],
					       dst_mem_size);
			} else {
				ret =
				    rga_map_user_memory(&pages[src_mem_size],
							&mmu_base[src_mem_size],
						dst_start, dst_mem_size);
				if (ret < 0) {
					pr_err("rga map dst memory failed\n");
					status = ret;
					break;
				}
			}
		} else {
			mmu_p = mmu_base + src_mem_size;
			for (i = 0; i < dst_mem_size; i++)
				mmu_p[i] =
				    (uint32_t)((dst_start + i) << PAGE_SHIFT);
		}
		mmu_base[all_size] = mmu_base[all_size - 1];

		    /* zsq
		     * change the buf address in req struct
		     */
		req->mmu_info.base_addr = (unsigned long)mmu_base_phys>>2;
		uv_size =
		    (req->src.uv_addr -
		     (src_start << PAGE_SHIFT)) >> PAGE_SHIFT;
		v_size =
		    (req->src.v_addr -
		      (src_start << PAGE_SHIFT)) >> PAGE_SHIFT;
		req->src.yrgb_addr = (req->src.yrgb_addr & (~PAGE_MASK));
		req->src.uv_addr =
		    (req->src.uv_addr & (~PAGE_MASK)) |
		    (uv_size << PAGE_SHIFT);
		req->src.v_addr =
		    (req->src.v_addr & (~PAGE_MASK)) | (v_size << PAGE_SHIFT);
		uv_size =
		    (req->dst.uv_addr -
		     (dst_start << PAGE_SHIFT)) >> PAGE_SHIFT;
		req->dst.yrgb_addr =
		    (req->dst.yrgb_addr & (~PAGE_MASK)) |
			(src_mem_size << PAGE_SHIFT);
		req->dst.uv_addr =
		    (req->dst.uv_addr & (~PAGE_MASK)) |
			((src_mem_size + uv_size) << PAGE_SHIFT);

		    /*record the malloc buf for the cmd end to release */
		    reg->MMU_base = mmu_base;

		    /* flush data to DDR */
#ifdef CONFIG_X86
		    clflush_cache_range(mmu_base, (all_size + 1) << 2);

#else	/*  */
		dmac_flush_range(mmu_base, (mmu_base + all_size + 1));
		outer_flush_range(virt_to_phys(mmu_base),
				  virt_to_phys(mmu_base + all_size + 1));

#endif	/*  */
		rga_mmu_buf_get(&rga_mmu_buf, (all_size + 15) & (~15));
		reg->MMU_len = (all_size + 15) & (~15);
		status = 0;

		return status;
	} while (0);
	return status;
}

static int rga_mmu_info_color_palette_mode(struct rga_reg *reg,
					   struct rga_req *req)
{
	int src_mem_size, dst_mem_size, cmd_mem_size;
	uint32_t src_start, dst_start, cmd_start;
	struct page **pages = NULL;
	uint32_t i;
	uint32_t all_size;
	uint32_t *mmu_base = NULL, *mmu_base_phys;
	uint32_t *mmu_p;
	int ret, status = 0;
	uint32_t stride;
	uint8_t shift;
	uint16_t sw, byte_num;

	shift = 3 - (req->palette_mode & 3);
	sw = req->src.vir_w;
	byte_num = sw >> shift;
	stride = (byte_num + 3) & (~3);

	do {
		src_mem_size =
		    rga_mem_size_cal(req->src.yrgb_addr, stride, &src_start);
		if (src_mem_size == 0)
			return -EINVAL;
		dst_mem_size =
		    rga_buf_size_cal(req->dst.yrgb_addr, req->dst.uv_addr,
				     req->dst.v_addr, req->dst.format,
				      req->dst.vir_w, req->dst.vir_h,
				      &dst_start);
		if (dst_mem_size == 0)
			return -EINVAL;
		cmd_mem_size =
		    rga_mem_size_cal((uint32_t)rga_service.cmd_buff,
				     RGA_CMD_BUF_SIZE, &cmd_start);
		if (cmd_mem_size == 0)
			return -EINVAL;
		src_mem_size = (src_mem_size + 15) & (~15);
		dst_mem_size = (dst_mem_size + 15) & (~15);
		cmd_mem_size = (cmd_mem_size + 15) & (~15);
		all_size = src_mem_size + dst_mem_size + cmd_mem_size + 1;

		if (rga_mmu_buf_get_try(&rga_mmu_buf,
			(all_size + 15) & (~15))) {
			pr_err("RGA Get MMU mem failed\n");
			status = -1;
			break;
		}

		pages = rga_mmu_buf.pages;
		mutex_lock(&rga_service.lock);
		mmu_base = rga_mmu_buf.buf_virtual +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mmu_base_phys = rga_mmu_buf.buf +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mutex_unlock(&rga_service.lock);

		    /* map src addr */
		    if (req->src.yrgb_addr < KERNEL_SPACE_VALID) {
			ret =
			    rga_map_user_memory(&pages[cmd_mem_size],
						&mmu_base[cmd_mem_size],
						 src_start, src_mem_size);
			if (ret < 0) {
				pr_err("rga map src memory failed\n");
				status = ret;
				break;
			}
		} else {
			mmu_p = mmu_base + cmd_mem_size;
			for (i = 0; i < src_mem_size; i++) {
				mmu_p[i] =
				    (uint32_t)virt_to_phys(
					(uint32_t *)((src_start + i) <<
								 PAGE_SHIFT));
			}
		}

		    /* map dst addr */
		    if (req->src.yrgb_addr < KERNEL_SPACE_VALID) {
			ret =
			    rga_map_user_memory(&pages
						 [cmd_mem_size + src_mem_size],
						 &mmu_base[cmd_mem_size +
							    src_mem_size],
						 dst_start, dst_mem_size);
			if (ret < 0) {
				pr_err("rga map dst memory failed\n");
				status = ret;
				break;
			}
		} else {
			mmu_p = mmu_base + cmd_mem_size + src_mem_size;
			for (i = 0; i < dst_mem_size; i++) {
				mmu_p[i] =
				    (uint32_t)virt_to_phys(
						(uint32_t *)((dst_start + i) <<
								 PAGE_SHIFT));
			}
		}

		    /* zsq
		     * change the buf address in req struct
		     * for the reason of lie to MMU
		     */
		req->mmu_info.base_addr = (unsigned long)(mmu_base_phys) >> 2;
		req->src.yrgb_addr =
		    (req->src.yrgb_addr & (~PAGE_MASK)) |
		    (cmd_mem_size << PAGE_SHIFT);
		req->dst.yrgb_addr =
		    (req->dst.yrgb_addr & (~PAGE_MASK)) |
		    ((cmd_mem_size + src_mem_size) << PAGE_SHIFT);

		    /*record the malloc buf for the cmd end to release */
		    reg->MMU_base = mmu_base;

		    /* flush data to DDR */

#ifdef CONFIG_X86
		    clflush_cache_range(mmu_base, (all_size + 1) << 2);

#else	/*  */
		dmac_flush_range(mmu_base, (mmu_base + all_size + 1));
		outer_flush_range(virt_to_phys(mmu_base),
				  virt_to_phys(mmu_base + all_size + 1));

#endif	/*  */
		rga_mmu_buf_get(&rga_mmu_buf, (all_size + 15) & (~15));
		reg->MMU_len = (all_size + 15) & (~15);
		return status;
	} while (0);

	return 0;
}

static int rga_mmu_info_color_fill_mode(struct rga_reg *reg,
					struct rga_req *req)
{
	int dst_mem_size;
	uint32_t dst_start;
	struct page **pages = NULL;
	uint32_t i;
	uint32_t all_size;
	uint32_t *mmu_base, *mmu_p, *mmu_base_phys;
	int ret;
	int status = 0;

	mmu_base = NULL;

	do {
		dst_mem_size =
		    rga_buf_size_cal(req->dst.yrgb_addr, req->dst.uv_addr,
				     req->dst.v_addr, req->dst.format,
				      req->dst.vir_w, req->dst.vir_h,
				      &dst_start);
		if (dst_mem_size == 0)
			return -EINVAL;
		all_size = dst_mem_size + 1;

		if (rga_mmu_buf_get_try(&rga_mmu_buf,
			(all_size + 15) & (~15))) {
			pr_err("color fill RGA Get MMU mem failed\n");
			print_info_mmu(req);
			status = -1;
			break;
		}

		mutex_lock(&rga_service.lock);
		mmu_base = rga_mmu_buf.buf_virtual +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mmu_base_phys = rga_mmu_buf.buf +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mutex_unlock(&rga_service.lock);
		pages = rga_mmu_buf.pages;

		if ((req->mmu_info.mmu_flag >> 10) & 1) {
			if (req->sg_dst) {
				ret =
				    rga_map_ion(req->sg_dst, &mmu_base[0],
						dst_mem_size);
			} else {
				ret =
				    rga_map_user_memory(&pages[0],
							&mmu_base[0],
							 dst_start,
							 dst_mem_size);
				if (ret < 0) {
					pr_err("rga map dst memory failed\n");
					status = ret;
					break;
				}
			}
		} else {
			mmu_p = mmu_base;
			for (i = 0; i < dst_mem_size; i++)
				mmu_p[i] =
				    (uint32_t)((dst_start + i) << PAGE_SHIFT);
		}
		mmu_base[all_size] = mmu_base[all_size - 1];

		    /* zsq
		     * change the buf address in req struct
		     */
		req->mmu_info.base_addr = (unsigned long)mmu_base_phys >> 2;
		req->dst.yrgb_addr = (req->dst.yrgb_addr & (~PAGE_MASK));

		    /*record the malloc buf for the cmd end to release */
		    reg->MMU_base = mmu_base;

		    /* flush data to DDR */
#ifdef CONFIG_X86
		    clflush_cache_range(mmu_base, (all_size + 1) << 2);

#else	/*  */
		dmac_flush_range(mmu_base, (mmu_base + all_size + 1));
		outer_flush_range(virt_to_phys(mmu_base),
				  virt_to_phys(mmu_base + all_size + 1));

#endif	/*  */
		rga_mmu_buf_get(&rga_mmu_buf, (all_size + 15) & (~15));
		reg->MMU_len = (all_size + 15) & (~15);
		return 0;
	} while (0);
	return status;
}

static int rga_mmu_info_line_point_drawing_mode(struct rga_reg *reg,
						struct rga_req *req)
{
	return 0;
}

static int rga_mmu_info_blur_sharp_filter_mode(struct rga_reg *reg,
					       struct rga_req *req)
{
	return 0;
}

static int rga_mmu_info_pre_scale_mode(struct rga_reg *reg,
				       struct rga_req *req)
{
	int src_mem_size, dst_mem_size;
	uint32_t src_start, dst_start;
	struct page **pages = NULL;
	uint32_t i;
	uint32_t all_size;
	uint32_t *mmu_base, *mmu_p, *mmu_base_phys;
	int ret;
	int status = 0;
	uint32_t uv_size, v_size;

	mmu_base = NULL;

	do {
		    /* cal src buf mmu info */
		    src_mem_size =
		    rga_buf_size_cal(req->src.yrgb_addr, req->src.uv_addr,
				     req->src.v_addr, req->src.format,
				      req->src.vir_w, req->src.vir_h,
				      &src_start);
		if (src_mem_size == 0)
			return -EINVAL;

		    /* cal dst buf mmu info */
		    dst_mem_size =
		    rga_buf_size_cal(req->dst.yrgb_addr, req->dst.uv_addr,
				     req->dst.v_addr, req->dst.format,
				      req->dst.vir_w, req->dst.vir_h,
				      &dst_start);
		if (dst_mem_size == 0)
			return -EINVAL;
		src_mem_size = (src_mem_size + 15) & (~15);
		all_size = src_mem_size + dst_mem_size + 1;

		if (rga_mmu_buf_get_try(&rga_mmu_buf,
			(all_size + 15) & (~15))) {
			pr_err("pre scale RGA Get MMU mem failed\n");
			print_info_mmu(req);
			status = -1;
			break;
		}

		    /*
		     * Allocate MMU Index mem
		     * This mem release in run_to_done fun
		     */
		pages = rga_mmu_buf.pages;
		mutex_lock(&rga_service.lock);
		mmu_base = rga_mmu_buf.buf_virtual +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mmu_base_phys = rga_mmu_buf.buf +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mutex_unlock(&rga_service.lock);

		    /* map src pages */
		    if ((req->mmu_info.mmu_flag >> 8) & 1) {
			if (req->sg_src) {
				ret =
				    rga_map_ion(req->sg_src, &mmu_base[0],
						src_mem_size);
			} else {
				ret =
				    rga_map_user_memory(&pages[0],
							&mmu_base[0],
							src_start,
							src_mem_size);
				if (ret < 0) {
					pr_err("rga map src memory failed\n");
					status = ret;
					break;
				}
			}
		} else {
			mmu_p = mmu_base;
			for (i = 0; i < src_mem_size; i++)
				mmu_p[i] =
				    (uint32_t)((src_start + i) << PAGE_SHIFT);
		}
		if ((req->mmu_info.mmu_flag >> 10) & 1) {
			if (req->sg_dst) {
				ret =
				    rga_map_ion(req->sg_dst,
						&mmu_base[src_mem_size],
						dst_mem_size);
			} else {
				ret =
				    rga_map_user_memory(&pages[src_mem_size],
							&mmu_base
							 [src_mem_size],
							 dst_start,
							 dst_mem_size);
				if (ret < 0) {
					pr_err("rga map dst memory failed\n");
					status = ret;
					break;
				}
			}
		} else {
			/* kernel space */
			mmu_p = mmu_base + src_mem_size;
			if (req->dst.yrgb_addr ==
			      (uint32_t)rga_service.pre_scale_buf) {
				for (i = 0; i < dst_mem_size; i++)
					mmu_p[i] =
					    rga_service.pre_scale_buf[i];
			} else {
				for (i = 0; i < dst_mem_size; i++)
					mmu_p[i] = (uint32_t)((dst_start + i) <<
							PAGE_SHIFT);
			}
		}
		mmu_base[all_size] = mmu_base[all_size - 1];

		    /* zsq
		     * change the buf address in req struct
		     * for the reason of lie to MMU
		     */
		req->mmu_info.base_addr = (unsigned long)(mmu_base_phys) >> 2;
		uv_size =
		    (req->src.uv_addr -
		     (src_start << PAGE_SHIFT)) >> PAGE_SHIFT;
		v_size =
		    (req->src.v_addr -
		      (src_start << PAGE_SHIFT)) >> PAGE_SHIFT;
		req->src.yrgb_addr  = (req->src.yrgb_addr & (~PAGE_MASK));
		req->src.uv_addr =
		    (req->src.uv_addr & (~PAGE_MASK)) |
		    (uv_size << PAGE_SHIFT);
		req->src.v_addr =
		    (req->src.v_addr & (~PAGE_MASK)) | (v_size << PAGE_SHIFT);
		uv_size =
		    (req->dst.uv_addr -
		     (dst_start << PAGE_SHIFT)) >> PAGE_SHIFT;
		v_size =
		    (req->dst.v_addr -
		      (dst_start << PAGE_SHIFT)) >> PAGE_SHIFT;
		req->dst.yrgb_addr =
		    (req->dst.yrgb_addr & (~PAGE_MASK)) |
		    ((src_mem_size) << PAGE_SHIFT);
		req->dst.uv_addr =
		    (req->dst.uv_addr & (~PAGE_MASK)) |
		    ((src_mem_size + uv_size) << PAGE_SHIFT);
		req->dst.v_addr =
		    (req->dst.v_addr & (~PAGE_MASK)) |
		    ((src_mem_size + v_size) << PAGE_SHIFT);

		    /*record the malloc buf for the cmd end to release */
		    reg->MMU_base = mmu_base;

		    /* flush data to DDR */
#ifdef CONFIG_X86
		    clflush_cache_range(mmu_base, (all_size + 1) << 2);

#else	/*  */
		dmac_flush_range(mmu_base, (mmu_base + all_size + 1));
		outer_flush_range(virt_to_phys(mmu_base),
				  virt_to_phys(mmu_base + all_size + 1));

#endif	/*  */
		rga_mmu_buf_get(&rga_mmu_buf, (all_size + 15) & (~15));
		reg->MMU_len = (all_size + 15) & (~15);
		return 0;
	} while (0);
	return status;
}

static int rga_mmu_info_update_palette_table_mode(struct rga_reg *reg,
						  struct rga_req *req)
{
	int src_mem_size, cmd_mem_size;
	uint32_t src_start, cmd_start;
	struct page **pages = NULL;
	uint32_t i;
	uint32_t all_size;
	uint32_t *mmu_base, *mmu_p, *mmu_base_phys;
	int ret, status = 0;

	mmu_base = NULL;

	do {
		    /* cal src buf mmu info */
		src_mem_size =
		rga_mem_size_cal(req->src.yrgb_addr,
				 req->src.vir_w * req->src.vir_h,
				      &src_start);
		if (src_mem_size == 0)
			return -EINVAL;

		    /* cal cmd buf mmu info */
		cmd_mem_size =
		    rga_mem_size_cal((uint32_t)rga_service.cmd_buff,
				     RGA_CMD_BUF_SIZE, &cmd_start);
		if (cmd_mem_size == 0)
			return -EINVAL;
		src_mem_size = (src_mem_size + 15) & (~15);
		all_size = src_mem_size + cmd_mem_size + 1;

		if (rga_mmu_buf_get_try(&rga_mmu_buf,
			(all_size + 15) & (~15))) {
			pr_err("RGA Get MMU mem failed\n");
			status = -1;
			break;
		}

		pages = rga_mmu_buf.pages;
		mutex_lock(&rga_service.lock);
		mmu_base = rga_mmu_buf.buf_virtual +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mmu_base_phys = rga_mmu_buf.buf +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mutex_unlock(&rga_service.lock);

		for (i = 0; i < cmd_mem_size; i++)
			mmu_base[i] =
			    virt_to_phys((uint32_t *)((cmd_start + i) <<
							PAGE_SHIFT));
		if (req->src.yrgb_addr < KERNEL_SPACE_VALID) {
			ret =
			    rga_map_user_memory(&pages[cmd_mem_size],
						&mmu_base[cmd_mem_size],
						 src_start, src_mem_size);
			if (ret < 0) {
				pr_err("rga map src memory failed\n");
				return -EINVAL;
			}
		} else {
			mmu_p = mmu_base + cmd_mem_size;
			for (i = 0; i < src_mem_size; i++)
				mmu_p[i] = (uint32_t)virt_to_phys(
						(uint32_t *)((src_start + i) <<
						PAGE_SHIFT));
		}

		    /* zsq
		     * change the buf address in req struct
		     * for the reason of lie to MMU
		     */
		req->mmu_info.base_addr = (unsigned long)(mmu_base_phys) >> 2;
		req->src.yrgb_addr
		    = (req->src.yrgb_addr & (~PAGE_MASK)) |
		    (cmd_mem_size << PAGE_SHIFT);

		    /*record the malloc buf for the cmd end to release */
		reg->MMU_base = mmu_base;

		/* flush data to DDR */
#ifdef CONFIG_X86
		clflush_cache_range(mmu_base, (all_size + 1) << 2);

#else	/*  */
		dmac_flush_range(mmu_base, (mmu_base + all_size));
		outer_flush_range(virt_to_phys(mmu_base),
				  virt_to_phys(mmu_base + all_size));

#endif	/*  */
		rga_mmu_buf_get(&rga_mmu_buf, (all_size + 15) & (~15));
		reg->MMU_len = (all_size + 15) & (~15);

		return 0;
	} while (0);
	return status;
}

static int rga_mmu_info_update_patten_buff_mode(struct rga_reg *reg,
						struct rga_req *req)
{
	int src_mem_size, cmd_mem_size;
	uint32_t src_start, cmd_start;
	struct page **pages = NULL;
	uint32_t i;
	uint32_t all_size;
	uint32_t *mmu_base, *mmu_p, *mmu_base_phys;
	int ret, status = 0;

	mmu_base = 0;
	mmu_p = 0;

	do {
		src_mem_size =
		    rga_mem_size_cal(req->pat.yrgb_addr,
				     req->pat.vir_w * req->pat.vir_h * 4,
				     &src_start);
		if (src_mem_size == 0)
			return -EINVAL;

		    /* cal cmd buf mmu info */
		    cmd_mem_size =
		    rga_mem_size_cal((uint32_t)rga_service.cmd_buff,
				     RGA_CMD_BUF_SIZE, &cmd_start);
		if (cmd_mem_size == 0)
			return -EINVAL;
		src_mem_size = (src_mem_size + 15) & (~15);
		all_size = src_mem_size + cmd_mem_size + 1;

		if (rga_mmu_buf_get_try(&rga_mmu_buf,
			(all_size + 15) & (~15))) {
			pr_err("RGA Get MMU mem failed\n");
			status = -1;
			break;
		}

		pages = rga_mmu_buf.pages;
		mutex_lock(&rga_service.lock);
		mmu_base = rga_mmu_buf.buf_virtual +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mmu_base_phys = rga_mmu_buf.buf +
			(rga_mmu_buf.front & (rga_mmu_buf.size - 1));
		mutex_unlock(&rga_service.lock);

		for (i = 0; i < cmd_mem_size; i++) {
			mmu_base[i] =
			    virt_to_phys((uint32_t *)((cmd_start + i) <<
							PAGE_SHIFT));
		}
		if (req->src.yrgb_addr < KERNEL_SPACE_VALID) {
			ret =
			    rga_map_user_memory(&pages[cmd_mem_size],
						&mmu_base[cmd_mem_size],
						 src_start, src_mem_size);
			if (ret < 0) {
				pr_err("rga map src memory failed\n");
				status = ret;
				break;
			}
		} else {
			mmu_p = mmu_base + cmd_mem_size;
			for (i = 0; i < src_mem_size; i++)
				mmu_p[i] = (uint32_t)virt_to_phys(
						(uint32_t *)((src_start + i) <<
						PAGE_SHIFT));
		}

		    /* zsq
		     * change the buf address in req struct
		     * for the reason of lie to MMU
		     */
		req->mmu_info.base_addr = (unsigned long)mmu_base_phys >> 2;
		req->src.yrgb_addr =
		    (req->src.yrgb_addr & (~PAGE_MASK)) |
		    (cmd_mem_size << PAGE_SHIFT);
		reg->MMU_base = mmu_base;

		    /* flush data to DDR */
#ifdef CONFIG_X86
		clflush_cache_range(mmu_base, (all_size + 1) << 2);

#else	/*  */
		dmac_flush_range(mmu_base, (mmu_base + all_size));
		outer_flush_range(virt_to_phys(mmu_base),
				  virt_to_phys(mmu_base + all_size));

#endif	/*  */
		rga_mmu_buf_get(&rga_mmu_buf, (all_size + 15) & (~15));
		reg->MMU_len = (all_size + 15) & (~15);
		return 0;
	} while (0);

	return status;
}

int rga_set_mmu_info(struct rga_reg *reg, struct rga_req *req)
{
	int ret;

	switch (req->render_mode) {
	case bitblt_mode:
		ret = rga_mmu_info_bitblt_mode(reg, req);
		break;
	case color_palette_mode:
		ret = rga_mmu_info_color_palette_mode(reg, req);
		break;
	case color_fill_mode:
		ret = rga_mmu_info_color_fill_mode(reg, req);
		break;
	case line_point_drawing_mode:
		ret = rga_mmu_info_line_point_drawing_mode(reg, req);
		break;
	case blur_sharp_filter_mode:
		ret = rga_mmu_info_blur_sharp_filter_mode(reg, req);
		break;
	case pre_scaling_mode:
		ret = rga_mmu_info_pre_scale_mode(reg, req);
		break;
	case update_palette_table_mode:
		ret = rga_mmu_info_update_palette_table_mode(reg, req);
		break;
	case update_patten_buff_mode:
		ret = rga_mmu_info_update_patten_buff_mode(reg, req);
		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}
