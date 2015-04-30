/*
 * Copyright (c) 2014, Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uio.h>
#include <linux/kthread.h> // FixMe: which to include for schedule_timeout()?
#include <linux/semaphore.h> // FixMe: which to include for schedule_timeout()?

#include "tee_rpc_driver.h"
#include "tee_rpc_driver_abi.h"
#include "tee_rpc_memory.h"
#include <linux/uaccess.h> //FixMe: which to include for schedule_timeout()?
#include <linux/fs.h> //FixMe: which to include for schedule_timeout()?

#define WRITE_TO 1

#define TEE_RPC_MEMORY_KZALLOC_NB_RETRIES      5
#define TEE_RPC_MEMORY_KZALLOC_RETRY_TIMEOUT  50

#ifdef TEE_RPC_MAP_PAGES

static int iov_to_pages(struct iovec *iov, struct page ***pages,
			int *num_page, unsigned long *start)
{
	unsigned long addr;
	int npage;
	void *page;
	unsigned long offset;
	unsigned int i;

	addr = (unsigned long)iov->iov_base;
	npage = (iov->iov_len + PAGE_SIZE - 1) / PAGE_SIZE;
	offset = addr & ~PAGE_MASK;

	i=0;
	do{
		page = kzalloc(npage * sizeof(struct page *), GFP_KERNEL);
		if(page == NULL){
			schedule_timeout (TEE_RPC_MEMORY_KZALLOC_RETRY_TIMEOUT);
		}
	} while ((page == NULL) && (i++ < TEE_RPC_MEMORY_KZALLOC_NB_RETRIES));
	if (!page){
		pr_err("[tee_rpc_memory] iov_to_pages: No mem for page\n");
		return -ENOMEM;
	}

	/* pin the user pages */
	npage = get_user_pages_fast(addr, npage, WRITE_TO, page);
	if (npage < 0) {
		kfree(page);
		return npage;
	}

	*pages = page;
	*num_page = npage;
	*start = offset;
	return 0;
}

static bool are_pages_phys_contig(struct page **pages_list,
				  int num_of_pages)
{
	int i;

	for (i = 0; i < (num_of_pages - 1); i++) {
		if ((page_to_phys(pages_list[i]) + PAGE_SIZE) !=
		    page_to_phys(pages_list[i + 1]))
			return false;
	}

	/* If we traverse all pages then they all follow one another */
	return true;
}

#endif

static int copy_to_contig(struct iovec *iov, struct pvec *pvec_iov)
{
	int ret;
	void *address = NULL;
	unsigned int i;

	// Input parameter check
	if((iov == NULL) || (iov->iov_base == NULL) || (pvec_iov == NULL)){
		pr_err("[tee_rpc_memory] copy_to_contig: input parameter error!");
		return -EFAULT;
	}

	if (iov->iov_len > KMALLOC_MAX_SIZE) {
		pr_err("Trying to alloc more than %lu memory not from kmalloc",
		       KMALLOC_MAX_SIZE);
		return -ENOMEM;
	}

	i=0;
	do{
		address = kzalloc(iov->iov_len, GFP_KERNEL);
	if(address == NULL){
			schedule_timeout (TEE_RPC_MEMORY_KZALLOC_RETRY_TIMEOUT);
	}
	} while ((address == NULL) && (i++ < TEE_RPC_MEMORY_KZALLOC_NB_RETRIES));
	if (!address){
    pr_err("[tee_rpc_memory] copy_to_contig: No mem for address\n");
		return -ENOMEM;
	}

	pr_debug("copy_from_user address 0x%p, %zu",
		 iov->iov_base, iov->iov_len);

	if (copy_from_user(address, iov->iov_base, iov->iov_len)) {
		ret = -EFAULT;
		goto out_err;
	}

	pr_debug("virt_to_phys\n");

	pvec_iov->phys_addr = virt_to_phys(address);
	pvec_iov->user_addr = iov->iov_base;
	pvec_iov->len = iov->iov_len;
	pvec_iov->is_contig = false;

	return 0;

out_err:
	kfree(address);
	return ret;
}


void free_pvec_address(u32 phys_addr)
{
	u8 *phys_addr_local;

	if (phys_addr == 0x00000000) {
		pr_err("[tee_rpc_memory]  free_pvec_address: !!! physical address is 0!!!\n");
	} else {
		if((phys_addr_local = (u8 *)phys_to_virt(phys_addr)) != NULL)
			kfree(phys_addr_local);
	}
}

void free_pvec(int num_vec, struct pvec *vector)
{
	int i;

	if (vector == NULL)
		return;

	/*TODO when we have a combination of pages and kmalloc revisit this!!*/
	for (i = 0; i < num_vec; i++){
		free_pvec_address(vector[i].phys_addr);
	}

	kfree(vector);
}

int tee_rpc_map_user_pages(struct tee_message __user *user_msg, int *num_v,
			   struct pvec **phys_iov, u32 *service, int *cmd_id)
{
	struct tee_message kern_msg;
	struct pvec *pvec_iov = NULL;
	struct iovec *kiov = NULL;
	int i;
	int ret = 0;
	int iov_size;

	pr_debug("tee_rpc_map_user_pages start\n");

	/* access permissions to arg were already checked in tee_rpc_ioctl.
	   first we copy the overall message */
	if (__copy_from_user(&kern_msg, user_msg, sizeof(struct tee_message))) {
		pr_err("Failed reading input parameters\n");
		return -EFAULT;
	}

	*num_v = kern_msg.num_vectors;

	if (*num_v == 0 || kern_msg.iov == NULL) {
		pr_err("No vectors contained in the message\n");
		goto no_iov_data;
	}

	/* next verify the contained iovec array and copy it */
	if (*num_v > UIO_MAXIOV)
		return -EINVAL;

	iov_size = *num_v * sizeof(struct iovec);

	/* Get the IOV from userspace */
	ret = !access_ok(ACCESS_READ, (void __user *)kern_msg.iov, iov_size);
	if (ret)
		return -EFAULT;

	i=0;
	do{
		kiov = kzalloc(iov_size, GFP_KERNEL);
		if(kiov == NULL){
			schedule_timeout (TEE_RPC_MEMORY_KZALLOC_RETRY_TIMEOUT);
		}
	} while ((kiov == NULL) && (i++ < TEE_RPC_MEMORY_KZALLOC_NB_RETRIES));
	if (kiov == NULL){
		pr_err("No mem for kiov\n");
		return -ENOMEM;
	}

	if (copy_from_user(kiov, kern_msg.iov, iov_size)) {
		ret = -EFAULT;
		goto out;
	}

	i=0;
	do{
		pvec_iov = kzalloc(*num_v * sizeof(struct pvec), GFP_KERNEL);
		if(pvec_iov == NULL){
			schedule_timeout (TEE_RPC_MEMORY_KZALLOC_RETRY_TIMEOUT);
		}
	} while ((pvec_iov == NULL) && (i++ < TEE_RPC_MEMORY_KZALLOC_NB_RETRIES));
	if (pvec_iov == NULL) {
		pr_err("[tee_rpc_memory] tee_rpc_map_user_pages: No mem for pvec_iov\n");
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < *num_v; i++) {
		/* struct page **pages = NULL; */
		/* int num_pages; */
		/* unsigned long start; */
		/* if (iov_to_pages(&iov[i], &pages, &num_pages, &start)) */
		/* goto cleanup_err; */

		/* if (are_pages_phys_contig(pages, num_pages)) { */
		/* pr_err("Pages are contig\n"); */
		/* pvec_iov[i].is_contig = true; */
		/* pvec_iov[i].len = iov[i].iov_len; */
		/* pvec_iov[i].phys_addr = (void *)start; */
		/* } else { */
		ret = copy_to_contig(&kiov[i], &pvec_iov[i]);
		if (ret) {
			pr_err("failed to copy the user memory locally\n");
			goto out;
		}
		/* } */
	}

no_iov_data:
	/* return the internal vector to the caller */
	*phys_iov = pvec_iov;
	*service = kern_msg.service;
	*cmd_id = kern_msg.command_id;

	pr_debug("copy_from_user success\n");

out:
	kfree(kiov);

	if (ret)
		free_pvec(*num_v, pvec_iov);

	return ret;
}

int tee_rpc_unmap_user_pages(int num_v, struct pvec *kiov)
{
	int i;

	pr_debug("tee_rpc_unmap_user_pages start\n");

	for (i = 0; i < num_v; i++) {
		if (kiov[i].is_contig) {
			/* We have mapped the pages from userspace directly */
			/* TODO free all the struct page **pages */
#ifdef TEE_RPC_MAP_PAGES

			set_page_dirty(kiov[i].phys_addr);
			put_page(kiov[i].phys_addr);
#endif
		} else {
			/* We copied the buffer from userspace to kern mem so we
			 * must copy the results back
			 */
			if (copy_to_user(kiov[i].user_addr,
					 phys_to_virt(kiov[i].phys_addr),
					 kiov[i].len)) {
				pr_err("Failed to copy back buffer\n");
			}
		}
	}

	return 0;
}
