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

#include <linux/semaphore.h>
#include <linux/io.h>
#include "sec_shared_mem.h"

DEFINE_SEMAPHORE(sec_malloc_sem);

#define MAGIC_WORD (0xDEADBEEF)
#define HEAP_SIZE (524288L+32768L)
unsigned char heap[HEAP_SIZE];
#define HEAP ((void *)heap)

int n_alloc = 0;

unsigned int sec_heap_start_addr = (unsigned int)&heap[0];
unsigned int sec_heap_stop_addr = (unsigned int)&heap[HEAP_SIZE-1];

typedef enum{
	ALLOC,
	FREE
} t_mem_stat;

struct t_mem_elm {
	int magic;
	int size;
	unsigned char *data;
	t_mem_stat status;
	struct t_mem_elm *next;
};

typedef struct t_mem_elm t_mem;

t_mem mem_hdr = {
	MAGIC_WORD, HEAP_SIZE, HEAP, FREE, NULL};


void *sec_shared_mem_alloc(size_t size)
{
	void *pret = NULL;
	t_mem *mem = &mem_hdr;
	size_t size_alloc = size;

	/* input check */
	if (size == 0) {
		pr_err("[sec_shared_mem] sec_shared_mem_alloc: cannot allocate 0 bytes\n");
		return pret;
	}

	/* ensure data is aligned on integer multiple addresses */
	if (size_alloc % sizeof(int)) {
		size_alloc += sizeof(int) -
			(size_alloc % sizeof(int));
	}

	/* lock the semaphore */
	if (down_interruptible(&sec_malloc_sem)) {
		pr_err("[sec_shared_mem] sec_shared_mem_alloc: semaphore aquire interupted\n");
		return pret;
	}

	while (NULL != mem) {
		if ((mem->status == FREE) && (mem->size >= size)) {
			mem->status = ALLOC;
			/* do we need to cut the block */
			if (mem->size-size > sizeof(t_mem)) {
				t_mem *mem_nxt;

				/* update next block */
				mem_nxt = (t_mem *)&mem->data[size];
				mem_nxt->magic = MAGIC_WORD;
				mem_nxt->data =
					&mem->data[size +
					sizeof(t_mem)];
				mem_nxt->size =
					mem->size-size-sizeof(t_mem);
				mem_nxt->status = FREE;
				mem_nxt->next = mem->next;
				mem->next = mem_nxt;

				/* update this block */
				mem->size = size;
			}

			pret = (void *)mem->data;
			goto sec_malloc_return;
		}

		mem = mem->next;
	}

sec_malloc_return:
	if (mem == NULL) {
		pr_err("[sec_shared_mem] sec_shared_mem_alloc: cannot allocate %d bytes\n",
			size);
	} else {
		/* zero memory */
		memset(pret, 0x00, size_alloc);
		/* update total alloc/free counter */
		++n_alloc;
	}

	/* release the semaphore */
	up(&sec_malloc_sem);

	return pret;
}

void sec_shared_mem_free(void *ptr)
{
	t_mem *mem = &mem_hdr;
	t_mem *mem_prev = NULL;

	/* input check */
	if (ptr == NULL) {
		pr_err("[sec_shared_mem] sec_shared_mem_free: pointer is already freed\n");
		return;
	}
	if (((unsigned int)ptr < sec_heap_start_addr) ||
			((unsigned int)ptr > sec_heap_stop_addr)) {
		pr_err("[sec_shared_mem] sec_shared_mem_free: cannot free pointer outside bounds\n");
		return;
	}

	/* lock the semaphore */
	if (down_interruptible(&sec_malloc_sem)) {
		pr_err("[sec_shared_mem] sec_shared_mem_free: semaphore aquire interupted\n");
		return;
	}

	while (NULL != mem) {
		if (mem->status == ALLOC && mem->data == ptr) {
			/*  check magic word */
			if (mem->magic != MAGIC_WORD) {
				pr_err("[sec_shared_mem] sec_shared_mem_free: memory has been overwritten!\n");
				goto sec_free_return;
			}

			/* update status for this block */
			mem->status = FREE;

			/* if next block is free */
			/* then include it in this block */
			if (NULL != mem->next) {
				if (FREE == mem->next->status) {
					mem->size =
						mem->size +
						sizeof(t_mem) +
						mem->next->size;
					mem->next = mem->next->next;
				}
			}

			/* if previous block is free */
			/* then include it in this block */
			if (NULL != mem_prev) {
				if (FREE == mem_prev->status) {
					mem_prev->size =
						mem_prev->size +
						sizeof(t_mem) +
						mem->size;
					mem_prev->next = mem->next;
				}
			}

			ptr = NULL;
			goto sec_free_return;
		}

		mem_prev = mem;
		mem = mem->next;
	}

	if (ptr != NULL)
		pr_err("[sec_shared_mem] sec_shared_mem_free: cannot free memory - unknown allocation!\n");

sec_free_return:
	if (--n_alloc < 0) {
		pr_err("[sec_shared_mem] sec_shared_mem_free: %d allocations!\n",
			n_alloc);
	}
	/* release the semaphore */
	up(&sec_malloc_sem);

	return;
}
