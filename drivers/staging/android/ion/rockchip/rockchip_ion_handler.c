/*
 * drivers/staging/android/ion/rockchip/rockchip_ion_handler.c
 *
 * Copyright (C) 2014 Jianqun.xu <xjq@rock-chips.com>
 * Copyright (C) 2014 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 */

#include <linux/fs.h>
#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/rockchip_ion.h>

#include "../ion_priv.h"

#define RK_ION_VBPIPE  "/dev/mvpipe-vpu_ion"
#define RK_MAX_CMD_LEN 10

/*
 * internal data
 */
static struct task_struct *rk_ion_task;
static struct ion_device *rk_ion_device;
static struct ion_client *rk_ion_client;
static struct file *rk_ion_vbpipe_filep;
static unsigned int rk_ion_heap_type_mask;
static uint32_t rk_ion_heap_capacity;

/*
 * internal functions
 */
static int rk_ion_handler(void *data);
static int rk_ion_vbpipe_handler(struct file *vbpipe);
static int rk_ion_handle_command(uint32_t cmd[], int len);


/*
 * initialize/start handler thread
 */
int rk_ion_handler_init(struct device_node *node, struct ion_device *idev,
			struct ion_platform_data *pdata)
{
	int i;

	rk_ion_device = idev;
	/*
	 * determine heap and size to be used:
	 * - defaults to ION_HEAP_TYPE_DMA_MASK
	 * - switch to ION_HEAP_TYPE_SECURE, if present
	 *   store size accordingly
	 */
	/* default settings */
	rk_ion_heap_type_mask	= ION_HEAP_TYPE_DMA_MASK;
	rk_ion_heap_capacity	= 0;

	if (pdata != NULL) {
		for (i = 0; i < pdata->nr; i++) {
			struct ion_platform_heap *heap_data =  &pdata->heaps[i];

			if (heap_data == NULL)
				continue;

			pr_debug("rk_ion_h: heap %s type %u id %u size %u\n",
				heap_data->name, heap_data->type,
				heap_data->id, heap_data->size);

			/* secure heap is present; use this one */
			if (heap_data->type == ION_HEAP_TYPE_DMA &&
				heap_data->id == ION_HEAP_TYPE_SECURE2) {

				rk_ion_heap_capacity = heap_data->size;
				rk_ion_heap_type_mask =
					ION_HEAP_TYPE_SECURE2_MASK;
				pr_info("rk_ion VPU uses %s size %u\n",
					heap_data->name,
					rk_ion_heap_capacity);
			}
		}
	}

	if (of_property_read_bool(node, "secvm-handler")) {
		pr_debug("rk_ion Init secure vm handler\n");

		rk_ion_client = ion_client_create(idev, "secure_vm");
		if (rk_ion_client == NULL) {
			pr_err("rk_ion cannot create ion client\n");
			pr_err("rk_ion cannot create secvm handler\n");

			goto end;
		}

		pr_debug("rk_ion created client for secure vm\n");

		rk_ion_task = kthread_run(&rk_ion_handler,
			(void *)idev, "ion_secvm_handler");

		if (rk_ion_task != NULL)
			pr_debug("rk_ion thread started: %s\n",
				rk_ion_task->comm);
		else {
			pr_err("rk_ion error starting secvm handler\n");
			ion_client_destroy(rk_ion_client);
		}
	} else
		pr_debug("rk_ion no secvm handler\n");

end:
	return 0;
}

/*
 * terminate handler thread, if present
 */
void rk_ion_handler_exit(void)
{
	if (rk_ion_task != NULL) {
		kthread_stop(rk_ion_task);
		rk_ion_task = NULL;
	}

	if (rk_ion_client != NULL) {
		ion_client_destroy(rk_ion_client);
		rk_ion_client = NULL;
	}
}



/*
 * ==========================================================
 * internal functions
 * ==========================================================
 */

/*
 * handler thread main function:
 * - running main loop
 * -- open vbpipe (have to loop)
 * -- handler loop
 * -- close vbpipe on error
 */
static int rk_ion_handler(void *data)
{
	int ret = 0;
	int i = 0;
	int error = 0;
	struct file *fp = NULL;
	mm_segment_t old_fs;

	/* struct ion_device *idev = (struct ion_device *)data; */

	pr_debug("rk_ion starting handler thread\n");

	while (!kthread_should_stop()) {

		/*
		 * try to open vbpipe: on failure, loop and try again
		 * (during boot time the file system may not yet be mounted)
		 */
		while (!kthread_should_stop() &&
			rk_ion_vbpipe_filep == NULL) {
			/*
			 * open vbpipe to secure vm
			 */
			old_fs = get_fs();
			set_fs(KERNEL_DS);

			pr_debug("rk_ion handler thread - open vbpipe\n");

			fp = filp_open(RK_ION_VBPIPE, O_RDWR, 0);

			set_fs(old_fs);

			if (IS_ERR(fp)) {
				pr_debug("rk_ion open vbpipe error %d\n",
					(int)fp);
				rk_ion_vbpipe_filep = NULL;

				/*
				 * on error wait, before trying again
				 */
				msleep(2000);

			} else {
				pr_info("rk_ion %s link 0x%p established\n",
					RK_ION_VBPIPE, fp);

				/*
				 * open OK, leave loop
				 */
				rk_ion_vbpipe_filep = fp;
			}

			i++;
		} /* open loop */

		/*
		 * main handler loop
		 * the loop is terminated on error or if the thread
		 * is cancelled.
		 */
		pr_debug("rk_ion: handler thread - entering read loop\n");

		while (!kthread_should_stop() && error == 0) {
			pr_debug("rk_ion wait for request on vbpipe\n");

			if (rk_ion_vbpipe_filep != NULL)
				error = rk_ion_vbpipe_handler(
						rk_ion_vbpipe_filep);
		} /* main handler loop */


		pr_debug("rk_ion: handler leaving loop, close vbpipe");

		/*
		 * left main handler loop; close vbpipe
		 */
		if (rk_ion_vbpipe_filep != NULL) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);

			ret = filp_close(rk_ion_vbpipe_filep, NULL);
			rk_ion_vbpipe_filep = NULL;

			set_fs(old_fs);

			pr_info("rk_ion vbpipe closed (%d)", ret);
		}
	} /* the big loop */

	/*do_exit(1);*/
	return 0;
}

/*
 * read request, handle it, and send response
 */
static int rk_ion_vbpipe_handler(struct file *filep)
{
	int ret = 0;
	mm_segment_t old_fs;
	int must;
	int done;
	int done_total;
	uint32_t buf[RK_MAX_CMD_LEN];
	unsigned char *datap;

	datap	   = (unsigned char *)buf;
	must	   = sizeof(buf);
	done_total = 0;

	while (must > 0) {
		done = 0;

		old_fs = get_fs();

		set_fs(KERNEL_DS);
		{
			done = filep->f_op->read(filep, datap, must, 0);
		}
		set_fs(old_fs);

		if (done < 0) {
			pr_err("rk_ion error %d reading vbpipe", done);
			ret = -1;
			break;
		}

		datap += done;
		done_total += done;
		must	 -= done;
	}

	if (ret == 0) {
		/*
		 * handle the command received
		 * (no error handling of this function, as we always
		 * return an error to the caller in secure vm)
		 */
		rk_ion_handle_command(buf, RK_MAX_CMD_LEN);

		datap = (unsigned char *)buf;
		must = sizeof(buf);
		done_total = 0;

		while (must > 0) {
			done = 0;

			old_fs = get_fs();

			set_fs(KERNEL_DS);
			{
				done = filep->f_op->write(filep,
					datap, must, 0);
			}
			set_fs(old_fs);

			if (done < 0) {
				pr_err("rk_ion error %d writing vbpipe",
					done);
				ret = -2;
				break;
			}

			datap	   += done;
			done_total += done;
			must	   -= done;
		}
	}

	return ret;
}


/*
 * process command received from vbpipe
 * do not return an error, but specify the error in the command
 * data structure/array to be returned to the caller
 *
 * command layout:
 * +----+----------------------------------------+
 * |  0 | magic: 0xb33f0a01 (versioning??)	 |
 * +----+----------------------------------------+
 * |  1 | command (1 = allocate, 2 = free)	 |
 * +----+----------------------------------------+
 * |  2 | result (0 = OK, error != 0)		 |
 * +----+----------------------------------------+
 * |  3 | size					 |
 * +----+----------------------------------------+
 * |  4 | bus address				 |
 * +----+----------------------------------------+
 * |  5	| ion handle				 |
 * +----+----------------------------------------+
 *
 * magic consists of:
 * - 0xbeef, the actual magic
 * - 0x0a, the length of the command vector
 * - 0x01, a version string
 */

#define VVPU_CMD_ION_ALLOC 1
#define VVPU_CMD_ION_FREE  2

static int rk_ion_handle_command(uint32_t cmd[], int cmd_len)
{
	int ret = 0;
	uint32_t magic;
	uint32_t command;
	ssize_t len;
	struct ion_handle *i_handle;
	ion_phys_addr_t phys_addr;

	magic	= cmd[0];
	command = cmd[1];

	pr_debug("rk_ion magic 0x%08x, cmd %d\n", magic, command);

	switch (command) {
	case VVPU_CMD_ION_ALLOC:
		cmd[2] = 1; /* assume failure; set to OK later */
		cmd[4] = 0;
		cmd[5] = 0;

		len = (ssize_t)cmd[3];

		/* if req len == -1, allocate whole heap */
		if (len == -1) {

			if (rk_ion_heap_capacity != 0) {
				len = (ssize_t) rk_ion_heap_capacity;
				pr_err("rk_ion req len = -1 adjust to %d\n",
					len);
			} else
				pr_err("rk_ion len = -1 not supported\n");
		}

		i_handle = ion_alloc(rk_ion_client, len,
			16, rk_ion_heap_type_mask, 0);

		pr_err("rk_ion ion_alloc(clnt %p, l %ul, 16, DMA, 0) = %p\n",
			rk_ion_client, len, i_handle);

		if (IS_ERR(i_handle))
			pr_err("rk_ion error ion_alloc() == 0x%p\n", i_handle);
		else {
			if (ion_phys(rk_ion_client, i_handle, &phys_addr,
					&len) == 0) {
				cmd[2] = 0; /* alloc and map OK */

				cmd[3] = (uint32_t)len; /* return size */
				cmd[4] = (uint32_t)phys_addr;
				cmd[5] = (uint32_t)i_handle;

				pr_err("rk_ion alloc l %d, 0x%08lx, h 0x%p\n",
					len, phys_addr, i_handle);
			} else {
				pr_err("rk_ion mapping error; handle 0x%p\n",
					i_handle);

				ion_free(rk_ion_client, i_handle);
				i_handle = 0;
			}
		}
		break;

	case VVPU_CMD_ION_FREE:
		i_handle = (struct ion_handle *)cmd[5];

		pr_err("rk_ion free handle 0x%p\n", i_handle);

		ion_free(rk_ion_client, i_handle);
		i_handle = 0;

		cmd[2] = 0; /* ion_free() does not report error */
		break;

	default:
		pr_err("rk_ion invalid command %d from secure vm\n", command);

	}

	return ret;
}

