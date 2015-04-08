/*
 *
 * Copyright (C) 2014 Intel Mobile Communications GmbH
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
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/platform_device_pm.h>
#endif

#include "vcodec_service.h"
#include "vcodec_power_handler.h"

#define VPU_POWER_VBPIPE  "/dev/mvpipe-vpu_power"

#define VPU_POWER_MAX_CMD_LEN 10
#define VPU_POWER_MAGIC 0xcdeb0a01

/*
 * process command received from vbpipe
 * do not return an error, but specify the error in the command
 * data structure/array to be returned to the caller
 *
 * command layout:
 * +----+----------------------------------------+
 * |  0 | magic: 0xcdeb0a01 (versioning??)	 |
 * +----+----------------------------------------+
 * |  1 | command (1 = power off, 2 = power on)	 |
 * +----+----------------------------------------+
 * |  2 | result (0 = OK, error != 0)		 |
 * +----+----------------------------------------+
 *
 * magic consists of:
 * - 0xcdeb, the actual magic
 * - 0x0a, the length of the command vector
 * - 0x01, a version string
 */

#define VVPU_CMD_POWER_OFF 1
#define VVPU_CMD_POWER_ON 2


/*
 * internal data
 */
static struct file *vpu_power_req_vbpipe_filep;

int vpu_power_req_vbpipe_init(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;

	if (vpu_power_req_vbpipe_filep == NULL) {
		/*
		 * open vbpipe to secure vm
		 */
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		pr_debug("vpu_power req pipe init\n");

		fp = filp_open(VPU_POWER_VBPIPE, O_RDWR, 0);

		set_fs(old_fs);

		if (IS_ERR(fp)) {
			pr_debug("vpu_power open vbpipe error %d\n",
				(int)fp);
			vpu_power_req_vbpipe_filep = NULL;
			return -1;
		} else {
			pr_info("vpu_power %p link 0x%p established\n",
				vpu_power_req_vbpipe_filep, fp);

			/*
			 * open OK, leave loop
			 */
			vpu_power_req_vbpipe_filep = fp;
		}
	}

	return 0;
}

int vpu_power_req_vbpipe_close(void)
{
	mm_segment_t old_fs;
	int ret = 0;

	if (vpu_power_req_vbpipe_filep != NULL) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		ret = filp_close(vpu_power_req_vbpipe_filep, NULL);
		vpu_power_req_vbpipe_filep = NULL;

		set_fs(old_fs);
	}

	return ret;
}

static int vpu_power_req_call(unsigned char *buf, int len)
{
	mm_segment_t old_fs;
	unsigned char *datap;
	int must;
	int done;
	int done_total;

	int ret = 0;

	datap	   = buf;
	must	   = len;
	done_total = 0;

	while (must > 0) {
		done = 0;

		old_fs = get_fs();

		set_fs(KERNEL_DS);
		{
			done = vpu_power_req_vbpipe_filep->f_op->
				write(vpu_power_req_vbpipe_filep,
					datap, must, 0);
		}
		set_fs(old_fs);

		if (done < 0) {
			pr_err("error %d writing vbpipe", done);
			break;
		}

		datap	   += done;
		done_total += done;
		must	   -= done;
	}


	datap	   = buf;
	must	   = len;
	done_total = 0;

	while (must > 0) {
		done = 0;

		old_fs = get_fs();

		set_fs(KERNEL_DS);
		{
			done = vpu_power_req_vbpipe_filep->f_op->
				read(vpu_power_req_vbpipe_filep,
					datap, must, 0);
		}
		set_fs(old_fs);

		if (done < 0) {
			pr_err("error %d reading vbpipe", done);
			break;
		}

		datap	   += done;
		done_total += done;
		must	   -= done;
	}

	return ret;
}

int vpu_suspend_req(void)
{
	uint32_t cmd[VPU_POWER_MAX_CMD_LEN];

	cmd[0] = VPU_POWER_MAGIC;
	cmd[1] = VVPU_CMD_POWER_OFF;
	cmd[2] = 0;

	vpu_power_req_call((unsigned char *)cmd, sizeof(cmd));
	return cmd[2];
}

int vpu_resume_req(void)
{
	uint32_t cmd[VPU_POWER_MAX_CMD_LEN];

	cmd[0] = VPU_POWER_MAGIC;
	cmd[1] = VVPU_CMD_POWER_ON;
	cmd[2] = 0;

	vpu_power_req_call((unsigned char *)cmd, sizeof(cmd));
	return cmd[2];
}

