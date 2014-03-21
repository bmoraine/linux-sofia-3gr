/*
 * Copyright (C) 2013, 2014 Intel Mobile Communications GmbH
 *
 * Notes:
 * Aug 22 2014: IMC: vbpipe interface to secure VM
 */

/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#include <linux/fs.h>
#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/ktime.h>

#include "vvpu_vbpipe.h"


/*
 * uncomment to trace each vvpu rpc call to ther kernel console
 * Warning: this causes load!!
 */
/** #define __VERBOSE_RPC__ **/

/* vbpipe designated for vvpu usage */
#define VVPU_VBPIPE		"/dev/vbpipe5"


/*
 * make sure only one driver is using the vvpu module at a time
 * this includes:
 * - init/deinit
 * - sending commands
 */
DEFINE_MUTEX(vvpu_mutex);

/*
 * everything we need to know
 */
static struct file *vvpu_vbpipe_filep;


#ifdef __VERBOSE_RPC__

static const char * const vtnames[] = {
		"h264dec",
		"mpeg4dec",
		"vp8dec",
		"pp",
		"h264enc",
		"mpeg4enc",
};

static const char * const vonames[] = {
		"ping",
		"get_api_version", /*  1 */
		"get_build",
		"init",
		"deinit",
		"decode", /*  5 */
		"next_frame",
		"get_info",
		"peek",
		"set_mvc",
		"set_info", /* 10 */
		"combined_mode_enable",
		"combined_mode_disable",
		"get_config",
		"set_config",
		"set_multiple_output", /* 15 */
		"get_next_output",
		"get_result",
		"get_user_data",
		"set_picture_buffers",
		"set_coding_ctrl", /* 20 */
		"get_coding_ctrl",
		"set_rate_ctrl",
		"get_rate_ctrl",
		"set_preprocessing",
		"get_preprocessing", /* 25 */
		"set_sei_userdata",
		"enc_strm_start",
		"enc_strm_encode",
		"enc_strm_end",	 /* 29 */

};

#endif



/*
 * initialize vvpu connection to the secure vm
 * internal function: extra param locked
 */
static int vvpu_vbpipe_init_int(struct device *dev, int locked)
{
	mm_segment_t old_fs;
	struct file *fp = NULL;

	int ret = 0;

	/* don't lock if already locked */
	if (locked == 0)
		if (mutex_lock_interruptible(&vvpu_mutex))
			return -1;

	if (vvpu_vbpipe_filep == NULL) {
		/*
		 * open vbpipe to secure vm
		 */
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		fp = filp_open(VVPU_VBPIPE, O_RDWR, 0);

		set_fs(old_fs);

		if (IS_ERR(fp)) {
			ret = (int)fp;

			dev_err(dev, "open vbpipe: error %d", ret);
			vvpu_vbpipe_filep = NULL;
		} else {
			dev_info(dev, "vbpipe %s link 0x%p established",
				VVPU_VBPIPE, fp);

			vvpu_vbpipe_filep = fp;

			ret = 0;
		}
	} else
		dev_info(dev, "init: vvpu already initialized");

	/* don't lock if it was locked before */
	if (locked == 0)
		mutex_unlock(&vvpu_mutex);

	return ret;
}

/*
 * initialize vvpu connection to the secure vm
 */
int vvpu_vbpipe_init(struct device *dev)
{
	/* from external, not yet locked */
	return vvpu_vbpipe_init_int(dev, 0);
}


/*
 * release vvpu connection to the secure vm
 */
int vvpu_vbpipe_release(struct device *dev)
{
	mm_segment_t old_fs;

	int ret = 0;

	if (mutex_lock_interruptible(&vvpu_mutex))
		return -1;

	if (vvpu_vbpipe_filep != NULL) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		ret = filp_close(vvpu_vbpipe_filep, NULL);
		vvpu_vbpipe_filep = NULL;

		set_fs(old_fs);

		dev_info(dev, "vbpipe closed (%d)", ret);
	} else
		dev_info(dev, "release: vvpu was not in use");

	mutex_unlock(&vvpu_mutex);

	return ret;
}


/*
 * send a payload to secure vm and wait for response
 */
static int vvpu_vbpipe_call(struct device *dev,
	unsigned char *buf, int len)
{
	mm_segment_t old_fs;
	unsigned char *datap;
	int must;
	int done;
	int done_total;

	int ret = 0;

	/* Acquire mutex; on failure return immediately */
	if (mutex_lock_interruptible(&vvpu_mutex))
		return -1;

	/*
	 * if vbpipe is not yet initialized, give it another chance
	 */
	if (vvpu_vbpipe_filep == NULL) {
		/* mutex already locked */
		ret = vvpu_vbpipe_init_int(dev, 1);

		if (ret != 0) {
			dev_err(dev, "init vbpipe error %d", ret);

			goto end;
		}
	}

	/*
	 * try to send command to secvm
	 */
	if (vvpu_vbpipe_filep != NULL) {

		datap	   = buf;
		must	   = len;
		done_total = 0;

		while (must > 0) {
			done = 0;

			old_fs = get_fs();

			set_fs(KERNEL_DS);
			{
				done = vvpu_vbpipe_filep->f_op->
					write(vvpu_vbpipe_filep,
						datap, must, 0);
			}
			set_fs(old_fs);

			if (done < 0) {
				dev_err(dev, "error %d writing vbpipe", done);
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
				done = vvpu_vbpipe_filep->f_op->
					read(vvpu_vbpipe_filep,
						datap, must, 0);
			}
			set_fs(old_fs);

			if (done < 0) {
				dev_err(dev, "error %d reading vbpipe", done);
				break;
			}

			datap	   += done;
			done_total += done;
			must	   -= done;
		}

	} else
		dev_err(dev, "no vbpipe link; command not sent");

end:
	mutex_unlock(&vvpu_mutex);

	return ret;
}



/*
 * Make an rpc call to secure vm via vbpipe interface.
 *
 * return the size of the transferred data.
 */
int vvpu_call(struct device *dev, struct vvpu_secvm_cmd *cmd_p)
{
#ifdef __VERBOSE_RPC__
	unsigned long long tic;
	struct timespec ts;
#endif
	unsigned int *arg = &(cmd_p->payload[0]);
	int size = sizeof(struct vvpu_secvm_cmd);

	int ret = 0;

#ifdef __VERBOSE_RPC__
	if (arg[0] < ARRAY_SIZE(vtnames) && arg[1] < ARRAY_SIZE(vonames)) {
		dev_info(dev, "vvpu cmd: %s-%s h=%x\n", vtnames[arg[0]],
			vonames[arg[1]], arg[2]);
	} else {
		dev_info(dev, "vvpu cmd type=%d op=%d h=0x%08x\n",
			arg[0], arg[1], arg[2]);
	}

	ktime_get_ts(&ts);
	tic = timespec_to_ns(&ts);
#endif

	vvpu_vbpipe_call(dev, (unsigned char *)arg, size);
	ret = size;

#ifdef __VERBOSE_RPC__
	ktime_get_ts(&ts);
	tic = timespec_to_ns(&ts) - tic;

	dev_info(dev, "vvpu cmd done in %u ns r=0x%08x h=0x%08x.\n",
		(unsigned int) tic, arg[3], arg[2]);
#endif

	return ret;
}
