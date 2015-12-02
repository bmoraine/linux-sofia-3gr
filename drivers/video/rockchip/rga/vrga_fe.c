/*
 * Copyright (C) 2013, 2014 Intel Mobile Communications GmbH
 *
 * Notes:
 * Aug 22 2014: IMC: fe interface to secure VM
 * Oct 30 2014: IMC: += vrga_ping()
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
 */

#include <linux/fs.h>
#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/ktime.h>

#include <sofia/vrga_fe.h>


/*
 * uncomment to trace each vrga rpc call to ther kernel console
 * Warning: this causes load!!
 */
/** #define __VERBOSE_RPC__ **/

/* fe designated for vrga usage */
#define VRGA_FE "/dev/mvpipe-rga"

/*
 * make sure only one driver is using the vrga module at a time
 * this includes:
 * - init/deinit
 * - sending commands
 */
DEFINE_MUTEX(vrga_mutex);

/*
 * everything we need to know
 */
static struct file *vrga_fe_filep;


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
		"ping",			     /*	 0: generic, testing purposes */
		"get_api_version",	     /*	 1: */
		"get_build",		     /*	 2: */
		"init",			     /*	 3: */
		"deinit",		     /*	 4: */
		"decode",		     /*	 5: */
		"next_frame",		     /*	 6: */
		"get_info",		     /*	 7: */
		"peek",			     /*	 8: */
		"set_mvc",		     /*	 9: h264 only */
		"set_info",		     /* 10: mpeg4 only */
		"combined_mode_enable",	     /* 11: pp only */
		"combined_mode_disable",     /* 12: pp only */
		"get_config",		     /* 13: pp only */
		"set_config",		     /* 14: pp only */
		"set_multiple_output",	     /* 15: pp only */
		"get_next_output",	     /* 16: pp only */
		"get_result",		     /* 17: pp only */
		"get_user_data",	     /* 18: mpeg4 only */
		"set_picture_buffers",	     /* 19: vp8 only */
		"set_coding_ctrl",	     /* 20: enc only */
		"get_coding_ctrl",	     /* 21: enc only */
		"set_rate_ctrl",	     /* 22: enc only */
		"get_rate_ctrl",	     /* 23: enc only */
		"set_preprocessing",	     /* 24: enc only */
		"get_preprocessing",	     /* 25: enc only */
		"set_sei_userdata",	     /* 26: enc only */
		"enc_strm_start",	     /* 27: enc only */
		"enc_strm_encode",	     /* 28: enc only */
		"enc_strm_end",		     /* 29: enc only */
		"enc_op_set_stream_info",    /* 30: enc only */
		"enc_op_get_stream_info",    /* 31: enc only */
		"enc_op_set_hw_burst_size",  /* 32: enc only */
		"enc_op_set_hw_burst_type",  /* 33: enc only */
		"enc_op_set_chr_qp_offset",  /* 34: enc only */
		"enc_op_set_filter",	     /* 35: enc only */
		"enc_op_get_filter",	     /* 36: enc only */
		"mem_op_alloc",		     /* 37: mem only */
		"mem_op_free",		     /* 38: mem only */

};

#endif



/*
 * initialize vrga connection to the secure vm
 * internal function: extra param locked
 */
static int vrga_fe_init_int(struct device *dev, int locked)
{
	mm_segment_t old_fs;
	struct file *fp = NULL;

	int ret = 0;

	/* don't lock if already locked */
	if (locked == 0)
		if (mutex_lock_interruptible(&vrga_mutex))
			return -1;

	if (vrga_fe_filep == NULL) {
		/*
		 * open fe to secure vm
		 */
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		fp = filp_open(VRGA_FE, O_RDWR, 0);

		set_fs(old_fs);

		if (IS_ERR(fp)) {
			ret = (int)fp;

			/* only log error if called from internal this module */
			if (locked != 0)
				dev_err(dev, "open fe: error %d", ret);

			vrga_fe_filep = NULL;
		} else {
			dev_dbg(dev, "fe %s link 0x%p established",
				VRGA_FE, fp);

			vrga_fe_filep = fp;
			ret = 0;
		}
	} else
		dev_info(dev, "init: vrga already initialized");

	/* don't lock if it was locked before */
	if (locked == 0)
		mutex_unlock(&vrga_mutex);

	return ret;
}

/*
 * initialize vrga connection to the secure vm
 */
int vrga_fe_init(struct device *dev)
{
	vrga_fe_filep = NULL;

	/* from external, not yet locked */
	return vrga_fe_init_int(dev, 0);
}


/*
 * release vrga connection to the secure vm
 */
int vrga_fe_release(struct device *dev)
{
	mm_segment_t old_fs;

	int ret = 0;

	if (mutex_lock_interruptible(&vrga_mutex))
		return -1;

	if (vrga_fe_filep != NULL) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		ret = filp_close(vrga_fe_filep, NULL);
		vrga_fe_filep = NULL;

		set_fs(old_fs);

		dev_info(dev, "fe closed (%d)", ret);
	} else
		dev_info(dev, "release: vrga was not in use");

	mutex_unlock(&vrga_mutex);

	return ret;
}


/*
 * send a payload to secure vm and wait for response
 */
static int vrga_fe_call(struct device *dev,
	unsigned char *buf, int len)
{
	mm_segment_t old_fs;
	unsigned char *datap;
	int must;
	int done;
	int done_total;

	int ret = 0;

	/*
	 * Adding variable loops_count to prevent dead cycle
	 * when vbpipe always can't be read
	 */
	int loops_count;

	/* Acquire mutex; on failure return immediately */
	if (mutex_lock_interruptible(&vrga_mutex))
		return -1;

	/*
	 * if fe is not yet initialized, give it another chance
	 */
	if (vrga_fe_filep == NULL) {
		/* mutex already locked */
		ret = vrga_fe_init_int(dev, 1);

		if (ret != 0) {
			dev_err(dev, "init fe error %d", ret);

			goto end;
		}
	}

	/*
	 * try to send command to secvm
	 */
	if (vrga_fe_filep != NULL) {

		datap	   = buf;
		must	   = len;
		done_total = 0;
		loops_count= 0;

		while (must > 0) {
			done = 0;

			old_fs = get_fs();

			set_fs(KERNEL_DS);
			{
				done = vrga_fe_filep->f_op->
					write(vrga_fe_filep,
						datap, must, 0);
			}
			set_fs(old_fs);

			if (done < 0) {
				dev_err(dev, "error %d writing fe", done);
				if ((done == -ERESTARTSYS) && (loops_count++ < 2))
					continue;

				ret = -EBUSY;
				break;
			} else {
				loops_count = 0;
			}

			datap	   += done;
			done_total += done;
			must	   -= done;
		}


		datap	   = buf;
		must	   = len;
		done_total = 0;
		loops_count = 0;

		while (must > 0) {
			done = 0;

			old_fs = get_fs();

			set_fs(KERNEL_DS);
			{
				done = vrga_fe_filep->f_op->
					read(vrga_fe_filep,
						datap, must, 0);
			}
			set_fs(old_fs);

			if (done < 0) {
				dev_err(dev, "error %d reading fe", done);
				if ((done == -ERESTARTSYS) && (loops_count++ < 2))
					continue;

				ret = -EBUSY;
				break;
			} else {
				loops_count = 0;
			}

			datap	   += done;
			done_total += done;
			must	   -= done;
		}

	} else
		dev_err(dev, "no fe link; command not sent");

end:
	mutex_unlock(&vrga_mutex);

	return ret;
}



/*
 * Make an rpc call to secure vm via fe interface.
 *
 * return the size of the transferred data.
 */
int vrga_call(struct device *dev, struct vrga_secvm_cmd *cmd_p)
{
#ifdef __VERBOSE_RPC__
	unsigned long long tic;
	struct timespec ts;
#endif
	unsigned int *arg = &(cmd_p->payload[0]);
	int size = sizeof(struct vrga_secvm_cmd);

	int ret = 0;

#ifdef __VERBOSE_RPC__
	if (arg[0] < ARRAY_SIZE(vtnames) && arg[1] < ARRAY_SIZE(vonames)) {
		dev_info(dev, "vrga cmd: %s-%s h=0x%08x %08x %08x %08x %08x\n",
			vtnames[arg[0]], vonames[arg[1]],
			arg[2], arg[3], arg[4], arg[5], arg[6]);
	} else {
		dev_info(dev,
			"vrga cmd type=%d op=%d h=0x%08x %08x %08x %08x %08x\n",
			arg[0], arg[1], arg[2], arg[3], arg[4], arg[5], arg[6]);
	}

	ktime_get_ts(&ts);
	tic = timespec_to_ns(&ts);
#endif

	ret = vrga_fe_call(dev, (unsigned char *)arg, size);
	/* return the size of transferred data */
	ret = (ret<0) ? 0 : size;

#ifdef __VERBOSE_RPC__
	ktime_get_ts(&ts);
	tic = timespec_to_ns(&ts) - tic;

	dev_info(dev, "vrga cmd done in %u ns r=0x%08x h=0x%08x.\n",
		(unsigned int) tic, arg[3], arg[2]);
#endif

	return ret;
}


/*
 * send a ping request to the secure vm which immediately returns
 * (takes the turn around time on the kernel log)
 */
int vrga_ping(struct device *dev, uint32_t cmd)
{
	unsigned long long tic;
	struct timespec ts;

	struct vrga_secvm_cmd secvm_cmd;
	uint32_t *arg = &(secvm_cmd.payload[0]);
	int size = sizeof(struct vrga_secvm_cmd);

	int ret = 0;

	secvm_cmd.payload[0] = VRGA_VTYPE_INIT_HANDSHAKE;
	secvm_cmd.payload[1] = cmd;
	secvm_cmd.payload[2] = 0xfefefefe;
	secvm_cmd.payload[3] = 0xefefefef;

	ktime_get_ts(&ts);
	tic = timespec_to_ns(&ts);

	dev_info(dev, "vrga_ping [0x%08x 0x%08x 0x%08x 0x%08x] len %d\n",
		arg[0], arg[1], arg[2], arg[3], size);

	ret = vrga_fe_call(dev, (unsigned char *)arg, size);

	if (ret == 0) {
		ktime_get_ts(&ts);
		tic = timespec_to_ns(&ts) - tic;

		dev_info(dev,
			"vrga_ping [0x%08x 0x%08x 0x%08x 0x%08x] %u ns\n",
			arg[0], arg[1], arg[2], arg[3], (unsigned int) tic);
	} else
		dev_warn(dev, "vrga_ping error\n");

	return ret;
}
