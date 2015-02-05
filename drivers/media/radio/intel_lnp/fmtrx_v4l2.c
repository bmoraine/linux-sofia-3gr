/*
 * Copyright (C) 2013 Intel Mobile Communications GmbH
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

/*
** =============================================================================
**
**				MODULE DESCRIPTION
**
** =============================================================================
*/
/* This file contains the FMR_V4L2 layer which is common to all platforms. */

/*
** =============================================================================
**
**				INCLUDE STATEMENTS
**
** =============================================================================
*/
#include "fmtrx_sys.h"		/* System related */
#include "fmtrx_common.h"
#include "fmtrx_v4l2.h"

/*
** =============================================================================
**
**				DEFINES
**
** =============================================================================
*/
#define FMDRV_V4L2_VERSION KERNEL_VERSION(1, 0, 0)

/*
** =============================================================================
**
**				STRUCT DECLARATIONS
**
** =============================================================================
*/

/*
** =============================================================================
**
**				LOCAL DATA DEFINITIONS
**
** =============================================================================
*/
static struct video_device video_dev;
static struct mutex interface_lock;
static struct fmrx_config *fmrx_cfg;
static u8 module_reference_count;
static const u8 v4l2_freq_cap_flag = V4L2_TUNER_CAP_LOW;

/*
** =============================================================================
**
**				LOCAL FUNCTION DECLARATIONS
**
** =============================================================================
*/
/* V4L2 file open method
 * @fp Pointer to the file descriptor
 */
static int fmtrx_v4l2_fops_open(
		struct file *fp);

/* V4L2 file poll method
 * @fp Pointer to the file descriptor
 * @pts Pointer to the poll descriptor
 */
static unsigned int fmtrx_v4l2_fops_poll(
		struct file *fp,
		struct poll_table_struct *pts);

/* V4L2 file read method
 * @fp Pointer to the file descriptor
 * @data Pointer to the memory where data will be stored
 * @count Amount of data to fetch
 * @ppos Offset in the data
 */
static ssize_t fmtrx_v4l2_fops_read(
		struct file *fp,
		char __user *data,
		size_t count,
		loff_t *ppos);

/* V4L2 file close method
 * @fp Pointer to the file descriptor
 */
static int fmtrx_v4l2_fops_release(
		struct file *fp);

/* V4L2 ioctl query capability method
 * @fp Pointer to the file descriptor
 * @priv Pointer to private data
 * @data Pointer to the fetched capability
 */
static int fmtrx_v4l2_iops_querycap(
		struct file *fp,
		void *priv,
		struct v4l2_capability *data);

/* V4L2 ioctl get tunner attributes
 * @fp Pointer to the file descriptor
 * @priv Pointer to private data
 * @data Pointer to tuner attributes
 */
static int fmtrx_v4l2_iops_gtuner(
		struct file *fp,
		void *priv,
		struct v4l2_tuner *data);

/* V4L2 ioctl set tunner attributes
 * @fp Pointer to the file descriptor
 * @priv Pointer to private data
 * @data Pointer to tuner attributes
 */
static int fmtrx_v4l2_iops_stuner(
		struct file *fp,
		void *priv,
		const struct v4l2_tuner *data);

/* V4L2 ioctl set tuner frequency
 * @fp Pointer to the file descriptor
 * @priv Pointer to private data
 * @data Pointer to frequency structure
 */
static int fmtrx_v4l2_iops_sfrequency(
		struct file *fp,
		void *priv,
		const struct v4l2_frequency *data);

/* V4L2 ioctl private handler
 * @fp Pointer to the file descriptor
 * @priv Pointer to private data
 * @valid_prio Priority
 * @cmd Command
 * @arg Input argument
 */
static long fmtrx_v4l2_iops_priv(
		struct file *fp,
		void *priv,
		bool valid_prio,
		unsigned int cmd,
		void *data);

/* V4L2 ioctl get tuner frequency
 * @fp Pointer to the file descriptor
 * @priv Pointer to private data
 * @data Pointer to frequency structure
 */
static int fmtrx_v4l2_iops_gfrequency(
		struct file *fp,
		void *priv,
		struct v4l2_frequency *data);

/* V4L2 ioctl query any control value
 * @fp Pointer to the file descriptor
 * @priv Pointer to private data
 * @data Pointer to control structure
 */
static int fmtrx_v4l2_iops_queryctrl(
		struct file *fp,
		void *priv,
		struct v4l2_queryctrl *data);

/* V4L2 ioctl get the current control value
 * @fp Pointer to the file descriptor
 * @priv Pointer to private data
 * @data Pointer to control structure
 */
static int fmtrx_v4l2_iops_gctrl(
		struct file *fp,
		void *priv,
		struct v4l2_control *data);

/* V4L2 ioctl set a control value
 * @fp Pointer to the file descriptor
 * @priv Pointer to private data
 * @data Pointer to control structure
 */
static int fmtrx_v4l2_iops_sctrl(
		struct file *fp,
		void *priv,
		struct v4l2_control *data);

/* V4L2 ioctl seek stations
 * @fp Pointer to the file descriptor
 * @priv Pointer to private data
 * @data Pointer to control structure
 */
static int fmtrx_v4l2_iops_hw_freq_seek(
		struct file *fp,
		void *priv,
		const struct v4l2_hw_freq_seek *data);


#ifdef CONFIG_COMPAT
/* V4L2 32-bit compatibility layer */
static long fmtrx_v4l2_fops_compat_ioctl32(
		struct file *file,
		unsigned int cmd,
		unsigned long arg);
#endif

/*
** =============================================================================
**
**				LOCAL FUNCTION DEFINITIONS
**
** =============================================================================
*/
static const struct v4l2_file_operations fmtrx_v4l2_fops = {
	.owner = THIS_MODULE,
	.open = fmtrx_v4l2_fops_open,
	.read = fmtrx_v4l2_fops_read,
	.poll = fmtrx_v4l2_fops_poll,
	.release = fmtrx_v4l2_fops_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = fmtrx_v4l2_fops_compat_ioctl32,
#endif
	.unlocked_ioctl = video_ioctl2
};

static const struct v4l2_ioctl_ops fmtrx_v4l2_iops = {
	.vidioc_querycap = fmtrx_v4l2_iops_querycap,
	.vidioc_g_tuner = fmtrx_v4l2_iops_gtuner,
	.vidioc_s_tuner = fmtrx_v4l2_iops_stuner,
	.vidioc_g_frequency = fmtrx_v4l2_iops_gfrequency,
	.vidioc_s_frequency = fmtrx_v4l2_iops_sfrequency,
	.vidioc_queryctrl = fmtrx_v4l2_iops_queryctrl,
	.vidioc_g_ctrl = fmtrx_v4l2_iops_gctrl,
	.vidioc_s_ctrl = fmtrx_v4l2_iops_sctrl,
	.vidioc_s_hw_freq_seek = fmtrx_v4l2_iops_hw_freq_seek,
	.vidioc_default = fmtrx_v4l2_iops_priv
};

int fmtrx_v4l2_init(
		struct device *dev,
		enum fmtrx_type type)
{
	int err = 0;
	struct v4l2_device *v4l2_dev = 0;

	/* Validate input arguments */
	if (0 == dev) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_init_exit;
	}

	/* Data structures */
	mutex_init(&interface_lock);

	v4l2_dev = kzalloc(sizeof(struct v4l2_device), GFP_KERNEL);
	if (0 == v4l2_dev) {
		err = -ENOMEM;
		fmtrx_sys_log
			("%s: %s %d,V4L2 device allocn failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_init_exit;
	}

	/* Register with FMR_V4L2 subsystem */
	err = v4l2_device_register(dev, v4l2_dev);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,V4L2 device reg failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_init_exit1;
	}

	video_dev.v4l2_dev = v4l2_dev;
	video_dev.fops = &fmtrx_v4l2_fops;
	video_dev.ioctl_ops = &fmtrx_v4l2_iops;
	video_dev.release = video_device_release_empty;
	video_dev.debug = 0;

	/* Register video device */
	err = video_register_device(&video_dev,
			VFL_TYPE_RADIO, -1);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,	V4L2 video device reg failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_init_exit2;
	}

	goto fmtrx_v4l2_init_exit;

fmtrx_v4l2_init_exit2:
	v4l2_device_unregister(v4l2_dev);
fmtrx_v4l2_init_exit1:
	if (0 != v4l2_dev)
		kfree(v4l2_dev);
fmtrx_v4l2_init_exit:
	return err;
}

int fmtrx_v4l2_deinit(
		struct device *dev,
		enum fmtrx_type type)
{
	int err = 0;

	/* Validate input arguments */
	if (0 == dev) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_deinit_exit;
	}

	/* Unregister from V4L2 subsystem */
	if (0 != video_dev.v4l2_dev) {
		/* Unregister video device */
		video_unregister_device(&video_dev);

		/* Unregister v4l2 device */
		v4l2_device_unregister(video_dev.v4l2_dev);

		kfree(video_dev.v4l2_dev);
	}

fmtrx_v4l2_deinit_exit:
	return err;
}

static int fmtrx_v4l2_fops_open(
		struct file *fp)
{
	int err = 0;

	mutex_lock(&interface_lock);
	/* Initialize FM core module only once
			for a open call */
	if (0 == module_reference_count) {
		/* Initialize the FM core module */
		err = fmtrx_init(FMTRX_INIT_MODE_ON, FMTRX_RX);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FMTRX core init failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_open_exit;
		}

		err = fmtrx_sys_get_rx_default_config(&fmrx_cfg);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Get FM default config failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_open_exit1;
		}

		err = fmrx_set_config(fmrx_cfg);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Set FM default config failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_open_exit1;
		}

		err = fmrx_power_on();
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FM RX power on failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_open_exit1;
		}
	}
	/* Keep track of driver open reference counts */
	module_reference_count++;
	goto fmtrx_v4l2_fops_open_exit;

fmtrx_v4l2_fops_open_exit1: {
		int err1 = fmtrx_init(
				FMTRX_INIT_MODE_OFF, FMTRX_RX);
		if (0 != err1) {
			fmtrx_sys_log
				("%s: %s %d,FMTRX core de-init failed! %d\n",
				FILE, __func__,
				__LINE__, err1);
		}
	}
	kfree(fmrx_cfg);
fmtrx_v4l2_fops_open_exit:
	mutex_unlock(&interface_lock);
	return err;
}

static unsigned int fmtrx_v4l2_fops_poll(
		struct file *fp,
		struct poll_table_struct *pts)
{
	int err = 0;

	return err;
}

static ssize_t fmtrx_v4l2_fops_read(
		struct file *fp,
		char __user *data,
		size_t count,
		loff_t *ppos)
{
	int err = 0;
	u16 requested_groups = 0, tries = 3,
			total_read_bytes = 0;
	struct rds_group *temp_data = 0;
	struct fmrx_config *temp_cfg;
	enum fmrx_state state = FMRX_HW_STATE_INVALID;
	struct rds_group_pkt rds_pkt = { 0, 0, 0 };

	mutex_lock(&interface_lock);
	/* Validate the number of bytes to read - should be
			greater than the size of RDS groups */
	if (sizeof(struct rds_group) > count)
		goto fmtrx_v4l2_fops_read_exit;

	err = fmrx_get_hw_state(&state);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Get hw state failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_fops_read_exit;
	}

	if (FMRX_HW_STATE_RX_ACTIVE != state) {
		err = -EBADFD;
		fmtrx_sys_log
		("%s: %s %d,FMR in wrong state! Current state: %d, %d\n",
			FILE, __func__,
			__LINE__, state, err);
		goto fmtrx_v4l2_fops_read_exit;
	}

	err = fmrx_get_config(&temp_cfg);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Get static config failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_fops_read_exit;
	}

	if (RDS_ONMODE_OFF == temp_cfg->rds_cfg.mode) {
		struct rds rds_cfg = { RDS_ONMODE_ON };

		err = fmrx_set_rds(&rds_cfg);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Set RDS config failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_read_exit;
		}
	}

	requested_groups = count / sizeof(struct rds_group);
	temp_data = kzalloc((sizeof(struct rds_group) *
				requested_groups), GFP_KERNEL);
	if (0 == temp_data) {
		err = -ENOMEM;
		goto fmtrx_v4l2_fops_read_exit;
	}

	rds_pkt.data = temp_data;
	rds_pkt.requested_groups = requested_groups;
	do {
		err = fmrx_get_rds_groups(&rds_pkt);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Get RDS groups failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_read_exit;
		}

		/* Return if there is no data available and the device is
				opened in non-blocking mode */
		if ((0 == rds_pkt.received_groups) &&
					(fp->f_flags & O_NONBLOCK)) {
			err = -ENODATA;
			goto fmtrx_v4l2_fops_read_exit;
		} else if (0 != rds_pkt.received_groups) {
			break;
		}

		if (0 == tries) {
			err = -ENODATA;
			break;
		}

		mdelay(FMTRX_RDS_TIMEOUTS);
		tries--;
	} while (0 == rds_pkt.received_groups);

	/* Copy to the user memory */
	if (0 != rds_pkt.received_groups) {
		total_read_bytes =
			rds_pkt.received_groups * sizeof(struct rds_group);

		err = copy_to_user(data, temp_data, total_read_bytes);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Copy to user failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			err = -EAGAIN;
		}
	}
	err = total_read_bytes;

fmtrx_v4l2_fops_read_exit:
	kfree(temp_data);
	mutex_unlock(&interface_lock);
	return err;
}

static int fmtrx_v4l2_fops_release(
		struct file *fp)
{
	int err = 0;
	struct fmrx_config *data = 0;

	mutex_lock(&interface_lock);
	/* Cleanup only when there is one Application
		holding the driver */
	if (module_reference_count == 1) {
		err = fmrx_power_off();
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FM RX power off failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		}

		err = fmrx_get_config(&data);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FM Get configuration failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		}
		kfree(data);

		/* De-initialize the FM code module */
		err = fmtrx_init(
				FMTRX_INIT_MODE_OFF, FMTRX_RX);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FMTRX core de-init failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		}
	}
	/* Keep track of driver open reference counts */
	module_reference_count--;
	mutex_unlock(&interface_lock);
	return err;
}

#ifdef CONFIG_COMPAT
static long fmtrx_v4l2_fops_compat_ioctl32(
		struct file *fp,
		unsigned int cmd,
		unsigned long arg)
{
	int err = 0;
	void __user *user = compat_ptr(arg);

#ifdef FMR_DEBUG_LVL2
	fmtrx_sys_log
	("%s: %s %d,32-bit compatibility layer invoked!\n",
		FILE, __func__,
		__LINE__);
#endif

	/* Validate input arguments */
	if ((0 == user) && (V4L2_CID_PRIV_INTEL_FMRX_NVM_COMMIT != cmd)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_fops_compat_ioctl32_exit;
	}

	/* Handle IOCTLs that need 32-bit to 64-bit conversion */
	switch (cmd) {
	case V4L2_CID_PRIV_INTEL_FMRX_AF_SWITCH: {
		struct af_info_32 __user *_data32 =
					(struct af_info_32 *)user;
		struct af_info _data64;
		compat_caddr_t p_freq_list;

		memset(&_data64, 0, sizeof(struct af_info));

		/* Copy from user space structure */
		err = get_user(_data64.count, &_data32->count);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Get user data failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_compat_ioctl32_exit;
		}

		err = get_user(_data64.pi_code, &_data32->pi_code);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Get user data failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_compat_ioctl32_exit;
		}

		err = get_user(p_freq_list, &_data32->freq_list);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Get user data failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_compat_ioctl32_exit;
		}
		_data64.freq_list = compat_ptr(p_freq_list);

		err = fmrx_af_switch(&_data64);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Switch to AF failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMTRX_READ:
	/* Intended fall through */
	case V4L2_CID_PRIV_INTEL_FMTRX_WRITE: {
		struct reg_info_32 __user *_data32 =
					(struct reg_info_32 *)user;
		struct reg_info _data64;
		compat_caddr_t p_data;
		u32 value = 0;

		memset(&_data64, 0, sizeof(struct reg_info));

		/* Copy from user space structure */
		err = get_user(_data64.addr_offs,
				&_data32->addr_offs);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Get user data failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_compat_ioctl32_exit;
		}

		err = get_user(p_data, &_data32->data);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Get user data failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_compat_ioctl32_exit;
		}
		_data64.data = compat_ptr(p_data);

		if (V4L2_CID_PRIV_INTEL_FMTRX_READ == cmd) {
			err = fmtrx_sys_reg_read32(_data64.addr_offs,
						&value);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,Read 32bit failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_fops_compat_ioctl32_exit;
			}

			err = copy_to_user(_data64.data, &value,
						sizeof(value));
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,Copy to user failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_fops_compat_ioctl32_exit;
			}
		} else {
			err = copy_from_user(&value, _data64.data,
						sizeof(value));
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,Copy from user failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_fops_compat_ioctl32_exit;
			}

			err = fmtrx_sys_reg_write32(_data64.addr_offs,
						value);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,Read 32bit failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_fops_compat_ioctl32_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_RDS_DATA: {
		struct rds_group_pkt_32 __user *_data32  =
					(struct rds_group_pkt_32 *)user;
		struct rds_group_pkt _data64;
		compat_caddr_t p_rds_groups;

		memset(&_data64, 0, sizeof(struct rds_group_pkt));

		/* Copy from user space structure */
		err = get_user(_data64.requested_groups,
				&_data32->requested_groups);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Get user data failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_compat_ioctl32_exit;
		}

		err = get_user(p_rds_groups, &_data32->data);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Get user data failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_compat_ioctl32_exit;
		}
		_data64.data = compat_ptr(p_rds_groups);

		err = fmrx_get_rds_groups(&_data64);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Get RDS groups failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_fops_compat_ioctl32_exit;
		}
		_data32->received_groups = _data64.received_groups;
		break;
	}
	default: {
		err = fmtrx_v4l2_iops_priv(fp, NULL,
						true, cmd, user);
		if (0 != err) {
			fmtrx_sys_log
			("%s: %s %d,Private IOCTL execution failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		}
		break;
	}
	}

fmtrx_v4l2_fops_compat_ioctl32_exit:
	return err;
}
#endif

static int fmtrx_v4l2_iops_querycap(
		struct file *fp,
		void *priv,
		struct v4l2_capability *data)
{
	int err = 0;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_iops_querycap_exit;
	}

	err = fmtrx_sys_get_driver_ver_info(
				data->driver, data->card);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,FM get driver version info failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_iops_querycap_exit;
	}

	data->version = FMDRV_V4L2_VERSION;
	data->capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO |
				V4L2_CAP_AUDIO | V4L2_CAP_RDS_CAPTURE |
				V4L2_CAP_HW_FREQ_SEEK;

fmtrx_v4l2_iops_querycap_exit:
	return err;
}

static int fmtrx_v4l2_iops_gtuner(
		struct file *fp,
		void *priv,
		struct v4l2_tuner *data)
{
	int err = 0;
	struct channel_info ch_info;
	struct fmrx_config *config = 0;
	struct band band_cfg = { 0, 0, 0, 0 };

	memset((u8 *)&ch_info, 0, sizeof(struct channel_info));

	/* Validate input arguments */
	if ((0 == data) || (0 != data->index)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_iops_gtuner_exit;
	}

	/* Fill configurations that are fixed */
	data->type = V4L2_TUNER_RADIO;
	data->capability = V4L2_TUNER_CAP_RDS |
					V4L2_TUNER_CAP_STEREO |
					v4l2_freq_cap_flag;

	data->capability = data->capability |
			V4L2_TUNER_CAP_HWSEEK_WRAP |
			V4L2_TUNER_CAP_HWSEEK_BOUNDED;

	/* Get driver version to fill tuner name */
	err = fmtrx_sys_get_driver_ver_info(data->name, 0);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,FM get driver version info failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_iops_gtuner_exit;
	}

	/* Get Band information */
	err = fmrx_get_band(&band_cfg);
	if (0 != err) {
		err = -EIO;
		fmtrx_sys_log
			("%s: %s %d,Fetch band information failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_iops_gtuner_exit;
	}
	/* Convert frequency units based on cap flag */
	if (V4L2_TUNER_CAP_LOW == v4l2_freq_cap_flag) {
		data->rangelow = HZ_TO_62_5_HZ(KHZTOHZ(band_cfg.min));
		data->rangehigh = HZ_TO_62_5_HZ(KHZTOHZ(band_cfg.max));
	} else {
		data->rangelow = KHZ_TO_62_5_KHZ(band_cfg.min);
		data->rangehigh = KHZ_TO_62_5_KHZ(band_cfg.max);
	}
	/* Get current channel information */
	err = fmrx_get_channel_info(&ch_info);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,FM get channel info failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_iops_gtuner_exit;
	}
	data->rxsubchans = (ch_info.is_stereo ?
				V4L2_TUNER_SUB_STEREO : V4L2_TUNER_SUB_MONO) |
				((true == ch_info.rds_enable) ?
				V4L2_TUNER_SUB_RDS : 0);
	data->signal = ch_info.rssi;

	/* Get static configuration */
	err = fmrx_get_config(&config);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,FM get static config failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_iops_gtuner_exit;
	}
	data->audmode = (config->force_mono) ?
			V4L2_TUNER_MODE_MONO : V4L2_TUNER_MODE_STEREO;

fmtrx_v4l2_iops_gtuner_exit:
	return err;
}

static int fmtrx_v4l2_iops_stuner(
		struct file *fp,
		void *priv,
		const struct v4l2_tuner *data)
{
	int err = 0;

	/* Validate input arguments */
	if ((0 == data) || (0 != data->index)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_iops_stuner_exit;
	}

	/* Force to mono mode, if audmode is set as
			V4L2_TUNER_MODE_MONO */
	err = fmrx_set_force_mono(
			(V4L2_TUNER_MODE_MONO == data->audmode) ?
			true : false);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Set force mono failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_iops_stuner_exit;
	}

fmtrx_v4l2_iops_stuner_exit:
	return err;
}

static int fmtrx_v4l2_iops_sfrequency(
		struct file *fp,
		void *priv,
		const struct v4l2_frequency *data)
{
	int err = 0;
	u32 frequency = 0;

	/* Validate input arguments */
	if ((0 == data) || (V4L2_TUNER_RADIO != data->type) ||
				(0 != data->tuner)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_iops_sfrequency_exit;
	}

	/* Cancel any ongoing seeks */
	err = fmrx_stop();
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Cancelling seek operation failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_iops_sfrequency_exit;
	}

	/* Convert frequency units based on cap flag */
	frequency =
			(V4L2_TUNER_CAP_LOW == v4l2_freq_cap_flag) ?
			HZTOKHZ(HZ_62_5_TO_HZ(data->frequency)) :
			KHZ_62_5_TO_KHZ(data->frequency);

	mutex_lock(&interface_lock);
	err = fmrx_station_tuning(frequency);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Set station tuning failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}
	mutex_unlock(&interface_lock);

fmtrx_v4l2_iops_sfrequency_exit:
	return err;
}

static int fmtrx_v4l2_iops_gfrequency(
		struct file *fp,
		void *priv,
		struct v4l2_frequency *data)
{
	int err = 0;
	u32 tuned_frequency = 0;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_iops_gfrequency_exit;
	}

	/* Get current tuned channel */
	err = fmrx_get_channel_freq(&tuned_frequency);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,FM get channel frequency failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_iops_gfrequency_exit;
	}

	/* Convert frequency units based on cap flag */
	data->frequency =
			(V4L2_TUNER_CAP_LOW == v4l2_freq_cap_flag) ?
			HZ_TO_62_5_HZ(KHZTOHZ(tuned_frequency)) :
			KHZ_TO_62_5_KHZ(tuned_frequency);

fmtrx_v4l2_iops_gfrequency_exit:
	return err;
}

static int fmtrx_v4l2_iops_queryctrl(
		struct file *fp,
		void *priv,
		struct v4l2_queryctrl *data)
{
	int err = 0;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_iops_queryctrl_exit;
	}

	/* Fill the query control with min/max/step/default
			values */
	switch (data->id) {
	case V4L2_CID_AUDIO_MUTE:
		err = v4l2_ctrl_query_fill(data, 0, 1, 1, 0);
		break;
	case V4L2_CID_AUDIO_VOLUME:
		err = v4l2_ctrl_query_fill(data, 0, 100, 1, 88);
		break;
	case V4L2_CID_TUNE_DEEMPHASIS:
		err = v4l2_ctrl_query_fill(data, 0, 2, 1, 1);
		break;
	case V4L2_CID_PRIV_INTEL_FMRX_OUTPUT_MODE:
		err = v4l2_ctrl_query_fill(data, 0, 1, 1, 1);
		break;
	case V4L2_CID_PRIV_INTEL_FMRX_ANTENNA:
		err = v4l2_ctrl_query_fill(data, 0, 3, 1, 1);
		break;
	case V4L2_CID_PRIV_INTEL_FMRX_SB:
		err = v4l2_ctrl_query_fill(data, 0, 2, 1, 2);
		break;
	default:
		err = -EINVAL;
		break;
	}

fmtrx_v4l2_iops_queryctrl_exit:
	return err;
}

static int fmtrx_v4l2_iops_gctrl(
		struct file *fp,
		void *priv,
		struct v4l2_control *data)
{
	int err = 0;
	struct fmrx_config *config = 0;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_iops_gctrl_exit;
	}

	/* Get static configuration */
	err = fmrx_get_config(&config);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,FM get static config failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_v4l2_iops_gctrl_exit;
	}

	switch (data->id) {
	case V4L2_CID_AUDIO_MUTE:
		data->value = config->mute;
		break;
	case V4L2_CID_AUDIO_VOLUME:
		data->value = config->vol_cfg.left;
		break;
	case V4L2_CID_TUNE_DEEMPHASIS:
		data->value = config->band_cfg.deemp;
		break;
	case V4L2_CID_PRIV_INTEL_FMRX_OUTPUT_MODE:
		data->value = config->routing;
		break;
	case V4L2_CID_PRIV_INTEL_FMRX_ANTENNA:
		data->value = config->antenna;
		break;
	case V4L2_CID_PRIV_INTEL_FMRX_SB:
		data->value = config->side;
		break;
	default:
		err = -EINVAL;
		break;
	}

fmtrx_v4l2_iops_gctrl_exit:
	return err;
}

static int fmtrx_v4l2_iops_sctrl(
		struct file *fp,
		void *priv,
		struct v4l2_control *data)
{
	int err = 0;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_iops_sctrl_exit;
	}

	switch (data->id) {
	case V4L2_CID_AUDIO_VOLUME:
		/* Set volume level for both channels */
		err = fmrx_set_volume_level(FMRX_AUD_CHN_ALL, data->value);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FM Set volume failed! %d\n",
				FILE, __func__, __LINE__, err);
			goto fmtrx_v4l2_iops_sctrl_exit;
		}
		break;
	case V4L2_CID_AUDIO_MUTE:
		/* Set mute flag for all channels */
		err = fmrx_set_mute(FMRX_AUD_CHN_ALL,
					data->value);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FM Set mute failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_iops_sctrl_exit;
		}
		break;
	case V4L2_CID_TUNE_DEEMPHASIS: {
		struct band band_cfg;

		err = fmrx_get_band(&band_cfg);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Get band config failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_iops_sctrl_exit;
		}

		band_cfg.deemp =
			(V4L2_DEEMPHASIS_75_uS == data->value) ?
					DEEMP_75US : DEEMP_50US;
		err = fmrx_set_band(&band_cfg);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FM Set band config failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_iops_sctrl_exit;
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_OUTPUT_MODE:
		err = fmrx_set_route(data->value);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FM Set route failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_iops_sctrl_exit;
		}
		break;
	case V4L2_CID_PRIV_INTEL_FMRX_ANTENNA:
		err = fmrx_set_antenna(data->value);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FM Set antenna failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_iops_sctrl_exit;
		}
		break;
	case V4L2_CID_PRIV_INTEL_FMRX_SB:
		err = fmrx_set_sideband(data->value, true);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FM Set sideband failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_v4l2_iops_sctrl_exit;
		}
		break;
	default:
		err = -EINVAL;
		break;
	}

fmtrx_v4l2_iops_sctrl_exit:
	return err;
}

static int fmtrx_v4l2_iops_hw_freq_seek(
		struct file *fp,
		void *priv,
		const struct v4l2_hw_freq_seek *data)
{
	int err = 0;
	enum seek_mode mode = FMRX_SEEK_MODE_INVALID;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_iops_hw_freq_seek_exit;
	}

	/* Check if auto seek is requested */
	if (data->reserved[0]) {
		mode = FMRX_SEEK_MODE_AUTO;
	} else if (data->seek_upward) {
		mode = (data->wrap_around) ? FMRX_SEEK_MODE_UP :
				FMRX_SEEK_MODE_UP_TO_LIMIT;
	} else {
		mode = (data->wrap_around) ? FMRX_SEEK_MODE_DOWN :
				FMRX_SEEK_MODE_DOWN_TO_LIMIT;
	}

	mutex_lock(&interface_lock);
	/* reserved[1] will have the signal level
		to detect */
	err = fmrx_station_seeking(mode);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,FM station seeking failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}
	mutex_unlock(&interface_lock);

fmtrx_v4l2_iops_hw_freq_seek_exit:
	return err;
}

static long fmtrx_v4l2_iops_priv(
		struct file *fp,
		void *priv,
		bool valid_prio,
		unsigned int cmd,
		void *data)
{
	int err = -ENOIOCTLCMD;

	/* Validate input arguments */
	if ((0 == data) &&
		(V4L2_CID_PRIV_INTEL_FMRX_NVM_COMMIT != cmd)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_v4l2_iops_priv_exit;
	}

	switch (cmd) {
	case V4L2_CID_PRIV_INTEL_FMRX_BAND: {
		struct band *band_cfg  = (struct band *) data;

		if (likely(access_ok(VERIFY_READ, band_cfg,
					(sizeof(struct band))))) {
			err = fmrx_set_band(band_cfg);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,FM Set band failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_SNC: {
		struct snc *snc_cfg  = (struct snc *) data;

		if (likely(access_ok(VERIFY_READ, snc_cfg,
					(sizeof(struct snc))))) {
			err = fmrx_set_snc(snc_cfg);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,FM Set SNC failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_SM: {
		struct sm *sm_cfg  = (struct sm *) data;

		if (likely(access_ok(VERIFY_READ, sm_cfg,
					(sizeof(struct sm))))) {
			err = fmrx_set_sm(sm_cfg);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,FM Set SM failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_AGC: {
		struct agc *agc_cfg  = (struct agc *) data;

		if (likely(access_ok(VERIFY_READ, agc_cfg,
				(sizeof(struct agc))))) {
			err = fmrx_set_agc(agc_cfg);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,FM Set AGC failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_RSSI: {
		struct rssi_notify *rssi_cfg  =
					(struct rssi_notify *) data;

		if (likely(access_ok(VERIFY_READ, rssi_cfg,
					(sizeof(struct rssi_notify))))) {
			err = fmrx_set_rssi_notification(rssi_cfg);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,FM Set RSSI failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_SET_HWPARAMS: {
		struct other_params *other_cfg  =
					(struct other_params *) data;

		if (likely(access_ok(VERIFY_READ, other_cfg,
				(sizeof(struct other_params))))) {
			err = fmrx_set_other_params(other_cfg);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,FM Set other params failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_GET_HWPARAMS: {
		struct other_params *other_cfg  =
					(struct other_params *) data;

		if (likely(access_ok(VERIFY_READ, other_cfg,
					(sizeof(struct other_params))))) {
			struct fmrx_config *data = 0;

			err = fmrx_get_config(&data);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,Get config failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
			memcpy((u8 *)other_cfg, (u8 *)&data->other_cfg,
						sizeof(struct other_params));
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_CHANNEL_INFO: {
		struct channel_info *ch_info  =
					(struct channel_info *) data;

		if (likely(access_ok(VERIFY_READ, ch_info,
					(sizeof(struct channel_info))))) {
			err = fmrx_get_channel_info(ch_info);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,FM Get dynamic config failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_STATIC_CFG: {
		struct fmrx_config *fmrx_cfg  = (struct fmrx_config *) data;

		if (likely(access_ok(VERIFY_READ, fmrx_cfg,
				(sizeof(struct fmrx_config))))) {
			struct fmrx_config *temp_cfg = 0;

			err = fmrx_get_config(&temp_cfg);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,FM Get static config failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
			memcpy((u8 *)fmrx_cfg, (u8 *)temp_cfg,
						sizeof(struct fmrx_config));
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMTRX_READ: {
		struct reg_info *reg_cfg  = (struct reg_info *) data;

		if (likely(access_ok(VERIFY_READ, reg_cfg,
				(sizeof(struct reg_info))))) {
			u32 data = 0;

			err = fmtrx_sys_reg_read32(reg_cfg->addr_offs,
						&data);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,Read 32bit failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
			*reg_cfg->data = data;
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMTRX_WRITE: {
		struct reg_info *reg_cfg  = (struct reg_info *) data;

		if (likely(access_ok(VERIFY_READ, reg_cfg,
				(sizeof(struct reg_info))))) {
			err = fmtrx_sys_reg_write32(reg_cfg->addr_offs,
						*reg_cfg->data);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,Read 32bit failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_SET_OFFSETS: {
		struct gain_offsets_pkt *gain_offsets_pkt_cfg  =
					(struct gain_offsets_pkt *) data;

		if (likely(access_ok(VERIFY_READ, gain_offsets_pkt_cfg,
				(sizeof(struct gain_offsets_pkt))))) {
			err = fmtrx_set_gain_offsets(gain_offsets_pkt_cfg);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,Set Gain offsets failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_GET_OFFSETS: {
		struct gain_offsets_pkt *gain_offsets_pkt_cfg  =
					(struct gain_offsets_pkt *) data;

		if (likely(access_ok(VERIFY_READ, gain_offsets_pkt_cfg,
				(sizeof(struct gain_offsets_pkt))))) {
			err = fmtrx_get_gain_offsets(gain_offsets_pkt_cfg);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,Get Gain offsets failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_SET_RSSI_OFFSETS: {
		struct rssi_offsets_pkt *rssi_offsets_pkt_cfg  =
				(struct rssi_offsets_pkt *) data;

		if (likely(access_ok(VERIFY_READ, rssi_offsets_pkt_cfg,
				(sizeof(struct rssi_offsets_pkt))))) {
			err = fmtrx_set_gain_rssi_offsets(rssi_offsets_pkt_cfg);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,Set Gain offsets failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_GET_RSSI_OFFSETS: {
		struct rssi_offsets_pkt *rssi_offsets_pkt_cfg  =
					(struct rssi_offsets_pkt *) data;

		if (likely(access_ok(VERIFY_READ, rssi_offsets_pkt_cfg,
				(sizeof(struct rssi_offsets_pkt))))) {
			err = fmtrx_get_gain_rssi_offsets(rssi_offsets_pkt_cfg);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,Get Gain offsets failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_AF_SWITCH: {
		struct af_info *af  = (struct af_info *) data;

		if (likely(access_ok(VERIFY_READ, af,
				(sizeof(struct af_info))))) {
			err = fmrx_af_switch(af);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,Switch to AF failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMTRX_GET_CORE_VERSION: {
		struct fmr_id *fmr_id_data  = (struct fmr_id *) data;

		if (likely(access_ok(VERIFY_READ, fmr_id_data,
				(sizeof(struct fmr_id))))) {
			err = fmtrx_get_id(fmr_id_data);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,Get core version failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_AUTO_SEEK_REPORT:
		err = 0;
		break;
	case V4L2_CID_PRIV_INTEL_FMRX_RDS_DATA: {
		struct rds_group_pkt *rds_pkt  = (struct rds_group_pkt *) data;

		if (likely(access_ok(VERIFY_READ, rds_pkt,
				(sizeof(struct rds_group_pkt))))) {
			err = fmrx_get_rds_groups(rds_pkt);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,Get RDS groups failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	case V4L2_CID_PRIV_INTEL_FMRX_NVM_COMMIT:
		err = 0;
		break;
	case V4L2_CID_PRIV_INTEL_FMTRX_RDS: {
		struct rds *rds_cfg  = (struct rds *) data;

		if (likely(access_ok(VERIFY_READ, rds_cfg,
				(sizeof(struct rds))))) {
			err = fmrx_set_rds(rds_cfg);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,Set RDS config failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_v4l2_iops_priv_exit;
			}
		}
		break;
	}
	default:
		break;
	}

fmtrx_v4l2_iops_priv_exit:
	return err;
}

/* end of file */
