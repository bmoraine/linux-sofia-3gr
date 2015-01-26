
/*
 * Rockchip vpu/hevc driver.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <asm/cacheflush.h>
#include <linux/uaccess.h>

#include <sofia/mv_svc_hypercalls.h>

#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/platform_device_pm.h>
#endif

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#include "vcodec_service.h"

enum VPU_HW_ID {
	VPU_DEC_ID_9190		= 0x6731,
	VPU_ID_8270		= 0x8270,
	VPU_ID_4831		= 0x4831,
	HEVC_ID			= 0x6867,
};

enum VPU_HW_TYPE_E {
	VPU_DEC_TYPE_9190	= 0,
	VPU_ENC_TYPE_8270	= 0x100,
	VPU_ENC_TYPE_4831	,
};

enum VPU_FREQ {
	VPU_FREQ_200M,
	VPU_FREQ_266M,
	VPU_FREQ_300M,
	VPU_FREQ_400M,
	VPU_FREQ_500M,
	VPU_FREQ_600M,
	VPU_FREQ_DEFAULT,
	VPU_FREQ_BUT,
};

struct VPU_HW_INFO_E {
	enum VPU_HW_ID		hw_id;
	unsigned long		hw_addr;
	unsigned long		enc_offset;
	unsigned long		enc_reg_num;
	unsigned long		enc_io_size;
	unsigned long		dec_offset;
	unsigned long		dec_reg_num;
	unsigned long		dec_io_size;
};

struct extra_info_elem {
	u32 index;
	u32 offset;
};

#define EXTRA_INFO_MAGIC	0x4C4A46

struct extra_info_for_iommu {
	u32 magic;
	u32 cnt;
	struct extra_info_elem elem[20];
};

#define VPU_SERVICE_SHOW_TIME			0

#if VPU_SERVICE_SHOW_TIME
static struct timeval enc_start, enc_end;
static struct timeval dec_start, dec_end;
static struct timeval pp_start,  pp_end;
#endif

#define MHZ					(1000*1000)

#define REG_NUM_9190_DEC			(159)
#define REG_NUM_9190_PP				(42)
#define REG_NUM_9190_DEC_PP			(159)
#define REG_NUM_DEC_PP				(159)

#define REG_NUM_ENC_8270			(96)
#define REG_SIZE_ENC_8270			(0x200)
#define REG_NUM_ENC_4831			(184)
#define REG_SIZE_ENC_4831			(0x400)

#define REG_NUM_HEVC_DEC			(68)

#define SIZE_REG(reg)				((reg)*4)

#define VPU_HW_ID_OFFSET			(0X01b8)
#define HEVC_HW_ID_OFFSET			(0X0)
#define VPU_DEC_REG_OFFSET			(50)
#define DEC_IRQ_TIMEOUT				(0x2000)
#define VPU_HEVC_SWITCH_REG			(0xe4800018)

static struct VPU_HW_INFO_E vpu_hw_set[] = {
	[0] = {
		.hw_id		= VPU_ID_8270,
		.hw_addr	= 0,
		.enc_offset	= 0x0,
		.enc_reg_num	= REG_NUM_ENC_8270,
		.enc_io_size	= REG_NUM_ENC_8270 * 4,
		.dec_offset	= REG_SIZE_ENC_8270,
		.dec_reg_num	= REG_NUM_9190_DEC_PP,
		.dec_io_size	= REG_NUM_9190_DEC_PP * 4,
	},
	[1] = {
		.hw_id		= VPU_ID_4831,
		.hw_addr	= 0,
		.enc_offset	= 0x0,
		.enc_reg_num	= REG_NUM_ENC_4831,
		.enc_io_size	= REG_NUM_ENC_4831 * 4,
		.dec_offset	= REG_SIZE_ENC_4831,
		.dec_reg_num	= REG_NUM_9190_DEC_PP,
		.dec_io_size	= REG_NUM_9190_DEC_PP * 4,
	},
	[2] = {
		.hw_id		= HEVC_ID,
		.hw_addr	= 0,
		.dec_offset	= 0x0,
		.dec_reg_num	= REG_NUM_HEVC_DEC,
		.dec_io_size	= REG_NUM_HEVC_DEC * 4,
	},
	[3] = {
		.hw_id		= VPU_DEC_ID_9190,
		.hw_addr	= 0,
		.enc_offset	= 0x0,
		.enc_reg_num	= 0,
		.enc_io_size	= 0,
		.dec_offset	= REG_SIZE_ENC_4831,
		.dec_reg_num	= REG_NUM_9190_DEC_PP,
		.dec_io_size	= REG_NUM_9190_DEC_PP * 4,
	},

};

#define DEC_INTERRUPT_REGISTER			1
#define PP_INTERRUPT_REGISTER			40
#define ENC_INTERRUPT_REGISTER			109
#define DEC_VPU_INTERRUPT_REGISTER		55

#define DEC_INTERRUPT_BIT			0x100
#define DEC_BUFFER_EMPTY_BIT			0x4000
#define DEC_VPU_INTERRUPT_BIT			0x1
#define DEC_VPU_BUFFER_EMPTY_BIT		0x40
#define PP_INTERRUPT_BIT			0x1
#define ENC_INTERRUPT_BIT			0x1

#define HEVC_DEC_INT_RAW_BIT			0x200
#define HEVC_DEC_STR_ERROR_BIT			0x4000
#define HEVC_DEC_BUS_ERROR_BIT			0x2000
#define HEVC_DEC_BUFFER_EMPTY_BIT		0x10000

#define VPU_REG_EN_ENC				103
#define VPU_REG_ENC_GATE			2
#define VPU_REG_ENC_GATE_BIT			(1<<4)

#define VPU_REG_EN_DEC				57
#define HEVC_REG_EN_DEC				1
#define VPU_REG_DEC_GATE			2
#define VPU_REG_DEC_GATE_BIT			(1<<10)
#define VPU_REG_EN_PP				41
#define VPU_REG_PP_GATE				1
#define VPU_REG_PP_GATE_BIT			(1<<8)
#define VPU_REG_EN_DEC_PP			1
#define VPU_REG_DEC_PP_GATE			61
#define VPU_REG_DEC_PP_GATE_BIT			(1<<8)

enum VPU_DEC_FMT {
	VPU_DEC_FMT_H264,
	VPU_DEC_FMT_MPEG4,
	VPU_DEC_FMT_H263,
	VPU_DEC_FMT_JPEG,
	VPU_DEC_FMT_VC1,
	VPU_DEC_FMT_MPEG2,
	VPU_DEC_FMT_MPEG1,
	VPU_DEC_FMT_VP6,
	VPU_DEC_FMT_RV,
	VPU_DEC_FMT_VP7,
	VPU_DEC_FMT_VP8,
	VPU_DEC_FMT_AVS,
	VPU_DEC_FMT_SVC,
	VPU_DEC_FMT_VC2,
	VPU_DEC_FMT_MVC,
	VPU_DEC_FMT_THEORA,
	VPU_DEC_FMT_RES
};

/*
 * struct for process session which connect to vpu
 */
struct vpu_session {
	enum VPU_CLIENT_TYPE		type;
	/* a linked list of data so we can access them for debugging */
	struct list_head	list_session;
	/* a linked list of register data waiting for process */
	struct list_head	waiting;
	/* a linked list of register data in processing */
	struct list_head	running;
	/* a linked list of register data processed */
	struct list_head	done;
	wait_queue_head_t	wait;
	pid_t			pid;
	atomic_t		task_running;
};

/*
 * struct for process register set
 */
struct vpu_reg {
	enum VPU_CLIENT_TYPE		type;
	enum VPU_FREQ		freq;
	struct vpu_session		*session;
	/* link to vpu service session */
	struct list_head	session_link;
	/* link to register set list */
	struct list_head	status_link;
	unsigned long		size;
	unsigned long		*reg;
};

struct vpu_device {
	atomic_t		irq_count_codec;
	atomic_t		irq_count_pp;
	unsigned long		iobaseaddr;
	unsigned int		iosize;
	u32		*hwregs;
};

enum vcodec_device_id {
	VCODEC_DEVICE_ID_VPU,
	VCODEC_DEVICE_ID_HEVC
};

struct vcodec_mem_region {
	struct list_head srv_lnk;
	struct list_head reg_lnk;
	struct list_head session_lnk;
	/* virtual address for iommu */
	unsigned long iova;
	unsigned long len;
	struct ion_handle *hdl;
};

struct vpu_service_info {
	struct wake_lock	wake_lock;
	struct delayed_work	power_off_work;
	/* lock */
	struct mutex		lock;
	/* link to link_reg in struct vpu_reg */
	struct list_head	waiting;
	/* link to link_reg in struct vpu_reg */
	struct list_head	running;
	/* link to link_reg in struct vpu_reg */
	struct list_head	done;
	/* link to list_session in struct vpu_session */
	struct list_head	session;
	atomic_t		total_running;
	bool			enabled;
	struct vpu_reg			*reg_codec;
	struct vpu_reg			*reg_pproc;
	struct vpu_reg			*reg_resev;
	struct vpuhwdecconfig_t	dec_config;
	struct vpuhwencconfig_t	enc_config;
	struct VPU_HW_INFO_E		*hw_info;
	unsigned long		reg_size;
	bool			auto_freq;
	bool			bug_dec_addr;
	atomic_t		freq_status;

	struct clk		*aclk_vcodec;
	struct clk		*hclk_vcodec;
	struct clk		*clk_core;
	struct clk		*clk_cabac;
	struct clk		*pd_video;

	int			irq_dec;
	int			irq_enc;

	struct vpu_device		enc_dev;
	struct vpu_device		dec_dev;

	struct device		*dev;

	struct cdev		cdev;
	dev_t			dev_t;
	struct class		*cls;
	struct device		*child_dev;

	struct dentry		*debugfs_dir;
	struct dentry		*debugfs_file_regs;

	u32 irq_status;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#endif

	enum vcodec_device_id	dev_id;

	u32			reserved_mode;

	struct delayed_work	simulate_work;
};

struct vpu_request {
	unsigned long *req;
	unsigned long size;
};

/* debugfs root directory for all device (vpu, hevc). */
static struct dentry *parent;
/* mutex for selecting operation registers of vpu or hevc */
static struct mutex g_mode_mutex;

#ifdef CONFIG_DEBUG_FS
static int vcodec_debugfs_init(void);
static void vcodec_debugfs_exit(void);
static struct dentry *vcodec_debugfs_dir(char *dirname,
					 struct dentry *parent);
static int debug_vcodec_open(struct inode *inode, struct file *file);

static const struct file_operations debug_vcodec_fops = {
	.open = debug_vcodec_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

#define VDPU_SOFT_RESET_REG	58
#define VDPU_CLEAR_CACHE_REG	516
#define VEPU_CLEAR_CACHE_REG	772
#define HEVC_CLEAR_CACHE_REG	260

#define VPU_REG_ENABLE(base, reg) { base[reg] = 1; }

#define VDPU_SOFT_RESET(base)	VPU_REG_ENABLE(base, VDPU_SOFT_RESET_REG)
#define VDPU_CLEAR_CACHE(base)	VPU_REG_ENABLE(base, VDPU_CLEAR_CACHE_REG)
#define VEPU_CLEAR_CACHE(base)	VPU_REG_ENABLE(base, VEPU_CLEAR_CACHE_REG)
#define HEVC_CLEAR_CACHE(base)	VPU_REG_ENABLE(base, HEVC_CLEAR_CACHE_REG)

#define VPU_POWER_OFF_DELAY		(4*HZ) /* 4s */
#define VPU_TIMEOUT_DELAY		(2*HZ) /* 2s */
#define VPU_SIMULATE_DELAY		msecs_to_jiffies(15)

#define BIT_VCODEC_SEL_RK3036		(1<<3)
#define BIT_VCODEC_SEL_RK312X		(1<<15)

static void vcodec_enter_mode_nolock(enum vcodec_device_id id,
				     u32 *reserved_mode)
{
	if (reserved_mode) {
		u32 val = 0;

		if (mv_svc_reg_read((uint32_t)VPU_HEVC_SWITCH_REG, &val, -1))
			pr_err("mv_svc_reg_read_service fails\n");
		*reserved_mode = val;
	}

	if (id == VCODEC_DEVICE_ID_HEVC) {
		if (mv_svc_reg_write((uint32_t)VPU_HEVC_SWITCH_REG, 0x1, -1))
			pr_err("mv_svc_reg_write_service fails\n");
	} else {
		if (mv_svc_reg_write((uint32_t)VPU_HEVC_SWITCH_REG, 0x0, -1))
			pr_err("mv_svc_reg_write_service fails\n");
	}
}

static void vcodec_exit_mode_nolock(enum vcodec_device_id id,
				    u32 reserved_mode)
{
	if (mv_svc_reg_write((uint32_t)VPU_HEVC_SWITCH_REG, reserved_mode, -1))
		pr_err("mv_svc_reg_write_service fails\n");
}

static void vcodec_enter_mode(enum vcodec_device_id id)
{
	mutex_lock(&g_mode_mutex);
	vcodec_enter_mode_nolock(id, NULL);
}

static void vcodec_exit_mode(void)
{
	mutex_unlock(&g_mode_mutex);
}

static void vpu_reset(struct vpu_service_info *pservice)
{
	pservice->reg_codec = NULL;
	pservice->reg_pproc = NULL;
	pservice->reg_resev = NULL;
}

static void reg_deinit(struct vpu_service_info *pservice, struct vpu_reg *reg);
static void vpu_service_session_clear(struct vpu_service_info *pservice,
				      struct vpu_session *session)
{
	struct vpu_reg *reg, *n;

	list_for_each_entry_safe(reg, n, &session->waiting, session_link) {
		reg_deinit(pservice, reg);
	}
	list_for_each_entry_safe(reg, n, &session->running, session_link) {
		reg_deinit(pservice, reg);
	}
	list_for_each_entry_safe(reg, n, &session->done, session_link) {
		reg_deinit(pservice, reg);
	}
}

static void vpu_service_dump(struct vpu_service_info *pservice)
{
	int running;
	struct vpu_reg *reg, *reg_tmp;
	struct vpu_session *session, *session_tmp;

	running = atomic_read(&pservice->total_running);
	pr_info("total_running %d\n", running);
	pr_info("reg_codec 0x%.8x\n", (unsigned int)pservice->reg_codec);
	pr_info("reg_pproc 0x%.8x\n", (unsigned int)pservice->reg_pproc);
	pr_info("reg_resev 0x%.8x\n", (unsigned int)pservice->reg_resev);

	list_for_each_entry_safe(session, session_tmp, &pservice->session,
				 list_session) {
		pr_info("session pid %d type %d:\n",
			session->pid, session->type);
		running = atomic_read(&session->task_running);
		pr_info("task_running %d\n", running);
		list_for_each_entry_safe(reg, reg_tmp, &session->waiting,
					 session_link) {
			pr_info("waiting register set 0x%.8x\n",
				(unsigned int)reg);
		}
		list_for_each_entry_safe(reg, reg_tmp, &session->running,
					 session_link) {
			pr_info("running register set 0x%.8x\n",
				(unsigned int)reg);
		}
		list_for_each_entry_safe(reg, reg_tmp, &session->done,
					 session_link) {
			pr_info("done    register set 0x%.8x\n",
				(unsigned int)reg);
		}
	}
}

static void vpu_service_power_off(struct vpu_service_info *pservice)
{
	int total_running;
	char *sta_name;

	if (!pservice->enabled)
		return;

	pservice->enabled = false;
	total_running = atomic_read(&pservice->total_running);
	if (total_running) {
		pr_alert("alert: power off when %d task running!!\n",
			 total_running);
		mdelay(50);
		pr_alert("alert: delay 50 ms for running task\n");
		vpu_service_dump(pservice);
	}

	pr_info("%s: power off...", dev_name(pservice->dev));

#ifdef CONFIG_PLATFORM_DEVICE_PM
	sta_name = (char *)pservice->pm_platdata->pm_state_D3_name;
	device_state_pm_set_state_by_name(pservice->dev,
					  sta_name);
#endif

	wake_unlock(&pservice->wake_lock);
	pr_info("done\n");
}

static inline void vpu_queue_power_off_work(struct vpu_service_info *pservice)
{
	queue_delayed_work(system_nrt_wq, &pservice->power_off_work,
			   VPU_POWER_OFF_DELAY);
}

static void vpu_power_off_work(struct work_struct *work_s)
{
	struct delayed_work *dlwork;
	struct vpu_service_info *pservice;

	dlwork = container_of(work_s, struct delayed_work, work);
	pservice = container_of(dlwork, struct vpu_service_info,
				power_off_work);
	if (mutex_trylock(&pservice->lock)) {
		vpu_service_power_off(pservice);
		mutex_unlock(&pservice->lock);
	} else {
		/* Come back later if the device is busy... */
		vpu_queue_power_off_work(pservice);
	}
}

static void vpu_service_power_on(struct vpu_service_info *pservice)
{
	static ktime_t last;
	ktime_t now = ktime_get();
	char *sta_name;

	if (ktime_to_ns(ktime_sub(now, last)) > NSEC_PER_SEC) {
		cancel_delayed_work_sync(&pservice->power_off_work);
		vpu_queue_power_off_work(pservice);
		last = now;
	}
	if (pservice->enabled)
		return;

	pservice->enabled = true;
	pr_info("%s: power on\n", dev_name(pservice->dev));

#ifdef CONFIG_PLATFORM_DEVICE_PM
	sta_name = (char *)pservice->pm_platdata->pm_state_D0_name;
	device_state_pm_set_state_by_name(pservice->dev,
					  sta_name);
#endif
	wake_lock(&pservice->wake_lock);
}

static inline bool reg_check_rmvb_wmv(struct vpu_reg *reg)
{
	unsigned long type = (reg->reg[3] & 0xF0000000) >> 28;

	return ((type == 8) || (type == 4));
}

static inline bool reg_check_interlace(struct vpu_reg *reg)
{
	unsigned long type = (reg->reg[3] & (1 << 23));

	return (type > 0);
}

static inline enum VPU_DEC_FMT reg_check_fmt(struct vpu_reg *reg)
{
	enum VPU_DEC_FMT type;

	type = (enum VPU_DEC_FMT)((reg->reg[3] & 0xF0000000) >> 28);
	return type;
}

static inline int reg_probe_width(struct vpu_reg *reg)
{
	int width_in_mb = reg->reg[4] >> 23;

	return width_in_mb * 16;
}

static struct vpu_reg *reg_init(struct vpu_service_info *pservice,
				struct vpu_session *session,
				void __user *src,
				unsigned long size)
{
	int extra_size = 0;
	struct extra_info_for_iommu extra_info;
	struct vpu_reg *reg;

	reg = kmalloc(sizeof(*reg)+pservice->reg_size, GFP_KERNEL);
	if (NULL == reg)
		return NULL;

	if (size > pservice->reg_size) {
		extra_size = size - pservice->reg_size;
		size = pservice->reg_size;
	}
	reg->session = session;
	reg->type = session->type;
	reg->size = size;
	reg->freq = VPU_FREQ_DEFAULT;
	reg->reg = (unsigned long *)&reg[1];
	INIT_LIST_HEAD(&reg->session_link);
	INIT_LIST_HEAD(&reg->status_link);

	if (copy_from_user(&reg->reg[0], (void __user *)src, size)) {
		pr_err("error: copy_from_user failed in reg_init\n");
		kfree(reg);
		return NULL;
	}

	if (copy_from_user(&extra_info, (u8 *)src + size, extra_size)) {
		pr_err("error: copy_from_user failed in reg_init\n");
		kfree(reg);
		return NULL;
	}

	mutex_lock(&pservice->lock);
	list_add_tail(&reg->status_link, &pservice->waiting);
	list_add_tail(&reg->session_link, &session->waiting);
	mutex_unlock(&pservice->lock);

	if (pservice->auto_freq) {
		if (reg->type == VPU_DEC || reg->type == VPU_DEC_PP) {
			if (reg_check_rmvb_wmv(reg)) {
				reg->freq = VPU_FREQ_200M;
			} else if (reg_check_fmt(reg) == VPU_DEC_FMT_H264) {
				if (reg_probe_width(reg) > 3200) {
					/* raise frequency for 4k avc. */
					reg->freq = VPU_FREQ_500M;
				}
			} else {
				if (reg_check_interlace(reg))
					reg->freq = VPU_FREQ_400M;
			}
		}
		if (reg->type == VPU_PP)
			reg->freq = VPU_FREQ_400M;
	}

	return reg;
}

static void reg_deinit(struct vpu_service_info *pservice, struct vpu_reg *reg)
{
	list_del_init(&reg->session_link);
	list_del_init(&reg->status_link);
	if (reg == pservice->reg_codec)
		pservice->reg_codec = NULL;
	if (reg == pservice->reg_pproc)
		pservice->reg_pproc = NULL;

	kfree(reg);
}

static void reg_from_wait_to_run(struct vpu_service_info *pservice,
				 struct vpu_reg *reg)
{
	list_del_init(&reg->status_link);
	list_add_tail(&reg->status_link, &pservice->running);

	list_del_init(&reg->session_link);
	list_add_tail(&reg->session_link, &reg->session->running);
}

static void reg_copy_from_hw(struct vpu_reg *reg, u32 *src, u32 count)
{
	int i;
	u32 *dst = (u32 *)&reg->reg[0];

	for (i = 0; i < count; i++)
		*dst++ = *src++;
}

static void reg_from_run_to_done(struct vpu_service_info *pservice,
				 struct vpu_reg *reg)
{
	int irq_reg = -1;

	list_del_init(&reg->status_link);
	list_add_tail(&reg->status_link, &pservice->done);

	list_del_init(&reg->session_link);
	list_add_tail(&reg->session_link, &reg->session->done);

	vcodec_enter_mode(pservice->dev_id);
	switch (reg->type) {
	case VPU_ENC: {
		pservice->reg_codec = NULL;
		reg_copy_from_hw(reg, pservice->enc_dev.hwregs,
				 pservice->hw_info->enc_reg_num);
		irq_reg = ENC_INTERRUPT_REGISTER;
		break;
	}
	case VPU_DEC: {
		int reg_len = pservice->hw_info->hw_id == HEVC_ID ?
			      REG_NUM_HEVC_DEC : REG_NUM_9190_DEC;

		pservice->reg_codec = NULL;
		reg_copy_from_hw(reg, pservice->dec_dev.hwregs, reg_len);
		irq_reg = DEC_INTERRUPT_REGISTER;
		if (HEVC_ID != pservice->hw_info->hw_id)
			irq_reg = DEC_VPU_INTERRUPT_REGISTER;
		break;
	}
	case VPU_PP: {
		pservice->reg_pproc = NULL;
		reg_copy_from_hw(reg, pservice->dec_dev.hwregs,
				 REG_NUM_9190_PP);
		pservice->dec_dev.hwregs[PP_INTERRUPT_REGISTER] = 0;
		break;
	}
	case VPU_DEC_PP: {
		pservice->reg_codec = NULL;
		pservice->reg_pproc = NULL;
		reg_copy_from_hw(reg, pservice->dec_dev.hwregs,
				 REG_NUM_9190_DEC_PP);
		pservice->dec_dev.hwregs[PP_INTERRUPT_REGISTER] = 0;
		break;
	}
	default: {
		pr_err("copy reg from hw with unknown type %d\n", reg->type);
		break;
	}
	}
	vcodec_exit_mode();

	if (irq_reg != -1)
		reg->reg[irq_reg] = pservice->irq_status;

	atomic_sub(1, &reg->session->task_running);
	atomic_sub(1, &pservice->total_running);
	wake_up(&reg->session->wait);
}

static void reg_copy_to_hw(struct vpu_service_info *pservice,
			   struct vpu_reg *reg)
{
	int i;
	u32 *src = (u32 *)&reg->reg[0];

	atomic_add(1, &pservice->total_running);
	atomic_add(1, &reg->session->task_running);

	vcodec_enter_mode(pservice->dev_id);

	switch (reg->type) {
	case VPU_ENC: {
		int enc_count = pservice->hw_info->enc_reg_num;
		u32 *dst = (u32 *)pservice->enc_dev.hwregs;

		pservice->reg_codec = reg;

		dst[VPU_REG_EN_ENC] = src[VPU_REG_EN_ENC] & 0x30;

		for (i = 0; i < VPU_REG_EN_ENC; i++)
			dst[i] = src[i];

		for (i = VPU_REG_EN_ENC + 1; i < enc_count; i++)
			dst[i] = src[i];

		VEPU_CLEAR_CACHE(dst);

		dst[VPU_REG_EN_ENC]   = src[VPU_REG_EN_ENC];

#if VPU_SERVICE_SHOW_TIME
		do_gettimeofday(&enc_start);
#endif
		break;
	}
	case VPU_DEC: {
		u32 *dst = (u32 *)pservice->dec_dev.hwregs;

		pservice->reg_codec = reg;

		if (pservice->hw_info->hw_id != HEVC_ID) {
			for (i = REG_NUM_9190_DEC - 1; i >= 0; i--) {
				if (VPU_REG_EN_DEC != i)
					dst[i] = src[i];
			}
			VDPU_CLEAR_CACHE(dst);
		} else {
			for (i = REG_NUM_HEVC_DEC - 1; i > HEVC_REG_EN_DEC; i--)
				dst[i] = src[i];
			HEVC_CLEAR_CACHE(dst);
		}

		if (pservice->hw_info->hw_id != HEVC_ID)
			dst[VPU_REG_EN_DEC] = src[VPU_REG_EN_DEC] | (1<<4);
		else
			dst[HEVC_REG_EN_DEC] = src[HEVC_REG_EN_DEC];
#if VPU_SERVICE_SHOW_TIME
		do_gettimeofday(&dec_start);
#endif
		break;
	}
	case VPU_PP: {
		u32 *dst = (u32 *)pservice->dec_dev.hwregs;

		pservice->reg_pproc = reg;
		for (i = 0; i < VPU_REG_EN_PP; i++)
			dst[i] = src[i];

		dst[VPU_REG_EN_PP] = src[VPU_REG_EN_PP];

#if VPU_SERVICE_SHOW_TIME
		do_gettimeofday(&pp_start);
#endif
		break;
	}
	case VPU_DEC_PP: {
		u32 *dst = (u32 *)pservice->dec_dev.hwregs;

		pservice->reg_codec = reg;
		pservice->reg_pproc = reg;

		VDPU_SOFT_RESET(dst);
		VDPU_CLEAR_CACHE(dst);

		for (i = 0; i < REG_NUM_9190_DEC_PP; i++) {
			if (VPU_REG_EN_DEC != i)
				dst[i] = src[i];
		}

		VDPU_CLEAR_CACHE(dst);
		dst[VPU_REG_EN_DEC]	 = src[VPU_REG_EN_DEC];

#if VPU_SERVICE_SHOW_TIME
		do_gettimeofday(&dec_start);
#endif
		break;
	}
	default: {
		pr_err("error: unsupport session type %d", reg->type);
		atomic_sub(1, &pservice->total_running);
		atomic_sub(1, &reg->session->task_running);
		break;
	}
	}

	vcodec_exit_mode();
}

static void try_set_reg(struct vpu_service_info *pservice)
{
	/* first get reg from reg list */
	if (!list_empty(&pservice->waiting)) {
		int can_set = 0;
		struct vpu_reg *reg = list_entry(pservice->waiting.next,
					  struct vpu_reg, status_link);

		vpu_service_power_on(pservice);

		switch (reg->type) {
		case VPU_ENC:
			if ((NULL == pservice->reg_codec) &&
			    (NULL == pservice->reg_pproc))
				can_set = 1;
			break;
		case VPU_DEC:
			if (NULL == pservice->reg_codec)
				can_set = 1;
			if (pservice->auto_freq &&
			    (NULL != pservice->reg_pproc)) {
				can_set = 0;
			}
			break;
		case VPU_PP:
			if (NULL == pservice->reg_codec) {
				if (NULL == pservice->reg_pproc)
					can_set = 1;
			} else {
				if ((VPU_DEC == pservice->reg_codec->type) &&
				    (NULL == pservice->reg_pproc))
					can_set = 1;
				/* can not charge frequency
				 * when vpu is working
				 */
				if (pservice->auto_freq)
					can_set = 0;
			}
			break;
		case VPU_DEC_PP:
			if ((NULL == pservice->reg_codec) &&
			    (NULL == pservice->reg_pproc))
				can_set = 1;
			break;
		default:
			pr_info("undefined reg type %d\n", reg->type);
			break;
		}
		if (can_set) {
			reg_from_wait_to_run(pservice, reg);
			reg_copy_to_hw(pservice, reg);
		}
	}
}

static int return_reg(struct vpu_service_info *pservice,
		      struct vpu_reg *reg, u32 __user *dst)
{
	int ret = 0;
	int reg_len = 0;

	switch (reg->type) {
	case VPU_ENC:
		if (copy_to_user(dst, &reg->reg[0],
				 pservice->hw_info->enc_io_size))
			ret = -EFAULT;
		break;
	case VPU_DEC:
		reg_len = pservice->hw_info->hw_id == HEVC_ID ?
				REG_NUM_HEVC_DEC : REG_NUM_9190_DEC;
		if (copy_to_user(dst, &reg->reg[0], SIZE_REG(reg_len)))
			ret = -EFAULT;
		break;
	case VPU_PP:
		if (copy_to_user(dst, &reg->reg[0],
				 SIZE_REG(REG_NUM_9190_PP)))
			ret = -EFAULT;
		break;
	case VPU_DEC_PP:
		if (copy_to_user(dst, &reg->reg[0],
				 SIZE_REG(REG_NUM_9190_DEC_PP)))
			ret = -EFAULT;
		break;
	default:
		ret = -EFAULT;
		pr_err("copy reg to user with unknown type %d\n", reg->type);
		break;
	}
	reg_deinit(pservice, reg);
	return ret;
}

static long vpu_service_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{
	struct vpu_service_info *pservice;
	struct vpu_session *session;
	atomic_t *tsk_running;
	int ret = 0;

	pservice = container_of(filp->f_dentry->d_inode->i_cdev,
				struct vpu_service_info, cdev);
	session = (struct vpu_session *)filp->private_data;
	tsk_running = &session->task_running;

	if (NULL == session)
		return -EINVAL;

	switch (cmd) {
	case VPU_IOC_SET_CLIENT_TYPE: {
		session->type = (enum VPU_CLIENT_TYPE)arg;
		break;
	}
	case VPU_IOC_GET_HW_FUSE_STATUS: {
		struct vpu_request req;
		int size;

		if (copy_from_user(&req, (void __user *)arg,
				   sizeof(struct vpu_request))) {
			pr_err("%s: %u\n", __func__, __LINE__);
			ret = -EFAULT;
		} else {
			if (VPU_ENC != session->type) {
				size = sizeof(struct vpuhwdecconfig_t);
				if (copy_to_user((void __user *)req.req,
						 &pservice->dec_config,
						 size)) {
					pr_err("%s: %u\n", __func__, __LINE__);
					ret = -EFAULT;
				}
			} else {
				size = sizeof(struct vpuhwencconfig_t);
				if (copy_to_user((void __user *)req.req,
						 &pservice->enc_config,
						 size)) {
					pr_err("%s: %u\n", __func__, __LINE__);
					ret = -EFAULT;
				}
			}
		}

		break;
	}
	case VPU_IOC_SET_REG: {
		struct vpu_request req;
		struct vpu_reg *reg;

		if (copy_from_user(&req, (void __user *)arg,
				   sizeof(struct vpu_request))) {
			pr_err("%s: %u\n", __func__, __LINE__);
			return -EFAULT;
		}
		reg = reg_init(pservice, session,
			       (void __user *)req.req, req.size);
		if (NULL == reg) {
			ret = -EFAULT;
		} else {
			mutex_lock(&pservice->lock);
			try_set_reg(pservice);
			mutex_unlock(&pservice->lock);
		}

		break;
	}
	case VPU_IOC_GET_REG: {
		struct vpu_request req;
		struct vpu_reg *reg;

		if (copy_from_user(&req, (void __user *)arg,
				   sizeof(struct vpu_request))) {
			pr_err("%s: %u\n", __func__, __LINE__);
			ret = -EFAULT;
		} else {
			ret = wait_event_timeout(session->wait,
						 !list_empty(&session->done),
						 VPU_TIMEOUT_DELAY);
			if (!list_empty(&session->done)) {
				if (ret < 0) {
					pr_err("pid %d wait_evernt ret %d\n",
					       session->pid, ret);
				}
				ret = 0;
			} else {
				if (unlikely(ret < 0)) {
					pr_err("pid %d wait task ret %d\n",
					       session->pid, ret);
				} else if (0 == ret) {
					pr_err("pid %d wait %d timeout\n",
					       session->pid,
					       atomic_read(tsk_running));
					ret = -ETIMEDOUT;
				}
			}
			if (ret < 0) {
				int task_r = atomic_read(tsk_running);

				mutex_lock(&pservice->lock);
				vpu_service_dump(pservice);
				if (task_r) {
					atomic_set(tsk_running, 0);
					atomic_sub(task_r,
						   &pservice->total_running);
					pr_err("task %d not ret, reset hw.",
					       task_r);
					vpu_reset(pservice);
					pr_err("done\n");
				}
				vpu_service_session_clear(pservice, session);
				mutex_unlock(&pservice->lock);
				return ret;
			}
		}
		mutex_lock(&pservice->lock);
		reg = list_entry(session->done.next, struct vpu_reg,
				 session_link);
		return_reg(pservice, reg, (u32 __user *)req.req);
		mutex_unlock(&pservice->lock);
		break;
	}
	case VPU_IOC_PROBE_IOMMU_STATUS: {
		int iommu_enable = 0;

		if (copy_to_user((void __user *)arg,
				 &iommu_enable, sizeof(int))) {
			pr_err("%s: %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	}
	default: {
		pr_err("unknown vpu service ioctl cmd %x\n", cmd);
		break;
	}
	}

	return ret;
}

static int vpu_service_check_hw(struct vpu_service_info *p,
				unsigned long hw_addr)
{
	int ret = -EINVAL, i = 0;
	u32 *tmp = (u32 *)ioremap_nocache(hw_addr, 0x4);
	u32 enc_id = *tmp;

	enc_id = (enc_id >> 16) & 0xFFFF;
	pr_info("checking hw id %x\n", enc_id);
	p->hw_info = NULL;
	for (i = 0; i < ARRAY_SIZE(vpu_hw_set); i++) {
		if (enc_id == vpu_hw_set[i].hw_id) {
			p->hw_info = &vpu_hw_set[i];
			ret = 0;
			break;
		}
	}
	iounmap((void *)tmp);
	return ret;
}

static int vpu_service_open(struct inode *inode, struct file *filp)
{
	struct vpu_service_info *pservice;
	struct vpu_session *session;

	pservice = container_of(inode->i_cdev, struct vpu_service_info, cdev);
	session = kmalloc(sizeof(*session), GFP_KERNEL);
	if (NULL == session) {
		pr_err("unable to allocate memory for vpu_session.\n");
		return -ENOMEM;
	}

	session->type	= VPU_TYPE_BUTT;
	session->pid	= current->pid;
	INIT_LIST_HEAD(&session->waiting);
	INIT_LIST_HEAD(&session->running);
	INIT_LIST_HEAD(&session->done);
	INIT_LIST_HEAD(&session->list_session);
	init_waitqueue_head(&session->wait);
	atomic_set(&session->task_running, 0);
	mutex_lock(&pservice->lock);
	list_add_tail(&session->list_session, &pservice->session);
	filp->private_data = (void *)session;
	mutex_unlock(&pservice->lock);

	pr_debug("dev opened\n");
	return nonseekable_open(inode, filp);
}

static int vpu_service_release(struct inode *inode, struct file *filp)
{
	struct vpu_service_info *pservice;
	int task_running;
	struct vpu_session *session;

	pservice = container_of(inode->i_cdev, struct vpu_service_info, cdev);
	session = (struct vpu_session *)filp->private_data;
	if (NULL == session)
		return -EINVAL;

	task_running = atomic_read(&session->task_running);
	if (task_running) {
		pr_err("session %d still has %d task running.\n",
		       session->pid, task_running);
		msleep(50);
	}
	wake_up(&session->wait);

	mutex_lock(&pservice->lock);
	/* remove this filp from the asynchronusly notified filp's */
	list_del_init(&session->list_session);
	vpu_service_session_clear(pservice, session);
	kfree(session);
	filp->private_data = NULL;
	mutex_unlock(&pservice->lock);

	pr_debug("dev closed\n");
	return 0;
}

static const struct file_operations vpu_service_fops = {
	.unlocked_ioctl = vpu_service_ioctl,
	.open		= vpu_service_open,
	.release	= vpu_service_release,
};

static irqreturn_t vdpu_irq(int irq, void *dev_id);
static irqreturn_t vdpu_isr(int irq, void *dev_id);
static irqreturn_t vepu_irq(int irq, void *dev_id);
static irqreturn_t vepu_isr(int irq, void *dev_id);
static void get_hw_info(struct vpu_service_info *pservice);

static void *sofia_vpubase;

static int vcodec_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res = NULL;
	struct device *dev = &pdev->dev;
	void __iomem *regs = NULL;
	struct device_node *np = pdev->dev.of_node;
	char *prop = (char *)dev_name(dev);
	struct vpu_service_info *pservice = NULL;
	struct vpu_device *edev, *ddev;
	struct VPU_HW_INFO_E *hwinfo;

	pservice = devm_kzalloc(dev, sizeof(struct vpu_service_info),
				GFP_KERNEL);

	pr_info("probe device %s\n", dev_name(dev));

	of_property_read_string(np, "hwname", (const char **)&prop);
	dev_set_name(dev, prop);

	if (strcmp(dev_name(dev), "hevc_service") == 0) {
		pservice->dev_id = VCODEC_DEVICE_ID_HEVC;
	} else if (strcmp(dev_name(dev), "vpu_service") == 0) {
		pservice->dev_id = VCODEC_DEVICE_ID_VPU;
	} else {
		dev_err(dev, "unknown device %s\n", dev_name(dev));
		return -1;
	}

#ifdef CONFIG_PLATFORM_DEVICE_PM
	pservice->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pservice->pm_platdata)) {
		dev_err(dev, "Error during device state pm init\n");
		return -ENOMEM;
	}
	ret = device_state_pm_set_class(dev,
					pservice->pm_platdata->pm_user_name);
#endif

	mutex_init(&g_mode_mutex);
	vcodec_enter_mode(pservice->dev_id);

	wake_lock_init(&pservice->wake_lock, WAKE_LOCK_SUSPEND, "vpu");
	INIT_LIST_HEAD(&pservice->waiting);
	INIT_LIST_HEAD(&pservice->running);
	INIT_LIST_HEAD(&pservice->done);
	INIT_LIST_HEAD(&pservice->session);
	mutex_init(&pservice->lock);
	pservice->reg_codec	= NULL;
	pservice->reg_pproc	= NULL;
	atomic_set(&pservice->total_running, 0);
	pservice->enabled = false;
	pservice->dev = dev;

	INIT_DELAYED_WORK(&pservice->power_off_work, vpu_power_off_work);

	vpu_service_power_on(pservice);

	mdelay(1);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	res->flags &= ~IORESOURCE_CACHEABLE;

	if (sofia_vpubase) {
		regs = sofia_vpubase;
	} else {
		regs = devm_ioremap_resource(pservice->dev, res);
		if (IS_ERR(regs)) {
			ret = PTR_ERR(regs);
			goto err;
		}
		sofia_vpubase = regs;
	}

	{
		u32 offset = res->start;

		if (VCODEC_DEVICE_ID_VPU == pservice->dev_id)
			offset += VPU_HW_ID_OFFSET;
		else if (VCODEC_DEVICE_ID_HEVC == pservice->dev_id)
			offset += HEVC_HW_ID_OFFSET;
		ret = vpu_service_check_hw(pservice, offset);
		if (ret < 0) {
			pr_err("hw info check failed\n");
			goto err;
		}
	}

	/* define regs address. */
	ddev = &pservice->dec_dev;
	edev = &pservice->enc_dev;
	hwinfo = pservice->hw_info;

	ddev->iobaseaddr = res->start + hwinfo->dec_offset;
	ddev->iosize = hwinfo->dec_io_size;
	ddev->hwregs = (u32 *)((u8 *)regs + hwinfo->dec_offset);
	pservice->reg_size = ddev->iosize;

	if (pservice->hw_info->hw_id != HEVC_ID) {
		edev->iobaseaddr = res->start + hwinfo->enc_offset;
		edev->iosize = hwinfo->enc_io_size;
		edev->hwregs = (u32 *)((u8 *)regs+hwinfo->enc_offset);
		pservice->reg_size = pservice->reg_size > edev->iosize ?
					pservice->reg_size : edev->iosize;

		pservice->irq_enc = platform_get_irq_byname(pdev, "irq_enc");
		if (pservice->irq_enc < 0) {
			dev_err(pservice->dev, "cannot find IRQ encoder\n");
			ret = -ENXIO;
			goto err;
		}

		ret = devm_request_threaded_irq(pservice->dev,
						pservice->irq_enc,
						vepu_irq, vepu_isr, 0,
						dev_name(pservice->dev),
						(void *)pservice);
		if (ret) {
			dev_err(pservice->dev, "can't request vepu irq %d\n",
				pservice->irq_enc);
			goto err;
		}
	}

	pservice->irq_dec = platform_get_irq_byname(pdev, "irq_dec");
	if (pservice->irq_dec < 0) {
		dev_err(pservice->dev, "cannot find IRQ decoder\n");
		ret = -ENXIO;
		goto err;
	}

	/* get the IRQ line */
	ret = devm_request_threaded_irq(pservice->dev, pservice->irq_dec,
					vdpu_irq, vdpu_isr, 0,
					dev_name(pservice->dev),
					(void *)pservice);
	if (ret) {
		dev_err(pservice->dev, "can't request vdpu irq %d\n",
			pservice->irq_dec);
		goto err;
	}

	atomic_set(&pservice->dec_dev.irq_count_codec, 0);
	atomic_set(&pservice->dec_dev.irq_count_pp, 0);
	atomic_set(&pservice->enc_dev.irq_count_codec, 0);
	atomic_set(&pservice->enc_dev.irq_count_pp, 0);

	/* create device */
	ret = alloc_chrdev_region(&pservice->dev_t, 0, 1, dev_name(dev));
	if (ret) {
		dev_err(dev, "alloc dev_t failed\n");
		goto err;
	}

	cdev_init(&pservice->cdev, &vpu_service_fops);

	pservice->cdev.owner = THIS_MODULE;
	pservice->cdev.ops = &vpu_service_fops;

	ret = cdev_add(&pservice->cdev, pservice->dev_t, 1);

	if (ret) {
		dev_err(dev, "add dev_t failed\n");
		goto err;
	}

	pservice->cls = class_create(THIS_MODULE, dev_name(dev));

	if (IS_ERR(pservice->cls)) {
		ret = PTR_ERR(pservice->cls);
		dev_err(dev, "class_create err:%d\n", ret);
		goto err;
	}

	pservice->child_dev = device_create(pservice->cls, dev,
					    pservice->dev_t, NULL,
					    dev_name(dev));

	platform_set_drvdata(pdev, pservice);

	get_hw_info(pservice);

#ifdef CONFIG_DEBUG_FS
	pservice->debugfs_dir =
		vcodec_debugfs_dir((char *)dev_name(dev), parent);
	if (pservice->debugfs_dir == NULL)
		pr_err("create debugfs dir %s failed\n", dev_name(dev));

	pservice->debugfs_file_regs =
		debugfs_create_file("regs", 0664,
				    pservice->debugfs_dir, pservice,
				    &debug_vcodec_fops);
#endif
	vcodec_exit_mode();
	vpu_service_power_off(pservice);

	pr_info("init success\n");

	return 0;

err:
	pr_info("init failed\n");
	vcodec_exit_mode();
	vpu_service_power_off(pservice);
	wake_lock_destroy(&pservice->wake_lock);

	if (res)
		devm_release_mem_region(&pdev->dev, res->start,
					resource_size(res));
	if (pservice->irq_enc > 0)
		free_irq(pservice->irq_enc, (void *)pservice);
	if (pservice->irq_dec > 0)
		free_irq(pservice->irq_dec, (void *)pservice);

	if (pservice->child_dev) {
		device_destroy(pservice->cls, pservice->dev_t);
		cdev_del(&pservice->cdev);
		unregister_chrdev_region(pservice->dev_t, 1);
	}

	if (pservice->cls)
		class_destroy(pservice->cls);

	return ret;
}

static int vcodec_remove(struct platform_device *pdev)
{
	struct vpu_service_info *pservice = platform_get_drvdata(pdev);
	struct resource *res;

	device_destroy(pservice->cls, pservice->dev_t);
	class_destroy(pservice->cls);
	cdev_del(&pservice->cdev);
	unregister_chrdev_region(pservice->dev_t, 1);

	free_irq(pservice->irq_enc, (void *)&pservice->enc_dev);
	free_irq(pservice->irq_dec, (void *)&pservice->dec_dev);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	devm_release_mem_region(&pdev->dev, res->start, resource_size(res));
	wake_lock_destroy(&pservice->wake_lock);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(pservice->debugfs_file_regs);
	debugfs_remove(pservice->debugfs_dir);
#endif

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id vcodec_service_dt_ids[] = {
	{.compatible = "rockchip,vpu_service",},
	{.compatible = "rockchip,hevc_service",},
	{},
};
#endif

static struct platform_driver vcodec_driver = {
	.probe = vcodec_probe,
	.remove = vcodec_remove,
	.driver = {
		.name = "vcodec",
		.owner = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(vcodec_service_dt_ids),
#endif
	},
};

static void get_hw_info(struct vpu_service_info *pservice)
{
	struct vpuhwdecconfig_t *dec = &pservice->dec_config;
	struct vpuhwencconfig_t *enc = &pservice->enc_config;

	if (pservice->dev_id == VCODEC_DEVICE_ID_VPU) {
		u32 configreg   = pservice->dec_dev.hwregs[VPU_DEC_HWCFG0];

		dec->h264support    = (configreg >> DWL_H264_E) & 0x3U;
		dec->jpegsupport    = (configreg >> DWL_JPEG_E) & 0x01U;
		if (dec->jpegsupport && ((configreg >> DWL_PJPEG_E) & 0x01U))
			dec->jpegsupport = JPEG_PROGRESSIVE;
		dec->mpeg4support   = (configreg >> DWL_MPEG4_E) & 0x3U;
		dec->vc1support     = (configreg >> DWL_VC1_E) & 0x3U;
		dec->mpeg2support   = (configreg >> DWL_MPEG2_E) & 0x01U;
		dec->sorensonsparksupport =
			(configreg >> DWL_SORENSONSPARK_E) & 0x01U;
		dec->refbufsupport  = (configreg >> DWL_REF_BUFF_E) & 0x01U;
		dec->vp6support     = (configreg >> DWL_VP6_E) & 0x01U;
		dec->maxdecpicwidth = 1920;

		configreg = pservice->enc_dev.hwregs[VPU_ENC_HWCFG];
		enc->maxencodedwidth = configreg & ((1 << 11) - 1);
		enc->h264enabled = (configreg >> 27) & 1;
		enc->mpeg4enabled = (configreg >> 26) & 1;
		enc->jpegenabled = (configreg >> 25) & 1;
		enc->vsenabled = (configreg >> 24) & 1;
		enc->rgbenabled = (configreg >> 28) & 1;
		enc->reg_size = pservice->reg_size;
		enc->reserv[0] = 0;
		enc->reserv[1] = 0;

		pservice->auto_freq = false;
		if (pservice->auto_freq) {
			pr_info("vpu_service set to auto frequency mode\n");
			atomic_set(&pservice->freq_status, VPU_FREQ_BUT);
		}

		pservice->bug_dec_addr = false;
	} else {
		dec->maxdecpicwidth = 1920;
		/* disable frequency switch in hevc.*/
		pservice->auto_freq = false;
	}
}

static irqreturn_t vdpu_irq(int irq, void *dev_id)
{
	struct vpu_service_info *pservice = (struct vpu_service_info *)dev_id;
	struct vpu_device *dev = &pservice->dec_dev;
	u32 raw_status;
	u32 irq_status;
	int val;

	vcodec_enter_mode_nolock(pservice->dev_id, &pservice->reserved_mode);

	if (HEVC_ID == pservice->hw_info->hw_id) {
		irq_status = readl(dev->hwregs + DEC_INTERRUPT_REGISTER);
		raw_status = irq_status;
		if (irq_status & DEC_INTERRUPT_BIT) {
			pr_debug("dec_isr dec %x\n", irq_status);
			/* clear dec IRQ */
			writel(0, dev->hwregs + DEC_INTERRUPT_REGISTER);
			atomic_add(1, &dev->irq_count_codec);
		}
	} else {
		irq_status = readl(dev->hwregs + DEC_VPU_INTERRUPT_REGISTER);
		raw_status = irq_status;
		if (irq_status & DEC_VPU_INTERRUPT_BIT) {
			pr_debug("dec_isr dec %x\n", irq_status);
			/* clear dec IRQ */
			val = irq_status & (~DEC_VPU_INTERRUPT_BIT|
					    DEC_VPU_BUFFER_EMPTY_BIT);
			writel(val, dev->hwregs + DEC_VPU_INTERRUPT_REGISTER);
			atomic_add(1, &dev->irq_count_codec);
		}

		irq_status = readl(dev->hwregs + PP_INTERRUPT_REGISTER);
		if (irq_status & PP_INTERRUPT_BIT) {
			pr_debug("vdpu_isr pp  %x\n", irq_status);
			/* clear pp IRQ */
			val = irq_status & (~PP_INTERRUPT_BIT);
			writel(val, dev->hwregs + PP_INTERRUPT_REGISTER);
			atomic_add(1, &dev->irq_count_pp);
		}
	}

	pservice->irq_status = raw_status;

	vcodec_exit_mode_nolock(pservice->dev_id, pservice->reserved_mode);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t vdpu_isr(int irq, void *dev_id)
{
	struct vpu_service_info *pservice = (struct vpu_service_info *)dev_id;
	struct vpu_device *dev = &pservice->dec_dev;

	mutex_lock(&pservice->lock);
	if (atomic_read(&dev->irq_count_codec)) {
#if VPU_SERVICE_SHOW_TIME
		do_gettimeofday(&dec_end);
		pr_info("dec task: %ld ms\n",
			(dec_end.tv_sec  - dec_start.tv_sec)  * 1000 +
			(dec_end.tv_usec - dec_start.tv_usec) / 1000);
#endif
		atomic_sub(1, &dev->irq_count_codec);
		if (NULL == pservice->reg_codec)
			pr_err("dec isr with no task waiting\n");
		else
			reg_from_run_to_done(pservice, pservice->reg_codec);
	}

	if (atomic_read(&dev->irq_count_pp)) {
#if VPU_SERVICE_SHOW_TIME
		do_gettimeofday(&pp_end);
		pr_info("pp  task: %ld ms\n",
			(pp_end.tv_sec  - pp_start.tv_sec)  * 1000 +
			(pp_end.tv_usec - pp_start.tv_usec) / 1000);
#endif

		atomic_sub(1, &dev->irq_count_pp);
		if (NULL == pservice->reg_pproc)
			pr_err("pp isr with no task waiting\n");
		else
			reg_from_run_to_done(pservice, pservice->reg_pproc);
	}
	try_set_reg(pservice);
	mutex_unlock(&pservice->lock);
	return IRQ_HANDLED;
}

static irqreturn_t vepu_irq(int irq, void *dev_id)
{
	struct vpu_service_info *pservice = (struct vpu_service_info *)dev_id;
	struct vpu_device *dev = &pservice->enc_dev;
	u32 irq_status;
	int val;

	vcodec_enter_mode_nolock(pservice->dev_id, &pservice->reserved_mode);
	irq_status = readl(dev->hwregs + ENC_INTERRUPT_REGISTER);

	pr_debug("vepu_irq irq status %x\n", irq_status);

#if VPU_SERVICE_SHOW_TIME
	do_gettimeofday(&enc_end);
	pr_info("enc task: %ld ms\n",
		(enc_end.tv_sec  - enc_start.tv_sec)  * 1000 +
		(enc_end.tv_usec - enc_start.tv_usec) / 1000);
#endif
	if (likely(irq_status & ENC_INTERRUPT_BIT)) {
		/* clear enc IRQ */
		val = irq_status & (~ENC_INTERRUPT_BIT);
		writel(val, dev->hwregs + ENC_INTERRUPT_REGISTER);
		atomic_add(1, &dev->irq_count_codec);
	}

	pservice->irq_status = irq_status;

	vcodec_exit_mode_nolock(pservice->dev_id, pservice->reserved_mode);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t vepu_isr(int irq, void *dev_id)
{
	struct vpu_service_info *pservice = (struct vpu_service_info *)dev_id;
	struct vpu_device *dev = &pservice->enc_dev;

	mutex_lock(&pservice->lock);
	if (atomic_read(&dev->irq_count_codec)) {
		atomic_sub(1, &dev->irq_count_codec);
		if (NULL == pservice->reg_codec)
			pr_err("enc isr with no task waiting\n");
		else
			reg_from_run_to_done(pservice, pservice->reg_codec);
	}
	try_set_reg(pservice);
	mutex_unlock(&pservice->lock);
	return IRQ_HANDLED;
}

static int __init vcodec_service_init(void)
{
	int ret;

	ret = platform_driver_register(&vcodec_driver);
	if (ret != 0) {
		pr_err("Platform device register failed (%d).\n", ret);
		return ret;
	}

#ifdef CONFIG_DEBUG_FS
	vcodec_debugfs_init();
#endif

	return ret;
}

static void __exit vcodec_service_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	vcodec_debugfs_exit();
#endif

	platform_driver_unregister(&vcodec_driver);
}

module_init(vcodec_service_init);
module_exit(vcodec_service_exit);

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>

static int vcodec_debugfs_init(void)
{
	parent = debugfs_create_dir("vcodec", NULL);
	if (!parent)
		return -1;

	return 0;
}

static void vcodec_debugfs_exit(void)
{
	debugfs_remove(parent);
}

static struct dentry *vcodec_debugfs_dir(char *dirname, struct dentry *parent)
{
	return debugfs_create_dir(dirname, parent);
}

static int debug_vcodec_show(struct seq_file *s, void *unused)
{
	struct vpu_service_info *pservice = s->private;
	unsigned int i, n;
	struct vpu_reg *reg, *reg_tmp;
	struct vpu_session *session, *session_tmp;

	mutex_lock(&pservice->lock);
	vpu_service_power_on(pservice);
	if (pservice->hw_info->hw_id != HEVC_ID) {
		seq_puts(s, "\nENC Registers:\n");
		n = pservice->enc_dev.iosize >> 2;
		for (i = 0; i < n; i++) {
			seq_printf(s, "\tswreg%d = %08X\n", i,
				   readl(pservice->enc_dev.hwregs + i));
		}
	}
	seq_puts(s, "\nDEC Registers:\n");
	n = pservice->dec_dev.iosize >> 2;
	for (i = 0; i < n; i++)
		seq_printf(s, "\tswreg%d = %08X\n", i,
			   readl(pservice->dec_dev.hwregs + i));

	seq_puts(s, "\nvpu service status:\n");
	list_for_each_entry_safe(session, session_tmp,
				 &pservice->session, list_session) {
		seq_printf(s, "session pid %d type %d:\n",
			   session->pid, session->type);
		/*seq_printf(s, "waiting reg set %d\n");*/
		list_for_each_entry_safe(reg, reg_tmp, &session->waiting,
					 session_link) {
			seq_puts(s, "waiting register set\n");
		}
		list_for_each_entry_safe(reg, reg_tmp, &session->running,
					 session_link) {
			seq_puts(s, "running register set\n");
		}
		list_for_each_entry_safe(reg, reg_tmp, &session->done,
					 session_link) {
			seq_puts(s, "done    register set\n");
		}
	}
	mutex_unlock(&pservice->lock);

	return 0;
}

static int debug_vcodec_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_vcodec_show, inode->i_private);
}

#endif
