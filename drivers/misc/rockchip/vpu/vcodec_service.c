
/*
 * Rockchip vpu/hevc driver.
 *
 * Copyright (C) 2014-2015 Fuzhou Rockchip Electronics Co., Ltd
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

#if defined(CONFIG_ION_ROCKCHIP)
#include <linux/rockchip_ion.h>
#endif

#if defined(CONFIG_ROCKCHIP_IOMMU) && defined(CONFIG_ION_ROCKCHIP)
#define CONFIG_VCODEC_MMU
#endif

#ifdef CONFIG_VCODEC_MMU
#include <linux/rockchip_iovmm.h>
#include <linux/dma-buf.h>
#endif

#include "vcodec_service.h"

#ifdef CONFIG_X86_INTEL_SOFIA
#include <linux/xgold_noc.h>
#endif

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

#define MHZ					(1000*1000)

#define REG_NUM_9190_DEC			(159)
#define REG_NUM_9190_DEC_START			(50)
#define REG_NUM_9190_DEC_END			(159)
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
#define VPU_SOFT_RST_REGISTER			58

#define DEC_INTERRUPT_BIT			0x100
#define DEC_BUFFER_EMPTY_BIT			0x4000
#define DEC_VPU_INTERRUPT_BIT			0x1
#define DEC_VPU_BUFFER_EMPTY_BIT		0x40
#define DEC_VPU_TIMEOUT_BIT			(0x1<<12)
#define DEC_VPU_ERR_BIT				(0x1<<13)
#define PP_INTERRUPT_BIT			0x1
#define ENC_INTERRUPT_BIT			0x1

#define HEVC_DEC_INT_RAW_BIT			0x200
#define HEVC_DEC_STR_ERROR_BIT			0x4000
#define HEVC_DEC_BUS_ERROR_BIT			0x2000
#define HEVC_DEC_BUFFER_EMPTY_BIT		0x10000
#define HEVC_DEC_TIMEOUT_BIT			(0X1<<15)
#define HEVC_CABAC_ERR_EN			(44)
#define HEVC_CABAC_ERR_EN_BIT			(0xffffffff)
#define HEVC_AUTO_GATING_EN			(0X1<<1)

#define VPU_REG_EN_ENC				103
#define VPU_REG_ENC_GATE			2
#define VPU_REG_ENC_GATE_BIT			(1<<4)

#define VPU_REG_EN_DEC				57
#define VPU_REG_DEC_WORK_BIT			1
#define HEVC_REG_EN_DEC				1
#define VPU_REG_DEC_GATE			2
#define VPU_REG_DEC_GATE_BIT			(1<<10)
#define VPU_REG_EN_PP				41
#define VPU_REG_PP_GATE				1
#define VPU_REG_PP_GATE_BIT			(1<<8)
#define VPU_REG_EN_DEC_PP			1
#define VPU_REG_DEC_PP_GATE			61
#define VPU_REG_DEC_PP_GATE_BIT			(1<<8)
#define VPU_REG_AXI_CTL_REG			56
#define VPU_AXI_DECODER			(0X1<<23)

#define VPU_REG_DEC_FORMAT		53

#if defined(CONFIG_VCODEC_MMU)
static u8 addr_tbl_vpu_h264dec[] = {
	64, 63, 84, 85, 86, 87, 88, 89,
	90, 91, 92, 93, 94, 95, 96, 97,
	98, 99, 61, 62
};

static u8 addr_tbl_vpu_vp8dec[] = {
	149, 64, 63, 131, 136, 137, 140, 141,
	142, 143, 144, 145, 146, 147, 61
};

static u8 addr_tbl_vpu_vp6dec[] = {
	64, 63, 131, 136, 145, 61
};

static u8 addr_tbl_vpu_jpegdec[] = {
	64, 61, 21, 22
};

static u8 addr_tbl_vpu_defaultdec[] = {
	64, 63, 131, 148, 134, 135, 61, 62
};

static u8 addr_tbl_vpu_enc[] = {
	77, 78, 56, 57, 63, 64, 48, 49, 50, 81
};

static u8 addr_tbl_hevc_dec[] = {
	4, 6, 7, 10, 11, 12, 13, 14,
	15, 16, 17, 18, 19, 20, 21, 22,
	23, 24, 42, 43
};

static u8 addr_tbl_pp[] = {
	12, 13, 18, 19, 20, 21, 22
};
#endif

#define VPU_DVFS_ENABLE		0
#define VPU_NUM_PM_STATES	4
#define VPU_MAX_PM_STATE	(VPU_NUM_PM_STATES - 1)
#define VPU_MIN_PM_STATE	1
#define VPU_OFF_PM_STATE	0
#define VPU_MAX_LOAD		80
#define VPU_MID_LOAD		50
#define VPU_LOW_LOAD		30
/*count vpu load per period time ms, value: > 100 ms*/
#define VPU_PERIOD_TIME		1000
#define VPU_MAX_TIME_PER_FRAME	35 /*ms*/

#define HEVC_COMPATIBLE_NAME "rockchip,hevc_service"
#define VPU_COMPATIBLE_NAME "rockchip,vpu_service"
#define VPU_MMU_DEVICE	"vpu_mmu"
#define HEVC_MMU_DEVICE	"hevc_mmu"

static char *vpu_pm_state_name[] = {
	"disable",
	"low_perf",
	"mid_perf",
	"high_perf",
};

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
#if defined(CONFIG_VCODEC_MMU)
	struct list_head	mem_region_list;
#endif
	struct vpu_subdev_data *data;

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
	VCODEC_DEVICE_ID_HEVC,
	VCODEC_DEVICE_ID_COMBO
};

enum vcodec_running_mode {
	VCODEC_RUNNING_MODE_NONE = -1,
	VCODEC_RUNNING_MODE_VPU,
	VCODEC_RUNNING_MODE_HEVC,
};

struct vcodec_mem_region {
	struct list_head srv_lnk;
	struct list_head reg_lnk;
	struct list_head session_lnk;
	/* virtual address for iommu */
	unsigned long iova;
	unsigned long len;
	u32 reg_idx;
	struct ion_handle *hdl;
};

enum vpu_ctx_state {
	MMU_ACTIVATED	= BIT(0)
};

struct vpu_subdev_data {
	struct cdev cdev;
	dev_t dev_t;
	struct class *cls;
	struct device *child_dev;

	int irq_enc;
	int irq_dec;
	struct vpu_service_info *pservice;

	u32 *regs;
	enum vcodec_running_mode mode;
	struct list_head lnk_service;

	struct device *dev;

	struct vpu_device		enc_dev;
	struct vpu_device		dec_dev;
	struct VPU_HW_INFO_E		*hw_info;

	u32 reg_size;
	unsigned long state;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dir;
	struct dentry *debugfs_file_regs;
#endif

#if defined(CONFIG_VCODEC_MMU)
	struct device *mmu_dev;
#endif
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

	bool			auto_freq;
	bool			bug_dec_addr;
	atomic_t		freq_status;

	struct clk		*aclk_vcodec;
	struct clk		*hclk_vcodec;
	struct clk		*clk_core;
	struct clk		*clk_cabac;
	struct clk		*pd_video;

	struct device		*dev;

	u32 irq_status;
#if defined(CONFIG_VCODEC_MMU)
	struct ion_client	*ion_client;
	struct list_head	mem_region_list;
#endif

#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
	struct device_state_pm_state *pm_states[VPU_NUM_PM_STATES];
	unsigned int cur_pm_state;
	unsigned int pre_pm_state;
#endif

	enum vcodec_device_id	dev_id;

	enum vcodec_running_mode curr_mode;
	u32 prev_mode;

	struct delayed_work	simulate_work;

	u32 *reg_base;
	u32 ioaddr;
	u32 subcnt;
	u32 vpu_load;
	u32 vpu_update_load;
	u32 vpu_codec_time;
	u32 vpu_pp_time;
	u32 vpu_time_per_frame;
	u64 vpu_start_time;
	u64 dec_start;
	u64 pp_start;
	u64 enc_start;
	struct timer_list timer;
	struct list_head subdev_list;
	/* for irq handler */
	spinlock_t irq_lock;
	/* for data */
	spinlock_t data_lock;
};

struct vpu_request {
	unsigned long *req;
	unsigned long size;
};

/* debugfs root directory for all device (vpu, hevc). */
static struct dentry *parent;

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

static void vcodec_enter_mode(struct vpu_subdev_data *data)
{
	struct vpu_service_info *pservice = data->pservice;
	struct vpu_subdev_data *subdata, *n;

	if (pservice->curr_mode == data->mode)
		return;

#if defined(CONFIG_VCODEC_MMU)
	list_for_each_entry_safe(subdata, n, &pservice->subdev_list,
				 lnk_service) {
		if (data != subdata && subdata->mmu_dev &&
		    test_bit(MMU_ACTIVATED, &subdata->state)) {
			clear_bit(MMU_ACTIVATED, &subdata->state);
			rockchip_iovmm_deactivate(subdata->dev);
		}
	}
#endif

	if (data->mode == VCODEC_RUNNING_MODE_HEVC) {
		if (mv_svc_reg_write((uint32_t)VPU_HEVC_SWITCH_REG, 0x1, -1))
			pr_err("mv_svc_reg_write_service fails\n");
	} else {
		if (mv_svc_reg_write((uint32_t)VPU_HEVC_SWITCH_REG, 0x0, -1))
			pr_err("mv_svc_reg_write_service fails\n");
	}
#if defined(CONFIG_VCODEC_MMU)
	if (data->mmu_dev && !test_bit(MMU_ACTIVATED, &data->state)) {
		set_bit(MMU_ACTIVATED, &data->state);
		BUG_ON(!pservice->enabled);
		if (pservice->enabled)
			rockchip_iovmm_activate(data->dev);
	}
#endif

	pservice->prev_mode = pservice->curr_mode;
	pservice->curr_mode = data->mode;
}

static void vcodec_exit_mode(struct vpu_subdev_data *data)
{
}

static void vpu_reset(struct vpu_subdev_data *data)
{
	struct vpu_service_info *pservice = data->pservice;

	pservice->reg_codec = NULL;
	pservice->reg_pproc = NULL;
	pservice->reg_resev = NULL;

#if defined(CONFIG_VCODEC_MMU)
	if (data->mmu_dev && test_bit(MMU_ACTIVATED, &data->state)) {
		clear_bit(MMU_ACTIVATED, &data->state);
		BUG_ON(!pservice->enabled);
		if (pservice->enabled)
			rockchip_iovmm_deactivate(data->dev);
	}
#endif
}

static void vpu_reset_iommu(struct vpu_subdev_data *data)
{
#if defined(CONFIG_VCODEC_MMU)
	struct vpu_service_info *psvc = data->pservice;

	if (data->mmu_dev && test_bit(MMU_ACTIVATED, &data->state)) {
		BUG_ON(!psvc->enabled);
		rockchip_iovmm_deactivate(data->dev);
		rockchip_iovmm_activate(data->dev);
	}
#endif
}

static void reg_deinit(struct vpu_subdev_data *data, struct vpu_reg *reg);
static void vpu_service_session_clear(struct vpu_subdev_data *data,
				      struct vpu_session *session)
{
	struct vpu_reg *reg, *n;

	list_for_each_entry_safe(reg, n, &session->waiting, session_link) {
		reg_deinit(data, reg);
	}
	list_for_each_entry_safe(reg, n, &session->running, session_link) {
		reg_deinit(data, reg);
	}
	list_for_each_entry_safe(reg, n, &session->done, session_link) {
		reg_deinit(data, reg);
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

/* need hold pservice lock */
static void vpu_update_load(struct vpu_service_info *psvc)
{
	u32 codec_load = 0, pp_load = 0, base = 0;
	u64 now;

	if (psvc->vpu_update_load) {
		now = ktime_to_ms(ktime_get());
		base = now - psvc->vpu_start_time;
		if (base >= VPU_PERIOD_TIME + 100 ||
		    base <= VPU_PERIOD_TIME - 100) {
			psvc->vpu_codec_time = 0;
			psvc->vpu_pp_time = 0;
		} else {
			/* load = max(codec_time, pp_time)/base.
			 * base is around per sec and caculated as follow:
			 * base = now - psvc->vpu_start_time;
			 */
			codec_load = psvc->vpu_codec_time * 100;
			codec_load /= base;
			psvc->vpu_codec_time = 0;
			pp_load = psvc->vpu_pp_time * 100;
			psvc->vpu_pp_time = 0;
			pp_load /= base;
			psvc->vpu_load = max(codec_load, pp_load);

			pr_info("vpu load: %d, codec: %d, pp: %d, base: %d\n",
				psvc->vpu_load, codec_load, pp_load, base);

			if (psvc->vpu_load <= VPU_MID_LOAD) {
				if (psvc->cur_pm_state > VPU_MIN_PM_STATE)
					psvc->cur_pm_state--;
			} else if (psvc->vpu_load >= VPU_MAX_LOAD) {
				if (psvc->cur_pm_state < VPU_MAX_PM_STATE)
					psvc->cur_pm_state++;
			}
		}
		psvc->vpu_update_load = 0;
		psvc->vpu_start_time = ktime_to_ms(ktime_get());
		mod_timer(&psvc->timer,
			  jiffies + msecs_to_jiffies(VPU_PERIOD_TIME));
	}
}

static void vpu_timer(unsigned long data)
{
	struct vpu_service_info *psvc = (struct vpu_service_info *)data;

	psvc->vpu_update_load = 1;
}

static void vpu_set_pm_state(struct vpu_service_info *psvc)
{
	struct device *dev = psvc->dev;
	int state;
	int ret;
	bool change = false;

	if (psvc->cur_pm_state == VPU_OFF_PM_STATE) {
		psvc->pre_pm_state = psvc->cur_pm_state;
		change = true;
	} else if (psvc->vpu_time_per_frame > VPU_MAX_TIME_PER_FRAME) {
		if (psvc->pre_pm_state != VPU_MAX_PM_STATE ||
		    psvc->cur_pm_state != VPU_MAX_PM_STATE) {
			psvc->pre_pm_state = VPU_MAX_PM_STATE;
			psvc->cur_pm_state = VPU_MAX_PM_STATE;
			change = true;
		}
		dev_dbg(dev, "time: %d\n", psvc->vpu_time_per_frame);
	} else if (psvc->pre_pm_state != psvc->cur_pm_state) {
		psvc->pre_pm_state = psvc->cur_pm_state;
		change = true;
	}

	if (change) {
		state = psvc->cur_pm_state;
		pr_info("->state: %s\n", psvc->pm_states[state]->name);
		ret = device_state_pm_set_state(dev, psvc->pm_states[state]);
		if (ret)
			dev_err(dev, "vpu pm set state failed, ret: %d\n", ret);
	}
}

static void vpu_service_power_off(struct vpu_service_info *pservice)
{
	int total_running;
	struct vpu_subdev_data *data = NULL, *n;

	if (!pservice->enabled)
		return;

	total_running = atomic_read(&pservice->total_running);
	if (total_running) {
		pr_alert("alert: power off when %d task running!!\n",
			 total_running);
		mdelay(50);
		pr_alert("alert: delay 50 ms for running task\n");
		vpu_service_dump(pservice);
	}

#if defined(CONFIG_VCODEC_MMU)
	list_for_each_entry_safe(data, n, &pservice->subdev_list, lnk_service) {
		if (data->mmu_dev && test_bit(MMU_ACTIVATED, &data->state)) {
			clear_bit(MMU_ACTIVATED, &data->state);
			rockchip_iovmm_deactivate(data->dev);
		}
	}
#endif

	pservice->curr_mode = VCODEC_RUNNING_MODE_NONE;

	del_timer_sync(&pservice->timer);

#ifdef CONFIG_PLATFORM_DEVICE_PM
	pservice->cur_pm_state = VPU_OFF_PM_STATE;
	vpu_set_pm_state(pservice);
#endif
	pservice->enabled = false;
	pr_info("%s: power off.\n", dev_name(pservice->dev));
	pservice->enabled = false;
	wake_unlock(&pservice->wake_lock);
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

/* need hold pservice lock */
static void vpu_service_power_on(struct vpu_service_info *pservice)
{
	static ktime_t last;
	ktime_t now = ktime_get();

	if (ktime_to_ns(ktime_sub(now, last)) > NSEC_PER_SEC) {
		cancel_delayed_work_sync(&pservice->power_off_work);
		vpu_queue_power_off_work(pservice);
		last = now;
	}
	if (pservice->enabled)
		return;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	pservice->cur_pm_state = VPU_MAX_PM_STATE;
	vpu_set_pm_state(pservice);
#endif
	pservice->enabled = true;
	pr_info("%s: power on.\n", dev_name(pservice->dev));

	mod_timer(&pservice->timer,
		  jiffies + msecs_to_jiffies(VPU_PERIOD_TIME));
	pservice->vpu_start_time = ktime_to_ms(ktime_get());
	pservice->vpu_update_load = 0;
	pservice->vpu_codec_time = 0;
	pservice->vpu_pp_time = 0;
	xgold_noc_qos_set("VPU");
	xgold_noc_qos_set("VPU128R");
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

	type = (enum VPU_DEC_FMT)(reg->reg[VPU_REG_DEC_FORMAT] & 0xf);
	return type;
}

static inline int reg_probe_width(struct vpu_reg *reg)
{
	int width_in_mb = reg->reg[4] >> 23;

	return width_in_mb * 16;
}

#if defined(CONFIG_VCODEC_MMU)
static int vcodec_fd_to_iova(struct vpu_subdev_data *data,
			     struct vpu_reg *reg, int fd)
{
	struct vpu_service_info *pservice = data->pservice;
	struct ion_handle *hdl;
	int ret = 0;
	struct vcodec_mem_region *mem_region;

	hdl = ion_import_dma_buf(pservice->ion_client, fd);
	if (IS_ERR(hdl)) {
		dev_err(data->dev,
			"import dma-buf from fd %d\n", fd);
		return PTR_ERR(hdl);
	}
	mem_region = kzalloc(sizeof(*mem_region), GFP_KERNEL);

	if (mem_region == NULL) {
		ion_free(pservice->ion_client, hdl);
		return -1;
	}

	mem_region->hdl = hdl;
	ret = ion_map_iommu(data->dev, pservice->ion_client,
			    mem_region->hdl, &mem_region->iova,
			    &mem_region->len);

	if (ret < 0) {
		dev_err(data->dev, "ion map iommu failed\n");
		kfree(mem_region);
		ion_free(pservice->ion_client, hdl);
		return ret;
	}
	INIT_LIST_HEAD(&mem_region->reg_lnk);
	list_add_tail(&mem_region->reg_lnk, &reg->mem_region_list);
	return mem_region->iova;
}

static int vcodec_bufid_to_iova(struct vpu_subdev_data *data, u8 *tbl,
				int size, struct vpu_reg *reg,
				struct extra_info_for_iommu *ext_inf)
{
	struct vpu_service_info *pservice = data->pservice;
	int i;
	int usr_fd = 0;
	int offset = 0;
	enum VPU_HW_ID hw_id = data->hw_info->hw_id;

	if (tbl == NULL || size <= 0) {
		dev_err(pservice->dev, "input arguments invalidate\n");
		return -1;
	}

	mutex_lock(&pservice->lock);
	vpu_service_power_on(pservice);
	mutex_unlock(&pservice->lock);

	for (i = 0; i < size; i++) {
		usr_fd = reg->reg[tbl[i]] & 0x3FF;

		if (tbl[i] == 62 && hw_id != HEVC_ID &&
		    (reg->type == VPU_DEC || reg->type == VPU_DEC_PP))
			/* special for vpu dec num 41 regitster */
			offset = reg->reg[tbl[i]] >> 10 << 4;
		else
			offset = reg->reg[tbl[i]] >> 10;

		if (usr_fd != 0) {
			struct ion_handle *hdl;
			int ret = 0;
			struct vcodec_mem_region *mem_region;
			struct ion_client *ion_cli = pservice->ion_client;

			hdl = ion_import_dma_buf(ion_cli, usr_fd);
			if (IS_ERR(hdl)) {
				dev_err(pservice->dev,
					"dma-buf fd %d failed, reg[%d]\n",
					usr_fd, tbl[i]);
				return PTR_ERR(hdl);
			}

			if (tbl[i] == 42 && hw_id == HEVC_ID) {
				int i = 0;
				char *pps;

				pps = (char *)ion_map_kernel(ion_cli, hdl);
				for (i = 0; i < 64; i++) {
					u32 scal_offset;
					u32 t;
					int scal_fd = 0;

					scal_offset = (u32)pps[i*80+74];
					scal_offset += (u32)pps[i*80+75] << 8;
					scal_offset += (u32)pps[i*80+76] << 16;
					scal_offset += (u32)pps[i*80+77] << 24;
					scal_fd = scal_offset&0x3ff;
					scal_offset = scal_offset >> 10;
					if (scal_fd > 0) {
						t = vcodec_fd_to_iova(data,
								      reg,
								      scal_fd);
						t += scal_offset;
						pps[i*80+74] = t&0xff;
						pps[i*80+75] = (t>>8)&0xff;
						pps[i*80+76] = (t>>16)&0xff;
						pps[i*80+77] = (t>>24)&0xff;
					}
				}
			}

			mem_region = kzalloc(sizeof(*mem_region), GFP_KERNEL);

			if (mem_region == NULL) {
				ion_free(ion_cli, hdl);
				return -1;
			}

			mem_region->hdl = hdl;
			mem_region->reg_idx = tbl[i];

			ret = ion_map_iommu(data->dev,
					    ion_cli,
					    mem_region->hdl,
					    &mem_region->iova,
					    &mem_region->len);

			if (ret < 0) {
				dev_err(pservice->dev, "ion map iommu failed\n");
				kfree(mem_region);
				ion_free(ion_cli, hdl);
				return ret;
			}
			reg->reg[tbl[i]] = mem_region->iova + offset;
			INIT_LIST_HEAD(&mem_region->reg_lnk);
			list_add_tail(&mem_region->reg_lnk,
				      &reg->mem_region_list);
		}
	}

	if (ext_inf != NULL && ext_inf->magic == EXTRA_INFO_MAGIC) {
		int index;

		for (i = 0; i < ext_inf->cnt; i++) {
			pr_info("reg[%d] + offset %d\n",
				ext_inf->elem[i].index,
				ext_inf->elem[i].offset);
			index = ext_inf->elem[i].index;
			reg->reg[index] += ext_inf->elem[i].offset;
		}
	}

	return 0;
}

static int vcodec_reg_address_translate(struct vpu_subdev_data *data,
					struct vpu_reg *reg,
					struct extra_info_for_iommu *ext_inf)
{
	enum VPU_HW_ID hw_id;
	u8 *tbl;
	int size = 0;

	hw_id = data->hw_info->hw_id;

	if (hw_id == HEVC_ID) {
		tbl = addr_tbl_hevc_dec;
		size = sizeof(addr_tbl_hevc_dec);
	} else {
		if (reg->type == VPU_DEC || reg->type == VPU_DEC_PP) {
			switch (reg_check_fmt(reg)) {
			case VPU_DEC_FMT_H264:
				{
					tbl = addr_tbl_vpu_h264dec;
					size = sizeof(addr_tbl_vpu_h264dec);
					break;
				}
			case VPU_DEC_FMT_VP8:
			case VPU_DEC_FMT_VP7:
				{
					tbl = addr_tbl_vpu_vp8dec;
					size = sizeof(addr_tbl_vpu_vp8dec);
					break;
				}

			case VPU_DEC_FMT_VP6:
				{
					tbl = addr_tbl_vpu_vp6dec;
					size = sizeof(addr_tbl_vpu_vp6dec);
					break;
				}

			case VPU_DEC_FMT_JPEG:
				{
					tbl = addr_tbl_vpu_jpegdec;
					size = sizeof(addr_tbl_vpu_jpegdec);
					break;
				}
			default:
				tbl = addr_tbl_vpu_defaultdec;
				size = sizeof(addr_tbl_vpu_defaultdec);
				break;
			}
		} else if (reg->type == VPU_ENC) {
			tbl = addr_tbl_vpu_enc;
			size = sizeof(addr_tbl_vpu_enc);
		} else if (reg->type == VPU_PP) {
			tbl = addr_tbl_pp;
			size = sizeof(addr_tbl_pp);
		}
	}

	if (size != 0)
		return vcodec_bufid_to_iova(data, tbl, size, reg, ext_inf);
	else
		return -1;
}
#endif

static struct vpu_reg *reg_init(struct vpu_subdev_data *data,
				struct vpu_session *session,
				void __user *src,
				unsigned long size)
{
	struct vpu_service_info *pservice = data->pservice;
	int extra_size = 0;
	struct extra_info_for_iommu extra_info;
	struct vpu_reg *reg;

	reg = kmalloc(sizeof(*reg)+data->reg_size, GFP_KERNEL);
	if (NULL == reg)
		return NULL;

	if (size > data->reg_size) {
		extra_size = size - data->reg_size;
		size = data->reg_size;
	}
	reg->session = session;
	reg->type = session->type;
	reg->size = size;
	reg->data = data;
	reg->freq = VPU_FREQ_DEFAULT;
	reg->reg = (unsigned long *)&reg[1];
	INIT_LIST_HEAD(&reg->session_link);
	INIT_LIST_HEAD(&reg->status_link);

#if defined(CONFIG_VCODEC_MMU)
	if (data->mmu_dev)
		INIT_LIST_HEAD(&reg->mem_region_list);
#endif

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

#if defined(CONFIG_VCODEC_MMU)
	if (data->mmu_dev &&
	    0 > vcodec_reg_address_translate(data, reg, &extra_info)) {
		pr_err("error: translate reg address failed\n");
		kfree(reg);
		return NULL;
	}
#endif
	mutex_lock(&pservice->lock);
	list_add_tail(&reg->status_link, &pservice->waiting);
	list_add_tail(&reg->session_link, &session->waiting);
	mutex_unlock(&pservice->lock);

	return reg;
}

static void reg_deinit(struct vpu_subdev_data *data, struct vpu_reg *reg)
{
	struct vpu_service_info *pservice = data->pservice;
#if defined(CONFIG_VCODEC_MMU)
	struct vcodec_mem_region *mem_region = NULL, *n;
#endif

	list_del_init(&reg->session_link);
	list_del_init(&reg->status_link);
	if (reg == pservice->reg_codec)
		pservice->reg_codec = NULL;
	if (reg == pservice->reg_pproc)
		pservice->reg_pproc = NULL;

#if defined(CONFIG_VCODEC_MMU)
	/* release memory region attach to this registers table. */
	if (data->mmu_dev) {
		list_for_each_entry_safe(mem_region, n,
					 &reg->mem_region_list, reg_lnk) {
			ion_free(pservice->ion_client, mem_region->hdl);
			list_del_init(&mem_region->reg_lnk);
			kfree(mem_region);
		}
	}
#endif

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

static void reg_from_run_to_done(struct vpu_subdev_data *data,
				 struct vpu_reg *reg)
{
	struct vpu_service_info *pservice = data->pservice;
	int irq_reg = -1;
	unsigned long flags;

	list_del_init(&reg->status_link);
	list_add_tail(&reg->status_link, &pservice->done);

	list_del_init(&reg->session_link);
	list_add_tail(&reg->session_link, &reg->session->done);

	switch (reg->type) {
	case VPU_ENC: {
		spin_lock_irqsave(&pservice->data_lock, flags);
		pservice->reg_codec = NULL;
		spin_unlock_irqrestore(&pservice->data_lock, flags);
		reg_copy_from_hw(reg, data->enc_dev.hwregs,
				 data->hw_info->enc_reg_num);
		irq_reg = ENC_INTERRUPT_REGISTER;
		break;
	}
	case VPU_DEC: {
		int reg_len = data->hw_info->hw_id == HEVC_ID ?
			      REG_NUM_HEVC_DEC : REG_NUM_9190_DEC;

		spin_lock_irqsave(&pservice->data_lock, flags);
		pservice->reg_codec = NULL;
		spin_unlock_irqrestore(&pservice->data_lock, flags);
		reg_copy_from_hw(reg, data->dec_dev.hwregs, reg_len);
		irq_reg = DEC_INTERRUPT_REGISTER;
		if (HEVC_ID != data->hw_info->hw_id)
			irq_reg = DEC_VPU_INTERRUPT_REGISTER;
		break;
	}
	case VPU_PP: {
		spin_lock_irqsave(&pservice->data_lock, flags);
		pservice->reg_pproc = NULL;
		spin_unlock_irqrestore(&pservice->data_lock, flags);
		reg_copy_from_hw(reg, data->dec_dev.hwregs,
				 REG_NUM_9190_PP);
		break;
	}
	case VPU_DEC_PP: {
		u32 *dst = (u32 *)data->dec_dev.hwregs;

		spin_lock_irqsave(&pservice->data_lock, flags);
		pservice->reg_codec = NULL;
		pservice->reg_pproc = NULL;
		spin_unlock_irqrestore(&pservice->data_lock, flags);
		reg_copy_from_hw(reg, data->dec_dev.hwregs,
				 REG_NUM_9190_DEC_PP);
		/* disable pp pipeline */
		dst[VPU_REG_EN_PP] &= ~(0x1<<4);

		break;
	}
	default: {
		pr_err("copy reg from hw with unknown type %d\n", reg->type);
		break;
	}
	}

	vcodec_exit_mode(data);

	if (irq_reg != -1)
		reg->reg[irq_reg] = pservice->irq_status;

	atomic_sub(1, &reg->session->task_running);
	atomic_sub(1, &pservice->total_running);
	wake_up(&reg->session->wait);
}

static void reg_copy_to_hw(struct vpu_subdev_data *data,
			   struct vpu_reg *reg)
{
	int i;
	u32 *src = (u32 *)&reg->reg[0];
	struct vpu_service_info *pservice = data->pservice;

	atomic_add(1, &pservice->total_running);
	atomic_add(1, &reg->session->task_running);

	if (pservice->auto_freq) {
		vpu_update_load(pservice);
		vpu_set_pm_state(pservice);
	}

	vcodec_enter_mode(data);
#if defined(CONFIG_VCODEC_MMU)
	if (data->mmu_dev) {
		rockchip_iovmm_invalidate_tlb(data->dev);
	}
#endif
	switch (reg->type) {
	case VPU_ENC: {
		int enc_count = data->hw_info->enc_reg_num;
		u32 *dst = (u32 *)data->enc_dev.hwregs;
		u32 *dst_dec = (u32 *)data->dec_dev.hwregs;

		pservice->reg_codec = reg;

		dst_dec[VPU_REG_AXI_CTL_REG] &= ~VPU_AXI_DECODER;
		dst[VPU_REG_EN_ENC] = src[VPU_REG_EN_ENC] & 0x30;

		for (i = 0; i < VPU_REG_EN_ENC; i++)
			dst[i] = src[i];

		for (i = VPU_REG_EN_ENC + 1; i < enc_count; i++)
			dst[i] = src[i];

		VEPU_CLEAR_CACHE(dst);

		dst[VPU_REG_EN_ENC]   = src[VPU_REG_EN_ENC];

		pservice->enc_start = ktime_to_ms(ktime_get());
		break;
	}
	case VPU_DEC: {
		u32 *dst = (u32 *)data->dec_dev.hwregs;

		pservice->reg_codec = reg;
		if (data->hw_info->hw_id != HEVC_ID) {
			/* axi signals selected for decoder */
			src[VPU_REG_AXI_CTL_REG] |= VPU_AXI_DECODER;
			for (i = REG_NUM_9190_DEC_END - 1;
			     i >= REG_NUM_9190_DEC_START; i--) {
				if (VPU_REG_EN_DEC != i)
					dst[i] = src[i];
			}
			VDPU_CLEAR_CACHE(dst);
		} else {
			src[HEVC_CABAC_ERR_EN] = HEVC_CABAC_ERR_EN_BIT;
			for (i = REG_NUM_HEVC_DEC - 1; i > HEVC_REG_EN_DEC; i--)
				dst[i] = src[i];
			HEVC_CLEAR_CACHE(dst);
		}

		if (data->hw_info->hw_id != HEVC_ID)
			dst[VPU_REG_EN_DEC] = src[VPU_REG_EN_DEC] | (1<<4);
		else
			dst[HEVC_REG_EN_DEC] = src[HEVC_REG_EN_DEC] | HEVC_AUTO_GATING_EN;

		pservice->dec_start = ktime_to_ms(ktime_get());
		break;
	}
	case VPU_PP: {
		u32 *dst = (u32 *)data->dec_dev.hwregs;

		pservice->reg_pproc = reg;
		for (i = 0; i < VPU_REG_EN_PP; i++)
			dst[i] = src[i];

		dst[VPU_REG_EN_PP] = src[VPU_REG_EN_PP];

		pservice->pp_start = ktime_to_ms(ktime_get());

		break;
	}
	case VPU_DEC_PP: {
		u32 *dst = (u32 *)data->dec_dev.hwregs;

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

		pservice->dec_start = ktime_to_ms(ktime_get());

		break;
	}
	default: {
		pr_err("error: unsupport session type %d", reg->type);
		atomic_sub(1, &pservice->total_running);
		atomic_sub(1, &reg->session->task_running);
		break;
	}
	}
}

static void try_set_reg(struct vpu_subdev_data *data)
{
	/* first get reg from reg list */
	struct vpu_service_info *pservice = data->pservice;

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
			if (NULL == pservice->reg_codec) {
				if (NULL == pservice->reg_pproc) {
					can_set = 1;
				} else if (VCODEC_RUNNING_MODE_VPU == reg->data->mode) {
					can_set = 1;
					if (pservice->auto_freq &&
					    pservice->vpu_update_load)
						can_set = 0;
				}
			}
			break;
		case VPU_PP:
			if (NULL == pservice->reg_pproc) {
				if (NULL == pservice->reg_codec) {
					can_set = 1;
				} else if (VPU_DEC == pservice->reg_codec->type &&
					   VCODEC_RUNNING_MODE_VPU == pservice->reg_codec->data->mode) {
					can_set = 1;
					/* can't change freq when vpu is working */
					if (pservice->auto_freq &&
					    pservice->vpu_update_load)
						can_set = 0;
				}
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
			reg_copy_to_hw(reg->data, reg);
		}
	}
}

static int return_reg(struct vpu_subdev_data *data,
		      struct vpu_reg *reg, u32 __user *dst)
{
	int ret = 0;
	int reg_len = 0;

	switch (reg->type) {
	case VPU_ENC:
		if (copy_to_user(dst, &reg->reg[0],
				 data->hw_info->enc_io_size))
			ret = -EFAULT;
		break;
	case VPU_DEC:
		reg_len = data->hw_info->hw_id == HEVC_ID ?
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
	reg_deinit(data, reg);
	return ret;
}

static long vpu_service_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{
	struct vpu_subdev_data *data;
	struct vpu_service_info *pservice;
	struct vpu_session *session;
	atomic_t *tsk_running;
	int ret = 0;

	data = container_of(filp->f_dentry->d_inode->i_cdev,
			    struct vpu_subdev_data, cdev);
	pservice = data->pservice;
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
		reg = reg_init(data, session,
			       (void __user *)req.req, req.size);
		if (NULL == reg) {
			ret = -EFAULT;
		} else {
			mutex_lock(&pservice->lock);
			try_set_reg(data);
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
					vpu_reset(data);
					pr_err("done\n");
				}
				vpu_service_session_clear(data, session);
				mutex_unlock(&pservice->lock);
				return ret;
			}
		}
		mutex_lock(&pservice->lock);
		reg = list_entry(session->done.next, struct vpu_reg,
				 session_link);
		return_reg(data, reg, (u32 __user *)req.req);
		mutex_unlock(&pservice->lock);
		break;
	}
	case VPU_IOC_PROBE_IOMMU_STATUS: {
		int iommu_enable = 0;

#if defined(CONFIG_VCODEC_MMU)
		iommu_enable = data->mmu_dev ? 1 : 0;
#endif
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

static int vpu_service_check_hw(struct vpu_subdev_data *data,
				unsigned long hw_addr)
{
	int ret = -EINVAL, i = 0;
	u32 *tmp = (u32 *)ioremap_nocache(hw_addr, 0x4);
	u32 enc_id = *tmp;

	enc_id = (enc_id >> 16) & 0xFFFF;
	dev_dbg(data->dev, "checking hw id %x\n", enc_id);
	data->hw_info = NULL;
	for (i = 0; i < ARRAY_SIZE(vpu_hw_set); i++) {
		if (enc_id == vpu_hw_set[i].hw_id) {
			data->hw_info = &vpu_hw_set[i];
			ret = 0;
			break;
		}
	}
	iounmap((void *)tmp);
	return ret;
}

static int vpu_service_open(struct inode *inode, struct file *filp)
{
	struct vpu_subdev_data *data;
	struct vpu_service_info *pservice;
	struct vpu_session *session;

	data = container_of(inode->i_cdev, struct vpu_subdev_data, cdev);
	pservice = data->pservice;
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
	struct vpu_subdev_data *data;
	struct vpu_service_info *pservice;
	int task_running;
	struct vpu_session *session;

	data = container_of(inode->i_cdev, struct vpu_subdev_data, cdev);
	pservice = data->pservice;
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
	vpu_service_session_clear(data, session);
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
static void get_hw_info(struct vpu_subdev_data *data);

#ifdef CONFIG_VCODEC_MMU
static struct device *get_device_by_compatible(const char *compt)
{
	struct device_node *dn = NULL;
	struct platform_device *pd = NULL;
	struct device *ret = NULL;

	dn = of_find_compatible_node(NULL, NULL, compt);
	if (!dn) {
		pr_err("can't find device node %s \r\n", compt);
		return NULL;
	}

	pd = of_find_device_by_node(dn);
	if (!pd) {
		pr_err("can't find platform device in device node %s\n", compt);
		return  NULL;
	}
	ret = &pd->dev;

	return ret;
}

#ifdef CONFIG_IOMMU_API
static inline void platform_set_sysmmu(struct device *iommu,
				       struct device *dev)
{
	dev->archdata.iommu = iommu;
}
#else
static inline void platform_set_sysmmu(struct device *iommu,
				       struct device *dev)
{
}
#endif

int vcodec_sysmmu_fault_handler(struct device *dev,
				enum rockchip_iommu_inttype itype,
				unsigned long pgtable_base,
				unsigned long fault_addr, unsigned int status)
{
	struct platform_device *pdev;
	struct vpu_subdev_data *data;
	struct vpu_service_info *pservice;
	struct device *subdev = NULL;
	unsigned long flags;

	pr_info("%s in\n", __func__);

	pr_err("page fault addr: 0x%lx, page table base: 0x%lx, status=%d\n",
	       fault_addr, pgtable_base, status);

	if (strstr(dev_name(dev), VPU_MMU_DEVICE))
		subdev = get_device_by_compatible(VPU_COMPATIBLE_NAME);
	else if (strstr(dev_name(dev), HEVC_MMU_DEVICE))
		subdev = get_device_by_compatible(HEVC_COMPATIBLE_NAME);

	if (!subdev)
		return 0;

	pdev = container_of(subdev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	pservice = data->pservice;

	spin_lock_irqsave(&pservice->data_lock, flags);
	if (pservice->reg_codec) {
		struct vcodec_mem_region *mem, *n;
		struct vpu_device *tdev;
		unsigned int count = 0;
		int i = 0;

		list_for_each_entry_safe(mem, n,
					 &pservice->reg_codec->mem_region_list,
					 reg_lnk) {
			pr_info("reg[%02u] mem region [%02d] 0x%lx 0x%lx\n",
				mem->reg_idx, i, mem->iova, mem->len);
			i++;
		}

		/* dump full regs info */
		pr_info("dump full regs info\n");
		if (VPU_ENC == pservice->reg_codec->type)
			tdev = &data->enc_dev;
		else
			tdev = &data->dec_dev;

		count = tdev->iosize >> 2;
		for (i = 0; i < count; i++)
			pr_info("reg[%03u] = %08x\n", i,
				readl(tdev->hwregs + i));
	}
	spin_unlock_irqrestore(&pservice->data_lock, flags);

	pr_info("%s: done\n", __func__);
	return 0;
}
#endif

static int vcodec_subdev_probe(struct platform_device *pdev,
			       struct vpu_service_info *pservice)
{
	int ret = 0;
	u32 offset = 0;
	u32 ioaddr = 0;
	struct device *dev = &pdev->dev;
	void __iomem *regs = NULL;
	struct device_node *np = pdev->dev.of_node;
	char *prop = (char *)dev_name(dev);
	struct vpu_subdev_data *data = NULL;
	struct vpu_device *edev, *ddev;
	struct VPU_HW_INFO_E *hwinfo;
#if defined(CONFIG_VCODEC_MMU)
	u32 iommu_en = 0;
	char name[40];

	of_property_read_u32(np, "rockchip,iommu-enabled", &iommu_en);
#endif

	data = devm_kzalloc(dev, sizeof(struct vpu_subdev_data),
			    GFP_KERNEL);

	dev_dbg(dev, "probe device\n");

	data->pservice = pservice;
	data->dev = dev;
	of_property_read_string(np, "devname", (const char **)&prop);

	if (strstr(prop, "vpu_service"))
		data->mode = VCODEC_RUNNING_MODE_VPU;
	else if (strstr(prop, "hevc_service"))
		data->mode = VCODEC_RUNNING_MODE_HEVC;
	else
		data->mode = VCODEC_RUNNING_MODE_NONE;

	data->regs = pservice->reg_base;
	regs = pservice->reg_base;
	ioaddr = pservice->ioaddr;

	clear_bit(MMU_ACTIVATED, &data->state);
	vcodec_enter_mode(data);

	offset = ioaddr;
	if (VCODEC_RUNNING_MODE_VPU == data->mode)
		offset += VPU_HW_ID_OFFSET;
	else if (VCODEC_RUNNING_MODE_HEVC == data->mode)
		offset += HEVC_HW_ID_OFFSET;
	ret = vpu_service_check_hw(data, offset);
	if (ret < 0) {
		dev_err(dev, "hw info check failed\n");
		goto err;
	}

	/* define regs address. */
	ddev = &data->dec_dev;
	edev = &data->enc_dev;
	hwinfo = data->hw_info;

	ddev->iobaseaddr = ioaddr + hwinfo->dec_offset;
	ddev->iosize = hwinfo->dec_io_size;
	ddev->hwregs = (u32 *)((u8 *)regs + hwinfo->dec_offset);
	data->reg_size = ddev->iosize;

	if (data->mode == VCODEC_RUNNING_MODE_VPU) {
		edev->iobaseaddr = ioaddr + hwinfo->enc_offset;
		edev->iosize = hwinfo->enc_io_size;
		edev->hwregs = (u32 *)((u8 *)regs+hwinfo->enc_offset);
		data->reg_size = data->reg_size > edev->iosize ?
					data->reg_size : edev->iosize;

		data->irq_enc = platform_get_irq_byname(pdev, "irq_enc");
		if (data->irq_enc < 0) {
			dev_err(dev, "cannot find IRQ encoder\n");
			ret = -ENXIO;
			goto err;
		}

		ret = devm_request_threaded_irq(dev,
						data->irq_enc,
						vepu_irq, vepu_isr, IRQF_SHARED,
						dev_name(dev),
						(void *)data);
		if (ret) {
			dev_err(dev, "can't request vepu irq %d\n",
				data->irq_enc);
			goto err;
		}
	}

	data->irq_dec = platform_get_irq_byname(pdev, "irq_dec");
	if (data->irq_dec < 0) {
		dev_err(dev, "cannot find IRQ decoder\n");
		ret = -ENXIO;
		goto err;
	}

	/* get the IRQ line */
	ret = devm_request_threaded_irq(dev, data->irq_dec,
					vdpu_irq, vdpu_isr, IRQF_SHARED,
					dev_name(dev),
					(void *)data);
	if (ret) {
		dev_err(dev, "can't request vdpu irq %d\n",
			data->irq_dec);
		goto err;
	}

	atomic_set(&data->dec_dev.irq_count_codec, 0);
	atomic_set(&data->dec_dev.irq_count_pp, 0);
	atomic_set(&data->enc_dev.irq_count_codec, 0);
	atomic_set(&data->enc_dev.irq_count_pp, 0);
#if defined(CONFIG_VCODEC_MMU)
	if (iommu_en) {
		vcodec_enter_mode(data);
		if (data->mode == VCODEC_RUNNING_MODE_HEVC)
			sprintf(name, HEVC_IOMMU_COMPATIBLE_NAME);
		else
			sprintf(name, VPU_IOMMU_COMPATIBLE_NAME);

		data->mmu_dev = get_device_by_compatible(name);
		if (data->mmu_dev)
			platform_set_sysmmu(data->mmu_dev, dev);

		rockchip_iovmm_set_fault_handler(dev,
						 vcodec_sysmmu_fault_handler);
	}
#endif

	/* create device */
	ret = alloc_chrdev_region(&data->dev_t, 0, 1, prop);
	if (ret) {
		dev_err(dev, "alloc dev_t failed\n");
		goto err;
	}

	cdev_init(&data->cdev, &vpu_service_fops);

	data->cdev.owner = THIS_MODULE;
	data->cdev.ops = &vpu_service_fops;

	ret = cdev_add(&data->cdev, data->dev_t, 1);

	if (ret) {
		dev_err(dev, "add dev_t failed\n");
		goto err;
	}

	data->cls = class_create(THIS_MODULE, prop);

	if (IS_ERR(data->cls)) {
		ret = PTR_ERR(data->cls);
		dev_err(dev, "class_create err:%d\n", ret);
		goto err;
	}

	data->child_dev = device_create(data->cls, dev,
					    data->dev_t, NULL,
					    prop);

	get_hw_info(data);
	platform_set_drvdata(pdev, data);

	INIT_LIST_HEAD(&data->lnk_service);
	list_add_tail(&data->lnk_service, &pservice->subdev_list);

#ifdef CONFIG_DEBUG_FS
	data->debugfs_dir =
		vcodec_debugfs_dir((char *)dev_name(dev), parent);
	if (data->debugfs_dir == NULL)
		pr_err("create debugfs dir %s failed\n", dev_name(dev));

	data->debugfs_file_regs =
		debugfs_create_file("regs", 0664,
				    data->debugfs_dir, data,
				    &debug_vcodec_fops);
#endif

	return 0;

err:
	if (data->irq_enc > 0)
		free_irq(data->irq_enc, (void *)data);
	if (data->irq_dec > 0)
		free_irq(data->irq_dec, (void *)data);

	if (data->child_dev) {
		device_destroy(data->cls, data->dev_t);
		cdev_del(&data->cdev);
		unregister_chrdev_region(data->dev_t, 1);
	}

	if (data->cls)
		class_destroy(data->cls);

	return ret;
}

static void vcodec_subdev_remove(struct vpu_subdev_data *data)
{
	device_destroy(data->cls, data->dev_t);
	class_destroy(data->cls);
	cdev_del(&data->cdev);
	unregister_chrdev_region(data->dev_t, 1);

	free_irq(data->irq_enc, (void *)&data);
	free_irq(data->irq_dec, (void *)&data);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(data->debugfs_file_regs);
	debugfs_remove(data->debugfs_dir);
#endif
}

static void vcodec_init_drvdata(struct vpu_service_info *pservice)
{
	pservice->dev_id = VCODEC_DEVICE_ID_COMBO;
	pservice->curr_mode = VCODEC_RUNNING_MODE_NONE;

	wake_lock_init(&pservice->wake_lock, WAKE_LOCK_SUSPEND, "vpu");
	INIT_LIST_HEAD(&pservice->waiting);
	INIT_LIST_HEAD(&pservice->running);
	INIT_LIST_HEAD(&pservice->done);
	INIT_LIST_HEAD(&pservice->session);
	INIT_LIST_HEAD(&pservice->subdev_list);
	mutex_init(&pservice->lock);
	spin_lock_init(&pservice->irq_lock);
	spin_lock_init(&pservice->data_lock);
	pservice->reg_codec	= NULL;
	pservice->reg_pproc	= NULL;
	atomic_set(&pservice->total_running, 0);
	pservice->enabled = false;
	pservice->auto_freq = VPU_DVFS_ENABLE;

	INIT_DELAYED_WORK(&pservice->power_off_work, vpu_power_off_work);

	setup_timer(&pservice->timer, vpu_timer,
		    (unsigned long)pservice);

#if defined(CONFIG_VCODEC_MMU)
	pservice->ion_client = rockchip_ion_client_create("vpu");
	if (IS_ERR(pservice->ion_client))
		dev_err(pservice->dev, "failed to create ion client for vcodec");
	else
		dev_dbg(pservice->dev, "vcodec ion client create success!\n");
#endif
}

static int vcodec_probe(struct platform_device *pdev)
{
	int i = 0;
	int ret = 0;
	struct resource *res = NULL;
	struct device *dev = &pdev->dev;
	void __iomem *regs = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct vpu_service_info *pservice = NULL;

	pservice = devm_kzalloc(dev, sizeof(struct vpu_service_info),
				GFP_KERNEL);

	pservice->dev = dev;
	dev_info(dev, "probe device.\n");
	of_property_read_u32(np, "subcnt", &pservice->subcnt);

	vcodec_init_drvdata(pservice);

#ifdef CONFIG_PLATFORM_DEVICE_PM
	pservice->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pservice->pm_platdata)) {
		dev_err(dev, "Error during device state pm init\n");
		return -ENOMEM;
	}
	ret = device_state_pm_set_class(dev,
					pservice->pm_platdata->pm_user_name);

	for (i = 0; i < VPU_NUM_PM_STATES; i++) {
		pservice->pm_states[i] = device_state_pm_get_state_handler(dev,
				vpu_pm_state_name[i]);
		if (!pservice->pm_states[i]) {
			dev_err(dev, "unable to get pm state handle\n");
			return -EINVAL;
		}
		dev_dbg(dev, "pm_states[%d].name = %s\n",
			i, pservice->pm_states[i]->name);
	}

#endif
	mutex_lock(&pservice->lock);
	vpu_service_power_on(pservice);
	mutex_unlock(&pservice->lock);

	mdelay(1);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(dev, "Error when get platform resource\n");
		goto err;
	}

	res->flags &= ~IORESOURCE_CACHEABLE;

	regs = devm_ioremap_resource(pservice->dev, res);
	if (IS_ERR(regs)) {
		ret = PTR_ERR(regs);
		goto err;
	}
	pservice->reg_base = regs;
	pservice->ioaddr = res->start;

	for (i = 0; i < pservice->subcnt; i++) {
		struct device_node *sub_np;
		struct platform_device *sub_pdev;

		sub_np = of_parse_phandle(np, "rockchip,sub", i);
		sub_pdev = of_find_device_by_node(sub_np);
		if (NULL == sub_pdev) continue;

		vcodec_subdev_probe(sub_pdev, pservice);
	}

	platform_set_drvdata(pdev, pservice);

	vpu_service_power_off(pservice);

	dev_info(dev, "init success.\n");

	return 0;

err:
	dev_err(dev, "init failed.\n");
	vpu_service_power_off(pservice);
	wake_lock_destroy(&pservice->wake_lock);

	if (res)
		devm_release_mem_region(&pdev->dev, res->start,
					resource_size(res));

	return ret;
}

static int vcodec_remove(struct platform_device *pdev)
{
	struct vpu_service_info *pservice = platform_get_drvdata(pdev);
	struct resource *res;
	struct vpu_subdev_data *data, *n;

	list_for_each_entry_safe(data, n, &pservice->subdev_list, lnk_service) {
		vcodec_subdev_remove(data);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	devm_release_mem_region(&pdev->dev, res->start, resource_size(res));
	wake_lock_destroy(&pservice->wake_lock);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id vcodec_service_dt_ids[] = {
	{.compatible = "rockchip,vpu_combo",},
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

static void get_hw_info(struct vpu_subdev_data *data)
{
	struct vpu_service_info *pservice = data->pservice;
	struct vpuhwdecconfig_t *dec = &pservice->dec_config;
	struct vpuhwencconfig_t *enc = &pservice->enc_config;

	if (data->mode == VCODEC_RUNNING_MODE_VPU) {
		u32 configreg   = data->dec_dev.hwregs[VPU_DEC_HWCFG0];

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

		configreg = data->enc_dev.hwregs[VPU_ENC_HWCFG];
		enc->maxencodedwidth = configreg & ((1 << 11) - 1);
		enc->h264enabled = (configreg >> 27) & 1;
		enc->mpeg4enabled = (configreg >> 26) & 1;
		enc->jpegenabled = (configreg >> 25) & 1;
		enc->vsenabled = (configreg >> 24) & 1;
		enc->rgbenabled = (configreg >> 28) & 1;
		enc->reg_size = data->reg_size;
		enc->reserv[0] = 0;
		enc->reserv[1] = 0;

		pservice->bug_dec_addr = false;
	} else {
		dec->maxdecpicwidth = 1920;
	}
}

static irqreturn_t vdpu_irq(int irq, void *dev_id)
{
	struct vpu_subdev_data *data = (struct vpu_subdev_data *)dev_id;
	struct vpu_service_info *pservice = data->pservice;
	struct vpu_device *dev = &data->dec_dev;
	u32 raw_status;
	u32 irq_status;
	u32 vpu_status;
	int val;
	unsigned long flags;
	irqreturn_t ret_irq = IRQ_NONE;

	spin_lock_irqsave(&pservice->irq_lock, flags);
	if (HEVC_ID == data->hw_info->hw_id) {
		irq_status = readl(dev->hwregs + DEC_INTERRUPT_REGISTER);
		raw_status = irq_status;
		if (irq_status & DEC_INTERRUPT_BIT) {
			pr_debug("dec_isr dec %x\n", irq_status);

			val = HEVC_DEC_STR_ERROR_BIT |
			      HEVC_DEC_BUS_ERROR_BIT |
			      HEVC_DEC_TIMEOUT_BIT;
			if (irq_status & val)
				vpu_reset_iommu(data);

			/* clear dec IRQ */
			writel(0, dev->hwregs + DEC_INTERRUPT_REGISTER);
			atomic_add(1, &dev->irq_count_codec);
			pservice->irq_status = raw_status;
			ret_irq = IRQ_WAKE_THREAD;
		}
	} else {
		irq_status = readl(dev->hwregs + DEC_VPU_INTERRUPT_REGISTER);
		raw_status = irq_status;

		if (irq_status & DEC_VPU_INTERRUPT_BIT) {
			pr_debug("dec_isr dec %x\n", irq_status);
			vpu_status = readl(dev->hwregs + VPU_REG_EN_DEC);

			if (vpu_status & VPU_REG_DEC_WORK_BIT)
				writel(0, dev->hwregs + VPU_REG_EN_DEC);

			/* clear dec IRQ */
			writel(0, dev->hwregs + DEC_VPU_INTERRUPT_REGISTER);
			atomic_add(1, &dev->irq_count_codec);
			pservice->irq_status = raw_status;
			ret_irq = IRQ_WAKE_THREAD;
		}

		irq_status = readl(dev->hwregs + PP_INTERRUPT_REGISTER);
		if (irq_status & PP_INTERRUPT_BIT) {
			pr_debug("vdpu_isr pp  %x\n", irq_status);
			/* clear pp IRQ */
			val = irq_status & (~PP_INTERRUPT_BIT);
			writel(val, dev->hwregs + PP_INTERRUPT_REGISTER);
			atomic_add(1, &dev->irq_count_pp);
			pservice->irq_status = raw_status;
			ret_irq = IRQ_WAKE_THREAD;
		}
	}

	spin_unlock_irqrestore(&pservice->irq_lock, flags);

	return ret_irq;
}

static irqreturn_t vdpu_isr(int irq, void *dev_id)
{
	struct vpu_subdev_data *data = (struct vpu_subdev_data *)dev_id;
	struct vpu_service_info *pservice = data->pservice;
	struct vpu_device *dev = &data->dec_dev;
	u32 time_delta = 0;
	u64 now;

	mutex_lock(&pservice->lock);
	if (atomic_read(&dev->irq_count_codec)) {
		atomic_sub(1, &dev->irq_count_codec);
		if (NULL == pservice->reg_codec)
			pr_err("dec isr with no task waiting\n");
		else
			reg_from_run_to_done(data, pservice->reg_codec);

		now = ktime_to_ms(ktime_get());
		time_delta = (now - pservice->dec_start);
		pservice->vpu_codec_time += time_delta;
		/* pr_info("time_delta: %d\n", time_delta); */
		pservice->vpu_time_per_frame = time_delta;
	}

	if (atomic_read(&dev->irq_count_pp)) {
		atomic_sub(1, &dev->irq_count_pp);
		if (NULL == pservice->reg_pproc)
			pr_err("pp isr with no task waiting\n");
		else
			reg_from_run_to_done(data, pservice->reg_pproc);

		now = ktime_to_ms(ktime_get());
		time_delta = (now - pservice->pp_start);
		pservice->vpu_pp_time += time_delta;
		/* pr_info("time_delta pp: %d\n", time_delta); */
		pservice->vpu_time_per_frame = time_delta;
	}
	try_set_reg(data);
	mutex_unlock(&pservice->lock);
	return IRQ_HANDLED;
}

static irqreturn_t vepu_irq(int irq, void *dev_id)
{
	struct vpu_subdev_data *data = (struct vpu_subdev_data *)dev_id;
	struct vpu_service_info *pservice = data->pservice;
	struct vpu_device *dev = &data->enc_dev;
	u32 irq_status;
	int val;
	unsigned long flags;
	irqreturn_t ret_irq = IRQ_NONE;

	spin_lock_irqsave(&pservice->irq_lock, flags);
	irq_status = readl(dev->hwregs + ENC_INTERRUPT_REGISTER);

	pr_debug("vepu_irq irq status %x\n", irq_status);

	if (likely(irq_status & ENC_INTERRUPT_BIT)) {
		/* clear enc IRQ */
		val = irq_status & (~ENC_INTERRUPT_BIT);
		writel(val, dev->hwregs + ENC_INTERRUPT_REGISTER);
		pservice->irq_status = irq_status;
		atomic_add(1, &dev->irq_count_codec);
		ret_irq = IRQ_WAKE_THREAD;
	}

	spin_unlock_irqrestore(&pservice->irq_lock, flags);

	return ret_irq;
}

static irqreturn_t vepu_isr(int irq, void *dev_id)
{
	struct vpu_subdev_data *data = (struct vpu_subdev_data *)dev_id;
	struct vpu_service_info *pservice = data->pservice;
	struct vpu_device *dev = &data->enc_dev;
	u64 now;
	u32 time_delta = 0;

	mutex_lock(&pservice->lock);

	if (atomic_read(&dev->irq_count_codec)) {
		atomic_sub(1, &dev->irq_count_codec);
		if (NULL == pservice->reg_codec)
			pr_err("enc isr with no task waiting\n");
		else
			reg_from_run_to_done(data, pservice->reg_codec);

		now = ktime_to_ms(ktime_get());
		time_delta = now - pservice->enc_start;
		/* pr_info("time_delta enc: %d\n", time_delta); */
		pservice->vpu_codec_time += time_delta;
		pservice->vpu_time_per_frame = time_delta;

	}
	try_set_reg(data);
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
	struct vpu_subdev_data *data = s->private;
	struct vpu_service_info *pservice = data->pservice;
	unsigned int i, n;
	struct vpu_reg *reg, *reg_tmp;
	struct vpu_session *session, *session_tmp;

	mutex_lock(&pservice->lock);
	vpu_service_power_on(pservice);
	if (data->hw_info->hw_id != HEVC_ID) {
		seq_puts(s, "\nENC Registers:\n");
		n = data->enc_dev.iosize >> 2;
		for (i = 0; i < n; i++) {
			seq_printf(s, "\tswreg%d = %08X\n", i,
				   readl(data->enc_dev.hwregs + i));
		}
	}
	seq_puts(s, "\nDEC Registers:\n");
	n = data->dec_dev.iosize >> 2;
	for (i = 0; i < n; i++)
		seq_printf(s, "\tswreg%d = %08X\n", i,
			   readl(data->dec_dev.hwregs + i));

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
