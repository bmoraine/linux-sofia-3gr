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

#define pr_fmt(fmt) "rga: " fmt
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/wakelock.h>

#include <linux/xgold_noc.h>

#if defined(CONFIG_ION_XGOLD) || defined(CONFIG_ION_ROCKCHIP)
#include "../../../staging/android/ion/ion.h"
#include <linux/rockchip_ion.h>
#endif

#if defined(CONFIG_MOBILEVISOR_VDRIVER_PIPE) && defined(CONFIG_SECURE_PLAYBACK)
#define RGA_SECURE_ACCESS
#endif

#if defined(RGA_SECURE_ACCESS)
#include <sofia/mv_svc_hypercalls.h>
#include <sofia/vrga_fe.h>
#endif /* RGA_SECURE_ACCESS */

#include "rga.h"
#include "rga_reg_info.h"
#include "rga_mmu_info.h"
#include "rga_api.h"

#define RGA_TEST_CASE 0

#define RGA_TEST 0
#define RGA_TEST_TIME 0
#define RGA_TEST_FLUSH_TIME 0
#define RGA_INFO_BUS_ERROR 1

#define PRE_SCALE_BUF_SIZE  (1024*1024*4)
#define USE_CMA_FOR_PRE_SCALE

#define RGA_POWER_OFF_DELAY	(4*HZ)	/* 4s */
#define RGA_TIMEOUT_DELAY	(1*HZ)	/* 1s */
#define RGA_FENCE_TIMEOUT_DELAY	msecs_to_jiffies(400) 	/* 400ms */

#define RGA_MAJOR		255

#define RGA_RESET_TIMEOUT	1000

/* Driver information */
#define DRIVER_DESC		"RGA Device Driver"
#define DRIVER_NAME		"rga"

#define RGA_VERSION   "1.001"

#define RGA_GET_SRC_FENCE_ERROR     -190
#define RGA_GET_DST_FENCE_ERROR     -191
#define RGA_GET_FENCE_PT_ERROR      -192
#define RGA_DMA_BUF_COPY_ERROR      -201
#define RGA_COPY_FROM_USER_ERROR    -202
#define RGA_UNKONW_CMD_ERROR        -203
#define RGA_CHECK_PARAM_ERROR       -204
#define RGA_GEN_REG_ERROR           -205
#define RGA_TIMEOUT_NUM_ERROR       -206

ktime_t rga_start;
ktime_t rga_end;
ktime_t rga_end_0;

uint32_t rga_fence_create_num = 0;
uint32_t rga_fence_interrupt_num = 0;
uint32_t rga_fence_timeout_num = 0;
uint32_t rga_int_f_num = 0;
uint32_t rga_int_b_num = 0;

struct rga_session rga_session_global;

struct rga_drvdata {
	struct miscdevice miscdev;
	struct device *dev;
	void *rga_base;
	int irq;

	struct delayed_work power_off_work;
	void (*rga_irq_callback)(int rga_retval);
	struct wake_lock wake_lock;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#else
	struct clk *pd_rga;
	struct clk *aclk_rga;
	struct clk *hclk_rga;
#endif

#if defined(CONFIG_ION_XGOLD) || defined(CONFIG_ION_ROCKCHIP)
	struct ion_client *ion_client;
	struct ion_handle *handle;
#endif
};

static struct rga_drvdata *drvdata;
struct rga_service_info rga_service;
struct rga_mmu_buf_t rga_mmu_buf;

static int rga_blit_async(struct rga_session *session, struct rga_req *req);
static void rga_del_running_list(void);
static void rga_del_running_list_timeout(struct work_struct *work);
static void rga_try_set_reg(void);
static int rga_create_sysfs(struct device *dev);

#if RGA_TEST
static void rga_test_0(void);
#endif

/* Logging */
#define RGA_DEBUG 0
#if RGA_DEBUG
#define DBG(format, args...) pr_info("%s: " format, DRIVER_NAME, ## args)
#define ERR(format, args...) pr_info("%s: " format, DRIVER_NAME, ## args)
#define WARNING(format, args...) pr_info("%s: " format, DRIVER_NAME, ## args)
#define INFO(format, args...) pr_info("%s: " format, DRIVER_NAME, ## args)
#else
#define DBG(format, args...)
#define ERR(format, args...)
#define WARNING(format, args...)
#define INFO(format, args...)
#endif

#if RGA_TEST
static void print_info(struct rga_req *req)
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
#endif

static inline void rga_write(u32 b, u32 r)
{
	__raw_writel(b, drvdata->rga_base + r);
}

static inline u32 rga_read(u32 r)
{
	return __raw_readl(drvdata->rga_base + r);
}

static void rga_soft_reset(void)
{
	u32 i;
	u32 reg;

#if !defined(RGA_SECURE_ACCESS)
	rga_write(1, RGA_SYS_CTRL);
#else
	mv_svc_reg_write(RGA_BASE + RGA_SYS_CTRL, 0x1, BIT(1));
#endif

	for (i = 0; i < RGA_RESET_TIMEOUT; i++) {
#if !defined(RGA_SECURE_ACCESS)
		reg = rga_read(RGA_SYS_CTRL) & 1;
#else
		mv_svc_reg_read(RGA_BASE + RGA_SYS_CTRL, &reg, BIT(1));
		reg &= 1;
#endif
		if (reg == 0)
			break;
		udelay(1);
	}

	if (i == RGA_RESET_TIMEOUT)
		ERR("soft reset timeout.\n");
}

static void rga_dump(void)
{
	int running;
	struct rga_reg *reg, *reg_tmp;
	struct rga_session *session, *session_tmp;

	running = atomic_read(&rga_service.total_running);
	pr_info("rga total_running %d\n", running);

	list_for_each_entry_safe(session,
				 session_tmp, &rga_service.session,
				 list_session) {
		pr_info("session pid %d:\n", session->pid);
		running = atomic_read(&session->task_running);
		pr_info("task_running %d\n", running);
		list_for_each_entry_safe(reg, reg_tmp,
					 &session->waiting, session_link) {
			pr_info("waiting register set 0x%.8x\n",
				(unsigned int)reg);
		}
		list_for_each_entry_safe(reg, reg_tmp,
					 &session->running, session_link) {
			pr_info("running register set 0x%.8x\n",
				(unsigned int)reg);
		}
	}
}

static inline void rga_queue_power_off_work(void)
{
	queue_delayed_work(system_nrt_wq, &drvdata->power_off_work,
			   RGA_POWER_OFF_DELAY);
}

/* Caller must hold rga_service.lock */
static void rga_power_on(void)
{
	static ktime_t last;
	ktime_t now = ktime_get();

	if (ktime_to_ns(ktime_sub(now, last)) > NSEC_PER_SEC) {
		cancel_delayed_work_sync(&drvdata->power_off_work);
		rga_queue_power_off_work();
		last = now;
	}
	if (rga_service.enable)
		return;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	device_state_pm_set_state_by_name(drvdata->dev,
					  drvdata->pm_platdata->
					  pm_state_D0_name);
#else
	clk_prepare_enable(struct clk *clk)(drvdata->aclk_rga);
	clk_prepare_enable(drvdata->hclk_rga);
	/*clk_prepare_enable(drvdata->pd_rga); */
#endif

	xgold_noc_qos_set("RGA");
	wake_lock(&drvdata->wake_lock);
	rga_service.enable = true;
}

/* Caller must hold rga_service.lock */
static void rga_power_off(void)
{
	if (!rga_service.enable)
		return;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	device_state_pm_set_state_by_name(drvdata->dev,
					  drvdata->pm_platdata->
					  pm_state_D3_name);
#else
	clk_disable_unprepare(drvdata->aclk_rga);
	clk_disable_unprepare(drvdata->hclk_rga);
#endif

	wake_unlock(&drvdata->wake_lock);
	rga_service.enable = false;
}

static void rga_power_off_work(struct work_struct *work)
{
	int total_running;

	if (mutex_trylock(&rga_service.lock)) {
		total_running = atomic_read(&rga_service.total_running);

	        if (total_running == 0) {
			/* power off only when no task running */
			rga_power_off();
			mutex_unlock(&rga_service.lock);
			return;
		} else {
			rga_dump();
			mutex_unlock(&rga_service.lock);
		}
	}

	/* Come back later if the device is busy... */
	rga_queue_power_off_work();
	return;
}

static int rga_flush(struct rga_session *session, unsigned long arg)
{
	int ret = 0;
	int ret_timeout;

#if RGA_TEST_FLUSH_TIME
	ktime_t start;
	ktime_t end;

	start = ktime_get();
#endif

	ret_timeout = wait_event_timeout(session->wait,
					 atomic_read(&session->done),
					 RGA_TIMEOUT_DELAY);

#if RGA_TEST_FLUSH_TIME
	end = ktime_get();
	end = ktime_sub(end, start);
	pr_info("one flush wait time %d\n", (int)ktime_to_us(end));
#endif

	return ret;
}

static int rga_get_result(struct rga_session *session, unsigned long arg)
{
	int ret = 0;
	int num_done;

	num_done = atomic_read(&session->num_done);

	if (unlikely(copy_to_user((void __user *)arg,
				   &num_done, sizeof(int)))) {
		pr_info("copy_to_user failed\n");
		ret = -EFAULT;
	}
	return ret;
}

static int rga_check_param(const struct rga_req *req)
{
	if (!((req->render_mode == color_fill_mode) ||
	      (req->render_mode == line_point_drawing_mode))) {
		if (unlikely((req->src.act_w <= 0) ||
			     (req->src.act_w > 8191) ||
			     (req->src.act_h <= 0) ||
			     (req->src.act_h > 8191))) {
			pr_info
			    ("invalid src resolution act_w = %d, act_h = %d\n",
			     req->src.act_w, req->src.act_h);
			return -EINVAL;
		}
	}

	if (!((req->render_mode == color_fill_mode) ||
	      (req->render_mode == line_point_drawing_mode))) {
		if (unlikely((req->src.vir_w <= 0) ||
			     (req->src.vir_w > 8191) ||
			     (req->src.vir_h <= 0) ||
			     (req->src.vir_h > 8191))) {
			pr_info
			    ("invalid src resolution vir_w = %d, vir_h = %d\n",
			     req->src.vir_w, req->src.vir_h);
			return -EINVAL;
		}
	}
	if (unlikely((req->dst.act_w <= 0) ||
		     (req->dst.act_w > 2048) ||
		     (req->dst.act_h <= 0) || (req->dst.act_h > 2048))) {
		pr_info("invalid dst resolution act_w = %d, act_h = %d\n",
			req->dst.act_w, req->dst.act_h);
		return -EINVAL;
	}

	if (unlikely((req->dst.vir_w <= 0) ||
		     (req->dst.vir_w > 4096) ||
		     (req->dst.vir_h <= 0) || (req->dst.vir_h > 2048))) {
		pr_info("invalid dstresolution vir_w = %d, vir_h = %d\n",
			req->dst.vir_w, req->dst.vir_h);
		return -EINVAL;
	}

	if (unlikely(req->src.vir_w < req->src.act_w)) {
		pr_info("invalid src_vir_w act_w = %d, vir_w = %d\n",
			req->src.act_w, req->src.vir_w);
		return -EINVAL;
	}

	if (unlikely(req->dst.vir_w < req->dst.act_w)) {
		if (req->rotate_mode != 1) {
			pr_info("invalid dst_vir_w act_h = %d, vir_h = %d\n",
				req->dst.act_w, req->dst.vir_w);
			return -EINVAL;
		}
	}

	return 0;
}

static void rga_copy_reg(struct rga_reg *reg, uint32_t offset)
{
	uint32_t i;
	uint32_t *cmd_buf;
	uint32_t *reg_p;

	if (atomic_read(&reg->session->task_running) != 0)
		pr_info("task_running is no zero\n");

	atomic_add(1, &rga_service.cmd_num);
	atomic_add(1, &reg->session->task_running);

	cmd_buf = (uint32_t *)rga_service.cmd_buff + offset * 32;
	reg_p = (uint32_t *)reg->cmd_reg;

	for (i = 0; i < 32; i++)
		cmd_buf[i] = reg_p[i];
}

static struct rga_reg *rga_reg_init(struct rga_session *session,
				    struct rga_req *req)
{
	int32_t ret;
	int32_t len = sizeof(struct rga_reg);
	struct rga_reg *reg = kzalloc(len, GFP_KERNEL);

	if (NULL == reg)
		return NULL;

	reg->session = session;
	INIT_LIST_HEAD(&reg->session_link);
	INIT_LIST_HEAD(&reg->status_link);

	reg->MMU_base = NULL;

	if (req->mmu_info.mmu_en) {
		ret = rga_set_mmu_info(reg, req);
		if (ret < 0) {
			pr_info("%s, [%d] set mmu info error\n",
				__func__, __LINE__);
			if (reg != NULL)
				kfree(reg);

			return NULL;
		}
	}

	if (RGA_gen_reg_info(req, (uint8_t *)reg->cmd_reg) == -1) {
		pr_info("gen reg info error\n");
		if (reg != NULL)
			kfree(reg);
		return NULL;
	}

	mutex_lock(&rga_service.lock);
	list_add_tail(&reg->status_link, &rga_service.waiting);
	list_add_tail(&reg->session_link, &session->waiting);
	reg->fence =
		rga_service.src_fence_flag > 0 ?
		sync_fence_fdget(rga_service.src_fence_fd) : NULL;
	reg->dst_fence =
		rga_service.dst_fence_flag > 0 ?
		sync_fence_fdget(rga_service.dst_fence_fd) : NULL;
	if (reg->dst_fence != NULL)
		sync_fence_put(reg->dst_fence);

	mutex_unlock(&rga_service.lock);

	return reg;
}

static struct rga_reg *rga_reg_init_2(struct rga_session *session,
				      struct rga_req *req0,
				      struct rga_req *req1)
{
	int32_t ret;
	int32_t len = sizeof(struct rga_reg);
	struct rga_reg *reg0, *reg1;

	reg0 = NULL;
	reg1 = NULL;

	do {
		reg0 = kzalloc(len, GFP_KERNEL);
		if (NULL == reg0)
			break;

		reg1 = kzalloc(len, GFP_KERNEL);
		if (NULL == reg1)
			break;

		reg0->session = session;
		INIT_LIST_HEAD(&reg0->session_link);
		INIT_LIST_HEAD(&reg0->status_link);

		reg1->session = session;
		INIT_LIST_HEAD(&reg1->session_link);
		INIT_LIST_HEAD(&reg1->status_link);

		req0->mmu_info.mmu_flag &= (~(1 << 10));

		if (req0->mmu_info.mmu_en) {
			ret = rga_set_mmu_info(reg0, req0);
			if (ret < 0)
				break;
		}

		RGA_gen_reg_info(req0, (uint8_t *)reg0->cmd_reg);

		req1->mmu_info.mmu_flag &= (~(1 << 8));

		if (req1->mmu_info.mmu_en) {
			ret = rga_set_mmu_info(reg1, req1);
			if (ret < 0) {
				pr_info("%s, [%d] set mmu info error\n",
					__func__, __LINE__);
				break;
			}
		}

		RGA_gen_reg_info(req1, (uint8_t *)reg1->cmd_reg);

		mutex_lock(&rga_service.lock);
		list_add_tail(&reg0->status_link, &rga_service.waiting);
		list_add_tail(&reg0->session_link, &session->waiting);
		list_add_tail(&reg1->status_link, &rga_service.waiting);
		list_add_tail(&reg1->session_link, &session->waiting);
		reg0->fence =
			rga_service.src_fence_flag > 0 ?
			sync_fence_fdget(rga_service.src_fence_fd) : NULL;
		reg0->dst_fence = NULL;
		reg1->fence = NULL;
		reg1->dst_fence =
		    rga_service.dst_fence_flag > 0 ?
		    sync_fence_fdget(rga_service.dst_fence_fd) : NULL;

		if (reg1->dst_fence != NULL)
			sync_fence_put(reg1->dst_fence);

		mutex_unlock(&rga_service.lock);

		return reg1;
	} while (0);

	if (reg0 != NULL)
		kfree(reg0);

	if (reg1 != NULL)
		kfree(reg1);

	return NULL;
}

/* Caller must hold rga_service.lock */
static void rga_reg_deinit(struct rga_reg *reg)
{
	list_del_init(&reg->session_link);
	list_del_init(&reg->status_link);
	kfree(reg);
}

/* Caller must hold rga_service.lock */
static void rga_reg_from_wait_to_run(struct rga_reg *reg)
{
	list_del_init(&reg->status_link);
	list_add_tail(&reg->status_link, &rga_service.running);

	list_del_init(&reg->session_link);
	list_add_tail(&reg->session_link, &reg->session->running);
}

/* Caller must hold rga_service.lock */
static void rga_service_session_clear(struct rga_session *session)
{
	struct rga_reg *reg, *n;

	list_for_each_entry_safe(reg, n, &session->waiting, session_link) {
		rga_reg_deinit(reg);
	}

	list_for_each_entry_safe(reg, n, &session->running, session_link) {
		rga_reg_deinit(reg);
	}
}

/* Caller must hold rga_service.lock */
static void rga_try_set_reg(void)
{
	struct rga_reg *reg;
	int err;
#if defined(RGA_SECURE_ACCESS)
	struct vrga_secvm_cmd vcmd;
	int vrga_ret;
#endif


	if (list_empty(&rga_service.running)) {
		if (!list_empty(&rga_service.waiting)) {
			/* RGA is idle */
			reg =
			    list_entry(rga_service.waiting.next,
				       struct rga_reg, status_link);

			rga_power_on();
			udelay(1);
			rga_copy_reg(reg, 0);
			rga_reg_from_wait_to_run(reg);

#ifdef CONFIG_X86
			clflush_cache_range(&rga_service.cmd_buff[0], 30 * 4);
#else
			dmac_flush_range(&rga_service.cmd_buff[0],
					 &rga_service.cmd_buff[28]);
			outer_flush_range(virt_to_phys
					  (&rga_service.cmd_buff[0]),
					  virt_to_phys(&rga_service.
						       cmd_buff[28]));
#endif

/* secvm change */
#if !defined(RGA_SECURE_ACCESS)
			rga_soft_reset();

			rga_write(0x0, RGA_SYS_CTRL);
			rga_write(0, RGA_MMU_CTRL);

			/* CMD buff */
			rga_write(virt_to_phys(rga_service.cmd_buff),
				  RGA_CMD_ADDR);
#endif

#if RGA_TEST
			{
				uint32_t i;
				uint32_t *p;

				p = rga_service.cmd_buff;
				pr_info("CMD_REG\n");
				for (i = 0; i < 7; i++) {
					pr_info("%.8x %.8x %.8x %.8x\n",
						p[0 + i * 4], p[1 + i * 4],
						p[2 + i * 4], p[3 + i * 4]);
				}
				pr_info("%.8x %.8x\n", p[0 + i * 4],
					p[1 + i * 4]);
			}
#endif

/* secvm change */
#if !defined(RGA_SECURE_ACCESS)
			/* master mode */
			rga_write((0x1 << 2) | (0x1 << 3), RGA_SYS_CTRL);

			/* All CMD finish int */
			rga_write(rga_read(RGA_INT) |
				  (0x1 << 10) | (0x1 << 8), RGA_INT);
#endif

#if RGA_TEST_TIME
			rga_start = ktime_get();
#endif
			if (reg->fence != NULL) {
				err = sync_fence_wait(reg->fence, 10000);
				sync_fence_put(reg->fence);
				if (err < 0)
					pr_info("error wait for src fence\n");
			}
			atomic_set(&reg->session->done, 0);
			if (atomic_read(&rga_service.delay_work_already_queue)) {
				cancel_delayed_work(&rga_service.fence_delayed_work);
				atomic_set(&rga_service.delay_work_already_queue, 0);
			}

			rga_int_f_num++;
			rga_start = ktime_get();
			atomic_set(&rga_service.interrupt_flag, 1);
 #if !defined(RGA_SECURE_ACCESS)
			rga_write(1, RGA_CMD_CTRL);
#else
			/* call into secvm to update rga registers */
			memset(&vcmd, 0, sizeof(vcmd));

			vcmd.payload[0] = VRGA_VTYPE_REG;
			vcmd.payload[1] = VRGA_VOP_REG_WRITE;
			vcmd.payload[2] = 0;
			vcmd.payload[3] = 0;
			vcmd.payload[4] = virt_to_phys(rga_service.cmd_buff);

			/* execute command */
			vrga_ret = vrga_call(drvdata->dev, &vcmd);
			if (vrga_ret == 0)
				dev_err(drvdata->dev, "error rga registers writing");

#endif
			queue_delayed_work(rga_service.fence_workqueue,
					&rga_service.fence_delayed_work,
					RGA_FENCE_TIMEOUT_DELAY);
			atomic_set(&rga_service.delay_work_already_queue, 1);

/*RGA_TEST
			{
				uint32_t i;

				pr_info("CMD_READ_BACK_REG\n");
				for (i = 0; i < 7; i++)
					pr_info("%.8x %.8x %.8x %.8x\n",
						rga_read(0x100 + i * 16 + 0),
						rga_read(0x100 + i * 16 + 4),
						rga_read(0x100 + i * 16 + 8),
						rga_read(0x100 + i * 16 + 12)
					    );
				pr_info("%.8x %.8x\n",
					rga_read(0x100 + i * 16 + 0),
					rga_read(0x100 + i * 16 + 4));
			}
*/
			rga_end = ktime_get();
		}
	}
}

/* Caller must hold rga_service.lock */
static void rga_del_running_list(void)
{
	struct rga_reg *reg;

	while (!list_empty(&rga_service.running)) {
		reg =
		    list_entry(rga_service.running.next,
			       struct rga_reg, status_link);

		if (reg->MMU_len != 0) {
			if (rga_mmu_buf.back +
				reg->MMU_len > 2*rga_mmu_buf.size)
				rga_mmu_buf.back =
				reg->MMU_len + rga_mmu_buf.size;
			else
				rga_mmu_buf.back += reg->MMU_len;
		}
		atomic_sub(1, &reg->session->task_running);
		atomic_sub(1, &rga_service.total_running);

		/*atomic_set(&reg->session->queue_work_done, 1);
		wake_up(&reg->session->queue_work_wait);*/

		if (atomic_read(&rga_service.delay_work_already_queue)) {
			cancel_delayed_work(&rga_service.fence_delayed_work);
			atomic_set(&rga_service.delay_work_already_queue, 0);
		}

		if (list_empty(&reg->session->waiting)) {
			atomic_set(&reg->session->done, 1);
			wake_up(&reg->session->wait);
		}
		if (reg->dst_fence != NULL) {
			sw_sync_timeline_inc(rga_service.timeline, 1);
			rga_fence_interrupt_num++;
		}

		rga_reg_deinit(reg);
	}
}

/* Caller must hold rga_service.lock */
static void rga_del_running_list_timeout(struct work_struct *work)
{
	struct rga_reg *reg;

	mutex_lock(&rga_service.lock);
	/*pr_info("Warning RGA process task timeout\n");*/
	rga_service.timeout_num++;
	while (!list_empty(&rga_service.running)) {
		reg =
		    list_entry(rga_service.running.next,
			       struct rga_reg, status_link);

		if (reg->MMU_len != 0) {
			if (rga_mmu_buf.back +
				reg->MMU_len > 2 * rga_mmu_buf.size)
				rga_mmu_buf.back =
					reg->MMU_len + rga_mmu_buf.size;
			else
				rga_mmu_buf.back += reg->MMU_len;
		}

		atomic_sub(1, &reg->session->task_running);
		atomic_sub(1, &rga_service.total_running);

		rga_end = ktime_sub(rga_end, rga_start);
		rga_end_0 = ktime_sub(rga_end_0, rga_start);

		/* whether is a fake Timeout */
		//if (atomic_read(&rga_service.interrupt_flag))
		    pr_info("RGA STATUS %.8x INT %.8x F %d B %d end %d end_0 %d\n", rga_read(0xc), rga_read(0x10),
		    rga_int_f_num, rga_int_b_num, (int)ktime_to_us(rga_end), (int)ktime_to_us(rga_end_0));

		rga_soft_reset();

		/*atomic_set(&reg->session->queue_work_done, 1);
		wake_up(&reg->session->queue_work_wait);*/

		if (list_empty(&reg->session->waiting)) {
			atomic_set(&reg->session->done, 1);
			wake_up(&reg->session->wait);
		}
		if (reg->dst_fence != NULL) {
			sw_sync_timeline_inc(rga_service.timeline, 1);
			rga_fence_timeout_num++;
		}

		rga_reg_deinit(reg);
	}

	/*while (!list_empty(&rga_service.waiting)) {
		reg =
		    list_entry(rga_service.waiting.next,
			       struct rga_reg, status_link);

		if (reg->MMU_len != 0) {
			if (rga_mmu_buf.back +
				reg->MMU_len > 2 * rga_mmu_buf.size)
				rga_mmu_buf.back =
					reg->MMU_len + rga_mmu_buf.size;
			else
				rga_mmu_buf.back += reg->MMU_len;
		}

		if (reg->dst_fence != NULL) {
			sw_sync_timeline_inc(rga_service.timeline, 1);
			rga_fence_timeout_num++;
		}

		list_del_init(&reg->status_link);
		list_del_init(&reg->session_link);
		if (list_empty(&reg->session->waiting)) {
			atomic_set(&reg->session->done, 1);
			wake_up(&reg->session->wait);
		}
		rga_reg_deinit(reg);
	}*/
	if (atomic_read(&rga_service.delay_work_already_queue)) {
		cancel_delayed_work(&rga_service.fence_delayed_work);
		atomic_set(&rga_service.delay_work_already_queue, 0);
	}
	atomic_set(&rga_service.interrupt_timeout_flag, 1);
	rga_try_set_reg();
	mutex_unlock(&rga_service.lock);
}

static int rga_convert_dma_buf(struct rga_req *req)
{
	struct ion_handle *hdl;
	ion_phys_addr_t phy_addr;
	size_t len;
	int ret;
	short src_fd, dst_fd;
	uint32_t src_offset, dst_offset;

	req->sg_src = NULL;
	req->sg_dst = NULL;

	src_fd = req->line_draw_info.color & 0xffff;
	dst_fd = (req->line_draw_info.color >> 16) & 0xffff;
	src_offset = req->line_draw_info.flag;
	dst_offset = req->line_draw_info.line_width;
	if (src_fd > 0) {
		hdl =
		    ion_import_dma_buf(drvdata->ion_client, src_fd);
		if (IS_ERR(hdl)) {
			ret = PTR_ERR(hdl);
			pr_info("RGA2 ERROR SRC ion buf handle\n");
			return ret;
		}
		if ((req->mmu_info.mmu_flag >> 8) & 1) {
			req->sg_src = ion_sg_table(drvdata->ion_client, hdl);
			req->src.yrgb_addr = req->src.uv_addr;
			req->src.uv_addr =
			    req->src.yrgb_addr +
			    (req->src.vir_w * req->src.vir_h);
			req->src.v_addr =
			    req->src.uv_addr +
			    ((req->src.vir_w * req->src.vir_h) >> 2);
		} else {
			ion_phys(drvdata->ion_client, hdl, &phy_addr, &len);
			req->src.yrgb_addr = phy_addr + src_offset;
			req->src.uv_addr =
			    req->src.yrgb_addr +
			    (req->src.vir_w * req->src.vir_h);
			req->src.v_addr =
			    req->src.uv_addr +
			    ((req->src.vir_w * req->src.vir_h) >> 2);
		}
		ion_free(drvdata->ion_client, hdl);
	}

	if (dst_fd > 0) {
		hdl = ion_import_dma_buf(drvdata->ion_client,
					 dst_fd);
		if (IS_ERR(hdl)) {
			ret = PTR_ERR(hdl);
			pr_info("RGA2 ERROR DST ion buf handle\n");
			return ret;
		}
		if ((req->mmu_info.mmu_flag >> 10) & 1) {
			req->sg_dst = ion_sg_table(drvdata->ion_client, hdl);
			req->dst.yrgb_addr = req->dst.uv_addr;
			req->dst.uv_addr =
			    req->dst.yrgb_addr +
			    (req->dst.vir_w * req->dst.vir_h);
			req->dst.v_addr =
			    req->dst.uv_addr +
			    ((req->dst.vir_w * req->dst.vir_h) >> 2);
		} else {
			ion_phys(drvdata->ion_client, hdl, &phy_addr, &len);
			req->dst.yrgb_addr = phy_addr + dst_offset;
			req->dst.uv_addr =
			    req->dst.yrgb_addr +
			    (req->dst.vir_w * req->dst.vir_h);
			req->dst.v_addr =
			    req->dst.uv_addr +
			    ((req->dst.vir_w * req->dst.vir_h) >> 2);
		}
		ion_free(drvdata->ion_client, hdl);
	}
	return 0;
}

static int rga_blit(struct rga_session *session, struct rga_req *req)
{
	int ret = -1;
	int num = 0;
	/*int already_queue;*/
	struct rga_reg *reg;
	struct rga_req req2;

	uint32_t saw, sah, daw, dah;

	struct sync_fence *retire_fence;
	struct sync_pt *retire_sync_pt;

	saw = req->src.act_w;
	sah = req->src.act_h;
	daw = req->dst.act_w;
	dah = req->dst.act_h;

#if RGA_TEST
	print_info(req);
#endif

	rga_service.src_fence_flag =
		(req->line_draw_info.start_point.y) & 0x1;
	rga_service.src_fence_fd = -1;
	if (rga_service.src_fence_flag)
		rga_service.src_fence_fd =
		(req->line_draw_info.start_point.x) & 0xffff;
	rga_service.dst_fence_flag =
		(req->line_draw_info.end_point.y) & 0x1;
	rga_service.dst_fence = NULL;
	if (rga_service.dst_fence_flag) {
		rga_service.timeline_max++;
		rga_service.dst_fence_fd = get_unused_fd();
		if (rga_service.dst_fence_fd < 0) {
			pr_info("dst_fence_fd=%d\n",
			rga_service.dst_fence_fd);
			ret = RGA_GET_SRC_FENCE_ERROR;
			return ret;
		}
		retire_sync_pt =
			sw_sync_pt_create(rga_service.timeline,
				rga_service.timeline_max);
		if (!retire_sync_pt) {
			pr_info("get retire_sync_pt error\n");
			rga_service.timeline_max--;
			ret = RGA_GET_FENCE_PT_ERROR;
			return ret;
		}
		retire_fence =
			sync_fence_create("rga_ret_fence",
				retire_sync_pt);
		if (!retire_fence) {
			pr_info("get retire_fence error\n");
			sync_pt_free(retire_sync_pt);
			rga_service.timeline_max--;
			ret = RGA_GET_DST_FENCE_ERROR;
			return ret;
		}
		rga_service.dst_fence = retire_fence;
		sync_fence_install(retire_fence,
			rga_service.dst_fence_fd);
		rga_fence_create_num++;
	}

	if (rga_convert_dma_buf(req)) {
		pr_info("RGA : DMA buf copy error\n");
		return RGA_DMA_BUF_COPY_ERROR;
	}

	do {
		if ((req->render_mode == bitblt_mode) &&
		    (((saw >> 1) >= daw) || ((sah >> 1) >= dah))) {
			/* generate 2 cmd for pre scale */

			ret = rga_check_param(req);
			if (ret == -EINVAL) {
				ret = RGA_CHECK_PARAM_ERROR;
				pr_info("req 0 argument is inval\n");
				break;
			}

			ret = rga_gen_two_pro(req, &req2);
			if (ret == -EINVAL) {
				ret = RGA_GEN_REG_ERROR;
				break;
			}

			ret = rga_check_param(req);
			if (ret == -EINVAL) {
				ret = RGA_CHECK_PARAM_ERROR;
				pr_info("req 1 argument is inval\n");
				break;
			}

			ret = rga_check_param(&req2);
			if (ret == -EINVAL) {
				ret = RGA_CHECK_PARAM_ERROR;
				pr_info("req 2 argument is inval\n");
				break;
			}

			reg = rga_reg_init_2(session, req, &req2);
			if (reg == NULL) {
				ret = RGA_GEN_REG_ERROR;
				break;
			}
			num = 2;

		} else {
			/* check value if legal */
			ret = rga_check_param(req);
			if (ret == -EINVAL) {
				ret = RGA_CHECK_PARAM_ERROR;
				pr_info("req argument is inval\n");
				break;
			}

			reg = rga_reg_init(session, req);
			if (reg == NULL) {
				ret = RGA_GEN_REG_ERROR;
				break;
			}
			num = 1;
		}

		mutex_lock(&rga_service.lock);
		atomic_add(num, &rga_service.total_running);
		rga_try_set_reg();
		mutex_unlock(&rga_service.lock);

		return 0;
	} while (0);

	if (rga_service.dst_fence != NULL)
		sync_fence_put(rga_service.dst_fence);

	return ret;
}

static int rga_blit_async(struct rga_session *session, struct rga_req *req)
{
	int ret = -1;

#if RGA_TEST
	pr_info("*** rga_blit_async proc ***\n");
#endif

	atomic_set(&session->done, 0);
	ret = rga_blit(session, req);
	return ret;
}

static int rga_blit_sync(struct rga_session *session, struct rga_req *req)
{
	int ret = -1;
	int ret_timeout = 0;

#if RGA_TEST
	pr_info("*** rga_blit_sync proc ***\n");
#endif

	atomic_set(&session->done, 0);
	ret = rga_blit(session, req);
	if (ret < 0)
		return ret;

	/*flush_kthread_worker(&session->update_regs_worker);*/
	ret_timeout
	    = wait_event_timeout(session->wait,
				 atomic_read(&session->done),
				 RGA_TIMEOUT_DELAY);

#if RGA_TEST_TIME
	rga_end = ktime_get();
	rga_end = ktime_sub(rga_end, rga_start);
	pr_info("sync one cmd end time %d\n", (int)ktime_to_us(rga_end));
#endif

	return ret;
}

static long rga_ioctl(struct file *file, uint32_t cmd, unsigned long arg)
{
	struct rga_req req, *req_p;
	int ret = 0;
	struct rga_session *session;

	req_p = (struct rga_req *)arg;

	mutex_lock(&rga_service.mutex);

	session = (struct rga_session *)file->private_data;
	rga_service.dst_fence_fd = -1;

	if (NULL == session) {
		pr_info("%s [%d] rga thread session is null\n",
			__func__, __LINE__);
		mutex_unlock(&rga_service.mutex);
		return -EINVAL;
	}

	memset(&req, 0x0, sizeof(req));

	switch (cmd) {
	case RGA_BLIT_SYNC:
		if (unlikely(copy_from_user(&req,
					    (struct rga_req *)arg,
					    sizeof(struct rga_req)))) {
			ERR("copy_from_user failed\n");
			ret = RGA_DMA_BUF_COPY_ERROR;
			break;
		}
		ret = rga_blit_sync(session, &req);
		break;
	case RGA_BLIT_ASYNC:
		if (unlikely
		    (copy_from_user
		     (&req, (struct rga_req *)arg, sizeof(struct rga_req)))) {
			ERR("copy_from_user failed\n");
			ret = RGA_DMA_BUF_COPY_ERROR;
			break;
		}

		if ((atomic_read(&rga_service.total_running) >= 16))
			ret = rga_blit_sync(session, &req);
		else
			ret = rga_blit_async(session, &req);
		break;
	case RGA_FLUSH:
		ret = rga_flush(session, arg);
		break;
	case RGA_GET_RESULT:
		ret = rga_get_result(session, arg);
		break;
	case RGA_GET_VERSION:
		ret = copy_to_user((void *)arg,
				   RGA_VERSION, sizeof(RGA_VERSION));
		break;
	default:
		ERR("unknown ioctl cmd!\n");
		ret = RGA_UNKONW_CMD_ERROR;
		break;
	}

	copy_to_user
		(&req_p->line_draw_info.end_point.x,
	     &rga_service.dst_fence_fd, sizeof(short));
	copy_to_user
		(&req_p->line_draw_info.start_point.x,
	     &rga_service.timeout_num, sizeof(short));
	mutex_unlock(&rga_service.mutex);

	return ret;
}

/*
static void rga_update_regs_handler(struct kthread_work *work)
{
	mutex_lock(&rga_service.lock);
	rga_try_set_reg();
	mutex_unlock(&rga_service.lock);
}
*/
static int rga_open(struct inode *inode, struct file *file)
{
	int32_t len = sizeof(struct rga_session);
	struct rga_session *session =
	    kcalloc(len, 1, GFP_KERNEL);
	if (NULL == session)
		return -ENOMEM;

	session->pid = current->pid;

	INIT_LIST_HEAD(&session->waiting);
	INIT_LIST_HEAD(&session->running);
	INIT_LIST_HEAD(&session->list_session);
	init_waitqueue_head(&session->wait);
	/*init_waitqueue_head(&session->queue_work_wait);*/
	mutex_lock(&rga_service.lock);
	list_add_tail(&session->list_session, &rga_service.session);
	mutex_unlock(&rga_service.lock);
	atomic_set(&session->task_running, 0);
	atomic_set(&session->num_done, 0);
    /*
	init_kthread_worker(&session->update_regs_worker);
	session->update_regs_thread =
		kthread_run(kthread_worker_fn,
		&session->update_regs_worker,
		"rockchip-rga");
	if (IS_ERR(session->update_regs_thread)) {
		int err = PTR_ERR(session->update_regs_thread);

		session->update_regs_thread = NULL;
		pr_err("failed to run rga_update_regs thread\n");
		return err;
	}
	init_kthread_work(&session->update_regs_work,
		rga_update_regs_handler);
	*/
	file->private_data = (void *)session;

	return nonseekable_open(inode, file);
}

static int rga_release(struct inode *inode, struct file *file)
{
	int task_running;
	struct rga_session *session = (struct rga_session *)file->private_data;

	if (NULL == session)
		return -EINVAL;

	task_running = atomic_read(&session->task_running);

	if (task_running) {
		pr_err("session %d still has %d task run when closing\n",
		       session->pid, task_running);
		msleep(100);
	}

	wake_up(&session->wait);
	mutex_lock(&rga_service.lock);
	list_del(&session->list_session);
	rga_service_session_clear(session);
	kfree(session);
	mutex_unlock(&rga_service.lock);

	return 0;
}

static irqreturn_t rga_irq_thread(int irq, void *dev_id)
{
	mutex_lock(&rga_service.lock);
	if (atomic_read(&rga_service.interrupt_timeout_flag)) {
        pr_info("rga timeout double in\n");
		mutex_unlock(&rga_service.lock);
		return IRQ_HANDLED;
	}
	if (atomic_read(&rga_service.delay_work_already_queue)) {
		cancel_delayed_work(&rga_service.fence_delayed_work);
		atomic_set(&rga_service.delay_work_already_queue, 0);
	}
	if (rga_service.enable) {
		rga_del_running_list();
		rga_try_set_reg();
	}
	mutex_unlock(&rga_service.lock);

	return IRQ_HANDLED;
}

static irqreturn_t rga_irq(int irq, void *dev_id)
{
	/*clear INT */
#if !defined(RGA_SECURE_ACCESS)
	rga_write(rga_read(RGA_INT) | (0x1 << 6) |
		(0x1 << 7) | (0x1 << 4), RGA_INT);
#else
	mv_svc_reg_write(RGA_BASE + RGA_INT,
		(0x1 << 6) | (0x1 << 7) | (0x1 << 4), BIT(4)|BIT(6)|BIT(7));
#endif
	atomic_set(&rga_service.interrupt_flag, 0);
	atomic_set(&rga_service.interrupt_timeout_flag, 0);
	rga_int_b_num++;
	rga_end_0 = ktime_get();

	return IRQ_WAKE_THREAD;
}

static const struct file_operations rga_fops = {
	.owner = THIS_MODULE,
	.open = rga_open,
	.release = rga_release,
	.unlocked_ioctl = rga_ioctl,
};

struct miscdevice rga_dev = {
	.minor = RGA_MAJOR,
	.name = "rga",
	.fops = &rga_fops,
};

#if defined(CONFIG_OF)
static const struct of_device_id rk_rga_dt_ids[] = {
	{.compatible = "rockchip,rga",},
	{},
};
#endif

static int rga_drv_probe(struct platform_device *pdev)
{
	struct rga_drvdata *data;
	struct resource *res;
	int ret = 0;
#ifdef USE_CMA_FOR_PRE_SCALE
	ion_phys_addr_t phy_addr;
	size_t len;
	int i;
#endif

	mutex_init(&rga_service.lock);
	mutex_init(&rga_service.mutex);
	atomic_set(&rga_service.total_running, 0);
	rga_service.enable = false;
	rga_service.timeline =
				sw_sync_timeline_create("rga-rockchip");
	rga_service.timeline_max = 0;
	rga_service.timeout_num = 0;
	atomic_set(&rga_service.delay_work_already_queue, 0);

	data = devm_kzalloc(&pdev->dev, sizeof(struct rga_drvdata), GFP_KERNEL);
	if (!data) {
		ERR("failed to allocate driver data.\n");
		return -ENOMEM;
	}

	data->dev = &pdev->dev;

	INIT_DELAYED_WORK(&data->power_off_work, rga_power_off_work);
	INIT_DELAYED_WORK(&rga_service.fence_delayed_work,
		rga_del_running_list_timeout);
	rga_service.fence_workqueue =
		alloc_ordered_workqueue("rga_fence_delaywork", 0);
	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "rga");

#ifdef CONFIG_PLATFORM_DEVICE_PM
	data->pm_platdata = of_device_state_pm_setup(pdev->dev.of_node);
	if (IS_ERR(data->pm_platdata)) {
		dev_err(&pdev->dev, "Error during device state pm inigit t\n");
		return -ENOMEM;
	}
	ret = device_state_pm_set_class(&pdev->dev,
					data->pm_platdata->pm_user_name);
#else
	data->aclk_rga = devm_clk_get(&pdev->dev, "aclk_rga");
	data->hclk_rga = devm_clk_get(&pdev->dev, "hclk_rga");
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->rga_base = devm_ioremap_resource(&pdev->dev, res);
	if (!data->rga_base) {
		ERR("rga ioremap failed\n");
		ret = -ENOENT;
		goto err_ioremap;
	}

	/* get the IRQ */
	ret = platform_get_irq(pdev, 0);
	data->irq = ret;
	if (ret <= 0) {
		ERR("failed to get rga irq resource (%d).\n", data->irq);
		ret = data->irq;
		goto err_irq;
	}

	/* request the IRQ */
	ret = devm_request_threaded_irq(&pdev->dev, data->irq,
					rga_irq, rga_irq_thread,
					IRQF_DISABLED | IRQF_SHARED, "rga",
					data);
	if (ret) {
		ERR("rga request_irq failed (%d).\n", ret);
		goto err_irq;
	}

	platform_set_drvdata(pdev, data);
	drvdata = data;

#if defined(CONFIG_ION_ROCKCHIP)
	pr_info("create rockchip ion client for RGA\n");
	data->ion_client = rockchip_ion_client_create("rga");
	if (IS_ERR(data->ion_client)) {
		dev_err(&pdev->dev, "failed to create ion client for rga");
		return PTR_ERR(data->ion_client);
	}

#ifdef USE_CMA_FOR_PRE_SCALE
	data->handle = ion_alloc(data->ion_client, (size_t)PRE_SCALE_BUF_SIZE,
				0, ION_HEAP_TYPE_SECURE_MASK, 0);
	if (IS_ERR(data->handle)) {
		dev_err(&pdev->dev, "failed to ion_alloc:%ld\n",
				PTR_ERR(data->handle));
		return -ENOMEM;
	}

	/* prepare pre-scale buffer */
	ion_phys(data->ion_client, data->handle, &phy_addr, &len);
	for (i = 0; i < 1024; i++)
		rga_service.pre_scale_buf[i] = phy_addr + (i<<12);
#endif

#elif defined(CONFIG_ION_XGOLD)
	pr_info("create xgold ion client for RGA\n");
	data->ion_client = xgold_ion_client_create("rga");
	if (IS_ERR(data->ion_client)) {
		dev_err(&pdev->dev, "failed to create ion client for rga");
		return PTR_ERR(data->ion_client);
	}

#ifdef USE_CMA_FOR_PRE_SCALE
	data->handle = ion_alloc(data->ion_client, (size_t)RGA_MMU_BUF_SIZE,
				0, ION_HEAP_TYPE_SECURE_MASK, 0);
	if (IS_ERR(data->handle)) {
		dev_err(&pdev->dev, "failed to ion_alloc:%ld\n",
				PTR_ERR(data->handle));
		return -ENOMEM;
	}

	/* prepare pre-scale buffer */
	ion_phys(data->ion_client, data->handle, &phy_addr, &len);
	for (i = 0; i < 1024; i++)
		rga_service.pre_scale_buf[i] = phy_addr + (i<<12);
#endif
#endif

#if defined(RGA_SECURE_ACCESS)
	/*
	 * initlialize secure VM decoder: init vbpipe
	 */
	dev_info(&pdev->dev, "init SecureVM rga");

	/* indicator for isr, ioctl, etc. */
	ret = vrga_fe_init(&pdev->dev);

	/*
	 * during system boot the vbpipe may not yet be accessible
	 * therefore ignore an error and init the pipe
	 * the first time it is used
	 */
	if (ret != 0) {
		dev_warn(&pdev->dev, "vbpipe open is postponed");

		/* TODO: ignore and skip probing; open pipe later */
		ret = 0;
	}
#endif

	ret = misc_register(&rga_dev);
	if (ret) {
		ERR("cannot register miscdev (%d)\n", ret);
		goto err_misc_register;
	}
	rga_create_sysfs(rga_dev.this_device);

	pr_info("Driver loaded successfully\n");

	return 0;

err_misc_register:
	free_irq(data->irq, pdev);
err_irq:
	iounmap(data->rga_base);
err_ioremap:
	wake_lock_destroy(&data->wake_lock);

	return ret;
}

static int rga_drv_remove(struct platform_device *pdev)
{
	struct rga_drvdata *data = platform_get_drvdata(pdev);

	if (!data)
		return -EINVAL;

	wake_lock_destroy(&data->wake_lock);
	misc_deregister(&data->miscdev);
	free_irq(data->irq, &data->miscdev);
	iounmap((void __iomem *)(data->rga_base));

#ifdef CONFIG_PLATFORM_DEVICE_PM
	device_state_pm_set_state_by_name(drvdata->dev,
					  drvdata->pm_platdata->
					  pm_state_D3_name);
#else
	devm_clk_put(&pdev->dev, data->aclk_rga);
	devm_clk_put(&pdev->dev, data->hclk_rga);
#endif

#if defined(CONFIG_ION_XGOLD) || defined(CONFIG_ION_ROCKCHIP)
	if (data != NULL && data->ion_client) {
		if (data->handle)
			ion_free(data->ion_client, data->handle);
		ion_client_destroy(data->ion_client);
	}
#endif

#if defined(RGA_SECURE_ACCESS)
	vrga_fe_release(&pdev->dev);
#endif

	return 0;
}

static struct platform_driver rga_driver = {
	.probe = rga_drv_probe,
	.remove = rga_drv_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "rga",
		   .of_match_table = of_match_ptr(rk_rga_dt_ids),
		   },
};

static int __init rga_init(void)
{
	int ret;
	uint32_t *mmu_buf;
	uint32_t i;
	uint32_t *buf_p;

	/* init rga_mmu_buf */
	memset(&rga_mmu_buf, 0x0, sizeof(rga_mmu_buf));

	/* malloc pre scale mid buf mmu table */
	mmu_buf = kzalloc(1024 * 8, GFP_KERNEL);
	if (mmu_buf == NULL)
		return -1;

#ifndef USE_CMA_FOR_PRE_SCALE
	/* malloc 4 M buf */
	for (i = 0; i < 1024; i++) {
		buf_p = (uint32_t *)__get_free_page(GFP_KERNEL | __GFP_ZERO);
		if (buf_p == NULL) {
			pr_info("RGA init pre scale buf falied\n");
			goto free_mmu_buf;
		}

		mmu_buf[i] = virt_to_phys((void *)((uint32_t)buf_p));
	}
#endif

	rga_service.pre_scale_buf = (uint32_t *)mmu_buf;

	buf_p = kmalloc(1024*(512+32), GFP_KERNEL);
	if (buf_p == NULL)
		goto free_mmu_buf;

	rga_mmu_buf.buf_virtual = buf_p;
	i = (virt_to_phys((void *)((uint32_t)buf_p)));
	rga_mmu_buf.buf = (unsigned int *)i;
	rga_mmu_buf.front = 0;
	rga_mmu_buf.back = 128*1024;
	rga_mmu_buf.size = 128*1024;
	rga_mmu_buf.pages = kmalloc((16384)*sizeof(struct page *), GFP_KERNEL);
	if (rga_mmu_buf.pages == NULL)
		goto free_mmu_buf;

	ret = platform_driver_register(&rga_driver);

	if (ret != 0) {
		pr_info("Platform device register failed (%d).\n", ret);
		goto free_mmu_buf;
	}

	{
		rga_session_global.pid = 0x0000ffff;
		INIT_LIST_HEAD(&rga_session_global.waiting);
		INIT_LIST_HEAD(&rga_session_global.running);
		INIT_LIST_HEAD(&rga_session_global.list_session);

		INIT_LIST_HEAD(&rga_service.waiting);
		INIT_LIST_HEAD(&rga_service.running);
		INIT_LIST_HEAD(&rga_service.done);
		INIT_LIST_HEAD(&rga_service.session);

		init_waitqueue_head(&rga_session_global.wait);
		list_add_tail(&rga_session_global.list_session,
			      &rga_service.session);
		atomic_set(&rga_session_global.task_running, 0);
		atomic_set(&rga_session_global.num_done, 0);
		/*init_kthread_worker(&rga_session_global.update_regs_worker);
		rga_session_global.update_regs_thread =
				kthread_run(kthread_worker_fn,
				&rga_session_global.update_regs_worker,
				"rockchip-rga");
	if (IS_ERR(rga_session_global.update_regs_thread)) {
			int err =
				PTR_ERR(rga_session_global.update_regs_thread);
			rga_session_global.update_regs_thread = NULL;
			pr_err("failed to run rga_update_regs thread\n");
			return err;
		}
		init_kthread_work(&rga_session_global.update_regs_work,
					rga_update_regs_handler);*/
	}

#if RGA_TEST_CASE
	rga_test_0();
#endif

	INFO("Module initialized.\n");

	return 0;

free_mmu_buf:
#ifndef USE_CMA_FOR_PRE_SCALE
	for (i = 0; i < 1024; i++) {
		if ((uint32_t *)mmu_buf[i] != NULL)
			__free_page((void *)mmu_buf[i]);
	}
#endif

	kfree(mmu_buf);

	if (rga_mmu_buf.buf_virtual != NULL)
		kfree(rga_mmu_buf.buf_virtual);
	if (rga_mmu_buf.pages != NULL)
		kfree(rga_mmu_buf.pages);

	return -ENOMEM;

}

static void __exit rga_exit(void)
{
#ifndef USE_CMA_FOR_PRE_SCALE
	uint32_t i;
#endif

	rga_power_off();

#ifndef USE_CMA_FOR_PRE_SCALE
	if (rga_service.pre_scale_buf != NULL) {
		for (i = 0; i < 1024; i++) {
			if ((uint32_t *)rga_service.pre_scale_buf[i] != NULL)
				__free_page(
				(void *)rga_service.pre_scale_buf[i]);
		}
		kfree((uint8_t *)rga_service.pre_scale_buf);
	}
#endif

	if (rga_mmu_buf.buf_virtual != NULL)
		kfree(rga_mmu_buf.buf_virtual);

	if (rga_mmu_buf.pages != NULL)
		kfree(rga_mmu_buf.pages);

	platform_driver_unregister(&rga_driver);
}

#if RGA_TEST_CASE
unsigned int src_buf[1920 * 1080];
unsigned int dst_buf[1920 * 1080];
static void rga_test_0(void)
{
	struct rga_req req;
	struct rga_session session;
	unsigned int *src, *dst;
	uint32_t i, j;
	uint8_t *p;
	uint8_t t;
	uint32_t *dst0, *dst1, *dst2;

	session.pid = current->pid;
	INIT_LIST_HEAD(&session.waiting);
	INIT_LIST_HEAD(&session.running);
	INIT_LIST_HEAD(&session.list_session);
	init_waitqueue_head(&session.wait);
	/* no need to protect */
	list_add_tail(&session.list_session, &rga_service.session);
	atomic_set(&session.task_running, 0);
	atomic_set(&session.num_done, 0);

	memset(&req, 0, sizeof(struct rga_req));
	src = src_buf;
	dst = dst_buf;

	dst0 = &dst_buf[0];

	i = 0;
	j = 0;

	pr_info("**********************************\n");
	pr_info("************ RGA_TEST ************\n");
	pr_info("********************************\n\n");

	req.src.act_w = 1280;
	req.src.act_h = 800;

	req.src.vir_w = 1280;
	req.src.vir_h = 800;
	req.src.yrgb_addr = 0;
	req.src.uv_addr = (uint32_t)virt_to_phys(src);
	req.src.v_addr = 0;
	req.src.format = RGA_FORMAT_RGBA_8888;

	req.dst.act_w = 1280;
	req.dst.act_h = 800;

	req.dst.vir_w = 1280;
	req.dst.vir_h = 800;
	req.dst.x_offset = 0;
	req.dst.y_offset = 0;

	dst = dst0;

	req.dst.yrgb_addr = 0;
	req.dst.uv_addr = ((uint32_t)virt_to_phys(dst));

	req.dst.format = RGA_FORMAT_RGBA_8888;

	req.clip.xmin = 0;
	req.clip.xmax = 1279;
	req.clip.ymin = 0;
	req.clip.ymax = 799;

	req.render_mode = color_fill_mode;
	req.fg_color = 0x80ffffff;

	req.rotate_mode = 0;
	/*
	   req.scale_mode = 2;

	   req.alpha_rop_flag = 0;
	   req.alpha_rop_mode = 0x19;
	   req.PD_mode = 3;

	   req.sina = 65536;
	   req.cosa = 0;

	   req.mmu_info.mmu_flag = 0x21;
	   /req.mmu_info.mmu_en = 1; */

	rga_blit_sync(&session, &req);
}

#endif

static int show_rga_info(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	return snprintf(buf,
		PAGE_SIZE,
		"create %d interrupt %d timeout %d add %d\n",
			rga_fence_create_num,
			rga_fence_interrupt_num,
			rga_fence_timeout_num,
			rga_fence_timeout_num + rga_fence_interrupt_num);
}

static struct device_attribute rga_attrs[] = {
	__ATTR(rga_info, S_IRUGO, show_rga_info, NULL),
};

int rga_create_sysfs(struct device *dev)
{
	int r, t;

	for (t = 0; t < ARRAY_SIZE(rga_attrs); t++) {
		r = device_create_file(dev, &rga_attrs[t]);
		if (r) {
			dev_err(dev, "failed to create sysfs " "file\n");
			return r;
		}
	}

	return 0;
}

void rga_remove_sysfs(struct device *dev)
{
	int t;

	for (t = 0; t < ARRAY_SIZE(rga_attrs); t++)
		device_remove_file(dev,
				   &rga_attrs[t]);
}

module_init(rga_init);
module_exit(rga_exit);

/* Module information */
MODULE_DESCRIPTION("Driver for rga device");
MODULE_LICENSE("GPL");
