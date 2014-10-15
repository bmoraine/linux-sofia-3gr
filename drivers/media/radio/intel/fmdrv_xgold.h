/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
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

#ifndef _FMDRV_XGOLD_H_
#define _FMDRV_XGOLD_H_

#include <linux/kthread.h>
#include <kfifo.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/idi/idi_device_pm.h>
#include <media/v4l2-device.h>
#include <aud_app_fmr_hld_rx.h>

#define RDS_MIN_FREE	4
#define RDS_PI_MODE	RDS_NORMAL_PI
#define NUM_INT_LINES	5

enum fmdrv_event {
	XGOLD_EVT_RADIO_READY,
	XGOLD_EVT_TUNE_SUCC,
	XGOLD_EVT_TUNE_FAIL,
	XGOLD_EVT_SEEK_COMPLETE,
};

struct xgold_fmdev_idi_pdev {
	struct idi_peripheral_device *fmr_dev;
	struct completion rx_complete;
	struct completion tx_complete;
};

struct xgold_fmrx_rds {
	atomic_t rds_mode;	/* 1 if RDS is ON, 0 if RDS is turned Off */

	wait_queue_head_t read_queue;
	u32 wr_idx;
	u32 rd_idx;
	atomic_t is_full;	/* 1 if buffer is full, else 0 */

	void *rds_rpt_buf;	/* rds buffer */
	u32 array_len;		/* length of the buffer */
	atomic_t rpt_cnt;	/* Available reports in the buffer */
};

/* PM State handlers */
struct fmdev_pm_state {
	struct device_state_pm_state *en_handler;
	struct device_state_pm_state *dis_handler;
	struct device_state_pm_state *en_int_clk_handler;
};

struct xgold_fmdev {
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	atomic_t dev_open_count;

	struct task_struct *hld_task;
	wait_queue_head_t wait_msg;

	struct mutex lock; /* v4l2 interface lock */

	/* wait queue for blocking event read */
	wait_queue_head_t event_queue;

	/* type of event occcured */
	enum fmdrv_event event_type;

	spinlock_t rds_buff_lock; /* To protect access to RDS buffer */
	struct xgold_fmrx_rds fmrx_rds;

	/* Transportation layer stuff */
	struct xgold_fmdev_idi_pdev idi_fmdev;

	/* hw info */
	struct resource *hw_ram;
	struct resource *xg_ram;
	struct resource *reg_space;
	void __iomem *ctrl_io;
	u32 irqs[NUM_INT_LINES];

	/*FM Radio device PM state handlers */
	struct fmdev_pm_state pm_state;
};

#endif /*_FMDRV_XGOLD_H_ */

