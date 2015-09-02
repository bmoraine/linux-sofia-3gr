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

/**
 * This file implements the interface between V4L2 subsystem and the FM Radio
 * driver. It registers with the V4L2 subsystem and implements the
 * standard/private IOCTLs/CTRLs. It also registers with the platform bus
 * (IDI in case of AG620).
 */

/* Standard linux interfaces */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/ioport.h>
#include <linux/types.h>

/* V4L2 interfaces */
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#ifdef CONFIG_IDI
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>
#endif

/* Frequency manager interfaces */
#ifdef CONFIG_IUI_FM_FMR
#include <linux/fm/iui_fm.h>
#include <linux/timer.h>
#endif

/* Local interfaces */
#include <fmdrv_v4l2.h>	/* User space common interfaces */

/* HW defines */
#define FMR_REG_RES_NAME		"register"
#define FMR_AGRAM_RES_NAME		"hwram"
#define FMR_XGRAM_RES_NAME		"ram"

#define XGOLD_FMDRV_VER			"1.00"
#define XGOLD_FMDRV_RADIO_VERSION	KERNEL_VERSION(0, 0, 1)
#define XGOLD_FMDRV_NAME		"ag6x0_fmr"
#define XGOLD_FMDRV_CARD_SHORT_NAME	"XGold FM Radio"
#define XGOLD_FMDRV_BUS_INFO		"IDI_IF"

#ifdef CONFIG_IUI_FM_FMR
#define TIME_TO_NOTIFY_FM	750 /* msec to notify FM */
#endif

/* RDS constants */
#define FMR_RDS_GRP_MAX		62
#define FMR_RDS_GRP_BLK_MAX	(FMR_RDS_GRP_MAX * 4)
#define FMR_RDS_TIMEOUT		2000
#define TIME_TO_DIS_RDS		3000 /* msecs to disable RDS reception */
#define FMR_RDS_GRP_LEN		12

/* Callbacks to register with HLD */
static struct fmrx_callbacks fmrx_cbs;

/* Timer for delayed FM notification */
#ifdef CONFIG_IUI_FM_FMR
static struct timer_list fm_notify_timer;
#endif

/* Timer for RDS unsubscribtion */
static struct timer_list fmr_rds_dis_timer;

/* XGold FM Radio Device structure */
static struct xgold_fmdev *xgold_fmdev;

#ifdef CONFIG_IDI
/* Order does matter */
static char *const xgold_fmr_irq_names[NUM_INT_LINES] = {
	"fmr_int",
	"fmrx_ev_oneshot",
	"fmrx_ev_tune",
	"fmtx_ev_calib",
	"fmtx_ev_tune",
};
#endif

#ifdef CONFIG_IUI_FM_FMR
/* FM mitigation callback */
enum iui_fm_mitigation_status fmr_fm_mitigation_cb(
	const enum iui_fm_macro_id macro_id,
	const struct iui_fm_mitigation *mitigation, const uint32_t sequence)
{
	int rc = 0;
	enum fmrx_sb_inj_side inj_side = FMR_FORCE_INVALID;
	enum iui_fm_mitigation_status mitigation_status =
		IUI_FM_MITIGATION_ERROR_INVALID_PARAM;

	if ((NULL == mitigation) || (IUI_FM_MACRO_ID_FMR != macro_id) ||
	    (IUI_FM_MITIGATION_TYPE_FMR != mitigation->type))
		return mitigation_status;

	inj_side = (IUI_FM_FMR_INJECTION_SIDE_LOW ==
		mitigation->info.fmr_inj_side) ? FMR_FORCE_LSI : FMR_FORCE_HSI;

	fmdrv_dbg("FM mitigation cb - change inj side to %d\n", inj_side);

	mutex_lock(&xgold_fmdev->lock);

	/* set injection side */
	rc = fmrx_set_sideband_injection(inj_side, FMR_SB_SEL_TMP);

	if (rc != 0) {
		fmdrv_warn("failed to set inj side : %d, rc: %d\n",
			   inj_side, rc);
		mitigation_status = IUI_FM_MITIGATION_ERROR;
	} else {
		fmdrv_dbg("requested inj side: %u, changed successfully\n",
			  inj_side);
		mitigation_status = IUI_FM_MITIGATION_COMPLETE_OK;
	}

	mutex_unlock(&xgold_fmdev->lock);

	return mitigation_status;
}

static void xgold_fmdrv_notify_fm_cb(unsigned long data)
{
	struct fmtrx_msg_params event_params = {0};

	event_params.fmr_mod_id = FMR_MODULE_SYS;
	event_params.fmr_mod_msg_id = MODULE_SYS_FM;
	memcpy(event_params.data, (struct iui_fm_fmr_info *)data,
	       sizeof(struct iui_fm_fmr_info));

	/* check if FMR device is active */
	if (FMTRX_HW_STATE_RX_ACTIVE == fmtrx_get_hw_state()) {
		if (0 != fmr_sys_msg_send(&event_params))
			fmdrv_err("error in sending notification to FM\n");
	}
}

static int xgold_fmdrv_fm_notify(u32 freq, enum fmrx_sb_inj_side inj_side)
{
	static struct iui_fm_fmr_info freq_mgr_fmrinfo;
	int rc = 0;

	freq_mgr_fmrinfo.rx_freq = freq / 1000; /* frequency in kHz */
	freq_mgr_fmrinfo.inj_side = (FMR_FORCE_LSI == inj_side) ?
		IUI_FM_FMR_INJECTION_SIDE_LOW : IUI_FM_FMR_INJECTION_SIDE_HIGH;

	fm_notify_timer.data = (unsigned long)&freq_mgr_fmrinfo;

	if (timer_pending(&fm_notify_timer))
		rc = mod_timer(&fm_notify_timer,
			       jiffies + msecs_to_jiffies(TIME_TO_NOTIFY_FM));
	else {
		fm_notify_timer.expires =
			jiffies + msecs_to_jiffies(TIME_TO_NOTIFY_FM);
		add_timer(&fm_notify_timer);
	}

	return rc;
}

static void xgold_fmdrv_fm_denotify(void)
{
	struct iui_fm_fmr_info freq_mgr_fmrinfo;
	struct iui_fm_freq_notification freq_mgr_notify_data;

	freq_mgr_fmrinfo.rx_freq = 0;
	freq_mgr_fmrinfo.inj_side = 0;

	freq_mgr_notify_data.type = IUI_FM_FREQ_NOTIFICATION_TYPE_FMR;
	freq_mgr_notify_data.info.fmr_info = &freq_mgr_fmrinfo;

	fmdrv_info("Notify FM with freq: %d and inj side: %d\n",
		   freq_mgr_fmrinfo.rx_freq, freq_mgr_fmrinfo.inj_side);

	fmdrv_err("FM notification error code %d\n",
		  iui_fm_notify_frequency(IUI_FM_MACRO_ID_FMR,
					  &freq_mgr_notify_data));
}
#endif

static int xgold_fmdrv_idi_channel_setup(struct xgold_fmdev *fmdev)
{
	int rc = 0;

#ifdef CONFIG_IDI
	static struct idi_channel_config idi_rx_cfg;

	idi_rx_cfg.dst_addr     = 0x01008530; /* ABB_FMR_TX */
	idi_rx_cfg.channel_opts = IDI_PRIMARY_CHANNEL;
	idi_rx_cfg.tx_or_rx     = 0;
	idi_rx_cfg.base         = 0xE3606C00; /* DBB_FMR_RX */
	idi_rx_cfg.cpu_base     = NULL;
	idi_rx_cfg.size         = 256;
	idi_rx_cfg.hw_fifo_size = 4;
	idi_rx_cfg.priority     = IDI_HIGH_PRIORITY;

	/* Configure IDI channel for DSP mode */
	rc = idi_set_channel_config(fmdev->idi_fmdev.fmr_dev, &idi_rx_cfg);
#endif

	return rc;
}

static int xgold_fmdrv_set_routing_mode(struct xgold_fmdev *fmdev,
	enum fmrx_aud_route aud_route)
{
	int rc = 0;

	switch (aud_route) {
	case FMR_AUD_PATH_DSP:
		fmdrv_err("Selecting DSP mode");

		rc = xgold_fmdrv_idi_channel_setup(fmdev);
		if (rc != 0) {
			fmdrv_err("IDI channel setup for DSP mode failed\n");
			goto xgold_fmdrv_set_routing_mode_err;
		}

		rc = fmrx_set_idi_handshake(true);
		if (rc != 0) {
			fmdrv_warn("setting IDI hs bit failed\n");
			goto xgold_fmdrv_set_routing_mode_err;
		}

		rc = fmrx_set_route((enum fmrx_aud_route) aud_route);
		if (rc != 0) {
			fmdrv_info("failed to set audio routing mode to %d\n",
				   aud_route);
			goto xgold_fmdrv_set_routing_mode_err;
		}
		break;

	case FMR_AUD_PATH_DAC:
		fmdrv_info("Selecting DAC mode");

		rc = fmrx_set_route((enum fmrx_aud_route) aud_route);
		if (rc != 0) {
			fmdrv_info("failed to set audio routing mode to %d\n",
				   aud_route);
			goto xgold_fmdrv_set_routing_mode_err;
		}

		rc = fmrx_set_idi_handshake(false);
		if (rc != 0) {
			fmdrv_warn("setting IDI hs bit failed\n");
			goto xgold_fmdrv_set_routing_mode_err;
		}
		break;

	case FMR_AUD_PATH_OFF:
		rc = fmrx_set_route((enum fmrx_aud_route) aud_route);
		if (rc != 0) {
			fmdrv_info("failed to set audio routing mode to %d\n",
				   aud_route);
			goto xgold_fmdrv_set_routing_mode_err;
		}
		break;

	default:
		rc = -EINVAL;
		fmdrv_dbg("Invalid audio routing mode\n");
		break;
	}

xgold_fmdrv_set_routing_mode_err:
	return rc;
}

static void xgold_fmdrv_clear_rds_buf(struct xgold_fmdev *fmdev)
{
	struct xgold_fmrx_rds *rds = &fmdev->fmrx_rds;

	rds->rd_idx = 0;
	rds->wr_idx = 0;
	memset(rds->rds_rpt_buf, 0, rds->array_len);
	atomic_set(&rds->is_full, 0);
	atomic_set(&rds->rpt_cnt, 0);
}

static int xgold_fmdrv_enable_rds(struct xgold_fmdev *fmdev)
{
	struct fmrx_rds_subscribe_params rds_params = {0};
	struct fmrx_rds_cfg rds_cfgs = {0};
	struct xgold_fmrx_rds *rds = &fmdev->fmrx_rds;
	int rc = 0;

	/* Set RDS sync Configuration when FMR is in enabled */
	rds_cfgs.rds_mode = FMR_RDS;
	rds_cfgs.min_free = RDS_MIN_FREE;
	rds_cfgs.sync_cfg.gblk = 20;
	rds_cfgs.sync_cfg.bblk = 20;
	rds_cfgs.sync_cfg.bblks = 20;

	rc = fmrx_set_rds_cfg(&rds_cfgs);
	if (rc != 0) {
		fmdrv_err("Failed to set the RDS configuration\n");
		goto xgold_fmdrv_enable_rds_err;
	}

	init_waitqueue_head(&rds->read_queue);
	spin_lock_init(&fmdev->rds_buff_lock);

	rds_params.min_free = RDS_MIN_FREE;
	rds_params.rds_pi_mode = RDS_PI_MODE;
	rds_params.rds_subsc_mode = RDS_MODE_IRQ;
	rds_params.rds_type = FMR_RDS;

	xgold_fmdrv_clear_rds_buf(fmdev);

	/* Subscribe for RDS data */
	rc = fmrx_subscribe_rds(&rds_params);

	if (rc == 0) {
		atomic_set(&rds->rds_mode, 1);
		fmdrv_err("RDS is enabled\n");
	} else {
		fmdrv_err("Failed to enable RDS\n");
		atomic_set(&rds->rds_mode, 0);
	}

xgold_fmdrv_enable_rds_err:
	return rc;
}

static int xgold_fmdrv_disable_rds(struct xgold_fmdev *fmdev)
{
	int rc = 0;
	struct xgold_fmrx_rds *rds = &fmdev->fmrx_rds;

	if (atomic_read(&rds->rds_mode) == 1) {
		rc = fmrx_unsubscribe_rds(); /* unsubscribe for RDS data */

		if (0 == rc) {
			atomic_set(&rds->rds_mode, 0);
			wake_up_interruptible_all(&rds->read_queue);

			fmdrv_err("RDS is disabled\n");
		} else {
			fmdrv_err("Failed to disable RDS\n");
		}
	}

	return rc;
}

static void xgold_fmdrv_dis_rds_on_buff_full(unsigned long data)
{
	if (atomic_read(&xgold_fmdev->fmrx_rds.is_full) == 1) {
		if (atomic_read(&xgold_fmdev->fmrx_rds.rds_mode) == 1)
			xgold_fmdrv_disable_rds(xgold_fmdev);
	}
}

void xgold_fmdrv_schedule_disable_rds(void)
{
	if (!timer_pending(&fmr_rds_dis_timer)) {
		fmdrv_dbg("RDS buffer is full\n");
		fmr_rds_dis_timer.expires =
			jiffies + msecs_to_jiffies(TIME_TO_DIS_RDS);
		add_timer(&fmr_rds_dis_timer);
	}
}

static int xgold_fmdrv_fmrx_rds(u16 filled_cnt, u16 overflow_cnt)
{
	u16 copied_grps = 0, i = 0;
	struct xgold_fmrx_rds *rds = &xgold_fmdev->fmrx_rds;
	int rc = -EIO;
	unsigned long flags = 0;
	struct fmtrx_rds_buff_t buf_desc;

	if (filled_cnt == 0) {
		rc = 0;
		goto xgold_fmdrv_fmrx_rds_err;
	}

	/* Return if RDS buffer is already full */
	if (atomic_read(&rds->is_full) == 1) {
		wake_up_interruptible_all(&rds->read_queue);
		xgold_fmdrv_schedule_disable_rds();
		rc = 0;
		goto xgold_fmdrv_fmrx_rds_err;
	}

	/* Allocate a non-cache 32-bit aligned memory */
	buf_desc.size = rds->array_len;
	buf_desc.buf = dma_alloc_coherent(NULL, buf_desc.size,
		&buf_desc.dma_handle, GFP_KERNEL | GFP_DMA);

	if (NULL == buf_desc.buf) {
		rc = -ENOMEM;
		fmdrv_crit("Error allocating buffer, for RDS data from FW\n");
		goto xgold_fmdrv_fmrx_rds_err;
	}

	memset(buf_desc.buf, 0, buf_desc.size);

	mutex_lock(&xgold_fmdev->lock);
	rc = fmrx_transfer_rds_groups(filled_cnt, &buf_desc, &copied_grps);
	mutex_unlock(&xgold_fmdev->lock);

	if (rc != 0) {
		fmdrv_err("Error in transferring RDS groups\n");
		goto xgold_fmdrv_fmrx_rds_err1;
	}

	spin_lock_irqsave(&xgold_fmdev->rds_buff_lock, flags);

	for (i = 0; i < copied_grps; i++) {
		memcpy(rds->rds_rpt_buf + rds->wr_idx,
		       buf_desc.buf + i * FMR_RDS_GRP_LEN, FMR_RDS_GRP_LEN);

		rds->wr_idx = rds->wr_idx + FMR_RDS_GRP_LEN;
		rds->wr_idx %= rds->array_len;

		/* Overflow check & start over */
		if (rds->wr_idx == rds->rd_idx) {
			fmdrv_dbg("RDS buffer overflow\n");
			atomic_set(&rds->is_full, 1);
			break;
		}

		atomic_inc(&rds->rpt_cnt);
	}

	spin_unlock_irqrestore(&xgold_fmdev->rds_buff_lock, flags);

	fmdrv_dbg("RDS records filled: %d\n", atomic_read(&rds->rpt_cnt));

	/* wake up read queue */
	if ((rds->wr_idx != rds->rd_idx) || (atomic_read(&rds->is_full) == 1))
		wake_up_interruptible_all(&rds->read_queue);

xgold_fmdrv_fmrx_rds_err1:
	if (NULL != buf_desc.buf)
		dma_free_coherent(NULL, buf_desc.size, buf_desc.buf,
				  buf_desc.dma_handle);

xgold_fmdrv_fmrx_rds_err:
	return rc;
}

/* Copies RDS data from internal buffer to user buffer */
static s32 xgold_fmdrv_xfer_rds_from_int_buff(struct file *file,
	struct fmtrx_rds_report *rds_rpt)
{
	unsigned long flags = 0;
	int rc = 0;
	struct xgold_fmdev *fmdev = video_drvdata(file);
	struct xgold_fmrx_rds *rds = &fmdev->fmrx_rds;
	u32 recs_req = 0; /* no of recs requested */
	u32 i = 0, recs_to_copy = 0;

	if (rds_rpt != NULL) {
		if (rds_rpt->rpt_array == NULL || rds_rpt->array_len <= 0 ||
		    rds_rpt->rpt_cnt == NULL) {
			rc = -EINVAL;
			goto xgold_fmdrv_transfer_rds_err;
		}
	} else {
		rc = -EINVAL;
		goto xgold_fmdrv_transfer_rds_err;
	}

	/* Data is not available, so either block or wait */
	if (atomic_read(&rds->rpt_cnt) == 0) {
		if (file->f_flags & O_NONBLOCK) {
			fmdrv_dbg("RDS data not available\n");
			rc = -EWOULDBLOCK;
			goto xgold_fmdrv_transfer_rds_err;
		}

		rc = wait_event_interruptible_timeout(rds->read_queue,
			(atomic_read(&rds->rpt_cnt) > 0) ||
			(atomic_read(&rds->rds_mode) == 0),
			msecs_to_jiffies(TIME_TO_DIS_RDS));

		if (rc == 0)
			return -ETIMEDOUT;

		if (rc < 0)
			goto xgold_fmdrv_transfer_rds_err;
	}

	/* Clear the user buffer */
	memset(rds_rpt->rpt_array, 0, rds_rpt->array_len);

	/* Calc number of rds reports to be accomodated in the given buf */
	recs_req = rds_rpt->array_len / FMR_RDS_GRP_LEN;

	/* Total records that can be returned - based on read buffer size and
	 * current available RDS buffers */
	recs_to_copy = (atomic_read(&rds->rpt_cnt) <= recs_req) ?
		atomic_read(&rds->rpt_cnt) : recs_req;

	spin_lock_irqsave(&fmdev->rds_buff_lock, flags);

	for (i = 0; i < recs_to_copy; i++) {
		if ((rds->wr_idx == rds->rd_idx) &&
		    (atomic_read(&rds->is_full) == 0))
			break;

		/* Copy to user space buffer */
		copy_to_user(rds_rpt->rpt_array + i * FMR_RDS_GRP_LEN,
			     rds->rds_rpt_buf + rds->rd_idx, FMR_RDS_GRP_LEN);

		rds->rd_idx = rds->rd_idx + FMR_RDS_GRP_LEN;
		rds->rd_idx %= rds->array_len;

		if (atomic_read(&rds->is_full) == 1)
			atomic_set(&rds->is_full, 0);

		atomic_dec(&rds->rpt_cnt);
	}

	spin_unlock_irqrestore(&fmdev->rds_buff_lock, flags);

	/* Copy no. of records copied */
	*rds_rpt->rpt_cnt = recs_to_copy;

	fmdrv_dbg("Copied records :%u\tRemaining records: %u\n",
		  *rds_rpt->rpt_cnt, atomic_read(&rds->rpt_cnt));

xgold_fmdrv_transfer_rds_err:
	return rc;
}

static int xgold_fmdrv_run_hld_task(void *data)
{
	while (!kthread_should_stop()) {
		fmr_sys_sim_dispatcher();
		msleep(50);
	}

	fmdrv_dbg("HLD task has been stopped!\r\n");

	return 0;
}

static int xgold_fmdrv_fmrx_seek_eval_report(enum fmrx_scan_status scan_status,
	enum fmtrx_as_eval_rpt_type rpt_type,/* Report type */
	struct fmtrx_rssi_report *report,/* Pointer to the seek report */
	u16 max_cnt) /* Number of stations found during seeks */
{
	u16 i = 0;
	fmdrv_dbg("%s Evaluation is done! No. of freq points evlauated %d",
		  FMRX_EVAL_AF_RPT == rpt_type ? "AF" : "Auto RSSI",  max_cnt);

	for (i = 0; i < max_cnt; i++)
		fmdrv_dbg("[%2d] Frequency %u Hz, RSSI %d dBuV.",
			  i, report[i].freq, (s16)report[i].rssi);

	return 0;
}

static int xgold_fmdrv_fmrx_seek_stepped(enum fmtrx_seek_dir seek_dir,
	u32 freq, s16 rssi, u16 pn)
{
	fmdrv_dbg("Seeking %s stepped: %d Hz, %d dBuV, PN is %d.",
		  (FMRX_SEEK_DOWN == seek_dir) ?
		  "down" : (FMRX_SEEK_UP == seek_dir) ? "up" :
		  (FMRX_SEEK_DOWN_TO_LIMIT == seek_dir) ?
		 "down to lower limit" : "up to higher limit", freq, rssi, pn);

	return 0;
}

static int xgold_fmdrv_fmrx_channel_tuned(
	enum fmrx_tuned_state tune_state,/* tune/seek state */
	u32 freq, /* The frequency found during tuning */
	s16 rssi, /* RSSI level of the found frequency */
	enum fmrx_sb_inj_side inj_side) /* Injection side */
{
	int rc = 0;

	fmdrv_dbg("Tune_state : %u, freq : %u rssi :%d, inj side: %d\n",
		  tune_state, freq, rssi, inj_side);

#ifdef CONFIG_IUI_FM_FMR
	rc = xgold_fmdrv_fm_notify(freq, inj_side);
#endif

	switch (tune_state) {
	case FMRX_TUNING_DONE:
		xgold_fmdev->event_type = XGOLD_EVT_TUNE_SUCC;
		fmdrv_dbg("Tuning done successful - %dHz, RSSI: %d.\n",
			  freq, rssi);
		break;

	case FMRX_TUNING_FAILED:
		xgold_fmdev->event_type = XGOLD_EVT_TUNE_FAIL;
		fmdrv_err("Tuning failed - %dHz, RSSI: %d.\n",
			  freq, rssi);
		break;

	case FMRX_SEEK_DONE:
		xgold_fmdev->event_type = XGOLD_EVT_SEEK_COMPLETE;
		fmdrv_dbg("Auto search up/down is done :%dHz, RSSI:%d\n",
			  freq, rssi);
		break;

	case FMRX_SEEK_FAILED:
		xgold_fmdev->event_type = XGOLD_EVT_TUNE_FAIL;
		fmdrv_err("Auto search up/down failed.\n");
		break;

	case FMRX_SEEK_STOP:
		xgold_fmdev->event_type = XGOLD_EVT_TUNE_SUCC;
		fmdrv_dbg("Seek done\n");
		break;

	case FMRX_AUTOEVAL_DONE:
		xgold_fmdev->event_type = XGOLD_EVT_SEEK_COMPLETE;
		fmdrv_dbg("Auto Eval done\n");
		break;

	case FMRX_AFEVAL_DONE:
		xgold_fmdev->event_type = XGOLD_EVT_SEEK_COMPLETE;
		fmdrv_dbg("AF Evaluation done\n");
		break;

	default:
		fmdrv_dbg("Unhandled tune state : %d\n", tune_state);
		break;
	}

	wake_up_interruptible_all(&xgold_fmdev->event_queue);
	return rc;
}

static int xgold_fmdrv_init_hld(struct xgold_fmdev *fmdev)
{
	int rc = -EIO;

	rc = fmr_sys_msg_init(fmdev);

	if (rc != 0) {
		fmdrv_dbg("Error in initialing system messaging\n");
		goto xgold_fmdrv_init_hld_err;
	}

	/* Initialize the FMR HLD module */
	rc = fmtrx_init(FMTRX_INIT_MODE_ON);
	if (rc != 0)
		goto xgold_fmdrv_init_hld_err;

	fmdrv_dbg("HLD module initialised successfully\n");

	/* Callbacks registering, map to 'NULL' if disabled */
	fmrx_cbs.p_aud_route_changed_cb = NULL;
	fmrx_cbs.p_seek_eval_rpt_cb = xgold_fmdrv_fmrx_seek_eval_report;
	fmrx_cbs.p_ch_tuned_cb = xgold_fmdrv_fmrx_channel_tuned;
	fmrx_cbs.p_pilot_found_cb = NULL;
	fmrx_cbs.p_pow_onoff_cb = NULL;
	fmrx_cbs.p_rds_cb = xgold_fmdrv_fmrx_rds;
	fmrx_cbs.p_rds_fastpi_cb = NULL;
	fmrx_cbs.p_rds_synced_cb = NULL;
	fmrx_cbs.p_rssi_cb = NULL;
	fmrx_cbs.p_seek_stepped_cb = xgold_fmdrv_fmrx_seek_stepped;

	rc = fmrx_register_callbacks(&fmrx_cbs);
	if (rc != 0)
		goto xgold_fmdrv_init_hld_err;

xgold_fmdrv_init_hld_err:
	return rc;
}

static int xgold_fmdrv_deinit_hld(void)
{
	int rc = -EIO;

	rc = fmtrx_init(FMTRX_INIT_MODE_OFF);
	if (rc != 0)
		fmdrv_err("HLD de-init failed\n");

	return rc;
}

static int xgold_fmdrv_close_fmdev(struct xgold_fmdev *fmdev)
{
	int rc = 0;
	enum fmtrx_hw_state fmr_hw_state = fmtrx_get_hw_state();
	struct xgold_fmrx_rds *rds = &fmdev->fmrx_rds;

	if (FMTRX_HW_STATE_SEEK_ACTIVE == fmr_hw_state) {
		fmdev->event_type = XGOLD_EVT_TUNE_FAIL;
		wake_up_interruptible_all(&fmdev->event_queue);
	}

	if ((FMTRX_HW_STATE_IDLE != fmr_hw_state) &&
	    (FMTRX_HW_STATE_INVALID != fmr_hw_state)) {
		/* Stop the HLD thread */
		if (fmdev->hld_task) {
			kthread_stop(fmdev->hld_task);
			fmdev->hld_task = NULL;
		}

		rc = fmrx_power_off();
		if (rc != 0) {
			fmdrv_err("couldn't power off Macro\n");
			goto xgold_fmdrv_close_fmdev_err;
		}

		fmdrv_info("power off successfull\n");

		fmr_sys_on_off_seq(FMR_POWER_OFF);

#ifdef CONFIG_IUI_FM_FMR
		del_timer_sync(&fm_notify_timer);
#endif
		del_timer_sync(&fmr_rds_dis_timer);

		/* free rds buffer */
		if (rds->rds_rpt_buf != NULL) {
			kfree(rds->rds_rpt_buf);
			fmdrv_info("RDS buffer freed\n");
			rds->rds_rpt_buf = NULL;
		}

#ifdef CONFIG_IUI_FM_FMR
		xgold_fmdrv_fm_denotify();
#endif

		atomic_set(&fmdev->dev_open_count, 0);
		iounmap(fmdev->ctrl_io);
		fmdev->ctrl_io = NULL;

		/* disown the reserved memory space */
		release_mem_region(fmdev->xg_ram->start,
				   resource_size(fmdev->xg_ram) +
				   resource_size(fmdev->reg_space));

		fmdrv_dbg("device successfully closed\n");
	}

xgold_fmdrv_close_fmdev_err:
	return rc;
}

static ssize_t xgold_fmdrv_v4l2_fops_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	ssize_t rc = 0;
	struct fmtrx_rds_report rds_rpt;
	struct xgold_fmdev *fmdev = video_drvdata(file);
	u8 rds_rpt_cnt = 0;

	if (FMTRX_HW_STATE_RX_ACTIVE != fmtrx_get_hw_state()) {
		rc = -EPERM;
		goto xgold_fmdrv_v4l2_fops_read_err;
	}

	/* Turn on RDS mode, if it is disabled */
	if (atomic_read(&fmdev->fmrx_rds.rds_mode) == 0) {
		mutex_lock(&fmdev->lock);
		rc = xgold_fmdrv_enable_rds(fmdev);
		mutex_unlock(&fmdev->lock);

		if (rc != 0) {
			fmdrv_err("Error in enabling RDS reception\n");
			goto xgold_fmdrv_v4l2_fops_read_err;
		}
	}

	rds_rpt.rpt_array = buf;
	rds_rpt.array_len = count;
	rds_rpt.rpt_cnt = &rds_rpt_cnt;

	rc = xgold_fmdrv_xfer_rds_from_int_buff(file, &rds_rpt);
	if (rc == 0)
		rc = rds_rpt_cnt * FMR_RDS_GRP_LEN;

xgold_fmdrv_v4l2_fops_read_err:
	return rc;
}

static u32 xgold_fmdrv_v4l2_fops_poll(struct file *file,
	struct poll_table_struct *pts)
{
	int rc = 0;
	struct xgold_fmdev *fmdev = video_drvdata(file);
	struct xgold_fmrx_rds *rds = &fmdev->fmrx_rds;
	enum fmtrx_hw_state fmr_hw_state = fmtrx_get_hw_state();

	if (FMTRX_HW_STATE_RX_ACTIVE != fmr_hw_state) {
		rc = 0;
		fmdrv_err("Not in receiving state\n");
		goto xgold_fmdrv_v4l2_fops_poll_err;
	}

	/* Turn on RDS mode, if it is disabled */
	if (atomic_read(&rds->rds_mode) == 0) {
		mutex_lock(&fmdev->lock);
		rc = xgold_fmdrv_enable_rds(fmdev);
		mutex_unlock(&fmdev->lock);

		if (rc != 0) {
			rc = POLLERR;
			fmdrv_err("Error in enabling RDS reception\n");
			goto xgold_fmdrv_v4l2_fops_poll_err;
		}
	}

	poll_wait(file, &rds->read_queue, pts);

	if (atomic_read(&rds->rpt_cnt) > 0)
		rc = POLLIN | POLLRDNORM;

xgold_fmdrv_v4l2_fops_poll_err:
	return rc;
}

static int xgold_fmdrv_v4l2_fops_open(struct file *file)
{
	struct xgold_fmdev *fmdev = NULL;
	struct xgold_fmrx_rds *rds = NULL;
	struct fmrx_cfg *cfg = NULL;
	int rc = 0;

#ifdef CONFIG_IUI_FM_FMR
	struct fmrx_ch_info ch_info;
#endif

	/* Get driver data */
	fmdev = video_drvdata(file);
	if (NULL == fmdev) {
		rc = -EINVAL;
		fmdrv_err("Invalid driver data\n");
		goto xgold_fmdrv_open_err1;
	}

	rds = &fmdev->fmrx_rds;

	if (atomic_read(&fmdev->dev_open_count) > 0) {
		fmdrv_err("device already in active state\n");
		atomic_inc(&fmdev->dev_open_count);
		return 0;
	}

	if (mutex_lock_interruptible(&fmdev->lock))
		return -EINTR;

	rc = fmr_sys_get_cfg(&cfg);
	if (rc) {
		fmdrv_err("Getting the config data failed. rc : %d\n", rc);
		goto xgold_fmdrv_open_err1;
	}

	rc = fmrx_set_cfg(cfg);
	if (rc) {
		fmdrv_err("Setting configuration failed\n");
		goto xgold_fmdrv_open_err2;
	}

	if (!request_mem_region(fmdev->xg_ram->start,
				resource_size(fmdev->xg_ram) +
				resource_size(fmdev->reg_space),
				dev_name(&fmdev->idi_fmdev.fmr_dev->device))) {
		fmdrv_err("Cannot reserve device memory space\n");
		rc = -EBUSY;
		goto xgold_fmdrv_open_err2;
	}

	/* Perform IO remap of FM Radio device physical space */
	fmdev->ctrl_io = ioremap(fmdev->xg_ram->start,
		resource_size(fmdev->xg_ram) +
		resource_size(fmdev->reg_space));

	if (fmdev->ctrl_io == NULL) {
		rc = -EPERM;
		fmdrv_crit("IO remap failed\n");
		goto xgold_fmdrv_open_err3;
	}

	fmdev->hld_task = kthread_run(xgold_fmdrv_run_hld_task, fmdev,
		"fmr_hld_task");
	if (IS_ERR(fmdev->hld_task)) {
		fmdrv_err("HLD task creation failed\n");
		rc = -EIO;
		goto xgold_fmdrv_open_err4;
	}

	rds->array_len = sizeof(struct v4l2_rds_data) * FMR_RDS_GRP_BLK_MAX;
	rds->rds_rpt_buf = kzalloc(rds->array_len, GFP_KERNEL);

	if (rds->rds_rpt_buf == NULL) {
		rc = -ENOMEM;
		fmdrv_dbg("Error in allocating rds buffer\n");
		goto xgold_fmdrv_open_err5;
	}

	/* Switch on the FMR macro */
	rc = fmrx_power_on();
	if (rc) {
		fmdrv_warn("Powering on failed\n");
		goto xgold_fmdrv_open_err6;
	}

#ifdef CONFIG_IUI_FM_FMR
	rc = fmrx_get_channel_info(&ch_info);
	if (rc) {
		fmdrv_err("Unable to get ch info to report to freq manager\n");
		goto xgold_fmdrv_open_err7;
	}

	xgold_fmdrv_fm_notify(ch_info.freq, ch_info.injside);
#endif

	atomic_inc(&fmdev->dev_open_count);
	mutex_unlock(&fmdev->lock);
	fmdrv_info("Device powered on successfully\n");

	return rc;

#ifdef CONFIG_IUI_FM_FMR
xgold_fmdrv_open_err7:
	fmdrv_info("Powering down\n");

	rc = fmrx_power_off();
	if (rc)
		fmdrv_err("Power off failed\n");
#endif

xgold_fmdrv_open_err6:
	if (rds->rds_rpt_buf != NULL) {
		kfree(rds->rds_rpt_buf);
		rds->rds_rpt_buf = NULL;
	}

xgold_fmdrv_open_err5:
	if (fmdev->hld_task) {
		kthread_stop(fmdev->hld_task);
		fmdev->hld_task = NULL;
	}

xgold_fmdrv_open_err4:
	iounmap(fmdev->ctrl_io);
	fmdev->ctrl_io = NULL;

xgold_fmdrv_open_err3:
	release_mem_region(fmdev->xg_ram->start, resource_size(fmdev->xg_ram) +
			   resource_size(fmdev->reg_space));

xgold_fmdrv_open_err2:
	fmr_sys_on_off_seq(FMR_POWER_OFF);
	cfg = NULL;

xgold_fmdrv_open_err1:
	mutex_unlock(&fmdev->lock);
	return rc;
}

static int xgold_fmdrv_v4l2_fops_release(struct file *file)
{
	int rc = 0;
	struct xgold_fmdev *fmdev = video_drvdata(file);

	fmdrv_dbg("device release requested\n");

	if (mutex_lock_interruptible(&fmdev->lock))
		return -EINTR;

	xgold_fmdrv_disable_rds(fmdev); /* disable rds */

	if (atomic_dec_and_test(&fmdev->dev_open_count)) {
		rc = xgold_fmdrv_close_fmdev(fmdev);
		if (rc != 0)
			fmdrv_warn("Unable to close the device\n");
	} else {
		fmdrv_dbg("Open count: %d. Device not closed\n",
			  atomic_read(&fmdev->dev_open_count));
	}

	mutex_unlock(&fmdev->lock);
	return rc;
}

/* Video4Linux Interface */
/* xgold_fmdrv_fops - file operations interface */
static const struct v4l2_file_operations xgold_fmdrv_fops = {
	.owner		= THIS_MODULE,
	.open		= xgold_fmdrv_v4l2_fops_open,
	.read		= xgold_fmdrv_v4l2_fops_read,
	.poll		= xgold_fmdrv_v4l2_fops_poll,
	.release	= xgold_fmdrv_v4l2_fops_release,
	.unlocked_ioctl	= video_ioctl2
};

/* xgold_fmdrv_vidioc_querycap - query device capabilities */
static int xgold_fmdrv_vidioc_querycap(struct file *file, void *priv,
	struct v4l2_capability *capability)
{
	strlcpy(capability->driver, XGOLD_FMDRV_NAME,
		sizeof(capability->driver));
	strlcpy(capability->card, XGOLD_FMDRV_CARD_SHORT_NAME,
		sizeof(capability->card));
	sprintf(capability->bus_info, XGOLD_FMDRV_BUS_INFO);
	capability->version = XGOLD_FMDRV_RADIO_VERSION;
	capability->capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO |
		V4L2_CAP_AUDIO | V4L2_CAP_RDS_CAPTURE | V4L2_CAP_HW_FREQ_SEEK;
	return 0;
}

/* xgold_fmdrv_vidioc_g_tuner - get tuner attributes */
static int xgold_fmdrv_vidioc_g_tuner(struct file *file, void *priv,
	struct v4l2_tuner *tuner)
{
	struct xgold_fmdev *fmdev = video_drvdata(file);
	int rc = -EIO;
	struct fmtrx_band fmrx_band;
	struct fmrx_ch_info ch_info;

	if (0 != tuner->index)
		return -EINVAL;

	if (mutex_lock_interruptible(&fmdev->lock))
		return -EINTR;

	rc = fmrx_get_band_info(&fmrx_band);
	if (rc != 0) {
		rc = rc;
		goto xgold_fmdrv_vidioc_g_tuner_err;
	}

	rc = fmrx_get_channel_info(&ch_info);
	if (rc != 0) {
		rc = rc;
		goto xgold_fmdrv_vidioc_g_tuner_err;
	}

	mutex_unlock(&fmdev->lock);

	strlcpy(tuner->name, XGOLD_FMDRV_NAME, sizeof(tuner->name));
	tuner->type = V4L2_TUNER_RADIO;
	tuner->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_RDS |
		V4L2_TUNER_CAP_STEREO;
	tuner->rangelow   = fmrx_band.min * 10 / 625;
	tuner->rangehigh  = fmrx_band.max * 10 / 625;
	tuner->rxsubchans = V4L2_TUNER_SUB_MONO | V4L2_TUNER_SUB_STEREO |
			((RDS_ON == ch_info.rds_en) ? V4L2_TUNER_SUB_RDS : 0);
	tuner->audmode = (ch_info.mono ?
		V4L2_TUNER_MODE_STEREO : V4L2_TUNER_MODE_MONO);
	tuner->signal = ch_info.rssi;

	return rc;

xgold_fmdrv_vidioc_g_tuner_err:
	mutex_unlock(&fmdev->lock);
	return rc;
}

/* xgold_fmdrv_vidioc_s_tuner - set tuner attributes */
static int xgold_fmdrv_vidioc_s_tuner(struct file *file, void *priv,
	const struct v4l2_tuner *tuner)
{
	struct xgold_fmdev *fmdev = video_drvdata(file);
	int rc = 0;
	struct fmrx_cfg *cfg = NULL;

	/* Check if tuner index is valid */
	if (0 != tuner->index)
		return -EINVAL;

	if (mutex_lock_interruptible(&fmdev->lock))
		return -EINTR;

	/* Get static configuration */
	rc = fmrx_get_cfg(&cfg);
	if (rc != 0) {
		fmdrv_warn("Failed to get the static cfg data\n");
		mutex_unlock(&fmdev->lock);
		return rc;
	}

	/* Set force mono if not set already */
	if (cfg->force_mono != (!tuner->audmode)) {
		rc = fmrx_set_force_mono(!tuner->audmode);
		if (rc != 0) {
			fmdrv_err("Failed to set RX stereo/mono mode\n");
			mutex_unlock(&fmdev->lock);
			return rc;
		}
	}

	mutex_unlock(&fmdev->lock);

	return rc;
}

/* Set tuner frequency */
static int xgold_fmdrv_vidioc_s_frequency(struct file *file, void *priv,
	const struct v4l2_frequency *v4l2_freq)
{
	struct xgold_fmdev *fmdev = video_drvdata(file);
	int rc = -EIO;
	u32 freq = 0;

	if (v4l2_freq == NULL)
		return -EINVAL;

	if (0 != v4l2_freq->tuner || V4L2_TUNER_RADIO != v4l2_freq->type)
		return -EINVAL;

	xgold_fmdrv_clear_rds_buf(fmdev);

	freq = v4l2_freq->frequency * 625 / 10; /* freq in Hz unit */

	/* Cancel any previous seek requests - unblock the SEEK IOCTL */
	if (FMTRX_HW_STATE_SEEK_ACTIVE == fmtrx_get_hw_state()) {
		fmdev->event_type = XGOLD_EVT_SEEK_COMPLETE;
		wake_up_interruptible(&fmdev->event_queue);

		fmr_sys_trigger_event(FMR_IR_CMD_DONE);
		fmdrv_info("Terminating the ongoing search\n");
	}

	if (mutex_lock_interruptible(&fmdev->lock))
		return -EINTR;

	fmdrv_info("Tuning to station %d Hz\n", freq);
	rc = fmrx_station_tuning(freq);
	if (rc != 0) {
		fmdrv_err("Tuning to station failed - rc : %d\n", rc);
		goto xgold_fmdrv_vidioc_s_freq_err;
	}

	/* Block until seek operation is complete based on the file
	 * open flags */
	if (!(file->f_flags & O_NONBLOCK)) {
		rc = wait_event_interruptible(fmdev->event_queue,
			(XGOLD_EVT_RADIO_READY != fmdev->event_type));
		if (rc)
			goto xgold_fmdrv_vidioc_s_freq_err;

		/* Set result to try again, if tune failed */
		if (XGOLD_EVT_TUNE_FAIL == fmdev->event_type)
			rc = -EAGAIN;

		fmdev->event_type = XGOLD_EVT_RADIO_READY;
	}

xgold_fmdrv_vidioc_s_freq_err:
	mutex_unlock(&fmdev->lock);
	return rc;
}

/* Get tuner radio frequency */
static int xgold_fmdrv_vidioc_g_frequency(struct file *file, void *priv,
	struct v4l2_frequency *v4l2_freq)
{
	struct fmrx_ch_info ch_info;
	int rc = -EIO;
	struct xgold_fmdev *fmdev = video_drvdata(file);

	if (FMTRX_HW_STATE_RX_ACTIVE != fmtrx_get_hw_state()) {
		fmdrv_err("Not in receiving state\n");
		rc = -EIO;
		goto xgold_fmdrv_vidioc_g_frequency_err;
	}

	if (mutex_lock_interruptible(&fmdev->lock))
		return -EINTR;

	rc = fmrx_get_channel_info(&ch_info);

	mutex_unlock(&fmdev->lock);

	v4l2_freq->type = V4L2_TUNER_RADIO;

	if (0 == rc) {
		v4l2_freq->frequency = ch_info.freq * 10 / 625;
	} else {
		v4l2_freq->frequency = 0;
		rc = -EINVAL;
		goto xgold_fmdrv_vidioc_g_frequency_err;
	}

xgold_fmdrv_vidioc_g_frequency_err:
	return rc;
}

/* xgold_fmdrv_vidioc_queryctrl - enumerate control items */
static int xgold_fmdrv_vidioc_queryctrl(struct file *file, void *priv,
	struct v4l2_queryctrl *v4l2_qc)
{
	switch (v4l2_qc->id) {
	case V4L2_CID_AUDIO_MUTE:
		return v4l2_ctrl_query_fill(v4l2_qc, 0, 1, 1, 1);

	case V4L2_CID_AUDIO_VOLUME:
		return v4l2_ctrl_query_fill(v4l2_qc, 0, 100, 1, 0);
	}

	return -EINVAL;
}

/* xgold_fmdrv_vidioc_g_ctrl - get the value of a control */
static int xgold_fmdrv_vidioc_g_ctrl(struct file *file, void *priv,
	struct v4l2_control *ctrl)
{
	struct xgold_fmdev *fmdev = video_drvdata(file);
	int rc = 0;

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE: {
		struct fmrx_cfg *cfg = NULL;

		if (mutex_lock_interruptible(&fmdev->lock))
			return -EINTR;

		rc = fmrx_get_cfg(&cfg);
		if (rc != 0)
			goto xgold_fmdrv_vidioc_g_ctrl_err;

		mutex_unlock(&fmdev->lock);

		ctrl->value = cfg->mute;
	}
	break;

	case V4L2_CID_AUDIO_VOLUME: {
		u16 vol = 0;
		mutex_lock(&fmdev->lock);
		rc = fmrx_get_volume(FMRX_AUD_CHN_ALL, &vol);
		mutex_unlock(&fmdev->lock);
		ctrl->value = vol;
	}
	break;

	default: {
		fmdrv_err("Invalid arguments\n");
		return -EINVAL;
	}
	}

	return rc;

xgold_fmdrv_vidioc_g_ctrl_err:
	mutex_unlock(&fmdev->lock);
	return rc;
}

/* xgold_fmdrv_vidioc_s_ctrl - set the value of a control */
static int xgold_fmdrv_vidioc_s_ctrl(struct file *file, void *priv,
	struct v4l2_control *ctrl)
{
	struct xgold_fmdev *fmdev = video_drvdata(file);
	int rc = 0;

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_VOLUME:
		fmdrv_dbg("Volume level to set: %u\n", ctrl->value);

		mutex_lock(&fmdev->lock);

		/* Set volume level for both channels */
		rc = fmrx_set_volume_level(FMRX_AUD_CHN_ALL, ctrl->value);
		if (rc != 0) {
			fmdrv_warn("Failed to set volume level to %d\n",
				   ctrl->value);
			goto xgold_fmdrv_vidioc_s_ctrl_err;
		}

		mutex_unlock(&fmdev->lock);
		break;

	case V4L2_CID_AUDIO_MUTE:
		mutex_lock(&fmdev->lock);

		/* Set mute flag for all channels */
		rc = fmrx_set_mute(FMRX_AUD_CHN_ALL, ctrl->value);
		if (rc != 0) {
			fmdrv_warn("Failed to set volume level to %d\n",
				   ctrl->value);
			goto xgold_fmdrv_vidioc_s_ctrl_err;
		}

		mutex_unlock(&fmdev->lock);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;

xgold_fmdrv_vidioc_s_ctrl_err:
	mutex_unlock(&fmdev->lock);
	return rc;
}

/* xgold_fmdrv_vidioc_g_audio - get audio attributes */
static int xgold_fmdrv_vidioc_g_audio(struct file *file, void *priv,
	struct v4l2_audio *v4l2_aud)
{
	memset(v4l2_aud, 0, sizeof(*v4l2_aud));
	v4l2_aud->index = 0;
	strlcpy(v4l2_aud->name, XGOLD_FMDRV_CARD_SHORT_NAME,
		sizeof(v4l2_aud->name));
	v4l2_aud->capability = V4L2_AUDCAP_STEREO | V4L2_AUDCAP_AVL;
	v4l2_aud->mode = 0;

	return 0;
}

static int xgold_fmdrv_vidioc_s_audio(struct file *file, void *priv,
	const struct v4l2_audio *v4l2_aud)
{
	return 0;
}

static int xgold_fmdrv_vidioc_s_freq_seek(struct file *file, void *fh,
	const struct v4l2_hw_freq_seek *freq_seek)
{
	int rc = 0;
	struct fmrx_seek_station_params seek_station_info;
	struct xgold_fmdev *fmdev = video_drvdata(file);
	enum fmtrx_seek_dir seek_dir;

	if (freq_seek == NULL)
		return -EINVAL;

	if (freq_seek->tuner != 0 || freq_seek->type != V4L2_TUNER_RADIO) {
		rc = -EINVAL;
		fmdrv_err("Invalid arguments!\n");
		goto xgold_fmdrv_vidioc_s_freq_seek_err;
	}

	if (file->f_flags & O_NONBLOCK)
		return -EWOULDBLOCK;

	/* To prevent another seek request */
	if (FMTRX_HW_STATE_SEEK_ACTIVE == fmtrx_get_hw_state()) {
		fmdrv_err("Channel scanning is ongoing!. Ignoring request\n");
		return -EPERM;
	}

	/* Check if auto seek is requested */
	if (freq_seek->seek_upward)
		seek_dir = (freq_seek->wrap_around) ?
			FMRX_SEEK_UP : FMRX_SEEK_UP_TO_LIMIT;
	else
		seek_dir = (freq_seek->wrap_around) ?
			FMRX_SEEK_DOWN : FMRX_SEEK_DOWN_TO_LIMIT;

	seek_station_info.seek_mode = seek_dir;

	if (mutex_lock_interruptible(&fmdev->lock))
		rc = -EINTR;

	/* disable RDS in seeking mode */
	xgold_fmdrv_disable_rds(fmdev);

	rc = fmrx_station_seeking(&seek_station_info);
	mutex_unlock(&fmdev->lock);

	if (rc != 0) {
		fmdrv_err("Seeking failed - Seek dir: %d, rc : %d\n",
			  seek_dir, rc);
		goto xgold_fmdrv_vidioc_s_freq_seek_err;
	}

	fmdrv_info("Station seeking started with seek mode: %d\n", seek_dir);

	/* Block until seek operation is complete, based on the file open
	 * flags */
	if (!(file->f_flags & O_NONBLOCK)) {
		rc = wait_event_interruptible(fmdev->event_queue,
			(XGOLD_EVT_SEEK_COMPLETE == fmdev->event_type ||
			 XGOLD_EVT_TUNE_FAIL == fmdev->event_type));

		/* Set result to try again, if tune failed */
		if (XGOLD_EVT_TUNE_FAIL == fmdev->event_type)
			rc = -EAGAIN;

		fmdev->event_type = XGOLD_EVT_RADIO_READY;
	}

xgold_fmdrv_vidioc_s_freq_seek_err:
	return rc;
}

static long xgold_fmdrv_priv_ctrls(struct file *file, void *fh,
	bool valid_prio, unsigned int cmd, void *arg)
{
	long rc = -EIO;
	struct xgold_fmdev *fmdev = video_drvdata(file);

	if (NULL == fmdev) {
		fmdrv_warn("Invalid driver data\n");
		return -EIO;
	}

	switch ((enum v4l2_cid_private_xgold_t)cmd) {
	case V4L2_CID_PRIV_XGOLD_FMR_SET_ANTENNA: {
		enum fmtrx_ant_type *ant_type = (enum fmtrx_ant_type *)arg;

#ifdef CONFIG_IUI_FM_FMR
		struct fmrx_ch_info ch_info;
#endif

		if (likely(access_ok(VERIFY_READ, ant_type,
				     (sizeof(enum fmtrx_ant_type))))) {
			fmdrv_info("antenna requested: %d\n", *ant_type);

			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			rc = fmrx_switch_antenna(*ant_type);
			if (rc != 0) {
				fmdrv_err("Failed to set antenna type to %d\n",
					  *ant_type);
				goto xgold_fmdrv_priv_ctrls_err;
			}

			mutex_unlock(&fmdev->lock);

#ifdef CONFIG_IUI_FM_FMR
			rc = fmrx_get_channel_info(&ch_info);
			if (rc != 0) {
				fmdrv_err("Unable to get channel info\n");
				break;
			}

			xgold_fmdrv_fm_notify(ch_info.freq, ch_info.injside);
#endif
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_SET_OUTPUT_MODE: {
		enum fmrx_aud_route *aud_routing = (enum fmrx_aud_route *)arg;

		if (likely(access_ok(VERIFY_READ, aud_routing,
				     (sizeof(enum fmrx_aud_route))))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			rc = xgold_fmdrv_set_routing_mode(fmdev, *aud_routing);
			mutex_unlock(&fmdev->lock);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_GET_OUTPUT_MODE: {
		enum fmrx_aud_route *aud_routing = (enum fmrx_aud_route *)arg;

		if (likely(access_ok(VERIFY_WRITE, aud_routing,
				     (sizeof(enum fmrx_aud_route))))) {
			struct fmrx_cfg rx_state;

			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			rc = fmrx_get_rx_data_state(&rx_state);

			mutex_unlock(&fmdev->lock);

			if (rc != 0) {
				fmdrv_err("Unable to get reciever state\n");
				break;
			}

			*aud_routing = rx_state.aud_path;

			fmdrv_info("audio routing mode: %s\n",
				   (FMR_AUD_PATH_DAC == *aud_routing) ?
				   "DAC Mode" : "DSP Mode");
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_SET_SB: {
		enum fmrx_sb_inj_side *inj_side = (enum fmrx_sb_inj_side *)arg;

#ifdef CONFIG_IUI_FM_FMR
		struct fmrx_ch_info ch_info;
#endif

		if (likely(access_ok(VERIFY_READ, inj_side,
				     (sizeof(enum fmrx_sb_inj_side))))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			/* Set injection side */
			rc = fmrx_set_sideband_injection(
					(enum fmrx_sb_inj_side) *inj_side,
					FMR_SB_SEL_PERST);
			mutex_unlock(&fmdev->lock);

			if (rc != 0) {
				fmdrv_warn("Failed to set the sideband to %d\n",
					   *inj_side);
				break;
			}

#ifdef CONFIG_IUI_FM_FMR
			rc = fmrx_get_channel_info(&ch_info);
			if (rc != 0) {
				fmdrv_err("Unable to get channel info\n");
				break;
			}

			xgold_fmdrv_fm_notify(ch_info.freq, ch_info.injside);
#endif
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_SET_BAND: {
		struct fmtrx_band *fmrx_band = (struct fmtrx_band *)arg;

		if (likely(access_ok(VERIFY_READ, fmrx_band,
				     (sizeof(struct fmtrx_band))))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			/* Set band and de-emp */
			rc = fmrx_set_band(fmrx_band);
			if (rc != 0)
				fmdrv_warn("Failed to set Band/Deemp\n");

			mutex_unlock(&fmdev->lock);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_SET_SNC: {
		struct fmrx_snc *snc_cfg = (struct fmrx_snc *)arg;

		if (likely(access_ok(VERIFY_READ, snc_cfg,
				     (sizeof(struct fmrx_snc))))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			/* Set stereo noise cancellation configuration */
			rc = fmrx_set_snc(snc_cfg);
			if (rc != 0)
				fmdrv_warn("Failed to set the SNC config\n");

			mutex_unlock(&fmdev->lock);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_SET_SM: {
		struct fmrx_sm *sm_cfg = (struct fmrx_sm *)arg;

		if (likely(access_ok(VERIFY_READ, sm_cfg,
				     (sizeof(struct fmrx_sm))))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			/* Set Soft mute configuration */
			rc = fmrx_set_soft_mute_cfg(sm_cfg);
			if (rc != 0)
				fmdrv_warn("Failed to set the soft mute\n");

			mutex_unlock(&fmdev->lock);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_SET_GAIN_CTRL: {
		struct fmrx_agc *agc_params = (struct fmrx_agc *)arg;

		if (likely(access_ok(VERIFY_READ, agc_params,
				     (sizeof(struct fmrx_agc))))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			/* Set RSSI configuration */
			rc = fmrx_set_agc_gain_cfg(agc_params);
			if (rc != 0)
				fmdrv_warn("Failed to set the AGC cfg data\n");

			mutex_unlock(&fmdev->lock);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_SET_RSSI: {
		struct fmrx_rssi_subsc *rssi_prms =
			(struct fmrx_rssi_subsc *)arg;

		if (likely(access_ok(VERIFY_READ, rssi_prms,
				     sizeof(struct fmrx_rssi_subsc)))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			/* Set RSSI configuration */
			rc = fmrx_set_rssi(rssi_prms);
			if (rc != 0)
				fmdrv_warn(
					"Failed to set the RSSI cfg data\n");

			mutex_unlock(&fmdev->lock);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_SET_HWPARAMS: {
		struct fmr_hw_params *hw_params = (struct fmr_hw_params *)arg;

		if (likely(access_ok(VERIFY_READ, hw_params,
				     (sizeof(struct fmr_hw_params))))) {
			struct fmrx_cfg *cfg = NULL;

			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			/* Get static configuration */
			rc = fmrx_get_cfg(&cfg);
			if (rc != 0) {
				fmdrv_warn(
					"Failed to get the static cfg data\n");
				goto xgold_fmdrv_priv_ctrls_err;
			}

			/* Update only HW params */
			cfg->other_cfg.pn_thr = hw_params->pn_thr;

			if (hw_params->int_lna_gain_reduction >
				FMR_LOGAIN_0DB_REDUC) {
				rc = -EINVAL;
				fmdrv_warn("Invalid ext LNA gain\n");
				goto xgold_fmdrv_priv_ctrls_err;
			}

			cfg->other_cfg.lna_out_gain =
				hw_params->int_lna_gain_reduction;
			cfg->other_cfg.rssi_off_int =
				hw_params->rssi_offset_int_ant;
			cfg->other_cfg.rssi_off_ext =
				hw_params->rssi_offset_ext_ant;
			cfg->other_cfg.vol_ramp = hw_params->vol_ramping_cfg;
			cfg->other_cfg.clk_swt_rng =
				hw_params->clk_switch_range_104;

			/* Switch off the FMR macro */
			fmdrv_dbg("Powering off\n");
			rc = fmrx_power_off();
			if (rc != 0) {
				fmdrv_warn("Powering off failed\n");
				goto xgold_fmdrv_priv_ctrls_err;
			}

			/* Configure the driver */
			fmdrv_dbg("Re-configure the driver\n");
			rc = fmrx_set_cfg(cfg);
			if (rc != 0) {
				fmdrv_warn("Err in configuring the driver\n");
				rc = -EIO;
				goto xgold_fmdrv_priv_ctrls_err;
			}

			fmdrv_info("Configured driver successfully\n");

			/* Switch on the FMR macro */
			fmdrv_info("Powering on\n");
			rc = fmrx_power_on();
			if (rc != 0)
				fmdrv_warn("Powering on failed\n");

			mutex_unlock(&fmdev->lock);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_SET_LNA_CFG: {
		struct fmr_ext_lna_params *ext_lna_cfg =
			(struct fmr_ext_lna_params *)arg;
		rc = 0;

		if (likely(access_ok(VERIFY_READ, ext_lna_cfg,
				     (sizeof(struct fmr_ext_lna_params))))) {
			struct fmrx_ext_lna_cfg lna_cfg;
			enum fmtrx_ant_type ant_type = FMR_ANT_TYPE_END;

			if (ext_lna_cfg->ant_type >= FMRADIO_ANT_INVALID ||
			    ext_lna_cfg->lna_cfg.tab_len > MAX_FREQ_OFFS_LEN) {
				fmdrv_warn("Invalid ext LNA config\n");
				rc = -EINVAL;
				break;
			}

			if (ext_lna_cfg->ant_type == FMR_ANT_EXTERNAL)
				ant_type = FMR_ANT_HS_SE;
			else if (ext_lna_cfg->ant_type == FMR_ANT_INTERNAL)
				ant_type = FMR_ANT_EBD_DE;

			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			rc = fmrx_get_ext_lna_rssi_comp(&lna_cfg, ant_type);
			if (rc != 0) {
				fmdrv_warn("Failed to get the ext LNA RSSI\n");
				goto xgold_fmdrv_priv_ctrls_err;
			}

			lna_cfg.tab_len = ext_lna_cfg->lna_cfg.tab_len;

			if (NULL != lna_cfg.band_split &&
			    NULL != lna_cfg.offsets) {
				/* Band split parameter copy */
				memcpy(lna_cfg.band_split,
				       ext_lna_cfg->lna_cfg.band_split,
				       sizeof(lna_cfg.band_split));

				/* ext. LNA gain offsets copy */
				memcpy(lna_cfg.offsets,
				       ext_lna_cfg->lna_cfg.offsets,
				       sizeof(lna_cfg.offsets));
			} else {
				fmdrv_dbg("External LNA is not present\n");
				rc = -EIO;
				goto xgold_fmdrv_priv_ctrls_err;
			}

			rc = fmrx_set_ext_lna_rssi_comp(lna_cfg, ant_type);
			if (rc != 0) {
				fmdrv_warn(
				"Failed to set the ext LNA RSSI data\n");
				goto xgold_fmdrv_priv_ctrls_err;
			}

			mutex_unlock(&fmdev->lock);

			fmdrv_info("Ext LNA config is set successfully\n");
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_GET_LNA_CFG: {
		struct fmr_ext_lna_params *ext_lna_cfg =
			(struct fmr_ext_lna_params *)arg;
		rc = 0;

		if (likely(access_ok(VERIFY_WRITE, ext_lna_cfg,
				     (sizeof(struct fmr_ext_lna_params))))) {
			struct fmrx_ext_lna_cfg lna_cfg;
			enum fmtrx_ant_type ant_type = FMR_ANT_TYPE_END;

			if (FMR_ANT_EXTERNAL == ext_lna_cfg->ant_type) {
				ant_type = FMR_ANT_HS_SE;
			} else if (FMR_ANT_INTERNAL == ext_lna_cfg->ant_type) {
				ant_type = FMR_ANT_EBD_DE;
			} else {
				rc = -EINVAL;
				break;
			}

			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			rc = fmrx_get_ext_lna_rssi_comp(&lna_cfg, ant_type);

			mutex_unlock(&fmdev->lock);
			if (rc != 0) {
				fmdrv_warn("Err - Getting the ext LNA cfg\n");
				break;
			}

			if (NULL == lna_cfg.band_split ||
			    NULL == lna_cfg.offsets) {
				fmdrv_dbg("External LNA is not present\n");
				rc = -EIO;
				ext_lna_cfg->lna_cfg.tab_len = 0;
				break;
			}

			ext_lna_cfg->lna_cfg.tab_len = lna_cfg.tab_len;

			memcpy(ext_lna_cfg->lna_cfg.band_split,
			       lna_cfg.band_split, sizeof(lna_cfg.band_split));

			memcpy(ext_lna_cfg->lna_cfg.offsets, lna_cfg.offsets,
			       sizeof(lna_cfg.offsets));
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_GET_STA_CFG: {
		struct fmrx_cfg *cfg = (struct fmrx_cfg *)arg;

		if (likely(access_ok(VERIFY_WRITE, cfg,
				     (sizeof(struct fmrx_cfg))))) {
			struct fmrx_cfg *drv_cfg = NULL;

			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			/* Get static configuration */
			rc = fmrx_get_cfg(&drv_cfg);
			mutex_unlock(&fmdev->lock);
			if (rc != 0) {
				fmdrv_warn("Error in the static cfg data\n");
				break;
			}

			memcpy(cfg, drv_cfg, sizeof(struct fmrx_cfg));
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	/* Get Reciever State data */
	case V4L2_CID_PRIV_XGOLD_FMR_GET_DYN_CFG: {
		struct fmrx_ch_info *ch_info = (struct fmrx_ch_info *)arg;

		if (likely(access_ok(VERIFY_WRITE, ch_info,
				     (sizeof(struct fmrx_ch_info))))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			rc = fmrx_get_channel_info(ch_info);
			mutex_unlock(&fmdev->lock);
			if (rc != 0)
				fmdrv_warn("Failed to get dynamic data\n");
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_GET_RDS_DATA: {
		struct fmtrx_rds_report *rds_user_rpt =
			(struct fmtrx_rds_report *)arg;
		struct xgold_fmrx_rds *rds = &fmdev->fmrx_rds;

		if (FMTRX_HW_STATE_RX_ACTIVE != fmtrx_get_hw_state()) {
			rc = -EPERM;
			break;
		}

		/* Turn on RDS mode, if it was disabled */
		if (atomic_read(&rds->rds_mode) == 0) {
			mutex_lock(&fmdev->lock);
			rc = xgold_fmdrv_enable_rds(fmdev);
			mutex_unlock(&fmdev->lock);

			if (rc != 0) {
				fmdrv_err("Err in subscribing RDS: %ld\n", rc);
				break;
			}
		}

		if (likely(access_ok(VERIFY_WRITE, rds_user_rpt,
				     (sizeof(struct fmtrx_rds_report))))) {
			if (likely(access_ok(VERIFY_WRITE,
					     rds_user_rpt->rpt_array,
					     rds_user_rpt->array_len))) {
				rc = xgold_fmdrv_xfer_rds_from_int_buff(file,
					rds_user_rpt);
				fmdrv_err("xfer RDS rc : %ld\n", rc);
			}
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_GET_VER_INFO: {
		struct fmr_version *fmr_info = (struct fmr_version *)arg;
		struct fmtrx_revision fmr_rev_info;

		if (likely(access_ok(VERIFY_WRITE, fmr_info,
				     (sizeof(struct fmr_version))))) {
			/* Below one should be replace with recieving active */
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			rc = fmrx_get_revisions(&fmr_rev_info);
			if (rc != 0) {
				fmdrv_warn("Failed to get, revision info\n");
				goto xgold_fmdrv_priv_ctrls_err;
			}

			sprintf(&fmr_info->man_id[0], "Intel");
			sprintf(&fmr_info->hw_id[0],  "Internal RTX");
			sprintf(&fmr_info->hw_ver[0], "HW: V %u",
				fmr_rev_info.hw_id);
			sprintf(&fmr_info->sw_ver[0], "SW: Ver %u  FW: Ver %u",
				fmr_rev_info.pkg_id, fmr_rev_info.fw_id);

			mutex_unlock(&fmdev->lock);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_REG_READ: {
		struct fmtrx_register_data *reg_info =
			(struct fmtrx_register_data *)arg;
		struct fmtrx_reg_data trx_reg_info;

		if (likely(access_ok(VERIFY_WRITE, reg_info,
				     (sizeof(struct fmtrx_register_data))))) {
			trx_reg_info.reg_addr = reg_info->reg_addr;
			trx_reg_info.reg_data = reg_info->reg_data;
			trx_reg_info.reg_type = FMTRX_32BIT_ACCESS;

			rc = fmtrx_reg_read(&trx_reg_info);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMR_REG_WRITE: {
		struct fmtrx_register_data *reg_info =
			(struct fmtrx_register_data *)arg;
		struct fmtrx_reg_data trx_reg_info;

		if (likely(access_ok(VERIFY_READ, reg_info,
				     sizeof(struct fmtrx_register_data)))) {
			trx_reg_info.reg_addr = reg_info->reg_addr;
			trx_reg_info.reg_data = reg_info->reg_data;
			trx_reg_info.reg_type = FMTRX_32BIT_ACCESS;

			rc = fmtrx_reg_write(&trx_reg_info);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMRX_SET_OFFSETS: {
		struct gain_offsets *gain_offs = (struct gain_offsets *)arg;

		if (likely(access_ok(VERIFY_READ, gain_offs,
				     sizeof(struct gain_offsets)))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			rc = fmtrx_set_gain_offsets(gain_offs);
			if (rc != 0)
				fmdrv_dbg("EC: %ld - Gain offsets\n", rc);
			mutex_unlock(&fmdev->lock);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMRX_GET_OFFSETS: {
		struct gain_offsets *gain_offs = (struct gain_offsets *)arg;

		if (likely(access_ok(VERIFY_WRITE, gain_offs,
				     sizeof(struct gain_offsets)))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			rc = fmtrx_get_gain_offsets(gain_offs);

			mutex_unlock(&fmdev->lock);

			if (rc != 0)
				fmdrv_dbg("EC: %ld -Getting gain offsets\n",
					  rc);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMRX_SET_RSSI_OFFSETS: {
		struct rssi_offsets *rssi_offs = (struct rssi_offsets *)arg;

		if (likely(access_ok(VERIFY_READ, rssi_offs,
				     sizeof(struct rssi_offsets)))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			rc = fmtrx_set_gain_rssi_offsets(rssi_offs);

			mutex_unlock(&fmdev->lock);

			if (rc != 0)
				fmdrv_dbg("EC: %ld -Setting RSSI offsets\n",
					  rc);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	case V4L2_CID_PRIV_XGOLD_FMRX_GET_RSSI_OFFSETS: {
		struct rssi_offsets *rssi_offs = (struct rssi_offsets *)arg;

		if (likely(access_ok(VERIFY_WRITE, rssi_offs,
				     sizeof(struct rssi_offsets)))) {
			if (mutex_lock_interruptible(&fmdev->lock))
				return -EINTR;

			rc = fmtrx_get_gain_rssi_offsets(rssi_offs);

			mutex_unlock(&fmdev->lock);

			if (rc != 0)
				fmdrv_dbg("EC: %ld -Getting RSSI offsets\n",
					  rc);
		} else {
			rc = -EFAULT;
			fmdrv_warn("Invalid args for IOCTL cmd: %u\n", cmd);
		}
	}
	break;

	default:
		rc = -ENOTTY;
		fmdrv_dbg("Inappropriate ioctl command : %u\n", cmd);
		break;
	}

	return rc;

xgold_fmdrv_priv_ctrls_err:
	mutex_unlock(&fmdev->lock);
	fmdrv_dbg("Returning from IOCTL\n");
	return rc;
}

/* v4l2_ioctl_ops - video device ioctl operations */
static const struct v4l2_ioctl_ops xgold_fmdrv_ioctl_ops = {
	.vidioc_querycap	= xgold_fmdrv_vidioc_querycap,
	.vidioc_g_tuner		= xgold_fmdrv_vidioc_g_tuner,
	.vidioc_s_tuner		= xgold_fmdrv_vidioc_s_tuner,
	.vidioc_g_audio		= xgold_fmdrv_vidioc_g_audio,
	.vidioc_s_audio		= xgold_fmdrv_vidioc_s_audio,
	.vidioc_g_frequency	= xgold_fmdrv_vidioc_g_frequency,
	.vidioc_s_frequency	= xgold_fmdrv_vidioc_s_frequency,
	.vidioc_queryctrl	= xgold_fmdrv_vidioc_queryctrl,
	.vidioc_g_ctrl		= xgold_fmdrv_vidioc_g_ctrl,
	.vidioc_s_ctrl		= xgold_fmdrv_vidioc_s_ctrl,
	.vidioc_s_hw_freq_seek	= xgold_fmdrv_vidioc_s_freq_seek,
	.vidioc_default		= xgold_fmdrv_priv_ctrls,
};

#ifdef CONFIG_IDI
static int xgold_probe_fmdrv(struct idi_peripheral_device *idi_per_dev,
	const struct idi_device_id *idi_dev_id)
{
	int rc = 0, i = 0;
	struct v4l2_device *v4l2_dev = NULL;
	struct idi_resource *idi_res = &idi_per_dev->resources;

	fmdrv_info("Device probe called\n");

	/* Allocate the FM device struct */
	xgold_fmdev = kzalloc(sizeof(*xgold_fmdev), GFP_KERNEL);
	if (NULL == xgold_fmdev) {
		rc = -ENOMEM;
		fmdrv_crit("Could not allocate driver data structure\n");
		goto xgold_fmdrv_probe_exit;
	}

	v4l2_dev = &xgold_fmdev->v4l2_dev;
	strlcpy(v4l2_dev->name, XGOLD_FMDRV_NAME, sizeof(v4l2_dev->name));

	dev_set_drvdata(&idi_per_dev->device, xgold_fmdev);
	video_set_drvdata(&xgold_fmdev->vdev, xgold_fmdev);

	fmdrv_info("V4l2 device registration\n");

	rc = v4l2_device_register(&idi_per_dev->device, v4l2_dev);
	if (rc) {
		v4l2_err(v4l2_dev, "Could not register v4l2_device\n");
		goto xgold_fmdrv_v4l2_dev_reg_err;
	}

	fmdrv_dbg("V4l2 device registration successful\n");

	strlcpy(xgold_fmdev->vdev.name, v4l2_dev->name,
		sizeof(v4l2_dev->name));
	xgold_fmdev->vdev.v4l2_dev = v4l2_dev;
	xgold_fmdev->vdev.fops = &xgold_fmdrv_fops;
	xgold_fmdev->vdev.ioctl_ops = &xgold_fmdrv_ioctl_ops;
	xgold_fmdev->vdev.release = video_device_release_empty;
	xgold_fmdev->vdev.debug = 0;

	if (video_register_device(&xgold_fmdev->vdev,
				  VFL_TYPE_RADIO, -1) < 0) {
		rc = -EINVAL;
		fmdrv_crit("Video device registration failed\n");
		goto xgold_fmdrv_vid_dev_reg_err;
	}

	fmdrv_dbg("Video device registration successful\n");

	/* Initialize wait queue for event read */
	init_waitqueue_head(&xgold_fmdev->event_queue);

	/* Initialize interface locking mutex */
	mutex_init(&xgold_fmdev->lock);

	/* IDI stuff */
	xgold_fmdev->idi_fmdev.fmr_dev = idi_per_dev;

	/* Get FMR HW RAM resource */
	xgold_fmdev->hw_ram = idi_get_resource_byname(idi_res, IORESOURCE_MEM,
		FMR_AGRAM_RES_NAME);
	if (!xgold_fmdev->hw_ram) {
		rc = -ENODEV;
		fmdrv_crit("Get HW RAM resource by name failed\n");
		goto xgold_fmdrv_req_io_err;
	}

	/* Get FMR Register space resource handle */
	xgold_fmdev->reg_space = idi_get_resource_byname(idi_res,
		IORESOURCE_MEM, FMR_REG_RES_NAME);
	if (xgold_fmdev->reg_space == NULL) {
		rc = -ENODEV;
		fmdrv_crit("Get register resource by name failed\n");
		goto xgold_fmdrv_vid_dev_reg_err;
	}

	/* Get FMR Register space resource handle */
	xgold_fmdev->xg_ram = idi_get_resource_byname(idi_res, IORESOURCE_MEM,
		FMR_XGRAM_RES_NAME);
	if (xgold_fmdev->xg_ram == NULL) {
		rc = -ENODEV;
		fmdrv_crit("Get register resource by name failed\n");
		goto xgold_fmdrv_vid_dev_reg_err;
	}

	/* Getting the IRQ lines */
	for (i = 0; i < ARRAY_SIZE(xgold_fmr_irq_names); i++) {
		struct resource *irq_rsrc;

		irq_rsrc = idi_get_resource_byname(idi_res, IORESOURCE_IRQ,
			xgold_fmr_irq_names[i]);

		if (irq_rsrc == NULL) {
			fmdrv_crit("Get %s resource by name failed\n",
				   xgold_fmr_irq_names[i]);
			rc = -EIO;
			goto xgold_fmdrv_req_io_err;
		}

		xgold_fmdev->irqs[i] = irq_rsrc->start;
	}

	rc = xgold_fmdrv_init_hld(xgold_fmdev);
	if (rc != 0) {
		fmdrv_warn("HLD init failed\n");
		goto xgold_fmdrv_req_io_err;
	}

#ifdef CONFIG_IUI_FM_FMR
	/* Register with Frequency manager */
	rc = iui_fm_register_mitigation_callback(IUI_FM_MACRO_ID_FMR,
						 fmr_fm_mitigation_cb);

	if (rc != 0) {
		fmdrv_err("IUI FM registration failed, Result %d\n", rc);
		rc = -EIO;
		goto xgold_fmdrv_req_fm_reg_err;
	}

	fmdrv_info("Registration with Frequency manager successful\n");
#endif

	atomic_set(&xgold_fmdev->dev_open_count, 0);

#ifdef CONFIG_IUI_FM_FMR
	init_timer(&fm_notify_timer);
	fm_notify_timer.function = xgold_fmdrv_notify_fm_cb;
#endif

	init_timer(&fmr_rds_dis_timer);
	fmr_rds_dis_timer.function = xgold_fmdrv_dis_rds_on_buff_full;
	fmr_rds_dis_timer.data = 0;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	rc = device_state_pm_set_class(&idi_per_dev->device,
		idi_per_dev->pm_platdata->pm_user_name);

	if (rc != 0) {
		fmdrv_err("FM Radio device PM registration failed\n");
		goto xgold_fmdrv_req_fm_reg_err;
	} else {
		fmdrv_info("FM Radio device PM registration successful\n");
	}
#endif

	xgold_fmdev->pm_state.en_handler =
		idi_peripheral_device_pm_get_state_handler(idi_per_dev,
							   "enable");

	if (xgold_fmdev->pm_state.en_handler == NULL) {
		rc = -EIO;
		fmdrv_err("Error in getting the \"enable\" PM handler\n");
		goto xgold_fmdrv_req_fm_reg_err;
	}

	xgold_fmdev->pm_state.dis_handler =
		idi_peripheral_device_pm_get_state_handler(idi_per_dev,
							   "disable");

	if (xgold_fmdev->pm_state.dis_handler == NULL) {
		rc = -EIO;
		fmdrv_err("Error in getting the \"disable\" PM handler\n");
		goto xgold_fmdrv_req_fm_reg_err;
	}

	xgold_fmdev->pm_state.en_int_clk_handler =
		idi_peripheral_device_pm_get_state_handler(idi_per_dev,
							   "enable_intclk");

	if (xgold_fmdev->pm_state.en_int_clk_handler == NULL) {
		rc = -EIO;
		fmdrv_err("Error - getting \"enable_intclk\" PM handler\n");
		goto xgold_fmdrv_req_fm_reg_err;
	}

	fmdrv_dbg("Module probe successful with result: %d\n", rc);

	return rc;

xgold_fmdrv_req_fm_reg_err:
	xgold_fmdrv_deinit_hld();

xgold_fmdrv_req_io_err:
	fmdrv_warn("IRQ request error\n");

xgold_fmdrv_vid_dev_reg_err:
	v4l2_device_unregister(&xgold_fmdev->v4l2_dev);
	fmdrv_warn("V4L2 reg error\n");

xgold_fmdrv_v4l2_dev_reg_err:
	kfree(xgold_fmdev);
	xgold_fmdev = NULL;
	fmdrv_warn("Device reg error\n");

xgold_fmdrv_probe_exit:
	fmdrv_warn("probe exit error\n");
	return rc;
}

static int xgold_remove_fmdrv(struct idi_peripheral_device *idi_per_dev)
{
	struct xgold_fmdev *fmdev = dev_get_drvdata(&idi_per_dev->device);
	int rc = 0;

	/* de-initialize HLD */
	rc = xgold_fmdrv_deinit_hld();
	if (rc != 0) {
		fmdrv_err("Couldn't de-init HLD\n");
		goto xgold_remove_fmdrv_err;
	}

	fmdrv_dbg("Video unregister\n");
	video_unregister_device(&fmdev->vdev);

	fmdrv_dbg("V4L2 unregister\n");
	v4l2_device_unregister(&fmdev->v4l2_dev);

	dev_set_drvdata(&idi_per_dev->device, NULL);

	/* de-register from Frequency manager */
#ifdef CONFIG_IUI_FM_FMR
	fmdrv_err("Frequency manager notification error code %d\n",
		  iui_fm_notify_frequency(IUI_FM_MACRO_ID_FMR, NULL));
#endif

	kfree(fmdev);
	fmdev = NULL;
	fmdrv_dbg("Module release successful\n");

xgold_remove_fmdrv_err:
	return rc;
}
#endif

static int xgold_fmdrv_suspend(struct device *dev)
{
	struct xgold_fmdev *fmdev = (struct xgold_fmdev *)dev_get_drvdata(dev);
	int rc = 0;
	struct fmrx_cfg rx_state;

	if (FMTRX_HW_STATE_RX_ACTIVE == fmtrx_get_hw_state()) {
		rc = fmrx_get_rx_data_state(&rx_state);

		if (rc != 0) {
			fmdrv_err("unable to get receiver state\n");
			goto xgold_fmdrv_suspend_err;
		}

		if (FMR_AUD_PATH_DAC == rx_state.aud_path) {
			fmr_sys_power_enable(true, false);

			/* delete the rds disable timer */
			del_timer_sync(&fmr_rds_dis_timer);
			xgold_fmdrv_disable_rds(fmdev);

			fmdrv_err("cannot be suspended in DAC mode\n");
		}
	} else {
		fmdrv_dbg("device is suspended\n");
	}

xgold_fmdrv_suspend_err:
	return rc;
}

static int xgold_fmdrv_resume(struct device *dev)
{

	/* check whether FMR device is active */
	if (FMTRX_HW_STATE_RX_ACTIVE == fmtrx_get_hw_state()) {
		fmr_sys_power_enable(true, true);

		fmdrv_dbg("device resumed\n");
	}

	return 0;
}

static const struct dev_pm_ops xgold_fmdrv_pm_ops = {
	/* for system suspend and resume */
	.suspend = xgold_fmdrv_suspend,
	.resume = xgold_fmdrv_resume,
};

#ifdef CONFIG_IDI
static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_VENDOR_ID_INTEL,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_FMR,
	},

	{ /* end: all zeroes */},
};

static struct idi_peripheral_driver xgold_fmdev_idi_pdrv = {
	.driver = {
		.owner = THIS_MODULE,
		.name = XGOLD_FMDRV_NAME,
		.pm = &xgold_fmdrv_pm_ops,
	},

	.p_type = IDI_FMR,
	.id_table = idi_ids,
	.probe = xgold_probe_fmdrv,
	.remove = xgold_remove_fmdrv,
};
#endif

/* module interface */
static int __init xgold_init_fmdrv(void)
{
	int rc = 0;
	fmdrv_info("Intel FM Radio driver version : %s\n", XGOLD_FMDRV_VER);

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	rc = device_state_pm_add_class(&fmr_pm_class);
	if (rc) {
		fmdrv_err("Error while adding FM Radio pm class\n");
		return rc;
	}
#endif

#ifdef CONFIG_IDI
	rc = idi_register_peripheral_driver(&xgold_fmdev_idi_pdrv);

	if (rc) {
		fmdrv_err("driver initialisation failed\n");
		rc = -ENODEV;
	} else {
		fmdrv_info("driver initialised successfully\n");
	}
#endif

	return rc;
}

static void __exit xgold_exit_fmdrv(void)
{
#ifdef CONFIG_IDI
	idi_unregister_peripheral_driver(&xgold_fmdev_idi_pdrv);
#endif
}

module_init(xgold_init_fmdrv);
module_exit(xgold_exit_fmdrv);

/* Module Info */
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("Intel Mobile Communications FM Radio Driver");
MODULE_VERSION(XGOLD_FMDRV_VER);
MODULE_LICENSE("GPL v2");
