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

/*
** ============================================================================
**
**                              MODULE DESCRIPTION
**
** ============================================================================
*/
/* This file contains system related functions needed by the HLD/LLD.
 * It includes OS interfaces like Timer, Queue, IDI, Interrupt, etc.
 */

/*
** ============================================================================
**
**                              INCLUDE STATEMENTS
**
** ============================================================================
*/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <spinlock.h>
#include <types.h>

#include <linux/idi/idi_interface.h>

#ifdef CONFIG_IUI_FM_FMR  /* Frequency manager interfaces */
#include <linux/fm/iui_fm.h>
#include <linux/timer.h>
#endif

#include <aud_app_fmr_sys.h>	/* System interfaces */
#include <aud_app_fmr_hld.h>
#include <fmdrv_xgold_sys.h>

/*
** ============================================================================
**
**                              LOCAL DECLARATIONS
**
** ============================================================================
*/
/* Holder for events */
struct fmtrx_int_ev_info {
	struct fmtrx_msg_params irq_event_params;
	u32 event_cnt;
};


#define XGOLD_FMDRV_FMR_INT		"FMRSWINT"
#define XGOLD_FMDRV_FMR_DED0_INT	"FMRDED0INT"
#define XGOLD_FMDRV_FMR_DED1_INT	"FMRDED1INT"

#define XGOLD_FMRX_FW	"fmr_rxmain.bin"
#define XGOLD_FMTX_FW	"fmr_txmain.bin"
#define FMTRX_RX_CFG	"fmr_rxnvm.bin"

#define XGOLD_FMR_SWINT_IRQ_LINE \
	(xgold_sys_fmdev->irqs[0]) /* FMR_INT */
#define XGOLD_FMR_DED0_IRQ_LINE \
	(xgold_sys_fmdev->irqs[1]) /* FMRX_EV_ONESHOT */
#define XGOLD_FMR_DED1_IRQ_LINE \
	(xgold_sys_fmdev->irqs[2]) /* FMRX_EV_TUNE  */

#define XGOLD_FMR_WAIT_EV	1
#define XGOLD_FMR_TRIG_EV	0
#define TIMEOUT_CMD_COMPLETE	5000

/*
** ============================================================================
**
**                               LOCAL DEFINITIONS
**
** ============================================================================
*/
static T_FMTRX_SYS_IRQ_HANDLER g_fmr_irq_handler; /* IRQ handler */

/* Events holder */
static struct fmtrx_int_ev_info fmr_int_event_info[FMR_I_EV_INVALID];
static atomic_t fmr_hisr_event_flag;

static struct fmtrx_sys_sim_msg *p_fmr_msg_list; /* messages list */
static struct completion idi_dma_write_sem;
static struct completion idi_dma_read_sem;
static struct timer_list xgold_fmdrv_generic_timer;
static struct xgold_fmdev *xgold_sys_fmdev;
static const struct firmware *fw_entry;

static wait_queue_head_t wait_msg;
static u8 fmr_cmd_wait_event = XGOLD_FMR_WAIT_EV;
static u8 fmr_cmd2_wait_event = XGOLD_FMR_WAIT_EV;
static struct fmrx_cfg *rx_cfg;

/*
** ============================================================================
**
**                       LOCAL FUNCTIONS DECLARATIONS
**
** ============================================================================
*/
/* Start initially with an empty queue
 * Returns: 0 (Successfully); other (Unsuccessfully)
 */
static s32 fmr_sys_sim_msg_init_queue(void)
{
	s32 rc = 0;

	/* initialize the messages list */
	p_fmr_msg_list = kmalloc(sizeof(*p_fmr_msg_list), GFP_KERNEL);

	if (NULL == p_fmr_msg_list) {
		fmdrv_err("message list allocation failed\n");
		rc = -ENOMEM;
	}

	INIT_LIST_HEAD(&p_fmr_msg_list->list);

	return rc;
}

/* Pop out the oldest message to be served
 * If return NULL, means the queue is empty or unexpected error happened
 */
static struct fmtrx_sys_sim_msg *fmr_sys_sim_msg_pop_elem(void)
{
	/* Check if the queue is empty */
	if (!list_empty(&p_fmr_msg_list->list))
		return list_first_entry(&p_fmr_msg_list->list,
			struct fmtrx_sys_sim_msg, list);
	else
		return NULL; /* if queue is empty */
}

/* IRQ Handler */
static irqreturn_t cb_fmr_lisr_int_handler(int vector, void *context)
{
	enum fmtrx_int_vec fmr_vector = FMR_INT_SWINT;
	irqreturn_t rc = IRQ_HANDLED;

	if (vector == XGOLD_FMR_SWINT_IRQ_LINE)
		fmr_vector = FMR_INT_SWINT;
	else if (vector == XGOLD_FMR_DED0_IRQ_LINE)
		fmr_vector = FMR_INT_DED0;
	else if (vector == XGOLD_FMR_DED1_IRQ_LINE)
		fmr_vector = FMR_INT_DED1;
	else
		return IRQ_NONE;

	if (NULL != g_fmr_irq_handler)
		g_fmr_irq_handler(fmr_vector);

	if (atomic_read(&fmr_hisr_event_flag)) {
		rc = IRQ_WAKE_THREAD;
		atomic_set(&fmr_hisr_event_flag, 0);
	}

	return rc;
}

/* Handler for LISR */
static irqreturn_t fmr_highisr(int irq, void *context)
{
	u8 idx = 0;

	/* Process all recorded events */
	for (idx = 0; idx < FMR_I_EV_INVALID; idx++) {
		if (fmr_int_event_info[idx].event_cnt > 0) {
			fmr_int_event_info[idx].event_cnt = 0;
			fmr_sys_msg_send(
				&fmr_int_event_info[idx].irq_event_params);
		}
	}

	return IRQ_HANDLED;
}

/* Read from ABB */
static void cb_idi_read_func(struct idi_transaction *trans)
{
	complete(&idi_dma_read_sem);
}

/* Write to ABB */
static void cb_idi_write_func(struct idi_transaction *trans)
{
	complete(&idi_dma_write_sem);
}

static void fmr_sys_sim_msg_free_qitems(void)
{
	struct fmtrx_sys_sim_msg *mail = NULL;

	/* Free all the pending taks in the queue */
	while (NULL != (mail = fmr_sys_sim_msg_pop_elem())) {
		list_del(&mail->list);
		kfree(mail);
	}
}

void fmr_sys_sim_dispatcher(void)
{
	struct fmtrx_sys_sim_msg *mail = NULL;

	/* check if there are pending taks in the queue */
	while (NULL != (mail = fmr_sys_sim_msg_pop_elem())) {
		switch (mail->msg.fmr_mod_id) {
		case FMR_MODULE_FMRX: {
			union fmrx_ev_prams rx_event_params;
			memcpy(&rx_event_params, mail->msg.data,
			       sizeof(union fmrx_ev_prams));
			fmrx_msg_dispatch(
				(enum fmrx_event) mail->msg.fmr_mod_msg_id,
				rx_event_params);
		}
		break;

		case FMR_MODULE_SYS: {
			switch (mail->msg.fmr_mod_msg_id) {
#ifdef CONFIG_IUI_FM_FMR
			case MODULE_SYS_FM: { /* Notify frequency manager */
				struct iui_fm_fmr_info fm_fmr_info;
				struct iui_fm_freq_notification fm_notify_data;

				memcpy(&fm_fmr_info, mail->msg.data,
				       sizeof(struct iui_fm_fmr_info));

				fm_notify_data.type =
					IUI_FM_FREQ_NOTIFICATION_TYPE_FMR;

				fm_notify_data.info.fmr_info = &fm_fmr_info;

				fmdrv_info(
					"Notify FM - freq: %u, inj side %d\n",
					fm_fmr_info.rx_freq,
					fm_fmr_info.inj_side);

				fmdrv_err("FM notification error code %d\n",
					  iui_fm_notify_frequency(
							IUI_FM_MACRO_ID_FMR,
							&fm_notify_data));
			}
			break;
#endif

			default:
			break;
			}
		}

		default:
			break;
		}

		/* point to next item in the queue */
		list_del(&mail->list);

		/* release the memory */
		kfree(mail);
		mail = NULL;
	}
}

int fmr_sys_msg_init(struct xgold_fmdev *g_xgold_fmdev)
{
	if (NULL != g_xgold_fmdev)
		xgold_sys_fmdev = g_xgold_fmdev;

	return fmr_sys_sim_msg_init_queue();
}

int fmr_sys_request_irq(void)
{
	int rc = 0;

	/* interrupt handler registration */
	rc = request_threaded_irq(XGOLD_FMR_SWINT_IRQ_LINE,
				cb_fmr_lisr_int_handler,
				fmr_highisr,
				IRQF_DISABLED,
				XGOLD_FMDRV_FMR_INT,
				(void *)xgold_sys_fmdev);
	if (rc < 0) {
		fmdrv_err("error in creating the lisr sw_irq: %d\n", rc);
		goto xgold_fmdrv_req_sw_irq_err;
	}

	rc = request_threaded_irq(XGOLD_FMR_DED0_IRQ_LINE,
				cb_fmr_lisr_int_handler,
				fmr_highisr,
				IRQF_DISABLED,
				XGOLD_FMDRV_FMR_DED0_INT,
				(void *)xgold_sys_fmdev);
	if (rc < 0) {
		fmdrv_err("error in creating the lisr ded0_irq: %d\n", rc);
		goto xgold_fmdrv_req_ded0_irq_err;
	}

	rc = request_threaded_irq(XGOLD_FMR_DED1_IRQ_LINE,
				cb_fmr_lisr_int_handler,
				fmr_highisr,
				IRQF_DISABLED,
				XGOLD_FMDRV_FMR_DED1_INT,
				(void *)xgold_sys_fmdev);
	if (rc < 0) {
		fmdrv_err("error in creating the lisr ded1_irq: %d\n", rc);
		goto xgold_fmdrv_req_ded1_irq_err;
	}

	return rc;

xgold_fmdrv_req_ded1_irq_err:
	free_irq(XGOLD_FMR_DED0_IRQ_LINE, (void *)xgold_sys_fmdev);

xgold_fmdrv_req_ded0_irq_err:
	free_irq(XGOLD_FMR_SWINT_IRQ_LINE, (void *)xgold_sys_fmdev);

xgold_fmdrv_req_sw_irq_err:
	return rc;
}

void fmr_sys_release_irq(void)
{
	free_irq(XGOLD_FMR_SWINT_IRQ_LINE, (void *)xgold_sys_fmdev);
	free_irq(XGOLD_FMR_DED0_IRQ_LINE,  (void *)xgold_sys_fmdev);
	free_irq(XGOLD_FMR_DED1_IRQ_LINE,  (void *)xgold_sys_fmdev);
}

/* Initializes all OS related structures */
int fmr_sys_init(void)
{
	int rc = 0;

	/* Create necessary events */
	init_completion(&idi_dma_read_sem);   /* IDI DMA read completion */
	init_completion(&idi_dma_write_sem);  /* IDI DMA write completion */

	init_waitqueue_head(&wait_msg);
	atomic_set(&fmr_hisr_event_flag, 0);

	return rc;
}

/* de-initializes all OS related structures */
void fmr_sys_deinit(void)
{
	/* free the message list */
	kfree(p_fmr_msg_list);

	/* delete timer */
	del_timer_sync(&xgold_fmdrv_generic_timer);
}

void fmr_sys_on_off_seq(enum fmr_onoff_seq seq)
{
	switch (seq) {
	case FMR_POWER_OFF:
		/* free the queue items during powering off */
		fmr_sys_sim_msg_free_qitems();

		if (rx_cfg != NULL) {
			kfree(rx_cfg);
			rx_cfg = NULL;
		}
		break;

	case FMR_POWER_ON:
	default:
		break;
	}

	return;
}

int fmr_sys_msg_send(const struct fmtrx_msg_params *const event_params)
{
	int rc = 0;
	struct fmtrx_sys_sim_msg *new_msg_node = NULL;

	if (NULL == event_params) {
		fmdrv_err("Invalid parameters\n");
		rc = -EINVAL;
		goto fmr_sys_msg_send_err;
	}

	/* Allocate memory space for storing the message item */
	new_msg_node = kzalloc(sizeof(*new_msg_node), GFP_KERNEL);

	if (NULL == new_msg_node) {
		fmdrv_crit("Could not allocate memory\n");
		rc = -ENOMEM;
	}

	memcpy(&new_msg_node->msg, event_params,
	       sizeof(struct fmtrx_msg_params));

	/* Insert the new item in the message list */
	list_add_tail(&(new_msg_node->list), &(p_fmr_msg_list->list));

fmr_sys_msg_send_err:
	return rc;
}

/* Saves the event information */
void fmr_sys_int_event_send(const enum fmtrx_int_ev fmtrx_int_evt_id,
	const struct fmtrx_msg_params fmtrx_int_params)
{
	fmr_int_event_info[fmtrx_int_evt_id].event_cnt++;
	fmr_int_event_info[fmtrx_int_evt_id].irq_event_params =
		fmtrx_int_params;
	atomic_inc(&fmr_hisr_event_flag);
}

/* Invoke power interfaces to request/release power demands */
int fmr_sys_power_enable(bool enable, bool idi_bus_required)
{
	int rc = -EIO;

#ifdef CONFIG_IDI
	struct device_state_pm_state *pm_handler = NULL;

	if (enable)
		pm_handler = xgold_sys_fmdev->pm_state.en_handler;
	else
		pm_handler = xgold_sys_fmdev->pm_state.dis_handler;

	if (NULL != pm_handler) {
		rc = idi_set_power_state(xgold_sys_fmdev->idi_fmdev.fmr_dev,
			(void *)pm_handler, idi_bus_required);
		if (0 != rc)
			fmdrv_err("IDI set power state failed!\n");
	} else {
		fmdrv_err("PM hanler is NULL!\n");
		rc = -EINVAL;
	}
#endif /* CONFIG_IDI */

	return rc;
}

/* Change clock from Main to Internal RF and vice versa */
void fmr_sys_clock_sel(enum fmtrx_clk_src clk_src)
{
	switch (clk_src) {
	case FMR_CLK_SEL_PLL:
		/* Change to main clock */
		fmr_sys_power_enable(true, true);
		break;

	case FMR_CLK_SEL_RF: {
#ifdef CONFIG_IDI
		int rc = 0;
		struct device_state_pm_state *pm_handler =
			xgold_sys_fmdev->pm_state.en_int_clk_handler;

		if (NULL == pm_handler) {
			fmdrv_err("Invalid power state pm_handler!\n");
			goto fmr_sys_clock_sel_err;
		}

		rc = idi_set_power_state(xgold_sys_fmdev->idi_fmdev.fmr_dev,
			(void *)pm_handler, true);
		if (0 != rc)
			fmdrv_err(
				"Error in set power state: enable_intclk!\n");
#endif
		}
		break;

	default:
		break;
	}

#ifdef CONFIG_IDI
fmr_sys_clock_sel_err:
#endif
	return;
}

void fmr_sys_idle_wait(u32 msecs)
{
	/* sleep timer */
	msleep((u32)msecs);
}

void fmr_sys_busy_wait(u32 usecs)
{
	/* Busy wait timer */
	udelay((unsigned long)usecs);
}

void cb_timer_complete_func(unsigned long data)
{
	fmtrx_sys_timer_cb timer_cb = (fmtrx_sys_timer_cb) data;

	if (NULL != timer_cb)
		timer_cb(0);
}

void fmr_sys_timer_schedule_cb(u32 time_ms, fmtrx_sys_timer_cb timer_cb)
{
	init_timer(&xgold_fmdrv_generic_timer);
	xgold_fmdrv_generic_timer.function = cb_timer_complete_func;
	xgold_fmdrv_generic_timer.data = (unsigned long)timer_cb;
	xgold_fmdrv_generic_timer.expires =
		jiffies + msecs_to_jiffies(time_ms);
	add_timer(&xgold_fmdrv_generic_timer);
}

void fmr_sys_trigger_event(enum fmtrx_trigger_events event_id)
{
	switch (event_id) {
	case FMR_IR_CMD_DONE:
		fmr_cmd_wait_event = XGOLD_FMR_TRIG_EV;
		break;

	case FMR_IR_CMD2_DONE:
		fmr_cmd2_wait_event = XGOLD_FMR_TRIG_EV;
		break;

	default:
		fmdrv_err("event trigger failed. Invalid event ID\n");
		return;
	}

	wake_up_interruptible(&wait_msg);
}

s32 fmr_sys_wait_for_event(enum fmtrx_trigger_events event_id)
{
	s32 rc = 0;

	switch (event_id) {
	case FMR_IR_CMD_DONE:
		rc = wait_event_interruptible_timeout(wait_msg,
			(fmr_cmd_wait_event != XGOLD_FMR_WAIT_EV),
			msecs_to_jiffies(TIMEOUT_CMD_COMPLETE));

		if (rc)
			fmr_cmd_wait_event = XGOLD_FMR_WAIT_EV;
		break;

	case FMR_IR_CMD2_DONE:
		rc = wait_event_interruptible_timeout(wait_msg,
			(fmr_cmd2_wait_event != XGOLD_FMR_WAIT_EV),
			msecs_to_jiffies(TIMEOUT_CMD_COMPLETE));

		if (rc)
			fmr_cmd2_wait_event = XGOLD_FMR_WAIT_EV;
		break;

	default:
		fmdrv_dbg("event wait failed. Invalid event ID\n");
		rc = -EINVAL;
	}

	if (rc >= 1)
		rc = 0;

	return rc;
}

void fmr_sys_irq_register(T_FMTRX_SYS_IRQ_HANDLER irq_handler)
{
	if (NULL != irq_handler)
		g_fmr_irq_handler = irq_handler;
	else
		g_fmr_irq_handler = NULL;
}

int fmr_sys_mem_read(void *buf_desc, void *src_offset, u32 size)
{
#ifdef CONFIG_IDI
	struct idi_transaction *trans_rx = NULL;
	struct fmtrx_rds_buff_t *buff_desc = buf_desc;
	int rc = 0;

	if (buf_desc == NULL || src_offset == NULL || size == 0) {
		fmdrv_dbg("Invalid arguments\n");
		rc = -EINVAL;
		goto fmr_sys_mem_read_err;
	}

	/* Allocate an IDI transaction */
	trans_rx = idi_alloc_transaction(GFP_KERNEL);
	if (NULL == trans_rx) {
		rc = -EIO;
		fmdrv_err("IDI transaction allocation failed\n");
		goto fmr_sys_mem_read_err;
	}

	/* Configure the transaction handle */
	trans_rx->peripheral = xgold_sys_fmdev->idi_fmdev.fmr_dev;
	trans_rx->idi_xfer.cpu_base = buff_desc->buf;
	trans_rx->idi_xfer.base = buff_desc->dma_handle;
	trans_rx->idi_xfer.dst_addr =
		(u32)(xgold_sys_fmdev->hw_ram->start + src_offset);
	trans_rx->idi_xfer.size = size;
	trans_rx->complete = cb_idi_read_func;

	/* Open the software DMA channel */
	rc = idi_open_software_channel(trans_rx->peripheral);
	if (rc) {
		fmdrv_dbg("channel_map = %d and sw_channel = %d\n",
			  trans_rx->peripheral->channel_map,
			  trans_rx->peripheral->sw_channel);
		goto fmr_sys_mem_read_err2;
	}

	/* Read data from ABB */
	rc = idi_software_read(trans_rx->peripheral, trans_rx);
	if (rc) {
		fmdrv_err("IDI read failed\n");
		goto fmr_sys_mem_read_err2;
	}

	/* Wait for the completion of IDI read */
	if (!wait_for_completion_timeout(&idi_dma_read_sem,
					 msecs_to_jiffies(10000))) {
		fmdrv_err("Timeout while waiting for FMR read completion\n");
		rc = -EIO;
	}

fmr_sys_mem_read_err2:
	rc = idi_close_software_channel(trans_rx->peripheral);
	if (rc)
		fmdrv_err("software channel close failed\n");

	idi_free_transaction(trans_rx);
#endif  /* CONFIG_IDI */

fmr_sys_mem_read_err:
	return rc;
}

int fmr_sys_mem_write(u8 *dst, u8 *src, u32 size)
{
	int rc = 0;

#ifdef CONFIG_IDI
	struct idi_transaction *trans_tx = NULL;
	dma_addr_t write_dma_handle;
	u32 *writebuff = NULL;

	/* Allocate an IDI transaction */
	trans_tx = idi_alloc_transaction(GFP_KERNEL);
	if (NULL == trans_tx) {
		fmdrv_err("IDI alloc failed\n");
		goto fmr_sys_mem_write_err;
	}

	/* Allocate a non-cache 32-bit aligned memory */
	writebuff = dma_alloc_coherent(NULL, size, &write_dma_handle,
		GFP_KERNEL | GFP_DMA);
	if (NULL == writebuff) {
		fmdrv_err("couldn't allocate dma buffer\n");
		goto fmr_sys_mem_write_err1;
	}

	/* Copy data to the non-cached address */
	memcpy(writebuff, src, size);

	/* Configure the transaction handle */
	trans_tx->peripheral = xgold_sys_fmdev->idi_fmdev.fmr_dev;
	trans_tx->idi_xfer.cpu_base = writebuff;
	trans_tx->idi_xfer.base = write_dma_handle;
	trans_tx->idi_xfer.dst_addr = xgold_sys_fmdev->hw_ram->start;

	trans_tx->idi_xfer.size = size;
	trans_tx->complete = cb_idi_write_func;

	/* Open the software DMA channel */
	rc = idi_open_software_channel(trans_tx->peripheral);
	if (rc) {
		fmdrv_dbg("channel_map = %d and sw_channel =%d\n",
			  trans_tx->peripheral->channel_map,
			  trans_tx->peripheral->sw_channel);
		goto fmr_sys_mem_write_err2;
	}

	/* Write the data to ABB */
	rc = idi_software_write(trans_tx->peripheral, trans_tx);
	if (rc) {
		fmdrv_err("IDI write failed");
		goto fmr_sys_mem_write_err3;
	}

	/* Wait for the completion of IDI write */
	if (!wait_for_completion_timeout(&idi_dma_write_sem,
					 msecs_to_jiffies(10000)))
		fmdrv_err("Timeout while waiting for FMR write completion\n");

fmr_sys_mem_write_err3:
	/* Free up the allocated non-cached memory */
	if (NULL != writebuff)
		dma_free_coherent(NULL, size, writebuff, write_dma_handle);

fmr_sys_mem_write_err2:
	rc = idi_close_software_channel(trans_tx->peripheral);
	if (rc)
		fmdrv_err("close software channel failed\n");

fmr_sys_mem_write_err1:
	idi_free_transaction(trans_tx);

fmr_sys_mem_write_err:
#endif	/* CONFIG_IDI */

	return rc;
}

/* Fetch firmware from user space. */
int fmr_sys_fetch_fw(enum fmtrx_mod_type fw_type, const u8 **fw_data,
	u16 *fw_size)
{
	int rc = -EINVAL;

	rc = request_firmware(&fw_entry,
		(FMR_MODULE_FMRX == fw_type) ? XGOLD_FMRX_FW : XGOLD_FMTX_FW,
		&xgold_sys_fmdev->idi_fmdev.fmr_dev->device);

	if (rc != 0 || fw_entry->data == NULL || fw_entry->size == 0) {
		fmdrv_err("FW fetch failed - rc: %d\n", rc);
		return -EIO;
	}

	if ((NULL != fw_data) && (NULL != fw_size)) {
		*fw_data = fw_entry->data;
		*fw_size = fw_entry->size;
	}

	return rc;
}

/* Release firmware fetched from user space */
void fmr_sys_release_fw(enum fmtrx_mod_type fw_type)
{
	release_firmware(fw_entry);
}

/* Read 16bit value from ABB FMR registers via IDI */
void fmr_sys_reg_read16(u32 addr, u16 *data)
{
	if (0 != addr)
		*data = ioread16((void __iomem *)(addr +
			(u32) xgold_sys_fmdev->ctrl_io));
}

/* Read 32bit value from ABB FMR registers via IDI */
void fmr_sys_reg_read32(u32 addr, u32 *data)
{
	if (0 != addr)
		*data = ioread32((void __iomem *)(addr +
			(u32) xgold_sys_fmdev->ctrl_io));
}

/* Write 16bit value to ABB FMR registers via IDI */
void fmr_sys_reg_write16(u32 addr, const u16 data)
{
	if (0 != addr)
		iowrite16(data, (void __iomem *)(addr +
			(u32) xgold_sys_fmdev->ctrl_io));
}

/* Write 32bit value to ABB FMR registers via IDI */
void fmr_sys_reg_write32(u32 addr, const u32 data)
{
	if (0 != addr)
		iowrite32(data, (void __iomem *)(addr +
			(u32) xgold_sys_fmdev->ctrl_io));
}

int fmr_sys_get_cfg(struct fmrx_cfg **cfg)
{
	int rc = 0;
	const struct firmware *fw = NULL;

	rx_cfg = kzalloc(sizeof(*rx_cfg), GFP_KERNEL);
	if (rx_cfg == NULL) {
		rc = -ENOMEM;
		*cfg = NULL;
		fmdrv_crit("Could not allocate cfg data structure\n");
		goto fmr_sys_get_cfg_err;
	}

	rc = request_firmware(&fw, FMTRX_RX_CFG,
			      &xgold_sys_fmdev->idi_fmdev.fmr_dev->device);

	if (rc != 0) {
		fmdrv_err("Configuration file: %s is missing\n", FMTRX_RX_CFG);
		rc = -EIO;
	}

	if (rc == 0) {
		if (fw->size != sizeof(struct fmrx_cfg)) {
			fmdrv_dbg("Cfg file mismatch\n");
			fmdrv_dbg("Expected: %u bytes. Recieved: %u bytes\n",
				  sizeof(struct fmrx_cfg), fw->size);
			rc = -EIO;
		}
	}

	if (rc == 0) {
		fmdrv_dbg("Using configuration data from config file\n");
		memcpy(rx_cfg, fw->data, fw->size);
	} else {
		struct rssi_offs rssi_offs_int_ant = {108000000, 108000000,
			108000000, 108000000, 108000000, 34, 0, 0, 0, 0, 0};

		struct rssi_offs rssi_offs_ext_ant = {108000000, 108000000,
			108000000, 108000000, 108000000, 0, 0, 0, 0, 0, 0};

		s16 ppf_offs_int_ant[MAX_OFFSETS] = {304, 296, 288, 82, 273,
			267, 258, 251, 243, 238, 230, 223, 217, 212, 206, 201};

		s16 ppf_offs_ext_ant[MAX_OFFSETS] = {310, 302, 295, 87, 280,
			272, 264, 257, 249, 243, 236, 230, 224, 217, 212, 208};

		s16 lna_offs_int_ant[MAX_OFFSETS] = {205, 202, 198, 96, 193,
			189, 187, 183, 179, 176, 172, 168, 164, 159, 155, 150};

		s16 lna_offs_ext_ant[MAX_OFFSETS] = {229, 225, 220, 15, 210,
			205, 202, 196, 192, 187, 182, 178, 172, 166, 158, 150};

		u16 cp_init_offs[MAX_OFFSETS] = {950, 850, 800, 750, 700, 650,
			600, 550, 500, 450, 450, 450, 350, 350, 300, 250};

		struct fmrx_ext_lna_cfg ext_lna_cfg_int_ant = {
			{82000000, 88000000, 94000000, 101000000, 108000000},
			{0, 0, 0, 0, 0, 0},
			6};

		struct fmrx_ext_lna_cfg ext_lna_cfg_ext_ant = {
			{82000000, 88000000, 94000000, 101000000, 108000000},
			{0, 0, 0, 0, 0, 0},
			6};

		fmdrv_err("Config from userspace failed. Using default cfg\n");

		/* Put as in xml file */
		rx_cfg->band.max  = 108 * MHZ;
		rx_cfg->band.min  = 875 * DMHZ;
		rx_cfg->band.step = 50000;
		rx_cfg->band.deem = FMR_50US;
		rx_cfg->force_mono = false;
		rx_cfg->antenna = FMR_ANT_HS_SE;
		rx_cfg->seek_rssi = 0; /* dBuV */
		rx_cfg->aud_path = FMR_AUD_PATH_DAC;
		rx_cfg->agc_cfg.en = true;
		rx_cfg->agc_cfg.gain_idx = AGC_GAIN_INDEX_7;
		rx_cfg->rssi_cfg.enable = false;
		rx_cfg->rssi_cfg.hi_thr = 40;
		rx_cfg->rssi_cfg.lo_thr = 10;
		rx_cfg->mute = false;
		rx_cfg->snc.en = true;
		rx_cfg->snc.lo_thr = 17;
		rx_cfg->snc.hi_thr = 40;
		rx_cfg->sm_cfg.en = true;
		rx_cfg->sm_cfg.thr = 20;
		rx_cfg->sm_cfg.step = 2000;
		rx_cfg->rf_force_sb = FMR_FORCE_NONE;
		rx_cfg->vol_cfg.left = 88;
		rx_cfg->vol_cfg.right = 88;
		rx_cfg->rds_cfg.mode = RDS_OFF;

		rx_cfg->other_cfg.pn_thr = 1000;
		rx_cfg->other_cfg.lna_out_gain = FMR_LOGAIN_0DB_REDUC;
		rx_cfg->other_cfg.vol_ramp = 60;
		rx_cfg->other_cfg.clk_swt_rng = 150 * KHZ;
		rx_cfg->other_cfg.rssi_off_int = 0;
		rx_cfg->other_cfg.rssi_off_ext = 0;

		memcpy(&rx_cfg->rssi_offs_int_ant, &rssi_offs_int_ant,
		       sizeof(struct rssi_offs));
		memcpy(&rx_cfg->rssi_offs_ext_ant, &rssi_offs_ext_ant,
		       sizeof(struct rssi_offs));

		memcpy(rx_cfg->lna_offs_int_ant, lna_offs_int_ant,
		       sizeof(lna_offs_int_ant));
		memcpy(rx_cfg->lna_offs_ext_ant, lna_offs_ext_ant,
		       sizeof(lna_offs_ext_ant));
		memcpy(rx_cfg->ppf_offs_int_ant, ppf_offs_int_ant,
		       sizeof(ppf_offs_int_ant));
		memcpy(rx_cfg->ppf_offs_ext_ant, ppf_offs_ext_ant,
		       sizeof(ppf_offs_ext_ant));

		memcpy(rx_cfg->cp_init_offs, cp_init_offs,
		       sizeof(cp_init_offs));

		memcpy(&rx_cfg->ext_lna_cfg[FMR_ANT_EBD_DE],
		       &ext_lna_cfg_int_ant, sizeof(struct fmrx_ext_lna_cfg));

		memcpy(&rx_cfg->ext_lna_cfg[FMR_ANT_HS_SE],
		       &ext_lna_cfg_ext_ant, sizeof(struct fmrx_ext_lna_cfg));

		rc = 0;
	}

	*cfg = rx_cfg;

	release_firmware(fw);

fmr_sys_get_cfg_err:
	return rc;
}
