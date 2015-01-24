/*
 * Intel Shared Transport Driver
 *
 * Copyright (C) 2013 -2014 Intel Corporation
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.,
 */

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/lbf_ldisc.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

#define RECEIVE_ROOM 4096
#define PROP_LNP_BT_GPIO_ENB "intel,bt-fmr-gpio-reset"

/*------ Forward Declaration-----*/
struct sk_buff *lbf_dequeue(void);
static int lbf_tty_write(struct tty_struct *tty, const unsigned char *data,
				int count);
static void lbf_enqueue(struct sk_buff *skb);
static void bt_rfkill_set_power(unsigned long blocked);
static void lbf_update_set_room(struct tty_struct *tty, signed int cnt);
static void lbf_ldisc_fw_download_complete(unsigned long);
static int lbf_ldisc_fw_download_init(void);
static unsigned int lbf_ldisc_get_read_cnt(void);

static struct st_proto_s lbf_st_proto[6] = {
		{ .chnl_id = INVALID, /* ACL */
		.hdr_len = INVALID, .offset_len_in_hdr = INVALID,
				.len_size = INVALID, .reserve = INVALID, },
		{ .chnl_id = INVALID, /* ACL */
		.hdr_len = INVALID, .offset_len_in_hdr = INVALID,
				.len_size = INVALID, .reserve = INVALID, },
		{ .chnl_id = HCI_ACLDATA_PKT, /* ACL */
		.hdr_len = HCI_ACL_HDR_SIZE, .offset_len_in_hdr = 3,
				.len_size = 2, .reserve = 8, },
		{ .chnl_id = HCI_SCODATA_PKT, /* SCO */
		.hdr_len = HCI_SCO_HDR_SIZE, .offset_len_in_hdr = 3,
				.len_size = 1, .reserve = 8, },
		{ .chnl_id = HCI_EVENT_PKT, /* HCI Events */
		.hdr_len = HCI_EVENT_HDR_SIZE, .offset_len_in_hdr = 2,
				.len_size = 1, .reserve = 8, },
		{ .chnl_id = HCI_EVENT_PKT, /* HCI Events */
		.hdr_len = HCI_EVENT_HDR_SIZE, .offset_len_in_hdr = 2,
				.len_size = 1, .reserve = 8, }, };

struct lbf_q_tx {
	struct sk_buff_head txq, tx_waitq;
	struct sk_buff *tx_skb;
	spinlock_t lock;
	struct mutex tx_wakeup;
	unsigned long tx_state;
	struct mutex writelock;
	int tbusy;
	int woke_up;
	struct mutex fwdownloadlock;
	struct tty_struct *tty;
	struct work_struct tx_wakeup_work;
} lbf_q_tx;

static struct lbf_q_tx *lbf_tx;
static struct fm_ld_drv_register *fm;


struct lbf_uart {

	const struct firmware *fw;
	unsigned int ld_installed;
	unsigned int lbf_rcv_state; /* Packet receive state information */
	unsigned int fmr_evt_rcvd;
	unsigned int lbf_ldisc_running;
	unsigned long rx_state;
	unsigned long rx_count;
	char *read_buf;
	int read_head;
	int read_cnt;
	int read_tail;
	wait_queue_t wait;
	wait_queue_head_t read_wait;
	struct mutex atomic_read_lock;
	spinlock_t tx_lock;
	spinlock_t tx_update_lock;
	struct tty_struct *tty;
	spinlock_t rx_lock;
	struct sk_buff *rx_skb;
	struct fm_ld_drv_register *pfm_ld_drv_register;
	unsigned short rx_chnl;
	unsigned short type;
	unsigned short bytes_pending;
	unsigned short len;
	struct wake_lock wakelock; /* Fixing System Sleep */

};

struct bcm_bt_lpm {
	unsigned int gpio_enable_bt;
	struct device *tty_dev;
} bt_lpm;

/* Static variable */

static int fw_isregister;
static int fw_download_state;
static unsigned int lbf_ldisc_running = INVALID;
static int bt_enable_state = DISABLE;

static inline void check_unthrottle(struct tty_struct *tty)
{
	if (tty->count)
		tty_unthrottle(tty);
}

static void reset_buffer_flags(struct tty_struct *tty)
{
	struct lbf_uart *lbf_ldisc;
	pr_debug("-> %s\n", __func__);
	lbf_ldisc = tty->disc_data;
	spin_lock(&lbf_ldisc->tx_lock);
	lbf_ldisc->read_head = lbf_ldisc->read_cnt = lbf_ldisc->read_tail = 0;
	spin_unlock(&lbf_ldisc->tx_lock);
	tty->receive_room = RECEIVE_ROOM;
	lbf_update_set_room(tty, 0);
	check_unthrottle(tty);

}

int lbf_tx_wakeup(struct lbf_uart *lbf_uart)
{
	struct sk_buff *skb;
	int len = 0;
	pr_debug("-> %s\n", __func__);

	mutex_lock(&lbf_tx->tx_wakeup);
	if (lbf_tx->tbusy) {
		lbf_tx->woke_up = 1;
		mutex_unlock(&lbf_tx->tx_wakeup);
		return len;
	}
	lbf_tx->tbusy = 1; /*busy.*/

check_again:
	lbf_tx->woke_up = 0;
	mutex_unlock(&lbf_tx->tx_wakeup);


	while ((skb = lbf_dequeue())) {

		if (lbf_tx->tty) {
			set_bit(TTY_DO_WRITE_WAKEUP, &lbf_tx->tty->flags);
			len = lbf_tty_write((void *) lbf_tx->tty, skb->data,
					skb->len);
			skb_pull(skb, len);

			if (skb->len) {
				lbf_tx->tx_skb = skb;
				pr_debug("-> %s added to pending buffer:%d\n",
					__func__, skb->len);
				break;
			}
			kfree_skb(skb);
		}
	}

	mutex_lock(&lbf_tx->tx_wakeup);
	if (lbf_tx->woke_up)
		goto check_again;
	lbf_tx->tbusy = 0; /* Done with Tx.*/
	mutex_unlock(&lbf_tx->tx_wakeup);

	pr_debug("<- %s\n", __func__);


	return len;
}

/* This is the internal write function - a wrapper
 * to tty->ops->write
 */
int lbf_tty_write(struct tty_struct *tty, const unsigned char *data, int count)
{
	int len;
	pr_debug("-> %s\n", __func__);
	len = tty->ops->write(tty, data, count);
	pr_debug("<- %s\n", __func__);
	return  len;
}

static void lbf_tx_wakeup_work(struct work_struct *work)
{
	lbf_tx_wakeup(NULL);
}

long lbf_write(struct sk_buff *skb)
{
	long len = skb->len;
	pr_debug("-> %s\n", __func__);

	lbf_enqueue(skb);
	lbf_tx_wakeup(NULL);
	pr_debug("<- %s\n", __func__);

	return len;
}

/* for protocols making use of shared transport */

long unregister_fmdrv_from_ld_driv(struct fm_ld_drv_register *fm_ld_drv_reg)
{
	unsigned long flags = 0;
	spin_lock_irqsave(&lbf_tx->lock, flags);
	fw_isregister = 0;
	spin_unlock_irqrestore(&lbf_tx->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(unregister_fmdrv_from_ld_driv);

long register_fmdrv_to_ld_driv(struct fm_ld_drv_register *fm_ld_drv_reg)
{
	unsigned long flags;
	long err = 0;
	fm = NULL;
	pr_debug("-> %s\n", __func__);

	if (lbf_ldisc_running == 1) {

		if (fm_ld_drv_reg == NULL || fm_ld_drv_reg->fm_cmd_handler
				== NULL
				|| fm_ld_drv_reg->ld_drv_reg_complete_cb
						== NULL) {
			pr_debug("fm_ld_drv_register reg_complete_cb not ready");
			return -EINVAL;
		}
		spin_lock_irqsave(&lbf_tx->lock, flags);
		fm_ld_drv_reg->fm_cmd_write = lbf_write;
		fm = fm_ld_drv_reg;
		fw_isregister = 1;
		spin_unlock_irqrestore(&lbf_tx->lock, flags);

		if (fm) {
			if (likely(fm->ld_drv_reg_complete_cb != NULL)) {
				fm->ld_drv_reg_complete_cb(fm->priv_data,
						0);
				pr_debug("Registration called");

			}
		}
	} else
		err = -EINVAL;

	return err;
}
EXPORT_SYMBOL_GPL(register_fmdrv_to_ld_driv);

static void lbf_ldisc_flush_buffer(struct tty_struct *tty)
{
	/* clear everything and unthrottle the driver */

	unsigned long flags;
	pr_debug("-> %s\n", __func__);
	reset_buffer_flags(tty);

	if (tty->link) {
		spin_lock_irqsave(&tty->ctrl_lock, flags);
		if (tty->link->packet) {
			tty->ctrl_status |= TIOCPKT_FLUSHREAD;
			wake_up_interruptible(&tty->link->read_wait);
		}
		spin_unlock_irqrestore(&tty->ctrl_lock, flags);
	}
	pr_debug("<- %s\n", __func__);
}

/**
 * lbf_dequeue - internal de-Q function.
 * If the previous data set was not written
 * completely, return that skb which has the pending data.
 * In normal cases, return top of txq.
 */
struct sk_buff *lbf_dequeue(void)
{
	struct sk_buff *returning_skb;

	if (lbf_tx->tx_skb != NULL) {
		returning_skb = lbf_tx->tx_skb;
		lbf_tx->tx_skb = NULL;
		return returning_skb;
	}
	return skb_dequeue(&lbf_tx->txq);
}

/**
 * lbf_enqueue - internal Q-ing function.
 * Will either Q the skb to txq or the tx_waitq
 * depending on the ST LL state.
 * txq needs protection since the other contexts
 * may be sending data, waking up chip.
 */
void lbf_enqueue(struct sk_buff *skb)
{
	mutex_lock(&lbf_tx->writelock);
	skb_queue_tail(&lbf_tx->txq, skb);
	mutex_unlock(&lbf_tx->writelock);
}

/* lbf_ldisc_open
 *
 * Called when line discipline changed to HCI_UART.
 *
 * Arguments:
 * tty pointer to tty info structure
 * Return Value:
 * 0 if success, otherwise error code
 */
static int lbf_ldisc_open(struct tty_struct *tty)
{

	struct lbf_uart *lbf_uart;
	pr_info("-> %s\n", __func__);
	if (tty->disc_data) {
		pr_debug("%s ldiscdata exist\n ", __func__);
		return -EEXIST;
	}
	/* Error if the tty has no write op instead of leaving an exploitable
	 hole */
	if (tty->ops->write == NULL) {
		pr_debug("%s write = NULL\n ", __func__);
		return -EOPNOTSUPP;
	}

	lbf_uart = kzalloc(sizeof(struct lbf_uart), GFP_KERNEL);
	if (!lbf_uart) {
		pr_debug(" kzalloc for lbf_uart failed\n ");
		tty_unregister_ldisc(N_INTEL_LDISC);
		return -ENOMEM;
	}

	lbf_tx = kzalloc(sizeof(struct lbf_q_tx), GFP_KERNEL);
	if (!lbf_tx) {
		pr_debug(" kzalloc for lbf_tx failed\n ");
		kfree(lbf_uart);

		tty_unregister_ldisc(N_INTEL_LDISC);
		return -ENOMEM;
	}

	wake_lock_init(&lbf_uart->wakelock, WAKE_LOCK_SUSPEND, "wakelockname");
	wake_lock(&lbf_uart->wakelock);
	tty->disc_data = lbf_uart;
	lbf_uart->tty = tty;
	lbf_tx->tty = tty;

	/* don't do an wakeup for now */
	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

	if (likely(!lbf_uart->read_buf)) {
		lbf_uart->read_buf
				= kzalloc(N_INTEL_LDISC_BUF_SIZE, GFP_KERNEL);

		if (!lbf_uart->read_buf) {
			kfree(lbf_uart);

			kfree(lbf_tx);

			tty_unregister_ldisc(N_INTEL_LDISC);
			return -ENOMEM;
		}
	}

	tty->receive_room = RECEIVE_ROOM;

	skb_queue_head_init(&lbf_tx->txq);
	skb_queue_head_init(&lbf_tx->tx_waitq);
	spin_lock_init(&lbf_uart->rx_lock);
	spin_lock_init(&lbf_tx->lock);
	mutex_init(&lbf_tx->tx_wakeup);
	INIT_WORK(&lbf_tx->tx_wakeup_work, lbf_tx_wakeup_work);
	spin_lock_init(&lbf_uart->tx_lock);
	spin_lock_init(&lbf_uart->tx_update_lock);

	mutex_init(&lbf_tx->writelock);
	mutex_init(&lbf_tx->fwdownloadlock);

	init_waitqueue_head(&lbf_uart->read_wait);
	mutex_init(&lbf_uart->atomic_read_lock);
	lbf_ldisc_running = 1;
	/* Flush any pending characters in the driver and line discipline. */
	reset_buffer_flags(tty);

	if (tty->ldisc->ops->flush_buffer)
		tty->ldisc->ops->flush_buffer(tty);
	tty_driver_flush_buffer(tty);

	pr_debug("<- %s\n", __func__);

	return 0;
}

/* lbf_ldisc_close()
 *
 * Called when the line discipline is changed to something
 * else, the tty is closed, or the tty detects a hangup.
 */

static void lbf_ldisc_close(struct tty_struct *tty)
{
	struct lbf_uart *lbf_uart = (void *) tty->disc_data;
	tty->disc_data = NULL; /* Detach from the tty */
	pr_info("-> %s\n", __func__);
	flush_work(&lbf_tx->tx_wakeup_work);
	skb_queue_purge(&lbf_tx->txq);
	skb_queue_purge(&lbf_tx->tx_waitq);
	lbf_uart->rx_count = 0;
	fw_download_state = 0;
	lbf_uart->rx_state = LBF_W4_H4_HDR;

	kfree_skb(lbf_uart->rx_skb);
	kfree(lbf_uart->read_buf);
	lbf_uart->rx_skb = NULL;
	lbf_ldisc_running = -1;
	bt_rfkill_set_power(DISABLE);

	wake_unlock(&lbf_uart->wakelock);
	wake_lock_destroy(&lbf_uart->wakelock);
	kfree(lbf_tx);
	kfree(lbf_uart);
}

/* lbf_ldisc_wakeup()
 *
 * Callback for transmit wakeup. Called when low level
 * device driver can accept more send data.
 *
 * Arguments: tty pointer to associated tty instance data
 * Return Value: None
 */
static void lbf_ldisc_wakeup(struct tty_struct *tty)
{

	struct lbf_uart *lbf_uart = (void *) tty->disc_data;
	pr_debug("-> %s\n", __func__);

	clear_bit(TTY_DO_WRITE_WAKEUP, &lbf_uart->tty->flags);

	/*lbf_tx_wakeup(NULL);*/
	schedule_work(&lbf_tx->tx_wakeup_work);

	pr_debug("<- %s\n", __func__);

}

/* select_proto()
 * Note the HCI Header type
 * Arguments : type: the 1st byte in the packet
 * Return Type: The Corresponding type in the array defined
 * for all headers
 */

static inline int select_proto(int type)
{
	return (type >= 2 && type < 6) ? type : INVALID;
}

/* st_send_frame()
 * push the skb received to relevant
 * protocol stacks
 * Arguments : tty pointer to associated tty instance data
 lbf_uart : Disc data for tty
 chnl_id : id to either BT or FMR
 * Return Type: int
 */
static int st_send_frame(struct tty_struct *tty, struct lbf_uart *lbf_uart)
{
	unsigned int i = 0;
	int ret = 0;
	int chnl_id = LBF_BT;
	unsigned char *buff;
	unsigned int opcode = 0;
	unsigned int count = 0;

	if (unlikely(lbf_uart == NULL || lbf_uart->rx_skb == NULL)) {
		pr_debug(" No channel registered, no data to send?");
		return ret;
	}
	buff = &lbf_uart->rx_skb->data[0];
	count = lbf_uart->rx_skb->len;
	STREAM_TO_UINT16(opcode, buff);
	pr_debug("opcode : 0x%x event code: 0x%x registered", opcode,
			 lbf_uart->rx_skb->data[1]);
	/* for (i = lbf_uart->rx_skb->len, j = 0; i > 0; i--, j++)
	 printk (KERN_ERR " --%d : 0x%x " ,j ,lbf_uart->rx_skb->data[j]);*/

	if (HCI_COMMAND_COMPLETE_EVENT == lbf_uart->rx_skb->data[1]) {
		switch (opcode) {
		case FMR_IRQ_CONFIRM:
		case FMR_SET_POWER:
		case FMR_READ:
		case FMR_WRITE:
		case FMR_SET_AUDIO:
		case FMR_TOP_READ:
		case FMR_TOP_WRITE:
			chnl_id = LBF_FM;
			pr_debug("Its FM event ");
			break;
		default:
			chnl_id = LBF_BT;
			pr_debug("Its BT event ");
			break;
		}
	}
	if (HCI_INTERRUPT_EVENT == lbf_uart->rx_skb->data[1]
			&& (lbf_uart->rx_skb->data[3] == FMR_DEBUG_EVENT))
		chnl_id = LBF_FM;

	if (chnl_id == LBF_FM) {
		if (likely(fm)) {
			if (likely(fm->fm_cmd_handler != NULL)) {
				if (unlikely(fm->fm_cmd_handler(fm->
					priv_data, lbf_uart->rx_skb) != 0))
					pr_debug("proto stack %d recv failed"
						, chnl_id);
				else
					ret = lbf_uart->rx_skb->len;
			}
		}
	else
		pr_debug("fm is NULL ");
	} else if (chnl_id == LBF_BT) {
		pr_debug("In buffer count %d readhead %d read_cnt: %d\n", count
			, lbf_uart->read_head, lbf_uart->read_cnt);
		spin_lock(&lbf_uart->tx_lock);
		i = min3((N_INTEL_LDISC_BUF_SIZE - lbf_uart->read_cnt),
				N_INTEL_LDISC_BUF_SIZE - lbf_uart->read_head,
				(int)lbf_uart->rx_skb->len);
		spin_unlock(&lbf_uart->tx_lock);
		memcpy(lbf_uart->read_buf + lbf_uart->read_head,
			 &lbf_uart->rx_skb->data[0], i);
		spin_lock(&lbf_uart->tx_lock);
		lbf_uart->read_head =
			(lbf_uart->read_head + i) & (N_INTEL_LDISC_BUF_SIZE-1);
		lbf_uart->read_cnt += i;
		spin_unlock(&lbf_uart->tx_lock);
		count = count - i;
		pr_debug("count: %d, i:%d read_cnt:%d\n", count, i,
					lbf_uart->read_cnt);

		if (unlikely(count)) {
			pr_debug(" Added in buffer inside count %d readhead %d\n"
				, count , lbf_uart->read_head);
			spin_lock(&lbf_uart->tx_lock);
			i = min3((N_INTEL_LDISC_BUF_SIZE - lbf_uart->read_cnt),
				(N_INTEL_LDISC_BUF_SIZE - lbf_uart->read_head),
				(int)count);
			spin_unlock(&lbf_uart->tx_lock);
			memcpy(lbf_uart->read_buf + lbf_uart->read_head,
			&lbf_uart->rx_skb->data[lbf_uart->rx_skb->len - count],
				i);
			spin_lock(&lbf_uart->tx_lock);
			lbf_uart->read_head =
			(lbf_uart->read_head + i) & (N_INTEL_LDISC_BUF_SIZE-1);
			lbf_uart->read_cnt += i;
			spin_unlock(&lbf_uart->tx_lock);

		}
	}

	if (lbf_uart->rx_skb) {
		kfree_skb(lbf_uart->rx_skb);
		lbf_uart->rx_skb = NULL;
	}
	lbf_uart->rx_state = LBF_W4_H4_HDR;
	lbf_uart->rx_count = 0;
	lbf_uart->rx_chnl = 0;
	lbf_uart->bytes_pending = 0;

	return ret;
}

/* lbf_ldisc_fw_download_init()
 * lock the Mutex to block the IOCTL on 2nd call
 * Return Type: void
 */
static int lbf_ldisc_fw_download_init(void)
{
	unsigned long ret = -1;
	pr_debug("->%s\n", __func__);
	if (mutex_lock_interruptible(&lbf_tx->fwdownloadlock))
		return -ERESTARTSYS;
	pr_debug("->%s fw_download_state : %d\n", __func__ , fw_download_state);
	if (bt_enable_state == DISABLE) {
		bt_rfkill_set_power(ENABLE);
		ret = DO_FW_DL;
	} else {
		if (fw_download_state == FW_FAILED)
			ret = FW_FAILED;
		else if (fw_download_state == FW_SUCCESS)
			ret = DO_STACK_INIT;
	}
	pr_debug("<- %s\n", __func__);
	return ret;
}

/* lbf_ldisc_fw_download_complete()
 * unlock the Mutex to unblock the IOCTL on 2nd call
 * Return Type: void
 */
static void lbf_ldisc_fw_download_complete(unsigned long arg)
{
	if (arg == FW_SUCCESS)
		fw_download_state = FW_SUCCESS;
	else if (arg == FW_FAILED) {
		fw_download_state = FW_FAILED;
		bt_rfkill_set_power(DISABLE);
		bt_rfkill_set_power(ENABLE);
	}
	mutex_unlock(&lbf_tx->fwdownloadlock);
}

/* st_check_data_len()
 * push the skb received to relevant
 * protocol stacks
 * Arguments : tty pointer to associated tty instance data
 *lbf_uart : Disc data for tty
 *len : lenn of data
 * Return Type: void
 */

static inline int st_check_data_len(struct lbf_uart *lbf_uart, int len)
{
	int room = skb_tailroom(lbf_uart->rx_skb);

	if (!len) {
		/* Received packet has only packet header and
		 * has zero length payload. So, ask ST CORE to
		 * forward the packet to protocol driver (BT/FM/GPS)
		 */
		st_send_frame(lbf_uart->tty, lbf_uart);

	} else if (len > room) {
		/* Received packet's payload length is larger.
		 * We can't accommodate it in created skb.
		 */
		pr_debug("Data length is too large len %d room %d", len, room);
		kfree_skb(lbf_uart->rx_skb);
	} else {
		/* Packet header has non-zero payload length and
		 * we have enough space in created skb. Lets read
		 * payload data */

		lbf_uart->rx_state = LBF_W4_DATA;
		lbf_uart->rx_count = len;

		return len;
	}

	/* Change LDISC state to continue to process next
	 * packet */
	lbf_uart->rx_state = LBF_W4_H4_HDR;
	lbf_uart->rx_skb = NULL;
	lbf_uart->rx_count = 0;
	lbf_uart->rx_chnl = 0;
	lbf_uart->bytes_pending = 0;

	return 0;
}

/**
 * lbf_update_set_room - receive space
 * @tty: terminal
 *
 * Called by the driver to find out how much data it is
 * permitted to feed to the line discipline without any being lost
 * and thus to manage flow control. Not serialized. Answers for the
 * "instant".
 */

static void lbf_update_set_room(struct tty_struct *tty, signed int cnt)
{
	struct lbf_uart *lbf_uart = (struct lbf_uart *) tty->disc_data;
	int left;
	int old_left;

	spin_lock(&lbf_uart->tx_update_lock);
	left = tty->receive_room + cnt;

	if (left < 0)
		left = 0;
	old_left = tty->receive_room;
	tty->receive_room = left;
	spin_unlock(&lbf_uart->tx_update_lock);

	pr_debug("->%s, RcvRoom:%d cnt: %d\n", __func__, tty->receive_room, cnt);
	if (left && !old_left) {
		WARN_RATELIMIT(tty->port->itty == NULL,
				"scheduling with invalid itty\n");
		/* see if ldisc has been killed - if so, this means that
		 * even though the ldisc has been halted and ->buf.work
		 * cancelled, ->buf.work is about to be rescheduled
		 */
		WARN_RATELIMIT(test_bit(TTY_LDISC_HALTED, &tty->flags),
				"scheduling buffer work for halted ldisc\n");
		schedule_work(&tty->port->buf.work);
	}
}

static inline int packet_state(int exp_count, int recv_count)
{
	int status = 0;
	if (exp_count > recv_count)
		status = exp_count - recv_count;

	return status;
}

/* lbf_ldisc_receive()
 *
 * Called by tty low level driver when receive data is
 * available.
 *
 * Arguments: tty pointer to tty isntance data
 * data pointer to received data
 * flags pointer to flags for data
 * count count of received data in bytes
 *
 * Return Value: None
 */
static void lbf_ldisc_receive(struct tty_struct *tty, const u8 *cp, char *fp,
		int count)
{

	unsigned char *ptr;
	int proto = INVALID;
	int pkt_status;
	unsigned short payload_len = 0;
	int len = 0, type = 0, i = 0;
	unsigned char *plen;
	int st_ret = 0;
	struct lbf_uart *lbf_uart = (struct lbf_uart *) tty->disc_data;
	/*unsigned long flags;*/

	pr_debug("-> %s\n", __func__);
	print_hex_dump_debug(">in>", DUMP_PREFIX_NONE, 16, 1, cp, count,
			0);
	ptr = (unsigned char *) cp;
	if (unlikely(ptr == NULL) || unlikely(lbf_uart == NULL)) {
		pr_debug(" received null from TTY ");
		return;
	}

	lbf_update_set_room(tty, (-1)*count);

	/*spin_lock_irqsave(&lbf_uart->rx_lock, flags);*/

	pr_debug("-> %s count: %d lbf_uart->rx_state: %ld\n", __func__ , count,
		 lbf_uart->rx_state);

	while (count) {
		if (likely(lbf_uart->bytes_pending)) {
			len = min_t(unsigned int, lbf_uart->bytes_pending,
				 count);
			lbf_uart->rx_count = lbf_uart->bytes_pending;
			memcpy(skb_put(lbf_uart->rx_skb, len),
					ptr, len);
		} else if ((lbf_uart->rx_count)) {
			len = min_t(unsigned int, lbf_uart->rx_count, count);
			memcpy(skb_put(lbf_uart->rx_skb, len), ptr, len);
		}

		switch (lbf_uart->rx_state) {
		case LBF_W4_H4_HDR:
			if (*ptr == 0xFF) {
				pr_debug(" Discard a byte 0x%x\n" , *ptr);
				ptr++;
				count = count - 1;
				continue;
			}
			switch (*ptr) {
			case LL_SLEEP_ACK:
				pr_debug("PM packet");
				continue;
			case LL_WAKE_UP_ACK:
				pr_debug("PM packet");
				continue;
			default:
				type = *ptr;
				proto = select_proto(type);
				if (proto != INVALID) {
					lbf_uart->rx_skb = alloc_skb(1700,
						GFP_ATOMIC);
					if (!lbf_uart->rx_skb)
						return;
					lbf_uart->rx_chnl = proto;
					lbf_uart->rx_state = LBF_W4_PKT_HDR;
					lbf_uart->rx_count =
				lbf_st_proto[lbf_uart->rx_chnl].hdr_len + 1;
				} else {
					pr_debug("Invalid header type: 0x%x\n",
					 type);
					count = 0;
					break;
				}
				continue;
			}
			break;

		case LBF_W4_PKT_HDR:
			pkt_status = packet_state(lbf_uart->rx_count, count);
			pr_debug("%s: lbf_uart->rx_count %ld\n", __func__,
				lbf_uart->rx_count);
			count -= len;
			ptr += len;
			lbf_uart->rx_count -= len;
			if (pkt_status) {
				lbf_uart->bytes_pending = pkt_status;
				count = 0;
			} else {
				plen = &lbf_uart->rx_skb->data
			[lbf_st_proto[lbf_uart->rx_chnl].offset_len_in_hdr];
				lbf_uart->bytes_pending = pkt_status;
				if (lbf_st_proto[lbf_uart->rx_chnl].len_size
					== 1)
					payload_len = *(unsigned char *)plen;
				else if (lbf_st_proto[lbf_uart->rx_chnl].
						len_size == 2)
					payload_len =
					__le16_to_cpu(*(unsigned short *)plen);
				else
					pr_debug(
					"%s: invalid length for id %d\n",
					__func__, proto);

				st_check_data_len(lbf_uart, payload_len);
				}
			continue;

		case LBF_W4_DATA:
			pkt_status = packet_state(lbf_uart->rx_count, count);
			count -= len;
			ptr += len;
			i = 0;
			lbf_uart->rx_count -= len;
			if (!pkt_status) {
				pr_debug(
				"\n---------Complete pkt received----------\n");
				lbf_uart->rx_state = LBF_W4_H4_HDR;
				st_ret = st_send_frame(tty, lbf_uart);
				lbf_uart->bytes_pending = 0;
				lbf_uart->rx_count = 0;
				lbf_uart->rx_skb = NULL;
			} else {
				lbf_uart->bytes_pending = pkt_status;
				count = 0;
			}

			if (st_ret > 0)
				lbf_update_set_room(tty, st_ret);
			continue;

		} /* end of switch rx_state*/

	} /* end of while*/

	/*spin_unlock_irqrestore(&lbf_uart->rx_lock, flags);*/

	if (waitqueue_active(&lbf_uart->read_wait))
		wake_up_interruptible(&lbf_uart->read_wait);


	if (tty->receive_room < TTY_THRESHOLD_THROTTLE)
		tty_throttle(tty);

	pr_debug("<- %s\n", __func__);
}

/* lbf_ldisc_ioctl()
 *
 * Process IOCTL system call for the tty device.
 *
 * Arguments:
 *
 * tty pointer to tty instance data
 * file pointer to open file object for device
 * cmd IOCTL command code
 * arg argument for IOCTL call (cmd dependent)
 *
 * Return Value: Command dependent
 */
static int lbf_ldisc_ioctl(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{

	int err = 0;
	pr_debug("-> %s\n", __func__);
	switch (cmd) {
	case BT_FW_DOWNLOAD_INIT:
		err = lbf_ldisc_fw_download_init();
		break;
	case BT_FW_DOWNLOAD_COMPLETE:
		lbf_ldisc_fw_download_complete(arg);
		break;
	default:
		err = n_tty_ioctl_helper(tty, file, cmd, arg);
	}
	pr_debug("<- %s\n", __func__);
	return err;
}

/* lbf_ldisc_compatioctl()
 *
 * Process IOCTL system call for the tty device.
 *
 * Arguments:
 *
 * tty pointer to tty instance data
 * file pointer to open file object for device
 * cmd IOCTL command code
 * arg argument for IOCTL call (cmd dependent)
 *
 * Return Value: Command dependent
 */
static long lbf_ldisc_compat_ioctl(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{

	long err = 0;
	pr_debug("-> %s\n", __func__);
	switch (cmd) {
	case BT_FW_DOWNLOAD_INIT:
		pr_debug("-> %s BT_FW_DOWNLOAD_INIT\n", __func__);
		err = lbf_ldisc_fw_download_init();
		break;
	case BT_FW_DOWNLOAD_COMPLETE:
		pr_debug("-> %s BT_FW_DOWNLOAD_COMPLETE\n", __func__);
		lbf_ldisc_fw_download_complete(arg);
		break;
	default:
		err = n_tty_ioctl_helper(tty, file, cmd, arg);
	}
	pr_debug("<- %s\n", __func__);
	return err;
}

/* copy_from_read_buf()
 *
 * Internal function for lbf_ldisc_read().
 *
 * Arguments:
 *
 * tty pointer to tty instance data
 * buf buf to copy to user space
 * nr number of bytes to be read
 *
 * Return Value: number of bytes copy to user spcae succesfully
 */
static int copy_from_read_buf(struct tty_struct *tty,
		unsigned char __user **b,
		size_t *nr)
{
	int retval = 0;
	size_t n;
	struct lbf_uart *lbf_ldisc = (struct lbf_uart *)tty->disc_data;
	spin_lock(&lbf_ldisc->tx_lock);
	n = min3(lbf_ldisc->read_cnt,
		 (N_INTEL_LDISC_BUF_SIZE - lbf_ldisc->read_tail), (int)*nr);
	spin_unlock(&lbf_ldisc->tx_lock);
	if (n) {
		retval =
		copy_to_user(*b, &lbf_ldisc->read_buf[lbf_ldisc->read_tail], n);
		n -= retval;
		spin_lock(&lbf_ldisc->tx_lock);
		lbf_ldisc->read_tail =
		(lbf_ldisc->read_tail + n) & (N_INTEL_LDISC_BUF_SIZE-1);
		lbf_ldisc->read_cnt -= n;
		spin_unlock(&lbf_ldisc->tx_lock);
		*b += n;
	}
	return n;
}

static unsigned int lbf_ldisc_get_read_cnt(void)
{
	int read_cnt = 0;
	struct lbf_uart *lbf_ldisc = (struct lbf_uart *)lbf_tx->tty->disc_data;
	spin_lock(&lbf_ldisc->tx_lock);
	read_cnt = lbf_ldisc->read_cnt;
	spin_unlock(&lbf_ldisc->tx_lock);
	return read_cnt;
}

/* lbf_ldisc_read()
 *
 * Process read system call for the tty device.
 *
 * Arguments:
 *
 * tty pointer to tty instance datan
 * file pointer to open file object for device
 * buf buf to copy to user space
 * nr number of bytes to be read
 *
 * Return Value: number of bytes read copied to user space
 * succesfully
 */
static ssize_t lbf_ldisc_read(struct tty_struct *tty, struct file *file,
		unsigned char __user *buf, size_t nr)
{
	unsigned char __user *b = buf;
	ssize_t size;
	int retval = 0;
	int state = -1;
	struct lbf_uart *lbf_uart = (struct lbf_uart *)tty->disc_data;
	pr_debug("-> %s\n", __func__);
	init_waitqueue_entry(&lbf_uart->wait, current);

	if (!lbf_ldisc_get_read_cnt()) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else {
			if (mutex_lock_interruptible(
				&lbf_uart->atomic_read_lock))
				return -ERESTARTSYS;
			else
				state = 0;
			}
	}

	add_wait_queue(&lbf_uart->read_wait, &lbf_uart->wait);
	while (nr) {
		if (lbf_ldisc_get_read_cnt() > 0) {
			int copied;
			__set_current_state(TASK_RUNNING);
			/* The copy function takes the read lock and handles
			 locking internally for this case */
			copied = copy_from_read_buf(tty, &b, &nr);
			copied += copy_from_read_buf(tty, &b, &nr);
			if (copied) {
				retval = 0;
				if (lbf_uart->read_cnt <=
					TTY_THRESHOLD_UNTHROTTLE) {
					check_unthrottle(tty);
				}
			break;
			}
		} else {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			continue;
		}
	}

	if (state == 0)
		mutex_unlock(&lbf_uart->atomic_read_lock);


	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&lbf_uart->read_wait, &lbf_uart->wait);



	size = b - buf;
	if (size)
		retval = size;

	lbf_update_set_room(tty, retval);


	pr_debug("<- %s\n", __func__);
	return retval;
}

/* lbf_ldisc_write()
 *
 * Process Write system call from user Space.
 *
 * Arguments:
 *
 * tty pointer to tty instance data
 * file pointer to open file object for device
 * buf data to pass down
 * nr number of bytes to be written down
 *
 * Return Value: no of bytes written succesfully
 */
static ssize_t lbf_ldisc_write(struct tty_struct *tty, struct file *file,
		const unsigned char *data, size_t count)
{
	struct sk_buff *skb = alloc_skb(count + 1, GFP_ATOMIC);
	int len = 0;

	pr_debug("-> %s\n", __func__);

	print_hex_dump_debug("<BT<", DUMP_PREFIX_NONE, 16, 1, data,
			count, 0);

	if (!skb)
		return -ENOMEM;

	memcpy(skb_put(skb, count), data, count);
	len = count;

	lbf_enqueue(skb);

	lbf_tx_wakeup(NULL);

	pr_debug("<- %s\n", __func__);
	return len;
}

/* lbf_ldisc_poll()
 *
 * Process POLL system call for the tty device.
 *
 * Arguments:
 *
 * tty pointer to tty instance data
 * file pointer to open file object for device
 *
 * Return Value: mask of events registered with this poll call
 */
static unsigned int lbf_ldisc_poll(struct tty_struct *tty, struct file *file,
		poll_table *wait)
{
	unsigned int mask = 0;
	struct lbf_uart *lbf_uart = (struct lbf_uart *) tty->disc_data;

	pr_debug("-> %s\n", __func__);

	if (lbf_uart->read_cnt > 0)
		mask |= POLLIN | POLLRDNORM;
	else {
		poll_wait(file, &lbf_uart->read_wait, wait);
		if (lbf_uart->read_cnt > 0)
			mask |= POLLIN | POLLRDNORM;
		if (tty->packet && tty->link->ctrl_status)
			mask |= POLLPRI | POLLIN | POLLRDNORM;
		if (test_bit(TTY_OTHER_CLOSED, &tty->flags))
			mask |= POLLHUP;
		if (tty_hung_up_p(file))
			mask |= POLLHUP;
		if (tty->ops->write && !tty_is_writelocked(tty)
				&& tty_chars_in_buffer(tty) < 256
				&& tty_write_room(tty) > 0)
			mask |= POLLOUT | POLLWRNORM;
	}
	pr_debug("<- %s\n", __func__);
	return mask;
}

/* lbf_ldisc_set_termios()
 *
 * It notifies the line discpline that a change has been made to the
 * termios structure
 *
 * Arguments:
 *
 * tty pointer to tty instance data
 * file pointer to open file object for device
 *
 * Return Value: void
 */
static void lbf_ldisc_set_termios(struct tty_struct *tty, struct ktermios *old)
{
	unsigned int cflag;
	cflag = tty->termios.c_cflag;
	pr_debug("-> %s\n", __func__);

	if (cflag & CRTSCTS)
		pr_debug(" - RTS/CTS is enabled\n");
	else
		pr_debug(" - RTS/CTS is disabled\n");
}


static int intel_bluetooth_pdata_probe(struct platform_device *pdev)
{
	struct pinctrl_state *pins_default, *pins_sleep, *pins_inactive;
	struct device_node *dn = pdev->dev.of_node;
	struct pinctrl *pinctrl;

	pr_debug("%s\n", __func__);
	if (!dn) {
		pr_err("cann't find matching node\n");
		goto skip_pinctrl;
	}

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl))
		goto skip_pinctrl;

	pins_default = pinctrl_lookup_state(pinctrl,
						 PINCTRL_STATE_DEFAULT);
	if (IS_ERR(pins_default))
		pr_err("could not get default pinstate\n");

	pins_sleep = pinctrl_lookup_state(pinctrl,
					       PINCTRL_STATE_SLEEP);
	if (IS_ERR(pins_sleep))
		pr_err("could not get sleep pinstate\n");

	pins_inactive = pinctrl_lookup_state(pinctrl,
					       "inactive");
	if (IS_ERR(pins_inactive))
		pr_err("could not get inactive pinstate\n");

	bt_lpm.gpio_enable_bt = of_get_named_gpio_flags(dn,
					PROP_LNP_BT_GPIO_ENB, 0, NULL);
	if (bt_lpm.gpio_enable_bt <= 0) {
		pr_err("Can't find node %s\n", PROP_LNP_BT_GPIO_ENB);
		goto error;
	} else
		pr_info("bt_fmr %d\n", bt_lpm.gpio_enable_bt);

	return 0;
error:
skip_pinctrl:
	pr_err("%s: devm_pinctrl_get not valid\n", __func__);
	return 0;
}

static void bt_rfkill_set_power(unsigned long blocked)
{
	pr_debug("%s blocked: %lu\n", __func__, blocked);
	if (ENABLE == blocked) {
		gpio_set_value(bt_lpm.gpio_enable_bt, 1);
		pr_info("%s: turn BT on\n", __func__);
		bt_enable_state = ENABLE;
	} else if (DISABLE == blocked) {
		gpio_set_value(bt_lpm.gpio_enable_bt, 0);
		pr_info("%s: turn BT off\n", __func__);
		bt_enable_state = DISABLE;
	}
}


static int intel_lpm_bluetooth_probe(struct platform_device *pdev)
{
	int default_state = 0;	/* off */
	int ret = 0;

	pr_debug("%s\n", __func__);
	if (pdev == NULL) {
		ret = -EINVAL;
		goto err_data_probe;
	}
	else
		bt_lpm.tty_dev = &pdev->dev;

	ret = intel_bluetooth_pdata_probe(pdev);

	if (ret < 0) {
		pr_err("%s: Cannot register platform data\n", __func__);
		ret = -EINVAL;
		goto err_data_probe;
	}

	ret = gpio_request(bt_lpm.gpio_enable_bt, "bt-reset");
	if (ret < 0) {
		pr_err("%s: Unable to request gpio %d\n", __func__,
				bt_lpm.gpio_enable_bt);
		ret = -EINVAL;
		goto err_gpio_enable_req;
	} else
		pr_err("%s: succeded to request gpio\n", __func__);

	ret = gpio_direction_output(bt_lpm.gpio_enable_bt, 0);
	if (ret < 0) {
		pr_err("%s: Unable to set int direction for gpio %d\n",
			__func__, bt_lpm.gpio_enable_bt);
		ret = -EINVAL;
		goto err_gpio_enable_dir;
	} else
		pr_err("%s: succeded to request direction gpio\n", __func__);

	pr_err("OFF : %s: gpio_enable=%d\n", __func__, bt_lpm.gpio_enable_bt);

	bt_rfkill_set_power(default_state);

	return ret;

err_gpio_enable_dir:
	gpio_free(bt_lpm.gpio_enable_bt);
err_gpio_enable_req:
err_data_probe:
	return ret;
}

static int intel_lpm_bluetooth_remove(struct platform_device *pdev)
{
	pr_err("%s\n", __func__);
	gpio_free(bt_lpm.gpio_enable_bt);
	return 0;
}

/*Line discipline op*/
static struct tty_ldisc_ops lbf_ldisc = {
	.magic = TTY_LDISC_MAGIC,
	.name = "n_ld_intel",
	.open = lbf_ldisc_open,
	.close = lbf_ldisc_close,
	.read = lbf_ldisc_read,
	.write = lbf_ldisc_write,
	.ioctl = lbf_ldisc_ioctl,
	.compat_ioctl = lbf_ldisc_compat_ioctl,
	.poll = lbf_ldisc_poll,
	.receive_buf = lbf_ldisc_receive,
	.write_wakeup = lbf_ldisc_wakeup,
	.flush_buffer = lbf_ldisc_flush_buffer,
	.set_termios = lbf_ldisc_set_termios,
	.owner = THIS_MODULE
};

static struct of_device_id intel_bluetooth_of_match[] = {
	{.compatible = "intel,bt-fmr",},
	{},
};

static struct platform_driver intel_bluetooth_platform_driver = {
	.probe = intel_lpm_bluetooth_probe,
	.remove = intel_lpm_bluetooth_remove,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = "bcm_bt_lpm",
		.owner = THIS_MODULE,
		.of_match_table = intel_bluetooth_of_match,
		},
};

static int __init lbf_uart_init(void)
{
	int err = 0;

	pr_debug("-> %s\n", __func__);

	err = platform_driver_register(&intel_bluetooth_platform_driver);
	if (err) {
		pr_err("Registration failed. (%d)", err);
		goto err_platform_register;
	} else
		pr_info("%s: Plarform registration succeded\n", __func__);

	/* Register the tty discipline */

	err = tty_register_ldisc(N_INTEL_LDISC, &lbf_ldisc);
	if (err) {
		pr_err("Line Discipline Registration failed. (%d)", err);
		goto err_tty_register;
	} else
		pr_info("%s: N_INTEL_LDISC registration succeded\n", __func__);

	pr_debug("<- %s\n", __func__);

	return err;

err_tty_register:
	platform_driver_unregister(&intel_bluetooth_platform_driver);
err_platform_register:
	return err;
}

static void __exit lbf_uart_exit(void)
{
	int err;
	pr_info("-> %s\n", __func__);

	/* Release tty registration of line discipline */
	err = tty_unregister_ldisc(N_INTEL_LDISC);
	if (err)
		pr_err("Can't unregister N_INTEL_LDISC line discipline (%d)",
			 err);
	platform_driver_unregister(&intel_bluetooth_platform_driver);

	pr_debug("<- %s\n", __func__);
}

module_init(lbf_uart_init);
module_exit(lbf_uart_exit);
MODULE_LICENSE("GPL");
