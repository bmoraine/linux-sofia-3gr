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

#ifndef _IMC_DEBUG_H
#define _IMC_DEBUG_H

enum idi_events {
	reserved,		/*  0 */

	/*
	 * Bus events.
	 */
	client_drv_reg,		/*  1 */
	controller_dev_reg,	/*  2 */
	peripheral_drv_reg,	/*  3 */
	start_rx,		/*  4 */
	stop_rx,		/*  5 */

	/*
	 * Interface events
	 */
	outstanding_read,	/*  6 */
	read_client_reg,	/*  7 */
	write_client_reg,	/*  8 */
	register_irq,		/*  9 */
	unregister_irq,		/* 10 */
	mask_irq,		/* 11 */
	async_read,		/* 12 */
	async_write,		/* 13 */
	transaction_flush,	/* 14 */
	channel_config,		/* 15 */
	buffer_info,		/* 16 */
	buffer_flush,		/* 17 */
	queue_status,		/* 18 */
	open_sw_chan,		/* 19 */
	close_sw_chan,		/* 20 */
	sw_read,		/* 21 */
	sw_write,		/* 22 */
	set_state,		/* 22 */

	/*
	 * Controller events
	 */
	set_cfg,		/* 23 */
	ctrl_suspend,		/* 24 */
	ctrl_resume,		/* 25 */
	start_xfer,		/* 26 */
	queue_xfer,		/* 27 */
	complete_xfer,		/* 28 */
	xfer_fwd,		/* 29 */
	xfer_resume,		/* 30 */
	rx_break,		/* 31 */
	tx_break,		/* 32 */
	rx_error,		/* 33 */
	slave_irq,		/* 34 */

};

#ifdef CONFIG_IDI_DEBUG
extern void __idi_debug_init(void);
extern void __idi_debug_remove(void);
extern void __idi_debug_add_event(enum idi_events event,
				  enum idi_peripheral_type p_type,
				  struct idi_transaction *trans);

static inline void idi_debug_init(void)
{
	__idi_debug_init();
}

static inline void idi_debug_remove(void)
{
	__idi_debug_remove();
}

static inline void idi_debug_add_event(enum idi_events event,
				       enum idi_peripheral_type p_type,
				       struct idi_transaction *trans)
{
	__idi_debug_add_event(event, p_type, trans);
}

#else /* !CONFIG_IDI_DEBUG */

static inline void idi_debug_init(void)
{
	return;
}

static inline void idi_debug_remove(void)
{
	return;
}

static inline void idi_debug_add_event(enum idi_events event,
				       enum idi_peripheral_type p_type,
				       struct idi_transaction *trans)
{
	return;
}

#endif /* CONFIG_IDI_DEBUG */

#endif /*_IMC_DEBUG_H */
