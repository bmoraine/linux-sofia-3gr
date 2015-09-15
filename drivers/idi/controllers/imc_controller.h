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

#ifndef _IMC_CONTROLLER_H
#define _IMC_CONTROLLER_H

#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
#endif

#include "imc_idi.h"

/**
 * struct imc_channel_ctx - Channel context information.
 * @priority: channel priority
 * @base: Physical address to buffer for BOUNCE_BUFFER
 * @cpu_base: logical address for BOUNCE_BUFFER
 * @hw_fifo_size: Size of the hw fifo, .i.e. address aliasing
 * @size: internal buffer size in frames.
 * @stride: how many frames to receive before notification (RING_BUFFER only)
 * @current_wr: current register values to restore after resume
 * @current_rd: current register value to restore after resume
 * @current_irq: current register value to restore after resume
 */
struct imc_channel_ctx {
	u32 flags;
	u32 channel_opts;
#define DBUF_UPPER_IN_PROGRESS  (1 << 0)
#define DBUF_LOWER_IN_PROGRESS  (1 << 1)
#define DBUF_FULL          (DBUF_UPPER_IN_USE | DBUF_LOWER_IN_USE)
	enum idi_priority priority;
	dma_addr_t base;
	u32 *cpu_base;
	u32 size;
	u32 hw_fifo_size;
	void (*end_of_packet) (void *);
	void *private_data;
	void (*start_tx) (struct idi_transaction *);
	int stride;
	u32 dst_addr;
	u32 dst_size;
	u32 current_wr;
	u32 current_rd;
	u32 current_irq;
};

/**
 * struct imc_controller - IMC IDI controller data
 * @dev: device associated to the controller (IDI controller)
 * @pdev: Parent device (platform_device or pci_device) of IDI controller
 * @ctrl_io: IDI I/O ctrl address
 * @isr_tasklet: first-level high priority interrupt handling tasklet
 * @fwd_tasklet: second level response forwarding tasklet
 * @tx_idle_poll: timer for polling the TX side idleness
 * @rx_idle_poll: timer for polling the RX side idleness
 * @idle_poll: timer for polling idleness
 * @sw_lock: spinlock for accessing software FIFO
 * @hw_lock: spinlock for accessing hardware FIFO
 * @tx_queue: channel-indexed array of FIFO of messages awaiting transmission
 * @rx_queue: channel-indexed array of FIFO of messages awaiting reception
 * @brk_queue: FIFO for RX break messages
 * @fwd_queue: FIFO of messages awaiting of being forwarded back to client
 * @tx_ctx: Context for the ongoing TX channels
 * @rx_ctx: Context for the ongoing RX channels
 * @tx_queue_busy: bitmap of busy (frozen) TX queues
 * @rx_queue_busy: bitmap of busy (frozen) RX queues
 * @txbuf_dummy: cpu address of tx dummy buffer
 * @txbuf_dummy_dma: dma address of tx dummy buffer
 * @rxbuf_dummy: cpu address of rx dummy buffer
 * @rxbuf_dummy_dma: dma address of rx dummy buffer
 * @xfer_running: bitfield of running transfer transactions
 * @xfer_resumed: bitfield of resumed transfer transactions
 * @tx_state: current state of the TX side (0 for idle, >0 for ACWAKE)
 * @rx_state: current state of the RX state (0 for idle, 1 for ACWAKE)
 * @suspend_state: current power state (0 for powered, >0 for suspended)
 * @mis_status: current interrupt status
 * @mis1_status: current interrupt status of mis1.  unused
 * @imsc_cfg: current IMSC bits
 * @imsc1_cfg: current IMSC bits
 * @clc_cnt_cfg: current CLC_CNT bits
 * @txconf_cfg: current TXCONF bits. NOTE: need to be in configration mode
 * @txcon_cfg: current TXCON bits
 * @txmask_cfg: current TXMASK bits
 * @tx2mask_cfg: current TX2MASK bits
 * @tx3mask_cfg: current TX3MASK bits
 * @rxmask_cfg: current RXMASK bits
 * @rx2mask_cfg: current RX2MASK bits
 * @rx3mask_cfg: current RX2MASK bits
 * @rxconf_cfg: currnent RXCONF bits.  NOTE: need to be in configration mode
 * @rxcon_cfg: current RXCON bits
 * @err_con_cfg: current ERR_CON bits
 * @ext_con_cfg: current EXT_CON bits
 * @ext_con2_cfg: current EXT_CON2 bits
 * @channel_con_cfg: current CHANNEL_CON bits
 * @slmask_con_cfg: current SLMASK_CON bits
 * @sl2mask_con_cfg: current SL2MASK_CON bits
 * @brk_us_delay: Minimal BREAK sequence delay in us
 * @ip_resumed: event signalling that the IP has actually been resumed
 * @stay_awake: Android wake lock for preventing entering low power mode
 * @dir: debugfs IDI root directory
 */
struct imc_controller {
	/* Devices and resources */
	struct idi_controller_device *idi;
	struct device *dev;
	struct device *pdev;
	struct device *client;
	void __iomem *ctrl_io;

	unsigned int irqs[IMC_IDI_MAX_IRQS];

	/* Dual-level interrupt tasklets */
	struct tasklet_struct isr_tasklet;
	struct tasklet_struct fwd_tasklet;
	/* Timers for polling TX and RX states */
	struct timer_list tx_idle_poll;
	struct timer_list rx_idle_poll;
	struct timer_list idle_poll;
	/* Queues and registers access locks */
	spinlock_t sw_lock;
	spinlock_t hw_lock;

	/* Software FIFO */
	struct list_head tx_queue[IDI_MAX_CHANNEL];
	struct list_head rx_queue[IDI_MAX_CHANNEL];
	struct list_head brk_queue;
	struct list_head fwd_queue;
	struct imc_channel_ctx tx_ctx[IDI_MAX_CHANNEL];
	struct imc_channel_ctx rx_ctx[IDI_MAX_CHANNEL];
	unsigned short tx_queue_busy;
	unsigned short rx_queue_busy;

	/* Dummy buffer needed for channel initialization */
	unsigned *txbuf_dummy;
	dma_addr_t txbuf_dummy_dma;
	unsigned *rxbuf_dummy;
	dma_addr_t rxbuf_dummy_dma;
	/* Current Scatter/Gather processed messages */
	u32 xfer_running;
	u32 xfer_resumed;

	/* Current RX and TX states (0 for idle) */
	int tx_state;
	int rx_state;
	int suspend_state;

	/* PM Platform data */
	struct device_pm_platdata *pm_platdata;
	/* IDI platform power state handlers */
	struct platform_device_pm_state *idi_pm_state[IDI_MAX_POWER_STATE];

	/*IDI device usage reference count*/
	unsigned long idi_usage_flag;
	unsigned long stream_usage_flag;

	/*IDI Current device state*/
	enum idi_internal_power_state idi_state;

	/* IDI controller interrupt bits */
	u32 imsc_cfg;
	u32 imsc1_cfg;

	u32 mis_status;
	u32 mis1_status;

	/* IDI controller setup */
	u32 clc_cnt_cfg;
	u32 txconf_cfg;		/* need to be in configration mode */
	u32 txcon_cfg;
	u32 txmask_cfg;
	u32 tx2mask_cfg;
	u32 tx3mask_cfg;
	u32 rxmask_cfg;
	u32 rx2mask_cfg;
	u32 rx3mask_cfg;
	u32 rxconf_cfg;		/* need to be in configration mode */
	u32 rxcon_cfg;
	u32 err_con_cfg;
	u32 ext_con_cfg;
	u32 ext_con2_cfg;
	u32 channel_con_cfg;
	u32 slmask_con_cfg;
	u32 sl2mask_con_cfg;

	/* IDI controller IP frequency */
	u32 brk_us_delay;
	wait_queue_head_t ip_resumed;

#ifdef CONFIG_WAKELOCK
	/* Android PM support */
	struct wake_lock stay_awake;
#endif

#ifdef CONFIG_DEBUG_FS
	struct dentry *dir;
	u32 debug_imsc;
	u32 debug_imsc1;
#endif
};

struct imc_controller *imc_alloc_controller(struct device *);
void imc_free_controller(struct imc_controller *);
int imc_add_controller(struct imc_controller *);
void imc_remove_controller(struct imc_controller *);
#if defined IDI_FM_SUPPORT
extern struct idi_controller_device *idi_controller;
enum iui_fm_mitigation_status idi_fm_cb(
					const enum iui_fm_macro_id macro_id,
					const struct iui_fm_mitigation *fm_req,
					const uint32_t sequence);
#endif

#endif /* _IMC_CONTROLLER_H */
