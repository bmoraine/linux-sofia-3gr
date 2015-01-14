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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_bus.h>

#include <linux/idi/idi_debug.h>

/*FIXME: The include should not be exported*/
#include "../controllers/imc_idi.h"

#include <sofia/mv_svc_hypercalls.h>

#define AG6X0_SCU_RES_NAME	"scu"
#define AG6X0_PMU_RES_NAME	"pmu"

/*
 * FIXME: Should we include complete SCU headers def ?
*/
#define SCU_SP_PUR(_base) ((_base) + 0x108)
	#define SCU_SP_PUR_SP_PUR_OFFSET 0x0
	#define SCU_SP_PUR_SP_PUR_WIDTH 0x20
	#define SCU_SP_PUR_SP_PUR_MASK 0xffffffff
	#define SCU_SP_PUR_SP_PUR(_reg) (((_reg) & 0xffffffff) >> 0x0)

#define SCU_CHIP_ID(_base) ((_base) + 0x128)
	#define SCU_CHIP_ID_CHREV_OFFSET 0x0
	#define SCU_CHIP_ID_CHREV_WIDTH 0x8
	#define SCU_CHIP_ID_CHREV_MASK 0xff
	#define SCU_CHIP_ID_CHREV(_reg) (((_reg) & 0xff) >> 0x0)
	#define SCU_CHIP_ID_CHID_OFFSET 0x8
	#define SCU_CHIP_ID_CHID_WIDTH 0x8
	#define SCU_CHIP_ID_CHID_MASK 0xff00
	#define SCU_CHIP_ID_CHID(_reg) (((_reg) & 0xff00) >> 0x8)

#define AG6x0_ENTER pr_debug("--> %s\n", __func__)
#define AG6x0_EXIT pr_debug("<-- %s\n", __func__)

#define AG6X0_NATIVE_IO_ACCESS BIT(0)
#define AG6X0_SECURE_IO_ACCESS BIT(1)

struct ag6x0_io {
	struct resource *res;
	void __iomem	*vaddr;
};

struct ag6x0_client {
	struct idi_client_device *client_device;
	struct idi_controller_device *controller_device;

	void __iomem *ctrl_io;
	unsigned scu_io_phys;
	struct ag6x0_io *io_map;
	int num_io_region;

	spinlock_t sw_lock;
	spinlock_t hw_lock;

	unsigned flags;
	/*
	 * Software controlled channels.
	 */
	u32 sw_channel_map;
	u32 in_use_sw_control;
	enum idi_peripheral_type sw_channel[IDI_MAX_CHANNEL];

	/*
	 * Outstanding (programable) register transfer
	 */
	u32 wr_register_chan;
	u32 rd_register_chan;

	struct tasklet_struct isr_tasklet;

	/*
	 * Register settings on the AG620 IDI interface.
	 */
	u32 imsc;
	u32 mis_status;
	u32 txcon_cfg;
	u32 txmask_con_cfg;
	u32 rxcon_cfg;
	u32 extcon_cfg;
};

static inline struct idi_client_device *to_idi_client(struct ag6x0_client
						      *ag6x0)
{
	return ag6x0->client_device;
}

static unsigned ag6x0_set_irq_channel_mask(void __iomem *ctrl,
					   unsigned channel, bool set)
{
	unsigned reg;
	reg = ioread32(ctrl);
	if (set)
		reg |= (1 << channel);
	else
		reg &= ~(1 << channel);

	iowrite32(reg, ctrl);

	return reg;
}

/*
 * Caller must take care of lock
 * channel: The IDI channel to mask/unmask
 * Returns Interrupt channel mask values
 */
static inline unsigned ag6x0_mask_tx_irq_ch(struct ag6x0_client *ag6x0,
					    unsigned channel)
{
	void __iomem *ctrl = ag6x0->ctrl_io;

	return ag6x0_set_irq_channel_mask(IMC_IDI_TXMASK_CON(ctrl), channel, 0);
}

static inline unsigned ag6x0_unmask_tx_irq_ch(struct ag6x0_client *ag6x0,
					      unsigned channel)
{
	void __iomem *ctrl = ag6x0->ctrl_io;

	return ag6x0_set_irq_channel_mask(IMC_IDI_TXMASK_CON(ctrl), channel, 1);
}

static inline unsigned ag6x0_mask_rx_irq_ch(struct ag6x0_client *ag6x0,
					    unsigned channel)
{
	void __iomem *ctrl = ag6x0->ctrl_io;

	return ag6x0_set_irq_channel_mask(IMC_IDI_RXMASK_CON(ctrl), channel, 0);
}

static inline unsigned ag6x0_unmask_rx_irq_ch(struct ag6x0_client *ag6x0,
					      unsigned channel)
{
	void __iomem *ctrl = ag6x0->ctrl_io;

	return ag6x0_set_irq_channel_mask(IMC_IDI_RXMASK_CON(ctrl), channel, 1);
}

/*
 * ag6x0_rmw_imsc : Set or clear interrupt source in IMSC register
 *
 *	Caller  must hold the hw lock
 *	returns imsc value written
 */

static unsigned ag6x0_rmw_imsc(struct ag6x0_client *ag6x0,
			       unsigned imsc, unsigned set_not_clear)
{
	unsigned reg;
	void __iomem *ctrl = ag6x0->ctrl_io;

	reg = ioread32(IMC_IDI_IMSC(ctrl));
	if (set_not_clear)
		reg |= imsc;
	else
		reg &= ~(imsc);

	iowrite32(reg, IMC_IDI_IMSC(ctrl));

	return reg;
}

static inline unsigned ag6x0_mask_irqs(struct ag6x0_client *ag6x0,
				       unsigned irqs)
{
	return ag6x0_rmw_imsc(ag6x0, irqs, 0);
}

static inline unsigned ag6x0_unmask_irqs(struct ag6x0_client *ag6x0,
					 unsigned irqs)
{
	return ag6x0_rmw_imsc(ag6x0, irqs, 1);
}

/**
 * ag6x0_isr_tasklet - low-latency interrupt management out of interrupt state
 * @ctx: AGold reference
 */
static void ag6x0_isr_tasklet(unsigned long ctx)
__acquires(&ag6x0->sw_lock) __releases(&ag6x0->sw_lock)
__acquires(&ag6x0->hw_lock) __releases(&ag6x0->hw_lock)
{
	struct ag6x0_client *ag6x0 = (struct ag6x0_client *)ctx;
	struct idi_controller_device *controller;
	void __iomem *ctrl = ag6x0->ctrl_io;
	u32 mis_status, xfer_status, imsc_cfg = 0, mask, ch, ch_mask;

	unsigned long flags;

	AG6x0_ENTER;
	/*
	 * Get a local copy of the current interrupt status and clear the
	 * "global" version.
	 */
	spin_lock_irqsave(&ag6x0->sw_lock, flags);
	mis_status = ag6x0->mis_status;
	ag6x0->mis_status = 0;
	spin_unlock_irqrestore(&ag6x0->sw_lock, flags);

	controller = ag6x0->controller_device;

	mask = 1;
	/* DBB -> ABB: The transaction is completed once the data have been
	   transfered out of the ABB FIFOs */
	if (mis_status & IMC_IDI_MIS_RX) {
		if (ag6x0->rxcon_cfg & IMC_IDI_RXCON_EN_CH) {
			ch_mask = ioread32(IMC_IDI_RXMASK_CON(ctrl));
			xfer_status = ioread32(IMC_IDI_RXIRQ_STAT(ctrl));
			xfer_status &= ch_mask;

			mask = 1;
			for (ch = 0; ch < IDI_MAX_CHANNEL; ch++) {
				if (xfer_status & mask) {
					iowrite32(BIT(ch),
						  IMC_IDI_RXIRQ_CON(ctrl));
					controller->xfer_complete(controller, 1,
								  ch);
				}
				mask <<= 1;
			}
		}

		imsc_cfg |= IMC_IDI_IMSC_RX;
	}
	/* ABB -> DBB: The transaction is not yet completed,
	   but we should notifies the peripheral driver
	   of the end of packet detection */

	if (mis_status & IMC_IDI_MIS_TX) {
		if (ag6x0->txcon_cfg & IMC_IDI_TXCON_EN_CH) {
			ch_mask = ioread32(IMC_IDI_TXMASK_CON(ctrl));
			xfer_status = ioread32(IMC_IDI_TXIRQ_STAT(ctrl));
			xfer_status &= ch_mask;

			mask = 1;
			for (ch = 0; ch < IDI_MAX_CHANNEL; ch++) {
				if (xfer_status & mask) {
					controller->xfer_complete(controller, 0,
								  ch);
					iowrite32(BIT(ch),
						  IMC_IDI_TXIRQ_CON(ctrl));
				}
				mask <<= 1;
			}
		}

		imsc_cfg |= IMC_IDI_IMSC_TX;

	}

	/*
	 * Re-enable relevant interrupts.  Locking here because this
	 * can change in the interrupt service routine?  Race issue here?
	 */
	spin_lock_irqsave(&ag6x0->hw_lock, flags);
	ag6x0->imsc = ag6x0_unmask_irqs(ag6x0, imsc_cfg);
	spin_unlock_irqrestore(&ag6x0->hw_lock, flags);
}				/* ag6x0_isr_tasklet() */

static int ag6x0_channel_setup(struct ag6x0_client *ag6x0)
{
	int channel;
	enum idi_channel_type *channels = to_idi_client(ag6x0)->channels;

	AG6x0_ENTER;
	    /*
	     * Find the outstanding read & write channels
	     *  and software controlled channels.
	     * FIXME: Should we consider DBB for write channel mapping ?
	     */
	    for (channel = 0; channel < IDI_MAX_CHANNEL; channel++) {
		switch (channels[channel]) {
		case OUTSTANDING_READ:
			ag6x0->wr_register_chan = channel;
			ag6x0->rd_register_chan = channel;
			break;

		case SOFTWARE_DEFINED:
			ag6x0->sw_channel_map |= 1 << channel;
			break;

		default:
			break;
		}
	}

	return 0;
}				/* ag6x0_channel_setup() */

/**
 * ag6x0_open_software_channel() - Open a software controlled channel
 * @client: the client device to access
 * @peripheral: the peripheral to assign the channel to
 *
 */
int ag6x0_open_software_channel(struct idi_client_device *client,
				struct idi_peripheral_device *peripheral)
{
	unsigned long flags;
	int bit_pos;
	int channel;
	int err = 0;
	struct ag6x0_client *ag6x0;

	AG6x0_ENTER;

	ag6x0 = dev_get_drvdata(&client->device);
	if (ag6x0 == NULL)
		return -ENODEV;

	/*
	 * If a channel has already been mapped.  Make sure things jibe with
	 * what we think is mapped.
	 */
	if (peripheral->sw_channel < IDI_MAX_CHANNEL) {
		/*
		 * If the peripheral channel is not correct, return an error.
		 */
		if ((ag6x0->
		     sw_channel_map & (1 << peripheral->sw_channel)) == 0)
			return -EINVAL;

		/*
		 * If the peripheral type doesn't match our mapping give an
		 * error.
		 */
		if (ag6x0->sw_channel[peripheral->sw_channel] !=
		    peripheral->p_type)
			return -EINVAL;

		/*
		 * Redundant open.
		 */
		return 0;
	}

	/*
	 * Check to see if any channels are available.
	 */
	if ((ag6x0->sw_channel_map & ag6x0->in_use_sw_control) ==
					ag6x0->sw_channel_map)
		return -EBUSY;

	spin_lock_irqsave(&ag6x0->sw_lock, flags);

	/*
	 * Walk the bit mapped channel list and find an unused channel.
	 */
	for (bit_pos = IDI_MAX_CHANNEL - 1, channel = IDI_MAX_CHANNEL;
	     bit_pos > 0 && channel == IDI_MAX_CHANNEL; bit_pos--) {
		/*
		 * If the bit is a valid channel, and it is unused map it.
		 */
		if (((1 << bit_pos) & ag6x0->sw_channel_map) &&
		    (((1 << bit_pos) & ag6x0->in_use_sw_control) == 0))
			channel = bit_pos;
	}

	/*
	 * Set the bit in the in_use mask and assign the channel to the
	 * requesting peripheral.
	 */
	if (channel == IDI_MAX_CHANNEL) {
		err = -EBUSY;
	} else {
		ag6x0->in_use_sw_control |= (1 << channel);
		ag6x0->sw_channel[channel] = peripheral->p_type;
		peripheral->sw_channel = channel;
		err = 0;
	}

	spin_unlock_irqrestore(&ag6x0->sw_lock, flags);

	return err;
}				/* ag6x0_open_software_channel() */

/**
 * ag6x0_close_software_channel - Free up a previously opened channel
 * @client: client instance to use
 * @peripheral: peripheral device that is closing the channel
 */
int ag6x0_close_software_channel(struct idi_client_device *client,
				 struct idi_peripheral_device *peripheral)
{
	unsigned long flags;
	struct ag6x0_client *ag6x0;
	void __iomem *ctrl;
	AG6x0_ENTER;

	ag6x0 = dev_get_drvdata(&client->device);
	if (ag6x0 == NULL)
		return -ENODEV;
	/*
	 * If no channel is recorded, we are done.
	 */
	if (peripheral->sw_channel == IDI_MAX_CHANNEL)
		return 0;

	/*
	 * If the peripheral channel is not a valid software channel, return an
	 * error.
	 */
	if ((ag6x0->sw_channel_map & (1 << peripheral->sw_channel)) == 0)
		return -EINVAL;

	/*
	 * Make sure that the peripheral channel matches the internal mapping.
	 */
	if (ag6x0->sw_channel[peripheral->sw_channel] != peripheral->p_type)
		return -EINVAL;

	ctrl = ag6x0->ctrl_io;

	spin_lock_irqsave(&ag6x0->sw_lock, flags);
	iowrite32((IMC_IDI_RXCH_RD_CON_RST |
			IMC_IDI_RXCH_RD_CON_RST_INT),
			IMC_IDI_RXCH_RD_CON(ctrl, peripheral->sw_channel));

	ag6x0->in_use_sw_control &= ~(1 << peripheral->sw_channel);
	ag6x0->sw_channel[peripheral->sw_channel] = IDI_MAX_CHANNEL;
	peripheral->sw_channel = IDI_MAX_CHANNEL;

	spin_unlock_irqrestore(&ag6x0->sw_lock, flags);

	return 0;
}				/* ag6x0_close_software_channel() */

static int ag6x0_software_read(struct idi_client_device *client,
			       struct idi_peripheral_device *peripheral,
			       struct idi_transaction *data)
{
	struct ag6x0_client *ag6x0;
	void __iomem *ctrl_io;

	AG6x0_ENTER;

	/*
	 * If we don't have a software channel assigned, we are done.
	 */
	if (peripheral->sw_channel == IDI_MAX_CHANNEL)
		return -EINVAL;

	ag6x0 = dev_get_drvdata(&client->device);
	if (ag6x0 == NULL)
		return -ENODEV;

	/*
	 * If the peripheral channel is not correct, return an error.
	 */
	if ((ag6x0->sw_channel_map & (1 << peripheral->sw_channel)) == 0)
		return -EINVAL;

	/*
	 * If the peripheral type doesn't match our mapping give an error.
	 */
	if (ag6x0->sw_channel[peripheral->sw_channel] != peripheral->p_type)
		return -EINVAL;

	/*
	 * Set the transaction channel to the SW channel and prime the receive
	 * information.
	 */
	data->channel = peripheral->sw_channel;
	data->t_type = IDI_TRANS_READ;
	idi_async(peripheral, data);

	/*
	 * Set up the AG6X0 TX channels with the address to get data from, and
	 * the size and start the transaction.
	 */
	ctrl_io = ag6x0->ctrl_io;

	iowrite32(data->idi_xfer.dst_addr,
		  IMC_IDI_TXCH_BASE(ctrl_io, peripheral->sw_channel));
	iowrite32(data->idi_xfer.size,
		  IMC_IDI_TXCH_SIZE(ctrl_io, peripheral->sw_channel));
	iowrite32(IMC_IDI_TXCH_WR_CON_RST,
		  IMC_IDI_TXCH_WR_CON(ctrl_io, peripheral->sw_channel));
	iowrite32(0, IMC_IDI_TXCH_WR_CON(ctrl_io, peripheral->sw_channel));

	return 0;
}				/* ag6x0_software_read() */

static int ag6x0_software_write(struct idi_client_device *client,
				struct idi_peripheral_device *peripheral,
				struct idi_transaction *data)
{
	struct ag6x0_client *ag6x0;
	void __iomem *ctrl_io;

	AG6x0_ENTER;
	/*
	 * If we don't have a software channel assigned, we are done.
	 */
	if (peripheral->sw_channel == IDI_MAX_CHANNEL)
		return -EINVAL;

	ag6x0 = dev_get_drvdata(&client->device);
	if (ag6x0 == NULL)
		return -ENODEV;

	/*
	 * If the peripheral channel is not correct, return an error.
	 */
	if ((ag6x0->sw_channel_map & (1 << peripheral->sw_channel)) == 0)
		return -EINVAL;

	/*
	 * If the peripheral type doesn't match our mapping give an error.
	 */
	if (ag6x0->sw_channel[peripheral->sw_channel] != peripheral->p_type)
		return -EINVAL;

	/*
	 * Set up the AG6x0 RX channels with the address to get data from, and
	 * the size and start the transaction.
	 */
	ctrl_io = ag6x0->ctrl_io;

	iowrite32(data->idi_xfer.dst_addr,
		  IMC_IDI_RXCH_BASE(ctrl_io, peripheral->sw_channel));
	iowrite32(data->idi_xfer.size,
		  IMC_IDI_RXCH_SIZE(ctrl_io, peripheral->sw_channel));
	iowrite32(IMC_IDI_RXCH_RD_CON_RST,
		  IMC_IDI_RXCH_RD_CON(ctrl_io, peripheral->sw_channel));

	iowrite32(0, IMC_IDI_RXCH_RD_CON(ctrl_io, peripheral->sw_channel));

	/*
	 * Set the transaction channel to the SW channel
	 */
	data->channel = peripheral->sw_channel;
	data->t_type = IDI_TRANS_WRITE;
	idi_async(peripheral, data);

	return 0;
}				/* ag6x0_software_write() */

/**
 * ag6x0_set_addr_size() - set the channel registers for a transfer.
 * @trans: the current transaction
 * @old_addr: the old destination address for memory or register
 * @old_size: the old size to set for the IRQ register so we don't get an
 *            interrupt on the client side.
 *
 * Trying to be smart about this:
 *    If the old address is not set (0), set it.
 *    If the new address is different than the old address, set it.
 *    If the new size is > the old size, set the new size.
 *
 */
static int ag6x0_set_addr_size(struct idi_client_device *client,
			       struct idi_transaction *trans,
			       u32 old_addr, u32 old_size)
{
	int set_it = 0;
	void __iomem *ctrl;
	struct ag6x0_client *ag6x0;
	u32 ch_size;
	u32 ch_addr;
	bool ch_is_fifo = false;
	AG6x0_ENTER;

	ag6x0 = dev_get_drvdata(&client->device);
	if ((client->channels[trans->channel] == FILE)
	    || (client->channels[trans->channel] == STREAM))
		ch_is_fifo = true;

	ch_size = round_up(trans->idi_xfer.size, sizeof(u32));
	ch_addr = trans->idi_xfer.dst_addr;
	/* Initial configuration */
	if (ch_is_fifo) {
		if ((old_addr != 0) && (old_size != 0)) {
			ch_size = old_size;
			ch_addr = old_addr;
			set_it = 1;
		} else {
			dev_err(&client->device,
				"Setting a channel with HW fifo as endpoint failed !\n");
			return -EINVAL;
		}
	} else {
		if (old_addr == 0)
			set_it = 1;
		if (trans->idi_xfer.dst_addr != old_addr)
			set_it = 1;
		if (ch_size > old_size)
			set_it = 1;
	}

	if (set_it) {
		ctrl = ag6x0->ctrl_io;

		dev_dbg(&client->device,
			"Set ABB %s Channel %d with %#x bytes @%x\n",
			((trans->t_type == IDI_TRANS_WRITE) ? "RX" : "TX"),
			trans->channel, ch_size, ch_addr);

		/*
		 * Remember, transmit on master side is a receive on the client
		 * side.
		 */
		if (trans->t_type == IDI_TRANS_WRITE) {
			unsigned stat;
			stat = ioread32(IMC_IDI_RXCH_RD_CON(ctrl,
						trans->channel));
			/*
			 * FIXME:
			 * Define the policy in case
			 * a pending request is active
			 */
			if (stat & IMC_IDI_RXCH_WR_STAT_DAT)
				dev_err(&client->device,
					"A pending request on RX CH %d will be cleared\n",
					trans->channel);

			iowrite32(ch_addr,
				  IMC_IDI_RXCH_BASE(ctrl, trans->channel));
			iowrite32(ch_size,
				  IMC_IDI_RXCH_SIZE(ctrl, trans->channel));
			iowrite32(IMC_IDI_RXCH_RD_CON_RST |
				  IMC_IDI_RXCH_RD_CON_RST_INT,
				  IMC_IDI_RXCH_RD_CON(ctrl, trans->channel));
			/* Explicitely disable the interrupt */
			iowrite32(ch_size + sizeof(u32),
				  IMC_IDI_RXCH_IRQ_CON(ctrl, trans->channel));
		} else {	/* read */
			unsigned stat;
			stat = ioread32(IMC_IDI_TXCH_WR_CON(ctrl,
						trans->channel));
			if (stat & IMC_IDI_TXCH_RD_STAT_DAT)
				dev_err(&client->device,
					"A pending request on TX CH %d will be cleared\n",
					trans->channel);
			iowrite32(ch_addr,
				  IMC_IDI_TXCH_BASE(ctrl, trans->channel));
			iowrite32(ch_size,
				  IMC_IDI_TXCH_SIZE(ctrl, trans->channel));
			iowrite32(IMC_IDI_TXCH_WR_CON_RST |
				  IMC_IDI_TXCH_WR_CON_RST_INT,
				  IMC_IDI_TXCH_WR_CON(ctrl, trans->channel));
			/* Explicitely disable the interrupt */
			iowrite32(ch_size + sizeof(u32),
				  IMC_IDI_TXCH_IRQ_CON(ctrl, trans->channel));
		}
	}

	return 0;
}				/* ag6x0_set_ch_addr_size() */

static void ag6x0_dump_channel(struct idi_client_device *client,
			       unsigned channel, unsigned tx_not_rx)
{
	struct ag6x0_client *ag6x0;
	void __iomem *ctrl;
	u32 base, size, control, irq, stat;
	struct device *dev = &client->device;

	AG6x0_ENTER;

	ag6x0 = dev_get_drvdata(&client->device);

	ctrl = ag6x0->ctrl_io;

	if (tx_not_rx) {
		base = ioread32(IMC_IDI_RXCH_BASE(ctrl, channel));
		size = ioread32(IMC_IDI_RXCH_SIZE(ctrl, channel));
		control = ioread32(IMC_IDI_RXCH_RD_CON(ctrl, channel));
		irq = ioread32(IMC_IDI_RXCH_IRQ_CON(ctrl, channel));
		stat = ioread32(IMC_IDI_RXCH_WR_STAT(ctrl, channel));
	} else {
		base = ioread32(IMC_IDI_TXCH_BASE(ctrl, channel));
		size = ioread32(IMC_IDI_TXCH_SIZE(ctrl, channel));
		control = ioread32(IMC_IDI_TXCH_WR_CON(ctrl, channel));
		irq = ioread32(IMC_IDI_TXCH_IRQ_CON(ctrl, channel));
		stat = ioread32(IMC_IDI_TXCH_RD_STAT(ctrl, channel));
	}

	dev_dbg(dev, "\t Base: %x\n", base);
	dev_dbg(dev, "\t Size: %x\n", size);
	dev_dbg(dev, "\t Control: %x\n", control);
	dev_dbg(dev, "\t IRQ: %x\n", irq);
	dev_dbg(dev, "\t Stat: %x\n", stat);
}

static int ag6x0_set_interrupt(struct idi_client_device *client,
			       struct idi_transaction *trans)
__acquires(&ag6x0->hw_lock) __releases(&ag6x0->hw_lock)
{
	struct ag6x0_client *ag6x0 = dev_get_drvdata(&client->device);
	void __iomem *ctrl = ag6x0->ctrl_io;
	unsigned long flags;

	u32 channel = trans->channel;

	int ret = 0;

	AG6x0_ENTER;


	/* TODO: Sanity check: Not a software,.., channel */

	/*
	 * If the Channel is a DMA, an interrupt is generated
	 * when last burst/single request is asserted
	 */
	switch (trans->t_type) {
	case IDI_TRANS_WRITE:
		if (client->channels[channel] == SOFTWARE_DEFINED)
			iowrite32(round_up(trans->idi_xfer.size, sizeof(u32)),
				  IMC_IDI_RXCH_IRQ_CON(ctrl, trans->channel));

		iowrite32((1 << channel), IMC_IDI_RXIRQ_CON(ctrl));
		spin_lock_irqsave(&ag6x0->hw_lock, flags);
		ag6x0_unmask_rx_irq_ch(ag6x0, channel);
		spin_unlock_irqrestore(&ag6x0->hw_lock, flags);
		break;
	case IDI_TRANS_READ:
		if (client->channels[channel] == SOFTWARE_DEFINED)
			iowrite32(round_up(trans->idi_xfer.size, sizeof(u32)),
				  IMC_IDI_TXCH_IRQ_CON(ctrl, trans->channel));

		iowrite32((1 << channel), IMC_IDI_TXIRQ_CON(ctrl));
		spin_lock_irqsave(&ag6x0->hw_lock, flags);
		ag6x0_unmask_tx_irq_ch(ag6x0, channel);
		spin_unlock_irqrestore(&ag6x0->hw_lock, flags);
		break;
	default:
		BUG();
	}

	return ret;
}

static int ag6x0_buffer_flush(struct idi_client_device *client,
				struct idi_transaction *trans)
__acquires(&ag6x0->hw_lock) __releases(&ag6x0->hw_lock)
{

	struct ag6x0_client *ag6x0 = dev_get_drvdata(&client->device);
	void __iomem *ctrl = ag6x0->ctrl_io;
	unsigned long flags;
	u32 mis_status;
	u32 channel = trans->channel;

	AG6x0_ENTER;
	switch (trans->t_type) {
	case IDI_TRANS_WRITE:
		spin_lock_irqsave(&ag6x0->hw_lock, flags);
		iowrite32(IMC_IDI_RXCH_RD_CON_RST |
				IMC_IDI_RXCH_RD_CON_RST_INT,
				IMC_IDI_RXCH_RD_CON(ctrl, channel));
		mis_status = ioread32(IMC_IDI_MIS(ctrl));
		spin_unlock_irqrestore(&ag6x0->hw_lock, flags);
		break;
	case IDI_TRANS_READ:
		spin_lock_irqsave(&ag6x0->hw_lock, flags);
		iowrite32(IMC_IDI_TXCH_WR_CON_RST |
				IMC_IDI_TXCH_WR_CON_RST_INT,
				IMC_IDI_TXCH_WR_CON(ctrl, channel));
		mis_status = ioread32(IMC_IDI_MIS(ctrl));
		spin_unlock_irqrestore(&ag6x0->hw_lock, flags);
		break;
	default:
		BUG();
	}
	return 0;
}

static int ag6x0_streaming_channel_flush(struct idi_client_device *client,
			unsigned int channel)
__acquires(&ag6x0->hw_lock) __releases(&ag6x0->hw_lock)
{
	struct ag6x0_client *ag6x0 = dev_get_drvdata(&client->device);
	void __iomem *ctrl = ag6x0->ctrl_io;
	unsigned long flags;

	AG6x0_ENTER;
	spin_lock_irqsave(&ag6x0->hw_lock, flags);
	iowrite32(IMC_IDI_RXCH_RD_CON_RST |
			IMC_IDI_RXCH_RD_CON_RST_INT,
			IMC_IDI_RXCH_RD_CON(ctrl, channel));

	iowrite32(IMC_IDI_TXCH_WR_CON_RST |
			IMC_IDI_TXCH_WR_CON_RST_INT,
			IMC_IDI_TXCH_WR_CON(ctrl, channel));
	spin_unlock_irqrestore(&ag6x0->hw_lock, flags);
	AG6x0_EXIT;

	return 0;
}

void __iomem *ag6x0_io_phys_to_virt(struct ag6x0_client *ag6x0,
					unsigned addr)

{
	unsigned i;
	struct ag6x0_io *io_map = ag6x0->io_map;

	if (io_map == NULL)
		return NULL;

	for (i = 0; i < ag6x0->num_io_region; i++) {
		struct resource *res = io_map[i].res;
		if ((addr >= res->start) && (addr <= res->end))
			return (io_map[i].vaddr +
					((resource_size(res) - 1) & addr));
	}

	return NULL;
}

static int ag6x0_ioread(struct idi_client_device *client,
					unsigned addr, unsigned *reg)
{
	int ret = 0;
	struct ag6x0_client *ag6x0;

	ag6x0 = dev_get_drvdata(&client->device);
	if (ag6x0 == NULL)
		return -ENODEV;

	if (ag6x0->flags & AG6X0_SECURE_IO_ACCESS) {
		ret = mv_svc_reg_read(addr, reg, -1);
		dev_dbg(&client->device,
			"Secure read access @%x--> %x:", addr, *reg);
	} else {
		void __iomem *vaddr = ag6x0_io_phys_to_virt(ag6x0, addr);
		if (vaddr == NULL)
			return -EINVAL;
		*reg = ioread32(vaddr);
		dev_dbg(&client->device,
			"Native read access @%x,%p --> %x:", addr, vaddr, *reg);
	}

	return ret;
}

static int ag6x0_iowrite(struct idi_client_device *client,
					unsigned addr, unsigned reg)
{
	int ret = 0;
	struct ag6x0_client *ag6x0;

	ag6x0 = dev_get_drvdata(&client->device);
	if (ag6x0 == NULL)
		return -ENODEV;

	if (ag6x0->flags & AG6X0_SECURE_IO_ACCESS) {
		dev_dbg(&client->device,
			"Secure write access %x@%x:", reg, addr);
		ret = mv_svc_reg_write(addr, reg, -1);

	} else {
		void __iomem *vaddr = ag6x0_io_phys_to_virt(ag6x0, addr);
		if (vaddr == NULL)
			return -EINVAL;
		iowrite32(reg, vaddr);
		dev_dbg(&client->device,
			"Native write access %x@%x,%p:", reg, addr, vaddr);
	}

	return ret;
}

static int ag6x0_atomic_iowrite(struct idi_client_device *client,
				unsigned addr, unsigned reg, unsigned mask)
{
	int ret = 0;
	struct ag6x0_client *ag6x0;
	unsigned scratch;

	ag6x0 = dev_get_drvdata(&client->device);
	if (ag6x0 == NULL)
		return -ENODEV;

	if (ag6x0->flags & AG6X0_SECURE_IO_ACCESS) {
		dev_dbg(&client->device,
			"Secure atomic write access %x@%x:",
							reg, addr);
		ret = mv_svc_reg_write(addr, reg, mask);

	} else {
		void __iomem *vaddr = ag6x0_io_phys_to_virt(ag6x0, addr);
		unsigned long flags;
		if (vaddr == NULL)
			return -EINVAL;
		/*
		 * If the Linux VM is interrupted, the spinlock will not
		 * be helpful to prevent a concurent access..
		*/

		spin_lock_irqsave(&ag6x0->hw_lock, flags);
		scratch = ioread32(vaddr);
		scratch &= ~mask;
		scratch |= (reg & mask);
		iowrite32(scratch, vaddr);
		spin_unlock_irqrestore(&ag6x0->hw_lock, flags);
		dev_dbg(&client->device,
			"Native Atomic write access %x@%x,%p:",
							reg, addr, vaddr);
	}

	return ret;
}


static int ag6x0_dev_setup(struct idi_client_device *client,
			   struct ag6x0_client *ag6x0)
{
	AG6x0_ENTER;
	/*
	 * So we don't have to call the driver to get the channel...
	 */
	client->wr_register_chan = ag6x0->wr_register_chan;
	client->rd_register_chan = ag6x0->rd_register_chan;

	client->open_software_channel = ag6x0_open_software_channel;
	client->close_software_channel = ag6x0_close_software_channel;
	client->software_read = ag6x0_software_read;
	client->software_write = ag6x0_software_write;
	client->set_addr_size = ag6x0_set_addr_size;
	client->dump_channel = ag6x0_dump_channel;
	client->set_interrupt = ag6x0_set_interrupt;
	client->flush_buffer = ag6x0_buffer_flush;
	client->streaming_channel_flush = ag6x0_streaming_channel_flush;
	client->ioread = ag6x0_ioread;
	client->iowrite = ag6x0_iowrite;

	dev_set_drvdata(&client->device, ag6x0);

	return 0;
}				/* ag6x0_dev_setup() */

static int ag6x0_register_setup(struct ag6x0_client *ag6x0)
{
	int chan;
	void __iomem *ctrl_io;
	enum idi_channel_type *channels = to_idi_client(ag6x0)->channels;
	unsigned stream_channels = 0, dma_channels = 0;
	unsigned nr_stream = 0, nr_dma = 0;

	AG6x0_ENTER;

	ctrl_io = ag6x0->ctrl_io;

	/* Clear interrupts */
	iowrite32(0, IMC_IDI_IMSC(ctrl_io));
	iowrite32((-1), IMC_IDI_ICR(ctrl_io));

	/* Disable channels interrupts until explicitely requested */
	iowrite32(0, IMC_IDI_TXMASK_CON(ctrl_io));
	iowrite32(0, IMC_IDI_RXMASK_CON(ctrl_io));

	for (chan = 0; chan < IDI_MAX_CHANNEL; chan++) {
		bool do_rst = true;
		if (channels[chan] == OUTSTANDING_READ)
			do_rst = false;

		/* Initilize the channels */
		iowrite32(0, IMC_IDI_RXCH_IRQ_CON(ctrl_io, chan));
		iowrite32(0, IMC_IDI_RXCH_SIZE(ctrl_io, chan));
		iowrite32(0, IMC_IDI_RXCH_BASE(ctrl_io, chan));

		if (do_rst) {
			iowrite32(IMC_IDI_RXCH_RD_CON_RST,
				  IMC_IDI_RXCH_RD_CON(ctrl_io, chan));

			/* Reset internal buffer to prevent dummy
			   data in buffer */
			udelay(1);
			iowrite32(IMC_IDI_RXCH_RD_CON_RST_INT,
				  IMC_IDI_RXCH_RD_CON(ctrl_io, chan));
		}

		/* Tx channels */
		iowrite32(0, IMC_IDI_TXCH_IRQ_CON(ctrl_io, chan));
		iowrite32(0, IMC_IDI_TXCH_SIZE(ctrl_io, chan));
		iowrite32(0, IMC_IDI_TXCH_BASE(ctrl_io, chan));

		if (do_rst)
			iowrite32(IMC_IDI_TXCH_WR_CON_RST,
				  IMC_IDI_TXCH_WR_CON(ctrl_io, chan));

		if (channels[chan] == FILE) {
			dma_channels |= (1 << chan);
			nr_dma++;
		}

		if (channels[chan] == STREAM) {
			stream_channels |= (1 << chan);
			nr_stream++;
		}
	}

	/*
	 * Enable the channels.
	 */
	ag6x0->rxcon_cfg = (IMC_IDI_RXCON_EN_LOG
				| IMC_IDI_RXCON_EN_CH
				| IMC_IDI_RXCON_FLOW(0)
				| IMC_IDI_RXCON_SKIP_LIMIT(4)
				| IMC_IDI_RXCON_FLUSH_TO(1))
				| IMC_IDI_RXCON_DIS_BURST(dma_channels);

	iowrite32(ag6x0->rxcon_cfg, IMC_IDI_RXCON(ctrl_io));

	ag6x0->txcon_cfg = (IMC_IDI_TXCON_EN_LNK
				| IMC_IDI_TXCON_EN_CH
				| IMC_IDI_TXCON_MASTER
				| IMC_IDI_TXCON_PRIORITY(stream_channels));

	iowrite32(ag6x0->txcon_cfg, IMC_IDI_TXCON(ctrl_io));

	ag6x0->extcon_cfg = (IMC_IDI_EXT_CON_SIG(IMC_IDI_EXT_CON_SIG_ENABLED)
			| IMC_IDI_EXT_CON_INT(IMC_IDI_EXT_CON_INT_ENABLED)
			| IMC_IDI_EXT_CON_STREAM(BIT(nr_stream) - 1)
			| IMC_IDI_EXT_CON_DMA(BIT(nr_dma) - 1)
			| IMC_IDI_EXT_CON_PRIORITY
				(IMC_IDI_ABB_EXT_CON_PRIORITY_REG
				| IMC_IDI_ABB_EXT_CON_PRIORITY_DMA)
			);

	iowrite32(ag6x0->extcon_cfg, IMC_IDI_EXT_CON(ctrl_io));

	iowrite32((-1), IMC_IDI_TXIRQ_CON(ctrl_io));
	iowrite32((-1), IMC_IDI_RXIRQ_CON(ctrl_io));

	ag6x0->imsc = IMC_IDI_IMSC_RX | IMC_IDI_IMSC_TX;
	iowrite32(ag6x0->imsc, IMC_IDI_IMSC(ctrl_io));

	return 0;
}				/* ag6x0_register_setup() */

static irqreturn_t ag6x0_isr(int irq, void *dev_id)
	__acquires(&ag6x0->hw_lock) __releases(&ag6x0->hw_lock)
{
	struct idi_client_device *client = dev_id;
	struct ag6x0_client *ag6x0 = dev_get_drvdata(&client->device);
	void __iomem *ctrl = ag6x0->ctrl_io;
	u32 mis_status;

	AG6x0_ENTER;

	spin_lock(&ag6x0->hw_lock);
	mis_status = ioread32(IMC_IDI_MIS(ctrl));
	dev_dbg(&client->device, "MIS %x, mis_status %x\n", mis_status,
		ag6x0->mis_status);

	if (mis_status) {
		ag6x0_mask_irqs(ag6x0, mis_status);
		iowrite32(mis_status, IMC_IDI_ICR(ctrl));
		ag6x0->imsc |= mis_status;
		ag6x0->mis_status |= mis_status;
	}

	spin_unlock(&ag6x0->hw_lock);

	if (mis_status)
		tasklet_hi_schedule(&ag6x0->isr_tasklet);

	AG6x0_EXIT;
	return IRQ_HANDLED;
}

static const struct ag6x0_irq_desc {
	const char *name;
	irq_handler_t handler;
	irq_handler_t handler_bh;
} irq_desc[3] = {
	{ .name = "err", .handler = ag6x0_isr, .handler_bh = NULL},
	{ .name = "tx", .handler = ag6x0_isr, .handler_bh = NULL},
	{ .name = "rx", .handler = ag6x0_isr, .handler_bh = NULL},
};

#ifdef CONFIG_X86_INTEL_SOFIA_ERRATA_002
static int ag6x0_request_irqs(struct idi_client_device *client)
{
	/* Due to the IDI bus HW limitation, we cannot request an ABB irq
	 * because it will go through the ABB chipirq, which could generate
	 * ABB RX transaction */

	return 0;
}
#else
static int ag6x0_request_irqs(struct idi_client_device *client)
{
	struct ag6x0_client *ag6x0;
	int ret = 0;
	int i = 0;
	struct idi_resource *idi_res = &client->idi_res;

	ag6x0 = dev_get_drvdata(&client->device);

	for (i = 0; i < ARRAY_SIZE(irq_desc); i++) {
		struct resource *res;
		const char *name = irq_desc[i].name;
		irq_handler_t handler = irq_desc[i].handler;
		irq_handler_t handler_bh = irq_desc[i].handler_bh;
		res = idi_get_resource_byname(idi_res, IORESOURCE_IRQ, name);
		if (!res) {
			ret = -EINVAL;
			goto fail_request_irq;
		}

		ret = request_threaded_irq(res->start, handler, handler_bh,
					   IRQF_SHARED, name, client);
		if (ret) {
			dev_err(&client->device,
				"Error while requesting %pa irq node\n",
				&res->start);
			ret = -EINVAL;
			goto fail_request_irq;
		}
	}

	return 0;

fail_request_irq:
	for (i--; i > (-1); i--) {
		struct resource *res;
		const char *name = irq_desc[i].name;

		if (0 > i)
			BUG();

		res = idi_get_resource_byname(idi_res, IORESOURCE_IRQ, name);

		/* This should not happen */
		if (!res)
			BUG();

		free_irq(res->start, client);
	}
	return ret;

}
#endif

static void ag6x0_free_irqs(struct idi_client_device *client)
{
	int i = 0;
	struct idi_resource *idi_res = &client->idi_res;
	struct resource *res;

	for (i = 0; i < ARRAY_SIZE(irq_desc); i++) {
		const char *name = irq_desc[i].name;

		res = idi_get_resource_byname(idi_res, IORESOURCE_IRQ, name);
		if (res)
			free_irq(res->start, client);
	}
}

static void ag6x0_unmap_io(struct idi_client_device *device,
				struct ag6x0_client *ag6x0)
{
	int i;

	for (i = 0; i < ag6x0->num_io_region; i++)
		iounmap(ag6x0->io_map[i].vaddr);

	kfree(ag6x0->io_map);
}

static int ag6x0_map_io(struct idi_client_device *client,
			struct ag6x0_client *ag6x0)
{
	struct idi_resource *idi_res = &client->idi_res;
	int i , j = 0;
	struct ag6x0_io *io_map;
	for (i = 0; i < idi_res->num_resources; i++)
		if (IORESOURCE_MEM == resource_type(&idi_res->resource[i]))
			j++;

	if (j == 0)
		return -ENODEV;

	io_map = kzalloc(sizeof(struct ag6x0_io) * j, GFP_KERNEL);
	if (io_map == NULL)
		return -ENOMEM;

	for (i = 0, j = 0; i < idi_res->num_resources; i++) {
		struct resource *r = &idi_res->resource[i];

		if (IORESOURCE_MEM == resource_type(r)) {
			io_map[j].res = r;
			io_map[j].vaddr = ioremap(r->start, resource_size(r));
			if (io_map[j].vaddr == NULL)
				goto unmap;
			if (!strcmp(r->name, AG6X0_SCU_RES_NAME))
				ag6x0->scu_io_phys = r->start;
			j++;

		}
	}

	ag6x0->num_io_region = j;
	ag6x0->io_map = io_map;
	return 0;

unmap:
	for (i = 0; i < j; i++)
		iounmap(io_map[i].vaddr);
	kfree(io_map);
	return -EINVAL;
}

/**
 * ag6x0_probe - Probe function to init the AG6x0.
 * @dev: client device
 * @id: list of ids to match.
 *
 */
static int ag6x0_probe(struct idi_client_device *client,
		       const struct idi_device_id *id)
{
	int err = 0, i;
	struct ag6x0_client *ag6x0;
	struct idi_controller_device *controller;
	struct resource *io_ctrl_resource;
	u32 reg, chipid = 0;
	struct device *dev = &client->device;
	struct device_node *np = dev->of_node;


	AG6x0_ENTER;

	ag6x0 = kzalloc(sizeof(*ag6x0), GFP_KERNEL);
	if (ag6x0 == NULL) {
		err = -ENOMEM;
		return err;
	}
	io_ctrl_resource = idi_get_resource_byname(&client->idi_res,
						   IORESOURCE_MEM, "idi");
	if (!io_ctrl_resource) {
		err = -ENODEV;
		goto no_ioremap;
	}

	ag6x0->ctrl_io = ioremap_nocache(io_ctrl_resource->start,
					 resource_size(io_ctrl_resource));

	if (ag6x0->ctrl_io == NULL) {
		err = -ENODEV;
		goto no_ioremap;
	}


	if (ag6x0_map_io(client, ag6x0)) {
		dev_err(&client->device, "Error while io mapping\n");
		goto no_ioremap;
	}

	/* Get SCU I/O access mode */
	if (of_find_property(np, "intel,vmm-secured-access", NULL)) {
		dev_info(&client->device, "AGold using secure access\n");
		ag6x0->flags |= AG6X0_SECURE_IO_ACCESS;
	} else {
		dev_info(&client->device, "AGold using native access\n");
		ag6x0->flags |= AG6X0_NATIVE_IO_ACCESS;
	}

	ag6x0->client_device = client;
	spin_lock_init(&ag6x0->sw_lock);
	spin_lock_init(&ag6x0->hw_lock);
	tasklet_init(&ag6x0->isr_tasklet, ag6x0_isr_tasklet,
		     (unsigned long)ag6x0);


	for (i = 0; i < IDI_MAX_CHANNEL; i++)
		ag6x0->sw_channel[i] = IDI_MAX_CHANNEL;

	controller = to_idi_controller_device(client->device.parent);
	if (controller == NULL) {
		err = -ENODEV;
		goto no_match;

	}
	ag6x0->controller_device = controller;

	ag6x0_channel_setup(ag6x0);
	ag6x0_dev_setup(client, ag6x0);
	ag6x0_register_setup(ag6x0);

	reg = ioread32(IMC_IDI_ID(ag6x0->ctrl_io));
	switch (reg & 0x000000FF) {
	case 0:
	case 1:
	case 2:
		dev_err(&client->device, "Unsupported device %d\n",
			reg & 0x000000FF);
		err = -ENODEV;
		goto no_match;
	case 3:
		dev_dbg(&client->device, "found an AG610\n");
		/*Read the CHIP ID from SCU required by GNSS driver*/
		ag6x0_ioread(client, SCU_CHIP_ID(ag6x0->scu_io_phys), &chipid);
		/*15:8 : Chip Identification Num ,7:0 -> Chip Revision Num*/
		chipid &= (SCU_CHIP_ID_CHID_MASK | SCU_CHIP_ID_CHREV_MASK);
		dev_dbg(&client->device, "AG610 Chip ID is %x\n", chipid);
		break;
	case 4:
		dev_dbg(&client->device, "found an AG620\n");
		/* DMA Request from WLAN is cleared */
		ag6x0_atomic_iowrite(client, SCU_SP_PUR(ag6x0->scu_io_phys),
							BIT(1), BIT(1));
		/*Read the CHIP ID from SCU required by GNSS driver*/
		ag6x0_ioread(client,
				SCU_CHIP_ID(ag6x0->scu_io_phys),
				&chipid);
		/*15:8 -> Chip Identification Num ,7:0 -> Chip Revision Num*/
		chipid &= (SCU_CHIP_ID_CHID_MASK | SCU_CHIP_ID_CHREV_MASK);
		dev_dbg(&client->device, "AG620 Chip ID is %x\n", chipid);
		break;
	}

	/*chipid will be zero if above switch case falls to default*/
	ag6x0->client_device->chipid = chipid;

	err = ag6x0_request_irqs(client);
	if (err)
		pr_err("\nError in setting up AGOLD620 irq's\n");

	AG6x0_EXIT;
	return err;

no_match:
	iounmap(ag6x0->ctrl_io);

no_ioremap:
	ag6x0_unmap_io(client, ag6x0);
	kfree(ag6x0);

	AG6x0_EXIT;
	return err;
}				/* ag6x0_probe() */

/**
 * ag6x0_remove - Clean up the ag6x0 device.
 * @dev: the device to work on.
 *
 * Do NOT call kfree on the p_device.  The release function in idi_bus.c
 * will take care of it.
 */
static int ag6x0_remove(struct idi_client_device *client)
{
	struct ag6x0_client *ag6x0;

	AG6x0_ENTER;

	ag6x0 = dev_get_drvdata(&client->device);
	dev_set_drvdata(&client->device, NULL);
	ag6x0_free_irqs(client);
	tasklet_kill(&ag6x0->isr_tasklet);
	ag6x0_unmap_io(client, ag6x0);
	iounmap(ag6x0->ctrl_io);
	kzfree(ag6x0);

	return 0;
}				/* ag6x0_remove() */

static const struct of_device_id ag6x0_match[] = {
	{.compatible = "intel,abb",},
	{.type = "agold",},
};

struct idi_client_driver ag6x0_client_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "AG6x0",
		.of_match_table = ag6x0_match,
	},
		.probe = ag6x0_probe,
		.remove = ag6x0_remove,
};

static int __init ag6x0_init(void)
{
	pr_info("IDI AG6x0 client init\n");
	return idi_register_client_driver(&ag6x0_client_driver);
}

static void __exit ag6x0_exit(void)
{
	pr_debug("IDI AG6x0 client exit\n");
	idi_unregister_client_driver(&ag6x0_client_driver);
}

module_init(ag6x0_init);
module_exit(ag6x0_exit);
