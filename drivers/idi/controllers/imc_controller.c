/*
 * imc_controller.c
 *
 * Implements the Intel Mobile Communications (IMC) IDI driver.
 * This module is base on the Intel intel_mid_hsi.c module.
 *
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 * Copyright (C) 2012 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 */
/*
 * Comments on the code:
 *
 * We cannot turn the IDI kernel off if there is an Audio path open.
 * How can we determine this?
 *
 * Sleep mode can be determined from lack of TX/RX requests, but there
 * can also be transparent register access which is NOT seen by the TX/RX
 * path.  pm_runtime_get() should be used by peripherals before doing a
 * transparent register access.
 *
 * Ring buffer and Double buffer flag to automatically resubmit read buffer?
 *
 */

/* Set the following if wanting to schedule a later suspend on idle state */
/* FIXME #define SCHEDULE_LATER_SUSPEND_ON_IDLE */

/* Set the following to disable the power management (for debugging) */
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_bus.h>

#if defined(CONFIG_FM)
#define IDI_FM_SUPPORT
#include <linux/fm/iui_fm.h>
#endif

#include <linux/idi/idi_debug.h>

#include "imc_controller.h"
#define IDI_IMC_ENTER pr_debug("--> %s\n", __func__)
#define IDI_IMC_EXIT  pr_debug("<-- %s\n", __func__)

#define DRVNAME "IMC IDI"
#define IDI_MPU_IRQ_NAME  "IDI_CONTROLLER_IRQ"

#define IDLE_POLL_JIFFIES (usecs_to_jiffies(10000))	/* 10 ms */
#define IDLE_TO_SUSPEND_DELAY 10	/* 10 ms */

/* This maps each channel to a single bit in DMA and IDI channels busy fields */
#define XFER_BUSY(ch)   (1<<(ch))
#define QUEUE_BUSY(ch)  (1<<(ch))

#define IN_BETWEEN(val, left, right) (((val) > (left) && (val) < (right)))

/* TX, RX and PM states */
enum {
	TX_SLEEPING,
	TX_READY
};

enum {
	IDI_PM_ASYNC,
	IDI_PM_SYNC
};

enum {
	RX_SLEEPING,
	RX_READY,
	RX_CAN_SLEEP
};

	enum {
	DEVICE_READY,
	DEVICE_SUSPENDED,
	DEVICE_AND_IRQ_SUSPENDED
};

#define IDI_DUMMY_RX_BUFFER_SIZE (IDI_MAX_CHANNEL * sizeof(int))
#define IDI_DUMMY_TX_BUFFER_SIZE (IDI_MAX_CHANNEL * sizeof(int))

/*
 * Disable the following to prevent ACWAKE toggling (IDI PM)
 * (for debugging purpose only)
 * This also disbales runtime power management
 * IDI PM is enabled by default
 */
static unsigned int idi_pm = 1;
module_param(idi_pm, uint, 0644);

static irqreturn_t imc_isr(int irq, void *imc);
static void imc_controller_enable_irqs(struct imc_controller *imc_idi);
static void imc_controller_disable_irqs(struct imc_controller *imc_idi);
static void imc_controller_disable_irqs_nosync(struct imc_controller *imc_idi);
static int imc_xfer_complete(struct imc_controller *imc_idi, int channel,
					int tx_not_rx, bool from_client);
static void imc_fwd_tasklet(unsigned long imc);
/*
 * Caller must take care of lock
 * channel: The IDI channel to mask/unmask
 * Returns Channel mask values
 */
static unsigned imc_idi_set_irq_channel_mask(void __iomem *ctrl,
						unsigned channel, bool set)
{
	unsigned reg = ioread32(ctrl);

	if (set)
		reg |= (1 << channel);
	else
		reg &= ~(1 << channel);

	iowrite32(reg, ctrl);

	return reg;
}

static inline unsigned imc_idi_mask_tx_irq_ch(struct imc_controller *imc_idi,
							 unsigned channel)
{
	void __iomem *ctrl = imc_idi->ctrl_io;

	return imc_idi_set_irq_channel_mask(IMC_IDI_TXMASK_CON(ctrl),
								channel, 0);
}

static inline unsigned imc_idi_unmask_tx_irq_ch(struct imc_controller *imc_idi,
							unsigned channel)
{
	void __iomem *ctrl = imc_idi->ctrl_io;

	return imc_idi_set_irq_channel_mask(IMC_IDI_TXMASK_CON(ctrl),
								channel, 1);
}

static inline unsigned imc_idi_mask_rx_irq_ch(struct imc_controller *imc_idi,
							unsigned channel)
{
	void __iomem *ctrl = imc_idi->ctrl_io;

	return imc_idi_set_irq_channel_mask(IMC_IDI_RXMASK_CON(ctrl),
								channel, 0);
}

static inline unsigned imc_idi_unmask_rx_irq_ch(struct imc_controller *imc_idi,
							unsigned channel)
{
	void __iomem *ctrl = imc_idi->ctrl_io;

	return imc_idi_set_irq_channel_mask(IMC_IDI_RXMASK_CON(ctrl),
								channel, 1);
}

/*
   imc_idi_dump_channel - Dump channel registers
*/
static void imc_idi_dump_channel(struct imc_controller *imc_idi,
				unsigned  channel, unsigned tx_not_rx)
{
	struct idi_controller_device *idi =
		to_idi_controller_device(imc_idi->dev);
	struct idi_client_device *client =
		to_idi_client_device(idi->client);

	dev_dbg(imc_idi->dev, " Dumping %s Channel %d:\n",
				 tx_not_rx ? "TX" : "RX", channel);
	if (tx_not_rx) {
		dev_dbg(imc_idi->dev, "\t Base: %x\n",
			ioread32(IMC_IDI_TXCH_BASE(imc_idi->ctrl_io,
								channel)));
		dev_dbg(imc_idi->dev, "\t Size: %x\n",
			ioread32(IMC_IDI_TXCH_SIZE(imc_idi->ctrl_io,
								channel)));
		dev_dbg(imc_idi->dev, "\t Control: %x\n",
			ioread32(IMC_IDI_TXCH_WR_CON(imc_idi->ctrl_io,
								channel)));
		dev_dbg(imc_idi->dev, "\t Stat: %x\n",
			ioread32(IMC_IDI_TXCH_RD_STAT(imc_idi->ctrl_io,
								channel)));
		dev_dbg(imc_idi->dev, "\t IRQ: %x\n",
			ioread32(IMC_IDI_TXCH_IRQ_CON(imc_idi->ctrl_io,
								channel)));
		dev_dbg(imc_idi->dev, "\t Next: %x\n",
			ioread32(IMC_IDI_TXCH_NEXT(imc_idi->ctrl_io,
								channel)));
	} else {
		dev_dbg(imc_idi->dev, "\t Base: %x\n",
			ioread32(IMC_IDI_RXCH_BASE(imc_idi->ctrl_io,
								channel)));
		dev_dbg(imc_idi->dev, "\t Size: %x\n",
			ioread32(IMC_IDI_RXCH_SIZE(imc_idi->ctrl_io,
								channel)));
		dev_dbg(imc_idi->dev, "\t Control: %x\n",
			ioread32(IMC_IDI_RXCH_RD_CON(imc_idi->ctrl_io,
								channel)));
		dev_dbg(imc_idi->dev, "\t Stat: %x\n",
			ioread32(IMC_IDI_RXCH_WR_STAT(imc_idi->ctrl_io,
								channel)));
		dev_dbg(imc_idi->dev, "\t IRQ: %x\n",
			ioread32(IMC_IDI_RXCH_IRQ_CON(imc_idi->ctrl_io,
								channel)));
		dev_dbg(imc_idi->dev, "\t Next: %x\n",
			ioread32(IMC_IDI_RXCH_NEXT(imc_idi->ctrl_io,
								channel)));
	}

	client->dump_channel(client, channel, tx_not_rx);
}

/**
 * is_in_tx_frame_mode - checks if the IDI controller is set in frame mode
 * @imc_idi: IMC IDI controller reference
 *
 * Returns 0 if in stream mode or IMC_IDI_TXCONF_TRANS if in frame mode.
 */
static inline int is_in_tx_frame_mode(struct imc_controller *imc_idi)
{
	IDI_IMC_ENTER;
	return imc_idi->txconf_cfg & IMC_IDI_TXCONF_TRANS;
}

/**
 * imc_set_interrupt - enabling and signalling interrupts
 * @ctrl: the IO-based address of the IDI hardware
 * @irq_enable: the bitfield of interrupt sources to enable
 */
static inline void imc_set_interrupt(void __iomem *ctrl,
				     struct imc_controller *imc_idi)
{
	unsigned tmp = 0;
	IDI_IMC_ENTER;
	/*
	 * A NULL pointer indicates disable.
	 */
	if (imc_idi) {
		tmp = ioread32(IMC_IDI_IMSC(ctrl));
		tmp &= IMSC_SLAVE_MASK;
		tmp |= imc_idi->imsc_cfg;
		iowrite32(tmp, IMC_IDI_IMSC(ctrl));
		tmp = ioread32(IMC_IDI_IMSC1(ctrl));
		tmp &= IMSC1_SLAVE_MASK;
		tmp |= imc_idi->imsc1_cfg;
		iowrite32(tmp, IMC_IDI_IMSC1(ctrl));
		dev_dbg(imc_idi->dev, "imsc %x, imsc1 %x\n",
			ioread32(IMC_IDI_IMSC(ctrl)),
			ioread32(IMC_IDI_IMSC1(ctrl)));
	} else {
		tmp = ioread32(IMC_IDI_IMSC(ctrl));
		tmp &= IMSC_SLAVE_MASK;
		iowrite32(tmp, IMC_IDI_IMSC(ctrl));
		tmp = ioread32(IMC_IDI_IMSC1(ctrl));
		tmp &= IMSC1_SLAVE_MASK;
		iowrite32(tmp, IMC_IDI_IMSC1(ctrl));
		pr_debug("imsc %x, imsc1 %x\n",
			ioread32(IMC_IDI_IMSC(ctrl)),
			ioread32(IMC_IDI_IMSC1(ctrl)));
	}

}				/* imc_set_interrupts() */

/**
 * idi_is_resumed - checks if the IDI controller IP has been resumed or not
 * @imc_idi: Intel IMC IDI controller reference
 *
 * This helper function is returning a non-zero value if the IDI IP has been
 * resumed or 0 if still suspended.
 */
#if defined IDI_PM_RUNTIME
static __must_check int idi_is_resumed(struct imc_controller *imc_idi)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	unsigned long flags;
	int ip_resumed;

	IDI_IMC_ENTER;
	spin_lock_irqsave(&imc_idi->hw_lock, flags);
	ip_resumed = (imc_idi->suspend_state == DEVICE_READY);
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

	return ip_resumed;

}				/* idi_is_resumed() */
#endif

/**
 * assert_acwake - asserting the ACWAKE line status
 * @imc_idi: IMC IDI controller reference
 * @mode: mode of the ACWAKE assertion
 *
 * The actual ACWAKE assertion happens when tx_state was 0.
 * If the transmitter is asleep, enable it.
 *
 * This re-enables the channel manangement bit for TX.
 * Channel managment is disabled in tx_idle_poll(), NOT deassert_acwake().
 *
 * TODO: It is not clear if this is useful for IDI.  We can't shut down the
 * link because it is unknown when transparent register access will occur.
 * may get renamed to transmist_runtime_get() or something similar.
 *
 *
 */

static void assert_acwake(struct imc_controller *imc_idi, int mode)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	unsigned long flags;
	int do_wakeup = 0;

	IDI_IMC_ENTER;
	spin_lock_irqsave(&imc_idi->hw_lock, flags);
	if (idi_pm)
		do_wakeup = (imc_idi->tx_state == TX_SLEEPING);

	imc_idi->tx_state++;
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
}				/* assert_acwake() */

/**
 * deassert_acwake - de-asserting the ACWAKE line status
 * @imc_idi: IMC IDI controller reference
 *
 * The actual ACWAKE de-assertion happens only when tx_state reaches 0.
 *
 * Returns 1 on success or 0 if the tx_state count was already 0.
 */
static int deassert_acwake(struct imc_controller *imc_idi)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	unsigned long flags;

	IDI_IMC_ENTER;
	spin_lock_irqsave(&imc_idi->hw_lock, flags);

	/*
	 * The deassert AC wake function is also used in the release function
	 * so that it can be called more times than expected should some
	 * stop_tx calls happen simultaneously. This makes the code cleaner and
	 * more robust.
	 */
	if (imc_idi->tx_state <= TX_SLEEPING) {
		imc_idi->tx_state = TX_SLEEPING;
		spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
		return 0;
	}

	--imc_idi->tx_state;
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

	return 1;

}

/**
 * tx_idle_poll - polling the TX FIFO emptiness
 * @param: hidden Intel IDI controller reference
 *
 * This polling timer is activated whenever tx_state is sleeping, and
 * re-activated until all internal TX FIFO are empty.
 */

static void tx_idle_poll(unsigned long param)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	struct imc_controller *imc_idi = (struct imc_controller *)param;
	unsigned long flags;
	int do_sleep = 0;

	IDI_IMC_ENTER;
	spin_lock_irqsave(&imc_idi->hw_lock, flags);

	if (idi_pm) {
		void __iomem *ctrl = imc_idi->ctrl_io;

		if (imc_idi->tx_state != TX_SLEEPING) {
			/* This timer has been called when tx_state == 0,
			 * if no longer to 0, then the pm_runtime_put has
			 * still to be taken */
			do_sleep = 1;
			goto tx_poll_out;
		}

		/*
		 * Prevent TX sleep and ACWAKE de-assertion until not empty
		 * If any part of the transmit pipe is busy, don't allow sleep.
		 */
		do_sleep = ((ioread32(IMC_IDI_TXSTAT(ctrl)) &
			     IMC_IDI_TXSTAT_BSY_MASK) == 0);
	} else {
		mod_timer(&imc_idi->tx_idle_poll, jiffies + IDLE_POLL_JIFFIES);
	}
tx_poll_out:
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

}				/* tx_idle_poll() */

/**
 * rx_idle_poll - polling the RX FIFO emptiness and CAWAKE line status
 * @param: hidden IMC IDI controller reference
 *
 * This polling timer is activated on CAWAKE rising interrupt, and re-activated
 * until the CAWAKE is low and all internal RX FIFO are empty.
 */
static void rx_idle_poll(unsigned long param)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	struct imc_controller *imc_idi = (struct imc_controller *)param;
	void __iomem *ctrl = imc_idi->ctrl_io;
	int schedule_rx_sleep = 0;
	unsigned long flags;

	IDI_IMC_ENTER;
	spin_lock_irqsave(&imc_idi->hw_lock, flags);

	if (unlikely(imc_idi->rx_state != RX_READY)) {
		spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
		return;
	}

	/* This shall almost never happen, but timer will be restarted on
	 * resume anyway */
	if (unlikely(imc_idi->suspend_state != DEVICE_READY)) {
		dev_warn(imc_idi->dev, "imc: poll whilst suspended !\n");
		spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
		return;
	}

	/*
	 * Prevent RX side disabling as long as CAWAKE is asserted or any RX
	 * is occurring.
	 */
	if (ioread32(IMC_IDI_RXSTAT(ctrl)) & (IMC_IDI_RXSTAT_BSY_MASK))
		mod_timer(&imc_idi->rx_idle_poll, jiffies + IDLE_POLL_JIFFIES);
	else {
		imc_idi->imsc_cfg |= IMC_IDI_IMSC_WAKE;
		imc_set_interrupt(ctrl, imc_idi);
		imc_idi->rx_state = RX_CAN_SLEEP;
		schedule_rx_sleep = 1;
	}
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

	if (schedule_rx_sleep)
		tasklet_hi_schedule(&imc_idi->isr_tasklet);

}

#ifdef IDLE_POLL
/**
 * idle_poll - polling the device for idleness
 * @param: hidden Intel IDI controller reference
 *
 * This polling timer is activated whenever the IDI device indicates that it
 * is idle.  If everything is quiet, we will head towards sleep.  If not
 * re-enable the IDLE interrupt.
 */
static void idle_poll(unsigned long param)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	tx_idle_poll(param);
	rx_idle_poll(param);
}				/* idle_poll() */
#endif

/**
 * has_enabled_acready - enable the ACREADY line
 * @imc_idi: IMC IDI controller reference
 *
 * Returns 1 if enabled or 0 if already enabled.
 */
static int has_enabled_acready(struct imc_controller *imc_idi)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	unsigned long flags;
	int do_wakeup;

	IDI_IMC_ENTER;
	spin_lock_irqsave(&imc_idi->hw_lock, flags);

	/* Do not re-wakeup the device if already woken up */
	do_wakeup = (imc_idi->rx_state == RX_SLEEPING);

	if ((do_wakeup) && (imc_idi->suspend_state == DEVICE_READY))
		mod_timer(&imc_idi->rx_idle_poll, jiffies + IDLE_POLL_JIFFIES);

	imc_idi->rx_state = RX_READY;
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

	return do_wakeup;

}				/* has_enabled_acready() */

/**
 * has_disabled_acready - try to disable the ACREADY line
 * @imc_idi: IMC IDI controller reference
 *
 * The actual RX disable can only happen if the CAWAKE line was low and all
 * RX hardware FIFO are empty, in which case rx_state has been set to
 * RX_CAN_SLEEP.
 *
 * Returns 1 if disabled or 0 if not.
 */
static int has_disabled_acready(struct imc_controller *imc_idi)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	void __iomem *ctrl = imc_idi->ctrl_io;
	int do_sleep = 0;
	unsigned long flags;

	IDI_IMC_ENTER;
	spin_lock_irqsave(&imc_idi->hw_lock, flags);
	if (imc_idi->rx_state != RX_CAN_SLEEP) {
		spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
		return 0;
	}

	/*
	 * If the receive side is NOT busy, we can go to sleep.
	 */
	do_sleep = !(ioread32(IMC_IDI_RXSTAT(ctrl)) & IMC_IDI_RXSTAT_BSY_MASK);

	/*
	 * Or we can look at the kernel idle bit.  Need to try both.
	 *
	 * do_sleep = ioread32(IMC_IDI_CLC_STAT(ctrl)) & IMC_IDI_CLC_STAT_KID);
	 */
	if (likely(do_sleep)) {
		imc_idi->rx_state = RX_SLEEPING;
	} else {
		mod_timer(&imc_idi->rx_idle_poll, jiffies + IDLE_POLL_JIFFIES);
		imc_idi->mis_status &= ~IMC_IDI_IMSC_WAKE;
		imc_idi->rx_state = RX_READY;
	}
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

	if (do_sleep) {
		/*
		 * FIXME: is 2 a valid wait time?  Need to calcluate at 200Mhz?
		 * Wait for 2 us (more than 1 IDI frame at 20 MHz) to ensure
		 * that the CAREADY will not rise back too soon
		 */
		udelay(2);
	}

	return do_sleep;

}				/* has_disabled_acready() */

/**
 * force_disable_acready - force disable of the ACREADY line
 * @imc_idi: IMC IDI controller reference
 */
static void force_disable_acready(struct imc_controller *imc_idi)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	void __iomem *ctrl = imc_idi->ctrl_io;
	struct idi_controller_device *idi =
	    to_idi_controller_device(imc_idi->dev);
	int do_sleep = 0;
	unsigned long flags;

	IDI_IMC_ENTER;
	spin_lock_irqsave(&imc_idi->hw_lock, flags);
	do_sleep = (imc_idi->rx_state != RX_SLEEPING);
	if (do_sleep)
		imc_idi->rx_state = RX_SLEEPING;


	/*
	 * Prevent the ACREADY change because of the CAWAKE toggling.
	 * The CAWAKE event interrupt shall be re-enabled whenever the
	 * RX fifo is no longer empty
	 */
	del_timer(&imc_idi->rx_idle_poll);
	imc_idi->mis_status &= ~IMC_IDI_IMSC_WAKE;
	imc_idi->imsc_cfg &= ~IMC_IDI_IMSC_WAKE;
	if (likely(imc_idi->suspend_state == DEVICE_READY)) {
		iowrite32(IMC_IDI_IMSC_WAKE, IMC_IDI_ICR(ctrl));
		imc_set_interrupt(ctrl, imc_idi);
	}
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

	if (do_sleep)
		idi_event(idi, IDI_EVENT_STOP_RX);
}

/**
 * unforce_disable_acready - unforce a previously disabled ACREADY line
 * @imc_idi: IMC IDI controller reference
 */
static void unforce_disable_acready(struct imc_controller *imc_idi)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	void __iomem *ctrl = imc_idi->ctrl_io;
	unsigned long flags;
	IDI_IMC_ENTER;

	spin_lock_irqsave(&imc_idi->hw_lock, flags);
	if (unlikely((imc_idi->rx_state == RX_SLEEPING) &&
		     (!(imc_idi->imsc_cfg & IMC_IDI_IMSC_WAKE)))) {
		imc_idi->imsc_cfg |= IMC_IDI_IMSC_WAKE;
		if (imc_idi->suspend_state == DEVICE_READY)
			imc_set_interrupt(ctrl, imc_idi);
	}
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
}				/* unforce_disble_acready() */

/**
 * imc_disable_in_progress_xfer - disable all transfers.
 * @imc_idi: controller reference
 */
static inline void imc_disable_in_progress_xfer(struct imc_controller *imc_idi)
{
	int i;

	IDI_IMC_ENTER;

	/*
	 * Reset all channels.
	 */
	for (i = 0; i < IDI_MAX_CHANNEL; i++) {
		iowrite32((IMC_IDI_TXCH_WR_CON_RST_INT |
			   IMC_IDI_TXCH_WR_CON_RST),
			  IMC_IDI_TXCH_WR_CON(imc_idi->ctrl_io, i));
		iowrite32((IMC_IDI_RXCH_RD_CON_RST_INT |
			   IMC_IDI_RXCH_RD_CON_RST),
			  IMC_IDI_RXCH_RD_CON(imc_idi->ctrl_io, i));
	}

}				/* imc_disable_in_progress_xfer() */

/**
 * imc_ctrl_set_cfg - IDI controller hardware configure
 * @imc_idi: IMC IDI controller reference
 *
 * Program the hardware in accordance with the settings stored in the IDI
 * controller software structure.
 *
 * Only set the registers if the config is different the actual.
 *
 * Since the configuration setting shodul be set to the defaults, we should
 * probably never see anything change. (tx/rx mask bits?)
 *
 * Returns success or an error if it is not possible to reprogram the device.
 */
static int imc_ctrl_set_cfg(struct imc_controller *imc_idi)
{
	void __iomem *ctrl = imc_idi->ctrl_io;
#ifdef DO_WE_NEED_TO_DO_THIS
	u32 timeout;
#endif
	u32 reg1, reg2;
	unsigned long flags;
	int err = 0;

	IDI_IMC_ENTER;
	idi_debug_add_event(set_cfg, 0, NULL);

	/*
	 * If the module has been disabled, nothing has been configured yet !
	 * FIXME: This logic may not be correct.  Not sure why we would disable
	 * the module.
	 */
	if ((imc_idi->clc_cnt_cfg & IMC_IDI_CLC_CNT_RMC_MASK) == 0) {
		dev_dbg(imc_idi->dev, "imc_ctrl_set_cfg:no RMC\n");
		return 0;
	}

	spin_lock_irqsave(&imc_idi->hw_lock, flags);

#ifdef PAS_PAGE61
	/*
	 * The PAS says that "There is no need to change this configuration at
	 * any time."
	 */
	reg1 = ioread32(IMC_IDI_CLC_CNT(ctrl));
	if (reg1 != imc_idi->clc_cnt_cfg)
		iowrite32(imc_idi->clc_cnt_cfg, IMC_IDI_CLC_CNT(ctrl));
#endif

	reg1 = ioread32(IMC_IDI_TXCONF(ctrl));
	reg2 = ioread32(IMC_IDI_RXCONF(ctrl));

	if ((reg1 != imc_idi->txconf_cfg) || (reg2 != imc_idi->rxconf_cfg)) {
#ifdef DO_WE_NEED_TO_DO_THIS
		/*
		 * To configure RXCONF and TXCONF the module needs to be put in
		 * config mode.  To do this, we will STOP normal operations.
		 */
		iowrite32(IMC_IDI_CLC_RUN(IMC_IDI_CLC_RUN_STOP),
			  IMC_IDI_CLC(ctrl));
		timeout = 100000;
		while (timeout &&
		       (ioread32(IMC_IDI_CLC_STAT(ctrl)) &
			IMC_IDI_CLC_STAT_RUN))
			timeout--;
		if (timeout == 0) {
			dev_info(imc_idi->dev,
				"imc_ctrl_set_cfg: failed to go to config mode\n");
			err = -EIO;
			goto unlock;
		}
#endif

		iowrite32(imc_idi->txconf_cfg, IMC_IDI_TXCONF(ctrl));
		iowrite32(imc_idi->rxconf_cfg, IMC_IDI_RXCONF(ctrl));

#ifdef DO_WE_NEED_TO_DO_THIS
		iowrite32(IMC_IDI_CLC_RUN(IMC_IDI_CLC_RUN_RUN),
			  IMC_IDI_CLC(ctrl));
		timeout = 1;
		while (timeout &&
		       (ioread32(IMC_IDI_CLC_STAT(ctrl)) & IMC_IDI_CLC_STAT_RUN)
		       == 0)
			timeout--;
		if (timeout == 0) {
			dev_info(imc_idi->dev,
				"imc_ctrl_set_cfg: failed to go to run mode\n");
			err = -EIO;
			goto unlock;
		}
#endif
	}

	reg1 = ioread32(IMC_IDI_TXCON(ctrl));
	if (reg1 != imc_idi->txcon_cfg)
		iowrite32(imc_idi->txcon_cfg, IMC_IDI_TXCON(ctrl));

	reg1 = ioread32(IMC_IDI_RXCON(ctrl));
	if (reg1 != imc_idi->rxcon_cfg)
		iowrite32(imc_idi->rxcon_cfg, IMC_IDI_RXCON(ctrl));

	reg1 = ioread32(IMC_IDI_EXT_CON(ctrl));
	if (reg1 != imc_idi->ext_con_cfg)
		iowrite32(imc_idi->ext_con_cfg, IMC_IDI_EXT_CON(ctrl));


	/*
	 * txmask_cfg and rxmask_cfg will be set when peripherals send data.
	 * Make sure they are consistant.
	 * tx3mask_cfg and rx3mask_cfg are fixed, and will need to be set so
	 * the LPE works correctly.
	 */
	reg1 = ioread32(IMC_IDI_TXMASK_CON(ctrl));
	if (reg1 != imc_idi->txmask_cfg)
		iowrite32(imc_idi->txmask_cfg, IMC_IDI_TXMASK_CON(ctrl));

	reg1 = ioread32(IMC_IDI_RXMASK_CON(ctrl));
	if (reg1 != imc_idi->rxmask_cfg)
		iowrite32(imc_idi->rxmask_cfg, IMC_IDI_RXMASK_CON(ctrl));

	reg1 = ioread32(IMC_IDI_TX2MASK_CON(ctrl));
	if (reg1 != imc_idi->tx2mask_cfg)
		iowrite32(imc_idi->tx2mask_cfg, IMC_IDI_TX2MASK_CON(ctrl));

	reg1 = ioread32(IMC_IDI_RX2MASK_CON(ctrl));
	if (reg1 != imc_idi->rx2mask_cfg)
		iowrite32(imc_idi->rx2mask_cfg, IMC_IDI_RX2MASK_CON(ctrl));

	reg1 = ioread32(IMC_IDI_TX3MASK_CON(ctrl));
	if (reg1 != imc_idi->tx3mask_cfg)
		iowrite32(imc_idi->tx3mask_cfg, IMC_IDI_TX3MASK_CON(ctrl));

	reg1 = ioread32(IMC_IDI_RX3MASK_CON(ctrl));
	if (reg1 != imc_idi->rx3mask_cfg)
		iowrite32(imc_idi->rx3mask_cfg, IMC_IDI_RX3MASK_CON(ctrl));

	reg1 = ioread32(IMC_IDI_ERR_CON(ctrl));
	if (reg1 != imc_idi->err_con_cfg)
		iowrite32(imc_idi->err_con_cfg, IMC_IDI_ERR_CON(ctrl));

#ifdef DO_WE_NEED_TO_DO_THIS
	/*
	 * It is not clear that we need to:
	 * Enable the module.
	 */
	iowrite32(IMC_IDI_CLC_MOD_EN(IMC_IDI_CLC_MOD_EN_EN_REQ),
		  IMC_IDI_CLC(ctrl));
	timeout = 1;
	while (timeout &&
	       (ioread32(IMC_IDI_CLC_STAT(ctrl)) & IMC_IDI_CLC_STAT_MODEN) == 0)
		timeout--;
	if (timeout == 0) {
		dev_info(imc_idi->dev,
			"imc_ctrl_set_cfg: failed to enable module\n");
		err = -EIO;
		goto unlock;
	}
#endif

	/*
	 * Enable the TX and RX link and channel layers.  We need to do this so
	 * transparent register access works.
	 * NOTE: EN_LNK and EN_LOG should NOT be disabled
	 * NOTE: we may need only EN_LNK for TR?
	 */
	imc_idi->txcon_cfg |= (IMC_IDI_TXCON_EN_CH | IMC_IDI_TXCON_EN_LNK
				| IMC_IDI_TXCON_MASTER);
	iowrite32(imc_idi->txcon_cfg, IMC_IDI_TXCON(ctrl));

	imc_idi->rxcon_cfg |= (IMC_IDI_RXCON_EN_CH | IMC_IDI_RXCON_EN_LOG);
	iowrite32(imc_idi->rxcon_cfg, IMC_IDI_RXCON(ctrl));

	/*
	 * Start the CAWAKE poll mechanism if RX is enabled.
	 * FIXME: is this the right thing to check?
	 * NOTE: we don't use the WAKE_WAKE signal.  Instead we will be resumed
	 * from an idle state, so this is probably obsolete code.
	 */
	if (imc_idi->rxconf_cfg & IMC_IDI_RXCONF_WAKE_WAKE) {
		dev_dbg(imc_idi->dev, "mod_timer\n");
		mod_timer(&imc_idi->rx_idle_poll, jiffies + IDLE_POLL_JIFFIES);
	}

	/* Enable the interrupts */
	imc_set_interrupt(ctrl, imc_idi);

#ifdef DO_WE_NEED_TO_DO_THIS
unlock:
#endif
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

	/*
	 * And we are done.
	 */
	return err;

}				/* imc_ctrl_set_cfg() */

/**
 * imc_ctrl_clean_reset - quick and clean hardware halt
 * @imc_idi: IMC IDI controller reference
 */
static void imc_ctrl_clean_reset(struct imc_controller *imc_idi)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	void __iomem *ctrl = imc_idi->ctrl_io;
	unsigned long flags;


	IDI_IMC_ENTER;
	/* Disable the interrupt line */
	imc_controller_disable_irqs(imc_idi);

	/* Deassert ACWAKE and ACREADY as shutting down */
	while (deassert_acwake(imc_idi))
		;

	force_disable_acready(imc_idi);

#ifdef IDLE_POLL
	/*
	 * Remove the IDLE poll timer
	 */
	del_timer_sync(&imc_idi->idle_poll);
#endif

	/* Disable (and flush) all tasklets */
	tasklet_disable(&imc_idi->isr_tasklet);
	tasklet_disable(&imc_idi->fwd_tasklet);

	spin_lock_irqsave(&imc_idi->hw_lock, flags);

	/* If suspended then there is nothing to do on the hardware side */
	if (imc_idi->suspend_state != DEVICE_READY)
		goto exit_clean_reset;

	/* Disable any in progress transactions */
	imc_disable_in_progress_xfer(imc_idi);

	/* Disable IRQ */
	imc_set_interrupt(ctrl, 0);

exit_clean_reset:
	imc_idi->xfer_running = 0;
	imc_idi->mis_status = 0;
	imc_idi->clc_cnt_cfg = 0;
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

#ifdef DO_WE_NEED_TO_DO_THIS
	/*
	 * Disable the module.
	 */
	iowrite32(IMC_IDI_CLC_MOD_EN(IMC_IDI_CLC_MOD_EN_DIS_REQ),
		  IMC_IDI_CLC(ctrl));
	timeout = 100000;
	while (timeout &&
		(ioread32(IMC_IDI_CLC_STAT(ctrl)) & IMC_IDI_CLC_STAT_MODEN))
		timeout--;
	if (timeout == 0)
		dev_dbg(imc_idi->dev, "imc_ctrl_suspend: failed to stop IDI module\n");

	/*
	 * Stop the kernel.
	 */
	iowrite32(IMC_IDI_CLC_RUN(IMC_IDI_CLC_RUN_STOP), IMC_IDI_CLC(ctrl));
	timeout = 100000;
	while (timeout &&
	       (ioread32(IMC_IDI_CLC_STAT(ctrl)) & IMC_IDI_CLC_STAT_RUN))
		timeout--;
	if (timeout == 0)
		dev_dbg(imc_idi->dev, "imc_ctrl_suspend: failed to stop IDI kernel\n");
#endif

	while (deassert_acwake(imc_idi))
		;
	(void)has_disabled_acready(imc_idi);

	/* Re-enable all tasklets */
	tasklet_enable(&imc_idi->fwd_tasklet);
	tasklet_enable(&imc_idi->isr_tasklet);

	/* Do not forget to re-enable the interrupt */
	 imc_controller_enable_irqs(imc_idi);

}				/* imc_ctrl_clean_reset() */

/**
 * imc_ctrl_full_reset - hardware soft reset
 * @imc_idi: IMC IDI controller reference
 *
 * Returns 0
 */
static int imc_ctrl_full_reset(struct imc_controller *imc_idi)
{
	void __iomem *ctrl = imc_idi->ctrl_io;
	int channel;
	unsigned *idi_dummy_tx_buffer = imc_idi->txbuf_dummy;
	unsigned *idi_dummy_rx_buffer = imc_idi->rxbuf_dummy;
	dma_addr_t idi_dummy_tx_buffer_dma = imc_idi->txbuf_dummy_dma;
	dma_addr_t idi_dummy_rx_buffer_dma = imc_idi->rxbuf_dummy_dma;

	IDI_IMC_ENTER;
	/*
	 * Set TX/RX pointers to dummy data buffer for the unitialized channels.
	 * This is needed to avoid continous interrupts from the data moving
	 * logic.  Use dummy rx buffers as long as to the channel is no real
	 * buffer assigned.  This avoids possible memory corruption
	 */
	memset(idi_dummy_tx_buffer, 0xAF, IDI_DUMMY_TX_BUFFER_SIZE);
	dev_dbg(imc_idi->dev, "sizeof(dummy_tx) = %ud\n",
						IDI_DUMMY_TX_BUFFER_SIZE);
	dev_dbg(imc_idi->dev, "sizeof(dummy_rx) = %ud\n",
						IDI_DUMMY_TX_BUFFER_SIZE);

	/*
	 * This was done based on the HSI IDI sample code.  It may be enough to
	 * just set WR_CON to 1 and IRQ_CON to 0. (same for RD_CON)
	 */
	dev_dbg(imc_idi->dev, "tx_buffer = 0x%p\n", idi_dummy_tx_buffer);
	dev_dbg(imc_idi->dev, "tx_buffer[0] = 0x%x\n",
		 idi_dummy_tx_buffer[0]);

	for (channel = 0; channel < IDI_MAX_CHANNEL; channel++) {

		iowrite32(idi_dummy_tx_buffer_dma,
			  IMC_IDI_TXCH_BASE(ctrl, channel));
		/* Set the Size to 0 before to reset the internal
			buffer, otherwise a packet is sent to Agold */
		iowrite32(0, IMC_IDI_TXCH_SIZE(ctrl, channel));
		iowrite32(3, IMC_IDI_TXCH_NEXT(ctrl, channel));

		iowrite32(0, IMC_IDI_TXCH_IRQ_CON(ctrl, channel));
		/* Reset the internal buffer */
		iowrite32(IMC_IDI_TXCH_WR_CON_RST_INT,
			  IMC_IDI_TXCH_WR_CON(ctrl, channel));
		udelay(10);
		/* Reset the external buffer */
		iowrite32(IMC_IDI_TXCH_WR_CON_RST,
			  IMC_IDI_TXCH_WR_CON(ctrl, channel));

		iowrite32(sizeof(u32), IMC_IDI_TXCH_SIZE(ctrl, channel));
		idi_dummy_tx_buffer_dma += sizeof(u32);
	}

	/*
	 * RX:
	 */
	memset(idi_dummy_rx_buffer, 0x5A, IDI_DUMMY_RX_BUFFER_SIZE);

	dev_dbg(imc_idi->dev, "rx_buffer    = 0x%p\n",
		 idi_dummy_rx_buffer);
	dev_dbg(imc_idi->dev, "rx_buffer[0] = 0x%x\n",
		 idi_dummy_rx_buffer[0]);

	for (channel = 0; channel < IDI_MAX_CHANNEL; channel++) {
		iowrite32(idi_dummy_rx_buffer_dma,
			  IMC_IDI_RXCH_BASE(ctrl, channel));
		iowrite32(sizeof(u32), IMC_IDI_RXCH_SIZE(ctrl, channel));
		iowrite32(0, IMC_IDI_RXCH_IRQ_CON(ctrl, channel));
		iowrite32(3, IMC_IDI_RXCH_NEXT(ctrl, channel));
		iowrite32(IMC_IDI_RXCH_RD_CON_RST |
				IMC_IDI_RXCH_RD_CON_RST_INT,
			  IMC_IDI_RXCH_RD_CON(ctrl, channel));
		idi_dummy_rx_buffer_dma += sizeof(u32);
	}

	/*
	 * Clear the transmit and receive paths.
	 */
	iowrite32((IMC_IDI_TXCON2_CLR_LOG | IMC_IDI_TXCON2_CLR_CH),
		  IMC_IDI_TXCON2(ctrl));
	iowrite32((IMC_IDI_RXCON2_CLR_LOG | IMC_IDI_RXCON2_CLR_CH),
		  IMC_IDI_RXCON2(ctrl));

	/*
	 * Clear any interrupts that are lying around.
	 */
	iowrite32(0xFFFFFFFF, IMC_IDI_ICR(ctrl));
	iowrite32(0x3FFF, IMC_IDI_ICR1(ctrl));

	iowrite32(IMC_IDI_TXIRQ_CON_CLR_ALL, IMC_IDI_TXIRQ_CON(ctrl));
	iowrite32(IMC_IDI_RXIRQ_CON_CLR_ALL, IMC_IDI_RXIRQ_CON(ctrl));

	return 0;

}				/* imc_ctrl_full_reset() */

#ifdef CONFIG_DEBUG_FS

static int imc_debug_show(struct seq_file *m, void *p)
{
	struct idi_controller_device *idi = m->private;
	struct imc_controller *imc_idi = idi_controller_get_drvdata(idi);
	void __iomem *ctrl = imc_idi->ctrl_io;
	unsigned int reg;

	IDI_IMC_ENTER;

	reg = ioread32(IMC_IDI_ID(ctrl));
	seq_printf(m, "REVISION: 0x%08x MODULE ID: 0x%08x  TOPSPIN: 0x%08x\n",
		   IMC_IDI_ID_REV_NUMBER(reg), IMC_IDI_ID_MOD_ID(reg),
		   IMC_IDI_ID_TS_REV_NR(reg));

	reg = ioread32(IMC_IDI_SWCID(ctrl));
	seq_printf(m, "Available Channels: %d\n",
		   (reg & IMC_IDI_SWCID_CH) ? 16 : 8);

	seq_printf(m, "SRB ID: 0x%08x\n",
		   ioread32(IMC_IDI_SRB_MSCONF_ID(ctrl)));
	seq_printf(m, "SRB ERR ID: 0x%08x\n",
		   ioread32(IMC_IDI_SRB_ERRCONF_ID(ctrl)));

	reg = ioread32(IMC_IDI_CLC_CNT(ctrl));
	seq_printf(m, "Clock SRM: 0x%08x  Clock ORMC: 0x%08x\n",
		   (reg & 0x000000FF), ((reg & 0x0000FF00) >> 8));

	seq_printf(m, "Clock Status: 0x%08x\n",
		   ioread32(IMC_IDI_CLC_STAT(ctrl)));
	seq_printf(m, "TX Configuration: 0x%08x\n",
		   ioread32(IMC_IDI_TXCONF(ctrl)));
	seq_printf(m, "TX Status: 0x%08x\n", ioread32(IMC_IDI_TXSTAT(ctrl)));
	seq_printf(m, "TX Control: 0x%08x\n", ioread32(IMC_IDI_TXCON(ctrl)));
	seq_printf(m, "TX Control2: 0x%08x\n", ioread32(IMC_IDI_TXCON2(ctrl)));
	seq_printf(m, "TX Debug: 0x%08x\n", ioread32(IMC_IDI_TXDEBUG(ctrl)));
	seq_printf(m, "TX IRQ Status: 0x%08x\n",
		   ioread32(IMC_IDI_TXIRQ_STAT(ctrl)));
	seq_printf(m, "TX IRQ Mask: 0x%08x\n",
		   ioread32(IMC_IDI_TXMASK_CON(ctrl)));
	seq_printf(m, "RX Configuration: 0x%08x\n",
		   ioread32(IMC_IDI_RXCONF(ctrl)));

	seq_printf(m, "RX Status: 0x%08x\n", ioread32(IMC_IDI_RXSTAT(ctrl)));
	seq_printf(m, "RX Control: 0x%08x\n", ioread32(IMC_IDI_RXCON(ctrl)));
	seq_printf(m, "RX Control2: 0x%08x\n", ioread32(IMC_IDI_RXCON2(ctrl)));
	seq_printf(m, "RX Debug: 0x%08x\n", ioread32(IMC_IDI_RXDEBUG(ctrl)));
	seq_printf(m, "RX IRQ Status: 0x%08x\n",
		   ioread32(IMC_IDI_RXIRQ_STAT(ctrl)));
	seq_printf(m, "RX IRQ Mask: 0x%08x\n",
		   ioread32(IMC_IDI_RXMASK_CON(ctrl)));

	seq_printf(m, "Channel 5: 0x%08x\n", ioread32(IMC_IDI_DEBUG5(ctrl)));
	seq_printf(m, "Channel 6: 0x%08x\n", ioread32(IMC_IDI_DEBUG6(ctrl)));
	seq_printf(m, "Channel 7: 0x%08x\n", ioread32(IMC_IDI_DEBUG7(ctrl)));
	seq_printf(m, "Channel 8: 0x%08x\n", ioread32(IMC_IDI_DEBUG8(ctrl)));
	seq_printf(m, "Channel 9: 0x%08x\n", ioread32(IMC_IDI_DEBUG9(ctrl)));
	seq_printf(m, "Channel 10: 0x%08x\n", ioread32(IMC_IDI_DEBUG10(ctrl)));

	seq_printf(m, "Error Control: 0x%08x\n",
		   ioread32(IMC_IDI_ERR_CON(ctrl)));
	seq_printf(m, "Error Control Ext: 0x%08x\n",
		   ioread32(IMC_IDI_EXT_CON(ctrl)));

	seq_printf(m, "Channel Assignment: 0x%08x\n",
		   ioread32(IMC_IDI_CHANNEL_CON(ctrl)));

	seq_printf(m, "Slave Mask: 0x%08x\n",
		   ioread32(IMC_IDI_SLMASK_CON(ctrl)));
	seq_printf(m, "Slave Mask 2: 0x%08x\n",
		   ioread32(IMC_IDI_SL2MASK_CON(ctrl)));

	seq_printf(m, "Interrupt Control: 0x%08x\n",
		   ioread32(IMC_IDI_IMSC(ctrl)));
	seq_printf(m, "Interrupt Control 1: 0x%08x\n",
		   ioread32(IMC_IDI_IMSC1(ctrl)));
	seq_printf(m, "Masked Interrupt Stat: 0x%08x\n",
		   ioread32(IMC_IDI_MIS(ctrl)));
	seq_printf(m, "Masked Interrupt Stat 1: 0x%08x\n",
		   ioread32(IMC_IDI_MIS1(ctrl)));

	return 0;

}

static int imc_debug_imsc(struct seq_file *m, void *p)
{
	struct idi_controller_device *idi = m->private;
	struct imc_controller *imc_idi = idi_controller_get_drvdata(idi);
	void __iomem *ctrl = imc_idi->ctrl_io;

	IDI_IMC_ENTER;

	iowrite32(0x00000010, IMC_IDI_MIS1(ctrl));

	imc_isr(0, imc_idi);

	return 0;

}

static int imc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, imc_debug_show, inode->i_private);
}

static int imc_imsc_open(struct inode *inode, struct file *file)
{
	return single_open(file, imc_debug_imsc, inode->i_private);
}

static const struct file_operations imc_regs_fops = {
	.open = imc_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations imc_imsc_fops = {
	.open = imc_imsc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init imc_debug_add_ctrl(struct idi_controller_device *idi)
{
	struct imc_controller *imc_idi = idi_controller_get_drvdata(idi);
	struct dentry *dir;

	IDI_IMC_ENTER;

	/*
	 * IDI controller
	 */
	dir = debugfs_create_dir(dev_name(&idi->device), NULL);
	if (IS_ERR(dir))
		return PTR_ERR(dir);

	debugfs_create_file("regs", S_IRUGO, dir, idi, &imc_regs_fops);
	debugfs_create_file("imsc", S_IRUGO, dir, idi, &imc_imsc_fops);

	imc_idi->dir = dir;

	return 0;

}
#endif /* CONFIG_DEBUG_FS */


static void imc_start_rx_bounce_xfer(struct idi_transaction *trans,
					struct imc_controller *imc_idi,
					struct imc_channel_ctx *channel_ctx)
{
	unsigned irq_less_or_equal_to_wr;
	void __iomem *ctrl = imc_idi->ctrl_io;
	u32 rd_pointer;
	u32 wr_pointer;
	u32 irq_pointer;
	u32 wrap = 0;
	u8 dat;
	unsigned ch_size_words = channel_ctx->size / sizeof(u32);

	/* Disable the channel interrupt */
	imc_idi_mask_rx_irq_ch(imc_idi, trans->channel);

	/* FIXME: Not sure we should clear the interrupt ? */

	/* Get Read Pointer */
	rd_pointer = ioread32(IMC_IDI_RXCH_RD_CON(ctrl, trans->channel));
	rd_pointer >>= IMC_IDI_RXCH_RD_CON_PTR_SHIFT;
	dev_dbg(imc_idi->dev, "rd_pointer = %x\n", rd_pointer);

	/* Compute and write the IRQ pointer */
	irq_pointer = rd_pointer +
			DIV_ROUND_UP(trans->idi_xfer.size, sizeof(u32));
	if (irq_pointer >= ch_size_words)
		irq_pointer -= ch_size_words;
	dev_dbg(imc_idi->dev, "Intial irq_pointer = %x\n", irq_pointer);
	iowrite32((irq_pointer << IMC_IDI_RXCH_IRQ_CON_PTR_SHIFT),
		IMC_IDI_RXCH_IRQ_CON(ctrl, trans->channel));

	/* Get Write Pointer */
	wr_pointer = ioread32(IMC_IDI_RXCH_WR_STAT(ctrl, trans->channel));
	dat = wr_pointer & IMC_IDI_RXCH_WR_STAT_DAT;
	wr_pointer >>= IMC_IDI_RXCH_WR_STAT_PTR_SHIFT;
	dev_dbg(imc_idi->dev, "wr_pointer = %x, DAT = %u\n", wr_pointer, dat);

	/* Check if wr_pointer has wrapped around */
	if (rd_pointer > wr_pointer)
		wrap = 1;

	/* If wr_pointer has wrapped around and reached rd_pointer,
	 * irq_pointer can never be greater than wr_pointer
	 */
	if (dat && (rd_pointer == wr_pointer))
		irq_less_or_equal_to_wr = 1;
	else
		irq_less_or_equal_to_wr = (irq_pointer == wr_pointer)
			|| ((!wrap) &&
				IN_BETWEEN(irq_pointer, rd_pointer, wr_pointer))
			|| (wrap &&
				(!IN_BETWEEN(irq_pointer, wr_pointer-1,
								rd_pointer+1)));

	/* Check if an interrupt is pending or due */
	if (((ioread32(IMC_IDI_RXIRQ_STAT(ctrl)) & (1 << trans->channel))
		&& irq_less_or_equal_to_wr) || !irq_less_or_equal_to_wr) {
		/* Int. pending AND irq_pointer<=wr_pointer */
		/* irq_pointer>wr_pointer (int. due) */

		/* We are good to go... */
		dev_dbg(imc_idi->dev,
			"Ready to hit channel RX interrupt for channel %u\n",
			trans->channel);
	} else {
		/* Oops... we have missed the interrupt */

		/* Force the interrupt to occur */
		dev_dbg(imc_idi->dev,
			"Missed channel RX interrupt on channel %u..."
				"forcing to trigger\n", trans->channel);
		while (!(ioread32(IMC_IDI_RXIRQ_STAT(ctrl))
					& (1 << trans->channel))) {
			wr_pointer = ioread32(IMC_IDI_RXCH_WR_STAT(ctrl,
							trans->channel));
			iowrite32(wr_pointer,
				IMC_IDI_RXCH_IRQ_CON(ctrl, trans->channel));
		}

		wr_pointer >>= IMC_IDI_RXCH_WR_STAT_PTR_SHIFT;
		dev_dbg(imc_idi->dev, "Final irq_pointer = %x\n", wr_pointer);
	}

	/* Unmask the channel and hit the interrupt (or leave it for hitting) */
	imc_idi_unmask_rx_irq_ch(imc_idi, trans->channel);

	/***** BEGIN DANGER ZONE (can be interrupted any time) *****/

	/* Cause the interrupt to trigger if we have missed it */
	if (ioread32(IMC_IDI_RXIRQ_STAT(ctrl)) & (1 << trans->channel))
		iowrite32(IMC_IDI_ISR_RX, IMC_IDI_ISR(ctrl));

	/***** END DANGER ZONE *****/
}

/**
 * do_imc_start_xfer - start/restart DMA data transfer helper function
 * @trans: reference to the message to transfer
 * @lch: DMA channel to consider
 * @imc_idi: IMC IDI controller reference
 * @resuming: flag stating if this is a first start or a resume restart
 */
static void do_imc_start_xfer(struct idi_transaction *trans,
			      struct imc_controller *imc_idi,
			      int resuming)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	struct idi_controller_device *idi;
	struct idi_client_device *client;
	struct imc_channel_ctx *channel_ctx;
	void __iomem *ctrl = imc_idi->ctrl_io;
	u32 mask;
	unsigned long flags;
#ifdef CODE_CLEANUP
	int nothing_to_do;
#endif
	int tx_not_rx = trans->t_type == IDI_TRANS_WRITE ? 1 : 0;
	u32 payload;

	IDI_IMC_ENTER;

	idi = to_idi_controller_device(imc_idi->dev);
	client = to_idi_client_device(idi->client);

	channel_ctx = (tx_not_rx) ?
	    &imc_idi->tx_ctx[trans->channel] : &imc_idi->rx_ctx[trans->channel];

	mask = 1 << trans->channel;

	spin_lock_irqsave(&imc_idi->hw_lock, flags);

	payload = round_up(trans->idi_xfer.size, sizeof(u32));
	dev_dbg(imc_idi->dev,
		"Pushing transaction %p on %s %d channel to controller\n",
			trans, (tx_not_rx ? "TX" : "RX"), trans->channel);

	/*
	 * All error checking should have been done before the transaction is
	 * queued, so I am not going to check again. Is this ok?  Need to make
	 * sure that all items queued have been error checked.
	 * What if an peripheral driver changes it?
	 */
	if (tx_not_rx) {
		/* Setup IRQ */
		/*
		 * Tx Irqs are triggered from * in * operation :
		 *           DBB for OUTSTANDING_READ channels
		 *           ABB for SOFTWARE_DEFINED channels
		 *	     DBB for DMA channels if IDI_TX_EARLY_IRQ set
		 *	     ABB for DMA channels if IDI_TX_EARLY_IRQ unset
		*/

		if (idi->channels[trans->channel] == SOFTWARE_DEFINED)
			client->set_interrupt(client, trans);

		if (trans->idi_xfer.base) {
			iowrite32(trans->idi_xfer.base,
				  IMC_IDI_TXCH_BASE(ctrl, trans->channel));

			iowrite32(payload,
				  IMC_IDI_TXCH_SIZE(ctrl, trans->channel));

			iowrite32(IMC_IDI_TXCH_WR_CON_RST,
				IMC_IDI_TXCH_WR_CON(ctrl, trans->channel));

			iowrite32(0,
				IMC_IDI_TXCH_WR_CON(ctrl, trans->channel));
			goto setup_done;
		}

		if (trans->idi_xfer.desc) {

			iowrite32((u32)(trans->idi_xfer.desc),
				IMC_IDI_TXCH_NEXT(ctrl, trans->channel));
			goto setup_done;
		}
		BUG();

	} else {/* RX side of things */

		if (trans->idi_xfer.base) {
			iowrite32((trans->idi_xfer.base),
				  IMC_IDI_RXCH_BASE(ctrl, trans->channel));
			iowrite32(trans->idi_xfer.size,
				  IMC_IDI_RXCH_SIZE(ctrl, trans->channel));

			iowrite32(IMC_IDI_RXCH_RD_CON_RST,
				  IMC_IDI_RXCH_RD_CON(ctrl, trans->channel));

			iowrite32(0,
				IMC_IDI_RXCH_RD_CON(ctrl, trans->channel));

			goto setup_done;
		}
		if (trans->idi_xfer.desc) {
			iowrite32((u32) (trans->idi_xfer.desc),
				IMC_IDI_RXCH_NEXT(ctrl, trans->channel));

			goto setup_done;
		}

		if (channel_ctx->base) {
			imc_start_rx_bounce_xfer(trans, imc_idi, channel_ctx);
			goto setup_done;
		}
		BUG();
	}			/* RX */

setup_done:
	imc_idi->xfer_running |= mask;
	imc_idi->xfer_resumed |= mask;
#ifdef CODE_CLEANUP
do_start_dma_done:
#endif
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

	if ((tx_not_rx) && (channel_ctx->start_tx))
		channel_ctx->start_tx(trans);

}				/* do_imc_start_xfer() */

/**
 * imc_start_xfer - start a data transfer
 * @trans: reference to the message to transfer
 * @lch: DMA channel to consider
 * @imc_idi: IMC IDI controller reference
 */
static void imc_start_xfer(struct idi_transaction *trans,
			   struct imc_controller *imc_idi)
{
	IDI_IMC_ENTER;
	idi_debug_add_event(start_xfer, 0, trans);

	do_imc_start_xfer(trans, imc_idi, 0);
}


/**
 * imc_transfer - starting transfer from TX or RX queue
 * @imc_idi: IMC IDI controller reference
 * @tx_not_rx: direction to consider (RX = 0, TX = anything else)
 * @imc_channel: IDI channel to consider
 *
 * TODO: The controller has a "basic transmission" mode.  This disables the
 *       channel controll method.  We need to add a flag to indicate basic
 *       transmission mode and update the code to allow for it.
 *
 *       IDI_TXCON:en_ch    (0) enables basic transmission (pauses channel
 *                          control)
 *       IDI_TXCON:channel  specifies the transfer channel
 *       IDI_TXD            The transmit data register.
 *       IDI_RXSTAT:channel The channel that basic mode received data on.
 *       IDI_RXSTAT:word    Data word was received by the link layer.
 *       IDI_RXCON:en_ch    (0) enables basic receive mode.
 *       IDI_RXD            The receive data register.
 *
 */
static void imc_transfer(struct imc_controller *imc_idi, int tx_not_rx,
			 unsigned int imc_channel)
__acquires(&imc_idi->sw_lock) __releases(&imc_idi->sw_lock)
{
	struct list_head *queue;
	struct idi_transaction *trans;
	unsigned long flags;

	IDI_IMC_ENTER;

	queue = (tx_not_rx) ?
	    &imc_idi->tx_queue[imc_channel] : &imc_idi->rx_queue[imc_channel];

	/*
	 * Get the spin lock
	 */
	spin_lock_irqsave(&imc_idi->sw_lock, flags);

	if (list_empty(queue)) {
		spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
		return;
	}

	/*
	 * If the first entry on the FIFO list is not QUEUED, we are currently
	 * working on it, so we are done.  The interrupt routine will start the
	 * next FIFO entry when the current one completes.
	 */
	trans = list_first_entry(queue, struct idi_transaction, link);
	if (trans->status != IDI_STATUS_QUEUED) {
		dev_dbg(imc_idi->dev, "Transaction %p will be pushed by ISR\n",
									trans);
		spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
		return;
	}

	trans->status = IDI_STATUS_PROCEEDING;

	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

	/* Assert ACWAKE (deasserted on complete or destruct) */
	if (tx_not_rx)
		assert_acwake(imc_idi, IDI_PM_ASYNC);
	else
		unforce_disable_acready(imc_idi);

	imc_start_xfer(trans, imc_idi);

}				/* imc_transfer() */


/**
 * imc_break_complete - break interrupt callback
 * @imc_idi: IMC IDI controller reference
 */
static void imc_break_complete(struct imc_controller *imc_idi)
__acquires(&imc_idi->sw_lock) __releases(&imc_idi->sw_lock)
{
	struct list_head *queue;
	struct list_head *node;
	struct idi_transaction *trans;
	unsigned long flags;

	dev_dbg(imc_idi->dev, "HWBREAK received\n");

	idi_debug_add_event(rx_break, 0, NULL);

	queue = &imc_idi->brk_queue;
	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	node = queue->next;
	while (node != queue) {
		trans = list_entry(node, struct idi_transaction, link);
		list_del(&trans->link);
		spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
		trans->status = IDI_STATUS_COMPLETE;
		trans->complete(trans);
		spin_lock_irqsave(&imc_idi->sw_lock, flags);
		node = queue->next;
	}
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

}				/* imc_break_complete() */

/**
 * imc_rx_error - handle RX error interrupt sources
 * @imc_idi: IMC IDI controller reference
 *
 * FIXME: the HAS has us reseting the system at this point.
 * What to do?
 */
static void imc_rx_error(struct imc_controller *imc_idi, int bits)
__acquires(&imc_idi->sw_lock) __releases(&imc_idi->sw_lock)
{
	IDI_IMC_ENTER;
	idi_debug_add_event(rx_error, 0, NULL);

	if (bits & IMC_IDI_MIS_TIME)
		dev_err(imc_idi->dev, "timeout\n");

	if (bits & IMC_IDI_MIS_STALL)
		dev_err(imc_idi->dev, "stall\n");

	if (bits & IMC_IDI_MIS_TRAIL)
		dev_err(imc_idi->dev, "trail\n");

	if (bits & IMC_IDI_MIS_ZERO)
		dev_err(imc_idi->dev, "zero\n");

#ifdef WONT_WORK
	struct list_head *queue;
	struct idi_transaction *trans = NULL;
	unsigned int i;
	unsigned long flags;
	/*
	 * How do we stop the peripherals from sending data?
	 * We could do a ag620_stop_tx() to shut down the TX pipe from AG620.
	 * Stop all current receives and mark them as errored.
	 * Clean buffers.
	 *
	 */
	for (i = 0; i < IDI_MAX_CHANNEL; i++) {
		queue = &imc_idi->rx_queue[i];
		spin_lock_irqsave(&imc_idi->sw_lock, flags);
		if (!list_empty(queue)) {
			trans =
			    list_first_entry(queue, struct idi_transaction,
					     link);
			trans->status = IDI_STATUS_ERROR;
		}
		spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
	}
#endif

}				/* imc_rx_error() */

/**
 * imc_async_break - send break message or queue break receive trans
 * @trans: reference to the IDI break message
 *
 * Return 0 if successful, -EINVAL if not in frame mode.
 */
static int imc_async_break(struct idi_transaction *trans)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
__acquires(&imc_idi->sw_lock) __releases(&imc_idi->sw_lock)
{
	struct idi_controller_device *idi = trans->context;
	struct imc_controller *imc_idi = idi_controller_get_drvdata(idi);
	void __iomem *ctrl = imc_idi->ctrl_io;
	unsigned long flags;

	IDI_IMC_ENTER;
	idi_debug_add_event(tx_break, 0, trans);

	/* Return an error if not in frame mode */
	if (unlikely(!is_in_tx_frame_mode(imc_idi)))
		return -EINVAL;

	if (trans->t_type == IDI_TRANS_WRITE) {
		assert_acwake(imc_idi, IDI_PM_ASYNC);
		spin_lock_irqsave(&imc_idi->hw_lock, flags);

		/*
		 * Send the break.
		 */
		iowrite32(IMC_IDI_TXCON2_BREAK, IMC_IDI_TXCON2(ctrl));

		/*
		 * Dummy read to force the write to happen.
		 * Then ensure that at least the minimal delay for a break
		 * sequence will be met.
		 */
		(void)ioread32(IMC_IDI_TXCON(ctrl));
		/* FIXME: need a value that will be valid
		 * for all clock speeds.
		 */
		imc_idi->brk_us_delay = 10;
		udelay(imc_idi->brk_us_delay);

		iowrite32(0, IMC_IDI_TXCON2(ctrl));

		spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
		trans->status = IDI_STATUS_COMPLETE;
		trans->complete(trans);
		(void)deassert_acwake(imc_idi);
	} else {
		spin_lock_irqsave(&imc_idi->sw_lock, flags);
		list_add_tail(&trans->link, &imc_idi->brk_queue);
		spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
	}

	return 0;

}				/* imc_async_break() */

/**
 * imc_mid_async - queue a IDI message and start transfer if possible
 * @trans: reference to the IDI message
 *
 * Validate the incoming transaction requst, and queue message to send when
 * possible.
 *
 * There are 5 methods for data transfer: ring buffer, double buffer, single
 * buffer, bounce buffer and scatter/gather.
 * Each channel may use a specific method.
 * Check the channel configuration (context), parameter check it and then
 * use appropriate method.
 *
 * Returns 0 if successful, -EINVAL if message pointer is NULL or channel
 * number is invalid, or transfer error if any.
 */
static int imc_mid_async(struct idi_transaction *trans)
__acquires(&imc_idi->sw_lock) __releases(&imc_idi->sw_lock)
{
	struct idi_controller_device *idi = trans->context;
	struct imc_controller *imc_idi = idi_controller_get_drvdata(idi);
	struct list_head *queue;
	unsigned short *queue_busy;
	unsigned int channel;
	int tx_not_rx;
	unsigned long flags;
	struct imc_channel_ctx *channel_ctx;

	IDI_IMC_ENTER;

	if (trans->break_frame)
		return imc_async_break(trans);

	channel = trans->channel;
	if (channel >= IDI_MAX_CHANNEL)
		return -EINVAL;

	if (trans->t_type == IDI_TRANS_WRITE) {
		tx_not_rx = 1;
		queue = &imc_idi->tx_queue[channel];
		queue_busy = &imc_idi->tx_queue_busy;
		channel_ctx = &imc_idi->tx_ctx[channel];
	} else {
		tx_not_rx = 0;
		queue = &imc_idi->rx_queue[channel];
		queue_busy = &imc_idi->rx_queue_busy;
		channel_ctx = &imc_idi->rx_ctx[channel];
	}

	dev_dbg(imc_idi->dev, "Attempt to queue transaction %p, %s, size %x, ch %d\n",
				trans, (tx_not_rx ? "TX" : "RX"),
				trans->idi_xfer.size, channel);

	/*
	 * A QUEUE_BUSY will only occur when the queues are being flushed.
	 */
	if (unlikely((*queue_busy) & QUEUE_BUSY(channel)))
		return -EBUSY;

	/*
	 * We have to have a far side address (IDI Slave) to send to.
	 */
	if ((idi->channels[channel] != OUTSTANDING_READ &&
		idi->channels[channel] != ERRORS) &&
		((channel_ctx->dst_addr == 0)
			&& (trans->idi_xfer.dst_addr == 0)))
		return -EINVAL;

	/*
	 * Make sure that the incoming transaction has the correct parameters.
	 */
	if ((trans->idi_xfer.size == 0)
		&& (trans->idi_xfer.desc == NULL)) {
		dev_err(imc_idi->dev,
			"Invalid size for transfer on channel %d\n", channel);
		return -EINVAL;
	}

	if ((trans->idi_xfer.base == 0)
		&& (trans->idi_xfer.desc == NULL)) {
		if (tx_not_rx) {
			dev_err(imc_idi->dev,
				"CH %d: Invalid xfer descriptor\n",
							channel);
			return -EINVAL;
		} else {
			if (channel_ctx->base == 0) {
				dev_err(imc_idi->dev,
					"CH %d: Invalid xfer descriptor\n",
								channel);

				return -EINVAL;
			}

		}
	}

	/*
	 * Set the status to QUEUED and add it to the list.
	 */
	dev_dbg(imc_idi->dev, "Transaction %p queued\n", trans);

	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	trans->status = IDI_STATUS_QUEUED;
	list_add_tail(&trans->link, queue);
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

	idi_debug_add_event(queue_xfer, 0, trans);

	imc_transfer(imc_idi, tx_not_rx, channel);

	return 0;

}				/* imc_mid_async() */

/**
 * imc_mid_xfer_complete - Notifies the IDI controller from the client a xfer
 * completed
 * @idi: reference to the IDI controller device
 * @tx_not_rx: Indicates a TX transaction if no null, ortherwise a RX
 * @channel: Channel number
 *
 * This callback has to be called in atomic context
 *
 * Returns 0 if successful, -EINVAL
 */
static int imc_mid_xfer_complete(struct idi_controller_device *idi,
					int tx_not_rx, int channel)
{
	int do_fwd = 0;
	struct imc_controller *imc_idi = idi_controller_get_drvdata(idi);

	do_fwd = imc_xfer_complete(imc_idi, channel, tx_not_rx, true);

	if (do_fwd)
		tasklet_schedule(&imc_idi->fwd_tasklet);

	return 0;
}

/**
 * imc_destruct_trans - helper function for cleanly destructing a message
 * @trans: reference to the message to destruct
 * @imc_idi: IMC IDI controller reference
 */
static void imc_destruct_trans(struct idi_transaction *trans,
					struct imc_controller *imc_idi)
{

	IDI_IMC_ENTER;

	if (trans->t_type == IDI_TRANS_WRITE)
		(void)deassert_acwake(imc_idi);

	trans->status = IDI_STATUS_FLUSH;
	if (trans->complete)
		trans->complete(trans);
	else
		idi_free_transaction(trans);
}

/**
 * imc_flush_queue - flushing all messages of a client from a queue
 * @queue: reference to the message queue to flush
 * @cl: IDI client reference
 * @imc_idi: IMC IDI controller reference
 */
static void imc_flush_queue(struct list_head *queue,
				struct imc_controller *imc_idi)
{
	struct idi_transaction *trans;
	struct idi_controller_device *idi;
	struct idi_client_device *client;
	unsigned long flags;
	void __iomem *ctrl;

	IDI_IMC_ENTER;
	ctrl = imc_idi->ctrl_io;
	idi = to_idi_controller_device(imc_idi->dev);
	client = to_idi_client_device(idi->client);
	if (client == NULL) {
		dev_err(imc_idi->dev, "imc_flush_queue: No client device\n");
		return;
	}

	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	while (!list_empty(queue)) {
		trans = list_first_entry(queue, struct idi_transaction, link);
		if (trans->t_type == IDI_TRANS_READ) {
			imc_idi_mask_rx_irq_ch(imc_idi, trans->channel);
			list_del(&trans->link);
			spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
			client->flush_buffer(client, trans);
			spin_lock_irqsave(&imc_idi->hw_lock, flags);
			iowrite32(0,
				IMC_IDI_RXCH_IRQ_CON(ctrl, trans->channel));
			iowrite32((IMC_IDI_RXCH_RD_CON_RST |
				IMC_IDI_RXCH_RD_CON_RST_INT),
				IMC_IDI_RXCH_RD_CON(ctrl, trans->channel));
			iowrite32((1 << trans->channel),
					IMC_IDI_RXIRQ_CON(ctrl));
			spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
		} else {
			imc_idi_mask_tx_irq_ch(imc_idi, trans->channel);
			list_del(&trans->link);
			spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
			client->flush_buffer(client, trans);
			spin_lock_irqsave(&imc_idi->hw_lock, flags);
			iowrite32(0,
				IMC_IDI_TXCH_IRQ_CON(ctrl, trans->channel));
			iowrite32((IMC_IDI_TXCH_WR_CON_RST |
				IMC_IDI_TXCH_WR_CON_RST_INT),
				IMC_IDI_TXCH_WR_CON(ctrl, trans->channel));
			iowrite32((1 << trans->channel),
						IMC_IDI_TXIRQ_CON(ctrl));
			spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
		}
		imc_destruct_trans(trans, imc_idi);
		spin_lock_irqsave(&imc_idi->sw_lock, flags);
	}
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
}

 /**
 * imc_streaming_channel_flush - flushing the internal and external channel buffers
 *				of the streaming channels on both the AGOLD
 *				and XGOLD side
 * @channell: logical channel
 * @imc_idi: IMC IDI controller reference
 */
static void imc_streaming_channel_flush(int channel,
				struct imc_controller *imc_idi)
{
	struct idi_controller_device *idi;
	struct idi_client_device *client;
	unsigned long flags;
	void __iomem *ctrl;

	IDI_IMC_ENTER;
	ctrl = imc_idi->ctrl_io;
	idi = to_idi_controller_device(imc_idi->dev);
	client = to_idi_client_device(idi->client);
	if (client == NULL) {
		dev_err(imc_idi->dev, "imc_streaming_channel_flush: No client device\n");
		return;
	}

	client->streaming_channel_flush(client, channel);

	spin_lock_irqsave(&imc_idi->hw_lock, flags);
	iowrite32((IMC_IDI_RXCH_RD_CON_RST |
		IMC_IDI_RXCH_RD_CON_RST_INT),
		IMC_IDI_RXCH_RD_CON(ctrl, channel));
	iowrite32((IMC_IDI_TXCH_WR_CON_RST |
		IMC_IDI_TXCH_WR_CON_RST_INT),
		IMC_IDI_TXCH_WR_CON(ctrl, channel));
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

}

/**
 * imc_device_init - initialize the hardware device
 * @imc_idi: IDI controller device.
 *
 * This stores the hardware setup and applies it in conformance with the
 * client settings.
 *
 * Return success or an error code if the cleint configuration is invalid.
 */
static int imc_device_init(struct imc_controller *imc_idi)
__acquires(&imc_idi->sw_lock) __releases(&imc_idi->sw_lock)
{
#ifdef READ_PLATFORM_DATA
	struct imc_mid_platform_data *pd;
#endif
	unsigned long flags;
	int err = 0;

	IDI_IMC_ENTER;

#ifdef READ_PLATFORM_DATA
	/* Read the platform data to initialise the device */
	pd = (struct imc_mid_platform_data *)(cl->device.platform_data);
	if (pd == NULL) {
		dev_dbg(&port->device, "platform data not found\n");
		return -EINVAL;
	}
#endif

	spin_lock_irqsave(&imc_idi->sw_lock, flags);

	/*
	 * Set the clock divider for HSI_CLC_CNT
	 */
	imc_idi->clc_cnt_cfg = IMC_IDI_CLC_CNT_RMC(2) | IMC_IDI_CLC_CNT_ORMC(1);

	imc_idi->txconf_cfg =
	    IMC_IDI_TXCONF_DESC(IMC_IDI_TXCONF_DESC_FOUR) |
	    IMC_IDI_TXCONF_TRANS |
	    IMC_IDI_TXCONF_BREAK(IMC_IDI_TXCONF_BREAK_37);

	/*
	 * Enable bits will be set in imc_ctrl_set_cfg()
	 */
/*
	FIXME: Channels configuration should come as platform data
*/
	imc_idi->txcon_cfg =
	    IMC_IDI_TXCON_MASTER |
	    IMC_IDI_TXCON_PRIORITY(IMC_IDI_STREAM_CHANNELS);

	/*
	 * txmask_con will be set when data is put on a given channel and
	 * cleared when the transfer is done.
	 *
	 * idi_txmask_con = all_non_audio_bits;
	 *   imc_idi->rxcmask_con = all_non_audio_bits;
	 */

	/*
	 * Setting tx3mask_conf for the LPE device.  Need to be able to figure
	 * out what these bits are.  Hmm, should probably have the "stream
	 * driver" set the channel through the client.
	 */
	imc_idi->tx3mask_cfg = IMC_IDI_STREAM_CHANNELS;
	imc_idi->rx3mask_cfg = IMC_IDI_STREAM_CHANNELS;

	imc_idi->rxconf_cfg =
	    IMC_IDI_RXCONF_TRANS_FRAME |
	    IMC_IDI_RXCONF_DESC(IMC_IDI_RXCONF_DESC_FOUR) |
	    IMC_IDI_RXCONF_BREAK(IMC_IDI_RXCONF_BREAK_37);

	/*
	 * Enable bits will be set in imc_ctrl_set_cfg()
	 */
	imc_idi->rxcon_cfg =
		IMC_IDI_RXCON_SKIP_LIMIT(4) |
		IMC_IDI_RXCON_FLUSH_TO(1) |
		IMC_IDI_RXCON_DIS_BURST(IMC_IDI_STREAM_CHANNELS);

	imc_idi->err_con_cfg =
	    IMC_IDI_ERR_CON_EN_TOC(IMC_IDI_ERR_CON_EN_TOC_ENABLED) |
	    IMC_IDI_ERR_CON_EN_TRC(IMC_IDI_ERR_CON_EN_TRC_ENABLED) |
	    IMC_IDI_ERR_CON_EN_BUC(IMC_IDI_ERR_CON_EN_BUC_ENABLED) |
	    IMC_IDI_ERR_CON_TIMEOUT(0xDE) |
	    IMC_IDI_ERR_CON_TRAILING(0xC) | IMC_IDI_ERR_CON_BURSTS(0x7);

/* FIXME: number of dma/stream channels should find out at runtime*/
	imc_idi->ext_con_cfg =
	    IMC_IDI_EXT_CON_SIG(IMC_IDI_EXT_CON_SIG_ENABLED) |
	    IMC_IDI_EXT_CON_INT(IMC_IDI_EXT_CON_INT_ENABLED) |
	    IMC_IDI_EXT_CON_STREAM(7) |
	    IMC_IDI_EXT_CON_DMA(7) |
	    IMC_IDI_EXT_CON_PRIORITY(IMC_IDI_DBB_EXT_CON_PRIORITY_REG
				| IMC_IDI_DBB_EXT_CON_PRIORITY_DMA) |
	    IMC_IDI_EXT_CON_TIMEOUT(4095) | IMC_IDI_EXT_CON_DMA4(1);

	/*
	 * FIXME: These values need to be determined by the audio team.
	 */
	imc_idi->ext_con2_cfg =
	    IMC_IDI_EXT_CON2_DIV1(0) |
	    IMC_IDI_EXT_CON2_SAMPLES1(0) |
	    IMC_IDI_EXT_CON2_DIV2(0) |
	    IMC_IDI_EXT_CON2_SAMPLES2(0) |
	    IMC_IDI_EXT_CON2_DIV3(0) | IMC_IDI_EXT_CON2_SAMPLES3(0);

	imc_idi->channel_con_cfg =
	    IMC_IDI_CHANNEL_CON_CHANNEL(~(IMC_IDI_STREAM_CHANNELS)) |
	    IMC_IDI_CHANNEL_CON_CHANNEL2(IMC_IDI_STREAM_CHANNELS);


	/*
	 * And the interrupts we are interested in.
	 */
	imc_idi->imsc_cfg =
	    (IMC_IDI_IMSC_RX | IMC_IDI_IMSC_TX |
	     IMC_IDI_IMSC_IDLE | IMC_IDI_IMSC_TIME | IMC_IDI_IMSC_BREAK |
	     IMC_IDI_IMSC_STALL | IMC_IDI_IMSC_TRAIL | IMC_IDI_IMSC_MASTER |
	     IMC_IDI_IMSC_ADDR | IMC_IDI_IMSC_ZERO);

	imc_idi->imsc_cfg |= IMC_IDI_IMSC_WAKE;

	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

	/* The controller will be configured on resume if necessary */
	if (unlikely(imc_idi->suspend_state == DEVICE_READY))
		err = imc_ctrl_set_cfg(imc_idi);

	return err;

}				/* imc_device_init() */

/**
 * enable_acready - CA_WAKE assertion event handler
 * @imc_idi: IMC IDI controller reference
 *
 * Notify the perpheral devices that we have a CA_WAKE event.  This is done
 * using the bus interface.
 *
 */
static void enable_acready(struct imc_controller *imc_idi)
{
	struct idi_controller_device *idi =
	    to_idi_controller_device(imc_idi->dev);

	IDI_IMC_ENTER;

	if (has_enabled_acready(imc_idi))
		idi_event(idi, IDI_EVENT_START_RX);
}

/**
 * try_disable_acready - CA_WAKE de-assertion event handler
 * @imc: IMC IDI controller reference
 */
static void try_disable_acready(struct imc_controller *imc_idi)
{
	struct idi_controller_device *idi =
	    to_idi_controller_device(imc_idi->dev);

	IDI_IMC_ENTER;

	if (has_disabled_acready(imc_idi))
		idi_event(idi, IDI_EVENT_STOP_RX);
}

/**
 * imc_xfer_complete - Data transfer complete status handler
 * @imc_idi: IMC IDI controller reference
 * @channel: IDI channel
 * @tx_not_rx: which direction
 *
 * Returns the number of managed DMA transfers.
 */
static int imc_xfer_complete(struct imc_controller *imc_idi, int channel,
			     int tx_not_rx, bool from_client)
__acquires(&imc_idi->sw_lock) __releases(&imc_idi->sw_lock)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	struct idi_transaction *trans;
	struct list_head *queue;
	struct imc_channel_ctx *channel_ctx;
	unsigned long flags;

	IDI_IMC_ENTER;
	if (channel >= IDI_MAX_CHANNEL)
		return 0;

	if (tx_not_rx == 1) {
		queue = &imc_idi->tx_queue[channel];
		channel_ctx = &imc_idi->tx_ctx[channel];
	} else {
		queue = &imc_idi->rx_queue[channel];
		channel_ctx = &imc_idi->rx_ctx[channel];
	}

	/* Completion from client for RX is a End Of Packet */
	if (from_client && !tx_not_rx) {
		if (channel_ctx->end_of_packet)
			channel_ctx->end_of_packet(channel_ctx->private_data);
		return 0;
	}
	imc_idi_dump_channel(imc_idi, channel, tx_not_rx);

	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	if (list_empty(queue)) {
		spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
		dev_err(imc_idi->dev,
			"An spurious %s end of transaction on channel %d\n",
				tx_not_rx ? "TX" : "RX", channel);
		return 0;
	}
	trans = list_first_entry(queue, struct idi_transaction, link);
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

	if (unlikely(!trans))
		return 0;

	idi_debug_add_event(complete_xfer, 0, trans);

	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	dev_dbg(imc_idi->dev, "Add trans %p to fwd queue\n", trans);
	list_del(&trans->link);
	list_add_tail(&trans->link, &imc_idi->fwd_queue);
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

	imc_transfer(imc_idi, tx_not_rx, channel);

	return 1;

}				/* imc_xfer_complete() */

/**
 * imc_isr_tasklet - low-latency interrupt management out of interrupt state
 * @imc: IMC IDI controller reference
 */
static void imc_isr_tasklet(unsigned long imc)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	struct imc_controller *imc_idi = (struct imc_controller *)imc;
	void __iomem *ctrl = imc_idi->ctrl_io;
	int ch;
	u32 mis_status;
	u32 xfer_status;
	u32 imsc_cfg = 0;
	u32 mask;
	u32 ch_mask;
	unsigned long flags;
	int do_fwd = 0;

	IDI_IMC_ENTER;

	/*
	 * Get a local copy of the current interrupt status and clear the
	 * "global" version.
	 */
	spin_lock_irqsave(&imc_idi->hw_lock, flags);
	mis_status = imc_idi->mis_status;
	imc_idi->mis_status = 0;
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

	if (mis_status & IMC_IDI_MIS_WAKE) {
		enable_acready(imc_idi);
		/*
		 * We dont' need to reset the wake flag because it is set and
		 * cleared in the idle and resume path routines.
		 */
	}

	/*
	 * Handle the various non data transfer interrutps.
	 */
	if (mis_status & IMC_IDI_RX_ERR_MASK) {
		imc_rx_error(imc_idi, mis_status);
		imsc_cfg |= (mis_status & IMC_IDI_RX_ERR_MASK);
	}

	if (mis_status & IMC_IDI_MIS_ADDR) {
		dev_err(imc_idi->dev, "imc_isr_tasklet: address error\n");
		imsc_cfg |= IMC_IDI_IMSC_ADDR;
	}

	if (mis_status & IMC_IDI_MIS_MASTER) {
		dev_err(imc_idi->dev, "imc_isr_tasklet: MASTER error\n");
		imsc_cfg |= IMC_IDI_IMSC_MASTER;
	}

	if (mis_status & IMC_IDI_MIS_BREAK) {
		imc_break_complete(imc_idi);
		imsc_cfg |= IMC_IDI_IMSC_BREAK;
	}

#ifdef IDLE_POLL
	if (mis_status & IMC_IDI_MIS_IDLE) {
		mod_timer(&imc_idi->idle_poll, jiffies + IDLE_POLL_JIFFIES);
#endif

	/*
	 * Determine if we have an RX or TX completion.  Grab the appropriate
	 * status bits and clear them.  Then process the appropriate
	 * transaction.
	 */
	mask = 1;
	if (mis_status & IMC_IDI_MIS_RX) {
		ch_mask = ioread32(IMC_IDI_RXMASK_CON(ctrl));
		xfer_status = ioread32(IMC_IDI_RXIRQ_STAT(ctrl));
		xfer_status &= ch_mask;
		iowrite32(xfer_status, IMC_IDI_RXIRQ_CON(ctrl));

		for (ch = 0; ch < IDI_MAX_CHANNEL; ch++) {
			if (xfer_status & mask) {
				if (imc_xfer_complete(imc_idi, ch, 0, false))
					do_fwd++;
			}
			mask <<= 1;
		}
		imsc_cfg |= IMC_IDI_IMSC_RX;
	}
	if (mis_status & IMC_IDI_MIS_TX) {
		ch_mask = ioread32(IMC_IDI_TXMASK_CON(ctrl));
		xfer_status = ioread32(IMC_IDI_TXIRQ_STAT(ctrl));
		xfer_status &= ch_mask;
		iowrite32(xfer_status, IMC_IDI_TXIRQ_CON(ctrl));

		mask = 1;
		for (ch = 0; ch < IDI_MAX_CHANNEL; ch++) {
			if (xfer_status & mask) {
				if (imc_xfer_complete(imc_idi, ch, 1, false))
					do_fwd++;
			}

			mask <<= 1;
		}
		imsc_cfg |= IMC_IDI_IMSC_TX;
	}

	try_disable_acready(imc_idi);

	if (do_fwd)
		tasklet_schedule(&imc_idi->fwd_tasklet);

	/* Re-enable relevant interrupts */
	if (imsc_cfg) {
		spin_lock_irqsave(&imc_idi->hw_lock, flags);
		imc_idi->imsc_cfg |= imsc_cfg;
		imc_set_interrupt(ctrl, imc_idi);
		spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
	}
	IDI_IMC_EXIT;
}				/* imc_isr_tasklet() */

/**
 * imc_fwd_tasklet - Send IDI transaction back to perpipheral
 * @imc: IMC IDI controller reference
 */
static void imc_fwd_tasklet(unsigned long imc)
__acquires(&imc_idi->sw_lock) __releases(&imc_idi->sw_lock)
__acquires(&imc_idi->sw_lock) __releases(&imc_idi->sw_lock)
{
	struct imc_controller *imc_idi = (struct imc_controller *)imc;
	struct idi_transaction *trans;
	struct imc_channel_ctx *channel_ctx;
	struct idi_xfer trans_idi_xfer;
	unsigned long flags;
	unsigned int trans_t_type, trans_channel;
	unsigned ch_size_words, rd_pointer, payload;
	void __iomem *ctrl;

	IDI_IMC_ENTER;

	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	while (!list_empty(&imc_idi->fwd_queue)) {
		trans = list_first_entry(&imc_idi->fwd_queue,
					 struct idi_transaction, link);
		list_del(&trans->link);
		spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

		dev_dbg(imc_idi->dev, "trans: %p, complete: %p\n",
						trans, trans->complete);
		if (trans->t_type == IDI_TRANS_WRITE)
			(void)deassert_acwake(imc_idi);

		idi_debug_add_event(xfer_fwd, 0, trans);

		/* Capture the required values before the transaction */
		/* is freed by the complete() function	*/
		trans_t_type = trans->t_type;
		trans_channel = trans->channel;
		trans_idi_xfer = trans->idi_xfer;

		if ((!trans->status != IDI_STATUS_ERROR)) {
			ctrl = imc_idi->ctrl_io;
			channel_ctx = &imc_idi->rx_ctx[trans_channel];

			/* BOUNCE buffer used*/
			if ((trans->t_type == IDI_TRANS_READ) &&
				(trans->idi_xfer.base == 0) &&
					(trans->idi_xfer.desc == NULL)) {
				struct idi_xfer *xfer = &trans->idi_xfer;
				unsigned payload = xfer->size;
				/* FIXME: Stop to mix size_t/size in words */
				unsigned rd_pointer, remaining;
				struct scatterlist *sg = &xfer->sg[0];

				/* Compute base address of buffer */
				rd_pointer =
					ioread32(IMC_IDI_RXCH_RD_CON(ctrl,
							trans_channel));
				remaining = channel_ctx->size - rd_pointer;
				rd_pointer >>= IMC_IDI_RXCH_RD_CON_PTR_SHIFT;
				xfer->cpu_base = channel_ctx->cpu_base;
				xfer->cpu_base += rd_pointer;

				/* Fill in scatterlist as the buffer might
				 * not be contigueous
				 */
				sg_init_table(sg, 2);
				sg_dma_address(sg) = channel_ctx->base +
							(rd_pointer * 4);
				if (remaining  > xfer->size) {
					sg_dma_len(sg) = xfer->size;
					sg_mark_end(sg);
					dev_dbg(imc_idi->dev,
						"SG0 @virt:%p, phys:%x, %x bytes\n",
						xfer->cpu_base,
						sg_dma_address(sg),
						sg_dma_len(sg));
				} else {
					sg_dma_len(sg) = remaining;
					dev_dbg(imc_idi->dev,
						"SG0 @virt%p, phys:%x, %x bytes\n",
						xfer->cpu_base,
						sg_dma_address(sg),
						sg_dma_len(sg));

					sg = sg_next(sg);
					sg_dma_address(sg) = channel_ctx->base;
					sg_dma_len(sg) = xfer->size - remaining;
					dev_dbg(imc_idi->dev, "SG1 @virt:%p, phys:%x, %x bytes\n",
						channel_ctx->cpu_base,
						sg_dma_address(sg),
						sg_dma_len(sg));
				}

				dev_dbg(imc_idi->dev, "xfer cpu base %p, payload %d\n",
					xfer->cpu_base, payload);
				imc_idi_dump_channel(imc_idi, trans_channel, 0);
			}
			trans->status = IDI_STATUS_COMPLETE;
		}

		if (unlikely(trans->break_frame)) {
			trans->break_frame = 0;
			if (trans->complete)
				trans->complete(trans);
			else
				idi_free_transaction(trans);
		} else {
			trans->complete(trans);
		}

		/* Update the read pointer */
		if (trans_t_type == IDI_TRANS_READ) {
			channel_ctx = &imc_idi->rx_ctx[trans_channel];

			/* Make sure we are dealing with bounce buffer only */
			if (channel_ctx->base &&
					!(trans_idi_xfer.base
						|| trans_idi_xfer.desc)) {
				ctrl = imc_idi->ctrl_io;
				ch_size_words = channel_ctx->size;
				ch_size_words /= sizeof(u32);
				rd_pointer = ioread32(
						IMC_IDI_RXCH_RD_CON(ctrl,
							trans_channel));
				rd_pointer >>= IMC_IDI_RXCH_RD_CON_PTR_SHIFT;
				payload = DIV_ROUND_UP(trans_idi_xfer.size,
								 sizeof(u32));

				rd_pointer += payload;
				if (rd_pointer >= ch_size_words)
					rd_pointer -= ch_size_words;

				dev_dbg(imc_idi->dev,
					"xfer payload %d, new rd_pointer %x\n",
					payload, rd_pointer);
				rd_pointer <<= IMC_IDI_RXCH_RD_CON_PTR_SHIFT;

				iowrite32(rd_pointer,
						IMC_IDI_RXCH_RD_CON(ctrl,
								trans_channel));
			}
		}

		spin_lock_irqsave(&imc_idi->sw_lock, flags);
	}
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

}				/* imc_fwd_tasklet() */

/**
 * imc_isr - IDI controller interrupt service routine
 * @irq: IRQ number
 * @imc: IMC IDI controller reference
 *
 * Clears and stores the interrupt sources and schedules a tasklet for handling
 * them efficiently.
 *
 * Returns IRQ_HANDLED as the interrupt sources are handled in all cases.
 */
static irqreturn_t imc_isr(int irq, void *imc)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	struct imc_controller *imc_idi = (struct imc_controller *)imc;
	void __iomem *ctrl = imc_idi->ctrl_io;
	u32 mis_status, mis1_status;

	IDI_IMC_ENTER;

	spin_lock(&imc_idi->hw_lock);

	/*
	 * The only interrupt source when suspended is an external wakeup, so
	 * notify it to the interrupt tasklet.  An external wakeup can come from
	 * the ACWAKE line, or by a forwarded interrupt.
	 */
	if (unlikely(imc_idi->suspend_state != DEVICE_READY)) {
		imc_idi->suspend_state = DEVICE_AND_IRQ_SUSPENDED;
		imc_controller_disable_irqs_nosync(imc_idi);
		imc_idi->mis_status |= IMC_IDI_MIS_WAKE;

		mis_status = ioread32(IMC_IDI_MIS(ctrl));
		if (mis_status & IMC_IDI_MIS_WAKE)
			dev_info(imc_idi->dev, "imc_isr: WAKE set\n");

		goto exit_irq;
	}

	/*
	 * Get the bits that caused the interrupt.
	 */
	mis_status = ioread32(IMC_IDI_MIS(ctrl));
	mis1_status = ioread32(IMC_IDI_MIS1(ctrl));

	dev_dbg(imc_idi->dev,
			"%s: MIS %x, idi mis_status %x, MIS1 %x\n",
					__func__, mis_status,
					imc_idi->mis_status, mis1_status);

	/*
	 * Don't disable the slave bits.
	 * The IRQ subystem is taking care of slave interrupts
	 */
	mis_status &= ~IMSC_SLAVE_MASK;

	/*
	 * Turn off any interrupt that we are going to handle.
	 * NOTE: We do not turn off the slave bits.  After we have notified the
	 * peripheral we are done with the interrupt, and can clear it.
	 */
	if (mis_status) {
		imc_idi->imsc_cfg &= ~mis_status;
		imc_set_interrupt(ctrl, imc_idi);
	}

	/*
	 * Clear the interrupt bits and note which ones are set so the tasklet
	 * knows what to do.
	 */
	if (mis_status) {

		dev_dbg(imc_idi->dev, "%s: MIS %x\n", __func__, mis_status);
		iowrite32((mis_status), IMC_IDI_ICR(ctrl));
		imc_idi->mis_status |= mis_status;

		/*
		 * dummy read for synchronization (pg. 75 of
		 * ipdb_hsi_arch_master.pdf)
		 */
		/* mis_status = ioread32(IMC_IDI_MIS(ctrl)); */
	}


exit_irq:
	spin_unlock(&imc_idi->hw_lock);

	/*
	 * Only schedule the tasklet if we had non-slave interrupt.
	 */
	if (mis_status)
		tasklet_hi_schedule(&imc_idi->isr_tasklet);

	IDI_IMC_EXIT;
	return IRQ_HANDLED;

}				/* imc_irq() */

/*  FIXME
 These two functions should be done by the runtime power managment.
 static int imc_request_access(struct idi_peripheral_device *peripheral);
 static int imc_release_access(struct idi_peripheral_device *peripheral);
*/

/**
 * imc_set_channel_config - Set the channels configuration
 * @idi: controller reference (bus)
 * @config: config to set.
 * @channel: The channel to set the config to.
 *
 */
static int imc_set_channel_config(struct idi_controller_device *idi,
				  struct idi_channel_config *config,
				  int channel)
{
	struct imc_controller *imc_idi;
	struct imc_channel_ctx *channel_ctx;
	struct idi_client_device *client;
	unsigned long flags;
	struct idi_transaction trans;
	u32 reg;

	IDI_IMC_ENTER;

	if ((idi == NULL) || (config == NULL))
		return -EINVAL;

	client = to_idi_client_device(idi->client);
	if (client == NULL)
		return -EINVAL;

	if ((channel < 0) || (channel >= IDI_MAX_CHANNEL))
		return -EINVAL;

	imc_idi = idi_controller_get_drvdata(idi);
	if (imc_idi == NULL)
		return -EINVAL;

	channel_ctx = (config->tx_or_rx) ?
	    &imc_idi->tx_ctx[channel] : &imc_idi->rx_ctx[channel];

	spin_lock_irqsave(&imc_idi->sw_lock, flags);

	channel_ctx->priority = config->priority;
	channel_ctx->base = config->base;
	channel_ctx->cpu_base = config->cpu_base;
	channel_ctx->hw_fifo_size = config->hw_fifo_size;
	channel_ctx->end_of_packet = config->end_of_packet;
	channel_ctx->private_data = config->private_data;
	channel_ctx->start_tx = config->start_tx;
	channel_ctx->channel_opts = config->channel_opts;
	/*
	 * Prepare the IDI client channel for BOUNCE BUFFER
	 */

	if ((idi->channels[channel] == FILE)
			|| (idi->channels[channel] == STREAM)) {
		trans.idi_xfer.dst_addr = 0;
		trans.idi_xfer.size = 0;
		trans.t_type = config->tx_or_rx ?
					IDI_TRANS_WRITE : IDI_TRANS_READ;
		trans.channel = channel;

		channel_ctx->dst_size = config->hw_fifo_size;
		channel_ctx->dst_addr = config->dst_addr;

		client->set_addr_size(client, &trans,
			config->dst_addr , channel_ctx->dst_size);

	}

	/* Get the End Of packet Interrupt for Rx */
	if (idi->channels[channel] == FILE) {

		/* We want an eop for Tx completion */
		if (config->tx_or_rx) {
			if (config->channel_opts & IDI_TX_EARLY_IRQ) {
				iowrite32(BIT(channel),
					IMC_IDI_TXIRQ_CON(imc_idi->ctrl_io));
				imc_idi_unmask_tx_irq_ch(imc_idi, channel);
			} else {
				/* TX Completion callback will be triggered
				 *  only when last data have been sent out
				 *  of the ABB FIFO
				*/
				imc_idi_mask_tx_irq_ch(imc_idi, channel);
				client->set_interrupt(client, &trans);
			}
		} else {
			imc_idi_unmask_rx_irq_ch(imc_idi, channel);
			if (config->end_of_packet)
				/* End of Packet callback for RX*/
				client->set_interrupt(client, &trans);
		}
	}


	channel_ctx->size = config->size;

	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

	/* Do I need to do it ?*/
	/*
	 * Set the channel registers as appropriate only in case of :
	 *  1)  bounce buffer operation for RX channels
	 *  2)  Stream channels
	 */

	if (((channel_ctx->base) && (!config->tx_or_rx)) ||
					(idi->channels[channel] == STREAM)) {
		if (config->tx_or_rx) {
			iowrite32((channel_ctx->base),
				  IMC_IDI_TXCH_BASE(imc_idi->ctrl_io,
								channel));
			iowrite32(channel_ctx->size,
				  IMC_IDI_TXCH_SIZE(imc_idi->ctrl_io,
								channel));
			iowrite32(IMC_IDI_TXCH_WR_CON_RST,
				  IMC_IDI_TXCH_WR_CON(imc_idi->ctrl_io,
								channel));
			ioread32(IMC_IDI_TXCH_WR_CON(imc_idi->ctrl_io,
								channel));

			iowrite32(channel_ctx->size,
				  IMC_IDI_TXCH_IRQ_CON(imc_idi->ctrl_io,
							       channel));
		} else {
			iowrite32((channel_ctx->base),
				  IMC_IDI_RXCH_BASE(imc_idi->ctrl_io,
								channel));
			iowrite32(channel_ctx->size,
				  IMC_IDI_RXCH_SIZE(imc_idi->ctrl_io,
								channel));
			iowrite32(IMC_IDI_RXCH_RD_CON_RST,
				  IMC_IDI_RXCH_RD_CON(imc_idi->ctrl_io,
								channel));

			ioread32(IMC_IDI_RXCH_RD_CON(imc_idi->ctrl_io,
								channel));
			iowrite32(channel_ctx->size,
				  IMC_IDI_RXCH_IRQ_CON(imc_idi->ctrl_io,
							       channel));
		}
	}

	/*
	 * Only the stream channels are allowed to set the priority bit.
	 */
	if (idi->channels[channel] == STREAM) {
		spin_lock_irqsave(&imc_idi->hw_lock, flags);
		reg = ioread32(IMC_IDI_TXCON(imc_idi->ctrl_io));
		/* TODO: Check if the operation is really needed */
		if (channel_ctx->priority == IDI_HIGH_PRIORITY)
			reg |= IMC_IDI_TXCON_PRIORITY(BIT(channel));
		else	/* set LOW_PRIORITY */
			reg &= ~IMC_IDI_TXCON_PRIORITY(BIT(channel));

		iowrite32(reg, IMC_IDI_TXCON(imc_idi->ctrl_io));
		spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
	}

	return 0;

}				/* imc_set_channel_config() */

static int imc_buffer_info(struct idi_controller_device *idi, int tx_or_rx,
			   int channel)
__acquires(&imc_idi->sw_lock) __releases(&imc_idi->sw_lock)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	struct imc_controller *imc_idi;
	struct imc_channel_ctx *channel_ctx;
	struct list_head *queue;
	struct idi_transaction *trans;
	unsigned long flags;
	u32 rd_ptr, wr_ptr, size, buf_size;
	int status;

	IDI_IMC_ENTER;

	if (idi == NULL)
		return -EINVAL;

	if (channel < 0 || channel > IDI_MAX_CHANNEL)
		return -EINVAL;

	imc_idi = idi_controller_get_drvdata(idi);
	if (imc_idi == NULL)
		return -EINVAL;

	if (tx_or_rx) {
		channel_ctx = &imc_idi->tx_ctx[channel];
		queue = &imc_idi->tx_queue[channel];
	} else {
		channel_ctx = &imc_idi->rx_ctx[channel];
		queue = &imc_idi->rx_queue[channel];
	}

	spin_lock_irqsave(&imc_idi->sw_lock, flags);

	if (list_empty(queue)) {
		spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
		dev_info(imc_idi->dev, " List is empty ..\n");
		return 0;
	}

	trans = list_first_entry(queue, struct idi_transaction, link);
	status = trans->status;
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

	if (status == IDI_STATUS_PROCEEDING) {
		if (tx_or_rx) {
			spin_lock_irqsave(&imc_idi->hw_lock, flags);
			size =
			    ioread32(IMC_IDI_TXCH_SIZE
				     (imc_idi->ctrl_io, channel));
			rd_ptr =
			    ioread32(IMC_IDI_TXCH_RD_STAT
				     (imc_idi->ctrl_io, channel));
			spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
			buf_size = (size - rd_ptr);
			if (rd_ptr & IMC_IDI_TXCH_RD_STAT_DAT)
				return buf_size;
		} else {
			spin_lock_irqsave(&imc_idi->hw_lock, flags);
			size =
			    ioread32(IMC_IDI_RXCH_SIZE
				     (imc_idi->ctrl_io, channel));
			wr_ptr =
			    ioread32(IMC_IDI_RXCH_WR_STAT
				     (imc_idi->ctrl_io, channel));
			spin_unlock_irqrestore(&imc_idi->hw_lock, flags);
			buf_size = (size - wr_ptr);
			if (wr_ptr & IMC_IDI_RXCH_WR_STAT_DAT)
				return buf_size;
		}
	}

	return 0;

}				/* imc_buffer_info() */

/**
 * imc_buffer_flush - flush an in progress receive transaction
 * @idi: IDI controller reference (bus)
 * @tx_or_rx: which direction to flush
 * @channel: which channel to flush
 *
 * Flushing a TX transaction doesn't make sense (it would be a stop), so this
 * will only handle RX transactions.
 *
 */
static int imc_buffer_flush(struct idi_controller_device *idi, int channel)
__acquires(&imc_idi->sw_lock) __releases(&imc_idi->sw_lock)
__acquires(&imc_idi->hw_lock) __releases(&imc_idi->hw_lock)
{
	struct imc_controller *imc_idi;
	void __iomem *ctrl;
	unsigned long flags;
	struct list_head *queue;
	unsigned short *queue_busy;
	struct imc_channel_ctx *channel_ctx;
	struct idi_transaction *trans;
	u32 wr_ptr, rd_ptr, size;

	IDI_IMC_ENTER;

	if (idi == NULL)
		return -EINVAL;

	if (channel < 0 || channel >= IDI_MAX_CHANNEL)
		return -EINVAL;

	imc_idi = idi_controller_get_drvdata(idi);
	if (imc_idi == NULL)
		return -EINVAL;

	ctrl = imc_idi->ctrl_io;

	channel_ctx = &imc_idi->rx_ctx[channel];
	queue = &imc_idi->rx_queue[channel];
	queue_busy = &imc_idi->rx_queue_busy;

	/*
	 * Prevent any new transactions from beeing added to the software queues
	 */
	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	*queue_busy |= (1 << channel);

	if (list_empty(queue)) {
		*queue_busy &= ~BIT(channel);
		spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
		dev_info(imc_idi->dev, " List is empty ..\n");
		return 0;
	}

	/*
	 * The entry needs to be IDI_STATUS_PROCEEDING for us to work on it.
	 */
	trans = list_first_entry(queue, struct idi_transaction, link);
	if (trans->status != IDI_STATUS_PROCEEDING) {
		*queue_busy &= ~BIT(channel);
		spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
		return -EINVAL;
	}

	if (trans->idi_xfer.desc) {
		iowrite32(IMC_IDI_RXCH_NEXT_LAST(1),
			IMC_IDI_RXCH_NEXT(ctrl, channel));
	}

	wr_ptr = ioread32(IMC_IDI_RXCH_WR_STAT(ctrl, channel));
	rd_ptr = ioread32(IMC_IDI_RXCH_RD_CON(ctrl, channel));
	size = wr_ptr - rd_ptr;

	list_del(&trans->link);

	trans->idi_xfer.size = size;
	trans->status = IDI_STATUS_FLUSH;
	trans->complete(trans);

	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

	spin_lock_irqsave(&imc_idi->hw_lock, flags);
	imc_set_interrupt(ctrl, NULL);
	iowrite32(IMC_IDI_RXCH_RD_CON_RST, IMC_IDI_RXCH_RD_CON(ctrl, channel));
	iowrite32((IMC_IDI_RXCON2_CLR_LOG | IMC_IDI_RXCON2_CLR_CH),
		  IMC_IDI_RXCON2(ctrl));
	iowrite32((1 << channel), IMC_IDI_RXIRQ_CON(ctrl));
	spin_unlock_irqrestore(&imc_idi->hw_lock, flags);

	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	*queue_busy &= ~BIT(channel);
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

	imc_transfer(imc_idi, 0, channel);

	return 0;

}				/* imc_buffer_flush() */

/**
 * imc_queue_status - Return the number of elements in the specified queue
 * @idi: controller reference (bus)
 * @tx_or_rx: which direction
 * @channel: specific channel
 *
 * This should probably not be use for anything other than diagnostic
 * purposes.
 */
static int imc_queue_status(struct idi_controller_device *idi, int tx_or_rx,
			    int channel)
{
	struct imc_controller *imc_idi;
	unsigned long flags;
	struct list_head *queue, *pos;
	int count;

	IDI_IMC_ENTER;

	if (idi == NULL)
		return -EINVAL;

	if (channel < 0 || channel >= IDI_MAX_CHANNEL)
		return -EINVAL;

	imc_idi = idi_controller_get_drvdata(idi);
	if (imc_idi == NULL)
		return -EINVAL;

	queue = (tx_or_rx) ?
	    &imc_idi->tx_queue[channel] : &imc_idi->rx_queue[channel];

	count = 0;
	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	list_for_each(pos, queue) {
		count++;
	}
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

	return count;
}				/* imc_queue_status() */

/*
 * Callback registered by IDI with FM to get notified for frequency changes.
 */
#if defined IDI_FM_SUPPORT && defined CONFIG_PLATFORM_DEVICE_PM
enum iui_fm_mitigation_status idi_fm_cb(
					const enum iui_fm_macro_id macro_id,
					const struct iui_fm_mitigation *fm_req,
					const uint32_t sequence)
{
	int err = 0;
	uint32_t mitigation_frequency;
	struct imc_controller *imc_idi;
	struct platform_device *pdev;
	enum iui_fm_mitigation_status ret = IUI_FM_MITIGATION_ERROR;

	if ((fm_req == NULL) ||
		(macro_id != IUI_FM_MACRO_ID_IDI) ||
		(fm_req->type != IUI_FM_MITIGATION_TYPE_KHZ))
		return IUI_FM_MITIGATION_ERROR_INVALID_PARAM;

	mitigation_frequency = fm_req->info.freq_khz;
	imc_idi = (struct imc_controller *)
		idi_controller_get_drvdata(idi_controller);
	pdev = to_platform_device(idi_controller->device.parent);

	switch (imc_idi->idi_state) {
	case ENABLE_NO_WLAN:
		if (mitigation_frequency == IDI_FREQUENCY_ENABLE_NO_WLAN_ALT) {
			err = platform_device_pm_set_state(pdev,
			imc_idi->idi_pm_state[ENABLE_NO_WLAN_ALT]);
			if (err) {
				pr_err("Failed to mitigate the  state\n");
				}
			else{
				imc_idi->idi_state = ENABLE_NO_WLAN_ALT;
				ret = IUI_FM_MITIGATION_COMPLETE_OK;
				}
			}
		else{
			pr_err("Invalid frequency requested by the FM");
			}
		break;

	case ENABLE_NO_WLAN_ALT:
		if (mitigation_frequency == IDI_FREQUENCY_ENABLE_NO_WLAN) {
			err = platform_device_pm_set_state(pdev,
			imc_idi->idi_pm_state[ENABLE_NO_WLAN]);
			if (err) {
				pr_err("Failed to mitigate the  state\n");
				}
			else {
				imc_idi->idi_state = ENABLE_NO_WLAN;
				ret = IUI_FM_MITIGATION_COMPLETE_OK;
				}
			}
		else{
			pr_err("Invalid frequency requested by the FM");
			}
		break;

	case ENABLE_WLAN:
		if (mitigation_frequency == IDI_FREQUENCY_ENABLE_WLAN_ALT) {
			err = platform_device_pm_set_state(pdev,
			imc_idi->idi_pm_state[ENABLE_WLAN_ALT]);
			if (err) {
				pr_err("Failed to mitigate the  state\n");
				}
			else{
				imc_idi->idi_state = ENABLE_WLAN_ALT;
				ret = IUI_FM_MITIGATION_COMPLETE_OK;
				}
			}
		else{
			pr_err("Invalid frequency requested by the FM");
			}
		break;

	case ENABLE_WLAN_ALT:
		if (mitigation_frequency == IDI_FREQUENCY_ENABLE_WLAN) {
			err = platform_device_pm_set_state(pdev,
			imc_idi->idi_pm_state[ENABLE_WLAN]);
			if (err) {
				pr_err("Failed to mitigate the  state\n");
				}
			else{
				imc_idi->idi_state = ENABLE_WLAN;
				ret = IUI_FM_MITIGATION_COMPLETE_OK;
				}
			}
		else{
			pr_err("Invalid frequency requested by the FM");
			}
		break;

	case ENABLE_AUDIO_ONLY:
		if (mitigation_frequency ==
			IDI_FREQUENCY_ENABLE_AUDIO_ONLY_ALT) {
			err = platform_device_pm_set_state(pdev,
			imc_idi->idi_pm_state[ENABLE_AUDIO_ONLY_ALT]);
			if (err) {
				pr_err("Failed to mitigate the  state\n");
				}
			else{
				imc_idi->idi_state = ENABLE_AUDIO_ONLY_ALT;
				ret = IUI_FM_MITIGATION_COMPLETE_OK;
				}
			}
		else{
			pr_err("Invalid frequency requested by the FM");
			}
		break;

	case ENABLE_AUDIO_ONLY_ALT:
		if (mitigation_frequency == IDI_FREQUENCY_ENABLE_AUDIO_ONLY) {
			err = platform_device_pm_set_state(pdev,
			imc_idi->idi_pm_state[ENABLE_AUDIO_ONLY]);
			if (err) {
				pr_err("Failed to mitigate the  state\n");
				}
			else {
				imc_idi->idi_state = ENABLE_AUDIO_ONLY;
				ret = IUI_FM_MITIGATION_COMPLETE_OK;
				}
			}
		else{
			pr_err("Invalid frequency requested by the FM");
			}
		break;

	case DISABLE:
	default:
		pr_err("Alternate Frequency cannot be requested at this state of IDI");
	}

	return ret;
}
#endif

/**
 * imc_set_power_state - Maintain the state of IDI controller and IDI devices.
 * @idi: IDI controller reference
 * @p_type: peripheral device type
 * @dev_state: peripheral device state
 * @idi_is_required : whether idi is required or not
 */
static int imc_set_power_state(struct idi_controller_device *idi,
				struct idi_peripheral_device *peripheral,
				void *handler,
				char *state,
				bool idi_is_required)
{
	struct imc_controller *imc_idi;
	struct platform_device *pdev;
	unsigned long flags;
	u32 ctrl_state, ctrl_state_new;
	int err = 0;
	bool idi_change_frequency = false;

#if defined IDI_FM_SUPPORT && defined CONFIG_PLATFORM_DEVICE_PM
	struct iui_fm_freq_notification fm_info;
	int ret = 0;
#endif

	IDI_IMC_ENTER;
	if ((idi == NULL) || (peripheral == NULL))
		return -EINVAL;

	if ((peripheral->p_type < 0) ||
			(peripheral->p_type >= IDI_MAX_PERIPHERAL))
		return -EINVAL;

	if ((handler == NULL) && (state == NULL))
		return -EINVAL;

	imc_idi = idi_controller_get_drvdata(idi);
	if (imc_idi == NULL)
		return -EINVAL;

	pdev = to_platform_device(idi->device.parent);

	if (idi_is_required) {
		if ((peripheral->p_type == IDI_BT_STREAM) ||
			(peripheral->p_type == IDI_AFE_STREAM) ||
				(peripheral->p_type == IDI_FMR_STREAM))
			set_bit((peripheral->p_type - 18),
					&imc_idi->stream_usage_flag);
		else
			set_bit((peripheral->p_type),
					&imc_idi->idi_usage_flag);
		smp_mb();
		pr_debug("idi_is_required = %d idi_usage_flag = 0x%08x\n",
				idi_is_required,
				(unsigned int)imc_idi->idi_usage_flag);
	} else {
		if ((peripheral->p_type == IDI_BT_STREAM) ||
			(peripheral->p_type == IDI_AFE_STREAM) ||
				(peripheral->p_type == IDI_FMR_STREAM))
			clear_bit((peripheral->p_type - 18),
					&imc_idi->stream_usage_flag);
		else
			clear_bit((peripheral->p_type),
					&imc_idi->idi_usage_flag);
		smp_mb();
		pr_debug(" idi_is_required = %d idi_usage_flag = 0x%08x\n",
				idi_is_required,
				(unsigned int)imc_idi->idi_usage_flag);
	}

	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	ctrl_state = imc_idi->idi_state;

	if ((imc_idi->idi_usage_flag == 0) &&
			(imc_idi->stream_usage_flag == 0))
			if (ctrl_state != DISABLE) {
				ctrl_state_new = DISABLE;
				idi_change_frequency = true;
				}

	if ((imc_idi->stream_usage_flag != 0) &&
			(imc_idi->idi_usage_flag == 0)) {
			if ((ctrl_state == ENABLE_AUDIO_ONLY) ||
				(ctrl_state == ENABLE_AUDIO_ONLY_ALT)) {
				idi_change_frequency = false;
				}
			else{
				idi_change_frequency = true;
				ctrl_state_new = ENABLE_AUDIO_ONLY;
				}
		}
	if (imc_idi->idi_usage_flag & BIT(IDI_WLAN)) {
		if ((ctrl_state == ENABLE_WLAN) ||
			(ctrl_state == ENABLE_WLAN_ALT)) {
			idi_change_frequency = false;
			}
		else{
			idi_change_frequency = true;
			ctrl_state_new = ENABLE_WLAN;
			}
		}
	else {
		if (imc_idi->idi_usage_flag & ~BIT(IDI_WLAN)) {
			if ((ctrl_state == ENABLE_NO_WLAN) ||
				(ctrl_state == ENABLE_NO_WLAN_ALT)) {
				idi_change_frequency = false;
				}
			else{
				idi_change_frequency = true;
				ctrl_state_new = ENABLE_NO_WLAN;
				}
			}
		}
	if (idi_change_frequency == true) {
		imc_idi->idi_state = ctrl_state_new;
		pr_debug(" ctrl_state = %d ctrl_state_new = %d\n",
				ctrl_state, ctrl_state_new);
	}
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);
	smp_mb();
#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (state == NULL)
		err = idi_peripheral_device_pm_set_state(peripheral, handler);
	else
		err =
		idi_peripheral_device_pm_set_state_by_name(peripheral, state);
	if (err)
		pr_err("Failed to set IDI device %d state\n",
						peripheral->p_type);
	if (idi_change_frequency == true) {
		err = platform_device_pm_set_state(pdev,
			imc_idi->idi_pm_state[ctrl_state_new]);
		if (err) {
			pr_err("Failed to set IDI controller state\n");
			return err;
		}
#if defined IDI_FM_SUPPORT
		else{
		switch (imc_idi->idi_state) {
		case ENABLE_AUDIO_ONLY:
			fm_info.info.freq_khz = IDI_FREQUENCY_ENABLE_AUDIO_ONLY;
			break;
		case ENABLE_WLAN:
			fm_info.info.freq_khz = IDI_FREQUENCY_ENABLE_WLAN;
			break;
		case ENABLE_NO_WLAN:
			fm_info.info.freq_khz = IDI_FREQUENCY_ENABLE_NO_WLAN;
			break;
		case DISABLE:
			fm_info.info.freq_khz = IDI_FREQUENCY_DISABLE;
			break;
		default:
			pr_err("Unexpected state of IDI\n");
			break;
		}

		fm_info.type = IUI_FM_FREQ_NOTIFICATION_TYPE_KHZ;
		ret = iui_fm_notify_frequency(IUI_FM_MACRO_ID_IDI, &fm_info);

		if (ret)
			pr_err("Failed to notify frequency manager\n");
		}
#endif
	}
#endif
	return err;
}

static int imc_mid_channel_flush(struct idi_controller_device *idi,
						int tx_not_rx, int channel)
{
	struct imc_controller *imc_idi = idi_controller_get_drvdata(idi);
	unsigned long flags;
	unsigned short *queue_busy;
	struct list_head *xfer_queue;

	IDI_IMC_ENTER;

	if (idi->channels[channel] == STREAM)
		imc_streaming_channel_flush(channel, imc_idi);
	else {

		if (tx_not_rx) {
			queue_busy = &imc_idi->tx_queue_busy;
			xfer_queue = &imc_idi->tx_queue[channel];
		} else {
			queue_busy = &imc_idi->rx_queue_busy;
			xfer_queue = &imc_idi->rx_queue[channel];
		}

	/* Set the xfer queue as BUSY */
	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	*queue_busy |= BIT(channel);
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

	/* Disable the ACREADY line not to be disturbed during flush */
	force_disable_acready(imc_idi);

	/* Flush the xfer queue */
	imc_flush_queue(xfer_queue, imc_idi);

	/* xfer queue can accept new transaction */
	spin_lock_irqsave(&imc_idi->sw_lock, flags);
	*queue_busy &= ~BIT(channel);
	spin_unlock_irqrestore(&imc_idi->sw_lock, flags);

		/* Ready to receive */
		unforce_disable_acready(imc_idi);
	}

	IDI_IMC_EXIT;
	return 0;
}

/**
 * imc_idi_device_init - initialise the IDI device callback functions
 * @idi: IDI controller reference
 * @imc_idi: IMC IDI controller reference
 */
static void imc_idi_device_init(struct idi_controller_device *idi,
				struct imc_controller *imc_idi)
{
	IDI_IMC_ENTER;

	idi->async = imc_mid_async;
	idi->flush = NULL;	/* flush everything */
	idi->channel_flush = imc_mid_channel_flush;
	idi->start_tx = NULL;
	idi->stop_tx = NULL;
	idi->xfer_complete = imc_mid_xfer_complete;
	idi->set_channel_config = imc_set_channel_config;
	idi->request_buffer_info = imc_buffer_info;
	idi->request_buffer_flush = imc_buffer_flush;
	idi->request_queue_status = imc_queue_status;
	idi->set_power_state = imc_set_power_state;
	idi_controller_set_drvdata(idi, imc_idi);

}				/* imc_idi_device_init() */

/**
 * imc_idi_device_exit - exit the IDI callback functions
 * @idi: IDI controller reference
 */
static void imc_idi_device_exit(struct idi_controller_device *idi)
{
	IDI_IMC_ENTER;

	idi->request_access = NULL;
	idi->release_access = NULL;
	idi->async = NULL;
	idi->setup = NULL;
	idi->flush = NULL;
	idi->start_tx = NULL;
	idi->stop_tx = NULL;
	idi_controller_set_drvdata(idi, NULL);

}				/* imc_idi_device_exit() */

/**
 * imc_controller_request_irqs - Request the imc controller irqs
 * @imc_idi: IMC IDI controller reference
 *
 * Returns success or an error code if all
		the controller IRQs cannot be requested.
 */

static int imc_controller_request_irqs(struct imc_controller *imc_idi)
{
	int i, err;

	for (i = 0; i < IMC_IDI_MAX_IRQS; i++) {
		if (imc_idi->irqs[i] == -1)
			return 0;

		err = request_irq(imc_idi->irqs[i], imc_isr,
				  IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND
								| IRQF_SHARED,
				  IDI_MPU_IRQ_NAME, imc_idi);
		if (err) {
			dev_err(imc_idi->dev,
				"Error (%d) while requesting IRQ %d\n",
				err, imc_idi->irqs[i]);
			return err;
		}
	}

	return 0;
}

/**
 * imc_controller_free_irqs - free all the IMC controller irqs
 * @imc_idi: IMC IDI controller reference
 *
 * This function frees all the IRQs previously requested by the driver.
 */

static void imc_controller_free_irqs(struct imc_controller *imc_idi)
{
	int i;

	for (i = 0; i < IMC_IDI_MAX_IRQS; i++) {
		if (imc_idi->irqs[i] == -1)
			return;

		free_irq(imc_idi->irqs[i] , imc_idi);
	}
}

/**
 * imc_controller_enable_irqs - Enable the IMC IDI controller IRQs
 * @imc_idi: IMC IDI controller reference
 *
 * This function enables all the IRQs.
 */

static void imc_controller_enable_irqs(struct imc_controller *imc_idi)
{
	int i;

	for (i = 0; i < IMC_IDI_MAX_IRQS; i++) {
		if (imc_idi->irqs[i] == -1)
			return;

		enable_irq(imc_idi->irqs[i]);
	}
}

static inline void _imc_controller_disable_irqs(struct imc_controller *imc_idi,
								 int nosync)
{
	int i;
	void (*_imc_idi_disable_irq)(unsigned int);

	if (nosync)
		_imc_idi_disable_irq = disable_irq;

	for (i = 0; i < IMC_IDI_MAX_IRQS; i++) {
		if (imc_idi->irqs[i] == -1)
			return;
		/*FIXME: When nosync = 0*/
		if (nosync)
			_imc_idi_disable_irq(imc_idi->irqs[i]);
	}
}

/**
 * imc_controller_disable_irqs - Disable the IMC IDI controller IRQs
 * @imc_idi: IMC IDI controller reference
 *
 * This function disables all the IRQs.
 */


static void imc_controller_disable_irqs(struct imc_controller *imc_idi)
{
	_imc_controller_disable_irqs(imc_idi, 0);
}

/**
 * imc_controller_disable_irqs_nosync - Disable the IMC IDI controller IRQs
 * @imc_idi: IMC IDI controller reference
 *
 * This function disables all the IRQs.
 */

static void imc_controller_disable_irqs_nosync(struct imc_controller *imc_idi)
{
	_imc_controller_disable_irqs(imc_idi, 1);
}

/**
 * imc_controller_init - initialise the controller structure
 * @imc_idi: IMC IDI controller reference
 *
 * Returns success or an error code if the controller IRQ cannot be requested.
 * This function initializes all the data structures needed by the driver.
 */
static int imc_controller_init(struct imc_controller *imc_idi)
{
	unsigned int ch;
	int err = 0;

	IDI_IMC_ENTER;

	for (ch = 0; ch < IDI_MAX_CHANNEL; ch++) {
		INIT_LIST_HEAD(&imc_idi->tx_queue[ch]);
		INIT_LIST_HEAD(&imc_idi->rx_queue[ch]);
	}
	INIT_LIST_HEAD(&imc_idi->brk_queue);
	INIT_LIST_HEAD(&imc_idi->fwd_queue);

	spin_lock_init(&imc_idi->sw_lock);
	spin_lock_init(&imc_idi->hw_lock);

	tasklet_init(&imc_idi->isr_tasklet, imc_isr_tasklet,
		     (unsigned long)imc_idi);
	tasklet_init(&imc_idi->fwd_tasklet, imc_fwd_tasklet,
		     (unsigned long)imc_idi);

	init_timer(&imc_idi->tx_idle_poll);
	imc_idi->tx_idle_poll.data = (unsigned long)imc_idi;
	imc_idi->tx_idle_poll.function = tx_idle_poll;

	init_timer(&imc_idi->rx_idle_poll);
	imc_idi->rx_idle_poll.data = (unsigned long)imc_idi;
	imc_idi->rx_idle_poll.function = rx_idle_poll;

#ifdef IDLE_POLL
	/*
	 * This may replace tx_idle and rx_idle polls.  But not yet.  Base this
	 * on the IDLE interrupt.
	 */
	init_timer(&imc_idi->idle_poll);
	imc_idi->tx_idle_poll.data = (unsigned long)imc_idi;
	imc_idi->tx_idle_poll.function = idle_poll;
#endif
	err = imc_controller_request_irqs(imc_idi);
	if (err < 0)
		dev_err(imc_idi->dev, "Request IRQs failed (%d)\n", err);

	imc_idi->idi_usage_flag = 0;
	imc_idi->stream_usage_flag = 0;

	imc_idi->txbuf_dummy = dma_alloc_coherent(NULL,
					IDI_DUMMY_TX_BUFFER_SIZE,
					&imc_idi->txbuf_dummy_dma,
					GFP_KERNEL);

	imc_idi->rxbuf_dummy = dma_alloc_coherent(NULL,
					IDI_DUMMY_RX_BUFFER_SIZE,
					&imc_idi->rxbuf_dummy_dma,
					GFP_KERNEL);

	return err;
}				/* imc_controller_init() */

/**
 * imc_controller_exit - set the controller driver to a reset state
 * @imc_idi: IMC IDI controller reference
 */
static void imc_controller_exit(struct imc_controller *imc_idi)
{
	IDI_IMC_ENTER;

	/* Reset the IDI hardware */
	imc_ctrl_clean_reset(imc_idi);

	/* Free the interrupt */
	imc_controller_free_irqs(imc_idi);

	/* Kill the tasklets */
	tasklet_kill(&imc_idi->isr_tasklet);
	tasklet_kill(&imc_idi->fwd_tasklet);

	dma_free_coherent(NULL, IDI_DUMMY_RX_BUFFER_SIZE,
			imc_idi->rxbuf_dummy, imc_idi->rxbuf_dummy_dma);
	dma_free_coherent(NULL, IDI_DUMMY_TX_BUFFER_SIZE,
			imc_idi->txbuf_dummy, imc_idi->txbuf_dummy_dma);
}				/* imc_controller_exit() */

struct imc_controller *imc_alloc_controller(struct device *dev)
{
	struct idi_controller_device *idi;
	struct imc_controller *imc_idi;

	idi = idi_alloc_controller(sizeof(struct imc_controller), dev);
	if (!idi)
		return ERR_PTR(-ENOMEM);

	imc_idi = idi_controller_priv(idi);
	imc_idi->idi = idi;

	memset(imc_idi->irqs, -1, ARRAY_SIZE(imc_idi->irqs));
	/* FIXME: not sure this code is Ok.. */
	imc_idi->dev = &idi->device;

	return imc_idi;
}
EXPORT_SYMBOL(imc_alloc_controller);

void imc_free_controller(struct imc_controller *imc_idi)
{
	kfree(imc_idi);
}
EXPORT_SYMBOL(imc_free_controller);

static int imc_init_irq_channels(struct imc_controller *imc_idi)
{
	struct idi_controller_device *idi = imc_idi->idi;
	void __iomem *ctrl = imc_idi->ctrl_io;
	int ch;

	/*
	 * By default, a channel interrupt is disabled
	 * The set_channel_configuration() will set the irq mask
	 * for STREAM and DMA channel type.
	 * Just enable/disable accordingly the outstanding/software
	 *  channels
	 *
	*/
	for (ch = 0; ch < IDI_MAX_CHANNEL; ch++) {
		if (idi->channels[ch] == OUTSTANDING_READ) {
			imc_idi_unmask_rx_irq_ch(imc_idi, ch);
			imc_idi_unmask_tx_irq_ch(imc_idi, ch);
			continue;
		}

		if (idi->channels[ch] == SOFTWARE_DEFINED) {
			imc_idi_mask_tx_irq_ch(imc_idi, ch);
			imc_idi_unmask_rx_irq_ch(imc_idi, ch);
			continue;
		}

		imc_idi_mask_rx_irq_ch(imc_idi, ch);
		imc_idi_mask_tx_irq_ch(imc_idi, ch);
	}

	imc_idi->rxmask_cfg = ioread32(IMC_IDI_RXMASK_CON(ctrl));
	imc_idi->txmask_cfg = ioread32(IMC_IDI_TXMASK_CON(ctrl));

	/* TODO: Multiprocessor support */
	imc_idi->tx2mask_cfg = 0;
	imc_idi->tx3mask_cfg = 0;
	imc_idi->rx2mask_cfg = 0;
	imc_idi->rx3mask_cfg = 0;

	return 0;
}

/**
 * imc_add_controller - make and init imc_idi controller
 * @imc: IDI/IMC controller reference
 *
 * Allocate imc_idi controller, attach to idi_controller, activate
 * PCI device and map memory for IDI and master DMA, init ports, and
 * register controller with IDI (perform board info scan there).
 *
 * Returns success or an error code if any initialization is failing.
 */
int imc_add_controller(struct imc_controller *imc_idi)
{
	int err = 0;
	void __iomem *ctrl;
	u32 reg = 0;
	struct idi_controller_device *idi = imc_idi->idi;

	IDI_IMC_ENTER;

	imc_idi_device_init(imc_idi->idi, imc_idi);

	err = imc_controller_init(imc_idi);
	if (err < 0)
		goto fail_controller_init;

	err = imc_ctrl_full_reset(imc_idi);
	if (err < 0)
		goto fail_controller_reset;

	err = imc_init_irq_channels(imc_idi);
	if (err < 0)
		goto fail_controller_reset;

	err = imc_device_init(imc_idi);
	if (err < 0)
		goto fail_controller_reset;

#ifdef CONFIG_DEBUG_FS
	err = imc_debug_add_ctrl(idi);
	if (err < 0)
		goto fail_add_debug;
#endif

	err = idi_register_controller(idi);
	if (err < 0)
		goto fail_controller_register;

	ctrl = imc_idi->ctrl_io;
	reg = ioread32(IMC_IDI_ID(ctrl));

	dev_info(imc_idi->dev, ": Rev: %d   Mod ID: %d   TS Rev: %d\n",
		IMC_IDI_ID_REV_NUMBER(reg),
		IMC_IDI_ID_MOD_ID(reg), IMC_IDI_ID_TS_REV_NR(reg));

	dev_info(imc_idi->dev, ": Software check: %d channels\n",
		(ioread32(IMC_IDI_SWCID(ctrl)) & IMC_IDI_SWCID_CH) ? 16 : 8);
	dev_info(imc_idi->dev, ": SRB Multiple Source: 0x%08x\n",
		ioread32(IMC_IDI_SRB_MSCONF_ID(ctrl)));
	dev_info(imc_idi->dev, ": SRB Error Config: 0x%08x\n",
		ioread32(IMC_IDI_SRB_ERRCONF_ID(ctrl)));

	return 0;

fail_controller_register:
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(imc_idi->dir);
fail_add_debug:
#endif
fail_controller_reset:
	imc_controller_exit(imc_idi);
fail_controller_init:

	return err;

}				/* imc_add_controller() */
EXPORT_SYMBOL(imc_add_controller);

/**
 * imc_remove_controller - stop controller and unregister with IDI
 * @imc: IDI controller reference
 * @pdev: PCI device reference
 *
 * Stop controller and unregister with IDI
 */
void imc_remove_controller(struct imc_controller *imc_idi)
{
	struct idi_controller_device *idi = imc_idi->idi;
	IDI_IMC_ENTER;

	idi_unregister_controller(imc_idi->idi);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(imc_idi->dir);
#endif
	imc_controller_exit(imc_idi);
	imc_idi_device_exit(idi);
}				/* imc_remove_controller() */
EXPORT_SYMBOL(imc_remove_controller);

MODULE_ALIAS("MPE");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("Intel Mobile Communications IDI Controller Driver");
MODULE_LICENSE("GPL v2");

