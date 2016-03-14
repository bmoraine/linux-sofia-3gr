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
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>
#include <linux/of.h>
#include <linux/clk-provider.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_tty.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_device_pm.h>

#include "imc_ag620_scu.h"
#include "imc_idi_btif.h"
#include "imc_idi_btauif.h"
#include <linux/imc_idi_btauif_driverif.h>
#include <linux/imc_idi_bt_ioctl.h>

#include <linux/netlink.h>
#include <net/sock.h>

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>

/* #define ENABLE_DAP_CLOCK */

#define BTIF_REG_RES_NAME	"register"
#define BTIF_RX_RES_NAME	"rx fifo"
#define BTIF_TX_RES_NAME	"tx fifo"
#define BTIF_SCU_RES_NAME	"scu"
#define BTIF_IRQ_NAME		"btif_req_irq"
#define BTIF_IRQ_SRC_NAME	"btif_mult_src"
#define BTIF_IRQ_ERR_NAME	"btif_err"
#define BTIP_IRQ_WAKEUP_NAME	"btip_wakeup"

#define BTIF_CLK_KERNEL_NAME "kernel"
#define BTIF_CLK_78M_NAME "btip_78M"
#define BTIF_CLK_104M_NAME "btip_104M"
#define BTIF_CLK_RTC_NAME "btip_rtc"
#define BTIF_REQ_VCXO_NAME "btip_vcxo"
#define BTIF_REQ_PWM_ON_NAME "btip_pwm_on"
#define BTIF_REQ_RF_ON_NAME "btip_rf_on"

#define SERIAL_BTIF_NAME  "ttyBT"
/* FIXME: Get the major and cleanly !*/
#define SERIAL_BTIF_MAJOR 4
#define SERIAL_BTIF_MINOR 204

#define BTIF_CONFIG_MODE 0xAA6
#define BTIF_RUN_MODE 0xAA5
#define BTIF_DISABLE 0xAAA

#define BT_AU_IF_CONFIG_MODE 0x6
#define BT_AU_IF_RUN_MODE 0x5

/* This could/should be dts parameter*/
#define BTIF_RX_TMO 8192

#define IMC_IDI_BTIF_TEMP_MEAS_INTERVAL_MSECS 2100
#define IMC_IDI_BTIF_TEMP_MEAS_TREF 38
#define IMC_IDI_BTIF_TEMP_MEAS_KT 32
#define IMC_IDI_BTIF_TEMP_MEAS_KT_SCALE 100

/*Enable for AG6XX chip*/
#define CONFIG_AG6XX

#ifdef CONFIG_AG6XX
#define IMC_IDI_BT_POWER_STATE_OFF_D3  5
#define IMC_IDI_BT_POWER_STATE_ON_D0  0
#endif
#define IMC_IDI_BTIF_ENTER pr_debug("--> %s\n", __func__);
#define IMC_IDI_BTIF_EXIT pr_debug("<-- %s\n", __func__);

#define IMC_IDI_BT_AU_IF_ENTER pr_debug("--> %s\n", __func__);
#define IMC_IDI_BT_AU_IF_EXIT pr_debug("<-- %s\n", __func__);

static int g_bt_audio_state;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock btauif_wake_lock;
#endif
struct timer_list temp_meas_timer;
static unsigned stack_pid;

enum netlink_message_code { HWUP_HIGH, HWUP_LOW, CTS_HIGH, CTS_LOW };
enum idi_devices_power_state { D0, D0I0, D0I1, D0I2, D0I3, D3, NA };

struct imc_idi_btif_platdata {
	struct clk *kernel;
	struct clk *bt_78M;
	struct clk *bt_104M;
	struct clk *bt_rtc;
	/* RCCONF clocks */
	struct clk *bt_vcxo;
	struct clk *bt_pwm_on;
	struct clk *bt_rf_on;
	/* Reset controller */
	struct reset_control *btif_rst;
	struct reset_control *btip_rst;
};

#define TX_BUFFER_SIZE 2048

#define AG610_BTIF 0x03015201
#define AG620_BTIF 0x03075202

struct imc_idi_btif_port {
	struct uart_port port;
	unsigned id;
	unsigned char state;
	unsigned char dstate;
	bool signaling_disabled;
	bool first_wakeup;
	bool tx_ongoing;
	struct idi_peripheral_device *p_dev;
	void __iomem *ctrl_io;
	void __iomem *scu_io;
	struct idi_transaction *tx_trans;
	struct idi_channel_config rx_ch_config;
	struct idi_channel_config tx_ch_config;
	spinlock_t rxbuff_lock;
	spinlock_t txstate_lock;
	spinlock_t hw_lock;
	spinlock_t state_lock;
	struct list_head rxbuff_busy_queue;
	struct sock *nl_socket;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock btif_wake_lock;
#endif
	struct iio_channel *temp_iio_chan;
	struct work_struct temp_meas_work;
	struct workqueue_struct *temp_meas_wq;
	unsigned srb_msconf;
	unsigned err_msconf;
	unsigned rx_mis;
	unsigned imsc;
	unsigned rps_stage;
	unsigned rx_fifosize;
	unsigned tx_fifosize;
	struct device_state_pm_state *current_pm_state;
	unsigned char tx_bounce[TX_BUFFER_SIZE] ____cacheline_aligned;
};

struct imc_idi_btif_pm_state {
	struct device_state_pm_state *d0_handler;
	struct device_state_pm_state *d0i2_handler;
	struct device_state_pm_state *d0i3_handler;
	struct device_state_pm_state *d3_handler;
};

static struct imc_idi_btif_pm_state btif_states;

#define to_imc_idi_btif_port(p)	\
	((container_of((p), struct imc_idi_btif_port, port)))

#define BT_OFF		(0)
#define BT_ON		(1 << 0)
#define BT_WAKEUP	(1 << 1)
#define BT_RX		(1 << 2)
#define BT_TX		(1 << 3)

#define BTIF_STATE_CLEAR_FLAGS 0
#define BTIF_STATE_ADD_FLAGS 1
#define BTIF_STATE_SET_FLAGS 2


static int imc_idi_btauif_audio_enable(unsigned char clk_rate,
					void __iomem *ctrl_io);
static int imc_idi_btauif_audio_enable_ex(struct imc_idi_btauif_config
					*config_params, void __iomem *ctrl_io);
static int imc_idi_btauif_audio_disable(void __iomem *ctrl_io);
static int btif_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg);


static unsigned _btif_set_state(struct imc_idi_btif_port *p_btif,
				unsigned char state, unsigned operation)
{
	unsigned long flags;
	spin_lock_irqsave(&p_btif->state_lock, flags);
	switch (operation) {
	case BTIF_STATE_CLEAR_FLAGS:
		p_btif->state &= ~(state);
		break;
	case BTIF_STATE_ADD_FLAGS:
		p_btif->state |= state;
		break;
	case BTIF_STATE_SET_FLAGS:
		p_btif->state = state;
		break;
	}
	spin_unlock_irqrestore(&p_btif->state_lock, flags);

	return p_btif->state;
}

static inline unsigned btif_enter_state(struct imc_idi_btif_port *p_btif,
					unsigned state)
{
	struct device *dev = &p_btif->p_dev->device;
	dev_dbg(dev, "Entering state %u\n", state);
	return _btif_set_state(p_btif, state, BTIF_STATE_ADD_FLAGS);
}

static inline unsigned btif_exit_state(struct imc_idi_btif_port *p_btif,
				       unsigned state)
{
	struct device *dev = &p_btif->p_dev->device;
	dev_dbg(dev, "Exiting state %u\n", state);
	return _btif_set_state(p_btif, state, BTIF_STATE_CLEAR_FLAGS);
}

static inline unsigned btif_set_state(struct imc_idi_btif_port *p_btif,
				      unsigned state)
{
	struct device *dev = &p_btif->p_dev->device;
	dev_dbg(dev, "Set state %u\n", state);
	return _btif_set_state(p_btif, state, BTIF_STATE_SET_FLAGS);
}

#if defined(CONFIG_OF)
static struct imc_idi_btif_platdata *imc_idi_btif_get_platdata(
					struct imc_idi_btif_port *p_btif)
{
	return (struct imc_idi_btif_platdata *)
	    dev_get_platdata(&p_btif->p_dev->device);
}
#endif

static void dump_xfer(struct imc_idi_btif_port *p_btif,
		      const char *prefix_str, struct idi_xfer *xfer)
{
	int rowsize = 16;
	int groupsize = 1;
	size_t sglen, payload;
	bool ascii = false;
	struct device *dev = &p_btif->p_dev->device;
	struct scatterlist *sg = &xfer->sg[0];
	const u8 *buffer = (const u8 *) xfer->cpu_base;
	int i, linelen, remaining, line_remaining_space = 0;
	unsigned char linebuf[32 * 3 + 2 + 32 + 1];
	remaining = sglen = payload = xfer->size;
	dev_dbg(dev, "%s: %x@%p\n", prefix_str, xfer->size, xfer->cpu_base);

	do {
		if (!(xfer->channel_opts & IDI_TX_CHANNEL)) {
			buffer = idi_xfer_dma_to_virt(dev, xfer, sg);
			remaining = sglen = sg_dma_len(sg);
		}

		if (unlikely(line_remaining_space)) {
			if (remaining < line_remaining_space)
				line_remaining_space = remaining;
			hex_dump_to_buffer(buffer, line_remaining_space,
				rowsize, groupsize, &linebuf[(linelen * 3) - 1],
				sizeof(linebuf), ascii);

			dev_dbg(dev, "%s%s\n", prefix_str, linebuf);

			buffer += line_remaining_space;
			remaining -= line_remaining_space;
			payload -= line_remaining_space;
			sglen = remaining;
			line_remaining_space = 0;
		}

		for (i = 0; i < sglen; i += rowsize) {
			linelen = min(remaining, rowsize);
			remaining -= rowsize;

			hex_dump_to_buffer(buffer + i, linelen, rowsize,
				groupsize, linebuf, sizeof(linebuf), ascii);

			if (unlikely((payload > sglen) && (remaining < 0))) {
				line_remaining_space = remaining * (-1);
				break;
			}
			dev_dbg(dev, "%s%s\n", prefix_str, linebuf);
		}

		payload -= sglen;
		if (!(xfer->channel_opts & IDI_TX_CHANNEL))
			sg = sg_next(sg);
	} while (payload);
}

/*
 * btif_set_interrupt : Set or clear interrupt source in IMSC register
 *
 *	returns imsc value written
 */
static inline unsigned btif_set_interrupt(struct imc_idi_btif_port *p_btif,
					  unsigned imsc, unsigned set_not_clear)
{
	unsigned reg;
	unsigned long flags;
	struct device *dev = &p_btif->p_dev->device;
	void __iomem *ctrl = p_btif->ctrl_io;

	spin_lock_irqsave(&p_btif->hw_lock, flags);
	reg = ioread32(BTIF_IMSC(ctrl));
	if (set_not_clear)
		reg |= imsc;
	else
		reg &= ~(imsc);
	iowrite32(reg, BTIF_IMSC(ctrl));
	ioread32(BTIF_IMSC(ctrl));	/* Dummy read */
	spin_unlock_irqrestore(&p_btif->hw_lock, flags);

	dev_dbg(dev, " IMSC has been set to %#x\n", reg);

	return reg;
}

static inline void imc_idi_btif_wakeup_bt(struct imc_idi_btif_port *p_btif,
					  bool wake)
{
	unsigned reg;

	if (idi_client_ioread(p_btif->p_dev,
			(unsigned)SCU_BT_CTL(p_btif->scu_io), &reg)) {
		dev_err(&p_btif->p_dev->device,
				"%s: idi read error %p\n", __func__,
				SCU_BT_CTL(p_btif->scu_io));
		return;
	}

	reg &= ~SCU_BT_CTL_WAKEUP_BT_MASK;

	if (wake)
		reg |= SCU_BT_CTL_WAKEUP_BT_WUP_REQ;

	if (idi_client_iowrite(p_btif->p_dev,
			(unsigned)SCU_BT_CTL(p_btif->scu_io), reg))
		dev_err(&p_btif->p_dev->device,
				"%s: idi write error %p\n", __func__,
				SCU_BT_CTL(p_btif->scu_io));
}

static inline void imc_idi_btif_set_rts(struct imc_idi_btif_port *p_btif,
					bool high)
{
	if (high)
		iowrite32(BTIF_MSS_SET_FCOSET_FCO,
			  BTIF_MSS_SET(p_btif->ctrl_io));
	else
		iowrite32(BTIF_MSS_CLR_FCOCLR_FCO,
			  BTIF_MSS_CLR(p_btif->ctrl_io));
}

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT

static void _imc_btip_spcu_req(struct imc_idi_btif_platdata *platdata,
			       bool enable)
{
	if (enable) {
		if (platdata->bt_vcxo) {
			clk_prepare(platdata->bt_vcxo);
			clk_enable(platdata->bt_vcxo);
		}

		if (platdata->bt_pwm_on) {
			clk_prepare(platdata->bt_pwm_on);
			clk_enable(platdata->bt_pwm_on);
		}

		if (platdata->bt_rf_on) {
			clk_prepare(platdata->bt_rf_on);
			clk_enable(platdata->bt_rf_on);
		}
	} else {
		if (platdata->bt_vcxo) {
			clk_disable(platdata->bt_vcxo);
			clk_unprepare(platdata->bt_vcxo);
		}

		if (platdata->bt_pwm_on) {
			clk_disable(platdata->bt_pwm_on);
			clk_unprepare(platdata->bt_pwm_on);
		}

		if (platdata->bt_rf_on) {
			clk_disable(platdata->bt_rf_on);
			clk_unprepare(platdata->bt_rf_on);
		}
	}
}

#endif

static void _imc_btip_wait_clk(struct imc_idi_btif_port *p_btif)
{
	int timeout = 50;
	unsigned reg;

	/* waiting that clocks are delivered to BT macro */
	do {
		if (idi_client_ioread(p_btif->p_dev,
					(unsigned)SCU_BT_STAT(p_btif->scu_io),
					&reg)) {
			dev_err(&p_btif->p_dev->device,
					"%s: idi read error %p\n",
					__func__, SCU_BT_STAT(p_btif->scu_io));
			return;
		}

		reg = SCU_BT_STAT_CLK78REQ(reg);

		if (!reg) {
			mdelay(1);
			if (timeout-- < 0) {
				pr_err("BT 78M clk not requested\n");
				return;
			}
		}
	} while (!reg);

#ifdef IMC_IDI_BTIF_PARANOID
	timeout = 50;
	do {
		if (idi_client_ioread(p_btif->p_dev,
					(unsigned)SCU_BT_STAT(p_btif->scu_io),
					&reg)) {
			dev_err(&p_btif->p_dev->device,
					"%s: idi read error %p\n",
					__func__, SCU_BT_STAT(p_btif->scu_io));
			return;
		}

		reg = SCU_BT_STAT_CLK104REQ(reg);

		if (!reg) {
			mdelay(1);
			if (timeout-- < 0) {
				pr_err("BT 104M clk not requested\n");
				return;
			}
		}
	} while (!reg);
#endif
}

static void imc_idi_btip_enable(struct imc_idi_btif_port *p_btif)
{
#if 0
	struct imc_idi_btif_platdata *platdata =
	    imc_idi_btif_get_platdata(p_btif);
#ifndef CONFIG_COMMON_CLK
	int ret;
#endif

	_imc_btip_spcu_req(platdata, true);

#ifdef CONFIG_COMMON_CLK
	if (platdata->bt_rtc) {
		clk_prepare(platdata->bt_rtc);
		clk_enable(platdata->bt_rtc);
	}

	if (platdata->bt_78M) {
		clk_prepare(platdata->bt_78M);
		clk_enable(platdata->bt_78M);
	}

	if (platdata->bt_104M) {
		clk_prepare(platdata->bt_104M);
		clk_enable(platdata->bt_104M);
	}
#endif
#endif
	/* Get prepared for default BD data event before releasing reset */
	btif_set_state(p_btif, BT_ON);
	btif_enter_state(p_btif, BT_RX);

	p_btif->first_wakeup = 1;
}

static void imc_idi_btip_disable(struct imc_idi_btif_port *p_btif)
{
#if 0
	struct imc_idi_btif_platdata *platdata =
				imc_idi_btif_get_platdata(p_btif);
#ifdef CONFIG_COMMON_CLK
	if (platdata->bt_78M) {
		clk_disable(platdata->bt_78M);
		clk_unprepare(platdata->bt_78M);
	}

	if (platdata->bt_104M) {
		clk_disable(platdata->bt_104M);
		clk_unprepare(platdata->bt_104M);
	}

	if (platdata->bt_rtc) {
		clk_disable(platdata->bt_rtc);
		clk_unprepare(platdata->bt_rtc);
	}
#endif
#endif
#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	_imc_btip_spcu_req(platdata, false);

#endif
	btif_set_state(p_btif, BT_OFF);
}

static void imc_btip_configure_fw(struct imc_idi_btif_port *p_btif,
				  bool disable_signaling)
{
	unsigned reg;

#define _16_BIT_HCI_ALIGNMENT_ALIVE_INDICATION_HANDLING_OFF (0<<0)
#define _16_BIT_HCI_ALIGNMENT_ALIVE_INDICATION_HANDLING_ON (1<<0)
#define NOKIA_MODE_FOR_INFINEON_SPECIFIC_EVENTS_OFF (0<<1)
#define NOKIA_MODE_FOR_INFINEON_SPECIFIC_EVENTS_ON (1<<1)
#define DISABLE_NOKIA_SPECIFIC_COMMANDS_EVENTS (0<<2)
#define ENABLE_NOKIA_SPECIFIC_COMMANDS_EVENTS (1<<2)
#define ROM_CHECKSUM_CALCULATION_DURING_BOOT (1<<3)
#define EXTERNAL_HCI_TEST_INTERFACE_UART_IS_USED (0<<4)
#define INTERNAL_BTIF_IS_USED (1<<4)
#define WAKEUP_HOST_IS_ALWAYS_KEPT_ACTIVE (0<<5)
#define WAKEUP_HOST_ACCORDING_H4N (1<<5)

#define HIGH_SPEED_ARC_CLOCK (1<<6)
#define HIGH_SPEED_PLL_CLOCK (1<<7)

	if (idi_client_ioread(p_btif->p_dev,
				(unsigned)SCU_BT_FWCTL(p_btif->scu_io), &reg)) {
		dev_err(&p_btif->p_dev->device,
				"%s: idi read error %p\n", __func__,
				SCU_BT_FWCTL(p_btif->scu_io));
		return;
	}

	reg |= HIGH_SPEED_ARC_CLOCK;
	reg |= HIGH_SPEED_PLL_CLOCK;
	reg |= INTERNAL_BTIF_IS_USED;

	if (disable_signaling)
		reg &= ~(WAKEUP_HOST_ACCORDING_H4N);
	else
		reg |= WAKEUP_HOST_ACCORDING_H4N;

	if (idi_client_iowrite(p_btif->p_dev,
				(unsigned)SCU_BT_FWCTL(p_btif->scu_io), reg))
		dev_err(&p_btif->p_dev->device,
				"%s: idi write error %p\n", __func__,
				SCU_BT_FWCTL(p_btif->scu_io));
}

static void imc_btip_init(struct imc_idi_btif_port *p_btif)
{
	unsigned reg;

	/* SCU static configuration (no external interactions) */
	reg = SCU_BT_CTL_WAKEUP_BT_NO_WUP_REQ
	    | SCU_BT_CTL_BT_EXT_PCM_INTERNAL
	    | SCU_BT_CTL_BT_EXT_UART_INTERNAL
	    | SCU_BT_CTL_BT_EXT_RST_INTERNAL
	    | SCU_BT_CTL_BT_EXT_WAKEUP_INTERNAL;

	reg |= SCU_BT_CTL_ARC_START_START;

	if (idi_client_iowrite(p_btif->p_dev,
				(unsigned)SCU_BT_CTL(p_btif->scu_io), reg))
		dev_err(&p_btif->p_dev->device,
				"%s: idi write error %p\n", __func__,
				SCU_BT_CTL(p_btif->scu_io));
}

/*
 *
 * End of REWORK NEEDED section
 *
 *
 */

#ifdef IMC_IDI_BTIF_DMA_FLOWCONTROL
static irqreturn_t imc_idi_btif_isr(int irq, void *dev_id)
{

	struct imc_idi_btif_port *p_btif = (struct imc_idi_btif_port *)dev_id;
	void __iomem *ctrl = p_btif->ctrl_io;
	struct device *dev = &p_btif->p_dev->device;
	unsigned mis;
	IMC_IDI_BTIF_ENTER;
	mis = ioread32(BTIF_MIS(ctrl));
	dev_dbg(dev, "%s: MIS %x\n", __func__, mis);

	return IRQ_HANDLED;
}
#endif

static irqreturn_t imc_idi_btif_src_isr(int irq, void *dev_id)
{
	struct imc_idi_btif_port *p_btif = (struct imc_idi_btif_port *)dev_id;
	void __iomem *ctrl = p_btif->ctrl_io;
	irqreturn_t ret = IRQ_NONE;
	unsigned mis, mis_src;
	IMC_IDI_BTIF_ENTER;
	mis = ioread32(BTIF_MIS(ctrl));
	mis_src = mis & p_btif->srb_msconf;

#ifdef IMC_IDI_BTIF_PARANOID
	if (mis_src) {
#endif
		p_btif->imsc = ioread32(BTIF_IMSC(ctrl));
		/* Mask interrupts to let soft irq handler being executed */
		btif_set_interrupt(p_btif, p_btif->srb_msconf, 0);
		iowrite32(mis_src, BTIF_ICR(ctrl));
		ioread32(BTIF_RIS(ctrl));	/* Dummy read */
		p_btif->rx_mis = mis_src;

		ret = IRQ_WAKE_THREAD;
		goto exit_src_isr;
#ifdef IMC_IDI_BTIF_PARANOID
	}
#endif

exit_src_isr:
	IMC_IDI_BTIF_EXIT;
	return ret;
}

static void btif_rx_transaction_complete(struct idi_transaction *trans)
{
	struct imc_idi_btif_port *p_btif =
	    dev_get_drvdata(&trans->peripheral->device);
	unsigned long flags;
	struct uart_port *port = &p_btif->port;
	struct tty_struct *tty = port->state->port.tty;
	struct idi_xfer *xfer = &trans->idi_xfer;
#ifdef IMC_IDI_BTIF_PARANOID
	struct list_head *busyqueue = &p_btif->rxbuff_busy_queue;
#endif

	IMC_IDI_BTIF_ENTER;

	spin_lock_irqsave(&p_btif->rxbuff_lock, flags);
#ifdef IMC_IDI_BTIF_PARANOID
	if (unlikely(list_empty(busyqueue)))
		BUG();
#endif

	list_del(&trans->queue);
	spin_unlock_irqrestore(&p_btif->rxbuff_lock, flags);

	/* Insert, and push chars to tty buffer */
	if (trans->status == IDI_STATUS_COMPLETE) {
		port->icount.rx +=
			idi_push_xfer_to_tty(p_btif->p_dev, xfer, tty);
		dump_xfer(p_btif, " <--- ", xfer);
	}

	idi_free_transaction(trans);

	IMC_IDI_BTIF_EXIT;
}

static struct idi_transaction *btif_alloc_rxtrans(
				struct imc_idi_btif_port *p_btif,
				size_t size)
{
	struct idi_transaction *trans;

	trans = idi_alloc_transaction(GFP_KERNEL);
#ifdef IMC_IDI_BTIF_PARANOID
	if (unlikely(!trans))
		return ERR_PTR(-ENOMEM);
#endif

	trans->idi_xfer.size = size;
	trans->idi_xfer.channel_opts = IDI_PRIMARY_CHANNEL;

	trans->complete = btif_rx_transaction_complete;

	return trans;
}

static int imc_idi_btif_read(struct imc_idi_btif_port *p_btif)
{
	void __iomem *ctrl = p_btif->ctrl_io;
	struct device *dev = &p_btif->p_dev->device;
	struct idi_peripheral_device *p_device = p_btif->p_dev;
	struct list_head *busyqueue = &p_btif->rxbuff_busy_queue;
	struct idi_transaction *trans;
	unsigned payload, reg;
	unsigned long flags;
	int ret;

	IMC_IDI_BTIF_ENTER;

	while (1) {
		reg = ioread32(BTIF_RPS_STAT(ctrl));
		payload = BTIF_RPS_STAT_RPS(reg);

		/* Allocate transaction */
		trans = btif_alloc_rxtrans(p_btif, payload);

		spin_lock_irqsave(&p_btif->rxbuff_lock, flags);
		list_add_tail(&trans->queue, busyqueue);
		spin_unlock_irqrestore(&p_btif->rxbuff_lock, flags);

		dev_dbg(dev, "%s:Queing async read, size %x, trans = %p\n",
			__func__, payload, trans);

		ret = idi_async_read(p_device, trans);
#ifdef IMC_IDI_BTIF_PARANOID
		if (unlikely(ret)) {
			dev_err(dev, "%s:Queing async read failed\n", __func__);
			return ret;
		}
#endif

		reg = ioread32(BTIF_RIS(ctrl));
		if (!(BTIF_RIS_RX_EOP(reg) || BTIF_RIS_RX_MRPSSTOP(reg)))
			break;
		iowrite32(BTIF_RIS_RX_EOP_MASK | BTIF_RIS_RX_MRPSSTOP_MASK,
				BTIF_ICR(ctrl));
		ioread32(BTIF_RIS(ctrl));	/* Dummy read */
	}

	IMC_IDI_BTIF_EXIT;
	return 0;
}

static void btif_idi_start_tx(struct idi_transaction *trans)
{
	struct imc_idi_btif_port *p_btif =
	    dev_get_drvdata(&trans->peripheral->device);
	struct idi_xfer *xfer = &trans->idi_xfer;

	iowrite32(xfer->size, BTIF_TPS_CTRL(p_btif->ctrl_io));
}

static void btif_stop_tx(struct uart_port *port)
{
	struct imc_idi_btif_port *p_btif = to_imc_idi_btif_port(port);

	IMC_IDI_BTIF_ENTER;

	if (p_btif->state & BT_TX)
		btif_exit_state(p_btif, BT_TX);

	IMC_IDI_BTIF_EXIT;
}

/*
 * imc_idi_tx: Queue xmit data in IDI queue.
 *
 */
static int imc_idi_tx(struct imc_idi_btif_port *p_btif)
{
	struct uart_port *port = &p_btif->port;
	int ret;
#ifdef IMC_IDI_BTIF_PARANOID
	struct device *dev = &p_btif->p_dev->device;
#endif
	struct idi_peripheral_device *p_device = p_btif->p_dev;
	struct idi_transaction *trans = p_btif->tx_trans;
	struct idi_xfer *xfer = &trans->idi_xfer;
	IMC_IDI_BTIF_ENTER;

#define MAX_BTIF_TX_SIZE 0x8000
	ret = idi_tty_prepare_tx_xfer(p_device, trans, port,
				      (unsigned *)p_btif->tx_bounce,
				      TX_BUFFER_SIZE, MAX_BTIF_TX_SIZE);

	if (ret == 0)
		return 0;

#ifdef IMC_IDI_BTIF_PARANOID
	if (p_btif->temp_iio_chan) {
#endif
		p_btif->tx_ongoing = true;
		if (unlikely(!timer_pending(&temp_meas_timer))) {
			queue_work(p_btif->temp_meas_wq,
						&p_btif->temp_meas_work);
			mod_timer(&temp_meas_timer, jiffies + msecs_to_jiffies(
					IMC_IDI_BTIF_TEMP_MEAS_INTERVAL_MSECS));
			p_btif->tx_ongoing = false;
		}
#ifdef IMC_IDI_BTIF_PARANOID
	}
#endif

	ret = idi_async_write(p_device, trans);

#ifdef IMC_IDI_BTIF_PARANOID
	if (unlikely(ret)) {
		dev_err(dev, "idi_async_write returns with error!\n");
		return -EINVAL;
	}
#endif

	IMC_IDI_BTIF_EXIT;
	return xfer->size;
}

static void btif_tx_transaction_complete(struct idi_transaction *trans)
{
	struct imc_idi_btif_port *p_btif =
	    dev_get_drvdata(&trans->peripheral->device);
	struct uart_port *port = &p_btif->port;
	struct device *dev = &p_btif->p_dev->device;
	struct idi_xfer *xfer = &trans->idi_xfer;
	unsigned long flags;
	int ret;

	IMC_IDI_BTIF_ENTER;
	idi_tty_complete_tx_xfer(p_btif->p_dev, trans, port);

	dump_xfer(p_btif, " ---> ", xfer);

	dev_dbg(dev, " --->[TTY]: %d bytes sent\n", xfer->size);

	spin_lock_irqsave(&p_btif->txstate_lock, flags);
	/* Check if TX has been stopped before continuing further */
	if (p_btif->state & BT_TX) {
		if (uart_circ_empty(&port->state->xmit)) {
			dev_dbg(dev, "Will stop TX port\n");
			btif_stop_tx(port);
			spin_unlock_irqrestore(&p_btif->txstate_lock, flags);
			return;
		}
		spin_unlock_irqrestore(&p_btif->txstate_lock, flags);
		/* Since we are in btif_tx_transaction_complete, some bytes
		 * MUST have been pushed. Call imc_idi_tx again. */
		spin_lock_irqsave(&port->lock, flags);
		ret = imc_idi_tx(p_btif);
		spin_unlock_irqrestore(&port->lock, flags);
	} else {
		spin_unlock_irqrestore(&p_btif->txstate_lock, flags);
		return;
	}

	/* Handle cases here for which btif_tx_transaction_complete
	 * will NOT be called again */
	if (ret == 0) {
		if (p_btif->state & BT_TX) {
			dev_dbg(dev, "Will stop TX port\n");
			btif_stop_tx(port);
		}
#ifdef IMC_IDI_BTIF_PARANOID
	} else if (unlikely(ret < 0)) {
		dev_err(dev, "Error %x while btif TX queuing attempts\n", ret);
		/* FIXME: Recover */
#endif
	}

	IMC_IDI_BTIF_EXIT;
}

static void btif_init_hw(struct imc_idi_btif_port *p_btif)
{
	void __iomem *base = p_btif->ctrl_io;
	unsigned reg;

	/* Set block in Configuration mode */
	iowrite32(BTIF_CONFIG_MODE, BTIF_CLC(base));

	/* Cleaning interrupts */
	iowrite32(0, BTIF_IMSC(base));
	ioread32(BTIF_IMSC(base));	/* Dummy read */

	/* Clear pending interrupt */
	iowrite32((-1), BTIF_ICR(base));
	ioread32(BTIF_RIS(base));	/* Dummy read */

	/* Set TopSpin FIFO configuration */

	/*
	 * We can not use BTIF_FIFO_CFG_RXBS_RXBS4/BTIF_FIFO_CFG_TXBS_TXBS4
	 * SMS02671092:
	 * Behavior: IDI requires a multiple of 4 Words as transfer length,
	 * otherwise the IDI address counter will not wrap.
	 * This cannot be guaranteed, if there are undefined number of SREQ
	 * in combination of BREQ.
	 * Impact:
	 *   IDI would generate wrong address for RXD access causing a HW error!
	 *   Workaround: limit to SREQ only (i. e. burst size of a single word).
	 *   Note: as SREQ are only issued in none-flow-controller mode, this
	 *   restriction would not apply for a configuration of the BTIF
	 * in flow controller mode.
	 *
	 */
	reg = BTIF_FIFO_CFG_TXFA_TXFA1
	    | BTIF_FIFO_CFG_TXBS_TXBS1
	    | BTIF_FIFO_CFG_TXFC_TXFC
	    | BTIF_FIFO_CFG_TX_SWAP_NO_TXSWAP
	    | BTIF_FIFO_CFG_RXFA_RXFA1
	    | BTIF_FIFO_CFG_RXBS_RXBS1
	    | BTIF_FIFO_CFG_RXFC_RXNFC | BTIF_FIFO_CFG_RX_SWAP_NO_RXSWAP;
	iowrite32(reg, BTIF_FIFO_CFG(base));

	/* Enable DMA requests */
	reg = BTIF_DMAE_TX_LSREQ_MASK
	    | BTIF_DMAE_TX_SREQ_MASK
	    | BTIF_DMAE_TX_LBREQ_MASK
	    | BTIF_DMAE_TX_BREQ_MASK
	    | BTIF_DMAE_RX_SREQ_MASK | BTIF_DMAE_RX_BREQ_MASK;
	iowrite32(reg, BTIF_DMAE(base));

	/* Set RX TimeOut for end of packet generation */
	iowrite32(BTIF_RX_TMO, BTIF_TMO_CFG(base));

	/* Enable interrupt at block level */
	reg = BTIF_RIS_RX_SREQ_MASK
	    | BTIF_RIS_RX_BREQ_MASK
	    | BTIF_RIS_TX_LSREQ_MASK
	    | BTIF_RIS_TX_SREQ_MASK
	    | BTIF_RIS_TX_LBREQ_MASK
	    | BTIF_RIS_TX_BREQ_MASK
	    | BTIF_RIS_RX_FUFL_MASK
	    | BTIF_RIS_TX_FOFL_MASK
	    | BTIF_RIS_RX_MRPSSTOP_MASK
	    | BTIF_RIS_RX_TMO_MASK | BTIF_RIS_RX_EOP_MASK | BTIF_RIS_FCI_MASK;

	iowrite32(reg, BTIF_IMSC(base));
	ioread32(BTIF_IMSC(base));	/* Dummy read */

	/* Enable AutoReceive */
	reg = BTIF_FIFO_CTRL_RX_START_MASK | BTIF_FIFO_CTRL_RX_AR_AR_ON;

	iowrite32(reg, BTIF_FIFO_CTRL(base));

	/* Configuration completed, back to run mode */
	iowrite32(BTIF_RUN_MODE, BTIF_CLC(base));

#define BTIF_MAX_PACKET_SIZE 1024

	/* Set MRPS to 1k */
	iowrite32(BTIF_MAX_PACKET_SIZE, BTIF_MRPS_CTRL(p_btif->ctrl_io));
}

static void btif_get_module_info(struct imc_idi_btif_port *p_btif)
{
	unsigned reg;

	reg = ioread32(BTIF_ID(p_btif->ctrl_io));
	dev_info(&p_btif->p_dev->device,
		 "Revision Id: %#x, Module Id: %#x, TopSpin Rev: %#x\n",
		 BTIF_ID_REV_NUMBER(reg), BTIF_ID_MOD_ID(reg),
		 BTIF_ID_TS_REV_NR(reg));
	p_btif->id = reg;

	reg = ioread32(BTIF_SRB_MSCONF(p_btif->ctrl_io));
	p_btif->srb_msconf = reg;
	reg = ioread32(BTIF_SRB_ERRCONF(p_btif->ctrl_io));
	p_btif->err_msconf = reg;

	reg = ioread32(BTIF_FIFO_ID(p_btif->ctrl_io));
	p_btif->rps_stage = (1 << BTIF_FIFO_ID_RPS_STAGE(reg));
	/* A stage is 4 bytes */
	p_btif->rx_fifosize = (1 << BTIF_FIFO_ID_RX_STAGE(reg)) * 4;
	p_btif->tx_fifosize = (1 << BTIF_FIFO_ID_TX_STAGE(reg)) * 4;
}

static int btif_prepare_tx(struct imc_idi_btif_port *p_btif)
{
	struct idi_channel_config *conf = &p_btif->tx_ch_config;
	struct idi_peripheral_device *p_device = p_btif->p_dev;
	struct idi_resource *idi_res = &p_device->resources;
	struct resource *res;
	struct idi_transaction *trans;
	struct idi_xfer *xfer;
	int ret = 0;

	IMC_IDI_BTIF_ENTER;
	trans = idi_alloc_transaction(GFP_KERNEL);

#ifdef IMC_IDI_BTIF_PARANOID
	if (!trans)
		return -ENOMEM;
#endif

	res =
	    idi_get_resource_byname(idi_res, IORESOURCE_MEM, BTIF_TX_RES_NAME);
#ifdef IMC_IDI_BTIF_PARANOID
	if (!res) {
		dev_err(&p_btif->p_dev->device,
			"idi_get_resource_byname - failed\n");
		return -EINVAL;
	}
#endif

	conf->tx_or_rx = 1;
	conf->priority = IDI_NORMAL_PRIORITY;
	conf->channel_opts = IDI_PRIMARY_CHANNEL | IDI_TX_CHANNEL;
	conf->size = 0;
	conf->cpu_base = NULL;
	conf->base = 0;
	conf->dst_addr = res->start;
	conf->hw_fifo_size = resource_size(res);
	conf->start_tx = btif_idi_start_tx;
	conf->end_of_packet = NULL;

	xfer = &trans->idi_xfer;
	xfer->base = conf->base;
	xfer->cpu_base = 0;
	xfer->size = 0;
	xfer->dst_addr = conf->dst_addr;
	xfer->channel_opts = IDI_PRIMARY_CHANNEL | IDI_TX_CHANNEL;

	trans->complete = btif_tx_transaction_complete;

	p_btif->tx_trans = trans;

	ret = idi_set_channel_config(p_device, conf);

#ifdef IMC_IDI_BTIF_PARANOID
	if (ret) {
		idi_free_transaction(trans);
		dev_err(&p_btif->p_dev->device,
			"Unable to set IDI write channel configuration\n");
	}
#endif

	IMC_IDI_BTIF_EXIT;

	return ret;
}

static int btif_prepare_rx(struct imc_idi_btif_port *p_btif)
{
	struct idi_peripheral_device *p_device = p_btif->p_dev;
	struct idi_resource *idi_res = &p_device->resources;
	struct resource *res;
	struct idi_channel_config *rx_conf = &p_btif->rx_ch_config;
#ifdef IMC_IDI_BTIF_PARANOID
	struct device *dev = &p_btif->p_dev->device;
#endif
	int ret = 0;

	IMC_IDI_BTIF_ENTER;

	res =
	    idi_get_resource_byname(idi_res, IORESOURCE_MEM, BTIF_RX_RES_NAME);
#ifdef IMC_IDI_BTIF_PARANOID
	if (!res) {
		dev_err(dev, "idi_get_resource_byname - failed\n");
		return -EINVAL;
	}
#endif

	rx_conf->tx_or_rx = 0;
	rx_conf->priority = IDI_NORMAL_PRIORITY;
	rx_conf->channel_opts = IDI_PRIMARY_CHANNEL;
	rx_conf->size = p_btif->rps_stage * BTIF_MAX_PACKET_SIZE;

	rx_conf->cpu_base = kmalloc(rx_conf->size, GFP_KERNEL | GFP_DMA);

#ifdef IMC_IDI_BTIF_PARANOID
	if (!rx_conf->cpu_base) {
		dev_err(dev, "Unable to allocate RX buffer\n");
		return -ENOMEM;
	}
#endif

	rx_conf->base = dma_map_single(NULL, rx_conf->cpu_base,
			rx_conf->size, DMA_FROM_DEVICE);

#ifdef IMC_IDI_BTIF_PARANOID
	if (!rx_conf->base) {
		dev_err(dev, "Unable to DMA-map RX buffer\n");
		kfree(rx_conf->cpu_base);
		rx_conf->cpu_base = NULL;
		return -ENOMEM;
	}
#endif

	rx_conf->hw_fifo_size = resource_size(res);

	rx_conf->dst_addr = res->start;

	ret = idi_set_channel_config(p_device, rx_conf);
#ifdef IMC_IDI_BTIF_PARANOID
	if (ret) {
		dev_err(dev, "Unable to set IDI read channel configuration\n");
		dma_unmap_single(NULL, rx_conf->base, rx_conf->size,
				DMA_FROM_DEVICE);
		kfree(rx_conf->cpu_base);
		rx_conf->cpu_base = NULL;
	}
#endif

	IMC_IDI_BTIF_EXIT;

	return ret;
}

static void imc_idi_btif_send_netlink_message(struct imc_idi_btif_port *p_btif,
					      enum netlink_message_code code)
{
	struct sk_buff *skb_out = NULL;
	struct nlmsghdr *nlh = NULL;
	size_t msglen = sizeof(enum netlink_message_code);
	int err;

#ifdef IMC_IDI_BTIF_PARANOID
	if ((!p_btif->nl_socket) || (!stack_pid))
		return;
#endif

	skb_out = nlmsg_new(msglen, GFP_ATOMIC);
#ifdef IMC_IDI_BTIF_PARANOID
	if (!skb_out) {
		dev_err(&p_btif->p_dev->device,
			"Could not allocate Netlink message\n");
		return;
	}
#endif

	nlh = nlmsg_put(skb_out, 0, 0, NLMSG_DONE, msglen, 0);
	NETLINK_CB(skb_out).dst_group = 0;

	memcpy(nlmsg_data(nlh), (void *)&code, msglen);

	err = nlmsg_unicast(p_btif->nl_socket, skb_out, stack_pid);
#ifdef IMC_IDI_BTIF_PARANOID
	if (err < 0)
		dev_err(&p_btif->p_dev->device,
			"Netlink message failed with %d\n", err);
#endif
}

static void imc_idi_btip_host_wakeup_clear_mask(
			struct imc_idi_btif_port *p_btif, bool clear, bool mask)
{
	unsigned reg;

	if (clear) { /* Clear */
		if (idi_client_iowrite(p_btif->p_dev,
				(unsigned)SCU_C3_IRQSC(p_btif->scu_io),
				SCU_C3_IRQSC_BT_WUP_CLEAR_INTERRUPT)) {
			dev_err(&p_btif->p_dev->device,
					"%s: idi write error %p\n",
					__func__, SCU_C3_IRQSC(p_btif->scu_io));
			return;
		}

		/* Dummy read */
		if (idi_client_ioread(p_btif->p_dev,
				(unsigned)SCU_RIS(p_btif->scu_io), &reg)) {
			dev_err(&p_btif->p_dev->device,
					"%s: idi read error %p\n",
					__func__, SCU_RIS(p_btif->scu_io));
			return;
		}
	}

	/* Mask/Unmask */
	if (idi_client_iowrite(p_btif->p_dev,
			(unsigned)SCU_C3_IRQSM(p_btif->scu_io),
			(mask) ? SCU_C3_IRQSM_BT_WUP_DISABLED :
			SCU_C3_IRQSM_BT_WUP_ENABLED)) {
		dev_err(&p_btif->p_dev->device,
				"%s: idi write error %p\n",
				__func__, SCU_C3_IRQSM(p_btif->scu_io));
		return;
	}

	/* Dummy read */
	if (idi_client_ioread(p_btif->p_dev,
			(unsigned)SCU_C3_IRQSM(p_btif->scu_io), &reg)) {
		dev_err(&p_btif->p_dev->device,
				"%s: idi read error %p\n",
				__func__, SCU_C3_IRQSM(p_btif->scu_io));
		return;
	}

	if (mask)
		return;

	/* Unmask the C3 at SCU */
	if (idi_client_ioread(p_btif->p_dev,
			(unsigned)SCU_IMSC(p_btif->scu_io), &reg)) {
		dev_err(&p_btif->p_dev->device,
				"%s: idi read error %p\n",
				__func__, SCU_IMSC(p_btif->scu_io));
		return;
	}

	if (idi_client_iowrite(p_btif->p_dev,
			(unsigned)SCU_IMSC(p_btif->scu_io),
			reg | SCU_IMSC_C3_ENABLED)) {
		dev_err(&p_btif->p_dev->device,
				"%s: idi write error %p\n",
				__func__, SCU_IMSC(p_btif->scu_io));
		return;
	}

	 /* Dummy read */
	if (idi_client_ioread(p_btif->p_dev,
			(unsigned)SCU_IMSC(p_btif->scu_io), &reg))
		dev_err(&p_btif->p_dev->device,
				"%s: idi read error %p\n",
				__func__, SCU_IMSC(p_btif->scu_io));
}

static irqreturn_t imc_idi_btif_src_isr_bh(int irq, void *dev_id)
{
	struct imc_idi_btif_port *p_btif = (struct imc_idi_btif_port *)dev_id;
	struct device *dev = &p_btif->p_dev->device;
	int handled = 0;
	unsigned mis = p_btif->rx_mis;
	unsigned mask = p_btif->imsc;
	bool cts;

	IMC_IDI_BTIF_ENTER;
	dev_dbg(dev, "%s: MIS %x\n", __func__, mis);

#ifdef IMC_IDI_BTIF_PARANOID
	if (unlikely(mis & BTIF_RIS_RX_TMO_MASK)) {
		dev_dbg(dev, "%s:TMO int detected mis = %#x\n", __func__, mis);
		handled = IRQ_HANDLED;
	}
#endif
	if (mis & (BTIF_RIS_RX_EOP_MASK | BTIF_RIS_RX_MRPSSTOP_MASK)) {
		dev_dbg(dev, "%s:%s interrupt detected\n", __func__,
				(mis & BTIF_RIS_RX_EOP_MASK) ? "EOP" : "MRPS");
		/* Check if RX has been stopped before processing packet */
		if (likely(p_btif->state & BT_RX))
			imc_idi_btif_read(p_btif);
		handled = IRQ_HANDLED;
	}

	if (mis & (BTIF_RIS_FCI_MASK | BTIF_RIS_FCI_N_MASK)) {
		mask &= ~(BTIF_RIS_FCI_MASK | BTIF_RIS_FCI_N_MASK);
		if (mis & BTIF_RIS_FCI_MASK) {
			dev_dbg(dev, "Got CTS high interrupt\n");
			mask |= BTIF_RIS_FCI_N_MASK;
			btif_enter_state(p_btif, BT_WAKEUP);
			cts = true;
			iowrite32(BTIF_RIS_FCI_N_MASK,
				  BTIF_ICR(p_btif->ctrl_io));
			if (!p_btif->signaling_disabled)
				imc_idi_btif_send_netlink_message(p_btif,
								  CTS_HIGH);
		} else {
			dev_dbg(dev, "Got CTS low interrupt\n");
			mask |= BTIF_RIS_FCI_MASK;
			btif_exit_state(p_btif, BT_WAKEUP);
			cts = false;
			iowrite32(BTIF_RIS_FCI_MASK, BTIF_ICR(p_btif->ctrl_io));
			if (!p_btif->signaling_disabled)
				imc_idi_btif_send_netlink_message(p_btif,
								  CTS_LOW);
		}
		ioread32(BTIF_RIS(p_btif->ctrl_io)); /* Dummy read */

		uart_handle_cts_change(&p_btif->port, cts);
		handled = IRQ_HANDLED;
	}

	btif_set_interrupt(p_btif, mask, 1);
	IMC_IDI_BTIF_EXIT;

	return IRQ_RETVAL(handled);
}

#ifdef IMC_IDI_BTIF_DMA_FLOWCONTROL
static irqreturn_t imc_idi_btif_err_isr(int irq, void *dev_id)
{
	struct imc_idi_btif_port *p_btif = (struct imc_idi_btif_port *)dev_id;
	void __iomem *ctrl = p_btif->ctrl_io;
	struct device *dev = &p_btif->p_dev->device;

	unsigned mis;
	IMC_IDI_BTIF_ENTER;

#define BTIF_HANDLE_ERR(_X_) { \
	if (mis & BTIF_RIS_##_X_##_MASK) { \
		iowrite32(BTIF_RIS_##_X_##_MASK, BTIF_ICR(ctrl)); \
		ioread32(BTIF_RIS(ctrl)); \
		dev_info(dev, "%s: "#_X_" detected\n", __func__); \
	} }

	while ((mis = (ioread32(BTIF_MIS(ctrl)) & p_btif->err_msconf))) {
		dev_err(dev, "%s: MIS %x\n", __func__, mis);
		BTIF_HANDLE_ERR(RX_FUFL);
		BTIF_HANDLE_ERR(TX_FOFL);
		return IRQ_HANDLED;
	}
	dev_err(dev, "%s: Dummy error interrupt detected !\n", __func__);
	IMC_IDI_BTIF_EXIT;
	return IRQ_NONE;
}
#endif

static irqreturn_t imc_idi_btip_host_wakeup_isr(int irq, void *dev_id)
{
	struct imc_idi_btif_port *p_btif = (struct imc_idi_btif_port *)dev_id;

	IMC_IDI_BTIF_ENTER;

	imc_idi_btip_host_wakeup_clear_mask(p_btif, true, true);

	IMC_IDI_BTIF_EXIT;
	return IRQ_WAKE_THREAD;
}

static irqreturn_t imc_idi_btip_host_wakeup_isr_bh(int irq, void *dev_id)
{
	struct imc_idi_btif_port *p_btif = (struct imc_idi_btif_port *)dev_id;
	struct device *dev = &p_btif->p_dev->device;
	unsigned reg;

	IMC_IDI_BTIF_ENTER;

	if (idi_client_ioread(p_btif->p_dev,
			(unsigned)SCU_BT_STAT(p_btif->scu_io), &reg)) {
		dev_err(&p_btif->p_dev->device,
				"%s: idi read error %p\n",
				__func__, SCU_BT_STAT(p_btif->scu_io));
	}

	if (SCU_BT_STAT_BT_WAKEUP_HOST(reg)) {
		dev_dbg(dev, "Got host wakeup high interrupt\n");

		if (unlikely(p_btif->first_wakeup)) {
			btif_get_module_info(p_btif);
			btif_prepare_rx(p_btif);
			btif_prepare_tx(p_btif);
			btif_init_hw(p_btif);
			p_btif->first_wakeup = 0;
		}

#ifdef CONFIG_HAS_WAKELOCK
		if (!wake_lock_active(&p_btif->btif_wake_lock))
			wake_lock(&p_btif->btif_wake_lock);
#endif

		if (likely(!p_btif->signaling_disabled))
			imc_idi_btif_send_netlink_message(p_btif, HWUP_HIGH);
	} else {
		dev_dbg(dev, "Got host wakeup low interrupt\n");
		if (likely(!p_btif->signaling_disabled))
			imc_idi_btif_send_netlink_message(p_btif, HWUP_LOW);
	}

	imc_idi_btip_host_wakeup_clear_mask(p_btif, false, false);

	IMC_IDI_BTIF_EXIT;

	return IRQ_RETVAL(IRQ_HANDLED);
}

static void __iomem *imc_idi_request_io_byname(
		struct idi_peripheral_device *p_device,
		const char *name,
		bool request, bool remap)
{

#ifndef CONFIG_NKERNEL
	void __iomem *io;
#endif
	struct resource *res;
	struct idi_resource *idi_res = &p_device->resources;

#ifdef IMC_IDI_BTIF_PARANOID
	if (!name)
		return NULL;
#endif

	res = idi_get_resource_byname(idi_res, IORESOURCE_MEM, name);
#ifdef IMC_IDI_BTIF_PARANOID
	if (!res)
		return NULL;
#endif

#ifndef CONFIG_NKERNEL
	if (request &&
	    (!request_mem_region(res->start, resource_size(res),
				 dev_name(&p_device->device))))
		return NULL;

	if (remap)
		io = ioremap(res->start, resource_size(res));
	else
		io = (void __iomem *)res->start;

	if (!io && request)
		release_mem_region(res->start, resource_size(res));

	return io;
#else
	return (void __iomem *)res->start;
#endif
}

static void imc_idi_release_io_byname(struct idi_peripheral_device *p_device,
				      const char *name,
				      void __iomem *io,
				      bool release)
{
#ifndef CONFIG_NKERNEL
	struct resource *res;
	struct idi_resource *idi_res = &p_device->resources;

#ifdef IMC_IDI_BTIF_PARANOID
	if (!name)
		return;
#endif

	res = idi_get_resource_byname(idi_res, IORESOURCE_MEM, name);

#ifdef IMC_IDI_BTIF_PARANOID
	if (!res)
		return;
#endif

	iounmap(io);
	if (release)
		release_mem_region(res->start, resource_size(res));
#endif
}

static const struct imc_idi_irq_desc {
	const char *name;
	irq_handler_t handler;
	irq_handler_t handler_bh;
	unsigned long flags;
} irq_desc[] = {
	{
#ifdef IMC_IDI_BTIF_DMA_FLOWCONTROL
		.name = BTIF_IRQ_NAME,
		.handler = imc_idi_btif_isr,
		.handler_bh = NULL,
		.flags = IRQF_SHARED
	}, {
		.name = BTIF_IRQ_ERR_NAME,
		.handler = imc_idi_btif_err_isr,
		.handler_bh = NULL,
		.flags = IRQF_SHARED
	}, {
#endif
		.name = BTIF_IRQ_SRC_NAME,
		.handler = imc_idi_btif_src_isr,
		.handler_bh = imc_idi_btif_src_isr_bh,
		.flags = IRQF_SHARED
	}, {
		.name = BTIP_IRQ_WAKEUP_NAME,
		.handler = imc_idi_btip_host_wakeup_isr,
		.handler_bh = imc_idi_btip_host_wakeup_isr_bh,
		.flags = IRQF_SHARED | IRQF_NO_SUSPEND
	},
};

static int imc_idi_request_irqs(struct idi_peripheral_device *p_device)
{
	int ret = 0;
	int i = 0;
	struct idi_resource *idi_res = &p_device->resources;
	struct imc_idi_btif_port *p_btif = dev_get_drvdata(&p_device->device);

	for (i = 0; i < ARRAY_SIZE(irq_desc); i++) {
		struct resource *res;
		const char *name = irq_desc[i].name;
		irq_handler_t handler = irq_desc[i].handler;
		irq_handler_t handler_bh = irq_desc[i].handler_bh;
		res = idi_get_resource_byname(idi_res, IORESOURCE_IRQ, name);
#ifdef IMC_IDI_BTIF_PARANOID
		if (!res) {
			ret = -EINVAL;
			goto fail_request_irq;
		}
#endif

		ret = request_threaded_irq(res->start, handler, handler_bh,
					irq_desc[i].flags, name, p_btif);
#ifdef IMC_IDI_BTIF_PARANOID
		if (ret) {
			dev_err(&p_device->device,
				"Error while requesting %d irq node\n",
				res->start);
			ret = -EINVAL;
			goto fail_request_irq;
		}
#endif
	}

	return 0;

#ifdef IMC_IDI_BTIF_PARANOID
fail_request_irq:
	for (i--; i > (-1); i--) {
		struct resource *res;
		const char *name = irq_desc[i].name;

		res = idi_get_resource_byname(idi_res, IORESOURCE_IRQ, name);

		/* This should not happen */
		if (!res)
			BUG();

		free_irq(res->start, p_btif);
	}
	return ret;
#endif

}

static void imc_idi_free_irqs(struct idi_peripheral_device *p_device)
{
	int i;
	struct idi_resource *idi_res = &p_device->resources;
	struct resource *res;
	struct imc_idi_btif_port *p_btif = dev_get_drvdata(&p_device->device);

	for (i = 0; i < ARRAY_SIZE(irq_desc); i++) {
		const char *name = irq_desc[i].name;

		res = idi_get_resource_byname(idi_res, IORESOURCE_IRQ, name);
#ifdef IMC_IDI_BTIF_PARANOID
		if (res)
#endif
			free_irq(res->start, p_btif);
	}
}

static void btif_start_tx(struct uart_port *port)
{
	struct imc_idi_btif_port *p_btif = to_imc_idi_btif_port(port);
	struct device *dev = &p_btif->p_dev->device;
	unsigned long flags;
	int ret;

	IMC_IDI_BTIF_ENTER;
/*
	if(p_btif->state & BT_TX)
		return;
*/
	spin_lock_irqsave(&p_btif->txstate_lock, flags);
	if ((!(p_btif->state & BT_TX) &&
			!(uart_circ_empty(&port->state->xmit)))) {
		btif_enter_state(p_btif, BT_TX);
		spin_unlock_irqrestore(&p_btif->txstate_lock, flags);
		ret = imc_idi_tx(p_btif);

		if (ret == 0) {
			if (p_btif->state & BT_TX) {
				dev_dbg(dev, "Will stop TX port\n");
				btif_stop_tx(&p_btif->port);
			}
#ifdef IMC_IDI_BTIF_PARANOID
		} else if (ret < 0) {
			dev_err(dev,
				"Error %x while btif TX queuing attempts\n",
				ret);
			/* FIXME: Recover */
#endif
		}
	} else
		spin_unlock_irqrestore(&p_btif->txstate_lock, flags);

	IMC_IDI_BTIF_EXIT;
}

static void btif_stop_rx(struct uart_port *port)
{
	struct imc_idi_btif_port *p_btif = to_imc_idi_btif_port(port);

	IMC_IDI_BTIF_ENTER;

	if (p_btif->state & BT_RX)
		btif_exit_state(p_btif, BT_RX);

	IMC_IDI_BTIF_EXIT;
}

static unsigned int btif_tx_empty(struct uart_port *port)
{
	struct imc_idi_btif_port *p_btif = to_imc_idi_btif_port(port);
	void __iomem *ctrl = p_btif->ctrl_io;

	if (BTIF_FIFO_STAT_TXFFS(ioread32(BTIF_FIFO_STAT(ctrl))) == 0)
		return TIOCSER_TEMT;
	else
		return 0;
}

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT

void imc_idi_set_btip_clk(struct imc_idi_btif_platdata *platdata, bool enable)
{
	if (enable) {
		_imc_btip_spcu_req(platdata, 1);

		if (platdata->bt_rtc)
			clk_prepare_enable(platdata->bt_rtc);

		if (platdata->bt_78M)
			clk_prepare_enable(platdata->bt_78M);

		if (platdata->bt_104M)
			clk_prepare_enable(platdata->bt_104M);
	} else {
		if ((platdata->bt_78M)
				&& (__clk_is_enabled(platdata->bt_78M)))
			clk_disable_unprepare(platdata->bt_78M);

		if ((platdata->bt_104M)
				&& (__clk_is_enabled(platdata->bt_104M)))
			clk_disable_unprepare(platdata->bt_104M);

		if ((platdata->bt_rtc)
				&& (__clk_is_enabled(platdata->bt_rtc)))
			clk_disable_unprepare(platdata->bt_rtc);

		_imc_btip_spcu_req(platdata, 0);
	}
}

void imc_idi_set_btif_clk(struct imc_idi_btif_platdata *platdata, bool enable)
{
	if (enable) {
		if (platdata->kernel)
			clk_prepare_enable(platdata->kernel);
	} else {
		if ((platdata->kernel)
				&& (__clk_is_enabled(platdata->kernel)))
			clk_disable_unprepare(platdata->kernel);
	}
}

#define BTIF_PM_STATE_D0 "enable"
#define BTIF_PM_STATE_D0I2 "enable_psv"
#define BTIF_PM_STATE_D0I3 "idle"
#define BTIF_PM_STATE_D3 "disable"
static int btif_set_pm_state(struct device *_dev,
		struct device_state_pm_state *state)
{
	struct imc_idi_btif_port *p_btif = dev_get_drvdata(_dev);
	struct imc_idi_btif_platdata *platdata =
				imc_idi_btif_get_platdata(p_btif);

	if (p_btif->current_pm_state == state)
		return 0;

	if (!strcmp(state->name, BTIF_PM_STATE_D0)) {

		if ((!p_btif->current_pm_state)
				|| (!strcmp(p_btif->current_pm_state->name,
						BTIF_PM_STATE_D3))) {
			/* Transition is from D3 */
			imc_idi_set_btif_clk(platdata, true);

			imc_idi_set_btip_clk(platdata, true);

			if (platdata->btif_rst)
				reset_control_reset(platdata->btif_rst);

			if (platdata->btip_rst)
				reset_control_reset(platdata->btip_rst);
		} else if (!strcmp(p_btif->current_pm_state->name,
				BTIF_PM_STATE_D0I2)) {
			/* Transition is from D0I2 */
			/* Placeholder */
		} else {
			/* Transition is from D0I3 */
			imc_idi_set_btif_clk(platdata, true);

			if (platdata->btif_rst)
				reset_control_deassert(platdata->btif_rst);
		}

	} else if (!strcmp(state->name, BTIF_PM_STATE_D0I2)) {

		/* Place-holder */

	} else if (!strcmp(state->name, BTIF_PM_STATE_D0I3)) {

			imc_idi_set_btif_clk(platdata, false);

			if (platdata->btif_rst)
				reset_control_assert(platdata->btif_rst);

	} else if (!strcmp(state->name, BTIF_PM_STATE_D3)) {

		imc_idi_set_btip_clk(platdata, false);

		if (platdata->btip_rst)
			reset_control_assert(platdata->btip_rst);

		if ((!strcmp(p_btif->current_pm_state->name, BTIF_PM_STATE_D0))
				|| (!strcmp(p_btif->current_pm_state->name,
						BTIF_PM_STATE_D0I2))) {
			/* Transition is from D0 or D0I2 */
			imc_idi_set_btif_clk(platdata, false);

			if (platdata->btif_rst)
				reset_control_assert(platdata->btif_rst);
		}

	} else
		return -EINVAL;

	p_btif->current_pm_state = state;
	return 0;
}

static struct device_state_pm_state *btif_get_initial_pm_state(
		struct device *_dev)
{

	return device_state_pm_get_state_handler(_dev, BTIF_PM_STATE_D3);
}

static struct device_state_pm_ops btif_pm_ops = {
	.set_state = btif_set_pm_state,
	.get_initial_state = btif_get_initial_pm_state,
};

/* IDI PM states & class */

static struct device_state_pm_state btif_pm_states[] = {
	{ .name = BTIF_PM_STATE_D0 },
	{ .name = BTIF_PM_STATE_D0I2 },
	{ .name = BTIF_PM_STATE_D0I3 },
	{ .name = BTIF_PM_STATE_D3 },
};


DECLARE_DEVICE_STATE_PM_CLASS(btif);
#endif

static void btif_update_temperature(struct work_struct *work)
{
	struct imc_idi_btif_port *p_btif = container_of(work,
			struct imc_idi_btif_port, temp_meas_work);
	struct device *dev = &p_btif->p_dev->device;
	int temp_val, ret;

	IMC_IDI_BTIF_ENTER;

	ret = iio_read_channel_processed(p_btif->temp_iio_chan, &temp_val);
#ifdef IMC_IDI_BTIF_PARANOID
	if (ret < 0) {
		dev_err(dev, "PMICTEMP_SENSOR read returned error %d\n", ret);
		return;
	}
#endif

	dev_dbg(dev, "PMICTEMP_SENSOR = %d\n", temp_val);

	/* Temp. conversion formula as per AGOLD620 AS HD */
	temp_val = -((temp_val - IMC_IDI_BTIF_TEMP_MEAS_TREF)
					* IMC_IDI_BTIF_TEMP_MEAS_KT);

	/* Multiplier to avoid fractional arithmatic */
	/* Divided by 16 in FW */
	temp_val *= 16;

	/* Set for round off */
	if (temp_val >= 0)
		temp_val += (5 * IMC_IDI_BTIF_TEMP_MEAS_KT_SCALE / 10);
	else
		temp_val -= (5 * IMC_IDI_BTIF_TEMP_MEAS_KT_SCALE / 10);

	/* Scaled up and then down to avoid floating point operation */
	temp_val /= IMC_IDI_BTIF_TEMP_MEAS_KT_SCALE;

	if (idi_client_iowrite(p_btif->p_dev,
				(unsigned)SCU_BT_TEMPMEAS(p_btif->scu_io),
				(signed short)temp_val))
		dev_err(&p_btif->p_dev->device,
				"%s: idi write error %p\n",
				__func__, SCU_BT_TEMPMEAS(p_btif->scu_io));
}

static void btif_temp_meas_timer_expired(unsigned long data)
{
	struct imc_idi_btif_port *p_btif = (struct imc_idi_btif_port *) data;
	if (p_btif->dstate == D3)
		return;
	if (p_btif->dstate == D0 || p_btif->dstate == D0I2 ||
					g_bt_audio_state == btauif_aud_on)
		queue_work(p_btif->temp_meas_wq, &p_btif->temp_meas_work);
	if (p_btif->tx_ongoing || (g_bt_audio_state == btauif_aud_on)) {
		mod_timer(&temp_meas_timer, jiffies + msecs_to_jiffies(
					IMC_IDI_BTIF_TEMP_MEAS_INTERVAL_MSECS));
		p_btif->tx_ongoing = false;
	}

	IMC_IDI_BTIF_EXIT;
}

static void imc_idi_btif_peripheral_flush(struct imc_idi_btif_port *p_btif)
{
	struct idi_peripheral_device *p_device = p_btif->p_dev;

	if (p_btif->dstate != D0I3) {
		/* Disable BTIF module */
		iowrite32(BTIF_DISABLE, BTIF_CLC(p_btif->ctrl_io));
	}

	/* Release IDI resources */
	idi_peripheral_flush(p_device, p_btif->rx_ch_config.channel_opts);
	idi_peripheral_flush(p_device, p_btif->tx_ch_config.channel_opts);
}

static struct imc_idi_btauif_ops btauif_ops = {
	.enable = imc_idi_btauif_audio_enable,
	.enable_ex = imc_idi_btauif_audio_enable_ex,
	.disable = imc_idi_btauif_audio_disable,
};

static int btif_startup(struct uart_port *port)
{
	int ret = 0;
	struct imc_idi_btif_port *p_btif = to_imc_idi_btif_port(port);
	struct idi_peripheral_device *p_device = p_btif->p_dev;
#ifdef IMC_IDI_BTIF_PARANOID
	struct idi_channel_config *rx_conf = &p_btif->rx_ch_config;
	struct device *dev = &p_btif->p_dev->device;
#endif

	IMC_IDI_BTIF_ENTER;

#ifdef IMC_IDI_BTIF_PARANOID
	if (p_btif->state & BT_ON)
		return -EBUSY;
#endif

	INIT_LIST_HEAD(&p_btif->rxbuff_busy_queue);

	imc_idi_btip_host_wakeup_clear_mask(p_btif, true, false);

	ret = imc_idi_request_irqs(p_device);
#ifdef IMC_IDI_BTIF_PARANOID
	if (ret)
		goto fail_request_irqs;
#endif

	/* Avoid spurious CTS high interrupt on BTIP RST release */
	imc_idi_btif_wakeup_bt(p_btif, true);

	imc_btip_configure_fw(p_btif, false);
	imc_idi_btip_enable(p_btif);

	p_btif->dstate = D3;

	/* Get the ADC channel for temperature sensor */
	/* FIXME: Get "pmic_temp" channel */
	p_btif->temp_iio_chan = iio_channel_get(NULL, "PMICTEMP_SENSOR");

#ifdef IMC_IDI_BTIF_PARANOID
	if (IS_ERR(p_btif->temp_iio_chan)) {
		dev_err(dev, "Error getting PMICTEMP_SENSOR channel\n");
		p_btif->temp_iio_chan = NULL;
		memset(&temp_meas_timer, 0, sizeof(struct timer_list));
	} else {
#endif
		p_btif->temp_meas_wq = alloc_workqueue("btif_temp_meas",
						WQ_UNBOUND | WQ_HIGHPRI, 1);
		INIT_WORK(&p_btif->temp_meas_work, btif_update_temperature);
		setup_timer(&temp_meas_timer, btif_temp_meas_timer_expired,
						(unsigned long) p_btif);
#ifdef IMC_IDI_BTIF_PARANOID
	}
#endif

	imc_idi_bt_sco_register(&btauif_ops);

	IMC_IDI_BTIF_EXIT;

	return 0;

#ifdef IMC_IDI_BTIF_PARANOID
fail_request_irqs:
	dma_unmap_single(NULL, rx_conf->base, rx_conf->size, DMA_FROM_DEVICE);
	kfree(rx_conf->cpu_base);
	rx_conf->cpu_base = NULL;
	return ret;
#endif
}				/* btif_startup */

static void btif_shutdown(struct uart_port *port)
{
	struct imc_idi_btif_port *p_btif = to_imc_idi_btif_port(port);
	struct idi_peripheral_device *p_device = p_btif->p_dev;
	struct idi_channel_config *rx_conf = &p_btif->rx_ch_config;

	IMC_IDI_BTIF_ENTER;

#ifdef IMC_IDI_BTIF_PARANOID
	if (p_btif->state & BT_OFF)
		return;
#endif

	btif_stop_tx(&p_btif->port);
	btif_stop_rx(&p_btif->port);

	imc_idi_btip_host_wakeup_clear_mask(p_btif, true, true);

	/* Cleanup, in case stack has not gracefully closed */
	if (p_btif->dstate != D3) {
		imc_idi_btif_wakeup_bt(p_btif, false);
		imc_idi_btif_set_rts(p_btif, false);
		imc_idi_btif_peripheral_flush(p_btif);
		idi_set_power_state(p_device,
				(void *) btif_states.d3_handler, false);
		p_btif->dstate = D3;
	}

#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(&p_btif->btif_wake_lock))
		wake_unlock(&p_btif->btif_wake_lock);

	if (wake_lock_active(&btauif_wake_lock))
		wake_unlock(&btauif_wake_lock);
#endif

	imc_idi_btip_disable(p_btif);
	imc_idi_free_irqs(p_device);

	if (p_btif->tx_trans) {
		idi_free_transaction(p_btif->tx_trans);
		p_btif->tx_trans = NULL;
	}

	/* Free the RX bounce buffer */
	if (rx_conf->cpu_base) {
		dma_unmap_single(NULL, rx_conf->base,
				rx_conf->size, DMA_FROM_DEVICE);
		kfree(rx_conf->cpu_base);
		rx_conf->cpu_base = NULL;
	}

	stack_pid = 0;

#ifdef IMC_IDI_BTIF_PARANOID
	if (p_btif->temp_iio_chan) {
#endif
		del_timer_sync(&temp_meas_timer);
		flush_workqueue(p_btif->temp_meas_wq);
		destroy_workqueue(p_btif->temp_meas_wq);

		/* Release the ADC channel for temperature sensor */
		iio_channel_release(p_btif->temp_iio_chan);
		p_btif->temp_iio_chan = NULL;
#ifdef IMC_IDI_BTIF_PARANOID
	}
#endif

	imc_idi_bt_sco_unregister(&btauif_ops);

#ifdef CONFIG_AG6XX
	pr_debug("Changing BT Controller Power State to OFF\n");
	btif_ioctl(port, IMC_IDI_BT_SET_POWER_STATE, IMC_IDI_BT_POWER_STATE_OFF_D3);
#endif

	IMC_IDI_BTIF_EXIT;

}				/* btif_shutdown */

static void btif_set_termios(struct uart_port *port, struct ktermios *termios,
			     struct ktermios *old)
{
}

static unsigned int btif_get_mctrl(struct uart_port *port)
{
	unsigned int ret = 0;
	struct imc_idi_btif_port *p_btif = to_imc_idi_btif_port(port);
	struct device *dev = &p_btif->p_dev->device;

	/* FIXME: This can get called after requesting RTS but
	   before receiving CTS from BT. This works around it
	   by making BT appear always ready to the tty. */
	ret = TIOCM_CTS;

	ret |= (TIOCM_DSR | TIOCM_CAR);

	dev_dbg(dev, "%s: mctrl %#x\n", __func__, ret);
	return ret;
}

static void btif_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static const char *btif_type(struct uart_port *port)
{
#ifdef IMC_IDI_BTIF_PARANOID
	return port->type == PORT_BTIF ? "XGOLD/BTIF" : NULL;
#else
	return "XGOLD/BTIF";
#endif
}

static void btif_release_port(struct uart_port *port)
{
#ifndef CONFIG_NKERNEL
	struct imc_idi_btif_port *p_btif = to_imc_idi_btif_port(port);
	struct idi_peripheral_device *p_device = p_btif->p_dev;
	struct resource *res;

	res = idi_get_resource_byname(&p_device->resources,
				      IORESOURCE_MEM, BTIF_REG_RES_NAME);
#ifdef IMC_IDI_BTIF_PARANOID
	if (!res)
		return;
#endif

	release_mem_region(res->start, resource_size(res));
#endif
}

#ifdef IMC_IDI_BTIF_PARANOID
static int btif_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_BTIF)
		return -EINVAL;

	return 0;
}
#endif

static int btif_request_port(struct uart_port *port)
{
	struct imc_idi_btif_port *p_btif = to_imc_idi_btif_port(port);
	struct idi_peripheral_device *p_device = p_btif->p_dev;
	struct resource *res;

	res = idi_get_resource_byname(&p_device->resources,
				      IORESOURCE_MEM, BTIF_REG_RES_NAME);
#ifdef IMC_IDI_BTIF_PARANOID
	if (!res)
		return -ENODEV;
#endif

#ifndef CONFIG_NKERNEL
	if (!request_mem_region(res->start, resource_size(res),
				dev_name(&p_device->device)))
		return -EBUSY;
#endif

	return 0;
}

static void btif_config_port(struct uart_port *port, int flags)
{
#ifdef IMC_IDI_BTIF_PARANOID
	if (flags & UART_CONFIG_TYPE) {
#endif
		port->type = PORT_BTIF;
		btif_request_port(port);
#ifdef IMC_IDI_BTIF_PARANOID
	}
#endif
}

static int btif_ioctl(struct uart_port *port, unsigned int cmd,
		      unsigned long arg)
{
	struct imc_idi_btif_port *p_btif = to_imc_idi_btif_port(port);
	struct imc_idi_btif_platdata *platdata =
			imc_idi_btif_get_platdata(p_btif);
	struct idi_peripheral_device *p_device = p_btif->p_dev;
	struct device *dev = &p_btif->p_dev->device;
	unsigned reg;
	int ret = 0;

	IMC_IDI_BTIF_ENTER;

	switch (cmd) {
	case IMC_IDI_BT_SET_POWER_STATE:
#ifdef IMC_IDI_BTIF_PARANOID
		if (unlikely(p_btif->signaling_disabled &&
					((arg == D0I2) || (arg == D0I3)))) {
			dev_err(dev,
				"BT signaling disabled, cannot change power state\n");
			ret = -EINVAL;
		} else if (p_btif->dstate != arg) {
#endif
			dev_dbg(dev, "Setting BT power state from %u to %lu\n",
				p_btif->dstate, arg);

			switch (arg) {
			case D0:
				ret = idi_set_power_state(p_device,
						(void *)btif_states.d0_handler
							, true);
				break;
			case D0I2:
				ret = idi_set_power_state(p_device,
						(void *)btif_states.d0i2_handler
							, true);
				break;
			case D0I3:
				ret = idi_client_ioread(p_btif->p_dev,
					(unsigned)SCU_BT_STAT(p_btif->scu_io),
					&reg);

				if (ret) {
					dev_err(&p_btif->p_dev->device,
						"%s: idi read error %p\n",
						__func__,
						SCU_BT_STAT(p_btif->scu_io));
					return ret;
				}

				if (SCU_BT_STAT_BT_WAKEUP_HOST(reg)) {
					dev_dbg(dev, "HWUP HIGH, skip D0I3\n");
					ret = -EBUSY;
					break;
				} else if (!(ioread32(
						BTIF_IMSC(p_btif->ctrl_io)) &
						(BTIF_RIS_RX_EOP_MASK |
						BTIF_RIS_RX_MRPSSTOP_MASK))) {
					dev_dbg(dev, "RX ongoing, skip D0I3\n");
					ret = -EBUSY;
					break;
				}
				imc_idi_btif_peripheral_flush(p_btif);

				if (platdata->btif_rst)
					reset_control_assert
						(platdata->btif_rst);
				ret = idi_set_power_state(p_device,
						(void *)btif_states.d0i3_handler
							, false);
				break;
			case D3:
				imc_idi_btif_peripheral_flush(p_btif);
				if (platdata->btip_rst)
					reset_control_assert
						(platdata->btip_rst);
				if (platdata->btif_rst)
					reset_control_assert
						(platdata->btif_rst);
				ret = idi_set_power_state(p_device,
						(void *)btif_states.d3_handler
							, false);
#ifdef IMC_IDI_BTIF_PARANOID
				break;
			default:
				dev_err(dev, "Invalid power state: %lu\n", arg);
				ret = -EINVAL;
#endif
			}

#ifdef CONFIG_HAS_WAKELOCK
			if ((arg == D0) || (arg == D0I2)) {
				if ((!ret) && (!wake_lock_active(
						&p_btif->btif_wake_lock)))
					wake_lock(&p_btif->btif_wake_lock);
			} else if ((arg == D0I3) || (arg == D3)) {
				if ((!ret) && (wake_lock_active(
						&p_btif->btif_wake_lock)))
					wake_unlock(&p_btif->btif_wake_lock);
			}
#endif

			if (arg == D0) {
				if (unlikely(p_btif->dstate == D3)) {
					if (platdata->btif_rst)
						reset_control_reset
							(platdata->btif_rst);
					if (platdata->btip_rst)
						reset_control_reset
							(platdata->btip_rst);
					_imc_btip_wait_clk(p_btif);
				}
				/* Re-configure IDI and BTIF */
				if (p_btif->dstate == D0I3) {
					if (platdata->btif_rst)
						reset_control_deassert
							(platdata->btif_rst);
					idi_set_channel_config(p_device,
						&p_btif->rx_ch_config);
					idi_set_channel_config(p_device,
						&p_btif->tx_ch_config);
					btif_init_hw(p_btif);
				}
			}
#ifdef IMC_IDI_BTIF_PARANOID
		}
		if (likely(!ret))
#endif
			p_btif->dstate = arg;
		break;
	case IMC_IDI_BT_SET_BT_WUP:
#ifdef IMC_IDI_BTIF_PARANOID
		if (likely(!p_btif->signaling_disabled)) {
#endif
			dev_dbg(dev, "Setting BT wakeup to %lu\n", arg);
#ifdef IMC_IDI_BTIF_PARANOID
			if (likely((arg == 0) || (arg == 1)))
#endif
				imc_idi_btif_wakeup_bt(p_btif, arg);
#ifdef IMC_IDI_BTIF_PARANOID
			else {
				dev_err(dev, "Invalid BT wakeup state: %lu\n",
					arg);
				ret = -EINVAL;
			}
		} else {
			dev_err(dev, "BT signaling disabled!\n");
			ret = -EINVAL;
		}
#endif
		break;
	case IMC_IDI_BT_GET_HOST_WUP:
		ret = idi_client_ioread(p_btif->p_dev,
			(unsigned)SCU_BT_STAT(p_btif->scu_io),
			&reg);

		if (ret) {
			dev_err(&p_btif->p_dev->device,
					"%s: idi read error %p\n", __func__,
					SCU_BT_STAT(p_btif->scu_io));
			return ret;
		}

		ret = SCU_BT_STAT_BT_WAKEUP_HOST(reg);
		break;
	case IMC_IDI_BT_SET_RTS:
#ifdef IMC_IDI_BTIF_PARANOID
		if (!p_btif->signaling_disabled) {
#endif
			dev_dbg(dev, "Setting BT RTS to %lu\n", arg);
#ifdef IMC_IDI_BTIF_PARANOID
			if (likely((arg == 0) || (arg == 1)))
#endif
				imc_idi_btif_set_rts(p_btif, arg);
#ifdef IMC_IDI_BTIF_PARANOID
			else {
				dev_err(dev, "Invalid RTS state: %lu\n", arg);
				ret = -EINVAL;
			}
		} else {
			dev_err(dev, "BT signaling disabled!\n");
			ret = -EINVAL;
		}
#endif
		break;
#ifdef IMC_IDI_BTIF_PARANOID
	case IMC_IDI_BT_GET_RTS:
		ret =
		    BTIF_MSS_STAT_FCO(ioread32(BTIF_MSS_STAT(p_btif->ctrl_io)));
		break;
#endif
	case IMC_IDI_BT_GET_CTS:
		ret =
		    BTIF_MSS_STAT_FCI(ioread32(BTIF_MSS_STAT(p_btif->ctrl_io)));
		break;
	case IMC_IDI_BT_DISABLE_SIGNALING:
#ifdef IMC_IDI_BTIF_PARANOID
		if (!p_btif->signaling_disabled) {
#endif
			dev_dbg(dev, "Disabling BT signaling\n");
			p_btif->signaling_disabled = 1;

			/* Disable H4N signaling in FW control */
			imc_btip_configure_fw(p_btif, true);

			/* Keep the WAKEUP_BT and RTS lines pulled up */
			imc_idi_btif_wakeup_bt(p_btif, true);
			imc_idi_btif_set_rts(p_btif, true);
#ifdef IMC_IDI_BTIF_PARANOID
		} else {
			dev_err(dev, "BT signaling already disabled\n");
			ret = -EINVAL;
		}
#endif
		break;
	case IMC_IDI_BT_SET_SCU_FWCTL:
		dev_dbg(dev, "Setting BT SCU FWCTL %lu\n", arg);
		if (idi_client_iowrite(p_btif->p_dev,
				(unsigned)SCU_BT_FWCTL(p_btif->scu_io), arg))
			dev_err(&p_btif->p_dev->device,
					"%s: idi write error %p\n",
					__func__, SCU_BT_FWCTL(p_btif->scu_io));
		break;
	case IMC_IDI_BT_GET_SCU_FWCTL:
		ret = idi_client_ioread(p_btif->p_dev,
				(unsigned)SCU_BT_FWCTL(p_btif->scu_io), &reg);

		if (ret) {
			dev_err(&p_btif->p_dev->device,
					"%s: idi read error %p\n",
					__func__, SCU_BT_FWCTL(p_btif->scu_io));
			return ret;
		}

		ret = SCU_BT_FWCTL_HOST2BT(reg);
		break;
	default:
		ret = -ENOIOCTLCMD;
	}

	IMC_IDI_BTIF_EXIT;

	return ret;
}

static struct uart_ops btif_ops = {
	.tx_empty = btif_tx_empty,
	.stop_rx = btif_stop_rx,
	.stop_tx = btif_stop_tx,
	.start_tx = btif_start_tx,
	.startup = btif_startup,
	.shutdown = btif_shutdown,
	.set_termios = btif_set_termios,
	.set_mctrl = btif_set_mctrl,
	.get_mctrl = btif_get_mctrl,
	.type = btif_type,
	.release_port = btif_release_port,
#ifdef IMC_IDI_BTIF_PARANOID
	.verify_port = btif_verify_port,
#endif
	.request_port = btif_request_port,
	.config_port = btif_config_port,
	.ioctl = btif_ioctl,
};

static struct uart_driver btif_reg = {
	.owner = THIS_MODULE,
	.driver_name = SERIAL_BTIF_NAME,
	.dev_name = SERIAL_BTIF_NAME,
	.major = SERIAL_BTIF_MAJOR,
	.minor = SERIAL_BTIF_MINOR,
	.nr = 1,
	.cons = NULL,
};

#if defined(CONFIG_OF)
static struct imc_idi_btif_platdata *imc_idi_btif_of_get_platdata(
				struct idi_peripheral_device *p_device)
{
	struct imc_idi_btif_platdata *platdata;

	struct device_node *np = p_device->device.of_node;
	struct device *dev = &p_device->device;

	platdata = kzalloc(sizeof(*platdata), GFP_KERNEL);
	if (!platdata)
		return NULL;

	platdata->kernel = of_clk_get_by_name(np, BTIF_CLK_KERNEL_NAME);
	if (IS_ERR(platdata->kernel)) {
		dev_warn(dev, "No kernel clock available\n");
		platdata->kernel = NULL;
	}

	platdata->bt_78M = of_clk_get_by_name(np, BTIF_CLK_78M_NAME);
	if (IS_ERR(platdata->bt_78M)) {
		dev_warn(dev, "No BTIP 78M clock available\n");
		platdata->bt_78M = NULL;
	}

	platdata->bt_104M = of_clk_get_by_name(np, BTIF_CLK_104M_NAME);
	if (IS_ERR(platdata->bt_104M)) {
		dev_warn(dev, "No BTIP 104M clock available\n");
		platdata->bt_104M = NULL;
	}

	platdata->bt_rtc = of_clk_get_by_name(np, BTIF_CLK_RTC_NAME);
	if (IS_ERR(platdata->bt_rtc)) {
		dev_warn(dev, "No BTIP 32kHz clock available\n");
		platdata->bt_rtc = NULL;
	}

	platdata->bt_vcxo = of_clk_get_by_name(np, BTIF_REQ_VCXO_NAME);
	if (IS_ERR(platdata->bt_vcxo)) {
		dev_warn(dev, "No BTIP VCXO request available\n");
		platdata->bt_vcxo = NULL;
	}

	platdata->bt_pwm_on = of_clk_get_by_name(np, BTIF_REQ_PWM_ON_NAME);
	if (IS_ERR(platdata->bt_pwm_on)) {
		dev_warn(dev, "No BTIP PWM ON request available\n");
		platdata->bt_pwm_on = NULL;
	}

	platdata->bt_rf_on = of_clk_get_by_name(np, BTIF_REQ_RF_ON_NAME);
	if (IS_ERR(platdata->bt_rf_on)) {
		dev_warn(dev, "No BTIP RF ON request available\n");
		platdata->bt_rf_on = NULL;
	}

	platdata->btif_rst = reset_control_get(dev, "btif");
	if (IS_ERR(platdata->btif_rst)) {
		dev_warn(dev, "No BtIf Reset controller found\n");
		platdata->btif_rst = NULL;
	}

	platdata->btip_rst = reset_control_get(dev, "btip");
	if (IS_ERR(platdata->btip_rst)) {
		dev_warn(dev, "No BtIP Reset controller found\n");
		platdata->btip_rst = NULL;
	}

	/* TODO: Get the wakeup .. from node */

	return platdata;

}
#endif /* CONFIG_OF */

static void netlink_callback(struct sk_buff *skb)
{
	/* Extract the Netlink Message Header */
	struct nlmsghdr *nlh = (struct nlmsghdr *)skb->data;

	/* Process ID (PID) of sending process */
	stack_pid = nlh->nlmsg_pid;
	pr_debug("BTIF got Netlink message from pid %u: %s",
		 stack_pid, (char *)nlmsg_data(nlh));
}

static struct netlink_kernel_cfg btif_netlink_cfg = {
	.groups = 0,
	.flags = 0,
	.input = netlink_callback,
	.cb_mutex = NULL,
	.bind = NULL,
};

static int imc_idi_btif_probe(struct idi_peripheral_device *p_device,
			      const struct idi_device_id *id)
{
	int ret = 0;
	struct imc_idi_btif_port *p_btif;
#ifndef CONFIG_NKERNEL
	struct resource *res;
#endif
#if defined(CONFIG_OF)
	struct imc_idi_btif_platdata *platdata;
#endif

	IMC_IDI_BTIF_ENTER;

	dev_dbg(&p_device->device,
		"IMC IDI BlueTooth Interface driver probe\n");

#ifdef ENABLE_DAP_CLOCK
	iowrite32(0x10000, 0xE640110C);
#endif

	p_btif = kzalloc(sizeof(*p_btif), GFP_KERNEL);
#ifdef IMC_IDI_BTIF_PARANOID
	if (!p_btif) {
		ret = -ENOMEM;
		goto exit_btif_probe;
	}
#endif
	p_btif->p_dev = p_device;

	dev_set_drvdata(&p_device->device, p_btif);

#if defined(CONFIG_OF)
	platdata = imc_idi_btif_of_get_platdata(p_device);
#ifdef IMC_IDI_BTIF_PARANOID
	if (!platdata) {
		ret = -EINVAL;
		goto fail_request;
	}
#endif
	p_device->device.platform_data = platdata;
#endif

#ifdef CONFIG_PLATFORM_DEVICE_PM
	ret = device_state_pm_set_class(&p_device->device,
		p_device->pm_platdata->pm_user_name);
#ifdef IMC_IDI_BTIF_PARANOID
	if (ret) {
		pr_err("BTIF device PM registration failed\n");
		goto fail_request;
	} else
#endif
		pr_err("BTIF device PM registration success\n");
#endif

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&p_btif->btif_wake_lock, WAKE_LOCK_SUSPEND,
					"btif_wakelock");
	wake_lock_init(&btauif_wake_lock, WAKE_LOCK_SUSPEND,
					"btauif_wakelock");
#endif

	p_btif->ctrl_io = imc_idi_request_io_byname(p_device,
						    BTIF_REG_RES_NAME, false,
						    true);

	p_btif->scu_io = imc_idi_request_io_byname(p_device,
						   BTIF_SCU_RES_NAME, true,
						   false);

#ifdef IMC_IDI_BTIF_PARANOID
	if ((!p_btif->ctrl_io) || (!p_btif->scu_io)) {
		ret = -EPERM;
		dev_err(&p_device->device,
			"Failed to request resource\n");

		goto fail_request;
	}
#endif

	dev_dbg(&p_device->device, "IO remapping: reg %p, scu %p\n",
		p_btif->ctrl_io, p_btif->scu_io);

	p_btif->port.ops = &btif_ops;
	p_btif->port.dev = &p_device->device;

#ifndef CONFIG_NKERNEL
	res = idi_get_resource_byname(&p_device->resources,
				      IORESOURCE_MEM, BTIF_REG_RES_NAME);
#ifdef IMC_IDI_BTIF_PARANOID
	if (!res)
		goto fail_add_uart_port;
#endif
#endif

	spin_lock_init(&p_btif->rxbuff_lock);
	spin_lock_init(&p_btif->txstate_lock);
	spin_lock_init(&p_btif->hw_lock);
	spin_lock_init(&p_btif->state_lock);

#ifndef CONFIG_NKERNEL
	p_btif->port.mapbase = res->start;
#else
	p_btif->port.mapbase = (resource_size_t) p_btif->ctrl_io;
#endif
	p_btif->port.membase = p_btif->ctrl_io;
	p_btif->port.iotype = UPIO_MEM;
	p_btif->port.flags = UPF_BOOT_AUTOCONF;
	p_btif->port.line = 0;

	btif_states.d0_handler = idi_peripheral_device_pm_get_state_handler(
			p_device, "enable");
	btif_states.d0i2_handler = idi_peripheral_device_pm_get_state_handler(
			p_device, "enable_psv");
	btif_states.d0i3_handler = idi_peripheral_device_pm_get_state_handler(
			p_device, "idle");
	btif_states.d3_handler = idi_peripheral_device_pm_get_state_handler(
			p_device, "disable");

	imc_btip_init(p_btif);

	btif_set_state(p_btif, BT_OFF);
	g_bt_audio_state = btauif_aud_off;

	ret = uart_add_one_port(&btif_reg, &p_btif->port);
#ifdef IMC_IDI_BTIF_PARANOID
	if (ret) {
		dev_dbg(&p_device->device, "Adding port failed!\n");
		goto fail_add_uart_port;
	}
#endif

	/* Create netlink socket */
	p_btif->nl_socket = netlink_kernel_create(&init_net,
					NETLINK_USERSOCK, &btif_netlink_cfg);

#ifdef IMC_IDI_BTIF_PARANOID
	if (!p_btif->nl_socket) {
		dev_err(&p_device->device,
			"Netlink receive handler registration failed\n");
		goto fail_create_nl_sk;
	} else
#endif
		dev_dbg(&p_device->device,
			"Netlink receive handler registration succeded\n");
	stack_pid = 0;

	IMC_IDI_BTIF_EXIT;

	return 0;

#ifdef IMC_IDI_BTIF_PARANOID
fail_create_nl_sk:
fail_add_uart_port:
fail_request:

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&p_btif->btif_wake_lock);
	wake_lock_destroy(&btauif_wake_lock);
#endif

#if defined(CONFIG_OF)
	kfree(platdata);
#endif
	kfree(p_btif);
exit_btif_probe:
	return ret;
#endif
}

static int imc_idi_btif_remove(struct idi_peripheral_device *p_device)
{
	int ret = 0;
	struct imc_idi_btif_port *p_btif;

	IMC_IDI_BTIF_ENTER;

	p_btif = dev_get_drvdata(&p_device->device);

	if (p_btif->nl_socket) {
		netlink_kernel_release(p_btif->nl_socket);
		dev_dbg(&p_device->device,
			"Netlink receive handler release succeded\n");
	}

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&p_btif->btif_wake_lock);
	wake_lock_destroy(&btauif_wake_lock);
#endif

	uart_remove_one_port(&btif_reg, &p_btif->port);

	dev_set_drvdata(&p_device->device, NULL);

	imc_idi_release_io_byname(p_device,
				  BTIF_REG_RES_NAME, p_btif->ctrl_io, false);

	imc_idi_release_io_byname(p_device,
				  BTIF_SCU_RES_NAME, p_btif->scu_io, true);

#if defined(CONFIG_OF)
	kfree(imc_idi_btif_get_platdata(p_btif));
#endif
	kfree(p_btif);

	IMC_IDI_BTIF_EXIT;
	return ret;
}
static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG610,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_BT,
	},
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_BT,
	},

	{ /* end: all zeroes */},
};

/*Called when the system is attempting to suspend. */
static int imc_idi_btif_suspend(struct device *dev)
{
#ifdef CONFIG_AG6XX
	struct imc_idi_btif_port *p_btif = dev_get_drvdata(dev);
	struct uart_port *port = &p_btif->port;
#endif

	pr_debug("BTIF: Platform entering suspend\n");

#ifdef CONFIG_AG6XX
	btif_ioctl(port, IMC_IDI_BT_SET_BT_WUP, false);
#endif
	return 0;
}

/*Called when the system is resuming from suspend. */
static int imc_idi_btif_resume(struct device *dev)
{
#ifdef CONFIG_AG6XX
	struct imc_idi_btif_port *p_btif = dev_get_drvdata(dev);
	struct uart_port *port = &p_btif->port;
#endif

	pr_debug("BTIF: Received platform resume event\n");

#ifdef CONFIG_AG6XX
	pr_debug("Changing BT Controller State to ON\n");
	btif_ioctl(port, IMC_IDI_BT_SET_POWER_STATE, IMC_IDI_BT_POWER_STATE_ON_D0);

	pr_debug("BTIF: Platform resuming from suspend\n");
	btif_ioctl(port, IMC_IDI_BT_SET_BT_WUP, true);
#endif

	return 0;
}

const struct dev_pm_ops imc_idi_btif_pm_ops = {
	.suspend = imc_idi_btif_suspend,
	.resume = imc_idi_btif_resume,
};

static struct idi_peripheral_driver imc_idi_btif_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = SERIAL_BTIF_NAME,
		   .pm = &imc_idi_btif_pm_ops,
		   },
	.p_type = IDI_BT,
/*	.id_table = idi_ids, */
	.probe = imc_idi_btif_probe,
	.remove = imc_idi_btif_remove,

};

MODULE_DEVICE_TABLE(idi, idi_ids);

static int __init imc_idi_btif_init(void)
{
	int ret = 0;
	pr_debug("IMC IDI BlueTooth Interface driver initialization\n");

	IMC_IDI_BTIF_ENTER;

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	ret = device_state_pm_add_class(&btif_pm_class);
	if (ret) {
		pr_err("Error while adding BTIF pm class\n");
		return ret;
	}
#endif

	ret = uart_register_driver(&btif_reg);
#ifdef IMC_IDI_BTIF_PARANOID
	if (ret)
		return -EINVAL;
#endif

	ret = idi_register_peripheral_driver(&imc_idi_btif_driver);
#ifdef IMC_IDI_BTIF_PARANOID
	if (ret)
		uart_unregister_driver(&btif_reg);
#endif

	IMC_IDI_BTIF_EXIT;
	return ret;
}

static void __exit imc_idi_btif_exit(void)
{
	IMC_IDI_BTIF_ENTER;

	idi_unregister_peripheral_driver(&imc_idi_btif_driver);
	uart_unregister_driver(&btif_reg);

	IMC_IDI_BTIF_EXIT;
}

module_init(imc_idi_btif_init);
module_exit(imc_idi_btif_exit);

int btauif_hal_audio_stop(void __iomem *ctrl_io)
{
	unsigned reg = 0;

	IMC_IDI_BT_AU_IF_ENTER;

	/*enter in config mode and stay in config mode to stop the module */
	iowrite32(BT_AU_IF_CONFIG_MODE, BT_AU_IF_CLC(ctrl_io));

	reg |= BT_AU_IF_FIFO_CTRL_TX_ABORT_MASK;
	reg |= BT_AU_IF_FIFO_CTRL_RX_ABORT_MASK;
	iowrite32(reg, BT_AU_IF_FIFO_CTRL(ctrl_io));

	reg = ioread32(BT_AU_IF_CTRL(ctrl_io));
	reg &= ~BT_AU_IF_CTRL_HANDSHAKE_EN_MASK;
	reg &= ~BT_AU_IF_CTRL_AUDIO_CLK_EN_MASK;
	iowrite32(reg, BT_AU_IF_CTRL(ctrl_io));

	IMC_IDI_BT_AU_IF_EXIT;

	return imc_idi_btauif_result_ok;
}

int btauif_hal_audio_start(void __iomem *ctrl_io)
{
	unsigned reg = 0;

	IMC_IDI_BT_AU_IF_ENTER;
	/* Config mode */
	iowrite32(BT_AU_IF_CONFIG_MODE, BT_AU_IF_CLC(ctrl_io));
	/* TPS register should be 0 */
	iowrite32(0, BT_AU_IF_TPS_CTRL(ctrl_io));
	pr_debug("Doing FIFO start\n");

	reg |= BT_AU_IF_FIFO_CTRL_TX_START_MASK;
	reg |= BT_AU_IF_FIFO_CTRL_RX_START_MASK;
	iowrite32(reg, BT_AU_IF_FIFO_CTRL(ctrl_io));

	pr_debug("Enabling handshake\n");

	reg = ioread32(BT_AU_IF_CTRL(ctrl_io));
	reg |= BT_AU_IF_CTRL_HANDSHAKE_EN_MASK;
	reg |= BT_AU_IF_CTRL_AUDIO_CLK_EN_MASK;
	iowrite32(reg, BT_AU_IF_CTRL(ctrl_io));

	pr_debug("Putting in Run mode\n");
	/* Run mode */
	iowrite32(BT_AU_IF_RUN_MODE, BT_AU_IF_CLC(ctrl_io));
	IMC_IDI_BT_AU_IF_EXIT;

	return imc_idi_btauif_result_ok;
}

int btauif_hal_audio_configure(struct imc_idi_btauif_config *config_params,
				void __iomem *ctrl_io)
{
	unsigned reg;

	IMC_IDI_BT_AU_IF_ENTER;
	/* Config mode */
	iowrite32(BT_AU_IF_CONFIG_MODE, BT_AU_IF_CLC(ctrl_io));
	reg = ioread32(BT_AU_IF_FIFO_CFG(ctrl_io));

	/* TX FIFO configuration */
	/* TX in NFC mode */
	reg &= ~BT_AU_IF_FIFO_CFG_TXFC_MASK;
	/* burst size is 1 word , not used in streaming */
	reg &= ~BT_AU_IF_FIFO_CFG_TXBS_MASK;
	/* TX Fifo alignment is one byte */
	reg &= ~BT_AU_IF_FIFO_CFG_TXFA_MASK;
	/* RX FIFO configuration */
	reg &= ~BT_AU_IF_FIFO_CFG_TX_SWAP_MASK;
	/* RX in NFC mode */
	reg &= ~BT_AU_IF_FIFO_CFG_RXFC_MASK;
	/* burst size is 1 word */
	reg &= ~BT_AU_IF_FIFO_CFG_RXBS_MASK;
	/* RX Fifo alignment is one byte */
	reg &= ~BT_AU_IF_FIFO_CFG_RXFA_MASK;
	reg &= ~BT_AU_IF_FIFO_CFG_RX_SWAP_MASK;

	iowrite32(reg, BT_AU_IF_FIFO_CFG(ctrl_io));

	/* mute configuration from user */
#ifdef IMC_IDI_BTIF_PARANOID
	if (NULL != config_params) {
#endif
		reg = ioread32(BT_AU_IF_CTRL(ctrl_io));
		reg &= ~BT_AU_IF_CTRL_AUDIO_CLK_DEF_MASK;
		reg |= (config_params->clk_rate <<
			BT_AU_IF_CTRL_AUDIO_CLK_DEF_OFFSET);
		iowrite32(reg, BT_AU_IF_CTRL(ctrl_io));

		reg = ioread32(BT_AU_IF_MUTE_CTRL(ctrl_io));
		reg &= ~BT_AU_IF_MUTE_CTRL_MUTE_SAMPLE_MASK;
		reg |= (config_params->mute_sample <<
			BT_AU_IF_MUTE_CTRL_MUTE_SAMPLE_OFFSET);
		reg &= ~BT_AU_IF_MUTE_CTRL_MUTE_THRSHLD_MASK;
		reg |= (config_params->mute_threshold <<
			BT_AU_IF_MUTE_CTRL_MUTE_THRSHLD_OFFSET);
		reg &= ~BT_AU_IF_MUTE_CTRL_MUTE_EN_MASK;
		reg |= (config_params->mute_enable <<
			BT_AU_IF_MUTE_CTRL_MUTE_EN_OFFSET);
		iowrite32(reg, BT_AU_IF_MUTE_CTRL(ctrl_io));
#ifdef IMC_IDI_BTIF_PARANOID
	} else {
		IMC_IDI_BT_AU_IF_EXIT;
		return imc_idi_btauif_result_error;
	}
#endif

	IMC_IDI_BT_AU_IF_EXIT;

	return imc_idi_btauif_result_ok;
}

int btauif_hal_audio_ctrl(
			struct imc_idi_btauif_config *config_params,
			bool mode, void __iomem *ctrl_io)
{
	IMC_IDI_BT_AU_IF_ENTER;

#ifdef IMC_IDI_BTIF_PARANOID
	if (!ctrl_io) {
		pr_err("BTAUIF register base not initialized\n");
		return imc_idi_btauif_result_internal_error;
	}
#endif

	switch (mode) {
	case btauif_audio_enable_cmd:
	{
		if (g_bt_audio_state == btauif_aud_off) {
#ifdef IMC_IDI_BTIF_PARANOID
			if (temp_meas_timer.function)
#endif
				if (!timer_pending(&temp_meas_timer))
					mod_timer(&temp_meas_timer, jiffies);

			btauif_hal_audio_configure(config_params, ctrl_io);
			btauif_hal_audio_start(ctrl_io);

			g_bt_audio_state = btauif_aud_on;
#ifdef CONFIG_HAS_WAKELOCK
			if (!wake_lock_active(&btauif_wake_lock))
				wake_lock(&btauif_wake_lock);
#endif
		}
	}
	break;

	case btauif_audio_disable_cmd:
	{
#ifdef IMC_IDI_BTIF_PARANOID
		if (g_bt_audio_state == btauif_aud_on) {
#endif
			btauif_hal_audio_stop(ctrl_io);

			g_bt_audio_state = btauif_aud_off;
#ifdef CONFIG_HAS_WAKELOCK
			if (wake_lock_active(&btauif_wake_lock))
				wake_unlock(&btauif_wake_lock);
#endif
#ifdef IMC_IDI_BTIF_PARANOID
		}
#endif
	}
#ifdef IMC_IDI_BTIF_PARANOID
	break;

	default:
		return imc_idi_btauif_result_error;
#endif
	}

	IMC_IDI_BT_AU_IF_EXIT;
	return imc_idi_btauif_result_ok;
}

int imc_idi_btauif_audio_enable_ex(struct imc_idi_btauif_config *config_params,
				void __iomem *ctrl_io)
{
	int result = imc_idi_btauif_result_error;
#ifdef IMC_IDI_BTIF_PARANOID
	if (!config_params) {
		pr_err("Configuration parameters not set\n");
		return imc_idi_btauif_result_invalid_param;
	}
#endif
	result = btauif_hal_audio_ctrl(config_params, btauif_audio_enable_cmd,
			ctrl_io);
	return result;
}

int imc_idi_btauif_audio_enable(unsigned char clk_rate, void __iomem *ctrl_io)
{
	int result = imc_idi_btauif_result_error;
	struct imc_idi_btauif_config config_params;

	config_params.clk_rate = clk_rate;
	config_params.mute_enable = imc_idi_btauif_mute_enable;
	config_params.mute_sample = 0x00;
	config_params.mute_threshold = 0x00;

	result = imc_idi_btauif_audio_enable_ex(&config_params, ctrl_io);

	return result;
}

int imc_idi_btauif_audio_disable(void __iomem *ctrl_io)
{
	int result = imc_idi_btauif_result_error;
	result =
	    btauif_hal_audio_ctrl(NULL, btauif_audio_disable_cmd, ctrl_io);
	return result;
}

MODULE_LICENSE("GPL");
