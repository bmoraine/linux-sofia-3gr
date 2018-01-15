/*
 *******************************************************************************
 *
 *  Component: Xgold MIPI DSI driver
 *
 *  Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *******************************************************************************
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/bitops.h>

#include "dsi_hwregs.h"

static inline void dsi_hal_irq_dsi_fin(struct dsi_display *display)
{
	complete(&display->sync.dsifin);
}

static inline void dsi_hal_irq_dsi_tr1(struct dsi_display *display)
{
	complete(&display->sync.dsitr1);
}

static inline void dsi_hal_irq_dsi_tr2(struct dsi_display *display)
{
	dsi_write_field(display, EXR_DSI_CFG, BITFLDS(EXR_DSI_CFG_TX, 1) |
		BITFLDS(EXR_DSI_CFG_LP, 1) |
		BITFLDS(EXR_DSI_CFG_MODE, 1) |
		BITFLDS(EXR_DSI_CFG_EOT, 1) |
		BITFLDS(EXR_DSI_CFG_TURN, 1) |
		BITFLDS(EXR_DSI_CFG_DATA, 1) |
		BITFLDS(EXR_DSI_CFG_EN, 1) |
		BITFLDS(EXR_DSI_CFG_SOURCE, 1));
}

#define DSI_INTERRUPT_CLEAR(_irq_) {\
if (dsi_irq_status & _irq_) { \
		dsi_irq_clear |= _irq_; \
	} }

#define DSI_LOG_INTERRUPT_DEBUG_HW_CB(_irq_, _cb_, _p_) {\
if (dsi_irq_status & _irq_) { \
		_cb_(_p_); \
		dsi_irq_clear |= _irq_; \
	} }

#define DSI_LOG_INTERRUPT_DEBUG_DBG_CB(_irq_, _cb_, _p_) {\
if (dsi_irq_status & _irq_) { \
		pr_info("[dsi]"#_irq_ "\n"); \
		_cb_(_p_); \
		dsi_irq_clear |= _irq_; \
	} }

#define DSI_LOG_INTERRUPT_ERROR(_irq_, _p_) {\
if (dsi_irq_status & _irq_) { \
	unsigned int stat, ris, txffs; \
	stat = dsi_read_field(_p_, EXR_DSI_STAT); \
	ris = dsi_read_field(_p_, EXR_DSI_RIS); \
	txffs = dsi_read_field(_p_, EXR_DSI_FIFO_STAT); \
pr_info("[dsi]"#_irq_\
" DSI_STAT:0x%08x DSI_RIS:0x%08x DSI_FIFO_STAT:0x%08x\n",\
	stat, ris, txffs); \
		dsi_irq_clear |= _irq_; \
	} }

static irqreturn_t dsi_hal_irq_err(int irq, void *dev_id)
{
	unsigned int dsi_irq_status = 0;
	unsigned int dsi_irq_clear = 0;
	struct dsi_display *display = (struct dsi_display *)dev_id;

	dsi_irq_status = dsi_read_field(display, EXR_DSI_RIS);

	/*pr_info("%s: Hardware Interrupt EXR_DSI_RIS = 0x%08x\n",
		__func__, dsi_irq_status);*/

	DSI_INTERRUPT_CLEAR(DSI_IRQ_ERR_IDLE);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSITR0, display);
	DSI_LOG_INTERRUPT_DEBUG_DBG_CB(DSI_IRQ_ERR_DSITR1,
				       dsi_hal_irq_dsi_tr1, display);
	DSI_LOG_INTERRUPT_DEBUG_DBG_CB(DSI_IRQ_ERR_DSITR2,
				       dsi_hal_irq_dsi_tr2, display);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSITR3, display);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSIUFL, display);
	DSI_LOG_INTERRUPT_DEBUG_HW_CB(DSI_IRQ_ERR_DSIFIN,
				       dsi_hal_irq_dsi_fin, display);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSILTO, display);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSIHTO, display);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSIRTO, display);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSIESC, display);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSISYN, display);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSICTR, display);
	DSI_INTERRUPT_CLEAR(DSI_IRQ_ERR_DSICON);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSIOFL, display);

/*	if (dif_err_irqss != dif_err_irqsc)
		dsi_err("unhandled error interrupts\n"
			"	IRQSS=0x%08x\n"
			"	IRQSC=0x%08x\n"
			"	unhdl=0x%08x\n",
			dif_err_irqss, dif_err_irqsc,
			dif_err_irqss & ~dif_err_irqsc);*/

	/* clear treated handled interrupts */
	dsi_write_field(display, EXR_DSI_ICR, dsi_irq_clear);
	return IRQ_HANDLED;
}

static void dsi_hal_unregister_irq(struct dsi_display *display)
{
	/* TODO gra_write_field(EXR_DIF_IMSC,0); */
	free_irq(display->irq.rx_breq, 0);
	free_irq(display->irq.err, 0);
	free_irq(display->irq.tx, 0);
	free_irq(display->irq.rx, 0);
}

static struct irqaction dsi_err_irq = {
	.dev_id = 0,
	.name = "dsi.err",
	.handler = dsi_hal_irq_err,
	.flags = IRQF_SHARED,
};

#define DSI_SETUP_IRQ(_irq_, _action_, _data_) {\
	if (_irq_) { \
		_action_.dev_id = (void *)_data_; \
		ret = setup_irq(_irq_, &_action_); \
		if (ret != 0) { \
			pr_info("setup irq %s %d failed(ret = %d)\n", \
					_action_.name, _irq_, ret); \
			goto exit_stage_1; \
		} \
	} }

static int dsi_hal_install_irqs(struct dsi_display *display)
{
	int ret = 0;

	dsi_write_field(display, EXR_DSI_ICR, 0x7FFFFF);
	DSI_SETUP_IRQ(display->irq.err, dsi_err_irq, display);

	return 0;

exit_stage_1:
	dsi_hal_unregister_irq(display);
	return ret;
}

int dsi_irq_probe(struct dsi_display *display)
{
	dsi_write_field(display, EXR_DSI_IMSC, 0); /* mask interrupt */

	return dsi_hal_install_irqs(display);
}

int dsi_irq_remove(struct dsi_display *display)
{
	/* unregister irq's */
	/* to detect errors on mode switching unregistering irqs must be done
	 * after switching to config mode */
	dsi_hal_unregister_irq(display);

	pr_info("dsi irq exit successfully!\n");
	return 0;
}

void dsi_interrupt_setup(struct dsi_display *display)
{
	dsi_write_field(display, EXR_DSI_ICR, 0x7FFFFF);
}

