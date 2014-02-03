/*
 ****************************************************************
 *
 *  Component: DCC driver
 *
 *  Copyright (C) 2011-2013, Intel Mobile Communications GmbH.
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
 ****************************************************************
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

#include "dcc-core.h"
#ifdef CONFIG_XGOLD_DCC_SYSFS
#include "dcc-sysfs.h"
#endif
#include "dcc-hwregs.h"

#define DCC_IRQ_ERR_ENABLED \
	(DCC_IRQ_ERR_MASK & (~DCC_IRQ_ERR_IDLE))

static void dcc_hal_signal_vsync(struct dcc_drvdata *pdata)
{
	struct timespec ts;

	ktime_get_ts(&ts);
	pdata->vsync_ts = (NSEC_PER_SEC * (long long unsigned)ts.tv_sec)
				+ ts.tv_nsec;
	DCC_DBG4("DCC_IRQ_VSYNC 0x%llx (%ds %li)\n",
			pdata->vsync_ts, (int)ts.tv_sec, ts.tv_nsec);
	queue_work(pdata->vsync_wq, &pdata->vsync_work);
}

static irqreturn_t dcc_hal_event_vsync(int irq, void *dev_id)
{
	struct dcc_drvdata *pdata = (struct dcc_drvdata *)dev_id;

	dcc_hal_signal_vsync(pdata);

	return IRQ_HANDLED;
}

static irqreturn_t dcc_hal_irq_cmd(int irq, void *dev_id)
{
	struct dcc_drvdata *pdata = (struct dcc_drvdata *)dev_id;
	DCC_DBG3("DCC_IRQ_CMD\n");
	gra_write_field(pdata, EXR_DIF_ICR, DCC_IRQ_CMD);
	complete(&pdata->sync.eoc);
	return IRQ_HANDLED;
}

static inline void dcc_hal_irq_dsi_fin(struct dcc_drvdata *pdata)
{
	complete(&pdata->sync.dsifin);
	return;
}

static inline void dcc_hal_irq_dsi_tr1(struct dcc_drvdata *pdata)
{
	complete(&pdata->sync.dsitr1);
	return;
}

static inline void dcc_hal_irq_dsi_tr2(struct dcc_drvdata *pdata)
{
	gra_write_field(pdata, INR_DIF_DSICFG, 0x12F8);
	return;
}


#define DCC_INTERRUPT_CLEAR(_irq_) {\
if (dif_err_irqss & _irq_) { \
		dif_err_irqsc |= _irq_; \
	} }
#define DCC_LOG_INTERRUPT_DEBUG(_irq_) {\
if (dif_err_irqss & _irq_) { \
		DCC_DBG3(#_irq_"\n"); \
		dif_err_irqsc |= _irq_; \
	} }
#define DCC_LOG_INTERRUPT_DEBUG_HW_CB(_irq_, _cb_, _p_) {\
if (dif_err_irqss & _irq_) { \
		DCC_DBG4(#_irq_"\n"); \
		_cb_(_p_); \
		dif_err_irqsc |= _irq_; \
	} }

#define DCC_LOG_INTERRUPT_DEBUG_DBG_CB(_irq_, _cb_, _p_) {\
if (dif_err_irqss & _irq_) { \
		DCC_DBG2(#_irq_"\n"); \
		_cb_(_p_); \
		dif_err_irqsc |= _irq_; \
	} }

#define DCC_LOG_INTERRUPT_ERROR(_irq_, _p_) {\
if (dif_err_irqss & _irq_) { \
	unsigned int stat, ris, txffs; \
	gra_read_field(_p_, EXR_DIF_STAT, &stat); \
	gra_read_field(_p_, EXR_DIF_RIS, &ris); \
	gra_read_field(_p_, EXR_DIF_TXFFS_STAT, &txffs); \
	dcc_err(#_irq_\
	" ERRIRQSS:0x%08x STAT:0x%08x RIS:0x%08x TXFFS:0x%08x\n",\
		dif_err_irqss, stat, ris, txffs); \
		dif_err_irqsc |= _irq_; \
	} }

static irqreturn_t dcc_hal_irq_err(int irq, void *dev_id)
{
	unsigned int dif_err_irqss = 0;
	unsigned int dif_err_irqsc = 0;
	struct dcc_drvdata *pdata = (struct dcc_drvdata *)dev_id;

	gra_read_field(pdata, EXR_DIF_ERRIRQSS, &dif_err_irqss);

	DCC_DBG4("%s: Hardware Interrupt EXR_DIF_ERRIRQSS = 0x%08x\n",
			__func__, dif_err_irqss);

	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_RXFUFL, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_RXFOFL, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_TXFOFL, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_PHASE, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_CMD, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_MASTER, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_TXUFL, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_MASTER2, pdata);
	DCC_INTERRUPT_CLEAR(DCC_IRQ_ERR_IDLE);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_DSITR0, pdata);
	DCC_LOG_INTERRUPT_DEBUG_DBG_CB(DCC_IRQ_ERR_DSITR1,
			dcc_hal_irq_dsi_tr1, pdata);
	DCC_LOG_INTERRUPT_DEBUG_DBG_CB(DCC_IRQ_ERR_DSITR2,
			dcc_hal_irq_dsi_tr2, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_DSITR3, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_DSIUFL, pdata);
	DCC_LOG_INTERRUPT_DEBUG_HW_CB(DCC_IRQ_ERR_DSIFIN,
			dcc_hal_irq_dsi_fin, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_DSILTO, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_DSIHTO, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_DSIRTO, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_DSIESC, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_DSISYN, pdata);
	DCC_LOG_INTERRUPT_ERROR(DCC_IRQ_ERR_DSICTR, pdata);
	DCC_INTERRUPT_CLEAR(DCC_IRQ_ERR_DSICON);

	if (dif_err_irqss != dif_err_irqsc)
		dcc_err("unhandled error interrupts\n"
			"	IRQSS=0x%08x\n"
			"	IRQSC=0x%08x\n"
			"	unhdl=0x%08x\n",
			dif_err_irqss, dif_err_irqsc,
			dif_err_irqss & ~dif_err_irqsc);

	/* clear treated handled interrupts */
	gra_write_field(pdata, EXR_DIF_ERRIRQSC, dif_err_irqsc);
	return IRQ_HANDLED;
}


static inline void dcc_hal_irq_burst(void)
{
	DCC_DBG2("%s: BURST INTERRUPT CALLED!\n", __func__);
	return;
}

static irqreturn_t dcc_hal_irq_tx(int irq, void *dev_id)
{
	unsigned int dif_ris;
	struct dcc_drvdata *pdata = (struct dcc_drvdata *)dev_id;

	gra_read_field(pdata, EXR_DIF_RIS, &dif_ris);

	/* TX_BREQ */
	if (dif_ris & 0x80) {
		/* Burst interrupt received
		 * -> now there is room in hardware fifo */
		dcc_hal_irq_burst();
		DCC_DBG2("%s: Scheduling burst thread now!\n", __func__);
		gra_write_field(pdata, EXR_DIF_ICR, 0x80);
	}

	return IRQ_HANDLED;
}


static irqreturn_t dcc_hal_irq_rx(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}


static irqreturn_t dcc_hal_irq_frame(int irq, void *dev_id)
{
	struct dcc_drvdata *pdata = (struct dcc_drvdata *)dev_id;

	if (pdata->display_autorefresh)
		dcc_hal_signal_vsync(pdata);
	queue_work(pdata->eof_wq, &pdata->eof_work);

	complete(&pdata->sync.eof);
	/* clear interrupt flag */
	DCC_DBG4("DCC_IRQ_FRAME\n");
	gra_write_field(pdata, EXR_DIF_ICR, DCC_IRQ_FRAME);

	return IRQ_HANDLED;
}


static void dcc_hal_unregister_irq(struct dcc_drvdata *pdata)
{
	/* TODO gra_write_field(EXR_DIF_IMSC,0); */
	free_irq(pdata->irq.frame, 0);
	free_irq(pdata->irq.cmd, 0);
	free_irq(pdata->irq.err, 0);
	free_irq(pdata->irq.tx, 0);
	free_irq(pdata->irq.rx, 0);
}

static struct irqaction dcc_frame_irq = {
	.dev_id = 0,
	.name = "dcc.frame",
	.handler = dcc_hal_irq_frame,
	.flags = IRQF_SHARED,
};
static struct irqaction dcc_cmd_irq = {
	.dev_id = 0,
	.name = "dcc.cmd",
	.handler = dcc_hal_irq_cmd,
	.flags = IRQF_SHARED,
};
static struct irqaction dcc_err_irq = {
	.dev_id = 0,
	.name = "dcc.sub",
	.handler = dcc_hal_irq_err,
	.flags = IRQF_SHARED,
};
static struct irqaction dcc_tx_irq = {
	.dev_id = 0,
	.name = "dcc.tx",
	.handler = dcc_hal_irq_tx,
	.flags = IRQF_SHARED,
};
static struct irqaction dcc_rx_irq = {
	.dev_id = 0,
	.name = "dcc.rx",
	.handler = dcc_hal_irq_rx,
	.flags = IRQF_SHARED,
};
static struct irqaction dcc_vsync_irq = {
	.dev_id = 0,
	.name = "dcc.vsync",
	.handler = dcc_hal_event_vsync,
	.flags = IRQF_SHARED,
};

#define DCC_SETUP_IRQ(_irq_, _action_, _data_) {\
	if (_irq_) { \
		_action_.dev_id = (void *)_data_; \
		ret = setup_irq(_irq_, &_action_); \
		if (ret != 0) { \
			dcc_err("setup irq %s %d failed(ret = %d)\n", \
					_action_.name, _irq_, ret); \
			goto exit_stage_1; \
		} \
	} }

static int dcc_hal_install_irqs(struct dcc_drvdata *pdata)
{
	int ret = 0;

	gra_write_field(pdata, EXR_DIF_ERRIRQSC, DCC_IRQ_ERR_ENABLED);
	DCC_SETUP_IRQ(pdata->irq.frame, dcc_frame_irq, pdata);
	DCC_SETUP_IRQ(pdata->irq.cmd, dcc_cmd_irq, pdata);
	DCC_SETUP_IRQ(pdata->irq.err, dcc_err_irq, pdata);
	DCC_SETUP_IRQ(pdata->irq.tx, dcc_tx_irq, pdata);
	DCC_SETUP_IRQ(pdata->irq.rx, dcc_rx_irq, pdata);

	if (!pdata->display_autorefresh)
		DCC_SETUP_IRQ(pdata->irq.vsync, dcc_vsync_irq, pdata);

	return 0;

exit_stage_1:
	/* unregister irq's */
	dcc_hal_unregister_irq(pdata);

	return ret;
}

int dcc_hal_probe(struct dcc_drvdata *pdata)
{
	int ret = 0;

	gra_write_field(pdata, EXR_DIF_IMSC, 0); /* mask interrupt */
	gra_write_field(pdata, EXR_DIF_ICR, 0xfff); /* clear interrupt */

	ret = dcc_hal_install_irqs(pdata);
	if (ret)
		goto exit;

	dcc_boot_dbg("IRQ's registered ... OK\n");

exit:
	return ret;
}

int dcc_hal_remove(struct dcc_drvdata *pdata)
{
	/* unregister irq's */
	/* to detect errors on mode switching unregistering irqs must be done
	 * after switching to config mode */
	dcc_hal_unregister_irq(pdata);

	dcc_boot_dbg("DCC hal exited successfully!\n");
	return 0;
}

int dcc_interrupt_setup(struct dcc_drvdata *p)
{
	unsigned int dif_imsc = 0;

	if (gra_write_field(p, EXR_DIF_ICR, 0xfff) != 0) {
		dcc_err("unable to write to the icr register\n");
		goto error;
	}
	if (gra_write_field(p, EXR_DIF_ERRIRQSC, 0x2000) != 0) {
		dcc_err("unable to write to the err_irqsc register\n");
		goto error;
	}

	if (gra_write_field(p, EXR_DIF_ERRIRQSC, 0x2000) != 0) {
		dcc_err("unable to write to the err_irqsc register\n");
		goto error;
	}

	dif_imsc = DCC_IRQ_ERR | DCC_IRQ_CMD | DCC_IRQ_FRAME;

	gra_write_field(p, EXR_DIF_IMSC, dif_imsc);

	/* activate the error interrupts */
	gra_write_field(p, EXR_DIF_ERRIRQSM, DCC_IRQ_ERR_ENABLED);

	return 0;

error:
	return -EIO;
}
