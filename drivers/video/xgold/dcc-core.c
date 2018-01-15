/*
 ****************************************************************
 *
 *  Component: DCC driver
 *
 *  Copyright (C) 2011, Intel Mobile Communications GmbH.
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

#include <linux/netlink.h>
#include <linux/connector.h>
#include <net/net_namespace.h>
#include <linux/cn_proc.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/platform_device_pm.h>
#endif
#include <linux/xgold_noc.h>

#include "dcc-core.h"
#include "dcc-sysfs.h"
#include "dcc-gra.h"
#include "dcc-hwregs.h"

#ifndef CONFIG_X86
	#define ioremap_wc ioremap
#endif


/* if FrameBuffer is in RGB32 888
 *  n    0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23
 * FB   R0 R1 R2 R3 R4 R5 R6 R7 G0 G1 G2 G3 G4 G5 G6 G7 B0 B1 B2 B3 B4 B5 B6 B7
 * ----------------------------------------------------------------------------
 * Din   0  1  2  3  4  5  6  7 |  8  9 10 11 12 13 14 15
 * LCD  G5 G6 G7 R3 R4 R5 R6 R7 | B3 B4 B5 B6 B7 G2 G3 G4
 *  n   13,14,15, 3, 4, 5, 6, 7,| 19,20,21,22,23,10,11,12
 */

int dcc_core_hwstop(struct dcc_drvdata *pdata)
{
	int ret = 0;
	gra_write_field(pdata, EXR_DIF_IMSC, 0); /* mask interrupt */
	gra_write_field(pdata, EXR_DIF_ICR, 0xfff); /* clear interrupt */
	gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_CONF);
	gra_write_field(pdata, EXR_DIF_CSREG_GRACMD, 0);
	gra_write_field(pdata, EXR_DIF_CLC, 0);

	return ret;
}

static void dcc_core_hwstart(struct dcc_drvdata *pdata)
{
	gra_write_field(pdata, EXR_DIF_CLC, (1 << 8));	/* set clock divider */
	dcc_hwreset(pdata);	/* set CONF mode (EXR_DIF_RUNCTRL = 0) */
#ifdef TEST
	gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_CONF);
	gra_write_field(pdata, EXR_DIF_CSREG_GRACMD, 0);
	gra_write_field(pdata, EXR_DIF_CON, 0x2);
#endif

	/**
	 * TX fifo must be configured AFTER the clock is enabled
	 * and BEFORE any access to the FIFO.
	 */

#define DIF_HWFIFO_FA_4WORD 2	/* TX fifo alignment size */
#define DIF_HWFIFO_BS_8WORD 3	/* TX fifo burst size */
#define DIF_HWFIFO_BS_4WORD 2	/* TX fifo burst size */

	gra_write_field(pdata, EXR_DIF_TXFIFOCFG_TXFA, DIF_HWFIFO_FA_4WORD);
	gra_write_field(pdata, EXR_DIF_TXFIFOCFG_TXBS, DIF_HWFIFO_BS_8WORD);

	gra_write_field(pdata, EXR_DIF_RXFIFOCFG_RXFA, DIF_HWFIFO_FA_4WORD);
	gra_write_field(pdata, EXR_DIF_RXFIFOCFG_RXBS, DIF_HWFIFO_BS_4WORD);
	gra_write_field(pdata, EXR_DIF_MRPS_CTRL, 4);
}

static int dcc_core_hwsetup(struct dcc_drvdata *pdata)
{
	int ret = 0;

	gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_RUN);
	gra_write_field(pdata, EXR_DIF_CSREG_GRACMD, 1);
	dcc_setbufferformat(pdata, dcc_get_fb_fmt(pdata));
	dcc_format_set_convertmatrix(pdata,
			DCC_FMT_ABGR8888, DCC_FMT_RGB565, 1);

	return ret;
}

int dcc_update_queue(struct dcc_drvdata *pdata, struct dcc_update *head,
			struct dcc_update_layers *updt, unsigned int tline)
{
	int ret = 0, new_entry = 0;
	struct dcc_update *update = list_first_entry_or_null(&head->list,
				struct dcc_update, list);

	if (!update) {
		update = (struct dcc_update *)
			devm_kzalloc(pdata->dev,
					sizeof(struct dcc_update), GFP_KERNEL);
		if (!update) {
			dcc_err("allocation of update item failed\n");
			return -EINVAL;
		}
		new_entry = 1;
	}

	/* if !autorefresh, try to manually fill xfer, else queue */
	if (!pdata->display_autorefresh) {
		if (!new_entry) {
			dcc_warn("update OFL: already queued ! --> framedrop\n");
			return -EBUSY;
		}
		if (!pdata->update_xfer) {
			update->timeline_expiracy = tline;
			pdata->timeline_current++;
			pdata->update_xfer = update;
			return 0;
		} else {
			dcc_warn("update OFL: queueing ! --> framedrop\n");
			return -EBUSY;
		}
	}

	/* fill update struct */
	memcpy(&update->updt, updt, sizeof(update->updt));
	update->timeline_expiracy = tline;
	DCC_DBG2("update queue tline:%d 0x%x --> %dx%d f:0x%x\n",
			update->timeline_expiracy, update->updt.back.phys,
			update->updt.back.src.w, update->updt.back.src.h,
			update->updt.flags);

	if (new_entry)
		list_add_tail(&update->list, &head->list);

	return ret;
}

int dcc_update_items(struct dcc_update *update)
{
	int i = 0;
	struct list_head *pos;
	struct list_head *n;
	list_for_each_safe(pos, n, &update->list)
		i++;

	return i;
}

struct dcc_update *dcc_update_dequeue(struct dcc_update *head)
{
	struct dcc_update *update = list_first_entry_or_null(&head->list,
				struct dcc_update, list);

	if (!update)
		return update;

	list_del(&update->list);
	return update;
}


static void vsync_wq(struct work_struct *ws)
{
	struct dcc_drvdata *p = m_to_dccdata(ws, vsync_work);
	DCC_DBG4("VSYNC wq ts:0x%llx\n", p->vsync_ts);
	p->vsync_us = measdelay_stop(NULL, &p->vsync_begin);
	measdelay_start(&p->vsync_begin);
#ifdef CONFIG_XGOLD_DCC_SYSFS
	sysfs_notify(&(p->dev->kobj), NULL, "vsyncts0");
#endif


}

#ifdef CONFIG_SW_SYNC_USER
int dcc_fence_update_timeline(struct sw_sync_timeline *timeline,
		int tcur, int texp)
{
	int delta = tcur - texp;
	if (delta != 1) {
		dcc_warn("update posted too fast texp:%d (cur:%d)\n",
				texp, tcur);
	}

	DCC_DBG2("FENCE: signal tline:%d\n", texp);
	if (timeline)
		sw_sync_timeline_inc(timeline, 1);
	return delta;
}
#endif

static void end_of_frame_wq(struct work_struct *ws)
{
	int i = 0;
	struct dcc_update *updt = NULL;
	struct dcc_drvdata *pdata = m_to_dccdata(ws, eof_work);

	down(&pdata->update_sem);

	/* XFER --> LAST */
	if (pdata->update_xfer) {

		/* release LAST */
		if (pdata->update_last) {
#ifdef CONFIG_SW_SYNC_USER
			dcc_fence_update_timeline(
					pdata->timeline,
					pdata->timeline_current,
					pdata->update_last->timeline_expiracy);
#endif
			devm_kfree(pdata->dev, pdata->update_last);
		}
		pdata->update_last = pdata->update_xfer;
		pdata->update_xfer = NULL;
	}

	i = dcc_update_items(&pdata->update_head);
	if (i > 1)
		dcc_warn("update queue = %d items\n", i);

	/* QUEUED --> XFER */
	updt = dcc_update_dequeue(&pdata->update_head);
	if (updt) {
		pdata->update_xfer = updt;
		pdata->timeline_current++;

		DCC_DBG2("update dequeue t:%d 0x%x\n",
				updt->timeline_expiracy,
				updt->updt.back.phys);
	}

	up(&pdata->update_sem);
}

static inline int dcc_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	int ret = 0;
	struct dcc_drvdata *pdata = dev_get_drvdata(dev);

	if (!IS_ERR(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dcc_err("%d:could not set pins\n", __LINE__);
	}
	return ret;
}


extern void xgold_noc_qos_set(char *name);
int dcc_core_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct dcc_drvdata *pdata =
		(struct dcc_drvdata *)platform_get_drvdata(pdev);

	if (!dev)
		return -EINVAL;

	init_completion(&pdata->sync.dsifin);
	pdata->sync.dsifin_to = 200;
	init_completion(&pdata->sync.dsitr1);
	pdata->sync.dsitr1_to = 80;
	init_completion(&pdata->sync.eoc);
	pdata->sync.eoc_to = 80;
	init_completion(&pdata->sync.eof);
	pdata->sync.eof_to = 80;

	INIT_WORK(&pdata->vsync_work, vsync_wq);
	INIT_WORK(&pdata->eof_work, end_of_frame_wq);

	/* Create workqueues */
	pdata->vsync_wq = alloc_ordered_workqueue(
		"vsync_wq", WQ_NON_REENTRANT | WQ_HIGHPRI);
	pdata->eof_wq = alloc_ordered_workqueue(
		"eof_wq", WQ_NON_REENTRANT | WQ_HIGHPRI);
	pdata->acq_wq = alloc_ordered_workqueue(
		"acq_wq", WQ_NON_REENTRANT | WQ_HIGHPRI);

	/* initialize some data fields */
	pdata->overlay_status = 0;
	pdata->debug.showfps = 0;
	pdata->debug.update_delay_max = 0;
	pdata->debug.cmd_delay_max = 0;
	pdata->debug.cmd_delay_last = 0;
	pdata->debug.fifowait = 0;
	pdata->debug.fifomaxlevel = 0;
	pdata->debug.frame_update_number = 0;
	pdata->dev = dev;

	/* gra driver information */
	dcc_boot_dbg("kernel frequency is %d MHz\n", pdata->clk_rate/1000/1000);

	/* Map dedicated memory */
	if (pdata->mem.pbase) {
		pdata->mem.vbase = ioremap_wc(pdata->mem.pbase,
						pdata->mem.size);
		if (pdata->mem.vbase == NULL) {
			dcc_err("failed to remap mem [0x%08x-0x%08x]\n",
					pdata->mem.pbase, pdata->mem.size);
		    goto exit;
		}
	} else {
		pdata->mem.vbase = dma_alloc_coherent(dev, pdata->mem.size,
				&pdata->mem.pbase, GFP_KERNEL);
	}
	dcc_boot_dbg("dedicated memory [0x%08x-0x%08x] remapped to 0x%08x\n",
			pdata->mem.pbase, pdata->mem.size,
			(unsigned)pdata->mem.vbase);

	/* setup hardware: DCC*/
	dcc_core_hwstart(pdata);
	/* Read DCC revision number */
	gra_read_field(pdata, EXR_DIF_ID_NUMBER, &pdata->id);

	/* install interrupts */
	ret = dcc_hal_probe(pdata);
	if (ret != 0) {
		dcc_err("Error during HAL initialization\n");
		goto exit_unmap;
	}

	/* Initialize display */
	pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pdata->pinctrl))
		goto exit_unmap;

	pdata->pins_default = pinctrl_lookup_state(pdata->pinctrl,
			PINCTRL_STATE_DEFAULT);
	if (IS_ERR(pdata->pins_default))
		dcc_err("could not get default pinstate\n");

	pdata->pins_sleep = pinctrl_lookup_state(pdata->pinctrl,
			PINCTRL_STATE_SLEEP);
	if (IS_ERR(pdata->pins_sleep))
		dcc_err("could not get sleep pinstate\n");

	pdata->pins_inactive = pinctrl_lookup_state(pdata->pinctrl,
			"inactive");
	if (IS_ERR(pdata->pins_inactive))
		dcc_err("could not get inactive pinstate\n");

	dcc_set_pinctrl_state(&pdev->dev, pdata->pins_default);

	/* Initialize interrupts */
	ret = dcc_interrupt_setup(pdata);
	if (ret != 0) {
		dcc_err("Error during Interrupt initialization\n");
		goto exit;
	}

	dcc_display_setup(pdata);
	dcc_boot_info("Display device %dx%d\n",
			pdata->display.xres, pdata->display.yres);
	dcc_core_hwsetup(pdata);

	dcc_boot_info("HWID 0x%x / DCC@%d MHz / %dMB RAM [0x%08x->0x%08x]\n",
			pdata->id, pdata->clk_rate/1000/1000,
			pdata->mem.size/1024/1024,
			pdata->mem.pbase,
			(unsigned)pdata->mem.vbase);

	pdata->overlay_nbr = 4;

#ifdef CONFIG_XGOLD_DCC_SYSFS
	dcc_sysfs_create(dev);
#endif

	return ret;

exit_unmap:
	dma_free_coherent(dev, pdata->mem.size,
			pdata->mem.vbase, pdata->mem.pbase);
	release_mem_region((unsigned int)pdata->mem.pbase,
			   (unsigned int)pdata->mem.size);
exit:
	dcc_err("FAIL: Display device registration\n");
	return ret;
}


int dcc_core_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct dcc_drvdata *pdata;

	if (!pdev)
		return -ENODEV;

	pdata = (struct dcc_drvdata *)platform_get_drvdata(pdev);
	if (!pdata)
		return -EINVAL;

	/* Destroy workqueues */
	flush_workqueue(pdata->vsync_wq);
	destroy_workqueue(pdata->vsync_wq);

	flush_workqueue(pdata->acq_wq);
	destroy_workqueue(pdata->acq_wq);

	flush_workqueue(pdata->eof_wq);
	destroy_workqueue(pdata->eof_wq);

	ret = dcc_core_hwstop(pdata);
	if (ret != 0)
		dcc_err("Error during HAL initialization\n");

	dcc_set_pinctrl_state(&pdev->dev, pdata->pins_inactive);

	ret = dcc_hal_remove(pdata);
	if (ret != 0)
		dcc_err("Error during HAL initialization\n");

#ifdef CONFIG_XGOLD_DCC_SYSFS
	dcc_sysfs_delete(&pdev->dev);
#endif

	/* release registers memory */
	iounmap(pdata->reg.vbase);
	release_mem_region((unsigned int)pdata->reg.pbase,
			   (unsigned int)pdata->reg.size);
	pdata->reg.vbase = NULL;

	/* release video memory */
	dma_free_coherent(&pdev->dev, pdata->mem.size,
			pdata->mem.vbase, pdata->mem.pbase);
	pdata->mem.vbase = NULL;

	return ret;
}

#ifdef CONFIG_PM
int dcc_core_suspend(struct platform_device *pdev)
{
	int ret;
	struct dcc_drvdata *pdata =
		(struct dcc_drvdata *)platform_get_drvdata(pdev);

	flush_workqueue(pdata->acq_wq);
	flush_workqueue(pdata->vsync_wq);
	flush_workqueue(pdata->eof_wq);

	/* suspend display */
	ret = dcc_display_suspend(pdata);
	if (ret != 0)
		dcc_err("Error during display suspend\n");

	ret = dcc_core_hwstop(pdata);

	dcc_set_pinctrl_state(&pdev->dev, pdata->pins_sleep);

	return ret;
}

int dcc_core_resume(struct platform_device *pdev)
{
	int ret = 0;
	struct dcc_drvdata *pdata;

	if (!pdev)
		return -ENODEV;

	pdata =	(struct dcc_drvdata *)platform_get_drvdata(pdev);
	if (!pdata)
		return -EINVAL;

	/* start DCC ip */
	dcc_set_pinctrl_state(&pdev->dev, pdata->pins_default);
	dcc_core_hwstart(pdata);

	gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_RUN);
	xgold_noc_qos_set("DCC2");
	gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_CONF);
	/* Initialize interrupts */
	ret = dcc_interrupt_setup(pdata);
	if (ret != 0)
		dcc_err("Error during Interrupt initialization\n");

	/* resume display */
	ret = dcc_display_setup(pdata);
	if (ret != 0)
		dcc_err("Error during display resume\n");

	dcc_core_hwsetup(pdata);

	return ret;
}
#endif /* CONFIG_PM */

long long dcc_timeval_diff(struct timeval *difference,
			   struct timeval *start_time, struct timeval *end_time)
{
	struct timeval temp_diff;

	if (difference == NULL)
		difference = &temp_diff;

	difference->tv_sec = end_time->tv_sec - start_time->tv_sec;
	difference->tv_usec = end_time->tv_usec - start_time->tv_usec;

	/* Using while instead of if below
	 * makes the code slightly more robust. */

	while (difference->tv_usec < 0) {
		difference->tv_usec += 1000000;
		difference->tv_sec -= 1;
	}

	return 1000000LL * difference->tv_sec + difference->tv_usec;

} /* dcc_timeval_diff() */
