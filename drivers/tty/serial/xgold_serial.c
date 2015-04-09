/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
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

#include <linux/slab.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/device_state_pm.h>


#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/dma-mapping.h>
#include <linux/serial_core.h>
#include <linux/platform_data/serial_xgold.h>
#include "xgold_usif.h"

#ifdef CONFIG_PM_RUNTIME
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/console.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#endif

#define RUNTIME_SUSPEND_DELAY		5000 /* ms */

#define USIF_FCI_POLL			msecs_to_jiffies(50)
#define RESCHEDULE_WQ_DELAY		msecs_to_jiffies(50)

#define XGOLD_USIF_ENTER pr_debug("--> %s\n", __func__);
#define XGOLD_USIF_EXIT pr_debug("<-- %s\n", __func__);

#define DECLARE_BAUD_RATE(A, B, C, D, E) { \
	.baud = A,\
	.dec = B,\
	.inc = C,\
	.bc = D,\
	.tmo = E\
	}

/*
 * Baud rate
 */
#define USIF_BAUD_MIN                   (9600)
#define USIF_BAUD_DEFAULT               (115200)

#define USIF_PIO_FIFO_SETUP (USIF_FIFO_CFG_RXFC_RXNFC \
			| USIF_FIFO_CFG_RXFA_RXFA4 | USIF_FIFO_CFG_RXBS_RXBS8 \
			| USIF_FIFO_CFG_TXFC_TXFC | USIF_FIFO_CFG_TXFA_TXFA1 \
			| USIF_FIFO_CFG_TXBS_TXBS8)

#define USIF_DMA_FIFO_SETUP (USIF_FIFO_CFG_RXFC_RXFC \
			| USIF_FIFO_CFG_RXFA_RXFA1 | USIF_FIFO_CFG_RXBS_RXBS4 \
			| USIF_FIFO_CFG_TXFC_TXFC | USIF_FIFO_CFG_TXFA_TXFA1 \
			| USIF_FIFO_CFG_TXBS_TXBS4)

#define USIF_IDI_FIFO_SETUP (USIF_FIFO_CFG_RXFC_RXFC \
			| USIF_FIFO_CFG_RXFA_RXFA1 | USIF_FIFO_CFG_RXBS_RXBS4 \
			| USIF_FIFO_CFG_TXFC_TXFC | USIF_FIFO_CFG_TXFA_TXFA1 \
			| USIF_FIFO_CFG_TXBS_TXBS4)

#define USIF_RX_ERR   (USIF_MIS_PE_MASK | USIF_MIS_FE_MASK \
			 | USIF_MIS_RXOF_MASK)

#define USIF_MIS_RECEIVE   (USIF_MIS_RX_BREQ_MASK | USIF_MIS_RX_LBREQ_MASK \
			| USIF_MIS_RX_SREQ_MASK | USIF_MIS_RX_LSREQ_MASK)

#define USIF_STA_INT    (USIF_MIS_TMO_MASK | USIF_MIS_FRM_PAUSE_MASK \
			| USIF_MIS_TX_FIN_MASK | USIF_MIS_RI_MASK \
			| USIF_MIS_DCD_MASK | USIF_MIS_DSR_MASK \
			| USIF_MIS_DTR_MASK | USIF_MIS_FCI_MASK)

#define USIF_ERR_INT    (USIF_MIS_MC_MASK | USIF_MIS_SLIP_MASK \
			| USIF_MIS_CRC_MASK | USIF_MIS_PHE_MASK \
			| USIF_MIS_FE_MASK | USIF_MIS_PE_MASK \
			| USIF_MIS_TXOF_MASK | USIF_MIS_TXUR_MASK \
			| USIF_MIS_RXUR_MASK)

#define USIF_MIS_TRANSMIT   (USIF_MIS_TX_BREQ_MASK | USIF_MIS_TX_LBREQ_MASK \
			| USIF_MIS_TX_SREQ_MASK | USIF_MIS_TX_LSREQ_MASK)

#define USIF_MIS_CLR_ALL                0xFFBFFBFF
#define USIF_MIS_CLR_ALL_TX		0xFFBFFBF0

#define USIF_IMSC_TX_RX_CONFIG	 (USIF_MIS_TRANSMIT | USIF_MIS_RECEIVE)

static int xgold_usif_write_tps(struct uart_port *port, unsigned cnt);

static void xgold_usif_dma_init(struct uart_usif_xgold_port *uxp);

static void xgold_usif_dma_rx_request(struct uart_usif_xgold_port *uxp);

static void xgold_usif_dma_rx_call_back(void *param);

static void xgold_usif_dma_tx_tasklet_func(unsigned long data);

static int usif_port_is_console(struct uart_port *port)
{
#ifdef CONFIG_SERIAL_XGOLD_CONSOLE
	return port->cons != NULL;
#else
	return 0;
#endif
}

static void xgold_usif_error(struct uart_port *port, int line,
				const char *function)
{
	struct uart_usif_xgold_port *uxp = to_usif_port(port);

	if (uxp->use_dma) {
		/* Safe to print error if the device is not the console */
		dev_err(port->dev, "%d:%s: unrecoverable error!\n",
					line, function);
	}
}

static void xgold_usif_clc_error(struct uart_port *port, int line,
				const char *function)
{
	if (!usif_port_is_console(port)) {
		/* Safe to print error if the device is not the console */
		dev_err(port->dev, "%d:%s: error when setting CLC register\n",
					line, function);
	}
}

static void usif_set_running_mode(struct uart_port *port)
{
	if (!USIF_CLC_STAT_MODEN(ioread32(USIF_CLC_STAT(port->membase)))) {
		xgold_usif_clc_error(port, __LINE__, __func__);
		return;
	}
	if (!USIF_CLC_STAT_RUN(ioread32(USIF_CLC_STAT(port->membase)))) {
		unsigned int i = 0;
		/* Switch to USIF working mode */
		iowrite32(USIF_CLC_RUN_RUN, USIF_CLC(port->membase));
		while (!USIF_CLC_STAT_RUN(
				ioread32(USIF_CLC_STAT(port->membase)))) {
			udelay(1);
			if (i++ > 10000) {
				/* It can take several 100000 clock cycles
				 * to write the CLC register.
				 * If the write was still not successful after
				 * 10ms we can assume something went wrong. */
				BUG();
			}
		}
	}
}

static void usif_set_config_mode(struct uart_port *port)
{
	if (!USIF_CLC_STAT_MODEN(ioread32(USIF_CLC_STAT(port->membase)))) {
		xgold_usif_clc_error(port, __LINE__, __func__);
		return;
	}

	if (USIF_CLC_STAT_RUN(ioread32(USIF_CLC_STAT(port->membase)))) {
		unsigned int i = 0;
		/* Switch to USIF configuration mode */
		iowrite32(USIF_CLC_RUN_STOP, USIF_CLC(port->membase));

		while (USIF_CLC_STAT_RUN(
				ioread32(USIF_CLC_STAT(port->membase)))) {
			udelay(1);
			if (i++ > 10000) {
				/* It can take several 100000 clock cycles
				 * to write the CLC register.
				 * If the write was still not successful after
				 * 10ms we can assume something went wrong. */
				BUG();
			}
		}
	}
}

static void usif_set_disabled_mode(struct uart_port *port)
{
	if (USIF_CLC_STAT_MODEN(ioread32(USIF_CLC_STAT(port->membase)))) {

		unsigned int i = 0;

		/* Switch to USIF disabled mode */
		iowrite32(USIF_CLC_MOD_EN_DIS_REQ, USIF_CLC(port->membase));

		while (USIF_CLC_STAT_MODEN(
				ioread32(USIF_CLC_STAT(port->membase)))) {
			udelay(1);
			if (i++ > 10000) {
				/* It can take several 100000 clock cycles
				 * to write the CLC register.
				 * If the write was still not successful after
				 * 10ms we can assume something went wrong. */
				BUG();
			}
		}
	}
}

static void usif_set_enabled_mode(struct uart_port *port)
{
	if (!USIF_CLC_STAT_MODEN(ioread32(USIF_CLC_STAT(port->membase)))) {

		unsigned int i = 0;

		/* Switch to USIF enabled mode */
		iowrite32(USIF_CLC_MOD_EN_EN_REQ, USIF_CLC(port->membase));

		while (!USIF_CLC_STAT_MODEN(
				ioread32(USIF_CLC_STAT(port->membase)))) {
			udelay(1);
			if (i++ > 10000) {
				/* It can take several 100000 clock cycles
				 * to write the CLC register.
				 * If the write was still not successful after
				 * 10ms we can assume something went wrong. */
				BUG();
			}
		}
	}
}

static void xgold_usif_runtime_pm_resume(struct uart_port *port)
{
#ifdef CONFIG_PM_RUNTIME
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);
	struct uart_usif_xgold_port *uxp = to_usif_port(port);

	if (platdata->runtime_pm_enabled) {
		if (platdata->rpm_auto_suspend_enable &&
			(pm_runtime_enabled(port->dev))) {
			int i = 0;

			mutex_lock(&uxp->runtime_lock);

			if (port->dev->power.runtime_status ==
					RPM_SUSPENDING) {
				/* Wait until we are suspended */
				while (port->dev->power.runtime_status !=
						RPM_SUSPENDED) {
					udelay(1);
					i++;
					if (i > 500000) {
						dev_err(port->dev,
						 "Deadlock in RPM resume %d\n",
							__LINE__);
						BUG();
					}
				}
			} else if (port->dev->power.runtime_status ==
						RPM_RESUMING) {
				/* Wait until we are resumed */
				while (port->dev->power.runtime_status !=
						RPM_ACTIVE) {
					udelay(1);
					i++;
					if (i > 500000) {
						dev_err(port->dev,
						 "Deadlock in RPM resume %d\n",
							__LINE__);
						BUG();
					}
				}
			}
			pm_runtime_get_sync(port->dev);
			mutex_unlock(&uxp->runtime_lock);
			return;
		}

		if (port->state->port.count > 0 &&
			pm_runtime_enabled(port->dev) &&
			port->dev->power.runtime_status != RPM_ACTIVE)
			/* We have called a port ops function while being
			 * runtime suspended. This is not allowed.
			 */
			BUG();
	}
#endif
}

static void xgold_usif_runtime_pm_autosuspend(struct uart_port *port)
{
#ifdef CONFIG_PM_RUNTIME
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);

	if (platdata->runtime_pm_enabled &&
		platdata->rpm_auto_suspend_enable &&
		pm_runtime_enabled(port->dev)) {
		pm_runtime_mark_last_busy(port->dev);
		pm_runtime_put_autosuspend(port->dev);
	}
#endif
}

static int xgold_usif_runtime_pm_suspended(struct uart_port *port,
			const char *function)
{
#ifdef CONFIG_PM_RUNTIME
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);

	if (platdata->runtime_pm_enabled) {
		if (platdata->rpm_auto_suspend_enable &&
			pm_runtime_enabled(port->dev)) {
			struct uart_usif_xgold_port *uxp = to_usif_port(port);
			if (port->dev->power.runtime_status !=  RPM_ACTIVE) {
				dev_warn(port->dev,
					"%s called while suspended\n",
					function);
				if (!delayed_work_pending(
						&uxp->usif_rpm_work)) {
					dev_dbg(port->dev,
						"scheduling RPM work\n");
					schedule_delayed_work(
						&uxp->usif_rpm_work, 0);
				}
				return 1;
			} else {
				/* This call will only increase the usage_count
				 * for Runtime PM so we stay in sync. */
				pm_runtime_get_noresume(port->dev);
				return 0;
			}
		}

		if (port->state->port.count > 0 &&
			pm_runtime_enabled(port->dev)) {
			if (port->dev->power.runtime_status == RPM_RESUMING
			  || port->dev->power.runtime_status == RPM_SUSPENDING)
				/* Return 1 to signal that we are either
				 * resuming or suspending and thus should not
				 * access any port registers. */
				return 1;
			if (port->dev->power.runtime_status == RPM_SUSPENDED)
				/* We have called a port ops function while
				 *  being runtime suspended. This is not
				  * allowed.
				 */
				BUG();
		}
	}
#endif
	return 0;
}

static inline int xgold_usif_set_pinctrl_state(struct device *dev,
						struct pinctrl_state *state)
{
	int ret = 0;
	struct xgold_usif_platdata *platdata = dev_get_platdata(dev);

	if (!platdata) {
		dev_err(dev, "Unable to retrieve usif platform data\n");
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(platdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	return ret;
}


#ifdef CONFIG_SERIAL_XGOLD_CONSOLE
static struct uart_usif_xgold_port *xgold_usif_ports[SERIAL_USIF_NR];
#endif

static struct s_xgold_usif_baudrate_divider {
	int baud;
	unsigned dec;
	unsigned inc;
	unsigned bc;
	unsigned tmo;
} xgold_usif_baudrate_divider[4][20] = {
	{
/*
	96Mhz ...
*/
		DECLARE_BAUD_RATE(4000000, 0x1, 0x2, 0x0, 0x4000),
		DECLARE_BAUD_RATE(3500000, 0x5, 0x7, 0x0, 0x4000),
		DECLARE_BAUD_RATE(3000000, 0x1, 0x1, 0x0, 0x4000),
		DECLARE_BAUD_RATE(2500000, 0x7, 0x5, 0x0, 0x4000),
		DECLARE_BAUD_RATE(2000000, 0x2, 0x1, 0x0, 0x4000),
		DECLARE_BAUD_RATE(1500000, 0x3, 0x1, 0x0, 0x4000),
		DECLARE_BAUD_RATE(1152000, 0x65, 0x18, 0x0, 0x4000),
		DECLARE_BAUD_RATE(1000000, 0x5, 0x1, 0x0, 0x4000),
		DECLARE_BAUD_RATE(921600, 0x211, 0x060, 0x0, 0x4000),
		DECLARE_BAUD_RATE(576000, 0x71, 0x0C, 0x0, 0x4000),
		DECLARE_BAUD_RATE(500000, 0x011, 0x1, 0x0, 0x4000),
		DECLARE_BAUD_RATE(460800, 0x241, 0x030, 0x0, 0x4000),
		DECLARE_BAUD_RATE(230400, 0x229, 0x48, 0x2, 0x2000),
		DECLARE_BAUD_RATE(115200, 0x229, 0x48, 0x5, 0x1000),
		DECLARE_BAUD_RATE(57600, 0x229, 0x048, 0xB, 0x800),
		DECLARE_BAUD_RATE(38400, 0x229, 0x48, 0x11, 0x555),
		DECLARE_BAUD_RATE(19200, 0x229, 0x48, 0x23, 0x2AA),
		DECLARE_BAUD_RATE(9600, 0x229, 0x048, 0x47, 0x155),
	},
/*
	48 Mhz
*/
	{
		DECLARE_BAUD_RATE(460800, 0x211, 0x060, 0x0, 0x2000),
		DECLARE_BAUD_RATE(230400, 0x211, 0x60, 0x1, 0x1000),
		DECLARE_BAUD_RATE(115200, 0x229, 0x48, 0x2, 0x800),
		DECLARE_BAUD_RATE(57600, 0x229, 0x048, 0x5, 0x555),
		DECLARE_BAUD_RATE(38400, 0x229, 0x48, 0xB, 0x2AA),
	},
/*
	104 Mhz
*/
	{
/*
 * NOTE : there is no definition of B3250000 so we use B3500000 instead.
 * there is also a problem in the GNSS core
 * so when we do B4000000 setting we actually setting it for 6500000Mbps
 * and when we do B3500000, we actually set for 3250000
 */
		DECLARE_BAUD_RATE(4000000, 0x0, 0x0, 0x0, 0xFFFF),
		DECLARE_BAUD_RATE(3500000, 0x0, 0x0, 0x1, 0xFFFF),
		DECLARE_BAUD_RATE(3000000, 0x1, 0xC, 0x1, 0xFFFF),
		DECLARE_BAUD_RATE(921600, 0x0B29, 0x1D8, 0x0, 0x4000),
		DECLARE_BAUD_RATE(460800, 0x1C2, 0x14D, 0x5, 0x4000),
		DECLARE_BAUD_RATE(115200, 0x1d0, 0x37, 0x5, 0x800),
	},

/*
	26 Mhz
*/
	{
/* NOTE: added this option if we set the GNSS UART2 clock
 * to be driven from the 26Mhz clock. actually it is 1.625Mbps
 */
		DECLARE_BAUD_RATE(1500000, 0x0, 0x0, 0x0, 0x4000),
		DECLARE_BAUD_RATE(115200, 0x198, 0x12e, 0x5, 0x800),
	}

};

static void xgold_usif_stop_tx(struct uart_port *);
static bool xgold_usif_is_tx_ready(struct uart_port *port);

static void xgold_usif_clk_enable(struct uart_port *port)
{
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);
	unsigned reg;
	int ret;

	/*
	 * Note: device_state_pm_set_state_by_name must not be called
	 * from atomic context.
	 */
	if (platdata->pm_platdata) {
		ret = device_state_pm_set_state_by_name(port->dev,
				platdata->pm_platdata->pm_state_D0_name);
		if (ret)
			dev_err(port->dev, "%s PM set state return %d\n",
					__func__, ret);
	}

	if (USIF_CLC_STAT_RUN(ioread32(USIF_CLC_STAT(port->membase))))
		/* We are in run mode which means it's safe to assume that
		 * clocks are already enabled. Return with no further
		 * action */
		return;

	reg = (platdata->ormc << USIF_CLC_CNT_ORMCSMC_OFFSET) |
		(platdata->rmc << USIF_CLC_CNT_RMC_OFFSET);

	if (ioread32(USIF_CLC_CNT(port->membase)) != reg) {
		unsigned int i = 0;
		iowrite32(reg, USIF_CLC_CNT(port->membase));
		while (ioread32(USIF_CLC_CNT(port->membase)) != reg) {
			udelay(1);
			if (i++ > 10000) {
				/* It can take several 100000 clock cycles
				 * to write the CLC register.
				 * If the write was still not successful after
				 * 10ms we can assume something went wrong. */
				BUG();
			}
		}
	}
}

static void xgold_usif_clk_disable(struct uart_port *port)
{
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);

	/*
	 * Note: device_state_pm_set_state_by_name must not be called
	 * from atomic context.
	 */
	if (platdata->pm_platdata)
		device_state_pm_set_state_by_name(port->dev,
			platdata->pm_platdata->pm_state_D3_name);

}

static bool xgold_usif_is_clk_enabled(struct uart_port *port)
{
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);

	if (platdata->pm_platdata) {
		if (strcmp(port->dev->pm_data.cur_state->name,
			platdata->pm_platdata->pm_state_D3_name) == 0)
			return false;
		else
			return true;
	}
	return false;
}

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
int xgold_usif_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);
	struct uart_port *port = &uxp->port;
	struct xgold_usif_platdata *platdata = dev_get_platdata(dev);
	int id = device_state_pm_get_state_id(dev, state->name);
	unsigned reg;

	switch (id) {
	case USIF_PM_STATE_D0:
		if (platdata->regulator)
			regulator_enable(platdata->regulator);
		if (platdata->clk_reg)
			clk_prepare_enable(platdata->clk_reg);
		if (platdata->clk_kernel)
			clk_prepare_enable(platdata->clk_kernel);

		reg = (platdata->ormc << USIF_CLC_CNT_ORMCSMC_OFFSET) |
		    (platdata->rmc << USIF_CLC_CNT_RMC_OFFSET);

		if (ioread32(USIF_CLC_CNT(port->membase)) != reg)
			iowrite32(reg, USIF_CLC_CNT(port->membase));
		break;

	case USIF_PM_STATE_D3:
		if (platdata->clk_reg)
			clk_disable_unprepare(platdata->clk_reg);
		if (platdata->clk_kernel)
			clk_disable_unprepare(platdata->clk_kernel);
		if (platdata->regulator)
			regulator_disable(platdata->regulator);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

struct device_state_pm_state *xgold_usif_get_initial_state(
		struct device *dev)
{
	return &usif_pm_states[USIF_PM_STATE_D3];
}
#endif

static void xgold_usif_set_fifo_cfg(struct uart_port *port)
{
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	/*FIXME:spinlock called from interrupt */

	iowrite32(uxp->fifo_cfg, USIF_FIFO_CFG(port->membase));
	iowrite32(USIF_FIFO_CTRL_RX_AR_AR_ON, USIF_FIFO_CTRL(port->membase));
}

static inline unsigned int handle_receive_error(struct uart_port *port,
						char *flg)
{
	unsigned int error_state;
	/* The 'flg' is set to normal, if no error detected */
	*flg = TTY_NORMAL;

	/* USIF supports RX overflow error, parity error and frame error.
	 * It does not support the break error */

	error_state = (ioread32(USIF_MIS(port->membase)) & USIF_RX_ERR);

	if (!error_state)
		return error_state;

	/* Get the stats */
	if (USIF_MIS_RXOF(error_state))
		port->icount.overrun++;
	else if (USIF_MIS_PE(error_state))
		port->icount.parity++;
	else if (USIF_MIS_FE(error_state))
		port->icount.frame++;

	/* Clear the interrupt node */
	iowrite32(error_state, USIF_ICR(port->membase));

	/* Report the error to tty if have been asked to */
	error_state &= port->read_status_mask;
	if (error_state & USIF_MIS_PE_MASK)
		*flg = TTY_PARITY;
	else if (error_state & USIF_MIS_FE_MASK)
		*flg = TTY_FRAME;
	else if (error_state & USIF_MIS_RXOF_MASK)
		*flg = TTY_OVERRUN;

	return error_state;
}

static void
_xgold_usif_set_termios(struct uart_port *port, struct ktermios *termios,
		struct ktermios *old)
{
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);
	unsigned int old_int_mask, reg;
	unsigned int mode_cfg = 0;
	unsigned int prtc_cfg = 0;
	unsigned int int_mask = 0;
	unsigned int baud = 0;
	unsigned fdiv_cfg, fdiv_inc, fdiv_dec;
	unsigned int baud_bc_cfg;
	unsigned int ictmo_cfg;
	unsigned index_baud_clk;
	int i = 0, j;

/* Macro definations for switch configuration register RUN_CTRL */
/* Baud rate definition - default values are for 115200 at 104MHz */
#define USIF_FDIV_INC_DEFAULT           0x37
#define USIF_FDIV_DEC_DEFAULT           0x1D0
#define USIF_BC_CFG_BCRV_DEFAULT        0x005
#define USIF_RX_ICTMO_BAUD_115200       0x800
#define USIF_RX_ICTMO_BAUD_DEFAULT      (USIF_RX_ICTMO_BAUD_115200)

	/* FIXME: PM callback should have done it.
		Should we call enable_mode ?
	xgold_usif_clk_enable(port);
	usif_set_enabled_mode(port);
	*/
	usif_set_config_mode(port);

	/* Read Hw information */
	uxp->fifo_hwid = ioread32(USIF_FIFO_ID(port->membase));
	uxp->hwid = ioread32(USIF_ID(port->membase));

	/* Set the default baud rate configuration, as 115200 */
	fdiv_inc = USIF_FDIV_INC_DEFAULT;
	fdiv_dec = USIF_FDIV_DEC_DEFAULT;
	baud_bc_cfg = USIF_BC_CFG_BCRV_DEFAULT;

	/* Set the default receive inter-character timeout, for 115200 */
	ictmo_cfg = USIF_RX_ICTMO_BAUD_DEFAULT;
	/* Get the numric baudrate */

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 16);

	/* If the baud rate is below 9600, default it to 9600 */
	if (baud < USIF_BAUD_MIN)
		baud = USIF_BAUD_MIN;
	switch (port->uartclk) {
	case 96000000:
		index_baud_clk = 0;
		break;
	case 48000000:
		index_baud_clk = 1;
		break;
	case 104000000:
		index_baud_clk = 2;
		break;
	case 26000000:
		index_baud_clk = 3;
		break;

	default:
		index_baud_clk = 0;
	}
	for (i = 0; i < ARRAY_SIZE(xgold_usif_baudrate_divider[index_baud_clk]);
			i++) {
		if (xgold_usif_baudrate_divider[index_baud_clk][i].baud != baud)
			continue;

		fdiv_inc = xgold_usif_baudrate_divider[index_baud_clk][i].inc;
		fdiv_dec = xgold_usif_baudrate_divider[index_baud_clk][i].dec;
		baud_bc_cfg = xgold_usif_baudrate_divider[index_baud_clk][i].bc;
		ictmo_cfg = xgold_usif_baudrate_divider[index_baud_clk][i].tmo;

		if (!platdata->ictmo)
			break;

		/* Overwrite default ictmo if one is set in platdata for this
		 * baud rate. */
		j = 0;
		while (platdata->ictmo[j]) {
			/* (j) is baud rate
			 * (j + 1) is associated ictmo_cfg value */
			ictmo_cfg = (baud == platdata->ictmo[j]) ?
				platdata->ictmo[j + 1] : ictmo_cfg;
			j += 2;
		}

		break;
	}

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		prtc_cfg |= (5 << USIF_PRTC_CFG_CLEN_OFFSET);
		break;
	case CS6:
		prtc_cfg |= (6 << USIF_PRTC_CFG_CLEN_OFFSET);
		break;
	case CS7:
		prtc_cfg |= (7 << USIF_PRTC_CFG_CLEN_OFFSET);
		break;
	case CS8:
	default:
		prtc_cfg |= (8 << USIF_PRTC_CFG_CLEN_OFFSET);
		break;
	}

	mode_cfg |= (USIF_MODE_CFG_TXEN_EN | USIF_MODE_CFG_MA_MASTER);

	/* Ignore all received characters if CREAD is not set */
	if ((termios->c_cflag & CREAD))
		mode_cfg |= USIF_MODE_CFG_RXEN_EN;

	/* Use two stop bits */
	if (termios->c_cflag & CSTOPB)
		prtc_cfg |= USIF_PRTC_CFG_STP2_MASK;

	if (termios->c_cflag & PARENB) {
		/* Enable parity, by default even parity */
		prtc_cfg |= USIF_PRTC_CFG_PAR_MASK;
		/* Set odd parity */
		if (termios->c_cflag & PARODD)
			prtc_cfg |= USIF_PRTC_CFG_ODD_MASK;
	}

	prtc_cfg |=
		((USIF_FIFO_ID_RX_STAGE(uxp->fifo_hwid)) <<
		 USIF_PRTC_CFG_FCOTL_OFFSET);

	/* Update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	/* Update the read_status_mask and ignore_status_mask */
	/* RX overflow/overrun error */
	port->read_status_mask = USIF_MIS_RXOF_MASK;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= USIF_MIS_FE_MASK | USIF_MIS_PE_MASK;
	/* USIF does not support the break interrupt error */
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= port->read_status_mask;
	/* Characters to ignore */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= USIF_MIS_FE_MASK | USIF_MIS_PE_MASK;

	if (termios->c_iflag & IGNBRK) {
		/* If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support) */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= USIF_MIS_RXOF_MASK;
	}

	old_int_mask = ioread32(USIF_IMSC(port->membase));

	/* Disable all interrupts */
	iowrite32(0, USIF_IMSC(port->membase));

	iowrite32(USIF_MIS_CLR_ALL, USIF_ICR(port->membase));

	/* Update the mode configuration register with the settings */
	iowrite32(mode_cfg, USIF_MODE_CFG(port->membase));

	/* Update the protocol configuration register with the settings */
	iowrite32(prtc_cfg, USIF_PRTC_CFG(port->membase));

	/* Update the fractional devider configuration register for baudrate */
	fdiv_cfg = (fdiv_dec << USIF_FDIV_CFG_DEC_OFFSET);
	fdiv_cfg |= (fdiv_inc << USIF_FDIV_CFG_INC_OFFSET);
	iowrite32(fdiv_cfg, USIF_FDIV_CFG(port->membase));

	/* Update the baudrate counter configuration register for baudrate */
	iowrite32(baud_bc_cfg, USIF_BC_CFG(port->membase));

	/* Update time out Intercharacter */
	iowrite32(ictmo_cfg, USIF_ICTMO_CFG(port->membase));

	/* Configure the FIFO */
	xgold_usif_set_fifo_cfg(port);

	int_mask = old_int_mask;

	/* Enable the CTS interrupt, and hw control of the transmission */
	/* We care about only CTS interrupt, now */
	if (termios->c_cflag & CRTSCTS) {
		int_mask |= USIF_MIS_FCI_MASK;
		reg = USIF_MSS_CTRL_FCOEN_FCOEN | USIF_MSS_CTRL_FCIEN_FCIEN;
		iowrite32(reg, USIF_MSS_CTRL(port->membase));

	} else {
		int_mask &= ~(USIF_MIS_FCI_MASK);
		reg = USIF_MSS_CTRL_FCOEN_FCODIS | USIF_MSS_CTRL_FCIEN_FCIDIS;
		iowrite32(reg, USIF_MSS_CTRL(port->membase));
	}

	/* We need to take care the error interrupt */
	int_mask |= USIF_ERR_INT;

	/* We need to get the end of transmission interrupt */
	int_mask |= USIF_MIS_TX_FIN_MASK;

	/* Switch to USIF working mode */
	usif_set_running_mode(port);

	/* Set the mask for interrupts */
	iowrite32(USIF_MIS_CLR_ALL, USIF_ICR(port->membase));
	iowrite32(int_mask, USIF_IMSC(port->membase));

	/* FIXME: PM callback should do the job..Disable clocks
	xgold_usif_clk_disable(port);
	*/

	dev_dbg(port->dev, " Setting baud rate as %d , uart clk as %d Hz\n",
		baud, port->uartclk);

	dev_dbg(port->dev, " Setting termios:\n"
		"\t c_iflag:%#x, c_oflag:%#x, c_cflag:%#x, c_lflag:%#x\n"
		"\t c_line:%#x, c_ispeed:%#x, c_ospeed:%#x\n",
		termios->c_iflag, termios->c_oflag,
		termios->c_cflag, termios->c_lflag,
		termios->c_line, termios->c_ispeed, termios->c_ospeed);

	dev_dbg(port->dev, " inc: %d, dec: %d, bc: %d, tmo: %d\n",
		fdiv_inc, fdiv_dec, baud_bc_cfg, ictmo_cfg);

	return;
}

static void
xgold_usif_set_termios(struct uart_port *port, struct ktermios *termios,
		struct ktermios *old)
{
	unsigned long flags;

	xgold_usif_runtime_pm_resume(port);

	spin_lock_irqsave(&port->lock, flags);
	_xgold_usif_set_termios(port, termios, old);
	spin_unlock_irqrestore(&port->lock, flags);

	xgold_usif_runtime_pm_autosuspend(port);
}

static const char *xgold_usif_type(struct uart_port *port)
{
	return port->type == PORT_XGOLD_USIF ? "XGOLD_USIF" : NULL;
}

static void xgold_usif_release_port(struct uart_port *port)
{
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);

	release_mem_region(port->mapbase, resource_size(&platdata->res_io));
}

static int xgold_usif_request_port(struct uart_port *port)
{
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);
	struct resource *res;

	res = request_mem_region(port->mapbase,
			resource_size(&platdata->res_io), "xgold-usif-serial");
	if (res == NULL)
		return -EBUSY;
	return 0;
}

static void xgold_usif_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_XGOLD_USIF;
		xgold_usif_request_port(port);
	}
}

static int xgold_usif_verify_port(struct uart_port *port,
		struct serial_struct *ser)
{
	int ret = 0;
	if ((ser->type != PORT_UNKNOWN) && (ser->type != PORT_XGOLD_USIF))
		ret = -EINVAL;
	if ((ser->irq < 0) || (ser->irq >= NR_IRQS))
		ret = -EINVAL;
	if ((ser->baud_base) < (USIF_BAUD_MIN))
		ret = -EINVAL;

	return ret;
}

#define USIF_PRINT_IRQ(_X_) \
{\
	if (USIF_MIS_##_X_(status) && \
			(uxp->flags & XGOLD_USIF_ALLOW_DEBUG)) \
		dev_dbg(port->dev, "%s: "#_X_" detected\n", __func__); \
}

/*
 * PIO Operations
 */

static inline void receive_chars_req(struct uart_port *port)
{
	struct tty_port *tty_port = &port->state->port;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	char flg;
	unsigned int char_cnt;
	unsigned int rsr, bytes_per_stage;
	/* Structure for receive word */
	union usif_rxd {
		struct {
			unsigned char byte[4];
		} rxd_byte;
		unsigned int rxd_word;
	} rxd_data;

#define RX_STAGE_SIZE 4
	bytes_per_stage =
		(RX_STAGE_SIZE / (1 << USIF_FIFO_CFG_RXFA(uxp->fifo_cfg)));
	rxd_data.rxd_word = ioread32(USIF_RXD(port->membase));
	rsr = handle_receive_error(port, &flg);
	for (char_cnt = 0; char_cnt < bytes_per_stage; char_cnt++) {
		port->icount.rx++;
		if (!uart_handle_sysrq_char(port,
					rxd_data.rxd_byte.byte[char_cnt])) {
			uart_insert_char(port, rsr, USIF_MIS_RXOF_INT_PEND,
					 (rxd_data.rxd_byte.byte[char_cnt]),
					 flg);
		}
	}
	tty_flip_buffer_push(tty_port);
}

static inline void receive_chars_pio(struct uart_port *port,
		unsigned int status)
{
	unsigned int i;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	if (USIF_MIS_RX_SREQ(status) || USIF_MIS_RX_LSREQ(status)) {
		receive_chars_req(port);
	} else {
		for (i = 0; i < (1UL << USIF_FIFO_CFG_RXBS(uxp->fifo_cfg)); i++)
			receive_chars_req(port);
	}
	iowrite32(status, USIF_ICR(port->membase));
}

static irqreturn_t receive_chars(struct uart_port *port, unsigned status)
{
	receive_chars_pio(port, status);
	return IRQ_HANDLED;
}

static void transmit_chars_pio(struct uart_port *port, unsigned int status)
{
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	int i, bytes_per_stage, payload = 0;

	/* Service the transmit request, depending on burst type */
	if (USIF_MIS_TX_LBREQ(status) || USIF_MIS_TX_BREQ(status))
		payload = (1UL << USIF_FIFO_CFG_TXBS(uxp->fifo_cfg));
	else if (USIF_MIS_TX_SREQ(status) || USIF_MIS_TX_LSREQ(status))
		payload = 1;

	if (!payload) {
		if (!usif_port_is_console(port))
			dev_err(port->dev, "No Valid burst found !\n");
		return;
	}

#define TX_STAGE_SIZE 4
	bytes_per_stage =
		(TX_STAGE_SIZE / (1 << USIF_FIFO_CFG_TXFA(uxp->fifo_cfg)));
	for (i = 0; i < payload; i++) {
		struct circ_buf *xmit = &port->state->xmit;
		/* Structure for transmit word */
		union usif_txd {
			struct {
				unsigned char byte[4];
			} txd_byte;
			unsigned int txd_word;
		} txd_data;

		unsigned int j;
		txd_data.txd_word = 0;
		/* Write to the TX fifo according to its alignment */
		for (j = 0; j < bytes_per_stage; j++) {
			if (!uxp->tx_tps_cnt)
				break;
			txd_data.txd_byte.byte[j] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			port->icount.tx++;
			uxp->tx_tps_cnt--;
		}
		iowrite32(txd_data.txd_word, USIF_TXD(port->membase));
	}

	if (uart_tx_stopped(port)) {
		unsigned reg;
		/* FIXME: spinlock */
		reg = ioread32(USIF_IMSC(port->membase));
		reg &= ~USIF_MIS_TRANSMIT;
		iowrite32(reg, USIF_IMSC(port->membase));
	}

	iowrite32(status, USIF_ICR(port->membase));
	return;
}

static irqreturn_t transmit_chars(struct uart_port *port, unsigned status)
{
	transmit_chars_pio(port, status);
	return IRQ_HANDLED;
}

static irqreturn_t handle_err_interrupt(struct uart_port *port, unsigned status)
{
	/* TODO: Error recovery */
	iowrite32(status, USIF_ICR(port->membase));

	return IRQ_HANDLED;
}

void xgold_usif_delete_modem_poll(struct uart_port *port)
{
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	unsigned modem_status, old_fci_stat;
	unsigned imsc;

	if (del_timer(&uxp->modem_poll)) {
		modem_status = USIF_MSS_STAT_FCI(
			ioread32(USIF_MSS_STAT(port->membase)));
		old_fci_stat = uxp->modem_status[1];

		pr_debug("%s: FCI %x\n", __func__, modem_status);
		/* Not really sure it's correct to clear the FCI */
		iowrite32(USIF_ICR_FCI_MASK, USIF_ICR(port->membase));
		imsc = ioread32(USIF_IMSC(port->membase));
		imsc |= USIF_IMSC_FCI_MASK;
		iowrite32(imsc, USIF_IMSC(port->membase));

		if (modem_status > old_fci_stat) {
			pr_debug("%s: FCI event 0->1 while timer is deleted\n",
					__func__);
			uart_handle_cts_change(port, modem_status);
			wake_up_interruptible(
					&port->state->port.delta_msr_wait);
		}
	}
}

static irqreturn_t handle_sta_interrupt(struct uart_port *port, unsigned status)
{
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int imsc;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);

	USIF_PRINT_IRQ(FCI)
	USIF_PRINT_IRQ(TX_FIN)
	USIF_PRINT_IRQ(TMO)
	USIF_PRINT_IRQ(FRM_PAUSE)
	USIF_PRINT_IRQ(RI)
	USIF_PRINT_IRQ(DCD)
	USIF_PRINT_IRQ(DSR)
	USIF_PRINT_IRQ(DTR)
	USIF_PRINT_IRQ(FCO)

	if (USIF_MIS_FCI(status)) {
		unsigned modem_status = USIF_MSS_STAT_FCI(
			ioread32(USIF_MSS_STAT(port->membase)));
		pr_debug("--> %s:FCI: %#x\n", __func__, modem_status);
		if (modem_status) {
			del_timer(&uxp->modem_poll);
			if (modem_status != uxp->modem_status[0]) {
				pr_debug("--> %s: update uart\n", __func__);
				uart_handle_cts_change(port, modem_status);
				wake_up_interruptible(
					&port->state->port.delta_msr_wait);
			}
		} else {
			imsc = ioread32(USIF_IMSC(port->membase));
			imsc &= ~USIF_IMSC_FCI_MASK;
			iowrite32(imsc, USIF_IMSC(port->membase));
			mod_timer(&uxp->modem_poll,
				jiffies + platdata->modem_poll_timeout);
		}
		uxp->modem_status[0] = modem_status;
	}

	if (USIF_MIS_TX_FIN(status)) {
		if (uxp->flags & XGOLD_USIF_ALLOW_DEBUG)
			dev_dbg(port->dev, "Read TPS %x\n",
				ioread32(USIF_TPS_CTRL(port->membase)));
		xgold_usif_delete_modem_poll(port);
		if (uart_tx_stopped(port)) {
			xgold_usif_stop_tx(port);
		} else if (platdata->datapath == USIF_SERIAL_DATAPATH_PIO) {
			/* Initiate another transmission, if the circular buffer
			 * has still characters to send
			 */
			uxp->tx_tps_cnt = uart_circ_chars_pending(xmit);
			if (uxp->tx_tps_cnt) {
				iowrite32(uxp->tx_tps_cnt,
						USIF_TPS_CTRL(port->membase));
				if (uart_circ_chars_pending(xmit) <
						WAKEUP_CHARS)
					uart_write_wakeup(port);
			} else {
				xgold_usif_stop_tx(port);
				/* Call this function only after we done
				 * with sending all characters
				 */
				uart_write_wakeup(port);
			}
		}
	}
	iowrite32(status, USIF_ICR(port->membase));

	return IRQ_HANDLED;
}

#define USIF_DMAE_TX_DMA_CTRL (USIF_DMAE_TX_BREQ_DMA_CTRL | \
				USIF_DMAE_TX_LBREQ_DMA_CTRL | \
				USIF_DMAE_TX_SREQ_DMA_CTRL | \
				USIF_DMAE_TX_LSREQ_DMA_CTRL)

#define USIF_DMAE_RX_DMA_CTRL (USIF_DMAE_RX_BREQ_DMA_CTRL | \
				USIF_DMAE_RX_LBREQ_DMA_CTRL | \
				USIF_DMAE_RX_SREQ_DMA_CTRL | \
				USIF_DMAE_RX_LSREQ_DMA_CTRL)

static irqreturn_t xgold_usif_handle_wake_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);

	if (platdata->runtime_pm_enabled) {
		spin_lock(&port->lock);

		if (!platdata->irq_wk_enabled) {
			dev_err(port->dev, "irq_wk is not enabled ???\n");
			spin_unlock(&port->lock);
			return IRQ_HANDLED;
		}

		/* Disable the interrupt. */
		if (platdata->irq_wk) {
			dev_dbg(port->dev, "disabling irq_wk\n");
			disable_irq_nosync(platdata->irq_wk);
			platdata->irq_wk_enabled = 0;
		}

		spin_unlock(&port->lock);

		if (jiffies_to_msecs(jiffies - uxp->last_irq_wk_time) < 1) {
			dev_dbg(port->dev, "irq_wk considered as a glitch, re-enable\n");
			platdata->irq_wk_enabled = 1;
			enable_irq(platdata->irq_wk);
		} else {
			/* Resume from work queue. */
			if (!delayed_work_pending(&uxp->usif_rpm_work))
				schedule_delayed_work(&uxp->usif_rpm_work, 0);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t handle_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	unsigned int status, mask;
	unsigned reg;

	spin_lock(&port->lock);

	mask = USIF_STA_INT | USIF_ERR_INT;

	/* Check if the USIF TX/RX irqs should be handled by DMA h/w */
	reg = ioread32(USIF_DMAE(port->membase));
	if (!(reg & USIF_DMAE_TX_DMA_CTRL))
		mask |= USIF_MIS_TRANSMIT;

	if (!(reg & USIF_DMAE_RX_DMA_CTRL))
		mask |= USIF_MIS_RECEIVE;

	while ((status = (ioread32(USIF_MIS(port->membase)) & mask))) {

		/* Handle the RX interrupts */
		if (status & USIF_MIS_RECEIVE)
			receive_chars(port, status & USIF_MIS_RECEIVE);

		/* Handle the Status interrupts */
		if (status & USIF_STA_INT)
			handle_sta_interrupt(port, status & USIF_STA_INT);

		/* Handle the Error interrupt */
		if (status & USIF_ERR_INT)
			handle_err_interrupt(port, status & USIF_ERR_INT);

		/* Handle the Tx interrupt */
		if (status & USIF_MIS_TRANSMIT)
			transmit_chars(port, status & USIF_MIS_TRANSMIT);
	}

	spin_unlock(&port->lock);

	if (usif_port_is_console(&uxp->port) &&
		uxp->trace_buf_list &&
		!list_empty(&uxp->trace_buf_list->list) &&
		!delayed_work_pending(&uxp->usif_rpm_work)) {
		/* Console only: Flush any logs that we have buffered
		 * while transferring data through TTY. */
		schedule_delayed_work(&uxp->usif_rpm_work, 0);
	}

	return IRQ_HANDLED;
}

static int xgold_usif_request_irqs(struct uart_port *port,
		irq_handler_t handler)
{
	int ret = 0, i;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);

	for (i = 0; i < USIF_IRQS_NR; i++) {
		if (platdata->irqs[i] == -1)
			return 0;
		ret = request_irq(platdata->irqs[i], handler, IRQF_SHARED,
				uxp->name, port);
		if (ret) {
			dev_err(port->dev, "Failure in requesting IRQ\n");
			return ret;
		}
	}
	return ret;
}

static int xgold_usif_free_irqs(struct uart_port *port)
{
	int i;
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);

	for (i = 0; i < USIF_IRQS_NR; i++) {
		if (platdata->irqs[i] == -1)
			return 0;
		free_irq(platdata->irqs[i], port);
	}
	return 0;
}

static void xgold_usif_shutdown(struct uart_port *port)
{
	struct uart_usif_xgold_port *uxp = to_usif_port(port);

#ifdef CONFIG_PM_RUNTIME
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);
	if (platdata->runtime_pm_enabled) {
		pm_runtime_get_sync(port->dev);
		cancel_delayed_work_sync(&uxp->usif_rpm_work);
	}
#endif

	/* Ensure that no TX is going on before we shutdown.
	 */
	while (USIF_FIFO_STAT_TXFFS(ioread32(USIF_FIFO_STAT(port->membase))))
		barrier();

	/* And wait a moment to ensure the TX FIFOs are empty.
	 * This will prevent issues with the echo-command which shuts
	 * down the port immediately after writing the data to it
	 * which can lead to truncation of bits if we don't wait here.
	 */
	mdelay(2);

	/*FIXME: XMM6321 is using spin_lock_irqsave */
	spin_lock_irq(&port->lock);
	if (uxp->shutdown) {
		usif_set_enabled_mode(port);
		usif_set_config_mode(port);
		uxp->shutdown(port);
		usif_set_running_mode(port);
	}

	del_timer(&uxp->modem_poll);
	/* Disable all interrupts, disable the port */
	iowrite32(0, USIF_IMSC(port->membase));
	/* FIXME: Is it really made in pm callback ?
	 * xgold_usif_clk_disable(port);
	*/
	spin_unlock_irq(&port->lock);

#ifdef CONFIG_PM_RUNTIME
	if (platdata->rpm_auto_suspend_enable &&
		platdata->runtime_pm_enabled) {
		pm_runtime_dont_use_autosuspend(port->dev);
		pm_runtime_disable(port->dev);
		pm_runtime_put_noidle(port->dev);
	}
#endif

	xgold_usif_free_irqs(port);
}

static void xgold_usif_modem_poll(unsigned long param)
{
	struct uart_port *port = (struct uart_port *) param;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	unsigned imsc, fci_stat, old_fci_stat, modem_status;
	unsigned long flags;
	bool update_uart = false;

	spin_lock_irqsave(&port->lock, flags);

	if (xgold_usif_is_clk_enabled(&uxp->port) == false) {
		/* The port has been powered off ->
		 * need to return here. */
		spin_unlock_irqrestore(&port->lock, flags);
		return;
	}

#ifdef CONFIG_PM_RUNTIME
	if (pm_runtime_enabled(port->dev) &&
		(port->dev->power.runtime_status !=  RPM_ACTIVE)) {
		pr_debug("fci timer expired while device inactive. ignore\n");
		spin_unlock_irqrestore(&port->lock, flags);
		return;
	}
	pm_runtime_get_noresume(port->dev);
#endif

	iowrite32(USIF_ICR_FCI_MASK, USIF_ICR(port->membase));
	imsc = ioread32(USIF_IMSC(port->membase));
	imsc |= USIF_IMSC_FCI_MASK;
	iowrite32(imsc, USIF_IMSC(port->membase));

	modem_status = ioread32(USIF_MSS_STAT(port->membase));
	fci_stat = USIF_MSS_STAT_FCI(modem_status);
	old_fci_stat = uxp->modem_status[0];

	if (fci_stat == old_fci_stat) {
		pr_debug("--> %s:FCI event @lvl %x has been stable\n",
						__func__, fci_stat);
		update_uart = true;
	} else {
		pr_debug("--> %s:FCI event @lvl %#x considered as a glitch\n",
				__func__, old_fci_stat);
		if (fci_stat > old_fci_stat)
			update_uart = true;
	}

	uxp->modem_status[1] = uxp->modem_status[0];
	spin_unlock_irqrestore(&port->lock, flags);

	if (update_uart) {
		uart_handle_cts_change(port, fci_stat);
		wake_up_interruptible(&port->state->port.delta_msr_wait);
	}

	xgold_usif_runtime_pm_autosuspend(port);
}

static int xgold_usif_startup_idi(struct uart_port *port)
{
	unsigned reg;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);
	int mrps;

	if (platdata == NULL)
		return -ENODEV;

	/* If mrps is negative, the driver is using RPS buffered feature */

	if (platdata->rx_mrps < 0) {
		mrps = platdata->rx_buffer_size;
		mrps /= (BIT(USIF_FIFO_ID_RPS_STAGE(uxp->fifo_hwid)));
		platdata->rx_mrps = mrps;
	}

	dev_dbg(port->dev, "RX buffer size: %d, MRPS: %d\n",
		platdata->rx_buffer_size, platdata->rx_mrps);

	iowrite32(platdata->rx_mrps, USIF_MRPS_CTRL(port->membase));
	/* Let the IDI aware of the FIFO requests */
	reg = USIF_DMAE_TX_BREQ_DMA_CTRL |
		USIF_DMAE_TX_LBREQ_DMA_CTRL |
		USIF_DMAE_TX_SREQ_DMA_CTRL |
		USIF_DMAE_TX_LSREQ_DMA_CTRL |
		USIF_DMAE_RX_BREQ_DMA_CTRL |
		USIF_DMAE_RX_LBREQ_DMA_CTRL |
		USIF_DMAE_RX_SREQ_DMA_CTRL |
		USIF_DMAE_RX_LSREQ_DMA_CTRL;

	iowrite32(reg, USIF_DMAE(port->membase));

	/* Enable only the receive interrupts here */
	iowrite32(USIF_MIS_CLR_ALL, USIF_ICR(port->membase));
	iowrite32(USIF_MIS_RECEIVE, USIF_IMSC(port->membase));

	return 0;
}

static void xgold_usif_shutdown_dma(struct uart_port *port)
{
	struct uart_usif_xgold_port *uxp = to_usif_port(port);

	if (uxp->dma_initialized) {
		uxp->dma_initialized = false;
		dmaengine_terminate_all(uxp->dma_tx_channel);
		dma_release_channel(uxp->dma_tx_channel);
		uxp->dma_tx_channel = NULL;
		dmaengine_terminate_all(uxp->dma_rx_channel);
		dma_release_channel(uxp->dma_rx_channel);
		uxp->dma_rx_channel = NULL;
		uxp->sg_txbuf->count = 0;
	}

	if (uxp->sg_rxbuf_a) {
		kfree(uxp->sg_rxbuf_a->sg_iotx);
		kfree(uxp->sg_rxbuf_a->usif_dmabuf);
		kfree(uxp->sg_rxbuf_a);
		uxp->sg_rxbuf_a = NULL;
	}

	if (uxp->sg_rxbuf_b) {
		kfree(uxp->sg_rxbuf_b->sg_iotx);
		kfree(uxp->sg_rxbuf_b->usif_dmabuf);
		kfree(uxp->sg_rxbuf_b);
		uxp->sg_rxbuf_b = NULL;
	}

	if (uxp->sg_txbuf) {
		kfree(uxp->sg_txbuf->sg_iotx);
		kfree(uxp->sg_txbuf);
		uxp->sg_txbuf = NULL;
	}

	if (uxp->dma_tx_buf) {
		free_page((unsigned long)uxp->dma_tx_buf);
		uxp->dma_tx_buf = NULL;
	}
}

static struct usif_dma_sg *xgold_usif_dma_alloc_buf(unsigned int size)
{
	struct usif_dma_sg *sg_buf =
		kzalloc(sizeof(struct usif_dma_sg *), GFP_KERNEL);

	if (sg_buf == NULL)
		return NULL;

	sg_buf->sg_iotx = kzalloc(sizeof(struct scatterlist), GFP_KERNEL);

	if (sg_buf->sg_iotx == NULL) {
		kfree(sg_buf);
		return NULL;
	}

	if (!size)
		return sg_buf;

	sg_buf->usif_dmabuf = kzalloc(size, GFP_KERNEL);

	if (sg_buf->usif_dmabuf == NULL) {
		kfree(sg_buf->sg_iotx);
		kfree(sg_buf);
		sg_buf = NULL;
		return NULL;
	}

	return sg_buf;

}

static int xgold_usif_startup_dma(struct uart_port *port)
{
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);
	unsigned reg;
	int mrps;

	/* Reset the Rx and Tx local counters */
	uxp->tx_tps_cnt = 0x00;

	reg = USIF_DMAE_TX_DMA_CTRL |
			USIF_DMAE_RX_DMA_CTRL;

	iowrite32(reg, USIF_DMAE(port->membase));

	if (platdata->rx_mrps < 0) {
		mrps = platdata->rx_buffer_size;
		mrps /= (BIT(USIF_FIFO_ID_RPS_STAGE(uxp->fifo_hwid)));
		platdata->rx_mrps = mrps;
	}

	iowrite32(platdata->rx_mrps, USIF_MRPS_CTRL(port->membase));

	/* Enable RX and TX interrupts */
	iowrite32(USIF_MIS_CLR_ALL, USIF_ICR(port->membase));
	iowrite32(USIF_IMSC_TX_RX_CONFIG, USIF_IMSC(port->membase));

	/* We use static SG buffers for both RX and TX. */
	if (!uxp->sg_rxbuf_a) {
		uxp->sg_rxbuf_a =
			xgold_usif_dma_alloc_buf(platdata->rx_buffer_size);

		if (!uxp->sg_rxbuf_a)
			return -ENOMEM;
	}

	if (!uxp->sg_rxbuf_b) {
		uxp->sg_rxbuf_b =
			xgold_usif_dma_alloc_buf(platdata->rx_buffer_size);

		if (!uxp->sg_rxbuf_b)
			return -ENOMEM;
	}

	if (!uxp->sg_txbuf) {
		uxp->sg_txbuf = xgold_usif_dma_alloc_buf(0);
		if (!uxp->sg_txbuf)
			return -ENOMEM;
	}

	if (!uxp->dma_tx_buf) {
		unsigned long page;
		page = get_zeroed_page(GFP_KERNEL);
		if (!page)
			return -ENOMEM;

		uxp->dma_tx_buf = (char *) page;
	}

	uxp->sg_rxbuf_a->uxp = uxp;
	uxp->sg_rxbuf_b->uxp = uxp;
	uxp->sg_txbuf->uxp = uxp;

	xgold_usif_dma_init(uxp);

	return 0;
}

static int xgold_usif_startup_pio(struct uart_port *port)
{
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	unsigned reg;

	/* Reset the Rx and Tx local counters */
	uxp->tx_tps_cnt = 0x00;

	/* Let the CPU aware of the FIFO requests */
	reg = USIF_DMAE_TX_BREQ_CPU_CTRL |
		USIF_DMAE_TX_LBREQ_CPU_CTRL |
		USIF_DMAE_TX_SREQ_CPU_CTRL |
		USIF_DMAE_TX_LSREQ_CPU_CTRL |
		USIF_DMAE_RX_BREQ_CPU_CTRL |
		USIF_DMAE_RX_LBREQ_CPU_CTRL |
		USIF_DMAE_RX_SREQ_CPU_CTRL | USIF_DMAE_RX_LSREQ_CPU_CTRL;

	iowrite32(reg, USIF_DMAE(port->membase));

	/* Enable only the receive interrupts here */
	iowrite32(USIF_MIS_CLR_ALL, USIF_ICR(port->membase));
	iowrite32(USIF_MIS_RECEIVE, USIF_IMSC(port->membase));
	return 0;
}

static int xgold_usif_startup(struct uart_port *port)
{
	int ret = 0;
	unsigned int mode_cfg = 0;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	struct tty_port *tty_port = &port->state->port;
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);

	snprintf(uxp->name, sizeof(uxp->name), "xgold_usif%d", port->line);

	/* Update the 'low latency' flag
	 * before enabling the receive interrupts
	 */
	tty_port->low_latency = ((port->flags & ASYNC_LOW_LATENCY) ? 1 : 0);

	/* Request IRQ for USIF (Platform dependent) */
	ret = xgold_usif_request_irqs(port, handle_interrupt);
	if (ret) {
		dev_err(port->dev, "XGOLD USIF: failure in requesting IRQ\n");
		return ret;
	}

	init_timer(&uxp->modem_poll);
	uxp->modem_poll.data = (unsigned long) &uxp->port;
	uxp->modem_poll.function = xgold_usif_modem_poll;

	xgold_usif_clk_enable(port);

	/* FIXME: XMM6321 is using spinlock_irq_save
		No need anyway to handle IRQ
		since they are globally disabled for this callback
	*/
	spin_lock_irq(&port->lock);

	/* Reset peripheral */
	if (!IS_ERR_OR_NULL(platdata->reset))
		reset_control_reset(platdata->reset);

	usif_set_enabled_mode(port);
	usif_set_config_mode(port);

	if (uxp->startup)
		ret = uxp->startup(port);

	/* Need to mask out the RX enable bit, it will be set in
	 * the termios-function if this port shall support RX of data. */
	mode_cfg = ioread32(USIF_MODE_CFG(port->membase));
	mode_cfg &= ~USIF_MODE_CFG_RXEN_EN;
	iowrite32(mode_cfg, USIF_MODE_CFG(port->membase));

	usif_set_running_mode(port);
	spin_unlock_irq(&port->lock);

	if (ret)
		return ret;

	/* Need a delay here for correct "halt" behavior.. */
	mdelay(3);

	if (uxp->use_dma) {
		/* Ensure we are ready to receive data
		 * by queuing an RX DMA request.
		 */
		spin_lock_irq(&port->lock);
		iowrite32(0, USIF_IMSC(port->membase));
		spin_unlock_irq(&port->lock);
		xgold_usif_dma_rx_request(uxp);
		spin_lock_irq(&port->lock);
		iowrite32(USIF_IMSC_TX_RX_CONFIG, USIF_IMSC(port->membase));
		spin_unlock_irq(&port->lock);
	}

#ifdef CONFIG_PM_RUNTIME
	if (platdata->rpm_auto_suspend_enable &&
		platdata->runtime_pm_enabled) {
		/* Enable Runtime PM and start the autosuspend timer */
		pm_runtime_get_noresume(port->dev);
		pm_runtime_enable(port->dev);
		pm_runtime_use_autosuspend(port->dev);
		pm_runtime_set_autosuspend_delay(port->dev,
				platdata->rpm_suspend_delay);
		pm_runtime_mark_last_busy(port->dev);
		pm_runtime_put_autosuspend(port->dev);
	}
#endif

	return 0;
}

static void xgold_usif_enable_ms(struct uart_port *port)
{
	unsigned int sta_int_mask;

	if (xgold_usif_runtime_pm_suspended(port, __func__))
		/* Need to return since we are suspended and can't wake up
		 * in IRQ locked context.
		 */
		return;

	/* Set the STA mask. We would like to process the CTS interrupt */
	sta_int_mask = ioread32(USIF_IMSC(port->membase));
	sta_int_mask |= USIF_MIS_FCI_MASK;
	iowrite32(sta_int_mask, USIF_IMSC(port->membase));

	xgold_usif_runtime_pm_autosuspend(port);
}

static void xgold_usif_stop_tx(struct uart_port *port)
{
	unsigned int cr;

#ifdef CONFIG_PM_RUNTIME
	if (pm_runtime_enabled(port->dev) &&
		(port->dev->power.runtime_status !=  RPM_ACTIVE)) {
		pr_debug("%s called while device inactive. ignore\n",
			__func__);
		return;
	}
	pm_runtime_get_noresume(port->dev);
#endif

	cr = ioread32(USIF_IMSC(port->membase));
	cr &= ~USIF_MIS_TRANSMIT;
	iowrite32(cr, USIF_IMSC(port->membase));

	xgold_usif_runtime_pm_autosuspend(port);
}

static void xgold_usif_dma_init(struct uart_usif_xgold_port *uxp)
{
	struct uart_port *port = &uxp->port;
	struct dma_slave_config usif_config_tx;
	struct dma_slave_config usif_config_rx;
	int ret = 0;

	if (uxp->dma_initialized)
		return;

	/* Channel name needs to match what is in dtsi. */
	uxp->dma_tx_channel = dma_request_slave_channel(port->dev, "tx");

	if (!uxp->dma_tx_channel) {
		dev_err(port->dev, "ERROR in getting USIF TX channel\n");
		return;
	}

	/* We need to set the physical address, since port->membase
	 * is pointing to a virtual address it must not be used here. */
	usif_config_tx.dst_addr = (dma_addr_t) USIF_TXD(port->mapbase);
	usif_config_tx.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	usif_config_tx.dst_maxburst = 4;
	usif_config_tx.device_fc = true;
	usif_config_tx.direction = DMA_TO_DEVICE;

	ret = dmaengine_slave_config(uxp->dma_tx_channel, &usif_config_tx);

	if (IS_ERR(&ret)) {
		dev_err(port->dev, "Error in dma TX slave configuration\n");
		return;
	}

	/* Channel name needs to match what is in dtsi. */
	uxp->dma_rx_channel = dma_request_slave_channel(port->dev, "rx");

	if (!uxp->dma_rx_channel) {
		dev_err(port->dev, "ERROR in getting USIF RX channel\n");
		return;
	}

	/* We need to set the physical address, since port->membase
	 * is pointing to a virtual address it must not be used here. */
	usif_config_rx.src_addr = (dma_addr_t) USIF_RXD(port->mapbase);
	usif_config_rx.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	usif_config_rx.src_maxburst = 4;
	usif_config_rx.device_fc = true;
	usif_config_rx.direction = DMA_FROM_DEVICE;

	ret = dmaengine_slave_config(uxp->dma_rx_channel, &usif_config_rx);

	uxp->dma_initialized = true;

	if (IS_ERR(&ret))
		dev_err(port->dev, "Error in dma RX slave configuration\n");
}

static void xgold_usif_dma_rx_request(struct uart_usif_xgold_port *uxp)
{
	struct xgold_usif_platdata *platdata = dev_get_platdata(uxp->port.dev);
	struct dma_async_tx_descriptor *usif_desc_rx;
	struct usif_dma_sg *usif_sg;
	dma_cookie_t dma_cookie_rx;

	if (uxp->use_rxbuf_b) {
		uxp->use_rxbuf_b = false;
		usif_sg = uxp->sg_rxbuf_a;
	} else {
		uxp->use_rxbuf_b = true;
		usif_sg = uxp->sg_rxbuf_b;
	}

	usif_sg->uxp = uxp;

	/* Queue the read buffer */
	sg_init_one(usif_sg->sg_iotx, usif_sg->usif_dmabuf,
			platdata->rx_buffer_size);
	dma_map_sg(uxp->port.dev, usif_sg->sg_iotx, 1, DMA_FROM_DEVICE);

	usif_desc_rx = dmaengine_prep_slave_sg(
			uxp->dma_rx_channel, usif_sg->sg_iotx, 1,
			DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);

	if (!usif_desc_rx) {
		xgold_usif_error(&uxp->port, __LINE__, __func__);
		return;
	}

	if (usif_desc_rx->phys != (dma_addr_t) USIF_RXD(uxp->port.mapbase))
		usif_desc_rx->phys = (dma_addr_t) USIF_RXD(uxp->port.mapbase);

	/* DMA callback */
	usif_desc_rx->callback = xgold_usif_dma_rx_call_back;
	usif_desc_rx->callback_param = usif_sg;

	/* Submit request to start RX */
	dma_cookie_rx = dmaengine_submit(usif_desc_rx);
	usif_sg->dma_cookie = dma_cookie_rx;

	dma_async_issue_pending(uxp->dma_rx_channel);

	return;
}

static void xgold_usif_dma_rx_call_back(void *param)
{
	struct usif_dma_sg *usif_sg =  (struct usif_dma_sg *)param;
	int rx_chars, count;

	if (usif_sg && usif_sg->uxp) {
		struct uart_usif_xgold_port *uxp = usif_sg->uxp;
		struct tty_port *tty_port = &uxp->port.state->port;
		struct xgold_usif_platdata *platdata =
				dev_get_platdata(uxp->port.dev);
		enum dma_status status;
		char flg;
		unsigned int rsr;

#ifdef CONFIG_PM_RUNTIME
		if ((platdata->runtime_pm_enabled) &&
			(platdata->rpm_auto_suspend_enable) &&
			(pm_runtime_enabled(uxp->port.dev))) {
			if ((uxp->port.dev->power.runtime_status ==
						RPM_SUSPENDED)) {
				dev_err(uxp->port.dev,
					"Error: USIF port is Runtime suspended in dma RX func\n");
				BUG();
			} else if (uxp->port.dev->power.runtime_status ==
						RPM_SUSPENDING) {
				dev_info(uxp->port.dev, "RX data while suspending\n");
				pm_runtime_get_noresume(uxp->port.dev);

			} else if (uxp->port.dev->power.runtime_status ==
						RPM_RESUMING) {
				dev_info(uxp->port.dev, "RX data while resuming\n");
				pm_runtime_get_noresume(uxp->port.dev);

			} else if (uxp->port.dev->power.runtime_status ==
						RPM_ACTIVE) {
				pm_runtime_get_noresume(uxp->port.dev);
			}
		}
#endif

		status = dma_async_is_tx_complete(uxp->dma_rx_channel,
				usif_sg->dma_cookie, NULL, NULL);

		if (status == DMA_IN_PROGRESS ||
			status == DMA_PAUSED ||
			status == DMA_ERROR) {
			xgold_usif_error(&uxp->port, __LINE__, __func__);
			xgold_usif_runtime_pm_autosuspend(&uxp->port);
			return;
		}

		spin_lock_irq(&uxp->port.lock);

		count = USIF_RPS_STAT_RPS(
				ioread32(USIF_RPS_STAT(uxp->port.membase)));

		dma_unmap_sg(uxp->port.dev, usif_sg->sg_iotx,
					1, DMA_FROM_DEVICE);

		BUG_ON(count > platdata->rx_buffer_size || count < 0);

		spin_unlock_irq(&uxp->port.lock);

		/* Set up next RX request asap. */
		xgold_usif_dma_rx_request(usif_sg->uxp);

		spin_lock_irq(&uxp->port.lock);

		rsr = handle_receive_error(&uxp->port, &flg);

		for (rx_chars = 0; rx_chars < count; rx_chars++) {
			if (!uart_handle_sysrq_char(&uxp->port,
					usif_sg->usif_dmabuf[rx_chars])) {
				uart_insert_char(&uxp->port, rsr,
						USIF_MIS_RXOF_INT_PEND,
						usif_sg->usif_dmabuf[rx_chars],
						flg);
				uxp->port.icount.rx++;
			}
		}
		spin_unlock_irq(&uxp->port.lock);

		tty_flip_buffer_push(tty_port);

		xgold_usif_runtime_pm_autosuspend(&uxp->port);

	} else {
		BUG();
	}
}

static void xgold_usif_dma_tx_call_back(void *param)
{
	struct usif_dma_sg *usif_sg =  (struct usif_dma_sg *)param;

	if (!usif_sg)
		BUG();

	if (usif_sg->uxp) {
		unsigned long flags;
		struct uart_usif_xgold_port *uxp = usif_sg->uxp;
		struct circ_buf *xmit = &uxp->port.state->xmit;
#ifdef CONFIG_PM_RUNTIME
		struct xgold_usif_platdata *platdata =
					dev_get_platdata(uxp->port.dev);

		if ((platdata->runtime_pm_enabled) &&
			(pm_runtime_enabled(uxp->port.dev)) &&
			(uxp->port.dev->power.runtime_status != RPM_ACTIVE)) {
			pr_err("%s: USIF is RPM suspended in DMA callback!\n",
					__func__);
			BUG();
			return;
		}
#endif

		spin_lock_irqsave(&uxp->port.lock, flags);

		dma_unmap_sg(uxp->port.dev, usif_sg->sg_iotx, 1, DMA_TO_DEVICE);

		if (!usif_sg->dmabuf_owned)
			/* Advance the tail pointer in the ring buffer since
			 * data is processed.
			 */
			xmit->tail = (xmit->tail + usif_sg->count) &
					(UART_XMIT_SIZE - 1);

		usif_sg->usif_dmabuf = NULL;
		usif_sg->dmabuf_owned = false;
		usif_sg->count = 0;
		usif_sg->dma_cookie = 0;

		if (uart_circ_chars_pending(xmit))
			/* We have more data to send -> reschedule
			 * the TX tasklet.
			 */
			tasklet_schedule(&uxp->tx_tasklet);

		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&uxp->port);

		spin_unlock_irqrestore(&uxp->port.lock, flags);

		xgold_usif_runtime_pm_autosuspend(&uxp->port);

	} else {
		BUG();
	}
}

static void xgold_usif_dma_submit_tx(struct usif_dma_sg *usif_sg)
{
	struct uart_usif_xgold_port *uxp = usif_sg->uxp;
	struct dma_async_tx_descriptor *usif_desc_tx;
	struct dma_chan *dma_tx_channel = uxp->dma_tx_channel;

	sg_init_one(usif_sg->sg_iotx, usif_sg->usif_dmabuf,
				round_up(usif_sg->count, 4));
	dma_map_sg(uxp->port.dev, usif_sg->sg_iotx, 1, DMA_TO_DEVICE);

	usif_desc_tx = dmaengine_prep_slave_sg(
			dma_tx_channel, usif_sg->sg_iotx, 1,
			DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);

	if (!usif_desc_tx)
		goto write_init_dma_fail;

	/* DMA callback */
	usif_desc_tx->callback = xgold_usif_dma_tx_call_back;
	usif_desc_tx->callback_param = usif_sg;

	/* USIF is flow controller so we need to write
	 * correct TPS value.
	 */
	xgold_usif_write_tps(&uxp->port, usif_sg->count);

	/* Submit request to start TX. */
	usif_sg->dma_cookie = dmaengine_submit(usif_desc_tx);
	dma_async_issue_pending(dma_tx_channel);

	return;

write_init_dma_fail:
	xgold_usif_error(&uxp->port, __LINE__, __func__);
	dma_unmap_sg(uxp->port.dev, usif_sg->sg_iotx, 1, DMA_TO_DEVICE);
	dmaengine_terminate_all(dma_tx_channel);
	uxp->sg_txbuf->count = 0;
	return;
}

static void xgold_usif_dma_tx_tasklet_func(unsigned long data)
{
	struct uart_port *port = (struct uart_port *)data;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	struct circ_buf *xmit = &uxp->port.state->xmit;
	int count;
	unsigned long flags;

	if (uart_circ_chars_pending(xmit)) {

		char *buf = NULL;

		if (xgold_usif_runtime_pm_suspended(port, __func__)) {
			if (!delayed_work_pending(&uxp->usif_rpm_work))
				schedule_delayed_work(&uxp->usif_rpm_work, 0);
			return;
		}

		spin_lock_irqsave(&port->lock, flags);

		if (uxp->sg_txbuf->count != 0) {
			/* A transfer is already ongoing -> return. */
			spin_unlock_irqrestore(&port->lock, flags);
			xgold_usif_runtime_pm_autosuspend(port);
			return;
		}

		count = uart_circ_chars_pending(xmit);

		if (xmit->tail + count > UART_XMIT_SIZE ||
				xmit->tail % 4 != 0) {
			/* Need to copy to local TX buffer. */
			int n;
			buf = uxp->dma_tx_buf;

			if (xmit->tail + count > UART_XMIT_SIZE) {
				n = UART_XMIT_SIZE - xmit->tail;
				memcpy(buf, &xmit->buf[xmit->tail], n);
				memcpy(&buf[n], xmit->buf, count - n);
			} else {
				memcpy(buf, &xmit->buf[xmit->tail], count);
			}

			uxp->sg_txbuf->usif_dmabuf = buf;
			uxp->sg_txbuf->dmabuf_owned = true;
			xmit->tail = (xmit->tail + count) &
					(UART_XMIT_SIZE - 1);
		} else {
			/* Use the ring buffer for TX.
			 * The tail pointer will be advanced
			 * in the TX complete callback to avoid
			 * data corruption.
			 */
			uxp->sg_txbuf->usif_dmabuf = &xmit->buf[xmit->tail];
			uxp->sg_txbuf->dmabuf_owned = false;
		}
		port->icount.tx += count;
		uxp->sg_txbuf->uxp = uxp;
		uxp->sg_txbuf->count = count;

		spin_unlock_irqrestore(&port->lock, flags);

		xgold_usif_dma_submit_tx(uxp->sg_txbuf);

	}

	return;
}

static void xgold_usif_start_tx(struct uart_port *port)
{
	unsigned int imsc;
	struct circ_buf *xmit = &port->state->xmit;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);

	if (uxp->use_dma) {
		if (uxp->sg_txbuf->count != 0)
			/* A transfer is already active. We cannot
			 * queue another DMA transfer because we need
			 * to write the TPS register which can corrupt the
			 * ongoing transfer.
			 * We simply return here since the TX tasklet will
			 * be scheduled from the TX complete callback.
			 */
			return;

		if (xgold_usif_runtime_pm_suspended(port, __func__))
			return;

		/* Submitting data to DMA happens in tasklet
		 * to prevent scheduled while atomic bug.
		*/
		tasklet_schedule(&uxp->tx_tasklet);

		xgold_usif_runtime_pm_autosuspend(port);

		return;
	}

	if (xgold_usif_runtime_pm_suspended(port, __func__))
		return;

	/* Initiate the transmission only if the transmission is
	 * not active already or stopped due to flow control.
	 */
	if (platdata->datapath == USIF_SERIAL_DATAPATH_PIO
			&& uxp->tx_tps_cnt == 0) {
		uxp->tx_tps_cnt = uart_circ_chars_pending(xmit);
		if (uxp->tx_tps_cnt == 0)
			goto start_tx_done;
		iowrite32(uxp->tx_tps_cnt, USIF_TPS_CTRL(port->membase));
	}

	/* Set the TX mask to generate forward requests */
	imsc = ioread32(USIF_IMSC(port->membase));
	imsc |= USIF_MIS_TRANSMIT;
	iowrite32(imsc, USIF_IMSC(port->membase));

start_tx_done:
	xgold_usif_runtime_pm_autosuspend(port);
}

static void xgold_usif_stop_rx(struct uart_port *port)
{
	unsigned int cr;

	if (xgold_usif_runtime_pm_suspended(port, __func__))
		/* Need to return since we are suspended and can't wake up
		 * in IRQ locked context.
		 */
		return;

	cr = ioread32(USIF_IMSC(port->membase));
	cr &= ~USIF_MIS_RECEIVE;
	iowrite32(cr, USIF_IMSC(port->membase));

	xgold_usif_runtime_pm_autosuspend(port);
}

static unsigned int xgold_usif_get_mctrl(struct uart_port *port)
{
	unsigned char status;
	unsigned int ret = 0;

	if (xgold_usif_runtime_pm_suspended(port, __func__))
		/* Need to return since we are suspended and can't wake up
		 * in IRQ locked context.
		 */
		return ret;

	status = ioread32(USIF_MSS_STAT(port->membase));

#define USIF_PRINT_MSS(_X_) \
{\
	if (status & USIF_MSS_STAT_##_X_(status))\
		dev_dbg(port->dev, "%s: "#_X_" detected\n", __func__); \
}

	USIF_PRINT_MSS(DCD)
	USIF_PRINT_MSS(RI)
	USIF_PRINT_MSS(DSR)
	USIF_PRINT_MSS(FCI)

	if (USIF_MSS_STAT_DCD(status))
		ret |= TIOCM_CAR;

	if (USIF_MSS_STAT_RI(status))
		ret |= TIOCM_RNG;

	if (USIF_MSS_STAT_DSR(status))
		ret |= TIOCM_DSR;

	if (USIF_MSS_STAT_FCI(status))
		ret |= TIOCM_CTS;

	xgold_usif_runtime_pm_autosuspend(port);

	return ret;
}

static void _xgold_usif_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	unsigned char m_set = 0x00;
	unsigned char m_clr = 0x00;
	unsigned int reg;

	if (!usif_port_is_console(port))
		dev_dbg(port->dev, "Setting mctrl to %x\n", mctrl);

	if (mctrl & TIOCM_RTS)
		m_set |= USIF_MSS_SET_FCOSET_MASK;
	else
		m_clr |= USIF_MSS_CLR_FCOCLR_MASK;

	if (mctrl & TIOCM_DTR)
		m_set |= USIF_MSS_SET_DTRSET_MASK;
	else
		m_clr |= USIF_MSS_CLR_DTRCLR_MASK;

	reg = ioread32(USIF_MSS_CTRL(port->membase));

	if (m_clr & USIF_MSS_CLR_FCOCLR_MASK) {
		reg &= ~USIF_MSS_CTRL_FCOEN_MASK;
		iowrite32(reg, USIF_MSS_CTRL(port->membase));
	}

	if (m_set & USIF_MSS_SET_FCOSET_MASK) {
		reg |= USIF_MSS_CTRL_FCOEN_MASK;
		iowrite32(reg, USIF_MSS_CTRL(port->membase));
	}

	if (m_set)
		iowrite32(m_set, USIF_MSS_SET(port->membase));

	if (m_clr)
		iowrite32(m_clr, USIF_MSS_CLR(port->membase));

}

static void xgold_usif_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	if (xgold_usif_runtime_pm_suspended(port, __func__))
		/* Need to return since we are suspended and can't wake up
		 * in IRQ locked context.
		 */
		return;

	_xgold_usif_set_mctrl(port, mctrl);

	xgold_usif_runtime_pm_autosuspend(port);
}

static unsigned int xgold_usif_tx_empty(struct uart_port *port)
{
	unsigned fifo_stat;
	int ret = TIOCSER_TEMT;

	xgold_usif_runtime_pm_resume(port);

	fifo_stat = ioread32(USIF_FIFO_STAT(port->membase));

	if (USIF_FIFO_STAT_TXFFS(fifo_stat)
			|| !xgold_usif_is_tx_ready(port))
		ret = 0;

	xgold_usif_runtime_pm_autosuspend(port);

	return ret;
}

static void xgold_usif_pm(struct uart_port *port, unsigned int state,
		unsigned int oldstate)
{
	unsigned long flags;

	xgold_usif_runtime_pm_resume(port);

	switch (state) {
	case UART_PM_STATE_ON:
		xgold_usif_clk_enable(port);
		/* FIXME: XMM6321 is spinlock_irq here, should we ? */
		spin_lock_irqsave(&port->lock, flags);
		usif_set_enabled_mode(port);
		usif_set_running_mode(port);
		spin_unlock_irqrestore(&port->lock, flags);
		break;

	case UART_PM_STATE_OFF:
		spin_lock_irqsave(&port->lock, flags);
		usif_set_disabled_mode(port);
		spin_unlock_irqrestore(&port->lock, flags);
		xgold_usif_clk_disable(port);
		break;

	default:
		if (!usif_port_is_console(port))
			dev_err(port->dev, "Unknown PM state %d\n", state);
	}

	xgold_usif_runtime_pm_autosuspend(port);
}

void xgold_usif_wake_peer(struct uart_port *port)
{
	unsigned modem_status;

	if (xgold_usif_runtime_pm_suspended(port, __func__))
		/* Need to return since we are suspended and can't wake up
		 * in IRQ locked context.
		 */
		return;

	modem_status = ioread32(USIF_MSS_STAT(port->membase));
	if (USIF_MSS_STAT_FCI(modem_status)) {
		uart_handle_cts_change(port, USIF_MSS_STAT_FCI(modem_status));
		wake_up_interruptible(&port->state->port.delta_msr_wait);
	}

	xgold_usif_runtime_pm_autosuspend(port);
}

struct uart_ops usif_ops = {
	.tx_empty = xgold_usif_tx_empty,
	.set_mctrl = xgold_usif_set_mctrl,
	.get_mctrl = xgold_usif_get_mctrl,
	.stop_tx = xgold_usif_stop_tx,
	.start_tx = xgold_usif_start_tx,
	.stop_rx = xgold_usif_stop_rx,
	.enable_ms = xgold_usif_enable_ms,
	.startup = xgold_usif_startup,
	.shutdown = xgold_usif_shutdown,
	.set_termios = xgold_usif_set_termios,
	.type = xgold_usif_type,
	.release_port = xgold_usif_release_port,
	.request_port = xgold_usif_request_port,
	.config_port = xgold_usif_config_port,
	.verify_port = xgold_usif_verify_port,
	.pm = xgold_usif_pm,
	.wake_peer = xgold_usif_wake_peer,
};

#ifdef CONFIG_PM_RUNTIME
#define WAKE_FROM_SLEEP		"Runtime PM Console Wakeup\n"
#define GO_TO_SLEEP		"Runtime PM Console Sleep\n"

static void xgold_usif_direct_write(struct uart_port *port, const char *s,
					unsigned int count);

int xgold_usif_serial_pm_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_usif_xgold_port *uxp = platform_get_drvdata(pdev);
	struct uart_port *port = &uxp->port;
	unsigned long flags;
	unsigned int imsc;
	struct circ_buf *xmit = &port->state->xmit;
	struct xgold_usif_platdata *platdata = dev_get_platdata(dev);
	char *usif_rpm_event[4];

	/* Disable all USIF irq as changing the pinctrl might cuase
	 * unintended interrupt. it will be restored when resume */
	spin_lock_irqsave(&uxp->port.lock, flags);
	imsc = ioread32(USIF_IMSC(port->membase));
	iowrite32(0, USIF_IMSC(port->membase));
	spin_unlock_irqrestore(&uxp->port.lock, flags);

	/* Ensure that no TX is going on before we suspend.
	 */
	if (USIF_FIFO_STAT_TXFFS(ioread32(USIF_FIFO_STAT(port->membase)))) {
		dev_dbg(dev, "tx in progress\n");
		goto device_busy;
	}

	if (uart_circ_chars_pending(xmit) > 0) {
		dev_dbg(dev, "uart still need to send\n");
		goto device_busy;
	}

	del_timer_sync(&uxp->modem_poll);

	mutex_lock(&uxp->runtime_lock);

	if (uxp->use_dma && uxp->dma_initialized) {
		uxp->dma_initialized = false;
		dmaengine_terminate_all(uxp->dma_tx_channel);
		dma_release_channel(uxp->dma_tx_channel);
		uxp->dma_tx_channel = NULL;
		dmaengine_terminate_all(uxp->dma_rx_channel);
		dma_release_channel(uxp->dma_rx_channel);
		uxp->dma_rx_channel = NULL;
	}

	if (xgold_usif_is_clk_enabled(&uxp->port) == false) {
		/* The port is already powered off ->
		 * nothing to be done here. */
		if (platdata->runtime_pm_debug &&
			!usif_port_is_console(&uxp->port))
			dev_warn(dev, "Port is already disabled\n");
		mutex_unlock(&uxp->runtime_lock);
		return 0;
	}

	if (platdata->runtime_pm_debug) {
#ifdef CONFIG_SERIAL_XGOLD_CONSOLE
		if (usif_port_is_console(&uxp->port) && uxp->port.line == 0) {
			spin_lock_irqsave(&uxp->port.lock, flags);
			xgold_usif_direct_write(&uxp->port, GO_TO_SLEEP,
					(strlen(GO_TO_SLEEP)));
			spin_unlock_irqrestore(&uxp->port.lock, flags);

			/* Wait 1ms to ensure that the sleep message is fully
			 * written to the console and that the TX_FIN interrupt
			 * is cleared before switching off. */
			mdelay(1);
		} else
#endif
			dev_info(dev, "Port is runtime suspended.\n");
	}

	/* FIXME: pin config may differ between runtime and normal
	 * suspend */
	xgold_usif_set_pinctrl_state(dev, platdata->pins_sleep);

	spin_lock_irqsave(&uxp->port.lock, flags);
	usif_set_enabled_mode(&uxp->port);
	usif_set_config_mode(&uxp->port);
	usif_set_disabled_mode(&uxp->port);
	spin_unlock_irqrestore(&uxp->port.lock, flags);

	/* Disable USIF clock */
	xgold_usif_clk_disable(&uxp->port);

	spin_lock_irqsave(&uxp->port.lock, flags);
	if (platdata->rpm_auto_suspend_enable && platdata->irq_wk) {
		if (!platdata->irq_wk_enabled) {
			platdata->irq_wk_enabled = 1;
			enable_irq(platdata->irq_wk);
			uxp->last_irq_wk_time = jiffies;
		}
	}
	spin_unlock_irqrestore(&uxp->port.lock, flags);

	/* generate uevent for suspend */
	if (platdata->rpm_gen_uevent) {
		usif_rpm_event[0] = kasprintf(GFP_KERNEL, "MAJOR=%d",
			uxp->drv->major);
		usif_rpm_event[1] = kasprintf(GFP_KERNEL, "MINOR=%d",
			(uxp->drv->minor + platdata->id));
		usif_rpm_event[2] = kasprintf(GFP_KERNEL, "%s",
			"UART_STATE=SUSPENDED");
		usif_rpm_event[3] = NULL;

		kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, usif_rpm_event);

		kfree(usif_rpm_event[0]);
		kfree(usif_rpm_event[1]);
		kfree(usif_rpm_event[2]);
	}

	mutex_unlock(&uxp->runtime_lock);

	return 0;

device_busy:
	spin_lock_irqsave(&uxp->port.lock, flags);
	iowrite32(imsc, USIF_IMSC(port->membase));
	spin_unlock_irqrestore(&uxp->port.lock, flags);
	return -EBUSY;
}

int xgold_usif_serial_pm_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_usif_xgold_port *uxp = platform_get_drvdata(pdev);
	unsigned long flags;
	struct xgold_usif_platdata *platdata = dev_get_platdata(dev);
	struct ktermios termios;
	int locked;
	char *usif_rpm_event[4];

	locked = mutex_trylock(&uxp->runtime_lock);

	if (platdata->rpm_auto_suspend_enable) {
		spin_lock_irqsave(&uxp->port.lock, flags);
		if (platdata->irq_wk) {
			if (platdata->irq_wk_enabled) {
				disable_irq_nosync(platdata->irq_wk);
				platdata->irq_wk_enabled = 0;
			}
		}
		spin_unlock_irqrestore(&uxp->port.lock, flags);
	}

	xgold_usif_clk_enable(&uxp->port);

	spin_lock_irqsave(&uxp->port.lock, flags);

	/* Reset peripheral */
	if (!IS_ERR_OR_NULL(platdata->reset))
		reset_control_reset(platdata->reset);

	usif_set_enabled_mode(&uxp->port);
	usif_set_config_mode(&uxp->port);

	if (uxp->startup)
		uxp->startup(&uxp->port);

	usif_set_running_mode(&uxp->port);

	/* Need to restore the termios settings. */
	memset(&termios, 0, sizeof(struct ktermios));
	if (uxp->port.cons)
		termios.c_cflag = uxp->port.cons->cflag;

	if (uxp->port.state->port.tty && termios.c_cflag == 0)
		termios = uxp->port.state->port.tty->termios;

	_xgold_usif_set_termios(&uxp->port, &termios, NULL);

	/* restore last mctrl settings */
	_xgold_usif_set_mctrl(&uxp->port, uxp->port.mctrl);

	spin_unlock_irqrestore(&uxp->port.lock, flags);

	xgold_usif_set_pinctrl_state(dev, platdata->pins_default);

	if (platdata->runtime_pm_debug) {
#ifdef CONFIG_SERIAL_XGOLD_CONSOLE
		if (usif_port_is_console(&uxp->port) && uxp->port.line == 0)
			xgold_usif_direct_write(&uxp->port, WAKE_FROM_SLEEP,
				(strlen(WAKE_FROM_SLEEP)));
		else
#endif
			dev_info(dev, "Port is runtime resumed.\n");
	}

	if (uxp->use_dma) {
		/* Ensure we are ready to receive data
		 * by queuing an RX DMA request.
		 */
		spin_lock_irqsave(&uxp->port.lock, flags);
		iowrite32(0, USIF_IMSC(uxp->port.membase));
		spin_unlock_irqrestore(&uxp->port.lock, flags);
		xgold_usif_dma_rx_request(uxp);
		spin_lock_irqsave(&uxp->port.lock, flags);
		iowrite32(USIF_MIS_CLR_ALL, USIF_ICR(uxp->port.membase));
		iowrite32(USIF_IMSC_TX_RX_CONFIG, USIF_IMSC(uxp->port.membase));
		spin_unlock_irqrestore(&uxp->port.lock, flags);
	}

	/* generate uevent for resume */
	if (platdata->rpm_gen_uevent) {
		usif_rpm_event[0] = kasprintf(GFP_KERNEL, "MAJOR=%d",
			uxp->drv->major);
		usif_rpm_event[1] = kasprintf(GFP_KERNEL, "MINOR=%d",
			(uxp->drv->minor + platdata->id));
		usif_rpm_event[2] = kasprintf(GFP_KERNEL, "%s",
			"UART_STATE=RESUMED");
		usif_rpm_event[3] = NULL;

		kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, usif_rpm_event);

		kfree(usif_rpm_event[0]);
		kfree(usif_rpm_event[1]);
		kfree(usif_rpm_event[2]);
	}

	if (locked)
		mutex_unlock(&uxp->runtime_lock);

	return 0;
}

#else
int xgold_usif_serial_pm_runtime_suspend(struct device *dev)
{
	return 0;
}

int xgold_usif_serial_pm_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM
static int xgold_usif_serial_suspend(struct device *dev)
{
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);
	struct uart_port *port = &uxp->port;
	struct xgold_usif_platdata *platdata = dev_get_platdata(dev);
	unsigned long flags;
	int ret = 0;

	if (!uxp->drv)
		return 0;

	/* Ensure that we are awake */
	xgold_usif_runtime_pm_resume(&uxp->port);

	if (uxp->use_dma && uxp->dma_initialized) {
		/* Release the DMA channel. */
		dmaengine_terminate_all(uxp->dma_tx_channel);
		dma_release_channel(uxp->dma_tx_channel);
		uxp->dma_tx_channel = NULL;
		dmaengine_terminate_all(uxp->dma_rx_channel);
		dma_release_channel(uxp->dma_rx_channel);
		uxp->dma_rx_channel = NULL;

		uxp->dma_initialized = false;
	}

	/* Ensure that no TX is going on before we suspend.
	 */

	if (xgold_usif_is_clk_enabled(port))
		while (USIF_FIFO_STAT_TXFFS(
			ioread32(USIF_FIFO_STAT(port->membase))))
				barrier();

	/* uart_suspend_port will call the PM callback which
	 * disables the USIF HW and powers it off. */
	ret = uart_suspend_port(uxp->drv, port);

	xgold_usif_set_pinctrl_state(dev, platdata->pins_sleep);

	/* Even if uart_suspend_port() already calls enable_irq_wake, we need
	 * to enable the irq_wk anyway as this is not the same interrupt line.
	 * Indeed uart_suspend_port() will enable the port->irq interrupt, here
	 * we need the irq_wk interrupt */
	spin_lock_irqsave(&uxp->port.lock, flags);
	if (device_may_wakeup(port->dev) && platdata->irq_wk) {
		platdata->irq_wk_enabled = 1;
		uxp->last_irq_wk_time = jiffies;
		enable_irq(platdata->irq_wk);
		enable_irq_wake(platdata->irq_wk);
	}
	spin_unlock_irqrestore(&uxp->port.lock, flags);

	return ret;
}

static int xgold_usif_serial_resume(struct device *dev)
{
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);
	struct uart_port *port = &uxp->port;
	struct xgold_usif_platdata *platdata = dev_get_platdata(dev);
	int ret = 0;

	if (!uxp->drv)
		return 0;

	xgold_usif_set_pinctrl_state(dev, platdata->pins_default);

	/* uart_resume_port will call the PM callback which
	 * powers on the USIF HW and sets it to run state. */
	ret = uart_resume_port(uxp->drv, port);

	if (device_may_wakeup(port->dev) && platdata->irq_wk) {
		disable_irq_wake(platdata->irq_wk);
		disable_irq(platdata->irq_wk);
	}

	if (uxp->use_dma)
		xgold_usif_dma_init(uxp);

	/* Restart auto-resume */
	xgold_usif_runtime_pm_autosuspend(port);

	return ret;
}

static const struct dev_pm_ops serial_xgold_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(xgold_usif_serial_suspend,
			xgold_usif_serial_resume)
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(xgold_usif_serial_pm_runtime_suspend,
			xgold_usif_serial_pm_runtime_resume, NULL)
#endif
};

#define SERIAL_XGOLD_PM_OPS (&serial_xgold_pm_ops)
#else
#define SERIAL_XGOLD_PM_OPS NULL
#endif

/*
 * Console
 */

#ifdef CONFIG_SERIAL_XGOLD_CONSOLE
#include <linux/console.h>

static inline void xgold_usif_putchar(struct uart_port *port, int s)
{
	while (USIF_FIFO_STAT_TXFFS(ioread32(USIF_FIFO_STAT(port->membase))))
		barrier();

	iowrite32(1, USIF_TPS_CTRL(port->membase));
	iowrite32(s, USIF_TXD(port->membase));
}

static void xgold_usif_direct_write(struct uart_port *port, const char *s,
					unsigned int count)
{
	unsigned int old_int_mask;
	old_int_mask = ioread32(USIF_IMSC(port->membase));
	iowrite32(USIF_MIS_RECEIVE, USIF_IMSC(port->membase));
	iowrite32(USIF_MIS_CLR_ALL, USIF_ICR(port->membase));

	uart_console_write(port, s, count, xgold_usif_putchar);

	while (USIF_FIFO_STAT_TXFFS(ioread32(USIF_FIFO_STAT(port->membase)))
			&& !xgold_usif_is_tx_ready(port))
		barrier();

	iowrite32(USIF_MIS_CLR_ALL, USIF_ICR(port->membase));
	iowrite32(old_int_mask, USIF_IMSC(port->membase));
}

static void xgold_usif_add_list(
				struct xgold_usif_trace_buffer_list *trace_list,
				int len, const char *trace_buf)
{
	struct xgold_usif_trace_buffer_list *tmp_list;
	char *buf = kzalloc(len, GFP_ATOMIC);

	if (buf == NULL)
		return;

	memcpy(buf, trace_buf, len);

	tmp_list = (struct xgold_usif_trace_buffer_list *)
		kzalloc(sizeof(struct xgold_usif_trace_buffer_list),
				GFP_ATOMIC);

	if (tmp_list == NULL) {
		kfree(buf);
		return;
	}

	tmp_list->trace_buf = buf;
	tmp_list->buf_len = len;
	list_add_tail(&tmp_list->list, &trace_list->list);
}

static void xgold_usif_console_write(struct console *co, const char *s,
		unsigned int count)
{
	struct uart_port *port = &xgold_usif_ports[co->index]->port;
	struct uart_usif_xgold_port *uxp = to_usif_port(port);
	unsigned int old_int_mask;
	unsigned long flags;
	int locked = 1;
#ifdef CONFIG_PM_RUNTIME
	struct xgold_usif_platdata *platdata = dev_get_platdata(port->dev);
#endif

	local_irq_save(flags);
	if (port->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock(&port->lock);
	else
		spin_lock(&port->lock);

#ifdef CONFIG_PM_RUNTIME
	if (platdata->runtime_pm_enabled &&
		port->dev->power.runtime_status != RPM_ACTIVE) {
		xgold_usif_add_list(uxp->trace_buf_list, count, s);

		if (!delayed_work_pending(&uxp->usif_rpm_work))
			schedule_delayed_work(&uxp->usif_rpm_work, 0);

		goto console_write_done;
	} else
		xgold_usif_runtime_pm_suspended(port, __func__);
#endif

	if ((uxp->tx_tps_cnt > 0) &&
		!oops_in_progress &&
		!port->sysrq) {
		/* TX from user side is in progress and we
		 * should not write to the console at the same
		 * time -> buffer the data to write them later.
		 */
		xgold_usif_add_list(uxp->trace_buf_list, count, s);

		goto console_write_done;
	}

	cancel_delayed_work(&uxp->usif_rpm_work);

	old_int_mask = ioread32(USIF_IMSC(port->membase));

	/* Reset all TX interrupts. */
	iowrite32(USIF_MIS_CLR_ALL_TX, USIF_ICR(port->membase));

	/* Write buffered logs first, if any. */
	if (uxp->trace_buf_list &&
		!list_empty(&uxp->trace_buf_list->list) &&
		locked) {
		struct xgold_usif_trace_buffer_list *tmp;
		struct list_head *pos, *q;

		list_for_each_entry(tmp, &uxp->trace_buf_list->list, list) {
			if (tmp && tmp->trace_buf &&
				tmp->buf_len > 0) {
				uart_console_write(port,
						tmp->trace_buf,
						tmp->buf_len,
						xgold_usif_putchar);
				kfree(tmp->trace_buf);
			}
		}

		list_for_each_safe(pos, q, &uxp->trace_buf_list->list) {
			tmp = list_entry(pos,
				struct xgold_usif_trace_buffer_list,
				list);
			list_del(pos);
			kfree(tmp);
		}
	}

	while (USIF_FIFO_STAT_TXFFS(ioread32(USIF_FIFO_STAT(port->membase))))
		barrier();

	uart_console_write(port, s, count, xgold_usif_putchar);

	/* Clear interrupt and set mask back */
	iowrite32(USIF_MIS_CLR_ALL_TX, USIF_ICR(port->membase));

	iowrite32(old_int_mask, USIF_IMSC(port->membase));

	xgold_usif_runtime_pm_autosuspend(port);

console_write_done:
	if (locked)
		spin_unlock(&port->lock);
	local_irq_restore(flags);
}

static void xgold_serial_usif_rpm_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct uart_usif_xgold_port *uxp = container_of(dwork,
			struct uart_usif_xgold_port, usif_rpm_work);

	struct uart_port *port = &uxp->port;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned long flags;

	xgold_usif_runtime_pm_resume(&uxp->port);

	spin_lock_irqsave(&uxp->port.lock, flags);

	if (usif_port_is_console(&uxp->port) &&
		uxp->tx_tps_cnt == 0 &&
		uxp->trace_buf_list &&
		!list_empty(&uxp->trace_buf_list->list)) {
		struct xgold_usif_trace_buffer_list *tmp;
		struct list_head *pos, *q;

		if (uxp->tx_tps_cnt > 0) {
			/* TX from user side is in progress ->
			 * reschedule the work queue.
			 */
			spin_unlock_irqrestore(&uxp->port.lock, flags);
			schedule_delayed_work(&uxp->usif_rpm_work,
					RESCHEDULE_WQ_DELAY);
			xgold_usif_runtime_pm_autosuspend(&uxp->port);
			return;
		}

		list_for_each_entry(tmp, &uxp->trace_buf_list->list, list) {
			if (tmp && tmp->trace_buf && tmp->buf_len > 0) {
				xgold_usif_direct_write(&uxp->port,
					tmp->trace_buf, tmp->buf_len);
				kfree(tmp->trace_buf);
			}
		}

		list_for_each_safe(pos, q, &uxp->trace_buf_list->list) {
			tmp = list_entry(pos,
					struct xgold_usif_trace_buffer_list,
					list);
			list_del(pos);
			kfree(tmp);
		}

	}

	if (uart_circ_chars_pending(xmit) > 0)
		xgold_usif_start_tx(&uxp->port);

	spin_unlock_irqrestore(&uxp->port.lock, flags);

	xgold_usif_runtime_pm_autosuspend(&uxp->port);
}

static void __init
xgold_usif_console_get_options(struct uart_port *port, int *baud,
		int *parity, int *bits, int *flow)
{
	unsigned int prtc_cfg;

	prtc_cfg = ioread32(USIF_PRTC_CFG(port->membase));

	*parity = 'n';
	if (USIF_PRTC_CFG_PAR(prtc_cfg)) {
		if (USIF_PRTC_CFG_ODD(prtc_cfg))
			*parity = 'o';
		else
			*parity = 'e';
	}

	/* By default, the flow control is disabled */
	*flow = 'n';

	/* By default, the character length is 8 */
	*bits = 0x08;

	/* By default, the USIF baud is set to 115200,
	 * so no need to calculate/set the baudrate configuration values */
	*baud = USIF_BAUD_DEFAULT;
}

static int __init xgold_usif_console_setup(struct console *co, char *options)
{
	struct uart_usif_xgold_port *uxp;
	struct uart_port *port;
	int baud = USIF_BAUD_DEFAULT;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int result = 0;

	uxp = xgold_usif_ports[co->index];
	port = &uxp->port;

	xgold_usif_runtime_pm_resume(port);

	/* Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have console
	 * support. */
	if (co->index >= SERIAL_USIF_NR)
		co->index = 0;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		xgold_usif_console_get_options(port, &baud, &parity, &bits,
				&flow);

	result = uart_set_options(port, co, baud, parity, bits, flow);

	xgold_usif_runtime_pm_autosuspend(port);

	return result;
}

static struct console xgold_usif_console = {
	.name = "ttyS",
	.write = xgold_usif_console_write,
	.device = uart_console_device,
	.setup = xgold_usif_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &xgold_usif_reg,
};

#if 0
static int __init xgold_usif_console_init(void)
{
	register_console(&xgold_usif_console);
	return 0;
}

console_initcall(xgold_usif_console_init);
#endif

static int __init xgold_usif_check_console_port(char *buf)
{
#define ASCII_OFFSET 48
	int len = strlen(xgold_usif_console.name);

	if (0 == strncmp(xgold_usif_console.name, buf, len)) {
		if (buf[len] >= 48 && buf[len] <= 57)
			/* Set the console port number. */
			xgold_usif_console.index =
					(int) (buf[len] - ASCII_OFFSET);
	}
	return 0;
}
early_param("console", xgold_usif_check_console_port);

#define SERIAL_SGOLD_USIF_CONSOLE	(&xgold_usif_console)
#else
#define SERIAL_SGOLD_USIF_CONSOLE	NULL
#endif /* CONFIG_SERIAL_XGOLD_USIF_CONSOLE */

#ifdef CONFIG_OF
static struct xgold_usif_platdata *xgold_usif_serial_get_platdata(
		struct device *dev, int extra)
{
	struct device_node *np = dev->of_node;
	struct xgold_usif_platdata *platdata;
	const char *str_datapath;
	const char *str_runtime_pm;
	unsigned int timer;
	int i, j, nb_of_it, it_wk, len, ret = 0;

	if (!np)
		return ERR_PTR(-EINVAL);

	platdata = kzalloc(sizeof(struct xgold_usif_platdata) + extra,
			GFP_KERNEL);
	if (!platdata)
		return ERR_PTR(-ENOMEM);

	if (of_property_read_u32(np, "intel,serial-port", &platdata->id)) {
		dev_err(dev, "Unable to read port property\n");
		ret = -ENODEV;
		goto free_platdata;
	}

	if (platdata->id > SERIAL_USIF_NR) {
		dev_err(dev, "%d port is > max allowed ports\n", platdata->id);
		ret = -ENODEV;
		goto free_platdata;
	}

	/* gpio */
	platdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(platdata->pinctrl))
		goto skip_pinctrl;

	platdata->pins_default = pinctrl_lookup_state(platdata->pinctrl,
						 PINCTRL_STATE_DEFAULT);
	if (IS_ERR(platdata->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	platdata->pins_sleep = pinctrl_lookup_state(platdata->pinctrl,
					       PINCTRL_STATE_SLEEP);
	if (IS_ERR(platdata->pins_sleep))
		dev_err(dev, "could not get sleep pinstate\n");

	platdata->pins_inactive = pinctrl_lookup_state(platdata->pinctrl,
					       "inactive");
	if (IS_ERR(platdata->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

skip_pinctrl:
	/* datapath */
	platdata->datapath = USIF_SERIAL_DATAPATH_PIO;
	ret =
	    of_property_read_string(np, "intel,usif-serial,datapath",
				    &str_datapath);

	if (!ret) {
		struct {
			enum usif_serial_datapath path;
			const char *name;
		} usif_serial_datapath_helper[] = {
			{ .path = USIF_SERIAL_DATAPATH_IDI, .name = "idi"},
			{ .path = USIF_SERIAL_DATAPATH_DMA, .name = "dma"},
			{ .path = USIF_SERIAL_DATAPATH_PIO, .name = "pio"},
		};
		unsigned i;
		for (i = 0; i < ARRAY_SIZE(usif_serial_datapath_helper); i++) {
			if (strcmp(usif_serial_datapath_helper[i].name,
							str_datapath) == 0) {
				platdata->datapath =
				    usif_serial_datapath_helper[i].path;
				break;
			}
		}

		if (!platdata->datapath)
			dev_warn(dev, "%s is an invalid Property of \"%s\"",
					str_datapath,
					"intel,usif-serial,datapath");
	}

	ret =
	    of_property_read_u32(np, "intel,usif-serial,ormc", &platdata->ormc);
	if (ret)
		platdata->ormc = 0;

	ret = of_property_read_u32(np, "intel,usif-serial,rmc", &platdata->rmc);
	if (ret)
		platdata->rmc = 1;

	ret = of_property_read_u32(np, "intel,usif-serial,rx,buffer-size",
				   &platdata->rx_buffer_size);
	if (ret)
		platdata->rx_buffer_size = 1024;

	ret = of_property_read_u32(np, "intel,usif-serial,rx,mrps",
				   &platdata->rx_mrps);
	if (ret)
		platdata->rx_mrps = -1;

	if (of_find_property(np, "intel,ictmo-cfg", &len)) {
		len /= 4;
		dev_info(dev, "intel,ictmo-cfg %d params\n", len);

		if (len % 2 != 0)
			dev_warn(dev, "Wrong format for ictmo-cfg property\n");

		/* Pairs (baud rate, ictmo), last element
		 * is NULL pair.
		 */
		platdata->ictmo = kzalloc(sizeof(unsigned) * (len + 1),
				GFP_KERNEL);
		if (!platdata->ictmo) {
			ret = -ENOMEM;
			goto free_platdata;
		}

		ret = of_property_read_u32_array(np,  "intel,ictmo-cfg",
				platdata->ictmo, len);
		if (ret) {
			dev_err(dev, "Error getting ictmo cfg params\n");
			goto free_ictmo;
		}
	}

	/* clocks */
	platdata->clk_reg = of_clk_get_by_name(np, "register");
	if (unlikely(IS_ERR(platdata->clk_reg)))
		platdata->clk_reg = NULL;

	platdata->clk_kernel = of_clk_get_by_name(np, "kernel");
	if (unlikely(IS_ERR(platdata->clk_kernel)))
		platdata->clk_kernel = NULL;

	/* regulator */
	/*TODO: Regulator are not needed on xg631 - to be checked */

	/* reset */
	platdata->reset = reset_control_get(dev, "usif");
	if (IS_ERR_OR_NULL(platdata->reset))
		dev_warn(dev, "missing usif reset controller\n");

	/* irqs */
	memset(platdata->irqs, -1, ARRAY_SIZE(platdata->irqs) * sizeof(short));

	nb_of_it = of_property_count_strings(np, "interrupt-names");
	if (nb_of_it <= 0) {
		dev_err(dev, "Unable to get usif interrupt-names property !");
		ret = -ENODEV;
		goto put_clk_register;
	}

	/* Filter the usif IRQ */
	for (i = 0, j = 0; i < nb_of_it; i++) {
		const char *name;
		if (!of_property_read_string_index
		    (np, "interrupt-names", i, &name)
		    && (strstr(name, "usif")))
			platdata->irqs[j++] = irq_of_parse_and_map(np, i);
	}

	if (of_find_property(np, "wakeup-source", NULL)) {
		it_wk = of_property_match_string(np, "interrupt-names",
						 "wk_int");
		if (it_wk > 0)
			platdata->irq_wk = irq_of_parse_and_map(np, it_wk);
		else
			platdata->irq_wk = platdata->irqs[0];
	} else
		platdata->irq_wk = 0;

	if (of_find_property(np, "intel,usif-debug", NULL))
		platdata->flags |= XGOLD_USIF_ALLOW_DEBUG;

	of_address_to_resource(np, 0, &platdata->res_io);

	if (of_property_read_u32(np, "pm,rpm_suspend_delay", &timer)) {
		platdata->modem_poll_timeout = USIF_FCI_POLL;
		dev_info(dev, "default 50ms fci poll\n");
	} else {
		platdata->modem_poll_timeout = msecs_to_jiffies(timer);
		dev_info(dev, "%dms fci poll\n", timer);
	}

	platdata->runtime_pm_enabled = 0;
	ret =
	    of_property_read_string(np, "pm,runtime",
				    &str_runtime_pm);
	if (!ret) {
		if (strcmp(str_runtime_pm, "enable") == 0)
			platdata->runtime_pm_enabled = 1;
	}
	dev_info(dev, "runtime_pm_enabled is %s\n",
		platdata->runtime_pm_enabled?"enable":"disable");

	platdata->runtime_pm_debug = 0;
	ret =
	    of_property_read_string(np, "pm,debug_rpm",
				    &str_runtime_pm);
	if (!ret) {
		if (strcmp(str_runtime_pm, "enable") == 0)
			platdata->runtime_pm_debug = 1;
	}
	dev_info(dev, "runtime_pm_debug is %s\n",
		platdata->runtime_pm_debug?"enable":"disable");

	if (of_property_read_u32(np, "pm,rpm_suspend_delay",
		&platdata->rpm_suspend_delay)) {
		platdata->rpm_suspend_delay = RUNTIME_SUSPEND_DELAY;
	}
	dev_info(dev, "rpm_suspend_delay set to %d\n",
			platdata->rpm_suspend_delay);

	platdata->rpm_auto_suspend_enable = 0;
	ret = of_property_read_string(np, "pm,auto_suspend_enable",
					&str_runtime_pm);
	if (!ret) {
		if (strcmp(str_runtime_pm, "enable") == 0)
			platdata->rpm_auto_suspend_enable = 1;
	}
	dev_info(dev, "auto_suspend_enable is %s\n",
		platdata->rpm_auto_suspend_enable?"enable":"disable");

	platdata->rpm_gen_uevent = 0;
	if (of_find_property(np, "pm,gen_uevent", NULL))
		platdata->rpm_gen_uevent = 1;

	return platdata;

put_clk_register:
	if (platdata->clk_reg)
		clk_put(platdata->clk_reg);
	if (platdata->clk_kernel)
		clk_put(platdata->clk_kernel);
free_ictmo:
	kfree(platdata->ictmo);
free_platdata:
	kfree(platdata);
	return ERR_PTR(ret);
}
#endif

static void xgold_usif_serial_put_platdata(struct device *dev)
{
	struct xgold_usif_platdata *platdata = dev_get_platdata(dev);
	if (platdata->clk_kernel)
		clk_put(platdata->clk_kernel);

	if (platdata->clk_reg)
		clk_put(platdata->clk_reg);
	kfree(platdata);
}

static bool xgold_usif_is_tx_ready(struct uart_port *port)
{
	bool ret;
	unsigned fifo_stat = ioread32(USIF_FIFO_STAT(port->membase));

	if (fifo_stat & USIF_FIFO_STAT_TPS_WE_MASK)
		ret = true;
	else
		ret = false;

	dev_dbg(port->dev, "fifo %s, fifo stat %x\n",
		(ret ? "ready" : "false"), fifo_stat);
	return ret;
}

static int xgold_usif_read_rps(struct uart_port *port)
{
	int ret;

	xgold_usif_runtime_pm_resume(port);

	ret = USIF_RPS_STAT_RPS(ioread32(USIF_RPS_STAT(port->membase)));

	xgold_usif_runtime_pm_autosuspend(port);

	return ret;
}

static int xgold_usif_write_tps(struct uart_port *port, unsigned cnt)
{
	if (xgold_usif_runtime_pm_suspended(port, __func__)) {
		/* Need to return since we are suspended and can't wake up
		* in IRQ locked context.
		*/
		dev_dbg(port->dev, "%s: cannot write TPS - USIF is RPM suspended %d\n",
				__func__, __LINE__);
		return 0;
	}

	while (!xgold_usif_is_tx_ready(port))
		;

	dev_dbg(port->dev, "Write TPS %x\n", cnt);
	iowrite32(cnt, USIF_TPS_CTRL(port->membase));

	xgold_usif_runtime_pm_autosuspend(port);
	return 0;
}

struct uart_usif_xgold_port *xgold_usif_add_port(struct device *dev,
						 struct uart_driver *drv,
						 int extra, int extra_platdata)
{
	struct uart_usif_xgold_port *uxp;
	struct xgold_usif_platdata *platdata;
	struct resource *res;
	void __iomem *ctrl;
	int ret = 0;

	uxp = kzalloc(sizeof(struct uart_usif_xgold_port) + extra, GFP_KERNEL);
	if (!uxp)
		return ERR_PTR(-ENOMEM);

	uxp->pm_ops = SERIAL_XGOLD_PM_OPS;
	uxp->read_rps = xgold_usif_read_rps;
	uxp->write_tps = xgold_usif_write_tps;
	uxp->is_tx_ready = xgold_usif_is_tx_ready;
	uxp->tx_tps_cnt = 0x0;
	uxp->dma_initialized = false;

	if (drv)
		uxp->drv = drv;
	else
		uxp->drv = &xgold_usif_reg;

#ifdef CONFIG_OF
	platdata = xgold_usif_serial_get_platdata(dev, extra_platdata);
#else
	platdata = dev_get_platdata(dev);
#endif
	if (IS_ERR_OR_NULL(platdata)) {
		ret = PTR_ERR(platdata);
		dev_err(dev, " This USIF will not be registered\n");
		goto free_uxp;
	}

	switch (platdata->datapath) {
	case USIF_SERIAL_DATAPATH_DMA:
		dev_warn(dev, "Data path property set to DMA\n");
		uxp->fifo_cfg = USIF_DMA_FIFO_SETUP;
		uxp->startup = xgold_usif_startup_dma;
		uxp->shutdown = xgold_usif_shutdown_dma;
		uxp->use_dma = true;
		tasklet_init(&uxp->tx_tasklet, xgold_usif_dma_tx_tasklet_func,
				(unsigned long)&uxp->port);
		break;
	case USIF_SERIAL_DATAPATH_IDI:
		dev_warn(dev, "Data path property set to IDI\n");
		uxp->fifo_cfg = USIF_IDI_FIFO_SETUP;
		uxp->startup = xgold_usif_startup_idi;
		break;
	case USIF_SERIAL_DATAPATH_PIO:
		uxp->fifo_cfg = USIF_PIO_FIFO_SETUP;
		uxp->startup = xgold_usif_startup_pio;
		break;
	default:
		dev_warn(dev, "Data path property set to PIO as default\n");
	}

	uxp->flags = platdata->flags;
	dev->platform_data = platdata;
	res = &platdata->res_io;

	ctrl = ioremap(res->start, resource_size(res));
	if (!ctrl) {
		ret = -ENOMEM;
		goto free_platdata;
	}

	uxp->port.dev = dev;
	uxp->port.mapbase = res->start;
	uxp->port.membase = ctrl;
	uxp->port.iotype = UPIO_MEM;
	uxp->port.irq = platdata->irqs[0];
	/* FIXME: Get the clock from platdata if NULL */
	if (platdata->clk_kernel == NULL)
		uxp->port.uartclk = 104000000;
	else
		uxp->port.uartclk = clk_get_rate(platdata->clk_kernel);
	uxp->port.flags = UPF_BOOT_AUTOCONF;
	uxp->port.line = platdata->id;
	uxp->port.ops = &usif_ops;
	uxp->use_rxbuf_b = false;
	uxp->is_console = false;
	mutex_init(&uxp->runtime_lock);
	/*
	 * Enable wakeup capability if possible
	 */
	if (platdata->irq_wk) {
		ret = device_init_wakeup(dev, true);
		if (ret)
			goto iounmap_ctrl;
	}

#ifdef CONFIG_SERIAL_XGOLD_CONSOLE
	if (xgold_usif_console.data == uxp->drv &&
			uxp->port.line == xgold_usif_console.index)
		uxp->is_console = true;

	if (uxp->is_console) {
		dev_dbg(dev, "Set uxp->port.line %d\n", uxp->port.line);
		xgold_usif_ports[uxp->port.line] = uxp;
	}
#endif
	/* Request the interrupt for wakeup of the console */
	if (platdata->irq_wk) {
		platdata->irq_wk_enabled = 0;
		ret = request_irq(platdata->irq_wk,
				xgold_usif_handle_wake_interrupt,
				IRQF_TRIGGER_NONE, uxp->name,
				&uxp->port);
		if (ret) {
			dev_err(dev, "%s: cannot request irq_wk %d\n",
					__func__, platdata->irq_wk);
			goto iounmap_ctrl;
		}
		disable_irq_nosync(platdata->irq_wk);
	}

	if (uxp->is_console) {
		/* Initialize the list for queuing traces */
		uxp->trace_buf_list = kzalloc(
			sizeof(struct xgold_usif_trace_buffer_list),
			GFP_KERNEL);
		if (uxp->trace_buf_list == NULL)
			goto iounmap_ctrl;
		INIT_LIST_HEAD(&uxp->trace_buf_list->list);
	}

	if (platdata->runtime_pm_enabled ||
		uxp->is_console) {
		/* Initialize the worker function on the global
		 * work queue */
		INIT_DELAYED_WORK(&uxp->usif_rpm_work,
			xgold_serial_usif_rpm_work);
	}

	xgold_usif_set_pinctrl_state(dev, platdata->pins_default);

	return uxp;

iounmap_ctrl:
	iounmap(ctrl);
free_platdata:
	xgold_usif_serial_put_platdata(dev);
free_uxp:
	kfree(uxp);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(xgold_usif_add_port);

void xgold_usif_remove_port(struct uart_usif_xgold_port *uxp)
{
	struct device *dev = uxp->port.dev;
	struct xgold_usif_platdata *platdata = dev_get_platdata(dev);

	device_init_wakeup(dev, false);

#ifdef CONFIG_SERIAL_XGOLD_CONSOLE
	if (platdata->runtime_pm_enabled) {
		if (platdata->irq_wk) {
			if (platdata->irq_wk_enabled)
				disable_irq_nosync(platdata->irq_wk);
			free_irq(platdata->irq_wk, &uxp->port);
			platdata->irq_wk_enabled = 0;
		}

		kfree(uxp->trace_buf_list);
		uxp->trace_buf_list = NULL;
	}

	dev_dbg(dev, "Remove uxp->port.line\n");
	xgold_usif_ports[uxp->port.line] = NULL;
#endif

	if (uxp->use_dma)
		tasklet_kill(&uxp->tx_tasklet);

	xgold_usif_set_pinctrl_state(dev, platdata->pins_inactive);

	del_timer(&uxp->modem_poll);
	iounmap(uxp->port.membase);
	xgold_usif_serial_put_platdata(dev);
}
EXPORT_SYMBOL(xgold_usif_remove_port);

struct uart_driver xgold_usif_reg = {
	.owner = THIS_MODULE,
	.driver_name = "ttyS",
	.dev_name = "ttyS",
	.major = SERIAL_USIF_MAJOR,
	.minor = SERIAL_USIF_MINOR,
	.nr = SERIAL_USIF_NR,
	.cons = SERIAL_SGOLD_USIF_CONSOLE,
};
