/*
 * Copyright (C) 2013 Intel Mobile Communications GmbH
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

#ifndef _XGOLD_SERIAL_H
#define _XGOLD_SERIAL_H

#include <linux/dmaengine.h>

/* USIF major device number */
#define SERIAL_USIF_MAJOR       4

/* USIF minor device number */
#define SERIAL_USIF_MINOR      64

#define SERIAL_USIF_NR		3
#define USIF_IRQS_NR 6

enum usif_serial_datapath {
	USIF_SERIAL_DATAPATH_PIO,
	USIF_SERIAL_DATAPATH_DMA,
	USIF_SERIAL_DATAPATH_IDI,
};

#define XGOLD_USIF_ALLOW_DEBUG		BIT(0)

/* Intel Xgold USIF */
struct xgold_usif_platdata {
	unsigned id;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	enum usif_serial_datapath datapath;
	unsigned rx_buffer_size;
	int rx_mrps;
	struct regulator *regulator;
	struct clk *clk_reg;
	struct clk *clk_kernel;
	struct reset_control *reset;
	short irqs[USIF_IRQS_NR];
	short irq_wk;
	short irq_wk_enabled;
	struct resource res_io;
	unsigned ormc;
	unsigned rmc;
	unsigned *ictmo;
	struct device_pm_platdata *pm_platdata;
	unsigned long flags;
	short runtime_pm_enabled;
	short runtime_pm_debug;
	short rpm_gen_uevent;
	unsigned int rpm_suspend_delay;
	short rpm_auto_suspend_enable;
	unsigned int modem_poll_timeout;
	unsigned long private[0] ____cacheline_aligned;
};

struct xgold_usif_trace_buffer_list {
	struct list_head list; /* kernel's list structure */
	int buf_len;
	char *trace_buf;
};

struct usif_dma_sg {
	struct scatterlist *sg_iotx;
	char *usif_dmabuf;
	bool dmabuf_owned;
	dma_cookie_t dma_cookie;
	struct uart_usif_xgold_port *uxp;
	int count;
};

struct uart_usif_xgold_port {
	struct uart_port port;
	unsigned hwid;
	unsigned fifo_hwid;
	struct uart_driver *drv;
	const struct dev_pm_ops *pm_ops;
	char name[16];
	unsigned fifo_cfg;
	unsigned int tx_progress;
	unsigned int tx_tps_cnt;
	unsigned long flags;
	unsigned int modem_status[2];
	struct timer_list modem_poll;
	bool is_console;

	/* For DMA mode */
	struct dma_chan *dma_rx_channel;
	struct dma_chan *dma_tx_channel;
	unsigned short use_dma;
	bool dma_initialized;
	struct usif_dma_sg *sg_rxbuf_a;
	struct usif_dma_sg *sg_rxbuf_b;
	bool use_rxbuf_b;
	struct usif_dma_sg *sg_txbuf;
	struct tasklet_struct tx_tasklet;
	char *dma_tx_buf;

	/* For runtime PM */
	struct xgold_usif_trace_buffer_list *trace_buf_list;
	struct delayed_work usif_rpm_work;
	struct mutex runtime_lock;
	unsigned long last_irq_wk_time;

	int (*startup)(struct uart_port *port);
	void (*shutdown)(struct uart_port *port);
	int (*read_rps)(struct uart_port *port);
	int (*write_tps)(struct uart_port *port, unsigned cnt);
	bool (*is_tx_ready)(struct uart_port *port);
	unsigned long private[0] ____cacheline_aligned;
};

static inline void *xgold_port_priv(struct uart_usif_xgold_port *port)
{
	return (void *)port->private;
}

static inline void *xgold_port_platdata_priv(struct xgold_usif_platdata
					     *platdata)
{
	return (void *)platdata->private;
}

static inline struct uart_usif_xgold_port *to_usif_port(struct uart_port *port)
{
	return container_of(port, struct uart_usif_xgold_port, port);
}

extern struct uart_driver xgold_usif_reg;
extern struct uart_usif_xgold_port *xgold_usif_add_port(struct device *,
							struct uart_driver *,
							int, int);
extern void xgold_usif_remove_port(struct uart_usif_xgold_port *);

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
extern struct device_state_pm_state usif_pm_states[];

/* USIF PM states index */
#define USIF_PM_STATE_D0	5
#define USIF_PM_STATE_D3	6

int xgold_usif_set_pm_state(struct device *, struct device_state_pm_state *);

struct device_state_pm_state *xgold_usif_get_initial_state(struct device *);
#endif

#endif /* _XGOLD_SERIAL_H */
