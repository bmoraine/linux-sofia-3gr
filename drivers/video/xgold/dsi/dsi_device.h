/*
 *******************************************************************************
 *
 *  Component: XGold MIPI DSI driver
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

#ifndef __XGOLD_MIPI_DSI_H__
#define __XGOLD_MIPI_DSI_H__

#include <linux/completion.h>
#include <linux/rockchip_screen.h>
#include "dsi_display.h"

struct dsi_irq {
	int rx;
	int tx;
	int err;
	int rx_breq;
};

struct dsi_sync_obj_s {
	struct completion dsifin;
	int dsifin_to;
	struct completion dsitr1;
	int dsitr1_to;
};

static inline int dsi_completion_timeout_ms(struct completion *comp, int to)
{
	long jiffies = msecs_to_jiffies(to);

	return wait_for_completion_timeout(comp, jiffies);
}


struct xgold_mipi_dsi_device {
	struct device *dev;
	struct rockchip_screen screen;
	bool sys_state;
	int gpio_vhigh;
	int gpio_vlow;
	int gpio_reset;
	int gpio_id0;
	int gpio_id1;
	int (*panel_init)(struct xgold_mipi_dsi_device *mipi_dsi);
	void (*power_on)(struct xgold_mipi_dsi_device *mipi_dsi);
	int (*sleep_in)(struct xgold_mipi_dsi_device *mipi_dsi);
	int (*sleep_out)(struct xgold_mipi_dsi_device *mipi_dsi);
	void (*power_off)(struct xgold_mipi_dsi_device *mipi_dsi);
	void __iomem *regbase;
	struct reset_control *dsi_reset;
	struct dsi_irq irq;
	struct dsi_sync_obj_s sync;
	struct dsi_display *cur_display;
	struct list_head display_list;
};

void dsi_start_video(struct dsi_display *display);
void dsi_interrupt_setup(struct xgold_mipi_dsi_device *mipi_dsi);
int dsi_probe(struct xgold_mipi_dsi_device *mipi_dsi);
int dsi_irq_probe(struct xgold_mipi_dsi_device *mipi_dsi);
int dsi_irq_remove(struct xgold_mipi_dsi_device *mipi_dsi);
int dsi_init(struct xgold_mipi_dsi_device *mipi_dsi);
void dsi_config(struct xgold_mipi_dsi_device *mipi_dsi, int type);
int dsi_stop(struct xgold_mipi_dsi_device *mipi_dsi);
int dsi_read_cmd(struct xgold_mipi_dsi_device *mipi_dsi, u32 type, u8 *cmd,
		 unsigned int cmd_len, u8 *data, unsigned int data_len);
#endif
