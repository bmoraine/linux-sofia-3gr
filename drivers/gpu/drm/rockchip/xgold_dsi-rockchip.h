/*
 * Component: XGold MIPI DSI driver
 *
 * Copyright (C) 2014, Intel Mobile Communications GmbH.
 * Copyright (C) 2016, Fuzhou Rockchip Electronics Co.Ltd
 * Author: Wenlong Zhuang <daisen.zhuang@rock-chips.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __XGOLD_MIPI_DSI_H__
#define __XGOLD_MIPI_DSI_H__

#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>
#include <linux/completion.h>

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

struct dsi_display_if_mipi_dsi {
	unsigned int dc_clk_rate;

	int mode;
	int bitrate;
	int nblanes;
	int id;
	int eot; /* end of transmission */
	int gate; /* clock gating */

/* phy0 */
	int share;
	int m;
	int n;
	int pwup;
	int calib;
	int to_lp_hs_req;
/* phy1 */
	int to_lp_hs_dis;
	int to_lp_hs_eot;
	int to_hs_zero;
	int to_hs_flip;
	int lp_clk_div;
/* phy2 */
	int to_hs_clk_pre;
	int to_hs_clk_post;
	int data_delay;
	int clock_delay;
	int lp_tx_tfall;
/* phy3 */
	int en;
	int lp_tx_trise;
	int lp_tx_vref;

/* video mode settings*/
	/* Timing: All values in pixclocks, except pixclock (of course) */
	int hfp; /* horizontal front porch (cycles/bytes) */
	int hfp_lp; /* LP / HS */
	int hbp; /* horizontal back porch (bytes) */
	int hbp_lp;
	int hsa; /* horizontal sync active (bytes) */
	int hsa_lp;
	int vfp; /* vertical front porch (line) */
	int vbp; /* vertical back porch (line) */
	int vsa; /* vertical sync active (line) */
	int video_mode;
	int video_pixel;
	int bllp_time;
	int line_time;
	int display_preinit;
	u32 dsi_cfg_reg;
};

struct dsi_display_if {
	int type;
	struct dsi_display_if_mipi_dsi dsi;
};

struct dsi_display {
	int fps;	/* framerate */
	int xres;	/* pixel width */
	int yres;	/* pixel heigth */
	int bpp;
	int xdpi;	/* pixel density per inch in x direction */
	int ydpi;	/* pixel density per inch in y direction */
	struct dsi_display_if dif;
};

struct xgold_mipi_dsi {
	struct mipi_dsi_host dsi_host;
	struct drm_device *drm_dev;
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct drm_panel *panel;
	struct device *dev;

	int suspend;
	int state;
	u32 format;

	void __iomem *regbase;
	struct videomode vm;
	struct reset_control *dsi_reset;
	struct dsi_irq irq;
	struct dsi_sync_obj_s sync;
	struct dsi_display display;
	struct device_pm_platdata *pm_platdata;
};

static inline int dsi_completion_timeout_ms(struct completion *comp, int to)
{
	long jiffies = msecs_to_jiffies(to);

	return wait_for_completion_timeout(comp, jiffies);
}

int xgold_dsi_pre_init(struct xgold_mipi_dsi *dsi);
int xgold_dsi_init(struct xgold_mipi_dsi *dsi);
int xgold_dsi_stop(struct xgold_mipi_dsi *dsi);
void xgold_dsi_start_video(struct xgold_mipi_dsi *dsi);
void xgold_dsi_start_command(struct xgold_mipi_dsi *dsi);
int xgold_dsi_init_config(struct xgold_mipi_dsi *dsi,
			  struct mipi_dsi_device *device);
void xgold_dsi_set_display_mode(struct xgold_mipi_dsi *dsi,
				struct videomode *vm, int fps);
void xgold_dsi_send_short_packet(struct xgold_mipi_dsi *dsi,
				 const struct mipi_dsi_msg *msg);
void xgold_dsi_send_long_packet(struct xgold_mipi_dsi *dsi,
				const struct mipi_dsi_msg *msg);
#endif
