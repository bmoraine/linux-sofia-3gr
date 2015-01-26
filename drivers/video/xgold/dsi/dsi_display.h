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

#ifndef __DSI_DISPLAY_H__
#define __DSI_DISPLAY_H__

#include <linux/jiffies.h>
#include <linux/completion.h>

#define BYTES_TO_PIXELS(bytes, bpp) (DIV_ROUND_CLOSEST(bytes * 8, bpp))
#define PIXELS_TO_BYTES(pixels, bpp) (DIV_ROUND_CLOSEST(pixels * bpp, 8))

#define LCD_MSG_LP 1

enum {
	DIF_TX_DATA = 1,
	DIF_TX_PIXELS = 2,
};

enum dsi_gpio_t {
	DSI_GPIO_VHIGH = 0,
	DSI_GPIO_VLOW = 1,
	DSI_GPIO_RESET = 2,
};

enum dsi_mode_t {
	DSI_VIDEO = 0,
	DSI_CMD = 1,
};

enum dsi_pix_stream_t {
	DSI_PIX_BIT16P = 0,
	DSI_PIX_BIT18P = 1,
	DSI_PIX_BIT18L = 2,
	DSI_PIX_BIT24P = 3,
};

enum dsi_video_mode_t {
	DSI_ACTIVE = 0,
	DSI_PULSES = 1,
	DSI_EVENTS = 2,
	DSI_BURST = 3,
};

struct display_msg {
	struct list_head list;
	const char *name;
	unsigned char type;
	unsigned char *datas;
	int length;
	int delay;		/*in ms */
	unsigned int flags;
};

struct display_gpio {
	struct list_head list;
	const char *name;
	enum dsi_gpio_t type;
	int value;
	int delay;		/*in ms */
};

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
	int brmin;
	int brdef;
	int brmax;
	int nblanes;
	int id;
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
	int gpio_vhigh;
	int gpio_vlow;
	int gpio_reset;
	struct display_gpio *gpios_power_on;
	struct display_gpio *gpios_power_off;
	struct display_msg *msgs_sleep_in;
	struct display_msg *msgs_sleep_out;
	struct display_msg *msgs_init;
	struct display_msg *msgs_update;
	int (*panel_init)(struct dsi_display *display);
	void (*power_on)(struct dsi_display *display);
	int (*sleep_in)(struct dsi_display *display);
	int (*sleep_out)(struct dsi_display *display);
	void (*power_off)(struct dsi_display *display);
	struct dsi_display_if dif;
	void __iomem *regbase;
	struct dsi_sync_obj_s sync;
	struct dsi_irq irq;
	struct reset_control *dsi_reset;
};

static inline int dsi_completion_timeout_ms(struct completion *comp, int to)
{
	long jiffies = msecs_to_jiffies(to);

	return wait_for_completion_timeout(comp, jiffies);
}

int dsi_set_phy_lock(struct dsi_display *display);
void dsi_start_video(struct dsi_display *display);
int dsi_interrupt_setup(struct dsi_display *display);
int dsi_probe(struct dsi_display *display);
int dsi_irq_probe(struct dsi_display *display);
int dsi_irq_remove(struct dsi_display *display);
int dsi_init(struct dsi_display *display);
void dsi_config(struct dsi_display *display, int type);
int dsi_stop(struct dsi_display *display);
#endif
