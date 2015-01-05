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
#ifndef __DCC_DISPLAY_H__
#define __DCC_DISPLAY_H__

struct xgold_lcd_periph_parameters {
	unsigned char cs_polarity;
	unsigned char cd_polarity;
	unsigned char wr_polarity;
	unsigned char rd_polarity;
	unsigned char hd_polarity;
	unsigned char vd_polarity;
};

struct xgold_lcd_timing_parameters {
	unsigned addr_delay_ns;
	unsigned cs_act_ns;
	unsigned data_delay_ns;
	unsigned wr_rd_act_ns;
	unsigned wr_rd_deact_ns;
	unsigned cs_deact_ns;
	unsigned access_cycle_ns;
};

enum {
	DIF_TX_DATA = 1,
	DIF_TX_PIXELS = 2,
};

#define LCD_MSG_CMD	0
#define LCD_MSG_DAT	1

struct display_msg {
	struct list_head list;
	const char *name;
	unsigned char header;
	unsigned char type;
	unsigned char *datas;
	int length;
	int delay;		/*in ms */
	unsigned int flags;
};

#define LCD_MSG_LP	1


enum {
	DCC_IF_MIPI_DBI = 1,
	DCC_IF_MIPI_DSI = 2,
};

#define DISPLAY_IS_MIPI_DBI_IF(_t_) \
		(_t_ == DCC_IF_MIPI_DBI)
#define DISPLAY_IS_MIPI_DSI_IF(_t_) \
		(_t_ == DCC_IF_MIPI_DSI)

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

struct dcc_display_if_mipi_dsi {
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
};

struct dcc_display_if_mipi_dbi {
	/* data on bus */
	int segments_per_pix;
	int bits_per_segment;
	/* timing */
	unsigned addr_delay_ns;
	unsigned cs_act_ns;
	unsigned data_delay_ns;
	unsigned wr_rd_act_ns;
	unsigned wr_rd_deact_ns;
	unsigned cs_deact_ns;
	unsigned access_cycle_ns;
	unsigned int mux_params[32];
	struct xgold_lcd_periph_parameters periph_params;
};

struct dcc_display_if {
	int type;
	int ncfg;
	union {
		struct dcc_display_if_mipi_dbi dbi;
		struct dcc_display_if_mipi_dsi dsi;
	} u;
};

struct display_reset {
	struct list_head list;
	int value;
	int mdelay;
};

struct dcc_display {
	int type;
	int fps;	/* framerate */
	int xres;	/* pixel width */
	int yres;	/* pixel heigth */
	int xdpi;	/* pixel density per inch in x direction */
	int ydpi;	/* pixel density per inch in y direction */
	unsigned char cs;
	struct display_reset *reset;
	struct display_msg *msgs_power_on;
	struct display_msg *msgs_power_off;
	struct display_msg *msgs_sleep_in;
	struct display_msg *msgs_sleep_out;
	struct display_msg *msgs_init;
	struct display_msg *msgs_update;
	int (*dif_init) (struct dcc_display *lcd);
	int (*dif_config) (struct dcc_display *lcd, int type);
	int (*dif_stop) (struct dcc_display *lcd);
	void (*panel_reset) (int gpio, struct dcc_display *lcd);
	int (*panel_init) (struct dcc_display *lcd);
	int (*power_on) (struct dcc_display *lcd);
	int (*sleep_in) (struct dcc_display *lcd);
	int (*sleep_out) (struct dcc_display *lcd);
	int (*power_off) (struct dcc_display *lcd);
	void (*send_cmd) (struct dcc_display *lcd,
					struct display_msg *msg);
	void (*frame_prepare) (struct dcc_display *lcd,
					int stride, int nlines);
	int (*frame_wfe) (struct dcc_display *lcd);
	int (*set_rate) (struct dcc_display *lcd, int val);
	int (*get_rate) (struct dcc_display *lcd);
	struct dcc_display_if dif;
};

int dcc_dsi_set_phy_lock(struct dcc_display *lcd);
void dcc_dsi_start_video(struct dcc_display *lcd);
#endif
