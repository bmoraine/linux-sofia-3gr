/* Definitions of rockchip screen
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ROCKCHIP_SCREEN_H
#define __ROCKCHIP_SCREEN_H

#include <linux/fb.h>

struct pwr_ctr {
	char name[32];
	int type;
	int is_rst;
	int gpio;
	int atv_val;
	const char *rgl_name;
	int volt;
	int delay;
};

struct rockchip_disp_reset_list {
	struct list_head list;
	int value;
	int mdelay;
};

struct rockchip_disp_pwr_ctr_list {
	struct list_head list;
	struct pwr_ctr pwr_ctr;
};

/*
 * rockchip_screen description
 * type: LVDS,RGB,MIPI,MCU
 * lvds_fromat: lvds data format,set it if the screen is lvds
 * face: thi display output face,18bit,24bit,etc
 * ft: the time need to display one frame time
 */
struct rockchip_screen {
	u16 type;
	u16 lvds_format;
	u16 face;
	u16 color_mode;
	u8 vop_id;
	u8 screen_id;
	struct device *dev;
	struct fb_videomode mode;

	int interlace;
	int pixelrepeat;	/* For 480i/576i format,
				 * pixel is repeated twice. */
	u16 width;
	u16 height;
	u16 ft;
	int *dsp_lut;

	u8 hdmi_resolution;
	u8 mcu_wrperiod;
	u8 mcu_usefmk;
	u8 mcu_frmrate;

	u8 pin_hsync;
	u8 pin_vsync;
	u8 pin_den;
	u8 pin_dclk;

	/* Swap rule */
	u8 swap_gb;
	u8 swap_rg;
	u8 swap_rb;
	u8 swap_delta;
	u8 swap_dumy;

	/*
	 * horizontal display start position on the sceen,
	 * then can be changed by application
	 */
	int xpos;
	int ypos;
	/* horizontal and vertical display size on screen
	 * they can be changed by application
	 */
	int xsize;
	int ysize;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
	struct rockchip_disp_reset_list *resetlist;
	int gpio_rst;
#else
	struct list_head pwrlist_head;
#endif
	struct rockchip_screen *ext_screen;
	/* Operation function */
	int (*init)(void);
	int (*standby)(u8 enable);
	int (*refresh)(u8 arg);
	int (*scandir)(u16 dir);
	int (*disparea)(u8 area);
};

size_t get_fb_size(void);
int rockchip_get_prmry_screen(struct rockchip_screen *screen);
int rockchip_set_prmry_screen(struct rockchip_screen *screen);
int rockchip_disp_pwr_enable(struct rockchip_screen *screen);
int rockchip_disp_pwr_disable(struct rockchip_screen *screen);

#endif
