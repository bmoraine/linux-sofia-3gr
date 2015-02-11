/*
* Copyright (C) 2014-2015 Intel Mobile Communications GmbH
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

#ifndef _DT_BINDINGS_SOFIAFB_H_
#define _DT_BINDINGS_SOFIAFB_H_

#define GPIO		0
#define REGULATOR	1

#define PRMRY		1	/* primary display device */
#define EXTEND		2	/* extend display device */

#define NO_DUAL		0
#define ONE_DUAL	1
#define DUAL		2

#define OUT_P888	0	/* 24bit screen,connect to lcdc D0~D23 */
#define OUT_P666	1	/* 18bit screen,connect to lcdc D0~D17 */
#define OUT_P565	2
#define OUT_S888x	4
#define OUT_CCIR656	6
#define OUT_S888	8
#define OUT_S888DUMY	12
#define OUT_P16BPP4	24
#define OUT_D888_P666	0x21	/* 18bit screen,connect to
				 * lcdc D2~D7, D10~D15, D18~D23 */
#define OUT_D888_P565	0x22

#define SCREEN_NULL		0
#define SCREEN_RGB		1
#define SCREEN_LVDS		2
#define SCREEN_DUAL_LVDS	3
#define SCREEN_MCU		4
#define SCREEN_TVOUT		5
#define SCREEN_HDMI		6
#define SCREEN_MIPI		7
#define SCREEN_DUAL_MIPI	8
#define SCREEN_EDP		9
#define SCREEN_TVOUT_TEST	10

#define LVDS_8BIT_1	0
#define LVDS_8BIT_2	1
#define LVDS_8BIT_3	2
#define LVDS_6BIT	3

#define NO_MIRROR	0
#define X_MIRROR	1
#define Y_MIRROR	2
#define X_Y_MIRROR	3
#define ROTATE_90	4
#define ROTATE_180	8
#define ROTATE_270	12

#define COLOR_RGB	0
#define COLOR_YCBCR	1

/* fb win map */
#define FB_DEFAULT_ORDER		0
#define FB0_WIN2_FB1_WIN1_FB2_WIN0	12
#define FB0_WIN1_FB1_WIN2_FB2_WIN0	21
#define FB0_WIN2_FB1_WIN0_FB2_WIN1	102
#define FB0_WIN0_FB1_WIN2_FB2_WIN1	120
#define FB0_WIN0_FB1_WIN1_FB2_WIN2	210
#define FB0_WIN1_FB1_WIN0_FB2_WIN2	201


#endif
