/*
 * rockchip RGA(2D raster graphic acceleration unit) hardware driver.
 *
 * Copyright (C) 2014 Rockchip Electronics Co., Ltd.
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

#ifndef _RGA_DRIVER_H_
#define _RGA_DRIVER_H_

#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/file.h>
#include <linux/kthread.h>
#include "../../drivers/staging/android/sw_sync.h"

#define RGA_BLIT_SYNC	0x5017
#define RGA_BLIT_ASYNC  0x5018
#define RGA_FLUSH       0x5019
#define RGA_GET_RESULT  0x501a
#define RGA_GET_VERSION 0x501b

#define RGA_REG_CTRL_LEN    0x8    /* 8  */
#define RGA_REG_CMD_LEN     0x20   /* 32 */
#define RGA_CMD_BUF_SIZE    0x700  /* 16*28*4 */

#define RGA_OUT_OF_RESOURCES    -10
#define RGA_MALLOC_ERROR        -11

/* RGA process mode enum */
enum {
	bitblt_mode = 0x0,
	color_palette_mode = 0x1,
	color_fill_mode = 0x2,
	line_point_drawing_mode = 0x3,
	blur_sharp_filter_mode = 0x4,
	pre_scaling_mode = 0x5,
	update_palette_table_mode = 0x6,
	update_patten_buff_mode = 0x7,
};

enum {
	rop_enable_mask = 0x2,
	dither_enable_mask = 0x8,
	fading_enable_mask = 0x10,
	PD_enbale_mask = 0x20,
};

enum  {
	yuv2rgb_mode0 = 0x0, /* BT.601 MPEG */
	yuv2rgb_mode1 = 0x1, /* BT.601 JPEG */
	yuv2rgb_mode2 = 0x2, /* BT.709      */
};

/* RGA rotate mode */
enum  {
	rotate_mode0 = 0x0, /* no rotate */
	rotate_mode1 = 0x1, /* rotate    */
	rotate_mode2 = 0x2, /* x_mirror  */
	rotate_mode3 = 0x3, /* y_mirror  */
};

enum  {
	color_palette_mode0 = 0x0, /* 1K */
	color_palette_mode1 = 0x1, /* 2K */
	color_palette_mode2 = 0x2, /* 4K */
	color_palette_mode3 = 0x3, /* 8K */
};

/*
//          Alpha    Red     Green   Blue
{  4, 32, {{32,24,   8, 0,  16, 8,  24,16 }}, GGL_RGBA },
{  4, 24, {{ 0, 0,   8, 0,  16, 8,  24,16 }}, GGL_RGB  },
{  3, 24, {{ 0, 0,   8, 0,  16, 8,  24,16 }}, GGL_RGB  },
{  4, 32, {{32,24,  24,16,  16, 8,   8, 0 }}, GGL_BGRA },
{  2, 16, {{ 0, 0,  16,11,  11, 5,   5, 0 }}, GGL_RGB  },
{  2, 16, {{ 1, 0,  16,11,  11, 6,   6, 1 }}, GGL_RGBA },
{  2, 16, {{ 4, 0,  16,12,  12, 8,   8, 4 }}, GGL_RGBA },
{  3, 24, {{ 0, 0,  24,16,  16, 8,   8, 0 }}, GGL_BGR  },

*/
enum {
	RGA_FORMAT_RGBA_8888 = 0x0,
	RGA_FORMAT_RGBX_8888 = 0x1,
	RGA_FORMAT_RGB_888 = 0x2,
	RGA_FORMAT_BGRA_8888 = 0x3,
	RGA_FORMAT_RGB_565 = 0x4,
	RGA_FORMAT_RGBA_5551 = 0x5,
	RGA_FORMAT_RGBA_4444 = 0x6,
	RGA_FORMAT_BGR_888 = 0x7,
	RGA_FORMAT_YCBCR_422_SP = 0x8,
	RGA_FORMAT_YCBCR_422_P = 0x9,
	RGA_FORMAT_YCBCR_420_SP = 0xa,
	RGA_FORMAT_YCBCR_420_P = 0xb,
	RGA_FORMAT_YCRCB_422_SP = 0xc,
	RGA_FORMAT_YCRCB_422_P = 0xd,
	RGA_FORMAT_YCRCB_420_SP = 0xe,
	RGA_FORMAT_YCRCB_420_P = 0xf,
	RGA_FORMAT_BPP1 = 0x10,
	RGA_FORMAT_BPP2 = 0x11,
	RGA_FORMAT_BPP4 = 0x12,
	RGA_FORMAT_BPP8 = 0x13,
};

struct rga_img_info_t  {
	unsigned int yrgb_addr;	/* yrgb    mem addr      */
	unsigned int uv_addr;	/* cb/cr   mem addr      */
	unsigned int v_addr;	/* cr      mem addr      */
	unsigned int format;	/*definition by RGA_FORMAT*/
	unsigned short act_w;
	unsigned short act_h;
	unsigned short x_offset;
	unsigned short y_offset;
	unsigned short vir_w;
	unsigned short vir_h;
	unsigned short endian_mode;
	unsigned short alpha_swap;
};

struct mdp_img_act  {
	unsigned short w;
	unsigned short h;
	short x_off;
	short y_off;
};

struct RANGE  {
	unsigned short min;
	unsigned short max;
};

struct POINT  {
	unsigned short x;
	unsigned short y;
};

struct RECT  {
	unsigned short xmin;
    /* width - 1 */
	unsigned short xmax;
	unsigned short ymin;
    /* height - 1 */
	unsigned short ymax;
};

struct RGB  {
	unsigned char r;
	unsigned char g;
	unsigned char b;
	unsigned char res;
};

struct MMU  {
	unsigned char mmu_en;
	uint32_t base_addr;
    /* [0] mmu enable [1] src_flush [2] dst_flush
       [3] CMD_flush [4~5] page size */
	uint32_t mmu_flag;
};

struct COLOR_FILL  {
	short gr_x_a;
	short gr_y_a;
	short gr_x_b;
	short gr_y_b;
	short gr_x_g;
	short gr_y_g;
	short gr_x_r;
	short gr_y_r;
};

struct FADING  {
	uint8_t b;
	uint8_t g;
	uint8_t r;
	uint8_t res;
};

struct line_draw_t  {
    /* LineDraw_start_point                */
	struct POINT start_point;
    /* LineDraw_end_point                  */
	struct POINT end_point;
    /* LineDraw_color                      */
	uint32_t color;
    /* (enum) LineDrawing mode sel         */
	uint32_t flag;
    /* range 1~16 */
	uint32_t line_width;
};

struct rga_req {
    /* (enum) process mode sel */
	uint8_t render_mode;
    /* src image info */
	struct rga_img_info_t src;
    /* dst image info */
	struct rga_img_info_t dst;
    /* patten image info */
	struct rga_img_info_t pat;
    /* rop4 mask addr */
	uint32_t rop_mask_addr;
    /* LUT addr */
	uint32_t LUT_addr;
    /* dst clip window default value is dst_vir */
	struct RECT clip;

	/* value from [0, w-1] / [0, h-1] */
    /* dst angle  default value 0  16.16 scan from table */
	int32_t sina;
    /* dst angle  default value 0  16.16 scan from table */
	int32_t cosa;
    /* alpha rop process flag           */
	uint16_t alpha_rop_flag;

	/* ([0] = 1 alpha_rop_enable)       */
	/* ([1] = 1 rop enable)             */
	/* ([2] = 1 fading_enable)          */
	/* ([3] = 1 PD_enable)              */
	/* ([4] = 1 alpha cal_mode_sel)     */
	/* ([5] = 1 dither_enable)          */
	/* ([6] = 1 gradient fill mode sel) */
	/* ([7] = 1 AA_enable)              */
    /* 0 nearst / 1 bilnear / 2 bicubic */
	uint8_t scale_mode;
    /* color key max */
	uint32_t color_key_max;
    /* color key min */
	uint32_t color_key_min;
    /* foreground color */
	uint32_t fg_color;
    /* background color */
	uint32_t bg_color;
    /* color fill use gradient */
	struct COLOR_FILL gr_color;
	struct line_draw_t line_draw_info;
	struct FADING fading;
    /* porter duff alpha mode sel */
	uint8_t PD_mode;
    /* global alpha value */
	uint8_t alpha_global_value;
    /* rop2/3/4 code  scan from rop code table */
	uint16_t rop_code;
    /* [2] 0 blur 1 sharp / [1:0] filter_type */
	uint8_t bsfilter_flag;
    /* (enum) color palatte  0/1bpp, 1/2bpp 2/4bpp 3/8bpp */
	uint8_t palette_mode;
    /* (enum) BT.601 MPEG / BT.601 JPEG / BT.709  */
	uint8_t yuv2rgb_mode;
    /* 0/big endian 1/little endian */
	uint8_t endian_mode;
    /* (enum) rotate mode  */
	uint8_t rotate_mode;

	/* 0x0,     no rotate  */
	/* 0x1,     rotate     */
	/* 0x2,     x_mirror   */
	/* 0x3,     y_mirror   */
    /* 0 solid color / 1 patten color */
	uint8_t color_fill_mode;
    /* mmu information */
	struct MMU mmu_info;
    /* ([0~1] alpha mode)       */
	uint8_t alpha_rop_mode;

	    /* ([2~3] rop   mode)       */
	    /* ([4]   zero  mode en)    */
	    /* ([5]   dst   alpha mode) */
	uint8_t src_trans_mode;
	struct sg_table *sg_src;
	struct sg_table *sg_dst;
};

struct TILE_INFO  {
	int64_t matrix[4];
    /* x axis tile num / tile size is 8x8 pixel */
	uint16_t tile_x_num;
    /* y axis tile num */
	uint16_t tile_y_num;
    /* dst pos x = (xstart - xoff) default value 0 */
	int16_t dst_x_tmp;
    /* dst pos y = (ystart - yoff) default value 0 */
	int16_t dst_y_tmp;
	uint16_t tile_w;
	uint16_t tile_h;
	int16_t tile_start_x_coor;
	int16_t tile_start_y_coor;
	int32_t tile_xoff;
	int32_t tile_yoff;
	int32_t tile_temp_xstart;
	int32_t tile_temp_ystart;

	    /* src tile incr */
	int32_t x_dx;
	int32_t x_dy;
	int32_t y_dx;
	int32_t y_dy;
	struct mdp_img_act dst_ctrl;
};

struct rga_mmu_buf_t {
	int32_t front;
	int32_t back;
	int32_t size;
	int32_t curr;
	unsigned int *buf;
	unsigned int *buf_virtual;
	struct page **pages;
};
/**
 * struct for process session which connect to rga
 *
 * @author ZhangShengqin (2012-2-15)
 */
struct rga_session {
	    /* a linked list of data so we can access them for debugging */
	struct list_head list_session;

	    /* a linked list of register data waiting for process */
	struct list_head waiting;

	    /* a linked list of register data in processing */
	struct list_head running;

	    /* all coommand this thread done */
	atomic_t done;
	wait_queue_head_t wait;
	pid_t pid;
	atomic_t task_running;
	atomic_t num_done;
	/*
	struct kthread_worker update_regs_worker;
	struct task_struct *update_regs_thread;
	struct kthread_work update_regs_work;
	atomic_t queue_work_done;
	wait_queue_head_t queue_work_wait;
	*/
};

struct rga_reg {
	struct rga_session *session;
	struct list_head session_link;	/* link to rga service session */
	struct list_head status_link;	/* link to register set list */
	uint32_t sys_reg[RGA_REG_CTRL_LEN];
	uint32_t cmd_reg[RGA_REG_CMD_LEN];
	uint32_t *MMU_base;
	struct sync_fence *fence;
	struct sync_fence *dst_fence;
	short dst_fence_fd;
	int MMU_len;
};

struct rga_service_info {
    /* mutex */
	struct mutex lock;
	/* timer for power off */
	struct timer_list timer;
	/* link to link_reg in struct vpu_reg */
	struct list_head waiting;
	/* link to link_reg in struct vpu_reg */
	struct list_head running;
	/* link to link_reg in struct vpu_reg */
	struct list_head done;
	/* link to list_session in struct vpu_session */
	struct list_head session;
	atomic_t total_running;
	struct rga_reg *reg;
	uint32_t cmd_buff[28 * 8];	/* cmd_buff for rga */
	uint32_t *pre_scale_buf;
	atomic_t int_disable;	/* 0 int enable 1 int disable  */
	atomic_t cmd_num;
	atomic_t rga_working;
	bool enable;
	short src_fence_fd;
	short src_fence_flag;
	short dst_fence_fd;
	short dst_fence_flag;
	struct sync_fence *dst_fence;
	struct sw_sync_timeline *timeline;
	uint32_t timeline_max;
	struct workqueue_struct *fence_workqueue;
	struct delayed_work fence_delayed_work;
	uint16_t timeout_num;
	atomic_t delay_work_already_queue;
	atomic_t interrupt_flag;
	atomic_t interrupt_timeout_flag;
	/* mutex */
	struct mutex mutex;
};

extern struct rga_service_info rga_service;

#if defined(CONFIG_ION_XGOLD)
extern struct ion_client *xgold_ion_client_create(const char *name);
#endif
#if defined(CONFIG_ION_ROCKCHIP)
extern struct ion_client *rockchip_ion_client_create(const char *name);
#endif

#define RGA_BASE                 0xe2500000

/*General Registers*/
#define RGA_SYS_CTRL             0x000
#define RGA_CMD_CTRL             0x004
#define RGA_CMD_ADDR             0x008
#define RGA_STATUS               0x00c
#define RGA_INT                  0x010
#define RGA_AXI_ID               0x014
#define RGA_MMU_STA_CTRL         0x018
#define RGA_MMU_STA              0x01c

/*Command code start*/
#define RGA_MODE_CTRL            0x100

/*Source Image Registers*/
#define RGA_SRC_Y_MST            0x104
#define RGA_SRC_CB_MST           0x108
#define RGA_MASK_READ_MST        0x108	/*repeat*/
#define RGA_SRC_CR_MST           0x10c
#define RGA_SRC_VIR_INFO         0x110
#define RGA_SRC_ACT_INFO         0x114
#define RGA_SRC_X_PARA           0x118
#define RGA_SRC_Y_PARA           0x11c
#define RGA_SRC_TILE_XINFO       0x120
#define RGA_SRC_TILE_YINFO       0x124
#define RGA_SRC_TILE_H_INCR      0x128
#define RGA_SRC_TILE_V_INCR      0x12c
#define RGA_SRC_TILE_OFFSETX     0x130
#define RGA_SRC_TILE_OFFSETY     0x134
#define RGA_SRC_BG_COLOR         0x138
#define RGA_SRC_FG_COLOR         0x13c
#define RGA_LINE_DRAWING_COLOR   0x13c	/*repeat*/
#define RGA_SRC_TR_COLOR0        0x140
#define RGA_CP_GR_A              0x140	/*repeat*/
#define RGA_SRC_TR_COLOR1        0x144
#define RGA_CP_GR_B              0x144	/*repeat*/

#define RGA_LINE_DRAW            0x148
#define RGA_PAT_START_POINT      0x148	/*repeat*/

/*Destination Image Registers*/
#define RGA_DST_MST              0x14c
#define RGA_LUT_MST              0x14c	/*repeat*/
#define RGA_PAT_MST              0x14c	/*repeat*/
#define RGA_LINE_DRAWING_MST     0x14c	/*repeat*/

#define RGA_DST_VIR_INFO         0x150

#define RGA_DST_CTR_INFO         0x154
#define RGA_LINE_DRAW_XY_INFO    0x154	/*repeat*/

/*Alpha/ROP Registers*/
#define RGA_ALPHA_CON            0x158

#define RGA_PAT_CON              0x15c
#define RGA_DST_VIR_WIDTH_PIX    0x15c	/*repeat*/

#define RGA_ROP_CON0             0x160
#define RGA_CP_GR_G              0x160	/*repeat*/
#define RGA_PRESCL_CB_MST        0x160	/*repeat*/

#define RGA_ROP_CON1             0x164
#define RGA_CP_GR_R              0x164	/*repeat*/
#define RGA_PRESCL_CR_MST        0x164	/*repeat*/

/*MMU Register*/
#define RGA_FADING_CON           0x168
#define RGA_MMU_CTRL             0x168	/*repeat*/
#define RGA_MMU_TBL              0x16c	/*repeat*/
#define RGA_YUV_OUT_CFG          0x170
#define RGA_DST_UV_MST           0x174
#define RGA_BLIT_COMPLETE_EVENT 1
long rga_ioctl_kernel(struct rga_req *req);

#endif	/*_RGA_DRIVER_H_*/
