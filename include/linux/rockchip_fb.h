/* Definitions of rockchip framebuffer
 *
 * Copyright (C) 2012-2015 Rockchip Electronics Co., Ltd.
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

#ifndef __ROCKCHIP_FB_H
#define __ROCKCHIP_FB_H

#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/rockchip_screen.h>
#if defined(CONFIG_OF)
#include <dt-bindings/sofiafb/sofia_fb.h>
#endif
#include "../../drivers/staging/android/sw_sync.h"
#include "../../drivers/staging/android/ion/ion.h"
#include <linux/file.h>
#include <linux/kthread.h>

#define SFA_MAX_VOP_SUPPORT	2
#define SFA_MAX_WIN_SUPPORT	5
#define SFA_MAX_FB_SUPPORT	5
#define SFA_MAX_BUF_NUM		11
#define SFA_WIN_MAX_AREA	4

#define SFA_FBIOSET_HWC_ADDR			0x4624
#define SFA_FBIOPUT_NUM_BUFFERS			0x4625
#define SFA_FBIOPUT_COLOR_KEY_CFG		0x4626

#define SFA_FBIOSET_CONFIG_DONE			0x4628
#define SFA_FBIOSET_VSYNC_ENABLE		0x4629
#define SFA_FBIOGET_DSP_ADDR			0x4630
#define SFA_FBIOGET_LIST_STA			0X4631
#define SFA_FBIOGET_IOMMU_STA			0x4632
#define SFA_FBIOSET_CLEAR_FB			0x4633

#define SFA_FBIOGET_PANEL_SIZE			0x5001
#define SFA_FBIOSET_YUV_ADDR			0x5002
#define SFA_FBIOGET_DMABUF_FD			0x5003
#define SFA_FBIOSET_DMABUF_FD			0x5004
#define SFA_FB_IOCTL_SET_I2P_ODD_ADDR		0x5005
#define SFA_FB_IOCTL_SET_I2P_EVEN_ADDR		0x5006
#define SFA_FBIOSET_OVERLAY_STA			0x5018
#define SFA_FBIOGET_OVERLAY_STA			0X4619
#define SFA_FBIOSET_ENABLE			0x5019
#define SFA_FBIOGET_ENABLE			0x5020

/* rockchip fb events */
#define SFA_LF_STATUS_FC		0xef
#define SFA_LF_STATUS_FR		0xee
#define SFA_LF_STATUS_NC		0xfe
#define SFA_LF_MAX_TIMEOUT		(1600000UL << 6)	/* >0.64s */

/*
 * x y mirror or rotate mode
 */
#define NO_MIRROR	0
#define X_MIRROR	1	/* up-down flip */
#define Y_MIRROR	2	/* left-right flip */
#define X_Y_MIRROR	3	/* the same as rotate 180 degrees */
#define ROTATE_90	4	/* clockwise rotate 90 degrees */
#define ROTATE_180	8	/* rotate 180 degrees
				 * It is recommended to use X_Y_MIRROR
				 * rather than ROTATE_180
				 */
#define ROTATE_270	12	/* clockwise rotate 270 degrees */


/*
 * pixel format definitions,this is copy
 * from android/system/core/include/system/graphics.h
 */

enum {
	HAL_PIXEL_FORMAT_RGBA_8888 = 1,
	HAL_PIXEL_FORMAT_RGBX_8888 = 2,
	HAL_PIXEL_FORMAT_RGB_888 = 3,
	HAL_PIXEL_FORMAT_RGB_565 = 4,
	HAL_PIXEL_FORMAT_BGRA_8888 = 5,
	HAL_PIXEL_FORMAT_RGBA_5551 = 6,
	HAL_PIXEL_FORMAT_RGBA_4444 = 7,

	/* 0x8 - 0xFF range unavailable */

	/*
	 * 0x100 - 0x1FF
	 *
	 * This range is reserved for pixel formats that are specific to
	 * the HAL implementation.  Implementations can use any value
	 * in this range to communicate video pixel formats between their
	 * HAL modules.  These formats must not have an alpha channel.
	 * Additionally, an EGLimage created from a gralloc buffer of one
	 * of these formats must be supported for use with the
	 * GL_OES_EGL_image_external OpenGL ES extension.
	 */

	/*
	 * Android YUV format:
	 *
	 * This format is exposed outside of the HAL to software decoders and
	 * applications.  EGLImageKHR must support it in conjunction with the
	 * OES_EGL_image_external extension.
	 *
	 * YV12 is a 4:2:0 YCrCb planar format comprised of a WxH Y plane
	 * followed by (W/2) x (H/2) Cr and Cb planes.
	 *
	 * This format assumes
	 * - an even width
	 * - an even height
	 * - a horizontal stride multiple of 16 pixels
	 * - a vertical stride equal to the height
	 *
	 *   y_size = stride * height
	 *   c_size = ALIGN(stride/2, 16) * height/2
	 *   size = y_size + c_size * 2
	 *   cr_offset = y_size
	 *   cb_offset = y_size + c_size
	 *
	 */
	HAL_PIXEL_FORMAT_YV12 = 0x32315659,	/* YCrCb 4:2:0 Planar */

	/* Legacy formats (deprecated), used by ImageFormat.java */
	HAL_PIXEL_FORMAT_YCbCr_422_SP = 0x10,	/* NV16 */
	HAL_PIXEL_FORMAT_YCrCb_420_SP = 0x11,	/* NV21 */
	HAL_PIXEL_FORMAT_YCbCr_422_I = 0x14,	/* YUY2 */
	HAL_PIXEL_FORMAT_YCrCb_NV12 = 0x20,	/* YUY2 */
	HAL_PIXEL_FORMAT_YCrCb_NV12_VIDEO = 0x21,	/* YUY2 */

	HAL_PIXEL_FORMAT_YCrCb_NV12_10 = 0x22,	/* YUV420_10bit */
	HAL_PIXEL_FORMAT_YCbCr_422_SP_10 = 0x23,	/* YUV422_10bit */
	HAL_PIXEL_FORMAT_YCrCb_420_SP_10 = 0x24,	/*YUV444_10bit */

	HAL_PIXEL_FORMAT_YCrCb_444 = 0x25,	/* yuv444 */

};

/* display data format */
enum data_format {
	ARGB888 = 0,
	RGB888,
	RGB565,
	YUV420 = 4,
	YUV422,
	YUV444,
	XRGB888,
	XBGR888,
	ABGR888,
	YUV420_A = 10,
	YUV422_A,
	YUV444_A,
	YUV420_NV21,
};

enum {
	SCALE_NONE = 0x0,
	SCALE_UP   = 0x1,
	SCALE_DOWN = 0x2
};

enum bcsh_bcs_mode {
	BRIGHTNESS = 0x0,
	CONTRAST = 0x1,
	SAT_CON = 0x2,
};

enum bcsh_hue_mode {
	H_SIN = 0x0,
	H_COS = 0x1,
};

struct rockchip_fb_rgb {
	struct fb_bitfield red;
	struct fb_bitfield green;
	struct fb_bitfield blue;
	struct fb_bitfield transp;
};

struct rockchip_fb_vsync {
	wait_queue_head_t wait;
	ktime_t timestamp;
	bool active;
	bool irq_stop;
	int irq_refcount;
	struct mutex irq_lock;	/* mutex for irq */
	struct task_struct *thread;
};

struct color_key_cfg {
	u32 win0_color_key_cfg;
	u32 win1_color_key_cfg;
	u32 win2_color_key_cfg;
};

struct rockchip_vop_bcsh {
	bool enable;
	u16 brightness;
	u16 contrast;
	u16 sat_con;
	u16 sin_hue;
	u16 cos_hue;
};

struct rockchip_vop_post_cfg {
	u32 xpos;
	u32 ypos;
	u32 xsize;
	u32 ysize;
};

struct rockchip_fb_area_par {
	u8  data_format;        /* area data fmt */
	short ion_fd;
	unsigned long phy_addr;
	short acq_fence_fd;
	u16 x_offset;
	u16 y_offset;
	u16 xpos;	/* start point in panel */
	u16 ypos;
	u16 xsize;	/* display window width/height */
	u16 ysize;
	u16 xact;	/* origin display window size */
	u16 yact;
	u16 xvir;	/* virtual width/height */
	u16 yvir;
	u8  fbdc_en;
	u8  fbdc_cor_en;
	u8  fbdc_data_format;
	u16 reserved0;
	u32 reserved1;
};

struct rockchip_fb_win_par {
	u8  win_id;
	u8  z_order;		/*win sel layer*/
	u8  alpha_mode;
	u16 g_alpha_val;
	u8  mirror_en;
	struct rockchip_fb_area_par area_par[SFA_WIN_MAX_AREA];
	u32 reserved0;
};

struct rockchip_fb_win_cfg_data {
	u8 wait_fs;
	short ret_fence_fd;
	short rel_fence_fd[SFA_MAX_BUF_NUM];
	struct  rockchip_fb_win_par win_par[SFA_MAX_WIN_SUPPORT];
	struct  rockchip_vop_post_cfg post_cfg;
};

struct rockchip_vop_win_area {
	bool state;
	enum data_format format;
	u8 fmt_cfg;
	u8 swap_rb;
	u8 swap_uv;
	u32 y_offset;		/* yuv/rgb offset -->LCDC_WINx_YRGB_MSTx */
	u32 c_offset;		/* cb cr offset -->LCDC_WINx_CBR_MSTx */
	u16 xpos;		/* start point in panel -->LCDC_WINx_DSP_ST */
	u16 ypos;
	u16 xsize;		/* display window width/height */
	u16 ysize;
	u16 xact;		/* origin display window size */
	u16 yact;
	u16 xvir;		/* virtual width/height -->LCDC_WINx_VIR */
	u16 yvir;

	u32 dsp_stx;
	u32 dsp_sty;
	u32 y_vir_stride;
	u32 uv_vir_stride;
	u32 y_addr;
	u32 uv_addr;
	u32 buff_len;

	unsigned long smem_start;
	unsigned long cbr_start;	/* Cbr memory start address */
#if defined(CONFIG_ION)
	struct ion_handle *ion_hdl;
	int dma_buf_fd;
	struct dma_buf *dma_buf;
#endif

	struct sync_fence *acq_fence;
};

struct rockchip_vop_win {
	char name[5];
	int id;
	bool state;		/* on or off */
	bool last_state;	/* on or off */
	u32 pseudo_pal[16];
	int z_order;		/* win sel layer */

	u8 area_num;
	u8 area_buf_num;

	u32 scale_yrgb_x;
	u32 scale_yrgb_y;
	u32 scale_cbcr_x;
	u32 scale_cbcr_y;
	bool support_3d;

	u8 alpha_en;
	u32 alpha_mode;
	u32 g_alpha_val;
	u32 color_key_val;
	u8 csc_mode;

	struct rockchip_vop_win_area area[SFA_WIN_MAX_AREA];
	struct rockchip_vop_post_cfg post_cfg;
	u32 reserved;
};

struct rockchip_fb_reg_data {
	int win_num;
	int buf_num;
	struct rockchip_vop_win vop_win[SFA_MAX_WIN_SUPPORT];
	struct list_head list;
};

struct rockchip_fb_trsm_ops {
	int (*enable)(void);
	int (*disable)(void);
	int (*detect_panel)(void);
};

struct rockchip_vop_driver;

struct rockchip_vop_drv_ops {
	int (*open)(struct rockchip_vop_driver *dev_drv,
			int layer_id, bool open);
	int (*win_direct_en)(struct rockchip_vop_driver *dev_drv,
			     int win_id, int en);
	int (*init_lcdc)(struct rockchip_vop_driver *dev_drv);
	int (*ioctl)(struct rockchip_vop_driver *dev_drv, unsigned int cmd,
		     unsigned long arg, int layer_id);
	int (*suspend)(struct rockchip_vop_driver *dev_drv);
	int (*resume)(struct rockchip_vop_driver *dev_drv);
	int (*blank)(struct rockchip_vop_driver *dev_drv, int layer_id,
		     int blank_mode);
	int (*set_par)(struct rockchip_vop_driver *dev_drv, int layer_id);
	int (*pan_display)(struct rockchip_vop_driver *dev_drv, int layer_id);
	int (*direct_set_addr)(struct rockchip_vop_driver *dev_drv,
			       int win_id, u32 addr);
	int (*lcdc_reg_update)(struct rockchip_vop_driver *dev_drv);
	ssize_t (*get_disp_info)(struct rockchip_vop_driver *dev_drv,
				 char *buf, int layer_id);
	int (*load_screen)(struct rockchip_vop_driver *dev_drv,
			bool initscreen);
	int (*get_win_state)(struct rockchip_vop_driver *dev_drv, int layer_id);
	/* overlay manager : fb-win map */
	int (*ovl_mgr)(struct rockchip_vop_driver *dev_drv, int swap, bool set);
	int (*fps_mgr)(struct rockchip_vop_driver *dev_drv, int fps, bool set);
	/* get win id for fb */
	int (*fb_get_win_id)(struct rockchip_vop_driver *dev_drv,
			const char *id);
	int (*fb_win_remap)(struct rockchip_vop_driver *dev_drv,
			    u16 fb_win_map_order);
	int (*set_dsp_lut)(struct rockchip_vop_driver *dev_drv, u32 *lut,
				int mode);
	int (*set_hwc_lut)(struct rockchip_vop_driver *dev_drv, u32 *hwc_lut,
			   int mode);
	int (*read_dsp_lut)(struct rockchip_vop_driver *dev_drv, int *lut);
	int (*set_irq_to_cpu)(struct rockchip_vop_driver *dev_drv, int enable);
	int (*poll_vblank)(struct rockchip_vop_driver *dev_drv);
	int (*get_dsp_addr)(struct rockchip_vop_driver *dev_drv,
			    unsigned int *dsp_addr);

	int (*set_dsp_bcsh_hue)(struct rockchip_vop_driver *dev_drv,
				int sin_hue, int cos_hue);
	int (*set_dsp_bcsh_bcs)(struct rockchip_vop_driver *dev_drv,
				enum bcsh_bcs_mode mode, int value);
	int (*get_dsp_bcsh_hue)(struct rockchip_vop_driver *dev_drv,
				enum bcsh_hue_mode mode);
	int (*get_dsp_bcsh_bcs)(struct rockchip_vop_driver *dev_drv,
				enum bcsh_bcs_mode mode);
	int (*open_bcsh)(struct rockchip_vop_driver *dev_drv, bool open);

	int (*dump_reg)(struct rockchip_vop_driver *dev_drv);
	int (*cfg_done)(struct rockchip_vop_driver *dev_drv);
	int (*mmu_en)(struct rockchip_vop_driver *dev_drv, bool enable);
	int (*reg_writel)(struct rockchip_vop_driver *dev_drv, u32 offset,
			  u32 val);
	u32 (*reg_readl)(struct rockchip_vop_driver *dev_drv, u32 offset);
};

struct rockchip_vop_driver {
	char name[6];
	u8 id;
	int prop;
	bool suspend_flag;
	struct device *dev;

	struct rockchip_vop_win *win[SFA_MAX_WIN_SUPPORT];
	u8 num_win;		/* the number of win */
	u8 num_buf;		/* the numer of buffer */
	u8 atv_layer_cnt;	/* active win number */
	u8 fb_index_base;	/* the first fb index of the vop device */

	struct rockchip_screen *screen0;
	struct rockchip_screen screen1;
	struct rockchip_screen *cur_screen;
	u32 pixclock;

	u16 rotate_mode;
	u16 overlay_mode;
	u16 output_color;

	u16 fb_win_map;
	char fb0_win_id;
	char fb1_win_id;
	char fb2_win_id;
	char fb3_win_id;

	char mmu_dts_name[40];
	struct device *mmu_dev;
	int iommu_enabled;

	struct mutex fb_win_id_mutex;	/* mutex when set fb win map*/
	struct mutex win_cfg_lock;	/* mutex when config win */
	struct mutex cfg_lock;
	struct mutex regs_lock;		/* mutex when update regs */
	struct completion frame_done;	/* sync for pan_display,when we set a
					 * new frame address to vop register,
					 * we must make sure the frame begain
					 * to display
					 */
	spinlock_t cpl_lock;	/* lock for completion frame done */
	int first_frame;
	struct rockchip_fb_vsync vsync_info;

	bool wait_fs;		/* wait for new frame start in kernel */
	struct rockchip_fb_win_cfg_data win_data;
	struct sw_sync_timeline *timeline;
	int timeline_max;

	struct list_head update_regs_list;
	struct mutex update_regs_list_lock;	/* muter for update regs */
	struct kthread_worker update_regs_worker;
	struct task_struct *update_regs_thread;
	struct kthread_work update_regs_work;
	wait_queue_head_t update_regs_wait;
	/*
	 * last_regs means this config is scanning on the devices.
	 */
	struct rockchip_fb_reg_data *last_regs;

	struct rockchip_vop_drv_ops *ops;
	struct rockchip_fb_trsm_ops *trsm_ops;
	struct rockchip_vop_bcsh bcsh;
	u32 *hwc_lut;
	bool bcsh_init_status;
};

struct rockchip_fb_par {
	int id;
	u32 state;

	/* Start of fb address (physical address) */
	unsigned long fb_phy_base;
	/* Start of fb address (virt address) */
	char __iomem *fb_virt_base;
	/* fb mem size */
	u32 fb_size;
	struct rockchip_vop_driver *vop_drv;

#if defined(CONFIG_ION_XGOLD) || defined(CONFIG_ION_ROCKCHIP)
	struct ion_handle *ion_hdl;
	struct dma_buf *dma_buf;
#endif

	u32 reserved[2];
};

/*
 * rockchip fb info
 * @disp_mode: dual display mode
 *	NO_DUAL,no dual display,
 *	ONE_DUAL,use one vop for dual display
 *	DUAL,use 2 vop for dual display
 * @num_fb:	the total number of fb
 * @num_vop:	the total number of vop
 */

struct rockchip_fb {
	u32 disp_mode;
	u32 num_fb;
	u32 num_vop;
	struct fb_info *fb[SFA_MAX_FB_SUPPORT * 2];
	struct rockchip_vop_driver *vop_dev_drv[SFA_MAX_VOP_SUPPORT];

#if defined(CONFIG_ION_XGOLD) || defined(CONFIG_ION_ROCKCHIP)
	int ion_server_type;
	struct ion_client *ion_client;
#endif

};

int rockchip_fb_register(struct rockchip_vop_driver *dev_drv,
		     struct rockchip_vop_win *win, int id);
int rockchip_fb_unregister(struct rockchip_vop_driver *dev_drv);
int rockchip_fb_create_sysfs(struct fb_info *fbi);
int rockchip_fb_trsm_ops_register(struct rockchip_fb_trsm_ops *ops, u16 type);
struct rockchip_fb_trsm_ops *rockchip_fb_trsm_ops_get(u16 type);

int rockchip_fb_poll_prmry_screen_vblank(void);
u16 rockchip_fb_get_prmry_screen_ft(void);
bool rockchip_fb_poll_wait_frame_complete(void);
u32 rockchip_fb_get_prmry_screen_pixclock(void);

int rockchip_fb_switch_screen(struct rockchip_screen *screen,
			  bool enable, int vop_id);
int rockchip_fb_disp_scale(u8 scale_x, u8 scale_y, u8 vop_id);
int rockchip_fb_calc_fps(struct rockchip_screen *screen, u32 pixclock);

char *get_format_string(enum data_format, char *fmt);
struct rockchip_vop_driver *get_vop_drv(char *name);
int support_loader_display(void);

extern struct ion_client *xgold_ion_client_create(const char *name);
extern struct ion_client *rockchip_ion_client_create(const char *name);

#endif
