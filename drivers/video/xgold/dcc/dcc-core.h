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
#ifndef __GRACORE_H__
#define __GRACORE_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/completion.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/connector.h>
#include <linux/videodev2.h>
#include <linux/cdev.h>
#include <linux/fb.h>
#include <android/sync.h>
#include <android/sw_sync.h>
#include <video/xgold-dcc.h>

#define DCC_MODULE_NAME	"dcc"

#define DCC_DEBUG_DISPLAY

#define GRACRHAL_HWFIFO_SIZE 0xF
#define HWFIFO_FREE_WORDS(_lvl_) (GRACRHAL_HWFIFO_SIZE - _lvl_)


#include "dcc-sysfs.h"
#include "dcc-display.h"


/**
 * Debug
 */
#define DCC_DEFAULT_DEBUG_LEVEL	1

#define dcc_info(fmt, arg...) \
{ \
	if (gradata->debug.level >= 1) \
		pr_info(DCC_MODULE_NAME"[I]: "fmt, ##arg); \
}

#define dcc_err(fmt, arg...) \
{ \
	if (gradata->debug.level >= 1) \
		pr_err(DCC_MODULE_NAME"[E]: "fmt, ##arg); \
}

#define dcc_boot_info(fmt, arg...) \
{ \
	if (gradata->debug.boot && (gradata->debug.level >= 1)) \
		pr_info(DCC_MODULE_NAME"[ ]: "fmt, ##arg); \
}
#define dcc_boot_dbg(fmt, arg...) \
{ \
	if (gradata->debug.boot && (gradata->debug.level >= 1)) \
		pr_debug(DCC_MODULE_NAME"[ ]: "fmt, ##arg); \
}

#define DCC_DBGT(fmt, arg...) \
		pr_info(DCC_MODULE_NAME"[T]: "fmt, ##arg)

#define dcc_warn(fmt, arg...) \
{ \
	if (gradata->debug.level >= 1) \
		pr_warn(DCC_MODULE_NAME"[W]: " fmt, ##arg); \
}

#define DCC_DBG1(fmt, arg...) \
{ \
	if (gradata->debug.level >= 1) \
		pr_info(DCC_MODULE_NAME"[ ]: "  fmt, ##arg); \
}

#define DCC_DBG2(fmt, arg...) \
{ \
	if (gradata->debug.level >= 2) \
		pr_info(DCC_MODULE_NAME"[ ]: "  fmt, ##arg); \
}

#define DCC_DBG3(fmt, arg...) \
{ \
	if (gradata->debug.level >= 3) \
		pr_info(DCC_MODULE_NAME"[ ]: "  fmt, ##arg); \
}

#define DCC_DBG4(fmt, arg...) \
{ \
	if (gradata->debug.level >= 4) \
		pr_info(DCC_MODULE_NAME"[ ]: "  fmt, ##arg); \
}


#define IS_DCC_FMT_YUV(_f_) (\
		(_f_ == DCC_FMT_YUV422PACKED) ||\
		(_f_ == DCC_FMT_YUV422PACKED) ||\
		(_f_ == DCC_FMT_YUV420PLANAR) ||\
		(_f_ == DCC_FMT_YVU420PLANAR) ||\
		(_f_ == DCC_FMT_YUV422PLANAR) ||\
		(_f_ == DCC_FMT_YUV444PACKED) ||\
		(_f_ == DCC_FMT_YUV444PLANAR) ||\
		(_f_ == DCC_FMT_YUV444SP) ||\
		(_f_ == DCC_FMT_YUV422SP) ||\
		(_f_ == DCC_FMT_YUV420SP))

struct dcc_debug_t {
	int showfps;
	long long update_delay_max;	/* max update delay in us */
	struct timeval update_delay;	/* update delay measure */
	long long cmd_delay_max;	/* max cmd delay in us */
	long long cmd_delay_last;	/* last cmd delay in us */
	struct timeval cmd_delay;	/* cmd delay measure */
	int level;
	int boot;
	int dbgdts;		/* enable dts parsing debug */
	int fifowait;		/* max successive number of times
				 * we had to wait for the fifo */
	int fifomaxlevel;	/* max filling level of the fifo */
	unsigned int frame_update_number;
};

struct dcc_test_t {
	int bootscreen;
	int bootscreen_msdelay;
	int mipidsi_vsync;
};


#define NB_MEAS 20
struct dcc_meas_t {
	unsigned max;
	unsigned idx;
	struct timeval t[NB_MEAS];
	unsigned diffms[NB_MEAS];
};

struct dcc_supply {
	struct list_head list;
	const char *name;
	struct regulator *regulator;
	unsigned voltage;
};

struct dcc_update {
	struct list_head list;
	const char *name;
	struct dcc_update_layers updt;
	int timeline_expiracy;
};

struct dcc_resource {
	dma_addr_t pbase;
	unsigned size;
	void *vbase;
};

struct dcc_irq {
	int rx;
	int tx;
	int err;
	int cmd;
	int frame;
	int vsync;
};

struct dcc_fb {
	int rotation;
	struct fb_info *info;
	u32 cmap[16];
};

struct dcc_sync_obj_s {
	struct completion eoc;
	int eoc_to;
	struct completion dsifin;
	int dsifin_to;
	struct completion dsitr1;
	int dsitr1_to;
	struct completion eof;
	int eof_to;
};

enum {
	DRV_DCC_SUSPENDED = 0,
	DRV_DCC_ENABLED,
};

struct dcc_drvdata {
	int drv_state;
	struct dcc_display display;
	struct device *dev;
	struct miscdevice devfile;
	struct reset_control *reset;
	struct dcc_supply *supply;
	struct clk *clk_kernel;
	struct clk *clk_ahb;
	struct clk *clk_master;
	struct clk *clk_pll;
	unsigned int id;	/* DCC module id number */
	struct dcc_resource reg;	/* register memory desc */
	struct dcc_resource mem;	/* dedicated memory desc */
	struct dcc_irq irq;
	int bytesppix;
	int fbfmt;
	int overlay_status;
	struct dcc_debug_t debug; /* debug configuration */
	struct dcc_meas_t meas;
	struct semaphore sem;
	unsigned int overlay_nbr;
	unsigned int overlay_updt_cnt;
	unsigned int clk_rate;
	int gpio_lcd_bias;
	int gpio_lcd_bias_msdelay;
	int gpio_reset;
	int gpio_cd;
	int gpio_reset_delay;
	struct dcc_fb *fb;
	struct dcc_sync_obj_s sync;
	int use_fences;
	int use_fbapi;
	int fbapi_nr_buffers;
	int display_preinit;
	int dcc_sprite2_unified;
	int display_autorefresh;
	int display_invert_composite;
	struct dcc_test_t test;	/* tests configuration */
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
	struct regulator *pm_lcd;
#endif
	long long unsigned vsync_ts;  /* vsync timestamp */
	long long vsync_us; /* vsync period */
	struct timeval vsync_begin;
	struct work_struct vsync_work;  /* vsync work_struct */
	struct workqueue_struct *vsync_wq; /* vsync workqueue */
	struct work_struct eof_work; /* end of frame work_struct */
	struct workqueue_struct *eof_wq;   /* end of frame workqueue */
	struct workqueue_struct *acq_wq;   /* frame acquire workqueue */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;

	int fence_fd;
	struct sw_sync_timeline *timeline;
	struct sw_sync_timeline *updt_done_tl;
	int timeline_current;
	int update_pt_last; /* update point that was on display */
	int update_pt_curr; /* update point currently on display */
	struct semaphore update_sem;
	struct kobject *kobj_mipidsi_phy;
	int(*drv_suspend)(struct device *dev);
	int(*drv_resume)(struct device *dev);
};

#define m_to_dccdata(_p_, _m_) \
		container_of(_p_, struct dcc_drvdata, _m_)

struct x_area_t {
	unsigned int w;
	unsigned int h;
	unsigned int fmt;
};
#define AREA_INIT(_area_, _w_, _h_, _fmt_) \
	_area_.w = _w_; \
	_area_.h = _h_; \
	_area_.fmt = _fmt_;

struct x_rect_t {
	unsigned int x;
	unsigned int y;
	unsigned int w;
	unsigned int h;
	unsigned int fmt;
	unsigned int flags;
};

#define RECT_INIT(_rect_, _x_, _y_, _w_, _h_, _fmt_, _flg_) \
	_rect_.x = _x_; \
	_rect_.y = _y_; \
	_rect_.w = _w_; \
	_rect_.h = _h_; \
	_rect_.fmt = _fmt_; \
	_rect_.flags = _flg_;

#define X2(_rect_)	(_rect_->x+_rect_->w-1)
#define Y2(_rect_)	(_rect_->y+_rect_->h-1)

struct dcc_sprite_t {
	unsigned int phys;
	unsigned int id;	/* sprite number [0,3] */
	unsigned int x;		/* top left x */
	unsigned int y;		/* top left y */
	unsigned int w;
	unsigned int h;
	unsigned int alpha;	/* global alpha value */
	unsigned int global;	/* use global alpha value
				   from DIF_SPRITE_SIZEx.ALPHA or update cmd */
	unsigned int fmt;
	unsigned int chromakey;	/* chromakey for overlay only */
};

struct dcc_acq_fence_work {
	struct work_struct work;
	struct dcc_update_layers update;
	struct dcc_drvdata *drv;
	int update_pt;
#if defined(CONFIG_SYNC)
	struct sync_fence *acquire_fence[DCC_OVERLAY_NUM + 2];
#endif
};

#define DCC_SPRITE_INIT(_sp_, _e_, _i_, _o_, \
		_x_, _y_, _w_, _h_, _a_, _g_, _f_, _c_) \
	_sp_.id     = _i_; \
	_sp_.phys   = _o_; \
	_sp_.x      = _x_; \
	_sp_.y      = _y_; \
	_sp_.w      = _w_; \
	_sp_.h      = _h_; \
	_sp_.alpha  = _a_; \
	_sp_.global = _g_; \
	_sp_.fmt    = _f_; \
	_sp_.chromakey = _c_;

/*exported function prototypes */

int dcc_of_parse(struct platform_device *pdev);

int dcc_core_probe(struct platform_device *pdev);
int dcc_core_remove(struct platform_device *pdev);

#ifdef CONFIG_PM
int dcc_core_suspend(struct platform_device *pdev);
int dcc_core_resume(struct platform_device *pdev);
#endif /* CONFIG_PM */

int dcc_hal_probe(struct dcc_drvdata *pdata);
int dcc_hal_remove(struct dcc_drvdata *pdata);
int dcc_interrupt_setup(struct dcc_drvdata *pdata);

int dcc_display_setup(struct dcc_drvdata *pdata);
int dcc_display_suspend(struct dcc_drvdata *pdata);
int dcc_display_resume(struct dcc_drvdata *pdata);

long long dcc_timeval_diff(struct timeval *difference,
			   struct timeval *start_time,
			   struct timeval *end_time);

int dcc_update_queue(struct dcc_drvdata *pdata, struct dcc_update *head,
			struct dcc_update_layers *updt, unsigned int tline);
int dcc_update_items(struct dcc_update *update);

/* MIPI-DBI */
int dcc_dbi_sync(struct dcc_display *lcd);
int dcc_dbi_set_bitmux(struct dcc_drvdata *gradrv);

/* Display interfaces */
int dcc_dsi_probe(struct dcc_display *lcd);
int dcc_dbi_probe(struct dcc_display *lcd);


extern struct dcc_drvdata *gradata;

static inline int dcc_get_fb_fmt(struct dcc_drvdata *data)
{
	return data->fbfmt;
}

static inline int dcc_get_fb_bpp(struct dcc_drvdata *data)
{
	return data->bytesppix;
}

static inline int dcc_get_display_h(struct dcc_drvdata *data)
{
	return data->display.yres;
}

static inline int dcc_get_display_w(struct dcc_drvdata *data)
{
	return data->display.xres;
}

static inline void measdelay_start(struct timeval *begin)
{
	do_gettimeofday(begin);
}

static inline long long measdelay_stop(const char *str, struct timeval *begin)
{
	struct timeval end;
	long long diffus = 0;

	do_gettimeofday(&end);
	diffus = dcc_timeval_diff(NULL, begin, &end);
	if (str)
		DCC_DBG2("%s (%lli usec)\n", str, diffus);

	return diffus;
}

static inline int dcc_down_timeout_ms(struct semaphore *sem, int to)
{
	long jiffies = msecs_to_jiffies(to);
	return down_timeout(sem, jiffies);
}

static inline int dcc_completion_timeout_ms(struct completion *comp, int to)
{
	long jiffies = msecs_to_jiffies(to);
	return wait_for_completion_timeout(comp, jiffies);
}

#endif /* __GRACORE_H__ */
