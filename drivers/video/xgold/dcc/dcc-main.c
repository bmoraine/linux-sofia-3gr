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

#include <linux/module.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

#include <linux/wait.h>
#include <linux/file.h>
#include <linux/kernel.h>

#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/platform_device_pm.h>
#endif

#if defined(CONFIG_PM_SLEEP) || defined(CONFIG_PM_RUNTIME)
#include <linux/pm_runtime.h>
#endif

#include "dcc-core.h"
#include "dcc-gra.h"

#ifndef CONFIG_X86
	#define ioremap_wc ioremap
#endif


struct dcc_drvdata *gradata;

/* PM states index */
#define DCC_PM_STATE_D3		0
#define DCC_PM_STATE_D0i3	1
#define DCC_PM_STATE_D0i2	2
#define DCC_PM_STATE_D0		3
/*#define DCC_PM_STATE_D0		4 */
#define DCC_PM_STATE_XX		(DCC_PM_STATE_D0+1)

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
static int dcc_set_pm_state(struct device *,
		struct device_state_pm_state *);
static struct device_state_pm_state *dcc_get_initial_state(
		struct device *);

static struct device_state_pm_ops dcc_pm_ops = {
	.set_state = dcc_set_pm_state,
	.get_initial_state = dcc_get_initial_state,
};

/* PM states & class */
static struct device_state_pm_state dcc_pm_states[] = {
	{ .name = "disable", }, /* D3 */
	{ .name = "low_perf", }, /* D0i3 */
	{ .name = "mid_perf", }, /* D0i2 */
	{ .name = "high_perf", }, /* D0 */
/*	{ .name = "ultra_high_perf", },*/ /* D0 */
};

DECLARE_DEVICE_STATE_PM_CLASS(dcc);
#endif

#if !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
static int dcc_core_power_set(struct dcc_drvdata *pdata, int en)
{
	int ret = 0;
	struct dcc_supply *supply;

	if (en) {
		if (pdata->supply) {
			list_for_each_entry(
					supply, &pdata->supply->list, list) {
				ret = regulator_enable(supply->regulator);
				if (ret) {
					dcc_err("can't enable regulator %s\n",
							supply->name);
					return ret;
				}
				if (supply->voltage) {
					ret = regulator_set_voltage(
							supply->regulator,
							supply->voltage,
							supply->voltage);
					if (ret) {
						dcc_err("can't set voltage %s %d\n",
							supply->name,
							supply->voltage);
						return ret;
					}
				}
				DCC_DBG2("Regulator %s ON (%d uV)\n",
						supply->name,
						supply->voltage);
			}
		}
	} else {
		list_for_each_entry(supply, &pdata->supply->list, list) {
			ret = regulator_disable(supply->regulator);
			if (ret) {
				dcc_err("can't disable regulator %s\n",
						supply->name);
				return ret;
			}
			DCC_DBG2("Regulator %s OFF\n", supply->name);
		}
	}

	return 0;
}


static int dcc_core_clock_enable(struct clk *clock)
{
	int ret = 0;

	ret = clk_prepare(clock);
	if (ret) {
		dcc_err("Error preparing clk\n");
		return -1;
	}

	ret = clk_enable(clock);
	if (ret)
		dcc_err("Error enabling clk\n");

	return ret;
}

static int dcc_core_clock_set(struct dcc_drvdata *pdata, int en)
{
	int ret = 0;

	if (en) {
		if (!IS_ERR_OR_NULL(pdata->clk_pll)) {
			ret = dcc_core_clock_enable(pdata->clk_pll);
			if (ret) {
				dcc_err("Error enabling master clk\n");
				return -1;
			}
		}
		if (!IS_ERR_OR_NULL(pdata->clk_kernel)) {
			ret = dcc_core_clock_enable(pdata->clk_kernel);
			if (ret) {
				dcc_err("Error enabling kernel clk\n");
				return -1;
			}
		}
		if (!IS_ERR_OR_NULL(pdata->clk_ahb)) {
			ret = dcc_core_clock_enable(pdata->clk_ahb);
			if (ret) {
				dcc_err("Error enabling ahb clk\n");
				return -1;
			}
		}

		if (!IS_ERR_OR_NULL(pdata->clk_master)) {
			ret = dcc_core_clock_enable(pdata->clk_master);
			if (ret) {
				dcc_err("Error enabling master clk\n");
				return -1;
			}
		}
	} else {
		if (!IS_ERR_OR_NULL(pdata->clk_pll))
			clk_disable(pdata->clk_pll);

		if (!IS_ERR_OR_NULL(pdata->clk_kernel))
			clk_disable(pdata->clk_kernel);

		if (!IS_ERR_OR_NULL(pdata->clk_ahb))
			clk_disable(pdata->clk_ahb);

		if (!IS_ERR_OR_NULL(pdata->clk_master))
			clk_disable(pdata->clk_master);
	}
	return ret;
}


static int dcc_set_pm_state_by_num(struct dcc_drvdata *pdata,
		int state_num)
{
	int ret;

	switch (state_num) {
	case DCC_PM_STATE_D0:
		/* Set Power ON */
		DCC_DBG2("set power ON\n");
		ret = dcc_core_power_set(pdata, 1);
		if (ret < 0) {
			dcc_err("Unable to set power ON\n");
			return ret;
		}

		/* Set Clocks ON */
		DCC_DBG2("set clocks ON\n");
		ret = dcc_core_clock_set(pdata, 1);
		if (ret < 0) {
			dcc_err("Unable to set clock ON\n");
			return ret;
		}
		break;

	case DCC_PM_STATE_D3:
		DCC_DBG2("set power OFF\n");
		ret = dcc_core_power_set(pdata, 0);
		if (ret < 0) {
			dcc_err("Unable to set power\n");
			return ret;
		}

		DCC_DBG2("set clocks OFF\n");
		ret = dcc_core_clock_set(pdata, 0);
		if (ret < 0) {
			dcc_err("Unable to set clock\n");
			return ret;
		}
		break;

	default:
		dcc_err("Unknown power state\n");
	}
	return 0;
}



static int dcc_get_pm_state_id(const char *name)
{
	int id;

	for (id = DCC_PM_STATE_D3; id < DCC_PM_STATE_XX; id++) {
		if (!strcmp(name, dcc_pm_states[id].name))
			return id;
	}
	return DCC_PM_STATE_XX;
}

static int dcc_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	int state_num;
	int ret = 0;
	struct dcc_drvdata *pdata = dev_get_drvdata(dev);

	state_num = dcc_get_pm_state_id(state->name);
	DCC_DBG2("set pm state (%d) %s\n", state_num, state->name);
	ret = dcc_set_pm_state_by_num(pdata, state_num);
	if (ret < 0)
		dcc_err("Unable to set pm state (%d)%s\n",
				state_num, state->name);

	return ret;
}

static struct device_state_pm_state *dcc_get_initial_state(
		struct device *dev)
{
	return &dcc_pm_states[DCC_PM_STATE_D3];
}
#endif


/*
 * This is called whenever a process attempts to open the device file
 */
static int dcc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static inline u32 convert_bitfield(int val, struct fb_bitfield *bf)
{
	unsigned int mask = (1 << bf->length) - 1;

	return (val >> (16 - bf->length) & mask) << bf->offset;
}


/* set the software color map.*/
static int dcc_fb_setcolreg(unsigned int regno, unsigned int red,
			    unsigned int green, unsigned int blue,
			    unsigned int transp, struct fb_info *info)
{
	struct dcc_fb *fb = info->par;

	DCC_DBG3("%s\n", __func__);
	if (regno < 16) {
		fb->cmap[regno] =
			convert_bitfield(transp, &fb->info->var.transp) |
			convert_bitfield(blue, &fb->info->var.blue) |
			convert_bitfield(green, &fb->info->var.green) |
			convert_bitfield(red, &fb->info->var.red);
		return 0;
	} else {
		return 1;
	}
}

/* check var to see if supported by this device.*/
static int dcc_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	DCC_DBG3("%s\n", __func__);

	if ((var->xoffset != info->var.xoffset) ||
	    (var->bits_per_pixel != info->var.bits_per_pixel) ||
	    (var->grayscale != info->var.grayscale)) {
			dcc_err("%s l.%d\n", __func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}

/* Handles screen rotation if device supports it. */
static int dcc_fb_set_par(struct fb_info *info)
{
	struct dcc_fb *fb = info->par;
	DCC_DBG3("%s\n", __func__);
	if (fb->rotation != fb->info->var.rotate) {
		info->fix.line_length = info->var.xres * gradata->bytesppix;
		fb->rotation = fb->info->var.rotate;
		dcc_info("DCC_FB_ROTATE\n");
	}
	return 0;
}

/* Pan the display if device supports it. */
static int dcc_fb_pan_display(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	unsigned int offset;
	int ret = 0;
	struct dcc_fb *fb = info->par;

	if (down_interruptible(&gradata->sem))
		return -ERESTARTSYS;
	offset = fb->info->var.xres * gradata->bytesppix * var->yoffset;
	DCC_DBG2("DCC_FB_PAN x=%d, y=%d -> 0x%08x\n", var->xoffset,
		   var->yoffset, (unsigned int)gradata->mem.pbase + offset);

	{
		struct dcc_rect_t r;
		DCC_INIT_RECT(r,
			(unsigned int)gradata->mem.pbase + offset,
			dcc_get_display_w(gradata),/* framebuffer width */
			0, 0,/* x, y */
			dcc_get_display_w(gradata),/* width */
			dcc_get_display_h(gradata),/* height */
			dcc_get_fb_fmt(gradata),/* format */
			0, DCC_UPDATE_ONESHOT_SYNC);
		wmb();
		ret = dcc_rq_update(gradata, &r, 0);
	}
	up(&gradata->sem);
	return ret;
}

#include "dcc-hwregs.h"
static int dcc_fb_blank(int blank, struct fb_info *info)
{
	int ret = 0;
	unsigned int sprite_confx = 0;
	struct dcc_rect_t r;

	if (down_interruptible(&gradata->sem))
		return -ERESTARTSYS;
	DCC_DBG2("DCC_FB_BLANK blank=%d\n", blank);
	DCC_INIT_RECT(r,
			(unsigned int)gradata->mem.pbase,
			dcc_get_display_w(gradata),/* framebuffer width */
			0, 0,/* x, y */
			dcc_get_display_w(gradata),/* width */
			dcc_get_display_h(gradata),/* height */
			dcc_get_fb_fmt(gradata),/* format */
			0, DCC_UPDATE_ONESHOT_SYNC);

	gra_read_field(gradata, INR_DIF_CONF, &sprite_confx);
	sprite_confx &= ~BITFLDS(INR_DIF_CONF_BACK, 1);
	sprite_confx |= BITFLDS(INR_DIF_CONF_BACK, !!blank);
	gra_write_field(gradata, INR_DIF_CONF, sprite_confx);

	ret = dcc_rq_update(gradata, &r, 0);

	up(&gradata->sem);
	return ret;
}

static ssize_t
dcc_fb_read(struct fb_info *info, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	u32 *buffer, *dst;
	u32 __iomem *src;
	int c, i, cnt = 0, err = 0;
	unsigned long total_size;

	total_size = info->screen_size;

	if (total_size == 0)
		total_size = info->fix.smem_len;

	if (p >= total_size)
		return 0;

	if (count >= total_size)
		count = total_size;

	if (count + p > total_size)
		count = total_size - p;

	buffer = kmalloc((count > PAGE_SIZE) ? PAGE_SIZE : count,
			 GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	src = (u32 __iomem *) (gradata->mem.vbase + p);

	if (info->fbops->fb_sync)
		info->fbops->fb_sync(info);

	while (count) {
		c  = (count > PAGE_SIZE) ? PAGE_SIZE : count;
		dst = buffer;
		for (i = c >> 2; i--; )
			*dst++ = fb_readl(src++);
		if (c & 3) {
			u8 *dst8 = (u8 *) dst;
			u8 __iomem *src8 = (u8 __iomem *) src;

			for (i = c & 3; i--;)
				*dst8++ = fb_readb(src8++);

			src = (u32 __iomem *) src8;
		}

		if (copy_to_user(buf, buffer, c)) {
			err = -EFAULT;
			break;
		}
		*ppos += c;
		buf += c;
		cnt += c;
		count -= c;
	}

	kfree(buffer);

	return (err) ? err : cnt;
}


static struct fb_ops dcc_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = dcc_fb_check_var,
	.fb_set_par = dcc_fb_set_par,
	.fb_setcolreg = dcc_fb_setcolreg,
	.fb_pan_display = dcc_fb_pan_display,
	.fb_read = dcc_fb_read,
	.fb_blank = dcc_fb_blank,
};


static int dcc_fb_init(struct platform_device *pdev)
{
	int ret;
	struct fb_info *info;
	struct dcc_fb *fb;
	size_t framesize;
	unsigned int width, height;
	struct dcc_drvdata *pdata = dev_get_drvdata(&pdev->dev);

	if (!pdata)
		return -EINVAL;

	info = framebuffer_alloc(sizeof(struct dcc_fb), &pdev->dev);
	if (info == NULL) {
		ret = -ENOMEM;
		goto err_fb_alloc_failed;
	}
	fb = info->par;
	fb->info = info;
	width = pdata->display.xres;
	height = pdata->display.yres;

	fb->info->fbops = &dcc_fb_ops;

	/* These modes are the ones currently required by Android */

	fb->info->flags = FBINFO_FLAG_DEFAULT
		       |FBINFO_VIRTFB
		       |FBINFO_HWACCEL_YPAN
		       |FBINFO_HWACCEL_XPAN;
	fb->info->pseudo_palette = fb->cmap;
	fb->info->fix.type = FB_TYPE_PACKED_PIXELS;
	fb->info->fix.visual = FB_VISUAL_TRUECOLOR;
	fb->info->fix.line_length = width * pdata->bytesppix;
	fb->info->fix.accel = FB_ACCEL_NONE;
	fb->info->fix.ypanstep = 1;

	fb->info->var.xres = width;
	fb->info->var.yres = height;
	fb->info->var.xres_virtual = width;
	fb->info->var.yres_virtual = height * pdata->fbapi_nr_buffers;
	fb->info->var.bits_per_pixel = pdata->bytesppix*8;
	fb->info->var.activate = FB_ACTIVATE_NOW;
	fb->info->var.width = 47;
	fb->info->var.height = 68;

	if (pdata->fbfmt == DCC_FMT_RGB565) {
		fb->info->var.red.offset = 11;
		fb->info->var.red.length = 5;
		fb->info->var.green.offset = 5;
		fb->info->var.green.length = 6;
		fb->info->var.blue.offset = 0;
		fb->info->var.blue.length = 5;
	} else if (pdata->fbfmt == DCC_FMT_RGB888) {
		fb->info->var.red.offset = 16;
		fb->info->var.red.length = 8;
		fb->info->var.green.offset = 8;
		fb->info->var.green.length = 8;
		fb->info->var.blue.offset = 0;
		fb->info->var.blue.length = 8;
	}

	framesize =
	    width * height * pdata->bytesppix * pdata->fbapi_nr_buffers;

	fb->info->screen_base = pdata->mem.vbase;
	fb->info->screen_size = framesize;
	fb->info->fix.smem_start = pdata->mem.pbase;
	fb->info->fix.smem_len = framesize;

	ret = fb_set_var(fb->info, &fb->info->var);
	if (ret)
		goto err_fb_set_var_failed;

	/* register framebuffer device */
	ret = register_framebuffer(fb->info);

	if (ret)
		goto err_register_framebuffer_failed;

	dcc_info("FB API registered %d buffers map mem %zuMB [0x%lx -> %p]\n",
			pdata->fbapi_nr_buffers,
			framesize/1024/1024,
			(unsigned long)pdata->mem.pbase,
			fb->info->screen_base);
	pdata->fb = fb;
	return 0;

err_register_framebuffer_failed:
	dcc_err("Framebuffer API failed\n");
err_fb_set_var_failed:
	kfree(fb);
err_fb_alloc_failed:
	return ret;
}

#ifdef CONFIG_SW_SYNC_USER
static int dcc_fence_init(struct dcc_drvdata *pdata)
{
	/* Create update_timeline */
	if (!pdata->timeline) {
		pdata->timeline = sw_sync_timeline_create("dcc-update");
		if (!pdata->timeline) {
			dcc_err("%s: cannot create time line", __func__);
			return -ENOMEM;
		}
		pdata->timeline_current = 0;
	}
	if (!pdata->updt_done_tl) {
		pdata->updt_done_tl = sw_sync_timeline_create("dcc-upd-queue");
		if (!pdata->updt_done_tl) {
			dcc_err("%s: cannot create time line", __func__);
			return -ENOMEM;
		}
	}
	return 0;
}

static int dcc_fence_create(struct dcc_drvdata *pdata,
		unsigned int timeline_value, const char *name)
{
	struct sync_pt *point;
	struct sync_fence *fence = NULL;
	int fd = -1;

	if (!pdata->timeline)
		return fd;

	/* Create sync point */
	point = sw_sync_pt_create(pdata->timeline,
			timeline_value);
	if (point == NULL)
		return -EINVAL;

	/* Create fence */
	fence = sync_fence_create(name, point);
	if (fence == NULL) {
		sync_pt_free(point);
		return -EINVAL;
	}

	/* Create fd */
	fd = get_unused_fd();
	if (fd < 0) {
		dcc_err("fence_fd not initialized\n");
		sync_fence_put(fence);
		sync_pt_free(point);
		return -EINVAL;
	}

	/* Finally install the fence to that file descriptor */
	sync_fence_install(fence, fd);
	return fd;
}
#endif

/*
 * This is called whenever a process attempts to open the device file
 */
static int dcc_close(struct inode *inode, struct file *file)
{
	return 0;
}

/*
 * Ioctl function implementation.
 */
long dcc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;
	struct miscdevice *miscdev = file->private_data;
	struct dcc_drvdata *pdata =
		container_of(miscdev, struct dcc_drvdata, devfile);

	if (_IOC_TYPE(cmd) != DCC_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > DCC_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err =
		    !access_ok(VERIFY_WRITE, (void __user *)arg,
			       _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err =
		    !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (err)
		return -EFAULT;

	if (pdata->drv_state == DRV_DCC_SUSPENDED)
		return 0;

	if (down_interruptible(&pdata->sem))
		return -ERESTARTSYS;

#if defined(CONFIG_PM_SLEEP) || defined(CONFIG_PM_RUNTIME)
	pm_request_resume(pdata->dev);
#endif

	switch (cmd) {
	case DCC_IOW_POWER_ON:{
			int val;
			if (copy_from_user
			    (&val, (void __user *)arg, sizeof(val)))
				return -EFAULT;
			/*dcc_config_reset();*/
			err = 0;
		}
		break;

	case DCC_IOW_CONVERT:{
			struct dcc_rq_t rq;
			if (copy_from_user(&rq, (void __user *)arg, sizeof(rq)))
				return -EFAULT;
			err = dcc_rq_convert(pdata, &rq);
		}
		break;

	case DCC_IOW_UPDATE:{
			struct dcc_rect_t r;
			if (copy_from_user(&r, (void __user *)arg, sizeof(r)))
				return -EFAULT;

			err = dcc_rq_update(pdata, &r, 0);
		}
		break;

	case DCC_IOW_FILLRECTANGLE:{
			struct dcc_rect_t r;
			if (copy_from_user(&r, (void __user *)arg, sizeof(r)))
				return -EFAULT;
			err = dcc_rq_fillrectangle(pdata, &r);
		}
		break;

	case DCC_IOW_DRAWLINE:{
			struct dcc_rect_t r;
			if (copy_from_user(&r, (void __user *)arg, sizeof(r)))
				return -EFAULT;
			err = dcc_rq_drawline(pdata, &r);
		}
		break;

	case DCC_IOW_DRAWLINEREL:{
			struct dcc_rect_t r;
			if (copy_from_user(&r, (void __user *)arg, sizeof(r)))
				return -EFAULT;
			err = dcc_rq_drawlinerel(pdata, &r);
		}
		break;

	case DCC_IOW_SETPIXEL:{
			struct dcc_point_t p;
			if (copy_from_user(&p, (void __user *)arg, sizeof(p)))
				return -EFAULT;
			err = dcc_rq_setpixel(pdata, &p);
		}
		break;

	case DCC_IOW_BLIT:{
			struct dcc_rq_t rq;
			if (copy_from_user(&rq, (void __user *)arg, sizeof(rq)))
				return -EFAULT;
			err = dcc_rq_blit(pdata, &rq);
		}
		break;

	case DCC_IOW_SCROLLMOVE:{
			struct dcc_rq_t rq;
			if (copy_from_user(&rq, (void __user *)arg, sizeof(rq)))
				return -EFAULT;
			err = dcc_rq_scrollmove(pdata, &rq);
		}
		break;

	case DCC_IOW_RESIZE:{
			struct dcc_rq_resize_t rq;
			if (copy_from_user(&rq, (void __user *)arg, sizeof(rq)))
				return -EFAULT;
			err = dcc_rq_resize(pdata, &rq);
		}
		break;

	case DCC_IOW_ROTATE:{
			struct dcc_rq_t rq;
			if (copy_from_user(&rq, (void __user *)arg, sizeof(rq)))
				return -EFAULT;
			err = dcc_rq_rotate(pdata, &rq);
		}
		break;

	case DCC_IOR_DISPLAY_INFO:{
			struct dcc_display_cfg_t display_cfg;

			display_cfg.width = dcc_get_display_w(pdata);
			display_cfg.height = dcc_get_display_h(pdata);
			display_cfg.xdpi = pdata->display.xdpi;
			display_cfg.ydpi = pdata->display.ydpi;
			display_cfg.refresh_rate = pdata->display.fps;
			display_cfg.mem_base = pdata->mem.pbase;
			display_cfg.mem_size = pdata->mem.size;
			display_cfg.format = dcc_get_fb_fmt(pdata);
			display_cfg.overlay_nbr = pdata->overlay_nbr;
			display_cfg.hwid = pdata->id;
			display_cfg.drvid = DCCDRV_VERSION_INT;


			if (copy_to_user
			    ((void __user *)arg, &display_cfg,
			     sizeof(display_cfg)))
				return -EFAULT;
		}
		break;

	case DCC_IOW_COMPOSE: {
		struct dcc_update_layers updt;
		int ovl_id;

		if (copy_from_user(&updt, (void __user *)arg, sizeof(updt)))
			return -EFAULT;

		updt.fence_retire = -1;
		updt.back.fence_release = -1;
		for (ovl_id = 0; ovl_id < DCC_OVERLAY_NUM; ovl_id++)
			updt.ovls[ovl_id].fence_release = -1;

		if (err == 0) {
#ifdef CONFIG_SW_SYNC_USER
			if (pdata->use_fences) {
				updt.fence_retire =
					dcc_fence_create(pdata,
						pdata->timeline_current,
						"dcc-updt-retire-fence");
				if (updt.back.phys &&
					(!DCC_UPDATE_NOBG_GET(updt.flags)))
					updt.back.fence_release =
						dcc_fence_create(pdata,
						pdata->timeline_current,
						"dcc-updt-release-fence");
				for (ovl_id = 0;
					ovl_id < DCC_OVERLAY_NUM; ovl_id++) {
					struct dcc_layer_ovl *l =
						&updt.ovls[ovl_id];
					if (l->phys)
						l->fence_release =
							dcc_fence_create(pdata,
						pdata->timeline_current,
						"dcc-ovl-release-fence");
				}
			}
#endif
			err = dcc_rq_acquire_and_compose(pdata, &updt,
				pdata->timeline_current);
			if (err == 0)
				pdata->timeline_current++;
		} else
			err = 0;

		/* return to user space */
		if (copy_to_user((void __user *)arg, &updt, sizeof(updt)))
			return -EFAULT;
		}
		break;

	default:
		dcc_err("unknown ioctl cmd!\n");
		return -ENOTTY;
	}

	up(&pdata->sem);
	return err;
}


#include <linux/delay.h>
static unsigned int dcc_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	mask = POLLIN | POLLRDNORM; /* readable */
	return mask;
}

const struct file_operations dcc_ops = {
	.read = NULL,
	.write = NULL,
	.poll = dcc_poll,
#ifdef CONFIG_COMPAT
	.compat_ioctl = dcc_ioctl,
#endif
	.unlocked_ioctl = dcc_ioctl,
	.open = dcc_open,
	.release = dcc_close,
};


static int dcc_main_suspend(struct device *dev);
static int dcc_main_resume(struct device *dev);

int dcc_main_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	resource_size_t res_size;
	struct dcc_drvdata *pdata;

	/* Allocate driver data record */
	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct dcc_drvdata), GFP_KERNEL);
	if (pdata == NULL) {
		dcc_err("Couldn't allocate driver data record\n");
		ret = -ENOMEM;
		goto exit;
	}
	platform_set_drvdata(pdev, pdata);
	gradata = pdata;

	pdata->dev = &pdev->dev;

	/* enable boot debug messages */
	pdata->debug.boot = 1;
	pdata->drv_suspend = dcc_main_suspend;
	pdata->drv_resume = dcc_main_resume;

	/* Map registers */
	res = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "registers");
	if (!res) {
		dcc_err("Can't find ioresource\n");
		return -ENOENT;
	}

	res_size = resource_size(res);
	if (!devm_request_mem_region(&pdev->dev, res->start, res_size,
				res->name))
		return -EBUSY;

	pdata->reg.vbase = devm_ioremap(&pdev->dev, res->start, res_size);
	if (!pdata->reg.vbase)
		return -EBUSY;

	dcc_boot_info("DCC registers remapped to 0x%p\n",
			pdata->reg.vbase);

	ret = dcc_of_parse(pdev);
	if (ret) {
		dcc_info("Driver probe aborted due to DTS parsing\n");
		ret = 0; /* can happen depending on the build*/
		goto exit;
	}

	dcc_boot_info("Driver Version %s compiled at %s (%s)\n",
		 DCCDRV_VERSION_STR, __DATE__, __TIME__);

	DCC_DBG2("DCC Driver configuration:\n");
	if (pdata->display_autorefresh)
		dcc_boot_info("\t%s: display auto refresh\n",
			pdata->display_autorefresh ? "ON " : "OFF");
	if (pdata->display_preinit)
		dcc_boot_info("\t%s: display pre-init\n",
			pdata->display_preinit ? "ON " : "OFF");
	if (pdata->use_fences)
		dcc_boot_info("\t%s: use fences\n",
			pdata->use_fences ? "ON " : "OFF");

	/* Initialize ioctl semaphore */
	sema_init(&pdata->sem, 1);
	sema_init(&pdata->update_sem, 1);
#ifdef CONFIG_SW_SYNC_USER
	dcc_fence_init(pdata);
#endif

#ifdef CONFIG_PLATFORM_DEVICE_PM
	ret = platform_device_pm_set_class(pdev,
			pdata->pm_platdata->pm_user_name);
	if (ret) {
		dcc_err("Error while setting the pm class\n");
		return ret;
	}

	ret = platform_device_pm_set_state_by_name(pdev,
			pdata->pm_platdata->pm_state_D0_name);
	if (ret) {
		dcc_err("Error during state transition to D0\n");
		return ret;
	}

	if (pdata->pm_lcd) {
		regulator_enable(pdata->pm_lcd);
		regulator_set_voltage(pdata->pm_lcd, 2800000, 3000000);
	}
#endif

	ret = dcc_core_probe(pdev);
	if (ret) {
		dcc_err("Failed to initialize hardware(%d)", ret);
		goto exit;
	}
	/* display boot animation */
	dcc_clearscreen(pdata);

	if (pdata->test.bootscreen)
		dcc_bootscreen(pdata);

	pdata->devfile.minor = MISC_DYNAMIC_MINOR;
	pdata->devfile.name = DCC_DRIVER_NAME;
	pdata->devfile.fops = &dcc_ops;
	pdata->devfile.parent = NULL;
	ret = misc_register(&pdata->devfile);
	if (ret)
		dcc_err("failed to register misc device.\n");

	if (pdata->use_fbapi)
		dcc_fb_init(pdev);

	dcc_boot_info("Device driver loaded successfully [/dev/%s]\n",
		 DCC_DRIVER_NAME);

	/* disable boot debug messages */
	pdata->debug.boot = 0;
	pdata->drv_state = DRV_DCC_ENABLED;
	return 0;

exit:
	return ret;
}

static int dcc_main_remove(struct platform_device *pdev)
{
	struct dcc_drvdata *pdata;
	int ret = 0;

	if (!pdev)
		return -ENODEV;
	pdata = (struct dcc_drvdata *)platform_get_drvdata(pdev);
	if (!pdata)
		return 0;

	dcc_core_remove(pdev);
	misc_deregister(&pdata->devfile);

#ifdef CONFIG_SW_SYNC_USER
	/* Delete timeline */
	if (pdata->timeline != NULL)
		sync_timeline_destroy((struct sync_timeline *)
				pdata->timeline);
	if (pdata->updt_done_tl != NULL)
		sync_timeline_destroy((struct sync_timeline *)
				pdata->updt_done_tl);
#endif
	devm_kfree(&pdev->dev, pdata);

	dcc_info("Device driver removed\n");
	return ret;
}

#ifdef CONFIG_PM
static int dcc_main_suspend(struct device *dev)
{
	int ret = -1;
	struct dcc_drvdata *pdata;
	struct timeval begin;
	long long diffus = 0;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);

	pdata = (struct dcc_drvdata *)platform_get_drvdata(pdev);
	if (!pdata)
		return -1;

	if (pdata->drv_state == DRV_DCC_SUSPENDED) {
		DCC_DBG2("already suspended\n");
		return 0;
	}

	pdata->drv_state = DRV_DCC_SUSPENDED;
	if (down_interruptible(&pdata->sem))
		return -1;

	measdelay_start(&begin);

	ret = dcc_core_suspend(pdev);
	if (ret)
		dcc_err("Unable to suspend core\n");

#ifdef CONFIG_PLATFORM_DEVICE_PM
	ret = platform_device_pm_set_state_by_name(pdev,
			pdata->pm_platdata->pm_state_D3_name);
	if (ret)
		dcc_err("Error during state transition to D3\n");

	if (pdata->pm_lcd)
		regulator_disable(pdata->pm_lcd);
#endif

	reset_control_assert(pdata->reset);
	diffus = measdelay_stop(NULL, &begin);
	DCC_DBG2("suspended (%lli usec)\n", diffus);
	up(&pdata->sem);
	return ret;
}

static void dcc_main_shutdown(struct platform_device *pdev)
{
	dcc_main_suspend(&pdev->dev);
}

static int dcc_main_resume(struct device *dev)
{
	int ret = 0;
	struct dcc_drvdata *pdata;
	struct timeval begin;
	long long diffus = 0;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);

	pdata = (struct dcc_drvdata *)platform_get_drvdata(pdev);
	if (!pdata)
		return -EINVAL;

	if (pdata->drv_state == DRV_DCC_ENABLED) {
		DCC_DBG2("already enabled\n");
		return 0;
	}

	if (down_interruptible(&pdata->sem))
		return -EINVAL;

	measdelay_start(&begin);
	reset_control_deassert(pdata->reset);
#ifdef CONFIG_PLATFORM_DEVICE_PM
	regulator_enable(pdata->pm_lcd);
	ret = platform_device_pm_set_state_by_name(pdev,
			pdata->pm_platdata->pm_state_D0_name);
#endif
	if (ret)
		dcc_err("Error during state transition to D0\n");

	/* resume IP */
	ret = dcc_core_resume(pdev);
	if (ret)
		dcc_err("Unable to resume core\n");

	dcc_clearscreen(pdata);
	up(&pdata->sem);

	diffus = measdelay_stop(NULL, &begin);
	DCC_DBG2("resumed (%lli usec)\n", diffus);
	pdata->drv_state = DRV_DCC_ENABLED;
	return ret;
}
#endif

static struct of_device_id xgold_dcc_of_match[] = {
	{ .compatible = "intel,dcc", },
	{ },
};

static const struct dev_pm_ops dcc_driver_pm_ops = {
#if CONFIG_PM
	.suspend = dcc_main_suspend,
	.resume = dcc_main_resume,
#endif
};

static struct platform_driver dcc_driver = {
	.probe = dcc_main_probe,
	.remove = dcc_main_remove,
	.shutdown = dcc_main_shutdown,
	.driver = {
		   .name = DCC_MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &dcc_driver_pm_ops,
		   .of_match_table = xgold_dcc_of_match,
		   },
};

static int __init gra_init(void)
{
#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&dcc_pm_class);
	if (ret)
		return ret;
#endif
	return platform_driver_register(&dcc_driver);
}

static void __exit gra_exit(void)
{
	platform_driver_unregister(&dcc_driver);
}

module_init(gra_init);
module_exit(gra_exit);

MODULE_DESCRIPTION("Display Controller driver");
MODULE_VERSION(DCCDRV_VERSION_STR);
MODULE_ALIAS("dcc");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, xgold_dcc_of_match);
