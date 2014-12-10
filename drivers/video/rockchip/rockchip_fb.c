/*
 * rockchip fb frameware driver
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/div64.h>
#include <linux/uaccess.h>
#include <linux/linux_logo.h>
#include <linux/rockchip_fb.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_platform.h>
#endif

#if defined(CONFIG_ION_XGOLD)
#include <linux/dma-buf.h>
#include <linux/highmem.h>
#endif

static bool hdmi_switch_complete;
static int fence_wait_begin;
struct list_head saved_list;
static struct platform_device *fb_pdev;
static struct rockchip_fb_trsm_ops *trsm_lvds_ops;
static struct rockchip_fb_trsm_ops *trsm_edp_ops;
static struct rockchip_fb_trsm_ops *trsm_mipi_ops;
static int uboot_logo_on;

int support_uboot_display(void)
{
	return uboot_logo_on;
}

char *get_format_string(enum data_format format, char *fmt)
{
	if (!fmt)
		return NULL;
	switch (format) {
	case ARGB888:
		strcpy(fmt, "ARGB888");
		break;
	case RGB888:
		strcpy(fmt, "RGB888");
		break;
	case RGB565:
		strcpy(fmt, "RGB565");
		break;
	case YUV420:
		strcpy(fmt, "YUV420");
		break;
	case YUV422:
		strcpy(fmt, "YUV422");
		break;
	case YUV444:
		strcpy(fmt, "YUV444");
		break;
	case XRGB888:
		strcpy(fmt, "XRGB888");
		break;
	case XBGR888:
		strcpy(fmt, "XBGR888");
		break;
	case ABGR888:
		strcpy(fmt, "XBGR888");
		break;
	default:
		strcpy(fmt, "invalid");
		break;
	}

	return fmt;
}

struct rockchip_vop_driver *get_vop_drv(char *name)
{
	struct rockchip_fb *sfb_info = NULL;
	struct rockchip_vop_driver *dev_drv = NULL;
	int i = 0;

	if (likely(fb_pdev))
		sfb_info = platform_get_drvdata(fb_pdev);
	else
		return NULL;

	for (i = 0; i < sfb_info->num_vop; i++) {
		if (!strcmp(sfb_info->vop_dev_drv[i]->name, name)) {
			dev_drv = sfb_info->vop_dev_drv[i];
			break;
		}
	}

	return dev_drv;
}

static struct rockchip_vop_driver *get_prmry_vop_drv(void)
{
	struct rockchip_fb *sfb_info = NULL;
	struct rockchip_vop_driver *dev_drv = NULL;
	int i = 0;

	if (likely(fb_pdev))
		sfb_info = platform_get_drvdata(fb_pdev);
	else
		return NULL;

	for (i = 0; i < sfb_info->num_vop; i++) {
		if (sfb_info->vop_dev_drv[i]->prop == PRMRY) {
			dev_drv = sfb_info->vop_dev_drv[i];
			break;
		}
	}

	return dev_drv;
}

int rockchip_fb_trsm_ops_register(struct rockchip_fb_trsm_ops *ops, u16 type)
{
	switch (type) {
	case SCREEN_RGB:
	case SCREEN_LVDS:
	case SCREEN_DUAL_LVDS:
		trsm_lvds_ops = ops;
		break;
	case SCREEN_EDP:
		trsm_edp_ops = ops;
		break;
	case SCREEN_MIPI:
	case SCREEN_DUAL_MIPI:
		trsm_mipi_ops = ops;
		break;
	default:
		pr_err("%s:un supported transmitter:%d!\n",
		       __func__, type);
		break;
	}
	return 0;
}

struct rockchip_fb_trsm_ops *rockchip_fb_trsm_ops_get(u16 type)
{
	struct rockchip_fb_trsm_ops *ops;

	switch (type) {
	case SCREEN_RGB:
	case SCREEN_LVDS:
	case SCREEN_DUAL_LVDS:
		ops = trsm_lvds_ops;
		break;
	case SCREEN_EDP:
		ops = trsm_edp_ops;
		break;
	case SCREEN_MIPI:
	case SCREEN_DUAL_MIPI:
		ops = trsm_mipi_ops;
		break;
	default:
		ops = NULL;
		pr_err("%s:un supported transmitter:%d!\n",
		       __func__, type);
		break;
	}
	return ops;
}

int rockchip_fb_calc_fps(struct rockchip_screen *screen, u32 pixclock)
{
	u32 x, y;
	unsigned long long hz;

	if (!screen) {
		pr_err("%s:null screen!\n", __func__);
		return 0;
	}
	x = screen->mode.xres + screen->mode.left_margin +
	    screen->mode.right_margin + screen->mode.hsync_len;
	y = screen->mode.yres + screen->mode.upper_margin +
	    screen->mode.lower_margin + screen->mode.vsync_len;

	hz = 1000000000000ULL;	/* 1e12 picoseconds per second */

	hz += (x * y) / 2;
	do_div(hz, x * y);	/* divide by x * y with rounding */

	hz += pixclock / 2;
	do_div(hz, pixclock);	/* divide by pixclock with rounding */

	return hz;
}

/*
  * get one frame time of the prmry screen, unit: us
  */
u16 rockchip_fb_get_prmry_screen_ft(void)
{
	struct rockchip_vop_driver *dev_drv = get_prmry_vop_drv();
	uint32_t htotal, vtotal, pixclock_ps;
	u64 pix_total, ft_us;

	if (unlikely(!dev_drv))
		return 0;

	pixclock_ps = dev_drv->pixclock;

	vtotal = (dev_drv->cur_screen->mode.upper_margin +
		  dev_drv->cur_screen->mode.lower_margin +
		  dev_drv->cur_screen->mode.yres +
		  dev_drv->cur_screen->mode.vsync_len);
	htotal = (dev_drv->cur_screen->mode.left_margin +
		  dev_drv->cur_screen->mode.right_margin +
		  dev_drv->cur_screen->mode.xres +
		  dev_drv->cur_screen->mode.hsync_len);
	pix_total = htotal * vtotal;
	ft_us = pix_total * pixclock_ps;
	do_div(ft_us, 1000000);

	if (ft_us > 0)
		dev_drv->cur_screen->ft = (u16)ft_us;
	else
		dev_drv->cur_screen->ft = 16;

	return dev_drv->cur_screen->ft;
}

u32 rockchip_fb_get_prmry_screen_pixclock(void)
{
	struct rockchip_vop_driver *dev_drv = get_prmry_vop_drv();

	if (unlikely(!dev_drv))
		return 0;
	else
		return dev_drv->pixclock;
}

int rockchip_fb_poll_prmry_screen_vblank(void)
{
	struct rockchip_vop_driver *dev_drv = get_prmry_vop_drv();

	if (likely(dev_drv)) {
		if (dev_drv->ops->poll_vblank)
			return dev_drv->ops->poll_vblank(dev_drv);
		else
			return SFA_LF_STATUS_NC;
	} else {
		return SFA_LF_STATUS_NC;
	}
}

bool rockchip_fb_poll_wait_frame_complete(void)
{
	uint32_t timeout = SFA_LF_MAX_TIMEOUT;
	struct rockchip_vop_driver *dev_drv = get_prmry_vop_drv();

	if (likely(dev_drv)) {
		if (dev_drv->ops->set_irq_to_cpu)
			dev_drv->ops->set_irq_to_cpu(dev_drv, 0);
	}

	if (rockchip_fb_poll_prmry_screen_vblank() == SFA_LF_STATUS_NC) {
		if (dev_drv->ops->set_irq_to_cpu)
			dev_drv->ops->set_irq_to_cpu(dev_drv, 1);
		return false;
	}

	while (!(rockchip_fb_poll_prmry_screen_vblank() == SFA_LF_STATUS_FR) &&
	       --timeout)
		;
	while (!(rockchip_fb_poll_prmry_screen_vblank() == SFA_LF_STATUS_FC) &&
	       --timeout)
		;

	if (likely(dev_drv)) {
		if (dev_drv->ops->set_irq_to_cpu)
			dev_drv->ops->set_irq_to_cpu(dev_drv, 1);
	}

	return true;
}

void rockchip_fb_fence_wait(struct rockchip_vop_driver *dev_drv,
			struct sync_fence *fence)
{
	int err = sync_fence_wait(fence, 1000);

	if (err >= 0)
		return;

	if (err == -ETIME)
		err = sync_fence_wait(fence, 10 * MSEC_PER_SEC);
	if (err < 0)
		dev_err(dev_drv->dev, "error waiting on fence\n");
}

void rockchip_fb_free_dma_buf(struct rockchip_fb_dma_buf_data *dma_buf_data)
{
	if (dma_buf_data->acq_fence)
		sync_fence_put(dma_buf_data->acq_fence);

	memset(dma_buf_data, 0, sizeof(struct rockchip_fb_dma_buf_data));
}

static void rockchip_fb_update_reg(struct rockchip_vop_driver *dev_drv,
			       struct rockchip_reg_data *regs)
{
	int i, ret = 0;
	ktime_t timestamp = dev_drv->vsync_info.timestamp;

	if (dev_drv->ops->lcdc_reg_update)
		dev_drv->ops->lcdc_reg_update(dev_drv);

	if (dev_drv->wait_fs == 0) {
		ret = wait_event_interruptible_timeout(dev_drv->vsync_info.wait,
				!ktime_equal(timestamp,
					     dev_drv->vsync_info.timestamp),
				msecs_to_jiffies(dev_drv->cur_screen->ft + 5));
	}

	sw_sync_timeline_inc(dev_drv->timeline, 1);

	if (dev_drv->win_data.acq_fence_fd[0] >= 0) {
		for (i = 0; i < SFA_MAX_LAYER_SUPPORT; i++) {
			if (dev_drv->win_data.acq_fence_fd[i] > 0) {
				put_unused_fd(
					dev_drv->win_data.acq_fence_fd[i]);
				dev_err(dev_drv->dev, "acq_fd=%d\n",
					dev_drv->win_data.acq_fence_fd[i]);
			}
			rockchip_fb_free_dma_buf(&regs->dma_buf_data[i]);
		}
	}
}

static void rockchip_fb_update_regs_handler(struct kthread_work *work)
{
	struct rockchip_vop_driver *dev_drv =
	    container_of(work, struct rockchip_vop_driver, update_regs_work);
	struct rockchip_reg_data *data, *next;

	mutex_lock(&dev_drv->update_regs_list_lock);
	saved_list = dev_drv->update_regs_list;
	list_replace_init(&dev_drv->update_regs_list, &saved_list);
	mutex_unlock(&dev_drv->update_regs_list_lock);

	list_for_each_entry_safe(data, next, &saved_list, list) {
		rockchip_fb_update_reg(dev_drv, data);
		list_del(&data->list);
		kfree(data);
	}
}

static int rockchip_fb_get_list_stat(struct rockchip_vop_driver *dev_drv)
{
	int i, j;

	i = list_empty(&dev_drv->update_regs_list);
	j = list_empty(&saved_list);
	if ((i == 1) && (j == 1))
		return 1;
	else
		return 0;
}

static int rockchip_fb_open(struct fb_info *info, int user)
{
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)info->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	int win_id;

	win_id = dev_drv->ops->fb_get_win_id(dev_drv, info->fix.id);
	/* if this layer aready opened ,no need to reopen */
	if (!dev_drv->win[win_id]->state)
		dev_drv->ops->open(dev_drv, win_id, 1);

	return 0;
}

static int rockchip_fb_close(struct fb_info *info, int user)
{
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)info->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	struct rockchip_vop_win *win = NULL;
	int win_id = dev_drv->ops->fb_get_win_id(dev_drv, info->fix.id);

	if (win_id >= 0) {
		win = dev_drv->win[win_id];
		if (fb_par->fb_phy_base > 0)
			info->fix.smem_start = fb_par->fb_phy_base;

		info->var.xres = dev_drv->screen0->mode.xres;
		info->var.yres = dev_drv->screen0->mode.yres;
		info->var.grayscale |=
		    (info->var.xres << 8) + (info->var.yres << 20);
#ifdef CONFIG_LOGO_LINUX_BMP
		info->var.bits_per_pixel = 32;
#else
		info->var.bits_per_pixel = 16;
#endif
		info->fix.line_length =
		    (info->var.xres) * (info->var.bits_per_pixel >> 3);
		info->var.xres_virtual = info->var.xres;
		info->var.yres_virtual = info->var.yres;
		info->var.width = dev_drv->screen0->width;
		info->var.height = dev_drv->screen0->height;
		info->var.pixclock = dev_drv->pixclock;
		info->var.left_margin = dev_drv->screen0->mode.left_margin;
		info->var.right_margin = dev_drv->screen0->mode.right_margin;
		info->var.upper_margin = dev_drv->screen0->mode.upper_margin;
		info->var.lower_margin = dev_drv->screen0->mode.lower_margin;
		info->var.vsync_len = dev_drv->screen0->mode.vsync_len;
		info->var.hsync_len = dev_drv->screen0->mode.hsync_len;
	}

	return 0;
}

static ssize_t rockchip_fb_read(struct fb_info *info, char __user *buf,
			    size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	u8 *buffer, *dst;
	u8 __iomem *src;
	int c, cnt = 0, err = 0;
	unsigned long total_size;
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)info->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	struct rockchip_vop_win *win = NULL;
	int win_id = 0;

	win_id = dev_drv->ops->fb_get_win_id(dev_drv, info->fix.id);
	if (win_id < 0)
		return -ENODEV;

	win = dev_drv->win[win_id];

	/* only read the current frame buffer */
	if (win->format == RGB565)
		total_size = win->xact * win->yact << 1;
	else
		total_size = win->xact * win->yact << 2;

	if (p >= total_size)
		return 0;

	if (count >= total_size)
		count = total_size;

	if (count + p > total_size)
		count = total_size - p;

	buffer = kmalloc((count > PAGE_SIZE) ? PAGE_SIZE : count, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	src = (u8 __iomem *)(info->screen_base + p + win->y_offset);

	while (count) {
		c = (count > PAGE_SIZE) ? PAGE_SIZE : count;
		dst = buffer;
		fb_memcpy_fromfb(dst, src, c);
		dst += c;
		src += c;

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

static ssize_t rockchip_fb_write(struct fb_info *info, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	u8 *buffer, *src;
	u8 __iomem *dst;
	int c, cnt = 0, err = 0;
	unsigned long total_size;
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)info->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	struct rockchip_vop_win *win = NULL;
	int win_id = 0;

	win_id = dev_drv->ops->fb_get_win_id(dev_drv, info->fix.id);
	if (win_id < 0)
		return -ENODEV;

	win = dev_drv->win[win_id];

	/* write the current frame buffer */
	if (win->format == RGB565)
		total_size = win->xact * win->yact << 1;
	else
		total_size = win->xact * win->yact << 2;

	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}

	buffer = kmalloc((count > PAGE_SIZE) ? PAGE_SIZE : count, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	dst = (u8 __iomem *)(info->screen_base + p + win->y_offset);

	while (count) {
		c = (count > PAGE_SIZE) ? PAGE_SIZE : count;
		src = buffer;

		if (copy_from_user(src, buf, c)) {
			err = -EFAULT;
			break;
		}

		fb_memcpy_tofb(dst, src, c);
		dst += c;
		src += c;
		*ppos += c;
		buf += c;
		cnt += c;
		count -= c;
	}

	kfree(buffer);

	return (cnt) ? cnt : err;
}

static int rockchip_fb_blank(int blank_mode, struct fb_info *info)
{
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)info->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	struct fb_fix_screeninfo *fix = &info->fix;
	int win_id;

	win_id = dev_drv->ops->fb_get_win_id(dev_drv, fix->id);
	if (win_id < 0)
		return -ENODEV;

	dev_drv->ops->blank(dev_drv, win_id, blank_mode);
	return 0;
}

static int rockchip_fb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	if (0 == var->xres_virtual || 0 == var->yres_virtual ||
	    0 == var->xres || 0 == var->yres || var->xres < 16 ||
	    ((16 != var->bits_per_pixel) && (32 != var->bits_per_pixel))) {
		dev_info(info->dev, "%s check var fail 1!!!\n", info->fix.id);
		dev_info(info->dev, "xres_vir:%d>>yres_vir:%d\n",
			 var->xres_virtual, var->yres_virtual);
		dev_info(info->dev, "xres:%d>>yres:%d\n", var->xres, var->yres);
		dev_info(info->dev, "bits_per_pixel:%d\n", var->bits_per_pixel);
		return -EINVAL;
	}

	if (((var->xoffset + var->xres) > var->xres_virtual) ||
	    ((var->yoffset + var->yres) > (var->yres_virtual))) {
		dev_info(info->dev, "%s check_var fail 2!!!\n", info->fix.id);
		dev_info(info->dev, "xoffset:%d>>xres:%d>>xres_vir:%d\n",
			 var->xoffset, var->xres, var->xres_virtual);
		dev_info(info->dev, "yoffset:%d>>yres:%d>>yres_vir:%d\n",
			 var->yoffset, var->yres, var->yres_virtual);
		return -EINVAL;
	}

	return 0;
}

static int rockchip_fb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)info->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	struct rockchip_vop_win *win = NULL;
	struct rockchip_screen *screen = dev_drv->cur_screen;
	int win_id = 0;
	u32 cblen = 0, crlen = 0;
	u16 xsize = 0, ysize = 0;	/* winx display window height/width */
	u32 xoffset = var->xoffset;	/* offset from virtual to visible */
	u32 yoffset = var->yoffset;
	u16 xpos = (var->nonstd >> 8) & 0xfff;	/* visiable pos in panel */
	u16 ypos = (var->nonstd >> 20) & 0xfff;
	u32 xvir = var->xres_virtual;
	u32 yvir = var->yres_virtual;
	u8 data_format = var->nonstd & 0xff;
	u32 stride = 0;
	u32 uv_stride = 0;

	var->pixclock = dev_drv->pixclock;
	win_id = dev_drv->ops->fb_get_win_id(dev_drv, info->fix.id);
	if (win_id < 0)
		return -ENODEV;

	win = dev_drv->win[win_id];

	/*
	 * if the application has specific the horizontal and
	 * vertical display size
	 */
	if (var->grayscale >> 8) {
		xsize = (var->grayscale >> 8) & 0xfff;
		ysize = (var->grayscale >> 20) & 0xfff;
	} else {	/* ohterwise full screen display */
		xsize = screen->mode.xres;
		ysize = screen->mode.yres;
	}

	dev_drv->screen0->xsize = xsize;
	dev_drv->screen0->ysize = ysize;
	dev_drv->screen0->xpos = xpos;
	dev_drv->screen0->ypos = ypos;

	/* calculate y_offset,c_offset,line_length,cblen and crlen  */
	switch (data_format) {
	case HAL_PIXEL_FORMAT_RGBX_8888:
		win->format = XBGR888;
		fix->line_length = 4 * xvir;
		win->y_offset = (yoffset * xvir + xoffset) * 4;
		break;
	case HAL_PIXEL_FORMAT_RGBA_8888:
		win->format = ABGR888;
		fix->line_length = 4 * xvir;
		win->y_offset = (yoffset * xvir + xoffset) * 4;
		break;
	case HAL_PIXEL_FORMAT_BGRA_8888:
		win->format = ARGB888;
		fix->line_length = 4 * xvir;
		win->y_offset = (yoffset * xvir + xoffset) * 4;
		break;
	case HAL_PIXEL_FORMAT_RGB_888:
		win->format = RGB888;
		fix->line_length = 3 * xvir;
		win->y_offset = (yoffset * xvir + xoffset) * 3;
		break;
	case HAL_PIXEL_FORMAT_RGB_565:
		win->format = RGB565;
		fix->line_length = 2 * xvir;
		win->y_offset = (yoffset * xvir + xoffset) * 2;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_422_SP:
		win->format = YUV422;
		fix->line_length = xvir;
		cblen = (xvir * yvir) >> 1;
		crlen = cblen;
		win->y_offset = yoffset * xvir + xoffset;
		win->c_offset = win->y_offset;
		break;
	case HAL_PIXEL_FORMAT_YCrCb_NV12:
		win->format = YUV420;
		fix->line_length = xvir;
		cblen = (xvir * yvir) >> 2;
		crlen = cblen;
		win->y_offset = yoffset * xvir + xoffset;
		win->c_offset = (yoffset >> 1) * xvir + xoffset;
		break;
	case HAL_PIXEL_FORMAT_YCrCb_444:
		win->format = 5;
		fix->line_length = xvir << 2;
		win->y_offset = yoffset * xvir + xoffset;
		win->c_offset = yoffset * 2 * xvir + (xoffset << 1);
		cblen = (xvir * yvir);
		crlen = cblen;
		break;
	default:
		dev_err(dev_drv->dev, "%s:un supported format:0x%x\n",
			__func__, data_format);
		return -EINVAL;
	}

	stride = fix->line_length;
	win->y_vir_stride = stride >> 2;
	win->uv_vir_stride = uv_stride >> 2;
	win->xpos = xpos;
	win->ypos = ypos;
	win->xsize = xsize;
	win->ysize = ysize;

	win->smem_start = fix->smem_start;
	win->cbr_start = fix->mmio_start;
	win->xact = var->xres;	/* winx active window height and width */
	win->yact = var->yres;
	win->xvir = var->xres_virtual;	/* virtual resolution stride */
	win->yvir = var->yres_virtual;

	dev_drv->ops->set_par(dev_drv, win_id);

	return 0;
}

static int rockchip_fb_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)info->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	struct rockchip_vop_win *win = NULL;
	int win_id = 0;
	u32 xoffset = var->xoffset;
	u32 yoffset = var->yoffset;
	u32 xvir = var->xres_virtual;
	u8 data_format = var->nonstd & 0xff;

	win_id = dev_drv->ops->fb_get_win_id(dev_drv, info->fix.id);
	if (win_id < 0)
		return -ENODEV;

	win = dev_drv->win[win_id];

	switch (win->format) {
	case XBGR888:
	case ARGB888:
	case ABGR888:
		win->y_offset = (yoffset * xvir + xoffset) * 4;
		break;
	case RGB888:
		win->y_offset = (yoffset * xvir + xoffset) * 3;
		break;
	case RGB565:
		win->y_offset = (yoffset * xvir + xoffset) * 2;
		break;
	case YUV422:
		win->y_offset = yoffset * xvir + xoffset;
		win->c_offset = win->y_offset;
		break;
	case YUV420:
		win->y_offset = yoffset * xvir + xoffset;
		win->c_offset = (yoffset >> 1) * xvir + xoffset;
		break;
	case YUV444:
		win->y_offset = yoffset * xvir + xoffset;
		win->c_offset = yoffset * 2 * xvir + (xoffset << 1);
		break;
	default:
		dev_err(dev_drv->dev, "un supported format:0x%x\n",
			data_format);
		return -EINVAL;
	}

	dev_drv->ops->pan_display(dev_drv, win_id);

	return 0;
}

static int rockchip_fb_config_done(struct fb_info *info)
{
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)info->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	struct rockchip_reg_data *regs;
	struct sync_fence *release_fence;
	struct sync_fence *retire_fence;
	struct sync_pt *release_sync_pt;
	struct sync_pt *retire_sync_pt;
	struct sync_fence *layer2_fence;
	struct sync_pt *layer2_pt;

	if ((fence_wait_begin == 1) && (!dev_drv->suspend_flag)) {
		dev_drv->win_data.rel_fence_fd[0] = get_unused_fd();
		if (dev_drv->win_data.rel_fence_fd[0] < 0) {
			dev_info(info->dev, "rel_fence_fd=%d\n",
				 dev_drv->win_data.rel_fence_fd[0]);
			return -EFAULT;
		}

		dev_drv->win_data.rel_fence_fd[1] = get_unused_fd();
		if (dev_drv->win_data.rel_fence_fd[1] < 0) {
			dev_info(info->dev, "rel_fence_fd=%d\n",
				 dev_drv->win_data.rel_fence_fd[1]);
			return -EFAULT;
		}

		dev_drv->win_data.ret_fence_fd = get_unused_fd();
		if (dev_drv->win_data.ret_fence_fd < 0) {
			dev_info(info->dev, "ret_fence_fd=%d\n",
				 dev_drv->win_data.ret_fence_fd);
			return -EFAULT;
		}
		mutex_lock(&dev_drv->update_regs_list_lock);
		dev_drv->timeline_max++;
		release_sync_pt =
		    sw_sync_pt_create(dev_drv->timeline, dev_drv->timeline_max);
		release_fence = sync_fence_create("rel_fence", release_sync_pt);
		sync_fence_install(release_fence,
				   dev_drv->win_data.rel_fence_fd[0]);

		layer2_pt =
		    sw_sync_pt_create(dev_drv->timeline, dev_drv->timeline_max);
		layer2_fence = sync_fence_create("rel2_fence", layer2_pt);
		sync_fence_install(layer2_fence,
				   dev_drv->win_data.rel_fence_fd[1]);

		retire_sync_pt =
		    sw_sync_pt_create(dev_drv->timeline, dev_drv->timeline_max);
		retire_fence = sync_fence_create("ret_fence", retire_sync_pt);
		sync_fence_install(retire_fence,
				   dev_drv->win_data.ret_fence_fd);

		regs = kzalloc(sizeof(*regs), GFP_KERNEL);
		if (dev_drv->wait_fs == 1) {
			rockchip_fb_update_reg(dev_drv, regs);
			kfree(regs);
			mutex_unlock(&dev_drv->update_regs_list_lock);
		} else {
			list_add_tail(&regs->list, &dev_drv->update_regs_list);
			mutex_unlock(&dev_drv->update_regs_list_lock);

			queue_kthread_work(&dev_drv->update_regs_worker,
					   &dev_drv->update_regs_work);
		}
	} else {
		if (dev_drv->ops->lcdc_reg_update)
			dev_drv->ops->lcdc_reg_update(dev_drv);
	}

	return 0;
}

static int rockchip_fb_ioctl(struct fb_info *info, unsigned int cmd,
			 unsigned long arg)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)info->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	u32 yuv_phy[2];
	int win_id = dev_drv->ops->fb_get_win_id(dev_drv, info->fix.id);
	int enable;	/* enable fb:1 enable;0 disable */
	int ovl;	/* overlay:	0:win1 on the top of win0
			 *		1:win0 on the top of win1 */
	int num_buf;	/* buffer number */
	int ret;
	void __user *argp = (void __user *)arg;
	unsigned int dsp_addr[2];
	int list_stat;

	switch (cmd) {
	case SFA_FBIOSET_YUV_ADDR:
		/* when in video mode, buff alloc by android */
		if (copy_from_user(yuv_phy, argp, 8))
			return -EFAULT;
		fix->smem_start = yuv_phy[0];	/* for y */
		fix->mmio_start = yuv_phy[1];	/* for uv */
		break;
	case SFA_FBIOSET_ENABLE:
		if (copy_from_user(&enable, argp, sizeof(enable)))
			return -EFAULT;
		dev_drv->ops->open(dev_drv, win_id, enable);
		break;
	case SFA_FBIOGET_ENABLE:
		enable = dev_drv->ops->get_win_state(dev_drv, win_id);
		if (copy_to_user(argp, &enable, sizeof(enable)))
			return -EFAULT;
		break;
	case SFA_FBIOSET_OVERLAY_STA:
		if (copy_from_user(&ovl, argp, sizeof(ovl)))
			return -EFAULT;
		dev_drv->ops->ovl_mgr(dev_drv, ovl, 1);
		break;
	case SFA_FBIOGET_OVERLAY_STA:
		ovl = dev_drv->ops->ovl_mgr(dev_drv, 0, 0);
		if (copy_to_user(argp, &ovl, sizeof(ovl)))
			return -EFAULT;
		break;
	case SFA_FBIOPUT_NUM_BUFFERS:
		if (copy_from_user(&num_buf, argp, sizeof(num_buf)))
			return -EFAULT;
		dev_drv->num_buf = num_buf;
		dev_info(info->dev, "rockchip3gr fb use %d buffers\n", num_buf);
		break;
	case SFA_FBIOSET_VSYNC_ENABLE:
		if (copy_from_user(&enable, argp, sizeof(enable)))
			return -EFAULT;
		dev_drv->vsync_info.active = enable;
		break;
	case SFA_FBIOGET_DSP_ADDR:
		dev_drv->ops->get_dsp_addr(dev_drv, dsp_addr);
		if (copy_to_user(argp, &dsp_addr, sizeof(dsp_addr)))
			return -EFAULT;
		break;
	case SFA_FBIOGET_LIST_STA:
		list_stat = rockchip_fb_get_list_stat(dev_drv);
		if (copy_to_user(argp, &list_stat, sizeof(list_stat)))
			return -EFAULT;
		break;
	case SFA_FBIOSET_CONFIG_DONE:
		ret = copy_from_user(&dev_drv->win_data,
				(struct rockchip_fb_win_cfg_data __user *)argp,
				sizeof(dev_drv->win_data));
		dev_drv->wait_fs = dev_drv->win_data.wait_fs;
		fence_wait_begin = dev_drv->win_data.fence_begin;
		rockchip_fb_config_done(info);

		if (copy_to_user
		    ((struct rockchip_fb_win_cfg_data __user *)arg,
		     &dev_drv->win_data, sizeof(dev_drv->win_data))) {
			ret = -EFAULT;
			break;
		}
		break;
	default:
		dev_drv->ops->ioctl(dev_drv, cmd, arg, win_id);
		break;
	}
	return 0;
}

static inline unsigned int chan_to_field(unsigned int chan,
					 struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int fb_setcolreg(unsigned regno,
			unsigned red, unsigned green, unsigned blue,
			unsigned transp, struct fb_info *info)
{
	unsigned int val;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/* true-colour, use pseudo-palette */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val = chan_to_field(red, &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue, &info->var.blue);
			pal[regno] = val;
		}
		break;
	default:
		return -1;	/* unknown type */
	}

	return 0;
}

static struct fb_ops fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = rockchip_fb_open,
	.fb_release = rockchip_fb_close,
	.fb_check_var = rockchip_fb_check_var,
	.fb_set_par = rockchip_fb_set_par,
	.fb_blank = rockchip_fb_blank,
	.fb_ioctl = rockchip_fb_ioctl,
	.fb_pan_display = rockchip_fb_pan_display,
	.fb_read = rockchip_fb_read,
	.fb_write = rockchip_fb_write,
	.fb_setcolreg = fb_setcolreg,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static struct fb_var_screeninfo def_var = {
#ifdef CONFIG_LOGO_LINUX_BMP
	.red = {16, 8, 0},
	.green = {8, 8, 0},
	.blue = {0, 8, 0},
	.transp = {0, 0, 0},
	.nonstd = HAL_PIXEL_FORMAT_BGRA_8888,
#else
	.red = {11, 5, 0},
	.green = {5, 6, 0},
	.blue = {0, 5, 0},
	.transp = {0, 0, 0},
	.nonstd = HAL_PIXEL_FORMAT_RGB_565,
#endif
	.grayscale = 0,
	.activate = FB_ACTIVATE_NOW,
	.accel_flags = 0,
	.vmode = FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo def_fix = {
	.type = FB_TYPE_PACKED_PIXELS,
	.type_aux = 0,
	.xpanstep = 1,
	.ypanstep = 1,
	.ywrapstep = 0,
	.accel = FB_ACCEL_NONE,
	.visual = FB_VISUAL_TRUECOLOR,
};

static int rockchip_fb_wait_for_vsync_thread(void *data)
{
	struct rockchip_vop_driver *dev_drv =
		(struct rockchip_vop_driver *)data;
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
	struct fb_info *fbi = sfb_info->fb[0];

	while (!kthread_should_stop()) {
		ktime_t timestamp = dev_drv->vsync_info.timestamp;
		int ret = wait_event_interruptible(dev_drv->vsync_info.wait,
				(!ktime_equal(timestamp,
					      dev_drv->vsync_info.timestamp) &&
				 dev_drv->vsync_info.active) ||
				dev_drv->vsync_info.irq_stop);

		if (!ret)
			sysfs_notify(&fbi->dev->kobj, NULL, "vsync");
	}

	return 0;
}

static ssize_t rockchip_fb_vsync_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;

	return scnprintf(buf, PAGE_SIZE, "%llu\n",
			 ktime_to_ns(dev_drv->vsync_info.timestamp));
}

static DEVICE_ATTR(vsync, S_IRUGO, rockchip_fb_vsync_show, NULL);

/*
 * function: used for other module that in the kernel which
 *	need show image directly through fb
 * @fb_id: fb id to display, default we use fb0 for ui display
 */
struct fb_info *rockchip_fb_get(int fb_id)
{
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
	struct fb_info *fb = sfb_info->fb[fb_id];

	return fb;
}
EXPORT_SYMBOL(rockchip_fb_get);

void rockchip_fb_direct_show(struct fb_info *fbi)
{
	rockchip_fb_set_par(fbi);
	rockchip_fb_pan_display(&fbi->var, fbi);
}
EXPORT_SYMBOL(rockchip_fb_direct_show);

/*
 * function: this function will be called by hdmi,when
 *		hdmi plug in/out
 * @screen: the screen attached to hdmi
 * @enable:
 *	true: hdmi plug in,
 *	false: hdmi plug out
 * @vop_id: the vop id the hdmi attached,0 or 1
 */
int rockchip_fb_switch_screen(struct rockchip_screen *screen,
			  bool enable, int vop_id)
{
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
	struct fb_info *info = NULL;
	struct rockchip_vop_driver *dev_drv = NULL;
	struct fb_var_screeninfo *fb_var = NULL;
	char name[6] = { 0 };

	hdmi_switch_complete = false;
	sprintf(name, "vop%d", vop_id);
	if (sfb_info->disp_mode != DUAL)
		dev_drv = sfb_info->vop_dev_drv[0];
	else
		dev_drv = get_vop_drv(name);

	if (dev_drv == NULL) {
		pr_err("%s driver not found!\n", name);
		return -ENODEV;
	}

	pr_info("hdmi %s vop%d\n", enable ? "connect to" : "remove from",
		dev_drv->id);

	memcpy(dev_drv->cur_screen, screen, sizeof(struct rockchip_screen));

	/* the main fb of vop */
	info = sfb_info->fb[dev_drv->fb_index_base];
	fb_var = &info->var;

	fb_var->grayscale &= 0xff;
	fb_var->grayscale |= (dev_drv->cur_screen->mode.xres << 8) +
	    (dev_drv->cur_screen->mode.yres << 20);

	info->fbops->fb_open(info, 1);
	dev_drv->ops->load_screen(dev_drv, 1);
	info->fbops->fb_set_par(info);

	hdmi_switch_complete = enable;
	info->fbops->fb_pan_display(fb_var, info);
	info->fbops->fb_ioctl(info, SFA_FBIOSET_CONFIG_DONE, 0);

	return 0;
}

/*
 * function: this function called by display device for
 *		scale the display size
 * @scale_x: scale rate of x resolution
 * @scale_y: scale rate of y resolution
 * @vop_id: the vop id
 */
int rockchip_fb_disp_scale(u8 scale_x, u8 scale_y, u8 vop_id)
{
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
	struct fb_info *info = NULL;
	struct fb_var_screeninfo *var = NULL;
	struct rockchip_vop_driver *dev_drv = NULL;
	u16 screen_x, screen_y;
	u16 xpos, ypos;
	u16 xsize, ysize;
	char name[6];

	sprintf(name, "vop%d", vop_id);

	if (sfb_info->disp_mode == DUAL) {
		dev_drv = get_vop_drv(name);
		if (dev_drv == NULL) {
			pr_err("%s driver not found!\n", name);
			return -ENODEV;
		}
	} else {
		dev_drv = sfb_info->vop_dev_drv[0];
	}

	/* only win0 support scale */
	info = sfb_info->fb[dev_drv->fb_index_base];

	var = &info->var;
	screen_x = dev_drv->cur_screen->mode.xres;
	screen_y = dev_drv->cur_screen->mode.yres;

	xpos = (screen_x - screen_x * scale_x / 100) >> 1;
	ypos = (screen_y - screen_y * scale_y / 100) >> 1;
	xsize = screen_x * scale_x / 100;
	ysize = screen_y * scale_y / 100;

	/* update android disp info */
	var->nonstd &= 0xff;
	var->nonstd |= (xpos << 8) + (ypos << 20);
	var->grayscale &= 0xff;
	var->grayscale |= (xsize << 8) + (ysize << 20);

	info->fbops->fb_set_par(info);
	info->fbops->fb_ioctl(info, SFA_FBIOSET_CONFIG_DONE, 0);
	return 0;
}

#if defined(CONFIG_ION_XGOLD)
static int rockchip_fb_alloc_by_ion(struct fb_info *fbi,
				       unsigned long fb_mem_size)
{
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct ion_handle *handle;
	ion_phys_addr_t phy_addr;
	size_t len;
	int ret = 0;

	handle = ion_alloc(sfb_info->ion_client, (size_t)fb_mem_size, 0,
			   ION_HEAP_TYPE_DMA_MASK, 0);
	if (IS_ERR(handle)) {
		dev_err(fbi->dev, "failed to ion_alloc:%ld\n", PTR_ERR(handle));
		return -ENOMEM;
	}

	fb_par->ion_hdl = handle;
	fb_par->dma_buf = ion_share_dma_buf(sfb_info->ion_client, handle);
	if (IS_ERR_OR_NULL(fb_par->dma_buf)) {
		dev_err(fbi->dev, "ion_share_dma_buf() failed\n");
		goto err_share_dma_buf;
	}

	ret = ion_phys(sfb_info->ion_client, handle, &phy_addr, &len);
	if (ret < 0) {
		dev_err(fbi->dev, "ion map to get phy addr failed\n");
		goto err_share_dma_buf;
	}

	fbi->fix.smem_start = phy_addr;
	fbi->fix.smem_len = len;
	fbi->screen_base = ion_map_kernel(sfb_info->ion_client, handle);
	memset(fbi->screen_base, 0, fbi->fix.smem_len);

	return 0;

err_share_dma_buf:
	ion_free(sfb_info->ion_client, handle);
	return -ENOMEM;
}
#else
static int rockchip_fb_alloc_dma_buffer(struct fb_info *fbi,
				    unsigned long fb_mem_size)
{
	dma_addr_t fb_mem_phys;
	void *fb_mem_virt;

	fb_mem_virt =
		dma_alloc_writecombine(fbi->dev, fb_mem_size, &fb_mem_phys,
				       GFP_KERNEL);
	if (!fb_mem_virt) {
		pr_err("%s: Failed to allocate framebuffer\n",
		       __func__);
		return -ENOMEM;
	}
	fbi->fix.smem_len = fb_mem_size;
	fbi->fix.smem_start = fb_mem_phys;
	fbi->screen_base = fb_mem_virt;

	return 0;
}
#endif

static int rockchip_fb_alloc_buffer(struct fb_info *fbi, int fb_id)
{
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	int ret = 0;
	unsigned long fb_mem_size;

	if (!strcmp(fbi->fix.id, "fb0")) {
		fb_mem_size = get_fb_size();
#if defined(CONFIG_ION_XGOLD)
		if (rockchip_fb_alloc_by_ion(fbi, fb_mem_size) < 0)
			return -ENOMEM;
#else
		if (rockchip_fb_alloc_dma_buffer(fbi, fb_mem_size) < 0)
			return -ENOMEM;
#endif
	} else {
		if (dev_drv->rotate_mode > X_Y_MIRROR) {
			/* fb_mem_size = get_rotate_fb_size(); */
			fb_mem_size = get_fb_size();
#if defined(CONFIG_ION_XGOLD)
			if (rockchip_fb_alloc_by_ion(fbi, fb_mem_size) < 0)
				return -ENOMEM;
#else
			if (rockchip_fb_alloc_dma_buffer(fbi, fb_mem_size) < 0)
				return -ENOMEM;
#endif
		} else {
			fbi->fix.smem_start = sfb_info->fb[0]->fix.smem_start;
			fbi->fix.smem_len = sfb_info->fb[0]->fix.smem_len;
			fbi->screen_base = sfb_info->fb[0]->screen_base;
		}
	}
	dev_info(fbi->dev, "fb%d:phy:%lx>>vir:%p>>len:0x%x\n", fb_id,
		 fbi->fix.smem_start, fbi->screen_base, fbi->fix.smem_len);

	fbi->screen_size = fbi->fix.smem_len;
	fb_par->fb_phy_base = fbi->fix.smem_start;
	fb_par->fb_virt_base = fbi->screen_base;
	fb_par->fb_size = fbi->fix.smem_len;

	return ret;
}

static int rockchip_fb_release_buffer(struct fb_info *fbi)
{
	struct rockchip_fb_par *fb_par;
#if defined(CONFIG_ION_XGOLD)
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
#endif

	if (!fbi) {
		pr_err("no need release null fb buffer!\n");
		return -EINVAL;
	}

	fb_par = (struct rockchip_fb_par *)fbi->par;

	/* buffer for fb1 and fb3 are alloc by android */
	if (!strcmp(fbi->fix.id, "fb1") || !strcmp(fbi->fix.id, "fb3"))
		return 0;
#if defined(CONFIG_ION_XGOLD)
	if (!IS_ERR_OR_NULL(fb_par->ion_hdl))
		ion_free(sfb_info->ion_client, fb_par->ion_hdl);
#else
	dma_free_writecombine(fbi->dev, fb_par->fb_size,
			      fb_par->fb_virt_base, fb_par->fb_phy_base);
#endif
	return 0;
}

static int init_vop_win(struct rockchip_vop_driver *dev_drv,
			struct rockchip_vop_win *def_win)
{
	int i;
	int vop_win_num = dev_drv->num_win;

	for (i = 0; i < vop_win_num; i++) {
		struct rockchip_vop_win *win = NULL;

		win = kzalloc(sizeof(*win), GFP_KERNEL);
		if (!win) {
			dev_err(dev_drv->dev, "kzmalloc for win fail!");
			return -ENOMEM;
		}

		strcpy(win->name, def_win[i].name);
		win->id = def_win[i].id;
		win->support_3d = def_win[i].support_3d;
		dev_drv->win[i] = win;
	}

	return 0;
}

static int init_vop_device_driver(struct rockchip_vop_driver *dev_drv,
				  struct rockchip_vop_win *def_win, int id)
{
	struct rockchip_screen *screen =
		devm_kzalloc(dev_drv->dev, sizeof(struct rockchip_screen),
			     GFP_KERNEL);

	if (!screen) {
		dev_err(dev_drv->dev, "malloc screen for vop%d fail!",
			dev_drv->id);
		return -ENOMEM;
	}

	screen->screen_id = 0;
	screen->vop_id = dev_drv->id;

	dev_drv->screen0 = screen;
	dev_drv->cur_screen = screen;

	sprintf(dev_drv->name, "vop%d", dev_drv->id);
	init_vop_win(dev_drv, def_win);
	init_completion(&dev_drv->frame_done);
	spin_lock_init(&dev_drv->cpl_lock);
	mutex_init(&dev_drv->fb_win_id_mutex);
	dev_drv->ops->fb_win_remap(dev_drv, dev_drv->fb_win_map);
	dev_drv->first_frame = 1;

	if (dev_drv->prop == PRMRY) {
		rockchip_set_prmry_screen(screen);
		rockchip_get_prmry_screen(screen);
	}
	dev_drv->trsm_ops = rockchip_fb_trsm_ops_get(screen->type);

	return 0;
}

#ifdef CONFIG_LOGO_LINUX_BMP
static struct linux_logo *bmp_logo;
static int fb_prepare_bmp_logo(struct fb_info *info, int rotate)
{
	bmp_logo = fb_find_logo(24);
	if (bmp_logo == NULL) {
		dev_err(info->dev, "%s error\n", __func__);
		return 0;
	}
	return 1;
}

static void fb_show_bmp_logo(struct fb_info *info, int rotate)
{
	unsigned char *src = bmp_logo->data;
	unsigned char *dst = info->screen_base;
	int i;
	unsigned int needwidth = (*(src - 24) << 8) | (*(src - 23));
	unsigned int needheight = (*(src - 22) << 8) | (*(src - 21));

	for (i = 0; i < needheight; i++)
		memcpy(dst + info->var.xres * i * 4,
		       src + bmp_logo->width * i * 4, needwidth * 4);
}
#endif

int rockchip_fb_register(struct rockchip_vop_driver *dev_drv,
		     struct rockchip_vop_win *vop_win, int id)
{
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
	struct fb_info *fbi;
	struct rockchip_fb_par *fb_par = NULL;
	int i = 0, ret = 0;
	int vop_id = 0;

	if (unlikely(!dev_drv) || unlikely(!vop_win)) {
		pr_err("null vop device driver?\n");
		return -ENOENT;
	}

	for (i = 0; i < SFA_MAX_VOP_SUPPORT; i++) {
		if (NULL == sfb_info->vop_dev_drv[i]) {
			sfb_info->vop_dev_drv[i] = dev_drv;
			sfb_info->vop_dev_drv[i]->id = id;
			sfb_info->num_vop++;
			break;
		}
	}
	if (i == SFA_MAX_VOP_SUPPORT) {
		pr_err("rockchip_fb_register vop out of support %d\n", i);
		return -ENOENT;
	}
	vop_id = i;
	init_vop_device_driver(dev_drv, vop_win, id);

	dev_drv->fb_index_base = sfb_info->num_fb;
	for (i = 0; i < dev_drv->num_win; i++) {
		fbi = framebuffer_alloc(0, &fb_pdev->dev);
		if (!fbi) {
			dev_err(&fb_pdev->dev, ">> fb framebuffer_alloc fail!");
			fbi = NULL;
			ret = -ENOMEM;
		}
		fb_par =
		    devm_kzalloc(&fb_pdev->dev, sizeof(struct rockchip_fb_par),
				 GFP_KERNEL);
		if (!fb_par) {
			dev_err(&fb_pdev->dev, "malloc fb_par for fb%d fail!",
				sfb_info->num_fb);
			return -ENOMEM;
		}
		fb_par->id = sfb_info->num_fb;
		fb_par->vop_drv = dev_drv;
		fbi->par = fb_par;
		fbi->var = def_var;
		fbi->fix = def_fix;
		sprintf(fbi->fix.id, "fb%d", sfb_info->num_fb);
		fb_videomode_to_var(&fbi->var, &dev_drv->cur_screen->mode);
		fbi->var.grayscale |=
		    (fbi->var.xres << 8) + (fbi->var.yres << 20);
#if defined(CONFIG_LOGO_LINUX_BMP)
		fbi->var.bits_per_pixel = 32;
#else
		fbi->var.bits_per_pixel = 16;
#endif
		fbi->fix.line_length =
		    (fbi->var.xres_virtual) * (fbi->var.bits_per_pixel >> 3);
		fbi->var.width = dev_drv->cur_screen->width;
		fbi->var.height = dev_drv->cur_screen->height;
		fbi->var.pixclock = dev_drv->pixclock;

		fbi->fbops = &fb_ops;
		fbi->flags = FBINFO_FLAG_DEFAULT;
		fbi->pseudo_palette = dev_drv->win[i]->pseudo_pal;

		if (i == 0)	/* only alloc memory for main fb */
			rockchip_fb_alloc_buffer(fbi, 0);

		ret = register_framebuffer(fbi);
		if (ret < 0) {
			dev_err(&fb_pdev->dev,
				"%s fb%d register_framebuffer fail!\n",
				__func__, sfb_info->num_fb);
			return ret;
		}

		rockchip_fb_create_sysfs(fbi);
		sfb_info->fb[sfb_info->num_fb] = fbi;
		dev_info(fbi->dev, "rockchip framebuffer registered:%s\n",
			 fbi->fix.id);
		sfb_info->num_fb++;

		if (i == 0) {
			init_waitqueue_head(&dev_drv->vsync_info.wait);
			ret = device_create_file(fbi->dev, &dev_attr_vsync);
			if (ret)
				dev_err(fbi->dev,
					"failed to create vsync file\n");

			dev_drv->vsync_info.thread =
			    kthread_run(rockchip_fb_wait_for_vsync_thread,
					dev_drv, "fb-vsync");

			if (dev_drv->vsync_info.thread == ERR_PTR(-ENOMEM)) {
				dev_err(fbi->dev,
					"failed to run vsync thread\n");
				dev_drv->vsync_info.thread = NULL;
			}
			dev_drv->vsync_info.active = 1;

			INIT_LIST_HEAD(&dev_drv->update_regs_list);
			mutex_init(&dev_drv->update_regs_list_lock);
			init_kthread_worker(&dev_drv->update_regs_worker);

			dev_drv->update_regs_thread =
			    kthread_run(kthread_worker_fn,
					&dev_drv->update_regs_worker,
					"rockchip-fb");
			if (IS_ERR(dev_drv->update_regs_thread)) {
				int err = PTR_ERR(dev_drv->update_regs_thread);

				dev_drv->update_regs_thread = NULL;
				pr_err("failed to run update_regs thread\n");
				return err;
			}
			init_kthread_work(&dev_drv->update_regs_work,
					  rockchip_fb_update_regs_handler);

			dev_drv->timeline =
				sw_sync_timeline_create("rockchip-fb");
			dev_drv->timeline_max = 1;
		}
	}

#if !defined(CONFIG_FRAMEBUFFER_CONSOLE) && defined(CONFIG_LOGO)
	/* show logo for primary display device */
	if (dev_drv->prop == PRMRY) {
		struct fb_info *main_fbi = sfb_info->fb[0];

		main_fbi->fbops->fb_open(main_fbi, 1);
		if (support_uboot_display())
			return 0;
		main_fbi->fbops->fb_set_par(main_fbi);

#if  defined(CONFIG_LOGO_LINUX_BMP)
		if (fb_prepare_bmp_logo(main_fbi, FB_ROTATE_UR)) {
			fb_set_cmap(&main_fbi->cmap, main_fbi);
			fb_show_bmp_logo(main_fbi, FB_ROTATE_UR);
		}
#else
		if (fb_prepare_logo(main_fbi, FB_ROTATE_UR)) {
			fb_set_cmap(&main_fbi->cmap, main_fbi);
			fb_show_logo(main_fbi, FB_ROTATE_UR);
		}
#endif
		main_fbi->fbops->fb_pan_display(&main_fbi->var, main_fbi);
		main_fbi->fbops->fb_ioctl(main_fbi, SFA_FBIOSET_CONFIG_DONE, 0);
	}
#endif
	return 0;
}

int rockchip_fb_unregister(struct rockchip_vop_driver *dev_drv)
{
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
	struct fb_info *fbi;
	int fb_index_base = dev_drv->fb_index_base;
	int fb_num = dev_drv->num_win;
	int i = 0;

	if (NULL == dev_drv) {
		pr_err("no need to unregister null vop device driver!\n");
		return -ENOENT;
	}

	if (sfb_info->vop_dev_drv[i]->vsync_info.thread) {
		sfb_info->vop_dev_drv[i]->vsync_info.irq_stop = 1;
		kthread_stop(sfb_info->vop_dev_drv[i]->vsync_info.thread);
	}

	for (i = 0; i < fb_num; i++)
		kfree(dev_drv->win[i]);

	for (i = fb_index_base; i < (fb_index_base + fb_num); i++) {
		fbi = sfb_info->fb[i];
		unregister_framebuffer(fbi);
		rockchip_fb_release_buffer(fbi);
		framebuffer_release(fbi);
	}
	sfb_info->vop_dev_drv[dev_drv->id] = NULL;
	sfb_info->num_vop--;

	return 0;
}

static int rockchip_fb_probe(struct platform_device *pdev)
{
	struct rockchip_fb *sfb_info = NULL;
	struct device_node *np = pdev->dev.of_node;
	u32 mode;

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}

	sfb_info = devm_kzalloc(&pdev->dev, sizeof(struct rockchip_fb),
				GFP_KERNEL);
	if (!sfb_info) {
		dev_err(&pdev->dev, "kmalloc for rockchip3gr fb fail!");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, sfb_info);

	if (!of_property_read_u32(np, "rockchip,disp-mode", &mode)) {
		sfb_info->disp_mode = mode;

	} else {
		dev_err(&pdev->dev, "no disp-mode node found!");
		return -ENODEV;
	}

	if (!of_property_read_u32(np, "rockchip,uboot-logo-on", &uboot_logo_on))
		pr_info("uboot-logo-on:%d\n", uboot_logo_on);

	dev_set_name(&pdev->dev, "rockchip-fb");
#if defined(CONFIG_ION_XGOLD)
	sfb_info->ion_client = xgold_ion_client_create("rockchip-fb");
	if (IS_ERR(sfb_info->ion_client)) {
		dev_err(&pdev->dev,
			"failed to create ion client for rockchip fb");
		return PTR_ERR(sfb_info->ion_client);
	}
#endif

	fb_pdev = pdev;
	dev_info(&pdev->dev, "rockchip framebuffer driver probe\n");
	return 0;
}

static int rockchip_fb_remove(struct platform_device *pdev)
{
	struct rockchip_fb *sfb_info = platform_get_drvdata(pdev);

	kfree(sfb_info);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void rockchip_fb_shutdown(struct platform_device *pdev)
{
	struct rockchip_fb *sfb_info = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < sfb_info->num_vop; i++) {
		if (!sfb_info->vop_dev_drv[i])
			continue;
	}
}

static const struct of_device_id rockchip_fb_dt_ids[] = {
	{.compatible = "rockchip,rockchip-fb",},
	{}
};

static struct platform_driver rockchip_fb_driver = {
	.probe = rockchip_fb_probe,
	.remove = rockchip_fb_remove,
	.driver = {
		   .name = "rockchip-fb",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rockchip_fb_dt_ids),
		   },
	.shutdown = rockchip_fb_shutdown,
};

module_platform_driver(rockchip_fb_driver);
