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

#define H_USE_FENCE	1
#define ION_HEAP(bit) (1 << (bit))

static bool hdmi_switch_complete;
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

static int rockchip_fb_get_pixel_width(int data_format)
{
	int pixel_width;

	switch (data_format) {
	case XBGR888:
	case ABGR888:
	case ARGB888:
		pixel_width = 4 * 8;
		break;
	case RGB888:
		pixel_width = 3 * 8;
		break;
	case RGB565:
		pixel_width = 2 * 8;
		break;
	case YUV422:
	case YUV420:
	case YUV444:
		pixel_width = 1 * 8;
		break;
	case YUV422_A:
	case YUV420_A:
	case YUV444_A:
		pixel_width = 8;
		break;
	default:
		pr_err("%s:un supported format:0x%x\n", __func__, data_format);
		return -EINVAL;
	}
	return pixel_width;
}

static int rockchip_fb_get_data_fmt(int data_format)
{
	int fb_data_fmt;

	switch (data_format) {
	case HAL_PIXEL_FORMAT_RGBX_8888:
		fb_data_fmt = XBGR888;
		break;
	case HAL_PIXEL_FORMAT_RGBA_8888:
		fb_data_fmt = ABGR888;
		break;
	case HAL_PIXEL_FORMAT_BGRA_8888:
		fb_data_fmt = ARGB888;
		break;
	case HAL_PIXEL_FORMAT_RGB_888:
		fb_data_fmt = RGB888;
		break;
	case HAL_PIXEL_FORMAT_RGB_565:
		fb_data_fmt = RGB565;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_422_SP:	/* yuv422 */
		fb_data_fmt = YUV422;
		break;
	case HAL_PIXEL_FORMAT_YCrCb_NV12:	/* YUV420---uvuvuv */
		fb_data_fmt = YUV420;
		break;
	case HAL_PIXEL_FORMAT_YCrCb_444:	/* yuv444 */
		fb_data_fmt = YUV444;
		break;
	case HAL_PIXEL_FORMAT_YCrCb_NV12_10:	/* yuv444 */
		fb_data_fmt = YUV420_A;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_422_SP_10:	/* yuv444 */
		fb_data_fmt = YUV422_A;
		break;
	case HAL_PIXEL_FORMAT_YCrCb_420_SP_10:	/* yuv444 */
		fb_data_fmt = YUV444_A;
		break;
	default:
		pr_err("%s:un supported format:0x%x\n", __func__, data_format);
		return -EINVAL;
	}

	return fb_data_fmt;
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

static void rockchip_fb_fence_wait(struct rockchip_vop_driver *dev_drv,
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

static int rockchip_fb_check_config_var(struct rockchip_fb_area_par *area_par,
				    struct rockchip_screen *screen)
{
	if ((area_par->x_offset + area_par->xact > area_par->xvir) ||
	    (area_par->xact <= 0) || (area_par->yact <= 0) ||
	    (area_par->xvir <= 0) || (area_par->yvir <= 0)) {
		pr_err("check config var fail 0:\n"
		       "x_offset=%d,xact=%d,xvir=%d\n",
		       area_par->x_offset, area_par->xact, area_par->xvir);
		return -EINVAL;
	}

	if ((area_par->xpos + area_par->xsize > screen->mode.xres) ||
	    (area_par->ypos + area_par->ysize > screen->mode.yres) ||
	    (area_par->xsize <= 0) || (area_par->ysize <= 0)) {
		pr_err("check config var fail 1:\n"
		       "xpos=%d,xsize=%d,xres=%d\n"
		       "ypos=%d,ysize=%d,yres=%d\n",
		       area_par->xpos, area_par->xsize, screen->mode.xres,
		       area_par->ypos, area_par->ysize, screen->mode.yres);
		return -EINVAL;
	}
	return 0;
}

static void rockchip_fb_free_dma_buf(struct rockchip_vop_driver *dev_drv,
				 struct rockchip_vop_win *win)
{
	int i;
	struct rockchip_vop_win_area *area;
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);

	if (unlikely(!dev_drv) || unlikely(!win))
		return;

	for (i = 0; i < win->area_num; i++) {
		area = &win->area[i];

		if (area->ion_hdl)
			ion_free(sfb_info->ion_client, area->ion_hdl);

		if (area->acq_fence)
			sync_fence_put(area->acq_fence);
	}
	memset(win, 0, sizeof(*win));
}

static void rockchip_fb_free_last_regs(struct rockchip_vop_driver *dev_drv,
				   struct rockchip_fb_reg_data *regs)
{
	int i = 0;
	struct rockchip_vop_win *win = NULL;

	if (unlikely(!dev_drv) || unlikely(!regs))
		return;

	mutex_lock(&dev_drv->regs_lock);

	for (i = 0; i < regs->win_num; i++) {
		win = &regs->vop_win[i];
		rockchip_fb_free_dma_buf(dev_drv, win);
	}
	kfree(regs);

	mutex_unlock(&dev_drv->regs_lock);
}

static int rockchip_fb_set_win_par(struct fb_info *info,
			       struct rockchip_fb_win_par *win_par,
			       struct rockchip_vop_win *vop_win)
{
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)info->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	struct rockchip_screen *screen = dev_drv->cur_screen;
	struct rockchip_screen pmy_screen;
	struct fb_fix_screeninfo *fix = &info->fix;
	u8 pixel_width = 0;
	u32 vir_width_bit;
	u32 stride, uv_stride;
	u32 stride_32bit_1 = 0;
	u32 stride_32bit_2 = 0;
	u32 xvir = 0, yvir = 0;
	u32 xoffset = 0, yoffset = 0;
	u16 uv_x_off, uv_y_off, uv_y_act;
	bool is_pic_yuv = false;
	u8 ppixel_a = 0, global_a = 0;
	int i = 0;

	vop_win->id = win_par->win_id;
	vop_win->z_order = win_par->z_order;

	rockchip_get_prmry_screen(&pmy_screen);
	for (i = 0; i < vop_win->area_num; i++) {
		vop_win->area[i].format = rockchip_fb_get_data_fmt(
				win_par->area_par[i].data_format);
		pixel_width = rockchip_fb_get_pixel_width(
				vop_win->area[i].format);

		rockchip_fb_check_config_var(
				&win_par->area_par[i], &pmy_screen);

		/* visiable pos in panel */
		vop_win->area[i].xpos = win_par->area_par[i].xpos;
		vop_win->area[i].ypos = win_par->area_par[i].ypos;

		/* realy size in panel */
		vop_win->area[i].xsize = win_par->area_par[i].xsize;
		vop_win->area[i].ysize = win_par->area_par[i].ysize;

		/* active size in panel */
		vop_win->area[i].xact = win_par->area_par[i].xact;
		vop_win->area[i].yact = win_par->area_par[i].yact;

		xoffset = win_par->area_par[i].x_offset;	/* buf offset */
		yoffset = win_par->area_par[i].y_offset;
		xvir = win_par->area_par[i].xvir;
		yvir = win_par->area_par[i].yvir;

		vop_win->area[i].xvir = xvir;
		vop_win->area[i].yvir = yvir;

		vir_width_bit = pixel_width * xvir;
		stride_32bit_1 =  ALIGN_N_TIMES(vir_width_bit, 32) / 8;
		stride_32bit_2 =  ALIGN_N_TIMES(vir_width_bit * 2, 32) / 8;

		stride = stride_32bit_1;	/* default rgb */
		fix->line_length = stride;
		vop_win->area[i].y_vir_stride = stride >> 2;

		if (screen->interlace == 1)
			vop_win->area[i].y_offset =
				yoffset * stride * 2 +
				xoffset * pixel_width / 8;
		else
			vop_win->area[i].y_offset =
				yoffset * stride + xoffset * pixel_width / 8;
	}

	/* update alpha config */
	ppixel_a = ((vop_win->area[0].format == ARGB888) ||
		    (vop_win->area[0].format == ABGR888)) ? 1 : 0;
	global_a = (win_par->g_alpha_val == 0) ? 0 : 1;
	vop_win->alpha_en = ppixel_a | global_a;
	vop_win->g_alpha_val = win_par->g_alpha_val;
	vop_win->alpha_mode = win_par->alpha_mode;

	/* only win0 support yuv and win0 only have a area */
	switch (vop_win->area[0].format) {
	case YUV422:
	case YUV422_A:
		is_pic_yuv = true;
		stride = stride_32bit_1;
		uv_stride = stride_32bit_1 >> 1;
		uv_x_off = xoffset >> 1;
		uv_y_off = yoffset;
		fix->line_length = stride;
		uv_y_act = win_par->area_par[0].yact >> 1;
		break;
	case YUV420:
	case YUV420_A:
		is_pic_yuv = true;
		stride = stride_32bit_1;
		uv_stride = stride_32bit_1;
		uv_x_off = xoffset;
		uv_y_off = yoffset >> 1;
		fix->line_length = stride;
		uv_y_act = win_par->area_par[0].yact >> 1;
		break;
	case YUV444:
	case YUV444_A:
		is_pic_yuv = true;
		stride = stride_32bit_1;
		uv_stride = stride_32bit_2;
		uv_x_off = xoffset * 2;
		uv_y_off = yoffset;
		fix->line_length = stride << 2;
		uv_y_act = win_par->area_par[0].yact;
		break;
	default:
		break;
	}

	if (is_pic_yuv) {
		vop_win->area[0].cbr_start =
			vop_win->area[0].smem_start + xvir * yvir;
		vop_win->area[0].uv_vir_stride = uv_stride >> 2;

		if (screen->interlace == 1) {
			vop_win->area[0].c_offset =
				uv_y_off * uv_stride * 2 +
				uv_x_off * pixel_width / 8;
		} else {
			vop_win->area[0].c_offset =
				uv_y_off * uv_stride +
				uv_x_off * pixel_width / 8;
		}
	}

	return 0;
}

static int rockchip_fb_set_win_buffer(struct fb_info *info,
				  struct rockchip_fb_win_par *win_par,
				  struct rockchip_vop_win *vop_win)
{
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
	struct ion_handle *hdl;
	ion_phys_addr_t phy_addr;
	int ion_fd, acq_fence_fd;
	size_t len;
	int ret = 0, i = 0;

	vop_win->area[0].smem_start = 0;
	vop_win->area_num = 0;

	if (win_par->area_par[0].phy_addr == 0) {
		for (i = 0; i < SFA_WIN_MAX_AREA; i++) {
			ion_fd = win_par->area_par[i].ion_fd;
			if (ion_fd <= 0)
				continue;

			hdl = ion_import_dma_buf(sfb_info->ion_client,
						 ion_fd);
			if (IS_ERR(hdl)) {
				pr_info("%s: Could not import handle: %d\n",
					__func__, (int)hdl);
				/*return -EINVAL; */
				break;
			}
			vop_win->area_num++;
			vop_win->area[i].ion_hdl = hdl;

			ret = ion_phys(sfb_info->ion_client, hdl, &phy_addr,
				       &len);
			if (ret < 0) {
				pr_err("%s:ion map to get phy addr failed\n",
				       __func__);
				ion_free(sfb_info->ion_client, hdl);
				return -ENOMEM;
			}
			vop_win->area[i].smem_start = phy_addr;
			vop_win->area_buf_num++;
		}
	} else {
		vop_win->area[0].smem_start = win_par->area_par[0].phy_addr;
		vop_win->area_num = 1;
	}

	if (vop_win->area[0].smem_start == 0 || vop_win->area_num == 0)
		return 0;

	for (i = 0; i < vop_win->area_num; i++) {
		acq_fence_fd = win_par->area_par[i].acq_fence_fd;
		if (acq_fence_fd > 0)
			vop_win->area[i].acq_fence =
				sync_fence_fdget(acq_fence_fd);
	}

	return 0;
}

static int rockchip_fb_get_win_from_regs(struct rockchip_fb_reg_data *regs,
				     struct rockchip_vop_win *vop_win)
{
	int i;
	struct rockchip_vop_win *regs_win = NULL;

	if (unlikely(!regs) || unlikely(!vop_win))
		return 0;

	for (i = 0; i < regs->win_num; i++) {
		if (regs->vop_win[i].id == vop_win->id) {
			regs_win = &regs->vop_win[i];
			break;
		}
	}
	if (regs_win == NULL)
		return -EINVAL;

	memcpy(vop_win, regs_win, sizeof(*regs_win));
	return 0;
}

static void rockchip_fb_update_reg(struct rockchip_vop_driver *dev_drv,
			       struct rockchip_fb_reg_data *regs)
{
	int i, j, ret;
	struct rockchip_vop_win *win;
	ktime_t timestamp = dev_drv->vsync_info.timestamp;
	bool wait_for_vsync;
	int count = 100;
	unsigned int dsp_addr[4];
	long timeout;
	u32 new_start, reg_start;

	/* acq_fence wait */
	for (i = 0; i < regs->win_num; i++) {
		win = &regs->vop_win[i];
		for (j = 0; j < SFA_WIN_MAX_AREA; j++) {
			if (win->area[j].acq_fence)
				rockchip_fb_fence_wait(dev_drv,
						   win->area[j].acq_fence);
		}
	}

	for (i = 0; i < dev_drv->num_win; i++) {
		win = dev_drv->win[i];
		ret = rockchip_fb_get_win_from_regs(regs, win);

		if (ret == 0) {
			mutex_lock(&dev_drv->win_cfg_lock);
			win->state = 1;
			dev_drv->ops->set_par(dev_drv, i);
			dev_drv->ops->pan_display(dev_drv, i);
			mutex_unlock(&dev_drv->win_cfg_lock);
		} else {
			win->z_order = -1;
			win->state = 0;
		}
	}

	dev_drv->ops->ovl_mgr(dev_drv, 0, 1);
	dev_drv->ops->cfg_done(dev_drv);

	do {
		timestamp = dev_drv->vsync_info.timestamp;
		timeout = wait_event_interruptible_timeout(
				dev_drv->vsync_info.wait,
				ktime_compare(
					dev_drv->vsync_info.timestamp,
					timestamp) > 0,
				msecs_to_jiffies(25));

		dev_drv->ops->get_dsp_addr(dev_drv, dsp_addr);
		wait_for_vsync = false;
		for (i = 0; i < dev_drv->num_win; i++) {
			if (dev_drv->win[i]->state == 1) {
				new_start =
					dev_drv->win[i]->area[0].smem_start +
					dev_drv->win[i]->area[0].y_offset;
				reg_start = dsp_addr[i];

				if (unlikely(new_start != reg_start)) {
					wait_for_vsync = true;
					dev_dbg(dev_drv->dev,
						 "win%d:new_addr:0x%08x cur_addr:0x%08x--%d\n",
						 i,
						 new_start,
						 reg_start,
						 101 - count);
					break;
				}
			}
		}
	} while (wait_for_vsync && count--);

#ifdef H_USE_FENCE
	sw_sync_timeline_inc(dev_drv->timeline, 1);
#endif

	if (dev_drv->last_regs)
		rockchip_fb_free_last_regs(dev_drv, dev_drv->last_regs);

	mutex_lock(&dev_drv->regs_lock);
	dev_drv->last_regs = regs;
	mutex_unlock(&dev_drv->regs_lock);
}

static void rockchip_fb_update_regs_handler(struct kthread_work *work)
{
	struct rockchip_vop_driver *dev_drv =
	    container_of(work, struct rockchip_vop_driver, update_regs_work);
	struct rockchip_fb_reg_data *data, *next;
	/* struct list_head saved_list; */

	mutex_lock(&dev_drv->update_regs_list_lock);
	saved_list = dev_drv->update_regs_list;
	list_replace_init(&dev_drv->update_regs_list, &saved_list);
	mutex_unlock(&dev_drv->update_regs_list_lock);

	list_for_each_entry_safe(data, next, &saved_list, list) {
		rockchip_fb_update_reg(dev_drv, data);
		list_del(&data->list);
	}

	if (dev_drv->wait_fs && list_empty(&dev_drv->update_regs_list))
		wake_up(&dev_drv->update_regs_wait);
}

static int rockchip_fb_update_win_config(struct fb_info *info,
				     struct rockchip_fb_win_cfg_data *win_data)
{
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)info->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	struct rockchip_fb_reg_data *regs;
	struct rockchip_fb_win_par *win_par;
	struct rockchip_vop_win *vop_win;
	int ret = 0, i = 0, j = 0;
	int list_is_empty = 0;

#ifdef H_USE_FENCE
	struct sync_fence *release_fence[SFA_MAX_BUF_NUM];
	struct sync_fence *retire_fence;
	struct sync_pt *release_sync_pt[SFA_MAX_BUF_NUM];
	struct sync_pt *retire_sync_pt;
	char fence_name[20];
#endif

	regs = kzalloc(sizeof(*regs), GFP_KERNEL);
	if (!regs) {
		pr_info("kmalloc rockchip_fb_reg_data failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < dev_drv->num_win; i++) {
		win_par = &win_data->win_par[i];
		vop_win = &regs->vop_win[j];
		if (win_par->win_id >= dev_drv->num_win) {
			pr_err("error:win_id bigger than vop win_num\n");
			continue;
		}

		ret = rockchip_fb_set_win_buffer(info, win_par, vop_win);
		if (ret < 0)
			return -ENOMEM;

		ret = rockchip_fb_set_win_par(info, win_par, vop_win);
		regs->win_num++;
		regs->buf_num += vop_win->area_buf_num;
		j++;
	}

	mutex_lock(&dev_drv->cfg_lock);
	if (!(dev_drv->suspend_flag == 0)) {
		rockchip_fb_update_reg(dev_drv, regs);
		pr_info("%s: suspend_flag = 1\n", __func__);
		goto err_out;
	}

	dev_drv->timeline_max++;
#ifdef H_USE_FENCE
	for (i = 0; i < SFA_MAX_BUF_NUM; i++) {
		if (i < regs->buf_num) {
			sprintf(fence_name, "fence%d", i);
			win_data->rel_fence_fd[i] = get_unused_fd();
			if (win_data->rel_fence_fd[i] < 0) {
				pr_info("get fence fd failed,rel_fence_fd=%d\n",
					win_data->rel_fence_fd[i]);
				ret = -EFAULT;
				goto err_out;
			}
			release_sync_pt[i] =
			    sw_sync_pt_create(dev_drv->timeline,
					      dev_drv->timeline_max);
			release_fence[i] =
			    sync_fence_create(fence_name, release_sync_pt[i]);
			sync_fence_install(release_fence[i],
					   win_data->rel_fence_fd[i]);
		} else {
			win_data->rel_fence_fd[i] = -1;
		}
	}

	win_data->ret_fence_fd = get_unused_fd();
	if (win_data->ret_fence_fd < 0) {
		pr_info("ret_fence_fd=%d\n", win_data->ret_fence_fd);
		ret = -EFAULT;
		goto err_out;
	}
	retire_sync_pt =
	    sw_sync_pt_create(dev_drv->timeline, dev_drv->timeline_max);
	retire_fence = sync_fence_create("ret_fence", retire_sync_pt);
	sync_fence_install(retire_fence, win_data->ret_fence_fd);
#else
	for (i = 0; i < SFA_MAX_BUF_NUM; i++)
		win_data->rel_fence_fd[i] = -1;

	win_data->ret_fence_fd = -1;
#endif

	if (dev_drv->wait_fs == 0) {
		mutex_lock(&dev_drv->update_regs_list_lock);
		list_add_tail(&regs->list, &dev_drv->update_regs_list);
		mutex_unlock(&dev_drv->update_regs_list_lock);
		queue_kthread_work(&dev_drv->update_regs_worker,
				   &dev_drv->update_regs_work);
	} else {
		mutex_lock(&dev_drv->update_regs_list_lock);
		list_is_empty = rockchip_fb_get_list_stat(dev_drv);
		mutex_unlock(&dev_drv->update_regs_list_lock);
		if (!list_is_empty) {
			ret = wait_event_timeout(dev_drv->update_regs_wait,
					rockchip_fb_get_list_stat(dev_drv),
					msecs_to_jiffies(60));
			if (ret > 0)
				rockchip_fb_update_reg(dev_drv, regs);
			else
				pr_info("%s: wait update_regs_wait timeout\n",
					__func__);
		} else if (ret == 0) {
			rockchip_fb_update_reg(dev_drv, regs);
		}
	}

err_out:
	mutex_unlock(&dev_drv->cfg_lock);
	return ret;
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
	if (win->area[0].format == RGB565)
		total_size = win->area[0].xact * win->area[0].yact << 1;
	else
		total_size = win->area[0].xact * win->area[0].yact << 2;

	if (p >= total_size)
		return 0;

	if (count >= total_size)
		count = total_size;

	if (count + p > total_size)
		count = total_size - p;

	buffer = kmalloc((count > PAGE_SIZE) ? PAGE_SIZE : count, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	src = (u8 __iomem *)(info->screen_base + p + win->area[0].y_offset);

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
	if (win->area[0].format == RGB565)
		total_size = win->area[0].xact * win->area[0].yact << 1;
	else
		total_size = win->area[0].xact * win->area[0].yact << 2;

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

	dst = (u8 __iomem *)(info->screen_base + p + win->area[0].y_offset);

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
		win->area[0].format = XBGR888;
		stride = 4 * xvir;
		fix->line_length = stride;
		if (screen->interlace == 1)
			win->area[0].y_offset =
				yoffset * stride * 2 + xoffset * 4;
		else
			win->area[0].y_offset = yoffset * stride + xoffset * 4;
		break;
	case HAL_PIXEL_FORMAT_RGBA_8888:
		win->area[0].format = ABGR888;
		stride = 4 * xvir;
		fix->line_length = stride;
		if (screen->interlace == 1)
			win->area[0].y_offset =
				yoffset * stride * 2 + xoffset * 4;
		else
			win->area[0].y_offset = yoffset * stride + xoffset * 4;
		break;
	case HAL_PIXEL_FORMAT_BGRA_8888:
		win->area[0].format = ARGB888;
		stride = 4 * xvir;
		fix->line_length = stride;
		if (screen->interlace == 1)
			win->area[0].y_offset =
				yoffset * stride * 2 + xoffset * 4;
		else
			win->area[0].y_offset = yoffset * stride + xoffset * 4;
		break;
	case HAL_PIXEL_FORMAT_RGB_888:
		win->area[0].format = RGB888;
		stride = 3 * xvir;
		fix->line_length = stride;
		if (screen->interlace == 1)
			win->area[0].y_offset =
				yoffset * stride * 2 + xoffset * 3;
		else
			win->area[0].y_offset = yoffset * stride + xoffset * 3;
		break;
	case HAL_PIXEL_FORMAT_RGB_565:
		win->area[0].format = RGB565;
		stride = 2 * xvir;
		fix->line_length = stride;
		if (screen->interlace == 1)
			win->area[0].y_offset =
				yoffset * stride * 2 + xoffset * 2;
		else
			win->area[0].y_offset = yoffset * stride + xoffset * 2;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_422_SP:
		win->area[0].format = YUV422;
		stride = xvir;
		uv_stride = stride >> 1;
		fix->line_length = stride;
		cblen = (xvir * yvir) >> 1;
		crlen = cblen;
		if (screen->interlace == 1) {
			win->area[0].y_offset = yoffset * stride * 2 + xoffset;
			win->area[0].c_offset =
				yoffset * uv_stride * 2 + (xoffset >> 1);
		} else {
			win->area[0].y_offset = yoffset * stride + xoffset;
			win->area[0].c_offset =
				yoffset * uv_stride + (xoffset >> 1);
		}
		break;
	case HAL_PIXEL_FORMAT_YCrCb_NV12:
		win->area[0].format = YUV420;
		stride = xvir;
		uv_stride = stride;
		fix->line_length = stride;
		cblen = (xvir * yvir) >> 2;
		crlen = cblen;
		if (screen->interlace == 1) {
			win->area[0].y_offset = yoffset * stride * 2 + xoffset;
			win->area[0].c_offset = yoffset  * uv_stride + xoffset;
		} else {
			win->area[0].y_offset = yoffset * stride + xoffset;
			win->area[0].c_offset =
				(yoffset >> 1) * uv_stride + xoffset;
		}
		break;
	case HAL_PIXEL_FORMAT_YCrCb_444:
		win->area[0].format = YUV444;
		stride = xvir;
		uv_stride = stride << 1;
		fix->line_length = stride << 2;
		if (screen->interlace == 1) {
			win->area[0].y_offset = yoffset * stride * 2 + xoffset;
			win->area[0].c_offset =
				yoffset * uv_stride * 2 + (xoffset << 1);
		} else {
			win->area[0].y_offset = yoffset * stride + xoffset;
			win->area[0].c_offset =
				yoffset * uv_stride + (xoffset << 1);
		}
		cblen = (xvir * yvir);
		crlen = cblen;
		break;
	default:
		dev_err(dev_drv->dev, "%s:un supported format:0x%x\n",
			__func__, data_format);
		return -EINVAL;
	}

	win->area[0].y_vir_stride = stride >> 2;
	win->area[0].uv_vir_stride = uv_stride >> 2;
	win->area[0].xpos = xpos;
	win->area[0].ypos = ypos;
	win->area[0].xsize = xsize;
	win->area[0].ysize = ysize;

	win->area[0].smem_start = fix->smem_start;
	win->area[0].cbr_start = fix->mmio_start;
	win->area[0].xact = var->xres;
	win->area[0].yact = var->yres;
	win->area[0].xvir = var->xres_virtual;	/* virtual resolution stride */
	win->area[0].yvir = var->yres_virtual;

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

	switch (win->area[0].format) {
	case XBGR888:
	case ARGB888:
	case ABGR888:
		win->area[0].y_offset = (yoffset * xvir + xoffset) * 4;
		break;
	case RGB888:
		win->area[0].y_offset = (yoffset * xvir + xoffset) * 3;
		break;
	case RGB565:
		win->area[0].y_offset = (yoffset * xvir + xoffset) * 2;
		break;
	case YUV422:
		win->area[0].y_offset = yoffset * xvir + xoffset;
		win->area[0].c_offset = win->area[0].y_offset;
		break;
	case YUV420:
		win->area[0].y_offset = yoffset * xvir + xoffset;
		win->area[0].c_offset = (yoffset >> 1) * xvir + xoffset;
		break;
	case YUV444:
		win->area[0].y_offset = yoffset * xvir + xoffset;
		win->area[0].c_offset = yoffset * 2 * xvir + (xoffset << 1);
		break;
	default:
		dev_err(dev_drv->dev, "un supported format:0x%x\n",
			data_format);
		return -EINVAL;
	}

	dev_drv->ops->pan_display(dev_drv, win_id);

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
	void __user *argp = (void __user *)arg;
	unsigned int dsp_addr[2];
	int list_stat;
	struct rockchip_fb_win_cfg_data *win_data;
#if defined(CONFIG_ION_XGOLD)
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
	struct ion_handle *hdl;
	ion_phys_addr_t phy_addr;
	size_t len;
	int fd;
#endif

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
		dev_info(info->dev, "rockchip fb use %d buffers\n", num_buf);
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

#if defined(CONFIG_ION_XGOLD)
	case SFA_FBIOSET_DMABUF_FD:
		if (copy_from_user(&fd, argp, sizeof(fd)))
			return -EFAULT;
		if (fd < 0) {
			dev_err(info->dev, "dmabuf fd is error\n");
			return -EFAULT;
		}
		hdl = ion_import_dma_buf(sfb_info->ion_client, fd);
		if (IS_ERR_OR_NULL(hdl)) {
			dev_err(info->dev, "import dma buf ion handle error\n");
			return PTR_ERR(hdl);
		}
		ion_phys(sfb_info->ion_client, hdl, &phy_addr, &len);
		fix->smem_start = phy_addr;
		break;
	case SFA_FBIOGET_DMABUF_FD:
		fd = -1;
		if (IS_ERR_OR_NULL(fb_par->ion_hdl)) {
			dev_err(info->dev,
				"get dma_buf fd failed,ion handle is err\n");
			return PTR_ERR(fb_par->ion_hdl);
		}
		fd = ion_share_dma_buf_fd(sfb_info->ion_client,
					  fb_par->ion_hdl);
		if (fd < 0) {
			dev_err(info->dev,
				"ion_share_dma_buf_fd failed\n");
			return fd;
		}
		if (copy_to_user(argp, &fd, sizeof(fd)))
			return -EFAULT;
		break;
#endif
	case SFA_FBIOSET_CONFIG_DONE:
		win_data = kzalloc(sizeof(*win_data), GFP_KERNEL);
		if (NULL == win_data)
			return -ENOMEM;

		if (copy_from_user(win_data,
				(struct rockchip_fb_win_cfg_data __user *)argp,
				sizeof(*win_data))) {
			kfree(win_data);
			return -EFAULT;
		};

		dev_drv->wait_fs = win_data->wait_fs;
		rockchip_fb_update_win_config(info, win_data);

		if (copy_to_user((struct rockchip_fb_win_cfg_data __user *)arg,
				 win_data, sizeof(*win_data))) {
			kfree(win_data);
			return -EFAULT;
		}
		kfree(win_data);
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
static int rockchip_fb_alloc_buffer_by_ion(struct fb_info *fbi,
				       unsigned long fb_mem_size)
{
	struct rockchip_fb *sfb_info = platform_get_drvdata(fb_pdev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct ion_handle *handle;
	ion_phys_addr_t phy_addr;
	size_t len;
	int ret = 0;

	if (IS_ERR_OR_NULL(sfb_info->ion_client)) {
		pr_err("%s: ion client is err or null\n", __func__);
		return -ENODEV;
	}

	handle = ion_alloc(sfb_info->ion_client, (size_t)fb_mem_size, 0,
			   ION_HEAP(ION_HEAP_TYPE_DMA), 0);
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
		if (rockchip_fb_alloc_buffer_by_ion(fbi, fb_mem_size) < 0)
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
			if (rockchip_fb_alloc_buffer_by_ion(
						fbi, fb_mem_size) < 0)
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
	mutex_init(&dev_drv->win_cfg_lock);
	mutex_init(&dev_drv->cfg_lock);
	mutex_init(&dev_drv->regs_lock);
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

		if (i == 0) {	/* only alloc memory for main fb */
			ret = rockchip_fb_alloc_buffer(fbi, 0);
			if (ret < 0) {
				dev_err(&fb_pdev->dev,
					"%s fb%d alloc buffer fail!\n",
					__func__, sfb_info->num_fb);
				return ret;
			}
		}

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
		dev_drv->ops->cfg_done(dev_drv);
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
		dev_err(&pdev->dev, "kmalloc for rockchip fb fail!");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, sfb_info);
	fb_pdev = pdev;

	if (!of_property_read_u32(np, "rockchip,disp-mode", &mode)) {
		sfb_info->disp_mode = mode;
	} else {
		dev_err(&pdev->dev, "no disp-mode node found! Set NO_DUAL mode\n");
		sfb_info->disp_mode = NO_DUAL;
	}

	if (!of_property_read_u32(np, "rockchip,uboot-logo-on", &uboot_logo_on))
		pr_info("uboot-logo-on:%d\n", uboot_logo_on);

	dev_set_name(&pdev->dev, "rockchip-fb");
#if defined(CONFIG_ION_XGOLD)
	sfb_info->ion_client = xgold_ion_client_create("rockchip-fb");
	if (IS_ERR_OR_NULL(sfb_info->ion_client)) {
		dev_err(&pdev->dev,
			"failed to create ion client for rockchip fb");
		return PTR_ERR(sfb_info->ion_client);
	}
#endif

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

struct platform_driver rockchip_fb_driver = {
	.probe = rockchip_fb_probe,
	.remove = rockchip_fb_remove,
	.driver = {
		   .name = "rockchip-fb",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rockchip_fb_dt_ids),
		   },
	.shutdown = rockchip_fb_shutdown,
};
