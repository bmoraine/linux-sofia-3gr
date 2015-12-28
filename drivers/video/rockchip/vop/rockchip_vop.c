/*
 * rockchip VOP(Video Output Processer) hardware driver.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/div64.h>
#include <linux/uaccess.h>
#include "rockchip_vop.h"

#if defined(CONFIG_ROCKCHIP_IOMMU)
#include <linux/rockchip_iovmm.h>
#include "../rockchip_disp_drv.h"
#endif

#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/platform_device_pm.h>
#endif
#include <linux/xgold_noc.h>

#define NOC_VOP_NAME "LCDC"

static int dbg_thresd;
module_param(dbg_thresd, int, S_IRUGO | S_IWUSR);

#define DBG(level, x...) do {			\
	if (unlikely(dbg_thresd >= level))	\
		pr_info(x); }			\
	while (0)

static struct rockchip_vop_win vop_win[] = {
	[0] = {
		.name = "win0",
		.id = 0,
		.support_3d = false,
	},
	[1] = {
		.name = "win1",
		.id = 1,
		.support_3d = false,
	},
	[2] = {
		.name = "hwc",
		.id = 2,
		.support_3d = false,
	},
};

static void rockchip_vop_clk_enable(struct vop_device *vop_dev)
{
	if (!vop_dev->clk_on) {
#ifdef CONFIG_PLATFORM_DEVICE_PM
		device_state_pm_set_state_by_name(vop_dev->dev,
				vop_dev->pm_platdata->pm_state_D0_name);
#else
		clk_prepare_enable(vop_dev->dclk);
#endif
		xgold_noc_qos_set(NOC_VOP_NAME);
		vop_dev->clk_on = true;
	}
}

static void rockchip_vop_clk_disable(struct vop_device *vop_dev)
{
	if (vop_dev->clk_on) {
#ifdef CONFIG_PLATFORM_DEVICE_PM
		device_state_pm_set_state_by_name(vop_dev->dev,
			vop_dev->pm_platdata->pm_state_D3_name);
#else
		clk_disable_unprepare(vop_dev->dclk);
#endif
		vop_dev->clk_on = false;
	}
}

static irqreturn_t rockchip_vop_isr(int irq, void *dev_id)
{
	struct vop_device *vop_dev = (struct vop_device *)dev_id;
	ktime_t timestamp = ktime_get();
	u32 int_reg = vop_readl(vop_dev, VOP_INT_STATUS);
	u32 irq_active = 0;

	irq_active = int_reg & INT_STA_MSK;
	if (irq_active)
		vop_writel(vop_dev, VOP_INT_STATUS,
			   int_reg | (irq_active << INT_CLR_SHIFT));

	if (int_reg & M_FS_INT_STA) {
		timestamp = ktime_get();

		if (0/* vop_dev->driver.wait_fs */) {
			spin_lock(&(vop_dev->driver.cpl_lock));
			complete(&(vop_dev->driver.frame_done));
			spin_unlock(&(vop_dev->driver.cpl_lock));
		}
		vop_dev->driver.vsync_info.timestamp = timestamp;
		wake_up_interruptible_all(&vop_dev->driver.vsync_info.wait);
	}

	if (int_reg & M_HS_INT_STA) {
		spin_lock(&vop_dev->driver.cpl_lock);
		complete(&vop_dev->driver.frame_done);
		spin_unlock(&vop_dev->driver.cpl_lock);
	}

#ifdef VOP_IRQ_EMPTY_DEBUG
	if (int_reg & M_WIN0_EMPTY_INT_STA) {
		vop_msk_reg(vop_dev, VOP_INT_STATUS, M_WIN0_EMPTY_INT_CLEAR,
			    V_WIN0_EMPTY_INT_CLEAR(1));
		dev_info(vop_dev->dev, "win0 empty irq\n");
	} else if (int_reg & M_WIN1_EMPTY_INT_STA) {
		vop_msk_reg(vop_dev, VOP_INT_STATUS, M_WIN1_EMPTY_INT_CLEAR,
			    V_WIN1_EMPTY_INT_CLEAR(1));
		dev_info(vop_dev->dev, "win1 empty irq\n");
	}
#endif

	return IRQ_HANDLED;
}

static int rockchip_vop_enable_irq(struct rockchip_vop_driver *dev_drv)
{
	u32 mask, val;
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on)) {
		mask = M_FS_INT_CLEAR | M_FS_INT_EN |
		    M_LF_INT_CLEAR | M_LF_INT_EN |
		    M_HS_INT_CLEAR | M_HS_INT_EN |
		    M_BUS_ERR_INT_CLEAR | M_BUS_ERR_INT_EN;
		val = V_FS_INT_CLEAR(1) | V_FS_INT_EN(1) |
		    V_LF_INT_CLEAR(1) | V_LF_INT_EN(1) |
		    V_HS_INT_CLEAR(1) | V_HS_INT_EN(1) |
		    V_BUS_ERR_INT_CLEAR(1) | V_BUS_ERR_INT_EN(0);

#ifdef VOP_IRQ_EMPTY_DEBUG
		mask |= M_WIN0_EMPTY_INT_EN | M_WIN1_EMPTY_INT_EN;
		val |= V_WIN0_EMPTY_INT_EN(1) | V_WIN1_EMPTY_INT_EN(1);
#endif

		vop_msk_reg(vop_dev, VOP_INT_STATUS, mask, val);
		spin_unlock(&vop_dev->reg_lock);
	} else {
		spin_unlock(&vop_dev->reg_lock);
	}

	return 0;
}

static int rockchip_vop_disable_irq(struct vop_device *vop_dev)
{
	u32 mask, val;

	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on)) {
		mask = M_FS_INT_CLEAR | M_FS_INT_EN |
		    M_LF_INT_CLEAR | M_LF_INT_EN |
		    M_HS_INT_CLEAR | M_HS_INT_EN |
		    M_BUS_ERR_INT_CLEAR | M_BUS_ERR_INT_EN;
		val = V_FS_INT_CLEAR(0) | V_FS_INT_EN(0) |
		    V_LF_INT_CLEAR(0) | V_LF_INT_EN(0) |
		    V_HS_INT_CLEAR(0) | V_HS_INT_EN(0) |
		    V_BUS_ERR_INT_CLEAR(0) | V_BUS_ERR_INT_EN(0);
#ifdef VOP_IRQ_EMPTY_DEBUG
		mask |= M_WIN0_EMPTY_INT_EN | M_WIN1_EMPTY_INT_EN;
		val |= V_WIN0_EMPTY_INT_EN(0) | V_WIN1_EMPTY_INT_EN(0);
#endif

		vop_msk_reg(vop_dev, VOP_INT_STATUS, mask, val);
		spin_unlock(&vop_dev->reg_lock);
	} else {
		spin_unlock(&vop_dev->reg_lock);
	}
	mdelay(1);
	return 0;
}

static void vop_read_reg_default_cfg(struct vop_device *vop_dev)
{
	int reg = 0;
	u32 val = 0;
	struct rockchip_vop_win *win0 = vop_dev->driver.win[0];
	struct rockchip_vop_win *win1 = vop_dev->driver.win[1];

	spin_lock(&vop_dev->reg_lock);
	for (reg = 0; reg <= VOP_MIPI_EDPI_CTRL; reg += 4) {
		val = vop_readl_bak(vop_dev, reg);
		if (reg == VOP_WIN0_ACT_INFO) {
			win0->area[0].xact = (val & M_ACT_WIDTH) + 1;
			win0->area[0].yact = ((val & M_ACT_HEIGHT) >> 16) + 1;
		}

		if (reg == VOP_WIN1_ACT_INFO) {
			win1->area[0].xact = (val & M_DSP_WIDTH) + 1;
			win1->area[0].yact = ((val & M_DSP_HEIGHT) >> 16) + 1;
		}
	}
	spin_unlock(&vop_dev->reg_lock);
}

static int rockchip_vop_alpha_cfg(struct vop_device *vop_dev)
{
	int win0_top = 0;
	u32 mask, val;
	struct rockchip_vop_driver *vop_drv = &vop_dev->driver;
	enum data_format win0_format = vop_drv->win[0]->area[0].format;
	enum data_format win1_format = vop_drv->win[1]->area[0].format;

	int win0_alpha_en = ((win0_format == ARGB888) ||
			     (win0_format == ABGR888)) ? 1 : 0;
	int win1_alpha_en = ((win1_format == ARGB888) ||
			     (win1_format == ABGR888)) ? 1 : 0;
	int atv_layer_cnt = vop_drv->win[0]->state + vop_drv->win[1]->state;
	u32 *_pv = (u32 *)vop_dev->regsbak;

	_pv += (VOP_DSP_CTRL0 >> 2);
	win0_top = ((*_pv) & (M_WIN0_TOP)) >> 8;
	if (win0_top && (atv_layer_cnt >= 2) && (win0_alpha_en)) {
		mask = M_WIN0_ALPHA_EN | M_WIN1_ALPHA_EN;
		val = V_WIN0_ALPHA_EN(1) | V_WIN1_ALPHA_EN(0);
		vop_msk_reg(vop_dev, VOP_ALPHA_CTRL, mask, val);

		mask = M_WIN0_ALPHA_MODE |
		    M_ALPHA_MODE_SEL0 | M_ALPHA_MODE_SEL1;
		val = V_WIN0_ALPHA_MODE(1) |
		    V_ALPHA_MODE_SEL0(1) | V_ALPHA_MODE_SEL1(0);
		vop_msk_reg(vop_dev, VOP_DSP_CTRL0, mask, val);
		/* this vop bg layer not support yuv domain overlay,so bg val
		   have to set 0x800a80 equeal to 0x000000 at rgb domian,after
		   android start we recover to 0x00000 */
		mask = M_BG_COLOR;
		val = V_BG_COLOR(0x000000);
		vop_msk_reg(vop_dev, VOP_DSP_CTRL1, mask, val);
	} else if ((!win0_top) && (atv_layer_cnt >= 2) && (win1_alpha_en)) {
		mask = M_WIN0_ALPHA_EN | M_WIN1_ALPHA_EN;
		val = V_WIN0_ALPHA_EN(0) | V_WIN1_ALPHA_EN(1);
		vop_msk_reg(vop_dev, VOP_ALPHA_CTRL, mask, val);

		mask = M_WIN1_ALPHA_MODE |
		    M_ALPHA_MODE_SEL0 | M_ALPHA_MODE_SEL1;
		if (vop_dev->driver.overlay_mode == VOP_YUV_DOMAIN)
			val = V_WIN0_ALPHA_MODE(1) |
			    V_ALPHA_MODE_SEL0(0) | V_ALPHA_MODE_SEL1(0);
		else
			val = V_WIN1_ALPHA_MODE(1) |
			    V_ALPHA_MODE_SEL0(1) | V_ALPHA_MODE_SEL1(0);
		vop_msk_reg(vop_dev, VOP_DSP_CTRL0, mask, val);
		/*this vop bg layer not support yuv domain overlay,so bg val
		   have to set 0x800a80 equeal to 0x000000 at rgb domian,after
		   android start we recover to 0x00000 */
		mask = M_BG_COLOR;
		val = V_BG_COLOR(0x000000);
		vop_msk_reg(vop_dev, VOP_DSP_CTRL1, mask, val);
	} else {
		mask = M_WIN0_ALPHA_EN | M_WIN1_ALPHA_EN;
		val = V_WIN0_ALPHA_EN(0) | V_WIN1_ALPHA_EN(0);
		vop_msk_reg(vop_dev, VOP_ALPHA_CTRL, mask, val);
	}

	if (vop_dev->driver.win[2]->state == 1) {
		mask = M_HWC_ALPAH_EN;
		val = V_HWC_ALPAH_EN(1);
		vop_msk_reg(vop_dev, VOP_ALPHA_CTRL, mask, val);

		mask = M_HWC_ALPHA_MODE;
		val = V_HWC_ALPHA_MODE(1);
		vop_msk_reg(vop_dev, VOP_DSP_CTRL0, mask, val);
	} else {
		mask = M_HWC_ALPAH_EN;
		val = V_HWC_ALPAH_EN(0);
		vop_msk_reg(vop_dev, VOP_ALPHA_CTRL, mask, val);
	}

	return 0;
}

static int rockchip_vop_axi_gather_cfg(struct vop_device *vop_dev,
				       struct rockchip_vop_win *win)
{
	u16 yrgb_gather_num = 8;
	u16 cbcr_gather_num = 2;

	switch (win->area[0].format) {
	case ARGB888:
	case XBGR888:
	case ABGR888:
		yrgb_gather_num = 8;
		break;
	case RGB888:
	case RGB565:
		yrgb_gather_num = 4;
		break;
	case YUV444:
	case YUV422:
	case YUV420:
	case YUV420_NV21:
		yrgb_gather_num = 2;
		cbcr_gather_num = 4;
		break;
	default:
		dev_err(vop_dev->driver.dev, "%s:un supported format!\n",
			__func__);
		return -EINVAL;
	}

	if (win->id == 0)
		vop_msk_reg(vop_dev, VOP_DMA_GATHER,
			    M_WIN0_YRGB_AXI_GATHER_EN |
			    M_WIN0_YRGB_AXI_GATHER_NUM |
			    M_WIN0_CBCR_AXI_GATHER_NUM,
			    V_WIN0_YRGB_AXI_GATHER_EN(1) |
			    V_WIN0_YRGB_AXI_GATHER_NUM(yrgb_gather_num) |
			    V_WIN0_CBCR_AXI_GATHER_NUM(cbcr_gather_num));
	else if (win->id == 1)
		vop_msk_reg(vop_dev, VOP_DMA_GATHER,
			    M_WIN1_AXI_GATHER_EN |
			    M_WIN1_AXI_GATHER_NUM,
			    V_WIN1_AXI_GATHER_EN(1) |
			    V_WIN1_AXI_GATHER_NUM(yrgb_gather_num));

	return 0;
}

static void vop_win_csc_mode(struct vop_device *vop_dev,
			     struct rockchip_vop_win *win)
{
	struct rockchip_vop_driver *dev_drv = &vop_dev->driver;
	struct rockchip_screen *screen = dev_drv->cur_screen;

	if (dev_drv->overlay_mode == VOP_YUV_DOMAIN) {
		switch (win->area[0].fmt_cfg) {
		case VOP_FORMAT_ARGB888:
		case VOP_FORMAT_RGB888:
		case VOP_FORMAT_RGB565:
			if ((screen->mode.xres < 1280) &&
			    (screen->mode.yres < 720)) {
				win->csc_mode = VOP_R2Y_CSC_BT601;
			} else {
				win->csc_mode = VOP_R2Y_CSC_BT709;
			}
			break;
		default:
			break;
		}
		if (win->id == 0) {
			vop_msk_reg(vop_dev, VOP_DSP_CTRL0, M_WIN0_CSC_MODE,
				    V_WIN0_CSC_MODE(win->csc_mode));
		} else if (win->id == 1) {
			vop_msk_reg(vop_dev, VOP_DSP_CTRL0, M_WIN1_CSC_MODE,
				    V_WIN1_CSC_MODE(win->csc_mode));
		}
	} else if (dev_drv->overlay_mode == VOP_RGB_DOMAIN) {
		switch (win->area[0].fmt_cfg) {
		case VOP_FORMAT_YCBCR420:
			if (win->id == 0) {
				win->csc_mode = VOP_Y2R_CSC_MPEG;
				vop_msk_reg(vop_dev, VOP_DSP_CTRL0,
					    M_WIN0_CSC_MODE,
					    V_WIN0_CSC_MODE(win->csc_mode));
			}
			break;
		default:
			break;
		}
	}
}

static void vop_win_update_regs(struct vop_device *vop_dev,
				struct rockchip_vop_win *win)
{
	u32 mask, val;
	int hwc_size;

	if (win->state == 1) {
		vop_win_csc_mode(vop_dev, win);

		if (win->id == 0) {
			mask = M_WIN0_EN | M_WIN0_FORMAT | M_WIN0_RB_SWAP |
				M_WIN0_UV_SWAP;
			val = V_WIN0_EN(win->state) |
			    V_WIN0_FORMAT(win->area[0].fmt_cfg) |
			    V_WIN0_RB_SWAP(win->area[0].swap_rb) |
			    V_WIN0_UV_SWAP(win->area[0].swap_uv);
			vop_msk_reg(vop_dev, VOP_SYS_CTRL, mask, val);
			vop_writel(vop_dev, VOP_WIN0_SCL_FACTOR_YRGB,
				   V_X_SCL_FACTOR(win->scale_yrgb_x) |
				   V_Y_SCL_FACTOR(win->scale_yrgb_y));
			vop_writel(vop_dev, VOP_WIN0_SCL_FACTOR_CBR,
				   V_X_SCL_FACTOR(win->scale_cbcr_x) |
				   V_Y_SCL_FACTOR(win->scale_cbcr_y));

			vop_msk_reg(vop_dev, VOP_WIN0_VIR,
				    M_YRGB_VIR | M_CBBR_VIR,
				    V_YRGB_VIR(win->area[0].y_vir_stride) |
				    V_CBCR_VIR(win->area[0].uv_vir_stride));
			vop_writel(vop_dev, VOP_WIN0_ACT_INFO,
				   V_ACT_WIDTH(win->area[0].xact) |
				   V_ACT_HEIGHT(win->area[0].yact));
			vop_writel(vop_dev, VOP_WIN0_DSP_ST,
				   V_DSP_STX(win->area[0].dsp_stx) |
				   V_DSP_STY(win->area[0].dsp_sty));
			vop_writel(vop_dev, VOP_WIN0_DSP_INFO,
				   V_DSP_WIDTH(win->area[0].xsize) |
				   V_DSP_HEIGHT(win->area[0].ysize));

			vop_writel(vop_dev, VOP_WIN0_YRGB_MST,
				   win->area[0].y_addr);
			vop_writel(vop_dev, VOP_WIN0_CBR_MST,
				   win->area[0].uv_addr);
		} else if (win->id == 1) {
			mask = M_WIN1_EN | M_WIN1_FORMAT | M_WIN1_RB_SWAP;
			val = V_WIN1_EN(win->state) |
			    V_WIN1_FORMAT(win->area[0].fmt_cfg) |
			    V_WIN1_RB_SWAP(win->area[0].swap_rb);
			vop_msk_reg(vop_dev, VOP_SYS_CTRL, mask, val);

			/* this vop unsupport win1 scale
			 * so xsize==xact and ysize==yact */
			win->area[0].xsize = win->area[0].xact;
			win->area[0].ysize = win->area[0].yact;
			vop_writel(vop_dev, VOP_WIN1_DSP_INFO,
				   V_DSP_WIDTH(win->area[0].xsize) |
				   V_DSP_HEIGHT(win->area[0].ysize));
			vop_writel(vop_dev, VOP_WIN1_DSP_ST,
				   V_DSP_STX(win->area[0].dsp_stx) |
				   V_DSP_STY(win->area[0].dsp_sty));

			vop_writel(vop_dev, VOP_WIN1_MST, win->area[0].y_addr);

			vop_msk_reg(vop_dev, VOP_WIN1_VIR, M_YRGB_VIR,
				    V_YRGB_VIR(win->area[0].y_vir_stride));

		} else if (win->id == 2) {
			mask = M_HWC_EN | M_HWC_LODAD_EN;
			val = V_HWC_EN(win->state) | V_HWC_LODAD_EN(1);
			vop_msk_reg(vop_dev, VOP_SYS_CTRL, mask, val);
			if ((win->area[0].xsize == 32) &&
			    (win->area[0].ysize == 32))
				hwc_size = 0;
			else if ((win->area[0].xsize == 64) &&
				 (win->area[0].ysize == 64))
				hwc_size = 1;
			else
				dev_err(vop_dev->dev,
					"unsupport hwc size:x=%d,y=%d\n",
					win->area[0].xsize, win->area[0].ysize);
			vop_writel(vop_dev, VOP_HWC_DSP_ST,
				   V_DSP_STX(win->area[0].dsp_stx) |
				   V_DSP_STY(win->area[0].dsp_sty));

			vop_writel(vop_dev, VOP_HWC_MST, win->area[0].y_addr);
		}
	} else {
		win->area[0].y_addr = 0;
		win->area[0].uv_addr = 0;
		if (win->id == 0)
			vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_WIN0_EN,
				    V_WIN0_EN(0));
		else if (win->id == 1)
			vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_WIN1_EN,
				    V_WIN1_EN(0));
		else if (win->id == 2)
			vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_HWC_EN,
				    V_HWC_EN(0));
	}
	rockchip_vop_alpha_cfg(vop_dev);
	rockchip_vop_axi_gather_cfg(vop_dev, win);
}

static void vop_win_enable(struct vop_device *vop_dev, unsigned int win_id,
			   bool open)
{
	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on) &&
	    vop_dev->driver.win[win_id]->state != open) {
		if (open) {
			if (!vop_dev->atv_layer_cnt) {
				dev_info(vop_dev->dev,
					 "wakeup from standby!\n");
				vop_dev->standby = 0;
			}
			vop_dev->atv_layer_cnt++;
		} else if ((vop_dev->atv_layer_cnt > 0) && (!open)) {
			vop_dev->atv_layer_cnt--;
		}
		vop_dev->driver.win[win_id]->state = open;
		if (!open) {
			vop_win_update_regs(vop_dev,
					    vop_dev->driver.win[win_id]);
			vop_cfg_done(vop_dev);
		}
		/* if no layer used,disable lcdc */
		if (!vop_dev->atv_layer_cnt) {
			dev_info(vop_dev->dev,
				 "no layer is used,go to standby!\n");
			vop_dev->standby = 1;
		}
	}
	spin_unlock(&vop_dev->reg_lock);
}

static int rockchip_vop_direct_set_win_addr(
	struct rockchip_vop_driver *dev_drv, int win_id, u32 addr)
{
	struct vop_device *vop_dev =
		container_of(dev_drv, struct vop_device, driver);

	if (win_id == 0) {
		spin_lock(&vop_dev->reg_lock);
		vop_writel(vop_dev, VOP_WIN0_YRGB_MST, addr);
		vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_WIN0_EN, V_WIN0_EN(1));
		vop_cfg_done(vop_dev);
		spin_unlock(&vop_dev->reg_lock);
	} else if (win_id == 1) {
		spin_lock(&vop_dev->reg_lock);
		vop_writel(vop_dev, VOP_WIN1_MST, addr);
		vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_WIN1_EN, V_WIN1_EN(1));
		vop_cfg_done(vop_dev);
		spin_unlock(&vop_dev->reg_lock);
	}

	return 0;
}

static int rockchip_vop_reg_update(struct rockchip_vop_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rockchip_vop_win *win0 = vop_dev->driver.win[0];
	struct rockchip_vop_win *win1 = vop_dev->driver.win[1];
	int timeout;
	unsigned long flags;

	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on)) {
		vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_LCDC_STANDBY,
			    V_LCDC_STANDBY(vop_dev->standby));
		vop_win_update_regs(vop_dev, win0);
		vop_win_update_regs(vop_dev, win1);
		rockchip_vop_alpha_cfg(vop_dev);
		vop_cfg_done(vop_dev);
	}
	spin_unlock(&vop_dev->reg_lock);

	if (0/* dev_drv->wait_fs */) {
		spin_lock_irqsave(&dev_drv->cpl_lock, flags);
		init_completion(&dev_drv->frame_done);
		spin_unlock_irqrestore(&dev_drv->cpl_lock, flags);
		timeout = wait_for_completion_timeout(&dev_drv->frame_done,
						      msecs_to_jiffies
						      (dev_drv->cur_screen->ft +
						       5));
		if (!timeout && (!dev_drv->frame_done.done)) {
			dev_warn(vop_dev->dev,
				 "wait for new frame start time out!\n");
			return -ETIMEDOUT;
		}
	}
	DBG(2, "%s for vop%d\n", __func__, vop_dev->id);
	return 0;
}

static void rockchip_vop_reg_restore(struct vop_device *vop_dev)
{
	memcpy((u8 *)vop_dev->regs, (u8 *)vop_dev->regsbak, 0xfc);
}

static int rockchip_vop_mmu_en(struct rockchip_vop_driver *dev_drv, bool enable)
{
	u32 mask, val;
	struct vop_device *vop_dev =
		container_of(dev_drv, struct vop_device, driver);

	/*spin_lock(&vop_dev->reg_lock);*/
	if (likely(vop_dev->clk_on)) {
		mask = M_MMU_EN | M_AXI_MAX_OUTSTANDING_EN |
			M_AXI_OUTSTANDING_MAX_NUM;
		val = V_MMU_EN(enable) | V_AXI_OUTSTANDING_MAX_NUM(31) |
			V_AXI_MAX_OUTSTANDING_EN(1);
		vop_msk_reg(vop_dev, VOP_BUS_INTF_CTRL, mask, val);
	}
	/*spin_unlock(&vop_dev->reg_lock);*/

#if defined(CONFIG_ROCKCHIP_IOMMU)
	if (dev_drv->iommu_enabled) {
		if (!dev_drv->mmu_dev)
			return -ENODEV;
		if (enable && !vop_dev->iommu_status) {
			vop_dev->iommu_status = 1;
			rockchip_iovmm_activate(dev_drv->dev);
		}
		if (!enable && vop_dev->iommu_status) {
			vop_dev->iommu_status = 0;
			rockchip_iovmm_deactivate(dev_drv->dev);
		}
	}
#endif

	return 0;
}

static int rockchip_vop_set_hwc_lut(struct rockchip_vop_driver *dev_drv,
				    u32 *hwc_lut, int mode)
{
	int i = 0;
	int __iomem *c;
	int v;
	int len = 256 * 4;
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	if (dev_drv->hwc_lut == NULL) {
		dev_drv->hwc_lut = devm_kzalloc(vop_dev->dev, len, GFP_KERNEL);
		if (!dev_drv->hwc_lut) {
			dev_warn(vop_dev->dev,
				 "Can't allocate hwc_lut!\n");
			return -ENOMEM;
		}
	}

	spin_lock(&vop_dev->reg_lock);
	vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_HWC_LUT_EN, V_HWC_LUT_EN(0));
	vop_cfg_done(vop_dev);
	mdelay(25);
	for (i = 0; i < 256; i++) {
		if (mode == 1)
			dev_drv->hwc_lut[i] = hwc_lut[i];
		v = dev_drv->hwc_lut[i];
		c = vop_dev->hwc_lut_addr_base + i;
		writel(v, c);
	}
	vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_HWC_LUT_EN, V_HWC_LUT_EN(1));
	vop_cfg_done(vop_dev);
	spin_unlock(&vop_dev->reg_lock);

	return 0;
}

static int rockchip_vop_set_lut(struct rockchip_vop_driver *dev_drv,
				u32 *dsp_lut, int mode)
{
	int i = 0;
	int __iomem *c;
	int v;
	size_t len = 256 * 4;
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	if (!dsp_lut)
		return 0;

	if (mode == 1 && !dev_drv->cur_screen->dsp_lut) {
		dev_drv->cur_screen->dsp_lut =
			devm_kzalloc(vop_dev->dev, len, GFP_KERNEL);
		if (!dev_drv->cur_screen->dsp_lut)
			return -ENOMEM;
	}

	spin_lock(&vop_dev->reg_lock);
	vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_DSP_LUT_EN, V_DSP_LUT_EN(0));
	vop_cfg_done(vop_dev);
	mdelay(25);
	for (i = 0; i < 256; i++) {
		if (mode == 1)
			dev_drv->cur_screen->dsp_lut[i] = dsp_lut[i];

		v = dsp_lut[i];
		c = vop_dev->dsp_lut_addr_base + i;
		writel(v, c);
	}
	vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_DSP_LUT_EN, V_DSP_LUT_EN(1));
	vop_cfg_done(vop_dev);
	spin_unlock(&vop_dev->reg_lock);
	return 0;
}

static int rockchip_vop_set_dclk(struct rockchip_vop_driver *dev_drv)
{
	int fps;
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rockchip_screen *screen = dev_drv->cur_screen;
	int ret = 0;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	vop_dev->pixclock =
		div_u64(1000000000000llu, screen->mode.pixclock);

	/*
	 * Mipi screen needs real dclk > screen->mode.pixclock,
	 * otherwise the mipi controller will miss the fifo, and get error.
	 */
	if (screen->type == SCREEN_MIPI)
		ret = device_state_pm_set_state_by_name(vop_dev->dev,
							"ultra_high_perf");
	else if (screen->mode.pixclock == 148500000)
		ret = device_state_pm_set_state_by_name(vop_dev->dev,
							"high_perf");
	else if (screen->mode.pixclock == 74250000)
		ret = device_state_pm_set_state_by_name(vop_dev->dev,
							"mid_perf");
	else if (screen->mode.pixclock == 27000000)
		ret = device_state_pm_set_state_by_name(vop_dev->dev,
							"low_perf");
	else
		ret = device_state_pm_set_state_by_name(vop_dev->dev,
							"ultra_high_perf");
	if (ret)
		dev_err(dev_drv->dev, "set vop%d dclk failed\n", vop_dev->id);
#else
	ret = clk_set_rate(vop_dev->dclk, screen->mode.pixclock);
	if (ret)
		dev_err(dev_drv->dev, "set vop%d dclk failed\n", vop_dev->id);
	vop_dev->pixclock =
	    div_u64(1000000000000llu, clk_get_rate(vop_dev->dclk));
#endif
	vop_dev->driver.pixclock = vop_dev->pixclock;

	fps = rockchip_fb_calc_fps(screen, vop_dev->pixclock);
	screen->ft = 1000 / fps;
	dev_info(vop_dev->dev, "%s: dclk:%lu>>fps:%d ",
		 vop_dev->driver.name, clk_get_rate(vop_dev->dclk), fps);
	return 0;
}

static int rockchip_vop_standby(struct rockchip_vop_driver *dev_drv,
				bool enable)
{
	struct vop_device *vop_dev =
		container_of(dev_drv, struct vop_device, driver);
	int timeout;
	unsigned long flags;

	if (unlikely(!vop_dev->clk_on))
		return 0;

	if (!enable) {
		spin_lock(&vop_dev->reg_lock);
		/* Recovery EDPI halt en */
		if (dev_drv->cur_screen->type == SCREEN_MIPI) {
			vop_msk_reg(vop_dev, VOP_MIPI_EDPI_CTRL,
				    M_EDPI_HALT_EN, V_EDPI_HALT_EN(1));
			vop_cfg_done(vop_dev);
		}

		vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_LCDC_STANDBY,
			    V_LCDC_STANDBY(0));
		spin_unlock(&vop_dev->reg_lock);
	} else {
		spin_lock_irqsave(&dev_drv->cpl_lock, flags);
		reinit_completion(&dev_drv->frame_done);
		spin_unlock_irqrestore(&dev_drv->cpl_lock, flags);

		spin_lock(&vop_dev->reg_lock);
		/* Disable EDPI halt to avoid vop standby time out */
		if (dev_drv->cur_screen->type == SCREEN_MIPI) {
			vop_msk_reg(vop_dev, VOP_MIPI_EDPI_CTRL,
				    M_EDPI_HALT_EN, V_EDPI_HALT_EN(0));
			vop_cfg_done(vop_dev);
		}

		vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_LCDC_STANDBY,
			    V_LCDC_STANDBY(1));
		spin_unlock(&vop_dev->reg_lock);

		/* wait for standby hold valid */
		timeout = wait_for_completion_timeout(&dev_drv->frame_done,
						      msecs_to_jiffies(25));

		if (!timeout && (!dev_drv->frame_done.done)) {
			dev_info(dev_drv->dev,
				 "wait for standby hold valid start time out!\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int rockchip_vop_pre_init(struct rockchip_vop_driver *dev_drv)
{
	struct vop_device *vop_dev = container_of(dev_drv,
						  struct vop_device, driver);
	struct rockchip_screen *screen = dev_drv->cur_screen;

	if (vop_dev->pre_init)
		return 0;

#if defined(CONFIG_ROCKCHIP_IOMMU)
	if (dev_drv->iommu_enabled) {
		dev_drv->mmu_dev =
			rockchip_disp_get_sysmmu_device(dev_drv->mmu_dts_name);
		if (dev_drv->mmu_dev)
			rockchip_disp_platform_set_sysmmu(dev_drv->mmu_dev,
							   dev_drv->dev);
		else
			dev_err(dev_drv->dev,
				"failed to get rockchip iommu device\n");
	}
#endif

	vop_dev->dclk = devm_clk_get(vop_dev->dev, "dclk_vop");

	if (IS_ERR(vop_dev->dclk)) {
		dev_err(vop_dev->dev, "failed to get vop%d clk source\n",
			vop_dev->id);
	}

	rockchip_disp_pwr_enable(screen);
	rockchip_vop_clk_enable(vop_dev);

	/* backup reg config at uboot */
	vop_read_reg_default_cfg(vop_dev);

	/* config qos */
	vop_msk_reg(vop_dev, VOP_BUS_INTF_CTRL,
		    M_NOC_QOS_EN | M_NOC_QOS_VALUE,
		    V_NOC_QOS_EN(1) | V_NOC_QOS_VALUE(3));

	/* config for the FRC mode of dither down */
	vop_writel(vop_dev, VOP_FRC_LOWER01_0, 0x12844821);
	vop_writel(vop_dev, VOP_FRC_LOWER01_1, 0x21488412);
	vop_writel(vop_dev, VOP_FRC_LOWER10_0, 0x55aaaa55);
	vop_writel(vop_dev, VOP_FRC_LOWER10_1, 0x55aaaa55);
	vop_writel(vop_dev, VOP_FRC_LOWER11_0, 0xdeb77deb);
	vop_writel(vop_dev, VOP_FRC_LOWER11_1, 0xed7bb7de);

	vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_AUTO_GATING_EN,
		    V_AUTO_GATING_EN(0));
	vop_cfg_done(vop_dev);

	/*      TODO Daisen wait to Modify at 2014.11.12
	   if ((dev_drv->ops->open_bcsh) &&
	   (dev_drv->output_color == COLOR_YCBCR)) {
	   if (support_loader_display())
	   dev_drv->bcsh_init_status = 1;
	   else
	   dev_drv->ops->open_bcsh(dev_drv, 1);
	   }
	 */
	vop_dev->pre_init = true;

	return 0;
}

static void rockchip_vop_deinit(struct vop_device *vop_dev)
{
	struct rockchip_vop_driver *dev_drv = &vop_dev->driver;

	rockchip_vop_standby(dev_drv, true);
	rockchip_vop_disable_irq(vop_dev);
	rockchip_vop_mmu_en(dev_drv, false);
	/* rockchip_vop_clk_disable(vop_dev); */
}

static void rockchip_vop_select_bcsh(struct rockchip_vop_driver *dev_drv,
				     struct vop_device *vop_dev)
{
	if (dev_drv->overlay_mode == VOP_YUV_DOMAIN) {
		if (dev_drv->output_color == COLOR_YCBCR)	/* bypass */
			vop_msk_reg(vop_dev, VOP_BCSH_CTRL,
				    M_BCSH_Y2R_EN | M_BCSH_R2Y_EN,
				    V_BCSH_Y2R_EN(0) | V_BCSH_R2Y_EN(0));
		else		/* YUV2RGB */
			vop_msk_reg(vop_dev, VOP_BCSH_CTRL,
				    M_BCSH_Y2R_EN | M_BCSH_Y2R_CSC_MODE |
				    M_BCSH_R2Y_EN,
				    V_BCSH_Y2R_EN(1) |
				    V_BCSH_Y2R_CSC_MODE(VOP_Y2R_CSC_MPEG) |
				    V_BCSH_R2Y_EN(0));
	} else {		/* overlay_mode=VOP_RGB_DOMAIN */
		if (dev_drv->output_color == COLOR_RGB)	/* bypass */
			vop_msk_reg(vop_dev, VOP_BCSH_CTRL,
				    M_BCSH_R2Y_EN | M_BCSH_Y2R_EN,
				    V_BCSH_R2Y_EN(1) | V_BCSH_Y2R_EN(1));
		else		/* RGB2YUV */
			vop_msk_reg(vop_dev, VOP_BCSH_CTRL,
				    M_BCSH_R2Y_EN |
				    M_BCSH_R2Y_CSC_MODE | M_BCSH_Y2R_EN,
				    V_BCSH_R2Y_EN(1) |
				    V_BCSH_R2Y_CSC_MODE(VOP_Y2R_CSC_MPEG) |
				    V_BCSH_Y2R_EN(0));
	}
}

static int rockchip_vop_load_screen(struct rockchip_vop_driver *dev_drv,
				    bool initscreen)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rockchip_screen *screen = dev_drv->cur_screen;
	u32 mask = 0, val = 0;
	u16 out_format = 0;
	u16 right_margin = screen->mode.right_margin;
	u16 left_margin = screen->mode.left_margin;
	u16 lower_margin = screen->mode.lower_margin;
	u16 upper_margin = screen->mode.upper_margin;
	u16 x_res = screen->mode.xres;
	u16 y_res = screen->mode.yres;
	u16 hsync_len = screen->mode.hsync_len;
	u16 vsync_len = screen->mode.vsync_len;
	u16 vtotal = 0;
	u16 vsync_st_f1 = 0, vsync_end_f1 = 0;
	u16 vact_st_f1 = 0, vact_end_f1 = 0;
	u16 line_num = 0;

	if (unlikely(!vop_dev->clk_on)) {
		dev_err(vop_dev->dev, "%s: vop is standby!\n", __func__);
		return 0;
	}

	spin_lock(&vop_dev->reg_lock);
	vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_LCDC_STANDBY,
		    V_LCDC_STANDBY(1));
	/* Select output color domain */
	/*dev_drv->output_color = screen->color_mode;
	   if (dev_drv->output_color == COLOR_YCBCR)
	   dev_drv->overlay_mode = VOP_YUV_DOMAIN;
	   else
	   dev_drv->overlay_mode = VOP_RGB_DOMAIN;
	 */
	dev_drv->overlay_mode = VOP_RGB_DOMAIN;
	vop_msk_reg(vop_dev, VOP_DSP_CTRL0, M_SW_OVERLAY_MODE,
		    V_SW_OVERLAY_MODE(dev_drv->overlay_mode));

	mask = M_RGB_DCLK_EN | M_LVDS_DCLK_EN | M_MIPI_DCLK_EN;
	switch (screen->type) {
	case SCREEN_RGB:
		mask |= M_RGB_DCLK_INVERT;
		val = V_RGB_DCLK_EN(1) | V_RGB_DCLK_INVERT(0);
		vop_msk_reg(vop_dev, VOP_BUS_INTF_CTRL, mask, val);
		break;
	case SCREEN_LVDS:
		mask |= M_LVDS_DCLK_INVERT;
		val = V_LVDS_DCLK_EN(1) | V_LVDS_DCLK_INVERT(0);
		vop_msk_reg(vop_dev, VOP_BUS_INTF_CTRL, mask, val);
		break;
	case SCREEN_MIPI:
		mask |= M_MIPI_DCLK_INVERT;
		val = V_MIPI_DCLK_EN(1) | V_MIPI_DCLK_INVERT(0);
		vop_msk_reg(vop_dev, VOP_BUS_INTF_CTRL, mask, val);
		vop_msk_reg(vop_dev, VOP_MIPI_EDPI_CTRL,
			    M_EDPI_HALT_EN, V_EDPI_HALT_EN(1));
		break;
	case SCREEN_HDMI:
		mask |= M_RGB_DCLK_INVERT;
		val = V_RGB_DCLK_EN(1) | V_RGB_DCLK_INVERT(0);
		vop_msk_reg(vop_dev, VOP_BUS_INTF_CTRL, mask, val);
		rockchip_vop_select_bcsh(dev_drv, vop_dev);
		break;
	default:
		dev_err(vop_dev->dev, "un supported interface!\n");
		break;
	}

	switch (screen->face) {
	case OUT_P565:
		out_format = OUT_P565;
		mask = M_DITHER_DOWN_EN |
		    M_DITHER_DOWN_MODE | M_DITHER_DOWN_SEL;
		val = V_DITHER_DOWN_EN(1) |
		    V_DITHER_DOWN_MODE(0) | V_DITHER_DOWN_SEL(1);
		break;
	case OUT_P666:
		out_format = OUT_P666;
		mask = M_DITHER_DOWN_EN |
		    M_DITHER_DOWN_MODE | M_DITHER_DOWN_SEL;
		val = V_DITHER_DOWN_EN(1) |
		    V_DITHER_DOWN_MODE(1) | V_DITHER_DOWN_SEL(1);
		break;
	case OUT_D888_P565:
		out_format = OUT_P888;
		mask = M_DITHER_DOWN_EN |
		    M_DITHER_DOWN_MODE | M_DITHER_DOWN_SEL;
		val = V_DITHER_DOWN_EN(1) |
		    V_DITHER_DOWN_MODE(0) | V_DITHER_DOWN_SEL(1);
		break;
	case OUT_D888_P666:
		out_format = OUT_P888;
		mask = M_DITHER_DOWN_EN |
		    M_DITHER_DOWN_MODE | M_DITHER_DOWN_SEL;
		val = V_DITHER_DOWN_EN(1) |
		    V_DITHER_DOWN_MODE(1) | V_DITHER_DOWN_SEL(1);
		break;
	case OUT_P888:
		out_format = OUT_P888;
		mask = M_DITHER_DOWN_EN | M_DITHER_UP_EN;
		val = V_DITHER_DOWN_EN(0) | V_DITHER_UP_EN(0);
		break;
	default:
		dev_err(vop_dev->dev, "un supported interface!\n");
		break;
	}

	mask |= M_DSP_OUT_FORMAT;
	val |= V_DSP_OUT_FORMAT(out_format);
	vop_msk_reg(vop_dev, VOP_DSP_CTRL0, mask, val);

	mask = M_HSYNC_POL | M_VSYNC_POL | M_DEN_POL | M_DCLK_POL;
	val = V_HSYNC_POL(screen->pin_hsync) |
	    V_VSYNC_POL(screen->pin_vsync) |
	    V_DEN_POL(screen->pin_den) | V_DCLK_POL(screen->pin_dclk);
	vop_msk_reg(vop_dev, VOP_DSP_CTRL0, mask, val);

	mask = M_BG_COLOR | M_DSP_BG_SWAP | M_DSP_RB_SWAP |
	    M_DSP_RG_SWAP | M_BLANK_EN | M_BLACK_EN;
	val = V_BG_COLOR(0x000000) | V_DSP_BG_SWAP(screen->swap_gb) |
	    V_DSP_RB_SWAP(screen->swap_rb) |
	    V_DSP_RG_SWAP(screen->swap_rg) | V_BLANK_EN(0) | V_BLACK_EN(0);
	vop_msk_reg(vop_dev, VOP_DSP_CTRL1, mask, val);

	/* config timing */
	vop_writel(vop_dev, VOP_DSP_HTOTAL_HS_END,
		   V_HSYNC(hsync_len) |
		   V_HORPRD(hsync_len + left_margin + x_res + right_margin));
	vop_writel(vop_dev, VOP_DSP_HACT_ST_END,
		   V_HAEP(hsync_len + left_margin + x_res) |
		   V_HASP(hsync_len + left_margin));

	if (screen->mode.vmode == FB_VMODE_INTERLACED) {
		vtotal = 2 * (vsync_len + upper_margin + lower_margin) +
		    y_res + 1;

		/* First Field Timing */
		vop_writel(vop_dev, VOP_DSP_VTOTAL_VS_END,
			   V_VSYNC(vsync_len) | V_VERPRD(vtotal));
		vop_writel(vop_dev, VOP_DSP_VACT_ST_END,
			   V_VAEP(vsync_len + upper_margin + y_res / 2) |
			   V_VASP(vsync_len + upper_margin));

		/* Second Field Timing */
		vsync_st_f1 = vsync_len + upper_margin + y_res / 2 +
		    lower_margin;
		vsync_end_f1 = 2 * vsync_len + upper_margin + y_res / 2 +
		    lower_margin;
		vact_st_f1 = 2 * (vsync_len + upper_margin) + y_res / 2 +
		    lower_margin + 1;
		vact_end_f1 = 2 * (vsync_len + upper_margin) + y_res +
		    lower_margin + 1;
		line_num = vsync_len + upper_margin + y_res / 2;
		vop_writel(vop_dev, VOP_DSP_VS_ST_END_F1,
			   V_VSYNC_ST_F1(vsync_st_f1) |
			   V_VSYNC_END_F1(vsync_end_f1));
		vop_writel(vop_dev, VOP_DSP_VACT_ST_END_F1,
			   V_VAEP(vact_end_f1) | V_VASP(vact_st_f1));

		vop_msk_reg(vop_dev, VOP_DSP_CTRL0,
			    M_INTERLACE_DSP_EN |
			    M_WIN0_YRGB_DEFLICK_EN |
			    M_WIN0_CBR_DEFLICK_EN |
			    M_INTERLACE_FIELD_POL |
			    M_WIN0_INTERLACE_EN |
			    M_WIN1_INTERLACE_EN,
			    V_INTERLACE_DSP_EN(1) |
			    V_WIN0_YRGB_DEFLICK_EN(1) |
			    V_WIN0_CBR_DEFLICK_EN(1) |
			    V_INTERLACE_FIELD_POL(0) |
			    V_WIN0_INTERLACE_EN(1) | V_WIN1_INTERLACE_EN(1));
	} else {
		vtotal = vsync_len + upper_margin + y_res + lower_margin;
		line_num = vsync_len + upper_margin + y_res;
		vop_writel(vop_dev, VOP_DSP_VTOTAL_VS_END,
			   V_VSYNC(vsync_len) | V_VERPRD(vtotal));
		vop_writel(vop_dev, VOP_DSP_VACT_ST_END,
			   V_VAEP(vsync_len + upper_margin + y_res) |
			   V_VASP(vsync_len + upper_margin));

		vop_msk_reg(vop_dev, VOP_DSP_CTRL0,
			    M_INTERLACE_DSP_EN |
			    M_WIN0_YRGB_DEFLICK_EN |
			    M_WIN0_CBR_DEFLICK_EN |
			    M_INTERLACE_FIELD_POL |
			    M_WIN0_INTERLACE_EN |
			    M_WIN1_INTERLACE_EN,
			    V_INTERLACE_DSP_EN(0) |
			    V_WIN0_YRGB_DEFLICK_EN(0) |
			    V_WIN0_CBR_DEFLICK_EN(0) |
			    V_INTERLACE_FIELD_POL(0) |
			    V_WIN0_INTERLACE_EN(0) |
			    V_WIN1_INTERLACE_EN(0));
	}

	vop_msk_reg(vop_dev, VOP_INT_STATUS,
		    M_LF_INT_NUM, V_LF_INT_NUM(line_num));
	spin_unlock(&vop_dev->reg_lock);

	rockchip_vop_set_dclk(dev_drv);

	if (screen->index >= 0 &&
	    dev_drv->trsm_ops && dev_drv->trsm_ops->enable)
		dev_drv->trsm_ops->enable();

	if (screen->init)
		screen->init();

	spin_lock(&vop_dev->reg_lock);
	vop_msk_reg(vop_dev, VOP_SYS_CTRL, M_LCDC_STANDBY,
		    V_LCDC_STANDBY(0));
	spin_unlock(&vop_dev->reg_lock);
	return 0;
}

static int rockchip_vop_open(struct rockchip_vop_driver *dev_drv, int win_id,
			     bool open)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	/* enable clk,when first layer open */
	if ((open) && (!vop_dev->atv_layer_cnt)) {
		rockchip_vop_pre_init(dev_drv);
		rockchip_vop_clk_enable(vop_dev);
		rockchip_vop_reg_restore(vop_dev);

		if (support_loader_display()) {
			rockchip_vop_set_dclk(dev_drv);
			rockchip_vop_enable_irq(dev_drv);
		} else {
			rockchip_vop_mmu_en(dev_drv, open);
			if (dev_drv->trsm_ops &&
			    dev_drv->trsm_ops->detect_panel) {
				dev_drv->cur_screen->index =
					dev_drv->trsm_ops->detect_panel();
				if (dev_drv->cur_screen->index >= 0) {
					rockchip_set_prmry_screen(
							dev_drv->cur_screen);
					rockchip_get_prmry_screen(
							dev_drv->cur_screen);
				}
			}
			rockchip_vop_load_screen(dev_drv, 1);
		}

		/* set screen lut */
		if (dev_drv->cur_screen->dsp_lut)
			rockchip_vop_set_lut(dev_drv,
				dev_drv->cur_screen->dsp_lut, 0);
	}

	if (win_id < ARRAY_SIZE(vop_win))
		vop_win_enable(vop_dev, win_id, open);
	else
		dev_err(vop_dev->dev, "invalid win id:%d\n", win_id);

	/* when all layer closed,disable clk */
	if ((!open) && (!vop_dev->atv_layer_cnt))
		rockchip_vop_deinit(vop_dev);

	return 0;
}

static int rockchip_vop_set_par(struct rockchip_vop_driver *dev_drv, int win_id)
{
	struct vop_device *vop_dev = container_of(dev_drv,
						  struct vop_device, driver);
	struct rockchip_screen *screen = dev_drv->cur_screen;
	struct rockchip_vop_win *win = NULL;
	char fmt[9] = "NULL";

	if (!screen) {
		dev_err(dev_drv->dev, "screen is null!\n");
		return -ENOENT;
	}

	if (win_id == 0) {
		win = dev_drv->win[0];
	} else if (win_id == 1) {
		win = dev_drv->win[1];
	} else if (win_id == 2) {
		win = dev_drv->win[2];
	} else {
		dev_err(dev_drv->dev, "un supported win number:%d\n", win_id);
		return -EINVAL;
	}

	spin_lock(&vop_dev->reg_lock);
	win->area[0].dsp_stx = win->area[0].xpos + screen->mode.left_margin +
	    screen->mode.hsync_len;
	if (screen->mode.vmode == FB_VMODE_INTERLACED) {
		win->area[0].ysize /= 2;
		win->area[0].dsp_sty = win->area[0].ypos / 2 +
		    screen->mode.upper_margin + screen->mode.vsync_len;
	} else {
		win->area[0].dsp_sty = win->area[0].ypos +
		    screen->mode.upper_margin + screen->mode.vsync_len;
	}

	if (win->area[0].xsize == 1 || win->area[0].ysize == 1) {
		dev_err(dev_drv->dev, "unsupported win number (x, y)=(%d, %d)\n",
				win->area[0].xsize, win->area[0].ysize);
	}

	win->scale_yrgb_x = CALSCALE(win->area[0].xact, win->area[0].xsize);
	win->scale_yrgb_y = CALSCALE(win->area[0].yact, win->area[0].ysize);

	switch (win->area[0].format) {
	case ARGB888:
		win->area[0].fmt_cfg = VOP_FORMAT_ARGB888;
		win->area[0].swap_rb = 0;
		break;
	case XBGR888:
		win->area[0].fmt_cfg = VOP_FORMAT_ARGB888;
		win->area[0].swap_rb = 1;
		break;
	case ABGR888:
		win->area[0].fmt_cfg = VOP_FORMAT_ARGB888;
		win->area[0].swap_rb = 1;
		break;
	case RGB888:
		win->area[0].fmt_cfg = VOP_FORMAT_RGB888;
		win->area[0].swap_rb = 0;
		break;
	case RGB565:
		win->area[0].fmt_cfg = VOP_FORMAT_RGB565;
		win->area[0].swap_rb = 0;
		break;
	case YUV444:
		if (win_id == 0) {
			win->area[0].fmt_cfg = VOP_FORMAT_YCBCR444;
			win->scale_cbcr_x =
				CALSCALE(win->area[0].xact, win->area[0].xsize);
			win->scale_cbcr_y =
				CALSCALE(win->area[0].yact, win->area[0].ysize);
			win->area[0].swap_rb = 0;
			win->area[0].swap_uv = 0;
		} else {
			dev_err(vop_dev->driver.dev,
				"%s:un supported format!\n", __func__);
		}
		break;
	case YUV422:
		if (win_id == 0) {
			win->area[0].fmt_cfg = VOP_FORMAT_YCBCR422;
			win->scale_cbcr_x = CALSCALE((win->area[0].xact / 2),
						     win->area[0].xsize);
			win->scale_cbcr_y = CALSCALE(win->area[0].yact,
						     win->area[0].ysize);
			win->area[0].swap_rb = 0;
			win->area[0].swap_uv = 0;
		} else {
			dev_err(vop_dev->driver.dev,
				"%s:un supported format!\n", __func__);
		}
		break;
	case YUV420:
		if (win_id == 0) {
			win->area[0].fmt_cfg = VOP_FORMAT_YCBCR420;
			win->scale_cbcr_x = CALSCALE(win->area[0].xact / 2,
						     win->area[0].xsize);
			win->scale_cbcr_y = CALSCALE(win->area[0].yact / 2,
						     win->area[0].ysize);
			win->area[0].swap_rb = 0;
			win->area[0].swap_uv = 0;
		} else {
			dev_err(vop_dev->driver.dev,
				"%s:un supported format!\n", __func__);
		}
		break;
	case YUV420_NV21:
		if (win_id == 0) {
			win->area[0].fmt_cfg = VOP_FORMAT_YCBCR420;
			win->scale_cbcr_x = CALSCALE(win->area[0].xact / 2,
						     win->area[0].xsize);
			win->scale_cbcr_y = CALSCALE(win->area[0].yact / 2,
						     win->area[0].ysize);
			win->area[0].swap_rb = 0;
			win->area[0].swap_uv = 1;
		} else {
			dev_err(vop_dev->driver.dev,
				"%s:un supported format!\n", __func__);
		}
		break;
	default:
		dev_err(vop_dev->driver.dev, "%s:un supported format!\n",
			__func__);
		break;
	}
	spin_unlock(&vop_dev->reg_lock);

	DBG(1,
	    "lcdc%d>>%s\n>>format:%s>>>xact:%d>>yact:%d>>xsize:%d>>ysize:%d\n"
	    ">>xvir:%d>>yvir:%d>>xpos:%d>>ypos:%d>>\n",
	    vop_dev->id, __func__, get_format_string(win->area[0].format, fmt),
	    win->area[0].xact, win->area[0].yact,
	    win->area[0].xsize, win->area[0].ysize,
	    win->area[0].xvir, win->area[0].yvir,
	    win->area[0].xpos, win->area[0].ypos);
	return 0;
}

static int rockchip_vop_pan_display(struct rockchip_vop_driver *dev_drv,
				    int win_id)
{
	struct vop_device *vop_dev = container_of(dev_drv,
						  struct vop_device, driver);
	struct rockchip_vop_win *win = NULL;
	struct rockchip_screen *screen = dev_drv->cur_screen;

	if (!screen) {
		dev_err(dev_drv->dev, "screen is null!\n");
		return -ENOENT;
	}

	if (win_id == 0) {
		win = dev_drv->win[0];
	} else if (win_id == 1) {
		win = dev_drv->win[1];
	} else if (win_id == 2) {
		win = dev_drv->win[2];
	} else {
		dev_err(dev_drv->dev, "invalid win number:%d!\n", win_id);
		return -EINVAL;
	}

	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on)) {
		win->area[0].y_addr =
			win->area[0].smem_start + win->area[0].y_offset;
		win->area[0].uv_addr =
			win->area[0].cbr_start + win->area[0].c_offset;
		if ((win->area[0].uv_addr == 0) &&
		    (win->area[0].format == YUV420 ||
		     win->area[0].format == YUV420_NV21))
			pr_err("error:uv_addr=0x%x,format=%d\n",
			       win->area[0].uv_addr, win->area[0].format);
		if (win->area[0].y_addr)
			vop_win_update_regs(vop_dev, win);
		/* vop_cfg_done(vop_dev); */
	}
	spin_unlock(&vop_dev->reg_lock);

	DBG(2, "lcdc%d>>%s:y_addr:0x%x>>uv_addr:0x%x>>offset:%d\n",
	    vop_dev->id, __func__, win->area[0].y_addr, win->area[0].uv_addr,
	    win->area[0].y_offset);
	/* this is the first frame of the system,enable frame start interrupt */
	if ((dev_drv->first_frame)) {
		dev_drv->first_frame = 0;
		rockchip_vop_enable_irq(dev_drv);
	}

	return 0;
}

static int rockchip_vop_ioctl(struct rockchip_vop_driver *dev_drv,
			      unsigned int cmd, unsigned long arg, int win_id)
{
	struct vop_device *vop_dev = container_of(dev_drv,
						  struct vop_device, driver);
	u32 panel_size[2];
	void __user *argp = (void __user *)arg;
	struct color_key_cfg clr_key_cfg;

	switch (cmd) {
	case SFA_FBIOGET_PANEL_SIZE:
		panel_size[0] = vop_dev->screen->mode.xres;
		panel_size[1] = vop_dev->screen->mode.yres;
		if (copy_to_user(argp, panel_size, 8))
			return -EFAULT;
		break;
	case SFA_FBIOPUT_COLOR_KEY_CFG:
		if (copy_from_user(&clr_key_cfg, argp,
				   sizeof(struct color_key_cfg)))
			return -EFAULT;
		vop_writel(vop_dev, VOP_WIN0_COLOR_KEY,
			   clr_key_cfg.win0_color_key_cfg);
		vop_writel(vop_dev, VOP_WIN1_COLOR_KEY,
			   clr_key_cfg.win1_color_key_cfg);
		break;

	default:
		break;
	}
	return 0;
}

static int rockchip_vop_get_win_id(struct rockchip_vop_driver *dev_drv,
				   const char *id)
{
	int win_id = 0;

	mutex_lock(&dev_drv->fb_win_id_mutex);
	if (!strcmp(id, "fb0"))
		win_id = dev_drv->fb0_win_id;
	else if (!strcmp(id, "fb1"))
		win_id = dev_drv->fb1_win_id;
	else if (!strcmp(id, "fb2"))
		win_id = dev_drv->fb2_win_id;
	mutex_unlock(&dev_drv->fb_win_id_mutex);

	return win_id;
}

static int rockchip_vop_get_win_state(struct rockchip_vop_driver *dev_drv,
				      int win_id)
{
	struct vop_device *vop_dev =
		container_of(dev_drv, struct vop_device, driver);

	if (win_id == 0)
		return vop_read_bit(vop_dev, VOP_SYS_CTRL, M_WIN0_EN);
	else if (win_id == 1)
		return vop_read_bit(vop_dev, VOP_SYS_CTRL, M_WIN1_EN);
	else if (win_id == 2)
		return vop_read_bit(vop_dev, VOP_SYS_CTRL, M_HWC_EN);
	return -1;
}

static int rockchip_vop_ovl_mgr(struct rockchip_vop_driver *dev_drv, int swap,
				bool set)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	int ovl;

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		if (set) {
			vop_msk_reg(vop_dev, VOP_DSP_CTRL0, M_WIN0_TOP,
				    V_WIN0_TOP(swap));
			ovl = swap;
		} else {
			ovl = vop_read_bit(vop_dev, VOP_DSP_CTRL0, M_WIN0_TOP);
		}
	} else {
		ovl = -EPERM;
	}
	spin_unlock(&vop_dev->reg_lock);

	return ovl;
}

static int rockchip_vop_early_suspend(struct rockchip_vop_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rockchip_screen *screen = dev_drv->cur_screen;

	if (dev_drv->suspend_flag)
		return 0;

	dev_drv->suspend_flag = true;
	flush_kthread_worker(&dev_drv->update_regs_worker);

	if (screen->index >= 0 &&
	    dev_drv->trsm_ops && dev_drv->trsm_ops->disable)
		dev_drv->trsm_ops->disable();
	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on)) {
		vop_msk_reg(vop_dev, VOP_DSP_CTRL1, M_BLANK_EN, V_BLANK_EN(1));
		vop_msk_reg(vop_dev, VOP_INT_STATUS,
			    M_FS_INT_CLEAR | M_LF_INT_CLEAR,
			    V_FS_INT_CLEAR(1) | V_LF_INT_CLEAR(1));
		vop_msk_reg(vop_dev, VOP_DSP_CTRL1, M_DSP_OUT_ZERO,
			    V_DSP_OUT_ZERO(1));
		vop_cfg_done(vop_dev);

		spin_unlock(&vop_dev->reg_lock);
	} else {
		spin_unlock(&vop_dev->reg_lock);
		return 0;
	}

	rockchip_vop_standby(dev_drv, true);
	rockchip_vop_mmu_en(dev_drv, false);
	rockchip_vop_clk_disable(vop_dev);
	rockchip_disp_pwr_disable(screen);
	return 0;
}

static int rockchip_vop_early_resume(struct rockchip_vop_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	if (!dev_drv->suspend_flag)
		return 0;

	rockchip_disp_pwr_enable(dev_drv->cur_screen);

	if (vop_dev->atv_layer_cnt) {
		rockchip_vop_clk_enable(vop_dev);
		rockchip_vop_reg_restore(vop_dev);
		rockchip_vop_mmu_en(dev_drv, true);

		/* config for the FRC mode of dither down */
		if (dev_drv->cur_screen &&
		    dev_drv->cur_screen->face != OUT_P888) {
			vop_writel(vop_dev, VOP_FRC_LOWER01_0, 0x12844821);
			vop_writel(vop_dev, VOP_FRC_LOWER01_1, 0x21488412);
			vop_writel(vop_dev, VOP_FRC_LOWER10_0, 0x55aaaa55);
			vop_writel(vop_dev, VOP_FRC_LOWER10_1, 0x55aaaa55);
			vop_writel(vop_dev, VOP_FRC_LOWER11_0, 0xdeb77deb);
			vop_writel(vop_dev, VOP_FRC_LOWER11_1, 0xed7bb7de);
		}

		/* set screen lut */
		if (dev_drv->cur_screen && dev_drv->cur_screen->dsp_lut)
			rockchip_vop_set_lut(dev_drv,
				dev_drv->cur_screen->dsp_lut, 0);
		/*set hwc lut */
		rockchip_vop_set_hwc_lut(dev_drv, dev_drv->hwc_lut, 0);

		spin_lock(&vop_dev->reg_lock);

		vop_msk_reg(vop_dev, VOP_DSP_CTRL1, M_DSP_OUT_ZERO,
			    V_DSP_OUT_ZERO(0));
		vop_msk_reg(vop_dev, VOP_DSP_CTRL1, M_BLANK_EN, V_BLANK_EN(0));
		vop_cfg_done(vop_dev);

		spin_unlock(&vop_dev->reg_lock);
	}

	dev_drv->suspend_flag = false;

	if (dev_drv->cur_screen &&
			dev_drv->cur_screen->index >= 0 &&
			dev_drv->trsm_ops &&
			dev_drv->trsm_ops->enable)
		dev_drv->trsm_ops->enable();

	/* VOP leave standby mode after DSI enable */
	rockchip_vop_standby(dev_drv, false);
	return 0;
}

static int rockchip_vop_blank(struct rockchip_vop_driver *dev_drv,
			      int win_id, int blank_mode)
{
	#if 0
	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		rockchip_vop_early_resume(dev_drv);
		break;
	case FB_BLANK_NORMAL:
		rockchip_vop_early_suspend(dev_drv);
		break;
	default:
		rockchip_vop_early_suspend(dev_drv);
		break;
	}
	#endif

	dev_info(dev_drv->dev, "blank mode:%d\n", blank_mode);

	return 0;
}

static int rockchip_vop_cfg_done(struct rockchip_vop_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	int i;
	unsigned int mask, val;
	struct rockchip_vop_win *win = NULL;

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		for (i = 0; i < ARRAY_SIZE(vop_win); i++) {
			win = dev_drv->win[i];
			if ((win->state == 0) && (win->last_state == 1)) {
				switch (win->id) {
				case 0:
					mask = M_WIN0_EN;
					val = V_WIN0_EN(0);
					vop_msk_reg(vop_dev, VOP_SYS_CTRL,
						    mask, val);
					break;
				case 1:
					mask = M_WIN1_EN;
					val = V_WIN1_EN(0);
					vop_msk_reg(vop_dev, VOP_SYS_CTRL,
						    mask, val);
					break;
				case 2:
					mask = M_HWC_EN;
					val = V_HWC_EN(0);
					vop_msk_reg(vop_dev, VOP_SYS_CTRL,
						    mask, val);
					break;
				default:
					break;
				}
			}
			win->last_state = win->state;
		}
		vop_cfg_done(vop_dev);
	}
	spin_unlock(&vop_dev->reg_lock);
	return 0;
}

/*
	a:[-30~0]:
	    sin_hue = sin(a)*256 +0x100;
	    cos_hue = cos(a)*256;
	a:[0~30]
	    sin_hue = sin(a)*256;
	    cos_hue = cos(a)*256;
*/
static int rockchip_vop_get_bcsh_hue(struct rockchip_vop_driver *dev_drv,
				     enum bcsh_hue_mode mode)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u32 val = 0;

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		val = vop_readl(vop_dev, VOP_BCSH_H);
		switch (mode) {
		case H_SIN:
			val &= M_BCSH_SIN_HUE;
			break;
		case H_COS:
			val &= M_BCSH_COS_HUE;
			val >>= 16;
			break;
		default:
			break;
		}
	}
	spin_unlock(&vop_dev->reg_lock);

	return val;
}

static int rockchip_vop_set_bcsh_hue(struct rockchip_vop_driver *dev_drv,
				     int sin_hue, int cos_hue)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u32 mask, val;

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		mask = M_BCSH_SIN_HUE | M_BCSH_COS_HUE;
		val = V_BCSH_SIN_HUE(sin_hue) | V_BCSH_COS_HUE(cos_hue);
		vop_msk_reg(vop_dev, VOP_BCSH_H, mask, val);
		vop_cfg_done(vop_dev);
	}
	spin_unlock(&vop_dev->reg_lock);

	return 0;
}

static int rockchip_vop_set_bcsh_bcs(struct rockchip_vop_driver *dev_drv,
				     enum bcsh_bcs_mode mode, int value)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u32 mask = 0, val = 0;

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		switch (mode) {
		case BRIGHTNESS:
			/* from 0 to 255,typical is 128 */
			if (value < 0x80)
				value += 0x80;
			else if (value >= 0x80)
				value = value - 0x80;
			mask = M_BCSH_BRIGHTNESS;
			val = V_BCSH_BRIGHTNESS(value);
			break;
		case CONTRAST:
			/* from 0 to 510,typical is 256 */
			mask = M_BCSH_CONTRAST;
			val = V_BCSH_CONTRAST(value);
			break;
		case SAT_CON:
			/* from 0 to 1015,typical is 256 */
			mask = M_BCSH_SAT_CON;
			val = V_BCSH_SAT_CON(value);
			break;
		default:
			break;
		}
		vop_msk_reg(vop_dev, VOP_BCSH_BCS, mask, val);
		vop_cfg_done(vop_dev);
	}
	spin_unlock(&vop_dev->reg_lock);
	return val;
}

static int rockchip_vop_get_bcsh_bcs(struct rockchip_vop_driver *dev_drv,
				     enum bcsh_bcs_mode mode)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u32 val = 0;

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		val = vop_readl(vop_dev, VOP_BCSH_BCS);
		switch (mode) {
		case BRIGHTNESS:
			val &= M_BCSH_BRIGHTNESS;
			if (val > 0x80)
				val -= 0x80;
			else
				val += 0x80;
			break;
		case CONTRAST:
			val &= M_BCSH_CONTRAST;
			val >>= 8;
			break;
		case SAT_CON:
			val &= M_BCSH_SAT_CON;
			val >>= 20;
			break;
		default:
			break;
		}
	}
	spin_unlock(&vop_dev->reg_lock);
	return val;
}

static int
rockchip_vop_open_bcsh(struct rockchip_vop_driver *dev_drv, bool open)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u32 mask, val;

	if (dev_drv->bcsh_init_status && open) {
		dev_drv->bcsh_init_status = 0;
		return 0;
	}
	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		rockchip_vop_select_bcsh(dev_drv, vop_dev);
		if (open) {
			vop_msk_reg(vop_dev,
				    VOP_BCSH_CTRL, M_BCSH_EN | M_BCSH_OUT_MODE,
				    V_BCSH_EN(1) | V_BCSH_OUT_MODE(3));
			vop_writel(vop_dev, VOP_BCSH_BCS,
				   V_BCSH_BRIGHTNESS(0x00) |
				   V_BCSH_CONTRAST(0x80) |
				   V_BCSH_SAT_CON(0x80));
			vop_writel(vop_dev, VOP_BCSH_H, V_BCSH_COS_HUE(0x80));
		} else {
			mask = M_BCSH_EN;
			val = V_BCSH_EN(0);
			vop_msk_reg(vop_dev, VOP_BCSH_CTRL, mask, val);
		}
		vop_cfg_done(vop_dev);
	}

	spin_unlock(&vop_dev->reg_lock);
	return 0;
}

static int rockchip_fb_win_remap(struct rockchip_vop_driver *dev_drv, u16 order)
{
	int fb2_win_id, fb1_win_id, fb0_win_id;

	mutex_lock(&dev_drv->fb_win_id_mutex);
	if (order == FB_DEFAULT_ORDER)
		order = FB0_WIN0_FB1_WIN1_FB2_WIN2;

	fb2_win_id = order / 100;
	fb1_win_id = (order / 10) % 10;
	fb0_win_id = order % 10;

	dev_drv->fb0_win_id = fb0_win_id;
	dev_drv->fb1_win_id = fb1_win_id;
	dev_drv->fb2_win_id = fb2_win_id;

	mutex_unlock(&dev_drv->fb_win_id_mutex);

	return 0;
}

static int rockchip_vop_fps_mgr(struct rockchip_vop_driver *dev_drv, int fps,
				bool set)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rockchip_screen *screen = dev_drv->cur_screen;
	u64 ft = 0;
	u32 dotclk;
	int ret;
	u32 pixclock;
	u32 x_total, y_total;

	if (set) {
		ft = div_u64(1000000000000llu, fps);
		x_total =
		    screen->mode.upper_margin + screen->mode.lower_margin +
		    screen->mode.yres + screen->mode.vsync_len;
		y_total =
		    screen->mode.left_margin + screen->mode.right_margin +
		    screen->mode.xres + screen->mode.hsync_len;
		dev_drv->pixclock = div_u64(ft, x_total * y_total);
		dotclk = div_u64(1000000000000llu, dev_drv->pixclock);
		ret = clk_set_rate(vop_dev->dclk, dotclk);
	}

	pixclock = div_u64(1000000000000llu,
			vop_dev->driver.cur_screen->mode.pixclock);
	dev_drv->pixclock = pixclock;
	vop_dev->pixclock = pixclock;
	fps = rockchip_fb_calc_fps(vop_dev->screen, pixclock);
	screen->ft = 1000 / fps;	/*one frame time in ms */

	if (set)
		dev_info(dev_drv->dev, "%s:dclk:%lu,fps:%d\n", __func__,
			 clk_get_rate(vop_dev->dclk), fps);

	return fps;
}

static int rockchip_vop_set_irq_to_cpu(struct rockchip_vop_driver *dev_drv,
				       int enable)
{
	struct vop_device *vop_dev = container_of(dev_drv,
						  struct vop_device, driver);
	if (enable)
		enable_irq(vop_dev->irq);
	else
		disable_irq(vop_dev->irq);
	return 0;
}

static int rockchip_vop_poll_vblank(struct rockchip_vop_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u32 int_reg;
	int ret;

	if (vop_dev->clk_on && (!dev_drv->suspend_flag)) {
		int_reg = vop_readl(vop_dev, VOP_INT_STATUS);
		if (int_reg & M_LF_INT_STA) {
			vop_msk_reg(vop_dev, VOP_INT_STATUS, M_LF_INT_CLEAR,
				    V_LF_INT_CLEAR(1));
			ret = SFA_LF_STATUS_FC;
		} else {
			ret = SFA_LF_STATUS_FR;
		}
	} else {
		ret = SFA_LF_STATUS_NC;
	}

	return ret;
}

static int rockchip_vop_get_dsp_addr(struct rockchip_vop_driver *dev_drv,
				     unsigned int *dsp_addr)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	if (vop_dev->clk_on) {
		dsp_addr[0] = vop_readl(vop_dev, VOP_WIN0_YRGB_MST);
		dsp_addr[1] = vop_readl(vop_dev, VOP_WIN1_MST);
	}
	return 0;
}

static ssize_t rockchip_vop_get_disp_info(struct rockchip_vop_driver *dev_drv,
					  char *buf, int win_id)
{
	struct vop_device *vop_dev = container_of(dev_drv, struct vop_device,
						  driver);
	char format_w0[9] = "NULL";
	char format_w1[9] = "NULL";
	char status_w0[9] = "NULL";
	char status_w1[9] = "NULL";
	u32 fmt_id, act_info, dsp_info, dsp_st, factor;
	u16 xvir_w0, x_act_w0, y_act_w0, x_dsp_w0, y_dsp_w0, x_st_w0, y_st_w0;
	u16 xvir_w1, x_act_w1, y_act_w1, x_dsp_w1, y_dsp_w1, x_st_w1, y_st_w1;
	u16 x_factor, y_factor, x_scale, y_scale;
	u16 ovl;
	u32 win1_dsp_yaddr = 0;

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		/* data format */
		fmt_id = vop_readl(vop_dev, VOP_SYS_CTRL);
		get_format_string((fmt_id & M_WIN0_FORMAT) >> 3, format_w0);
		get_format_string((fmt_id & M_WIN1_FORMAT) >> 6, format_w1);

		/* win status */
		if (fmt_id & M_WIN0_EN)
			strcpy(status_w0, "enabled");
		else
			strcpy(status_w0, "disabled");

		if ((fmt_id & M_WIN1_EN) >> 1)
			strcpy(status_w1, "enabled");
		else
			strcpy(status_w1, "disabled");

		/* ovl */
		ovl = vop_read_bit(vop_dev, VOP_DSP_CTRL0, M_WIN0_TOP);

		/* xvir */
		xvir_w0 = vop_readl(vop_dev, VOP_WIN0_VIR) & M_YRGB_VIR;
		xvir_w1 = vop_readl(vop_dev, VOP_WIN1_VIR) & M_YRGB_VIR;

		/* xact/yact */
		act_info = vop_readl(vop_dev, VOP_WIN0_ACT_INFO);
		x_act_w0 = (act_info & M_ACT_WIDTH) + 1;
		y_act_w0 = ((act_info & M_ACT_HEIGHT) >> 16) + 1;

		/* xsize/ysize */
		dsp_info = vop_readl(vop_dev, VOP_WIN0_DSP_INFO);
		x_dsp_w0 = (dsp_info & M_DSP_WIDTH) + 1;
		y_dsp_w0 = ((dsp_info & M_DSP_HEIGHT) >> 16) + 1;

		dsp_info = vop_readl(vop_dev, VOP_WIN1_DSP_INFO);
		x_dsp_w1 = (dsp_info & M_DSP_WIDTH) + 1;
		y_dsp_w1 = ((dsp_info & M_DSP_HEIGHT) >> 16) + 1;

		/* If unsupport win1 scaler
		 * so win1 act info same as dsp info */
		x_act_w1 = x_dsp_w1;
		y_act_w1 = y_dsp_w1;

		/* xpos/ypos */
		dsp_st = vop_readl(vop_dev, VOP_WIN0_DSP_ST);
		x_st_w0 = dsp_st & M_DSP_STX;
		y_st_w0 = (dsp_st & M_DSP_STY) >> 16;

		dsp_st = vop_readl(vop_dev, VOP_WIN1_DSP_ST);
		x_st_w1 = dsp_st & M_DSP_STX;
		y_st_w1 = (dsp_st & M_DSP_STY) >> 16;

		/* scale factor */
		factor = vop_readl(vop_dev, VOP_WIN0_SCL_FACTOR_YRGB);
		x_factor = factor & M_X_SCL_FACTOR;
		y_factor = (factor & M_Y_SCL_FACTOR) >> 16;
		x_scale = 4096 * 100 / x_factor;
		y_scale = 4096 * 100 / y_factor;

		/* dsp addr */
		win1_dsp_yaddr = vop_readl(vop_dev, VOP_WIN1_MST);

	} else {
		spin_unlock(&vop_dev->reg_lock);
		return -EPERM;
	}
	spin_unlock(&vop_dev->reg_lock);
	return snprintf(buf, PAGE_SIZE,
			"win0:%s\n"
			"xvir:%d\n"
			"xact:%d\n"
			"yact:%d\n"
			"xdsp:%d\n"
			"ydsp:%d\n"
			"x_st:%d\n"
			"y_st:%d\n"
			"x_scale:%d.%d\n"
			"y_scale:%d.%d\n"
			"format:%s\n"
			"YRGB buffer addr:0x%08x\n"
			"CBR buffer addr:0x%08x\n\n"
			"win1:%s\n"
			"xvir:%d\n"
			"xact:%d\n"
			"yact:%d\n"
			"xdsp:%d\n"
			"ydsp:%d\n"
			"x_st:%d\n"
			"y_st:%d\n"
			"format:%s\n"
			"YRGB buffer addr:0x%08x\n"
			"overlay:%s\n",
			status_w0,
			xvir_w0,
			x_act_w0,
			y_act_w0,
			x_dsp_w0,
			y_dsp_w0,
			x_st_w0,
			y_st_w0,
			x_scale / 100,
			x_scale % 100,
			y_scale / 100,
			y_scale % 100,
			format_w0,
			vop_readl(vop_dev, VOP_WIN0_YRGB_MST),
			vop_readl(vop_dev, VOP_WIN0_CBR_MST),
			status_w1,
			xvir_w1,
			x_act_w1,
			y_act_w1,
			x_dsp_w1,
			y_dsp_w1,
			x_st_w1,
			y_st_w1,
			format_w1,
			win1_dsp_yaddr,
			ovl ? "win0 on the top of win1\n" :
			"win1 on the top of win0\n");
}

static int rockchip_vop_reg_writel(struct rockchip_vop_driver *dev_drv,
				   u32 offset, u32 val)
{
	struct vop_device *vop_dev =
		container_of(dev_drv, struct vop_device, driver);

	writel(val, vop_dev->regs + offset);
	return 0;
}

static u32 rockchip_vop_reg_readl(struct rockchip_vop_driver *dev_drv,
				  u32 offset)
{
	struct vop_device *vop_dev =
		container_of(dev_drv, struct vop_device, driver);

	return readl(vop_dev->regs + offset);
}

static int rockchip_vop_reg_dump(struct rockchip_vop_driver *dev_drv)
{
	struct vop_device *vop_dev = container_of(dev_drv,
						  struct vop_device,
						  driver);
	int *cbase = (int *)vop_dev->regs;
	int *regsbak = (int *)vop_dev->regsbak;
	int i, j;

	pr_info("back up reg:\n");
	for (i = 0; i <= (0xDC >> 4); i++) {
		for (j = 0; j < 4; j++)
			pr_info("%08x  ", *(regsbak + i * 4 + j));
		pr_info("\n");
	}

	pr_info("lcdc reg:\n");
	for (i = 0; i <= (0xDC >> 4); i++) {
		for (j = 0; j < 4; j++)
			pr_info("reg%x: %08x  ", (i * 4 + j) * 4,
				readl(cbase + i * 4 + j));
		pr_info("\n");
	}
	return 0;
}

static struct rockchip_vop_drv_ops vop_drv_ops = {
	.open = rockchip_vop_open,
	.load_screen = rockchip_vop_load_screen,
	.set_par = rockchip_vop_set_par,
	.pan_display = rockchip_vop_pan_display,
	.direct_set_addr = rockchip_vop_direct_set_win_addr,
	.blank = rockchip_vop_blank,
	.ioctl = rockchip_vop_ioctl,
	.get_win_state = rockchip_vop_get_win_state,
	.ovl_mgr = rockchip_vop_ovl_mgr,
	.get_disp_info = rockchip_vop_get_disp_info,
	.fps_mgr = rockchip_vop_fps_mgr,
	.fb_get_win_id = rockchip_vop_get_win_id,
	.fb_win_remap = rockchip_fb_win_remap,
	.poll_vblank = rockchip_vop_poll_vblank,
	.get_dsp_addr = rockchip_vop_get_dsp_addr,
	.cfg_done = rockchip_vop_cfg_done,
	.dump_reg = rockchip_vop_reg_dump,
	.set_dsp_bcsh_hue = rockchip_vop_set_bcsh_hue,
	.set_dsp_bcsh_bcs = rockchip_vop_set_bcsh_bcs,
	.get_dsp_bcsh_hue = rockchip_vop_get_bcsh_hue,
	.get_dsp_bcsh_bcs = rockchip_vop_get_bcsh_bcs,
	.open_bcsh = rockchip_vop_open_bcsh,
	.set_dsp_lut = rockchip_vop_set_lut,
	.set_hwc_lut = rockchip_vop_set_hwc_lut,
	.set_irq_to_cpu = rockchip_vop_set_irq_to_cpu,
	.lcdc_reg_update = rockchip_vop_reg_update,
	.mmu_en = rockchip_vop_mmu_en,
	.reg_writel = rockchip_vop_reg_writel,
	.reg_readl = rockchip_vop_reg_readl,
};

#if defined(CONFIG_OF)
static const struct of_device_id rockchip_vop_dt_ids[] = {
	{ .compatible = "rockchip,vop", },
	{ },
};
#endif

static int rockchip_vop_parse_dt(struct vop_device *vop_dev)
{
	struct device_node *np = vop_dev->dev->of_node;
	u32 val;

#if defined(CONFIG_ROCKCHIP_IOMMU)
	if (of_property_read_u32(np, "rockchip,iommu-enabled", &val))
		vop_dev->driver.iommu_enabled = 0;
	else
		vop_dev->driver.iommu_enabled = val;
#else
	vop_dev->driver.iommu_enabled = 0;
#endif

	if (of_property_read_u32(np, "rockchip,fb-win-map", &val))
		vop_dev->driver.fb_win_map = FB_DEFAULT_ORDER;
	else
		vop_dev->driver.fb_win_map = val;

	return 0;
}

static int rockchip_vop_probe(struct platform_device *pdev)
{
	struct vop_device *vop_dev = NULL;
	struct rockchip_vop_driver *dev_drv;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret = -EINVAL;

	vop_dev = devm_kzalloc(dev, sizeof(struct vop_device), GFP_KERNEL);
	if (!vop_dev) {
		dev_err(&pdev->dev, "rockchip vop device kzalloc fail!\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, vop_dev);
	vop_dev->dev = dev;
	if (rockchip_vop_parse_dt(vop_dev)) {
		dev_err(vop_dev->dev, "rockchip vop parse dt failed!\n");
		goto err_parse_dt;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(vop_dev->dev,
				"rockchip vop get mem resource failed!\n");
		goto err_parse_dt;
	}
	vop_dev->reg_phy_base = res->start;
	vop_dev->len = resource_size(res);
	vop_dev->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(vop_dev->regs)) {
		ret = PTR_ERR(vop_dev->regs);
		goto err_remap_reg;
	}

	vop_dev->regsbak = devm_kzalloc(dev, vop_dev->len, GFP_KERNEL);
	if (IS_ERR(vop_dev->regsbak)) {
		dev_err(&pdev->dev,
			"rockchip vop device kmalloc regsbak fail!\n");
		ret = PTR_ERR(vop_dev->regsbak);
		goto err_remap_reg;
	}
	vop_dev->hwc_lut_addr_base = (vop_dev->regs + VOP_HWC_LUT_ADDR);
	vop_dev->dsp_lut_addr_base = (vop_dev->regs + VOP_DSP_LUT_ADDR);
	vop_dev->prop = PRMRY;
	dev_set_name(vop_dev->dev, "vop%d", vop_dev->id);
	dev_drv = &vop_dev->driver;
	dev_drv->dev = dev;
	dev_drv->prop = vop_dev->prop;
	dev_drv->id = vop_dev->id;
	dev_drv->ops = &vop_drv_ops;
	dev_drv->num_win = ARRAY_SIZE(vop_win);
	spin_lock_init(&vop_dev->reg_lock);

	vop_dev->irq = platform_get_irq(pdev, 0);
	if (vop_dev->irq < 0) {
		dev_err(&pdev->dev, "cannot find IRQ for vop%d\n", vop_dev->id);
		ret = -ENXIO;
		goto err_request_irq;
	}
	ret = devm_request_irq(dev, vop_dev->irq, rockchip_vop_isr,
			       IRQF_DISABLED | IRQF_SHARED,
			       dev_name(dev), vop_dev);
	if (ret) {
		dev_err(&pdev->dev, "cannot requeset irq %d - err %d\n",
			vop_dev->irq, ret);
		goto err_request_irq;
	}

#if defined(CONFIG_ROCKCHIP_IOMMU)
	if (dev_drv->iommu_enabled)
		strcpy(dev_drv->mmu_dts_name, VOP_IOMMU_COMPATIBLE_NAME);
#endif

#ifdef CONFIG_PLATFORM_DEVICE_PM
	vop_dev->pm_platdata = of_device_state_pm_setup(dev->of_node);
	if (IS_ERR(vop_dev->pm_platdata)) {
		dev_err(&pdev->dev, "Error during device state pm init\n");
		return -ENOMEM;
	}

	ret = device_state_pm_set_class(&pdev->dev,
				vop_dev->pm_platdata->pm_user_name);
#endif

	ret = rockchip_fb_register(dev_drv, vop_win, vop_dev->id);
	if (ret < 0) {
		dev_err(dev, "register fb for vop%d failed!\n", vop_dev->id);
		goto err_register_fb;
	}
	vop_dev->screen = dev_drv->screen0;

	dev_info(dev, "vop%d probe ok, iommu %s\n",
		 vop_dev->id, dev_drv->iommu_enabled ? "enabled" : "disabled");

	return 0;
err_register_fb:
err_request_irq:
	devm_kfree(vop_dev->dev, vop_dev->regsbak);
err_remap_reg:
err_parse_dt:
	devm_kfree(&pdev->dev, vop_dev);
	return ret;
}

#if defined(CONFIG_PM)
static int rockchip_vop_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	return 0;
}

static int rockchip_vop_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define rockchip_vop_suspend NULL
#define rockchip_vop_resume  NULL
#endif

static int rockchip_vop_remove(struct platform_device *pdev)
{
	return 0;
}

static void rockchip_vop_shutdown(struct platform_device *pdev)
{
	struct vop_device *vop_dev = platform_get_drvdata(pdev);
	struct rockchip_vop_driver *dev_drv = &vop_dev->driver;
	struct rockchip_screen *screen = dev_drv->cur_screen;

	dev_drv->suspend_flag = true;
	flush_kthread_worker(&dev_drv->update_regs_worker);
	kthread_stop(dev_drv->update_regs_thread);

	if (screen->index >= 0 &&
	    dev_drv->trsm_ops && dev_drv->trsm_ops->disable)
		dev_drv->trsm_ops->disable();

	rockchip_vop_deinit(vop_dev);
	rockchip_disp_pwr_disable(vop_dev->driver.cur_screen);
}

struct platform_driver rockchip_vop_driver = {
	.probe = rockchip_vop_probe,
	.remove = rockchip_vop_remove,
	.driver = {
		   .name = "rockchip-vop",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rockchip_vop_dt_ids),
		   },
	.suspend = rockchip_vop_suspend,
	.resume = rockchip_vop_resume,
	.shutdown = rockchip_vop_shutdown,
};
