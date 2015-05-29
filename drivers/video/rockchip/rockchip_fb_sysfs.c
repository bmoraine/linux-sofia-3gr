/*
 * drivers/video/rockchip/rockchip_fb_sysfs.c
 *	--sysfs entries for device fb
 *
 * Copyright (C) 2012-2015 Rockchip Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/fb.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <asm/div64.h>
#include <linux/rockchip_screen.h>
#include <linux/rockchip_fb.h>
#include <linux/string.h>

static ssize_t show_screen_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	struct rockchip_screen *screen = dev_drv->cur_screen;
	int fps = 0;

	u32 x = screen->mode.left_margin + screen->mode.right_margin +
	    screen->mode.xres + screen->mode.hsync_len;
	u32 y = screen->mode.upper_margin + screen->mode.lower_margin +
	    screen->mode.yres + screen->mode.vsync_len;
	u64 ft = (u64)x * y * (dev_drv->pixclock);

	if (ft > 0)
		fps = div64_u64(1000000000000llu, ft);
	return snprintf(buf, PAGE_SIZE,
			"xres:%d\nyres:%d\nfps:%d\nwidth:%dmm\nheight:%dmm\n",
			screen->mode.xres, screen->mode.yres, fps,
			screen->width, screen->height);
}

static ssize_t show_disp_info(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	int win_id = dev_drv->ops->fb_get_win_id(dev_drv, fbi->fix.id);

	if (dev_drv->ops->get_disp_info)
		return dev_drv->ops->get_disp_info(dev_drv, buf, win_id);

	return 0;
}

static void fill_buffer(void *handle, void *vaddr, int size)
{
	struct file *filp = handle;

	if (filp)
		vfs_write(filp, vaddr, size, &filp->f_pos);
}

static int dump_win(struct rockchip_fb *sfb_info,
		    struct rockchip_vop_win_area *area_data,
		    u8 data_format, int win_id, int area_id)
{
	void __iomem *vaddr = NULL;
	struct file *filp;
	mm_segment_t old_fs;
	char name[100] = {0};
	char fmt[10] = {0};
	struct ion_handle *ion_handle = area_data->ion_hdl;
	int width = area_data->xvir;
	int height = area_data->yvir;

	if (ion_handle) {
		vaddr = ion_map_kernel(sfb_info->ion_client, ion_handle);
	} else if (area_data->smem_start && area_data->smem_start != -1) {
		unsigned long start;
		unsigned int nr_pages;
		struct page **pages;
		int i = 0;

		start = area_data->smem_start;
		nr_pages = width * height * 3 / 2 / PAGE_SIZE;
		pages = kzalloc(sizeof(struct page) * nr_pages, GFP_KERNEL);
		if (pages == NULL) {
			pr_err("failed to alloc %d pages struct\n",
					nr_pages);
			return -ENOMEM;
		}
		while (i < nr_pages) {
			pages[i] = phys_to_page(start);
			start += PAGE_SIZE;
			i++;
		}
		vaddr = vmap(pages, nr_pages, VM_MAP,
			     pgprot_writecombine(PAGE_KERNEL));
		if (!vaddr) {
			pr_err("failed to vmap phy addr %lx\n",
			       area_data->smem_start);
			kfree(pages);
			return -1;
		}
	} else {
		return -1;
	}

	snprintf(name, 100, "/data/win%d_%d_%dx%d_%s.bin", win_id, area_id,
		 width, height, get_format_string(data_format, fmt));

	pr_info("dump win == > %s\n", name);

	filp = filp_open(name, O_RDWR | O_CREAT, 0x664);
	if (!filp)
		pr_info("fail to create %s\n", name);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fill_buffer(filp, vaddr, width * height * 4);

	set_fs(old_fs);

	if (ion_handle)
		ion_unmap_kernel(sfb_info->ion_client, ion_handle);
	else if (vaddr)
		vunmap(vaddr);

	filp_close(filp, NULL);

	return 0;
}

static ssize_t set_dump_info(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)

{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	struct rockchip_fb *sfb_info = dev_get_drvdata(fbi->device);
	struct rockchip_fb_reg_data *last_regs;
	struct rockchip_vop_win *win_data;
	struct rockchip_vop_win_area *area_data;
	int i, j;

	if (!sfb_info->ion_client)
		return 0;

	last_regs = devm_kzalloc(dev, sizeof(*last_regs), GFP_KERNEL);
	if (!last_regs)
		return -ENOMEM;

	mutex_lock(&dev_drv->regs_lock);

	if (!dev_drv->last_regs) {
		mutex_unlock(&dev_drv->regs_lock);
		return 0;
	}
	memcpy(last_regs, dev_drv->last_regs, sizeof(*last_regs));
	mutex_unlock(&dev_drv->regs_lock);

	for (i = 0; i < last_regs->win_num; i++) {
		for (j = 0; j < SFA_WIN_MAX_AREA; j++) {
			win_data = &last_regs->vop_win[i];
			area_data = &win_data->area[j];
			if (dump_win(sfb_info, area_data,
				     area_data->format, win_data->id, j))
				continue;
		}
	}
	devm_kfree(dev, last_regs);

	return count;
}

static ssize_t show_phys(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%lx-----0x%x\n",
			fbi->fix.smem_start, fbi->fix.smem_len);
}

static ssize_t show_virt(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%p-----0x%x\n",
			fbi->screen_base, fbi->fix.smem_len);
}

static ssize_t show_fb_state(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;

	int win_id = dev_drv->ops->fb_get_win_id(dev_drv, fbi->fix.id);
	int state = dev_drv->ops->get_win_state(dev_drv, win_id);

	return snprintf(buf, PAGE_SIZE, "%s\n", state ? "enabled" : "disabled");
}

static ssize_t show_dual_mode(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb *sfb_info = dev_get_drvdata(fbi->device);
	int mode = sfb_info->disp_mode;

	return snprintf(buf, PAGE_SIZE, "%d\n", mode);
}

static ssize_t set_fb_state(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	int win_id = dev_drv->ops->fb_get_win_id(dev_drv, fbi->fix.id);
	int state;
	int ret;

	ret = kstrtoint(buf, 0, &state);
	if (ret)
		return ret;
	dev_drv->ops->open(dev_drv, win_id, state);
	if (state) {
		dev_drv->ops->set_par(dev_drv, win_id);
		dev_drv->ops->pan_display(dev_drv, win_id);
		dev_drv->ops->cfg_done(dev_drv);
	}
	return count;
}

static ssize_t show_overlay(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	int ovl = 0;

	if (dev_drv->ops->ovl_mgr)
		ovl = dev_drv->ops->ovl_mgr(dev_drv, 0, 0);

	if (ovl < 0)
		return ovl;

	return snprintf(buf, PAGE_SIZE, "%s\n",
			ovl ? "win0 on the top of win1" :
			"win1 on the top of win0");
}

static ssize_t set_overlay(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	u32 ovl;
	int ret;

	ret = kstrtou32(buf, 0, &ovl);
	if (ret)
		return ret;
	if (dev_drv->ops->ovl_mgr)
		ret = dev_drv->ops->ovl_mgr(dev_drv, ovl, 1);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t show_fps(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	int fps = 0;

	if (dev_drv->ops->fps_mgr)
		fps = dev_drv->ops->fps_mgr(dev_drv, 0, 0);
	if (fps < 0)
		return fps;

	return snprintf(buf, PAGE_SIZE, "fps:%d\n", fps);
}

static ssize_t set_fps(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	u32 fps;
	int ret;

	ret = kstrtou32(buf, 0, &fps);
	if (ret)
		return ret;

	if (fps == 0 || fps > 60) {
		dev_info(dev, "unsupport fps value,pelase set 1~60\n");
		return count;
	}

	if (dev_drv->ops->fps_mgr)
		ret = dev_drv->ops->fps_mgr(dev_drv, fps, 1);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t show_fb_win_map(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;

	mutex_lock(&dev_drv->fb_win_id_mutex);
	ret =
	    snprintf(buf, PAGE_SIZE, "fb0:win%d\nfb1:win%d\nfb2:win%d\n",
		     dev_drv->fb0_win_id, dev_drv->fb1_win_id,
		     dev_drv->fb2_win_id);
	mutex_unlock(&dev_drv->fb_win_id_mutex);

	return ret;
}

static ssize_t set_fb_win_map(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	u32 order;
	int ret;

	ret = kstrtou32(buf, 0, &order);
	if ((order != FB0_WIN2_FB1_WIN1_FB2_WIN0) &&
	    (order != FB0_WIN1_FB1_WIN2_FB2_WIN0) &&
	    (order != FB0_WIN2_FB1_WIN0_FB2_WIN1) &&
	    (order != FB0_WIN0_FB1_WIN2_FB2_WIN1) &&
	    (order != FB0_WIN0_FB1_WIN1_FB2_WIN2) &&
	    (order != FB0_WIN1_FB1_WIN0_FB2_WIN2)) {
		dev_info(dev, "un supported map\n"
			 "you can use the following order:\n" "201:\n"
			 "fb0-win1\n" "fb1-win0\n" "fb2-win2\n" "210:\n"
			 "fb0-win0\n" "fb1-win1\n" "fb2-win2\n" "120:\n"
			 "fb0-win0\n" "fb1-win2\n" "fb2-win1\n" "102:\n"
			 "fb0-win2\n" "fb1-win0\n" "fb2-win1\n" "021:\n"
			 "fb0-win1\n" "fb1-win2\n" "fb2-win0\n" "012:\n"
			 "fb0-win2\n" "fb1-win1\n" "fb2-win0\n");
		return count;
	}

	if (dev_drv->ops->fb_win_remap)
		dev_drv->ops->fb_win_remap(dev_drv, order);

	return count;
}

static ssize_t show_hwc_lut(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int i = 0, len = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;

	if (dev_drv->hwc_lut) {
		memset(buf, 0, PAGE_SIZE);
		for (i = 0; i < 256; i++) {
			snprintf(buf + len, PAGE_SIZE - len, "0x%08x ",
				 dev_drv->hwc_lut[i]);
			if (((i + 1) % 8) == 0)
				strcat(buf, "\n");

			len = strlen(buf);
		}
		return len;
	}

	return 0;
}

static ssize_t set_hwc_lut(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	u32 *hwc_lut = NULL;
	const char *start = buf;
	int i = 0, temp = 0;
	int space_max;
	size_t size = 256 * 4;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;

	hwc_lut = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!hwc_lut)
		return 0;

	/* printk("count:%d\n>>%s\n\n", count, start); */
	for (i = 0; i < 256; i++) {
		space_max = 15;	/* max space number 15 */
		kstrtoul(start, 16, (unsigned long *)&temp);
		hwc_lut[i] = temp;
		do {
			start++;
			space_max--;
		} while ((*start != ' ') && space_max);

		if (!space_max)
			break;

		start++;
	}
#ifdef FBSYS_DEBUG
	for (i = 0; i < 16; i++) {
		for (j = 0; j < 16; j++)
			pr_info("0x%08x ", hwc_lut[i * 16 + j]);
		pr_info("\n");
	}
#endif
	if (dev_drv->ops->set_hwc_lut)
		dev_drv->ops->set_hwc_lut(dev_drv, hwc_lut, 1);

	return count;
}

static ssize_t show_dsp_lut(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int i = 0, len = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;

	if (dev_drv->cur_screen->dsp_lut) {
		memset(buf, 0, PAGE_SIZE);
		for (i = 0; i < 256; i++) {
			snprintf(buf + len, PAGE_SIZE - len, "0x%08x ",
				 dev_drv->cur_screen->dsp_lut[i]);
			if (((i + 1) % 8) == 0)
				strcat(buf, "\n");

			len = strlen(buf);
		}
		return len;
	}

	return 0;
}

static ssize_t set_dsp_lut(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	u32 *dsp_lut = NULL;
	const char *start = buf;
	char curr1[20];
	int index = 0;
	const char *curr;
	int i = 0, temp = 0;
	int space_max = 10;
	size_t size = 256 * 4;
	unsigned long temp1 = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;

	dsp_lut = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!dsp_lut)
		return 0;

	/* init by default value */
	for (i = 0; i < 256; i++) {
		temp = i;
		dsp_lut[i] = temp + (temp << 8) + (temp << 16);
	}

	/* printk("count:%d\n>>%s\n\n",count,start); */

	for (i = 0; i < 256; i++) {
		index = 0;
		space_max = 20;	/* max space number 10 */
		curr = start;
		do {
			index++;
			start++;
			space_max--;
		} while ((*start != ' ') && space_max && (index < 20));
		strncpy(curr1, curr, index);
		curr1[index] = '\0';
		kstrtoul(curr1, 0, (unsigned long *)&temp1);
		dsp_lut[i] = temp1;
		if (!space_max)
			break;
		index = 0;
		start++;
	}
#ifdef FBSYS_DEBUG
	for (i = 0; i < 16; i++) {
		for (j = 0; j < 16; j++)
			pr_info("0x%08x ", dsp_lut[i * 16 + j]);
		pr_info("\n");
	}
#endif
	if (dev_drv->ops->set_dsp_lut)
		dev_drv->ops->set_dsp_lut(dev_drv, dsp_lut, 1);

	return count;
}

static ssize_t show_dsp_bcsh(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	int brightness = 0, contrast = 0, sat_con = 0;
	int sin_hue = 0, cos_hue = 0;

	if (dev_drv->ops->get_dsp_bcsh_bcs) {
		brightness = dev_drv->ops->get_dsp_bcsh_bcs(dev_drv,
							    BRIGHTNESS);
		contrast = dev_drv->ops->get_dsp_bcsh_bcs(dev_drv, CONTRAST);
		sat_con = dev_drv->ops->get_dsp_bcsh_bcs(dev_drv, SAT_CON);
	}
	if (dev_drv->ops->get_dsp_bcsh_hue) {
		sin_hue = dev_drv->ops->get_dsp_bcsh_hue(dev_drv, H_SIN);
		cos_hue = dev_drv->ops->get_dsp_bcsh_hue(dev_drv, H_COS);
	}
	return snprintf(buf, PAGE_SIZE,
			"brightness:%4d,contrast:%4d,sat_con:%4d,sin_hue:%4d,cos_hue:%4d\n",
			brightness, contrast, sat_con, sin_hue, cos_hue);
}

static ssize_t set_dsp_bcsh(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct rockchip_fb_par *fb_par = (struct rockchip_fb_par *)fbi->par;
	struct rockchip_vop_driver *dev_drv = fb_par->vop_drv;
	int brightness, contrast, sat_con, sin_hue, cos_hue;
	int ret = 0;

	if (!strncmp(buf, "open", 4)) {
		if (dev_drv->ops->open_bcsh)
			ret = dev_drv->ops->open_bcsh(dev_drv, 1);
		else
			ret = -1;
	} else if (!strncmp(buf, "close", 5)) {
		if (dev_drv->ops->open_bcsh)
			ret = dev_drv->ops->open_bcsh(dev_drv, 0);
		else
			ret = -1;
	} else if (!strncmp(buf, "brightness", 10)) {
		ret = sscanf(buf, "brightness %d", &brightness);
		if (unlikely(brightness > 255)) {
			dev_err(fbi->dev,
				"brightness should be [0:255],now=%d\n\n",
				brightness);
			brightness = 255;
		}
		if (dev_drv->ops->set_dsp_bcsh_bcs)
			ret = dev_drv->ops->set_dsp_bcsh_bcs(dev_drv,
							     BRIGHTNESS,
							     brightness);
		else
			ret = -1;
	} else if (!strncmp(buf, "contrast", 8)) {
		ret = sscanf(buf, "contrast %d", &contrast);
		if (unlikely(contrast > 510)) {
			dev_err(fbi->dev,
				"contrast should be [0:510],now=%d\n",
				contrast);
			contrast = 510;
		}
		if (dev_drv->ops->set_dsp_bcsh_bcs)
			ret = dev_drv->ops->set_dsp_bcsh_bcs(dev_drv,
							     CONTRAST,
							     contrast);
		else
			ret = -1;
	} else if (!strncmp(buf, "sat_con", 7)) {
		ret = sscanf(buf, "sat_con %d", &sat_con);
		if (unlikely(sat_con > 1015)) {
			dev_err(fbi->dev,
				"sat_con should be [0:1015],now=%d\n", sat_con);
			sat_con = 1015;
		}
		if (dev_drv->ops->set_dsp_bcsh_bcs)
			ret = dev_drv->ops->set_dsp_bcsh_bcs(dev_drv,
							     SAT_CON, sat_con);
		else
			ret = -1;
	} else if (!strncmp(buf, "hue", 3)) {
		ret = sscanf(buf, "hue %d %d", &sin_hue, &cos_hue);
		if (unlikely(sin_hue > 511 || cos_hue > 511)) {
			dev_err(fbi->dev, "sin_hue=%d,cos_hue=%d\n",
				sin_hue, cos_hue);
		}
		if (dev_drv->ops->set_dsp_bcsh_hue)
			ret = dev_drv->ops->set_dsp_bcsh_hue(dev_drv,
							     sin_hue, cos_hue);
		else
			ret = -1;
	} else {
		dev_info(dev, "format error\n");
	}

	if (ret < 0)
		return ret;

	return count;
}

static struct device_attribute rockchip_fb_attrs[] = {
	__ATTR(phys_addr, S_IRUGO, show_phys, NULL),
	__ATTR(virt_addr, S_IRUGO, show_virt, NULL),
	__ATTR(disp_info, S_IRUGO | S_IWUSR, show_disp_info, set_dump_info),
	__ATTR(screen_info, S_IRUGO, show_screen_info, NULL),
	__ATTR(dual_mode, S_IRUGO, show_dual_mode, NULL),
	__ATTR(enable, S_IRUGO | S_IWUSR, show_fb_state, set_fb_state),
	__ATTR(overlay, S_IRUGO | S_IWUSR, show_overlay, set_overlay),
	__ATTR(fps, S_IRUGO | S_IWUSR, show_fps, set_fps),
	__ATTR(map, S_IRUGO | S_IWUSR, show_fb_win_map, set_fb_win_map),
	__ATTR(dsp_lut, S_IRUGO | S_IWUSR, show_dsp_lut, set_dsp_lut),
	__ATTR(hwc_lut, S_IRUGO | S_IWUSR, show_hwc_lut, set_hwc_lut),
	__ATTR(bcsh, S_IRUGO | S_IWUSR, show_dsp_bcsh, set_dsp_bcsh),
};

int rockchip_fb_create_sysfs(struct fb_info *fbi)
{
	int r, t;

	for (t = 0; t < ARRAY_SIZE(rockchip_fb_attrs); t++) {
		r = device_create_file(fbi->dev, &rockchip_fb_attrs[t]);
		if (r) {
			dev_err(fbi->dev, "failed to create sysfs " "file\n");
			return r;
		}
	}

	return 0;
}

void rockchip_fb_remove_sysfs(struct rockchip_fb *sfb_info)
{
	int i, t;

	for (i = 0; i < sfb_info->num_fb; i++) {
		for (t = 0; t < ARRAY_SIZE(rockchip_fb_attrs); t++)
			device_remove_file(sfb_info->fb[i]->dev,
					   &rockchip_fb_attrs[t]);
	}
}
