/*
 * Copyright (C) 2015-2016 Fuzhou Rockchip Electronics Co.Ltd
 * Author: Wenlong Zhuang <daisen.zhuang@rock-chips.com>
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

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_of.h>

#include <linux/component.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fcntl.h>

#include <video/videomode.h>
#include <video/of_videomode.h>
#include <video/of_display_timing.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_vop.h"
#include "rockchip_lvds.h"

#define DISPLAY_OUTPUT_RGB		0
#define DISPLAY_OUTPUT_LVDS		1

#define connector_to_lvds(c) \
		container_of(c, struct rockchip_lvds, connector)

#define encoder_to_lvds(c) \
		container_of(c, struct rockchip_lvds, encoder)

#define BR_READ  0
#define BR_WRITE 1
#define BR_OFF  "0"

static char brightness[4] = "127";

struct rockchip_lvds {
	void *base;
	struct device *dev;
	void __iomem *regs;

	struct videomode vm;
	int output;
	int format;
	int data_width;

	struct drm_device *drm_dev;
	struct drm_panel *panel;
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct device_pm_platdata *pm_platdata;

	struct mutex suspend_lock;
	int suspend;
};

static void lvds_brightness(char* br_value, bool action)
{
        int fd;
        mm_segment_t old_fs = get_fs();
        set_fs(KERNEL_DS);

        fd = sys_open("/sys/class/leds/lcd-backlight/brightness", O_RDWR, 0644);
        if (fd >= 0) {
                if(action == BR_WRITE)
                        sys_write(fd, br_value, strlen(br_value));
                else
                        sys_read(fd, br_value, strlen(br_value));
                sys_close(fd);
        }
        else
        {
                DRM_ERROR("Error opening sysfs file /sys/class/leds/lcd-backlight/brightness\n");
        }
        set_fs(old_fs);
}

static inline u32 lvds_readl(struct rockchip_lvds *lvds, u32 offset)
{
	return readl(lvds->regs + offset);
}

static inline void lvds_writel(struct rockchip_lvds *lvds, u32 offset, u32 val)
{
	writel(val, lvds->regs + offset);
}

static inline void lvds_msk_reg(struct rockchip_lvds *lvds, u32 offset, u64 val)
{
	u32 v = readl(lvds->regs + offset);

	v &= (~(val >> 32));
	v |= (u32)val;
	writel(v, lvds->regs + offset);
}

static inline bool lvds_phy_lockon(struct rockchip_lvds *lvds)
{
	u32 val;

	val = lvds_readl(lvds, LVDS_REG_PLL_STA);
	if (val & M_LVDS_PLL_LOCK)
		return true;
	else
		return false;
}

static inline int lvds_name_to_format(const char *s)
{
	if (!s)
		return -EINVAL;

	if (strncmp(s, "lvds_8bit_1", 11) == 0)
		return LVDS_8BIT_1;
	else if (strncmp(s, "lvds_8bit_2", 11) == 0)
		return LVDS_8BIT_2;
	else if (strncmp(s, "lvds_8bit_3", 11) == 0)
		return LVDS_8BIT_3;
	else if (strncmp(s, "lvds_6bit", 9) == 0)
		return LVDS_6BIT;

	return -EINVAL;
}

static inline int lvds_name_to_output(const char *s)
{
	if (!s)
		return -EINVAL;

	if (strncmp(s, "rgb", 3) == 0)
		return DISPLAY_OUTPUT_RGB;
	else if (strncmp(s, "lvds", 4) == 0)
		return DISPLAY_OUTPUT_LVDS;

	return -EINVAL;
}

static int rockchip_lvds_poweron(struct rockchip_lvds *lvds)
{
	int delay_times = 10;
	int ret;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (lvds->pm_platdata)
		ret = device_state_pm_set_state_by_name(
				lvds->dev, lvds->pm_platdata->pm_state_D0_name);
#endif

	/* iomux set to default state */
	if (lvds->dev->pins && lvds->dev->pins->default_state) {
		ret = pinctrl_select_state(lvds->dev->pins->p,
					   lvds->dev->pins->default_state);
		if (ret < 0) {
			dev_err(lvds->dev, "Error setting PIN default state\n");
			return ret;
		}
	}

	lvds_msk_reg(lvds, LVDS_REG_CTRL, V_LVDS_POWER_MODE(1));

	/* delay for waitting pll lock on */
	while (--delay_times) {
		if (lvds_phy_lockon(lvds))
			break;

		usleep_range(100, 101);
	}

        lvds_brightness(brightness, BR_WRITE);
	return 0;
}

static void rockchip_lvds_poweroff(struct rockchip_lvds *lvds)
{
	int ret;

	/* power down */
	lvds_msk_reg(lvds, LVDS_REG_CTRL, V_LVDS_POWER_MODE(0));

	if (lvds->dev->pins && lvds->dev->pins->sleep_state) {
		ret = pinctrl_select_state(lvds->dev->pins->p,
					   lvds->dev->pins->sleep_state);
		if (ret < 0)
			dev_err(lvds->dev, "Error setting PIN sleep state\n");
	}

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (lvds->pm_platdata)
		ret = device_state_pm_set_state_by_name(
				lvds->dev, lvds->pm_platdata->pm_state_D3_name);
#endif

        lvds_brightness(brightness, BR_READ);
        lvds_brightness(BR_OFF, BR_WRITE);
}

static enum drm_connector_status
rockchip_lvds_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void rockchip_lvds_connector_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs rockchip_lvds_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = rockchip_lvds_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = rockchip_lvds_connector_destroy,
};

static int rockchip_lvds_connector_get_modes(struct drm_connector *connector)
{
	struct rockchip_lvds *lvds = connector_to_lvds(connector);
	struct drm_panel *panel = lvds->panel;
	struct drm_display_mode *mode;

	if (panel && panel->funcs->get_modes)
		return panel->funcs->get_modes(panel);

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode.\n");
		return 0;
	}

	drm_display_mode_from_videomode(&lvds->vm, mode);

	mode->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	return 1;
}

static struct drm_encoder *
rockchip_lvds_connector_best_encoder(struct drm_connector *connector)
{
	struct rockchip_lvds *lvds = connector_to_lvds(connector);

	return &lvds->encoder;
}

static enum drm_mode_status rockchip_lvds_connector_mode_valid(
		struct drm_connector *connector,
		struct drm_display_mode *mode)
{
	return MODE_OK;
}

static const struct drm_connector_helper_funcs
rockchip_lvds_connector_helper_funcs = {
	.get_modes = rockchip_lvds_connector_get_modes,
	.mode_valid = rockchip_lvds_connector_mode_valid,
	.best_encoder = rockchip_lvds_connector_best_encoder,
};

static void rockchip_lvds_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct rockchip_lvds *lvds = encoder_to_lvds(encoder);
	int ret;

	mutex_lock(&lvds->suspend_lock);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		if (!lvds->suspend)
			goto out;

		drm_panel_prepare(lvds->panel);
		ret = rockchip_lvds_poweron(lvds);
		if (ret < 0) {
			drm_panel_unprepare(lvds->panel);
			return;
		}
		drm_panel_enable(lvds->panel);
		rockchip_drm_crtc_user_commit(encoder->crtc,
					      DRM_MODE_CONNECTOR_LVDS, 0);

		lvds->suspend = false;
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		if (lvds->suspend)
			goto out;

		drm_panel_disable(lvds->panel);
		rockchip_lvds_poweroff(lvds);
		drm_panel_unprepare(lvds->panel);

		lvds->suspend = true;
		break;
	default:
		break;
	}

out:
	mutex_unlock(&lvds->suspend_lock);
}

static bool
rockchip_lvds_encoder_mode_fixup(struct drm_encoder *encoder,
				 const struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void rockchip_lvds_encoder_mode_set(struct drm_encoder *encoder,
					   struct drm_display_mode *mode,
					   struct drm_display_mode *adjusted)
{
	struct rockchip_lvds *lvds = encoder_to_lvds(encoder);
	u32 val;
	/* u8 pin_hsync = (mode->flags & DRM_MODE_FLAG_PHSYNC) ? 1 : 0; */
	/* u8 pin_dclk = (mode->flags & DRM_MODE_FLAG_PCSYNC) ? 1 : 0; */

	if (lvds->output == DISPLAY_OUTPUT_RGB) {
		lvds_msk_reg(lvds, LVDS_REG_CTRL, V_LVDS_TTL_MODE_EN(1));
	} else {
		val = V_LVDS_TTL_MODE_EN(0) |
			V_LVDS_SELECT(lvds->format) |
			V_LVDS_MSBSEL(LVDS_MSB_D7);
		if (lvds->data_width == 24)
			val |= V_LVDS_DATA_BITS(LVDS_8_BIT);
		else
			val |= V_LVDS_DATA_BITS(LVDS_6_BIT);

		val |= V_LVDS_CLK_EDGE(LVDS_EDGE_FALL) |
			V_LVDS_CLK_DS1(LVDS_SKEW_CLK_0PS) |
			V_LVDS_OFFSET_VOLT(LVDS_OFFSET_800MV) |
			V_LVDS_SWING(LVDS_SWING_500MV) |
			V_LVDS_PRE_EMPHASIS(LVDS_EMP_2DB);
		lvds_msk_reg(lvds, LVDS_REG_CTRL, val);
	}
}

static void rockchip_lvds_encoder_prepare(struct drm_encoder *encoder)
{
	struct rockchip_lvds *lvds = encoder_to_lvds(encoder);
	int ret;

	ret = rockchip_drm_crtc_mode_config(encoder->crtc,
					    lvds->connector.connector_type,
					    ROCKCHIP_OUT_MODE_P888);
	if (ret < 0) {
		dev_err(lvds->dev, "Could not set crtc mode config: %d\n", ret);
		return;
	}
}

static void rockchip_lvds_encoder_commit(struct drm_encoder *encoder)
{
	rockchip_lvds_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void rockchip_lvds_encoder_disable(struct drm_encoder *encoder)
{
	rockchip_lvds_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static struct drm_encoder_helper_funcs rockchip_lvds_encoder_helper_funcs = {
	.dpms = rockchip_lvds_encoder_dpms,
	.mode_fixup = rockchip_lvds_encoder_mode_fixup,
	.mode_set = rockchip_lvds_encoder_mode_set,
	.prepare = rockchip_lvds_encoder_prepare,
	.commit = rockchip_lvds_encoder_commit,
	.disable = rockchip_lvds_encoder_disable,
};

static void rockchip_lvds_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static struct drm_encoder_funcs rockchip_lvds_encoder_funcs = {
	.destroy = rockchip_lvds_encoder_destroy,
};

static const struct of_device_id rockchip_lvds_dt_ids[] = {
	{
		.compatible = "rockchip,nanosilicon-lvds",
	},
	{}
};
MODULE_DEVICE_TABLE(of, rockchip_lvds_dt_ids);

static int rockchip_lvds_bind(struct device *dev, struct device *master,
			      void *data)
{
	struct rockchip_lvds *lvds = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	int ret;

	lvds->drm_dev = drm_dev;

	encoder = &lvds->encoder;
	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm_dev,
							     dev->of_node);

	ret = drm_encoder_init(drm_dev, encoder, &rockchip_lvds_encoder_funcs,
			       DRM_MODE_ENCODER_LVDS);
	if (ret < 0) {
		DRM_ERROR("failed to initialize encoder with drm\n");
		return ret;
	}

	drm_encoder_helper_add(encoder, &rockchip_lvds_encoder_helper_funcs);

	connector = &lvds->connector;
	connector->dpms = DRM_MODE_DPMS_OFF;

	ret = drm_connector_init(drm_dev, connector,
				 &rockchip_lvds_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if (ret < 0) {
		DRM_ERROR("failed to initialize connector with drm\n");
		goto err_free_encoder;
	}

	drm_connector_helper_add(connector,
				 &rockchip_lvds_connector_helper_funcs);

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret < 0) {
		DRM_ERROR("failed to attach connector and encoder\n");
		goto err_free_connector;
	}

/*
	ret = drm_panel_attach(lvds->panel, connector);
	if (ret < 0) {
		DRM_ERROR("failed to attach connector and encoder\n");
		goto err_free_connector;
	}
*/
	return 0;

err_free_connector:
	drm_connector_cleanup(connector);
err_free_encoder:
	drm_encoder_cleanup(encoder);
	return ret;
}

static void rockchip_lvds_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct rockchip_lvds *lvds = dev_get_drvdata(dev);
	/* struct drm_device *drm_dev = data; */

	rockchip_lvds_encoder_dpms(&lvds->encoder, DRM_MODE_DPMS_OFF);
/*
	drm_panel_detach(lvds->panel);
*/
	drm_connector_cleanup(&lvds->connector);
	drm_encoder_cleanup(&lvds->encoder);
}

static const struct component_ops rockchip_lvds_component_ops = {
	.bind = rockchip_lvds_bind,
	.unbind = rockchip_lvds_unbind,
};

static int rockchip_lvds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rockchip_lvds *lvds;
	struct resource *res;
	const char *name;
	int i, ret;

	if (!dev->of_node)
		return -ENODEV;

	lvds = devm_kzalloc(&pdev->dev, sizeof(*lvds), GFP_KERNEL);
	if (!lvds)
		return -ENOMEM;

	lvds->dev = dev;
	lvds->suspend = true;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lvds->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(lvds->regs))
		return PTR_ERR(lvds->regs);

	dev_set_drvdata(dev, lvds);
	mutex_init(&lvds->suspend_lock);

	if (of_property_read_string(dev->of_node, "rockchip,output", &name))
		/* default set it as output rgb */
		lvds->output = DISPLAY_OUTPUT_RGB;
	else
		lvds->output = lvds_name_to_output(name);

	if (of_property_read_string(dev->of_node, "rockchip,data-mapping",
				    &name)) {
		/* default set it as LVDS_8BIT_1 */
		lvds->format = LVDS_8BIT_1;
	} else {
		lvds->format = lvds_name_to_format(name);
		if (lvds->format < 0) {
			dev_err(&pdev->dev,
				"rockchip-lvds unsupport format[%s]\n", name);
			return -EINVAL;
		}
	}

	if (of_property_read_u32(dev->of_node, "rockchip,data-width", &i)) {
		lvds->data_width = 24;
	} else {
		if (i == 24 || i == 18) {
			lvds->data_width = i;
		} else {
			dev_err(&pdev->dev,
				"rockchip-lvds unsupport data-width[%d]\n", i);
			return -EINVAL;
		}
	}

	ret = of_get_videomode(dev->of_node, &lvds->vm, OF_USE_NATIVE_MODE);
	if (ret) {
		DRM_ERROR("failed: of_get_videomode() : %d\n", ret);
		return ret;
	}

#ifdef CONFIG_PLATFORM_DEVICE_PM
	lvds->pm_platdata = of_device_state_pm_setup(dev->of_node);
	if (lvds->pm_platdata) {
		ret = device_state_pm_set_class(dev,
					lvds->pm_platdata->pm_user_name);
		if (ret < 0) {
			kfree(lvds->pm_platdata);
			lvds->pm_platdata = NULL;
			return -EINVAL;
		}
	}
#endif

	return component_add(&pdev->dev, &rockchip_lvds_component_ops);
}

static int rockchip_lvds_remove(struct platform_device *pdev)
{
	/* struct rockchip_lvds *lvds = dev_get_drvdata(&pdev->dev); */

	component_del(&pdev->dev, &rockchip_lvds_component_ops);

	return 0;
}

struct platform_driver rockchip_lvds_driver = {
	.probe = rockchip_lvds_probe,
	.remove = rockchip_lvds_remove,
	.driver = {
		   .name = "rockchip-lvds",
		   .of_match_table = of_match_ptr(rockchip_lvds_dt_ids),
	},
};
module_platform_driver(rockchip_lvds_driver);

MODULE_AUTHOR("Wenlong Zhuang <daisen.zhuang@rock-chips.com>");
MODULE_DESCRIPTION("ROCKCHIP LVDS Driver");
MODULE_LICENSE("GPL v2");
