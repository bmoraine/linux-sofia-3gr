/*
 * Component: XGold MIPI DSI driver
 *
 * Copyright (C) 2016, Fuzhou Rockchip Electronics Co.Ltd
 * Author: Wenlong Zhuang <daisen.zhuang@rock-chips.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_of.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/gpio.h>

#include <video/videomode.h>
#include <video/mipi_display.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_vop.h"
#include "xgold_dsi-rockchip.h"

#define DSI_STATE_INITIALIZED		BIT(0)
#define DSI_STATE_TRANSMIT_CMD		BIT(1)
#define DSI_STATE_TRANSMIT_VIDEO	BIT(2)

#define PROP_DISPLAY_DCCLK	"intel,display-dc-clkrate"
#define PROP_DISPLAY_BITRATE	"intel,display-dsi-bitrate"

#define host_to_dsi(x) container_of(x, struct xgold_mipi_dsi, dsi_host)
#define connector_to_dsi(x) container_of(x, struct xgold_mipi_dsi, connector)
#define encoder_to_dsi(x) container_of(x, struct xgold_mipi_dsi, encoder)

static int xgold_dsi_host_attach(struct mipi_dsi_host *host,
				 struct mipi_dsi_device *device)
{
	struct xgold_mipi_dsi *dsi = host_to_dsi(host);

	dsi->format = device->format;
	xgold_dsi_init_config(dsi, device);

	dsi->panel = of_drm_find_panel(device->dev.of_node);
	if (dsi->panel)
		return drm_panel_attach(dsi->panel, &dsi->connector);

	return -EINVAL;
}

static int xgold_dsi_host_detach(struct mipi_dsi_host *host,
				   struct mipi_dsi_device *device)
{
	struct xgold_mipi_dsi *dsi = host_to_dsi(host);

	drm_panel_detach(dsi->panel);
	return 0;
}

static ssize_t xgold_dsi_host_transfer(struct mipi_dsi_host *host,
				       const struct mipi_dsi_msg *msg)
{
	struct xgold_mipi_dsi *dsi = host_to_dsi(host);
	int ret = 0;

	if (dsi->suspend)
		return -EINVAL;

	if (!(dsi->state & DSI_STATE_INITIALIZED) ||
	    (dsi->state & DSI_STATE_TRANSMIT_VIDEO)) {
		xgold_dsi_init(dsi);
		dsi->state |= DSI_STATE_INITIALIZED;
	}

	if (!(dsi->state & DSI_STATE_TRANSMIT_CMD)) {
		xgold_dsi_start_command(dsi);
		dsi->state &= ~DSI_STATE_TRANSMIT_VIDEO;
		dsi->state |= DSI_STATE_TRANSMIT_CMD;
	}

	if (msg->tx_len == 0)
		return -EINVAL;

	switch (msg->type) {
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
		xgold_dsi_send_short_packet(dsi, msg);
		break;
	case MIPI_DSI_DCS_LONG_WRITE:
		xgold_dsi_send_long_packet(dsi, msg);
		break;
	case MIPI_DSI_DCS_READ:
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		/* ret = xgold_dsi_read_cmd(dsi); */
	default:
		dev_err(dsi->dev, "unsupported message type\n");
		ret = -EINVAL;
	}

	ret = dsi_completion_timeout_ms(&dsi->sync.dsifin,
					dsi->sync.dsifin_to);
	if (!ret) {
		DRM_DEBUG("dsifin interrupt timedout %dms\n",
			  dsi->sync.dsifin_to);
	} else {
		/* DSI_DBG2("eoc received\n"); */
#ifdef USE_DSI_ACKNOWLEDGE
		ret = dsi_mipidsi_ack_wait(display);
		dsi_mipidsi_force_ownership(display);
#endif
	}

	return ret;
}

static const struct mipi_dsi_host_ops xgold_dsi_host_ops = {
	.attach = xgold_dsi_host_attach,
	.detach = xgold_dsi_host_detach,
	.transfer = xgold_dsi_host_transfer,
};

static int xgold_dsi_poweron(struct xgold_mipi_dsi *dsi)
{
	if (!(dsi->state & DSI_STATE_INITIALIZED)) {
		xgold_dsi_init(dsi);
		dsi->state |= DSI_STATE_INITIALIZED;
	}

	if (!(dsi->state & DSI_STATE_TRANSMIT_VIDEO)) {
		xgold_dsi_start_video(dsi);
		dsi->state &= ~DSI_STATE_TRANSMIT_CMD;
		dsi->state |= DSI_STATE_TRANSMIT_VIDEO;
	}

	return 0;
}

static int xgold_dsi_poweroff(struct xgold_mipi_dsi *dsi)
{
	if (dsi->state & DSI_STATE_INITIALIZED) {
		xgold_dsi_stop(dsi);
		dsi->state = 0;
	}

	return 0;
}

static enum drm_connector_status
xgold_dsi_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void xgold_dsi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
	connector->dev = NULL;
}

static const struct drm_connector_funcs xgold_dsi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = xgold_dsi_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = xgold_dsi_connector_destroy,
};

static int xgold_dsi_get_modes(struct drm_connector *connector)
{
	struct xgold_mipi_dsi *dsi = connector_to_dsi(connector);

	if (dsi->panel)
		return dsi->panel->funcs->get_modes(dsi->panel);

	return 0;
}

static struct drm_encoder *
xgold_dsi_best_encoder(struct drm_connector *connector)
{
	struct xgold_mipi_dsi *dsi = connector_to_dsi(connector);

	return &dsi->encoder;
}

static const struct drm_connector_helper_funcs
xgold_dsi_connector_helper_funcs = {
	.get_modes = xgold_dsi_get_modes,
	.best_encoder = xgold_dsi_best_encoder,
};

static void xgold_dsi_enable(struct drm_encoder *encoder)
{
	struct xgold_mipi_dsi *dsi = encoder_to_dsi(encoder);
	int ret;

	if (!dsi->suspend)
		return;

	/* pm switch to power on */
	if (dsi->pm_platdata)
		device_state_pm_set_state_by_name(
			dsi->dev, dsi->pm_platdata->pm_state_D0_name);

	dsi->suspend = false;
	drm_panel_prepare(dsi->panel);
	ret = xgold_dsi_poweron(dsi);
	if (ret < 0) {
		drm_panel_unprepare(dsi->panel);
		return;
	}
	drm_panel_enable(dsi->panel);

	rockchip_drm_crtc_user_commit(encoder->crtc,
				      dsi->connector.connector_type, 0);
}

static void xgold_dsi_disable(struct drm_encoder *encoder)
{
	struct xgold_mipi_dsi *dsi = encoder_to_dsi(encoder);

	if (dsi->suspend)
		return;

	drm_panel_disable(dsi->panel);
	xgold_dsi_poweroff(dsi);
	drm_panel_unprepare(dsi->panel);

	/* pm switch to power off */
	if (dsi->pm_platdata)
		device_state_pm_set_state_by_name(
			dsi->dev, dsi->pm_platdata->pm_state_D3_name);

	dsi->suspend = true;
}

static void xgold_dsi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	switch (mode) {
	case DRM_MODE_DPMS_ON:
		xgold_dsi_enable(encoder);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		xgold_dsi_disable(encoder);
		break;
	default:
		break;
	}
}

static bool xgold_dsi_encoder_mode_fixup(struct drm_encoder *encoder,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void xgold_dsi_encoder_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct xgold_mipi_dsi *dsi = encoder_to_dsi(encoder);
	struct videomode *vm = &dsi->vm;
	struct drm_display_mode *m = adjusted_mode;

	drm_display_mode_to_videomode(m, vm);
	xgold_dsi_set_display_mode(dsi, vm, m->vrefresh);
}

static void xgold_dsi_encoder_prepare(struct drm_encoder *encoder)
{
	struct xgold_mipi_dsi *dsi = encoder_to_dsi(encoder);
	int ret;
	u32 interface_pix_fmt;

	switch (dsi->format) {
	case MIPI_DSI_FMT_RGB888:
		interface_pix_fmt = ROCKCHIP_OUT_MODE_P888;
		break;
	case MIPI_DSI_FMT_RGB666:
		interface_pix_fmt = ROCKCHIP_OUT_MODE_P666;
		break;
	case MIPI_DSI_FMT_RGB565:
		interface_pix_fmt = ROCKCHIP_OUT_MODE_P565;
		break;
	default:
		WARN_ON(1);
		return;
	}

	ret = rockchip_drm_crtc_mode_config(encoder->crtc,
					    dsi->connector.connector_type,
					    interface_pix_fmt);
	if (ret < 0) {
		dev_err(dsi->dev, "Could not set crtc mode config: %d\n", ret);
		return;
	}
}

static void xgold_dsi_encoder_commit(struct drm_encoder *encoder)
{
	xgold_dsi_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static const struct drm_encoder_helper_funcs xgold_dsi_encoder_helper_funcs = {
	.dpms = xgold_dsi_encoder_dpms,
	.mode_fixup = xgold_dsi_encoder_mode_fixup,
	.mode_set = xgold_dsi_encoder_mode_set,
	.prepare = xgold_dsi_encoder_prepare,
	.commit = xgold_dsi_encoder_commit,
	.enable = xgold_dsi_enable,
	.disable = xgold_dsi_disable,
};

static const struct drm_encoder_funcs xgold_dsi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int xgold_dsi_bind(struct device *dev, struct device *master,
			  void *data)
{
	struct xgold_mipi_dsi *dsi = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	int ret;

	dsi->drm_dev = drm_dev;

	encoder = &dsi->encoder;
	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm_dev,
							     dev->of_node);

	ret = drm_encoder_init(drm_dev, encoder, &xgold_dsi_encoder_funcs,
			       DRM_MODE_ENCODER_DSI);
	if (ret < 0) {
		DRM_ERROR("failed to initialize encoder with drm\n");
		return ret;
	}

	drm_encoder_helper_add(encoder, &xgold_dsi_encoder_helper_funcs);

	connector = &dsi->connector;
	connector->dpms = DRM_MODE_DPMS_OFF;

	ret = drm_connector_init(drm_dev, connector,
				 &xgold_dsi_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret < 0) {
		DRM_ERROR("failed to initialize connector with drm\n");
		goto err_free_encoder;
	}

	drm_connector_helper_add(connector,
				 &xgold_dsi_connector_helper_funcs);

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret < 0) {
		DRM_ERROR("failed to attach connector and encoder\n");
		goto err_free_connector;
	}

	dsi->dsi_host.ops = &xgold_dsi_host_ops;
	dsi->dsi_host.dev = dev;
	mipi_dsi_host_register(&dsi->dsi_host);

	xgold_dsi_pre_init(dsi);

	return 0;

err_free_connector:
	drm_connector_cleanup(connector);
err_free_encoder:
	drm_encoder_cleanup(encoder);
	return ret;
}

static void xgold_dsi_unbind(struct device *dev, struct device *master,
			     void *data)
{
	struct xgold_mipi_dsi *dsi = dev_get_drvdata(dev);
	/* struct drm_device *drm_dev = data; */

	xgold_dsi_encoder_dpms(&dsi->encoder, DRM_MODE_DPMS_OFF);

	drm_panel_detach(dsi->panel);

	drm_connector_cleanup(&dsi->connector);
	drm_encoder_cleanup(&dsi->encoder);
}

static const struct component_ops xgold_dsi_component_ops = {
	.bind = xgold_dsi_bind,
	.unbind = xgold_dsi_unbind,
};

static int xgold_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct xgold_mipi_dsi *dsi;
	struct resource *res;
	int ret = 0;

	if (!dev->of_node) {
		dev_err(dev, "Don't find xgold mipi dsi device tree node.\n");
		return -EINVAL;
	}

	dsi = devm_kzalloc(&pdev->dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi) {
		dev_err(dev, "kzalloc xgold_mipi_dsi device failed\n");
		return -ENOMEM;
	}

	dsi->dev = &pdev->dev;
	dsi->suspend = true;

	dev_set_drvdata(dev, dsi);
	dev_set_name(dsi->dev, "xgold-mipi-dsi");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "mipi_dsi_phy");
	dsi->regbase = devm_ioremap_resource(dev, res);
	if (IS_ERR(dsi->regbase)) {
		dev_err(&pdev->dev, "ioremap xgold mipi_dsi_phy reg failed\n");
		return PTR_ERR(dsi->regbase);
	}

	dsi->irq.err = platform_get_irq(pdev, 0);
	if (dsi->irq.err < 0) {
		dev_err(&pdev->dev, "Cannot find ERR IRQ for XGold MIPI DSI\n");
		return dsi->irq.err;
	}

	dsi->dsi_reset = devm_reset_control_get(dev, "dsi");
	if (IS_ERR(dsi->dsi_reset)) {
		dev_err(&pdev->dev, "Get xgold dsi reset control failed\n");
		dsi->dsi_reset = NULL;
	}

	if (of_property_read_u32(dev->of_node, PROP_DISPLAY_DCCLK,
				 &dsi->display.dif.dsi.dc_clk_rate)) {
		DRM_ERROR("Can't get DSI Controller clock rate\n");
		return -EINVAL;
	}

	if (of_property_read_u32(dev->of_node, PROP_DISPLAY_BITRATE,
				 &dsi->display.dif.dsi.bitrate)) {
		dsi->display.dif.dsi.bitrate = 0;
		DRM_DEBUG("Can't get DSI bit clock rate\n");
	}

#ifdef CONFIG_PLATFORM_DEVICE_PM
	dsi->pm_platdata = of_device_state_pm_setup(dev->of_node);
	if (dsi->pm_platdata) {
		ret = device_state_pm_set_class(dev,
						dsi->pm_platdata->pm_user_name);
		if (ret < 0) {
			kfree(dsi->pm_platdata);
			dsi->pm_platdata = NULL;
			return -EINVAL;
		}
	}

#endif

	return component_add(&pdev->dev, &xgold_dsi_component_ops);
}

static int xgold_dsi_remove(struct platform_device *pdev)
{
	return 0;
}

static void xgold_dsi_shutdown(struct platform_device *pdev)
{
}

#ifdef CONFIG_OF
static const struct of_device_id xgold_dsi_dt_ids[] = {
	{.compatible = "intel,xgold-mipi_dsi",},
	{}
};
#endif

struct platform_driver xgold_dsi_driver = {
	.probe = xgold_dsi_probe,
	.remove = xgold_dsi_remove,
	.shutdown = xgold_dsi_shutdown,
	.driver = {
		.name = "xgold-mipi-dsi",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(xgold_dsi_dt_ids),
#endif
	},
};
module_platform_driver(xgold_dsi_driver);

MODULE_AUTHOR("Wenlong Zhuang <daisen.zhuang@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip Specific XGOLD-DSI Driver Extension");
MODULE_LICENSE("GPL v2");
