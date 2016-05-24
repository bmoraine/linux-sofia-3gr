/*
 * Copyright (C) 2016, Fuzhou Rockchip Electronics Co.Ltd
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
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>

#include <linux/component.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/leds.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#include <video/mipi_display.h>
#include <video/videomode.h>
#include <video/of_videomode.h>
#include <video/of_display_timing.h>

#define PROP_DISPLAY_RAMLESS	"intel,display-ramless"
#define PROP_DISPLAY_FPS	"intel,display-fps"
#define PROP_DISPLAY_LANES	"intel,display-if-nblanes"
#define PROP_DISPLAY_VIDEOMODE	"intel,display-vid-mode"
#define PROP_DISPLAY_EOT	"intel,display-eot"
#define PROP_DISPLAY_GATE	"intel,display-gate"

#define PROP_DISPLAY_GPIORST	"intel,display-gpio-reset"
#define PROP_DISPLAY_GPIOVH	"intel,display-gpio-vhigh"
#define PROP_DISPLAY_GPIOVL	"intel,display-gpio-vlow"
#define PROP_DISPLAY_GPIOVPWR	"intel,display-gpio-vpwr"

#define GPIO_LIST_POWER_ON	"gpio-power-on"
#define GPIO_LIST_POWER_OFF	"gpio-power-off"

#define PROP_DISPLAY_GPIOTYPE	"intel,gpio-type"
#define PROP_DISPLAY_GPIOVALUE	"intel,gpio-value-delay"

#define CMD_LIST_INIT		"cmd-init"

#define PROP_DISPLAY_CMDTYPE	"intel,cmd-type"
#define PROP_DISPLAY_CMDDATA	"intel,cmd-data"
#define PROP_DISPLAY_CMDDELAY	"intel,cmd-delay"
#define PROP_DISPLAY_CMDLP	"intel,cmd-lp"

#define DISPLAY_TIMINGS_NODE	"display-timings"

#define LCD_MSG_LP 1

#define PROP_DISPLAY_ENABLE_DELAY	"intel,display-enable-delay"
#define PROP_DISPLAY_DISABLE_DELAY	"intel,display-disable-delay"
#define PROP_DISPLAY_PREPARE_DELAY	"intel,display-prepare-delay"
#define PROP_DISPLAY_UNPREPARE_DELAY	"intel,display-unprepare-delay"

enum display_power_t {
	DISPLAY_GPIO_VHIGH = 0,
	DISPLAY_GPIO_VLOW = 1,
	DISPLAY_GPIO_RESET = 2,
	DISPLAY_GPIO_VPWR = 3,
};

enum display_backlight_t {
	PWM_BACKLIGHT = 0,
	LED_BACKLIGHT,
};

struct display_backlight {
	enum display_backlight_t type;
	union {
		struct backlight_device *pwm_bl;
		struct led_classdev *led_bl;
	} u;
};

struct display_cmd {
	struct list_head list;
	const char *name;
	unsigned char type;
	unsigned char *datas;
	int length;
	int delay;		/*in ms */
	unsigned int flags;
};

struct display_power {
	struct list_head list;
	const char *name;
	enum display_power_t type;
	int value;
	int delay;		/*in ms */
};

struct rockchip_panel {
	struct device *dev;
	struct drm_panel panel;
	struct videomode vm;

	struct display_backlight *backlight;
	struct display_power *power_on;
	struct display_power *power_off;
	struct display_cmd *cmds_init;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *           become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *          display the first valid frame after starting to receive
	 *          video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *           turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *             to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;

	int fps;
	int gpio_vhigh;
	int gpio_vlow;
	int gpio_reset;
	int gpio_vpwr;
};

static inline struct rockchip_panel *to_rockchip_panel(struct drm_panel *panel)
{
	return container_of(panel, struct rockchip_panel, panel);
}

static struct of_device_id backlight_dt_match[] = {
	{	.compatible = "pwm-backlight",
		.data = (void *)PWM_BACKLIGHT,
	},
	{	.compatible = "intel,agold620-led",
		.data = (void *)LED_BACKLIGHT,
	},
	{	.compatible = "pwm-leds",
		.data = (void *)LED_BACKLIGHT,
	},
	{},
};

static struct display_backlight *
rockchip_panel_get_backlight(struct rockchip_panel *ctx)
{
	struct device_node *np = ctx->dev->of_node;
	struct device_node *backlight_np;
	struct display_backlight *backlight = NULL;

	backlight_np = of_parse_phandle(np, "backlight", 0);
	if (backlight_np) {
		const struct of_device_id *match;

		match = of_match_node(backlight_dt_match, backlight_np);
		if (!match) {
			dev_info(ctx->dev, "No find match backlight dt node\n");
			return NULL;
		}

		backlight =
			devm_kzalloc(ctx->dev, sizeof(*backlight), GFP_KERNEL);
		if (!backlight)
			return NULL;

		backlight->type = (unsigned long)match->data;

		if (backlight->type == PWM_BACKLIGHT)
			backlight->u.pwm_bl =
				of_find_backlight_by_node(backlight_np);
		else if (backlight->type == LED_BACKLIGHT)
			backlight->u.led_bl =
				of_find_led_classdev_by_node(backlight_np);

		of_node_put(backlight_np);

		if (!backlight->u.pwm_bl && !backlight->u.led_bl) {
			devm_kfree(ctx->dev, backlight);
			backlight = NULL;
			dev_info(ctx->dev, "No find panel backlight device\n");
		}
	} else {
		dev_info(ctx->dev, "No find panel backlight device node\n");
	}

	return backlight;
}

static int rockchip_panel_backlight_on(struct rockchip_panel *ctx)
{
	if (!ctx->backlight)
		ctx->backlight = rockchip_panel_get_backlight(ctx);

	if (likely(ctx->backlight)) {
		if (ctx->backlight->type == PWM_BACKLIGHT) {
			ctx->backlight->u.pwm_bl->props.power =
						FB_BLANK_UNBLANK;
			backlight_update_status(ctx->backlight->u.pwm_bl);
		} else if (ctx->backlight->type == LED_BACKLIGHT) {
			led_classdev_resume(ctx->backlight->u.led_bl);
		} else {
			pr_debug("%s: unknown backlight type!\n", __func__);
		}
	} else {
		return -ENODEV;
	}

	return 0;
}

static int rockchip_panel_backlight_off(struct rockchip_panel *ctx)
{
	if (!ctx->backlight)
		ctx->backlight = rockchip_panel_get_backlight(ctx);

	if (likely(ctx->backlight)) {
		if (ctx->backlight->type == PWM_BACKLIGHT) {
			ctx->backlight->u.pwm_bl->props.power =
						FB_BLANK_POWERDOWN;
			backlight_update_status(ctx->backlight->u.pwm_bl);
		} else if (ctx->backlight->type == LED_BACKLIGHT) {
			led_classdev_suspend(ctx->backlight->u.led_bl);
		} else {
			pr_debug("%s: unknown backlight type!\n", __func__);
		}
	} else {
		return -ENODEV;
	}

	return 0;
}

static int rockchip_panel_write_dsi_cmd(struct mipi_dsi_device *dsi,
					struct display_cmd *cmd)
{
	switch (cmd->type) {
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_DCS_LONG_WRITE:
		return mipi_dsi_dcs_write_buffer(dsi, cmd->datas, cmd->length);
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_GENERIC_LONG_WRITE:
		return mipi_dsi_generic_write(dsi, cmd->datas, cmd->length);
	default:
		pr_err("%s: unsupported mipi dsi cmd type=0x%x\n",
		       __func__, cmd->type);
		return -EINVAL;
	}
}

static int rockchip_panel_display_on(struct rockchip_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret)
		return ret;

	msleep(120);

	return mipi_dsi_dcs_set_display_on(dsi);
}

static int rockchip_panel_display_off(struct rockchip_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		return ret;

	msleep(20);

	return mipi_dsi_dcs_set_display_off(dsi);
}

static int rockchip_panel_display_mode_settings(struct rockchip_panel *ctx)
{
	return 0;
}

static int rockchip_panel_power_settings(struct rockchip_panel *ctx)
{
	return 0;
}

static int rockchip_panel_gamma_settings(struct rockchip_panel *ctx)
{
	return 0;
}

static void rockchip_panel_send_cmdlist(struct rockchip_panel *ctx,
					struct display_cmd *cmds)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct display_cmd *cmd;

	list_for_each_entry(cmd, &cmds->list, list) {
		pr_debug("Sending command 0x%02x of length %d\n",
			 cmd->type, cmd->length);
		rockchip_panel_write_dsi_cmd(dsi, cmd);
	}
}

static int rockchip_panel_init(struct rockchip_panel *ctx)
{
	int ret;

	dev_dbg(ctx->panel.dev, "initializing LCD\n");
	if (ctx->cmds_init)
		rockchip_panel_send_cmdlist(ctx, ctx->cmds_init);

	ret = rockchip_panel_display_mode_settings(ctx);
	if (ret)
		return ret;

	ret = rockchip_panel_power_settings(ctx);
	if (ret)
		return ret;

	return rockchip_panel_gamma_settings(ctx);

	return 0;
}

static void rockchip_panel_set_powerlist(struct rockchip_panel *ctx,
					 struct display_power *powers)
{
	struct display_power *power;

	list_for_each_entry(power, &powers->list, list) {
		switch (power->type) {
		case DISPLAY_GPIO_VHIGH:
			if (!ctx->gpio_vhigh)
				break;

			gpio_direction_output(ctx->gpio_vhigh, power->value);
			break;
		case DISPLAY_GPIO_VLOW:
			if (!ctx->gpio_vlow)
				break;

			gpio_direction_output(ctx->gpio_vlow, power->value);
			break;
		case DISPLAY_GPIO_RESET:
			if (!ctx->gpio_reset)
				break;

			gpio_direction_output(ctx->gpio_reset, power->value);
			break;

		case DISPLAY_GPIO_VPWR:
			if (!ctx->gpio_vpwr)
				break;

			gpio_direction_output(ctx->gpio_vpwr, power->value);
			break;
		default:
			break;
		}

		if (power->delay)
			mdelay(power->delay);
	}
}

static int rockchip_panel_power_off(struct rockchip_panel *ctx)
{
	if (ctx->power_off)
		rockchip_panel_set_powerlist(ctx, ctx->power_off);
	return 0;
}

static int rockchip_panel_power_on(struct rockchip_panel *ctx)
{
	if (ctx->power_on)
		rockchip_panel_set_powerlist(ctx, ctx->power_on);
	return 0;
}

static int rockchip_panel_unprepare(struct drm_panel *panel)
{
	struct rockchip_panel *ctx = to_rockchip_panel(panel);
	int ret = 0;

	ret = rockchip_panel_power_off(ctx);

	if (ctx->delay.unprepare)
		msleep(ctx->delay.unprepare);

	return ret;
}

static int rockchip_panel_prepare(struct drm_panel *panel)
{
	struct rockchip_panel *ctx = to_rockchip_panel(panel);
	int ret = 0;

	ret = rockchip_panel_power_on(ctx);
	if (ret < 0)
		return ret;

	if (ctx->delay.prepare)
		msleep(ctx->delay.prepare);

	ret = rockchip_panel_init(ctx);
	if (ret < 0)
		return ret;

	return rockchip_panel_display_on(ctx);
}

static int rockchip_panel_disable(struct drm_panel *panel)
{
	struct rockchip_panel *ctx = to_rockchip_panel(panel);

	rockchip_panel_backlight_off(ctx);

	if (ctx->delay.disable)
		msleep(ctx->delay.disable);

	return rockchip_panel_display_off(ctx);
}

static int rockchip_panel_enable(struct drm_panel *panel)
{
	struct rockchip_panel *ctx = to_rockchip_panel(panel);

	if (ctx->delay.enable)
		msleep(ctx->delay.enable);

	rockchip_panel_backlight_on(ctx);

	return 0;
}

/*
static const struct drm_display_mode default_mode = {
	.clock = 27000,
	.hdisplay = 480,
	.hsync_start = 480 + 10,
	.hsync_end = 480 + 10 + 59,
	.htotal = 480 + 10 + 59 + 10,
	.vdisplay = 800,
	.vsync_start = 800 + 15,
	.vsync_end = 800 + 15 + 15,
	.vtotal = 800 + 15 + 15 + 15,
	.vrefresh = 60,
};
*/

static int rockchip_panel_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct rockchip_panel *ctx = to_rockchip_panel(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode\n");
		return 0;
	}

	drm_display_mode_from_videomode(&ctx->vm, mode);

	mode->vrefresh = ctx->fps;
	mode->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs rockchip_panel_drm_funcs = {
	.unprepare = rockchip_panel_unprepare,
	.prepare = rockchip_panel_prepare,
	.disable = rockchip_panel_disable,
	.enable = rockchip_panel_enable,
	.get_modes = rockchip_panel_get_modes,
};

static int panel_parse_display_cmd(struct rockchip_panel *ctx,
				   struct device_node *n,
				   struct display_cmd *cmd)
{
	int ret = 0, i;
	u32 val;
	const __be32 *p;
	struct property *prop;

	cmd->flags = 0;
	cmd->length = 0;
	of_property_for_each_u32(n, PROP_DISPLAY_CMDDATA, prop, p, val) {
		cmd->length++;
	};

	/* allocate data array if needed */
	if (cmd->length > 0) {
		cmd->datas = devm_kzalloc(
				ctx->dev, cmd->length * sizeof(u8), GFP_KERNEL);
		if (!cmd->datas)
			return -ENOMEM;
	}

	cmd->name = n->name;

	/* populate header+data */
	i = 0;
	of_property_for_each_u32(n, PROP_DISPLAY_CMDDATA, prop, p, val) {
		if (cmd->datas)
			cmd->datas[i] = val;
		i++;
	}

	ret = of_property_read_u32(n, PROP_DISPLAY_CMDTYPE, &val);
	if (ret)
		cmd->type = 0;
	else
		cmd->type = val;

	ret = of_property_read_u32(n, PROP_DISPLAY_CMDDELAY, &cmd->delay);
	if (ret)
		cmd->delay = 0;

	ret = of_property_read_u32(n, PROP_DISPLAY_CMDLP, &val);
	if (!ret && val)
		cmd->flags |= LCD_MSG_LP;

	return 0;
}

static int panel_parse_display_cmdlist(struct rockchip_panel *ctx,
				       struct device_node *n,
				       struct display_cmd **cmdlist)
{
	struct device_node *child;
	struct display_cmd *cmd;

	/* allocate cmd list */
	*cmdlist = devm_kzalloc(ctx->dev, sizeof(*cmd), GFP_KERNEL);
	if (!*cmdlist)
		return -ENOMEM;

	INIT_LIST_HEAD(&(*cmdlist)->list);
	for_each_child_of_node(n, child) {
		cmd = devm_kzalloc(ctx->dev, sizeof(*cmd), GFP_KERNEL);
		if (!cmd)
			return -ENOMEM;

		if (!panel_parse_display_cmd(ctx, child, cmd))
			list_add_tail(&cmd->list, &(*cmdlist)->list);
		else
			devm_kfree(ctx->dev, cmd);
	}

	return 0;
}

static int panel_parse_display_gpio(struct device_node *n,
				    struct display_power *gpio)
{
	const char *string;
	int array[2];
	int ret;

	gpio->name = n->name;
	ret = of_property_read_string(n, PROP_DISPLAY_GPIOTYPE, &string);
	if (ret) {
		pr_err("%s: Get %s failed\n", __func__, PROP_DISPLAY_GPIOTYPE);
		return ret;
	} else if (!strcmp("vhigh", string)) {
		gpio->type = DISPLAY_GPIO_VHIGH;
	} else if (!strcmp("vlow", string)) {
		gpio->type = DISPLAY_GPIO_VLOW;
	} else if (!strcmp("reset", string)) {
		gpio->type = DISPLAY_GPIO_RESET;
	} else if (!strcmp("vpwr", string)) {
		gpio->type = DISPLAY_GPIO_VPWR;
	}

	ret = of_property_read_u32_array(n, PROP_DISPLAY_GPIOVALUE, array, 2);
	if (ret) {
		pr_err("%s: Get %s failed\n", __func__, PROP_DISPLAY_GPIOVALUE);
		return ret;
	}

	gpio->value = array[0];
	gpio->delay = array[1];

	return 0;
}

static int panel_parse_display_powerlist(struct rockchip_panel *ctx,
					 struct device_node *n,
					 struct display_power **powerlist)
{
	struct device_node *child;
	struct display_power *power;

	*powerlist = devm_kzalloc(ctx->dev, sizeof(*power), GFP_KERNEL);
	if (!*powerlist)
		return -ENOMEM;

	INIT_LIST_HEAD(&(*powerlist)->list);
	for_each_child_of_node(n, child) {
		power = devm_kzalloc(ctx->dev, sizeof(*power), GFP_KERNEL);
		if (!power)
			return -ENOMEM;

		if (!panel_parse_display_gpio(child, power))
			list_add_tail(&power->list, &(*powerlist)->list);
		else
			devm_kfree(ctx->dev, power);
	}

	return 0;
}

static int panel_request_gpio(int gpio, enum of_gpio_flags gpio_flags,
			      const char *name)
{
	unsigned long flags = 0;
	int ret;

	if (!gpio_is_valid(gpio))
		return -EINVAL;

	if (gpio_flags & OF_GPIO_ACTIVE_LOW)
		flags = GPIOF_OUT_INIT_LOW;
	else
		flags = GPIOF_OUT_INIT_HIGH;

	ret = gpio_request_one(gpio, flags, name);
	if (ret) {
		pr_err("%s: request power gpio %s failed: %d\n",
		       __func__, name, ret);
		return -EINVAL;
	}

	return 0;
}

static int panel_parse_gpio(struct rockchip_panel *ctx)
{
	struct device_node *np = ctx->dev->of_node;
	enum of_gpio_flags gpio_flags;

	ctx->gpio_vhigh = of_get_named_gpio_flags(
				np, PROP_DISPLAY_GPIOVH, 0, &gpio_flags);
	if (panel_request_gpio(ctx->gpio_vhigh, gpio_flags, "disp_vhigh"))
		ctx->gpio_vhigh = 0;

	ctx->gpio_vlow = of_get_named_gpio_flags(
				np, PROP_DISPLAY_GPIOVL, 0, &gpio_flags);
	if (panel_request_gpio(ctx->gpio_vlow, gpio_flags, "disp_vlow"))
		ctx->gpio_vlow = 0;

	ctx->gpio_reset = of_get_named_gpio_flags(
				np, PROP_DISPLAY_GPIORST, 0, &gpio_flags);
	if (panel_request_gpio(ctx->gpio_reset, gpio_flags, "disp_rst"))
		ctx->gpio_reset = 0;

	ctx->gpio_vpwr = of_get_named_gpio_flags(
				np, PROP_DISPLAY_GPIOVPWR, 0, &gpio_flags);
	if (panel_request_gpio(ctx->gpio_vpwr, gpio_flags, "disp_vpwr"))
		ctx->gpio_vpwr = 0;

	return 0;
}

static int panel_parse_dsi_dt(struct rockchip_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct device_node *np = ctx->dev->of_node;
	const char *string;
	int value, ret = 0;

	ret = of_property_read_u32(np, PROP_DISPLAY_RAMLESS, &value);
	if (ret || value)
		dsi->mode_flags |= MIPI_DSI_MODE_VIDEO; /* otherwise cmd mode */

	ret = of_property_read_u32(np, PROP_DISPLAY_LANES, &dsi->lanes);
	if (ret)
		dsi->lanes = 4;

	/* default use LP Mode to send cmd,maybe need to modify */
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/* default set format RGB888,maybe need to modify */
	dsi->format = MIPI_DSI_FMT_RGB888;

	ret = of_property_read_u32(np, PROP_DISPLAY_EOT, &value);
	if (ret || value)
		dsi->mode_flags |= MIPI_DSI_MODE_EOT_PACKET;

	ret = of_property_read_u32(np, PROP_DISPLAY_GATE, &value);
	if (ret || value)
		dsi->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = of_property_read_string(np, PROP_DISPLAY_VIDEOMODE, &string);
	if (ret) {
		dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
	} else if (!strcmp("pulses", string)) {
		dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
	} else if (!strcmp("burst", string)) {
		dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
	} else {
		dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
		pr_info("%s: Unknown dsi video mode type %s\n",
			__func__, string);
	}

	return 0;
}

static int rockchip_panel_parse_dt(struct rockchip_panel *ctx)
{
	struct device_node *np = ctx->dev->of_node;
	struct device_node *child;
	int ret = 0;

	of_property_read_u32(np, PROP_DISPLAY_ENABLE_DELAY,
			     &ctx->delay.enable);
	of_property_read_u32(np, PROP_DISPLAY_DISABLE_DELAY,
			     &ctx->delay.disable);
	of_property_read_u32(np, PROP_DISPLAY_PREPARE_DELAY,
			     &ctx->delay.prepare);
	of_property_read_u32(np, PROP_DISPLAY_UNPREPARE_DELAY,
			     &ctx->delay.unprepare);

	ret = of_get_videomode(np, &ctx->vm, OF_USE_NATIVE_MODE);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(np, PROP_DISPLAY_FPS, &ctx->fps);
	if (ret)
		ctx->fps = 60;

	panel_parse_gpio(ctx);
	panel_parse_dsi_dt(ctx);

	for_each_child_of_node(np, child) {
		if (!strcmp(child->name, CMD_LIST_INIT))
			ret = panel_parse_display_cmdlist(
					ctx, child, &ctx->cmds_init);
		else if (!strcmp(child->name, GPIO_LIST_POWER_ON))
			ret = panel_parse_display_powerlist(
					ctx, child, &ctx->power_on);
		else if (!strcmp(child->name, GPIO_LIST_POWER_OFF))
			ret = panel_parse_display_powerlist(
					ctx, child, &ctx->power_off);
		else if (!strcmp(child->name, DISPLAY_TIMINGS_NODE))
			continue;
		else
			pr_info("%s: In node %s, unexpected child %s !\n",
				__func__, np->name, child->name);

		if (ret) {
			pr_info("%s: Node %s parsing failed %d\n",
				__func__, child->name, ret);
		}
	};

	return 0;
}

static int rockchip_panel_probe(struct mipi_dsi_device *dsi)
{
	struct rockchip_panel *ctx;
	int ret;

	ctx = devm_kzalloc(&dsi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = &dsi->dev;
	mipi_dsi_set_drvdata(dsi, ctx);

	ret = rockchip_panel_parse_dt(ctx);
	if (ret < 0)
		return ret;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = ctx->dev;
	ctx->panel.funcs = &rockchip_panel_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

	return ret;
}

static int rockchip_panel_remove(struct mipi_dsi_device *dsi)
{
	struct rockchip_panel *ctx = mipi_dsi_get_drvdata(dsi);

	rockchip_panel_display_off(ctx);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id rockchip_panel_of_match[] = {
	{ .compatible = "rockchip,panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, rockchip_panel_of_match);

static struct mipi_dsi_driver rockchip_panel_driver = {
	.probe = rockchip_panel_probe,
	.remove = rockchip_panel_remove,
	.driver = {
		.name = "panel-rockchip",
		.of_match_table = rockchip_panel_of_match,
	},
};
module_mipi_dsi_driver(rockchip_panel_driver);

MODULE_AUTHOR("Wenlong Zhuang <daisen.zhuang@rock-chips.com>");
MODULE_DESCRIPTION("rockchip LCD panel Driver");
MODULE_LICENSE("GPL v2");
