/*
 *******************************************************************************
 *
 *  Component: XGold MIPI DSI driver
 *
 *  Copyright (C) 2014, Intel Mobile Communications GmbH.
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
 *******************************************************************************
 */

#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/rockchip_fb.h>
#include <linux/reset.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>

#include "dsi_device.h"
#include "dsi_hwregs.h"
#include "dsi_dts.h"

#define PROP_DISPLAY            "intel,display"
#define PROP_SCREEN             "intel,screen"

#define PROP_DISPLAY_DCCLK      "intel,display-dc-clkrate"
#define PROP_DISPLAY_RAMLESS    "intel,display-ramless"
#define PROP_DISPLAY_FPS        "intel,display-fps"
#define PROP_DISPLAY_LANES      "intel,display-if-nblanes"
#define PROP_DISPLAY_PREINIT    "intel,display-preinit"
#define PROP_DISPLAY_VIDEOMODE  "intel,display-vid-mode"
#define PROP_DISPLAY_VIDEOID    "intel,display-vid-id"
#define PROP_DISPLAY_EOT        "intel,display-eot"
#define PROP_DISPLAY_GATE       "intel,display-gate"

#define PROP_DISPLAY_GPIORST    "intel,display-gpio-reset"
#define PROP_DISPLAY_GPIOVH     "intel,display-gpio-vhigh"
#define PROP_DISPLAY_GPIOVL     "intel,display-gpio-vlow"
#define PROP_DISPLAY_GPIOID0    "intel,display-gpio-id0"
#define PROP_DISPLAY_GPIOID1    "intel,display-gpio-id1"

#define PANEL_DETECT_NODE       "panel-detect"

#define PROP_DETECT_METHOD      "intel,id-detect-method"
#define PROP_ID_VERIFICATION    "intel,id-verification"

#define GPIO_LIST_POWER_ON      "gpio-power-on"
#define GPIO_LIST_POWER_OFF     "gpio-power-off"

#define PROP_DISPLAY_GPIOTYPE   "intel,gpio-type"
#define PROP_DISPLAY_GPIOVALUE  "intel,gpio-value-delay"

#define CMD_LIST_INIT           "cmd-init"
#define CMD_LIST_UPDATE         "cmd-update"
#define CMD_LIST_SLEEP_IN       "cmd-sleep-in"
#define CMD_LIST_SLEEP_OUT      "cmd-sleep-out"

#define PROP_DISPLAY_CMDTYPE    "intel,cmd-type"
#define PROP_DISPLAY_CMDDATA    "intel,cmd-data"
#define PROP_DISPLAY_CMDDELAY   "intel,cmd-delay"
#define PROP_DISPLAY_CMDLP      "intel,cmd-lp"

#define DISPLAY_TIMINGS_NODE    "display-timings"

#define PORCH_SYNC_MAX 0xFF

static struct of_device_id display_of_match[] = {
	{ .compatible = PROP_DISPLAY, },
	{ },
};

static struct of_device_id screen_of_match[] = {
	{ .compatible = PROP_SCREEN, },
	{ },
};

static int dsi_of_parse_display_cmd(struct platform_device *pdev,
				    struct device_node *n,
				    struct display_msg *cmd)
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
		cmd->datas = devm_kzalloc(&pdev->dev,
				cmd->length*sizeof(u8), GFP_KERNEL);
		if (!cmd->datas) {
			pr_err("%s: Can't alloc array for %s length %dbytes\n",
			       __func__, n->name, cmd->length);
			return -ENOMEM;
		}
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

static int dsi_of_parse_display_msglist(struct platform_device *pdev,
					struct device_node *n,
					struct display_msg **msglist)
{
	struct device_node *child;
	struct display_msg *msg;

	/* allocate cmd list */
	*msglist = devm_kzalloc(&pdev->dev, sizeof(struct display_msg),
				GFP_KERNEL);
	if (!*msglist) {
		pr_err("%s: Can't alloc commands table\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&(*msglist)->list);
	for_each_child_of_node(n, child) {
		msg = (struct display_msg *) devm_kzalloc(&pdev->dev,
			sizeof(struct display_msg), GFP_KERNEL);
		if (!msg) {
			pr_err("%s: Allocation of display msg failed\n",
			       __func__);
			return -EINVAL;
		}

		if (!dsi_of_parse_display_cmd(pdev, child, msg))
			list_add_tail(&msg->list, &(*msglist)->list);
		else
			devm_kfree(&pdev->dev, msg);
	}

	return 0;
}

static int dsi_of_parse_display_gpio(struct platform_device *pdev,
				     struct device_node *n,
				     struct display_gpio *gpio)
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
		gpio->type = DSI_GPIO_VHIGH;
	} else if (!strcmp("vlow", string)) {
		gpio->type = DSI_GPIO_VLOW;
	} else if (!strcmp("reset", string)) {
		gpio->type = DSI_GPIO_RESET;
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

static int dsi_of_parse_display_gpiolist(struct platform_device *pdev,
					 struct device_node *n,
					 struct display_gpio **gpiolist)
{
	struct device_node *child;
	struct display_gpio *gpio;

	*gpiolist = devm_kzalloc(&pdev->dev, sizeof(struct display_gpio),
				 GFP_KERNEL);
	if (!*gpiolist) {
		pr_err("%s: Can't alloc gpio table\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&(*gpiolist)->list);
	for_each_child_of_node(n, child) {
		gpio = (struct display_gpio *) devm_kzalloc(&pdev->dev,
			sizeof(struct display_gpio), GFP_KERNEL);
		if (!gpio) {
			pr_err("%s: Allocation of display gpio failed\n",
			       __func__);
			return -EINVAL;
		}

		if (!dsi_of_parse_display_gpio(pdev, child, gpio))
			list_add_tail(&gpio->list, &(*gpiolist)->list);
		else
			devm_kfree(&pdev->dev, gpio);
	}

	return 0;
}

static int dsi_of_parse_gpio(struct platform_device *pdev,
			     struct xgold_mipi_dsi_device *mipi_dsi)
{
	struct device_node *screen_dev_n;
	enum of_gpio_flags gpio_flags;
	unsigned long flags;
	int ret;

	screen_dev_n = of_find_matching_node(NULL, screen_of_match);
	if (!screen_dev_n) {
		pr_err("%s: Can't find screen matching node\n", __func__);
		return -EINVAL;
	}

	mipi_dsi->gpio_vhigh = of_get_named_gpio_flags(screen_dev_n,
		PROP_DISPLAY_GPIOVH, 0, &gpio_flags);
	if (!gpio_is_valid(mipi_dsi->gpio_vhigh))
		mipi_dsi->gpio_vhigh = 0;

	mipi_dsi->gpio_vlow = of_get_named_gpio_flags(screen_dev_n,
			PROP_DISPLAY_GPIOVL, 0, &gpio_flags);
	if (!gpio_is_valid(mipi_dsi->gpio_vlow))
		mipi_dsi->gpio_vlow = 0;

	mipi_dsi->gpio_reset = of_get_named_gpio_flags(screen_dev_n,
			PROP_DISPLAY_GPIORST, 0, &gpio_flags);
	if (!gpio_is_valid(mipi_dsi->gpio_reset))
		mipi_dsi->gpio_reset = 0;

	mipi_dsi->gpio_id0 = of_get_named_gpio_flags(screen_dev_n,
			PROP_DISPLAY_GPIOID0, 0, &gpio_flags);
	if (gpio_is_valid(mipi_dsi->gpio_id0)) {
		flags = GPIOF_IN;
		ret = gpio_request_one(mipi_dsi->gpio_id0, flags, "disp_id0");

		if (ret) {
			pr_err("%s: request display id0 gpio fail: %d\n",
			       __func__, ret);
			mipi_dsi->gpio_id0 = 0;
		}
	} else {
		mipi_dsi->gpio_id0 = 0;
	}

	mipi_dsi->gpio_id1 = of_get_named_gpio_flags(screen_dev_n,
			PROP_DISPLAY_GPIOID1, 0, &gpio_flags);
	if (gpio_is_valid(mipi_dsi->gpio_id1)) {
		flags = GPIOF_IN;
		ret = gpio_request_one(mipi_dsi->gpio_id1, flags, "disp_id1");

		if (ret) {
			pr_err("%s: request display id1 gpio fail: %d\n",
			       __func__, ret);
			mipi_dsi->gpio_id1 = 0;
		}
	} else {
		mipi_dsi->gpio_id1 = 0;
	}

	return 0;
}

static int
dsi_of_parse_display_timing(struct dsi_display *display,
			    struct device_node *display_dev_n)
{
	struct display_timings *disp_timing;
	struct display_timing *dt;

	disp_timing = of_get_display_timings(display_dev_n);
	if (!disp_timing) {
		pr_err("parse display timing err\n");
		return -EINVAL;
	}
	dt = display_timings_get(disp_timing, disp_timing->native_mode);
	if (IS_ERR_OR_NULL(dt)) {
		pr_err("%s: Can't alloc display\n", __func__);
		return -ENOMEM;
	}
	display->xres = dt->hactive.typ;
	display->yres = dt->vactive.typ;
	if (dt->face == OUT_P565) {
		display->bpp = 16;
		display->dif.dsi.video_pixel = DSI_PIX_BIT16P;
	} else if (dt->face == OUT_P666) {
		display->bpp = 18;
		display->dif.dsi.video_pixel = DSI_PIX_BIT18P;
	} else {
		display->bpp = 24;
		display->dif.dsi.video_pixel = DSI_PIX_BIT24P;
	}

	display->dif.dsi.hfp = PIXELS_TO_BYTES(dt->hfront_porch.typ,
					       display->bpp);
	if (display->dif.dsi.hfp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! HFP = %d, MAX HFP is %d\n", __func__,
			dt->hfront_porch.typ,
			BYTES_TO_PIXELS(PORCH_SYNC_MAX, display->bpp));
		display->dif.dsi.hfp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.hbp = PIXELS_TO_BYTES(dt->hback_porch.typ,
					       display->bpp);
	if (display->dif.dsi.hbp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! HBP = %d, MAX HBP is %d\n", __func__,
			dt->hback_porch.typ,
			BYTES_TO_PIXELS(PORCH_SYNC_MAX, display->bpp));
		display->dif.dsi.hbp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.hsa = PIXELS_TO_BYTES(dt->hsync_len.typ,
					       display->bpp);
	if (display->dif.dsi.hsa > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! HSA = %d, MAX HSA is %d\n", __func__,
			dt->hsync_len.typ,
			BYTES_TO_PIXELS(PORCH_SYNC_MAX, display->bpp));
		display->dif.dsi.hsa = PORCH_SYNC_MAX;
	}

	display->dif.dsi.vfp = dt->vfront_porch.typ;
	if (display->dif.dsi.vfp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! VFP = %d, MAX VFP is %d\n", __func__,
			dt->vfront_porch.typ, PORCH_SYNC_MAX);
		display->dif.dsi.vfp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.vbp = dt->vback_porch.typ;
	if (display->dif.dsi.vbp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! VBP = %d, MAX VBP is %d\n", __func__,
			dt->vback_porch.typ, PORCH_SYNC_MAX);
		display->dif.dsi.vbp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.vsa = dt->vsync_len.typ;
	if (display->dif.dsi.vsa > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! VSA = %d, MAX VSA is %d\n", __func__,
			dt->vsync_len.typ, PORCH_SYNC_MAX);
		display->dif.dsi.vsa = PORCH_SYNC_MAX;
	}
	display->dif.dsi.hfp_lp = 0;
	display->dif.dsi.hbp_lp = 0;
	display->dif.dsi.hsa_lp = 0;
	kfree(disp_timing);
	return 0;
}

static int dsi_of_parse_panel_detect_node(struct platform_device *pdev,
					  struct device_node *n,
					  struct dsi_panel_id_detect **det)
{
	int ret, i;
	const char *string;
	u32 val;
	const __be32 *p;
	struct property *prop;
	struct dsi_panel_id_detect *id_det;

	id_det = (struct dsi_panel_id_detect *) devm_kzalloc(&pdev->dev,
			sizeof(struct dsi_panel_id_detect), GFP_KERNEL);
	if (!id_det) {
		pr_err("%s: Can't alloc panel id struct\n", __func__);
		return -ENOMEM;
	}
	*det = id_det;

	ret = of_property_read_string(n, PROP_DETECT_METHOD, &string);
	if (ret) {
		pr_err("%s: Get %s failed\n", __func__, PROP_DETECT_METHOD);
		id_det->method = DETECT_METHOD_UNKNOWN;
	} else if (!strcmp("gpio", string)) {
		id_det->method = DETECT_METHOD_GPIO;
	} else if (!strcmp("mipi", string)) {
		id_det->method = DETECT_METHOD_MIPI;
	} else {
		pr_err("%s: Get unknown method %s\n", __func__, string);
		id_det->method = DETECT_METHOD_UNKNOWN;
	}

	id_det->cmd_length = 0;
	of_property_for_each_u32(n, PROP_DISPLAY_CMDDATA, prop, p, val) {
		id_det->cmd_length++;
	};

	/* allocate data array if needed */
	if (id_det->cmd_length > 0) {
		id_det->cmd_datas = devm_kzalloc(&pdev->dev,
				id_det->cmd_length*sizeof(u8), GFP_KERNEL);
		if (!id_det->cmd_datas) {
			pr_err("%s: Can't alloc array for %s length %dbytes\n",
			       __func__, n->name, id_det->cmd_length);
			return -ENOMEM;
		}
	}

	/* populate header+data */
	i = 0;
	of_property_for_each_u32(n, PROP_DISPLAY_CMDDATA, prop, p, val) {
		if (id_det->cmd_datas)
			id_det->cmd_datas[i] = val;
		i++;
	}

	ret = of_property_read_u32(n, PROP_DISPLAY_CMDTYPE, &val);
	if (ret)
		id_det->cmd_type = 0;
	else
		id_det->cmd_type = val;

	id_det->id_length = 0;
	of_property_for_each_u32(n, PROP_ID_VERIFICATION, prop, p, val) {
		id_det->id_length++;
	};

	/* allocate data array if needed */
	if (id_det->id_length > 0) {
		id_det->id_verification = devm_kzalloc(&pdev->dev,
				id_det->id_length*sizeof(u8), GFP_KERNEL);
		if (!id_det->id_verification) {
			pr_err("%s: Can't alloc array for %s length %dbytes\n",
			       __func__, n->name, id_det->id_length);
			return -ENOMEM;
		}
	}

	/* populate header+data */
	i = 0;
	of_property_for_each_u32(n, PROP_ID_VERIFICATION, prop, p, val) {
		if (id_det->id_verification)
			id_det->id_verification[i] = val;
		i++;
	}

	return 0;
}

int dsi_of_parse_display(struct platform_device *pdev,
			 struct xgold_mipi_dsi_device *mipi_dsi)
{
	int value, ret = 0;
	const char *string;
	struct device_node *display_dev_n, *child;
	struct dsi_display *display;
	struct dsi_display *display_curr;
	int index = 0;
	dsi_of_parse_gpio(pdev, mipi_dsi);
	mipi_dsi->dsi_reset = devm_reset_control_get(&pdev->dev, "dsi");
	if (IS_ERR(mipi_dsi->dsi_reset)) {
		pr_err("%s: get dsi reset control failed\n", __func__);
		mipi_dsi->dsi_reset = NULL;
	}
	INIT_LIST_HEAD(&(mipi_dsi)->display_list);

	for_each_matching_node(display_dev_n, display_of_match) {
		if (!display_dev_n) {
			pr_err(
			       "%s: Can't find display matching node\n",
			       __func__);
			return -EINVAL;
		}
		display = (struct dsi_display *) devm_kzalloc(
				&pdev->dev, sizeof(struct dsi_display),
				GFP_KERNEL);
		if (IS_ERR_OR_NULL(display)) {
			pr_err("%s: Can't alloc display\n", __func__);
			list_for_each_entry(display,
					&(mipi_dsi)->display_list, list) {
				devm_kfree(&pdev->dev, display);
			}
			return -ENOMEM;
		}
		dsi_of_parse_display_timing(display, display_dev_n);
		if (of_property_read_u32(display_dev_n, PROP_DISPLAY_DCCLK,
			&display->dif.dsi.dc_clk_rate)) {
			pr_err("%s: Can't get DC clock rate\n", __func__);
			return -EINVAL;
		}
		/* DSI_CFG default value */
		display->dif.dsi.dsi_cfg_reg = BITFLDS(EXR_DSI_CFG_VSYNC, 1) |
					       BITFLDS(EXR_DSI_CFG_PSYNC, 1);
		ret = of_property_read_u32(
					   display_dev_n, PROP_DISPLAY_RAMLESS,
					   &value);
		if (ret || value)
			display->dif.dsi.mode = DSI_VIDEO;
		else
			display->dif.dsi.mode = DSI_CMD;
		ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_FPS,
					   &display->fps);
		if (ret)
			display->fps = 60;
		ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_LANES,
					   &display->dif.dsi.nblanes);
		if (ret)
			display->dif.dsi.nblanes = 4;
		ret = of_property_read_u32(
					   display_dev_n, PROP_DISPLAY_EOT,
					   &value);
		if (ret || value)
			display->dif.dsi.dsi_cfg_reg |=
				BITFLDS(EXR_DSI_CFG_EOT, 1);
		ret = of_property_read_u32(
					   display_dev_n,
					   PROP_DISPLAY_GATE, &value);
		if (ret || value)
			display->dif.dsi.dsi_cfg_reg |=
				BITFLDS(EXR_DSI_CFG_GATE, 1);
		ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_PREINIT,
					   &display->dif.dsi.display_preinit);
		if (ret)
			display->dif.dsi.display_preinit = 0;
			ret = of_property_read_string(
						      display_dev_n,
						      PROP_DISPLAY_VIDEOMODE,
						      &string);
		if (ret) {
			display->dif.dsi.video_mode = DSI_BURST;
		} else if (!strcmp("active", string)) {
			display->dif.dsi.video_mode = DSI_ACTIVE;
		} else if (!strcmp("pulses", string)) {
			display->dif.dsi.video_mode = DSI_PULSES;
		} else if (!strcmp("events", string)) {
			display->dif.dsi.video_mode = DSI_EVENTS;
		} else if (!strcmp("burst", string)) {
			display->dif.dsi.video_mode = DSI_BURST;
		} else {
			display->dif.dsi.video_mode = DSI_BURST;
			pr_info("%s: Unknown dsi video mode type %s\n",
				__func__, string);
		}
		ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_VIDEOID,
					   &display->dif.dsi.id);
		if (ret)
			display->dif.dsi.id = 0;
		for_each_child_of_node(display_dev_n, child) {
			if (!strcmp(child->name, CMD_LIST_INIT)) {
				ret = dsi_of_parse_display_msglist(pdev, child,
					&display->msgs_init);
			} else if (!strcmp(child->name, CMD_LIST_UPDATE)) {
				ret = dsi_of_parse_display_msglist(pdev, child,
					&display->msgs_update);
			} else if (!strcmp(child->name, GPIO_LIST_POWER_ON)) {
				ret = dsi_of_parse_display_gpiolist(pdev, child,
					&display->gpios_power_on);
			} else if (!strcmp(child->name, GPIO_LIST_POWER_OFF)) {
				ret = dsi_of_parse_display_gpiolist(pdev, child,
					&display->gpios_power_off);
			} else if (!strcmp(child->name, CMD_LIST_SLEEP_IN)) {
				ret = dsi_of_parse_display_msglist(pdev, child,
					&display->msgs_sleep_in);
			} else if (!strcmp(child->name, CMD_LIST_SLEEP_OUT)) {
				ret = dsi_of_parse_display_msglist(pdev, child,
					&display->msgs_sleep_out);
			} else if (!strcmp(child->name, PANEL_DETECT_NODE)) {
				ret = dsi_of_parse_panel_detect_node(pdev,
					child, &display->id_detect);
			} else if (!strcmp(child->name, DISPLAY_TIMINGS_NODE)) {
				continue;
			} else {
				pr_info("%s: In node %s, unexpected child %s !\n",
					__func__, display_dev_n->name,
					child->name);
			}
			if (ret) {
				pr_info("%s: Node %s parsing failed %d\n",
				 __func__, child->name, ret);
			}
		};
		list_add_tail(&display->list, &(mipi_dsi)->display_list);
	}


	list_for_each_entry(display_curr, &(mipi_dsi)->display_list, list) {
		if (mipi_dsi->screen.index < 0 ||
		    mipi_dsi->screen.index == index++) {
			mipi_dsi->cur_display = display_curr;
			break;
		}
	}
	return 0;
}

