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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/rockchip_fb.h>
#include <linux/reset.h>
#include <linux/gpio.h>

#include "dsi_device.h"
#include "dsi_hwregs.h"
#include "dsi_dts.h"

static struct xgold_mipi_dsi_device *xgold_mipi_dsi;

static int xgold_mipi_dsi_enable(void)
{
	struct xgold_mipi_dsi_device *mipi_dsi = xgold_mipi_dsi;

	if (unlikely(!mipi_dsi) || mipi_dsi->sys_state)
		return 0;

	dsi_init(mipi_dsi);
	if (mipi_dsi->power_on)
		mipi_dsi->power_on(mipi_dsi);

	dsi_config(mipi_dsi, DIF_TX_DATA);
	if (mipi_dsi->panel_init)
		mipi_dsi->panel_init(mipi_dsi);

	if (mipi_dsi->sleep_out)
		mipi_dsi->sleep_out(mipi_dsi);

	dsi_config(mipi_dsi, DIF_TX_PIXELS);
	mipi_dsi->sys_state = true;

	return 0;
}

static int xgold_mipi_dsi_disable(void)
{
	struct xgold_mipi_dsi_device *mipi_dsi = xgold_mipi_dsi;

	if (unlikely(!mipi_dsi) || !mipi_dsi->sys_state)
		return 0;

	dsi_config(mipi_dsi, DIF_TX_DATA);
	if (mipi_dsi->sleep_in)
		mipi_dsi->sleep_in(mipi_dsi);

	dsi_stop(mipi_dsi);
	if (mipi_dsi->power_off)
		mipi_dsi->power_off(mipi_dsi);


	mipi_dsi->sys_state = false;

	return 0;
}

static int xgold_mipi_dsi_detect_panel(void)
{
	struct xgold_mipi_dsi_device *mipi_dsi = xgold_mipi_dsi;
	int panel_id = -1, temp_id = -1;
	struct dsi_display *display;
	unsigned char *id = NULL;
	int ret = 0;

	list_for_each_entry(display, &(mipi_dsi)->display_list, list) {
		temp_id++;
		mipi_dsi->cur_display = display;

		if (!display->id_detect ||
		    display->id_detect->method == DETECT_METHOD_UNKNOWN ||
		    !display->id_detect->id_verification) {
			panel_id = temp_id;
			dev_err(mipi_dsi->dev,
				"not found panel detection/method/id\n");
			break;
		}

		id = (unsigned char *)devm_kzalloc(mipi_dsi->dev,
				  display->id_detect->id_length * sizeof(u8),
				  GFP_KERNEL);

		if (!id) {
			dev_err(mipi_dsi->dev, "kzalloc id failed\n");
			break;
		}

		if (mipi_dsi->power_on)
			mipi_dsi->power_on(mipi_dsi);

		if (display->id_detect->method == DETECT_METHOD_GPIO) {
			if (mipi_dsi->gpio_id0) {
				gpio_direction_input(mipi_dsi->gpio_id0);
				id[0] = gpio_get_value(mipi_dsi->gpio_id0);
			}
			if (mipi_dsi->gpio_id1) {
				gpio_direction_input(mipi_dsi->gpio_id1);
				id[1] = gpio_get_value(mipi_dsi->gpio_id1);
			}
		} else if (display->id_detect->method == DETECT_METHOD_MIPI) {
			dsi_init(mipi_dsi);
			dsi_config(mipi_dsi, DIF_TX_DATA);
			ret = dsi_read_cmd(mipi_dsi,
					   display->id_detect->cmd_type,
					   display->id_detect->cmd_datas,
					   display->id_detect->cmd_length,
					   id, display->id_detect->id_length);
			dsi_stop(mipi_dsi);
			if (ret != display->id_detect->id_length)
				dev_err(mipi_dsi->dev,
					"expect id len %d but get %d bytes\n",
					display->id_detect->id_length, ret);
		}

		if (mipi_dsi->power_off)
			mipi_dsi->power_off(mipi_dsi);

		if (!memcmp(id, display->id_detect->id_verification,
			    display->id_detect->id_length)) {
			devm_kfree(mipi_dsi->dev, id);
			panel_id = temp_id;
			dev_info(mipi_dsi->dev, "use panel source %d\n",
				 panel_id);
			break;
		}
		devm_kfree(mipi_dsi->dev, id);
	}

	return panel_id;
}

static struct rockchip_fb_trsm_ops trsm_mipi_dsi_ops = {
	.enable = xgold_mipi_dsi_enable,
	.disable = xgold_mipi_dsi_disable,
	.detect_panel = xgold_mipi_dsi_detect_panel,
};

static int xgold_mipi_dsi_probe(struct platform_device *pdev)
{
	struct xgold_mipi_dsi_device *mipi_dsi;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "Don't find xgold mipi dsi device tree node.\n");
		return -EINVAL;
	}

	mipi_dsi = devm_kzalloc(&pdev->dev,
				sizeof(struct xgold_mipi_dsi_device),
				GFP_KERNEL);
	if (!mipi_dsi) {
		dev_err(&pdev->dev, "kzalloc xgold_mipi_dsi device failed\n");
		return -ENOMEM;
	}

	mipi_dsi->dev = &pdev->dev;
	rockchip_get_prmry_screen(&mipi_dsi->screen);
	if (mipi_dsi->screen.type != SCREEN_MIPI) {
		dev_err(&pdev->dev, "screen is not SCREEN MIPI!\n");
		devm_kfree(&pdev->dev, mipi_dsi);
		mipi_dsi = NULL;
		return -EINVAL;
	}

	platform_set_drvdata(pdev, mipi_dsi);
	dev_set_name(mipi_dsi->dev, "xgold-mipi_dsi");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "mipi_dsi_phy");
	mipi_dsi->regbase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mipi_dsi->regbase)) {
		dev_err(&pdev->dev, "ioremap xgold mipi_dsi_phy reg failed\n");
		return PTR_ERR(mipi_dsi->regbase);
	}

	mipi_dsi->irq.err = platform_get_irq(pdev, 0);
	if (mipi_dsi->irq.err < 0) {
		dev_err(&pdev->dev, "Cannot find ERR IRQ for XGold MIPI DSI\n");
		return mipi_dsi->irq.err;
	}

	xgold_mipi_dsi = mipi_dsi;
	rockchip_fb_trsm_ops_register(&trsm_mipi_dsi_ops, SCREEN_MIPI);
	if (support_loader_display())
		mipi_dsi->sys_state = true;

	dev_info(&pdev->dev, "XGold MIPI DSI driver probe success\n");
	dsi_of_parse_display(pdev, mipi_dsi);
	dsi_probe(mipi_dsi);
	dsi_irq_probe(mipi_dsi);

	return 0;
}

static int xgold_mipi_dsi_remove(struct platform_device *pdev)
{
	struct dsi_display *display_curr;

	list_for_each_entry(display_curr,
			    &(xgold_mipi_dsi)->display_list, list) {
		devm_kfree(&pdev->dev, display_curr);
	}
	dsi_irq_remove(xgold_mipi_dsi);
	devm_kfree(&pdev->dev, xgold_mipi_dsi);

	xgold_mipi_dsi = NULL;

	return 0;
}

static void xgold_mipi_dsi_shutdown(struct platform_device *pdev)
{
}

#ifdef CONFIG_OF
static const struct of_device_id xgold_mipi_dsi_dt_ids[] = {
	{.compatible = "intel,xgold-mipi_dsi",},
	{}
};
#endif

struct platform_driver xgold_mipi_dsi_driver = {
	.probe = xgold_mipi_dsi_probe,
	.remove = xgold_mipi_dsi_remove,
	.shutdown = xgold_mipi_dsi_shutdown,
	.driver = {
		.name = "xgold-mipi_dsi",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(xgold_mipi_dsi_dt_ids),
#endif
	},
};

