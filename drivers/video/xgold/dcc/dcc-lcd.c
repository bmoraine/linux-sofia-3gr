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

#include <linux/delay.h>
#include <linux/gpio.h>

#include "dcc-core.h"
#include "dcc-display.h"
#include "dcc-hwregs.h"

void dcc_display_reset(int gpio, struct dcc_display *lcd)
{
	struct display_reset *res =
		list_first_entry_or_null(&lcd->reset->list,
				struct display_reset, list);
	if (!res)
		return;

	if (gpio_request(gpio, "dif_reset")) {
		dcc_err("failed to request gpio set %d\n",
				gpio);
	}

	list_for_each_entry(res, &lcd->reset->list, list) {
		DCC_DBG2("gpio-reset %d %dms\n",
		 res->value, res->mdelay);
			gpio_set_value(gpio, !!res->value);
		mdelay(res->mdelay);
	}
}

int dcc_display_setup(struct dcc_drvdata *pdata)
{
	struct dcc_display *lcd = &pdata->display;
	int err = 0;

	if (pdata->gpio_lcd_bias) {
		if (gpio_request(pdata->gpio_lcd_bias, "gpio_lcd_bias")) {
			dcc_err("failed to request gpio bias set %d\n",
					pdata->gpio_lcd_bias);
		}
		mdelay(pdata->gpio_lcd_bias_msdelay);
	}

	dcc_boot_dbg("Initializing display hardware\n");
	lcd->panel_reset = dcc_display_reset;
	if (!pdata->display_preinit)
		lcd->panel_reset(pdata->gpio_reset, lcd);

	/**
	 * Display interface configuration
	 */
	if (DISPLAY_IS_MIPI_DBI_IF(lcd->dif.type))
		err = dcc_dbi_probe(lcd);
	else if (DISPLAY_IS_MIPI_DSI_IF(lcd->dif.type))
		err = dcc_dsi_probe(lcd);
	else
		goto error;

	if ((lcd->dif_config == NULL) || (lcd->panel_init == NULL))
		goto error;

	if (lcd->dif_init)
		lcd->dif_init(lcd);
	lcd->dif_config(lcd, DIF_TX_DATA);

	lcd->panel_init(lcd);

	lcd->dif_config(lcd, DIF_TX_PIXELS);

	/* switch to GRA */
	gra_write_field(pdata, EXR_DIF_CSREG_GRACMD, 1);

	if (DISPLAY_IS_MIPI_DBI_IF(lcd->dif.type)) {
		gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_CONF);
		dcc_dbi_sync(lcd);
		err = dcc_dbi_set_bitmux(pdata);
		if (0 != err) {
			dcc_err("Error on setting display parameters\n");
			goto error;
		}
		gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_RUN);
	}
	pdata->display_preinit = 0;
	dcc_boot_dbg("Display_Interface loaded successfully!\n");
	return 0;
error:
	dcc_err("Display setup failed\n");
	return -EINVAL;
}

int dcc_display_suspend(struct dcc_drvdata *pdata)
{
	pdata->display.dif_config(&pdata->display, DIF_TX_DATA);
	pdata->display.power_off(&pdata->display);
	pdata->display.dif_stop(&pdata->display);
	gpio_set_value(pdata->gpio_reset, 0);
	gpio_free(pdata->gpio_reset);
	return 0;
}

int dcc_display_sleepin(struct dcc_drvdata *pdata)
{
	pdata->display.dif_config(&pdata->display, DIF_TX_DATA);
	pdata->display.sleep_in(&pdata->display);
	pdata->display.dif_stop(&pdata->display);
	return 0;
}

int dcc_display_sleepout(struct dcc_drvdata *pdata)
{
	pdata->display.dif_init(&pdata->display);
	pdata->display.dif_config(&pdata->display, DIF_TX_DATA);
	pdata->display.sleep_out(&pdata->display);
	pdata->display.dif_config(&pdata->display, DIF_TX_PIXELS);

	return 0;
}
