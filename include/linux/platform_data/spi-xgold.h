/*
* Copyright (C) 2012-2013 Intel Mobile Communications GmbH
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

#ifndef _SPI_XGOLD_H
#define _SPI_XGOLD_H

#define XGOLD2XXSPI		0x01
#define XGOLD6XXSPI		0x02
#define XGOLD_SPI_REG_RES_NAME	"register"
#define DRIVER_NAME		"usif-spi"
#define XGOLD_MAX_CS		4

struct xgold_spi_platdata {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct pinctrl_state *pins_reset;
	char *clk_spi_name;
	char *clk_ahb_name;
	struct clk *clock_ahb;
	struct clk *clock_spi;
	const char *dma_tx_name;
	const char *dma_rx_name;
	int *cs_gpios;
	unsigned num_chipselect;
	unsigned hw_type;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#endif
};

#endif
