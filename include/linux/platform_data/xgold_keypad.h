/*
* Copyright (C) 2011-2013 Intel Mobile Communications GmbH
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

#define XGOLD_KPD_MODULE_NAME		"kpd-xgold"

struct xgold_keypad_platform_data {
	struct matrix_keypad_platform_data *key_cfg;
	struct clk *clk_ahb;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	struct device_pm_platdata *pm_platdata;
#endif
};
