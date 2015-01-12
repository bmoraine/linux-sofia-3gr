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

#ifndef __XGOLD_MIPI_DSI_H__
#define __XGOLD_MIPI_DSI_H__

#include <linux/rockchip_screen.h>
#include "dsi_display.h"

struct xgold_mipi_dsi_device {
	struct device *dev;
	struct rockchip_screen screen;
	bool sys_state;
	struct dsi_display display;
};

#endif
