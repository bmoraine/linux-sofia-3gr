/*
 * Copyright (C) 2014-2015 Rockchip Electronics Co., Ltd.
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
/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
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

#ifndef _ROCKCHIP_DISP_DRV_H_
#define _ROCKCHIP_DISP_DRV_H_

extern struct platform_driver rockchip_fb_driver;
extern struct platform_driver rockchip_screen_driver;
extern struct platform_driver nanosilicon_lvds_driver;
#ifdef CONFIG_XGOLD_MIPI_DSI
extern struct platform_driver xgold_mipi_dsi_driver;
#endif
extern struct platform_driver rockchip_vop_driver;

#ifdef CONFIG_ROCKCHIP_IOMMU
struct device *rockchip_disp_get_sysmmu_device(const char *compt);
void rockchip_disp_platform_set_sysmmu(struct device *sysmmu,
			struct device *dev);
#endif

#endif
