/*
 * rockchip display system driver
 *
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

#include <linux/platform_device.h>
#include "rockchip_disp_drv.h"
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_platform.h>
#endif
#ifdef CONFIG_ROCKCHIP_IOMMU
#include <linux/rockchip_iovmm.h>
#endif

/* platform device pointer for rockchip display device. */
static struct platform_device *rockchip_disp_pdev;

#ifdef CONFIG_ROCKCHIP_IOMMU

/* rockchip_disp_get_sysmmu_device()
 * @compt: dts device compatible name
 * return:
 *	success: pointer to the device inside of platform device
 *	fail: NULL
 */
struct device *rockchip_disp_get_sysmmu_device(const char *compt)
{
	struct device_node *dev_node = NULL;
	struct platform_device *plat_dev = NULL;
	struct device *dev = NULL;

	dev_node = of_find_compatible_node(NULL, NULL, compt);
	if (!dev_node) {
		pr_err("can't find device node %s \r\n", compt);
		return NULL;
	}

	plat_dev = of_find_device_by_node(dev_node);
	if (!plat_dev) {
		pr_err("can't find platform device in device node %s \r\n",
		       compt);
		return  NULL;
	}
	dev = &plat_dev->dev;

	return dev;
}
#endif

#ifdef CONFIG_IOMMU_API
void rockchip_disp_platform_set_sysmmu(struct device *sysmmu,
		struct device *dev)
{
	dev->archdata.iommu = sysmmu;
}
#else
void rockchip_disp_platform_set_sysmmu(struct device *sysmmu,
		struct device *dev)
{
}
#endif

static int rockchip_disp_platform_probe(struct platform_device *pdev)
{
	return 0;
}

static int rockchip_disp_platform_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver rockchip_disp_platform_driver = {
	.probe		= rockchip_disp_platform_probe,
	.remove		= rockchip_disp_platform_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "rockchip-disp",
	},
};

static int __init rockchip_disp_init(void)
{
	int ret;

#ifdef CONFIG_FB_ROCKCHIP
	ret = platform_driver_register(&rockchip_fb_driver);
	if (ret < 0)
		goto out_fb;

	ret = platform_driver_register(&rockchip_screen_driver);
	if (ret < 0)
		goto out_screen;
#endif

#ifdef CONFIG_LVDS_NANOSILICON
	ret = platform_driver_register(&nanosilicon_lvds_driver);
	if (ret < 0)
		goto out_lvds;
#endif

#ifdef CONFIG_XGOLD_MIPI_DSI
	ret = platform_driver_register(&xgold_mipi_dsi_driver);
	if (ret < 0)
		goto out_mipi_dsi;
#endif

#ifdef CONFIG_ROCKCHIP_VOP
	ret = platform_driver_register(&rockchip_vop_driver);
	if (ret < 0)
		goto out_vop;
#endif

	ret = platform_driver_register(&rockchip_disp_platform_driver);
	if (ret < 0)
		goto out_disp;

	rockchip_disp_pdev = platform_device_register_simple(
			"rockchip-disp", -1, NULL, 0);
	if (IS_ERR(rockchip_disp_pdev)) {
		ret = PTR_ERR(rockchip_disp_pdev);
		goto out;
	}

	return 0;

out:
	platform_driver_unregister(&rockchip_disp_platform_driver);

out_disp:

#ifdef CONFIG_ROCKCHIP_VOP
	platform_driver_unregister(&rockchip_vop_driver);
out_vop:
#endif

#ifdef CONFIG_XGOLD_MIPI_DSI
	platform_driver_unregister(&xgold_mipi_dsi_driver);
out_mipi_dsi:
#endif

#ifdef CONFIG_LVDS_NANOSILICON
	platform_driver_unregister(&nanosilicon_lvds_driver);
out_lvds:
#endif

#ifdef CONFIG_FB_ROCKCHIP
	platform_driver_unregister(&rockchip_screen_driver);
out_screen:
	platform_driver_unregister(&rockchip_fb_driver);
out_fb:
#endif
	return ret;
}

static void __exit rockchip_disp_exit(void)
{
	platform_device_unregister(rockchip_disp_pdev);

	platform_driver_unregister(&rockchip_disp_platform_driver);

#ifdef CONFIG_ROCKCHIP_VOP
	platform_driver_unregister(&rockchip_vop_driver);
#endif

#ifdef CONFIG_XGOLD_MIPI_DSI
	platform_driver_unregister(&xgold_mipi_dsi_driver);
#endif

#ifdef CONFIG_LVDS_NANOSILICON
	platform_driver_unregister(&nanosilicon_lvds_driver);
#endif

#ifdef CONFIG_FB_ROCKCHIP
	platform_driver_unregister(&rockchip_screen_driver);
	platform_driver_unregister(&rockchip_fb_driver);
#endif
}

fs_initcall(rockchip_disp_init);
module_exit(rockchip_disp_exit);
