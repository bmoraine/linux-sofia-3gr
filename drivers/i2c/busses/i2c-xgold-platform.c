/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/device.h>
#include <linux/bcd.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/device_state_pm.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/amba/pl08x.h>
#include <linux/i2c.h>
#include <linux/platform_data/i2c-xgold.h>

#include "i2c-xgold.h"


static struct of_device_id xgold_i2c_of_match[] = {
	{ .compatible = "intel,i2c",},
	{ },
};

int xgold_platform_i2c_probe(struct platform_device *pdev)
{
	struct xgold_i2c_dev *i2c_dev;
	struct xgold_i2c_platdata *platdata;
	struct xgold_i2c_algo_data *algo_data;
	struct resource *res;

	i2c_dev = xgold_i2c_init_driver(&pdev->dev);
	if (IS_ERR(i2c_dev))
		return PTR_ERR(i2c_dev);

	algo_data = &i2c_dev->algo_data;
	platdata = pdev->dev.platform_data;

	/* Map registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	if (!devm_request_mem_region(&pdev->dev, res->start,
				resource_size(res), res->name))
		return -EBUSY;

	algo_data->regs = devm_ioremap(&pdev->dev, res->start,
			resource_size(res));
	if (!algo_data->regs)
		return -EBUSY;

	algo_data->regs_phys = res->start;

	/* request DMA channel */
	algo_data->dmach = dma_request_slave_channel(&pdev->dev, "rxtx");

	/* i2c core probe */
	if (i2c_dev->core_ops->probe)
		return i2c_dev->core_ops->probe(&pdev->dev);

	return -EINVAL;
}

static int xgold_platform_i2c_remove(struct platform_device *pdev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(&pdev->dev);
	if (i2c_dev->core_ops->remove)
		return i2c_dev->core_ops->remove(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int xgold_platform_i2c_suspend(struct device *dev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	struct xgold_i2c_platdata *platdata = dev_get_platdata(dev);
	int ret = 0;

	if (i2c_dev->pm_ops && i2c_dev->pm_ops->suspend)
		ret = i2c_dev->pm_ops->suspend(dev);

	if (ret)
		return ret;

	ret = device_state_pm_set_state_by_name(dev,
			platdata->pm_platdata->pm_state_D3_name);

	if (ret)
		dev_err(dev, "Error during state transition %s\n",
				platdata->pm_platdata->pm_state_D3_name);

	return ret;
}

static int xgold_platform_i2c_resume(struct device *dev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	struct xgold_i2c_platdata *platdata = dev_get_platdata(dev);
	int ret = 0;

	if (i2c_dev->pm_ops && i2c_dev->pm_ops->resume)
		ret = i2c_dev->pm_ops->resume(dev);

	if (ret)
		return ret;

	ret = device_state_pm_set_state_by_name(dev,
			platdata->pm_platdata->pm_state_D0_name);

	if (ret)
		dev_err(dev, "Error during state transition %s\n",
				platdata->pm_platdata->pm_state_D0_name);

	return 0;
}

static const struct dev_pm_ops xgold_platform_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(xgold_platform_i2c_suspend,
			xgold_platform_i2c_resume)
};

#define XGOLD_PLAT_I2C_PM_OPS (&xgold_platform_i2c_pm_ops)
#else
#define XGOLD_PLAT_I2C_PM_OPS NULL
#endif

static struct platform_driver xgold_platform_i2c_driver = {
	.probe = xgold_platform_i2c_probe,
	.remove = xgold_platform_i2c_remove,
	.driver = {
		.name = "xgold-i2c",
		.owner = THIS_MODULE,
		.of_match_table = xgold_i2c_of_match,
		.pm = XGOLD_PLAT_I2C_PM_OPS
	},
};

static int __init xgold_platform_i2c_init_driver(void)
{
	return platform_driver_register(&xgold_platform_i2c_driver);
}

static void __exit xgold_platform_i2c_exit_driver(void)
{
	platform_driver_unregister(&xgold_platform_i2c_driver);
}

subsys_initcall(xgold_platform_i2c_init_driver);
module_exit(xgold_platform_i2c_exit_driver);

MODULE_DESCRIPTION("XGold i2c driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, xgold_i2c_of_match);
