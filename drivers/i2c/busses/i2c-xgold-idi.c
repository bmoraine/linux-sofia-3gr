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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/i2c.h>
#include <linux/platform_data/i2c-xgold.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_device_pm.h>

#include "i2c-xgold.h"

static int xgold_idi_i2c_probe(struct idi_peripheral_device *pdev,
				const struct idi_device_id *id)
{
	struct xgold_i2c_dev *i2c_dev;
	struct xgold_i2c_algo_data *algo_data;
	struct idi_resource *idi_res = &pdev->resources;
	struct resource *res;
	resource_size_t res_size;

	i2c_dev = xgold_i2c_init_driver(&pdev->device);
	if (IS_ERR(i2c_dev))
		return PTR_ERR(i2c_dev);

	dev_set_drvdata(&pdev->device, i2c_dev);
	algo_data = &i2c_dev->algo_data;

	/* Map registers */
	res = idi_get_resource_byname(idi_res, IORESOURCE_MEM, "register");
	if (!res) {
		dev_err(&pdev->device,
			"idi_get_resource_byname - failed\n");
		return -ENOENT;
	}

	res_size = resource_size(res);
	if (!devm_request_mem_region(&pdev->device, res->start, res_size,
				res->name))
		return -EBUSY;

	algo_data->regs = devm_ioremap(&pdev->device, res->start, res_size);
	if (!algo_data->regs)
		return -EBUSY;

	/* i2c core probe */
	if (i2c_dev->core_ops->probe)
		return i2c_dev->core_ops->probe(&pdev->device);

	return -EINVAL;
}

static int xgold_idi_i2c_remove(struct idi_peripheral_device *pdev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(&pdev->device);

	if (i2c_dev->core_ops->remove)
		return i2c_dev->core_ops->remove(&pdev->device);

	return 0;
}


#ifdef CONFIG_PM
static int xgold_idi_i2c_suspend(struct device *dev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	struct xgold_i2c_platdata *platdata = dev_get_platdata(dev);
	struct idi_peripheral_device *pdev = to_idi_peripheral_device(dev);
	struct device_state_pm_state *handler;
	int ret = 0;

	if (i2c_dev->pm_ops && i2c_dev->pm_ops->suspend)
		ret = i2c_dev->pm_ops->suspend(dev);

	if (ret)
		return ret;

	handler = idi_peripheral_device_pm_get_state_handler(pdev,
			platdata->pm_platdata->pm_state_D3_name);

	if (!handler)
		return -EINVAL;

	ret = idi_set_power_state(pdev, handler, false);

	if (ret)
		dev_err(dev, "Error during state transition %s\n",
				platdata->pm_platdata->pm_state_D3_name);

	return ret;
}

static int xgold_idi_i2c_resume(struct device *dev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	struct xgold_i2c_platdata *platdata = dev_get_platdata(dev);
	struct idi_peripheral_device *pdev = to_idi_peripheral_device(dev);
	struct device_state_pm_state *handler;
	int ret = 0;

	if (i2c_dev->pm_ops && i2c_dev->pm_ops->resume)
		ret = i2c_dev->pm_ops->resume(dev);

	if (ret)
		return ret;

	handler = idi_peripheral_device_pm_get_state_handler(pdev,
			platdata->pm_platdata->pm_state_D0_name);

	if (!handler)
		return -EINVAL;

	ret = idi_set_power_state(pdev, handler, true);

	if (ret)
		dev_err(dev, "Error during state transition %s\n",
				platdata->pm_platdata->pm_state_D0_name);

	return ret;
}

static const struct dev_pm_ops xgold_idi_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(xgold_idi_i2c_suspend,
			xgold_idi_i2c_resume)
};

#define XGOLD_IDI_I2C_PM_OPS (&xgold_idi_i2c_pm_ops)
#else
#define XGOLD_IDI_I2C_PM_OPS NULL
#endif

static struct idi_peripheral_driver xgold_idi_i2c_driver = {
	.p_type = IDI_I2C,
	.probe = xgold_idi_i2c_probe,
	.remove = xgold_idi_i2c_remove,
	.driver = {
		.name = "xgold-i2c-idi",
		.owner = THIS_MODULE,
		.pm = XGOLD_IDI_I2C_PM_OPS
	},
};

static int __init xgold_idi_i2c_init_driver(void)
{
	return idi_register_peripheral_driver(&xgold_idi_i2c_driver);
}

static void __exit xgold_idi_i2c_exit_driver(void)
{
	idi_unregister_peripheral_driver(&xgold_idi_i2c_driver);
}

late_initcall(xgold_idi_i2c_init_driver);
module_exit(xgold_idi_i2c_exit_driver);
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, xgold_i2c_of_match);
