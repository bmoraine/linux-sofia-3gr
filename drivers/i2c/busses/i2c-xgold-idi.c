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
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_device_pm.h>

#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#define IDI_I2C_SUSPEND_DELAY 1000
#endif

#include "i2c-xgold.h"

static int xgold_idi_i2c_probe(struct idi_peripheral_device *pdev,
				const struct idi_device_id *id)
{
	struct xgold_i2c_dev *i2c_dev;
	struct xgold_i2c_algo_data *algo_data;
	struct xgold_i2c_platdata *platdata;
	struct idi_resource *idi_res = &pdev->resources;
	struct resource *res;
	resource_size_t res_size;
	int ret = 0;

	i2c_dev = xgold_i2c_init_driver(&pdev->device);
	if (IS_ERR(i2c_dev))
		return PTR_ERR(i2c_dev);

	dev_set_drvdata(&pdev->device, i2c_dev);
	algo_data = &i2c_dev->algo_data;
	platdata = pdev->device.platform_data;

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

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_enable(&pdev->device);
	/* 1sec delay */
	pm_runtime_set_autosuspend_delay(&pdev->device, IDI_I2C_SUSPEND_DELAY);
#else
	ret = idi_set_power_state_by_name(pdev,
			(char *)platdata->pm_platdata->pm_state_D0_name, true);

	if (ret) {
		dev_err(&pdev->device, "Error during state transition %s\n",
				platdata->pm_platdata->pm_state_D0_name);
		return ret;
	}
#endif

	/* i2c core probe */
	if (i2c_dev->core_ops->probe)
		ret = i2c_dev->core_ops->probe(&pdev->device);

	if (ret)
		goto err_probe;

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_use_autosuspend(&pdev->device);
#endif

	return 0;

err_probe:
	/* TODO free runtime pm ... */

	return ret;
}

static int xgold_idi_i2c_remove(struct idi_peripheral_device *pdev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(&pdev->device);
	struct xgold_i2c_platdata *platdata = dev_get_platdata(&pdev->device);
	int ret = 0;

	if (i2c_dev->core_ops->remove)
		ret = i2c_dev->core_ops->remove(&pdev->device);

	if (ret)
		return ret;

	return idi_set_power_state_by_name(pdev,
			(char *)platdata->pm_platdata->pm_state_D3_name, false);
}

static void xgold_idi_i2c_shutdown(struct idi_peripheral_device *pdev)
{
	struct xgold_i2c_platdata *platdata = dev_get_platdata(&pdev->device);

	idi_set_power_state_by_name(pdev,
			(char *)platdata->pm_platdata->pm_state_D3_name, false);
}

#ifdef CONFIG_PM
static int xgold_idi_i2c_suspend(struct device *dev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	struct xgold_i2c_platdata *platdata = dev_get_platdata(dev);
	struct idi_peripheral_device *pdev = to_idi_peripheral_device(dev);
	int ret = 0;

	if (atomic_cmpxchg(&i2c_dev->is_suspended, 0, 1))
		return 0;

	if (i2c_dev->pm_ops && i2c_dev->pm_ops->suspend)
		ret = i2c_dev->pm_ops->suspend(dev);

	if (ret)
		return ret;

	ret = idi_set_power_state_by_name(pdev,
			(char *)platdata->pm_platdata->pm_state_D3_name, false);

	if (ret) {
		dev_err(dev, "Error during state transition %s\n",
				platdata->pm_platdata->pm_state_D3_name);
		atomic_set(&i2c_dev->is_suspended, 0);
	}

	return ret;
}

static int xgold_idi_i2c_resume(struct device *dev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	struct xgold_i2c_platdata *platdata = dev_get_platdata(dev);
	struct idi_peripheral_device *pdev = to_idi_peripheral_device(dev);
	int ret = 0;

	if (!atomic_cmpxchg(&i2c_dev->is_suspended, 1, 0))
		return 0;

	ret = idi_set_power_state_by_name(pdev,
			(char *)platdata->pm_platdata->pm_state_D0_name, true);

	if (ret) {
		dev_err(dev, "Error during state transition %s\n",
				platdata->pm_platdata->pm_state_D0_name);
		atomic_set(&i2c_dev->is_suspended, 1);
		return ret;
	}

	if (i2c_dev->pm_ops && i2c_dev->pm_ops->resume)
		return i2c_dev->pm_ops->resume(dev);

	return 0;
}

static const struct dev_pm_ops xgold_idi_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(xgold_idi_i2c_suspend,
			xgold_idi_i2c_resume)
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(xgold_idi_i2c_suspend,
			xgold_idi_i2c_resume, NULL)
#endif
};

#define XGOLD_IDI_I2C_PM_OPS (&xgold_idi_i2c_pm_ops)
#else
#define XGOLD_IDI_I2C_PM_OPS NULL
#endif
static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_I2C,
	},

	{ /* end: all zeroes */},
};

static struct idi_peripheral_driver xgold_idi_i2c_driver = {
	.p_type = IDI_I2C,
	.id_table = idi_ids,
	.probe = xgold_idi_i2c_probe,
	.remove = xgold_idi_i2c_remove,
	.shutdown = xgold_idi_i2c_shutdown,
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
