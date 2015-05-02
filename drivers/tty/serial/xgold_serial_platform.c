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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/serial_core.h>
#include <linux/platform_data/serial_xgold.h>

#if defined CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
/* USIF PM states & class & pm ops */
static struct device_state_pm_ops usif_pm_ops = {
	.set_state = xgold_usif_set_pm_state,
	.get_initial_state = xgold_usif_get_initial_state,
};

struct device_state_pm_state usif_pm_states[] = {
	{.name = "enable_26M", },
	{.name = "enable_26M_hperf", },
	{.name = "enable_96M", },
	{.name = "enable_96M_hperf", },
	{.name = "enable_104M", },
	{.name = "enable_104M_hperf", }, /* D0 */
	{.name = "disable", } /* D3 */
};

DECLARE_DEVICE_STATE_PM_CLASS(usif);
#endif

static int xgold_usif_serial_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct uart_usif_xgold_port *uxp;
	struct xgold_usif_platdata *platdata;
	struct console *usif_console = NULL;

	uxp = xgold_usif_add_port(&pdev->dev, &xgold_usif_reg, 0, 0);
	if (IS_ERR_OR_NULL(uxp))
		return -EINVAL;

	platform_set_drvdata(pdev, uxp);

	/* FIXME: Register device pm class */
	platdata = dev_get_platdata(&pdev->dev);
	platdata->pm_platdata = of_device_state_pm_setup(pdev->dev.of_node);
	if (IS_ERR(platdata->pm_platdata)) {
		dev_warn(&pdev->dev, "Missing pm platdata properties\n");
		/* FIXME: for legacy only. Should never be NULL ! */
		platdata->pm_platdata = NULL;
	}

	if (platdata->pm_platdata)
		ret = platform_device_pm_set_class(pdev,
				platdata->pm_platdata->pm_user_name);

	/* Make sure that only the device that acts as console
	 * gets assigned the driver console.
	 */
	usif_console = uxp->drv->cons;
	if (!uxp->is_console)
		uxp->drv->cons = NULL;

	ret = uart_add_one_port(uxp->drv, &uxp->port);

	uxp->drv->cons = usif_console;

	if (ret)
		goto err;


	return ret;

err:
	xgold_usif_remove_port(uxp);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int xgold_usif_serial_remove(struct platform_device *pdev)
{
	struct uart_usif_xgold_port *uxp = platform_get_drvdata(pdev);

	uart_remove_one_port(uxp->drv, &uxp->port);

	xgold_usif_remove_port(uxp);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int xgold_usif_serial_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_usif_xgold_port *uxp = platform_get_drvdata(pdev);

	if (uxp->pm_ops && uxp->pm_ops->suspend)
		return uxp->pm_ops->suspend(dev);

	return 0;
}

static int xgold_usif_serial_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_usif_xgold_port *uxp = platform_get_drvdata(pdev);

	if (uxp->pm_ops && uxp->pm_ops->resume)
		return uxp->pm_ops->resume(dev);

	return 0;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int xgold_usif_pm_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_usif_xgold_port *uxp = platform_get_drvdata(pdev);

	if (uxp->pm_ops && uxp->pm_ops->runtime_suspend)
		return uxp->pm_ops->runtime_suspend(dev);

	return 0;
}

static int xgold_usif_pm_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_usif_xgold_port *uxp = platform_get_drvdata(pdev);

	if (uxp->pm_ops && uxp->pm_ops->runtime_resume)
		return uxp->pm_ops->runtime_resume(dev);

	return 0;
}
#endif

static struct of_device_id xgold_usif_serial_of_match[] = {
	{.compatible = "intel,usif-serial",},
	{},
};

static const struct dev_pm_ops pm_ops = {
#ifdef CONFIG_PM
	SET_SYSTEM_SLEEP_PM_OPS(xgold_usif_serial_suspend,
			xgold_usif_serial_resume)
#endif
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(xgold_usif_pm_runtime_suspend,
			xgold_usif_pm_runtime_resume, NULL)
#endif
};

static struct platform_driver xgold_usif_ser_drv = {
	.probe = xgold_usif_serial_probe,
	.remove = xgold_usif_serial_remove,
	.driver = {
		.name = "serial-usif-xgold",
		.owner = THIS_MODULE,
		.of_match_table = xgold_usif_serial_of_match,
		.pm = &pm_ops,
	},
};

static int __init xgold_usif_init(void)
{
	int ret = 0;
	pr_info("XGOLD USIF serial driver\n");

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	ret = device_state_pm_add_class(&usif_pm_class);
	if (ret)
		return ret;
#endif
	ret = uart_register_driver(&xgold_usif_reg);
	if (ret == 0) {
		ret = platform_driver_register(&xgold_usif_ser_drv);
		if (ret)
			uart_unregister_driver(&xgold_usif_reg);
	}
	return ret;
}

static void __exit xgold_usif_exit(void)
{
	platform_driver_unregister(&xgold_usif_ser_drv);
	uart_unregister_driver(&xgold_usif_reg);
}

module_exit(xgold_usif_exit);
module_init(xgold_usif_init);

MODULE_AUTHOR("Intel");
MODULE_DESCRIPTION("XGOLD USIF serial port driver");
MODULE_LICENSE("GPL");
