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

#include <linux/module.h>
#include <linux/console.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/platform_device_pm.h>
#endif


struct lcd_drvdata {
	struct device_pm_platdata *pm_platdata;
};

int lcd_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	struct lcd_drvdata *pdata;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	platform_set_drvdata(pdev, pdata);
	/* pm */
	pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pdata->pm_platdata)) {
		dev_err(&pdev->dev, "Error during device state pm init\n");
		return -ENOMEM;
	}

	dev_info(&pdev->dev, "lcd probe\n");

	ret = device_state_pm_set_class(&pdev->dev,
			pdata->pm_platdata->pm_user_name);

	ret = device_state_pm_set_state_by_name(&pdev->dev,
			pdata->pm_platdata->pm_state_D0_name);

	return ret;
}


static int lcd_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct lcd_drvdata *pdata =
	    (struct lcd_drvdata *)platform_get_drvdata(pdev);

	ret = device_state_pm_set_state_by_name(&pdev->dev,
			pdata->pm_platdata->pm_state_D3_name);
	dev_info(&pdev->dev, "lcd remove\n");
	return ret;
}

#ifdef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int lcd_suspend(struct device *dev)
{
	int ret = 0;
	struct lcd_drvdata *pdata =
	    (struct lcd_drvdata *)dev_get_drvdata(dev);

	ret = device_state_pm_set_state_by_name(dev,
			pdata->pm_platdata->pm_state_D3_name);
	dev_info(dev, "lcd suspend\n");
	return ret;
}

static int lcd_resume(struct device *dev)
{
	int ret = 0;
	struct lcd_drvdata *pdata =
	    (struct lcd_drvdata *)dev_get_drvdata(dev);

	ret = device_state_pm_set_state_by_name(dev,
			pdata->pm_platdata->pm_state_D0_name);
	dev_info(dev, "lcd resume\n");
	return ret;
}
#else
#define lcd_suspend NULL
#define lcd_resume NULL
#endif




static struct of_device_id xgold_lcd_of_match[] = {
	{ .compatible = "intel,lcd", },
	{ },
};

static const struct dev_pm_ops lcd_pm_ops = {
	.suspend_late = lcd_suspend,
	.resume_early = lcd_resume,
};

static struct platform_driver lcd_driver = {
	.probe = lcd_probe,
	.remove = lcd_remove,
	.driver = {
		   .name = "lcd",
		   .owner = THIS_MODULE,
		   .pm = &lcd_pm_ops,
		   .of_match_table = xgold_lcd_of_match,
		   },
};

static int __init lcd_init(void)
{
	return platform_driver_register(&lcd_driver);
}

static void __exit lcd_exit(void)
{
	platform_driver_unregister(&lcd_driver);
}

module_init(lcd_init);
module_exit(lcd_exit);

MODULE_DESCRIPTION("lcd dummy driver");
MODULE_ALIAS("lcd");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, xgold_lcd_of_match);
