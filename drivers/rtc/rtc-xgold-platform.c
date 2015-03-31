#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/device.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include "rtc-xgold.h"

#define XGOLD_RTC_ENTER	pr_info("%s, l.%d\n", __func__, __LINE__)

static struct of_device_id xgold_rtc_of_match[] = {
	{ .compatible = "intel,rtc",},
	{ },
};

#ifdef CONFIG_PM
static int xgold_platform_rtc_suspend(struct platform_device *pdev,
							pm_message_t state)
{
	struct rtc_driver_data *rtcd = platform_get_drvdata(pdev);
	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(rtcd->pm_irq);
	return 0;
}

static int xgold_platform_rtc_resume(struct platform_device *pdev)
{
	struct rtc_driver_data *rtcd = platform_get_drvdata(pdev);
	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(rtcd->pm_irq);
	return 0;
}
#else
#define xgold_platform_rtc_suspend NULL
#define xgold_platform_rtc_resume  NULL
#endif


static int __init xgold_platform_rtc_probe(struct platform_device *pdev)
{
	struct rtc_driver_data *rtcd;
	struct device_node *np = pdev->dev.of_node;
	int len;

	rtcd = xgold_rtc_init_driver(&pdev->dev);
	if (IS_ERR(rtcd))
		return PTR_ERR(rtcd);

	/* Are rtc accessors synchronous or not */
	rtcd->async_mode = 0;
	if (of_find_property(np, "intel,async", &len))
		rtcd->async_mode = 1;
	dev_info(&pdev->dev, "rtc access are %shronous\n",
			(rtcd->async_mode) ? "async" : "sync");

	rtcd->base_year = 0;
	if (of_find_property(np, "intel,base_year", &len)) {
		len /= sizeof(u32);
		if (of_property_read_u32_array(np, "intel,base_year",
					(u32 *) &rtcd->base_year, len)) {
			dev_err(&pdev->dev,
				"can't get intel,base_year property from device tree\n"
				);
			return -EINVAL;
		}
	}
	dev_dbg(&pdev->dev, "rtc year base = %d\n", rtcd->base_year);

	if (rtcd->core_ops->probe)
		return rtcd->core_ops->probe(&pdev->dev);

	return 0;
}

static int __exit xgold_platform_rtc_remove(struct platform_device *pdev)
{
	struct rtc_driver_data *rtcd = platform_get_drvdata(pdev);
	if (rtcd->core_ops->remove)
		return rtcd->core_ops->remove(&pdev->dev);

	return 0;
}

static void xgold_platform_rtc_shutdown(struct platform_device *pdev)
{
	struct rtc_driver_data *rtcd = platform_get_drvdata(pdev);

	if (rtcd->core_ops->shutdown)
		rtcd->core_ops->shutdown(&pdev->dev);
}

MODULE_ALIAS("platform:xgold_rtc");
static struct platform_driver xgold_platform_rtc_driver = {
	.probe = xgold_platform_rtc_probe,
	.remove = __exit_p(xgold_platform_rtc_remove),
	.suspend = xgold_platform_rtc_suspend,
	.resume = xgold_platform_rtc_resume,
	.shutdown = xgold_platform_rtc_shutdown,
	.driver = {
		   .name = "xgold_rtc",
		   .owner = THIS_MODULE,
		   .of_match_table = xgold_rtc_of_match,
	},
};

static int __init xgold_platform_rtc_init(void)
{
	return platform_driver_register(&xgold_platform_rtc_driver);
}

static void __exit xgold_platform_rtc_exit(void)
{
	platform_driver_unregister(&xgold_platform_rtc_driver);
}

module_init(xgold_platform_rtc_init);
module_exit(xgold_platform_rtc_exit);

MODULE_LICENSE("GPL");

