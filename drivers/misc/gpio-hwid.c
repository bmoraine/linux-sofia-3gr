
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/seq_file.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <misc/gpio-hwid.h>

#define PROP_HWID "intel,gpio-hw-id"

struct hwid_data {
	int hwid0;
	int hwid1;
};

static struct hwid_data _hwid_data;

hwid_t platform_gpio_hwid_get(void)
{
	struct hwid_data *hwid = &_hwid_data;
	u8 id0, id1;

	id0 = gpio_get_value(hwid->hwid0);
	id1 = gpio_get_value(hwid->hwid1);

	return (id0 | id1 << 1);
}
EXPORT_SYMBOL_GPL(platform_gpio_hwid_get);

static int of_get_hwid(struct platform_device *pdev)
{
	struct hwid_data *hwid = &_hwid_data;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	hwid->hwid0 = of_get_named_gpio(np, PROP_HWID, 0);
	hwid->hwid1 = of_get_named_gpio(np, PROP_HWID, 1);

	if (!gpio_is_valid(hwid->hwid0) || !gpio_is_valid(hwid->hwid1)) {
		dev_err(dev, "invalid HWID\n");
		return -ENODEV;
	}

	dev_dbg(dev, "HWID0: %d, HWID1: %d\n", hwid->hwid0, hwid->hwid1);

	if (gpio_request(hwid->hwid0, "hwid0-gpio")) {
		dev_err(dev, "%s: request gpio %d fail!\n", __func__, hwid->hwid0);
		return -EINVAL;
	}
	if (gpio_request(hwid->hwid1, "hwid1-gpio")) {
		dev_err(dev, "%s: request gpio %d fail!\n", __func__, hwid->hwid1);
		return -EINVAL;
	}

	return 0;
}

static ssize_t hwid_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", platform_gpio_hwid_get());
}

static struct device_attribute hwid_attr = {
	.attr = {
		.name = "hwid",
		.mode = S_IRUSR,
	},
	.show = hwid_show,
};

static void hwid_setup_sysfs_attr(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	if (device_create_file(dev, &hwid_attr))
		dev_err(dev, "Unable to create sysfs entry: '%s'\n",
				hwid_attr.attr.name);
}

static int hwid_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device *dev = &pdev->dev;

	err = of_get_hwid(pdev);

	if (err) {
		dev_err(dev, "fail to probe hwid\n");
		return err;
	}

	hwid_setup_sysfs_attr(pdev);

	return err;
}

static int hwid_remove(struct platform_device *pdev)
{
	struct hwid_data *hwid = &_hwid_data;
	if (gpio_is_valid(hwid->hwid0))
		gpio_free(hwid->hwid0);
	if (gpio_is_valid(hwid->hwid1))
		gpio_free(hwid->hwid1);
	return 0;
}

static struct of_device_id hwid_match_table[] = {
	{.compatible = "intel,gpio-hwid" },
	{},
};

static struct platform_driver gpio_hwid_driver = {
	.probe		= hwid_probe,
	.remove		= hwid_remove,
	.driver		= {
		.name = "GPIO_HWID",
		.owner = THIS_MODULE,
		.of_match_table = hwid_match_table,
	},
};

static int __init hwid_init(void)
{
	int ret = 0;
	pr_debug("Intel Generic HWID driver init\n");

	ret = platform_driver_register(&gpio_hwid_driver);

	if (ret < 0) {
		pr_err("%s:unable to add HWID driver\n", __func__);
		return -ENODEV;
	}

	return ret;
}

static void __exit hwid_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&gpio_hwid_driver);
}

module_init(hwid_init);
module_exit(hwid_exit);

MODULE_DESCRIPTION("Intel Generic GPIO HWID driver");
MODULE_LICENSE("GPL");
