/*
* Copyright (C) 2013 Intel Mobile Communications GmbH
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/leds.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <sofia/mv_svc_hypercalls.h>
#include <linux/leds-xgold.h>

static inline int xgold_led_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	int32_t ret = 0;
	struct xgold_led_data *led = dev_get_drvdata(dev);
	pr_debug("%s -->\n", __func__);
	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(led->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	return ret;
}

static void xgold_led_work(struct work_struct *work)
{
	struct xgold_led_data *led =
			container_of(work, struct xgold_led_data, work);
	struct platform_device *pdev = led->pdev;
	struct device *dev = &pdev->dev;
	static bool power_on; /* FIXME */
	pr_debug("%s --> power-on:%d - brightness:%#x\n", __func__,
					power_on, led->led_brightness);
	if (led->led_brightness) {
		if (!power_on) {
			if (led->set_clk && led->set_clk(dev, true))
				dev_err(dev, " set_clk failed\n");
		}
		if (led->set_backlight && led->set_backlight(dev))
			dev_err(dev, " set_backlight failed\n");
		power_on = true;
	} else {
		if (led->set_backlight && led->set_backlight(dev))
			dev_err(dev, " set_backlight failed\n");
		if (power_on) {
			if (led->set_clk && led->set_clk(dev, false))
				dev_err(dev, " set_clk failed\n");
		}
		power_on = false;
	}
}

static void xgold_led_brightness_set(struct led_classdev *led_cdev,
					enum led_brightness brightness)
{
	struct xgold_led_data *led =
		container_of(led_cdev, struct xgold_led_data, led_cdev);
	unsigned long flags;
	pr_debug("%s -->\n", __func__);
	spin_lock_irqsave(&led->lock, flags);
	led->led_brightness = brightness;
	schedule_work(&led->work);
	spin_unlock_irqrestore(&led->lock, flags);
	return;
}

#define PROP_LED_K2 "intel,led-k2"
#define PROP_LED_K1MAX "intel,led-k1max"
#define PROP_LED_K2MAX "intel,led-k2max"
#define PROP_LED_UP "intel,led-up"
#define PROP_LED_DOWN "intel,led-down"
#define PROP_LED_CTRL_UP "intel,led-ctrl-up"
#define PROP_LED_CTRL_DOWN "intel,led-ctrl-down"
#define PROP_LED_SAFE "intel,led-safe"

int32_t xgold_led_get_conf(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct xgold_led_data *led = platform_get_drvdata(pdev);
	struct xgold_led_config *conf = &led->config;
	of_property_read_u32(np, PROP_LED_K2, &conf->k2);
	of_property_read_u32(np, PROP_LED_K1MAX, &conf->k1max);
	of_property_read_u32(np, PROP_LED_K2MAX, &conf->k2max);
	of_property_read_u32(np, PROP_LED_UP, &conf->up);
	of_property_read_u32(np, PROP_LED_DOWN, &conf->down);
	of_property_read_u32(np, PROP_LED_CTRL_UP, &conf->ctrl_up);
	of_property_read_u32(np, PROP_LED_CTRL_DOWN, &conf->ctrl_down);
	of_property_read_u32(np, PROP_LED_SAFE, &conf->safe);
	pr_debug("%s: %s:%#x\n", __func__, PROP_LED_K2, conf->k2);
	pr_debug("%s: %s:%#x\n", __func__, PROP_LED_K1MAX, conf->k1max);
	pr_debug("%s: %s:%#x\n", __func__, PROP_LED_K2MAX, conf->k2max);
	pr_debug("%s: %s:%#x\n", __func__, PROP_LED_UP, conf->up);
	pr_debug("%s: %s:%#x\n", __func__, PROP_LED_DOWN, conf->down);
	pr_debug("%s: %s:%#x\n", __func__, PROP_LED_CTRL_UP, conf->ctrl_up);
	pr_debug("%s: %s:%#x\n", __func__, PROP_LED_CTRL_DOWN, conf->ctrl_down);
	pr_debug("%s: %s:%#x\n", __func__, PROP_LED_SAFE, conf->safe);
	return 0;
}

int32_t xgold_led_probe(struct platform_device *pdev)
{
	int32_t ret = 0;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *bl_res, *cgu_res;
	struct xgold_led_data *led = platform_get_drvdata(pdev);
	pr_debug("%s -->\n", __func__);

	/* Fill-up xgold_led_data structure */
	led->pdev = pdev;
	led->np = np;
	led->led_brightness = LED_HALF;
	spin_lock_init(&led->lock);
	led->led_cdev.name = "lcd-backlight";
	led->led_cdev.brightness = LED_HALF;
	led->led_cdev.brightness_set = xgold_led_brightness_set;

	if (led_classdev_register(NULL, &led->led_cdev)) {
		dev_err(dev, "unable to register with Leds class\n");
		return -EINVAL;
	}

	/* Parsing DT */
	led->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(led->pm_platdata))
		dev_dbg(dev, "no state pm defined\n");

	if (of_find_property(np, "intel,vmm-secured-access", NULL)) {
		dev_dbg(&pdev->dev, "using secure access\n");
		led->flags |= XGOLD_LED_USE_SECURE_IO_ACCESS;
	} else {
		dev_dbg(&pdev->dev, "using native access\n");
		led->flags |= XGOLD_LED_USE_NATIVE_IO_ACCESS;
	}

	/* Get io resources */
	bl_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmu-bl");
	if (bl_res) {
		dev_dbg(dev, "HW resources available\n");
		led->physio = bl_res->start;
		led->mmio = devm_ioremap(dev, bl_res->start,
				resource_size(bl_res));
		if (!led->mmio) {
			dev_err(dev, "IO remap operation failed\n");
			return -ENODEV;
		}
	} else
		dev_dbg(dev, "no HW resources available\n");

	cgu_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cgu-bl");
	if (cgu_res) {
		dev_dbg(dev, "CGU HW resources available\n");
		led->cgu_physio = cgu_res->start;
		led->cgu_mmio = devm_ioremap(dev, cgu_res->start,
						resource_size(cgu_res));
		if (!led->cgu_mmio) {
			dev_err(dev, "CGU IO remap operation failed\n");
			return -ENODEV;
		}
	} else
		dev_dbg(dev, "no CGU HW resources available\n");

	if (of_find_property(np, "intel,flags-use-safe-ctrl", NULL)) {
		dev_dbg(dev, "safe-ctrl enabled\n");
		led->flags |= XGOLD_LED_USE_SAFE_CTRL;
	}

	 if (xgold_led_get_conf(pdev)) {
			dev_err(dev, "no backlight config availale\n");
			return -ENODEV;
	}

	/* Pinctrl */
	led->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(led->pinctrl)) {
		dev_err(dev, "could not get pinctrl\n");
		return -EINVAL;
	}

	led->pins_default = pinctrl_lookup_state(led->pinctrl,
			PINCTRL_STATE_DEFAULT);
	if (IS_ERR(led->pins_default))
		dev_dbg(dev, "could not get default pinstate\n");

	led->pins_sleep = pinctrl_lookup_state(led->pinctrl,
			PINCTRL_STATE_SLEEP);
	if (IS_ERR(led->pins_sleep))
		dev_dbg(dev, "could not get sleep pinstate\n");

	led->pins_inactive = pinctrl_lookup_state(led->pinctrl,
			"inactive");
	if (IS_ERR(led->pins_inactive))
		dev_dbg(dev, "could not get inactive pinstate\n");

	xgold_led_set_pinctrl_state(&pdev->dev, led->pins_default);

	if (led->init)
		ret = led->init(dev);
	if (led->set_clk)
		ret |= led->set_clk(dev, true);
	if (led->set_gpio)
		ret = led->set_gpio(dev, true);
	if (led->set_backlight)
		ret |= led->set_backlight(dev);

	if (ret < 0) {
		dev_err(dev, "xgold led init failed\n");
		return -EINVAL;
	}
	INIT_WORK(&led->work, xgold_led_work);
	return 0;
}

int32_t xgold_led_remove(struct platform_device *pdev)
{
	struct xgold_led_data *led = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int32_t ret = 0;
	pr_debug("%s: -->\n", __func__);
	xgold_led_set_pinctrl_state(dev, led->pins_inactive);
	led->set_clk(dev, false);
	platform_set_drvdata(pdev, NULL);
	cancel_work_sync(&led->work);
	led_classdev_unregister(&led->led_cdev);
	ret = led->exit(dev);
	return ret;
}

#ifdef CONFIG_PM
static int32_t xgold_led_suspend(struct device *dev)
{
	struct xgold_led_data *led = dev_get_drvdata(dev);
	pr_debug("%s: -->\n", __func__);
	xgold_led_set_pinctrl_state(dev, led->pins_sleep);
	if (led->set_clk)
		return led->set_clk(dev, false);
	else
		return 0;
}

static int32_t xgold_led_resume(struct device *dev)
{
	struct xgold_led_data *led = dev_get_drvdata(dev);
	pr_debug("%s: -->\n", __func__);
	xgold_led_set_pinctrl_state(dev, led->pins_default);
	return 0;
}
#else
#define xgold_led_suspend		NULL
#define xgold_led_resume		NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops xgold_led_pm = {
	.suspend = xgold_led_suspend,
	.resume = xgold_led_resume,
};
