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
#include <linux/leds.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <sofia/mv_svc_hypercalls.h>
#include <sofia/vmm_pmic.h>
#include <linux/leds-xgold.h>
#include <linux/leds-pmic.h>

#define XGOLD_LED_MODULE_NAME "leds-pmic"

#if 0
#define PMIC_K2_VAL 0x12a
#define PMIC_K1MAX_HIGH 0x00
#define PMIC_K1MAX_LOW 0xb8
#define PMIC_LED_CTRL_UP 0x83
#define PMIC_LED_CTRL_DOWN 0x80
#define PMIC_LED_CFG_UP 0x0a
#define PMIC_LED_CFG_DOWN 0x04
#else
#define PMIC_K2_VAL (led->config.k2)
#define PMIC_K1MAX_HIGH ((led->config.k1max & 0xFF00) >> 8)
#define PMIC_K1MAX_LOW ((led->config.k1max & 0xFF) >> 0)
#define PMIC_LED_CTRL_UP (led->config.ctrl_up)
#define PMIC_LED_CTRL_DOWN (led->config.ctrl_down)
#define PMIC_LED_CFG_UP (led->config.up)
#define PMIC_LED_CFG_DOWN (led->config.down)
#endif

static inline void pmic_led_on(struct device *dev)
{
	struct xgold_led_data *led = dev_get_drvdata(dev);
	int32_t intensity = SCALING_INTENSITY(led->led_brightness);
	int32_t val = (PMIC_K2_VAL * 100)/intensity;
	pr_debug("%s -->\n", __func__);
	vmm_pmic_reg_write(PMIC_BL_ADDR | LED_K1MAX_HIGH_REG, PMIC_K1MAX_HIGH);
	vmm_pmic_reg_write(PMIC_BL_ADDR | LED_K1MAX_LOW_REG, PMIC_K1MAX_LOW);
	vmm_pmic_reg_write(PMIC_BL_ADDR | LED_K2_HIGH_CTRL_REG,
					(val & 0xFF00) >> 8);
	vmm_pmic_reg_write(PMIC_BL_ADDR | LED_K2_LOW_CTRL_REG, (val & 0xFF));
	vmm_pmic_reg_write(PMIC_BL_ADDR | LED_CFG_REG, PMIC_LED_CFG_UP);
	vmm_pmic_reg_write(PMIC_BL_ADDR | LED_CTRL_REG, PMIC_LED_CTRL_UP);
}

static inline void pmic_led_off(struct device *dev)
{
	struct xgold_led_data *led = dev_get_drvdata(dev);
	pr_debug("%s -->\n", __func__);
	vmm_pmic_reg_write(PMIC_BL_ADDR | LED_CFG_REG, PMIC_LED_CFG_DOWN);
	vmm_pmic_reg_write(PMIC_BL_ADDR | LED_CTRL_REG, PMIC_LED_CTRL_DOWN);
}

static int32_t pmic_set_gpio(struct device *dev, bool on)
{
	pr_debug("%s -->\n", __func__);
	/* CABC PCL must configured as input */
	vmm_pmic_reg_write(PMIC_DEV1_ADDR | GPIO0P6CTLO_REG, 0x40);
	vmm_pmic_reg_write(PMIC_DEV1_ADDR | GPIO0P6CTLI_REG, 0x00);
	return 0;
}

static int32_t pmic_led_set_backlight(struct device *dev)
{
	struct xgold_led_data *led = dev_get_drvdata(dev);
	pr_debug("%s(%#x) -->\n", __func__, led->led_brightness);
	if (led->led_brightness)
		pmic_led_on(dev);
	else
		pmic_led_off(dev);
	return 0;
}

static int32_t pmic_led_probe(struct platform_device *pdev)
{
	struct xgold_led_data *led;
	struct device *dev = &pdev->dev;
	dev_info(dev, "PMIC backlight driver probed\n");
	led = devm_kzalloc(dev, sizeof(struct xgold_led_data), GFP_KERNEL);
	if (!led) {
		dev_err(dev, "not enough memory for driver data\n");
		return -ENOMEM;
	}
	led->set_gpio = pmic_set_gpio;
	led->set_backlight = pmic_led_set_backlight;
	dev_set_drvdata(&pdev->dev, led);

	return xgold_led_probe(pdev);
}

int32_t pmic_led_remove(struct platform_device *pdev)
{
	return xgold_led_remove(pdev);
}

static const struct of_device_id pmic_led_of_match[] = {
	{
		.compatible = "intel,pmic-led",
	},
	{},
};

static struct platform_driver pmic_led_driver = {
	.driver = {
		.name = XGOLD_LED_MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &xgold_led_pm,
		.of_match_table = pmic_led_of_match,
	},
	.probe = pmic_led_probe,
	.remove = pmic_led_remove,
};

static int32_t __init pmic_led_init(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int32_t ret = device_state_pm_add_class(&bl_pm_class);
	if (ret) {
		pr_err("%s: ERROR adding pm class\n", __func__);
		return ret;
	}
#endif
	return platform_driver_register(&pmic_led_driver);
}

static void __exit pmic_led_exit(void)
{
	platform_driver_unregister(&pmic_led_driver);
}

module_init(pmic_led_init);
module_exit(pmic_led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("PMIC backlight driver");
MODULE_DEVICE_TABLE(of, pmic_led_of_match);
