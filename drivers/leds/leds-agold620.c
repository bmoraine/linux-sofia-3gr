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
#include <linux/delay.h>
#define XGOLD_LED_MODULE_NAME "leds-agold620"
#define PROP_BL_GPIO_ENABLE	"intel,bl-gpio-enable"

/* Offset Registers */
#define LED_K1_CONTROL	0x000
#define LED_K1MAX	0x008
#define LED_K2MAX	0x018
#define LED_K2_CONTROL	0x00C
#define LED_CTRL	0x02C
#define LED_STBY	0x040
#define LED_K3MAX	0x04C
#define LED_K4_CONTROL	0x050
#define SAFE_LED_CTRL	0x154

/* LED BL Normal */
#if 0
#define SCU_K2_VAL 0x143
#define SCU_K1MAX_VAL 0x120
#define SCU_K2MAX_VAL 0xFFFF
#define SCU_LED_UP 0x10104
#define SCU_LED_DOWN 0x200
#define SCU_SAFE_LED_UP 0x12
#else
#define SCU_K2_VAL (led->config.k2)
#define SCU_K1MAX_VAL (led->config.k1max)
#define SCU_K2MAX_VAL (led->config.k2max)
#define SCU_LED_UP (led->config.up)
#define SCU_LED_DOWN (led->config.down)
#define SCU_SAFE_LED_UP (led->config.safe)
#endif

#define SCU_LED_UP_CMP_100mv	0x10106
#define SCU_LED_UP_CMP_200mv	0x10116

#define XGOLD_LED_USE_SAFE_CTRL		BIT(0)
#define XGOLD_LED_USE_SECURE_IO_ACCESS	BIT(1)
#define XGOLD_LED_USE_NATIVE_IO_ACCESS	BIT(2)

#define TOTAL_CLK_20K_IN_26M 1310
/* In micro secs */
#define DELAY_TIME_FOR_LED_CTRL_200MV 25

/* Micro seconds to Nano seconds*/
#define US_TO_NS(value) ktime_set(value/1000000, (value % 1000000)*1000)

static int led_ctrl = SCU_LED_UP_CMP_100mv;

static void led_write32(struct xgold_led_data *led,
				uint16_t offset, uint32_t val)
{
	int32_t ret = 0;
	if (led->flags & XGOLD_LED_USE_SECURE_IO_ACCESS) {
		ret = mv_svc_reg_write_only(led->physio + offset, val,
						0xFFFFFFFF);
		WARN_ON(ret);
	} else {
		iowrite32(val, (char *)led->mmio + offset);
	}
}

static enum hrtimer_restart agold620_led_hrtimer_callback(struct hrtimer *timer)
{
	struct xgold_led_data *led = container_of(timer,
					struct xgold_led_data, timer);
	pr_debug("%s -->\n", __func__);
	led_ctrl = SCU_LED_UP_CMP_200mv;
	led_write32(led, LED_CTRL, SCU_LED_DOWN);
	led_write32(led, LED_CTRL, led_ctrl);
	return HRTIMER_NORESTART;
}

static inline void agold620_led_on(struct device *dev)
{
	struct xgold_led_data *led = dev_get_drvdata(dev);
	int32_t intensity = SCALING_INTENSITY(led->led_brightness);
	int freerun_mode = 0;
	int generation_mode = SCU_SAFE_LED_UP & 0x03;

	if (generation_mode == 0x01)
		freerun_mode = 1;
	led_write32(led, LED_CTRL, SCU_LED_DOWN);
	pr_debug("%s -->\n", __func__);
	if (freerun_mode) {
		int32_t val = (TOTAL_CLK_20K_IN_26M * intensity)/100;
		mdelay(10);
		led_write32(led, LED_K2_CONTROL, TOTAL_CLK_20K_IN_26M - val);
		led_write32(led, LED_K1_CONTROL, val);
	} else {
		int32_t val = (SCU_K2_VAL * 100)/intensity;
		mdelay(10);
		led_write32(led, LED_K2_CONTROL, val);
	}
	led_write32(led, LED_K1MAX, SCU_K1MAX_VAL);
	led_write32(led, LED_K2MAX, SCU_K2MAX_VAL);
	if (led->flags & XGOLD_LED_USE_SAFE_CTRL)
		led_write32(led, SAFE_LED_CTRL, SCU_SAFE_LED_UP);
	led_write32(led, LED_CTRL, SCU_LED_UP);
}

static inline void agold620_led_off(struct device *dev)
{
	struct xgold_led_data *led = dev_get_drvdata(dev);
	pr_debug("%s -->\n", __func__);
	led_write32(led, LED_CTRL, SCU_LED_DOWN);
}

static int32_t agold620_led_hwinit(struct device *dev)
{
	int32_t ret = 0;
	struct xgold_led_data *led = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = led->pm_platdata;
	pr_debug("%s -->\n", __func__);
	spin_lock_init(&led->timer_lock);
	ret = device_state_pm_set_class(dev, pm_platdata->pm_user_name);
	hrtimer_init(&led->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	led->timer.function = agold620_led_hrtimer_callback;
	led_write32(led, LED_CTRL, SCU_LED_DOWN);
	return ret;
}

static int32_t agold620_led_set_clk(struct device *dev, bool on)
{
	int32_t ret = 0;
	unsigned long flags;
	struct xgold_led_data *led = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = led->pm_platdata;
	pr_debug("%s(%d) -->\n", __func__, on);
	led->set_gpio(dev, on);
	if (on) {
		ret = device_state_pm_set_state_by_name(dev,
			pm_platdata->pm_state_D0_name);
		if (ret) {
			dev_err(dev, "Power On Failed\n");
			return -EINVAL;
		}
retry:
		spin_lock_irqsave(&led->timer_lock, flags);
		if (hrtimer_try_to_cancel(&led->timer) < 0) {
			spin_unlock_irqrestore(&led->timer_lock, flags);
			goto retry;
		}
		hrtimer_start(&led->timer,
				US_TO_NS(DELAY_TIME_FOR_LED_CTRL_200MV),
				HRTIMER_MODE_REL);
		spin_unlock_irqrestore(&led->timer_lock, flags);
	} else {
		ret = device_state_pm_set_state_by_name(dev,
			pm_platdata->pm_state_D3_name);
		if (ret) {
			dev_err(dev, "Power Off Failed\n");
			return -EINVAL;
		}
		led_ctrl = SCU_LED_UP_CMP_100mv;
	}
	return 0;
}

static int32_t agold620_of_gpio(struct device *dev)
{
	struct xgold_led_data *led = dev_get_drvdata(dev);
	struct device_node *np = led->np;
	enum of_gpio_flags gpio_flags;
	pr_debug("%s -->\n", __func__);
	led->gpio = of_get_named_gpio_flags(np,
					PROP_BL_GPIO_ENABLE, 0, &gpio_flags);
	if (!gpio_is_valid(led->gpio)) {
		dev_dbg(dev, "no gpio resource\n");
		return 0;
	}
	if (gpio_request(led->gpio, "bk_enable"))
		dev_err(dev, "request lcd rst gpio fail\n");
	else {
		dev_dbg(dev, "reseting lcd thru gpio\n");
		led->set_gpio(dev, true);
	}
	return 0;
}

static int32_t agold620_set_gpio(struct device *dev, bool on)
{
	struct xgold_led_data *led = dev_get_drvdata(dev);
	if (gpio_is_valid(led->gpio)) {
		gpio_direction_output(led->gpio, on);
		iowrite32(on ? 0x3 : 0x0, led->cgu_mmio);
	}
	return 0;
}

static int32_t agold620_led_set_backlight(struct device *dev)
{
	struct xgold_led_data *led = dev_get_drvdata(dev);
	unsigned long flags = 0;
	spinlock_t *lock = &led->lock;
	pr_debug("%s(%#x) -->\n", __func__, led->led_brightness);
	mdelay(10);
	spin_lock_irqsave(lock, flags);
	if (led->led_brightness)
		agold620_led_on(dev);
	else
		agold620_led_off(dev);
	spin_unlock_irqrestore(lock, flags);
	return 0;
}

static int32_t agold620_led_probe(struct platform_device *pdev)
{
	struct xgold_led_data *led;
	struct device *dev = &pdev->dev;
	struct resource *bl_res, *cgu_res;
	dev_info(dev, "AGOLD620 backlight driver probed\n");
	led = devm_kzalloc(dev, sizeof(struct xgold_led_data), GFP_KERNEL);
	if (!led) {
		dev_err(dev, "not enough memory for driver data\n");
		return -ENOMEM;
	}
	led->set_clk = agold620_led_set_clk;
	led->init = agold620_led_hwinit;
	led->set_gpio = agold620_set_gpio;
	led->set_backlight = agold620_led_set_backlight;
	led->np = pdev->dev.of_node;
	dev_set_drvdata(dev, led);
	led->pdev = pdev;
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
	agold620_of_gpio(dev);

	return xgold_led_probe(pdev);
}

int32_t agold620_led_remove(struct platform_device *pdev)
{
		return xgold_led_remove(pdev);
}

static const struct of_device_id agold620_led_of_match[] = {
	{
		.compatible = "intel,agold620-led",
	},
	{},
};

static struct platform_driver agold620_led_driver = {
	.driver = {
		.name = XGOLD_LED_MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &xgold_led_pm,
		.of_match_table = agold620_led_of_match,
	},
	.probe = agold620_led_probe,
	.remove = agold620_led_remove,
};

static int32_t __init agold620_led_init(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int32_t ret = device_state_pm_add_class(&bl_pm_class);
	if (ret) {
		pr_err("%s: ERROR adding pm class\n", __func__);
		return ret;
	}
#endif
	return platform_driver_register(&agold620_led_driver);
}

static void __exit agold620_led_exit(void)
{
	platform_driver_unregister(&agold620_led_driver);
}

module_init(agold620_led_init);
module_exit(agold620_led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("AGOLD620 backlight driver");
MODULE_DEVICE_TABLE(of, agold620_led_of_match);
