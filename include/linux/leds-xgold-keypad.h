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

/* Scalling of intensity */
#define DRIVER_MAX_INTENSITY    20
#define ANDROID_MAX_INTENSITY   LED_FULL
#define SCALING_INTENSITY(val) \
		((DRIVER_MAX_INTENSITY * val)/ANDROID_MAX_INTENSITY)

#define XGOLD_LED_USE_SAFE_CTRL		BIT(0)
#define XGOLD_LED_USE_SECURE_IO_ACCESS	BIT(1)
#define XGOLD_LED_USE_NATIVE_IO_ACCESS	BIT(2)
#define XGOLD_LED_CORE_SUSPENDRESUME	BIT(3)

struct xgold_keypad_led_config {
	int32_t ctrl_up;
	int32_t ctrl_down;
	int32_t cfg_down;
	int32_t cfg_time_step;
};

struct xgold_keypad_led_data {
	/* Callbacks */
	int32_t (*init)(struct device *dev);
	int32_t (*exit)(struct device *dev);
	int32_t (*set_clk)(struct device *dev, bool on);
	int32_t (*set_gpio)(struct device *dev, bool on);
	int32_t (*set_backlight)(struct device *dev);

	struct platform_device *pdev;
	struct device_node *np;

	struct led_classdev led_cdev;
	struct work_struct work;
	enum led_brightness led_brightness;
	spinlock_t lock;
	struct hrtimer timer;
	spinlock_t timer_lock;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#endif
	unsigned long flags;
	/* Pinctrl */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	int32_t gpio;
	struct xgold_keypad_led_config config;
};

int32_t xgold_keypad_led_probe(struct platform_device *pdev);
int32_t xgold_keypad_led_remove(struct platform_device *pdev);
extern const struct dev_pm_ops xgold_keypad_led_pm;
