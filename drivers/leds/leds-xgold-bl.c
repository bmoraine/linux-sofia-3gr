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
#include <sofia/vmm_pmic.h>
#include "leds-pmic.h"

#define TRUE 1
#define FALSE 0

#define XGOLD_LED_BL_MODULE_NAME		"leds-xgold-bl"

#define LED_K1_CONTROL	0x000
#define LED_K1MAX	0x008
#define LED_K2MAX	0x018
#define LED_K2_CONTROL	0x00C
#define LED_CTRL	0x02C
#define LED_STBY	0x040
#define LED_K3MAX	0x04C
#define LED_K4_CONTROL	0x050
#define SAFE_LED_CTRL	0x154

/* LED PMIC */

#define PMIC_K2_VAL		0x12A
#define PMIC_K1MAX_HIGH		0x0
#define PMIC_K1MAX_LOW		0xb8
#define PMIC_LED_CTRL_UP	0x83
#define PMIC_LED_CTRL_DOWN	0x80
#define PMIC_LED_CFG_UP		0xa
#define PMIC_LED_CFG_DOWN	0x04

/* LED BL Normal */
#define SCU_K2_VAL	0x143
#define SCU_K1MAX_VAL	0x120
#define SCU_K2MAX_VAL	0xFFFF
#define SCU_LED_UP	0x10104
#define SCU_LED_DOWN	0x200
#define SCU_SAFE_LED_UP	0x12

/* LED BL LPBL */
#define SCU_K4_VAL		0xFF
#define SCU_K3MAX_VAL		0xFB
#define SCU_LED_STBY_UP		0x303
#define SCU_LED_STBY_DOWN	0x0

#define SCU_LED_UP_CMP_100mv	0x10106
#define SCU_LED_UP_CMP_200mv	0x10116

#define XGOLD_LED_USE_SAFE_CTRL		BIT(0)

#define BL_MODE_ON	\
	do { \
		if (pdata->pmic_bl) {\
			val = (PMIC_K2_VAL * 100)/intensity; \
			vmm_pmic_reg_write(PMIC_BL_ADDR | LED_K1MAX_HIGH_REG,\
					PMIC_K1MAX_HIGH);\
			vmm_pmic_reg_write(PMIC_BL_ADDR | LED_K1MAX_LOW_REG,\
					PMIC_K1MAX_LOW);\
			vmm_pmic_reg_write(PMIC_BL_ADDR | LED_K2_HIGH_CTRL_REG,\
					(val & 0xFF00) >> 8);\
			vmm_pmic_reg_write(PMIC_BL_ADDR | LED_K2_LOW_CTRL_REG,\
					(val & 0xFF));\
			vmm_pmic_reg_write(PMIC_BL_ADDR | LED_CFG_REG,\
					PMIC_LED_CFG_UP);\
			vmm_pmic_reg_write(PMIC_BL_ADDR | LED_CTRL_REG,\
					PMIC_LED_CTRL_UP);\
		} else {\
			val = (SCU_K2_VAL*100)/intensity; \
			led_write32(mmio_base, LED_CTRL, SCU_LED_DOWN); \
			led_write32(mmio_base, LED_K2_CONTROL, val); \
			led_write32(mmio_base, LED_K1MAX, SCU_K1MAX_VAL); \
			led_write32(mmio_base, LED_K2MAX, SCU_K2MAX_VAL); \
			if (pdata->flags & XGOLD_LED_USE_SAFE_CTRL) \
				led_write32(mmio_base, SAFE_LED_CTRL,\
						SCU_SAFE_LED_UP);\
			led_write32(mmio_base, LED_CTRL, SCU_LED_UP); \
		} \
	} while (0);

#define SET_LCD_BL_ON BL_MODE_ON

#define BL_MODE_OFF	\
	do { \
		if (pdata->pmic_bl) { \
			vmm_pmic_reg_write(PMIC_BL_ADDR | LED_CFG_REG,\
					PMIC_LED_CFG_DOWN);\
			vmm_pmic_reg_write(PMIC_BL_ADDR | LED_CTRL_REG,\
					PMIC_LED_CTRL_DOWN);\
		} else {\
			led_write32(mmio_base, LED_CTRL, SCU_LED_DOWN);\
		} \
	} while (0);
#define SET_LCD_BL_OFF BL_MODE_OFF

/*In micro secs*/
#define DELAY_TIME_FOR_LED_CTRL_200MV 25
/* Scalling of intensity */
#define DRIVER_MAX_INTENSITY    100
#define ANDROID_MAX_INTENSITY   255
#define SCALING_INTENSITY(val) \
		((DRIVER_MAX_INTENSITY * val)/ANDROID_MAX_INTENSITY)
/* Micro seconds to Nano seconds*/
#define US_TO_NS(value) ktime_set(value/1000000, (value % 1000000)*1000)

struct xgold_led_bl_device {
	int (*init)(struct device *dev, void *mmio_base);
	void (*exit)(void);
	int (*set_clk)(struct device *dev, bool on);
	int (*set_lcd_backlight)(struct device *dev, void *mmio_base,
					 spinlock_t *lock, int intensity);
	struct platform_device *pdev;
	void __iomem *mmio_base;
	struct led_classdev led_bl_cdev;
	struct work_struct work;
	enum led_brightness led_bl_brightness;
	spinlock_t lock;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#endif
	unsigned long flags;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	bool pmic_bl;
};

static int led_ctrl = SCU_LED_UP_CMP_100mv;

/* Macros for power enable, disable */
#define XG_PM_DISABLE		0x0
#define XG_PM_ENABLE		0x1
#define XG_PM_NOF_STATES	0x2


#if !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
static int xgold_bl_set_pm_state(struct device *,
		struct device_state_pm_state *);
static struct device_state_pm_state *xgold_bl_get_initial_state(
		struct device *);

static struct device_state_pm_ops bl_pm_ops = {
	.set_state = xgold_bl_set_pm_state,
	.get_initial_state = xgold_bl_get_initial_state,
};

/* PM states & class */
static struct device_state_pm_state bl_pm_states[] = {
	{ .name = "disable", }, /* D3 */
	{ .name = "enable", }, /* D0 */
};

DECLARE_DEVICE_STATE_PM_CLASS(bl);

static int xgold_bl_get_pm_state_id(char *name)
{
	int id;

	for (id = XG_PM_DISABLE; id < XG_PM_NOF_STATES; id++) {
		if (!strcmp(name, bl_pm_states[id].name))
			return id;
	}
	return XG_PM_NOF_STATES;
}

static int xgold_bl_set_pm_state_by_num(int state_num)
{
	return 0;
}

static int xgold_bl_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	int state_num;
	int ret = 0;

	state_num = xgold_bl_get_pm_state_id(state->name);
	dev_info(dev, "set pm state (%d) %s\n", state_num, state->name);
	ret = xgold_bl_set_pm_state_by_num(state_num);
	if (ret < 0)
		dev_err(dev, "Unable to set pm state (%d)%s\n",
				state_num, state->name);

	return ret;
}

static struct device_state_pm_state *xgold_bl_get_initial_state(
		struct device *dev)
{
	return &bl_pm_states[XG_PM_DISABLE];
}

#endif


/*
PCL_LOCK_EXCLUSIVE          = 1
PCL_OPER_ACTIVATE          = 0,
#define XGOLD_PINCONF_PACK(_param_, _arg_) ((_param_) << 16 | (_arg_))
static const unsigned long pcl_activate =
		XGOLD_PINCONF_PACK(PCL_LOCK_EXCLUSIVE, PCL_OPER_ACTIVATE);
static const unsigned long pcl_deactivate =
		XGOLD_PINCONF_PACK(PCL_LOCK_EXCLUSIVE, PCL_OPER_DEACTIVATE);
*/
static void led_write32(void *mmio_base, u16 offset, u32 val)
{
	if (mmio_base)
		iowrite32(val, (char *)mmio_base + offset);
}

struct xgold_bl_timer {
	void __iomem *mmio_base;
	struct hrtimer timer;
	spinlock_t lock;
};
struct xgold_bl_timer *bl_timer;

static enum hrtimer_restart xgold_bl_hrtimer_callback(struct hrtimer *timer)
{
	led_ctrl = SCU_LED_UP_CMP_200mv;
	led_write32(bl_timer->mmio_base, LED_CTRL, SCU_LED_DOWN);
	led_write32(bl_timer->mmio_base, LED_CTRL, led_ctrl);
	return HRTIMER_NORESTART;
}

static inline int xgold_led_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	int ret = 0;
	struct xgold_led_bl_device *pdata = dev_get_drvdata(dev);

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	return ret;
}

static int xgold_led_bl_cbinit(struct device *dev , void *mmio_base)
{
	int ret;
	struct xgold_led_bl_device *pdata = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = pdata->pm_platdata;

	bl_timer = kzalloc(sizeof(struct xgold_bl_timer), GFP_KERNEL);
	if (!bl_timer) {
		dev_err(dev, "not enough memory for Timer data\n");
		return -ENOMEM;
	}
	spin_lock_init(&bl_timer->lock);

	if (!pm_platdata || !pm_platdata->pm_state_D3_name
			|| !pm_platdata->pm_state_D0_name) {
		dev_err(dev, "null pointer\n");
		return -EINVAL;
	}

	ret = device_state_pm_set_class(dev, pm_platdata->pm_user_name);

	hrtimer_init(&bl_timer->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bl_timer->timer.function = xgold_bl_hrtimer_callback;
	bl_timer->mmio_base = mmio_base;
	led_write32(mmio_base, LED_CTRL, SCU_LED_DOWN);

	return ret;
}

static void xgold_led_bl_cbexit(void)
{
	kfree(bl_timer);
	return;
}

static int xgold_bl_set_clk(struct device *dev,
					bool on)
{
	int ret;
	unsigned long flags;
	struct xgold_led_bl_device *pdata = dev_get_drvdata(dev);
	struct device_pm_platdata *pm_platdata = pdata->pm_platdata;

	dev_dbg(dev, "%s %s\n", __func__, on ? "ON" : "OFF");
	if (on) {
		ret = device_state_pm_set_state_by_name(dev,
			pm_platdata->pm_state_D0_name);
		if (ret) {
			dev_err(dev, "%s Power On Failed\n", __func__);
			return 0; /* ret; */
		}
retry:
		spin_lock_irqsave(&bl_timer->lock, flags);
		if (hrtimer_try_to_cancel(&bl_timer->timer) < 0) {
			spin_unlock_irqrestore(&bl_timer->lock, flags);
			goto retry;
		}
		hrtimer_start(&bl_timer->timer,
				US_TO_NS(DELAY_TIME_FOR_LED_CTRL_200MV),
				HRTIMER_MODE_REL);
		spin_unlock_irqrestore(&bl_timer->lock, flags);
	} else {
		ret = device_state_pm_set_state_by_name(dev,
			pm_platdata->pm_state_D3_name);
		if (ret) {
			dev_err(dev, "%s Power Off Failed\n", __func__);
			return ret;
		}
		led_ctrl = SCU_LED_UP_CMP_100mv;
	}
	return 0;
}

static int xgold_set_lcd_backlight(struct device *dev,
				void *mmio_base, spinlock_t *lock,
				int intensity)
{
	struct xgold_led_bl_device *pdata = dev_get_drvdata(dev);
	unsigned long flags = 0;
	int val;
	dev_dbg(dev, "%s %d\n", __func__, intensity);

	intensity = SCALING_INTENSITY(intensity);
	if (intensity) {
		if (!pdata->pmic_bl)
			spin_lock_irqsave(lock, flags);
		SET_LCD_BL_ON;
		if (!pdata->pmic_bl)
			spin_unlock_irqrestore(lock, flags);
	} else {
		if (!pdata->pmic_bl)
			spin_lock_irqsave(lock, flags);
		SET_LCD_BL_OFF;
		if (!pdata->pmic_bl)
			spin_unlock_irqrestore(lock, flags);
	}
	return 0;
}

static void xgold_led_bl_work(struct work_struct *work)
{
	struct xgold_led_bl_device *led_bl =
			container_of(work, struct xgold_led_bl_device, work);
	static int power_on = FALSE;
	int ret = 0;

	if (led_bl->led_bl_brightness) {
		if (power_on == FALSE) {
			if (led_bl->set_clk) {
				ret = led_bl->set_clk(
						&led_bl->pdev->dev,
						TRUE);
				if (ret) {
					dev_err(&led_bl->pdev->dev,
					"%s Set Clk Failed\n", __func__);
					return;
				}
			}
			power_on = TRUE;
		}
		if (led_bl->set_lcd_backlight) {
			ret = led_bl->set_lcd_backlight(
					&led_bl->pdev->dev,
					led_bl->mmio_base, &led_bl->lock,
					led_bl->led_bl_brightness);
			if (ret) {
				dev_err(&led_bl->pdev->dev,
				"%s Set led_bl_cdev Failed\n", __func__);
				return;
			}
		}
	} else {
		if (led_bl->set_lcd_backlight) {
			ret = led_bl->set_lcd_backlight(
					&led_bl->pdev->dev,
					led_bl->mmio_base, &led_bl->lock,
					led_bl->led_bl_brightness);
			if (ret) {
				dev_err(&led_bl->pdev->dev,
				 "%s Set led_bl_cdev Failed\n", __func__);
				return;
			}
		}
		if (power_on == TRUE) {
			if (led_bl->set_clk) {
				ret = led_bl->set_clk(
					&led_bl->pdev->dev,
					FALSE);
				if (ret) {
					dev_err(&led_bl->pdev->dev,
					"%s Set Clk Failed\n", __func__);
					return;
				}
			}
			power_on = FALSE;
		}
	}
}

static void xgold_bl_brightness_set(struct led_classdev *led_cdev,
					enum led_brightness brightness)
{
	struct xgold_led_bl_device *led_bl =
	container_of(led_cdev, struct xgold_led_bl_device, led_bl_cdev);
	unsigned long flags;

	spin_lock_irqsave(&led_bl->lock, flags);
	led_bl->led_bl_brightness = brightness;
	schedule_work(&led_bl->work);
	spin_unlock_irqrestore(&led_bl->lock, flags);
	return;
}

static int xgold_led_bl_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct xgold_led_bl_device *led_bl;
	struct device_node *nbl;

	led_bl = kzalloc(sizeof(struct xgold_led_bl_device), GFP_KERNEL);
	if (!led_bl) {
		dev_err(&pdev->dev,
			"not enough memory for driver data\n");
		return -ENOMEM;
	}

	nbl = pdev->dev.of_node;

	led_bl->init = xgold_led_bl_cbinit;
	led_bl->exit = xgold_led_bl_cbexit;
	led_bl->set_clk = xgold_bl_set_clk;
	led_bl->set_lcd_backlight = xgold_set_lcd_backlight;

	led_bl->pdev = pdev;
	led_bl->led_bl_brightness = LED_HALF;
	INIT_WORK(&led_bl->work, xgold_led_bl_work);
	spin_lock_init(&led_bl->lock);

	led_bl->led_bl_cdev.name = "lcd-backlight";
	led_bl->led_bl_cdev.brightness = LED_HALF;
	led_bl->led_bl_cdev.brightness_set = xgold_bl_brightness_set;

	ret = led_classdev_register(NULL, &led_bl->led_bl_cdev);
	if (ret != 0) {
		dev_err(&pdev->dev,
			" unable to register with Leds class\n");
		ret = -EINVAL;
		goto failed_unmap;
	}

	dev_set_drvdata(&pdev->dev, led_bl);

	if (of_device_is_compatible(nbl, "intel,pmic-led")) {
		dev_info(&pdev->dev, "PMIC Backlight driver probed\n");
		led_bl->pmic_bl = true;
		led_bl->set_clk = NULL;

		/* CABC PCL must configured as input */
		vmm_pmic_reg_write(PMIC_DEV1_ADDR | GPIO0P6CTLO_REG, 0x40);
		vmm_pmic_reg_write(PMIC_DEV1_ADDR | GPIO0P6CTLI_REG, 0x00);
		return 0;
	}

	/* device pm */
	led_bl->pm_platdata = of_device_state_pm_setup(nbl);
	if (IS_ERR(led_bl->pm_platdata)) {
		dev_err(&pdev->dev, "Error during device state pm init\n");
		ret = -EINVAL;
		goto failed_free_mem;
	}

	if (!led_bl->pm_platdata ||
			!led_bl->pm_platdata->pm_state_D3_name ||
			!led_bl->pm_platdata->pm_state_D0_name) {
		dev_err(&pdev->dev, "missing PM info\n");
		return -EINVAL;
	}

	led_bl->mmio_base = of_iomap(nbl, 0);
	if (led_bl->mmio_base == NULL) {
		dev_err(&pdev->dev,
			" I/O remap failed\n");
		ret = -EINVAL;
		goto failed_free_mem;
	}

	if (of_find_property(nbl, "intel,flags-use-safe-ctrl", NULL))
		led_bl->flags |= XGOLD_LED_USE_SAFE_CTRL;

	led_bl->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(led_bl->pinctrl))
		goto failed_unmap;

	led_bl->pins_default = pinctrl_lookup_state(led_bl->pinctrl,
			PINCTRL_STATE_DEFAULT);
	if (IS_ERR(led_bl->pins_default))
		dev_err(&pdev->dev, "could not get default pinstate\n");

	led_bl->pins_sleep = pinctrl_lookup_state(led_bl->pinctrl,
			PINCTRL_STATE_SLEEP);
	if (IS_ERR(led_bl->pins_sleep))
		dev_err(&pdev->dev, "could not get sleep pinstate\n");

	led_bl->pins_inactive = pinctrl_lookup_state(led_bl->pinctrl,
			"inactive");
	if (IS_ERR(led_bl->pins_inactive))
		dev_err(&pdev->dev, "could not get inactive pinstate\n");

	xgold_led_set_pinctrl_state(&pdev->dev, led_bl->pins_default);

	if (led_bl->init) {
		ret = led_bl->init(&led_bl->pdev->dev,
						led_bl->mmio_base);
		ret = led_bl->set_clk(&led_bl->pdev->dev,
						TRUE);
		ret = led_bl->set_lcd_backlight(
				&led_bl->pdev->dev,
				led_bl->mmio_base, &led_bl->lock,
				led_bl->led_bl_brightness);
		if (ret < 0) {
			dev_err(&pdev->dev, "Plat Init Failed: %s\n",
					led_bl->led_bl_cdev.name);
			ret = -EINVAL;
			goto failed_unregister_led_class;
		}
	}
	return 0;

failed_unregister_led_class:
	led_classdev_unregister(&led_bl->led_bl_cdev);
failed_unmap:
	iounmap(led_bl->mmio_base);
failed_free_mem:
	kfree(led_bl);
	return ret;
}

static int xgold_led_bl_remove(struct platform_device *pdev)
{
	struct xgold_led_bl_device *led_bl = platform_get_drvdata(pdev);

	xgold_led_set_pinctrl_state(&pdev->dev, led_bl->pins_inactive);
	led_bl->set_clk(&pdev->dev, FALSE);
	platform_set_drvdata(pdev, NULL);
	cancel_work_sync(&led_bl->work);
	led_classdev_unregister(&led_bl->led_bl_cdev);
	led_bl->exit();
	return 0;
}

#ifdef CONFIG_PM
static int xgold_led_bl_suspend(struct device *dev)
{
	struct xgold_led_bl_device *led_bl = dev_get_drvdata(dev);

	xgold_led_set_pinctrl_state(dev, led_bl->pins_sleep);
	if (led_bl->set_clk)
		return led_bl->set_clk(dev, FALSE);
	else
		return 0;
}

static int xgold_led_bl_resume(struct device *dev)
{
	struct xgold_led_bl_device *led_bl = dev_get_drvdata(dev);

	xgold_led_set_pinctrl_state(dev, led_bl->pins_default);
	return 0;
}

#else

#define xgold_led_bl_suspend		NULL
#define xgold_led_bl_resume		NULL

#endif /* CONFIG_PM */

static const struct dev_pm_ops xgold_led_bl_pm = {
	.suspend = xgold_led_bl_suspend,
	.resume = xgold_led_bl_resume,
};

static struct of_device_id xgold_led_bl_of_match[] = {
	{.compatible = "intel,led-bl",},
	{.compatible = "intel,pmic-led",},
	{},
};

static struct platform_driver xgold_led_bl_driver = {
	.driver		= {
		.name	= XGOLD_LED_BL_MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm = &xgold_led_bl_pm,
		.of_match_table = xgold_led_bl_of_match,
	},
	.probe		= xgold_led_bl_probe,
	.remove		= xgold_led_bl_remove,
};

static int xgold_led_bl_init(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&bl_pm_class);
	if (ret) {
		pr_err("%s: %s: ERROR adding %s pm class\n",
				XGOLD_LED_BL_MODULE_NAME, __func__,
				bl_pm_class.name);
		return ret;
	}
#endif
	return platform_driver_register(&xgold_led_bl_driver);
}

static void xgold_led_bl_exit(void)
{
	platform_driver_unregister(&xgold_led_bl_driver);
}

module_init(xgold_led_bl_init);
module_exit(xgold_led_bl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("xgold backlight Driver");
MODULE_DEVICE_TABLE(of, xgold_led_bl_of_match);
