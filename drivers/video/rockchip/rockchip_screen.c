/*
 * rockchip screen driver
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

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <linux/of_gpio.h>
#include <linux/rockchip_screen.h>
#include <linux/rockchip_fb.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>

#define PROP_DISPLAY            "intel,display"

#define NODE_DISPLAY_PANEL	"display-panel0"

#define PROP_DISPLAY_GPIORST    "intel,display-gpio-reset"
#define PROP_DISPLAY_GPIOVH     "intel,display-gpio-vhigh"
#define PROP_DISPLAY_GPIOVL     "intel,display-gpio-vlow"

#define GPIO_LIST_POWER_ON      "gpio-power-on"
#define GPIO_LIST_POWER_OFF     "gpio-power-off"

#define PROP_DISPLAY_GPIOTYPE   "intel,gpio-type"
#define PROP_DISPLAY_GPIOVALUE  "intel,gpio-value-delay"

static LIST_HEAD(screen_list);
static struct rockchip_screen *cur_screen;
static int panel_source;

size_t get_fb_size(void)
{
	size_t size = 0;
	u32 xres = 0;
	u32 yres = 0;

	if (unlikely(!cur_screen))
		return 0;

	xres = cur_screen->mode.xres;
	yres = cur_screen->mode.yres;

	xres = ALIGN_N_TIMES(xres, 32);

	/* three buffer as default */
	size = (xres * yres << 2) * 3;
	return ALIGN(size, SZ_1M);
}

int rockchip_get_prmry_screen(struct rockchip_screen *screen)
{
	if (unlikely(!cur_screen) || unlikely(!screen))
		return -1;

	memcpy(screen, cur_screen, sizeof(struct rockchip_screen));
	return 0;
}

int rockchip_set_prmry_screen(struct rockchip_screen *screen)
{
	struct rockchip_screen *s_screen;

	if (unlikely(!screen))
		return -ENODEV;

	if (screen->index < 0)
		return -EINVAL;

	list_for_each_entry(s_screen, &screen_list, panel_list) {
		if (s_screen->index == screen->index)
			cur_screen = s_screen;
	}

	if (cur_screen) {
		cur_screen->vop_id = screen->vop_id;
		cur_screen->screen_id = screen->screen_id;
	}
	return 0;
}

static void rockchip_screen_set_gpiolist(struct rockchip_screen *screen,
					 struct display_pwr_gpio *gpios)
{
	struct display_pwr_gpio *gpio;

	list_for_each_entry(gpio, &gpios->list, list) {
		switch (gpio->type) {
		case DISPLAY_GPIO_VHIGH:
			if (!screen->gpio_vhigh)
				break;

			gpio_direction_output(screen->gpio_vhigh, gpio->value);
			break;

		case DISPLAY_GPIO_VLOW:
			if (!screen->gpio_vlow)
				break;

			gpio_direction_output(screen->gpio_vlow, gpio->value);
			break;

		case DISPLAY_GPIO_RESET:
			if (!screen->gpio_reset)
				break;

			gpio_direction_output(screen->gpio_reset, gpio->value);
			break;
		}

		if (gpio->delay)
			mdelay(gpio->delay);
	}
}

static void rockchip_screen_power_on(struct rockchip_screen *screen)
{
	if (screen->gpios_power_on)
		rockchip_screen_set_gpiolist(screen, screen->gpios_power_on);
}

static void rockchip_screen_power_off(struct rockchip_screen *screen)
{
	if (screen->gpios_power_off)
		rockchip_screen_set_gpiolist(screen, screen->gpios_power_off);
}

/*
 * display power control parse from dts
 */

static int rockchip_screen_parse_display_gpio(struct device_node *n,
					      struct display_pwr_gpio *gpio)
{
	const char *string;
	int array[2];
	int ret;

	gpio->name = n->name;
	ret = of_property_read_string(n, PROP_DISPLAY_GPIOTYPE, &string);
	if (ret) {
		pr_err("%s: Get %s failed\n", __func__, PROP_DISPLAY_GPIOTYPE);
		return ret;
	} else if (!strcmp("vhigh", string)) {
		gpio->type = DISPLAY_GPIO_VHIGH;
	} else if (!strcmp("vlow", string)) {
		gpio->type = DISPLAY_GPIO_VLOW;
	} else if (!strcmp("reset", string)) {
		gpio->type = DISPLAY_GPIO_RESET;
	}

	ret = of_property_read_u32_array(n, PROP_DISPLAY_GPIOVALUE, array, 2);
	if (ret) {
		pr_err("%s: Get %s failed\n", __func__, PROP_DISPLAY_GPIOVALUE);
		return ret;
	}

	gpio->value = array[0];
	gpio->delay = array[1];

	return 0;
}

static int
rockchip_screen_parse_display_gpiolist(struct rockchip_screen *screen,
				       struct device_node *n,
				       struct display_pwr_gpio **gpiolist)
{
	struct device_node *child;
	struct display_pwr_gpio *gpio;

	*gpiolist = devm_kzalloc(screen->dev, sizeof(struct display_pwr_gpio),
				 GFP_KERNEL);
	if (!*gpiolist) {
		pr_err("%s: Can't alloc gpio table\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&(*gpiolist)->list);
	for_each_child_of_node(n, child) {
		gpio = devm_kzalloc(screen->dev, sizeof(*gpio), GFP_KERNEL);
		if (!gpio) {
			pr_err("%s: Allocation of display gpio failed\n",
			       __func__);
			return -EINVAL;
		}

		if (!rockchip_screen_parse_display_gpio(child, gpio))
			list_add_tail(&gpio->list, &(*gpiolist)->list);
		else
			devm_kfree(screen->dev, gpio);
	}

	return 0;
}

static int rockchip_screen_parse_gpio(struct rockchip_screen *screen)
{
	struct device_node *np = screen->dev->of_node;
	enum of_gpio_flags gpio_flags;
	unsigned long flags;
	int ret;

	if (!np) {
		pr_err("%s: Can't find screen matching node\n", __func__);
		return -EINVAL;
	}

	screen->gpio_vhigh = of_get_named_gpio_flags(np,
			PROP_DISPLAY_GPIOVH, 0, &gpio_flags);
	if (gpio_is_valid(screen->gpio_vhigh)) {
		if (support_loader_display()) {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_OUT_INIT_HIGH;
		} else {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_HIGH;
			else
				flags = GPIOF_OUT_INIT_LOW;
		}
		ret = gpio_request_one(screen->gpio_vhigh, flags, "disp_vhigh");
		if (ret) {
			pr_err("%s: request display high power gpio fail: %d\n",
			       __func__, ret);
			screen->gpio_vhigh = 0;
		}
	} else {
		screen->gpio_vhigh = 0;
	}

	screen->gpio_vlow = of_get_named_gpio_flags(np,
			PROP_DISPLAY_GPIOVL, 0, &gpio_flags);
	if (gpio_is_valid(screen->gpio_vlow)) {
		if (support_loader_display()) {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_OUT_INIT_HIGH;
		} else {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_HIGH;
			else
				flags = GPIOF_OUT_INIT_LOW;
		}
		ret = gpio_request_one(screen->gpio_vlow, flags, "disp_vlow");
		if (ret) {
			pr_err("%s: request display low power gpio fail: %d\n",
			       __func__, ret);
			screen->gpio_vlow = 0;
		}
	} else {
		screen->gpio_vlow = 0;
	}

	screen->gpio_reset = of_get_named_gpio_flags(np,
			PROP_DISPLAY_GPIORST, 0, &gpio_flags);
	if (gpio_is_valid(screen->gpio_reset)) {
		if (support_loader_display()) {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_OUT_INIT_HIGH;
		} else {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_HIGH;
			else
				flags = GPIOF_OUT_INIT_LOW;
		}
		ret = gpio_request_one(screen->gpio_reset, flags, "disp_rst");
		if (ret) {
			pr_err("%s: request display reset gpio fail: %d\n",
			       __func__, ret);
			screen->gpio_reset = 0;
		}
	} else {
		screen->gpio_reset = 0;
	}

	return 0;
}

static int rockchip_disp_pwr_ctr_parse_dt(struct device_node *np,
					  struct rockchip_screen *screen)
{
	int ret = 0;
	struct device_node *child;

	if (screen->type != SCREEN_MIPI) {
		rockchip_screen_parse_gpio(screen);

		for_each_child_of_node(np, child) {
			if (!strcmp(child->name, GPIO_LIST_POWER_ON)) {
				ret = rockchip_screen_parse_display_gpiolist(
						screen, child,
						&screen->gpios_power_on);
			} else if (!strcmp(child->name, GPIO_LIST_POWER_OFF)) {
				ret = rockchip_screen_parse_display_gpiolist(
						screen, child,
						&screen->gpios_power_off);
			}

			if (ret)
				pr_info("%s: Node %s parsing failed %d\n",
					__func__, child->name, ret);
		}
	}

	return 0;
}

int rockchip_disp_pwr_enable(struct rockchip_screen *screen)
{
	int ret = 0;

	if (unlikely(!screen))
		return -ENODEV;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (screen->pm_platdata) {
		ret = device_state_pm_set_state_by_name(
			screen->dev, screen->pm_platdata->pm_state_D0_name);
		if (ret < 0) {
			dev_err(screen->dev, "Error while setting the pm class\n");
			return ret;
		}
	}
#endif

	if (!support_loader_display() && screen->type != SCREEN_MIPI) {
		if (screen->power_on)
			screen->power_on(screen);
	}

	return 0;
}

int rockchip_disp_pwr_disable(struct rockchip_screen *screen)
{
	int ret = 0;

	if (unlikely(!screen))
		return -ENODEV;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (screen->pm_platdata) {
		ret = device_state_pm_set_state_by_name(
			screen->dev, screen->pm_platdata->pm_state_D3_name);

		if (ret < 0) {
			dev_err(screen->dev, "could not set PM state: %s\n",
				screen->pm_platdata->pm_state_D3_name);
			return ret;
		}
	}
#endif

	if (screen->type != SCREEN_MIPI) {
		if (screen->power_off)
			screen->power_off(screen);
	}

	return 0;
}

static int rockchip_fb_videomode_from_timing(const struct display_timing *dt,
					  struct rockchip_screen *screen)
{
	screen->mode.pixclock = dt->pixelclock.typ;
	screen->mode.left_margin = dt->hback_porch.typ;
	screen->mode.right_margin = dt->hfront_porch.typ;
	screen->mode.xres = dt->hactive.typ;
	screen->mode.hsync_len = dt->hsync_len.typ;
	screen->mode.upper_margin = dt->vback_porch.typ;
	screen->mode.lower_margin = dt->vfront_porch.typ;
	screen->mode.yres = dt->vactive.typ;
	screen->mode.vsync_len = dt->vsync_len.typ;
	screen->type = dt->screen_type;
	screen->lvds_format = dt->lvds_format;
	screen->face = dt->face;
	screen->color_mode = dt->color_mode;
	screen->dsp_lut = dt->dsp_lut;
	screen->width = dt->width;
	screen->height = dt->height;

	if (dt->flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		screen->pin_dclk = 1;
	else
		screen->pin_dclk = 0;
	if (dt->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		screen->pin_hsync = 1;
	else
		screen->pin_hsync = 0;
	if (dt->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		screen->pin_vsync = 1;
	else
		screen->pin_vsync = 0;
	if (dt->flags & DISPLAY_FLAGS_DE_HIGH)
		screen->pin_den = 1;
	else
		screen->pin_den = 0;

	return 0;
}

static struct rockchip_screen *
rockchip_prase_screen_dt(struct device_node *np,
			 struct platform_device *pdev)

{
	struct display_timings *disp_timing;
	struct display_timing *dt;
	struct rockchip_screen *screen = NULL;

	disp_timing = of_get_display_timings(np);
	if (!disp_timing) {
		pr_err("parse display timing err\n");
		return NULL;
	}

	dt = display_timings_get(disp_timing, disp_timing->native_mode);
	if (!dt)
		return NULL;

	screen = devm_kzalloc(&pdev->dev,
			sizeof(struct rockchip_screen), GFP_KERNEL);
	if (!screen) {
		dev_err(&pdev->dev, "kmalloc for rockchip screen fail!\n");
		return NULL;
	}
	screen->dev = &pdev->dev;
	rockchip_fb_videomode_from_timing(dt, screen);
	rockchip_disp_pwr_ctr_parse_dt(np, screen);
	display_timings_release(disp_timing);

	return screen;
}

static int rockchip_screen_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *display_np;
	struct rockchip_screen *screen;
	int ret = 0;
	int index = 0;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#endif

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}

#ifdef CONFIG_PLATFORM_DEVICE_PM
	pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pm_platdata)) {
		dev_err(&pdev->dev, "Error during device state pm init.\n");
		pm_platdata = NULL;
		return -EINVAL;
	}

	if (pm_platdata) {
		ret = device_state_pm_set_class(&pdev->dev,
					pm_platdata->pm_user_name);
		if (ret < 0) {
			dev_err(&pdev->dev, "Fail to init PM state!\n");
			kfree(pm_platdata);
			pm_platdata = NULL;
			return -EINVAL;
		}
	}
#endif


	for_each_compatible_node(display_np, NULL, PROP_DISPLAY) {
		screen = rockchip_prase_screen_dt(display_np, pdev);
		if (screen) {
			screen->index = index;
			screen->power_on = rockchip_screen_power_on;
			screen->power_off = rockchip_screen_power_off;
			screen->pm_platdata = pm_platdata;
			list_add_tail(&screen->panel_list, &screen_list);

			if (panel_source < 0 && screen->index == 0)
				screen->index = panel_source;
			if (screen->index == panel_source)
				cur_screen = screen;
		}
		index++;
	}

	dev_info(&pdev->dev, "rockchip screen probe %s\n",
		 ret ? "failed" : "success");
	return ret;
}

static __init int parse_panel_source(char *arg)
{
	int ret;

	if (!arg)
		return -EINVAL;

	ret = sscanf(arg, "%d", &panel_source);
	if (ret != 1) {
		panel_source = 0;
		return -EINVAL;
	}

	return 0;
}
early_param("panelsource", parse_panel_source);

static const struct of_device_id rockchip_screen_dt_ids[] = {
	{.compatible = "rockchip,screen",},
	{}
};

struct platform_driver rockchip_screen_driver = {
	.probe = rockchip_screen_probe,
	.driver = {
		   .name = "rockchip-screen",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rockchip_screen_dt_ids),
		   },
};
