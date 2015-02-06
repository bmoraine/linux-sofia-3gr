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
#include <linux/rockchip_screen.h>
#include <video/display_timing.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <video/of_display_timing.h>
#include <dt-bindings/sofiafb/sofia_fb.h>
#endif

#define PROP_DISPLAY_GPIO_RST	"intel,disp-gpio-reset"

#define OF_GET_U32(_n_, _p_, _pval_, _e_) \
	do { \
		_e_ = of_property_read_u32(_n_, _p_, _pval_); \
		if (_e_) { \
			*_pval_ = 0; \
		} \
	} while (0)

static struct rockchip_screen *sfa_screen;

size_t get_fb_size(void)
{
	size_t size = 0;
	u32 xres = 0;
	u32 yres = 0;

	if (unlikely(!sfa_screen))
		return 0;

	xres = sfa_screen->mode.xres;
	yres = sfa_screen->mode.yres;

	xres = ALIGN_N_TIMES(xres, 32);

	/* three buffer as default */
	size = (xres * yres << 2) * 3;
	return ALIGN(size, SZ_1M);
}

int rockchip_get_prmry_screen(struct rockchip_screen *screen)
{
	if (unlikely(!sfa_screen) || unlikely(!screen))
		return -1;

	memcpy(screen, sfa_screen, sizeof(struct rockchip_screen));
	return 0;
}

int rockchip_set_prmry_screen(struct rockchip_screen *screen)
{
	if (unlikely(!sfa_screen) || unlikely(!screen))
		return -1;

	sfa_screen->vop_id = screen->vop_id;
	sfa_screen->screen_id = screen->screen_id;
	return 0;
}

/*
 * display power control parse from dts
 */
#ifdef CONFIG_PLATFORM_DEVICE_PM
static int rockchip_disp_pwr_ctr_parse_dt(struct device_node *np,
				       struct rockchip_screen *screen)
{
	enum of_gpio_flags flags;
	int ret = 0, i, length = 0;
	unsigned int val;
	const __be32 *p;
	struct property *prop;
	int *array;
	struct rockchip_disp_reset_list *res;

	screen->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(screen->pm_platdata)) {
		dev_err(screen->dev, "Error during device state pm init.\n");
		screen->pm_platdata = NULL;
		return -EINVAL;
	}

	screen->gpio_rst =
		of_get_named_gpio_flags(np, PROP_DISPLAY_GPIO_RST, 0, &flags);

	if (!gpio_is_valid(screen->gpio_rst)) {
		dev_err(screen->dev, "ivalid gpio reset\n");
		return -EINVAL;
	}

	ret = gpio_request(screen->gpio_rst, "lcd_rst");
	if (ret) {
		dev_err(screen->dev, "request lcd rst gpio fail:%d\n", ret);
		return ret;
	}

	/* count array size */
	of_property_for_each_u32(np, "intel,display-reset", prop, p, val) {
		length++;
	};

	if (length == 0) {
		screen->resetlist = NULL;
		return 0;
	}

	if (length % 2) {
		dev_err(screen->dev, "intel,display-reset array length should be even\n");
		return -EINVAL;
	}

	array = devm_kzalloc(screen->dev, length * sizeof(int), GFP_KERNEL);
	ret = of_property_read_u32_array(np, "intel,display-reset",
					 array, length);
	if (ret) { /* already checked few lines before but does not hurt */
		pr_err("Can't read property:%s\n", "intel,display-reset");
		screen->resetlist = NULL;
		return 0;
	}

	screen->resetlist =
		devm_kzalloc(screen->dev,
				sizeof(struct rockchip_disp_reset_list),
				GFP_KERNEL);
	if (!screen->resetlist) {
		dev_err(screen->dev, "Can't alloc array for disp_reset_list\n");
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&screen->resetlist->list);

	for (i = 0; i < length; i += 2) {
		res = (struct rockchip_disp_reset_list *)
			devm_kzalloc(screen->dev, sizeof(*res), GFP_KERNEL);
		if (!res) {
			pr_err("allocation of reset failed\n");
			return -EINVAL;
		}
		res->value = array[i];
		res->mdelay = array[i+1];
		list_add_tail(&res->list, &screen->resetlist->list);
	}
	devm_kfree(screen->dev, array);
	return 0;
}

int rockchip_disp_pwr_enable(struct rockchip_screen *screen)
{
	int ret = 0;
	struct rockchip_disp_reset_list *resetlist = screen->resetlist;
	struct rockchip_disp_reset_list *res;

	if (screen->pm_platdata) {
		ret = device_state_pm_set_state_by_name(screen->dev,
				screen->pm_platdata->pm_state_D0_name);
		if (ret < 0) {
			dev_err(screen->dev, "Error while setting the pm class\n");
			return ret;
		}
	}

	if (!resetlist)
		return 0;
	res = list_first_entry_or_null(&resetlist->list,
				       struct rockchip_disp_reset_list, list);

	if (!res)
		return 0;

	list_for_each_entry(res, &screen->resetlist->list, list) {
		gpio_direction_output(screen->gpio_rst, res->value);
		mdelay(res->mdelay);
	}

	return 0;
}

int rockchip_disp_pwr_disable(struct rockchip_screen *screen)
{
	int ret = 0;
	struct rockchip_disp_reset_list *res;

	if (screen->pm_platdata) {
		ret = device_state_pm_set_state_by_name(screen->dev,
				screen->pm_platdata->pm_state_D3_name);

		if (ret < 0) {
			dev_err(screen->dev, "could not set PM state: %s\n",
				screen->pm_platdata->pm_state_D3_name);
			return ret;
		}
	}

	if (screen->resetlist) {
		res = list_last_entry(&screen->resetlist->list,
				      struct rockchip_disp_reset_list, list);
		gpio_direction_output(screen->gpio_rst, !res->value);
	}
	return 0;
}

#else
static int rockchip_disp_pwr_ctr_parse_dt(struct device_node *np,
				       struct rockchip_screen *screen)
{
	struct device_node *root = of_get_child_by_name(np, "power_ctr");
	struct device_node *child;
	struct rockchip_disp_pwr_ctr_list *pwr_ctr;
	struct list_head *pos;
	enum of_gpio_flags flags;
	u32 val = 0;
	u32 debug = 0;
	int ret;

	INIT_LIST_HEAD(&screen->pwrlist_head);
	if (!root) {
		pr_err("can't find power_ctr node for screen\n");
		return -ENODEV;
	}

	for_each_child_of_node(root, child) {
		pwr_ctr = kzalloc(sizeof(*pwr_ctr), GFP_KERNEL);
		strcpy(pwr_ctr->pwr_ctr.name, child->name);
		if (!of_property_read_u32(child, "rockchip,power_type", &val)) {
			if (val == GPIO) {
				pwr_ctr->pwr_ctr.type = GPIO;
				pwr_ctr->pwr_ctr.gpio =
				    of_get_gpio_flags(child, 0, &flags);
				if (!gpio_is_valid(pwr_ctr->pwr_ctr.gpio)) {
					pr_err("%s ivalid gpio\n", child->name);
					return -EINVAL;
				}
				pwr_ctr->pwr_ctr.atv_val =
				    !(flags & OF_GPIO_ACTIVE_LOW);
				ret = gpio_request(pwr_ctr->pwr_ctr.gpio,
						   child->name);
				if (ret)
					pr_err("request %s gpio fail:%d\n",
					       child->name, ret);
			} else {
				pwr_ctr->pwr_ctr.type = REGULATOR;
				pwr_ctr->pwr_ctr.rgl_name = NULL;
				ret = of_property_read_string(child,
						"rockchip,regulator_name",
						&(pwr_ctr->pwr_ctr.rgl_name));
				if (ret ||
				    IS_ERR_OR_NULL(pwr_ctr->pwr_ctr.rgl_name))
					pr_err("get regulator name failed!\n");
				if (!of_property_read_u32(child,
							  "rockchip,regulator_voltage",
							  &val))
					pwr_ctr->pwr_ctr.volt = val;
				else
					pwr_ctr->pwr_ctr.volt = 0;
			}
		};

		if (!of_property_read_u32(child, "rockchip,delay", &val))
			pwr_ctr->pwr_ctr.delay = val;
		else
			pwr_ctr->pwr_ctr.delay = 0;
		list_add_tail(&pwr_ctr->list, &screen->pwrlist_head);
	}

	of_property_read_u32(root, "rockchip,debug", &debug);

	if (debug) {
		list_for_each(pos, &screen->pwrlist_head) {
			pwr_ctr = list_entry(pos,
					     struct rockchip_disp_pwr_ctr_list,
					     list);
			pr_info("pwr_ctr_name:%s\n" "pwr_type:%s\n"
			       "gpio:%d\n" "atv_val:%d\n" "delay:%d\n\n",
			       pwr_ctr->pwr_ctr.name,
			       (pwr_ctr->pwr_ctr.type ==
				GPIO) ? "gpio" : "regulator",
			       pwr_ctr->pwr_ctr.gpio, pwr_ctr->pwr_ctr.atv_val,
			       pwr_ctr->pwr_ctr.delay);
		}
	}

	return 0;
}

int rockchip_disp_pwr_enable(struct rockchip_screen *screen)
{
	struct list_head *pos;
	struct rockchip_disp_pwr_ctr_list *pwr_ctr_list;
	struct pwr_ctr *pwr_ctr;
	struct regulator *regulator_lcd = NULL;
	int count = 10;

	if (unlikely(!screen))
		return -ENODEV;
	if (list_empty(&screen->pwrlist_head))
		return 0;
	list_for_each(pos, &screen->pwrlist_head) {
		pwr_ctr_list = list_entry(pos,
				struct rockchip_disp_pwr_ctr_list, list);
		pwr_ctr = &pwr_ctr_list->pwr_ctr;
		if (pwr_ctr->type == GPIO) {
			gpio_direction_output(pwr_ctr->gpio, pwr_ctr->atv_val);
			mdelay(pwr_ctr->delay);
		} else if (pwr_ctr->type == REGULATOR) {
			if (pwr_ctr->rgl_name)
				regulator_lcd =
				    regulator_get(NULL, pwr_ctr->rgl_name);
			if (regulator_lcd == NULL) {
				pr_err
				    ("%s: regulator [%s] get failed\n",
				     __func__, pwr_ctr->rgl_name);
				continue;
			}
			regulator_set_voltage(regulator_lcd, pwr_ctr->volt,
					      pwr_ctr->volt);
			while (!regulator_is_enabled(regulator_lcd)) {
				if (regulator_enable(regulator_lcd) == 0 ||
				    count == 0)
					break;

				pr_err("regulator_enable failed,count=%d\n",
				       count);
				count--;
			}
			regulator_put(regulator_lcd);
			msleep(pwr_ctr->delay);
		}
	}

	return 0;
}

int rockchip_disp_pwr_disable(struct rockchip_screen *screen)
{
	struct list_head *pos;
	struct rockchip_disp_pwr_ctr_list *pwr_ctr_list;
	struct pwr_ctr *pwr_ctr;
	struct regulator *regulator_lcd = NULL;
	int count = 10;

	if (unlikely(!screen))
		return -ENODEV;
	if (list_empty(&screen->pwrlist_head))
		return 0;
	list_for_each(pos, &screen->pwrlist_head) {
		pwr_ctr_list = list_entry(pos,
				struct rockchip_disp_pwr_ctr_list, list);
		pwr_ctr = &pwr_ctr_list->pwr_ctr;
		if (pwr_ctr->type == GPIO) {
			gpio_set_value(pwr_ctr->gpio, !pwr_ctr->atv_val);
		} else if (pwr_ctr->type == REGULATOR) {
			if (pwr_ctr->rgl_name)
				regulator_lcd =
				    regulator_get(NULL, pwr_ctr->rgl_name);
			if (regulator_lcd == NULL) {
				pr_err
				    ("%s: regulator [%s] get failed\n",
				     __func__, pwr_ctr->rgl_name);
				continue;
			}
			while (regulator_is_enabled(regulator_lcd) > 0) {
				if (regulator_disable(regulator_lcd) == 0 ||
				    count == 0)
					break;

				pr_err("regulator_disable failed,count=%d\n",
				       count);
				count--;
			}
			regulator_put(regulator_lcd);
		}
	}
	return 0;
}
#endif

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

static int rockchip_prase_timing_dt(struct device_node *np,
				 struct rockchip_screen *screen)
{
	struct display_timings *disp_timing;
	struct display_timing *dt;

	disp_timing = of_get_display_timings(np);
	if (!disp_timing) {
		pr_err("parse display timing err\n");
		return -EINVAL;
	}
	dt = display_timings_get(disp_timing, disp_timing->native_mode);
	rockchip_fb_videomode_from_timing(dt, screen);
	return 0;
}

static int rockchip_screen_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}
	sfa_screen = devm_kzalloc(&pdev->dev,
				  sizeof(struct rockchip_screen), GFP_KERNEL);
	if (!sfa_screen) {
		dev_err(&pdev->dev, "kmalloc for rockchip screen fail!\n");
		return -ENOMEM;
	}

	sfa_screen->dev =  &pdev->dev;
	ret = rockchip_prase_timing_dt(np, sfa_screen);
	rockchip_disp_pwr_ctr_parse_dt(np, sfa_screen);

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (sfa_screen->pm_platdata) {
		ret = device_state_pm_set_class(&pdev->dev,
				sfa_screen->pm_platdata->pm_user_name);
		if (ret < 0) {
			dev_err(&pdev->dev, "ERROR while LVDS initialize its PM state!\n");
			kfree(sfa_screen->pm_platdata);
			sfa_screen->pm_platdata = NULL;
		}
	}
#endif

	dev_info(&pdev->dev, "rockchip screen probe %s\n",
		 ret ? "failed" : "success");
	return ret;
}

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
