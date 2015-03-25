/*
 * drivers/video/nanosilicon/nanosilicon_lvds.c
 *	Nanosilicon lvds hardware driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/rockchip_fb.h>
#include "nanosilicon_lvds.h"

#define LVDS_VOP_REG_OFFSET	0x140
static struct lvds_device *nanosilicon_lvds;

static int lvds_writel(struct lvds_device *lvds, u32 offset, u32 val)
{
	struct rockchip_screen *screen;
	struct rockchip_vop_driver *vop_drv;
	char name[10] = {0};

	if (unlikely(!lvds))
		return 0;

	screen = &lvds->screen;
	sprintf(name, "vop%d", screen->vop_id);
	vop_drv = get_vop_drv(name);
	if (!vop_drv) {
		dev_err(lvds->dev, "no find vop drv\n");
		return -ENODEV;
	}

	offset += LVDS_VOP_REG_OFFSET;
	return vop_drv->ops->reg_writel(vop_drv, offset, val);
}

static u32 lvds_readl(struct lvds_device *lvds, u32 offset)
{
	struct rockchip_screen *screen;
	struct rockchip_vop_driver *vop_drv;
	char name[10] = {0};

	if (unlikely(!lvds))
		return 0;

	screen = &lvds->screen;
	sprintf(name, "vop%d", screen->vop_id);
	vop_drv = get_vop_drv(name);
	if (!vop_drv) {
		dev_err(lvds->dev, "no find vop drv\n");
		return -ENODEV;
	}

	offset += LVDS_VOP_REG_OFFSET;
	return vop_drv->ops->reg_readl(vop_drv, offset);
}

static int lvds_msk_reg(struct lvds_device *lvds, u32 offset, u32 msk, u32 val)
{
	u32 temp = 0;

	if (unlikely(!lvds))
		return 0;

	temp = lvds_readl(lvds, offset) & (0xffff - msk);
	return lvds_writel(lvds, offset, temp | (val & msk));
}

static int nanosilicon_lvds_disable(void)
{
	struct lvds_device *lvds = nanosilicon_lvds;
	int ret = 0;

	if (unlikely(!lvds) || !lvds->sys_state)
		return 0;

	/* power down */
	lvds_msk_reg(lvds, LVDS_REG_CTRL, M_LVDS_POWER_MODE,
		     V_LVDS_POWER_MODE(0));

	if (lvds->pins && lvds->pins->p) {
		ret = pinctrl_select_state(lvds->pins->p,
					   lvds->pins->sleep_state);
		if (ret < 0)
			dev_err(lvds->dev, "Error while setting PIN sleep state\n");
	}
	lvds->sys_state = false;
	return 0;
}

static void nanosilicon_lvds_signal_set(struct lvds_device *lvds)
{
	u32 mask = 0, val = 0;

	mask = M_LVDS_CLK_EDGE | M_LVDS_CLK_DS1 | M_LVDS_OFFSET_VOLT |
	    M_LVDS_SWING | M_LVDS_PRE_EMPHASIS;

	val |= V_LVDS_CLK_EDGE(LVDS_EDGE_FALL);
	val |= V_LVDS_CLK_DS1(LVDS_SKEW_CLK_0PS);
	val |= V_LVDS_OFFSET_VOLT(LVDS_OFFSET_800MV);	/* offset 0.8v */
	val |= V_LVDS_SWING(LVDS_SWING_200MV);
	val |= V_LVDS_PRE_EMPHASIS(LVDS_EMP_0DB);

	lvds_msk_reg(lvds, LVDS_REG_CTRL, mask, val);
}

static void nanosilicon_output_lvds(struct lvds_device *lvds,
				 struct rockchip_screen *screen)
{
	u32 mask = 0, val = 0;
	/*u32 delay_times = 20;*/

	/* power down */
	lvds_msk_reg(lvds, LVDS_REG_CTRL, M_LVDS_POWER_MODE,
		     V_LVDS_POWER_MODE(0));

	/* config parameter */
	mask = M_LVDS_TTL_MODE_EN | M_LVDS_SELECT | M_LVDS_MSBSEL |
	    M_LVDS_CLK_EDGE | M_LVDS_DATA_BITS;

	val |= V_LVDS_TTL_MODE_EN(0);
	val |= V_LVDS_SELECT(screen->lvds_format);
	val |= V_LVDS_MSBSEL(LVDS_MSB_D7);
	if (screen->face == OUT_P888)
		val |= V_LVDS_DATA_BITS(LVDS_8_BIT);
	else
		val |= V_LVDS_DATA_BITS(LVDS_6_BIT);

	lvds_msk_reg(lvds, LVDS_REG_CTRL, mask, val);
	nanosilicon_lvds_signal_set(lvds);

	/* power up */
	lvds_msk_reg(lvds, LVDS_REG_CTRL, M_LVDS_POWER_MODE,
		     V_LVDS_POWER_MODE(1));

	/* delay for waitting pll lock on */
	/*while (delay_times--) {
		if (lvds_phy_lockon(lvds)) {
			msleep(1);
			break;
		}
		udelay(100);
	}*/
}

static void nanosilicon_output_lvttl(struct lvds_device *lvds,
				  struct rockchip_screen *screen)
{
	lvds_msk_reg(lvds, LVDS_REG_CTRL, M_LVDS_TTL_MODE_EN,
		     V_LVDS_TTL_MODE_EN(1));
	/* power up */
	lvds_msk_reg(lvds, LVDS_REG_CTRL, M_LVDS_POWER_MODE,
		     V_LVDS_POWER_MODE(1));
}

static int nanosilicon_lvds_enable(void)
{
	struct lvds_device *lvds = nanosilicon_lvds;
	struct rockchip_screen *screen;
	int ret = 0;

	if (unlikely(!lvds) || lvds->sys_state)
		return 0;

	/* iomux set to default state */
	if (lvds->pins && lvds->pins->p) {
		ret = pinctrl_select_state(lvds->pins->p,
					   lvds->pins->default_state);
		if (ret < 0) {
			dev_err(lvds->dev, "Error while setting PIN default state\n");
			return ret;
		}
	}

	screen = &lvds->screen;
	rockchip_get_prmry_screen(screen);

	switch (screen->type) {
	case SCREEN_LVDS:
		nanosilicon_output_lvds(lvds, screen);
		break;
	case SCREEN_RGB:
		nanosilicon_output_lvttl(lvds, screen);
		break;
	default:
		dev_info(lvds->dev, "unsupport screen type\n");
		break;
	}

	lvds->sys_state = true;
	return 0;
}

static struct rockchip_fb_trsm_ops trsm_lvds_ops = {
	.enable = nanosilicon_lvds_enable,
	.disable = nanosilicon_lvds_disable,
};

static int nanosilicon_lvds_of_parse_dt(struct lvds_device *lvds)
{
	int ret = 0;

	lvds->pins = devm_kzalloc(lvds->dev, sizeof(*lvds->pins),
				  GFP_KERNEL);
	if (!lvds->pins) {
		dev_err(lvds->dev, "kzalloc lvds pins failed\n");
		return -ENOMEM;
	}

	lvds->pins->p = devm_pinctrl_get(lvds->dev);
	if (IS_ERR(lvds->pins->p)) {
		dev_err(lvds->dev, "Can not get pinctrl.\n");
		goto out;
	}

	lvds->pins->default_state = pinctrl_lookup_state(
		lvds->pins->p, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(lvds->pins->default_state)) {
		dev_err(lvds->dev, "could not get default pinctrl state\n");
		goto out;
	}
	lvds->pins->sleep_state = pinctrl_lookup_state(
		lvds->pins->p, PINCTRL_STATE_SLEEP);
	if (IS_ERR(lvds->pins->sleep_state))
		dev_err(lvds->dev, "could not get sleep pinctrl state\n");

	return ret;

out:
	devm_kfree(lvds->dev, lvds->pins);
	lvds->pins = NULL;
	ret = -EINVAL;
	return ret;
}

static int nanosilicon_lvds_probe(struct platform_device *pdev)
{
	struct lvds_device *lvds;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	if (!np) {
		dev_err(&pdev->dev, "Don't find lvds device tree node.\n");
		return -EINVAL;
	}

	lvds =
	    devm_kzalloc(&pdev->dev, sizeof(struct lvds_device),
			 GFP_KERNEL);
	if (!lvds) {
		dev_err(&pdev->dev, "kzalloc nanosilicon lvds failed\n");
		return -ENOMEM;
	}
	lvds->dev = &pdev->dev;

	rockchip_get_prmry_screen(&lvds->screen);
	if ((lvds->screen.type != SCREEN_RGB) &&
	    (lvds->screen.type != SCREEN_LVDS)) {
		dev_err(&pdev->dev, "screen is not lvds/rgb!\n");
		ret = -EINVAL;
		goto err_free_mem;
	}

	platform_set_drvdata(pdev, lvds);
	dev_set_name(lvds->dev, "nanosilicon-lvds");

	ret = nanosilicon_lvds_of_parse_dt(lvds);
	if (ret < 0)
		goto err_free_mem;

	nanosilicon_lvds = lvds;
	rockchip_fb_trsm_ops_register(&trsm_lvds_ops, SCREEN_LVDS);
	if (support_loader_display())
		lvds->sys_state = true;

	dev_info(&pdev->dev, "nanosilicon lvds driver probe success\n");

	return 0;

err_free_mem:
	devm_kfree(&pdev->dev, lvds);
	lvds = NULL;
	return ret;
}

static int nanosilicon_lvds_remove(struct platform_device *pdev)
{
	struct lvds_device *lvds = platform_get_drvdata(pdev);

	if (lvds->pins && lvds->pins->p) {
		pinctrl_select_state(lvds->pins->p,
				     lvds->pins->sleep_state);
		devm_kfree(lvds->dev, lvds->pins);
	}
	devm_kfree(&pdev->dev, lvds);
	lvds = NULL;
	return 0;
}

static void nanosilicon_lvds_shutdown(struct platform_device *pdev)
{
	struct lvds_device *lvds = platform_get_drvdata(pdev);

	if (lvds->pins && lvds->pins->p) {
		pinctrl_select_state(lvds->pins->p,
				     lvds->pins->sleep_state);
		devm_kfree(lvds->dev, lvds->pins);
	}
	devm_kfree(&pdev->dev, lvds);
	lvds = NULL;
}

#if defined(CONFIG_OF)
static const struct of_device_id nanosilicon_lvds_dt_ids[] = {
	{.compatible = "nanosilicon,nanosilicon-lvds",},
	{}
};
#endif

struct platform_driver nanosilicon_lvds_driver = {
	.driver = {
		   .name = "nanosilicon-lvds",
		   .owner = THIS_MODULE,
#if defined(CONFIG_OF)
		   .of_match_table = of_match_ptr(nanosilicon_lvds_dt_ids),
#endif
		   },
	.probe = nanosilicon_lvds_probe,
	.remove = nanosilicon_lvds_remove,
	.shutdown = nanosilicon_lvds_shutdown,
};
