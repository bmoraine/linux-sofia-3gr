
/*
 ****************************************************************
 *
 *  Component: DCC driver
 *
 *  Copyright (C) 2011, Intel Mobile Communications GmbH.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************
 */

#include <linux/module.h>
#include <linux/console.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/regulator/consumer.h>

#include "dcc-core.h"


static struct of_device_id xgold_display_of_match[] = {
	{ .compatible = "intel,display", },
	{ },
};

static struct of_device_id xgold_graphics_of_match[] = {
	{ .compatible = "intel,graphics", },
	{ },
};


#define PROP_DISPLAY_RESOLUTION		"intel,display-resolution"
#define PROP_DISPLAY_PIXELFORMAT	"intel,display-pixelformat"
#define PROP_DISPLAY_INTERFACE		"intel,display-if"
#define PROP_DISPLAY_IF_SEGMENTS	"intel,display-if-segments-per-pix"
#define PROP_DISPLAY_IF_BITS		"intel,display-if-bits-per-segment"


#define NODE_DISPLAY_CMDINIT		"cmd-init"
#define NODE_DISPLAY_CMDUPDATE		"cmd-update"
#define PROP_DISPLAY_CMDDELAY		"intel,cmd-delay"
#define PROP_DISPLAY_TIM_ADDR		"intel,adress-delay"
#define PROP_DISPLAY_TIM_CSACT		"intel,cs-active"
#define PROP_DISPLAY_TIM_DATA		"intel,data-delay"
#define PROP_DISPLAY_TIM_WRACT		"intel,wr-active"
#define PROP_DISPLAY_TIM_WRDEACT	"intel,wr-deactive"
#define PROP_DISPLAY_TIM_CSDEACT	"intel,cs-deactive"
#define PROP_DISPLAY_TIM_CYCLE		"intel,access-cycle"

#define PROP_DISPLAY_GPIO_LCD_BIAS	"intel,lcd-bias-en"
#define PROP_DISPLAY_GPIO_RST		"intel,dcc-gpio-reset"
#define PROP_DISPLAY_GPIO_RST_DLY	"intel,dcc-gpio-reset-delay"
#define PROP_DISPLAY_GPIO_CD		"intel,dcc-gpio-cd"
#define PROP_FB_BASE			"intel,fb-mem"
#define PROP_DCC_FBPIXELFORMAT		"intel,dcc-fbpixelformat"
#define PROP_DCC_MUX			"intel,dcc-mux"
#define PROP_PINCTRL			"intel,pinctrl"
#define PROP_PINCTRL_GROUP		"intel,pinctrl-group"
#define PROP_PCL_NUM			"intel,num"

#define OF_KERNEL_CLK	"clk_kernel"
#define OF_AHB_CLK	"clk_ahb"
#define OF_MASTER_CLK	"clk_master"
#define OF_PLL_CLK	"clk_pll"

#define OF_GET_U32(_n_, _p_, _pval_, _e_) \
	do { \
		_e_ = of_property_read_u32(_n_, _p_, _pval_); \
		if (_e_) { \
			*_pval_ = 0; \
		} \
	} while (0)

#define OF_CHECK_GET_U32(_n_, _p_, _pval_, _e_) \
	do { \
		_e_ = of_property_read_u32(_n_, _p_, _pval_); \
		if (_e_) \
			dcc_err("Can't read property:%s\n", _p_); \
	} while (0)

int dcc_of_parse_display_timing(struct device_node *n,
		struct dcc_display_if_mipi_dbi *dbi)
{
	int ret = 0;
	OF_CHECK_GET_U32(n, PROP_DISPLAY_TIM_ADDR, &dbi->addr_delay_ns, ret);
	OF_CHECK_GET_U32(n, PROP_DISPLAY_TIM_CSACT, &dbi->cs_act_ns, ret);
	OF_CHECK_GET_U32(n, PROP_DISPLAY_TIM_DATA, &dbi->data_delay_ns, ret);
	OF_CHECK_GET_U32(n, PROP_DISPLAY_TIM_WRACT, &dbi->wr_rd_act_ns, ret);
	OF_CHECK_GET_U32(n, PROP_DISPLAY_TIM_WRDEACT,
				&dbi->wr_rd_deact_ns, ret);
	OF_CHECK_GET_U32(n, PROP_DISPLAY_TIM_CSDEACT, &dbi->cs_deact_ns, ret);
	OF_CHECK_GET_U32(n, PROP_DISPLAY_TIM_CYCLE, &dbi->access_cycle_ns, ret);
	return ret;
}


int dcc_of_parse_mux(struct device_node *n,
			unsigned int *mux)
{
	int ret, i, nval;
	u32 val;
	const __be32 *p;
	struct property *prop;

	/* init */
	for (i = 0; i < 32; i++)
		mux[i] = 0;

	nval = 0;
	of_property_for_each_u32(n, PROP_DCC_MUX, prop, p, val) {
		nval++;
	};
	if (nval >= 32) {
		dcc_err("too much val(%d). Limit is %d\n", nval, 32);
		return -EINVAL;
	}
	ret = of_property_read_u32_array(n, PROP_DCC_MUX, mux, nval);
	if (ret) {
		dcc_err("Can't read property:%s\n", PROP_DCC_MUX);
		return -EINVAL;
	}
/*	for(i=0;i<32;i++)
 *		printk("\n %d", mux[i]);
 *	printk("\n"); */
	return 0;
}

#define OF_FIND_MAP_IRQ(_node_, _name_, _irq_) \
{ \
	int i = of_property_match_string(_node_, "interrupt-names", _name_); \
	if (IS_ERR_VALUE(i)) { \
			DCC_DBG2("interrupt: %s not found\n", _name_); \
			_irq_ = 0; \
	} else {\
		_irq_ = irq_of_parse_and_map(_node_, i); \
		DCC_DBG2("interrupt: %s found idx %d irq %d\n", \
		_name_, i, _irq_); \
	} \
}

int dcc_of_parse_irq(struct platform_device *pdev, struct device_node *ndcc)
{
	struct dcc_drvdata *pdata =
		(struct dcc_drvdata *)platform_get_drvdata(pdev);

	if (!pdata)
		return -EINVAL;

	OF_FIND_MAP_IRQ(ndcc, "rx", pdata->irq.rx);
	OF_FIND_MAP_IRQ(ndcc, "tx", pdata->irq.tx);
	OF_FIND_MAP_IRQ(ndcc, "err", pdata->irq.err);
	OF_FIND_MAP_IRQ(ndcc, "cmd", pdata->irq.cmd);
	OF_FIND_MAP_IRQ(ndcc, "frame", pdata->irq.frame);
	OF_FIND_MAP_IRQ(ndcc, "vsync", pdata->irq.vsync);

	return 0;
}

int dcc_of_parse_graphics(struct platform_device *pdev,
		struct device_node *ngraphics)
{
	int ret = 0, length = 0;
	const __be32 *p;
	unsigned int val;
	struct property *prop;
	const char *string;
	u32 array[2];
	struct device_node *nif;
	struct dcc_drvdata *pdata =
		(struct dcc_drvdata *)platform_get_drvdata(pdev);

	if (!pdata)
		return -EINVAL;

	/* dedicated ram */
	of_property_for_each_u32(ngraphics, "intel,dcc-mem",
			prop, p, val) {
		length++;
	};

	if (length == 1)
		ret = of_property_read_u32(ngraphics, "intel,dcc-mem",
			&pdata->mem.size);
	else {
		ret = of_property_read_u32_array(ngraphics, "intel,dcc-mem",
				array, 2);
		pdata->mem.pbase = array[0];
		pdata->mem.size = array[1];
	}
	if (ret || (length > 2)) {
		dcc_err("Can't read property:%s\n", "intel,dcc-mem");
		return -EINVAL;
	}

	/* enable / disable display automatic refresh mode */
	ret = of_property_read_u32(ngraphics, "intel,display-ramless",
			&pdata->display_autorefresh);
	if (ret < 0)
		pdata->display_autorefresh = 0;

	/* enable / disable display composite invertion mode */
	if (of_property_read_bool(ngraphics,
				"intel,display-invert-composite"))
		pdata->display_invert_composite = 1;
	else
		pdata->display_invert_composite = 0;

	/* enable / disable fb API */
	OF_GET_U32(ngraphics, "intel,fb-api", &pdata->use_fbapi, ret);

	/* enable / disable fb API */
	ret = of_property_read_u32(ngraphics, "intel,fb-nr-buffers",
			&pdata->fbapi_nr_buffers);
	if ((ret < 0) && (pdata->use_fbapi))
		pdata->fbapi_nr_buffers = 1;

	/* interface */
	nif = of_parse_phandle(ngraphics, PROP_DISPLAY_INTERFACE, 0);
	if (!nif) {
		dcc_err("Can't find node %s\n", PROP_DISPLAY_INTERFACE);
		return -1;
	}

	OF_CHECK_GET_U32(ngraphics, "intel,display-fps",
			&pdata->display.fps, ret);

	DCC_DBG3("Display interface %s\n", nif->name);
	if (strcmp(nif->name, "mipi-dbi") == 0) {
		struct dcc_display_if_mipi_dbi *dif =
					&pdata->display.dif.u.dbi;
		pdata->display.dif.type = DCC_IF_MIPI_DBI;
		OF_CHECK_GET_U32(nif, PROP_DISPLAY_IF_SEGMENTS,
			&dif->segments_per_pix, ret);
		OF_CHECK_GET_U32(nif, PROP_DISPLAY_IF_BITS,
			&dif->bits_per_segment, ret);
		/* mux */
		ret |= dcc_of_parse_mux(ngraphics,
				&dif->mux_params[0]);
		if (ret)
			DCC_DBG3("Node mux parsing failed\n");

		/* polarity */
		dif->periph_params.cs_polarity = 0;
		dif->periph_params.cd_polarity = 0;
		dif->periph_params.wr_polarity = 0;
		dif->periph_params.rd_polarity = 0;
		dif->periph_params.vd_polarity = 0;

		ret |= dcc_of_parse_display_timing(nif,	dif);
		if (ret)
			dcc_err("Node %s parsing failed\n", nif->name);

	} else if (strcmp(nif->name, "mipi-dsi") == 0) {
		struct dcc_display_if_mipi_dsi *dif =
					&pdata->display.dif.u.dsi;
		pdata->display.dif.type = DCC_IF_MIPI_DSI;
		OF_CHECK_GET_U32(ngraphics, "intel,display-if-rate",
				&dif->brdef, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-min",
				&dif->brmin, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-max",
				&dif->brmax, ret);
		OF_GET_U32(nif, "intel,display-if-phy-n",
				&dif->n, ret);
		OF_GET_U32(nif, "intel,display-if-phy-m",
				&dif->m, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-calib",
				&dif->calib, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-pwup",
				&dif->pwup, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-to_lp_hs_req",
				&dif->to_lp_hs_req, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-to_lp_hs_dis",
				&dif->to_lp_hs_dis, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-to_lp_hs_eot",
				&dif->to_lp_hs_eot, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-to_hs_zero",
				&dif->to_hs_zero, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-to_hs_flip",
				&dif->to_hs_flip, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-lp_clk_div",
				&dif->lp_clk_div, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-to_hs_clk_pre",
				&dif->to_hs_clk_pre, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-to_hs_clk_post",
				&dif->to_hs_clk_post, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-data_delay",
				&dif->data_delay, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-clock_delay",
				&dif->clock_delay, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-lp_tx_tfall",
				&dif->lp_tx_tfall, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-lp_tx_rise",
				&dif->lp_tx_trise, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-phy-lp_tx_vref",
				&dif->lp_tx_vref, ret);
		OF_CHECK_GET_U32(nif, "intel,display-if-nblanes",
				&dif->nblanes, ret);

		/* video/comand mode */
		if (pdata->display_autorefresh)
			dif->mode = DSI_VIDEO;
		else
			dif->mode = DSI_CMD;

		if (ret) {
			dcc_err("Node %s parsing failed\n", nif->name);
			return -EINVAL;
		}
	}


	/* features */
	ret = of_property_read_u32(ngraphics, "intel,dcc-use-fences",
			&pdata->use_fences);
	if (ret < 0)
		pdata->use_fences = 0;

	/* disable driver screen operations */
	ret = of_property_read_u32(ngraphics, "intel,display-preinit",
			&pdata->display_preinit);
	if (ret < 0)
		pdata->display_preinit = 0;

	/* debug */
	ret = of_property_read_u32(ngraphics, "intel,dcc-debug-level",
			&pdata->debug.level);
	if (ret < 0)
		pdata->debug.level = DCC_DEFAULT_DEBUG_LEVEL;

	/* tests */
	ret = of_property_read_u32(ngraphics, "intel,dcc-test-bootscreen",
			&pdata->test.bootscreen);
	if (ret < 0)
		pdata->test.bootscreen = 0;

	ret = of_property_read_u32(ngraphics,
			"intel,dcc-test-bootscreen-msdelay",
			&pdata->test.bootscreen_msdelay);
	if (ret < 0)
		pdata->test.bootscreen_msdelay = 500;

	ret = of_property_read_u32(ngraphics,
			"intel,lcd-bias-en-msdelay",
			&pdata->gpio_lcd_bias_msdelay);
	if (ret < 0)
		pdata->gpio_lcd_bias_msdelay = 10;

	ret = of_property_read_u32(ngraphics, "intel,dcc-test-mipidsi_vsync",
			&pdata->test.mipidsi_vsync);
	if (ret < 0)
		pdata->test.mipidsi_vsync = 0;

	/* pixel format */
	ret = of_property_read_string(
			ngraphics, PROP_DCC_FBPIXELFORMAT, &string);
	if (ret < 0) {
		dcc_err("%s MISSING pin property %s\n",
				ngraphics->name, PROP_DCC_FBPIXELFORMAT);
	}

	if (strncmp("RGB565", string, strlen("RGB565")) == 0) {
		pdata->fbfmt = DCC_FMT_RGB565;
		pdata->bytesppix = 2;
	} else if (strncmp("RGB888", string, strlen("RGB888")) == 0) {
		pdata->fbfmt = DCC_FMT_RGB888;
		pdata->bytesppix = 4;
	} else {
		dcc_err("%s unknown pixelformat %s\n",
				PROP_DCC_FBPIXELFORMAT, string);
		goto error;
	}
	dcc_boot_info("framebuffer pixelformat %s - %d bytes/pixel\n",
			string, pdata->bytesppix);

	return 0;
error:
	return -EINVAL;
}


int dcc_of_parse_dcc(struct platform_device *pdev, struct device_node *ndcc)
{
	int ret;
	struct dcc_drvdata *pdata =
	    (struct dcc_drvdata *)platform_get_drvdata(pdev);

	if (!pdata)
		return -EINVAL;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	pdata->pm_platdata = of_device_state_pm_setup(ndcc);
	if (IS_ERR(pdata->pm_platdata)) {
		dcc_err("Error during device state pm init\n");
		return -EINVAL;
	}
#endif

	/* supply */
#if !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	if (of_find_property(ndcc, "intel,supply", NULL)) {
		struct dcc_supply *supply;
		int i, len = of_property_count_strings(ndcc, "intel,supply");
		pdata->supply = (struct dcc_supply *)
			devm_kzalloc(&pdev->dev,
					sizeof(struct dcc_supply), GFP_KERNEL);
		if (!pdata->supply) {
			dcc_err("allocation of regulator failed\n");
			return -EINVAL;
		}
		DCC_DBG3("Found supply list of %d elements\n", len);
		INIT_LIST_HEAD(&pdata->supply->list);
		for (i = 0; i < len; i++) {
			char strvolt[128] = "\0";
			supply = (struct dcc_supply *)
				devm_kzalloc(&pdev->dev,
					sizeof(struct dcc_supply), GFP_KERNEL);
			if (!supply) {
				dcc_err("allocation of regulator failed\n");
				return -EINVAL;
			}
			of_property_read_string_index(ndcc, "intel,supply", i,
					&supply->name);
			/* get voltage value if any */
			sprintf(strvolt, "intel,%s-voltage", supply->name);
			ret = of_property_read_u32(ndcc, strvolt,
					&supply->voltage);
			if (ret)
				supply->voltage = 0;

			supply->regulator =
				regulator_get(pdata->dev, supply->name);
			if (IS_ERR(supply->regulator))
				dcc_err("can't get regulator %s",
						supply->name);
			list_add_tail(&supply->list,
					&pdata->supply->list);
			DCC_DBG3(
			"Add regulator %s(%d uV) to dcc regulator list\n",
			supply->name, supply->voltage);
		}
	}
#else
	pdata->pm_lcd =	regulator_get(pdata->dev, "lcd");
#endif

	/* clock */
#if !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	pdata->clk_pll = of_clk_get_by_name(ndcc, OF_PLL_CLK);
	if (IS_ERR(pdata->clk_pll))/* not mandatory clk */
		dcc_info("Clk %s not found\n", OF_PLL_CLK);

	pdata->clk_kernel = of_clk_get_by_name(ndcc, OF_KERNEL_CLK);
	if (IS_ERR(pdata->clk_kernel)) {
		dcc_err("Clk %s not found\n", OF_KERNEL_CLK);
		return -1;
	}
	pdata->clk_ahb = of_clk_get_by_name(ndcc, OF_AHB_CLK);
	if (IS_ERR(pdata->clk_ahb)) {
		dcc_err("Clk %s not found\n", OF_AHB_CLK);
		return -1;
	}
	pdata->clk_master = of_clk_get_by_name(ndcc, OF_MASTER_CLK);
	if (IS_ERR(pdata->clk_master)) {
		dcc_err("Clk %s not found\n", OF_MASTER_CLK);
		return -1;
	}
#endif

	/* reset */
	pdata->reset = devm_reset_control_get(pdata->dev, "dcc");
	if (IS_ERR(pdata->reset))
		pdata->reset = NULL;

	/* interrupts */
	ret = dcc_of_parse_irq(pdev, ndcc);
	if (ret < 0) {
		dcc_err("error parsing interrupts in %s\n", ndcc->name);
		goto error;
	}

	/* get cmd afterward delay */
	ret = of_property_read_u32(ndcc, "intel,dcc-clkrate",
			&pdata->clk_rate);
	if (ret) {
		dcc_err("Can't find node %s\n", "intel,dcc-clkrate");
		goto error;
	}

	pdata->gpio_reset = of_get_named_gpio_flags(ndcc,
			PROP_DISPLAY_GPIO_RST, 0, NULL);
	if (pdata->gpio_reset <= 0) {
		dcc_err("Can't find node %s\n", PROP_DISPLAY_GPIO_RST);
		goto error;
	}

	pdata->gpio_lcd_bias = of_get_named_gpio_flags(ndcc,
			PROP_DISPLAY_GPIO_LCD_BIAS, 0, NULL);
	if (pdata->gpio_lcd_bias <= 0)
		pdata->gpio_lcd_bias = 0;

	if (of_parse_phandle(ndcc, PROP_DISPLAY_GPIO_CD, 0)) {
		pdata->gpio_cd = of_get_named_gpio_flags(ndcc,
				PROP_DISPLAY_GPIO_CD, 0, NULL);
		if (pdata->gpio_cd <= 0) {
			dcc_err("Can't find node %s\n", PROP_DISPLAY_GPIO_CD);
			goto error;
		}
	}

	return 0;
error:
	return -EINVAL;
}

#ifdef DEBUG
void dump_cmds(struct display_msg *cmds, int ncmds)
{
	int i, j;

	for (i = 0; i < ncmds; i++) {
		char strdatas[512] = "\0";
		char strbyte[5] = "\0";

		sprintf(strbyte, "%02x ", cmds[i].header);
		strcat(strdatas, strbyte);
		for (j = 0; j < cmds[i].length; j++) {
			sprintf(strbyte, "%02x ", cmds[i].datas[j]);
			strcat(strdatas, strbyte);
		}
		DCC_DBG3("d(%dms) \tl(%02d) p(0x%02x)%s \t%s\n",
				cmds[i].delay, cmds[i].length,
				cmds[i].type, cmds[i].name, strdatas);
	}

}
#endif

int dcc_of_parse_display_reset(struct platform_device *pdev,
				struct device_node *n,
				struct display_reset **resetlist)
{
	int ret = 0, i, length = 0;
	unsigned int val;
	const __be32 *p;
	struct property *prop;
	int *array;
	struct display_reset *res;

	/* count array size */
	of_property_for_each_u32(n, "intel,display-reset", prop, p, val) {
		length++;
	};

	if (length == 0) {
		*resetlist = NULL;
		return 0;
	}

	if (length % 2) {
		dcc_err("intel,display-reset array length should be even\n");
		return -EINVAL;
	}

	array = devm_kzalloc(&pdev->dev, length*sizeof(int), GFP_KERNEL);
	if (!array)
		return -ENOMEM;

	ret = of_property_read_u32_array(n, "intel,display-reset",
					array, length);
	if (ret) { /* already checked few lines before but does not hurt */
		dcc_err("Can't read property:%s\n", "intel,display-resolution");
		*resetlist = NULL;
		return 0;
	}

	*resetlist = devm_kzalloc(&pdev->dev,
			sizeof(struct display_reset), GFP_KERNEL);
	if (!*resetlist) {
		dcc_err("Can't alloc array for display-reset length\n");
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&(*resetlist)->list);

	for (i = 0; i < length; i += 2) {
		res = (struct display_reset *)
			devm_kzalloc(&pdev->dev,
				sizeof(struct display_reset), GFP_KERNEL);
		if (!res) {
			dcc_err("allocation of reset failed\n");
			return -EINVAL;
		}
		res->value = array[i];
		res->mdelay = array[i+1];
		list_add_tail(&res->list, &(*resetlist)->list);
	}
	devm_kfree(&pdev->dev, array);
	return 0;
}


int dcc_of_parse_display_cmd(struct platform_device *pdev,
				struct device_node *n,
				struct display_msg *cmd)
{
#define PROP_DISPLAY_CMDDATA	"intel,cmd-data"

	int ret = 0, i;
	u32 val;
	const __be32 *p;
	struct property *prop;

	cmd->flags = 0;
	/* count array size */
	cmd->length = 0;
	of_property_for_each_u32(n, PROP_DISPLAY_CMDDATA, prop, p, val) {
		cmd->length++;
	};
	cmd->length--; /* minus header byte */

	/* allocate data array if needed */
	if (cmd->length > 0) {
		cmd->datas = devm_kzalloc(&pdev->dev,
				cmd->length*sizeof(u8), GFP_KERNEL);
		if (!cmd->datas) {
			dcc_err("Can't alloc array for %s length %dbytes\n",
					n->name, cmd->length);
			return -ENOMEM;
		}
	}
	cmd->name = n->name;

	/* populate header+data */
	i = -1;
	of_property_for_each_u32(n, PROP_DISPLAY_CMDDATA, prop, p, val) {
		if (i == -1)
			cmd->header = val;
		else if (cmd->datas)
			cmd->datas[i] = val;
		i++;
	}

	/* get cmd afterward delay */
	ret = of_property_read_u32(n, "intel,cmd-type", &val);
	if (ret)
		/* we don't care as this is an optional property */
		cmd->type = 0;
	else
		cmd->type = val;

	/* get cmd afterward delay */
	ret = of_property_read_u32(n, PROP_DISPLAY_CMDDELAY, &cmd->delay);
	if (ret)
		/* we don't care as this is an optional property */
		cmd->delay = 0;

	/* get cmd afterward delay */
	ret = of_property_read_u32(n, "intel,cmd-lp", &val);
	if (!ret && val)
		cmd->flags |= LCD_MSG_LP;


	DCC_DBG3("\tcmd 0x%02x type: 0x%02x delay(%d) length(%d) %s\n",
		cmd->header, cmd->type, cmd->delay, cmd->length, cmd->name);

	return 0;
}

int dcc_of_parse_display_msglist(struct platform_device *pdev,
				struct device_node *n,
				struct display_msg **msglist)
{
	int ret = 0, ncmds;
	struct device_node *child;
	struct display_msg *msg;

	/* count group nodes */
	ncmds = of_get_child_count(n);

	DCC_DBG3("%s command list: %d commands\n", n->name, ncmds);

	/* allocate cmd list */
	*msglist = devm_kzalloc(&pdev->dev,
			sizeof(struct display_msg), GFP_KERNEL);
	if (!*msglist) {
		dcc_err("Can't alloc commands table\n");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&(*msglist)->list);
	for_each_child_of_node(n, child) {
		msg = (struct display_msg *)
			devm_kzalloc(&pdev->dev,
				sizeof(struct display_msg), GFP_KERNEL);
		if (!msg) {
			dcc_err("allocation of display msg failed\n");
			return -EINVAL;
		}

		dcc_of_parse_display_cmd(pdev, child, msg);
		list_add_tail(&msg->list, &(*msglist)->list);
	}

	return ret;
}

int dcc_of_parse_display(struct platform_device *pdev,
			struct device_node *ndisplay)
{
	int ret;
	u32 array[2];
	struct device_node *child;
	struct dcc_drvdata *pdata =
	    (struct dcc_drvdata *)platform_get_drvdata(pdev);
	struct dcc_display *display = &pdata->display;

	if (!pdata)
		return -EINVAL;

	/* resolution */
	ret = of_property_read_u32_array(ndisplay,
			"intel,display-resolution", array, 2);
	if (ret) {
		dcc_err("Can't read property:%s\n", "intel,display-resolution");
		return -EINVAL;
	}
	display->xres = array[0];
	display->yres = array[1];
	DCC_DBG3("Display device %dx%d\n", display->xres, display->yres);

	/* dpi */
	ret = of_property_read_u32_array(ndisplay,
			"intel,display-dpi", array, 2);
	if (ret) {
		dcc_err("Can't read property:%s\n", "intel,display-dpi");
		return -EINVAL;
	}
	display->xdpi = array[0];
	display->ydpi = array[1];

	if (DISPLAY_IS_MIPI_DSI_IF(display->dif.type)) {

		const char *string;
		struct dcc_display_if_mipi_dsi *dif =
					&pdata->display.dif.u.dsi;

		OF_GET_U32(ndisplay, "intel,display-vid-hfp-hs-bytes",
				&dif->hfp, ret);
		if (dif->hfp)
			dif->hfp_lp = 0;
		else {
			OF_GET_U32(ndisplay, "intel,display-vid-hfp-lp-cycles",
					&dif->hfp, ret);
			if (dif->hfp)
				dif->hfp_lp = 1;
		}

		OF_GET_U32(ndisplay, "intel,display-vid-hbp-hs-bytes",
				&dif->hbp, ret);
		if (dif->hbp)
			dif->hbp_lp = 0;
		else {
			OF_GET_U32(ndisplay, "intel,display-vid-hbp-lp-cycles",
					&dif->hbp, ret);
			if (dif->hbp)
				dif->hbp_lp = 1;
		}

		OF_GET_U32(ndisplay, "intel,display-vid-hsa-hs-bytes",
				&dif->hsa, ret);
		if (dif->hsa)
			dif->hsa_lp = 0;
		else {
			OF_GET_U32(ndisplay, "intel,display-vid-hsa-lp-cycles",
					&dif->hsa, ret);
			if (dif->hsa)
				dif->hsa_lp = 1;
		}
		OF_GET_U32(ndisplay, "intel,display-vid-vfp-lines",
				&dif->vfp, ret);
		OF_GET_U32(ndisplay, "intel,display-vid-vbp-lines",
				&dif->vbp, ret);
		OF_GET_U32(ndisplay, "intel,display-vid-vsa-lines",
				&dif->vsa, ret);

		ret = of_property_read_string(
				ndisplay, "intel,display-vid-mode", &string);
		if (ret < 0)
			dif->video_mode = 0;

		if (strncmp("active", string, strlen("active")) == 0) {
			dif->video_mode = DSI_ACTIVE;
		} else if (strncmp("pulses", string, strlen("pulses")) == 0) {
			dif->video_mode = DSI_PULSES;
		} else if (strncmp("events", string, strlen("events")) == 0) {
			dif->video_mode = DSI_EVENTS;
		} else if (strncmp("burst", string, strlen("burst")) == 0) {
			dif->video_mode = DSI_BURST;
		} else {
			dcc_err("%s unknown dsi video mode type %s\n",
					"intel,display-vid-mode", string);
		}

		OF_GET_U32(ndisplay, "intel,display-vid-id", &dif->id, ret);

		ret = of_property_read_string(
				ndisplay, "intel,display-vid-pixel", &string);
		if (ret < 0)
			dif->video_pixel = 0;

		if (strncmp("16packed", string,
					strlen("16packed")) == 0) {
			dif->video_pixel = DSI_PIX_BIT16P;
		} else if (strncmp("18packed", string,
					strlen("18packed")) == 0) {
			dif->video_pixel = DSI_PIX_BIT18P;
		} else if (strncmp("18loose", string,
					strlen("18loose")) == 0) {
			dif->video_pixel = DSI_PIX_BIT18L;
		} else if (strncmp("24packed", string,
					strlen("24packed")) == 0) {
			dif->video_pixel = DSI_PIX_BIT24P;
		} else {
			dcc_err("%s unknown dsi video pixel %s\n",
					"intel,display-vid-pixel", string);
		}
	}

	dcc_of_parse_display_reset(pdev, ndisplay, &display->reset);

	for_each_child_of_node(ndisplay, child) {
		if (strcmp(child->name, "cmd-init") == 0) {
			ret = dcc_of_parse_display_msglist(pdev, child,
					&display->msgs_init);
			if (ret) {
				DCC_DBG3("Node %s parsing failed\n",
						child->name);
				/* this is not a mandatory section */
			}
		} else if (strcmp(child->name, "cmd-update") == 0) {
			ret = dcc_of_parse_display_msglist(pdev, child,
					&display->msgs_update);
			if (ret)
				dcc_err("Node %s parsing failed\n",
						child->name);

		} else if (strcmp(child->name, "cmd-power-on") == 0) {
			ret = dcc_of_parse_display_msglist(pdev, child,
					&display->msgs_power_on);
			if (ret)
				dcc_err("Node %s parsing failed\n",
						child->name);

		} else if (strcmp(child->name, "cmd-power-off") == 0) {
			ret = dcc_of_parse_display_msglist(pdev, child,
					&display->msgs_power_off);
			if (ret)
				dcc_err("Node %s parsing failed\n",
						child->name);

		} else if (strcmp(child->name, "cmd-sleep-in") == 0) {
			ret = dcc_of_parse_display_msglist(pdev, child,
					&display->msgs_sleep_in);
			if (ret)
				dcc_err("Node %s parsing failed\n",
						child->name);

		} else if (strcmp(child->name, "cmd-sleep-out") == 0) {
			ret = dcc_of_parse_display_msglist(pdev, child,
					&display->msgs_sleep_out);
			if (ret)
				dcc_err("Node %s parsing failed\n",
						child->name);

		} else {
			DCC_DBG3("In node %s, unexpected child %s !\n",
					ndisplay->name, child->name);
		}

	};

	return ret;
}

int dcc_of_parse(struct platform_device *pdev)
{
	int ret;
	struct device_node *ndcc, *ndisplay, *ngraphics;

	/* Parse root node devicetree */
	ndcc = pdev->dev.of_node;
	if (!ndcc)
		DCC_DBG3("Can't find dcc matching node\n");

	ndisplay = of_find_matching_node(NULL, xgold_display_of_match);
	if (!ndisplay)
		DCC_DBG3("Can't find display matching node\n");

	ngraphics = of_find_matching_node(NULL, xgold_graphics_of_match);
	if (!ndisplay)
		DCC_DBG3("Can't find graphics matching node\n");

	if (!ndcc || !ndisplay || !ngraphics) {
		ret = -1;
		goto exit;
	}

	ret = dcc_of_parse_dcc(pdev, ndcc);
	if (ret) {
		dcc_err("%s failed parsing %s node\n", __func__, ndcc->name);
		goto exit;
	}

	ret = dcc_of_parse_graphics(pdev, ngraphics);
	if (ret) {
		dcc_err("%s failed parsing %s node\n", __func__, ndcc->name);
		goto exit;
	}

	ret = dcc_of_parse_display(pdev, ndisplay);
	if (ret) {
		dcc_err("%s failed parsing %s node\n",
				__func__, ndisplay->name);
		goto exit;
	}

	DCC_DBG3("device tree parsed successfully.\n");
exit:
	of_node_put(ndisplay);
	of_node_put(ndcc);
	of_node_put(ngraphics);

	return ret;
}
