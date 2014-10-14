/*
 * Copyright (c) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mmc/host.h>
#include "sdhci-pltfm.h"
#include "sdhci-of-imc.h"
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/mv_svc_hypercalls.h>
#endif

#define XGOLD_DEFAULT_QUIRKS  (SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK \
				| SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN)
#define XGOLD_DEFAULT_QUIRKS2  0
/*TO DO: remove quirk for card detection when RTC enabled */

#ifdef CONFIG_PLATFORM_DEVICE_PM
#define SDHCI_PM_D3	0
#define SDHCI_PM_D0	1
#define SDHCI_PM_D0i2	2
#define SDHCI_PM_D0i3	3

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT

static int xgold_sdhci_ctrl_set_pm_state(struct device *,
		struct device_state_pm_state *);
static struct device_state_pm_state *xgold_sdhci_ctrl_get_initial_state(
		struct device *);

static struct device_state_pm_ops sdhci_pm_ops = {
	.set_state = xgold_sdhci_ctrl_set_pm_state,
	.get_initial_state = xgold_sdhci_ctrl_get_initial_state,
};

/* clocks PM states & class */
static struct device_state_pm_state sdhci_pm_states[] = {
	{ .name = "disable", }, /* D3 */
	{ .name = "high_perf", }, /* D0 */
	{ .name = "mid_perf", }, /* D0i2 */
};

DECLARE_DEVICE_STATE_PM_CLASS(sdhci);

#endif
#endif

static inline int xgold_sdhci_set_pinctrl_state(struct device *dev,
						struct pinctrl_state *state)
{
	int ret = 0;
	struct xgold_mmc_pdata *pdata = dev_get_platdata(dev);

	if (!pdata) {
		dev_err(dev, "Unable to retrieve mmc platform data\n");
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	return ret;
}


static unsigned int xgold_sdhci_of_get_max_clock(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	return mmc_pdata->max_clock;
}
static unsigned int xgold_sdhci_of_get_min_clock(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct device_node *np = pdev->dev.of_node;
	unsigned int min_clock;
	of_property_read_u32(np, "intel,min_clock", &min_clock);
	return min_clock;
}

int xgold_sdhci_of_set_timing(struct sdhci_host *host, unsigned int uhs)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	unsigned tap_index;
	int ret = 0;
	int mode = 0;

	if (mmc_pdata->tap_reg) {
		switch (uhs) {
		case MMC_TIMING_LEGACY:
			tap_index = 0;
			break;
		case MMC_TIMING_MMC_HS:
			tap_index = 1;
			mode = 1;
			break;
		case MMC_TIMING_SD_HS:
			tap_index = 3;
			mode = 3;
			break;
		case MMC_TIMING_UHS_DDR50:
			tap_index = 5;
			mode = 2;
			break;
		case MMC_TIMING_UHS_SDR50:
			tap_index = 4;
			mode = 4;
			break;
		case MMC_TIMING_UHS_SDR104:
			tap_index = 6;
			mode = 6;
			break;
		default:
			tap_index = 0;
			break;
		}
		dev_dbg(&pdev->dev, "Set tap values to mode %d\n", mode);
#ifdef CONFIG_X86_INTEL_SOFIA
		if (mmc_pdata->io_master == SCU_IO_ACCESS_BY_VMM) {
			if (mv_svc_reg_write((uint32_t)mmc_pdata->tap_reg,
						mmc_pdata->tap_values[tap_index],
						-1))
				dev_err(&pdev->dev, "mv_svc_reg_write_service fails @%#x\n",
						(uint32_t)mmc_pdata->tap_reg);
		} else
#endif
			iowrite32(mmc_pdata->tap_values[tap_index],
					(void __iomem *)mmc_pdata->tap_reg);

		if (mmc_pdata->tap_reg2) {
#ifdef CONFIG_X86_INTEL_SOFIA
			if (mmc_pdata->io_master == SCU_IO_ACCESS_BY_VMM) {
				if (mv_svc_reg_write((uint32_t)mmc_pdata->tap_reg2,
						mmc_pdata->tap_values2[tap_index],
						-1))
					dev_err(&pdev->dev, "mv_svc_reg_write_service fails @%#x\n",
						(uint32_t)mmc_pdata->tap_reg2);
			} else
#endif
				iowrite32(mmc_pdata->tap_values2[tap_index],
					(void __iomem *)mmc_pdata->tap_reg2);
		}
	}
	return ret;
}
/*
 * Max clock card we have to deal with :
 * 1 - SD 1.0 : 25 MHz  ( obsolete )
 * 2 - MMC 3.3 : 20 Mhz ( obsolete )
 * 3 - SD 2.0 :  50 Mhz
 * 4 - MMC 4.4 : 26 - 52 Mhz
 * 5 - SD 3.0 : 25 - 50 - 100 - 208
 *
 * Therefore, we have to deal with PLLA (208 Mhz) and Phs4 (96Mhz)
 */
static void xgold_sdhci_of_set_clock(struct sdhci_host *host,
							unsigned int clock)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	unsigned int request_clock = 0;

	switch (clock) {
	case 26000000:
	case 52000000:
#if defined CONFIG_MMC_XGOLD_FORCE_48M
		request_clock = 48000000;
		break;
#endif
	case 208000000:
	case 96000000:
	case 48000000:
	case 24000000:
		request_clock = clock;
		break;
	case 25000000:
		request_clock = 24000000;
		break;
	case 50000000:
		request_clock = 48000000;
		break;
	case 100000000:
		request_clock = 96000000;
		break;
	default:
		return;
	}

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (host->max_clk != request_clock) {
		host->max_clk = mmc_pdata->max_clock;
#else
	if (host->max_clk != request_clock) {
		clk_set_rate(mmc_pdata->master_clk, request_clock);
		host->max_clk = clk_get_rate(mmc_pdata->master_clk);
#endif
	}
}
/*****************************************************************************\
 *                                                                           *
 * Suspend/resume                                                            *
 *                                                                           *
\*****************************************************************************/

#ifdef CONFIG_PM

static void xgold_sdhci_of_suspend(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	if (device_may_wakeup(&pdev->dev)) {
		enable_irq_wake(mmc_pdata->irq_wk);
		enable_irq(mmc_pdata->irq_wk);
	}
/* TODO: called before sleep commands for card... should not stop clock ! */
#if 0
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	if (mmc_pdata->bus_clk)
		clk_disable(mmc_pdata->bus_clk);
	if (mmc_pdata->master_clk)
		clk_disable(mmc_pdata->master_clk);
#endif
}

static void xgold_sdhci_of_resume(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	if (device_may_wakeup(&pdev->dev)) {
		disable_irq_wake(mmc_pdata->irq_wk);
		disable_irq(mmc_pdata->irq_wk);
	}
#if 0
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	if (mmc_pdata->master_clk)
		clk_enable(mmc_pdata->master_clk);
	if (mmc_pdata->bus_clk)
		clk_enable(mmc_pdata->bus_clk);
#endif
}

#endif
static void xgold_sdhci_of_init(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct device_node *np = pdev->dev.of_node;
	unsigned int bits;
	if ((of_property_read_u32(np, "intel,is_8_bits", &bits) == 0) &&
		bits == 1)
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;

	if ((of_property_read_u32(np, "intel,is_non_removable", &bits) == 0) &&
		bits == 1)
		host->mmc->caps |= MMC_CAP_NONREMOVABLE;
}

static struct sdhci_ops xgold_sdhci_ops = {
	.set_clock = xgold_sdhci_of_set_clock,
	.get_max_clock = xgold_sdhci_of_get_max_clock,
	.get_min_clock = xgold_sdhci_of_get_min_clock,
	.set_uhs_signaling = xgold_sdhci_of_set_timing,
#ifdef CONFIG_PM
	.platform_suspend = xgold_sdhci_of_suspend,
	.platform_resume = xgold_sdhci_of_resume,
#endif
	.platform_init = xgold_sdhci_of_init,
};

static struct sdhci_pltfm_data sdhci_xgold_pdata = {
	.quirks = XGOLD_DEFAULT_QUIRKS,
	.quirks2 = XGOLD_DEFAULT_QUIRKS2,
	.ops = &xgold_sdhci_ops,
};

static irqreturn_t xgold_detect(int irq, void *dev_id)
{
	pr_info("%s: SD card inserted\n", __func__);
	return IRQ_HANDLED;
}

/*
 * Get intel,io-access property if any from dts
 */
bool xgold_sdhci_get_io_master(struct device_node *np)
{
	if (of_find_property(np, "intel,io-access-guest", NULL))
		return SCU_IO_ACCESS_BY_LNX;
	return SCU_IO_ACCESS_BY_VMM;
}

static int xgold_sdhci_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct xgold_mmc_pdata *mmc_pdata;
	struct resource res;
	void __iomem *scu_base;
	u32 offset;
	void __iomem *corereg;
	int it_wk, i;
#if defined CONFIG_PLATFORM_DEVICE_PM && defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	struct device_node *pm_node;
#endif
	int ret;
	unsigned int quirktab[2];
	mmc_pdata = devm_kzalloc(&pdev->dev, sizeof(*mmc_pdata), GFP_KERNEL);
	if (!mmc_pdata)
		return -ENOMEM;
	pdev->dev.platform_data = mmc_pdata;
	of_property_read_u32(np, "intel,max_clock", &mmc_pdata->max_clock);
	of_property_read_u32(np, "intel,min_clock", &mmc_pdata->min_clock);
	ret = of_address_to_resource(of_parse_phandle(np,
			"intel,tap-hwbase", 0), 0, &res);
	if (ret)
		dev_err(&pdev->dev, "no address base for tap values\n");

	mmc_pdata->io_master = xgold_sdhci_get_io_master(np);
	if (mmc_pdata->io_master == SCU_IO_ACCESS_BY_LNX)
		scu_base = devm_ioremap(&pdev->dev, res.start,
				resource_size(&res));
	else
		scu_base = (void __iomem *)res.start;
	pr_info("sdhci: io:%s-@:%#x\n",
		mmc_pdata->io_master == SCU_IO_ACCESS_BY_LNX ?
		"linux" : "vmm", (uint32_t)scu_base);
	of_property_read_u32(np, "intel,tap_reg", &offset);
	mmc_pdata->tap_reg = (void *)scu_base + offset;
	if (!of_property_read_u32(np, "intel,tap_reg2", &offset))
		mmc_pdata->tap_reg2 = (void *)scu_base + offset;
	else
		mmc_pdata->tap_reg2 = NULL;

	ret |= of_property_read_u32_array(np, "intel,tap_values",
					&mmc_pdata->tap_values[0], 7);
	if (ret) {
		dev_dbg(&pdev->dev, "no tap values\n");
		mmc_pdata->tap_reg = 0;
		mmc_pdata->tap_reg2 = 0;
	} else
		of_property_read_u32_array(np, "intel,tap_values2",
						&mmc_pdata->tap_values2[0], 7);

	/* correct corecfg register if needed */
	for (i = 0; i < 5; i++) {
		if (!of_property_read_u32_index(np, "intel,corecfg_reg",
				i, &offset)) {
			corereg = scu_base + offset;
			if (!of_property_read_u32_index(np,
					"intel,corecfg_val", i , &offset)) {
#ifdef CONFIG_X86_INTEL_SOFIA
				if (mmc_pdata->io_master == SCU_IO_ACCESS_BY_VMM) {
					if (mv_svc_reg_write((uint32_t)corereg, offset, -1))
						dev_err(&pdev->dev, "mv_svc_reg_write_service fails @%#x\n", (uint32_t)corereg);
				} else
#endif
					writel(offset, corereg);
			}
		}
	}

	/* correct corecfg register if needed */
	if (!of_property_read_u32(np, "intel,corecfg_reg", &offset)) {
		corereg = scu_base + offset;
		if (!of_property_read_u32(np, "intel,corecfg_val", &offset))
			writel(offset, corereg);
	}

	of_property_read_u32_array(np, "intel,quirks", &quirktab[0], 2);

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	/* clock */
	mmc_pdata->bus_clk = of_clk_get_by_name(np, "clk_ahb");
	if (IS_ERR(mmc_pdata->bus_clk)) {
		dev_dbg(&pdev->dev, "clk_ahb not found\n");
		return -1;
	}
	mmc_pdata->master_clk = of_clk_get_by_name(np, "clk_core");
	if (IS_ERR(mmc_pdata->master_clk)) {
		dev_dbg(&pdev->dev, "clk_core not found\n");
		return -1;
	}
#endif

#if defined CONFIG_PLATFORM_DEVICE_PM && defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	pm_node = of_get_child_by_name(np, "clock_ctrl");
	mmc_pdata->pm_platdata_clock_ctrl = of_device_state_pm_setup(pm_node);

	if (IS_ERR(mmc_pdata->pm_platdata_clock_ctrl)) {
		dev_err(&pdev->dev, "Error during device state pm clock init\n");
		kfree(mmc_pdata);
		return -1;
	}
	/* set pm classes */
	ret = device_state_pm_set_class(&mmc_pdata->dev,
		mmc_pdata->pm_platdata_clock_ctrl->pm_user_name);
	    if (ret) {
		dev_err(&pdev->dev, "Error while setting the pm clock ctrl class\n");
		kfree(mmc_pdata);
		return -1;
	}
	device_state_pm_set_state_by_name(&mmc_pdata->dev,
			mmc_pdata->pm_platdata_clock_ctrl->pm_state_D0_name);
#else
	if (clk_prepare(mmc_pdata->bus_clk)) {
		dev_dbg(&pdev->dev, "clk_ahb prepare failed\n");
		return -1;
	}
	if (clk_enable(mmc_pdata->bus_clk)) {
		dev_dbg(&pdev->dev, "clk_ahb enable failed\n");
		return -1;
	}

	if (clk_prepare(mmc_pdata->master_clk)) {
		dev_dbg(&pdev->dev, "clk_core prepare failed\n");
		return -1;
	}
	if (clk_enable(mmc_pdata->master_clk)) {
		dev_dbg(&pdev->dev, "clk_core enable failed\n");
		return -1;
	}
#endif
	/* pcl */
	mmc_pdata->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(mmc_pdata->pinctrl)) {
		dev_err(&pdev->dev, "unable to get pinctrl for sdhci\n");
		goto err_end;
	}
	mmc_pdata->pins_default = pinctrl_lookup_state(mmc_pdata->pinctrl,
						 PINCTRL_STATE_DEFAULT);
	if (IS_ERR(mmc_pdata->pins_default))
		dev_err(&pdev->dev, "could not get default pinstate\n");

	mmc_pdata->pins_sleep = pinctrl_lookup_state(mmc_pdata->pinctrl,
					       PINCTRL_STATE_SLEEP);
	if (IS_ERR(mmc_pdata->pins_sleep))
		dev_err(&pdev->dev, "could not get sleep pinstate\n");

	mmc_pdata->pins_inactive = pinctrl_lookup_state(mmc_pdata->pinctrl,
					       "inactive");
	if (IS_ERR(mmc_pdata->pins_inactive))
		dev_err(&pdev->dev, "could not get inactive pinstate\n");


	xgold_sdhci_set_pinctrl_state(&pdev->dev, mmc_pdata->pins_default);

	it_wk = of_property_match_string(np, "interrupt-names",
					 "wake");
	if (it_wk > 0) {
		mmc_pdata->irq_wk = irq_of_parse_and_map(np, it_wk);
		device_init_wakeup(&pdev->dev, 1);
		ret = devm_request_irq(&pdev->dev, mmc_pdata->irq_wk,
			xgold_detect,
			IRQF_SHARED | IRQF_NO_SUSPEND, "wk_int", mmc_pdata);
		if (ret != 0) {
			dev_err(&pdev->dev,
				"setup irq%d failed with ret = %d\n",
				mmc_pdata->irq_wk, ret);
			return -1;
		}
		disable_irq(mmc_pdata->irq_wk);
	}
	/* quirks */
	sdhci_xgold_pdata.quirks |= quirktab[0];
	sdhci_xgold_pdata.quirks2 |= quirktab[1];
	ret = sdhci_pltfm_register(pdev, &sdhci_xgold_pdata, 0);
err_end:
	return ret;
}

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
static int xgold_sdhci_ctrl_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	struct xgold_mmc_pdata *mmc_pdata = dev_get_platdata(dev);

	if (!strcmp(state->name, sdhci_pm_states[SDHCI_PM_D0].name)) {
		if (clk_prepare(mmc_pdata->bus_clk)) {
			dev_dbg(dev, "clk_ahb prepare failed\n");
			return -1;
		}
		if (clk_enable(mmc_pdata->bus_clk)) {
			dev_dbg(dev, "clk_ahb enable failed\n");
			return -1;
		}
		if (clk_prepare(mmc_pdata->master_clk)) {
			dev_dbg(dev, "clk_core prepare failed\n");
			return -1;
		}
		if (clk_enable(mmc_pdata->master_clk)) {
			dev_dbg(dev, "clk_core enable failed\n");
			return -1;
		}
	/*	clk_set_rate(mmc_pdata->master_clk, 52000000); */
	} else if (!strcmp(state->name, sdhci_pm_states[SDHCI_PM_D0i2].name)) {
		clk_set_rate(mmc_pdata->master_clk, mmc_pdata->max_clock);
	} else if (!strcmp(state->name, sdhci_pm_states[SDHCI_PM_D3].name)) {
		/* TO DO */
	} else
		return -EINVAL;

	return 0;

}
static struct device_state_pm_state *xgold_sdhci_ctrl_get_initial_state(
		struct device *dev)
{
	return &sdhci_pm_states[SDHCI_PM_D3];
}
#endif

static int xgold_sdhci_remove(struct platform_device *pdev)
{
	return sdhci_pltfm_unregister(pdev);
}

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
int xgold_sdhci_pm_init(void)
{
	int ret;
	ret = device_state_pm_add_class(&sdhci_pm_class);
	return ret;

}
subsys_initcall(xgold_sdhci_pm_init);
#endif

static const struct of_device_id xgold_sdhci_of_match[] = {
	{ .compatible = "intel,sdhci" },
	{ }
};
MODULE_DEVICE_TABLE(of, xgold_sdhci_of_match);

static struct platform_driver xgold_sdhci_driver = {
	.driver = {
		.name = "xgold-sdhci",
		.owner = THIS_MODULE,
		.of_match_table = xgold_sdhci_of_match,
		.pm = SDHCI_PLTFM_PMOPS,
	},
	.probe = xgold_sdhci_probe,
	.remove = xgold_sdhci_remove,
};

module_platform_driver(xgold_sdhci_driver);

MODULE_DESCRIPTION("SDHCI for XGOLD driver");
MODULE_LICENSE("GPL v2");
