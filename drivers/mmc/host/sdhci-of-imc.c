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
#include <linux/mmc/slot-gpio.h>
#include <linux/pm_runtime.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/mv_svc_hypercalls.h>
#endif

#include "sdhci-pltfm.h"
#include "sdhci-of-imc.h"

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

static void xgold_default_regs_fixup(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	u32 offset, value;
	int i;

	for (i = 0; i < xgold_mmc_total_regs; i++) {
		if (!of_property_read_u32_index(np, "intel,corecfg_reg",
				i, &offset))
			if (!of_property_read_u32_index(np,
					"intel,corecfg_val", i , &value))
				xgold_sdhci_scu_write(pdata, offset, value);
	}
}

int xgold_sdhci_of_set_timing(struct sdhci_host *host, unsigned int uhs)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	unsigned tap_index;
	int mode = 0;

	if (mmc_pdata->tap_reg_offset == -1)
		goto out;

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
	xgold_sdhci_scu_write(mmc_pdata, mmc_pdata->tap_reg_offset,
			      mmc_pdata->tap_values[tap_index]);

	if (mmc_pdata->tap_reg2_offset > 0)
		xgold_sdhci_scu_write(mmc_pdata, mmc_pdata->tap_reg2_offset,
			      mmc_pdata->tap_values2[tap_index]);
	/*
	 * Some fixup needed here...
	 */
	if (mmc_pdata->fixup & XGOLD_DEFAULT_REGS_FIXUP)
		xgold_default_regs_fixup(host);
out:
	return 0;
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

int xgold_sdhci_of_select_card_ds(struct sdhci_host *host, unsigned int dtr)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;

	/* TODO: return drive strength function of speed */
	return mmc_pdata->card_drive_strength[0];
}

void xgold_sdhci_of_card_status(struct sdhci_host *host, bool status)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	if (!status) {
		dev_dbg(&pdev->dev, "card event: card removed\n");
		/* pull down on pads */
		xgold_sdhci_set_pinctrl_state(&pdev->dev,
				mmc_pdata->pins_inactive);
		mmc_pdata->pins_restore_default = 0;
	} else {
		/* pull up on pads */
		dev_dbg(&pdev->dev, "card event: card inserted\n");
		xgold_sdhci_set_pinctrl_state(&pdev->dev,
				mmc_pdata->pins_default);
		mmc_pdata->pins_restore_default = 1;
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
#ifdef CONFIG_PM_RUNTIME
	if (host->runtime_suspended)
		pm_runtime_resume(&pdev->dev);
#endif
	if (device_may_wakeup(&pdev->dev)) {
		enable_irq_wake(mmc_pdata->irq_wk);
		enable_irq(mmc_pdata->irq_wk);
	}

#if defined CONFIG_PLATFORM_DEVICE_PM && defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	if (device_state_pm_set_state_by_name(&mmc_pdata->dev,
			mmc_pdata->pm_platdata_clock_ctrl->pm_state_D3_name))
		dev_err(&pdev->dev, "set pm state D0i2 during runtime suspend failed !\n");
	/* on SF boards sd cards power (vmmc) is enabled by IOs power (vqmmc):
	 * need to disable at suspend otherwise resume is failing
	 * (card not resetted)
	 * kernel legacy sdhci is powering off vmmc at suspend */
	if (host->vqmmc)
		regulator_disable(host->vqmmc);
#endif
}

static void xgold_sdhci_of_resume(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	u32 timeout = 1000;
	if (device_may_wakeup(&pdev->dev)) {
		disable_irq_wake(mmc_pdata->irq_wk);
		disable_irq_nosync(mmc_pdata->irq_wk);
	}
#if defined CONFIG_PLATFORM_DEVICE_PM && defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	if (host->vqmmc)
		regulator_enable(host->vqmmc);

	if (device_state_pm_set_state_by_name(&mmc_pdata->dev,
	    mmc_pdata->pm_platdata_clock_ctrl->pm_state_D0_name))
		dev_err(&pdev->dev, "set pm state D0 during runtime resume  failed !\n");
#endif
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
#endif

	/*
	 * Timeout after 250 ms to 500 ms if SDHCI_CARD_DET_STABLE
	 * doesn't happen.
	 */
	while (!(sdhci_readl(host, SDHCI_PRESENT_STATE) &
			      SDHCI_CARD_DET_STABLE) && --timeout)
		usleep_range(250, 500);

	WARN(!timeout, "Timeout waiting for SDHCI_CARD_DET_STABLE: %s\n",
			mmc_hostname(host->mmc));
}

#endif
#ifdef CONFIG_PM_RUNTIME
static void xgold_cd_irq_disable(struct xgold_mmc_pdata *pd)
{
	unsigned long irqflags;

	spin_lock_irqsave(&pd->irq_lock, irqflags);
	if (!pd->irq_is_disable) {
		pd->irq_is_disable = 1;
		disable_irq_nosync(pd->irq_eint);
	}
	spin_unlock_irqrestore(&pd->irq_lock, irqflags);
}

static void xgold_cd_irq_enable(struct xgold_mmc_pdata *pd)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&pd->irq_lock, irqflags);
	if (pd->irq_is_disable) {
		enable_irq(pd->irq_eint);
		pd->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&pd->irq_lock, irqflags);
}


static void xgold_sdhci_of_runtime_suspend(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	xgold_sdhci_set_pinctrl_state(&pdev->dev,
			(mmc_pdata->pins_restore_default == 1 ?
			mmc_pdata->pins_sleep : mmc_pdata->pins_inactive));
#if defined CONFIG_PLATFORM_DEVICE_PM && defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	if (device_state_pm_set_state_by_name(&mmc_pdata->dev,
			mmc_pdata->pm_platdata_clock_ctrl->pm_state_D0i2_name))
		dev_err(&pdev->dev, "set pm state D0i2 during runtime suspend failed !\n");
#endif
	if (mmc_pdata->irq_eint)
		xgold_cd_irq_enable(mmc_pdata);
}

static void xgold_sdhci_of_runtime_resume(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	u32 timeout = 1000;

	if (mmc_pdata->irq_eint)
		xgold_cd_irq_disable(mmc_pdata);
	xgold_sdhci_set_pinctrl_state(&pdev->dev,
			(mmc_pdata->pins_restore_default == 1 ?
			mmc_pdata->pins_default : mmc_pdata->pins_inactive));
#if defined CONFIG_PLATFORM_DEVICE_PM && defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	if (device_state_pm_set_state_by_name(&mmc_pdata->dev,
			mmc_pdata->pm_platdata_clock_ctrl->pm_state_D0_name))
		dev_err(&pdev->dev, "set pm state D0 during runtime resume  failed !\n");
#endif

	/*
	 * Timeout after 250 ms to 500 ms if SDHCI_CARD_DET_STABLE
	 * doesn't happen.
	 */
	while (!(sdhci_readl(host, SDHCI_PRESENT_STATE) &
			      SDHCI_CARD_DET_STABLE) && --timeout)
		usleep_range(250, 500);

	WARN(!timeout, "Timeout waiting for SDHCI_CARD_DET_STABLE: %s\n",
			mmc_hostname(host->mmc));
}

#endif

bool xgold_sdhci_is_rpm_enabled(struct device_node *np)
{
	u32 rpm_enabled;
	if (!of_property_read_u32(np, "intel,rpm_enabled", &rpm_enabled))
		return !!rpm_enabled;
	return 0;
}


static void xgold_sdhci_of_init(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct xgold_mmc_pdata *mmc_pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	u32 timeout = 1000;

	mmc_of_parse(host->mmc);

	mmc_pdata->rpm_enabled = xgold_sdhci_is_rpm_enabled(np);
	pr_info("sdhci: %s, rpm = %d\n", dev_name(&pdev->dev),
						mmc_pdata->rpm_enabled);
	if (mmc_pdata->cd_gpio > 0)
		mmc_gpio_request_cd(host->mmc, mmc_pdata->cd_gpio, 0);

	/*
	 * Timeout after 250 ms to 500 ms if SDHCI_CARD_DET_STABLE
	 * doesn't happen
	 */
	while (!(sdhci_readl(host, SDHCI_PRESENT_STATE) &
			      SDHCI_CARD_DET_STABLE) && --timeout)
		usleep_range(250, 500);

	WARN(!timeout, "Timeout waiting for SDHCI_CARD_DET_STABLE: %s\n",
			mmc_hostname(host->mmc));

	if (mmc_pdata->fixup & XGOLD_DEFAULT_REGS_FIXUP)
		xgold_default_regs_fixup(host);

	host->mmc->caps2 |= MMC_CAP2_CACHE_CTRL | MMC_CAP2_POLL_R1B_BUSY;
}

static bool xgold_sdhci_of_paranoia(struct sdhci_host *host)
{
	if ((host->recovery = (pm_runtime_enabled(host->mmc->parent) &&
			       pm_runtime_suspended(host->mmc->parent))))
		pr_err("%s: trying to write while runtime suspended\n",
			 mmc_hostname(host->mmc));
	return host->recovery;
}

static void xgold_sdhci_of_writel(struct sdhci_host *host, u32 val, int reg)
{
	xgold_sdhci_of_paranoia(host);
	writel(val, host->ioaddr + reg);
}

static void xgold_sdhci_of_writew(struct sdhci_host *host, u16 val, int reg)
{
	xgold_sdhci_of_paranoia(host);
	writew(val, host->ioaddr + reg);
}

static void xgold_sdhci_of_writeb(struct sdhci_host *host, u8 val, int reg)
{
	xgold_sdhci_of_paranoia(host);
	writeb(val, host->ioaddr + reg);
}

static struct sdhci_ops xgold_sdhci_ops = {
	.write_l = xgold_sdhci_of_writel,
	.write_w = xgold_sdhci_of_writew,
	.write_b = xgold_sdhci_of_writeb,
	.set_clock = xgold_sdhci_of_set_clock,
	.get_max_clock = xgold_sdhci_of_get_max_clock,
	.get_min_clock = xgold_sdhci_of_get_min_clock,
	.card_status = xgold_sdhci_of_card_status,
	.set_uhs_signaling = xgold_sdhci_of_set_timing,
	.select_card_drive_strength = xgold_sdhci_of_select_card_ds,
#ifdef CONFIG_PM
	.platform_suspend = xgold_sdhci_of_suspend,
	.platform_resume = xgold_sdhci_of_resume,
#ifdef CONFIG_PM_RUNTIME
	.platform_runtime_suspend = xgold_sdhci_of_runtime_suspend,
	.platform_runtime_resume = xgold_sdhci_of_runtime_resume,
#endif
#endif
	.platform_init = xgold_sdhci_of_init,
};

static struct sdhci_pltfm_data sdhci_xgold_pdata_default = {
	.quirks = XGOLD_DEFAULT_QUIRKS,
	.quirks2 = XGOLD_DEFAULT_QUIRKS2,
	.ops = &xgold_sdhci_ops,
};

static irqreturn_t xgold_detect(int irq, void *dev_id)
{
	pr_info("%s: SD card inserted or removed\n", __func__);
	return IRQ_HANDLED;
}
static irqreturn_t xgold_eint_detect(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct xgold_mmc_pdata *mmc_pdata =
		(struct xgold_mmc_pdata *)pdev->dev.platform_data;
	struct sdhci_host *host = platform_get_drvdata(pdev);
	pr_info("%s: SD card removed/inserted during runtime suspend\n",
								__func__);
	if (mmc_pdata->irq_eint)
		xgold_cd_irq_disable(mmc_pdata);

	tasklet_schedule(&host->card_tasklet);
	return IRQ_HANDLED;
}


/*
 * Get intel,io-access property if any from dts
 */
bool xgold_sdhci_get_io_master(struct device_node *np)
{
	if (of_find_property(np, "intel,vmm-secured-access", NULL))
		return SCU_IO_ACCESS_BY_VMM;
	return SCU_IO_ACCESS_BY_LNX;
}

static int xgold_sdhci_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct xgold_mmc_pdata *mmc_pdata;
	struct resource res;
	struct sdhci_pltfm_data sdhci_xgold_pdata;
	int it_wk, it_eint;
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

	if (mmc_pdata->io_master == SCU_IO_ACCESS_BY_LNX) {
		mmc_pdata->scu_base = devm_ioremap(&pdev->dev, res.start,
				resource_size(&res));
		dev_info(&pdev->dev, "sdhci: io: linux-@:%p\n",
				mmc_pdata->scu_base);
	} else {
		mmc_pdata->scu_base_phys = res.start;
		dev_info(&pdev->dev, "sdhci: io: vmm-@:%pa\n",
				&mmc_pdata->scu_base_phys);
	}

	if (of_property_read_u32(np, "intel,tap_reg",
				&mmc_pdata->tap_reg_offset))
		mmc_pdata->tap_reg_offset = -1;
	if (of_property_read_u32(np, "intel,tap_reg2",
				&mmc_pdata->tap_reg2_offset))
		mmc_pdata->tap_reg2_offset = -1;

	ret |= of_property_read_u32_array(np, "intel,tap_values",
					&mmc_pdata->tap_values[0], 7);
	if (ret) {
		dev_dbg(&pdev->dev, "no tap values\n");
		mmc_pdata->tap_reg_offset = -1;
		mmc_pdata->tap_reg2_offset = -1;
	} else
		of_property_read_u32_array(np, "intel,tap_values2",
						&mmc_pdata->tap_values2[0], 7);

	of_property_read_u32_array(np, "intel,quirks", &quirktab[0], 2);
	of_property_read_u32(np, "intel,fixup", &mmc_pdata->fixup);
	of_property_read_u32_array(np, "intel,card_drive_strength",
					&mmc_pdata->card_drive_strength[0], 4);

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
	mmc_pdata->pins_restore_default = 1;

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
		disable_irq_nosync(mmc_pdata->irq_wk);
	}

	spin_lock_init(&mmc_pdata->irq_lock);
	it_eint = of_property_match_string(np, "interrupt-names",
					 "eint");
	if (it_eint > 0) {
		mmc_pdata->irq_eint = irq_of_parse_and_map(np, it_eint);
		ret = devm_request_irq(&pdev->dev, mmc_pdata->irq_eint,
			xgold_eint_detect,
			IRQF_SHARED | IRQF_NO_SUSPEND, "eint_int", pdev);
		if (ret != 0) {
			dev_err(&pdev->dev,
				"setup irq%d failed with ret = %d\n",
				mmc_pdata->irq_eint, ret);
			return -1;
		}
		xgold_cd_irq_disable(mmc_pdata);
	} else
		mmc_pdata->irq_eint = 0;


	mmc_pdata->cd_gpio = of_get_named_gpio(np, "cd-gpios", 0);

	/* quirks */
	sdhci_xgold_pdata.quirks = sdhci_xgold_pdata_default.quirks
							| quirktab[0];
	sdhci_xgold_pdata.quirks2 = sdhci_xgold_pdata_default.quirks2
							| quirktab[1];
	sdhci_xgold_pdata.ops = sdhci_xgold_pdata_default.ops;
	ret = sdhci_pltfm_register(pdev, &sdhci_xgold_pdata, 0);

	/* If runtime pm can be enabled, set it here, right after sdhci_add_host(
	 * sdhci_pltfm_register). That's because func sdhci_add_host accesses host
	 * registers without getting the controller dev first, that's not safe if
	 * rpm is enabled. */
	/* enable runtime pm support per slot */
	if (mmc_pdata->rpm_enabled) {
		pm_runtime_set_active(&pdev->dev);
		pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
		pm_runtime_use_autosuspend(&pdev->dev);
		pm_suspend_ignore_children(&pdev->dev, 1);
		pm_runtime_enable(&pdev->dev);
	}

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
	struct xgold_mmc_pdata *mmc_pdata =
		(struct xgold_mmc_pdata *)pdev->dev.platform_data;
	if (mmc_pdata && mmc_pdata->rpm_enabled) {
		pm_runtime_forbid(&pdev->dev);
		pm_runtime_get_noresume(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
	} else {
		pr_err("platform data not found.\n");
		return -EINVAL;
	}
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
