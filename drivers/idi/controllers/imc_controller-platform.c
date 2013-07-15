/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/platform_device_pm.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_of.h>

#if defined(CONFIG_FM)
#define IDI_FM_SUPPORT
#include <linux/fm/iui_fm.h>
#endif

#if defined IDI_FM_SUPPORT
struct idi_controller_device *idi_controller;
#endif

#include <linux/idi/idi_debug.h>

#include "imc_controller.h"

#define IDI_IMC_ENTER pr_debug("--> %s\n", __func__)
#define IDI_IMC_EXIT pr_debug("<-- %s\n", __func__)

#define IDI_CLASS_NAME "idi"
const char *idi_power_states[] = {
"enable_no_wlan", "enable_no_wlan_alt1", "enable_wlan",
"enable_wlan_alt1", "enable_audio_only", "enable_audio_only_alt1", "disable"};

struct imc_controller_platform_data {
	struct device_pm_platdata *dev_pm;
	enum idi_channel_type *channels;
	/* channels, ... */
};

struct imc_idi_pm_data {
	struct clk *clk_kernel;
	struct clk *clk_bus;
};

static struct device_pm_platdata *imc_idi_pm_get_platdata(
					struct device *_dev)
{
	struct device_node *np = _dev->of_node;
	struct device_pm_platdata *dev_pm_data;
	struct imc_idi_pm_data *pm_data;
	struct clk *clk_kernel;
	struct clk *clk_bus;


	dev_pm_data = of_device_state_pm_setup(np);

	pm_data = kzalloc(sizeof(struct imc_idi_pm_data), GFP_KERNEL);
	if (pm_data == NULL)
		return ERR_PTR(-ENOMEM);

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	clk_kernel = of_clk_get_by_name(np, "kernel");
	if (unlikely(IS_ERR(clk_kernel))) {
		dev_err(_dev, "Unable to get kernel clock !");
		return ERR_PTR(-EINVAL);
	}

	clk_bus = of_clk_get_by_name(np, "bus");
	if (unlikely(IS_ERR(clk_bus))) {
		dev_err(_dev, "Unable to get bus clock !");
		return ERR_PTR(-EINVAL);
	}
#else
	clk_kernel = NULL;
	clk_bus = NULL;
#endif
	pm_data->clk_kernel = clk_kernel;
	pm_data->clk_bus = clk_bus;
	dev_pm_data->priv = pm_data;

	return dev_pm_data;
}


static struct imc_controller_platform_data *imc_idi_get_platdata(
							struct device *_dev)
{
#ifdef CONFIG_OF
	struct imc_controller_platform_data *platdata;

	platdata = kzalloc(
			sizeof(struct imc_controller_platform_data),
			GFP_KERNEL);
	if (platdata == NULL)
		return ERR_PTR(-ENODEV);

	platdata->dev_pm = imc_idi_pm_get_platdata(_dev);

	_dev->platform_data = platdata;
	return platdata;

#else
	return &_dev->platform_data;
#endif
}

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
/* IDI PM states & class */
static struct device_state_pm_state idi_pm_states[] = {
	{ .name = "enable_no_wlan", }, /* DO */
	{ .name = "enable_no_wlan_alt1", },
	{ .name = "enable_wlan", },
	{ .name = "enable_wlan_alt1", },
	{ .name = "enable_audio_only", }, /* D0I2 ?? */
	{ .name = "enable_audio_only_alt1", },
	{ .name = "disable" }, /* D3 */
};

#define IDI_PM_STATE_D0		0
#define IDI_PM_STATE_D0I2	4
#define IDI_PM_STATE_D3		6

static int imc_idi_set_pm_state(struct device *_dev,
		struct device_state_pm_state *state)
{
	struct imc_controller_platform_data *platdata = dev_get_platdata(_dev);
	struct device_pm_platdata *dev_pm;
	struct imc_idi_pm_data *pm_data;
	int id = device_state_pm_get_state_id(_dev, state->name);
	struct clk *clk_kernel;
	struct clk *clk_bus;

	if (platdata == NULL)
		return -ENODEV;

	dev_pm = platdata->dev_pm;
	if (dev_pm == NULL)
		return -ENODEV;

	pm_data = dev_pm->priv;
	if (pm_data == NULL)
		return -ENODEV;

	clk_kernel = pm_data->clk_kernel;
	clk_bus = pm_data->clk_bus;

	dev_dbg(_dev, "%s state id %d\n", __func__, id);

	switch (id) {
	case IDI_PM_STATE_D0:
		clk_prepare_enable(clk_kernel);
		clk_prepare_enable(clk_bus);
		break;

	case IDI_PM_STATE_D0I2:
		/* TODO */
		break;

	case IDI_PM_STATE_D3:
		/* FIXME : crash when accessing IDI controller registers */
		/* clk_disable_unprepare(clk_kernel);
		clk_disable_unprepare(clk_bus); */
		break;

	default:
		dev_err(_dev, "Unsuported state id %d\n", id);
		return -EINVAL;
	}

	dev_info(_dev, "kernel clk: %lu Hz, bus clk: %lu Hz\n",
		 clk_get_rate(clk_kernel), clk_get_rate(clk_bus));

	return 0;
}

static struct device_state_pm_state *imc_idi_get_initial_state(
		struct device *_dev)
{
	return device_state_pm_get_state_handler(_dev,
			idi_pm_states[IDI_PM_STATE_D3].name);
}

static struct device_state_pm_ops idi_pm_ops = {
	.set_state = imc_idi_set_pm_state,
	.get_initial_state = imc_idi_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(idi);
#endif

/**
 * pltfm_imc_add_controller - make and init imc_idi controller
 * @pdev: PCI device reference
 *
 * Allocate imc_idi controller, attach to idi_controller, activate
 * PCI device and map memory for IDI and master DMA, init ports, and
 * register controller with IDI (perform board info scan there).
 *
 * Returns success or an error code if any initialization is failing.
 */

static int pltfm_imc_add_controller(struct platform_device *pdev)
{
	struct imc_controller *imc_idi;
	int err = 0, i;
	struct resource *dbb_res;
	struct device_node *np = pdev->dev.of_node;
	struct imc_controller_platform_data *platdata;
	struct device_pm_platdata *dev_pm;
#if defined IDI_FM_SUPPORT && defined CONFIG_PLATFORM_DEVICE_PM
	struct iui_fm_freq_notification fm_info;
#endif

#ifdef CONFIG_PLATFORM_DEVICE_PM
	int index;
#endif

	IDI_IMC_ENTER;

	imc_idi = imc_alloc_controller(&pdev->dev);
	if (IS_ERR(imc_idi))
		return PTR_ERR(imc_idi);

	platdata = imc_idi_get_platdata(&pdev->dev);
	if (IS_ERR(platdata)) {
		err = PTR_ERR(platdata);
		goto fail_imc_idi_enable_device;
	}

#ifdef CONFIG_PLATFORM_DEVICE_PM
	dev_pm = platdata->dev_pm;
	imc_idi->pm_platdata = dev_pm;
#endif

	err = of_idi_populate_channels_map(np, imc_idi->idi->channels);
	if (err)
		goto fail_imc_idi_enable_device;

	for (i = 0; i < of_irq_count(np); i++)
		imc_idi->irqs[i] = irq_of_parse_and_map(np, i);

	dbb_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ididbb");

	if (!dbb_res) {
		err = -EINVAL;
		dev_err(&pdev->dev, "ididbb get resource fails %d\n", err);
		goto no_dbb_res;
	}

	if (!request_mem_region(dbb_res->start, resource_size(dbb_res),
					dev_name(&pdev->dev))) {
		err = -EPERM;
		goto fail_request_dbb_region;
	}

	imc_idi->ctrl_io = ioremap(dbb_res->start, resource_size(dbb_res));
	if (!imc_idi->ctrl_io) {
		err = -EPERM;
		goto no_dbb_remap;
	}

#ifdef CONFIG_PLATFORM_DEVICE_PM
	/*Register IDI with PM framework */
	err = platform_device_pm_set_class(pdev, dev_pm->pm_user_name);
	if (err)
		pr_err("Register %s to platform PM class failed\n",
				dev_pm->pm_user_name);
	/* Get the PM state handle for IDI states*/
	for (index = 0; index < IDI_MAX_POWER_STATE; index++)
		imc_idi->idi_pm_state[index] =
				platform_device_pm_get_state_handler(pdev,
						idi_power_states[index]);

   /* Register the callback function*/
	#if defined IDI_FM_SUPPORT
	err = iui_fm_register_mitigation_callback(
					IUI_FM_MACRO_ID_IDI,
					idi_fm_cb);
	#endif

	/* Set the IDI device state to default configuration*/
	err = platform_device_pm_set_state(pdev,
			imc_idi->idi_pm_state[ENABLE_NO_WLAN]);
	if (!err) {
		imc_idi->idi_state = ENABLE_NO_WLAN;
	/* Notify the Frequency manager*/
	#if defined IDI_FM_SUPPORT
		fm_info.type = IUI_FM_FREQ_NOTIFICATION_TYPE_KHZ;
		fm_info.info.freq_khz = IDI_FREQUENCY_ENABLE_NO_WLAN;
		err = iui_fm_notify_frequency(IUI_FM_MACRO_ID_IDI, &fm_info);
		if (err)
			pr_err("Failed to notify frequency manager\n");
	#endif
	} else
		pr_err("Failed to set IDI to default state");
#endif
	err = imc_add_controller(imc_idi);
	if (err)
		goto fail_add_controller;

	platform_set_drvdata(pdev, (void *)imc_idi->idi);
#if defined IDI_FM_SUPPORT
	idi_controller = imc_idi->idi;
#endif
	IDI_IMC_EXIT;
	return 0;

fail_add_controller:
	iounmap(imc_idi->ctrl_io);
no_dbb_remap:
	release_mem_region(dbb_res->start, resource_size(dbb_res));
fail_request_dbb_region:
no_dbb_res:
fail_imc_idi_enable_device:
	imc_free_controller(imc_idi);
	IDI_IMC_EXIT;
	return err;

}				/* pltfm_imc_add_controller() */

/**
 * pltfm_imc_remove_controller - stop controller and unregister with IDI
 * @pdev: Platform device reference
 *
 * Stop controller and unregister with IDI
 */
static void pltfm_imc_remove_controller(struct platform_device *pdev)
{
	struct idi_controller_device *idi =
	    (struct idi_controller_device *)platform_get_drvdata(pdev);

	struct imc_controller *imc_idi =
	    (struct imc_controller *)idi_controller_get_drvdata(idi);

#if defined IDI_FM_SUPPORT
	int err = 0;
#endif

#ifdef CONFIG_OF
	struct device_node *np = pdev->dev.of_node;
#endif
	struct resource res;
	IDI_IMC_ENTER;

	imc_remove_controller(imc_idi);
	platform_set_drvdata(pdev, NULL);

#ifdef CONFIG_OF
	of_address_to_resource(np, 0, &res);
#endif

	iounmap(imc_idi->ctrl_io);
	release_mem_region(res.start, resource_size(&res));
	imc_free_controller(imc_idi);

 /* Deregister the callback function*/
#if defined IDI_FM_SUPPORT
	err = iui_fm_register_mitigation_callback(
					IUI_FM_MACRO_ID_IDI,
					NULL);
	if (err)
		pr_err("Failed to deregister the callback function");
#endif

	IDI_IMC_EXIT;
}				/* imc_remove_controller() */

/**
 * imc_idi_probe - device PCI probe
 * @pdev: Plaform device reference
 *
 * Allocate, add controller to the IDI framework and initialise its hardware.
 *
 * Returns success or an error code if any initialisation is failing.
 */
static int imc_idi_probe(struct platform_device *pdev)
{
	int err = 0;
	IDI_IMC_ENTER;

	err = pltfm_imc_add_controller(pdev);

	if (err < 0) {
		dev_err(&pdev->dev, "imc controller probe error exit");
		return err;
	}

	IDI_IMC_EXIT;
	return 0;
}				/* imc_idi_probe() */

/**
 * imc_idi_remove - called during PCI device exit
 * @pdev: platform device reference
 *
 * Remove the IDI controller from the IDI framework and free its memory.
 */

static int imc_idi_remove(struct platform_device *pdev)
{
	IDI_IMC_ENTER;

	pltfm_imc_remove_controller(pdev);

	IDI_IMC_EXIT;
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id imc_idi_controller_of_match[] = {
	{.compatible = "intel,idiper",},
	{},
};
#endif

struct platform_driver imc_idi_driver = {
	.driver	= {
		.name = "pltfm_imc_idi",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = imc_idi_controller_of_match,
#endif
	/*	.pm = &imc_idi_rtpm, */
	},
	.probe	= imc_idi_probe,
	.remove = imc_idi_remove,
};

/**
 * imc_idi_init - IDI controller driver entry point and initialization
 *
 * Returns success or an error code if driver registration is failing.
 */
static int __init imc_idi_init(void)
{

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret;
	ret = device_state_pm_add_class(&idi_pm_class);
	if (ret) {
		pr_err("Error while adding imc idi pm class\n");
		return ret;
	}
#endif
	pr_err("Registering IDI driver\n");
	return platform_driver_register(&imc_idi_driver);
}				/* imc_idi_init() */

module_init(imc_idi_init);

/**
 * imc_idi_exit - frees resources taken by the IDI controller driver
 */
static void __exit imc_idi_exit(void)
{
	platform_driver_unregister(&imc_idi_driver);

}				/* imc_idi_exit() */

module_exit(imc_idi_exit);

MODULE_ALIAS("platform:imc_idi");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("Intel IDI Controller Platform Driver");
MODULE_LICENSE("GPL v2");
