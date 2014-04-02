/*
 * Component: XGOLD BT SCO driver
 *
 * Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Contributor(s):
 */

#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/slab.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/clk-provider.h>
#include <linux/module.h>

#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_of.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/imc_idi_btauif_driverif.h>

#define TX_FROM_ABB		0
#define TX_FROM_DBB		1

#define BT_AUD_BRIDGE_OFF       0
#define BT_AUD_BRIDGE_ON_8K     1
#define BT_AUD_BRIDGE_ON_16K    2

#define BTAUIF_REG_RES_NAME	"register"
#define BTAUIF_CLK_48K_NAME	"btauif_48K"

#define bt_aud_bridge_debug	pr_debug
#define bt_aud_bridge_err	pr_err

struct bt_streaming_priv {
	/* State variable to store current status of bridge */
	int bridge_control;
	struct idi_peripheral_device *agold_bt_sco_device;
	struct idi_channel_config rx_config;
	struct idi_channel_config tx_config;
	void __iomem *ctrl_io;
	struct device_state_pm_state *current_pm_state;
	struct imc_idi_btauif_ops *ops;
};

struct bt_streaming_priv bt_sco_data;

struct imc_idi_btauif_pm_state {
	struct device_state_pm_state *d0_handler;
	struct device_state_pm_state *d3_handler;
};

static struct imc_idi_btauif_pm_state btauif_states;

#ifdef CONFIG_OF
struct imc_idi_btauif_platdata {
	struct clk *bt_48K;
	/* Reset controller */
	struct reset_control *btauif_rst;
};
#endif

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
void imc_idi_set_btauif_clk(struct imc_idi_btauif_platdata *platdata,
				bool enable)
{
	if (enable) {
		if (platdata->bt_48K)
			clk_prepare_enable(platdata->bt_48K);
	} else {
		if ((platdata->bt_48K)
				&& (__clk_is_enabled(platdata->bt_48K)))
			clk_disable_unprepare(platdata->bt_48K);
	}
}

#define BTAUIF_PM_STATE_D0 "enable"
#define BTAUIF_PM_STATE_D3 "disable"
static int btauif_set_pm_state(struct device *_dev,
		struct device_state_pm_state *state)
{
	struct imc_idi_btauif_platdata *platdata =
		(struct imc_idi_btauif_platdata *) dev_get_platdata(_dev);

	if (bt_sco_data.current_pm_state == state)
		return 0;

	if (!strcmp(state->name, BTAUIF_PM_STATE_D0)) {
		imc_idi_set_btauif_clk(platdata, true);
		if (platdata->btauif_rst)
			reset_control_reset(platdata->btauif_rst);
	} else if (!strcmp(state->name, BTAUIF_PM_STATE_D3)) {
		imc_idi_set_btauif_clk(platdata, false);
		if (platdata->btauif_rst)
			reset_control_assert(platdata->btauif_rst);
	} else
		return -EINVAL;

	bt_sco_data.current_pm_state = state;
	return 0;
}

static struct device_state_pm_state *btauif_get_initial_pm_state(
		struct device *_dev)
{

	return device_state_pm_get_state_handler(_dev, BTAUIF_PM_STATE_D3);
}

static struct device_state_pm_ops btauif_pm_ops = {
	.set_state = btauif_set_pm_state,
	.get_initial_state = btauif_get_initial_pm_state,
};

/* IDI PM states & class */

static struct device_state_pm_state btauif_pm_states[] = {
	{ .name = BTAUIF_PM_STATE_D0 },
	{ .name = BTAUIF_PM_STATE_D3 },
};


DECLARE_DEVICE_STATE_PM_CLASS(btauif);
#endif

/* Function to return the information regarding the bt bridge alsa control */
int bt_sco_control_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 3;

	return 0;
}

/* Function to set the bt bridge alsa control */
int bt_sco_control_set(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	int result = 0;
	int control_val;
	struct imc_idi_btauif_platdata *platdata;
	int btauif_rate = imc_idi_btauif_clk_8k;

	if (ucontrol->value.enumerated.item[0] > 2)
		return -EINVAL;

	if (!bt_sco_data.agold_bt_sco_device)
		return -ENODEV;

	platdata = bt_sco_data.agold_bt_sco_device->device.platform_data;
	control_val = ucontrol->value.enumerated.item[0];

	switch (control_val) {
	case BT_AUD_BRIDGE_OFF:
		bt_aud_bridge_debug("%s: Disabling BT SCO bridge\n",
				__func__);

		if (bt_sco_data.ops && bt_sco_data.ops->disable)
			result = bt_sco_data.ops->disable(bt_sco_data.ctrl_io);
		else
			result = -ENODEV;

		if (0 == result) {
			if (platdata->btauif_rst)
				reset_control_assert(platdata->btauif_rst);
			result = idi_set_power_state(
				bt_sco_data.agold_bt_sco_device,
				(void *) btauif_states.d3_handler, false);
		}
		break;

	case BT_AUD_BRIDGE_ON_8K:
	case BT_AUD_BRIDGE_ON_16K:
		bt_aud_bridge_debug("%s: Enabling BT SCO bridge for rate %d\n",
				__func__, control_val);

		result = idi_set_channel_config(
			bt_sco_data.agold_bt_sco_device,
			&(bt_sco_data.rx_config));
		if (0 == result)
			result = idi_set_channel_config(
				bt_sco_data.agold_bt_sco_device,
				&(bt_sco_data.tx_config));
		if (0 == result) {
			result = idi_set_power_state(
				bt_sco_data.agold_bt_sco_device,
				(void *) btauif_states.d0_handler, true);
			if (platdata->btauif_rst)
				reset_control_reset(platdata->btauif_rst);
		}

		if (0 == result) {
			if (BT_AUD_BRIDGE_ON_16K == control_val)
				btauif_rate = imc_idi_btauif_clk_16k;

			if (bt_sco_data.ops && bt_sco_data.ops->enable)
				result = bt_sco_data.ops->enable(btauif_rate,
							bt_sco_data.ctrl_io);
			else
				result = -ENODEV;
		}
		break;

	default:
		bt_aud_bridge_debug("%s: Invalid parameter\n", __func__);
		result = -EINVAL;
		break;
	}

	if (0 == result) {
		/* Update the bridge state to be returned as control */
		bt_sco_data.bridge_control = control_val;
	}
	bt_aud_bridge_debug("%s: Returns %d\n", __func__, result);

	return result;
}

/* Function to get the value of bt bridge alsa control */
int bt_sco_control_get(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = bt_sco_data.bridge_control;
	return 0;
}

void imc_idi_bt_sco_register(struct imc_idi_btauif_ops *ops)
{
	bt_sco_data.ops = ops;
}
EXPORT_SYMBOL_GPL(imc_idi_bt_sco_register);

void imc_idi_bt_sco_unregister(struct imc_idi_btauif_ops *ops)
{
	if (bt_sco_data.ops == ops)
		bt_sco_data.ops = NULL;
}
EXPORT_SYMBOL_GPL(imc_idi_bt_sco_unregister);

/* ASOC platform controls to control BT Audio bridge */
static const struct snd_kcontrol_new bt_sco_controls[] = {
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = "AGOLD BT Audio Bridge Control",
	 .info = bt_sco_control_info,
	 .get = bt_sco_control_get,
	 .put = bt_sco_control_set,
	 },
};

/* Initialization function to be called from ASOC platform driver */
int xgold_bt_sco_soc_init(struct snd_soc_platform *platform)
{
	int ret;
	ret = snd_soc_add_platform_controls(platform, bt_sco_controls,
					    ARRAY_SIZE(bt_sco_controls));
	if (ret != 0) {
		bt_aud_bridge_err("%s: dsp audio controls reg failed %d\n",
				  __func__, ret);
	}
	return ret;
}

#ifdef CONFIG_OF
static struct imc_idi_btauif_platdata *imc_idi_btauif_of_get_platdata(
				struct idi_peripheral_device *p_device)
{
	struct imc_idi_btauif_platdata *platdata;

	struct device_node *np = p_device->device.of_node;
	struct device *dev = &p_device->device;

	platdata = kzalloc(sizeof(*platdata), GFP_KERNEL);
	if (!platdata)
		return NULL;

	platdata->bt_48K = of_clk_get_by_name(np, BTAUIF_CLK_48K_NAME);
	if (IS_ERR(platdata->bt_48K)) {
		dev_warn(dev, "No 48k clock available\n");
		platdata->bt_48K = NULL;
	}
	platdata->btauif_rst = reset_control_get(dev, "btauif");
	if (IS_ERR(platdata->btauif_rst)) {
		dev_warn(dev, "No BTAUIF Reset controller found\n");
		platdata->btauif_rst = NULL;
	}

	return platdata;

}
#endif /* CONFIG_OF */

static int xgold_bt_sco_streaming_probe(struct idi_peripheral_device *pdev,
						const struct idi_device_id *id)
{
#ifdef CONFIG_OF
	struct imc_idi_btauif_platdata *platdata;
#endif
	struct resource *btauif_res, *dsp_res, *regres;
	struct idi_resource *idi_res = &pdev->resources;
	int ret = 0;

	bt_aud_bridge_debug("%s\n", __func__);

#ifdef CONFIG_OF
	platdata = imc_idi_btauif_of_get_platdata(pdev);
	if (!platdata) {
		ret = -EINVAL;
		goto fail_request;
	}
	pdev->device.platform_data = platdata;
#endif

#ifdef CONFIG_PLATFORM_DEVICE_PM
	ret = device_state_pm_set_class(&pdev->device,
		pdev->pm_platdata->pm_user_name);
	if (ret) {
		pr_err("BTAUIF device PM registration failed\n");
		goto fail_request;
	} else
		pr_err("BTAUIF device PM registration success\n");
#endif

	memset(&bt_sco_data, 0, sizeof(bt_sco_data));
	bt_sco_data.agold_bt_sco_device = pdev;
	bt_sco_data.bridge_control = 0;

	regres = idi_get_resource_byname(&pdev->resources, IORESOURCE_MEM,
			BTAUIF_REG_RES_NAME);

	if (!regres) {
		bt_aud_bridge_err("Register buffer info missing\n");
		return -EINVAL;
	}

	if (!request_mem_region(regres->start, resource_size(regres),
					dev_name(&pdev->device))) {
		return -EBUSY;
	}
	bt_sco_data.ctrl_io = devm_ioremap(&pdev->device, regres->start,
			resource_size(regres));

	if (!bt_sco_data.ctrl_io) {
		ret = -EPERM;
		dev_err(&pdev->device,
			"Failed to request resource\n");

		goto fail_request;
	}

	bt_sco_data.agold_bt_sco_device = pdev;
	bt_sco_data.bridge_control = 0;

	/*
	 * Prepare IDI RX channel : ABB->DBB
	 */
	btauif_res = idi_get_resource_byname(idi_res,
			IORESOURCE_MEM, "btauif_out");
	if (!btauif_res) {
		bt_aud_bridge_err("BTAUIF-OUT buffer info missing\n");
		return -EINVAL;
	}

	dsp_res = idi_get_resource_byname(idi_res,
			IORESOURCE_MEM, "dsp_bt_in");
	if (!dsp_res) {
		bt_aud_bridge_err("DSP-BT-IN buffer info missing\n");
		return -EINVAL;
	}

	/* Setup RX config for IDI */
	bt_sco_data.rx_config.dst_addr = btauif_res->start; /* ABB_TX */
	bt_sco_data.rx_config.channel_opts = IDI_PRIMARY_CHANNEL;
	bt_sco_data.rx_config.tx_or_rx = TX_FROM_ABB;
	bt_sco_data.rx_config.base = dsp_res->start; /* DBB_RX */
	bt_sco_data.rx_config.cpu_base = NULL;
	bt_sco_data.rx_config.size = resource_size(dsp_res);
	bt_sco_data.rx_config.hw_fifo_size = resource_size(btauif_res);
	bt_sco_data.rx_config.priority = IDI_HIGH_PRIORITY;

	/*
	 * Prepare IDI TX channel : DBB->ABB
	 */
	btauif_res = idi_get_resource_byname(idi_res,
			IORESOURCE_MEM, "btauif_in");
	if (!btauif_res) {
		bt_aud_bridge_err("BTAUIF-IN buffer info missing\n");
		return -EINVAL;
	}

	dsp_res = idi_get_resource_byname(idi_res,
			IORESOURCE_MEM, "dsp_bt_out");
	if (!dsp_res) {
		bt_aud_bridge_err("DSP-BT-OUT buffer info missing\n");
		return -EINVAL;
	}

	/* Setup TX config for IDI */
	bt_sco_data.tx_config.dst_addr = btauif_res->start; /* ABB_RX */
	bt_sco_data.tx_config.channel_opts = IDI_PRIMARY_CHANNEL;
	bt_sco_data.tx_config.tx_or_rx = TX_FROM_DBB;
	bt_sco_data.tx_config.base = dsp_res->start;	/* DBB_TX */
	bt_sco_data.tx_config.cpu_base = NULL;
	bt_sco_data.tx_config.size = resource_size(dsp_res);
	bt_sco_data.tx_config.hw_fifo_size = resource_size(btauif_res);
	bt_sco_data.tx_config.priority = IDI_HIGH_PRIORITY;

	btauif_states.d0_handler = idi_peripheral_device_pm_get_state_handler(
			bt_sco_data.agold_bt_sco_device, "enable");
	btauif_states.d3_handler = idi_peripheral_device_pm_get_state_handler(
			bt_sco_data.agold_bt_sco_device, "disable");

	idi_set_channel_config(pdev, &bt_sco_data.rx_config);
	idi_set_channel_config(pdev, &bt_sco_data.tx_config);

	bt_sco_data.ops = NULL;

	return 0;

fail_request:

#ifdef CONFIG_OF
	kfree(platdata);
#endif
	return ret;
}

static int xgold_bt_sco_streaming_remove(struct idi_peripheral_device *p_device)
{
#ifdef CONFIG_SND_SOC_NATIVE_AUDIO_MODE
	iounmap(bt_sco_data.ctrl_io);
#endif

#ifdef CONFIG_OF
	kfree(p_device->device.platform_data);
#endif

	bt_sco_data.agold_bt_sco_device = NULL;
	bt_sco_data.ops = NULL;
	return 0;
}

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG610,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_BT_STREAM,
	},
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_BT_STREAM,
	},

	{ /* end: all zeroes */},
};

struct idi_peripheral_driver xgold_bt_sco_streaming_drv = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "xgold_bt_sco",
		   .pm = NULL,
		   },
	.p_type = IDI_BT_STREAM,
	.id_table = idi_ids,
	.probe = xgold_bt_sco_streaming_probe,
	.remove = xgold_bt_sco_streaming_remove,
};

MODULE_DEVICE_TABLE(idi, idi_ids);

static int __init xgold_bt_sco_streaming_init(void)
{
	int ret = 0;
	bt_aud_bridge_debug("%s:\n", __func__);

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	ret = device_state_pm_add_class(&btauif_pm_class);
	if (ret) {
		pr_err("Error while adding BTAUIF pm class\n");
		return ret;
	}
#endif

	ret = idi_register_peripheral_driver(&xgold_bt_sco_streaming_drv);
	if (ret < 0) {
		bt_aud_bridge_err
		("%s : unable to register BT SCO streaming IDI client driver\n",
		__func__);
		ret = -ENODEV;
	}
	return ret;
}

module_init(xgold_bt_sco_streaming_init);

static void __exit xgold_bt_sco_streaming_exit(void)
{

	bt_aud_bridge_debug("%s:\n", __func__);
	idi_unregister_peripheral_driver(&xgold_bt_sco_streaming_drv);
}

module_exit(xgold_bt_sco_streaming_exit);

MODULE_DESCRIPTION("XGOLD BT SCO streaming IDI client driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL v2");
