/*
 * Component: XGOLD voice driver
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
 * Suryaprakash
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <linux/io.h>
#include <linux/delay.h>
#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/platform_device_pm.h>
#endif
#include "dsp_audio_platform.h"

#define	xgold_err(fmt, arg...) \
		pr_err("snd: voice: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: voice: "fmt, ##arg)

struct xgold_voice {
	struct device *dev;
	struct dsp_audio_device *dsp;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct device_pm_platdata *pm_platdata;
};

static inline int i2s1_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	int ret = 0;
	struct xgold_voice *voice_data = dev_get_drvdata(dev);

	if (!voice_data) {
		dev_err(dev, "Unable to retrieve voice data\n");
		return -EINVAL;
	}
	if (!IS_ERR_OR_NULL(voice_data->pinctrl)) {
		if (!IS_ERR_OR_NULL(state)) {
			ret = pinctrl_select_state(voice_data->pinctrl, state);
			if (ret)
				dev_err(dev, "%s %d:could not set pins\n",
					__func__, __LINE__);
		}
	}
	return ret;
}

/* Set I2S1 devcice details to DSP structure */
static void i2s1_set_device_data(struct device *dev,
	enum i2s_devices device)
{
	struct xgold_voice *voice = dev_get_drvdata(dev);
	struct dsp_i2s_device *i2s_dev =
		voice->dsp->p_dsp_common_data->p_i2s_dev[device];

	xgold_debug("%s: device %d\n", __func__, device);

	i2s_dev->dev = dev;
	i2s_dev->pinctrl = voice->pinctrl;
	i2s_dev->pins_default = voice->pins_default;
	i2s_dev->pins_inactive = voice->pins_inactive;
	i2s_dev->pins_sleep = voice->pins_sleep;
	i2s_dev->pm_platdata = voice->pm_platdata;
}

static void xgold_voice_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct xgold_voice *voice = dev_get_drvdata(dai->dev);
	if (!dai->active) {
		xgold_debug("%s: voice dai is active\n", __func__);
		dsp_cmd_afe_streaming_off(voice->dsp);
	}
}

static int xgold_voice_trigger(struct snd_pcm_substream *stream,
		int cmd, struct snd_soc_dai *dai)
{
	xgold_debug("%s : cmd %d", __func__, cmd);
	/* FIXME */
	return 0;
}

static struct snd_soc_dai_ops voice_ops = {
	.trigger = xgold_voice_trigger,
	.shutdown = xgold_voice_shutdown,
};

static struct snd_soc_dai_driver xgold_dai_voice = {
	.name = "Voice",
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &voice_ops,
};

static const struct snd_soc_component_driver xgold_voice_component = {
	.name = "xgold-voice",
};

static int xgold_voice_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct xgold_voice *voice_data_ptr = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *dsp_of_node;

	xgold_err("%s :\n", __func__);

	voice_data_ptr = kzalloc(sizeof(struct xgold_voice), GFP_KERNEL);
	if (voice_data_ptr == NULL)
		return -ENOMEM;

	voice_data_ptr->dev = &pdev->dev;

#ifdef CONFIG_OF
	dsp_of_node = of_parse_phandle(np, "intel,dsp", 0);
	if (!dsp_of_node) {
		xgold_err("Unable to get dsp node\n");
		kzfree(voice_data_ptr);
		return -EINVAL;
	}

	voice_data_ptr->dsp = of_dsp_register_client(&pdev->dev, dsp_of_node);
#endif
	if (!voice_data_ptr->dsp) {
		xgold_err("Cannot register as dsp client\n");
		kzfree(voice_data_ptr);
		return -EPROBE_DEFER;
	}

	ret = snd_soc_register_component(&pdev->dev, &xgold_voice_component,
			&xgold_dai_voice, 1);

	if (ret < 0)
		xgold_err("Failed to register XGOLD platform driver 1\n");

	/* pinctrl */
	voice_data_ptr->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(voice_data_ptr->pinctrl)) {
		voice_data_ptr->pinctrl = NULL;
		goto skip_pinctrl;
	}

	voice_data_ptr->pins_default =
		pinctrl_lookup_state(voice_data_ptr->pinctrl,
						 PINCTRL_STATE_DEFAULT);
	if (IS_ERR(voice_data_ptr->pins_default))
		xgold_debug("could not get default pinstate\n");

	voice_data_ptr->pins_sleep =
		pinctrl_lookup_state(voice_data_ptr->pinctrl,
					       PINCTRL_STATE_SLEEP);
	if (IS_ERR(voice_data_ptr->pins_sleep))
		xgold_debug("could not get sleep pinstate\n");

	voice_data_ptr->pins_inactive =
		pinctrl_lookup_state(voice_data_ptr->pinctrl,
					       "inactive");
	if (IS_ERR(voice_data_ptr->pins_inactive))
		xgold_debug("could not get inactive pinstate\n");

skip_pinctrl:
	voice_data_ptr->pm_platdata =
		of_device_state_pm_setup(pdev->dev.of_node);
	if (IS_ERR(voice_data_ptr->pm_platdata)) {
		xgold_debug("Missing pm platdata properties\n");
		voice_data_ptr->pm_platdata = NULL;
	}

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (voice_data_ptr->pm_platdata) {
		ret = platform_device_pm_set_class(pdev,
				voice_data_ptr->pm_platdata->pm_user_name);

		if (ret < 0) {
			xgold_err("%s: failed to set PM class error %d\n",
					__func__, ret);
			kzfree(voice_data_ptr);
			return ret;
		}

		/* Disable I2S1 Power and clock domains */
		ret = platform_device_pm_set_state_by_name(pdev,
				voice_data_ptr->pm_platdata->pm_state_D3_name);

		if (ret < 0)
			xgold_err("%s: failed to set PM state error %d\n",
					__func__, ret);
	}
#endif

	platform_set_drvdata(pdev, voice_data_ptr);

	/* Disable I2S1 pins at init */
	ret = i2s1_set_pinctrl_state(&pdev->dev,
			voice_data_ptr->pins_inactive);

	/* set I2s1 device details */
	i2s1_set_device_data(&pdev->dev, XGOLD_I2S1);

	return ret;
}

static int xgold_voice_remove(struct platform_device *pdev)
{
	xgold_debug("%s :\n", __func__);
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static struct of_device_id xgold_snd_voice_of_match[] = {
	{ .compatible = "intel,xgold-snd-voice", },
	{ },
};

static struct platform_driver xgold_snd_voice_drv = {
	.driver = {
		.name = "XGOLD_Voice",
		.owner = THIS_MODULE,
		.of_match_table = xgold_snd_voice_of_match,
	},
	.probe = xgold_voice_probe,
	.remove = xgold_voice_remove,
};

static int __init xgold_snd_voice_init(void)
{
	int ret = 0;

	xgold_debug("%s\n", __func__);

	ret = platform_driver_register(&xgold_snd_voice_drv);

	if (ret < 0)
		xgold_err("%s : Unable to register voice platform driver\n",
				__func__);

	return ret;
}
module_init(xgold_snd_voice_init);

static void __exit snd_xgold_voice_exit(void)
{
	xgold_debug("%s\n", __func__);
	platform_driver_unregister(&xgold_snd_voice_drv);
}
module_exit(snd_xgold_voice_exit);

MODULE_DESCRIPTION("XGOLD ASOC Voice Platform driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL V2");
MODULE_DEVICE_TABLE(of, xgold_snd_voice_of_match);
