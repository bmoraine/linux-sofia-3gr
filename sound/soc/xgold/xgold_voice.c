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

#define	xgold_err(fmt, arg...) \
		pr_err("snd: voice: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: voice: "fmt, ##arg)

static int xgold_voice_trigger(struct snd_pcm_substream *stream,
		int cmd, struct snd_soc_dai *dai)
{
	xgold_debug("%s : cmd %d", __func__, cmd);
	/* FIXME */
	return 0;
}

static struct snd_soc_dai_ops voice_ops = {
	.trigger = xgold_voice_trigger,
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

	xgold_debug("%s :\n", __func__);

	ret = snd_soc_register_component(&pdev->dev, &xgold_voice_component,
			&xgold_dai_voice, 1);

	if (ret < 0)
		xgold_err("Failed to register XGOLD platform driver 1\n");

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
