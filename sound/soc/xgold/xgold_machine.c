/*
 * Component: XGOLD Machine driver
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/consumer.h>
#include <linux/iio/driver.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include "xgold_jack.h"

#define PCL_LOCK_EXCLUSIVE	1
#define PCL_OPER_ACTIVATE	0
#define PROP_CODEC_DAI_NAME	"intel,codec_dai_name"
#define PROP_SPK_AMP_NAME	"intel,spk-gpio-sd"

#define	xgold_err(fmt, arg...) \
		pr_err("snd: mac: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: mac: "fmt, ##arg)

int audio_native_mode;

struct xgold_mc_private {
	struct xgold_jack *jack;
	int spk_pin;
};

struct snd_soc_jack hs_jack;
struct xgold_mc_private *mc_dev_ctx;

/* Enable/Disable IHF ext amplifier */
int spk_amp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	if (mc_dev_ctx->spk_pin <= 0)
		return 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		xgold_debug("%s: SND_SOC_DAPM_PRE_PMU\n", __func__);
		if (mc_dev_ctx->spk_pin > 0)
			gpiod_set_value(gpio_to_desc(mc_dev_ctx->spk_pin), 1);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		xgold_debug("%s: SND_SOC_DAPM_PRE_PMD\n", __func__);
		if (mc_dev_ctx->spk_pin > 0)
			gpiod_set_value(gpio_to_desc(mc_dev_ctx->spk_pin), 0);
		break;

	default:
		break;
	}

	return 0;
}

/* XGOLD machine DAPM */
static const struct snd_soc_dapm_widget xgold_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", spk_amp_event),
	SND_SOC_DAPM_HP("Headset", NULL),
	SND_SOC_DAPM_SPK("Earphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Handset Mic", NULL)
};

/* XGOLD Machine driver audio map*/
static const struct snd_soc_dapm_route audio_map[] = {

	/* External Speakers: LOUDSpeaker */
	{"Speaker", NULL, "LOUDSPEAKER"},

	/* Headset Stereophone (Headphone): HSL, HSR */
	{"Headset", NULL, "HSL"},
	{"Headset", NULL, "HSR"},

	/* Earphone speaker */
	{"Earphone", NULL, "EARPIECE"},

	/* Microphone */
	{"Headset Mic", NULL, "AMIC2"},
	{"Handset Mic", NULL, "AMIC1"}
};

static int xgold_snd_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	int ret = 0;

	xgold_debug("%s\n", __func__);

	/* Add XGOLD specific widgets */
	ret = snd_soc_dapm_new_controls(&codec->dapm, xgold_dapm_widgets,
					ARRAY_SIZE(xgold_dapm_widgets));
	if (ret) {
		xgold_err("Failed to add DAPM controls\n");
		return ret;
	}

	/* Set up XGOLD specific audio path audio_map */
	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	/* XGOLD connected pins */
	snd_soc_dapm_enable_pin(&codec->dapm, "Speaker");
	snd_soc_dapm_enable_pin(&codec->dapm, "Earphone");
	snd_soc_dapm_enable_pin(&codec->dapm, "Headset");
	snd_soc_dapm_enable_pin(&codec->dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(&codec->dapm, "Handset Mic");

	ret = snd_soc_dapm_sync(&codec->dapm);
	if (ret) {
		xgold_err("Failed to enable dapm pins\n");
		return ret;
	}

	/* Set up jack detection */
	ret = xgold_jack_setup(codec, &hs_jack);

	return ret;
}

/* Digital audio interface glue - connects codec <--> CPU
 * xgold dai's are devided into Front End (FE) and Back End (BE) devices
 * FE devices are pure streaming devices, not in charge of enabling any codecs.
 * BE devices are used for enabling codecs alone.
 *
 * Note: Speech probe index define XGOLD_SPEECH_PROBE_DEVICE_OFSET must be
 * updated in case speech probes are relocated in this array.
 */
static struct snd_soc_dai_link xgold_dai[] = {
	/* PCM front end device */
	{
		.name = "XGOLD_PCM",
		.stream_name = "PCM Audio",
		.init = xgold_snd_init,
		.ignore_suspend = 1,
	},
	/* Codec back end device */
	{
		.name = "XGOLD_VOICE",
		.stream_name = "Voice",
		.ignore_suspend = 1,
	},
	{
		.name = "XGOLD_HW_PROBE_A",
		.stream_name = "XGOLD_HW_PROBE_A",
		.ignore_suspend = 1,
	},
	{
		.name = "XGOLD_HW_PROBE_B",
		.stream_name = "XGOLD_HW_PROBE_B",
		.ignore_suspend = 1,
	},

#ifdef CONFIG_SND_SOC_XGOLD_SPEECH_PROBE
	/* Speech probe front end devices */
	{
		.name = "XGOLD_SPEECH_PROBE_A",
		.stream_name = "XGOLD_SPEECH_PROBE_A",
		.ignore_suspend = 1,
	},
	{
		.name = "XGOLD_SPEECH_PROBE_B",
		.stream_name = "XGOLD_SPEECH_PROBE_B",
		.ignore_suspend = 1,
	},
	{
		.name = "XGOLD_SPEECH_PROBE_C",
		.stream_name = "XGOLD_SPEECH_PROBE_C",
		.ignore_suspend = 1,
	},
	/* ALSA allows only 8 pcm devices with
	static minor numbers check after LTE PO?*/
#ifdef CONFIG_INCREASE_PCM_DEVICE
	{
		.name = "XGOLD_SPEECH_PROBE_D",
		.stream_name = "XGOLD_SPEECH_PROBE_D",
		.ignore_suspend = 1,
	},
	{
		.name = "XGOLD_SPEECH_PROBE_E",
		.stream_name = "XGOLD_SPEECH_PROBE_E",
		.ignore_suspend = 1,
	},
	{
		.name = "XGOLD_SPEECH_PROBE_F",
		.stream_name = "XGOLD_SPEECH_PROBE_F",
		.ignore_suspend = 1,
	},
#endif
#endif
	/* PCM2 front end device */
	{
		.name = "XGOLD_PCM2",
		.stream_name = "PCM Audio 2",
		.ignore_suspend = 1,
	},
};

/* Audio machine driver */
static struct snd_soc_card xgold_snd_card = {
	.name = "xgold_afe-audio",
	.owner = THIS_MODULE,
	.dai_link = xgold_dai,
	.num_links = ARRAY_SIZE(xgold_dai),
};

static int xgold_mc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *codec_of_node = NULL;
	struct device_node *jack_of_node = NULL;
	struct snd_soc_dai_link *dai_link;
	struct xgold_mc_private *mc_drv_ctx;
	const char *codec_dai_name = NULL;
	int ret = 0;
	int i;

	xgold_debug("%s:\n", __func__);

	xgold_snd_card.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&xgold_snd_card, &audio_native_mode);

#ifdef CONFIG_OF
	codec_of_node = of_parse_phandle(np,
			"intel,audio-codec", 0);

	if (codec_of_node == NULL) {
		xgold_err("Unable to get codec node\n");
		return -ENODEV;
	}

	ret = of_property_read_string(codec_of_node,
				PROP_CODEC_DAI_NAME,
				&codec_dai_name);

	if (ret) {
		xgold_err("Cannot get codec dai name ret %d\n", ret);
		return ret;
	}

	if (!audio_native_mode) {
		for (i = 0; i < xgold_snd_card.num_links; i++) {
			dai_link = &xgold_snd_card.dai_link[i];

			if (!strncmp(dai_link->stream_name, "PCM Audio",
				strlen("PCM Audio"))) {
				dai_link->cpu_of_node =
					dai_link->platform_of_node =
					of_parse_phandle(np, "intel,pcm-audio",
							0);
				dai_link->codec_dai_name = "snd-soc-dummy-dai";
				dai_link->codec_name = "snd-soc-dummy";

			} else if (!strcmp(dai_link->stream_name, "Voice")) {
				/* FIXME: for Voice stream, platform is
				 * snd-soc-dummy */
				dai_link->cpu_of_node = of_parse_phandle(np,
						"intel,pcm-voice", 0);

				dai_link->codec_of_node = codec_of_node;
				dai_link->codec_dai_name = codec_dai_name;

			} else if (!strncmp(dai_link->stream_name,
					"XGOLD_SPEECH_PROBE_",
					strlen("XGOLD_SPEECH_PROBE_"))) {
				dai_link->cpu_of_node =
					dai_link->platform_of_node =
					of_parse_phandle(np, "intel,speech", 0);
				dai_link->codec_dai_name = "snd-soc-dummy-dai";
				dai_link->codec_name = "snd-soc-dummy";
			} else if (!strncmp(dai_link->stream_name,
						"XGOLD_HW_PROBE_A",
						strlen("XGOLD_HW_PROBE_A"))) {
				dai_link->cpu_of_node =
				dai_link->platform_of_node =
				of_parse_phandle(np, "intel,pcm-audio", 0);
				dai_link->codec_dai_name = "snd-soc-dummy-dai";
				dai_link->codec_name = "snd-soc-dummy";
			} else if (!strncmp(dai_link->stream_name,
						"XGOLD_HW_PROBE_B",
						strlen("XGOLD_HW_PROBE_B"))) {
				dai_link->cpu_of_node =
				dai_link->platform_of_node =
				of_parse_phandle(np, "intel,pcm-audio", 0);
				dai_link->codec_dai_name = "snd-soc-dummy-dai";
				dai_link->codec_name = "snd-soc-dummy";
			}

			if (!dai_link->cpu_of_node)
				pr_err("error for %s DAI binding\n",
						dai_link->name);
		}
	} else {
		/* setup devices for native kernel support */
		for (i = 0; i < xgold_snd_card.num_links; i++) {
			dai_link = &xgold_snd_card.dai_link[i];
			dai_link->codec_of_node = codec_of_node;
			dai_link->codec_dai_name = codec_dai_name;

			if (!strncmp(dai_link->stream_name, "PCM Audio",
				strlen("PCM Audio")))
				dai_link->cpu_of_node =
					dai_link->platform_of_node =
					of_parse_phandle(np, "intel,pcm-audio",
							0);
			else if (!strcmp(dai_link->stream_name, "Voice"))
				/* FIXME: for Voice stream, platform is
				 * snd-soc-dummy */
				dai_link->cpu_of_node = of_parse_phandle(np,
						"intel,pcm-voice", 0);
			else if (!strncmp(dai_link->stream_name,
						"XGOLD_SPEECH_PROBE_",
						strlen("XGOLD_SPEECH_PROBE_")))
				dai_link->cpu_of_node =
					dai_link->platform_of_node =
					of_parse_phandle(np, "intel,speech", 0);
			else if (!strncmp(dai_link->stream_name,
							"XGOLD_HW_PROBE_A",
						strlen("XGOLD_HW_PROBE_A")))
				dai_link->cpu_of_node =
				dai_link->platform_of_node =
					of_parse_phandle(np, "intel,pcm-audio",
							0);
			else if (!strncmp(dai_link->stream_name,
						"XGOLD_HW_PROBE_B",
						strlen("XGOLD_HW_PROBE_B")))
				dai_link->cpu_of_node =
				dai_link->platform_of_node =
					of_parse_phandle(np, "intel,pcm-audio",
							0);
			if (!dai_link->cpu_of_node)
				pr_err("error for %s DAI binding\n",
						dai_link->name);
		}
	}
#endif

	ret = snd_soc_register_card(&xgold_snd_card);
	if (ret < 0) {
		xgold_err("%s: unable to register sound card err %d\n",
			  __func__, ret);
		return ret;
	}

	mc_drv_ctx = devm_kzalloc(&pdev->dev, sizeof(*mc_drv_ctx), GFP_ATOMIC);
	if (!mc_drv_ctx) {
		xgold_err("Allocation failed!\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, mc_drv_ctx);

	/* spk gpio */
	mc_drv_ctx->spk_pin = of_get_named_gpio_flags(np,
			PROP_SPK_AMP_NAME, 0, NULL);

	if (mc_drv_ctx->spk_pin <= 0) {
		xgold_debug("%s: unable to get speaker node %s\n",
			  __func__, PROP_SPK_AMP_NAME);
	} else {
		xgold_debug("%s: Get speaker node %s value: %d !!!\n",
			  __func__, PROP_SPK_AMP_NAME, mc_drv_ctx->spk_pin);
		ret = gpio_request(mc_drv_ctx->spk_pin, PROP_SPK_AMP_NAME);
		if (!ret) {
			xgold_debug("req gpio_request success!:%d\n", ret);
			gpiod_direction_output(
				gpio_to_desc(mc_drv_ctx->spk_pin), 0);
		} else {
			xgold_err("req gpio_request failed:%d\n", ret);
		}
	}

	jack_of_node = of_parse_phandle(np, "intel,jack", 0);
	if (!jack_of_node)
		return 0;

	mc_drv_ctx->jack = of_xgold_jack_probe(pdev, jack_of_node, &hs_jack);
	if (IS_ERR_OR_NULL(mc_drv_ctx->jack)) {
		xgold_err("Jack detection probe failed.\n");
		mc_drv_ctx->jack = NULL;
		/* Allow machine probe to succeed anyway */
	}

	mc_dev_ctx = mc_drv_ctx;

	return 0;
}

static int xgold_mc_remove(struct platform_device *pdev)
{
	struct xgold_mc_private *mc_drv_ctx = platform_get_drvdata(pdev);

	if (mc_drv_ctx) {
		xgold_jack_remove(mc_drv_ctx->jack);
		if (mc_drv_ctx->spk_pin > 0)
			gpio_free(mc_drv_ctx->spk_pin);
	}

	xgold_debug("%s:\n", __func__);
	return 0;
}

static struct of_device_id xgold_snd_asoc_of_match[] = {
	{ .compatible = "intel,xgold-snd-asoc", },
	{ },
};

static int __init setup_xgold_audio_native(char *str)
{
	audio_native_mode = 1;
	return 0;
}
early_param("kernel_audio_standalone", setup_xgold_audio_native);

static struct platform_driver xgold_snd_mc_drv = {
	.driver = {
		.name = "XGOLD_MACHINE",
		.owner = THIS_MODULE,
		.of_match_table = xgold_snd_asoc_of_match,
	},
	.probe = xgold_mc_probe,
	.remove = xgold_mc_remove,
};

static int __init xgold_snd_soc_init(void)
{
	int ret = 0;
	xgold_debug("ALSA SOC XGOLD machine driver init\n");

	ret = platform_driver_register(&xgold_snd_mc_drv);

	if (ret < 0) {
		xgold_err("%s:unable to add machine driver\n", __func__);
		return -ENODEV;
	}

	return ret;
}
module_init(xgold_snd_soc_init);

static void __exit snd_soc_xgold_exit(void)
{
	xgold_debug("%s\n", __func__);
	platform_driver_unregister(&xgold_snd_mc_drv);
}

module_exit(snd_soc_xgold_exit);

MODULE_DESCRIPTION("XGOLD ASOC Machine driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL V2");
MODULE_DEVICE_TABLE(of, xgold_snd_asoc_of_match);
