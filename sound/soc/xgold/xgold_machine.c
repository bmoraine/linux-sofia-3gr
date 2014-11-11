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

#include "agold620-headset.h"

#define PCL_LOCK_EXCLUSIVE	1
#define PCL_OPER_ACTIVATE	0
#define PROP_CODEC_DAI_NAME "intel,codec_dai_name"


#define	xgold_err(fmt, arg...) \
		pr_err("snd: mac: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: mac: "fmt, ##arg)

int audio_native_mode;

struct xgold_mc_private {
	struct iio_channel *iio_client;
	unsigned int jack_irq;
	unsigned int button_irq;
	void __iomem *mmio_base;
	int buttons_enabled;
};

static struct snd_soc_jack hs_jack;

/* XGOLD machine DAPM */
static const struct snd_soc_dapm_widget xgold_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
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

	{"Headset Mic", NULL, "AMIC2"},
	{"Handset Mic", NULL, "AMIC1"}
};

static int xgold_snd_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	int ret = 0;
	int i;
	int type;

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

	type = SND_JACK_HEADSET;
	for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++)
		type |= xgold_hs_keymap[i].type;

	ret = snd_soc_jack_new(codec, "XGOLD Audio Jack",
			type,
			&hs_jack);

	if (ret) {
		xgold_err("Jack creation failed\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
		ret = snd_jack_set_key(hs_jack.jack,
			xgold_hs_keymap[i].type,
			xgold_hs_keymap[i].key_code);
		if (ret) {
			xgold_err("Failed to set headset key\n");
			return ret;
		}
	}

	return ret;
}

/* Digital audio interface glue - connects codec <--> CPU
 * xgold dai's are devided into Front End (FE) and Back End (BE) devices
 * FE devices are pure streaming devices, not in charge of enabling any codecs.
 * BE devices are used for enabling codecs alone.
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

#ifdef CONFIG_SND_SOC_XGOLD632_SPEECH_PROBE
	/* Speech probe front end devices */
	{
		.name = "XGOLD_SPEECH_PROBE_A",
		.stream_name = "Speech Probe_A",
		.ignore_suspend = 1,
	},
	{
		.name = "XGOLD_SPEECH_PROBE_B",
		.stream_name = "Speech Probe_B",
		.ignore_suspend = 1,
	},
	{
		.name = "XGOLD_SPEECH_PROBE_C",
		.stream_name = "Speech Probe_C",
		.ignore_suspend = 1,
	},
	{
		.name = "XGOLD_SPEECH_PROBE_D",
		.stream_name = "Speech Probe_D",
		.ignore_suspend = 1,
	},
	/* ALSA allows only 8 pcm devices with
	static minor numbers check after LTE PO?*/
#ifdef CONFIG_INCREASE_PCM_DEVICE
	{
		.name = "XGOLD_SPEECH_PROBE_E",
		.stream_name = "Speech Probe_E",
		.ignore_suspend = 1,
	},
	{
		.name = "XGOLD_SPEECH_PROBE_F",
		.stream_name = "Speech Probe_F",
		.ignore_suspend = 1,
	}
#endif
#endif
};

/* Audio machine driver */
static struct snd_soc_card xgold_snd_card = {
	.name = "xgold_afe-audio",
	.owner = THIS_MODULE,
	.dai_link = xgold_dai,
	.num_links = ARRAY_SIZE(xgold_dai),
};

/* Call to AFE to change the VBIAS settings */
static void configure_vbias(enum xgold_vbias state)
{
	struct agold_afe_acc_det acc_det_par;

	switch (state) {
	case XGOLD_VBIAS_ENABLE:
		acc_det_par.vumic_conf.vmode = AGOLD_AFE_VUMIC_MODE_ULP;
		acc_det_par.vumic_conf.hzmic =
			AGOLD_AFE_HZVUMIC_NORMAL_POWER_DOWN;
		acc_det_par.vumic_conf.vmicsel = AGOLD_AFE_VMICSEL_2_1_V;
		acc_det_par.micldo_mode = AGOLD_AFE_MICLDO_MODE_NORMAL;
		acc_det_par.xb_mode = AGOLD_AFE_XB_ON;
		break;
	case XGOLD_VBIAS_ULP_ON:
		acc_det_par.vumic_conf.vmode = AGOLD_AFE_VUMIC_MODE_ULP;
		acc_det_par.vumic_conf.hzmic =
			AGOLD_AFE_HZVUMIC_NORMAL_POWER_DOWN;
		acc_det_par.vumic_conf.vmicsel = AGOLD_AFE_VMICSEL_1_9_V;
		acc_det_par.micldo_mode = AGOLD_AFE_MICLDO_MODE_LOW_POWER;
		acc_det_par.xb_mode = AGOLD_AFE_XB_OFF;
		break;
	default:
		return;
	}

	if (agold_afe_set_acc_det_with_lock(acc_det_par))
		xgold_err("Error when setting VBIAS!\n");

}

static u32 read_state(struct xgold_mc_private *mc_drv_ctx)
{
	int volt, ret;

	ret = iio_read_channel_raw(mc_drv_ctx->iio_client, &volt);
	if (ret < 0) {
		xgold_err("Unable to read channel volt\n");
		return XGOLD_ERROR;
	}

	if (volt >= AHJ_TYPE_MIN_MV &&
			volt <= AHJ_TYPE_MAX_MV)
			return XGOLD_HEAD_SET;
		else if (volt >= HEADPHONE_MIN_MV &&
				volt <= HEADPHONE_MAX_MV)
			return XGOLD_HEAD_PHONE;
		else if (volt > AHJ_TYPE_MAX_MV)
			return XGOLD_HEADSET_REMOVED;
		else
			return XGOLD_INVALID;
}

static void xgold_jack_check(struct xgold_mc_private *mc_drv_ctx)
{
	u32 state, old_state;
	u32 detect;
	int status = 0;
	enum xgold_vbias vbias;

	configure_vbias(XGOLD_VBIAS_ENABLE);

	/* First, make sure we have a stable state.
	   Headset insertion takes a bit of time(~> 500ms),
	   so make sure that two consecutive reads agree.
	*/
	do {
		msleep(250);
		old_state = read_state(mc_drv_ctx);
		msleep(250);
		state = read_state(mc_drv_ctx);
	} while (state != old_state);

	if (XGOLD_ERROR == state) {
		xgold_err("Unable to determine state.\n");
		return;
	}

	switch (state) {
	case XGOLD_HEAD_PHONE:
		xgold_debug("Headphone inserted\n");
		vbias = XGOLD_VBIAS_ULP_ON;
		detect = XGOLD_DETECT_REMOVAL;
		status = SND_JACK_HEADPHONE;
		mc_drv_ctx->buttons_enabled = false;
		break;
	case XGOLD_HEAD_SET:
		xgold_debug("Headset inserted\n");
		vbias = XGOLD_VBIAS_ENABLE;
		detect = XGOLD_DETECT_REMOVAL_HOOK;
		status = SND_JACK_HEADSET;
		mc_drv_ctx->buttons_enabled = true;
		break;
	case XGOLD_HEADSET_REMOVED:
		xgold_debug("Headphone/headset removed\n");
		vbias = XGOLD_VBIAS_ULP_ON;
		detect = XGOLD_DETECT_INSERTION;
		mc_drv_ctx->buttons_enabled = false;
		break;
	default:
		xgold_debug("Invalid headset state!\n");
		return;
	}

	configure_vbias(vbias);
	msleep(VBIAS_SETTLING_TIME_MS);

	/* Check if there really is a state change */
	if (status != (hs_jack.status & SND_JACK_HEADSET)) {
		iowrite32(detect, mc_drv_ctx->mmio_base);
		snd_soc_jack_report(&hs_jack, status, SND_JACK_HEADSET);
	}
}

static irqreturn_t xgold_jack_detection(int irq, void *data)
{
	xgold_jack_check((struct xgold_mc_private *)data);
	return IRQ_HANDLED;
}

static void xgold_button_check(struct xgold_mc_private *mc_drv_ctx)
{
	int ret, volt, i;
	int key_index = -1;
	u32 detect;
	int status;
	enum snd_jack_types type;

	ret = iio_read_channel_raw(mc_drv_ctx->iio_client, &volt);
	if (ret < 0) {
		xgold_err("Unable to read channel volt\n");
		return;
	}

	/* check if a key has been pressed and remember this*/
	for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
		if ((volt >= xgold_hs_keymap[i].min_mv) &&
			(volt <= xgold_hs_keymap[i].max_mv)) {
			xgold_hs_keymap[i].pressed = 1;
			key_index = i;
			break;
		}
	}

	if (key_index > -1) {
		xgold_debug("button press index %d\n", key_index);
		type = xgold_hs_keymap[key_index].type;
		status = type;
		detect = XGOLD_DETECT_HOOK_RELEASE;
	} else {
		/* key released, figure out which */
		for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
			if (1 == xgold_hs_keymap[i].pressed) {
				xgold_debug("button release, index %d\n", i);
				xgold_hs_keymap[i].pressed = 0;
				key_index = i;
				type = xgold_hs_keymap[key_index].type;
				status = 0;
			}
		}
		detect = XGOLD_DETECT_REMOVAL_HOOK;
	}

	if (key_index > -1) {
		snd_soc_jack_report(&hs_jack, status, type);
		iowrite32(detect, mc_drv_ctx->mmio_base);
	}
}

static irqreturn_t xgold_button_detection(int irq, void *data)
{
	struct xgold_mc_private *mc_drv_ctx = data;
	if (mc_drv_ctx->buttons_enabled)
		xgold_button_check(mc_drv_ctx);
	return IRQ_HANDLED;
}

static int xgold_jack_probe(struct platform_device *pdev,
		struct xgold_mc_private *mc_drv_ctx)
{
	struct resource *res;
	int ret;

	mc_drv_ctx->buttons_enabled = 0;

	res = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "headset-registers");
	if (res == NULL) {
		xgold_err("no I/O memory defined in platform data\n");
		return -EINVAL;
	}

	mc_drv_ctx->mmio_base = ioremap(res->start, resource_size(res));

	xgold_debug("ioremap %p\n", mc_drv_ctx->mmio_base);

	if (mc_drv_ctx->mmio_base == NULL) {
		xgold_err("failed to remap I/O memory\n");
		return -ENOMEM;
	}

	mc_drv_ctx->jack_irq =
			platform_get_irq_byname(pdev, "acd1");
	if (mc_drv_ctx->jack_irq < 0) {
		xgold_err("no headset plug irq defined\n");
		return -EINVAL;
	}

	mc_drv_ctx->iio_client = iio_channel_get(NULL, "ACCID_SENSOR");
	if (IS_ERR(mc_drv_ctx->iio_client)) {
		xgold_err("iio channel error\n");
		return -EINVAL;
	}

	/* Configure the Accessory settings to detect Insertion */
	iowrite32(XGOLD_DETECT_INSERTION, mc_drv_ctx->mmio_base);

	ret = devm_request_threaded_irq(&(pdev->dev), mc_drv_ctx->jack_irq,
			NULL,
			xgold_jack_detection,
			IRQF_SHARED | IRQF_ONESHOT, "jack_irq", mc_drv_ctx);
	if (ret) {
		xgold_err("setup of jackirq failed!\n");
		return -EINVAL;
	}

	mc_drv_ctx->button_irq =
			platform_get_irq_byname(pdev, "acd2");
	if (mc_drv_ctx->button_irq < 0) {
		xgold_err("no irq_hook irq defined\n");
		return -EINVAL;
	}

	ret = devm_request_threaded_irq(&(pdev->dev), mc_drv_ctx->button_irq,
			NULL,
			xgold_button_detection,
			IRQF_SHARED | IRQF_ONESHOT, "button_irq", mc_drv_ctx);
	if (ret < 0) {
		xgold_err("setup of button irq failed!\n");
		return -EINVAL;
	}

	return 0;
}

static int xgold_mc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *codec_of_node = NULL;
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

			if (!strcmp(dai_link->stream_name, "PCM Audio")) {
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
						"Speech Probe",
						strlen("Speech Probe"))) {
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

			if (!strcmp(dai_link->stream_name, "PCM Audio"))
				dai_link->cpu_of_node =
					dai_link->platform_of_node =
					of_parse_phandle(np, "intel,pcm-audio",
							0);
			else if (!strcmp(dai_link->stream_name, "Voice"))
				/* FIXME: for Voice stream, platform is
				 * snd-soc-dummy */
				dai_link->cpu_of_node = of_parse_phandle(np,
						"intel,pcm-voice", 0);
			else if (!strncmp(dai_link->stream_name, "Speech Probe",
						strlen("Speech Probe")))
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

	ret = xgold_jack_probe(pdev, mc_drv_ctx);
	if (ret < 0) {
		xgold_err("Jack detection probe failed.\n");
		/* Allow machine probe to succeed anyway */
	}

	return 0;
}

static int xgold_mc_remove(struct platform_device *pdev)
{
	struct xgold_mc_private *mc_drv_ctx = platform_get_drvdata(pdev);

	if (mc_drv_ctx && mc_drv_ctx->iio_client)
		iio_channel_release(mc_drv_ctx->iio_client);

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
early_param("nomodem", setup_xgold_audio_native);

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
