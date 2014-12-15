/*
 * Component: XGOLD audio jack driver
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/consumer.h>
#include <linux/iio/driver.h>
#include <sound/soc.h>

#include "xgold_jack.h"

/* FIXME */
#include "../codecs/agold_acc_det.h"

#define	xgold_err(fmt, arg...) \
		pr_err("snd: jack: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: jack: "fmt, ##arg)


#define AHJ_TYPE_MIN_MV 475
#define AHJ_TYPE_MAX_MV 1700

#define HEADPHONE_MIN_MV 0
#define HEADPHONE_MAX_MV 50

/**
 * Different VBIAS settings
**/
enum xgold_vbias {
	XGOLD_VBIAS_ENABLE,
	XGOLD_VBIAS_ULP_ON,
};

enum xgold_headset_type {
	XGOLD_HEADSET_REMOVED,
	XGOLD_HEADSET,
	XGOLD_HEADPHONE,
	XGOLD_INVALID,
	XGOLD_ERROR
};

/*struct hs_cfg {
	int min_mv;
	int max_mv;
	enum xgold_headset_type type;
};*/

struct hs_key_cfg {
	int min_mv;
	int max_mv;
	enum snd_jack_types type;
	int key_code;
	int pressed;
};

/* AFE register values */
#define XGOLD_DETECT_INSERTION		0x80FB
#define XGOLD_DETECT_REMOVAL		0xC0F3
#define XGOLD_DETECT_REMOVAL_HOOK	0xCAF3
#define XGOLD_DETECT_HOOK_RELEASE	0xC2F3

#define VBIAS_SETTLING_TIME_MS		20

/* Headset keymap */
struct hs_key_cfg xgold_hs_keymap[] = {
	{0, 50, SND_JACK_BTN_0 , KEY_MEDIA, 0},
	{100, 150, SND_JACK_BTN_1, KEY_VOLUMEUP, 0},
	{275, 325, SND_JACK_BTN_2, KEY_VOLUMEDOWN, 0},
};

/* Call to AFE to change the VBIAS settings */
static void configure_vbias(enum xgold_vbias state)
{
	struct agold_afe_acc_det acc_det_par;

	xgold_debug("--> %s: %s\n", __func__, (state == XGOLD_VBIAS_ENABLE) ?
			"XGOLD_VBIAS_ENABLE" : "XGOLD_VBIAS_ULP_ON");

	acc_det_par.vumic_conf.vmode = AGOLD_AFE_VUMIC_MODE_ULP;
	acc_det_par.vumic_conf.hzmic = AGOLD_AFE_HZVUMIC_NORMAL_POWER_DOWN;

	switch (state) {
	case XGOLD_VBIAS_ENABLE:
		acc_det_par.vumic_conf.vmicsel = AGOLD_AFE_VMICSEL_2_1_V;
		acc_det_par.micldo_mode = AGOLD_AFE_MICLDO_MODE_NORMAL;
		acc_det_par.xb_mode = AGOLD_AFE_XB_ON;
		break;
	case XGOLD_VBIAS_ULP_ON:
		acc_det_par.vumic_conf.vmicsel = AGOLD_AFE_VMICSEL_1_9_V;
		acc_det_par.micldo_mode = AGOLD_AFE_MICLDO_MODE_LOW_POWER;
		acc_det_par.xb_mode = AGOLD_AFE_XB_OFF;
		break;
	default:
		return;
	}

	if (agold_afe_set_acc_det_with_lock(acc_det_par))
		xgold_err("Error when setting VBIAS!\n");

	xgold_debug("<-- %s\n", __func__);
}

static u32 read_state(struct xgold_jack *jack)
{
	int volt, ret;

	ret = iio_read_channel_raw(jack->iio_client, &volt);
	if (ret < 0) {
		xgold_err("Unable to read channel volt\n");
		return XGOLD_ERROR;
	}

	xgold_debug("%s: measured voltage %d\n", __func__, volt);

	if (volt >= AHJ_TYPE_MIN_MV && volt <= AHJ_TYPE_MAX_MV)
		return XGOLD_HEADSET;
	else if (volt >= HEADPHONE_MIN_MV && volt <= HEADPHONE_MAX_MV)
		return XGOLD_HEADPHONE;
	else if (volt > AHJ_TYPE_MAX_MV)
		return XGOLD_HEADSET_REMOVED;
	else
		return XGOLD_INVALID;
}

static void xgold_jack_check(struct xgold_jack *jack)
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
		old_state = read_state(jack);
		msleep(250);
		state = read_state(jack);
	} while (state != old_state);

	if (XGOLD_ERROR == state) {
		xgold_err("Unable to determine state.\n");
		return;
	}

	switch (state) {
	case XGOLD_HEADPHONE:
		xgold_debug("Headphone inserted\n");
		vbias = XGOLD_VBIAS_ULP_ON;
		detect = XGOLD_DETECT_REMOVAL;
		status = SND_JACK_HEADPHONE;
		jack->buttons_enabled = false;
		break;
	case XGOLD_HEADSET:
		xgold_debug("Headset inserted\n");
		vbias = XGOLD_VBIAS_ENABLE;
		detect = XGOLD_DETECT_REMOVAL_HOOK;
		status = SND_JACK_HEADSET;
		jack->buttons_enabled = true;
		break;
	case XGOLD_HEADSET_REMOVED:
		xgold_debug("Headphone/headset removed\n");
		vbias = XGOLD_VBIAS_ULP_ON;
		detect = XGOLD_DETECT_INSERTION;
		jack->buttons_enabled = false;
		break;
	default:
		xgold_debug("Invalid headset state!\n");
		return;
	}

	configure_vbias(vbias);
	msleep(VBIAS_SETTLING_TIME_MS);

	/* Check if there really is a state change */
	if (status != (jack->hs_jack->status & SND_JACK_HEADSET)) {
		iowrite32(detect, jack->mmio_base);
		snd_soc_jack_report(jack->hs_jack, status, SND_JACK_HEADSET);
	}
}

static irqreturn_t xgold_jack_detection(int irq, void *data)
{
	xgold_jack_check((struct xgold_jack *)data);
	return IRQ_HANDLED;
}

static void xgold_button_check(struct xgold_jack *jack)
{
	int ret, volt, i;
	int key_index = -1;
	u32 detect;
	int status;
	enum snd_jack_types type;

	ret = iio_read_channel_raw(jack->iio_client, &volt);
	if (ret < 0) {
		xgold_err("Unable to read channel volt\n");
		return;
	}

	xgold_debug("%s: measured voltage %d\n", __func__, volt);

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
		snd_soc_jack_report(jack->hs_jack, status, type);
		iowrite32(detect, jack->mmio_base);
	}
}

static irqreturn_t xgold_button_detection(int irq, void *data)
{
	struct xgold_jack *jack = data;
	if (jack->buttons_enabled)
		xgold_button_check(jack);
	return IRQ_HANDLED;
}

int xgold_jack_setup(struct snd_soc_codec *codec, struct snd_soc_jack *hs_jack)
{
	int i, type, ret;

	xgold_debug("%s\n", __func__);

	type = SND_JACK_HEADSET;
	for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++)
		type |= xgold_hs_keymap[i].type;

	ret = snd_soc_jack_new(codec, "Headset", type, hs_jack);

	if (ret) {
		xgold_err("Jack creation failed\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
		ret = snd_jack_set_key(hs_jack->jack,
			xgold_hs_keymap[i].type,
			xgold_hs_keymap[i].key_code);
		if (ret)
			xgold_err("Failed to set headset key\n");
	}

	return ret;
}

struct xgold_jack *of_xgold_jack_probe(struct platform_device *pdev,
		struct device_node *np, struct snd_soc_jack *hs_jack)
{
	struct xgold_jack *jack;
	struct resource regs;
	struct resource *res;
	int num_irq, i, ret;

	jack = devm_kzalloc(&pdev->dev, sizeof(*jack), GFP_ATOMIC);
	if (!jack) {
		xgold_err("Allocation failed!\n");
		ret = -ENOMEM;
		goto out;
	}

	jack->buttons_enabled = false;
	jack->jack_irq = jack->button_irq = -1;
	jack->hs_jack = hs_jack;

	if (of_address_to_resource(np, 0, &regs)) {
		ret = -ENOENT;
		goto out;
	}

	jack->mmio_base = devm_ioremap(
			&pdev->dev, regs.start, resource_size(&regs));
	if (jack->mmio_base == NULL) {
		xgold_err("failed to remap I/O memory\n");
		ret = -ENXIO;
		goto out;
	}

	xgold_debug("ioremap %p\n", jack->mmio_base);

	num_irq = of_irq_count(np);
	if (!num_irq) {
		xgold_err("no headset plug irq defined\n");
		ret = -EINVAL;
		goto out;
	}

	res = devm_kzalloc(&pdev->dev, sizeof(*res) * num_irq, GFP_KERNEL);
	if (!res) {
		ret = -ENOMEM;
		goto out;
	}

	of_irq_to_resource_table(np, res, num_irq);
	for (i = 0; i < num_irq; i++)
		if (strcmp(res[i].name, "acd1") == 0)
			jack->jack_irq = res[i].start;

	jack->iio_client = iio_channel_get(NULL, "ACCID_SENSOR");
	if (IS_ERR(jack->iio_client)) {
		xgold_err("iio channel error\n");
		ret = -EINVAL;
		goto out;
	}

	/* Configure the Accessory settings to detect Insertion */
	iowrite32(XGOLD_DETECT_INSERTION, jack->mmio_base);

	ret = devm_request_threaded_irq(&(pdev->dev), jack->jack_irq,
			NULL,
			xgold_jack_detection,
			IRQF_SHARED | IRQF_ONESHOT, "jack_irq", jack);
	if (ret) {
		xgold_err("setup of jack irq failed!\n");
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < num_irq; i++)
		if (strcmp(res[i].name, "acd2") == 0)
			jack->button_irq = res[i].start;

	ret = devm_request_threaded_irq(&(pdev->dev), jack->button_irq,
			NULL,
			xgold_button_detection,
			IRQF_SHARED | IRQF_ONESHOT, "button_irq", jack);
	if (ret < 0) {
		xgold_err("setup of button irq failed!\n");
		ret = -EINVAL;
		goto out;
	}

	return jack;

out:
	return ERR_PTR(ret);
}

void xgold_jack_remove(struct xgold_jack *jack)
{
	if (jack && jack->iio_client)
		iio_channel_release(jack->iio_client);
}
