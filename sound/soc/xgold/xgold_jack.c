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

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>
#endif

#include <sofia/vmm_pmic.h>

#include "xgold_jack.h"

/* FIXME */
#include "../codecs/afe_acc_det.h"

#define	xgold_err(fmt, arg...) \
		pr_err("snd: jack: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: jack: "fmt, ##arg)

/* define the type index for headset typemap */
#define HEADPHONE_INDEX 0
#define HEADSET_INDEX 1

/* this variable to overcome the mute after slow removal of jack */
#define JACK_CHECK_PROSS_START 1
#define JACK_CHECK_PROSS_END   0

#define TIME_TO_NOTIFY_JACK	500 /* msec to ignore spurious interrupt */

/**
 * Different VBIAS settings
**/
enum xgold_vbias {
	XGOLD_VBIAS_ENABLE,
	XGOLD_VBIAS_ULP_ON,
	XGOLD_VBIAS_DISABLE
};

enum xgold_headset_type {
	XGOLD_HEADSET_REMOVED,
	XGOLD_HEADSET,
	XGOLD_HEADPHONE,
	XGOLD_INVALID,
	XGOLD_ERROR
};

struct hs_cfg {
	int min_mv;
	int max_mv;
	enum xgold_headset_type type;
};

struct hs_key_cfg {
	int min_mv;
	int max_mv;
	enum snd_jack_types type;
	int key_code;
	int pressed;
};

static struct timer_list jack_detect_timer;

/* AFE register values */
#define XGOLD_DETECT_INSERTION \
	0x800B /* ACD1: insertion; ACD2: disabled; DEBT: 1msc */
#define XGOLD_DETECT_REMOVAL_HEADSET \
	0xC003 /* ACD1: removal; ACD2: headset insertion; DEBT: 1msc  */
#define XGOLD_DETECT_REMOVAL_HOOK \
	0xCA03 /* ACD1: headset removal; ACD2: hook key press; DEBT: 1msc  */
#define XGOLD_DETECT_HOOK_RELEASE \
	0xC203 /* ACD1: removal; ACD2: hook key release; DEBT: 1msc  */

#define XGOLD_DETECT_INSERTION_WITH_EINT \
	0x0008 /* ACD1: insertion detection disabled; ACD2: disabled */
#define XGOLD_DETECT_REMOVAL_HEADSET_WITH_EINT \
	0x0008 /* ACD1: removal detection disabled; ACD2: headset insertion */
#define XGOLD_DETECT_REMOVAL_HOOK_WITH_EINT \
	0x4A08 /* ACD1: headset removal disabled; ACD2: hook key press */
#define XGOLD_DETECT_HOOK_RELEASE_WITH_EINT \
	0x4208 /* ACD1: removal detection disabled; ACD2: hook key release */


/* PMIC register offset */
/* IRQ registers offsets */
#define IRQMULT_REG			0x1e
#define MIRQMULT_REG			0x1f

/* Masks and bits */
#define IRQMULT_ACCDET1_M		0x01
#define IRQMULT_ACCDET2_M		0x02
#define IRQMULT_ACCDETAUX_M		0x04
#define IRQMULT_ACCDETALL_M \
	(IRQMULT_ACCDET1_M | IRQMULT_ACCDET2_M | IRQMULT_ACCDETAUX_M)
#define IRQMULT_ACCDET_EINT_M \
	(IRQMULT_ACCDET2_M | IRQMULT_ACCDETAUX_M)

/* PMIC registers offsets */
#define ACC_DET_LOW_REG			0x21
#define ACC_DET_HIGH_REG		0x20
#define ACC_DET_AUX_REG			0x23

#define VBIAS_SETTLING_TIME_MS		20

/* Headset keymap */
struct hs_key_cfg xgold_hs_keymap[] = {
	{0, 65, SND_JACK_BTN_0 , KEY_MEDIA, 0},
	{130, 220, SND_JACK_BTN_1, KEY_VOLUMEUP, 0},
	{225, 450, SND_JACK_BTN_2, KEY_VOLUMEDOWN, 0},
	{70, 125, SND_JACK_BTN_3 , KEY_VOICECOMMAND, 0},
};

/* Headset Typemap */
struct hs_cfg xgold_hs_typemap[] = {
	{0, 50, XGOLD_HEADPHONE},
	{475, 1850, XGOLD_HEADSET},
};
static int jack_write(struct xgold_jack *jack, unsigned val)
{
#ifdef CONFIG_X86_INTEL_SOFIA
	return mv_svc_reg_write(jack->base_phys, val, -1);
#else
	iowrite32(val, jack->mmio_base);
	return 0;
#endif
}

static inline int jack_set_pinctrl_state(struct xgold_jack *jack,
		struct pinctrl_state *state)
{
	int ret = 0;

	if (!IS_ERR_OR_NULL(jack->pinctrl)) {
		if (!IS_ERR_OR_NULL(state)) {
			ret = pinctrl_select_state(
					jack->pinctrl,
					state);
			if (ret)
				xgold_err("%s: cannot set pins\n", __func__);
		}
	}
	return ret;
}


/* PMIC reg accesses */
static int xgold_jack_pmic_reg_read(u32 dev_addr, u32 reg_addr,
		u8 *p_reg_val)
{
	u32 vmm_addr, reg_val = 0;
	int ret;

	vmm_addr = ((dev_addr & 0xFF) << 24) | (reg_addr & 0xFF);
	ret = vmm_pmic_reg_read(vmm_addr, &reg_val);
	*p_reg_val = (u8)(reg_val & 0xFF);
	xgold_debug("%s: read @%X return %X\n", __func__, reg_addr, reg_val);

	return ret;
}

static int xgold_jack_pmic_reg_write(u32 dev_addr, u32 reg_addr, u8 reg_val)
{
	u32 vmm_addr, val = reg_val;

	vmm_addr = ((dev_addr & 0xFF) << 24) | (reg_addr & 0xFF);
	xgold_debug("%s: write @%X value %X\n", __func__, reg_addr, val);
	return vmm_pmic_reg_write(vmm_addr, val);
}

/* Call to AFE to change the VBIAS settings */
static void configure_vbias(struct xgold_jack *jack, enum xgold_vbias state)
{
	struct afe_acc_det acc_det_par;
	int ret;

	xgold_debug("--> %s: %s\n", __func__, state ?
		((state == XGOLD_VBIAS_DISABLE) ? "XGOLD_VBIAS_DISABLE" :
		"XGOLD_VBIAS_ULP_ON") : "XGOLD_VBIAS_ENABLE");

	acc_det_par.vumic_conf.vmode = AFE_VUMIC_MODE_ULP;
	acc_det_par.vumic_conf.hzmic = AFE_HZVUMIC_NORMAL_POWER_DOWN;

	switch (state) {
	case XGOLD_VBIAS_ENABLE:
		acc_det_par.vumic_conf.vmicsel = AFE_VMICSEL_2_1_V;
		acc_det_par.micldo_mode = AFE_MICLDO_MODE_NORMAL;
		acc_det_par.xb_mode = AFE_XB_ON;
		break;
	case XGOLD_VBIAS_ULP_ON:
		acc_det_par.vumic_conf.vmicsel = AFE_VMICSEL_1_9_V;
		acc_det_par.micldo_mode = AFE_MICLDO_MODE_LOW_POWER;
		acc_det_par.xb_mode = AFE_XB_OFF;
		break;
	case XGOLD_VBIAS_DISABLE:
		acc_det_par.vumic_conf.vmode = AFE_VUMIC_MODE_POWER_DOWN;
		acc_det_par.vumic_conf.vmicsel = AFE_VMICSEL_1_9_V;
		acc_det_par.micldo_mode = AFE_MICLDO_MODE_OFF;
		acc_det_par.xb_mode = AFE_XB_OFF;
		break;
	default:
		return;
	}

	if (jack->flags & XGOLD_JACK_PMIC)
		ret = pmic_afe_set_acc_det_with_lock(acc_det_par);
	else
		ret = agold_afe_set_acc_det_with_lock(acc_det_par);

	if (ret)
		xgold_err("Error when setting VBIAS!\n");

	xgold_debug("<-- %s\n", __func__);
}

static u32 read_state(struct xgold_jack *jack)
{
	int volt, ret;

	ret = iio_read_channel_processed(jack->iio_client, &volt);
	if (ret < 0) {
		xgold_err("Unable to read channel volt\n");
		return XGOLD_ERROR;
	}

	xgold_debug("%s: measured voltage %d\n", __func__, volt);

	if (volt >= xgold_hs_typemap[HEADSET_INDEX].min_mv &&
			volt <= xgold_hs_typemap[HEADSET_INDEX].max_mv)
		return XGOLD_HEADSET;
	else if (volt >= xgold_hs_typemap[HEADPHONE_INDEX].min_mv &&
			volt <= xgold_hs_typemap[HEADPHONE_INDEX].max_mv)
		return XGOLD_HEADPHONE;
	else if (volt > xgold_hs_typemap[HEADSET_INDEX].max_mv)
		return XGOLD_HEADSET_REMOVED;
	else
		return XGOLD_INVALID;
}

static void xgold_jack_acc_det_write(struct xgold_jack *jack,
		unsigned val)
{
	int ret;

	xgold_debug("%s: write val 0x%X, mode %s\n", __func__, val,
			(jack->flags & XGOLD_JACK_PMIC) ? "PMIC" : "IO");

	if (jack->flags & XGOLD_JACK_PMIC) {
		ret = xgold_jack_pmic_reg_write(jack->pmic_addr,
				ACC_DET_HIGH_REG, (val >> 8) & 0xFF);
		if (ret) {
			xgold_err("%s: cannot write ACC_DET_HIGH\n",
					__func__);
			return;
		}

		ret = xgold_jack_pmic_reg_write(jack->pmic_addr,
				ACC_DET_LOW_REG, val & 0xFF);
		if (ret) {
			xgold_err("%s: cannot write ACC_DET_LOW\n",
					__func__);
			return;
		}
	} else
		jack_write(jack, val);
}

static void xgold_jack_check(struct xgold_jack *jack)
{
	u32 state, old_state;
	u32 detect;
	int status = 0, retry = 10;
	enum xgold_vbias vbias;

	/*  set the flag for button thread to wait until release it.*/
	configure_vbias(jack, XGOLD_VBIAS_ENABLE);

	/* First, make sure we have a stable state.
	   Headset insertion takes a bit of time(~> 500ms),
	   so make sure that two consecutive reads agree.
	*/
	if (jack->use_acd1_for_jack_det) {
		msleep(400);
		do {
			msleep(50);
			old_state = read_state(jack);
			msleep(50);
			state = read_state(jack);
			retry--;
		} while ((state != old_state) ||
			 ((state == XGOLD_ERROR) && (retry)));
	} else {
		/* In gpio detect mechanism Bias voltage stablisation will
		 * take ~> 150ms at after full insertion.So read the state after
		 * 150ms during normal mode and deep sleep mode will
		 * take ~>200ms to get stable.So this retry mechanism will help
		 * to get stable state while system wake up from sleep. */
		if (!jack->hs_plug_detect) {
			msleep(100);
			do {
				msleep(50);
				old_state = read_state(jack);
				msleep(50);
				state = read_state(jack);
				retry--;
			} while ((state != old_state) ||
			 ((state == XGOLD_ERROR) && (retry)));
		} else
			state = XGOLD_HEADSET_REMOVED;
	}

	if (XGOLD_ERROR == state) {
		xgold_err("Unable to determine state.\n");
		return;
	}

	switch (state) {
	case XGOLD_HEADPHONE:
		xgold_debug("Headphone inserted\n");
		vbias = XGOLD_VBIAS_ENABLE;
		if (!jack->use_acd1_for_jack_det) {
			detect = XGOLD_DETECT_REMOVAL_HEADSET_WITH_EINT;
			irq_set_irq_type(jack->jack_irq, IRQ_TYPE_EDGE_RISING);
			jack->hs_plug_detect = true;
			vbias = XGOLD_VBIAS_DISABLE;
		} else
			detect = XGOLD_DETECT_REMOVAL_HEADSET;
		status = SND_JACK_HEADPHONE;
		jack->buttons_enabled = false;
		break;
	case XGOLD_HEADSET:
		xgold_debug("Headset inserted\n");
		vbias = XGOLD_VBIAS_ENABLE;
		if (!jack->use_acd1_for_jack_det) {
			detect = XGOLD_DETECT_REMOVAL_HOOK_WITH_EINT;
			irq_set_irq_type(jack->jack_irq, IRQ_TYPE_EDGE_RISING);
			jack->hs_plug_detect = true;
		} else
			detect = XGOLD_DETECT_REMOVAL_HOOK;
		status = SND_JACK_HEADSET;
		jack->buttons_enabled = true;
		break;
	case XGOLD_HEADSET_REMOVED:
		xgold_debug("Headphone/headset removed\n");
		vbias = XGOLD_VBIAS_ULP_ON;
		if (!jack->use_acd1_for_jack_det) {
			detect = XGOLD_DETECT_INSERTION_WITH_EINT;
			irq_set_irq_type(jack->jack_irq, IRQ_TYPE_EDGE_FALLING);
			jack->hs_plug_detect = false;
			vbias = XGOLD_VBIAS_DISABLE;
		} else
			detect = XGOLD_DETECT_INSERTION;
		jack->buttons_enabled = false;
		break;
	default:
		xgold_debug("Invalid headset state!\n");
		return;
	}

	configure_vbias(jack, vbias);
	msleep(VBIAS_SETTLING_TIME_MS);

	/* Check if there really is a state change */
	if (status != (jack->hs_jack->status & SND_JACK_HEADSET)) {
		xgold_jack_acc_det_write(jack, detect);
		snd_soc_jack_report(jack->hs_jack, status, SND_JACK_HEADSET);
	}
}

static irqreturn_t xgold_jack_detection(int irq, void *data)
{
	struct xgold_jack *jack = data;

	xgold_debug("%s\n", __func__);

	if (jack->use_acd1_for_jack_det && (jack->flags & XGOLD_JACK_PMIC)) {
			xgold_jack_pmic_reg_write(jack->pmic_irq_addr,
					IRQMULT_REG, IRQMULT_ACCDET1_M);
	}

	/* set the flag for button thread to wait until release it.*/
	if (jack->jack_check_in_progress)
		return IRQ_HANDLED;

	/* timer to ignore the spurious interrupt which comes within 500ms
	 * to the prevous jack detection interrupt.*/
	if (!jack->use_acd1_for_jack_det) {
		if (timer_pending(&jack_detect_timer)) {
			return IRQ_HANDLED;
		} else {
			jack_detect_timer.expires =
				jiffies + msecs_to_jiffies(TIME_TO_NOTIFY_JACK);
			add_timer(&jack_detect_timer);
		}
	}
	/* get the wake lock to aviod the system enter into sleep
	 * during ADC measurement */
	wake_lock(&jack->wlock);

	jack->jack_check_in_progress = JACK_CHECK_PROSS_START;

	xgold_jack_check((struct xgold_jack *)data);

	/* release the flag for button thread to continue.*/
	jack->jack_check_in_progress = JACK_CHECK_PROSS_END;

	/* release the wake lock*/
	wake_unlock(&jack->wlock);

	return IRQ_HANDLED;
}

static void xgold_button_check(struct xgold_jack *jack)
{
	int ret, volt, i;
	int key_index = -1;
	u32 detect;
	int status;
	enum snd_jack_types type;

	/* wait to check the interrupt due to slow removal of jack */
	msleep_interruptible(30);
	/* If the interrupt due to slow removal of jack,*/
	/*return without action */
	if (jack->jack_check_in_progress)
		return;

	ret = iio_read_channel_processed(jack->iio_client, &volt);
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
		xgold_jack_acc_det_write(jack, detect);
	}
}

static irqreturn_t xgold_button_detection(int irq, void *data)
{
	struct xgold_jack *jack = data;

	xgold_debug("%s\n", __func__);

	if (jack->flags & XGOLD_JACK_PMIC)
		xgold_jack_pmic_reg_write(jack->pmic_irq_addr,
				IRQMULT_REG, IRQMULT_ACCDET2_M);

	if (!jack->use_acd1_for_jack_det && !jack->hs_plug_detect)
		return IRQ_HANDLED;

	if ((jack->hs_jack->status & SND_JACK_HEADSET) != SND_JACK_HEADSET) {
		/* this interrupt may occurs in case of slow jack insertion */
		xgold_debug("button detection while no headset\n");
		return xgold_jack_detection(irq, data);
	}

	if (jack->buttons_enabled)
		xgold_button_check(jack);

	return IRQ_HANDLED;
}

static void xgold_jack_notify_cb(unsigned long data)
{
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
	unsigned value;
	unsigned num_button_val = ARRAY_SIZE(xgold_hs_keymap)*2;
	unsigned button_array[num_button_val];
	unsigned num_hs_type_val = ARRAY_SIZE(xgold_hs_typemap)*2;
	unsigned hs_type_array[num_hs_type_val];

	jack = devm_kzalloc(&pdev->dev, sizeof(*jack), GFP_ATOMIC);
	if (!jack) {
		xgold_err("Allocation failed!\n");
		ret = -ENOMEM;
		goto out;
	}

	jack->buttons_enabled = false;
	jack->jack_irq = jack->button_irq = -1;
	jack->hs_jack = hs_jack;
	jack->jack_check_in_progress = JACK_CHECK_PROSS_END;

	if (of_device_is_compatible(np, "intel,headset,pmic"))
		jack->flags |= XGOLD_JACK_PMIC;

	if (jack->flags & XGOLD_JACK_PMIC) {
		/* PMIC device address */
		ret = of_property_read_u32_index(np, "intel,reg", 0, &value);
		if (ret)
			goto out;

		if (value > 0xFF) {
			ret = -ERANGE;
			goto out;
		}

		jack->pmic_addr = (unsigned char)value;

		/* FIXME: should be handled by VMM, not linux driver */
		/* PMIC device address for IRQ handling */
		ret = of_property_read_u32_index(np, "intel,irq-reg", 0,
				&value);
		if (ret)
			goto out;

		if (value > 0xFF) {
			ret = -ERANGE;
			goto out;
		}

		jack->pmic_irq_addr = (unsigned char)value;
	} else {
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
		jack->base_phys = regs.start;
		xgold_debug("ioremap %p\n", jack->mmio_base);
	}

	num_irq = of_irq_count(np);
	if (!num_irq) {
		xgold_err("no headset plug irq defined\n");
		ret = -EINVAL;
		goto out;
	}

	/* pinctrl */
	jack->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(jack->pinctrl)) {
		xgold_err("no pinctrl !\n");
		jack->pinctrl = NULL;
		goto skip_pinctrl;
	}

	jack->pins_default = pinctrl_lookup_state(jack->pinctrl,
						 PINCTRL_STATE_DEFAULT);
	if (IS_ERR(jack->pins_default))
		xgold_err("could not get default pinstate\n");

	jack->pins_sleep = pinctrl_lookup_state(jack->pinctrl,
					       PINCTRL_STATE_SLEEP);
	if (IS_ERR(jack->pins_sleep))
		xgold_err("could not get sleep pinstate\n");

	jack->pins_inactive =
		pinctrl_lookup_state(jack->pinctrl,
					       "inactive");
	if (IS_ERR(jack->pins_inactive))
		xgold_err("could not get inactive pinstate\n");

skip_pinctrl:

	res = devm_kzalloc(&pdev->dev, sizeof(*res) * num_irq, GFP_KERNEL);
	if (!res) {
		ret = -ENOMEM;
		goto out;
	}

	of_irq_to_resource_table(np, res, num_irq);
	for (i = 0; i < num_irq; i++)
		if (strncmp(res[i].name, "acd1", sizeof("acd1")) == 0) {
			jack->jack_irq = res[i].start;
			jack->use_acd1_for_jack_det = true;
		} else if (strncmp(res[i].name, "jack_det_eint",
			sizeof("jack_det_eint")) == 0) {
			jack->jack_irq = res[i].start;
			jack->use_acd1_for_jack_det = false;
		}

	xgold_debug("%s: use %s interrupt\n", __func__,
			(jack->use_acd1_for_jack_det) ? "ACD" : "EINT");

	jack->iio_client = iio_channel_get(NULL, "ACCID_SENSOR");
	if (IS_ERR(jack->iio_client)) {
		xgold_err("iio channel error\n");
		ret = -EINVAL;
		goto out;
	}

	wake_lock_init(&jack->wlock, WAKE_LOCK_SUSPEND, "intel_acc");

	/* Configure the Accessory settings to detect Insertion */
	if (!jack->use_acd1_for_jack_det) {

		jack->hs_plug_detect = false;

		init_timer(&jack_detect_timer);
		jack_detect_timer.function = xgold_jack_notify_cb;

		ret = jack_set_pinctrl_state(jack, jack->pins_default);
		if (ret) {
			xgold_err("setup of default pinctrl state failed!\n");
			goto out;
		}

		ret = devm_request_threaded_irq(&(pdev->dev), jack->jack_irq,
				NULL,
				xgold_jack_detection,
				IRQF_SHARED | IRQF_ONESHOT | IRQF_NO_SUSPEND |
				IRQ_TYPE_LEVEL_LOW, "jack_irq", jack);
	} else {
		xgold_jack_acc_det_write(jack, XGOLD_DETECT_INSERTION);

		ret = devm_request_threaded_irq(&(pdev->dev), jack->jack_irq,
				NULL,
				xgold_jack_detection,
				IRQF_SHARED | IRQF_ONESHOT | IRQF_NO_SUSPEND,
				"jack_irq", jack);
	}

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

	/* FIXME: below code should be handled by irqchip level/vmm, when
	 * requesting for the PMIC ACD interrupt, and not in this driver */
	if (jack->flags & XGOLD_JACK_PMIC) {
		int tries;
		char val;
		u8 mask = (jack->use_acd1_for_jack_det) ?
			IRQMULT_ACCDETALL_M : IRQMULT_ACCDET_EINT_M;

		/* Unmask IRQMULT interrupt */
		xgold_err("%s: Warning! may apply changes to MIRQMULT register\n",
				__func__);

		xgold_jack_pmic_reg_read(jack->pmic_irq_addr,
				MIRQMULT_REG, &val);
		tries = 0;
		while ((val & mask) && tries++ < 20) {
			xgold_jack_pmic_reg_write(jack->pmic_irq_addr,
					MIRQMULT_REG,
					val & ~mask);

			/* read again to ensure Mask is correctly configured */
			xgold_jack_pmic_reg_read(jack->pmic_irq_addr,
					MIRQMULT_REG, &val);
			xgold_debug("%s: MIRQMULT is 0x%02X\n", __func__, val);
		}

		if (tries >= 20) {
			ret = -EIO;
			goto out;
		}

		xgold_err("%s MIRQLVL1 is 0x%02X\n", __func__, val);
	}
	/* end of FIXME */

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"intel,hs-button-levels", button_array, num_button_val);
	if (ret) {
		pr_debug("intel,hs-button-levels not found,use default values.\n");
	} else {
		value = 0;
		for (i = 0; i < (num_button_val/2); i++) {
			xgold_hs_keymap[i].min_mv = button_array[value++];
			xgold_hs_keymap[i].max_mv = button_array[value++];
		}
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"intel,hs-type-levels", hs_type_array, num_hs_type_val);
	if (ret) {
		pr_debug("intel,hs-type-levels not found,use default values.\n");
	} else {
		value = 0;
		for (i = 0; i < (num_hs_type_val/2); i++) {
			xgold_hs_typemap[i].min_mv = hs_type_array[value++];
			xgold_hs_typemap[i].max_mv = hs_type_array[value++];
		}
	}

	return jack;

out:
	return ERR_PTR(ret);
}

void xgold_jack_remove(struct xgold_jack *jack)
{
	if (jack) {
		if (!IS_ERR(jack->iio_client))
			iio_channel_release(jack->iio_client);

		disable_irq(jack->jack_irq);
		wake_lock_destroy(&jack->wlock);

		if (!jack->use_acd1_for_jack_det) {
			jack_set_pinctrl_state(jack, jack->pins_inactive);
			del_timer_sync(&jack_detect_timer);
		}
	}
}
