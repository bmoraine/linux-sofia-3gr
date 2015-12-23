/*
 * Component: AGOLD AFE ALSA SOC driver
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

#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/reset.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>

#include "agold_afe.h"

#ifdef CONFIG_SND_SOC_AGOLD_ACC_DET_INTERFACE
#include "afe_acc_det.h"
#endif

#define AFE_MAX_SAMPLING_RATE 3
#define AGOLD_AFE_NOF_BYTES_PER_REG 4

#ifdef CONFIG_SND_SOC_AGOLD_HSOFC_SUPPORT
#define LEFT_CHANNEL  0
#define RIGHT_CHANNEL 1

#define AFE_NOF_HSOSC_TRIM_VALUES 64
/* Number of times AFE HSOSC comparator output has to be read for getting the
   calibration result
*/
#define AFE_HSOSC_READ_COUNT 1500
/* Confidence measure to declare a successful calibration */
#define AFE_COMPARATOR_VALIDATE_COUNT 800
/* Number of gain stages for offset calibration */
#define AFE_NVM_NOF_HSOSC 8

/* Calibration result returned to user space */
struct cal_result {
	u8 hs_calib_done;
	/* Container for final offset calibration values */
	struct channels  hs_ofs_cal_val[AFE_NVM_NOF_HSOSC];
	u32 timestamp_cal_left[AFE_NOF_EPHSLS_HW_GAIN_VALUES];
	u32 timestamp_cal_right[AFE_NOF_EPHSLS_HW_GAIN_VALUES];
};

static struct cal_result hs_cal_result;

/* Indicates values to be set in AFE registers according to corresponding
   index
*/
static const u8 hs_ofs_conv[AFE_NOF_HSOSC_TRIM_VALUES] = {
	63, 62, 61, 60, 59, 58, 57, 56, 55, 54,
	53, 52, 51, 50, 49, 48, 47, 46, 45, 44,
	43, 42, 41, 40, 39, 38, 37, 36, 35, 34,
	33, 32, 0, 1, 2, 3, 4, 5, 6, 7,
	8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
	18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
	28, 29, 30, 31
};

/* Container for offset calibration values for step1*/
static struct channels  hs_ofs_cal_val_step1[AFE_NVM_NOF_HSOSC];

struct hsosc_ctl {
	u16 num_gains;
	u16 length;
	char data[0];
};

struct hsosc_data {
	u16 num_gains;
	u16 length;
	u16 *p_data;
};

/* It is observed that offset values obtained through calibration always show
   a fixed deviation from the actual measured values. This array contains the
   calculated deviation which has to be added/subtracted accordingly
*/
static struct channels hs_ofs_delta[AUD_ANALOG_GAIN_END+1] = {
	{6, 18},     /* -9.1 dB  */
	{6, 18},     /* -9.1 dB  */
	{6, 15},     /* -6 dB    */
	{5, 11},     /* -3.1 dB  */
	{5,  6},     /*  0 -dB   */
	{3,  4},     /*  6 dB  */
	{3,  4},     /*  6 dB  */
	{0,  0}      /*  Invalid */
};

static u16 afe_hal_hs_gain[AFE_NOF_EPHSLS_HW_GAIN_VALUES] = {
	AFE_GAIN_OUT_HSGAIN_HS_M9_1DB, /* Max-18 dB -9.1 dB */
	AFE_GAIN_OUT_HSGAIN_HS_M9_1DB, /* Max-15 dB      -9.1 dB(In target) */
	AFE_GAIN_OUT_HSGAIN_HS_M6_0DB, /* Max-12 dB      -6   dB(In target) */
	AFE_GAIN_OUT_HSGAIN_HS_M3_1DB, /* Max-9  dB      -3.1 dB(In target) */
	AFE_GAIN_OUT_HSGAIN_HS_P0_0DB, /* Max-6  dB       0   dB(In target) */
	/* don't use step 5. For loudspeaker, the 'max-3dB' step does not exist,
	  and since the digital part of the AFE destination volume is the same
	  for all AFE modes, this analog step cannot be used in the
	  gain calculation */
	AFE_GAIN_OUT_HSGAIN_HS_P6_0DB, /* Max-3             6 dB(In target) */
	AFE_GAIN_OUT_HSGAIN_HS_P6_0DB, /* Max               6 dB(In target) */
	AFE_GAIN_OUT_HSGAIN_HS_OFF
};

static enum aud_analog_gain_value afe_hal_ana_gain =
						AUD_ANALOG_GAIN_MAX_MINUS_12DB;

/* Default register settings for headset offset calibration */
static const u32 agold_afe_reg_cal_prepare[] = {
	0x0076600,		/*AFE_AUDOUTCTRL1 */
	0x28001000, /*AFE_AUDOUTCTRL2 */
	0x0, /*AFE_AUDOUTCTRL3 */
	0x0, /*AFE_AUDINCTRL */
	0x0E000273, /*AFE_GAIN_OUT */
	0x0, /*AFE_GAIN_IN */
	0x0, /*IDI bridge register */
	0x0,
#ifdef CONFIG_SND_SOC_AGOLD_620
	/****** DIGMIC_CONTROL1 Register ******/
	/*	DIGMICON - 0 - off
		DIGMICSEL - 00 - No selection
		SMPLINDM - 00 - Mic1 at Digup2, mic2 at Digup1
		SMPLDM2  - 0 - Mic2 sampled at Rising edge
		SMPLDM1  - 0 - Mic1 sampled at falling edge
	 */
	0x00,
#endif
};

static int agold_afe_perform_hsosc(void);
#endif

static int agold_afe_handle_codec_power(struct snd_soc_codec *codec,
			enum snd_soc_bias_level request_level);

static int agold_afe_get_reg_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo);

static int agold_afe_get_reg_val(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo);

static int agold_afe_set_reg_val(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo);

/* Pointer to agold codec driver local data */
/* The codec local driver data is stored as local pointer.
 * The reason for not using the snd_soc_codec_set_drvdata()
 * and snd_soc_codec_get_drvdata is the need to access the driver local data
 * by accessory identification API where codec instance wont be available.
 */
static struct agold_afe_data *agold_afe_priv;

static /*const*/ u32 agold_afe_reg_cache[] = {
	0x00000000,	/* AFE_PWR */
	/* AUTO_MUTE- Disable
		AUDIO_IN_CLK - Low power
		AUDOUTRATE - 48khz
	*/
	0x00010110,	/* AFE_BCON */
	0x0801000,	/* AFE_AUDOUTCTRL1 */
	0x45010800,	/* AFE_AUDOUTCTRL2 */
	0xFFFFFFFF,	/* AFE_AUDOUTCTRL3 */
	0x6080000,	/* AFE_AUDINCTRL */
	0x0E030273,	/* AFE_GAIN_OUT */
	/* HSOCCALR - 0x10 (bits 31-26) - Best working value */
	0x4000024A,		/* AFE_GAIN_IN */
	0x0,		/* IDI bridge register */
	0x0,
#ifdef CONFIG_SND_SOC_AGOLD_620
	/****** DIGMIC_CONTROL1 Register ******/
	/*	DIGMICON - 0 - off
		DIGMICSEL - 00 - No selection
		SMPLINDM - 00 - Mic1 at Digup2, mic2 at Digup1
		SMPLDM2  - 0 - Mic2 sampled at Rising edge
		SMPLDM1  - 0 - Mic1 sampled at falling edge
	 */
	0x00,
#endif
};

enum agold_afe_power_index {
	AGOLD_AFE_POWER_STATE_OFF,
	AGOLD_AFE_POWER_STATE_NO_HEADSET,
	AGOLD_AFE_POWER_STATE_HEADSET_1,
	AGOLD_AFE_POWER_STATE_HEADSET_2,
	AGOLD_AFE_POWER_STATE_HEADSET_3,
	AGOLD_AFE_POWER_STATE_HEADSET_4
};

/* Power state strings for normal AFE operation */
static const char * const agold_afe_power_states[] = {
	"disable",
	"no_headset",
	"headset",
	"headset_12m",
	"headset_10m",
	"headset_9_6m"
};

/* Power state strings for Direct DAC AFE operation */
static const char * const agold_afe_power_states_direct_dac[] = {
	"disable",
	"no_headset",
	"slpret_headset",
	"slpret_headset_12m",
	"slpret_headset_10m",
	"slpret_headset_9_6m"
};

/* Power state index for a CP frequency */
static int agold_afe_get_headset_power_index(int cp_freq)
{
	int index = -1;

	switch (cp_freq) {
	case 13000:
		index = AGOLD_AFE_POWER_STATE_HEADSET_1;
		break;
	case 12000:
		index = AGOLD_AFE_POWER_STATE_HEADSET_2;
		break;
	case 10400:
		index = AGOLD_AFE_POWER_STATE_HEADSET_3;
		break;
	case 9600:
		index = AGOLD_AFE_POWER_STATE_HEADSET_4;
		break;
	default:
		break;
	}
	return index;
}

static int agold_afe_get_inrate_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	afe_debug("%s :\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.max = 3;
	return 0;
}

/* Read current inrate value*/
static int agold_afe_get_inrate_val(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct agold_afe_data *agold_afe = snd_soc_codec_get_drvdata(codec);
	afe_debug("%s:\n", __func__);

	mutex_lock(&codec->mutex);
	uinfo->value.integer.value[0] = agold_afe->afe_in_samplerate;
	mutex_unlock(&codec->mutex);

	afe_debug("%s: Current AFE IN Sample rate = %ld\n", __func__,
			uinfo->value.integer.value[0]);
	return 0;
}

/* Set afe inrate */
static int agold_afe_set_inrate_val(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct agold_afe_data *agold_afe = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	afe_debug("%s :\n", __func__);

	if (AFE_MAX_SAMPLING_RATE < uinfo->value.integer.value[0]) {
		afe_err("%s: %ld is an invalid inrate request\n", __func__,
			uinfo->value.integer.value[0]);
		return -EINVAL;
	}
	mutex_lock(&codec->mutex);
	agold_afe->afe_in_samplerate = uinfo->value.integer.value[0];
	mutex_unlock(&codec->mutex);
	afe_debug("%s : AFE IN Samplerate set = %d\n", __func__,
			agold_afe->afe_in_samplerate);
	return ret;
}

static int agold_afe_get_cp_frequency_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	afe_debug("%s :\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.max = 13000;
	return 0;
}

/* Read current charge pump frequency */
static int agold_afe_get_cp_frequency_val(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct agold_afe_data *agold_afe = snd_soc_codec_get_drvdata(codec);
	afe_debug("%s:\n", __func__);

	mutex_lock(&codec->mutex);
	uinfo->value.integer.value[0] = agold_afe->afe_pow.cp_freq;
	mutex_unlock(&codec->mutex);

	afe_debug("%s: CP frequency is %ld\n", __func__,
			uinfo->value.integer.value[0]);
	return 0;
}

/* Set the charge pump frequency */
static int agold_afe_set_cp_frequency_val(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct agold_afe_data *agold_afe = snd_soc_codec_get_drvdata(codec);
	int index;
	int ret = 0;
	afe_debug("%s :\n", __func__);

	index = agold_afe_get_headset_power_index(
		uinfo->value.integer.value[0]);

	if (index < 0) {
		afe_err("%s:invalid frequency request\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&codec->mutex);
	agold_afe->afe_pow.cp_freq = uinfo->value.integer.value[0];
	ret = agold_afe_handle_codec_power(codec,
			agold_afe->afe_pow.current_bias);
	mutex_unlock(&codec->mutex);
	afe_debug("%s :CP frequency set is %d\n", __func__,
			agold_afe->afe_pow.cp_freq);
	return ret;
}

static int agold_afe_handle_codec_power(struct snd_soc_codec *codec,
		enum snd_soc_bias_level request_level)
{
	struct agold_afe_data *agold_afe = snd_soc_codec_get_drvdata(codec);
	const char *agold_afe_power_state = NULL;
	int ret = 0;
	int index = -1;

	afe_debug("%s:\n", __func__);

	if (SND_SOC_BIAS_OFF == request_level)
		index = AGOLD_AFE_POWER_STATE_OFF;
	else {
		if (agold_afe->afe_pow.hs_on)
			index = agold_afe_get_headset_power_index(
					agold_afe->afe_pow.cp_freq);
		else
			index = AGOLD_AFE_POWER_STATE_NO_HEADSET;
	}

	if (index < 0) {
		afe_err("%s: Invalid power index\n", __func__);
		return -EINVAL;
	}

	if (agold_afe->afe_pow.direct_dac_on)
		agold_afe_power_state =
			agold_afe_power_states_direct_dac[index];
	else
		agold_afe_power_state =
			agold_afe_power_states[index];

	afe_debug("%s: Selected power id %d, string %s\n",
			__func__, index, agold_afe_power_state);

	if (index == AGOLD_AFE_POWER_STATE_OFF
		|| agold_afe->afe_pow.direct_dac_on)
		ret = idi_set_power_state_by_name(agold_afe->dev,
				(char *)agold_afe_power_state, false);
	else {
		int i;
		ret = idi_set_power_state_by_name(agold_afe->dev,
				(char *)agold_afe_power_state, true);
		/* Clear the AFE internal FIFO after power on */
		for (i = 0; i < agold_afe->fifosize; i++)
			iowrite32(0, agold_afe->fifobase);
	}

	agold_afe->afe_pow.current_bias = request_level;
	if (ret)
		afe_err("%s: Failed to set power %d\n", __func__, ret);

	return ret;
}

/* Retruns pointer to the local codec driver data */
struct agold_afe_data *agold_afe_get_private_data(void)
{
	return agold_afe_priv;
}

/* Stores the supplied pointer as local codec driver data pointer*/
static void agold_afe_set_private_data(struct agold_afe_data *agold_afe_pvt)
{
	agold_afe_priv = agold_afe_pvt;
}

static char *get_reg_name(short int regnum)
{
	switch (regnum) {
	case AGOLD_AFE_PWR:
		return "AGOLD_AFE_PWR";
	case AGOLD_AFE_BCON:
		return "AGOLD_AFE_BCON";
	case AGOLD_AFE_AUDOUTCTRL1:
		return "AGOLD_AFE_AUDOUTCTRL1";
	case AGOLD_AFE_AUDOUTCTRL2:
		return "AGOLD_AFE_AUDOUTCTRL2";
	case AGOLD_AFE_AUDOUTCTRL3:
		return "AGOLD_AFE_AUDOUTCTRL3";
	case AGOLD_AFE_AUDIOINCTRL:
		return "AGOLD_AFE_AUDIOINCTRL";
	case AGOLD_AFE_GAIN_OUT:
		return "AGOLD_AFE_GAIN_OUT";
	case AGOLD_AFE_GAIN_IN:
		return "AGOLD_AFE_GAIN_IN";
	case AGOLD_AFE_AUD2IDICTRL:
		return "AGOLD_AFE_AUD2IDICTRL";
	case AGOLD_AFE_MUTE:
		return "AGOLD_AFE_MUTE";
#ifdef CONFIG_SND_SOC_AGOLD_620
	case AGOLD_AFE_DIGMIC_CONTROL1:
		return "AGOLD_AFE_DIGMIC_CONTROL1";
#endif
	default:
		afe_err("Unknown register/n");
		return "NULL";
	}
}

static int agold_afe_get_reg_addr(unsigned int reg)
{
	afe_debug("Get offset for reg %d\n", reg);

	switch (reg) {

	case AGOLD_AFE_PWR:
		return AG6X0_AFE_POWER;
	case AGOLD_AFE_BCON:
		return AG6X0_AFE_BCON;
	case AGOLD_AFE_AUDOUTCTRL1:
		return AG6X0_AFE_AUDOUTCTRL1;
	case AGOLD_AFE_AUDOUTCTRL2:
		return AG6X0_AFE_AUDOUTCTRL2;
	case AGOLD_AFE_AUDOUTCTRL3:
		return AG6X0_AFE_AUDOUTCTRL3;
	case AGOLD_AFE_AUDIOINCTRL:
		return AG6X0_AFE_AUDIOINCTRL;
	case AGOLD_AFE_GAIN_OUT:
		return AG6X0_AFE_GAIN_OUT;
	case AGOLD_AFE_GAIN_IN:
		return AG6X0_AFE_GAIN_IN;
	case AGOLD_AFE_AUD2IDICTRL:
		return AG6X0_AFE_AUD2IDICTRL;
	case AGOLD_AFE_MUTE:
		return AG6X0_AFE_MUTE;
#ifdef CONFIG_SND_SOC_AGOLD_620
	case AGOLD_AFE_DIGMIC_CONTROL1:
		return AG6X0_AFE_DIGMICCTRL;
#endif
	}
	return -1;
}

static void afe_trigger_work_handler(struct work_struct *work)
{
	u32 reg = 0;

	struct agold_afe_data *afe =
		container_of(work, struct agold_afe_data, afe_trigger_work);
	struct snd_soc_codec *codec = afe->codec;

	afe_debug(" %s cmd %d:\n", __func__, afe->cmd);
	switch (afe->cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		mutex_lock(&codec->mutex);
		reg = snd_soc_read(codec, AGOLD_AFE_BCON);
		/* For capture stream, ensure that AUDINSTART
		is not set in DAC mode */
		if (afe->stream == SNDRV_PCM_STREAM_CAPTURE &&
			!(reg & AFE_BCON_FMR_DIRECT)) {
			afe_debug("%s : Enabling In start bit\n", __func__);
			/* Enable AUDINSTRT */
			reg |= AFE_BCON_AUDINSTRT;
			snd_soc_write(codec, AGOLD_AFE_BCON, reg);
			/* Program the INRATE bits */
			reg &= 0xFFFFFF3F;
			reg |= (afe->afe_in_samplerate <<
					AFE_BCON_AUD_INRATE_POS);
			afe_debug("%s: AFE IN RATE is set to  %d\n", __func__,
			afe->afe_in_samplerate);
			snd_soc_write(codec, AGOLD_AFE_BCON, reg);
		}
		mutex_unlock(&codec->mutex);
		break;

	default:
		break;
	}
}

#ifdef CONFIG_SND_SOC_AGOLD_HSOFC_SUPPORT

/* Read AFE Power register */
static u32 agold_afe_read_power_reg(struct snd_soc_codec *codec)
{
	struct agold_afe_data *agold_afe = snd_soc_codec_get_drvdata(codec);
	u32 reg = 0;
	unsigned int offset = 0;
	offset = agold_afe_get_reg_addr(0);
	mutex_lock(&codec->mutex);
	reg = readl(agold_afe->membase + offset);
	mutex_unlock(&codec->mutex);
	return reg;
}

static int agold_afe_trigger_calibration_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	pr_debug("%s :\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(struct cal_result);
	return 0;
}

static int agold_afe_get_trigger_calibration(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	mutex_lock(&codec->mutex);
	memcpy(&uinfo->value.bytes.data, (void *)&hs_cal_result,
		sizeof(struct cal_result));
	mutex_unlock(&codec->mutex);
	return 0;
}

static int agold_afe_set_trigger_calibration(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo)
{
	int ret;
	int i;
	struct hsosc_ctl *hs_ofs_control;
	struct hsosc_data hs_ofs_data;
	/* If the first byte is non zero get the gains to be calibrated
	   from user space else use the default gain table.
	TODO: Get the gains to be calibrated always from user space. Remove the
			default gain table.*/
	if (uinfo->value.bytes.data[0] != 0) {
		hs_ofs_control = (struct hsosc_ctl *)uinfo->value.bytes.data;
		hs_ofs_data.num_gains = hs_ofs_control->num_gains;
		hs_ofs_data.p_data = (u16 *)hs_ofs_control->data;
		afe_debug("%s: num_gains =  %d\n",
					__func__,
					hs_ofs_data.num_gains);
		memcpy((void *)afe_hal_hs_gain, hs_ofs_data.p_data,
			(hs_ofs_data.num_gains*2));
		for (i = 0; i < hs_ofs_data.num_gains; i++) {
			if (20 < afe_hal_hs_gain[i]) {
				hs_ofs_delta[i].left = 3;
				hs_ofs_delta[i].right = 4;
			} else if (20 >= afe_hal_hs_gain[i]
					&& 11 <= afe_hal_hs_gain[i]) {
				hs_ofs_delta[i].left = 5;
				hs_ofs_delta[i].right = 6;
			} else if (10 > afe_hal_hs_gain[i]
					&& 6 <= afe_hal_hs_gain[i]) {
				hs_ofs_delta[i].left = 5;
				hs_ofs_delta[i].right = 11;
			} else if (6 > afe_hal_hs_gain[i]
					&& 3 <= afe_hal_hs_gain[i]) {
				hs_ofs_delta[i].left = 6;
				hs_ofs_delta[i].right = 15;
			} else if (3 > afe_hal_hs_gain[i]) {
				hs_ofs_delta[i].left = 6;
				hs_ofs_delta[i].right = 18;
			}
			afe_debug("%s: hs_ofs_delta[%d] = {%d, %d}\n", __func__,
				i, hs_ofs_delta[i].left, hs_ofs_delta[i].right);
			afe_debug("%s: afe_hal_hs_gain[%d] = %d\n", __func__,
				i, afe_hal_hs_gain[i]);
		}
	}
	ret = agold_afe_perform_hsosc();
	return ret;
}

static int agold_afe_hs_calibration_event_handler(
	struct snd_soc_codec *codec,
	int event1)
{
	int ret = 0;
	u32 reg = 0;
	int i = 0;
	unsigned int offset = 0;
	struct resource *aferes;
	struct agold_afe_data *agold_afe = agold_afe_get_private_data();
	enum agold_afe_hs_calib_event event =
			(enum agold_afe_hs_calib_event)event1;

	switch (event) {
	case AGOLD_AFE_PREPARE_CALIBRATION:
		agold_afe->afe_pow.hs_on = 1;
		agold_afe->afe_pow.cp_freq = 13000;
		ret = agold_afe_handle_codec_power(codec, SND_SOC_BIAS_PREPARE);
		if (ret < 0) {
			afe_err("failed to enable AFE clks %d\n", ret);
			break;
		}
		reg = snd_soc_read(codec, AGOLD_AFE_BCON);
		/*Power on AFE */
		reg |= AFE_BCON_AFE_PWR;
		snd_soc_write(codec, AGOLD_AFE_BCON, reg);

		/* Power on XBON central biasing */
		reg |= AFE_BCON_XBON;
		snd_soc_write(codec, AGOLD_AFE_BCON, reg);

		/* Enable AUDOUTSTRT */
		reg |= AFE_BCON_AUDOUTSTRT;
		/*DSP always output 48KHz */
		reg &= 0xFFFFFFE3;
		reg |= (0x4 << AFE_BCON_AUD_OUTRATE_POS);
		reg |= AFE_BCON_MODE;
		reg &= ~AFE_BCON_FMR_DIRECT;
		reg |= AFE_BCON_MUTE_DIS;
		reg &= ~AFE_BCON_AUDINCLK;
		snd_soc_write(codec, AGOLD_AFE_BCON, reg);
		/* Read AFE Power register */
		offset = agold_afe_get_reg_addr(0);
		reg = readl(agold_afe->membase + offset);
		snd_soc_write(codec, AGOLD_AFE_PWR, reg);
		/* Initialize AFE registers with default values */
		for (i = 0; i < ARRAY_SIZE(agold_afe_reg_cal_prepare); i++)
			snd_soc_write(codec, i+2, agold_afe_reg_cal_prepare[i]);
		aferes = idi_get_resource_byname(&agold_afe->dev->resources,
					IORESOURCE_MEM, "afe-in-fifo");
		if (!aferes) {
			afe_err("AFE-IN fifo info missing\n");
			kfree(agold_afe);
			/* Initialize AFE registers back to default values */
			for (i = 0; i < ARRAY_SIZE(agold_afe_reg_cache); i++)
				snd_soc_write(codec, i, agold_afe_reg_cache[i]);
			return -EINVAL;
		}
		/* Clear the AFE internal FIFO after bootup*/
		for (i = 0; i < agold_afe->fifosize; i++)
			writel(0, agold_afe->fifobase);

		break;
	case AGOLD_AFE_STOP_CALIBRATION:
		/*disable AUDOUT STRT */
		reg = snd_soc_read(codec, AGOLD_AFE_BCON);
		reg &= ~AFE_BCON_AUDOUTSTRT;
		snd_soc_write(codec, AGOLD_AFE_BCON, reg);
		/* Initialize AFE registers with default values */
		for (i = 0; i < ARRAY_SIZE(agold_afe_reg_cache); i++)
			snd_soc_write(codec, i, agold_afe_reg_cache[i]);
		/* Disable AFE power */
		agold_afe->afe_pow.hs_on = 0;
		ret = agold_afe_handle_codec_power(codec, SND_SOC_BIAS_OFF);
		if (ret < 0) {
			afe_err("failed to disable AFE %d\n", ret);
			break;
		}
		break;
	case AGOLD_AFE_CALIBRATE_LEFT_CH:
		reg = snd_soc_read(codec, AGOLD_AFE_PWR);
		reg &= ~AFE_PWR_HSR;
		reg |= AFE_PWR_HSL;
		snd_soc_write(codec, AGOLD_AFE_PWR, reg);
		break;
	case AGOLD_AFE_CALIBRATE_RIGHT_CH:
		reg = snd_soc_read(codec, AGOLD_AFE_PWR);
		reg &= ~AFE_PWR_HSL;
		reg |= AFE_PWR_HSR;
		snd_soc_write(codec, AGOLD_AFE_PWR, reg);
		break;
	default:
		break;
	}
	return ret;
}

static int agold_afe_hs_calr_set_value(struct snd_soc_codec *codec, u32 value)
{
	u32 reg = 0;
	mutex_lock(&codec->mutex);
	reg = snd_soc_read(codec, AGOLD_AFE_GAIN_IN);
	reg &= ~(AFE_GAIN_IN_HSOCCALR_MASK);
	reg |= value << AFE_GAIN_IN_HSOCCALR_OFFSET;
	snd_soc_write(codec, AGOLD_AFE_GAIN_IN, reg);
	mutex_unlock(&codec->mutex);
	return 0;
}

static int agold_afe_hs_call_set_value(struct snd_soc_codec *codec, u32 value)
{
	u32 reg = 0;
	mutex_lock(&codec->mutex);
	reg = snd_soc_read(codec, AGOLD_AFE_GAIN_IN);
	reg &= ~(AFE_GAIN_IN_HSOCCALL_MASK);
	reg |= value << AFE_GAIN_IN_HSOCCALL_OFFSET;
	snd_soc_write(codec, AGOLD_AFE_GAIN_IN, reg);
	mutex_unlock(&codec->mutex);
	return 0;
}

static int agold_afe_calibrate_hsosc_channel(
	u8 *p_index, /* start index for calibration */
	u8 channel,  /* channel to be calibrated */
	u8 gain_index) /* index of the gain table to be calibrated */
{
	bool found = false; /* Flag to store the result */
	u8 index = *p_index; /* index to be used for starting calibration */
	u16 hsosc_comp_read_count = 0; /* comparator output read count */
	u16 loop_count = 0; /* loop counter */
	u32 reg = 0;
	u32 hssoccalr, hssoccall;
	struct agold_afe_data *agold_afe = agold_afe_get_private_data();
	struct snd_soc_codec *codec = agold_afe->codec;
	reg = snd_soc_read(codec, AGOLD_AFE_AUDOUTCTRL1);
	reg |= 1 << AFE_AUDOUTCTRL1_HSOCSOC_OFFSET;

	snd_soc_write(codec, AGOLD_AFE_AUDOUTCTRL1, reg);
	while (true != found && index < AFE_NOF_HSOSC_TRIM_VALUES) {
		if (RIGHT_CHANNEL == channel) {
			/* Set calibratration index */
			hssoccalr = hs_ofs_conv[index];
			hssoccall = 0;
			agold_afe_hs_calibration_event_handler(codec,
						AGOLD_AFE_CALIBRATE_RIGHT_CH);
		} else {
			/* Set calibratration index */
			hssoccall = hs_ofs_conv[index];
			hssoccalr = 0;
			agold_afe_hs_calibration_event_handler(codec,
						AGOLD_AFE_CALIBRATE_LEFT_CH);
			/* It has been observed that the for the first
			calibration the HSOSC needs around 20ms to settle
			before calibration can start.*/
			if (0 == gain_index)
				mdelay(20);
		}
		agold_afe_hs_calr_set_value(codec, hssoccalr);
		agold_afe_hs_call_set_value(codec, hssoccall);
		hsosc_comp_read_count = 0;
		for (loop_count = 0; loop_count < AFE_HSOSC_READ_COUNT;
				loop_count++) {
			u32 afe_power = 0;
			afe_power = (u32)agold_afe_read_power_reg(codec);
			if (afe_power & AFE_POWER_HSOCCOMP_MASK)
				hsosc_comp_read_count++;
			udelay(2);
		}
		/* Choose the first value for which comparator output
			is always +ve */
		if (AFE_COMPARATOR_VALIDATE_COUNT <= hsosc_comp_read_count)
			found = true;
		else {
			index++;
			if (AFE_NOF_HSOSC_TRIM_VALUES == index)
				/* Terminate the loop at max count */
				found = true;
		}
	}

	if (index < AFE_NOF_HSOSC_TRIM_VALUES) {
		u16 delta_val = 0;
		if (LEFT_CHANNEL == channel)
			hs_ofs_cal_val_step1[afe_hal_ana_gain].left =
							hs_ofs_conv[index];
		else
			hs_ofs_cal_val_step1[afe_hal_ana_gain].right =
							hs_ofs_conv[index];

		/* Step 2 - Add delta after step 1 */
		if (LEFT_CHANNEL == channel) {
			delta_val = hs_ofs_delta[afe_hal_ana_gain].left;
			hs_cal_result.hs_ofs_cal_val[afe_hal_ana_gain].left =
						hs_ofs_conv[index + delta_val];
			hssoccall =
			hs_cal_result.hs_ofs_cal_val[afe_hal_ana_gain].left;
			hssoccalr = 0;
			agold_afe_hs_calr_set_value(codec, hssoccalr);
			agold_afe_hs_call_set_value(codec, hssoccall);
		} else {
			delta_val = hs_ofs_delta[afe_hal_ana_gain].right;
			hs_cal_result.hs_ofs_cal_val[afe_hal_ana_gain].right =
						hs_ofs_conv[index + delta_val];
			hssoccall = 0;
			hssoccalr =
			hs_cal_result.hs_ofs_cal_val[afe_hal_ana_gain].right;
			agold_afe_hs_calr_set_value(codec, hssoccalr);
			agold_afe_hs_call_set_value(codec, hssoccall);
		}
	} else {
		/* Default to an known calibration value if calibration fails
			for some unknown reason */
		if (LEFT_CHANNEL == channel)
			hs_cal_result.hs_ofs_cal_val[afe_hal_ana_gain].left =
				0x20;
		else
			hs_cal_result.hs_ofs_cal_val[afe_hal_ana_gain].right =
				0x10;
	}
	/* Reset the trim values to zero at the end of calibration */
	hssoccall = 0;
	hssoccalr = 0;
	agold_afe_hs_calr_set_value(codec, hssoccalr);
	agold_afe_hs_call_set_value(codec, hssoccall);

	return 0;
}

static int agold_afe_prepare_hsosc(void)
{
	int ret = 0;
	struct agold_afe_data *agold_afe = agold_afe_get_private_data();
	struct snd_soc_codec *codec = agold_afe->codec;
	pr_debug("%s:\n", __func__);
	ret = agold_afe_hs_calibration_event_handler(codec,
				AGOLD_AFE_PREPARE_CALIBRATION);
	return ret;
}

static int agold_afe_stop_hsosc(void)
{
	int ret = 0;
	struct agold_afe_data *agold_afe = agold_afe_get_private_data();
	struct snd_soc_codec *codec = agold_afe->codec;
	pr_debug("%s:\n", __func__);
	ret = agold_afe_hs_calibration_event_handler(codec,
				AGOLD_AFE_STOP_CALIBRATION);
	return ret;
}

static void afe_hal_set_analog_out_gain(
	enum aud_analog_gain_value analog_gain)
{
	u32 reg = 0;
	struct agold_afe_data *agold_afe = agold_afe_get_private_data();
	struct snd_soc_codec *codec = agold_afe->codec;
	pr_debug("%s:\n", __func__);
	reg = snd_soc_read(codec, AGOLD_AFE_GAIN_OUT);
	reg &= ~AFE_HS_GAIN_MASK;
	reg |= afe_hal_hs_gain[analog_gain] << AFE_GAIN_OUT_HS_OFFSET;
	snd_soc_write(codec, AGOLD_AFE_GAIN_OUT, reg);
	afe_hal_ana_gain = analog_gain;
}

static int agold_afe_perform_hsosc(void)
{
	int result;
	u8 gain_index = 0; /* Loop counter for gain steps */
	u8 start_index = 0;
	struct timeval time_us;
	unsigned long time_before_cal_us, time_after_cal_us;
	pr_debug("%s:\n", __func__);
	/* Program AFE registers to start with a known configuration */
	result = agold_afe_prepare_hsosc();
	if (!result) {
		for (gain_index = AUD_ANALOG_GAIN_MAX_MINUS_18DB;
			gain_index < AFE_NOF_EPHSLS_HW_GAIN_VALUES-1;
			gain_index++) {
			start_index = 0;
			afe_hal_set_analog_out_gain(
				(enum aud_analog_gain_value)gain_index);
			udelay(1);
			/* Capture time stamp before calibrating left channel */
			do_gettimeofday(&time_us);
			time_before_cal_us =
				1000000 * time_us.tv_sec + time_us.tv_usec;
			result = agold_afe_calibrate_hsosc_channel(&start_index,
				LEFT_CHANNEL, gain_index);
			do_gettimeofday(&time_us);
			time_after_cal_us =
				(1000000 * time_us.tv_sec + time_us.tv_usec);
			hs_cal_result.timestamp_cal_left[gain_index] =
				(u32)(time_after_cal_us - time_before_cal_us);
		}
		for (gain_index = AUD_ANALOG_GAIN_MAX_MINUS_18DB;
			gain_index < AFE_NOF_EPHSLS_HW_GAIN_VALUES-1;
			gain_index++) {
			start_index = 0;
			afe_hal_set_analog_out_gain(
				(enum aud_analog_gain_value)gain_index);
			udelay(1);
			/* Capture time stamp before calibrating left channel */
			do_gettimeofday(&time_us);
			time_before_cal_us =
				1000000 * time_us.tv_sec + time_us.tv_usec;
			result = agold_afe_calibrate_hsosc_channel(&start_index,
					RIGHT_CHANNEL, gain_index);
			do_gettimeofday(&time_us);
			time_after_cal_us =
				(1000000 * time_us.tv_sec + time_us.tv_usec);
			hs_cal_result.timestamp_cal_right[gain_index] =
				(u32)(time_after_cal_us - time_before_cal_us);
		}
		hs_cal_result.hs_calib_done = 1;
	}
	/* Clean up AFE registers configured for HSOSC */
	result = agold_afe_stop_hsosc();
	if (result) {
		afe_err("%s:Failed to stop HS Offset Calibration\n", __func__);
		return -EINVAL;
	}

	return result;
}
#endif

static const char * const agold_afe_mic_mux_text[] = {
	"AMIC1", "AMIC2"
};

static const char * const agold_afe_dacsel_ep_text[] = {"RDAC", "LDAC" };

static const char * const agold_afe_dacsel_hs_text[] = {"LDAC", "RDAC" };

static const char * const agold_afe_out_sel_text[] = {"NC", "LS"};

static const char * const agold_afe_hsl_out_sel_text[] = {"NC", "HS"};

static const char * const agold_afe_gen_sw_enum_text[] = {"OFF", "ON"};

static const char * const agold_epout_mode_text[] = { "Differential",
	"Single Ended"};

static const char * const agold_ep_robust_mode_text[] = { "Standard",
	"5V Robust" };

static const char * const agold_mic_voltage_text[] = { "1.9", "2.0", "2.1",
	"2.2" };

static const char * const agold_afe_hsps_text[] = { "Enable", "Disable" };

static const char * const agold_afe_hsout_swg[] = { "1.3", "1.5" };

static const char * const agold_afe_hshcm[] = { "Disable", "Enable" };

static const char * const agold_afe_route_text[] = { "DSP", "FM" };

static const char * const agold_afe_hsps_ramp[] = { "50ms", "100ms",
						"200ms", "400ms" };

#ifdef CONFIG_SND_SOC_AGOLD_620
static const char * const agold_afe_dmic_path_text[] = {
	"MIC1_P1_MIC2_P2", "MIC1_P1_MIC2_P1",
	"MIC1_P2_MIC2_P2", "MIC1_P2_MIC2_P1"
};

static const char * const agold_afe_dmic1_sample_phase_text[] = {
	"falling", "rising"
};

static const char * const agold_afe_dmic2_sample_phase_text[] = {
	"rising", "falling"
};
#endif

static const DECLARE_TLV_DB_SCALE(DGAINCR_TLV, -2400, 1200, 1);
static const DECLARE_TLV_DB_SCALE(DGAINFR_TLV, -450, 0, 0);
static const DECLARE_TLV_DB_SCALE(DGAINCL_TLV, -2400, 1200, 1);
static const DECLARE_TLV_DB_SCALE(DGAINFL_TLV, -450, 0, 0);
static const DECLARE_TLV_DB_SCALE(EPGAIN_TLV, -1200, 1200, 1);
static const DECLARE_TLV_DB_SCALE(HSGAIN_TLV, -1200, 600, 1);
static const DECLARE_TLV_DB_SCALE(LSGAIN_TLV, 0, 2400, 1);
static const DECLARE_TLV_DB_SCALE(MICGAIN_TLV, -600, 3900, 1);

static const struct soc_enum agold_afe_ep_mux_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDOUTCTRL1, 3, 2, agold_afe_dacsel_ep_text);

static const struct soc_enum agold_afe_hsl_mux_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDOUTCTRL1, 22, 2, agold_afe_dacsel_hs_text);

static const struct soc_enum agold_afe_route_mux_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_BCON, 19, 2, agold_afe_route_text);

static const struct soc_enum agold_afe_mic_mux_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDIOINCTRL, 5, 2, agold_afe_mic_mux_text);

static const struct soc_enum agold_afe_out_sel_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(agold_afe_out_sel_text),
					agold_afe_out_sel_text);

static const struct soc_enum agold_afe_hsl_out_sel_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(agold_afe_out_sel_text),
					agold_afe_hsl_out_sel_text);

static const struct soc_enum agold_afe_hs_sw_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDOUTCTRL2, 12, 2, agold_afe_gen_sw_enum_text);

#ifdef CONFIG_SND_SOC_AGOLD_620
static const struct soc_enum agold_afe_dmic_path_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_DIGMIC_CONTROL1, 4, 4, agold_afe_dmic_path_text);

static const struct soc_enum agold_afe_dmic1_sample_phase_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_DIGMIC_CONTROL1, 7, 2,
	agold_afe_dmic1_sample_phase_text);

static const struct soc_enum agold_afe_dmic2_sample_phase_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_DIGMIC_CONTROL1, 6, 2,
	agold_afe_dmic2_sample_phase_text);
#endif

static const struct snd_kcontrol_new agold_afe_ep_sel =
SOC_DAPM_ENUM("Route", agold_afe_ep_mux_enum);

static const struct snd_kcontrol_new agold_afe_hsl_sel =
SOC_DAPM_ENUM("Route", agold_afe_hsl_mux_enum);

static const struct snd_kcontrol_new agold_afe_route_sel =
SOC_DAPM_ENUM("Route", agold_afe_route_mux_enum);

static const struct snd_kcontrol_new agold_afe_mic_sel =
SOC_DAPM_ENUM("Route", agold_afe_mic_mux_enum);

static const struct snd_kcontrol_new agold_afe_out_sel =
SOC_DAPM_ENUM_VIRT("Route", agold_afe_out_sel_enum);

static const struct snd_kcontrol_new agold_afe_hsl_out_sel =
SOC_DAPM_ENUM_VIRT("Route", agold_afe_hsl_out_sel_enum);

static const struct snd_kcontrol_new agold_afe_hs_sw =
SOC_DAPM_ENUM("Route", agold_afe_hs_sw_enum);

static const struct snd_kcontrol_new agold_afe_ep_sw =
SOC_DAPM_SINGLE_VIRT("Switch", 1);

static const struct snd_kcontrol_new agold_afe_amic_sw =
SOC_DAPM_SINGLE("Switch", AGOLD_AFE_PWR, 4, 1, 0);

#ifdef CONFIG_SND_SOC_AGOLD_620
static const struct snd_kcontrol_new agold_afe_dmic1_sw =
SOC_DAPM_SINGLE("Switch", AGOLD_AFE_DIGMIC_CONTROL1, 2, 1, 0);

static const struct snd_kcontrol_new agold_afe_dmic2_sw =
SOC_DAPM_SINGLE("Switch", AGOLD_AFE_DIGMIC_CONTROL1, 3, 1, 0);
#endif

static const struct soc_enum agold_afe_ep_out_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDOUTCTRL1, 4, 2, agold_epout_mode_text);

static const struct soc_enum agold_afe_ep_robust_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDOUTCTRL1, 2, 2, agold_ep_robust_mode_text);

static const struct soc_enum agold_afe_mic1_voltage_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDIOINCTRL, 23, 4, agold_mic_voltage_text);

static const struct soc_enum agold_afe_mic2_voltage_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDIOINCTRL, 23, 4, agold_mic_voltage_text);

static const struct soc_enum agold_afe_hsps_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDOUTCTRL1, 25, 2, agold_afe_hsps_text);

static const struct soc_enum agold_afe_hsout_swg_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDOUTCTRL1, 23, 2, agold_afe_hsout_swg);

static const struct soc_enum agold_afe_hshcm_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDOUTCTRL1, 24, 2, agold_afe_hshcm);

static const struct soc_enum agold_afe_hsps_ramp_enum =
SOC_ENUM_SINGLE(AGOLD_AFE_AUDOUTCTRL1, 26, 4, agold_afe_hsps_ramp);

static const struct snd_kcontrol_new agold_afe_snd_controls[] = {
	SOC_SINGLE_TLV("RDAC Coarse Gain", AGOLD_AFE_GAIN_OUT, 2, 6, 0,
		DGAINCR_TLV),
	SOC_SINGLE_TLV("RDAC Fine Gain", AGOLD_AFE_GAIN_OUT, 0, 3, 0,
		DGAINFR_TLV),
	SOC_SINGLE_TLV("LDAC Coarse Gain", AGOLD_AFE_GAIN_OUT, 7, 6, 0,
		DGAINCL_TLV),
	SOC_SINGLE_TLV("LDAC Fine Gain", AGOLD_AFE_GAIN_OUT, 5, 3, 0,
		DGAINFR_TLV),
	SOC_SINGLE_TLV("Earpiece Gain", AGOLD_AFE_GAIN_OUT, 16, 8, 0,
		EPGAIN_TLV),
	SOC_SINGLE_TLV("Headset Gain", AGOLD_AFE_GAIN_OUT, 24, 28, 0,
		HSGAIN_TLV),
	SOC_SINGLE_TLV("LS Gain", AGOLD_AFE_GAIN_OUT, 20, 7, 0, LSGAIN_TLV),
	SOC_SINGLE_TLV("MIC Gain", AGOLD_AFE_GAIN_IN, 0, 31, 0, MICGAIN_TLV),
	SOC_ENUM("Earpiece Output Mode", agold_afe_ep_out_enum),
	SOC_ENUM("Earpiece Robust Mode", agold_afe_ep_robust_enum),
	SOC_ENUM("MIC1 Voltage", agold_afe_mic1_voltage_enum),
	SOC_ENUM("MIC2 Voltage", agold_afe_mic2_voltage_enum),
	SOC_ENUM("HS Pop Supression Enable", agold_afe_hsps_enum),
	SOC_ENUM("HS Output SWG", agold_afe_hsout_swg_enum),
	SOC_ENUM("HS High Current Mode", agold_afe_hshcm_enum),
	SOC_ENUM("HS Pop Suppression Ramp", agold_afe_hsps_ramp_enum),
	SOC_SINGLE("LS ClassD Frequency Divider", AGOLD_AFE_AUDOUTCTRL2, 24,
		   255, 0),
	SOC_SINGLE("HS Left offset calibration", AGOLD_AFE_GAIN_IN, 18,
			 63, 0),
	SOC_SINGLE("HS Right offset calibration", AGOLD_AFE_GAIN_IN, 26,
			 63, 0),
	/* Try to use SND_SOC_BYTES instead */
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "AFE CP Frequency Control",
		.info = agold_afe_get_cp_frequency_info,
		.get = agold_afe_get_cp_frequency_val,
		.put = agold_afe_set_cp_frequency_val,
	},
/* Turn on SND_SOC_AGOLD_AFE_REG_CONTROL to debug the AFE Register.
   Alsa-state should be disabled to avoid dummy values being written
   into register, which causes reboots. */
#ifdef CONFIG_SND_SOC_AGOLD_AFE_REG_CONTROL
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "AFE REG Control",
		.info = agold_afe_get_reg_info,
		.get = agold_afe_get_reg_val,
		.put = agold_afe_set_reg_val,
	},
#endif
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "AFE SET INRATE",
		.info = agold_afe_get_inrate_info,
		.get = agold_afe_get_inrate_val,
		.put = agold_afe_set_inrate_val
	},
/* Turn on SND_SOC_AGOLD_HSOFC_SUPPORT to allow tuning/calibration of HS
   Offset.
   Alsa-state should be disabled to avoid dummy values being written
   into register, which causes reboots. */
#ifdef CONFIG_SND_SOC_AGOLD_HSOFC_SUPPORT
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "AFE HS Offset Calibration Start",
		.info = agold_afe_trigger_calibration_info,
		.get = agold_afe_get_trigger_calibration,
		.put = agold_afe_set_trigger_calibration,
	},
#endif
#ifdef CONFIG_SND_SOC_AGOLD_620
	SOC_ENUM("DMIC Path", agold_afe_dmic_path_enum),
	SOC_ENUM("DMIC1 Sample Phase", agold_afe_dmic1_sample_phase_enum),
	SOC_ENUM("DMIC2 Sample Phase", agold_afe_dmic2_sample_phase_enum)
#endif
};

void afe_writel(struct snd_soc_codec *codec, unsigned int value, void *addr)
{
	afe_debug("@0x%p wr 0x%x\n", addr, value);
	iowrite32(value, addr);
}

/* Update the AGOLD AFE register cache */
static inline void afe_write_register_cache(struct snd_soc_codec *codec,
						unsigned int reg, u32 value)
{
	u32 *cache = codec->reg_cache;

	afe_debug("%s: configure AFE register index %d ba %p, val %X\n",
			__func__, reg, cache, value);

	cache[reg] = value;
}

static int agold_afe_get_reg_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	afe_debug("%s :\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = AGOLD_AFE_NOF_BYTES_PER_REG * AGOLD_AFE_CACHEREGNUM;
	return 0;
}

/* Read out all supported AFE registers */
static int agold_afe_get_reg_val(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo)
{
	u8 reg = 0;
	unsigned int offset = 0;
	u32 reg_val = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct agold_afe_data *agold_afe = snd_soc_codec_get_drvdata(codec);

	for (reg = 0; reg < AGOLD_AFE_CACHEREGNUM; reg++) {
		offset = agold_afe_get_reg_addr(reg);

		mutex_lock(&codec->mutex);
		reg_val = readl(agold_afe->membase + offset);
		mutex_unlock(&codec->mutex);

		afe_debug("%s :reg %d addr %p val 0x%8.8x\n", __func__, reg,
			  agold_afe->membase + offset, reg_val);

		uinfo->value.bytes.data[reg * AGOLD_AFE_NOF_BYTES_PER_REG + 0] =
			(0xFF000000 & reg_val) >> 24;
		uinfo->value.bytes.data[reg * AGOLD_AFE_NOF_BYTES_PER_REG + 1] =
			(0x00FF0000 & reg_val) >> 16;
		uinfo->value.bytes.data[reg * AGOLD_AFE_NOF_BYTES_PER_REG + 2] =
			(0x0000FF00 & reg_val) >> 8;
		uinfo->value.bytes.data[reg * AGOLD_AFE_NOF_BYTES_PER_REG + 3] =
			(0x000000FF & reg_val);
	}
	return 0;
}

/* Set specific AFE register (first byte is the register ID,
   next 4 bytes the 32bit reg values (MSB first)) */
static int agold_afe_set_reg_val(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo)
{
	u8 reg = uinfo->value.bytes.data[0];
	unsigned int offset = 0;
	u32 reg_val = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct agold_afe_data *agold_afe = snd_soc_codec_get_drvdata(codec);

	offset = agold_afe_get_reg_addr(reg);

	afe_debug("%s :reg %d offset %x\n", __func__, reg, offset);

	reg_val = (uinfo->value.bytes.data[4]) |
	       (uinfo->value.bytes.data[3] << 8) |
	       (uinfo->value.bytes.data[2] << 16) |
	       (uinfo->value.bytes.data[1] << 24);

	mutex_lock(&codec->mutex);
	afe_write_register_cache(codec, reg, reg_val);
	afe_writel(codec, reg_val, agold_afe->membase + offset);
	mutex_unlock(&codec->mutex);

	afe_debug("%s :reg_val 0x%8.8x\n", __func__, reg_val);

	return 0;
}

/* Read from AGOLD AFE register cache (no effective HW read) */
static inline unsigned int agold_afe_reg_read_cache(struct snd_soc_codec *codec,
					unsigned int reg)
{
	u32 *cache = codec->reg_cache;

	/* function returns only unsigned int so return 0 if out of range */
	if (reg >= AGOLD_AFE_CACHEREGNUM)
		return 0;

	return cache[reg];
}

/* Sets the requested values to AFE register */
static inline int agold_afe_reg_write(struct snd_soc_codec *codec,
				unsigned int reg, unsigned int value)
{
	unsigned int final_value = value;
	int offset = 0;
	struct agold_afe_data *agold_afe = NULL;
	enum snd_soc_bias_level bias_level;

	agold_afe = (struct agold_afe_data *)snd_soc_codec_get_drvdata(codec);

	if (reg >= AGOLD_AFE_CACHEREGNUM || agold_afe == NULL)
		return -EIO;

#ifdef CONFIG_SND_SOC_AGOLD_ACC_DET_INTERFACE
	if ((reg == AGOLD_AFE_AUDIOINCTRL) || (reg == AGOLD_AFE_BCON)) {
		/* Combine the values requested for accessory identification
		 * and MIC use case */
		agold_afe_calculate_acc_settings(reg, value, &final_value);
	}
#endif
	afe_debug("%s: reg %d val %x\n", __func__, reg, final_value);

	bias_level = agold_afe->afe_pow.current_bias;
	if (bias_level == SND_SOC_BIAS_OFF) {
		afe_debug("Trying to write reg while power domain is off !\n");
		agold_afe_handle_codec_power(codec, SND_SOC_BIAS_STANDBY);
	}

	offset = agold_afe_get_reg_addr(reg);

	if (offset < 0)
		return -1;

	afe_write_register_cache(codec, reg, final_value);
	afe_writel(codec, final_value, agold_afe->membase + offset);

	if (bias_level == SND_SOC_BIAS_OFF)
		agold_afe_handle_codec_power(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int agold_afe_dacl_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	u32 reg = snd_soc_read(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1);

	afe_debug("%s\n", __func__);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe_debug("%s: SND_SOC_DAPM_PRE_PMU\n", __func__);
		reg |= (1 << 16);
		/* NSRON */
		snd_soc_write(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1, reg);
		break;
	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s: SND_SOC_DAPM_POST_PMD\n", __func__);
		reg &= ~(1 << 16);
		/* NSROFF */
		snd_soc_write(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1, reg);
		break;
	}
	return 0;
}

static int agold_afe_dacr_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	u32 reg = snd_soc_read(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1);

	afe_debug("%s\n", __func__);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe_debug("%s: SND_SOC_DAPM_PRE_PMU\n", __func__);
		reg |= (1 << 17);
		/*NSRON*/
		snd_soc_write(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1, reg);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s: SND_SOC_DAPM_POST_PMD\n", __func__);
		reg &= ~(1 << 17);	/*NSROFF */
		snd_soc_write(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1, reg);
		break;
	}
	return 0;
}

static int agold_afe_hs_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct agold_afe_data *agold_afe =
		snd_soc_codec_get_drvdata(w->dapm->codec);

	afe_debug("%s:\n", __func__);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		agold_afe->afe_pow.hs_on = 1;
		agold_afe_handle_codec_power(w->dapm->codec,
			w->dapm->codec->dapm.bias_level);
		break;

	case SND_SOC_DAPM_POST_PMU:
		afe_debug("%s:SND_SOC_DAPM_POST_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMD\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s : SND_SOC_DAPM_POST_PMD\n", __func__);
		agold_afe->afe_pow.hs_on = 0;
		agold_afe_handle_codec_power(w->dapm->codec,
			w->dapm->codec->dapm.bias_level);
		break;
	}

	return 0;
}

static int agold_afe_mic_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
#ifdef CONFIG_SND_SOC_AGOLD_ACC_DET_INTERFACE
	u32 reg;
#endif

	pr_debug("%s: Event : %d\n", __func__, event);

#ifdef CONFIG_SND_SOC_AGOLD_ACC_DET_INTERFACE
	reg = snd_soc_read(w->dapm->codec, AGOLD_AFE_AUDIOINCTRL);
	agold_afe_acc_update_mic_status(event);

	/* Flush out all the register settings to INCTRL */
	return snd_soc_write(w->dapm->codec, AGOLD_AFE_AUDIOINCTRL, reg);
#else
	return 0;
#endif
}

static int agold_afe_mic1_mode_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{

	afe_debug("%s: Event : %d\n", __func__, event);

#ifdef CONFIG_SND_SOC_AGOLD_ACC_DET_INTERFACE
	return agold_afe_acc_update_mic1_mode_status(event);
#else
	return 0;
#endif
}

static u32 agold_afe_get_hsamp_ramp_time(u32 step)
{
	/*Optimum time in mili seconds. Used as default */
	u32 wait_time = 200000;

	switch (step) {
	case 0:
		wait_time = 50000;
		break;
	case 1:
		wait_time = 100000;
		break;
	case 2:
		wait_time = 200000;
		break;
	case 3:
		wait_time = 400000;
		break;
	default:
		break;
	}
	return wait_time;
}

static int agold_afe_ep_amp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	u32 reg = 0;
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe_debug("%s: SND_SOC_DAPM_PRE_PMU\n", __func__);
		reg = snd_soc_read(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1);
		reg |= (1 << 1); /* EPLDO ON */
		snd_soc_write(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1, reg);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s: SND_SOC_DAPM_POST_PMD\n", __func__);
		reg = snd_soc_read(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1);
		reg &= ~(1 << 1); /* EPLDO OFF */
		snd_soc_write(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1, reg);
		break;
	}
	return 0;
}

static int agold_afe_hsr_amp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	u32 reg = 0;
	u32 wait_time;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe_debug("%s: SND_SOC_DAPM_PRE_PMU\n", __func__);
		reg = snd_soc_read(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1);
		reg |= (1 << 18); /* HSLDO ON */
		snd_soc_write(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1, reg);
		break;

	case SND_SOC_DAPM_POST_PMU:
		afe_debug("%s: SND_SOC_DAPM_POST_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		afe_debug("%s: SND_SOC_DAPM_PRE_PMD\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s: SND_SOC_DAPM_POST_PMD\n", __func__);
		/* Wait till ramp down is complete to avoid pops */
		reg = snd_soc_read(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1);
		wait_time = agold_afe_get_hsamp_ramp_time((reg >> 26) & 0x3);
		afe_debug("%s:Waiting for %d us before turing off HS LDO\n",
			__func__, wait_time);
		usleep_range(wait_time, wait_time + 25000);

		reg &= ~(1 << 18); /* HSLDO OFF */
		snd_soc_write(w->dapm->codec, AGOLD_AFE_AUDOUTCTRL1, reg);
		break;
	}
	return 0;
}

static inline int agold_afe_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	int ret = 0;
	struct agold_afe_data *agold_afe = dev_get_drvdata(dev);

	if (!agold_afe) {
		dev_err(dev, "Unable to retrieve agold afe data\n");
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(agold_afe->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	return ret;
}

#ifdef CONFIG_SND_SOC_AGOLD_620
static int agold_afe_dmic_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct agold_afe_data *agold_afe =
		(struct agold_afe_data *)snd_soc_codec_get_drvdata(w->codec);
	int ret;
	afe_debug("%s\n", __func__);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe_debug("%s: SND_SOC_DAPM_PRE_PMU\n", __func__);

		ret = agold_afe_set_pinctrl_state(w->codec->dev,
				agold_afe->pins_default);
		if (ret)
			afe_err("%s: Activating PCL pad failed\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMU:
		afe_debug("%s: SND_SOC_DAPM_POST_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		afe_debug("%s: SND_SOC_DAPM_PRE_PMD\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s: SND_SOC_DAPM_POST_PMD\n", __func__);
		ret = agold_afe_set_pinctrl_state(w->codec->dev,
				agold_afe->pins_sleep);
		if (ret)
			afe_err("%s: Deactivating PCL pad failed\n", __func__);
		break;
	}
	return 0;
}
#endif

static int agold_afe_mux_fm_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	u32 reg = 0;
	struct agold_afe_data *agold_afe = snd_soc_codec_get_drvdata(w->codec);

	afe_debug("%s:\n", __func__);

	reg = snd_soc_read(w->codec, AGOLD_AFE_BCON);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe_debug("%s: SND_SOC_DAPM_PRE_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMU:
		afe_debug("%s: POST_PMU BCON mode %x", __func__, reg);

		if (reg & AFE_BCON_FMR_DIRECT) {
			/* Prepare AFE for DAC mode */
			/* Disable AUDINSTRT */
			reg &= ~AFE_BCON_AUDINSTRT;
			snd_soc_write(w->codec, AGOLD_AFE_BCON, reg);

			agold_afe->afe_pow.direct_dac_on = 1;
			agold_afe->dac_dsp_transit = 1;

			agold_afe_handle_codec_power(w->dapm->codec,
			w->dapm->codec->dapm.bias_level);
		}
		break;

	case SND_SOC_DAPM_PRE_PMD:
		afe_debug("%s: SND_SOC_DAPM_PRE_PMD\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s: SND_SOC_DAPM_POST_PMD\n", __func__);
		agold_afe->afe_pow.direct_dac_on = 0;

		agold_afe_handle_codec_power(w->dapm->codec,
					w->dapm->codec->dapm.bias_level);
		break;

	case SND_SOC_DAPM_POST_REG:
		afe_debug("%s: SND_SOC_DAPM_POST_REG\n", __func__);
		agold_afe->dac_dsp_transit = 1;

		if (reg & AFE_BCON_FMR_DIRECT) {

			/* Prepare AFE for DAC mode */
			/* Disable AUDINSTRT */
			reg &= ~AFE_BCON_AUDINSTRT;
			snd_soc_write(w->codec, AGOLD_AFE_BCON, reg);

			agold_afe->afe_pow.direct_dac_on = 1;

			agold_afe_handle_codec_power(w->dapm->codec,
			agold_afe->afe_pow.current_bias);
		} else {
			agold_afe->afe_pow.direct_dac_on = 0;

			agold_afe_handle_codec_power(w->dapm->codec,
			agold_afe->afe_pow.current_bias);
		}
		break;

	default:
		afe_err("%s: unsupported event %d\n", __func__, event);
		return -EINVAL;
	}

	return 0;
}

/* AGOLD AFE Codec DAPM */
static const struct snd_soc_dapm_widget agold_afe_dapm_widgets[] = {

	/* DAPM Outputs */
	SND_SOC_DAPM_OUTPUT("HSL"),
	SND_SOC_DAPM_OUTPUT("HSR"),
	SND_SOC_DAPM_OUTPUT("EARPIECE"),
	SND_SOC_DAPM_OUTPUT("LOUDSPEAKER"),

	/* Inputs */
	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_INPUT("AMIC2"),
	SND_SOC_DAPM_INPUT("DMIC1"),
	SND_SOC_DAPM_INPUT("DMIC2"),

	SND_SOC_DAPM_PGA_E("Earpiece Amplifier", AGOLD_AFE_PWR, 0, 0,
			NULL, 0, agold_afe_ep_amp_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA("LS Amp Enable", AGOLD_AFE_PWR, 3, 0, NULL, 0),

	SND_SOC_DAPM_PGA_E("Headset Right Amp", AGOLD_AFE_PWR,
			2, 0, NULL, 0, agold_afe_hsr_amp_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA("Headset Left Amp", AGOLD_AFE_PWR,
			1, 0, NULL, 0),

	SND_SOC_DAPM_VIRT_MUX("LS Output Route", SND_SOC_NOPM, 0, 0,
							&agold_afe_out_sel),

	SND_SOC_DAPM_SWITCH("EP Enable", SND_SOC_NOPM, 0, 0,
			&agold_afe_ep_sw),

	SND_SOC_DAPM_MUX_E("HSR Enable Switch", SND_SOC_NOPM, 0, 0,
			&agold_afe_hs_sw, agold_afe_hs_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_VIRT_MUX("HSL Enable Route", SND_SOC_NOPM, 0, 0,
							&agold_afe_hsl_out_sel),


#ifdef CONFIG_SND_SOC_AGOLD_620
	SND_SOC_DAPM_SWITCH("DMIC1 Enable", SND_SOC_NOPM,
			0, 0, &agold_afe_dmic1_sw),
	SND_SOC_DAPM_SWITCH("DMIC2 Enable", SND_SOC_NOPM,
			0, 0, &agold_afe_dmic2_sw),
#endif

	SND_SOC_DAPM_SWITCH("AMIC On", SND_SOC_NOPM, 0, 0, &agold_afe_amic_sw),

	SND_SOC_DAPM_DAC_E("DACL", "Playback Left", AGOLD_AFE_AUDOUTCTRL1,
			9, 0, agold_afe_dacl_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("DACR", "Playback Right", AGOLD_AFE_AUDOUTCTRL1,
			10, 0, agold_afe_dacr_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("EP Output Route", SND_SOC_NOPM, 0, 0,
			&agold_afe_ep_sel),

	SND_SOC_DAPM_MUX("HSL Output Route", SND_SOC_NOPM, 0, 0,
			&agold_afe_hsl_sel),

	SND_SOC_DAPM_MUX_E("FMR AFE output Route", SND_SOC_NOPM, 0, 0,
			&agold_afe_route_sel, agold_afe_mux_fm_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
			SND_SOC_DAPM_POST_REG
			),

	SND_SOC_DAPM_AIF_IN("FM In", "Playback", 0,
			SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("DSP In", "Playback", 0,
			SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_OUT("Audio Capture", "Capture", 0,
			SND_SOC_NOPM, 0, 0),

#ifdef CONFIG_SND_SOC_AGOLD_620
	SND_SOC_DAPM_PGA_E("DMIC On",
			AGOLD_AFE_DIGMIC_CONTROL1, 0, 0, NULL, 0,
			agold_afe_dmic_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
#endif

	SND_SOC_DAPM_MUX("MICIN Sel", SND_SOC_NOPM, 0, 0, &agold_afe_mic_sel),

	SND_SOC_DAPM_SUPPLY("MIC1 BIAS", AGOLD_AFE_AUDIOINCTRL,
			20, 0, agold_afe_mic1_mode_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("MIC2 BIAS", AGOLD_AFE_AUDIOINCTRL,
			22, 0, NULL,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("MIC Enable", AGOLD_AFE_AUDIOINCTRL,
			28, 0, NULL, 0, agold_afe_mic_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD)

};

static const struct snd_soc_dapm_route agold_afe_audio_map[] = {

	/* Speaker Output Map */
	{"FMR AFE output Route", "FM", "FM In"},
	{"FMR AFE output Route", "DSP", "DSP In"},
	{"DACL", NULL, "FMR AFE output Route"},
	{"LS Output Route", "LS", "DACL"},
	{"LS Amp Enable", NULL , "LS Output Route"},
	{"LOUDSPEAKER", NULL, "LS Amp Enable"},

	/* Headset Output Map */
	{"FMR AFE output Route", "FM", "FM In"},
	{"FMR AFE output Route", "DSP", "DSP In"},
	{"DACL", NULL, "FMR AFE output Route"},
	{"DACR", NULL, "FMR AFE output Route"},
	{"HSL Output Route", "LDAC", "DACL"},
	{"HSL Output Route", "RDAC", "DACR"},
	{"Headset Left Amp", NULL, "HSL Output Route"},
	{"HSL Enable Route", "HS", "Headset Left Amp"},
	{"HSL", NULL, "HSL Enable Route"},

	{"HSR", NULL, "Headset Right Amp"},
	{"Headset Right Amp", NULL , "HSR Enable Switch"},
	{"HSR Enable Switch", "ON", "DACR"},
	{"HSR Enable Switch", "ON", "DACL"},

	/* Earpiece Output Map */
	{"EP Output Route", "RDAC", "DACR"},
	{"EP Output Route", "LDAC", "DACL"},
	{"Earpiece Amplifier", "Switch", "EP Output Route"},
	{"EP Enable", "Switch", "Earpiece Amplifier"},
	{"EARPIECE", NULL, "EP Enable"},

	/* Audio Input Map */
	{"AMIC1", "NULL", "MIC1 BIAS"},
	{"AMIC2", "NULL", "MIC2 BIAS"},
	{"MICIN Sel", "AMIC1", "AMIC1"},
	{"MICIN Sel", "AMIC2", "AMIC2"},
	{"MIC Enable", NULL, "MICIN Sel"},
	{"AMIC On", "Switch", "MIC Enable"},
	{"Audio Capture", NULL, "AMIC On"},

#ifdef CONFIG_SND_SOC_AGOLD_620
	{"DMIC1", NULL, "MIC1 BIAS"},
	{"DMIC2", NULL, "MIC1 BIAS"},
	{"DMIC1 Enable", "Switch", "DMIC1"},
	{"DMIC2 Enable", "Switch", "DMIC2"},
	{"DMIC On", NULL, "DMIC1 Enable"},
	{"DMIC On", NULL, "DMIC2 Enable"},
	{"MIC Enable", NULL, "DMIC On"},
	{"Audio Capture", NULL, "MIC Enable"},
#endif
};

static int agold_afe_configure_idi_channel(struct snd_soc_codec *codec)
{
	int ret = -EINVAL;
	struct agold_afe_data *agold_afe;

	agold_afe =
		(struct agold_afe_data *)snd_soc_codec_get_drvdata(codec);

	if (agold_afe) {
		afe_debug("%s: Trying to configure IDI rx channel\n", __func__);
		ret = idi_set_channel_config(agold_afe->dev,
				&agold_afe->rx_config);

		if (ret)
			goto out;

		afe_debug("%s: Trying to configure IDI tx channel\n", __func__);
		ret = idi_set_channel_config(agold_afe->dev,
				&agold_afe->tx_config);
	}

out:
	if (ret < 0)
		afe_err("%s: Failed to configure IDI tx channel\n",
				__func__);

	return ret;
}

static int agold_afe_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct agold_afe_data *agold_afe =
		snd_soc_codec_get_drvdata(dai->codec);
	u32 reg = 0;
	int ret = 0;
	afe_debug("%s: DAC transit %d\n", __func__, agold_afe->dac_dsp_transit);

	mutex_lock(&dai->codec->mutex);
	if (!dai->active || agold_afe->dac_dsp_transit) {
		ret = agold_afe_configure_idi_channel(dai->codec);

		if (agold_afe->dac_dsp_transit == 1)
			agold_afe->dac_dsp_transit = 0;

		if (!ret)
			ret = agold_afe_handle_codec_power(dai->codec,
					SND_SOC_BIAS_PREPARE);
	}

	/* Program BCON MODE bit before AUDOUTSTRT bit */
	reg = snd_soc_read(dai->codec, AGOLD_AFE_BCON);
	reg |= (AFE_BCON_MODE);
	snd_soc_write(dai->codec, AGOLD_AFE_BCON, reg);

	if (!ret &&
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		afe_debug("%s : Enabling IDI HS bit\n",
				__func__);
		if (!(reg & AFE_BCON_FMR_DIRECT)) {
			/* Enable AUDOUTSTRT */
			reg |= AFE_BCON_AUDOUTSTRT;
			snd_soc_write(dai->codec, AGOLD_AFE_BCON, reg);

			/* Enable IDI handshake */
			reg = snd_soc_read(dai->codec,
					AGOLD_AFE_AUD2IDICTRL);
			reg |= AFE_AUD2IDICTRL_IDI_EN |
				AFE_AUD2IDICTRL_CNT_MAX |
				AFE_AUD2IDICTRL_CNT_MIN;
			snd_soc_write(dai->codec,
					AGOLD_AFE_AUD2IDICTRL, reg);
		} else {
		/* While transitioning from DSP -> DAC FM mode disable IDI HS
			and OUT_STRT bits */
			afe_debug("%s : Disabling IDI HS bit\n",
				__func__);
			/*disable AUDOUT START */
			reg &= ~AFE_BCON_AUDOUTSTRT;
			snd_soc_write(dai->codec, AGOLD_AFE_BCON, reg);
			/* disable IDI handshake */
			reg = snd_soc_read(dai->codec,
					AGOLD_AFE_AUD2IDICTRL);
			reg &= ~AFE_AUD2IDICTRL_IDI_EN;
			snd_soc_write(dai->codec,
					AGOLD_AFE_AUD2IDICTRL, reg);
		}
	}
	mutex_unlock(&dai->codec->mutex);

	if (ret)
		afe_err("%s: Failed to startup\n", __func__);

	return ret;
}

static void agold_afe_shutdown(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	u32 reg = 0;
	/* Ensure that AFE stops the input data to DSP. This is required
	 * in case of delayed powered down where bias may still be kept
	 * while the receiver(DSP) is not powered and ready */
	afe_debug("%s\n", __func__);
	mutex_lock(&dai->codec->mutex);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
			!dai->capture_active) {
		afe_debug("%s : Disabling In start bit\n", __func__);
		reg = snd_soc_read(dai->codec, AGOLD_AFE_BCON);
		/* Disable AUDINSTRT */
		reg &= ~AFE_BCON_AUDINSTRT;
		snd_soc_write(dai->codec, AGOLD_AFE_BCON, reg);
	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
			!dai->playback_active) {
		/* Disabling IDI HS bit in shutdown, when switching from
			DAC/DSP mode as BIAS_OFF may not be always called,
			so disabling HS bit, OUT_STRT in shutdown.
			However, the DAC's are still powered on and only
			powered down in the DAPM power down sequence.
			This results in pop noise on LS randomly. Therefore
			disable IDI HS & OUT_STRT only when shutdown is called
			with SND_SOC_BIAS_OFF. */

		/* Pop noise on LS is heard randomly */
		if (SND_SOC_BIAS_OFF == dai->codec->dapm.bias_level) {

			afe_debug("%s : Disabling IDI HS bit\n", __func__);
			reg = snd_soc_read(dai->codec, AGOLD_AFE_BCON);

			/* Disable AUDOUTSTRT */
			reg &= ~AFE_BCON_AUDOUTSTRT;
			snd_soc_write(dai->codec, AGOLD_AFE_BCON, reg);

			/* Disable IDI handshake */
			reg = snd_soc_read(dai->codec, AGOLD_AFE_AUD2IDICTRL);
			reg &= ~AFE_AUD2IDICTRL_IDI_EN;
			snd_soc_write(dai->codec, AGOLD_AFE_AUD2IDICTRL, reg);
		}
	}

	if ((0 == dai->active) &&
		(SND_SOC_BIAS_OFF == dai->codec->dapm.bias_level)) {
		pr_debug("%s: Turning off AFE clocks\n", __func__);
		agold_afe_handle_codec_power(dai->codec,
			SND_SOC_BIAS_OFF);
	}

	mutex_unlock(&dai->codec->mutex);
}

static int agold_afe_trigger(struct snd_pcm_substream *substream,
			int cmd, struct snd_soc_dai *dai)
{
	struct agold_afe_data *agold_afe =
		snd_soc_codec_get_drvdata(dai->codec);

	agold_afe->stream = substream->stream;
	agold_afe->cmd = cmd;
	schedule_work(&agold_afe->afe_trigger_work);

	return 0;
}

static int agold_afe_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	int *audio_native = snd_soc_card_get_drvdata(codec->card);

	u32 reg = 0;

	unsigned int rate, format;
	rate = params_rate(params);
	format = params_format(params);

	afe_debug("%s : rate %d format %d stream %d\n", __func__,
		rate, format, substream->stream);

	mutex_lock(&dai->codec->mutex);
	reg = snd_soc_read(codec, AGOLD_AFE_BCON);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {

		if (*audio_native)
			reg |= BIT(5); /* INSTART */

	}

	/* DSP always output 48kHz */
	reg &= 0xFFFFFFE3;
	reg |= (0x4 << AFE_BCON_AUD_OUTRATE_POS);


	if (*audio_native)
		reg |= AFE_BCON_AFE_PWR | AFE_BCON_AUDOUTSTRT;

	snd_soc_write(codec, AGOLD_AFE_BCON, reg);

	mutex_unlock(&dai->codec->mutex);
	return 0;
}

static int agold_afe_codec_probe(struct snd_soc_codec *codec)
{
	int *audio_native = snd_soc_card_get_drvdata(codec->card);
	struct agold_afe_data *agold_afe = NULL;
	unsigned int i = 0;
	int ret = 0;

	afe_debug("%s\n", __func__);

	codec->dapm.bias_level = SND_SOC_BIAS_OFF;
	codec->dapm.idle_bias_off = 1;

	agold_afe = (struct agold_afe_data *)snd_soc_codec_get_drvdata(codec);
	if (agold_afe != NULL) {
		agold_afe->codec = codec;
		agold_afe_set_private_data(agold_afe);
	} else {
		afe_err("NULL pointer check failed for agold_afe\n");
		return -EINVAL;
	}
	/* Default AFE IN sample rate to 48KHz */
	agold_afe->afe_in_samplerate = 3;
	agold_afe_handle_codec_power(codec, SND_SOC_BIAS_STANDBY);
	if (*audio_native) {
		/* Prepare AFE registers */
		agold_afe->aud2idictrl_cfg = BIT(31);

		agold_afe->power_cfg =
			(0 << 18) |	/* EP built in selftest mode enable */
			(0 << 17) |	/* LS built in selftest mode enable */
			(0 << 16) |	/* MIC built in selftest mode enable */
			(1 << 4) |	/* MIC path on/off */
			(0 << 3) |	/* LS on/off */
			(0 << 1) |	/* HS amplifiers on/off */
			(1 << 0);	/* EP amplifier on/off */

		agold_afe->bcon_cfg =
			(1 << 31) |	/* Switch on central biasing */
			(0 << 19) |	/* No direct path to FMR */
			(1 << 16) |	/* Auto mute disabled */
			(1 << 15) |	/* Power on AFE */
		/*      (0 << 8) |*/	/* Audio in clock Perf mode (4MHz) */
			(1 << 8) |	/* Audio in clock LowPow mode (2MHz) */
		/*      (3 << 6) */	/* Audio in rate 48kHz */
			(0 << 6) |	/* Audio in rate 8kHz */
			(0 << 5) |	/* Audio in Enabled */
			(4 << 2) |	/* Audio out rate 48kHz */
			(0 << 1) |	/* Audio out enabled */
			(0 << 0);	/* AFE on */

		agold_afe->audoutctrl1_cfg =
			(0 << 30) |	/* Trace off */
			(0 << 26) |	/* HS pop suppression ramp delay 50ms */
			(0 << 25) |	/* HS pop suppression disabled */
			(0 << 23) |	/* HS output swing 1.3V */
			(0 << 22) |	/* HS stereo mode */
			(0 << 18) |	/* HS LDO off */
			(1 << 17) |	/* Right noise shaper on */
			(1 << 16) |	/* Lefy noise shaper on */
			(0 << 13) |	/* biDWA on */
			(1 << 11) |	/* Audio out dipher -46dB */
			(1 << 10) |	/* Switch on right DAC */
			(1 << 9) |	/* Switch off left DAC */
			(0 << 4) |	/* EPoutput mode differential */
			(0 << 3) |	/* EP on right channel */
			(0 << 2) |	/* EP 5V Robust Off */
			(1 << 1);	/* EP LDO on */

		agold_afe->audoutctrl2_cfg =
			(40 << 24) |	/* FCTRL = 40  */
			(0 << 23) |	/* 3-level classD modulation */
			(0 << 22) |	/* normal classD clock */
			(8 << 17) |	/* CP clock divided by 8 */
			(0 << 16) |	/* CP jittering disabled */
			(0 << 15) |	/* CP normal mode */
			(1 << 14) |	/* Minimum frequency enabled */
			(1 << 13) |	/* Output voltage not regulated */
			(0 << 12) |	/* CP powered down */
			(0 << 10);	/* CP Slop 0 clock period */

		agold_afe->audoutctrl3_cfg = 0;

		agold_afe->audinctrl_cfg =
			(0 << 29) |	/* ACD off */
			(2 << 27) |	/* MIC LDO operation mode */
			(0 << 26) |	/* VUMIC high impedance mode  */
			(0 << 25) |	/* VMIC high impedance mode */
			(0 << 23) |	/* Microphone supply voltage */
			(0 << 21) |	/* VUMIC off */
			(1 << 20) |	/* VMIC normal talk mode */
			(0 << 18) |	/* Internal LDO regulator 2.5V */
			(0 << 16) |	/* loop test off */
			(0 << 6) |	/* Chooper is switched off */
			(0 << 5);	/* MIC1 input is selected */

		agold_afe->gainout_cfg = 0x40273;

		agold_afe->gainin_cfg =
			(0 << 8) |	/* DIGMICGAIN2 */
			(3 << 5) |	/* Microphone gain -4dB */
			(0 << 0);	/* Microphone gain 0dB */

		/* Setup AFE registers and cache reg */
		snd_soc_write(codec, AGOLD_AFE_AUD2IDICTRL,
				agold_afe->aud2idictrl_cfg);
		mdelay(100);

		snd_soc_write(codec, AGOLD_AFE_PWR, agold_afe->power_cfg);
		snd_soc_write(codec, AGOLD_AFE_BCON, agold_afe->bcon_cfg);
		mdelay(100);

		snd_soc_write(codec, AGOLD_AFE_AUDOUTCTRL1,
				agold_afe->audoutctrl1_cfg);
		snd_soc_write(codec, AGOLD_AFE_AUDOUTCTRL2,
				agold_afe->audoutctrl2_cfg);
		snd_soc_write(codec, AGOLD_AFE_AUDOUTCTRL3,
				agold_afe->audoutctrl3_cfg);
		snd_soc_write(codec, AGOLD_AFE_AUDIOINCTRL,
				agold_afe->audinctrl_cfg);
		snd_soc_write(codec, AGOLD_AFE_GAIN_OUT,
				agold_afe->gainout_cfg);
		snd_soc_write(codec, AGOLD_AFE_GAIN_IN,
				agold_afe->gainin_cfg);
	}

	/* Initialize AFE registers with reset default values */
	for (i = 0; i < ARRAY_SIZE(agold_afe_reg_cache); i++)
		snd_soc_write(codec, i, agold_afe_reg_cache[i]);

	agold_afe_handle_codec_power(codec, SND_SOC_BIAS_OFF);
	return ret;
}

static int agold_afe_set_bias(struct snd_soc_codec *codec,
			enum snd_soc_bias_level level)
{
	u32 reg = snd_soc_read(codec, AGOLD_AFE_BCON);
	int ret = 0;
	int i;
	struct agold_afe_data *agold_afe =
		(struct agold_afe_data *)snd_soc_codec_get_drvdata(codec);

	afe_debug("%s : level %d\n", __func__, level);

	switch (level) {

	case SND_SOC_BIAS_PREPARE:
		/* Enable power only if coming from standby */
		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY) {
			afe_debug("%s: Power on AFE\n", __func__);
			agold_afe_handle_codec_power(codec,
				SND_SOC_BIAS_PREPARE);
			/* Power on AFE */
			reg |= AFE_BCON_AFE_PWR;
			snd_soc_write(codec, AGOLD_AFE_BCON, reg);

			/* Power on XBON central biasing */
			reg |= AFE_BCON_XBON;
			snd_soc_write(codec, AGOLD_AFE_BCON, reg);

			/* Program Default Out gain */
			reg = snd_soc_read(codec, AGOLD_AFE_GAIN_OUT);
			snd_soc_write(codec, AGOLD_AFE_GAIN_OUT, reg);

			/* Program default In gain */
			reg = snd_soc_read(codec, AGOLD_AFE_GAIN_IN);
			snd_soc_write(codec, AGOLD_AFE_GAIN_IN, reg);
		}
		/* Report that codec is prepared even if it is
		   already in PREPARE state */
		codec->dapm.bias_level = level;
		break;

	case SND_SOC_BIAS_ON:
		afe_debug("reg %s reg val %x\n",
			get_reg_name(AGOLD_AFE_BCON),
			snd_soc_read(codec, AGOLD_AFE_BCON));

		afe_debug("reg %s reg val %x\n",
			get_reg_name(AGOLD_AFE_PWR),
			snd_soc_read(codec, AGOLD_AFE_PWR));

		afe_debug("reg %s reg val %x\n",
			get_reg_name(AGOLD_AFE_AUDOUTCTRL1),
			snd_soc_read(codec, AGOLD_AFE_AUDOUTCTRL1));

		afe_debug("reg %s reg val %x\n",
			get_reg_name(AGOLD_AFE_AUDOUTCTRL2),
			snd_soc_read(codec, AGOLD_AFE_AUDOUTCTRL2));

		afe_debug("reg %s reg val %x\n",
			get_reg_name(AGOLD_AFE_AUDOUTCTRL3),
			snd_soc_read(codec, AGOLD_AFE_AUDOUTCTRL3));

		afe_debug("reg %s reg val %x\n",
			get_reg_name(AGOLD_AFE_AUDIOINCTRL),
			snd_soc_read(codec, AGOLD_AFE_AUDIOINCTRL));

		afe_debug("reg %s reg val %x\n",
			get_reg_name(AGOLD_AFE_GAIN_OUT),
			snd_soc_read(codec, AGOLD_AFE_GAIN_OUT));

		afe_debug("reg %s reg val %x\n",
			get_reg_name(AGOLD_AFE_GAIN_IN),
			snd_soc_read(codec, AGOLD_AFE_GAIN_IN));

		codec->dapm.bias_level = level;
		break;

	case SND_SOC_BIAS_OFF:
		/* Turn off if codec is inactive or system shutting down */
		if ((!codec->active || agold_afe->codec_force_shutdown) &&
				codec->dapm.bias_level != SND_SOC_BIAS_OFF) {

			/* Disable AUDOUT STRT */
			reg &= ~AFE_BCON_AUDOUTSTRT;
			snd_soc_write(codec, AGOLD_AFE_BCON, reg);
			/* Disable BCON mode */
			reg &= ~AFE_BCON_MODE;
			snd_soc_write(codec, AGOLD_AFE_BCON, reg);

			/* Disable IDI handshake */
			reg = snd_soc_read(codec, AGOLD_AFE_AUD2IDICTRL);
			reg &= ~AFE_AUD2IDICTRL_IDI_EN;
			snd_soc_write(codec, AGOLD_AFE_AUD2IDICTRL, reg);

			/* Disable AFE power */
			reg = snd_soc_read(codec, AGOLD_AFE_BCON);
			reg &= ~(AFE_BCON_AFE_PWR | AFE_BCON_XBON);
			snd_soc_write(codec, AGOLD_AFE_BCON, reg);

			/* Apply AFE reset to avoid channel swap */
			if (agold_afe->aferst != NULL) {
				reset_control_assert(agold_afe->aferst);
				udelay(10);
				reset_control_deassert(agold_afe->aferst);
				for (i = 0; i < ARRAY_SIZE(agold_afe_reg_cache);
							i++) {
					reg = snd_soc_read(codec, i);
					snd_soc_write(codec, i, reg);
				}
			}

			/* Disable AFE power */
			ret = agold_afe_handle_codec_power(codec, level);

			if (ret < 0) {
				afe_err("failed to disable AFE clks %d\n", ret);
				break;
			}

			/* Report level only if AFE is powered down */
			codec->dapm.bias_level = level;
		}
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_PREPARE) {
			/* If it is turndown sequence, standby only if all
				streams are closed and codec is not active*/
			if (!codec->active)
				codec->dapm.bias_level = level;
		}
		/* In turn ON sequence, put state to standby*/
		else
			codec->dapm.bias_level = level;
		break;

	default:
		break;
	}
	return ret;
}

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT

#define AFE_PM_STATE_D0		0
#define AFE_PM_STATE_D0I0	1
#define AFE_PM_STATE_D0I1	2
#define AFE_PM_STATE_D0I2	3
#define AFE_PM_STATE_D3		4

/* IDI PM states & class */
static struct device_state_pm_state afe_pm_states[] = {
	{ .name = "headset" }, /* D0 */
	{ .name = "no_headset"}, /* D0i0 */
	{ .name = "slpret_headset"}, /* D0i1 */
	{ .name = "slpret_no_headset"}, /* D0i2 */
	{ .name = "disable"}, /* D3 */
};

static int afe_set_pm_state(struct device *_dev,
		struct device_state_pm_state *state)
{
	afe_debug("%s\n", __func__);
	return 0;
}

static struct device_state_pm_state *afe_get_initial_pm_state(
		struct device *_dev)
{
	return &afe_pm_states[AFE_PM_STATE_D3];
}

static struct device_state_pm_ops afe_pm_ops = {
	.set_state = afe_set_pm_state,
	.get_initial_state = afe_get_initial_pm_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(afe);
#endif


static int agold_afe_remove(struct snd_soc_codec *codec)
{
	struct agold_afe_data *agold_afe = snd_soc_codec_get_drvdata(codec);

	afe_debug("%s\n", __func__);

	kfree(agold_afe);
	snd_soc_codec_set_drvdata(codec, NULL);
	agold_afe_set_bias(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int agold_afe_suspend(struct snd_soc_codec *codec)
{
	struct agold_afe_data *agold_afe = snd_soc_codec_get_drvdata(codec);
	unsigned ret = 0;

	afe_debug("%s\n", __func__);
	mutex_lock(&codec->mutex);

	if (!agold_afe->afe_pow.direct_dac_on)
		ret = agold_afe_set_bias(codec, SND_SOC_BIAS_OFF);

	mutex_unlock(&codec->mutex);

	return ret;
}

static int agold_afe_resume(struct snd_soc_codec *codec)
{
	afe_debug("%s\n", __func__);
	agold_afe_configure_idi_channel(codec);

	return 0;
}

static int agold_afe_pre_reboot_handler(struct notifier_block *notifier,
		unsigned long event, void *data)
{
	struct agold_afe_data *agold_afe = agold_afe_get_private_data();

	if (agold_afe) {
		afe_debug("%s: Muting all the audio outputs\n", __func__);
		snd_soc_write(agold_afe->codec, AGOLD_AFE_GAIN_OUT, 0);
	}

	/* Always return success, no need to block if any failure
	 * as the result could be just a pop noise */

	return 0;
}

static struct notifier_block agold_afe_reboot_notifier = {
	.notifier_call = agold_afe_pre_reboot_handler
};

static struct snd_soc_codec_driver soc_codec_dev_afe = {
	.probe = agold_afe_codec_probe,
	.remove = agold_afe_remove,
	.suspend = agold_afe_suspend,
	.resume = agold_afe_resume,
	.read = agold_afe_reg_read_cache,
	.write = agold_afe_reg_write,
	.reg_word_size = sizeof(u32),
	.set_bias_level = agold_afe_set_bias,
	.dapm_widgets = agold_afe_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(agold_afe_dapm_widgets),
	.dapm_routes = agold_afe_audio_map,
	.num_dapm_routes = ARRAY_SIZE(agold_afe_audio_map),
	.reg_cache_size = ARRAY_SIZE(agold_afe_reg_cache),
	.reg_word_size = sizeof(u32),
	.reg_cache_default = agold_afe_reg_cache,
	.num_controls = ARRAY_SIZE(agold_afe_snd_controls),
	.controls = agold_afe_snd_controls
};

static struct snd_soc_dai_ops agold_afe_dai_ops = {
	.hw_params = agold_afe_hw_params,
	.trigger = agold_afe_trigger,
	.shutdown = agold_afe_shutdown,
	.startup = agold_afe_startup,
};

static struct snd_soc_dai_driver agold_afe_dai = {
	.name = "ag620_afe",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AGOLD_AFE_OUT_RATES,
		.formats = AGOLD_AFE_OUT_FORMAT,
		},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AGOLD_AFE_IN_RATES,
		.formats = AGOLD_AFE_IN_FORMAT,
		},
	.ops = &agold_afe_dai_ops,
};

static int agold_afe_device_probe(struct idi_peripheral_device *pdev,
				const struct idi_device_id *id)
{
	int ret = 0;
	struct device_node *np = pdev->device.of_node;
	struct agold_afe_data *agold_afe;
	struct resource *aferes, *dspres, *regres, *fifores;
	struct idi_resource *idi_res = &pdev->resources;

	afe_debug("%s\n", __func__);

	agold_afe = devm_kzalloc(&pdev->device, sizeof(struct agold_afe_data),
			GFP_KERNEL);

	if (agold_afe == NULL) {
		afe_err("%s :Failed allocating driver data", __func__);
		return -ENOMEM;
	}
	agold_afe->dev = pdev;
	agold_afe->codec_force_shutdown = 0;

	/* Use a workqueue to avoid calling a mutex (sleeping function)
	 * in an atomic context. */
	INIT_WORK(&agold_afe->afe_trigger_work, afe_trigger_work_handler);

	/* pm */
	ret = device_state_pm_set_class(&pdev->device,
			pdev->pm_platdata->pm_user_name);
	if (ret) {
		dev_err(&pdev->device,
			"AFE %s failed to set class : %d\n",
			pdev->pm_platdata->pm_user_name, ret);
		return ret;
	}

	regres = idi_get_resource_byname(idi_res, IORESOURCE_MEM, "register");
	if (!regres) {
		afe_err("Register buffer info missing\n");
		return -EINVAL;
	}

	if (!request_mem_region(regres->start, resource_size(regres),
					dev_name(&pdev->device))) {
		return -EBUSY;
	}
	agold_afe->membase = devm_ioremap(&pdev->device, regres->start,
			resource_size(regres));
	afe_debug("Register base @ %p\n", agold_afe->membase);

	/*
	 * Prepare IDI RX channel : ABB->DBB
	 */
	aferes = idi_get_resource_byname(idi_res, IORESOURCE_MEM, "afe-in");
	if (!aferes) {
		afe_err("AFE-in buffer info missing\n");
		return -EINVAL;
	}

	dspres = idi_get_resource_byname(idi_res, IORESOURCE_MEM, "dsp-in");
	if (!dspres) {
		afe_err("DSP-in buffer info missing\n");
		return -EINVAL;
	}

	/* Setup RX config for IDI */
	agold_afe->rx_config.dst_addr = aferes->start; /* ABB_TX */
	agold_afe->rx_config.channel_opts = IDI_PRIMARY_CHANNEL;
	agold_afe->rx_config.tx_or_rx = 0;
	agold_afe->rx_config.base = dspres->start; /* DBB_RX */
	agold_afe->rx_config.cpu_base = NULL;
	agold_afe->rx_config.size = resource_size(dspres);
	agold_afe->rx_config.hw_fifo_size = resource_size(aferes);
	agold_afe->rx_config.priority = IDI_HIGH_PRIORITY;

	/*
	 * Prepare IDI TX channel : DBB->ABB
	 */
	aferes = idi_get_resource_byname(idi_res, IORESOURCE_MEM, "afe-out");
	if (!aferes) {
		afe_err("AFE-out buffer info missing\n");
		return -EINVAL;
	}

	dspres = idi_get_resource_byname(idi_res, IORESOURCE_MEM, "dsp-out");
	if (!dspres) {
		afe_err("DSP-out buffer info missing\n");
		return -EINVAL;
	}

	/* Setup TX config for IDI */
	agold_afe->tx_config.dst_addr = aferes->start; /* ABB_RX */
	agold_afe->tx_config.channel_opts = IDI_PRIMARY_CHANNEL;
	agold_afe->tx_config.tx_or_rx = 1;
	agold_afe->tx_config.base = dspres->start; /* DBB_TX */
	agold_afe->tx_config.cpu_base = NULL;
	agold_afe->tx_config.size = resource_size(dspres);
	agold_afe->tx_config.hw_fifo_size = resource_size(aferes);
	agold_afe->tx_config.priority = IDI_HIGH_PRIORITY;

	/* Get AFE-in FIFO resource */
	fifores = idi_get_resource_byname(idi_res, IORESOURCE_MEM,
			"afe-in-fifo");
	if (!fifores) {
		afe_err("AFE-in FIFO info missing\n");
		return -EINVAL;
	}

	if (!request_mem_region(fifores->start, resource_size(fifores),
					dev_name(&pdev->device))) {
		return -EBUSY;
	}

	agold_afe->fifobase = devm_ioremap(&pdev->device, fifores->start, 4);
	agold_afe->fifosize = resource_size(fifores);

	afe_debug("Fifo base @ 0x%p\n", agold_afe->fifobase);

	/* clock */
	agold_afe->clk = of_clk_get_by_name(np, "clk_afe");
	if (IS_ERR(agold_afe->clk)) {
		afe_debug("AFE clk not found\n");
		agold_afe->clk = NULL;
	}

	/* Reset */
	agold_afe->aferst = reset_control_get(&pdev->device, "aferst");
	if (IS_ERR(agold_afe->aferst)) {
		afe_debug("AFE reset bit not found\n");
		agold_afe->aferst = NULL;
	}

	clk_prepare(agold_afe->clk);

	/* AFE reset toggling */
	if (agold_afe->aferst != NULL) {
		reset_control_assert(agold_afe->aferst);
		udelay(100);
		reset_control_deassert(agold_afe->aferst);
	}

	clk_enable(agold_afe->clk);

	idi_set_channel_config(pdev, &agold_afe->rx_config);
	idi_set_channel_config(pdev, &agold_afe->tx_config);

	/* pinctrl */
	agold_afe->pinctrl = devm_pinctrl_get(&pdev->device);
	if (IS_ERR(agold_afe->pinctrl))
		goto skip_pinctrl;

	agold_afe->pins_default = pinctrl_lookup_state(agold_afe->pinctrl,
						 PINCTRL_STATE_DEFAULT);
	if (IS_ERR(agold_afe->pins_default))
		afe_err("could not get default pinstate\n");

	agold_afe->pins_sleep = pinctrl_lookup_state(agold_afe->pinctrl,
					       PINCTRL_STATE_SLEEP);
	if (IS_ERR(agold_afe->pins_sleep))
		afe_err("could not get sleep pinstate\n");

	agold_afe->pins_inactive = pinctrl_lookup_state(agold_afe->pinctrl,
					       "inactive");
	if (IS_ERR(agold_afe->pins_inactive))
		afe_err("could not get inactive pinstate\n");

skip_pinctrl:
	ret = dev_set_drvdata(&pdev->device, agold_afe);

	agold_afe_set_pinctrl_state(&pdev->device, agold_afe->pins_inactive);

	agold_afe->afe_pow.cp_freq = AGOLD_AFE_CP_DEFAULT_FREQ_KHZ;
	agold_afe->afe_pow.direct_dac_on = 0;
	agold_afe->afe_pow.hs_on = 0;
	agold_afe->afe_pow.current_bias = SND_SOC_BIAS_OFF;

	ret = snd_soc_register_codec(&pdev->device,
				&soc_codec_dev_afe, &agold_afe_dai, 1);

	if (ret < 0) {
		afe_err("unable to register codec driver\n");
		kfree(agold_afe);
		dev_set_drvdata(&pdev->device, NULL);
		return ret;
	}

	/* Register a pre-reboot notifier.
	 * No harm if it fails as reboot notifier does mute during
	 * shutdown to remove any noise */
	register_reboot_notifier(&agold_afe_reboot_notifier);

	return ret;

/* FIXME : handle errors + do kfree */
}

static int agold_afe_drv_remove(struct idi_peripheral_device
		*p_device)
{
	struct agold_afe_data *agold_afe = dev_get_drvdata(&p_device->device);
	afe_debug("%s :\n", __func__);

	unregister_reboot_notifier(&agold_afe_reboot_notifier);

	if (agold_afe) {
		agold_afe_set_pinctrl_state(&p_device->device,
				agold_afe->pins_inactive);
		kfree(agold_afe);
		dev_set_drvdata(&p_device->device, NULL);
	}
	snd_soc_unregister_codec(&p_device->device);
	return 0;
}

static void agold_afe_drv_shutdown(struct idi_peripheral_device *dev)
{
	afe_debug("%s:\n", __func__);
}

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_AFE,
	},

	{ /* end: all zeroes */},
};

static struct idi_peripheral_driver agold_snd_afe_drv = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ag620_afe",
		},
	.p_type = IDI_AFE,
	.id_table = idi_ids,
	.probe = agold_afe_device_probe,
	.remove = agold_afe_drv_remove,
	.shutdown = agold_afe_drv_shutdown,
};

static int __init agold_afe_modinit(void)
{
	int ret = 0;
	afe_debug("%s,\n", __func__);

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	ret = device_state_pm_add_class(&afe_pm_class);
	if (ret) {
		afe_err("Error while adding AFE pm class\n");
		return ret;
	}
#endif

	ret = idi_register_peripheral_driver(&agold_snd_afe_drv);
	if (ret < 0) {
		afe_err("%s : unable to register afe driver\n", __func__);
		return -ENODEV;
	}
	return ret;
}

module_init(agold_afe_modinit);

static void __exit agold_afe_modexit(void)
{
	afe_debug("%s,\n", __func__);
	idi_unregister_peripheral_driver(&agold_snd_afe_drv);
}

module_exit(agold_afe_modexit);

MODULE_DESCRIPTION("AGOLD620 AFE driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL V2");
