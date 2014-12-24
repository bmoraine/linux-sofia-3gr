/*
 * Component: XGOLD Audio DSP Driver
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

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include "dsp_audio_platform.h"

#define	xgold_err(fmt, arg...) \
		pr_err("snd: plf: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: plf: "fmt, ##arg)

/* Info function for the controls */
int dsp_audio_control_info(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = 512; /* Max no of bytes used for the byte control */
	return 0;
}

/* Function to read data from the Shared Memory */
static int dsp_audio_rw_shm_control_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *pcm = snd_soc_dai_get_drvdata(cpu_dai);
	struct dsp_audio_device *dsp = pcm->dsp;
	int ret = 0;

	xgold_debug("%s\n", __func__);

	if (dsp) {
		struct dsp_rw_shm_header *p_rw_shm_header =
			(struct dsp_rw_shm_header *)ucontrol->value.bytes.data;

		struct dsp_rw_shm_data rw_shm_data;
		rw_shm_data.word_offset = p_rw_shm_header->word_offset;
		rw_shm_data.len_in_bytes = p_rw_shm_header->len_in_bytes;
		rw_shm_data.p_data = (U16 *) p_rw_shm_header->data;
		ret = dsp->p_dsp_common_data->ops->set_controls(
				dsp, DSP_AUDIO_CONTROL_READ_SHM,
				&rw_shm_data);
	} else
		ret = -ENODEV;

	return 0;
}

/* Function to write to the shared memory */
static int dsp_audio_rw_shm_control_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *pcm = snd_soc_dai_get_drvdata(cpu_dai);
	struct dsp_audio_device *dsp = pcm->dsp;
	int ret = 0;

	pr_info("%s: dsp %p\n", __func__, dsp);
	xgold_debug("%s\n", __func__);
	if (dsp) {
		struct dsp_rw_shm_header *p_rw_shm_header =
			(struct dsp_rw_shm_header *)ucontrol->value.bytes.data;

		struct dsp_rw_shm_data rw_shm_data;
		rw_shm_data.word_offset = p_rw_shm_header->word_offset;
		rw_shm_data.len_in_bytes = p_rw_shm_header->len_in_bytes;
		rw_shm_data.p_data = (U16 *) p_rw_shm_header->data;

		ret = dsp->p_dsp_common_data->ops->set_controls(
				dsp, DSP_AUDIO_CONTROL_WRITE_SHM,
				&rw_shm_data);
	} else
		ret = -ENODEV;

	return ret;
}

/* Get function for the control DSP_AUDIO_CONTROL_SEND_CMD */
static int dsp_audio_send_cmd_control_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	xgold_debug("%s\n", __func__);
	return 0;
}

/* Set function to handle the control to send the DSP command */
static int dsp_audio_send_cmd_control_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *pcm = snd_soc_dai_get_drvdata(cpu_dai);
	struct dsp_audio_device *dsp = pcm->dsp;
	int ret = 0;

	xgold_debug("%s\n", __func__);
	if (dsp) {
		struct dsp_aud_cmd_header *p_cmd_header =
			(struct dsp_aud_cmd_header *)ucontrol->value.bytes.data;

		struct dsp_aud_cmd_data cmd_data;
		/* fill the DSP command structure */
		cmd_data.command_id = p_cmd_header->command_id;
		cmd_data.command_len = p_cmd_header->command_len;
		cmd_data.p_data = (U16 *) p_cmd_header->data;

		ret = dsp->p_dsp_common_data->ops->set_controls(
				dsp, DSP_AUDIO_CONTROL_SEND_CMD,
				&cmd_data);
	} else {
		ret = -ENODEV;
	}

	return ret;
}

/* Get function for the control DSP Audio Power */
static int dsp_audio_power_control_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);
	struct dsp_audio_device *dsp = xgold_pcm->dsp;
	int ret = 0;

	if (dsp)
		ucontrol->value.integer.value[0] =
			dsp->p_dsp_common_data->control_priv.dsp_keep_powered;
	else
		ret = -ENODEV;

	return ret;
}

/* Set function to handle the control to power up/down the DSP*/
static int dsp_audio_power_control_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);
	struct dsp_audio_device *dsp = xgold_pcm->dsp;
	int ret = 0;
	int power_control;

	if (dsp) {
		if (ucontrol->value.integer.value[0])
			power_control = 1;
		else
			power_control = 0;

		if (dsp->p_dsp_common_data->control_priv.dsp_keep_powered !=
				power_control) {
			xgold_debug("%s: Trying to set %d\n",
					 __func__, power_control);
			dsp->p_dsp_common_data->ops->set_controls(dsp,
					DSP_AUDIO_POWER_REQ,
					(void *)&power_control);
			dsp->p_dsp_common_data->control_priv.dsp_keep_powered =
				power_control;
		}
	} else
		ret = -ENODEV;

	return ret;
}

/* Info function for the controls */
int dsp_audio_power_control_info(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}
/* Soc platform controls */
static const struct snd_kcontrol_new dsp_audio_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "DSP Audio send cmd",
		.info = dsp_audio_control_info,
		.get = dsp_audio_send_cmd_control_get,
		.put = dsp_audio_send_cmd_control_set,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "DSP Audio RW SHM",
		.info = dsp_audio_control_info,
		.get = dsp_audio_rw_shm_control_get,
		.put = dsp_audio_rw_shm_control_set,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "DSP Audio Power",
		.info = dsp_audio_power_control_info,
		.get = dsp_audio_power_control_get,
		.put = dsp_audio_power_control_set,
	},
};

/* Initialize the dsp audio platform that adds soc platform controls */
int dsp_audio_platform_init(struct snd_soc_platform *platform)
{
	int ret;

	xgold_debug("in %s\n", __func__);

	ret = snd_soc_add_platform_controls(platform, dsp_audio_controls,
			ARRAY_SIZE(dsp_audio_controls));
	if (ret != 0)
		xgold_debug("dsp audio controls reg failed %d", ret);
	return 0;
}
