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

#define PROP_DSP_USECASE_PREFIX "intel,XGold-usecase"
#define PROP_DSP_USECASE_PARAMETER_PREFIX "-parameter-"
#define USECASE_NAME_MAX	35

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

/* Set function to handle the control to power up/down the DSP*/
static int dsp_audio_use_case_parse_send(
	struct snd_kcontrol *kcontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);
	struct dsp_audio_device *dsp = xgold_pcm->dsp;
	struct device_node *dsp_of_node = dsp->dev->of_node;
	char use_case_name[USECASE_NAME_MAX] = PROP_DSP_USECASE_PREFIX;
	int length = 0;
	int ret = 0;
	u16 *dsp_cmd_ptr = NULL;
	struct dsp_aud_cmd_data dsp_cmd_data;
	int count = 0;

	xgold_debug("%s: Use case ID is %lu\n",
		__func__, kcontrol->private_value);

	/* Use case payload property is of format
	intel,XGold-usecase-parameter-x where x is use case id*/
	snprintf(&use_case_name[strlen(PROP_DSP_USECASE_PREFIX)],
	(USECASE_NAME_MAX - strlen(PROP_DSP_USECASE_PREFIX)), "%s%lu",
	PROP_DSP_USECASE_PARAMETER_PREFIX, kcontrol->private_value);

	if (!of_find_property(dsp_of_node, use_case_name, &length)) {
		xgold_err("%s: Unable find property %s\n",
					__func__, use_case_name);
		return -EINVAL;
	}
	xgold_debug("%s: %s propery length is %d\n", __func__,
				use_case_name, length);

	if (length <= 0) {
		xgold_err("%s: Invalid DSP command payload length %d\n",
				__func__, length);
		return -EINVAL;
	}
	/* Allocate memory for dsp command payload */
	dsp_cmd_ptr = kzalloc(length, GFP_KERNEL);
	if (NULL == dsp_cmd_ptr) {
		xgold_err("%s: Memory allocation for DSP command failed\n",
					__func__);
		return -EINVAL;
	}
	/* Read the DSP command payload for use case from device tree */
	ret = of_property_read_u16_array(dsp_of_node, use_case_name,
			dsp_cmd_ptr, length / 2);
	if (ret != 0) {
		xgold_err("%s: Read %s property failed with error %d\n",
					__func__, use_case_name, ret);
		kfree(dsp_cmd_ptr);
		return -EINVAL;
	}
	/* Convert length to length of U16 type */
	length = length / 2;

	/* Send all the DSP commands for the use case*/
	while (count < length) {
		dsp_cmd_data.command_id = *((u16 *)dsp_cmd_ptr + count);
		dsp_cmd_data.command_len = *((u16 *)dsp_cmd_ptr + count + 1);
		dsp_cmd_data.p_data = ((u16 *)dsp_cmd_ptr + count + 2);
		xgold_debug("%s: command id %d, length %d\n", __func__,
			dsp_cmd_data.command_id, dsp_cmd_data.command_len);
		/* Send the DSP command */
		ret = dsp->p_dsp_common_data->ops->set_controls(
				dsp, DSP_AUDIO_CONTROL_SEND_CMD,
				&dsp_cmd_data);
		if (ret < 0){
			xgold_err("%s: DSP command %d failed, length: %d\n",
						__func__,
						dsp_cmd_data.command_id,
						dsp_cmd_data.command_len);
			}
		/* Increment the count to point to next payload in U16 */
		count += (dsp_cmd_data.command_len +
			sizeof(dsp_cmd_data.command_len) +
			sizeof(dsp_cmd_data.command_id)) / 2;
	}
	kfree(dsp_cmd_ptr);
	return 0;
}

/* Set function to handle the control to power up/down the DSP*/
static int dsp_audio_use_case_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);
	struct dsp_audio_device *dsp = xgold_pcm->dsp;
	int ret = 0;
	int control;

	if (dsp) {
		if (ucontrol->value.integer.value[0])
			control = 1;
		else
			control = 0;

		if (kcontrol->private_value == PCM_AUDIO_PLAYBACK){
			if (dsp->p_dsp_common_data->control_priv.pcm_audio_playback !=
					control) {
				xgold_debug("%s: Trying to set %d\n",
						 __func__, control);
				ret = dsp_audio_use_case_parse_send(kcontrol);
				dsp->p_dsp_common_data->control_priv.pcm_audio_playback =
					control;
				}
		}
		else if (kcontrol->private_value == PCM_AUDIO_RECORD){
			if (dsp->p_dsp_common_data->control_priv.pcm_audio_record !=
					control) {
				xgold_debug("%s: Trying to set %d\n",
						 __func__, control);
				ret = dsp_audio_use_case_parse_send(kcontrol);
				dsp->p_dsp_common_data->control_priv.pcm_audio_record =
					control;
			}
		}
		if (ret < 0)
			xgold_err("%s: Failed to set usecase %d\n",
				__func__, ret);
	} else
		ret = -ENODEV;

	return ret;
}

/* Get function for DSP inbuilt use case control */
static int dsp_audio_use_case_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);
	struct dsp_audio_device *dsp = xgold_pcm->dsp;
	int ret = 0;

	if (dsp) {
		if (kcontrol->private_value == PCM_AUDIO_PLAYBACK){
			ucontrol->value.integer.value[0] =
				dsp->p_dsp_common_data->control_priv.pcm_audio_playback;
		} else if (kcontrol->private_value == PCM_AUDIO_RECORD){
			ucontrol->value.integer.value[0] =
				dsp->p_dsp_common_data->control_priv.pcm_audio_record;
		}
	} else
		ret = -ENODEV;

	return ret;
}

/* Info function for the controls */
static int dsp_audio_use_case_info(
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
/* Turn on SND_XGOLD_DSP_SEND_CMD to enable debugging/calibration options.
   This function allows User to send commands to DSP.
   Alsa-state should be disabled to avoid dummy values being written
   into register, which causes reboots. */
#ifdef CONFIG_SND_XGOLD_DSP_SEND_CMD
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "DSP Audio send cmd",
		.info = dsp_audio_control_info,
		.get = dsp_audio_send_cmd_control_get,
		.put = dsp_audio_send_cmd_control_set,
	},
#endif
/* Turn on SND_XGOLD_DSP_RW_SHARED_MEM to enable debugging/calibration options.
   This function allows user to read and write into the DSP shared memory.
   Alsa-state should be disabled to avoid dummy values being written
   into register, which causes reboots. */
#ifdef CONFIG_SND_XGOLD_DSP_RW_SHARED_MEM
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "DSP Audio RW SHM",
		.info = dsp_audio_control_info,
		.get = dsp_audio_rw_shm_control_get,
		.put = dsp_audio_rw_shm_control_set,
	},
#endif
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "DSP Audio Power",
		.info = dsp_audio_power_control_info,
		.get = dsp_audio_power_control_get,
		.put = dsp_audio_power_control_set,
	},
};

static struct snd_kcontrol_new dsp_audio_usecase_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = NULL,
	.info = dsp_audio_use_case_info,
	.get = dsp_audio_use_case_get,
	.put = dsp_audio_use_case_set,
};

static int dsp_audio_builtin_usecase_of_parse(
	struct snd_soc_platform *platform)
{
	struct device_node *np = platform->dev->of_node;
	const char *name = NULL;
	int ret = 0;
	struct device_node *dsp_of_node;
	int i;
	int count;
	int length;
	struct property *prop;

	dsp_of_node = of_parse_phandle(np, "intel,dsp", 0);
	if (!dsp_of_node) {
		xgold_err("%s: Unable to get dsp node\n", __func__);
		return -EINVAL;
	}
	prop = of_find_property(dsp_of_node, PROP_DSP_USECASE_PREFIX, &length);

	if (NULL == prop) {
		xgold_err("%s: Unable to find property %s\n",
			__func__, PROP_DSP_USECASE_PREFIX);
		return -EINVAL;
	}

	count = of_property_count_strings(dsp_of_node,
			PROP_DSP_USECASE_PREFIX);

	xgold_debug("%s: Number of usecases is %d\n", __func__, count);

	if (count <= 0)
		return 0;

	for (i = 1; i <= count; i++) {

		name = of_prop_next_string(prop, name);
		if (NULL == name)
			break;
		xgold_debug("%s: control name is %s\n",
			__func__, name);
		dsp_audio_usecase_control.name = name;
		dsp_audio_usecase_control.private_value = i;

		ret = snd_soc_add_platform_controls(platform,
			&dsp_audio_usecase_control, 1);
		if (ret != 0)
			xgold_err("dsp audio controls reg failed %d\n", ret);
	}
	return 0;
}

/* Initialize the dsp audio platform that adds soc platform controls */
int dsp_audio_platform_init(struct snd_soc_platform *platform)
{
	int ret;

	xgold_debug("in %s\n", __func__);

	ret = snd_soc_add_platform_controls(platform, dsp_audio_controls,
			ARRAY_SIZE(dsp_audio_controls));
	if (ret != 0)
		xgold_debug("dsp audio controls reg failed %d\n", ret);

	ret = dsp_audio_builtin_usecase_of_parse(platform);
	if (ret != 0)
		xgold_debug("dsp audio usecase parsing failed %d\n", ret);
	return 0;
}
