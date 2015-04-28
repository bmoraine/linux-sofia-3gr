/*
 * Component: XGOLD speech probe driver
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

#include "dsp_audio_platform.h"
#include "aud_lib_dsp_internal.h"
#include "dsp_audio_hal_internal.h"
#include "xgold_pcm.h"

/* index of the first speech probe device in the array xgold_dai */
#define XGOLD_SPEECH_PROBE_DEVICE_OFSET 4

#define XGOLD_SPEECH_PROBE_RATES (SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000)
#define XGOLD_SPEECH_PROBE_FORMAT SNDRV_PCM_FMTBIT_S16_LE

#define XGOLD_MAX_SPEECH_PROBE_RING_SIZE (4 * 1024)

#define XGOLD_MIN_SPEECH_PROBE_SELECT 1
#define XGOLD_MAX_SPEECH_PROBE_SELECT 21

#define	xgold_err(fmt, arg...) \
		pr_err("snd: speech: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: speech: "fmt, ##arg)

/* convert from device ID in xgold_speech_probe_controls to probe point ID
	enum xgold_speech_probe_point_id */
#define	PROBE_DEVICE_TO_PROBE_POINT_ID(a) \
	(a - XGOLD_SPEECH_PROBE_DEVICE_OFSET)


#define	PROBE_POINT_TO_STREAM_PLAY(a)	(2 * a)
#define	PROBE_POINT_TO_STREAM_REC(a)	(2 * a + 1)

enum xgold_speech_probe_point_id {
	PROBE_POINT_A,
	PROBE_POINT_B,
	PROBE_POINT_C,
	PROBE_POINT_D,
#ifdef CONFIG_INCREASE_PCM_DEVICE
	PROBE_POINT_E,
	PROBE_POINT_F,
#endif
	PROBE_POINT_END
};

enum xgold_speech_probe_lisr_id {
	PROBE_A_STREAM_PLAY,
	PROBE_A_STREAM_REC,
	PROBE_B_STREAM_PLAY,
	PROBE_B_STREAM_REC,
	PROBE_C_STREAM_PLAY,
	PROBE_C_STREAM_REC,
	PROBE_D_STREAM_PLAY,
	PROBE_D_STREAM_REC,
#ifdef CONFIG_INCREASE_PCM_DEVICE
	PROBE_E_STREAM_PLAY,
	PROBE_E_STREAM_REC,
	PROBE_F_STREAM_PLAY,
	PROBE_F_STREAM_REC,
#endif
	PROBE_POINT_STREAM_END
};

struct xgold_speech_probes_status {
	bool active;
	unsigned int speech_probe_sel;
};

struct xgold_audio_speech_probe {
	struct device *dev;
	struct dsp_audio_device *dsp;
	struct xgold_speech_probes_status
		sp_status[PROBE_POINT_STREAM_END];
};

struct xgold_audio_speech_runtime_data {
	struct xgold_audio_speech_probe *speech_probe;
	struct snd_pcm_substream *stream;
	enum xgold_speech_probe_point_id probe_point_id;
	enum xgold_speech_probe_lisr_id probe_lisr_id;
	unsigned short *hwptr;
	unsigned int hwptr_done;
	unsigned int periods;
	unsigned int period_size_bytes;
};

static const struct snd_pcm_hardware
	xgold_speech_probe_extract_cfg = {
		.info = SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_SYNC_START|
			SNDRV_PCM_INFO_RESUME,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.rate_min = 8000,
		.rate_max = 16000,
		.channels_min = 1,
		.channels_max = 1,
		.buffer_bytes_max = 10240,
		.period_bytes_min = 160,
		.period_bytes_max = 640,
		.periods_min = 1,
		.periods_max = 16,
};

static int xgold_speech_probe_get_probe_point_id(
			struct snd_pcm_substream *substream)
{
	int io_probe_point;
	char *substream_id = substream->pcm->id;

	if (strstr(substream_id, "SPEECH_PROBE_A"))
		io_probe_point = PROBE_POINT_A;
	else if (strstr(substream_id, "SPEECH_PROBE_B"))
		io_probe_point = PROBE_POINT_B;
	else if (strstr(substream_id, "SPEECH_PROBE_C"))
		io_probe_point = PROBE_POINT_C;
	else if (strstr(substream_id, "SPEECH_PROBE_D"))
		io_probe_point = PROBE_POINT_D;
#ifdef CONFIG_INCREASE_PCM_DEVICE
	else if (strstr(substream_id, "SPEECH_PROBE_E"))
		io_probe_point = PROBE_POINT_E;
	else if (strstr(substream_id, "SPEECH_PROBE_F"))
		io_probe_point = PROBE_POINT_F;
#endif
	else {
		io_probe_point = PROBE_POINT_END;
		xgold_err("%s - unknown probe point ID!!\n", __func__);
	}
	return io_probe_point;
}

static int xgold_speech_probe_get_record_lisr_id(
	int probe_point_id)
{
	int probe_lisr_id;

	switch (probe_point_id) {
	case PROBE_POINT_A:
		probe_lisr_id = PROBE_A_STREAM_REC;
		break;
	case PROBE_POINT_B:
		probe_lisr_id = PROBE_B_STREAM_REC;
		break;
	case PROBE_POINT_C:
		probe_lisr_id = PROBE_C_STREAM_REC;
		break;
	case PROBE_POINT_D:
		probe_lisr_id = PROBE_D_STREAM_REC;
		break;
#ifdef CONFIG_INCREASE_PCM_DEVICE
	case PROBE_POINT_E:
		probe_lisr_id = PROBE_E_STREAM_REC;
		break;
	case PROBE_POINT_F:
		probe_lisr_id = PROBE_F_STREAM_REC;
		break;
#endif /* CONFIG_INCREASE_PCM_DEVICE */
	default:
		probe_lisr_id = PROBE_POINT_STREAM_END;
		break;
	}
	return probe_lisr_id;
}

static int xgold_speech_probe_get_playback_lisr_id(
	int probe_point_id)
{
	int probe_lisr_id;

	switch (probe_point_id) {
	case PROBE_POINT_A:
		probe_lisr_id = PROBE_A_STREAM_PLAY;
		break;
	case PROBE_POINT_B:
		probe_lisr_id = PROBE_B_STREAM_PLAY;
		break;
	case PROBE_POINT_C:
		probe_lisr_id = PROBE_C_STREAM_PLAY;
		break;
	case PROBE_POINT_D:
		probe_lisr_id = PROBE_D_STREAM_PLAY;
		break;
#ifdef CONFIG_INCREASE_PCM_DEVICE
	case PROBE_POINT_E:
		probe_lisr_id = PROBE_E_STREAM_PLAY;
		break;
	case PROBE_POINT_F:
		probe_lisr_id = PROBE_F_STREAM_PLAY;
		break;
#endif /* CONFIG_INCREASE_PCM_DEVICE */
	default:
		probe_lisr_id = PROBE_POINT_STREAM_END;
		break;
	}
	return probe_lisr_id;
}

static int xgold_speech_probe_get_lisr_cb_id(
	int probe_lisr_id)
{
	int lisr_cb_id;

	switch (probe_lisr_id) {
	case PROBE_A_STREAM_REC:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_POINT_A;
		break;
	case PROBE_B_STREAM_REC:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_POINT_B;
		break;
	case PROBE_C_STREAM_REC:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_POINT_C;
		break;
	case PROBE_D_STREAM_REC:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_POINT_D;
		break;
#ifdef CONFIG_INCREASE_PCM_DEVICE
	case PROBE_E_STREAM_REC:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_POINT_E;
		break;
	case PROBE_F_STREAM_REC:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_POINT_F;
		break;
#endif /* CONFIG_INCREASE_PCM_DEVICE */
	case PROBE_A_STREAM_PLAY:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_INJECT_A;
		break;
	case PROBE_B_STREAM_PLAY:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_INJECT_B;
		break;
	case PROBE_C_STREAM_PLAY:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_INJECT_C;
		break;
	case PROBE_D_STREAM_PLAY:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_INJECT_D;
		break;
#ifdef CONFIG_INCREASE_PCM_DEVICE
	case PROBE_E_STREAM_PLAY:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_INJECT_E;
		break;
	case PROBE_F_STREAM_PLAY:
		lisr_cb_id = DSP_LISR_CB_SPEECH_IO_INJECT_F;
		break;
#endif /* CONFIG_INCREASE_PCM_DEVICE */
	default:
		lisr_cb_id = DSP_LISR_CB_END;
		break;
	}
	return lisr_cb_id;

}

static U16 xgold_speech_probe_get_shm_offset(
	int probe_point_id,
	struct dsp_audio_device *dsp)
{
	U16 shmem_offset;

	switch (probe_point_id) {
	case PROBE_POINT_A:
		shmem_offset = dsp->p_dsp_common_data->
			buf_sm_speech_probe_a_offset;
		break;
	case PROBE_POINT_B:
		shmem_offset = dsp->p_dsp_common_data->
			buf_sm_speech_probe_b_offset;
		break;
	case PROBE_POINT_C:
		shmem_offset = dsp->p_dsp_common_data->
			buf_sm_speech_probe_c_offset;
		break;
	case PROBE_POINT_D:
		shmem_offset = dsp->p_dsp_common_data->
			buf_sm_speech_probe_d_offset;
		break;
#ifdef CONFIG_INCREASE_PCM_DEVICE
	case PROBE_POINT_E:
		shmem_offset = dsp->p_dsp_common_data->
			buf_sm_speech_probe_e_offset;
		break;
	case PROBE_POINT_F:
		shmem_offset = dsp->p_dsp_common_data->
			buf_sm_speech_probe_f_offset;
		break;
#endif /* CONFIG_INCREASE_PCM_DEVICE */
	default:
		xgold_err("No offset associated\n");
		shmem_offset = 0;
		break;
	}
	return shmem_offset;
}

void xgold_speech_probe_interrupt_handler(void *dev)
{
	unsigned int length = 0;
	struct xgold_audio_speech_runtime_data *xrtd =
		(struct xgold_audio_speech_runtime_data *)dev;

	struct dsp_audio_device *dsp;
	struct dsp_rw_shm_data rw_shm_data;
	enum dsp_audio_controls dsp_control;
	static int debug_log_cnt = 10;

	if (!xrtd) {
		xgold_err("Runtime data is NULL (line %d)\n", __LINE__);
		return;
	}

	if (!xrtd->speech_probe) {
		xgold_err("Runtime data is NULL (line %d)\n", __LINE__);
		return;
	}

	if (!xrtd->speech_probe->dsp) {
		xgold_err("Runtime data is NULL (line %d)\n", __LINE__);
		return;
	}

	dsp = xrtd->speech_probe->dsp;

	if (!xrtd->stream) {
		/* It happens stream is NULL, when inject + extract is started
		   for the same probe point.
		   No degrade found from this, so just return. */
		xgold_debug("Runtime data is NULL (line %d) - probe_lisr_id: %d\n",
			__LINE__, xrtd->probe_lisr_id);
		return;
	}

	if (!xrtd->stream->runtime) {
		xgold_err("Runtime data is NULL (line %d)\n", __LINE__);
		return;
	}
	if (!xrtd->stream->runtime->dma_area) {
		xgold_err("Runtime data is NULL (line %d)\n", __LINE__);
		return;
	}

	xrtd->hwptr =
		(unsigned short *)(xrtd->stream->runtime->dma_area +
		xrtd->period_size_bytes * xrtd->hwptr_done);

	if (!xrtd->hwptr) {
		xgold_err("Runtime data is NULL (line %d)\n", __LINE__);
		return;
	}
	rw_shm_data.p_data = xrtd->hwptr;

	rw_shm_data.word_offset =
		xgold_speech_probe_get_shm_offset(xrtd->probe_point_id, dsp);

	length = xrtd->stream->runtime->period_size *
		xrtd->stream->runtime->channels;
	rw_shm_data.len_in_bytes = length * 2;

	if (debug_log_cnt == 10) {
		xgold_debug("enter - probe_lisr_id: %d\n", xrtd->probe_lisr_id);
		xgold_debug("word_offset: %d, len_in_bytes: %d, p_data: 0X%p\n",
			rw_shm_data.word_offset, rw_shm_data.len_in_bytes,
			rw_shm_data.p_data);
		debug_log_cnt = 0;
	} else
		debug_log_cnt++;

	if (xrtd->stream->stream == SNDRV_PCM_STREAM_CAPTURE)
		dsp_control = DSP_AUDIO_CONTROL_READ_SHM;
	else
		dsp_control = DSP_AUDIO_CONTROL_WRITE_SHM;

	/* read/write data */
	dsp->p_dsp_common_data->ops->set_controls(
		xrtd->speech_probe->dsp, dsp_control, &rw_shm_data);

	xrtd->hwptr_done++;
	xrtd->periods++;

	xrtd->periods %= xrtd->stream->runtime->periods;
	xrtd->hwptr_done %= xrtd->stream->runtime->periods;

	snd_pcm_period_elapsed(xrtd->stream);
}

static int xgold_speech_probe_open(struct snd_pcm_substream *substream)
{
	int probe_point, probe_lisr_id, lisr_cb_id;
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct xgold_audio_speech_runtime_data *xrtd;

	struct xgold_audio_speech_probe *xgold_ptr =
		snd_soc_platform_get_drvdata(rtd->platform);

	probe_point = xgold_speech_probe_get_probe_point_id(substream);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		probe_lisr_id =
			xgold_speech_probe_get_record_lisr_id(probe_point);
	} else {
		probe_lisr_id =
			xgold_speech_probe_get_playback_lisr_id(probe_point);
	}
	lisr_cb_id = xgold_speech_probe_get_lisr_cb_id(probe_lisr_id);

	xgold_debug("enter for %s-In - substream: 0X%p, lisr_cb_id: %d\n",
		((substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
		"REC" : "PLAY"), substream, lisr_cb_id);

	xgold_debug("runtime: 0X%p\n", runtime);

	xrtd = kzalloc(sizeof(struct xgold_audio_speech_runtime_data),
			GFP_KERNEL);
	if (!xrtd)
		return -ENOMEM;

	xrtd->probe_point_id = probe_point;
	xrtd->probe_lisr_id = probe_lisr_id;

	register_dsp_audio_lisr_cb(lisr_cb_id,
		xgold_speech_probe_interrupt_handler,
		(void *)xrtd);

	snd_soc_set_runtime_hwparams(substream,
		&xgold_speech_probe_extract_cfg);

	xrtd->speech_probe = xgold_ptr;
	runtime->private_data = xrtd;

	return ret;
}

static int xgold_speech_probe_close(struct snd_pcm_substream *substream)
{
	int lisr_cb_id;
	struct xgold_audio_speech_runtime_data *xrtd =
		substream->runtime->private_data;

	if (!xrtd) {
		xgold_err("Runtime data is NULL.\n");
		return 0;
	}

	xgold_debug("enter - substream: 0X%p, probe_lisr_id: %d\n",
		substream, xrtd->probe_lisr_id);

	lisr_cb_id = xgold_speech_probe_get_lisr_cb_id(xrtd->probe_lisr_id);

	register_dsp_audio_lisr_cb(lisr_cb_id, NULL, NULL);

	kfree(xrtd);

	return 0;
}

static int xgold_speech_probe_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	int ret;
	struct xgold_audio_speech_runtime_data *xrtd =
		substream->runtime->private_data;

	xgold_debug("%s - substream: 0x%p - probe_lisr_id %d\n",
		((substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
		"REC" : "PLAY"), substream, xrtd->probe_lisr_id);

	/* Allocating DMA buffer */
	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (ret < 0)
		xgold_err("Failed to allocate memory error %d", ret);
	else
		memset(substream->runtime->dma_area, 0,
				params_buffer_bytes(params));

	return ret;
}

static int xgold_speech_probe_hw_free(struct snd_pcm_substream *substream)
{
	struct xgold_audio_speech_runtime_data *xrtd =
		substream->runtime->private_data;

	xgold_debug("enter - substream: 0X%p\n", substream);

	if (!xrtd) {
		xgold_err("Runtime data is NULL.\n");
		return 0;
	}

	/*Free DMA buffer */
	return snd_pcm_lib_free_pages(substream);
}

static int xgold_speech_probe_prepare(struct snd_pcm_substream *substream)
{
	struct xgold_audio_speech_runtime_data *xrtd =
		substream->runtime->private_data;

	xgold_debug("enter - substream: 0X%p\n", substream);

	if (!xrtd) {
		xgold_err("Runtime data is NULL.\n");
		return 0;
	}

	xrtd->hwptr_done = 0;
	xrtd->periods = 0;
	xrtd->stream = substream;
	xgold_debug("hwptr_done and periods set to 0 for probe_lisr_id %d\n",
		xrtd->probe_lisr_id);

	return 0;
}

static U16 dsp_cmd_get_on_off(
	int trigger_cmd,
	enum xgold_speech_probe_lisr_id play_lisr_id,
	enum xgold_speech_probe_lisr_id rec_lisr_id,
	struct xgold_audio_speech_runtime_data *xrtd)
{
	U16 switch_off = 0;
	U16 switch_update = 1;
	U16 switch_init_on = 3;

	bool play_status = xrtd->speech_probe->sp_status[play_lisr_id].active;
	bool rec_status = xrtd->speech_probe->sp_status[rec_lisr_id].active;

	struct dsp_audio_device *dsp =
		xrtd->speech_probe->dsp;

	/* init&on is 2 for XG642 and 3 for later variants */
	if (dsp->id == XGOLD_DSP_XG642)
		switch_init_on = 2;

	xgold_debug("trigger_cmd: %d, play_status: %d, rec_status: %d",
		trigger_cmd, play_status, rec_status);
	if (SNDRV_PCM_TRIGGER_START == trigger_cmd) {
		if ((true == play_status) && (true == rec_status)) {
			/* issue update since probe is already active */
			return switch_update;
		} else {
			/* issue init&on since probe is not active */
			return switch_init_on;
		}
	} else {
		if ((false == play_status) && (false == rec_status)) {
			/* issue off since probe is not active */
			return switch_off;
		} else {
			/* issue update one probe is still active */
			return switch_update;
		}
	}
	return switch_off;
}

static void dsp_cmd_set_params(
	int trigger_cmd,
	struct T_AUD_DSP_CMD_SPEECH_PROBE_PAR *speech_probe_par,
	struct xgold_audio_speech_runtime_data *xrtd)
{
	int probe_point = xrtd->probe_point_id;
	int stream_id_play, stream_id_rec;
	bool play_status, rec_status;

	if (probe_point >= PROBE_POINT_END) {
		xgold_err("%s: device: %d is out of range\n",
				__func__, probe_point);
		return;
	}

	stream_id_play = PROBE_POINT_TO_STREAM_PLAY(probe_point);
	stream_id_rec = PROBE_POINT_TO_STREAM_REC(probe_point);
	play_status = xrtd->speech_probe->sp_status[stream_id_play].active;
	rec_status = xrtd->speech_probe->sp_status[stream_id_rec].active;

	/* if inject is idle, mute gain 2 to avoid adding same signal again
	   back to UL/DL path */
	if (false == play_status)
		speech_probe_par->gain2 = DSP_GAIN_MUTE_VALUE;

	/* if extract is idle, mute gain 6 to ensure only the samples from
	   a potential player gets injected */
	if (false == rec_status)
		speech_probe_par->gain6 = DSP_GAIN_MUTE_VALUE;

	/* if both inject and extract is active, assume it is for
	   third party algo processing, hence mute gain1 */
	if ((true == play_status) && (true == rec_status))
		speech_probe_par->gain1 = DSP_GAIN_MUTE_VALUE;

	/* TODO: probe points 8 and 9 are used for voip and
	   mic recording use cases, and therefore controlled from IMAS
	   in userspace.
	   So move DSP part to DSP driver, and make a proper conflict handler */

	/* only basic inject/extract through shared memory is supported here */
	speech_probe_par->mode = 1;

	speech_probe_par->sm_buf_id = probe_point;

	speech_probe_par->select =
		xrtd->speech_probe->sp_status[stream_id_play].speech_probe_sel;

	speech_probe_par->on_off =
		dsp_cmd_get_on_off(trigger_cmd, stream_id_play,
			stream_id_rec, xrtd);

	return;
}

static int xgold_speech_probe_trigger(struct snd_pcm_substream *substream,
			int cmd)
{
	struct xgold_audio_speech_runtime_data *xrtd =
		substream->runtime->private_data;
	struct xgold_audio_speech_probe *xgold_ptr;

	struct T_AUD_DSP_CMD_SPEECH_PROBE_PAR speech_probe_par = {0};

	#ifdef CONFIG_SPEECH_PROBE_DEBUG
	struct T_AUD_DSP_CMD_VB_SET_SPEECH_PATH_PAR speech_path_on = {0};
	#endif

	if (!xrtd) {
		xgold_err("Runtime data is NULL.\n");
		return 0;
	}

	xgold_debug("enter - substream: 0X%p, probe_lisr_id %d\n",
		substream, xrtd->probe_lisr_id);

	xgold_ptr = xrtd->speech_probe;

	speech_probe_par.sampling_rate_ext =
		(substream->runtime->rate == 8000) ? 0 : 3;
	speech_probe_par.sampling_rate_inj =
		(substream->runtime->rate == 8000) ? 0 : 3;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:

		/* update status flag for the stream and update dsp cmds */
		xrtd->speech_probe->sp_status[
			xrtd->probe_lisr_id].active = true;
		dsp_cmd_set_params(cmd, &speech_probe_par, xrtd);

		xgold_debug("Trigger Start for %s\n",
			(substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
				"REC" : "PLAY");
		xgold_debug(
			"period size %ld ,periods %d buffer size %ld rate %d channels %d\n",
			substream->runtime->period_size,
			substream->runtime->periods,
			substream->runtime->buffer_size,
			substream->runtime->rate,
			substream->runtime->channels);

		xrtd->period_size_bytes = frames_to_bytes(
				xrtd->stream->runtime,
				xrtd->stream->runtime->period_size);

		/* For SOFIA 3G IRQ 2 is mapped to speech probes
		 For SOFIA LTE FBA Interrupt 2 is hared with VOLTE */
		if (xgold_ptr->dsp->id == XGOLD_DSP_XG642)
			xgold_ptr->dsp->p_dsp_common_data->ops->
				irq_activate(DSP_IRQ_2);
		else
			xgold_ptr->dsp->p_dsp_common_data->ops->
				irq_activate(DSP_FBA_IRQ_2);

#ifdef CONFIG_SPEECH_PROBE_DEBUG
		speech_path_on.setting = 1;

		/* Enable Speech path */
		dsp_audio_cmd(DSP_AUD_SPEECH_PATH,
			sizeof(struct T_AUD_DSP_CMD_VB_SET_SPEECH_PATH_PAR),
			(u16 *)&speech_path_on);
		xgold_debug("Speech path command sent to dsp\n");
#endif

		/* Enable Speech probes */
		xgold_debug("send DSP_AUD_SPEECH_PROBE cmd:\n");
		xgold_debug("on_off: %d, select: %d, sm_buf_id: %d\n",
			speech_probe_par.on_off, speech_probe_par.select,
			speech_probe_par.sm_buf_id);
		xgold_debug("rate inject: %d, rate extract: %d, mode: %d\n",
			speech_probe_par.sampling_rate_inj,
			speech_probe_par.sampling_rate_ext,
			speech_probe_par.mode);
		xgold_debug("Gain 1 -> 6:\n %d %d\n %d %d\n %d %d\n",
			speech_probe_par.gain1, speech_probe_par.gain2,
			speech_probe_par.gain3, speech_probe_par.gain4,
			speech_probe_par.gain5, speech_probe_par.gain6);
		dsp_audio_cmd(DSP_AUD_SPEECH_PROBE,
			sizeof(struct T_AUD_DSP_CMD_SPEECH_PROBE_PAR),
			(u16 *)&speech_probe_par);

#ifdef CONFIG_SPEECH_PROBE_DEBUG
		dsp_start_audio_hwafe();
#endif

		break;

	case SNDRV_PCM_TRIGGER_STOP:

		/* update status flag for the stream and update dsp cmds */
		xrtd->speech_probe->sp_status[
			xrtd->probe_lisr_id].active = false;
		dsp_cmd_set_params(cmd, &speech_probe_par, xrtd);

		xgold_debug("Trigger stop for %s\n",
			(substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
				"REC" : "PLAY");

		#ifdef CONFIG_SPEECH_PROBE_DEBUG
		speech_path_on.setting = 0;

		/*Disable Speech path */
		dsp_audio_cmd(DSP_AUD_SPEECH_PATH,
			sizeof(struct T_AUD_DSP_CMD_VB_SET_SPEECH_PATH_PAR),
			(u16 *)&speech_path_on);
		#endif

		xgold_debug("send DSP_AUD_SPEECH_PROBE cmd:\n");
		xgold_debug("on_off: %d, select: %d, sm_buf_id: %d\n",
			speech_probe_par.on_off, speech_probe_par.select,
			speech_probe_par.sm_buf_id);
		xgold_debug("rate inject: %d, rate extract: %d, mode: %d\n",
			speech_probe_par.sampling_rate_inj,
			speech_probe_par.sampling_rate_ext,
			speech_probe_par.mode);
		xgold_debug("Gain 1 -> 6:\n %d %d\n %d %d\n %d %d\n",
			speech_probe_par.gain1, speech_probe_par.gain2,
			speech_probe_par.gain3, speech_probe_par.gain4,
			speech_probe_par.gain5, speech_probe_par.gain6);
		dsp_audio_cmd(
			DSP_AUD_SPEECH_PROBE,
			sizeof(struct T_AUD_DSP_CMD_SPEECH_PROBE_PAR),
			(u16 *)&speech_probe_par);

		#ifdef CONFIG_SPEECH_PROBE_DEBUG
		dsp_stop_audio_hwafe();
		#endif

		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		xgold_debug("Trigger pause\n");
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		xgold_debug("Trigger pause release\n");
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t xgold_speech_probe_pointer(
			struct snd_pcm_substream *substream)
{
	struct xgold_audio_speech_runtime_data *xrtd =
		substream->runtime->private_data;
	unsigned int offset;

	if (!xrtd) {
		xgold_err("Runtime data is NULL.\n");
		return 0;
	}

	/* xgold_debug("enter for probe_lisr_id %d\n", xrtd->probe_lisr_id); */

	offset =
		(xrtd->periods) * frames_to_bytes(substream->runtime,
		substream->runtime->period_size);

	return bytes_to_frames(substream->runtime, offset);
}

static int xgold_speech_probe_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	xgold_debug("enter\n");

	ret = snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL),
			XGOLD_MAX_SPEECH_PROBE_RING_SIZE,
			XGOLD_MAX_SPEECH_PROBE_RING_SIZE);
	if (ret < 0)
		xgold_debug("failed to pre allocate DMA buffer");
	return ret;
}

static void xgold_speech_probe_free(struct snd_pcm *pcm)
{
	xgold_debug("enter\n");
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

/* Info function for speech probe SHMEM / io point mapping */
int xgold_speech_probe_ctl_info(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	xgold_debug("%s :\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = XGOLD_MIN_SPEECH_PROBE_SELECT;
	uinfo->value.integer.max = XGOLD_MAX_SPEECH_PROBE_SELECT;
	return 0;
}

/* Get function for speech probe SHMEM / io point mapping */
static int xgold_speech_probe_ctl_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int stream_id_play, stream_id_rec;

	struct xgold_speech_probes_status *sp_status_play;
	struct xgold_speech_probes_status *sp_status_rec;

	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);

	struct xgold_audio_speech_probe *speech_probe =
		snd_soc_dai_get_drvdata(cpu_dai);

	int probe_point_id =
		PROBE_DEVICE_TO_PROBE_POINT_ID(ucontrol->id.device);

	/* range check probe device id defined in
	   xgold_speech_probe_controls[] */
	if (probe_point_id >= PROBE_POINT_END) {
		xgold_err("device: %d is out of range\n", probe_point_id);
		return -EINVAL;
	}

	/* setup pointers to speech probe status of the given device */
	stream_id_play = PROBE_POINT_TO_STREAM_PLAY(probe_point_id);
	stream_id_rec = PROBE_POINT_TO_STREAM_REC(probe_point_id);

	sp_status_play = &speech_probe->sp_status[stream_id_play];
	sp_status_rec = &speech_probe->sp_status[stream_id_play];

	if (sp_status_play->speech_probe_sel !=
		sp_status_rec->speech_probe_sel) {
		xgold_err("play/rec probe select (%d / %d) not identical!\n",
			sp_status_play->speech_probe_sel,
			sp_status_rec->speech_probe_sel);
		return -EINVAL;
	}

	xgold_debug("play.probe_sel: %d, rec.probe_sel: %d\n",
		sp_status_play->speech_probe_sel,
		sp_status_rec->speech_probe_sel);

	ucontrol->value.integer.value[0] =
		(long int)sp_status_play->speech_probe_sel;

	xgold_debug("%s - get value: %d for device: %d\n",
		__func__, (int)ucontrol->value.integer.value[0],
		probe_point_id);

	return 0;
}

/* Set function for speech probe SHMEM / io point mapping */
static int xgold_speech_probe_ctl_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int stream_id_play, stream_id_rec;

	struct xgold_speech_probes_status *sp_status_play;
	struct xgold_speech_probes_status *sp_status_rec;

	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);

	struct xgold_audio_speech_probe *speech_probe =
		snd_soc_dai_get_drvdata(cpu_dai);

	int probe_point_id =
		PROBE_DEVICE_TO_PROBE_POINT_ID(ucontrol->id.device);

	unsigned int probe_sel_req =
		(unsigned int)ucontrol->value.integer.value[0];

	/* range check probe select value */
	if ((XGOLD_MAX_SPEECH_PROBE_SELECT < probe_sel_req) ||
		(XGOLD_MIN_SPEECH_PROBE_SELECT > probe_sel_req)) {
		xgold_err("device: %d is out of range\n", probe_point_id);
		return -EINVAL;
	}

	/* range check probe device id defined in
	   xgold_speech_probe_controls[] */
	if (probe_point_id >= PROBE_POINT_END) {
		xgold_err("device: %d is out of range\n", probe_point_id);
		return -EINVAL;
	}

	/* setup pointers to speech probe status of the given device */
	stream_id_play = PROBE_POINT_TO_STREAM_PLAY(probe_point_id);
	stream_id_rec = PROBE_POINT_TO_STREAM_REC(probe_point_id);

	sp_status_play = &speech_probe->sp_status[stream_id_play];
	sp_status_rec = &speech_probe->sp_status[stream_id_rec];

	/* if play or rec is active, don't allow to change the probe select */
	if (true == sp_status_play->active) {
		xgold_debug("id: %d, PLAY is active for probe: %d\n",
			probe_point_id, sp_status_play->speech_probe_sel);
		if (probe_sel_req != sp_status_play->speech_probe_sel) {
			xgold_err("probe sel req: %d fail!, active play probe using: %d\n",
			probe_sel_req, sp_status_play->speech_probe_sel);
			return -EPERM;
		}
	} else if (true == sp_status_rec->active) {
		xgold_debug("id: %d, REC is active for probe: %d\n",
			probe_point_id, sp_status_rec->speech_probe_sel);
		if (probe_sel_req != sp_status_rec->speech_probe_sel) {
			xgold_err("probe sel req: %d fail!, active rec probe using: %d\n",
			probe_sel_req, sp_status_rec->speech_probe_sel);
			return -EPERM;
		}
	}

	sp_status_rec->speech_probe_sel =
		sp_status_play->speech_probe_sel = probe_sel_req;

	xgold_debug("%s - set value: %d for device: %d\n",
		__func__, probe_sel_req,
		probe_point_id);
	return 0;
}

/* Soc xgold pcm controls */
static const struct snd_kcontrol_new xgold_speech_probe_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.device = 0 + XGOLD_SPEECH_PROBE_DEVICE_OFSET,
		.name = "PCM Speech probe 1",
		.info = xgold_speech_probe_ctl_info,
		.get = xgold_speech_probe_ctl_get,
		.put = xgold_speech_probe_ctl_set,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.device = 1 + XGOLD_SPEECH_PROBE_DEVICE_OFSET,
		.name = "PCM Speech probe 2",
		.info = xgold_speech_probe_ctl_info,
		.get = xgold_speech_probe_ctl_get,
		.put = xgold_speech_probe_ctl_set,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.device = 2 + XGOLD_SPEECH_PROBE_DEVICE_OFSET,
		.name = "PCM Speech probe 3",
		.info = xgold_speech_probe_ctl_info,
		.get = xgold_speech_probe_ctl_get,
		.put = xgold_speech_probe_ctl_set,
	},
#ifdef CONFIG_INCREASE_PCM_DEVICE
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.device = 3 + XGOLD_SPEECH_PROBE_DEVICE_OFSET,
		.name = "PCM Speech probe 4",
		.info = xgold_speech_probe_ctl_info,
		.get = xgold_speech_probe_ctl_get,
		.put = xgold_speech_probe_ctl_set,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.device = 4 + XGOLD_SPEECH_PROBE_DEVICE_OFSET,
		.name = "PCM Speech probe 5",
		.info = xgold_speech_probe_ctl_info,
		.get = xgold_speech_probe_ctl_get,
		.put = xgold_speech_probe_ctl_set,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.device = 5 + XGOLD_SPEECH_PROBE_DEVICE_OFSET,
		.name = "PCM Speech probe 6",
		.info = xgold_speech_probe_ctl_info,
		.get = xgold_speech_probe_ctl_get,
		.put = xgold_speech_probe_ctl_set,
	},
#endif /* CONFIG_INCREASE_PCM_DEVICE */
};

static int xgold_speech_soc_probe(struct snd_soc_platform *platform)
{
	int ret = 0;
	xgold_debug("%s\n", __func__);
	dsp_audio_platform_init(platform);

  /* add speech probe controls */
	ret = snd_soc_add_platform_controls(platform,
		xgold_speech_probe_controls,
		ARRAY_SIZE(xgold_speech_probe_controls));

	if (ret < 0) {
		xgold_err("%s: Unable to add speech probe platform controls\n",
		__func__);
		return -ENODEV;
	}

	return ret;
}

static struct snd_pcm_ops xgold_speech_probe_ops = {
	.open = xgold_speech_probe_open,
	.close = xgold_speech_probe_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = xgold_speech_probe_hw_params,
	.hw_free = xgold_speech_probe_hw_free,
	.prepare = xgold_speech_probe_prepare,
	.trigger = xgold_speech_probe_trigger,
	.pointer = xgold_speech_probe_pointer,
};

static struct snd_soc_dai_driver xgold_dai_speech_probe[] = {
{
	.name = "XGOLD_SPEECH_PROBE_A",
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
	.capture =  {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
},
{
	.name = "XGOLD_SPEECH_PROBE_B",
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
},
{
	.name = "XGOLD_SPEECH_PROBE_C",
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
	.capture =  {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
},
{
	.name = "XGOLD_SPEECH_PROBE_D",
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
	.capture =  {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
},
{
	.name = "XGOLD_SPEECH_PROBE_E",
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
},
{
	.name = "XGOLD_SPEECH_PROBE_F",
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = XGOLD_SPEECH_PROBE_RATES,
		.formats = XGOLD_SPEECH_PROBE_FORMAT,
	},
}
};

static const struct snd_soc_component_driver xgold_speech_probe_component = {
	.name = "xgold-speech",
};

struct snd_soc_platform_driver xgold_speech_probe_platform = {
	.probe = xgold_speech_soc_probe,
	.ops = &xgold_speech_probe_ops,
	.pcm_new = xgold_speech_probe_new,
	.pcm_free = xgold_speech_probe_free,
};

static int xgold_speech_probe(struct platform_device *pdev)
{
	struct xgold_audio_speech_probe *sp_ptr;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *dsp_of_node;
	int ret = 0;
	int idx;

	xgold_debug("enter\n");

	sp_ptr =
		kzalloc((sizeof(struct xgold_audio_speech_probe)), GFP_KERNEL);

	if (sp_ptr == NULL)
		return -ENOMEM;

	sp_ptr->dev = &pdev->dev;

#ifdef CONFIG_OF
	dsp_of_node = of_parse_phandle(np, "intel,dsp", 0);
	if (!dsp_of_node) {
		xgold_err("Unable to get dsp node\n");
		kzfree(sp_ptr);
		return -EINVAL;
	}

	sp_ptr->dsp =
		of_dsp_register_client(&pdev->dev, dsp_of_node);
#endif
	if (!sp_ptr->dsp) {
		xgold_err("Cannot register as dsp client\n");
		kzfree(sp_ptr);
		return -EPROBE_DEFER;
	}

	ret = snd_soc_register_platform(&pdev->dev,
			&xgold_speech_probe_platform);
	if (ret < 0) {
		xgold_err("Failed to register XGOLD Speech platform driver\n");
		kfree(sp_ptr);
		return ret;
	}

	ret = snd_soc_register_component(&pdev->dev,
			&xgold_speech_probe_component,
			xgold_dai_speech_probe,
			ARRAY_SIZE(xgold_dai_speech_probe));
	if (ret < 0) {
		xgold_err("Failed to register XGOLD Speech platform driver 1\n");
		kfree(sp_ptr);
		return ret;
	}

	/* select default speech probes select values */
	idx = PROBE_A_STREAM_PLAY;
	sp_ptr->sp_status[idx].speech_probe_sel =
		sp_ptr->sp_status[idx+1].speech_probe_sel = 1;

	idx = PROBE_B_STREAM_PLAY;
	sp_ptr->sp_status[idx].speech_probe_sel =
		sp_ptr->sp_status[idx+1].speech_probe_sel = 8;

	idx = PROBE_C_STREAM_PLAY;
	sp_ptr->sp_status[idx].speech_probe_sel =
		sp_ptr->sp_status[idx+1].speech_probe_sel = 16;

	idx = PROBE_D_STREAM_PLAY;
	sp_ptr->sp_status[idx].speech_probe_sel =
		sp_ptr->sp_status[idx+1].speech_probe_sel = 19;

#ifdef CONFIG_INCREASE_PCM_DEVICE
	idx = PROBE_E_STREAM_PLAY;
	sp_ptr->sp_status[idx].speech_probe_sel =
		sp_ptr->sp_status[idx+1].speech_probe_sel = 9;

	idx = PROBE_F_STREAM_PLAY;
	sp_ptr->sp_status[idx].speech_probe_sel =
		sp_ptr->sp_status[idx+1].speech_probe_sel = 14;
#endif /* CONFIG_INCREASE_PCM_DEVICE */

	platform_set_drvdata(pdev, sp_ptr);

	return ret;
}

static int xgold_speech_probe_remove(struct platform_device *pdev)
{
	xgold_debug("enter\n");
	snd_soc_unregister_component(&pdev->dev);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct of_device_id xgold_snd_speech_of_match[] = {
	{ .compatible = "intel,xgold-snd-speech", },
	{ },
};

static struct platform_driver xgold_snd_speech_probe_drv = {
	.driver = {
		.name = "Speech Probe",
		.owner = THIS_MODULE,
		.of_match_table = xgold_snd_speech_of_match,
	},
	.probe = xgold_speech_probe,
	.remove = xgold_speech_probe_remove,
};

static int __init xgold_snd_speech_probe_init(void)
{
	int ret = 0;

	xgold_debug("enter\n");

	ret = platform_driver_register(&xgold_snd_speech_probe_drv);

	if (ret < 0) {
		xgold_err("%s : Unable to add register speech probe platform driver\n",
			__func__);
		return -ENODEV;
	}
	return ret;
}

module_init(xgold_snd_speech_probe_init);

static void __exit snd_xgold_speech_probe_exit(void)
{
	xgold_debug("enter\n");
	platform_driver_unregister(&xgold_snd_speech_probe_drv);
}

module_exit(snd_xgold_speech_probe_exit);

MODULE_DESCRIPTION("XGOLD ASOC Platform driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL V2");
MODULE_DEVICE_TABLE(of, xgold_snd_speech_of_match);
