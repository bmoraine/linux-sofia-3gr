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

#define XGOLD_SPEECH_PROBE_RATES (SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000)
#define XGOLD_SPEECH_PROBE_FORMAT SNDRV_PCM_FMTBIT_S16_LE

#define XGOLD_MAX_SPEECH_PROBE_RING_SIZE (4 * 1024)

#define XGOLD_NOF_SPEECH_PROBES 6



#define	xgold_err(fmt, arg...) \
		pr_err("snd: speech: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: speech: "fmt, ##arg)

struct xgold_audio_speech_probe {
	struct device *dev;
	struct dsp_audio_device *dsp;
	struct xgold_audio_stream
		audio_stream[NR_STREAM * XGOLD_NOF_SPEECH_PROBES];
};

enum {
	PROBE_POINT_A,
	PROBE_POINT_B,
	PROBE_POINT_C,
	PROBE_POINT_D,
	PROBE_POINT_E,
	PROBE_POINT_F,
	PROBE_POINT_END
};

enum {
	PROBE_A_STREAM_PLAY,
	PROBE_A_STREAM_REC,
	PROBE_B_STREAM_PLAY,
	PROBE_B_STREAM_REC,
	PROBE_C_STREAM_PLAY,
	PROBE_C_STREAM_REC,
	PROBE_D_STREAM_PLAY,
	PROBE_D_STREAM_REC,
	PROBE_E_STREAM_PLAY,
	PROBE_E_STREAM_REC,
	PROBE_F_STREAM_PLAY,
	PROBE_F_STREAM_REC,
	PROBE_POINT_STREAM_END
};

static const struct snd_pcm_hardware
	xgold_speech_probe_extract_cfg[XGOLD_NOF_SPEECH_PROBES] = {
	{
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
		.buffer_bytes_max = 1280,
		.period_bytes_min = 160,
		.period_bytes_max = 640,
		.periods_min = 1,
		.periods_max = 2,
	},
	{
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
		.buffer_bytes_max = 1280,
		.period_bytes_min = 160,
		.period_bytes_max = 640,
		.periods_min = 1,
		.periods_max = 2,
	},
	{
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
		.buffer_bytes_max = 1280,
		.period_bytes_min = 160,
		.period_bytes_max = 640,
		.periods_min = 1,
		.periods_max = 2,
	},
	{
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
		.buffer_bytes_max = 1280,
		.period_bytes_min = 160,
		.period_bytes_max = 640,
		.periods_min = 1,
		.periods_max = 2,
	},
	{
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
		.buffer_bytes_max = 1280,
		.period_bytes_min = 160,
		.period_bytes_max = 640,
		.periods_min = 1,
		.periods_max = 2,
	},
	{
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
		.buffer_bytes_max = 1280,
		.period_bytes_min = 160,
		.period_bytes_max = 640,
		.periods_min = 1,
		.periods_max = 2,
	},
};

void xgold_pcm_speech_io_point_a_interrupt_handler(void *dev)
{
	struct xgold_audio_speech_probe *xgold_ptr =
		(struct xgold_audio_speech_probe *)dev;

	struct xgold_audio_stream *xgold_stream =
		&xgold_ptr->audio_stream[PROBE_A_STREAM_REC];

	struct dsp_rw_shm_data rw_shm_data;

	#ifdef CONFIG_SPEECH_PROBE_DEBUG
	static int count;

	if (count == 10) {
		xgold_debug("%s:\n", __func__);
		count = 0;
	} else
		count++;
	#endif

	xgold_stream->hwptr =
		(unsigned short *)(xgold_stream->stream->runtime->dma_area +
		xgold_stream->period_size_bytes * xgold_stream->hwptr_done);

	rw_shm_data.word_offset = xgold_ptr->dsp->p_dsp_common_data->
		buf_sm_speech_probe_a_offset;

	rw_shm_data.len_in_bytes = 320; /* read the samples */
	rw_shm_data.p_data = xgold_stream->hwptr;

	xgold_ptr->dsp->p_dsp_common_data->ops->set_controls(
			xgold_ptr->dsp,
			DSP_AUDIO_CONTROL_READ_SHM,
			&rw_shm_data);

	xgold_stream->hwptr_done++;
	xgold_stream->periods++;

	xgold_stream->periods %= xgold_stream->stream->runtime->periods + 1;
	xgold_stream->hwptr_done %= xgold_stream->stream->runtime->periods + 1;

	snd_pcm_period_elapsed(xgold_stream->stream);
}

void xgold_pcm_speech_io_point_b_interrupt_handler(void *dev)
{
	struct xgold_audio_speech_probe *xgold_ptr =
		(struct xgold_audio_speech_probe *)dev;

	struct xgold_audio_stream *xgold_stream =
	&xgold_ptr->audio_stream[PROBE_B_STREAM_REC];

	struct dsp_rw_shm_data rw_shm_data;

	#ifdef CONFIG_SPEECH_PROBE_DEBUG
	static int count;

	if (count == 10) {
		xgold_debug("%s:\n", __func__);
		count = 0;
	} else
		count++;
	#endif

	xgold_stream->hwptr =
		(unsigned short *)(xgold_stream->stream->runtime->dma_area +
		xgold_stream->period_size_bytes * xgold_stream->hwptr_done);

	rw_shm_data.word_offset = xgold_ptr->dsp->p_dsp_common_data->
		buf_sm_speech_probe_b_offset;

	rw_shm_data.len_in_bytes = 320; /* read the samples */
	rw_shm_data.p_data = xgold_stream->hwptr;

	xgold_ptr->dsp->p_dsp_common_data->ops->set_controls(
			xgold_ptr->dsp,
			DSP_AUDIO_CONTROL_READ_SHM,
			&rw_shm_data);

	xgold_stream->hwptr_done++;
	xgold_stream->periods++;

	xgold_stream->periods %= xgold_stream->stream->runtime->periods + 1;
	xgold_stream->hwptr_done %= xgold_stream->stream->runtime->periods + 1;

	snd_pcm_period_elapsed(xgold_stream->stream);
}

void xgold_pcm_speech_io_point_c_interrupt_handler(void *dev)
{
	struct xgold_audio_speech_probe *xgold_ptr =
		(struct xgold_audio_speech_probe *)dev;

	struct xgold_audio_stream *xgold_stream =
		&xgold_ptr->audio_stream[PROBE_C_STREAM_REC];

	struct dsp_rw_shm_data rw_shm_data;

#ifdef CONFIG_SPEECH_PROBE_DEBUG
	static int count;

	if (count == 10) {
		xgold_debug("%s\n", __func__);
		count = 0;
	} else
		count++;
#endif

	xgold_stream->hwptr =
		(unsigned short *)(xgold_stream->stream->runtime->dma_area +
		xgold_stream->period_size_bytes * xgold_stream->hwptr_done);

	/* read the samples */
	rw_shm_data.word_offset = xgold_ptr->dsp->p_dsp_common_data->
		buf_sm_speech_probe_c_offset;

	rw_shm_data.len_in_bytes = 320;
	rw_shm_data.p_data = xgold_stream->hwptr;

	xgold_ptr->dsp->p_dsp_common_data->ops->set_controls(
			xgold_ptr->dsp,
			DSP_AUDIO_CONTROL_READ_SHM,
			&rw_shm_data);

	xgold_stream->hwptr_done++;
	xgold_stream->periods++;

	xgold_stream->periods %= xgold_stream->stream->runtime->periods + 1;
	xgold_stream->hwptr_done %= xgold_stream->stream->runtime->periods + 1;

	snd_pcm_period_elapsed(xgold_stream->stream);
}

void xgold_pcm_speech_io_point_d_interrupt_handler(void *dev)
{
	struct xgold_audio_speech_probe *xgold_ptr =
		(struct xgold_audio_speech_probe *)dev;

	struct xgold_audio_stream *xgold_stream =
		&xgold_ptr->audio_stream[PROBE_D_STREAM_REC];

	struct dsp_rw_shm_data rw_shm_data;

#ifdef CONFIG_SPEECH_PROBE_DEBUG
	static int count;

	if (count == 10) {
		xgold_debug("%s\n", __func__);
		count = 0;
	} else
		count++;
#endif

	xgold_stream->hwptr =
		(unsigned short *)(xgold_stream->stream->runtime->dma_area +
		xgold_stream->period_size_bytes * xgold_stream->hwptr_done);

	rw_shm_data.word_offset = xgold_ptr->dsp->p_dsp_common_data->
		buf_sm_speech_probe_d_offset;

	rw_shm_data.len_in_bytes = 320; /* read the samples */
	rw_shm_data.p_data = xgold_stream->hwptr;

	xgold_ptr->dsp->p_dsp_common_data->ops->set_controls(
			xgold_ptr->dsp,
			DSP_AUDIO_CONTROL_READ_SHM,
			&rw_shm_data);

	xgold_stream->hwptr_done++;
	xgold_stream->periods++;

	xgold_stream->periods %= xgold_stream->stream->runtime->periods + 1;
	xgold_stream->hwptr_done %= xgold_stream->stream->runtime->periods + 1;

	snd_pcm_period_elapsed(xgold_stream->stream);
}

void xgold_pcm_speech_io_point_e_interrupt_handler(void *dev)
{
	struct xgold_audio_speech_probe *xgold_ptr =
		(struct xgold_audio_speech_probe *)dev;

	struct xgold_audio_stream *xgold_stream =
		&xgold_ptr->audio_stream[PROBE_E_STREAM_REC];

	struct dsp_rw_shm_data rw_shm_data;

#ifdef CONFIG_SPEECH_PROBE_DEBUG
	static int count;

	if (count == 10) {
		xgold_debug("%s\n", __func__);
		count = 0;
	} else
		count++;
#endif

	xgold_stream->hwptr =
		(unsigned short *)(xgold_stream->stream->runtime->dma_area +
		xgold_stream->period_size_bytes * xgold_stream->hwptr_done);

	rw_shm_data.word_offset = xgold_ptr->dsp->p_dsp_common_data->
		buf_sm_speech_probe_e_offset;

	rw_shm_data.len_in_bytes = 320; /* read the samples */
	rw_shm_data.p_data = xgold_stream->hwptr;

	xgold_ptr->dsp->p_dsp_common_data->ops->set_controls(
			xgold_ptr->dsp,
			DSP_AUDIO_CONTROL_READ_SHM,
			&rw_shm_data);

	xgold_stream->hwptr_done++;
	xgold_stream->periods++;

	xgold_stream->periods %= xgold_stream->stream->runtime->periods + 1;
	xgold_stream->hwptr_done %= xgold_stream->stream->runtime->periods + 1;

	snd_pcm_period_elapsed(xgold_stream->stream);
}

void xgold_pcm_speech_io_point_f_interrupt_handler(void *dev)
{
	struct xgold_audio_speech_probe *xgold_ptr =
		(struct xgold_audio_speech_probe *)dev;

	struct xgold_audio_stream *xgold_stream =
		&xgold_ptr->audio_stream[PROBE_F_STREAM_REC];

	struct dsp_rw_shm_data rw_shm_data;

	#ifdef CONFIG_SPEECH_PROBE_DEBUG
	static int count;

	if (count == 10) {
		xgold_debug("%s:\n", __func__);
		count = 0;
	} else
		count++;
	#endif


	xgold_stream->hwptr =
		(unsigned short *)(xgold_stream->stream->runtime->dma_area +
		xgold_stream->period_size_bytes * xgold_stream->hwptr_done);

	rw_shm_data.word_offset = xgold_ptr->dsp->p_dsp_common_data->
		buf_sm_speech_probe_f_offset;

	rw_shm_data.len_in_bytes = 320; /* read the samples */
	rw_shm_data.p_data = xgold_stream->hwptr;

	xgold_ptr->dsp->p_dsp_common_data->ops->set_controls(
			xgold_ptr->dsp,
			DSP_AUDIO_CONTROL_READ_SHM,
			&rw_shm_data);

	xgold_stream->hwptr_done++;
	xgold_stream->periods++;

	xgold_stream->periods %= xgold_stream->stream->runtime->periods + 1;
	xgold_stream->hwptr_done %= xgold_stream->stream->runtime->periods + 1;

	snd_pcm_period_elapsed(xgold_stream->stream);
}

static int xgold_speech_probe_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	struct xgold_audio_speech_probe *xgold_ptr =
		snd_soc_platform_get_drvdata(rtd->platform);

	/* Support Only for Extraction */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return -EPERM;

	xgold_debug("xgold_speech_probe_open for %s-In & %s - 0X%p\n",
		((substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
		"REC" :
		"PLAY"),
		rtd->cpu_dai->name, substream);

	switch (rtd->cpu_dai->name[13]) {
	case 'A':

		register_dsp_audio_lisr_cb(DSP_LISR_CB_SPEECH_IO_POINT_A,
		xgold_pcm_speech_io_point_a_interrupt_handler,
		(void *)xgold_ptr);

		xgold_debug("speech_probe_A cb register\n");
		snd_soc_set_runtime_hwparams(substream,
		&xgold_speech_probe_extract_cfg[0]);

		break;
	case 'B':

		register_dsp_audio_lisr_cb(DSP_LISR_CB_SPEECH_IO_POINT_B,
		xgold_pcm_speech_io_point_b_interrupt_handler,
		(void *)xgold_ptr);

		xgold_debug("speech_probe_B cb register\n");

		snd_soc_set_runtime_hwparams(substream,
		&xgold_speech_probe_extract_cfg[1]);

		break;
	case 'C':

		register_dsp_audio_lisr_cb(DSP_LISR_CB_SPEECH_IO_POINT_C,
		xgold_pcm_speech_io_point_c_interrupt_handler,
		(void *)xgold_ptr);

		xgold_debug("speech_probe_C cb register\n");

		snd_soc_set_runtime_hwparams(substream,
		&xgold_speech_probe_extract_cfg[2]);

		break;
	case 'D':

		register_dsp_audio_lisr_cb(DSP_LISR_CB_SPEECH_IO_POINT_D,
		xgold_pcm_speech_io_point_d_interrupt_handler,
		(void *)xgold_ptr);

		xgold_debug("speech_probe_D cb register\n");

		snd_soc_set_runtime_hwparams(substream,
		&xgold_speech_probe_extract_cfg[3]);

		break;
	case 'E':

		register_dsp_audio_lisr_cb(DSP_LISR_CB_SPEECH_IO_POINT_E,
		xgold_pcm_speech_io_point_e_interrupt_handler,
		(void *)xgold_ptr);

		xgold_debug("speech_probe_E cb register\n");

		snd_soc_set_runtime_hwparams(substream,
		&xgold_speech_probe_extract_cfg[4]);
		break;
	case 'F':

		register_dsp_audio_lisr_cb(DSP_LISR_CB_SPEECH_IO_POINT_F,
		xgold_pcm_speech_io_point_f_interrupt_handler,
		(void *)xgold_ptr);

		xgold_debug("speech_probe_F cb register\n");

		snd_soc_set_runtime_hwparams(substream,
		&xgold_speech_probe_extract_cfg[5]);

		break;
	default:
		xgold_debug(
			"speech_probe req for %c\n",
			rtd->cpu_dai->name[13]);
		break;
	}

	return ret;
}

static int xgold_speech_probe_close(struct snd_pcm_substream *substream)
{
	xgold_debug("XGOLD Closing speech probe device\n");
	return 0;
}

static int xgold_speech_probe_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct xgold_audio_speech_probe *xgold_ptr =
				snd_soc_platform_get_drvdata(rtd->platform);
	int ret;

	xgold_debug("%s for %s-In - 0X%p\n", __func__,
		((substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
		"REC" :
		"PLAY"),
		substream);

	/* Allocating DMA buffer */
	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (ret < 0) {
		xgold_err("Failed to allocate memory error %d", ret);
		return ret;
	}

	memset(substream->runtime->dma_area, 0, params_buffer_bytes(params));

	/* Support only for extraction */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return -EPERM;

	switch (rtd->cpu_dai->name[13]) {
	case 'A':

		xgold_ptr->audio_stream[PROBE_A_STREAM_REC].stream = substream;
		xgold_debug("Malloc for audio_stream[1]= 0X%p\n",
		xgold_ptr->audio_stream[PROBE_A_STREAM_REC].stream);

		break;
	case 'B':

		xgold_ptr->audio_stream[PROBE_B_STREAM_REC].stream = substream;
		xgold_debug("Malloc for audio_stream[3]= 0X%p\n",
		xgold_ptr->audio_stream[PROBE_B_STREAM_REC].stream);

		break;

	case 'C':

		xgold_ptr->audio_stream[PROBE_C_STREAM_REC].stream = substream;
		xgold_debug("Malloc for audio_stream[5]= 0X%p\n",
		xgold_ptr->audio_stream[PROBE_C_STREAM_REC].stream);

		break;

	case 'D':

		xgold_ptr->audio_stream[PROBE_D_STREAM_REC].stream = substream;
		xgold_debug("Malloc for audio_stream[7]= 0X%p\n",
		xgold_ptr->audio_stream[PROBE_D_STREAM_REC].stream);

		break;

	case 'E':

		xgold_ptr->audio_stream[PROBE_E_STREAM_REC].stream = substream;
		xgold_debug("Malloc for audio_stream[9]= 0X%p\n",
		xgold_ptr->audio_stream[PROBE_E_STREAM_REC].stream);

		break;
	case 'F':

		xgold_ptr->audio_stream[PROBE_F_STREAM_REC].stream = substream;
		xgold_debug("Malloc for audio_stream[11]= 0X%p\n",
		xgold_ptr->audio_stream[PROBE_F_STREAM_REC].stream);

		break;
	default:
		xgold_debug("No valid stream found\n");
		break;
	}

	xgold_debug("%s for %s-Out\n", __func__,
			(substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
			"REC" : "PLAY");
	return ret;
}

static int xgold_speech_probe_hw_free(struct snd_pcm_substream *substream)
{
	xgold_debug("%s\n", __func__);
	/*Free DMA buffer */
	return snd_pcm_lib_free_pages(substream);
}

static int xgold_speech_probe_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct xgold_audio_speech_probe *xgold_ptr;
	xgold_ptr = snd_soc_platform_get_drvdata(rtd->platform);

	xgold_debug("%s for %s-In\n", __func__,
	((substream->stream == SNDRV_PCM_STREAM_CAPTURE) ? "REC" : "PLAY"));

	/* Currently only extraction supported */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return -EPERM;

	switch (rtd->cpu_dai->name[13]) {
	case 'A':

		xgold_ptr->audio_stream[PROBE_A_STREAM_REC].hwptr_done = 0;
		xgold_ptr->audio_stream[PROBE_A_STREAM_REC].periods = 0;
		break;

	case 'B':

		xgold_ptr->audio_stream[PROBE_B_STREAM_REC].hwptr_done = 0;
		xgold_ptr->audio_stream[PROBE_B_STREAM_REC].periods = 0;
		break;

	case 'C':

		xgold_ptr->audio_stream[PROBE_C_STREAM_REC].hwptr_done = 0;
		xgold_ptr->audio_stream[PROBE_C_STREAM_REC].periods = 0;
		break;

	case 'D':

		xgold_ptr->audio_stream[PROBE_D_STREAM_REC].hwptr_done = 0;
		xgold_ptr->audio_stream[PROBE_D_STREAM_REC].periods = 0;
		break;

	case 'E':

		xgold_ptr->audio_stream[PROBE_E_STREAM_REC].hwptr_done = 0;
		xgold_ptr->audio_stream[PROBE_E_STREAM_REC].periods = 0;
		break;

	case 'F':

		xgold_ptr->audio_stream[PROBE_F_STREAM_REC].hwptr_done = 0;
		xgold_ptr->audio_stream[PROBE_F_STREAM_REC].periods = 0;
		break;

	default:
		xgold_debug("no valid stream found\n");
		break;
	}
	xgold_debug("%s for %s-Out\n", __func__,
	((substream->stream == SNDRV_PCM_STREAM_CAPTURE) ? "REC" : "PLAY"));

	return 0;
}

static int xgold_speech_probe_set_param(
			struct T_AUD_DSP_CMD_SPEECH_PROBE_PAR *speech_probe_par,
			int probe_point)
{
	switch (probe_point) {
	case PROBE_POINT_A:
		speech_probe_par->setting = 8;
		speech_probe_par->sm_buf_id = 0;
		break;
	case PROBE_POINT_B:
		speech_probe_par->setting = 9;
		speech_probe_par->sm_buf_id = 1;
		break;
	case PROBE_POINT_C:
		speech_probe_par->setting = 1;
		speech_probe_par->sm_buf_id = 2;
		break;
	case PROBE_POINT_D:
		speech_probe_par->setting = 14;
		speech_probe_par->sm_buf_id = 3;
		break;
	case PROBE_POINT_E:
		speech_probe_par->setting = 7;
		speech_probe_par->sm_buf_id = 4;
		break;
	case PROBE_POINT_F:
		speech_probe_par->setting = 10;
		speech_probe_par->sm_buf_id = 5;
		break;
	default:
		break;
	}
	return 1;
}

static int xgold_speech_probe_get_probe_point(
			struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int io_probe_point;

	switch (rtd->cpu_dai->name[13]) {
	case 'A':
		io_probe_point = PROBE_POINT_A;
		break;
	case 'B':
		io_probe_point = PROBE_POINT_B;
		break;
	case 'C':
		io_probe_point = PROBE_POINT_C;
		break;
	case 'D':
		io_probe_point = PROBE_POINT_D;
		break;
	case 'E':
		io_probe_point = PROBE_POINT_E;
		break;
	case 'F':
		io_probe_point = PROBE_POINT_F;
		break;
	default:
		io_probe_point = PROBE_POINT_END;
		break;
	}
	return io_probe_point;
}

static int xgold_speech_probe_get_stream_point(
		struct snd_pcm_substream *substream,
		int io_probe_point)
{
	int io_stream_point;

	switch (io_probe_point) {
	case PROBE_POINT_A:
		io_stream_point = PROBE_A_STREAM_REC;
		break;
	case PROBE_POINT_B:
		io_stream_point = PROBE_B_STREAM_REC;
		break;
	case PROBE_POINT_C:
		io_stream_point = PROBE_C_STREAM_REC;
		break;
	case PROBE_POINT_D:
		io_stream_point = PROBE_D_STREAM_REC;
		break;
	case PROBE_POINT_E:
		io_stream_point = PROBE_E_STREAM_REC;
		break;
	case PROBE_POINT_F:
		io_stream_point = PROBE_F_STREAM_REC;
		break;
	default:
		io_stream_point = PROBE_POINT_STREAM_END;
		break;
	}
	return io_stream_point;
}

static int xgold_speech_probe_trigger(struct snd_pcm_substream *substream,
			int cmd)
{
	int probe_point, stream_point;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct xgold_audio_speech_probe *xgold_ptr;
	struct xgold_audio_stream *xgold_stream;
	struct T_AUD_DSP_CMD_SPEECH_PROBE_PAR speech_probe_par = {0};

	#ifdef CONFIG_SPEECH_PROBE_DEBUG
	struct T_AUD_DSP_CMD_VB_SET_SPEECH_PATH_PAR speech_path_on = {0};
	#endif

	/* Extraction only supported */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return -EPERM;

	speech_probe_par.sampling_rate_ext =
		(substream->runtime->rate == 8000) ? 0 : 3;
	speech_probe_par.sampling_rate_inj =
		(substream->runtime->rate == 8000) ? 0 : 3;
	speech_probe_par.mode = 1;


	probe_point = xgold_speech_probe_get_probe_point(substream);

	stream_point =
		xgold_speech_probe_get_stream_point(substream, probe_point);

	xgold_speech_probe_set_param(&speech_probe_par, probe_point);

	xgold_ptr = snd_soc_platform_get_drvdata(rtd->platform);

	xgold_debug("%s\n", __func__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:

		speech_probe_par.on_off = 2;
		xgold_debug("%s :Trigger Start for %s\n", __func__,
			(substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
			"REC" :
			"PLAY");
		xgold_debug(
			"period size %ld ,periods %d buffer size %ld rate %d channels %d\n",
			substream->runtime->period_size,
			substream->runtime->periods,
			substream->runtime->buffer_size,
			substream->runtime->rate,
			substream->runtime->channels);

		xgold_stream = &xgold_ptr->audio_stream[stream_point];
		xgold_stream->period_size_bytes =
			frames_to_bytes(xgold_stream->stream->runtime,
			xgold_stream->stream->runtime->period_size);

		xgold_debug("Speech command sent to dsp\n");

		/* For SOFIA 3G IRQ 2 is mapped to speech probes */
		if (xgold_ptr->dsp->id == XGOLD_DSP_XG642)
			xgold_ptr->dsp->p_dsp_common_data->ops->
				irq_activate(DSP_IRQ_2);
		else
			xgold_ptr->dsp->p_dsp_common_data->ops->
				irq_activate(DSP_IRQ_6);

#ifdef CONFIG_SPEECH_PROBE_DEBUG
		speech_path_on.setting = 1;

		/* Enable Speech path */
		dsp_audio_cmd(DSP_AUD_SPEECH_PATH,
			sizeof(struct T_AUD_DSP_CMD_VB_SET_SPEECH_PATH_PAR),
			(u16 *)&speech_path_on);
#endif

		/* Enable Speech probes */
		dsp_audio_cmd(DSP_AUD_SPEECH_PROBE,
			sizeof(struct T_AUD_DSP_CMD_SPEECH_PROBE_PAR),
			(u16 *)&speech_probe_par);

#ifdef CONFIG_SPEECH_PROBE_DEBUG
		dsp_start_audio_hwafe();
#endif

		break;

	case SNDRV_PCM_TRIGGER_STOP:

		xgold_debug("%s :Trigger stop for %s\n", __func__,
			(substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
			"REC" :
			"PLAY");

		#ifdef CONFIG_SPEECH_PROBE_DEBUG
		speech_path_on.setting = 0;

		/*Disable Speech path */
		dsp_audio_cmd(DSP_AUD_SPEECH_PATH,
			sizeof(struct T_AUD_DSP_CMD_VB_SET_SPEECH_PATH_PAR),
			(u16 *)&speech_path_on);
		#endif

		speech_probe_par.on_off = 0;

		dsp_audio_cmd(
			DSP_AUD_SPEECH_PROBE,
			sizeof(struct T_AUD_DSP_CMD_SPEECH_PROBE_PAR),
			(u16 *)&speech_probe_par);

		#ifdef CONFIG_SPEECH_PROBE_DEBUG
		dsp_stop_audio_hwafe();
		#endif

		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		xgold_debug("%s: Trigger pause\n", __func__);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		xgold_debug("%s: Trigger pause release\n", __func__);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t xgold_speech_probe_pointer(
			struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct xgold_audio_speech_probe *xgold_ptr =
		snd_soc_platform_get_drvdata(rtd->platform);

	struct xgold_audio_stream xgold_stream;
	unsigned int offset;

	switch (rtd->cpu_dai->name[13]) {
	case 'A':

		xgold_stream = xgold_ptr->audio_stream[PROBE_A_STREAM_REC];
		break;

	case 'B':

		xgold_stream = xgold_ptr->audio_stream[PROBE_B_STREAM_REC];
		break;

	case 'C':

		xgold_stream = xgold_ptr->audio_stream[PROBE_C_STREAM_REC];
		break;

	case 'D':

		xgold_stream = xgold_ptr->audio_stream[PROBE_D_STREAM_REC];
		break;

	case 'E':

		xgold_stream = xgold_ptr->audio_stream[PROBE_E_STREAM_REC];
		break;

	case 'F':

		xgold_stream = xgold_ptr->audio_stream[PROBE_F_STREAM_REC];
		break;

	default:
		xgold_debug("no valid stream found\n");
		return 0;
	}

	offset =
		(xgold_stream.periods) * frames_to_bytes(substream->runtime,
		substream->runtime->period_size);

	return bytes_to_frames(substream->runtime, offset);
}

static int xgold_speech_probe_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	xgold_debug("%s\n", __func__);

	ret = snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL),
			XGOLD_MAX_SPEECH_PROBE_RING_SIZE,
			XGOLD_MAX_SPEECH_PROBE_RING_SIZE);
	if (ret < 0)
		xgold_debug("%s : failed to pre allocate DMA buffer", __func__);
	return ret;
}

static void xgold_speech_probe_free(struct snd_pcm *pcm)
{
	xgold_debug("%s\n", __func__);
	snd_pcm_lib_preallocate_free_for_all(pcm);
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
	.name = "Speech Probe_A",
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
	.name = "Speech Probe_B",
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
	.name = "Speech Probe_C",
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
	.name = "Speech Probe_D",
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
	.name = "Speech Probe_E",
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
	.name = "Speech Probe_F",
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
	.name		= "xgold-speech",
};

struct snd_soc_platform_driver xgold_speech_probe_platform = {
	.ops = &xgold_speech_probe_ops,
	.pcm_new = xgold_speech_probe_new,
	.pcm_free = xgold_speech_probe_free,
};

static int xgold_speech_probe(struct platform_device *pdev)
{
	struct xgold_audio_speech_probe *speech_probe_data_ptr;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *dsp_of_node;
	int ret = 0;

	xgold_debug("%s\n", __func__);

	speech_probe_data_ptr =
		kzalloc((sizeof(struct xgold_audio_speech_probe)), GFP_KERNEL);

	if (speech_probe_data_ptr == NULL)
		return -ENOMEM;

	speech_probe_data_ptr->dev = &pdev->dev;

#ifdef CONFIG_OF
	dsp_of_node = of_parse_phandle(np, "intel,dsp", 0);
	if (!dsp_of_node) {
		xgold_err("Unable to get dsp node\n");
		kzfree(speech_probe_data_ptr);
		return -EINVAL;
	}

	speech_probe_data_ptr->dsp =
		of_dsp_register_client(&pdev->dev, dsp_of_node);
#endif
	if (!speech_probe_data_ptr->dsp) {
		xgold_err("Cannot register as dsp client\n");
		kzfree(speech_probe_data_ptr);
		return -EPROBE_DEFER;
	}

	ret = snd_soc_register_platform(&pdev->dev,
			&xgold_speech_probe_platform);
	if (ret < 0) {
		xgold_err("Failed to register XGOLD Speech platform driver\n");
		kfree(speech_probe_data_ptr);
		return ret;
	}

	ret = snd_soc_register_component(&pdev->dev,
			&xgold_speech_probe_component,
			xgold_dai_speech_probe,
			ARRAY_SIZE(xgold_dai_speech_probe));
	if (ret < 0) {
		xgold_err("Failed to register XGOLD Speech platform driver 1\n");
		kfree(speech_probe_data_ptr);
		return ret;
	}

	platform_set_drvdata(pdev, speech_probe_data_ptr);

	return ret;
}

static int xgold_speech_probe_remove(struct platform_device *pdev)
{
	xgold_debug("%s :\n", __func__);
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

	xgold_debug("%s\n", __func__);

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
	xgold_debug("%s\n", __func__);
	platform_driver_unregister(&xgold_snd_speech_probe_drv);
}

module_exit(snd_xgold_speech_probe_exit);

MODULE_DESCRIPTION("XGOLD ASOC Platform driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL V2");
MODULE_DEVICE_TABLE(of, xgold_snd_speech_of_match);
