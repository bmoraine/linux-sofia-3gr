/*
 * Component: XGOLD pcm driver
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
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/amba/pl08x.h>

#include "dsp_audio_platform.h"
#include "aud_lib_dsp_internal.h"
#include "dsp_audio_hal_internal.h"
#include "xgold_pcm.h"

#ifdef CONFIG_SND_SOC_AGOLD_BT_SCO_STREAMING
#include "agold_bt_sco_streaming.h"
#endif

#define XGOLD_OUT_RATES SNDRV_PCM_RATE_8000_48000
#define XGOLD_OUT_FORMAT SNDRV_PCM_FMTBIT_S16_LE

#define XGOLD_IN_RATES SNDRV_PCM_RATE_8000_48000
#define XGOLD_IN_FORMAT SNDRV_PCM_FMTBIT_S16_LE

#define XGOLD_MAX_RING_SIZE		(32 * 1024)
#define XGOLD_MAX_SG_LIST		4
#define DMA_BURST_SIZE			256

/* FIXME */
static unsigned dma_mode = 1;

#define OFFSET_SM_AUDIO_BUFFER_1_DL	594
#define OFFSET_SM_AUDIO_BUFFER_SIZE_UL	1074
#define OFFSET_SM_AUDIO_BUFFER_UL	114

#define	xgold_err(fmt, arg...) \
		pr_err("snd: pcm: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: pcm: "fmt, ##arg)

struct xgold_dma_pcm_stream {
	struct scatterlist *dma_sgl;
	struct dma_chan *dmach;
	dma_cookie_t dma_cookie;
};

struct xgold_audio_stream {
	unsigned short *hwptr;
	unsigned int hwptr_done;
	unsigned int periods;
	unsigned int period_size_bytes;
	struct snd_pcm_substream *stream;
};

struct xgold_audio {
	struct device *dev;
	/* FIXME: union */
	struct xgold_audio_stream audio_stream[NR_STREAM];
	struct xgold_dma_pcm_stream audio_dma_stream[NR_STREAM];
};

static const struct snd_pcm_hardware xgold_pcm_play_cfg = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_SYNC_START |
		SNDRV_PCM_INFO_RESUME,
	.formats = XGOLD_OUT_FORMAT,
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = 61440,
	.period_bytes_min = 40,
	.period_bytes_max = 960,
	.periods_min = 2,
	.periods_max = 64,
};

static struct snd_pcm_hardware xgold_pcm_record_cfg = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_SYNC_START |
		SNDRV_PCM_INFO_RESUME,
	.formats = XGOLD_OUT_FORMAT,
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = 61440,
	.period_bytes_min = 40,
	.period_bytes_max = 960,
	.periods_min = 2,
	.periods_max = 64,
};

struct dsp_audio_device *p_dsp_audio_dev;

static struct T_AUD_DSP_CMD_VB_SET_GAIN_TIMECONST gain_const = { 0 };
static struct T_AUD_DSP_CMD_VB_SET_GAIN_HW_PAR pcm_gain = { 0 };
static struct T_AUD_DSP_CMD_VB_SET_IIR_PAR pcm_iir = { 0 };
static struct T_AUD_DSP_CMD_VB_SET_SWM_AFE_OUT_PAR pcm_afe = { 0 };
static struct T_AUD_DSP_CMD_VB_SET_SWM_PCM_OUT_PAR pcm_afe_in = { 0 };
static struct T_AUD_DSP_CMD_VB_SET_SWM_MIX_MATRIX mix_matrix = { 0 };
static struct T_AUD_DSP_CMD_PCM_PLAY_PAR pcm_par = { 0 };
static struct T_AUD_DSP_CMD_PCM_REC_PAR pcm_rec_par = { 0 };

int get_dsp_pcm_rate(unsigned int rate)
{
	switch (rate) {
	case 8000:
		return 0;
	case 11025:
		return 1;
	case 12000:
		return 2;
	case 16000:
		return 3;
	case 22050:
		return 4;
	case 24000:
		return 5;
	case 32000:
		return 6;
	case 44100:
		return 7;
	case 48000:
		return 8;
	}
	return -1;
}

int get_dsp_pcm_channels(unsigned int channels)
{
	if (channels == 1)
		return 0;
	else if (channels == 2)
		return 3;
	else
		return -1;
}

void setup_pcm_record_path(void)
{
	gain_const.timeconstant = 50;

	pcm_afe_in.afe_in = 1;
	pcm_afe_in.setting = 1;

	pcm_iir.iir_select = 0;
	pcm_iir.setting = 0;

	pcm_afe.afe_in = 1;
	pcm_afe.setting = 1;

	xgold_debug("PCM record VB SET GAIN const\n");
	dsp_audio_cmd(DSP_AUD_SET_GAIN_TIMECONST,
		sizeof(struct T_AUD_DSP_CMD_VB_SET_GAIN_TIMECONST),
		(u16 *)&gain_const);

	xgold_debug("VB SET PCM_OUT\n");
	dsp_audio_cmd(DSP_AUD_SET_SWM_PCM_OUT,
		sizeof(struct T_AUD_DSP_CMD_VB_SET_SWM_PCM_OUT_PAR),
		(u16 *)&pcm_afe_in);

	xgold_debug("VB SET IIR\n");
	dsp_audio_cmd(DSP_AUD_SET_IIR,
		sizeof(struct T_AUD_DSP_CMD_VB_SET_IIR_PAR), (u16 *)&pcm_iir);

	xgold_debug("VB SET HW GAIN\n");
	dsp_audio_cmd(DSP_AUD_SET_GAIN_HW,
		sizeof(struct T_AUD_DSP_CMD_VB_SET_GAIN_HW_PAR),
		(u16 *)&pcm_gain);
}

void setup_pcm_play_path(void)
{
	xgold_debug("Setup PCM path\n");

	pcm_afe.pcm_in = 1;
	pcm_afe.setting = 1;
	gain_const.timeconstant = 50;
	pcm_iir.iir_select = 10;
	pcm_iir.setting = 0;

	xgold_debug("VB SET GAIN const\n");
	dsp_audio_cmd(DSP_AUD_SET_GAIN_TIMECONST,
		sizeof(struct T_AUD_DSP_CMD_VB_SET_GAIN_TIMECONST),
		(u16 *)&gain_const);

	xgold_debug("VB SET AFE\n");

	dsp_audio_cmd(DSP_AUD_SET_SWM_AFE_OUT,
		sizeof(struct T_AUD_DSP_CMD_VB_SET_SWM_AFE_OUT_PAR),
		(u16 *)&pcm_afe);

	dsp_audio_cmd(DSP_AUD_SET_IIR,
		sizeof(struct T_AUD_DSP_CMD_VB_SET_IIR_PAR), (u16 *)&pcm_iir);

	pcm_iir.iir_select = 11;

	dsp_audio_cmd(DSP_AUD_SET_IIR,
		sizeof(struct T_AUD_DSP_CMD_VB_SET_IIR_PAR), (u16 *)&pcm_iir);

	xgold_debug("VB SET HW GAIN\n");

	dsp_audio_cmd(DSP_AUD_SET_GAIN_HW,
		sizeof(struct T_AUD_DSP_CMD_VB_SET_GAIN_HW_PAR),
		(u16 *)&pcm_gain);

	mix_matrix.select = 0;
	mix_matrix.alpha_0 = 0x7FFF;
	mix_matrix.alpha_3 = 0x7FFF;

	dsp_audio_cmd(DSP_AUD_SET_SWM_MIX_MATRIX,
		sizeof(struct T_AUD_DSP_CMD_VB_SET_SWM_MIX_MATRIX),
		(u16 *)&mix_matrix);
}

void xgold_dsp_pcm_rec_handler(void *dev)
{
	unsigned short nof_bytes_to_read = 0;
	struct dsp_rw_shm_data rw_shm_data;
	struct xgold_audio *xgold_ptr = (struct xgold_audio *)dev;
	struct xgold_audio_stream *xgold_stream;

	xgold_debug("%s\n", __func__);

	if (!xgold_ptr) {
		xgold_err("%s: stream data is NULL!!\n", __func__);
		return;
	}

	xgold_stream = &xgold_ptr->audio_stream[STREAM_REC];
	if ((NULL == xgold_stream->stream) ||
		(NULL == xgold_stream->stream->runtime) ||
		(NULL == xgold_stream->stream->runtime->dma_area)) {
			xgold_err("%s: stream data is NULL!!\n", __func__);
			return;
	}

	xgold_stream->hwptr =
		(unsigned short *)(xgold_stream->stream->runtime->dma_area +
			xgold_stream->period_size_bytes *
			xgold_stream->hwptr_done);

	/* read the buffer size */
	rw_shm_data.word_offset = OFFSET_SM_AUDIO_BUFFER_SIZE_UL;
	rw_shm_data.len_in_bytes = 2;
	rw_shm_data.p_data = &nof_bytes_to_read;
	p_dsp_audio_dev->ops->set_controls(
		DSP_AUDIO_CONTROL_READ_SHM, (void *)&rw_shm_data);

	/* read the samples */
	rw_shm_data.word_offset = OFFSET_SM_AUDIO_BUFFER_UL;
	rw_shm_data.len_in_bytes =
		(2 * nof_bytes_to_read *
		xgold_stream->stream->runtime->channels);

	rw_shm_data.p_data = xgold_stream->hwptr;
	p_dsp_audio_dev->ops->set_controls(
		DSP_AUDIO_CONTROL_READ_SHM, (void *)&rw_shm_data);

	xgold_stream->hwptr_done++;
	xgold_stream->periods++;
	xgold_stream->periods %= xgold_stream->stream->runtime->periods;
	xgold_stream->hwptr_done %= xgold_stream->stream->runtime->periods;
	snd_pcm_period_elapsed(xgold_stream->stream);
}

static void xgold_pcm_dma_submit(void *dev, dma_addr_t dma_addr);

void xgold_dsp_pcm_play_handler(void *dev)
{
	int i;
	struct xgold_audio *xgold_ptr = (struct xgold_audio *)dev;
	struct xgold_audio_stream *xgold_stream;

	unsigned int length = 0;
	unsigned short remaining_block = 0;
	struct dsp_rw_shm_data rw_shm_data;

	xgold_debug("%s\n", __func__);

	if (!xgold_ptr) {
		xgold_err("%s: stream data is NULL!!\n", __func__);
		return;
	}

	xgold_stream = &xgold_ptr->audio_stream[STREAM_PLAY];

	if ((NULL == xgold_stream->stream) ||
		(NULL == xgold_stream->stream->runtime) ||
		(NULL == xgold_stream->stream->runtime->dma_area)) {
			xgold_err("%s: stream data is NULL!!\n", __func__);
			return;
	}
	xgold_stream->hwptr =
		(unsigned short *)(xgold_stream->stream->runtime->dma_area +
			xgold_stream->period_size_bytes *
			xgold_stream->hwptr_done);

	rw_shm_data.word_offset = p_dsp_audio_dev->pcm_offset[0];
	rw_shm_data.len_in_bytes = 2;
	rw_shm_data.p_data = &remaining_block;
	p_dsp_audio_dev->ops->set_controls(
		DSP_AUDIO_CONTROL_READ_SHM, (void *)&rw_shm_data);

	/* write the samples */
	for (i = 0; i < remaining_block; i++) {
		length = xgold_stream->stream->runtime->period_size *
			xgold_stream->stream->runtime->channels;

		rw_shm_data.word_offset = OFFSET_SM_AUDIO_BUFFER_1_DL;
		rw_shm_data.len_in_bytes = length * 2;
		rw_shm_data.p_data = xgold_stream->hwptr;
		p_dsp_audio_dev->ops->set_controls(
			DSP_AUDIO_CONTROL_WRITE_SHM, (void *)&rw_shm_data);

		xgold_stream->hwptr_done++;
		xgold_stream->periods++;
		xgold_stream->periods %= xgold_stream->stream->runtime->periods;
		xgold_stream->hwptr_done %=
			xgold_stream->stream->runtime->periods;

		pcm_par.setting = 2;
		pcm_par.mode = get_dsp_pcm_channels(
			xgold_stream->stream->runtime->channels);

		pcm_par.rate = get_dsp_pcm_rate(
			xgold_stream->stream->runtime->rate);

		pcm_par.req = 0;

		/* Note that the DSP command sending involves mutex usage.
		Ensure that pcm buffeer handler is not called from
		interrupt context */
		dsp_audio_cmd(
			DSP_AUD_PCM1_PLAY,
			sizeof(struct T_AUD_DSP_CMD_PCM_PLAY_PAR),
			(u16 *) &pcm_par);
	}
	/*
	 * Period elapsed should be called once even
	 * for multiple buffer feeds
	 */
	snd_pcm_period_elapsed(xgold_stream->stream);
}

void xgold_dsp_pcm_dma_play_handler(void *dev)
{
	struct xgold_audio *xgold_ptr = (struct xgold_audio *)dev;
	struct xgold_audio_stream *xgold_stream;
	dma_addr_t dma_addr;

	xgold_debug("%s\n", __func__);

	if (!xgold_ptr) {
		xgold_err("%s: stream data is NULL!!\n", __func__);
		return;
	}

	xgold_stream = &(xgold_ptr->audio_stream[STREAM_PLAY]);

	if (!xgold_stream->stream || !xgold_stream->stream->runtime ||
			!xgold_stream->stream->runtime->dma_area) {
		xgold_err("%s: stream data is NULL!!\n", __func__);
		return;
	}

	xgold_stream->hwptr_done += XGOLD_MAX_SG_LIST;
	xgold_stream->periods += XGOLD_MAX_SG_LIST;
	xgold_stream->periods %= xgold_stream->stream->runtime->periods;
	xgold_stream->hwptr_done %=
		xgold_stream->stream->runtime->periods;

	dma_addr = xgold_stream->stream->runtime->dma_addr +
		xgold_stream->period_size_bytes * xgold_stream->hwptr_done;

	/* Period elapsed should be called once even
		for multiple buffer feeds */
	snd_pcm_period_elapsed(xgold_stream->stream);

	/* Reload scatter list and submit DMA request */
	xgold_pcm_dma_submit((void *)xgold_ptr, dma_addr);

	/* Restart the DMA tx for next data transfer i.e. after 20 ms */
	dma_async_issue_pending(xgold_ptr->audio_dma_stream[STREAM_PLAY].dmach);

	xgold_debug("dma tx started\n");
}

static void xgold_pcm_dma_submit(void *dev,
		dma_addr_t dma_addr)
{
	struct xgold_audio *xgold_ptr = (struct xgold_audio *)dev;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t dma_cookie_tx;
	int i = 0;

	struct xgold_dma_pcm_stream *pcm_dma_stream =
			&(xgold_ptr->audio_dma_stream[STREAM_PLAY]);
	struct xgold_audio_stream *xgold_stream =
		&(xgold_ptr->audio_stream[STREAM_PLAY]);

	xgold_stream->period_size_bytes =
		frames_to_bytes(xgold_stream->stream->runtime,
				xgold_stream->stream->runtime->
				period_size);

	/* xgold_debug("%s - period_size_bytes %d\n",
			__func__,
			xgold_stream->period_size_bytes);*/

	/* Prepare the scatter list */
	while (i != XGOLD_MAX_SG_LIST) {
		sg_set_buf(pcm_dma_stream->dma_sgl+i,
			(void *)phys_to_virt(dma_addr),
			xgold_stream->period_size_bytes);
		i++;
		dma_addr += xgold_stream->period_size_bytes;
	}

	dma_map_sg(xgold_ptr->dev, pcm_dma_stream->dma_sgl,
					XGOLD_MAX_SG_LIST,
					DMA_TO_DEVICE);

	/* Prepare DMA slave sg */
	desc = pcm_dma_stream->dmach->device->device_prep_slave_sg(
				pcm_dma_stream->dmach,
				pcm_dma_stream->dma_sgl,
				XGOLD_MAX_SG_LIST,
				DMA_SL_MEM_TO_MEM,
				DMA_PREP_INTERRUPT,
				NULL);

	/* Set the DMA callback */
	desc->callback = xgold_dsp_pcm_dma_play_handler;
	desc->callback_param = xgold_ptr;

	/* Submit DMA request */
	dma_cookie_tx = dmaengine_submit(desc);
}

int register_audio_dsp(struct dsp_audio_device *dsp)
{
	if (!dsp)
		return -ENODEV;
	if (p_dsp_audio_dev) {
		xgold_err("we already have a device %s\n",
				p_dsp_audio_dev->name);
		return -EEXIST;
	}
	xgold_debug("registering device %s\n", dsp->name);
	p_dsp_audio_dev = dsp;
	return 0;
}
EXPORT_SYMBOL_GPL(register_audio_dsp);

int unregister_audio_dsp(struct dsp_audio_device *dsp)
{
	if (dsp != p_dsp_audio_dev)
		return -EINVAL;
	if (p_dsp_audio_dev) {
		xgold_debug("unregister %s\n", p_dsp_audio_dev->name);
		p_dsp_audio_dev = NULL;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(unregister_audio_dsp);

static struct snd_soc_dai_driver xgold_dai_shm = {
	.name = "AGOLD_PCM",
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = XGOLD_OUT_RATES,
		.formats = XGOLD_OUT_FORMAT,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = XGOLD_IN_RATES,
		.formats = XGOLD_IN_FORMAT,
	},
};

static const struct snd_soc_component_driver xgold_shm_component = {
	.name = "xgold-pcm",
};

static int xgold_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct xgold_audio *xgold_ptr =
		snd_soc_platform_get_drvdata(rtd->platform);
	int ret = 0;

	xgold_debug("--> %s\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		snd_soc_set_runtime_hwparams(substream, &xgold_pcm_play_cfg);

		if (!dma_mode)
			register_dsp_audio_lisr_cb(
				DSP_LISR_CB_PCM_PLAYER,
				xgold_dsp_pcm_play_handler,
				(void *)xgold_ptr);
	} else {
		snd_soc_set_runtime_hwparams(substream, &xgold_pcm_record_cfg);

		register_dsp_audio_lisr_cb(
			DSP_LISR_CB_PCM_RECORDER,
			xgold_dsp_pcm_rec_handler,
			(void *)xgold_ptr);
	}

	/* Make sure, that the period size is always even */
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIODS, 2);

	if (ret < 0) {
		xgold_debug("Failed to set period size\n");
		return ret;
	}

	ret = snd_pcm_hw_constraint_integer(
		runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	if (ret < 0) {
		xgold_debug("Failed to set buffer size\n");
		return ret;
	}

	xgold_debug("<-- %s: %d\n", __func__, ret);
	return ret;
}

static int xgold_pcm_close(struct snd_pcm_substream *substream)
{
	xgold_debug("XGOLD Closing pcm device\n");
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (!dma_mode)
			register_dsp_audio_lisr_cb(
				DSP_LISR_CB_PCM_PLAYER,
				NULL,
				NULL);
	} else
		register_dsp_audio_lisr_cb(
			DSP_LISR_CB_PCM_RECORDER,
			NULL,
			NULL);

	return 0;
}

static int xgold_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	int ret;

	xgold_debug("%s\n", __func__);

	if (dma_mode) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			substream->dma_buffer.dev.type = SNDRV_DMA_TYPE_DEV;
		else
			substream->dma_buffer.dev.type =
				SNDRV_DMA_TYPE_CONTINUOUS;
	}

	/* Allocating DMA buffer */
	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (ret < 0)
		xgold_err("%s: Failed to alloc pages for DMA: %d\n",
				__func__, ret);
	else
		memset(substream->runtime->dma_area, 0,
				params_buffer_bytes(params));

	return ret;
}

static int xgold_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct xgold_audio *xgold_ptr =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct xgold_dma_pcm_stream *pcm_dma_stream;

	xgold_debug("%s\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK && dma_mode) {
		pcm_dma_stream = &xgold_ptr->audio_dma_stream[STREAM_PLAY];

		/* Release the DMA channel */
		if (pcm_dma_stream->dmach) {
			dma_release_channel(pcm_dma_stream->dmach);
			pcm_dma_stream->dmach = NULL;
		}

		/* Free scatter list memory*/
		kfree(pcm_dma_stream->dma_sgl);
	}

	/* Free DMA buffer */
	return snd_pcm_lib_free_pages(substream);
}

static int xgold_pcm_play_dma_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct xgold_audio *xgold_ptr;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct xgold_dma_pcm_stream *pcm_dma_stream;
	struct dma_slave_config pcm_dma_config;
	dma_addr_t dma_addr;
	int ret = 0;
	dma_cap_mask_t tx_mask;
	unsigned short *shm_base =
		(unsigned short *)dsp_get_audio_shmem_base_addr() +
		OFFSET_SM_AUDIO_BUFFER_1_DL;

	xgold_ptr = snd_soc_platform_get_drvdata(rtd->platform);

	dma_cap_zero(tx_mask);
	dma_cap_set(DMA_SLAVE, tx_mask);

	pcm_dma_stream = &xgold_ptr->audio_dma_stream[STREAM_PLAY];

#ifdef CONFIG_OF
	pcm_dma_stream->dmach =
		xgold_of_dsp_get_dmach(p_dsp_audio_dev, STREAM_PLAY);
#else
	pcm_dma_stream->dmach =
		dma_request_channel(tx_mask, pl08x_filter_id,
				(void *)"dsp_dma_req1");
#endif

	if (!pcm_dma_stream->dmach) {
		xgold_err("%s: dma channel req fail\n", __func__);
		return -EIO;
	}

	/* Config DMA slave parameters */
	pcm_dma_config.direction = DMA_TO_DEVICE;
	pcm_dma_config.dst_addr = (dma_addr_t)shm_base;
	pcm_dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	pcm_dma_config.dst_maxburst = DMA_BURST_SIZE;
	pcm_dma_config.device_fc = false;

	ret = dmaengine_slave_config(pcm_dma_stream->dmach,
					&pcm_dma_config);
	if (ret) {
		xgold_debug("pcm: error in dma slave configuration\n");
		return ret;
	}

	dma_addr = runtime->dma_addr;

	pcm_dma_stream->dma_sgl =
		kzalloc(sizeof(struct scatterlist) * XGOLD_MAX_SG_LIST,
						GFP_KERNEL);
	if (!pcm_dma_stream->dma_sgl)
		return -ENOMEM;

	sg_init_table(pcm_dma_stream->dma_sgl, XGOLD_MAX_SG_LIST);

	/* Load scatter list and submit DMA request */
	xgold_pcm_dma_submit(xgold_ptr, dma_addr);

	return 0;
}

static int xgold_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct xgold_audio *xgold_ptr;
	xgold_ptr = snd_soc_platform_get_drvdata(rtd->platform);

	xgold_debug("%s\n", __func__);

	if (p_dsp_audio_dev->native_mode)
		dsp_start_audio_hwafe();

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		xgold_ptr->audio_stream[STREAM_PLAY].hwptr_done = 0;
		xgold_ptr->audio_stream[STREAM_PLAY].periods = 0;
		xgold_ptr->audio_stream[STREAM_PLAY].stream = substream;

		if (dma_mode)
			xgold_pcm_play_dma_prepare(substream);

		if (p_dsp_audio_dev->native_mode)
			setup_pcm_play_path();

	} else {
		xgold_ptr->audio_stream[STREAM_REC].hwptr_done = 0;
		xgold_ptr->audio_stream[STREAM_REC].periods = 0;
		xgold_ptr->audio_stream[STREAM_REC].stream = substream;

		if (p_dsp_audio_dev->native_mode)
			setup_pcm_record_path();
	}

	return 0;
}

static int xgold_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct xgold_audio *xgold_ptr;
	struct xgold_audio_stream *xgold_stream;

	xgold_ptr = snd_soc_platform_get_drvdata(rtd->platform);
	xgold_debug("%s\n", __func__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		xgold_stream = &xgold_ptr->audio_stream[STREAM_PLAY];
		xgold_debug("%s: Trigger Start\n", __func__);
		xgold_debug(
			"period size %ld, periods %d buffer size %ld "
			"rate %d channels %d\n",
			substream->runtime->period_size,
			substream->runtime->periods,
			substream->runtime->buffer_size,
			substream->runtime->rate,
			substream->runtime->channels);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			xgold_stream->period_size_bytes =
				frames_to_bytes(xgold_stream->stream->runtime,
						xgold_stream->stream->runtime->
						period_size);

			pcm_par.mode = get_dsp_pcm_channels(
				substream->runtime->channels);
			pcm_par.setting = 1;
			pcm_par.rate =
				get_dsp_pcm_rate(substream->runtime->rate);

			if (dma_mode) {
				pcm_par.req = 1;
				/* request DMA to start tx */
				dma_async_issue_pending(xgold_ptr->
						audio_dma_stream[STREAM_PLAY].
						dmach);
			} else {
				pcm_par.req = 0;
				/* Activate the interrupt*/
				dsp_audio_irq_activate(p_dsp_audio_dev,
						DSP_IRQ_1);
			}

			xgold_debug("PCM play cmd mode %d rate %d req %d",
					pcm_par.mode, pcm_par.rate,
					pcm_par.req);
			dsp_audio_cmd(
				DSP_AUD_PCM1_PLAY,
				sizeof(struct T_AUD_DSP_CMD_PCM_PLAY_PAR),
				(u16 *) &pcm_par);
		} else {
			xgold_stream = &xgold_ptr->audio_stream[STREAM_REC];
			xgold_stream->period_size_bytes =
				frames_to_bytes(xgold_stream->stream->runtime,
						xgold_stream->stream->runtime->
						period_size);

			pcm_rec_par.mode =
				get_dsp_pcm_channels(
					substream->runtime->channels);
			pcm_rec_par.rate =
				get_dsp_pcm_rate(
					substream->runtime->rate);
			pcm_rec_par.path_select = 0;
			pcm_rec_par.setting = 3;
			pcm_rec_par.req = 0;

			/* Activate the interrupt*/
			dsp_audio_irq_activate(p_dsp_audio_dev, DSP_IRQ_2);

			xgold_debug("PCM record cmd mode %d rate %d",
				pcm_rec_par.mode, pcm_rec_par.rate);

			dsp_audio_cmd(
				DSP_AUD_PCM_REC,
				sizeof(struct T_AUD_DSP_CMD_PCM_REC_PAR),
				(u16 *) &pcm_rec_par);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:

		xgold_debug("%s: Trigger stop\n", __func__);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (dma_mode) {
				/* request DMA shutdown */
				dmaengine_terminate_all(
				xgold_ptr->audio_dma_stream[STREAM_PLAY].dmach);
			}

			pcm_par.setting = 0;
			dsp_audio_cmd(
				DSP_AUD_PCM1_PLAY,
				sizeof(struct T_AUD_DSP_CMD_PCM_PLAY_PAR),
				(u16 *)&pcm_par);
		} else {
			pcm_rec_par.setting = 0;
			dsp_audio_cmd(
				DSP_AUD_PCM_REC,
				sizeof(struct T_AUD_DSP_CMD_PCM_REC_PAR),
				(u16 *)&pcm_rec_par);

		}

		xgold_debug("DSP stopped\n");
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

static snd_pcm_uframes_t xgold_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct xgold_audio *xgold_ptr =
			snd_soc_platform_get_drvdata(rtd->platform);
	struct xgold_audio_stream xgold_stream;
	unsigned int offset;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		xgold_stream = xgold_ptr->audio_stream[STREAM_PLAY];
	else
		xgold_stream = xgold_ptr->audio_stream[STREAM_REC];

	offset = xgold_stream.periods * frames_to_bytes(substream->runtime,
			substream->runtime->period_size);

	return bytes_to_frames(substream->runtime, offset);
}

static u64 xgold_pcm_dmamask = DMA_BIT_MASK(32);

static int xgold_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	xgold_debug("%s\n", __func__);

	if (dma_mode) {
		if (!rtd->dev->dma_mask)
			rtd->dev->dma_mask = &xgold_pcm_dmamask;

		dma_set_coherent_mask(rtd->dev, DMA_BIT_MASK(32));

		ret = snd_pcm_lib_preallocate_pages_for_all(
			rtd->pcm,
			SNDRV_DMA_TYPE_DEV,
			rtd->dev,
			XGOLD_MAX_RING_SIZE,
			XGOLD_MAX_RING_SIZE);
	} else
		ret = snd_pcm_lib_preallocate_pages_for_all(
			rtd->pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL),
			XGOLD_MAX_RING_SIZE,
			XGOLD_MAX_RING_SIZE);

	if (ret < 0)
		xgold_err("%s : failed to pre allocate DMA buffer", __func__);

	return ret;
}

static void xgold_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	xgold_debug("%s\n", __func__);
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static int xgold_pcm_soc_probe(struct snd_soc_platform *platform)
{
	int ret = 0;
	xgold_debug("%s\n", __func__);
	dsp_audio_platform_init(platform);
#ifdef CONFIG_SND_SOC_AGOLD_BT_SCO_STREAMING
	ret = xgold_bt_sco_soc_init(platform);
#endif
	return ret;
}

static struct snd_pcm_ops xgold_pcm_ops = {
	.open = xgold_pcm_open,
	.close = xgold_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = xgold_pcm_hw_params,
	.hw_free = xgold_pcm_hw_free,
	.prepare = xgold_pcm_prepare,
	.trigger = xgold_pcm_trigger,
	.pointer = xgold_pcm_pointer,
};

static struct snd_soc_platform_driver xgold_soc_platform = {
	.probe = xgold_pcm_soc_probe,
	.ops = &xgold_pcm_ops,
	.pcm_new = xgold_pcm_new,
	.pcm_free = xgold_pcm_free_dma_buffers,
};

static int xgold_pcm_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct xgold_audio *pcm_data_ptr = NULL;

	xgold_debug("%s\n", __func__);

	pcm_data_ptr = kzalloc(sizeof(struct xgold_audio), GFP_KERNEL);

	if (pcm_data_ptr == NULL)
		return -ENOMEM;

	pcm_data_ptr->dev = &pdev->dev;
	ret =
	snd_soc_register_platform(&pdev->dev, &xgold_soc_platform);

	if (ret < 0) {
		xgold_err("Failed to register XGOLD platform driver\n");
		kfree(pcm_data_ptr);
		return ret;
	}

	ret = snd_soc_register_component(
		&pdev->dev,
		&xgold_shm_component,
		&xgold_dai_shm, 1);

	if (ret < 0) {
		xgold_err("Failed to register XGOLD platform driver 1\n");
		kfree(pcm_data_ptr);
		return ret;
	}

	platform_set_drvdata(pdev, pcm_data_ptr);

	return ret;
}

static int xgold_pcm_remove(struct platform_device *pdev)
{
	xgold_debug("%s :\n", __func__);
	snd_soc_unregister_component(&pdev->dev);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct of_device_id xgold_snd_pcm_of_match[] = {
	{ .compatible = "intel,xgold-snd-pcm", },
	{ },
};

static struct platform_driver xgold_snd_pcm_drv = {
	.driver = {
		.name = "AGOLD_PCM",
		.owner = THIS_MODULE,
		.of_match_table = xgold_snd_pcm_of_match,
	},
	.probe = xgold_pcm_probe,
	.remove = xgold_pcm_remove,
};

static int __init xgold_snd_pcm_init(void)
{
	int ret = 0;

	xgold_debug("%s\n", __func__);

	ret = platform_driver_register(&xgold_snd_pcm_drv);

	if (ret < 0) {
		xgold_err("%s : Unable to add register pcm platform driver\n",
		__func__);
		return -ENODEV;
	}
	return ret;
}
module_init(xgold_snd_pcm_init);

static void __exit snd_xgold_pcm_exit(void)
{
	xgold_debug("%s\n", __func__);
	platform_driver_unregister(&xgold_snd_pcm_drv);
}
module_exit(snd_xgold_pcm_exit);

MODULE_DESCRIPTION("XGOLD ASOC Platform driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL V2");
MODULE_DEVICE_TABLE(of, xgold_snd_pcm_of_match);
