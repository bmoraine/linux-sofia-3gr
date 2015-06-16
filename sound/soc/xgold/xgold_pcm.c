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
#include <linux/of.h>
#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/platform_device_pm.h>
#endif
#include "dsp_audio_platform.h"
#include "aud_lib_dsp_internal.h"
#include "dsp_audio_hal_internal.h"
#include "xgold_pcm.h"
#include "xgold_machine.h"
#include "agold_bt_sco_streaming.h"

/* DMA DUMP */
#include <linux/vmalloc.h>
#include <linux/fs.h>

#define DUMP_DMA_PERIOD (10)
#define DUMP_DMA_SIZE (4 * 48000 * DUMP_DMA_PERIOD)

/* RECORD DUMP */
#define DUMP_RECORD_PERIOD (10)
#define DUMP_RECORD_SIZE (4 * 48000 * DUMP_RECORD_PERIOD)

/* Fix me move this to DTS */
#define XGOLD_OUT_RATES SNDRV_PCM_RATE_8000_48000
#define XGOLD_OUT_FORMAT SNDRV_PCM_FMTBIT_S16_LE

#define XGOLD_IN_RATES SNDRV_PCM_RATE_8000_48000
#define XGOLD_IN_FORMAT SNDRV_PCM_FMTBIT_S16_LE

#define XGOLD_MAX_RING_SIZE		(380 * 1024)
#define XGOLD_MAX_SG_LIST		4
#define DMA_BURST_SIZE			256

#define PROP_PCM_DMA_EN_NAME		"intel,pcm,dma_en"
#define PROP_SND_SOC_BT_INIT		"intel,pcm,bt_init_en"
#define PROP_PCM_REC_DMA_EN_NAME	"intel,pcm,rec_dma_en"
#define PROP_SND_SOC_BURST_EN_NAME_PCM2		"intel,pcm,burst_mode_en_pcm2"

#define ON 1
#define OFF 0

#define SYSFS_INPUT_VAL_LEN (1)

#define SM_AUDIO_BUFFER_DL_SAMPLES 240
/* SHM can store 5 ms data, it is 960 bytes
   Maximum period circle is 24 * 5 = 120 ms
   DMA circle is 120 * XGOLD_MAX_SG_LIST = 480 ms */
#define XGOLD_MAX_PERIOD_BYTES (960 * 24) /* 24 * 5 = 120 ms */
#define XGOLD_MAX_BUFFER_BYTES (XGOLD_MAX_PERIOD_BYTES * 16)

/* index of the first HW probe device in the array xgold_dai */
#define XGOLD_HW_PROBE_DEVICE_OFSET 2

#define XGOLD_MIN_HW_PROBE_SELECT (1)
#define XGOLD_MAX_HW_PROBE_SELECT (14)

/* convert from device ID in xgold_speech_probe_controls to probe point ID
	enum xgold_speech_probe_point_id */
#define	HW_PROBE_DEVICE_TO_PROBE_POINT_ID(a) \
	(a - XGOLD_HW_PROBE_DEVICE_OFSET)

static unsigned int bt_init_en;

#define	xgold_err(fmt, arg...) \
		pr_err("snd: pcm: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: pcm: "fmt, ##arg)

enum xgold_pcm_attribute {
	XGOLD_PCM_ATTR_SEND_DSP_CMD = 1,
	XGOLD_PCM_ATTR_END,
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
	.buffer_bytes_max = XGOLD_MAX_BUFFER_BYTES,
	.period_bytes_min = 40,
	.period_bytes_max = XGOLD_MAX_PERIOD_BYTES,
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
	.buffer_bytes_max = 112896,
	.period_bytes_min = 40,
	.period_bytes_max = 1764,
	.periods_min = 2,
	.periods_max = 64,
};

static u16 xgold_pcm_sysfs_attribute_value;

/* validation arrays for the HW probe points */
const unsigned xg642_hw_probe_valid[] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9};

const unsigned hw_probe_valid[] = {
	1, 2, 3, 4, 9, 10, 11, 12, 13, 14};

static void xgold_pcm_dma_play_submit(struct xgold_runtime_data *, dma_addr_t);
static void xgold_pcm_dma_rec_submit(struct xgold_runtime_data *, dma_addr_t);

static bool is_hw_probe_point_valid(
	enum dsp_id dsp_id,
	unsigned int probe_sel_req)
{
	int idx = 0;

	if (XGOLD_DSP_XG642 == dsp_id) {
		int loop_max = sizeof(xg642_hw_probe_valid)/sizeof(unsigned);
		for (idx = 0; idx < loop_max; idx++) {
			if (xg642_hw_probe_valid[idx] == probe_sel_req)
				return true;
		}
	} else {
		int loop_max = sizeof(hw_probe_valid)/sizeof(unsigned);
		for (idx = 0; idx < loop_max; idx++) {
			if (hw_probe_valid[idx] == probe_sel_req)
				return true;
		}
	}
	return false;
}

/* Info function for pcm rec path select control */
int xgold_pcm_rec_path_sel_ctl_info(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	xgold_debug("%s\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static inline int i2s_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	int ret = 0;
	struct xgold_pcm *pcm = dev_get_drvdata(dev);

	if (!pcm) {
		dev_err(dev, "Unable to retrieve pcm data\n");
		return -EINVAL;
	}
	if (!IS_ERR_OR_NULL(pcm->pinctrl)) {
		if (!IS_ERR_OR_NULL(state)) {
			ret = pinctrl_select_state(
					pcm->pinctrl,
					state);
			if (ret)
				dev_err(dev,
					"%d:could not set pins\n", __LINE__);
		}
	}
	return ret;
}

/* FIXME why not I2S8 ?*/
/* Set I2S2 device details to DSP structure */
static void i2s2_set_device_data(struct device *dev,
	enum i2s_devices device)
{
	struct xgold_pcm *pcm = dev_get_drvdata(dev);
	struct dsp_i2s_device *i2s_dev =
		pcm->dsp->p_dsp_common_data->p_i2s_dev[device];

	xgold_debug("%s: device %d\n", __func__, device);

	i2s_dev->dev = dev;
	i2s_dev->pinctrl = pcm->pinctrl;
	i2s_dev->pins_default = pcm->pins_default;
	i2s_dev->pins_inactive = pcm->pins_inactive;
	i2s_dev->pins_sleep = pcm->pins_sleep;
	i2s_dev->pm_platdata = pcm->pm_platdata;
}

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

/* FIXME use DAPM to setup DSP path */
void setup_pcm_record_path(struct xgold_pcm *xgold_pcm)
{
	switch (xgold_pcm_sysfs_attribute_value) {
	case XGOLD_PCM_ATTR_SEND_DSP_CMD:
		xgold_debug("Setup PCM path\n");
		xgold_pcm->dsp->p_dsp_common_data->ops->set_controls(
				xgold_pcm->dsp,
				DSP_AUDIO_CONTROL_SET_REC_PATH,
				NULL);
		break;
	default:
		break;
	}
}

/* FIXME use DAPM to setup DSP path */
void setup_pcm_play_path(struct xgold_pcm *xgold_pcm)
{
	switch (xgold_pcm_sysfs_attribute_value) {
	case XGOLD_PCM_ATTR_SEND_DSP_CMD:
		xgold_debug("Setup PCM path\n");
		xgold_pcm->dsp->p_dsp_common_data->ops->set_controls(
				xgold_pcm->dsp,
				DSP_AUDIO_CONTROL_SET_PLAY_PATH,
				NULL);
		break;
	default:
		break;
	}
}

void xgold_dsp_hw_probe_a_handler(void *dev)
{
	struct xgold_runtime_data *xrtd = (struct xgold_runtime_data *)dev;
	struct xgold_pcm *xgold_pcm;
	struct dsp_rw_shm_data rw_shm_data;
	static int count;

	if (count == 20) {
		xgold_debug("%s\n", __func__);
		count = 0;
	} else
		count++;

	if (!xrtd) {
		xgold_err("%s: xgold runtime data is NULL!!\n", __func__);
		return;
	}

	xgold_pcm = xrtd->pcm;
	if (!xgold_pcm || !xrtd->stream || !xrtd->stream->runtime ||
			!xrtd->stream->runtime->dma_area) {
		xgold_err("%s: stream data is NULL!!\n", __func__);
		return;
	}

	xrtd->hwptr = (unsigned short *)(xrtd->stream->runtime->dma_area +
			xrtd->period_size_bytes * xrtd->hwptr_done);

	/* read the samples */
	rw_shm_data.word_offset =
		xgold_pcm->dsp->p_dsp_common_data->buf_sm_hw_probe_a_offset;

	/* TODO:
	 * could we use same calculation as for playback and speech probes?
	 *	length = xrtd->stream->runtime->period_size *
	 *	xrtd->stream->runtime->channels;
	 *	rw_shm_data.len_in_bytes = length * 2;
	*/
	rw_shm_data.len_in_bytes =
		(2 * 240 * xrtd->stream->runtime->channels);

	rw_shm_data.p_data = xrtd->hwptr;

	xgold_pcm->dsp->p_dsp_common_data->ops->set_controls(
			xgold_pcm->dsp,
			DSP_AUDIO_CONTROL_READ_SHM,
			&rw_shm_data);

	xrtd->hwptr_done++;
	xrtd->periods++;
	xrtd->periods %= xrtd->stream->runtime->periods;
	xrtd->hwptr_done %= xrtd->stream->runtime->periods;
	snd_pcm_period_elapsed(xrtd->stream);
}

/* FIXME: factorise probe_a/b handlers */
void xgold_dsp_hw_probe_b_handler(void *dev)
{
	struct xgold_runtime_data *xrtd = (struct xgold_runtime_data *)dev;
	struct xgold_pcm *xgold_pcm;
	struct dsp_rw_shm_data rw_shm_data;
	static int count;

	if (count == 20) {
		xgold_debug("%s\n", __func__);
		count = 0;
	} else
		count++;

	if (!xrtd) {
		xgold_err("%s: xgold runtime data is NULL!!\n", __func__);
		return;
	}

	xgold_pcm = xrtd->pcm;
	if (!xgold_pcm || !xrtd->stream || !xrtd->stream->runtime ||
			!xrtd->stream->runtime->dma_area) {
		xgold_err("%s: stream data is NULL!!\n", __func__);
		return;
	}

	xrtd->hwptr = (unsigned short *)(xrtd->stream->runtime->dma_area +
			xrtd->period_size_bytes * xrtd->hwptr_done);

	/* read the samples */
	rw_shm_data.word_offset =
		xgold_pcm->dsp->p_dsp_common_data->buf_sm_hw_probe_b_offset;

	rw_shm_data.len_in_bytes =
		(2 * 240 * xrtd->stream->runtime->channels);

	rw_shm_data.p_data = xrtd->hwptr;

	xgold_pcm->dsp->p_dsp_common_data->ops->set_controls(
			xgold_pcm->dsp,
			DSP_AUDIO_CONTROL_READ_SHM,
			&rw_shm_data);

	xrtd->hwptr_done++;
	xrtd->periods++;
	xrtd->periods %= xrtd->stream->runtime->periods;
	xrtd->hwptr_done %= xrtd->stream->runtime->periods;
	snd_pcm_period_elapsed(xrtd->stream);
}

void xgold_dsp_pcm_rec_handler(void *dev)
{
	struct xgold_runtime_data *xrtd = (struct xgold_runtime_data *)dev;
	struct xgold_pcm *xgold_pcm;
	struct dsp_rw_shm_data rw_shm_data;
	unsigned short nof_bytes_to_read = 0;
	unsigned short *pData;
	xgold_debug("%s\n", __func__);
	if (!xrtd) {
		xgold_err("%s: xgold runtime data is NULL!!\n", __func__);
		return;
	}

	xgold_pcm = xrtd->pcm;
	if (!xgold_pcm || !xrtd->stream || !xrtd->stream->runtime ||
			!xrtd->stream->runtime->dma_area) {
		xgold_err("%s: stream data is NULL!!\n", __func__);
		return;
	}
	xrtd->hwptr = (unsigned short *)(xrtd->stream->runtime->dma_area +
			xrtd->total_nof_bytes_read +
			xrtd->period_size_bytes * xrtd->hwptr_done);

	/* read the buffer size */
	rw_shm_data.word_offset =
		xgold_pcm->dsp->p_dsp_common_data->buf_size_ul_offset;

	rw_shm_data.len_in_bytes = 2;
	rw_shm_data.p_data = &nof_bytes_to_read;
	xgold_pcm->dsp->p_dsp_common_data->ops->set_controls(
			xgold_pcm->dsp,
			DSP_AUDIO_CONTROL_READ_SHM,
			&rw_shm_data);
	rw_shm_data.word_offset =
		xgold_pcm->dsp->p_dsp_common_data->buf_sm_ul_offset;

	/* TODO: does nof_bytes_to_read really mean nof words... ? */
	rw_shm_data.len_in_bytes =
		(2 * nof_bytes_to_read * xrtd->stream->runtime->channels);

	rw_shm_data.p_data = xrtd->hwptr;

	xgold_pcm->dsp->p_dsp_common_data->ops->set_controls(
			xgold_pcm->dsp,
			DSP_AUDIO_CONTROL_READ_SHM,
			&rw_shm_data);

	/* trigger data save for record dump */
	if (xgold_pcm->record_dump == 1) {
		if (xgold_pcm->dump_record_buffer) {
			int nToWrite =
				2 * nof_bytes_to_read *
					xrtd->stream->runtime->channels;
			pData =
			(unsigned short *)(xrtd->stream->runtime->dma_area
				+ xrtd->period_size_bytes * xrtd->hwptr_done);
			if (DUMP_RECORD_SIZE -
				xgold_pcm->dump_record_buffer_pos >
				nToWrite) {
				memcpy(xgold_pcm->dump_record_buffer +
					xgold_pcm->dump_record_buffer_pos,
					(void *)pData,
					nToWrite);
				xgold_pcm->dump_record_buffer_pos += nToWrite;
			} else {
				int nRemain =
					nToWrite -
					(DUMP_RECORD_SIZE -
					xgold_pcm->dump_record_buffer_pos);
				memcpy(xgold_pcm->dump_record_buffer +
					xgold_pcm->dump_record_buffer_pos,
					(void *)pData,
					nToWrite - nRemain);
				memcpy(xgold_pcm->dump_record_buffer,
					(void *)(pData + nToWrite - nRemain),
					nRemain);
				xgold_pcm->dump_record_buffer_pos = nRemain;
				xgold_pcm->record_dump = 2;
				schedule_work(&xgold_pcm->record_dump_work);
			}
		}
	}
	xrtd->total_nof_bytes_read += (2 * nof_bytes_to_read *
		xrtd->stream->runtime->channels);

	if (xrtd->total_nof_bytes_read >= xrtd->period_size_bytes) {
		xrtd->total_nof_bytes_read = 0;
		xrtd->hwptr_done++;
		xrtd->periods++;
		xrtd->periods %= xrtd->stream->runtime->periods;
		xrtd->hwptr_done %= xrtd->stream->runtime->periods;
		snd_pcm_period_elapsed(xrtd->stream);
	}
}

void xgold_dsp_pcm_play_handler(void *dev)
{
	struct xgold_runtime_data *xrtd = (struct xgold_runtime_data *)dev;
	struct xgold_pcm *xgold_pcm;
	unsigned int length = 0;
	unsigned short remaining_block = 0;
	struct dsp_rw_shm_data rw_shm_data;
	int i;
	unsigned int buf_sm_dl_offset = 0;
	unsigned short buffer_mode;
	unsigned short dma_req_interval_time;
	unsigned short buffer_size;

	xgold_debug("%s\n", __func__);

	if (!xrtd) {
		xgold_err("%s: xgold runtime data is NULL!!\n", __func__);
		return;
	}

	xgold_pcm = xrtd->pcm;
	if (!xgold_pcm || !xrtd->stream || !xrtd->stream->runtime ||
			!xrtd->stream->runtime->dma_area) {
		xgold_err("%s: stream data is NULL!!\n", __func__);
		return;
	}

	if (xrtd->stream_type == STREAM_PLAY) {
		rw_shm_data.word_offset =
			xgold_pcm->dsp->p_dsp_common_data->pcm_offset[0];
		buf_sm_dl_offset =
			xgold_pcm->dsp->p_dsp_common_data->buf_sm_dl_offset;
		buffer_mode = xgold_pcm->buffer_mode[0];
		buffer_size = xgold_pcm->buffer_size[0];
		dma_req_interval_time = xgold_pcm->dma_req_interval_time[0];
	} else {
		rw_shm_data.word_offset =
			xgold_pcm->dsp->p_dsp_common_data->pcm_offset[1];
		buf_sm_dl_offset =
			xgold_pcm->dsp->p_dsp_common_data->buf_sm_dl2_offset;
		buffer_mode = xgold_pcm->buffer_mode[1];
		buffer_size = xgold_pcm->buffer_size[1];
		dma_req_interval_time = xgold_pcm->dma_req_interval_time[1];
	}
	rw_shm_data.len_in_bytes = 2;
	rw_shm_data.p_data = &remaining_block;
	xgold_pcm->dsp->p_dsp_common_data->ops->set_controls(
			xgold_pcm->dsp,
			DSP_AUDIO_CONTROL_READ_SHM,
			&rw_shm_data);

	/* write the samples */
	for (i = 0; i < remaining_block; i++) {
		xrtd->hwptr =
			(unsigned short *)(xrtd->stream->runtime->dma_area +
				xrtd->period_size_bytes * xrtd->hwptr_done);
		length = xrtd->stream->runtime->period_size *
			xrtd->stream->runtime->channels;
		rw_shm_data.word_offset = buf_sm_dl_offset;
		rw_shm_data.len_in_bytes = length * 2;
		rw_shm_data.p_data = xrtd->hwptr;
		xgold_pcm->dsp->p_dsp_common_data->ops->set_controls(
				xgold_pcm->dsp,
				DSP_AUDIO_CONTROL_WRITE_SHM,
				&rw_shm_data);

		xrtd->hwptr_done++;
		xrtd->periods++;
		xrtd->periods %= xrtd->stream->runtime->periods;
		xrtd->hwptr_done %= xrtd->stream->runtime->periods;

		dsp_pcm_feed(xgold_pcm->dsp, xrtd->stream_type,
				xrtd->stream->runtime->channels,
				xrtd->stream->runtime->rate,
				buffer_mode,
				dma_req_interval_time,
				buffer_size);
	}
	/*
	 * Period elapsed should be called once even
	 * for multiple buffer feeds
	 */
	snd_pcm_period_elapsed(xrtd->stream);
}

void xgold_dsp_pcm_dma_play_handler(void *dev)
{
	struct xgold_runtime_data *xrtd = (struct xgold_runtime_data *)dev;
	struct xgold_pcm *xgold_pcm;
	dma_addr_t dma_addr;

	xgold_debug("%s\n", __func__);

	if (!xrtd) {
		xgold_err("%s: xgold runtime data is NULL!!\n", __func__);
		return;
	}

	if (xrtd->dma_stop == true) {
		complete(&xrtd->dma_complete);
		xgold_debug("%s: dma complete\n", __func__);
		return;
	}

	xgold_pcm = xrtd->pcm;
	if (!xgold_pcm || !xrtd->stream || !xrtd->stream->runtime ||
			!xrtd->stream->runtime->dma_area) {
		xgold_debug("%s: stream data is NULL!!\n", __func__);
		return;
	}

	if (!xrtd->dmach) {
		xgold_debug("%s: dma channel is NULL\n", __func__);
		return;
	}

	if (xgold_pcm->dma_dump == 1) {
		if (xgold_pcm->dump_dma_buffer) {
			int nToWrite =
				xrtd->period_size_bytes * XGOLD_MAX_SG_LIST;
			dma_addr = xrtd->stream->runtime->dma_addr +
				xrtd->period_size_bytes * xrtd->hwptr_done;
			dma_addr = (dma_addr_t)phys_to_virt(dma_addr);
			if (DUMP_DMA_SIZE - xgold_pcm->dump_dma_buffer_pos >
				nToWrite) {
				memcpy(xgold_pcm->dump_dma_buffer +
					xgold_pcm->dump_dma_buffer_pos,
					(void *)dma_addr,
					nToWrite);
				xgold_pcm->dump_dma_buffer_pos += nToWrite;
			} else {
				int nRemain =
					nToWrite -
					(DUMP_DMA_SIZE -
					xgold_pcm->dump_dma_buffer_pos);
				memcpy(xgold_pcm->dump_dma_buffer +
					xgold_pcm->dump_dma_buffer_pos,
					(void *)dma_addr,
					nToWrite - nRemain);
				memcpy(xgold_pcm->dump_dma_buffer,
					(void *)(dma_addr + nToWrite - nRemain),
					nRemain);
				xgold_pcm->dump_dma_buffer_pos = nRemain;
				xgold_pcm->dma_dump = 2;
				schedule_work(&xgold_pcm->dma_dump_work);
			}
		}
	}

	if (xrtd->stream_type == STREAM_PLAY) {
		xrtd->hwptr_done += xrtd->dma_sgl_count;
		xrtd->periods += xrtd->dma_sgl_count;
	} else {
		xrtd->hwptr_done++;
		xrtd->periods++;
	}
	xrtd->periods %= xrtd->stream->runtime->periods;
	xrtd->hwptr_done %= xrtd->stream->runtime->periods;

	dma_addr = xrtd->stream->runtime->dma_addr +
		xrtd->period_size_bytes * xrtd->hwptr_done;

	/* Period elapsed should be called once even
	 * for multiple buffer feeds */
	snd_pcm_period_elapsed(xrtd->stream);

	/* Reload scatter list and submit DMA request */
	xgold_pcm_dma_play_submit(xrtd, dma_addr);

	/* Restart the DMA tx for next data transfer i.e. after 20 ms */
	dma_async_issue_pending(xrtd->dmach);

	xgold_debug("dma tx started\n");
}

void xgold_dsp_pcm_dma_rec_handler(void *dev)
{
	struct xgold_runtime_data *xrtd = (struct xgold_runtime_data *)dev;
	struct xgold_pcm *xgold_pcm;
	dma_addr_t dma_addr;
	if (!xrtd) {
		xgold_err("%s: xgold runtime data is NULL!!\n", __func__);
		return;
	}

	if (xrtd->dma_stop == true) {
		complete(&xrtd->dma_complete);
		xgold_debug("%s: dma complete\n", __func__);
		return;
	}

	xgold_pcm = xrtd->pcm;
	if (!xgold_pcm || !xrtd->stream || !xrtd->stream->runtime ||
			!xrtd->stream->runtime->dma_area) {
		xgold_debug("%s: stream data is NULL!!\n", __func__);
		return;
	}

	if (!xrtd->dmach) {
		xgold_debug("%s: dma channel is NULL\n", __func__);
		return;
	}
	xgold_debug("<-- %s hwptr_done %d periods %d, runtime periods %d total_nof_bytes_read %d\n",
			__func__,
			xrtd->hwptr_done,
			xrtd->periods,
			xrtd->stream->runtime->periods,
			xrtd->total_nof_bytes_read);

	/* trigger data save for record dump */
	if (xgold_pcm->record_dump == 1) {
		if (xgold_pcm->dump_record_buffer) {
			int nToWrite =
				(xrtd->have_harmonics) ?
				xrtd->period_size_bytes :
				xrtd->period_size_bytes * XGOLD_MAX_SG_LIST;
			dma_addr = xrtd->stream->runtime->dma_addr +
				xrtd->period_size_bytes * xrtd->hwptr_done;
			dma_addr = (dma_addr_t)phys_to_virt(dma_addr);
			if (DUMP_RECORD_SIZE -
				xgold_pcm->dump_record_buffer_pos > nToWrite) {
				memcpy(xgold_pcm->dump_record_buffer +
					xgold_pcm->dump_record_buffer_pos,
					(void *)dma_addr,
					nToWrite);
				xgold_pcm->dump_record_buffer_pos += nToWrite;
			} else {
				int nRemain =
					nToWrite -
					(DUMP_RECORD_SIZE -
					xgold_pcm->dump_record_buffer_pos);
				memcpy(xgold_pcm->dump_record_buffer +
					xgold_pcm->dump_record_buffer_pos,
					(void *)dma_addr,
					nToWrite - nRemain);
				memcpy(xgold_pcm->dump_record_buffer,
					(void *)(dma_addr + nToWrite - nRemain),
					nRemain);
				xgold_pcm->dump_record_buffer_pos = nRemain;
				xgold_pcm->record_dump = 2;
				schedule_work(&xgold_pcm->record_dump_work);
			}
		}
	}

	if (xrtd->total_nof_bytes_read >= xrtd->period_size_bytes) {
		xrtd->total_nof_bytes_read = 0;
		xrtd->hwptr_done += (xrtd->have_harmonics) ?
			1 : xrtd->dma_sgl_count;
		xrtd->periods += (xrtd->have_harmonics) ?
			1 : xrtd->dma_sgl_count;
		xrtd->periods %= xrtd->stream->runtime->periods;
		xrtd->hwptr_done %= xrtd->stream->runtime->periods;
		snd_pcm_period_elapsed(xrtd->stream);
	}


	dma_addr = xrtd->stream->runtime->dma_addr +
		xrtd->total_nof_bytes_read +
		xrtd->period_size_bytes * xrtd->hwptr_done;

	/* Reload scatter list and submit DMA request */
	xgold_pcm_dma_rec_submit(xrtd, dma_addr);

	/* Restart the DMA tx for next data transfer i.e. after 20 ms */
	dma_async_issue_pending(xrtd->dmach);

	xgold_debug("dma rx started\n");
}

static void xgold_pcm_dma_play_submit(struct xgold_runtime_data *xrtd,
		dma_addr_t dma_addr)
{
	struct xgold_pcm *xgold_pcm = xrtd->pcm;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t dma_cookie_tx;
	int i = 0;

	unsigned int dma_bytes = SM_AUDIO_BUFFER_DL_SAMPLES * 4;

	xrtd->period_size_bytes =
		frames_to_bytes(xrtd->stream->runtime,
				xrtd->stream->runtime->period_size);

	xgold_debug("%s - period_size_bytes %d\n",
			__func__,
			xrtd->period_size_bytes);

	/* Prepare the scatter list */
	while (i != xrtd->dma_sgl_count) {
		sg_set_buf(xrtd->dma_sgl + i,
			(void *)phys_to_virt(dma_addr),
			dma_bytes);
		i++;
		dma_addr += dma_bytes;
	}

	dma_map_sg(xgold_pcm->dev, xrtd->dma_sgl, xrtd->dma_sgl_count,
			DMA_TO_DEVICE);

	/* Prepare DMA slave sg */
	desc = dmaengine_prep_slave_sg(xrtd->dmach,
			xrtd->dma_sgl,
			xrtd->dma_sgl_count,
			DMA_MEM_TO_DEV_INCR,
			DMA_PREP_INTERRUPT);

	if (!desc) {
		xgold_err("<-- %s, dmaengine_prep_slave_sg returns NULL\n",
			__func__);
		return;
	}

	/* Set the DMA callback */
	desc->callback = xgold_dsp_pcm_dma_play_handler;
	desc->callback_param = xrtd;

	/* Submit DMA request */
	dma_cookie_tx = dmaengine_submit(desc);

	xgold_debug("<-- %s\n", __func__);
}

static void xgold_pcm_dma_rec_submit(struct xgold_runtime_data *xrtd,
		dma_addr_t dma_addr)
{
	struct xgold_pcm *xgold_pcm = xrtd->pcm;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t dma_cookie_tx;
	int i = 0;
	unsigned int dma_bytes;

	dma_bytes = xrtd->stream->runtime->period_size*
		xrtd->stream->runtime->channels*2;
	if (xrtd->have_harmonics) {
		/* for 44.1kHz sampling rate, and dividers */
		dma_bytes = (dma_bytes >> 4);
		dma_bytes = (dma_bytes << 4) / xrtd->dma_sgl_count;
	}
	xrtd->period_size_bytes =
		frames_to_bytes(xrtd->stream->runtime,
				xrtd->stream->runtime->period_size);
	xrtd->total_nof_bytes_read += xrtd->stream->runtime->period_size *
		xrtd->stream->runtime->channels*2;

	xgold_debug(" %s ,period_size_bytes %d channels %d dma_bytes %d dma_sgl_count%d\n",
			__func__,
			xrtd->period_size_bytes,
			xrtd->stream->runtime->channels,
			dma_bytes,
			xrtd->dma_sgl_count);

	/* Prepare the scatter list */
	while (i < xrtd->dma_sgl_count) {
		int cur_dma_bytes;
		if (xrtd->have_harmonics)
			cur_dma_bytes = (i == (xrtd->dma_sgl_count - 1)) ?
				dma_bytes + 2 *
					xrtd->stream->runtime->channels :
				dma_bytes;
		else
			cur_dma_bytes = dma_bytes;

		xgold_debug(" %d - period_size_bytes %d\n", i, cur_dma_bytes);

		sg_set_buf(xrtd->dma_sgl + i,
			(void *)phys_to_virt(dma_addr),
			cur_dma_bytes);
		i++;
		dma_addr += cur_dma_bytes;
	}

	dma_map_sg(xgold_pcm->dev, xrtd->dma_sgl, xrtd->dma_sgl_count,
			DMA_FROM_DEVICE);

	/* Prepare DMA slave sg */
	desc = dmaengine_prep_slave_sg(xrtd->dmach,
			xrtd->dma_sgl,
			xrtd->dma_sgl_count,
			DMA_DEV_INCR_TO_MEM,
			DMA_PREP_INTERRUPT);

	if (!desc) {
		xgold_err("<-- %s, dmaengine_prep_slave_sg returns NULL\n",
			__func__);
		return;
	}

	/* Set the DMA callback */
	desc->callback = xgold_dsp_pcm_dma_rec_handler;
	desc->callback_param = xrtd;

	/* Submit DMA request */
	dma_cookie_tx = dmaengine_submit(desc);

	xgold_debug("<-- %s\n", __func__);
}

static struct snd_soc_dai_driver xgold_dai_shm = {
	.name = "XGOLD_PCM",
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
	struct xgold_runtime_data *xrtd;

	char *substream_id = substream->pcm->id;
	struct xgold_pcm *xgold_pcm =
		snd_soc_platform_get_drvdata(rtd->platform);

	int ret = 0;
	bool power_state = ON;

	xgold_debug("--> %s\n", __func__);

	xgold_debug("%s: Requesting to power on dsp\n", __func__);
	ret = xgold_pcm->dsp->p_dsp_common_data->
		ops->set_controls(xgold_pcm->dsp,
			DSP_AUDIO_POWER_REQ, &power_state);

	if (ret < 0) {
		xgold_err("%s : power request failed %d\n",
			__func__, ret);
		return ret;
	}

	xrtd = kzalloc(sizeof(struct xgold_runtime_data), GFP_KERNEL);
	if (!xrtd)
		return -ENOMEM;

	if (strstr(substream_id, "XGOLD_HW_PROBE_A"))
		xrtd->stream_type = HW_PROBE_A;
	else if (strstr(substream_id, "XGOLD_HW_PROBE_B"))
		xrtd->stream_type = HW_PROBE_B;
	else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		if (strstr(substream_id, "PCM Audio 2"))
			xrtd->stream_type = STREAM_PLAY2;
		else
			xrtd->stream_type = STREAM_PLAY;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		xrtd->stream_type = STREAM_REC;
	else {
		ret = -EINVAL;
		goto out;
	}

	xgold_debug("stream type %d\n", xrtd->stream_type);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		snd_soc_set_runtime_hwparams(substream, &xgold_pcm_play_cfg);

		/* Only extraction is supported for hw_probes */
		if (xrtd->stream_type == HW_PROBE_B ||
				xrtd->stream_type == HW_PROBE_A) {
			ret = -EINVAL;
			goto out;
		}

		if (!xgold_pcm->play_dma_mode) {
			if (xrtd->stream_type == STREAM_PLAY)
				register_dsp_audio_lisr_cb(
					DSP_LISR_CB_PCM_PLAYER,
					xgold_dsp_pcm_play_handler,
					(void *)xrtd);
			else
				register_dsp_audio_lisr_cb(
					DSP_LISR_CB_PCM_PLAYER_A,
					xgold_dsp_pcm_play_handler,
					(void *)xrtd);
		}
	} else {
		snd_soc_set_runtime_hwparams(substream, &xgold_pcm_record_cfg);

		/* HW probe only extraction is supported*/
		if (xrtd->stream_type == HW_PROBE_A) {
			xgold_debug("registering hw_probe_a callback\n");
			register_dsp_audio_lisr_cb(
					DSP_LISR_CB_HW_PROBE_A,
					xgold_dsp_hw_probe_a_handler,
					(void *)xrtd);
		} else if (xrtd->stream_type == HW_PROBE_B) {
			xgold_debug("registering hw_probe_b callback\n");
			register_dsp_audio_lisr_cb(
					DSP_LISR_CB_HW_PROBE_B,
					xgold_dsp_hw_probe_b_handler,
					(void *)xrtd);
		} else {
			if (!xgold_pcm->rec_dma_mode) {
				register_dsp_audio_lisr_cb(
					DSP_LISR_CB_PCM_RECORDER,
					xgold_dsp_pcm_rec_handler,
					(void *)xrtd);
			}
		}
	}

	/* Make sure, that the period size is always even */
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_PERIODS, 2);

	if (ret < 0) {
		xgold_debug("Failed to set period size\n");
		goto out;
	}

	ret = snd_pcm_hw_constraint_integer(
		runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	if (ret < 0) {
		xgold_debug("Failed to set buffer size\n");
		goto out;
	}

	xrtd->pcm = xgold_pcm;
	runtime->private_data = xrtd;

out:
	xgold_debug("<-- %s: %d\n", __func__, ret);

	if (ret < 0)
		kfree(xrtd);

	return ret;
}

static int xgold_pcm_close(struct snd_pcm_substream *substream)
{
	struct xgold_runtime_data *xrtd = substream->runtime->private_data;
	struct xgold_pcm *xgold_pcm;
	int ret = 0;
	bool power_state = OFF;

	xgold_debug("XGOLD Closing pcm device\n");

	if (!xrtd) {
		xgold_err("Runtime data is NULL.\n");
		return 0;
	}

	xgold_pcm = xrtd->pcm;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (!xgold_pcm->play_dma_mode) {
			if (xrtd->stream_type == STREAM_PLAY)
				register_dsp_audio_lisr_cb(
					DSP_LISR_CB_PCM_PLAYER,
					NULL,
					NULL);
			else
				register_dsp_audio_lisr_cb(
					DSP_LISR_CB_PCM_PLAYER_A,
					NULL,
					NULL);
		}
	} else if (xrtd->stream_type == HW_PROBE_A)
		register_dsp_audio_lisr_cb(
			DSP_LISR_CB_HW_PROBE_A,
			NULL,
			NULL);
	else if (xrtd->stream_type == HW_PROBE_B)
		register_dsp_audio_lisr_cb(
			DSP_LISR_CB_HW_PROBE_B,
			NULL,
			NULL);
	else if (xrtd->stream_type == STREAM_REC && !xgold_pcm->rec_dma_mode) {
		register_dsp_audio_lisr_cb(
			DSP_LISR_CB_PCM_RECORDER,
			NULL,
			NULL);
		}
	else {
		xgold_err("Unsupported stream type %d\n", xrtd->stream_type);
		ret = -EINVAL;
	}

	kfree(xrtd);

	xgold_debug("%s: Requesting to suspend dsp\n", __func__);
	ret = xgold_pcm->dsp->p_dsp_common_data->
		ops->set_controls(xgold_pcm->dsp,
			DSP_AUDIO_POWER_REQ, &power_state);

	return ret;
}

static int xgold_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct xgold_runtime_data *xrtd = substream->runtime->private_data;
	struct xgold_pcm *xgold_pcm = xrtd->pcm;
	int ret;

	xgold_debug("%s\n", __func__);

	if ((xgold_pcm->play_dma_mode &&
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ||
		(xgold_pcm->rec_dma_mode && substream->stream ==
			SNDRV_PCM_STREAM_CAPTURE))
			substream->dma_buffer.dev.type = SNDRV_DMA_TYPE_DEV;
		else
			substream->dma_buffer.dev.type =
				SNDRV_DMA_TYPE_CONTINUOUS;

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
	struct xgold_runtime_data *xrtd = substream->runtime->private_data;
	struct xgold_pcm *xgold_pcm;

	xgold_debug("%s\n", __func__);

	if (!xrtd || !xrtd->dmach)
		return snd_pcm_lib_free_pages(substream);

	xgold_pcm = xrtd->pcm;

	if ((xgold_pcm->play_dma_mode && substream->stream ==
			SNDRV_PCM_STREAM_PLAYBACK) ||
		(xgold_pcm->rec_dma_mode && substream->stream ==
			SNDRV_PCM_STREAM_CAPTURE)) {
		int ret = wait_for_completion_timeout(&xrtd->dma_complete,
				msecs_to_jiffies(120));
		if (ret == 0)
			xgold_debug("%s: dma completion timeout\n", __func__);

		/* request DMA shutdown */
		xgold_debug("terminate all dma: %p\n", xrtd->dmach);
		dmaengine_terminate_all(xrtd->dmach);

		/* Release the DMA channel */
		dma_release_channel(xrtd->dmach);
		xrtd->dmach = NULL;

		/* Free scatter list memory*/
		kfree(xrtd->dma_sgl);
	}

	/* Free DMA buffer */
	return snd_pcm_lib_free_pages(substream);
}

static int xgold_pcm_play_dma_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct xgold_runtime_data *xrtd = runtime->private_data;
	struct xgold_pcm *xgold_pcm = xrtd->pcm;
	struct dma_slave_config pcm_dma_config;
	dma_addr_t shm_base, dma_addr;
	int ret = 0;
#ifndef CONFIG_OF
	dma_cap_mask_t tx_mask;
#endif
	unsigned short shm_samples = SM_AUDIO_BUFFER_DL_SAMPLES;

	if ((runtime->period_size % shm_samples) != 0) {
		xgold_err("%s: invalid period_size = %d\n", __func__,
			(int)runtime->period_size);
		return -EINVAL;
	}

#ifdef CONFIG_OF
	xrtd->dmach = xgold_of_dsp_get_dmach(xgold_pcm->dsp, xrtd->stream_type);
#else
	dma_cap_zero(tx_mask);
	dma_cap_set(DMA_SLAVE, tx_mask);

	xrtd->dmach = dma_request_channel(tx_mask, pl08x_filter_id,
			(void *)"dsp_dma_req1");
#endif

	if (!xrtd->dmach) {
		xgold_err("%s: dma channel req fail\n", __func__);
		return -EIO;
	}

	xrtd->dma_stop = false;
	init_completion(&xrtd->dma_complete);

	if (xrtd->stream_type == STREAM_PLAY)
		shm_base = dsp_get_audio_shmem_base_addr(xgold_pcm->dsp) +
		xgold_pcm->dsp->p_dsp_common_data->buf_sm_dl_offset * 2;
	else
		shm_base = dsp_get_audio_shmem_base_addr(xgold_pcm->dsp) +
		xgold_pcm->dsp->p_dsp_common_data->buf_sm_dl2_offset * 2;

	/* Config DMA slave parameters */
	pcm_dma_config.direction = DMA_TO_DEVICE;
	pcm_dma_config.dst_addr = shm_base;
	pcm_dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	pcm_dma_config.dst_maxburst = DMA_BURST_SIZE;
	pcm_dma_config.device_fc = false;

	ret = dmaengine_slave_config(xrtd->dmach,
					&pcm_dma_config);
	if (ret) {
		xgold_debug("pcm: error in dma slave configuration\n");
		return ret;
	}

	dma_addr = runtime->dma_addr;

	xrtd->dma_sgl = kzalloc(sizeof(struct scatterlist) *
			xrtd->dma_sgl_count, GFP_KERNEL);

	if (!xrtd->dma_sgl)
		return -ENOMEM;

	sg_init_table(xrtd->dma_sgl, xrtd->dma_sgl_count);

	/* Load scatter list and submit DMA request */
	xgold_pcm_dma_play_submit(xrtd, dma_addr);

	return 0;
}

static int xgold_pcm_rec_dma_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct xgold_runtime_data *xrtd = runtime->private_data;
	struct xgold_pcm *xgold_pcm = xrtd->pcm;
	struct dma_slave_config pcm_dma_config;
	dma_addr_t shm_base, dma_addr;
	int ret = 0;
#ifndef CONFIG_OF
	dma_cap_mask_t tx_mask;
#endif

	xgold_debug("%s period_size = %d\n", __func__,
			(int)runtime->period_size);

#ifdef CONFIG_OF
	xrtd->dmach = xgold_of_dsp_get_dmach(xgold_pcm->dsp, xrtd->stream_type);
#else
	dma_cap_zero(tx_mask);
	dma_cap_set(DMA_SLAVE, tx_mask);

	xrtd->dmach = dma_request_channel(tx_mask, pl08x_filter_id,
			(void *)"dsp_dma_req3");
#endif

	if (!xrtd->dmach) {
		xgold_err("%s: dma channel req fail\n", __func__);
		return -EIO;
	}

	xrtd->dma_stop = false;
	init_completion(&xrtd->dma_complete);

	if (xrtd->stream->runtime->period_size % 2)
		/* for 44.1kHz sampling rate, and dividers */
		xrtd->have_harmonics = true;

	shm_base = dsp_get_audio_shmem_base_addr(xgold_pcm->dsp) +
	xgold_pcm->dsp->p_dsp_common_data->buf_sm_ul_offset * 2;

	/* Config DMA slave parameters */
	pcm_dma_config.direction = DMA_FROM_DEVICE;
	pcm_dma_config.src_addr = shm_base;
	pcm_dma_config.src_addr_width =
		(xrtd->have_harmonics && xrtd->stream->runtime->channels == 1) ?
		DMA_SLAVE_BUSWIDTH_2_BYTES : DMA_SLAVE_BUSWIDTH_4_BYTES;
	pcm_dma_config.src_maxburst = DMA_BURST_SIZE;
	pcm_dma_config.device_fc = false;

	ret = dmaengine_slave_config(xrtd->dmach,
					&pcm_dma_config);
	if (ret) {
		xgold_debug("pcm: error in dma slave configuration\n");
		return ret;
	}

	dma_addr = runtime->dma_addr;
	if (!xrtd->have_harmonics)
		xrtd->dma_sgl_count = XGOLD_MAX_SG_LIST;
	else {
		switch (substream->runtime->rate) {
		case 44100:
			xrtd->dma_sgl_count = 2;
			break;
		case 22050:
			xrtd->dma_sgl_count = 4;
			break;
		case 11025:
			xrtd->dma_sgl_count = 8;
			break;
		default:
			return -EINVAL;
		}
	}

	xrtd->dma_sgl = kzalloc(sizeof(struct scatterlist) *
			xrtd->dma_sgl_count, GFP_KERNEL);

	if (!xrtd->dma_sgl)
		return -ENOMEM;

	sg_init_table(xrtd->dma_sgl, xrtd->dma_sgl_count);

	/* Load scatter list and submit DMA request */
	xgold_pcm_dma_rec_submit(xrtd, dma_addr);

	return 0;
}

static int xgold_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct xgold_runtime_data *xrtd = substream->runtime->private_data;
	struct xgold_pcm *xgold_pcm = xrtd->pcm;
	unsigned short shm_samples = SM_AUDIO_BUFFER_DL_SAMPLES;
	int ret = 0;

	xgold_debug("%s\n", __func__);

	xrtd->hwptr_done = 0;
	xrtd->periods = 0;
	xrtd->stream = substream;

	xrtd->dma_sgl_count = substream->runtime->period_size / shm_samples;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Normal mode      - ALSA Ring buffer size =
					psize * pcnt = 5ms*8  - 40ms */
		/* Deep buffer mode - ALSA Ring buffer size =
					psize * pcnt = 80ms*2 - 160ms */
		xgold_debug("%s(): playback_mode pcm1:%d pcm2:%d\n", __func__,
		xgold_pcm->playback_mode[0], xgold_pcm->playback_mode[1]);
		if (xgold_pcm->play_dma_mode) {
			if (xrtd->stream_type == STREAM_PLAY) {
				/* Burst mode */
				if (BURST == xgold_pcm->playback_mode[0]) {
					xgold_pcm->buffer_mode[0] = BURST_MODE;
					xgold_pcm->buffer_size[0] =
							BURST_BUFFER_SIZE;
					xgold_pcm->dma_req_interval_time[0] =
							BURST_DMA_INTERVAL;
				} else {	/* Normal mode */
					xgold_pcm->buffer_mode[0] = NORMAL_MODE;
					xgold_pcm->buffer_size[0] =
							NORMAL_BUFFER_SIZE;
					xgold_pcm->dma_req_interval_time[0] =
							NORMAL_DMA_INTERVAL;
				}
				xrtd->dma_sgl_count *= XGOLD_MAX_SG_LIST;
			} else {
				/* Burst mode */
				if (BURST == xgold_pcm->playback_mode[1]) {
					xgold_pcm->buffer_mode[1] = BURST_MODE;
					xgold_pcm->buffer_size[1] =
							BURST_BUFFER_SIZE;
					xgold_pcm->dma_req_interval_time[1] =
							BURST_DMA_INTERVAL;
				} else {	/* Normal mode */
					xgold_pcm->buffer_mode[1] =
							NORMAL_MODE;
					xgold_pcm->buffer_size[1] =
							NORMAL_BUFFER_SIZE;
					xgold_pcm->dma_req_interval_time[1] =
							NORMAL_DMA_INTERVAL;
				}
			}
			ret = xgold_pcm_play_dma_prepare(substream);
		} else {
			/* Configure to NORMAL mode in Interrupt mode */
			if (xrtd->stream_type == STREAM_PLAY) {
				xgold_pcm->buffer_mode[0] = NORMAL_MODE;
				xgold_pcm->buffer_size[0] = NORMAL_BUFFER_SIZE;
				xgold_pcm->dma_req_interval_time[0] =
							NORMAL_DMA_INTERVAL;
			} else {
				xgold_pcm->buffer_mode[1] = NORMAL_MODE;
				xgold_pcm->buffer_size[1] = NORMAL_BUFFER_SIZE;
				xgold_pcm->dma_req_interval_time[1] =
							NORMAL_DMA_INTERVAL;
			}
		}
		xgold_debug("%s(): buffer_mode pcm1:%d pcm2:%d\n", __func__,
			xgold_pcm->buffer_mode[0], xgold_pcm->buffer_mode[1]);

		if (audio_native_mode)
			setup_pcm_play_path(xgold_pcm);
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (xgold_pcm->rec_dma_mode)
			ret = xgold_pcm_rec_dma_prepare(substream);
		if (ret < 0) {
			xgold_err("%s : rec dma prepare failed %d\n",
			__func__, ret);
			return ret;
		}
		if (audio_native_mode)
			setup_pcm_record_path(xgold_pcm);
	}

	return ret;
}

static int xgold_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct xgold_runtime_data *xrtd = substream->runtime->private_data;
	struct xgold_pcm *xgold_pcm = xrtd->pcm;
	struct dsp_audio_device *dsp = xgold_pcm->dsp;

	xgold_debug("%s type %d\n", __func__, substream->stream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		xgold_debug("%s: Trigger Start\n", __func__);
		xgold_debug("period size %ld, periods %d buffer size %ld\n",
				substream->runtime->period_size,
				substream->runtime->periods,
				substream->runtime->buffer_size);
		xgold_debug("rate %d channels %d\n",
				substream->runtime->rate,
				substream->runtime->channels);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			xrtd->period_size_bytes =
				frames_to_bytes(xrtd->stream->runtime,
					xrtd->stream->runtime->period_size);

			if (xgold_pcm->play_dma_mode)
				/* request DMA to start tx */
				dma_async_issue_pending(xrtd->dmach);

			if (STREAM_PLAY == xrtd->stream_type) {
				dsp_pcm_play(dsp, xrtd->stream_type,
					substream->runtime->channels,
					substream->runtime->rate,
					xgold_pcm->play_dma_mode,
					xgold_pcm->buffer_mode[0],
					xgold_pcm->dma_req_interval_time[0],
					xgold_pcm->buffer_size[0]);
			} else {
				dsp_pcm_play(dsp, xrtd->stream_type,
					substream->runtime->channels,
					substream->runtime->rate,
					xgold_pcm->play_dma_mode,
					xgold_pcm->buffer_mode[1],
					xgold_pcm->dma_req_interval_time[1],
					xgold_pcm->buffer_size[1]);
			}
		} else if (xrtd->stream_type == HW_PROBE_A) {
			xrtd->period_size_bytes = frames_to_bytes(
					xrtd->stream->runtime,
					xrtd->stream->runtime->period_size);
			xrtd->pcm->
				hw_probe_status[HW_PROBE_POINT_A].active = true;
		} else if (xrtd->stream_type == HW_PROBE_B) {
			xrtd->period_size_bytes = frames_to_bytes(
					xrtd->stream->runtime,
					xrtd->stream->runtime->period_size);
			xrtd->pcm->
				hw_probe_status[HW_PROBE_POINT_B].active = true;
		} else {
			xrtd->period_size_bytes = frames_to_bytes(
					xrtd->stream->runtime,
					xrtd->stream->runtime->period_size);

			if (xgold_pcm->rec_dma_mode)
				/* request DMA to start rx */
				dma_async_issue_pending(xrtd->dmach);

			dsp_pcm_rec(dsp, substream->runtime->channels,
					substream->runtime->rate,
					xgold_pcm->rec_dma_mode,
					xgold_pcm->path_select);
		}
		/* call hw probe function with runtime as parameter, and
		   the function will enable/disable HW probe as requested */
		dsp_cmd_hw_probe(xgold_pcm->dsp, xrtd);

		/* HW_AFE should be sent after audio codec is powered up */
		if (xgold_pcm_sysfs_attribute_value ==
				XGOLD_PCM_ATTR_SEND_DSP_CMD &&
				xrtd->stream_type != HW_PROBE_A &&
				xrtd->stream_type != HW_PROBE_B)
			dsp_start_audio_hwafe();
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		xgold_debug("%s: Trigger stop\n", __func__);

		if (xgold_pcm->play_dma_mode || xgold_pcm->rec_dma_mode)
			xrtd->dma_stop = true;

		dsp_pcm_stop(dsp, xrtd->stream_type);
		xgold_debug("DSP stopped\n");

		if (xrtd->stream_type == HW_PROBE_A)
			xrtd->pcm->
			hw_probe_status[HW_PROBE_POINT_A].active = false;
		else if (xrtd->stream_type == HW_PROBE_B)
			xrtd->pcm->
			hw_probe_status[HW_PROBE_POINT_B].active = false;

		/* call hw probe function with runtime as parameter, and
		   the function will enable/disable HW probe as requested */
		dsp_cmd_hw_probe(xgold_pcm->dsp, xrtd);

		/* HW_AFE should be switched off before audio codec power
		 * down */
		if (xgold_pcm_sysfs_attribute_value ==
				XGOLD_PCM_ATTR_SEND_DSP_CMD &&
				xrtd->stream_type != HW_PROBE_A &&
				xrtd->stream_type != HW_PROBE_B)
			dsp_stop_audio_hwafe();
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		xgold_err("%s: Trigger pause\n", __func__);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		xgold_err("%s: Trigger pause release\n", __func__);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t xgold_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct xgold_runtime_data *xrtd = substream->runtime->private_data;
	unsigned int offset;

	offset = xrtd->periods * frames_to_bytes(substream->runtime,
			substream->runtime->period_size);

	return bytes_to_frames(substream->runtime, offset);
}

static u64 xgold_pcm_dmamask = DMA_BIT_MASK(32);

static int xgold_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct xgold_pcm *xgold_pcm =
		snd_soc_platform_get_drvdata(rtd->platform);
	int ret = 0;
	xgold_debug("%s\n", __func__);

	if (xgold_pcm->play_dma_mode || xgold_pcm->rec_dma_mode) {
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

static ssize_t xgold_pcm_attribute_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	size_t size_copied;
	u16 value;
	value = xgold_pcm_sysfs_attribute_value;
	size_copied = sprintf(buf, "%d\n", value);
	return size_copied;
}

static ssize_t xgold_pcm_attribute_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	int sysfs_val;
	int ret;
	size_t size_to_cpy;
	char strvalue[SYSFS_INPUT_VAL_LEN + 1];
	size_to_cpy = (count > SYSFS_INPUT_VAL_LEN)
		? SYSFS_INPUT_VAL_LEN : count;
	strncpy(strvalue, buf, size_to_cpy);
	strvalue[size_to_cpy] = '\0';
	ret = kstrtoint(strvalue, 10, &sysfs_val);
	if (ret != 0)
		return ret;
	xgold_pcm_sysfs_attribute_value = sysfs_val;
	xgold_debug("sysfs attr %s=%d\n", attr->attr.name, sysfs_val);
	return count;
}

static DEVICE_ATTR(xgold_pcm_sysfs_attribute, S_IRUSR | S_IWUSR,
		xgold_pcm_attribute_show,
		xgold_pcm_attribute_store);

static struct attribute *xgold_pcm_attributes[] = {
	&dev_attr_xgold_pcm_sysfs_attribute.attr,
	NULL,
};

static const struct attribute_group xgold_pcm_attr_group = {
	.attrs = xgold_pcm_attributes,
};

static void xgold_pcm_register_sysfs_attr(struct device *dev)
{
	int err = sysfs_create_group(&dev->kobj, &xgold_pcm_attr_group);
	if (err)
		pr_err("Unable to create sysfs entry: '%s'\n",
			dev_attr_xgold_pcm_sysfs_attribute.attr.name);
}

/* Get function for the pcm rec path select control */
static int xgold_pcm_rec_path_sel_ctl_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);

	xgold_debug("%s - get value %d\n",
		__func__, (int)xgold_pcm->path_select);
	ucontrol->value.integer.value[0] = (int)xgold_pcm->path_select;
	return 0;
}

/* Set function for the pcm rec path select control */
static int xgold_pcm_rec_path_sel_ctl_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);

	xgold_debug("%s - set value %d\n",
		__func__, (int)ucontrol->value.integer.value[0]);
	xgold_pcm->path_select =
		(unsigned int)ucontrol->value.integer.value[0];
	return 0;
}

/* Mixer Control to Select PCM2 - Playback mode (NORMAL/BURST) */
static int xgold_pcm2_playback_mode_sel_ctl_info(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	xgold_debug("%s\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

/* Get function for the PCM2 playback mode select control */
static int xgold_pcm2_playback_mode_sel_ctl_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);

	xgold_debug("%s - get value PCM2 playback_mode:%d\n", __func__,
		(unsigned int)(xgold_pcm->buffer_mode[1] ? BURST : NORMAL));
	ucontrol->value.integer.value[0] =
		(unsigned int)(xgold_pcm->buffer_mode[1] ? BURST : NORMAL);
	return 0;
}

/* Set function for the PCM2 playback mode select control */
static int xgold_pcm2_playback_mode_sel_ctl_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);

	xgold_debug("%s - set value PCM2 playback_mode:%d\n", __func__,
	(unsigned int)(ucontrol->value.integer.value[0] ? BURST : NORMAL));

	xgold_pcm->playback_mode[1] =
	(unsigned int)(ucontrol->value.integer.value[0] ? BURST : NORMAL);

	return 0;
}

/* RECORD DUMP */
static void record_dump_handler(struct work_struct *work)
{
	struct file *filp = NULL;
	mm_segment_t old_fs;
	struct xgold_pcm *xgold_pcm =
		container_of(work, struct xgold_pcm, record_dump_work);

	xgold_debug("%s\n", __func__);

	if (xgold_pcm->record_dump == 2) {
		filp = filp_open("/data/record_dump.pcm",
			O_CREAT|O_RDWR|O_TRUNC, 0600);
		if (filp) {
			old_fs = get_fs();
			set_fs(get_ds());

			filp->f_op->write(
				filp,
				xgold_pcm->dump_record_buffer +
				xgold_pcm->dump_record_buffer_pos,
				DUMP_RECORD_SIZE -
				xgold_pcm->dump_record_buffer_pos,
				&filp->f_pos);
			if (xgold_pcm->dump_record_buffer_pos != 0)
				filp->f_op->write(
				filp,
				xgold_pcm->dump_record_buffer,
				xgold_pcm->dump_record_buffer_pos,
				&filp->f_pos);

			set_fs(old_fs);

			filp_close(filp, NULL);
	  } else {
			xgold_err("%s, Create file failtrue!!\n", __func__);
		}
	} else {
		xgold_err("%s, Should not be here\n", __func__);
	}

	xgold_pcm->record_dump = 0;
}

static int xgold_pcm_record_dump_sel_ctl_info(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	xgold_debug("%s\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int xgold_pcm_record_dump_sel_ctl_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);
	xgold_debug("%s - get value %d\n",
		__func__, (int)xgold_pcm->record_dump);
	ucontrol->value.integer.value[0] = (int)xgold_pcm->record_dump;
	return 0;
}

/* Set function for the pcm rec dump select control */
static int xgold_pcm_record_dump_sel_ctl_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);

	xgold_debug("%s - set value %d\n",
		__func__, (int)ucontrol->value.integer.value[0]);

	if ((int)xgold_pcm->record_dump == 0 &&
		(unsigned int)ucontrol->value.integer.value[0] != 0) {
		/* Allocate memory to stroe record data */
		if (xgold_pcm->dump_record_buffer == NULL) {
			xgold_pcm->dump_record_buffer =
				vmalloc(DUMP_RECORD_SIZE);
			if (xgold_pcm->dump_record_buffer == NULL) {
				xgold_err("DUMP memory cannot be allocated\n");
				return -ENOMEM;
			} else {
				memset(xgold_pcm->dump_record_buffer,
				0, DUMP_RECORD_SIZE);
				/* Add work for dumping record data*/
				INIT_WORK(&xgold_pcm->record_dump_work,
				record_dump_handler);
				xgold_debug("DUMP record memory thread scheduled\n");
			}
		}

		if (xgold_pcm->dump_record_buffer != NULL)
			xgold_pcm->record_dump = 1;
	}
	return 0;
}
/* DMA DUMP */
static void dma_dump_handler(struct work_struct *work)
{
	struct file *filp = NULL;
	mm_segment_t old_fs;
	struct xgold_pcm *xgold_pcm =
		container_of(work, struct xgold_pcm, dma_dump_work);

	xgold_debug("%s\n", __func__);

	if (xgold_pcm->dma_dump == 2) {
		filp = filp_open("/data/dma_dump.pcm",
			O_CREAT|O_RDWR|O_TRUNC, 0600);
		if (filp) {
			old_fs = get_fs();
			set_fs(get_ds());

			filp->f_op->write(
				filp,
				xgold_pcm->dump_dma_buffer +
				xgold_pcm->dump_dma_buffer_pos,
				DUMP_DMA_SIZE - xgold_pcm->dump_dma_buffer_pos,
				&filp->f_pos);
			if (xgold_pcm->dump_dma_buffer_pos != 0)
				filp->f_op->write(
				filp,
				xgold_pcm->dump_dma_buffer,
				xgold_pcm->dump_dma_buffer_pos,
				&filp->f_pos);

			set_fs(old_fs);

			filp_close(filp, NULL);
	  } else {
			xgold_err("%s, Create file failtrue!!\n", __func__);
		}
	} else {
		xgold_err("%s, Should not be here\n", __func__);
	}

	xgold_pcm->dma_dump = 0;
}

static int xgold_pcm_dma_dump_sel_ctl_info(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	xgold_debug("%s\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int xgold_pcm_dma_dump_sel_ctl_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);
	xgold_debug("%s - get value %d\n",
		__func__, (int)xgold_pcm->dma_dump);
	ucontrol->value.integer.value[0] = (int)xgold_pcm->dma_dump;
	return 0;
}

/* Set function for the pcm rec path select control */
static int xgold_pcm_dma_dump_sel_ctl_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);

	xgold_debug("%s - set value %d\n",
		__func__, (int)ucontrol->value.integer.value[0]);

	if (xgold_pcm->play_dma_mode &&
		(int)xgold_pcm->dma_dump == 0 &&
		(unsigned int)ucontrol->value.integer.value[0] != 0) {
		/* Allocate memory to stroe DMA data */
		if (xgold_pcm->dump_dma_buffer == NULL) {
			xgold_pcm->dump_dma_buffer = vmalloc(DUMP_DMA_SIZE);
			if (xgold_pcm->dump_dma_buffer == NULL) {
				xgold_err("DUMP memory cannot be allocated\n");
			} else {
				memset(xgold_pcm->dump_dma_buffer,
				0, DUMP_DMA_SIZE);
				/* Add work for dumping dma data*/
				INIT_WORK(&xgold_pcm->dma_dump_work,
				dma_dump_handler);
			}
		}

		if (xgold_pcm->dump_dma_buffer != NULL)
			xgold_pcm->dma_dump = 1;
	}
	return 0;
}

/* Info function for HW probe SHMEM / io point mapping */
int xgold_pcm_hw_probe_ctl_info(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	xgold_debug("%s :\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = XGOLD_MIN_HW_PROBE_SELECT;

	/* TODO: Also check DSP id here... */
	uinfo->value.integer.max = XGOLD_MAX_HW_PROBE_SELECT;
	return 0;
}

/* Get function for HW probe SHMEM / io point mapping */
static int xgold_pcm_hw_probe_ctl_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);

	int probe_point_id =
		HW_PROBE_DEVICE_TO_PROBE_POINT_ID(ucontrol->id.device);

	/* range check probe device id defined in
	   xgold_speech_probe_controls[] */
	if (probe_point_id >= HW_PROBE_POINT_END) {
		xgold_err("device: %d is out of range\n", probe_point_id);
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = (long int)xgold_pcm->
		hw_probe_status[probe_point_id].hw_probe_sel;

	xgold_debug("%s - get value: %d for device: %d\n",
		__func__, (int)ucontrol->value.integer.value[0],
		probe_point_id);

	return 0;
}

/* Set function for HW probe SHMEM / io point mapping */
static int xgold_pcm_hw_probe_ctl_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct xgold_pcm *xgold_pcm = snd_soc_dai_get_drvdata(cpu_dai);

	int probe_point_id =
		HW_PROBE_DEVICE_TO_PROBE_POINT_ID(ucontrol->id.device);

	unsigned int probe_sel_req =
		(unsigned int)ucontrol->value.integer.value[0];

	struct xgold_pcm_hw_probe_status *hwp_status =
		&xgold_pcm->hw_probe_status[probe_point_id];

	/* renge check requested probe point */
	if (!is_hw_probe_point_valid(xgold_pcm->dsp->id, probe_sel_req)) {
		xgold_err("requested probe point: %d is out of range\n",
			probe_sel_req);
		return -EINVAL;
	}

	/* range check probe device id defined in
	   xgold_speech_probe_controls[] */
	if (probe_point_id >= HW_PROBE_POINT_END) {
		xgold_err("device: %d is out of range\n", probe_point_id);
		return -EINVAL;
	}

	/* if hw probe is active, don't allow to change the probe select */
	if ((true == hwp_status->active) &&
		(probe_sel_req != hwp_status->hw_probe_sel)) {
			xgold_err("fail: %d, hwp_status state: %s\n",
				probe_sel_req,
				(hwp_status->active ? "ACTIVE" : "IDLE"));
			return -EPERM;
	}

	xgold_pcm->hw_probe_status[probe_point_id].hw_probe_sel =
		probe_sel_req;

	xgold_debug("%s - set value: %d for device: %d\n",
		__func__, probe_sel_req, probe_point_id);
	return 0;
}

/* Soc xgold pcm controls */
static const struct snd_kcontrol_new xgold_pcm_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.device = 0,
		.name = "PCM Rec path select",
		.info = xgold_pcm_rec_path_sel_ctl_info,
		.get = xgold_pcm_rec_path_sel_ctl_get,
		.put = xgold_pcm_rec_path_sel_ctl_set,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "PCM2 (Deep Buffer) Playback Burst mode",
		.info = xgold_pcm2_playback_mode_sel_ctl_info,
		.get = xgold_pcm2_playback_mode_sel_ctl_get,
		.put = xgold_pcm2_playback_mode_sel_ctl_set,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Start RECORD dump",
		.info = xgold_pcm_record_dump_sel_ctl_info,
		.get = xgold_pcm_record_dump_sel_ctl_get,
		.put = xgold_pcm_record_dump_sel_ctl_set,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Start DMA dump",
		.info = xgold_pcm_dma_dump_sel_ctl_info,
		.get = xgold_pcm_dma_dump_sel_ctl_get,
		.put = xgold_pcm_dma_dump_sel_ctl_set,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.device = 0 + XGOLD_HW_PROBE_DEVICE_OFSET,
		.name = "PCM HW probe 1",
		.info = xgold_pcm_hw_probe_ctl_info,
		.get = xgold_pcm_hw_probe_ctl_get,
		.put = xgold_pcm_hw_probe_ctl_set,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.device = 1 + XGOLD_HW_PROBE_DEVICE_OFSET,
		.name = "PCM HW probe 2",
		.info = xgold_pcm_hw_probe_ctl_info,
		.get = xgold_pcm_hw_probe_ctl_get,
		.put = xgold_pcm_hw_probe_ctl_set,
	},
};

static int xgold_pcm_soc_probe(struct snd_soc_platform *platform)
{
	int ret = 0;
	xgold_debug("%s\n", __func__);
	dsp_audio_platform_init(platform);

	if (bt_init_en)
		ret = xgold_bt_sco_soc_init(platform);

	if (ret < 0) {
		xgold_err("%s: Unable to initialize xgold bt sco soc\n",
		__func__);
		return ret;
	}

	xgold_debug("Add HW probe mixer control\n");
	ret = snd_soc_add_platform_controls(platform, xgold_pcm_controls,
		ARRAY_SIZE(xgold_pcm_controls));

	if (ret < 0) {
		xgold_err("%s: Unable to add pcm platform controls\n",
		__func__);
		return -ENODEV;
	}

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
	int res = 0;
	struct xgold_pcm *pcm = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *dsp_of_node;

	xgold_debug("%s\n", __func__);

	pcm = kzalloc(sizeof(struct xgold_pcm), GFP_KERNEL);

	if (pcm == NULL)
		return -ENOMEM;

	pcm->dev = &pdev->dev;

#ifdef CONFIG_OF
	dsp_of_node = of_parse_phandle(np, "intel,dsp", 0);
	if (!dsp_of_node) {
		xgold_err("Unable to get dsp node\n");
		kzfree(pcm);
		return -EINVAL;
	}

	pcm->dsp = of_dsp_register_client(&pdev->dev, dsp_of_node);
#endif
	if (!pcm->dsp) {
		xgold_err("Cannot register as dsp client\n");
		kzfree(pcm);
		return -EPROBE_DEFER;
	}

	pcm->play_dma_mode =
		(of_find_property(np, PROP_PCM_DMA_EN_NAME, NULL)) ?
		true : false;
	pcm->rec_dma_mode =
		(of_find_property(np, PROP_PCM_REC_DMA_EN_NAME, NULL)) ?
		true : false;
	bt_init_en = (of_find_property(np, PROP_SND_SOC_BT_INIT, NULL)) ?
		true : false;

	if (pcm->play_dma_mode) {
		/* Configure PCM1 to NORMAL mode by default */
		/* Configure PCM2 in NORMAL/BURST mode */
		pcm->playback_mode[0] = NORMAL;
		pcm->playback_mode[1] =
			(of_find_property(np, PROP_SND_SOC_BURST_EN_NAME_PCM2,
			NULL)) ? BURST : NORMAL;

		xgold_debug("%s(): playback_mode pcm1:%d pcm2:%d\n", __func__,
				pcm->playback_mode[0], pcm->playback_mode[1]);

		if (BURST == pcm->playback_mode[0]) {	/* Burst mode */
			pcm->buffer_mode[0] = BURST_MODE;
			pcm->buffer_size[0] = BURST_BUFFER_SIZE;
			pcm->dma_req_interval_time[0] = BURST_DMA_INTERVAL;
		} else {                        /* Normal mode */
			pcm->buffer_mode[0] = NORMAL_MODE;
			pcm->buffer_size[0] = NORMAL_BUFFER_SIZE;
			pcm->dma_req_interval_time[0] = NORMAL_DMA_INTERVAL;
		}

		if (BURST == pcm->playback_mode[1]) {	/* Burst mode */
			pcm->buffer_mode[1] = BURST_MODE;
			pcm->buffer_size[1] = BURST_BUFFER_SIZE;
			pcm->dma_req_interval_time[1] = BURST_DMA_INTERVAL;
		} else {                        /* Normal mode */
			pcm->buffer_mode[1] = NORMAL_MODE;
			pcm->buffer_size[1] = NORMAL_BUFFER_SIZE;
			pcm->dma_req_interval_time[1] = NORMAL_DMA_INTERVAL;
		}
	} else {
		/* Configure to NORMAL mode in Interrupt mode by default */
		pcm->playback_mode[0] = NORMAL;
		pcm->playback_mode[1] = NORMAL;

		xgold_debug("%s(): playback_mode pcm1:%d pcm2:%d\n", __func__,
				pcm->playback_mode[0], pcm->playback_mode[1]);

		pcm->buffer_mode[0] = NORMAL_MODE;
		pcm->buffer_size[0] = NORMAL_BUFFER_SIZE;
		pcm->dma_req_interval_time[0] = NORMAL_DMA_INTERVAL;

		pcm->buffer_mode[1] = NORMAL_MODE;
		pcm->buffer_size[1] = NORMAL_BUFFER_SIZE;
		pcm->dma_req_interval_time[1] = NORMAL_DMA_INTERVAL;
	}

	ret = snd_soc_register_platform(&pdev->dev, &xgold_soc_platform);

	if (ret < 0) {
		xgold_err("Failed to register XGOLD platform driver\n");
		kfree(pcm);
		return ret;
	}

	ret = snd_soc_register_component(&pdev->dev, &xgold_shm_component,
			&xgold_dai_shm, 1);

	if (ret < 0) {
		xgold_err("Failed to register XGOLD platform driver 1\n");
		kfree(pcm);
		return ret;
	}

	/* pinctrl */
	pcm->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pcm->pinctrl)) {
		pcm->pinctrl = NULL;
		goto skip_pinctrl;
	}

	pcm->pins_default = pinctrl_lookup_state(pcm->pinctrl,
						 PINCTRL_STATE_DEFAULT);
	if (IS_ERR(pcm->pins_default))
		xgold_debug("could not get default pinstate\n");

	pcm->pins_sleep = pinctrl_lookup_state(pcm->pinctrl,
					       PINCTRL_STATE_SLEEP);
	if (IS_ERR(pcm->pins_sleep))
		xgold_debug("could not get sleep pinstate\n");

	pcm->pins_inactive =
		pinctrl_lookup_state(pcm->pinctrl,
					       "inactive");
	if (IS_ERR(pcm->pins_inactive))
		xgold_debug("could not get inactive pinstate\n");

skip_pinctrl:
	pcm->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pcm->pm_platdata)) {
		xgold_debug("Missing pm platdata properties\n");
		pcm->pm_platdata = NULL;
	} else
		xgold_debug("%s: pm class name %s pdev->dev\n", __func__,
				pcm->pm_platdata->pm_user_name);

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (pcm->pm_platdata) {
		res = platform_device_pm_set_class(pdev,
				pcm->pm_platdata->pm_user_name);

		if (res < 0)
			xgold_err("%s: failed to set PM class error %d\n",
				__func__, res);

		/* Disable I2S2 Power and clock domains */
		res = device_state_pm_set_state_by_name(
				pcm->dev,
				pcm->pm_platdata->pm_state_D3_name);
		if (res < 0)
			xgold_err("%s: failed to set PM state error %d\n",
					__func__, res);
	}
#endif

	/* afe_out/i2s2_out default on sofia_3g/sofia_lte */
	if (pcm->dsp->id == XGOLD_DSP_XG642) {
		pcm->hw_probe_status[HW_PROBE_POINT_A].hw_probe_sel = 1;
		pcm->hw_probe_status[HW_PROBE_POINT_B].hw_probe_sel = 2;
	} else {
		pcm->hw_probe_status[HW_PROBE_POINT_A].hw_probe_sel = 3;
		pcm->hw_probe_status[HW_PROBE_POINT_B].hw_probe_sel = 4;
	}
	xgold_debug("HW probe select initialized to: %d / %d\n",
		pcm->hw_probe_status[HW_PROBE_POINT_A].hw_probe_sel,
		pcm->hw_probe_status[HW_PROBE_POINT_B].hw_probe_sel);

	platform_set_drvdata(pdev, pcm);
	/* Disable I2S2 pins at init */
	res = i2s_set_pinctrl_state(&pdev->dev, pcm->pins_inactive);

	/* set I2s2 device details */
	i2s2_set_device_data(&pdev->dev, XGOLD_I2S2);

	xgold_pcm_register_sysfs_attr(&pdev->dev);

	return ret;
}

static int xgold_pcm_remove(struct platform_device *pdev)
{
	struct xgold_pcm *pcm = platform_get_drvdata(pdev);

	if (pcm && pcm->dump_record_buffer) {
		vfree(pcm->dump_record_buffer);
		pcm->dump_record_buffer = NULL;
		pcm->dump_record_buffer_pos = 0;
		pcm->record_dump = 0;
		destroy_work_on_stack(&pcm->record_dump_work);
	}

	if (pcm && pcm->dump_dma_buffer) {
		vfree(pcm->dump_dma_buffer);
		pcm->dump_dma_buffer = NULL;
		pcm->dump_dma_buffer_pos = 0;
		pcm->dma_dump = 0;
		destroy_work_on_stack(&pcm->dma_dump_work);
	}

	xgold_debug("%s\n", __func__);

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
		.name = "XGOLD_PCM",
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
		xgold_err("%s: Unable to add register pcm platform driver\n",
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
