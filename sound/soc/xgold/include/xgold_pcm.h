/*
 * Component: XGOLD PCM header file
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

#ifndef __XGOLD_PCM_H__
#define __XGOLD_PCM_H__

enum xgold_hw_probe_stream_type {
	HW_PROBE_POINT_A = 0,
	HW_PROBE_POINT_B,
	HW_PROBE_POINT_END
};

enum xgold_pcm_stream_type {
	STREAM_PLAY = 0,
	STREAM_PLAY2,
	STREAM_REC,
	HW_PROBE_A,
	HW_PROBE_B,
	NR_STREAM
};

enum dma_shm_sample_index {
	DMA_SHM_SAMPLE_INDEX_8K = 0,
	DMA_SHM_SAMPLE_INDEX_11K,
	DMA_SHM_SAMPLE_INDEX_12K,
	DMA_SHM_SAMPLE_INDEX_16K,
	DMA_SHM_SAMPLE_INDEX_22K,
	DMA_SHM_SAMPLE_INDEX_24K,
	DMA_SHM_SAMPLE_INDEX_32K,
	DMA_SHM_SAMPLE_INDEX_44K,
	DMA_SHM_SAMPLE_INDEX_48K,
	DMA_SHM_SAMPLE_INDEX_END,
};

struct xgold_pcm_hw_probe_status {
	bool active;
	unsigned int hw_probe_sel;
};

enum xgold_pcm_playback_mode {
	NORMAL = 0,	/* NORMAL is supported in both DMA/Interrupt mode */
	BURST,		/* BURST is supported in DMA mode only */
};

enum xgold_pcm_buffer_type {
	NORMAL_MODE = 0,
	BURST_MODE = 2,
};

enum xgold_pcm_dma_interval_time {
	NORMAL_DMA_INTERVAL = 0,	/* Not Used */
	BURST_DMA_INTERVAL = 2,		/* 250 us */
};

enum xgold_pcm_buffer_size {
	NORMAL_BUFFER_SIZE = 2,		/* 10 ms */
	BURST_BUFFER_SIZE = 16,		/* 80 ms */
};

struct xgold_pcm {
	struct device *dev;
	struct dsp_audio_device *dsp;
	bool play_dma_mode;
	bool rec_dma_mode;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct device_pm_platdata *pm_platdata;
	unsigned int path_select;
	unsigned short playback_mode[2];
	unsigned short buffer_mode[2];
	unsigned short dma_req_interval_time[2];
	unsigned short buffer_size[2];
	/* DMA DUMP */
	int dma_dump;
	struct work_struct dma_dump_work;
	unsigned char *dump_dma_buffer;
	int dump_dma_buffer_pos;
	struct xgold_pcm_hw_probe_status
		hw_probe_status[HW_PROBE_POINT_END];
	/* RECORD DUMP */
	int record_dump;
	struct work_struct record_dump_work;
	unsigned char *dump_record_buffer;
	int dump_record_buffer_pos;
};

struct xgold_runtime_data {
	struct xgold_pcm *pcm;
	struct snd_pcm_substream *stream;
	enum xgold_pcm_stream_type stream_type;
	unsigned short *hwptr;
	unsigned int hwptr_done;
	unsigned int periods;
	unsigned int period_size_bytes;
	/* DMA stream */
	unsigned short dma_bytes;
	unsigned short dma_sgl_count;
	struct scatterlist *dma_sgl;
	struct dma_chan *dmach;
	dma_cookie_t dma_cookie;
	struct completion dma_complete;
	bool dma_stop;
	unsigned total_nof_bytes_read;
	bool have_harmonics;
};

#endif /* __XGOLD_PCM_H__ */
