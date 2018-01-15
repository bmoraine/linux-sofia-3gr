/*
 * Component: XGOLD DSP Audio Driver
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

#ifndef __DSP_AUDIO_PLATFORM_H__
#define __DSP_AUDIO_PLATFORM_H__

#include <sound/soc.h>
#include "bastypes.h"
#include "dsp_audio_driverif.h"

/* List of the platform controls supported by the DSP driver */
enum dsp_audio_controls {
	DSP_AUDIO_CONTROL_SEND_CMD = 0,
	DSP_AUDIO_CONTROL_READ_SHM,
	DSP_AUDIO_CONTROL_WRITE_SHM,
	DSP_AUDIO_CONTROL_SET_PLAY_PATH,
	DSP_AUDIO_CONTROL_SET_REC_PATH,
	DSP_AUDIO_POWER_REQ,
	DSP_AUDIO_CONTROL_END
};

struct dsp_ops {
	int (*open)(void);
	int (*set_controls)(enum dsp_audio_controls cmd, void *arg);
	int (*irq_activate)(enum dsp_irq_no);
	int (*irq_deactivate)(enum dsp_irq_no);
	int (*close)(void);
};

struct dsp_clk {
	struct list_head node;
	struct clk *clk;
};

struct xgold_dsp_reg {
	void __iomem *reg;
	unsigned char shift;
	unsigned char width;
};

enum dsp_id {
	XGOLD_DSP_XG223 = 0,
	XGOLD_DSP_XG631,
	XGOLD_DSP_XG632,
	XGOLD_DSP_XG642,
	XGOLD_DSP_XG742_FBA,
	XGOLD_DSP_XG742_SBA
};

struct dsp_audio_device {
	char *name;
	struct device *dev;
	int dsp_sched_start;
	struct list_head pipe;
	struct list_head done;
	struct list_head free;
	enum dsp_id id;
	void __iomem *shm_mem;
	dma_addr_t shm_mem_phys;
	void __iomem *shm_regs;
	unsigned short *patch;
	unsigned patch_length;
	struct list_head clk_list;
	struct regulator *regulator;
	unsigned interrupts[8]; /* must be same size as dsp_interrupt above */
	struct communication_flag {
		unsigned get;
		unsigned set;
		unsigned clear;
	} uccf;
	unsigned mcu2dsp;
	unsigned imsc;
	unsigned icr;
	unsigned mis;
	struct xgold_dsp_reg rst;
	struct dsp_common_data *p_dsp_common_data;
	unsigned rstmods;
	unsigned clc;
	unsigned stream_status;
	struct reset_control *rst_ctl;
	struct device_pm_platdata *pm_platdata;
	struct list_head node;
};

/* TODO  add device instances for SBA/FBA */
struct dsp_common_data {
	int native_mode;
	int rst_done;
	struct dsp_ops *ops;
	struct device *fba_dev;
	unsigned pcm_offset[2];
	unsigned buf_size_ul_offset;
	unsigned buf_sm_dl_offset;
	unsigned buf_sm_ul_offset;
	unsigned buf_sm_hw_probe_a_offset;
	unsigned buf_sm_hw_probe_b_offset;
	unsigned buf_sm_speech_probe_a_offset;
	unsigned buf_sm_speech_probe_b_offset;
	unsigned buf_sm_speech_probe_c_offset;
	unsigned buf_sm_speech_probe_d_offset;
	unsigned buf_sm_speech_probe_e_offset;
	unsigned buf_sm_speech_probe_f_offset;
	unsigned num_dsp;
};

int register_dsp_audio_lisr_cb(enum dsp_lisr_cb lisr_type,
			       void (*p_func) (void *), void *);
int register_audio_dsp(struct dsp_audio_device *dsp);
int unregister_audio_dsp(struct dsp_audio_device *dsp);
int dsp_audio_platform_init(struct snd_soc_platform *platform);
int dsp_start_audio_hwafe(void);
int dsp_stop_audio_hwafe(void);

/* extern declarations */
extern struct dsp_audio_device *p_dsp_audio_dev;

#endif /*__DSP_AUDIO_PLATFORM_H__ */
