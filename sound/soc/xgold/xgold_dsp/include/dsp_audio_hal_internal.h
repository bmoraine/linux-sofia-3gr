/*
 * Component: XGOLD6321 DSP Audio Driver
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

#ifndef _DSP_HAL_INTERNAL_H
#define _DSP_HAL_INTERNAL_H
#include <linux/dmaengine.h>
#include <dsp_audio_platform.h>

/*---------------------------------------------------------------------------
							Function Declarations
----------------------------------------------------------------------------*/
void dsp_generate_audio_dsp_interrupt(struct dsp_audio_device *dsp,
		enum dsp_irq_no dsp_interrupt);
U32 dsp_get_audio_imsc(struct dsp_audio_device *dsp);
void dsp_set_audio_imsc(struct dsp_audio_device *dsp, U32 reg);
void dsp_set_audio_icr(struct dsp_audio_device *dsp, U32 icr);
#ifdef CONFIG_OF
struct dma_chan *xgold_of_dsp_get_dmach(struct dsp_audio_device *dsp,
		int req);
#endif

/*------------------------------------------------------------------------------
			Defines for offsets into frame based audio shared memory
------------------------------------------------------------------------------*/
#define DSP_AUDIO_COMMAND_PIPE_SM_OFFSET                4
#endif /* _DSP_HAL_INTERNAL_H */

