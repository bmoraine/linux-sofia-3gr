/*
 * Copyright (C) 2012 Intel Mobile Communications GmbH
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
 */

/*
** =============================================================================
**
**                              MODULE DESCRIPTION
**
** =============================================================================
*/
/* This file contains the DSP command structures available on a given chip. */

#ifndef _AUD_LIB_DSP_HAL_H
#define _AUD_LIB_DSP_HAL_H

/*
** =============================================================================
**
**                              INCLUDE STATEMENTS
**
** =============================================================================
*/
#include "bastypes.h"           /* basic types */
#include "dsp_audio_driverif.h" /* dsp commands */
/*


** =============================================================================
**
**                             GLOBAL DEFINITIONS
**
** =============================================================================
*/
#define	DSP_GAIN_MUTE_VALUE	-960

/*
** =============================================================================
**
**                          GLOBAL TYPE DEFINITIONS
**
** =============================================================================
*/

/*----------------------------------------------------------------------------*/
/*                 Data structures related to DSP commands                    */
/*----------------------------------------------------------------------------*/

/**
   \brief T_AUD_DSP_CMD_VB_SET_SWM_AFE_OUT_PAR

   Structure holding the parameters of a VB_SET_SWM_SPEECH_AFE DSP command
*/
struct T_AUD_DSP_CMD_VB_SET_SWM_AFE_OUT_PAR {
	U16 setting;
	U16 afe_in;
	U16 speech_in;
	U16 probe_in;
	U16 bt_in;
	U16 i2s_in;
	U16 pcm_in;
	U16 digmic_in;
	U16 fmr_in;
	U16 tone_in;
	U16 sidetone_in;
	U16 ppl_in;
	U16 pcm_a_in;
};

/**
   \brief T_AUD_DSP_CMD_VB_SET_SWM_PCM_OUT_PAR

   Structure holding the parameters of a VB_SET_SWM_SPEECH_PCM DSP command
*/
struct T_AUD_DSP_CMD_VB_SET_SWM_PCM_OUT_PAR {
	U16 setting;
	U16 afe_in;
	U16 speech_in;
	U16 probe_in;
	U16 bt_in;
	U16 i2s_in;
	U16 pcm_in;
	U16 digmic_in;
	U16 fmr_in;
	U16 tone_in;
	U16 sidetone_in;
	U16 ppl_in;
	U16 pcm_a_in;
};

/**
   \brief T_AUD_DSP_CMD_VB_SET_GAIN_HW_PAR

   Structure holding the parameters of a VB_SET_GAIN DSP command
*/
struct T_AUD_DSP_CMD_VB_SET_GAIN_HW_PAR {
	U16  scal_afe_in;
	U16  scal_afe_out_l;
	U16  scal_afe_out_r;
	U16  scal_bt_in;
	U16  scal_bt_out;
	U16  scal_digmic_in;
	U16  scal_fmr_in;
	U16  scal_fmr_out;
	U16  scal_i2s_in;
	U16  scal_i2s_out;
	U16  scal_tone_in;
	U16  scal_hf_out;
	U16  scal_speech_out;
	U16  scal_sidetone_in;
	U16  scal_speech_in;
	U16  scal_ppl_in;
	U16  scal_pcm_out;
	U16  scal_pcm_in_left;
	U16  scal_pcm_in_right;
	U16  scal_speech_out_cal;
	U16  scal_speech_in_cal;
	U16  scal_sns_out;
	U16  pcm_a_in_left;
	U16  pcm_a_in_right;
	U16  scal_pcm_vibra_out;
};

/**
   \brief T_AUD_DSP_CMD_VB_SET_GAIN_TIMECONST

   Structure holding the parameters of a VB_SET_GAIN_TIMECONST DSP command
 */
struct T_AUD_DSP_CMD_VB_SET_GAIN_TIMECONST {
	U16 timeconstant;
};

/**
   \brief T_AUD_DSP_CMD_VB_SET_IIR_PAR

   Structure holding the parameters of a VB_SET_IIR DSP command
*/
struct T_AUD_DSP_CMD_VB_SET_IIR_PAR {
	U16 iir_select;
	U16 setting;
	U16 order;
	U16 biquad_0_a1;
	U16 biquad_0_b1;
	U16 biquad_0_a2;
	U16 biquad_0_b2;
	U16 biquad_0_a0;
	U16 biquad_1_a1;
	U16 biquad_1_b1;
	U16 biquad_1_a2;
	U16 biquad_1_b2;
	U16 biquad_1_a0;
	U16 biquad_2_a1;
	U16 biquad_2_b1;
	U16 biquad_2_a2;
	U16 biquad_2_b2;
	U16 biquad_2_a0;
	U16 biquad_3_a1;
	U16 biquad_3_b1;
	U16 biquad_3_a2;
	U16 biquad_3_b2;
	U16 biquad_3_a0;
};

/**
   \brief T_AUD_DSP_CMD_VB_HW_AFE_PAR

   Structure holding the parameters of a VB_HW_AFE DSP command
*/
struct T_AUD_DSP_CMD_VB_HW_AFE_PAR {
	U16  setting;
	U16  ratesw;
};

/**
   \brief T_AUD_DSP_CMD_PCM_PLAY_PAR

   Structure holding the parameters of a PCM_PLAY DSP command
*/
struct T_AUD_DSP_CMD_PCM_PLAY_PAR {
	U16  setting;
	U16  mode;
	U16  rate;
	U16  req;
	U16  buffer_mode;
	U16  dma_req_interval_time;
	U16  buffer_size;
};

/**
   \brief T_AUD_DSP_CMD_PCM_REC_PAR

   Structure holding the parameters of a PCM_REC DSP command
*/
struct T_AUD_DSP_CMD_PCM_REC_PAR {
	U16  setting;
	U16  mode;
	U16  rate;
	U16  path_select;
	U16  req;
};

/**
   \brief T_AUD_DSP_CMD_SPEECH_PROBE_PAR

   Structure holding the parameters of a SPEECH_PROBE DSP command
*/
struct T_AUD_DSP_CMD_SPEECH_PROBE_PAR {
	U16 on_off;
	U16 select;
	U16 mode;
	U16 gain1;
	U16 gain2;
	U16 gain3;
	U16 gain4;
	U16 gain5;
	U16 gain6;
	U16 sm_buf_id;
	U16 sm_buf_mix_id;
	U16 sampling_rate_inj;
	U16 sampling_rate_ext;
};

/**
   \brief T_AUD_DSP_CMD_VB_SET_SWM_MIX_MATRIX

   Structure holding the parameters of a VB_SET_SWM_MIX_MATRIX DSP command
*/
struct T_AUD_DSP_CMD_VB_SET_SWM_MIX_MATRIX {
	U16 select;
	U16 alpha_0;
	U16 alpha_1;
	U16 alpha_2;
	U16 alpha_3;
};

struct T_AUD_DSP_CMD_HW_PROBE {
	U16 probe_index;
	U16 setting;
	U16 sm_interface;
	U16 mix_flag;
	U16 injection_gain;
};

struct T_AUD_DSP_CMD_VB_SET_SPEECH_PATH_PAR {
	U16  setting;
	U16  rate;
};



#endif /* _AUD_LIB_DSP_HAL_H*/

