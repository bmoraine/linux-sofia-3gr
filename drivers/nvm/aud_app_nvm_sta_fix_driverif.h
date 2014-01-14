/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef _AUD_APP_NVM_STA_FIX_DRIVERIF_H
#define _AUD_APP_NVM_STA_FIX_DRIVERIF_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*
** =============================================================================
**
**                              INTERFACE DESCRIPTION
**
** =============================================================================
*/

/*
 *  @file aud_app_fmr_nvm_sta_fix_driverif.h
 *   This  file contains nvm data information for FMR
 *   modules.
 */

/*
** =============================================================================
**
**                              INCLUDE STATEMENTS
**
** =============================================================================
*/
#include "nvm/bastypes.h"           /* Basic types */

/*
** =============================================================================
**
**                                   DEFINES
**
** =============================================================================
*/
#define FREQ_BAND_SPLIT     5
#define CP_INIT_TABLE_LEN   16

/*
** =============================================================================
**
**                              ENUMERATIONS
**
** =============================================================================
*/

/*
 * List of different events to handle different aud app requests
 */

/*
** =============================================================================
**
**                   AUDIO APPLICATION NVM STRUCTURE
**
** =============================================================================
*/

/* @brief
 * Structure for holding NVM data for FMR
 */
typedef struct {
	U32  band_max;                              /* Maximum frequency in frequency band */
	U32  band_min;                              /* Manimum frequency in frequency band */
	U32  step_size;                             /* Step size in frequency band */
	S16  deem_sel;                              /* Deemphasis time constant */

	U32  frequency;                             /* Default frequency i.e last frequency befoer switch off */
	S16  routing;                               /* Output destination of FM audio output */

	S16  snc_lo_thr;                            /* SNC lower threshold */
	S16  snc_up_thr;                            /* SNC higher threshold */
	BOOL snc_en;                                /* Enable or disable SNC */

	U16  sm_step;                               /* Soft mute step size */
	S16  sm_thr;                                /* Threshold for softmute */
	BOOL sm_en;                                 /* Enable or disable softmute */

	U16  agc_gain_idx;                          /* AGC gain index 0-15 */
	BOOL agc_en;                                /* Enable or disable AGC */

	S16  pilot_lo_thr;                          /* Lower amplitude for pilot signal */
	S16  pilot_up_thr;                          /* Higher amplitude for pilot signal */

	S16  volume;                                /* Volume before switch off */
	BOOL force_mono;                            /* Mono or stereo */

	S16  phase_noise_thr;                       /* Phase noise detection threshold */

	BOOL ext_lna_present;                        /* External LNA is present or not */

	/* LNA Configuration for internal antenna */
	S8   ext_lna_tab_len_int_ant;                      /* External LNA table len where gain is constant for intern antenna */
	U32  ext_lna_freq_spilt_int_ant[FREQ_BAND_SPLIT];  /* frequency points which are used for splitting the FM band */
	S8   ext_lna_offset_int_ant[FREQ_BAND_SPLIT];      /* external LNA gain in dB, adjust rssi_other_offset for intern ant */

	/* LNA Configuration for external antenna */
	S8   ext_lna_tab_len_ext_ant;                      /* External LNA table len where gain is constant for extern antenna */
	U32  ext_lna_freq_spilt_ext_ant[FREQ_BAND_SPLIT];  /* frequency points which are used for splitting the FM band */
	S8   ext_lna_offset_ext_ant[FREQ_BAND_SPLIT];      /* external LNA gain in dB, adjust rssi_other_offset for ext antenna */

	S16  rssi_offset_int_antenna;               /* RSSI offset compensation for path via internal/embedded antenna */
	S16  rssi_offset_ext_antenna;               /* RSSI offset compensation for path via external antenna */

	U8   int_lna_gain_reduction;                /* Reduction of internal LNA gain  */
	U16  volume_ramping_cfg;                    /* IIR filter configuration for volume change ramping */
	U32  clk_switch_range_104;                  /* FMRadio clock switching range around 104 MHz channel */

	//U16  cp_init_table[CP_INIT_TABLE_LEN];      /* Cp initial values table */
	U8   antenna_selection;                     /* Antenna selection and config for Rx: ext/int, tuned/no tuning */
	U8   rx_if_selection;                       /* RX IF selection: 0 -> 275KHz,  1->  525KHz */
} T_AUD_APP_FMR_NVM;


/* This type shall be included in nvm_cfg.h by the storage team for FMR module */
typedef struct {
	T_AUD_APP_FMR_NVM aud_app_fmr_nvm_sta;
} T_FMR_NVM_STA_FIX;

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif /* _AUD_APP_NVM_STA_FIX_DRIVERIF_H */
/*!
 *  \}
 */
/* End of file. */
