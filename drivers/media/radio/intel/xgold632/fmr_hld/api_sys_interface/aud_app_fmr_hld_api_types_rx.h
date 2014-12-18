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

/*
** ============================================================================
**
**				MODULE DESCRIPTION
**
** ============================================================================
*/

/*
* @file aud_app_fmr_hld_api_types_rx.h
*
* This file defines the types used for the FMR RX high level dirver APIs
* declaration.
*
*/

#ifndef AUD_APP_FMR_HLD_API_TYPES_RX_H
#define AUD_APP_FMR_HLD_API_TYPES_RX_H

/*
** ============================================================================
**
**				INCLUDE STATEMENTS
**
** ============================================================================
*/

#include <types.h>
#include <aud_app_fmr_hld_api_types_common.h>

/*
** ============================================================================
**
**				     DEFINES
**
** ============================================================================
*/

/* RDS ring-buffer size defined by FW */
#define RDS_RING_BUFFER_SIZE 29

/* The maximum possible number of valid RDS groups in ring-buffer */
#define RDS_MAX_VALID_GROUPS (RDS_RING_BUFFER_SIZE - 1)

#define MAX_FREQ_SPLIT		5
#define MAX_FREQ_OFFS_LEN	(MAX_FREQ_SPLIT + 1)

/*
** ============================================================================
**
**				  Type defintions
**
** ============================================================================
*/

/* Seeking direction */
enum fmtrx_seek_dir {
	/* Tuning to provided frequency, don't seek */
	FMRX_SEEK_OFF,
	/* Searches increasing frequencies and auto wrap around */
	FMRX_SEEK_UP,
	/* Searches decreasing frequencies and auto wrap around */
	FMRX_SEEK_DOWN,
	/* Searches stops at upper FM band limits, no wrapping */
	FMRX_SEEK_UP_TO_LIMIT,
	/* Searches stops at lower FM band limits, no wrapping */
	FMRX_SEEK_DOWN_TO_LIMIT,
	FMRX_SEEK_MODE_INVALID
};

/* Gain control of the Int LNA	if external LNA used for cord */
/* antenna for fulfillment of testcases detector trigger */
enum fmrx_lna_out_gain {
	FMR_LOGAIN_9DB_REDUC = 0, /* Int LNA  would have 9 dB gain reduction */
	FMR_LOGAIN_6DB_REDUC = 1, /* Int LNA  would have 6 dB gain reduction */
	FMR_LOGAIN_3DB_REDUC = 2, /* Int LNA  would have 3 dB gain reduction */
	FMR_LOGAIN_0DB_REDUC = 3  /* Int LNA  would have 0 dB gain reduction */
};

/*
 * @breif
 * Watchdog checking modes
 */
enum fmrx_watchdog_mode {
	POLLING   = 0, /* polling mode, means compare with the data GSM timer*/
	CHECKING  = 1	/* checking mode, means compare immediately */
};


/*
 * @breif
 *  3 status for ADC clcok switching
 */
enum fmrx_clk_switch {
	ADC_ON_ONLY = 0,  /* ADC clock switched on only */
	BOTH_ON     = 1,  /* Both ADC clock & CGU clock were switched on */
	BOTH_OFF    = 2   /* None of ADC and CGU clocks was switched on */
};

/*
 * @breif
 * 3 states for RDS working mode selection
 */
enum fmrx_rds_mode {
	RDS_OFF     = 0,  /* RDS disabled */
	RDS_ON	    = 1,  /* RDS working in normal mode */
	RDS_RETAIN  = 2   /* RDS working in retain mode, therefore no interrupt
				generated for data transfer */
};

/* scan state */
enum fmrx_scan_status {
	SCAN_SUCCESS = 0,   /* seek/scan/evaluat success */
	SCAN_FAILURE = 1,   /* seek/scan/evaluat failure */
	SCAN_ONGOING = 2,   /* seek/scan/evaluat ongoing */
	SCAN_STOP    = 3,   /* seek/scan/evaluat stop */
	SCAN_INVALID	    /* seek/scan/evaluat invalid */
};

/*
 * @brief
 * RDS PI working mode selection
 */
enum fmrx_rds_pi_mode {
	RDS_NORMAL_PI = 0,/* Normal working mode */
	RDS_FAST_PI   = 1 /* every PI code received will issue an interrupt */
};

/*
 * @brief
 * For audio routing selection
 */
enum fmrx_aud_route {
	FMR_AUD_PATH_DAC = 0,  /* DAC interface selected */
	FMR_AUD_PATH_DSP = 1,  /* SRC (Teaklite) interface selected */
	FMR_AUD_PATH_OFF = 2,  /* FMR Audio output is turned off */
	FMR_AUD_PATH_INVALID
};

/*
 * @brief
 * Force Auto/Low/High side inject
 */
enum fmrx_sb_inj_side {
	FMR_FORCE_HSI	= 0,
	FMR_FORCE_LSI	= 1,
	FMR_FORCE_NONE	= 2,
	FMR_FORCE_INVALID
};

/*
 * @brief
 * Tracing feature selection
 */
enum fmrx_test_trace_sel {
	FMR_RX_TRACE_DISABLED	  = 0,	/* Disabled the feature when "0" */
	FMR_RX_TRACE_AUDIO_OUT_LR = 1 /* Dumpping the data audio o/p to AFE*/
};

/*
 * @brief
 * FM Audio channel selection
 */
enum fmrx_aud_channel {
	FMRX_AUD_CHN_ALL    = 0,  /* Selects all possible channels */
	FMRX_AUD_CHN_LEFT   = 1,  /* Selects only left channel */
	FMRX_AUD_CHN_RIGHT  = 2,  /* Selects only right channel */
	FMRX_AUD_CHN_END    = 3   /* Invalid, only used internally */
};

/*
 * Enumeration for AGC gain index
 */
enum fmrx_agc_gain_index {
	AGC_GAIN_INDEX_0,	   /* Gain index 0  (-7.5dB)*/
	AGC_GAIN_INDEX_1,	   /* Gain index 1  (-7dB)  */
	AGC_GAIN_INDEX_2,	   /* Gain index 2  (-6.5dB)*/
	AGC_GAIN_INDEX_3,	   /* Gain index 3  (-6dB)  */
	AGC_GAIN_INDEX_4,	   /* Gain index 4  (-5.5dB)*/
	AGC_GAIN_INDEX_5,	   /* Gain index 5  (-5dB)  */
	AGC_GAIN_INDEX_6,	   /* Gain index 6  (-4.5dB)*/
	AGC_GAIN_INDEX_7,	   /* Gain index 7  (-4dB)  */
	AGC_GAIN_INDEX_8,	   /* Gain index 8  (-3.5dB)*/
	AGC_GAIN_INDEX_9,	   /* Gain index 9  (-3dB)  */
	AGC_GAIN_INDEX_10,	   /* Gain index 10 (-2.5dB)*/
	AGC_GAIN_INDEX_11,	   /* Gain index 11 (-2dB)  */
	AGC_GAIN_INDEX_12,	   /* Gain index 12 (-1.5dB)*/
	AGC_GAIN_INDEX_13,	   /* Gain index 13 (-1dB)  */
	AGC_GAIN_INDEX_14,	   /* Gain index 14 (-0.5dB)*/
	AGC_GAIN_INDEX_15,	   /* Gain index 15 (0dB)   */
	AGC_GAIN_INDEX_END,
};

enum fmtrx_inj_sel {
	/* used for retuning the reciever with the given sb. This setting is
	 * not persistent
	 */
	FMR_SB_SEL_TMP = 0,

	/* This causes the given sideband to be persistent until it is reset
	 * with the new value
	 */
	FMR_SB_SEL_PERST,
	FMR_SB_SEL_INVALID
};

struct rssi_notify {
	u8 enable;	/* 1 - enable. 0 - disable. */
	s16 lo_thr;	/* Unit in dBuV */
	s16 hi_thr;	/* Unit in dBuV */
} __packed;

struct volume {
	u8 left;
	u8 right;
} __packed;

struct other_params {
	u16 pn_thr;
	u8 lna_out_gain;
	s16 vol_ramp;
	u32 clk_swt_rng;
	s16 rssi_off_int;/* Overall RSSI offset for embeded antenna */
	s16 rssi_off_ext;/* Overall RSSI offset for headset antenna */
} __packed;

struct rds {
	u8 mode;	/* 1 - enable. 0 - disable. */
} __packed;

struct af_info {
	u32 count;
	u32 *freq_list;
	s16 pi_code;
} __packed;

/*
 * @brief
 * External LNA config
 */
struct fmrx_ext_lna_cfg {
	/* Stores the frequency points which are used for splitting the FM band
	 * into several ranges */
	u32 band_split[MAX_FREQ_SPLIT];

	/* Stores the ext. LNA gain offsets in dBuV corresponding to the ranges
	 * defined in ext_lna_band_split[] */
	s8 offsets[MAX_FREQ_OFFS_LEN];

	/* Indicates how many items are avilable in offsets table */
	u8 tab_len;
} __packed;

/*
 * @brief
 *  Structure for holding the parameters required for station tuning.
 */
struct fmrx_tune_station {
	u32 freq;
};

struct fmtrx_ant_selftest_freq {
	u32 freq;
};

struct fmrx_sine_tone_test_params {
	struct fmtrx_test_sine_cfg left_ch;
	struct fmtrx_test_sine_cfg right_ch;
	enum fmrx_aud_route   aud_routing;
};

struct fmrx_seek_station_params {
	enum	fmtrx_seek_dir seek_mode;	/* seek direction / mode */
	u16	seek_pn;	/* PN target for seeking / autoseeking */
};


/*
 * @brief
 * AF RSSI measure parameters
 */
struct fmrx_af_measure_rssi {
	u32 max_count;
	struct fmtrx_rssi_report *rssi_report;
};

/*
 * @brief
 * AF Evaluation parameters
 */
struct fmrx_af_eval {
	u32 af_freq;
	u16 pi_code;
	s16 af_rssi_th;
};

struct fmrx_pilot_state {
	s32 pilot_found; /* Flag to indicate pilot detected */
};

struct fmrx_rssi_ev_info {
	s16  rssi;  /* The actual RSSI value reported with the RSSI event */
	s32 last_rssi_enable;
};

struct fmrx_rds_sync_ev_info {
	s32 rds_sync;
};

struct fmrx_rds_ev_info {
	u16 valid_grps;/* number of RDS groups available in FW ring-buffer */
	u16 discard_grps;/* number of RDS groups to be discarded */
};

/*
 * @brief
 * Structure for holding the frequency tracking enable / disable information
 */
struct fmrx_en_freq_tracking {
	s32 freq_tracking;
};

/*
 * Structure for holding the channel and the corresponfing volume
 *
 * This structure is a part of the struct fmrx_msgbox_buff union type
 * which holds parameters sent from the HLD interface to FM Rx module.
 */
struct fmrx_set_aud_ch_vol {
	enum fmrx_aud_channel channel;
	u8 volume;
};

/*
 * Structure for holding the channel and the corresponfing volume
 *
 * This structure is a part of the struct fmrx_msgbox_buff union type
 * which holds parameters sent from the HLD interface to FM Rx module.
 */
struct fmrx_get_aud_ch_vol {
	enum fmrx_aud_channel channel;
	u16 *p_volume;
};

struct fmrx_set_mute {
	enum fmrx_aud_channel channel;
	s32 mute;
};

struct fmrx_set_force_mono {
	s32 mono;
};

struct fmrx_set_ext_lna_rssi_off {
	u32 *split_tab;
	s8  *offset_tab;
	s8  band_len;
};

struct fmrx_rssi_offsets {
	s16 rssi_offset_hs;	/* Offset to internally measured RSSI */
	s16 rssi_offset_embed;	/* Offset to internally measured RSSI */
};

struct fmrx_rssi_subsc {
	s32 subscribe;		/* Enables RSSI callback */
	s16 lower_thres;	/* Lower bound for RSSI reporting */
	s16 upper_thres;	/* Upper bound for RSSI reporting */
};

struct fmrx_rds_subscribe_params {
	enum fmtrx_rds_mode rds_type;
	enum fmtrx_subscribe_rds_mode rds_subsc_mode;
	enum fmrx_rds_pi_mode rds_pi_mode;
	u8 min_free;
};

struct fmrx_check_watchdog_status {
	enum fmrx_watchdog_mode check_mode;
	s32 *mini_dsp_alive;
};

struct fmrx_trace_params {
	enum fmrx_test_trace_sel trace_sel;
	u32 *trace_data;
	u32 target_num;
};

struct fmrx_trace_pt_sel {
	u8 trace_point;
};

struct fmrx_get_rds_grps {
	u16	group_cnt;
	u16	*copied_grps;
	void	*rds_buf_desc;
};

/* RDS synchornization configure */
struct fmrx_rds_sync_cfg {
	/*
	* Good blocks after synchronisation
	* This the number of good blocks to be received before the good
	* block counter and the bad block counter are reset. If set to zero,
	* the good and bad block counters are never reset (they will
	* saturate at 63)
	*/
	u8 gblk;

	/*
	* Bad blocks after synchronisation
	* This is the maximum number of bad blocks that can be received
	* before synchronisation is lost. If set to zero, synchronisation will
	* never be lost.
	*/
	u8 bblk;

	/*
	* Bad blocks during synchronisation search
	* This is the maximum number of bad blocks that can be received
	* during synchronisation search before the search starts over
	* again. If set to zero, synchronisation will never be restarted.
	*/
	u8 bblks;
};

/*
 * @brief
 * RDS configuration
 */
struct fmrx_rds_cfg {
	/*
	 * @brief
	 * RDS working mode selection
	 */
	enum fmtrx_rds_mode rds_mode;

	/*
	 * @brief
	 * Number of free spaces in FMR-internal group buffer below which a
	 * callback with type FMR_RDS_CNT occurs.
	 *
	 * If set to 0, buffering is disabled and groups are passed as soon as
	 * they are received using callback type FMR_RDS_GROUP
	 * (legacy behaviour).
	 *
	 * The maximum valid value is the (buffer_size - 2), for value
	 * (buffer_size - 1) & above, the irq will be generated only when the
	 * buffer is full.
	 */
	u8 min_free;

	/*
	 * @brief
	 * Parameters for acquisition, maintenance and loss of synch
	 */
	struct fmrx_rds_sync_cfg sync_cfg;
};

/*
 * @brief
 * Current channel information for reporting to the upper layer (UTA)
 */
struct fmrx_ch_info {
	u32	state;	/* The current receiving frequency */
	u32	freq;	/* The current receiving frequency */
	s32	mono;	/* The current mono/stereo status */
	s32	pilot_found; /* Flag to indicate pilot detected */
	s16	rssi;	/* The current RSSI vlaue */
	enum fmrx_sb_inj_side injside; /* Injection side */
	u16	pn;	/* The Phase Noise figure */
	u32	foffs;	/* The frequency offset */
	u16	pilotamp;	/* The pilot amplitude */
	u32	seek_state;	/* Seek state */
	s32	rds_en;		/* - RDS Enabled, 0 - RDS Disabled */
	u32	rds_mode;	/* 0 is RDS, 1 is RBDS */
	s32	rds_sync;	/* 1:RDS Sync enabled, 0: RDS Sync disabled */
};

/*
 * @brief
 * PN configuration
 */
struct fmrx_pn_cfg {
	s16 pn_thr;		/* PN detection threshold  */
	s16 pn_win_size;	/* PN detection window size*/
	s16 pn_settle_blocks;	/* PN measurement time */
};

/*
 * @brief
 * FM band parameters
 */
struct fmtrx_band {
	u32 max;	/* The FM band high band limit */
	u32 min;	/* The FM band low band limit */
	u32 step;	/* The stepsize for seeking/autoseeking */
	u8 deem;	/* Deemphasis mode selection */
} __packed;

/*
 * @brief
 * SoftMute configuration
 */
struct fmrx_sm {
	u8 en;	/* 1 - enable. 0 - disable. */
	s16 thr;/* units: dBuV */
	u16 step;
} __packed;

/*
 * @brief
 * AGC configuration
 */
struct fmrx_agc {
	u8 en;	/* 1 - enable. 0 - disable. */
	u8 gain_idx;
} __packed;

/*
 * @brief
 * structure for SNC configuration
 */
struct fmrx_snc {
	u8 en;		/* 1 - enable. 0 - disable. */
	s16 lo_thr;	/* Unit in dBuV */
	s16 hi_thr;	/* Unit in dBuV */
} __packed;

/*
 * @brief
 * AGC Manual mode config
 */
struct fmrx_test_agc_man_cfg {
	s32 manual;		/* enable/disable the manual AGC mode */
	u8 lna_gain;		/* LNA gain setting in AGC manual mode */
	u8 lna_out_gain;	/* Gain of the LNA II stage, in 3 dB steps */
	u8 ppf_gain;		/* PPF gain setting in AGC manual mode */
	u8 ppf_gain_ctrl2nd;	/* Gain of the PPF control 2nd stage */
};

struct fmrx_set_ext_lna_rssi_comp {
	struct fmrx_ext_lna_cfg lna_cfg;
	enum fmtrx_ant_type	ant_type;
};

struct fmrx_get_ext_lna_rssi_comp {
	struct fmrx_ext_lna_cfg *lna_cfg;
	enum fmtrx_ant_type ant_type;
};

/*
 * @brief
 * FW register Dumping for debug.
 */
struct fmrx_fw_reg {
	s16 high_16;   /* High 16-bit of FW register */
	s16 low_16;    /* Low 16-bit of FW register */
};

/* @brief
 *  Extensive Channel RSSI information
 */
struct fmrx_ext_ch_rssi {
	/* measured rssi of wanted channel at current sideband (still including
	 * all spurs) */
	s16 rssi;
	/* effective rssi value of wanted channel at current sideband */
	s16 rssi_eff;
	/* offset for rssi threshold due to noise figure (LNA, PPF,
	 * PPFCTLR_2ND settings) during channel evaluation */
	s16 rssi_nf_offs;
	/* the raw rssi of the image channel sideband (no suppression) */
	s16 image_rssi_raw;
	/* the rssi of the image within wanted channel sideband, image
	 * suppression is already subtracted (included in rssi_tot) */
	s16 image_rssi_eff;
};


/* Channel information which is collected during the channel autoscan
 * procedure
 */
struct fmrx_eval_ch_info {
	u32 freq;
	struct fmrx_ext_ch_rssi hi_rssi;
	struct fmrx_ext_ch_rssi li_rssi;
	u8 inj_side;	/* LO frequency injection side 0->low; 1->high */
	s16 rssi_eff;	/* Effective RSSI of the selected injection side */
	u16 pn;		/* Phase noise of the channel */
	s32 foffs;	/* Frequency offset */
	u16 pilot_ampl; /* amplitude of pilot */
};


/*
 * @brief
 * FM Radio band sweeping parameters
 */
struct fmrx_band_sweep {
	u16 maxcnt;
	struct fmrx_eval_ch_info *p_report;
};


/*
 * @brief
 * FM Radio auto evaluation parameters.
 */
struct fmrx_ae_params {
	u32 min;
	u32 max;
	u32 step;
	u16 maxcnt;
	enum fmtrx_ant_type ant_type;
	struct fmtrx_rssi_report *scan_report;
};

/*
 * @brief
 * FM Radio auto evaluation parameters.
 */
struct fmrx_ae_progress {
	u32 scan_count;
	u32 total_count;
};

struct fmrx_sb_sel {
	enum fmrx_sb_inj_side inj_side;
	enum fmtrx_inj_sel sb_sel;
};


#endif	/* End of file */

