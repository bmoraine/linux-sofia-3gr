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
 * @file aud_app_fmr_hld_rx.h
 *
 * This file contains the FM Rx types and description of FM Rx exported API
 *
 **/

#ifndef AUD_APP_FMR_HLD_RX_H
#define AUD_APP_FMR_HLD_RX_H

#include <types.h>
#include <aud_app_fmr_hld_api_types_common.h>
#include <aud_app_fmr_hld_api_types_rx.h>
#include <aud_app_fmr_hld_msg.h>
#include <aud_app_fmr_lld_api_rx.h>

#define MAX_OFFSETS		16
#define OFFS_ARRAY_SIZE		(sizeof(u16) * MAX_OFFSETS)
#define CP_INIT_TABLE_LEN	16

/*
 * @brief
 * Enumeration for FM Rx states
 */
enum fmrx_sm_state {
	FMRX_SM_STATE_INVALID,

	/*
	 * The FM radio resource is in Idle state, it is disabled (powered off)
	 * Configuration cmds (e.g. setting frequency, band limits, etc. can be
	 * performed and will take effect at power-on.
	 */
	FMRX_SM_STATE_IDLE,

	/*
	 * Audio either turned on (LISTENING) or turned off (MUTED) depending
	 * on state variable. RSSI events are generated as requested; host
	 * responsibility to change from MUTED to LISTENING if required
	 * (e.g. if RSSI increases.
	 * Note: recommended to perform explicit retune to current channel in
	 * this case to avoid unmuting due to image channel).
	 *
	 * When the resource is in Rx Active state, power-down is not allowed.
	 *
	 * RDS events are enabled, if stereo signal is found and RDS is enabled.
	 * Stereo mode is enabled, if stereo signal is found and mono mode is
	 * not selected.
	 *
	 * Before leaving this state, the audio is always muted.
	 *
	 * A periodic timer function is enabled to check calibration.
	 */
	FMRX_SM_STATE_ACTIVE,  /* FMRX_SM_STATE_NORMAL_RECEIVE */

	/*
	 * Scanning through frequencies looking for the requested RSSI level;
	 * updates the frequency state variable. On reaching the upper or lower
	 * band limit, searching resumes at the other band limit. If the
	 * specified RSSI threshold is met, or if the frequency returns to
	 * beyond the starting frequency, then the receiver returns to
	 * RECEIVING state. On re-entering RECEIVING state, an RSSI event is
	 * generated corresponding to the current signal strength. Calibration
	 * always performed before returning to RECEIVING state.
	 */
	FMRX_SM_STATE_SEEK_ACTIVE, /*  The resource is in Rx seeking state */

	/*
	 * Scanning through a list of specified frequencies to determine RSSI,
	 * returning as quickly as possible to the current frequency (state
	 * variable) with minimum disruption. RSSI event for the current
	 * frequency is generated on returning to the RECEIVING state.
	 */
	FMRX_SM_STATE_AUTO_EVAL,

	/*
	 * When FM RX RDS is active
	 */
	FMRX_SM_STATE_RDS_ACTIVE,
};


/*
 * @brief
 *  Enums used to indicate FM Rx Events
 *  API cmds which result in a message sending to FM Rx module
 */
enum fmrx_event {
	FMRX_EVENT_POWER_ON,			/* 0 */
	FMRX_EVENT_POWER_OFF,

	FMRX_EVENT_TUNE_STATION,
	FMRX_EVENT_SEEK_STATION,
	FMRX_EVENT_AUTO_EVAL_GET_CAP,
	FMRX_EVENT_AUTO_EVAL,			/* 5 */
	FMRX_EVENT_AUTO_EVAL_STOP,
	FMRX_EVENT_AUTO_EVAL_PROGRESS,

	FMRX_EVENT_AF_MEASURE_RSSI,
	FMRX_EVENT_AF_EVALUATE,
	FMRX_EVENT_SWITCH_ANT,			/* 10 */

	FMRX_EVENT_REGISTER_CBS,
	FMRX_EVENT_TRANSFER_RDS_GROUPS,

	FMRX_EVENT_SET_BAND,
	FMRX_EVENT_SET_ROUTE,
	FMRX_EVENT_SET_SNC,			/* 15 */
	FMRX_EVENT_SET_CONFIG_DATA,
	FMRX_EVENT_SET_VOLUME,
	FMRX_EVENT_SET_MUTE,
	FMRX_EVENT_SET_FORCE_MONO,
	FMRX_EVENT_SET_RDS_CONFIG,		/* 20 */
	FMRX_EVENT_SET_SOFT_MUTE,
	FMRX_EVENT_SET_AGC_GAIN,
	FMRX_EVENT_SET_RSSI_OFFSET,
	FMRX_EVENT_SET_DEEMPHASIS,
	FMRX_EVENT_SET_EXT_LNA_RSSI_OFFSET,	/* 25 */
	FMRX_EVENT_SET_EXT_LNA_RSSI_COMPEN,
	FMRX_EVENT_SET_IDI_HANDSHAKE,
	FMRX_EVENT_SET_SIDEBAND,

	FMRX_EVENT_GET_VOLUME,
	FMRX_EVENT_GET_CHANNEL_INFO,		/* 30 */
	FMRX_EVENT_GET_FREQ_BAND_INFO,
	FMRX_EVENT_GET_RSSI,
	FMRX_EVENT_GET_RSSI_OFFSET,
	FMRX_EVENT_GET_REVISIONS,
	FMRX_EVENT_GET_RX_STATE,		/* 35 */
	FMRX_EVENT_GET_CONFIG_DATA,
	FMRX_EVENT_GET_EXT_LNA_RSSI_COMPEN,

	FMRX_EVENT_SUBSCRIBE_RSSI,
	FMRX_EVENT_UNSUBSCRIBE_RSSI,
	FMRX_EVENT_SUBSCRIBE_RDS,		/* 40 */
	FMRX_EVENT_UNSUBSCRIBE_RDS,
	FMRX_EVENT_SUBSCRIBE_RDS_SYNC,
	FMRX_EVENT_UNSUBSCRIBE_RDS_SYNC,

	FMRX_EVENT_TEST_DUMP_FW_REG,
	FMRX_EVENT_TEST_WITH_SINE_WAVE_GEN,	/* 45 */

	/* FM Rx interrupts de-coupling */
	FMRX_EVENT_DC_RDS_CB,
	FMRX_EVENT_DC_RDS_FASTPI_CB,
	FMRX_EVENT_DC_RDSSYNC_CB,
	FMRX_EVENT_DC_PILOT_CB,
	FMRX_EVENT_DC_RSSI_CB,			/* 50 */

	/* FM Rx internal message for state updating */
	FMRX_EVENT_SEEKING4NEXT,
	FMRX_EVENT_SEEKOREVAL_DONE,
	FMRX_EVENT_EVAL4NEXT,
	FMRX_EVENT_AF_EVAL4NEXT,

	/* Common events for FM Rx and FM Tx */
	FMRX_EVENT_ANTENNA_SELFTEST,		/* 55 */

	/* Register events */
	FMRX_EVENT_REG_READ,
	FMRX_EVENT_REG_WRITE,

	FMRX_EVENT_END				/* 58 */
};

/*
 * @brief
 * FM Radio HW state
 */
enum fmrx_hw_state {
	FMRX_HW_STATE_IDLE,
	FMRX_HW_STATE_ACTIVE,
	FMRX_HW_STATE_SEEK_ACTIVE,
	FMRX_HW_STATE_INVALID
};

enum gain_offset_type {
	GAIN_OFFSET_LNA,
	GAIN_OFFSET_PPF,
	GAIN_OFFSET_CP_INIT,
	GAIN_OFFSET_RSSI,
	GAIN_OFFSET_INVALID
};

struct rssi_offs {
	u32 frequency1;
	u32 frequency2;
	u32 frequency3;
	u32 frequency4;
	u32 frequency5;
	u16 offset1;
	u16 offset2;
	u16 offset3;
	u16 offset4;
	u16 offset5;
	u16 offset6;
} __packed;

struct rssi_offsets {
	enum fmtrx_ant_type antenna;
	enum gain_offset_type off_type;
	struct rssi_offs offsets;
};

struct gain_offsets {
	enum fmtrx_ant_type antenna;
	enum gain_offset_type off_type;
	s16 offsets[MAX_OFFSETS];
};

/*
 * @brief
 * Structure defining the FMRadio IP configuration for reception
 */
struct fmrx_cfg {
	struct fmtrx_band band;
	u8 force_mono;/* 1 - to force mono -Disables stereo, if ch is stereo */
	u8 antenna;		/* 0 is internal, 1 external */
	s16 seek_rssi;		/* seek rssi */
	u8 aud_path;		/* 0 is DAC mode, 1 is DSP mode */
	struct fmrx_agc agc_cfg;/* AGC Gain control configuration */
	struct rssi_notify rssi_cfg;
	u8 mute;		/* 1 - Volume muted, 0 - Unmuted */
	struct fmrx_snc snc;
	struct fmrx_sm sm_cfg;
	u8 rf_force_sb;	/* Force sideband for debugging; 0 means not forced */
	struct volume vol_cfg;
	struct rds rds_cfg;
	struct other_params other_cfg;
	struct rssi_offs rssi_offs_int_ant;
	struct rssi_offs rssi_offs_ext_ant;
	u16 lna_offs_int_ant[MAX_OFFSETS];
	u16 lna_offs_ext_ant[MAX_OFFSETS];
	u16 ppf_offs_int_ant[MAX_OFFSETS];
	u16 ppf_offs_ext_ant[MAX_OFFSETS];
	u16 cp_init_offs[MAX_OFFSETS];
	struct fmrx_ext_lna_cfg ext_lna_cfg[FMR_ANT_TYPE_END];
} __packed;

struct fmrx_misc_cfg {
	u16 blend_max;

	s32 rssi_enable;	/* Enables RSSI fmrx_cbs */
	s16 rssi_lower;		/* Lower bound for RSSI reporting */
	s16 rssi_upper;		/* Upper bound for RSSI reporting */
	s32 rssi_verbose;	/* Enables verbose RSSI reporting */

	s32 rdssync_en;		/* Enables / disables RDS Sync */
	s32 rds_sync;
	u16 rds_min_free;	/* Free spaces in FMR-internal group buffer */
	enum fmtrx_rds_mode rds_type;	/* Defines RDS / RBDS */
	enum fmrx_rds_pi_mode rds_pi_mode;/* Fast_PI mode or normal mode */
	enum fmtrx_subscribe_rds_mode rds_mode;
	struct fmrx_rds_sync_cfg rds_sync_cfg;

	u16 ant_selftest_low_thr;
	u16 ant_selftest_high_thr;

	/* Pointer to all callback function pointers */
	const struct fmrx_callbacks *fmrx_cbs;
} __packed;

/*
 * @brief
 * Structure defining the state of FM Radio reciever
 */
struct fmrx_state {
	u32 freq;	/* current channel freq */

	/* Whether the internal RF clock is used for digital clock when tuning
	 * station is around 104 MHz */
	enum fmrx_clk_switch rf_clk;

	enum fmtrx_seek_dir seek_mode;	/* Current seek direction */
	u32 seek_step;	/* Frequency step for seeking */
	u32 seek_start;	/* Starting freq for seek */
	u16 scan_idx;	/* Index used in AF evaluation / auto-scan */
	u16 scan_max;	/* Number of entires in scan array */
	u32 scan_step;
	u32 scan_band_min;
	u32 scan_band_max;
	enum fmtrx_ant_type scan_ant;
	enum fmrx_scan_status scan_status;/* record the scan state for scan */

	/* Data structure where AF evaluation/auto-scan data will be stored */
	struct fmtrx_rssi_report *scan_array;

	s32 pilot_found;	/* Flag to indicate pilot detected */

	enum fmtrx_rds_mode rds_type_bkp;
	enum fmrx_rds_mode rds_en_bkp;
	enum fmrx_rds_pi_mode rds_pi_mode_bkp;
	enum fmtrx_subscribe_rds_mode rds_mode_bkp;

	u8 af_eval_state; /* State of AF evaluation */
	s32 idi_hs_ctrl;   /* State of FMR IDI handshake control bit */

	/* Results from last channel evaluation */
	struct fmrx_eval_ch_info ch_info;

	u16 af_pi_code; /* PI code to be validated during AF evaluation */

	struct fmrx_cfg *cfg; /* configuration from userspace*/
	struct fmrx_misc_cfg *mcfg; /* misc configuration in the driver */

	/* FM Radio receiver states */
	enum fmrx_sm_state curr_sm_state;
	enum fmrx_sm_state prev_sm_state;
} __packed;

/*
 * Union that holds the possible combinations of parameters sent from the
 * audio application dispatcher.
 */
union fmrx_ev_prams {
	struct fmrx_tune_station		ch_tune;
	enum fmrx_aud_route			aud_routing;
	enum fmtrx_ant_type			ant_type;
	struct fmtrx_ant_selftest_freq		ant_selftest_freq;
	struct fmrx_snc				*p_snc_cfg;
	struct fmrx_seek_station_params		*p_seek_station;
	struct fmrx_ae_params			*p_ae_info;
	struct fmtrx_band			*p_sweep_band;
	struct fmrx_ae_progress			ch_ae_stat;
	struct fmrx_band_sweep			band_sweep;
	struct fmrx_af_measure_rssi		get_af_rssi;
	struct fmrx_af_eval			af_eval_params;
	struct fmrx_pilot_state			pilot_ev;
	struct fmrx_rssi_ev_info		rssi_ev;
	struct fmrx_rds_sync_ev_info		rds_sync_ev;
	struct fmrx_rds_pi_code			rds_fastpi_ev;
	struct fmrx_rds_ev_info			fmrx_rds_ev_info;
	struct fmrx_trace_pt_sel		trace_ev;
	struct fmrx_cfg				*p_state_data;
	struct fmrx_callbacks			*p_reg_cbs;
	struct fmrx_cfg				*p_set_cfg;
	struct fmrx_set_aud_ch_vol		set_ch_vol;
	struct fmtrx_band			*p_set_freq_band;
	struct fmrx_set_mute			set_mute;
	struct fmrx_set_force_mono		force_mono;
	struct fmrx_rds_cfg			*p_set_rds_cfg;
	struct fmrx_sm				*p_set_sm_cfg;
	struct fmrx_agc				*p_agc_gain_cfg;
	struct fmrx_rssi_offsets		set_rssi_offs;
	enum fmtrx_emph_sel			set_deem;
	struct fmrx_set_ext_lna_rssi_off	set_ext_lna_cfg;
	struct fmrx_set_ext_lna_rssi_comp	set_ext_lna_rssi_comp;
	struct fmrx_get_ext_lna_rssi_comp	get_ext_lna_rssi_comp;
	struct fmrtrx_set_id_hs_params		set_idi_hs_flag;
	struct fmrx_get_rds_grps		get_rds_grps;
	struct fmrx_get_aud_ch_vol		get_ch_vol;
	struct fmrx_ch_info			*p_ch_info;
	struct fmtrx_band			*p_get_freq_band;
	s16					*p_rssi;
	s16					*p_get_rssi_offset;
	struct fmtrx_revision			*p_get_rev_ids;
	struct fmrx_cfg				**p_get_cfg_data;
	struct fmrx_rssi_subsc			rssi_subscribe;
	struct fmrx_rds_subscribe_params	*rds_subscribe;
	struct fmrx_check_watchdog_status	check_wd_status;
	struct fmrx_sb_sel			rf_sb_sel;
	struct fmrx_fw_reg			*p_dump_regs;
	struct fmtrx_reg_data			*p_read_reg;
	struct fmtrx_reg_data			*p_write_reg;
};


/*
 * @brief
 * Struct that contains the event and the neccesary parameters when sending a
 * event/message from the fmr dispatcher to FM Rx event handler dispatcher
 */
struct fmrx_msgbox_buff {
	enum fmrx_event event;  /* Event to FM Rx event handler */
	union fmrx_ev_prams params; /* Parameters for the event */
} __packed;

int fmrx_init(enum fmtrx_init_mode fmrx_init_mode);
int fmrx_event_dispatcher(struct fmrx_msgbox_buff *p_fmrx_msg);
enum fmrx_hw_state fmrx_get_hw_state(void);
void fmrx_irq_handler(u32 int_status);
int fmtrx_set_gain_offsets(struct gain_offsets *gain_off);
int fmtrx_get_gain_offsets(struct gain_offsets *gain_offs);
int fmtrx_set_gain_rssi_offsets(struct rssi_offsets *rssi_offs);
int fmtrx_get_gain_rssi_offsets(struct rssi_offsets *rssi_offs);

#endif

/* end of file */
