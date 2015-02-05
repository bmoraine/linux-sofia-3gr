/*
 * Copyright (C) 2013 Intel Mobile Communications GmbH
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
** =============================================================================
**
**				INTERFACE DESCRIPTION
**
** =============================================================================
*/
/**
 * @file fmtrx_common.h
 *
 * This file contains interfaces that are common to all modules.
 *
 **/

#ifndef	_FM_TRX_COMMON_H_
#define	_FM_TRX_COMMON_H_

/*
** =============================================================================
**
**				INCLUDE	STATEMENTS
**
** =============================================================================
*/

/*
** =============================================================================
**
**				DEFINES
**
** =============================================================================
*/
#define RSSI_TO_DBUV(X) (X / 4)
#define RSSI_TO_0_25_DBUV(X) (X * 4)
#define STEP_SIZE_TO_LINEAR(X) (X / 4)

#define KHZ_TO_62_5_KHZ(X) (X * 10 / 625)
#define KHZ_62_5_TO_KHZ(X) KHZ_ROUNDED((X * 625 / 10))
#define KHZ_ROUNDED(X) (X - (X % 50))

#define HZ_TO_62_5_HZ(X) KHZ_TO_62_5_KHZ(X)
#define HZ_62_5_TO_HZ(X) KHZ_62_5_TO_KHZ(X)

#define HZTOKHZ(X) (X / 1000)
#define KHZTOHZ(X) (X * 1000)

#ifdef FMR_HOST_TEST
#define FMTRX_TIMEOUTS 0
#define FMTRX_RDS_FAST_PI_TIMEOUTS 0
#define FMTRX_RDS_TIMEOUTS 0
#else
#define FMTRX_TIMEOUTS 5000
#define FMTRX_RDS_FAST_PI_TIMEOUTS 1000
#define FMTRX_RDS_TIMEOUTS 1000
#endif
#define MAX_OFFSETS 16

/*
** =============================================================================
**
**				EXPORTED ENUM DEFINITIONS
**
** =============================================================================
*/
enum antenna_type {
	ANTENNA_HS_SINGLEEND,
	ANTENNA_HS_DIFFERENTIAL,
	ANTENNA_EBD_SINGLEEND,
	ANTENNA_EBD_DIFFERENTIAL,
	ANTENNA_INVALID
};

enum lna_out_gain {
	GAIN_9DB_REDUC,
	GAIN_6DB_REDUC,
	GAIN_3DB_REDUC,
	GAIN_0DB_REDUC,
	GAIN_INVALID
};

enum gain_offset_type {
	GAIN_OFFSET_LNA,
	GAIN_OFFSET_PPF,
	GAIN_OFFSET_RSSI,
	GAIN_OFFSET_CP_INIT,
	GAIN_OFFSET_EXT_LNA,
	GAIN_OFFSET_INVALID
};

enum deemphasis_type {
	DEEMP_50US,
	DEEMP_75US,
	DEEMP_INVALID
};

enum routing_mode {
	ROUTING_DAC,
	ROUTING_SRC,
	ROUTING_INVALID
};

enum agc_gain_index {
	AGC_GAIN_INDEX_0,
	AGC_GAIN_INDEX_1,
	AGC_GAIN_INDEX_2,
	AGC_GAIN_INDEX_3,
	AGC_GAIN_INDEX_4,
	AGC_GAIN_INDEX_5,
	AGC_GAIN_INDEX_6,
	AGC_GAIN_INDEX_7,
	AGC_GAIN_INDEX_8,
	AGC_GAIN_INDEX_9,
	AGC_GAIN_INDEX_10,
	AGC_GAIN_INDEX_11,
	AGC_GAIN_INDEX_12,
	AGC_GAIN_INDEX_13,
	AGC_GAIN_INDEX_14,
	AGC_GAIN_INDEX_15,
	AGC_GAIN_INVALID
};

enum rds_type {
	RDS_NORMAL,
	RDS_RBDS,
	RDS_MMBS,
	RDS_INVALID
};

enum fmtrx_type {
	FMTRX_RX,
	FMTRX_TX,
	FMTRX_TRX,
	FMTRX_INVALID
};

enum injection_side {
	INJECTION_SIDE_HSI,
	INJECTION_SIDE_LSI,
	INJECTION_SIDE_AUTO,
	INJECTION_SIDE_INVALID
};

enum rds_onmode {
	RDS_ONMODE_OFF,
	RDS_ONMODE_ON,
	RDS_ONMODE_RETAIN,
	RDS_ONMODE_INVALID
};

enum fmtrx_init_mode {
	FMTRX_INIT_MODE_ON,
	FMTRX_INIT_MODE_OFF,
	FMTRX_INIT_MODE_INVALID
};

enum tune_status {
	FMRX_TUNE_STATUS_OK,
	FMRX_TUNE_STATUS_FAILED,
	FMRX_TUNE_STATUS_ONGOING,
	FMRX_STATUS_INVALID
};

enum fmrx_state {
	FMRX_HW_STATE_IDLE,
	FMRX_HW_STATE_RX_ACTIVE,
	FMRX_HW_STATE_RX_SEEKING,
	FMRX_HW_STATE_INVALID
};

enum audio_channel {
	FMRX_AUD_CHN_ALL,
	FMRX_AUD_CHN_LEFT,
	FMRX_AUD_CHN_RIGHT,
	FMRX_AUD_CHN_INVALID
};

enum seek_mode {
	FMRX_SEEK_MODE_OFF,
	FMRX_SEEK_MODE_UP,
	FMRX_SEEK_MODE_DOWN,
	FMRX_SEEK_MODE_UP_TO_LIMIT,
	FMRX_SEEK_MODE_DOWN_TO_LIMIT,
	FMRX_SEEK_MODE_AUTO,
	FMRX_SEEK_MODE_INVALID
};

enum fmtrx_v4l2_cid_private {
	/* Private Controls */
	V4L2_CID_PRIV_INTEL_FMRX_OUTPUT_MODE = (V4L2_CID_PRIVATE_BASE + 1),
	V4L2_CID_PRIV_INTEL_FMRX_ANTENNA,
	V4L2_CID_PRIV_INTEL_FMRX_SB,
	/* Private IOCTLS */
	V4L2_CID_PRIV_INTEL_FMRX_BAND,
	V4L2_CID_PRIV_INTEL_FMRX_SNC,
	V4L2_CID_PRIV_INTEL_FMRX_SM,
	V4L2_CID_PRIV_INTEL_FMRX_AGC,
	V4L2_CID_PRIV_INTEL_FMRX_RSSI,
	V4L2_CID_PRIV_INTEL_FMRX_SET_HWPARAMS,
	V4L2_CID_PRIV_INTEL_FMRX_GET_HWPARAMS,
	V4L2_CID_PRIV_INTEL_FMRX_CHANNEL_INFO,
	V4L2_CID_PRIV_INTEL_FMRX_STATIC_CFG,
	V4L2_CID_PRIV_INTEL_FMRX_SET_OFFSETS,
	V4L2_CID_PRIV_INTEL_FMRX_GET_OFFSETS,
	V4L2_CID_PRIV_INTEL_FMRX_SET_RSSI_OFFSETS,
	V4L2_CID_PRIV_INTEL_FMRX_GET_RSSI_OFFSETS,
	V4L2_CID_PRIV_INTEL_FMRX_AUTO_SEEK_REPORT,
	V4L2_CID_PRIV_INTEL_FMRX_RDS_DATA,
	V4L2_CID_PRIV_INTEL_FMRX_NVM_COMMIT,
	V4L2_CID_PRIV_INTEL_FMRX_AF_SWITCH,
	V4L2_CID_PRIV_INTEL_FMTRX_READ,
	V4L2_CID_PRIV_INTEL_FMTRX_WRITE,
	V4L2_CID_PRIV_INTEL_FMTRX_GET_CORE_VERSION,
	V4L2_CID_PRIV_INTEL_FMTRX_RDS,
	V4L2_CID_PRIV_INTEL_FMTRX_MAX
};


/*
** =============================================================================
**
**				EXPORTED STRUCT	DEFINITIONS
**
** =============================================================================
*/
struct channel_dbg_info {
	u32 frequency;
	u32 lo_freq;
	u16 vco_freq;
	u16 inj_side;
	s16 lsi_rssi;
	s16 lsi_img_rssi;
	s16 hsi_rssi;
	s16 hsi_img_rssi;
	s32 foffs;
	s16 pn;
};

struct channel_info {
	enum fmrx_state state;
	u32 frequency;
	bool is_stereo;
	s16 rssi;
	enum injection_side inj_side;
	s16 pn;
	s32 foffs;
	u16 pilot_amplitude;
	bool rds_enable;
	bool is_rds_sync_enabled;
};

struct fmr_id {
	u32 hw_id;
	u32 fw_id;
	u32 fw_timestamp;
	u8 hw_sw_ver[10];
};

struct fmrx_callbacks {
	int (*p_fmrx_channel_tuned_cb)(
	    enum tune_status state,
	    u32 frequency,
	    s16 rssi,
	    enum injection_side side);

	int (*p_fmrx_rds_cb)(
	    u16 filled_cnt,
	    u16 overflow_cnt);
};

struct rds_block {
	u16 data;
	u8 block;
} __packed;

struct rds_group {
	struct rds_block blocka;
	struct rds_block blockb;
	struct rds_block blockc;
	struct rds_block blockd;
};

struct rds_group_pkt {
	struct rds_group *data;
	u16 requested_groups;
	u16 received_groups;
};

struct reg_info {
	u32 addr_offs;
	s32 *data;
};

struct band {
	u32 max;
	u32 min;
	u32 step;
	enum deemphasis_type deemp;
};

struct agc {
	bool enable;
	enum agc_gain_index idx;
} __packed;

struct rssi_notify {
	bool enable;
	s16 lo_thr;
	s16 hi_thr;
} __packed;

struct snc {
	bool enable;
	s16 lo_thr;
	s16 hi_thr;
} __packed;

struct sm {
	bool enable;
	s16 thr;
	u16 step;
} __packed;

struct volume {
	u8 left;
	u8 right;
};

struct rssi_offsets {
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
};

struct rssi_offsets_pkt {
	enum antenna_type antenna;
	enum gain_offset_type type;
	struct rssi_offsets offsets;
};

struct gain_offsets_pkt {
	enum antenna_type antenna;
	enum gain_offset_type type;
	u16 offsets[MAX_OFFSETS];
};

struct other_params {
	u16 pn_thr;
	enum lna_out_gain lna_type;
	s16 volume_ramp;
	u32 clk_switch_range_104;
	s16 int_rssi_other_offset;
	s16 ext_rssi_other_offset;
	s16 seek_thr;
} __packed;

struct rds {
	enum rds_onmode mode;
};

struct af_info {
	u32 count;
	u32 *freq_list;
	s16 pi_code;
};

struct fmrx_config {
	struct band band_cfg;
	enum antenna_type antenna;
	enum routing_mode routing;
	struct agc agc_cfg;
	struct rssi_notify rssi_cfg;
	bool force_mono;
	bool mute;
	struct snc snc_cfg;
	struct sm sm_cfg;
	enum injection_side side;
	struct volume vol_cfg;
	struct rds rds_cfg;
	struct other_params other_cfg;
	struct rssi_offsets int_rssi_offsets;
	struct rssi_offsets ext_rssi_offsets;
	struct rssi_offsets int_ext_lna_offsets;
	struct rssi_offsets ext_ext_lna_offsets;
	u16 int_lna_offsets[MAX_OFFSETS];
	u16 ext_lna_offsets[MAX_OFFSETS];
	u16 int_ppf_offsets[MAX_OFFSETS];
	u16 ext_ppf_offsets[MAX_OFFSETS];
	u16 cp_init_offsets[MAX_OFFSETS];
} __packed;

struct af_info_32 {
	u32 count;
	u32 freq_list;
	s16 pi_code;
};

struct reg_info_32 {
	u32 addr_offs;
	u32 data;
};

struct rds_group_pkt_32 {
	u32 data;
	u16 requested_groups;
	u16 received_groups;
};

/*
** =============================================================================
**
**				EXPORTED FUNCTION DECLARATIONS
**
** =============================================================================
*/
/* Initialize the FM Core module
 * @mode Type of mode (ON/OFF)
 * @type Type of module (RX/TX/RTX)
 */
int fmtrx_init(
	enum fmtrx_init_mode mode,
	enum fmtrx_type type);

/* Register callbacks for event notifications
 * @data Pointer to the callback structure
 */
int fmrx_register_callbacks(
	struct fmrx_callbacks *data);

/* Register callbacks for event notifications
 * @state Pointer to the state of FMTRX
 */
int fmrx_get_hw_state(
	enum fmrx_state *state);

/* Power on the FMR IP - module RX
 */
int fmrx_power_on(
	void);

/* Power off the FMR IP - module RX
 */
int fmrx_power_off(
	void);

/* Set FM RX band configuraiton
 * @cfg Pointer to the band informaiton
 */
int fmrx_set_band(
	struct band *data);

/* Get FM RX band configuraiton
 * @cfg Pointer to the band informaiton
 */
int fmrx_get_band(
	struct band *data);

/* Get FM RX channel information
 * @cfg Pointer to the channel informaiton
 */
int fmrx_get_channel_info(
	struct channel_info *data);

/* Get FM RX tuned frequency
 * @tuned_frequency Pointer to the tuned channel frequency
 */
int fmrx_get_channel_freq(
	u32 *tuned_frequency);

/* Get FM RX static configuration
 * @cfg Pointer to the static data
 */
int fmrx_get_config(
	struct fmrx_config **data);

/* Set FM RX static configuration
 * @cfg Pointer to the static data
 */
int fmrx_set_config(
	struct fmrx_config *data);

/* Force to play the channel in mono mode
 * @force_mono Enable or disable mono mode
 */
int fmrx_set_force_mono(
	bool force_mono);

/* Tune to a station
 * @frequency Wanted channel
 */
int fmrx_station_tuning(
	u32 frequency);

/* Set volume level for both or left or right channels
 * @type Type of audio channel
 * @level Value between 0 to 100
 */
int fmrx_set_volume_level(
	enum audio_channel type,
	u8 level);

/* Set mute for both or left or right channels
 * @type Type of audio channel
 * @enable Mute or Unmute
 */
int fmrx_set_mute(
	enum audio_channel type,
	bool enable);

/* Search up/down station
 * @mode Type of seek mode
 */
int fmrx_station_seeking(
	enum seek_mode mode);

/* Cancel any ongoing operation like seek/auto evaluate/etc.
 */
int fmrx_stop(
	void);

/* Set routing mode
 * @routing DAC or SRC mode
 */
int fmrx_set_route(
	enum routing_mode routing);

/* Select which antenna to use
 * @antenna Type of antenna - internal/external
 */
int fmrx_set_antenna(
	enum antenna_type antenna);

/* Select side band
 * @side High or Low or Auto side injection
 */
int fmrx_set_sideband(
	enum injection_side side,
	bool force);

/* Set Stereo noise cancellation
 * @data Pointer to SNC configuration structure
 */
int fmrx_set_snc(
	struct snc *data);

/* Set Soft mute configuration
 * @data Pointer to the SM configuration structure
 */
int fmrx_set_sm(
	struct sm *data);

/* Set AGC configuration
 * @data Pointer to AGC configuration structure
 */
int fmrx_set_agc(
	struct agc *data);

/* Set RSSI notification
 * @data Pointer to RSSI configuration structure
 */
int fmrx_set_rssi_notification(
	struct rssi_notify *data);

/* Set other parameters
 * @data Pointer to other parameters
 */
int fmrx_set_other_params(
	struct other_params *data);

/* Set Gain RSSI offsets for internal/external LNA
 * @data Pointer to gain offset
 */
int fmtrx_set_gain_rssi_offsets(
	struct rssi_offsets_pkt *data);

/* Get Gain RSSI offsets for internal/external LNA
 * @data Pointer to gain offset
 */
int fmtrx_get_gain_rssi_offsets(
	struct rssi_offsets_pkt *data);

/* Set Gain offsets like LNA, PPF, CP
 * @data Pointer to gain offset
 */
int fmtrx_set_gain_offsets(
	struct gain_offsets_pkt *data);

/* Get Gain offsets like LNA, PPF, CP
 * @data Pointer to gain offset
 */
int fmtrx_get_gain_offsets(
	struct gain_offsets_pkt *data);

/* Get RDS groups
 * @groups_to_read Number of groups to be fetched from the FMR IP memory
 * @data Pointer to the RDS groups buffer
 * @groups_read Number of actual groups returned
 */
int fmrx_get_rds_groups(
	struct rds_group_pkt *data);


/* Set RDS configuration
 * @data Pointer to RDS configuration
 */
int fmrx_set_rds(
	struct rds *data);

/* Get current running core module version
 * @data Pointer to FMRID
 */
int fmtrx_get_id(
	struct fmr_id *data);

/* Evaluate and switch to a valid alternative frequency
 * @data Pointer to AF info
 */
int fmrx_af_switch(
	struct af_info *data);

#endif	/* _FM_TRX_COMMON_H_	*/
