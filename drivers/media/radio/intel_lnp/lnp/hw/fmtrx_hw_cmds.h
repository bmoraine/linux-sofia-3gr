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
 * @file aud_app_fmr_lld_api_rx.h
 *
 * This file defines the API provided by the FM radio RX low level driver.
 *
 **/

#ifndef	_FM_TRX_HW_CMDS_H_
#define	_FM_TRX_HW_CMDS_H_

/*
** =============================================================================
**
**				INCLUDE STATEMENTS
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

/*
** =============================================================================
**
**				EXPORTED ENUM	DEFINITIONS
**
** =============================================================================
*/
enum interrupt_position {
	POS_IR_HWINT  = 0,
	POS_IR_SWINT0 = 1,
	POS_IR_SWINT1 = 2,
	POS_IR_SWINT2 = 3,
	POS_IR_SWINT3 = 4,
	POS_IR_SWINT4 = 5,
	POS_IR_BRK    = 6,
	POS_IR_BUSERR = 7,
	POS_IR_DED0   = 8,
	POS_IR_DED1   = 9,
	POS_IR_DED2   = 10,
	POS_IR_DED3   = 11
};

enum interrupt_type {
	IR_HWINT            = 1 << POS_IR_HWINT,
	/* Software Interrupt 0 */
	IR_SWINT0           = 1 << POS_IR_SWINT0,
	IR_RX_RSSI          = 1 << POS_IR_SWINT0,
	IR_TX_ANT_TRACKING  = 1 << POS_IR_SWINT0,
	/* Software interrupt 1 */
	IR_SWINT1           = 1 << POS_IR_SWINT1,
	IR_RX_RDS           = 1 << POS_IR_SWINT1,
	IR_TX_RDS           = 1 << POS_IR_SWINT1,
	/* Software interrupt 2 */
	IR_SWINT2           = 1 << POS_IR_SWINT2,
	IR_RX_RDSSYNC       = 1 << POS_IR_SWINT2,
	/* Software interrupt 3 */
	IR_SWINT3           = 1 << POS_IR_SWINT3,
	IR_RX_PILOT         = 1 << POS_IR_SWINT3,
	/* Software interrupt 4 */
	IR_SWINT4           = 1 << POS_IR_SWINT4,
	IR_RX_JOINT         = 1 << POS_IR_SWINT4,
	IR_TX_JOINT         = 1 << POS_IR_SWINT4,
	/* Hardware break */
	IR_BRK              = 1 << POS_IR_BRK,
	/* Bus error */
	IR_BUSERR           = 1 << POS_IR_BUSERR,
	/* Dedicate interrupt 0 */
	IR_INTDED0          = 1 << POS_IR_DED0,
	IR_TRX_CMD          = 1 << POS_IR_DED0,
	/* Dedicate interrupt 1 */
	IR_INTDED1          = 1 << POS_IR_DED1,
	IR_TRX_CMD2         = 1 << POS_IR_DED1,
	/* Dedicate interrupt 2 */
	IR_INTDED2          = 1 << POS_IR_DED2,
	/* Dedicate interrupt 3 */
	IR_INTDED3          = 1 << POS_IR_DED3,
	IR_TRX_TRACE        = 1 << POS_IR_DED3
};

/*
** =============================================================================
**
**				EXPORTED STRUCT DEFINITIONS
**
** =============================================================================
*/
struct dsp_ch_info {
	u32 ch_freq;
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

struct dsp_rds_group {
	s16 blockb;
	s16 blockc;
	s16 blockd;
	s16 status;
};

struct dsp_rds_buf_info {
	u16 size;
	u16 buf_start;
	u16 buf_end;
	u16 host_read_ptr;
	u16 fw_write_ptr;
};

struct dsp_rf_poweron_cmd_params {
	u16 antenna_type;
	u16 lna_out_gain;
	u16 reg_vdd;
	u16 reserved;
};

struct dsp_band_cfg_cmd_params {
	u32 lower_band_limit;
	u32 higher_band_limit;
};

struct dsp_agc_gain_cmd_params {
	u16 agc_en;
	u16 fixed_gain_idx;
	u16 reserved1;
	u16 reserved2;
};

struct dsp_clk_sel_cmd_params {
	u16 clk_sel;
	u16 reserved1;
	u16 reserved2;
	u16 reserved3;
};

struct dsp_channel_tune_cmd_params {
	u32 channel_freq;
	u16 inj_side_sel;
	u16 rssi_threshold;
	u16 force_meas;
};

struct dsp_channel_search_cmd_params {
	u32 ch_start_freq;
	u32 ch_stop_freq;
	u16 ch_step_size;
	u16 inj_side_sel;
	u16 rssi_thres;
	u16 pn_thres;
	u16 force_meas;
};

struct dsp_if_sel_cmd_params {
	u16 if_sel;
	u16 reserved1;
	u16 reserved2;
	u16 reserved3;
};

struct dsp_rds_cmd_params {
	u16 rds_sync_cnt_good;
	u16 rds_sync_cnt_bad;
	u16 rds_track_cnt_bad;
};

struct dsp_gain_offsets {
	u16 offset1;
	u16 offset2;
	u16 offset3;
	u16 offset4;
	u16 offset5;
	u16 offset6;
	u16 offset7;
	u16 offset8;
	u16 offset9;
	u16 offset10;
	u16 offset11;
	u16 offset12;
	u16 offset13;
	u16 offset14;
	u16 offset15;
	u16 offset16;
};

struct dsp_dummy_params {
	u16 param1;
};

struct dsp_cmd_pkt {
	s16 cmd_id;
	union {
		struct dsp_rf_poweron_cmd_params rf_poweron_params;
		struct dsp_band_cfg_cmd_params band_cfg_params;
		struct dsp_agc_gain_cmd_params agc_params;
		struct dsp_clk_sel_cmd_params clk_sel_params;
		struct dsp_channel_tune_cmd_params channel_tune_params;
		struct dsp_channel_search_cmd_params channel_search_params;
		struct dsp_if_sel_cmd_params if_sel_params;
		struct dsp_gain_offsets offs_params;
		struct dsp_dummy_params dummy_params;
		struct dsp_rds_cmd_params rds_params;
	} cmd_params;
};

/*
** =============================================================================
**
**				EXPORTED FUNCTION DECLARATIONS
**
** =============================================================================
*/

#endif	/* _FM_TRX_HW_CMDS_H_ */

