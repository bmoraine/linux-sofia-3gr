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
 * @file
 *
 * FMR HLD API -  FMR System Interface
 *
 * System Messageing Interface.
 *
 * This file descrbes the abstraction layer of	messageing functionality
 * required by the FMR driver. The messaging abstraction layer is divided into
 * two parts, the system specific part and the FMR driver part.
 *
 * This file only contains the the FMR driver specific part of messageing
 * abstraction layer which should not be changed by platform.
 *
 * The implementation of this part of messaging functionality is part of the
 * FMR driver delivery.
 *
 */

#ifndef AUD_APP_FMR_HLD_MSG_H
#define AUD_APP_FMR_HLD_MSG_H

/*
** ============================================================================
**
**				INCLUDE STATEMENTS
**
** ============================================================================
*/
#include <types.h>
#include "aud_app_fmr_hld_api_types_rx.h"
#include "aud_app_fmr_hld_api_types_common.h"

/*
** ============================================================================
**
**				Type Declarations
**
** ============================================================================
*/

#define MSG_SEND_PARAMS 6


/* @brief
  The state of seeking or tuning.
 */
enum fmrx_tuned_state {
	FMRX_TUNING_DONE    = 0, /* Tuning request completed */
	FMRX_TUNING_FAILED  = 1, /* Tuning failed, not a valid channel */
	FMRX_AUTOSEEK_DONE  = 2, /* Autoseeking done, tune to the fist valid */
	FMRX_AUTOEVAL_DONE  = 3, /* AutoEval done, tune to the start freq */
	FMRX_AFEVAL_DONE    = 4, /* AFE Eval done, tune to the start freq */
	FMRX_SEEK_DONE = 5,      /* Seeking done */
	FMRX_SEEK_FAILED = 6, /* Autoseeking seeking failed, no valid found */
	FMRX_SEEK_STOP = 7    /* Autoseeking/seeking stoped by user */
};

/* @brief
 *  The current power state of FMRx module.
 */
enum fmrx_pow_state {
	FMRX_SWITCHED_ON  = 0,
	FMRX_SWITCHED_OFF = 1
};

/* type of automatic channel evaluation */
enum fmtrx_as_eval_rpt_type {
	FMRX_EVAL_AF_RPT,		/* Report AfEval */
	FMRX_EVAL_AUTO_RPT,		/* Report AutoEval */
	FMRX_SEEK_AUTO_RPT,		/* Report Autoseeking report */
	FMRX_EVAL_AF_MEAS_RSSI_RPT	/* Report AF RSSI measurement */
};

struct fmtrx_msg_params {
	u8 fmr_mod_id; /* To identify whether FM Tx or FM Rx module */
	u16 fmr_mod_msg_id;/* Message id of the Mod identified by fmr_mod_id */
	u32 data[MSG_SEND_PARAMS];/* Event data */
} __packed;

/*
** ============================================================================
**
**		Function Declarations and function pointer types
**
** ============================================================================
*/

/*
 * @brief
 * Data structure with function pointers to be called as notification
 * callbacks.
 * Pointers must be set to NULL if the callback is not used.
 * @see fmrx_register_callbacks
 */
struct fmrx_callbacks {
	/*
	 * notification that seeking is in progress (upwards or downwards) and
	 * an channel has been evaluated.
	 * @param dir	Current direction of seeking.
	 * @param freq	Frequency which has been evaluated
	 * @param rssi	RSSI value of the evaluated frequency
	 * @param pn	PN value of the channel be evaluated
	 */
	int (*p_seek_stepped_cb)(enum fmtrx_seek_dir dir, u32 freq, s16 rssi,
		u16 pn);
	/*
	 * notification after tuning/seeking has  been finished. For seeking
	 * the state indicates if the channel is valid or invalid
	 * @param tune_state Result of tuning
	 * @param freq			 Frequency which is now tuned
	 * @param rssi			 RSSI of the tuned channel
	 */
	int (*p_ch_tuned_cb)(enum fmrx_tuned_state tune_state, u32 freq,
		s16 rssi, enum fmrx_sb_inj_side inj_side);

	/*
	 * notification that automatic evaluation of channel has been finished
	 * and the report is ready.
	 *
	 * @note   The report is not in-order and it may contain a variable
	 *         number of values. Each element in the report needs to be
	 *         checked if it is valid by comparing the reported RSSI.
	 *
	 * @param  rpt_type  reason for this report
	 * @param  report  the list of evalated channels
	 * @param  maxcount  the length of the list (as provided by e.g.
	 *		 #fmrx_af_evaluate() and #fmrx_channel_auto_eval())
	 */
	int (*p_seek_eval_rpt_cb)(enum fmrx_scan_status scan_status,
		enum fmtrx_as_eval_rpt_type rpt_type,
		struct fmtrx_rssi_report *report, u16 maxcount);

	/*
	 * notification after RX power on/off finished
	 * @param  power_state New driver state, either ON or OFF
	 */
	int (*p_pow_onoff_cb)(enum fmrx_pow_state power_state);

	/*
	 * notification after the audio routing has been changed
	 * @param audio_state  New audio routing which FMR driver is using from
	 *                     now.
	 */
	int (*p_aud_route_changed_cb)(enum fmrx_aud_route audio_state);

	/*
	 * notification about rds sync found/lost
	 * @param  synced  true, if sync was found, false otherwise
	 */
	int (*p_rds_synced_cb)(s32 synced);

	/*
	 * Call back function of the rds data transfer in normal mode
	 * @param filled_cnt	No. of RDS groups available in FW ring-buffer
	 * @param overflow_cnt	No of RDS groups be discarded
	 */
	int (*p_rds_cb)(u16 filled_cnt, u16 overflow_cnt);

	/*
	 * Call back function of the rds data transfer in fastPI mode.
	 * @param pi  PI code of the station.
	 * @param err Error information of the PI code
	 */
	int (*p_rds_fastpi_cb)(u16 pi, u8 err);

	/*
	 * Call back function of RSSI event.
	 * @param rssi_report The actual RSSI value reported with the RSSI
	 *                    event
	 * @param act_most    The actual mono/stereo status: Mono (true);
	 *                    Stereo(false).
	 */
	int (*p_rssi_cb)(s16 rssi_report, s32 act_most);

	/*
	 * Call back function of Pilot event.
	 * @param pilot_found true, if pilot is identified; false, otherwise.
	 */
	int (*p_pilot_found_cb)(s32 pilot_found);

	/*
	 * Call back function of the tracing done
	 * @param copied_bytes	Number of copied bytes.
	 */
	int (*p_test_trace_done_cb)(u16 copied_bytes);

	/*
	 * Call back function of the antenna selftest done
	 * @param result  The antenna selftest result.
	 */
	int (*p_ant_selftest_done_cb)(enum fmtrx_ant_selftest_res result);

	/*
	 * Call back funtion of FMRadio RX RTX tuning failed.
	 * @param freq The channel frequency where the rtx rx tuning failed.
	 */
	int (*p_fmrx_ant_tuning_fail_cb)(u32 freq);
} __packed;

/*
 * @brief
 * Registers callback functions for RX mode.
 * @param  callbacks   the structure of callback functions
 */
int fmrx_register_callbacks(struct fmrx_callbacks *fmrx_callbacks);


#endif

