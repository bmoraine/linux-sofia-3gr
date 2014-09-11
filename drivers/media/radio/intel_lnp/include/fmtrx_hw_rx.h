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
**							INTERFACE DESCRIPTION
**
** =============================================================================
*/
/**
 * @file fmtrx_hw_rx.h
 *
 * This file contains interfaces that are HW related. These interfaces form the
 * abstraction layer for the hardware.
 **/

#ifndef _FM_TRX_HW_H_
#define _FM_TRX_HW_H_

/*
** =============================================================================
**
**							INCLUDE STATEMENTS
**
** =============================================================================
*/

/*
** =============================================================================
**
**							DEFINES
**
** =============================================================================
*/

/*
** =============================================================================
**
**					EXPORTED ENUM DEFINITIONS
**
** =============================================================================
*/

/*
** =============================================================================
**
**					EXPORTED STRUCT DEFINITIONS
**
** =============================================================================
*/

/*
** =============================================================================
**
**					EXPORTED FUNCTION DECLARATIONS
**
** =============================================================================
*/
/* Power on the RF macro
 * @antenna_type Select which antenna type to configure during power on
 * @lna_out_gain Select the LNA out gain value
 */
int fmtrx_hw_rf_poweron(
		enum antenna_type type,
		enum lna_out_gain gain);

/* Power off the RF macro
 */
int fmtrx_hw_rf_poweroff(
		void);

/* Start the minidsp to execute FW
 */
int fmtrx_hw_start_minidsp(
		void);

/* Check status of minidsp
 * @status minidsp is running or halted
 */
int fmtrx_hw_is_minidsp_running(
		bool *status);

/* Stop the minidsp to execute FW
 */
int fmtrx_hw_stop_minidsp(
		void);

/* Set Gain offsets for different blocks in FMR IP
 * @gain_offset_type Type of block (LNA, PPF, RSSI, CPINIT)
 * @offs Offset array
 * @size Size of the offset array
 */
int fmtrx_hw_set_gain_offsets(
		enum gain_offset_type type,
		u8  *offs,
		u32 size);

/* Set the RSSI other offset
 * @rssi_other_offset RSSI other offset
 */
int fmrx_hw_set_rssi_other_offset(
		s16 rssi_other_offset);

/* Set band/region information
 * @lo_band Start frequency (in khz) of the band
 * @hi_band End frequency (in khz) of the band
 */
int fmtrx_hw_set_band(
		u32 lo_band,
		u32 hi_band);

/* Set audio configuration like de-emp
 * @deemphasis_type De-emphasis is selected based on the region
 */
int fmrx_hw_set_audio_deemp(
		enum deemphasis_type type);

/* Set audio configuration like volume ramping
 * @volume_ramp Volume ramp down value that will be used during mute
 */
int fmrx_hw_set_audio_volumeramp(
		s16 volume_ramp);

/* Set audio configuration like force mono
 * @force_mono Select to force the channel reception in mono mode
 */
int fmrx_hw_set_audio_forcemono(
		bool force_mono);

/* Set audio mute
 * @deemphasis_type De-emphasis is selected based on the region
 * @mute_enable Enable/disable mute
 * @mute_wait If set, the mute function will block until the volume ramp down completes, when muted
 */
int fmrx_hw_set_mute(
		bool mute_enable,
		bool mute_wait);

/* Set audio volume
 * @left Left channel volume
 * @right Right channel volume
 */
int fmrx_hw_set_volume(
		u8 left,
		u8 right);

/* Set audio routing path
 * @routing_mode Direct DAC mode or SRC mode
 */
int fmrx_hw_set_routing(
		enum routing_mode mode);

/* Set stereo noise cancellation parameters
 * @cfg Pointer to the SNC configuration structure
 */
int fmrx_hw_set_snc(
		struct snc *cfg);

/* Set soft mute parameters
 * @cfg Pointer to the SM configuration structure
 */
int fmrx_hw_set_sm(
		struct sm *cfg);

/* Set Automatic Gain Control parameters
 * @cfg Pointer to the AGC configuration structure
 */
int fmrx_hw_set_agc(
		struct agc *cfg);

/* Set the RSSI thresholds for RSSI change notifications
 * @lo_thr RSSI lower thresholds below which an interrupt is generated
 * @hi_thr RSSI higher thresholds below which an interrupt is generated
 */
int fmrx_hw_set_rssi_notification(
		struct rssi_notify *cfg);

/* Set RDS parameters
 * @good_blks Number of good blocks after sync
 * @bad_blks Number of bad blocks before sync loss
 * @bad_blks_search Number of bad blocks during sync search
 */
int fmrx_hw_set_rds_cfg(
		u16 good_blks,
		u16 bad_blks,
		u16 bad_blks_search);

/* Set RDS power on mode
 * @rds_onmode RDS power modes can be ON, OFF & RETAIN
 */
int fmrx_hw_set_rds_onmode(
		enum rds_onmode mode);

/* Set RDS PI mode
 * @rds_pimode Enable RDS pi mode or normal mode
 */
int fmrx_hw_set_rds_pimode(
		bool rds_pimode);

/* Set IDI handshake control bit
 * @fmtrx_type FM RX or TX
 * @hs_enable Enable/disable the HS bit
 */
int fmtrx_hw_set_idi_hs(
		enum fmtrx_type type,
		bool hs_enable);

/* Select the clock source for FMR IP digital
 * @clk_source Select mainclk or internal RF
 */
int fmrx_hw_set_clk_source(
		enum clk_source src);

/* Clear the interrupt by setting the Interrupt clear HW register
 * @interrupt_clr Interrupt clear value
 */
int fmtrx_hw_set_interrupt_clear(
		u32 interrupt_clr);

/* Tune to a station
 * @frequency Wanted channel in khz
 * @injection_side Selection of injection side (High, Low and Auto)
 * @rssi_thr RSSI threshold, below which channel tuning fails
 * @clk_switch_range_104 Range in which clock switch to internal RF should happen
 */
int fmrx_hw_channel_tune(
		u32 frequency,
		enum injection_side side,
		s16 rssi_thr,
		u32 clk_switch_range_104);

/* Search next/previous channel
 * @start_frequency Frequency from which FW starts its search in khz
 * @stop_frequency Frequency from which FW ends its search in khz
 * @step Frequency steps in khz
 * @injection_side Selection of injection side (High, Low and Auto)
 * @rssi_thr RSSI threshold, below which channels are rejected
 * @pn_thr Phase noise, above which channels are rejected
 * @found_frequency Valid channel found by FW
 */
int fmrx_hw_channel_search(
		u32 start_frequency,
		u32 stop_frequency,
		s16 step,
		enum injection_side side,
		s16 rssi_thr,
		u16 pn_thr);

/* Reset RDS block
 */
int fmrx_hw_rds_reset(
		void);

/* Get channel information from FMR IP
 * @data Pointer to the fetched channel information
 */
int fmrx_hw_get_channel_info(
		struct channel_info *data);

/* Get channel frequency from FMR IP
 * @tuned_frequency Pointer to the tuned channel frequency
 */
int fmrx_hw_get_channel_freq(
		u32 *data);

/* Get channel signal strength from FMR IP
 * @data Pointer to the RSSI value
 */
int fmrx_hw_get_channel_rssi(
		s16 *data);

/* Get interrupt status from HW register
 * @interrupt_status Pointer to interrupt status value
 */
int fmtrx_hw_get_interrupt_status(
		u32 *interrupt_status);

/* Get the chip/lld version details
 * @data Pointer to chip version information.
 */
int fmrx_hw_get_id(
		struct fmr_id *data);

/* Get RDS groups
 * @groups_to_read Number of groups to be fetched from the FMR IP memory
 * @rds_groups Pointer to the RDS groups buffer
 * @restore_pi_blocks If true, type A block will be restored
 * @groups_read Number of actual groups returned
 */
int fmrx_hw_get_rds_groups(
		u16 groups_to_read,
		struct rds_group *rds_groups,
		bool restore_pi_blocks,
		u16 *groups_read);

/* Get PI code
 * @data Pointer to the PI code
 */
int fmrx_hw_get_rds_pi(
		s16 *data);

/* Get FW state
 * @state Pointer to the state of the FW
 */
int fmrx_hw_get_fw_state(
		enum fmrx_state *state);

/* Set FW state
 * @state New state
 */
int fmrx_hw_set_fw_state(
		enum fmrx_state state);

#endif  /* _FM_TRX_HW_H_  */

