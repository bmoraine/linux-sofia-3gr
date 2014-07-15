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
 * @file aud_app_fmr_hld_api_rx.h
 *
 * Contains the declaration of FMR RX high level driver APIs.
 *
 **/

#ifndef AUD_APP_FMR_HLD_API_RX_H
#define AUD_APP_FMR_HLD_API_RX_H

/*
** ============================================================================
**
**				INCLUDE STATEMENTS
**
** ============================================================================
*/
#include "aud_app_fmr_hld_rx.h"


/*
** ============================================================================
**
**				Function Declarations
**
** ============================================================================
*/

/*
  @defgroup FMR_RX FMRadio Receiver API Definition
  @brief All the API functions for FMR receiver is provided here;
  some of them have the restriction for calling them, please be awre of this.
*/

/*
 *
 * @brief Turns on the FM radio and begins reception.
 *
 * Default settings are used for initial frequency, mute state, etc. unless
 * these have been explicitly set before powering on.
 *
 * @pre FMR macro powered up and clocks supplied (by Platform APIs)
 * @pre FMR in reset state and with reset line released (by Platform APIs)
 * @pre Host audio read process enabled (Teaklite or DAC)
 *
 * @impl Direct call.
 * @see fmrx_initialise_state()
 * @see fmrx_power_off()
 */
int fmrx_power_on(void);


/*
 *
 * @brief Turns off the FM radio.
 *
 * Audio output would be smoothly muted before the disabling the FM Radio.
 * It is the responsibility of the caller to subsequently turn off the
 * audio read process and power off the FMR subsystem / clocks as needed.
 *
 * @impl Message based.
 * @see fmrx_power_on()
 */
int fmrx_power_off(void);


/*
 *
 * @brief Sets the receiver to a particular frequency.
 *
 * @param freq	 The frequency in hertz. If this is outside the defined FM
 *	 band then the current frequency would not be changed.
 *
 * @param force_mono Forces mono mode (same as calling fmrx_set_force_mono).
 *
 * @impl Message based.
 * @see fmrx_set_force_mono()
 */
int fmrx_station_tuning(u32 freq);

/*
 *
 * @brief Starts a search for stations with at least the given RSSI level.
 *
 * The search is terminated on wrapping around the band edge back to the start
 * frequency, reaching the upper/lower band limits or detecting the first valid
 * station. Search could be terminated by calling the fmrx_station_tuning()or
 * fmrx_power_off().
 *
 * @param mode	Seeking mode direction.
 * @param rssi	The minimum RSSI value required for being acepted as a valid
 * annel.
 *
 * @param pn	The maximum phase noise value required for being acepted
 * as a valid channel.
 *
 * @param force_mono Forces mono mode (same as calling fmrx_set_force_mono)
 * when entering the normal listening state.
 *
 * @impl Message based.
 */
int fmrx_station_seeking(struct fmrx_seek_station_params *seek_station_info);

/*
 *
 * @brief Performs a RSSI scanning of a defined block of frequencies and
 * indicated antenna types.
 *
 * Recording frequencies with the measured RSSI.
 * The search stops either when the fm block given has been covered
 * or when the report array is full. Unused entries in the report array have
 * RSSI values set to -10000.
 *
 * \note
 * - Even the function call is returned, the report list doesn't contain
 *   the valid data until the callback function is issued.
 * - When the scanning is finished, the antenna selection shall be returned to
 *   its original state.
 *
 * @param ant_typ   Antenna type indication.
 * @param space     Step size for evaluation.
 * @param min  Minimum operating frequency.
 * @param max  Maximum operating frequency.
 * @param report    Pointer to array of struct fmtrx_rssi_report structs to
 * record results.
 * @param maxcnt    Maximum number of elements in report array.
 *
 * @impl Message based.
 *
 * @see fmrx_set_rssi_offset()
 * @see fmrx_switch_antenna()
 * @see struct fmrx_callbacks
 *
 */
int fmrx_channel_auto_eval(struct fmrx_ae_params *fmrx_auto_eval_info);

/*
 * @brief Stop the ongoing auto evaluation
 *
 * Cancel any ongoing auto evaluation and provide the callback with scan
 * stopped notification.
 *
 * @impl Message based.
 */
int fmrx_channel_auto_eval_stop(void);

/*
 * @brief Get the current status of auto evaluation
 *
 * Fetch the channel scanned count and the total channels to scan count while
 * RSSI band sweep is ongoing.
 *
 * @param channel_count   Pointer to scanned channel count
 * @param total_channels  Pointer to total number of channels to scan
 *
 * @impl Message based.
 */
int fmrx_channel_auto_eval_progress(u16 *channel_count,
	u16 *total_channels);

/*
 * @brief Get the capablities of auto evaluation
 *
 * Fetch the band capabilities for RSSI scan like step size, max/min frequency.
 *
 * @param bandinfo		Pointer to RSSI sweep band information
 *
 * @impl Message based.
 */
int fmrx_channel_auto_eval_get_cap(struct fmtrx_band *band_info);

/*
 *
 * @brief Searches through the entire band from the min frequency until
 * max frequency.
 *
 * Recording frequencies where stations passing the search criteria.
 * The search stops either when the whole band has been covered
 * or when the report array is full. Updates the frequency state. Unused
 * entries in the report array have RSSI values set to -10000.
 *
 *
 * @param rssi		Minimum RSSI value to be accepted as a valid channel.
 * @param pn		Maximum Phase Noise value to go futher evalating.
 * @param report  The pointer to array of reports.
 *
 * @see fmrx_station_seeking()
 *
 */
int fmrx_start_auto_seeking(s16 rssi, struct fmtrx_as_report *report);

/*
*
* @brief Quickly checks the effective RSSI on a given list of channels, with as
* little disruption as possible to reception on the current channel.
* Returns back to the frequency which is used before the AF RSSI measurement.
*
* @param list  List of report elements giving frequencies to check. Measured
*	 RSSI values are written into this array. Entries with frequencies
*	 outside the allowed band are ignored. If the last entry in the list
*	 is the current channel then a fresh RSSI evaluation (including
*	 image channel strengths) is performed and the optimum RSSI is returned.
* @param cnt  Number of elements in list
*
* @impl Message based.
*/
int fmrx_af_measure_rssi(struct fmtrx_rssi_report *list, u32 cnt);

/*
 *
 * @brief Evaluates the given AF frequency. First, RSSI measurement is obtained
 * and validated against the given RSSI threshold. Second, PI code from RDS is
 * fetched and validated with the given PI code. If both conditions are passed,
 * then driver switches to the AF channel else, driver stays on the same
 * channel.
 *
 * @param af_freq     AF frequency to be evaluated
 * @param rssi_thres  Minimum RSSI threshold for AF
 * @param pi_code     Station PI code to be validated
 *
 * @impl Message based.
 */
int fmrx_af_evaluate(u32 af_freq, u16 pi_code, s16 rssi_thres);


/*
 * @brief For switching between internal antenna and ext. antenna (headset)
 *
 * @param antenna The antenna type selection between the internal and external
 * ones.
 *
 * @impl Message based.
 */
int fmrx_switch_antenna(enum fmtrx_ant_type fmrx_ant_type);


/*
 *
 * @brief Sets the routing for the audio outputs (Teaklite or DAC).
 *
 * If the FM radio is currently running, the audio will be muted and the
 * receive process stopped after which the routing is changed and the receive
 * process restarted.
 *
 * @param routing  Audio routing [FMR_AUD_PATH_DAC, FMR_AUD_PATH_DSP]
 *
 * @impl Message based.
 * @see fmrx_initialise_state()
 */
int fmrx_set_route(enum fmrx_aud_route routing);

/*
 *
 * @brief Sets the allowed operating band as well as the seeking stepsize for
 * the FM radio.
 *
 * @pre This should not be called during seeking; it is typically called at
 * initialization.
 *
 * Typical FM band setting for diffrent region,
 * - Europe / US: 87.5 ~ 108 MHz;
 * - Japan: 76 ~ 108 MHz (wide band) / 76 ~ 90 MHz (narrow band).
 *
 * @param min_freq Minimum operating frequency, the lowest value could be
 * 76 MHz.
 *
 * @param max_freq Maximum operating frequency, the highest vlaue could be
 * 108 MHz.
 *
 * @param freq_step  Frequency grid used for seeking, the reasonable values
 * would be 50 / 100 / 200 KHz.
 *
 * @impl Direct call.
 * @see fmrx_initialise_state()
 */
int fmrx_set_band(struct fmtrx_band *fmrx_band);


/*
 *
 * @brief Sets the audio output volume.
 *   By selecting the channel each time when setting the audio volume,which can
 *   control the audio friendly and dynamically for different channel.
 *
 * @param channel		the channel type.
 * @param volume   0 : audio off (-44 dB); >= 112, full volume (12 dB).
 * Volume range is 56 dB, and step size is 0.5 dB for each.
 *
 * @impl Direct call.
 * @see fmrx_initialise_state()
 */
int fmrx_set_volume_level(enum fmrx_aud_channel aud_channel,
	u8 aud_ch_vol);


/*
 *
 * @brief Temporarily sets the output volume to zero, and then disables the
 * FM audio processing.
 *
 * By setting the volume to 0 first, there is a optimized ramping function to
 * give a quick response in mini DSP FW to prevent from pop / click in audio
 * caused by immediate change in signal level. However, if customers need a
 * fading response, then this would be implemented in glue layer by calling
 * volume setting function subsequently.
 *
 * If the audio is already muted, has no effect.
 * When unmuted, the volume is returned to that given in the last call to
 * fmrx_set_volume().
 *
 * @param mute true = audio muted, false = audio unmuted
 * @impl Message based.
 * @see fmrx_initialise_state()
 */
int fmrx_set_mute(enum fmrx_aud_channel aud_channel, s32 mute);

/*
 *
 * @brief Forces mono mode.
 *
 * Otherwise, stereo is enabled if a stereo signal
 * is identified (stereo noise cancellation may reduce the stereo effect
 * for weak signals).
 *
 * @param mono true = mono mode forced, false = mono mode not forced
 * @impl Message based.
 *
 * @see fmrx_set_snc()
 * @see fmrx_initialise_state()
 */
int fmrx_set_force_mono(s32 mono);


/*
 *
 * @brief Stereo Noise Cancellation,
 * configures signal level dependent blending from stereo to mono.
 *
 */
int fmrx_set_snc(struct fmrx_snc *snc_config);

/*
 *
 * @brief Configures the signal level dependent reduction of audio output
 * volume.
 *
 */
int fmrx_set_soft_mute_cfg(struct fmrx_sm *sm_cfg);

/*
 *
 * @brief Configures the AGC in FW.
 *
 * @param agc_gain_cfg
 * - en true  = Enable AGC algorithm
 *	false = Disable AGC algorithm
 * - gain_index If AGC is disabled, gain index should be provided
 *
 * @note
 * AGC is enabled:
 *   - Gain is selected automatically based on the input deviation by FW.
 *
 * @note
 * AGC is disabled:
 * - No automatic gain is applied. The gain index value from 0-15 set by user
 * is selected and appropriate gain is applied.
 *
 * @impl Direct call.
 *
 */
int fmrx_set_agc_gain_cfg(struct fmrx_agc *agc_gain_cfg);

/*
 *
 * @brief Sets offset value to internal RSSI.
 *
 * @pre Should only be called at initialization or when the FM radio is powered
 * off.
 *
 * @param  offset_hs Per the headset antenna selected, offset in dB added to
 * internally calculated value
 *
 *@param  offset_embed Per the embeded antenna selected, offset in dB added to
 * internally calculated value
 *
 * @impl Direct call.
 * @see fmrx_initialise_state()
 * @see fmrx_get_rssi_offset()
 */
int fmrx_set_rssi_offset(s16 offset_hs, s16 offset_embed);

/*
 *
 * @brief Sets the deemphasis mode of reception.
 *
 * @param deem_mode Mode selection of 50 us / 75 us.
 *
 * @pre Should only be called at initialization or when the FM radio is powered
 * off.
 *
 * @see fmrx_initialise_state()
 *
 */
int fmrx_set_deemphasis_cfg(enum fmtrx_emph_sel deem_mode);

/*
 * \brief Configure the offset tables of specific antenna type for compensating
 * the affection from External LNA.
 *
 * The new pointers of tables would be accepted only when "OFF" and "Normal
 * Receiving mode; and under "Normal Receiving" mode, the setting would take
 * effect immediately.
 *
 * @param lna_cfg takes the LNA compensation table for specific antenna type
 * @param ant_type takes the antenna type to be updated
 * @see fmrx_set_rssi_offset()
 * @see fmrx_initialise_state()
 * @see struct fmrx_ext_lna_cfg
 * @see enum fmtrx_ant_type
 */
int fmrx_set_ext_lna_rssi_comp(
	struct fmrx_ext_lna_cfg lna_cfg,
	enum fmtrx_ant_type ant_type);

/*
 * \brief Get the configuration of the offset tables of specific antenna type
 *  for compensating the affection from External LNA.
 *
 * The new pointers of tables would be accepted only when "OFF" and "Normal
 * Receiving" mode; and under "Normal Receiving" mode, the setting would take
 * effect immediately.
 *
 * @param lna_cfg takes the LNA compensation table for specific antenna type
 * @param ant_type takes the antenna type to be updated
 * @see fmrx_set_rssi_offset()
 * @see fmrx_initialise_state()
 * @see struct fmrx_ext_lna_cfg
 * @see enum fmtrx_ant_type
 */
int fmrx_get_ext_lna_rssi_comp(
	struct fmrx_ext_lna_cfg *lna_cfg, enum fmtrx_ant_type ant_type);

/*
 *
 * @brief Over-writes the default values in FmrxInitializeState() with the
 * platform specified configuration.
 *
 * @param static_config Contains the configuration which are supposed to keep
 * static during the FMR running.
 *
 * @see FmrxInitializeState()
 *
 */
int fmrx_set_cfg(struct fmrx_cfg *cfg);

/*
 *
 * @brief Returns the static configurations of FM Rx
 *
 * @param static_config_data Contains the configuration which are supposed to
 * keep static during the FMR running.
 *
 * @impl Direct call.
 *
 */
int fmrx_get_cfg(struct fmrx_cfg **cfg);

/*
 *
 * @brief Sets RDS configuration: RDS mode (RDS/RBDS) and minimum number of
 * free space before generating an interrupt to the host;
 *
 * Default is RDS. Only has effect when powering on the FM radio.
 *
 * @pre Should only be called at initialization or when the FM radio is powered
 * off.
 * @param rds_cfgs The RDS configurations
 *
 *
 * @impl Direct call.
 */
int fmrx_set_rds_cfg(struct fmrx_rds_cfg *rds_cfgs);

/*
 * @brief Returns the packaged channel information.
 *
 * @return Report the working status of current Channel.
 *
 */
int fmrx_get_channel_info(struct fmrx_ch_info *channel_info);

/*
 * @brief
 * Function to get the FM Rx Data state varaible. Used primarily for testing /
 * debugging purpose.
 *
 * @param *fmrx_data_state Pointer to the location where the FM Rx data state
 * to be stored.
 *
 * @return  The result of the operation is returned in int
 */

int fmrx_get_rx_data_state(
	struct fmrx_cfg *fmrx_data_state);

/*
 *
 * @brief Get the audio output volume.
 *   By selecting the channel each time when getting the audio volume,which can
 *   get the audio volume value for different channel.
 *
 * @param channel		the channel type.
 *
 * @return the audio volume value, the result can be the left channel volume,
 * the right channel volume, or both(high 8bit is the left channel volume,
 * low 8bit is the right channel volume).
 *
 * @impl Direct call.
 */
int fmrx_get_volume(enum fmrx_aud_channel aud_channel,
	u16 *channel_volume);

int fmrx_get_band_info(struct fmtrx_band *fmrx_band);


/*
 * @brief Gets the current Received Signal Strength Indication (RSSI).
 * @return RSSI value unit in dBm / dBuV.
 */
int fmrx_get_rssi_level(s16 *rssi_level);


/*
 *
 * @brief Returns the current RSSI offset setting.
 *
 * @return unit in dBuV/dBm
 * @impl Direct call.
 * @see fmrx_set_rssi_offset()
 *
 */
int fmrx_get_rssi_offset(s16 *offset_hs);


/*
 * \brief Returns the Revision information for FMRadio receiver.
 * \return rev Revision structure which includes,
 * - Hardware ID
 * - FW ID
 * - FW Timestamp
 * - HLD ID
 * - LLD ID
 * - Overall Software Package ID
 */
int fmrx_get_revisions(struct fmtrx_revision *fmtrx_rev);

/*
 *
 * @brief Sub/UnSubscribes to RSSI reporting.
 *
 * An RSSI event will be generated if the
 * effective RSSI goes from within the range of the lower and upper bounds
 * to outside of the range, or if a stereo signal is detected / lost.
 *
 * If lower is set greater than upper, all changes in RSSI are reported.
 *
 * @param lower  Lower RSSI value; RSSI event reported if the RSSI drops
 *	 from above to below this value
 * @param upper  Upper RSSI value; RSSI event reported if the RSSI drops
 *	 from below to above this value
 *
 * @impl Message based.
 * @see fmrx_unsubscribe_rssi()
 */
int fmrx_set_rssi(struct fmrx_rssi_subsc *rssi_config);

/*
 *
 * @brief Subscribes to RSSI reporting.
 *
 * An RSSI event will be generated if the
 * effective RSSI goes from within the range of the lower and upper bounds
 * to outside of the range, or if a stereo signal is detected / lost.
 *
 * If lower is set greater than upper, all changes in RSSI are reported.
 *
 * @param lower  Lower RSSI value; RSSI event reported if the RSSI drops
 *	 from above to below this value
 * @param upper  Upper RSSI value; RSSI event reported if the RSSI drops
 *	 from below to above this value
 *
 * @impl Message based.
 * @see fmrx_unsubscribe_rssi()
 */
int fmrx_subscribe_rssi(s16 lower_thres,
	s16 upper_thres);

/*
 *
 * @brief Cancels subscription to RSSI reporting, no RSSI events will be
 * generated
 *
 * @impl Message based.
 *
 * @see fmrx_subscribe_rssi()
 * @see fmrx_set_rssi_offset()
 *
 */
int fmrx_unsubscribe_rssi(void);


/*
 *
 * @brief Subscribes to RDS events.
 *
 * Events will be generated in different circumstances
 * depending on the parameters to this function call.
 *
 * @param mode RDS_MODE_POLL/RDS_MODE_IRQ
 *
 * @param fastPI   When true, a callback with type FMR_RDS_PI will be
 *		 generated whenever a valid PI is received
 *		 (possibly before obtaining full RDS sync).
 *
 * @param min_free Number of free spaces in FMR-internal group buffer
 *		 below which a callback with type FMR_RDS_CNT occurs.
 *		 If set to 0, buffering is disabled and groups are
 *		 passed as soon as they are received using callback
 *		 type  struct fmrx_rds_group(legacy behaviour).
 *			 The maximum valid value is the (buffer_size-2),
 *			 for value (buffer_size-1) & above, the irq will
 *			 be generated only when the buffer is full.
 *
 * @impl Message based.
 * @see fmrx_unsubscribe_rds()
 */
int fmrx_subscribe_rds(struct fmrx_rds_subscribe_params *rds_params);


/*
 *
 * @brief Cancels subscription to RDS events and disables RDS processing to
 * save power
 *
 * @impl Message based.
 * @see fmrx_subscribe_rds()
 */
int fmrx_unsubscribe_rds(void);

/*
 *
 * @brief Subscribes to RDS SYNC events.
 *
 * Events will be generated per RDS synchronization lost / found.
 *
 * @impl Message based.
 * @see fmrx_unsubscribe_rds_sync()
 */
int fmrx_subscibe_rds_sync(void);


/*
 *
 * @brief Cancels subscription to RDS SYNC events
 *
 * @impl Message based.
 * @see fmrx_subscibe_rds_sync()
 */
int fmrx_unsubscribe_rds_sync(void);

/*
 *
 * @brief Transfers RDS groups stored in the FMR-internal group buffer.
 *
 * Only usable when fmrx_subscribe_rds() has been called with parameter
 * min_free greater than 0; if called in other situations has no effect
 * (returns zero)
 *
 * The ring-buffer could support maximum 28 groups (4 blocks for each)
 * corresponding to 112 blocks; however, we don't store block A,
 * the PI code is store in the dedicated FW register (in order to save the
 * memory usage);
 * In LLD/HLD, by polling the PI register, the groups structure would be
 * recovered.
 *
 * @param group_cnt  Number of groups requested to be transferred
 *
 * @param group_buff Pointer to a storage area to which RDS groups will be
 * transferred
 *
 * @return The number of groups actually transferred, less than or equal to
 * group_cnt
 *
 * @impl Direct call.
 */
int fmrx_transfer_rds_groups(u16 group_cnt,
	void *rds_buf_desc, u16 *copied_groups);


/*
 * @brief Sets IDI handshake bit to 0 or 1. If 1, IDI will start
 * transfer of samples from FMR IP to IDI ABB TX. If 0, IDI will stop transfer
 * of samples.
 *
 * @param enable  If true, enable IDI handshake bit, else, disable it.
 *
 */
int fmrx_set_idi_handshake(s32 enable);

/*
  @ingroup FMR_RX
  @defgroup FMR_RX_UTILITIES_TEST Testing Utilities
  @brief This section describes all the functions for test mode only!
*/
/* @{ */
/*
 *
 * @brief Forces the High/Low/Auto side inject during the channel evalutation.
 *
 * @param sb FMR_FORCE_NONE = auto determined; FMR_FORCE_HSI = forces to high
 * side injection; FMR_FORCE_LSI = forces to low side injection.
 *
 * @pre Should only be called at initialization or when the FM radio is powered
 * off.
 * @pre For test purpose only.
 *
 * @see fmrx_initialise_state()
 *
 */
int fmrx_set_sideband_injection(enum fmrx_sb_inj_side sideband,
	enum fmtrx_inj_sel sb_sel);

/*
 *
 * \brief Dumps the FW regiters contents for system debugging.
 * \param fw_reg structure for interested FW register while debugging.
 *
 * @pre For test purpose only.
 */
int fmrx_test_dump_fw_reg(struct fmrx_fw_reg *fw_reg);


/*
 * @brief Tracing samples from mini DSP data path. This is only be used for
 * testing & debugging only.
 * @param trace_sel  Tracing position selection, disable the feature when "0".
 * @param trace_data Pointer to the tracing data buffer.
 * @param target_num The number of 32-bits word would be dumpped, the tracing
 * done callback would be called when the specified number is reached, and rest
 * trace_sel to 0 for disable.
 */
int fmrx_test_trace(enum fmrx_test_trace_sel trace_sel,
	u32 *trace_data, u32 target_num);

/* @} */ /* FMR_RX_UTILITIES_TEST */

#endif
