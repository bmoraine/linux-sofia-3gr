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
**                              MODULE DESCRIPTION
**
** ============================================================================
*/
/*
 * @file aud_app_fmr_hld_rx_rf.h
 *
 * This file defines the detailed sequence of FM Rx operations necessary to
 * perform the required use cases (power on, power off, seek, ...)
 *
 */

#if !defined(AUD_APP_FMR_HLD_RX_RF_H)
#define AUD_APP_FMR_HLD_RX_RF_H

/*
** ============================================================================
**
**                              INCLUDE STATEMENTS
**
** ============================================================================
*/
#include <aud_app_fmr_lld_api_rx.h>
#include <aud_app_fmr_hld_rx.h>

/*
** ============================================================================
**
**                                   DEFINES
**
** ============================================================================
*/


#define FMR_FREF		26000000 /* Reference frequency */
#define FMR_OTHER_OFFSET_DEFAULT -24
#define AUD_MAX_VOL		112	/* Max audio level */
#define BABS_MAX_LOOP		3	/* Max BABS loop in one-shot */
#define BABS_WAIT_US		300	/* Time to wait after BABS for ct/cf */
#define RC_ALG_CNT		10	/* RC alignment running count */
#define BABS_CNT		10	/* Babs running count */
#define CLK_SWT_MIN(x)		(104000000 - (x))
#define CLK_SWT_MAX(x)		(104000000 + (x))
#define RSSI_TO_EXT(x)		(x / 4)	/* Conversion to dBuV */
#define RSSI_TO_INT(x)		(x * 4)	/* Conversion to 1/4 of dBuV */

/* The fast mode of PhaseNoise estimate */
#if !defined(PN_EST_FAST)
#define PN_EST_FAST 512
#endif

/* The slow mode of PhaseNoise estimate */
#if !defined(PN_EST_SLOW)
#define PN_EST_SLOW (PN_EST_FAST*16)
#endif

/* As the RSSI interrupt subscribeing in "Verbose" mode, the interrput keeps
 * being generated whenever the signal cross 2 verbose steps */
#if !defined(RSSI_VERBOSE_STEP)
#define RSSI_VERBOSE_STEP  12 /* 3 dB as default step size */
#endif

/* Number of blocks to measure oneshot during evaluation over */
#define EVAL_NBLOCKS 1

/* Time in microseconds to allow for settling on new frequency */
/* for one-shot RSSI change it to 160 us */
#define TIM_SETTLE_US 160

/* Time in microseconds to allow for settling on image swap */
/* for one-shot RSSI change it to 160 us */
#define TIM_SWAP_US 160

/* Amount by which image channel is reduced in wanted band */
#define IMG_SUPPRESS  (50*4)

/* Deadband where RSSI values are considered equivalent */
#define RSSI_DEADBAND 4

/* RC alignment time constant */
/* Based on 13 MHz BUS clock */
#define RC_ALIGN_RUNTIME (40)

/* Value by which the RSSI threshold shall be increased per step of the LNA.
 * Unit is [dBuV*4] */
#define RSSI_THR_NF_LNA  ((36 - 22) * 4 / 16)

/* Value by which the RSSI threshold shall be increased per step of the PPF.
 * Unit is [dBuV*4.] */
#define RSSI_THR_NF_PPF  4

/* Value by which the RSSI threshold shall be increased per step of the
 * PPFCTRL_2ND. Unit is [dBuV*4] */
#define RSSI_THR_NF_PPF2ND  (1)
#define HZ_TO_KHZ(freq)	((freq) / 1000)
#define KHZ_TO_HZ(freq)	((freq) * 1000)


/*
** ============================================================================
**
**                              Function Declaration
**
** ============================================================================
*/

/* Move firmware from idle to receive state */
void fmr_idle2receive(void);

/* Move firmware from receive to idle state */
void fmr_receive2idle(void);

/*
 *
 * Prepares the FM radio for reception. After this state the radio will be
 * receiving.
 *
 * Duration ~2ms
 *
 * @pre FMR macro powered up and clocks supplied
 * @pre FMR in reset state and with reset line released
 * @pre Host audio read process enabled (Teaklite or DAC)
 *
 * @post FM radio running, waiting to be tuned to a channel
 *
 * @param state    HLD state used to configure firmware at startup
 *
 */
int fmr_start_receiver(struct fmrx_state *state);

/*
 *
 * Mutes the audio, then turns off the FM radio. After calling this function
 * the power and clocks can be turned off and the reset applied.
 *
 * Takes ~10ms for the audio to be fully muted
 *
 * @post FMR audio muted, RF part powered off and minidsp halted.
 *
 */
void fmr_stop_receiver(struct fmrx_state *state);

/*
 *
 * Performs necessary steps before leaving the RECEIVING state:
 *
 * @li  mutes the audio
 * @li  disables RSSI, pilot, RDS and frequency offset event generation
 *
 */
void fmr_receiving_exit_actions(struct fmrx_state *state);

/*
 *
 * Performs necessary steps on beginning reception at a new frequency
 *
 * Prior to leaving the RECEIVING state all events should be disabled. On
 * re-entering the RECEIVING state, this function does the following
 *
 * @li resets the RSSI measurement (and in turn the soft mute / SNC state)
 * @li resets the pilot PLL
 * @li resets the "pilot_found" state variable
 * @li resets the RDS state
 * @li restores the audio volume (if not muted)
 * @li re-enables events
 *
 * As soon as a valid RSSI measurement has been done by the minidsp firmware,
 * events will be enabled.
 *
 */
void fmr_receiving_entry_actions(struct fmrx_state *state);

/*
*
* Utility function to mute the audio, returns only when the output volume
* has reached zero.
*
* @pre  Audio either not muted, or muted with output volume set to zero
*
*/
void fmr_mute_wait(void);

/*
*
* Performs the mapping from host volume to FMR-internal volume
*
* @param host_volume  A volume level from 0..63
*
* @return The volume mapped to a logarithmic scale
*
*/
u16 fmr_host_to_fmr_volume(u8 host_volume);

/*
* \brief Look up the frequency based RSSI offset table to get the correct
* LNA offset for RSSI calculation.
*
* \param state STATE which containing the pointer to RxState.
*
* \return external lna offset
*/

s16 fmr_peek_ext_lna_table(struct fmrx_state *state);

/*
* Evaluates the RSSI on a particular frequency against a given minimum
* criterion.
*
* If a signal appears to be present that meets the RSSI requirement, checks the
* image channels and adjacent channels to ensure that the measured signal is
* not a spurious reception. Returns the "effective" RSSI value measured for
* that channel, i.e. taking into account interference from the image channel
* and leaves the receiver tuned in the optimum configuration (upper / lower
* sideband injection)
*
* Measurements will stop as soon as a measurement indicates that RSSI is less
* than rssi_thr.
*
* The measured image channel RSSI is returned to allow estimates of effective
* RSSI to be performed based on later RSSI measurements.
*
* Minimum measurement duration: ~200us (no signal found)
*
* Maximum measurement duration: ~1400us (both sidebands evaluated)
*
* @note Audio should be muted before calling this function, normal RSSI
* callbacks should also be disabled since some spurious values will appear when
* changing channels.
*
* @param frequency The wanted channel frequency to measure on
*
* @param rssi_thr_base The minimum acceptable RSSI level
*
* @param state pointer to the internal State registering
*
* @return The optimum measured RSSI level
**/
u16 fmr_evaluate_channel(struct fmrx_state *state, u32 freq, s16 rssi_thres);

/*
*
* Calculates an effective RSSI based on measured RSSI and image RSSI
*
* Effective RSSI calculation:
*
* At maximum gain, with 0dBuV input signal the resulting SNR
* at the demodulator input is about 8dB.
*
* The image channel is mirrored into the wanted channel with
* a certain reduction in signal strength. The SNR of the wanted
* channel is therefore the measured RSSI of the wanted channel
* minus the RSSI of the image channel minus the amount of image
* suppression. This can be converted into an "effective" RSSI
* by subtracting the 8dB SNR corresponding to 0dBuV.
*
* @param measured  The measured RSSI value on the wanted channel
* @param image    The measured RSSI value on the image channel
*
* @return  An estimate of the effective RSSI value
*
*/
s16 fmr_calc_effective_rssi(s16 measured, s16 image);

/*
*
* Calculates a target measured RSSI based on an effective RSSI and image RSSI
* (this is the reverse of fmr_calc_effective_rssi)
*
* @param effective  The effective RSSI that is targeted
* @param image    The measured RSSI value on the image channel
*
* @return  The measured RSSI value that should correspond to the effective RSSI
*
*/
s16 fmr_calc_rssi_target(s16 effective, s16 image);

/*
 *@brief Reactivate the RDS
 */
void fmr_reactive_rds(struct fmrx_state *state);

/*
*
* Calculate the noise figure of one-shot rssi measurement
*
* @param rssi_data  The RSSI data structure contain oneshot result
*
*/
s16 fmr_calc_rssi_nf_offset(const struct fmtrx_rssi_data *rssi_data);

/*
* Function:... fmr_clk_switch
* Parameters:
* Returns:.... ADC clock switching and CGU clock switching
* Description:
*/
void fmr_clk_switch(struct fmrx_state *state);

/*
*
* Sets up RSSI event thresholds
*
* Registers RSSI levels, converting to effective RSSI values
*
* @param state  The HLD state structure
*
*/
void fmr_setup_rssi_thresholds(struct fmrx_state *state);
void fmr_ant_switch(struct fmrx_state *state);
void fmtrx_enable_test_trace_int(s32 en);
void fmrx_set_agc_gain(u8 agc_enable, u16 gain_index);
void fmr_send_ch_search_cmd(struct fmrx_state *state,
	struct fmtrx_ch_search_cmd *ch_search_params);
enum fmrx_ch_search_status fmr_check_ch_search_status(
	struct fmrx_state *state);
void fmrx_get_ch_status(struct fmrx_state *state);
int fmr_set_gain_offsets(struct fmrx_state *state,
	enum gain_offset_type off_type, s16 *gain_offs, u32 size);
int fmr_set_rssi_gain_offsets(struct fmrx_state *state,
	struct rssi_offs *gain_offs);
int fmrx_set_fw_state(enum fmrx_sm_state state);

#endif  /* AUD_APP_FMR_HLD_RX_RF_H */

