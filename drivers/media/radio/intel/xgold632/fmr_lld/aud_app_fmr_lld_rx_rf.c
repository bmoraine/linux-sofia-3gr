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

/* MODULE DESCRIPTION */
/* This file contains the RX RF related functionalities */

/* INCLUDE STATEMENTS */
#include <linux/errno.h>

#include "aud_app_fmr_lld_api_rx.h"	/* LLD RX prototypes */
#include "aud_app_fmr_lld_basic_rw.h"	/* Basic read/write functionality */
#include "aud_app_fmr_hld_api.h"	/* HLD interfaces */
#include "aud_app_fmr_lld_rx_rf.h"	/* RX RF interfaces */
#include "aud_app_fmr_sys.h"		/* System interfaces */
#include <fmr_rx_api.h>			/* FW interfaces */

/* DEFINES */
#if !defined(DRV_FEAT_AUD_APP_FREQ_MGR_PRESENT)
/* For DCDC clock switching */
#if !defined(WITH_DCDC_TAB)
#define WITH_DCDC_TAB
#endif
#endif

/* GLOBAL DATA DEFINITIONS */

/* @brief
 * Volume mapping to linear FW volume values of RX.
 *
 *	There is 113 steps in total, and the step spacing is -0.5 dB.
 *	12dB equals 32767
 *
 *	Volume Level	Gain in dB		FW Value
 *
 *	112			12		32767
 *
 *	99			11.5		30934
 *
 *	..			..		..
 *
 *	88			0		8231
 *
 *	0			-50		0
 */

static u16 fmrx_volume_mapping[113] = {
	0, 55, 58, 62, 65, 69, 73, 78, 82, 87, 92, 98, 104, 110,
	116, 123, 130, 138, 146, 155, 164, 174, 184, 195, 207, 219,
	232, 246, 260, 276, 292, 309, 328, 347, 368, 389, 413, 437,
	463, 490, 519, 550, 583, 617, 654, 693, 734, 777, 823, 872,
	923, 978, 1036, 1098, 1163, 1232, 1304, 1382, 1464, 1550,
	1642, 1740, 1843, 1952, 2067, 2190, 2320, 2457, 2603, 2757,
	2920, 3093, 3277, 3471, 3677, 3894, 4125, 4370, 4628, 4903,
	5193, 5501, 5827, 6172, 6538, 6925, 7336, 7770, 8231, 8718,
	9235, 9782, 10362, 10976, 11626, 12315, 13045, 13818, 14636,
	15504, 16422, 17395, 18426, 19518, 20675, 21900, 23197, 24572,
	26028, 27570, 29204, 30934, 32767
};

/* LOCAL FUNCTION DEFINITIONS */
static s32 fmtrx_send_cmd(u16 cmd_id, u16 *cmd_param, u8 count,
	enum fmtrx_trigger_events trig_evt)
{
	s32 rc = 0;

	fmrx_send_cmd(cmd_id, cmd_param, count, true);
	rc = fmr_sys_wait_for_event(trig_evt);

	return rc;
}

static s16 fmr_calc_rssi_other_offs(struct fmrx_state *state)
{
	s16 rc = 0;
	s16 lna_gain = 0;

	/* external lna table rssi offset */
	rc = fmr_peek_ext_lna_table(state);

	/* offset for single ended antenna RX path*/
	if (state->cfg->antenna == FMR_ANT_HS_SE)
		rc = rc + state->cfg->other_cfg.rssi_off_ext;
	else /* offset for differential antenna RX path */
		rc = rc + state->cfg->other_cfg.rssi_off_int;

	/* count in the 2nd stage LNA out gain */
	lna_gain = (3 - state->cfg->other_cfg.lna_out_gain) * 3;
	rc += RSSI_TO_INT(lna_gain);

	/* constant offset */
	return rc + FMR_OTHER_OFFSET_DEFAULT;
}

static void fmtrx_rf_poweron(struct fmrx_state *state)
{
	struct fmtrx_rf_pow_on_cmd rf_pow_on_cmd_params;
	u16 ant_type = 0;

	/* Send RF power up cmd */
	switch (state->cfg->antenna) {
	case FMR_ANT_HS_SE:
		ant_type = ANTTYPE_HS_SINGLE;
		break;

	case FMR_ANT_EBD_DE:
		ant_type = ANTTYPE_EBD_DIFF;
		break;

	default:
		ant_type = ANTTYPE_HS_SINGLE;
		break;
	}

	rf_pow_on_cmd_params.ant_type = ant_type;
	rf_pow_on_cmd_params.lna_out_gain =
		(u16)state->cfg->other_cfg.lna_out_gain;
	rf_pow_on_cmd_params.reg_vdd = 0;
	rf_pow_on_cmd_params.reserved = 0;

	/* Send RF power up cmd */
	fmtrx_send_cmd(FMRX_CMD_RF_POWER_UP, (u16 *)&rf_pow_on_cmd_params,
		       sizeof(rf_pow_on_cmd_params) / 2, FMR_IR_CMD_DONE);
}

static void fmr_setup_band(void)
{
	struct fmtrx_band_cfg_cmd band_cfg = {0, 0};

	band_cfg.higher_band_limit = HZ_TO_KHZ(HIGHER_BAND_LIMIT);
	band_cfg.lower_band_limit  = HZ_TO_KHZ(LOWER_BAND_LIMIT);

	fmtrx_send_cmd(FMRX_CMD_CFG_BAND, (u16 *)&band_cfg,
		       sizeof(band_cfg) / 2, FMR_IR_CMD2_DONE);
}

static void fmrx_set_rds_config(struct fmrx_lld_rds_cfg *rds_cfg)
{
	struct fmrx_rds_cfg_cmd rds_cmd_prarams = {0, 0, 0};

	rds_cmd_prarams.rds_sync_cnt_good = rds_cfg->good_blocks;
	rds_cmd_prarams.rds_sync_cnt_bad = rds_cfg->bad_blocks_search;
	rds_cmd_prarams.rds_track_cnt_bad = rds_cfg->bad_blocks_sync;

	fmtrx_send_cmd(FMRX_CMD_CFG_RDS, (u16 *)&rds_cmd_prarams,
		       sizeof(rds_cmd_prarams) / 2, FMR_IR_CMD2_DONE);
}

int fmrx_set_fw_state(enum fmrx_sm_state sm_state)
{
	int rc = 0;
	u16 cmd_id = 0;

	/* Validate input arguments */
	if (sm_state <= FMRX_SM_STATE_INVALID) {
		fmdrv_err("Invalid arguments!\n");
		rc = -EINVAL;
		goto fmrx_hw_set_fw_state_err;
	}

	/* Select command id based on the requested FW sm_state */
	switch (sm_state) {
	case FMRX_SM_STATE_IDLE:
		cmd_id = FMRX_CMD_IDLE;
		break;

	case FMRX_SM_STATE_ACTIVE:
		cmd_id = FMRX_STATE_RECEIVE;
		break;

	default:
		rc = -EINVAL;
		goto fmrx_hw_set_fw_state_err;
	}

	/* Send command to DSP */
	rc = fmtrx_send_cmd(cmd_id, NULL, 0, FMR_IR_CMD_DONE);

	if (0 != rc)
		fmdrv_err("Send sm_state change command failed! rc %d\n", rc);

fmrx_hw_set_fw_state_err:
	return rc;
}

int fmr_set_gain_offsets(struct fmrx_state *state,
	enum gain_offset_type off_type, s16 *gain_offs, u32 size)
{
	int rc = 0;

	/* Validate input arguments */
	if (off_type > GAIN_OFFSET_INVALID || gain_offs == NULL || size == 0) {
		rc = -EINVAL;
		fmdrv_err("Invalid arguments!\n");
		goto fmr_set_gain_offsets_err;
	}

	switch (off_type) {
	case GAIN_OFFSET_LNA:
		rc = fmrx_set_rssi_lna_offsets(gain_offs, true);
		break;

	case GAIN_OFFSET_PPF:
		rc = fmrx_set_rssi_ppf_offsets(gain_offs, true);
		break;

	case GAIN_OFFSET_CP_INIT:
		/* Send RF power down command */
		fmtrx_send_cmd(FMRX_CMD_RF_POWER_DOWN, NULL, 0,
			       FMR_IR_CMD_DONE);

		fmrx_set_fw_state(FMRX_SM_STATE_IDLE);

		fmtrx_send_cmd(FMRX_CMD_CFG_ANT_TUNE_CP_INIT,
			       (u16 *)gain_offs,
			       (size * sizeof(u16)) / 2,
			       FMR_IR_CMD2_DONE);

		fmtrx_rf_poweron(state);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	if (0 != rc)
		fmdrv_err("Offset configure command failed! rc: %d\n", rc);
	else
		fmr_evaluate_channel(state, state->freq, RSSI_MIN);

fmr_set_gain_offsets_err:
	return rc;
}

int fmr_set_rssi_gain_offsets(struct fmrx_state *state,
	struct rssi_offs *gain_offs)
{
	struct rssi_offs rssi_offsets;
	int rc = 0;

	if (gain_offs == NULL)
		return -EINVAL;

	rssi_offsets.frequency1 = HZ_TO_KHZ(gain_offs->frequency1);
	rssi_offsets.frequency2 = HZ_TO_KHZ(gain_offs->frequency2);
	rssi_offsets.frequency3 = HZ_TO_KHZ(gain_offs->frequency3);
	rssi_offsets.frequency4 = HZ_TO_KHZ(gain_offs->frequency4);
	rssi_offsets.frequency5 = HZ_TO_KHZ(gain_offs->frequency5);

	rssi_offsets.offset1 = RSSI_TO_INT(gain_offs->offset1);
	rssi_offsets.offset2 = RSSI_TO_INT(gain_offs->offset2);
	rssi_offsets.offset3 = RSSI_TO_INT(gain_offs->offset3);
	rssi_offsets.offset4 = RSSI_TO_INT(gain_offs->offset4);
	rssi_offsets.offset5 = RSSI_TO_INT(gain_offs->offset5);
	rssi_offsets.offset6 = RSSI_TO_INT(gain_offs->offset6);

	rc = fmtrx_send_cmd(FMRX_CMD_CFG_RSSI_CH_OFFS,
			    (u16 *)&rssi_offsets,
			    sizeof(struct fmrx_rssi_ch_offs_cmd) / 2,
			    FMR_IR_CMD2_DONE);
	if (0 != rc)
		fmdrv_err("RSSI Gain Offset config failed! rc: %d\n", rc);
	else
		fmr_evaluate_channel(state, state->freq, RSSI_MIN);

	return rc;
}

u16 fmr_evaluate_channel(struct fmrx_state *state, u32 freq, s16 rssi_thres)
{
	struct fmtrx_ch_tune_cmd ch_tune_cmd_params = {0};

	/* close the clock switching feature */
	if (BOTH_OFF != state->rf_clk) {
		state->rf_clk = BOTH_OFF;
		fmr_clk_switch(state);
	}

	ch_tune_cmd_params.channel_freq = HZ_TO_KHZ(freq);
	ch_tune_cmd_params.force_meas = 0;
	ch_tune_cmd_params.rssi_threshold = rssi_thres;

	switch (state->cfg->rf_force_sb) {
	case FMR_FORCE_HSI:
		ch_tune_cmd_params.inj_side_sel =
			FMRX_CH_TUNE_INJ_SIDE_FORCE_HSI;
		break;

	case FMR_FORCE_LSI:
		ch_tune_cmd_params.inj_side_sel =
			FMRX_CH_TUNE_INJ_SIDE_FORCE_LSI;
		break;

	case FMR_FORCE_NONE:
		ch_tune_cmd_params.inj_side_sel = FMRX_CH_TUNE_INJ_SIDE_AUTO;
		break;

	default:
		ch_tune_cmd_params.inj_side_sel = FMRX_CH_TUNE_INJ_SIDE_AUTO;
		break;
	}

	/* Sending Channel tune command */
	fmtrx_send_cmd(FMRX_CMD_CH_TUNE, (u16 *)&ch_tune_cmd_params,
		       sizeof(ch_tune_cmd_params) / 2, FMR_IR_CMD_DONE);

	/* Check whether the channel tuning was successful */
	return fmtrx_read16(FMR_RXMAIN_CH_TUNE_STATUS_ADDR);
}

void fmrx_get_ch_status(struct fmrx_state *state)
{
	struct fmtrx_lld_aud_state audio_status;

	fmrx_get_audio_status(&audio_status);
	state->pilot_found = audio_status.is_stereo;
	state->freq = KHZ_TO_HZ(fmtrx_read32(FMR_RXMAIN_CH_INFO_CH_FREQ_ADDR));
	state->ch_info.inj_side =
		fmtrx_read16(FMR_RXMAIN_CH_INFO_INJ_SIDE_ADDR);
	state->ch_info.foffs = fmtrx_read32(FMR_RXMAIN_CH_INFO_FOFFS_ADDR);
	state->ch_info.hi_rssi.rssi =
		fmtrx_read16(FMR_RXMAIN_CH_INFO_HSI_RSSI_ADDR);
	state->ch_info.li_rssi.rssi =
		fmtrx_read16(FMR_RXMAIN_CH_INFO_LSI_RSSI_ADDR);
	state->ch_info.pn = fmtrx_read16(FMR_RXMAIN_CH_INFO_PN_ADDR);
	state->ch_info.rssi_eff = fmtrx_read16(FMR_RXMAIN_RSSI_ADDR);
}

void fmr_receiving_exit_actions(struct fmrx_state *state)
{
	/* Mute audio (returns quickly if already muted) */
	fmr_mute_wait();

	/* Prevent all events */
	fmtrx_enable_interrupts(~0U, false, CTX_THREAD);

	/* Disable RDS processing */
	fmrx_rds_enable((enum fmtrx_lld_rds_en) RDS_OFF);
}

void fmr_receiving_entry_actions(struct fmrx_state *state)
{
	u32 event_mask = 0;
	u32 clk_swt_min = CLK_SWT_MIN(state->cfg->other_cfg.clk_swt_rng);
	u32 clk_swt_max = CLK_SWT_MAX(state->cfg->other_cfg.clk_swt_rng);

	/* Clock switching around 104 MHz (+/- 500 KHz) */
	if ((BOTH_ON != state->rf_clk) && (state->freq > clk_swt_min) &&
	    (state->freq < clk_swt_max)) {
		state->rf_clk = BOTH_ON;
		fmr_clk_switch(state);
	}

	fmrx_get_ch_status(state);

	/* Set up RSSI thresholds */
	fmr_setup_rssi_thresholds(state);

	 /* Reset RDS when change stations */
	if (RDS_ON == state->cfg->rds_cfg.mode)
		fmr_reactive_rds(state);

	if (state->mcfg->rssi_enable)
		event_mask |= IR_RX_RSSI;

	/* event_mask |= IR_RX_PILOT; */

	/* Enable RSSI and RDS events if appropriate, and pilot event always */
	fmtrx_enable_interrupts(event_mask, true, CTX_THREAD);

	/* Turn back on sound, according to current setting */
	fmrx_audio_mute(state->cfg->mute);
	fmrx_audio_processing_enable(!state->cfg->mute);
}

int fmr_start_receiver(struct fmrx_state *state)
{
	struct fmrx_lld_rds_cfg rds_config;
	struct fmtrx_lld_rssi_cfg rssi_cfg;
	struct fmtrx_lld_aud_cfg audio_conf;
	s16 snc_step = 0;
	struct fmrx_sysclk_cmd mdsp_clk_cmd_params = {0};
	int rc = 0;
	u16 fmr_fw_size;
	const u8 *fmr_fw_data;

	/* System initialization */
	fmr_sys_power_enable(true, true);

	/* enable all FMR internal clocks */
	fmtrx_init_subsystem();

	/* Fetch firmware image */
	rc = fmr_sys_fetch_fw(FMR_MODULE_FMRX, &fmr_fw_data, &fmr_fw_size);

	/* Return if fetch is not successful */
	if (0 != rc) {
		fmr_sys_power_enable(false, false);
		return rc;
	}

	/* Download firmware image */
	fmtrx_download_fw((u8 *)0, fmr_fw_size, (u8 *)fmr_fw_data);
	fmr_sys_release_fw(FMR_MODULE_FMRX);

	rc = fmr_sys_request_irq();
	if (rc) {
		/* turn off the fmr power domain */
		fmr_sys_power_enable(false, false);
		fmdrv_dbg("Failed to request IRQ lines\n");
		return rc;
	}

	/* Start mdsp running, this will change fw mode to FMRX_STATE_IDLE */
	fmtrx_run_mini_dsp(0);

	/* Configuration for clock frequency of the mini DSP subsystem */
	mdsp_clk_cmd_params.minidsp_clk = 3; /* To use 69 MHz */
	fmtrx_send_cmd(FMRX_CMD_CFG_SYSCLK, (u16 *)&mdsp_clk_cmd_params,
		       sizeof(mdsp_clk_cmd_params) / 2, FMR_IR_CMD2_DONE);

	/* Cp init table configuration */
	if (state->cfg->antenna == FMR_ANT_EBD_DE)
		fmtrx_send_cmd(FMRX_CMD_CFG_ANT_TUNE_CP_INIT,
			       (u16 *)state->cfg->cp_init_offs,
			       (sizeof(state->cfg->cp_init_offs) *
			       sizeof(u16)) / 2, FMR_IR_CMD2_DONE);

	fmtrx_rf_poweron(state);

	/* Set up the frequency band */
	fmr_setup_band();

	/* Set deemphasis time constant */
	if (FMR_AUD_PATH_DAC == state->cfg->aud_path) {
		if (FMR_50US == state->cfg->band.deem)
			fmrx_set_deemphasis(DEEM_50us_DAC);
		else
			fmrx_set_deemphasis(DEEM_75us_DAC);
	} else {
		if (FMR_50US == state->cfg->band.deem)
			fmrx_set_deemphasis(DEEM_50us_SRC);
		else
			fmrx_set_deemphasis(DEEM_75us_SRC);
	}

	/* Audio configuration */
	audio_conf.audiogain_iir_param = state->cfg->other_cfg.vol_ramp;
	fmrx_set_audio_config(audio_conf);

	/* Set audio parameters for mast gain and dac volume */
	fmrx_audio_mute(true);

	fmrx_set_aud_volume_dac(
		fmr_host_to_fmr_volume(state->cfg->vol_cfg.left),
		fmr_host_to_fmr_volume(state->cfg->vol_cfg.right));

	fmrx_set_aud_volume_src(
		fmr_host_to_fmr_volume(state->cfg->vol_cfg.left),
		fmr_host_to_fmr_volume(state->cfg->vol_cfg.right));

	/* Firstly mute the audio output, unmute it in entry action */
	fmrx_audio_processing_enable(false);

	/* Set up audio routing, direct to DAC mode */
	fmrx_set_output((enum fmtrx_lld_aud_routing)(LLD_FMR_AUD_ROUTING_DAC));

	if (state->cfg->antenna == FMR_ANT_HS_SE) {
		rssi_cfg.rssi_lna_offs = state->cfg->lna_offs_ext_ant;
		rssi_cfg.rssi_ppf_offs = state->cfg->ppf_offs_ext_ant;

		/* channel frequency dependent RSSI offset configuration */
		fmr_set_rssi_gain_offsets(state,
					  &state->cfg->rssi_offs_ext_ant);
	} else {
		rssi_cfg.rssi_lna_offs = state->cfg->lna_offs_int_ant;
		rssi_cfg.rssi_ppf_offs = state->cfg->ppf_offs_int_ant;

		fmr_set_rssi_gain_offsets(state,
					  &state->cfg->rssi_offs_int_ant);
	}

	rssi_cfg.rssi_other_offset = fmr_calc_rssi_other_offs(state);

	/* Copy RSSI other offset to FW */
	fmrx_set_rssi_other_offs(rssi_cfg.rssi_other_offset);

	/* Copy LNA RSSI offsets to FW */
	fmrx_set_rssi_lna_offsets(rssi_cfg.rssi_lna_offs, true);

	/* Copy PPF RSSI offsets to FW */
	fmrx_set_rssi_ppf_offsets(rssi_cfg.rssi_ppf_offs, true);

	/* Configure for SoftMute */
	fmrx_set_soft_mute(state->cfg->sm_cfg.en,
			   state->cfg->sm_cfg.step / 4,
			   RSSI_TO_INT(state->cfg->sm_cfg.thr));

	/* To avoid division by zero */
	if ((0 == state->cfg->snc.hi_thr) && (0 == state->cfg->snc.lo_thr)) {
		snc_step = 0;
	} else {
		s16 thres_diff = (state->cfg->snc.hi_thr -
				  state->cfg->snc.lo_thr);
		snc_step = state->mcfg->blend_max / thres_diff;
	}

	/* Configure for SNC */
	fmrx_set_snc_cfg(state->cfg->snc.en, snc_step,
			 RSSI_TO_INT(state->cfg->snc.hi_thr),
			 RSSI_TO_INT(state->cfg->snc.lo_thr));

	/* Configure for AGC */
	fmrx_set_agc_gain(state->cfg->agc_cfg.en,
			  state->cfg->agc_cfg.gain_idx);

	/* Set up RDS parameters */
	rds_config.mode	= (enum fmtrx_lld_rds_mode) state->mcfg->rds_type;
	rds_config.good_blocks	= state->mcfg->rds_sync_cfg.gblk;
	rds_config.bad_blocks_sync = state->mcfg->rds_sync_cfg.bblk;
	rds_config.bad_blocks_search = state->mcfg->rds_sync_cfg.bblks;
	fmrx_set_rds_config(&rds_config);

	/* Prevent any events (they will be enabled later) */
	fmtrx_enable_interrupts(~0U, false, CTX_THREAD);

	/* Set frequency, use minimum threshold */
	fmr_evaluate_channel(state, state->freq, RSSI_MIN);

	return rc;
}

void fmr_stop_receiver(struct fmrx_state *state)
{
	/* Mute audio, ~1ms between checks, ~10ms step delay */
	fmr_mute_wait();

	/* disable IDI RX transfer */
	if (FMR_AUD_PATH_OFF == state->cfg->aud_path)
		fmtrx_set_idi_xfr(true, false);

	/* Send RF power down command */
	fmtrx_send_cmd(FMRX_CMD_RF_POWER_DOWN, NULL, 0, FMR_IR_CMD_DONE);

	/* Prevent all further interrupts from FMR Macro */
	fmtrx_enable_interrupts(~0U, false, CTX_THREAD);

	/* Release FM Radio IRQ lines */
	fmr_sys_release_irq();

	/* Stop minidsp */
	fmrx_send_cmd(FMRX_CMD_HALT, NULL, 0, false);
	fmr_sys_idle_wait(50);

	/* turn off the fmr power domain */
	fmr_sys_power_enable(false, false);
}

void fmr_mute_wait(void)
{
	struct fmtrx_lld_aud_state audio_state;

	/* Number of blocks to sleep between checks */
	u16 blkres = 6;

	/* Set volume to zero, volume will start to decrease */
	fmrx_audio_mute(true);

	do {
		fmrx_get_audio_status(&audio_state);

		/* Volume not yet reached zero, sleep for a while */
		fmr_sys_busy_wait(blkres * 160);
	} while ((audio_state.audio_en) &&
		 ((audio_state.gains.audio_gain_l != 0) ||
		 (audio_state.gains.audio_gain_r != 0)));

	/* Volume has now reached zero, we can disable (mute) the audio
	processing */
	fmrx_audio_processing_enable(false);
}

void fmr_reactive_rds(struct fmrx_state *state)
{
	u32 event_mask = 0;
	struct fmrx_lld_rds_cfg rds_conf;

	/* RDS configuration of HW, all default vlaue or reconfigured value
	 * before the poweron would be used heres
	 */
	rds_conf.bad_blocks_search = (u16) state->mcfg->rds_sync_cfg.bblks;
	rds_conf.bad_blocks_sync = (u16) state->mcfg->rds_sync_cfg.bblk;
	rds_conf.good_blocks = (u16) state->mcfg->rds_sync_cfg.gblk;
	rds_conf.mode = (enum fmtrx_lld_rds_mode) state->mcfg->rds_type;
	fmrx_set_rds_config(&rds_conf);

	if (RDS_MODE_IRQ == state->mcfg->rds_mode) {
		if (state->mcfg->rdssync_en)
			event_mask |= IR_RX_RDSSYNC;

		event_mask |= IR_RX_RDS;

		fmtrx_enable_interrupts(event_mask, true, CTX_THREAD);
	} else	/* Disable the RDS interrupt */
		fmtrx_enable_interrupts(IR_RX_RDS | IR_RX_RDSSYNC, false,
					CTX_THREAD);

	/* Reset the RDS */
	fmrx_rds_reset();
	fmrx_rds_set_pi_mode(state->mcfg->rds_pi_mode);
	fmrx_rds_enable((enum fmtrx_lld_rds_en) state->mcfg->rds_mode);
	fmrx_rds_set_notify_interval(state->mcfg->rds_min_free);
}

s16 fmr_calc_rssi_nf_offset(const struct fmtrx_rssi_data *rssi_data)
{
	s16 rssi_nf_offset;
	s16 thr_lna, thr_ppf, thr_ppf2nd;

	thr_lna = (s16)((15 - rssi_data->lna_gain) * RSSI_THR_NF_LNA);
	thr_ppf = (s16)((15 - rssi_data->ppf_gain) * RSSI_THR_NF_PPF);
	thr_ppf2nd = (s16)((1 - rssi_data->ppf_2nd)
		* RSSI_THR_NF_PPF2ND);

	rssi_nf_offset = thr_lna + thr_ppf + thr_ppf2nd;
	return rssi_nf_offset;
}

enum fmrx_ch_search_status fmr_check_ch_search_status(struct fmrx_state *state)
{
	enum fmrx_ch_search_status rc = FMRX_CH_SEARCH_STAT_INVALID;
	s16 ch_search_status = fmtrx_read16(FMR_RXMAIN_CH_SEARCH_STATUS_ADDR);

	fmdrv_info("Ch search status: %u\n", ch_search_status);
	if (FMRX_CH_SEARCH_STATUS_OK == ch_search_status) {
		fmrx_get_ch_status(state);
		rc = FMRX_CH_SEARCH_STAT_OK;
	} else {
		rc = FMRX_CH_SEARCH_STAT_FAIL;
	}

	return rc;
}

void fmr_send_ch_search_cmd(struct fmrx_state *state,
	struct fmtrx_ch_search_cmd *ch_search_params)
{
	switch (state->cfg->rf_force_sb) {
	case FMR_FORCE_HSI:
		ch_search_params->inj_side_sel =
			FMRX_CH_TUNE_INJ_SIDE_FORCE_HSI;
		break;

	case FMR_FORCE_LSI:
		ch_search_params->inj_side_sel =
			FMRX_CH_TUNE_INJ_SIDE_FORCE_LSI;
		break;

	case FMR_FORCE_NONE:
		ch_search_params->inj_side_sel = FMRX_CH_TUNE_INJ_SIDE_AUTO;
		break;

	default:
		ch_search_params->inj_side_sel = FMRX_CH_TUNE_INJ_SIDE_AUTO;
		break;
	}

	fmtrx_send_cmd(FMRX_CMD_CH_SEARCH, (u16 *)ch_search_params,
		       sizeof(struct fmtrx_ch_search_cmd) / 2,
		       FMR_IR_CMD_DONE);
}

s16 fmr_peek_ext_lna_table(struct fmrx_state *state)
{
	u8 i = 0;
	s16 rc = 0;
	s8 ext_lna_tab_len = 0;
	s8 *ext_lna_offs = NULL;
	u32 *ext_lna_band_split = NULL;
	struct fmrx_ext_lna_cfg *ext_lna = NULL;

	/* Get compensation table in terms of antenna type */
	ext_lna = &state->cfg->ext_lna_cfg[state->cfg->antenna];
	ext_lna_tab_len = ext_lna->tab_len;
	ext_lna_offs = ext_lna->offsets;
	ext_lna_band_split = ext_lna->band_split;

	/* Compensation gain according band */
	if (0 == ext_lna_tab_len) {
		if (NULL != ext_lna_offs)
			rc = RSSI_TO_INT(ext_lna_offs[0]);
		else
			rc = 0;
	} else {
		for (i = 0; i < ext_lna_tab_len; i++) {
			if (ext_lna_band_split[i] >= state->freq ||
			    ext_lna_band_split[i] == 0) {
				rc = RSSI_TO_INT(ext_lna_offs[i]);
				break;
			}
		}
	}

	return rc;
}

u16 fmr_host_to_fmr_volume(u8 host_volume)
{
	if (host_volume >= AUD_MAX_VOL)
		host_volume = AUD_MAX_VOL;

	return fmrx_volume_mapping[host_volume];
}

void fmr_clk_switch(struct fmrx_state *state)
{
	struct fmrx_clk_sel_cmd clk_sel_cfg = {0};

	switch (state->rf_clk) {
	case BOTH_ON: {
		clk_sel_cfg.clk_sel = 1;

		/* Sending Channel tune command */
		fmtrx_send_cmd(FMRX_CMD_CFG_CLK_SEL, (u16 *)&clk_sel_cfg,
			       sizeof(clk_sel_cfg) / 2, FMR_IR_CMD2_DONE);

		/* Switching to internal rf clock */
		fmr_sys_clock_sel(FMR_CLK_SEL_RF);
	}
	break;

	default: { /* BOTH_OFF */
		clk_sel_cfg.clk_sel = 0;

		/* Switching back to main clock */
		fmr_sys_clock_sel(FMR_CLK_SEL_PLL);

		/* Sending Channel tune command */
		fmtrx_send_cmd(FMRX_CMD_CFG_CLK_SEL, (u16 *)&clk_sel_cfg,
			       sizeof(clk_sel_cfg) / 2, FMR_IR_CMD2_DONE);
	}
	break;
	}
}

void fmr_setup_rssi_thresholds(struct fmrx_state *state)
{
	struct fmtrx_lld_ev_thres thresholds;
	s16 rssi_low;
	s16 rssi_up;

	if (state->mcfg->rssi_lower <= -60) {       /* Lowest: -122 dBm */
		/* Set lower boundary to minimum negative RSSI
		(disable lower reporting) */
		rssi_low = -10000;
	} else {
		/* Set lower boundary mid-way between current and next lower */
		rssi_low = state->mcfg->rssi_lower - RSSI_VERBOSE_STEP / 2;
	}

	if (state->mcfg->rssi_upper >= 400) { /* Highest: -7 dBm */
		/* Set upper boundary to maximum positive RSSI
		(disable upper reporting) */
		rssi_up = 10000;
	} else {
		/* Set upper boundary midway between current and next higher */
		rssi_up = state->mcfg->rssi_upper + RSSI_VERBOSE_STEP / 2;
	}

	thresholds.rssi_lower = rssi_low;
	thresholds.rssi_upper = rssi_up;
	fmrx_set_rssi_thresholds(&thresholds);
}

void fmr_ant_switch(struct fmrx_state *state)
{
	struct fmtrx_lld_rssi_cfg rssi_cfg;
	u32 clk_swt_min = CLK_SWT_MIN(state->cfg->other_cfg.clk_swt_rng);
	u32 clk_swt_max = CLK_SWT_MAX(state->cfg->other_cfg.clk_swt_rng);

	/* Clock switching around 104 MHz (+/- 500 KHz) */
	if ((state->freq > clk_swt_min) &&
	    (state->freq < clk_swt_max)) {
		/* close the clock switching feature */
		if (BOTH_OFF != state->rf_clk) {
			state->rf_clk = BOTH_OFF;
			fmr_clk_switch(state);
		}
	}

	/* Send RF power down command */
	fmtrx_send_cmd(FMRX_CMD_RF_POWER_DOWN, NULL, 0, FMR_IR_CMD_DONE);

	fmrx_set_fw_state(FMRX_SM_STATE_IDLE);

	/* Cp init table configuration */
	if (state->cfg->antenna == FMR_ANT_EBD_DE)
		fmtrx_send_cmd(FMRX_CMD_CFG_ANT_TUNE_CP_INIT,
			       (u16 *)state->cfg->cp_init_offs,
			       (sizeof(state->cfg->cp_init_offs) *
			       sizeof(u16)) / 2, FMR_IR_CMD2_DONE);

	fmtrx_rf_poweron(state);

	if (state->cfg->antenna == FMR_ANT_HS_SE) {
		rssi_cfg.rssi_lna_offs = state->cfg->lna_offs_ext_ant;
		rssi_cfg.rssi_ppf_offs = state->cfg->ppf_offs_ext_ant;

		fmr_set_rssi_gain_offsets(state,
					  &state->cfg->rssi_offs_ext_ant);
	} else {
		rssi_cfg.rssi_lna_offs = state->cfg->lna_offs_int_ant;
		rssi_cfg.rssi_ppf_offs = state->cfg->ppf_offs_int_ant;

		fmr_set_rssi_gain_offsets(state,
					  &state->cfg->rssi_offs_int_ant);
	}

	/* Copy LNA RSSI offsets to FW */
	fmrx_set_rssi_lna_offsets(rssi_cfg.rssi_lna_offs, true);

	/* Copy PPF RSSI offsets to FW */
	fmrx_set_rssi_ppf_offsets(rssi_cfg.rssi_ppf_offs, true);
}

void fmtrx_enable_test_trace_int(s32 en)
{
	fmtrx_enable_interrupts(IR_TRX_TRACE, en, CTX_THREAD);
}

void fmrx_set_agc_gain(u8 agc_enable, u16 gain_index)
{
	struct fmtrx_agc_cmd agc_cmd_params = {0};

	if (agc_enable)
		agc_cmd_params.agc_en = AGC_ENABLED;
	else
		agc_cmd_params.agc_en = AGC_DISABLED;

	agc_cmd_params.fixed_gain_idx = gain_index;
	agc_cmd_params.reserved1 = 0;
	agc_cmd_params.reserved2 = 0;

	fmtrx_send_cmd(FMRX_CMD_CFG_AGC, (u16 *)&agc_cmd_params,
		       sizeof(agc_cmd_params) / 2, FMR_IR_CMD2_DONE);
}

