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
 *
 * @file aud_app_fmr_hld_rx.c
 *
 * Implementation of the FM Radio Rx Module functionality.
 * This includes FM Radio Rx SM, event handler, IRQ handler and Rx object.
 *
 */

#include <linux/err.h>

#include <aud_app_fmr_hld.h>
#include <aud_app_fmr_hld_rx.h>       /* Header for this file */
#include <aud_app_fmr_lld_rx_rf.h>
#include <aud_app_fmr_lld_api_rx.h>
#include <aud_app_fmr_sys_os.h>
#include <aud_app_fmr_sys_msg.h>
#include <aud_app_fmr_sys.h>
#include <fmr_rx_api.h>

/* GLOBAL DEFINITIONS */
#define FMRX_SWEEP_BAND_MIN_FREQ	(65  * 1000000)	/* in Hz */
#define FMRX_SWEEP_BAND_MAX_FREQ	(200 * 1000000)	/* in Hz */
#define FMRX_SWEEP_BAND_STEP_SIZE	(50000)	/* in Hz */
#define RDS_MAX_VALID_GRPS		(FMRX_RDS_MAX_VALID_GRPS * 4)
#define FMTRX_IP_REG_START_ADD		(FMR_RXMAIN_HOSTIF_BUF_ADDR)
#define FMTRX_IP_REG_END_ADD		(0x8780)

/* LOCAL FUNCTION DECLARATIONS */

/* FM Rx module supporting functions */
static int fmrx_initialise_state(void);

/* FM Rx state dependent, event handlers */
static int fmrx_idle(struct fmrx_msgbox_buff *p_msg);
static int fmrx_normal_receive(struct fmrx_msgbox_buff *p_msg);
static int fmrx_seek(struct fmrx_msgbox_buff *p_msg);

/* FM Rx SM handling functions */
static void fmrx_set_sm_state(enum fmrx_sm_state new_state);
static enum fmrx_sm_state fmrx_get_sm_state(void);

/* FM Rx Module IRQ helper functions */
static void fmrx_rdssync_cb(void);
static void fmrx_rds_cb(void);
static void fmrx_rssi_cb(s16 rssi);
static void fmrx_pilot_cb(void);
static void fmrx_timer_cb(void *args);

/* LOCAL DATA DEFINITIONS */

/*  RDS fast pi timer for AF evaluation in ms */
static u16 fast_pi_timer_duration = 2000;

/* Internal FM Rx static state */
static struct fmrx_state rx_state;
static struct fmrx_misc_cfg misc_cfg;

/* LOCAL FUNCTIONS DEFINITIONS */

/* Set the FM Rx SM state */
static void fmrx_set_sm_state(const enum fmrx_sm_state new_state)
{
	rx_state.curr_sm_state = new_state;
}

/* Inquire the FM Rx SM state */
static enum fmrx_sm_state fmrx_get_sm_state(void)
{
	return rx_state.curr_sm_state;
}

/* FM Rx state is initialized to default values */
static int fmrx_initialise_state(void)
{
	int rc = 0;
	memset(&rx_state, 0, sizeof(struct fmrx_state));
	memset(&misc_cfg, 0, sizeof(struct fmrx_misc_cfg));

	misc_cfg.rds_type = FMR_RBDS;
	misc_cfg.rds_min_free = 2;
	misc_cfg.rds_pi_mode = RDS_NORMAL_PI;
	misc_cfg.rds_sync_cfg.bblks = 4;
	misc_cfg.rds_sync_cfg.bblk = 7;
	misc_cfg.rds_sync_cfg.gblk = 4;
	misc_cfg.rds_mode = RDS_MODE_IRQ;
	misc_cfg.blend_max = 32767;
	misc_cfg.rssi_enable = false;	/* Disable RSSI reporting */
	misc_cfg.ant_selftest_low_thr = 50;
	misc_cfg.ant_selftest_high_thr = 2000;

	/* Frequency 88 MHz (allowed in most countries) */
	rx_state.freq = 88 * MHZ;
	rx_state.seek_step = 100 * KHZ;
	rx_state.pilot_found = false;
	rx_state.scan_status = SCAN_SUCCESS;
	rx_state.idi_hs_ctrl = false;
	rx_state.rf_clk = BOTH_OFF;

	rx_state.cfg = NULL;
	rx_state.mcfg = &misc_cfg;

	fmrx_set_sm_state(FMRX_SM_STATE_IDLE);
	rx_state.prev_sm_state = FMRX_SM_STATE_IDLE;

	return rc;
}

static void fmrx_get_config(struct fmrx_cfg **cfg)
{
	*cfg = rx_state.cfg;
}

static int fmrx_get_ch_info(struct fmrx_ch_info *ch_info)
{
	int rc = 0;
	struct fmtrx_lld_aud_state audio_status;

	if (NULL == ch_info) {
		rc = -EINVAL;
		goto fmrx_get_ch_info_err;
	}

	/* get the audio status */
	fmrx_get_audio_status(&audio_status);

	/* update the channel info */
	fmrx_get_ch_status(&rx_state);

	ch_info->state = fmrx_get_hw_state();
	ch_info->freq = rx_state.freq;
	ch_info->mono = audio_status.is_stereo;
	ch_info->pilot_found = audio_status.is_stereo;
	ch_info->rssi = RSSI_TO_EXT(fmrx_get_rssi());
	ch_info->injside = (u8)rx_state.ch_info.inj_side;
	ch_info->pn = (u16)fmrx_get_phase_noise();
	ch_info->foffs = fmrx_get_freq_offs_int();
	ch_info->pilotamp = fmrx_get_pilot_ampl();
	ch_info->seek_state = rx_state.scan_status;
	ch_info->rds_en = (u32)rx_state.cfg->rds_cfg.mode;
	ch_info->rds_mode = (u32)misc_cfg.rds_type;
	ch_info->rds_sync = fmrx_rds_get_sync();

fmrx_get_ch_info_err:
	return rc;
}

static int fmrx_set_config(struct fmrx_cfg *const cfg)
{
	if (cfg == NULL)
		return -EINVAL;

	rx_state.cfg = cfg;
	return 0;
}

static int fmrx_set_band_info(struct fmtrx_band *set_band)
{
	int rc = 0;

	if (NULL == set_band) {
		rc = -EINVAL;
		goto fmrx_set_band_info_err;
	}

	/* Validate input arguments */
	if ((0 == set_band->min) || (0 == set_band->max) ||
	    (set_band->min >= set_band->max) ||
	    (LOWER_BAND_LIMIT > set_band->min) ||
	    (HIGHER_BAND_LIMIT < set_band->max) ||
	    (FMR_75US < set_band->deem)) {
		rc = -EINVAL;
		fmdrv_err("Invalid arguments!\n");
	} else {
		rx_state.cfg->band.min = set_band->min;
		rx_state.cfg->band.max = set_band->max;
		rx_state.seek_step = set_band->step;
		rx_state.cfg->band.deem = set_band->deem;
	}

fmrx_set_band_info_err:
	return rc;
}

static int fmtrx_read_reg(struct fmtrx_reg_data *p_read_reg)
{
	int rc = 0;

	if (NULL == p_read_reg) {
		rc = -EINVAL;
		goto fmtrx_read_reg_err;
	}

	if (p_read_reg->reg_addr < FMTRX_IP_REG_START_ADD ||
	    p_read_reg->reg_addr > FMTRX_IP_REG_END_ADD) {
		rc = -EINVAL;
		goto fmtrx_read_reg_err;
	}

	if (FMTRX_16BIT_ACCESS == p_read_reg->reg_type)
		*p_read_reg->reg_data =
			(s32)fmtrx_read16(p_read_reg->reg_addr);
	else
		*p_read_reg->reg_data = fmtrx_read32(p_read_reg->reg_addr);

fmtrx_read_reg_err:
	return rc;
}

static int fmtrx_write_reg(struct fmtrx_reg_data *p_write_reg)
{
	int rc = 0;

	if (NULL == p_write_reg) {
		rc = -EINVAL;
		goto fmtrx_write_reg_err;
	}

	if (p_write_reg->reg_addr < FMTRX_IP_REG_START_ADD ||
	    p_write_reg->reg_addr > FMTRX_IP_REG_END_ADD) {
		rc = -EINVAL;
		goto fmtrx_write_reg_err;
	}

	if (FMTRX_16BIT_ACCESS == p_write_reg->reg_type)
		fmtrx_write16(p_write_reg->reg_addr, (*p_write_reg->reg_data));
	else
		fmtrx_write32(p_write_reg->reg_addr, (*p_write_reg->reg_data));

fmtrx_write_reg_err:
	return rc;
}

int fmtrx_set_gain_offsets(struct gain_offsets *gain_cfg)
{
	int rc = 0;
	u16 *gain_offsets = NULL;
	bool is_ext = false;

	/* Validate input arguments */
	if (gain_cfg == NULL)
		return -EINVAL;

	/* Validate antenna type */
	if (gain_cfg->antenna >= FMR_ANT_TYPE_END || gain_cfg->offsets == NULL)
		return -EINVAL;

	is_ext = (FMR_ANT_HS_SE == gain_cfg->antenna) ? true : false;

	switch (gain_cfg->off_type) {
	case GAIN_OFFSET_LNA:
		gain_offsets = is_ext ? rx_state.cfg->lna_offs_ext_ant :
			rx_state.cfg->lna_offs_int_ant;
		break;

	case GAIN_OFFSET_PPF:
		gain_offsets = is_ext ? rx_state.cfg->ppf_offs_ext_ant :
			rx_state.cfg->ppf_offs_int_ant;
		break;

	case GAIN_OFFSET_CP_INIT:
		gain_offsets = rx_state.cfg->cp_init_offs;
		break;

	default:
		rc = -EINVAL;
		goto fmtrx_set_gain_offsets_err;
	}

	if (rx_state.cfg->antenna == gain_cfg->antenna) {
		switch (gain_cfg->off_type) {
		case GAIN_OFFSET_LNA:
		case GAIN_OFFSET_PPF:
			rc = fmr_set_gain_offsets(&rx_state,
						  gain_cfg->off_type,
						  gain_cfg->offsets,
						  sizeof(gain_cfg->offsets));
			if (rc != 0)
				goto fmtrx_set_gain_offsets_err;
			break;

		case GAIN_OFFSET_CP_INIT:
			/* If antenna is not embedded, not required to copy
			 * offsets to FW, but stored for later use */
			if (gain_cfg->antenna != FMR_ANT_EBD_DE)
				break;

			fmr_receiving_exit_actions(&rx_state);

			fmr_set_gain_offsets(&rx_state, gain_cfg->off_type,
					     gain_cfg->offsets,
					     sizeof(gain_cfg->offsets));

			fmr_evaluate_channel(&rx_state, rx_state.freq,
					     RSSI_MIN);
			fmr_receiving_entry_actions(&rx_state);
			break;

		default:
			break;
		}
	}

	memcpy((u8 *)gain_offsets, (u8 *)gain_cfg->offsets, OFFS_ARRAY_SIZE);

fmtrx_set_gain_offsets_err:
	return rc;
}

int fmtrx_get_gain_offsets(struct gain_offsets *gain_cfg)
{
	int rc = 0;
	u16 *gain_offsets = 0;
	bool is_ext = false;

	/* Validate input arguments */
	if (gain_cfg == NULL)
		return -EINVAL;

	/* Validate antenna type */
	if (FMR_ANT_TYPE_END <= gain_cfg->antenna)
		return -EINVAL;

	is_ext = (FMR_ANT_HS_SE == gain_cfg->antenna) ? true : false;

	switch (gain_cfg->off_type) {
	case GAIN_OFFSET_LNA:
		gain_offsets = is_ext ? rx_state.cfg->lna_offs_ext_ant :
			rx_state.cfg->lna_offs_int_ant;
		break;

	case GAIN_OFFSET_PPF:
		gain_offsets = is_ext ? rx_state.cfg->ppf_offs_ext_ant :
			rx_state.cfg->ppf_offs_int_ant;
		break;

	case GAIN_OFFSET_CP_INIT:
		gain_offsets = rx_state.cfg->cp_init_offs;
		break;

	default:
		rc = -EINVAL;
		goto fmtrx_get_gain_offsets_err;
	}

	memcpy(&gain_cfg->offsets, gain_offsets, OFFS_ARRAY_SIZE);

fmtrx_get_gain_offsets_err:
	return rc;
}

int fmtrx_set_gain_rssi_offsets(struct rssi_offsets *gain_cfg)
{
	int rc = 0;
	struct rssi_offs *rssi_offs = NULL;
	bool is_ext = false;
	u32 band_min = rx_state.cfg->band.min;
	u32 band_max = rx_state.cfg->band.max;

	/* Validate input arguments */
	if (gain_cfg == NULL)
		return -EINVAL;

	if (gain_cfg->antenna >= FMR_ANT_TYPE_END ||
	    gain_cfg->off_type != GAIN_OFFSET_RSSI)
		return -EINVAL;

	if (!(gain_cfg->offsets.frequency1 >= band_min &&
	      gain_cfg->offsets.frequency1 <= band_max &&
	      gain_cfg->offsets.frequency2 >= band_min &&
	      gain_cfg->offsets.frequency2 <= band_max &&
	      gain_cfg->offsets.frequency3 >= band_min &&
	      gain_cfg->offsets.frequency3 <= band_max &&
	      gain_cfg->offsets.frequency4 >= band_min &&
	      gain_cfg->offsets.frequency4 <= band_max &&
	      gain_cfg->offsets.frequency5 >= band_min &&
	      gain_cfg->offsets.frequency5 <= band_max)) {
		rc = -EINVAL;
		fmdrv_err("Invalid arguments!\n");
		goto fmtrx_set_gain_rssi_offsets_err;
	}

	is_ext = (FMR_ANT_HS_SE == gain_cfg->antenna) ? true : false;

	if (rx_state.cfg->antenna == gain_cfg->antenna) {
		rc = fmr_set_rssi_gain_offsets(&rx_state, &gain_cfg->offsets);
		if (0 != rc)
			goto fmtrx_set_gain_rssi_offsets_err;
	}

	rssi_offs = is_ext ? &rx_state.cfg->rssi_offs_ext_ant :
		&rx_state.cfg->rssi_offs_int_ant;

	memcpy(rssi_offs, &gain_cfg->offsets, sizeof(struct rssi_offs));

fmtrx_set_gain_rssi_offsets_err:
	return rc;
}

int fmtrx_get_gain_rssi_offsets(struct rssi_offsets *gain_cfg)
{
	int rc = 0;
	struct rssi_offs *rssi_offs = NULL;
	bool is_ext = false;

	/* Validate input arguments */
	if (gain_cfg == NULL)
		return -EINVAL;

	/* Validate antenna type */
	if (gain_cfg->antenna >= FMR_ANT_TYPE_END ||
	    gain_cfg->off_type != GAIN_OFFSET_RSSI)
		return -EINVAL;

	is_ext = (FMR_ANT_HS_SE == gain_cfg->antenna) ? true : false;

	rssi_offs = is_ext ? &rx_state.cfg->rssi_offs_ext_ant :
		&rx_state.cfg->rssi_offs_int_ant;

	memcpy(&gain_cfg->offsets, rssi_offs, sizeof(struct rssi_offs));

	return rc;
}

/* Event handler when FM Rx module is in idle state */
static int fmrx_idle(struct fmrx_msgbox_buff *p_msg)
{
	int rc = 0;

	if (NULL == p_msg) {
		rc = -EINVAL;
		goto fmrx_idle_err;
	}

	switch (p_msg->event) {
	case FMRX_EVENT_POWER_ON:
		rc = fmr_start_receiver(&rx_state);
		if (rc != 0)
			return rc;

		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);
		fmr_receiving_entry_actions(&rx_state);

#ifdef DEBUG_FMR_HLD
		fmdrv_info("FMRX_SM_STATE_IDLE >> FMRX_SM_STATE_ACTIVE\n");
#endif

		/* Callback for PowerOn to the upper layer */
		if (NULL != misc_cfg.fmrx_cbs->p_pow_onoff_cb)
			misc_cfg.fmrx_cbs->p_pow_onoff_cb(FMRX_SWITCHED_ON);
		break;

	case FMRX_EVENT_SET_BAND:
		rc = fmrx_set_band_info(p_msg->params.p_set_freq_band);
		break;

	case FMRX_EVENT_REGISTER_CBS:
		if (NULL != p_msg->params.p_reg_cbs)
			misc_cfg.fmrx_cbs = p_msg->params.p_reg_cbs;
		else
			rc = -EINVAL;
		break;

	case FMRX_EVENT_SET_CONFIG_DATA:
		rc = fmrx_set_config(p_msg->params.p_set_cfg);
		break;

	default:
#ifdef DEBUG_FMR_HLD
		fmdrv_info("%s: Incorrect cfg. Cmd : %u, has been ignored!\n",
			   __func__, p_msg->event);
#endif
		rc = -EPERM;
		break;
	}

fmrx_idle_err:
	return rc;
}

/* Event handler when FM Rx is active */
static int fmrx_normal_receive(struct fmrx_msgbox_buff *p_msg)
{
	int rc = 0;

	if (NULL == p_msg) {
		rc = -EINVAL;
		goto fmrx_normal_receive_err;
	}

	switch (p_msg->event) {
	case FMRX_EVENT_POWER_OFF: {
		if (BOTH_OFF != rx_state.rf_clk) {
			rx_state.rf_clk = BOTH_OFF;
			fmr_clk_switch(&rx_state);
		}

		fmr_stop_receiver(&rx_state);
		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_IDLE);

#ifdef DEBUG_FMR_HLD
		fmdrv_info("FMRX_SM_STATE_ACTIVE >> FMRX_SM_STATE_IDLE\n");
#endif

		/* Callback for PowerOff to the upper layer */
		if (NULL != misc_cfg.fmrx_cbs->p_pow_onoff_cb)
			misc_cfg.fmrx_cbs->p_pow_onoff_cb(FMRX_SWITCHED_OFF);
	}
	break;

	case FMRX_EVENT_SET_BAND: {
		rc = fmrx_set_band_info(p_msg->params.p_set_freq_band);
	}
	break;

	case FMRX_EVENT_SET_SNC: {
		s16 snc_step = 0;

		if (NULL == p_msg->params.p_snc_cfg) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		rx_state.cfg->snc.en = p_msg->params.p_snc_cfg->en;
		rx_state.cfg->snc.lo_thr = p_msg->params.p_snc_cfg->lo_thr;
		rx_state.cfg->snc.hi_thr = p_msg->params.p_snc_cfg->hi_thr;

		/* To avoid division by zero */
		if ((rx_state.cfg->snc.hi_thr == 0) &&
		    (rx_state.cfg->snc.lo_thr == 0)) {
			snc_step = 0;
		} else {
			s16 snc_diff = (rx_state.cfg->snc.hi_thr -
				rx_state.cfg->snc.lo_thr);

			if (snc_diff != 0)
				snc_step = misc_cfg.blend_max / snc_diff;
		}

		fmrx_set_snc_cfg(rx_state.cfg->snc.en, snc_step,
				 RSSI_TO_INT(rx_state.cfg->snc.hi_thr),
				 RSSI_TO_INT(rx_state.cfg->snc.lo_thr));
	}
	break;

	case FMRX_EVENT_SET_SOFT_MUTE: {
		if (NULL == p_msg->params.p_set_sm_cfg) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		rx_state.cfg->sm_cfg.en = p_msg->params.p_set_sm_cfg->en;
		rx_state.cfg->sm_cfg.thr = p_msg->params.p_set_sm_cfg->thr;
		rx_state.cfg->sm_cfg.step = p_msg->params.p_set_sm_cfg->step;

		/* Configure for SoftMute */
		fmrx_set_soft_mute(rx_state.cfg->sm_cfg.en,
				   rx_state.cfg->sm_cfg.step / 4,
				   RSSI_TO_INT(rx_state.cfg->sm_cfg.thr));
	}
	break;

	case FMRX_EVENT_TUNE_STATION: {
		if ((p_msg->params.ch_tune.freq > rx_state.cfg->band.max) ||
		    (p_msg->params.ch_tune.freq < rx_state.cfg->band.min)) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		fmr_receiving_exit_actions(&rx_state);
		rx_state.freq = p_msg->params.ch_tune.freq;

		fmr_evaluate_channel(&rx_state, rx_state.freq, RSSI_MIN);
		fmr_receiving_entry_actions(&rx_state);

		/* Callback for channel tuning, to the upper layer */
		if (NULL != misc_cfg.fmrx_cbs->p_ch_tuned_cb)
			misc_cfg.fmrx_cbs->p_ch_tuned_cb(
				FMRX_TUNING_DONE, rx_state.freq,
				RSSI_TO_EXT(rx_state.ch_info.rssi_eff),
				rx_state.ch_info.inj_side);
	}
	break;

	case FMRX_EVENT_ANTENNA_SELFTEST: {
		/* backup the antenna type and FM Radio frequency selections */
		enum fmtrx_ant_type ant_bkp = rx_state.cfg->antenna;
		u32 freq_bkp = rx_state.freq;
		u16 ant_tune_cap_val = 0;
		enum fmtrx_ant_selftest_res ant_self_test_result =
			FMR_ANT_SELFTEST_FAILED_FATAL;

		/* exit the receiving state */
		fmr_receiving_exit_actions(&rx_state);

		/* specify the ant type and channel freq */
		rx_state.cfg->antenna = FMR_ANT_EBD_DE;

		rx_state.freq = p_msg->params.ant_selftest_freq.freq;

		/* channel evaluation with FM Rx tuning */
		fmr_evaluate_channel(&rx_state, rx_state.freq, RSSI_MIN);

		/* read out the antenna tune cap values */
		ant_tune_cap_val = fmtrx_get_ant_tune_cap();

#if !defined AUD_APP_HOST_TEST
		/* check the cp values */
		if (ant_tune_cap_val < misc_cfg.ant_selftest_low_thr ||
		    ant_tune_cap_val > misc_cfg.ant_selftest_high_thr)
			ant_self_test_result = FMR_ANT_SELFTEST_FAILED_FATAL;
		else
			ant_self_test_result = FMR_ANT_SELFTEST_OK;
#else
		/* Proper ant_tune_cap_val values cannot be obtained on
		 * host test. Hence threshold levels check always fails
		 * on host
		 */
		ant_self_test_result = FMR_ANT_SELFTEST_OK;
#endif
		/* Restore the freq and antenna type to its previous values */
		rx_state.cfg->antenna = ant_bkp;
		rx_state.freq = freq_bkp;

		/* channel evaluation for recover */
		fmr_evaluate_channel(&rx_state, rx_state.freq, RSSI_MIN);

		/* enter the normal receiving mode */
		fmr_receiving_entry_actions(&rx_state);

		/* callback to up-layer */
		if (NULL != misc_cfg.fmrx_cbs->p_ant_selftest_done_cb)
			misc_cfg.fmrx_cbs->p_ant_selftest_done_cb(
				ant_self_test_result);
	}
	break;

	case FMRX_EVENT_SEEK_STATION: {
		struct fmtrx_msg_params msg_params = {0};

		fmr_receiving_exit_actions(&rx_state);
		rx_state.seek_mode = p_msg->params.p_seek_station->seek_mode;
		rx_state.seek_start = rx_state.freq;
		rx_state.scan_status = SCAN_ONGOING;

		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_SEEK_ACTIVE);

		/* close the clock switching feature before BSBS */
		if (BOTH_OFF != rx_state.rf_clk) {
			rx_state.rf_clk = BOTH_OFF;
			fmr_clk_switch(&rx_state);
		}

#ifdef DEBUG_FMR_HLD
		fmdrv_info("ACTIVE >> SEEK_ACTIVE\n");
#endif

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKING4NEXT;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);
	}
	break;

	case FMRX_EVENT_SEEKOREVAL_DONE: {
		fmr_evaluate_channel(&rx_state, rx_state.freq, RSSI_MIN);
		fmr_receiving_entry_actions(&rx_state);

		/* Callback for channel tuning, to the upper layer */
		if (NULL != misc_cfg.fmrx_cbs->p_ch_tuned_cb)
			misc_cfg.fmrx_cbs->p_ch_tuned_cb(
				FMRX_TUNING_DONE, rx_state.freq,
				RSSI_TO_EXT(rx_state.ch_info.rssi_eff),
				rx_state.ch_info.inj_side);

		/* Callback for tuning to a channel to the upper layer */
		if (NULL != misc_cfg.fmrx_cbs->p_ch_tuned_cb) {
			enum fmrx_tuned_state tuned = FMRX_SEEK_FAILED;

			/* check agains last state */
			if (SCAN_STOP == rx_state.scan_status) {
				tuned = FMRX_SEEK_STOP;
			} else if (SCAN_FAILURE == rx_state.scan_status) {
				tuned = FMRX_SEEK_FAILED;
			} else {
				switch (rx_state.prev_sm_state) {
				case FMRX_SM_STATE_SEEK_ACTIVE:
					tuned = FMRX_SEEK_DONE;
					break;

				default:
					tuned = FMRX_SEEK_FAILED;
					break;
				}

				rx_state.scan_status = SCAN_SUCCESS;
			}

			/* callback to up-layer */
			misc_cfg.fmrx_cbs->p_ch_tuned_cb(tuned, rx_state.freq,
				RSSI_TO_EXT(rx_state.ch_info.rssi_eff),
				rx_state.ch_info.inj_side);
		}
	}
	break;

	case FMRX_EVENT_AUTO_EVAL: {
		struct fmtrx_msg_params msg_params = {0};

		if (NULL == p_msg->params.p_ae_info) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}


		/* exit the receiving mode */
		fmr_receiving_exit_actions(&rx_state);

		/* update the paramters to rx_cfg */
		rx_state.scan_ant = p_msg->params.p_ae_info->ant_type;
		rx_state.scan_step = p_msg->params.p_ae_info->step;
		rx_state.scan_band_min = p_msg->params.p_ae_info->min;
		rx_state.scan_band_max = p_msg->params.p_ae_info->max;
		rx_state.scan_array = p_msg->params.p_ae_info->scan_report;
		rx_state.scan_max = p_msg->params.p_ae_info->maxcnt;

		fmdrv_dbg("Max no. of channels to scan: %u\n",
			  rx_state.scan_max);

		rx_state.scan_idx = 0;
		rx_state.seek_start = rx_state.freq;
		rx_state.freq = rx_state.scan_band_min;
		rx_state.scan_status = SCAN_ONGOING;

		/* Switch the antenna type if needed. */
		if (rx_state.scan_ant != rx_state.cfg->antenna) {
			enum fmtrx_ant_type temp;

			temp = rx_state.cfg->antenna;
			rx_state.cfg->antenna = rx_state.scan_ant;
			rx_state.scan_ant = temp;
			fmr_ant_switch(&rx_state);
		}

		/* go to Auto evaluate state */
		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_AUTO_EVAL);

#ifdef DEBUG_FMR_HLD
		fmdrv_info("ACTIVE >> AUTO_EVAL\n");
#endif

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_EVAL4NEXT;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);
	}
	break;

	case FMRX_EVENT_GET_CONFIG_DATA:
		fmrx_get_config(p_msg->params.p_get_cfg_data);
		break;

	case FMRX_EVENT_GET_EXT_LNA_RSSI_COMPEN: {
		enum fmtrx_ant_type ant_type =
			p_msg->params.get_ext_lna_rssi_comp.ant_type;

		if (ant_type >= FMR_ANT_TYPE_END) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		memcpy(p_msg->params.get_ext_lna_rssi_comp.lna_cfg,
		       &rx_state.cfg->ext_lna_cfg[ant_type],
		       sizeof(struct fmrx_ext_lna_cfg));
	}
	break;

	case FMRX_EVENT_AUTO_EVAL_PROGRESS: {
		p_msg->params.ch_ae_stat.scan_count = rx_state.scan_idx;
		p_msg->params.ch_ae_stat.total_count =
		((rx_state.scan_band_max - rx_state.scan_band_min) /
			rx_state.scan_step) + 1;
	}
	break;

	case FMRX_EVENT_AUTO_EVAL_GET_CAP: {
		if (NULL == p_msg->params.p_sweep_band) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		p_msg->params.p_sweep_band->max = FMRX_SWEEP_BAND_MAX_FREQ;
		p_msg->params.p_sweep_band->min = FMRX_SWEEP_BAND_MIN_FREQ;
		p_msg->params.p_sweep_band->step = FMRX_SWEEP_BAND_STEP_SIZE;
	}
	break;

	case FMRX_EVENT_AF_EVALUATE: {
		struct fmtrx_msg_params msg_params = {0};

		fmr_receiving_exit_actions(&rx_state);
		rx_state.seek_start = rx_state.freq;
		rx_state.freq = p_msg->params.af_eval_params.af_freq;
		rx_state.af_pi_code = p_msg->params.af_eval_params.pi_code;
		rx_state.cfg->force_mono = false;
		rx_state.scan_status = SCAN_SUCCESS;
		fmr_evaluate_channel(&rx_state, rx_state.freq, RSSI_MIN);

		/* Check if the measured RSSI is greater than given RSSI
		 * threshold */
		if (RSSI_TO_EXT(rx_state.ch_info.rssi_eff) >=
			(s16)(p_msg->params.af_eval_params.af_rssi_th)) {
			/* Backup RDS configuration to restore after AF
			 * evaluation */
			rx_state.rds_en_bkp = rx_state.cfg->rds_cfg.mode;
			rx_state.rds_type_bkp = misc_cfg.rds_type;
			rx_state.rds_mode_bkp = misc_cfg.rds_mode;
			rx_state.rds_pi_mode_bkp = misc_cfg.rds_pi_mode;

			rx_state.cfg->rds_cfg.mode = RDS_ON;
			misc_cfg.rds_mode = RDS_MODE_IRQ;
			misc_cfg.rds_type = FMR_RBDS;
			misc_cfg.rds_pi_mode = RDS_FAST_PI;

			/* Enable RDS in Fast pi mode and tune to the AF
			 * channel */
			fmr_receiving_entry_actions(&rx_state);
			rx_state.af_eval_state = true;
			fmr_sys_timer_schedule_cb(fast_pi_timer_duration,
						  fmrx_timer_cb);
		} else {
			/* Restore original configuration before AF evaluation
			 */
			rx_state.freq = rx_state.seek_start;
			rx_state.cfg->rds_cfg.mode = rx_state.rds_en_bkp;
			misc_cfg.rds_type = rx_state.rds_type_bkp;
			misc_cfg.rds_mode =
				rx_state.rds_mode_bkp;
			misc_cfg.rds_pi_mode = rx_state.rds_pi_mode_bkp;
			rx_state.scan_status = SCAN_FAILURE;

			/* Composing the mail */
			msg_params.fmr_mod_id = FMR_MODULE_FMRX;
			msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKOREVAL_DONE;
			memcpy(msg_params.data, &(p_msg->params),
			       sizeof(union fmrx_ev_prams));
			rc = fmr_sys_msg_send(&msg_params);

			if (NULL != misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb) {
				struct fmtrx_rssi_report scan_array;
				scan_array.freq = rx_state.freq;
				scan_array.rssi = RSSI_TO_EXT(fmrx_get_rssi());

				misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb(
					rx_state.scan_status, FMRX_EVAL_AF_RPT,
					&scan_array, 0);
			}
		}
	}
	break;

	case FMRX_EVENT_AF_MEASURE_RSSI: {
		struct fmtrx_msg_params msg_params = {0};
		if (NULL == p_msg->params.get_af_rssi.rssi_report) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		/* exit the receiving mode */
		fmr_receiving_exit_actions(&rx_state);

		/* backup the start frequency */
		rx_state.seek_start = rx_state.freq;
		rx_state.scan_array = p_msg->params.get_af_rssi.rssi_report;
		rx_state.scan_max = (u16)p_msg->params.get_af_rssi.max_count;
		rx_state.scan_idx = 0;
		rx_state.scan_status = SCAN_ONGOING;

		/* go to autoeval state */
		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_AUTO_EVAL);

#ifdef DEBUG_FMR_HLD
		fmdrv_info("STATE_ACTIVE >> STATE_AUTO_EVAL\n");
#endif

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_AF_EVAL4NEXT;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);
	}
	break;

	case FMRX_EVENT_SET_ROUTE: {
		if (FMR_AUD_PATH_OFF ==
			(enum fmrx_aud_route)p_msg->params.aud_routing) {
			fmr_mute_wait();
			fmtrx_set_idi_xfr(true, false);
			fmdrv_info("Audio routing off requested\n");
		} else {
			rx_state.cfg->aud_path = p_msg->params.aud_routing;

			/* mute audio before changing routing */
			fmr_mute_wait();
			fmrx_set_output(
				(enum fmtrx_lld_aud_routing)(
				(FMR_AUD_PATH_DAC == rx_state.cfg->aud_path) ?
				LLD_FMR_AUD_ROUTING_DAC :
				LLD_FMR_AUD_ROUTING_DSP));

			/* change the deemphasis accordingly */
			if (FMR_AUD_PATH_DAC == rx_state.cfg->aud_path) {
				if (FMR_50US == rx_state.cfg->band.deem)
					fmrx_set_deemphasis(LLD_DEEM_50us_DAC);
				else
					fmrx_set_deemphasis(LLD_DEEM_75us_DAC);
			} else {
				if (FMR_50US == rx_state.cfg->band.deem)
					fmrx_set_deemphasis(LLD_DEEM_50us_SRC);
				else
					fmrx_set_deemphasis(LLD_DEEM_75us_SRC);
			}

			/* Restore mute state */
			fmrx_audio_mute(rx_state.cfg->mute);
			fmrx_audio_processing_enable(!rx_state.cfg->mute);

			/* Callback for audio path switching done */
			if (NULL !=
				misc_cfg.fmrx_cbs->p_aud_route_changed_cb)
				misc_cfg.fmrx_cbs->p_aud_route_changed_cb(
					rx_state.cfg->aud_path);
		}
	}
	break;

	case FMRX_EVENT_SWITCH_ANT: {
		/* update only when different */
		if (rx_state.cfg->antenna != p_msg->params.ant_type) {
			/* change the antenna type in state register */
			rx_state.cfg->antenna = p_msg->params.ant_type;
			fmr_receiving_exit_actions(&rx_state);

			/* perform the antenna switching */
			fmr_ant_switch(&rx_state);

			fmr_evaluate_channel(&rx_state, rx_state.freq,
					     RSSI_MIN);
			fmr_receiving_entry_actions(&rx_state);
		}
	}
	break;

	case FMRX_EVENT_SET_AGC_GAIN: {
		fmrx_set_agc_gain(p_msg->params.p_agc_gain_cfg->en,
				  (u16)p_msg->params.p_agc_gain_cfg->gain_idx);
		rx_state.cfg->agc_cfg.en = p_msg->params.p_agc_gain_cfg->en;
		rx_state.cfg->agc_cfg.gain_idx =
			p_msg->params.p_agc_gain_cfg->gain_idx;
	}
	break;

	case FMRX_EVENT_TRANSFER_RDS_GROUPS: {
		u16 valid_groups = 0;
		struct fmtrx_lld_aud_state audio_status;

		if (NULL == p_msg->params.get_rds_grps.rds_buf_desc) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		fmrx_get_audio_status(&audio_status);

		/* Make sure that the RDS is on and channel is streo */
		if (RDS_ON == rx_state.cfg->rds_cfg.mode &&
		    audio_status.is_stereo) {
			/* Update the status of current RDS synchronization */
			misc_cfg.rds_sync = fmrx_rds_get_sync();

			rc = fmrx_rds_get_groups(
				p_msg->params.get_rds_grps.group_cnt,
				p_msg->params.get_rds_grps.rds_buf_desc, true,
				&valid_groups);
			if (rc != 0)
				valid_groups = 0;
			else
				fmdrv_dbg("Valid groups : %u\n", valid_groups);
		} else {
			valid_groups = 0;
		}


		if (NULL != p_msg->params.get_rds_grps.copied_grps)
			*(p_msg->params.get_rds_grps.copied_grps) =
				valid_groups;
		else
			rc = -EINVAL;
	}
	break;

	case FMRX_EVENT_SET_VOLUME: {
		/* Set volume according channel selected */
		switch (p_msg->params.set_ch_vol.channel) {
		case FMRX_AUD_CHN_RIGHT: {
			rx_state.cfg->vol_cfg.right =
				p_msg->params.set_ch_vol.volume;
		}
		break;

		case FMRX_AUD_CHN_LEFT: {
			rx_state.cfg->vol_cfg.left =
				p_msg->params.set_ch_vol.volume;
		}
		break;

		case FMRX_AUD_CHN_ALL: {
			rx_state.cfg->vol_cfg.right =
				p_msg->params.set_ch_vol.volume;
			rx_state.cfg->vol_cfg.left =
				p_msg->params.set_ch_vol.volume;
		}
		break;

		default: {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}
		break;
		}

		fmrx_set_aud_volume_dac(
			fmr_host_to_fmr_volume(rx_state.cfg->vol_cfg.left),
			fmr_host_to_fmr_volume(rx_state.cfg->vol_cfg.right));

		fmrx_set_aud_volume_src(
			fmr_host_to_fmr_volume(rx_state.cfg->vol_cfg.left),
			fmr_host_to_fmr_volume(rx_state.cfg->vol_cfg.right));
	}
	break;

	case FMRX_EVENT_GET_VOLUME: {
		if (NULL != p_msg->params.get_ch_vol.p_volume) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}


		/* Set volume according channel selected */
		switch (p_msg->params.set_ch_vol.channel) {
		case FMRX_AUD_CHN_RIGHT: {
			*(p_msg->params.get_ch_vol.p_volume) =
				rx_state.cfg->vol_cfg.right;
		}
		break;

		case FMRX_AUD_CHN_LEFT: {
			*(p_msg->params.get_ch_vol.p_volume) =
				rx_state.cfg->vol_cfg.left;
		}
		break;

		case FMRX_AUD_CHN_ALL: {
			*(p_msg->params.get_ch_vol.p_volume) =
				rx_state.cfg->vol_cfg.left;
		}
		break;

		default: {
#ifdef DEBUG_FMR_HLD
		fmdrv_info("%s: Incorrect ch type. Cmd has been ignored!\n",
			   __func__);
#endif
		}
		break;
		}
	}
	break;

	case FMRX_EVENT_SET_MUTE: {
		if (p_msg->params.set_mute.mute) {
			if (!rx_state.cfg->mute)
				fmr_mute_wait();
		} else {
			if (rx_state.cfg->mute) {
				/* restore volume */
				fmrx_audio_mute(false);
				fmrx_audio_processing_enable(true);
			}
		}

		rx_state.cfg->mute = p_msg->params.set_mute.mute;
	}
	break;

	case FMRX_EVENT_SET_FORCE_MONO: {
		rx_state.cfg->force_mono = p_msg->params.force_mono.mono;

		if (rx_state.cfg->force_mono)
			fmrx_force_mono(true);
		else
			fmrx_force_mono(false);
	}
	break;

	case FMRX_EVENT_SET_RDS_CONFIG: {
		if (NULL == p_msg->params.p_set_rds_cfg) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		/* Disable the RDS event */
		fmtrx_enable_interrupts(IR_RX_RDS | IR_RX_RDSSYNC,
					false, CTX_THREAD);

		/* Disable RDS processing */
		fmrx_rds_enable((enum fmtrx_lld_rds_en)RDS_OFF);

		/* Update the configuration in state first */
		misc_cfg.rds_type = p_msg->params.p_set_rds_cfg->rds_mode;
		misc_cfg.rds_min_free =
			p_msg->params.p_set_rds_cfg->min_free;
		misc_cfg.rds_sync_cfg =
			p_msg->params.p_set_rds_cfg->sync_cfg;

		/* Reactivate RDS */
		fmr_reactive_rds(&rx_state);
	}
	break;

	case FMRX_EVENT_SET_EXT_LNA_RSSI_COMPEN: {
		enum fmtrx_ant_type ant_type =
			p_msg->params.set_ext_lna_rssi_comp.ant_type;

		if (ant_type >= FMR_ANT_TYPE_END) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		rx_state.cfg->ext_lna_cfg[ant_type] =
			p_msg->params.set_ext_lna_rssi_comp.lna_cfg;

		fmrx_set_rssi_other_offs(fmr_peek_ext_lna_table(&rx_state));
	}
	break;

	case FMRX_EVENT_GET_CHANNEL_INFO: {
		rc = fmrx_get_ch_info(p_msg->params.p_ch_info);
	}
	break;

	case FMRX_EVENT_GET_RSSI: {
		s16 rssi = fmrx_get_rssi();

		if (NULL == p_msg->params.p_sweep_band) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		*(p_msg->params.p_rssi) = (s16) RSSI_TO_EXT(rssi);
	}
	break;

	case FMRX_EVENT_GET_RSSI_OFFSET: {
		if (NULL == p_msg->params.p_get_rssi_offset) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		*(p_msg->params.p_get_rssi_offset) =
		(s16) RSSI_TO_EXT(rx_state.cfg->other_cfg.rssi_off_ext);
	}
	break;

	case FMRX_EVENT_GET_REVISIONS: {
		if (NULL == p_msg->params.p_get_rev_ids) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		fmrx_get_id(&(p_msg->params.p_get_rev_ids->hw_id),
			    &(p_msg->params.p_get_rev_ids->fw_id),
			    &(p_msg->params.p_get_rev_ids->fw_timestamp),
			    &(p_msg->params.p_get_rev_ids->lld_id));

		p_msg->params.p_get_rev_ids->hld_id = FMR_HLD_ID;
		p_msg->params.p_get_rev_ids->pkg_id = FMR_SW_PKG_ID;
	}
	break;

	case FMRX_EVENT_GET_FREQ_BAND_INFO: {
		if (NULL == p_msg->params.p_get_freq_band) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		p_msg->params.p_get_freq_band->min = rx_state.cfg->band.min;
		p_msg->params.p_get_freq_band->max = rx_state.cfg->band.max;
		p_msg->params.p_get_freq_band->step =
			rx_state.seek_step;
	}
	break;

	case FMRX_EVENT_SUBSCRIBE_RSSI: {
		if (p_msg->params.rssi_subscribe.lower_thres >
		p_msg->params.rssi_subscribe.upper_thres) {
			/* Verbose mode - report whenever RSSI changes by more
			 * than one verbose step */
			/* We span two verbose steps to give hysteresis */
			misc_cfg.rssi_verbose = true;
			misc_cfg.rssi_lower = fmrx_get_rssi();
			misc_cfg.rssi_upper = misc_cfg.rssi_lower +
					RSSI_VERBOSE_STEP;

#ifdef DEBUG_FMR_HLD
			fmdrv_info("RSSI event, subscribed to verbose mode\n");
#endif
		} else {
			misc_cfg.rssi_lower =
			RSSI_TO_INT(p_msg->params.rssi_subscribe.lower_thres);
			misc_cfg.rssi_upper =
			RSSI_TO_INT(p_msg->params.rssi_subscribe.upper_thres);
			misc_cfg.rssi_verbose = false;
		}

		misc_cfg.rssi_enable = true;

		/* If not in RECEIVING state, we will set thresholds on entry*/
		fmr_setup_rssi_thresholds(&rx_state);

		/* Only enable events when we are receiving */
		fmtrx_enable_interrupts(IR_RX_RSSI, true, CTX_THREAD);
	}
	break;

	case FMRX_EVENT_UNSUBSCRIBE_RSSI: {
		misc_cfg.rssi_enable = false;
		fmtrx_enable_interrupts(IR_RX_RSSI, false, CTX_THREAD);
	}
	break;

	case FMRX_EVENT_SUBSCRIBE_RDS: {
		if (NULL == p_msg->params.rds_subscribe) {
			rc = -EINVAL;
			goto fmrx_normal_receive_err;
		}

		/* update the RDS configuration into the rx_cfg */
		rx_state.cfg->rds_cfg.mode = RDS_ON;
		misc_cfg.rds_type = p_msg->params.rds_subscribe->rds_type;
		misc_cfg.rds_mode =
			p_msg->params.rds_subscribe->rds_subsc_mode;
		misc_cfg.rds_pi_mode =
			p_msg->params.rds_subscribe->rds_pi_mode;
		misc_cfg.rds_min_free =
			(u16)p_msg->params.rds_subscribe->min_free;

		fmr_reactive_rds(&rx_state);
	}
	break;

	case FMRX_EVENT_UNSUBSCRIBE_RDS: {
		rx_state.cfg->rds_cfg.mode = RDS_OFF;
		fmrx_rds_enable(rx_state.cfg->rds_cfg.mode);

		/* If the RDS needs to be disabled, the sync notify should be
		 * disabled as  well */
		fmtrx_enable_interrupts(IR_RX_RDS | IR_RX_RDSSYNC, false,
					CTX_THREAD);
	}
	break;

	case FMRX_EVENT_SUBSCRIBE_RDS_SYNC: {
		misc_cfg.rdssync_en = true;

		if (RDS_OFF != rx_state.cfg->rds_cfg.mode)
			fmtrx_enable_interrupts(IR_RX_RDSSYNC, true,
						CTX_THREAD);
	}
	break;

	case FMRX_EVENT_UNSUBSCRIBE_RDS_SYNC: {
		misc_cfg.rdssync_en = false;
		fmtrx_enable_interrupts(IR_RX_RDSSYNC, false, CTX_THREAD);
	}
	break;

	case FMRX_EVENT_SET_SIDEBAND: {
		fmdrv_info("Current RF sb: %u\n", rx_state.cfg->rf_force_sb);

		/* To tune with a different sb, other than current ch sb */
		if (p_msg->params.rf_sb_sel.inj_side !=
			rx_state.ch_info.inj_side) {
			enum fmrx_sb_inj_side sb = rx_state.cfg->rf_force_sb;

			rx_state.cfg->rf_force_sb =
				p_msg->params.rf_sb_sel.inj_side;

			/* Stop reception smoothly */
			fmr_receiving_exit_actions(&rx_state);

			/* Jump to channel (set very low RSSI to evaluate) */
			fmr_evaluate_channel(&rx_state, rx_state.freq,
					     RSSI_MIN);

			/* Force the restart of the receiving process */
			fmr_receiving_entry_actions(&rx_state);

			fmdrv_info("Tuned with changed Inj side: %u\n",
				   rx_state.cfg->rf_force_sb);

			/* restore the sb with the default setting */
			if (FMR_SB_SEL_TMP == p_msg->params.rf_sb_sel.sb_sel)
				rx_state.cfg->rf_force_sb = sb;
		} else {
			/* When GTI wants to set SB, for the next tuning */
			if (FMR_SB_SEL_PERST == p_msg->params.rf_sb_sel.sb_sel)
				rx_state.cfg->rf_force_sb =
					p_msg->params.rf_sb_sel.inj_side;
		}
	}
	break;

	case FMRX_EVENT_DC_PILOT_CB: {
		/* Callback function for pilot change */
		if (NULL != misc_cfg.fmrx_cbs->p_pilot_found_cb)
			misc_cfg.fmrx_cbs->p_pilot_found_cb(
				(s32)p_msg->params.pilot_ev.pilot_found);
	}
	break;

	case FMRX_EVENT_DC_RSSI_CB: {
		/* Call host callback function, if enabled */
		if (NULL != misc_cfg.fmrx_cbs->p_rssi_cb) {
			if (((s32)p_msg->params.rssi_ev.last_rssi_enable)) {
				misc_cfg.fmrx_cbs->p_rssi_cb(
				(s16)p_msg->params.rssi_ev.rssi,
				!rx_state.pilot_found ||
				rx_state.cfg->force_mono);
			}
		}
	}
	break;

	case FMRX_EVENT_DC_RDSSYNC_CB: {
		/* Call host callback funciton, if enabled */
		if (NULL != misc_cfg.fmrx_cbs->p_rds_synced_cb) {
			misc_cfg.fmrx_cbs->p_rds_synced_cb(
				(s32)p_msg->params.rds_sync_ev.rds_sync);
		}
	}
	break;

	case FMRX_EVENT_DC_RDS_FASTPI_CB: {
		if ((NULL != misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb) &&
		    rx_state.af_eval_state) {
			/* Current frequency and RSSI value */
			struct fmtrx_rssi_report scan_array;
			scan_array.freq = rx_state.freq;
			scan_array.rssi = RSSI_TO_EXT(fmrx_get_rssi());
			rx_state.scan_status = (rx_state.af_pi_code !=
				(u16)p_msg->params.rds_fastpi_ev.pi) ?
				SCAN_FAILURE : SCAN_SUCCESS;

			/* Restore orig configuration before AF evaluation */
			rx_state.cfg->rds_cfg.mode = rx_state.rds_en_bkp;
			misc_cfg.rds_type = rx_state.rds_type_bkp;
			misc_cfg.rds_mode =
				rx_state.rds_mode_bkp;
			misc_cfg.rds_pi_mode = rx_state.rds_pi_mode_bkp;

			if (SCAN_SUCCESS == rx_state.scan_status) {
				/* Enable/Disable RDS back again in original
				 *mode */
				fmr_reactive_rds(&rx_state);
			} else {
				/* Re-tune to the original frequency */
				fmr_receiving_exit_actions(&rx_state);
				rx_state.freq = rx_state.seek_start;
				fmr_evaluate_channel(&rx_state, rx_state.freq,
						     RSSI_MIN);
				fmr_receiving_entry_actions(&rx_state);
			}

			/* Invoke callback to notify the rc of AF evaluate */
			misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb(
				rx_state.scan_status,
				FMRX_EVAL_AF_RPT,
				&scan_array,
				(u16)p_msg->params.rds_fastpi_ev.pi);
			rx_state.af_eval_state = false;
		}

		/* Call host callback funciton, if enabled */
		if (NULL != misc_cfg.fmrx_cbs->p_rds_fastpi_cb) {
			misc_cfg.fmrx_cbs->p_rds_fastpi_cb(
				(u16)p_msg->params.rds_fastpi_ev.pi,
				(u8)p_msg->params.rds_fastpi_ev.errs);
		}
	}
	break;

	case FMRX_EVENT_DC_RDS_CB: {
		/* Call host callback funciton, if enabled */
		if (NULL != misc_cfg.fmrx_cbs->p_rds_cb)
			misc_cfg.fmrx_cbs->p_rds_cb(
			(u16)p_msg->params.fmrx_rds_ev_info.valid_grps,
			(u16)p_msg->params.fmrx_rds_ev_info.discard_grps);
	}
	break;


	case FMRX_EVENT_GET_RX_STATE: {
		if (NULL != p_msg->params.p_state_data)
			memcpy(p_msg->params.p_state_data, rx_state.cfg,
			       sizeof(struct fmrx_cfg));
		else
			rc = -EINVAL;
	}
	break;

	case FMRX_EVENT_TEST_DUMP_FW_REG: {
		if (NULL != p_msg->params.p_dump_regs) {
			p_msg->params.p_dump_regs->high_16 =
				fmtrx_read16(FMR_RXMAIN_EVENT_EN_ADDR);
			p_msg->params.p_dump_regs->low_16 =
				fmtrx_read16(FMR_RXMAIN_EVENT_STATUS_ADDR);
		} else {
			rc = -EINVAL;
		}
	}
	break;

	case FMRX_EVENT_SET_IDI_HANDSHAKE: {
		rx_state.idi_hs_ctrl =
			p_msg->params.set_idi_hs_flag.enable;
		fmtrx_set_idi_xfr(true, rx_state.idi_hs_ctrl);

		if (rx_state.idi_hs_ctrl)
			fmdrv_info("Handshake bit is enabled\n");
		else
			fmdrv_info("Handshake bit is disabled\n");
	}
	break;

	case FMRX_EVENT_REG_READ: {
		rc = fmtrx_read_reg(p_msg->params.p_read_reg);
	}
	break;

	case FMRX_EVENT_REG_WRITE: {
		rc = fmtrx_write_reg(p_msg->params.p_write_reg);
	}
	break;

	default: {
#ifdef DEBUG_FMR_HLD
		fmdrv_info("%s: Incorrect cfg. Cmd : %u, has been ignored!\n",
			   __func__, p_msg->event);
#endif
		rc = -EPERM;
	}
	break;
	}

fmrx_normal_receive_err:
	return rc;
}

static int fmrx_seek_next_channel(struct fmrx_msgbox_buff *p_msg)
{
	s32 ch_seek_done = false;
	s16 act_sk_rssi = 0;
	int rc = 0;
	struct fmtrx_msg_params msg_params = {0};
	struct fmtrx_ch_search_cmd ch_search_cmd = {0};
	enum fmrx_ch_search_status ch_search_status =
		FMRX_CH_SEARCH_STAT_INVALID;
	u32 clk_swt_min = CLK_SWT_MIN(rx_state.cfg->other_cfg.clk_swt_rng);
	u32 clk_swt_max = CLK_SWT_MAX(rx_state.cfg->other_cfg.clk_swt_rng);

	if (NULL == p_msg) {
		rc = -EINVAL;
		goto seek_next_ch_err;
	}

	if (FMRX_SEEK_UP == rx_state.seek_mode ||
	    FMRX_SEEK_UP_TO_LIMIT == rx_state.seek_mode) {
		/* increase the frequency by one step */
		rx_state.freq += rx_state.seek_step;
		ch_search_cmd.ch_step = HZ_TO_KHZ(rx_state.seek_step);

		/* if the highest frequency in the band limit is hit */
		if (rx_state.freq > rx_state.cfg->band.max) {
			if (FMRX_SEEK_UP_TO_LIMIT == rx_state.seek_mode) {
				/* exit the seeking, and stop at the
				 * high band limit */
				rx_state.freq = rx_state.cfg->band.max;

				rx_state.prev_sm_state = fmrx_get_sm_state();

				fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);
				rx_state.scan_status = SCAN_FAILURE;

				/* Composing the mail */
				msg_params.fmr_mod_id = FMR_MODULE_FMRX;
				msg_params.fmr_mod_msg_id =
					FMRX_EVENT_SEEKOREVAL_DONE;
				memcpy(msg_params.data, &(p_msg->params),
				       sizeof(union fmrx_ev_prams));

				rc = fmr_sys_msg_send(&msg_params);

				ch_seek_done = true;
			} else
				/* wrap around to low band limit */
				rx_state.freq = rx_state.cfg->band.min;
		}

#if defined AUD_APP_HOST_TEST
		/* On Host perform do a dummy check to avoid the test
		 * case waiting for a longer timer
		 */
		if (FMRX_SEEK_UP_TO_LIMIT == rx_state.seek_mode)
			rx_state.freq = rx_state.cfg->band.max;

		if (FMRX_SEEK_UP == rx_state.seek_mode)
			rx_state.freq = rx_state.seek_start;
#endif

		if (FMRX_SEEK_UP == rx_state.seek_mode &&
		    rx_state.freq == rx_state.seek_start) {
			rx_state.prev_sm_state = fmrx_get_sm_state();
			fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);
			rx_state.scan_status = SCAN_FAILURE;

			/* Composing the mail */
			msg_params.fmr_mod_id = FMR_MODULE_FMRX;
			msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKOREVAL_DONE;

			memcpy(msg_params.data, &(p_msg->params),
			       sizeof(union fmrx_ev_prams));
			rc = fmr_sys_msg_send(&msg_params);

			ch_seek_done = true;
		}
	} else {
		/* decrease the frequency by one step */
		rx_state.freq -= rx_state.seek_step;
		ch_search_cmd.ch_step =
			-(HZ_TO_KHZ(rx_state.seek_step));

		/* hit the low band limit */
		if (rx_state.freq < rx_state.cfg->band.min) {
			if (FMRX_SEEK_DOWN_TO_LIMIT == rx_state.seek_mode) {
				/* exit the seeking, and stop at low
				 * band limit */
				rx_state.freq = rx_state.cfg->band.min;

				rx_state.prev_sm_state = fmrx_get_sm_state();
				fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);

#ifdef DEBUG_FMR_HLD
				fmdrv_info("SEEK_ACTIVE >> ACTIVE\n");
#endif

				/* Scan failed, no valid found */
				rx_state.scan_status = SCAN_FAILURE;

				/* Composing the mail */
				msg_params.fmr_mod_id = FMR_MODULE_FMRX;
				msg_params.fmr_mod_msg_id =
					FMRX_EVENT_SEEKOREVAL_DONE;

				memcpy(msg_params.data, &(p_msg->params),
				       sizeof(union fmrx_ev_prams));

				rc = fmr_sys_msg_send(&msg_params);

				ch_seek_done = true;
			} else
				/* wrap around to high band limit */
				rx_state.freq = rx_state.cfg->band.max;
		}

#if defined AUD_APP_HOST_TEST
		/* On Host perform do a dummy check to avoid the test
		 * case waiting for a longer timer*/
		if (FMRX_SEEK_DOWN_TO_LIMIT == rx_state.seek_mode)
			rx_state.freq = rx_state.cfg->band.min;

		if (FMRX_SEEK_DOWN == rx_state.seek_mode)
			rx_state.freq = rx_state.seek_start;
#endif

		if (FMRX_SEEK_DOWN == rx_state.seek_mode &&
		    rx_state.freq == rx_state.seek_start) {
			rx_state.prev_sm_state = fmrx_get_sm_state();
			fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);

			/* Scan failed, no valid found */
			rx_state.scan_status = SCAN_FAILURE;

			/* Composing the mail */
			msg_params.fmr_mod_id = FMR_MODULE_FMRX;
			msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKOREVAL_DONE;
			memcpy(msg_params.data, &(p_msg->params),
			       sizeof(union fmrx_ev_prams));
			rc = fmr_sys_msg_send(&msg_params);

			ch_seek_done = true;

#ifdef DEBUG_FMR_HLD
			fmdrv_info("SEEK_ACTIVE >> ACTIVE\n");
#endif
		}
	}

	if (ch_seek_done) {
		rc = 0;
		goto seek_next_ch_err;
	}

	switch (rx_state.seek_mode) {
	case FMRX_SEEK_UP:
		ch_search_cmd.ch_stop_freq =
		HZ_TO_KHZ((rx_state.seek_start - rx_state.seek_step));
		break;

	case FMRX_SEEK_UP_TO_LIMIT:
		ch_search_cmd.ch_stop_freq =
			HZ_TO_KHZ(rx_state.cfg->band.max);
		break;

	case FMRX_SEEK_DOWN:
		ch_search_cmd.ch_stop_freq =
		HZ_TO_KHZ((rx_state.seek_start + rx_state.seek_step));
		break;

	case FMRX_SEEK_DOWN_TO_LIMIT:
		ch_search_cmd.ch_stop_freq =
			HZ_TO_KHZ(rx_state.cfg->band.min);
		break;

	default:
		break;
	}

	ch_search_cmd.ch_start_freq = HZ_TO_KHZ(rx_state.freq);
	ch_search_cmd.pn_thres = rx_state.cfg->other_cfg.pn_thr;

	/* get the actual seeking rssi here */
	act_sk_rssi = ((rx_state.freq > clk_swt_min) &&
			(rx_state.freq < clk_swt_max)) ?
			rx_state.cfg->seek_rssi : rx_state.cfg->seek_rssi;
	ch_search_cmd.rssi_thres = RSSI_TO_INT(act_sk_rssi);
	ch_search_cmd.force_meas = 0;

	fmr_send_ch_search_cmd(&rx_state, &ch_search_cmd);

	/* now check for the channel search status */
	ch_search_status = fmr_check_ch_search_status(&rx_state);

	if (FMRX_CH_SEARCH_STAT_OK == ch_search_status) {
		fmdrv_info("Channel frequency: %u\n", rx_state.freq);
		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);

#ifdef DEBUG_FMR_HLD
		fmdrv_info("SEEK_ACTIVE >> ACTIVE\n");
#endif

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKOREVAL_DONE;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);
	} else {
		/* fall back to the frequency, to which it was,
		 * before starting the channel search
		 */
		rx_state.freq = rx_state.seek_start;

		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);

#ifdef DEBUG_FMR_HLD
		fmdrv_info("SEEK_ACTIVE >> ACTIVE\n");
#endif

		/* Scan failed, no valid channel found */
		rx_state.scan_status = SCAN_FAILURE;

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKOREVAL_DONE;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);
	}

seek_next_ch_err:
	return rc;
}

/* Event handler when FM Rx is in seek state */
static int fmrx_seek(struct fmrx_msgbox_buff *p_msg)
{
	int rc = 0;

	if (NULL == p_msg) {
		rc = -EINVAL;
		goto fmrx_seek_err;
	}

	switch (p_msg->event) {
	case FMRX_EVENT_POWER_OFF: {
		if (BOTH_OFF != rx_state.rf_clk) {
			rx_state.rf_clk = BOTH_OFF;
			fmr_clk_switch(&rx_state);
		}

		fmr_stop_receiver(&rx_state);

		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_IDLE);

#ifdef DEBUG_FMR_HLD
		fmdrv_info("SEEK_ACTIVE >> IDLE\n");
#endif

		/* Callback for PowerOff to the upper layer */
		if (NULL != misc_cfg.fmrx_cbs->p_pow_onoff_cb)
			misc_cfg.fmrx_cbs->p_pow_onoff_cb(FMRX_SWITCHED_OFF);
	}
	break;

	case FMRX_EVENT_GET_CONFIG_DATA:
		fmrx_get_config(p_msg->params.p_get_cfg_data);
		break;

	case FMRX_EVENT_GET_FREQ_BAND_INFO: {
		if (NULL == p_msg->params.p_get_freq_band) {
			rc = -EINVAL;
			goto fmrx_seek_err;
		}

		p_msg->params.p_get_freq_band->min = rx_state.cfg->band.min;
		p_msg->params.p_get_freq_band->max = rx_state.cfg->band.max;
		p_msg->params.p_get_freq_band->step = rx_state.seek_step;
	}
	break;

	case FMRX_EVENT_GET_EXT_LNA_RSSI_COMPEN: {
		enum fmtrx_ant_type ant_type =
			p_msg->params.get_ext_lna_rssi_comp.ant_type;

		if (ant_type >= FMR_ANT_TYPE_END) {
			rc = -EINVAL;
			goto fmrx_seek_err;
		}

		memcpy(p_msg->params.get_ext_lna_rssi_comp.lna_cfg,
		       &rx_state.cfg->ext_lna_cfg[ant_type],
		       sizeof(struct fmrx_ext_lna_cfg));
	}
	break;

	case FMRX_EVENT_GET_CHANNEL_INFO:
		rc = fmrx_get_ch_info(p_msg->params.p_ch_info);
		break;

	case FMRX_EVENT_TUNE_STATION: {
		struct fmtrx_msg_params msg_params;

		/* update the parameters to rx_cfg */
		rx_state.freq = p_msg->params.ch_tune.freq;

		/* Set the FW to idle state */
		fmrx_set_fw_state(FMRX_SM_STATE_IDLE);

		/* Terminate the autoseeking mode, switch back to
		 * normal receiving mode */
		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);
		rx_state.scan_status = SCAN_STOP;

#ifdef DEBUG_FMR_HLD
		fmdrv_info("SEEK_ACTIVE >> ACTIVE\n");
#endif

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKOREVAL_DONE;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);
	}
	break;

	case FMRX_EVENT_SEEK_STATION: {
		struct fmtrx_msg_params msg_params = {0};

		/* new request has no influence on the same seeking mode */
		if (rx_state.seek_mode ==
			p_msg->params.p_seek_station->seek_mode)
			break;

		/* otherwise, update the seeking direction and other param */
		rx_state.seek_mode = p_msg->params.p_seek_station->seek_mode;
		rx_state.seek_start = rx_state.freq;

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKING4NEXT;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);
	}
	break;

	case FMRX_EVENT_SEEKING4NEXT: {
		fmrx_seek_next_channel(p_msg);
	}
	break;

	case FMRX_EVENT_SWITCH_ANT: {
		/* update only when different */
		if (rx_state.cfg->antenna != p_msg->params.ant_type) {
			/* change the antenna type in state register */
			rx_state.cfg->antenna = p_msg->params.ant_type;

			/* perform the antenna switching */
			fmr_ant_switch(&rx_state);

			/* terminate the seeking */
			rx_state.freq = rx_state.seek_start;
			fmr_evaluate_channel(&rx_state, rx_state.freq,
					     RSSI_MIN);
			fmr_receiving_entry_actions(&rx_state);

			rx_state.prev_sm_state = fmrx_get_sm_state();
			fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);

#ifdef DEBUG_FMR_HLD
			fmdrv_info("SEEK_ACTIVE >> ACTIVE\n");
#endif
		}
	}
	break;

	case FMRX_EVENT_SET_AGC_GAIN: {
		fmrx_set_agc_gain(p_msg->params.p_agc_gain_cfg->en,
				  (u16)p_msg->params.p_agc_gain_cfg->gain_idx);
		rx_state.cfg->agc_cfg.en = p_msg->params.p_agc_gain_cfg->en;
		rx_state.cfg->agc_cfg.gain_idx =
			p_msg->params.p_agc_gain_cfg->gain_idx;
	}
	break;

	case FMRX_EVENT_GET_RX_STATE: {
		if (NULL != p_msg->params.p_state_data)
			memcpy(p_msg->params.p_state_data, rx_state.cfg,
			       sizeof(struct fmrx_cfg));
		else
			rc = -EINVAL;
	}
	break;

	case FMRX_EVENT_TEST_DUMP_FW_REG: {
		if (NULL == p_msg->params.p_dump_regs) {
			rc = -EINVAL;
			goto fmrx_seek_err;
		}

		p_msg->params.p_dump_regs->high_16 =
			fmtrx_read16(FMR_RXMAIN_EVENT_EN_ADDR);
		p_msg->params.p_dump_regs->low_16 =
			fmtrx_read16(FMR_RXMAIN_EVENT_STATUS_ADDR);
	}
	break;

	case FMRX_EVENT_REG_READ: {
		rc = fmtrx_read_reg(p_msg->params.p_read_reg);
	}
	break;

	case FMRX_EVENT_REG_WRITE: {
		rc = fmtrx_write_reg(p_msg->params.p_write_reg);
	}
	break;

	case FMRX_EVENT_SET_VOLUME: {
	/* Set volume according channel selected */
	switch (p_msg->params.set_ch_vol.channel) {
		case FMRX_AUD_CHN_RIGHT: {
			rx_state.cfg->vol_cfg.right =
				p_msg->params.set_ch_vol.volume;
		}
		break;

		case FMRX_AUD_CHN_LEFT: {
			rx_state.cfg->vol_cfg.left =
				p_msg->params.set_ch_vol.volume;
		}
		break;

		case FMRX_AUD_CHN_ALL: {
			rx_state.cfg->vol_cfg.right =
				p_msg->params.set_ch_vol.volume;
			rx_state.cfg->vol_cfg.left =
				p_msg->params.set_ch_vol.volume;
		}
		break;

		default: {
			rc = -EINVAL;
			goto fmrx_seek_err;
		}
		break;
		}

		fmrx_set_aud_volume_dac(
			fmr_host_to_fmr_volume(rx_state.cfg->vol_cfg.left),
			fmr_host_to_fmr_volume(rx_state.cfg->vol_cfg.right));

		fmrx_set_aud_volume_src(
			fmr_host_to_fmr_volume(rx_state.cfg->vol_cfg.left),
			fmr_host_to_fmr_volume(rx_state.cfg->vol_cfg.right));
		}
		break;

	case FMRX_EVENT_SET_ROUTE: {
		if (FMR_AUD_PATH_OFF ==
			(enum fmrx_aud_route)p_msg->params.aud_routing) {
			fmr_mute_wait();
			fmtrx_set_idi_xfr(true, false);
			fmdrv_info("Audio routing off requested\n");
		} else {
			rx_state.cfg->aud_path = p_msg->params.aud_routing;

			/* mute audio before changing routing */
			fmr_mute_wait();
			fmrx_set_output(
				(enum fmtrx_lld_aud_routing)(
				(FMR_AUD_PATH_DAC == rx_state.cfg->aud_path) ?
				LLD_FMR_AUD_ROUTING_DAC :
				LLD_FMR_AUD_ROUTING_DSP));

			/* change the deemphasis accordingly */
			if (FMR_AUD_PATH_DAC == rx_state.cfg->aud_path) {
				if (FMR_50US == rx_state.cfg->band.deem)
					fmrx_set_deemphasis(LLD_DEEM_50us_DAC);
				else
					fmrx_set_deemphasis(LLD_DEEM_75us_DAC);
			} else {
				if (FMR_50US == rx_state.cfg->band.deem)
					fmrx_set_deemphasis(LLD_DEEM_50us_SRC);
				else
					fmrx_set_deemphasis(LLD_DEEM_75us_SRC);
			}

			/* Restore mute state */
			fmrx_audio_mute(rx_state.cfg->mute);
			fmrx_audio_processing_enable(!rx_state.cfg->mute);

			/* Callback for audio path switching done */
			if (NULL !=
				misc_cfg.fmrx_cbs->p_aud_route_changed_cb)
				misc_cfg.fmrx_cbs->p_aud_route_changed_cb(
					rx_state.cfg->aud_path);
		}
	}
	break;

	case FMRX_EVENT_SET_IDI_HANDSHAKE: {
		rx_state.idi_hs_ctrl =
			p_msg->params.set_idi_hs_flag.enable;
		fmtrx_set_idi_xfr(true, rx_state.idi_hs_ctrl);

		if (rx_state.idi_hs_ctrl)
			fmdrv_info("Handshake bit is enabled\n");
		else
			fmdrv_info("Handshake bit is disabled\n");
	}
	break;

	default: {
#ifdef DEBUG_FMR_HLD
		fmdrv_info("%s: Incorrect cfg. Cmd : %u, has been ignored!\n",
			   __func__, p_msg->event);
#endif
		rc = -EPERM;
	}
	break;
	}

fmrx_seek_err:
	return rc;
}

static int fmrx_ae_next_ch(struct fmrx_msgbox_buff *p_msg)
{
	struct fmtrx_msg_params msg_params = {0};
	int rc = 0;

	if (rx_state.scan_idx < rx_state.scan_max &&
	    rx_state.freq <= rx_state.scan_band_max &&
	    rx_state.freq >= rx_state.scan_band_min) {
		/* evaluate the channel and get the reported RSSI by
		 * oneshot measurement */
		fmr_evaluate_channel(&rx_state, rx_state.freq, RSSI_MIN);

		/* recorde the rc to the report array */
		rx_state.scan_array[rx_state.scan_idx].freq = rx_state.freq;
		rx_state.scan_array[rx_state.scan_idx].rssi =
			RSSI_TO_EXT(fmrx_get_rssi());
		rx_state.scan_idx++;

		/* increase the freq per the scan stepsize, from min to max
		 * always */
		rx_state.freq += rx_state.scan_step;

		/* Composing the message to send it for evalutating the
		 * next one */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_EVAL4NEXT;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);
	} else {
		u16 ch_count;

		/* finished the autoeval return to normal receiving mode */
		rx_state.freq = rx_state.seek_start;

		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);

#ifdef DEBUG_FMR_HLD
		fmdrv_info("AUTO_EVAL >> ACTIVE\n");
#endif

		/* mark all the unused items in report array as
		 * invalid */
		ch_count = rx_state.scan_idx;

		while (rx_state.scan_idx < rx_state.scan_max) {
			rx_state.scan_array[rx_state.scan_idx].freq =
				rx_state.cfg->band.min;
			rx_state.scan_array[rx_state.scan_idx].rssi =
				RSSI_MIN;
			rx_state.scan_idx++;
		}

		/*Restoring scan_idx to its last index so as to report
		 * correct progress */
		rx_state.scan_idx = ch_count;
		rx_state.scan_status = SCAN_SUCCESS;

		/* Recover the antenna type if needed. */
		if (rx_state.scan_ant != rx_state.cfg->antenna) {
			rx_state.cfg->antenna = rx_state.scan_ant;
			fmr_ant_switch(&rx_state);
		}

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKOREVAL_DONE;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);

		/* Callback of finished the autoeval */
		if (NULL != misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb) {
			misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb(
				rx_state.scan_status,	FMRX_EVAL_AUTO_RPT,
				rx_state.scan_array, ch_count);
		}
	}

	return rc;
}

static int fmrx_af_eval_next_ch(struct fmrx_msgbox_buff *p_msg)
{
	struct fmtrx_msg_params msg_params = {0};
	int rc = 0;

	if (rx_state.scan_idx < rx_state.scan_max) {
		u32 scan_freq = rx_state.scan_array[rx_state.scan_idx].freq;
		fmr_evaluate_channel(&rx_state, scan_freq, RSSI_MIN);

		rx_state.scan_array[rx_state.scan_idx].rssi =
			RSSI_TO_EXT(fmrx_get_rssi());

		rx_state.scan_idx++;

		/* send message for evalutate the next one */

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_AF_EVAL4NEXT;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);
	} else {
		rx_state.freq = rx_state.seek_start;
		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);
		rx_state.scan_status = SCAN_SUCCESS;

#ifdef DEBUG_FMR_HLD
		fmdrv_info("AUTO_EVAL >> ACTIVE\n");
#endif

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKOREVAL_DONE;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);

		/* callback of finished the AF evaluation */
		if (NULL != misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb) {
			misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb(
				rx_state.scan_status,
				FMRX_EVAL_AF_MEAS_RSSI_RPT,
				rx_state.scan_array, rx_state.scan_max);
		}
	}

	return rc;
}

/* Event handler when FM Rx is active */
static int fmrx_auto_evaluate(struct fmrx_msgbox_buff *p_msg)
{
	int rc = 0;

	if (NULL == p_msg) {
		rc = -EINVAL;
		goto fmrx_auto_evaluate_err;
	}

	switch (p_msg->event) {
	case FMRX_EVENT_POWER_OFF: {
		/* switch back the rf clock to all off */
		if (BOTH_OFF != rx_state.rf_clk) {
			rx_state.rf_clk = BOTH_OFF;
			fmr_clk_switch(&rx_state);
		}

		/* return to the start frequency and stop the receiver */
		rx_state.freq = rx_state.seek_start;
		fmr_stop_receiver(&rx_state);

		/* switch to the idle mode */
		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_IDLE);

#ifdef DEBUG_FMR_HLD
		fmdrv_info("FMRX_SM_STATE_AUTO_EVAL >> FMRX_SM_STATE_IDLE\n");
#endif

		/* Callback for PowerOff to the upper layer */
		if (NULL != misc_cfg.fmrx_cbs->p_pow_onoff_cb)
			misc_cfg.fmrx_cbs->p_pow_onoff_cb(FMRX_SWITCHED_OFF);
	}
	break;

	case FMRX_EVENT_GET_CONFIG_DATA:
		fmrx_get_config(p_msg->params.p_get_cfg_data);
		break;

	case FMRX_EVENT_GET_EXT_LNA_RSSI_COMPEN: {
		enum fmtrx_ant_type ant_type =
			p_msg->params.get_ext_lna_rssi_comp.ant_type;

		if (ant_type >= FMR_ANT_TYPE_END) {
			rc = -EINVAL;
			goto fmrx_auto_evaluate_err;
		}

		memcpy(p_msg->params.get_ext_lna_rssi_comp.lna_cfg,
		       &rx_state.cfg->ext_lna_cfg[ant_type],
		       sizeof(struct fmrx_ext_lna_cfg));
	}
	break;

	case FMRX_EVENT_GET_FREQ_BAND_INFO: {
		if (NULL == p_msg->params.p_get_freq_band) {
			rc = -EINVAL;
			goto fmrx_auto_evaluate_err;
		}

		p_msg->params.p_get_freq_band->min = rx_state.cfg->band.min;
		p_msg->params.p_get_freq_band->max = rx_state.cfg->band.max;
		p_msg->params.p_get_freq_band->step = rx_state.seek_step;
	}
	break;

	case FMRX_EVENT_GET_CHANNEL_INFO: {
		rc = fmrx_get_ch_info(p_msg->params.p_ch_info);
	}
	break;

	case FMRX_EVENT_TUNE_STATION: {
		struct fmtrx_msg_params msg_params = {0};

		if ((p_msg->params.ch_tune.freq > rx_state.cfg->band.max) ||
		    (p_msg->params.ch_tune.freq < rx_state.cfg->band.min)) {
			rc = -EINVAL;
			goto fmrx_auto_evaluate_err;
		}

		/* update the parameters to rx_cfg */
		rx_state.freq = p_msg->params.ch_tune.freq;

		/* switch back to normal receiving mode */
		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);

		rx_state.scan_status = SCAN_STOP;

#ifdef DEBUG_FMR_HLD
		fmdrv_info("AUTO_EVAL >> ACTIVE\n");
#endif

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKOREVAL_DONE;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);
	}
	break;

	case FMRX_EVENT_EVAL4NEXT:
		rc = fmrx_ae_next_ch(p_msg);
		break;

	case FMRX_EVENT_AF_EVAL4NEXT:
		rc = fmrx_af_eval_next_ch(p_msg);
		break;

	case FMRX_EVENT_SET_ROUTE:
		rx_state.cfg->aud_path = p_msg->params.aud_routing;
		break;

	case FMRX_EVENT_SWITCH_ANT: {
		/* update only when different */
		if (rx_state.cfg->antenna !=
			(enum fmtrx_ant_type)p_msg->params.ant_type) {
			/* change the antenna type in state register */
			rx_state.cfg->antenna =
			(enum fmtrx_ant_type)p_msg->params.ant_type;

			/* perform the antenna switching */
			fmr_ant_switch(&rx_state);

			/* terminate the auto evaluation */
			rx_state.freq = rx_state.seek_start;
			fmr_evaluate_channel(&rx_state, rx_state.freq,
					     RSSI_MIN);
			fmr_receiving_entry_actions(&rx_state);

			rx_state.prev_sm_state = fmrx_get_sm_state();
			fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);


#ifdef DEBUG_FMR_HLD
			fmdrv_info("AUTO_EVAL >> ACTIVE\n");
#endif
		}
	}
	break;

	case FMRX_EVENT_SET_AGC_GAIN: {
		fmrx_set_agc_gain(p_msg->params.p_agc_gain_cfg->en,
				  (u16)p_msg->params.p_agc_gain_cfg->gain_idx);
		rx_state.cfg->agc_cfg.en = p_msg->params.p_agc_gain_cfg->en;
		rx_state.cfg->agc_cfg.gain_idx =
			p_msg->params.p_agc_gain_cfg->gain_idx;
	}
	break;

	case FMRX_EVENT_GET_RX_STATE: {
		if (NULL != p_msg->params.p_state_data)
			memcpy(p_msg->params.p_state_data, rx_state.cfg,
			       sizeof(struct fmrx_cfg));
		else
			rc = -EINVAL;
	}
	break;

	case FMRX_EVENT_TEST_DUMP_FW_REG: {
		if (NULL != p_msg->params.p_dump_regs) {
			p_msg->params.p_dump_regs->high_16 =
				fmtrx_read16(FMR_RXMAIN_EVENT_EN_ADDR);
			p_msg->params.p_dump_regs->low_16 =
				fmtrx_read16(FMR_RXMAIN_EVENT_STATUS_ADDR);
		} else {
			rc = -EINVAL;
		}
	}
	break;

	case FMRX_EVENT_AUTO_EVAL_STOP: {
		u16 ch_count;
		struct fmtrx_msg_params msg_params = {0};

		/* Finished the autoeval return to normal receiving mode */
		rx_state.freq = rx_state.seek_start;
		rx_state.prev_sm_state = fmrx_get_sm_state();
		fmrx_set_sm_state(FMRX_SM_STATE_ACTIVE);

#ifdef DEBUG_FMR_HLD
		fmdrv_info("AUTO_EVAL >> ACTIVE\n");
#endif

		ch_count = rx_state.scan_idx;

		/* Mark all the unused items in report array as invalid */
		while (rx_state.scan_idx < rx_state.scan_max) {
			rx_state.scan_array[rx_state.scan_idx].freq =
							rx_state.cfg->band.min;
			rx_state.scan_array[rx_state.scan_idx].rssi =
				RSSI_MIN;
			rx_state.scan_idx++;
		}

		/* Restoring rx_state.scan_idx for capturing the right
		 * progress state/count */
		rx_state.scan_idx = ch_count;

		/* Updating rx_state.scan_status */
		rx_state.scan_status = SCAN_STOP;

		/* Recover the antenna type if needed. */
		if (rx_state.scan_ant != rx_state.cfg->antenna) {
			rx_state.cfg->antenna = rx_state.scan_ant;
			fmr_ant_switch(&rx_state);
		}

		/* Composing the mail */
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKOREVAL_DONE;
		memcpy(msg_params.data, &(p_msg->params),
		       sizeof(union fmrx_ev_prams));
		rc = fmr_sys_msg_send(&msg_params);

		/*Callback of stopping the autoeval */
		if (NULL != misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb) {
			misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb(
				rx_state.scan_status, FMRX_EVAL_AUTO_RPT,
				rx_state.scan_array, ch_count);
		}
	}
	break;

	case FMRX_EVENT_AUTO_EVAL_PROGRESS: {
		p_msg->params.ch_ae_stat.scan_count = rx_state.scan_idx;
		p_msg->params.ch_ae_stat.total_count =
		((rx_state.scan_band_max - rx_state.scan_band_min) /
			rx_state.scan_step) + 1;
	}
	break;

	case FMRX_EVENT_AUTO_EVAL_GET_CAP: {
		if (NULL != p_msg->params.p_sweep_band) {
			p_msg->params.p_sweep_band->max =
				FMRX_SWEEP_BAND_MAX_FREQ;
			p_msg->params.p_sweep_band->min =
				FMRX_SWEEP_BAND_MIN_FREQ;
			p_msg->params.p_sweep_band->step =
				FMRX_SWEEP_BAND_STEP_SIZE;
		}
	}
	break;

	case FMRX_EVENT_REG_READ: {
		rc = fmtrx_read_reg(p_msg->params.p_read_reg);
	}
	break;

	case FMRX_EVENT_REG_WRITE: {
		rc = fmtrx_write_reg(p_msg->params.p_write_reg);
	}
	break;

	default: {
#ifdef DEBUG_FMR_HLD
		fmdrv_info("%s: Incorrect cfg. Cmd : %u, has been ignored!\n",
			   __func__, p_msg->event);
#endif
		rc = -EPERM;
	}
	break;
	}

fmrx_auto_evaluate_err:
	return rc;
}

/* FM Radio pilot callback */
static void fmrx_pilot_cb(void)
{
	union fmrx_ev_prams rx_ev_info;
	struct fmtrx_msg_params msg_params = {0};
	struct fmtrx_lld_aud_state audio_status;

	fmrx_get_audio_status(&audio_status);

	if (rx_state.pilot_found != audio_status.is_stereo) {
		rx_state.pilot_found = audio_status.is_stereo;

		/* de-coupling the callback */
		rx_ev_info.pilot_ev.pilot_found = rx_state.pilot_found;
		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_DC_PILOT_CB;
		memcpy(msg_params.data, &rx_ev_info,
		       sizeof(union fmrx_ev_prams));
		fmr_sys_int_event_send(FMR_I_EV_RX_PILOT, msg_params);
	}
}

static void fmrx_rssi_cb(s16 rssi)
{
	union fmrx_ev_prams rx_ev_info;
	struct fmtrx_msg_params msg_params = {0};

	rx_ev_info.rssi_ev.last_rssi_enable = misc_cfg.rssi_enable;

	if (misc_cfg.rssi_enable && misc_cfg.rssi_verbose) {
		misc_cfg.rssi_lower = rssi - RSSI_VERBOSE_STEP / 2;
		misc_cfg.rssi_upper = rssi + RSSI_VERBOSE_STEP / 2;
	} else {
		/* Disable RSSI callbacks */
		fmtrx_enable_interrupts(IR_RX_RSSI, false, CTX_IRQ);
		misc_cfg.rssi_enable = false;
	}

	/* Setup thresholds according to FM Rx SM state */
	fmr_setup_rssi_thresholds(&rx_state);

	/* de-coupling the callback */
	rx_ev_info.rssi_ev.rssi = RSSI_TO_EXT(rssi);

	msg_params.fmr_mod_id = FMR_MODULE_FMRX;
	msg_params.fmr_mod_msg_id = FMRX_EVENT_DC_RSSI_CB;
	memcpy(msg_params.data, &rx_ev_info, sizeof(union fmrx_ev_prams));
	fmr_sys_int_event_send(FMR_I_EV_RX_RSSI, msg_params);
}

/* FM Radio RDS callback */
static void fmrx_rds_cb(void)
{
	u16 valid_grps = 0, discard_grps = 0;
	union fmrx_ev_prams rx_ev_info;
	struct fmtrx_msg_params msg_params  = {0};

	/* initialize event info structure to 0 */
	memset((void *)&rx_ev_info, 0, sizeof(rx_ev_info));

	/* if fastPI mode is enabled */
	if (RDS_FAST_PI == misc_cfg.rds_pi_mode) {
		struct fmrx_rds_pi_code pi = {0};
		fmrx_rds_get_pi(&pi);

		/* de-coupling the callback */
		rx_ev_info.rds_fastpi_ev.pi = pi.pi;
		rx_ev_info.rds_fastpi_ev.errs = pi.errs;

		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_DC_RDS_FASTPI_CB;
		memcpy(msg_params.data, &rx_ev_info,
		       sizeof(union fmrx_ev_prams));
		fmr_sys_int_event_send(FMR_I_EV_RX_RDS_FASTPI, msg_params);
	} else {
		/* check the number of valid groups in FW ring-buffer */
		valid_grps = fmrx_rds_get_group_count();
		discard_grps = fmrx_rds_get_discard_count();

		/* de-coupling the callback */
		rx_ev_info.fmrx_rds_ev_info.valid_grps = valid_grps;
		rx_ev_info.fmrx_rds_ev_info.discard_grps = discard_grps;

		msg_params.fmr_mod_id = FMR_MODULE_FMRX;
		msg_params.fmr_mod_msg_id = FMRX_EVENT_DC_RDS_CB;
		memcpy(msg_params.data, &rx_ev_info,
		       sizeof(union fmrx_ev_prams));
		fmr_sys_int_event_send(FMR_I_EV_RX_RDS, msg_params);
	}
}

static void fmrx_rdssync_cb(void)
{
	struct fmtrx_msg_params msg_params  = {0};
	union fmrx_ev_prams rx_ev_info;

	/* Update the RDS sync satus */
	misc_cfg.rds_sync = fmrx_rds_get_sync();

	/* de-coupling the callback */
	rx_ev_info.rds_sync_ev.rds_sync = misc_cfg.rds_sync;

	msg_params.fmr_mod_id = FMR_MODULE_FMRX;
	msg_params.fmr_mod_msg_id = FMRX_EVENT_DC_RDSSYNC_CB;
	memcpy(msg_params.data, &rx_ev_info, sizeof(union fmrx_ev_prams));
	fmr_sys_int_event_send(FMR_I_EV_RX_RDS_SYNC, msg_params);
}

static void fmrx_timer_cb(void *args)
{
	struct fmtrx_msg_params msg_params;
	struct fmtrx_rssi_report scan_array;

	if (!rx_state.af_eval_state)
		return;

	scan_array.freq = rx_state.freq;
	scan_array.rssi = RSSI_TO_EXT(fmrx_get_rssi());

	/* Restore original configuration before AF evaluation */
	rx_state.freq = rx_state.seek_start;
	rx_state.scan_status = SCAN_FAILURE;
	rx_state.af_eval_state = false;
	rx_state.cfg->rds_cfg.mode = rx_state.rds_en_bkp;
	misc_cfg.rds_type = rx_state.rds_type_bkp;
	misc_cfg.rds_mode = rx_state.rds_mode_bkp;
	misc_cfg.rds_pi_mode = rx_state.rds_pi_mode_bkp;

	if (NULL != misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb)
		misc_cfg.fmrx_cbs->p_seek_eval_rpt_cb(rx_state.scan_status,
			FMRX_EVAL_AF_RPT, &scan_array, 0);

	/* Exit from the current tuned alternate frequency */
	fmr_receiving_exit_actions(&rx_state);

	/* Composing the mail */
	msg_params.fmr_mod_id = FMR_MODULE_FMRX;
	msg_params.fmr_mod_msg_id = FMRX_EVENT_SEEKOREVAL_DONE;
	fmr_sys_msg_send(&msg_params);
}

/* FM Rx interrupt handler */
void fmrx_irq_handler(u32 int_status)
{
	s16 rssi = 0;
	s32 last_pilot_found;

	/* Save last pilot found in case of mono / stereo switch */
	last_pilot_found = rx_state.pilot_found;

	/* RDS interrupt */
	if (int_status & IR_RX_RDS)
		fmrx_rds_cb();

	/* RDS sync interrupt */
	if (int_status & IR_RX_RDSSYNC)
		fmrx_rdssync_cb();

	/* Pilot interrupt */
	if (int_status & IR_RX_PILOT)
		fmrx_pilot_cb();

	/* Trigger RSSI callback if RSSI event, or if stereo / mono state has
	changed */
	if ((int_status & IR_RX_RSSI) ||
	    (last_pilot_found != rx_state.pilot_found)) {
		rssi = fmrx_get_rssi();
		fmrx_rssi_cb(rssi);
	}
}

/* Dispatcher for the FM Rx module */
int fmrx_event_dispatcher(struct fmrx_msgbox_buff *p_msg)
{
	int rc = 0;

	if (NULL == p_msg) {
		rc = -EINVAL;
		goto fmrx_event_dispatcher_err;
	}

	switch (fmrx_get_sm_state()) {
	/*
	 * FM radio disabled (powered off). Configuration cmds (e.g. setting
	 * frequency, band limits, etc. can be performed and will take effect
	 * at power-on.
	 */
	case FMRX_SM_STATE_IDLE:
		rc = fmrx_idle(p_msg);
		break;

	/*
	 * Audio either turned on (LISTENING) or turned off (MUTED) depending
	 * FM Rx SM state variable. RSSI events generated as requested; host
	 * responsibility to change from MUTED to LISTENING if required
	 * e.g. if RSSI increases.
	 * Note: recommended to perform explicit retune to current channel in
	 * this case to avoid unmuting due to image channel).
	 *
	 * RDS events enabled if stereo signal found and RDS enabled. Stereo
	 * enabled if stereo signal found and mono not selected.
	 *
	 * Before leaving this state, the audio is always muted.
	 *
	 * A periodic timer function is enabled to check calibration.
	 */
	case FMRX_SM_STATE_ACTIVE:
		rc = fmrx_normal_receive(p_msg);
		break;

	/*
	 * Scanning through frequencies looking for the requested RSSI level;
	 * updates the frequency state variable. On reaching the upper or lower
	 * band limit, searching resumes at the other band limit. If the
	 * specified RSSI threshold is met, or if the frequency returns to
	 * beyond the starting frequency, then the receiver returns to
	 * RECEIVING state. On re-entering RECEIVING state, an RSSI event is
	 * nerated corresponding to the current signal strength. Calibration
	 * ways performed before returning to RECEIVING state.
	 */
	case FMRX_SM_STATE_SEEK_ACTIVE:
		rc = fmrx_seek(p_msg);
		break;

	/*
	 * Scanning through a list of specified frequencies to determine RSSI,
	 * returning as quickly as possible to the current frequency (state
	 * variable) with minimum disruption. RSSI event for the current
	 * frequency is generated on returning to the RECEIVING state.
	 */
	case FMRX_SM_STATE_AUTO_EVAL:
		rc = fmrx_auto_evaluate(p_msg);
		break;

	default:
		fmdrv_err("Invalid FM Rx SM state\n");
		rc = -EPERM;
		break;
	}

fmrx_event_dispatcher_err:
	return rc;
}

/* Initialisation function for the FM Rx module */
int fmrx_init(enum fmtrx_init_mode fmrx_op_mode)
{
	int rc = 0;

	if (fmrx_op_mode >= FMTRX_INIT_MODE_INVALID &&
	    fmrx_op_mode < FMTRX_INIT_MODE_ON) {
		rc = -EINVAL;
		goto fmrx_init_err;
	}

	switch (fmrx_op_mode) {
	case FMTRX_INIT_MODE_ON:
		rc = fmrx_initialise_state();
		break;

	case FMTRX_INIT_MODE_OFF:
		if (FMRX_SM_STATE_INVALID != fmrx_get_sm_state()) {
			/* Terminate FM Rx action smoothly */
			if (FMRX_SM_STATE_ACTIVE == fmrx_get_sm_state()) {
				fmr_receiving_exit_actions(&rx_state);
				fmr_stop_receiver(&rx_state);
			}

			fmrx_set_sm_state(FMRX_SM_STATE_INVALID);
			rx_state.prev_sm_state = FMRX_SM_STATE_INVALID;
		}
		break;

	default:
		rc = -EINVAL;
		break;
	}

fmrx_init_err:
	return rc;
}

/* Inquire the FM Rx HW state */
enum fmrx_hw_state fmrx_get_hw_state(void)
{
	enum fmrx_hw_state fmrx_hw_status = FMRX_HW_STATE_INVALID;
	enum fmrx_sm_state fmrx_sm_status = fmrx_get_sm_state();

	switch (fmrx_sm_status) {
	case FMRX_SM_STATE_IDLE:
		fmrx_hw_status = FMRX_HW_STATE_IDLE;
		break;

	/* Intended fall through */
	case FMRX_SM_STATE_ACTIVE:
	case FMRX_SM_STATE_RDS_ACTIVE:
		fmrx_hw_status = FMRX_HW_STATE_ACTIVE;
		break;

	/* Intended fall through */
	case FMRX_SM_STATE_SEEK_ACTIVE:
	case FMRX_SM_STATE_AUTO_EVAL:
		fmrx_hw_status = FMRX_HW_STATE_SEEK_ACTIVE;
		break;

	default:
		fmrx_hw_status = FMRX_HW_STATE_INVALID;
		break;
	}

	return fmrx_hw_status;
}

/* End of file */
