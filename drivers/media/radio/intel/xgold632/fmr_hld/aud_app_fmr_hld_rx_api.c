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
/* This file contains the HLD API for internal FM Rx */

/* INCLUDE STATEMENTS */
#include <linux/err.h>
#include <aud_app_fmr_hld.h>
#include <aud_app_fmr_hld_rx.h>
#include <aud_app_fmr_sys.h>

/* Exported FM Rx functionality */

/* Power ON FM Radio */
int fmrx_power_on(void)
{
	int rc = -EIO;

	if (FMTRX_HW_STATE_IDLE == fmtrx_get_hw_state()) {
		struct fmrx_msgbox_buff rx_msg;

		rx_msg.event = FMRX_EVENT_POWER_ON;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EPERM;
	}

	return rc;
}

/* Power Off FM Radio */
int fmrx_power_off(void)
{
	int rc = -EIO;
	enum fmrx_hw_state fmrx_hw_status = fmrx_get_hw_state();

	if (FMRX_HW_STATE_ACTIVE == fmrx_hw_status) {
		struct fmrx_msgbox_buff rx_msg;

		rx_msg.event = FMRX_EVENT_POWER_OFF;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EPERM;
	}

	return rc;
}

/* Subscribe for RDS data */
int fmrx_subscribe_rds(
	struct fmrx_rds_subscribe_params *rds_params)
{
	int rc = -EIO;

	if (NULL != rds_params) {
		struct fmrx_msgbox_buff rx_msg;

		rx_msg.event = FMRX_EVENT_SUBSCRIBE_RDS;
		rx_msg.params.rds_subscribe = rds_params;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Unsubscribe for RDS data */
int fmrx_unsubscribe_rds(void)
{
	int rc = -EIO;
	struct fmrx_msgbox_buff rx_msg;

	rx_msg.event = FMRX_EVENT_UNSUBSCRIBE_RDS;
	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

/* Subscribe for FM Rx RSSI information */
int fmrx_set_rssi(struct fmrx_rssi_subsc *rssi_config)
{
	int rc = -EIO;

	if (NULL == rssi_config) {
		rc = -EINVAL;
		goto fmrx_set_rssi_err;
	}

	if (rssi_config->subscribe)
		rc = fmrx_subscribe_rssi(rssi_config->lower_thres,
			rssi_config->upper_thres);
	else
		rc = fmrx_unsubscribe_rssi();

fmrx_set_rssi_err:
	return rc;
}

/*
 * Subscribe for FM Rx RSSI information.
 */
int fmrx_subscribe_rssi(s16 lower_thres, s16 upper_thres)
{
	int rc = -EIO;
	struct fmrx_msgbox_buff rx_msg;

	rx_msg.event = FMRX_EVENT_SUBSCRIBE_RSSI;
	rx_msg.params.rssi_subscribe.lower_thres = lower_thres;
	rx_msg.params.rssi_subscribe.upper_thres = upper_thres;
	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

/* Unsubscribes for RSSI information */
int fmrx_unsubscribe_rssi(void)
{
	int rc = -EIO;

	struct fmrx_msgbox_buff rx_msg;
	rx_msg.event = FMRX_EVENT_UNSUBSCRIBE_RSSI;
	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

int fmrx_station_tuning(u32 freq)
{
	int rc = -EIO;
	struct fmrx_msgbox_buff rx_msg;
	struct fmtrx_band fm_band;

	rx_msg.event = FMRX_EVENT_GET_FREQ_BAND_INFO;
	rx_msg.params.p_get_freq_band = &fm_band;
	rc = fmrx_event_dispatcher(&rx_msg);

	if (0 == rc) {
		if (freq <= fm_band.max && freq >= fm_band.min) {
			rx_msg.event = FMRX_EVENT_TUNE_STATION;
			rx_msg.params.ch_tune.freq = freq;
			rc = fmrx_event_dispatcher(&rx_msg);
		}
	}

	return rc;
}

/* Seeks for FM Radio valid channels */
int fmrx_station_seeking(struct fmrx_seek_station_params *seek_ch_info)
{
	int rc = -EIO;

	if (NULL != seek_ch_info) {
		if (seek_ch_info->seek_mode < FMRX_SEEK_MODE_INVALID ||
		    seek_ch_info->seek_mode >= FMRX_SEEK_OFF) {
			struct fmrx_msgbox_buff rx_msg;

			rx_msg.event = FMRX_EVENT_SEEK_STATION;
			rx_msg.params.p_seek_station = seek_ch_info;
			rc = fmrx_event_dispatcher(&rx_msg);
		} else {
			rc = -EINVAL;
		}
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/*
 * Quickly checks the effective RSSI on a given list of channels, with as
 * ttle disruption as possible to reception on the current channel.
 */
int fmrx_af_measure_rssi(struct fmtrx_rssi_report *list, u32 cnt)
{
	int rc = -EIO;

	if (NULL != list && cnt > 0) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_AF_MEASURE_RSSI;
		rx_msg.params.get_af_rssi.max_count   = cnt;
		rx_msg.params.get_af_rssi.rssi_report = list;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/*
 * Evaluates the given AF frequency. First, RSSI measurement is obtained and
 * validated against the given RSSI threshold. Second, PI code from RDS is
 * fetched and validated with the given PI code. If both conditions are passed,
 * en driver switches to the AF channel else, driver stays on the same channel.
 */
int fmrx_af_evaluate(u32 af_freq, u16 pi_code, s16 rssi_thres)
{
	int rc = -EIO;
	struct fmrx_msgbox_buff rx_msg;

	rx_msg.event = FMRX_EVENT_AF_EVALUATE;
	rx_msg.params.af_eval_params.af_freq = af_freq;
	rx_msg.params.af_eval_params.pi_code = pi_code;
	rx_msg.params.af_eval_params.af_rssi_th = rssi_thres;
	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

int fmrx_switch_antenna(enum fmtrx_ant_type fmrx_ant_type)
{
	int rc = -EIO;

	if (fmrx_ant_type < FMR_ANT_TYPE_END) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_SWITCH_ANT;
		rx_msg.params.ant_type = fmrx_ant_type;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}


	return rc;
}

/* Transfers the FM Rx RDS groups */
int fmrx_transfer_rds_groups(u16 group_cnt, void *rds_buf_desc,
	u16 *copied_groups)
{
	int rc = -EIO;

	if (NULL != rds_buf_desc && NULL != copied_groups) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_TRANSFER_RDS_GROUPS;
		rx_msg.params.get_rds_grps.copied_grps = copied_groups;
		rx_msg.params.get_rds_grps.rds_buf_desc = rds_buf_desc;
		rx_msg.params.get_rds_grps.group_cnt = group_cnt;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Registers the FM Rx callback functions */
int fmrx_register_callbacks(struct fmrx_callbacks *fmrx_callbacks)
{
	int rc = -EIO;

	if (NULL != fmrx_callbacks) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_REGISTER_CBS;
		rx_msg.params.p_reg_cbs = fmrx_callbacks;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

int fmrx_set_cfg(struct fmrx_cfg *cfg)
{
	int rc = -EIO;

	if (NULL != cfg) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_SET_CONFIG_DATA;
		rx_msg.params.p_set_cfg = cfg;

		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Gets the HLD configuration data */
int fmrx_get_cfg(struct fmrx_cfg **cfg)
{
	int rc = -EIO;

	struct fmrx_msgbox_buff rx_msg;
	rx_msg.event = FMRX_EVENT_GET_CONFIG_DATA;
	rx_msg.params.p_get_cfg_data = cfg;

	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

/* Sets the volume level of a audio channel of FM playback */
int fmrx_set_volume_level(enum fmrx_aud_channel aud_channel, u8 aud_ch_vol)
{
	int rc = -EIO;

	if (aud_channel >= FMRX_AUD_CHN_ALL &&
	    aud_channel < FMRX_AUD_CHN_END) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_SET_VOLUME;
		rx_msg.params.set_ch_vol.volume = aud_ch_vol;
		rx_msg.params.set_ch_vol.channel  = aud_channel;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Sets FM Reciever frequency band */
int fmrx_set_band(struct fmtrx_band *fmrx_band)
{
	int rc = -EIO;

	if (NULL != fmrx_band) {
		if (fmrx_band->max > fmrx_band->min) {
			struct fmrx_msgbox_buff rx_msg;

			rx_msg.event = FMRX_EVENT_SET_BAND;
			rx_msg.params.p_set_freq_band = fmrx_band;

			rc = fmrx_event_dispatcher(&rx_msg);
		} else {
			rc = -EINVAL;
		}
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Function to set the audio routing for FM Radio audio */
int fmrx_set_route(enum fmrx_aud_route routing)
{
	int rc = -EIO;

	if (FMR_AUD_PATH_INVALID > routing) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_SET_ROUTE;
		rx_msg.params.aud_routing = routing;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

int fmrx_set_mute(enum fmrx_aud_channel aud_channel, s32 mute)
{
	int rc = -EIO;

	if (FMRX_AUD_CHN_END > aud_channel) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_SET_MUTE;
		rx_msg.params.set_mute.mute = mute;

		/* aud_channel parameter is just ignored in the lower layers */
		rx_msg.params.set_mute.channel = aud_channel;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Sets the mono/stereo mode of FM radio playback */
int fmrx_set_force_mono(s32 mono)
{
	int rc = -EIO;
	struct fmrx_msgbox_buff rx_msg;

	rx_msg.event = FMRX_EVENT_SET_FORCE_MONO;
	rx_msg.params.force_mono.mono = mono;
	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

/* Sets the FM Rx RDS configuration */
int fmrx_set_rds_cfg(struct fmrx_rds_cfg *rds_cfgs)
{
	int rc = -EIO;

	if (NULL != rds_cfgs) {
		struct fmrx_msgbox_buff rx_msg;

		rx_msg.event = FMRX_EVENT_SET_RDS_CONFIG;
		rx_msg.params.p_set_rds_cfg = rds_cfgs;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Sets deemphasis configuration for FM reciever */
int fmrx_set_deemphasis_cfg(enum fmtrx_emph_sel deem_mode)
{
	int rc = -EIO;
	struct fmrx_msgbox_buff rx_msg;

	rx_msg.event = FMRX_EVENT_SET_DEEMPHASIS;
	rx_msg.params.set_deem = deem_mode;
	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

/* Gets the FM reciever Data state */
int fmrx_get_rx_data_state(
	struct fmrx_cfg *fmrx_data_state)
{
	int rc = -EIO;

	if (NULL != fmrx_data_state) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_GET_RX_STATE;
		rx_msg.params.p_state_data = fmrx_data_state;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

int fmrx_get_channel_info(struct fmrx_ch_info *channel_info)
{
	int rc = -EIO;

	if (NULL != channel_info) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_GET_CHANNEL_INFO;
		rx_msg.params.p_ch_info = channel_info;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Gets the FM Radio volume level on a given audio channel */
int fmrx_get_volume(enum fmrx_aud_channel aud_channel,
	u16 *channel_volume)
{
	int rc = -EIO;

	if ((NULL != channel_volume) || (aud_channel < FMRX_AUD_CHN_END)) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_GET_VOLUME;
		rx_msg.params.get_ch_vol.channel  = aud_channel;
		rx_msg.params.get_ch_vol.p_volume = channel_volume;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Get the FM Rx band of operation */
int fmrx_get_band_info(struct fmtrx_band *fmrx_band)
{
	int rc = -EIO;

	if (NULL != fmrx_band) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_GET_FREQ_BAND_INFO;
		rx_msg.params.p_set_freq_band = fmrx_band;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Gets FM Radio HLD, LLD and FW versions */
int fmrx_get_revisions(struct fmtrx_revision *fmtrx_rev)
{
	int rc = -EIO;

	if (NULL != fmtrx_rev) {
			struct fmrx_msgbox_buff rx_msg;
			rx_msg.event = FMRX_EVENT_GET_REVISIONS;
			rx_msg.params.p_get_rev_ids = fmtrx_rev;
			rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Force the FM Radio LO frequency sideband to the given value for debugging */
int fmrx_set_sideband_injection(enum fmrx_sb_inj_side sideband,
	enum fmtrx_inj_sel sb_sel)
{
	int rc = -EIO;

	if (FMR_FORCE_HSI <= sideband && FMR_FORCE_INVALID > sideband &&
	    FMR_SB_SEL_TMP <= sb_sel && FMR_SB_SEL_INVALID > sb_sel) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_SET_SIDEBAND;
		rx_msg.params.rf_sb_sel.inj_side = sideband;
		rx_msg.params.rf_sb_sel.sb_sel = sb_sel;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Dispatches a given FM Rx event to FM Rx module */
int fmrx_msg_dispatch(enum fmrx_event event,
	union fmrx_ev_prams evt_params)
{
	int rc = -EIO;

	if (FMRX_EVENT_END > event) {
		enum fmrx_hw_state fmrx_hw_state = fmrx_get_hw_state();

		if ((FMRX_HW_STATE_ACTIVE == fmrx_hw_state) ||
		    (FMRX_HW_STATE_SEEK_ACTIVE == fmrx_hw_state)) {
			struct fmrx_msgbox_buff fmrx_msg;
			fmrx_msg.event	= event;
			fmrx_msg.params = evt_params;
			rc = fmrx_event_dispatcher(&fmrx_msg);
		} else {
			rc = -EPERM;
		}
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Enable/disable the RX IDI handshake bit */
int fmrx_set_idi_handshake(s32 enable)
{
	int rc = -EIO;
	struct fmrx_msgbox_buff rx_msg;

	rx_msg.event = FMRX_EVENT_SET_IDI_HANDSHAKE;
	rx_msg.params.set_idi_hs_flag.enable = enable;
	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

/* Subscribes for RDS sync */
int fmrx_subscibe_rds_sync(void)
{
	int rc = -EIO;
	struct fmrx_msgbox_buff rx_msg;

	rx_msg.event = FMRX_EVENT_SUBSCRIBE_RDS_SYNC;
	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

/* Unsubscribes RDS sync */
int fmrx_unsubscribe_rds_sync(void)
{
	int rc = -EIO;
	struct fmrx_msgbox_buff rx_msg;

	rx_msg.event = FMRX_EVENT_UNSUBSCRIBE_RDS_SYNC;
	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

/* Starts the auto evaluation for FM radio channels */
int fmrx_channel_auto_eval(struct fmrx_ae_params *ch_ae_info)
{
	int rc = -EIO;

	if (NULL != ch_ae_info) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_AUTO_EVAL;
		rx_msg.params.p_ae_info = ch_ae_info;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Get the capablities of band sweep */
int fmrx_channel_auto_eval_get_cap(
	struct fmtrx_band *band_info)
{
	int rc = -EIO;

	if (NULL != band_info) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_AUTO_EVAL_GET_CAP;
		rx_msg.params.p_sweep_band = band_info;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Stop the ongoing auto evaluation of RSSI measurements */
int fmrx_channel_auto_eval_stop(void)
{
	int rc = -EIO;
	struct fmrx_msgbox_buff rx_msg;

	rx_msg.event = FMRX_EVENT_AUTO_EVAL_STOP;
	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

/* Get the current status of Auto evaluation */
int fmrx_channel_auto_eval_progress(u16 *ch_count,
	u16 *tot_chs)
{
	int rc = -EIO;

	if ((NULL != ch_count) && (NULL != tot_chs)) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_AUTO_EVAL_PROGRESS;
		rc = fmrx_event_dispatcher(&rx_msg);

		if (0 == rc) {
			*ch_count = rx_msg.params.ch_ae_stat.scan_count;
			*tot_chs = rx_msg.params.ch_ae_stat.total_count;
		}
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Sets the softmute configuration for FM Reception */
int fmrx_set_soft_mute_cfg(struct fmrx_sm *sm_cfg)
{
	int rc = -EIO;

	if (NULL != sm_cfg) {
			struct fmrx_msgbox_buff rx_msg;
			rx_msg.event = FMRX_EVENT_SET_SOFT_MUTE;
			rx_msg.params.p_set_sm_cfg = sm_cfg;
			rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Sets the AGC configuration for FM Reception */
int fmrx_set_agc_gain_cfg(struct fmrx_agc *gain_cfg)
{
	int rc = -EIO;

	if (NULL != gain_cfg) {
		if (gain_cfg->gain_idx <= AGC_GAIN_INDEX_15) {
			struct fmrx_msgbox_buff rx_msg;
			rx_msg.event = FMRX_EVENT_SET_AGC_GAIN;
			rx_msg.params.p_agc_gain_cfg = gain_cfg;
			rc = fmrx_event_dispatcher(&rx_msg);
		} else {
			rc = -EINVAL;
		}
	} else {
		rc = -EINVAL;
	}

	return rc;
}


/* Set SNC configuration for FM radio in reciever mode */
int fmrx_set_snc(struct fmrx_snc *snc_config)
{
	int rc = -EIO;

	if (NULL != snc_config) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_SET_SNC;
		rx_msg.params.p_snc_cfg = snc_config;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Sets RSSI offset configuration */
int fmrx_set_rssi_offset(s16 offset_hs, s16 offset_embed)
{
	int rc = -EIO;

	struct fmrx_msgbox_buff rx_msg;
	rx_msg.event = FMRX_EVENT_SET_RSSI_OFFSET;
	rx_msg.params.set_rssi_offs.rssi_offset_embed = offset_embed;
	rx_msg.params.set_rssi_offs.rssi_offset_hs = offset_hs;
	rc = fmrx_event_dispatcher(&rx_msg);

	return rc;
}

/* Sets the external LNA RSSI compensation */
int fmrx_get_ext_lna_rssi_comp(struct fmrx_ext_lna_cfg *lna_cfg,
	enum fmtrx_ant_type ant_type)
{
	int rc = -EIO;

	if (ant_type < FMR_ANT_TYPE_END && NULL != lna_cfg) {
		struct fmrx_msgbox_buff rx_msg;

		rx_msg.event = FMRX_EVENT_GET_EXT_LNA_RSSI_COMPEN;
		rx_msg.params.get_ext_lna_rssi_comp.ant_type = ant_type;
		rx_msg.params.get_ext_lna_rssi_comp.lna_cfg = lna_cfg;

		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Sets the external LNA RSSI compensation */
int fmrx_set_ext_lna_rssi_comp(
	struct fmrx_ext_lna_cfg lna_cfg, enum fmtrx_ant_type ant_type)
{
	int rc = -EIO;

	if (ant_type < FMR_ANT_TYPE_END) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_SET_EXT_LNA_RSSI_COMPEN;
		rx_msg.params.set_ext_lna_rssi_comp.ant_type = ant_type;
		rx_msg.params.set_ext_lna_rssi_comp.lna_cfg  = lna_cfg;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}


/* Get the RSSI level of the current FM channel of reception */
int fmrx_get_rssi_level(s16 *rssi_level)
{
	int rc = -EIO;

	if (NULL != rssi_level) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_GET_RSSI;
		rx_msg.params.p_rssi = rssi_level;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Gets RSSI offset information */
int fmrx_get_rssi_offset(s16 *offset_hs)
{
	int rc = -EIO;

	if (NULL != offset_hs) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_GET_RSSI_OFFSET;
		rx_msg.params.p_get_rssi_offset = offset_hs;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Dumps FW register content for debugging */
int fmrx_test_dump_fw_reg(struct fmrx_fw_reg *fw_reg)
{
	int rc = -EIO;

	if (NULL != fw_reg) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_TEST_DUMP_FW_REG;
		rx_msg.params.p_dump_regs = fw_reg;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

