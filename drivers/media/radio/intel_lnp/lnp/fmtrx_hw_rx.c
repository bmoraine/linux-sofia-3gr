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
**				MODULE DESCRIPTION
**
** =============================================================================
*/
/* This file contains the HW abstraction interfaces for RX functionality */

/*
** =============================================================================
**
**				INCLUDE STATEMENTS
**
** =============================================================================
*/
#include "fmtrx_sys.h"		/* System related */
#include "fmtrx_common.h"		/* Common TRX data types */
#include "fmtrx_hw_rx.h"		/* HW interfaces */
#include "fmtrx_hw_cmds.h"		/* FW command interfaces */
#include "fmr_rx_api.h"		/* FW interfaces */

/*
** =============================================================================
**
**				DEFINES
**
** =============================================================================
*/
#define FMR_HW_SW_ID "1.0"
#define LO_BAND_LIMIT 76000
#define HI_BAND_LIMIT 108000
#define FREQUENCY_104MHZ 104000

#define FW_RDS_GROUP_SIZE sizeof(struct rds_group)
#define BYTES_TO_RDS_GROUP_CNT(X) ((X / sizeof(struct rds_group)) - 1)

#define RDS_MIN_FREE_THR 2

/*
** =============================================================================
**
**				LOCAL DATA DECLARATIONS
**
** =============================================================================
*/

/*
** =============================================================================
**
**				LOCAL DATA DEFINITIONS
**
** =============================================================================
*/

/*
** =============================================================================
**
**				LOCAL FUNCTION DECLARATIONS
**
** =============================================================================
*/
/* Send command to FW
 * @type FMR RX or TX
 * @cmd_pkt Pointer to the command buffer that has command id and command params
 * @cmd_params_size Size of the command params
 */
static int fmtrx_send_cmd(
		enum fmtrx_type type,
		struct dsp_cmd_pkt *cmd_pkt,
		u16 cmd_params_size);

/* Set interrupts in FMR IP
 * @interrupt_mask Interrupt mask to be written to the HW register
 * @interrupt_enable Enable/disable interrupts that are masked
 */
static int fmtrx_hw_set_interrupts(
		enum interrupt_type intmask,
		bool interrupt_enable);

/* Get channel information from FMR IP - FW
 * @data Pointer to the fetched channel information
 */
static int fmrx_hw_get_fw_channel_info(
		struct dsp_ch_info *data);

/* Get FW RDS related buffer information
 * @size Pointer to the fetched size information
 * @buf_start Pointer to the fetched RDS buffer start address
 * @buf_end Pointer to the fetched RDS buffer end address
 * @host_read_ptr Pointer to the HOST read pointer location
 * @fw_write_ptr Pointer to the FW write pointer location
 */
static int fmrx_hw_get_rds_fw_buffer_info(
		struct dsp_rds_buf_info *info);

/* Get RDS group status
 * @buf_info Pointer to the RDS buffer information [IN]
 * @group_count Pointer to the number of groups available [OUT]
 * @discard_count Pointer to the number of groups discarded [OUT]
 */
static int fmrx_hw_get_rds_count_status(
		struct dsp_rds_buf_info *buf_info,
		u16 *group_count,
		u16 *discard_count);

/* Write data to a bit field
 * @addroffs FMR IP address offset
 * @pos Bit position
 * @width Bit field width
 * @value Value to be written
 */
static int fmtrx_write_field(
		u32 addroffs,
		u16 pos,
		u16 width,
		u32 value);

/* Write data to a particular bit
 * @addroffs FMR IP address offset
 * @pos Bit position
 * @value Value to be written
 */
static int fmtrx_write_bit(
		u32 addroffs,
		u16 pos,
		u32 value);

/* Read masked data from a 32-bit address
 * @addroffs FMR IP address offset
 * @value Value to be read
 * @mask Mask to be applied
 */
static int fmtrx_read32_masked(
		u32 addroffs,
		u32 *value,
		u32 mask);

/*
** =============================================================================
**
**				EXPORTED FUNCTION DEFINITIONS
**
** =============================================================================
*/

int fmtrx_hw_rf_poweron(
		enum antenna_type type,
		enum lna_out_gain gain)
{
	int err = 0;
	struct dsp_cmd_pkt cmd_pkt = {
		.cmd_id = FMRX_CMD_RF_POWER_UP,
		.cmd_params.rf_poweron_params = {
		(u16)type, (u16)gain, 0, 0 } };

	/* Validate input arguments */
	if ((ANTENNA_INVALID <= type) || (GAIN_INVALID <= gain)) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmtrx_hw_rf_poweron_exit;
	}

	switch (type) {
	case ANTENNA_HS_SINGLEEND:
		cmd_pkt.cmd_params.rf_poweron_params.antenna_type =
					ANTTYPE_HS_SINGLE;
		break;
	case ANTENNA_HS_DIFFERENTIAL:
		cmd_pkt.cmd_params.rf_poweron_params.antenna_type =
					ANTTYPE_HS_DIFF;
		break;
	case ANTENNA_EBD_SINGLEEND:
		cmd_pkt.cmd_params.rf_poweron_params.antenna_type =
					ANTTYPE_EBD_SINGLE;
		break;
	case ANTENNA_EBD_DIFFERENTIAL:
		cmd_pkt.cmd_params.rf_poweron_params.antenna_type =
					ANTTYPE_EBD_DIFF;
		break;
	case ANTENNA_INVALID:
	/* Intended fallthrough */
	default:
		err = -EINVAL;
		goto fmtrx_hw_rf_poweron_exit;
	}

	/* Send command to DSP */
	err = fmtrx_send_cmd(
			FMTRX_RX, &cmd_pkt,
			sizeof(struct dsp_rf_poweron_cmd_params));
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Send RF power on command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

fmtrx_hw_rf_poweron_exit:
	return err;
}

int fmtrx_hw_rf_poweroff(
		void)
{
	int err = 0;
	struct dsp_cmd_pkt cmd_pkt = {
		.cmd_id = FMRX_CMD_RF_POWER_DOWN };

	/* Send command to DSP */
	err = fmtrx_send_cmd(
		FMTRX_RX, &cmd_pkt, 0);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d,Send RF power off command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	return err;
}

int fmtrx_hw_start_minidsp(
		void)
{
	int err = 0;

	/* Set PC to zero */
	err = fmtrx_write_field(
			MINIDSPCTL_ADDR, MINIDSPCTL_PC_POS32,
			MINIDSPCTL_PC_WIDTH, 0);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write PC failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	/* Set run bit to start executing FW */
	err = fmtrx_write_bit(
			MDSPHALTCTL_ADDR,
			MDSPHALTCTL_MST_RUN_POS32, 1);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write RUN bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	return err;
}

int fmtrx_hw_is_minidsp_running(
		bool *status)
{
	int err = 0;
	u32 val = 0;

	if (0 == status) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmtrx_hw_is_minidsp_running_exit;
	}

	err = fmtrx_sys_reg_read32(
			MINIDSPCTL_ADDR, &val);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	*status =
			(bool)((val & MDSPDBG_RUNNING_MASK32) >>
					MDSPDBG_RUNNING_POS32);

fmtrx_hw_is_minidsp_running_exit:
	return err;
}

int fmtrx_hw_stop_minidsp(
		void)
{
	int err = 0;

	/* Set halt bit to stop executing FW */
	err = fmtrx_write_bit(
		MDSPHALTCTL_ADDR,
		MDSPHALTCTL_MST_HALT_POS32, 1);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write HALT bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	return err;
}

int fmtrx_hw_set_gain_offsets(
		enum gain_offset_type type,
		u8  *offs,
		u32 size)
{
	int err = 0;
	struct dsp_cmd_pkt cmd_pkt = {
		.cmd_id = FMRX_CMD_CFG_LNA_GAINOFFS,
		.cmd_params.offs_params = {
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0}
	};

	/* Validate input arguments */
	if ((GAIN_OFFSET_INVALID <= type) || (0 == offs) || (0 == size)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmtrx_hw_set_gain_offsets_exit;
	}

	memcpy((u8 *)&cmd_pkt.cmd_params.offs_params,
				offs, size);

	switch (type) {
	case GAIN_OFFSET_LNA:
		cmd_pkt.cmd_id =
				FMRX_CMD_CFG_LNA_GAINOFFS;
		break;
	case GAIN_OFFSET_PPF:
		cmd_pkt.cmd_id =
				FMRX_CMD_CFG_PPF_GAINOFFS;
		break;
	case GAIN_OFFSET_RSSI:
		cmd_pkt.cmd_id =
				FMRX_CMD_CFG_RSSI_CH_OFFS;
		/* Convert from dBuV to 0.25dBuV for FW */
		cmd_pkt.cmd_params.offs_params.offset11 =
				RSSI_TO_0_25_DBUV(cmd_pkt.cmd_params.
					offs_params.offset11);
		cmd_pkt.cmd_params.offs_params.offset12 =
				RSSI_TO_0_25_DBUV(cmd_pkt.cmd_params.
					offs_params.offset12);
		cmd_pkt.cmd_params.offs_params.offset13 =
				RSSI_TO_0_25_DBUV(cmd_pkt.cmd_params.
					offs_params.offset13);
		cmd_pkt.cmd_params.offs_params.offset14 =
				RSSI_TO_0_25_DBUV(cmd_pkt.cmd_params.
					offs_params.offset14);
		cmd_pkt.cmd_params.offs_params.offset15 =
				RSSI_TO_0_25_DBUV(cmd_pkt.cmd_params.
					offs_params.offset15);
		cmd_pkt.cmd_params.offs_params.offset16 =
				RSSI_TO_0_25_DBUV(cmd_pkt.cmd_params.
					offs_params.offset16);
		break;
	case GAIN_OFFSET_CP_INIT:
		cmd_pkt.cmd_id =
				FMRX_CMD_CFG_ANT_TUNE_CP_INIT;
		break;
	default:
		cmd_pkt.cmd_id =
				FMRX_CMD_CFG_LNA_GAINOFFS;
		break;
	}

	/* Send command to DSP */
	err = fmtrx_send_cmd(
		FMTRX_RX, &cmd_pkt, size);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Send offset config command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

fmtrx_hw_set_gain_offsets_exit:
	return err;
}

int fmrx_hw_set_rssi_other_offset(
		s16 rssi_other_offset)
{
	int err = 0;

	rssi_other_offset = RSSI_TO_0_25_DBUV(rssi_other_offset);

	err = fmtrx_sys_reg_write16(
			FMR_RXMAIN_RSSI_OTHER_OFFS_ADDR,
			rssi_other_offset);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	return err;
}

int fmtrx_hw_set_band(
		u32 lo_band,
		u32 hi_band)
{
	int err = 0;
	struct dsp_cmd_pkt cmd_pkt = {
			.cmd_id = FMRX_CMD_CFG_BAND,
				.cmd_params.band_cfg_params = {
					lo_band, hi_band } };

	/* Validate input arguments */
	if ((0 == lo_band) || (0 == hi_band) ||
			(lo_band >= hi_band) ||
			(LO_BAND_LIMIT > lo_band) ||
			(HI_BAND_LIMIT < hi_band)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmtrx_hw_set_band_exit;
	}

	/* Send command to DSP */
	err = fmtrx_send_cmd(
		FMTRX_RX, &cmd_pkt,
		sizeof(struct dsp_band_cfg_cmd_params));
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Send band config command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

fmtrx_hw_set_band_exit:
	return err;
}

int fmrx_hw_set_audio_deemp(
		enum deemphasis_type type)
{
	int err = 0;
	u16 de_emp = 0;

	if (DEEMP_INVALID <= type) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmrx_hw_set_audio_deemp_exit;
	}

	de_emp =
		(DEEMP_50US == type) ? DEEM_50us_SRC :
		 DEEM_75us_SRC;
	err = fmtrx_sys_reg_write16(
			FMR_RXMAIN_DEEM_TD_ADDR,
			de_emp);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Set De-emp failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_audio_deemp_exit;
	}

fmrx_hw_set_audio_deemp_exit:
	return err;
}

int fmrx_hw_set_audio_volumeramp(
		s16 volume_ramp)
{
	int err = 0;

	err = fmtrx_sys_reg_write16(
			FMR_RXMAIN_AUDIOGAIN_IIR_PARAM_ADDR,
			volume_ramp);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Set volume ramp failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	return err;
}

int fmrx_hw_set_audio_forcemono(
		bool force_mono)
{
	int err = 0;

	err = fmtrx_sys_reg_write16(
			FMR_RXMAIN_FORCE_MONO_ADDR,
			((force_mono) ? FORCE_MONO :
				EN_STEREO));
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Set force mono failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	return err;
}

int fmrx_hw_set_mute(
		bool mute_enable,
		bool mute_wait)
{
	int err = 0;
	u16 left = 0, right = 0;

	err = fmtrx_sys_reg_write16(
			FMR_RXMAIN_AUDIO_MUTE_ADDR, ((mute_enable) ?
			AUDIO_MUTE_ENABLED : AUDIO_MUTE_DISABLED));
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Set mute failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_mute_exit;
	}

	/* Loop until the audio volume becomes zero */
	if ((true == mute_wait) &&
			(true == mute_enable)) {
		do {
			err = fmtrx_sys_reg_read16(
				FMR_RXMAIN_AUDIO_GAIN_OUT_L_ADDR,
				&left);
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d,Read 16-bit failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmrx_hw_set_mute_exit;
			}

			err = fmtrx_sys_reg_read16(
				FMR_RXMAIN_AUDIO_GAIN_OUT_R_ADDR,
				&right);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,Read 16-bit failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmrx_hw_set_mute_exit;
			}
		} while ((left != 0) || (right != 0));
	}

fmrx_hw_set_mute_exit:
	return err;
}

int fmrx_hw_set_volume(
		u8 left,
		u8 right)
{
	int err = 0;

	/* Volume change is not allowed - AGC
			drives full gain in LnP */

	return err;
}

int fmrx_hw_set_routing(
		enum routing_mode mode)
{
	int err = 0;

	/* Routing change is not allowed - it's
			always SRC in LnP */

	return err;
}

int fmrx_hw_set_snc(
		struct snc *cfg)
{
	int err = 0;
	s16 step = 0, hi_thr = 0, lo_thr = 0;

	/* Validate input arguments */
	if ((0 == cfg) || (cfg->lo_thr > cfg->hi_thr)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments\n",
			FILE, __func__);
		goto fmrx_hw_set_snc_exit;
	}

	/* Convert to FW units */
	lo_thr = RSSI_TO_0_25_DBUV(cfg->lo_thr);
	hi_thr = RSSI_TO_0_25_DBUV(cfg->hi_thr);

	/* Calculate step size */
	step =
		(cfg->lo_thr == cfg->hi_thr) ?
		0 : (32767 / (hi_thr - lo_thr));
	/* Absolute value is needed */
	step = (step < 0) ? (-step) : step;

	err = fmtrx_sys_reg_write16(
				FMR_RXMAIN_SNC_STEP_ADDR,
				step);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_snc_exit;
	}

	err = fmtrx_sys_reg_write16(
				FMR_RXMAIN_SNC_THR_UP_ADDR,
				hi_thr);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_snc_exit;
	}

	err = fmtrx_sys_reg_write16(
				FMR_RXMAIN_SNC_THR_LO_ADDR,
				lo_thr);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_snc_exit;
	}

	err = fmtrx_sys_reg_write16(
				FMR_RXMAIN_SNC_EN_ADDR,
				((cfg->enable) ? SNC_ENABLED :
				SNC_DISABLED));
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_snc_exit;
	}

fmrx_hw_set_snc_exit:
	return err;
}

int fmrx_hw_set_sm(
		struct sm *cfg)
{
	int err = 0;
	s16 thr = 0;
	u16 step = 0;

	/* Validate input arguments */
	if (0 == cfg) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments\n",
			FILE, __func__);
		goto fmrx_hw_set_sm_exit;
	}

	/* Convert to FW units */
	thr = RSSI_TO_0_25_DBUV(cfg->thr);
	step = STEP_SIZE_TO_LINEAR(cfg->step);

	err = fmtrx_sys_reg_write16(
				FMR_RXMAIN_SM_STEP_ADDR, step);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_sm_exit;
	}

	err = fmtrx_sys_reg_write16(
				FMR_RXMAIN_SM_THR_UP_ADDR, thr);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_sm_exit;
	}

	err = fmtrx_sys_reg_write16(
			FMR_RXMAIN_SM_EN_ADDR,
			((cfg->enable) ? SOFT_MUTE_ENABLED :
				SOFT_MUTE_DISABLED));
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_sm_exit;
	}

fmrx_hw_set_sm_exit:
	return err;
}

int fmrx_hw_set_agc(
		struct agc *cfg)
{
	int err = 0;
	struct dsp_cmd_pkt cmd_pkt = {
			.cmd_id = FMRX_CMD_CFG_AGC,
			.cmd_params.agc_params = {
			 AGC_DISABLED, 0, 0, 0} };

	/* Validate input arguments */
	if (0 == cfg) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments\n",
			FILE, __func__);
		goto fmrx_hw_set_agc_gain_exit;
	}

	/* Validate input arguments */
	if (AGC_GAIN_INVALID <= cfg->idx) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments\n",
			FILE, __func__);
		goto fmrx_hw_set_agc_gain_exit;
	}

	cmd_pkt.cmd_params.agc_params.agc_en =
			(cfg->enable) ? AGC_ENABLED : AGC_DISABLED;
	cmd_pkt.cmd_params.agc_params.fixed_gain_idx =
			cfg->idx;

	/* Send command to DSP */
	err = fmtrx_send_cmd(
			FMTRX_RX, &cmd_pkt,
			sizeof(struct dsp_agc_gain_cmd_params));
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d,Send AGC gain control command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

fmrx_hw_set_agc_gain_exit:
	return err;
}

int fmrx_hw_set_rssi_notification(
		struct rssi_notify *cfg)
{
	int err = 0;

	/* RSSI notification feature disabled for LNP */

	return err;
}

int fmrx_hw_set_rds_cfg(
		u16 good_blks,
		u16 bad_blks,
		u16 bad_blks_search)
{
	int err = 0;
	struct dsp_cmd_pkt cmd_pkt = {
			.cmd_id = FMRX_CMD_CFG_RDS,
			.cmd_params.rds_params = {
			good_blks, bad_blks, bad_blks_search } };

	/* Send command to DSP */
	err = fmtrx_send_cmd(
			FMTRX_RX, &cmd_pkt,
			sizeof(struct dsp_rds_cmd_params));
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Send RDS Cfg command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_rds_cfg_exit;
	}

fmrx_hw_set_rds_cfg_exit:
	return err;
}

int fmrx_hw_set_rds_onmode(
		enum rds_onmode mode)
{
	int err = 0;
	u16 fw_mode = 0;

	if (RDS_ONMODE_INVALID <= mode) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments",
			FILE, __func__);
		goto fmrx_hw_set_rds_onmode_exit;
	}

	switch (mode) {
	case RDS_ONMODE_OFF:
		fw_mode = RDS_MODE_OFF;
		break;
	case RDS_ONMODE_ON:
		fw_mode = RDS_MODE_ON;
		break;
	case RDS_ONMODE_RETAIN:
		fw_mode = RDS_MODE_RETAIN;
		break;
	default:
		fw_mode = RDS_MODE_OFF;
		break;
	}

	err = fmtrx_sys_reg_write16(
				FMR_RXMAIN_RDS_MODE_ADDR,
				fw_mode);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_rds_onmode_exit;
	}

fmrx_hw_set_rds_onmode_exit:
	return err;
}

int fmrx_hw_set_rds_pimode(
		bool rds_pimode)
{
	int err = 0;

	err = fmtrx_sys_reg_write16(
				FMR_RXMAIN_RDS_FAST_PI_ADDR,
				rds_pimode);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	return err;
}

int fmtrx_hw_set_idi_hs(
		enum fmtrx_type type,
		bool hs_enable)
{
	int err = 0;

	return err;
}

int fmrx_hw_set_clk_source(
		enum clk_source src)
{
	int err = 0;

	/* Clock source change is not allowed - FW
			controls clock in LnP */

	return err;
}

int fmtrx_hw_set_interrupt_clear(
		u32 interrupt_clr)
{
	int err = 0;

	err = fmtrx_sys_reg_write32(
				INTSETCTL_ADDR,
				interrupt_clr << 16);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	return err;
}

int fmrx_hw_channel_tune(
		u32 frequency,
		enum injection_side side,
		s16 rssi_thr,
		u32 clk_switch_range_104)
{
	int err = 0;
	s16 status = 0;
	struct dsp_cmd_pkt cmd_pkt = { .cmd_id = FMRX_CMD_CH_TUNE,
		.cmd_params.channel_tune_params = {
			frequency, side,
			RSSI_TO_0_25_DBUV(rssi_thr), 0 } };

	if ((LO_BAND_LIMIT > frequency) ||
			(HI_BAND_LIMIT < frequency) ||
			(INJECTION_SIDE_INVALID <= side)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmrx_hw_channel_tune_exit;
	}

	/* Select FW injection side */
	switch (side) {
	case INJECTION_SIDE_AUTO:
		cmd_pkt.cmd_params.channel_tune_params.inj_side_sel =
				FMRX_CH_TUNE_INJ_SIDE_AUTO;
		break;
	case INJECTION_SIDE_HSI:
		cmd_pkt.cmd_params.channel_tune_params.inj_side_sel =
				FMRX_CH_TUNE_INJ_SIDE_FORCE_HSI;
		break;
	case INJECTION_SIDE_LSI:
		cmd_pkt.cmd_params.channel_tune_params.inj_side_sel =
				FMRX_CH_TUNE_INJ_SIDE_FORCE_LSI;
		break;
	default:
		cmd_pkt.cmd_params.channel_tune_params.inj_side_sel =
				FMRX_CH_TUNE_INJ_SIDE_AUTO;
		break;
	}

	err = fmrx_hw_set_clk_source(CLK_SRC_MAINCLK);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Switch to main clock failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_channel_tune_exit;
	}

	/* Send command to DSP */
	err = fmtrx_send_cmd(
				FMTRX_RX, &cmd_pkt,
				sizeof(struct dsp_channel_tune_cmd_params));
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d,Send Channel tune command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_channel_tune_exit;
	}

	/* Fetch the channel tune status */
	err = fmtrx_sys_reg_read16(
				FMR_RXMAIN_CH_TUNE_STATUS_ADDR,
				(u16 *)&status);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_channel_tune_exit;
	}

#ifdef FMR_HOST_TEST
	status = FMRX_CH_TUNE_STATUS_OK;
#endif

	/* Classify err code based on status value */
	switch (status) {
	case FMRX_CH_TUNE_STATUS_OK:
		err = 0;
		break;
	case FMRX_CH_TUNE_STATUS_OUT_OF_BAND:
		err = -ECHRNG;
		break;
	case FMRX_CH_TUNE_STATUS_RUNNING:
		err = -EALREADY;
		break;
	case FMRX_CH_TUNE_STATUS_PLL_FAIL:
	default:
		err = -EAGAIN;
		break;
	}

	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Channel tune failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_channel_tune_exit;
	}

	/* Switch to internal clock, if needed */
	if ((frequency >
			(FREQUENCY_104MHZ - clk_switch_range_104)) &&
	    (frequency <
	    (FREQUENCY_104MHZ + clk_switch_range_104))) {
		err = fmrx_hw_set_clk_source(
						CLK_SRC_INTERNAL_RFCLK);
		if (0 != err) {
			fmtrx_sys_log
			("%s: %s %d,Switch to internal clock failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_hw_channel_tune_exit;
		}
	}

#ifdef CONFIG_IUI_FM_FMR
	{
		union component_data mitigate_data;

		mitigate_data.fm_cfg.frequency = frequency;
		mitigate_data.fm_cfg.side = side;
		fmtrx_sys_mitigate_interference(COMPONENT_FM,
			&mitigate_data);
	}
#endif /* CONFIG_IUI_FM_FMR */

fmrx_hw_channel_tune_exit:
	return err;
}

int fmrx_hw_channel_search(
		u32 start_frequency,
		u32 stop_frequency,
		s16 step,
		enum injection_side side,
		s16 rssi_thr,
		u16 pn_thr)
{
	int err = 0;
	s16 status = 0;
	struct dsp_cmd_pkt cmd_pkt = {.cmd_id = FMRX_CMD_CH_SEARCH,
		.cmd_params.channel_search_params = {
		start_frequency, stop_frequency, step,
		side, RSSI_TO_0_25_DBUV(rssi_thr),
		pn_thr, 0 }
	};

	/* Validate input arguments */
	if ((LO_BAND_LIMIT > start_frequency) ||
		(HI_BAND_LIMIT < stop_frequency) ||
		(INJECTION_SIDE_INVALID <= side)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmrx_hw_channel_search_exit;
	}

	/* Select FW injection side */
	switch (side) {
	case INJECTION_SIDE_AUTO:
		cmd_pkt.cmd_params.channel_search_params.inj_side_sel =
				FMRX_CH_SEARCH_INJ_SIDE_AUTO;
		break;
	case INJECTION_SIDE_HSI:
		cmd_pkt.cmd_params.channel_search_params.inj_side_sel =
				FMRX_CH_SEARCH_INJ_SIDE_FORCE_HSI;
		break;
	case INJECTION_SIDE_LSI:
		cmd_pkt.cmd_params.channel_search_params.inj_side_sel =
				FMRX_CH_SEARCH_INJ_SIDE_FORCE_LSI;
		break;
	default:
		cmd_pkt.cmd_params.channel_search_params.inj_side_sel =
				FMRX_CH_SEARCH_INJ_SIDE_AUTO;
		break;
	}

	err = fmrx_hw_set_clk_source(
				CLK_SRC_MAINCLK);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Switch to main clock failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_channel_search_exit;
	}

	/* Send command to DSP */
	err = fmtrx_send_cmd(
				FMTRX_RX, &cmd_pkt,
				sizeof(struct dsp_channel_search_cmd_params));
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Send Channel tune command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_channel_search_exit;
	}

	/* Fetch the channel tune status */
	err = fmtrx_sys_reg_read16(
			FMR_RXMAIN_CH_SEARCH_STATUS_ADDR,
			(u16 *)&status);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_channel_search_exit;
	}

#ifdef FMR_HOST_TEST
	status = FMRX_CH_SEARCH_STATUS_OK;
#endif

	/* Classify err code based on status value */
	switch (status) {
	case FMRX_CH_SEARCH_STATUS_OK:
		err = 0;
		break;
	case FMRX_CH_SEARCH_STATUS_IDLE:
	/* Intended fallthrough */
	case FMRX_CH_SEARCH_STATUS_OUT_OF_BAND:
		err = -ECHRNG;
		break;
	case FMRX_CH_SEARCH_STATUS_RUNNING:
		err = -EALREADY;
		break;
	case FMRX_CH_SEARCH_STATUS_FAIL:
	/* Intended fallthrough */
	case FMRX_CH_SEARCH_STATUS_PLL_FAIL:
	/* Intended fallthrough */
	default:
		err = -EAGAIN;
		break;
	}
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Channel search failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_channel_search_exit;
	}

#ifdef FMR_DEBUG_LVL1
	{
		struct dsp_ch_info data;
		enum injection_side side = INJECTION_SIDE_INVALID;

		err = fmrx_hw_get_fw_channel_info(&data);
		if (0 != err) {
			fmtrx_sys_log
			("%s: %s %d,Get FW channel information failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_hw_channel_search_exit;
		}

		fmtrx_sys_log
			("FMR_HW: Frequency %d\n",
			data.ch_freq);
		fmtrx_sys_log
			("FMR_HW: LO frequency %d\n",
			data.lo_freq);
		fmtrx_sys_log
			("FMR_HW: VCO frequency %d\n",
			data.vco_freq);
		fmtrx_sys_log
			("FMR_HW: LSI RSSI %d\n",
			RSSI_TO_DBUV(data.lsi_rssi));
		fmtrx_sys_log
			("FMR_HW: LSI IMG RSSI %d\n",
			RSSI_TO_DBUV(data.lsi_img_rssi));
		fmtrx_sys_log
			("FMR_HW: HSI RSSI %d\n",
			RSSI_TO_DBUV(data.hsi_rssi));
		fmtrx_sys_log
			("FMR_HW: HSI IMG RSSI %d\n",
			RSSI_TO_DBUV(data.hsi_img_rssi));
		fmtrx_sys_log
			("FMR_HW: FOFFS %d\n", data.foffs);
		fmtrx_sys_log
			("FMR_HW: PN %d\n", data.pn);
		switch (data.inj_side) {
		case FMRX_CH_TUNE_INJ_SIDE_AUTO:
			side = INJECTION_SIDE_AUTO;
			break;
		case FMRX_CH_TUNE_INJ_SIDE_FORCE_LSI:
			side = INJECTION_SIDE_LSI;
			break;
		case FMRX_CH_TUNE_INJ_SIDE_FORCE_HSI:
			side = INJECTION_SIDE_HSI;
			break;
		default:
			break;
		}
		fmtrx_sys_log
			("FMR_HW: Injection side %d\n", side);
	}
#endif

fmrx_hw_channel_search_exit:
	return err;
}

int fmrx_hw_rds_reset(
		void)
{
	int err = 0;
	u32 host_read_ptr = 0;

	/* Set HOST read pointer to the last group */
	host_read_ptr =
			FMR_RXMAIN_HOSTIF_BUF_END_ADDR -
			FW_RDS_GROUP_SIZE;

	err = fmtrx_sys_reg_write16(
				FMR_RXMAIN_RDS_HOST_READ_PTR_ADDR,
				host_read_ptr);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_rds_reset_exit;
	}

fmrx_hw_rds_reset_exit:
	return err;
}

int fmrx_hw_get_fw_state(
		enum fmrx_state *state)
{
	int err = 0;
	u16 temp_state = 0;

	if (0 == state) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmrx_hw_get_fw_state_exit;
	}

	/* Read state */
	err = fmtrx_sys_reg_read16(
				FMR_RXMAIN_STATE_ADDR, &temp_state);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_fw_state_exit;
	}
	switch (temp_state) {
	case FMRX_STATE_IDLE:
		*state = FMRX_HW_STATE_IDLE;
		break;
	case FMRX_STATE_RECEIVE:
	/* Intended fallthrough */
	case FMRX_STATE_RUN_BABS:
	/* Intended fallthrough */
	case FMRX_STATE_RC_ALIGN:
	/* Intended fallthrough */
	case FMRX_STATE_RF_POWER_UP:
	/* Intended fallthrough */
	case FMRX_STATE_RF_POWER_DOWN:
	/* Intended fallthrough */
	case FMRX_STATE_CH_TUNE:
		*state = FMRX_HW_STATE_RX_ACTIVE;
		break;
	case FMRX_STATE_CH_SEARCH:
		*state = FMRX_HW_STATE_RX_SEEKING;
		break;
	case FMRX_STATE_HALTED:
	/* Intended fallthrough */
	default:
		*state = FMRX_HW_STATE_INVALID;
		break;
	}

fmrx_hw_get_fw_state_exit:
	return err;
}

int fmrx_hw_set_fw_state(
		enum fmrx_state state)
{
	int err = 0;
	struct dsp_cmd_pkt cmd_pkt = {
				.cmd_id = FMRX_CMD_IDLE,
				.cmd_params.dummy_params = { 0 } };

	/* Validate input arguments */
	if (FMRX_HW_STATE_INVALID <= state) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmrx_hw_set_fw_state_exit;
	}

	/* Select command id based on the requested FW state */
	switch (state) {
	case FMRX_HW_STATE_IDLE:
		cmd_pkt.cmd_id = FMRX_CMD_IDLE;
		break;
	case FMRX_HW_STATE_RX_ACTIVE:
		cmd_pkt.cmd_id = FMRX_CMD_RECEIVE;
		break;
	default:
		err = -EINVAL;
		goto fmrx_hw_set_fw_state_exit;
	}

	/* Send command to DSP */
	err = fmtrx_send_cmd(
				FMTRX_RX, &cmd_pkt, 0);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Send state change command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_set_fw_state_exit;
	}

fmrx_hw_set_fw_state_exit:
	return err;
}

int fmrx_hw_get_channel_info(
		struct channel_info *data)
{
	int err = 0;
	u16 stereo = 0, inj_side = 0,
		rds_enable = 0, rds_sync = 0;

	if (0 == data) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmrx_hw_get_channel_info_exit;
	}

	/* Read frequency */
	err = fmtrx_sys_reg_read32(
				FMR_RXMAIN_CH_INFO_CH_FREQ_ADDR,
				&data->frequency);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_channel_info_exit;
	}

	/* Read stereo mode */
	err = fmtrx_sys_reg_read16(
			FMR_RXMAIN_STEREO_STATUS_ADDR,
				&stereo);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_channel_info_exit;
	}
	data->is_stereo =
			(STEREO_STATUS_STEREO == stereo) ?
			true : false;

	err = fmtrx_sys_reg_read16(
				FMR_RXMAIN_RSSI_ADDR,
				(u16 *)&data->rssi);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_channel_info_exit;
	}
	data->rssi = RSSI_TO_DBUV(data->rssi);

	err = fmtrx_sys_reg_read16(
				FMR_RXMAIN_PN_ADDR,
				(u16 *)&data->pn);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_channel_info_exit;
	}

	err = fmtrx_sys_reg_read32(
				FMR_RXMAIN_FOFFS_ADDR,
				(u32 *)&data->foffs);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_channel_info_exit;
	}

	err = fmtrx_sys_reg_read16(
			FMR_RXMAIN_PILOT_AMPL_HZ_ADDR,
			(u16 *)&data->pilot_amplitude);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_channel_info_exit;
	}

	err = fmtrx_sys_reg_read16(
			FMR_RXMAIN_CH_INFO_INJ_SIDE_ADDR,
			&inj_side);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_channel_info_exit;
	}

	switch (inj_side) {
	case FMRX_CH_TUNE_IS_SELECTED_LSI:
		data->inj_side = INJECTION_SIDE_LSI;
		break;
	case FMRX_CH_TUNE_IS_SELECTED_HSI:
		data->inj_side = INJECTION_SIDE_HSI;
		break;
	default:
		data->inj_side = INJECTION_SIDE_AUTO;
		break;
	}

	err = fmtrx_sys_reg_read16(
				FMR_RXMAIN_RDS_MODE_ADDR, &rds_enable);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_channel_info_exit;
	}
	data->rds_enable =
				(RDS_MODE_OFF == rds_enable) ?
				false : true;

	err = fmtrx_sys_reg_read16(
				FMR_RXMAIN_RDS_SYNC_ADDR, &rds_sync);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_channel_info_exit;
	}
	data->is_rds_sync_enabled =
				(0 == rds_sync) ? false : true;

	err = fmrx_hw_get_fw_state(&data->state);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Get FW state failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_channel_info_exit;
	}

fmrx_hw_get_channel_info_exit:
	return err;
}

int fmrx_hw_get_channel_freq(
		u32 *data)
{
	int err = 0;

	if (0 == data) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmrx_hw_get_channel_freq_exit;
	}

	/* Read frequency */
	err = fmtrx_sys_reg_read32(
			FMR_RXMAIN_CH_INFO_CH_FREQ_ADDR, data);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

fmrx_hw_get_channel_freq_exit:
	return err;
}

int fmrx_hw_get_channel_rssi(
		s16 *data)
{
	int err = 0;

	if (0 == data) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmrx_hw_get_channel_rssi_exit;
	}

	/* Read RSSI */
	err = fmtrx_sys_reg_read16(
			FMR_RXMAIN_RSSI_ADDR, data);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_channel_rssi_exit;
	}
	*data = RSSI_TO_DBUV(*data);

fmrx_hw_get_channel_rssi_exit:
	return err;
}

int fmtrx_hw_get_interrupt_status(
		u32 *interrupt_status)
{
	int err = 0;

	if (0 == interrupt_status) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmtrx_hw_get_interrupt_status_exit;
	}

	err = fmtrx_read32_masked(INTSTATUS_ADDR,
						interrupt_status,
						INTSTATUS_INTDED_MASK32 |
						INTSTATUS_BUSERR_MASK32 |
						INTSTATUS_BRK_MASK32 |
						INTSTATUS_SWINT_MASK32 |
						INTSTATUS_HWINT_MASK32);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 32-bit masked failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

fmtrx_hw_get_interrupt_status_exit:
	return err;
}

int fmrx_hw_get_id(
		struct fmr_id *data)
{
	int err = 0;

	if (0 == data) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmrx_hw_get_id_exit;
	}

	err = fmtrx_sys_reg_read32(
			FMRID_ADDR, &data->hw_id);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_id_exit;
	}

	err = fmtrx_sys_reg_read32(
			FMR_RXMAIN_FW_ID_ADDR, &data->fw_id);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_id_exit;
	}

	err = fmtrx_sys_reg_read32(
			FMR_RXMAIN_FW_BUILDTIME_ADDR,
			&data->fw_timestamp);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_id_exit;
	}

	memcpy(data->hw_sw_ver, FMR_HW_SW_ID, sizeof(FMR_HW_SW_ID));

fmrx_hw_get_id_exit:
	return err;
}

int fmrx_hw_get_rds_count_status(
		struct dsp_rds_buf_info *buf_info,
		u16 *group_count,
		u16 *discard_count)
{
	int err = 0;
	u16 group_count_t = 0,
		discard_count_t = 0,
		avail_groups_size = 0;

	/* Validate input arguments */
	if (((0 == group_count) &&
			(0 == discard_count)) ||
			(0 == buf_info)) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmrx_hw_get_rds_count_status_exit;
	}

	/* Fetch number of discarded blocks */
	err = fmtrx_sys_reg_read16(
			FMR_RXMAIN_RDS_DISCARDED_BLOCKS_ADDR,
			&discard_count_t);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_rds_count_status_exit;
	}

	/* Calculate available groups */
	/* Avoid floating point exceptions,
		if buf_info.size will be '0' */
	if (0 != buf_info->size) {
		avail_groups_size =
			(buf_info->fw_write_ptr - buf_info->host_read_ptr +
			buf_info->size) % (buf_info->size);
	}

	if (avail_groups_size == 0 &&
			discard_count_t == 0) {
		/* RDS group buffer is empty */
		group_count_t = 0;
	} else if (avail_groups_size == 0 &&
			discard_count_t != 0) {
		/* RDS group buffer is full */
		group_count_t =
				BYTES_TO_RDS_GROUP_CNT(buf_info->size);
	} else {
		/* Available RDS groups */
		group_count_t =
				BYTES_TO_RDS_GROUP_CNT(avail_groups_size);
	}

	if (0 != group_count)
		*group_count = group_count_t;

	if (0 != discard_count)
		*discard_count = discard_count_t;

fmrx_hw_get_rds_count_status_exit:
	return err;
}

int fmrx_hw_get_rds_groups(
		u16 groups_to_read,
		struct rds_group *rds_groups,
		bool restore_pi_blocks,
		u16 *groups_read)
{
	int err = 0;
	u32 host_offs = 0, read_offs = 0;
	u16 avail_group_count = 0,
		groups_part1 = 0,
		groups_part2 = 0;
	struct dsp_rds_buf_info buf_info = {
				0, 0, 0, 0, 0 };

#ifdef FMR_DEBUG_LVL2
	fmtrx_sys_log
	("%s: %s %d,Requested RDS groups %d!\n",
	FILE, __func__,
	__LINE__, groups_to_read);
#endif

	/* Validate input arguments */
	if ((0 == rds_groups) ||
			(0 == groups_to_read)) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmrx_hw_get_rds_groups_exit;
	}

	/* Get total RDS buffer size, start/end/host_read/write
			address from FW */
	err = fmrx_hw_get_rds_fw_buffer_info(&buf_info);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Get RDS buffer info failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_rds_groups_exit;
	}

#ifdef FMR_DEBUG_LVL2
	fmtrx_sys_log
	("%s: %s %d,RDS Buffer info,Host - 0x%x,Write - 0x%x!\n",
	FILE, __func__,
	__LINE__, buf_info.host_read_ptr,
	buf_info.fw_write_ptr);
#endif

	/* Get available number of RDS groups */
	err = fmrx_hw_get_rds_count_status(
			&buf_info, &avail_group_count,
				0);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Get RDS group count failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		err = -EINVAL;
		goto fmrx_hw_get_rds_groups_exit;
	}

	/* Return if no RDS data is available */
	if (0 == avail_group_count) {
		fmtrx_sys_log
			("%s: %s %d,No RDS data!\n",
			FILE, __func__,
			__LINE__);
		goto fmrx_hw_get_rds_groups_exit;
	}

#ifdef FMR_DEBUG_LVL2
	fmtrx_sys_log
	("%s: %s %d,Available RDS groups - %d!\n",
	FILE, __func__,
	__LINE__, avail_group_count);
#endif

	/* User can't read groups more than what is
			available - Calculate groups that can be
			actually read */
	groups_to_read =
			(groups_to_read > avail_group_count) ?
			avail_group_count : groups_to_read;

#ifdef FMR_DEBUG_LVL2
	fmtrx_sys_log
	("%s: %s %d,No.of RDS groups to be read - %d!\n",
	FILE, __func__,
	__LINE__, groups_to_read);
#endif

	/* Find the next read offset */
	read_offs =
			buf_info.host_read_ptr - buf_info.buf_start;
	buf_info.host_read_ptr =
				buf_info.buf_start +
				(read_offs + FW_RDS_GROUP_SIZE) %
				buf_info.size;

	/* Calculate number of groups from current read
			offset to end of the buffer */
	groups_part1 =
			(buf_info.buf_end - buf_info.host_read_ptr) /
			FW_RDS_GROUP_SIZE;

	/* If groups to be read is more than the available
			groups (determined above),calculate number of
			remaining groups to be read from start of the
			buffer */
	if (groups_to_read > groups_part1)
		groups_part2 = groups_to_read - groups_part1;
	else
		groups_part1 = groups_to_read;

	/* Fetch RDS groups from FW memory */
	err = fmtrx_sys_mem_read(
			buf_info.host_read_ptr,
			(u8 *)rds_groups, groups_part1 *
			FW_RDS_GROUP_SIZE);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,FMR IP Memory read failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_rds_groups_exit;
	}

	/* Fetch remaining groups from the start of the buffer */
	if (groups_part2) {
		/* Fetch RDS groups from FW memory */
		err = fmtrx_sys_mem_read(
				buf_info.buf_start, (u8 *)rds_groups +
				(groups_part1 * FW_RDS_GROUP_SIZE),
				groups_part2 * FW_RDS_GROUP_SIZE);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,FMR IP Memory read failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_hw_get_rds_groups_exit;
		}
	}

	/* Avoid floating point exceptions, if buf_info.size
			will be '0' */
	if (0 != buf_info.size) {
		host_offs = (read_offs +
			(groups_to_read * FW_RDS_GROUP_SIZE)) %
			buf_info.size;
	}

	/* Update HOST read pointer based on the number of
			groups fetched */
	buf_info.host_read_ptr =
			buf_info.buf_start + host_offs;

#ifdef FMR_DEBUG_LVL2
	fmtrx_sys_log
	("%s: %s %d,New host offset - 0x%x!\n",
	FILE, __func__,
	__LINE__, buf_info.host_read_ptr);
#endif

	err = fmtrx_sys_reg_write16(
			FMR_RXMAIN_RDS_HOST_READ_PTR_ADDR,
			buf_info.host_read_ptr);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_rds_groups_exit;
	}

	/* Update the number of groups actually read */
	if (0 != groups_read)
		*groups_read = groups_to_read;

fmrx_hw_get_rds_groups_exit:
	return err;
}

int fmrx_hw_get_rds_pi(
		s16 *data)
{
	int err = 0;
	u16 rds_sync = 0;

	/* Validate input arguments */
	if (0 == data) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmrx_hw_get_rds_pi_exit;
	}

	/* Check if RDS is already in sync */
	err = fmtrx_sys_reg_read16(
			FMR_RXMAIN_RDS_SYNC_ADDR, &rds_sync);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_rds_pi_exit;
	}

	/* RDS is already in sync, just fetch the PI code */
	if (rds_sync) {
		err = fmtrx_sys_reg_read16(FMR_RXMAIN_RDS_PI_ADDR, data);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Read 16-bit failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_hw_get_rds_pi_exit;
		}
	} else {
#ifdef FMR_INTR_MODE
		/* Enable RDS event */
		err = fmtrx_hw_set_interrupts(IR_SWINT1, true);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Set interrupt failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_hw_get_rds_pi_exit;
		}
#endif

		/* Wait for FAST PI interrupt */
		err = fmtrx_sys_wait_for_event(
				FMTRX_RDS_FAST_PI_TIMEOUTS);
		if (0 != err) {
			fmtrx_sys_log
			("%s: %s %d,Wait for Fast PI event failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_hw_get_rds_pi_exit1;
		} else {
			/* Read PI code */
			err = fmtrx_sys_reg_read16(
					FMR_RXMAIN_RDS_PI_ADDR, data);
			if (0 != err) {
				fmtrx_sys_log
					("%s: %s %d,Read 16-bit failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmrx_hw_get_rds_pi_exit1;
			}
		}
	}

fmrx_hw_get_rds_pi_exit1:
#ifdef FMR_INTR_MODE
	{
		int err1 = 0;
		/* Disable RDS event */
		err1 = fmtrx_hw_set_interrupts(IR_SWINT1, false);
		if (0 != err1) {
			fmtrx_sys_log
				("%s: %s %d,Set interrupt failed! %d\n",
				FILE, __func__,
				__LINE__, err1);
		}
	}
#endif
fmrx_hw_get_rds_pi_exit:
	return err;
}

/*
** =============================================================================
**
**				LOCAL FUNCTION DEFINITIONS
**
** =============================================================================
*/
static int fmrx_hw_get_fw_channel_info(
		struct dsp_ch_info *data)
{
	int err = 0;

	if (0 == data) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		err = -EINVAL;
		goto fmrx_hw_get_fw_channel_info_exit;
	}

	err = fmtrx_sys_mem_read(
			FMR_RXMAIN_CH_INFO_CH_FREQ_ADDR,
			(u8 *)data, sizeof(struct dsp_ch_info));
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,FMR IP Memory read failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

fmrx_hw_get_fw_channel_info_exit:
	return err;
}

int fmrx_hw_get_rds_fw_buffer_info(
		struct dsp_rds_buf_info *info)
{
	int err = 0;

	/* Validate input arguments */
	if (0 == info) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n", FILE, __func__);
		err = -EINVAL;
		goto fmrx_hw_get_rds_fw_buffer_info_exit;
	}

	/* Fetch start RDS buffer address - fixed address */
	info->buf_start = FMR_RXMAIN_HOSTIF_BUF_ADDR;
	info->buf_end = FMR_RXMAIN_HOSTIF_BUF_END_ADDR;

	/* Calculate RDS buffer size */
	info->size = info->buf_end - info->buf_start;

	/* Fetch host read pointer which points to
			the last read RDS group */
	err = fmtrx_sys_reg_read16(
			FMR_RXMAIN_RDS_HOST_READ_PTR_ADDR,
			&info->host_read_ptr);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_rds_fw_buffer_info_exit;
	}

	/* Fetch fw write pointer which points to the location
		where FW will start writing the next RDS group */
	err = fmtrx_sys_reg_read16(
			FMR_RXMAIN_RDS_WRITE_PTR_ADDR,
			&info->fw_write_ptr);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_hw_get_rds_fw_buffer_info_exit;
	}

fmrx_hw_get_rds_fw_buffer_info_exit:
	return err;
}

static int fmtrx_send_cmd(
		enum fmtrx_type type,
		struct dsp_cmd_pkt *cmd_pkt,
		u16 cmd_params_size)
{
	int err = 0;
	u8 *cmd_params = 0, offs = 0;
#ifdef FMR_INTR_MODE
	enum interrupt_type ir_type = IR_TRX_CMD;
#endif

	/* Validate input arguments */
	if ((0 == cmd_pkt) || (FMTRX_INVALID <= type)) {
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		err = -EINVAL;
		goto fmtrx_send_cmd_exit;
	}

#ifdef FMR_INTR_MODE
	/* CFG commands (-ve cmd_id) generate CMD2 interrupt */
	if (0 > cmd_pkt->cmd_id)
		ir_type = IR_TRX_CMD2;
#endif

	/* Write command id value to the FW command id address */
	err = fmtrx_sys_reg_write16(
			FMR_RXMAIN_CMD_ID_ADDR, cmd_pkt->cmd_id);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 16-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_send_cmd_err1;
	}

	/* Write command param values to the FW
			command param address */
	cmd_params = (u8 *)&cmd_pkt->cmd_params;
	while (0 != cmd_params_size) {
		err = fmtrx_sys_reg_write16(
				FMR_RXMAIN_CMD_PARAM1_ADDR + offs,
				*((u16 *)(cmd_params + offs)));
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Write 16-bit failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_send_cmd_err1;
		}
		cmd_params_size -= 2;
		offs += 2;
	}

#ifdef FMR_INTR_MODE
	/* Enable mask for command complete interrupt */
	err = fmtrx_hw_set_interrupts(ir_type, true);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d,Enable Command complete interrupt failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_send_cmd_exit;
	}
#endif

	/* Notify FW about new command in the command
			buffer */
	err = fmtrx_write_bit(TRIGSETCTL_ADDR,
			TRIGSETCTL_SWTRIGSET_POS32, 1);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_send_cmd_err1;
	}

	err = fmtrx_sys_wait_for_event(FMTRX_TIMEOUTS);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Wait for FW command complete failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

fmtrx_send_cmd_err1:
#ifdef FMR_INTR_MODE
	{
		int err1 = fmtrx_hw_set_interrupts(ir_type,
					false);
		if (0 != err1) {
			fmtrx_sys_log
			("%s: %s %d,Set Cmd complete interrupt failed! %d\n",
				FILE, __func__,
				__LINE__, err1);
		}
	}
#endif
fmtrx_send_cmd_exit:
	return err;
}

static int fmtrx_hw_set_interrupts(
		enum interrupt_type intmask,
		bool interrupt_enable)
{
	int err = 0;

	/* Select only Dedicated & SW/HW interrupts */
	intmask = intmask &
			(INTMASK_INTMASK_MASK32 |
			INTMASK_INTMASKDED_MASK32);

	/* Enable IRQ lines */
	if (interrupt_enable) {
		err = fmtrx_sys_irq_enable();
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,Enable IRQ failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_hw_set_interrupts_exit;
		}
	}

	/* Write to the Interrupt mask register */
	err = fmtrx_sys_reg_write32(INTMASK_ADDR,
			(interrupt_enable) ? intmask : 0);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_hw_set_interrupts_err1;
	}

	/* Skip IRQ disable path when interrupt 'interrupt_enable'
			is requested */
	if (interrupt_enable)
		goto fmtrx_hw_set_interrupts_exit;

fmtrx_hw_set_interrupts_err1:
	/* Disable IRQ lines */
	{
		int err1 = 0;
		err1 = fmtrx_sys_irq_disable();
		if (0 != err1) {
			fmtrx_sys_log
				("%s: %s %d,Disable IRQ failed! %d\n",
				FILE, __func__,
				__LINE__, err1);
		}
	}
fmtrx_hw_set_interrupts_exit:
	return err;
}

static int fmtrx_write_bit(
		u32 addroffs,
		u16 pos,
		u32 value)
{
	int err = 0;

	err = fmtrx_write_field(addroffs,
				pos, 1, value);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	return err;
}

static int fmtrx_write_field(
		u32 addroffs,
		u16 pos,
		u16 width,
		u32 value)
{
	int err = 0;
	u32 mask_tmp = 0, val_tmp = 0;

	err = fmtrx_sys_reg_read32(addroffs, &val_tmp);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_write_field_exit;
	}

	mask_tmp = ((1 << width) - 1) << pos;
	val_tmp = (val_tmp & (~mask_tmp)) | ((value << pos) & mask_tmp);

	err = fmtrx_sys_reg_write32(addroffs, val_tmp);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Write 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_write_field_exit;
	}

fmtrx_write_field_exit:
	return err;
}

static int fmtrx_read32_masked(
		u32 addroffs,
		u32 *value,
		u32 mask)
{
	int err = 0;
	u32 tmp_mask = 0;

	err = fmtrx_sys_reg_read32(addroffs, &tmp_mask);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Read 32-bit failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}

	if (0 != value)
		*value = tmp_mask & mask;

	return err;
}

/* end of file */
