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
/* This file contains the low level driver interfaces for RX functionality */

/*
** ============================================================================
**
**                              INCLUDE STATEMENTS
**
** ============================================================================
*/
#include <linux/string.h>		/* memcpy */
#include <linux/kernel.h>
#include <spinlock.h>
#include <errno.h>

#include "aud_app_fmr_lld_api_rx.h"	/* LLD RX prototypes */
#include "aud_app_fmr_lld_basic_rw.h"	/* Basic read/write functionality */
#include "fmr_rx_api.h"			/* Firmware interface */

#include "aud_app_fmr_sys_os.h"
/*
** ============================================================================
**
**                                   DEFINES
**
** ============================================================================
*/
/* Max RDS goup cont of firmware ring buffer */
#define FMR_RDS_GRP_LEN	12

/*
** ============================================================================
**
**                           LOCAL DATA DEFINITIONS
**
** ============================================================================
*/

/* Internal state/flag of the LLD */
static u16 rds_buf_start;
static u16 rds_buf_end;
static u16 rds_buf_size;
static u16 fmr_rds_group_cnt;

/*
** ============================================================================
**
**                          LOCAL FUNCTION DEFINITIONS
**
** ============================================================================
*/

/*
 * This function is called to initialize the fmr subsystem. It needs
 * to be called once after booting or a hard reset.
 *  - Setup of mini DSP clock frequency
 *  - Setup of system clock frequency dividers
 *     Preconditions:
 *  - FMR digital power supply enabled.
 *  - FMR clock enabled.
 */
void fmtrx_init_subsystem(void)
{
	/* internal static values */
	rds_buf_start = 0;
	rds_buf_end = 0;
	rds_buf_size = 0;
	fmr_rds_group_cnt = 0;
}

/*
 * FUNCTION: fmtrx_wait_at_least
 *  RETURNVALUE: none
 *  PARAMETERS:
 *     microsecond : microsecond time to wait, maximum input vlaue is 160000.
 *   Wait for at least number of us.
 */
void fmtrx_wait_at_least(u32 usecs)
{
	fmr_sys_busy_wait(usecs);
}

/*
 * Triggers the mini DSP firmware entry point by writing the MINIDSPCTL hw
 * register. This automatically switches on the internal mini DSP clock.
 */
void fmtrx_run_mini_dsp(u16 pc)
{
	/*set PC */
	fmtrx_write_field(MINIDSPCTL_ADDR, MINIDSPCTL_PC_POS32,
			  MINIDSPCTL_PC_WIDTH, pc);

	/* existing master halted state */
	fmtrx_write_bit(MDSPHALTCTL_ADDR, MDSPHALTCTL_MST_RUN_POS32, 1);
}

void fmtrx_download_fw(u8 *addr_off, u16 size, u8 *image)
{
	/* halt MDSP */
	fmtrx_halt_mini_dsp();

	/* set PC to 0 */
	fmtrx_write_field(MINIDSPCTL_ADDR, MINIDSPCTL_PC_POS32,
			  MINIDSPCTL_PC_WIDTH, 0);

	fmr_sys_mem_write(addr_off, image, size);
}

/*
 * Halts the mini DSP. It writes to MINIDSPCTL hw register and forces a hard
 * stop. This function is not safe because it may interrupt firmware in a
 * critical section. o do a safe stop use fmrx_halt_fw() instead.
 *
 */
void fmtrx_halt_mini_dsp(void)
{
	fmtrx_write_bit(MDSPHALTCTL_ADDR, MDSPHALTCTL_MST_HALT_POS32, 1);
}

/* Reads mask bits from INTCTL hardware register. */
u32 fmtrx_get_interrupt_mask(void)
{
	return fmtrx_read32_masked(INTMASK_ADDR,
		INTMASK_INTMASK_MASK32 | INTMASK_INTMASKDED_MASK32);
}

void fmtrx_enable_interrupts(u32 intmask, s32 enb, enum fmtrx_running_ctx ctx)
{
	u16 tmp_msk = 0;
	unsigned long lock_flags = 0;
	static DEFINE_SPINLOCK(slock);

	/* Filter only software interrupts */
	intmask = intmask &
		(INTMASK_INTMASK_MASK32 | INTMASK_INTMASKDED_MASK32);

	if (CTX_THREAD == ctx)
		spin_lock_irqsave(&slock, lock_flags);

	tmp_msk = fmtrx_read32(INTMASK_ADDR);

	if (enb)
		tmp_msk |= intmask;
	else
		tmp_msk &= ~intmask;

	fmtrx_write32(INTMASK_ADDR, tmp_msk);

	if (CTX_THREAD == ctx)
		spin_unlock_irqrestore(&slock, lock_flags);
}

/* Reads interrupt status bits from INTCTL hardware register */
u32 fmtrx_get_interrupt_status(void)
{
	return fmtrx_read32_masked(INTSTATUS_ADDR, INTSTATUS_INTDED_MASK32 |
		INTSTATUS_BUSERR_MASK32 | INTSTATUS_BRK_MASK32 |
		INTSTATUS_SWINT_MASK32 | INTSTATUS_HWINT_MASK32);
}

/* Clears all dedicated and software interrupts.*/
void fmtrx_clear_interrupt(u32 int_clr)
{
	fmtrx_write32(INTSETCTL_ADDR, int_clr << 16);
}

void fmtrx_get_mini_dsp_state(struct fmtrx_mdsp_state *mdsp_state)
{
	u32 val = fmtrx_read32(MINIDSPCTL_ADDR);
	mdsp_state->pc =
		(val & MINIDSPCTL_PC_MASK32) >> MINIDSPCTL_PC_POS32;
	mdsp_state->running =
		(val & MDSPDBG_RUNNING_MASK32) >> MDSPDBG_RUNNING_POS32;
}

/* Copy hardware and firmware identifiers. The hardware id is read from
 * MDSPID hardware register.
 */
void fmrx_get_id(u32 *hw_id, u32 *fw_id, u32 *fw_timestamp, u32 *lld_id)
{
	/* hwid is the whole module/module rev number 16 bits */
	*hw_id = fmtrx_read32(FMRID_ADDR);

	/* fwid is from the fw in the mini DSP */
	*fw_id = fmtrx_read32(FMR_RXMAIN_FW_ID_ADDR);
	*fw_timestamp = fmtrx_read32(FMR_RXMAIN_FW_BUILDTIME_ADDR);

	/* lldid is from the constant value defined in LLD */
	*lld_id = FMR_LLD_ID;
}


/* Reads the actual RSSI value from firmware register (RSSI). */
s16 fmrx_get_rssi(void)
{
	return fmtrx_read16(FMR_RXMAIN_RSSI_ADDR);
}

s32 fmrx_get_freq_offs_int(void)
{
	s32 foffs = 0, foffs_int = 0;

	foffs_int = (s32)fmtrx_read32(FMR_RXMAIN_FREQTRACK_FOFFS_INT_ADDR);
	foffs  = (s32)(foffs_int * 13);
	return foffs >> 15;
}

u16 fmrx_get_pilot_ampl(void)
{
	return fmtrx_read16(FMR_RXMAIN_PILOT_AMPL_HZ_ADDR);
}

s32 fmrx_set_rssi_lna_offsets(const s16 *rssi_lna_gainoffs, s32 intr)
{
	fmrx_send_cmd(FMRX_CMD_CFG_LNA_GAINOFFS, (u16 *)rssi_lna_gainoffs,
		      16, intr);

	/* Wait for command complete event */
	return fmr_sys_wait_for_event(FMR_IR_CMD2_DONE);
}

s32 fmrx_set_rssi_ppf_offsets(const s16 *rssi_ppf_gainoffs, s32 intr)
{
	/* Copy PPF offsets */
	fmrx_send_cmd(FMRX_CMD_CFG_PPF_GAINOFFS, (u16 *)rssi_ppf_gainoffs,
		      16, intr);

	/* Wait for command complete event */
	return fmr_sys_wait_for_event(FMR_IR_CMD2_DONE);
}

void fmrx_set_rssi_other_offs(s16 rssi_other_offset)
{
	fmtrx_write16(FMR_RXMAIN_RSSI_OTHER_OFFS_ADDR, rssi_other_offset);
}

void fmrx_set_output(enum fmtrx_lld_aud_routing route)
{
	fmtrx_write16(FMR_RXMAIN_SR_ADDR, route);
}

/* Totally enable or disable the audio processing block. Enables or disables
 * the AUDIO_EN bit
 */
void fmrx_audio_processing_enable(u8 enable)
{
	if (enable)
		fmtrx_write16(FMR_RXMAIN_AUDIO_EN_ADDR, AUDIO_ENABLED);
	else
		fmtrx_write16(FMR_RXMAIN_AUDIO_EN_ADDR, AUDIO_DISABLED);
}

void fmrx_set_aud_volume_dac(u16 left_volume, u16 right_volume)
{
	/*set Left audio gain */
	fmtrx_write16(FMR_RXMAIN_DAC_GAIN_LEFT_ADDR, left_volume);

	/*set right audio gain */
	fmtrx_write16(FMR_RXMAIN_DAC_GAIN_RIGHT_ADDR, right_volume);
}

void fmrx_set_aud_volume_src(u16 left_volume, u16 right_volume)
{
	/*set left audio gain */
	fmtrx_write16(FMR_RXMAIN_SRC_GAIN_LEFT_ADDR, left_volume);

	/*set right audio gain */
	fmtrx_write16(FMR_RXMAIN_SRC_GAIN_RIGHT_ADDR, right_volume);
}

void fmrx_audio_mute(s32 mute)
{
	fmtrx_write16(FMR_RXMAIN_AUDIO_MUTE_ADDR, mute);
}

void fmrx_set_soft_mute(u8 enable, u16 stepsize, s16 rssi_thres)
{
	if (enable)
		fmtrx_write16(FMR_RXMAIN_SM_EN_ADDR, SOFT_MUT_ENABLED);
	else
		fmtrx_write16(FMR_RXMAIN_SM_EN_ADDR, SOFT_MUTE_DISABLED);

	/* set stepsize of 0-32767 */
	if (0 != stepsize)
		fmtrx_write16(FMR_RXMAIN_SM_STEP_ADDR, stepsize);

	if (0 != rssi_thres)
		fmtrx_write16(FMR_RXMAIN_SM_THR_UP_ADDR, rssi_thres);
}

/*
* Function:... fmrx_set_deemphasis
* Parameters: deem_mode :DEEM_50us, DEEM_75us
* Returns:.... -
*  Set deemphasis time constant. This may only be called at startup.
*/
void fmrx_set_deemphasis(enum fmtrx_lld_aud_routing local_deem_mode_e)
{
	/*Set the deemphasis time constant once during startup */
	fmtrx_write16(FMR_RXMAIN_DEEM_TD_ADDR, local_deem_mode_e);
}

/* Read estimated phase noise from mini DSP memory (PN) */
u16 fmrx_get_phase_noise(void)
{
	return fmtrx_read16(FMR_RXMAIN_PN_ADDR);
}

void fmrx_set_snc_cfg(u8 enable, u16 stepsize, s16 upper_rssi_thres,
	s16 lower_rssi_thres)
{
	if (enable) {
		/* write to FW register to enable SNC */
		fmtrx_write16(FMR_RXMAIN_SNC_EN_ADDR, SNC_ENABLED);
	} else {
		/* write to FW register to disable SNC */
		fmtrx_write16(FMR_RXMAIN_SNC_EN_ADDR, SNC_DISABLED);
	}

	/*write to FW register for stepsize from 0 -32767 */
	fmtrx_write16(FMR_RXMAIN_SNC_STEP_ADDR, stepsize);

	/* write to FW register for upper rssi threshold */
	fmtrx_write16(FMR_RXMAIN_SNC_THR_UP_ADDR, upper_rssi_thres);

	/* write to FW register for lower rssi threshold */
	fmtrx_write16(FMR_RXMAIN_SNC_THR_LO_ADDR, lower_rssi_thres);
}

/* Forces mono or enables stereo. */
void fmrx_force_mono(s32 mono)
{
	/* write to FW register to force mono or enable stereo */
	fmtrx_write16(FMR_RXMAIN_FORCE_MONO_ADDR, mono);
}

/* swap channels */
void fmrx_channel_swap(s32 swap)
{
	/* write to FW register to swap channels. */
	fmtrx_write16(FMR_RXMAIN_SWAP_LR_ADDR, swap);
}

void fmrx_set_audio_config(struct fmtrx_lld_aud_cfg audio_conf)
{
	/*This only affects one FW register FMR_AUDIOGAIN_IIR_PARAM */
	fmtrx_write16(FMR_RXMAIN_AUDIOGAIN_IIR_PARAM_ADDR,
		      audio_conf.audiogain_iir_param);
}

/*
 * Copy the current audio status into struct. It contains
 * - Current volume (VOL),
 * - Output volume  (OVOL)
 * - Soft_mute_level (SM_LEVEL) 16 FMR_BIT signed 0 to 0x7fff
 * - Stereo_blend (SNC_LEVEL) 16 FMR_BIT signed 0 to 0x7fff
 */
void fmrx_get_audio_status(struct fmtrx_lld_aud_state *audio_state)
{
	s16 stereo_register = 0;

	/* get current left channel gain */
	audio_state->gains.audio_gain_l =
		fmtrx_read16(FMR_RXMAIN_AUDIO_GAIN_OUT_L_ADDR);

	/* get current right channel gain */
	audio_state->gains.audio_gain_r =
		fmtrx_read16(FMR_RXMAIN_AUDIO_GAIN_OUT_R_ADDR);

	/* get soft mute level */
	audio_state->sm_level = fmtrx_read16(FMR_RXMAIN_SM_LEVEL_ADDR);

	/* get stereo blend */
	audio_state->snc_level =
		fmtrx_read16(FMR_RXMAIN_SNC_STEREO_LEVEL_LMR_ADDR);

	/* get stereo/mono status */
	stereo_register = fmtrx_read16(FMR_RXMAIN_STEREO_STATUS_ADDR);

	audio_state->is_stereo =
		(STEREO_STATUS_STEREO == stereo_register) ? true : false;

	/* get audio processing block status */
	audio_state->audio_en = fmtrx_read16(FMR_RXMAIN_AUDIO_EN_ADDR);
}

void fmrx_rds_enable(enum fmtrx_lld_rds_en enb)
{
	/* Configure the FW register of RDS enable. */
	fmtrx_write16(FMR_RXMAIN_RDS_MODE_ADDR, enb);
}

void fmrx_rds_reset(void)
{
	u16 host_read_ptr = 0, host_irq_ptr = 0;

	/* Get the info of buffer size, buf_start, buf_end, group_counter */
	rds_buf_start = fmtrx_read16(FMR_RXMAIN_RDS_BUF_START_ADDR);
	rds_buf_end = fmtrx_read16(FMR_RXMAIN_RDS_BUF_END_ADDR);

	/* Buffer size in byte. By default, the group counter is maxmium value
	 * of buffer size in group */
	rds_buf_size = rds_buf_end - rds_buf_start;

	fmr_rds_group_cnt = rds_buf_size / FMR_RDS_GRP_LEN - 1;

	/* Set the read pointer at the last group location */
	host_read_ptr = rds_buf_end - FMR_RDS_GRP_LEN;

	/* Get the interrupt when the buffer is half filled */
	host_irq_ptr = rds_buf_start +
		(((fmr_rds_group_cnt + 1) / 2) * FMR_RDS_GRP_LEN);

	fmtrx_write16(FMR_RXMAIN_RDS_HOST_READ_PTR_ADDR, host_read_ptr);
	fmtrx_write16(FMR_RXMAIN_RDS_HOST_IRQ_PTR_ADDR, host_irq_ptr);
}

void fmrx_rds_set_pi_mode(enum fmtrx_lld_rds_pi mode)
{
	/* Configure the FW register bits */
	fmtrx_write16(FMR_RXMAIN_RDS_FAST_PI_ADDR, mode);
}

void fmrx_rds_get_pi(struct fmrx_rds_pi_code *pi)
{
	u32 rds_pi_addr = 0;

	rds_pi_addr = fmtrx_read32(FMR_RXMAIN_RDS_PI_ADDR);

	pi->pi = (u16)(rds_pi_addr & 0x0000ffff);
	pi->errs = (fmtrx_read16(FMR_RXMAIN_RDS_PI_ERRS_ADDR) & 0x0f);
}

/* Return sync status of RDS module. TRUE, if RDS module is synchronized */
s32 fmrx_rds_get_sync(void)
{
	s32 rds_sync_status = false;
	s32 rds_sync = fmtrx_read16(FMR_RXMAIN_RDS_SYNC_ADDR);

	rds_sync_status = (0 != rds_sync) ? true : false;

	return rds_sync_status;
}
/*
 * Function: fmrx_rds_set_notify_interval
 * setting takes effect in the end of later call of the fmrx_rds_get_groups()
 *
 * Parameter:
 *  - u16 min_free
 *
 * Returns:
 *  - Sets the number of free spaces in FMR-internal group buffer,
 *  - below which a callback occurs.
 *  - If min_free set to 0, buffering is disabled and groups are
 *  - passed as soon as they are received.
 */
void fmrx_rds_set_notify_interval(u16 min_free)
{
	if (min_free == 0)
		fmr_rds_group_cnt = 1;
	else if ((rds_buf_size / FMR_RDS_GRP_LEN - 1) > min_free)
		fmr_rds_group_cnt =
			(rds_buf_size / FMR_RDS_GRP_LEN - 1) - min_free;
	else
		fmr_rds_group_cnt = rds_buf_size / FMR_RDS_GRP_LEN - 1;
}

u16 fmrx_rds_get_group_count(void)
{
	s16 host_read_ptr, fw_write_ptr;
	u32 num_bytes = 0;
	host_read_ptr = fmtrx_read16(FMR_RXMAIN_RDS_HOST_READ_PTR_ADDR);
	fw_write_ptr = fmtrx_read16(FMR_RXMAIN_RDS_WRITE_PTR_ADDR);

#if !defined AUD_APP_HOST_TEST
	num_bytes =
		((fw_write_ptr - host_read_ptr + rds_buf_size) % rds_buf_size);
#endif

	/* Buffer is full */
	if (num_bytes == 0)
		num_bytes = rds_buf_size;

	return num_bytes / FMR_RDS_GRP_LEN - 1;
}

u16 fmrx_rds_get_discard_count(void)
{
	return fmtrx_read16(FMR_RXMAIN_RDS_DISCARDED_BLOCKS_ADDR) /
		FMR_RDS_GRP_LEN;
}

s32 fmrx_rds_get_groups(u16 count, void *rds_buf_desc, s32 restore_pi_blks,
	u16 *valid_groups)
{
	u16 valid_group_num = 0, host_read_ptr = 0, host_irq_ptr = 0;
	u16 fw_write_ptr = 0;
	u16 read_offset = 0, offs = 0;
	struct fmrx_rds_pi_code pi;
	s32 rc = 0;

	pi.pi = 0;
	pi.errs = 0;

	/* First, get the actual number of valid groups in FW ring buffer */
	valid_group_num = fmrx_rds_get_group_count();

	if (valid_group_num <= 0) {
		*valid_groups = 0;
		rc = -EIO;
		goto fmrx_rds_get_groups_err;
	}

	if (count <= valid_group_num)
		valid_group_num = count;

	/* Prepare for address handling */
	host_read_ptr = fmtrx_read16(FMR_RXMAIN_RDS_HOST_READ_PTR_ADDR);
	fw_write_ptr = fmtrx_read16(FMR_RXMAIN_RDS_WRITE_PTR_ADDR);
	host_irq_ptr = fmtrx_read16(FMR_RXMAIN_RDS_HOST_IRQ_PTR_ADDR);

	/* Read out the whole RDS FW ring buffer by IDI DMA channel */
	rc = fmr_sys_mem_read(rds_buf_desc, (void *)(u32)rds_buf_start,
		rds_buf_size);

	if (0 != rc) {
		fmdrv_dbg("error in reading RDS data\n");
		*valid_groups = 0;
		goto fmrx_rds_get_groups_err;
	}

	read_offset = host_read_ptr - rds_buf_start;

	/* Move read pointer, over all valid groups read */
	offs = (read_offset + FMR_RDS_GRP_LEN * valid_group_num) %
		rds_buf_size;
	host_read_ptr = rds_buf_start + offs;
	fmtrx_write16(FMR_RXMAIN_RDS_HOST_READ_PTR_ADDR, host_read_ptr);

	/* Update the HOST_IRQ_PTR according to the group_cnt */
	offs = (offs + fmr_rds_group_cnt * FMR_RDS_GRP_LEN) % rds_buf_size;
	host_irq_ptr = rds_buf_start + offs;
	fmtrx_write16(FMR_RXMAIN_RDS_HOST_IRQ_PTR_ADDR, host_irq_ptr);

	/* Call the get_pi() function before to recover the current PI */
	if (restore_pi_blks)
		fmrx_rds_get_pi(&pi);

	/* the number of groups actually copied from FW buffer into the host
	 * buffer
	 */
	*valid_groups = valid_group_num;

fmrx_rds_get_groups_err:
	return rc;
}


void fmrx_set_event_mask(u16 event_mask)
{
	fmtrx_write16(FMR_RXMAIN_EVENT_EN_ADDR, event_mask);
}

u16 fmrx_get_event_mask(void)
{
	/* get event enable fw register */
	return fmtrx_read16(FMR_RXMAIN_EVENT_EN_ADDR);
}

/*
* Function:... fmrx_get_event_status
* Parameters:
* Returns:.... -
*  Read the current event status (EVENT_STATUS).
*/
u16 fmrx_get_event_status(void)
{
	return fmtrx_read16(FMR_RXMAIN_EVENT_STATUS_ADDR);
}

/*
 * Set thresholds for RSSI, pilot amplitude and frequency offset. If a  value
 * raises above its upper or falls below its lower threshold the corresponding
 * interrupt will be triggered.The struct contains thresholds for:
 *  - rssi
 *  - pilot amplitude
 *  - frequency offset
 */
void fmrx_set_rssi_thresholds(struct fmtrx_lld_ev_thres *rssi_thres)
{
	fmtrx_write16(FMR_RXMAIN_EV_RSSI_THR_UP_ADDR, rssi_thres->rssi_upper);
	fmtrx_write16(FMR_RXMAIN_EV_RSSI_THR_LO_ADDR, rssi_thres->rssi_lower);
}

/*
 *@brief send the cmd message to firmware. usually one interrupt will be
 * triggered by firmware for each cmd
 */
void fmrx_send_cmd(u16 cmd_no, u16 *cmd_param, u8 count, s32 intr)
{
	u16 escape_cnt = 0, offset = 0;
	s16 cmd_id = (s16)cmd_no;

	/* Optionally, check whether the previous cmd execution is executed */
	while (fmtrx_read_bit(TRIGCTL_ADDR, TRIGCTL_STATUS_SW_POS32) &&
	       escape_cnt++ < 100)
		fmtrx_wait_at_least(250);

	/* Clear existing cmd interrupt */
	fmtrx_clear_interrupt(IR_TRX_CMD | IR_TRX_CMD2);

	/* Mask/Unmask cmd done interrupt */
	fmtrx_enable_interrupts(IR_TRX_CMD | IR_TRX_CMD2, intr, CTX_THREAD);

	/* Write cmd ID */
	fmtrx_write16(FMR_RXMAIN_CMD_ID_ADDR, cmd_id);

	if (NULL != cmd_param) {
		/* Copy only needed parameters for specific cmds */
		while (count--) {
			fmtrx_write16(FMR_RXMAIN_CMD_PARAM1_ADDR + offset,
				      *cmd_param);
			offset += 2;
			cmd_param++;
		}
	}

	/* Trigger cmd execution */
	fmtrx_write_bit(TRIGSETCTL_ADDR, TRIGSETCTL_SWTRIGSET_POS32, 1);
}

u16 fmtrx_get_ant_tune_cap(void)
{
	return fmtrx_read16(FMR_RXMAIN_CP_TUNE_ADDR);
}

/* Enable/Disable the IDI transfer in RX/TX  */
void fmtrx_set_idi_xfr(s32 rx, s32 enb)
{
	/* For enable, first config the RX/TX, then enable */
	if (enb)
		fmtrx_write_bit(IDICTL_ADDR, IDICTL_RX_TX_POS32, rx ? 0 : 1);

	fmtrx_write_bit(IDICTL_ADDR, IDICTL_IDI_EN_POS32, enb ? 1 : 0);
}

