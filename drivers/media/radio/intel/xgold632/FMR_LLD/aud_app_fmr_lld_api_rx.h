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
**			INTERFACE DESCRIPTION
**
** ============================================================================
*/

/*
 * @file aud_app_fmr_lld_api_rx.h
 *
 * This file defines the API provided by the FM radio RX low level driver
 *
 */

#ifndef _AUD_APP_FMR_LLD_API_RX_H_
#define _AUD_APP_FMR_LLD_API_RX_H_

/*
** ============================================================================
**
**				INCLUDE STATEMENTS
**
** ============================================================================
*/
#include <types.h>

#include "aud_app_fmr_lld_basic_rw.h"  /* Include types for address and size */
#include "fmr_rx_api.h"

/*
** ============================================================================
**
**				DEFINES
**
** ============================================================================
*/
/* Revision ID of Low Level Driver, upated per every new release.
 * The ID number is stored in Hex format, e.g. version 1.6.0 would be stored in
 * as 0X0160.
 */
#define FMR_LLD_ID		0x0110	/* V1.1.0 */
#define DEV_HARD_LIMIT		32767
#define LOWER_BAND_LIMIT	76000000
#define HIGHER_BAND_LIMIT	108000000

/*
** ============================================================================
**
**			EXPORTED ENUM DEFINITIONS
**
** ============================================================================
*/

/* Enum for DAC and SRC mode */
enum fmtrx_lld_aud_routing {
	/* Route FMR samples to DAC */
	LLD_FMR_AUD_ROUTING_DAC    = 0,

	/* Route FMR samples to SRC (DSP) */
	LLD_FMR_AUD_ROUTING_DSP = 2
};

/* Enum for different De-emphasis types */
enum fmtrx_lld_demp_mode {
	LLD_DEEM_50us_SRC = 0, /* 50us in SRC mode */
	LLD_DEEM_75us_SRC = 1, /* 75us in SRC mode */
	LLD_DEEM_50us_DAC = 2, /* 50us in DAC mode */
	LLD_DEEM_75us_DAC = 3  /* 75us in DAC mode */
};

/* Enum for different RDS states */
enum fmtrx_lld_rds_en {
	LLD_RDS_OFF    = 0, /* Disable RDS */
	LLD_RDS_ON     = 1, /* Enable RDS */
	LLD_RDS_RETAIN = 2  /* Enable RDS in retain mode, i.e. no interrupts
				generated for data */
};

/* Enum for Normal and Fast PI mode in RDS */
enum fmtrx_lld_rds_pi {
	LLD_RDS_NORMAL_PI = 0, /* Regular RDS subscription for PI */
	LLD_RDS_FAST_PI   = 1  /* FAST mode where FW decodes PI and reports */
};

/* Enum for different RDS modes */
enum fmtrx_lld_rds_mode {
	RDS  = 0, /* Radio Data system */
	RBDS = 1, /* Radio Broadcast Data System */
	MMBS = 2  /* MMBS mode */
};
/* Enum for Interrupt vector bit positions in the Interrupt status register */
enum fmtrx_lld_int_pos {
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
	POS_IR_DED3   = 11,
};

/* Enum for different Interrupt vectors */
enum fmtrx_int_vecs {
	IR_HWINT			= 1 << POS_IR_HWINT,
	/* Software Interrupt 0 */
	IR_SWINT0			= 1 << POS_IR_SWINT0,
	IR_RX_RSSI			= 1 << POS_IR_SWINT0,
	IR_TX_ANT_TRACKING  = 1 << POS_IR_SWINT0,
	/* Software interrupt 1 */
	IR_SWINT1			= 1 << POS_IR_SWINT1,
	IR_RX_RDS			= 1 << POS_IR_SWINT1,
	IR_TX_RDS			= 1 << POS_IR_SWINT1,
	/* Software interrupt 2 */
	IR_SWINT2			= 1 << POS_IR_SWINT2,
	IR_RX_RDSSYNC			= 1 << POS_IR_SWINT2,
	/* Software interrupt 3 */
	IR_SWINT3			= 1 << POS_IR_SWINT3,
	IR_RX_PILOT			= 1 << POS_IR_SWINT3,
	/* Software interrupt 4 */
	IR_SWINT4			= 1 << POS_IR_SWINT4,
	IR_RX_JOINT			= 1 << POS_IR_SWINT4,
	IR_TX_JOINT			= 1 << POS_IR_SWINT4,
	/* Hardware break */
	IR_BRK				= 1 << POS_IR_BRK,
	/* Bus error */
	IR_BUSERR			= 1 << POS_IR_BUSERR,
	/* Dedicate interrupt 0 */
	IR_INTDED0			= 1 << POS_IR_DED0,
	IR_TRX_CMD			= 1 << POS_IR_DED0,
	/* Dedicate interrupt 1 */
	IR_INTDED1			= 1 << POS_IR_DED1,
	IR_TRX_CMD2			= 1 << POS_IR_DED1,
	/* Dedicate interrupt 2 */
	IR_INTDED2			= 1 << POS_IR_DED2,
	/* Dedicate interrupt 3 */
	IR_INTDED3			= 1 << POS_IR_DED3,
	IR_TRX_TRACE			= 1 << POS_IR_DED3,
};

/* Enum for different error levels */
enum fmtrx_lld_err_level {
	/* No error happens */
	NE				= 0,
	/* Failure in fmtrx_wait_at_least */
	WAIT_AT_LEAST			= 1,
	/* Failure in sending RX command */
	FMRX_SEND_CMD			= 2,
	/* Failure in sending TX command */
	FMTX_SEND_CMD			= 4,
	/* Failure during reset of event register */
	EVENT_RST			= 16,
	/* Failure in reset of FW module  */
	MODULE_RST_FAIL			= 64,
	/* Failure in enabling of frequency tracking algorithm */
	FREQ_TRACK_EN			= 128,
};

/* Enum for Intermediate frequency types */
enum fmtrx_lld_if_sel {
	FMRX_LLD_IF_275K = 0, /* Intermediate frequency of 275khz */
	FMRX_LLD_IF_525K = 1 /* Intermediate frequency of 525khz */
};

/* Enum of different RX FW states */
enum fmrx_fw_state {
	FMRX_FW_STATE_RES0	 = 0,	/* Reserved state */
	FMRX_FW_STATE_CALIB_RX	 = 2,	/* RX calibration */
	FMRX_FW_STATE_TUNING_RX  = 4,	/* Antenna tuning to receive */
	FMRX_FW_STATE_RECEIVING  = 6,	/* RX receiving state */
	FMRX_FW_STATE_IDLE	 = 8,	/* No reception, but FW running */
	FMRX_FW_STATE_HALTED	 = 10,	/* FW halted */
	FMRX_FW_STATE_RUN_BABS	 = 12,	/* BABS algorithm */
	FMRX_FW_STATE_RC_ALIGN	 = 14	/* RC alignment algorithm */
};

/* Enum for IRQ or Thread context */
enum fmtrx_running_ctx {
	CTX_IRQ,   /* Used in Interrupt context to avoid disable/enable of
			interrupt mask on DBB side */
	CTX_THREAD /* Used in Thread context to disable/enable of interrupt
			mask on DBB side */
};

/*
** ============================================================================
**
**		EXPORTED STRUCTURE DEFINITIONS
**
** ============================================================================
*/
/* Structure to hold FMR mini-DSP state */
struct fmtrx_mdsp_state {
	u16	pc;			/* Program counter */
	s32	running;	/* Current state of DSP - Running or halted */
};

/* Structure to hold Audio gains levels */
struct fmtrx_lld_aud_gain {
	u16 audio_gain_l;		/* Left channel gain */
	u16 audio_gain_r;		/* Right channel gain */
};

/* Structure to hold RSSI offsets */
struct fmtrx_lld_rssi_cfg {
	const s16 *rssi_lna_offs;	/* Pointer to LNA offset table */
	const s16 *rssi_ppf_offs;	/* Pointer to PPF offset table */
	s16 rssi_other_offset;		/* Offset to adjust RSSI value */
};

/* Structure to hold Audio configurations */
struct fmtrx_lld_aud_cfg {
	u16 audiogain_iir_param;/* Volume ramping configuration */
	s16 audio_rms_thr;	/* Saturation threshold */
	s16 audio_rms_inc;	/* Saturation step size */
	s16 audio_rms_max;	/* MAX RMS */
};

/* Structure to hold Volume levels */
struct fmtrx_lld_aud_state {
	struct fmtrx_lld_aud_gain gains; /* Left and Right channel gains */
	u16 sm_level;		/* Soft mute level (0 to 0x7FFF) */
	u16 snc_level;		/* Stereo Noise Cancellation (0 to 0x7FFF) */
	s32 is_stereo;		/* Stereo or mono */
	u8 audio_en;		/* Audio Processing Block State*/
};

/* Structure to hold RDS configuration */
struct fmrx_lld_rds_cfg {
	/* Good blocks needed for synchronization */
	u16 good_blocks;

	/* Bad blocks before sync loss */
	u16 bad_blocks_sync;

	/* Bad blocks accepted during synch search */
	u16 bad_blocks_search;

	/* Mode - RDS or RBDS */
	enum fmtrx_lld_rds_mode  mode;
};

/* Structure to hold PI code information read from FW registers */
struct fmrx_rds_pi_code {
	u16 pi;	/* PI code */
	unsigned res_0:4;/* No. of corrected errors for last PI block */
	unsigned res_1:4;/* No. of corrected errors for PI block before */
	unsigned errs:4; /* No of corrected errors for the most old PI blk */
};

/* Structure to hold RDS data */
struct fmrx_lld_rds_blk {
	u16	rxblock;	/* RDS data */
	u8	blockid;	/* Type of block, i.e. A, B, C */
	u8	errs_corr;	/* Number of corrected errors */
};

/* Structure to hold different threshold levels */
struct fmtrx_lld_ev_thres {
	s16  rssi_upper; /* RSSI upper thres - used for RSSI subscription */
	s16  rssi_lower; /* RSSI lower thres - used for RSSI subscription */
	s16  freq_upper; /* Upper frequency offset */
	s16  freq_lower; /* Lower frequency offset */
};

/* Structure to hold One-shot RSSI data */
struct fmtrx_rssi_data {
	/* RSSI value after oneshot measurement with all offsets applied */
	s16 rssi;
	s16 lna_gain;	/* Value of LNA_GAIN HW register at oneshot time */
	s16 ppf_gain;	/* Value of PPF_GAIN HW register at oneshot time */
	s16 ppf_2nd;	/* Value of PPFCTL_2ND HW register at oneshot time */
};

/* Structure to hold AGC configuration cmd */
struct fmtrx_agc_cmd {
	u16 agc_en;		 /* AGC enable/disable */
	u16 fixed_gain_idx;	 /* AGC gain index incase AGC is disabled */
	u16 reserved1;
	u16 reserved2;
};

/* RF poweron cmd structure */
struct fmtrx_rf_pow_on_cmd {
	u16 ant_type;
	u16 lna_out_gain;
	u16 reg_vdd;
	u16 reserved;
};

/* Configuration parameters for channel tune cmd structure */
struct fmtrx_ch_tune_cmd {
	u32 channel_freq;
	u16 inj_side_sel;
	s16 rssi_threshold;
	u16 force_meas;
};

struct fmtrx_band_cfg_cmd {
	u32 lower_band_limit;	/* lower frequency within the band */
	u32 higher_band_limit; /* higher frequency within the band */
};

/* Configuration parameters for FM channel search cmd */
struct fmtrx_ch_search_cmd {
	u32 ch_start_freq; /* Start freq in kHz. Must be within band limits */
	u32 ch_stop_freq;  /* Stop freq in kHz. Must be within band limits */
	s16 ch_step;		/* Freq increment in kHz. */
	u16 inj_side_sel;	/* Injection side */
	s16 rssi_thres;	/* RSSI threshold */
	s16  pn_thres;	/* pn threshold */
	u16 force_meas;	/* Force even unnecessary measurements */
};


/* Channel search status */
enum fmrx_ch_search_status {
	FMRX_CH_SEARCH_STAT_OK,
	FMRX_CH_SEARCH_STAT_THRES_FAIL,
	FMRX_CH_SEARCH_STAT_FAIL,
	FMRX_CH_SEARCH_STAT_INVALID,
};

/* Configuration for FM Radio digital clock source */
struct fmrx_clk_sel_cmd {
	u16 clk_sel;
	u16 reserved1;
	u16 reserved2;
	u16 reserved3;
};

/* Configuration for RSSI Channel offsets */
struct fmrx_rssi_ch_offs_cmd {
	u32 freq_split0;   /* First frequency split in [kHz] */
	u32 freq_split1;
	u32 freq_split2;
	u32 freq_split3;
	u32 freq_split4;
	u16 rssi_offset0;  /* First RSSI offset in [dB*0.25] */
	u16 rssi_offset1;
	u16 rssi_offset2;
	u16 rssi_offset3;
	u16 rssi_offset4;
	u16 rssi_offset5;
};

/* Configuration for clock frequency of the mini DSP subsystem. */
struct fmrx_sysclk_cmd {
	u16 minidsp_clk;
	u16 reserved1;
	u16 reserved2;
	u16 reserved3;
};

/* Configuration for Cp init values for Antenna tuning */
struct fmrx_ant_cpinit_cmd {
	u16 cp_init0;
	u16 cp_init1;
	u16 cp_init2;
	u16 cp_init3;
	u16 cp_init4;
	u16 cp_init5;
	u16 cp_init6;
	u16 cp_init7;
	u16 cp_init8;
	u16 cp_init9;
	u16 cp_init10;
	u16 cp_init11;
	u16 cp_init12;
	u16 cp_init13;
	u16 cp_init14;
	u16 cp_init15;
};

/* good/bad block configuration for RDS processing */
struct fmrx_rds_cfg_cmd {
	/* Good blocks needed for synch */
	u16 rds_sync_cnt_good;

	/* Bad blocks accepted during synch search */
	u16 rds_sync_cnt_bad;

	/* Bad blocks accepted during synchronization */
	u16 rds_track_cnt_bad;
};

/*
** ============================================================================
**
**						EXPORTED DATA DEFINITIONS
**
** ============================================================================
*/

/*
** ============================================================================
**
**				 EXPORTED FUNCTION DECLARATIONS
**
** ============================================================================
*/

/*
 * Wait for at least number of us. This wait operation is performed by waiting
 * for an interrupt from FMR chip.
 *
 * @param  mircosecond			Time in microseconds
 *
 * @return void
 */
void fmtrx_wait_at_least(u32 mircosecond);

/*
 * This function is called to initialize the fmr subsystem. It needs to be
 * called once after booting or a hard reset.
 *  - Setup of mini DSP clock frequency
 *  - Setup of system clock frequency dividers
 *     Preconditions:
 *  - FMR digital power supply enabled.
 *  - FMR clock enabled.
 *
 * @param  interm_freq_sel  Intermediate frequency (525khz or 275khz)
 *
 * @return void
*/
void fmtrx_init_subsystem(void);

/*
 * Triggers the mini DSP firmware entry point by writing the MINIDSPCTL hw
 * register. This automatically switches on the internal mini DSP clock.
 * After triggering the mini DSP at its firmware entry point first the
 * firmware initialization routine is executed which resets some variables and
 * HW registers.
 * Afterwards it automatically proceeds to run the receiver main loop.
 * Precondition:
 *  - mini DSP is halted
 *  - a valid frequency is set for receiving
 *  - RF is powered on and calibrated, and initial frequency is programmed
 *  - audio options, audio output and deemphasis are configured
 *  - RSSI configured
 *  - RDS configured
 *
 * @param  pc				Program counter for FW to jump to
 *
 * @return void
*/
void fmtrx_run_mini_dsp(u16 pc);

/*
 * Halts the mini DSP. It writes to MINIDSPCTL hw register and forces a hard
 * stop. This function is not safe because it may interrupt firmware in a
 * critical section.  To do a safe stop use fmrx_halt_fw() instead.
 *
 * @param  void
 *
 * @return void
*/
void fmtrx_halt_mini_dsp(void);

/*
 * Reads mask bits from INTCTL hardware register.
 *
 * @param  void
 *
 * @return void
*/
u32 fmtrx_get_interrupt_mask(void);

/*
 * Reads interrupt status bits from INTCTL hardware register.
 *
 * @param  void
 *
 * @return u32			 Value from Interrupt status register
*/
u32 fmtrx_get_interrupt_status(void);

/*
 * Clears all the interupts by register INTSETCTL. It also tries to clear the
 * BUSERR and HWINT bits by resetting the bus address generating the error or
 * clearing the appropriate trigger.
 *
 * @param  int_clr   Interrupt bits to clear
 *
 * @return void
 */
void fmtrx_clear_interrupt(u32 int_clr);

/*
 * Enable/Disable interrupts by INTMASK.
 *
 * @param  intmask	Bitfield of interrupt masks
 * @param  enb		Disable/enable interrupt mask
 * @param  ctx		Running context of the caller, thread or irq
 *
 * @return void
 */

void fmtrx_enable_interrupts(u32 intmask, s32 enb, enum fmtrx_running_ctx ctx);

/*
 * Copies the following fields from MINIDSPCTL hardware register into a struct.
 * @param  mdsp_state			Pointer to the DSP state
 *
 * @return void
 */
void fmtrx_get_mini_dsp_state(struct fmtrx_mdsp_state *mdsp_state);

/*
 * Downloads a firmware image to the mini DSP code memory.
 *
 * @param  address	Destination address in mini DSP code memory
 * @param  size		Size of image in bytes
 * @param  image	Pointer to the code image
 * Note: FMR submodule is from 0xE1600000 - 0xE16FFFFF (AHB_PER1 subsystem)
 * MiniDSP RAM from 0xE1600000 - 0xE1603FFF (ES1.x)
 *				0xE1602FFF (ES2 or above)
 *
 * @return void
 */
void fmtrx_download_fw(u8 *addr_off, u16 size, u8 *image);

/*
 * Copy hardware and firmware identifiers. The hardware id is read from MDSPID
 * hardware register.
 *
 * @param  hw_id	Pointer to a address containing FMR_HW_ID
 * @param  fw_id	Pointer to a address containing FMR_FW_ID
 * @param  fw_timestamp	Pointer to FW RX binary timestamp
 * @param  lldid	Pointer to a address containing FMR_LLD_ID
 *
 * @return void
 */
void fmrx_get_id(u32 *hw_id, u32 *fw_id, u32 *fw_timestamp, u32 *lld_id);

/*
 * Reads the actual RSSI value from firmware register (RSSI).
 *
 * @param  void
 *
 * @return s16				RSSI value read from FW register
 */
s16 fmrx_get_rssi(void);

/*
 * Reads actual frequency offset from mini DSP memory (INTFOFFS).
 *
 * @param  void
 *
 * @return s32				Frequency offset read from FW register
 */
s32 fmrx_get_freq_offs_int(void);

/*
 * Reads pilot amplitude from mini DSP memory (PILOT).
 *
 * @param  void
 *
 * @return s16				Pilot amplitude read from FW register
 */
u16 fmrx_get_pilot_ampl(void);

/*
 * Sets values used for signal measurement. This may be set at start-up only.
 * The gain offset values consist of the LNA and PPF gain at the different gain
 * settings, but also a constant system-dependent gain setting.
 *
 * @param  rssi_conf struct containing RSSI measurement configuration
 * @param  intr	true, to generate interrupt after FW executes the offset copy
 *
 * @return void
 */
s32 fmrx_set_rssi_lna_offsets(const s16 *rssi_lna_gainoffs, s32 intr);

/*
 * Sets values used for signal measurement. This may be set at start-up only.
 *
 * @param  rssi_conf	structcontaining RSSI measurement configuration
 * @param  intr	true, to generate interrupt after FW executes the offset copy
 *
 * @return void
 */
s32 fmrx_set_rssi_ppf_offsets(const s16 *rssi_ppf_gainoffs, s32 intr);


/*
 * Configurates the RSSI Other offset.
 *
 * @param  rssi_other_offs  RSSI offset adjustment.
 *
 * @return void
 */
void fmrx_set_rssi_other_offs(s16 rssi_other_offs);


/*
 * Set up samplerate and routing of audio output.
 * This function may only be called once before the mini DSP is triggered,
 * typically at startup or after a hard reset.
 * Writes to SR firmware register.
 * Precondition:
 *   - mini DSP not running
 *
 * @param  routing	FMR_AUD_PATH_DAC - direct connection to DAC
 *			AUDIOOUT_ZBUS_48 - 48kHz via Z-bus
 *
 * @return void
 */
void fmrx_set_output(enum fmtrx_lld_aud_routing route);

/*
 * Enables/Disables the Audio processing block in FW
 * If enable is set to true, AUDIO_EN bit is set to 1 and audio
 * processing block is enabled. false, otherwise.
 *
 * @param  enable			true or false
 *
 * @return void
 */
void fmrx_audio_processing_enable(u8 enable_audio);

/*
 * Set up volume of audio output for different channel in DAC router mode.
 * If volume is set to 0 the audio part is not switched off - this can be done
 * via fmrx_audio_mute(). Writes to VOL firmware register.
 *
 * @param  left_volume	 Left channel gain (Linear scale from  0 to 32767).
 * @param  right_volume  Right channel gain (Linear scale from  0 to 32767).
 *
 * @return void
 */
void fmrx_set_aud_volume_dac(u16 left_volume, u16 right_volume);

/*
 * Set up volume of audio output for different channel in SRC router mode.
 * If volume is set to 0 the audio part is not switched off - this can be done
 * via fmrx_audio_mute().
 * Writes to VOL firmware register.
 *
 * @param  left_volume	Left channel gain (Linear scale from 0 to 32767).
 * @param  right_volume Right channel gain (Linear scale from 0 to 32767).
 *
 * @return void
 */
void fmrx_set_aud_volume_src(u16 left_volume, u16 right_volume);


/*
 * Disables audio processing (AUDIO_EN).
 *
 * @param  mute		true disables audio processing, false enables it.
 *
 * @return void
 */
void fmrx_audio_mute(int mute);

/*
* Enables signal level dependent reduction of output volume. If RSSI level
* falls below rssi_thres the soft mute will reduce the output volume.
*
* @param  enable	true enables soft mute, false disables soft mute
* @param  stepsize	From 0 to 32767
* @param  rssi_thres	RSSI level below which the soft mute takes effect
*
* @return void
*/
void fmrx_set_soft_mute(u8 enabled, u16 stepsize, s16 rssi_thres);

/*
 * Set deemphasis time constant. This may only be called at startup.
 *
 * @param  deem_mode			DEEM_50us, DEEM_75us
 *
 * @return void
 */
void fmrx_set_deemphasis(enum fmtrx_lld_aud_routing deem_mode_e);


/*
 * Read estimated phase noise from mini DSP memory (PN).
 *
 * @param  void
 *
 * @return void
 */
u16 fmrx_get_phase_noise(void);

/*
 * Enables signal level dependent blend from stereo to mono.
 *
 * @param  enabled true - enables SNC between the upper and lower rssi thres
 *		false - disables SNC. Thresholds are used for hard switch
 * @param  stepsize	Blend per 0.25*dB. From 0 to 32767
 * @param  upper_rssi_thres Unit is same as RSSI level
 * @param  lower_rssi_thres Unit is same as RSSI level
 *
 * @return void
 */
void fmrx_set_snc_cfg(u8 enable, u16 stepsize, s16 upper_rssi_thres,
	s16 lower_rssi_thres);

/*
 * Forces mono or enables stereo.
 *
 * @param  mono	true set FMR working in mono mode; false stereo mode
 *
 * @return void
 */
void fmrx_force_mono(int mono);

/*
 * Write to FW register to swap the left and right channels
 *
 * @param  swap				true to swap, false to disable
 *
 * @return void
 */
void fmrx_channel_swap(int swap);

/*
 * Write Audio configuration like IIR and Saturation settings.
 *
 * @param  audio_conf	Audio configuration
 *
 * @return void
 */
void fmrx_set_audio_config(struct fmtrx_lld_aud_cfg audio_conf);

/*
 *  Copy the current audio status into struct. It contains
 *  - Current volume (VOL),
 *  - Output volume  (OVOL)
 *  - Soft_mute_level (SM_LEVEL) 16 FMR_BIT signed 0 to 0x7fff
 *  - Stereo_blend (SNC_LEVEL) 16 FMR_BIT signed 0 to 0x7fff
 *
 * @param  audio_state			Pointer to volume configuration
 *
 * @return void
 */
void fmrx_get_audio_status(struct fmtrx_lld_aud_state *audio_state);

/*
 * Enables/Disables RDS.Setup RDS hardware block and RDS processing in fw.
 * The driver initializes its internal read pointers to the RDS ring buffer.
 *
 * @param  enable			RDS_OFF / RDS_ON / RDS_RETAIN
 *
 * @return void
 */
void fmrx_rds_enable(enum fmtrx_lld_rds_en enable_rds);


/*
 * Resets the RDS module. It clears the RDS hardware block and firmware module.
 * @param  void
 *
 * @return void
 */
void fmrx_rds_reset(void);

/*
 * Enables / Disables fast PI mode in hardware block and firmware. If fast
 * PI mode is enabled, each time a block of type A or C' is received and the
 * current PI is considered as valid, an interrupt is asserted.
 *
 * @param  mode				RDS_NORMAL_PI / RDS_FAST_PI
 * @return void
 */
void fmrx_rds_set_pi_mode(enum fmtrx_lld_rds_pi mode);

/*
 * Reads out the current PI and error information from firmware registers. The
 * PI value and error information is decoded into a struct.
 * @param  pi				PI code read from FW
 *
 *  @return void
 */
void fmrx_rds_get_pi(struct fmrx_rds_pi_code *pi);

/*
 * Return sync status of RDS module.
 *
 * @param  void
 *
 * @return sync status.
 */
int fmrx_rds_get_sync(void);

/*
 * Sets the number of decoded RDS groups after which an interrupt to the host
 * should be asserted. The setting takes effect after the call of
 * fmrx_rds_get_groups().
 * @param  group_cnt Generate interrupt when specified number of groups is
 * reached
 *
 * @return void
 */
void fmrx_rds_set_notify_interval(u16 group_cnt);

/*
 * Returns the number of RDS groups in the ring buffer which have not been read
 * by host
 *
 * @return u16			Number of groups
 */
u16 fmrx_rds_get_group_count(void);

/*
 * Returns the number of RDS groups discarded.
 *
 * @param  void
 *
 * @return u16			 Number of groups
 */
u16 fmrx_rds_get_discard_count(void);

/*
 * Reads out the specified number of groups from RDS data buffer and copies
 * them into the data structures of host buffer.
 *
 * @param  count	Number of groups to be read
 * @param  blocks	Buffer to store the RDS blocks
 * @param  restore_pi_blks If true, type A block will be restored
 * @param  valid_groups valid_groups available in the buff
 *
 * @return s32		error code
 */
s32 fmrx_rds_get_groups(u16 count, void *rds_buf_desc, s32 restore_pi_blks,
	u16 *valid_groups);

/*
 * Sets the event mask for RX. Only interrupts for the events set to 1 in the
 * mask will be triggered.
 *
 * @param  event_mask			Mask value to enable certain events
 *
 * @return void
 */
void fmrx_set_event_mask(u16 event_mask);

/*
 * Read the current event mask of RX. Only fw/driver related.
 *
 * @param  void
 *
 * @return u16			 Event mask
 */
u16 fmrx_get_event_mask(void);

/*
 * Read the current event status of RX (EVENT_STATUS).
 *
 * @param  void
 *
 * @return u16			 Event status
 */
u16 fmrx_get_event_status(void);

/*
 * Enables or disables RX firmware events
 *
 * @param  mask		Bitfield of event mask
 * @param  en		true : enables events, false : disables events
 * @param  ctx		Caller context, thread or irq
 *
 * @return void
 */
void fmrx_enable_events(u32 mask, int en, enum fmtrx_running_ctx ctx);

/*
 * Set thresholds for RSSI
 *
 * @param  rssi_thres			Pointer to RSSI thresholds
 *
 * @return void
 */
void fmrx_set_rssi_thresholds(struct fmtrx_lld_ev_thres *rssi_thres);

/* FW Interface */

/*
 * Send the cmd message to RX firmware. usually one interrupt will be
 * triggered by firmwar for each cmd
 *
 * @param  cmd_id    cmd ID
 * @param  cmd_param cmd parameters, maximally 16 16-bit half word
 * @param  count     Actual number of cmd parameters carried by cmd_param
 * @param  intr      Whether the complete of this cmd triggers int to host
 *
 * @return void
 */
void fmrx_send_cmd(u16 cmd_id, u16 *cmd_param, u8 count, int intr);

/*
 * Read actual state of firmware top level state machine.
 *
 * @param  void
 *
 * @return enum fmrx_fw_state		State of RX firmware
 */
enum fmrx_fw_state fmrx_get_state(void);

/*
 * Disable/Enable IDI transfer in RX/Tx
 *
 * @param  rx				true, for RX; false, for TX
 * @param  enb				Enable/Disable IDI handshake
 *
 * @return void
 */
void fmtrx_set_idi_xfr(int rx, int enb);

/*
 * Get CP tune value from FW register
 *
 * @param  void
 *
 * @return u16 CP tune value
 */
u16 fmtrx_get_ant_tune_cap(void);

#endif	/* _AUD_APP_FMR_LLD_API_RX_H_ */

