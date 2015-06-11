/*
 * Component: XGOLD DSP Audio Driver
 *
 * Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Contributor(s):
 */

#ifndef _DSP_AUDIO_DRIVERIF_H
#define _DSP_AUDIO_DRIVERIF_H

#include "bastypes.h"

struct dsp_audio_device;

/** @addtogroup DSP_drv_ispec Interface Specification for DSP Driver
	to other driver modules */
/*@{*/

/**
    @defgroup dsp_drv_dsp_cmds Defines, types and functions for
	implementing the DSP command interface used by other driver modules
    @section dsp_drv_dsp_cmds Defines, types and functions for
	implementing the DSP command interface used by other driver modules
*/


/** \brief Maximum number of data words for audio DSP command. */
#define CMD_AUD_DATA_WORDS          33

/**
    \brief dsp_aud_cmd_pipes - Micro controler audio related commands to
	shared memory
*/
struct dsp_aud_cmd_pipes {
	U16  cmd_0_data[CMD_AUD_DATA_WORDS]; /**< Command pipe 1 */
	U16  cmd_1_data[CMD_AUD_DATA_WORDS]; /**< Command pipe 2 */
	U16  cmd_2_data[CMD_AUD_DATA_WORDS]; /**< Command pipe 3 */
};

/**
    \brief dsp_aud_cmd_header - header
*/
struct dsp_aud_cmd_header {
	U16    command_id;
	U16    command_len;
	char   data[0];
};

/**
    \brief dsp_rw_shm_header - header
*/
struct dsp_rw_shm_header {
	U16    word_offset;
	U16    len_in_bytes;
	char   data[0];
};

/**
    \brief dsp_aud_cmd_data - header
*/
struct dsp_aud_cmd_data {
	U16    command_id;
	U16    command_len;
	U16   *p_data;
};

/**
    \brief dsp_rw_shm_data - header
*/
struct dsp_rw_shm_data {
	U16    word_offset;
	U16    len_in_bytes;
	U16    *p_data;
};
/*--------------------------------------------------*/
/*	Declarations for accessing communication flags	*/
/*--------------------------------------------------*/

/*
	* Enumeration holding all the communication flags between the IA
	* and the Frame Based Audio DSP.
	*
	* The defined communiactions flags are used together with the functions
	* dsp_set_audio_dsp_communication_flag(enum dsp_irq_comm_flag comm_flag)
	* dsp_reset_audio_dsp_communication_flag(
			enum dsp_irq_comm_flag comm_flag)
	* dsp_read_audio_dsp_communication_flag(
			enum dsp_irq_comm_flag comm_flag)
 */
enum dsp_irq_comm_flag {
	DSP_IRQ_COMM_FLAG_0 = 0,
	DSP_IRQ_COMM_FLAG_1,
	DSP_IRQ_COMM_FLAG_2,
	DSP_IRQ_COMM_FLAG_3,
	DSP_IRQ_COMM_FLAG_4,
	DSP_IRQ_COMM_FLAG_5,
	DSP_IRQ_COMM_FLAG_6,
	DSP_IRQ_COMM_FLAG_7,
	DSP_IRQ_COMM_FLAG_8,
	DSP_IRQ_COMM_FLAG_9,
	DSP_IRQ_COMM_FLAG_10,
	DSP_IRQ_COMM_FLAG_11,
	DSP_IRQ_COMM_FLAG_12,
	DSP_IRQ_COMM_FLAG_13,
	DSP_IRQ_COMM_FLAG_14,
	DSP_IRQ_COMM_FLAG_15,
	DSP_IRQ_COMM_FLAG_END
};

/**
    \brief Enumeration holding all the interrupts the MCU can give
	to the Audio DSP.
*/
enum dsp_irq_no {
	DSP_IRQ_0 = 0,
	DSP_IRQ_1,
	DSP_IRQ_2,
	DSP_IRQ_3,
	DSP_IRQ_4,
	DSP_IRQ_5,
	DSP_IRQ_6,
	DSP_IRQ_7,
	DSP_IRQ_8,
	DSP_IRQ_9,
	DSP_IRQ_10,
	DSP_IRQ_11,
	DSP_IRQ_12,
	DSP_IRQ_13,
	DSP_IRQ_14,
	DSP_IRQ_15,
	DSP_FBA_IRQ_0,
	DSP_FBA_IRQ_1,
	DSP_FBA_IRQ_2,
	DSP_FBA_IRQ_3,
	DSP_IRQ_END
};

/**
 * @enum DSP_ERROR_CODES
 * Enumeration of possible error codes used in the dsp driver. A negative value
 * always means error.
 * @typedef enum DSP_ERROR_CODES dsp_err_code
 * Enum type of DSP_ERROR_CODES
 * @ingroup dsp_modem_dsp_irq
 */
enum dsp_err_code {
	DSP_SUCCESS              =  0,   /**< Call succeded */
	DSP_ERR_INVALID_MODULE   = -1,   /**< Invalid module selected */
	DSP_ERR_INVALID_IRQ      = -2,   /**< Interrupt not supported*/
	DSP_ERR_INVALID_MODE     = -3,   /**< Interrupt mode not supported */
	DSP_ERR_INVALID_FILTER   = -4,   /**< Interrupt filter not supported */
	DSP_ERR_INVALID_HANDLE   = -5,   /**< Supplied handle invalid */
	DSP_ERR_INVALID_REQUEST  = -6,   /**< Request invalid */
	DSP_ERR_RES_IN_USE       = -7,   /**< Resource already in use */
	DSP_ERR_NOT_ALLOC        = -8,   /**< Resource not allocated */
	DSP_ERR_INVALID_PTR      = -9,   /**< Invalid pointer provided */
	DSP_ERR_INVALID_FUNC_PTR = -10,   /**< Invalid function pointer provided */
	DSP_ERR_COMMAND_FAILURE  = -11   /**< Invalid function pointer provided */
};

/*
 *
   \brief DSP_AUDIO_CMD holds the possible DSP commands for the Audio DSP
*/
enum dsp_aud_cmds {
	DSP_AUDIO_CMD_VB_HW_AFE    = 2,  /* start or to stop the firmware */
	DSP_AUDIO_CMD_IDLE         = 14, /* set dsp subsystem to idle state */
	DSP_SBA_VB_HW_I2S2         = 22, /* start or to stop the I2S2 hw IF */
	DSP_AUD_SET_SWM_AFE_OUT    = 27, /* set afe out switch matrix path */
	DSP_AUD_SET_SWM_PCM_OUT    = 32, /* set pcm out switch matrix path */
	DSP_AUD_SET_GAIN_HW        = 33, /* set dsp scheduler gains */
	DSP_AUD_SET_GAIN_TIMECONST = 36, /* set dsp gains timeconstant */
	DSP_AUD_SET_IIR            = 39, /* set the dsp IIR filter coeff */
	DSP_AUD_PCM1_PLAY          = 58, /* start and stop PCM playback */
	DSP_AUD_PCM_REC            = 59, /* start and stop a PCM recording */
	DSP_AUD_SPEECH_PROBE       = 66, /* config/start/stop a speech I/O */
	DSP_AUD_SET_SWM_MIX_MATRIX = 77, /* set the mix matrix coefficients */
	DSP_AUD_VB_HW_I2S1         = 86,
	DSP_AUD_HW_PROBE           = 100,
	DSP_AUD_PCM2_PLAY          = 106, /* start and stop PCM2 playback */
	DSP_AUD_SPEECH_PATH        = 26
};

/**
 * \brief dsp_add_fba_audio_msg_2_dsp \n
 *
 * Send an audio dsp command immediately to dsp command pipe #2
 * without buffering.
 *
 * \param dsp_device
 * \param word msg_id
 * \param word msg_length
 * \param word* p_msg_par
 * \return None
 *
 */
enum dsp_err_code dsp_add_audio_msg_2_dsp(struct dsp_audio_device *dsp,
	U16 msg_id,
	U16 msg_length,
	U16 *p_msg_par);

/**
 * \brief dsp_add_audio_msg_2_dsp_cmd_pipe_1 \n
 *
 * Send an audio dsp command issued in atomic context to dsp
 * command pipe #1 without buffering.
 *
 * \param dsp_device
 * \param word msg_id
 * \param word msg_length
 * \param word* p_msg_par
 * \return None
 *
 */
enum dsp_err_code dsp_add_audio_msg_2_dsp_cmd_pipe_1(
	struct dsp_audio_device *dsp,
	U16 msg_id,
	U16 msg_length,
	U16 *p_msg_par);

/**
 * @brief Activate a previously allocated interrupt.
 *
 * When called the dsp will forward the interupts to the ICU.
 *
 * @param dsp       Pointer to the dsp audio device struct
 * @param irq_no    Interrupt number
 *
 * @return       Error code from enum DSP_ERROR_CODES.
 */
enum dsp_err_code dsp_audio_irq_activate(struct dsp_audio_device *dsp,
			enum dsp_irq_no);

/**
 * @brief De-activate a previously allocated interrupt.
 *
 * When called the fuction will mask the interrupt.
 *
 * @param dsp       Pointer to the dsp audio device struct
 * @param irq_no    Interrupt number
 *
 * @return       Error code from enum DSP_ERROR_CODES.
 */
enum dsp_err_code dsp_audio_irq_deactivate(struct dsp_audio_device *dsp,
			enum dsp_irq_no);

/**
 *
 * @brief Acknowledge interrupt interrupt in hardware
 *
   @param dsp       pointer to dsp devicer
 * @param irq_no    Interrupt number
 *
 * @return       Error code from enum DSP_ERROR_CODES.
 */
enum dsp_err_code dsp_audio_irq_ack(struct dsp_audio_device *dsp,
			enum dsp_irq_no irq_no);
/**
  \brief Sets a communication flag between the MCU and frame based audio DSP.
  \param dsp_device pointer to dsp device
  \param DSP_AUDIO_FBA_DSP_COMM_FLAG comm_flag: Which communication flag to set
  \return none
*/
void dsp_set_audio_dsp_communication_flag(struct dsp_audio_device *dsp,
			enum dsp_irq_comm_flag);

/**
  \brief Resets a communication flag between the MCU and frame based audio DSP.
  \param dsp_device pointer to dsp device
  \param dsp_irq_comm_flag comm_flag: Which communication flag to reset
  \return none
*/
void dsp_reset_audio_dsp_communication_flag(struct dsp_audio_device *dsp,
			enum dsp_irq_comm_flag);

/**
  \brief Reads the value of a communication flag between the MCU and
   audio DSP.
  \param dsp_device pointer to dsp device
  \param DSP_AUDIO_DSP_COMM_FLAG comm_flag: Which communication flag to read
  \return The value of the specified communication flag
*/
int dsp_read_audio_dsp_communication_flag(struct dsp_audio_device *dsp,
			enum dsp_irq_comm_flag);
/*@}*/

/**
    @defgroup dsp_drv_irq Defines, types and functions for implementing the
				DSP interrupt registration interface.
    @section dsp_drv_irq  Defines, types and functions for implementing the
				DSP interrupt registration interface.
*/
/*@{*/
/* Use this construction as the same IF is defined in dsp_driverif.h
   The same interface are to be used by protocol stack and driver modules. */

/**
  @brief Initialization of the dsp audio sub-module

  @param None

  @return enum dsp_err_code error

 */
enum dsp_err_code dsp_audio_init(struct list_head *dsp_linked_list);

/**
  @brief Register DSP INT interrupt function and enable  interrupt

  @param DSP_AUDIO_TYPE: Indication of the dsp type.
  @param U16: Command id.
  @param U16: Command length.
  @param U16*: Pointer to command parameters.

  @return enum dsp_err_code error
 */
enum dsp_err_code dsp_audio_cmd(
	U16    command_id,
	U16    command_len,
	U16    *p_command);

/**
  @brief Register DSP INT interrupt function and enable  interrupt

  @param DSP_AUDIO_TYPE: Indication of the dsp type.
  @param U16: Command id.
  @param U16: Command length.
  @param U16*: Pointer to command parameters.

  @return enum dsp_err_code error
 */
enum dsp_err_code dsp_audio_cmd_pipe_1(
	U16    command_id,
	U16    command_len,
	U16    *p_command);

/**
  @brief Reads requested no of bytes from the shared memory

  @param U16*: Destination to copy the data read
  @param U16:  word offset in shared memory to read
  @param U16:  No of bytes to read

  @return enum dsp_err_code error
 */
enum dsp_err_code dsp_audio_read_shm(
	struct dsp_audio_device *dsp_dev,
	U16    *p_dest,
	U16    shm_word_offset,
	U16    len_in_bytes
);

/**
  @brief Returns the base address of the DSP shared memory

  @return base address of DSP shmem

 */
dma_addr_t dsp_get_audio_shmem_base_addr(struct dsp_audio_device *dsp_dev);


/**
  @brief Writes requested no of bytes to the shared memory

  @param U16*: Source to copy the data from
  @param U16:  word offset in shared memory to write
  @param U16:  No of bytes to write

  @return enum dsp_err_code error

 */
enum dsp_err_code dsp_audio_write_shm(
	struct dsp_audio_device *dsp_dev,
	U16   *p_src,
	U16    shm_word_offset,
	U16    len_in_bytes
);

/**
 * List of all lisr callbacks to be registered
 */
enum dsp_lisr_cb {
	DSP_LISR_CB_TONE,
	DSP_LISR_CB_PCM_PLAYER,
	DSP_LISR_CB_PCM_RECORDER,
	DSP_LISR_CB_SPEECH_IO_POINT_A,
	DSP_LISR_CB_SPEECH_IO_POINT_B,
	DSP_LISR_CB_SPEECH_IO_POINT_C,
	DSP_LISR_CB_SPEECH_IO_POINT_D,
	DSP_LISR_CB_SPEECH_IO_POINT_E,
	DSP_LISR_CB_SPEECH_IO_POINT_F,
	DSP_LISR_CB_PCM_PLAYER_A,
	/* 10 */
	DSP_LISR_CB_HW_PROBE_A,
	DSP_LISR_CB_HW_PROBE_B,
	DSP_LISR_CB_SPEECH_IO_INJECT_A,
	DSP_LISR_CB_SPEECH_IO_INJECT_B,
	DSP_LISR_CB_SPEECH_IO_INJECT_C,
	DSP_LISR_CB_SPEECH_IO_INJECT_D,
	DSP_LISR_CB_SPEECH_IO_INJECT_E,
	DSP_LISR_CB_SPEECH_IO_INJECT_F,
	DSP_LISR_CB_END
};

/**
    \brief struct dsp_lisr_cb_conf - interrupt handler callback structure
*/
struct dsp_lisr_cb_conf {
	enum dsp_lisr_cb lisr_cb_type;
	enum dsp_irq_comm_flag  comm_flag_no;
};

/**
  @brief Get the interrupt callback list for requested interrupt no

  @param enum dsp_irq_no: requested interrupt no

  @return enum dsp_err_code error
 */
const struct dsp_lisr_cb_conf *dsp_audio_get_lisr_cb(enum dsp_irq_no);
/*@}*/
#endif /* _DSP_AUDIO_DRIVERIF_H */
/*@}*/
