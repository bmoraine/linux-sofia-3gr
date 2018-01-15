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

/*---------------------------------------------*/
/*              Include files.                 */
/*---------------------------------------------*/
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <sound/soc.h>

#include "bastypes.h"
#include "dsp_audio_driverif.h"
#include "dsp_audio_internal.h"
#include "dsp_audio_communication_internal.h"
#include "dsp_audio_hal_internal.h"
#include "dsp_audio_platform.h"

#define	xgold_err(fmt, arg...) \
		pr_err("snd: com: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: com: "fmt, ##arg)

/* ===========================================================================
**
**                             LOCAL DATA
**
** =========================================================================*/
/*!< Pointer to access command pipes for audio in shared memory */
struct dsp_aud_cmd_pipes *p_sm_aud_cmd_data;
/*!< Mutex to serialize one command at a time in the command pipe */
static struct mutex dsp_audio_cmd_pipe_lock;

/*===========================================================================
**
**								LOCAL PROTOTYPES
**
**=========================================================================*/

/*
**===========================================================================
**
**								LOCAL FUNCTIONS
**
**===========================================================================
*/

/*
**===========================================================================
**
**                              EXPORTED FUNCTIONS
**
**===========================================================================
*/

/****************************************************************************
* Function:... dsp_audio_communication_init
* Parameters:. none
* Returns:.... none
* Description: This procedure initializes the dsp_communication sub-module
*****************************************************************************/
void dsp_audio_communication_init(void)
{
	/* set to command pipe shared memory */
	p_sm_aud_cmd_data = p_dsp_audio_dev->shm_mem +
			DSP_AUDIO_COMMAND_PIPE_SM_OFFSET * 2;

	/* Initialize the mutex for command pipe */
	mutex_init(&dsp_audio_cmd_pipe_lock);
}

/*****************************************************************************
* Function:... dsp_cmd_int_2_audio_dsp
* Parameters:. none
* Returns:.... none
* Description: This procedure sends either the command interrupt 0, 1 or
*              2 to the frame based audio DSP.
*****************************************************************************/
void dsp_cmd_int_2_audio_dsp(enum dsp_irq_no int_no, U16 msg_id)
{
	int timeout_value;
	enum dsp_irq_comm_flag comm_flag;
	S16 *p_msg_id_shmem; /* Pointer into shared memory where the
							msg_id is placed */
/* Init comm_flag */
	comm_flag = DSP_IRQ_COMM_FLAG_0;

/* Init p_msg_id_shmem */
	p_msg_id_shmem = (S16 *)&(p_sm_aud_cmd_data->cmd_0_data[0]);

/* Map interrupt number to corresponding communication flag */
	switch (int_no) {
	case DSP_IRQ_0:
		comm_flag = DSP_IRQ_COMM_FLAG_0;
/* P_msg_id_shmem already initialized to point to pipe 0 */
	break;

	case DSP_IRQ_1:
		comm_flag = DSP_IRQ_COMM_FLAG_1;
/* Point to msg_id in shared memory pipe 1 */
		p_msg_id_shmem = (S16 *)&(p_sm_aud_cmd_data->cmd_1_data[0]);
	break;

	case DSP_IRQ_2:
		comm_flag = DSP_IRQ_COMM_FLAG_2;
/* Point to msg_id in shared memory pipe 2 */
		p_msg_id_shmem = (S16 *)&(p_sm_aud_cmd_data->cmd_2_data[0]);
	break;

	default:
		dump_stack();
	break;
	}

/* Set communication flag. Command is accepted when DSP clears flag. */
	dsp_set_audio_dsp_communication_flag(comm_flag);
/* Generate the interrupt in DSP subsystem */
	dsp_generate_audio_dsp_interrupt(int_no);
/* Init timeout_value */
	timeout_value = 0;

/* Check that DSP subsystem clears the communication flag indicating the
	command is accepted within the timeout time */
/* IRQ may delay this even longer than the remaining timeout-duration */
/* A DSP command is accepted by DSP when the corresponding comm flag is
	cleared and the DSP command ID is negated in shared memory. The way
	the timeout loop is made is that a loop is performed as long as the
	comm flag is not cleared and the ID is not negated. In the loop there
	is a busywait of 1 us. We loop a maximum of DSP_AUDIO_CMD_TIMEOUT.
	A DSP command should be accepted within 400us. */
	do {
		udelay(1);
		timeout_value++;
	} while ((timeout_value <= DSP_AUDIO_CMD_TIMEOUT) &&
			((dsp_read_audio_dsp_communication_flag(comm_flag)) ||
			(*p_msg_id_shmem != -msg_id)));

/* If timeout value larger than DSP_AUDIO_CMD_TIMEOUT then trap!
	Means DSP was to long to respond and probably crashed */
	if (timeout_value > DSP_AUDIO_CMD_TIMEOUT) {
		xgold_err("DSP is not responding and probably crashed\n");
	/* ToDo: add call to kernel stack dump here */
		dump_stack();
	}
}

/*****************************************************************************
* Function:... dsp_add_audio_msg_2_dsp
* Parameters:. msg_id: the ID number of the frame based audio DSP command
*              msg_length: Number of 16-bit words in the command
*              p_msg_par: Pointer to the command data
* Returns:.... none
* Description: This procedure implements the frame based audio dsp command
				interface.
******************************************************************************/
void dsp_add_audio_msg_2_dsp(U16  msg_id, U16 msg_length, U16 *p_msg_par)
{
	U16  *p_dsp_cmd_pipe;

	xgold_debug("in %s\n", __func__);

/* Make a sanity check. Check that no parameters are send if message length is
	larger than the message ID length */
	if (msg_length > DSP_AUDIO_CMD_ID_LEN && p_msg_par == NULL) {
		xgold_debug("Too long message length\n");
		return;
	}

/* lock the pipe 2 */
	mutex_lock(&dsp_audio_cmd_pipe_lock);

/* Point to pipe 2 */
	p_dsp_cmd_pipe = p_sm_aud_cmd_data->cmd_2_data;

/* copy message id */
	*p_dsp_cmd_pipe++ = msg_id;

/* Decrement message length */
	--msg_length;

/* copy message parameter */
	if (NULL != p_msg_par)
		memcpy((void *)p_dsp_cmd_pipe, p_msg_par,
			msg_length * sizeof(U16));

	xgold_debug("cmd id = %d, msg_length = %d", msg_id, msg_length);

/* Generate interrupt to dsp for command in pipe 2 */
	dsp_cmd_int_2_audio_dsp(DSP_IRQ_2, msg_id);

/* unlock the pipe 2 */
	mutex_unlock(&dsp_audio_cmd_pipe_lock);
}
