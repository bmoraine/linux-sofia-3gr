/*
 * Component: XGOLD6321 DSP Audio Driver
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <sound/soc.h>

#include "bastypes.h"
#include "dsp_audio_internal.h"
#include "dsp_audio_driverif.h"
#include "dsp_audio_platform.h"
#include "dsp_audio_hal_internal.h"
#include "dsp_audio_communication_internal.h"

#define	xgold_err(fmt, arg...) \
		pr_err("snd: aud: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: aud: "fmt, ##arg)

/* FIXME: do not use extern */
extern struct dsp_audio_device *p_dsp_audio_dev;

/*
** ===========================================================================
**
**                       EXPORTED FUNCTIONS
**
** ===========================================================================
*/

/******************************************************************************
* Function:... dsp_audio_init
* Parameters:. None
* Returns:.... error
* Description: This function initializes the Audio DSP resources
******************************************************************************/
enum dsp_err_code dsp_audio_init(void)
{
	dsp_audio_communication_init();
	return DSP_SUCCESS;
}

/******************************************************************************
* Function:... dsp_audio_cmd
* Parameters:. command id,command lenght, command paramaters
* Returns:.... error
* Description: This function sends the dsp command to a requested DSP.
******************************************************************************/
enum dsp_err_code dsp_audio_cmd(
	U16 command_id,
	U16 command_len,
	U16 *p_command)
{
	xgold_debug("in %s\n", __func__);
	dsp_add_audio_msg_2_dsp(command_id,
				command_len / 2 + DSP_AUDIO_CMD_ID_LEN,
				p_command);

	return DSP_SUCCESS;
}
EXPORT_SYMBOL(dsp_audio_cmd);

/******************************************************************************
* Function:... dsp_audio_read_shm
* Parameters:. destination to copy read data,offset in SHM to read, length
* Returns:.... error
* Description: This function reads the data from shared memory of
				requested size
******************************************************************************/
enum dsp_err_code dsp_audio_read_shm(
	U16 *p_dest,
	U16 shm_word_offset,
	U16 len_in_bytes)
{
	memcpy((U8 *) p_dest,
		(U8 *) (p_dsp_audio_dev->shm_mem + shm_word_offset * 2),
		len_in_bytes);

	xgold_debug("%s: src %p, dst %p, len %d, val %x\n", __func__,
		(U8 *) (p_dsp_audio_dev->shm_mem + shm_word_offset * 2),
		(U8 *) p_dest, len_in_bytes, *p_dest);

	return DSP_SUCCESS;
}
EXPORT_SYMBOL(dsp_audio_read_shm);

/*****************************************************************************
* Function:... dsp_audio_read_shm
* Parameters:. source of data to write to SHM,offset in SHM to write, length
* Returns:.... error
* Description: This function writes the data of requested size to
				the shared memory
******************************************************************************/
enum dsp_err_code dsp_audio_write_shm(
	U16 *p_src,
	U16 shm_word_offset,
	U16 len_in_bytes)
{
	xgold_debug("%s: dst %p, src %p, len %d\n", __func__,
		(U8 *) (p_dsp_audio_dev->shm_mem + shm_word_offset * 2),
		(U8 *) p_src, len_in_bytes);

	memcpy((U8 *) (p_dsp_audio_dev->shm_mem + shm_word_offset * 2),
		(U8 *) p_src, len_in_bytes);

	return DSP_SUCCESS;
}
EXPORT_SYMBOL(dsp_audio_write_shm);

