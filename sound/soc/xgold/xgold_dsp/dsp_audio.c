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
#include <linux/dmaengine.h>
#include <sound/soc.h>

#include "bastypes.h"
#include "dsp_audio_internal.h"
#include "dsp_audio_driverif.h"
#include "dsp_audio_platform.h"
#include "dsp_audio_hal_internal.h"
#include "dsp_audio_communication_internal.h"
#include "xgold_pcm.h"

#define	xgold_err(fmt, arg...) \
		pr_err("snd: aud: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: aud: "fmt, ##arg)

#define NUM_OF_FBA_DSP_CMD 42
#define NUM_OF_SBA_DSP_CMD 44

/* FIXME */
struct dsp_aud_list {
	u16 num_dsp;
	struct dsp_audio_device *dsp_fba;
	struct dsp_audio_device *dsp_sba;
};

/* FIXME: remove global variable */
static struct dsp_aud_list dsp;

static u16 supported_cmd_sba[NUM_OF_SBA_DSP_CMD] = {
	14, 22, 26, 28, 29, 31, 32, 33, 36, 38, 39, 40, 41, 42, 43,
	57, 58, 59, 62, 63, 64, 65, 66, 67, 68, 69, 70, 75, 84, 85,
	86, 87, 88, 89, 98, 100, 103, 106, 108, 109, 110, 112, 120,
	126
};

static u16 supported_cmd_fba[NUM_OF_FBA_DSP_CMD] = {
	14, 18, 33, 35, 37, 38, 39, 40, 47, 48, 49, 51, 52, 55, 62,
	63, 64, 65, 66, 69, 70, 78, 80, 84, 90, 91, 92, 94, 95, 97,
	98, 104, 105, 107, 111, 113, 114, 117, 119, 121, 124, 125,
};

/*
** ===========================================================================
**
**                       EXPORTED FUNCTIONS
**
** ===========================================================================
*/

/******************************************************************************
* Function:... is_cmd_supported_on_dsp
* Parameters:. command_id, p_supported_cmd_array, array_size
* Returns:.... 0x7fff if cmd not supported, else returns the command id
* Description: This function checks whether the command id
				is supported on the dsp.
*****************************************************************************/

static int is_cmd_supported_on_dsp(
	u16 command_id,
	u16 *p_supported_cmd_array,
	u16 array_size) {
	int i = 0;
	int ret = 0;
	u16 supported_cmd;
	for (i = 0; i < array_size; i++) {
		supported_cmd = *p_supported_cmd_array++;
		if (supported_cmd == command_id) {
			ret = command_id;
			break;
		} else if (supported_cmd > command_id) {
			ret = 0x7fff;
			break;
		} else {
			continue;
		}
	}
	return ret;
}

/******************************************************************************
* Function:... dsp_audio_init
* Parameters:. pointer to the list of dsp's in the system
* Returns:.... error
* Description: This function initializes the Audio DSP resources
******************************************************************************/
enum dsp_err_code dsp_audio_init(struct list_head *list)
{
	u16 num_dsp = 0;
	struct dsp_audio_device *dsp_dev = NULL;

	list_for_each_entry(dsp_dev, list, node) {
		num_dsp++;
		switch (dsp_dev->id) {
		case XGOLD_DSP_XG742_FBA:
		case XGOLD_DSP_XG632:
		case XGOLD_DSP_XG642:
		case XGOLD_DSP_XG631:
		case XGOLD_DSP_XG223:
			dsp.dsp_fba = dsp_dev;
			break;
		case XGOLD_DSP_XG742_SBA:
			dsp.dsp_sba = dsp_dev;
			break;
		default:
			break;
		}
	}
	dsp.num_dsp = num_dsp;

	xgold_debug("Num of dsp's = %d", num_dsp);

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
	enum dsp_err_code dsp_result = DSP_SUCCESS;
	U16 is_command_id_supported = 0x7fff;
	U16 is_command_valid = 0;
	xgold_debug("in %s\n", __func__);
	if (dsp.num_dsp == 2) {
		/* call the command if it is relevant for
			the frame-based processing */
		is_command_id_supported = is_cmd_supported_on_dsp(
						command_id,
						supported_cmd_fba,
						sizeof(supported_cmd_fba));
		if (0x7fff != is_command_id_supported) {
			is_command_valid = 1;
			dsp_add_audio_msg_2_dsp(dsp.dsp_fba,
					command_id, command_len/2 +
					DSP_AUDIO_CMD_ID_LEN, p_command);
		}

		/* call the command if it is relevant for
			the sample-based processing */
		is_command_id_supported = is_cmd_supported_on_dsp(
						command_id,
						supported_cmd_sba,
						sizeof(supported_cmd_sba));
		if (0x7fff != is_command_id_supported) {

			is_command_valid = 1;
			/* Enable I2S2 */
			if (command_id == 22) {
				if (*p_command != 0)
					dsp.dsp_sba->p_dsp_common_data->
					i2s_set_power_state(
					dsp.dsp_sba->p_dsp_common_data->
					p_snd_soc_platform,
					1);
				else
					dsp.dsp_sba->p_dsp_common_data->
					i2s_set_power_state(
					dsp.dsp_sba->p_dsp_common_data->
					p_snd_soc_platform,
					0);
			}

			dsp_add_audio_msg_2_dsp(dsp.dsp_sba,
					command_id, command_len/2 +
					DSP_AUDIO_CMD_ID_LEN, p_command);
		}

		/* return error if an error occurred */
		if (!is_command_valid) {
			pr_info("Dsp Audio Drv: TRAP\n");
			return DSP_ERR_COMMAND_FAILURE;
		}

	} else {
		dsp_add_audio_msg_2_dsp(dsp.dsp_fba, command_id,
			command_len / 2 + DSP_AUDIO_CMD_ID_LEN,
			p_command);
	}
	return dsp_result;
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
		(U8 *) (((2 == dsp.num_dsp) ? dsp.dsp_sba->shm_mem
		: dsp.dsp_fba->shm_mem) + shm_word_offset * 2),
		len_in_bytes);

	xgold_debug("%s: src %p, dst %p, len %d, val %x\n", __func__,
		(U8 *) (((2 == dsp.num_dsp) ? dsp.dsp_sba->shm_mem
		: dsp.dsp_fba->shm_mem) + shm_word_offset * 2),
		(U8 *) p_dest, len_in_bytes, *p_dest);

	return DSP_SUCCESS;
}
EXPORT_SYMBOL(dsp_audio_read_shm);

/*****************************************************************************
* Function:... dsp_audio_write_shm
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
		(U8 *) (((2 == dsp.num_dsp) ? dsp.dsp_sba->shm_mem
		: dsp.dsp_fba->shm_mem) + shm_word_offset * 2),
		(U8 *) p_src, len_in_bytes);

	memcpy((U8 *) (((2 == dsp.num_dsp) ? dsp.dsp_sba->shm_mem
		: dsp.dsp_fba->shm_mem) + shm_word_offset * 2),
		(U8 *) p_src, len_in_bytes);

	return DSP_SUCCESS;
}
EXPORT_SYMBOL(dsp_audio_write_shm);

/*****************************************************************************
* Function:... dsp_get_audio_shmem_base_addr
* Parameters:. None
* Returns:.... Base address of DSP shared memory
* Description: This function returns the dsp shmem base address to the client.
******************************************************************************/
dma_addr_t dsp_get_audio_shmem_base_addr(void)
{
	return (2 == dsp.num_dsp) ? dsp.dsp_sba->shm_mem_phys
		: dsp.dsp_fba->shm_mem_phys;
}
EXPORT_SYMBOL(dsp_get_audio_shmem_base_addr);

#ifdef CONFIG_OF
/* TODO */
struct dma_chan *xgold_of_dsp_get_dmach(
		struct dsp_audio_device *dsp_dev, int req)
{
	switch (req) {
	case STREAM_PLAY:
		return dma_request_slave_channel(dsp.dsp_fba->dev, "play1");
	case STREAM_REC:
		return dma_request_slave_channel(dsp.dsp_fba->dev, "record");
	default:
		return NULL;
	}
}
#endif

