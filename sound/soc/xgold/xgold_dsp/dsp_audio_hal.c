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

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <sound/soc.h>

#include "bastypes.h"
#include "dsp_audio_driverif.h"
#include "dsp_audio_internal.h"
#include "dsp_audio_hal_internal.h"
#include "dsp_audio_communication_internal.h"
#include "dsp_audio_hal_internal.h"
#include "dsp_audio_platform.h"
#include "xgold_pcm.h"

#define	xgold_err(fmt, arg...) \
		pr_err("snd: hal: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_info("snd: hal: "fmt, ##arg)


/*
**===========================================================================
**
**								LOCAL FUNCTIONS
**
** ==========================================================================
*/

/* FUNCTION-DESCRIPTION
 *
 * FUNCTION-NAME: dsp_set_audio_dsp_communication_flag
 *
 * PROCESSING:
 * Sets the specified DSP communication flag in the audio DSP
 *
 * RETURN VALUE: None
 *
 */
void dsp_set_audio_dsp_communication_flag(enum dsp_irq_comm_flag comm_flag)
{
	if (comm_flag < DSP_IRQ_COMM_FLAG_0 ||
			comm_flag > DSP_IRQ_COMM_FLAG_15) {
		xgold_err("%s - Invalid comm flag\n", __func__);
		return;
	}

	/* Set the specified communication flag */
	iowrite32((u32)BIT(comm_flag),
		p_dsp_audio_dev->shm_regs + p_dsp_audio_dev->uccf.set);
}

/* FUNCTION-DESCRIPTION
 *
 * FUNCTION-NAME: dsp_reset_audio_dsp_communication_flag
 *
 * PROCESSING:
 * Resets the specified DSP communication flag in the audio DSP
 *
 * RETURN VALUE: None
 *
 */
void dsp_reset_audio_dsp_communication_flag(enum dsp_irq_comm_flag comm_flag)
{
	if (comm_flag < DSP_IRQ_COMM_FLAG_0 ||
			comm_flag > DSP_IRQ_COMM_FLAG_15) {
		xgold_err("%s - Invalid comm flag\n", __func__);
		return;
	}

	iowrite32((u32)BIT(comm_flag),
		p_dsp_audio_dev->shm_regs + p_dsp_audio_dev->uccf.clear);
}

/* FUNCTION-DESCRIPTION
 *
 * FUNCTION-NAME: dsp_read_audio_dsp_communication_flag
 *
 * PROCESSING:
 * Reads the specified DSP communication flag in the  audio DSP
 *
 * RETURN VALUE: int - the value of the communication flag
 *
 */
int dsp_read_audio_dsp_communication_flag(enum dsp_irq_comm_flag comm_flag)
{
	int return_val;

	if (comm_flag < DSP_IRQ_COMM_FLAG_0 ||
			comm_flag > DSP_IRQ_COMM_FLAG_15) {
		xgold_err("%s - Invalid comm flag\n", __func__);
		return 0;
	}

	return_val = ioread32(p_dsp_audio_dev->shm_regs +
		p_dsp_audio_dev->uccf.get);

	return return_val & BIT(comm_flag) ? 1 : 0;
}

/* FUNCTION-DESCRIPTION
 *
 * FUNCTION-NAME: dsp_generate_audio_dsp_interrupt
 *
 * PROCESSING:
 * Generates an interrupt in the audio DSP
 *
 * RETURN VALUE: None
 *
 */
void dsp_generate_audio_dsp_interrupt(enum dsp_irq_no dsp_interrupt)
{
	/* Range check */
	if (DSP_IRQ_0 > dsp_interrupt || DSP_IRQ_3 < dsp_interrupt) {
		xgold_err("%s - Invalid dsp interrupt parameter\n", __func__);
		return;
	}

	/* Do not read and bit OR the intended flag as the interrupt register
	is shared between multiple OS to avoid race conditions
	Directly set the intended interrupt bit to 1 and other bits to 0
	DSP can accept very short pulse, so even if one OS has just set bit for
	other interrupt to 1, pulling it 0 will not cause any harm*/
	iowrite32(BIT(dsp_interrupt), p_dsp_audio_dev->shm_regs +
		p_dsp_audio_dev->mcu2dsp);
	iowrite32(0, p_dsp_audio_dev->shm_regs +
		p_dsp_audio_dev->mcu2dsp);
}

/* FUNCTION-DESCRIPTION
 *
 * FUNCTION-NAME: dsp_get_audio_imsc
 *
 * PROCESSING:
 * Gets the IMSC register value
 *
 * RETURN VALUE: U32 The value of the IMSC register
 *
 */
U32 dsp_get_audio_imsc(void)
{
	return ioread32(p_dsp_audio_dev->shm_regs + p_dsp_audio_dev->imsc);
}

/* FUNCTION-DESCRIPTION
 *
 * FUNCTION-NAME: dsp_set_audio_imsc
 *
 * PROCESSING:
 * Sets new value for the IMSC register
 *
 * RETURN VALUE: None
 *
 */
void dsp_set_audio_imsc(U32 reg)
{
	iowrite32(reg, p_dsp_audio_dev->shm_regs + p_dsp_audio_dev->imsc);
}

/* FUNCTION-DESCRIPTION
 *
 * FUNCTION-NAME: dsp_set_audio_icr
 *
 * PROCESSING:
 * Sets the ICR register for clearing interrupt on audio DSP
 *
 * RETURN VALUE: None
 *
 */
void dsp_set_audio_icr(U32 icr)
{
	iowrite32(icr, p_dsp_audio_dev->shm_regs + p_dsp_audio_dev->icr);
	/* Read any DSP modem register to be sure that the ICR
		is really written to the DSP ICR register */
	(void)ioread32(p_dsp_audio_dev->shm_regs + p_dsp_audio_dev->imsc);
}

/* FUNCTION-DESCRIPTION
 *
 * FUNCTION-NAME: dsp_get_audio_shmem_base_addr
 *
 * PROCESSING:
 * Returns the shared memory base address
 *
 * RETURN VALUE: None
 *
 */
U32 dsp_get_audio_shmem_base_addr(void)
{
	return p_dsp_audio_dev->shm_mem_phys;
}

#ifdef CONFIG_OF
/* TODO */
struct dma_chan *xgold_of_dsp_get_dmach(struct dsp_audio_device *dsp, int req)
{
	switch (req) {
	case STREAM_PLAY:
		return dma_request_slave_channel(dsp->dev, "play1");
	case STREAM_REC:
		return dma_request_slave_channel(dsp->dev, "record");
	default:
		return NULL;
	}
}
#endif
