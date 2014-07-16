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

#include <linux/init.h>
#include <linux/module.h>
#include "bastypes.h"
#include "dsp_audio_driverif.h"
#include "dsp_audio_hal_internal.h"
#include "dsp_audio_internal.h"

/* Interrupt callback list for DSP_INT_0 */
const struct dsp_lisr_cb_conf lisr_int0_db[] = {
	{DSP_LISR_CB_TONE, DSP_IRQ_COMM_FLAG_3},
	{DSP_LISR_CB_END, (enum dsp_irq_comm_flag)-1},
};

/* Interrupt callback list for DSP_INT_1 */
const struct dsp_lisr_cb_conf lisr_int1_db[] = {
	{DSP_LISR_CB_PCM_PLAYER, DSP_IRQ_COMM_FLAG_3},
	{DSP_LISR_CB_PCM_PLAYER_A, DSP_IRQ_COMM_FLAG_5},
	{DSP_LISR_CB_END, (enum dsp_irq_comm_flag)-1},
};

/* Interrupt callback list for DSP_INT_2 */
const struct dsp_lisr_cb_conf lisr_int2_db[] = {
	{DSP_LISR_CB_PCM_RECORDER, DSP_IRQ_COMM_FLAG_4},
	{DSP_LISR_CB_END, (enum dsp_irq_comm_flag)-1},
};

/* Interrupt callback list for DSP_INT_3 */
const struct dsp_lisr_cb_conf lisr_int3_db[] = {
	{DSP_LISR_CB_HW_PROBE_A, DSP_IRQ_COMM_FLAG_1},
	{DSP_LISR_CB_HW_PROBE_B, DSP_IRQ_COMM_FLAG_2},
	{DSP_LISR_CB_END, (enum dsp_irq_comm_flag)-1},
};

/* Interrupt callback list for DSP_INT_6 */
const struct dsp_lisr_cb_conf lisr_int6_db[] = {
	{DSP_LISR_CB_SPEECH_IO_POINT_A, DSP_IRQ_COMM_FLAG_8},
	{DSP_LISR_CB_SPEECH_IO_POINT_B, DSP_IRQ_COMM_FLAG_9},
	{DSP_LISR_CB_SPEECH_IO_POINT_C, DSP_IRQ_COMM_FLAG_10},
	{DSP_LISR_CB_SPEECH_IO_POINT_D, DSP_IRQ_COMM_FLAG_11},
	{DSP_LISR_CB_SPEECH_IO_POINT_E, DSP_IRQ_COMM_FLAG_12},
	{DSP_LISR_CB_SPEECH_IO_POINT_F, DSP_IRQ_COMM_FLAG_13},
	{DSP_LISR_CB_END, (enum dsp_irq_comm_flag)-1},
};

/* Interrupt callback list for DSP interrupts */

const struct dsp_lisr_cb_conf *p_lisr_db[DSP_IRQ_7] = {
	lisr_int0_db,
	lisr_int1_db,
	lisr_int2_db,
	lisr_int3_db,
	NULL,
	NULL,
	lisr_int6_db,
};

/* FUNCTION-DESCRIPTION
 *
 * FUNCTION-NAME: dsp_audio_get_lisr_db
 *
 * PROCESSING:
 * Returns the interrupt callback list for requested interrupt no
 *
 * RETURN VALUE: None
 *
 */
const struct dsp_lisr_cb_conf *dsp_audio_get_lisr_db(enum dsp_irq_no irq_no)
{
	if (irq_no > DSP_IRQ_7)
		return NULL;
	return (const struct dsp_lisr_cb_conf *)p_lisr_db[irq_no];
}

/* FUNCTION-DESCRIPTION
**
** FUNCTION-NAME: dsp_irq_activate
**
** PROCESSING:
** Activate interrupt interrupt in hardware
**
** RETURN VALUE:
** DSP_SUCCESS             Successfull completion
** DSP_ERR_INVALID_HANDLE  Invalid interrupt handle
** DSP_ERR_NOT_ALLOC       Interrupt Not allocated
**
*/
enum dsp_err_code dsp_audio_irq_activate(struct dsp_audio_device *dsp,
		enum dsp_irq_no irq_no)
{
	U32 reg;

	if (irq_no > DSP_IRQ_7)
		return DSP_ERR_INVALID_IRQ;
	/* Read IMSC register for sba DSP */
	reg = dsp_get_audio_imsc(dsp);
	/* Check if the interrupt is disabled */

	/* For LTE IRQs are split beween 2 dsps [0..3]*/
	irq_no = irq_no % 4;

	if (!(reg & BIT(irq_no))) {
		/* Set bit in IMSC register for intended interrupt */
		reg |= BIT(irq_no);
		/* Set the updated value for the IMSC register */
		dsp_set_audio_imsc(dsp, reg);
	}
	return DSP_SUCCESS;
}

/* EXPORT_SYMBOL(dsp_audio_irq_activate); */

/* FUNCTION-DESCRIPTION
**
** FUNCTION-NAME: dsp_audio_irq_deactivate
**
** PROCESSING:
** Deativate interrupt interrupt in hardware
**
** RETURN VALUE:
** DSP_SUCCESS             Successfull completion
** DSP_ERR_INVALID_HANDLE  Invalid interrupt handle
** DSP_ERR_NOT_ALLOC       Interrupt Not allocated
**
*/
enum dsp_err_code dsp_audio_irq_deactivate(struct dsp_audio_device *dsp,
				enum dsp_irq_no irq_no)
{
	enum dsp_err_code ret = DSP_SUCCESS;
	U32 reg;
	if (irq_no > DSP_IRQ_7)
		return DSP_ERR_INVALID_IRQ;
	/* Read IMSC register for modem DSP */
	reg = dsp_get_audio_imsc(dsp);
	if (0 != (reg & (1 << irq_no))) {
		/* Reset bit in IMSC register for intended interrupt */
		reg &= ~(1 << irq_no);
		/* Set the updated value for the IMSC register */
		dsp_set_audio_imsc(dsp, reg);
	}
	return ret;
}

/* FUNCTION-DESCRIPTION
**
** FUNCTION-NAME: dsp_audio_irq_ack
**
** PROCESSING:
** Acknowledge interrupt interrupt in hardware
**
** RETURN VALUE:
** DSP_SUCCESS             Successfull completion
** DSP_ERR_INVALID_HANDLE  Invalid interrupt handle
** DSP_ERR_NOT_ALLOC       Interrupt Not allocated
*/
enum dsp_err_code dsp_audio_irq_ack(struct dsp_audio_device *dsp_dev,
			enum dsp_irq_no irq_no)
{
	enum dsp_err_code ret = DSP_SUCCESS;

	if (irq_no > DSP_IRQ_7)
		return DSP_ERR_INVALID_IRQ;

	/* For LTE IRQs are split beween 2 dsps [0..3]*/
	irq_no = irq_no % 4;

	/* Set the ICR register */
	dsp_set_audio_icr(dsp_dev, 1 << irq_no);
	return ret;
}
