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
**				MODULE DESCRIPTION
**
** ============================================================================
*/
/* This file contains the hardware abstraction layer functions for internal FM,
   radio which control the mini DSP firmware */

/*
** ============================================================================
**
**				INCLUDE STATEMENTS
**
** ============================================================================
*/
#include <linux/err.h>


#include <aud_app_fmr_hld_rx.h>
#include <aud_app_fmr_hld.h>
#include <aud_app_fmr_sys.h>

/*
** ============================================================================
**
**				     DEFINES
**
** ============================================================================
*/

/*
** ============================================================================
**
**			     GLOBAL DATA DEFINITIONS
**
** ============================================================================
*/

/*
** ============================================================================
**
**			     LOCAL DATA DEFINITIONS
**
** ============================================================================
*/

/* Flag for indicating, whether HLD is initialised */
static enum fmtrx_init_state fmtrx_init_state = FMTRX_INIT_STATE_UNINITIALISED;

/* Core dump section for both FM Rx and FM Tx */
struct fmtrx_dump_seg cd_rtx[] = {
	{"0xE1608000++0x760",  0xE1608000, 0x760}, /* Chip register space  */
	{"0xE1603000++0x1000", 0xE1603000, 0x1000} /* FW register RX space */
};

/* Debug only, to check if core dump works */
#ifdef DEBUG_FMR_HLD
struct dump_buff {
	u8 hw[0x760];
	u8 fw[0x1000];
};

struct dump_buff cd_buf;
#endif /* DEBUG_FMR_HLD */

/* core dump descriptor. nothing to dump in idle */
struct fmtrx_core_dump cd_desc = {cd_rtx, 2};

#if defined AUD_APP_HOST_TEST
/* Array for storing the seek report on host test */
struct fmtrx_rssi_report seek_report_array[25];
#endif

/*
** ============================================================================
**
**			    LOCAL FUNCTION DECLARATIONS
**
** ============================================================================
*/

/*
** ============================================================================
**
**			    LOCAL FUNCTION DEFINITIONS
**
** ============================================================================
*/

/* FM Radio interrupt handler */
void fmtrx_irq_handler(enum fmtrx_int_vec vec)
{
	u32 intr_clr = 0, intr_sta = 0, intr_sw = 0;

	switch (vec) {
	case FMR_INT_DED0:
		/* Command done interrupt */
		intr_clr |= IR_TRX_CMD;

		/* Trigger command done event */
		fmr_sys_trigger_event(FMR_IR_CMD_DONE);

		/* Unmask the command interrupt */
		fmtrx_enable_interrupts(IR_TRX_CMD, false, CTX_IRQ);
		break;

	case FMR_INT_DED1:
		/* Command2 done interrupt */
		intr_clr |= IR_TRX_CMD2;
		fmr_sys_trigger_event(FMR_IR_CMD2_DONE);

		/* Unmask the command2 interrupt, not mandatory?? */
		fmtrx_enable_interrupts(IR_TRX_CMD2, false, CTX_IRQ);
		break;

	case FMR_INT_DED2:
		/* Unused */
		intr_clr |= IR_INTDED2;
		break;

	case FMR_INT_DED3:
		intr_clr |= IR_INTDED3;
		break;

	case FMR_INT_SWINT:
		/* Read interrupt status register */
		intr_sta = fmtrx_get_interrupt_status();

		/* Software interrupts */
		intr_sw = intr_sta & (IR_SWINT0 | IR_SWINT1 | IR_SWINT2 |
			IR_SWINT3 | IR_SWINT4);

		/* Separated event mapping for RX and TX */
		if (FMTRX_HW_STATE_RX_ACTIVE == fmtrx_get_hw_state())
			fmrx_irq_handler(intr_sw);

		/* Clear all software interrupt bits */
		intr_clr = intr_clr | intr_sw;
		break;

	default:
		break;
	}

	/* Clear dedicated and sotware interrupts */
	if (intr_clr)
		fmtrx_clear_interrupt(intr_clr);
}

/*
** ============================================================================
**
**			   EXPORTED FUNCTIONS DEFINITIONS
**
** ============================================================================
*/

/*
** ============================================================================
**
**				FM TRX functionality
**
** ============================================================================
*/

/* Inquire the FM Radio HW state */
enum fmtrx_hw_state fmtrx_get_hw_state(void)
{
	enum fmtrx_hw_state fmtrx_hw_state = FMTRX_HW_STATE_IDLE;
	enum fmrx_hw_state fmrx_hw_state = fmrx_get_hw_state();

	if (FMRX_HW_STATE_IDLE == fmrx_hw_state)
		fmtrx_hw_state = FMTRX_HW_STATE_IDLE;
	else if (FMRX_HW_STATE_ACTIVE == fmrx_hw_state)
		fmtrx_hw_state = FMTRX_HW_STATE_RX_ACTIVE;
	else if (FMRX_HW_STATE_SEEK_ACTIVE == fmrx_hw_state)
		fmtrx_hw_state = FMTRX_HW_STATE_SEEK_ACTIVE;
	else
		fmtrx_hw_state = FMTRX_HW_STATE_INVALID;

	return fmtrx_hw_state;
}

/* Initializes the FMTRX SM */
int fmtrx_init(enum fmtrx_init_mode fmtrx_init_mode)
{
	s32 rc = 0;

	switch (fmtrx_init_mode) {
	case FMTRX_INIT_MODE_ON: {
		if (FMTRX_INIT_STATE_UNINITIALISED == fmtrx_init_state) {
			/* Initialize system specific data structures */
			rc = fmr_sys_init();

			rc = fmrx_init(FMTRX_INIT_MODE_ON);

			/* Register the FM TRX IRQ handler with the system */
			fmr_sys_irq_register(fmtrx_irq_handler);

			fmtrx_init_state = FMTRX_INIT_STATE_INITIALISED;
		}
	}
	break;

	case FMTRX_INIT_MODE_OFF: {
		if (FMTRX_INIT_STATE_INITIALISED == fmtrx_init_state) {
			/* Register the FM TRX IRQ handler with the system */
			fmr_sys_irq_register(NULL);

			rc = fmrx_init(FMTRX_INIT_MODE_OFF);

			/* De-initialize system specific data structures */
			fmr_sys_deinit();

			fmtrx_init_state = FMTRX_INIT_STATE_UNINITIALISED;
		}
	}
	break;

	default: {
		rc = -EINVAL;
		fmdrv_info("Invalid initialisation mode\n");
	}
	break;
	}

	return rc;
}

#if !defined AUD_APP_HOST_TEST
void fmtrx_get_dump_info(struct fmtrx_core_dump *cd_info)
{
	memcpy(cd_info, &cd_desc, sizeof(struct fmtrx_core_dump));
}

void fmtrx_dump(u8 *buf, struct fmtrx_dump_seg *seg)
{
	/* For debugging purpose only */
#if  DEBUG_FMR_CD == 1
	if (0xE1608000 == seg->addr_start) {
		memcpy(&cd_buf.hw, (u8 *)seg->addr_start, seg->size);
		memcpy(buf, &cd_buf.hw, seg->size);
	} else {
		memcpy(&cd_buf.fw, (u8 *)seg->addr_start, seg->size);
		memcpy(buf, &cd_buf.fw, seg->size);
	}
#else
	memcpy(buf, (u8 *)seg->addr_start, seg->size);
#endif
}

void fmtrx_wait_atleast(u32 time2run)
{
	if (FMTRX_HW_STATE_IDLE != fmtrx_get_hw_state())
		fmr_sys_busy_wait(time2run);
}

/* Read from a register */
int fmtrx_reg_read(struct fmtrx_reg_data *reg_access)
{
	s32 rc = -EIO;

	if (NULL != reg_access) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_REG_READ;
		rx_msg.params.p_read_reg = reg_access;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Write to a register */
int fmtrx_reg_write(struct fmtrx_reg_data *reg_access)
{
	s32 rc = -EIO;

	if (NULL != reg_access) {
		struct fmrx_msgbox_buff rx_msg;
		rx_msg.event = FMRX_EVENT_REG_WRITE;
		rx_msg.params.p_write_reg = reg_access;
		rc = fmrx_event_dispatcher(&rx_msg);
	} else {
		rc = -EINVAL;
	}

	return rc;
}
#endif
