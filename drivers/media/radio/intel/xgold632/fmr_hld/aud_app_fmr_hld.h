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
**			       INTERFACE DESCRIPTION
**
** ============================================================================
*/

/*
 *
 * @file aud_app_fmr_hld.h
 *
 * Internal declarations for the FMR HLD implementation
 *
 */

#ifndef AUD_APP_FMR_HLD_H
#define AUD_APP_FMR_HLD_H

/*
** ============================================================================
**
**				INCLUDE STATEMENTS
**
** ============================================================================
*/
#include <aud_app_fmr_hld_rx.h>
#include <aud_app_fmr_hld_api.h>
#include <aud_app_fmr_sys_os.h>
#include <aud_app_fmr_sys_msg.h>

/*
** ============================================================================
**
**				    DEFINES
**
** ============================================================================
*/

/* Revision ID of HLD and SW Package, updated per every new release */
/* The ID number is stored in Hex format, e.g. version 1.6.0 would be stored
 * in  ID regster as 0X0160
 */

/* v3.0.1 */
#define FMR_HLD_ID		0X301;

/* v3.0.0 */
#define FMR_SW_PKG_ID		0X301;

/* RDS ring-buffer size defined by FW */
#define FMRX_RDS_RING_BUFFER_SIZE 29
#define FMTX_RDS_RING_BUFFER_SIZE 29

/* The maximum possible number of valid RDS groups in ring-buffer */
#define FMRX_RDS_MAX_VALID_GRPS (FMRX_RDS_RING_BUFFER_SIZE - 1)

#define FMTRX_FSM_TRC_LEN	50

/*
** ============================================================================
**
**			      EXPORTED TYPE DEFINITIONS
**
** ============================================================================
*/


/*
** ============================================================================
**
**			   EXPORTED FUNCTION DECLARATIONS
**
** ============================================================================
*/


/*
  @defgroup FMR_RX FMRadio Receiver API Definition
  @brief All the API functions for FMR receiver is provided here;
  some of them have the restriction for calling them, please be awre of this.

*/
/* @{ */
/* @} */ /* FMR_RX */

/*
  @ingroup FMR_RX
  @defgroup FMR_RX_ACTIONS Main Actions
  @brief This section describes all the main actions.
*/
/* @{ */

/*
  @ingroup FMR_RX
  @defgroup FMR_RX_TIMER Timer
  @brief This section describes the timer utilities for short term wait.
*/
/* @{ */

/*
 * \brief Wait for at least number of us.
 * Provides a counter routing to system level by using the inside FMR timer.
 * @param time2run unit in micro second, maximum allowed value is 160000 us.
 */
void fmtrx_wait_atleast(u32 time2run);


/* @} */ /* FMR_RX_TIMER */

/*
 * @brief
 * Function for sending a FM Rx event to FM Rx dispatcher. This function will
 * make a direct function call to the FM Rx event handler with the requested
 * event and parameters required to handle the event.
 *
 * @param   event	    An FM Rx action to be taken on FM Transmitter.
 * @param   event_parmams   This type is a union that contains the data for
 *			    each interface function respectively.
 *
 * @return  The result of the operation is returned in int
 */

int fmrx_msg_dispatch(enum fmrx_event event, union fmrx_ev_prams evt_params);

/*
 * @brief For antenna selftest to make sure that the FMRadio antenna has been
 * correctly soldered. It could be called whenever the radio is in idle,
 * receiving or transmitting mode; and, the result of testing would be reported
 * by selftest done callback.
 *
 * @param   freq Specified frequency for checking the antenna connection.
 *
 * @return  The result of the operation is returned in int
 *
 * @impl Message based.
 */
int fmtrx_antenna_selftest(u32 freq);

/*
 * @brief Gets the FM TRX Hw state
 *
 * @return  The result of the operation is returned in int
 *
 * @impl    Message based.
 */
enum fmtrx_hw_state fmtrx_get_hw_state(void);

#endif

