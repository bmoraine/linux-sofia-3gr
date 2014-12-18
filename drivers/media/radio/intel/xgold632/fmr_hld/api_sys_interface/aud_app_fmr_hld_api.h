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
 * @file aud_app_fmr_hld_api.h
 *
 * Top level header file for FMRadio API interface.
 *
 **/

#ifndef AUD_APP_FMR_HLD_API_H
#define AUD_APP_FMR_HLD_API_H

/*
** ============================================================================
**
**				INCLUDE STATEMENTS
**
** ============================================================================
*/

#include "aud_app_fmr_hld_api_rx.h"
#include "aud_app_fmr_sys.h"

/*
** ============================================================================
**
**						 DEFINES
**
** ============================================================================
*/

/* The switching of debug information output */
#if !defined(DEBUG_FMR_HLD)
#define DEBUG_FMR_HLD 0
/* #warning DEBUG_FMR_HLD not specified in Makefile. Default = 0 */
#endif

/* enable/disable FSM trace, by default it's disabled */
#if !defined(DEBUG_FMR_FSM_TRC)
#define DEBUG_FMR_FSM_TRC 0
#endif

/* enable/disable FSM trace, by default it's disabled */
#if !defined(DEBUG_FMR_CD)
#define DEBUG_FMR_CD 0
#endif

/* Make the FmtrxRssi unit in dBuV (1) / dBm (0) by enabling this definition.*/
#if !defined(FMTRX_RSSI_IN_DBUV)
#define FMTRX_RSSI_IN_DBUV 1
/* #warning FMTRX_RSSI_IN_DBUV not specified in Makefile. Default = 1 */
#endif

/* Active the maximum volume mapping for FMRadio receiver */
/* which the 100 refer to 0X7FFF in mini DSP FW */
#if !defined(FMTRX_MAX_VOL_MAP)
#define FMTRX_MAX_VOL_MAP 1
/* #warning FMTRX_MAX_VOL_MAP not specified in Makefile. Default = 1 */
#endif

/*
** ============================================================================
**
**					 Exported Function Declarations
**
** ============================================================================
*/

/*
 *
 * @brief
 * Initializes the FM TRX Finite Control state FM Radio High Level Driver.
 *
 * @pre  Should only be called at initialization or when the FM radio is
 * powered off.
 *
 * @impl Direct call.
 */
int fmtrx_init(const enum fmtrx_init_mode fmtrx_init_mode);

/*
 * @brief Gets the FM TRX Hw state
 *
 * @return  The result of the operation is returned in int
 *
 * @impl    Message based.
 */
enum fmtrx_hw_state fmtrx_get_hw_state(void);

/*
*
* @brief Returns the core dump info.
*
* @param cd_info  carry the core dump descriptor
* @impl Direct call.
*
*/
void fmtrx_get_dump_info(struct fmtrx_core_dump *cd_info);

/*
 *
 * @brief Dump specific memory section.
 *
 * @param buf  carry the dump data
 * @param seg  memory section to dump
 * @impl Direct call.
 *
 */
void fmtrx_dump(u8 *buf, struct fmtrx_dump_seg *seg);

/*
*
* @brief Read register data
*
* @param reg_access  Pointer to register access data
* @impl Direct call.
*
*/
int fmtrx_reg_read(struct fmtrx_reg_data *reg_access);

/*
 *
 * @brief Write register data
 *
 * @param reg_access  Pointer to register access data
 * @impl Direct call.
 *
 */
int fmtrx_reg_write(struct fmtrx_reg_data *reg_access);

#endif
