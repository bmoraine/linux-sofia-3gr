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

/* @file
 *
 *FMR HLD API -  FMR System Interface
 *
 * System Messageing Interface.
 *
 * This file descrbes the abstraction layer of  messageing functionality
 * required by the FMR driver.
 *
 */

#ifndef AUD_APP_FMR_SYS_MSG_H
#define AUD_APP_FMR_SYS_MSG_H

/*
** ============================================================================
**
**				INCLUDE STATEMENTS
**
** ============================================================================
*/

#include "aud_app_fmr_hld_api_types_common.h"
#include "aud_app_fmr_hld_msg.h"

/*
** ============================================================================
**
**				Function Declarations
**
** ============================================================================
*/

int fmr_sys_msg_send(const struct fmtrx_msg_params *const event_params);

/* free the queue items during powering off */
void fmr_sys_sim_msg_free_queue_items(void);

/* @} */

#endif

