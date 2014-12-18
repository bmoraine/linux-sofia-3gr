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

/* \file
 * FMR HLD API - System Interface
 * This file collects all sub-modules of the system interface in one header
 * file
 */

#ifndef AUD_APP_FMR_SYS_H
#define AUD_APP_FMR_SYS_H

#define FMR_SPEC_BODY { return; }
#define FMR_SPEC_BODY_INT { return 0; }

#include <types.h>
#include "aud_app_fmr_hld_msg.h"
#include "aud_app_fmr_sys_os.h"
#include "aud_app_fmr_sys_msg.h"

#define fmdrv_err(format, ...) \
	pr_err("FMR: " format, ## __VA_ARGS__)

#define fmdrv_warn(format, ...) \
	pr_warn("FMR: " format, ##__VA_ARGS__)

#define fmdrv_dbg(format, ...) \
	pr_debug("FMR: " format, ##__VA_ARGS__)

#define fmdrv_emerg(format, ...) \
	pr_emerg("FMR: " format, ## __VA_ARGS__)

#define fmdrv_crit(format, ...) \
	pr_crit("FMR: " format, ## __VA_ARGS__)

#define fmdrv_info(format, ...) \
	pr_info("FMR: " format, ## __VA_ARGS__)

/* @brief
 * Initialize the system context for FMR.
 * Integration layer could use this to initialize system message queue/events/
 * timer and etc.
 */
int fmr_sys_init(void);

/* @brief
 * De-initialize the system context for FMR.
 * Integration layer could use this to initialize system message queue/events/
 * timer and etc.
 */
void fmr_sys_deinit(void);

#endif	/* AUD_APP_FMR_SYS_H */

