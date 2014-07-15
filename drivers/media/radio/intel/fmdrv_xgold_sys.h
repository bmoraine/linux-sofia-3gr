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

#ifndef _FMDRV_XGOLD_SYS_H_
#define _FMDRV_XGOLD_SYS_H_

#include <linux/list.h>

#include <fmdrv_xgold.h>
#include <aud_app_fmr_sys.h>
#include <aud_app_fmr_hld_rx.h>

enum fmr_info_comm_sys {
	MODULE_SYS_FM = 0
};

enum fmr_onoff_seq {
	FMR_POWER_ON,
	FMR_POWER_OFF,
	FMR_POWER_INVALID
};

struct fmtrx_sys_sim_msg {
	struct list_head list; /* kernel's list struct */
	struct fmtrx_msg_params msg;
};

void fmr_sys_sim_dispatcher(void);
int fmr_sys_msg_init(struct xgold_fmdev *fmdev);
int fmr_sys_msg_send(const struct fmtrx_msg_params *const event_params);
void fmr_sys_on_off_seq(enum fmr_onoff_seq seq);
int fmr_sys_get_cfg(struct fmrx_cfg **cfg);


#endif /* _FMDRV_XGOLD_SYS_H_ */
