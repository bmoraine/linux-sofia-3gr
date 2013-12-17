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

#ifndef __XGOLDPM_H
#define __XGOLDPM_H

struct xgold_user_info {
	int user_id;
	int per_id;
};

/* Function prototype for the platform specific PM call */
int xgold_dev_pm_set_state(struct device *dev,
		struct device_state_pm_state *new_state);

/* Function prototype for getting the initial state */
struct device_state_pm_state*
xgold_dev_pm_get_initial_state(struct device *dev);

#endif
