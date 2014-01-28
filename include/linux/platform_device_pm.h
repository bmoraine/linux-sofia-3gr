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

#ifndef _PLATFORM_DEVICE_PM_H_
#define _PLATFORM_DEVICE_PM_H_

typedef struct device_state_pm_state platform_device_pm_state;

/* Used by device driver to set the power class of the given platform_device */
int platform_device_pm_set_class(struct platform_device *, const char *);

/* Used by device driver to retrieve a pointer
 * to a state object from the state name
 */
struct platform_device_pm_state *platform_device_pm_get_state_handler(
			struct platform_device *, const char *);

/* Used by device driver to set the power state of the given platform_device */
int platform_device_pm_set_state_by_name(
			struct platform_device *, const char *);

/* Used by device driver to set the power state of the given platform_device */
int platform_device_pm_set_state(struct platform_device *,
			struct platform_device_pm_state *);

int platform_device_pm_set_active_state(struct platform_device *,
			struct platform_device_pm_state *);

int platform_device_pm_activate(struct platform_device *, bool);

int platform_device_pm_get_state_id(
			struct platform_device *, const char *);
#endif
