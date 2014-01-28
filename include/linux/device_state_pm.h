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

#ifndef _DEVICE_STATE_PM_H_
#define _DEVICE_STATE_PM_H_

struct device_state_pm_state {
	const char *name;
	void *config;
	unsigned int config_size; /* in Bytes */
};

/* Used by device driver to set the power state of the given device */
int device_state_pm_set_state_by_name(struct device *, const char *);

/* Used by device driver to set the power state of the given device */
int device_state_pm_set_state(struct device *, struct device_state_pm_state *);

/* Used by device driver to get the state handler for a given device state */
struct device_state_pm_state *device_state_pm_get_state_handler(
					struct device *, const char *);

int device_state_pm_set_active_state(struct device *,
					struct device_state_pm_state *);

int device_state_pm_activate(struct device *, bool);

/* Used by the corresponding drivers to register to pm framework
 * to register a new class that is supported */
int device_state_pm_set_class(struct device *, const char *);

/* Remove a device that was previously registered
 * with device_state_pm_set_class
*/
int device_state_pm_remove_device(struct device *);

int device_state_pm_get_state_id(struct device *, const char *);

int device_state_pm_set_state_by_name_silent(struct device *, const char *);

#endif
