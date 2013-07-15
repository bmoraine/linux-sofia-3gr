/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
*/

#include <linux/idi/idi_interface.h>

int idi_device_pm_set_class(struct idi_peripheral_device *peripheral);
int idi_device_pm_set_state(struct idi_peripheral_device *peripheral,
						int idi_state);


int idi_peripheral_device_pm_set_state_by_name(
					struct idi_peripheral_device *idipdev,
					const char *state_name);
int idi_peripheral_device_pm_set_state(struct idi_peripheral_device *idipdev,
					void *state);
struct device_state_pm_state *idi_peripheral_device_pm_get_state_handler(
					struct idi_peripheral_device *idipdev,
					const char *state_name);
int idi_peripheral_device_pm_set_class(struct idi_peripheral_device *idipdev,
					const char *class_name);
int idi_peripheral_device_pm_set_active_state(
					struct idi_peripheral_device *idipdev,
					void *state);
int idi_peripheral_device_pm_activate(struct idi_peripheral_device *idipdev,
					bool activate);
