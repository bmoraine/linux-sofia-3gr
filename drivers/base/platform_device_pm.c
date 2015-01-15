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

#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/device_state_pm.h>
#include <linux/slab.h>

/* Used by the platform device driver (typically in the probe function)
 * to configure which platform power management class this device belongs to.
 * It is also used to initialize other parts of pm_data of pdev. */

int platform_device_pm_set_class(struct platform_device *pdev,
						const char *class_name)
{
	return device_state_pm_set_class(&(pdev->dev), class_name);
}
EXPORT_SYMBOL(platform_device_pm_set_class);

/* Used by platform device driver to obtain a ptr to the state structure. This
 * can be used with platform_device_pm_set_state to omit the string search of
 * platform_device_pm_set_state_by_name */
struct platform_device_pm_state *platform_device_pm_get_state_handler(
			struct platform_device *pdev, const char *state_name)
{
	return (struct platform_device_pm_state *)
			device_state_pm_get_state_handler(&pdev->dev,
								state_name);
}
EXPORT_SYMBOL(platform_device_pm_get_state_handler);

/* Set the device state of the given platform_device */
int platform_device_pm_set_state(struct platform_device *pdev,
				struct platform_device_pm_state *state)
{
	return device_state_pm_set_state(&(pdev->dev),
					(struct device_state_pm_state *)state);
}
EXPORT_SYMBOL(platform_device_pm_set_state);

/* Set the device state of the given platform_device */
int platform_device_pm_set_state_by_name(struct platform_device *pdev,
				const char *class_name)
{
	return device_state_pm_set_state_by_name(&(pdev->dev), class_name);
}
EXPORT_SYMBOL(platform_device_pm_set_state_by_name);

int platform_device_pm_set_active_state(struct platform_device *pdev,
				struct platform_device_pm_state *state)
{
	return device_state_pm_set_active_state(&(pdev->dev),
				(struct device_state_pm_state *)state);
}
EXPORT_SYMBOL(platform_device_pm_set_active_state);

int platform_device_pm_activate(struct platform_device *pdev, bool activate)
{
	return device_state_pm_activate(&(pdev->dev), activate);
}
EXPORT_SYMBOL(platform_device_pm_activate);

/* TODO */
int platform_device_pm_get_state_id(struct platform_device *pdev,
				const char *state_name)
{
	return device_state_pm_get_state_id(&pdev->dev, state_name);
}
EXPORT_SYMBOL(platform_device_pm_get_state_id);
