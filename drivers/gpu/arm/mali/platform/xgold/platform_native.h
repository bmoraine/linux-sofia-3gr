/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Notes:
 * Jul 16 2014: IMC: [OSS Scan] Add missing license type
 * Jun 02 2014: IMC: Splitup platform adaption for better readability
 */


#if !defined(CONFIG_PLATFORM_DEVICE_PM_VIRT)
int mali_platform_native_init(struct mali_platform_data *pdata);
void mali_platform_native_probe(struct device *dev, struct device_node *np);
#endif

