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
 * Nov 18 2014: IMC: Adaptions for Mali Utgard driver r5p0-01rel0
 * Jul 16 2014: IMC: [OSS Scan] Add missing license type
 * Jun 02 2014: IMC: Add pm and debugfs support
 *                   Splitup platform adaption for better readability
 */


int platform_debugfs_register(struct mali_platform_pm *pdev_pm,
	struct platform_device *pdev);

