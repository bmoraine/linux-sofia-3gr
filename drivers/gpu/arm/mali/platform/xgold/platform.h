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
 *
 * Notes:
 * Nov 18 2014: IMC: Adaptions for Mali Utgard driver r5p0-01rel0
 *                   - Use Mali DT support instead of Intel bringup variant
 * Jul 16 2014: IMC: [OSS Scan] Add missing license type
 * Jun 02 2014: IMC: Change hooks to platform adaption to use platform_device
 *                   Add hooks for platform runtime PM
 * Mar 14 2014: IMC: Add basic support for SoFIA
 */


extern int mali_platform_init(void);
extern int mali_platform_device_init(struct platform_device *pdev);
extern int mali_platform_device_deinit(struct platform_device *pdev);

extern int mali_platform_suspend(struct platform_device *pdev);
extern int mali_platform_resume(struct platform_device *pdev);

extern int mali_platform_runtime_suspend(struct platform_device *pdev);
extern int mali_platform_runtime_resume(struct platform_device *pdev);
extern int mali_platform_runtime_idle(struct platform_device *pdev);

