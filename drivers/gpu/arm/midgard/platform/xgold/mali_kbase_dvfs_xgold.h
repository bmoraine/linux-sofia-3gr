/*
 * Copyright (C) 2015 Intel Mobile Communications GmbH
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
 * March 11 2015: IMC: Add initial xgold platform adaptation code
 *
 */
#ifndef __MALI_KBASE_DVFS_XGOLD__
#define __MALI_KBASE_DVFS_XGOLD__
#define MAX_UTILISATION 75
#define MIN_UTILISATION 25
int kbase_platform_dvfs_init(struct xgold_platform_context *plf_context);
void kbase_platform_dvfs_term(struct xgold_platform_context *plf_context);
void kbase_platform_dvfs_enable(bool enable, struct kbase_device *kbdev);
#endif

