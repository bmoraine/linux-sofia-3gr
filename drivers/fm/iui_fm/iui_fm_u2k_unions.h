/*
 * Copyright (C) 2013 Intel Mobile Communications GmbH
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

#ifndef _IUI_FM_U2K_UNIONS_H
#define _IUI_FM_U2K_UNIONS_H

#include <linux/fm/iui_fm.h>
#include <linux/fm/iui_fm_serialized.h>

#define FM_COMPILETIME_ASSERT_EQUALS(val1, val2) \
	extern char __iui_fm_cta[((val1) == (val2)) ? 1 : -1]

/**
 * Union of message structure types, used for internal storage of a message
 * to avoid dynamic allocation.
 */
union iui_fm_u2k_message_union {
	struct iui_fm_u2k_mitigation_message mitigation;
};

/**
 * Mirror of IuiFmMitigationUnion, but contains actual data instead of
 * pointers.
 */
union iui_fm_u2k_mitigation_union {
	struct iui_fm_emmc_freq_info emmc_info;
	uint32_t freq_khz;
	struct iui_fm_wlan_mitigation wlan_mitigation;
	enum iui_fm_fmr_injection_side fmr_inj_side;
	struct iui_fm_bt_channel_mask bt_ch_mask;
	struct iui_fm_gnss_mitigation gnss_mitigation;
};

/**
  * Union of all possible message data structures.
  */
union iui_fm_u2k_message_data_union {
	union iui_fm_u2k_mitigation_union mitigation_data;
};

/* Ensure offset of each union field element is 0, to ensure writing directly
to the union type rather than the specific field is safe. */
FM_COMPILETIME_ASSERT_EQUALS(
offsetof(union iui_fm_u2k_message_union, mitigation), 0);

FM_COMPILETIME_ASSERT_EQUALS(
offsetof(union iui_fm_u2k_mitigation_union, emmc_info), 0);
FM_COMPILETIME_ASSERT_EQUALS(
offsetof(union iui_fm_u2k_mitigation_union, freq_khz), 0);
FM_COMPILETIME_ASSERT_EQUALS(
offsetof(union iui_fm_u2k_mitigation_union, wlan_mitigation), 0);
FM_COMPILETIME_ASSERT_EQUALS(
offsetof(union iui_fm_u2k_mitigation_union, fmr_inj_side), 0);
FM_COMPILETIME_ASSERT_EQUALS(
offsetof(union iui_fm_u2k_mitigation_union, bt_ch_mask), 0);
FM_COMPILETIME_ASSERT_EQUALS(
offsetof(union iui_fm_u2k_mitigation_union, gnss_mitigation), 0);

FM_COMPILETIME_ASSERT_EQUALS(
offsetof(union iui_fm_u2k_message_data_union, mitigation_data), 0);

#endif /* _IUI_FM_U2K_UNIONS_H */
