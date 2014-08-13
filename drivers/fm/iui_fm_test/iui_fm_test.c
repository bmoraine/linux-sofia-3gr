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

#include <linux/fm/iui_fm.h>

#include <linux/init.h>
#include <linux/module.h>


MODULE_LICENSE("GPL v2");

enum iui_fm_mitigation_status iui_fm_test_cb0(
		const enum iui_fm_macro_id macro_id,
		const struct iui_fm_mitigation *mitigation,
		const uint32_t sequence)
{
	pr_alert("FMK Test CB0: macro_id: %i, mitigation: 0x%08X,
		sequence: %i\n", macro_id, (unsigned int) mitigation,
		sequence);
	if (mitigation != NULL) {
		pr_alert("  mitigation {type: %i, info:
			(0x%08X == %i)}\n", mitigation->type,
			mitigation->info.freq_khz, mitigation->info.freq_khz);
	}

	return IUI_FM_MITIGATION_COMPLETE_OK;
}


static int iui_fm_test_init(void)
{
	int32_t result;
	struct iui_fm_wlan_mitigation wlan_mitigation = {
		1, /* num_channels */
		{
			{
				2412000,			/* frequency */
				IUI_FM_WLAN_NO_TX_PWR_LIMIT	/* max_tx_pwr */
			}, /* channel_tx_pwr[0] */
			{
				0,                              /* frequency */
				IUI_FM_WLAN_NO_TX_PWR_LIMIT     /* max_tx_pwr */
			}  /* channel_tx_pwr[1] */
		}, /* channel_tx_pwr[] */
		0, /* wlan_adc_dac_freq */
		IUI_FM_WLAN_RX_GAIN_NORMAL /*, rx_gain_behavior */
		/* 0 */ /* rx_gain_reduction */
	};

	struct iui_fm_mitigation mitigation = {
		IUI_FM_MITIGATION_TYPE_WLAN, /* type */
	};
	mitigation.info.wlan_mitigation = &wlan_mitigation;

	pr_alert("iui_fm_test_init()\n");
	result = iui_fm_register_mitigation_callback(IUI_FM_MACRO_ID_GNSS,
			iui_fm_test_cb0);
	pr_alert("iui_fm_test_init():
			iui_fm_register_mitigation_callback(): %i\n", result);

	result = iui_fm_mitigation_complete(IUI_FM_MACRO_ID_WLAN,
			IUI_FM_MITIGATION_ERROR, &mitigation, 12);

	pr_alert("iui_fm_test_init():
			iui_fm_mitigation_complete(): %i\n", result);

	return 0;
}

static void iui_fm_test_exit(void)
{
	int32_t result;
	pr_alert("iui_fm_test_exit()\n");
	result = iui_fm_register_mitigation_callback(IUI_FM_MACRO_ID_IDI, NULL);
	pr_alert("iui_fm_test_exit():
			iui_fm_register_mitigation_callback(): %i\n", result);
}

module_init(iui_fm_test_init);
module_exit(iui_fm_test_exit);
