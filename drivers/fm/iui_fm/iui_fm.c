/*
 * Copyright (C) 2013-2014 Intel Mobile Communications GmbH
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/uio.h>

#include <linux/fm/fmdev.h>
#include <linux/fm/iui_fm.h>
#include <linux/fm/iui_fm_serialized.h>
#include "iui_fm_u2k_unions.h"
#include "iui_fm_macro_support.h"

MODULE_LICENSE("GPL v2");

/**
 * Return codes used by the IUI_FM kernel module.  Values match those in
 * UtaCommonReturnCodes ("uta_common_return_codes.h").
 *
 * NOTE: IUI_FM_ERROR_NOT_SUPPORTED would normally equal
 * UTA_ERROR_NOT_SUPPORTED (-7), but for now we are silently ignoring
 * API calls from unsupported macros. This can be changed once all macros
 * obey the "CONFIG_IUI_FM_<macro_id>" definitions in the iui_fm Kconfig.
 */
enum iui_fm_return_codes {
	IUI_FM_SUCCESS = 0,
	IUI_FM_FAILURE = -1,
	IUI_FM_ERROR_INVALID_HANDLE = -3,
	IUI_FM_ERROR_INVALID_PARAM = -5,
	IUI_FM_ERROR_NOT_SUPPORTED = IUI_FM_SUCCESS /* -7 */
};

/**
 * Structure to store specific information for one macro.
 */
struct iui_fm_macro_info {
	iui_fm_mitigation_cb mitigation_cb;
	uint32_t sequence;
};

/**
 * Structure to store the global state of the iui_fm kernel module.
 *
 * @iui_fm_mutex   Protects the macro_info array from multiple access.
 * @iui_fm_thread  Kernel thread structure.
 * @macro_info     Array of macro-specific information, such as currently
 *                 registered frequency mitigation callback functions.
 */
struct iui_fm_kernel_info {
	struct mutex iui_fm_mutex;
	struct task_struct *iui_fm_thread;
	struct iui_fm_macro_info macro_info[IUI_FM_MACRO_ID_MAX];
};

/* Stores the global state data of the iui_fm kernel module. */
static struct iui_fm_kernel_info iui_fm_kernel_state;

/* Indicates whether a macro is supported in the iui_fm API's. */
static bool iui_fm_is_macro_supported(const enum iui_fm_macro_id macro_id)
{
	static const unsigned int macro_support_bitfield =
		(IUI_FM_EMMC_SUPPORT << IUI_FM_MACRO_ID_EMMC) |
		(IUI_FM_CLASS_D_SUPPORT << IUI_FM_MACRO_ID_CLASS_D) |
		(IUI_FM_PMU_CP_SUPPORT << IUI_FM_MACRO_ID_PMU_CP) |
		(IUI_FM_MS_CP_SUPPORT << IUI_FM_MACRO_ID_MS_CP) |
		(IUI_FM_DCDC_SUPPORT << IUI_FM_MACRO_ID_DCDC) |
		(IUI_FM_IDI_SUPPORT << IUI_FM_MACRO_ID_IDI) |
		(IUI_FM_WLAN_SUPPORT << IUI_FM_MACRO_ID_WLAN) |
		(IUI_FM_FMR_SUPPORT << IUI_FM_MACRO_ID_FMR) |
		(IUI_FM_BT_SUPPORT << IUI_FM_MACRO_ID_BT) |
		(IUI_FM_GNSS_SUPPORT << IUI_FM_MACRO_ID_GNSS);

	return macro_support_bitfield & (1 << macro_id);
}


/* Utility to repeatedly receive data from fmdev in blocking mode until the
 requested number of bytes is received, or an error occurs. */
static ssize_t iui_fm_kernel_receive_full(char *buffer, size_t size)
{
	size_t bytes_left, bytes_read = 0;
	ssize_t status = 0;

	pr_debug("iui_fm_kernel_receive_full(size: %u)\n", size);
	do {
		buffer += bytes_read;
		bytes_left = size - bytes_read;
		status = fmdev_receive(buffer, bytes_left, true);
		bytes_read += status;
	} while (status > 0 && bytes_read < size);

	if (status > 0)
		return bytes_read;
	else
		return status;
}

/* Utility function to trace a human-readable display of the contents of a
 frequency notification. */
static void iui_fm_print_notification_info(
		const struct iui_fm_freq_notification * const notification)
{
	unsigned int i;
	if (IS_ERR(notification)) {
		pr_debug("  IS_ERR(notification)!\n");
		return;
	}

	switch (notification->type) {
	case IUI_FM_FREQ_NOTIFICATION_TYPE_EMMC:
		if (!IS_ERR(notification->info.emmc_info)) {
			pr_debug("  emmc_info: {\n    emmc_frequency: {\n");
			for (i = 0; i < IUI_FM_EMMC_DEVICE_ID_MAX; i++) {
				pr_debug("      [%u]: { clk_src: %u, frequency: %u }\n",
						i,
						notification->info.emmc_info->
						emmc_frequency[i].clk_src,
						notification->info.emmc_info->
						emmc_frequency[i].frequency);
			}
			pr_debug("    }\n  }\n");
		} else
			pr_debug("  IS_ERR(emmc_info)!\n");
		break;

	case IUI_FM_FREQ_NOTIFICATION_TYPE_KHZ:
		pr_debug("  freq_khz: %u\n",
				notification->info.freq_khz);
		break;

	case IUI_FM_FREQ_NOTIFICATION_TYPE_WLAN:
		if (!IS_ERR(notification->info.wlan_info)) {
			pr_debug("  wlan_info: {\n    num_channels: %u\n    channel_info: {\n",
					notification->info.wlan_info->
					num_channels);
			for (i = 0; i < IUI_FM_WLAN_MAX_CHANNELS; i++) {
				pr_debug("      [%u]: { frequency: %u, bandwidth: %i }\n",
						i,
						notification->info.wlan_info->
						channel_info[i].frequency,
						notification->info.wlan_info->
						channel_info[i].bandwidth);
			}
			pr_debug("    }\n    wlan_adc_dac_freq: %u\n  }\n",
						notification->info.wlan_info->
							wlan_adc_dac_freq);
		} else
			pr_debug("  IS_ERR(wlan_info)!\n");
		break;

	case IUI_FM_FREQ_NOTIFICATION_TYPE_FMR:
		if (!IS_ERR(notification->info.fmr_info)) {
			pr_debug("  fmr_info: { rx_freq: %u, inj_side: %u }\n",
					notification->info.fmr_info->rx_freq,
					notification->info.fmr_info->inj_side);
		} else
			pr_debug("  IS_ERR(fmr_info)!\n");
		break;

	case IUI_FM_FREQ_NOTIFICATION_TYPE_BT:
		if (!IS_ERR(notification->info.bt_info)) {
			pr_debug("  bt_info: { bt_state: %u }\n",
					notification->info.bt_info->bt_state);
		} else
			pr_debug("  IS_ERR(bt_info)!\n");
		break;

	case IUI_FM_FREQ_NOTIFICATION_TYPE_GNSS:
		if (!IS_ERR(notification->info.gnss_info)) {
			pr_debug("  gnss_info: { state: %u, snr: %i, bus_frequency: %u, pll_frequency: %u, adc_frequency: %u }\n",
					notification->info.gnss_info->state,
					notification->info.gnss_info->snr,
					notification->info.gnss_info->
							bus_frequency,
					notification->info.gnss_info->
							pll_frequency,
					notification->info.gnss_info->
							adc_frequency);
		} else
			pr_debug("  IS_ERR(gnss_info)!\n");
		break;

	default:
		pr_debug("  Invalid notification type: %i!\n",
		notification->type);
		break;
	}
}

/* Utility function to trace a human-readable display of the contents of a
 frequency mitigation structure. */
static void iui_fm_print_mitigation_info(
		const struct iui_fm_mitigation * const mitigation)
{
	unsigned int i;

	if (IS_ERR(mitigation)) {
		pr_debug("  IS_ERR(Mitigation)!\n");
		return;
	}

	switch (mitigation->type) {
	case IUI_FM_MITIGATION_TYPE_EMMC:
		if (!IS_ERR(mitigation->info.emmc_info)) {
			pr_debug("  emmc_info: {\n    emmc_frequency: {\n");
			for (i = 0; i < IUI_FM_EMMC_DEVICE_ID_MAX; i++) {
				pr_debug("      [%u]: { clk_src: %u,frequency: %u }\n",
						i,
						mitigation->info.emmc_info->
						emmc_frequency[i].clk_src,
						mitigation->info.emmc_info->
						emmc_frequency[i].frequency);
			}
			pr_debug("    }\n  }\n");
		} else
			pr_debug("  IS_ERR(emmc_info)!\n");
		break;

	case IUI_FM_MITIGATION_TYPE_KHZ:
		pr_debug("  freq_khz: %u\n",
				mitigation->info.freq_khz);
		break;

	case IUI_FM_MITIGATION_TYPE_WLAN:
		if (!IS_ERR(mitigation->info.wlan_mitigation)) {
			pr_debug("  wlan_mitigation: {\n    num_channels: %u\n    channel_tx_pwr: {\n",
					mitigation->info.wlan_mitigation->
							num_channels);
			for (i = 0; i < IUI_FM_WLAN_MAX_CHANNELS; i++) {
				pr_debug("      [%u]: { frequency: %u, max_tx_pwr: %i }\n",
						i,
						mitigation->info.wlan_mitigation
						->channel_tx_pwr[i].frequency,
						mitigation->info.wlan_mitigation
						->channel_tx_pwr[i].max_tx_pwr);
			}
			pr_debug("    }\n    wlan_adc_dac_freq: %u\n    rx_gain_behavior: %u\n  }\n",
					mitigation->info.wlan_mitigation->
							wlan_adc_dac_freq,
					mitigation->info.wlan_mitigation->
							rx_gain_behavior);
		} else
			pr_debug("  IS_ERR(wlan_mitigation)!\n");
		break;

	case IUI_FM_MITIGATION_TYPE_FMR:
		pr_debug("  fmr_inj_side: %u\n",
				mitigation->info.fmr_inj_side);
		break;

	case IUI_FM_MITIGATION_TYPE_BT:
		if (!IS_ERR(mitigation->info.bt_ch_mask)) {
			pr_debug("  bt_ch_mask: {\n");
			for (i = 0; i < IUI_FM_BT_CHANNEL_MASK_WORDS; i++) {
				pr_debug("    [%u]: 0x%08X\n", i,
						mitigation->info.bt_ch_mask->
								bt_ch_mask[i]);
			}
			pr_debug("  }\n");
		} else
			pr_debug("  IS_ERR(bt_ch_mask)!\n");

		break;

	case IUI_FM_MITIGATION_TYPE_GNSS:
		if (!IS_ERR(mitigation->info.gnss_mitigation)) {
			pr_debug("  gnss_mitigation: { priority: %i, bus_frequency: %u, pll_frequency: %u, adc_frequency: %u }\n",
					(int) mitigation->info.gnss_mitigation
							->priority,
					mitigation->info.gnss_mitigation->
							bus_frequency,
					mitigation->info.gnss_mitigation->
							pll_frequency,
					mitigation->info.gnss_mitigation->
							adc_frequency);
		} else
			pr_debug("  IS_ERR(gnss_mitigation)!\n");
		break;

	default:
		pr_debug("  Invalid mitigation type: %i!\n",
				mitigation->type);
		break;
	}
}


/* See prototype in "iui_fm.h". */
int32_t iui_fm_register_mitigation_callback(
				const enum iui_fm_macro_id macro_id,
				const iui_fm_mitigation_cb mitigation_cb)
{
	struct iui_fm_k2u_message_header header = {
		IUI_FM_K2U_MESSAGE_ID_REGISTER_CB,
		macro_id
	};
	struct iui_fm_k2u_register_cb_message message;
	struct iovec vector[] = {
		{&header, sizeof(header)},
		{&message, sizeof(message)}
	};

	pr_debug("iui_fm_register_mitigation_callback(macro_id: %i, mitigation_cb: 0x%08X)\n",
					macro_id, (unsigned int) mitigation_cb);

	if (macro_id < IUI_FM_MACRO_ID_FIRST ||
				macro_id >= IUI_FM_MACRO_ID_MAX)
		return IUI_FM_ERROR_INVALID_PARAM;

	if (!iui_fm_is_macro_supported(macro_id)) {
		pr_warn("iui_fm_register_mitigation_callback(macro_id: %i, mitigation_cb: 0x%08X): Unsupported Macro!\n",
					macro_id, (unsigned int) mitigation_cb);

		return IUI_FM_ERROR_NOT_SUPPORTED;
	}

	mutex_lock(&(iui_fm_kernel_state.iui_fm_mutex));
	iui_fm_kernel_state.macro_info[macro_id].mitigation_cb =
						mitigation_cb;
	mutex_unlock(&(iui_fm_kernel_state.iui_fm_mutex));

	if (mitigation_cb != NULL)
		message.registered = true;
	else
		message.registered = false;

	if (fmdev_send_vector(vector, ARRAY_SIZE(vector))) {
		pr_crit("iui_fm_register_mitigation_callback(): Failed to write header and message!\n");
		return IUI_FM_FAILURE;
	}

	return IUI_FM_SUCCESS;
}
EXPORT_SYMBOL(iui_fm_register_mitigation_callback);

/* See prototype in "iui_fm.h". */
int32_t iui_fm_notify_frequency(const enum iui_fm_macro_id macro_id,
		const struct iui_fm_freq_notification * const notification)
{
	struct iui_fm_k2u_message_header header = {
		IUI_FM_K2U_MESSAGE_ID_NOTIFY_FREQUENCY,
		macro_id
	};
	struct iui_fm_k2u_notify_frequency_message message;
	struct iovec vector[3] = {
		{&header, sizeof(header)},
		{&message, sizeof(message)}
	};
	pr_debug("iui_fm_notify_frequency(macro_id: %i, notification: %p)\n",
							macro_id, notification);
	iui_fm_print_notification_info(notification);

	if (IS_ERR(notification))
		return IUI_FM_ERROR_INVALID_HANDLE;

	if (macro_id < IUI_FM_MACRO_ID_FIRST
				|| macro_id >= IUI_FM_MACRO_ID_MAX) {
		pr_crit("iui_fm_notify_frequency(macro_id: %i): Macro ID out of range!\n",
								macro_id);
		return IUI_FM_ERROR_INVALID_PARAM;
	}
	if (!iui_fm_is_macro_supported(macro_id)) {
		pr_warn("iui_fm_notify_frequency(macro_id: %i): Unsupported Macro!\n",
								macro_id);
		return IUI_FM_ERROR_NOT_SUPPORTED;
	}

	message.type = notification->type;
	switch (notification->type) {
	case IUI_FM_FREQ_NOTIFICATION_TYPE_EMMC:
		vector[2].iov_base = notification->info.emmc_info;
		vector[2].iov_len  = sizeof(*(notification->info.emmc_info));
		break;
	case IUI_FM_FREQ_NOTIFICATION_TYPE_KHZ:
		vector[2].iov_base = (void *) &(notification->info.freq_khz);
		vector[2].iov_len  = sizeof(notification->info.freq_khz);
		break;
	case IUI_FM_FREQ_NOTIFICATION_TYPE_WLAN:
		vector[2].iov_base = notification->info.wlan_info;
		vector[2].iov_len  = sizeof(*(notification->info.wlan_info));
		break;
	case IUI_FM_FREQ_NOTIFICATION_TYPE_FMR:
		vector[2].iov_base = notification->info.fmr_info;
		vector[2].iov_len  = sizeof(*(notification->info.fmr_info));
		break;
	case IUI_FM_FREQ_NOTIFICATION_TYPE_BT:
		vector[2].iov_base = notification->info.bt_info;
		vector[2].iov_len  = sizeof(*(notification->info.bt_info));
		break;
	case IUI_FM_FREQ_NOTIFICATION_TYPE_GNSS:
		vector[2].iov_base = notification->info.gnss_info;
		vector[2].iov_len  = sizeof(*(notification->info.gnss_info));
		break;
	default:
		return IUI_FM_ERROR_INVALID_PARAM;
		break;
	}

	if (fmdev_send_vector(vector, ARRAY_SIZE(vector))) {
		pr_crit("iui_fm_notify_frequency(): Vector send failed!\n");
		return IUI_FM_FAILURE;
	}

	return IUI_FM_SUCCESS;
}
EXPORT_SYMBOL(iui_fm_notify_frequency);

/* See prototype in "iui_fm.h". */
int32_t iui_fm_mitigation_complete(const enum iui_fm_macro_id macro_id,
			const enum iui_fm_mitigation_status status,
			const struct iui_fm_mitigation * const mitigation,
			const uint32_t sequence)
{
	int32_t result = IUI_FM_SUCCESS;
	uint32_t latest_sequence;

	struct iui_fm_k2u_message_header header = {
		IUI_FM_K2U_MESSAGE_ID_MITIGATION_COMPLETE,
		macro_id
	};
	struct iui_fm_k2u_mitigation_complete_message message = {
		status,
		sequence
	};
	struct iovec vector[3] = {
		{&header, sizeof(header)},
		{&message, sizeof(message)}
	};

	pr_debug("iui_fm_mitigation_complete(macro_id: %i, status: %i, mitigation: %p, sequence: %u)\n",
					macro_id, status, mitigation, sequence);
	iui_fm_print_mitigation_info(mitigation);

	if (IS_ERR(mitigation))
		return IUI_FM_ERROR_INVALID_HANDLE;
	if (macro_id < IUI_FM_MACRO_ID_FIRST ||
				macro_id >= IUI_FM_MACRO_ID_MAX)
		return IUI_FM_ERROR_INVALID_PARAM;
	if (!iui_fm_is_macro_supported(macro_id)) {
		pr_warn("iui_fm_mitigation_complete(macro_id: %i): Unsupported Macro!\n",
								macro_id);
		return IUI_FM_ERROR_NOT_SUPPORTED;
	}

	mutex_lock(&(iui_fm_kernel_state.iui_fm_mutex));
	latest_sequence = iui_fm_kernel_state.macro_info[macro_id].sequence;
	mutex_unlock(&(iui_fm_kernel_state.iui_fm_mutex));

	if (sequence > latest_sequence)
		return IUI_FM_ERROR_INVALID_PARAM;
	if (sequence < latest_sequence)
		return IUI_FM_SUCCESS; /* Filter old sequence numbers
						silently. */

	message.type = mitigation->type;
	switch (mitigation->type) {
	case IUI_FM_MITIGATION_TYPE_EMMC:
		vector[2].iov_base = mitigation->info.emmc_info;
		vector[2].iov_len = sizeof(*(mitigation->info.emmc_info));
		break;
	case IUI_FM_MITIGATION_TYPE_KHZ:
		vector[2].iov_base = (void *) &(mitigation->info.freq_khz);
		vector[2].iov_len = sizeof(mitigation->info.freq_khz);
		break;
	case IUI_FM_MITIGATION_TYPE_WLAN:
		vector[2].iov_base = mitigation->info.wlan_mitigation;
		vector[2].iov_len = sizeof(*(mitigation->info.wlan_mitigation));
		break;
	case IUI_FM_MITIGATION_TYPE_FMR:
		vector[2].iov_base = (void *) &(mitigation->info.fmr_inj_side);
		vector[2].iov_len = sizeof(mitigation->info.fmr_inj_side);
		break;
	case IUI_FM_MITIGATION_TYPE_BT:
		vector[2].iov_base = mitigation->info.bt_ch_mask;
		vector[2].iov_len = sizeof(*(mitigation->info.bt_ch_mask));
		break;
	case IUI_FM_MITIGATION_TYPE_GNSS:
		vector[2].iov_base = mitigation->info.gnss_mitigation;
		vector[2].iov_len = sizeof(*(mitigation->info.gnss_mitigation));
		break;
	default:
		return IUI_FM_ERROR_INVALID_PARAM;
		break;
	}

	if (fmdev_send_vector(vector, ARRAY_SIZE(vector))) {
		pr_crit("iui_fm_mitigation_complete(): Vector send failed!\n");
		return IUI_FM_FAILURE;
	}

	pr_debug("iui_fm_mitigation_complete(): %i\n", result);
	return result;
}
EXPORT_SYMBOL(iui_fm_mitigation_complete);

/**
 * Helper function to call a registered frequency mitigation callback function
 * based on a User-to-Kernel mitigation message.
 */
void iui_fm_kernel_call_freq_mitigation_cb(
			const enum iui_fm_macro_id macro_id,
			const struct iui_fm_mitigation *mitigation,
			const uint32_t sequence)
{
	iui_fm_mitigation_cb mitigation_cb;

	pr_debug("iui_fm_kernel_call_freq_mitigation_cb(macro_id: %i, mitigation: %p, sequence: %u)\n",
						macro_id, mitigation, sequence);
	iui_fm_print_mitigation_info(mitigation);

	mutex_lock(&(iui_fm_kernel_state.iui_fm_mutex));
	mitigation_cb = iui_fm_kernel_state.macro_info[macro_id].mitigation_cb;
	if (sequence <= iui_fm_kernel_state.macro_info[macro_id].sequence) {
		pr_crit("iui_fm_kernel_call_freq_mitigation_cb(): Warning: Old sequence %u received from FM, current: %u\n",
			sequence,
			iui_fm_kernel_state.macro_info[macro_id].sequence);
	}
	if (!iui_fm_is_macro_supported(macro_id)) {
		pr_warn("iui_fm_kernel_call_freq_mitigation_cb(macro_id: %i): Unsupported Macro!\n",
								macro_id);
	}

	iui_fm_kernel_state.macro_info[macro_id].sequence = sequence;
	mutex_unlock(&(iui_fm_kernel_state.iui_fm_mutex));

	if (mitigation_cb != NULL) {
		enum iui_fm_mitigation_status status =
			mitigation_cb(macro_id, mitigation, sequence);
		pr_debug("iui_fm_kernel_call_freq_mitigation_cb(): Registered Callback: 0x%08X(): %i\n",
					(unsigned int) mitigation_cb, status);

		/* When receiving a mitigation request from FM, the userspace
		layer returns immediately (after passing the message to the
		kernel) with IUI_FM_MITIGATION_ASYNC_PENDING. So if the kernel-
		side macro returns immediately we still have to send an
		asynchronous response. */
		if (status != IUI_FM_MITIGATION_ASYNC_PENDING) {
			iui_fm_mitigation_complete(macro_id, status,
							mitigation, sequence);
		}
	} else {
		pr_crit("iui_fm_kernel_call_freq_mitigation_cb(): No registered callback for macro %i\n",
				macro_id);
	}
}


/* Message sizes - Indexed by enum iui_fm_k2u_message_id */
static const uint32_t u2k_message_sizes[] = {
	sizeof(struct iui_fm_u2k_mitigation_message)
};

/* Mitigation Data Sizes - Indexed by enum iui_fm_mitigation_type. */
static const uint32_t mitigation_data_sizes[] = {
	sizeof(struct iui_fm_emmc_freq_info),
	sizeof(uint32_t),
	sizeof(struct iui_fm_wlan_mitigation),
	sizeof(enum iui_fm_fmr_injection_side),
	sizeof(struct iui_fm_bt_channel_mask),
	sizeof(struct iui_fm_gnss_mitigation)
};

/**
  * Read a single message from the fmdev character device and process it.
  */
static int iui_fm_kernel_thread_process_next_message(void)
{
	struct iui_fm_u2k_message_header header = {
		IUI_FM_U2K_MESSAGE_ID_MAX,  /* message_id */
		IUI_FM_MACRO_ID_INVALID     /* macro_id */
	};
	union iui_fm_u2k_message_union message_union;
	union iui_fm_u2k_message_data_union data_union;
	size_t  bytes_to_read = sizeof(struct iui_fm_u2k_message_header);
	ssize_t bytes_read = 0;
	struct iui_fm_mitigation miti;

	pr_debug("iui_fm_kernel_thread_process_next_message()\n");

	/* Read a message header. */
	bytes_read = iui_fm_kernel_receive_full((char *) &header,
								bytes_to_read);

	/* header error*/
	if (bytes_read != bytes_to_read) {
		pr_crit("iui_fm_kernel_thread_process_next_message(): Tried to read %u header bytes, received %i!\n",
					bytes_to_read, bytes_read);
		return IUI_FM_FAILURE;
	} else if (header.message_id < IUI_FM_U2K_MESSAGE_ID_FIRST ||
			header.message_id >= IUI_FM_U2K_MESSAGE_ID_MAX) {
		pr_crit("iui_fm_kernel_thread_process_next_message(): Invalid header.message_id: %i!\n",
					header.message_id);
		return IUI_FM_ERROR_INVALID_PARAM;
	} else if (header.macro_id < IUI_FM_MACRO_ID_FIRST ||
				header.macro_id >= IUI_FM_MACRO_ID_MAX) {
		pr_crit("iui_fm_kernel_thread_process_next_message(): Invalid header.macro_id: %i!\n",
				header.macro_id);
		return IUI_FM_ERROR_INVALID_PARAM;
	}

	/* Read the message structure corresponding to the
		previously read Message ID. */
	bytes_to_read = u2k_message_sizes[header.message_id];
	bytes_read = iui_fm_kernel_receive_full((char *) &message_union,
								bytes_to_read);

	/* message read error */
	if (bytes_read != bytes_to_read) {
		pr_crit("iui_fm_kernel_thread_process_next_message(): Tried to read %u message bytes, received %i!\n",
					bytes_to_read, bytes_read);
		return IUI_FM_FAILURE;
	}

	switch (header.message_id) {
	case IUI_FM_U2K_MESSAGE_ID_MITIGATION:
		if (message_union.mitigation.type <
					IUI_FM_MITIGATION_TYPE_FIRST ||
					message_union.mitigation.type >=
					IUI_FM_MITIGATION_TYPE_MAX) {
			pr_crit("iui_fm_kernel_thread_process_next_message(): Invalid Mitigation Type: %i!\n",
						message_union.mitigation.type);
			return IUI_FM_ERROR_INVALID_PARAM;
		}

		bytes_to_read = mitigation_data_sizes
						[message_union.mitigation.type];
		bytes_read = iui_fm_kernel_receive_full(
					(char *) &data_union, bytes_to_read);
		if (bytes_read != bytes_to_read) {
			pr_crit("iui_fm_kernel_thread_process_next_message(): Tried to read %u data bytes, received %i!\n",
					bytes_to_read, bytes_read);
			return IUI_FM_FAILURE;
		}

		miti.type = message_union.mitigation.type;
		switch (miti.type) {
		case IUI_FM_MITIGATION_TYPE_EMMC:
			miti.info.emmc_info =
				&(data_union.mitigation_data.emmc_info);
			break;
		case IUI_FM_MITIGATION_TYPE_KHZ:
			miti.info.freq_khz =
				data_union.mitigation_data.freq_khz;
			break;
		case IUI_FM_MITIGATION_TYPE_WLAN:
			miti.info.wlan_mitigation =
				&(data_union.mitigation_data.wlan_mitigation);
			break;
		case IUI_FM_MITIGATION_TYPE_FMR:
			miti.info.fmr_inj_side =
				data_union.mitigation_data.fmr_inj_side;
			break;
		case IUI_FM_MITIGATION_TYPE_BT:
			miti.info.bt_ch_mask =
				&(data_union.mitigation_data.bt_ch_mask);
			break;
		case IUI_FM_MITIGATION_TYPE_GNSS:
			miti.info.gnss_mitigation =
				&(data_union.mitigation_data.gnss_mitigation);
			break;
		default:
			/* Unreachable.
			   Mitigation type was already range-checked. */
			pr_crit("iui_fm_kernel_thread_process_next_message(): Invalid Mitigation Type: %i!\n",
								miti.type);
			return IUI_FM_ERROR_INVALID_PARAM;
		}

		iui_fm_kernel_call_freq_mitigation_cb(header.macro_id, &miti,
					message_union.mitigation.sequence);
		break;
	default:
		/* Unreachable. Message type was already range-checked. */
		pr_crit("iui_fm_kernel_thread_process_next_message(): Invalid header.message_id: %i!\n",
							header.message_id);
		return IUI_FM_ERROR_INVALID_PARAM;
	}

	pr_debug("iui_fm_kernel_thread_process_next_message(): Success!\n");
	return IUI_FM_SUCCESS;
}

/**
  * Entry point for the IuiFm Kernel Thread.
  */
static int iui_fm_kernel_thread_fn(void *data)
{
	int status = 0;
	pr_debug("iui_fm_kernel_thread_fn()\n");

	while (status == 0) {
		if (kthread_should_stop()) {
			pr_crit("iui_fm_kernel_thread_fn(): kthread_should_stop() returned true\n");
			break;
		}
		status = iui_fm_kernel_thread_process_next_message();
	}

	pr_crit("iui_fm_kernel_thread_fn(): exiting\n");
	/* Kill the thread if we leave the infinite loop. */
	do_exit(status);
}

static int iui_fm_kernel_init(void)
{
	enum iui_fm_macro_id macro_id;
	pr_debug("iui_fm_kernel_init()\n");

	/* Initialize callback pointers to NULL. */
	for (macro_id = IUI_FM_MACRO_ID_FIRST; macro_id < IUI_FM_MACRO_ID_MAX;
				macro_id++) {
		iui_fm_kernel_state.macro_info[macro_id].mitigation_cb = NULL;
		iui_fm_kernel_state.macro_info[macro_id].sequence = 0;
	}

	mutex_init(&(iui_fm_kernel_state.iui_fm_mutex));

	iui_fm_kernel_state.iui_fm_thread = kthread_run(
				iui_fm_kernel_thread_fn, NULL, "iui_fm");

	if (IS_ERR(iui_fm_kernel_state.iui_fm_thread)) {
		pr_crit("iui_fm_kernel_init(): Failed to start iui_fm kernel thread: %i\n",
				(int) iui_fm_kernel_state.iui_fm_thread);
		return PTR_ERR(iui_fm_kernel_state.iui_fm_thread);
	} else {
		return 0;
	}
}

static void iui_fm_kernel_exit(void)
{
	pr_crit("iui_fm_kernel_exit()\n");
	kthread_stop(iui_fm_kernel_state.iui_fm_thread);
}

subsys_initcall(iui_fm_kernel_init);
module_exit(iui_fm_kernel_exit);
