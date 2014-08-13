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

#ifndef _IUI_FM_SERIALIZED_H
#define _IUI_FM_SERIALIZED_H

/**
 * Kernelspace to Userspace message ID
 */
enum iui_fm_k2u_message_id {
	IUI_FM_K2U_MESSAGE_ID_REGISTER_CB,
	IUI_FM_K2U_MESSAGE_ID_NOTIFY_FREQUENCY,
	IUI_FM_K2U_MESSAGE_ID_MITIGATION_COMPLETE,
	IUI_FM_K2U_MESSAGE_ID_MAX,
	IUI_FM_K2U_MESSAGE_ID_FIRST = IUI_FM_K2U_MESSAGE_ID_REGISTER_CB
};

/**
 * Userspace to Kernelspace message ID
 */
enum iui_fm_u2k_message_id {
	IUI_FM_U2K_MESSAGE_ID_MITIGATION,
	IUI_FM_U2K_MESSAGE_ID_MAX,
	IUI_FM_U2K_MESSAGE_ID_FIRST = IUI_FM_U2K_MESSAGE_ID_MITIGATION
};

/**
 * Common fields to be put at the beginning of all Kernel-to-User message
 * structures.
 *
 * @message_id  Identifier of the message type.
 * @macro_id    Identifier of the macro causing the message to be send.
 */
struct iui_fm_k2u_message_header {
	enum iui_fm_k2u_message_id message_id;
	enum iui_fm_macro_id macro_id;
};

/**
 * Common fields to be put at the beginning of all User-to-Kernel message
 * structures.
 *
 * @message_id  Identifier of the message type.
 * @macro_id    Identifier of the kernel-side macro that will receive the
 *              message.
 */
struct iui_fm_u2k_message_header {
	enum iui_fm_u2k_message_id message_id;
	enum iui_fm_macro_id macro_id;
};

/**
 * Message structure for an in-kernel callback registration (Kernel to User).
 *
 * @registered  Indicate whether the macro is registering (TRUE) or
 *              deregistering (FALSE) a callback, based on whether the
 *              callback pointer passed to
 *               iui_fm_register_mitigation_callback() was non-NULL.
 */
struct iui_fm_k2u_register_cb_message {
	uint32_t registered;
};

/**
 * Message structure for a Frequency Notification message (Kernel to User).
 *
 * @type  Indicates the type of the message-specific data following the
 *        message, based on the types in iui_fm_notification_union.
 */
struct iui_fm_k2u_notify_frequency_message {
	enum iui_fm_freq_notification_type type;
};

/**
 * Message structure for a Mitigation Complete message (Kernel to User).
 *
 * @status
 * @sequence
 * @type  Indicates the type of the message-specific data following the
 *        message, based on the types in iui_fm_mitigation_union.
 */
struct iui_fm_k2u_mitigation_complete_message {
	enum iui_fm_mitigation_status status;
	uint32_t sequence;
	enum iui_fm_mitigation_type type;
};

/**
 * Message structure for a Mitigation message (User to Kernel).
 *
 * @sequence
 * @type  Indicates the type of the message-specific data following the
 *        message, based on the types in iui_fm_mitigation_union.
 */
struct iui_fm_u2k_mitigation_message {
	uint32_t sequence;
	enum iui_fm_mitigation_type type;
};

#endif /* _IUI_FM_SERIALIZED_H */
