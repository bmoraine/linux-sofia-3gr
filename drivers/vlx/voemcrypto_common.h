/*
 ****************************************************************
 *
 *  Component: VLX VOEMCRYPTO interface
 *
 *  Copyright (C) 2011, Red Bend Ltd.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *  Contributor(s):
 *    Adam Mirowski (adam.mirowski@redbend.com)
 *
 ****************************************************************
 */

#ifndef VOEMCRYPTO_COMMON_H
#define VOEMCRYPTO_COMMON_H

/*==========================================================================
	TYPES & DEFINITIONS
==========================================================================*/

/* lookup string to search for vlink */
#define VOEMCRYPTO_VRPC_NAME "voemcrypto"

/* define error codes */
#define VOEMCRYPTO_ERR_NOLINK     1
#define VOEMCRYPTO_ERR_PMEMSIZE   2
#define VOEMCRYPTO_ERR_OPEN_FAIL  3

enum voemcrypto_cmd_t {
	VOEMCRYPTO_CMD_INITIALIZE = 0,
	VOEMCRYPTO_CMD_TERMINATE = 1,
	VOEMCRYPTO_CMD_OPEN_SESSION = 2,
	VOEMCRYPTO_CMD_CLOSE_SESSION = 3,
	VOEMCRYPTO_CMD_GEN_DERIVED_KEYS = 4,
	VOEMCRYPTO_CMD_GEN_NONCE = 5,
	VOEMCRYPTO_CMD_GEN_SIGN = 6,
	VOEMCRYPTO_CMD_LOAD_KEYS = 7,
	VOEMCRYPTO_CMD_REFRESH_KEYS = 8,
	VOEMCRYPTO_CMD_SELECT_KEY = 9,
	VOEMCRYPTO_CMD_DECRYPT_CTR = 10,
	VOEMCRYPTO_CMD_WRAP_KEYBOX = 11,
	VOEMCRYPTO_CMD_INSTALL_KEYBOX = 12,
	VOEMCRYPTO_CMD_IS_KEYBOX_VALID = 13,
	VOEMCRYPTO_CMD_GET_DEV_ID = 14,
	VOEMCRYPTO_CMD_GET_KEYDATA = 15,
	VOEMCRYPTO_CMD_GET_RANDOM = 16,
	VOEMCRYPTO_CMD_API_VER = 17,
	VOEMCRYPTO_CMD_SEC_LVL = 18,
	VOEMCRYPTO_CMD_REWRAP_DEV_RSA_KEY = 19,
	VOEMCRYPTO_CMD_LOAD_DEV_RSA_KEY = 20,
	VOEMCRYPTO_CMD_GEN_RSA_SIGN = 21,
	VOEMCRYPTO_CMD_DERIVE_KEYS_FROM_SESSION_KEY = 22,
	VOEMCRYPTO_CMD_GENERIC_ENCRYPT = 23,
	VOEMCRYPTO_CMD_GENERIC_DECRYPT = 24,
	VOEMCRYPTO_CMD_GENERIC_SIGN = 25,
	VOEMCRYPTO_CMD_GENERIC_VERIFY = 26,
	VOEMCRYPTO_CMD_MAX = 0xFFFFFFFF
};

/* IOCTL settings */
#define MAGIC_NUM 10
#define VOEMC_IOCTL_INIT _IOR(MAGIC_NUM, 1, uint32_t)
#define VOEMC_IOCTL_TERM _IOR(MAGIC_NUM, 2, uint32_t)
#define VOEMC_IOCTL_OPEN _IOR(MAGIC_NUM, 3, uint32_t)
#define VOEMC_IOCTL_CLOSE _IOR(MAGIC_NUM, 4, uint32_t)
#define VOEMC_IOCTL_GENNONCE _IOR(MAGIC_NUM, 5, uint32_t)
#define VOEMC_IOCTL_GENDERIVEDKEYS _IOR(MAGIC_NUM, 6, uint32_t)
#define VOEMC_IOCTL_GENSIGN _IOR(MAGIC_NUM, 7, uint32_t)
#define VOEMC_IOCTL_LOADKEYS _IOR(MAGIC_NUM, 8, uint32_t)
#define VOEMC_IOCTL_REFRESHKEYS _IOR(MAGIC_NUM, 9, uint32_t)
#define VOEMC_IOCTL_SELECTKEY _IOR(MAGIC_NUM, 10, uint32_t)
#define VOEMC_IOCTL_DECRYPTCTR _IOR(MAGIC_NUM, 11, uint32_t)
#define VOEMC_IOCTL_INSTALLKEYBOX _IOR(MAGIC_NUM, 12, uint32_t)
#define VOEMC_IOCTL_ISKEYBOXVALID _IOR(MAGIC_NUM, 13, uint32_t)
#define VOEMC_IOCTL_GETDEVICEID _IOR(MAGIC_NUM, 14, uint32_t)
#define VOEMC_IOCTL_GETKEYDATA _IOR(MAGIC_NUM, 15, uint32_t)
#define VOEMC_IOCTL_GETRANDOM _IOR(MAGIC_NUM, 16, uint32_t)
#define VOEMC_IOCTL_WRAPKEYBOX _IOR(MAGIC_NUM, 17, uint32_t)
#define VOEMC_IOCTL_REWRAPDEVRSAKEY _IOR(MAGIC_NUM, 18, uint32_t)
#define VOEMC_IOCTL_LOADDEVRSAKEY _IOR(MAGIC_NUM, 19, uint32_t)
#define VOEMC_IOCTL_GENRSASIGN _IOR(MAGIC_NUM, 20, uint32_t)
#define VOEMC_IOCTL_DERKEYSFROMSESSIONKEY _IOR(MAGIC_NUM, 21, uint32_t)
#define VOEMC_IOCTL_GENENCRYPT _IOR(MAGIC_NUM, 22, uint32_t)
#define VOEMC_IOCTL_GENDECRYPT _IOR(MAGIC_NUM, 23, uint32_t)
#define VOEMC_IOCTL_GENERICSIGN _IOR(MAGIC_NUM, 24, uint32_t)
#define VOEMC_IOCTL_GENVERIFY _IOR(MAGIC_NUM, 25, uint32_t)


struct voemc_generic_t {
	uint32_t cmd;
	uint32_t result;
};

struct voemc_init_t {
	uint32_t cmd;
	uint32_t result;
};

struct voemc_term_t {
	uint32_t cmd;
	uint32_t result;
};

struct voemc_open_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
};

struct voemc_close_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
};

struct voemc_gennonce_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint32_t nonce;
};

struct voemc_genderivedkeys_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *mac_key_context; /* phy addr to i/p data (shmem or kernel) */
	uint32_t mac_key_context_length;
	uint8_t *enc_key_context; /* phy addr to i/p data (shmem or kernel) */
	uint32_t enc_key_context_length;
};

struct voemc_gensign_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *message; /* phy addr to i/p data (shmem or kernel) */
	size_t message_length;
	uint8_t *signature; /* phy addr to o/p data (shmem or kernel) */
	size_t *signature_length;
};


struct voemc_key_object {
	uint8_t *key_id;
	size_t key_id_length;
	uint8_t *key_data_iv;
	uint8_t *key_data;
	size_t key_data_length;
	uint8_t *key_control_iv;
	uint8_t *key_control;
};


struct voemc_key_refresh_object {
	uint8_t *key_id;
	size_t key_id_length;
	uint8_t *key_control_iv;
	uint8_t *key_control;
};


struct voemc_loadkeys_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *message; /* phy addr to i/p data (shmem or kernel) */
	size_t message_length;
	uint8_t *signature; /* phy addr to i/p data (shmem or kernel) */
	size_t signature_length;
	uint8_t	*enc_mac_key_iv;
	uint8_t	*enc_mac_keys;
	size_t num_keys;
	struct voemc_key_object *key_array;
};


struct voemc_refreshkeys_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *message; /* phy addr to i/p data (shmem or kernel) */
	size_t	message_length;
	uint8_t *signature; /* phy addr to i/p data (shmem or kernel) */
	size_t	signature_length;
	size_t	num_keys;
	struct voemc_key_refresh_object *key_array;
};


struct voemc_selectkey_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *key_id; /* phy addr to i/p data (shmem or kernel) */
	size_t key_id_length;
};


enum voemc_buffer_type {
	voemc_buffer_type_clear,
	voemc_buffer_type_secure,
	voemc_buffer_type_direct
};

struct voemc_dest_buffer_desc {
	enum voemc_buffer_type type;
	union {
		struct { /* type == OEMCrypto_BufferType_Clear */
			uint8_t *address;
			size_t max_length;
		} clear;
		struct { /* type == OEMCrypto_BufferType_Secure */
			void *handle;
			size_t max_length;
			size_t offset;
		} secure;
		struct { /* type == OEMCrypto_BufferType_Direct */
			bool is_video;
		} direct;
	} buffer;
};


struct voemc_decryptctr_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *data_addr;
	size_t data_length;
	bool is_encrypted;
	uint8_t iv[16];
	size_t block_offset;
	struct voemc_dest_buffer_desc *out_buffer;
	uint8_t subsample_flags;
};

struct voemc_installkeybox_t {
	uint32_t cmd;
	uint32_t result;
	uint8_t *keybox; /* to backend */
	size_t key_box_length;
};

struct voemc_iskeyboxvalid_t {
	uint32_t cmd;
	uint32_t result;
};

struct voemc_getdevid_t {
	uint32_t cmd;
	uint32_t result;
	uint8_t *device_id;
	size_t id_length;
};

struct voemc_getkeydata_t {
	uint32_t cmd;
	uint32_t result;
	uint8_t *key_data;
	size_t key_data_length;
};

struct voemc_getrandom_t {
	uint32_t cmd;
	uint32_t result;
	uint8_t *random_data;
	size_t data_length;
};

struct voemc_wrapkeybox_t {
	uint32_t cmd;
	uint32_t result;
	uint8_t *keybox; /* to backend */
	size_t keybox_length;
	uint8_t *wrapped_keybox;
	size_t wrapped_keybox_length;
	uint8_t *transport_key;
	size_t transport_key_length;
};

struct voemc_rewrapdevrsakey_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *message;
	size_t message_length;
	uint8_t *signature;
	size_t signature_length;
	uint32_t *nonce;
	uint8_t *enc_rsa_key;
	size_t enc_rsa_key_length;
	uint8_t *enc_rsa_key_iv;
	uint8_t *wrapped_rsa_key;
	size_t  *wrapped_rsa_key_length;
};

struct voemc_loaddevrsakey_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *wrapped_rsa_key;
	size_t wrapped_rsa_key_length;
};

struct voemc_genrsasign_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *message;
	size_t message_length;
	uint8_t *signature;
	size_t signature_length;
};

struct voemc_derkeysfrsessionkey_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *enc_session_key;
	size_t enc_session_key_length;
	uint8_t *mac_key_context;
	size_t mac_key_context_length;
	uint8_t *enc_key_context;
	size_t enc_key_context_length;
};

struct voemc_genencrypt_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *in_buffer;
	size_t buffer_length;
	uint8_t iv[16];
	uint32_t algorithm;
	uint8_t *out_buffer;
};

struct voemc_gendecrypt_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *in_buffer;
	size_t buffer_length;
	uint8_t iv[16];
	uint32_t algorithm;
	uint8_t *out_buffer;
};

struct voemc_genericsign_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *in_buffer;
	size_t buffer_length;
	uint32_t algorithm;
	uint8_t *signature;
	size_t signature_length;
};

struct voemc_genverify_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	uint8_t *in_buffer;
	size_t buffer_length;
	uint32_t algorithm;
	uint8_t *signature;
	size_t signature_length;
};

struct voemc_apiversion_t {
	uint32_t cmd;
	uint32_t version;
};

struct voemc_securitylevel_t {
	uint32_t cmd;
	char *level;
};


/*****************************************************************************/
/* Licence Description                                                       */
/*****************************************************************************/

MODULE_DESCRIPTION("VLX Virtual OEM Crypto Front End Driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL");

/*****************************************************************************/
#endif /* VOEMCRYPTO_COMMON_H */
/*****************************************************************************/

/*----- End of file -----*/


