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

/*======================================================================== */
/* TYPES & DEFINITIONS */
/* ======================================================================== */

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
	VOEMCRYPTO_CMD_IOCTL_GETADDR = 27,
	VOEMCRYPTO_CMD_UPDATEUSAGETABLE = 28,
	VOEMCRYPTO_CMD_DEACTIVEUSAGEENTRY = 29,
	VOEMCRYPTO_CMD_REPORTUSAGE = 30,
	VOEMCRYPTO_CMD_DELETEUSAGEENTRY = 31,
	VOEMCRYPTO_CMD_DELETEUSAGETABLE = 32,
	/* API v10 */
	VOEMCRYPTO_CMD_QUERYKEYCONTROL = 33,
	VOEMCRYPTO_CMD_COPYBUFFER = 34,
	VOEMCRYPTO_CMD_LOADTESTKEYBOX = 35,
	VOEMCRYPTO_CMD_GETHDCPCAPABILITY = 36,
	VOEMCRYPTO_CMD_ISANTIROLLBACKHWPRESENT = 37,
	VOEMCRYPTO_CMD_GETNUMBEROFOPENSESSIONS = 38,
	VOEMCRYPTO_CMD_GETMAXNUMBEROFSESSIONS = 39,
	VOEMCRYPTO_CMD_FORCEDELETEUSAGEENTRY = 40,
	/* Classic */
	VOEMCRYPTO_CMD_WVC_DECRYPT_VIDEO = 50, /* Widevine classic */
	VOEMCRYPTO_CMD_WVC_DECRYPT_AUDIO = 51, /* Widevine classic */
	VOEMCRYPTO_CMD_WVC_SET_ENTITLEMENT_KEY = 52, /* Widevine classic */
	VOEMCRYPTO_CMD_WVC_DERIVE_CONTROL_WORD = 53, /* Widevine classic */
	VOEMCRYPTO_CMD_MAX = 0xFFFFFFFF
};

/* IOCTL settings */
#define MAGIC_NUM 10
#define VOEMC_IOCTL_INIT \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_INITIALIZE, uint32_t)
#define VOEMC_IOCTL_TERM \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_TERMINATE, uint32_t)
#define VOEMC_IOCTL_OPEN \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_OPEN_SESSION, uint32_t)
#define VOEMC_IOCTL_CLOSE \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_CLOSE_SESSION, uint32_t)
#define VOEMC_IOCTL_GENDERIVEDKEYS \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GEN_DERIVED_KEYS, uint32_t)
#define VOEMC_IOCTL_GENNONCE \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GEN_NONCE, uint32_t)
#define VOEMC_IOCTL_GENSIGN \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GEN_SIGN, uint32_t)
#define VOEMC_IOCTL_LOADKEYS \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_LOAD_KEYS, uint32_t)
#define VOEMC_IOCTL_REFRESHKEYS \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_REFRESH_KEYS, uint32_t)
#define VOEMC_IOCTL_SELECTKEY \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_SELECT_KEY, uint32_t)
#define VOEMC_IOCTL_DECRYPTCTR \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_DECRYPT_CTR, uint32_t)
#define VOEMC_IOCTL_INSTALLKEYBOX \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_INSTALL_KEYBOX, uint32_t)
#define VOEMC_IOCTL_ISKEYBOXVALID \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_IS_KEYBOX_VALID, uint32_t)
#define VOEMC_IOCTL_GETDEVICEID \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GET_DEV_ID, uint32_t)
#define VOEMC_IOCTL_GETKEYDATA \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GET_KEYDATA, uint32_t)
#define VOEMC_IOCTL_GETRANDOM \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GET_RANDOM, uint32_t)
#define VOEMC_IOCTL_APIVER \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_API_VER, uint32_t)
#define VOEMC_IOCTL_SEC_LVL \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_SEC_LVL, uint32_t)
#define VOEMC_IOCTL_WRAPKEYBOX \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_WRAP_KEYBOX, uint32_t)
#define VOEMC_IOCTL_REWRAPDEVRSAKEY \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_REWRAP_DEV_RSA_KEY, uint32_t)
#define VOEMC_IOCTL_LOADDEVRSAKEY \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_LOAD_DEV_RSA_KEY, uint32_t)
#define VOEMC_IOCTL_GENRSASIGN \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GEN_RSA_SIGN, uint32_t)
#define VOEMC_IOCTL_DERKEYSFROMSESSIONKEY \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_DERIVE_KEYS_FROM_SESSION_KEY, uint32_t)
#define VOEMC_IOCTL_GENENCRYPT \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GENERIC_ENCRYPT, uint32_t)
#define VOEMC_IOCTL_GENDECRYPT \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GENERIC_DECRYPT, uint32_t)
#define VOEMC_IOCTL_GENERICSIGN \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GENERIC_SIGN, uint32_t)
#define VOEMC_IOCTL_GENVERIFY \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GENERIC_VERIFY, uint32_t)
#define VOEMC_IOCTL_GETADDR \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_IOCTL_GETADDR, uint32_t)
#define VOEMC_IOCTL_UPDATEUSAGETABLE \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_UPDATEUSAGETABLE, uint32_t)
#define VOEMC_IOCTL_DEACTIVEUSAGEENTRY \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_DEACTIVEUSAGEENTRY, uint32_t)
#define VOEMC_IOCTL_REPORTUSAGE \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_REPORTUSAGE, uint32_t)
#define VOEMC_IOCTL_DELETEUSAGEENTRY \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_DELETEUSAGEENTRY, uint32_t)
#define VOEMC_IOCTL_DELETEUSAGETABLE \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_DELETEUSAGETABLE, uint32_t)
/* API v10 */
#define VOEMC_IOCTL_QUERYKEYCONTROL \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_QUERYKEYCONTROL, uint32_t)
#define VOEMC_IOCTL_COPYBUFFER \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_COPYBUFFER, uint32_t)
#define VOEMC_IOCTL_LOADTESTKEYBOX \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_LOADTESTKEYBOX, uint32_t)
#define VOEMC_IOCTL_GETHDCPCAPABILITY \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GETHDCPCAPABILITY, uint32_t)
#define VOEMC_IOCTL_ISANTIROLLBACKHWPRESENT \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_ISANTIROLLBACKHWPRESENT, uint32_t)
#define VOEMC_IOCTL_GETNUMBEROFOPENSESSIONS \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GETNUMBEROFOPENSESSIONS, uint32_t)
#define VOEMC_IOCTL_GETMAXNUMBEROFSESSIONS \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_GETMAXNUMBEROFSESSIONS, uint32_t)
#define VOEMC_IOCTL_FORCEDELETEUSAGEENTRY \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_FORCEDELETEUSAGEENTRY, uint32_t)
/* Classic */
#define VOEMC_IOCTL_WVC_DECRYPT_VIDEO \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_WVC_DECRYPT_VIDEO, uint32_t)
#define VOEMC_IOCTL_WVC_DECRYPT_AUDIO \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_WVC_DECRYPT_AUDIO, uint32_t)
#define VOEMC_IOCTL_WVC_SET_ENTITLEMENT_KEY \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_WVC_SET_ENTITLEMENT_KEY, uint32_t)
#define VOEMC_IOCTL_WVC_DERIVE_CONTROL_WORD \
	_IOR(MAGIC_NUM, VOEMCRYPTO_CMD_WVC_DERIVE_CONTROL_WORD, uint32_t)

struct voemc_input_buffer_t {
	uint32_t size;
	uint32_t baddr;
	uint32_t phys;
	uint32_t phys_size;
};
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
	const uint8_t *mac_key_context; /* phy addr to i/p data (sh. or ke.) */
	uint32_t mac_key_context_length;
	const uint8_t *enc_key_context; /* phy addr to i/p data (sh. or ke.) */
	uint32_t enc_key_context_length;
};

struct voemc_gensign_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *message; /* phy addr to i/p data (shmem or kernel) */
	size_t message_length;
	uint8_t *signature; /* phy addr to o/p data (shmem or kernel) */
	size_t *signature_length;
};

struct voemc_key_object_t {
	uint8_t *key_id;
	size_t   key_id_length;
	uint8_t *key_data_iv;
	uint8_t *key_data;
	size_t   key_data_length;
	uint8_t *key_control_iv;
	uint8_t *key_control;
};

struct voemc_key_refresh_object_t {
	const uint8_t *key_id;
	size_t         key_id_length;
	const uint8_t *key_control_iv;
	const uint8_t *key_control;
};

struct voemc_loadkeys_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *message; /* phy addr to i/p data (shmem or kernel) */
	size_t message_length;
	const uint8_t *signature; /* phy addr to i/p data (shmem or kernel) */
	size_t signature_length;
	const uint8_t *enc_mac_key_iv;
	const uint8_t *enc_mac_keys;
	size_t num_keys;
	const struct voemc_key_object_t *key_array;
	const uint8_t *pst;
	size_t pst_length;
};

struct voemc_refreshkeys_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *message; /* phy addr to i/p data (shmem or kernel) */
	size_t  message_length;
	const uint8_t *signature; /* phy addr to i/p data (shmem or kernel) */
	size_t  signature_length;
	size_t  num_keys;
	const struct voemc_key_refresh_object_t *key_array;
};

struct voemc_selectkey_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *key_id; /* phy addr to i/p data (shmem or kernel) */
	size_t key_id_length;
};

/* Use this for vbpipe */
struct voemc_selectkey_pipe_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	size_t key_id_length;
	uint8_t key_id[64 - sizeof(size_t) - sizeof(uint32_t)*3];
};

/* Struct for widevine classic OEMCrypto_DecryptVideo */
struct voem_wvc_decrypto_video_t {
	uint32_t cmd;
	uint32_t result;
	uint8_t iv[16];
	uint8_t *input;
	uint32_t inputLength;
	uint32_t output_handle;
	uint32_t output_offset;
	uint32_t *outputLength;
	uint8_t align64[64 - sizeof(uint32_t)*6 -
		sizeof(uint8_t)*2 - sizeof(uint8_t)*16];
};

/* Struct for widevine classic OEMCrypto_DecryptAudio */
struct voem_wvc_decrypto_audio_t {
	uint32_t cmd;
	uint32_t result;
	uint8_t iv[16];
	uint8_t *input;
	uint32_t inputLength;
	uint32_t output_handle;
	uint32_t *outputLength;
	uint8_t align64[64 - sizeof(uint32_t)*5 -
		sizeof(uint8_t)*2 - sizeof(uint8_t)*16];
};

enum voemc_buffer_type {
	voemc_buffer_type_clear,
	voemc_buffer_type_secure,
	voemc_buffer_type_direct
};

struct voemc_dest_buffer_desc_t {
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
	const uint8_t *data_addr;
	size_t data_length;
	bool is_encrypted;
	uint8_t iv[16];
	size_t block_offset;
	const struct voemc_dest_buffer_desc *out_buffer;
	uint8_t subsample_flags;
};

struct voemc_decryptctr_mini_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *data_addr;
	size_t data_length;
	bool is_encrypted;
	uint8_t iv[16];
	size_t block_offset;
	uint8_t subsample_flags;
};

struct voemc_decryptctr_transfer_t {
	struct voemc_decryptctr_mini_t req;
	struct voemc_dest_buffer_desc_t dest_buffer_desc;
};

struct voemc_installkeybox_t {
	uint32_t cmd;
	uint32_t result;
	const uint8_t *keybox; /* to backend */
	size_t keybox_length;
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
	const uint8_t *keybox; /* to backend */
	size_t keybox_length;
	uint8_t *wrapped_keybox;
	size_t wrapped_keybox_length;
	const uint8_t *transport_key;
	size_t transport_key_length;
};

struct voemc_rewrapdevrsakey_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *message;
	size_t message_length;
	const uint8_t *signature;
	size_t signature_length;
	const uint32_t *nonce;
	const uint8_t *enc_rsa_key;
	size_t enc_rsa_key_length;
	const uint8_t *enc_rsa_key_iv;
	uint8_t *wrapped_rsa_key;
	size_t  *wrapped_rsa_key_length;
};

struct voemc_loaddevrsakey_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *wrapped_rsa_key;
	size_t wrapped_rsa_key_length;
};

struct voemc_genrsasign_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *message;
	uint32_t algorithm;
	size_t message_length;
	uint8_t *signature;
	size_t signature_length;
};

struct voemc_derkeysfrsessionkey_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *enc_session_key;
	size_t enc_session_key_length;
	const uint8_t *mac_key_context;
	size_t mac_key_context_length;
	const uint8_t *enc_key_context;
	size_t enc_key_context_length;
};

struct voemc_genencrypt_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *in_buffer;
	size_t buffer_length;
	uint8_t iv[16];
	uint32_t algorithm;
	uint8_t *out_buffer;
};

struct voemc_gendecrypt_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *in_buffer;
	size_t buffer_length;
	uint8_t iv[16];
	uint32_t algorithm;
	uint8_t *out_buffer;
};

struct voemc_genericsign_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *in_buffer;
	size_t buffer_length;
	uint32_t algorithm;
	uint8_t *signature;
	size_t signature_length;
};

struct voemc_genverify_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *in_buffer;
	size_t buffer_length;
	uint32_t algorithm;
	const uint8_t *signature;
	size_t signature_length;
};

struct voemc_apiversion_t {
	uint32_t cmd;
	uint32_t version;
};

struct voemc_securitylevel_t {
	uint32_t cmd;
	const char *level;
};

struct voem_input_buffer {
	uint32_t size;
	uint32_t baddr;
	uint32_t phys;
	uint32_t phys_size;
};

struct voemc_update_usage_table_t {
	uint32_t cmd;
	uint32_t result;
};

struct voemc_deactive_usage_entry_t {
	uint32_t cmd;
	uint32_t result;
	char *pst;
	int pst_length;
};

struct voemc_report_usage_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const char *pst;
	int pst_length;
	char *rpt_buffer;
	size_t rpt_buffer_length;
};

struct voemc_delete_usage_entry_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *pst;
	size_t pst_length;
	const uint8_t *message;
	size_t message_length;
	const uint8_t *signature;
	size_t signature_length;
};

struct  voemc_delete_usage_table_t {
	uint32_t cmd;
	uint32_t result;
};

/* Struct for widevine classic OEMCrypto_DecryptVideo */
struct voem_wvc_decrypt_video_t {
	uint32_t cmd;
	uint32_t result;
	uint8_t iv[16];
	const uint8_t *input;
	uint32_t input_length;
	uint32_t output_handle;
	uint32_t output_offset;
	uint32_t pad_4bytes;
	uint32_t output_length;
	uint8_t align64[64 - sizeof(uint32_t)*6 -
		sizeof(uint8_t)*16 - sizeof(void *)*2];
};

/* Struct for widevine classic OEMCrypto_DecryptAudio */
struct voem_wvc_decrypt_audio_t {
	uint32_t cmd;
	uint32_t result;
	uint8_t iv[16];
	const uint8_t *input;
	uint32_t input_length;
	uint32_t pad_4bytes;
	uint8_t *output;
	uint32_t output_length;
	uint8_t align64[64 - sizeof(uint32_t)*4 -
		sizeof(uint8_t)*16 - sizeof(void *)*3];
};

/* Struct for widevine classic OEMCrypto_SetEntitlementKey */
struct voem_wvc_set_entitlement_key_t {
	uint32_t cmd;
	uint32_t result;
	uint8_t *emm_key;
	uint32_t emm_key_length;
	uint8_t align64[64 - sizeof(uint32_t)*3 - sizeof(uint8_t)];
};

/* Struct for widevine classic OEMCrypto_SetEntitlementKey */
struct voem_wvc_derive_control_word_t {
	uint32_t cmd;
	uint32_t result;
	uint8_t *ecm;
	uint32_t length;
	uint32_t *flags;
	uint8_t align64[64 - sizeof(uint32_t)*4 - sizeof(uint8_t)];
};

/*****************************************************************************/
/* Widevine API Level 10 additions                                           */
/*****************************************************************************/
enum OEMCrypto_HDCP_Capability {
	HDCP_NONE = 0, /* No HDCP supported, no secure data path. */
	HDCP_V1 = 1, /* HDCP version 1.0 */
	HDCP_V2 = 2, /* HDCP version 2.0 */
	HDCP_V2_1 = 3, /* HDCP version 2.1 */
	HDCP_V2_2 = 4, /* HDCP version 2.2 */
	HDCP_NO_DIGITAL_OUTPUT = 0xff /* No digital output. */
};

struct voemc_gethdcpcapability_t {
	uint32_t cmd;
	uint32_t result;
	enum OEMCrypto_HDCP_Capability *current_v;
	enum OEMCrypto_HDCP_Capability *maximum;
};

struct voemc_querykeycontrol_t {
	uint32_t cmd;
	uint32_t result;
	uint32_t session;
	const uint8_t *key_id;
	uint32_t key_id_length;
	uint8_t *key_control_block;
	size_t *key_control_block_length;
};

struct voemc_copybuffer_t {
	uint32_t cmd;
	uint32_t result;
	const uint8_t *data_addr;
	size_t data_length;
	struct voemc_dest_buffer_desc_t *out_buffer;
	uint8_t subsample_flags;
};

struct voemc_loadtestkeybox_t {
	uint32_t cmd;
	uint32_t result;
};

struct voemc_isantirollbackhwpresent_t {
	uint32_t cmd;
	uint32_t result;
	bool *is_present;
};

struct voemc_getnumberofopensessions_t {
	uint32_t cmd;
	uint32_t result;
	size_t *count;
};

struct voemc_getmaxnumberofsessions_t {
	uint32_t cmd;
	uint32_t result;
	size_t *max;
};

struct voemc_forcedeleteusageentry_t {
	uint32_t cmd;
	uint32_t result;
	const void __user *pst;
	size_t pst_length;
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

