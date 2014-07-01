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

#define VOEMCRYPTO_VRPC_NAME "voemcrypto"



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
	VOEMCRYPTO_CMD_MAX
};

struct voemcrypto_req_t {
	enum voemcrypto_cmd_t cmd;
	unsigned int arg[15];
};


#endif /* VOEMCRYPTO_COMMON_H */
