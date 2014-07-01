/*
 ****************************************************************
 *
 *  Component: Virtual Miscellaneous RPC Front-End
 *
 *  Copyright (C) 2011 Intel Mobile Communications GmbH
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
 ****************************************************************
 */

#ifndef _VRPC_MISC_COMMON_H_
#define _VRPC_MISC_COMMON_H_

#define	VMISC_VRPC_NAME	"vmisc"

// it is not 4096 due to the 20 bytes overhead 
// in vrpc & vmisc_rpc_req_t
#define MAX_KPANIC_SIZE ((64*1024)-20)

// VMISC RPC request
typedef struct vmisc_rpc_req_t {
    nku32_f cmd;	// command
    nku32_f arg;	// argument (optional)
} vmisc_rpc_req_t;

// VMISC RPC result
typedef struct vmisc_rpc_res_t {
    nku32_f res;	// result
    nku32_f value;	// value
} vmisc_rpc_res_t;

typedef enum {
    VMISC_RPC_CMD_LOG_KPANIC,
    VMISC_RPC_CMD_MAX
} vmisc_rpc_cmd_t;

#define VMISC_RPC_CMD_NAME {"log_kpanic"}

#endif /* _VRPC_MISC_COMMON_H_ */
