/* ----------------------------------------------------------------------------
   Copyright (C) 2014 Intel Mobile Communications GmbH

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

  ---------------------------------------------------------------------------*/
/*
 * NOTES:
 * 1) This source file is included in guests including Linux and purposely
 * kept in line with Linux kernel coding guidelines for ease of portability and
 * maintainability. See linux/Documentation/CodingStyle for coding guidelines.
 * Please check all further changes are compliant with Linux coding standard
 * and clear all commands reported by Linux checkpatch.pl
 * Use command: linux/scripts/checkpatch.pl -f <filename>
 * Clear ALL warnings and errors reported.
 *
 * 2) Use only C99 fixed width types for definition as this header needs to be
 * both 32bit/64bit portable.
 * Avoid the use of pointers/enum in structure as that make the structure
 * variable size based on 32/64bit toolchain used.
*/

#ifndef _VMM_GUEST_IPC_H
#define _VMM_GUEST_IPC_H

#ifdef __KERNEL__
#include "linux/types.h"
#else
#include "stdint.h"
#endif

#define vmm_paddr_t uint32_t

/** @typedef vmm_vlink_state
 *  @brief indicate the current status of a vlink
 */
enum vmm_vlink_state {
	VLINK_INIT,
	VLINK_OFF,
	VLINK_ON
};

/** @brief Vlink structure
 *
 *  Used to store information of a vlink between
 *  server(destination) and client(caller, source) OS
 */
struct vmm_vlink {
	/** unique name identifier */
	vmm_paddr_t   name;
	/** logical vlink id */
	uint32_t      link;
	/** os id of guest who is server */
	uint32_t      s_id;
	/** additional info for server setup */
	vmm_paddr_t   s_info;
	/** server end state */
	uint32_t      s_state;
	/** os id of guest who is client */
	uint32_t      c_id;
	/** additional info for client setup */
	vmm_paddr_t   c_info;
	/** client end state */
	uint32_t      c_state;
	/** physical address of the next vlink */
	vmm_paddr_t   next;
};

#endif /* _VMM_GUEST_IPC_H */
