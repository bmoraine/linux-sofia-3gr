/*
* Copyright (C) 2015 Intel Mobile Communications GmbH
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
*
*/

#if !defined(VD_STP_PROTOCOL_H)
#define VD_STP_PROTOCOL_H

/**
@addtogroup <DOXYGEN_GROUP>
@{
  < module description >
  ...
*/
/*****************************************************************************/
/* DEFINITIONS                                                          */
/*****************************************************************************/
/* The following are put here as they may be required by CDS.
    Alternatively CDD_STP_Protocol should be split
	into MIPI send and Core dump STP protocol parts.
    CDS will also send using the same protocol, but
	the interface will be OCT and it would obviously be
    preferable to have only one core dump protocol encoder*/

/* Leave bottom byte of magic number zero
	as it is used for Core dump message type. */

/* ASCII magic number = CD! */
#define VD_STP_HDR_MAGIC_NUMBER           0x43442100
#define VD_STP_HDR_MSG_TYPE_HEADER        0x00
#define VD_STP_HDR_MSG_TYPE_INFO          0x01
#define VD_STP_HDR_MSG_TYPE_DATA          0x02
#define VD_STP_HDR_MSG_TYPE_TERMINATOR    0xFF

#define VD_STP_HEADER_SIZE                12
#define VD_STP_DUMP_REGION_HEADER_SIZE    8
#define VD_STP_HDR_MAG_NUM_BPOSN          0
#define VD_STP_HDR_TYPE_BPOSN             3
#define VD_STP_HDR_SEG_SEQ_BPOSN          4
#define VD_STP_HDR_FRAME_NUM_BPOSN        7
#define VD_STP_HDR_FRAME_SIZE_BPOSN       8

#define VD_STP_CHID_CORE_DUMP             250
#define VD_STP_CHID_CORE_DUMP_INFO        251

/*****************************************************************************/
/* TYPE DEFINITIONS                                                          */
/*****************************************************************************/

/*============================================================================
FUNCTION PROTOTYPES
============================================================================*/
void VD_stp_start(struct cd_sharemem *cd_info);

#endif /* !defined(VD_STP_PROTOCOL_H) */


