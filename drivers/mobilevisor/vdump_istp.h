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

#if !defined(VD_ISTP_PROTOCOL_H)
#define VD_ISTP_PROTOCOL_H

/**
@addtogroup <DOXYGEN_GROUP>
@{
  < module description >
  ...
*/
/*****************************************************************************/
/* DEFINITIONS                                                          */
/*****************************************************************************/


/*****************************************************************************/
/* TYPE DEFINITIONS                                                          */
/*****************************************************************************/

/*============================================================================
FUNCTION PROTOTYPES
============================================================================*/

void VD_Istp_SendDataEnd(void);
void VD_Istp_SendData(unsigned char *pData, unsigned int length);
char VD_Istp_init(unsigned short ccid);
void VD_Istp_deinit(void);
void VD_Istp_setCID(unsigned short ccid);

#endif /* !defined(VD_ISTP_PROTOCOL_H) */


