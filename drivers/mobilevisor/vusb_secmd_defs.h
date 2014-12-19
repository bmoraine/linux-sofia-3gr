/*
 ****************************************************************
 *
 *  Component: Virtual VUSB SIO Extender Front End
 *
 *  Copyright (C) 2011 - 2012 Intel Mobile Communications GmbH
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
 *
 ****************************************************************
 */

/*****************************************************************************/
/*****************************************************************************/
/**                                                                         **/
/**     NOTE: THE CONTENT OF THIS FILE IS SHARED IN COMMON BETWEEN THE      **/
/**           VUSB BACK END AND FRONT END, AND ANY MAINTENANCE CHANGES      **/
/**           MADE IN EITHER MUST BE PROPAGATED INTO THE OTHER              **/
/**                                                                         **/
/**     FILE: (FE) /linux/linux-2.6-vdrivers/drivers/vlx/vusb_secmd_defs.h  **/
/**           (BE) /mhw_vlx_al_src/device/vusb/src/vusb_secmd_defs.h        **/
/**                                                                         **/
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
#if !defined(VUSB_SECMD_DEFS_H)
#define VUSB_SECMD_DEFS_H
/*****************************************************************************/

/*****************************************************************************/
/* DEFINES                                                                   */
/*****************************************************************************/

#define VUSB_SECMD_BUFFER_SIZE  256

#define VUSB_SECMD_MAX_MESSAGE  80
#define VUSB_SECMD_MAX_PORTION  20

#define VUSB_SECMD_MAX_LINKS    3

/************************************/
/* COMMAND STRINGS (GENERAL)        */
/************************************/

#define VUSB_SECMD_TXT_SEPARATOR                ":"

/************************************/
/* SETUP COMMAND STRINGS            */
/************************************/

#define VUSB_SECMD_TXT_NUM_LINKS                "NUM_LINKS"

#define VUSB_SECMD_TXT_LINK_DATA                "LINK_DATA"

/************************************/
/* Startup Acknowledgement          */
/************************************/

#define VUSB_SECMD_TXT_LINK                     "LINK"

#define VUSB_SECMD_TXT_START                    "START"

#define VUSB_SECMD_TXT_GOOD                     "SUCCESS"
#define VUSB_SECMD_TXT_FAIL                     "FAILED"

/************************************/
/* MODEM SIGNALS COMMAND STRINGS    */
/************************************/

#define VUSB_SECMD_TXT_MODEM                    "MODEM"

#define VUSB_SECMD_TXT_SIGNAL_DTR               "DTR"
#define VUSB_SECMD_TXT_SIGNAL_DSR               "DSR"
#define VUSB_SECMD_TXT_SIGNAL_RTS               "RTS"
#define VUSB_SECMD_TXT_SIGNAL_CTS               "CTS"
#define VUSB_SECMD_TXT_SIGNAL_DCD               "DCD"

#define VUSB_SECMD_TXT_STATE_ON                 "ON"
#define VUSB_SECMD_TXT_STATE_OFF                "OFF"

/*****************************************************************************/
/* Licence Description                                                       */
/*****************************************************************************/

MODULE_DESCRIPTION("VLX Virtual USB SIO Extender Front End Driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL");

/*****************************************************************************/
#endif /* VUSB_SECMD_DEFS_H */
/*****************************************************************************/
