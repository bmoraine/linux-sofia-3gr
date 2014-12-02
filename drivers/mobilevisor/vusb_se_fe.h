/*
 *******************************************************************************
 *
 *  Component: VLX Virtual USB SIO Extender Front End (Kernel Mode)
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
 *******************************************************************************
 */

/*****************************************************************************/
#ifndef VUSB_SE_FE_H
#define VUSB_SE_FE_H
/*****************************************************************************/

/*****************************************************************************/
/* DEFINES                                                                   */
/*****************************************************************************/

#define VUSB_SE_FE_DATA_BUFSIZ  512

#define VUSB_SE_FE_LOG_SIZE     100

/************************/
/* LOCAL TRUE/FALSE     */
/************************/

#define VUSB_SE_FE_FALSE (0 == 1)

#define VUSB_SE_FE_TRUE  (0 == 0)

/************************/
/* LOGGING TAGS         */
/************************/

#define VUSB_SE_FE_NAME  "VUSB_SE_FE"

/*****************************************************************************/
/* DATA TYPES                                                                */
/*****************************************************************************/

/*****************************************************************************/
/* MACRO DEFINITIONS                                                         */
/*****************************************************************************/

/************************/
/* Logging              */
/************************/

    /*==================================================================*/
    /* NB: This logging (using printk) can *ONLY* be enabled in a build */
    /*     where the VCONS link is not activated. This is because the   */
    /*     SIO extender takes the VCONS traffic across from Linux to    */
    /*     MEX and if logging is active in the SIO Extender, then it    */
    /*     would recurse to log the logging                             */
    /*==================================================================*/

#ifdef CONFIG_VUSB_DEBUG

#define VUSB_SE_FE_INF(format, args...) \
	pr_info("%s(INF) (%d)" format "\n", VUSB_SE_FE_NAME, __LINE__, ## args)
#define VUSB_SE_FE_LOG(format, args...) \
	pr_info("%s(LOG) (%d)" format "\n", VUSB_SE_FE_NAME, __LINE__, ## args)
#define VUSB_SE_FE_ERR(format, args...) \
	pr_info("%s(ERR) (%d)" format "\n", VUSB_SE_FE_NAME, __LINE__, ## args)

#else

static char vusbsefe_log_buff[VUSB_SE_FE_LOG_SIZE];

#define VUSB_SE_FE_INF(format, args...) \
	snprintf(vusbsefe_log_buff, VUSB_SE_FE_LOG_SIZE, format, ## args);
#define VUSB_SE_FE_LOG(format, args...) \
	snprintf(vusbsefe_log_buff, VUSB_SE_FE_LOG_SIZE, format, ## args);
#define VUSB_SE_FE_ERR(format, args...) \
	snprintf(vusbsefe_log_buff, VUSB_SE_FE_LOG_SIZE, format, ## args);

#endif

/************************/
/* ARRAY SIZE           */
/************************/

#include <linux/kernel.h>

/*****************************************************************************/
/* EXPORTED FUNCTION PROTOTYPES                                              */
/*****************************************************************************/

/*****************************************************************************/
/* Licence Description                                                       */
/*****************************************************************************/

MODULE_DESCRIPTION("VLX Virtual USB SIO Extender Front End Driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL");

/*****************************************************************************/
#endif /* VUSB_SE_FE_H */
/*****************************************************************************/
