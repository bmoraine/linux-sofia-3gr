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

/*
** =============================================================================
**
**							INTERFACE DESCRIPTION
**
** =============================================================================
*/
/**
 * @file fmtrx_v4l2.h
 *
 * This	file contains interfaces that are V4L2 related.
 *
 **/

#ifndef	_FM_TRX_V4L2_H_
#define	_FM_TRX_V4L2_H_

/*
** =============================================================================
**
**							INCLUDE STATEMENTS
**
** =============================================================================
*/

/*
** =============================================================================
**
**							DEFINES
**
** =============================================================================
*/

/*
** =============================================================================
**
**						EXPORTED ENUM DEFINITIONS
**
** =============================================================================
*/

/*
** =============================================================================
**
**						LOCAL FUNCTION DECLARATIONS
**
** =============================================================================
*/

/*
** =============================================================================
**
**					EXPORTED FUNCTION DECLARATIONS
**
** =============================================================================
*/
/* Initialize the FMTRX V4L2 core
 * @dev Pointer to the device structure
 * @type Type of module requested
 */
int fmtrx_v4l2_init(
		struct device *dev,
		enum fmtrx_type type);

/* Uninitialize the FMTRX V4L2 core
 * @dev Pointer to the device structure
 */
int fmtrx_v4l2_deinit(
		struct device *dev,
		enum fmtrx_type type);

#endif	/* _FM_TRX_V4L2_H_	*/

