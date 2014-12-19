/*
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
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
 */

#ifndef VPOWER__H
#define VPOWER__H

typedef enum {
	PRH_OK                              =  0,   /**< All things are ok */
	PRH_ERR_INV_USER_ID                 = -1,   /**< Invalid user driver identifier */
	PRH_ERR_INV_PER_ID                  = -2,   /**< Invalid HW peripheral identifier */
	PRH_ERR_INV_TYPE_ID                 = -3,   /**< Invalid HW peripheral type */
	PRH_ERR_INV_MODE_INFO               = -4,   /**< Invalid power mode information */
	PRH_ERR_INV_MODE_TRANSITION         = -5,   /**< Invalid power mode transition */
	PRH_ERR_INV_POINTER                 = -6,   /**< Invalid pointer */
	PRH_ERR_POW_DRV                     = -7,   /**< POW driver internal error */
	PRH_ERR_EMIF_DRV                    = -8,   /**< EBU driver internal error */
	PRH_ERR_OMP_DRV                     = -9,   /**< OMP driver internal error */
	PRH_ERR_SCU_DRV                     = -10,  /**< EBU driver internal error */
	PRH_ERR_PER_IN_USE                  = -11,  /**< HW peripheral set already in use */
	PRH_ERR_INIT                        = -12,  /**< PRH driver initialization error */
	PRH_ERR_LATE_INIT                   = -13,  /**< PRH driver late initialization error */
	PRH_ERR_INV_LOCK_VAL                = -14,  /**< PRH driver internal error */
	PRH_ERR_INV_CMD_VAL                 = -15,  /**< PRH driver internal error */
	PRH_ERR_INV_FUNC_PTR                = -16,  /**< PRH driver internal error */
	PRH_ERR_INTERNAL                    = -17,  /**< PRH driver internal error */

	/* For internal use only. Update last error code if the list above is changed. */
	PRH_ERR_LAST_ENUM = PRH_ERR_INTERNAL,

	/* Number error codes : -PRH_ERR_LAST_ENUM plus 1 for PRH_OK */
	PRH_ERR_NOF_ERROR_CODES = (-PRH_ERR_LAST_ENUM) + 1	/**< PRH driver number of error codes */
} ePRH_RETURN_T;

extern ePRH_RETURN_T vpower_call_prh(uint32_t, uint32_t,
					uint32_t * const, uint32_t);

#endif /* VPOWER_H */
