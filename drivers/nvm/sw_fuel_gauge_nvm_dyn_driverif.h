/*
 ****************************************************************
 *
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
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

#ifndef _SW_FUEL_GAUGE_NVM_DYN_DRIVERIF_H
#define _SW_FUEL_GAUGE_NVM_DYN_DRIVERIF_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*
** =============================================================================
**
**                              INTERFACE DESCRIPTION
**
** =============================================================================
*/

/**
 *  @file sw_fuel_gauge_nvm_dyn_driverif.h
 *   This  file contains nvm data information for software fuel gauge
 *   modules.
 */

/*
** =============================================================================
**
**                              INCLUDE STATEMENTS
**
** =============================================================================
*/
#include "nvm/bastypes.h"           /* Basic types */

/*
** =============================================================================
**
**                                   DEFINES
**
** =============================================================================
*/

/*
** =============================================================================
**
**                              ENUMERATIONS
**
** =============================================================================
*/

/*
** =============================================================================
**
**                  SOFTWARE FUEL GAUGE NVM STRUCTURE
**
** =============================================================================
*/

/** @brief
 * Structure for holding NVM data for software fuel gauge calibration point
 */
typedef struct {
	S32 cc_charge_mc;
	S32 cc_discharge_mc;
	S32 cc_balanced_mc;
	S16 cc_error_mc;
	long long rtc_time_sec;
	S16 soc_permil;
	S16 soc_error_permil;
	U16 qmax_n_minus_2;
	S16 qmax_n_minus_2_err_permil;
	U16 qmax_n_minus_1;
	S16 qmax_n_minus_1_err_permil;
	U16 checksum;
} T_SOC_CAL_PNT_NVM;

/** @brief
* Structure for holding current SoC data
*/
struct soc_nvm {
	U32 base_soc;
	U32 last_reported_soc;
	U16 checksum;
};

/* This type shall be included in nvm_cfg.h by the storage team for
 * software fuel gauge module
 */
typedef struct {
	T_SOC_CAL_PNT_NVM soc_cal_pnt_nvm_dyn;
	struct soc_nvm soc_data;
} T_SW_FUEL_GAUGE_NVM_DYN;

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* _SW_FUEL_GAUGE_NVM_DYN_DRIVERIF_H */
/*!
 *  \}
 */
/* End of file. */
