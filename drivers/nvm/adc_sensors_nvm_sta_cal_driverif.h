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

#ifndef _ADC_SENSORS_NVM_STA_FIX_DRIVERIF_H
#define _ADC_SENSORS_NVM_STA_FIX_DRIVERIF_H

#ifdef __cplusplus
    extern "C" {
#endif // __cplusplus

/*
** =============================================================================
**
**                              INTERFACE DESCRIPTION
**
** =============================================================================
*/

/**
 *  @file intel_adc_nvm_sta_fix_driverif.h
 *   This  file contains nvm data information for ADC sensors
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
#define ADC_SENSORS_CHANNEL_MAX     12

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
**                  ADC SENSORS NVM STRUCTURE
**
** =============================================================================
*/

/** @brief
 * Structure of calibration data for each ADC channel
 */
typedef struct {
  U32 gain;
  S32 offset;
  U32 shift;
} adc_sensors_calibration;

/** @brief
 * Structure for holding NVM data for ADC sensors
 */
typedef struct {
  adc_sensors_calibration  adc_sensors_calibration_data[ADC_SENSORS_CHANNEL_MAX];
} T_ADC_SENSORS_NVM;

/* This type shall be included in nvm_cfg.h by the storage team for ADC sensors module */
typedef struct {
  T_ADC_SENSORS_NVM adc_nvm_sta;
} T_ADC_SENSORS_NVM_STA_CAL;

#ifdef __cplusplus
    }
#endif  // __cplusplus

#endif /* _ADC_SENSORS_NVM_STA_FIX_DRIVERIF_H */
/*!
 *  \}
 */
/* End of file. */
