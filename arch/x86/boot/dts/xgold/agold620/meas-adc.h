/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MEAS_ADC_H
#define _MEAS_ADC_H

#define ADC_HAL_AVG_SAMPLE_LEVEL_LOW    0
#define ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM 1
#define ADC_HAL_AVG_SAMPLE_LEVEL_HIGH   2

#define ADC_V_BAT_TYP 0
#define ADC_V_BAT_MIN 1
#define ADC_V_BAT_OCV 2
#define ADC_V_CHR_NOM 3
#define ADC_V_CHR_USB 4
#define ADC_I_BAT 5
#define ADC_I_CHR 6
#define ADC_I_HW 7
#define ADC_T_BAT_0 8
#define ADC_T_BAT_1 9
#define ADC_T_DBB_IC_0 10
#define ADC_T_DBB_IC_1 11
#define ADC_T_PMIC_IC_0 12
#define ADC_T_PMIC_IC_1 13
#define ADC_T_RF 14
#define ADC_T_PCB 15
#define ADC_T_REF 16
#define ADC_T_SYS_0 17
#define ADC_T_SYS_1 18
#define ADC_T_SYS_2 19
#define ADC_T_CHR_IC 20
#define ADC_ID_BAT 21
#define ADC_ID_PCB 22
#define ADC_ID_ACC 23
#define ADC_ANAMON 24
#define ADC_MAX_NO_OF_CHANNELS 25

#define ADC_PHY_OFF 0
#define ADC_PHY_M0 1
#define ADC_PHY_M1 2
#define ADC_PHY_M2 3
#define ADC_PHY_M3 4
#define ADC_PHY_M4 5
#define ADC_PHY_M5 6
#define ADC_PHY_M6 7
#define ADC_PHY_M7 8
#define ADC_PHY_M8 9
#define ADC_PHY_M9 10
#define ADC_PHY_M10 11
#define ADC_PHY_M11 12
#define ADC_PHY_M12 13
#define ADC_PHY_M13 14
#define ADC_PHY_M15 15

#define IIO_VOLTAGE     0
#define IIO_CURRENT     1
#define IIO_POWER       2
#define IIO_ACCEL       3
#define IIO_ANGL_VEL    4
#define IIO_MAGN        5
#define IIO_LIGHT       6
#define IIO_INTENSITY   7
#define IIO_PROXIMITY   8
#define IIO_TEMP        9
#define IIO_INCLI       10
#define IIO_ROT         11
#define IIO_ANGL        12
#define IIO_TIMESTAMP   13
#define IIO_CAPACITANCE 14
#define IIO_ALTVOLTAGE  15
#define IIO_CCT         16
#define IIO_PRESSURE    17
#define IIO_HUMIDITYRELATIVE 18
#define IIO_RESISTANCE       19
#define IIO_COMPOSITE        20

#endif
