/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _HIRQ_IRQ_H
#define _HIRQ_IRQ_H

/* HIRQ nodes from HIRQ domain point of view */
#define HIRQ_VMM_BROADCAST 0
#define HIRQ_PMIC_TEST 1
#define HIRQ_HIRQ_TEST 2
#define HIRQ_PMIC_PRH_SIGNALING 3
/* The order of PMIC HIRQ's should match their bit-position in IRQLVL1_REG */
#define HIRQ_PMIC_POWER_BUTTON 4
#define HIRQ_PMIC_TIME_UNIT 5
#define HIRQ_PMIC_THERMAL_UNIT 6
#define HIRQ_PMIC_BCU 7
#define HIRQ_PMIC_ADC 8
#define HIRQ_PMIC_CHARGER 9
#define HIRQ_PMIC_GPIO 10
#define HIRQ_PMIC_CRITICAL_EVENT 11
/* Please do not change the order of PMIC HIRQ's */
#define HIRQ_PMIC_VTIMER 12

#endif /* _HIRQ_IRQ_H */
