/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _HIRQ_IRQ_H
#define _HIRQ_IRQ_H

#define HIRQ_OFFSET 512

/* HIRQ nodes from HIRQ domain point of view */
#define HIRQ_VMM_BROADCAST		(512 - HIRQ_OFFSET)
#define HIRQ_PMIC_TEST			(513 - HIRQ_OFFSET)
#define HIRQ_HIRQ_TEST			(514 - HIRQ_OFFSET)
#define HIRQ_PRH_SIGNALING		(515 - HIRQ_OFFSET)
#define HIRQ_VTIMER				(516 - HIRQ_OFFSET)
#define HIRQ_SPCU_THERM0_LOW	(517 - HIRQ_OFFSET)
#define HIRQ_SPCU_THERM0_HIGH	(518 - HIRQ_OFFSET)
#define HIRQ_SPCU_THERM1_LOW	(519 - HIRQ_OFFSET)
#define HIRQ_SPCU_THERM1_HIGH	(520 - HIRQ_OFFSET)
#define HIRQ_RTC_ALARM			(521 - HIRQ_OFFSET)
#define HIRQ_CPU_CLK_NOTIFY		(522 - HIRQ_OFFSET)
#define HIRQ_MODEM_DVFS			(523 - HIRQ_OFFSET)
#define HIRQ_VM_VERIFY			(524 - HIRQ_OFFSET)
#define HIRQ_SPCU_THERM2_LOW	(525 - HIRQ_OFFSET)
#define HIRQ_SPCU_THERM2_HIGH	(526 - HIRQ_OFFSET)
#define HIRQ_SPCU_THERM3_LOW	(527 - HIRQ_OFFSET)
#define HIRQ_SPCU_THERM3_HIGH	(528 - HIRQ_OFFSET)
#define HIRQ_SPCU_THERM4_LOW	(529 - HIRQ_OFFSET)
#define HIRQ_SPCU_THERM4_HIGH	(530 - HIRQ_OFFSET)

#endif /* _HIRQ_IRQ_H */
