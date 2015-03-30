/*
 * Copyright (C) 2014-2015 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _SOFIA_3G_SOC_IRQ_H
#define _SOFIA_3G_SOC_IRQ_H

#ifndef CONFIG_X86_INTEL_SOFIA
/* HW nodes definitions based on HW attributions */
#include "xgold/sofia_3g_soc/irq-hw.h"
#else
/* Vectors nodes definitions based on VMM attributions */
#include "xgold/sofia_3g_soc/irq-vectors.h"
#include "xgold/sofia_3g_soc/irq-vectors-plf.h"
#endif

/* IRQ DOMAINS HEADERS */
#include "xgold/sofia_3g_soc/irq-apic.h"
#include "xgold/sofia_3g_soc/irq-eint.h"
#include "xgold/sofia_3g_soc/irq-hirq.h"
#include "xgold/sofia_3g_soc/irq-abb.h"
#include "xgold/agold620/irq-pmu.h"

#endif /* _SOFIA_3G_SOC_IRQ_H */
