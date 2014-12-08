/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _SOFIA_LTE_SOC_IRQ_H
#define _SOFIA_LTE_SOC_IRQ_H

#ifndef CONFIG_X86_INTEL_SOFIA
/* HW nodes definitions based on HW attributions */
#include "xgold/sofia_lte_soc/irq-hw.h"
#else
/* Vectors nodes definitions based on VMM attributions */
#include "xgold/sofia_lte_soc/irq-vectors.h"
#endif

/* IRQ DOMAINS HEADERS */
#include "xgold/sofia_lte_soc/irq-apic.h"
#include "xgold/sofia_lte_soc/irq-eint.h"
#include "xgold/sofia_lte_soc/irq-hirq.h"

#endif /* _SOFIA_LTE_SOC_IRQ_H */
