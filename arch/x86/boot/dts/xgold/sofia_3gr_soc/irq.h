#ifndef _SOFIA_3G_SOC_IRQ_H
#define _SOFIA_3G_SOC_IRQ_H

#ifndef CONFIG_X86_INTEL_SOFIA
/* HW nodes definitions based on HW attributions */
#include "xgold/sofia_3g_soc/irq-hw.h"
#else
/* Vectors nodes definitions based on VMM attributions */
#include "xgold/sofia_3g_soc/irq-vectors.h"
#endif

/* IRQ DOMAINS HEADERS */
#include "xgold/sofia_3g_soc/irq-apic.h"
#include "xgold/sofia_3g_soc/irq-eint.h"
#include "xgold/sofia_3g_soc/irq-hirq.h"
#include "xgold/sofia_3g_soc/irq-abb.h"
#include "xgold/agold620/irq-pmu.h"

#endif /* _SOFIA_3G_SOC_IRQ_H */
