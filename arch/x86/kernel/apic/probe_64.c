/*
 * Copyright 2004 James Cleverdon, IBM.
 * Copyright (C) 2014-2015 Intel Mobile Communications GmbH
 *
 * Subject to the GNU Public License, v.2
 *
 * Generic APIC sub-arch probe layer.
 *
 * Hacked for x86-64 by James Cleverdon from i386 architecture code by
 * Martin Bligh, Andi Kleen, James Bottomley, John Stultz, and
 * James Cleverdon.
 */
#include <linux/threads.h>
#include <linux/cpumask.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/hardirq.h>
#include <linux/dmar.h>

#include <asm/smp.h>
#include <asm/apic.h>
#include <asm/ipi.h>
#include <asm/setup.h>

static int cmdline_apic __initdata;
static int __init parse_apic(char *arg)
{

	struct apic **drv;
	pr_info("%s\n", __func__);

	if (!arg)
		return -EINVAL;

	for (drv = __apicdrivers; drv < __apicdrivers_end; drv++) {
		if (!strcmp((*drv)->name, arg)) {
			apic = *drv;
			pr_info("Switched APIC routing to %s.\n",
					apic->name);
			cmdline_apic = 1;
			return 0;
		}
	}

	/* Parsed again by __setup for debug/verbose */
	return 0;
}
early_param("apic", parse_apic);

/*
 * Check the APIC IDs in bios_cpu_apicid and choose the APIC mode.
 */
void __init default_setup_apic_routing(void)
{
	if (!cmdline_apic) {
		struct apic **drv;

		enable_IR_x2apic();

		for (drv = __apicdrivers; drv < __apicdrivers_end; drv++) {
			if ((*drv)->probe && (*drv)->probe()) {
				if (apic != *drv) {
					apic = *drv;
					pr_info("Switched APIC routing to %s.\n",
							apic->name);
				}
				break;
			}
		}

		if (x86_platform.apic_post_init)
			x86_platform.apic_post_init();
	}
}

/* Same for both flat and physical. */

void apic_send_IPI_self(int vector)
{
	__default_send_IPI_shortcut(APIC_DEST_SELF, vector, APIC_DEST_PHYSICAL);
}

int __init default_acpi_madt_oem_check(char *oem_id, char *oem_table_id)
{
	struct apic **drv;

	for (drv = __apicdrivers; drv < __apicdrivers_end; drv++) {
		if ((*drv)->acpi_madt_oem_check(oem_id, oem_table_id)) {
			if (apic != *drv) {
				apic = *drv;
				pr_info("Setting APIC routing to %s.\n",
					apic->name);
			}
			return 1;
		}
	}
	return 0;
}
