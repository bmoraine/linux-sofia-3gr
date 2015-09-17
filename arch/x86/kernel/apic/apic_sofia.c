/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/smp.h>
#include <linux/percpu.h>
#include <linux/cpumask.h>
#include <linux/syscore_ops.h>
#include <asm/apic.h>
#include <asm/irq_vectors.h>

#include <sofia/mv_hypercalls.h>
#include <sofia/mv_svc_hypercalls.h>

#define LAPIC_PADDR_BASE   0xFEE00000

static u32 sofia_apic_mem_read(u32 reg);
static void sofia_apic_mem_write(u32 reg, u32 v);

static void sofia_apic_send_ipi_mask(const struct cpumask *mask, int vector)
{
	if ((vector != RESCHEDULE_VECTOR) &&
		(vector != CALL_FUNCTION_SINGLE_VECTOR) &&
		(vector != CALL_FUNCTION_VECTOR) &&
		(vector != LOCAL_TIMER_VECTOR) &&
		(vector != IRQ_MOVE_CLEANUP_VECTOR)) {
		pr_err("%s: Unknown vector %x\n", __func__, vector);
		BUG();
	}

	mv_ipi_post(vector, cpumask_bits(mask)[0]);
}


static void sofia_send_ipi_allbutself(int vector)
{
	unsigned int vcpus;

	if (!(num_online_cpus() > 1))
		return;

	vcpus = (1 << num_possible_cpus()) - 1;
	vcpus &= ~(1 << smp_processor_id());
	mv_ipi_post(vector, vcpus);
}

static void sofia_send_IPI_all(int vector)
{
	unsigned int vcpus;

	vcpus = (1 << num_possible_cpus()) - 1;
	mv_ipi_post(vector, vcpus);

}

static int sofia_wakeup_secondary_cpu(int apicid, unsigned long start_ip)
{
	/* FIXME: Not so good to highjack the parameter.. */
#ifdef CONFIG_X86_32
	unsigned long hack_start_ip = (unsigned long)__pa(startup_32_smp);
#else
	unsigned long hack_start_ip = (unsigned long)__pa(secondary_startup_64);
#endif
	unsigned cpu;

	for_each_possible_cpu(cpu) {
		if (per_cpu(x86_cpu_to_apicid, cpu) == apicid)
			mv_vcpu_start(cpu, hack_start_ip);
	}

	return 0;
}

static int sofia_apic_suspend(void)
{
	unsigned v;

	v = sofia_apic_mem_read(APIC_LVTT);
	v |= (APIC_LVT_MASKED | LOCAL_TIMER_VECTOR);
	sofia_apic_mem_write(APIC_LVTT, v);
	sofia_apic_mem_write(APIC_TMICT, 0);

	return 0;
}

static void sofia_apic_resume(void)
{
	unsigned v;

	v = sofia_apic_mem_read(APIC_LVTT);
	v &= ~(APIC_LVT_MASKED);
	v |= LOCAL_TIMER_VECTOR;
	sofia_apic_mem_write(APIC_LVTT, v);

}

static struct syscore_ops sofia_apic_syscore_ops = {
	.resume		= sofia_apic_resume,
	.suspend	= sofia_apic_suspend,
};

static int sofia_apic_probe(void)
{
	register_syscore_ops(&sofia_apic_syscore_ops);

	return 1;
}

static u32 sofia_apic_mem_read(u32 reg)
{
	unsigned addr = LAPIC_PADDR_BASE + reg;
	unsigned mask = 0xFFFFFFFF;
	unsigned val;
	unsigned ret;

	ret = mv_svc_reg_read(addr, &val, mask);
	if (ret) {
		pr_err("%s: Read %x APIC register failed\n",
				__func__, addr);
		pr_err("%s: Continue with native read...\n", __func__);

		return native_apic_mem_read(reg);

	}

	return val;
}


static void sofia_apic_mem_write(u32 reg, u32 v)
{
	unsigned addr = LAPIC_PADDR_BASE + reg;
	unsigned mask = 0xFFFFFFFF;
	int ret;

	switch (reg) {
	case APIC_LVTT:
	case APIC_TMICT:
	case APIC_TDCR:
	case APIC_TMCCT:
		ret = mv_svc_reg_write(addr, v, mask);
		if (ret) {
			pr_err("%s: Write %x APIC register failed\n",
					__func__, addr);
			pr_err("%s: Continue with native write...\n", __func__);
			native_apic_mem_write(reg, v);
		}
		break;
	default:
		return;
	}
}

#ifdef CONFIG_X86_32
static int sofia_early_logical_apicid(int cpu)
{
	return BIT(cpu);
}
#endif

static void sofia_apic_eoi_write(u32 reg, u32 value)
{
	mv_virq_eoi(value);
}

static int sofia_apic_id_registered(void)
{
	return physid_isset(read_apic_id(), phys_cpu_present_map);
}


static void sofia_init_lapic_ldr(void) { }
static void sofia_apic_wait_icr_idle(void) { }

static void sofia_apic_ipi_self(int vector)
{
	unsigned int vcpus;
	vcpus = (1 << smp_processor_id());

	mv_ipi_post(vector, vcpus);
}

static struct apic apic_sofia = {

	.name				= "sofia",
	.probe				= sofia_apic_probe,
	.acpi_madt_oem_check		= NULL,
	.apic_id_valid			= default_apic_id_valid,
	.apic_id_registered		= sofia_apic_id_registered,

	.irq_delivery_mode		= 0,
	.irq_dest_mode			= 1,

	.target_cpus			= default_target_cpus,
	.disable_esr			= 0,
	.dest_logical			= 0,
	.check_apicid_used		= NULL,
	.check_apicid_present		= NULL,

	.vector_allocation_domain	= flat_vector_allocation_domain,
	.init_apic_ldr			= sofia_init_lapic_ldr,

	.ioapic_phys_id_map		= NULL,
	.setup_apic_routing		= NULL,
	.multi_timer_check		= NULL,
	.cpu_present_to_apicid		= default_cpu_present_to_apicid,
	.apicid_to_cpu_present		= NULL,
	.setup_portio_remap		= NULL,
	.check_phys_apicid_present	= default_check_phys_apicid_present,
	.enable_apic_mode		= NULL,
	.phys_pkg_id			= default_phys_pkg_id,
	.mps_oem_check			= NULL,

	.get_apic_id			= default_get_apic_id,
	.set_apic_id			= NULL,
	.apic_id_mask			= 0x0F << 24,

	.cpu_mask_to_apicid_and		= flat_cpu_mask_to_apicid_and,

	.send_IPI_mask			= sofia_apic_send_ipi_mask,
	.send_IPI_mask_allbutself	= NULL,
	.send_IPI_allbutself		= sofia_send_ipi_allbutself,
	.send_IPI_all			= sofia_send_IPI_all,
	.send_IPI_self			= sofia_apic_ipi_self,

	.wakeup_secondary_cpu		= sofia_wakeup_secondary_cpu,

	.trampoline_phys_low		= DEFAULT_TRAMPOLINE_PHYS_LOW,
	.trampoline_phys_high		= DEFAULT_TRAMPOLINE_PHYS_HIGH,

	.wait_for_init_deassert		= NULL,

	.smp_callin_clear_local_apic	= NULL,
	.inquire_remote_apic		= NULL,

	.read				= sofia_apic_mem_read,
	.write				= sofia_apic_mem_write,
	.eoi_write			= sofia_apic_eoi_write,
	.icr_read			= NULL,
	.icr_write			= NULL,
	.wait_icr_idle			= sofia_apic_wait_icr_idle,
	.safe_wait_icr_idle		= NULL,

#ifdef CONFIG_X86_32
	.x86_32_early_logical_apicid	= sofia_early_logical_apicid,
#endif
};

apic_driver(apic_sofia);

