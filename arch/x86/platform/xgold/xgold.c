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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/bootmem.h>
#include <linux/slab.h>
#include <linux/sched.h>

#include <linux/clk-provider.h>
#include <linux/clk/xgold.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/sys_soc.h>
#include <linux/rtc.h>
#include <asm/x86_init.h>
#include <asm/irq.h>
#include <asm/apic.h>
#include <asm/i8259.h>
#ifdef CONFIG_X86_IO_APIC_WATCHDOG
#include <asm/io_apic_watchdog.h>
#endif
#include <asm/desc.h>
#include <asm/hw_irq.h>

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/nk_sofia_bridge.h>
#include <sofia/pal_shared_data.h>
#include <sofia/vmm_platform_service.h>
#endif

#define XGOLD_ENTER pr_info("--> %s\n", __func__)
#define XGOLD_EXIT  pr_info("<-- %s\n", __func__)

extern void setup_apic_timer(void);
extern void xgold_setup_secondary_APIC_clock(void);
extern void sofia_vmm_init_secondary(void);
extern void sofia_vmm_init(void);

static void __init xgold_soc_init(void)
{
	struct device_node *np = of_find_node_by_path("/xgold");
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	struct device *parent;
	void __iomem *hw_base = NULL;
	unsigned id = 0, rev = 0, id_mask = 0, rev_mask = 0;
	unsigned ret = 0, tmp = 0, i = 0;
	int of_ret;
	u32 reg[3];

	if (!np) {
		ret = -ENODEV;
		goto bug;
	}

	/* Register soc attributes */
	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr) {
		ret = -ENOMEM;
		goto bug;
	}

	hw_base = of_iomap(of_parse_phandle(np, "intel,scu-phys", 0), 0);
	if (!hw_base) {
		ret = -ENOMEM;
		goto bug;
	}

	/* Get chipid register from Device Tree */
	of_ret = of_property_read_u32_array(np, "intel,chipid", reg, 3);
	switch (of_ret) {
	case 0:		/* property gives register offset/field/mask */
		tmp = readl_relaxed(hw_base + reg[0]);
		for (i = 0; i < reg[2]; i++)
			id_mask |= 1 << i;

		id = ((tmp >> reg[1]) & id_mask);
		break;

	case -EOVERFLOW:	/* property gives directly the value */
		of_property_read_u32(np, "intel,chipid", &id);
		break;

	default:
		ret = -EINVAL;
		goto unmap;
	}

	/* Get revision register from Device Tree */
	of_ret = of_property_read_u32_array(np, "intel,rev", reg, 3);
	switch (of_ret) {
	case 0:		/* property gives register offset/field/mask */
		tmp = readl_relaxed(hw_base + reg[0]);
		for (i = 0; i < reg[2]; i++)
			rev_mask |= 1 << i;

		rev = ((tmp >> reg[1]) & rev_mask);
		break;

	case -EOVERFLOW:	/* property gives directly the value */
		of_property_read_u32(np, "intel,rev", &rev);
		break;

	default:
		ret = -EINVAL;
		goto unmap;
	}

	soc_dev_attr->soc_id = kasprintf(GFP_KERNEL, "%x", id);
	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "%x", rev);
	soc_dev_attr->machine = of_get_property(np, "intel,machine", NULL);
	if (soc_dev_attr->machine == NULL) {
		ret = -EINVAL;
		goto unmap;
	}
	soc_dev_attr->family = "xgold";

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR_OR_NULL(soc_dev)) {
		ret = PTR_ERR(soc_dev);
		goto soc_attr_free;
	}
	parent = soc_device_to_device(soc_dev);

	/*
	 * Populate all intel,soc compatibles and childs as platform
	 * devices, according to platform-dev compatible list provided in the
	 * soc device node itself.
	 */
	for_each_compatible_node(np, NULL, "intel,soc") {
		int nr;
		struct of_device_id *match_id;
		const char *name;

		nr = of_property_count_strings(np, "intel,platform-dev");
		match_id = kzalloc(nr * sizeof(struct of_device_id),
				GFP_KERNEL);
		if (!match_id) {
			ret = -ENOMEM;
			goto soc_attr_free;
		}
		for (i = 0; i < nr; i++) {
			of_property_read_string_index(np, "intel,platform-dev",
				i, &name);
			sprintf((char *)&match_id[i].compatible,
					"intel,%s", name);
		}

		/*
		 * Use Soc dev as parent for every platform devices
		 * remaining on the main bus
		 * FIXME: what about other ICs such as pmic ? can we use
		 * same parent ?
		 */
		ret = of_platform_populate(np, match_id, NULL, parent);
		kfree(match_id);
		if (ret)
			goto soc_attr_free;
	}

	/* Add every platform devices not in soc node */
	ret = of_platform_populate(NULL, NULL, NULL, NULL);
	if (ret)
		goto soc_attr_free;

	return;

soc_attr_free:
	kfree(soc_dev_attr);
unmap:
	iounmap(hw_base);
bug:
	BUG_ON(ret);
}

static int __init xgold_init_machine(void)
{
	int ret = 0;
	XGOLD_ENTER;
	xgold_soc_init();
	set_sched_clock_stable();
	XGOLD_EXIT;
	return ret;
}

arch_initcall(xgold_init_machine);

static void __init x86_xgold_time_init(void)
{
	XGOLD_ENTER;
	xgold_init_clocks();
/*
 * IO-APIC/LAPIC need to be initialized in native case
 * */
#ifndef CONFIG_X86_INTEL_SOFIA
#ifndef CONFIG_SMP
	physid_set_mask_of_physid(boot_cpu_physical_apicid,
					 &phys_cpu_present_map);
#endif
	setup_local_APIC();
#endif

	clocksource_of_init();

#ifdef CONFIG_X86_IO_APIC_WATCHDOG
	io_apic_watchdog_init();
#endif
	XGOLD_EXIT;
}

static __init void x86_xgold_default_banner(void)
{

	pr_info("Booting XGOLD kernel\n");
}

static int xgold_pci_init(void)
{
	return 0;
}

static int xgold_pci_arch_init(void)
{
	return 0;
}

/*
 * FIXME: Shall I use default_clock_source() to retrieve stm clocksource,
 *	and not using this ugly exported xgold_stm_clock_source_read()?
 * What is the impact of overriding default jiffies clocksource ?
 * */
extern cycle_t xgold_stm_clock_source_read(struct clocksource *);
static unsigned long __init xgold_calibrate_tsc(void)
{
#define STM_1MS 26000 /* Assuming STM clock is 26MHz*/
	struct clocksource *cs = NULL;
	cycle_t t1, t2, delta;
	u64 prev_tsc = 0, tsc, delta_tsc;

	t1 = xgold_stm_clock_source_read(cs);
	prev_tsc = get_cycles();
	while (1) {
		t2 = xgold_stm_clock_source_read(cs);
		delta = t2 - t1;
		if (delta >= (10 * STM_1MS))
			break;
	}
	tsc = get_cycles();
	delta_tsc = tsc - prev_tsc;

	/* Because of wrong CPU fuses values, we have to divide by 9 */
	do_div(delta_tsc, 90000);
	pr_info("%s: Tsc rate estimated to %llu MHz\n", __func__, delta_tsc);
	/* FIXME:
	 * tsc rate is inconsistent on FPGA/VP, return tsc as unstable for now
	 *	The lapic timer frequency needs to come from hw...
	 * */
#ifdef CONFIG_X86_LOCAL_APIC
	/* CPU clock is 4x timer clock  (1 000 000 / 4)*/
	lapic_timer_frequency = ((unsigned int)delta_tsc * 250000);
	pr_info("lapic frequency is %d Hz\n", lapic_timer_frequency);
	lapic_timer_frequency /= HZ;
#endif
	return 0;
}

void xgold_save_clock_state(void)
{
	tsc_save_sched_clock_state();
}

void xgold_restore_clock_state(void)
{
	tsc_restore_sched_clock_state();
}

static int xgold_i8042_init(void)
{
	return 0;
}
#ifdef CONFIG_X86_INTEL_XGOLD_EXPERIMENTAL
static void xgold_rtc_get_time(struct timespec *ts)
{
	unsigned long long time_us = 0;

	vmm_rtc_get_time_us(&time_us);
	do_div(time_us, 1000000);

	ts->tv_sec = time_us;
	ts->tv_nsec = 0;
}

static int xgold_rtc_set_time(const struct timespec *ts)
{
	struct rtc_time tm;

	rtc_time_to_tm(ts->tv_sec, &tm);
	if (!rtc_valid_tm(&tm)) {
		struct rtc_datetime_shared_data rtc_data;

		rtc_data.m_year = tm.tm_year;
		rtc_data.m_month = tm.tm_mon;
		rtc_data.m_day = tm.tm_mday;
		rtc_data.m_hour = tm.tm_hour;
		rtc_data.m_minute = tm.tm_min;
		rtc_data.m_second = tm.tm_sec;
		rtc_data.m_msecond = 0;

		vmm_rtc_set_datetime(&rtc_data);
	} else {
		pr_err("%s: Invalid RTC value !\n", __func__);
	}
	return 0;
}
#else

static void xgold_rtc_get_time(struct timespec *ts)
{
}

static int xgold_rtc_set_time(const struct timespec *ts)
{
	return 0;
}
#endif

static void __init xgold_rtc_init(void)
{
	x86_platform.get_wallclock = xgold_rtc_get_time;
	x86_platform.set_wallclock = xgold_rtc_set_time;
}

#ifdef CONFIG_X86_INTEL_SOFIA
#ifdef CONFIG_X86_LOCAL_APIC
void sofia_init_irq(void)
{
	native_init_IRQ();
/*
 * the cpu bitmap does not matter here as it's local interrupt
 */
	vmm_guest_request_virq(LOCAL_TIMER_VECTOR, 1);
	vmm_virq_unmask(LOCAL_TIMER_VECTOR);
#ifdef CONFIG_SMP
	vmm_guest_request_virq(RESCHEDULE_VECTOR, 1);
	vmm_virq_unmask(RESCHEDULE_VECTOR);
	vmm_guest_request_virq(CALL_FUNCTION_VECTOR, 1);
	vmm_virq_unmask(CALL_FUNCTION_VECTOR);
	vmm_guest_request_virq(CALL_FUNCTION_SINGLE_VECTOR, 1);
	vmm_virq_unmask(CALL_FUNCTION_SINGLE_VECTOR);
#endif
}
#endif
#endif

/*
* XGOLD specific x86_init function overrides and early setup
* calls.
*/
void __init x86_xgold_early_setup(void)
{
	x86_init.resources.probe_roms = x86_init_noop,
	x86_init.resources.reserve_resources = x86_init_noop,
	x86_init.irqs.pre_vector_init = x86_init_noop,
	x86_init.oem.banner = x86_xgold_default_banner,
	x86_init.pci.init = xgold_pci_init,
	x86_init.pci.init_irq = x86_init_noop,
	x86_init.pci.fixup_irqs = x86_init_noop,
	x86_init.pci.arch_init = xgold_pci_arch_init,
	x86_init.timers.wallclock_init = xgold_rtc_init;

	/* Not needed for PC */
	x86_init.timers.timer_init = x86_xgold_time_init,
	x86_platform.calibrate_tsc = xgold_calibrate_tsc;
	x86_platform.save_sched_clock_state = xgold_save_clock_state;
	x86_platform.restore_sched_clock_state = xgold_restore_clock_state;
	x86_platform.i8042_detect = xgold_i8042_init;

#ifdef CONFIG_X86_INTEL_SOFIA
#ifdef CONFIG_X86_LOCAL_APIC
	x86_init.irqs.intr_init = sofia_init_irq;
#endif
#ifdef CONFIG_SMP
	x86_cpuinit.early_percpu_clock_init = sofia_vmm_init_secondary;
#endif
#endif
	legacy_pic = &null_legacy_pic;

#ifdef CONFIG_X86_IO_APIC
	x86_io_apic_ops.read = direct_io_apic_read;
	x86_io_apic_ops.write = direct_io_apic_write;
	x86_io_apic_ops.modify	= direct_io_apic_write;
#endif
};

#ifndef CONFIG_X86_IO_APIC
int __init arch_probe_nr_irqs(void)
{
	return 0;
}
#endif

int __init __weak arch_early_irq_init(void)
{
	return 0;
}

