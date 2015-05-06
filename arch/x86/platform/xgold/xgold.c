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
#include <asm/xgold.h>
#ifdef CONFIG_X86_IO_APIC_WATCHDOG
#include <asm/io_apic_watchdog.h>
#endif
#include <asm/desc.h>
#include <asm/hw_irq.h>

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>
#include <sofia/mv_hypercalls.h>
#include <sofia/mv_gal.h>
#endif

#define XGOLD_ENTER pr_info("--> %s\n", __func__)
#define XGOLD_EXIT  pr_info("<-- %s\n", __func__)

enum intel_xgold_timer_options {
	INTEL_XGOLD_TIMER_DEFAULT,
	INTEL_XGOLD_TIMER_SOCTIMER_ONLY,
	INTEL_XGOLD_TIMER_LAPIC_SOCTIMER,
};

static enum intel_xgold_timer_options intel_xgold_timer_options;

static inline int __init setup_x86_intel_xgold_timer(char *arg)
{
	if (!arg)
		return -EINVAL;
	if (strcmp("soctimer_only", arg) == 0)
		intel_xgold_timer_options = INTEL_XGOLD_TIMER_SOCTIMER_ONLY;
	else if (strcmp("lapic_and_soctimer", arg) == 0)
		intel_xgold_timer_options = INTEL_XGOLD_TIMER_LAPIC_SOCTIMER;
	else {
		pr_warn("X86 XGOLD timer option %s is invalid\n", arg);
		return -EINVAL;
	}

	return 0;
}
__setup("x86_intel_xgold_timer=", setup_x86_intel_xgold_timer);

#ifdef NYET
static void __init xgold_soc_parse_timer(struct device_node *np)
{
	const char *timer_str =  NULL;

	if (!of_property_read_string(np, "intel,timer", &timer_str))
		setup_x86_intel_xgold_timer((char *)timer_str);
}
#endif

static void __init xgold_soc_init(void)
{
	struct device_node *np = of_find_node_by_path("/xgold");
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	struct device *parent;
	void __iomem *read_addr = NULL;
	void __iomem *hw_base = NULL;
	uint32_t scu_base = 0;
	struct resource res;
	bool scu_acccess_vmm = false;
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

	if (of_find_property(np, "intel,vmm-secured-access", NULL))
		scu_acccess_vmm = true;

	read_addr = of_parse_phandle(np, "intel,scu-phys", 0);
	hw_base = of_iomap(read_addr, 0);
	if (!hw_base) {
		ret = -ENOMEM;
		goto bug;
	}

	ret = of_address_to_resource(read_addr, 0, &res);

	scu_base = (uint32_t)res.start;

	/* Get chipid register from Device Tree */
	of_ret = of_property_read_u32_array(np, "intel,chipid", reg, 3);
	switch (of_ret) {
	case 0:		/* property gives register offset/field/mask */
#ifdef CONFIG_X86_INTEL_SOFIA
		if (scu_acccess_vmm) {
			if (mv_svc_reg_read(((uint32_t)scu_base + reg[0]),
					(uint32_t *)&tmp, -1))
				pr_err("mv_svc_reg_read fails at %#x",
					(uint32_t)(scu_base + reg[0]));
		} else
#else
			tmp = readl_relaxed(hw_base + reg[0]);
#endif
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
	case 0:		/* property gives register reg[0]/field/mask */
#ifdef CONFIG_X86_INTEL_SOFIA
		if (mv_svc_reg_read((uint32_t)(scu_base + reg[0]),
				(uint32_t *)&tmp, -1))
			pr_err("mv_svc_reg_read fails at %#x\n",
				(uint32_t)(scu_base + reg[0]));
#else
		tmp = readl_relaxed(hw_base + reg[0]);
#endif
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

	pr_info("register chip revision 0x%s\n", soc_dev_attr->revision);
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
		/*
		 * Allocate nr + 1 to ensure match_id table is NULL terminated
		 */
		match_id = kzalloc((nr + 1) * sizeof(struct of_device_id),
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
static bool x86_intel_xgold_needs_broadcast;

bool xgold_platform_needs_broadcast_timer(void)
{
	return x86_intel_xgold_needs_broadcast;
}

static void __init x86_intel_xgold_time_init(void)
{
	x86_intel_xgold_needs_broadcast = false;
	switch (intel_xgold_timer_options) {
	case INTEL_XGOLD_TIMER_SOCTIMER_ONLY:
		pr_info("Intel x86 xgold using soctimer only\n");
		x86_init.timers.setup_percpu_clockev = x86_init_noop;
		x86_cpuinit.setup_percpu_clockev = x86_init_noop;
		break;
	case INTEL_XGOLD_TIMER_LAPIC_SOCTIMER:
		pr_info("Intel x86 xgold using soctimer and lapic\n");
		x86_init.timers.setup_percpu_clockev = setup_boot_APIC_clock;
		x86_cpuinit.setup_percpu_clockev = setup_secondary_APIC_clock;
		x86_intel_xgold_needs_broadcast = true;
		break;
	default:
		pr_info("Intel x86 xgold using lapic only\n");
		x86_init.timers.setup_percpu_clockev = setup_boot_APIC_clock;
		x86_cpuinit.setup_percpu_clockev = setup_secondary_APIC_clock;
		if (!boot_cpu_has(X86_FEATURE_ARAT))
			x86_intel_xgold_needs_broadcast = true;
	}
	return;
}


static void __init x86_xgold_time_init(void)
{
	XGOLD_ENTER;
	x86_intel_xgold_time_init();
	xgold_init_clocks();
/*
 * IO-APIC/LAPIC need to be initialized in native case
 * */

#ifdef CONFIG_X86_INTEL_SOFIA
	sofia_vmm_init();
#else
#ifndef CONFIG_SMP
	physid_set_mask_of_physid(boot_cpu_physical_apicid,
					 &phys_cpu_present_map);
#endif
	if (intel_xgold_timer_options
			!= INTEL_XGOLD_TIMER_SOCTIMER_ONLY)
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

#define XGOLD_CALIB_DURATION 10 /* ms */
#define XGOLD_STM_1MS 26000 /* Assuming STM clock is 26MHz */
#define APIC_DIVISOR 16
extern cycle_t xgold_stm_clock_source_read(struct clocksource *);
static unsigned long xgold_calibrate_tsc(void)
{
	long lapic_cal_t1, lapic_cal_t2;
	unsigned long long lapic_cal_tsc1, lapic_cal_tsc2;
	cycle_t xgold_stm_cal_t1, xgold_stm_cal_t2, delta_stm;
	long delta, deltatsc;
	uint32_t reg;
	pr_info("%s: Run it for %d ms\n", __func__, XGOLD_CALIB_DURATION);
	/*
	 * Setup the APIC counter to maximum. There is no way the lapic
	 * can underflow in the 100ms detection time frame
	 * Divide PICLK by 16
	 */
	reg = LOCAL_TIMER_VECTOR;
	reg |= APIC_LVT_TIMER_PERIODIC;
	reg |= APIC_LVT_MASKED;
	apic_write(APIC_LVTT, reg);
	reg = APIC_TDR_DIV_16;
	apic_write(APIC_TDCR, reg);
	reg = (0xffffffff / APIC_DIVISOR);
	apic_write(APIC_TMICT, reg);

	lapic_cal_t1 = apic_read(APIC_TMCCT);
	lapic_cal_tsc1 = get_cycles();
	xgold_stm_cal_t1 = xgold_stm_clock_source_read(NULL);
	/* Calibrate for 10ms */
	while (1) {
		xgold_stm_cal_t2 = xgold_stm_clock_source_read(NULL);
		delta_stm = xgold_stm_cal_t2 - xgold_stm_cal_t1;
		if (delta_stm >= (XGOLD_CALIB_DURATION * XGOLD_STM_1MS))
			break;
	}
	lapic_cal_t2 = apic_read(APIC_TMCCT);
	lapic_cal_tsc2 = get_cycles();
	pr_debug("%s: ... xgold stm t1 = %lld\n", __func__, xgold_stm_cal_t1);
	pr_debug("%s: ... xgold stm t2 = %lld\n", __func__, xgold_stm_cal_t2);
	pr_debug("%s: ... xgold stm delta = %lld\n", __func__, delta_stm);

	/* Build delta t1-t2 as apic timer counts down */
	delta = lapic_cal_t1 - lapic_cal_t2;
	pr_debug("%s: ... lapic t1 = %ld\n", __func__, lapic_cal_t1);
	pr_debug("%s: ... lapic t2 = %ld\n", __func__, lapic_cal_t2);
	pr_debug("%s: ... lapic delta = %ld\n", __func__, delta);

	deltatsc = (long)(lapic_cal_tsc2 - lapic_cal_tsc1);
	pr_debug("%s: ... tsc t1 = %lld\n", __func__, lapic_cal_tsc1);
	pr_debug("%s: ... tsc t2 = %lld\n", __func__, lapic_cal_tsc2);
	pr_debug("%s: ... tsc delta = %ld\n", __func__, deltatsc);

	lapic_timer_frequency = (delta * APIC_DIVISOR);
	lapic_timer_frequency /= XGOLD_CALIB_DURATION; /* in kHZ */
	pr_info("%s: LAPIC Timer frequency is %u.%04u MHz.\n", __func__,
		    lapic_timer_frequency / 1000, lapic_timer_frequency % 1000);
	lapic_timer_frequency *= 1000; /* in HZ */
	lapic_timer_frequency /= HZ;

	return deltatsc/XGOLD_CALIB_DURATION;
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

static void xgold_rtc_get_time(struct timespec *ts)
{
	unsigned long long time_us = 0;

	mv_svc_rtc_get_time_us(&time_us);
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

		mv_svc_rtc_set_datetime(&rtc_data);
	} else {
		pr_err("%s: Invalid RTC value !\n", __func__);
	}
	return 0;
}

static void xgold_rtc_get_time_stub(struct timespec *ts)
{
}

static int xgold_rtc_set_time_stub(const struct timespec *ts)
{
	return 0;
}

static void __init xgold_rtc_init(void)
{
	struct device_node *np = of_find_node_by_path("/xgold");
	struct device_node *np_rtc = of_find_node_by_name(np, "rtc");
	if (np_rtc) {
		x86_platform.get_wallclock = xgold_rtc_get_time;
		x86_platform.set_wallclock = xgold_rtc_set_time;
		of_node_put(np_rtc);
	} else {
		x86_platform.get_wallclock = xgold_rtc_get_time_stub;
		x86_platform.set_wallclock = xgold_rtc_set_time_stub;
	}
}

#ifdef CONFIG_X86_INTEL_SOFIA
#ifdef CONFIG_X86_LOCAL_APIC
void sofia_init_irq(void)
{
	native_init_IRQ();
	/*
	 * the cpu bitmap does not matter here as it's local interrupt
	 */
	mv_virq_request(LOCAL_TIMER_VECTOR, 1);
	mv_virq_unmask(LOCAL_TIMER_VECTOR);
#ifdef CONFIG_SMP
	mv_virq_request(RESCHEDULE_VECTOR, 1);
	mv_virq_unmask(RESCHEDULE_VECTOR);
	mv_virq_request(CALL_FUNCTION_VECTOR, 1);
	mv_virq_unmask(CALL_FUNCTION_VECTOR);
	mv_virq_request(CALL_FUNCTION_SINGLE_VECTOR, 1);
	mv_virq_unmask(CALL_FUNCTION_SINGLE_VECTOR);
	mv_virq_request(REBOOT_VECTOR, 1);
	mv_virq_unmask(REBOOT_VECTOR);
#endif
}
#endif
#endif

static void __init xgold_reserve_resources(void)
{
	pstore_ram_reserve_memory();
}

/*
 * XGOLD specific x86_init function overrides and early setup
 * calls.
 */
void __init x86_xgold_early_setup(void)
{
	x86_init.resources.probe_roms = x86_init_noop,
	x86_init.resources.reserve_resources = xgold_reserve_resources,
	x86_init.irqs.pre_vector_init = x86_init_noop,
	x86_init.oem.banner = x86_xgold_default_banner,
	x86_init.pci.init = xgold_pci_init,
	x86_init.pci.init_irq = x86_init_noop,
	x86_init.pci.fixup_irqs = x86_init_noop,
	x86_init.pci.arch_init = xgold_pci_arch_init,
	x86_init.timers.wallclock_init = xgold_rtc_init;

	/* Not needed for PC */
	x86_init.timers.timer_init = x86_xgold_time_init,
	x86_init.timers.setup_percpu_clockev = x86_init_noop;
	x86_cpuinit.setup_percpu_clockev = x86_init_noop;
	x86_platform.calibrate_tsc = xgold_calibrate_tsc;
	x86_platform.save_sched_clock_state = xgold_save_clock_state;
	x86_platform.restore_sched_clock_state = xgold_restore_clock_state;
	x86_platform.i8042_detect = xgold_i8042_init;

#ifdef CONFIG_X86_INTEL_SOFIA
#ifdef CONFIG_X86_LOCAL_APIC
	x86_init.irqs.intr_init = sofia_init_irq;
#endif
#ifdef CONFIG_SMP
	x86_cpuinit.early_percpu_clock_init = (void *)sofia_vmm_init_secondary;
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
