/*
 * Copyright (c) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#ifdef CONFIG_ARM
#include <asm/sched_clock.h>
#elif defined CONFIG_X86
#include <asm/time.h>
#endif
#include "stm_regs.h"

#include "asm/xgold.h"
#include "asm/clocksource.h"

static void __iomem *stm_hw_base;
static unsigned period0;
static unsigned long clk_rate;
static struct clk *stm_clk;
#ifdef CONFIG_XGOLD_STM_TIMER_SCHED_CLOCK
static int stm_disabled __read_mostly = 1;
static unsigned long sched_clock_mult __read_mostly;
static cycle_t cycle_last, cycle_offset;
static DEFINE_SPINLOCK(stm_shed_lock);
#endif
static DEFINE_SPINLOCK(stm_hw_lock);
static int irq_nodes[2];
cpumask_t stm_cpumask = CPU_MASK_NONE;
/* Will be used by vdso code */
unsigned long stm_addr;

struct xgold_stm_clkevt {
	struct clock_event_device evt;
	unsigned id;
	bool enable;
	unsigned last_alarm;
	char name[8];
};


#define REG_STM_CMPx(cpu) \
	((cpu == 0) ? REG_STM_CMP0_OFFSET : REG_STM_CMP1_OFFSET)

#define REG_STM_CMPXIRx_IRQSM(cpu) \
	((cpu == 0) ? \
	 REG_STM_CMPXIR0_IRQSM_OFFSET : REG_STM_CMPXIR1_IRQSM_OFFSET)

#define REG_STM_CMPXIRx_IRQSC(cpu) \
	((cpu == 0) ? \
	 REG_STM_CMPXIR0_IRQSC_OFFSET : REG_STM_CMPXIR1_IRQSC_OFFSET)

static inline struct xgold_stm_clkevt *
				evt_to_stm_dev(struct clock_event_device *evt)
{
	return container_of(evt, struct xgold_stm_clkevt, evt);
}


static inline void xgold_stm_clr_irq(struct xgold_stm_clkevt *stm_clkevt)
{
	int id = stm_clkevt->id;

	iowrite32(BIT(id), stm_hw_base + REG_STM_CMPXIRx_IRQSC(id));
}

static void _xgold_stm_enable_irq(struct xgold_stm_clkevt *stm_clkevt)
{
	unsigned tmp;
	unsigned irq = stm_clkevt->id;

	tmp = ioread32(stm_hw_base + REG_STM_CMPXIRx_IRQSM(irq));
	tmp |= BIT(irq);
	stm_clkevt->enable = true;
	iowrite32(tmp, stm_hw_base + REG_STM_CMPXIRx_IRQSM(irq));
}

static void _xgold_stm_disable_irq(struct xgold_stm_clkevt *stm_clkevt)
{
	unsigned tmp;
	unsigned irq = stm_clkevt->id;

	tmp = ioread32(stm_hw_base + REG_STM_CMPXIRx_IRQSM(irq));
	tmp &= ~BIT(irq);
	stm_clkevt->enable = false;
	iowrite32(tmp, stm_hw_base + REG_STM_CMPXIRx_IRQSM(irq));
}

static inline void xgold_stm_set_event(struct xgold_stm_clkevt *stm_clkevt,
					unsigned cycles)
{
	unsigned long flags;
	unsigned alarm, initial, tmp, period, wrap, id;

	id = stm_clkevt->id;

	if (cycles)
		period = cycles;
	else
		period = period0;

	initial = ioread32(stm_hw_base + REG_STM_TIM0_OFFSET);
	alarm = initial + period;

	if (alarm < period)
		wrap = 1;
	else
		wrap = 0;

	iowrite32(alarm, stm_hw_base + REG_STM_CMPx(id));
	stm_clkevt->last_alarm = alarm;

	spin_lock_irqsave(&stm_hw_lock, flags);
	if (cycles)
		_xgold_stm_enable_irq(stm_clkevt);
	spin_unlock_irqrestore(&stm_hw_lock, flags);

	tmp = ioread32(stm_hw_base + REG_STM_TIM0_OFFSET);
	/* We might have missed the interrupt
	 * Trigger it by software if needed
	 * */
	if ((((wrap == 1) && (tmp > alarm) && (tmp < initial))
		|| ((wrap == 0) && ((tmp > alarm) || (tmp < initial))))
		&& stm_clkevt->last_alarm != 0) {
		pr_err("%s:(now %#x): Last alarm (@%#x, + %#x cycles) was missed!\n",
				__func__, tmp, stm_clkevt->last_alarm, period);
		iowrite32(BIT(id), stm_hw_base + REG_STM_ISR_OFFSET);
	}
}

static void xgold_stm_reload(struct xgold_stm_clkevt *stm_clkevt,
				unsigned period)
{
	xgold_stm_set_event(stm_clkevt, period);
}

static void xgold_stm_unmask_irq(struct xgold_stm_clkevt *stm_clkevt)
{
	unsigned long flags;

	xgold_stm_clr_irq(stm_clkevt);

	spin_lock_irqsave(&stm_hw_lock, flags);
	_xgold_stm_enable_irq(stm_clkevt);
	spin_unlock_irqrestore(&stm_hw_lock, flags);

}

static void xgold_stm_mask_irq(struct xgold_stm_clkevt *stm_clkevt)
{
	unsigned long flags;

	spin_lock_irqsave(&stm_hw_lock, flags);
	_xgold_stm_disable_irq(stm_clkevt);
	spin_unlock_irqrestore(&stm_hw_lock, flags);

	xgold_stm_clr_irq(stm_clkevt);
}

static void xgold_stm_tick_set_mode(enum clock_event_mode mode,
				    struct clock_event_device *evt)
{
	struct xgold_stm_clkevt *stm_clkevt = evt_to_stm_dev(evt);
	unsigned id = stm_clkevt->id;

	switch (mode) {
	case CLOCK_EVT_MODE_UNUSED:
		pr_debug("Clock event device %d set to UNUSED\n", id);
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		pr_debug("Clock event device %d set to PERIODIC\n", id);
		xgold_stm_unmask_irq(stm_clkevt);
		xgold_stm_reload(stm_clkevt, period0);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		pr_debug("Clock event device %d set to SHUTDOWN\n", id);
		xgold_stm_mask_irq(stm_clkevt);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		pr_debug("Clock event device %d set to ONESHOT\n", id);
		xgold_stm_unmask_irq(stm_clkevt);
		break;
	case CLOCK_EVT_MODE_RESUME:
		pr_debug("Clock event device %d set to RESUME\n", id);
		xgold_stm_unmask_irq(stm_clkevt);
		break;
	}
}

static int xgold_stm_tick_set_next_event(unsigned long cycles,
					 struct clock_event_device *evt)
{
	struct xgold_stm_clkevt *stm_clkevt = evt_to_stm_dev(evt);

	xgold_stm_set_event(stm_clkevt, cycles);
	return 0;
}

static irqreturn_t xgold_dyn_timer_isr(int irq, void *dev_id)
{
	struct xgold_stm_clkevt *stm_clkevt = dev_id;
	struct clock_event_device *evt = &stm_clkevt->evt;

	xgold_stm_clr_irq(stm_clkevt);
	stm_clkevt->last_alarm = 0;

	if (evt->event_handler == NULL)
		return IRQ_HANDLED;

	evt->event_handler(evt);

	if (evt->mode == CLOCK_EVT_MODE_PERIODIC)
		xgold_stm_reload(stm_clkevt, 0);

	return IRQ_HANDLED;
}

static void __init xgold_stm_init(void)
{
	unsigned tmp;
	iowrite32(0, stm_hw_base + REG_STM_CLC_OFFSET);
	ioread32(stm_hw_base + REG_STM_CLC_OFFSET);

	/*Assume only 32bit comparison is needed so far */
	tmp = (STM_CMCON_MSIZE0_COMP31 << STM_CMCON_MSIZE0_OFFSET);
	tmp |= (STM_CMCON_MSTART0_LB0 << STM_CMCON_MSTART0_OFFSET);
	tmp |= (STM_CMCON_MSIZE1_COMP31 << STM_CMCON_MSIZE1_OFFSET);
	tmp |= (STM_CMCON_MSTART1_LB0 << STM_CMCON_MSTART1_OFFSET);

	iowrite32(tmp, stm_hw_base + REG_STM_CMCON_OFFSET);
	iowrite32(0, stm_hw_base + REG_STM_CMPXIR0_IRQSM_OFFSET);
	iowrite32(0, stm_hw_base + REG_STM_CMPXIR1_IRQSM_OFFSET);
	iowrite32((STM_CMPXIR0_IRQSC_CMP0CL_CLRC0 <<
			STM_CMPXIR0_IRQSC_CMP0CL_OFFSET) |
		       (STM_CMPXIR0_IRQSC_CMP1CL_CLRC1 <<
			STM_CMPXIR0_IRQSC_CMP1CL_OFFSET),
		       stm_hw_base + REG_STM_CMPXIR0_IRQSC_OFFSET);
	iowrite32((STM_CMPXIR1_IRQSC_CMP0CL_CLRC0 <<
			STM_CMPXIR1_IRQSC_CMP0CL_OFFSET) |
		       (STM_CMPXIR1_IRQSC_CMP1CL_CLRC1 <<
			STM_CMPXIR1_IRQSC_CMP1CL_OFFSET),
		       stm_hw_base + REG_STM_CMPXIR1_IRQSC_OFFSET);

	iowrite32(0x3, stm_hw_base + REG_STM_IMSC_OFFSET);
}

/**************************************
  Clock source declaration
**************************************/
cycle_t notrace xgold_stm_clock_source_read(struct clocksource *cs)
{
	return (cycle_t) ioread32(stm_hw_base + REG_STM_TIM0_OFFSET);
}
EXPORT_SYMBOL(xgold_stm_clock_source_read);

static void xgold_stm_evt_suspend(struct clock_event_device *evt)
{
	struct xgold_stm_clkevt *stm_clkevt = evt_to_stm_dev(evt);
	xgold_stm_mask_irq(stm_clkevt);
}

static void xgold_stm_evt_resume(struct clock_event_device *evt)
{
	struct xgold_stm_clkevt *stm_clkevt = evt_to_stm_dev(evt);
	xgold_stm_unmask_irq(stm_clkevt);
}

static struct clock_event_device xgold_stm_clockevent = {
	.name = "xgold_stm_clockevent",
	.features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.shift = 32,		/*FIXME : Refine this choice */
	.rating = 10,
	.set_next_event = xgold_stm_tick_set_next_event,
	.set_mode = xgold_stm_tick_set_mode,
	.suspend = xgold_stm_evt_suspend,
	.resume = xgold_stm_evt_resume,

};
static DEFINE_PER_CPU(struct xgold_stm_clkevt, stm_events);

static struct clocksource xgold_stm_clocksource = {
	.name = "xgold_stm_clocksource",
	.rating = 300,		/*FIXME: Refine this choice */
	.read = xgold_stm_clock_source_read,
	.mask = CLOCKSOURCE_MASK(32),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS | CLOCK_SOURCE_SUSPEND_NONSTOP,
	.archdata       = { .vclock_mode = VCLOCK_STM },
};

#ifdef CONFIG_ARM
static notrace u32 xgold_sched_clock_read(void)
{
	return ioread32(stm_hw_base + REG_STM_TIM0_OFFSET);
}
#endif

static void xgold_stm_clkevent_setup(void)
{
	int ret;
	int cpu = smp_processor_id();
	struct xgold_stm_clkevt *stm_clkevt = &__get_cpu_var(stm_events);
	struct clock_event_device *levt = &stm_clkevt->evt;

	if (!cpu_isset(cpu, stm_cpumask)) {
		pr_debug("%s: Skip clkevent for CPU%d\n", __func__, cpu);
		return;
	} else
		pr_info("%s: Setup clkevent for CPU%d\n", __func__, cpu);

	if (levt->irq != 0)
		return;

	stm_clkevt->id = cpu;
	stm_clkevt->enable = false;
	stm_clkevt->last_alarm = 0;
	snprintf(stm_clkevt->name, 8, "STM %d", cpu);

	memcpy(levt, &xgold_stm_clockevent, sizeof(*levt));

	levt->cpumask = cpumask_of(cpu);
	levt->irq = irq_nodes[cpu];

	ret = request_irq(levt->irq, xgold_dyn_timer_isr,
		IRQF_TIMER | IRQF_DISABLED | IRQF_NOBALANCING | IRQF_PERCPU,
		stm_clkevt->name, stm_clkevt);

	BUG_ON(ret);

	disable_irq(levt->irq);
	irq_set_affinity(levt->irq, levt->cpumask);
	enable_irq(levt->irq);

	pr_err("%s: Installing vector %d done\n", __func__, irq_nodes[cpu]);

	clockevents_register_device(levt);
}

struct xgold_stm_work_struct {
	struct delayed_work work;
	struct completion complete;
};

static void xgold_stm_work(struct work_struct *w)
{
	struct xgold_stm_work_struct *xgold_stm_work;
	xgold_stm_work = container_of(w, struct xgold_stm_work_struct, work.work);

	/* Register the clock event device */
	xgold_stm_clkevent_setup();

	complete(&xgold_stm_work->complete);
}

static int xgold_stm_cpu_notify(struct notifier_block *n,
		unsigned long action, void *hcpu)
{
	unsigned long cpu = (unsigned long) hcpu;
	struct xgold_stm_work_struct work;
	struct xgold_stm_clkevt *stm_clkevt = &per_cpu(stm_events, cpu);
	struct clock_event_device *levt = &stm_clkevt->evt;

	switch (action & 0xf) {
	case CPU_ONLINE:
		INIT_DELAYED_WORK_ONSTACK(&work.work, xgold_stm_work);
		init_completion(&work.complete);
		/* FIXME: add schedule_work_on() */
		schedule_delayed_work_on(cpu, &work.work, 0);
		wait_for_completion(&work.complete);
		destroy_timer_on_stack(&work.work.timer);
		break;
	case CPU_DEAD:
		if (!cpu_isset(cpu, stm_cpumask))
			return NOTIFY_OK;

		if (levt->irq) {
			free_irq(levt->irq, stm_clkevt);
			levt->irq = 0;
		}

		xgold_stm_mask_irq(stm_clkevt);

		break;
	}

	return NOTIFY_OK;
}


static void __init xgold_of_timer_map(struct device_node *np)
{
	int ret, i, mask;
	unsigned int faf, evt_rating, src_rating, evt_min, evt_max, clk_val;
	struct resource res;

	stm_clk = of_clk_get_by_name(np, "kernel");

	if (stm_clk != ERR_PTR(-ENOENT))
		clk_rate = clk_get_rate(stm_clk);
	clk_prepare_enable(stm_clk);

	/* feng: map the STM HW base to vbase range */
	of_address_to_resource(np, 0, &res);
	stm_addr = res.start;

	stm_hw_base = of_iomap(np, 0);
	if (!stm_hw_base)
		panic("unable to map timer cpu registers\n");

	/* Get the interrupts property */
	for (i = 0; i < of_irq_count(np); i++) {
		irq_nodes[i] = irq_of_parse_and_map(np, i);
		BUG_ON(irq_nodes[i] == 0);
	}

	ret = of_property_read_u32(np, "intel,faf", &faf);
	if (!ret) {
		pr_info("%s:Initializing STM FAF to %#x\n", __func__, faf);
		iowrite32(faf, stm_hw_base + 0x70);
	}

	ret = of_property_read_bool(np, "intel,stm,is_suspended");
	if (ret) {
		pr_info("%s:Clocksource is not persistent in suspend\n",
				__func__);
		xgold_stm_clocksource.flags &= ~CLOCK_SOURCE_SUSPEND_NONSTOP;
	}

	ret = of_property_read_u32(np, "intel,stm,evt,rating", &evt_rating);
	if (!ret) {
		pr_info("%s:Clock event device rating set to %#x\n",
							__func__, evt_rating);
		xgold_stm_clockevent.rating = evt_rating;
	}

	ret = of_property_read_u32(np, "intel,stm,src,rating", &src_rating);
	if (!ret) {
		pr_info("%s:Clock source device rating set to %#x\n",
							__func__, src_rating);
		xgold_stm_clocksource.rating = src_rating;
	}

	if (stm_clk == ERR_PTR(-ENOENT)) {
		ret = of_property_read_u32(np, "intel,stm,rate", &clk_val);
		if (!ret)
			clk_rate = clk_val;
		else
			clk_rate = 26 * 1e6;

		pr_info("%s:Clock frequency %dMHz\n",
				"XGOLD STM", clk_val/1000/1000);
	}

	ret = of_property_read_u32(np, "intel,evt,min-ns", &evt_min);
	if (!ret)
		xgold_stm_clockevent.min_delta_ns = evt_min;
	else
		xgold_stm_clockevent.min_delta_ns = 38000;

	pr_info("XGOLD STM:Clock event min precision %llins\n",
			       xgold_stm_clockevent.min_delta_ns);

	ret = of_property_read_u32(np, "intel,evt,max-ns", &evt_max);
	if (!ret)
		xgold_stm_clockevent.max_delta_ns = evt_max;
	else
		xgold_stm_clockevent.max_delta_ns = 1000000000;

	pr_info("XGOLD STM:Clock event max precision %llins\n",
				xgold_stm_clockevent.max_delta_ns);

	ret = of_property_read_u32(np, "intel,stm,cpumask", &mask);
	if (ret) {
		/* property not defined, set stm_cpumask to default : CPU0/1 */
		mask = cpumask_bits(cpu_possible_mask)[0];
	}
	while (mask) {
		unsigned cpu = ffs(mask) - 1;
		pr_info("%s: Masking cpu%d for STM usage\n", __func__, cpu);
		cpu_set(cpu, stm_cpumask);
		clear_bit(cpu, (unsigned long *)&mask);
	}
}

#ifdef CONFIG_XGOLD_STM_TIMER_SCHED_CLOCK
unsigned long long notrace sched_clock(void)
{
	cycle_t cycle;
	unsigned long flags;

	if (unlikely(stm_disabled))
		return (jiffies_64 - INITIAL_JIFFIES) * (NSEC_PER_SEC / HZ);

	spin_lock_irqsave(&stm_shed_lock, flags);
	cycle = xgold_stm_clock_source_read(NULL);
	cycle &= xgold_stm_clocksource.mask;
	/* Counter wrapped */
	if (unlikely(cycle_last > cycle))
		cycle_offset += BIT_ULL(32);

	cycle_last = cycle;
	cycle += cycle_offset;
	spin_unlock_irqrestore(&stm_shed_lock, flags);

	return cycle * sched_clock_mult;
}
#endif


void __init xgold_timer_init(struct device_node *np)
{
	struct clock_event_device *ce = &xgold_stm_clockevent;
	struct clocksource *cs = &xgold_stm_clocksource;

	xgold_of_timer_map(np);

	xgold_stm_init();

	period0 = clk_rate / HZ;

	BUG_ON(clocksource_register_hz(cs, clk_rate));

#ifdef CONFIG_XGOLD_STM_TIMER_SCHED_CLOCK
	pr_info("XGOLD STM timer used to provide sched clock\n");
	sched_clock_mult = NSEC_PER_SEC / clk_rate;
	stm_disabled = 0;
#endif

#ifdef CONFIG_ARM
	setup_sched_clock(xgold_sched_clock_read, 32, clk_rate);
#endif

	clockevents_calc_mult_shift(ce, clk_rate, 180);

	pr_info("XGOLD STM timer clkevent [%llins-%llins]\n",
			ce->min_delta_ns, ce->max_delta_ns);
	/* Register immediately the clock event on BOOT cpu */
	xgold_stm_clkevent_setup();

#ifdef CONFIG_X86
	global_clock_event = &xgold_stm_clockevent;
#endif


}
static __init int xgold_stm_late_init(void)
{
	unsigned cpu;
	for_each_online_cpu(cpu) {
		xgold_stm_cpu_notify(NULL, CPU_ONLINE, (void *)(long)cpu);
	}

	/* This notifier should be called after workqueue is ready */
	hotcpu_notifier(xgold_stm_cpu_notify, CPU_PRI_WORKQUEUE_UP);
	return 0;
}

early_initcall(xgold_stm_late_init);



CLOCKSOURCE_OF_DECLARE(xgold_timer_stm, "intel,stm", xgold_timer_init);

