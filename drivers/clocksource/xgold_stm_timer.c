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
#ifdef CONFIG_ARM
#include <asm/sched_clock.h>
#elif defined CONFIG_X86
#include <asm/time.h>
#endif
#include "stm_regs.h"

static void __iomem *stm_hw_base;
static unsigned period0;
static unsigned long clk_rate = 26 * 1e6;
static struct clk *stm_clk;
static unsigned last_alarm;
static bool irq_enabled;
#ifdef CONFIG_XGOLD_STM_TIMER_SCHED_CLOCK
static int stm_disabled __read_mostly = 1;
static unsigned long sched_clock_mult __read_mostly;
#endif


static inline void xgold_stm_set_event(unsigned cycles)
{
	unsigned alarm, initial, tmp, period, wrap;

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

	iowrite32(alarm, stm_hw_base + REG_STM_CMP0_OFFSET);
	last_alarm = alarm;

	if (cycles) {
		tmp = readl_relaxed(stm_hw_base + REG_STM_CMPXIR0_IRQSM_OFFSET);
		tmp &= (STM_CMPXIR0_IRQSM_CMP0EN_MASK
			<< STM_CMPXIR0_IRQSM_CMP0EN_OFFSET);
		tmp |= (STM_CMPXIR0_IRQSM_CMP0EN_C0IR0
			<< STM_CMPXIR0_IRQSM_CMP0EN_OFFSET);

		iowrite32(tmp, stm_hw_base + REG_STM_CMPXIR0_IRQSM_OFFSET);
	}

	tmp = ioread32(stm_hw_base + REG_STM_TIM0_OFFSET);
	/* We might have missed the interrupt
	 * Trigger it by software if needed
	 * */
	if ((((wrap == 1) && (tmp > alarm) && (tmp < initial))
		|| ((wrap == 0) && ((tmp > alarm) || (tmp < initial))))
		&& last_alarm != 0) {
		pr_err("%s:(now %#x): Last alarm (@%#x, + %#x cycles) was missed!\n",
				__func__, tmp, last_alarm, period);
		iowrite32(STM_ISR_IR0_MASK << STM_ISR_IR0_OFFSET,
				stm_hw_base + REG_STM_ISR_OFFSET);
	}

}

void xgold_stm_reload(unsigned period)
{
	xgold_stm_set_event(period);
}

static void xgold_stm_unmask_irq(void)
{

	iowrite32((STM_CMPXIR0_IRQSC_CMP0CL_CLRC0 <<
			STM_CMPXIR0_IRQSC_CMP0CL_OFFSET),
		       stm_hw_base + REG_STM_CMPXIR0_IRQSC_OFFSET);

	iowrite32((STM_IMSC_IR0_INT_EN	<< STM_IMSC_IR0_OFFSET),
		       stm_hw_base + REG_STM_IMSC_OFFSET);

	irq_enabled = true;
}

static void xgold_stm_mask_irq(void)
{
	iowrite32((STM_IMSC_IR0_INT_DIS	<< STM_IMSC_IR0_OFFSET),
		       stm_hw_base + REG_STM_IMSC_OFFSET);

	irq_enabled = false;
	iowrite32((STM_CMPXIR0_IRQSC_CMP0CL_CLRC0 <<
			STM_CMPXIR0_IRQSC_CMP0CL_OFFSET) |
		       (STM_CMPXIR0_IRQSC_CMP1CL_CLRC1 <<
			STM_CMPXIR0_IRQSC_CMP1CL_OFFSET),
		       stm_hw_base + REG_STM_CMPXIR0_IRQSC_OFFSET);
}

static void xgold_stm_tick_set_mode(enum clock_event_mode mode,
				    struct clock_event_device *evt)
{
	switch (mode) {
	case CLOCK_EVT_MODE_UNUSED:
		pr_debug("Clock event device set to UNUSED\n");
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		pr_debug("Clock event device set to PERIODIC\n");
		if (!irq_enabled)
			xgold_stm_unmask_irq();
		xgold_stm_reload(period0);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		pr_debug("Clock event device set to SHUTDOWN\n");
		xgold_stm_mask_irq();
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		pr_debug("Clock event device set to ONESHOT\n");
		if (!irq_enabled)
			xgold_stm_unmask_irq();
		break;
	case CLOCK_EVT_MODE_RESUME:
		pr_debug("Clock event device set to RESUME\n");
		break;
	}
}

static int xgold_stm_tick_set_next_event(unsigned long cycles,
					 struct clock_event_device *evt)
{
	xgold_stm_set_event(cycles);
	return 0;
}

irqreturn_t xgold_dyn_timer_isr(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	iowrite32((STM_CMPXIR0_IRQSC_CMP0CL_CLRC0 <<
			STM_CMPXIR0_IRQSC_CMP0CL_OFFSET),
		       stm_hw_base + REG_STM_CMPXIR0_IRQSC_OFFSET);

	last_alarm = 0;

	if (evt->event_handler == NULL)
		return IRQ_HANDLED;

	evt->event_handler(evt);

	if (evt->mode == CLOCK_EVT_MODE_PERIODIC)
		xgold_stm_reload(0);

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
	tmp |= (STM_CMCON_MSIZE1_COMP0 << STM_CMCON_MSIZE1_OFFSET);
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

	xgold_stm_unmask_irq();
}

/**************************************
  Clock source declaration
**************************************/
cycle_t xgold_stm_clock_source_read(struct clocksource *cs)
{

	u64 ull_time;
	u64 ull_time2;

	unsigned *ptime = (unsigned *)&ull_time;
	unsigned *ptime2 = (unsigned *)&ull_time2;

	ptime[0] = ioread32(stm_hw_base + REG_STM_TIM0_OFFSET);
	ptime[1] = ioread32(stm_hw_base + REG_STM_CAP_OFFSET);
	ptime2[0] = ioread32(stm_hw_base + REG_STM_TIM0_OFFSET);

	/* Detect overflow condition by reading the TIM0 again
	 *  if overflow, read the CAP again to get the correct time
	 */
	if (ptime[0] <= ptime2[0])
		return ull_time;

	ptime2[1] = ioread32(stm_hw_base + REG_STM_CAP_OFFSET);
	return ull_time2;
}

static void xgold_stm_evt_suspend(struct clock_event_device *evt_device)
{
	xgold_stm_mask_irq();
}

static void xgold_stm_evt_resume(struct clock_event_device *evt_device)
{
	xgold_stm_unmask_irq();
}

static struct clock_event_device xgold_stm_clockevent = {
	.name = "xgold_stm_clockevent",
	.features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_MODE_PERIODIC,
	.shift = 32,		/*FIXME : Refine this choice */
	.rating = 10,
	.set_next_event = xgold_stm_tick_set_next_event,
	.set_mode = xgold_stm_tick_set_mode,
	.suspend = xgold_stm_evt_suspend,
	.resume = xgold_stm_evt_resume,

};

static struct clocksource xgold_stm_clocksource = {
	.name = "xgold_stm_clocksource",
	.rating = 300,		/*FIXME: Refine this choice */
	.read = xgold_stm_clock_source_read,
	.mask = CLOCKSOURCE_MASK(56),
	.shift = 24,		/*FIXME: Refine this choice */
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct irqaction xgold_stm_irq = {
	.name = "stm_timer",
	.flags = IRQF_DISABLED,
	.handler = xgold_dyn_timer_isr,
	.dev_id = &xgold_stm_clockevent,
};

#ifdef CONFIG_ARM
static notrace u32 xgold_sched_clock_read(void)
{
	return ioread32(stm_hw_base + REG_STM_TIM0_OFFSET);
}
#endif

static void __init xgold_of_timer_map(struct device_node *np)
{
	int ret;
	unsigned int intspec, faf;

	stm_clk = of_clk_get_by_name(np, "kernel");

	if (stm_clk != ERR_PTR(-ENOENT))
		clk_rate = clk_get_rate(stm_clk);
	clk_prepare_enable(stm_clk);
	stm_hw_base = of_iomap(np, 0);
	if (!stm_hw_base)
		panic("unable to map timer cpu registers\n");

	/* Get the interrupts property */
	intspec = irq_of_parse_and_map(np, 0);

	BUG_ON(!intspec);
	xgold_stm_irq.irq = intspec;

	ret = of_property_read_u32(np, "intel,faf", &faf);
	if (!ret) {
		pr_info("%s:Initializing STM FAF to %#x\n", __func__, faf);
		iowrite32(faf, stm_hw_base + 0x70);
	}
}

#ifdef CONFIG_XGOLD_STM_TIMER_SCHED_CLOCK
unsigned long long notrace sched_clock(void)
{
	cycle_t cycle;

	if (unlikely(stm_disabled))
		return (jiffies_64 - INITIAL_JIFFIES) * (NSEC_PER_SEC / HZ);

	cycle = xgold_stm_clock_source_read(NULL);
	cycle &= xgold_stm_clocksource.mask;

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

	BUG_ON(setup_irq(xgold_stm_irq.irq, &xgold_stm_irq));
	clockevents_calc_mult_shift(ce, clk_rate, 180);

/*FIXME: Not sure about max/min_delta_ns */
	ce->max_delta_ns = clockevent_delta2ns(0xffffffff, ce);
	ce->min_delta_ns = clockevent_delta2ns(1024, ce);
	ce->cpumask = cpumask_of(0);
	clockevents_register_device(ce);
#ifdef CONFIG_X86
	global_clock_event = &xgold_stm_clockevent;
#endif

}

CLOCKSOURCE_OF_DECLARE(xgold_timer_stm, "intel,stm", xgold_timer_init);

