/*
 * Copyright (c) 2015 Intel Mobile Communications GmbH
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
#include "cc_regs.h"

#define XGOLD_CC_EVT_DRV_NAME "CAPCOM"

#define XGOLD_CC_EVT_MAX_IRQ 2
#define XGOLD_CC_EVT_NAME "xgold CAPCOM clockevent"
#define XGOLD_CC_EVT_FEATURES  (CLOCK_EVT_FEAT_ONESHOT \
		| CLOCK_EVT_FEAT_PERIODIC)
#define XGOLD_CC_EVT_RATING 10
#define XGOLD_CC_EVT_CLK_RATE (26 * 1e6)
#define XGOLD_CC_EVT_SHIFT 32
#define XGOLD_CC_TIMER_MAX_VALUE 0x80000000 /* 0x7FFFFFFF */
#define XGOLD_CC_TRACE pr_err("%s:[%d]--> line %d\n",\
		__func__, smp_processor_id(), __LINE__)

static struct cpumask capcom_cpumask = CPU_MASK_NONE;

struct xgold_cc_hw {
	char name[32];
	void __iomem *base;
	int irqs[XGOLD_CC_EVT_MAX_IRQ];
	spinlock_t lock;
	struct cpumask mask;
	int rating;
	unsigned long clk_rate;
	unsigned period_cycles;
	unsigned int min_delta_ns;
	unsigned int max_delta_ns;
};

#define XGOLD_CC_EVT_ENABLE		BIT(0)
#define XGOLD_CC_EVT_MODE_PERIODIC	BIT(1)

struct xgold_cc_clkevt {
	struct clock_event_device evt;
	struct xgold_cc_hw *hw;
	unsigned cpu;
	unsigned id;
	bool enable;
	unsigned last_alarm;
	unsigned flags;
};

static inline bool xgold_cc_evt_is_enabled(struct xgold_cc_clkevt *cc_evt)
{
	return cc_evt->flags & XGOLD_CC_EVT_ENABLE;
}

static inline bool xgold_cc_evt_is_mode_periodic(struct xgold_cc_clkevt *cc_evt)
{
	return cc_evt->flags & XGOLD_CC_EVT_MODE_PERIODIC;
}

static inline bool xgold_cc_evt_is_mode_oneshot(struct xgold_cc_clkevt *cc_evt)
{
	return !(xgold_cc_evt_is_mode_periodic(cc_evt));
}

static DEFINE_PER_CPU(struct xgold_cc_clkevt, cc_events);

/* CAPCOM register handling */
static void xgold_cc_hw_init(struct xgold_cc_hw *hw)
{
	void __iomem *base = hw->base;
	unsigned long flags;
	unsigned reg;

	spin_lock_irqsave(&hw->lock, flags);
	reg = ioread32(CC_CLC(base));
	reg &= ~CC_CLC_ORMC_MASK;
	reg &= ~CC_CLC_RMC_MASK;
	reg |= (1 << CC_CLC_RMC_OFFSET);
/*	reg |= (1 << CC_CLC_ORMC_OFFSET); */
	iowrite32(reg, CC_CLC(base));

	reg = CC_T01CON_T1M_TM | CC_T01CON_T0M_TM;
	iowrite32(reg, CC_T01CON(base));
	reg = CC_IMSC_T1_EN | CC_IMSC_T0_EN;
	iowrite32(reg, CC_IMSC(base));

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void xgold_cc_evt_enable(struct xgold_cc_clkevt *cc_evt)
{
	unsigned reg;
	void __iomem *base = cc_evt->hw->base;

	reg = ioread32(CC_T01CON(base));
	if (cc_evt->id == 0)
		reg |= CC_T01CON_T0R_EN;
	else
		reg |= CC_T01CON_T1R_EN;
	iowrite32(reg, CC_T01CON(base));

	cc_evt->flags |= XGOLD_CC_EVT_ENABLE;
}

static void xgold_cc_evt_disable(struct xgold_cc_clkevt *cc_evt)
{
	unsigned reg;
	void __iomem *base = cc_evt->hw->base;

	reg = ioread32(CC_T01CON(base));
	if (cc_evt->id == 0)
		reg &= ~CC_T01CON_T0R_EN;
	else
		reg &= ~CC_T01CON_T1R_EN;
	iowrite32(reg, CC_T01CON(base));

	cc_evt->flags &= ~XGOLD_CC_EVT_ENABLE;
}

static inline struct xgold_cc_clkevt *
				evt_to_cc_dev(struct clock_event_device *evt)
{
	return container_of(evt, struct xgold_cc_clkevt, evt);
}

static inline void xgold_cc_evt_disable_safe(struct xgold_cc_clkevt *cc_evt)
{
	spinlock_t *lock = &cc_evt->hw->lock;
	unsigned long flags;

	spin_lock_irqsave(lock, flags);
	xgold_cc_evt_disable(cc_evt);
	spin_unlock_irqrestore(lock, flags);
}

static inline void xgold_cc_evt_enable_safe(struct xgold_cc_clkevt *cc_evt)
{
	spinlock_t *lock = &cc_evt->hw->lock;
	unsigned long flags;

	spin_lock_irqsave(lock, flags);
	xgold_cc_evt_enable(cc_evt);
	spin_unlock_irqrestore(lock, flags);
}


static void xgold_cc_evt_suspend(struct clock_event_device *evt)
{
	struct xgold_cc_clkevt *cc_evt = evt_to_cc_dev(evt);
	xgold_cc_evt_disable_safe(cc_evt);
}

static void xgold_cc_evt_resume(struct clock_event_device *evt)
{
	struct xgold_cc_clkevt *cc_evt = evt_to_cc_dev(evt);
	xgold_cc_evt_enable_safe(cc_evt);
}

static inline void xgold_cc_evt_clear(struct xgold_cc_clkevt *cc_evt)
{
	void __iomem *base = cc_evt->hw->base;
	unsigned icr, ovf;

	if (cc_evt->id == 0) {
		icr = CC_ICR_T0_CSR;
		ovf = CC_T01OCR_CT0_CSR;
	} else {
		icr = CC_ICR_T1_CSR;
		ovf = CC_T01OCR_CT1_CSR;
	}

	iowrite32(ovf, CC_T01OCR(base));
	iowrite32(icr, CC_ICR(base));
}

static void _xgold_cc_set_mode(struct xgold_cc_clkevt *cc_evt, bool periodic)
{
	spinlock_t *lock = &cc_evt->hw->lock;
	void __iomem *base = cc_evt->hw->base;
	unsigned period;
	unsigned long flags;

	if (xgold_cc_evt_is_mode_periodic(cc_evt))
		return;

	spin_lock_irqsave(lock, flags);

	if (xgold_cc_evt_is_enabled(cc_evt))
		xgold_cc_evt_disable(cc_evt);

	if (periodic)
		period = cc_evt->hw->period_cycles;
	else
		period = 0;

	if (cc_evt->id == 0) {
		iowrite32(period, CC_T0REL(base));
		iowrite32(period, CC_T0(base));
	} else {
		iowrite32(period, CC_T1REL(base));
		iowrite32(period, CC_T1(base));
	}

	if (periodic) {
		cc_evt->flags |= XGOLD_CC_EVT_MODE_PERIODIC;
		xgold_cc_evt_enable(cc_evt);
	} else {
		cc_evt->flags &= ~XGOLD_CC_EVT_MODE_PERIODIC;
	}

	spin_unlock_irqrestore(lock, flags);

}

static inline void xgold_cc_set_periodic(struct xgold_cc_clkevt *cc_evt)
{
	_xgold_cc_set_mode(cc_evt, true);
}

static inline void xgold_cc_set_oneshot(struct xgold_cc_clkevt *cc_evt)
{
	_xgold_cc_set_mode(cc_evt, false);
}

static void xgold_cc_set_mode(enum clock_event_mode mode,
				    struct clock_event_device *evt)
{
	struct xgold_cc_clkevt *cc_evt = evt_to_cc_dev(evt);
	unsigned cpu = smp_processor_id();

	switch (mode) {
	case CLOCK_EVT_MODE_UNUSED:
		pr_info("Clock event device %d set to UNUSED\n", cpu);
		/*TODO: shutdown hardware */
		xgold_cc_evt_disable_safe(cc_evt);
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		pr_info("Clock event device %d set to PERIODIC\n", cpu);
		xgold_cc_set_periodic(cc_evt);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		pr_info("Clock event device %d set to SHUTDOWN\n", cpu);
		xgold_cc_evt_disable_safe(cc_evt);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		pr_info("Clock event device %d set to ONESHOT\n", cpu);
		xgold_cc_set_oneshot(cc_evt);
		break;
	case CLOCK_EVT_MODE_RESUME:
		pr_info("Clock event device %d set to RESUME\n", cpu);
		xgold_cc_evt_enable_safe(cc_evt);
		break;
	}
}


static int xgold_cc_set_next_event(unsigned long cycles,
					 struct clock_event_device *evt)
{
	struct xgold_cc_clkevt *cc_evt = evt_to_cc_dev(evt);
	void __iomem *base = cc_evt->hw->base;
	spinlock_t *lock = &cc_evt->hw->lock;
	unsigned long flags;
	unsigned alarm = XGOLD_CC_TIMER_MAX_VALUE;

	alarm -= cycles;

	spin_lock_irqsave(lock, flags);
	if (unlikely(xgold_cc_evt_is_enabled(cc_evt)))
		xgold_cc_evt_disable(cc_evt);

	/* Clear the overflow flag */
	if (cc_evt->id == 0) {
		iowrite32(CC_T01OCR_CT0_CSR, CC_T01OCR(base));
		iowrite32(alarm, CC_T0(base));
	} else {
		iowrite32(CC_T01OCR_CT1_CSR, CC_T01OCR(base));
		iowrite32(alarm, CC_T1(base));
	}

	xgold_cc_evt_enable(cc_evt);
	spin_unlock_irqrestore(lock, flags);
#if 0
	/* Did we miss the interrupt ? retirgger it if needed */
	if (cc_evt->id == 0) {
		if  (unlikely(CC_T0_OVF0(ioread32(CC_T0(base)))))
			iowrite32(CC_ISR_T0_SI, CC_ISR(base));
	} else {
		if (unlikely(CC_T1_OVF1(ioread32(CC_T1(base)))))
			iowrite32(CC_ISR_T1_SI, CC_ISR(base));
	}
#endif
	return 0;
}

static irqreturn_t xgold_cc_timer_isr(int irq, void *dev_id)
{
	struct xgold_cc_clkevt *cc_evt = dev_id;
	struct clock_event_device *evt = &cc_evt->evt;
	spinlock_t *lock = &cc_evt->hw->lock;
	unsigned long flags;

	/* next_event will enable the interrupt */
	if (xgold_cc_evt_is_mode_oneshot(cc_evt)) {
		spin_lock_irqsave(lock, flags);
		xgold_cc_evt_disable(cc_evt);
		spin_unlock_irqrestore(lock, flags);
	}

	xgold_cc_evt_clear(cc_evt);
	cc_evt->last_alarm = 0;

	if (evt->event_handler == NULL)
		return IRQ_HANDLED;

	evt->event_handler(evt);

	return IRQ_HANDLED;

}

static int xgold_cc_evt_register(void)
{
	int ret;
	unsigned cpu = smp_processor_id();
	struct xgold_cc_clkevt *cc_evt;
	struct clock_event_device *evt;

	cc_evt =  &__get_cpu_var(cc_events);
	if (cc_evt->hw == NULL) {
		pr_err("%s: cpu: %d:per CPU capcom events not initialized\n",
				__func__, cpu);
		return -EINVAL;
	}

	evt = &cc_evt->evt;

	if (!cpumask_test_cpu(cpu, evt->cpumask)) {
		pr_err("Capcom not used for cpu %d\n", cpu);
		return -EINVAL;
	}

	ret = request_irq(evt->irq, xgold_cc_timer_isr,
		IRQF_TIMER | IRQF_DISABLED | IRQF_NOBALANCING | IRQF_PERCPU,
		evt->name, cc_evt);
	BUG_ON(ret);

	disable_irq(evt->irq);
	irq_set_affinity(evt->irq, evt->cpumask);
	enable_irq(evt->irq);

	pr_info("Registering Capcom as clock event for cpu %d\n", cpu);
	clockevents_register_device(evt);

	return 0;
}

struct xgold_cc_work_struct {
	struct delayed_work work;
	struct completion complete;
};

static void xgold_cc_work(struct work_struct *w)
{
	struct xgold_cc_work_struct *xgold_cc_work;
	xgold_cc_work = container_of(w, struct xgold_cc_work_struct, work.work);

	xgold_cc_evt_register();

	complete(&xgold_cc_work->complete);
}

static int xgold_cc_cpu_notify(struct notifier_block *n,
		unsigned long action, void *hcpu)
{
	unsigned long cpu = (unsigned long) hcpu;
	struct xgold_cc_work_struct work;
	struct xgold_cc_clkevt *cc_evt = &per_cpu(cc_events, cpu);
	struct clock_event_device *evt = &cc_evt->evt;

	if (cc_evt->hw == NULL)
		return NOTIFY_OK;

	switch (action & 0xf) {
	case CPU_ONLINE:
		INIT_DELAYED_WORK_ONSTACK(&work.work, xgold_cc_work);
		init_completion(&work.complete);
		/* FIXME: add schedule_work_on() */
		schedule_delayed_work_on(cpu, &work.work, 0);
		wait_for_completion(&work.complete);
		destroy_timer_on_stack(&work.work.timer);
		break;
	case CPU_DEAD:
		if (evt->irq)
			free_irq(evt->irq, cc_evt);

		if (cc_evt->flags & XGOLD_CC_EVT_ENABLE)
			xgold_cc_evt_disable(cc_evt);
		break;
	}
	return NOTIFY_OK;
}

static void __init xgold_cc_hw_register(struct xgold_cc_hw *cc_hw)
{
	int cpu, i = 0;
	struct xgold_cc_clkevt *cc_evt;
	struct clock_event_device *evt;

	pr_info("%s: Registering cc event HW:\n\t%lu Hz, mask %lx, rating %d\n",
			XGOLD_CC_EVT_DRV_NAME,
			cc_hw->clk_rate,
			cpumask_bits(&cc_hw->mask)[0],
			cc_hw->rating);

	for_each_cpu_mask(cpu, cc_hw->mask) {
		cc_evt = &per_cpu(cc_events, cpu);
		cc_evt->cpu = cpu;
		cc_evt->id = i;
		cc_evt->hw = cc_hw;
		evt = &cc_evt->evt;
		evt->name = "Clk event CapCom";
		evt->irq = cc_hw->irqs[i++];
		evt->rating = cc_hw->rating;
		evt->cpumask = cpumask_of(cpu);
		evt->suspend = xgold_cc_evt_suspend;
		evt->resume = xgold_cc_evt_resume;
		evt->set_next_event = xgold_cc_set_next_event;
		evt->set_mode = xgold_cc_set_mode;
		evt->shift = XGOLD_CC_EVT_SHIFT;
		evt->features = XGOLD_CC_EVT_FEATURES;
		clockevents_calc_mult_shift(evt, cc_hw->clk_rate, 180);
		evt->max_delta_ns = cc_hw->max_delta_ns;
		evt->min_delta_ns = cc_hw->min_delta_ns;
		pr_info("Initializing capcom clock events %s, mask %lu,cc id %d, mult %x, shift %x, max %lli, min %lli\n",
			evt->name,
			cpumask_bits(evt->cpumask)[0],
			cc_evt->id,
			evt->mult,
			evt->shift,
			evt->max_delta_ns,
			evt->min_delta_ns
			);
	}

	cpumask_or(&capcom_cpumask, &capcom_cpumask, &cc_hw->mask);
}

void __init xgold_timer_cc_init(struct device_node *np)
{
	int ret, cpu;
	unsigned i, nr_irqs, evt_min, evt_max;
	unsigned long mask;
	unsigned evt_rating = XGOLD_CC_EVT_RATING;
	unsigned clk_rate = XGOLD_CC_EVT_CLK_RATE;
	struct xgold_cc_hw *cc_hw;
	cc_hw = kzalloc(sizeof(*cc_hw), GFP_KERNEL);
	BUG_ON(cc_hw == NULL);

	cc_hw->base = of_iomap(np, 0);
	BUG_ON(cc_hw->base == NULL);

	nr_irqs = of_irq_count(np);
	BUG_ON(((nr_irqs > (XGOLD_CC_EVT_MAX_IRQ)) || (nr_irqs == 0)));

	/* Get the interrupts property */
	for (i = 0; i < nr_irqs; i++) {
		cc_hw->irqs[i] = irq_of_parse_and_map(np, i);
		BUG_ON(cc_hw->irqs[i] == 0);
	}

	ret = of_property_read_u32(np, "intel,cc,evt,rating", &evt_rating);
	if (!ret)
		pr_info("%s:Clock event device rating set to %#x\n",
							__func__, evt_rating);

	cc_hw->rating = evt_rating;

	ret = of_property_read_u32(np, "intel,cc,rate", &clk_rate);
	if (!ret)
		pr_info("%s:Input clock rate set to %#x\n",
				__func__, clk_rate);

	ret = of_property_read_u32(np, "intel,evt,min-ns", &evt_min);
	if (!ret)
		cc_hw->min_delta_ns = evt_min;
	else
		cc_hw->min_delta_ns = 38000;

	pr_info("%s:Clock event min set to %d\n",
			__func__, cc_hw->min_delta_ns);

	ret = of_property_read_u32(np, "intel,evt,max-ns", &evt_max);
	if (!ret)
		cc_hw->max_delta_ns = evt_max;
	else
		cc_hw->max_delta_ns = 1000000000;

	pr_info("%s:Clock event max set to %d\n",
			__func__, cc_hw->max_delta_ns);

	cc_hw->clk_rate = clk_rate;
	/* Counting upwards */
	cc_hw->period_cycles = XGOLD_CC_TIMER_MAX_VALUE;
	cc_hw->period_cycles -= (clk_rate / HZ);
	mask = 0;
	ret = of_property_read_u32(np, "intel,cc,cpumask", (unsigned *)&mask);
	if (ret) {
		mask = BIT(XGOLD_CC_EVT_MAX_IRQ) - 1;
		pr_info("%s: Set default cpumask to %lx\n",
				__func__, mask);
	}

	for_each_set_bit(cpu, &mask, nr_cpu_ids) {
		pr_info("%s: Set cpu%d as valid for CC usage\n", __func__, cpu);
		cpumask_set_cpu(cpu, &cc_hw->mask);
	}

	spin_lock_init(&cc_hw->lock);

	xgold_cc_hw_register(cc_hw);

	/* Initialize the hardware */
	xgold_cc_hw_init(cc_hw);

	if (cpumask_test_cpu(smp_processor_id(), &cc_hw->mask))
		xgold_cc_evt_register();
}

static __init int xgold_cc_late_init(void)
{
	unsigned cpu;

	for_each_online_cpu(cpu) {
		xgold_cc_cpu_notify(NULL, CPU_ONLINE, (void *)(long)cpu);
	}

	/* This notifier should be called after workqueue is ready */
	hotcpu_notifier(xgold_cc_cpu_notify, CPU_PRI_WORKQUEUE_UP);
	return 0;
}
early_initcall(xgold_cc_late_init);

CLOCKSOURCE_OF_DECLARE(xgold_timer_cc, "intel,capcom,clocksource", xgold_timer_cc_init);

