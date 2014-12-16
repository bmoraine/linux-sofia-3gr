/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include "noc.h"
#include "noc_regs.h"

/* In the line describing the measurement to be done, trace port select info
 * is at position 0 */
#define IDX_TRACE_PORT_SEL	0
/* In the line describing the measurement to be done, init flow info is at
 * position 1 */
#define IDX_INIT_FLOW		1
/* In the line describing the measurement to be done, target flow info is
 * at position 2 */
#define IDX_TARGET_FLOW		2
/* There are either:
 3 data to describe a measurement: trace port select + init flow + target flow
 or 1 data to describe a measurement: counter port select */
#define IDX_NUMBER		3

/**
 * Create a new stat_measure from parsed dts entry
 * @param idx_trace_port_sel First entry in the dts line stat,x as index
 * @param idx_init_flow Second entry in the dts line stat,x as index
 * @param idx_target_flow Third entry in the dts line stat,x as index
 * @param str Same values as above but as a table of string
 * @return The created xgold_noc_stat_measure object
 */
static struct xgold_noc_stat_measure *new_stat_measure(struct
						       xgold_noc_stat_probe
						       *stat_probe,
						       unsigned
						       idx_trace_port_sel,
						       unsigned idx_init_flow,
						       unsigned idx_target_flow,
						       const char **str)
{
	struct xgold_noc_stat_measure *stat_measure;
	unsigned long flags;

	stat_measure = kzalloc(sizeof(struct xgold_noc_stat_measure),
			       GFP_KERNEL);
	if (!stat_measure)
		return ERR_PTR(-ENOMEM);
	INIT_LIST_HEAD(&stat_measure->link);
	spin_lock_init(&stat_measure->lock);
	stat_measure->idx_trace_port_sel = idx_trace_port_sel;
	stat_measure->idx_init_flow = idx_init_flow;
	stat_measure->idx_target_flow = idx_target_flow;
	if (idx_init_flow != -1) {
		stat_measure->init_flow = kstrdup(str[IDX_INIT_FLOW],
						  GFP_KERNEL);
		stat_measure->target_flow = kstrdup(str[IDX_TARGET_FLOW],
						    GFP_KERNEL);
	}
	stat_measure->min = -1;	/* Set to max value */

	spin_lock_irqsave(&stat_probe->lock, flags);
	list_add_tail(&stat_measure->link, &stat_probe->measure);
	spin_unlock_irqrestore(&stat_probe->lock, flags);

	return stat_measure;
}

/**
 * Create a new stat_probe. A stat_probe contains all measurements which
 * are linked to the same probe
 * @param noc_stat the stat_probe will be attached to noc_stat
 * @param probe the stat_probe will refer to probe
 * @return The created xgold_noc_stat_probe object
 */
static struct xgold_noc_stat_probe *new_stat_probe(struct xgold_noc_stat
						   *noc_stat,
						   struct xgold_noc_probe
						   *probe)
{
	struct xgold_noc_stat_probe *stat_probe;
	unsigned long flags;

	stat_probe = kzalloc(sizeof(struct xgold_noc_stat_probe), GFP_KERNEL);
	if (!stat_probe)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&stat_probe->measure);
	INIT_LIST_HEAD(&stat_probe->link);
	spin_lock_init(&stat_probe->lock);

	spin_lock_irqsave(&noc_stat->lock, flags);
	list_add_tail(&stat_probe->link, &noc_stat->probe);
	spin_unlock_irqrestore(&noc_stat->lock, flags);
	stat_probe->probe = probe;

	return stat_probe;
}

/**
 * From one line of dts, add a new stat_measure and a stat_probe (if needed)
 * @param noc_device A ref to the device
 * @param np A ref to the dts node
 * @param name the line of the dts
 * @return
 */
static int new_probe_and_measure(struct xgold_noc_device *noc_device,
				 struct device_node *np, const char *name)
{
	int lg, i, j, index;

	struct xgold_noc_stat_probe *stat_probe, *entry;
	struct xgold_noc_stat *noc_stat = noc_device->stat;
	struct xgold_noc_probe *probe;
	struct xgold_noc_stat_measure *stat_measure;

	const char *str[IDX_NUMBER], *s;
	struct property *prop;
	struct list_head *ptr;

	unsigned idx_trace_port_sel, idx_init_flow, idx_target_flow;
	bool found;

	/* Count number of strings per measurement
	 * (normally 3: TracePortSel, initFlow, targetFlow) */
	lg = of_property_count_strings(np, name);
	if (lg != IDX_NUMBER && lg != 1)
		return -EINVAL;	/* Invalid data, let's skip this measurement */

	index = 0;
	of_property_for_each_string(np, name, prop, s)
		str[index++] = s;

	/* Search which probe can be used with TracePortSel equal to
	 * first value in dts */
	for (i = 0; i < noc_device->nr_probes; i++) {
		probe = &noc_device->probes[i];
		for (j = 0; probe->available_portsel[j] != NULL; j++) {
			if (str[0] && strcmp(str[0],
				probe->available_portsel[j]) == 0)
				break;
		}
		if (probe->available_portsel[j] != NULL)
			break;
	}
	if (i == noc_device->nr_probes)
		return -EINVAL;	/* Invalid data, let's skip this measurement. */

	idx_trace_port_sel = j;	/* Numerical value of TracePortSel */

	idx_init_flow = idx_target_flow = -1;

	if (index == 3) {
		/* Search for init flow index and target flow index; */
		struct xgold_register *reg;
		struct xgold_bitfield *bf;
		int i;

		reg = probe->filters->route_id;
		list_for_each_entry(bf, &reg->bitfields, link) {
			if (strcmp(bf->name, "InitFlow") == 0) {
				/* FIXME hardcoded */
				i = 0;
				while (bf->lut[i] != NULL) {
					if (str[IDX_INIT_FLOW] &&
						strcmp(str[IDX_INIT_FLOW],
						bf->lut[i]) == 0) {
						idx_init_flow = i;
						break;
					}
					i += 1;
				}
			}
			if (strcmp(bf->name, "TargetFlow") == 0) {
				/* FIXME string is hardcoded */
				i = 0;
				while (bf->lut[i] != NULL) {
					if (str[IDX_TARGET_FLOW] &&
						strcmp(str[IDX_TARGET_FLOW],
						bf->lut[i]) == 0) {
						idx_target_flow = i;
						break;
					}
					i += 1;
				}
			}
		}
		if (idx_init_flow == -1 || idx_target_flow == -1)
			/* Invalid data, let's skip this measurement. */
			return -EINVAL;
	} else {
	}

	/* Look if the probe was already existing in noc_stat */
	if (!list_empty(&noc_stat->probe)) {
		found = false;
		list_for_each(ptr, &noc_stat->probe) {
			entry = list_entry(ptr, struct xgold_noc_stat_probe,
					   link);
			if (entry->probe == probe) {
				/* It is found, create a new measure */
				stat_measure = new_stat_measure(entry,
							idx_trace_port_sel,
							idx_init_flow,
							idx_target_flow,
							str);
				if (IS_ERR(stat_measure))
					goto error;
				found = true;
				break;
			}
		}
		if (found == false) {
			/* Not found, we must create a stat_probe */
			stat_probe = new_stat_probe(noc_stat, probe);
			if (IS_ERR(stat_probe))
				goto error;
			/* Then add the measurement */
			stat_measure = new_stat_measure(stat_probe,
							idx_trace_port_sel,
							idx_init_flow,
							idx_target_flow, str);
			if (IS_ERR(stat_measure))
				goto free_stat_probe;
		}
	} else {
		/* No stat_probe created up to now, let's create one. */
		stat_probe = new_stat_probe(noc_stat, probe);
		if (IS_ERR(stat_probe))
			goto error;
		/* Then add the measurement */
		stat_measure = new_stat_measure(stat_probe, idx_trace_port_sel,
						idx_init_flow, idx_target_flow,
						str);
		if (IS_ERR(stat_measure))
			goto free_stat_probe;
	}
	return 0;

free_stat_probe:
	kfree(stat_probe);
error:
	return -ENOMEM;
}

/**
 * Return the value of a bitfield
 * @param bf the bitfield to read
 * @return the value of the bitfield
 */
static unsigned xgold_noc_bf_read(struct xgold_bitfield *bf)
{
	struct xgold_register *reg;
	unsigned bf_value;

	reg = bf->parent;
	bf_value = ioread32(XGOLDREG_ADDR(reg));
	bf_value &= bf->mask;
	bf_value >>= bf->offset;

	return bf_value;
}

/**
 * Write some bits in a register
 * @param reg The register to write
 * @param mask The mask for the bits (shifted)
 * @param offset The offset of the bits
 * @param value The value to write
 * @return 0 if ok, -EINVAL if value is not valid
 */
static int xgold_noc_reg_write(struct xgold_register *reg, unsigned mask,
			       unsigned offset, unsigned value)
{
	unsigned reg_value;
	unsigned long flags;

	if (value > (mask >> offset))
		return -EINVAL;

	spin_lock_irqsave(&reg->hw_lock, flags);
	reg_value = ioread32(XGOLDREG_ADDR(reg));
	reg_value &= ~mask;
	reg_value |= (value << offset);
	iowrite32(reg_value, XGOLDREG_ADDR(reg));
	spin_unlock_irqrestore(&reg->hw_lock, flags);

	return 0;
}

/**
 * Read some bits in a register
 * @param reg The register to read
 * @param mask The mask for the bits (shifted)
 * @param offset The offset of the bits
 * @return the read value
 */
static unsigned xgold_noc_reg_read(struct xgold_register *reg, unsigned mask,
				   unsigned offset)
{
	unsigned reg_value;

	reg_value = ioread32(XGOLDREG_ADDR(reg));
	reg_value &= ~mask;
	reg_value >>= offset;

	return reg_value;
}

/**
 * Configure NOC registers from a stat_measure
 * @param noc_device A reference to the device
 * @param stat_probe The stat_probe to find the measure
 * @return 0 if ok, -EINVAL otherwise
 */
int xgold_noc_stat_configure_measure(struct xgold_noc_device *noc_device,
				     struct xgold_noc_stat_probe *stat_probe)
{
	struct xgold_noc_stat_measure *stat_measure;
	struct xgold_noc_probe *probe;
	int i, ret;

	stat_measure = list_first_entry(&stat_probe->measure,
					struct xgold_noc_stat_measure, link);
	probe = stat_probe->probe;

	/* Disable NOC */
	ret = xgold_noc_reg_write(probe->cfg_ctl, PROBE_CFGCTL_GLOBALEN_MASK,
				  PROBE_CFGCTL_GLOBALEN_OFFSET, 0);
	while (xgold_noc_reg_read(probe->cfg_ctl, PROBE_CFGCTL_ACTIVE_WIDTH,
				  PROBE_CFGCTL_ACTIVE_OFFSET) != 0)
		;

	if (stat_measure->idx_init_flow != 0xff) {
		/* Select probe TracePortSel */
		ret |= xgold_noc_reg_write(probe->trace_port_sel,
				   PROBE_TRACEPORTSEL_TRACEPORTSEL_MASK,
				   PROBE_TRACEPORTSEL_TRACEPORTSEL_OFFSET,
				   stat_measure->idx_trace_port_sel);
	} else {
		/* Select counter PortSel */
		ret |= xgold_noc_reg_write(probe->counters[0].portsel->parent,
			   PROBE_COUNTERS_PORTSEL_COUNTERS_PORTSEL_MASK,
			   PROBE_COUNTERS_PORTSEL_COUNTERS_PORTSEL_OFFSET,
			   stat_measure->idx_trace_port_sel);
	}

	/* Reset Filters */
	for (i = 0; i < noc_device->nr_filters; i++) {
		ret |= xgold_noc_reg_write(probe->filters[i].route_mask->parent,
			   PROBE_FILTERS_ROUTEIDBASE_FILTERS_ROUTEIDBASE_MASK,
			   PROBE_FILTERS_ROUTEIDBASE_FILTERS_ROUTEIDBASE_OFFSET,
			   0);
		ret |=
		    xgold_noc_reg_write(probe->filters[i].window_size->parent,
			PROBE_FILTERS_WINDOWSIZE_FILTERS_WINDOWSIZE_MASK,
			PROBE_FILTERS_WINDOWSIZE_FILTERS_WINDOWSIZE_OFFSET,
			0x3f);
		ret |=
		    xgold_noc_reg_write(probe->filters[i].security_mask->parent,
			PROBE_FILTERS_SECURITYMASK_FILTERS_SECURITYMASK_MASK,
			PROBE_FILTERS_SECURITYMASK_FILTERS_SECURITYMASK_OFFSET,
			0);
		ret |=
		    xgold_noc_reg_write(probe->filters[i].op_code, 0xf, 0, 0xf);
		ret |=
		    xgold_noc_reg_write(probe->filters[i].status, 0x3, 0, 0x3);
		ret |=
		    xgold_noc_reg_write(probe->filters[i].length->parent,
				PROBE_FILTERS_LENGTH_FILTERS_LENGTH_MASK,
				PROBE_FILTERS_URGENCY_FILTERS_URGENCY_OFFSET,
				0x7);
		ret |=
		    xgold_noc_reg_write(probe->filters[i].urgency->parent,
				PROBE_FILTERS_URGENCY_FILTERS_URGENCY_MASK,
				PROBE_FILTERS_URGENCY_FILTERS_URGENCY_OFFSET,
				0);
	}
	if (stat_measure->idx_init_flow != 0xff) {
		/* Enable Filter 1 */
		/* Search for offset and mask for initflow and targetflow */
		struct xgold_bitfield *bf;
		struct xgold_register *reg = probe->filters[0].route_id;
		unsigned offset_init, offset_target, mask_init, mask_target;

		offset_init = offset_target = mask_init = mask_target = -1;
		list_for_each_entry(bf, &reg->bitfields, link) {
			if (strcmp(bf->name, "InitFlow") == 0) {
				/* FIXME string is hardcoded */
				offset_init = bf->offset;
				mask_init = bf->mask;
			}
			if (strcmp(bf->name, "TargetFlow") == 0) {
				/* FIXME string is hardcoded */
				offset_target = bf->offset;
				mask_target = bf->mask;
			}
		}
		/* It should never happen */
		BUG_ON((offset_init == -1) || (offset_target == -1));

		ret |= xgold_noc_reg_write(probe->filters[0].route_mask->parent,
					   mask_init, offset_init,
					   (mask_init >> offset_init));
		ret |= xgold_noc_reg_write(probe->filters[0].route_mask->parent,
					   mask_target, offset_target,
					   (mask_target >> offset_target));
		ret |= xgold_noc_reg_write(probe->filters[0].route_id,
					   mask_init, offset_init,
					   stat_measure->idx_init_flow);
		ret |= xgold_noc_reg_write(probe->filters[0].route_id,
					   mask_target, offset_target,
					   stat_measure->idx_target_flow);
	}
	if (stat_measure->idx_init_flow != 0xff) {
		/* Enable Filter LUT */
		/* To select only filter 0 */
		ret |= xgold_noc_reg_write(probe->filter_lut,
					   PROBE_FILTERLUT_FILTERLUT_MASK,
					   PROBE_FILTERLUT_FILTERLUT_OFFSET,
					   0xaaaa);
	}
	/* Enable Alarm and statistics */
	ret |= xgold_noc_reg_write(probe->main_ctl, PROBE_MAINCTL_STATEN_MASK,
				   PROBE_MAINCTL_STATEN_OFFSET, 1);
	ret |= xgold_noc_reg_write(probe->main_ctl, PROBE_MAINCTL_ALARMEN_MASK,
				   PROBE_MAINCTL_ALARMEN_OFFSET, 1);

	/* Set period to max value */
	ret |= xgold_noc_reg_write(probe->stat_period,
				   PROBE_STATPERIOD_STATPERIOD_MASK,
				   PROBE_STATPERIOD_STATPERIOD_OFFSET,
				   DURATION);

	/* Select sources for counter0 and 1 */
	if (stat_measure->idx_init_flow != 0xff) {
		ret |=
		    xgold_noc_reg_write(probe->counters[0].source_event->parent,
				PROBE_COUNTERS_SRC_INTEVENT_MASK,
				/* LUT_BYTE_EN = 0x11, LUT_BYTE = 0x12 */
				PROBE_COUNTERS_SRC_INTEVENT_OFFSET,
				0x11);
		ret |=
		    xgold_noc_reg_write(probe->counters[1].source_event->parent,
					PROBE_COUNTERS_SRC_INTEVENT_MASK,
					/* chain */
					PROBE_COUNTERS_SRC_INTEVENT_OFFSET,
					0x10);
	} else {
		ret |=
		    xgold_noc_reg_write(probe->counters[0].source_event->parent,
					PROBE_COUNTERS_SRC_INTEVENT_MASK,
					/* BYTE */
					PROBE_COUNTERS_SRC_INTEVENT_OFFSET,
					0x8);
		ret |=
		    xgold_noc_reg_write(probe->counters[1].source_event->parent,
					PROBE_COUNTERS_SRC_INTEVENT_MASK,
					/* chain */
					PROBE_COUNTERS_SRC_INTEVENT_OFFSET,
					0x10);
	}

	/* Set alarm mode */
	/* min */
	ret |= xgold_noc_reg_write(probe->counters[0].alarm_mode->parent,
			   PROBE_COUNTERS_ALARMMODE_COUNTERS_ALARMMODE_MASK,
			   PROBE_COUNTERS_ALARMMODE_COUNTERS_ALARMMODE_OFFSET,
			   1);
	/* off */
	ret |= xgold_noc_reg_write(probe->counters[1].alarm_mode->parent,
			   PROBE_COUNTERS_ALARMMODE_COUNTERS_ALARMMODE_MASK,
			   PROBE_COUNTERS_ALARMMODE_COUNTERS_ALARMMODE_OFFSET,
			   0);

	/* Set alarm min value */
	ret |= xgold_noc_reg_write(probe->stat_alarm_min,
			   PROBE_STATALARMMIN_STATALARMMIN_MASK,
			   PROBE_STATALARMMIN_STATALARMMIN_OFFSET,
			   0xffffffff);

	/* Enable */
	ret |= xgold_noc_reg_write(probe->cfg_ctl, PROBE_CFGCTL_GLOBALEN_MASK,
				   PROBE_CFGCTL_GLOBALEN_OFFSET, 1);

	return ret;
}

/**
 * Program all noc registers for all probes
 * @param _dev A reference to the device
 * @return 0
 */
static int xgold_noc_stat_configure(struct device *_dev)
{
	struct xgold_noc_device *noc_device = dev_get_drvdata(_dev);
	struct xgold_noc_stat_probe *stat_probe;
	struct xgold_noc_stat *noc_stat = noc_device->stat;
	struct list_head *ptr;
	unsigned ret;

	/* Loop over all probes */
	list_for_each(ptr, &noc_stat->probe) {
		stat_probe = list_entry(ptr, struct xgold_noc_stat_probe, link);
		/* Program first measurement */
		ret = xgold_noc_stat_configure_measure(noc_device, stat_probe);
		if (ret)
			return ret;
	}
	return 0;
}

/**
 * Entry point for the NOC sniffer
 * @param _dev A reference to the device
 * @return 0 if ok, if error returns 0 but print an error message
 */
int xgold_noc_stat_init(struct device *_dev)
{
	struct xgold_noc_stat *noc_stat;
	struct xgold_noc_device *noc_device = dev_get_drvdata(_dev);
	struct device_node *np = _dev->of_node;
	int stat_idx = 0;
	bool ret;
	char mystr[32];
	int err;

	/* Create a stat device. */
	noc_stat = kzalloc(sizeof(struct xgold_noc_stat), GFP_KERNEL);
	if (!noc_stat)
		return -ENOMEM;
	noc_device->stat = noc_stat;
	noc_stat->run = false;
	spin_lock_init(&noc_stat->lock);
	INIT_LIST_HEAD(&noc_stat->probe);

	/* Parse dts and build matrix node */
	do {
		scnprintf(mystr, ARRAY_SIZE(mystr), "stat,%d", stat_idx);
		ret = of_property_read_bool(np, mystr);
		if (ret == true) {
			stat_idx++;
			err = new_probe_and_measure(noc_device, np, mystr);
			if (err)
				dev_err(_dev,
					"Error when adding measure: %s!\n",
					mystr);
		}
	} while (ret == true);

	/* If no stat line in dts then we can release the noc_stat object */
	if (stat_idx == 0)
		goto no_sniffer;

	ret = of_property_read_u32(np, "clock,rate", &noc_stat->clock_rate);
	if (ret) {
		dev_err(_dev,
			"\"clock,rate\" property missing. Will use clk_get_rate() to discover clock rate.");
	}

	/* Program measurement */
	if (noc_stat->run) {
		ret = xgold_noc_stat_configure(_dev);
		if (ret)
			dev_err(_dev,
				"Error when configuring stat measures!\n");
	}
	return 0;

no_sniffer:
	kfree(noc_stat);
	return -EINVAL;
}

/**
 * Interrupt processing when a measurement is done
 * @param noc_device A reference to the device
 * @param probe The probe for which a measurement is done
 */
void xgold_noc_stat_measure(struct xgold_noc_probe *probe)
{
	struct xgold_noc_device *noc_device = probe->parent;
	struct xgold_noc_stat_probe *stat_probe;
	struct xgold_noc_stat_measure *stat_measure;
	unsigned counter;
	unsigned ret;
	unsigned long flags;

	/* Find which stat_probe is linked to the probe
	 * which generated the interrupt */
	list_for_each_entry(stat_probe, &noc_device->stat->probe, link) {
		if (stat_probe->probe == probe)
			break;
	}

	/* Take measure object */
	stat_measure = list_first_entry(&stat_probe->measure,
					struct xgold_noc_stat_measure, link);

	/* Read HW counters */
	counter = xgold_noc_bf_read(probe->counters[1].value);
	counter = ((counter << 16)
		   | (xgold_noc_bf_read(probe->counters[0].value)));

	/* Update stat_measure statistics */
	if (counter > stat_measure->max) {
		spin_lock_irqsave(&stat_measure->lock, flags);
		stat_measure->max = counter;
		spin_unlock_irqrestore(&stat_measure->lock, flags);
	}
	if (counter < stat_measure->min) {
		spin_lock_irqsave(&stat_measure->lock, flags);
		stat_measure->min = counter;
		spin_unlock_irqrestore(&stat_measure->lock, flags);
	}
	stat_measure->now = counter;
	if (stat_measure->iteration != MAX_ITERATION) {
		spin_lock_irqsave(&stat_measure->lock, flags);
		stat_measure->iteration += 1;
		spin_unlock_irqrestore(&stat_measure->lock, flags);

		spin_lock_irqsave(&stat_measure->lock, flags);
		stat_measure->mean += counter;
		spin_unlock_irqrestore(&stat_measure->lock, flags);
	}

	/* Program next measure */
	if (noc_device->stat->run == true) {
		/* Move to the next measure */
		spin_lock_irqsave(&stat_probe->lock, flags);
		list_move(&stat_probe->measure, &stat_measure->link);
		spin_unlock_irqrestore(&stat_probe->lock, flags);
		/* Program it */
		ret = xgold_noc_stat_configure_measure(noc_device, stat_probe);
		if (ret)
			dev_err(noc_device->dev,
				"Error when configuring stat measures!\n");
	}
}
