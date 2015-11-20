/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*#define DEBUG	1 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/interrupt.h>

#include "noc.h"
#include "noc_regs.h"

#define XGOLD632_NOC_ERROR_ID 0x011B0B00
#define XGOLD631_NOC_ERROR_ID 0x00FA1200
#define XGOLD726_NOC_ERROR_ID 0x010C0C00


/**
 * Tasklet called by interrupt when a measurement is finished.
 * @param data a pointer to the struct xgold_noc_probe which generated the
 * interrupt
 */
void xgold_noc_stat_measure_tasklet(unsigned long data)
{
	struct xgold_noc_probe *probe = (struct xgold_noc_probe *)data;
	xgold_noc_stat_measure(probe);
}

/**
 *
 * @param _dev
 * @param np
 * @param property_name
 * @return
 */
static const char **xgold_get_strings_from_dts(struct device *_dev,
					       struct device_node *np,
					       const char *property_name)
{
	struct property *prop;
	const char *s;
	const char **lut = NULL;
	int i = 0, lut_len;

	if (of_property_read_bool(np, property_name)) {
		lut_len = of_property_count_strings(np, property_name);

		if (IS_ERR_VALUE(lut_len)) {
			dev_err(_dev, "Invalid description of %s for %s\n",
				property_name, np->name);
			return ERR_PTR(lut_len);
		}

		lut = kcalloc(lut_len + 1, sizeof(char *), GFP_KERNEL);
		if (!lut)
			return ERR_PTR(-ENOMEM);

		of_property_for_each_string(np, property_name, prop, s)
		    lut[i++] = s;

		/* To find out the end of the list */
		lut[lut_len] = NULL;
	}

	return lut;
}

static struct xgold_bitfield *xgold_alloc_bitfield(void)
{
	struct xgold_bitfield *bitfield;

	bitfield = kzalloc(sizeof(*bitfield), GFP_KERNEL);
	if (!bitfield)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&bitfield->lock);
	INIT_LIST_HEAD(&bitfield->link);
	return bitfield;
}

static struct xgold_bitfield *xgold_new_bitfield(struct xgold_register *reg,
						 const char *name,
						 unsigned offset,
						 unsigned width,
						 const char **lut)
{
	struct xgold_bitfield *bf;
	unsigned long flags;

	if (!reg)
		return ERR_PTR(-EINVAL);

	bf = xgold_alloc_bitfield();
	if (!bf)
		return ERR_PTR(-ENOMEM);

	bf->parent = reg;
	bf->name = kstrdup(name, GFP_KERNEL);
	bf->offset = offset;
	bf->length = width;
	bf->mask = ((BIT(width) - 1) << offset);
	bf->lut = lut;

	spin_lock_irqsave(&reg->lock, flags);
	list_add_tail(&bf->link, &reg->bitfields);
	spin_unlock_irqrestore(&reg->lock, flags);

	return bf;
}

static void xgold_free_bitfield(struct xgold_bitfield *bf)
{
	kfree(bf->name);
	kfree(bf);
}

static struct xgold_bitfield *xgold_get_bitfield(struct device *_dev,
						 struct device_node *np)
{
	int ret, i;
	unsigned values[2];
	struct xgold_bitfield *bitfield;
	unsigned count;

	if (!np->name)
		return ERR_PTR(-EINVAL);

	bitfield = xgold_alloc_bitfield();
	if (!bitfield)
		return ERR_PTR(-ENOMEM);

	bitfield->name = np->name;

	ret = of_property_read_u32_array(np, "offset,length", values, 2);
	if (ret) {
		dev_err(_dev,
			"\"Offset,length\" properties of register %s\n missing",
			bitfield->name);
		ret = -EINVAL;
		goto free_bf;
	}

	bitfield->offset = values[0];
	bitfield->length = values[1];
	bitfield->mask = (((1UL << bitfield->length) - 1) << bitfield->offset);

	if ((bitfield->offset > 31) || (bitfield->length > 32)) {
		ret = -EINVAL;
		goto free_bf;
	}

	of_property_read_string(np, "description", &bitfield->description);

	bitfield->lut = xgold_get_strings_from_dts(_dev, np, "lut");
	if (IS_ERR(bitfield->lut)) {
		ret = PTR_ERR(bitfield->lut);
		dev_err(_dev, "Error %d while parsing lut of %s\n", ret,
			bitfield->name);
		bitfield->lut = NULL;
	}

	ret = of_property_read_u32(np, "aperture-size", &count);
	if (!ret) {
		unsigned *values_tab;
		bitfield->aperture_size = (u8) count;
		if (bitfield->aperture_size == 0) {
			ret = -EINVAL;
			dev_err(_dev,
				"Error while parsing aperture of %s bitfield. Size should not be 0!\n",
				bitfield->name);
			goto free_all;
		}
		bitfield->aperture_idx = kcalloc(bitfield->aperture_size,
						 sizeof(u16), GFP_KERNEL);
		if (!bitfield->aperture_idx) {
			ret = -ENOMEM;
			goto free_all;
		}
		bitfield->aperture_base = kcalloc(bitfield->aperture_size,
						  sizeof(unsigned), GFP_KERNEL);
		if (!bitfield->aperture_base) {
			kfree(bitfield->aperture_idx);
			ret = -ENOMEM;
			goto free_all;
		}
		values_tab = kcalloc(bitfield->aperture_size * 2,
				     sizeof(unsigned), GFP_KERNEL);
		if (!values_tab) {
			kfree(bitfield->aperture_idx);
			kfree(bitfield->aperture_base);
			ret = -ENOMEM;
			goto free_all;
		}
		ret = of_property_read_u32_array(np,
						 "aperture-idx,aperture-base",
						 values_tab,
						 bitfield->aperture_size * 2);
		if (ret) {
			ret = -EINVAL;
			dev_err(_dev,
				"Error while parsing aperture of %s bitfield. Table is missing!\n",
				bitfield->name);
			kfree(bitfield->aperture_idx);
			kfree(bitfield->aperture_base);
			kfree(values_tab);
			goto free_all;
		}
		for (i = 0; i < bitfield->aperture_size; i++) {
			bitfield->aperture_idx[i] = values_tab[2 * i];
			bitfield->aperture_base[i] = values_tab[2 * i + 1];
		}
		kfree(values_tab);
	}

	dev_dbg(_dev, "Bitfield %s, offset %x, length %d created\n",
		bitfield->name, bitfield->offset, bitfield->length);

	return bitfield;

free_all:
	kfree(bitfield->lut);
free_bf:
	kfree(bitfield);
	return ERR_PTR(ret);

}

static struct xgold_register *xgold_alloc_register(void)
{
	struct xgold_register *reg;

	reg = kzalloc(sizeof(*reg), GFP_KERNEL);
	if (!reg)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&reg->lock);
	spin_lock_init(&reg->hw_lock);
	INIT_LIST_HEAD(&reg->link);
	INIT_LIST_HEAD(&reg->bitfields);

	return reg;
}

static struct xgold_register *xgold_new_register(struct device *parent,
				 const char *name,
				 void __iomem *base,
				 unsigned offset,
				 unsigned width)
{
	struct xgold_register *reg;

	reg = xgold_alloc_register();
	if (!reg)
		return ERR_PTR(-ENOMEM);

	reg->parent = parent;
	reg->name = kstrdup(name, GFP_KERNEL);
	reg->base = base;
	reg->offset = offset;
	reg->length = width;

	return reg;
}

static void xgold_free_register(struct xgold_register *reg)
{
	struct xgold_bitfield *bf, *tmp;

	list_for_each_entry_safe(bf, tmp, &reg->bitfields, link)
		xgold_free_bitfield(bf);

	kfree(reg->name);
	kfree(reg);
}

/**
 * Create ErrorLog register from dts file
 * @param _dev
 * @param np
 * @return
 */
static struct xgold_register *xgold_get_register(struct device *_dev,
						 struct device_node *np)
{
	int ret;
	struct device_node *child;
	unsigned values[2];
	struct xgold_register *reg;

	if (!np->name)
		return ERR_PTR(-EINVAL);

	reg = xgold_alloc_register();
	if (!reg)
		return ERR_PTR(-ENOMEM);

	reg->name = np->name;
	reg->parent = _dev;
	reg->base = of_iomap(_dev->of_node, 0);

	ret = of_property_read_u32_array(np, "offset,length", values, 2);
	if (ret) {
		dev_err(_dev,
			"\"Offset,length\" properties of register %s\n missing",
			reg->name);
		ret = -EINVAL;
		goto free_reg;
	}

	reg->offset = values[0];
	reg->length = values[1];
	if (reg->length > 32) {
		ret = -EINVAL;
		goto free_reg;
	}

	/* Not mandatory properties  */
	of_property_read_string(np, "description", &reg->description);

	ret = of_property_read_u32(np, "aperture-link", values);
	if (!ret)
		reg->aperture_link = values[0];
	else
		reg->aperture_link = -1;

	/* Create bitfields */
	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, "intel,xgold,bitfield")) {
			struct xgold_bitfield *bitfield;
			bitfield = xgold_get_bitfield(_dev, child);
			if (!(IS_ERR(bitfield))) {
				list_add_tail(&bitfield->link, &reg->bitfields);
				bitfield->parent = reg;
			}
		}
	}
	dev_dbg(_dev, "Register %s, offset %x, length %d created\n", reg->name,
		reg->offset, reg->length);
	return reg;

free_reg:
	kfree(reg);
	return ERR_PTR(ret);
}

/**
 * Create ErrorLogx register set from DTS file
 * @param _dev
 * @param regs
 * @return
 */
static int xgold_get_registers(struct device *_dev, struct list_head *regs)
{
	struct device_node *parent = _dev->of_node;
	struct device_node *np;
	struct xgold_register *reg;

	for_each_child_of_node(parent, np) {
		if (of_device_is_compatible(np, "intel,xgold,register")) {
			reg = xgold_get_register(_dev, np);
			if (!(IS_ERR(reg)))
				list_add_tail(&reg->link, regs);
		}
	}

	return 0;
}

static const char *noc_cnt_src_evt_lut[] = {
	"off",
	"cycle",
	"idle",
	"xfer",
	"busy",
	"wait",
	"pkt",
	"lut",
	"byte",
	"press0",
	"disabled",
	"disabled",
	"filt0",
	"filt1",
	"filt2",
	"filt3",
	"chain",
	"lut_byte_en",
	"lut_byte",
	"filt_byte_en",
	"filt_byte",
	"disabled",
	"disabled",
	"disabled",
	"disabled",
	"disabled",
	"disabled",
	"disabled",
	"disabled",
	"disabled",
	"disabled",
	"disabled",
	NULL,
};

static const char *noc_cnt_alarm_mode_lut[] = {
	"off",
	"min",
	"max",
	"min_max",
	NULL
};

static const char *xgold_noc_enable_enum[] = {
	"disable",
	"enable",
	NULL,
};

static const char *xgold_noc_on_off_enum[] = {
	"off",
	"on",
	NULL
};

static struct of_device_id xgold_noc_of_match[] = {
	{
	 .compatible = "intel,l1noc",},
	{
	 .compatible = "intel,l2noc",},
	{
	 .compatible = "intel,modemnoc",},
	{
	 .compatible = "intel,audionoc",},
	{},
};

static int xgold_noc_get_error(struct xgold_noc_device *noc_device)
{
	void __iomem *base = noc_device->hw_base;
	struct xgold_noc_error *noc_err;
	unsigned long flags;

	struct device *mydev = noc_device->dev;

	noc_err = kzalloc(sizeof(struct xgold_noc_error), GFP_ATOMIC);
	if (!noc_err)
		return -ENOMEM;

	noc_err->timestamp = get_jiffies_64();

	noc_err->err[0] = ioread32(ERRLOG_0_ERRLOG0(base));
	noc_err->err[1] = ioread32(ERRLOG_0_ERRLOG1(base));
	noc_err->err[2] = ioread32(ERRLOG_0_ERRLOG3(base));
	noc_err->err[3] = ioread32(ERRLOG_0_ERRLOG5(base));
	noc_err->err[4] = ioread32(ERRLOG_0_ERRLOG7(base));

	spin_lock_irqsave(&noc_device->lock, flags);
	list_add_tail(&noc_err->link, &noc_device->err_queue);
	spin_unlock_irqrestore(&noc_device->lock, flags);

	dev_err_ratelimited(mydev,
		"Error interrupt"
		" -0 0x%08x -1 0x%08x -3 0x%08x -5 0x%08x -7 0x%08x\n",
		noc_err->err[0], noc_err->err[1],
		noc_err->err[2], noc_err->err[3], noc_err->err[4]);

	iowrite32(1, ERRLOG_0_ERRCLR(base));

	return 0;
}

static irqreturn_t xgold_noc_error_irq(int irq, void *dev)
{
	struct xgold_noc_device *noc_device = dev;
	void __iomem *base = noc_device->hw_base;

	while (ioread32(ERRLOG_0_ERRVLD(base)))
		xgold_noc_get_error(noc_device);

	BUG_ON(noc_device->trap_on_error == true);

	return IRQ_HANDLED;
}

/**
 * Copy ErrorLog registers into error_registers field + Enable error interrupt
 * @param _dev
 * @return
 */
static int xgold_noc_error_init(struct device *_dev)
{
	struct xgold_noc_device *noc_device = dev_get_drvdata(_dev);
	void __iomem *base = noc_device->hw_base;
	char reg_name[16];
	struct xgold_register *reg;
	int i, found = false;
	/* FIXME: Should come from the DTS */
	unsigned err_log_lut[] = {
		0,
		1,
		3,
		5,
		7
	};

	for (i = 0; i < XGOLD_NOC_ERROR_REGISTERS; i++) {
		scnprintf(reg_name, ARRAY_SIZE(reg_name), "ErrorLogger%d",
			  err_log_lut[i]);
		list_for_each_entry(reg, &noc_device->registers, link) {
			if (!strcmp(reg_name, reg->name)) {
				found = true;
				break;
			}
		}

		if (!found) {
			dev_err(_dev, "%s register not found !\n", reg_name);
			return -EINVAL;
		}
		noc_device->error_registers[i] = reg;
	}

	iowrite32(1, ERRLOG_0_FAULTEN(base));

	return 0;
}

static struct xgold_bitfield *xgold_noc_create_register_and_bitfield(
		struct xgold_noc_device *noc_device,
		const char *name,
		unsigned reg_offset,
		unsigned bf_offset,
		unsigned bf_width,
		const char **bf_lut)
{
	struct xgold_register *reg;

	if (noc_device == NULL)
		return ERR_PTR(-EINVAL);

	reg = xgold_new_register(noc_device->dev, name, noc_device->hw_base,
				 reg_offset, 32);
	if (IS_ERR_OR_NULL(reg))
		return ERR_PTR(-EINVAL);

	return xgold_new_bitfield(reg, name, bf_offset, bf_width, bf_lut);
}

static irqreturn_t xgold_noc_stat_irq(int irq, void *dev)
{
	int i;
	unsigned status;
	struct xgold_noc_device *noc_device = dev;
	void __iomem *base = noc_device->hw_base;
	unsigned probe_offset = noc_device->probe_offset;

	for (i = 0; i < noc_device->nr_probes; i++) {
		status = ioread32(PROBE_STATALARMSTATUS(base, probe_offset, i));
		if (status) {
			/* Disable alarm */
			iowrite32(0, PROBE_MAINCTL(base, probe_offset, i));
			/* Clear alarm */
			iowrite32(1, PROBE_STATALARMCLR(base, probe_offset, i));
			tasklet_schedule(&noc_device->probes[i].tasklet);
		}
	}

	return IRQ_HANDLED;
}

/**
 * Create a counter register set (includes PortSel, AlarmMode, Src, Val)
 * @param noc_device
 * @param cnt
 * @param probe
 * @param cnt_id
 * @return
 */
static int xgold_noc_init_counter(struct xgold_noc_device *noc_device,
				  struct xgold_noc_counter *cnt,
				  struct xgold_noc_probe *probe,
				  unsigned cnt_id)
{
	struct xgold_bitfield *bf;
	char _name[64];

	if ((noc_device == NULL) || (cnt == NULL) || (probe == NULL))
		return -EINVAL;

	cnt->id = cnt_id;
	cnt->parent = probe;

	/* Port Selection */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_counter%d_portsel",
		  probe->id, cnt_id);

	bf = xgold_noc_create_register_and_bitfield(noc_device, _name,
			    PROBE_COUNTERS_PORTSEL
			    (noc_device->probe_offset,
			     probe->id, cnt->id),
			    PROBE_COUNTERS_PORTSEL_COUNTERS_PORTSEL_OFFSET,
			    PROBE_COUNTERS_PORTSEL_COUNTERS_PORTSEL_WIDTH,
			    probe->available_portsel);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	cnt->portsel = bf;

	/* Alarm mode */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_counter%d_alarm_mode",
		  probe->id, cnt_id);

	bf = xgold_noc_create_register_and_bitfield(noc_device, _name,
			    PROBE_COUNTERS_ALARMMODE
			    (noc_device->probe_offset,
			     probe->id, cnt->id),
			    PROBE_COUNTERS_ALARMMODE_COUNTERS_ALARMMODE_OFFSET,
			    PROBE_COUNTERS_ALARMMODE_COUNTERS_ALARMMODE_WIDTH,
			    noc_cnt_alarm_mode_lut);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	cnt->alarm_mode = bf;

	/* Source Event */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_counter%d_source_event",
		  probe->id, cnt_id);

	bf = xgold_noc_create_register_and_bitfield(noc_device, _name,
				    PROBE_COUNTERS_SRC
				    (noc_device->probe_offset,
				     probe->id, cnt->id),
				    PROBE_COUNTERS_SRC_INTEVENT_OFFSET,
				    PROBE_COUNTERS_SRC_INTEVENT_WIDTH,
				    noc_cnt_src_evt_lut);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	cnt->source_event = bf;

	/* Value */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_counter%d_value",
		  probe->id, cnt_id);
	bf = xgold_noc_create_register_and_bitfield(noc_device, _name,
				    PROBE_COUNTERS_VAL
				    (noc_device->probe_offset,
				     probe->id, cnt->id),
				    PROBE_COUNTERS_VAL_COUNTERS_VAL_OFFSET,
				    PROBE_COUNTERS_VAL_COUNTERS_VAL_WIDTH,
				    NULL);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	cnt->value = bf;

	return 0;
}

static struct xgold_register *xgold_noc_get_compatible_register(struct
								xgold_noc_device
								*noc_device,
								const char
								*compatible)
{
	struct device *_dev = noc_device->dev;
	struct device_node *parent = _dev->of_node;
	struct device_node *np;
	struct xgold_register *reg;
	int ret;

	for_each_child_of_node(parent, np) {
		if (of_device_is_compatible(np, compatible)) {
			list_for_each_entry(reg, &noc_device->registers, link) {
				ret = strcmp(np->name, reg->name);
				if (ret == 0)
					return reg;
			}
		}
	}

	return NULL;
}

/**
 * Create a filter register set (includes RouteIdBase, RouteIdMask, AddrBase_Low,
 *    WindowSize, SecurityBase, SecurityMask, Opcode, Status, Length, Urgency)
 * @param noc_device
 * @param filter
 * @param probe
 * @param id
 * @return
 */
static int xgold_noc_init_filter(struct xgold_noc_device *noc_device,
				 struct xgold_noc_filter *filter,
				 struct xgold_noc_probe *probe, unsigned id)
{
	struct xgold_bitfield *bf, *bf_template;
	struct xgold_register *reg, *reg_template;
	char _name[64];
	unsigned route_idmask_width;
	unsigned security_mask_width;

	if ((noc_device == NULL) || (filter == NULL) || (probe == NULL))
		return -EINVAL;

	filter->id = id;
	filter->parent = probe;

	/* Route Id Base */
	reg_template = xgold_noc_get_compatible_register(noc_device,
							 "intel,xgold,noc,filter,routeid");
	if (!reg_template)
		return -ENODEV;

	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_filter%d_routeId",
		  probe->id, id);

	reg = xgold_new_register(noc_device->dev, _name, noc_device->hw_base,
				 PROBE_FILTERS_ROUTEIDBASE(noc_device->
							   probe_offset,
							   probe->id, id), 32);

	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;
	route_idmask_width = 0;
	list_for_each_entry(bf_template, &reg_template->bitfields, link) {

		bf = xgold_new_bitfield(reg, bf_template->name,
					bf_template->offset,
					bf_template->length, bf_template->lut);
		if (IS_ERR_OR_NULL(bf))
			return -EINVAL;
		if ((bf_template->offset + bf_template->length)
		    > route_idmask_width)
			route_idmask_width = (bf_template->offset
					      + bf_template->length);
	}
	filter->route_id = reg;

	/* Route Id Mask */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_filter%d_route_mask",
		  probe->id, id);
	bf = xgold_noc_create_register_and_bitfield(noc_device, _name,
						    PROBE_FILTERS_ROUTEIDMASK
						    (noc_device->probe_offset,
						     probe->id, id), 0,
						    route_idmask_width, NULL);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	filter->route_mask = bf;

	/* Address base low */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_filter%d_address_base",
		  probe->id, id);

	bf = xgold_noc_create_register_and_bitfield(noc_device, _name,
		    PROBE_FILTERS_ADDRBASE_LOW
		    (noc_device->probe_offset,
		     probe->id, id),
		    PROBE_FILTERS_ADDRBASE_LOW_FILTERS_ADDRBASE_LOW_OFFSET,
		    PROBE_FILTERS_ADDRBASE_LOW_FILTERS_ADDRBASE_LOW_WIDTH,
		    NULL);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	filter->addr_base = bf;

	/* Window Size */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_filter%d_window_size",
		  probe->id, id);

	bf = xgold_noc_create_register_and_bitfield(noc_device, _name,
			    PROBE_FILTERS_WINDOWSIZE
			    (noc_device->probe_offset,
			     probe->id, id),
			    PROBE_FILTERS_WINDOWSIZE_FILTERS_WINDOWSIZE_OFFSET,
			    PROBE_FILTERS_WINDOWSIZE_FILTERS_WINDOWSIZE_WIDTH,
			    NULL);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	filter->window_size = bf;

	/* Security Base */
	reg_template = xgold_noc_get_compatible_register(noc_device,
							 "intel,xgold,noc,filter,security");
	if (!reg_template)
		return -ENODEV;

	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_filter%d_security_base",
		  probe->id, id);

	reg = xgold_new_register(noc_device->dev, _name, noc_device->hw_base,
				 PROBE_FILTERS_SECURITYBASE(noc_device->
							    probe_offset,
							    probe->id, id), 32);

	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;

	filter->security_base = reg;
	security_mask_width = 0;
	list_for_each_entry(bf_template, &reg_template->bitfields, link) {
		bf = xgold_new_bitfield(reg, bf_template->name,
					bf_template->offset,
					bf_template->length, bf_template->lut);
		if (IS_ERR_OR_NULL(bf))
			return -EINVAL;
		if ((bf_template->offset + bf_template->length)
		    > security_mask_width)
			security_mask_width = (bf_template->offset
					       + bf_template->length);
	}

	/* Security Mask */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_filter%d_security_mask",
		  probe->id, id);

	bf = xgold_noc_create_register_and_bitfield(noc_device, _name,
						    PROBE_FILTERS_SECURITYMASK
						    (noc_device->probe_offset,
						     probe->id, id), 0,
						    security_mask_width, NULL);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	filter->security_mask = bf;

	/* Op Code */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_filter%d_op_code",
		  probe->id, id);

	reg = xgold_new_register(noc_device->dev, _name, noc_device->hw_base,
				 PROBE_FILTERS_OPCODE(noc_device->probe_offset,
						      probe->id, id), 32);

	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;

	bf = xgold_new_bitfield(reg, "opcode_rden",
				PROBE_FILTERS_OPCODE_RDEN_OFFSET,
				PROBE_FILTERS_OPCODE_RDEN_WIDTH,
				xgold_noc_enable_enum);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	bf = xgold_new_bitfield(reg, "opcode_wren",
				PROBE_FILTERS_OPCODE_WREN_OFFSET,
				PROBE_FILTERS_OPCODE_WREN_WIDTH,
				xgold_noc_enable_enum);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	bf = xgold_new_bitfield(reg, "opcode_locken",
				PROBE_FILTERS_OPCODE_LOCKEN_OFFSET,
				PROBE_FILTERS_OPCODE_LOCKEN_WIDTH,
				xgold_noc_enable_enum);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	bf = xgold_new_bitfield(reg, "opcode_urgen",
				PROBE_FILTERS_OPCODE_URGEN_OFFSET,
				PROBE_FILTERS_OPCODE_URGEN_WIDTH,
				xgold_noc_enable_enum);

	filter->op_code = reg;

	/* Status */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_filter%d_status",
		  probe->id, id);

	reg = xgold_new_register(noc_device->dev, _name, noc_device->hw_base,
				 PROBE_FILTERS_STATUS(noc_device->probe_offset,
						      probe->id, id), 32);

	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;

	bf = xgold_new_bitfield(reg, "status_request_enable",
				PROBE_FILTERS_STATUS_REQEN_OFFSET,
				PROBE_FILTERS_STATUS_REQEN_WIDTH,
				xgold_noc_enable_enum);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	bf = xgold_new_bitfield(reg, "status_response_enable",
				PROBE_FILTERS_STATUS_RSPEN_OFFSET,
				PROBE_FILTERS_STATUS_RSPEN_WIDTH,
				xgold_noc_enable_enum);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	filter->status = reg;
	/* Length */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_filter%d_length",
		  probe->id, id);

	bf = xgold_noc_create_register_and_bitfield(noc_device, _name,
				    PROBE_FILTERS_LENGTH
				    (noc_device->probe_offset,
				     probe->id, id),
				    PROBE_FILTERS_LENGTH_FILTERS_LENGTH_OFFSET,
				    PROBE_FILTERS_LENGTH_FILTERS_LENGTH_WIDTH,
				    NULL);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	filter->length = bf;

	/* Urgency */
	scnprintf(_name, ARRAY_SIZE(_name), "probe%d_filter%d_urgency",
		  probe->id, id);

	bf = xgold_noc_create_register_and_bitfield(noc_device, _name,
			    PROBE_FILTERS_URGENCY
			    (noc_device->probe_offset,
			     probe->id, id),
			    PROBE_FILTERS_URGENCY_FILTERS_URGENCY_OFFSET,
			    PROBE_FILTERS_URGENCY_FILTERS_URGENCY_WIDTH,
			    xgold_noc_enable_enum);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	filter->urgency = bf;

	return 0;
}

static void xgold_noc_remove_filter(struct xgold_noc_filter *filter)
{
	xgold_free_register(filter->urgency->parent);
	xgold_free_register(filter->length->parent);
	xgold_free_register(filter->status);
	xgold_free_register(filter->op_code);
	xgold_free_register(filter->security_mask->parent);
	xgold_free_register(filter->security_base);
	xgold_free_register(filter->window_size->parent);
	xgold_free_register(filter->addr_base->parent);
	xgold_free_register(filter->route_mask->parent);
	xgold_free_register(filter->route_id);
}

static void xgold_noc_remove_counter(struct xgold_noc_counter *cnt)
{
	xgold_free_register(cnt->portsel->parent);
	xgold_free_register(cnt->alarm_mode->parent);
	xgold_free_register(cnt->source_event->parent);
	xgold_free_register(cnt->value->parent);
}

static void xgold_noc_remove_probe(struct xgold_noc_probe *probe)
{
	unsigned i;
	struct xgold_noc_filter *filter;
	struct xgold_noc_counter *counter;

	/*TODO free registers */

	for (i = 0, filter = probe->filters; i < probe->parent->nr_filters;
	     i++, filter++)
		xgold_noc_remove_filter(filter);

	for (i = 0, counter = probe->counters; i < probe->parent->nr_counters;
	     i++, counter++)
		xgold_noc_remove_counter(counter);

	kfree(probe->filters);
	kfree(probe->counters);

}

/**
 * Create a probe register set (includes MainCtl, ConfigCtl, StatPeriod, StatAlarmMin, StatAlarmMax
 * @param noc_device
 * @param probe
 * @param cnt_id
 * @return
 */
static int xgold_noc_init_probe(struct xgold_noc_device *noc_device,
				struct xgold_noc_probe *probe, unsigned cnt_id)
{
	struct device *_dev = noc_device->dev;
	struct device_node *np = _dev->of_node;
	struct xgold_register *reg;
	struct xgold_bitfield *bf;
	char mystr[64];
	unsigned probe_offset = noc_device->probe_offset;

	scnprintf(mystr, ARRAY_SIZE(mystr), "probe,portsel,%d", cnt_id);
	probe->available_portsel = xgold_get_strings_from_dts(_dev, np, mystr);
	probe->parent = noc_device;
	probe->id = cnt_id;
	tasklet_init(&probe->tasklet, xgold_noc_stat_measure_tasklet,
		     (unsigned long)probe);

	/**
	 * Register: MainCtl
	 */
	scnprintf(mystr, ARRAY_SIZE(mystr), "probe%d_MainCtl", cnt_id);
	reg = xgold_new_register(noc_device->dev, mystr, noc_device->hw_base,
				 PROBE_MAINCTL(0, probe_offset, cnt_id), 32);

	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;

	probe->main_ctl = reg;

	bf = xgold_new_bitfield(reg, "StatCondDump",
				PROBE_MAINCTL_STATCONDDUMP_OFFSET,
				PROBE_MAINCTL_STATCONDDUMP_WIDTH,
				xgold_noc_on_off_enum);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	bf = xgold_new_bitfield(reg, "Stat_Enable",
				PROBE_MAINCTL_STATEN_OFFSET,
				PROBE_MAINCTL_STATEN_WIDTH,
				xgold_noc_enable_enum);

	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	bf = xgold_new_bitfield(reg, "Alarm_Enable",
				PROBE_MAINCTL_ALARMEN_OFFSET,
				PROBE_MAINCTL_ALARMEN_WIDTH,
				xgold_noc_enable_enum);
	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	/**
	 * Register: ConfigCtl
	 */
	scnprintf(mystr, ARRAY_SIZE(mystr), "probe%d_ConfigCtl", cnt_id);
	reg = xgold_new_register(noc_device->dev, mystr, noc_device->hw_base,
				 PROBE_CFGCTL(0, probe_offset, cnt_id), 32);
	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;

	probe->cfg_ctl = reg;

	bf = xgold_new_bitfield(reg, "Global_Enable",
				PROBE_CFGCTL_GLOBALEN_OFFSET,
				PROBE_CFGCTL_GLOBALEN_WIDTH,
				xgold_noc_enable_enum);
	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	bf = xgold_new_bitfield(reg, "Active",
				PROBE_CFGCTL_GLOBALEN_OFFSET,
				PROBE_CFGCTL_GLOBALEN_WIDTH,
				xgold_noc_on_off_enum);
	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	/**
	 * Register: TracePortSel
	 */
	scnprintf(mystr, ARRAY_SIZE(mystr), "probe%d_TracePortSel", cnt_id);
	reg = xgold_new_register(noc_device->dev, mystr, noc_device->hw_base,
				 PROBE_TRACEPORTSEL(0, probe_offset, cnt_id),
				 32);
	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;

	probe->trace_port_sel = reg;

	bf = xgold_new_bitfield(reg, "TracePortSel",
				PROBE_TRACEPORTSEL_TRACEPORTSEL_OFFSET,
				PROBE_TRACEPORTSEL_TRACEPORTSEL_WIDTH,
				probe->available_portsel);
	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	/**
	 * Register: FilterLUT
	 */
	scnprintf(mystr, ARRAY_SIZE(mystr), "probe%d_FilterLUT", cnt_id);
	reg = xgold_new_register(noc_device->dev, mystr, noc_device->hw_base,
				 PROBE_FILTERLUT(0, probe_offset, cnt_id), 32);
	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;

	probe->filter_lut = reg;

	bf = xgold_new_bitfield(reg, "FilterLUT",
				PROBE_FILTERLUT_FILTERLUT_OFFSET,
				PROBE_FILTERLUT_FILTERLUT_WIDTH, NULL);
	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	/**
	 * Register: StatPeriod
	 */
	scnprintf(mystr, ARRAY_SIZE(mystr), "probe%d_StatPeriod", cnt_id);
	reg = xgold_new_register(noc_device->dev, mystr, noc_device->hw_base,
				 PROBE_STATPERIOD(0, probe_offset, cnt_id), 32);
	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;

	probe->stat_period = reg;

	bf = xgold_new_bitfield(reg, "StatPeriod",
				PROBE_STATPERIOD_STATPERIOD_OFFSET,
				PROBE_STATPERIOD_STATPERIOD_WIDTH, NULL);
	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	/**
	 * Register: StatAlarmMax
	 */
	scnprintf(mystr, ARRAY_SIZE(mystr), "probe%d_StatAlarmMax", cnt_id);
	reg = xgold_new_register(noc_device->dev, mystr, noc_device->hw_base,
				 PROBE_STATALARMMAX(0, probe_offset, cnt_id),
				 32);

	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;

	probe->stat_alarm_max = reg;

	bf = xgold_new_bitfield(reg, "StatAlarmMax",
				PROBE_STATALARMMAX_STATALARMMAX_OFFSET,
				PROBE_STATALARMMAX_STATALARMMAX_WIDTH, NULL);

	/**
	 * Register: StatAlarmMin
	 */
	scnprintf(mystr, ARRAY_SIZE(mystr), "probe%d_StatAlarmMin", cnt_id);
	reg = xgold_new_register(noc_device->dev, mystr, noc_device->hw_base,
				 PROBE_STATALARMMIN(0, probe_offset, cnt_id),
				 32);

	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;

	probe->stat_alarm_min = reg;

	bf = xgold_new_bitfield(reg, "StatAlarmMin",
				PROBE_STATALARMMIN_STATALARMMIN_OFFSET,
				PROBE_STATALARMMIN_STATALARMMIN_WIDTH, NULL);
	if (IS_ERR_OR_NULL(bf))
		return -EINVAL;

	return 0;
}



static int xgold_noc_parse_dts_qoscfg(struct device *_dev,
				char *propname,
				struct regcfg **config)
{
	int def_len;
	struct property *prop;
	struct device_node *np = _dev->of_node;

	prop = of_find_property(np, propname, &def_len);
	if (prop != NULL) {
		unsigned i;
		struct regcfg *reg;

		*config = (struct regcfg *)
			devm_kzalloc(_dev,
					sizeof(struct regcfg), GFP_KERNEL);
		if (*config == NULL) {
			dev_err(_dev, "%s: Memory allocation failed!",
					__func__);
			BUG();
		}
		INIT_LIST_HEAD(&(*config)->list);

		for (i = 0; i < (def_len / sizeof(unsigned)); i += 2) {
			reg = (struct regcfg *)
				devm_kzalloc(_dev,
				sizeof(struct regcfg), GFP_KERNEL);
			if (reg == NULL) {
				dev_err(_dev, "%s: Memory allocation failed!",
						__func__);
				BUG();
			}
			of_property_read_u32_index(np, propname, i,
					&reg->offset);
			of_property_read_u32_index(np, propname, i+1,
					&reg->value);

			list_add_tail(&reg->list,
					&(*config)->list);
		}
	} else
		*config = NULL;

	return *config == NULL ? -EINVAL : 0;
}

static int xgold_noc_parse_dts_qoslist(struct device *_dev)
{
	int def_len;
	struct property *prop;
	struct device_node *np = _dev->of_node;
	struct xgold_noc_device *noc_device = dev_get_drvdata(_dev);

	prop = of_find_property(np, "intel,qos-configs", &def_len);
	if (prop != NULL) {
		char str[64] = "\0";
		struct dev_qos_cfg *devqos;
		int i, ncfg = of_property_count_strings(np,
				"intel,qos-configs");

		noc_device->qos = (struct dev_qos_cfg *)
			devm_kzalloc(_dev,
					sizeof(struct dev_qos_cfg), GFP_KERNEL);
		if (noc_device->qos == NULL) {
			dev_err(_dev, "%s: Memory allocation failed!",
					__func__);
			BUG();
		}
		INIT_LIST_HEAD(&noc_device->qos->list);
		for (i = 0; i < ncfg; i++) {
			int ret = 0;
			devqos = (struct dev_qos_cfg *)
				devm_kzalloc(_dev,
					sizeof(struct dev_qos_cfg), GFP_KERNEL);
			if (devqos == NULL) {
				dev_err(_dev, "%s: Memory allocation failed!",
						__func__);
				BUG();
			}
			of_property_read_string_index(np,
					"intel,qos-configs", i,
					&devqos->name);

			/* test if controlled by noc driver */
			snprintf(str, sizeof(str),
				 "intel,%s-qos-owner", devqos->name);
			if (of_property_read_bool(np, str))
				devqos->noc_owner = 1;
			else
				devqos->noc_owner = 0;

			snprintf(str, sizeof(str),
				 "intel,%s-qos-settings", devqos->name);
			ret = xgold_noc_parse_dts_qoscfg(_dev, str,
					&devqos->config);
			if (ret) {
				dev_info(_dev, "%s not add to list\n", str);
				devm_kfree(_dev, devqos);
				return -EINVAL;
			} else {
				dev_info(_dev, "%s added to list(%d)\n",
						str, devqos->noc_owner);
				list_add_tail(&devqos->list,
					&noc_device->qos->list);
			}
		}
	} else {
		dev_info(_dev, "no QoS config list\n");
		noc_device->qos = NULL;
	}
	return 0;
}
static struct xgold_noc_device *noc_dev_glob[2] = {NULL, NULL};
static int noc_idev_glob;
void xgold_noc_qos_set(const char *name)
{
	struct dev_qos_cfg *qos;
	struct regcfg *reg;
	int idev = 0;
	for (idev = 0; idev < 2; idev++) {
		struct xgold_noc_device *noc_dev =
					noc_dev_glob[idev];

		if ((!noc_dev) || (!noc_dev->qos))
			return;

		list_for_each_entry(qos, &noc_dev->qos->list, list) {

			if (strcmp(qos->name, name) == 0) {
				if (qos->config) {
					pr_debug("Set QoS config %s\n",
							qos->name);
					list_for_each_entry(reg,
							&qos->config->list,
							list) {
						iowrite32(reg->value,
							noc_dev->hw_base +
							reg->offset);
					}
				}
				return;
			}
		}
	}
	pr_debug("QoS config %s not found\n", name);
}
EXPORT_SYMBOL(xgold_noc_qos_set);

static void noc_device_qos_set(struct xgold_noc_device *noc_device)
{
	struct dev_qos_cfg *qos;
	if (!noc_device) {
		pr_err("%s NULL noc device\n", __func__);
		return;
	}
	if (!noc_device->qos)
		return;

	list_for_each_entry(qos, &noc_device->qos->list, list) {
		if (qos->noc_owner) {
			pr_debug("noc: Set default QoS config %s\n",
					qos->name);
			xgold_noc_qos_set(qos->name);
		}
	}
}

/**
 * Parse dts to get number of probes and number of filters and counters per probe
 * @param _dev
 * @return
 */
static int xgold_noc_parse_dts(struct device *_dev)
{
	struct device_node *np = _dev->of_node;
	struct xgold_noc_device *noc_device = dev_get_drvdata(_dev);
	int ret;

	ret = of_property_read_u32(np, "probe,nr", &noc_device->nr_probes);
	if (ret) {
		dev_err(_dev, "\"probe,nr\" property missing");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "probe,offset",
				   &noc_device->probe_offset);
	if (ret) {
		dev_err(_dev, "\"probe,offset\" property missing");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "counter,nr", &noc_device->nr_counters);
	if (ret) {
		dev_err(_dev, "\"counter,nr\" property missing");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "filter,nr", &noc_device->nr_filters);
	if (ret) {
		dev_err(_dev, "\"filter,nr\" property missing");
		return -EINVAL;
	}

	noc_device->trap_on_error = of_property_read_bool(np, "errors,trap");

	dev_info(_dev, "%d probe(s), %d filter(s), %d counter(s), error policy: %s\n",
		 noc_device->nr_probes, noc_device->nr_filters,
		 noc_device->nr_counters,
		 noc_device->trap_on_error ? "trap" : "print");


	xgold_noc_parse_dts_qoslist(_dev);
	noc_dev_glob[noc_idev_glob] = noc_device;
	noc_device_qos_set(noc_device);
	noc_idev_glob++;


	return 0;
}

/**
 * Create probes, filters and counters registers
 * @param _dev
 * @return
 */
static int xgold_noc_init_device(struct device *_dev)
{
	struct xgold_noc_device *noc_device = dev_get_drvdata(_dev);
	struct xgold_noc_probe *probes;
	struct xgold_noc_filter *filters;
	struct xgold_noc_counter *counters;
	int ret, i, j;

	probes = kcalloc(noc_device->nr_probes, sizeof(struct xgold_noc_probe),
			 GFP_KERNEL);
	if (!probes)
		return -ENOMEM;

	noc_device->probes = probes;

	for (i = 0; i < noc_device->nr_probes; i++) {
		/* Initialise probes */
		struct xgold_noc_probe *probe = &noc_device->probes[i];

		ret = xgold_noc_init_probe(noc_device, probe, i);
		if (ret)
			return -EINVAL;

		/* Initialise filters */
		filters = kcalloc(noc_device->nr_filters,
				  sizeof(struct xgold_noc_filter), GFP_KERNEL);
		if (!filters)
			return -ENOMEM;	/* FIXME: Free allocated structs */

		probe->filters = filters;
		for (j = 0; j < noc_device->nr_filters; j++) {
			ret = xgold_noc_init_filter(noc_device, &filters[j],
						    probe, j);
			if (ret)
				return ret;
		}

		/* initialize counters */
		counters = kcalloc(noc_device->nr_counters,
				   sizeof(struct xgold_noc_counter),
				   GFP_KERNEL);
		if (!counters)
			return -ENOMEM;	/* FIXME: Free allocated structs */

		probe->counters = counters;
		for (j = 0; j < noc_device->nr_counters; j++) {
			ret = xgold_noc_init_counter(noc_device, &counters[j],
						     probe, j);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int xgold_noc_probe(struct platform_device *pdev)
{
	struct device *mydev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct xgold_noc_device *noc_device;
	int ret;

	dev_info(mydev, "XGOLD NoC driver initialization %s\n",
		 dev_name(mydev));
	noc_device = kzalloc(sizeof(struct xgold_noc_device), GFP_KERNEL);
	if (!noc_device)
		return -ENOMEM;

	noc_device->dev = mydev;
	dev_set_drvdata(mydev, noc_device);

	noc_device->hw_base = of_iomap(np, 0);
	if (noc_device->hw_base == NULL) {
		dev_err(mydev, "Error while remapping\n");
		ret = -EINVAL;
		goto free_noc_device;
	}

	noc_device->clock = of_clk_get(np, 0);
	if (IS_ERR_OR_NULL(noc_device->clock)) {
		dev_dbg(mydev, "Error %d while getting clock\n",
			(int)noc_device->clock);
	}

	if (!IS_ERR_OR_NULL(noc_device->clock)) {
		ret = clk_prepare_enable(noc_device->clock);
		if (ret) {
			dev_err(mydev, "Error %d while enabling clock\n",
				(int)noc_device->clock);

		}
	}
	if (!IS_ERR_OR_NULL(noc_device->clock))
		dev_dbg(mydev, "Clock rate %lu\n",
				clk_get_rate(noc_device->clock));

	noc_device->err_irq = platform_get_irq_byname(pdev, "error");
	if (IS_ERR_VALUE(noc_device->err_irq)) {
		dev_warn(mydev, "error irq not defined\n");

		ret = noc_device->err_irq;
		goto put_clock;
	} else {
		ret = request_irq(noc_device->err_irq, xgold_noc_error_irq,
				IRQF_SHARED, "noc error", noc_device);
		if (ret) {
			dev_err(mydev, "Error %d while installing error irq\n",
					(int)noc_device->err_irq);

			goto put_clock;
		}
	}

	noc_device->stat_irq = platform_get_irq_byname(pdev, "stat_alarm");
	if (IS_ERR_VALUE(noc_device->stat_irq)) {
		dev_err(mydev, "Error %d while extracting stat alarm irq\n",
			(int)noc_device->stat_irq);

		ret = noc_device->stat_irq;
		goto free_err_irq;
	}
	ret = request_irq(noc_device->stat_irq, xgold_noc_stat_irq,
			  IRQF_SHARED, "noc stat", noc_device);
	if (ret) {
		dev_err(mydev, "Error %d while installing error irq\n",
			(int)noc_device->stat_irq);

		goto free_err_irq;
	}

	INIT_LIST_HEAD(&noc_device->err_queue);
	INIT_LIST_HEAD(&noc_device->registers);
	spin_lock_init(&noc_device->lock);

	ret = xgold_get_registers(mydev, &noc_device->registers);
	if (ret) {
		dev_err(mydev, "Error while gettings registers\n");
		goto free_stat_irq;
	}

	/* Get these parameters from the dts */

	ret = xgold_noc_parse_dts(mydev);
	if (ret) {
		dev_err(mydev, "Parsing Dts failed\n");
		goto free_stat_irq;
	}

	ret = xgold_noc_init_device(mydev);
	if (ret)
		dev_err(mydev, "Failed to init internal NoC modelling\n");

	ret = xgold_noc_error_init(mydev);
	if (ret) {
		dev_err(mydev, "Initialisation of error handling failed\n");
		goto free_stat_irq;
	}

	ret = xgold_noc_stat_init(mydev);
	if (ret)
		dev_err(mydev, "No NOC sniffer or NOC sniffer init failure\n");
	else {
		ret = xgold_noc_stat_debugfs_init(mydev);
		if (ret)
			dev_err(mydev,
				"Initialisation of NOC sniffer sysfs failed\n");
	}

	xgold_noc_debug_init(mydev);

	return 0;
free_stat_irq:
	free_irq(noc_device->stat_irq, noc_device);
free_err_irq:
	free_irq(noc_device->err_irq, noc_device);
put_clock:
	if (!IS_ERR_OR_NULL(noc_device->clock))
		clk_put(noc_device->clock);
	iounmap(noc_device->hw_base);
free_noc_device:
	kfree(noc_device);
	return ret;
}

static int xgold_noc_remove(struct platform_device *pdev)
{
	struct device *mydev = &pdev->dev;
	struct xgold_noc_device *noc_device = dev_get_drvdata(mydev);
	struct xgold_noc_error *noc_err, *tmp_err;
	struct xgold_register *reg, *tmp_reg;
	struct xgold_noc_probe *probe;
	struct xgold_noc_stat_measure *stat_measure, *tmp_stat_measure;
	struct xgold_noc_stat_probe *stat_probe, *tmp_stat_probe;
	int i;

	dev_info(mydev, "Removing %s\n", dev_name(mydev));

	list_for_each_entry_safe(stat_probe, tmp_stat_probe,
				 &noc_device->stat->probe, link) {
		list_for_each_entry_safe(stat_measure, tmp_stat_measure,
					 &stat_probe->measure, link) {
			list_del(&stat_measure->link);
			kfree(stat_measure);
		}
		list_del(&stat_probe->link);
		kfree(stat_probe);
	}

	list_for_each_entry_safe(noc_err, tmp_err,
				&noc_device->err_queue, link) {
		list_del(&noc_err->link);
		kfree(noc_err);
	}

	list_for_each_entry_safe(reg, tmp_reg, &noc_device->registers, link) {
		list_del(&reg->link);
		xgold_free_register(reg);
	}

	for (i = 0, probe = noc_device->probes; i < noc_device->nr_probes;
	     i++, probe++)
		xgold_noc_remove_probe(probe);

	kfree(noc_device->probes);

	kfree(noc_device->stat);

	/* TODO: Free resources */
	if (!IS_ERR_OR_NULL(noc_device->clock))
		clk_put(noc_device->clock);
	iounmap(noc_device->hw_base);
	kfree(noc_device);
	return 0;
}

static int noc_resume(struct device *dev)
{
	struct xgold_noc_device *noc_device = dev_get_drvdata(dev);
	void __iomem *base = noc_device->hw_base;

	noc_device_qos_set(noc_device);

	while (ioread32(ERRLOG_0_ERRVLD(base))) {
		dev_err(dev, "NoC Error pending during resume\n");
		xgold_noc_get_error(noc_device);
	}

	iowrite32(1, ERRLOG_0_FAULTEN(base));

	return 0;
}

static int noc_suspend(struct device *dev)
{
	struct xgold_noc_device *noc_device = dev_get_drvdata(dev);
	void __iomem *base = noc_device->hw_base;

	iowrite32(0, ERRLOG_0_FAULTEN(base));

	while (ioread32(ERRLOG_0_ERRVLD(base))) {
		dev_err(dev, "NoC Error pending during suspend\n");
		xgold_noc_get_error(noc_device);
	}

	return 0;
}

static const struct dev_pm_ops noc_driver_pm_ops = {
#if CONFIG_PM
	.suspend = noc_suspend,
	.resume = noc_resume,
#endif
};

static struct platform_driver xgold_noc_driver = {
	.probe = xgold_noc_probe,
	.remove = xgold_noc_remove,
	.driver = {
		   .name = "xgold-noc",
		   .owner = THIS_MODULE,
		   .pm = &noc_driver_pm_ops,
		   .of_match_table = of_match_ptr(xgold_noc_of_match),},
};

static int __init xgold_noc_init(void)
{
	pr_info("XGOLD Network On Chip (NoC) initialisation");

	return platform_driver_register(&xgold_noc_driver);

}

static void __exit xgold_noc_exit(void)
{
	platform_driver_unregister(&xgold_noc_driver);
}

subsys_initcall(xgold_noc_init);
module_exit(xgold_noc_exit);

MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_VERSION("1.1.0");
MODULE_LICENSE("GPL");
