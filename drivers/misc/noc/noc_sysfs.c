/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/interrupt.h>

#include "noc.h"

#define MAX_ENUM_SIZE 254
#define XGOLDREG_ADDR(_reg) ((void __iomem *) (reg->base + reg->offset))

static int xgold_bitfield_debug_print(struct seq_file *m,
				      struct xgold_bitfield *bitfield,
				      unsigned value,
				      unsigned value_init_target_subrange,
				      const char *prefix)
{
	if (value_init_target_subrange != -1) {
		/* Special case for register 3 with aperture table */
		u16 idx = (value_init_target_subrange & bitfield->mask)
		    >> bitfield->offset;
		u16 i;
		for (i = 0; i < bitfield->aperture_size; i++) {
			if (bitfield->aperture_idx[i] == idx)
				break;
		}
		if (i != bitfield->aperture_size)
			value |= bitfield->aperture_base[i];
		return seq_printf(m, "%s%s: 0x%x\n", prefix, bitfield->name,
				  value);

	} else {
		unsigned bf_value = (value & bitfield->mask)
		    >> bitfield->offset;

		if (bitfield->lut)
			return seq_printf(m, "%s%s: %s\n", prefix,
					  bitfield->name,
					  bitfield->lut[bf_value]);
		else
			return seq_printf(m, "%s%s: %x\n", prefix,
					  bitfield->name, bf_value);
	}

}

static int xgold_register_debug_print(struct seq_file *m,
				      struct xgold_register *reg,
				      unsigned value,
				      unsigned value_init_target_subrange,
				      const char *prefix)
{
	struct xgold_bitfield *bf;
	char myprefix[64];

	scnprintf(myprefix, ARRAY_SIZE(myprefix), "\t%s", prefix);
	seq_printf(m, "%s%s: %#x\n", prefix, reg->name, value);

	list_for_each_entry(bf, &reg->bitfields, link)
	    xgold_bitfield_debug_print(m, bf, value,
				       value_init_target_subrange, myprefix);

	return 0;
}

int noc_debug_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

static int noc_error_dump(struct seq_file *m,
			  struct xgold_noc_device *noc_device,
			  struct xgold_noc_error *noc_err)
{
	unsigned i, ret;
	unsigned err_init_target_subrange;

	ret = seq_printf(m, "timestamp: %lld:\n", noc_err->timestamp);
	if (ret)
		return ret;

	/* This below value must be passed to be able to calculate
	 * the real address from the aperture table */
	/* FIXME: hardcoded!!! */
	err_init_target_subrange = noc_err->err[1];
	for (i = 0; i < XGOLD_NOC_ERROR_REGISTERS; i++) {
		struct xgold_register *reg = noc_device->error_registers[i];
		unsigned err = noc_err->err[i];
		if ((reg->aperture_link != -1)
		    && (reg->aperture_link > 0
			&& reg->aperture_link < XGOLD_NOC_ERROR_REGISTERS))
			err_init_target_subrange =
			    noc_err->err[reg->aperture_link];
		else
			err_init_target_subrange = -1;
		xgold_register_debug_print(m, reg, err,
					   err_init_target_subrange, "\t");
	}

	return 0;
}

static const char *xgold_bitfield_get_enum_from_value(struct xgold_bitfield *bf,
						      unsigned value)
{
	const char **lut = bf->lut;
	const char *mystr;
	unsigned i;

	if (lut == NULL)
		return ERR_PTR(-EINVAL);

	for (mystr = lut[0], i = 0; mystr; mystr = lut[++i])
		;

	if (value > i)
		return ERR_PTR(-EINVAL);

	return lut[value];
}

static int xgold_bitfield_read(struct xgold_bitfield *bf, unsigned *value)
{
	struct xgold_register *reg = bf->parent;

	if (reg == NULL)
		return -EINVAL;

	if (reg->base == NULL)
		return -EINVAL;

	if (value == NULL)
		return -EINVAL;

	*value = ioread32(XGOLDREG_ADDR(reg));
	*value &= bf->mask;
	*value >>= bf->offset;

	dev_dbg(reg->parent,
		"Read  Register 0x%08x @%p (%s), bitfield (%s) "
		"val=0x%x, %d bit at offset %d, mask %x\n",
		ioread32(XGOLDREG_ADDR(reg)), XGOLDREG_ADDR(reg),
		reg->name, bf->name, *value, bf->length, bf->offset, bf->mask);

	return 0;
}

static const char *xgold_bitfield_read_enum(struct xgold_bitfield *bf)
{
	int ret;
	unsigned value;

	ret = xgold_bitfield_read(bf, &value);
	if (ret)
		return NULL;

	return xgold_bitfield_get_enum_from_value(bf, value);
}

static int xgold_bitfield_get_value_from_enum(struct xgold_bitfield *bf,
					      const char *str_value,
					      unsigned *value)
{
	const char **lut = bf->lut;
	const char *mystr;
	unsigned i;

	if (lut == NULL) {
		if (sscanf(str_value, "%x", value) == 1)
			return 0;

		return -EINVAL;
	}

	for (mystr = lut[0], i = 0; mystr; mystr = lut[++i]) {
		if (!(strcmp(str_value, mystr))) {
			*value = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int xgold_bitfield_write(struct xgold_bitfield *bf, unsigned value)
{
	unsigned long flags;
	unsigned reg_value;
	struct xgold_register *reg = bf->parent;

	if (reg == NULL)
		return -EINVAL;

	if (reg->base == NULL)
		return -EINVAL;

	/*if (value >= (BIT(bf->length)-1)) */
	/*      return -EINVAL; */

	spin_lock_irqsave(&reg->hw_lock, flags);
	reg_value = ioread32(XGOLDREG_ADDR(reg));
	reg_value &= ~bf->mask;
	reg_value |= (value << bf->offset);
	iowrite32(reg_value, XGOLDREG_ADDR(reg));
	spin_unlock_irqrestore(&reg->hw_lock, flags);

	dev_dbg(reg->parent,
		"Write Register 0x%08x @%p (%s), bitfield (%s) val=0x%x"
		", %d bit at offset %d, mask %x\n",
		reg_value, XGOLDREG_ADDR(reg), reg->name, bf->name,
		value, bf->length, bf->offset, bf->mask);
	return 0;
}

static int xgold_bitfield_write_enum(struct xgold_bitfield *bf, const char *str)
{
	int value, ret;

	ret = xgold_bitfield_get_value_from_enum(bf, str, &value);
	if (ret)
		return ret;

	ret = xgold_bitfield_write(bf, value);

	return ret;
}

static int noc_debug_available_show(struct seq_file *m, void *p)
{
	const char **to_show = m->private;
	const char *str;
	int i = 0;
	for (str = to_show[0]; str; str = to_show[++i])
		seq_printf(m, "%s ", str);

	seq_puts(m, "\r\n");
	return 0;
}

static int noc_debug_available_open(struct inode *inode, struct file *file)
{
	return single_open(file, noc_debug_available_show, inode->i_private);
}

static ssize_t noc_debug_bf_write(struct file *filp, const char __user *ubuf,
				  size_t cnt, loff_t *ppos)
{
	struct xgold_bitfield *bf = filp->private_data;
	char buf[MAX_ENUM_SIZE + 1];
	int i;
	size_t ret;
	int err;

	ret = cnt;

	if (cnt > MAX_ENUM_SIZE)
		cnt = MAX_ENUM_SIZE;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;

	buf[cnt] = 0;

	/* strip ending whitespace. */
	for (i = cnt - 1; i > 0 && isspace(buf[i]); i--)
		buf[i] = 0;

	if (bf->lut) {
		err = xgold_bitfield_write_enum(bf, buf);
		if (err)
			return err;
	} else {
		unsigned value = 0;
		err = sscanf(buf, "%x\n", &value);
		/*if (err) */
		/*      return -ENODEV; */
		err = xgold_bitfield_write(bf, value);
		if (err)
			return -ENODEV;
	}
	*ppos += ret;

	return ret;
}

static ssize_t noc_debug_bf_read(struct file *filp, char __user *ubuf,
				 size_t cnt, loff_t *ppos)
{
	struct xgold_bitfield *bf = filp->private_data;
	char buf[MAX_ENUM_SIZE + 2];
	int r;
	const char *str;
	unsigned value;

	if (bf == NULL)
		return -ENODEV;

	if (bf->lut) {
		str = xgold_bitfield_read_enum(bf);
		if (IS_ERR_OR_NULL(str))
			str = "UNKNOWN BITFIELD VALUE";
		r = snprintf(buf, sizeof(buf), "%s\n", str);
	} else {
		r = xgold_bitfield_read(bf, &value);
		if (r)
			return -ENODEV;

		r = sprintf(buf, "%#x\n", value);
	}

	return simple_read_from_buffer(ubuf, cnt, ppos, buf, r);
}

static int noc_debug_error_show(struct seq_file *m, void *p)
{
	struct xgold_noc_device *noc_device = m->private;
	struct xgold_noc_error *noc_err;
	unsigned long flags;

	spin_lock_irqsave(&noc_device->lock, flags);
	list_for_each_entry(noc_err, &noc_device->err_queue, link) {
		spin_unlock_irqrestore(&noc_device->lock, flags);
		noc_error_dump(m, noc_device, noc_err);
		spin_lock_irqsave(&noc_device->lock, flags);
	}
	spin_unlock_irqrestore(&noc_device->lock, flags);
	return 0;
}

static int noc_debug_error_open(struct inode *inode, struct file *file)
{
	return single_open(file, noc_debug_error_show, inode->i_private);
}

static const struct file_operations noc_debug_error_fops = {
	.open = noc_debug_error_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations noc_debug_fops = {
	.open = noc_debug_open,
	.read = noc_debug_bf_read,
	.write = noc_debug_bf_write,
	.llseek = generic_file_llseek,
};

static const struct file_operations noc_debug_available_fops = {
	.open = noc_debug_available_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/**
 * Create sysfs
 * @param _dev
 * @return
 */
int xgold_noc_debug_init(struct device *_dev)
{
	struct xgold_noc_device *noc_device = dev_get_drvdata(_dev);
	struct device_node *np = _dev->of_node;
	struct dentry *dir;
	char name[64];
	int i, j;

	scnprintf(name, ARRAY_SIZE(name), "%s_debug", np->name);

	dir = debugfs_create_dir(name, NULL);
	if (IS_ERR(dir))
		return PTR_ERR(dir);

	noc_device->dir = dir;

	debugfs_create_file("errors", S_IRUGO, dir, noc_device,
			    &noc_debug_error_fops);

	for (i = 0; i < noc_device->nr_probes; i++) {
		struct dentry *probe_dir;
		struct xgold_register *reg;
		struct xgold_bitfield *bf;
		struct xgold_noc_probe *probe = &noc_device->probes[i];
		scnprintf(name, ARRAY_SIZE(name), "probe%d", i);
		probe_dir = debugfs_create_dir(name, noc_device->dir);
		if (IS_ERR(probe_dir))
			return PTR_ERR(probe_dir);

		reg = probe->main_ctl;
		list_for_each_entry(bf, &reg->bitfields, link) {
			debugfs_create_file(bf->name,
					    S_IRUGO | S_IWUSR, probe_dir, bf,
					    &noc_debug_fops);
		}

		reg = probe->cfg_ctl;
		list_for_each_entry(bf, &reg->bitfields, link) {
			debugfs_create_file(bf->name,
					    S_IRUGO | S_IWUSR, probe_dir, bf,
					    &noc_debug_fops);
		}

		reg = probe->trace_port_sel;
		list_for_each_entry(bf, &reg->bitfields, link) {
			debugfs_create_file(bf->name,
					    S_IRUGO | S_IWUSR, probe_dir, bf,
					    &noc_debug_fops);
		}

		reg = probe->filter_lut;
		list_for_each_entry(bf, &reg->bitfields, link) {
			debugfs_create_file(bf->name,
					    S_IRUGO | S_IWUSR, probe_dir, bf,
					    &noc_debug_fops);
		}

		reg = probe->stat_period;
		list_for_each_entry(bf, &reg->bitfields, link) {
			debugfs_create_file(bf->name,
					    S_IRUGO | S_IWUSR, probe_dir, bf,
					    &noc_debug_fops);
		}

		reg = probe->stat_alarm_max;
		list_for_each_entry(bf, &reg->bitfields, link) {
			debugfs_create_file(bf->name,
					    S_IRUGO | S_IWUSR, probe_dir, bf,
					    &noc_debug_fops);
		}

		reg = probe->stat_alarm_min;
		list_for_each_entry(bf, &reg->bitfields, link) {
			debugfs_create_file(bf->name,
					    S_IRUGO | S_IWUSR, probe_dir, bf,
					    &noc_debug_fops);
		}

		for (j = 0; j < noc_device->nr_filters; j++) {
			struct xgold_noc_filter *filter;
			struct dentry *filter_dir;

			filter = &probe->filters[j];

			scnprintf(name, ARRAY_SIZE(name), "filter%d", j);
			filter_dir = debugfs_create_dir(name, probe_dir);
			if (IS_ERR(filter_dir))
				return PTR_ERR(filter_dir);

			reg = filter->route_id;
			list_for_each_entry(bf, &reg->bitfields, link) {
				debugfs_create_file(bf->name,
						    S_IRUGO | S_IWUSR,
						    filter_dir, bf,
						    &noc_debug_fops);

				if (j != 0)
					continue;

				if ((!(strcmp(bf->name, "InitFlow")))
				    || (!(strcmp(bf->name, "TargetFlow")))) {
					scnprintf(name, ARRAY_SIZE(name),
						  "available_%s_source_event",
						  bf->name);
					debugfs_create_file(name, S_IRUGO,
						    filter_dir, bf->lut,
						    &noc_debug_available_fops);
				}
			}

			reg = filter->security_base;
			list_for_each_entry(bf, &reg->bitfields, link) {
				debugfs_create_file(bf->name,
						    S_IRUGO | S_IWUSR,
						    filter_dir, bf,
						    &noc_debug_fops);
			}

			reg = filter->op_code;
			list_for_each_entry(bf, &reg->bitfields, link) {
				debugfs_create_file(bf->name,
						    S_IRUGO | S_IWUSR,
						    filter_dir, bf,
						    &noc_debug_fops);
			}

			reg = filter->status;
			list_for_each_entry(bf, &reg->bitfields, link) {
				debugfs_create_file(bf->name,
						    S_IRUGO | S_IWUSR,
						    filter_dir, bf,
						    &noc_debug_fops);
			}

			bf = filter->addr_base;
			debugfs_create_file("address_base",
					    S_IRUGO | S_IWUSR, filter_dir, bf,
					    &noc_debug_fops);

			bf = filter->window_size;
			debugfs_create_file("window_size",
					    S_IRUGO | S_IWUSR, filter_dir, bf,
					    &noc_debug_fops);

			bf = filter->length;
			debugfs_create_file("packet_length",
					    S_IRUGO | S_IWUSR, filter_dir, bf,
					    &noc_debug_fops);
			bf = filter->urgency;
			debugfs_create_file("urgency_enable",
					    S_IRUGO | S_IWUSR, filter_dir, bf,
					    &noc_debug_fops);

			bf = filter->route_mask;
			debugfs_create_file("route_mask",
					    S_IRUGO | S_IWUSR, filter_dir, bf,
					    &noc_debug_fops);

			bf = filter->security_mask;
			debugfs_create_file("security_mask",
					    S_IRUGO | S_IWUSR, filter_dir, bf,
					    &noc_debug_fops);

		}

		for (j = 0; j < noc_device->nr_counters; j++) {
			struct xgold_noc_counter *cnt;
			struct dentry *dir_cnt;
			cnt = &probe->counters[j];

			scnprintf(name, ARRAY_SIZE(name), "counter%d", j);
			dir_cnt = debugfs_create_dir(name, probe_dir);
			if (IS_ERR(dir_cnt))
				return PTR_ERR(dir_cnt);

			/* We assume the counters are the same */
			if (j == 0) {
				scnprintf(name, ARRAY_SIZE(name),
					  "available_counter_port_selection");
				debugfs_create_file(name, S_IRUGO, probe_dir,
						    cnt->portsel->lut,
						    &noc_debug_available_fops);

				scnprintf(name, ARRAY_SIZE(name),
					  "available_counter_alarm_mode");
				debugfs_create_file(name, S_IRUGO, probe_dir,
						    cnt->alarm_mode->lut,
						    &noc_debug_available_fops);

				scnprintf(name, ARRAY_SIZE(name),
					  "available_counter_source_event");
				debugfs_create_file(name, S_IRUGO, probe_dir,
						    cnt->source_event->lut,
						    &noc_debug_available_fops);
			}

			debugfs_create_file("port_selection", S_IRUGO | S_IWUSR,
					    dir_cnt, cnt->portsel,
					    &noc_debug_fops);

			debugfs_create_file("alarm_mode", S_IRUGO | S_IWUSR,
					    dir_cnt, cnt->alarm_mode,
					    &noc_debug_fops);

			debugfs_create_file("source_event", S_IRUGO | S_IWUSR,
					    dir_cnt, cnt->source_event,
					    &noc_debug_fops);

			debugfs_create_file("value", S_IRUGO, dir_cnt,
					    cnt->value, &noc_debug_fops);

		}

	}

	return 0;
}
