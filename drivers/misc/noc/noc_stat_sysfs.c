/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <asm/div64.h>
#include <linux/interrupt.h>

#include "noc.h"

#define SIZE_BIG_BUF	4096
#define SIZE_SMALL_BUF	256

const char *__clk_get_name(struct clk *clk);

/**
 * Called when writing run file
 */
static ssize_t noc_stat_run_write(struct file *filp, const char __user *ubuf,
				  size_t cnt, loff_t *ppos)
{
	struct xgold_noc_device *noc_device = filp->private_data;
	struct xgold_noc_stat_probe *stat_probe;
	struct xgold_noc_stat_measure *stat_measure;
	char buf[SIZE_SMALL_BUF];
	unsigned ret;
	unsigned long flags;

	if (cnt > SIZE_SMALL_BUF)
		cnt = SIZE_SMALL_BUF - 1;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;

	if (buf[0] == '0')
		noc_device->stat->run = false;	/* FIXME spinlock? */
	else if (buf[0] == '1') {
		noc_device->stat->run = true;
		/* Reset statistics */
		list_for_each_entry(stat_probe,
					&noc_device->stat->probe, link) {
			list_for_each_entry(stat_measure, &stat_probe->measure,
					    link) {
				spin_lock_irqsave(&stat_measure->lock, flags);
				stat_measure->min = -1;
				stat_measure->max = 0;
				stat_measure->now = 0;
				stat_measure->iteration = 0;
				stat_measure->mean = 0;
				spin_unlock_irqrestore(&stat_measure->lock,
						       flags);
			}
		}
		/* Launch measurements */
		list_for_each_entry(stat_probe,
				&noc_device->stat->probe, link) {
			ret = xgold_noc_stat_configure_measure(noc_device,
							       stat_probe);
			if (ret)
				dev_err(noc_device->dev,
					"Error when configuring stat measures!\n");
		}
	} else
		return -EFAULT;

	*ppos += cnt;

	return cnt;
}

/**
 * Called when reading run file
 */
static ssize_t noc_stat_run_read(struct file *filp, char __user *ubuf,
				 size_t cnt, loff_t *ppos)
{
#define RUN_STR_SIZE 11
	struct xgold_noc_device *noc_device = filp->private_data;
	char buf[RUN_STR_SIZE];
	int r;

	r = snprintf(buf, RUN_STR_SIZE, "%i\n", noc_device->stat->run);

	return simple_read_from_buffer(ubuf, cnt, ppos, buf, r);
}

/**
 * Called when writing config file
 */
static ssize_t noc_stat_config_write(struct file *filp,
				     const char __user *ubuf, size_t cnt,
				     loff_t *ppos)
{
	return 0;
}

/**
 * Called when reading config file
 */
static ssize_t noc_stat_config_read(struct file *filp, char __user *ubuf,
				    size_t cnt, loff_t *ppos)
{
	return 0;
}

/**
 * Called when reading readme file
 * @param m
 * @param p
 * @return
 */
static int noc_stat_readme_show(struct seq_file *m, void *p)
{
	seq_puts(m,
		 "How to use the NOC sniffer:\nFile run: write '1' to enable "
		 "sniffer (enabling sniffer resets statistics),"
		 " write '0' to disable sniffer.\n"
		 "File config: Not yet implemented.\n"
		 "File results: contain the results of measurement in KB/s "
		 "for all nodes defined in dts.\n");
	return 0;
}

/**
 * Called when reading results file.
 */
static ssize_t noc_stat_results_read(struct file *filp, char __user *ubuf,
				     size_t cnt, loff_t *ppos)
{
	struct xgold_noc_device *noc_device = filp->private_data;
	struct xgold_noc_stat *stat;
	struct xgold_noc_stat_probe *stat_probe;
	struct xgold_noc_stat_measure *stat_measure;
	u64 meas_min, meas_max, meas_mean, meas_now;
	u64 clock_noc;
	struct clk *myclk;
	char *buf1;
	char buf2[SIZE_SMALL_BUF + 2];
	int r;
	ssize_t ret;
	int count;		/* Used to avoid write over buffer size */

	if (noc_device == NULL)
		return -ENODEV;

	stat = noc_device->stat;
	if (!stat)
		return -ENODEV;
	clock_noc = (u64) stat->clock_rate;

	buf1 = kzalloc(sizeof(*buf1) * SIZE_BIG_BUF, GFP_KERNEL);
	if (!buf1)
		return -ENODEV;

	myclk = noc_device->clock;
	if (clock_noc == 0)
		clock_noc = (u64) clk_get_rate(myclk);

	r = snprintf(buf1, SIZE_BIG_BUF,
		     "\t##### RESULTS #####\n Clk is: %llu Hz\n", clock_noc);
	count = SIZE_BIG_BUF - strlen(buf1);
	list_for_each_entry(stat_probe, &stat->probe, link) {
		list_for_each_entry(stat_measure, &stat_probe->measure, link) {
			if (count > 0) {
				if (stat_measure->idx_init_flow != 0xff)
					r = snprintf(buf2, SIZE_SMALL_BUF,
						     "Probe %i (%s): %s-->%s\n",
						     stat_probe->probe->id,
						     stat_probe->probe->
						     available_portsel
						     [stat_measure->
						      idx_trace_port_sel],
						     stat_measure->init_flow,
						     stat_measure->target_flow);
				else
					r = snprintf(buf2, SIZE_SMALL_BUF,
						     "Probe %i (%s):\n",
						     stat_probe->probe->id,
						     stat_probe->probe->
						     available_portsel
						     [stat_measure->
						      idx_trace_port_sel]);

				strncat(buf1, buf2, count);
				count -= strlen(buf2);
			}
			if (count > 0) {
				if (stat_measure->iteration != 0) {
					meas_min = (u64) stat_measure->min;
					meas_max = (u64) stat_measure->max;
					meas_mean = (u64) stat_measure->mean;
					do_div(meas_mean,
					       stat_measure->iteration);
					meas_now = (u64) stat_measure->now;
					meas_min *= clock_noc;
					meas_max *= clock_noc;
					meas_mean *= clock_noc;
					meas_now *= clock_noc;
					r = snprintf(buf2, SIZE_SMALL_BUF,
						     "\t min=%u (KB/s), max=%u (KB/s) mean=%u (KB/s) current=%u (KB/s) NbOfMeasure=%i\n",
						     (unsigned)(meas_min >>
								(10 +
								 DURATION)),
						     (unsigned)(meas_max >>
								(10 +
								 DURATION)),
						     (unsigned)(meas_mean >>
								(10 +
								 DURATION)),
						     (unsigned)(meas_now >>
								(10 +
								 DURATION)),
						     stat_measure->iteration);
					strncat(buf1, buf2, count);
					count -= strlen(buf2);
				}
			}
		}
	}

	r = strlen(buf1) + 1;

	ret = simple_read_from_buffer(ubuf, cnt, ppos, buf1, r);

	kfree(buf1);
	return ret;

}

/**
 * Called when open results file
 * @param inode
 * @param file
 * @return
 */
static int noc_stat_results_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

/**
 * Called when opening config file
 * @param inode
 * @param file
 * @return
 */
static int noc_stat_config_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

/**
 * Called when opening run file
 * @param inode
 * @param file
 * @return
 */
static int noc_stat_run_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

/**
 * Called when reading readme file
 * @param inode
 * @param file
 * @return
 */
static int noc_stat_readme_open(struct inode *inode, struct file *file)
{
	return single_open(file, noc_stat_readme_show, inode->i_private);
}

static const struct file_operations noc_stat_results_fops = {
	.open = noc_stat_results_open,
	.read = noc_stat_results_read,
	.llseek = generic_file_llseek,
};

static const struct file_operations noc_stat_config_fops = {
	.open = noc_stat_config_open,
	.read = noc_stat_config_read,
	.write = noc_stat_config_write,
	.llseek = generic_file_llseek,
};

static const struct file_operations noc_stat_run_fops = {
	.open = noc_stat_run_open,
	.read = noc_stat_run_read,
	.write = noc_stat_run_write,
	.llseek = generic_file_llseek,
};

static const struct file_operations noc_stat_readme_fops = {
	.open = noc_stat_readme_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/**
 * Entry point for NOC sniffer sysfs
 * @param _dev A reference to the device
 * @return 0
 */
int xgold_noc_stat_debugfs_init(struct device *_dev)
{
	struct xgold_noc_device *noc_device = dev_get_drvdata(_dev);
	struct device_node *np = _dev->of_node;
	struct dentry *dir, *d;
	char name[32];

	scnprintf(name, ARRAY_SIZE(name), "%s_stat", np->name);

	dir = debugfs_create_dir(name, NULL);
	if (IS_ERR(dir))
		return PTR_ERR(dir);

	noc_device->stat->dir = dir;

	d = debugfs_create_file("results", S_IRUGO, dir, noc_device,
				&noc_stat_results_fops);
	if (IS_ERR(d))
		return PTR_ERR(d);
	d = debugfs_create_file("config", S_IRUGO | S_IWUSR, dir, noc_device,
				&noc_stat_config_fops);
	if (IS_ERR(d))
		return PTR_ERR(d);
	d = debugfs_create_file("run", S_IRUGO | S_IWUSR, dir, noc_device,
				&noc_stat_run_fops);
	if (IS_ERR(d))
		return PTR_ERR(d);
	d = debugfs_create_file("readme.txt", S_IRUGO, dir, noc_device,
				&noc_stat_readme_fops);
	if (IS_ERR(d))
		return PTR_ERR(d);

	return 0;
}
