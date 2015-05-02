/*
 * Copyright (C) 2015 Intel Corp. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s(): " fmt, __func__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/kref.h>
#include <linux/slab.h>

#include <linux/notifier.h>
#include <linux/rpmb.h>

static struct blocking_notifier_head rpmb_notifier;
static DEFINE_MUTEX(rpmb_dev_list_mutex);
static LIST_HEAD(rpmb_dev_list);

struct rpmb_dev *rpmb_dev_get(struct rpmb_dev *dev)
{
	if (dev && kref_get_unless_zero(&dev->refcnt))
		return dev;
	return NULL;
}
EXPORT_SYMBOL_GPL(rpmb_dev_get);

static void rpmb_dev_release(struct kref *refcnt)
{
	struct rpmb_dev *dev = container_of(refcnt, struct rpmb_dev, refcnt);
	put_device(dev->parent);
	kfree(dev);
}

void rpmb_dev_put(struct rpmb_dev *dev)
{
	if (dev)
		kref_put(&dev->refcnt, rpmb_dev_release);
}
EXPORT_SYMBOL_GPL(rpmb_dev_put);

int rpmb_send_req(struct rpmb_dev *dev, struct rpmb_data *data)
{
	struct gendisk *disk;
	int err;

	if (!dev || !data)
		return -EINVAL;

	disk = dev_to_disk(dev->parent);

	mutex_lock(&dev->lock);
	if (dev->ops && dev->ops->send_rpmb_req)
		err = dev->ops->send_rpmb_req(disk, data);
	else
		err = -EOPNOTSUPP;
	mutex_unlock(&dev->lock);
	return err;
}
EXPORT_SYMBOL_GPL(rpmb_send_req);

struct rpmb_dev *rpmb_dev_find_by_disk(struct gendisk *disk)
{
	struct rpmb_dev *_dev, *dev = NULL;

	if (!disk)
		return NULL;

	mutex_lock(&rpmb_dev_list_mutex);
	list_for_each_entry(_dev, &rpmb_dev_list, dev_list) {
		if (_dev->parent == disk_to_dev(disk)) {
			dev = rpmb_dev_get(_dev);
			break;
		}
	}
	mutex_unlock(&rpmb_dev_list_mutex);

	return dev;
}
EXPORT_SYMBOL_GPL(rpmb_dev_find_by_disk);

struct rpmb_dev *rpmb_dev_get_default(void)
{
	struct rpmb_dev *_dev, *dev;

	mutex_lock(&rpmb_dev_list_mutex);
	_dev = list_first_entry_or_null(&rpmb_dev_list,
					struct rpmb_dev, dev_list);
	dev = rpmb_dev_get(_dev);
	mutex_unlock(&rpmb_dev_list_mutex);

	if (dev)
		pr_debug("default selected %s:%s\n",
			 dev_name(dev->parent), dev->disk->disk_name);
	else
		pr_warn("no disk found\n");

	return dev;
}
EXPORT_SYMBOL_GPL(rpmb_dev_get_default);


static int __rpmb_dev_unregister(struct rpmb_dev *dev)
{

	pr_debug("unregister disk %s:%s\n",
		 dev_name(dev->parent), dev->disk->disk_name);

	mutex_unlock(&rpmb_dev_list_mutex);
	blocking_notifier_call_chain(&rpmb_notifier, RPMB_PART_REMOVE, dev);

	mutex_lock(&rpmb_dev_list_mutex);
	list_del(&dev->dev_list);
	rpmb_dev_put(dev);


	return 0;
}

int rpmb_dev_unregister(struct gendisk *disk)
{
	struct rpmb_dev *dev;

	if (!disk)
		return -EINVAL;

	dev = rpmb_dev_find_by_disk(disk);
	if (!dev) {
		pr_warn("no disk found %s\n", disk->disk_name);
		return -ENODEV;
	}

	mutex_lock(&rpmb_dev_list_mutex);
	__rpmb_dev_unregister(dev);
	mutex_unlock(&rpmb_dev_list_mutex);

	rpmb_dev_put(dev);

	return 0;
}
EXPORT_SYMBOL_GPL(rpmb_dev_unregister);

int rpmb_dev_suspend(struct gendisk *disk)
{
	struct rpmb_dev *dev;
	if (!disk)
		return -EINVAL;

	dev = rpmb_dev_find_by_disk(disk);
	if (!dev) {
		pr_warn("no disk found %s\n", disk->disk_name);
		return -ENODEV;
	}

	pr_debug("suspend %s:%s\n", dev_name(dev->parent), disk->disk_name);

	blocking_notifier_call_chain(&rpmb_notifier, RPMB_PART_SUSPEND, dev);
	rpmb_dev_put(dev);

	return 0;
}
EXPORT_SYMBOL_GPL(rpmb_dev_suspend);

int rpmb_dev_resume(struct gendisk *disk)
{
	struct rpmb_dev *dev;

	if (!disk)
		return -EINVAL;

	dev = rpmb_dev_find_by_disk(disk);
	if (!dev) {
		pr_warn("no disk found %s\n", disk->disk_name);
		return -ENODEV;
	}

	pr_debug("suspend %s:%s\n", dev_name(dev->parent), disk->disk_name);

	blocking_notifier_call_chain(&rpmb_notifier, RPMB_PART_RESUME, dev);
	rpmb_dev_put(dev);

	return 0;
}
EXPORT_SYMBOL_GPL(rpmb_dev_resume);

static void rpmb_dev_work(struct work_struct *work)
{
	struct rpmb_dev *dev = container_of(work, struct rpmb_dev, work);
	blocking_notifier_call_chain(&rpmb_notifier, RPMB_PART_ADD, dev);
}

struct rpmb_dev *rpmb_dev_register(struct gendisk *disk, struct rpmb_ops *ops)
{
	struct rpmb_dev *dev;
	struct device *parent;

	if (!disk || !ops)
		return ERR_PTR(-EINVAL);

	parent = disk_to_dev(disk);

	if (!parent || !ops->send_rpmb_req)
		return ERR_PTR(-EINVAL);

	dev = kzalloc(sizeof(struct rpmb_dev), GFP_KERNEL);
	if (!dev)
		return ERR_PTR(-ENOMEM);

	mutex_init(&dev->lock);
	kref_init(&dev->refcnt);
	dev->parent = get_device(parent);
	dev->name   = dev_name(parent);
	dev->ops    = ops;
	dev->disk   = disk;
	INIT_WORK(&dev->work, rpmb_dev_work);

	mutex_lock(&rpmb_dev_list_mutex);
	list_add_tail(&dev->dev_list, &rpmb_dev_list);
	mutex_unlock(&rpmb_dev_list_mutex);

	schedule_work(&dev->work);

	pr_debug("registered disk %s:%s\n", dev_name(parent), disk->disk_name);

	return dev;
}
EXPORT_SYMBOL_GPL(rpmb_dev_register);


int rpmb_register_notify(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&rpmb_notifier, nb);

}
EXPORT_SYMBOL_GPL(rpmb_register_notify);

int rpmb_unregister_notify(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&rpmb_notifier, nb);
}
EXPORT_SYMBOL_GPL(rpmb_unregister_notify);


static int __init rpmb_init(void)
{
	return 0;
}

static void __exit rpmb_exit(void)
{
	struct rpmb_dev *dev, *n;

	mutex_lock(&rpmb_dev_list_mutex);
	list_for_each_entry_safe(dev, n, &rpmb_dev_list, dev_list)
		__rpmb_dev_unregister(dev);
	mutex_unlock(&rpmb_dev_list_mutex);
}

subsys_initcall(rpmb_init);
module_exit(rpmb_exit);
MODULE_LICENSE("GPL");
