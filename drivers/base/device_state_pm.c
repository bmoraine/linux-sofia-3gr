/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
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

#include <linux/device.h>
#include <linux/device_pm_data.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/stat.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>

static LIST_HEAD(class_list);
static LIST_HEAD(user_list);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(class_list_mtx);
static DEFINE_MUTEX(user_list_mtx);
static DEFINE_MUTEX(device_list_mtx);

struct pm_sysfs_work_t {
	struct work_struct task;
	struct device *dev;
};

static void update_residencies(struct device *dev);
static void device_state_pm_add_sysfs_work(struct work_struct *w);

/* Used by the platform dependant power management */
int device_state_pm_add_class(struct device_state_pm_class *class)
{
	if (NULL == class)
		return -EINVAL;
	PMDTRACE("#PM_DBG: Added->PM Class Name: %s\n", class->name);
	mutex_lock(&class_list_mtx);
	list_add_tail(&class->list, &class_list);
	mutex_unlock(&class_list_mtx);
	return 0;
}
EXPORT_SYMBOL_GPL(device_state_pm_add_class);

int device_state_pm_add_user(struct device_state_pm_dev *user)
{
	if (NULL == user)
		return -EINVAL;
	PMDTRACE("#PM_DBG: Added->PM User Name: %s\n", user->name);
	mutex_lock(&user_list_mtx);
	list_add_tail(&user->list, &user_list);
	mutex_unlock(&user_list_mtx);
	return 0;
}
EXPORT_SYMBOL_GPL(device_state_pm_add_user);

struct device_state_pm_class *device_state_pm_find_class(const char *name)
{
	struct device_state_pm_class *c;

	if (NULL == name)
		return NULL;

	mutex_lock(&class_list_mtx);
	list_for_each_entry(c, &class_list, list) {
		if (0 == strcmp(c->name, name)) {
			mutex_unlock(&class_list_mtx);
			return c;
		}
	}

	mutex_unlock(&class_list_mtx);
	return NULL;
}
EXPORT_SYMBOL_GPL(device_state_pm_find_class);

/*  Used by the device driver (typically in the probe function) to configure
 *  which platform power management class this device belongs to. It is also
 *  used to initialize other parts of pm_data of dev.
 */
int device_state_pm_set_class(struct device *dev, const char *name)
{
	struct device_state_pm_class *c;
	struct device_state_pm_dev *u;
	int flag = 0, ret = 0;

	if (dev == NULL) {
		PMETRACE("Error in dev: %s\n", name);
		return -EINVAL;
	}
	mutex_lock(&user_list_mtx);
	list_for_each_entry(u, &user_list, list) {
		if (!strcmp(u->name, name)) {
			dev->pm_data.pm_user = u;
			flag = 1;
			break;
		}
	}
	mutex_unlock(&user_list_mtx);

	if (!flag) {
		PMETRACE("Error in class: %s\n", name);
		return -ENODEV;
	}
	mutex_lock(&class_list_mtx);
	list_for_each_entry(c, &class_list, list) {
		if (!strcmp(c->name, u->reqd_class)) {
			struct pm_sysfs_work_t *w;
			if (!c->ops || !c->ops->get_initial_state) {
				ret = -EINVAL;
				goto error;
			}
			PMDTRACE("#PM_DBG: Registered->PM class name: %s\n",
								u->reqd_class);
			dev->pm_data.pm_class = c;
			dev->pm_data.active_jiffies = 0;
			dev->pm_data.inactive_jiffies = 0;
			dev->pm_data.accounting_timestamp = jiffies;
			dev->pm_data.cur_state = c->ops->get_initial_state(dev);
			if (dev->kobj.kset) {
				w = kmalloc(sizeof(struct pm_sysfs_work_t),
								GFP_KERNEL);
				if (w) {
					INIT_WORK((struct work_struct *)w,
						device_state_pm_add_sysfs_work);
					w->dev = dev;
					schedule_work((struct work_struct *)w);
				} else {
					PMETRACE("kmalloc failed\n");
					ret = -ENOMEM;
					goto error;
				}
			}
			mutex_lock(&device_list_mtx);
			list_add_tail(&(dev->pm_data.list), &device_list);
			mutex_unlock(&device_list_mtx);
			goto out;
		}
	}
error:
	PMDTRACE("#PM_DBG: ERROR->PM class name not registered: %s\n",
								u->reqd_class);
out:
	mutex_unlock(&class_list_mtx);
	return ret;
}
EXPORT_SYMBOL_GPL(device_state_pm_set_class);

int device_state_pm_remove_device(struct device *dev)
{
	if ((!dev) || (dev->pm_data.pm_class == NULL) ||
			(dev->pm_data.pm_user == NULL)) {
		return -ENOENT;
	}
	dev->pm_data.pm_class = NULL;
	dev->pm_data.pm_user = NULL;
	mutex_lock(&device_list_mtx);
	list_del(&(dev->pm_data.list));
	mutex_unlock(&device_list_mtx);
	return 0;
}
EXPORT_SYMBOL_GPL(device_state_pm_remove_device);

/* Used by device driver to obtain a ptr to the state structure. This
 * can be used with device_state_pm_set_state to omit the string search of
 * device_state_pm_set_state_by_name */
struct device_state_pm_state *device_state_pm_get_state_handler(
				struct device *dev, const char *state_name)
{
	int i;
	struct device_state_pm_class *c;

	if ((!dev) || (dev->pm_data.pm_class == NULL)) {
		PMETRACE("Error in dev:%s\n", state_name);
		return NULL;
	}

	c = dev->pm_data.pm_class;
	for (i = 0; i < c->num_states; i++) {
		if (!strcmp(c->states[i].name, state_name))
			return &(c->states[i]);
	}
	PMETRACE("Error in state handler:%s\n", state_name);
	return NULL;
}
EXPORT_SYMBOL_GPL(device_state_pm_get_state_handler);

static int _device_state_pm_set_state(struct device *dev,
		struct device_state_pm_state *state)
{
	int ret;
	PMDTRACE("#PM_DBG: Device:%s, Required state:%s\n",
			dev->pm_data.pm_user->name,
			state->name);
	if (state == dev->pm_data.cur_state)
		return 0;
	might_sleep(); /* Prints stack, if called from atomic context*/
	ret = dev->pm_data.pm_class->ops->set_state(dev, state);
	if (ret)
		return ret;
	update_residencies(dev);
	dev->pm_data.cur_state = state;
	return ret;
}

/* Set the device state of the given device */
int device_state_pm_set_state(struct device *dev,
		struct device_state_pm_state *state)
{
	if ((!dev) || (!dev->pm_data.pm_class) || (!state)) {
		PMETRACE("Error in dev\n");
		return -EINVAL;
	}
	if (state != &(dev->pm_data.pm_class->states[0]))
		dev->pm_data.req_state = state;
	return _device_state_pm_set_state(dev, state);
}
EXPORT_SYMBOL_GPL(device_state_pm_set_state);

/* Set the device state of the given device */
int device_state_pm_set_state_by_name(struct device *dev, const char *name)
{
	int i;
	struct device_state_pm_class *c;

	if ((!dev) || (!dev->pm_data.pm_class) || (!name)) {
		PMETRACE("Error in dev\n");
		return -EINVAL;
	}
	c = dev->pm_data.pm_class;
	for (i = 0; i < c->num_states; i++) {
		if (!strcmp(name, c->states[i].name)) {
			struct device_state_pm_state *state = &(c->states[i]);
			PMDTRACE("#PM_DBG: State matched:%s\n", name);
			if (state != &(dev->pm_data.pm_class->states[0]))
				dev->pm_data.req_state = state;
			return _device_state_pm_set_state(dev, state);
		}
	}
	PMDTRACE("#PM_DBG: Error->State not matched:%s\n", name);
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(device_state_pm_set_state_by_name);

/* Select which active state shall be used. Change to it if the device is
 * already active */
int device_state_pm_set_active_state(struct device *dev,
		struct device_state_pm_state *state)
{
	if ((!dev) || (!dev->pm_data.pm_class) || (!state) ||
	    (state == &(dev->pm_data.pm_class->states[0]))) {
		PMETRACE("Error in dev\n");
		return -EINVAL;
	}
	dev->pm_data.req_state = state;
	if (dev->pm_data.cur_state != &(dev->pm_data.pm_class->states[0])) {
		/* If state is active then change the active state */
		_device_state_pm_set_state(dev, state);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(device_state_pm_set_active_state);

int device_state_pm_activate(struct device *dev, bool activate)
{
	if ((!dev) || (!dev->pm_data.pm_class)) {
		PMETRACE("Error in dev\n");
		return -EINVAL;
	}
	if (activate)
		_device_state_pm_set_state(dev, dev->pm_data.req_state);
	else
		_device_state_pm_set_state(dev,
				&(dev->pm_data.pm_class->states[0]));
	return 0;
}
EXPORT_SYMBOL_GPL(device_state_pm_activate);

/* SYSFS support
 * Example: cat /sys/devices/platform/dcc/pm_state
 */
static const char devpm_group_name[] = "devpm";

static ssize_t device_state_pm_show_state(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", dev->pm_data.cur_state->name);
}

static DEVICE_ATTR(devpm_state, S_IRUGO, device_state_pm_show_state, NULL);
static ssize_t device_state_pm_show_active_time(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	update_residencies(dev);
	return sprintf(buf, "%i\n",
			jiffies_to_msecs(dev->pm_data.active_jiffies));
}
static DEVICE_ATTR(devpm_state_active_time, S_IRUGO,
			device_state_pm_show_active_time, NULL);

static ssize_t device_state_pm_show_inactive_time(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	update_residencies(dev);
	return sprintf(buf, "%i\n",
			jiffies_to_msecs(dev->pm_data.inactive_jiffies));
}
static DEVICE_ATTR(devpm_state_inactive_time, S_IRUGO,
				device_state_pm_show_inactive_time, NULL);

static ssize_t device_state_pm_show_devpm_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", dev->pm_data.pm_user->name);
}
static DEVICE_ATTR(devpm_name, S_IRUGO, device_state_pm_show_devpm_name, NULL);

static ssize_t device_state_pm_show_devpm_class(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", dev->pm_data.pm_class->name);
}
static DEVICE_ATTR(devpm_class, S_IRUGO,
		device_state_pm_show_devpm_class, NULL);

static struct attribute *devpm_attrs[] = {
	&dev_attr_devpm_state.attr,
	&dev_attr_devpm_state_active_time.attr,
	&dev_attr_devpm_state_inactive_time.attr,
	&dev_attr_devpm_name.attr,
	&dev_attr_devpm_class.attr,
	NULL,
};
static struct attribute_group devpm_attr_group = {
	.name   = devpm_group_name,
	.attrs  = devpm_attrs,
};

static void device_state_pm_add_sysfs_work(struct work_struct *work)
{
	int ret;
	struct device *dev;
	dev = ((struct pm_sysfs_work_t *)work)->dev;
	ret = sysfs_create_group(&dev->kobj, &devpm_attr_group);
	if (ret) {
		PMETRACE("Error adding sysfs file\n");
		return;
	}

	kfree((void *)work);
}


/* This is more or less a copy of what runtime PM (very nicely:)) does */
static void update_residencies(struct device *dev)
{
	unsigned long now = jiffies;
	int delta;
	delta = now - dev->pm_data.accounting_timestamp;
	if (delta < 0)
		delta = 0;
	dev->pm_data.accounting_timestamp = now;
	if (dev->pm_data.cur_state == &(dev->pm_data.pm_class->states[0]))
		dev->pm_data.inactive_jiffies += delta;
	else
		dev->pm_data.active_jiffies += delta;
}

#ifdef CONFIG_OF

#define OF_PM_USER_NAME		"pm,user-name"
#define OF_PM_CLASS_NAME	"pm,class-name"
#define OF_PM_STATE_DO		"pm,state-D0"
#define OF_PM_STATE_DOi0	"pm,state-D0i0"
#define OF_PM_STATE_DOi1	"pm,state-D0i1"
#define OF_PM_STATE_DOi2	"pm,state-D0i2"
#define OF_PM_STATE_DOi3	"pm,state-D0i3"
#define OF_PM_STATE_DOi4	"pm,state-D0i4"
#define OF_PM_STATE_DOi5	"pm,state-D0i5"
#define OF_PM_STATE_DOi6	"pm,state-D0i6"
#define OF_PM_STATE_D1		"pm,state-D1"
#define OF_PM_STATE_D2		"pm,state-D2"
#define OF_PM_STATE_D3		"pm,state-D3"

struct device_pm_platdata *of_device_state_pm_setup(
		struct device_node *np)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	struct device_state_pm_dev *pm_user;
#endif
	struct device_pm_platdata *pm_platdata;
	int ret;

	pm_platdata = kzalloc(sizeof(*pm_platdata), GFP_KERNEL);
	if (!pm_platdata)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_string(np, OF_PM_USER_NAME,
			&pm_platdata->pm_user_name);
	if (ret) {
		ret = -EINVAL;
		goto err_free_pm_data;
	}

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	ret = of_property_read_string(np, OF_PM_CLASS_NAME,
			&pm_platdata->pm_class_name);
	if (ret) {
		PMETRACE("ERROR: Cannot get %s property of node %s\n",
				OF_PM_CLASS_NAME, np->name);
		ret = -EINVAL;
		goto err_free_pm_data;
	}

	pm_user = kzalloc(sizeof(*pm_user), GFP_KERNEL);
	if (!pm_user) {
		ret = -ENOMEM;
		goto err_free_pm_data;
	}
	pm_user->name = (char *)pm_platdata->pm_user_name;
	pm_user->reqd_class = (char *)pm_platdata->pm_class_name;
	/* user_config is don't care */

	ret = device_state_pm_add_user(pm_user);
	if (ret) {
		PMETRACE("ERROR: Cannot add pm user %s\n", pm_user->name);
		goto err_free_pm_data;
	}
#endif

	of_property_read_string(np, OF_PM_STATE_DO,
			&pm_platdata->pm_state_D0_name);
	of_property_read_string(np, OF_PM_STATE_DOi0,
			&pm_platdata->pm_state_D0i0_name);
	of_property_read_string(np, OF_PM_STATE_DOi1,
			&pm_platdata->pm_state_D0i1_name);
	of_property_read_string(np, OF_PM_STATE_DOi2,
			&pm_platdata->pm_state_D0i2_name);
	of_property_read_string(np, OF_PM_STATE_DOi3,
			&pm_platdata->pm_state_D0i3_name);
	of_property_read_string(np, OF_PM_STATE_DOi4,
			&pm_platdata->pm_state_D0i4_name);
	of_property_read_string(np, OF_PM_STATE_DOi5,
			&pm_platdata->pm_state_D0i5_name);
	of_property_read_string(np, OF_PM_STATE_DOi6,
			&pm_platdata->pm_state_D0i6_name);

	of_property_read_string(np, OF_PM_STATE_D1,
			&pm_platdata->pm_state_D1_name);
	of_property_read_string(np, OF_PM_STATE_D2,
			&pm_platdata->pm_state_D2_name);
	of_property_read_string(np, OF_PM_STATE_D3,
			&pm_platdata->pm_state_D3_name);

	pm_platdata->pm_states_names[PM_STATE_D0] =
				pm_platdata->pm_state_D0_name;

	pm_platdata->pm_states_names[PM_STATE_D1] =
				pm_platdata->pm_state_D1_name;

	pm_platdata->pm_states_names[PM_STATE_D2] =
				pm_platdata->pm_state_D2_name;

	pm_platdata->pm_states_names[PM_STATE_D3] =
				pm_platdata->pm_state_D3_name;

	pm_platdata->pm_states_names[PM_STATE_D0i0] =
				pm_platdata->pm_state_D0i0_name;

	pm_platdata->pm_states_names[PM_STATE_D0i1] =
				pm_platdata->pm_state_D0i1_name;

	pm_platdata->pm_states_names[PM_STATE_D0i2] =
				pm_platdata->pm_state_D0i2_name;

	pm_platdata->pm_states_names[PM_STATE_D0i3] =
				pm_platdata->pm_state_D0i3_name;

	pm_platdata->pm_states_names[PM_STATE_D0i4] =
				pm_platdata->pm_state_D0i4_name;

	pm_platdata->pm_states_names[PM_STATE_D0i5] =
				pm_platdata->pm_state_D0i5_name;

	pm_platdata->pm_states_names[PM_STATE_D0i6] =
				pm_platdata->pm_state_D0i6_name;

	return pm_platdata;

err_free_pm_data:
	kfree(pm_platdata);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(of_device_state_pm_setup);
#endif

int device_pm_get_states_handlers(
		struct device *dev, struct device_pm_platdata *pdata)
{
	int i;

	if ((pdata == NULL) || (dev == NULL))
		return -EINVAL;

	for (i = 0; i < PM_NR_STATES; i++) {
		enum device_pm_state state = i;
		const char *name = get_device_pm_state_name(pdata, state);
		if (name)
			pdata->pm_states[i] =
				device_state_pm_get_state_handler(dev, name);
		else
			pdata->pm_states[i] = NULL;
	}

	return 0;
}
EXPORT_SYMBOL(device_pm_get_states_handlers);

/* Retrive state index in a device's class from the state name */
int device_state_pm_get_state_id(struct device *dev, const char *state_name)
{
	struct device_state_pm_class *class = dev->pm_data.pm_class;
	int i = 0;

	if (!class)
		return -ENODEV;

	for (i = 0; i < class->num_states; i++) {
		if (!strcmp(state_name, class->states[i].name))
			return i;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(device_state_pm_get_state_id);

