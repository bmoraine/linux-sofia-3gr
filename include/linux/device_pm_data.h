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

/*
 * A device_state_pm_state defines mainly the device external configuration of
 * clocks and power supply. Its main purpose is to allow a device drivers to
 * request the required clock and power resources not necessarily having
 * knowledge about clock and power structures. Changing the power state of a
 * device by only modifying device internals can also be reflected by changing
 * its device_state_pm_state but this is not required, i.e. up to the device
 * driver.
 *
 * A device_state_pm_class defines a list of supported device states that maybe
 * used by different devices. An example of such a class are the PCI device
 * power states defined as D0 ... D3cold. Defining new classes enables you to
 * define an arbitrary amount of low power states for a given device but you
 * should add new classes carefully.
 *
 * A class being used by a device needs to be supported by the underlying power
 * management implementation that is platform dependant. This makes the concept
 *  mainly useful for embedded systems but less useful for PC-like environments
 */

#ifndef _DEVICE_PM_DATA_H_
#define _DEVICE_PM_DATA_H_

#include <linux/device_state_pm.h>

struct device_state_pm_ops {
	int (*set_state)(struct device *, struct device_state_pm_state *);
	struct device_state_pm_state* (*get_initial_state)(struct device *);
};

struct device_state_pm_dev {
	const char *name;		/* Name of the user */
	void *user_config;	/* User details */
	const char *reqd_class;	/* Required class name */
	struct list_head list;
};

struct device_state_pm_class {
	const char *name;			/* Name of the dev pm class */
	struct device_state_pm_state *states;	/* Supported states */
	int num_states;				/* No. of states */
	struct device_state_pm_ops *ops;	/* State change handler*/
	struct list_head list;			/* list for all classes */
};

struct device_state_pm_data {
	struct list_head list;
	struct device_state_pm_class *pm_class;
	struct device_state_pm_dev  *pm_user;
	struct device_state_pm_state *cur_state;
	struct device_state_pm_state *req_state;
	unsigned long active_jiffies;
	unsigned long inactive_jiffies;
	unsigned long accounting_timestamp;
};

/* Used by the PM Framework to register a new class that is supported */
int device_state_pm_add_class(struct device_state_pm_class *class);
int device_state_pm_add_user(struct device_state_pm_dev *user);


/*----- Tracing -----*/
#ifdef CONFIG_PLATFORM_DEVICE_PM_DEBUG
#define PMDTRACE(format, args...) \
	pr_debug("%s: " format, __func__, ##args)
#else
#define PMDTRACE(x...)
#endif

#define PMNTRACE(x...)	pr_notice("#PM: " x)
#define PMWTRACE(x...)	pr_warn("#PM: " x)
#define PMETRACE(x...)	pr_err("#PM: " x)


#ifdef CONFIG_OF
struct device_node;
struct device_pm_platdata *of_device_state_pm_setup(struct device_node *);
#endif

enum device_pm_state {
	PM_STATE_D0 = 0,
	PM_STATE_D1 = 1,
	PM_STATE_D2 = 2,
	PM_STATE_D3 = 3,
	PM_STATE_D0i0 = 4,
	PM_STATE_D0i1 = 5,
	PM_STATE_D0i2 = 6,
	PM_STATE_D0i3 = 7,
	PM_STATE_D0i4 = 8,
	PM_STATE_D0i5 = 9,
	PM_STATE_D0i6 = 10,
};

#define PM_NR_STATES (PM_STATE_D0i6 + 1)

struct device_pm_platdata {
	const char *pm_user_name;
	const char *pm_class_name;
	const char *pm_state_D0_name;
	const char *pm_state_D0i0_name;
	const char *pm_state_D0i1_name;
	const char *pm_state_D0i2_name;
	const char *pm_state_D0i3_name;
	const char *pm_state_D0i4_name;
	const char *pm_state_D0i5_name;
	const char *pm_state_D0i6_name;
	const char *pm_state_D1_name;
	const char *pm_state_D2_name;
	const char *pm_state_D3_name;
	struct device_state_pm_state *pm_states[PM_NR_STATES];
	const char *pm_states_names[PM_NR_STATES];
	void *priv;
};

int device_pm_get_states_handlers(struct device *, struct device_pm_platdata *);

static inline struct device_state_pm_state *get_device_pm_state(
		struct device_pm_platdata *pdata, enum device_pm_state state)
{
	return pdata->pm_states[state];
}

static inline const char *get_device_pm_state_name(
		struct device_pm_platdata *pdata, enum device_pm_state state)
{
	return pdata->pm_states_names[state];
}


#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT

#define DECLARE_DEVICE_STATE_PM_CLASS(_name) \
	static struct device_state_pm_class _name##_pm_class = { \
		.name = #_name "_class", \
		.states = _name##_pm_states, \
		.num_states = ARRAY_SIZE(_name##_pm_states), \
		.ops = &_name##_pm_ops, \
	}
#endif

#endif
