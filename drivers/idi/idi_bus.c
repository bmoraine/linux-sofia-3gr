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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>

#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_bus.h>

#include <linux/idi/idi_debug.h>

MODULE_LICENSE("GPL");

#define IDI_BUS_ENTER pr_debug("--> %s\n", __func__)
#define IDI_BUS_EXIT pr_debug("<-- %s\n", __func__)

#define IDI_PREFIX "idi:"

static DEFINE_IDR(idi_bus_idr);
static DEFINE_SPINLOCK(idi_bus_lock);

static LIST_HEAD(__idi_client_list);
static DECLARE_RWSEM(__idi_client_device_lock);

struct _idi_client_device_info {
	struct list_head list;
	int busnum;
	struct idi_client_device_info info;
};

static struct device_type idi_ctrl = {
	.name = "idi_controller",

};

static struct device_type idi_client = {
	.name = "idi_client",
};

static struct device_node idi_client_info = {
	.type = "agold",
};

static struct device_type idi_peripheral = {
	.name = "idi_peripheral",
};

struct bus_type idi_bus_type;

static void idi_scan_static_client(struct idi_controller_device *);
static struct idi_peripheral_device *idi_verify_peripheral(struct device *);
static ssize_t modalias_show(struct device *dev, struct device_attribute *a,
			     char *buf)
{
	struct idi_peripheral_device *pdev = idi_verify_peripheral(dev);
	pr_debug("%s:\n", __func__);

	if (pdev)
		return snprintf(buf, PAGE_SIZE + 1,
			"MODALIAS=%sv%08Xd%08Xsd%08X\n", IDI_PREFIX,
			pdev->id.vendor, pdev->id.device, pdev->id.subdevice);
	else
		return snprintf(buf, PAGE_SIZE + 1, "%s%s\n", IDI_PREFIX,
			dev_name(dev));
}

static struct device_attribute idi_dev_attrs[] = {
	__ATTR_RO(modalias),
	__ATTR_NULL,
};

static int idi_bus_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct idi_peripheral_device *pdev = idi_verify_peripheral(dev);
	pr_debug("%s: %s\n", __func__, dev_name(dev));

	if (pdev)
		add_uevent_var(env, "MODALIAS=%sv%08Xd%08Xsd%08X",
			IDI_PREFIX, pdev->id.vendor,
			pdev->id.device, pdev->id.subdevice);
	else
		add_uevent_var(env, "MODALIAS=%s%s",
				IDI_PREFIX, dev_name(dev));

	return 0;

}				/* idi_bus_uevent() */

/**
 * idi_verify_peripheral - make sure device is a peripheral device.
 * @dev: device to check.
 *
 */
static struct idi_peripheral_device *idi_verify_peripheral(struct device *dev)
{
	return (dev && (dev->type == &idi_peripheral)) ?
	    to_idi_peripheral_device(dev) : NULL;

}				/* idi_verify_peripheral() */

/**
 * idi_verify_client - make sure device is a client device.
 * @dev: device to check.
 *
 */
static struct idi_client_device *idi_verify_client(struct device *dev)
{
	return (dev && (dev->type == &idi_client)) ?
				to_idi_client_device(dev) : NULL;

}				/* idi_verify_client() */


static inline const struct idi_device_id *idi_match_one_device(
		const struct idi_device_id *id,
		const struct idi_peripheral_device *peripheral)
{
	const struct idi_device_id *devid = &peripheral->id;

	pr_debug("driver id: %x:%x:%x, device id: %x:%x:%x\n",
		id->vendor, id->device, id->subdevice,
		devid->vendor, devid->device, devid->subdevice);

#define IDI_MATCH(_id, _devid, _str)\
	(_id->_str == IDI_ANY_ID || _id->_str == _devid->_str)
	if (
		(IDI_MATCH(id, devid, vendor)) &&
		(IDI_MATCH(id, devid, device)) &&
		(IDI_MATCH(id, devid, subdevice)))
		return id;
	return NULL;
}




static const struct idi_device_id *idi_match_device(
		struct idi_peripheral_device *peripheral,
		struct idi_peripheral_driver *driver)
{
	const struct idi_device_id *ids = driver->id_table;

	if (ids == NULL)
		return NULL;
	while (ids->vendor) {
		if (idi_match_one_device(ids, peripheral))
			return ids;
		ids++;
	}

	return NULL;
}

/**
 * idi_bus_match - match the device id to a driver id
 * @device: device to attempt to match to a driver id
 * @driver: driver to match to.
 *
 * This may need to be a bit more sophisticated.  The ID may be a better
 * thing to use.  FIXME: id table matching?
 **/
static int idi_bus_match(struct device *device, struct device_driver *driver)
{
	struct idi_peripheral_device *peripheral =
	    idi_verify_peripheral(device);
	struct idi_client_device *client = idi_verify_client(device);
	struct idi_peripheral_driver *p_drv = NULL;
	int client_driver = 0;
#ifdef CONFIG_OF
	const struct of_device_id *match;
#endif
	const struct idi_device_id *found_id;

	pr_debug("%s: ", __func__);

	/*
	 * If we have a controller device, ignore it.
	 */
	if ((peripheral == NULL) && (client == NULL)) {
		pr_debug("%s:controller matched\n", __func__);
		return 0;
	}


#ifdef CONFIG_OF
	/*
	 * If we have a client driver, note that fact.
	 */
	match = of_match_device(driver->of_match_table, device);
	if (match) {
		client_driver = 1;
		pr_debug("%s:Got a match - device %p, driver %p\n", __func__,
			 device, driver);
	}
#else

	if (client)
		client_driver = 1;
#endif

	if (peripheral && (client_driver == 0)) {
		pr_debug("%s:for device: %s  driver: %s\n", __func__,
			 dev_name(device), driver->name);
		p_drv = to_idi_peripheral_driver(driver);

		found_id = idi_match_device(peripheral, p_drv);
		if (found_id) {
			pr_debug("%s: IDI ID match, %x:%x:%x!\n",
					__func__,
					found_id->vendor,
					found_id->device,
					found_id->subdevice);
			goto success;
		}

		/*
		 * Make sure the device type and the driver type match on the
		 * IDI level.
		 */
		if (peripheral->p_type != p_drv->p_type) {
			pr_debug("%s:no match for peripheral type %d\n",
					__func__, peripheral->p_type);
			return 0;
		}
		pr_warn("%s:%s-%s: Using peripheral type for registration process is deprecated\n",
				__func__, dev_name(device), driver->name);
		pr_warn("%s:%s-%s: Please idi device id and driver id table\n",
				__func__, dev_name(device), driver->name);
success:
		pr_debug("%s:SUCCESS : device: %s driver: %s\n", __func__,
			 dev_name(device), driver->name);

		return 1;
	}

	/*
	 * Only one client
	 */
	if (client && client_driver) {
		pr_debug("%s:Success for device: %s  driver: %s\n", __func__,
			 dev_name(device), driver->name ? driver->name : "");

		return 1;
	}

	return 0;

}				/* idi_bus_match() */

static int idi_bus_probe(struct device *dev)
{
	struct idi_peripheral_device *peripheral = idi_verify_peripheral(dev);
	struct idi_client_device *client = idi_verify_client(dev);
	struct idi_peripheral_driver *p_drv;
	struct idi_client_driver *c_drv;
	int status = 0;

	pr_debug("%s:device %p, client %p, peripheral %p\n", __func__, dev,
		 client, peripheral);

	if ((peripheral == NULL) && (client == NULL)) {
		pr_debug("%s:device = %p not a peripheral/client device\n",
			 __func__, dev);
		return 0;
	}

	if (!dev->driver) {
		pr_debug("%s:no driver found\n", __func__);
		return -ENODEV;
	}

	/*
	 * We need to suport two different devices here that have different
	 * capabilites.
	 */
	if (client) {
		c_drv = to_idi_client_driver(dev->driver);
		if (!c_drv->probe) {
			pr_debug("%s:no client driver probe\n", __func__);
			return -ENODEV;
		}
		client->driver = c_drv;

		status = c_drv->probe(client, c_drv->id_table);
		if (status)
			client->driver = NULL;

		return status;
	}

	if (peripheral) {
		p_drv = to_idi_peripheral_driver(dev->driver);
		if (!p_drv->probe) {
			pr_debug("%s:no driver probe\n", __func__);
			return -ENODEV;
		}
		peripheral->driver = p_drv;

		status = p_drv->probe(peripheral, p_drv->id_table);
		if (status)
			peripheral->driver = NULL;

		return status;
	}

	return -ENODEV;

}				/* idi_bus_probe() */

/**
 * idi_bus_remove - remove a driver from a device.
 * @dev: the device to remove the driver from.
 */
static int idi_bus_remove(struct device *dev)
{
	struct idi_peripheral_device *peripheral = idi_verify_peripheral(dev);
	struct idi_client_device *client = idi_verify_client(dev);
	struct idi_peripheral_driver *p_drv;
	struct idi_client_driver *c_drv;
	int status = 0;

	pr_debug("%s: device = %p\n", __func__, dev);

	if (((peripheral == NULL) && (client == NULL))
					|| (dev->driver == NULL)) {
		if (dev->driver == NULL)
			pr_debug("%s: no driver\n", __func__);

		return 0;
	}

	if (peripheral) {
		p_drv = to_idi_peripheral_driver(dev->driver);
		pr_debug("%s: peripheral = %d\n", __func__, p_drv->p_type);
		if (p_drv->remove) {
			pr_debug("%s: calling driver remove\n", __func__);
			dev_dbg(dev, "remove\n");
			status = p_drv->remove(peripheral);
		} else {
			pr_debug("%s: no driver remove\n", __func__);
			dev->driver = NULL;
			status = 0;
		}

		if (status == 0)
			peripheral->driver = NULL;

		return status;
	}

	if (client) {
		c_drv = to_idi_client_driver(dev->driver);
		pr_debug("%s: client %s\n", __func__, dev_name(dev));
		if (c_drv->remove) {
			dev_dbg(dev, "%s: driver remove\n", __func__);
			status = c_drv->remove(client);
		} else {
			pr_debug("%s: no driver remove\n", __func__);
			dev->driver = NULL;
			status = 0;
		}

		if (status == 0)
			client->driver = NULL;

		return status;
	}

	return -ENODEV;

}				/* idi_bus_remove() */

/**
 * idi_bus_shudown - Call the peripheral driver shutdown function if it exists.
 * @dev: device getting shutdown.
 */
static void idi_bus_shutdown(struct device *dev)
{
	struct idi_peripheral_device *peripheral = idi_verify_peripheral(dev);
	struct idi_client_device *client = idi_verify_client(dev);
	struct idi_peripheral_driver *p_drv;
	struct idi_client_driver *c_drv;

	pr_debug("%s: device = %p\n", __func__, dev);

	if (((peripheral == NULL) && (client == NULL))
					|| (dev->driver == NULL)) {
		pr_debug("%s: controller not shutdown\n", __func__);
		return;
	}

	if (peripheral) {
		p_drv = to_idi_peripheral_driver(dev->driver);
		pr_debug("%s: peripheral = %d\n", __func__, p_drv->p_type);
		if (p_drv->shutdown) {
			pr_debug("%s: calling driver shutdown\n", __func__);
			p_drv->shutdown(peripheral);
		}
	}

	if (client) {
		c_drv = to_idi_client_driver(dev->driver);
		pr_debug("%s: client = %p\n", __func__, client);
		if (c_drv->shutdown) {
			pr_debug("%s: calling driver shutdown\n", __func__);
			c_drv->shutdown(client);
		}
	}

}				/* idi_bus_shutdown() */

struct bus_type idi_bus_type = {
	.name = "idi",
	.dev_attrs = idi_dev_attrs,
	.match = idi_bus_match,
	.uevent = idi_bus_uevent,
	.probe = idi_bus_probe,
	.remove = idi_bus_remove,
	.shutdown = idi_bus_shutdown,
};

/**
 * idi_controller_release - Release the controller device structure
 * @dev: controller struct device pointer
 *
 * When the reference count on the controller device reaches zero, this
 * function will be called to release the allocated memory.
 *
 * This could be combined into one release function and just check the type.
 * Not sure if that would be better.
 *
 */
static void idi_controller_release(struct device *dev)
{
	struct idi_controller_device *controller;

	controller = to_idi_controller_device(dev);

	pr_debug("%s: controller = %p\n", __func__, controller);

	kfree(controller);

}				/* idi_controller_release() */

/**
 * idi_peripheral_release - Release peripheral device structure
 * @dev: peripheral struct device pointer
 *
 * When the reference count on the peripheral device reaches zero, this
 * function will be called to release the allocated memory.
 *
 */
static void idi_peripheral_release(struct device *dev)
{
	struct idi_peripheral_device *peripheral;

	peripheral = to_idi_peripheral_device(dev);

	pr_debug("%s: perpheral = %p\n", __func__, peripheral);

	kfree(peripheral->resources.resource);
	kfree(peripheral);

}				/* idi_peripheral_release() */

/**
 * idi_client_release - Release the client device structure
 * @dev: client struct device pointer
 *
 * When the reference count on the client device reaches zero, this function
 * will be called to release the allocated memory.
 *
 */
static void idi_client_release(struct device *dev)
{
	struct idi_client_device *client;

	client = to_idi_client_device(dev);

	pr_debug("%s: client = %p\n", __func__, client);
	kfree(client->idi_res.resource);
	kfree(client);

}				/* idi_client_release() */

int idi_add_client_device(int busnum, struct idi_client_device_info const *info)
{
	int status = 0;
	struct _idi_client_device_info *devinfo;
	struct idi_controller_device *idi;
	IDI_BUS_ENTER;
	down_write(&__idi_client_device_lock);

	devinfo = kzalloc(sizeof(*devinfo), GFP_KERNEL);
	if (!devinfo) {
		pr_debug("idi-bus: can't allocate client device info!\n");
		status = (-ENOMEM);
		goto no_mem;
	}

	devinfo->busnum = busnum;
	devinfo->info = *info;
	list_add_tail(&devinfo->list, &__idi_client_list);
no_mem:
	up_write(&__idi_client_device_lock);

	idi = idi_get_controller(busnum);
	if (idi)
		idi_scan_static_client(idi);

	return status;
}
EXPORT_SYMBOL(idi_add_client_device);


/* #if 0 code changed to #ifdef CODE_CLEANUP, so all code within is DISABLED */
#ifdef CODE_CLEANUP
int __init
idi_register_peripheral_info(int busnum,
			     struct idi_peripheral_device_info const *info,
			     unsigned len)
{
	 /*TODO*/ return 0;
}

static void idi_scan_static_peripheral_info(struct idi_client_device *client)
{
 /*TODO*/}

#endif

/**
 * idi_new_peripheral_device - instantiate an idi peripheral device
 * @client: The idi client device managing the device
 * @info: describes one idi peripheral device
 *
 * Create an idi peripheral device.
 * Binding is handled through the driver model.
 * This call is not appropriate for use by mainboard initialization logic,
 * which usually runs during an arch_initcall() long
 * before the idi client device is available
 *
 * This returns the new idi peripheral device; which may be saved for
 * later use with idi_unregister_peripheral_device();
 * or NULL to indicate an error
 */

struct idi_peripheral_device *idi_new_peripheral_device(
					struct idi_client_device *client,
					struct idi_peripheral_device_info
					const *info)
{
	struct idi_peripheral_device *p_device;
	struct idi_controller_device *controller;
	unsigned long flags;
	int status;

	controller = to_idi_controller_device(client->device.parent);

	p_device = kzalloc(sizeof(*p_device), GFP_KERNEL);
	if (!p_device)
		return NULL;

	p_device->device.parent = &client->device;
	p_device->device.dma_mask = client->device.dma_mask;
	p_device->device.coherent_dma_mask = client->device.coherent_dma_mask;

	p_device->p_type = info->p_type;
	p_device->id = info->id;
	p_device->channel_map = info->channel_map;
	p_device->pm_platdata = info->pm_platdata;
	p_device->sw_channel = IDI_MAX_CHANNEL;

	p_device->controller = controller;
	dev_set_name(&p_device->device, "%s-%d", info->name, controller->id);

	spin_lock_irqsave(&client->sw_lock, flags);
	list_add_tail(&p_device->link, &client->peripherals);
	spin_unlock_irqrestore(&client->sw_lock, flags);

	p_device->device.type = &idi_peripheral;
	p_device->device.bus = &idi_bus_type;
	p_device->device.release = idi_peripheral_release;
	p_device->device.platform_data = info->platform_data;
	p_device->device.of_node = info->of_node;

	p_device->resources = info->resources;

	status = device_register(&p_device->device);
	if (status)
		goto out_idi_new_peripheral;

	dev_dbg(&p_device->device,
		"peripheral [%s] registered to idi client %s\n",
		dev_name(&p_device->device), dev_name(&client->device));
	return p_device;

out_idi_new_peripheral:
	kfree(p_device);
	return NULL;
}
EXPORT_SYMBOL(idi_new_peripheral_device);

/**
 * idi_unregister_peripheral_device - reverse effect of idi_new_peripheral_device()
 * @peripheral: value returned from idi_new_peripheral_device()
 *
 */
void idi_unregister_peripheral_device(struct idi_peripheral_device *peripheral)
{

	device_unregister(&peripheral->device);
}
EXPORT_SYMBOL(idi_unregister_peripheral_device);

/**
 * idi_get_resource_byname - get a resource for an idi device by name
 * @res: idi resource
 * @type: resource type
 * @name: resource name
 * Borrowed from platform_get_resource_byname()
 */
struct resource *idi_get_resource_byname(struct idi_resource *idi_res,
					 unsigned int type, const char *name)
{
	int i;

	for (i = 0; i < idi_res->num_resources; i++) {
		struct resource *r = &idi_res->resource[i];

		if (unlikely(!r->name))
			continue;

		if (type == resource_type(r) && !strcmp(r->name, name))
			return r;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(idi_get_resource_byname);

/**
 * idi_alloc_controller - Allocate an HSI controller and do some initialiation.
 * @extra: extra size to host private controller data
 * @dev: Parent device
 *
 * Return NULL on failure or a pointer to an hsi_controller on success.
 *
 * NOTE: the controller device data structure is freed using the device release
 * function.  This means that the device driver shouldn't free the memory
 * directly.  The device subsystem will call the release function when all
 * references to the device are done.
 */
struct idi_controller_device *idi_alloc_controller(int extra,
						   struct device *dev)
{
	struct idi_controller_device *idi;
	int err;

	pr_debug("%s:\n", __func__);

	idi = kzalloc(sizeof(struct idi_controller_device) + extra, GFP_KERNEL);

	if (!idi)
		return NULL;

	/* FIXME: Remove the idr during device release */

	idr_preload(GFP_KERNEL);
	spin_lock(&idi_bus_lock);
	err = idr_alloc(&idi_bus_idr, idi, 0, 0, GFP_NOWAIT);
	if (err >= 0)
		idi->id = err;
	spin_unlock(&idi_bus_lock);
	idr_preload_end();
	if (err)
		goto free;

	/*
	 * FIXME: Should probably set these to null routines rather
	 * than NULL.
	 */

	idi->request_access = NULL;
	idi->release_access = NULL;
	idi->async = NULL;
	idi->setup = NULL;
	idi->flush = NULL;
	idi->start_tx = NULL;
	idi->stop_tx = NULL;
	idi->release = NULL;

	idi->device.parent = dev;
	idi->device.dma_mask = dev->dma_mask;
	idi->device.coherent_dma_mask = dev->coherent_dma_mask;
	dev_set_name(&idi->device, "idi-%d", idi->id);

	return idi;
free:
	kfree(idi);
	return NULL;

}				/* idi_alloc_controller() */
EXPORT_SYMBOL_GPL(idi_alloc_controller);

/**
 * idi_unregister_controller - Unregister an IDI controller
 * @idi: idi device to unregister.
 */
void idi_unregister_controller(struct idi_controller_device *idi)
{
	pr_debug("%s:\n", __func__);

	if (idi == NULL) {
		pr_debug("null idi\n");
		return;
	}

	/*
	 * The "client" is a pointer.
	 */
	if (idi->client)
		device_unregister(idi->client);

	device_unregister(&idi->device);
}
EXPORT_SYMBOL_GPL(idi_unregister_controller);

/**
 * idi_register_client_device - Regiser a client device.
 * @c_device: A client device to register.
 *
 */
static int idi_register_client_device(struct idi_client_device *c_device)
{
	pr_debug("%s: client = %p  dev = %p\n", __func__, c_device,
		 &c_device->device);

	c_device->device.type = &idi_client;
	c_device->device.bus = &idi_bus_type;
	if (c_device->device.of_node == NULL)
		c_device->device.of_node = &idi_client_info;
	c_device->device.release = idi_client_release;

	return device_register(&c_device->device);

}				/* idi_register_client_device() */

static int _idi_add_client_device(struct idi_client_device_info const *info,
				  struct idi_controller_device *idi)
{
	int status = 0;
	struct idi_client_device *c_device;

	IDI_BUS_ENTER;
	c_device = kzalloc(sizeof(*c_device), GFP_KERNEL);
	if (c_device == NULL)
		return -ENOMEM;

	c_device->idi_res = info->idi_res;
	memcpy(c_device->channels, info->channels, sizeof(info->channels));

	c_device->device.of_node = info->of_node;
	c_device->device.parent = &idi->device;
	c_device->device.dma_mask = idi->device.dma_mask;
	c_device->device.coherent_dma_mask = idi->device.coherent_dma_mask;
	INIT_LIST_HEAD(&c_device->peripherals);
	dev_set_name(&c_device->device, "idi_client-%d", idi->id);

	spin_lock_init(&c_device->sw_lock);

	status = idi_register_client_device(c_device);
	if (status != 0) {
		kfree(c_device);
		pr_debug("%s: failed to create client device\n", __func__);
	} else {
		idi->client = &c_device->device;
	}

	return status;
}

static void idi_scan_static_client(struct idi_controller_device *idi)
{
	struct _idi_client_device_info *devinfo;

	IDI_BUS_ENTER;
	down_write(&__idi_client_device_lock);
	list_for_each_entry(devinfo, &__idi_client_list, list) {
		/*FIXME: We assume that one client per controller
		 * do sanity check earlier
		 */
		if ((devinfo->busnum == idi->id) &&
		    _idi_add_client_device(&devinfo->info, idi)) {
			pr_debug("Add client to idi controller failed\n");
		}
	}

	up_write(&__idi_client_device_lock);
}

/*
 * Function:
 *    idi_register_controller()
 *
 * Description:
 *    Register all devices needed for IDI module.  An IDI device has a "master"
 *    (device) and a "slave" (client) associated with it, so we create both
 *    here.
 *
 * Parameters:
 *    idi    A controller instance.
 */
int idi_register_controller(struct idi_controller_device *idi)
{
	int err = 0;

	pr_debug("%s: idi: %p  dev: %p\n", __func__, idi, &idi->device);
	idi_debug_add_event(controller_dev_reg, 0, NULL);

	idi->device.type = &idi_ctrl;
	idi->device.bus = &idi_bus_type;
	idi->device.release = idi_controller_release;

	err = device_register(&idi->device);
	if (err != 0)
		return err;
	/*
	 * Scan any client device already "added" for registration to the bus
	 */
	if (!idi->client)
		idi_scan_static_client(idi);

	return err;

}				/* idi_register_controller() */
EXPORT_SYMBOL(idi_register_controller);

struct idi_controller_device *idi_get_controller(int nr)
{
	struct idi_controller_device *idi;
	spin_lock(&idi_bus_lock);
	idi = idr_find(&idi_bus_idr, nr);
	spin_unlock(&idi_bus_lock);

	return idi;
}
EXPORT_SYMBOL(idi_get_controller);

struct idi_client_device *idi_get_client(int nr)
{
	struct idi_controller_device *controller = idi_get_controller(nr);
	if (controller)
		return to_idi_client_device(controller->client);
	return NULL;
}
EXPORT_SYMBOL(idi_get_client);

/**
 * idi_register_peripheral_driver - Register a peripheral driver
 * @driver - reference to the idi client driver.
 *
 */
int idi_register_peripheral_driver(struct idi_peripheral_driver *driver)
{
	int err = 0;

	pr_debug("%s:\n", __func__);

	if (driver == NULL)
		return -EINVAL;

	idi_debug_add_event(peripheral_drv_reg, driver->p_type, NULL);

	if (((driver->p_type == IDI_INTERNAL) ||
	    (driver->p_type == IDI_MAX_PERIPHERAL))
	    && (driver->id_table == NULL))
		return -EINVAL;

	driver->driver.bus = &idi_bus_type;

	err = driver_register(&driver->driver);
	if (err == 0)
		pr_debug("%s: peripheral driver %s registered\n", __func__,
			 driver->driver.name);
	else
		pr_debug("%s: error %d\n", __func__, err);

	return err;

}				/* idi_register_peripheral_driver() */
EXPORT_SYMBOL(idi_register_peripheral_driver);

/**
 * idi_unregister_peripheral_driver - Unregister peripheral idi device driver
 * @driver - reference to the idi peripheral device driver.
 */
void idi_unregister_peripheral_driver(struct idi_peripheral_driver *driver)
{
	if (driver == NULL)
		return;

	pr_debug("%s: %s\n", __func__, driver->driver.name);

	driver_unregister(&driver->driver);

}				/* idi_unregister_peripheral_driver() */
EXPORT_SYMBOL(idi_unregister_peripheral_driver);

/*
 * Function:
 *    idi_register_client_driver()
 *
 * Description:
 *    Register an IDI client driver.  Special case peripheral driver.
 *
 * Parameters:
 *    @driver - reference to the idi client driver.
 */
int idi_register_client_driver(struct idi_client_driver *driver)
{
	int err = 0;

	pr_debug("%s:\n", __func__);

	idi_debug_add_event(client_drv_reg, 0, NULL);

	if (driver == NULL)
		return -EINVAL;

	driver->driver.bus = &idi_bus_type;

	err = driver_register(&driver->driver);
	if (err == 0) {
		pr_debug("%s: client driver %s registered\n", __func__,
			 driver->driver.name);
	} else {
		pr_debug("%s: error %d\n", __func__, err);
	}

	return err;

}				/* idi_register_client_driver() */
EXPORT_SYMBOL(idi_register_client_driver);

/**
 * idi_unregister_client_driver - Unregister client idi device driver
 * @driver - reference to the idi client device driver.
 */
void idi_unregister_client_driver(struct idi_client_driver *driver)
{
	if (driver == NULL)
		return;

	pr_debug("%s: %s\n", __func__, driver->driver.name);

	driver_unregister(&driver->driver);

}				/* idi_unregister_client_driver() */
EXPORT_SYMBOL(idi_unregister_client_driver);

static int idi_start_rx(struct idi_peripheral_device *peripheral, void *data)
{
	struct idi_peripheral_driver *driver = peripheral->driver;

	pr_debug("%s:\n", __func__);
	idi_debug_add_event(start_rx, peripheral->p_type, NULL);

	if (driver && driver->idi_start_rx)
		(*driver->idi_start_rx) (peripheral);

	return 0;

}				/* idi_start_rx() */

static int idi_stop_rx(struct idi_peripheral_device *peripheral, void *data)
{
	struct idi_peripheral_driver *driver = peripheral->driver;

	pr_debug("%s:\n", __func__);
	idi_debug_add_event(stop_rx, peripheral->p_type, NULL);

	if (driver && driver->idi_stop_rx)
		(*driver->idi_stop_rx) (peripheral);

	return 0;

}				/* idi_stop_rx() */

static void idi_for_each_peripheral(struct idi_client_device *client,
		void *data,
		int (*fn) (struct idi_peripheral_device *peripheral,
			void *data))
{
	struct idi_peripheral_device *peripheral;
	unsigned long flags;

	pr_debug("%s:\n", __func__);

	spin_lock_irqsave(&client->sw_lock, flags);
	list_for_each_entry(peripheral, &client->peripherals, link) {
		spin_unlock_irqrestore(&client->sw_lock, flags);
		(*fn) (peripheral, data);
		spin_lock_irqsave(&client->sw_lock, flags);
	}
	spin_unlock_irqrestore(&client->sw_lock, flags);


}				/* idi_for_each_peripheral() */

/**
 * idi_event - Notifies peripheral devices about events.
 * @event: The event type
 *
 * Peripherals should not be concerned about wake line behavior. However, due
 * to a race condition in HSI HW protocol, peripherals need to be notified
 * about wake line changes, so they can implement a workaround for it.
 *
 * Events:
 * IDI_EVENT_START_RX - Incoming wake line high
 * IDI_EVENT_STOP_RX - Incoming wake line down
 */
void idi_event(struct idi_controller_device *idi, unsigned int event)
{
	struct idi_client_device *client;
	int (*fn) (struct idi_peripheral_device *peripheral, void *data);

	pr_debug("%s:\n", __func__);


	if (!idi || (!idi->client))
		return;

	client = to_idi_client_device(idi->client);

	switch (event) {
	case IDI_EVENT_START_RX:
		fn = idi_start_rx;
		break;
	case IDI_EVENT_STOP_RX:
		fn = idi_stop_rx;
		break;
	default:
		return;
	}
	idi_for_each_peripheral(client, NULL, fn);

}
EXPORT_SYMBOL_GPL(idi_event);

static int __init idi_bus_register(void)
{
	pr_debug("%s:\n", __func__);

	idi_debug_init();

	return bus_register(&idi_bus_type);

}				/* idi_bus_register() */

static void idi_bus_exit(void)
{
	pr_debug("%s:\n", __func__);

	idi_debug_remove();

	bus_unregister(&idi_bus_type);

}				/* idi_bus_exit() */

postcore_initcall(idi_bus_register);

/*module_init(idi_bus_register);*/
module_exit(idi_bus_exit);
