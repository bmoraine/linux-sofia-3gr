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
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_bus.h>

#define CHANNEL_MAP_PROPERTY_NAME "idi-channels"
#define PERIPH_TYPE_PROPERTY_NAME "idi-peripheral-type"

#ifdef CONFIG_OF

static struct idi_channel_type_lut {
	enum idi_channel_type type;
	const char *property_value;
} channel_type_lut[] = {
	{ .type = RESERVED, .property_value = "reserved"},
	{ .type = SOFTWARE_DEFINED, .property_value = "software"},
	{ .type = REGISTER_ACCESS, .property_value = "register"},
	{ .type = FILE_CONTROL, .property_value = "flow"},
	{ .type = OUTSTANDING_READ, .property_value = "outstanding"},
	{ .type = STREAM, .property_value = "stream"},
	{ .type = FILE, .property_value = "dma"},
	{ .type = SIGNAL_FORWARDING, .property_value = "signal"},
	{ .type = INTERRUPTS, .property_value = "interrupts"},
	{ .type = ERRORS, .property_value = "errors"},
};

static struct of_device_id idi_client_ids[] = {
	{.compatible = "intel,idi,client"},
	{.compatible = "intel,idi,abb"},
	{.compatible = "intel,abb"},
	{},
};

int of_idi_populate_channels_map(struct device_node *np,
				 enum idi_channel_type *channels)
{
	int ret, i, j;
	unsigned nr_channels;

	memset(channels, RESERVED, sizeof(channels));

	nr_channels = of_property_count_strings(np, CHANNEL_MAP_PROPERTY_NAME);
	if (IS_ERR_VALUE(nr_channels) || !nr_channels) {
		pr_err(" %s property not found in device node %s\n",
		       CHANNEL_MAP_PROPERTY_NAME, np->name);
		return -EINVAL;
	}

	if (nr_channels > IDI_MAX_CHANNEL)
		return -EINVAL;

	for (i = 0; i < nr_channels; i++) {
		const char *ch_type;

		ret =
		    of_property_read_string_index(np, CHANNEL_MAP_PROPERTY_NAME,
						  i, &ch_type);
		if (ret)
			return ret;

		for (j = 0; j < ARRAY_SIZE(channel_type_lut); j++) {
			if (strcmp(channel_type_lut[j].property_value, ch_type)
			    == 0) {
				channels[i] = channel_type_lut[j].type;
				break;
			}
		}

	}

	return 0;
}
EXPORT_SYMBOL(of_idi_populate_channels_map);

int of_idi_resource_populate(struct device_node *np,
			     struct idi_resource *idi_res)
{
	struct resource *res, temp_res;
	int rc, i, num_reg = 0, num_irq;

	/* count the io and irq resources */
	if (of_can_translate_address(np))
		while (of_address_to_resource(np, num_reg, &temp_res) == 0)
			num_reg++;
	num_irq = of_irq_count(np);

	/* Populate the resource table */
	if (num_irq || num_reg) {
		res = kzalloc(sizeof(*res) * (num_irq + num_reg), GFP_KERNEL);
		if (!res)
			return -ENOMEM;

		idi_res->num_resources = num_reg + num_irq;
		idi_res->resource = res;

		for (i = 0; i < num_reg; i++, res++) {
			rc = of_address_to_resource(np, i, res);
			WARN_ON(rc);
		}
		WARN_ON(of_irq_to_resource_table(np, res, num_irq) != num_irq);
	}

	return 0;
}
EXPORT_SYMBOL(of_idi_resource_populate);

static int __init idi_client_device_register(struct device_node *np)
{
	int ret = 0;
	struct idi_client_device_info info;

	info.of_node = np;

	ret = of_idi_resource_populate(np, &info.idi_res);
	if (ret)
		return -EINVAL;

	ret = of_idi_populate_channels_map(np, info.channels);
	if (ret)
		return -EINVAL;

	ret = idi_add_client_device(0, &info);
	if (ret) {
		pr_debug("Add ABB client device failed\n");
		return -EINVAL;
	}

	return 0;
}

static struct of_device_id idi_peripheral_ids[] = {
	{.compatible = "intel,idi,peripheral"},
	{},
};

struct idi_peripheral_type_lut {
	enum idi_peripheral_type type;
	const char *property_value;
};

static struct idi_peripheral_type_lut type_lut[] = {
	{.type = IDI_FMR, .property_value = "intel,idi,fmr"},
	{.type = IDI_AFE, .property_value = "intel,idi,afe"},
	{.type = IDI_BT, .property_value = "intel,idi,bt"},
	{.type = IDI_BT_STREAM, .property_value = "intel,idi,bt-stream"},
	{.type = IDI_I2C, .property_value = "intel,idi,i2c"},
	{.type = IDI_WLAN, .property_value = "intel,idi,wlan"},
	{.type = IDI_GNSS, .property_value = "intel,idi,gnss"},
	{.type = IDI_RTC, .property_value = "intel,idi,rtc"},
	{.type = IDI_MEAS, .property_value = "intel,idi,meas"},
	{.type = IDI_BAT, .property_value = "intel,idi,bat_hal"},
	{.type = IDI_CHG, .property_value = "intel,idi,fan54x"},
	{.type = IDI_CHG, .property_value = "intel,idi,smb345"},
	{.type = IDI_CCD, .property_value = "intel,idi,fg_hal"},
	{.type = IDI_BNT, .property_value = "intel,idi,brownout"},
	{.type = IDI_ERROR, .property_value = "intel,idi,error"},
};

int of_idi_peripheral_find_channels_map(struct device_node *np)
{
	int channel, ret;
	struct property *prop;
	unsigned channels[3];
	unsigned nr_channels;

	memset(channels, 0, sizeof(channels));

	prop = of_find_property(np, CHANNEL_MAP_PROPERTY_NAME, &nr_channels);
	if (!prop) {
		pr_err("%s property not found in device node %s\n",
		       CHANNEL_MAP_PROPERTY_NAME, np->name);
		return -EINVAL;
	}
	nr_channels /= sizeof(unsigned);

	if (nr_channels > ARRAY_SIZE(channels)) {
		pr_err("%s property is invalid on node %s\n",
				CHANNEL_MAP_PROPERTY_NAME, np->name);
		 pr_err("\tMax channels supported by peripheral is %d\n",
				       ARRAY_SIZE(channels));
		return -EINVAL;

	}

	ret = of_property_read_u32_array(np,
					 CHANNEL_MAP_PROPERTY_NAME,
					 channels, nr_channels);
	if (ret) {
		pr_err("%s property reading error in device node %s\n",
		       CHANNEL_MAP_PROPERTY_NAME, np->name);
		return -EINVAL;
	}
	channel = (channels[0] & IDI_CHANNEL_MASK)
	    | ((channels[1] & IDI_CHANNEL_MASK) << IDI_SECONDARY_SHIFT)
	    | ((channels[2] & IDI_CHANNEL_MASK) << IDI_TERTIARY_SHIFT);

	return channel;
}
EXPORT_SYMBOL(of_idi_peripheral_find_channels_map);

static unsigned idi_peripheral_of_get_id(struct device_node *np,
							const char *name)
{
	struct property *prop;
	unsigned id_value;

	prop = of_find_property(np, name, &id_value);
	if (of_property_read_bool(np, name) == false)
		return IDI_ANY_ID;

	of_property_read_u32(np, name, &id_value);
	pr_debug("Device %s, idi-id-%s : %x\n", np->name, name, id_value);
	return id_value;
}

static void idi_peripheral_of_get_device_id(struct device_node *np,
		struct idi_device_id *id)
{
	id->vendor = idi_peripheral_of_get_id(np, "idi-id-vendor");
	id->device = idi_peripheral_of_get_id(np, "idi-id-device");
	id->subdevice = idi_peripheral_of_get_id(np, "idi-id-subdevice");
	if ((id->vendor == IDI_ANY_ID) && (id->device == IDI_ANY_ID)
		&& (id->subdevice == IDI_ANY_ID))
		memset(id, 0, sizeof(struct idi_device_id));
}

static enum idi_peripheral_type idi_peripheral_find_type(struct device_node *np)
{
	int i;
	const char *type;
	if (of_property_read_string(np, PERIPH_TYPE_PROPERTY_NAME, &type)) {
		pr_err("%s property not found in device node %s\n",
		       PERIPH_TYPE_PROPERTY_NAME, np->name);
		return IDI_MAX_PERIPHERAL;
	}

	for (i = 0; i < ARRAY_SIZE(type_lut); i++) {
		if (strcmp(type_lut[i].property_value, type) == 0)
			return type_lut[i].type;
	}

	return IDI_MAX_PERIPHERAL;
}

static int __init idi_peripheral_device_register(struct device_node *np)
{
	int status;
	struct idi_peripheral_device_info *info;
	struct idi_peripheral_device *p_device;
	struct idi_client_device *client;

	pr_debug("Parsing idi peripheral %s\n", np->name);

	client = idi_get_client(0);
	if (!client) {
		pr_err("IDI Client device not found !\n");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct idi_peripheral_device_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	/* Get PM platform data */
	info->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(info->pm_platdata)) {
		pr_err("%s: Miss PM info\n", info->name);
		info->pm_platdata = NULL;
	}

	/* Get peripheral type */
	info->p_type = idi_peripheral_find_type(np);
	if (info->p_type == IDI_MAX_PERIPHERAL) {
		pr_err("%s: Could not get valid type\n", info->name);
		status = -EINVAL;
		goto free_peripheral;
	}

	/* Extract IDI device id information */
	idi_peripheral_of_get_device_id(np, &info->id);

	status = of_idi_peripheral_find_channels_map(np);
	if (status < 0) {
		pr_err("%s: Could not get valid channel map\n", info->name);
		goto free_peripheral;
	}

	info->channel_map = status;

	/* Get peripheral name */
	strlcpy(info->name, np->name, IDI_PERIPH_MAX_NAME_SIZE);

	/*
	 * From of_device_alloc()
	 */
	status = of_idi_resource_populate(np, &info->resources);
	if (status)
		goto free_peripheral;

	info->of_node = of_node_get(np);

	p_device = idi_new_peripheral_device(client, info);
	if (!p_device) {
		pr_debug("New IDI peripheral device allocation failed !\n");
		status = -EINVAL;
		goto free_resources;
	}
	goto free_peripheral;

free_resources:
	kfree(info->resources.resource);
free_peripheral:
	kfree(info);
	return status;
}

static int __init idi_of_populate(void)
{
	int i, ret;
	struct device_node *np, *from = NULL;

	struct {
		struct of_device_id *ids;
		int (*populate) (struct device_node *np);
	} idi_bus_populate[] = {
		{
			.ids = idi_client_ids,
			.populate = idi_client_device_register,
		},
		{
			.ids = idi_peripheral_ids,
			.populate = idi_peripheral_device_register,
		},
	};

	for (i = 0; i < ARRAY_SIZE(idi_bus_populate); i++) {
		while ((np =
			of_find_matching_node(from,
					      idi_bus_populate[i].ids)) !=
		       NULL) {
			ret = idi_bus_populate[i].populate(np);
			if (ret) {
				pr_err("Populating IDI node %s failed!\n",
								np->name);
/*				return ret; */
			}
			from = np;
		}
	}
	return 0;
}

#else
struct idi_plat_info_list {
	struct list_head node;
	struct idi_peripheral_device_info *info;
};

static LIST_HEAD(idi_list);

void idi_client_device_register(struct idi_client_device_board_info *info)
{
	if (idi_add_client_device(0, (struct idi_client_device_info *)info))
		pr_debug("Add ABB client device failed\n");
}

static int __init idi_of_populate(void)
{
	int ret = 0;
	struct idi_plat_info_list *idi_data;
	struct idi_client_device *client;
	struct idi_peripheral_device *p_device;

	client = idi_get_client(0);
	if (!client) {
		pr_debug("IDI Client device not found !\n");
		return -ENODEV;
	}

	list_for_each_entry(idi_data, &idi_list, node) {
		p_device = idi_new_peripheral_device(client, idi_data->info);
		if (!p_device) {
			pr_debug("New IDI peripheral device allocation failed !\n");
			ret = -EINVAL;
		}

#ifdef CONFIG_PLATFORM_DEVICE_PM
		ret = idi_device_pm_set_class(p_device);
		if (ret)
			pr_err("IDI device %d PM registration failed\n",
							p_device->p_type);
#endif
	}
	return ret;
}

void idi_peripheral_device_register(struct idi_peripheral_device_info *info)
{
	struct idi_plat_info_list *idi_data;

	idi_data = kzalloc(sizeof(struct idi_plat_info_list), GFP_KERNEL);
	if (WARN_ON(!idi_data))
		return;

	if (info) {
		idi_data->info = info;
		list_add_tail(&idi_data->node, &idi_list);
	}
}

#endif

device_initcall(idi_of_populate);
