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

#ifndef _IDI_OF_H
#define _IDI_OF_H
int of_idi_populate_channels_map(struct device_node *, enum idi_channel_type *);
int of_idi_resource_populate(struct device_node *, struct idi_resource *);
int of_idi_peripheral_find_channels_map(struct device_node *);

#ifndef CONFIG_OF
struct idi_client_device_board_info {
	enum idi_channel_type channels[IDI_MAX_CHANNEL];
	struct idi_resource idi_res;
};

void idi_client_device_register(struct idi_client_device_board_info *);
void idi_peripheral_device_register(struct idi_peripheral_device_info *);
#endif

#endif /* _IDI_OF_H */

