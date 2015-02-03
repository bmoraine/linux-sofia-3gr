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

#ifndef _IDI_CONTROLLER_H
#define _IDI_CONTROLLER_H

struct idi_peripheral_device;

struct idi_controller_device {
	int id;
	struct device device;
	struct device *client;
	enum idi_channel_type channels[IDI_MAX_CHANNEL];

	int (*request_access) (struct idi_peripheral_device *);
	int (*release_access) (struct idi_peripheral_device *);

	int (*async) (struct idi_transaction *);
	int (*setup) (struct idi_controller_device *);
	int (*channel_flush) (struct idi_controller_device *, int , int);
	int (*flush) (struct idi_controller_device *);
	int (*start_tx) (struct idi_controller_device *);
	int (*stop_tx) (struct idi_controller_device *);
	int (*release) (struct idi_controller_device *);
	int (*xfer_complete)(struct idi_controller_device *, int, int);
	int (*set_channel_config) (struct idi_controller_device *,
				   struct idi_channel_config *,
				   int);
	int (*request_buffer_info) (struct idi_controller_device *, int, int);
	int (*request_buffer_flush) (struct idi_controller_device *, int);
	int (*request_queue_status) (struct idi_controller_device *, int, int);
	int (*set_power_state) (struct idi_controller_device *,
			struct idi_peripheral_device *, void *, char *, bool);
	unsigned long		private[0] ____cacheline_aligned;
};

static inline void *idi_controller_priv(struct idi_controller_device *idi)
{
	return (void *)idi->private;
}

#define to_idi_controller_device(_dev)  \
	container_of(_dev, struct idi_controller_device, device)

#endif /* _IDI_CONTROLLER_H */
