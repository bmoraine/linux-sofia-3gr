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

#ifndef _IDI_BUS_H
#define _IDI_BUS_H

/* IDI peripheral event codes */
enum {
	IDI_EVENT_START_RX,
	IDI_EVENT_STOP_RX,
};

struct idi_peripheral_driver;
struct idi_peripheral_device;
struct idi_peripheral_device_info;
/**
 * struct idi_client_device - IDI client device object (Virtual)
 * @flag: Any flags needed.
 * @device: The peripheral's device structure.
 * @driver: Pointer to the peripheral device driver
 * @irq_map:
 * @idi_resource: I/O and interrupts information
 * @channel_map:
 * @wr_register_chan: used for programed register writes
 * @rd_register_chan: used for programmed register_reads
 * @peripherals: list of associtated perpheral devices
 * @sw_lock:
 */
struct idi_client_device {
	u32 flags;
	struct device device;
	struct idi_client_driver *driver;
	struct idi_controller_device *controller;
	enum idi_channel_type channels[IDI_MAX_CHANNEL];

	/*   u32                     pm_current_state; */

	struct idi_resource idi_res;

	struct list_head peripherals;
	spinlock_t sw_lock;

	int wr_register_chan;
	int rd_register_chan;
	u32 chipid;
	/*
	 * should these be part of the driver?
	 */
	int (*open_software_channel) (struct idi_client_device *,
			struct idi_peripheral_device *);
	int (*close_software_channel) (struct idi_client_device *,
			struct idi_peripheral_device *);
	int (*software_read) (struct idi_client_device *,
			struct idi_peripheral_device *,
			struct idi_transaction *);
	int (*software_write) (struct idi_client_device *,
			struct idi_peripheral_device *,
			struct idi_transaction *);
	int (*set_addr_size) (struct idi_client_device *,
			struct idi_transaction *, u32, u32);
	void (*dump_channel)(struct idi_client_device *, unsigned, unsigned);

	int (*set_interrupt)(struct idi_client_device *,
			struct idi_transaction *);
	int (*flush_buffer)(struct idi_client_device *,
			struct idi_transaction *);
	int (*streaming_channel_flush)(struct idi_client_device *, u32);
	int (*ioread)(struct idi_client_device *, unsigned, unsigned *);
	int (*iowrite)(struct idi_client_device *, unsigned, unsigned);
};
struct idi_client_device_info {
	enum idi_channel_type channels[IDI_MAX_CHANNEL];
	struct idi_resource idi_res;
	struct device_node *of_node;
};
#define to_idi_client_device(_dev)  \
	container_of(_dev, struct idi_client_device, device)

/**
 * struct idi_client_driver - IDI client device driver
 * @flags:
 * @ptype:
 * @probe:
 * @remove:
 * @shutdown:
 * @suspend:
 * @resume:
 * @driver:
 * @device:
 * @id_table:
 * @ctrl_io:
 * @link:
 * @sw_lock:
 */
struct idi_client_driver {
	u32 flags;

	int (*probe) (struct idi_client_device *, const struct idi_device_id *);
	int (*remove) (struct idi_client_device *);

	/*
	 * driver model interfaces that don't relate to enumeration.
	 */
	void (*shutdown) (struct idi_client_device *);
	int (*suspend) (struct idi_client_device *, pm_message_t mesg);
	int (*resume) (struct idi_client_device *);

	struct device_driver driver;
	struct device *device;
	struct idi_device_id *id_table;

};
#define to_idi_client_driver(_drv)  \
	container_of(_drv, struct idi_client_driver, driver)

static inline void idi_controller_set_drvdata(struct idi_controller_device *idi,
					      void *data)
{
	dev_set_drvdata(&idi->device, data);
}

static inline void *idi_controller_get_drvdata(struct idi_controller_device
					       *idi)
{
	return dev_get_drvdata(&idi->device);
}

static inline void idi_client_set_drvdata(struct idi_client_device *client,
					  void *data)
{
	dev_set_drvdata(&client->device, data);
}

static inline void *idi_client_get_drvdata(struct idi_client_device *client)
{
	return dev_get_drvdata(&client->device);
}

int idi_add_client_device(int, struct idi_client_device_info const *);
int idi_register_client_driver(struct idi_client_driver *);
void idi_unregister_client_driver(struct idi_client_driver *);

struct idi_peripheral_device *idi_new_peripheral_device(
				struct idi_client_device *,
				struct idi_peripheral_device_info const *);

int idi_register_controller(struct idi_controller_device *);
void idi_unregister_controller(struct idi_controller_device *);
struct idi_controller_device *idi_get_controller(int);
struct idi_client_device *idi_get_client(int);
struct idi_controller_device *idi_alloc_controller(int extra, struct device *);
void idi_event(struct idi_controller_device *, unsigned int);
void idi_unregister_peripheral_device(struct idi_peripheral_device *);

#endif /* _IDI_BUS_H */
