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

#ifndef _IDI_IF_H_
#define _IDI_IF_H_

#include <linux/scatterlist.h>
#include <linux/mod_devicetable.h>
#define IDI_KERNEL_MODULE
#define IDI_POWER_API_V2
/*
 * IDI message t_type (bits)
 */
#define IDI_TRANS_READ    (0)
#define IDI_TRANS_WRITE   (1)

#define IDI_MAX_IRQ      (32)

#define IDI_MAX_CHANNEL  (16)
#define IDI_MAX_POWER_STATE 7
/*
 * Allow for peripheral devices to have more than one channel associated with
 * the device.
 */
#define IDI_PRIMARY_CHANNEL   BIT(0)
#define IDI_SECONDARY_CHANNEL BIT(1)
#define IDI_TERTIARY_CHANNEL  BIT(2)
#define IDI_CHANNEL_OPTS_MASK 0x7
#define IDI_TX_EARLY_IRQ	BIT(4)
#define IDI_TX_CHANNEL		BIT(5)
#define IDI_CHANNEL_MASK      (0x0000000F)
#define IDI_SECONDARY_SHIFT   (4)
#define IDI_TERTIARY_SHIFT    (8)

#define IDI_BYTES_TO_FRAMES(x) (((x) + 3) >> 2)
#define IDI_FRAMES_TO_BYTES(x) ((x) << 2)

/* The different frequency values for the IDI*/
#define IDI_FREQUENCY_ENABLE_NO_WLAN        104000
#define IDI_FREQUENCY_ENABLE_NO_WLAN_ALT    138700
#define IDI_FREQUENCY_ENABLE_WLAN           208000
#define IDI_FREQUENCY_ENABLE_WLAN_ALT       178300
#define IDI_FREQUENCY_ENABLE_AUDIO_ONLY     104000
#define IDI_FREQUENCY_ENABLE_AUDIO_ONLY_ALT 138700
#define IDI_FREQUENCY_DISABLE               0

/*
 * The following defines are for the outstanding read feature of the IDI
 * module.  When building a list of registers to read/write, the <READ|WRITE>_
 * macros should be used.  A write will be a command_word + data_word.  A
 * Read will be the command_word, but a second data buffer will be needed to
 * receive the data (see idi_outstanding_read()).
 *
 */
#define OUTSTANDING_WRITE       BIT(31)
#define OUTSTANDING_WORD_WRITE  (0xF << 25)
#define OUTSTANDING_WORD_READ   (0xF << 25)
#define OUTSTANDING_SHORT_LO    (0x3 << 25)
#define OUTSTANDING_SHORT_HI    (0xC << 25)
#define OUTSTANDING_CHAR_0      (0x1 << 25)
#define OUTSTANDING_CHAR_1      (0x2 << 25)
#define OUTSTANDING_CHAR_2      (0x4 << 25)
#define OUTSTANDING_CHAR_3      (0x8 << 25)
#define OUTSTANDING_ADDR_MASK(a)   ((BIT(25) - 1) & a)


#define READ_WORD(a) \
	(OUTSTANDING_WORD_READ | (OUTSTANDING_ADDR_MASK(a)))
#define READ_SHORT_LO(a) \
	(OUTSTANDING_SHORT_LO  | (OUTSTANDING_ADDR_MASK(a)))
#define READ_SHORT_HI(a) \
	(OUTSTANDING_SHORT_HI  | (OUTSTANDING_ADDR_MASK(a)))
#define READ_CHAR_0(a) \
	(OUTSTANDING_CHAR_0    | (OUTSTANDING_ADDR_MASK(a)))
#define READ_CHAR_1(a) \
	(OUTSTANDING_CHAR_1    | (OUTSTANDING_ADDR_MASK(a)))
#define READ_CHAR_2(a) \
	(OUTSTANDING_CHAR_2    | (OUTSTANDING_ADDR_MASK(a)))
#define READ_CHAR_3(a) \
	(OUTSTANDING_CHAR_3    | (OUTSTANDING_ADDR_MASK(a)))

#define WRITE_WORD(a) \
	(OUTSTANDING_WRITE | OUTSTANDING_WORD_WRITE \
						| (OUTSTANDING_ADDR_MASK(a)))
#define WRITE_SHORT_LO(a) \
	(OUTSTANDING_WRITE | OUTSTANDING_SHORT_LO | (OUTSTANDING_ADDR_MASK(a)))
#define WRITE_SHORT_HI(a) \
	(OUTSTANDING_WRITE | OUTSTANDING_SHORT_HI | (OUTSTANDING_ADDR_MASK(a)))
#define WRITE_CHAR_0(a) \
	(OUTSTANDING_WRITE | OUTSTANDING_CHAR_0 | (OUTSTANDING_ADDR_MASK(a)))
#define WRITE_CHAR_1(a) \
	(OUTSTANDING_WRITE | OUTSTANDING_CHAR_1 | (OUTSTANDING_ADDR_MASK(a)))
#define WRITE_CHAR_2(a) \
	(OUTSTANDING_WRITE | OUTSTANDING_CHAR_2 | (OUTSTANDING_ADDR_MASK(a)))
#define WRITE_CHAR_3(a) \
	(OUTSTANDING_WRITE | OUTSTANDING_CHAR_3 | (OUTSTANDING_ADDR_MASK(a)))


struct idi_device_id;
struct idi_transaction;
struct device_pm_platdata;
/**
 * enum idi_peripheral_type - Peripherals supported by the IDI bus.
 * NOTE: these can be used as an index into an array, don't change the
 * increment.
 */
enum idi_peripheral_type { IDI_INTERNAL, IDI_FMR, IDI_AFE, IDI_BT, IDI_I2C,
	IDI_WLAN, IDI_GNSS, IDI_RTC, IDI_MEAS, IDI_REG_FW, IDI_VIBT, IDI_BKL,
	IDI_ACC, IDI_CHG, IDI_BAT, IDI_CCD, IDI_BNT, IDI_DBG, IDI_BT_STREAM,
	IDI_AFE_STREAM,	IDI_FMR_STREAM,	IDI_ERROR, IDI_MAX_PERIPHERAL,
};

enum idi_internal_power_state {
	ENABLE_NO_WLAN,
	ENABLE_NO_WLAN_ALT,
	ENABLE_WLAN,
	ENABLE_WLAN_ALT,
	ENABLE_AUDIO_ONLY,
	ENABLE_AUDIO_ONLY_ALT,
	DISABLE
};

enum idi_priority { IDI_NORMAL_PRIORITY, IDI_HIGH_PRIORITY };

enum idi_channel_type {
	RESERVED, SOFTWARE_DEFINED, REGISTER_ACCESS, FILE_CONTROL,
	OUTSTANDING_READ, STREAM, FILE, SIGNAL_FORWARDING,
	INTERRUPTS, ERRORS,
};

/* IDI message status codes */
enum idi_status {
	IDI_STATUS_COMPLETE,	/* Message transfer is completed */
	IDI_STATUS_PROCEEDING,	/* Message transfer is ongoing */
	IDI_STATUS_QUEUED,	/* Message waiting to be served */
	IDI_STATUS_FLUSH,	/* Message has been flushed */
	IDI_STATUS_ERROR,	/* Error when message transfer was ongoing */
};

/**
 * struct idi_channel_config - Channel configuration information.
 * @channel: peripheral channel
 * @tx_or_rx: 1 for the transmit side
 * @method: data transfer method
 * @priority: channel priority
 * @base: physical base address of a ring or double buffer or bounce buffer
 * @cpu_base: logical base address of a bounce buffer
 * @dest_addr: physical base address of remote HW FIFO for bounce buffer mode
 * @size: actual size (in frames) of the buffer
 * @is_hw_fifo: Hw fifo on client side
 * @hw_fifo_size: Size of the hw fifo, .i.e. address aliasing
 * @size: internal buffer size in frames.
 * @end_of_packet: End of packet callback
 * @private_data: Private peripheral driver data
 * @start_tx: Start TX transaction
 */
struct idi_channel_config {
	int channel_opts;
	int tx_or_rx;
	enum idi_priority priority;
	dma_addr_t base;
	u32 *cpu_base;
	dma_addr_t dst_addr;
	u32 size;
	u32 hw_fifo_size;
	void (*end_of_packet)(void *);
	void *private_data;
	void (*start_tx)(struct idi_transaction *);
};

/**
 * struct idi_sg_desc - Define an IDI descriptor
 * @next: Physical address of the next descriptor in the chain.
 * @base: Physical address of the data buffer.
 * @size: Number of frames in buffer.
 * @pad:  Must be zero.
 *
 * The idi_sg_desc data structure describes a list of data buffers to use in
 * a scatter/gather fashion.  All addresses MUST be physical addresses.  The
 * next element MUST be on a 16 byte boundary.  The base element must be on a
 * 4 byte boundary.  The size is the number of frames (4 byte words) that the
 * buffer pointed to by base contains.
 */
struct idi_sg_desc {
	u32 next;
#define IDI_SG_NEXT_MASK (0xFFFFFFF0)
#define IDI_SG_NEXT_LAST (1 << 0)
#define IDI_SG_NEXT_INT  (1 << 1)
	dma_addr_t base;
#define IDI_SG_BASE_MASK (0xFFFFFFFC)
	u32 size;
#define IDI_SG_SIZE_SHIFT (2)
	u32 pad;
};

/**
 * struct idi_xfer - Information needed to transfer data
 * @desc: a pointer to one or more descriptors
 * @buffer: data buffer
 * @size: size of buffer, or buffers in descriptor chain, in frames
 * @chan_opts: The peripheral channel to send the data on
 * @dst_addr: The destination address of the register space or memory to write
 *            data to.
 *
 * desc and base MUST be virtual addresses.
 *
 */
struct idi_xfer {
	struct idi_sg_desc *desc;
	dma_addr_t base;
	u32 *cpu_base;
	u32 size;
	int channel_opts;
	u32 dst_addr;
	struct scatterlist sg[2];
};

/**
 * struct idi_transaction - IDI message descriptor
 * @idi_xfer: Description of data buffers.
 * @context: Peripheral context data associated to the transfer
 * @complete: Transfer completion callback
 * @queue: used to link to peripheral driver queue;
 * @link: used to link to appropriate channel list.
 * @peripheral: requesting peripheral device.
 * @status: Status of the transfer when completed
 * @actual_len: Actual length of data transfered on completion
 * @channel: Channel to TX/RX the message
 * @resumbit: put message back on queue?
 * @t_type: Transfer type (TX if set, RX otherwise)
 * @break_frame: if true IDI will send/receive a break frame (FRAME MODE)
 */
struct idi_transaction {
	/*
	 * Peripheral driver responsibilities
	 */
	struct idi_xfer idi_xfer;
	void *context;
	void (*complete) (struct idi_transaction *);

	struct list_head queue;
	/*
	 * Internal state.  Used by IDI driver.
	 */
	struct list_head link;
	struct idi_peripheral_device *peripheral;
	int status;
	unsigned int channel;
	unsigned int resubmit:1;
	unsigned int t_type:1;
	unsigned int break_frame:1;

};

/**
 * struct idi_resource - IDI resource object
 * @resource: I/O and interrupts information
 * @num_resources: How many resource
 */

struct idi_resource {
	u32 num_resources;
	struct resource *resource;
};

/**
 * struct idi_peripheral_device - IDI Peripheral device object (Virtual)
 * @flag: Any flags needed.
 * @ptype: The peripheral type associated with this device.
 * @device: The peripheral's device structure.
 * @driver: Pointer to the peripheral device driver
 * @controller: Pointer to the IDI controller
 * @pm_current_state: power management state  FIXME: not used
 * @irq_map: Map of the available IRQs for this peripheral
 * @resource: I/O space information
 * @channel_map: channel map associated with this device
 * @sw_channel: allocated channel for sofware controlled transfers
 * @list: so we can keep track of the devices.
 *
 */
struct idi_peripheral_device {
	u32 flags;
	enum idi_peripheral_type p_type;
	struct idi_device_id id;
	struct device device;
	struct idi_peripheral_driver *driver;
	struct idi_controller_device *controller;

	/*   u32                     pm_current_state; */
	struct idi_resource resources;
	u32 channel_map;
	u32 sw_channel;
	struct list_head link;
	struct device_pm_platdata *pm_platdata;

};

#define to_idi_peripheral_device(_dev) \
	container_of(_dev, struct idi_peripheral_device, device)

/**
 * struct idi_peripheral_device - IDI Peripheral device object information
 * @p_type: The peripheral type associated with this device.
 * @resource: I/O and interrupt resource information
 * @name: The peripheral name
 * @of_node: OF node
 * @pm_platdata: Unified platform PM informations
 * @platform_data: Any specific platform information the driver should be aware.
 */

struct idi_peripheral_device_info {
	enum idi_peripheral_type p_type;
	struct idi_device_id id;
#define IDI_PERIPH_MAX_NAME_SIZE 16
	char name[IDI_PERIPH_MAX_NAME_SIZE];
	struct idi_resource resources;
	struct device_node *of_node;
	unsigned channel_map;
	struct device_pm_platdata *pm_platdata;
	void *platform_data;
};

/**
 * struct idi_peripheral_driver - IDI peripheral device driver
 * @flags: If needed.
 * @p_type: Type of peripheral device supported.
 * @probe:
 * @remove:
 * @shutdown:
 * @suspend:
 * @resume:
 * @driver: The driver
 * @device: which device the driver is controllering
 * @id_table: list of optional devices supported
 * @ctrl_io: memory maped IO pointer
 * @link:
 * @sw_lock:
 */
struct idi_peripheral_driver {
	u32 flags;
	enum idi_peripheral_type p_type;

	int (*probe) (struct idi_peripheral_device *,
		      const struct idi_device_id *);
	int (*remove) (struct idi_peripheral_device *);

	/*
	 * driver model interfaces that don't relate to enumeration.
	 */
	void (*shutdown) (struct idi_peripheral_device *);
	int (*suspend) (struct idi_peripheral_device *, pm_message_t mesg);
	int (*resume) (struct idi_peripheral_device *);

/*FIXME: Do we need start_rx,stop_rx */
	void (*idi_start_rx) (struct idi_peripheral_device *);
	void (*idi_stop_rx) (struct idi_peripheral_device *);

	struct device_driver driver;
	struct device *device;
	const struct idi_device_id *id_table;

	void __iomem *ctrl_io;

	/* Queues and registers access locks */
	spinlock_t sw_lock;	/* used when accessing software structures */

};

#define to_idi_peripheral_driver(_drv) \
	container_of(_drv, struct idi_peripheral_driver, driver)

static inline void
idi_peripheral_set_drvdata(struct idi_peripheral_device *peripheral, void *data)
{
	dev_set_drvdata(&peripheral->device, data);
}

static inline void
*idi_peripheral_get_drvdata(struct idi_peripheral_device *peripheral)
{
	return dev_get_drvdata(&peripheral->device);
}

int idi_register_peripheral_driver(struct idi_peripheral_driver *);
void idi_unregister_peripheral_driver(struct idi_peripheral_driver *);
struct resource *idi_get_resource_byname(struct idi_resource *,
					      unsigned int type,
					      const char *name);

int idi_outstanding_read(struct idi_peripheral_device *,
			 struct idi_transaction *,
			 struct idi_transaction *);

/*int idi_request_access(struct idi_peripheral_device *peripheral); */
/*int idi_release_access(struct idi_peripheral_device *peripheral); */

struct idi_transaction *idi_alloc_transaction(gfp_t flags);
void idi_free_transaction(struct idi_transaction *);

int idi_async(struct idi_peripheral_device *, struct idi_transaction *);
int idi_async_read(struct idi_peripheral_device *, struct idi_transaction *);
int idi_async_write(struct idi_peripheral_device *, struct idi_transaction *);
int idi_peripheral_flush(struct idi_peripheral_device *, int);

int idi_set_channel_config(struct idi_peripheral_device *,
					struct idi_channel_config *);
int idi_request_buffer_info(struct idi_peripheral_device *, int, int);
int idi_request_buffer_flush(struct idi_peripheral_device *, int);
int idi_request_queue_status(struct idi_peripheral_device *, int, int);
int idi_open_software_channel(struct idi_peripheral_device *);
int idi_close_software_channel(struct idi_peripheral_device *);
int idi_software_write(struct idi_peripheral_device *,
						struct idi_transaction *);
int idi_software_read(struct idi_peripheral_device *, struct idi_transaction *);
int idi_set_power_state(struct idi_peripheral_device *, void *, bool);
int idi_set_power_state_by_name(struct idi_peripheral_device *, char *, bool);
int idi_get_client_id(struct idi_peripheral_device *, u32 *);
int idi_client_ioread(struct idi_peripheral_device *, unsigned, unsigned *);
int idi_client_iowrite(struct idi_peripheral_device *, unsigned, unsigned);

#endif
