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

/**
 * Module:
 *    idi_interface.c
 *
 * Description:
 *    This module contains all of the functionality needed to implement the
 *    interface used by peripheral drivers to access the IDI module.
 *
 *    NOTE: this file in the top level IDI directory.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>

#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_bus.h>

#include <linux/idi/idi_debug.h>

MODULE_LICENSE("GPL");

/**
 * idi_outstanding_read - Use the oustanding read functionality
 * @peripheral: device to read the registers from
 * @regs: a list of registers to read or write.
 * @data: buffer to put the registers values in.
 *
 * idi_outstanding_read() uses the idi_async() function call because the
 * idi_async_<read|write>() functions set the channel based on the peripheral.
 * Since we don't want that to happen, we set the channel and direction, then
 * call the idi_async().
 *
 * The peripheral uses the raw base address (from idi_request_register_base())
 * + the register offset to get the 25bit address.
 *
 * If there are reads and writes intermixed, it is possible for the read
 * transaction to complete before the programmed read/writes (i.e. we have 5
 * reads and 5 writes in that order.  Reads finish first and complete the read
 * buffer which completes).  Peripheral drivers should be aware of that.
 *
 */
int idi_outstanding_read(struct idi_peripheral_device *peripheral,
			 struct idi_transaction *regs,
			 struct idi_transaction *data)
{
	struct idi_client_device *client;
	struct idi_controller_device *idi;
	int ret = 0;
#ifdef FIXME
	u32 *ptr;
	int reads = 0;
#endif
	if ((peripheral == NULL) || (regs == NULL))
		return -EINVAL;

	idi_debug_add_event(outstanding_read, peripheral->p_type, regs);

	idi = peripheral->controller;
	if (idi == NULL)
		return -ENODEV;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	if (regs->complete == NULL)
		return -EINVAL;
#ifdef FIXME
	/*
	 * If there is no read buffer, walk the requests and make sure that
	 * there are no reads requests.
	 *
	 * If there is a read buffer, make sure the read count matches the read
	 * buffer size.
	 */
	if (data == NULL) {
		/*
		 * Write commands have the high bit set, and are 2 words long.
		 * If we find any non-writes, we have an error.
		 */
		for (ptr = regs->idi_xfer.cpu_base;
		     ptr < (regs->idi_xfer.cpu_base +
			(regs->idi_xfer.size/sizeof(u32))); ptr++) {
			if (*ptr & OUTSTANDING_WRITE)
				ptr++;
			else
				return -EINVAL;
		}
	} else {
		if (data->complete == NULL)
			return -EINVAL;

		/*
		 * Count the reads.  Make sure the read buffer size is correct
		 */
		for (ptr = regs->idi_xfer.cpu_base;
		     ptr < (regs->idi_xfer.cpu_base +
			(regs->idi_xfer.size/sizeof(u32))); ptr++) {
			if (*ptr & OUTSTANDING_WRITE)
				ptr++;
			else
				reads++;
		}

		if (reads != (data->idi_xfer.size/sizeof(u32)))
			return -EINVAL;
	}
#endif
	/*
	 * If we have a read, set it up.
	 */
	ret = 0;
	if (data) {
		data->channel = client->rd_register_chan;
		data->t_type = IDI_TRANS_READ;
		ret = idi_async(peripheral, data);
	}

	/*
	 * Assuming no issues, set up the write.
	 */
	if (ret == 0) {
		regs->channel = client->wr_register_chan;
		regs->t_type = IDI_TRANS_WRITE;
		ret = idi_async(peripheral, regs);
		if ((ret != 0) && data)
			idi->request_buffer_flush(idi, data->channel);
	}

	return ret;

}				/* idi_outstanding_read() */
EXPORT_SYMBOL(idi_outstanding_read);

/**
 * idi_alloc_transaction - Allocate a transaction data structure
 * @flags: Kernel allocation flags
 *
 * Return NULL on failure or a pointer to an idi_transactin_t on success.
 */
struct idi_transaction *idi_alloc_transaction(gfp_t flags)
{
	struct idi_transaction *trans;

	trans = kzalloc(sizeof(*trans), flags);
	return trans;

}				/* idi_alloc_transaction() */
EXPORT_SYMBOL_GPL(idi_alloc_transaction);

/**
 * idi_free_transation - Free a transaction data structure
 * @trans: The transaction data structure to free
 *
 */
void idi_free_transaction(struct idi_transaction *trans)
{
	if (trans == NULL)
		return;

	kfree(trans);

}				/* idi_free_trans() */
EXPORT_SYMBOL_GPL(idi_free_transaction);

static int idi_channel_assign(struct idi_peripheral_device *peripheral,
			      int channel_opts)
{
	unsigned long channel;

	switch (channel_opts & IDI_CHANNEL_OPTS_MASK) {
	case IDI_PRIMARY_CHANNEL:
		channel = peripheral->channel_map;
		break;

	case IDI_SECONDARY_CHANNEL:
		channel = peripheral->channel_map >> IDI_SECONDARY_SHIFT;
		break;

	case IDI_TERTIARY_CHANNEL:
		channel = peripheral->channel_map >> IDI_TERTIARY_SHIFT;
		break;

	default:
		return IDI_MAX_CHANNEL;
	}

	/*
	 * If the user requested a non-primary channel on a device that has
	 * only one or for some reason channel 0 is configured (0 is reserved),
	 * make sure to return an error.
	 */
	if ((channel & IDI_CHANNEL_MASK) == 0) {
		pr_debug("Bad channel selected\n");
		return IDI_MAX_CHANNEL;
	}
	return channel & IDI_CHANNEL_MASK;
}				/* idi_channel_assign() */

/**
 * idi_async - Submit an IDI transfer to the controller
 * @peripheral: IDI device sending the transfer
 * @trans: The IDI tranaction passed to controller
 *
 * In general the idi_<sync|async>_<read|write> functions should be used and not
 * this function.
 *
 * The IDI message must have the channel, t_type, complete and destructor
 * fields set beforehand.
 *
 * IDI controllers rely on pre-allocated buffers from their clients and they
 * do not allocate buffers on their own.
 *
 * Once the IDI message transfer finishes, the idi controller calls the
 * complete callback with the status and actual_len fields of the idi message
 * updated. The complete callback can be called before returning from
 * idi_async.
 *
 * Returns -errno on failure or 0 on success
 */
int idi_async(struct idi_peripheral_device *peripheral,
	      struct idi_transaction *trans)
{
	struct idi_client_device *client;
	struct idi_controller_device *idi;

	if ((peripheral == NULL) || (trans == NULL))
		return -EINVAL;

	trans->peripheral = peripheral;

	/*
	 * t_type is one bit.  The check is not very useful.
	 */
	if ((trans->t_type != IDI_TRANS_READ)
	    && (trans->t_type != IDI_TRANS_WRITE))
		return -EINVAL;

	if (trans->channel == IDI_MAX_CHANNEL)
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	idi = peripheral->controller;
	if (idi == NULL)
		return -ENODEV;

	WARN_ON_ONCE(!trans->complete);
	trans->context = idi;

	return idi->async(trans);

}				/* idi_async() */
EXPORT_SYMBOL_GPL(idi_async);

/**
 * idi_async_read - Set up an asynchronous read.
 * @peripheral: Pointer to the peripheral device
 * @trans: idi transaction descriptor of the transfer
 *
 */
int idi_async_read(struct idi_peripheral_device *peripheral,
		   struct idi_transaction *trans)
{
	/*
	 * Set the transfer direction and channel based on the peripheral.
	 */
	trans->t_type = IDI_TRANS_READ;
	trans->channel =
	    idi_channel_assign(peripheral, trans->idi_xfer.channel_opts);
	if (trans->channel == IDI_MAX_CHANNEL)
		return -EINVAL;

	idi_debug_add_event(async_read, peripheral->p_type, trans);

	return idi_async(peripheral, trans);

}				/* idi_async_read() */
EXPORT_SYMBOL_GPL(idi_async_read);

/**
 * idi_async_write - Set up an asynchronous read
 * @peripheral:Pointer to the peripheral device
 * @trans:  idi transaction descriptor of the transfer
 *
 * Return:
 *    -errno  on failure
 *    0       on success
 */
int idi_async_write(struct idi_peripheral_device *peripheral,
		    struct idi_transaction *trans)
{
	trans->t_type = IDI_TRANS_WRITE;
	trans->channel =
	    idi_channel_assign(peripheral, trans->idi_xfer.channel_opts);
	if (trans->channel == IDI_MAX_CHANNEL)
		return -EINVAL;

	idi_debug_add_event(async_write, peripheral->p_type, trans);

	return idi_async(peripheral, trans);

}				/* idi_async_write() */
EXPORT_SYMBOL_GPL(idi_async_write);

/*
 * Function:
 *    idi_peripheral_flush()
 *
 * Description:
 *    Flush all pending transactions of idi peripheral device.
 *    This function will destroy all pending idi_msg in the given channel
 *    and reset the HW port so it is ready to receive and transmit
 *    from a clean state.
 *
 * Parameters:
 *    @peripheral: Pointer to the IDI peripheral device
 *    @channel_opts: Peripheral channel to flush
 *
 * Return:
 *    -errno  on failure
 *    0       on success
 */
int idi_peripheral_flush(struct idi_peripheral_device *peripheral,
					 int channel_opts)
{
	struct idi_controller_device *idi;
	int tx_not_rx = 0;
	int channel;

	if (peripheral == NULL)
		return -EINVAL;

	idi = peripheral->controller;
	if (idi == NULL)
		return -ENODEV;

	channel = idi_channel_assign(peripheral, channel_opts);
	if (channel == IDI_MAX_CHANNEL)
		return -EINVAL;

	if (channel_opts & IDI_TX_CHANNEL)
		tx_not_rx = 1;

	idi_debug_add_event(transaction_flush, peripheral->p_type, NULL);

	return peripheral->controller->channel_flush(idi, tx_not_rx, channel);

}				/* idi_peripheral_flush() */
EXPORT_SYMBOL_GPL(idi_peripheral_flush);

/**
 * idi_set_channel_config - Set channel configuration
 * @peripheral: device to find buffer information for
 * @config: configuration to set on the assigned channel
 */
int idi_set_channel_config(struct idi_peripheral_device *peripheral,
			   struct idi_channel_config *config)
{
	struct idi_client_device *client;
	struct idi_controller_device *idi;
	int channel;

	if ((peripheral == NULL) || (config == NULL))
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	idi = peripheral->controller;
	if (idi == NULL)
		return -ENODEV;

	/*
	 * Check with the client driver to make sure this channel belongs to the
	 * given peripheral, and then set the channel configuration.
	 */
	channel = idi_channel_assign(peripheral, config->channel_opts);
	if (channel == IDI_MAX_CHANNEL)
		return -EINVAL;

	idi_debug_add_event(channel_config, peripheral->p_type, NULL);

	return idi->set_channel_config(idi, config, channel);

}				/* idi_set_buffer_size() */
EXPORT_SYMBOL_GPL(idi_set_channel_config);

/**
 * idi_request_buffer_info - Find out how much space is available
 * @peripheral: device to find buffer information for
 *
 * idi_request_buffer_info() will forward a request to the IDI controller that
 * will determine how much space is available (writes), or how much space has
 * been used (reads) for a buffer that is currently associated with the given
 * channel.
 *
 */
int idi_request_buffer_info(struct idi_peripheral_device *peripheral,
			    int tx_or_rx, int channel_opts)
{
	struct idi_client_device *client;
	struct idi_controller_device *idi;
	int channel;

	if (peripheral == NULL)
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	idi = peripheral->controller;
	if (idi == NULL)
		return -ENODEV;

	/*
	 * Check with the client driver to make sure this channel belongs to the
	 * given peripheral, and then get the requested info.
	 */
	channel = idi_channel_assign(peripheral, channel_opts);
	if (channel == IDI_MAX_CHANNEL)
		return -EINVAL;

	idi_debug_add_event(buffer_info, peripheral->p_type, NULL);

	return idi->request_buffer_info(idi, tx_or_rx, channel);

}				/* idi_request_buffer_info() */
EXPORT_SYMBOL_GPL(idi_request_buffer_info);

/**
 * idi_request_buffer_flush - Force a read transaction to complete
 * @peripheral: device to find buffer information for.
 * @channel: which peripheral specific channel to operate on
 *
 * idi_request_buffer_flush() will stop the current read transaction on the
 * given channel.  It will then call the completion routine for the operation
 * that was occuring on that channel.
 *
 */
int idi_request_buffer_flush(struct idi_peripheral_device *peripheral,
			     int channel_opts)
{
	struct idi_client_device *client;
	struct idi_controller_device *idi;
	int channel;

	if (peripheral == NULL)
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	idi = peripheral->controller;
	if (idi == NULL)
		return -ENODEV;

	/*
	 * Check with the client driver to make sure this channel belongs to the
	 * given peripheral, and then flush the buffer.
	 */
	channel = idi_channel_assign(peripheral, channel_opts);
	if (channel == IDI_MAX_CHANNEL)
		return -EINVAL;

	idi_debug_add_event(buffer_flush, peripheral->p_type, NULL);

	return idi->request_buffer_flush(idi, channel);

}				/* idi_request_queue_status() */
EXPORT_SYMBOL_GPL(idi_request_buffer_flush);

/**
 * idi_request_queue_status - Get information on queued transactions
 * @peripheral: device to find buffer information for.
 * @tx_or_rx: which direction
 * @channel: which peripheral specific channel to get info on.
 */
int idi_request_queue_status(struct idi_peripheral_device *peripheral,
			     int tx_or_rx, int channel_opts)
{
	struct idi_client_device *client;
	struct idi_controller_device *idi;
	int channel;

	if (peripheral == NULL)
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	idi = peripheral->controller;
	if (idi == NULL)
		return -ENODEV;

	/*
	 * Check with the client driver to make sure this channel belongs to the
	 * given peripheral, and then get the queue status.
	 */
	channel = idi_channel_assign(peripheral, channel_opts);
	if (channel == IDI_MAX_CHANNEL)
		return -EINVAL;

	idi_debug_add_event(queue_status, peripheral->p_type, NULL);

	return idi->request_queue_status(idi, tx_or_rx, channel);

}				/* idi_request_queue_status() */
EXPORT_SYMBOL_GPL(idi_request_queue_status);

/**
 * idi_open_software_channel - Get a channel for software controlled transfers
 * @peripheral: peripheral that would like to do the transfers
 *
 * A peripheral may need to do a memory to memory data transfer.  In order to
 * do this, it will need a channel to do the transfers on.  This function
 * allows it to allocate a channel for this purpose.  This will be mostly
 * used for firmware uploads for peripheral devices.
 */
int idi_open_software_channel(struct idi_peripheral_device *peripheral)
{
	struct idi_client_device *client;

	if (peripheral == NULL)
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	idi_debug_add_event(open_sw_chan, peripheral->p_type, NULL);

	return client->open_software_channel(client, peripheral);

}				/* idi_open_software_channel() */
EXPORT_SYMBOL_GPL(idi_open_software_channel);

/**
 * idi_close_software_channel - Release a channel
 * @peripheral: peripheral that has channel allocated.
 *
 * Once a peripheral is done transfering data, it needs to release the
 * software controlled channel resereved by the open function.
 *
 */
int idi_close_software_channel(struct idi_peripheral_device *peripheral)
{
	struct idi_client_device *client;

	if (peripheral == NULL)
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	idi_debug_add_event(close_sw_chan, peripheral->p_type, NULL);

	return client->close_software_channel(client, peripheral);

}				/* idi_close_software_channel() */
EXPORT_SYMBOL_GPL(idi_close_software_channel);

/**
 * idi_software_read - Do a software controlled read.
 * @peripheral: peripheral that has channel allocated.
 * @data: a pointer to transaction context to do a read.
 *
 */
int idi_software_read(struct idi_peripheral_device *peripheral,
		      struct idi_transaction *data)
{
	struct idi_client_device *client;

	if ((peripheral == NULL) || (data == NULL))
		return -EINVAL;

	data->peripheral = peripheral;

	if (data->complete == NULL)
		return -EINVAL;

	if (data->idi_xfer.dst_addr == 0)
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	idi_debug_add_event(sw_read, peripheral->p_type, data);

	/*
	 * The client will make sure that a software controlled channel is
	 * associated with the peripheral, and then set up the read.
	 */
	return client->software_read(client, peripheral, data);

}				/* idi_software_read() */
EXPORT_SYMBOL_GPL(idi_software_read);

int idi_software_write(struct idi_peripheral_device *peripheral,
		       struct idi_transaction *data)
{
	struct idi_client_device *client;

	if ((peripheral == NULL) || (data == NULL))
		return -EINVAL;

	data->peripheral = peripheral;

	if (data->complete == NULL)
		return -EINVAL;

	if (data->idi_xfer.dst_addr == 0)
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	idi_debug_add_event(sw_write, peripheral->p_type, data);

	/*
	 * The client will make sure that a software controlled channel is
	 * associated with the peripheral, and then set up the read.
	 */
	return client->software_write(client, peripheral, data);

}				/* idi_software_write() */
EXPORT_SYMBOL_GPL(idi_software_write);

/**
 * idi_set_power_state - Set the IDI controller and IDI devices state.
 * @peripheral: peripheral device wants to set the state.
 * @handler: handler for the peripheral device state.
 * @idi_is_required : whether idi is required or not.
 */
int idi_set_power_state(struct idi_peripheral_device *peripheral,
				void *handler,
				bool idi_is_required)
{
	struct idi_client_device *client;
	struct idi_controller_device *idi;
	if ((peripheral == NULL) || (handler == NULL))
		return -EINVAL;
	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;
	idi = peripheral->controller;
	if (idi == NULL)
		return -ENODEV;
	idi_debug_add_event(set_state, peripheral->p_type, NULL);
	return idi->set_power_state(
						idi,
						peripheral,
						handler,
						NULL,
						idi_is_required);
}
EXPORT_SYMBOL_GPL(idi_set_power_state);

/**
 * idi_set_power_state_by_name - Set the IDI controller and IDI devices state.
 * @peripheral: peripheral device wants to set the state.
 * @state: string for the peripheral device state.
 * @idi_is_required : whether idi is required or not
 */
int idi_set_power_state_by_name(
			struct idi_peripheral_device *peripheral,
	char *state,
	bool idi_is_required)
{
	struct idi_client_device *client;
	struct idi_controller_device *idi;
	if ((peripheral == NULL) || (state == NULL))
		return -EINVAL;
	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;
	idi = peripheral->controller;
	if (idi == NULL)
		return -ENODEV;
	idi_debug_add_event(set_state, peripheral->p_type, NULL);
	return idi->set_power_state(
						idi,
						peripheral,
						NULL,
						state,
						idi_is_required);
	}
EXPORT_SYMBOL_GPL(idi_set_power_state_by_name);

/**
 * idi_get_client_id - Retrieve the ChipID of the client i.e AGOLD
 * @peripheral: peripheral device who needs the ChipID
 * @chipid : function stores the chipid of AGOLD
 * return value : zero on sucesss and -EINVAL/-ENODEV on error
 */
int idi_get_client_id(struct idi_peripheral_device *peripheral, u32 *chipid)
{
	struct idi_client_device *client;

	if (peripheral == NULL)
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	*chipid = client->chipid;
	return 0;
}
EXPORT_SYMBOL_GPL(idi_get_client_id);

int idi_client_ioread(struct idi_peripheral_device *peripheral,
					unsigned addr, unsigned *reg)
{
	struct idi_client_device *client;

	if (peripheral == NULL)
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	if (client->ioread == NULL)
		return -ENODEV;

	return client->ioread(client, addr, reg);

}
EXPORT_SYMBOL_GPL(idi_client_ioread);

int idi_client_iowrite(struct idi_peripheral_device *peripheral,
					unsigned addr, unsigned value)
{
	struct idi_client_device *client;

	if (peripheral == NULL)
		return -EINVAL;

	client = to_idi_client_device(peripheral->device.parent);
	if (client == NULL)
		return -ENODEV;

	if (client->iowrite == NULL)
		return -ENODEV;

	return client->iowrite(client, addr, value);

}
EXPORT_SYMBOL_GPL(idi_client_iowrite);
