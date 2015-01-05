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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/idi/idi_interface.h>


void *idi_xfer_dma_to_virt(struct device *_dev,
					struct idi_xfer *xfer,
					struct scatterlist *sg)
{
	struct scatterlist *st_sg = &xfer->sg[0];
	unsigned offset = ((unsigned) xfer->cpu_base) - sg_dma_address(st_sg);


	return (void *)(sg_dma_address(sg) + offset);
}
EXPORT_SYMBOL_GPL(idi_xfer_dma_to_virt);

/**
 * idi_push_xfer_to_tty - Push an IDI xfer to a tty xmit buffer
 * @pdev: IDI peripheral device attached to the transaction
 * @xfer: IDI xfer to push to tty xmit buffer
 * @tty:  tty to use
 *
 * idi_push_xfer_to_tty() will push xfer characters to tty buffer
 */

int idi_push_xfer_to_tty(struct idi_peripheral_device *pdev,
			 struct idi_xfer *xfer, struct tty_struct *tty)
{
	struct device *dev = &pdev->device;
	int payload, len, myroom, acc = 0;
	struct scatterlist *sg;
	unsigned char *buffer;

	payload = xfer->size;
	sg = &xfer->sg[0];
	if (!sg) {
		dev_err(dev, "xfer\'s SG list is null\n");
		return -EINVAL;
	}

	buffer = idi_xfer_dma_to_virt(dev, xfer, sg);
	len = sg_dma_len(sg);

	dma_sync_sg_for_cpu(NULL, sg, sg_nents(sg), DMA_FROM_DEVICE);
	do {
		dev_dbg(dev, "\tRequest %#x bytes from tty\n", payload);
		myroom = tty_buffer_request_room(tty->port, payload);
		acc = 0;
		dev_dbg(dev, "Pushing %#x bytes from %p to tty:\n", myroom,
			buffer);
		do {
			dev_dbg(dev, "\tbuffer %p len %#x bytes\n", buffer,
				len);
			if (myroom > len) {
				dev_dbg(dev,
					"\t[TTY] insert full SG %p %#x bytes\n",
					buffer, len);
				tty_insert_flip_string(tty->port, buffer, len);
				myroom -= len;
				acc += len;
				sg = sg_next(sg);
				if (sg == NULL)
					BUG();

				buffer = idi_xfer_dma_to_virt(dev, xfer, sg);
				len = sg_dma_len(sg);
			} else {
				unsigned char *buffer2;
				dev_dbg(dev, "\t[TTY] insert %p %#x bytes\n",
					buffer, myroom);
				tty_insert_flip_string(tty->port,
								buffer, myroom);
				acc += myroom;
				buffer += myroom;
				myroom = 0;
				buffer2 = idi_xfer_dma_to_virt(dev, xfer, sg);
				len =
				    (int)(buffer2 + sg_dma_len(sg)) -
				    (int)buffer;
				if (!(len > 0)) {
					sg = sg_next(sg);
					if (sg) {
						len = sg_dma_len(sg);
						buffer = idi_xfer_dma_to_virt(
								dev, xfer, sg);
					}
				}
			}
		} while (myroom && sg);

		tty_flip_buffer_push(tty->port);
		dev_dbg(dev, "<--- [TTY]: %#x bytes pushed to flip buffer\n",
			acc);
		payload -= acc;
	} while (payload && sg);

	dma_sync_sg_for_device(NULL, &xfer->sg[0], sg_nents(&xfer->sg[0]),
			DMA_FROM_DEVICE);

	return xfer->size - payload;
}
EXPORT_SYMBOL_GPL(idi_push_xfer_to_tty);

/**
 * idi_tty_prepare_tx_xfer - prepare an IDI transaction for TX operation
 * @pdev: IDI peripheral device attached to the transaction
 * @trans: the IDI transaction to fill in
 * @port: the uart port to pull the data from
 * @buffer: U32 aligned buffer in case the start address is not u32 aligned
 * @buffer_size: Size of buffer
 * @max_payload: Maximum size of payload, ignored if null
 *
 * idi_tty_prepare_tx_xfer() fills in idi write transaction from tty buffer
 * Caller must hold the port lock
 */

int idi_tty_prepare_tx_xfer(struct idi_peripheral_device *p_device,
			    struct idi_transaction *trans,
			    struct uart_port *port,
			    unsigned *buffer,
			    size_t buffer_size, size_t max_payload)
{
	struct circ_buf *xmit = &port->state->xmit;
	struct idi_xfer *xfer = &trans->idi_xfer;
	struct device *dev = &p_device->device;
	char *start;

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return 0;

	xfer->size = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
	start = &xmit->buf[xmit->tail];

	if ((max_payload) && (xfer->size > max_payload))
		xfer->size = max_payload;

	if (!IS_ALIGNED((u32) start, sizeof(u32))) {
		if (xfer->size > buffer_size) {
			/* Be nice with next xfer, align it ! */
			xfer->size = buffer_size - (xfer->size
						    - round_down(xfer->size,
								 sizeof(u32)));
		}
		dev_dbg(dev, "Copying %d bytes to %p from %p\n",
			xfer->size, buffer, start);
		memcpy(buffer, start, xfer->size);

		xfer->cpu_base = buffer;
	} else {
		xfer->cpu_base = (u32 *) start;
	}

#ifdef USE_MAP_SINGLE
	xfer->base = dma_map_single(dev, xfer->cpu_base,
				    xfer->size, DMA_TO_DEVICE);
#else
	xfer->base = dma_map_page(dev,
			virt_to_page(xfer->cpu_base),
			offset_in_page(xfer->cpu_base),
			xfer->size,
			DMA_TO_DEVICE);

#endif
	if (dma_mapping_error(dev, xfer->base)) {
		/* FIXME: Recover from the error */
		dev_err(dev, "Incorrect DMA  mapping\n");
	}

	return xfer->size;
}
EXPORT_SYMBOL_GPL(idi_tty_prepare_tx_xfer);

/**
 * idi_tty_complete_tx_xfer - Complete an IDI write transaction
 * @pdev: IDI peripheral device attached to the transaction
 * @trans: the IDI transaction to fill in
 * @port: the uart port to pull the data from
 *
 * idi_tty_complete_tx_xfer() completes an IDI write operation
 */

void idi_tty_complete_tx_xfer(struct idi_peripheral_device *p_device,
			      struct idi_transaction *trans,
			      struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	struct device *dev = &p_device->device;
	struct idi_xfer *xfer = &trans->idi_xfer;
	unsigned long flags;

#ifdef USE_MAP_SINGLE
	dma_unmap_single(dev, xfer->base, xfer->size, DMA_TO_DEVICE);
#else
	dma_unmap_page(dev, xfer->base, xfer->size, DMA_TO_DEVICE);
#endif

	spin_lock_irqsave(&port->lock, flags);

	/* Move tail pointer */
	xmit->tail += xfer->size;
	xmit->tail &= UART_XMIT_SIZE - 1;
	port->icount.tx += xfer->size;

	spin_unlock_irqrestore(&port->lock, flags);

	/* Request for more characters if not sufficient */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

}
EXPORT_SYMBOL_GPL(idi_tty_complete_tx_xfer);
