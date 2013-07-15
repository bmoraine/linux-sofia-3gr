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

#ifndef _IDI_TTY_H
#define _IDI_TTY_H
void *idi_xfer_dma_to_virt(struct device *,
					struct idi_xfer *,
					struct scatterlist *);

int idi_push_xfer_to_tty(struct idi_peripheral_device *,
						struct idi_xfer *,
						struct tty_struct *);

int idi_tty_prepare_tx_xfer(struct idi_peripheral_device *,
					 struct idi_transaction *,
					 struct uart_port *,
					 unsigned *,
					 size_t,
					 size_t);


void idi_tty_complete_tx_xfer(struct idi_peripheral_device *,
					 struct idi_transaction *,
					 struct uart_port *);


#endif /* _IDI_TTY_H */

