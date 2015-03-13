/*************************************************************************
 *
 *  Copyright (C) 2015 Intel Mobile Communications GmbH
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************/
#ifndef __LINUX_USB_GADGET_IOCTL_H
#define __LINUX_USB_GADGET_IOCTL_H

#include <linux/usb/gadget.h>

#define IOCTL_TYPE_MASK		(0x03 << 14)
#define IOCTL_TYPE_STANDARD	(0x00 << 14)
#define IOCTL_TYPE_VENDOR	(0x01 << 14)

/* vendor list */
#define IOCTL_CODE_DWC3		(IOCTL_TYPE_VENDOR | 0x5533)

struct usb_gadget_ioctl_params {
	unsigned	subcode;
	void		*params;
};

/* vendor DWC3 subcodes */
#define VENDOR_DWC3_EBC_EXT_ADD		(0x01)
#define VENDOR_DWC3_EBC_EXT_REMOVE	(0x02)

/**
 * usb_gadget_ioctl - send specific i/o command
 * @gadget: controller that handles i/o command
 * @opcode: i/o command operation code
 * @params: i/o command parameters
 *
 * returns zero on success, else negative errno.
 */
static inline int usb_gadget_ioctl(
	struct usb_gadget *gadget, unsigned opcode,
	struct usb_gadget_ioctl_params *params)
{
	if (!gadget || !gadget->ops)
		return -EINVAL;
	if (!gadget->ops->ioctl)
		return -EOPNOTSUPP;
	return gadget->ops->ioctl(gadget, opcode, (unsigned long) params);
}

#endif /* __LINUX_USB_GADGET_IOCTL_H */
