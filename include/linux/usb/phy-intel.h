/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

enum intel_usb_events {
	INTEL_USB_EVENT_NONE,		/* no events or cable disconnected */
	INTEL_USB_EVENT_VBUS,		/* vbus valid event */
	INTEL_USB_EVENT_ID,		/* id was grounded */
	INTEL_USB_EVENT_CHARGER,	/* usb dedicated charger */
	INTEL_USB_EVENT_ENUMERATED,	/* gadget driver enumerated */
	INTEL_USB_DRV_VBUS,		/* drive vbus */
	INTEL_USB_DRV_VBUS_ERR,		/* drive vbus error */
	INTEL_USB_ID_SESSION,		/* id session */
};
