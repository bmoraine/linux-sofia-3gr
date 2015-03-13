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
#ifndef __LINUX_USB_DWC3_EXTENSION_H
#define __LINUX_USB_DWC3_EXTENSION_H

#include <linux/dma-mapping.h>

/* dwc3 ebc extension */
struct dwc3_ebc_ext {
	/* internal */
	struct list_head list;
	/* required */
	bool (*needed)(struct dwc3_ebc_ext *ext, const void *data);
	/* required */
	unsigned trb_pool_size;
	/* required */
	bool trb_pool_linked;
};

/**
 * check if extension is valid
 * @ext: ebc extension
 *
 * return true if is valid
 */
static inline bool dwc3_ebc_ext_valid(
	struct dwc3_ebc_ext *ext)
{
	if (!ext || !ext->needed)
		return false;
	if (!ext->trb_pool_size
	|| ((ext->trb_pool_size - 1) & ext->trb_pool_size) /* not power of 2 */
	|| ((ext->trb_pool_size < 2) && ext->trb_pool_linked))
		return false;
	return true;
}

/**
 * check if extension is needed
 * @ext: ebc extension
 * @data: input for analysis
 *
 * return true if is needed
 */
static inline bool dwc3_ebc_ext_needed(
	struct dwc3_ebc_ext *ext, const void *data)
{
	return ext && ext->needed && ext->needed(ext, data);
}

/**
 * get trb pool size required
 * @ext: ebc extension
 *
 * return required trbs
 */
static inline unsigned dwc3_ebc_ext_trb_pool_size(
	struct dwc3_ebc_ext *ext)
{
	return ext ? ext->trb_pool_size : 0;
}

/**
 * check if extension uses trb pool linked
 * linked pools do not need to sw re-queue
 * @ext: ebc extension
 *
 * return true if uses trb pool linked
 */
static inline bool dwc3_ebc_ext_trb_pool_linked(
	struct dwc3_ebc_ext *ext)
{
	return ext ? ext->trb_pool_linked : false;
}

#endif /* __LINUX_USB_DWC3_EXTENSION_H */
