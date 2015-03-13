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
	/* optional */
	void* (*trb_pool_alloc)(struct dwc3_ebc_ext *ext, struct device *dev,
				size_t size, dma_addr_t *dma, gfp_t flags);
	/* optional */
	void (*trb_pool_free)(struct dwc3_ebc_ext *ext, struct device *dev,
				size_t size, void *cpu_addr, dma_addr_t dma);
	/* optional */
	int (*xfer_run_stop)(struct dwc3_ebc_ext *ext, unsigned ep, bool run);
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
	if (!ext->trb_pool_alloc != !ext->trb_pool_free)
		/* alloc & free: supply both or none */
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

/**
 * check if extension uses external trb pool
 * @ext: ebc extension
 *
 * return true if uses external pool
 */
static inline bool dwc3_ebc_ext_trb_pool_needed(
	struct dwc3_ebc_ext *ext)
{
	return ext && ext->trb_pool_alloc && ext->trb_pool_free;
}

/**
 * alloc external trb pool
 * maybe call from interrupt context
 * @ext: ebc extension
 * @dev: current device
 * @size: allocation size
 * @dma: dma handler (output)
 * @flags: flags for allocation
 *
 * return cpu addr to external pool or NULL if error
 */
static inline void *dwc3_ebc_ext_trb_pool_alloc(
	struct dwc3_ebc_ext *ext, struct device *dev,
	size_t size, dma_addr_t *dma, gfp_t flags)
{
	if (ext && ext->trb_pool_alloc)
		return ext->trb_pool_alloc(ext, dev, size, dma, flags);
	return NULL;
}

/**
 * free external trb pool
 * maybe call from interrupt context
 * @ext: ebc extension
 * @dev: current device
 * @size: allocated size
 * @cpu_addr: cpu pointer
 * @dma: allocated handler
 */
static inline void dwc3_ebc_ext_trb_pool_free(
	struct dwc3_ebc_ext *ext, struct device *dev,
	size_t size, void *cpu_addr, dma_addr_t dma)
{
	if (ext && ext->trb_pool_free)
		ext->trb_pool_free(ext, dev, size, cpu_addr, dma);
}

/**
 * request extension to start transfers
 * @ext: ebc extension
 * @ep: endpoint number
 *
 * return zero on success, else negative errno
 */
static inline int dwc3_ebc_ext_xfer_start(
	struct dwc3_ebc_ext *ext, unsigned ep)
{
	if (ext && ext->xfer_run_stop)
		return ext->xfer_run_stop(ext, ep, true);
	return -EOPNOTSUPP;
}

/**
 * request extension to stop transfers
 * @ext: ebc extension
 * @ep: endpoint number
 *
 * return zero on success, else negative errno
 */
static inline int dwc3_ebc_ext_xfer_stop(
	struct dwc3_ebc_ext *ext, unsigned ep)
{
	if (ext && ext->xfer_run_stop)
		return ext->xfer_run_stop(ext, ep, false);
	return -EOPNOTSUPP;
}

#endif /* __LINUX_USB_DWC3_EXTENSION_H */
