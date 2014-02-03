#ifndef _DWC_OS_DEP_H_
#define _DWC_OS_DEP_H_

/**
 * @file
 *
 * This file contains OS dependent structures.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/dma-mapping.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/stat.h>
#include <linux/pci.h>
#include <linux/completion.h>
#include <linux/version.h>
#include <linux/irq.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>


/** The OS page size */
#define DWC_OS_PAGE_SIZE	PAGE_SIZE

typedef struct os_dependent {
	/** Base address returned from ioremap() */
	void *base;

	/** Register offset for Diagnostic API */
	uint32_t reg_offset;

	struct platform_device *dev;
} os_dependent_t;

#ifdef __cplusplus
}
#endif

#endif /* _DWC_OS_DEP_H_ */
