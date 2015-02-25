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
/*----------------------------------------------------------------------*/
/* INCLUDE								*/
/*----------------------------------------------------------------------*/
#include <linux/usb/debug.h>

/*----------------------------------------------------------------------*/
/* MODULE DATA								*/
/*----------------------------------------------------------------------*/
static struct dvc_trace_dev {
	struct usb_function	function;
	struct usb_ep		*ep_in;
	struct usb_request	*req;
	unsigned long		addr;	/* todo: change via attr interface */
	size_t			size;	/* todo: change via attr interface */
} dvc_trace_dev;

static inline struct dvc_trace_dev *func_to_dvc_trace(struct usb_function *f)
{
	return container_of(f, struct dvc_trace_dev, function);
}

/* set to non-zero to automatically start transfers */
#define	DVC_TRACE_BUF_ADDR	(0xE9100000UL)
#define	DVC_TRACE_BUF_SIZE	(1024UL)

#define dvc_trace_printk(lvl, fmt, args...) \
	printk(lvl "[%s] " fmt "\n", __func__, ## args)
#define dvc_trace_err(fmt, args...)  \
	dvc_trace_printk(KERN_ERR, fmt, ##args)
#define dvc_trace_warn(fmt, args...) \
	dvc_trace_printk(KERN_WARNING, fmt, ##args)
#define dvc_trace_info(fmt, args...) \
	dvc_trace_printk(KERN_INFO, fmt, ##args)
#define dvc_trace_dbg(fmt, args...)  \
	dvc_trace_printk(KERN_DEBUG, fmt, ##args)

/*----------------------------------------------------------------------*/
/* USB DESCRIPTORS							*/
/*----------------------------------------------------------------------*/
static struct usb_interface_assoc_descriptor dvc_trace_iad_desc	= {
	.bLength		= sizeof(dvc_trace_iad_desc),
	.bDescriptorType	= USB_DT_INTERFACE_ASSOCIATION,
	/* .bFirstInterface	= DYNAMIC, */
	.bInterfaceCount	= 2, /*	debug control +	data */
	.bFunctionClass		= USB_CLASS_DEBUG,
	.bFunctionSubClass	= USB_SUBCLASS_DVC_TRACE,
	/* .bFunctionProtocol	= 0, */
	/* .iFunction		= DYNAMIC, */
};

static struct usb_interface_descriptor dvc_trace_ctrl_intf_desc = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bNumEndpoints		= 0,
	.bInterfaceClass	= USB_CLASS_DEBUG,
	.bInterfaceSubClass	= USB_SUBCLASS_DEBUG_CONTROL,
	/* .bInterfaceProtocol	= 0, */
};

DECLARE_DC_DEBUG_ATTR_DESCR(DVC_TRACE, 2, 32);
#define	DC_DVC_TRACE_ATTRI_SIZE		DC_DBG_ATTRI_SIZE(2, 32)
#define	DC_DVC_TRACE_TOTAL_LENGTH	(DC_DVC_TRACE_ATTRI_SIZE)

static struct DC_DEBUG_ATTR_DESCR(DVC_TRACE) dvc_trace_dbg_attri_desc = {
	.bLength		= DC_DVC_TRACE_ATTRI_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubtype	= DC_DEBUG_ATTRIBUTES,
	.bcdDC			= cpu_to_le16(0x0100),
	.wTotalLength		= cpu_to_le16(DC_DVC_TRACE_TOTAL_LENGTH),
	.bmSupportedFeatures	= 0,	/* Debug Event not supported */
	.bControlSize		= 2,
	.bmControl		= {	/* Debug-Control not supported */
		[0]		= 0x00,
		[1]		= 0x00,
	},
	.wAuxDataSize		= cpu_to_le16(0x20),
	.dInputBufferSize	= cpu_to_le32(0x00), /*	per SAS	*/
	.dOutputBufferSize	= cpu_to_le32(DVC_TRACE_BUF_SIZE),
	.qBaseAddress		= 0,	/* revision */
	.hGlobalID		= {	/* revision */
		[0]		= 0,
		[1]		= 0,
	}
};

static struct usb_interface_descriptor dvc_trace_data_intf_desc = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bAlternateSetting	= 0,
	.bNumEndpoints		= 1,
	.bInterfaceClass	= USB_CLASS_DEBUG,
	.bInterfaceSubClass	= USB_SUBCLASS_DVC_TRACE,
	/* .bInterfaceProtocol	= 0, */
};

static struct usb_endpoint_descriptor dvc_trace_fullspeed_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor dvc_trace_highspeed_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_endpoint_descriptor dvc_trace_superspeed_desc	= {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor	dvc_trace_superspeed_comp_desc = {
	.bLength		= USB_DT_SS_EP_COMP_SIZE,
	.bDescriptorType	= USB_DT_SS_ENDPOINT_COMP,
	.bMaxBurst		= 0,
	.bmAttributes		= 0,
};

static struct usb_descriptor_header *dvc_trace_fs_descs[] = {
	(struct	usb_descriptor_header *) &dvc_trace_iad_desc,
	(struct	usb_descriptor_header *) &dvc_trace_data_intf_desc,
	(struct	usb_descriptor_header *) &dvc_trace_fullspeed_desc,
	(struct	usb_descriptor_header *) &dvc_trace_ctrl_intf_desc,
	(struct	usb_descriptor_header *) &dvc_trace_dbg_attri_desc,
	NULL,
};

static struct usb_descriptor_header *dvc_trace_hs_descs[] = {
	(struct	usb_descriptor_header *) &dvc_trace_iad_desc,
	(struct	usb_descriptor_header *) &dvc_trace_data_intf_desc,
	(struct	usb_descriptor_header *) &dvc_trace_highspeed_desc,
	(struct	usb_descriptor_header *) &dvc_trace_ctrl_intf_desc,
	(struct	usb_descriptor_header *) &dvc_trace_dbg_attri_desc,
	NULL,
};

static struct usb_descriptor_header *dvc_trace_ss_descs[] = {
	(struct	usb_descriptor_header *) &dvc_trace_iad_desc,
	(struct	usb_descriptor_header *) &dvc_trace_data_intf_desc,
	(struct	usb_descriptor_header *) &dvc_trace_superspeed_desc,
	(struct	usb_descriptor_header *) &dvc_trace_superspeed_comp_desc,
	(struct	usb_descriptor_header *) &dvc_trace_ctrl_intf_desc,
	(struct	usb_descriptor_header *) &dvc_trace_dbg_attri_desc,
	NULL,
};

/* string descriptors: */
#define	DVC_TRACE_CTRL_IDX     0
#define	DVC_TRACE_DATA_IDX     1
#define	DVC_TRACE_IAD_IDX      2

/* static strings, in UTF-8 */
static struct usb_string dvc_trace_string_defs[] = {
	[DVC_TRACE_CTRL_IDX].s = "Debug Sub-Class DvC.Trace (Control)",
	[DVC_TRACE_DATA_IDX].s = "Debug Sub-Class DvC.Trace (Data)",
	[DVC_TRACE_IAD_IDX].s =	"Debug Sub-Class DvC.Trace",
	{  /* ZEROES END LIST */ },
};

static struct usb_gadget_strings dvc_trace_string_table	= {
	.language =	0x0409,	/* en-us */
	.strings =	dvc_trace_string_defs,
};

static struct usb_gadget_strings *dvc_trace_strings[] =	{
	&dvc_trace_string_table,
	NULL,
};

/**
 * completion handler, it always run isr
 */
static void dvc_trace_complete(
	struct usb_ep *ep,
	struct usb_request *req)
{
	int ret = 0;


	if (!ep || !req) {
		dvc_trace_err("invalid parameter");
		return;
	}

	dvc_trace_info("%s --> %d, %u/%u", ep->name,
			req->status, req->actual, req->length);

	if (req->status < 0) {
		if (-ESHUTDOWN != req->status && -ECONNRESET != req->status)
			dvc_trace_err("request error on %s: %d",
				ep->name, req->status);
		if (-ESHUTDOWN != req->status)
			usb_ep_fifo_flush(ep);
		return;
	}

	ret = usb_ep_queue(ep, req, GFP_KERNEL);
	if (ret < 0) {
		dvc_trace_err("queue error on %s: %d", ep->name, ret);
		return;
	}

	dvc_trace_dbg("new request queued on %s", ep->name);
}

/**
 * todo: comment
 */
static int dvc_trace_start_transfers(
	struct dvc_trace_dev *dev,
	unsigned long addr, size_t size)
{
	int ret = 0;


	if (!dev || !dev->ep_in || !dev->req) {
		dvc_trace_err("invalid parameter");
		return -EINVAL;
	}

	if (dev->addr && dev->size)
		usb_ep_dequeue(dev->ep_in, dev->req);

	dev->addr = addr;
	dev->size = size;
	if (!dev->addr || !dev->size) {
		dvc_trace_info("stop transfers on %s, addr: %08lX, size: %d",
				dev->ep_in->name, dev->addr, dev->size);
		return 0;
	}

	dev->req->buf = 0;
	dev->req->length = dev->size;
	dev->req->dma = dev->addr;

	ret = usb_ep_queue(dev->ep_in, dev->req, GFP_KERNEL);
	if (ret < 0) {
		dvc_trace_err("queue error on %s: %d", dev->ep_in->name, ret);
		return ret;
	}

	return 0;
}

/**
 * todo: comment
 */
static int dvc_trace_function_bind(
	struct usb_configuration *c,
	struct usb_function *f)
{
	struct dvc_trace_dev *dev = NULL;
	struct usb_string *us = NULL;
	int ret	= 0;


	if (!c || !f) {
		dvc_trace_err("invalid parameter");
		return -EINVAL;
	}
	dev = func_to_dvc_trace(f);
	if (!dev) {
		dvc_trace_err("initialization issue");
		return -EINVAL;
	}

	if (dev->ep_in || dev->req) {
		dvc_trace_err("nested bind, unbind 1st, ep_in: %p req: %p",
				dev->ep_in, dev->req);
		return -EINVAL;
	}

	/* maybe allocate device-global string IDs, and patch descriptors */
	us = usb_gstrings_attach(c->cdev, dvc_trace_strings,
			ARRAY_SIZE(dvc_trace_string_defs));
	if (IS_ERR(us)) {
		dvc_trace_err("usb get strings failed: %ld", PTR_ERR(us));
		return PTR_ERR(us);
	}
	dvc_trace_data_intf_desc.iInterface = us[DVC_TRACE_DATA_IDX].id;
	dvc_trace_ctrl_intf_desc.iInterface = us[DVC_TRACE_CTRL_IDX].id;
	dvc_trace_iad_desc.iFunction = us[DVC_TRACE_IAD_IDX].id;

	/* allocate instance-specific interface IDs, and patch descriptors */
	ret = usb_interface_id(c, f);
	if (ret	< 0) {
		dvc_trace_err("usb get interface failed for id: %d", ret);
		return ret;
	}
	dvc_trace_data_intf_desc.bInterfaceNumber = ret;
	dvc_trace_iad_desc.bFirstInterface = ret;

	ret = usb_interface_id(c, f);
	if (ret	< 0) {
		dvc_trace_err("usb get interface failed for id: %d", ret);
		return ret;
	}
	dvc_trace_ctrl_intf_desc.bInterfaceNumber = ret;

	/* allocate instance-specific endpoints */
	dev->ep_in = usb_ep_autoconfig_ss(c->cdev->gadget,
					&dvc_trace_fullspeed_desc,
					&dvc_trace_superspeed_comp_desc);
	if (!dev->ep_in) {
		dvc_trace_err("usb ep autoconfig failed");
		return -ENODEV;
	}
	dev->ep_in->driver_data	= dev;	/* claim */

	dev->req = usb_ep_alloc_request(dev->ep_in, GFP_ATOMIC);
	if (!dev->req) {
		dvc_trace_err("usb ep alloc request failed for %s",
				dev->ep_in->name);
		dev->ep_in->driver_data = NULL;
		return -ENOMEM;
	}
	dev->req->complete = dvc_trace_complete;
	dev->req->context = dev;

	dvc_trace_highspeed_desc.bEndpointAddress =
		dvc_trace_fullspeed_desc.bEndpointAddress;
	dvc_trace_superspeed_desc.bEndpointAddress =
		dvc_trace_fullspeed_desc.bEndpointAddress;
	ret = usb_assign_descriptors(f,
		dvc_trace_fs_descs, dvc_trace_hs_descs, dvc_trace_ss_descs);
	if (ret) {
		dvc_trace_err("usb assign descriptors failed: %d", ret);
		usb_ep_free_request(dev->ep_in, dev->req);
		dev->ep_in->driver_data = NULL;
		return ret;
	}

	return 0;
}

/**
 * todo: comment
 */
static void dvc_trace_function_unbind(
	struct usb_configuration *c,
	struct usb_function *f)
{
	struct dvc_trace_dev *dev = NULL;


	if (!c || !f) {
		dvc_trace_err("invalid parameter");
		return;
	}
	dev = func_to_dvc_trace(f);
	if (!dev) {
		dvc_trace_err("initialization issue");
		return;
	}

	dvc_trace_string_defs[DVC_TRACE_CTRL_IDX].id = 0;
	usb_free_all_descriptors(f);
	usb_ep_free_request(dev->ep_in, dev->req);
	dev->req = NULL;
	dev->ep_in->driver_data = NULL;
	dev->ep_in = NULL;
}

/**
 * todo: comment
 */
static int dvc_trace_function_set_alt(
	struct usb_function *f,
	unsigned intf, unsigned	alt)
{
	struct dvc_trace_dev *dev = NULL;


	if (!f || !f->config || !f->config->cdev) {
		dvc_trace_err("invalid parameter");
		return -EINVAL;
	}
	dev = func_to_dvc_trace(f);
	if (!dev) {
		dvc_trace_err("initialization issue");
		return -EINVAL;
	}

	if (intf == dvc_trace_data_intf_desc.bInterfaceNumber) {
		int ret = config_ep_by_speed(f->config->cdev->gadget,
						f, dev->ep_in);
		if (ret < 0) {
			dvc_trace_err("intf: %d alt: %d config_ep_by_speed: %d",
					intf, alt, ret);
			return ret;
		}
		/* HACK gadget driver data should be for internal usage only */
		/* but dwc3 needs to know this ep will be used for dvc trace */
		dev->ep_in->driver_data	= &dvc_trace_data_intf_desc;
		ret = usb_ep_enable(dev->ep_in);
		if (ret < 0) {
			dvc_trace_err("intf: %d alt: %d usb_ep_enable: %d",
					intf, alt, ret);
			return ret;
		}
		dvc_trace_info("intf: %d alt: %d ep %s enabled",
				intf, alt, dev->ep_in->name);

		ret = dvc_trace_start_transfers(dev, DVC_TRACE_BUF_ADDR,
						DVC_TRACE_BUF_SIZE);
		if (ret < 0) {
			dvc_trace_err("cannot start transfers on %s: %d",
					dev->ep_in->name, ret);
			return ret;
		}
	}
	return 0;
}

/**
 * todo: comment
 */
static int dvc_trace_function_setup(
	struct usb_function *f,
	const struct usb_ctrlrequest *c)
{
	if (!f || !c) {
		dvc_trace_err("invalid parameter");
		return -EINVAL;
	}

	dvc_trace_warn("%02x.%02x v%04x i%04x l%u",
			c->bRequestType, c->bRequest,
			le16_to_cpu(c->wValue),
			le16_to_cpu(c->wIndex),
			le16_to_cpu(c->wLength));
	return -EOPNOTSUPP;
}

/**
 * todo: comment
 */
static void dvc_trace_function_disable(
	struct usb_function *f)
{
	struct dvc_trace_dev *dev = NULL;


	if (!f) {
		dvc_trace_err("invalid parameter");
		return;
	}
	dev = func_to_dvc_trace(f);
	if (!dev) {
		dvc_trace_err("initialization issue");
		return;
	}

	usb_ep_disable(dev->ep_in);
	dev->ep_in->driver_data = NULL;
}

/*----------------------------------------------------------------------*/
/* ANDROID INTERFACE							*/
/*----------------------------------------------------------------------*/
/**
 * todo: comment
 */
static int dvc_trace_bind_config(
	struct usb_configuration *c)
{
	struct dvc_trace_dev *dev = NULL;


	if (!c) {
		dvc_trace_err("invalid parameter");
		return -EINVAL;
	}
	dev = &dvc_trace_dev;	/* todo: get it dynamically */
	if (!dev) {
		dvc_trace_err("initialization issue");
		return -EINVAL;
	}

	dev->function.name = "dvc_trace";
	dev->function.strings =	dvc_trace_strings;
	dev->function.fs_descriptors = dvc_trace_fs_descs;
	dev->function.hs_descriptors = dvc_trace_hs_descs;
	dev->function.ss_descriptors = dvc_trace_ss_descs;
	dev->function.bind = dvc_trace_function_bind;
	dev->function.unbind = dvc_trace_function_unbind;
	dev->function.set_alt =	dvc_trace_function_set_alt;
	dev->function.setup = dvc_trace_function_setup;
	dev->function.disable =	dvc_trace_function_disable;
	return usb_add_function(c, &dev->function);
}
