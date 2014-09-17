/****************************************************************
 *
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 *  This program is free software: you can redistribute	it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the	Free Software Foundation.
 *
 *  This program is distributed	in the hope that it will be useful,
 *  but	WITHOUT	ANY WARRANTY; without even the implied warranty	of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR	PURPOSE.
 *
 *  You	should have received a copy of the GNU General Public License Version 2
 *  along with this program. If	not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef	PHY_INTEL_USB_H
#define	PHY_INTEL_USB_H

/*----------------------------------------------------------------------*/
/* INCLUDE								*/
/*----------------------------------------------------------------------*/
#include <linux/power_supply.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/usb/otg-fsm.h>

/*----------------------------------------------------------------------*/
/* DEFINE								*/
/*----------------------------------------------------------------------*/
#define intel_phy_printk(lvl, fmt, args...) \
	printk(lvl "[%s] " fmt "\n", __func__, ## args)
#define intel_phy_err(fmt, args...)  \
	intel_phy_printk(KERN_ERR, fmt, ##args)
#define intel_phy_warn(fmt, args...) \
	intel_phy_printk(KERN_WARNING, fmt, ##args)
#define intel_phy_info(fmt, args...) \
	intel_phy_printk(KERN_INFO, fmt, ##args)
#define intel_phy_dbg(fmt, args...)  \
	intel_phy_printk(KERN_DEBUG, fmt, ##args)

struct intel_phy {
	spinlock_t			lck;
	struct usb_phy			phy;
	struct usb_otg			otg;
	struct otg_fsm			fsm;
	struct notifier_block		event_nb;
	struct delayed_work		event_wq;
	unsigned long			input;
#define EVENT				(0 * 4)
#define NONE				(1 * 4)
#define VBUS				(2 * 4)
#define ID				(3 * 4)
	struct usb_bus			*input_host;
	struct usb_gadget		*input_gadget;
	struct power_supply_cable_props	input_props;
	struct power_supply_cable_props	cable_props;
	struct dentry			*debugfs_root;
};

/*-----------------------------------------------------------------------*/
/* FUNCTIONS								 */
/*-----------------------------------------------------------------------*/
static inline struct usb_gadget *intel_phy_get_gadget(struct intel_phy *iphy)
{
	return iphy ? iphy->otg.gadget : NULL;
}

static inline enum usb_device_speed intel_phy_get_speed(struct intel_phy *iphy)
{
	if (IS_ERR_OR_NULL(iphy) || IS_ERR_OR_NULL(iphy->otg.gadget))
		return USB_SPEED_UNKNOWN;
	return iphy->otg.gadget->speed;
}

int intel_phy_init(struct device *dev, struct intel_phy *iphy,
	struct otg_fsm_ops *otg_fsm_ops, enum usb_phy_type type);
int intel_phy_exit(struct device *dev, struct intel_phy *iphy);
int intel_phy_notify(
	struct intel_phy *iphy, enum usb_phy_events event, void *priv);
int intel_phy_kernel_trap(void);
int intel_phy_kernel_trap_enable(bool value);

#endif /* PHY_INTEL_USB_H */

/*-----------------------------------------------------------------------*/
/* FILE END								 */
/*-----------------------------------------------------------------------*/
