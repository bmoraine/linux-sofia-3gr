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
#include <linux/usb.h>
#include <linux/usb/hcd.h>

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

/*----------------------------------------------------------------------*/
/*  A-DEVICE timing constants in millseconds				*/
/*--------------------------------------------------------------------- */

/* Wait for VBUS Rise  */
/* a_wait_vrise 100 ms, otg rev 1.0, section: 5.5*/
#define TA_WAIT_VRISE	(100)

/* Wait for B-Connect */
/* a_wait_bcon > 1 sec to infinite, otg rev 2.0 1.1a, section 5.5*/
#define TA_WAIT_BCON	(10000)

/* Wait for VFALL */
/* a_wait_vfall < 1 sec, otg rev 2.0 1.1a, section: 4.4 */
#define TSSEND_LKG	(100)

/* SE0 time before SRP */
/* b_se0_srp > 1 sec, otg rev 2.0 1.1a, section 5.5 */
#define TB_SE0_SRP	(1000)

/* SRP fail time */
/* b_srp_fail > 5 sec to 6 sec, otg rev 2.0 1.1a, sections 5.5 */
#define TB_SRP_FAIL	(5000)

/* A-SE0 to B-Reset */
/* b_ase0_brst > 155ms, otg rev 2.0 1.1a, sections 5.5*/
#define TB_ASE0_BRST	(155)

/* A-Idle to B-Disconnect */
/* a_aidl_bdis > 200ms, otg rev 2.0 1.1a, sections 5.5 */
#define TA_AIDL_BDIS	(200)

/* B-Idle to A-Disconnect */
/* a_bidl_adis > 150ms to 200ms, otg rev 2.1 1.1a, sections 5.5 */
#define TA_BIDL_ADIS	(155)


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
#define VBUS_ERR			(4 * 4)
#define B_CONN				(5 * 4)
	struct usb_bus			*input_host;
	struct usb_gadget		*input_gadget;
	struct timer_list		tmr[NUM_OTG_FSM_TIMERS];
	enum otg_fsm_timer		timer;
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

static inline struct usb_bus *intel_phy_get_host(struct intel_phy *iphy)
{
	return iphy ? iphy->otg.host : NULL;
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
