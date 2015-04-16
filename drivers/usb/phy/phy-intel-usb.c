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

/*----------------------------------------------------------------------*/
/* INCLUDE								*/
/*----------------------------------------------------------------------*/
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/usb/phy-intel.h>
#include "phy-intel-usb.h"

/*----------------------------------------------------------------------*/
/* DEFINE								*/
/*----------------------------------------------------------------------*/
#define INTEL_PHY_CHRG_IUNIT	(100)		  /* mA */
#define INTEL_PHY_CHRG_ICFG_MAX	(500)		  /* mA */
#define INTEL_PHY_CHRG_IDEV_CHG	(1500)		  /* mA */

static bool kernel_trap = true;

/*----------------------------------------------------------------------*/
/* PROTOTYPES								*/
/*----------------------------------------------------------------------*/
static int intel_phy_event_nb(
	struct notifier_block *nb, unsigned long event,	void *priv);
static int intel_phy_otg_set_host(
	struct usb_otg *otg, struct usb_bus *host);
static int intel_phy_otg_set_peripheral(
	struct usb_otg *otg, struct usb_gadget *gadget);
static void intel_phy_otg_fsm_drv_vbus(
	struct otg_fsm *fsm, int on);
static void intel_phy_otg_fsm_add_timer(
	struct otg_fsm *fsm, enum otg_fsm_timer timer);
static void intel_phy_otg_fsm_del_timer(
	struct otg_fsm *fsm, enum otg_fsm_timer timer);
static int intel_phy_set_vbus(struct usb_phy *phy, int on);
static int intel_phy_set_power(struct usb_phy *phy, unsigned mA);
static int intel_phy_notify_connect(
	struct usb_phy *phy, enum usb_device_speed speed);
static int intel_phy_notify_disconnect(
	struct usb_phy *phy, enum usb_device_speed speed);
static int intel_phy_debugfs_init(struct intel_phy *iphy);
static int intel_phy_debugfs_exit(struct intel_phy *iphy);

/*----------------------------------------------------------------------*/
/* LOCAL - UTILITIES							*/
/*----------------------------------------------------------------------*/
static inline spinlock_t *intel_phy_get_lck(struct intel_phy *iphy)
{
	return iphy ? &iphy->lck : NULL;
}

static inline struct usb_phy *intel_phy_get_phy(struct intel_phy *iphy)
{
	return iphy ? &iphy->phy : NULL;
}

static enum usb_otg_state intel_phy_get_state(struct intel_phy *iphy)
{
	return iphy ? iphy->phy.state : OTG_STATE_UNDEFINED;
}

static const char *intel_phy_get_state_string(struct intel_phy *iphy)
{
	return usb_otg_state_string(intel_phy_get_state(iphy));
}

static inline struct usb_otg *intel_phy_get_otg(struct intel_phy *iphy)
{
	return iphy ? iphy->phy.otg : NULL;
}

static inline struct otg_fsm *intel_phy_get_fsm(struct intel_phy *iphy)
{
	return iphy ? &iphy->fsm : NULL;
}

static inline unsigned long *intel_phy_get_input(struct intel_phy *iphy)
{
	return iphy ? &iphy->input : NULL;
}

/**
 * @todo: comment
 */
static bool intel_phy_chrg_enumerate(
	enum power_supply_charger_cable_type chrg_type)
{
	bool enumerate = false;

	enumerate |= POWER_SUPPLY_CHARGER_TYPE_USB_CDP == chrg_type;
	enumerate |= POWER_SUPPLY_CHARGER_TYPE_USB_SDP == chrg_type;
	return enumerate;
}

/**
 * @todo: comment
 */
static const char *intel_phy_chrg_to_str(
	enum power_supply_charger_cable_type chrg_type)
{
	switch (chrg_type) {
	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		return "CHARGER_USB_CDP";
	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		return "CHARGER_USB_DCP";
	case POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
		return "CHARGER_USB_ACA";
	case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
		return "CHARGER_USB_SDP";
	case POWER_SUPPLY_CHARGER_TYPE_NONE:
		return "CHARGER_NONE";
	default:
		return "INVALID_CHARGER";
	}
}

/**
 * @todo: comment
 */
static const char *intel_phy_otg_timer_to_str(enum otg_fsm_timer timer)
{
	switch (timer) {
	case A_WAIT_VRISE:
		return "a_wait_vrise_tmr";
	case A_WAIT_VFALL:
		return "a_wait_vfall_tmr";
	case A_WAIT_BCON:
		return "a_wait_bcon_tmr";
	case B_SE0_SRP:
		return "b_se0_srp";
	case B_SRP_FAIL:
		return "b_srp_fail";
	case B_ASE0_BRST:
		return "b_ase0_brst";
	case A_AIDL_BDIS:
		return "a_aidl_bdis";
	case A_BIDL_ADIS:
		return "a_bidl_adis";
	default:
		return "invalid timer";
	}
}

/**
 * @todo: comment
 */
static void intel_phy_otg_set_tmout(unsigned long data)
{
	if (IS_ERR_OR_NULL((void *)data)) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}
}

/**
 * @todo: comment
 */
static int intel_phy_otg_get_timer_data(
	struct timer_list *tmr, enum otg_fsm_timer timer)
{
	if (IS_ERR_OR_NULL(tmr)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}


	/* TODO: upgrade specific tmr->function when actual
	 * timer handling is required */
	switch (timer) {
	case A_WAIT_VRISE:
		tmr->expires = jiffies + msecs_to_jiffies(TA_WAIT_VRISE);
		break;
	case A_WAIT_VFALL:
		tmr->expires = jiffies + msecs_to_jiffies(TSSEND_LKG);
		break;
	case A_WAIT_BCON:
		tmr->expires = jiffies + msecs_to_jiffies(TA_WAIT_BCON);
		break;
	case B_SE0_SRP:
		tmr->expires = jiffies + msecs_to_jiffies(TB_SE0_SRP);
		break;
	case B_SRP_FAIL:
		tmr->expires = jiffies + msecs_to_jiffies(TB_SRP_FAIL);
		break;
	case B_ASE0_BRST:
		tmr->expires = jiffies + msecs_to_jiffies(TB_ASE0_BRST);
		break;
	case A_AIDL_BDIS:
		tmr->expires = jiffies + msecs_to_jiffies(TA_AIDL_BDIS);
		break;
	case A_BIDL_ADIS:
		tmr->expires = jiffies + msecs_to_jiffies(TA_BIDL_ADIS);
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

/*----------------------------------------------------------------------*/
/* LOCAL - MISCELLANEOUS						*/
/*----------------------------------------------------------------------*/
/**
 * initialize driver data
 */
static int intel_phy_data_init(struct intel_phy *iphy,
	struct device *dev, struct otg_fsm_ops *otg_fsm_ops)
{
	if (IS_ERR_OR_NULL(dev) || IS_ERR_OR_NULL(iphy)
	|| (IS_ERR_OR_NULL(otg_fsm_ops))) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	/* initialize PHY structure */
	iphy->phy.dev = dev;
	iphy->phy.label = "intel-phy";
	iphy->phy.state = OTG_STATE_UNDEFINED;
	iphy->phy.last_event = USB_EVENT_NONE;
	iphy->phy.set_vbus = intel_phy_set_vbus;
	iphy->phy.set_power = intel_phy_set_power;
	iphy->phy.notify_connect = intel_phy_notify_connect;
	iphy->phy.notify_disconnect = intel_phy_notify_disconnect;

	/* initialize OTG structure */
	iphy->phy.otg = &iphy->otg;
	iphy->phy.otg->phy = &iphy->phy;
	iphy->phy.otg->set_host = intel_phy_otg_set_host;
	iphy->phy.otg->set_peripheral = intel_phy_otg_set_peripheral;
	iphy->phy.otg->start_hnp = NULL;	/* add for OTG HNP support */
	iphy->phy.otg->start_srp = NULL;	/* add for OTG SRP support */

	/* initialize OTG FSM structure */
	iphy->fsm.otg = iphy->phy.otg;
	iphy->fsm.ops = otg_fsm_ops;
	iphy->fsm.ops->drv_vbus = intel_phy_otg_fsm_drv_vbus;
	iphy->fsm.ops->add_timer = intel_phy_otg_fsm_add_timer;
	iphy->fsm.ops->del_timer = intel_phy_otg_fsm_del_timer;
	iphy->fsm.id = 1;
	iphy->fsm.b_sess_vld = 0;
	mutex_init(&iphy->fsm.lock);

	/* initialize INTEL structure */
	spin_lock_init(intel_phy_get_lck(iphy));
	iphy->event_nb.notifier_call = intel_phy_event_nb;
	set_bit(ID, intel_phy_get_input(iphy));	/* default B: not grounded */
	iphy->input_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
	iphy->input_props.ma = 0;
	iphy->cable_props = iphy->input_props;
	return 0;
}

/**
 * should be atomic context
 */
static int intel_phy_otg_fsm_pre_process_chrg(struct intel_phy *iphy)
{
	enum usb_phy_events event = USB_EVENT_NONE;
	bool enumerate = false;


	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	switch (iphy->input_props.chrg_type) {
	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		/* the configuration desc. requests CONFIG_USB_GADGET_VBUS_DRAW
		 * so when stack asks it the device was successfully enumerated
		 * however if host is a CDP, then device can charge IDEV_CHG */
		if (iphy->input_props.ma == CONFIG_USB_GADGET_VBUS_DRAW)
			iphy->input_props.ma = INTEL_PHY_CHRG_IDEV_CHG;
		/* fall through */
	case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
		if (iphy->input_props.ma != 0)
			event = USB_EVENT_ENUMERATED;
		enumerate = true;
		break;
	case POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
		/* fall through */
	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		iphy->input_props.ma = INTEL_PHY_CHRG_IDEV_CHG;
		event = USB_EVENT_CHARGER;
		break;
	case POWER_SUPPLY_CHARGER_TYPE_NONE:
		iphy->input_props.ma = 0;
		/* todo: start charger detection */
		break;
	default:
		intel_phy_err("charger type not supported");
		return intel_phy_kernel_trap();
	}

	iphy->cable_props = iphy->input_props;

	/* return if no event to report */
	if (USB_EVENT_NONE == event)
		goto done;

	intel_phy_notify(iphy, event, &iphy->cable_props);
	intel_phy_info("%s, %d mA",
			intel_phy_chrg_to_str(iphy->cable_props.chrg_type),
			iphy->cable_props.ma);
done:
	return 0;
}

/**
 * OTG FSM pre process
 * Return: 0 to execute the OTG state machine, -EPERM to avoid
 * executing OTG state machine
 */
static int intel_phy_otg_fsm_pre_process(struct intel_phy *iphy)
{
	struct usb_otg *otg = intel_phy_get_otg(iphy);
	struct otg_fsm *fsm = intel_phy_get_fsm(iphy);
	unsigned long *input = NULL;
	unsigned long flags = 0;


	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	spin_lock_irqsave(intel_phy_get_lck(iphy), flags);

	input = intel_phy_get_input(iphy);

	/* bind change can only occur while detached */
	if (otg->host != iphy->input_host) {
		if (!fsm->id)
			set_bit(NONE, input);
		else
			otg->host = iphy->input_host;
	}
	if (otg->gadget != iphy->input_gadget) {
		if (fsm->b_sess_vld)
			set_bit(NONE, input);
		else
			otg->gadget = iphy->input_gadget;
	}

	/* at least one detach or re-bind occurred so force detach first */
	if (test_bit(NONE, input)) {
		fsm->id = 1;
		fsm->b_sess_vld = 0;
		fsm->a_bus_drop = 1;
		fsm->a_bus_req = 0;
		set_bit(EVENT, input);
		if (OTG_STATE_UNDEFINED != otg->phy->state)
			clear_bit(NONE, input);
	} else {
		fsm->id         = test_bit(ID,   input);
		fsm->b_sess_vld = test_bit(VBUS, input);
	}

	switch (otg->phy->state) {
	case OTG_STATE_UNDEFINED:
		/* if ID or NONE during boot up then power
		 * off GADGET, if VBUS then dont power off GADGET
		 * and continue fsm execution by forcing it else
		 * skip fsm execution */
		if (!test_bit(ID, input)) {
			fsm->protocol = PROTO_GADGET;
			/* refer OTG rev 2.0 1.1a, section 7.1 for A-device */
			set_bit(EVENT, input);
		} else if (test_and_clear_bit(NONE, input))
			fsm->protocol = PROTO_GADGET;
		else if (test_bit(VBUS, input)) {
			set_bit(EVENT, input);
			/* force USB power off if non-enumerate charger */
			if (!intel_phy_chrg_enumerate(
				iphy->input_props.chrg_type))
				fsm->protocol = PROTO_GADGET;
		} else
			goto skip1;
		break;
	case OTG_STATE_B_IDLE:
		if (!fsm->id || fsm->b_sess_vld) {
			int ret = 0;
			/* refer OTG rev 2.0 1.1a, section 7.1 for A-device */
			if (!fsm->id)
				set_bit(EVENT, input);
			ret = intel_phy_otg_fsm_pre_process_chrg(iphy);
			if (IS_ERR_VALUE(ret)) {
				intel_phy_err("OTG FSM pre process chg type");
				goto error;
			}
			if (!intel_phy_chrg_enumerate(
				iphy->cable_props.chrg_type)) {
				/* if charger will not enumerate us */
				fsm->b_sess_vld = 0;
			}
		}
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (fsm->id && fsm->b_sess_vld) {
			int ret = intel_phy_otg_fsm_pre_process_chrg(iphy);
			if (IS_ERR_VALUE(ret)) {
				intel_phy_err("OTG FSM pre process chg type");
				goto error;
			}
		}
		break;
	case OTG_STATE_A_IDLE:
		if (!fsm->id) {
			fsm->a_bus_drop = 0;
			fsm->a_bus_req = 1;
			set_bit(EVENT, input);
		}
		break;
	case OTG_STATE_A_WAIT_VRISE:
		if (!fsm->id && !fsm->a_bus_drop && fsm->a_bus_req)
			fsm->a_vbus_vld = 1;
		break;
	case OTG_STATE_A_WAIT_BCON:
		if (test_and_clear_bit(VBUS_ERR, input)) {
			/* go to OTG_STATE_A_VBUS_ERR state */
			fsm->a_vbus_vld = 0;
			set_bit(EVENT, input);
		} else if (!fsm->id && fsm->a_vbus_vld)
			fsm->b_conn = test_bit(B_CONN, input);
		else if (fsm->id)
			fsm->b_conn = 0;
		break;
	case OTG_STATE_A_HOST:
		if (test_and_clear_bit(VBUS_ERR, input)) {
			/* go to OTG_STATE_A_VBUS_ERR state */
			fsm->a_vbus_vld = 0;
			fsm->b_conn = 1;
			set_bit(EVENT, input);
		} else if (!fsm->id && fsm->a_vbus_vld)
			fsm->b_conn = test_bit(B_CONN, input);
		break;
	case OTG_STATE_A_WAIT_VFALL:
		/* go to OTG_STATE_B_IDLE */
		fsm->a_wait_vfall_tmout = 1;
		set_bit(EVENT, input);
		break;
	case OTG_STATE_A_VBUS_ERR:
		fsm->a_bus_drop = 1;
		fsm->a_bus_req = 0;
		/* force ID bit to floated and switch to B_IDLE
		 * else we will always request VBUS and fail in case
		 * of VBUS ERR condition */
		set_bit(ID, input);
		set_bit(EVENT, input);
		break;
	default:
		break;
	}

	spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
	return 0;
skip1:
	spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
	return -EPERM;
error:
	spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
	return intel_phy_kernel_trap();
}

/**
 * OTG FSM post process
 */
static int intel_phy_otg_fsm_post_process(struct intel_phy *iphy)
{
	struct usb_phy *phy = intel_phy_get_phy(iphy);


	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	/* pm_runtime_resume */
	switch (phy->state) {
	case OTG_STATE_B_IDLE:
		pm_runtime_put_noidle(phy->dev);
		pm_runtime_suspend(phy->dev);
		break;
	default:
		break;
	}
	return 0;
}

/**
 * @todo: comment
 */
static void intel_phy_event_wq(struct work_struct *work)
{
	struct intel_phy *iphy = NULL;
	struct otg_fsm *fsm = NULL;


	if (IS_ERR_OR_NULL(work)) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}
	iphy = container_of(work, struct intel_phy, event_wq.work);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("initialization issue");
		intel_phy_kernel_trap();
		return;
	}
	fsm = intel_phy_get_fsm(iphy);
	if (IS_ERR_OR_NULL(fsm)) {
		intel_phy_err("initialization issue");
		intel_phy_kernel_trap();
		return;
	}

	pm_runtime_resume(intel_phy_get_phy(iphy)->dev);
	while (test_and_clear_bit(EVENT, intel_phy_get_input(iphy))) {

		intel_phy_dbg("%04X", cpu_to_le32(*intel_phy_get_input(iphy)));

		if (!intel_phy_otg_fsm_pre_process(iphy)) {
			if (otg_statemachine(fsm))
				intel_phy_otg_fsm_post_process(iphy);
		}
	}
}

static void intel_phy_schedule_event_wq(struct intel_phy *iphy)
{
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}

	/* wq must always check for pending events */
	set_bit(EVENT, intel_phy_get_input(iphy));

	/* todo: delayed wq will be used in charger type detection and OTG */
	/* in old kernels version system work queue use one thread per cpu,
	 * so if schedule it in a specific cpu only one thread will run it
	 * todo: check if it is still valid in the curretn kernel version */
	/* no effect if already scheduled */
	schedule_delayed_work_on(0, &iphy->event_wq, 0);
}

/*----------------------------------------------------------------------*/
/* MODULE INTERFACE							*/
/*----------------------------------------------------------------------*/
/**
 * phy event handler - grab the spinlocks only for the
 * events handled in the notifier
 *
 */
static int intel_phy_event_nb(
	struct notifier_block *nb, unsigned long event,	void *priv)
{
	struct intel_phy *iphy = NULL;
	unsigned long *input = NULL;
	unsigned long flags = 0;


	if (IS_ERR_OR_NULL(nb) || IS_ERR_OR_NULL(priv)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	iphy = container_of(nb, struct intel_phy, event_nb);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}
	input = intel_phy_get_input(iphy);
	if (IS_ERR_OR_NULL(input)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	/* atomic notifier might not run in wq cpu */
	/* as agreed: event payload is cable props */
	switch (event) {
	case USB_EVENT_NONE:
		spin_lock_irqsave(intel_phy_get_lck(iphy), flags);
		set_bit(NONE, input);
		clear_bit(VBUS, input);
		clear_bit(VBUS_ERR, input);
		set_bit(ID, input);
		iphy->input_props = *((struct power_supply_cable_props *)priv);
		intel_phy_schedule_event_wq(iphy);
		spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
		break;
	case USB_EVENT_VBUS:
		spin_lock_irqsave(intel_phy_get_lck(iphy), flags);
		if (test_and_set_bit(VBUS, input)) {
			intel_phy_warn("disconnect missed");
			set_bit(NONE, input);
		}
		iphy->input_props = *((struct power_supply_cable_props *)priv);
		intel_phy_schedule_event_wq(iphy);
		spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
		break;
	case USB_EVENT_ID:
		spin_lock_irqsave(intel_phy_get_lck(iphy), flags);
		if (!test_and_clear_bit(ID, input)) {
			intel_phy_warn("disconnect missed");
			set_bit(NONE, input);
		}
		iphy->input_props = *((struct power_supply_cable_props *)priv);
		intel_phy_schedule_event_wq(iphy);
		spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
		break;
	case INTEL_USB_DRV_VBUS_ERR:
		spin_lock_irqsave(intel_phy_get_lck(iphy), flags);
		set_bit(VBUS_ERR, input);
		intel_phy_schedule_event_wq(iphy);
		spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
		break;
	default:
		/* if events generated by this module need to be handled
		 * spinlock should be released before calling notifier */
		break;
	}
	return NOTIFY_OK;
}

/**
 * OTG API: bind/unbind the host controller
 */
static int intel_phy_otg_set_host(
	struct usb_otg *otg, struct usb_bus *host)
{
	struct intel_phy *iphy = NULL;
	unsigned long flags = 0;


	if (IS_ERR_OR_NULL(otg)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	iphy = container_of(otg, struct intel_phy, otg);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	intel_phy_dbg("request host %s", host ? "bind" : "unbind");

	spin_lock_irqsave(intel_phy_get_lck(iphy), flags);
	pm_runtime_get_sync(otg->phy->dev);
	iphy->input_host = host;
	intel_phy_schedule_event_wq(iphy);
	spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
	return 0;
}

/**
 * OTG API: bind/unbind the peripheral controller
 */
static int intel_phy_otg_set_peripheral(
	struct usb_otg *otg, struct usb_gadget *gadget)
{
	struct intel_phy *iphy = NULL;
	unsigned long flags = 0;


	if (IS_ERR_OR_NULL(otg)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	iphy = container_of(otg, struct intel_phy, otg);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	intel_phy_dbg("request gadget %s", gadget ? "bind" : "unbind");

	spin_lock_irqsave(intel_phy_get_lck(iphy), flags);
	pm_runtime_get_sync(otg->phy->dev);
	iphy->input_gadget = gadget;
	intel_phy_schedule_event_wq(iphy);
	spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
	return 0;
}

/**
 * PHY API: vbus on/off
 */
static int intel_phy_set_vbus(struct usb_phy *phy, int on)
{
	struct intel_phy *iphy = NULL;


	if (IS_ERR_OR_NULL(phy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	iphy = container_of(phy, struct intel_phy, phy);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	intel_phy_otg_fsm_drv_vbus(intel_phy_get_fsm(iphy), on);
	return 0;
}

/**
 * PHY API: effective for B devices, ignored for A-peripheral
 */
static int intel_phy_set_power(struct usb_phy *phy, unsigned mA)
{
	struct intel_phy *iphy = NULL;
	unsigned long flags = 0;


	if (IS_ERR_OR_NULL(phy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	iphy = container_of(phy, struct intel_phy, phy);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	intel_phy_dbg("request mA = %u", mA);

	spin_lock_irqsave(intel_phy_get_lck(iphy), flags);
	pm_runtime_get_sync(phy->dev);
	iphy->input_props.ma = mA;
	intel_phy_schedule_event_wq(iphy);
	spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
	return 0;
}

/**
 * PHY API: effective for A-host only
 */
static int intel_phy_notify_connect(
	struct usb_phy *phy, enum usb_device_speed speed)
{
	struct intel_phy *iphy = NULL;
	unsigned long flags = 0;


	if (IS_ERR_OR_NULL(phy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	iphy = container_of(phy, struct intel_phy, phy);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	spin_lock_irqsave(intel_phy_get_lck(iphy), flags);
	set_bit(B_CONN, intel_phy_get_input(iphy));
	intel_phy_schedule_event_wq(iphy);
	spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
	return 0;
}

/**
 * PHY API: effective for A-host only
 */
static int intel_phy_notify_disconnect(
	struct usb_phy *phy, enum usb_device_speed speed)
{
	struct intel_phy *iphy = NULL;
	unsigned long flags = 0;


	if (IS_ERR_OR_NULL(phy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	iphy = container_of(phy, struct intel_phy, phy);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	spin_lock_irqsave(intel_phy_get_lck(iphy), flags);
	clear_bit(B_CONN, intel_phy_get_input(iphy));
	intel_phy_schedule_event_wq(iphy);
	spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
	return 0;
}

/**
 * OTG FSM API: vbus on/off
 */
static void intel_phy_otg_fsm_drv_vbus(struct otg_fsm *fsm, int on)
{
	struct intel_phy *iphy = NULL;
	unsigned long flags = 0;


	if (IS_ERR_OR_NULL(fsm)) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}
	iphy = container_of(fsm, struct intel_phy, fsm);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("initialization issue");
		intel_phy_kernel_trap();
		return;
	}

	intel_phy_dbg("set vbus %s", on ? "on" : "off");

	spin_lock_irqsave(intel_phy_get_lck(iphy), flags);
	intel_phy_notify(iphy, INTEL_USB_DRV_VBUS, &on);
	spin_unlock_irqrestore(intel_phy_get_lck(iphy), flags);
	return;
}

/**
 * OTG FSM API: add timer
 */
static void intel_phy_otg_fsm_add_timer(
	struct otg_fsm *fsm, enum otg_fsm_timer timer)
{
	struct intel_phy *iphy = NULL;
	struct timer_list *tmr = NULL;


	if (IS_ERR_OR_NULL(fsm)) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}
	iphy = container_of(fsm, struct intel_phy, fsm);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}

	if (timer >= NUM_OTG_FSM_TIMERS)
		return;
	tmr = &iphy->tmr[timer];
	if (IS_ERR_OR_NULL(tmr)) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}

	if (timer_pending(tmr)) {
		intel_phy_warn("Timer %s already running",
			intel_phy_otg_timer_to_str(timer));
		return;
	}
	init_timer(tmr);
	tmr->data = (unsigned long)iphy;
	tmr->function = &intel_phy_otg_set_tmout;
	if (intel_phy_otg_get_timer_data(tmr, timer))
		return;
	add_timer(tmr);
	return;
}

/**
 * OTG FSM API: delete timer
 */
static void intel_phy_otg_fsm_del_timer(
	struct otg_fsm *fsm, enum otg_fsm_timer timer)
{
	struct intel_phy *iphy = NULL;
	struct timer_list *tmr = NULL;


	if (IS_ERR_OR_NULL(fsm)) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}
	iphy = container_of(fsm, struct intel_phy, fsm);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}

	if (timer >= NUM_OTG_FSM_TIMERS)
		return;
	tmr = &iphy->tmr[timer];
	if (IS_ERR_OR_NULL(tmr)) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}

	if (timer_pending(tmr))
		del_timer_sync(tmr);
	return;
}

/**
 * to be called by driver probe only
 */
int intel_phy_init(struct device *dev, struct intel_phy *iphy,
	struct otg_fsm_ops *otg_fsm_ops, enum usb_phy_type type)
{
	struct device_pm_platdata *pm_data = NULL;
	int ret = 0;

	if (IS_ERR_OR_NULL(dev) || IS_ERR_OR_NULL(iphy)
	|| (IS_ERR_OR_NULL(otg_fsm_ops))) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	/* initialize phy data */
	ret = intel_phy_data_init(iphy, dev, otg_fsm_ops);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("phy data init");
		goto error_trap;
	}

	/* set phy type */
	ret = usb_add_phy(intel_phy_get_phy(iphy), type);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("usb add phy, %d", ret);
		goto error_trap;
	}

	/* register to platform device pm */
	pm_data = of_device_state_pm_setup(dev->of_node);
	if (IS_ERR_OR_NULL(pm_data)) {
		intel_phy_err("device state pm init");
		goto error_remove_phy;
	}
	ret = device_state_pm_set_class(dev, pm_data->pm_user_name);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("device set pm class, %d", ret);
		goto error_remove_phy;
	}

	/* start listen events */
	INIT_DELAYED_WORK(&iphy->event_wq, intel_phy_event_wq);
	usb_register_notifier(intel_phy_get_phy(iphy), &iphy->event_nb);

	/* pm configuration */
	ret = device_init_wakeup(dev, true);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("device init wakeup, %d", ret);
		goto error_remove_phy;
	}
	ret = pm_runtime_set_active(dev);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("pm runtime set active, %d", ret);
		goto error_remove_phy;
	}
	pm_runtime_enable(dev);

	ret = intel_phy_debugfs_init(iphy);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("debugfs initialization");
		goto error_remove_phy;
	}

	return 0;

error_remove_phy:
	usb_remove_phy(intel_phy_get_phy(iphy));
error_trap:
	return intel_phy_kernel_trap();
}

/**
 * to be called by driver remove only
 */
int intel_phy_exit(struct device *dev, struct intel_phy *iphy)
{
	if (IS_ERR_OR_NULL(dev) || IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	intel_phy_debugfs_exit(iphy);

	pm_runtime_resume(dev);
	pm_runtime_disable(dev);

	usb_unregister_notifier(intel_phy_get_phy(iphy), &iphy->event_nb);
	cancel_delayed_work_sync(&iphy->event_wq);

	usb_remove_phy(intel_phy_get_phy(iphy));

	pm_runtime_set_suspended(dev);
	return 0;
}

/**
 * @todo: comment
 */
int intel_phy_notify(
	struct intel_phy *iphy, enum usb_phy_events event, void *priv)
{
	if (IS_ERR_OR_NULL(iphy) || IS_ERR_OR_NULL(priv)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	atomic_notifier_call_chain(
		&intel_phy_get_phy(iphy)->notifier,
		(unsigned long) event, priv);
	return 0;
}

/**
 * @todo: comment
 */
int intel_phy_kernel_trap(void)
{
	intel_phy_err(">>>>> fatal error <<<<<");
	if (kernel_trap)
		BUG();
	return -1;
}

/**
 * @todo: comment
 */
int intel_phy_kernel_trap_enable(bool value)
{
	kernel_trap = value;
	return 0;
}

/*----------------------------------------------------------------------*/
/* LOCAL - DEBUGFS & TEST						*/
/*----------------------------------------------------------------------*/
static int intel_phy_debugfs_open(struct inode *inode, struct file *file);
static ssize_t intel_phy_debugfs_dbg_read(
	struct file *file, char __user *ubuf, size_t count, loff_t *ppos);
static ssize_t intel_phy_debugfs_dbg_write(
	struct file *file, const char __user *ubuf, size_t count, loff_t *ppos);

/**
 * @todo: comment
 */
static void intel_phy_ms_pause(unsigned ms)
{
	if (ms < 20)
		mdelay(ms);
	else
		msleep(ms);
}

/**
 * @todo: comment
 */
static int intel_phy_test_seven(struct intel_phy *iphy,
	unsigned loop, unsigned t1, unsigned t2)
{
	const unsigned long event[] = {
		USB_EVENT_NONE,
		USB_EVENT_ID,
		INTEL_USB_DRV_VBUS_ERR,
		USB_EVENT_NONE,
		USB_EVENT_VBUS,
		USB_EVENT_NONE,
		USB_EVENT_ID,
		INTEL_USB_DRV_VBUS_ERR,
	};
	unsigned i = 0, j = 0;


	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	for (i = 0; i < loop; i++) {
		intel_phy_info("t1 = %u, initial OTG state: %s", t1,
				intel_phy_get_state_string(iphy));

		for (j = 0; j < ARRAY_SIZE(event); j++) {
			intel_phy_notify(iphy, event[j], &iphy->input_props);
			intel_phy_ms_pause(t1);
			intel_phy_info("after %u ms, current OTG state: %s",
					t1, intel_phy_get_state_string(iphy));

		}
	}
	intel_phy_info("test loop: %u", loop);
	return 0;
}

/**
 * @todo: comment
 */
static int intel_phy_test_six(struct intel_phy *iphy,
	unsigned loop, unsigned t1, unsigned t2)
{
	const unsigned long event[] = {
		USB_EVENT_NONE,
		USB_EVENT_ID,
		INTEL_USB_DRV_VBUS_ERR,
		USB_EVENT_NONE,
		USB_EVENT_VBUS,
		USB_EVENT_NONE,
		USB_EVENT_ID,
	};
	unsigned i = 0, j = 0;


	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	for (i = 0; i < loop; i++) {
		intel_phy_info("t1 = %u, initial OTG state: %s", t1,
				intel_phy_get_state_string(iphy));

		for (j = 0; j < ARRAY_SIZE(event); j++) {
			intel_phy_notify(iphy, event[j], &iphy->input_props);
			intel_phy_ms_pause(t1);
			intel_phy_info("pause %u ms, current OTG state: %s",
					t1, intel_phy_get_state_string(iphy));
		}
	}
	intel_phy_info("test loop: %u", loop);
	return 0;
}

/**
 * @todo: comment
 */
static int intel_phy_test_five(struct intel_phy *iphy,
	unsigned loop, unsigned t1, unsigned t2)
{
	unsigned i = 0;


	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	for (i = 0; i < loop; i++) {
		intel_phy_notify(iphy, USB_EVENT_NONE, &iphy->input_props);
		intel_phy_ms_pause(t1);
		intel_phy_notify(iphy, USB_EVENT_ID, &iphy->input_props);
		intel_phy_ms_pause(t2);
	}
	intel_phy_info("test loop: %u", loop);
	return 0;
}

/**
 * @todo: comment
 */
static int intel_phy_test_four(struct intel_phy *iphy,
	unsigned loop, unsigned t1, unsigned t2)
{
	unsigned i = 0;
	unsigned long event = 0;

	if (IS_ERR_OR_NULL(iphy)) {
			intel_phy_err("invalid parameter");
			return intel_phy_kernel_trap();
	}

	for (i = 0; i < loop; i++) {
		intel_phy_info("force cable %s", t1 ? "attach" : "detach");
		if (t1)
			event = USB_EVENT_VBUS;
		else
			event = USB_EVENT_NONE;
		intel_phy_notify(iphy, event, &iphy->input_props);
	}
	intel_phy_info("test loop: %u", loop);
	return 0;
}

/**
 * @todo: comment
 */
static int intel_phy_test_three(struct intel_phy *iphy,
	unsigned loop, unsigned t1, unsigned t2)
{
	const unsigned long event[] = {
		USB_EVENT_NONE,
		USB_EVENT_VBUS,
		USB_EVENT_NONE,
		USB_EVENT_ID,
		USB_EVENT_NONE,
		USB_EVENT_VBUS,
	};
	unsigned i = 0, j = 0;


	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	for (i = 0; i < loop; i++) {
		intel_phy_info("t1 = %u, initial OTG state: %s", t1,
				intel_phy_get_state_string(iphy));

		for (j = 0; j < ARRAY_SIZE(event); j++) {
			intel_phy_notify(iphy, event[j], &iphy->input_props);
			intel_phy_info("pause %u ms, current OTG state: %s",
					t1, intel_phy_get_state_string(iphy));
			intel_phy_ms_pause(t1);
		}
	}
	intel_phy_info("test loop: %u", loop);
	return 0;
}

/**
 * @todo: comment
 */
static int intel_phy_test_two(struct intel_phy *iphy,
	unsigned loop, unsigned t1, unsigned t2)
{
	struct usb_gadget *gadget = NULL;
	unsigned i = 0;


	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	gadget = intel_phy_get_gadget(iphy);
	if (IS_ERR_OR_NULL(gadget)) {
		intel_phy_warn("gadget not registered");
		return 0;
	}

	for (i = 0; i < loop; i++) {
		usb_gadget_disconnect(gadget);
		intel_phy_ms_pause(t1);
		usb_gadget_connect(gadget);
		intel_phy_ms_pause(t2);
	}
	intel_phy_info("test loop: %u", loop);
	return 0;
}

/**
 * @todo: comment
 */
static int intel_phy_test_one(struct intel_phy *iphy,
	unsigned loop, unsigned t1, unsigned t2)
{
	unsigned i = 0;


	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	for (i = 0; i < loop; i++) {
		intel_phy_notify(iphy, USB_EVENT_NONE, &iphy->input_props);
		intel_phy_ms_pause(t1);
		intel_phy_notify(iphy, USB_EVENT_VBUS, &iphy->input_props);
		intel_phy_ms_pause(t2);
	}
	intel_phy_info("test loop: %u", loop);
	return 0;
}

/**
 * @todo: comment
 */
static int intel_phy_test_zero(struct intel_phy *iphy,
	unsigned dummy1, unsigned dummy2, unsigned dummy3)
{
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	dummy1 = NUM_OTG_FSM_TIMERS;

	/* disable crash during this test */
	intel_phy_kernel_trap_enable(false);

	/* internal */
	intel_phy_get_phy(NULL);
	intel_phy_get_otg(NULL);
	intel_phy_get_fsm(NULL);
	intel_phy_get_input(NULL);
	intel_phy_chrg_to_str(0);
	intel_phy_otg_timer_to_str(0);
	intel_phy_otg_set_tmout(0);
	intel_phy_otg_get_timer_data(NULL, 0);
	intel_phy_otg_fsm_pre_process_chrg(NULL);
	intel_phy_otg_fsm_pre_process(NULL);
	intel_phy_otg_fsm_post_process(NULL);
	intel_phy_event_wq(NULL);
	intel_phy_data_init(NULL, NULL, NULL);

	/* interface */
	intel_phy_event_nb(NULL, 0, NULL);
	intel_phy_event_nb(intel_phy_get_phy(iphy)->notifier.head, 0, NULL);
	intel_phy_otg_set_host(NULL, NULL);
	intel_phy_otg_set_peripheral(NULL, NULL);
	intel_phy_otg_fsm_drv_vbus(NULL, 0);
	intel_phy_otg_fsm_add_timer(NULL, 0);
	intel_phy_otg_fsm_add_timer(intel_phy_get_fsm(iphy), dummy1);
	intel_phy_otg_fsm_add_timer(intel_phy_get_fsm(iphy), dummy1+1);
	intel_phy_otg_fsm_del_timer(NULL, 0);
	intel_phy_otg_fsm_del_timer(intel_phy_get_fsm(iphy), dummy1);
	intel_phy_otg_fsm_del_timer(intel_phy_get_fsm(iphy), dummy1+1);
	intel_phy_set_vbus(NULL, 0);
	intel_phy_set_power(NULL, 0);
	intel_phy_notify_connect(NULL, 0);
	intel_phy_notify_disconnect(NULL, 0);
	intel_phy_init(NULL, NULL, NULL, 0);
	intel_phy_exit(NULL, NULL);
	intel_phy_notify(NULL, 0, NULL);
	intel_phy_notify(iphy, 0, NULL);

	/* debugfs and test */
	intel_phy_ms_pause(0);
	intel_phy_test_seven(NULL, 0, 0, 0);
	intel_phy_test_six(NULL, 0, 0, 0);
	intel_phy_test_five(NULL, 0, 0, 0);
	intel_phy_test_four(NULL, 0, 0, 0);
	intel_phy_test_three(NULL, 0, 0, 0);
	intel_phy_test_two(NULL, 0, 0, 0);
	intel_phy_test_one(NULL, 0, 0, 0);
	intel_phy_test_zero(NULL, 0, 0, 0);
	intel_phy_debugfs_open(NULL, NULL);
	intel_phy_debugfs_dbg_read(NULL, NULL, 0, NULL);
	intel_phy_debugfs_dbg_write(NULL, NULL, 0, NULL);
	intel_phy_debugfs_init(NULL);
	intel_phy_debugfs_exit(NULL);

	/* re-enable crash after this test */
	intel_phy_kernel_trap_enable(true);

	return 0;
}

/**
 * @todo: comment
 */
static int intel_phy_debugfs_open(struct inode *inode, struct file *file)
{
	if (IS_ERR_OR_NULL(inode) || IS_ERR_OR_NULL(file)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (IS_ERR_OR_NULL(inode->i_private)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	file->private_data = inode->i_private;
	return 0;
}

/**
 * @todo: comment
 */
static ssize_t intel_phy_debugfs_dbg_read(
	struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	struct intel_phy *iphy = NULL;
	struct usb_phy *phy = NULL;
	struct otg_fsm *fsm = NULL;


	if (IS_ERR_OR_NULL(file) || IS_ERR_OR_NULL(ubuf)
	|| (IS_ERR_OR_NULL(ppos))) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (IS_ERR_OR_NULL(file->private_data)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	iphy = file->private_data;

	phy = intel_phy_get_phy(iphy);
	fsm = intel_phy_get_fsm(iphy);

	/* todo: log in provided buffer instead */

	intel_phy_info("%s, %d mA",
			intel_phy_chrg_to_str(iphy->cable_props.chrg_type),
			iphy->cable_props.ma);
	intel_phy_info("PHY type: %s", usb_phy_type_string(phy->type));
	intel_phy_info("OTG state: %s\n"
			"id            : %d\n"
			"adp_change    : %d\n"
			"power_up      : %d\n"
			"test_device   : %d\n"
			"a_bus_drop    : %d\n"
			"a_bus_req     : %d\n"
			"a_srp_det     : %d\n"
			"a_vbus_vld    : %d\n"
			"b_conn        : %d\n"
			"a_bus_resume  : %d\n"
			"a_bus_suspend : %d\n"
			"a_conn        : %d\n"
			"b_bus_req     : %d\n"
			"b_se0_srp     : %d\n"
			"b_ssend_srp   : %d\n"
			"b_sess_vld    : %d\n",
			usb_otg_state_string(phy->state),
			fsm->id,
			fsm->adp_change,
			fsm->power_up,
			fsm->test_device,
			fsm->a_bus_drop,
			fsm->a_bus_req,
			fsm->a_srp_det,
			fsm->a_vbus_vld,
			fsm->b_conn,
			fsm->a_bus_resume,
			fsm->a_bus_suspend,
			fsm->a_conn,
			fsm->b_bus_req,
			fsm->b_se0_srp,
			fsm->b_ssend_srp,
			fsm->b_sess_vld);
	return 0;
}

/**
 * @todo: comment
 */
static ssize_t intel_phy_debugfs_dbg_write(
	struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct intel_phy *iphy = NULL;
	char buf[32] = {0};
	unsigned test, loop, t1, t2, end = 0;


	if (IS_ERR_OR_NULL(file) || IS_ERR_OR_NULL(ubuf)
	|| (IS_ERR_OR_NULL(ppos))) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (IS_ERR_OR_NULL(file->private_data)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	iphy = file->private_data;

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		intel_phy_err("copy from user");
		return intel_phy_kernel_trap();
	}

	if (sscanf(buf, "%u %u %u %u %u", &test, &loop, &t1, &t2, &end) != 4) {
		intel_phy_err("wrong parameter number!");
		goto info;
	}
	/* fix KW warnings */
	if (test > USHRT_MAX)
		test = USHRT_MAX;
	if (loop > USHRT_MAX)
		loop = USHRT_MAX;
	if (t1 > USHRT_MAX)
		t1 = USHRT_MAX;
	if (t2 > USHRT_MAX)
		t2 = USHRT_MAX;

	switch (test) {
	case 0:
		intel_phy_test_zero(iphy, loop, t1, t2);
		break;
	case 1:
		intel_phy_test_one(iphy, loop, t1, t2);
		break;
	case 2:
		intel_phy_test_two(iphy, loop, t1, t2);
		break;
	case 3:
		intel_phy_test_three(iphy, loop, t1, t2);
		break;
	case 4:
		intel_phy_test_four(iphy, loop, t1, t2);
		break;
	case 5:
		intel_phy_test_five(iphy, loop, t1, t2);
		break;
	case 6:
		intel_phy_test_six(iphy, loop, t1, t2);
		break;
	case 7:
		intel_phy_test_seven(iphy, loop, t1, t2);
		break;
	default:
		intel_phy_err("test not defined!");
		break;
	}

	return count;
info:
	intel_phy_info("<test> <loop> <t1> <t2>");
	intel_phy_info("test 0: test invalid parameter, boost coverage");
	intel_phy_info("test 1: dettach    <wait t1> attach  <wait t2>");
	intel_phy_info("test 2: disconnect <wait t1> connect <wait t2>");
	intel_phy_info("test 3: generate predefined phy event sequence");
	intel_phy_info("test 4: force vbus        <t1 = attach/detach>");
	return count;
}

/**
 * @todo: comment
 */
static const struct file_operations intel_phy_debugfs_fops = {
	.open = intel_phy_debugfs_open,
	.read = intel_phy_debugfs_dbg_read,
	.write = intel_phy_debugfs_dbg_write,
};

/**
 * @todo: comment
 */
static int intel_phy_debugfs_init(struct intel_phy *iphy)
{
	struct dentry *root = NULL;
	struct dentry *file = NULL;


	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	root = debugfs_create_dir("intel_phy", NULL);
	if (IS_ERR_OR_NULL(root)) {
		intel_phy_err("debugfs create dir");
		return intel_phy_kernel_trap();
	}

	file = debugfs_create_file("dbg", S_IRUGO | S_IWUSR,
		root, iphy, &intel_phy_debugfs_fops);
	if (IS_ERR_OR_NULL(file)) {
		intel_phy_err("debugfs create file");
		debugfs_remove(root);
		return intel_phy_kernel_trap();
	}

	iphy->debugfs_root = root;
	return 0;
}

/**
 * @todo: comment
 */
static int intel_phy_debugfs_exit(struct intel_phy *iphy)
{
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	if (IS_ERR_OR_NULL(iphy->debugfs_root)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	debugfs_remove_recursive(iphy->debugfs_root);
	iphy->debugfs_root = NULL;
	return 0;
}
