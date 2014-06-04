/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/power_supply.h>
#include <linux/reset.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/wakelock.h>
#include "phy-intel.h"

#define DECLARE_USB_ACCESSOR(TYPE)\
int usb_parse_##TYPE(struct intel_usbphy *iphy, struct device *dev)\
{\
	int ret = 0;\
	unsigned int len = 0;\
	unsigned int len_value = 0;\
	unsigned int idx = 0;\
	unsigned int idx_val = 0;\
	unsigned int j = 0;\
	u32 *array;\
	u32 *array_value;\
	struct device_node *np = dev->of_node;\
	struct usb_reg *reg;\
	if (of_find_property(np, "intel,"#TYPE, &len) && \
		of_find_property(np, "intel,"#TYPE "-value", &len_value)) {\
		iphy->TYPE = devm_kzalloc(dev,\
				sizeof(struct usb_reg), GFP_KERNEL);\
		if (!iphy->TYPE) {\
			dev_err(dev, "allocation of iphy->TYPE failed\n");\
		} \
		INIT_LIST_HEAD(&iphy->TYPE->list);\
		len /= sizeof(u32);\
		array = devm_kzalloc(dev, len * sizeof(u32), GFP_KERNEL);\
		if (!array) {\
			dev_err(dev, "allocation of array failed\n");\
			return ret;\
		} \
		ret = of_property_read_u32_array(np, "intel," #TYPE,\
							array, len);\
		if (ret != 0) {\
			dev_err(dev, "read " #TYPE " property failed: %d\n",\
									ret);\
			return ret;\
		} \
		len_value /= sizeof(u32);\
		array_value = devm_kzalloc(dev,\
				len_value * sizeof(u32), GFP_KERNEL);\
		if (!array_value) {\
			dev_err(dev, "allocation of array_value failed\n");\
			return ret;\
		} \
		ret = of_property_read_u32_array(np, "intel," #TYPE "-value",\
						array_value, len_value);\
		if (ret != 0) {\
			dev_err(dev,\
				"read " #TYPE "-value property failed: %d\n",\
									ret);\
			return ret;\
		} \
		dev_dbg(dev,\
			"Parsing " #TYPE " and " #TYPE "-value properties\n");\
		for (idx = 0, idx_val = 0; idx+2 < len\
				&& idx_val+1 < len_value; idx += 3,\
							idx_val += 2) {\
			reg = devm_kzalloc(dev,\
					sizeof(struct usb_reg), GFP_KERNEL);\
			if (!reg) {\
				dev_err(dev, "allocation of reg failed\n");\
				return ret;\
			} \
			reg->base = array[idx];\
			reg->offset = array[idx+1]; \
			for (j = 0; j < array[idx+2]; j++) \
				reg->mask |= 1 << j; \
			reg->mask <<= array[idx+1]; \
			reg->disable = array_value[idx_val];\
			reg->enable = array_value[idx_val+1];\
			list_add_tail(&reg->list, &iphy->TYPE->list);\
			dev_dbg(dev, "base=%#x, offset=%#x", \
					(u32)reg->base,\
							(u32)reg->offset);\
			dev_dbg(dev, "mask=%#x, disable=%#x, enable=%#x\n",\
					(u32)reg->mask, (u32)reg->disable,\
							(u32)reg->enable);\
		} \
	} \
	return ret;\
};

#define PARSE_USB_ACCESSOR(TYPE, ENABLE) { \
	if (of_find_property(np, "intel,"#TYPE, &len)) { \
		ret = usb_parse_##TYPE(iphy, dev); \
		if (ret != 0) { \
			return -EINVAL; \
		} \
		usb_enable_##TYPE(iphy, ENABLE); \
	} \
}

#define PARSE_USB_STATUS(TYPE) { \
	if (of_find_property(np, "intel,"#TYPE, &len)) { \
		ret = usb_parse_##TYPE(iphy, dev); \
		if (ret != 0)\
			return -EINVAL; \
	} \
}

#define DECLARE_USB_STATUS(TYPE)\
int usb_##TYPE##_status(struct intel_usbphy *iphy) \
{\
	struct usb_reg *reg;\
	unsigned tmp = 0;\
	if (iphy->TYPE) {\
		list_for_each_entry(reg, &iphy->TYPE->list, list) {\
			tmp = ((ioread32(iphy->scuregs + reg->base))\
					& (reg->mask)) >> reg->offset;\
			if (tmp == reg->enable)\
				return 1;\
			else\
				return 0;\
		} \
	} \
	return 0;\
} \
DECLARE_USB_ACCESSOR(TYPE)

#define DECLARE_USB_ENABLE(TYPE)\
int usb_enable_##TYPE(struct intel_usbphy *iphy, bool enable)\
{\
	struct usb_reg *regs_##TYPE;\
	unsigned tmp = 0;\
	if (iphy->TYPE) {\
		list_for_each_entry(regs_##TYPE, &iphy->TYPE->list, list) {\
			if (enable) {\
				tmp = (ioread32(iphy->scuregs \
							+ regs_##TYPE->base))\
							& ~(regs_##TYPE->mask);\
				tmp |= (regs_##TYPE->enable\
						<< regs_##TYPE->offset);\
				iowrite32(tmp, iphy->scuregs\
							+ regs_##TYPE->base);\
			} else { \
				tmp = (ioread32(iphy->scuregs\
							+ regs_##TYPE->base))\
							& ~(regs_##TYPE->mask);\
				tmp |= (regs_##TYPE->disable\
						<< regs_##TYPE->offset);\
				iowrite32(tmp, iphy->scuregs\
							+ regs_##TYPE->base);\
			} \
		} \
	} \
	return 0;\
};\
DECLARE_USB_ACCESSOR(TYPE)

DECLARE_USB_ENABLE(phy_sus)
DECLARE_USB_ENABLE(trim)
DECLARE_USB_ENABLE(pll_en)
DECLARE_USB_ENABLE(avalid)
DECLARE_USB_ENABLE(bvalid)
DECLARE_USB_ENABLE(vbusvalid)
DECLARE_USB_ENABLE(sessend)
DECLARE_USB_ENABLE(commononn)
DECLARE_USB_ENABLE(srp_clear)
DECLARE_USB_ENABLE(vdatsrcenb)
DECLARE_USB_ENABLE(vdatdetenb)
DECLARE_USB_ENABLE(dcdenb)
DECLARE_USB_ENABLE(chrgsel)
DECLARE_USB_STATUS(drvvbus)
DECLARE_USB_STATUS(ridgnd)
DECLARE_USB_STATUS(fsvplus)
DECLARE_USB_STATUS(chrgdet)
DECLARE_USB_STATUS(hsrs)

int usb_enable_reset(struct intel_usbphy *iphy, bool reset, char *name)
{
	struct usb_reset *usbreset;
	if (iphy->reset) {
		list_for_each_entry(usbreset, &iphy->reset->list, list) {
			if (!name || !strcmp(usbreset->res_name, name)) {
				if (reset && usbreset->reset)
					reset_control_assert(usbreset->reset);
				else if (!reset && usbreset->reset)
					reset_control_deassert(usbreset->reset);
			}
		}
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int intel_otg_suspend(struct intel_usbphy *iphy)
{
	bool device_bus_suspend;
	struct usb_phy *phy = &iphy->phy;
	int ret = 0;
	void __iomem *pcgctlregs = phy->io_priv;

	if (atomic_read(&iphy->in_lpm))
		return 0;

	device_bus_suspend = phy->otg->gadget && test_bit(ID, &iphy->inputs) &&
		test_bit(A_BUS_SUSPEND, &iphy->inputs);
	if (device_bus_suspend) {
		/* power off USB core, restore isolation */
		ret = device_state_pm_set_state(iphy->dev,
				iphy->pm_states[USB_PMS_SUSPEND]);
		if (ret	< 0) {
			dev_err(iphy->dev,
					"pm set state disable failed: %d", ret);
			return -EINVAL;
		}
		usb_enable_pll_en(iphy, false);
		usb_enable_reset(iphy, true, "bus");
		if (device_may_wakeup(iphy->dev))
			enable_irq_wake(iphy->resume_irq);

		atomic_set(&iphy->in_lpm, 1);
		atomic_set(&iphy->bus_suspended, 1);
		dev_dbg(phy->dev, "USB bus in low power mode\n");
		wake_unlock(&iphy->wlock);
		return 0;
	}

	/* Disable DCD circuit*/
	usb_enable_dcdenb(iphy, 0);
	/* Turn off voltage source at DP/PM in case it was on */
	usb_enable_vdatsrcenb(iphy, 0);
	/* Turn off comparator and current source */
	usb_enable_vdatdetenb(iphy, 0);
	/* Turn off battery charging source select */
	usb_enable_chrgsel(iphy, 0);

	/* USB core gate power down sequence */
	writel(readl(pcgctlregs) & ~PCGCTL_PWRCLMP, pcgctlregs);
	writel(readl(pcgctlregs) & ~PCGCTL_RSTPDWNMODULE, pcgctlregs);

	usb_enable_phy_sus(iphy, false);

	writel(readl(pcgctlregs) & ~PCGCTL_STOPPCLK, pcgctlregs);

	usb_enable_pll_en(iphy, false);

	/* power off USB core, restore isolation */
	ret = device_state_pm_set_state(iphy->dev,
			iphy->pm_states[USB_PMS_DISABLE]);
	if (ret	< 0) {
		dev_err(iphy->dev, "pm set state disable failed: %d", ret);
		return -EINVAL;
	}

	atomic_set(&iphy->in_lpm, 1);
	wake_unlock(&iphy->wlock);
	dev_dbg(phy->dev, "USB in low power mode\n");
	return 0;
}

static int intel_otg_resume(struct intel_usbphy *iphy)
{
	int ret = 0;
	int i = 0;
	struct usb_phy *phy = &iphy->phy;
	void __iomem *pcgctlregs = phy->io_priv;

	if (!atomic_read(&iphy->in_lpm))
		return 0;

	wake_lock(&iphy->wlock);

	if (atomic_read(&iphy->bus_suspended)) {
		ret = device_state_pm_set_state(iphy->dev,
				iphy->pm_states[USB_PMS_ENABLE_ISO]);
		if (ret)
			dev_err(iphy->dev, "can't resume power\n");
		clear_bit(A_BUS_SUSPEND, &iphy->inputs);
		atomic_set(&iphy->in_lpm, 0);
		atomic_set(&iphy->bus_suspended, 0);
		usb_enable_pll_en(iphy, true);
		usb_enable_reset(iphy, false, "bus");
		if (device_may_wakeup(iphy->dev))
			disable_irq_wake(iphy->resume_irq);
		return 0;
	}


	/* power up USB core and PHY */
	ret = device_state_pm_set_state(iphy->dev,
			iphy->pm_states[USB_PMS_ENABLE]);
	if (ret	< 0) {
		dev_err(iphy->dev, "pm set state enable failed: %d\n", ret);
		return -EINVAL;
	}

	/* wait for LDO stabilization */
	mdelay(1);

	/* USB core gate power up sequence */
	writel(readl(pcgctlregs) & ~PCGCTL_STOPPCLK, pcgctlregs);
	writel(readl(pcgctlregs) & ~PCGCTL_PWRCLMP, pcgctlregs);
	udelay(50);
	writel(readl(pcgctlregs) & ~PCGCTL_RSTPDWNMODULE, pcgctlregs);

	ret = device_state_pm_set_state(iphy->dev,
			iphy->pm_states[USB_PMS_ENABLE_ISO]);
	if (ret	< 0) {
		dev_err(iphy->dev, "pm set state enable iso failed: %d\n", ret);
		return -EINVAL;
	}


	/* reset USB core and PHY */
	usb_enable_reset(iphy, true, "usb");
	usb_enable_reset(iphy, true, "bus");

	/* remove PHY from suspend state */
	usb_enable_phy_sus(iphy, true);
	usb_enable_pll_en(iphy, true);

	/* wait for HW stability */
	udelay(200);

	/* remove reset for USB core and PHY */
	usb_enable_reset(iphy, false, "usb");
	usb_enable_reset(iphy, false, "bus");

	/* wait for reset to complete */
	while (usb_hsrs_status(iphy) && i++ < 500)
		udelay(1);

	if (i == 500) {
		dev_err(iphy->dev, "usb didn't exit from reset\n");
		return -EINVAL;
	}

	dev_dbg(iphy->dev, "got usb reset in %d us\n", i);

	atomic_set(&iphy->in_lpm, 0);
	dev_dbg(phy->dev, "USB exited from low power mode\n");
	return 0;
}
#endif

static int intel_usb2phy_set_suspend(struct usb_phy *phy, int suspend)
{
	struct intel_usbphy *iphy = container_of(phy, struct intel_usbphy, phy);
	/* TODO: add suspend in case of A_HOST.. */
	if (suspend) {
		switch (phy->state) {
		case OTG_STATE_B_PERIPHERAL:
			dev_dbg(phy->dev, "peripheral bus suspend\n");
			set_bit(A_BUS_SUSPEND, &iphy->inputs);
			if (!atomic_read(&iphy->in_lpm))
				schedule_work(&iphy->sm_work);
			break;

		default:
			break;
		}
	} else {
		switch (phy->state) {
		case OTG_STATE_B_PERIPHERAL:
			dev_dbg(phy->dev, "peripheral bus resume\n");
			clear_bit(A_BUS_SUSPEND, &iphy->inputs);
			 if (atomic_read(&iphy->in_lpm))
				schedule_work(&iphy->sm_work);
			break;
		default:
			break;
		}
	}
	return 0;
}

static irqreturn_t intel_usb2phy_resume(int irq, void *dev)
{
	struct intel_usbphy *iphy = (struct intel_usbphy *) dev;
	intel_usb2phy_set_suspend(&iphy->phy, 0);
	return IRQ_HANDLED;
}

static void intel_otg_notify_charger(struct intel_usbphy *iphy, unsigned mA)
{
	if (iphy->cur_power == mA)
		return;

	dev_dbg(iphy->phy.dev, "Avail curr from USB = %u\n", mA);
	iphy->cable_props.ma = mA;
	iphy->cable_props.chrg_type = iphy->chg_type;
	if (!test_bit(B_SESS_VLD, &iphy->inputs))
		iphy->cable_props.chrg_evt =
			POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
	else
		iphy->cable_props.chrg_evt =
			POWER_SUPPLY_CHARGER_EVENT_CONNECT;

	atomic_notifier_call_chain(&iphy->phy.notifier,
			USB_EVENT_CHARGER, &iphy->cable_props);
	iphy->cur_power = mA;
}

static int intel_usb2phy_set_power(struct usb_phy *phy, unsigned mA)
{
	struct intel_usbphy *iphy = container_of(phy, struct intel_usbphy, phy);
	if (iphy->chg_type == POWER_SUPPLY_CHARGER_TYPE_USB_SDP)
		intel_otg_notify_charger(iphy, mA);
	return 0;
}

static void intel_otg_start_peripheral(struct usb_otg *otg, int on)
{
	struct intel_usbphy *iphy = container_of(otg->phy,
			struct intel_usbphy, phy);
	if (!otg->gadget)
		return;

	if (on) {
		dev_dbg(otg->phy->dev, "gadget on\n");
		usb_gadget_vbus_connect(otg->gadget);
	} else {
		dev_dbg(otg->phy->dev, "gadget off\n");
		usb_gadget_vbus_disconnect(otg->gadget);
	}

	/*
	 * Assert vbus valid and B-Valid
	 * to indicate a B session
	 */
	usb_enable_avalid(iphy, on);
	usb_enable_bvalid(iphy, on);
	usb_enable_vbusvalid(iphy, on);
	usb_enable_sessend(iphy, !on);
}

static int intel_usb2phy_set_peripheral(struct usb_otg *otg,
		struct usb_gadget *gadget)
{
	struct intel_usbphy *iphy = container_of(otg->phy,
			struct intel_usbphy, phy);
	if (!otg)
		return -ENODEV;

	if (!gadget) {
		if (otg->phy->state == OTG_STATE_B_PERIPHERAL) {
			pm_runtime_get_sync(otg->phy->dev);
			intel_otg_start_peripheral(otg, 0);
			otg->gadget = NULL;
			otg->phy->state = OTG_STATE_UNDEFINED;
			schedule_work(&iphy->sm_work);
		} else {
			otg->gadget = NULL;
		}
		return 0;
	}

	otg->gadget = gadget;

	pm_runtime_get_sync(otg->phy->dev);
	schedule_work(&iphy->sm_work);

	return 0;
}

static void intel_otg_start_host(struct usb_otg *otg, int on)
{
	struct intel_usbphy *iphy =
		container_of(otg->phy, struct intel_usbphy, phy);
	struct usb_hcd *hcd;

	if (!otg->host)
		return;

	hcd = bus_to_hcd(otg->host);

	if (on) {
		dev_dbg(iphy->dev, "host on\n");

#ifdef CONFIG_USB
		usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
#endif
	} else {
		dev_dbg(iphy->dev, "host off\n");

#ifdef CONFIG_USB
		usb_remove_hcd(hcd);
#endif
	}
}

static int intel_usb2phy_set_host(struct usb_otg *otg,
		struct usb_bus *host)
{
	if (!otg)
		return -ENODEV;

	if (!host) {
		otg->host = NULL;
		return -ENODEV;
	}

	otg->host = host;
	return 0;
}

static int intel_usb2phy_notifier(struct notifier_block *nb,
		unsigned long event, void *priv)
{
	static bool init;
	struct intel_usbphy *iphy = container_of(nb,
			struct intel_usbphy, usb_nb);
	int *vbus;

	/*TODO: What if LPM ? */
	switch (event) {
	case USB_EVENT_VBUS:
		vbus = (int *) priv;
		if (*vbus) {
			dev_dbg(iphy->dev, "BSV set\n");
			set_bit(B_SESS_VLD, &iphy->inputs);
		} else {
			dev_dbg(iphy->dev, "BSV clear\n");
			clear_bit(B_SESS_VLD, &iphy->inputs);
			clear_bit(A_BUS_SUSPEND, &iphy->inputs);
		}

		if (!init) {
			init = true;
			complete(&iphy->bms_vbus_init);
			dev_dbg(iphy->dev, "BMS VBUS init complete\n");
			return NOTIFY_OK;
		}
		if (atomic_read(&iphy->pm_suspended))
			iphy->sm_work_pending = true;
		else
			schedule_work(&iphy->sm_work);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static const char *chg_to_string(enum power_supply_charger_cable_type chg_type)
{
	switch (chg_type) {
	case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
		return "USB_SDP_CHARGER";
	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		return "USB_DCP_CHARGER";
	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		return "USB_CDP_CHARGER";
	default:
		return "INVALID_CHARGER";
	}
}

static void intel_chg_enable_secondary_det(struct intel_usbphy *iphy)
{
	/* USB Secondary detection */
	/* Charging Source Select DP */
	usb_enable_chrgsel(iphy, true);
	usb_enable_vdatsrcenb(iphy, true);
	usb_enable_vdatdetenb(iphy, true);

}

static void intel_chg_enable_primary_det(struct intel_usbphy *iphy, bool on)
{
	/* Enable primary detection */
	usb_enable_vdatsrcenb(iphy, on);
	usb_enable_vdatdetenb(iphy, on);
}

static void intel_chg_enable_dcd(struct intel_usbphy *iphy, bool on)
{
	if (on) {
		/* Turn off  voltage source at DP/PM in case it was on */
		usb_enable_vdatsrcenb(iphy, false);
		/* Turn off comparator and current source */
		usb_enable_vdatdetenb(iphy, false);
		/* Turn on DCD circuitry */
	}
	usb_enable_dcdenb(iphy, on);
}

#define CHG_DCD_TIMEOUT		(500 * HZ/1000) /* 500 msec */
#define CHG_DCD_POLL_TIME	(100 * HZ/1000) /* 100 msec */
#define CHG_PRIMARY_DET_TIME	(40 * HZ/1000) /* TVDPSRC_ON */
#define CHG_SECONDARY_DET_TIME	(40 * HZ/1000) /* TVDMSRC_ON */

static void intel_chg_detect_work(struct work_struct *w)
{
	struct intel_usbphy *iphy
			= container_of(w, struct intel_usbphy, chg_work.work);
	struct usb_phy *phy = &iphy->phy;
	bool is_dcd = false, tmout = 0, vout;
	unsigned long delay;

	dev_dbg(phy->dev, "chg detection work\n");

	switch (iphy->chg_state) {
	case USB_CHG_STATE_UNDEFINED:
		/* Start DCD processing stage 1 */
		intel_chg_enable_dcd(iphy, true);

		iphy->chg_state = USB_CHG_STATE_WAIT_FOR_DCD;
		iphy->dcd_time = 0;
		delay = CHG_DCD_POLL_TIME;
		break;
	case USB_CHG_STATE_WAIT_FOR_DCD:
		/* get data contact detection status */
		/*  0 -> DCD detected */
		is_dcd = !usb_fsvplus_status(iphy);
		iphy->dcd_time += CHG_DCD_POLL_TIME;
		tmout = iphy->dcd_time >= CHG_DCD_TIMEOUT;
		/* stage 2 */
		if (is_dcd || tmout) {
			/* stage 4 */
			/* Turn off DCD circuitry */
			intel_chg_enable_dcd(iphy, false);
			intel_chg_enable_primary_det(iphy, true);
			delay = CHG_PRIMARY_DET_TIME;
			iphy->chg_state = USB_CHG_STATE_DCD_DONE;
		} else {
			/* stage 3 */
			delay = CHG_DCD_POLL_TIME;
		}
		break;
	case USB_CHG_STATE_DCD_DONE:
		vout = usb_chrgdet_status(iphy);
		intel_chg_enable_primary_det(iphy, false);
		if (usb_fsvplus_status(iphy)) {
			/* Special charger found */
			dev_dbg(iphy->dev, "charger detection: Special charger found\n");
			iphy->chg_type =
				POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
			iphy->chg_state =
				USB_CHG_STATE_DETECTED;
			delay = 0;
		} else if (!vout) {
			if (tmout)
				/* floating charger found */
				iphy->chg_type =
					POWER_SUPPLY_CHARGER_TYPE_NONE;
			else
				iphy->chg_type =
					POWER_SUPPLY_CHARGER_TYPE_USB_SDP;

			iphy->chg_state = USB_CHG_STATE_DETECTED;
			delay = 0;

		} else {
			intel_chg_enable_secondary_det(iphy);
			delay = CHG_SECONDARY_DET_TIME;
			iphy->chg_state = USB_CHG_STATE_PRIMARY_DONE;
		}
		break;
	case USB_CHG_STATE_PRIMARY_DONE:
		vout = usb_chrgdet_status(iphy);
		/* Turn off  voltage source */
		usb_enable_vdatsrcenb(iphy, false);
		/* Turn off comparator and current source */
		usb_enable_vdatdetenb(iphy, false);
		if (!vout)
			iphy->chg_type = POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
		else {
			/* pull D+ line to VDP_SRC after DCP detection */
			iphy->chg_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
			usb_enable_chrgsel(iphy, false);
			usb_enable_vdatsrcenb(iphy, true);
		}
		iphy->chg_state = USB_CHG_STATE_SECONDARY_DONE;
		/* fall through */
	case USB_CHG_STATE_SECONDARY_DONE:
		iphy->chg_state = USB_CHG_STATE_DETECTED;
	case USB_CHG_STATE_DETECTED:
		dev_dbg(phy->dev, "chg_type = %s\n",
			chg_to_string(iphy->chg_type));
		schedule_work(&iphy->sm_work);
		return;
	default:
		return;
	}

	schedule_delayed_work(&iphy->chg_work, delay);
}

static void intel_otg_init_sm(struct intel_usbphy *iphy)
{
	/*TODO: host mode*/
	set_bit(ID, &iphy->inputs);
	/* Wait for charger IC input */
	wait_for_completion(&iphy->bms_vbus_init);
}

static void intel_otg_sm_work(struct work_struct *w)
{
	struct intel_usbphy *iphy = container_of(w, struct intel_usbphy,
			sm_work);
	struct usb_otg *otg = iphy->phy.otg;
	bool work = 0;

	pm_runtime_resume(otg->phy->dev);
	dev_dbg(iphy->dev, "%s work\n", usb_otg_state_string(otg->phy->state));
	switch (otg->phy->state) {
	case OTG_STATE_UNDEFINED:
		intel_otg_init_sm(iphy);
		otg->phy->state = OTG_STATE_B_IDLE;
		if (!test_bit(B_SESS_VLD, &iphy->inputs) &&
				test_bit(ID, &iphy->inputs)) {
			/*
			 * FIXME: Charger IC driver is waiting for
			 * POWER_SUPPLY_CHARGER_EVENT_DISCONNECT to
			 * stop charging if cable is disconnected.
			 * HACK: intel_otg_notify_charger check that cur_power
			 * is different from requested value.
			 * This is why cur_power is set to 1 prior calling it.
			 */
			iphy->cur_power =  1;
			intel_otg_notify_charger(iphy, 0);
			pm_runtime_put_noidle(otg->phy->dev);
			pm_runtime_suspend(otg->phy->dev);
			break;
		}
		/* FALL THROUGH */
	case OTG_STATE_B_IDLE:
		if (!test_bit(ID, &iphy->inputs) && otg->host) {
			otg->phy->state = OTG_STATE_A_IDLE;
			work = 1;
		} else if (test_bit(B_SESS_VLD, &iphy->inputs)) {
			switch (iphy->chg_state) {
			case USB_CHG_STATE_UNDEFINED:
				intel_chg_detect_work(&iphy->chg_work.work);
				break;
			case USB_CHG_STATE_DETECTED:
				switch (iphy->chg_type) {
				case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
					intel_otg_notify_charger(iphy,
							IDEV_CHG_MAX);
					pm_runtime_put_noidle(otg->phy->dev);
					pm_runtime_suspend(otg->phy->dev);
					break;
				case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
					intel_otg_notify_charger(iphy, IUNIT);
					intel_otg_start_peripheral(otg, 1);
					otg->phy->state
						= OTG_STATE_B_PERIPHERAL;
					break;
				case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
					intel_otg_notify_charger(iphy, IUNIT);
					intel_otg_start_peripheral(otg, 1);
					otg->phy->state
						= OTG_STATE_B_PERIPHERAL;
					break;
				case POWER_SUPPLY_CHARGER_TYPE_NONE:
					intel_otg_notify_charger(iphy,
							ICFG_MAX);
					pm_runtime_put_noidle(otg->phy->dev);
					pm_runtime_suspend(otg->phy->dev);
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
		} else {
			/*
			 * If charger detection work is pending, decrement
			 * the pm usage counter to balance with the one that
			 * is incremented in charger detection work.
			 */
			dev_dbg(iphy->dev, "chg_work cancel");
			cancel_delayed_work_sync(&iphy->chg_work);
			iphy->chg_state = USB_CHG_STATE_UNDEFINED;
			intel_otg_notify_charger(iphy, 0);
			iphy->chg_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
			pm_runtime_put_noidle(otg->phy->dev);
			pm_runtime_suspend(otg->phy->dev);
		}
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (!test_bit(B_SESS_VLD, &iphy->inputs) ||
				!test_bit(ID, &iphy->inputs)) {
			iphy->chg_state = USB_CHG_STATE_UNDEFINED;
			intel_otg_notify_charger(iphy, 0);
			iphy->chg_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
			if (atomic_read(&iphy->bus_suspended))
				pm_runtime_barrier(otg->phy->dev);
			intel_otg_start_peripheral(otg, 0);
			otg->phy->state = OTG_STATE_B_IDLE;
			work = 1;
		} else if (test_bit(A_BUS_SUSPEND, &iphy->inputs) &&
				test_bit(B_SESS_VLD, &iphy->inputs)) {
			dev_dbg(iphy->dev, "a_bus_suspend && b_sess_vld\n");
			pm_runtime_put_noidle(otg->phy->dev);
			pm_runtime_suspend(otg->phy->dev);
		}
		break;
	case OTG_STATE_A_HOST:
		if (test_bit(ID, &iphy->inputs)) {
			intel_otg_start_host(otg, 0);
			otg->phy->state = OTG_STATE_B_IDLE;
			work = 1;
		}
		break;
	default:
		break;
	}

	if (work)
		schedule_work(&iphy->sm_work);
}

static int intel_otg_mode_show(struct seq_file *s, void *unused)
{
	struct intel_usbphy *iphy = s->private;
	struct usb_otg *otg = iphy->phy.otg;

	switch (otg->phy->state) {
	case OTG_STATE_A_HOST:
		seq_puts(s, "host\n");
		break;
	case OTG_STATE_B_PERIPHERAL:
		seq_puts(s, "peripheral\n");
		break;
	default:
		seq_puts(s, "none\n");
		break;
	}

	return 0;
}

static int intel_otg_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, intel_otg_mode_show, inode->i_private);
}

static ssize_t intel_otg_mode_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct intel_usbphy *iphy = s->private;
	char buf[16];
	struct usb_phy *phy = &iphy->phy;
	int status = count;
	enum usb_mode_type req_mode;

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		status = -EFAULT;
		goto out;
	}

	if (!strncmp(buf, "host", 4)) {
		req_mode = USB_HOST;
	} else if (!strncmp(buf, "peripheral", 10)) {
		req_mode = USB_PERIPHERAL;
	} else if (!strncmp(buf, "none", 4)) {
		req_mode = USB_NONE;
	} else {
		status = -EINVAL;
		goto out;
	}

	switch (req_mode) {
	case USB_NONE:
		switch (phy->state) {
		case OTG_STATE_A_HOST:
		case OTG_STATE_B_PERIPHERAL:
			set_bit(ID, &iphy->inputs);
			clear_bit(B_SESS_VLD, &iphy->inputs);
			break;
		default:
			goto out;
		}
		break;
	case USB_PERIPHERAL:
		switch (phy->state) {
		case OTG_STATE_B_IDLE:
		case OTG_STATE_A_HOST:
			set_bit(ID, &iphy->inputs);
			set_bit(B_SESS_VLD, &iphy->inputs);
			break;
		default:
			goto out;
		}
		break;
	case USB_HOST:
		switch (phy->state) {
		case OTG_STATE_B_IDLE:
		case OTG_STATE_B_PERIPHERAL:
			clear_bit(ID, &iphy->inputs);
			break;
		default:
			goto out;
		}
		break;
	default:
		goto out;
	}

	pm_runtime_resume(phy->dev);
	schedule_work(&iphy->sm_work);
out:
	return status;
}

const struct file_operations intel_otg_mode_fops = {
	.open = intel_otg_mode_open,
	.read = seq_read,
	.write = intel_otg_mode_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int intel_otg_show_chg_type(struct seq_file *s, void *unused)
{
	struct intel_usbphy *iphy = s->private;

	seq_printf(s, "%s\n", chg_to_string(iphy->chg_type));
	return 0;
}

static int intel_otg_chg_open(struct inode *inode, struct file *file)
{
	return single_open(file, intel_otg_show_chg_type, inode->i_private);
}

const struct file_operations intel_otg_chg_fops = {
	.open = intel_otg_chg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int intel_otg_show_otg_state(struct seq_file *s, void *unused)
{
	struct intel_usbphy *iphy = s->private;
	struct usb_phy *phy = &iphy->phy;

	seq_printf(s, "%s\n", usb_otg_state_string(phy->state));
	return 0;
}

static int intel_otg_otg_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, intel_otg_show_otg_state, inode->i_private);
}

const struct file_operations intel_otg_state_fops = {
	.open = intel_otg_otg_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *intel_otg_dbg_root;

static int intel_otg_debugfs_init(struct intel_usbphy *iphy)
{
	struct dentry *intel_otg_dentry;

	intel_otg_dbg_root = debugfs_create_dir("intel_otg", NULL);

	if (!intel_otg_dbg_root || IS_ERR(intel_otg_dbg_root))
		return -ENODEV;

	intel_otg_dentry = debugfs_create_file("mode", S_IRUGO | S_IWUSR,
		intel_otg_dbg_root, iphy,
		&intel_otg_mode_fops);

	if (!intel_otg_dentry) {
		debugfs_remove(intel_otg_dbg_root);
		intel_otg_dbg_root = NULL;
		return -ENODEV;
	}

	intel_otg_dentry = debugfs_create_file("chg_type", S_IRUGO,
		intel_otg_dbg_root, iphy,
		&intel_otg_chg_fops);

	if (!intel_otg_dentry) {
		debugfs_remove_recursive(intel_otg_dbg_root);
		return -ENODEV;
	}

	intel_otg_dentry = debugfs_create_file("otg_state", S_IRUGO,
		intel_otg_dbg_root, iphy,
		&intel_otg_state_fops);

	if (!intel_otg_dentry) {
		debugfs_remove_recursive(intel_otg_dbg_root);
		return -ENODEV;
	}
	return 0;
}

static void intel_otg_debugfs_cleanup(void)
{
	debugfs_remove_recursive(intel_otg_dbg_root);
}

static int intel_usb2phy_probe(struct platform_device *pdev)
{
	struct device_pm_platdata *pm_platdata;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct usb_reset *reset = NULL;
	struct intel_usbphy *iphy;
	struct usb_otg *otg;
	struct resource *res;
	int len = 0, index = 0;
	int ret;
	int i = 0;

	iphy = devm_kzalloc(dev, sizeof(*iphy), GFP_KERNEL);
	if (!iphy) {
		dev_err(&pdev->dev, "unable to allocate memory for USB2 PHY\n");
		return -ENOMEM;
	}

	otg = devm_kzalloc(&pdev->dev, sizeof(*otg), GFP_KERNEL);
	if (!otg) {
		dev_err(&pdev->dev, "unable to allocate memory for USB OTG\n");
		return -ENOMEM;
	}

	/* Register to platform device pm */
	pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pm_platdata)) {
		dev_err(dev, "Error during device state pm init\n");
		return -EINVAL;
	}

	ret = platform_device_pm_set_class(pdev,
			pm_platdata->pm_user_name);
	if (ret) {
		dev_err(&pdev->dev, "Error while setting the pm class\n");
		return ret;
	}

	/* Get power states */
	len = of_property_count_strings(np, "states-names");
	for (index = 0; index < len; index++) {
		const char *usb_power_state;
		of_property_read_string_index(np, "states-names",
						index, &usb_power_state);
		dev_dbg(dev, "get state handler for %s\n", usb_power_state);
		iphy->pm_states[index] =
			device_state_pm_get_state_handler(dev,
							usb_power_state);
		if (!iphy->pm_states[index]) {
			dev_err(dev, "pm get state handler failed");
			return -EINVAL;
		}
	}

	/*
	 * Request SCU memory region
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "scu_usb");
	if (!res) {
		dev_err(dev, "could not determine device base address\n");
		return -EINVAL;
	}

	iphy->scuregs = devm_ioremap_resource(dev, res);
	if (IS_ERR_OR_NULL(iphy->scuregs))
		return -ENOMEM;

	PARSE_USB_STATUS(drvvbus);
	PARSE_USB_STATUS(ridgnd);
	PARSE_USB_STATUS(fsvplus);
	PARSE_USB_STATUS(chrgdet);
	PARSE_USB_STATUS(hsrs);
	PARSE_USB_ACCESSOR(trim, true);
	PARSE_USB_ACCESSOR(phy_sus, true);
	PARSE_USB_ACCESSOR(pll_en, false);
	PARSE_USB_ACCESSOR(avalid, false);
	PARSE_USB_ACCESSOR(bvalid, false);
	PARSE_USB_ACCESSOR(vbusvalid, false);
	PARSE_USB_ACCESSOR(sessend, true);
	PARSE_USB_ACCESSOR(commononn, true);
	PARSE_USB_ACCESSOR(srp_clear, false);
	PARSE_USB_ACCESSOR(vdatsrcenb, false);
	PARSE_USB_ACCESSOR(vdatdetenb, false);
	PARSE_USB_ACCESSOR(dcdenb, false);
	PARSE_USB_ACCESSOR(chrgsel, false);

	/* Register Resume interrupt used to detect Resume from Host */
	iphy->resume_irq = platform_get_irq_byname(pdev, "resume");
	if (!IS_ERR_VALUE(iphy->resume_irq)) {
		ret = devm_request_irq(dev, iphy->resume_irq,
				intel_usb2phy_resume,
				IRQF_SHARED, "usb_resume", iphy);
		if (ret != 0) {
			dev_err(dev,
				"setup irq%d failed with ret = %d\n",
				iphy->resume_irq, ret);
			return -EINVAL;
		}
	} else {
		dev_info(dev, "resume irq not found\n");
	}

	/* Register reset controllers */
	if (of_find_property(np, "reset-names", NULL)) {
		len = of_property_count_strings(np, "reset-names");
		iphy->reset = devm_kzalloc(dev,
				sizeof(struct usb_reset), GFP_KERNEL);
		if (!iphy->reset) {
			dev_err(dev, "allocation of reset failed\n");
			return -ENOMEM;
		}
		INIT_LIST_HEAD(&iphy->reset->list);
		for (i = 0; i < len; i++) {
			reset = devm_kzalloc(dev,
					sizeof(struct usb_reset), GFP_KERNEL);
			if (!reset) {
				dev_err(dev, "allocation of reset failed\n");
				return -ENOMEM;
			}
			of_property_read_string_index(np, "reset-names", i,
						      &reset->res_name);
			reset->reset = reset_control_get(dev, reset->res_name);
			if (IS_ERR(reset->reset)) {
				dev_warn(dev, "missing ahb reset controller\n");
				reset->reset = NULL;
			}
			list_add_tail(&reset->list, &iphy->reset->list);
			dev_dbg(dev, "Add reset %s to usb reset list\n",
				reset->res_name);
		}
	}

	wake_lock_init(&iphy->wlock, WAKE_LOCK_SUSPEND, "intel_otg");
	INIT_WORK(&iphy->sm_work, intel_otg_sm_work);
	INIT_DELAYED_WORK(&iphy->chg_work, intel_chg_detect_work);
	init_completion(&iphy->bms_vbus_init);

	iphy->dev		= &pdev->dev;
	iphy->phy.dev		= iphy->dev;
	iphy->phy.label		= "intel-phy";
	iphy->phy.set_power	= intel_usb2phy_set_power;
	iphy->phy.set_suspend	= intel_usb2phy_set_suspend;
	iphy->phy.state		= OTG_STATE_UNDEFINED;
	iphy->phy.type		= USB_PHY_TYPE_USB2;

	iphy->phy.otg			= otg;
	iphy->phy.otg->phy		= &iphy->phy;
	iphy->phy.otg->set_host		= intel_usb2phy_set_host;
	iphy->phy.otg->set_peripheral	= intel_usb2phy_set_peripheral;

	ret = usb_add_phy_dev(&iphy->phy);
	if (ret) {
		dev_err(&pdev->dev, "can't register transceiver, err: %d\n",
			ret);
		return ret;
	}

	platform_set_drvdata(pdev, iphy);

	ATOMIC_INIT_NOTIFIER_HEAD(&iphy->phy.notifier);

	ret = intel_otg_debugfs_init(iphy);
	if (ret)
		dev_dbg(&pdev->dev, "mode debugfs file is not available\n");

	/*
	 * Enable USB controller and phy power
	 */
	ret = platform_device_pm_set_state_by_name(pdev, "enable");
	if (ret < 0) {
		dev_err(dev, "set power state enable failed\n");
		return ret;
	}

	ret = platform_device_pm_set_state_by_name(pdev, "enable_iso");
	if (ret < 0) {
		dev_err(dev, "set power state enable_iso failed\n");
		return ret;
	}


	usb_enable_reset(iphy, true, NULL);
	usb_enable_pll_en(iphy, false);
	mdelay(1);
	usb_enable_pll_en(iphy, true);
	usb_enable_reset(iphy, false, NULL);

	i = 0;
	while (usb_hsrs_status(iphy) && i++ < 500)
		udelay(1);

	pr_info("got usb reset in %d us\n", i);

	iphy->usb_nb.notifier_call = intel_usb2phy_notifier;
	usb_register_notifier(&iphy->phy, &iphy->usb_nb);
	device_init_wakeup(&pdev->dev, true);
	wake_lock(&iphy->wlock);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;
}

static int intel_usb2phy_remove(struct platform_device *pdev)
{
	struct intel_usbphy *iphy = platform_get_drvdata(pdev);
	struct usb_phy *phy = &iphy->phy;
	if (phy->otg->host || phy->otg->gadget)
		return -EBUSY;
	intel_otg_debugfs_cleanup();
	cancel_delayed_work_sync(&iphy->chg_work);
	cancel_work_sync(&iphy->sm_work);

	pm_runtime_resume(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	wake_lock_destroy(&iphy->wlock);

	usb_remove_phy(&iphy->phy);

	free_irq(iphy->resume_irq, iphy);

	iounmap(iphy->scuregs);
	pm_runtime_set_suspended(&pdev->dev);
	kfree(iphy->phy.otg);
	kfree(iphy);
	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int intel_otg_runtime_idle(struct device *dev)
{
	struct intel_usbphy *iphy = dev_get_drvdata(dev);
	struct usb_phy *phy = &iphy->phy;

	dev_dbg(dev, "OTG runtime idle\n");

	if (phy->state == OTG_STATE_UNDEFINED)
		return -EAGAIN;
	else
		return 0;
}

static int intel_otg_runtime_suspend(struct device *dev)
{
	struct intel_usbphy *iphy = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG runtime suspend\n");
	return intel_otg_suspend(iphy);
}

static int intel_otg_runtime_resume(struct device *dev)
{
	struct intel_usbphy *iphy = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG runtime resume\n");
	pm_runtime_get_noresume(dev);
	return intel_otg_resume(iphy);
}
#endif

#ifdef CONFIG_PM_SLEEP
static int intel_otg_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct intel_usbphy *iphy = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG PM suspend\n");

	atomic_set(&iphy->pm_suspended, 1);
	ret = intel_otg_suspend(iphy);
	if (ret)
		atomic_set(&iphy->pm_suspended, 0);

	return ret;
}

static int intel_otg_pm_resume(struct device *dev)
{
	int ret = 0;
	struct intel_usbphy *iphy = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG PM resume\n");

	atomic_set(&iphy->pm_suspended, 0);
	if (iphy->async_int || iphy->sm_work_pending) {
		pm_runtime_get_noresume(dev);
		ret = intel_otg_resume(iphy);

		/* Update runtime PM status */
		pm_runtime_disable(dev);
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);

		if (iphy->sm_work_pending) {
			iphy->sm_work_pending = false;
			schedule_work(&iphy->sm_work);
		}
	}

	return ret;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops intel_usb2phy_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(intel_otg_pm_suspend, intel_otg_pm_resume)
	SET_RUNTIME_PM_OPS(intel_otg_runtime_suspend, intel_otg_runtime_resume,
				intel_otg_runtime_idle)
};
#endif

static const struct of_device_id intel_usbphy_dt_match[] = {
	{
		.compatible = "intel,usb2phy",
	},
	{},
};
MODULE_DEVICE_TABLE(of, intel_usbphy_dt_match);

static struct platform_driver intel_usb2phy_driver = {
	.probe		= intel_usb2phy_probe,
	.remove		= intel_usb2phy_remove,
	.driver		= {
		.name	= "intel-usb2phy",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &intel_usb2phy_dev_pm_ops,
#endif
		.of_match_table = of_match_ptr(intel_usbphy_dt_match),
	},
};

static int __init intel_usb2phy_driver_init(void)
{
	return platform_driver_register(&intel_usb2phy_driver);
}
subsys_initcall(intel_usb2phy_driver_init);

static void __exit intel_usb2phy_driver_exit(void)
{
	platform_driver_unregister(&intel_usb2phy_driver);
}
module_exit(intel_usb2phy_driver_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:intel-usb2phy");
