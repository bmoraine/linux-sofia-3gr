/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define IDEV_CHG_MAX	1500
#define ICFG_MAX	500
#define IUNIT		100

#define PCGCTL_RSTPDWNMODULE		(1 << 3)
#define PCGCTL_PWRCLMP			(1 << 2)
#define PCGCTL_STOPPCLK			(1 << 0)

enum usb_mode_type {
	USB_NONE = 0,
	USB_PERIPHERAL,
	USB_HOST,
	USB_OTG,
};

enum usb_chg_state {
	USB_CHG_STATE_UNDEFINED = 0,
	USB_CHG_STATE_WAIT_FOR_DCD,
	USB_CHG_STATE_DCD_DONE,
	USB_CHG_STATE_PRIMARY_DONE,
	USB_CHG_STATE_SECONDARY_DONE,
	USB_CHG_STATE_DETECTED,
};

struct intel_usbphy {
	struct usb_phy	phy;
	struct device	*dev;
#define	USB_PMS_DISABLE			0
#define	USB_PMS_SUSPEND			1
#define	USB_PMS_SUSPEND_NO_PSV		2
#define	USB_PMS_ENABLE			3
#define	USB_PMS_ENABLE_ISO		4
#define USB_PMS_MAX			5
	struct device_state_pm_state *pm_states[USB_PMS_MAX];
	void __iomem	*scuregs;
	spinlock_t	lock;
	struct work_struct sm_work;
	int id_irq;
	int resume_irq;
#define ID		0
#define B_SESS_VLD	1
#define A_BUS_SUSPEND	2
	unsigned long inputs;
	atomic_t in_lpm;
	atomic_t pm_suspended;
	atomic_t bus_suspended;
	bool sm_work_pending;
	int async_int;
	struct wake_lock wlock;
	struct delayed_work chg_work;
	enum usb_chg_state chg_state;
	enum power_supply_charger_cable_type chg_type;
	struct power_supply_cable_props cable_props;
	unsigned dcd_time;
	unsigned cur_power;
	struct notifier_block usb_nb;
	struct completion bms_vbus_init;
	struct usb_reset *reset;
	struct usb_reg *phy_sus;
	struct usb_reg *trim;
	struct usb_reg *pll_en;
	struct usb_reg *avalid;
	struct usb_reg *bvalid;
	struct usb_reg *vbusvalid;
	struct usb_reg *sessend;
	struct usb_reg *commononn;
	struct usb_reg *ridgnd;
	struct usb_reg *drvvbus;
	struct usb_reg *srp_clear;
	struct usb_reg *vdatsrcenb;
	struct usb_reg *vdatdetenb;
	struct usb_reg *dcdenb;
	struct usb_reg *chrgsel;
	struct usb_reg *fsvplus;
	struct usb_reg *chrgdet;
	struct usb_reg *hsrs;
};

struct usb_reg {
	struct list_head list;
	long base;
	long offset;
	long mask;
	int disable;
	int enable;
};

struct usb_reset {
	struct list_head list;
	const char *res_name;
	struct reset_control *reset;
};

