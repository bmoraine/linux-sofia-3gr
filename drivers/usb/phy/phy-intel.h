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

/* Timeout (in msec) values (min - max) associated with OTG timers */

#define TA_WAIT_VRISE	100	/* ( - 100)  */
#define TA_WAIT_VFALL	500	/* ( - 1000) */

#define PHY_AUTO_SUSPEND_DEALY 150 /* ms */

/*
 * This option is set for embedded hosts or OTG devices in which leakage
 * currents are very minimal.
 */
#if 0
#define TA_WAIT_BCON	30000	/* (1100 - 30000) */
#endif

/*
 * Don't use this timer if SRP/ADP are not supported
 */
#define TA_WAIT_BCON	-1

/* Timeout variables */

#define A_WAIT_VRISE	0
#define A_WAIT_VFALL	1
#define A_WAIT_BCON	2
#define A_AIDL_BDIS	3
#define A_BIDL_ADIS	4
#define B_SRP_FAIL	5
#define B_ASE0_BRST	6
#define A_TST_MAINT	7
#define B_TST_SRP	8
#define B_TST_CONFIG	9

/*
 * Supported USB modes
 *
 * USB_PERIPHERAL       Only peripheral mode is supported.
 * USB_HOST             Only host mode is supported.
 * USB_OTG              OTG mode is supported.
 *
 */
enum usb_mode_type {
	USB_NONE = 0,
	USB_PERIPHERAL,
	USB_HOST,
	USB_OTG,
};

/*
 * Different states involved in USB charger detection.
 *
 * USB_CHG_STATE_UNDEFINED	USB charger is not connected or detection
 *				process is not yet started.
 * USB_CHG_STATE_WAIT_FOR_DCD   Waiting for Data pins contact.
 * USB_CHG_STATE_DCD_DONE       Data pin contact is detected.
 * USB_CHG_STATE_PRIMARY_DONE   Primary detection is completed (Detects
 *				between SDP and DCP/CDP).
 * USB_CHG_STATE_SECONDARY_DONE Secondary detection is completed (Detects
 *				between DCP and CDP).
 * USB_CHG_STATE_DETECTED       USB charger type is determined.
 *
 */
enum usb_chg_state {
	USB_CHG_STATE_UNDEFINED = 0,
	USB_CHG_STATE_WAIT_FOR_DCD,
	USB_CHG_STATE_DCD_DONE,
	USB_CHG_STATE_PRIMARY_DONE,
	USB_CHG_STATE_SECONDARY_DONE,
	USB_CHG_STATE_DETECTED,
};

enum usb_scu_io_master {
	SCU_IO_ACCESS_BY_VMM,
	SCU_IO_ACCESS_BY_LNX
};

struct intel_usb_addr {
	void __iomem			*logic_addr;
	uint32_t			 phy_addr;
	enum usb_scu_io_master		 scu_io_master;
};
/*
 * struct intel_usbphy: OTG/Phy driver data.
 * @phy: usb phy handle
 * @dev: device handle
 * @pm_states: platform device pm states handles
 * @scuregs: io_address of scu logical and physical
 * @sm_work: OTG state machine work
 * @sm_work_pending: Flag to ask for state machine work while in lpm
 * @id_irq: IRQ number used for ID pin detection
 * @resume_irq: IRQ number used for resume detection
 * @inputs: OTG state machine inputs
 * @in_lpm: Flag to indicate low power mode (LPM) state.
 * @pm_suspended: Flag to indicate deep sleep
 * @bus_suspended: Flag to indicate usb bus suspend
 * @async_int: IRQ line on which ASYNC interrupt arrived in LPM.
 * @wlock: Wake lock struct to prevent system suspend when
 *               USB is active.
 * @chg_work: Charger detection work.
 * @chg_state: The state of charger detection process.
 * @chg_type: The type of charger attached.
 * @cable_props: Power supply cable description
 * @cur_power: The amount of mA available from downstream port.
 * @active_tmout: List of current running timeouts from OTG state machine
 * @tmouts: List of expired timeouts from OTG state machine
 * @timer: The hrtimer to handle OTG state machine timeout
 * @usb_nb: The notifier block used to know about usb phy events
 * @bms_vbus_init: The completion to wait for first vbus event
 * @power_budget: VBUS power budget in mA for host mode.
 * @mode: Supported mode (OTG/peripheral/host).
 * @reset: List of USB HW resets
 */
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
	struct intel_usb_addr scuregs;
	struct work_struct sm_work;
	bool sm_work_pending;
	int id_irq;
	int resume_irq;
#define ID		0
#define B_SESS_VLD	1
#define ID_A		2
#define ID_B		3
#define ID_C		4
#define A_BUS_DROP	5
#define A_BUS_REQ	6
#define A_SRP_DET	7
#define A_VBUS_VLD	8
#define B_CONN		9
#define ADP_CHANGE	10
#define POWER_UP	11
#define A_CLR_ERR	12
#define A_BUS_RESUME	13
#define A_BUS_SUSPEND	14
#define A_CONN		15
#define B_BUS_REQ	16
	unsigned long inputs;
	atomic_t in_lpm;
	atomic_t pm_suspended;
	atomic_t bus_suspended;
	int host_bus_suspend;
	int device_bus_suspend;
	int async_int;
	struct wake_lock wlock;
	struct delayed_work chg_work;
	enum usb_chg_state chg_state;
	enum power_supply_charger_cable_type chg_type;
	struct power_supply_cable_props cable_props;
	unsigned dcd_time;
	unsigned cur_power;
	u8 active_tmout;
	unsigned long tmouts;
	bool tmout;
	struct hrtimer timer;
	struct notifier_block usb_nb;
	struct completion bms_vbus_init;
	int power_budget;
	enum usb_dr_mode mode;
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
	struct usb_reg *iddig0;
	struct usb_reg *drvvbus;
	struct usb_reg *srp_clear;
	struct usb_reg *vdatsrcenb;
	struct usb_reg *vdatdetenb;
	struct usb_reg *dcdenb;
	struct usb_reg *chrgsel;
	struct usb_reg *fsvplus;
	struct usb_reg *chrgdet;
	struct usb_reg *hsrs;
	int usbid_gpio;
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

