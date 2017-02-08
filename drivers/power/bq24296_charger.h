/*
 * -------------------------------------------------------------------------
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/pinctrl/consumer.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>

#include <linux/time.h>
#include <linux/wakelock.h>

#include <linux/power/charger_debug.h>

#ifndef BIT
#define BIT(x)	(1 << (x))
#endif

/* Register definitions */
#define BQ00_INPUT_SRC_CONT_REG              0X00
#define BQ01_PWR_ON_CONF_REG                 0X01
#define BQ02_CHARGE_CUR_CONT_REG             0X02
#define BQ03_PRE_CHARGE_TERM_CUR_REG         0X03
#define BQ04_CHARGE_VOLT_CONT_REG            0X04
#define BQ05_CHARGE_TERM_TIMER_CONT_REG      0X05
#define BQ06_IR_COMP_THERM_CONT_REG          0X06
#define BQ07_MISC_OPERATION_CONT_REG         0X07
#define BQ08_SYSTEM_STATUS_REG               0X08
#define BQ09_FAULT_REG                       0X09
#define BQ0A_VENDOR_PART_REV_STATUS_REG      0X0A

/* BQ00 Input Source Control Register MASK */
#define EN_HIZ			BIT(7)
#define VINDPM_MASK		(BIT(6)|BIT(5)|BIT(4)|BIT(3))
#define IINLIM_MASK		(BIT(2)|BIT(1)|BIT(0))

/* BQ01 Power-On Configuration  Register MASK */
#define RESET_REG_MASK		BIT(7)
#define WATCHDOG_TIME_RESET	BIT(6)
#define CHG_CONFIG_MASK		(BIT(5)|BIT(4))
#define OTG_ENABLE_MASK		BIT(5)

/* #define SYSTEM_MIN_VOLTAGE_MASK    0x0E */
#define SYS_MIN_VOL_MASK	(BIT(3)|BIT(2)|BIT(1))
#define BOOST_LIM		BIT(0)

/* BQ02 Charge Current Control Register MASK */
#define ICHG_MASK		(BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3)|BIT(2))
#define FORCE_20PCT_MASK	BIT(0)

/* BQ03 Pre-Charge, Termination Current Control Register MASK */
#define IPRECHG_MASK		(BIT(7)|BIT(6)|BIT(5)|BIT(4))
#define ITERM_MASK		(BIT(3)|BIT(2)|BIT(1)|BIT(0))

/* BQ04 Charge Voltage Control Register MASK */
#define CHG_VOLTAGE_LIMIT_MASK	(BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3)|BIT(2))
#define BATLOWV_MASK		BIT(1)
#define VRECHG_MASK		BIT(0)

/* BQ05 Charge Termination, Timer-Control Register MASK */
#define EN_CHG_TERM_MASK	BIT(7)
#define I2C_TIMER_MASK		(BIT(5)|BIT(4))
#define EN_CHG_TIMER_MASK	BIT(3)
#define CHG_TIMER_MASK		(BIT(2)|BIT(1))

/* BQ06 IR Compensation, Thermal Regulation Control Register MASK */
#define IR_COMP_R_MASK		(BIT(7)|BIT(6)|BIT(5))
#define IR_COMP_VCLAMP_MASK	(BIT(4)|BIT(3)|BIT(2))

/* BQ07 Misc-Operation Control Register MASK */
#define BATFET_DISABLE_MASK	BIT(5)

/* BQ08 SYSTEM_STATUS_REG Mask */
#define VBUS_STAT_MASK		(BIT(7)|BIT(6))
#define PRE_CHARGE_MASK		BIT(4)
#define FAST_CHARGE_MASK	BIT(5)
#define CHRG_STAT_MASK		(FAST_CHARGE_MASK|PRE_CHARGE_MASK)
#define DPM_STAT_MASK		BIT(3)
#define PG_STAT_MASK		BIT(2)
#define THERM_STAT_MASK		BIT(1)
#define VSYS_STAT_MASK		BIT(0)

/* BQ09 FAULT_REG Mask */
#define CHRG_FAULT_MASK		(BIT(5)|BIT(4))

#define LT_CABLE_56K		6
#define LT_CABLE_130K		7
#define LT_CABLE_910K		11

enum bq24296_chg_status {
	BQ_CHG_STATUS_NONE = 0,
	BQ_CHG_STATUS_PRE_CHARGE = 1,
	BQ_CHG_STATUS_FAST_CHARGE = 2,
	BQ_CHG_STATUS_FULL = 3,
	BQ_CHG_STATUS_EXCEPTION = 4,
};

static const char *const bq24296_chg_status[] = {
	"none",
	"pre-charge",
	"fast-charge",
	"full",
	"exception"
};

/**
 * struct fan54x_state - FAN54x charger current state
 * @vbus		equals 'VBUS_ON' (1) if valid vbus connected
 *			otherwise is 'VBUS_OFF'. '-1' -  means uninitialized
 * @pok_b		value of POK_B
 * @ovp_flag		value of OVP flag bit
 * @ovp_recov		value of OVP_RECOV flag bit
 * @vbus_fault		'1' if vbus is fault, '0' otherwise
 */
struct bq24296_state {
	enum bq24296_chg_status status;
	int vbus;
	int cc;
	int max_cc;
	int cv;
	int iterm;
	int inlmt;
	int health;
	int cable_type;
	int throttle;
	bool charger_enabled;
	bool charging_enabled;
	bool to_enable_boost;
	bool boost_enabled;
};

/**
 * struct unfreezable_bh_struct -  structure describing suspend aware bottom half
 * @wq				pointer to  i2c client's structure.
 * @work			work associated with irq.
 * @evt_wakelock		wakelock acquired when bottom half is scheduled.
 * @in_suspend			Suspend flag. 'true' if driver was suspended
 *				'false' otherwise.
 * @pending_evt			pending event for bottom half execution flag.
 * @lock			spinlock protecting in_suspend and pending_evt
 *				flags.
 */
struct unfreezable_bh_struct {
	struct workqueue_struct *wq;
	struct work_struct work;
	struct wake_lock evt_wakelock;
	bool in_suspend;
	bool pending_evt;
	spinlock_t lock;
};

/**
 * struct fan54020_charger - FAN54020 charger driver internal structure
 * @client			pointer to  i2c client's structure
 * @ididev			pointer to idi device
 * @chgint_bh			structure describing bottom half of
 *				CHGINT irq. See structure definition for
 *				details.
 * @charging_work		work providing charging heartbeat for PSC
 * @ctrl_io			PMU Charger regs physical address
 * @otg_handle			Pointer to USB OTG internal structure
 *				used for sending VBUS notifications.
 * @usb_psy			power supply instance struct for USB path
 * @ac_psy			power supply instance struct for AC path
 * @current_psy			pointer to psy representing current charging
 *				path.
 * @debugfs_root_dir		debugfs fan54020 charger root directory
 * @prop_lock			synchronization semaphore
 * @model_name			model name of charger chip
 * @manufacturer		manufacturer name of charger chip
 * @ack_time			last CONTINUE_CHARGING timestamp in jiffies
 * @fake_vbus			value of fake vbus event
 * @state			charger state structure
 */
struct bq24296_charger {
	int vendor;
	int chg_current_ma;
	int term_current_ma;
	int pre_chg_current_ma;
	int force_ichg_decrease;
	int vbat_full_mv;
	int vbat_min_mv;
	int vbat_max_mv;
	int max_vbus_ilimit;
	int min_vbus_ilimit;
	int sys_vmin_mv;
	int vbus_in_limit_mv;
	int irq;
	int usb_present;
	int usb_online;
	int ac_present;
	int ac_online;
	int chg_type;
	int charging_disabled;
	int *throttle_values;
	int throttle_levels;

	struct i2c_client *client;
	struct idi_peripheral_device *ididev;

	struct unfreezable_bh_struct chgint_bh;
	struct unfreezable_bh_struct boost_op_bh;

	struct delayed_work charging_work;
	struct delayed_work boost_work;
	struct resource *ctrl_io_res;
	struct usb_phy *otg_handle;

	struct power_supply usb_psy;
	struct power_supply ac_psy;

	struct power_supply *current_psy;

	struct dentry *debugfs_root_dir;

	struct semaphore prop_lock;
	struct wake_lock suspend_lock;
	const char *model_name;
	const char *manufacturer;

	unsigned long ack_time;
	struct notifier_block otg_nb;
	int fake_vbus;
	struct bq24296_state state;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct pinctrl_state *pins_active;
	struct device_pm_platdata *pm_platdata;

	int (*configure_chip)(struct bq24296_charger *chrgr,
		bool enable_charging);
	int (*enable_charger)(struct bq24296_charger *chrgr,
		bool enable);
};

#define MAX_NR_OF_I2C_RETRIES 1
#define CHRGR_WORK_DELAY (10*HZ)
#define BOOST_WORK_DELAY (10*HZ)
#define EVT_WAKELOCK_TIMEOUT (2*HZ)
#define EVENTS_LOG_FILENAME "events_log"
#define DBG_REGS_FILENAME "charger_regs"
#define DBG_STATE_FILENAME "charger_state"
#define DBG_CHARGING_STATE_FILENAME "charging_state"
#define LOG_LINE_LENGTH (64)
#define LINES_PER_PAGE (PAGE_SIZE/LOG_LINE_LENGTH)
#define SHADOW_REGS_NR 10

#define fit_in_range(__val, __MIN, __MAX) ((__val > __MAX) ? __MAX : \
					(__val < __MIN) ? __MIN : __val)

#define SYSFS_FAKE_VBUS_SUPPORT 1

#define SYSFS_INPUT_VAL_LEN (1)

extern struct charger_debug_data bq24296_chrgr_dbg;

extern struct bq24296_charger bq24296_chrgr_data;

int bq24296_otg_notification_handler(struct notifier_block *nb,
				     unsigned long event, void *data);

extern int bq24296_enable_charging(struct bq24296_charger *chrgr, bool enable);

extern int bq24296_configure_chip(struct bq24296_charger *chrgr,
				  bool enable_charging);
