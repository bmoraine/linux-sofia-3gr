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
#include <linux/alarmtimer.h>

#include <linux/power/charger_debug.h>

struct charger_attrmap {
	const char *rpt;
	int type;
	int reg_addr;
	int shift;
	int mask;
};

enum ATTR_TYPE {
	FULL_REG,
	BITS,
};

enum FAN54x_ATTRS {
	IC_INFO_REG,
	CHARGE_CTRL0_REG,
	CHARGE_CTRL1_REG,
	CHARGE_CTRL2_REG,
	IBAT_REG,
	VOREG_REG,
	IBUS_REG,
	INT_REG,
	STATUS_REG,
	INT_MASK_REG,
	ST_MASK_REG,
	TMR_RST_REG,
	SAFETY_REG,
	MONITOR_REG,
	STATE_REG,
	ADP_CTRL_REG,
	ADP_CNT_REG,
	TMR_CTRL_REG,
	SP_CHARGER_REG,


	VENDOR_INFO,
	PN_INFO,
	REV_INFO,
	HZ_MODE,
	BOOST_EN,
	BOOST_UP,
	OTG_EN,
	LDO,
	IOCHARGE,
	ITERM,
	VOREG,
	IBUS,
	VLOWV,
	TMR_RST,
	T32_TO,
	CHG_EN,
	IO_LEVEL,

	ATTR_MAX,
};

int fan54x_attr_write(struct i2c_client *client,
				int attr,
				u8 val);

int fan54x_attr_read(struct i2c_client *client,
				int attr,
				u8 *val);

enum charger_status {
	FAN54x_STATUS_UNKNOWN,
	FAN54x_STATUS_READY,
	FAN54x_STATUS_FAULT,
};


/**
 * struct fan54x_state - FAN54x charger current state
 * @vbus		equals 'VBUS_ON' (1) if valid vbus connected
 *			otherwise is 'VBUS_OFF'. '-1' -  means uninitialized
 * @to_enable_boost	informs whether boost is to be enabled
 * @boost_enabled	informs if boost mode is enabled
 * @pok_b		value of POK_B
 * @ovp_flag		value of OVP flag bit
 * @ovp_recov		value of OVP_RECOV flag bit
 * @vbus_fault		'1' if vbus is fault, '0' otherwise
 * @tsd_flag		thermal shutdown fault
 * @bat_uv		Battery voltage below 2.7v
 * @boost_ov		Boost out of regulation due to sustained current limit.
 */
struct fan54x_state {
	enum charger_status status;
	int vbus;
	int cc;
	int max_cc;
	int cv;
	int iterm;
	int inlmt;
	int health;
	int cable_type;
	bool charger_enabled;
	bool charging_enabled;
	bool to_enable_boost;
	bool boost_enabled;
	int throttle;
	unsigned int pok_b:1;
	unsigned int ovp_flag:1;
	unsigned int ovp_recov:1;
	unsigned int t32s_timer_expired:1;
	unsigned int vbus_fault:1;
	unsigned int treg_flag:1;
	unsigned int ot_recov_flag:1;
	unsigned int tsd_flag:1;

	unsigned int vbus_ovp:1;
	unsigned int sleep_mode:1;
	unsigned int poor_input_source:1;
	unsigned int bat_ovp:1;
	unsigned int no_bat:1;

	/* Boost mode specific states */
	unsigned int bat_uv:1;
	unsigned int boost_ov:1;
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
struct	unfreezable_bh_struct {
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
 * @boost_op_bh			structure describing bottom half of boost
 *				enable/disable operation.
 * @charging_work		work providing charging heartbeat for PSC
 * @boost_work			work feeding watchdog during boost mode
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
 * @otg_nb			OTG notifier block
 * @fake_vbus			value of fake vbus event
 * @state			charger state structure
 */
struct fan54x_charger {
	int vendor;
	int pn;
	int rev;
	int max_voreg;
	int min_voreg;
	int max_iocharge;
	int min_iocharge;
	int max_ibus_limit;
	int min_ibus_limit;
	int default_cc;
	int default_cv;
	int *throttle_values;
	int throttle_levels;

	struct i2c_client *client;
	struct idi_peripheral_device *ididev;

	struct unfreezable_bh_struct chgint_bh;
	struct unfreezable_bh_struct boost_op_bh;
	struct unfreezable_bh_struct boost_bh;

	struct delayed_work charging_work;
	struct alarm boost_alarm;
	struct delayed_work chgdet_work;
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
	struct fan54x_state state;
	int irq;
	int irq_chgdet;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct pinctrl_state *pins_active;
	struct device_pm_platdata *pm_platdata;

	struct charger_attrmap *attrmap;
	int (*configure_chip)(struct fan54x_charger *chrgr,
					bool enable_charging);
	int (*enable_charger)(struct fan54x_charger *chrgr, bool enable);
	int (*enable_charging)(struct fan54x_charger *chrgr, bool enable);
	int (*get_charger_state)(struct fan54x_charger *chrgr);
	int (*get_clr_wdt_expiry_flag)(struct fan54x_charger *chrgr);
	int (*calc_iocharge_regval)(struct fan54x_charger *chrgr,
					int current_to_set_ma);
	int (*get_iocharge_val)(int regval);
};

#define MAX_NR_OF_I2C_RETRIES 1
#define CHRGR_WORK_DELAY (10*HZ)
#define BOOST_ALARM_DELAY (10) /* sec */
#define EVT_WAKELOCK_TIMEOUT (2*HZ)
#define EVENTS_LOG_FILENAME "events_log"
#define DBG_REGS_FILENAME "charger_regs"
#define DBG_STATE_FILENAME "charger_state"
#define LOG_LINE_LENGTH (64)
#define LINES_PER_PAGE (PAGE_SIZE/LOG_LINE_LENGTH)
#define SHADOW_REGS_NR 10

#define fit_in_range(__val, __MIN, __MAX) ((__val > __MAX) ? __MAX : \
					(__val < __MIN) ? __MIN : __val)

#define SYSFS_FAKE_VBUS_SUPPORT 1

#define SYSFS_INPUT_VAL_LEN (1)

extern struct charger_debug_data chrgr_dbg;

extern struct fan54x_charger fan54020_chrgr_data;
extern struct fan54x_charger fan54015_chrgr_data;

int fan54x_i2c_update_reg(struct i2c_client *client, u8 reg_addr,
					u8 mask, int shift, u8 data);
int fan54x_otg_notification_handler(struct notifier_block *nb,
				unsigned long event, void *data);
