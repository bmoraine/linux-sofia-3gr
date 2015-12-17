/*
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */

#define SMB345_NAME "smb345_charger"
#define pr_fmt(fmt) SMB345_NAME": "fmt

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/pinctrl/consumer.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/usb/phy-intel.h>

#include <linux/time.h>
#include <linux/wakelock.h>

/*
 * Development debugging is not enabled in release image to prevent
 * loss of event history in the debug array which has a limited size
 */
#include <linux/power/charger_debug.h>

/* I2C communication related */
#define I2C_RETRY_COUNT 3
#define I2C_RETRY_DELAY 5

#define CFG_CHARGE_CURRENT            0x00
#define CFG_CHARGE_CURRENT_FCC_MASK        0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT        5
#define CFG_CHARGE_CURRENT_PCC_MASK        0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT        3
#define CFG_CHARGE_CURRENT_TC_MASK        0x07
#define CFG_CHARGE_CURRENT_ALL        0x41
#define CFG_CURRENT_LIMIT            0x01
#define CFG_CURRENT_LIMIT_DC_MASK        0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT        4
#define CFG_CURRENT_LIMIT_USB_MASK        0x0f
#define CFG_CURRENT_LIMIT_SMB346_MASK   0xf0
#define CFG_CURRENT_LIMIT_SMB346_VALUE_2000 0x70
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1800 0x60
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1500 0x50
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1200 0x40
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1000 0x30
#define CFG_CURRENT_LIMIT_SMB346_VALUE_700 0x20
#define CFG_CURRENT_LIMIT_SMB346_VALUE_500 0x10
#define IBUS_MIN_LIMIT_MA 100
#define IBUS_LIMIT_STEP_MA 400
#define IBUS_MAX_LIMIT_MA 900
#define CFG_VAR_FUNC		0x02
#define CFG_VAR_FUNC_AICL_MASK BIT(4)
#define CFG_VAR_FUNC_DISABLE_AICL  0x00
#define CFG_VAR_FUNC_ENABLE_AICL  0x10
#define FLOAT_VOLTAGE_REG        0x03

#define IOCHARGE_MAX_MA 1500
#define DEFAULT_CC 350
#define DEFAULT_CV 3380

#define MAX_NR_OF_I2C_RETRIES 1
#define CHRGR_WORK_DELAY (10*HZ)
#define BOOST_WORK_DELAY (10*HZ)
#define EVT_WAKELOCK_TIMEOUT (2*HZ)
#define DBG_STATE_FILENAME "charger_state"

/* Pin and enable control */
#define CFG_PIN                    0x06
#define CFG_PIN_EN_CTRL_MASK            0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH        0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW        0x60
#define CFG_PIN_EN_APSD_IRQ            BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR        BIT(2)
/* Command registers */
#define CMD_A                    0x30
#define CMD_A_CHG_ENABLED            BIT(1)
#define CMD_A_SUSPEND_ENABLED            BIT(2)
#define CMD_A_OTG_ENABLED            BIT(4)
#define CMD_A_ALLOW_WRITE            BIT(7)
#define CMD_B                    0x31
#define CMD_B_USB9_AND_HC_MODE    0x03
#define CMD_C                    0x33

/* Interrupt Status registers */
#define IRQSTAT_A                0x35
#define IRQSTAT_C                0x37
#define IRQSTAT_C_TERMINATION_STAT        BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ        BIT(1)
#define IRQSTAT_C_TAPER_IRQ            BIT(3)
#define IRQSTAT_D                0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT        BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ        BIT(3)
#define IRQSTAT_E                0x39
#define IRQSTAT_E_USBIN_UV_STAT            BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ            BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT            BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ            BIT(5)
#define IRQSTAT_F                0x3a
#define IRQSTAT_F_OTG_UV_IRQ            BIT(5)
#define IRQSTAT_F_OTG_UV_STAT            BIT(4)

/* Status registers */
#define STAT_A                    0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK        0x3f
#define STAT_B                    0x3c
#define STAT_C                    0x3d
#define STAT_C_CHG_ENABLED            BIT(0)
#define STAT_C_HOLDOFF_STAT            BIT(3)
#define STAT_C_CHG_MASK                0x06
#define STAT_C_CHG_SHIFT            1
#define STAT_C_CHG_TERM                BIT(5)
#define STAT_C_CHARGER_ERROR            BIT(6)
#define STAT_E                    0x3f

/* PMU Register used for POK status */
#define PMU_CONFIG_O    0x00U
    #define PMU_CONFIG(_base) ((_base) + PMU_CONFIG_O)

    #define PMU_CONFIG_CHGDET_O (BIT(1))
    #define PMU_CONFIG_CHGDET_M (BIT(1))

#define CHARGER_CONTROL_O 0x0
#define CHARGER_CONTROL(_base) ((_base) + CHARGER_CONTROL_O)
	#define CHARGER_CONTROL_CIEDG_O 26
	#define CHARGER_CONTROL_CIEDG_M 0x1
	#define CHARGER_CONTROL_CILVL_O 25
	#define CHARGER_CONTROL_CILVL_M 0x1
	#define CHARGER_CONTROL_CISENS_O 24
	#define CHARGER_CONTROL_CISENS_M 0x1
	#define CHARGER_CONTROL_CIEN_O 23
	#define CHARGER_CONTROL_CIEN_M 0x1
	#define CHARGER_CONTROL_CIDBT_O 17
	#define CHARGER_CONTROL_CIDBT_M 0x7
	#define CHARGER_CONTROL_CHGLVL_O 1
	#define CHARGER_CONTROL_CHGLVL_M 0x1
	/*chrg det*/
	#define CHARGER_CONTROL_CDETSENS_O 10
	#define CHARGER_CONTROL_CDETSENS_M 0x1
	#define CHARGER_CONTROL_CHDETLVL_O 6
	#define CHARGER_CONTROL_CHDETLVL_M 0x1
	#define CHARGER_CONTROL_CHDWEN_O 5
	#define CHARGER_CONTROL_CHDWEN_M 0x1

#define CHARGER_CONTROL_CIEDG_FALLING 0
#define CHARGER_CONTROL_CILVL_LOW 0
#define CHARGER_CONTROL_CISENS_EDGE 1
#define CHARGER_CONTROL_CIEN_EN 0
#define CHARGER_CONTROL_CHGLVL_LOW 0
#define CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE 0
/*chrg det*/
#define CHARGER_CONTROL_CDETSENS_EDGE 1
#define CHARGER_CONTROL_CHDETLVL_LOW 0
#define CHARGER_CONTROL_CHDETLVL_HIGH 1
#define CHARGER_CONTROL_CHDWEN_EN 1

#define CHARGER_CONTROL_WR_O 0x8
#define CHARGER_CONTROL_WR(_base) ((_base) + CHARGER_CONTROL_WR_O)
	#define CHARGER_CONTROL_WR_WS_O 0
	#define CHARGER_CONTROL_WR_WS_M 0x1

#define SMB345_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))
#define OTG_TLIM_THERM_CNTRL_REG                0x0A
#define OTG_CURRENT_LIMIT_AT_USBIN_MASK    SMB345_MASK(2, 2)
#define OTG_CURRENT_LIMIT_750mA    (BIT(2) | BIT(3))
#define OTG_CURRENT_LIMIT_500mA    BIT(3)
#define OTG_CURRENT_LIMIT_250mA    BIT(2)
#define OTG_BATTERY_UVLO_THRESHOLD_MASK    SMB345_MASK(2, 0)

#define fit_in_range(__val, __MIN, __MAX) ((__val > __MAX) ? __MAX : \
					(__val < __MIN) ? __MIN : __val)

#define SYSFS_FAKE_VBUS_SUPPORT 1

#define SYSFS_INPUT_VAL_LEN (1)
struct workqueue_struct *charger_work_queue = NULL;
struct delayed_work charger_work;

enum {
	POK_B_VALID = 0,
	POK_B_INVAL,

	BOOSTOV_OCCURED = 1,

	BATUV_OCCURED = 1,

	TSD_OCCURRED = 1,

	OVP_OCCURRED = 1,

	OVP_RECOV_OCCURRED = 1,

	T32_TO_OCCURRED = 1,

	VBUS_FAULT = 1,

	TREG_IS_ON = 1,

	OT_RECOV_OCCURRED = 1,

	VBUS_OFF = 0,
	VBUS_ON,
};

enum charger_status {
	SMB345_STATUS_UNKNOWN,
	SMB345_STATUS_READY,
	SMB345_STATUS_FAULT,
};

/**
 * struct smb345_state - SMB345 charger current state
 * @status		charger driver status. Please see enum definition for
 *			details.
 * @vbus		equals 'VBUS_ON' (1) if valid vbus connected
 *			otherwise is 'VBUS_OFF'. '-1' -  means uninitialized.
 * @cc			current output current [mA] set on HW.
 * @max_cc		maximum output current [mA] that can be set on HW.
 * @cv			current output voltage [mV] set on HW.
 * @iterm		HW charging termination current [mA]. Not used as
 *			termination is controlled by SW.
 * @inlmt		input current limit [mA].
 * @health		charger chip health.
 * @cable_type		type of currently attached cable.
 * @charger_enabled	informs if charger is enabled for use.
 * @charging_enabled	informs if charging is currently enabled or not.
 * @to_enable_boost	informs whether boost is to be enabled
 * @boost_enabled	informs if boost mode is enabled
 * @pok_b		value of POK_B signal
 * @ovp_flag		value of OVP flag bit
 * @ovp_recov		value of OVP_RECOV flag bit
 * @t32s_timer_expired	charging watchdog timer expiration flag
 * @vbus_fault		'1' if vbus is fault, '0' otherwise
 * @treg_flag		charger thermal regulation active flag
 * @ot_recov_flag	over temperature recovery flag
 * @tsd_flag		thermal shutdown fault
 * @bat_uv		Battery voltage below 2.7v
 * @boost_ov		Boost out of regulation due to sustained current limit.
 */
struct smb345_state {
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
	unsigned int pok_b:1;
	unsigned int ovp_flag:1;
	unsigned int ovp_recov:1;
	unsigned int t32s_timer_expired:1;
	unsigned int vbus_fault:1;
	unsigned int treg_flag:1;
	unsigned int ot_recov_flag:1;
	unsigned int tsd_flag:1;

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
	struct delayed_work work;
	struct wake_lock evt_wakelock;
	bool in_suspend;
	bool pending_evt;
	spinlock_t lock;
};


/**
 * struct smb345_charger - SMB345 charger driver internal structure
 * @client			pointer to  i2c client's structure
 * @ididev			pointer to idi device
 * @chgint_bh			structure describing bottom half of
 *				CHGINT irq. See structure definition for
 *				details.
 * @boost_op_bh			structure describing bottom half of boost
 *				enable/disable operation.
 * @charging_work		work providing charging heartbeat for PSC
 * @boost_work			work feeding watchdog during boost mode
 * @otg_handle			Pointer to USB OTG internal structure
 *				used for sending VBUS notifications.
 * @usb_psy			power supply instance struct for USB path
 * @ac_psy			power supply instance struct for AC path
 * @current_psy			pointer to psy representing current charging
 *				path.
 * @prop_lock			synchronization semaphore
 * @model_name			model name of charger chip
 * @manufacturer		manufacturer name of charger chip
 * @ack_time			last CONTINUE_CHARGING timestamp in jiffies
 * @otg_nb			OTG notifier block
 * @fake_vbus			value of fake vbus event
 * @state			charger state structure
 */
struct smb345_charger {
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

	struct semaphore prop_lock;
	struct wake_lock suspend_lock;
	const char *model_name;
	const char *manufacturer;

	unsigned long ack_time;
	struct notifier_block otg_nb;

	int fake_vbus;
	struct smb345_state state;
	int chgint_irq;
	int chgdet_irq;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct device_pm_platdata *pm_platdata;

	int chg_otg_en_gpio;
	struct power_supply_cable_props *props;
	unsigned long chgint_debounce;
};

static int smb345_otg_notification_handler(struct notifier_block *nb,
		unsigned long event, void *data);

static struct smb345_charger chrgr_data = {
	.model_name = "SMB345",
	.manufacturer = "TBD",

	.otg_nb = {
		.notifier_call = smb345_otg_notification_handler,
	},

	.chgint_bh = {
		.in_suspend = false,
		.pending_evt = false,
	},
	.boost_op_bh = {
		.in_suspend = false,
		.pending_evt = false,
	},
	.fake_vbus = -1,

	.state = {
		.status = SMB345_STATUS_UNKNOWN,
		.vbus = -1,

	},
};
static int smb345_enable_charging(struct smb345_charger *chrgr,
		bool enable, int ma);
static int smb345_i2c_read_reg(struct i2c_client *client,
		u8 reg_addr, u8 *data);

static int smb345_i2c_write_reg(struct i2c_client *client,
		u8 reg_addr, u8 data);

static struct charger_debug_data chrgr_dbg;

static struct power_supply_throttle smb345_dummy_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
	},
};

static char *smb345_supplied_to[] = {
		"battery",
};

static enum power_supply_property smb345_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE
};

static struct smb345_charger *smb345_dev;

#ifdef SYSFS_FAKE_VBUS_SUPPORT

/**
 * fake_vbus_show	Called when value queried from driver to sysfs
 * @dev			[in] pointer to device
 * @attr		[in] pointer to devices's attribute
 * @buf			[out] pointer to buffer that is to be filled with
 *			string. Passed from driver to sysfs
 *
 * Returns:		number of characters read
 */
static ssize_t fake_vbus_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct smb345_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
	size_t size_copied;
	int value;

	value = chrgr->fake_vbus;
	size_copied = sprintf(buf, "%d\n", value);

	return size_copied;
}

/**
 * fake_vbus_store	Called when value written from sysfs to driver
 * @dev			[in] pointer to device
 * @attr		[in] pointer to devices's attribute
 * @buf			[in] pointer to buffer containing string passed
 *			from sysfs
 * @count		[in] number of characters in string
 *
 * Returns:		number of characters written
 */
static ssize_t fake_vbus_store(
			struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct smb345_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
	int sysfs_val;
	int ret;
	size_t size_to_cpy;
	char strvalue[SYSFS_INPUT_VAL_LEN + 1];

	size_to_cpy = (count > SYSFS_INPUT_VAL_LEN) ?
				SYSFS_INPUT_VAL_LEN : count;

	strncpy(strvalue, buf, size_to_cpy);
	strvalue[size_to_cpy] = '\0';

	ret = kstrtoint(strvalue, 10, &sysfs_val);
	if (ret != 0)
		return ret;

	sysfs_val = (sysfs_val == 0) ? 0 : 1;

	down(&chrgr->prop_lock);

	chrgr->fake_vbus = -1;

	if (chrgr->state.vbus != 1) {
		pr_err("fake vbus event requested when USB cable removed !\n");
		up(&chrgr->prop_lock);
		return count;
	}

	if (sysfs_val == 0) {
		chrgr->fake_vbus = 0;
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
				USB_EVENT_VBUS, &chrgr->fake_vbus);

		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_FAKE_VBUS,
						chrgr->fake_vbus, 0);
		pr_info("fake vbus removal sent.\n");

	} else if (sysfs_val == 1 && !chrgr->state.charging_enabled) {
		chrgr->fake_vbus = 1;
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
				USB_EVENT_VBUS, &chrgr->fake_vbus);

		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_FAKE_VBUS,
						chrgr->fake_vbus, 0);
		pr_info("fake vbus connection sent\n");
	}

	up(&chrgr->prop_lock);

	return count;
}

static struct device_attribute smb345_fake_vbus_attr = {
	.attr = {
		.name = "fake_vbus_event",
		.mode = S_IRUSR | S_IWUSR,
	},
	.show = fake_vbus_show,
	.store = fake_vbus_store,
};

/**
 * smb345_setup_sysfs_attr	Sets up sysfs entries for smb345 i2c device
 * @chrgr			[in] pointer to charger driver internal
 *				structure
 */
static void smb345_setup_fake_vbus_sysfs_attr(struct smb345_charger *chrgr)
{
	struct device *dev = &chrgr->client->dev;
	int err;

	err = device_create_file(dev, &smb345_fake_vbus_attr);
	if (err)
		pr_err("Unable to create sysfs entry: '%s'\n",
				smb345_fake_vbus_attr.attr.name);
}

#else

static inline void smb345_setup_fake_vbus_sysfs_attr(
					struct smb345_charger *chrgr)
{
	(void) chrgr;
}

#endif /*SYSFS_FAKE_VBUS_SUPPORT*/

static int smb345_i2c_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;
	struct smb345_charger *chrgr  = i2c_get_clientdata(client);

	do {
		ret = i2c_smbus_read_byte_data(chrgr->client, reg);
		if (ret < 0) {
			retry_count--;
			dev_warn(&chrgr->client->dev,
				"i2c read fail: can't read reg 0x%02X: %d\n",
					reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	if (ret < 0)
		return ret;
	else
		*val = ret;

	return 0;
}

static int smb345_i2c_write_reg(struct i2c_client *client,
		u8 reg, u8 val)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;
	struct smb345_charger *chrgr = i2c_get_clientdata(client);

	do {
		ret = i2c_smbus_write_byte_data(chrgr->client, reg, val);
		if (ret < 0) {
			retry_count--;
			dev_err(&chrgr->client->dev,
			"i2c write fail: can't write %02X to %02X: %d\n",
					val, reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	return ret;
}

static int smb345_masked_write(struct i2c_client *client, int reg,
		u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = smb345_i2c_read_reg(client, reg, &temp);
	if (rc) {
		pr_err("smb345_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}

	temp &= ~mask;
	temp |= val & mask;
	rc = smb345_i2c_write_reg(client, reg, temp);
	if (rc) {
		pr_err("smb345_write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;
}

static int smb345_set_writable(struct smb345_charger *chrgr, bool writable)
{
	int ret;
	u8 val = 0;

	ret = smb345_i2c_read_reg(chrgr->client, CMD_A, &val);
	if (ret != 0)
		return ret;

	if (writable)
		val |= CMD_A_ALLOW_WRITE;
	else
		val &= ~CMD_A_ALLOW_WRITE;

	return smb345_i2c_write_reg(chrgr->client, CMD_A, val);
}

static int __maybe_unused smb345_set_voreg(struct smb345_charger *chrgr,
				int volt_to_set, int *volt_set, bool propagate)
{
	return 0;
}

static int __maybe_unused smb345_set_iocharge(
			struct smb345_charger *chrgr, int curr_to_set,
						int *curr_set, bool propagate)
{
	return 0;
}

static inline bool smb345_is_online(struct smb345_charger *chrgr,
						struct power_supply *psy)
{
	if (!chrgr->state.charger_enabled)
		return false;

	if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		return ((chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_CDP) ||
			(chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_DCP));

	else if (psy->type == POWER_SUPPLY_TYPE_USB)
		return ((chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
			(chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING));

	return false;
}

static int __maybe_unused smb345_set_ibus_limit(struct smb345_charger *chrgr,
						int ilim_to_set, int *ilim_set)
{
	return 0;
}

static int __maybe_unused smb345_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct smb345_charger *chrgr = &chrgr_data;
	down(&chrgr->prop_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		chrgr->state.charger_enabled = val->intval;
		break;
	default:
		break;
	};

	up(&chrgr->prop_lock);
	return 0;
}

static int __maybe_unused smb345_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct smb345_charger *chrgr = &chrgr_data;
	int ret = 0;

	down(&chrgr->prop_lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = ((chrgr->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
				(chrgr->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING));

		else if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = ((chrgr->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_DCP) ||
				(chrgr->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_CDP));
		else
			val->intval = 0;

		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = smb345_is_online(chrgr, psy);
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		val->intval = chrgr->state.charger_enabled;
		break;

	default:
		ret =  -EINVAL;
		break;
	};

	up(&chrgr->prop_lock);

	return ret;
}

static void smb345_set_boost(struct work_struct *work)
{
	int ret;
	struct smb345_charger *chrgr = &chrgr_data;
	int on = chrgr->state.to_enable_boost;

	pr_info("%s(): %s\n", __func__, (on) ? "enable" : "disable");

	ret = smb345_set_writable(chrgr, true);
	if (ret < 0)
		goto boost_fail;


	if (on) {
		/* I2C disable OTG function */
		ret = smb345_masked_write(chrgr->client, CMD_A, BIT(4), 0);
		if (ret) {
			pr_err("fail to set OTG Disable bit ret=%d\n", ret);
			goto boost_fail;
		}

		/* Switching Frequency change 1.5Mhz 07h[7]="1" */
		ret = smb345_masked_write(chrgr->client,
				CFG_CHARGE_CURRENT_TC_MASK, BIT(7), BIT(7));
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n",
					ret);
			goto boost_fail;
		}

		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(chrgr->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_CURRENT_LIMIT_AT_USBIN_MASK, 0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n",
					ret);
			goto boost_fail;
		}

		/* Set OTG battery UVLO threshold to 2.7V: 0Ah[1:0]="00" */
		ret = smb345_masked_write(chrgr->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_BATTERY_UVLO_THRESHOLD_MASK, 0);
		if (ret) {
			pr_err("fail to set OTG battery UVLO threshold to 2.7V ret=%d\n",
					ret);
			goto boost_fail;
		}

		/* Toggle to enable OTG function: output low */
		gpio_direction_output(chrgr->chg_otg_en_gpio, 1);

		/* Set OTG current limit to 500mA: 0Ah[3:2]="01" */
		ret = smb345_masked_write(chrgr->client,
				OTG_TLIM_THERM_CNTRL_REG,
				BIT(2) | BIT(3), BIT(2));

		if (ret) {
			pr_err("fail to set OTG current limit 500mA ret=%d\n",
					ret);
			goto boost_fail;
		}
	} else {
		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(chrgr->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_CURRENT_LIMIT_AT_USBIN_MASK, 0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n",
					ret);
			goto boost_fail;
		}

		/* Toggle to enable OTG function: output high */
		gpio_direction_output(chrgr->chg_otg_en_gpio, 0);
	}

	return;
boost_fail:
	pr_err("%s: fail to boost\n", __func__);
}
/**
 * smb345_chgint_cb_work_func	function executed by
 *				smb345_charger::chgint_cb_work work
 * @work			[in] pointer to associated 'work' structure
 */
static void smb345_chgint_cb_work_func(struct work_struct *work)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	struct smb345_charger *chrgr = &chrgr_data;
	int ret, vbus_state_prev;
	u8 status;

	down(&chrgr->prop_lock);
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_CHGINT_CB, 0, 0);

	vbus_state_prev = chrgr->state.vbus;

	pm_state_en = idi_peripheral_device_pm_get_state_handler(chrgr->ididev,
			"enable");

	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		goto chgint_fail;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
				chrgr->ididev, "disable");

	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		goto chgint_fail;
	}

	ret = idi_set_power_state(chrgr->ididev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		goto chgint_fail;
	}

	ret = smb345_i2c_read_reg(chrgr->client, STAT_C, &status);
	chrgr->state.vbus = (status & STAT_C_CHG_ENABLED) ? VBUS_ON : VBUS_OFF;
	/*FIXME DO I NEED THIS */
	ret = idi_set_power_state(chrgr->ididev, pm_state_dis, false);
	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

	/* If vbus status changed, then notify USB OTG */
	if (chrgr->state.vbus != vbus_state_prev) {
		if (chrgr->otg_handle) {
			pr_info("%s: vbus changed: %d\n", __func__,
					chrgr->state.vbus);
			atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
					USB_EVENT_VBUS, &chrgr->state.vbus);

			CHARGER_DEBUG_REL(
					chrgr_dbg, CHG_DBG_VBUS,
					chrgr->state.vbus, 0);
		}
	}

	/* XXX: CHGDET configuration */
	ret = idi_client_ioread(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), &regval);

	/* charger detection CHARGER_CONTROL_CHDETLVL*/
	regval &= ~(CHARGER_CONTROL_CHDETLVL_M << CHARGER_CONTROL_CHDETLVL_O);

	if (chrgr->state.vbus)
		regval |= (CHARGER_CONTROL_CHDETLVL_HIGH
				<< CHARGER_CONTROL_CHDETLVL_O);
	else
		regval |= (CHARGER_CONTROL_CHDETLVL_LOW
				<< CHARGER_CONTROL_CHDETLVL_O);

	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), regval);

	/* charger control WR strobe */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

	/* This second triggering of the write strobe is intentional
	 * to make sure CHARGER_CTRL value is propagated to HW properly
	 */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

chgint_fail:
	up(&chrgr->prop_lock);
}

static int unfreezable_bh_create(struct unfreezable_bh_struct *bh,
		const char *wq_name, const char *wakelock_name,
		void (*work_func)(struct work_struct *work))
{
	spin_lock_init(&bh->lock);

	/* Create private, single-threaded workqueue instead of using one of
	the system predefined workqueues to reduce latency */
	bh->wq = create_singlethread_workqueue(wq_name);

	if (NULL == bh->wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&bh->work, work_func);

	wake_lock_init(&bh->evt_wakelock,
			WAKE_LOCK_SUSPEND,
			wakelock_name);

	return 0;
}

static void unfreezable_bh_destroy(struct unfreezable_bh_struct *bh)
{
	cancel_delayed_work_sync(&bh->work);

	destroy_workqueue(bh->wq);

	wake_lock_destroy(&bh->evt_wakelock);
}

static void unfreezable_bh_schedule(struct unfreezable_bh_struct *bh)
{
	struct smb345_charger *chrgr = &chrgr_data;
	spin_lock(&bh->lock);

	if (!bh->in_suspend) {
		queue_delayed_work(bh->wq, &bh->work, chrgr->chgint_debounce);

		wake_lock_timeout(&bh->evt_wakelock, EVT_WAKELOCK_TIMEOUT);
	} else {
		bh->pending_evt = true;
	}

	spin_unlock(&bh->lock);
}

static void __maybe_unused unfreezable_bh_resume(
				struct unfreezable_bh_struct *bh)
{
	unsigned long flags;
	struct smb345_charger *chrgr = &chrgr_data;

	spin_lock_irqsave(&bh->lock, flags);

	bh->in_suspend = false;
	if (bh->pending_evt) {
		bh->pending_evt = false;

		queue_delayed_work(bh->wq, &bh->work, chrgr->chgint_debounce);

		wake_lock_timeout(&bh->evt_wakelock, EVT_WAKELOCK_TIMEOUT);
	}

	spin_unlock_irqrestore(&bh->lock, flags);
}

static void __maybe_unused unfreezable_bh_suspend(
				struct unfreezable_bh_struct *bh)
{
	unsigned long flags;

	spin_lock_irqsave(&bh->lock, flags);
	bh->in_suspend = true;
	spin_unlock_irqrestore(&bh->lock, flags);
}

static irqreturn_t smb345_charger_chgint_cb(int irq, void *dev)
{
	struct smb345_charger *chrgr = dev;

	unfreezable_bh_schedule(&chrgr->chgint_bh);
	return IRQ_HANDLED;
}

/**
 * smb345_configure_pmu_irq - function configuring PMU's Charger status IRQ
 * @chrgr		[in] pointer to charger driver internal structure
 */
static int smb345_configure_pmu_irq(struct smb345_charger *chrgr)
{
	int ret;

	/* register callback with PMU for CHGDET irq */
	ret = request_irq(chrgr->chgdet_irq,
		smb345_charger_chgint_cb,
			IRQF_NO_SUSPEND, SMB345_NAME, chrgr);
	if (ret != 0) {
		pr_err("Failed to register @PMU for GHGINT irq! ret=%d", ret);
		return ret;
	}

	return 0;
}

static int smb345_configure_pmu_regs(struct smb345_charger *chrgr)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	int ret;

	if (!chrgr->ctrl_io_res || !chrgr->ididev)
		return -EINVAL;

	pm_state_en =
		idi_peripheral_device_pm_get_state_handler(
				chrgr->ididev, "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return -EINVAL;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
				chrgr->ididev, "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return -EINVAL;
	}

	pr_info("Getting PM state handlers: OK\n");

	ret = idi_set_power_state(chrgr->ididev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return -EIO;
	}

	ret = idi_client_ioread(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), &regval);


	/* ChargerResetLevel - CHARGER_CONTROL_CHGLVL_LOW */
	regval &= ~(CHARGER_CONTROL_CHGLVL_M << CHARGER_CONTROL_CHGLVL_O);
	regval |= (CHARGER_CONTROL_CHGLVL_LOW << CHARGER_CONTROL_CHGLVL_O);

	/* charger detection CHARGER_CONTROL_CDETSENS*/
	regval &= ~(CHARGER_CONTROL_CDETSENS_M << CHARGER_CONTROL_CDETSENS_O);
	regval |= (CHARGER_CONTROL_CDETSENS_EDGE << CHARGER_CONTROL_CDETSENS_O);

	/* charger detection CHARGER_CONTROL_CHDETLVL*/
	regval &= ~(CHARGER_CONTROL_CHDETLVL_M << CHARGER_CONTROL_CHDETLVL_O);
	regval |= (CHARGER_CONTROL_CHDETLVL_LOW << CHARGER_CONTROL_CHDETLVL_O);

	/* charger detection CHARGER_CONTROL_CHDWEN*/
	regval &= ~(CHARGER_CONTROL_CHDWEN_M << CHARGER_CONTROL_CHDWEN_O);
	regval |= (CHARGER_CONTROL_CHDWEN_EN << CHARGER_CONTROL_CHDWEN_O);

	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), regval);

	/* charger control WR strobe */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

	/* This second triggering of the write strobe is intentional
	 * to make sure CHARGER_CTRL value is propagated to HW properly
	 */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

	ret = idi_set_power_state(chrgr->ididev, pm_state_dis, false);

	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

	return 0;
}

static void do_charger(struct work_struct *work)
{
	struct smb345_charger *chrgr = &chrgr_data;

	down(&chrgr->prop_lock);

	pr_info("%s(), ma = %u, type = %d, connect = %d\n",
			__func__, chrgr->props->ma, chrgr->props->chrg_type,
			chrgr->props->chrg_evt);

	if (chrgr->props->chrg_evt == POWER_SUPPLY_CHARGER_EVENT_CONNECT) {
		chrgr->state.cable_type = chrgr->props->chrg_type;
		switch (chrgr->props->chrg_type) {
		case POWER_SUPPLY_CHARGER_TYPE_NONE:
			break;

		case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
		case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
			smb345_enable_charging(chrgr, true, chrgr->props->ma);

			break;

		default:
			break;

		}
	} else if (chrgr->props->chrg_evt
			== POWER_SUPPLY_CHARGER_EVENT_DISCONNECT) {
		smb345_enable_charging(chrgr, false, 0);
	}
	up(&chrgr->prop_lock);
}

static int smb345_write(struct smb345_charger *chrgr, u8 reg, u8 val)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		ret = i2c_smbus_write_byte_data(chrgr->client, reg, val);
		if (ret < 0) {
			retry_count--;
			dev_warn(&chrgr->client->dev,
					"fail to write reg %02xh: %d\n",
					reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	return ret;
}

static int smb345_read(struct smb345_charger *chrgr, u8 reg)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		ret = i2c_smbus_read_byte_data(chrgr->client, reg);
		if (ret < 0) {
			retry_count--;
			dev_warn(&chrgr->client->dev, "fail to read reg %02xh: %d\n",
					reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	return ret;
}
static int smb345_set_current_limits(struct smb345_charger *chrgr,
		int current_limit, bool is_twinsheaded)
{
	int ret;
	ret = smb345_set_writable(chrgr, true);
	if (ret < 0) {
		pr_info("%s() set writable as true failed, return %d\n",
				__func__, ret);
		return ret;
	}

	pr_info("%s()-cable_type = %d, current_limit = %d\n",
			__func__, chrgr->state.cable_type, current_limit);
	if (chrgr->state.cable_type == POWER_SUPPLY_CHARGER_TYPE_USB_SDP
			&& current_limit > 500) {
		pr_info("USB IN but current limit > 500, return\n");
		return 0;
	}

	switch (current_limit) {
	case 500:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_500);
		break;
	case 700:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_700);
		break;
	case 1000:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1000);
		break;

	case 1200:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1200);
		break;

	case 1500:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1500);
		break;

	case 1800:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1800);
		break;

	case 2000:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_2000);
		break;
	default:
		break;
	}
	return ret;
}

static int smb345_enable_charging(struct smb345_charger *chrgr,
		bool enable, int ma)
{
	int ret = 0;

	ret = smb345_set_writable(chrgr, true);
	if (ret < 0) {
		pr_err("%s() set writable as true failed, return %d\n",
				__func__, ret);
		goto fail;
	}

	pr_info("%s(), enable = %d for type=%d, ma=%d\n",
			__func__, enable, chrgr->state.cable_type, ma);
	if (enable == true) {
		if (ma < 500)
			return 0;

		/* config charge voltage as 4.3v */
		ret = smb345_masked_write(chrgr->client,
					FLOAT_VOLTAGE_REG,
					BIT(0) | BIT(1) | BIT(2) |
					BIT(3) | BIT(4) | BIT(5),
					BIT(0) | BIT(1) | BIT(5));

		/* In some cases, AICL will case the input current to be
		 * unexpectedly small. And trigger AICL again have no effect.
		 * So we need disable it to prevent current reduce.
		 */
		ret = smb345_masked_write(chrgr->client,
						CFG_VAR_FUNC,
						CFG_VAR_FUNC_AICL_MASK,
						CFG_VAR_FUNC_DISABLE_AICL);
		if (ret) {
			pr_err("fail to disable auto input current limit\n");
			goto fail;
		}

		/* Diable Charger, Set Pin to acitve High as EN is gounded */
		ret = smb345_masked_write(chrgr->client,
						CFG_PIN,
						CFG_PIN_EN_CTRL_MASK,
						CFG_PIN_EN_CTRL_ACTIVE_HIGH);
		if (ret) {
			pr_err("fail to disable charger\n");
			goto fail;
		}

		/* Configure current limits */
		if (smb345_set_current_limits(chrgr, ma, false) < 0) {
			dev_err(&chrgr->client->dev,
				"%s: fail to set max current limits\n",
				__func__);
			goto fail;
		}

		/* Re-enable Charger */
		ret = smb345_masked_write(chrgr->client,
						CFG_PIN,
						CFG_PIN_EN_CTRL_MASK,
						CFG_PIN_EN_CTRL_ACTIVE_LOW);
		if (ret) {
			pr_err("fail to enable charger\n");
			goto fail;
		}


		if (chrgr->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_SDP) {
			power_supply_changed(&chrgr->usb_psy);
			chrgr->state.charger_enabled = 1;
		} else if (chrgr->state.cable_type
				== POWER_SUPPLY_CHARGER_TYPE_USB_CDP ||
				chrgr->state.cable_type
				== POWER_SUPPLY_CHARGER_TYPE_USB_DCP) {
			power_supply_changed(&chrgr->ac_psy);
			chrgr->state.charger_enabled = 1;
		}

	} else {
		power_supply_changed(&chrgr->ac_psy);
		power_supply_changed(&chrgr->usb_psy);
		chrgr->state.charger_enabled = 0;
	}

fail:
	return ret;
}

static int smb345_otg_notification_handler(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct smb345_charger *chrgr = &chrgr_data;
	chrgr->props = (struct power_supply_cable_props *)data;

	switch (event) {
	case USB_EVENT_CHARGER:
		queue_delayed_work(charger_work_queue, &charger_work, 0);
		break;
	case INTEL_USB_DRV_VBUS:
		pr_info("%s() INTEL_USB_DRV_VBUS\n", __func__);
		if (!data)
			return NOTIFY_BAD;
		chrgr->state.to_enable_boost = *((bool *)data);
		unfreezable_bh_schedule(&chrgr->boost_op_bh);
		break;

	default:
		break;
	}
	return NOTIFY_OK;
}

static int smb345_set_pinctrl_state(struct i2c_client *client,
		struct pinctrl_state *state)
{
	struct smb345_charger *chrgr = i2c_get_clientdata(client);
	int ret;

	ret = pinctrl_select_state(chrgr->pinctrl, state);
	if (ret != 0) {
		pr_err("failed to configure CHGRESET pin !\n");
		return -EIO;
	}

	return 0;
}

int smb345_get_charging_status(void)
{
	int ret, status;
	int irqstat_c;

	if (!smb345_dev)
		return -EINVAL;

	ret = smb345_read(smb345_dev, STAT_C);
	if (ret < 0)
		return ret;

	irqstat_c = smb345_read(smb345_dev, IRQSTAT_C);
	if (irqstat_c < 0)
		return irqstat_c;

	if ((ret & STAT_C_CHARGER_ERROR) ||
		(ret & STAT_C_HOLDOFF_STAT)) {
		/* set to NOT CHARGING upon charger error
		 or charging has stopped. */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		if ((ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT)
			/* set to charging if battery is in pre-charge,
			 * fast charge or taper charging mode. */
			status = POWER_SUPPLY_STATUS_CHARGING;
		else if (ret & STAT_C_CHG_TERM)
			/* set the status to FULL if battery is not in
			 * precharge, fast charge or taper charging mode AND
			 * charging is terminated at least once. */
			status = POWER_SUPPLY_STATUS_FULL;
		else
			/* in this case no charger error or termination
			 * occured but charging is not in progress!!! */
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;

		if (irqstat_c & IRQSTAT_C_TERMINATION_STAT)
			status = POWER_SUPPLY_STATUS_FULL;
	}
	return status;
}

bool get_sw_charging_toggle(void)
{
	return true;
}

int get_charger_type(void)
{
	if (!smb345_dev) {
		pr_err("%s: smb345_dev is null due to probe function has error\n",
				__func__);
		return -1;
	}

	return 0;
}

void aicl_dete_worker(struct work_struct *dat)
{
	return;
}
EXPORT_SYMBOL(aicl_dete_worker);

bool smb345_has_charger_error(void)
{
	return false;
}

int smb345_dump_registers(struct seq_file *s)
{
	return 0;
}

static int smb345_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct usb_phy *otg_handle;
	struct smb345_charger *chrgr = &chrgr_data;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	int ret;
	u32 val;

	INIT_CHARGER_DEBUG_ARRAY(chrgr_dbg);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_PROBE, 0, 0);

	/* Get pinctrl configurations */
	chrgr->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(chrgr->pinctrl))
		BUG();

	chrgr->pins_default = pinctrl_lookup_state(chrgr->pinctrl,
						PINCTRL_STATE_DEFAULT);
	if (IS_ERR(chrgr->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	chrgr->pins_sleep = pinctrl_lookup_state(chrgr->pinctrl,
						PINCTRL_STATE_SLEEP);
	if (IS_ERR(chrgr->pins_sleep))
		dev_err(dev, "could not get sleep pinstate\n");

	chrgr->pins_inactive = pinctrl_lookup_state(chrgr->pinctrl,
						"inactive");
	if (IS_ERR(chrgr->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

	/* Register i2c clientdata before calling pinctrl */
	i2c_set_clientdata(client, chrgr);

	pr_info("%s\n", __func__);

	chrgr->client = client;

	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg_handle == NULL) {
		pr_err("ERROR!: getting OTG transceiver failed\n");
		return -EINVAL;
	}
	chrgr->otg_handle = otg_handle;

	/*
	 * Get interrupt from device tree
	 */
	chrgr->chgint_irq = irq_of_parse_and_map(np, 0);
	if (!chrgr->chgint_irq) {
		ret = -EINVAL;
		pr_err("can't get irq\n");
		goto remap_fail;
	}
	client->irq = chrgr->chgint_irq;

	chrgr->chgdet_irq = irq_of_parse_and_map(np, 1);
	if (!chrgr->chgdet_irq) {
		ret = -EINVAL;
		pr_err("Unable to retrieve IRQ for charger detection\n");
		goto remap_fail;
	}

	chrgr->chg_otg_en_gpio = of_get_named_gpio(np, "intel,otg-enable", 0);
	if (!gpio_is_valid(chrgr->chg_otg_en_gpio)) {
		pr_err("Unable to retrieve intel,otg-enable pin\n");
	} else {
		ret = gpio_request(chrgr->chg_otg_en_gpio, "smb346_otg");
		if (ret < 0)
			pr_err("%s: request CHG_OTG gpio fail!\n", __func__);
	}

	ret = of_property_read_u32(np, "intel,chgint-debounce", &val);
	if (ret) {
		pr_err("Unable to retrieve intel,chgint-debounce\n");
		val = 0;
	}
	chrgr->chgint_debounce = msecs_to_jiffies(val);

	/* Setup the PMU registers. The charger IC reset line
	   is deasserted at this point. From this point onwards
	   the charger IC will act upon the values stored in its
	   registers instead of displaying default behaviour  */
	ret = smb345_configure_pmu_regs(chrgr);
	if (ret != 0)
		goto pre_fail;

	ret = smb345_set_writable(chrgr, true);

	if (ret != 0)
		pr_err("failed to enable writable\n");

	smb345_dev = chrgr;

	/* Refer to SMB345 Application Note 72 to solve serious problems */
	ret = smb345_masked_write(chrgr->client,
			OTG_TLIM_THERM_CNTRL_REG,
			OTG_CURRENT_LIMIT_AT_USBIN_MASK,
			OTG_CURRENT_LIMIT_250mA);
	if (ret < 0)
		return ret;

	sema_init(&chrgr->prop_lock, 1);

	/* Set up the wake lock to prevent suspend when charging. */
	wake_lock_init(&chrgr->suspend_lock,
			WAKE_LOCK_SUSPEND,
			"smb345_wake_lock");

	/* It is OK to setup the wake lock here, because it is
	before the interrupt is later enabled with the call:
	smb345_configure_pmu_irq(). */
	if (unfreezable_bh_create(&chrgr->chgint_bh, "chrgr_wq",
			"smb345_evt_lock", smb345_chgint_cb_work_func)) {
		ret = -ENOMEM;
		goto wq_creation_fail;
	}

	ret = smb345_set_pinctrl_state(client, chrgr->pins_default);
	if (ret != 0)
		return ret;

	chrgr->usb_psy.name           = "usb_charger";
	chrgr->usb_psy.type           = POWER_SUPPLY_TYPE_USB;
	chrgr->usb_psy.properties     = smb345_power_props;
	chrgr->usb_psy.num_properties = ARRAY_SIZE(smb345_power_props);
	chrgr->usb_psy.get_property   = smb345_charger_get_property;
	chrgr->usb_psy.set_property   = smb345_charger_set_property;
	chrgr->usb_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->usb_psy.supplied_to = smb345_supplied_to;
	chrgr->usb_psy.num_supplicants = ARRAY_SIZE(smb345_supplied_to);
	chrgr->usb_psy.throttle_states = smb345_dummy_throttle_states;
	chrgr->usb_psy.num_throttle_states =
				ARRAY_SIZE(smb345_dummy_throttle_states);

	chrgr->current_psy = &chrgr->usb_psy;

	ret = power_supply_register(&client->dev, &chrgr->usb_psy);
	if (ret)
		goto fail;

	chrgr->ac_psy.name           = "ac_charger";
	chrgr->ac_psy.type           = POWER_SUPPLY_TYPE_MAINS;
	chrgr->ac_psy.properties     = smb345_power_props;
	chrgr->ac_psy.num_properties = ARRAY_SIZE(smb345_power_props);
	chrgr->ac_psy.get_property   = smb345_charger_get_property;
	chrgr->ac_psy.set_property   = smb345_charger_set_property;
	chrgr->ac_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->ac_psy.supplied_to = smb345_supplied_to;
	chrgr->ac_psy.num_supplicants = ARRAY_SIZE(smb345_supplied_to);
	chrgr->ac_psy.throttle_states = smb345_dummy_throttle_states;
	chrgr->ac_psy.num_throttle_states =
				ARRAY_SIZE(smb345_dummy_throttle_states);

	ret = power_supply_register(&client->dev, &chrgr->ac_psy);
	if (ret)
		goto fail_ac_registr;


	chrgr->ack_time = jiffies;

	ret = smb345_configure_pmu_irq(chrgr);
	if (ret != 0)
		goto pmu_irq_fail;

	i2c_set_clientdata(client, chrgr);

	smb345_setup_fake_vbus_sysfs_attr(chrgr);
	charger_work_queue = create_singlethread_workqueue("charger_workqueue");

	/* Read the VBUS presence status for initial update by
	making a dummy interrupt bottom half invocation */
	queue_delayed_work(chrgr->chgint_bh.wq, &chrgr->chgint_bh.work, 0);

	if (unfreezable_bh_create(&chrgr->boost_op_bh, "boost_op_wq",
				"smb345_boost_lock", smb345_set_boost)) {
		ret = -ENOMEM;
		goto pmu_irq_fail;
	}

	ret = usb_register_notifier(otg_handle, &chrgr->otg_nb);
	if (ret) {
		pr_err("ERROR!: registration for OTG notifications failed\n");
		goto boost_reg_fail;
	}
	INIT_DELAYED_WORK(&charger_work, do_charger);
	device_init_wakeup(&client->dev, true);

	return 0;

boost_reg_fail:
	unfreezable_bh_destroy(&chrgr->boost_op_bh);
pmu_irq_fail:
	power_supply_unregister(&chrgr->ac_psy);
fail_ac_registr:
	power_supply_unregister(&chrgr->usb_psy);
fail:
	unfreezable_bh_destroy(&chrgr->chgint_bh);
pre_fail:
wq_creation_fail:
remap_fail:
	usb_put_phy(otg_handle);
	return ret;
}

static int __exit smb345_i2c_remove(struct i2c_client *client)
{
	int ret = 0;
	struct smb345_charger *chrgr = &chrgr_data;
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_REMOVE, 0, 0);

	free_irq(client->irq, chrgr);
	power_supply_unregister(&chrgr_data.usb_psy);
	power_supply_unregister(&chrgr_data.ac_psy);
	wake_lock_destroy(&chrgr_data.suspend_lock);

	unfreezable_bh_destroy(&chrgr_data.chgint_bh);
	unfreezable_bh_destroy(&chrgr_data.boost_op_bh);

	cancel_delayed_work_sync(&chrgr_data.charging_work);
	if (chrgr_data.otg_handle)
		usb_put_phy(chrgr_data.otg_handle);

	ret = smb345_set_pinctrl_state(client, chrgr->pins_inactive);

	return ret;
}

static int smb345_idi_probe(struct idi_peripheral_device *ididev,
					const struct idi_device_id *id)
{
	struct resource *res;
	struct smb345_charger *chrgr = &chrgr_data;
	int ret = 0;

	spin_lock_init(&chrgr_dbg.lock);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_PROBE, 0, 0);
	pr_info("%s\n", __func__);

	res = idi_get_resource_byname(&ididev->resources,
			IORESOURCE_MEM, "registers");

	if (res == NULL) {
		pr_err("getting pmu_chgr_ctrl resources failed!\n");
		return -EINVAL;
	}

	chrgr->ctrl_io_res = res;

	chrgr->ididev = ididev;

	ret = idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
								__func__);
		return ret;
	}

	return 0;
}

static int __exit smb345_idi_remove(struct idi_peripheral_device *ididev)
{
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_REMOVE, 0, 0);
	pr_info("%s\n", __func__);

	return 0;
}

static int smb345_suspend(struct device *dev)
{
	struct smb345_charger *chrgr = &chrgr_data;

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SUSPEND_OK, 0, 0);
	if (device_may_wakeup(dev)) {
		pr_info("%s: enable wakeirq\n", __func__);
		enable_irq_wake(chrgr->chgdet_irq);
	}
	unfreezable_bh_suspend(&chrgr_data.chgint_bh);
	unfreezable_bh_suspend(&chrgr_data.boost_op_bh);
	return 0;
}

static int smb345_resume(struct device *dev)
{
	struct smb345_charger *chrgr = &chrgr_data;
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_RESUME, 0, 0);

	unfreezable_bh_resume(&chrgr_data.chgint_bh);
	unfreezable_bh_resume(&chrgr_data.boost_op_bh);

	if (device_may_wakeup(dev)) {
		pr_info("%s: disable wakeirq\n", __func__);
		disable_irq_wake(chrgr->chgdet_irq);
	}
	return 0;
}

const struct dev_pm_ops smb345_pm = {
	.suspend = smb345_suspend,
	.resume = smb345_resume,
};


static const struct i2c_device_id smb345_id[] = {
	{"smb345_charger", 0}, { }
};

MODULE_DEVICE_TABLE(i2c, smb345_id);

static struct i2c_driver smb345_i2c_driver = {
	.probe          = smb345_i2c_probe,
	.remove         = smb345_i2c_remove,
	.id_table       = smb345_id,
	.driver = {
		.name   = SMB345_NAME,
		.owner  = THIS_MODULE,
		.pm = &smb345_pm,
	},
};

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_CHG,
	},

	{ /* end: all zeroes */},
};

static struct idi_peripheral_driver smb345_idi_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "smb345_idi",
		.pm = NULL,
	},
	.p_type = IDI_CHG,
	.id_table = idi_ids,
	.probe  = smb345_idi_probe,
	.remove = smb345_idi_remove,
};


static int __init smb345_init(void)
{
	int ret;

	ret = idi_register_peripheral_driver(&smb345_idi_driver);
	if (ret)
		return ret;


	ret = i2c_add_driver(&smb345_i2c_driver);
	if (ret)
		return ret;

	return 0;
}

late_initcall(smb345_init);

static void __exit smb345_exit(void)
{
	i2c_del_driver(&smb345_i2c_driver);
}
module_exit(smb345_exit);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Charger Driver for SMB345 charger IC");
