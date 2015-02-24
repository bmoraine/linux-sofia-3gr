/*
 * -------------------------------------------------------------------------
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#define FAN54020_NAME "fan54020_charger"
#define pr_fmt(fmt) FAN54020_NAME": "fmt

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

#include <linux/alarmtimer.h>
#include <linux/hrtimer.h>


/*
 * Development debugging is not enabled in release image to prevent
 * loss of event history in the debug array which has a limited size
 */
#include <linux/power/charger_debug.h>

#define REG_IC_INFO 0x00
#define REG_CHARGE_CTRL1 0x01
#define REG_CHARGE_CTRL2 0x02
#define REG_IBAT 0x03
#define REG_VOREG 0x04
#define REG_IBUS 0x05
#define REG_INT 0x06
#define REG_STATUS 0x07
#define REG_INT_MASK 0x08
#define REG_ST_MASK 0x09
#define REG_TMR_RST 0x0a
#define REG_SAFETY 0x0f
#define REG_MONITOR 0x10
#define REG_STATE 0x1f
#define REG_ADP_CTRL 0x20
#define REG_ADP_CNT 0x21
#define REG_TMR_CTRL 0x22

/* REG_IC_INFO */
#define VENDOR 0x4
#define VENDOR_O 5
#define VENDOR_M 0x7

#define PN 0x1
#define PN_O 3
#define PN_M 0x3

#define REV_O 0
#define REV_M 0x7

/* REG_CHARGE_CTRL1 */
#define HZ_MODE_O 6
#define HZ_MODE_M 0x1

/* REG_CHARGE_CTRL2 */
#define ITERM_DIS_M 0x1
#define ITERM_DIS_O 0
#define LDO_OFF_O 4
#define BOOST_UP 5
#define BOOST_EN 6

/* REG_IBAT */
#define IOCHARGE_MIN_MA 350
#define IOCHARGE_STEP_MA 100
#define IOCHARGE_STEP_START_MA 400
#define IOCHARGE_STEP_START_REGVAL 1
#define IOCHARGE_MAX_MA 1500
#define IOCHARGE_O 4
#define IOCHARGE_M 0xf

#define DEFAULT_CC 350

/* REG_VOREG  */
#define VOREG_MIN_MV 3380
#define VOREG_STEP_MV 20
#define VOREG_MAX_MV 4440
#define VOREG_M 0x3f

#define DEFAULT_CV 3380

/* REG_IBUS */
#define IBUS_MIN_LIMIT_MA 100
#define IBUS_LIMIT_STEP_MA 400
#define IBUS_MAX_LIMIT_MA 900
#define IBUS_M 0x3
#define IBUS_O 0
#define IBUS_NO_LIMIT 3

/* REG_INT and REG_INT_MASK */
#define TSD_FLAG_O 7
#define OVP_FLAG_O 6
#define OVP_RECOV_O 1
#define BOOSTOV_O 5
#define TC_TO_O 4
#define BAT_UV_O 3
#define DBP_TO_O 3
#define TREG_FLAG_O 5
#define OT_RECOV_O 2
#define INT_MASK_ALL 0xff

/* REG_STATUS and REG_ST_MASK */
#define POK_B_O 6
#define ST_MASK_ALL 0xff

/* REG_TMR_RST */
#define TMR_RST_O 7

/* REG_SAFETY  */
#define ISAFE_O 4
#define VSAFE_O 0
#define FAN54020_CUR_LIMIT 0xf /* 1500mA */
#define FAN54020_VOLT_LIMIT 0xf /* 4440mV */

/* REG_STATE */
#define ST_CODE_O 4
#define ST_CODE_M 0xf
#define ST_VBUS_FAULT 0x1a
#define ST_VBUS_FAULT2 0x1d

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

#define CHARGER_CONTROL_CIEDG_FALLING 0
#define CHARGER_CONTROL_CILVL_LOW 0
#define CHARGER_CONTROL_CISENS_EDGE 1
#define CHARGER_CONTROL_CIEN_EN 0
#define CHARGER_CONTROL_CHGLVL_LOW 0
#define CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE 0

#define CHARGER_CONTROL_WR_O 0x8
#define CHARGER_CONTROL_WR(_base) ((_base) + CHARGER_CONTROL_WR_O)
	#define CHARGER_CONTROL_WR_WS_O 0
	#define CHARGER_CONTROL_WR_WS_M 0x1

#define MAX_NR_OF_I2C_RETRIES 1
#define CHRGR_WORK_DELAY (10*HZ)
#define BOOST_WORK_DELAY (10*HZ)
#define EVT_WAKELOCK_TIMEOUT (2*HZ)
#define EVENTS_LOG_FILENAME "events_log"
#define DBG_REGS_FILENAME "charger_regs"
#define DBG_STATE_FILENAME "charger_state"
#define LOG_LINE_LENGTH (64)
#define LINES_PER_PAGE (PAGE_SIZE/LOG_LINE_LENGTH)
#define SHADOW_REGS_NR 10

#define CHARGER_WATCHDOG_TIMER_SECS (10)

#define fit_in_range(__val, __MIN, __MAX) ((__val > __MAX) ? __MAX : \
					(__val < __MIN) ? __MIN : __val)

#define SYSFS_FAKE_VBUS_SUPPORT 1

#define SYSFS_INPUT_VAL_LEN (1)

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
	FAN54020_STATUS_UNKNOWN,
	FAN54020_STATUS_READY,
	FAN54020_STATUS_FAULT,
};

#define SET_ALARM_TIMER(__alarm, tout_delay) \
	do {\
		struct timespec __delta;\
		__delta.tv_sec = tout_delay;\
		__delta.tv_nsec = 0;\
		alarm_start_relative(__alarm, timespec_to_ktime(__delta));\
	} while (0)




/**
 * struct fan54020_state - FAN54020 charger current state
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
struct fan54020_state {
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
	struct work_struct work;
	struct wake_lock evt_wakelock;
	bool in_suspend;
	bool pending_evt;
	spinlock_t lock;
};

static void unfreezable_bh_schedule(struct unfreezable_bh_struct *bh);



/**
 * struct fan54020_charger - FAN54020 charger driver internal structure
 * @client			pointer to  i2c client's structure
 * @ididev			pointer to idi device
 * @chgint_bh			structure describing bottom half of
 *				CHGINT irq. See structure definition for
 *				details.
 * @boost_op_bh			structure describing bottom half of boost
 *				enable/disable operation.
 * @boost_worker_bh		structure describing bottom half of boost
 *				timer tick operation.
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
 * @otg_nb			OTG notifier block
 * @fake_vbus			value of fake vbus event
 * @state			charger state structure
 * @chrgr_boost_en_timer	timer to handle keep boost enabled
 */
struct fan54020_charger {
	struct i2c_client *client;
	struct idi_peripheral_device *ididev;

	struct unfreezable_bh_struct chgint_bh;
	struct unfreezable_bh_struct boost_op_bh;
	struct unfreezable_bh_struct boost_worker_bh;

	struct delayed_work charging_work;
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
	struct fan54020_state state;
	int irq;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct pinctrl_state *pins_active;
	struct device_pm_platdata *pm_platdata;

	struct alarm chrgr_boost_en_timer;

};

static int fan54020_otg_notification_handler(struct notifier_block *nb,
		unsigned long event, void *data);

static struct fan54020_charger chrgr_data = {
	.model_name = "FAN54020",
	.manufacturer = "FAIRCHILD",

	.otg_nb = {
		.notifier_call = fan54020_otg_notification_handler,
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
		.status = FAN54020_STATUS_UNKNOWN,
		.vbus = -1,
		.cc = DEFAULT_CC,
		.max_cc = IOCHARGE_MAX_MA,
		.cv = DEFAULT_CV,
		.iterm = 0,
		.health = POWER_SUPPLY_HEALTH_UNKNOWN,
		.cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE,
		.charger_enabled = false,
		.charging_enabled = true, /* initially HZ mode is switched off*/
		.pok_b = 0,
		.ovp_flag = 0,
		.ovp_recov = 0,
		.t32s_timer_expired = 0,
		.vbus_fault = 0,
		.treg_flag = 0,
		.ot_recov_flag = 0,
		.tsd_flag = 0,
	},
};

static int fan54020_configure_chip(
			struct fan54020_charger *chrgr, bool enable_charging);

static int fan54020_i2c_read_reg(
			struct i2c_client *client, u8 reg_addr, u8 *data);

static int fan54020_i2c_write_reg(
			struct i2c_client *client, u8 reg_addr, u8 data);

static int fan54020_enable_charging(
			struct fan54020_charger *chrgr, bool enable);

static struct charger_debug_data chrgr_dbg;

struct shadow_reg {
	const char *name;
	u8 value;
};

static struct shadow_reg shadow_registers[SHADOW_REGS_NR] = {
	{
		.name = "ic_info",
	}, {
		.name = "charge_ctrl1",
	}, {
		.name = "charge_ctrl2",
	}, {
		.name = "ibat",
	}, {
		.name = "voreg",
	}, {
		.name = "ibus",
	}, {
		.name = "interrupt",
	}, {
		.name = "status",
	}, {
		.name = "int_mask",
	}, {
		.name = "st_mask",
	},
};

static struct power_supply_throttle fan54020_dummy_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
	},
};

static char *fan54020_supplied_to[] = {
		"battery",
};

static enum power_supply_property fan54020_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_HEALTH
};

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
	struct fan54020_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
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
	struct fan54020_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
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
		pr_info("fake vbus removal sent\n");

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

static struct device_attribute fan54020_fake_vbus_attr = {
	.attr = {
		.name = "fake_vbus_event",
		.mode = S_IRUSR | S_IWUSR,
	},
	.show = fake_vbus_show,
	.store = fake_vbus_store,
};

/**
 * fan54020_setup_sysfs_attr	Sets up sysfs entries for fan54020 i2c device
 * @chrgr			[in] pointer to charger driver internal
 *				structure
 */
static void fan54020_setup_fake_vbus_sysfs_attr(struct fan54020_charger *chrgr)
{
	struct device *dev = &chrgr->client->dev;
	int err;

	err = device_create_file(dev, &fan54020_fake_vbus_attr);
	if (err)
		pr_err("Unable to create sysfs entry: '%s'\n",
				fan54020_fake_vbus_attr.attr.name);
}

#else

static inline void fan54020_setup_fake_vbus_sysfs_attr(
					struct fan54020_charger *chrgr)
{
	(void) chrgr;
}

#endif /*SYSFS_FAKE_VBUS_SUPPORT*/





#ifdef CONFIG_DEBUG_FS


static int dbg_evt_open(struct inode *inode, struct file *file)
{

	/* save private data (the address of test_mod) in file struct
	(will be used by read()) */
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t dbg_evt_read(
			struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{

	/* obtaining private data saved by open method */
	struct charger_debug_data *dbg_array =
			(struct charger_debug_data *) filp->private_data;

	unsigned long time_stamp_jiffies, time_stamp_s;
	enum charger_debug_event event;
	const char *event_str;
	u32 cnt, read_idx;
	int prm, prm2;
	ssize_t retval;

	int i, chars_count, total_chars = 0;

	char log_line[LOG_LINE_LENGTH];

	for (i = 0; i < LINES_PER_PAGE; i++) {

		chars_count = 0;

		spin_lock(&dbg_array->lock);
		cnt = dbg_array->count;

		if (!cnt) {
			spin_unlock(&dbg_array->lock);
			return total_chars;
		}

		read_idx = dbg_array->read_index;
		event = dbg_array->log_array[read_idx].event;
		event_str = dbg_array->log_array[read_idx].event_string;
		time_stamp_jiffies =
			dbg_array->log_array[read_idx].time_stamp_jiffies;

		prm = dbg_array->log_array[read_idx].param;
		prm2 = dbg_array->log_array[read_idx].param2;
		dbg_array->count--;
		dbg_array->read_index++;
		dbg_array->read_index &= (CHARGER_DEBUG_DATA_SIZE-1);


		spin_unlock(&dbg_array->lock);


		time_stamp_s = time_stamp_jiffies/HZ;
		chars_count += snprintf(
			log_line, LOG_LINE_LENGTH, "[%5lu.%3lu]: %s ",
			time_stamp_s, (time_stamp_jiffies - time_stamp_s*HZ),
								event_str);

		if (event == CHG_DBG_REG) {
			chars_count += snprintf(
				log_line + chars_count,
				LOG_LINE_LENGTH - chars_count,
				"addr=0x%x, val=0x%x\n", prm, prm2);

		} else if (event == CHG_DBG_I2C_READ_ERROR ||
				event == CHG_DBG_I2C_WRITE_ERROR) {

			chars_count += snprintf(
				log_line + chars_count,
				LOG_LINE_LENGTH - chars_count,
				"err=%d, at addr=0x%x\n", prm, prm2);

		} else if (event < CHG_DBG_LAST_NO_PARAM_EVENT) {

			chars_count += snprintf(
				log_line + chars_count,
				LOG_LINE_LENGTH - chars_count, "\n");
		} else {
			chars_count += snprintf(
				log_line + chars_count,
				LOG_LINE_LENGTH - chars_count, "val=%d\n", prm);
		}

		/* copy data from driver to user space buffer */
		if (copy_to_user(buf + total_chars, log_line, chars_count)) {
			retval = -EFAULT;
			goto out;
		}
		total_chars += chars_count;

	}

	retval = total_chars;

out:
	return retval;
}

static const struct file_operations fan54020_evt_dbg_fops = {
	.open = dbg_evt_open,
	.read = dbg_evt_read,
};



static int fan54020_dbg_regs_show(struct seq_file *m, void *data)
{
	int i, ret = 0;
	struct fan54020_charger *chrgr = (struct fan54020_charger *) m->private;
	u8 charge_ctrl1_reg, tmr_rst_reg, safety_reg, monitor_reg,
						state_reg, tmr_ctrl_reg;

	unsigned long timestamp_jiffies, timestamp_s;

	down(&chrgr->prop_lock);

	timestamp_jiffies = jiffies - INITIAL_JIFFIES;
	timestamp_s = timestamp_jiffies/HZ;
	seq_printf(m, "[%5lu.%3lu] :\n", timestamp_s,
				(timestamp_jiffies - timestamp_s*HZ));

	ret = fan54020_i2c_read_reg(chrgr->client, REG_CHARGE_CTRL1,
							&charge_ctrl1_reg);
	if (ret != 0)
		goto out;

	shadow_registers[REG_CHARGE_CTRL1].value = charge_ctrl1_reg;

	for (i = 0; i < SHADOW_REGS_NR; ++i) {
		seq_printf(m, "%s = 0x%x\n", shadow_registers[i].name,
						shadow_registers[i].value);
	}

	ret = fan54020_i2c_read_reg(chrgr->client, REG_TMR_RST, &tmr_rst_reg);
	if (ret != 0)
		goto out;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_SAFETY, &safety_reg);
	if (ret != 0)
		goto out;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_MONITOR, &monitor_reg);
	if (ret != 0)
		goto out;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_STATE, &state_reg);
	if (ret != 0)
		goto out;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_TMR_CTRL, &tmr_ctrl_reg);
	if (ret != 0)
		goto out;

	seq_printf(m, "tmr_rst = 0x%x\n", tmr_rst_reg);
	seq_printf(m, "safety = 0x%x\n", safety_reg);
	seq_printf(m, "monitor = 0x%x\n", monitor_reg);
	seq_printf(m, "state = 0x%x\n", state_reg);
	seq_printf(m, "tmr_ctrl = 0x%x\n\n\n", tmr_ctrl_reg);

out:
	up(&chrgr->prop_lock);

	return ret;
}

static int fan54020_dbg_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, fan54020_dbg_regs_show, inode->i_private);
}

static const struct file_operations fan54020_dbg_regs_fops = {
	.open = fan54020_dbg_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int fan54020_dbg_state_show(struct seq_file *m, void *data)
{
	struct fan54020_charger *chrgr = (struct fan54020_charger *) m->private;
	unsigned long timestamp_jiffies, timestamp_s;

	(void)data;

	timestamp_jiffies = jiffies - INITIAL_JIFFIES;
	timestamp_s = timestamp_jiffies/HZ;
	seq_printf(m, "[%5lu.%3lu] :\n", timestamp_s,
				(timestamp_jiffies - timestamp_s*HZ));

	seq_printf(m, "vbus = %d\n", chrgr->state.vbus);
	seq_printf(m, "cc = %d\n", chrgr->state.cc);
	seq_printf(m, "max_cc = %d\n", chrgr->state.max_cc);
	seq_printf(m, "cv = %d\n", chrgr->state.cv);
	seq_printf(m, "iterm = %d\n", chrgr->state.iterm);
	seq_printf(m, "inlmt = %d\n", chrgr->state.inlmt);
	seq_printf(m, "health = %d\n", chrgr->state.health);
	seq_printf(m, "cable_type = %d\n", chrgr->state.cable_type);
	seq_printf(m, "charger_enabled = %d\n", chrgr->state.charger_enabled);
	seq_printf(m, "charging_enabled = %d\n",
				chrgr->state.charging_enabled);
	seq_printf(m, "to_enable_boost = %d\n\n",
				chrgr->state.to_enable_boost);
	seq_printf(m, "boost_enabled = %d\n\n",
				chrgr->state.boost_enabled);

	seq_printf(m, "pok_b = %u\n", chrgr->state.pok_b);
	seq_printf(m, "ovp_flag = %u\n", chrgr->state.ovp_flag);
	seq_printf(m, "ovp_recov = %u\n", chrgr->state.ovp_recov);
	seq_printf(m, "t32s_timer_expired = %u\n",
				chrgr->state.t32s_timer_expired);

	seq_printf(m, "vbus_fault = %u\n", chrgr->state.vbus_fault);
	seq_printf(m, "treg_flag = %u\n", chrgr->state.treg_flag);
	seq_printf(m, "ot_recov_flag = %u\n\n", chrgr->state.ot_recov_flag);
	seq_printf(m, "bat_uv = %u\n", chrgr->state.bat_uv);
	seq_printf(m, "boost_ov = %u\n\n", chrgr->state.boost_ov);

	return 0;
}

static int fan54020_dbg_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, fan54020_dbg_state_show, inode->i_private);
}

static const struct file_operations fan54020_dbg_state_fops = {
	.open = fan54020_dbg_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


/**
 * fan54020_setup_debugfs - sets up debugfs entries for fan54020 charger driver
 * @chrgr		[in] pointer to charger driver internal structure
 * @dbg_data		[in] pointer to debug array containing events logs.
 */
static void fan54020_setup_debugfs(struct fan54020_charger *chrgr,
				struct charger_debug_data *dbg_data)
{
	struct dentry *dbgfs_entry;

	dbgfs_entry = debugfs_create_dir(FAN54020_NAME, NULL);
	if (!dbgfs_entry)
		return;

	chrgr->debugfs_root_dir = dbgfs_entry;

	(void)debugfs_create_file(EVENTS_LOG_FILENAME, S_IRUGO,
			dbgfs_entry, dbg_data, &fan54020_evt_dbg_fops);

	(void)debugfs_create_file(DBG_REGS_FILENAME, S_IRUGO,
			dbgfs_entry, chrgr, &fan54020_dbg_regs_fops);

	(void)debugfs_create_file(DBG_STATE_FILENAME, S_IRUGO,
			dbgfs_entry, chrgr, &fan54020_dbg_state_fops);

	return;
}

/**
 * fan54020_remove_debugfs_dir	recursively removes debugfs root directory
 *				of FAN54020 charger driver
 * @chrgr			[in] pointer to charger driver's
 *				internal structure
 */
static void fan54020_remove_debugfs_dir(struct fan54020_charger *chrgr)
{
	debugfs_remove_recursive(chrgr->debugfs_root_dir);
	return;
}

#else

static inline void fan54020_setup_debugfs(struct fan54020_charger *chrgr,
				struct charger_debug_data *dbg_data)
{

}

static inline void fan54020_remove_debugfs_dir(struct fan54020_charger *chrgr)
{

}

#endif /* CONFIG_DEBUG_FS  */

/**
 * fan54020_i2c_read_reg - function for reading fan54020 registers
 * @client		[in] pointer i2c client structure
 * @reg_addr		[in] register address
 * @data		[out] value read from register
 *
 * Returns '0' on success
 */
static int fan54020_i2c_read_reg(struct i2c_client *client, u8 reg_addr,
								u8 *data)
{
	int ret, cnt = MAX_NR_OF_I2C_RETRIES;
	struct i2c_msg msgs[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 1,
			.buf    = &reg_addr,
		},
		{
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 1,
			.buf    = data,
		},
	};

	do {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2) {
			if (reg_addr < SHADOW_REGS_NR)
				shadow_registers[reg_addr].value = *data;
			CHARGER_DEBUG_READ_REG(chrgr_dbg, reg_addr, *data);
			return 0;
		}
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_READ_ERROR, ret,
								reg_addr);
	} while (cnt--);

	return ret;
}

/**
 * fan54020_i2c_write_reg - function for writing to fan54020 registers
 * @client		[in] pointer i2c client structure
 * @reg_addr		[in] register address
 * @data		[in] value to be written to register
 *
 * Returns '0' on success.
 */
static int fan54020_i2c_write_reg(struct i2c_client *client, u8 reg_addr,
								u8 data)
{
	int ret, cnt = MAX_NR_OF_I2C_RETRIES;
	u8 out_buf[2];
	struct i2c_msg msgs[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = out_buf,
		},
	};
	out_buf[0] = reg_addr;
	out_buf[1] = data;

	do {
		ret = i2c_transfer(client->adapter, msgs, 1);
		CHARGER_DEBUG_WRITE_REG(chrgr_dbg, reg_addr, data);
		if (ret == 1) {
			if (reg_addr < SHADOW_REGS_NR)
				shadow_registers[reg_addr].value = data;
			return 0;
		}
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_WRITE_ERROR, ret,
								reg_addr);
	} while (cnt--);

	return ret;
}

static int fan54020_trigger_wtd(struct fan54020_charger *chrgr)
{
	int ret;

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TRIGGERING_WTD, 0, 0);
	ret = fan54020_i2c_write_reg(chrgr->client, REG_TMR_RST,
							(1<<TMR_RST_O));
	return ret;
}


static int fan54020_set_voreg(struct fan54020_charger *chrgr,
				int volt_to_set, int *volt_set, bool propagate)
{
	int ret, volt_to_set_mv;
	u8 volt_regval, readback_val;

	volt_to_set_mv = fit_in_range(volt_to_set, VOREG_MIN_MV, VOREG_MAX_MV);

	volt_regval = (u8)((volt_to_set_mv - VOREG_MIN_MV) / VOREG_STEP_MV);

	if (propagate) {
		ret = fan54020_i2c_write_reg(chrgr->client, REG_VOREG,
								volt_regval);
		if (ret != 0)
			return ret;

		/* Reading back the register value becuase it could ignore our
		setting as a result of being limited by the value of
		SAFETY register  */
		ret = fan54020_i2c_read_reg(chrgr->client, REG_VOREG,
								&readback_val);
		if (ret != 0)
			return ret;

		if (volt_regval != readback_val) {
			pr_err("I2C write() error! Register VOREG still contains the old value\n");
			return -EIO;
		}

	}

	*volt_set = ((int)volt_regval * VOREG_STEP_MV) + VOREG_MIN_MV;

	return 0;
}

static int fan54020_set_iocharge(
			struct fan54020_charger *chrgr, int curr_to_set,
						int *curr_set, bool propagate)
{
	int ret, current_to_set_ma;
	u8 regval, cur_regval, readback_val;

	if (curr_to_set < IOCHARGE_MIN_MA) {
		if (chrgr->state.charging_enabled && propagate) {
			ret = fan54020_enable_charging(chrgr, false);
			if (ret != 0)
				return ret;
			chrgr->state.charging_enabled = 0;
		}
		*curr_set = 0;
		return 0;
	}

	current_to_set_ma = fit_in_range(curr_to_set, IOCHARGE_MIN_MA,
							IOCHARGE_MAX_MA);

	if (current_to_set_ma < IOCHARGE_STEP_START_MA) {
		regval = 0;
	} else {
		regval = IOCHARGE_STEP_START_REGVAL +
		(u8)((current_to_set_ma - IOCHARGE_STEP_START_MA) /
							IOCHARGE_STEP_MA);
	}

	if (propagate) {
		ret = fan54020_i2c_read_reg(chrgr->client, REG_IBAT,
								&cur_regval);
		if (ret != 0)
			return ret;

		cur_regval &= ~(IOCHARGE_M << IOCHARGE_O);
		cur_regval |= regval << IOCHARGE_O;

		ret = fan54020_i2c_write_reg(chrgr->client, REG_IBAT,
								cur_regval);
		if (ret != 0)
			return ret;

		/* Reading back the register value becuase it could ignore our
		setting as a result of being limited by the value of
		SAFETY register  */
		ret = fan54020_i2c_read_reg(chrgr->client, REG_IBAT,
							&readback_val);
		if (ret != 0)
			return ret;

		if (readback_val != cur_regval) {
			pr_err("I2C write() error! Register IBAT still contains the old value\n");
			return -EIO;
		}


		regval = (readback_val >> IOCHARGE_O) & IOCHARGE_M;
	}

	*curr_set = (regval == 0) ? IOCHARGE_MIN_MA :
	((regval - IOCHARGE_STEP_START_REGVAL) * IOCHARGE_STEP_MA
						+ IOCHARGE_STEP_START_MA);

	return 0;
}

static int fan54020_set_ibus_limit(struct fan54020_charger *chrgr,
						int ilim_to_set, int *ilim_set)
{
	int ret, current_to_set_ma;
	u8 regval, cur_regval, readback_val;

	if (ilim_to_set < IBUS_MIN_LIMIT_MA) {
		if (chrgr->state.charging_enabled) {
			ret = fan54020_enable_charging(chrgr, false);
			if (ret != 0)
				return ret;
			chrgr->state.charging_enabled = 0;
		}
		*ilim_set = 0;
		return 0;
	}

	if (ilim_to_set <= IBUS_MAX_LIMIT_MA) {
		current_to_set_ma = fit_in_range(ilim_to_set,
					IBUS_MIN_LIMIT_MA, IBUS_MAX_LIMIT_MA);

		regval = (u8)((current_to_set_ma - IBUS_MIN_LIMIT_MA) /
							IBUS_LIMIT_STEP_MA);
	} else {
		regval = IBUS_NO_LIMIT;
	}

	ret = fan54020_i2c_read_reg(chrgr->client, REG_IBUS, &cur_regval);
	if (ret != 0)
		return ret;

	cur_regval &= ~(IBUS_M << IBUS_O);
	cur_regval |= (regval << IBUS_O);

	ret = fan54020_i2c_write_reg(chrgr->client, REG_IBUS, cur_regval);
	if (ret != 0)
		return ret;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_IBUS, &readback_val);
	if (ret != 0)
		return ret;

	if (readback_val != cur_regval) {
		pr_err("I2C write() error! Register IBUS still contains the old value\n");
		return -EIO;
	}


	*ilim_set = (regval == IBUS_NO_LIMIT) ? ilim_to_set :
			(regval * IBUS_LIMIT_STEP_MA + IBUS_MIN_LIMIT_MA);

	return 0;
}

static inline bool fan54020_is_online(struct fan54020_charger *chrgr,
						struct power_supply *psy)
{
	if (!(chrgr->state.health == POWER_SUPPLY_HEALTH_GOOD) ||
		!chrgr->state.charger_enabled)
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

static void fan54020_charging_worker(struct work_struct *work)
{
	int ret;
	struct fan54020_charger *chrgr =
		container_of(work, struct fan54020_charger, charging_work.work);


	if (!time_after(jiffies, chrgr->ack_time + (60*HZ))) {
		down(&chrgr->prop_lock);
		ret = fan54020_trigger_wtd(chrgr);
		up(&chrgr->prop_lock);
		if (ret != 0)
			return;
	}

	power_supply_changed(chrgr->current_psy);
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TRIG_POWER_SUPPLY_CHARGER, 0, 0);

	schedule_delayed_work(&chrgr->charging_work, CHRGR_WORK_DELAY);

	return;
}

static int fan54020_enable_charging(struct fan54020_charger *chrgr, bool enable)
{
	int ret;
	u8 chr_reg;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_CHARGE_CTRL1, &chr_reg);
	if (ret != 0)
		return ret;

	chr_reg &= ~(1 << HZ_MODE_O);
	chr_reg |= ((enable) ? (0 << HZ_MODE_O) : (1 << HZ_MODE_O));

	ret = fan54020_i2c_write_reg(chrgr->client, REG_CHARGE_CTRL1, chr_reg);
	if (ret != 0)
		return ret;

	if (enable) {
		/*
		 * Obtain Wake Lock to prevent suspend during charging
		 * because the charger watchdog needs to be cont. retriggered.
		 */
		wake_lock(&chrgr->suspend_lock);
		schedule_delayed_work(&chrgr->charging_work, CHRGR_WORK_DELAY);
	} else {
		/* Release Wake Lock to allow suspend during discharging */
		wake_unlock(&chrgr->suspend_lock);
		cancel_delayed_work(&chrgr->charging_work);
	}
	return 0;
}


static int fan54020_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	int value_to_set, value_set = 0, ret = 0;
	struct fan54020_charger *chrgr = &chrgr_data;

	bool call_psy_changed = false;

	down(&chrgr->prop_lock);

	if (chrgr->state.status == FAN54020_STATUS_FAULT) {
		ret = -EFAULT;
		CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_SET_PROPERTY_ERROR, ret, 0);

		CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_DRIVER_IN_FAULT_STATE, 0, 0);
		up(&chrgr->prop_lock);
		return ret;
	}

	switch (psp) {

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		CHARGER_DEBUG_REL(
			chrgr_dbg, CHG_DBG_SET_PROP_CABLE_TYPE, val->intval, 0);

		value_set = val->intval;
		if (chrgr->state.cable_type == val->intval)
			break;
		chrgr->state.cable_type = val->intval;


		if ((chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
			(chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING))

			chrgr->current_psy = &chrgr->usb_psy;
		else
			chrgr->current_psy = &chrgr->ac_psy;

		call_psy_changed = true;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_CURRENT,
								val->intval, 0);

		value_to_set = fit_in_range(val->intval, 0,
						chrgr->state.max_cc);

		fan54020_set_iocharge(chrgr, value_to_set, &value_set, false);
		if (value_set == chrgr->state.cc)
			break;
		ret = fan54020_set_iocharge(chrgr, value_to_set,
							&value_set, true);

		if (ret) {
			chrgr->state.status = FAN54020_STATUS_FAULT;
			chrgr->state.health =
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}

		chrgr->state.cc = value_set;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_VOLTAGE,
								val->intval, 0);

		fan54020_set_voreg(chrgr, val->intval, &value_set, false);
		if (value_set == chrgr->state.cv)
			break;
		ret = fan54020_set_voreg(chrgr, val->intval, &value_set, true);
		if (ret) {
			chrgr->state.status = FAN54020_STATUS_FAULT;
			chrgr->state.health =
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}

		chrgr->state.cv = value_set;
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_ENABLE_CHARGER,
								val->intval, 0);

		chrgr->state.charger_enabled = val->intval;
		value_set = val->intval;
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_ENABLE_CHARGING,
								val->intval, 0);

		if (val->intval == chrgr->state.charging_enabled) {
			value_set = val->intval;
			break;
		}
		ret = fan54020_enable_charging(chrgr, val->intval);
		if (!ret) {
			chrgr->state.charging_enabled = val->intval;
			value_set = val->intval;
			call_psy_changed = true;
		}
		break;

	case POWER_SUPPLY_PROP_INLMT:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_INLMT,
							val->intval, 0);

		value_to_set = val->intval;
		ret = fan54020_set_ibus_limit(chrgr, value_to_set, &value_set);
		if (ret) {
			chrgr->state.status = FAN54020_STATUS_FAULT;
			chrgr->state.health =
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}

		chrgr->state.inlmt = value_set;
		if (value_set > IBUS_MAX_LIMIT_MA)
			chrgr->state.max_cc =
				fit_in_range(value_set, 0, IOCHARGE_MAX_MA);
		break;

	case POWER_SUPPLY_PROP_CONTINUE_CHARGING:
		CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_SET_PROP_CONTINUE_CHARGING,
								val->intval, 0);

		chrgr->ack_time = jiffies;
		value_set = val->intval;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_TERM_CUR,
								val->intval, 0);
		chrgr->state.iterm = value_set;
		break;

	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_MIN_TEMP,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_MAX_TEMP,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		chrgr->state.health = val->intval;
		call_psy_changed = true;
		break;

	default:
		break;
	};

	if (!ret)
		CHARGER_DEBUG_REL(
			chrgr_dbg, CHG_DBG_SET_PROPERTY_VALUE_SET,
							value_set, 0);
	else
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROPERTY_ERROR,
								ret, 0);

	up(&chrgr->prop_lock);

	if (call_psy_changed)
		power_supply_changed(psy);

	return ret;
}

static int fan54020_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct fan54020_charger *chrgr = &chrgr_data;

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

		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_PRESENT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = fan54020_is_online(chrgr, psy);
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_ONLINE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chrgr->state.health;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_HEALTH,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_USB;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_TYPE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		val->intval = chrgr->state.cable_type;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_CABLE_TYPE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		val->intval = chrgr->state.charger_enabled;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_ENABLE_CHARGER,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		val->intval = chrgr->state.charging_enabled;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_ENABLE_CHARGING,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = 0;
		CHARGER_DEBUG_DEV(
			chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_CONTROL_LIMIT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chrgr->model_name;
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = chrgr->manufacturer;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = chrgr->state.cc;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_CURRENT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = chrgr->state.cv;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_VOLTAGE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chrgr->state.max_cc;
		CHARGER_DEBUG_DEV(
			chrgr_dbg, CHG_DBG_GET_PROP_MAX_CHARGE_CURRENT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_INLMT:
		val->intval = chrgr->state.inlmt;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_INLMT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		val->intval = chrgr->state.iterm;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_TERM_CUR,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_PRIORITY:
		val->intval = 0;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_PRIORITY,
								val->intval, 0);
		break;


	default:
		ret = -EINVAL;
		break;
	};

	up(&chrgr->prop_lock);

	return ret;
}

/*
* Boost enable timer call back
*
* @alrm [in] parameter passed from timer. Not used in this case.
* @t    [in]    parameter passed from timer. Not used in this case.
*
* Returns:	   Whether alarm timer should restart.
*/
static enum alarmtimer_restart charger_boost_en_timer_expired_cb(
		struct alarm *alrm, ktime_t t)
{
	(void)alrm;

	/* Schedule the execution of watchdog clearing. */
	unfreezable_bh_schedule(&(chrgr_data.boost_worker_bh));

	return ALARMTIMER_NORESTART;
}

static void fan54020_boost_worker(struct work_struct *work)
{
	int ret;
	u8 charge_ctrl2;
	struct fan54020_charger *chrgr =
		container_of(work, struct fan54020_charger,
		boost_worker_bh.work);

	down(&chrgr->prop_lock);

	ret = fan54020_i2c_read_reg(chrgr->client,
			REG_CHARGE_CTRL2, &charge_ctrl2);
	charge_ctrl2 &= (1 << BOOST_EN);

	if ((!ret) && charge_ctrl2 && chrgr->state.boost_enabled) {
		ret = fan54020_trigger_wtd(chrgr);

		/* Start the timer again if boost is enabled and
		 * i2c write successful. */
		if (!ret)
			SET_ALARM_TIMER(&(chrgr->chrgr_boost_en_timer),
				CHARGER_WATCHDOG_TIMER_SECS);
		else
			pr_err("%s: I2C write failed. boost timer not enabled.\n",
					__func__);
	}

	up(&chrgr->prop_lock);
	return;
}

static void fan54020_set_boost(struct work_struct *work)
{
	struct fan54020_charger *chrgr =
		container_of(work, struct fan54020_charger,
					boost_op_bh.work);
	int on = chrgr->state.to_enable_boost;
	int ret = 0;
	u8 chr_reg;

	down(&chrgr->prop_lock);

	/* Clear state flags */
	chrgr->state.boost_ov = 0;
	chrgr->state.bat_uv = 0;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_CHARGE_CTRL2, &chr_reg);
	if (ret)
		goto exit_boost;

	if (on) {
		/* Enable boost regulator */
		chr_reg |= (1 << BOOST_EN);
		ret = fan54020_i2c_write_reg(chrgr->client,
				REG_CHARGE_CTRL2, chr_reg);
		if (ret)
			goto exit_boost;

		/* Boost startup time is 2 ms max */
		mdelay(2);

		/* Ensure Boost is in regulation */
		ret = fan54020_i2c_read_reg(chrgr->client,
				REG_CHARGE_CTRL2, &chr_reg);
		if (ret)
			goto exit_boost;

		if (!(chr_reg & (1 << BOOST_UP))) {
			/*
			 * In case of BOOST fault, BOOST_EN bit is automatically
			 * cleared
			 */
			pr_err("%s: boost mode didn't go in regulation\n",
					__func__);
			ret = -EINVAL;
			goto exit_boost;
		}

		/* Enable boost mode flag */
		chrgr->state.boost_enabled = 1;

		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_BOOST_ENABLED, 0, 0);

		SET_ALARM_TIMER(&(chrgr->chrgr_boost_en_timer),
						CHARGER_WATCHDOG_TIMER_SECS);

	} else {
		(void)alarm_cancel(&(chrgr_data.chrgr_boost_en_timer));

		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_BOOST_DISABLED, 0, 0);

		/* Disable boost mode flag */
		chrgr->state.boost_enabled = 0;

		/* Disable boost regulator */
		chr_reg &= ~(1 << BOOST_EN);
		ret = fan54020_i2c_write_reg(chrgr->client,
				REG_CHARGE_CTRL2, chr_reg);
		if (ret)
			pr_err("%s: fail to disable boost mode\n", __func__);
	}

exit_boost:
	if (on && ret)
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
					INTEL_USB_DRV_VBUS_ERR, NULL);
	up(&chrgr->prop_lock);
	return;
}

/**
 * fan54020_chgint_cb_work_func	function executed by
 *				fan54020_charger::chgint_cb_work work
 * @work			[in] pointer to associated 'work' structure
 */
static void fan54020_chgint_cb_work_func(struct work_struct *work)
{
	u8 charge_ctrl1_reg, interrupt_reg, vbus_stat_reg, state_reg;
	int ret, vbus_state_prev, health_prev, state;
	struct fan54020_charger *chrgr =
		container_of(work, struct fan54020_charger,
					chgint_bh.work);

	down(&chrgr->prop_lock);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_CHGINT_CB, 0, 0);

	if (chrgr->state.status == FAN54020_STATUS_FAULT) {
		CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_DRIVER_IN_FAULT_STATE, 0, 0);
		up(&chrgr->prop_lock);
		return;
	}

	vbus_state_prev = chrgr->state.vbus;
	health_prev = chrgr->state.health;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_CHARGE_CTRL1,
							&charge_ctrl1_reg);
	if (ret != 0)
		goto fail;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_INT, &interrupt_reg);
	if (ret != 0)
		goto fail;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_STATUS, &vbus_stat_reg);
	if (ret != 0)
		goto fail;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_STATE, &state_reg);
	if (ret != 0)
		goto fail;

	/* Checking for OVP_FLAG or OVP_RECOV occurrance */

	/* Common flags between charger mode and boost mode */
	chrgr->state.tsd_flag = (interrupt_reg & (1<<TSD_FLAG_O)) ?
							TSD_OCCURRED : 0;

	chrgr->state.ovp_flag = (interrupt_reg & (1<<OVP_FLAG_O)) ?
							OVP_OCCURRED : 0;

	chrgr->state.t32s_timer_expired = (interrupt_reg & (1<<TC_TO_O)) ?
							T32_TO_OCCURRED : 0;

	/* Handle interrupts specific to boost mode*/
	if (chrgr->state.boost_enabled) {
		chrgr->state.boost_ov = (interrupt_reg & (1<<BOOSTOV_O)) ?
							BOOSTOV_OCCURED : 0;
		chrgr->state.bat_uv = (interrupt_reg & (1<<BAT_UV_O)) ?
							BATUV_OCCURED : 0;

		if (chrgr->state.boost_ov || chrgr->state.bat_uv
			|| chrgr->state.tsd_flag || chrgr->state.ovp_flag ||
			 chrgr->state.t32s_timer_expired) {
			/*
			 * In case of BOOST fault, BOOST_EN bit is automatically
			 * cleared. Only need to clear boost enabled sw flag and
			 * stop the booster workqueue
			 */
			atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
					INTEL_USB_DRV_VBUS_ERR, NULL);

			wake_unlock(&chrgr->suspend_lock);
			if (chrgr->state.boost_ov)
				pr_err("%s: boost mode overcurrent detected\n",
						__func__);
			if (chrgr->state.bat_uv)
				pr_err("%s: boost mode under voltage detected\n",
						__func__);

			goto fail;
		}
	}

	chrgr->state.ovp_recov = (interrupt_reg & (1<<OVP_RECOV_O)) ?
							OVP_RECOV_OCCURRED : 0;


	chrgr->state.treg_flag = (interrupt_reg & (1<<TREG_FLAG_O)) ?
							TREG_IS_ON : 0;

	chrgr->state.ot_recov_flag = (interrupt_reg & (1<<OT_RECOV_O)) ?
							OT_RECOV_OCCURRED : 0;

	/* Checking for POK_B status change */
	chrgr->state.pok_b = (vbus_stat_reg & (1<<POK_B_O)) ?
						POK_B_INVAL : POK_B_VALID;
	chrgr->state.vbus = (chrgr->state.pok_b == POK_B_VALID) ?
							VBUS_ON : VBUS_OFF;

	state = ((state_reg >> ST_CODE_O) & ST_CODE_M);
	chrgr->state.vbus_fault =
			(state >= ST_VBUS_FAULT && state < ST_VBUS_FAULT2) ?
								VBUS_FAULT : 0;

	if (chrgr->state.treg_flag)
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TREG_IS_ON, 0, 0);

	if (chrgr->state.ot_recov_flag)
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_OT_RECOVERY, 0, 0);

	if (chrgr->state.t32s_timer_expired) {
		chrgr->state.health = POWER_SUPPLY_HEALTH_DEAD;
		chrgr->state.status = FAN54020_STATUS_FAULT;
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_T32_TIMER_EXPIRED, 0, 0);
		power_supply_changed(chrgr->current_psy);
		goto fail;
	}

	if (chrgr->state.health != POWER_SUPPLY_HEALTH_DEAD) {
		chrgr->state.health = (chrgr->state.vbus == VBUS_ON) ?
			POWER_SUPPLY_HEALTH_GOOD : POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (chrgr->state.vbus_fault) {
		chrgr->state.health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_VBUS_FAULT, 0, 0);
	}

	if (chrgr->state.ovp_flag) {
		chrgr->state.health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		pr_err("VBUS Over-Voltage Shutdown!");
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_VBUS_OVP, 0, 0);
	}

	if (chrgr->state.tsd_flag) {
		chrgr->state.health = POWER_SUPPLY_HEALTH_OVERHEAT;
		pr_err("Chip Over-temperature Shutdown!");
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TSD_IS_ON, 0, 0);
	}

	if (chrgr->state.ovp_recov) {
		pr_info("VBUS OVP Recovery occurred!");
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_VBUS_OVP_RECOV, 0, 0);
	}

	if (health_prev != chrgr->state.health &&
		chrgr->state.health != POWER_SUPPLY_HEALTH_GOOD &&
		 chrgr->state.health != POWER_SUPPLY_HEALTH_UNKNOWN)
				power_supply_changed(chrgr->current_psy);


	/* when vbus is disconnected all registers are reseted (known HW bug)
	therefore need to reconfigure the chip */
	if (chrgr->state.vbus == VBUS_OFF && vbus_state_prev == VBUS_ON) {
		chrgr->state.cc = DEFAULT_CC;
		chrgr->state.cv = DEFAULT_CV;
		ret = fan54020_configure_chip(
					chrgr, chrgr->state.charging_enabled);
		if (ret != 0)
			goto fail;
	}

	/* If vbus status changed, then notify USB OTG */
	if (chrgr->state.vbus != vbus_state_prev) {

		if (chrgr->otg_handle) {
			atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
					USB_EVENT_VBUS, &chrgr->state.vbus);

			CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_VBUS, chrgr->state.vbus, 0);
		}
	}

fail:
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

	INIT_WORK(&bh->work, work_func);

	wake_lock_init(&bh->evt_wakelock,
			WAKE_LOCK_SUSPEND,
			wakelock_name);

	return 0;
}

static void unfreezable_bh_destroy(struct unfreezable_bh_struct *bh)
{
	cancel_work_sync(&bh->work);

	destroy_workqueue(bh->wq);

	wake_lock_destroy(&bh->evt_wakelock);
}

static void unfreezable_bh_schedule(struct unfreezable_bh_struct *bh)
{
	spin_lock(&bh->lock);

	if (!bh->in_suspend) {
		queue_work(bh->wq, &bh->work);

		wake_lock_timeout(&bh->evt_wakelock, EVT_WAKELOCK_TIMEOUT);
	} else {
		bh->pending_evt = true;
	}

	spin_unlock(&bh->lock);
}

static void unfreezable_bh_resume(struct unfreezable_bh_struct *bh)
{
	unsigned long flags;

	spin_lock_irqsave(&bh->lock, flags);

	bh->in_suspend = false;
	if (bh->pending_evt) {
		bh->pending_evt = false;

		queue_work(bh->wq, &bh->work);

		wake_lock_timeout(&bh->evt_wakelock, EVT_WAKELOCK_TIMEOUT);
	}

	spin_unlock_irqrestore(&bh->lock, flags);
}

static void unfreezable_bh_suspend(struct unfreezable_bh_struct *bh)
{
	unsigned long flags;

	spin_lock_irqsave(&bh->lock, flags);
	bh->in_suspend = true;
	spin_unlock_irqrestore(&bh->lock, flags);
}

static irqreturn_t fan54020_charger_chgint_cb(int irq, void *dev)
{
	struct fan54020_charger *chrgr = dev;

	unfreezable_bh_schedule(&chrgr->chgint_bh);

	return IRQ_HANDLED;
}

/**
 * fan54020_configure_pmu_irq - function configuring PMU's Charger status IRQ
 * @chrgr		[in] pointer to charger driver internal structure
 */
static int fan54020_configure_pmu_irq(struct fan54020_charger *chrgr)
{
	int ret;

	/* register callback with PMU for CHGINT irq */
	ret = request_irq(chrgr->irq,
		fan54020_charger_chgint_cb,
			IRQF_NO_SUSPEND, FAN54020_NAME, chrgr);
	if (ret != 0) {
		pr_err("Failed to register @PMU for GHGINT irq! ret=%d", ret);
		return ret;
	}

	return 0;
}

/**
 * fan54020_configure_pmu_regs -function configuring PMU's Charger
 *				part registers
 * @chrgr			[in] pointer to charger driver's internal struct
 */
static int fan54020_configure_pmu_regs(struct fan54020_charger *chrgr)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	int ret;


	if (!chrgr->ididev)
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

	/* ChargerIRQEdge - CHARGER_CONTROL_CIEDG_FALLING */
	regval &= ~(CHARGER_CONTROL_CIEDG_M << CHARGER_CONTROL_CIEDG_O);
	regval |= (CHARGER_CONTROL_CIEDG_FALLING << CHARGER_CONTROL_CIEDG_O);

	/* ChargerIRQLevel - CHARGER_CONTROL_CILVL_LOW */
	regval &= ~(CHARGER_CONTROL_CILVL_M << CHARGER_CONTROL_CILVL_O);
	regval |= (CHARGER_CONTROL_CILVL_LOW << CHARGER_CONTROL_CILVL_O);

	/* ChargerIRQSensibility - CHARGER_CONTROL_CISENS_EDGE */
	regval &= ~(CHARGER_CONTROL_CISENS_M << CHARGER_CONTROL_CISENS_O);
	regval |= (CHARGER_CONTROL_CISENS_EDGE << CHARGER_CONTROL_CISENS_O);

	/* ChargerIRQEnable - CHARGER_CONTROL_CIEN_EN */
	regval &= ~(CHARGER_CONTROL_CIEN_M << CHARGER_CONTROL_CIEN_O);
	regval |= (CHARGER_CONTROL_CIEN_EN << CHARGER_CONTROL_CIEN_O);

	/* ChargerResetLevel - CHARGER_CONTROL_CHGLVL_LOW */
	regval &= ~(CHARGER_CONTROL_CHGLVL_M << CHARGER_CONTROL_CHGLVL_O);
	regval |= (CHARGER_CONTROL_CHGLVL_LOW << CHARGER_CONTROL_CHGLVL_O);

	/* ChargerIRQDebounce - CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE - as vbus
	signal is clear enough (not bouncing) */
	regval &= ~(CHARGER_CONTROL_CIDBT_M << CHARGER_CONTROL_CIDBT_O);
	regval |= (CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE <<
						CHARGER_CONTROL_CIDBT_O);

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

/**
 * fan54020_get_clr_wdt_expiry_flag -	func. gets WDT expiry status and clears
 *					expired status if it had expired
 * @chrgr		[in] pointer to charger driver internal structure
 */
static int fan54020_get_clr_wdt_expiry_flag(struct fan54020_charger *chrgr)
{
	u8 interrupt_reg = 0;
	int ret, wtd_expired;

	/* If the WDT interrupt was previously set,
	it will now be cleared upon reading */
	ret = fan54020_i2c_read_reg(chrgr->client, REG_INT, &interrupt_reg);
	if (ret != 0)
		return ret;

	wtd_expired = (interrupt_reg & (1<<TC_TO_O)) ? T32_TO_OCCURRED : 0;

	return wtd_expired;
}


/**
 * fan54020_configure_chip - function configuring FAN54020 chip registers
 * @chrgr		[in] pointer to charger driver internal structure
 * @enable_charging	[in] controls if charging should be anebled or disabled
 */
static int fan54020_configure_chip(struct fan54020_charger *chrgr,
							bool enable_charging)
{
	int ret;
	u8 chr_reg, readback_val;

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_CONFIGURE_CHIP, 0, 0);

	ret = fan54020_i2c_read_reg(chrgr->client, REG_CHARGE_CTRL2, &chr_reg);
	if (ret != 0)
		return ret;

	/*
	 * Set LDO_OFF bit to '0'
	 * 3.3V LDO is ON and biased from VBAT when:
	 * VBUS < VBAT && DBP pin is HIGH
	 * Setting valid for CHIP id 0x88
	 */
	chr_reg &= ~(1 << LDO_OFF_O);

	ret = fan54020_i2c_write_reg(chrgr->client, REG_CHARGE_CTRL2, chr_reg);
	if (ret != 0)
		return ret;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_CHARGE_CTRL1, &chr_reg);
	if (ret != 0)
		return ret;


	if (enable_charging == true)
		chr_reg &= ~(1 << HZ_MODE_O);
	else
		chr_reg |= (1 << HZ_MODE_O);


	ret = fan54020_i2c_write_reg(chrgr->client, REG_CHARGE_CTRL1, chr_reg);
	if (ret != 0)
		return ret;



	chr_reg = INT_MASK_ALL;
	chr_reg &= ~((1 << TSD_FLAG_O) | (1 << OVP_FLAG_O) |
			(1 << OVP_RECOV_O) | (1 << TC_TO_O) |
			(1 << DBP_TO_O) | (1 << TREG_FLAG_O) |
			(1 << OT_RECOV_O));

	ret = fan54020_i2c_write_reg(chrgr->client, REG_INT_MASK, chr_reg);
	if (ret != 0)
		return ret;


	ret = fan54020_i2c_read_reg(chrgr->client, REG_INT_MASK, &readback_val);
	if (ret != 0)
		return ret;




	ret = fan54020_i2c_read_reg(chrgr->client, REG_ST_MASK, &chr_reg);
	if (ret != 0)
		return ret;

	chr_reg = ST_MASK_ALL;
	chr_reg &= ~(1 << POK_B_O);

	ret = fan54020_i2c_write_reg(chrgr->client, REG_ST_MASK, chr_reg);
	if (ret != 0)
		return ret;

	ret = fan54020_i2c_read_reg(chrgr->client, REG_ST_MASK, &readback_val);
	if (ret != 0)
		return ret;



	return 0;
}

static int fan54020_set_pinctrl_state(struct i2c_client *client,
		struct pinctrl_state *state)
{
	struct fan54020_charger *chrgr = i2c_get_clientdata(client);
	int ret;

	ret = pinctrl_select_state(chrgr->pinctrl, state);
	if (ret != 0) {
		pr_err("failed to configure CHGRESET pin !\n");
		return -EIO;
	}

	return 0;
}

static int fan54020_otg_notification_handler(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct fan54020_charger *chrgr = &chrgr_data;

	switch (event) {
	case INTEL_USB_DRV_VBUS:
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

static int fan54020_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}


static int fan54020_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct usb_phy *otg_handle;
	struct fan54020_charger *chrgr = &chrgr_data;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	bool wtd_expired;
	u8 ic_info;
	int ret;

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

	chrgr->pins_active = pinctrl_lookup_state(chrgr->pinctrl,
						"active");
	if (IS_ERR(chrgr->pins_active))
		dev_err(dev, "could not get active pinstate\n");

	/* Register i2c clientdata before calling pinctrl */
	i2c_set_clientdata(client, chrgr);

	pr_info("%s\n", __func__);

	/* Read HW id */
	ret = fan54020_i2c_read_reg(client, REG_IC_INFO, &ic_info);
	if (ret != 0)
		return -ENODEV;

	/* Check if the HW is supported */
	if (((ic_info >> VENDOR_O) & VENDOR_M) != VENDOR ||
				((ic_info >> PN_O) & PN_M) != PN)
		return -ENODEV;

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
	chrgr->irq = irq_of_parse_and_map(np, 0);
	if (!chrgr->irq) {
		ret = -EINVAL;
		pr_err("can't get irq\n");
		goto remap_fail;
	}
	client->irq = chrgr->irq;

	/* Setup the PMU registers. The charger IC reset line
	is deasserted at this point. From this point onwards
	the charger IC will act upon the values stored in its
	registers instead of displaying default behaviour  */
	ret = fan54020_configure_pmu_regs(chrgr);
	if (ret != 0)
		goto pre_fail;

	/* If charger IC is later than version 1.0 then the safety
	registers need to be written before any others to prevent
	the default voltage and current safety limits from being assumed.
	The values written are the maximum values possible to allow freedom
	of setting all required settings during runtime. */
	if (((ic_info >> REV_O) & REV_M) != 0) {
		ret = fan54020_i2c_write_reg(client, REG_SAFETY,
				(FAN54020_CUR_LIMIT << ISAFE_O) |
				(FAN54020_VOLT_LIMIT << VSAFE_O));
		if (ret != 0) {
			ret = -ENODEV;
			goto pre_fail;
		}
	}

	/* Trigger charger watchdog to prevent expiry if the watchdog
	is about to expire due to a reset sequence having taken a long time.
	Triggering the watchdog if it has already expired has no effect,
	i.e. charging will not start again and the watchdog expiration flag in
	the interrupt register will not be cleared. */
	ret = fan54020_trigger_wtd(chrgr);
	if (ret != 0)
		goto pre_fail;

	INIT_DELAYED_WORK(&chrgr->charging_work, fan54020_charging_worker);

	if (unfreezable_bh_create(&chrgr->boost_worker_bh, "boost_wq",
			"fan54020_boost_trgr_lock", fan54020_boost_worker)) {
		ret = -ENOMEM;
		goto pre_fail;
	}

	sema_init(&chrgr->prop_lock, 1);

	/* Set up the wake lock to prevent suspend when charging. */
	wake_lock_init(&chrgr->suspend_lock,
			WAKE_LOCK_SUSPEND,
			"fan54020_wake_lock");

	/* It is OK to setup the wake lock here, because it is
	before the interrupt is later enabled with the call:
	fan54020_configure_pmu_irq(). */
	if (unfreezable_bh_create(&chrgr->chgint_bh, "chrgr_wq",
			"fan54020_evt_lock", fan54020_chgint_cb_work_func)) {
		ret = -ENOMEM;
		goto wq_creation_fail;
	}

	ret = fan54020_get_clr_wdt_expiry_flag(chrgr);
	if (ret < 0)
		goto fail;

	wtd_expired = (ret == T32_TO_OCCURRED) ? true : false;

	/* Update internal respresentation of charging status.
	chrgr->state.charging_enabled is preinitialised to true.
	It will be set false if the charger watchdog is found to be
	expired at boot. Charging will either be enabled or disabled
	according to the outcome of the first iteration of the
	PS Charger safety loop. */
	if (wtd_expired) {
		pr_err("Charging watchdog expiration detected!\n");
		chrgr->state.charging_enabled = false;
	}

	ret = fan54020_configure_chip(chrgr, chrgr->state.charging_enabled);
	if (ret != 0)
		goto fail;

	ret = fan54020_set_pinctrl_state(client, chrgr->pins_active);
	if (ret != 0)
		return ret;

	chrgr->usb_psy.name           = "usb_charger";
	chrgr->usb_psy.type           = POWER_SUPPLY_TYPE_USB;
	chrgr->usb_psy.properties     = fan54020_power_props;
	chrgr->usb_psy.num_properties = ARRAY_SIZE(fan54020_power_props);
	chrgr->usb_psy.get_property   = fan54020_charger_get_property;
	chrgr->usb_psy.set_property   = fan54020_charger_set_property;
	chrgr->usb_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->usb_psy.supplied_to = fan54020_supplied_to;
	chrgr->usb_psy.num_supplicants = ARRAY_SIZE(fan54020_supplied_to);
	chrgr->usb_psy.throttle_states = fan54020_dummy_throttle_states;
	chrgr->usb_psy.num_throttle_states =
				ARRAY_SIZE(fan54020_dummy_throttle_states);
	chrgr->usb_psy.property_is_writeable = fan54020_property_is_writeable;

	chrgr->current_psy = &chrgr->usb_psy;

	ret = power_supply_register(&client->dev, &chrgr->usb_psy);
	if (ret)
		goto fail;

	chrgr->ac_psy.name           = "ac_charger";
	chrgr->ac_psy.type           = POWER_SUPPLY_TYPE_MAINS;
	chrgr->ac_psy.properties     = fan54020_power_props;
	chrgr->ac_psy.num_properties = ARRAY_SIZE(fan54020_power_props);
	chrgr->ac_psy.get_property   = fan54020_charger_get_property;
	chrgr->ac_psy.set_property   = fan54020_charger_set_property;
	chrgr->ac_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->ac_psy.supplied_to = fan54020_supplied_to;
	chrgr->ac_psy.num_supplicants = ARRAY_SIZE(fan54020_supplied_to);
	chrgr->ac_psy.throttle_states = fan54020_dummy_throttle_states;
	chrgr->ac_psy.num_throttle_states =
				ARRAY_SIZE(fan54020_dummy_throttle_states);

	ret = power_supply_register(&client->dev, &chrgr->ac_psy);
	if (ret)
		goto fail_ac_registr;


	chrgr->ack_time = jiffies;

	if (chrgr->state.charging_enabled)
		schedule_delayed_work(&chrgr->charging_work, 0);

	ret = fan54020_configure_pmu_irq(chrgr);
	if (ret != 0)
		goto pmu_irq_fail;

	i2c_set_clientdata(client, chrgr);

	fan54020_setup_debugfs(chrgr, &chrgr_dbg);
	fan54020_setup_fake_vbus_sysfs_attr(chrgr);

	chrgr->state.status = FAN54020_STATUS_READY;

	/* Read the VBUS presence status for initial update by
	making a dummy interrupt bottom half invocation */
	queue_work(chrgr->chgint_bh.wq, &chrgr->chgint_bh.work);

	if (unfreezable_bh_create(&chrgr->boost_op_bh, "boost_op_wq",
			"fan54020_boost_lock", fan54020_set_boost)) {
		ret = -ENOMEM;
		goto pmu_irq_fail;
	}

	/* Initialise the alarm timer used for updating the
	 * t32s timer reset bit. */
	alarm_init(&(chrgr->chrgr_boost_en_timer),
			 ALARM_REALTIME,
			  charger_boost_en_timer_expired_cb);

	ret = usb_register_notifier(otg_handle, &chrgr->otg_nb);
	if (ret) {
		pr_err("ERROR!: registration for OTG notifications failed\n");
		goto boost_fail;
	}

	device_init_wakeup(&client->dev, true);

	return 0;

boost_fail:
	unfreezable_bh_destroy(&chrgr->boost_op_bh);
pmu_irq_fail:
	power_supply_unregister(&chrgr->ac_psy);
fail_ac_registr:
	power_supply_unregister(&chrgr->usb_psy);
fail:
	unfreezable_bh_destroy(&chrgr->chgint_bh);
wq_creation_fail:
	unfreezable_bh_destroy(&chrgr->boost_worker_bh);
pre_fail:
remap_fail:
	usb_put_phy(otg_handle);
	return ret;
}

static int __exit fan54020_i2c_remove(struct i2c_client *client)
{
	int ret = 0;
	struct fan54020_charger *chrgr = i2c_get_clientdata(client);

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

	ret = fan54020_set_pinctrl_state(client, chrgr->pins_inactive);
	if (ret != 0)
		return ret;
	fan54020_remove_debugfs_dir(&chrgr_data);

	(void)alarm_cancel(&(chrgr_data.chrgr_boost_en_timer));

	return 0;
}

static int fan54020_idi_probe(struct idi_peripheral_device *ididev,
					const struct idi_device_id *id)
{
	struct resource *res;
	struct fan54020_charger *chrgr = &chrgr_data;
	int ret = 0;

	spin_lock_init(&chrgr_dbg.lock);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_PROBE, 0, 0);
	pr_info("%s\n", __func__);

	res = idi_get_resource_byname(&ididev->resources,
				IORESOURCE_MEM, "registers");

	if (res == NULL) {
		pr_err("getting PMU's Charger registers resources failed!\n");
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

static int __exit fan54020_idi_remove(struct idi_peripheral_device *ididev)
{
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_REMOVE, 0, 0);
	pr_info("%s\n", __func__);

	return 0;
}

/**
 * fan54020_suspend() - Called when the system is attempting to suspend.
 * If charging is in progress EBUSY is returned to abort the suspend and
 * an error is logged, as the wake lock should prevent the situation.
 * @dev		[in] Pointer to the device.(not used)
 * returns	EBUSY if charging is ongoing, else 0
 */
static int fan54020_suspend(struct device *dev)
{
	/* Unused parameter */
	struct fan54020_charger *chrgr = &chrgr_data;
	(void)dev;

	if (chrgr_data.state.charging_enabled) {
		/* If charging is in progess, prevent suspend. */
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SUSPEND_ERROR, 0, 0);
		return -EBUSY;
	} else {
		/* Not charging - allow suspend. */
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SUSPEND_OK, 0, 0);
		if (device_may_wakeup(dev)) {
			pr_info("fan: enable wakeirq\n");
			enable_irq_wake(chrgr->irq);
		}
		unfreezable_bh_suspend(&chrgr_data.chgint_bh);
		unfreezable_bh_suspend(&chrgr_data.boost_op_bh);
		unfreezable_bh_suspend(&chrgr_data.boost_worker_bh);
		return 0;
	}
}

/**
 * fan54020_resume() - Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int fan54020_resume(struct device *dev)
{
	/* Unused parameter */
	struct fan54020_charger *chrgr = &chrgr_data;
	(void)dev;

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_RESUME, 0, 0);

	unfreezable_bh_resume(&chrgr_data.chgint_bh);
	unfreezable_bh_resume(&chrgr_data.boost_op_bh);
	unfreezable_bh_resume(&chrgr_data.boost_worker_bh);

	if (device_may_wakeup(dev)) {
		pr_info("fan: disable wakeirq\n");
		disable_irq_wake(chrgr->irq);
	}
	return 0;
}

const struct dev_pm_ops fan54020_pm = {
	.suspend = fan54020_suspend,
	.resume = fan54020_resume,
};


static const struct i2c_device_id fan54020_id[] = {
	{"fan54x_charger", 0}, { }
};

MODULE_DEVICE_TABLE(i2c, fan54020_id);

static struct i2c_driver fan54020_i2c_driver = {
	.probe          = fan54020_i2c_probe,
	.remove         = fan54020_i2c_remove,
	.id_table       = fan54020_id,
	.driver = {
		.name   = FAN54020_NAME,
		.owner  = THIS_MODULE,
		.pm = &fan54020_pm,
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

static struct idi_peripheral_driver fan54020_idi_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "fan54x_idi",
		.pm = NULL,
	},
	.p_type = IDI_CHG,
	.id_table = idi_ids,
	.probe  = fan54020_idi_probe,
	.remove = fan54020_idi_remove,
};


static int __init fan54020_init(void)
{
	int ret;

	ret = idi_register_peripheral_driver(&fan54020_idi_driver);
	if (ret)
		return ret;


	ret = i2c_add_driver(&fan54020_i2c_driver);
	if (ret)
		return ret;

	return 0;
}

late_initcall(fan54020_init);

static void __exit fan54020_exit(void)
{
	i2c_del_driver(&fan54020_i2c_driver);
}
module_exit(fan54020_exit);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Charger Driver for FAN54020 charger IC");
