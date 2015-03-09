/*
 * -------------------------------------------------------------------------
 *  Copyright (C) 2015 Intel Mobile Communications GmbH
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

#define FAN54015_NAME "fan54015_charger"
#define pr_fmt(fmt) FAN54015_NAME": "fmt

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
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <sofia/vmm_pmic.h>
#include <sofia/vmm_pmic-ext.h>

/*
 * Development debugging is not enabled in release image to prevent
 * loss of event history in the debug array which has a limited size
 */
#include <linux/power/charger_debug.h>

#define BYTE_MASK 0xFF

/* PMIC DEV addresses */
#define DEV1 0x4E
#define DEV3 0x5E

/* PMIC DEV1 registers */
#define CHGRIRQ0 0x09
#define MCHGRIRQ0 0x17
#define CHGINTB_O  0

#define IRQLVL1 0x02
#define MIRQLVL1 0x0E
#define CHGR_O 5

/* PMIC DEV3 registers */
#define CHGRCTRL0 0x16
#define CCSM_OFF_O 5
#define SWCONTROL_O 3

#define CHRLEDCTRL 0x1F
#define CHRLEDF_O 4
#define CHRLEDF_W 2
#define SWLEDON_O 1
#define CHRLEDFN_O 0

/* fan54015 vendor code and part number */
#define FAN54015_VENDOR 0x04
#define FAN54015_PN 0x05

/* Charger IC Registers */
#define REG_CONTROL0 0x00
#define REG_CONTROL1 0x01
#define REG_OREG 0x02
#define REG_IC_INFO 0x03
#define REG_IBAT 0x04
#define REG_SP_CHARGER 0x05
#define REG_SAFETY 0x06
#define REG_MONITOR0 0x10

/* REG_CONTROL0 */
#define TMR_RST_O 7
#define EN_STAT_O 6
#define STAT_O 4
#define BOOST_O 3
#define FAULT_O 0

#define TMR_RST_W 1
#define EN_STAT_W 1
#define STAT_W 2
#define BOOST_W 1
#define FAULT_W 3

/* REG_CONTROL1 */
#define IINLIM_O 6
#define VLOWV_O 4
#define TE_O 3
#define CE_O 2
#define HZ_MODE_O 1
#define OPA_MODE_O 0

#define IINLIM_W 2
#define VLOWV_W 2
#define TE_W 1
#define CE_W 1
#define HZ_MODE_W 1
#define OPA_MODE_W 1

/* REG_OREG */
#define OREG_O 2
#define OTG_PL_O 1
#define OTG_EN_O 0

#define OREG_W 6
#define OTG_PL_W 1
#define OTG_EN_W 1

/* REG_IC_INFO */
#define VC_O 5
#define PN_O 2
#define REV_O 0

#define VC_W 3
#define PN_W 3
#define REV_W 2

/* REG_IBAT */
#define RESET_O 7
#define IOCHARGE_O 4
#define ITERM_O 0

#define RESET_W 1
#define IOCHARGE_W 3
#define ITERM_W  3

/* REG_SP_CHARGER */
#define DIS_VREG_O 6
#define IO_LEVEL_O 5
#define SP_O 4
#define EN_LEVEL_O 3
#define VSP_O 0

#define DIS_VREG_W 1
#define IO_LEVEL_W 1
#define SP_W 1
#define EN_LEVEL_W 1
#define VSP_W 3

/* REG_SAFETY */
#define ISAFE_O 4
#define VSAFE_O 0

#define ISAFE_W 3
#define VSAFE_W 4

/* REG_MONITOR0 */
#define ITERM_CMP_O 7
#define VBAT_CMP_O 6
#define LINCHG_O 5
#define T_120_O 4
#define ICHG_O 3
#define IBUS_O 2
#define VBUS_VALID_O 1
#define CV_O 0

#define ITERM_CMP_W 1
#define VBAT_CMP_W 1
#define LINCHG_W 1
#define T_120_W 1
#define ICHG_W 1
#define IBUS_W 1
#define VBUS_VALID_W 1
#define CV_W 1

/* Default targets and ranges */
#define DEFAULT_CV 3540
#define DEFAULT_CC 550
#define DEFAULT_IBUS 100

#define IOCHARGE_STEP_MA 100
#define IOCHARGE_MIN_MA 550
#define IOCHARGE_MAX_MA 1450

#define VOREG_STEP_MV 20
#define VOREG_MIN_MV 3500
#define VOREG_MAX_MV 4440

#define IBUS_MAX_LIMIT_MA 900

/* Logging and debugging parameters */
#define MAX_NR_OF_I2C_RETRIES 1
#define CHRGR_WORK_DELAY (10*HZ)
#define EVT_WAKELOCK_TIMEOUT (2*HZ)
#define EVENTS_LOG_FILENAME "events"
#define DBG_REGS_FILENAME "regs"
#define DBG_STATE_FILENAME "state"
#define LOG_LINE_LENGTH (64)
#define LINES_PER_PAGE (PAGE_SIZE/LOG_LINE_LENGTH)
#define SHADOW_REGS_NR 10
#define SYSFS_INPUT_VAL_LEN (1)
#define FAN54015_DBGFS_REG_LEN 16

#define fit_in_range(_val, _MIN, _MAX) ((_val > _MAX) ? _MAX : \
					(_val < _MIN) ? _MIN : _val)

#define set_reg_field(_reg, _offset, _size, _val)\
do {\
	u8 _tmp;\
	_tmp = _val & ((1 << _size) - 1);\
	_reg &= ~(((1 << _size) - 1) << _offset);\
	_reg |= _tmp << _offset;\
} while (0)

#define get_reg_field(_reg, _offset, _size)\
({\
	u8 _val;\
	_val = (_reg >> _offset) & ((1 << _size) - 1);\
	_val;\
})

#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"

#define BYTETOBINARY(byte)  \
  (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0)


enum {
	POK_B_VALID = 0,
	POK_B_INVAL,

	TSD_OCCURRED = 1,

	OVP_OCCURRED = 1,

	OVP_RECOV_OCCURRED = 1,

	T32_TO_OCCURRED = 1,

	VBUS_FAULT = 1,

	TREG_IS_ON = 1,

	OT_RECOV_OCCURRED = 1,

	VBUS_OFF = 0,
	VBUS_ON,

	CLEAR = 0,
	SET = 1,
};

enum chrledfreq {
	QUARTER_HZ,
	HALF_HZ,
	ONE_HZ,
	TWO_HZ,
};

enum vlowv_lvl {
	VLOWV_3_4V = 0,
	VLOWV_3_5V,
	VLOWV_3_6V,
	VLOWV_3_7V,
};

enum vsafe {
	VSAFE4P20 = 0,
	VSAFE4P22,
	VSAFE4P24,
	VSAFE4P26,
	VSAFE4P28,
	VSAFE4P30,
	VSAFE4P32,
	VSAFE4P34,
	VSAFE4P36,
	VSAFE4P38,
	VSAFE4P40,
	VSAFE4P42,
	VSAFE4P44,
};

enum isafe {
	ISAFE550 = 0,
	ISAFE650,
	ISAFE750,
	ISAFE850,
	ISAFE1050,
	ISAFE1150,
	ISAFE1350,
	ISAFE1450,
};

enum charger_status {
	FAN54015_STATUS_UNKNOWN,
	FAN54015_STATUS_READY,
	FAN54015_STATUS_FAULT,
};

static const int iocharge_ma[8] = {550, 650, 750, 850, 1050, 1150, 1350, 1450};

/**
 * struct fan54015_state - FAN54015 charger current state
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
 * @vbus_ovp		VBUS over voltage fault flag
 * @sleep_mode		sleep mode flag
 * @poor_input_src	poor input source fault flag
 * @battery_ovp		battery over voltage fault flag
 * @thermal_shutdown	thermal shutdown fault flag
 * @timer_fault		timer fault flag
 * @no_battery		no battery fault flag
 */
struct fan54015_state {
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
	unsigned int vbus_ovp:1;
	unsigned int sleep_mode:1;
	unsigned int poor_input_src:1;
	unsigned int battery_ovp:1;
	unsigned int thermal_shutdown:1;
	unsigned int timer_fault:1;
	unsigned int no_battery:1;
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
 * struct fan54015_charger - FAN54015 charger driver internal structure
 * @charging_work		work providing charging heartbeat for PSC
 * @ctrl_io			PMU Charger regs physical address
 * @otg_handle			Pointer to USB OTG internal structure
 *				used for sending VBUS notifications.
 * @usb_psy			power supply instance struct for USB path
 * @ac_psy			power supply instance struct for AC path
 * @current_psy			pointer to psy representing current charging
 *				path.
 * @debugfs_root_dir		debugfs fan54015 charger root directory
 * @prop_lock			synchronization semaphore
 * @model_name			model name of charger chip
 * @manufacturer		manufacturer name of charger chip
 * @ack_time			last CONTINUE_CHARGING timestamp in jiffies
 * @state			charger state structure
 */
struct fan54015_charger {
	struct i2c_client *client;

	struct unfreezable_bh_struct chgerr_bh;


	struct delayed_work charging_work;
	void __iomem *ctrl_io;
	struct usb_phy *otg_handle;

	struct power_supply_cable_props cable_props;

	struct power_supply usb_psy;
	struct power_supply ac_psy;

	struct power_supply *current_psy;

	struct dentry *debugfs_root_dir;

	struct semaphore prop_lock;
	struct wake_lock suspend_lock;
	const char *model_name;
	const char *manufacturer;

	unsigned long ack_time;

	struct fan54015_state state;
	int irq_chgintb;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct pinctrl_state *pins_active;
	struct device_pm_platdata *pm_platdata;
};

static struct fan54015_charger fan54015_chrgr_data = {
	.model_name = "FAN54015",
	.manufacturer = "FAIRCHILD",

	.chgerr_bh = {
		.in_suspend = false,
		.pending_evt = false,
	},
	.state = {
		.status = FAN54015_STATUS_UNKNOWN,
		.vbus = -1,
		.cc = DEFAULT_CC,
		.max_cc = IOCHARGE_MAX_MA,
		.cv = DEFAULT_CV,
		.iterm = 0,
		.health = POWER_SUPPLY_HEALTH_UNKNOWN,
		.cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE,
		.charger_enabled = false,
		.charging_enabled = false, /* initially HZ mode is switched off*/
		.vbus_ovp = 0,
		.sleep_mode = 0,
		.poor_input_src = 0,
		.battery_ovp = 0,
		.thermal_shutdown = 0,
		.timer_fault = 0,
		.no_battery = 0,
	},
};

static struct charger_debug_data chrgr_dbg;

static struct power_supply_throttle fan54015_dummy_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
	},
};

static char *fan54015_supplied_to[] = {
		"battery",
};

static enum power_supply_property fan54015_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

/**
 * PMIC register read function
 *
 * @dev		PMIC slave device
 * @reg		register address
 * @p_val	pointer to buffer storing the data read
 * @return	0 on success, else error code
 */
static int fan54015_pmic_reg_read(u32 dev, u32 reg, u8 *p_val)
{
	u32 vmm_addr, data;
	int ret;

	vmm_addr = ((dev & BYTE_MASK) << 24) | (reg & BYTE_MASK);
	ret = vmm_pmic_reg_read(vmm_addr, &data);
	data &= BYTE_MASK;
	*p_val = (u8) data;

	return ret;
}

/**
 * PMIC register write function
 *
 * @dev		PMIC slave device
 * @reg		register address
 * @val		value to be written
 * @return	0 on success, else error code
 */
static int fan54015_pmic_reg_write(u32 dev, u32 reg, u8 val)
{
	u32 vmm_addr, data;

	vmm_addr = ((dev & BYTE_MASK) << 24) | (reg & BYTE_MASK);
	data = (u32) val;
	return vmm_pmic_reg_write(vmm_addr, data);
}

/**
 * PMIC register bit field set function (read-modify-write)
 *
 * @dev		PMIC slave devcie
 * @reg		register address
 * @mask	register mask
 * @val		value to be written
 * @return	0 on success, else error code
 */
static int fan54015_pmic_reg_set(u32 dev, u32 reg, u8 mask, u8 val)
{
	u32 vmm_addr;

	vmm_addr = ((dev & BYTE_MASK) << 24) | (reg & BYTE_MASK);
	return pmic_reg_set_field(vmm_addr, mask, val);
}

/**
 * Function for reading fan54015 registers
 * @client	pointer i2c client structure
 * @reg_addr	register address
 * @data	value read from register
 * @returns	0 if successful, else error code
 */
static int fan54015_i2c_reg_read(struct i2c_client *client, u8 reg_addr,
								u8 *data)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg_addr);

	if (ret >= 0) {
		*data = (u8) ret;
		ret = 0;
		CHARGER_DEBUG_READ_REG(chrgr_dbg, reg_addr, *data);
	} else {
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_READ_ERROR, ret,
								reg_addr);

		dev_err(&client->dev, "I2C read error. reg=%d, data=0x%02X, ret=0x%02X\n",
			reg_addr, *data, ret);
	}
	return ret;
}

/**
 * Function for writing to fan54015 registers
 * @client	pointer i2c client structure
 * @reg_addr	register address
 * @data	value to be written to register
 * @return	0 if successful, else error code
 */
static int fan54015_i2c_reg_write(struct i2c_client *client, u8 reg_addr,
								u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg_addr, data);

	if (ret >= 0) {
		CHARGER_DEBUG_WRITE_REG(chrgr_dbg, reg_addr, data);
		ret = 0;
	}
	else {
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_WRITE_ERROR, ret,
								reg_addr);

		dev_err(&client->dev, "WRITE END reg=%d, data=0x%02X, ret=0x%02X\n",
			reg_addr, data, ret);
	}
	return ret;
}

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

static const struct file_operations fan54015_evt_dbg_fops = {
	.open = dbg_evt_open,
	.read = dbg_evt_read,
};

static int fan54015_dbg_regs_dump(struct seq_file *m, void *data)
{
	struct fan54015_charger *chrgr = (struct fan54015_charger *) m->private;
	u8 reg;
	int ret;

	ret = fan54015_i2c_reg_read(chrgr->client, REG_CONTROL0, &reg);
	seq_printf(m, "REG_CONTROL0:\t\tHEX=0x%02X, BIN=0b"BYTETOBINARYPATTERN"\n",
				reg, BYTETOBINARY(reg));

	ret = fan54015_i2c_reg_read(chrgr->client, REG_CONTROL1, &reg);
	seq_printf(m, "REG_CONTROL1:\t\tHEX=0x%02X, BIN=0b"BYTETOBINARYPATTERN"\n",
				reg, BYTETOBINARY(reg));

	ret = fan54015_i2c_reg_read(chrgr->client, REG_OREG, &reg);
	seq_printf(m, "REG_OREG:\t\tHEX=0x%02X, BIN=0b"BYTETOBINARYPATTERN"\n",
				reg, BYTETOBINARY(reg));

	ret = fan54015_i2c_reg_read(chrgr->client, REG_IC_INFO, &reg);
	seq_printf(m, "REG_IC_INFO:\t\tHEX=0x%02X, BIN=0b"BYTETOBINARYPATTERN"\n",
				reg, BYTETOBINARY(reg));

	ret = fan54015_i2c_reg_read(chrgr->client, REG_IBAT, &reg);
	seq_printf(m, "REG_IBAT:\t\tHEX=0x%02X, BIN=0b"BYTETOBINARYPATTERN"\n",
				reg, BYTETOBINARY(reg));

	ret = fan54015_i2c_reg_read(chrgr->client, REG_SP_CHARGER, &reg);
	seq_printf(m, "REG_SP_CHARGER:\tHEX=0x%02X, BIN=0b"BYTETOBINARYPATTERN"\n",
				reg, BYTETOBINARY(reg));

	ret = fan54015_i2c_reg_read(chrgr->client, REG_SAFETY, &reg);
	seq_printf(m, "REG_SAFETY:\t\tHEX=0x%02X, BIN=0b"BYTETOBINARYPATTERN"\n",
				reg, BYTETOBINARY(reg));

	ret = fan54015_i2c_reg_read(chrgr->client, REG_MONITOR0, &reg);
	seq_printf(m, "REG_MONITOR0:\t\tHEX=0x%02X, BIN=0b"BYTETOBINARYPATTERN"\n",
				reg, BYTETOBINARY(reg));

	return 0;
}

static int fan54015_dbg_regs_dump_open(struct inode *inode, struct file *file)
{
	return single_open(file, fan54015_dbg_regs_dump, inode->i_private);
}

static int fan54015_dbg_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t fan54015_dbg_regs_read(struct file *fp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	ssize_t cnt;
	int ret;
	char str[FAN54015_DBGFS_REG_LEN];
	u8 data, reg;
	struct fan54015_charger *chrgr = &fan54015_chrgr_data ;

	BUG_ON(!buf || !f_pos || !fp);

	if (*f_pos >= FAN54015_DBGFS_REG_LEN - 1)
		return 0;

	reg  = (u8) ((u32)fp->private_data);

	ret = fan54015_i2c_reg_read(chrgr->client, reg, &data);
	if (ret)
		return -EIO;

	cnt = snprintf(str, FAN54015_DBGFS_REG_LEN, "0x%02X: "BYTETOBINARYPATTERN"\n",
							data, BYTETOBINARY(data));

	if (*f_pos > cnt)
		return 0;
	if (cnt < 0)
		return -EIO;
	if (*f_pos + count > cnt)
		count = cnt - *f_pos;
	if (copy_to_user(buf, str + *f_pos, count))
		return -EFAULT;

	*f_pos += count;

	return count;
}

static ssize_t fan54015_dbg_regs_write(struct file *fp,	const char __user *buf,
					size_t count, loff_t *f_pos)
{
	u8 data, reg;
	int ret;
	char str[FAN54015_DBGFS_REG_LEN];
	struct fan54015_charger *chrgr = &fan54015_chrgr_data ;

	BUG_ON(!buf || !f_pos || !fp);
	reg = (u8) ((u32) fp->private_data);

	if (*f_pos >= FAN54015_DBGFS_REG_LEN)
		return -EFAULT;
	if (*f_pos + count > FAN54015_DBGFS_REG_LEN)
		count = FAN54015_DBGFS_REG_LEN - *f_pos;
	if (copy_from_user(str + *f_pos, buf, count))
		return -EFAULT;
	*f_pos += count;

	snprintf(str, *f_pos, "%s\n", str);

	ret = kstrtou8(str, 0, &data);
	if (ret)
		return ret;

	dev_info(&chrgr->client->dev,
			"sysfs fan54015 write: reg=0x%x, val=0x%x\n", reg, data);

	ret = fan54015_i2c_reg_write(chrgr->client, reg, data);
	if (ret)
		return -EIO;

	return count;
}

static const struct file_operations fan54015_dbg_regs_dump_fops = {
	.open = fan54015_dbg_regs_dump_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations fan54015_dbg_regs_fops = {
	.owner = THIS_MODULE,
	.open = fan54015_dbg_regs_open,
	.read = fan54015_dbg_regs_read,
	.write = fan54015_dbg_regs_write,
};


static int fan54015_dbg_state_show(struct seq_file *m, void *data)
{
	struct fan54015_charger *chrgr = (struct fan54015_charger *) m->private;
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
	seq_printf(m, "charging_enabled = %d\n\n",
				chrgr->state.charging_enabled);

	seq_printf(m, "vbus_ovp = %u\n", chrgr->state.vbus_ovp);
	seq_printf(m, "sleep_mode = %u\n", chrgr->state.sleep_mode);
	seq_printf(m, "poor_input_src = %u\n",
				chrgr->state.poor_input_src);

	seq_printf(m, "battery_ovp = %u\n", chrgr->state.battery_ovp);
	seq_printf(m, "thermal_shutdown = %u\n", chrgr->state.thermal_shutdown);
	seq_printf(m, "timer_fault = %u\n\n", chrgr->state.timer_fault);
	seq_printf(m, "no_battery = %u\n\n", chrgr->state.no_battery);

	return 0;
}

static int fan54015_dbg_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, fan54015_dbg_state_show, inode->i_private);
}

static const struct file_operations fan54015_dbg_state_fops = {
	.open = fan54015_dbg_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/**
 * fan54015_setup_debugfs - sets up debugfs entries for fan54015 charger driver
 * @chrgr		[in] pointer to charger driver internal structure
 * @dbg_data		[in] pointer to debug array containing events logs.
 */
static void fan54015_setup_debugfs(struct fan54015_charger *chrgr,
				struct charger_debug_data *dbg_data)
{
	struct dentry *dbgfs_entry;

	dbgfs_entry = debugfs_create_dir(FAN54015_NAME, NULL);
	if (!dbgfs_entry)
		return;

	chrgr->debugfs_root_dir = dbgfs_entry;

	(void)debugfs_create_file(EVENTS_LOG_FILENAME, S_IRUGO,
			dbgfs_entry, dbg_data, &fan54015_evt_dbg_fops);

	(void)debugfs_create_file(DBG_STATE_FILENAME, S_IRUGO,
			dbgfs_entry, chrgr, &fan54015_dbg_state_fops);

	dbgfs_entry = debugfs_create_dir(DBG_REGS_FILENAME, dbgfs_entry);
	if (!dbgfs_entry)
		return;

	(void)debugfs_create_file("all_regs", S_IRUGO,
			dbgfs_entry, chrgr, &fan54015_dbg_regs_dump_fops);
	(void)debugfs_create_file("CONTROL0", S_IRUGO | S_IWUGO,
			dbgfs_entry, (void *) REG_CONTROL0, &fan54015_dbg_regs_fops);
	(void)debugfs_create_file("CONTROL1", S_IRUGO | S_IWUGO,
			dbgfs_entry, (void *) REG_CONTROL1, &fan54015_dbg_regs_fops);
	(void)debugfs_create_file("OREG", S_IRUGO | S_IWUGO,
			dbgfs_entry, (void *) REG_OREG, &fan54015_dbg_regs_fops);
	(void)debugfs_create_file("IC_INFO", S_IRUGO | S_IWUGO,
			dbgfs_entry, (void *) REG_IC_INFO, &fan54015_dbg_regs_fops);
	(void)debugfs_create_file("IBAT", S_IRUGO | S_IWUGO,
			dbgfs_entry, (void *) REG_IBAT, &fan54015_dbg_regs_fops);
	(void)debugfs_create_file("VBUS_CONTROL", S_IRUGO | S_IWUGO,
			dbgfs_entry, (void *) REG_SP_CHARGER, &fan54015_dbg_regs_fops);
	(void)debugfs_create_file("SAFETY", S_IRUGO | S_IWUGO,
			dbgfs_entry, (void *) REG_SAFETY, &fan54015_dbg_regs_fops);
	(void)debugfs_create_file("MONITOR0", S_IRUGO | S_IWUGO,
			dbgfs_entry, (void *) REG_MONITOR0, &fan54015_dbg_regs_fops);
	return;
}

/**
 * fan54015_remove_debugfs_dir	Recursively removes debugfs root directory
 *				of FAN54015 charger driver
 * @chrgr			[in] pointer to charger driver's
 *				internal structure
 */
static void fan54015_remove_debugfs_dir(struct fan54015_charger *chrgr)
{
	debugfs_remove_recursive(chrgr->debugfs_root_dir);
	return;
}

#else

static inline void fan54015_setup_debugfs(struct fan54015_charger *chrgr,
				struct charger_debug_data *dbg_data)
{

}

static inline void fan54015_remove_debugfs_dir(struct fan54015_charger *chrgr)
{

}

#endif /* CONFIG_DEBUG_FS  */

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

/**
 * Disable CCSM and use SW to control charging/LED
 *
 * @return	0 if successful, else error code
 */
static int fan54015_sw_control_overide(void)
{
	int ret;
	u8 mask = 0, val;

	/* Disable CCSM and enable SW control */
	ret = fan54015_pmic_reg_read(DEV3, CHGRCTRL0, &val);
	if (ret)
		return ret;

	set_reg_field(val, CCSM_OFF_O, 1, SET);
	set_reg_field(val, SWCONTROL_O, 1, SET);

	ret = fan54015_pmic_reg_write(DEV3, CHGRCTRL0, val);
	if (ret)
		return ret;

	/* SW control LED */
	val = 0;
	set_reg_field(mask, CHRLEDFN_O, 1, SET);
	set_reg_field(mask, SWLEDON_O, 1, SET);

	set_reg_field(val, CHRLEDFN_O, 1, SET);
	set_reg_field(val, SWLEDON_O, 1, CLEAR);

	ret = fan54015_pmic_reg_set(DEV3, CHRLEDCTRL, mask, val);

	return ret;
}

/**
 * Set charger LED frequency
 *
 * @freq	frequency to be set
 * @return	0 if successful, else error code
 */
static int fan54015_set_chr_led_freq(enum chrledfreq freq)
{
	int ret;
	u8 mask = 0, val = 0;
	switch (freq) {
	case QUARTER_HZ:
		val = 0;
		break;
	case HALF_HZ:
		val = 1;
		break;
	case ONE_HZ:
		val = 2;
		break;
	case TWO_HZ:
		val = 3;
		break;
	default:
		return -EINVAL;
		break;
	}

	set_reg_field(mask, CHRLEDF_O, CHRLEDF_W, 0x3);
	set_reg_field(val, CHRLEDF_O, CHRLEDF_W, val);

	ret = fan54015_pmic_reg_set(DEV3, CHRLEDCTRL, mask, val);

	return ret;
}

/**
 * Trigger charger watchdog
 *
 * @chrge	pointer to charger driver internal structure
 * @return	0 if successful, else error code
 */
static int fan54015_trigger_wtd(struct fan54015_charger *chrgr)
{
	int ret = 0;
	u8 ctrl0_reg;

	ret = fan54015_i2c_reg_read(chrgr->client, REG_CONTROL0, &ctrl0_reg);
	if (ret)
		return -EIO;
	set_reg_field(ctrl0_reg, TMR_RST_O, TMR_RST_W, SET);
	ret = fan54015_i2c_reg_write(chrgr->client, REG_CONTROL0, ctrl0_reg);
	if (ret)
		return -EIO;

	return ret;
}

/**
 * Reset charger IC
 *
 * @chrgr	pointer to charger driver internal structure
 * @return	0 if successful, else error code
 */
static int fan54015_reset_ic(struct fan54015_charger *chrgr)
{
	int ret = 0;
	u8 ibat_reg;

	ret = fan54015_i2c_reg_read(chrgr->client, REG_IBAT, &ibat_reg);
	if (ret)
		return -EIO;

	set_reg_field(ibat_reg, RESET_O, RESET_W, SET);

	ret = fan54015_i2c_reg_write(chrgr->client, REG_IBAT, ibat_reg);
	if (ret)
		return -EIO;

	return ret;
}

/**
 * Set IO_LEVEL to 0. Output current is controlled by IOCHARGE bits
 *
 * @chrgr	pointer to charger driver internal structure
 * @return	0 if successful, else error code
 */
static int fan54015_disable_iolevel(struct fan54015_charger *chrgr)
{
	int ret = 0;
	u8 vbus_ctrl_reg;

	ret = fan54015_i2c_reg_read(chrgr->client, REG_SP_CHARGER, &vbus_ctrl_reg);
	if (ret)
		return -EIO;

	set_reg_field(vbus_ctrl_reg, IO_LEVEL_O, IO_LEVEL_W, CLEAR);

	ret = fan54015_i2c_reg_write(chrgr->client, REG_SP_CHARGER, vbus_ctrl_reg);
	if (ret)
		return -EIO;

	return ret;
}

/**
 * Set weak battery voltage threshold
 *
 * @chrgr	pointer to charger driver internal structure
 * @lvl		weak byttery voltage to set
 * @return	0 if successful, else error code
 */
static int fan54015_set_weakbat_level(struct fan54015_charger *chrgr,
					enum vlowv_lvl lvl)
{
	int ret = 0;
	u8 ctrl1_reg;

	ret = fan54015_i2c_reg_read(chrgr->client, REG_CONTROL1, &ctrl1_reg);
	if (ret)
		return -EIO;

	set_reg_field(ctrl1_reg, VLOWV_O, VLOWV_W, lvl);

	ret = fan54015_i2c_reg_write(chrgr->client, REG_CONTROL1, ctrl1_reg);
	if (ret)
		return -EIO;

	return ret;
}

/**
 * Set charger safe voltage
 *
 * @chrgr	pointer to charger driver internal structure
 * @volt_safe	safe voltage to set
 * @return	0 if succeccful, else error code
 */
static int fan54015_set_vsafe(struct fan54015_charger *chrgr,
						enum vsafe volt_safe)
{
	int ret = 0;
	u8 safety_reg;

	ret = fan54015_i2c_reg_read(chrgr->client, REG_SAFETY, &safety_reg);
	if (ret)
		return -EIO;

	set_reg_field(safety_reg, VSAFE_O, VSAFE_O, volt_safe);

	ret = fan54015_i2c_reg_write(chrgr->client, REG_SAFETY, safety_reg);
	if (ret)
		return -EIO;

	return ret;
}

/**
 * Set charger safe iocharge
 *
 * @chrgr	pointer to charger driver internal structure
 * @volt_safe	safe vioarge to set
 * @return	0 if succeccful, else error code
 */
static int fan54015_set_isafe(struct fan54015_charger *chrgr, enum isafe iochr_safe)
{
	int ret = 0;
	u8 safety_reg;

	ret = fan54015_i2c_reg_read(chrgr->client, REG_SAFETY, &safety_reg);
	if (ret)
		return -EIO;

	set_reg_field(safety_reg, ISAFE_O, ISAFE_W, iochr_safe);

	ret = fan54015_i2c_reg_write(chrgr->client, REG_SAFETY, safety_reg);
	if (ret)
		return -EIO;

	return ret;
}

/**
 * Set charger target voltage
 *
 * @chrgr	pointer to charger driver internal structure
 * @volt_to_set	target voltage to set
 * @volt_set	pointer to target voltage actually set to HW
 * #propagate	whether to propage to HW
 * @return	0 if successful, else error code
 */
static int fan54015_set_voreg(struct fan54015_charger *chrgr,
				int volt_to_set, int *volt_set, bool propagate)
{
	u8 oreg_reg, voreg_val;
	int ret = 0;

	if (!volt_set)
		return -EINVAL;

	volt_to_set = fit_in_range(volt_to_set, VOREG_MIN_MV, VOREG_MAX_MV);

	voreg_val = (volt_to_set - VOREG_MIN_MV) / VOREG_STEP_MV;

	*volt_set = voreg_val * VOREG_STEP_MV + VOREG_MIN_MV;

	if (propagate) {
		ret = fan54015_i2c_reg_read(chrgr->client, REG_OREG, &oreg_reg);
		if (ret)
			return -EIO;

		set_reg_field(oreg_reg, OREG_O, OREG_W, voreg_val);

		ret = fan54015_i2c_reg_write(chrgr->client, REG_OREG, oreg_reg);
		if (ret)
			return -EIO;
	}
	dev_err(&chrgr->client->dev,
		"%s: ###### volt_to_set=%d, volt_set=%d, prop=%d\n",
	       __func__, volt_to_set, *volt_set, propagate);
	return ret;
}

/**
 * Set charger target current
 *
 * @chrgr	pointer to charger driver internal structure
 * @curr_to_set	target current to set
 * @curr_set	pointer to current target actually set to HW
 * @propagate	whether to propage to HW
 * @return	0 if successful, else error code
 */
static int fan54015_set_iocharge(struct fan54015_charger *chrgr,
			unsigned int curr_to_set, int *curr_set, bool propagate)
{
	u8 iochr_val, ibat_reg;
	int ret = 0;

	if (!curr_set)
		return -EINVAL;

	curr_to_set = fit_in_range(curr_to_set, IOCHARGE_MIN_MA, IOCHARGE_MAX_MA);

	for (iochr_val = 0; iochr_val < 8; iochr_val++) {
		if (curr_to_set <= iocharge_ma[iochr_val]) {
			*curr_set = iocharge_ma[iochr_val];
			break;
		}
	}

	if (propagate) {

		ret = fan54015_i2c_reg_read(chrgr->client, REG_IBAT, &ibat_reg);
		if (ret)
			return -EIO;

		set_reg_field(ibat_reg, IOCHARGE_O, IOCHARGE_W, iochr_val);

		ret = fan54015_i2c_reg_write(chrgr->client, REG_IBAT, ibat_reg);
		if (ret)
			return -EIO;
	}
	dev_err(&chrgr->client->dev,
		"%s: ###### curr_to_set=%d, curr_set=%d, prop=%d\n",
		__func__, curr_to_set, *curr_set, propagate);
	return ret;
}

/**
 * Set charger input current limit
 *
 * @chrgr	pointer to charger driver internal structure
 * @ibus_to_set	target input current limit to set
 * @ibus_set	pointer to input current limit actually set to HW
 * @return	0 if successful, else error code
 */
static int fan54015_set_ibus_limit(struct fan54015_charger *chrgr,
						int ibus_to_set, int *ibus_set)
{
	int ret;
	u8 ctrl1_reg, ibuslim_val;
	enum chrledfreq freq;

	/*TODO chrgr?*/
	if (!ibus_set)
		return -EINVAL;

	if (ibus_to_set <= 100) {
		ibuslim_val = 0x00;
		*ibus_set = 100;
		freq = HALF_HZ;
	} else if (ibus_to_set < 500) {
		ibuslim_val = 0x00;
		*ibus_set = 100;
		freq = HALF_HZ;

	} else if (ibus_to_set < 800) {
		ibuslim_val = 0x01;
		*ibus_set = 500;
		freq = ONE_HZ;
	} else {
		ibuslim_val = 0x3;
		*ibus_set = ibus_to_set;
		freq = TWO_HZ;
	}

	ret = fan54015_i2c_reg_read(chrgr->client, REG_CONTROL1, &ctrl1_reg);
	if (ret)
		return -EIO;

	set_reg_field(ctrl1_reg, IINLIM_O, IINLIM_W, ibuslim_val);

	ret = fan54015_i2c_reg_write(chrgr->client, REG_CONTROL1, ctrl1_reg);
	if (ret)
		return -EIO;

	ret = fan54015_set_chr_led_freq(freq);

	return ret;
}


/**
 * Function to enable / disable charging
 *
 * @chrgr	pointer to charger driver internal structure
 * @enable	whether enable or disable charging
 * @return	0 if successful, else error code
 */
static int fan54015_enable_charging(struct fan54015_charger *chrgr, bool enable)
{
	u8 val;
	u8 mask = 0;
	int ret;

	ret = fan54015_i2c_reg_read(chrgr->client, REG_OREG, &val);

	if (enable)
		fan54015_trigger_wtd(chrgr);

	ret = fan54015_i2c_reg_read(chrgr->client, REG_CONTROL1, &val);
	if (ret)
		return -EIO;

	if (enable) {
		set_reg_field(val, HZ_MODE_O, HZ_MODE_W, CLEAR);
		set_reg_field(val, CE_O, CE_W, CLEAR);
	} else {
		/* HZ mode is always disabled */
		set_reg_field(val, HZ_MODE_O, HZ_MODE_W, SET);
		set_reg_field(val, CE_O, CE_W, SET);
	}

	ret = fan54015_i2c_reg_write(chrgr->client, REG_CONTROL1, val);
	if (ret)
		return -EIO;

	set_reg_field(mask, SWLEDON_O, 1, SET);
	val = 0;

	if (enable)
		set_reg_field(val, SWLEDON_O, 1, SET);
	else
		set_reg_field(val, SWLEDON_O, 1, CLEAR);

	ret = fan54015_pmic_reg_set(DEV3, CHRLEDCTRL, mask, val);

	if (enable) {
		wake_lock(&chrgr->suspend_lock);
		schedule_delayed_work(&chrgr->charging_work, CHRGR_WORK_DELAY);
	} else {
		wake_unlock(&chrgr->suspend_lock);
		cancel_delayed_work(&chrgr->charging_work);
	}
	return ret;
}

/**
 * Configuring FAN54015 chip registers
 * @chrgr	pointer to charger driver internal structure
 * @enable	controls if charging should be anebled or disabled
 * @return	0 if successful, else error code
 */
static int fan54015_configure_charger(struct fan54015_charger *chrgr, bool enable)
{
	int ret, val;

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_CONFIGURE_CHIP, 0, 0);

	ret = fan54015_reset_ic(chrgr);
	if (ret)
		return ret;
	ret = fan54015_disable_iolevel(chrgr);
	if (ret)
		return ret;
	ret = fan54015_set_weakbat_level(chrgr, VLOWV_3_4V);
	if (ret)
		return ret;
	ret = fan54015_set_ibus_limit(chrgr, DEFAULT_IBUS, &val);
	if (ret)
		return ret;
	ret = fan54015_set_iocharge(chrgr, DEFAULT_CC, &val, true);
	if (ret)
		return ret;
	ret = fan54015_set_voreg(chrgr, DEFAULT_CV, &val, true);

	return ret;
}

/**
 * Set default target targets.
 *
 * @chrgr	pointer to charger driver internal structure
 * @return	0 if successful, else error code
 */
static int fan54015_set_default_targets(struct fan54015_charger *chrgr)
{
	int ret, val;

	ret = fan54015_disable_iolevel(chrgr);
	if (ret)
		return ret;

	ret = fan54015_set_weakbat_level(chrgr, VLOWV_3_4V);
	if (ret)
		return ret;

	ret = fan54015_set_ibus_limit(chrgr, DEFAULT_IBUS, &val);
	if (ret)
		return ret;

	ret = fan54015_set_iocharge(chrgr, DEFAULT_CC, &val, true);
	if (ret)
		return ret;

	ret = fan54015_set_voreg(chrgr, DEFAULT_CV, &val, true);
	if (ret)
		return ret;

	ret = fan54015_set_vsafe(chrgr, VSAFE4P20);
	if (ret)
		return ret;

	ret = fan54015_set_isafe(chrgr, ISAFE1450);

	return ret;
}

/**
 * Check whether charging is on-going on the specified charger
 *
 * @chrgr	pointer to charger driver internal structure
 * @psy		pointer to power supply charger instance
 * @return	true if charging is on-going, else false
 */
static inline bool fan54015_is_online(struct fan54015_charger *chrgr,
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

/**
 * Working function to trigger watchdog and update PSY
 *
 * @work	pointer to work instance
 */
static void fan54015_charging_worker(struct work_struct *work)
{
	int ret;
	struct fan54015_charger *chrgr =
		container_of(work, struct fan54015_charger, charging_work.work);


	if (!time_after(jiffies, chrgr->ack_time + (60*HZ))) {
		down(&chrgr->prop_lock);
		ret = fan54015_trigger_wtd(chrgr);
		up(&chrgr->prop_lock);
		if (ret != 0)
			return;
	}

	power_supply_changed(chrgr->current_psy);
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TRIG_POWER_SUPPLY_CHARGER, 0, 0);

	schedule_delayed_work(&chrgr->charging_work, CHRGR_WORK_DELAY);

	return;
}

/**
 * Power supply charger property set function
 *
 * @psy		pointer to power supply class charger instance
 * @psp		property to set
 * @val		pointer to value to set
 * @return	0 if successful, else error code
 */
static int fan54015_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	int value_to_set, value_set = 0, ret = 0;
	struct fan54015_charger *chrgr = &fan54015_chrgr_data ;

	bool call_psy_changed = false;

	down(&chrgr->prop_lock);

	if (chrgr->state.status == FAN54015_STATUS_FAULT) {
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
			POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING)) {

			chrgr->current_psy = &chrgr->usb_psy;
			chrgr->state.health = POWER_SUPPLY_HEALTH_GOOD;
			ret = fan54015_configure_charger(chrgr, true);
			if (ret)
				dev_err(&chrgr->client->dev,
					"%s - fail to config IC\n", __func__);
		} else if ((chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_CDP) ||
			(chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_DCP)) {

			chrgr->current_psy = &chrgr->ac_psy;
			chrgr->state.health = POWER_SUPPLY_HEALTH_GOOD;
			ret = fan54015_configure_charger(chrgr, true);
			if (ret)
				dev_err(&chrgr->client->dev,
					"%s - fail to config IC\n", __func__);
		} else if (chrgr->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_NONE) {
			chrgr->state.health = POWER_SUPPLY_HEALTH_UNKNOWN;
		}
		call_psy_changed = true;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_CURRENT,
								val->intval, 0);

		value_to_set = fit_in_range(val->intval, 0,
						chrgr->state.max_cc);

		fan54015_set_iocharge(chrgr, value_to_set, &value_set, false);
		if (value_set == chrgr->state.cc)
			break;
		ret = fan54015_set_iocharge(chrgr, value_to_set,
							&value_set, true);

		if (ret) {
			chrgr->state.status = FAN54015_STATUS_FAULT;
			chrgr->state.health =
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}

		chrgr->state.cc = value_set;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_VOLTAGE,
								val->intval, 0);

		fan54015_set_voreg(chrgr, val->intval, &value_set, false);
		if (value_set == chrgr->state.cv)
			break;
		ret = fan54015_set_voreg(chrgr, val->intval, &value_set, true);
		if (ret) {
			chrgr->state.status = FAN54015_STATUS_FAULT;
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
		ret = fan54015_enable_charging(chrgr, val->intval);
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
		ret = fan54015_set_ibus_limit(chrgr, value_to_set, &value_set);
		if (ret) {
			chrgr->state.status = FAN54015_STATUS_FAULT;
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

/**
 * Power supply charger property get function
 *
 * @psy		pointer to power supply class charger instance
 * @psp		property to get
 * @val		pointer to buffer to store value get
 * @return	0 if successful, else error code
 */
static int fan54015_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct fan54015_charger *chrgr = &fan54015_chrgr_data ;

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
		val->intval = fan54015_is_online(chrgr, psy);
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

/**
 * Worker function process charger error
 *
 * @work	pointer to work structure
 * @return	0 if successful, else error code
 */
static void fan54015_charger_error_handler(struct work_struct *work)
{
	int  health_prev, ret = 0;
	u8 val, reg = 0;
	struct fan54015_charger *chrgr = container_of(work,
				struct fan54015_charger, chgerr_bh.work);

	ret = fan54015_i2c_reg_read(chrgr->client, REG_CONTROL0, &reg);
	if (ret) {
		dev_err(&chrgr->client->dev,
				"%s: Fail to read CONTROL0\n", __func__);
		return;
	}

	val = get_reg_field(reg, STAT_O, STAT_W);

	down(&chrgr->prop_lock);
	health_prev = chrgr->state.health;

	if (!val) {
		chrgr->state.vbus_ovp = 0;
		chrgr->state.sleep_mode = 0;
		chrgr->state.poor_input_src = 0;
		chrgr->state.battery_ovp = 0;
		chrgr->state.thermal_shutdown = 0;
		chrgr->state.timer_fault = 0;
		chrgr->state.no_battery = 0;
		up(&chrgr->prop_lock);

		return;
	}

	val = get_reg_field(reg, FAULT_O, FAULT_W);

	switch (val) {
	case 1:
		/* VBUS OVP */
		chrgr->state.vbus_ovp = 1;
		chrgr->state.health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_VBUS_FAULT, 0, 0);
		dev_err(&chrgr->client->dev, "%s: VBUS over voltage!", __FILE__);
		break;

	case 2:
		/* Sleep Mode */
		chrgr->state.sleep_mode = 1;
		dev_err(&chrgr->client->dev, "%s: Charger in sleep mode!", __FILE__);
		break;

	case 3:
		/* Poor Input Source */
		chrgr->state.poor_input_src = 1;
		chrgr->state.health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_POOR_INPUT_SRC, 0, 0);
		dev_err(&chrgr->client->dev, "%s: Poor input source fault!", __FILE__);
		break;
	case 4:
		/* Battery OVP */
		chrgr->state.battery_ovp = 1;
		chrgr->state.health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_VBAT_OVP, 0, 0);
		dev_err(&chrgr->client->dev, "%s: Battery over voltage!", __FILE__);
		break;
	case 5:
		/* Thermal Shutdown */
		chrgr->state.thermal_shutdown = 1;
		chrgr->state.health = POWER_SUPPLY_HEALTH_OVERHEAT;
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TSD_IS_ON, 0, 0);
		dev_err(&chrgr->client->dev, "%s: Chip Over-temperature Shutdown!", __FILE__);
		break;
	case 6:
		/* Timer Fault */
		chrgr->state.timer_fault = 1;
		chrgr->state.health = POWER_SUPPLY_HEALTH_DEAD;
		chrgr->state.status = FAN54015_STATUS_FAULT;
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_T32_TIMER_EXPIRED, 0, 0);
		dev_err(&chrgr->client->dev, "%s: Timer expired!", __FILE__);
		break;
	case 7:
		/* No Battery */
		chrgr->state.no_battery = 1;
		chrgr->state.health = POWER_SUPPLY_HEALTH_DEAD;
		chrgr->state.status = FAN54015_STATUS_FAULT;
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_NO_BAT, 0, 0);
		dev_err(&chrgr->client->dev, "%s: No battery!", __FILE__);
		break;
	default:
		/* No Fault */
		break;
	}

	if (health_prev != chrgr->state.health &&
		chrgr->state.health != POWER_SUPPLY_HEALTH_GOOD &&
		 chrgr->state.health != POWER_SUPPLY_HEALTH_UNKNOWN)
				power_supply_changed(chrgr->current_psy);

	up(&chrgr->prop_lock);

	return;
}

/**
 * Thread context charger interrupt handler
 *
 * @irq		IRQ number
 * @dev		pointer to device handle
 * @return	IRQ_HANDLED if the interrupt is CHGINT, else IRQ_NONE
 */
static irqreturn_t fan54015_pmic_chrgr_cb_work_func(int irq, void *dev)
{
	struct fan54015_charger *chrgr = (struct fan54015_charger *) dev;

	if (irq != chrgr->irq_chgintb) {
		dev_err(&chrgr->client->dev, "%s - wrong irq, irq=%d\n", __func__, irq);
		return IRQ_NONE;
	}

	unfreezable_bh_schedule(&chrgr->chgerr_bh);

	return IRQ_HANDLED;
}


/**
 * Configure charger PMIC interrupt
 *
 * @chrgr	pointer to charger driver internal structure
 * @return	0 if successful, else error code
 */
static int fan54015_configure_pmic_irq(struct fan54015_charger *chrgr)
{
	int ret = 0;
	u8 mask = 0;

	if (!IS_ERR_VALUE(chrgr->irq_chgintb)) {
	        ret = devm_request_threaded_irq(&chrgr->client->dev,
					chrgr->irq_chgintb, NULL,
					fan54015_pmic_chrgr_cb_work_func,
					IRQF_SHARED | IRQF_ONESHOT,
					FAN54015_NAME, chrgr);

		if (ret != 0) {

			dev_err(&chrgr->client->dev, "setup irq %d failed: %d\n",
					chrgr->irq_chgintb, ret);
			return -EINVAL;
		}
	}

	/* Clear Pending CHGINTB interrupt */
	mask = 0;
	set_reg_field(mask, CHGINTB_O, 1, SET);
	ret = fan54015_pmic_reg_write(DEV1, CHGRIRQ0, mask);
	if (ret)
		return -EIO;

	/* Unmask CHGINTB interrupts */
	mask = 0;
	set_reg_field(mask, CHGINTB_O, 1, SET);
	ret = fan54015_pmic_reg_set(DEV1, MCHGRIRQ0, mask, 0);
	if (ret != 0)
		return -EIO;

	/* Unmask LVL1 CHGR interrupt */
	set_reg_field(mask, CHGR_O, 1, SET);
	ret = fan54015_pmic_reg_set(DEV1, MIRQLVL1, mask, 0);

	return ret;
}

static int fan54015_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct usb_phy *otg_handle;
	struct fan54015_charger *chrgr = &fan54015_chrgr_data ;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	int ret;
	u8 ic_info;

	INIT_CHARGER_DEBUG_ARRAY(chrgr_dbg);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_PROBE, 0, 0);
	dev_err(&client->dev, "%s Entry.\n",__func__);

	/* Take control for SW charging */
	ret = fan54015_sw_control_overide();
	if (ret != 0)
		return -EIO;

	i2c_set_clientdata(client, chrgr);

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable, Abort.\n");
		return -ENODEV;
	}

	/* Read charger HW ID and check if it's supported */
	ret = fan54015_i2c_reg_read(client, REG_IC_INFO, &ic_info);
	if (ret != 0)
		return -ENODEV;

	if (get_reg_field(ic_info, VC_O, VC_W) != FAN54015_VENDOR ||
			get_reg_field(ic_info, PN_O, PN_W) != FAN54015_PN) {
		dev_err(&client->dev, "%s - IC is not supported. ic_info=0x%X\n",
				__func__, ic_info);
		return -ENODEV;
	}

	chrgr->client = client;

	/* Get USB phy */
	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (IS_ERR_OR_NULL(otg_handle)) {
		dev_err(&client->dev, "ERROR!: getting OTG transceiver failed\n");
		return -EINVAL;
	}
	chrgr->otg_handle = otg_handle;


	/* Get interrupt from device tree */
	chrgr->irq_chgintb = irq_of_parse_and_map(np, 0);

	if (!chrgr->irq_chgintb) {
		ret = -EINVAL;
		dev_err(&client->dev, "can't get irq, irq_chgintb=%d\n", chrgr->irq_chgintb);
		goto remap_fail;
	}
	dev_err(&client->dev, "%s, irq_chgintb=%d\n", __func__, chrgr->irq_chgintb);

	/* Setup the PMU registers */

	/* Trigger charger watchdog */
	INIT_DELAYED_WORK(&chrgr->charging_work, fan54015_charging_worker);

	sema_init(&chrgr->prop_lock, 1);

	/* Set up the wake lock to prevent suspend when charging. */
	wake_lock_init(&chrgr->suspend_lock,
			WAKE_LOCK_SUSPEND,
			"fan54015_wake_lock");

	if (unfreezable_bh_create(&chrgr->chgerr_bh, "chrgr_err_wq",
			"fan54015_err_lock", fan54015_charger_error_handler)) {
		ret = -ENOMEM;
		goto wq_creation_fail;
	}

	ret = fan54015_set_default_targets(chrgr);
	if (ret != 0)
		goto fail;

	chrgr->usb_psy.name           = "usb_charger";
	chrgr->usb_psy.type           = POWER_SUPPLY_TYPE_USB;
	chrgr->usb_psy.properties     = fan54015_power_props;
	chrgr->usb_psy.num_properties = ARRAY_SIZE(fan54015_power_props);
	chrgr->usb_psy.get_property   = fan54015_charger_get_property;
	chrgr->usb_psy.set_property   = fan54015_charger_set_property;
	chrgr->usb_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->usb_psy.supplied_to = fan54015_supplied_to;
	chrgr->usb_psy.num_supplicants = ARRAY_SIZE(fan54015_supplied_to);
	chrgr->usb_psy.throttle_states = fan54015_dummy_throttle_states;
	chrgr->usb_psy.num_throttle_states =
				ARRAY_SIZE(fan54015_dummy_throttle_states);

	chrgr->current_psy = &chrgr->usb_psy;

	ret = power_supply_register(&client->dev, &chrgr->usb_psy);
	if (ret)
		goto fail;

	chrgr->ac_psy.name           = "ac_charger";
	chrgr->ac_psy.type           = POWER_SUPPLY_TYPE_MAINS;
	chrgr->ac_psy.properties     = fan54015_power_props;
	chrgr->ac_psy.num_properties = ARRAY_SIZE(fan54015_power_props);
	chrgr->ac_psy.get_property   = fan54015_charger_get_property;
	chrgr->ac_psy.set_property   = fan54015_charger_set_property;
	chrgr->ac_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->ac_psy.supplied_to = fan54015_supplied_to;
	chrgr->ac_psy.num_supplicants = ARRAY_SIZE(fan54015_supplied_to);
	chrgr->ac_psy.throttle_states = fan54015_dummy_throttle_states;
	chrgr->ac_psy.num_throttle_states =
				ARRAY_SIZE(fan54015_dummy_throttle_states);

	ret = power_supply_register(&client->dev, &chrgr->ac_psy);
	if (ret)
		goto fail_ac_registr;


	chrgr->ack_time = jiffies;

	if (chrgr->state.charging_enabled)
		schedule_delayed_work(&chrgr->charging_work, 0);

	ret = fan54015_configure_pmic_irq(chrgr);

	if (ret != 0)
		goto pmu_irq_fail;
	fan54015_setup_debugfs(chrgr, &chrgr_dbg);

	chrgr->state.status = FAN54015_STATUS_READY;

	dev_err(&client->dev, "%s - finish\n", __func__);
	return 0;

pmu_irq_fail:
	power_supply_unregister(&chrgr->ac_psy);
fail_ac_registr:
	power_supply_unregister(&chrgr->usb_psy);
fail:
	unfreezable_bh_destroy(&chrgr->chgerr_bh);
wq_creation_fail:
remap_fail:
	usb_put_phy(otg_handle);

	return ret;
}

static int __exit fan54015_i2c_remove(struct i2c_client *client)
{
	struct fan54015_charger *chrgr = i2c_get_clientdata(client);
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_REMOVE, 0, 0);

	power_supply_unregister(&chrgr->usb_psy);
	power_supply_unregister(&chrgr->ac_psy);
	wake_lock_destroy(&chrgr->suspend_lock);

	unfreezable_bh_destroy(&chrgr->chgerr_bh);

	cancel_delayed_work_sync(&chrgr->charging_work);
	if (chrgr->otg_handle)
		usb_put_phy(chrgr->otg_handle);

	/* Set pin control */
	fan54015_remove_debugfs_dir(chrgr);
	return 0;
}

/**
 * Called when the system is attempting to suspend.
 * If charging is in progress EBUSY is returned to abort the suspend and
 * an error is logged, as the wake lock should prevent the situation.
 *
 * @dev		[in] Pointer to the device.(not used)
 * @returns	EBUSY if charging is ongoing, else 0
 */
static int fan54015_suspend(struct device *dev)
{
	/* Unused parameter */
	struct fan54015_charger *chrgr = &fan54015_chrgr_data ;
	(void)dev;

	if (fan54015_chrgr_data .state.charging_enabled) {
		/* If charging is in progess, prevent suspend. */
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SUSPEND_ERROR, 0, 0);
		return -EBUSY;
	} else {
		/* Not charging - allow suspend. */
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SUSPEND_OK, 0, 0);
		if (device_may_wakeup(dev)) {
			dev_info(&chrgr->client->dev, "fan: enable wakeirq\n");
			enable_irq_wake(chrgr->irq_chgintb);
		}
		unfreezable_bh_suspend(&fan54015_chrgr_data .chgerr_bh);
	}
	return 0;
}

/**
 * Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * @returns	0
 */
static int fan54015_resume(struct device *dev)
{
	/* Unused parameter */
	struct fan54015_charger *chrgr = &fan54015_chrgr_data ;
	(void)dev;

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_RESUME, 0, 0);

	unfreezable_bh_resume(&fan54015_chrgr_data .chgerr_bh);

	if (device_may_wakeup(dev)) {
		dev_info(&chrgr->client->dev, "fan: disable wakeirq\n");
		disable_irq_wake(chrgr->irq_chgintb);
	}
	return 0;
}

const struct dev_pm_ops fan54015_pm = {
	.suspend = fan54015_suspend,
	.resume = fan54015_resume,
};

static const struct i2c_device_id fan54015_id[] = {
	{"fan54015", 0}, { }
};

MODULE_DEVICE_TABLE(i2c, fan54015_id);

static struct i2c_driver fan54015_i2c_driver = {
	.probe = fan54015_i2c_probe,
	.remove = fan54015_i2c_remove,
	.id_table       = fan54015_id,
	.driver = {
		.name = FAN54015_NAME,
		.owner = THIS_MODULE,
		.pm = &fan54015_pm,
	},
};

static int __init fan54015_init(void)
{
	int ret;
	ret = i2c_add_driver(&fan54015_i2c_driver);
	if (ret)
		return ret;
	return 0;
}


static void __exit fan54015_exit(void)
{
	i2c_del_driver(&fan54015_i2c_driver);
}

device_initcall_sync(fan54015_init);
module_exit(fan54015_exit);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Charger Driver for FAN54015 charger IC");
