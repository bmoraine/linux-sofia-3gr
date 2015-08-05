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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; If not, see <http://www.gnu.org/licenses/>.
 *
 */

#define BQ24261_NAME "bq24261_charger"
#define pr_fmt(fmt) BQ24261_NAME": "fmt

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
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/power/charger_debug.h>
#include <linux/usb/phy-intel.h>
#include <sofia/vmm_pmic.h>
#include <sofia/vmm_pmic-ext.h>

#define BYTE_MASK	0xFF

/* PMIC DEV addresses */
#define DEV1		0x4E
#define DEV3		0x5E

/* PMIC DEV1 registers */
#define CHGRIRQ0	0x09
#define MCHGRIRQ0	0x17
#define CHGINTB_O	0

#define IRQLVL1		0x02
#define MIRQLVL1	0x0E
#define CHGR_O		5

/* PMIC DEV3 registers */
#define CHGRCTRL0	0x16
#define CCSM_OFF_O	5
#define SWCONTROL_O	3

#define CHRLEDCTRL	0x1F
#define CHRLEDF_O	4
#define CHRLEDF_W	2
#define SWLEDON_O	1
#define CHRLEDFN_O	0

#define CHGDISCTRL	0x2F
#define CHGDISFN_O	6
#define CHGDISDRV_O	4
#define CHGDISOUT_O	0

/* BQ24261 vendor code and part number */
#define BQ24261_VENDOR	0x2
#define BQ24261_PN	0x0

/* BQ24261 charger IC registers */
#define REG0_STAT_CTRL	0x00
#define TMR_RST_O	7
#define EN_BOOST_O	6
#define STAT_O		4
#define STAT_W		2
#define EN_SHIPMODE_O	3
#define FAULT_O		0
#define FAULT_W		3

#define REG1_CONTROL	0x01
#define RESET_O		7
#define IN_LIMIT_O	4
#define IN_LIMIT_W	3
#define EN_STAT_O	3
#define TE_O		2
#define CE_O		1
#define HZ_MODE_O	0

#define REG2_VOLTAGE	0x02
#define VBREG_O		2
#define VBREG_W		6
#define MOD_FREQ_O	0
#define MOD_FREQ_W	2

#define REG3_IC_INFO	0x03
#define VENDOR_O	5
#define VENDOR_W	3
#define PN_O		3
#define PN_W		2
#define REV_O		0
#define REV_W		3

#define REG4_CURRENT	0x04
#define ICHRG_O		3
#define ICHRG_W		5
#define ITERM_O		0
#define ITERM_W		3

#define REG5_MINSYS	0x05
#define MINSYS_STATUS_O	7
#define VINDPM_STATUS_O	6
#define LOW_CHG_O	5
#define D_PLUS_MINUS_O	4
#define CD_STATUS_O	3
#define VINDPM_O	0
#define VINDPM_W	3

#define REG6_SAFETY	0x06
#define TMR2X_EN_O	7
#define TMR_O		5
#define TMR_W		2
#define BOOST_ILIM_O	4
#define TS_EN_O		3
#define TS_FAULT_O	1
#define TS_FAULT_W	2
#define VINDPM_OFF_O	0

/* Default targets and ranges */
#define DEFAULT_VBREG_MV	3600
#define DEFAULT_ICHRG_MA	1000
#define DEFAULT_IBUS_MA		100

#define VBREG_MIN_MV		3500
#define VBREG_MAX_MV		4440
#define VBREG_STEP_MV		20

#define ICHRG_MIN_MA		500
#define ICHRG_MAX_MA		3000
#define ICHRG_STEP_MA		100

#define CHRGR_WORK_DELAY (10*HZ)
#define BOOST_WORK_DELAY (10*HZ)
#define EVT_WAKELOCK_TIMEOUT (2*HZ)

/* Logging and debugging parameters */
#define LOG_LINE_LENGTH (64)
#define LINES_PER_PAGE (PAGE_SIZE/LOG_LINE_LENGTH)
#define DBGFS_REG_LEN 16

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

enum {
	CLEAR = 0,
	SET = 1,

	FAULT_VBUS_OVP = 1,
	FAULT_LOW_SUPPLY = 2,
	FAULT_THERMAL = 3,
	FAULT_BAT_TEMP = 4,
	FAULT_TIMER = 5,
	FAULT_BAT_OVP = 6,
	FAULT_NO_BAT = 7,

	COOLING_NORMAL = 0,
	COOLING_WARNING = 1,
	COOLING_ALERT = 2,
	COOLING_CRITICAL = 3,
};

enum chr_led_freq {
	LED_FREQ_QUARTER_HZ,
	LED_FREQ_HALF_HZ,
	LED_FREQ_ONE_HZ,
	LED_FREQ_TWO_HZ,
};

enum bq24261_safety_timer_limit {
	BQ24261_SAFETY_1_25_MIN,
	BQ24261_SAFETY_6_HOUR,
	BQ24261_SAFETY_9_HOUR,
	BQ24261_SAFETY_DISABLED,
};

enum charger_status {
	BQ24261_STATUS_UNKNOWN,
	BQ24261_STATUS_READY,
	BQ24261_STATUS_FAULT,
};

/*
 * @status		charger driver status
 * @cc			current output current [mA] set on HW
 * @max_cc		maximum output current [mA] that can be set on HW
 * @cooling_cc		output current [mA] set on HW during cooling operation
 * @cv			current output voltage [mV] set on HW
 * @iterm		HW charging termination current [mA]
 * @inlmt		input current limit [mA]
 * @health		charger chip health
 * @cable_type		type of currently attached cable
 * @charger_enabled	informs if charger is enabled for use
 * @charging_enabled	informs if charging is currently enabled or not
 * @to_enable_boost	indicates whether boost mode is to be enabled
 * @boost_enabled	indicates whether boost mode is enabled
 * @cooling_state	cooling device state
 * @vbus_ovp_fault	VBUS over voltage fault flag and boost mode OVP flag
 * @low_supply_fault	low supply connected fault flag and boost mode OCP flag
 * @thermal_fault	thermal fault flag
 * @bat_temp_fault	battery temperature fault flag
 * @timer_fault		timer fault flag
 * @bat_ovp_fault	battery over voltage fault flag
 * @no_bat_fault	no battery fault flag
 */
struct bq24261_state {
	enum charger_status status;
	int cc;
	int max_cc;
	int cooling_cc;
	int cv;
	int iterm;
	int inlmt;
	int health;
	int cable_type;
	bool charger_enabled;
	bool charging_enabled;
	bool to_enable_boost;
	bool boost_enabled;
	int cooling_state;
	unsigned int vbus_ovp_fault:1;
	unsigned int low_supply_fault:1;
	unsigned int thermal_fault:1;
	unsigned int bat_temp_fault:1;
	unsigned int timer_fault:1;
	unsigned int bat_ovp_fault:1;
	unsigned int no_bat_fault:1;
};

/*
 * @wq			pointer to work queue
 * @work		work struct
 * @evt_wakelock	wakelock acquired when bottom half is scheduled
 * @in_suspend		Suspend flag
 * @pending_evt		pending event for bottom half execution flag
 * @lock		spinlock protecting in_suspend and pending_evt flags
 */
struct	unfreezable_bh_struct {
	struct workqueue_struct *wq;
	struct work_struct work;
	struct wake_lock evt_wakelock;
	bool in_suspend;
	bool pending_evt;
	spinlock_t lock;
};

/*
 * @client		i2c client device pointer
 * @chgerr_bh		charger error handler bottom half
 * @boost_en_bh		boost enable bottom half
 * @charging_work	work providing charging heartbeat
 * @boost_work		work providing boost heartbeat
 * @otg_handle		pointer to USB OTG internal structure
 * @usb_psy		power supply instance struct for USB path
 * @ac_psy		power supply instance struct for AC path
 * @current_psy		pointer to psy representing current charging path.
 * @prop_lock		synchronization semaphore
 * @model_name		model name of charger chip
 * @manufacturer	manufacturer name of charger chip
 * @ack_time		last CONTINUE_CHARGING timestamp in jiffies
 * @otg_nb		OTG notifier block
 * @state		charger state structure
 * @irq			irq number for charger error interrupt
 * @debugfs_root_dir	debugfs bq24261 charger root directory
 * @reg_debug		pointer to bq24261 register debug structure
 */
struct bq24261_data {
	struct i2c_client *client;
	struct unfreezable_bh_struct chgerr_bh;
	struct unfreezable_bh_struct boost_en_bh;
	struct delayed_work charging_work;
	struct delayed_work boost_work;
	struct usb_phy *otg_handle;
	struct power_supply_cable_props cable_props;
	struct power_supply usb_psy;
	struct power_supply ac_psy;
	struct power_supply *current_psy;
	struct semaphore prop_lock;
	struct wake_lock suspend_lock;
	const char *model_name;
	const char *manufacturer;
	unsigned long ack_time;
	struct notifier_block otg_nb;
	struct bq24261_state state;
	int irq;
	struct dentry *debugfs_root_dir;
	struct reg_debug_data *reg_debug;
};

/*
 * @reg		charger register index
 * @pdata	pointer to charger driver data
 */
struct reg_debug_data {
	u8 reg;
	struct bq24261_data *pdata;
};

static struct charger_debug_data chrgr_dbg = {
};

static struct power_supply_throttle bq24261_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
	}
};

static char *bq24261_supplied_to[] = {
		"battery",
};

static enum power_supply_property bq24261_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

/**
 * PMIC register read function
 *
 * @dev		[in] PMIC slave device
 * @reg		[in] register address
 * @pval	[out] pointer to buffer storing the data read
 * @return	0 on success, else error code
 */
static int bq24261_pmic_reg_read(u32 dev, u32 reg, u8 *pval)
{
	u32 vmm_addr, data;
	int ret;

	if (!pval)
		return -EINVAL;

	vmm_addr = ((dev & BYTE_MASK) << 24) | (reg & BYTE_MASK);
	ret = vmm_pmic_reg_read(vmm_addr, &data);
	data &= BYTE_MASK;
	*pval = (u8) data;

	return ret;
}

/**
 * PMIC register write function
 *
 * @dev		[in] PMIC slave device
 * @reg		[in] register address
 * @val		[in] value to be written
 * @return	0 on success, else error code
 */
static int bq24261_pmic_reg_write(u32 dev, u32 reg, u8 val)
{
	u32 vmm_addr, data;

	vmm_addr = ((dev & BYTE_MASK) << 24) | (reg & BYTE_MASK);
	data = (u32) val;
	return vmm_pmic_reg_write(vmm_addr, data);
}

/**
 * PMIC register bit field set function (read-modify-write)
 *
 * @dev		[in] PMIC slave device
 * @reg		[in] register address
 * @mask	[in] register mask
 * @val		[in] value to be written
 * @return	0 on success, else error code
 */
static int bq24261_pmic_reg_set(u32 dev, u32 reg, u8 mask, u8 val)
{
	u32 vmm_addr;

	vmm_addr = ((dev & BYTE_MASK) << 24) | (reg & BYTE_MASK);
	return pmic_reg_set_field(vmm_addr, mask, val);
}

/**
 * bq24261 register read function
 *
 * @client	[in] pointer to i2c client
 * @reg		[in] register address
 * @pval	[out] pointer to buffer storing the data read
 * @return	0 on success, else error code
 */
static int bq24261_i2c_reg_read(struct i2c_client *client, u8 reg, u8 *pval)
{
	int ret;

	if (!client || !pval)
		return -EINVAL;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret >= 0) {
		*pval = (u8) ret;
		ret = 0;
		CHARGER_DEBUG_READ_REG(chrgr_dbg, reg, *pval);
	} else {
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_READ_ERROR, ret, reg);
		dev_err(&client->dev, "I2C RD error! reg=%d, ret=0x%02X\n",
			reg, ret);
	}
	return ret;
}

/**
 * bq24261 register write function
 *
 * @client	[in] pointer to i2c client
 * @reg		[in] register address
 * @data	[in] value to be written to register
 * @return	0 on success, else error code
 */
static int bq24261_i2c_reg_write(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;

	if (!client)
		return -EINVAL;

	ret = i2c_smbus_write_byte_data(client, reg, data);

	if (ret >= 0) {
		CHARGER_DEBUG_WRITE_REG(chrgr_dbg, reg, data);
		ret = 0;
	} else {
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_WRITE_ERROR, ret, reg);
		dev_err(&client->dev,
			"I2C WR error! reg=%d, data=0x%02X, ret=0x%02X\n",
			reg, data, ret);
	}
	return ret;
}

#ifdef CONFIG_DEBUG_FS

static int dbg_evt_open(struct inode *inode, struct file *file)
{
	if (!inode || !file)
		return -EINVAL;

	/* save private data (the address of test_mod) in file struct
	(will be used by read()) */
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t dbg_evt_read(struct file *filp, char __user *buf, size_t count,
				loff_t *f_pos)
{
	struct charger_debug_data *dbg_array;
	unsigned long time_stamp_jiffies, time_stamp_s;
	enum charger_debug_event event;
	const char *event_str;
	u32 cnt, read_idx;
	int prm, prm2;
	ssize_t retval;

	int i, chars_count, total_chars = 0;

	char log_line[LOG_LINE_LENGTH];

	if (!filp || !buf || !f_pos)
		return -EINVAL;

	/* obtaining private data saved by open method */
	dbg_array = (struct charger_debug_data *) filp->private_data;

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

static const struct file_operations bq24261_evt_dbg_fops = {
	.open = dbg_evt_open,
	.read = dbg_evt_read,
};

static int bq24261_dbg_regs_dump(struct seq_file *m, void *data)
{
	struct bq24261_data *pdata;
	u8 reg;
	int ret;

	if (!m || !data)
		return -EINVAL;

	pdata = (struct bq24261_data *) m->private;

	ret = bq24261_i2c_reg_read(pdata->client, REG0_STAT_CTRL, &reg);
	seq_printf(m, "REG0(STAT/CTRL):\tHEX=0x%02X\n", reg);

	ret = bq24261_i2c_reg_read(pdata->client, REG1_CONTROL, &reg);
	seq_printf(m, "REG1(CONTROL):\t\tHEX=0x%02X\n", reg);

	ret = bq24261_i2c_reg_read(pdata->client, REG2_VOLTAGE, &reg);
	seq_printf(m, "REG2(VOLTAGE):\t\tHEX=0x%02X\n", reg);

	ret = bq24261_i2c_reg_read(pdata->client, REG3_IC_INFO, &reg);
	seq_printf(m, "REG3(IC_INFO):\t\tHEX=0x%02X\n", reg);

	ret = bq24261_i2c_reg_read(pdata->client, REG4_CURRENT, &reg);
	seq_printf(m, "REG4(CURRENT):\t\tHEX=0x%02X\n", reg);

	ret = bq24261_i2c_reg_read(pdata->client, REG5_MINSYS, &reg);
	seq_printf(m, "REG5(MINSYS):\t\tHEX=0x%02X\n", reg);

	ret = bq24261_i2c_reg_read(pdata->client, REG6_SAFETY, &reg);
	seq_printf(m, "REG6(SAFETY):\t\tHEX=0x%02X\n", reg);
	return 0;
}

static int bq24261_dbg_regs_dump_open(struct inode *i, struct file *file)
{
	if (!i || !file)
		return -EINVAL;

	return single_open(file, bq24261_dbg_regs_dump, i->i_private);
}

static int bq24261_dbg_regs_open(struct inode *i, struct file *file)
{
	if (!i || !file)
		return -EINVAL;

	file->private_data = i->i_private;

	return 0;
}

static ssize_t bq24261_dbg_regs_read(struct file *fp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	ssize_t cnt;
	int ret;
	char str[DBGFS_REG_LEN];
	u8 data, reg;
	struct reg_debug_data *pdbgdata;
	struct bq24261_data *pdata;

	BUG_ON(!buf || !f_pos || !fp);

	pdbgdata = (struct reg_debug_data *) fp->private_data;
	reg = pdbgdata->reg;
	pdata = pdbgdata->pdata;
	if (*f_pos >= DBGFS_REG_LEN - 1)
		return 0;

	ret = bq24261_i2c_reg_read(pdata->client, reg, &data);

	if (ret)
		return -EIO;

	cnt = snprintf(str, DBGFS_REG_LEN, "0x%02X\n", data);

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

static ssize_t bq24261_dbg_regs_write(struct file *fp,	const char __user *buf,
					size_t count, loff_t *f_pos)
{
	u8 data, reg;
	int ret;
	char str[DBGFS_REG_LEN + 1];
	struct reg_debug_data *pdbgdata;
	struct bq24261_data *pdata;

	BUG_ON(!buf || !f_pos || !fp);

	pdbgdata = (struct reg_debug_data *) fp->private_data;
	reg = pdbgdata->reg;
	pdata = pdbgdata->pdata;

	if (*f_pos >= DBGFS_REG_LEN)
		return -EFAULT;
	if (*f_pos + count > DBGFS_REG_LEN)
		count = DBGFS_REG_LEN - *f_pos;
	if (copy_from_user(str + *f_pos, buf, count))
		return -EFAULT;
	*f_pos += count;

	snprintf(str, *f_pos, "%s\n", str);

	ret = kstrtou8(str, 0, &data);
	if (ret)
		return ret;

	ret = bq24261_i2c_reg_write(pdata->client, reg, data);
	if (ret)
		return ret;

	return count;
}

static const struct file_operations bq24261_dbg_regs_dump_fops = {
	.open = bq24261_dbg_regs_dump_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations bq24261_dbg_regs_fops = {
	.owner = THIS_MODULE,
	.open = bq24261_dbg_regs_open,
	.read = bq24261_dbg_regs_read,
	.write = bq24261_dbg_regs_write,
};


static int bq24261_dbg_state_show(struct seq_file *m, void *data)
{
	struct bq24261_data *pdata;
	unsigned long timestamp_jiffies, timestamp_s;

	(void)data;

	if (!m)
		return -EINVAL;

	pdata = (struct bq24261_data *) m->private;

	timestamp_jiffies = jiffies - INITIAL_JIFFIES;
	timestamp_s = timestamp_jiffies/HZ;
	seq_printf(m, "[%5lu.%3lu] :\n", timestamp_s,
				(timestamp_jiffies - timestamp_s*HZ));

	seq_printf(m, "cc = %d\n", pdata->state.cc);
	seq_printf(m, "max_cc = %d\n", pdata->state.max_cc);
	seq_printf(m, "cooling_cc = %d\n", pdata->state.cooling_cc);
	seq_printf(m, "cv = %d\n", pdata->state.cv);
	seq_printf(m, "iterm = %d\n", pdata->state.iterm);
	seq_printf(m, "inlmt = %d\n", pdata->state.inlmt);
	seq_printf(m, "health = %d\n", pdata->state.health);
	seq_printf(m, "cable_type = %d\n", pdata->state.cable_type);
	seq_printf(m, "charger_enabled = %d\n", pdata->state.charger_enabled);
	seq_printf(m, "charging_enabled = %d\n\n",
				pdata->state.charging_enabled);
	seq_printf(m, "to_enable_boost = %d\n", pdata->state.to_enable_boost);
	seq_printf(m, "boost_enabled = %d\n", pdata->state.boost_enabled);
	seq_printf(m, "cooling_state = %d\n\n", pdata->state.cooling_state);
	seq_printf(m, "vbus_ovp_fault = %u\n", pdata->state.vbus_ovp_fault);
	seq_printf(m, "low_supply_fault = %u\n", pdata->state.low_supply_fault);
	seq_printf(m, "thermal_fault = %u\n", pdata->state.thermal_fault);
	seq_printf(m, "bat_temp_fault = %u\n", pdata->state.bat_temp_fault);
	seq_printf(m, "timer_fault = %u\n\n", pdata->state.timer_fault);
	seq_printf(m, "bat_ovp_fault = %u\n", pdata->state.bat_ovp_fault);
	seq_printf(m, "no_bat_fault = %u\n\n", pdata->state.no_bat_fault);

	return 0;
}

static int bq24261_dbg_state_open(struct inode *i, struct file *file)
{
	if (!i || !file)
		return -EINVAL;

	return single_open(file, bq24261_dbg_state_show, i->i_private);
}

static const struct file_operations bq24261_dbg_state_fops = {
	.open = bq24261_dbg_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/**
 * Set up debugfs entries for bq24261 charger driver
 * @pdata	[in] pointer to charger driver internal structure
 * @dbg_data	[in] pointer to debug array containing events logs
 */
static void bq24261_setup_debugfs(struct bq24261_data *pdata,
				struct charger_debug_data *dbg_data)
{
	struct dentry *dbgfs_entry;
	struct device *pdev;
	struct reg_debug_data *preg_debug;
	int i;

	if (!pdata || !dbg_data)
		return;

	dbgfs_entry = debugfs_create_dir(BQ24261_NAME, NULL);

	if (!dbgfs_entry)
		return;

	pdev = &pdata->client->dev;

	pdata->debugfs_root_dir = dbgfs_entry;

	(void)debugfs_create_file("events", S_IRUGO,
			dbgfs_entry, dbg_data, &bq24261_evt_dbg_fops);

	(void)debugfs_create_file("states", S_IRUGO,
			dbgfs_entry, pdata, &bq24261_dbg_state_fops);

	dbgfs_entry = debugfs_create_dir("regs", dbgfs_entry);
	if (!dbgfs_entry)
		return;

	(void)debugfs_create_file("all", S_IRUGO,
			dbgfs_entry, pdata, &bq24261_dbg_regs_dump_fops);

	preg_debug = devm_kzalloc(pdev, sizeof(struct reg_debug_data)*7,
				GFP_KERNEL);

	if (!preg_debug)
		return;

	pdata->reg_debug = preg_debug;

	for (i = 0; i < 7; i++) {
		preg_debug[i].reg = i;
		preg_debug[i].pdata = pdata;
	}

	(void)debugfs_create_file("REG0_STAT_CTRL", S_IRUGO | S_IWUSR,
			dbgfs_entry, (void *) &preg_debug[0],
			&bq24261_dbg_regs_fops);
	(void)debugfs_create_file("REG1_CONTROL", S_IRUGO | S_IWUSR,
			dbgfs_entry, (void *) &preg_debug[1],
			&bq24261_dbg_regs_fops);
	(void)debugfs_create_file("REG2_VOLTAGE", S_IRUGO | S_IWUSR,
			dbgfs_entry, (void *) &preg_debug[2],
			&bq24261_dbg_regs_fops);
	(void)debugfs_create_file("REG3_IC_INFO", S_IRUGO | S_IWUSR,
			dbgfs_entry, (void *) &preg_debug[3],
			&bq24261_dbg_regs_fops);
	(void)debugfs_create_file("REG4_CURRENT", S_IRUGO | S_IWUSR,
			dbgfs_entry, (void *) &preg_debug[4],
			&bq24261_dbg_regs_fops);
	(void)debugfs_create_file("REG5_MINSYS", S_IRUGO | S_IWUSR,
			dbgfs_entry, (void *) &preg_debug[5],
			&bq24261_dbg_regs_fops);
	(void)debugfs_create_file("REG6_SAFETY", S_IRUGO | S_IWUSR,
			dbgfs_entry, (void *) &preg_debug[6],
			&bq24261_dbg_regs_fops);
	return;
}

/**
 * Recursively removes debugfs root directory of BQ24261 charger driver
 *
 * @pdata		[in] pointer to charger driver's internal structure
 */
static void bq24261_remove_debugfs_dir(struct bq24261_data *pdata)
{
	if (!pdata)
		return;

	debugfs_remove_recursive(pdata->debugfs_root_dir);

	devm_kfree(&pdata->client->dev, pdata->reg_debug);

	return;
}

#else

static inline void bq24261_setup_debugfs(struct bq24261_data *pdata,
				struct charger_debug_data *dbg_data)
{

}

static inline void bq24261_remove_debugfs_dir(struct bq24261_data *pdata)
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
 * @pdata	pointer to charger driver internal structure
 * @return	0 if successful, else error code
 */
static int bq24261_sw_control_override(struct bq24261_data *pdata)
{
	int ret;
	u8 mask = 0, val;

	if (!pdata || !pdata->client)
		return -EINVAL;

	/* Disable CCSM and enable SW control */
	ret = bq24261_pmic_reg_read(DEV3, CHGRCTRL0, &val);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to read CHGRCTRL0, err=%d\n",
			__func__, ret);
		return ret;
	}

	set_reg_field(val, CCSM_OFF_O, 1, SET);
	set_reg_field(val, SWCONTROL_O, 1, SET);

	ret = bq24261_pmic_reg_write(DEV3, CHGRCTRL0, val);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to write CHGRCTRL0, err=%d\n",
			__func__, ret);
		return ret;
	}

	/* SW control LED */
	val = 0;
	set_reg_field(mask, CHRLEDFN_O, 1, SET);
	set_reg_field(mask, SWLEDON_O, 1, SET);

	set_reg_field(val, CHRLEDFN_O, 1, SET);
	set_reg_field(val, SWLEDON_O, 1, CLEAR);

	ret = bq24261_pmic_reg_set(DEV3, CHRLEDCTRL, mask, val);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to write CHRLEDCTRL, err=%d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

/**
 * Set charging LED frequency
 *
 * @freq	frequency to be set
 * @return	0 if successful, else error code
 */
static int bq24261_set_chrled_freq(enum chr_led_freq freq)
{
	int ret;
	u8 mask = 0, val = 0;
	switch (freq) {
	case LED_FREQ_QUARTER_HZ:
		val = 0;
		break;
	case LED_FREQ_HALF_HZ:
		val = 1;
		break;
	case LED_FREQ_ONE_HZ:
		val = 2;
		break;
	case LED_FREQ_TWO_HZ:
		val = 3;
		break;
	default:
		return -EINVAL;
		break;
	}

	set_reg_field(mask, CHRLEDF_O, CHRLEDF_W, 0x3);
	set_reg_field(val, CHRLEDF_O, CHRLEDF_W, val);

	ret = bq24261_pmic_reg_set(DEV3, CHRLEDCTRL, mask, val);

	return ret;
}

/**
 * Trigger charger watchdog
 *
 * @pdata	pointer to charger driver internal structure
 * @return	0 if successful, else error code
 */
static int bq24261_trigger_wdt(struct bq24261_data *pdata)
{
	int ret = 0;
	u8 reg;

	if (!pdata || !pdata->client)
		return -EINVAL;

	ret = bq24261_i2c_reg_read(pdata->client, REG0_STAT_CTRL, &reg);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to read REG0, err=%d\n", __func__, ret);
		return ret;
	}
	set_reg_field(reg, TMR_RST_O, 1, SET);
	ret = bq24261_i2c_reg_write(pdata->client, REG0_STAT_CTRL, reg);
	if (ret)
		dev_err(&pdata->client->dev,
			"%s - fail to write REG0, err=%d\n", __func__, ret);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TRIGGERING_WTD, 0, 0);

	return ret;
}

/**
 * Enable charger IC via CHGDIS pin
 *
 * @pdata	pointer to charger driver internal structure
 * @enable	whether to enable / disable charger IC
 * @return	0 if successful, else error code
 */
static int bq24261_enable_ic(struct bq24261_data *pdata, bool enable)
{
	u8 val;
	int ret;

	if (!pdata || !pdata->client)
		return -EINVAL;

	ret = bq24261_pmic_reg_read(DEV3, CHGDISCTRL, &val);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to read CHGDISCTRL, err=%d\n",
			__func__, ret);
		return ret;
	}

	set_reg_field(val, CHGDISDRV_O, 1, SET);
	set_reg_field(val, CHGDISFN_O, 1, CLEAR);

	if (enable)
		set_reg_field(val, CHGDISOUT_O, 1, CLEAR);
	else
		set_reg_field(val, CHGDISOUT_O, 1, SET);

	ret = bq24261_pmic_reg_write(DEV3, CHGDISCTRL, val);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to write CHGDISCTRL, err=%d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

/**
 * Enable/disable charger IC HW end-of-charge
 *
 * @pdata	pointer to charger driver internal structure
 * @enable	to enable/disable HW EOC
 * @return	0 if successful, else error code
 */
static int bq24261_enable_hw_termination(struct bq24261_data *pdata,
					bool enable)
{
	u8 val;
	int ret;

	if (!pdata || !pdata->client)
		return -EINVAL;

	ret = bq24261_i2c_reg_read(pdata->client, REG1_CONTROL, &val);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to read REG1, err=%d\n", __func__, ret);
		return ret;
	}

	if (enable)
		set_reg_field(val, TE_O, 1, SET);
	else
		set_reg_field(val, TE_O, 1, CLEAR);

	set_reg_field(val, RESET_O, 1, CLEAR);

	ret = bq24261_i2c_reg_write(pdata->client, REG1_CONTROL, val);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to write REG1, err=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

/**
 * Enable/disbale charger IC HW battery temperature control
 *
 * @pdata	pointer to charger driver internal structure
 * @enable	to enable/disable HW battery temperature control
 * @return	0 if successful, else error code
 */
static int bq24261_enable_ts(struct bq24261_data *pdata, bool enable)
{
	u8 val;
	int ret;

	if (!pdata || !pdata->client)
		return -EINVAL;

	ret = bq24261_i2c_reg_read(pdata->client, REG6_SAFETY, &val);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to read REG6, err=%d\n", __func__, ret);
		return ret;
	}

	if (enable)
		set_reg_field(val, TS_EN_O, 1, SET);
	else
		set_reg_field(val, TS_EN_O, 1, CLEAR);

	ret = bq24261_i2c_reg_write(pdata->client, REG6_SAFETY, val);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to write REG6, err=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

/**
 * Configure charger safety timer (not the watchdog)
 *
 * @pdata	pointer to charger driver internal structure
 * @limit	duration of the charger safety timer
 * @return	0 if successful, else error code
 */
static int bq24261_set_safety_timer(struct bq24261_data *pdata,
				enum bq24261_safety_timer_limit limit)
{
	u8 val;
	int ret;

	if (!pdata || !pdata->client)
		return -EINVAL;

	ret = bq24261_i2c_reg_read(pdata->client, REG6_SAFETY, &val);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to read REG6, err=%d\n", __func__, ret);
		return ret;
	}

	set_reg_field(val, TMR_O, TMR_W, limit);

	ret = bq24261_i2c_reg_write(pdata->client, REG6_SAFETY, val);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to write REG6, err=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

/**
 * Configuring BQ24261 chip registers when it's in DEFAULT state. This function
 * disable the HW EOC, disable the safety timer (not the watchdog) and
 * disable the charger IC battery temperature functionality.
 *
 * @pdata	pointer to charger driver internal structure
 * @return	0 if successful, else error code
 */
static int bq24261_configure_charger(struct bq24261_data *pdata)
{
	int ret;

	if (!pdata)
		return -EINVAL;

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_CONFIGURE_CHIP, 0, 0);

	ret = bq24261_enable_hw_termination(pdata, false);
	if (ret)
		return ret;
	ret = bq24261_enable_ts(pdata, false);
	if (ret)
		return ret;
	ret = bq24261_set_safety_timer(pdata, BQ24261_SAFETY_DISABLED);

	return ret;
}

/**
 * Set charger target voltage
 *
 * @pdata	pointer to charger driver internal structure
 * @volt_to_set	target voltage to set
 * @volt_set	pointer to target voltage actually set to HW
 * @propagate	whether to propage to HW
 * @return	0 if successful, else error code
 */
static int bq24261_set_vbreg(struct bq24261_data *pdata,
				int volt_to_set, int *volt_set, bool propagate)
{
	u8 reg, vbreg_val;
	int ret = 0;

	if (!volt_set || !pdata || !pdata->client)
		return -EINVAL;

	volt_to_set = fit_in_range(volt_to_set, VBREG_MIN_MV, VBREG_MAX_MV);

	vbreg_val = (volt_to_set - VBREG_MIN_MV) / VBREG_STEP_MV;

	*volt_set = vbreg_val * VBREG_STEP_MV + VBREG_MIN_MV;

	if (propagate) {
		ret = bq24261_i2c_reg_read(pdata->client, REG2_VOLTAGE,
						&reg);
		if (ret) {
			dev_err(&pdata->client->dev,
				"%s - fail to read REG2, err=%d\n", __func__,
				ret);
			return ret;
		}

		set_reg_field(reg, VBREG_O, VBREG_W, vbreg_val);

		ret = bq24261_i2c_reg_write(pdata->client, REG2_VOLTAGE,
						reg);
		if (ret) {
			dev_err(&pdata->client->dev,
				"%s - fail to write REG2, err=%d\n", __func__,
				ret);
			return ret;
		}
	}

	return ret;
}

/**
 * Set charger input current limit
 *
 * @pdata	pointer to charger driver internal structure
 * @ibus_to_set	target input current limit to set
 * @ibus_set	pointer to input current limit actually set to HW
 * @return	0 if successful, else error code
 */
static int bq24261_set_in_limit(struct bq24261_data *pdata, int ibus_to_set,
				int *ibus_set)
{
	int ret;
	u8 reg, ibuslim_val;
	enum chr_led_freq freq;

	if (!ibus_set || !pdata || !pdata->client)
		return -EINVAL;

	if (ibus_to_set < 150) {
		ibuslim_val = 0x00;
		*ibus_set = 100;
		freq = LED_FREQ_QUARTER_HZ;
	} else if (ibus_to_set < 500) {
		ibuslim_val = 0x01;
		*ibus_set = 150;
		freq = LED_FREQ_QUARTER_HZ;
	} else if (ibus_to_set < 900) {
		ibuslim_val = 0x02;
		*ibus_set = 500;
		freq = LED_FREQ_HALF_HZ;
	} else if (ibus_to_set < 1500) {
		ibuslim_val = 0x03;
		*ibus_set = 900;
		freq = LED_FREQ_ONE_HZ;
	} else if (ibus_to_set < 1950) {
		ibuslim_val = 0x04;
		*ibus_set = 1500;
		freq = LED_FREQ_ONE_HZ;
	} else if (ibus_to_set < 2500) {
		ibuslim_val = 0x05;
		*ibus_set = 1950;
		freq = LED_FREQ_TWO_HZ;
	} else if (ibus_to_set < 3000) {
		ibuslim_val = 0x06;
		*ibus_set = 2500;
		freq = LED_FREQ_TWO_HZ;
	} else {
		ibuslim_val = 0x07;
		*ibus_set = 3000;
		freq = LED_FREQ_TWO_HZ;
	}

	ret = bq24261_i2c_reg_read(pdata->client, REG1_CONTROL,
					&reg);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to read REG1, err=%d\n", __func__, ret);
		return ret;
	}

	set_reg_field(reg, IN_LIMIT_O, IN_LIMIT_W, ibuslim_val);
	set_reg_field(reg, RESET_O, 1, CLEAR);

	ret = bq24261_i2c_reg_write(pdata->client, REG1_CONTROL, reg);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to write REG1, err=%d\n", __func__, ret);
		return ret;
	}

	ret = bq24261_set_chrled_freq(freq);
	if (ret) {
		dev_err(&pdata->client->dev,
			"%s - fail to set LED frequency, err=%d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

/**
 * Check whether charging is on-going on the specified charger
 *
 * @pdata	pointer to charger driver internal structure
 * @psy		pointer to power supply charger instance
 * @return	true if charging is on-going, else false
 */
static inline bool bq24261_is_online(struct bq24261_data *pdata,
						struct power_supply *psy)
{
	if (!pdata || !psy)
		return -EINVAL;

	if (!(pdata->state.health == POWER_SUPPLY_HEALTH_GOOD) ||
		!pdata->state.charger_enabled)
		return false;

	if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		return ((pdata->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_CDP) ||
			(pdata->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_DCP));

	else if (psy->type == POWER_SUPPLY_TYPE_USB)
		return ((pdata->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
			(pdata->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING));

	return false;
}

/**
 * Worker function to trigger watchdog and update PSY during charging
 *
 * @work	pointer to work instance
 */
static void bq24261_charging_worker(struct work_struct *work)
{
	int ret;
	struct bq24261_data *pdata =
		container_of(work, struct bq24261_data, charging_work.work);

	ret = bq24261_trigger_wdt(pdata);
	if (ret) {
		dev_err(&pdata->client->dev, "%s: Fail to clear WDT\n",
			__func__);
		return;
	}

	if (!time_after(jiffies, pdata->ack_time + (60*HZ))) {
		power_supply_changed(pdata->current_psy);
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TRIG_POWER_SUPPLY_CHARGER,
				0, 0);
	}

	schedule_delayed_work(&pdata->charging_work, CHRGR_WORK_DELAY);

	return;
}

/**
 * Worker function to trigger watchdog and update PSY during boost mode
 *
 * @work	pointer to work instance
 */
static void bq24261_boost_worker(struct work_struct *work)
{
	int ret;
	struct bq24261_data *pdata =
		container_of(work, struct bq24261_data, boost_work.work);

	if (pdata->state.boost_enabled) {
		ret = bq24261_trigger_wdt(pdata);
		if (ret) {
			dev_err(&pdata->client->dev, "%s: Fail to clear WDT\n",
				__func__);
			return;
		}
		schedule_delayed_work(&pdata->boost_work, BOOST_WORK_DELAY);
	}
}

/**
 * Worker function to enable/disable boost mode
 *
 * @work	pointer to work instance
 */
static void bq24261_enable_boost(struct work_struct *work)
{
	u8 reg0, reg1;
	struct bq24261_data *pdata =
		container_of(work, struct bq24261_data, boost_en_bh.work);
	int ret;
	bool to_enable = pdata->state.to_enable_boost;

	down(&pdata->prop_lock);

	pdata->state.vbus_ovp_fault = 0;
	pdata->state.low_supply_fault = 0;

	ret = bq24261_i2c_reg_read(pdata->client, REG0_STAT_CTRL, &reg0);
	if (ret) {
		dev_err(&pdata->client->dev, "%s: Fail to read REG0\n",
			__func__);
		goto exit_boost1;
	}

	ret = bq24261_i2c_reg_read(pdata->client, REG1_CONTROL, &reg1);
	if (ret) {
		dev_err(&pdata->client->dev, "%s: Fail to read REG1\n",
			__func__);
		goto exit_boost1;
	}

	if (to_enable) {
		set_reg_field(reg0, EN_BOOST_O, 1, SET);
		set_reg_field(reg1, HZ_MODE_O, 1, CLEAR);
		wake_lock(&pdata->suspend_lock);
		schedule_delayed_work(&pdata->boost_work, BOOST_WORK_DELAY);
	} else {
		set_reg_field(reg0, EN_BOOST_O, 1, CLEAR);
		set_reg_field(reg1, HZ_MODE_O, 1, SET);
		cancel_delayed_work(&pdata->boost_work);
		wake_unlock(&pdata->suspend_lock);
	}

	set_reg_field(reg1, RESET_O, 1, CLEAR);

	ret = bq24261_i2c_reg_write(pdata->client, REG0_STAT_CTRL, reg0);
	if (ret) {
		dev_err(&pdata->client->dev, "%s: Fail to write REG0\n",
			__func__);
		goto exit_boost2;
	}

	ret = bq24261_i2c_reg_write(pdata->client, REG1_CONTROL, reg1);
	if (ret) {
		dev_err(&pdata->client->dev, "%s: Fail to write REG1\n",
			__func__);
		goto exit_boost2;
	}

	pdata->state.boost_enabled = to_enable;

	if (to_enable)
		dev_info(&pdata->client->dev, "Boost mode enabled\n");
	else
		dev_info(&pdata->client->dev, "Boost mode disabled\n");

	up(&pdata->prop_lock);
	return;

exit_boost2:
	if (to_enable) {
		cancel_delayed_work(&pdata->boost_work);
		wake_unlock(&pdata->suspend_lock);
	}
exit_boost1:
	pdata->state.boost_enabled = false;
	pdata->state.to_enable_boost = false;
	up(&pdata->prop_lock);
}

/**
 * Function to enable / disable charging
 *
 * @pdata	pointer to charger driver internal structure
 * @enable	whether enable or disable charging
 * @return	0 if successful, else error code
 */
static int bq24261_enable_charging(struct bq24261_data *pdata, bool enable)
{
	u8 val, mask = 0;
	int ret;
	struct device *pdev;

	if (!pdata || !pdata->client)
		return -EINVAL;

	pdev = &pdata->client->dev;

	if (enable) {
		ret = bq24261_trigger_wdt(pdata);
		if (ret)
			return ret;
	}

	ret = bq24261_i2c_reg_read(pdata->client, REG1_CONTROL, &val);
	if (ret) {
		dev_err(pdev, "%s - fail to read REG1, err=%d\n", __func__,
			ret);
		return ret;
	}

	if (enable) {
		set_reg_field(val, HZ_MODE_O, 1, CLEAR);
		set_reg_field(val, CE_O, 1, CLEAR);
	} else {
		set_reg_field(val, HZ_MODE_O, 1, SET);
		set_reg_field(val, CE_O, 1, SET);
	}
	set_reg_field(val, RESET_O, 1, CLEAR);

	ret = bq24261_i2c_reg_write(pdata->client, REG1_CONTROL, val);
	if (ret) {
		dev_err(pdev, "%s - fail to write REG1, err=%d\n", __func__,
			ret);
		return ret;
	}

	set_reg_field(mask, SWLEDON_O, 1, SET);
	val = 0;

	if (enable)
		set_reg_field(val, SWLEDON_O, 1, SET);
	else
		set_reg_field(val, SWLEDON_O, 1, CLEAR);

	ret = bq24261_pmic_reg_set(DEV3, CHRLEDCTRL, mask, val);
	if (ret) {
		dev_err(pdev, "%s - fail to write CHRLEDCTRL, err=%d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

/**
 * Set charger target current
 *
 * @pdata	pointer to charger driver internal structure
 * @curr_to_set	target current to set
 * @curr_set	pointer to current target actually set to HW
 * @propagate	whether to propage to HW
 * @return	0 if successful, else error code
 */
static int bq24261_set_ichrg(struct bq24261_data *pdata,
			unsigned int curr_to_set, int *curr_set, bool propagate)
{
	u8 iochr_val, reg;
	int ret = 0;

	if (!curr_set || !pdata || !pdata->client)
		return -EINVAL;

	if (curr_to_set < ICHRG_MIN_MA) {
		if (pdata->state.charging_enabled && propagate) {
			ret = bq24261_enable_charging(pdata, false);
			if (ret != 0)
				return ret;
			pdata->state.charging_enabled = 0;
			dev_info(&pdata->client->dev,
				"Output %dmA is too low, disable charging",
				curr_to_set);
		}
		*curr_set = 0;
		return 0;
	}

	curr_to_set = fit_in_range(curr_to_set, ICHRG_MIN_MA, ICHRG_MAX_MA);

	iochr_val = (curr_to_set - ICHRG_MIN_MA) / ICHRG_STEP_MA;

	*curr_set = iochr_val * ICHRG_STEP_MA + ICHRG_MIN_MA;

	if (propagate) {
		ret = bq24261_i2c_reg_read(pdata->client, REG4_CURRENT, &reg);
		if (ret) {
			dev_err(&pdata->client->dev,
				"%s - fail to read REG4, err=%d\n", __func__,
				ret);
			return ret;
		}

		set_reg_field(reg, ICHRG_O, ICHRG_W, iochr_val);

		ret = bq24261_i2c_reg_write(pdata->client, REG4_CURRENT, reg);
		if (ret) {
			dev_err(&pdata->client->dev,
				"%s - fail to write REG4, err=%d\n", __func__,
				ret);
			return ret;
		}
	}

	return ret;
}

/**
 * Calculate the output current according to requested output current and
 * cooling level
 *
 * @pdata	pointer to charger driver internal structure
 * @cc		output current requested
 * @level	cooling level requested
 * @return	non-negative current, else error code
 */
static int bq24261_calc_cooling_current(struct bq24261_data *pdata, int cc,
					int level)
{
	struct power_supply_throttle *pthrottle;
	unsigned int num_levels = COOLING_CRITICAL + 1;

	if (!pdata)
		return -EINVAL;

	switch (level) {
	case COOLING_NORMAL:
	case COOLING_WARNING:
	case COOLING_ALERT:
	case COOLING_CRITICAL:
		cc = cc * (num_levels - level) / num_levels;
		break;

	default:
		return -EINVAL;
	}

	pthrottle = pdata->usb_psy.throttle_states + level;

	if (cc < ICHRG_MIN_MA)
		pthrottle->throttle_action = PSY_THROTTLE_DISABLE_CHARGING;
	else
		pthrottle->throttle_action = PSY_THROTTLE_CC_LIMIT;

	if (pdata->state.cooling_state != level)
		power_supply_changed(pdata->current_psy);

	return cc;
}

/**
 * Cooling request handler
 *
 * @pdata	pointer to charger driver internal structure
 * @level	cooling level requested
 * @return	0 if successful, else error code
 */
static int bq24261_cooling_action(struct bq24261_data *pdata, int level)
{
	struct device *pdev;
	int cc, cc_set, cur_level, ret;

	if (!pdata || !pdata->client)
		return -EINVAL;

	pdev = &pdata->client->dev;
	cur_level = pdata->state.cooling_state;

	if (cur_level == level)
		return 0;

	cc = bq24261_calc_cooling_current(pdata, pdata->state.cc, level);

	if (cc < 0) {
		dev_err(pdev, "Unknown cooling level - %d\n", level);
		return -EINVAL;
	}

	ret = bq24261_set_ichrg(pdata, cc, &cc_set, true);

	if (ret) {
		dev_err(pdev, "%s - fail to set cc, ret=%d\n", __func__, ret);
		return ret;
	}

	dev_info(pdev, "cooling state %d, output current %dmA\n", level,
		cc_set);

	pdata->state.cooling_cc = cc_set;
	pdata->state.cooling_state = level;

	return 0;
}

/**
 * Power supply charger property set function
 *
 * @pdata	pointer to charger driver internal structure
 * @psy		pointer to power supply class charger instance
 * @psp		property to set
 * @val		pointer to value to set
 * @return	0 if successful, else error code
 */
static int bq24261_charger_set_property(struct bq24261_data *pdata,
					struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	int value_to_set, value_set = 0, ret = 0;

	bool call_psy_changed = false;

	if (!pdata || !pdata->client || !psy || !val)
		return -EINVAL;

	down(&pdata->prop_lock);

	if (pdata->state.status == BQ24261_STATUS_FAULT) {
		ret = -EFAULT;
		CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_SET_PROPERTY_ERROR, ret, 0);

		CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_DRIVER_IN_FAULT_STATE, 0, 0);
		up(&pdata->prop_lock);
		return ret;
	}

	switch (psp) {

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		CHARGER_DEBUG_REL(
			chrgr_dbg, CHG_DBG_SET_PROP_CABLE_TYPE, val->intval, 0);

		value_set = val->intval;
		if (pdata->state.cable_type == val->intval)
			break;
		pdata->state.cable_type = val->intval;


		if ((pdata->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
			(pdata->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING)) {

			pdata->current_psy = &pdata->usb_psy;
			pdata->state.health = POWER_SUPPLY_HEALTH_GOOD;
			ret = bq24261_configure_charger(pdata);
			if (ret)
				dev_err(&pdata->client->dev,
					"%s - fail to config IC\n",
					__func__);
			wake_lock(&pdata->suspend_lock);
			schedule_delayed_work(&pdata->charging_work, 0);
		} else if ((pdata->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_CDP) ||
			(pdata->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_USB_DCP)) {

			pdata->current_psy = &pdata->ac_psy;
			pdata->state.health = POWER_SUPPLY_HEALTH_GOOD;
			ret = bq24261_configure_charger(pdata);
			if (ret)
				dev_err(&pdata->client->dev,
					"%s - fail to config IC\n",
					__func__);
			wake_lock(&pdata->suspend_lock);
			schedule_delayed_work(&pdata->charging_work, 0);
		} else if (pdata->state.cable_type ==
			POWER_SUPPLY_CHARGER_TYPE_NONE) {
			wake_unlock(&pdata->suspend_lock);
			cancel_delayed_work(&pdata->charging_work);
			pdata->state.health = POWER_SUPPLY_HEALTH_UNKNOWN;
		}

		call_psy_changed = true;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_CURRENT,
								val->intval, 0);

		value_to_set = fit_in_range(val->intval, 0,
						pdata->state.max_cc);

		value_to_set = bq24261_calc_cooling_current(pdata, value_to_set,
						pdata->state.cooling_state);
		if (value_to_set < 0)
			break;

		bq24261_set_ichrg(pdata, value_to_set,
					&value_set, false);
		if (pdata->state.cooling_state == COOLING_NORMAL) {
			if (value_set == pdata->state.cc)
				break;
		} else {
			if (value_set == pdata->state.cooling_cc)
				break;
		}

		ret = bq24261_set_ichrg(pdata, value_to_set,
						&value_set, true);

		if (ret) {
			pdata->state.status = BQ24261_STATUS_FAULT;
			pdata->state.health =
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}

		pdata->state.cooling_cc = value_set;

		if (pdata->state.cooling_state == COOLING_NORMAL)
			pdata->state.cc = value_set;
		else
			pdata->state.cc = fit_in_range(val->intval, 0,
						pdata->state.max_cc);

		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_VOLTAGE,
								val->intval, 0);

		bq24261_set_vbreg(pdata, val->intval, &value_set, false);
		if (value_set == pdata->state.cv)
			break;
		ret = bq24261_set_vbreg(pdata, val->intval, &value_set, true);
		if (ret) {
			pdata->state.status = BQ24261_STATUS_FAULT;
			pdata->state.health =
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}

		pdata->state.cv = value_set;
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_ENABLE_CHARGER,
								val->intval, 0);

		pdata->state.charger_enabled = val->intval;
		value_set = val->intval;
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_ENABLE_CHARGING,
								val->intval, 0);

		if (val->intval == pdata->state.charging_enabled) {
			value_set = val->intval;
			break;
		}
		ret = bq24261_enable_charging(pdata, val->intval);
		if (!ret) {
			pdata->state.charging_enabled = val->intval;
			value_set = val->intval;
			call_psy_changed = true;
		}
		break;

	case POWER_SUPPLY_PROP_INLMT:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_INLMT,
							val->intval, 0);

		value_to_set = val->intval;
		ret = bq24261_set_in_limit(pdata, value_to_set, &value_set);
		if (ret) {
			pdata->state.status = BQ24261_STATUS_FAULT;
			pdata->state.health =
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}

		pdata->state.inlmt = value_set;
		break;

	case POWER_SUPPLY_PROP_CONTINUE_CHARGING:
		CHARGER_DEBUG_REL(
				chrgr_dbg, CHG_DBG_SET_PROP_CONTINUE_CHARGING,
								val->intval, 0);

		pdata->ack_time = jiffies;
		value_set = val->intval;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_TERM_CUR,
								val->intval, 0);
		pdata->state.iterm = value_set;
		break;

	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SET_PROP_MIN_TEMP,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		bq24261_cooling_action(pdata, val->intval);
		CHARGER_DEBUG_DEV(
			chrgr_dbg, CHG_DBG_SET_PROP_CHARGE_CONTROL_LIMIT,
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

	up(&pdata->prop_lock);

	if (call_psy_changed)
		power_supply_changed(psy);

	return ret;
}

/**
 * Wrapper for USB power supply charger property set function
 *
 * @psy		pointer to power supply class charger instance
 * @psp		property to set
 * @val		pointer to value to set
 * @return	0 if successful, else error code
 */
static int bq24261_usb_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct bq24261_data *pdata =
			container_of(psy, struct bq24261_data, usb_psy);

	return bq24261_charger_set_property(pdata, psy, psp, val);
}

/**
 * Wrapper for AC power supply charger property set function
 *
 * @psy		pointer to power supply class charger instance
 * @psp		property to set
 * @val		pointer to value to set
 * @return	0 if successful, else error code
 */
static int bq24261_ac_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct bq24261_data *pdata =
			container_of(psy, struct bq24261_data, ac_psy);

	return bq24261_charger_set_property(pdata, psy, psp, val);
}

/**
 * Power supply charger property get function
 *
 * @pdata	pointer to charger driver internal structure
 * @psy		pointer to power supply class charger instance
 * @psp		property to get
 * @val		pointer to buffer to store value get
 * @return	0 if successful, else error code
 */
static int bq24261_charger_get_property(struct bq24261_data *pdata,
					struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;

	if (!pdata || !pdata->client || !psy || !val)
		return -EINVAL;

	down(&pdata->prop_lock);

	switch (psp) {

	case POWER_SUPPLY_PROP_PRESENT:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = ((pdata->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
				(pdata->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING));

		else if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = ((pdata->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_DCP) ||
				(pdata->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_CDP));
		else
			val->intval = 0;

		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_PRESENT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq24261_is_online(pdata, psy);
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_ONLINE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = pdata->state.health;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_HEALTH,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_USB;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_TYPE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		val->intval = pdata->state.cable_type;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_CABLE_TYPE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		val->intval = pdata->state.charger_enabled;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_ENABLE_CHARGER,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		val->intval = pdata->state.charging_enabled;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_ENABLE_CHARGING,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = pdata->state.cooling_state;
		CHARGER_DEBUG_DEV(
			chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_CONTROL_LIMIT,
								val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = pdata->usb_psy.num_throttle_states;
		CHARGER_DEBUG_DEV(
			chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_CONTROL_LIMIT_MAX,
								val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = pdata->model_name;
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = pdata->manufacturer;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = pdata->state.cc;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_CURRENT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = pdata->state.cv;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_CHARGE_VOLTAGE,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = pdata->state.max_cc;
		CHARGER_DEBUG_DEV(
			chrgr_dbg, CHG_DBG_GET_PROP_MAX_CHARGE_CURRENT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_INLMT:
		val->intval = pdata->state.inlmt;
		CHARGER_DEBUG_DEV(chrgr_dbg, CHG_DBG_GET_PROP_INLMT,
								val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		val->intval = pdata->state.iterm;
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

	up(&pdata->prop_lock);

	return ret;
}

/**
 * Wrapper for USB power supply charger property get function
 *
 * @psy		pointer to power supply class charger instance
 * @psp		property to get
 * @val		pointer to buffer storing value to get
 * @return	0 if successful, else error code
 */
static int bq24261_usb_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq24261_data *pdata =
			container_of(psy, struct bq24261_data, usb_psy);

	return bq24261_charger_get_property(pdata, psy, psp, val);
}

/**
 * Wrapper for AC power supply charger property get function
 *
 * @psy		pointer to power supply class charger instance
 * @psp		property to get
 * @val		pointer to buffer storing value to get
 * @return	0 if successful, else error code
 */
static int bq24261_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq24261_data *pdata =
			container_of(psy, struct bq24261_data, ac_psy);

	return bq24261_charger_get_property(pdata, psy, psp, val);
}

/**
 * Set default target targets
 *
 * @psy		pointer to power supply class charger instance
 * @return	0 if successful, else error code
 */
static int bq24261_set_default_targets(struct bq24261_data *pdata)
{
	int ret, val;

	if (!pdata)
		return -EINVAL;

	ret = bq24261_set_in_limit(pdata, DEFAULT_IBUS_MA, &val);
	if (ret)
		return ret;

	ret = bq24261_set_ichrg(pdata, DEFAULT_ICHRG_MA, &val, true);
	if (ret)
		return ret;

	ret = bq24261_set_vbreg(pdata, DEFAULT_VBREG_MV, &val, true);
	if (ret)
		return ret;

	return ret;
}

/**
 * Worker function process charger error
 *
 * @work	pointer to work structrure
 * @return	0 if successful, else error code
 */
static void bq24261_charger_error_handler(struct work_struct *work)
{
	int  health_prev, ret = 0;
	u8 val, reg = 0;
	bool boost_fault = false;
	struct bq24261_data *pdata = container_of(work,
				struct bq24261_data, chgerr_bh.work);
	struct device *pdev = &pdata->client->dev;

	ret = bq24261_i2c_reg_read(pdata->client, REG0_STAT_CTRL, &reg);
	if (ret) {
		dev_err(pdev, "%s: Fail to read REG0\n", __func__);
		return;
	}

	val = get_reg_field(reg, STAT_O, STAT_W);

	down(&pdata->prop_lock);
	health_prev = pdata->state.health;

	if (val != 0x3) {
		pdata->state.vbus_ovp_fault = 0;
		pdata->state.low_supply_fault = 0;
		pdata->state.thermal_fault = 0;
		pdata->state.bat_temp_fault = 0;
		pdata->state.timer_fault = 0;
		pdata->state.bat_ovp_fault = 0;
		pdata->state.no_bat_fault = 0;

		up(&pdata->prop_lock);
		return;
	}

	val = get_reg_field(reg, FAULT_O, FAULT_W);

	switch (val) {
	case FAULT_VBUS_OVP:
		/* VBUS OVP fault */
		pdata->state.vbus_ovp_fault = 1;
		pdata->state.health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_VBUS_FAULT, 0, 0);
		if (pdata->state.boost_enabled) {
			dev_err(pdev, "%s - Boost mode over voltage!\n",
				__func__);
			boost_fault = true;
		} else {
			dev_err(pdev, "%s - VBUS over voltage!\n", __func__);
		}
		break;

	case FAULT_LOW_SUPPLY:
		if (pdata->state.boost_enabled) {
			dev_err(pdev, "%s - Boost mode over current!\n",
				__func__);
			boost_fault = true;
		} else {
			/* Low supply connected */
			dev_dbg(pdev,
				"%s - Low supply connected (not fault)!\n",
				__func__);
		}
		break;

	case FAULT_THERMAL:
		/* Thermal Shutdown */
		pdata->state.thermal_fault = 1;
		pdata->state.health = POWER_SUPPLY_HEALTH_OVERHEAT;
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_TSD_IS_ON, 0, 0);
		dev_err(pdev, "%s - Chip Over-temperature Shutdown!\n",
			__func__);
		break;

	case FAULT_BAT_TEMP:
		/* Battery temperature fault */
		pdata->state.bat_temp_fault = 1;
		dev_err(pdev, "%s - Battery temperature fault!\n", __func__);
		break;

	case FAULT_TIMER:
		/* Timer Fault */
		if (pdata->state.charger_enabled &&
			pdata->state.charging_enabled) {
			pdata->state.health = POWER_SUPPLY_HEALTH_DEAD;
			pdata->state.status = BQ24261_STATUS_FAULT;
			CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_T32_TIMER_EXPIRED,
					 0, 0);

			pdata->state.timer_fault = 1;
			dev_err(pdev, "%s - Timer expired!\n", __func__);
		}
		break;

	case FAULT_BAT_OVP:
		/* Battery OVP (VBAT > CVT). It's not a fault when charging is
		not enabled */
		if (pdata->state.charger_enabled &&
			pdata->state.charging_enabled) {
			pdata->state.bat_ovp_fault = 1;
			dev_err(pdev, "%s - Battery over voltage!\n", __func__);
		}
		break;

	case FAULT_NO_BAT:
		/* No Battery */
		pdata->state.bat_ovp_fault = 1;
		pdata->state.health = POWER_SUPPLY_HEALTH_DEAD;
		pdata->state.status = BQ24261_STATUS_FAULT;
		dev_err(pdev, "%s - No battery!", __func__);
		break;

	default:
		break;
	}

	if (boost_fault) {
		/* Notify USB about the VBUS error, payload is dummy */
		atomic_notifier_call_chain(&pdata->otg_handle->notifier,
					INTEL_USB_DRV_VBUS_ERR, &ret);
		pdata->state.boost_enabled = false;
	} else if (health_prev != pdata->state.health &&
		pdata->state.health != POWER_SUPPLY_HEALTH_GOOD &&
		 pdata->state.health != POWER_SUPPLY_HEALTH_UNKNOWN) {
			power_supply_changed(pdata->current_psy);
	}

	up(&pdata->prop_lock);

	return;
}

/**
 * Thread context charger interrupt handler
 *
 * @irq		IRQ number
 * @dev		pointer to device handle
 * @return	IRQ_HANDLED if the interurpt is CHGINT, else IRQ_NONE
 */
static irqreturn_t bq24261_pmic_pdata_cb_work_func(int irq, void *dev)
{
	struct bq24261_data *pdata = (struct bq24261_data *) dev;

	if (!pdata || !pdata->client)
		return IRQ_NONE;

	if (irq != pdata->irq) {
		dev_err(&pdata->client->dev,
			"%s - wrong irq, irq=%d\n", __func__, irq);
		return IRQ_NONE;
	}

	unfreezable_bh_schedule(&pdata->chgerr_bh);

	return IRQ_HANDLED;
}


/**
 * Configure charger PMIC interrupt
 *
 * @work	pointer to work structrure
 * @return	0 if successful, else error code
 */
static int bq24261_configure_pmic_irq(struct bq24261_data *pdata)
{
	int ret = 0;
	u8 mask = 0;
	struct device *pdev;

	if (!pdata || !pdata->client)
		return -EINVAL;

	pdev = &pdata->client->dev;

	/* Get interrupt from device tree */
	pdata->irq = irq_of_parse_and_map(pdev->of_node, 0);

	if (!pdata->irq) {
		dev_err(pdev, "%s - fail to get irq %d\n", __func__,
				pdata->irq);
		return -EINVAL;
	}

	if (!IS_ERR_VALUE(pdata->irq)) {
		ret = devm_request_threaded_irq(&pdata->client->dev,
					pdata->irq, NULL,
					bq24261_pmic_pdata_cb_work_func,
					IRQF_ONESHOT | IRQF_NO_SUSPEND,
					BQ24261_NAME, pdata);

		if (ret) {
			dev_err(pdev, "%s - setup irq %d failed: %d\n",
				__func__, pdata->irq, ret);
			return -EINVAL;
		}
	}

	/* Clear Pending CHGINTB interrupt */
	mask = 0;
	set_reg_field(mask, CHGINTB_O, 1, SET);
	ret = bq24261_pmic_reg_write(DEV1, CHGRIRQ0, mask);
	if (ret) {
		dev_err(pdev, "%s - fail to write CHGRIRQ0\n", __func__);
		return ret;
	}

	/* Unmask CHGINTB interrupts */
	mask = 0;
	set_reg_field(mask, CHGINTB_O, 1, SET);
	ret = bq24261_pmic_reg_set(DEV1, MCHGRIRQ0, mask, 0);
	if (ret) {
		dev_err(pdev, "%s - fail to write MCHGRIRQ0\n", __func__);
		return ret;
	}

	/* Unmask LVL1 CHGR interrupt */
	set_reg_field(mask, CHGR_O, 1, SET);
	ret = bq24261_pmic_reg_set(DEV1, MIRQLVL1, mask, 0);
	if (ret)
		dev_err(pdev, "%s - fail to set MIRQLVL1\n", __func__);

	return ret;
}

static int bq24261_otg_notification_handler(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct bq24261_data *pdata =
			container_of(nb, struct bq24261_data, otg_nb);

	switch (event) {
	case INTEL_USB_DRV_VBUS:
		if (!data)
			return NOTIFY_BAD;
		pdata->state.to_enable_boost = *((bool *)data);
		unfreezable_bh_schedule(&pdata->boost_en_bh);
		break;

	default:
		break;
	}

	return NOTIFY_OK;
}

static int bq24261_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct usb_phy *otg_handle;
	struct bq24261_data *pdata;
	struct device *pdev = &client->dev;
	int ret;
	u8 ic_info;

	INIT_CHARGER_DEBUG_ARRAY(chrgr_dbg);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_PROBE, 0, 0);

	pdata = devm_kzalloc(pdev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->client = client;
	pdata->model_name = "BQ24261";
	pdata->manufacturer = "Texas Instrument";
	pdata->chgerr_bh.in_suspend = false;
	pdata->chgerr_bh.pending_evt = false;
	pdata->boost_en_bh.in_suspend = false;
	pdata->boost_en_bh.pending_evt = false;
	pdata->otg_nb.notifier_call = bq24261_otg_notification_handler;
	pdata->state.status = BQ24261_STATUS_UNKNOWN;
	pdata->state.cc = DEFAULT_ICHRG_MA;
	pdata->state.max_cc = ICHRG_MAX_MA;
	pdata->state.cooling_cc = 0;
	pdata->state.cv = DEFAULT_VBREG_MV;
	pdata->state.iterm = 0;
	pdata->state.health = POWER_SUPPLY_HEALTH_UNKNOWN;
	pdata->state.cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE,
	pdata->state.charger_enabled = false;
	pdata->state.charging_enabled = false;
	pdata->state.to_enable_boost = false;
	pdata->state.boost_enabled = false;
	pdata->state.cooling_state = COOLING_NORMAL;
	pdata->state.vbus_ovp_fault = 0;
	pdata->state.low_supply_fault = 0;
	pdata->state.thermal_fault = 0;
	pdata->state.bat_temp_fault = 0;
	pdata->state.timer_fault = 0;
	pdata->state.bat_ovp_fault = 0;
	pdata->state.no_bat_fault = 0;

	i2c_set_clientdata(client, pdata);

	/* Take control for SW charging */
	ret = bq24261_sw_control_override(pdata);
	if (ret) {
		dev_err(pdev, "%s - fail to disable CCSM\n", __func__);
		return ret;
	}

	/* Enable charger IC via CHGDIS pin */
	ret = bq24261_enable_ic(pdata, true);
	if (ret) {
		dev_err(pdev, "%s - fail to enable charger IC\n", __func__);
		return ret;
	}

	/* Read charger HW ID and check if it's supported */
	ret = bq24261_i2c_reg_read(client, REG3_IC_INFO, &ic_info);
	if (ret) {
		dev_err(pdev, "%s - fail to read IC_INFO\n", __func__);
		return ret;
	}

	if (get_reg_field(ic_info, VENDOR_O, VENDOR_W) != BQ24261_VENDOR ||
			get_reg_field(ic_info, PN_O, PN_W) != BQ24261_PN) {
		dev_err(pdev, "%s - IC is not supported. ic_info=0x%X\n",
					__func__, ic_info);
		return -ENODEV;
	}

	pdata->client = client;

	/* Get USB phy */
	otg_handle = devm_usb_get_phy(pdev, USB_PHY_TYPE_USB2);
	if (IS_ERR_OR_NULL(otg_handle)) {
		dev_err(pdev, "%s - fail to get OTG transceiver\n", __func__);
		return -ENODEV;
	}
	pdata->otg_handle = otg_handle;

	sema_init(&pdata->prop_lock, 1);

	INIT_DELAYED_WORK(&pdata->charging_work, bq24261_charging_worker);
	INIT_DELAYED_WORK(&pdata->boost_work, bq24261_boost_worker);

	/* Set up the wake lock to prevent suspend when charging. */
	wake_lock_init(&pdata->suspend_lock,
			WAKE_LOCK_SUSPEND,
			"bq24261_wake_lock");

	if (unfreezable_bh_create(&pdata->chgerr_bh, "bq24261_err_wq",
			"bq24261_err_lock", bq24261_charger_error_handler)) {
		ret = -ENOMEM;
		goto err_wq_fail;
	}

	if (unfreezable_bh_create(&pdata->boost_en_bh, "bq24261_boost_wq",
			"bq24261_boost_lock", bq24261_enable_boost)) {
		ret = -ENOMEM;
		goto boost_wq_fail;
	}

	ret = bq24261_set_default_targets(pdata);
	if (ret) {
		dev_err(pdev, "%s - fail to set default target\n", __func__);
		goto set_target_fail;
	}

	pdata->usb_psy.name           = "usb_charger";
	pdata->usb_psy.type           = POWER_SUPPLY_TYPE_USB;
	pdata->usb_psy.properties     = bq24261_power_props;
	pdata->usb_psy.num_properties = ARRAY_SIZE(bq24261_power_props);
	pdata->usb_psy.get_property   = bq24261_usb_get_property;
	pdata->usb_psy.set_property   = bq24261_usb_set_property;
	pdata->usb_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	pdata->usb_psy.supplied_to = bq24261_supplied_to;
	pdata->usb_psy.num_supplicants = ARRAY_SIZE(bq24261_supplied_to);
	pdata->usb_psy.throttle_states = bq24261_throttle_states;
	pdata->usb_psy.num_throttle_states =
				ARRAY_SIZE(bq24261_throttle_states);

	pdata->current_psy = &pdata->usb_psy;

	ret = power_supply_register(&client->dev, &pdata->usb_psy);
	if (ret) {
		dev_err(pdev, "%s - fail to register usb psy\n", __func__);
		goto set_target_fail;
	}

	pdata->ac_psy.name           = "ac_charger";
	pdata->ac_psy.type           = POWER_SUPPLY_TYPE_MAINS;
	pdata->ac_psy.properties     = bq24261_power_props;
	pdata->ac_psy.num_properties = ARRAY_SIZE(bq24261_power_props);
	pdata->ac_psy.get_property   = bq24261_ac_get_property;
	pdata->ac_psy.set_property   = bq24261_ac_set_property;
	pdata->ac_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	pdata->ac_psy.supplied_to = bq24261_supplied_to;
	pdata->ac_psy.num_supplicants = ARRAY_SIZE(bq24261_supplied_to);
	pdata->ac_psy.throttle_states = bq24261_throttle_states;
	pdata->ac_psy.num_throttle_states =
				ARRAY_SIZE(bq24261_throttle_states);

	ret = power_supply_register(&client->dev, &pdata->ac_psy);
	if (ret) {
		dev_err(pdev, "%s - fail to register ac psy\n", __func__);
		goto ac_register_fail;
	}

	bq24261_setup_debugfs(pdata, &chrgr_dbg);

	ret = bq24261_configure_pmic_irq(pdata);
	if (ret)
		goto pmu_irq_fail;

	ret = usb_register_notifier(otg_handle, &pdata->otg_nb);
	if (ret) {
		dev_err(pdev, "fail to register OTG notification\n");
		goto pmu_irq_fail;
	}

	pdata->ack_time = jiffies;
	pdata->state.status = BQ24261_STATUS_READY;

	return 0;

pmu_irq_fail:
	bq24261_remove_debugfs_dir(pdata);
	power_supply_unregister(&pdata->ac_psy);
ac_register_fail:
	power_supply_unregister(&pdata->usb_psy);
set_target_fail:
	unfreezable_bh_destroy(&pdata->boost_en_bh);
boost_wq_fail:
	unfreezable_bh_destroy(&pdata->chgerr_bh);
err_wq_fail:
	wake_lock_destroy(&pdata->suspend_lock);

	return ret;
}

static int __exit bq24261_i2c_remove(struct i2c_client *client)
{
	struct bq24261_data *pdata = i2c_get_clientdata(client);
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_REMOVE, 0, 0);

	bq24261_remove_debugfs_dir(pdata);
	power_supply_unregister(&pdata->usb_psy);
	power_supply_unregister(&pdata->ac_psy);
	wake_lock_destroy(&pdata->suspend_lock);
	unfreezable_bh_destroy(&pdata->boost_en_bh);
	unfreezable_bh_destroy(&pdata->chgerr_bh);
	cancel_delayed_work_sync(&pdata->boost_work);
	cancel_delayed_work_sync(&pdata->charging_work);

	return 0;
}

/**
 * Called when the system is attempting to suspend.
 * If charging is in progress EBUSY is returned to abort the suspend and
 * an error is logged, as the wake lock should prevent the situation.
 *
 * @pdev	Pointer to the device.
 * @returns	0 if driver can suspend, else error code
 */
static int bq24261_suspend(struct device *pdev)
{
	struct bq24261_data *pdata;

	if (!pdev)
		return -EINVAL;

	pdata = dev_get_drvdata(pdev);

	if (!pdata) {
		dev_err(pdev, "%s: fail to get driver data\n", __func__);
		return -EINVAL;
	}

	if (pdata->state.charging_enabled) {
		/* If charging is in progess, prevent suspend. */
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SUSPEND_ERROR, 0, 0);
		return -EBUSY;
	} else {
		/* Not charging - allow suspend. */
		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SUSPEND_OK, 0, 0);
		if (device_may_wakeup(pdev)) {
			dev_info(pdev, "enable wakeirq\n");
			enable_irq_wake(pdata->irq);
		}
		unfreezable_bh_suspend(&pdata->chgerr_bh);
	}
	return 0;
}

/**
 * Called when the system is resuming from suspend.
 *
 * @pdev	Pointer to the device.
 * @returns	0 if driver can resume, else error code
 */
static int bq24261_resume(struct device *pdev)
{
	struct bq24261_data *pdata;

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_RESUME, 0, 0);

	if (!pdev)
		return -EINVAL;

	pdata = dev_get_drvdata(pdev);

	if (!pdata) {
		dev_err(pdev, "%s: fail to get driver data\n", __func__);
		return -EINVAL;
	}

	unfreezable_bh_resume(&pdata->chgerr_bh);

	if (device_may_wakeup(pdev)) {
		dev_info(pdev, "disable wakeirq\n");
		disable_irq_wake(pdata->irq);
	}
	return 0;
}

const struct dev_pm_ops bq24261_pm = {
	.suspend = bq24261_suspend,
	.resume = bq24261_resume,
};

static const struct i2c_device_id bq24261_id[] = {
	{"bq24261", 0}, { }
};

MODULE_DEVICE_TABLE(i2c, bq24261_id);

static struct i2c_driver bq24261_i2c_driver = {
	.probe = bq24261_i2c_probe,
	.remove = bq24261_i2c_remove,
	.id_table       = bq24261_id,
	.driver = {
		.name = BQ24261_NAME,
		.owner = THIS_MODULE,
		.pm = &bq24261_pm,
	},
};

static int __init bq24261_init(void)
{
	int ret;
	ret = i2c_add_driver(&bq24261_i2c_driver);
	if (ret)
		return ret;
	return 0;
}

static void __exit bq24261_exit(void)
{
	i2c_del_driver(&bq24261_i2c_driver);
}

module_init(bq24261_init);
module_exit(bq24261_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BQ24261 charger IC driver");
