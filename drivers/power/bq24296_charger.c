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

#define BQ24296_NAME "bq24296_charger"
#define pr_fmt(fmt) BQ24296_NAME": "fmt

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
#include <linux/slab.h>

#include <linux/time.h>
#include <linux/wakelock.h>
#include "bq24296_charger.h"

/*
 * Development debugging is not enabled in release image to prevent
 * loss of event history in the debug array which has a limited size
 */
#include <linux/power/charger_debug.h>

#define VOREG_STEP_MV               20
#define IOCHARGE_STEP_MA            100
#define IBUS_LIMIT_STEP_MA          400
#define IBUS_NO_LIMIT               3

#define IBAT_MAX_MA                 3008
#define IBAT_MIN_MA                 512
#define IBAT_STEP_MA                64
#define VIN_LIMIT_MIN_MV            3880
#define VIN_LIMIT_MAX_MV            5080
#define VIN_LIMIT_STEP_MV           80

#define IPRECHG_MIN_MA              128
#define IPRECHG_MAX_MA              2048
#define IPRECHG_STEP_MA             128

#define ITERM_MIN_MA                128
#define ITERM_MAX_MA                2048
#define ITERM_STEP_MA               128

enum {
	VBUS_OFF = 0,
	VBUS_ON,

	T32_TO_OCCURRED = 1,
};

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

#define CHARGER_WR_O 0xC
#define CHARGER_WR(_base) ((_base) + CHARGER_WR_O)
#define CHARGER_WR_CHGRST_O 0
#define CHARGER_WR_CHGRST_M 0x1

#define CHARGER_WR_CHGRST_NORESET 0
#define CHARGER_WR_CHGRST_RESET 1

struct charger_debug_data bq24296_chrgr_dbg = {
	.printk_logs_en = 0,
};

static struct power_supply_throttle bq24296_dummy_throttle_states[] = {
	{
	 .throttle_action = PSY_THROTTLE_CC_LIMIT,
	 },
	{
	 .throttle_action = PSY_THROTTLE_CC_LIMIT,
	 },
	{
	 .throttle_action = PSY_THROTTLE_DISABLE_CHARGING,
	 },
};

static char *bq24296_supplied_to[] = {
	"battery",
};

static enum power_supply_property bq24296_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
};

struct input_ma_limit_entry {
	int icl_ma;
	u8 value;
};

static struct input_ma_limit_entry ilimit_table[] = {
	{100, 0x00},
	{150, 0x01},
	{500, 0x02},
	{900, 0x03},
	{1200, 0x04},
	{1500, 0x05},
	{2000, 0x06},
	{3000, 0x07},
};

static int bq24296_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;
	int retry_times = 3;

	if (!client || !val)
		return -EINVAL;

read_retry:
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret >= 0) {
		*val = (u8) ret;
		ret = 0;
		CHARGER_DEBUG_READ_REG(bq24296_chrgr_dbg, reg, *val);
	} else {
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_I2C_READ_ERROR,
				  ret, reg);
		if (retry_times <= 0) {
			dev_err(&client->dev,
				"I2C RD error after retry! reg=%d, ret=0x%02X\n",
				reg, ret);
		} else {
			retry_times--;
			dev_err(&client->dev, "I2C RD timeout at retry#%d!\n",
				(3 - retry_times));
			msleep(2000);
			goto read_retry;
		}
	}
	return ret;
}

static void bq24296_dump_all_reg(struct i2c_client *client)
{
	int i, ret;
	u8 val;
	pr_info("%s:\n", __func__);
	for (i = 0; i <= 10; i++) {
		ret = bq24296_read_reg(client, i, &val);
		pr_info("reg[0x%x] = 0x%x\n", i, val);
	}
}

static int bq24296_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;
	int retry_times = 3;

	if (!client)
		return -EINVAL;

write_retry:
	ret = i2c_smbus_write_byte_data(client, reg, val);

	if (ret >= 0) {
		CHARGER_DEBUG_WRITE_REG(bq24296_chrgr_dbg, reg, val);
		ret = 0;
	} else {
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_I2C_WRITE_ERROR,
				  ret, reg);
		if (retry_times <= 0) {
			dev_err(&client->dev,
				"I2C WR error after retry! reg=%d, val=0x%02X, ret=0x%02X\n",
				reg, val, ret);
		} else {
			retry_times--;
			dev_err(&client->dev, "I2C WD timeout at retry#%d!\n",
				(3 - retry_times));
			msleep(2000);
			goto write_retry;
		}
	}
	return ret;
}

static int bq24296_masked_write(struct i2c_client *client, int reg,
				u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = bq24296_read_reg(client, reg, &temp);
	if (rc) {
		pr_err("%s: bq24296_read_reg failed: reg=%03X, rc=%d\n",
		       __func__, reg, rc);
		return rc;
	}

	temp &= ~mask;
	temp |= val & mask;

	rc = bq24296_write_reg(client, reg, temp);
	if (rc) {
		pr_err("%s: bq24296_write failed: reg=%03X, rc=%d\n",
		       __func__, reg, rc);
		return rc;
	}

	return 0;
}

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
	struct bq24296_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
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
static ssize_t fake_vbus_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct bq24296_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
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
	pr_debug("%s: sysfs_val=%d, chrgr->state.charging_enabled=%d\n",
		 __func__, sysfs_val, chrgr->state.charging_enabled);
	if (chrgr->state.vbus != 1) {
		pr_err("fake vbus event requested when USB cable removed !\n");
		up(&chrgr->prop_lock);
		return count;
	}

	if (sysfs_val == 0) {
		chrgr->fake_vbus = 0;
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
					   USB_EVENT_NONE, &chrgr->fake_vbus);

		CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_FAKE_VBUS,
				  chrgr->fake_vbus, 0);
		pr_debug("fake vbus removal sent\n");

	} else if (sysfs_val == 1 && !chrgr->state.charging_enabled) {
		chrgr->fake_vbus = 1;
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
					   USB_EVENT_VBUS, &chrgr->fake_vbus);

		CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_FAKE_VBUS,
				  chrgr->fake_vbus, 0);
		pr_debug("fake vbus connection sent\n");
	}

	up(&chrgr->prop_lock);

	return count;
}

static struct device_attribute bq24296_fake_vbus_attr = {
	.attr = {
		 .name = "fake_vbus_event",
		 .mode = S_IRUSR | S_IWUSR,
		 },
	.show = fake_vbus_show,
	.store = fake_vbus_store,
};

/**
 * bq24296_setup_sysfs_attr	Sets up sysfs entries for bq24296 i2c device
 * @chrgr			[in] pointer to charger driver internal
 *				structure
 */
static void bq24296_setup_fake_vbus_sysfs_attr(struct bq24296_charger *chrgr)
{
	struct device *dev = &chrgr->client->dev;
	int err;

	err = device_create_file(dev, &bq24296_fake_vbus_attr);
	if (err)
		pr_err("Unable to create sysfs entry: '%s'\n",
		       bq24296_fake_vbus_attr.attr.name);
}

#else

static inline void bq24296_setup_fake_vbus_sysfs_attr(struct bq24296_charger
						      *chrgr)
{
	(void)chrgr;
}

#endif /*SYSFS_FAKE_VBUS_SUPPORT */

#ifdef CONFIG_DEBUG_FS
static int dbg_evt_open(struct inode *inode, struct file *file)
{

	/* save private data (the address of test_mod) in file struct
	   (will be used by read()) */
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t dbg_evt_read(struct file *filp, char __user *buf,
			    size_t count, loff_t *f_pos)
{

	/* obtaining private data saved by open method */
	struct charger_debug_data *dbg_array =
	    (struct charger_debug_data *)filp->private_data;

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
		dbg_array->read_index &= (CHARGER_DEBUG_DATA_SIZE - 1);

		spin_unlock(&dbg_array->lock);

		time_stamp_s = time_stamp_jiffies / HZ;
		chars_count +=
		    snprintf(log_line, LOG_LINE_LENGTH, "[%5lu.%3lu]: %s ",
			     time_stamp_s,
			     (time_stamp_jiffies - time_stamp_s * HZ),
			     event_str);

		if (event == CHG_DBG_REG) {
			chars_count += snprintf(log_line + chars_count,
						LOG_LINE_LENGTH - chars_count,
						"addr=0x%x, val=0x%x\n", prm,
						prm2);

		} else if (event == CHG_DBG_I2C_READ_ERROR ||
			   event == CHG_DBG_I2C_WRITE_ERROR) {

			chars_count += snprintf(log_line + chars_count,
						LOG_LINE_LENGTH - chars_count,
						"err=%d, at addr=0x%x\n", prm,
						prm2);

		} else if (event < CHG_DBG_LAST_NO_PARAM_EVENT) {

			chars_count += snprintf(log_line + chars_count,
						LOG_LINE_LENGTH - chars_count,
						"\n");
		} else {
			chars_count += snprintf(log_line + chars_count,
						LOG_LINE_LENGTH - chars_count,
						"val=%d\n", prm);
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

static const struct file_operations bq24296_evt_dbg_fops = {
	.open = dbg_evt_open,
	.read = dbg_evt_read,
};

static int bq24296_dbg_regs_show(struct seq_file *m, void *data)
{
	int i, ret = 0;
	struct bq24296_charger *chrgr = (struct bq24296_charger *)m->private;
	u8 val;
	unsigned long timestamp_jiffies, timestamp_s;

	down(&chrgr->prop_lock);

	timestamp_jiffies = jiffies - INITIAL_JIFFIES;
	timestamp_s = timestamp_jiffies / HZ;
	seq_printf(m, "[%5lu.%3lu] :\n", timestamp_s,
		   (timestamp_jiffies - timestamp_s * HZ));

	for (i = 0; i <= 0x0A; i++) {
		ret = bq24296_read_reg(chrgr->client, i, &val);
		if (ret)
			goto out;
		seq_printf(m, "reg[0x%x] = 0x%x\n", i, val);
	}

out:
	up(&chrgr->prop_lock);

	return ret;
}

static int bq24296_dbg_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, bq24296_dbg_regs_show, inode->i_private);
}

static const struct file_operations bq24296_dbg_regs_fops = {
	.open = bq24296_dbg_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bq24296_dbg_state_show(struct seq_file *m, void *data)
{
	struct bq24296_charger *chrgr = (struct bq24296_charger *)m->private;
	unsigned long timestamp_jiffies, timestamp_s;

	(void)data;

	timestamp_jiffies = jiffies - INITIAL_JIFFIES;
	timestamp_s = timestamp_jiffies / HZ;
	seq_printf(m, "[%5lu.%3lu] :\n", timestamp_s,
		   (timestamp_jiffies - timestamp_s * HZ));

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

	return 0;
}

static int bq24296_dbg_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, bq24296_dbg_state_show, inode->i_private);
}

static const struct file_operations bq24296_dbg_state_fops = {
	.open = bq24296_dbg_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bq24296_charging_state_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t bq24296_set_charging(struct file *filp,
				    const char __user *buffer, size_t count,
				    loff_t *offset)
{
	struct bq24296_charger *chrgr =
	    (struct bq24296_charger *)filp->private_data;
	int intval = simple_strtol(buffer, NULL, 10);

	if (intval > 0) {
		pr_debug("%s: enable_charging\n", __func__);
		bq24296_enable_charging(chrgr, true);
	} else {
		pr_debug("%s: disable_charging\n", __func__);
		bq24296_enable_charging(chrgr, false);
	}

	*offset += count;
	return count;
}

static ssize_t bq24296_get_charging(struct file *filp, char __user *buffer,
				    size_t count, loff_t *offset)
{
	struct bq24296_charger *chrgr =
	    (struct bq24296_charger *)filp->private_data;
	char *buf;
	int len;

	buf = kasprintf(GFP_KERNEL, "%d\n", chrgr->state.charging_enabled);

	if (!buf) {
		pr_err("%s ENOMEM\n", __func__);
		return -ENOMEM;
	}

	if (count < strlen(buf)) {
		kfree(buf);
		pr_err("%s ENOSPC\n", __func__);
		return -ENOSPC;
	}

	len = simple_read_from_buffer(buffer, count, offset, buf, strlen(buf));
	kfree(buf);
	return len;
}

static const struct file_operations bq24296_dbg_charging_fops = {
	.open = bq24296_charging_state_open,
	.read = bq24296_get_charging,
	.write = bq24296_set_charging,
};

/**
 * bq24296_setup_debugfs - sets up debugfs entries for bq24296 charger driver
 * @chrgr		[in] pointer to charger driver internal structure
 * @dbg_data		[in] pointer to debug array containing events logs.
 */
static void bq24296_setup_debugfs(struct bq24296_charger *chrgr,
				  struct charger_debug_data *dbg_data)
{
	struct dentry *dbgfs_entry;

	dbgfs_entry = debugfs_create_dir(BQ24296_NAME, NULL);
	if (!dbgfs_entry)
		return;

	chrgr->debugfs_root_dir = dbgfs_entry;

	(void)debugfs_create_file(EVENTS_LOG_FILENAME, S_IRUGO,
				  dbgfs_entry, dbg_data, &bq24296_evt_dbg_fops);

	(void)debugfs_create_file(DBG_REGS_FILENAME, S_IRUGO,
				  dbgfs_entry, chrgr, &bq24296_dbg_regs_fops);

	(void)debugfs_create_file(DBG_STATE_FILENAME, S_IRUGO,
				  dbgfs_entry, chrgr, &bq24296_dbg_state_fops);

	(void)debugfs_create_file(DBG_CHARGING_STATE_FILENAME,
				  S_IFREG | S_IRUGO, dbgfs_entry, chrgr,
				  &bq24296_dbg_charging_fops);

	return;
}

/**
 * bq24296_remove_debugfs_dir	recursively removes debugfs root directory
 *				of BQ24296 charger driver
 * @chrgr			[in] pointer to charger driver's
 *				internal structure
 */
static void bq24296_remove_debugfs_dir(struct bq24296_charger *chrgr)
{
	debugfs_remove_recursive(chrgr->debugfs_root_dir);
	return;
}

#else

static inline void bq24296_setup_debugfs(struct bq24296_charger *chrgr,
					 struct charger_debug_data *dbg_data)
{

}

static inline void bq24296_remove_debugfs_dir(struct bq24296_charger *chrgr)
{

}

#endif /* CONFIG_DEBUG_FS  */
static int bq24296_get_clr_wdt_expiry_flag(struct bq24296_charger *chrgr)
{
	u8 t32_to;
	int ret, wtd_expired;

	/* If the WDT interrupt was previously set,
	   it will now be cleared upon reading */
	ret = bq24296_read_reg(chrgr->client, BQ09_FAULT_REG, &t32_to);
	if (ret)
		return ret;

	t32_to &= 0x80;
	wtd_expired = t32_to ? T32_TO_OCCURRED : 0;
	pr_debug("%s: get watchdog flag = %d !!!\n", __func__, wtd_expired);

	return wtd_expired;
}

static int bq24296_trigger_wtd(struct bq24296_charger *chrgr)
{
	int ret;
	u8 reg01;
	CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_TRIGGERING_WTD, 0, 0);
	ret = bq24296_masked_write(chrgr->client, BQ01_PWR_ON_CONF_REG,
				WATCHDOG_TIME_RESET, 1);
	ret = bq24296_read_reg(chrgr->client, BQ01_PWR_ON_CONF_REG, &reg01);
	pr_debug("%s: reg01 readback=0x%x\n", __func__, reg01);
	return ret;
}

static int bq24296_get_prop_charge_type(struct bq24296_charger *chrgr)
{
	int ret = 0;
	u8 sys_status;
	enum bq24296_chg_status status;
	int chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	ret = bq24296_read_reg(chrgr->client,
			BQ08_SYSTEM_STATUS_REG, &sys_status);
	if (ret) {
		pr_err("%s: fail to read BQ08_SYSTEM_STATUS_REG. ret=%d\n",
			__func__, ret);
		chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		goto exception_handling;
	}

	sys_status &= CHRG_STAT_MASK;
	if (sys_status == 0x10) {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		status = BQ_CHG_STATUS_PRE_CHARGE;
	} else if (sys_status == 0x20) {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		status = BQ_CHG_STATUS_FAST_CHARGE;
	} else if (sys_status == 0x30) {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		status = BQ_CHG_STATUS_FULL;
	} else {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		status = BQ_CHG_STATUS_NONE;
	}
	pr_debug("%s: bq-chg-status (%d = %s).\n",
		__func__, status, bq24296_chg_status[status]);
	if (chrgr->state.status != status) {
		if (status == BQ_CHG_STATUS_NONE
			|| status == BQ_CHG_STATUS_FULL) {
			pr_debug("%s: Charging stopped.\n", __func__);
		} else {
			pr_debug("%s: Charging started.\n", __func__);
		}
		chrgr->state.status = status;
	}

	return chg_type;

exception_handling:
	chrgr->state.status = BQ_CHG_STATUS_EXCEPTION;
	return chg_type;

}

#define VBAT_MAX_MV  4400
#define VBAT_MIN_MV  3504
#define VBAT_STEP_MV  16
static int bq24296_set_vbat_max(struct bq24296_charger *chrgr, int mv)
{
	u8 reg_val = 0;
	int set_vbat = 0;

	if (mv < VBAT_MIN_MV)
		mv = VBAT_MIN_MV;
	if (mv > VBAT_MAX_MV)
		mv = VBAT_MAX_MV;

	reg_val = (mv - VBAT_MIN_MV) / VBAT_STEP_MV;
	set_vbat = reg_val * VBAT_STEP_MV + VBAT_MIN_MV;
	reg_val = reg_val << 2;

	pr_debug("%s: req_vbat = %d set_vbat = %d reg_val = 0x%02x\n", __func__,
		 mv, set_vbat, reg_val);

	return bq24296_masked_write(chrgr->client, BQ04_CHARGE_VOLT_CONT_REG,
					CHG_VOLTAGE_LIMIT_MASK, reg_val);
}

static int bq24296_set_input_vin_limit(struct bq24296_charger *chrgr, int mv)
{
	u8 reg_val = 0;
	int set_vin = 0;

	if (mv < VIN_LIMIT_MIN_MV)
		mv = VIN_LIMIT_MIN_MV;
	if (mv > VIN_LIMIT_MAX_MV)
		mv = VIN_LIMIT_MAX_MV;

	reg_val = (mv - VIN_LIMIT_MIN_MV) / VIN_LIMIT_STEP_MV;
	set_vin = reg_val * VIN_LIMIT_STEP_MV + VIN_LIMIT_MIN_MV;
	reg_val = reg_val << 3;

	pr_debug("%s: req_vin = %d set_vin = %d reg_val = 0x%02x\n", __func__,
		mv, set_vin, reg_val);

	return bq24296_masked_write(chrgr->client, BQ00_INPUT_SRC_CONT_REG,
		VINDPM_MASK, reg_val);
}

static int bq24296_get_input_vin_limit(struct bq24296_charger *chrgr, int *mv)
{
	u8 reg_val = 0;
	int ret;

	ret = bq24296_read_reg(chrgr->client,
				BQ00_INPUT_SRC_CONT_REG, &reg_val);
	if (ret) {
		pr_err("%s: failed to read BQ00_SYSTEM_STATUS_REG ret=%d\n",
			__func__, ret);
		return ret;
	}
	*mv = (reg_val & VINDPM_MASK >> 3) * VIN_LIMIT_STEP_MV +
	    VIN_LIMIT_MIN_MV;
	return ret;
}

static int bq24296_set_ibus_limit(struct bq24296_charger *chrgr,
	int ilim_to_set, int *ilim_set)
{
	int ret, current_to_set_ma, i;
	u8 regval, readback_val;

	if (ilim_to_set < chrgr->min_vbus_ilimit) {
		if (chrgr->state.charging_enabled) {
			ret = bq24296_enable_charging(chrgr, false);
			if (ret != 0)
				return ret;
			chrgr->state.charging_enabled = 0;
		}
		*ilim_set = 0;
		return 0;
	}

	if (ilim_to_set <= chrgr->max_vbus_ilimit) {
		current_to_set_ma = fit_in_range(ilim_to_set,
			chrgr->min_vbus_ilimit,
			chrgr->max_vbus_ilimit);

		for (i = ARRAY_SIZE(ilimit_table) - 1; i > 0; i--) {
			if (current_to_set_ma >= ilimit_table[i].icl_ma)
				break;
		}
		regval = ilimit_table[i].value;
	} else
		regval = IBUS_NO_LIMIT;

	ret = bq24296_masked_write(chrgr->client, BQ00_INPUT_SRC_CONT_REG,
			IINLIM_MASK, regval);
	if (ret)
		return ret;

	ret = bq24296_read_reg(chrgr->client,
			BQ00_INPUT_SRC_CONT_REG, &readback_val);
	if (ret) {
		pr_err("%s: failed to read BQ00_INPUT_SRC_CONT_REG ret=%d\n",
			__func__, ret);
		return -EIO;
	}
	readback_val &= 0x07;
	if (readback_val != regval) {
#if 0
		pr_err("%s: I2C write error! register ILIMIT still contains the old value\n",
		__func__);
#endif
		return -EIO;
	}

	*ilim_set = (regval == IBUS_NO_LIMIT) ? ilim_to_set :
		(ilimit_table[readback_val].icl_ma);

	pr_debug("%s: ilimit_set = %d\n", __func__, *ilim_set);

	return 0;
}

static int bq24296_set_force_ichg_decrease
	(struct bq24296_charger *chrgr, bool enable) {
	int ret;
	u8 val = (u8) (!!enable);

	pr_debug("enable=%d\n", enable);

	ret = bq24296_masked_write(chrgr->client,
		BQ02_CHARGE_CUR_CONT_REG, FORCE_20PCT_MASK,
		val);
	if (ret) {
		pr_err("%s: failed to set FORCE_20PCT ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int bq24296_get_force_ichg_decrease
	(struct bq24296_charger *chrgr, int *enable) {
	int ret;
	u8 val;

	ret = bq24296_read_reg(chrgr->client, BQ02_CHARGE_CUR_CONT_REG, &val);
	if (ret) {
		pr_err("%s: failed to get FORCE_20PCT ret=%d\n", __func__, ret);
		return ret;
	}
	*enable = (val & FORCE_20PCT_MASK) ? 1 : 0;

	return 0;
}

static int bq24296_set_ibat_max(struct bq24296_charger *chrgr, int ma)
{
	u8 reg_val = 0;
	int set_ibat = 0;
	int ret = 0;

	if (ma < IBAT_MIN_MA) {
		bq24296_enable_charging(chrgr, false);
		ma = IBAT_MIN_MA;
	} else {
		bq24296_enable_charging(chrgr, true);
	}
	if (ma > IBAT_MAX_MA)
		ma = IBAT_MAX_MA;

	reg_val = (ma - IBAT_MIN_MA) / IBAT_STEP_MA;
	set_ibat = reg_val * IBAT_STEP_MA + IBAT_MIN_MA;
	reg_val = reg_val << 2;
	pr_debug("%s: req_ibat = %d set_ibat = %d reg_val = 0x%02x\n",
		 __func__, ma, set_ibat, reg_val);

	ret = bq24296_masked_write(chrgr->client, BQ02_CHARGE_CUR_CONT_REG,
				ICHG_MASK, reg_val);
	if (ret) {
		pr_err("%s: failed to set ICHG_MASK ret=%d\n", __func__, ret);
		return ret;
	}
#ifdef CONFIG_CHARGER_BQ24296_BPROFILE_FROM_DTS
	bq24296_set_force_ichg_decrease(chrgr, chrgr->force_ichg_decrease);
#endif

	return 0;
}

static int bq24296_get_ibat_max(struct bq24296_charger *chrgr, int *mv)
{
	u8 reg_val = 0;
	int ret;

	ret = bq24296_read_reg(chrgr->client,
				BQ02_CHARGE_CUR_CONT_REG, &reg_val);
	if (ret) {
		pr_err("%s: failed to read BQ08_SYSTEM_STATUS_REG ret=%d\n",
			__func__, ret);
		return -EIO;
	}
	reg_val = reg_val >> 2;
	*mv = reg_val * IBAT_STEP_MA + IBAT_MIN_MA;
	return ret;
}

static int bq24296_get_adjust_ibat(struct bq24296_charger *chrgr, int *mv)
{
	int ret, enable;

	ret = bq24296_get_ibat_max(chrgr, mv);
	if (ret)
		return ret;
	bq24296_get_force_ichg_decrease(chrgr, &enable);
	if (enable)
		*mv /= 5;
	return 0;
}

static int bq24296_set_prechg_ilimit(struct bq24296_charger *chrgr, int ma)
{
	u8 reg_val = 0;
	int set_ma = 0;

	if (ma < IPRECHG_MIN_MA)
		ma = IPRECHG_MIN_MA;
	if (ma > IPRECHG_MAX_MA)
		ma = IPRECHG_MAX_MA;

	reg_val = (ma - IPRECHG_MIN_MA) / IPRECHG_STEP_MA;
	set_ma = reg_val * IPRECHG_STEP_MA + IPRECHG_MIN_MA;
	reg_val = reg_val << 4;

	pr_debug("%s: req_prechg_ilimit = %d set_prechg_ilimit = %d reg_val = 0x%02x\n",
		__func__, ma, set_ma, reg_val);

	return bq24296_masked_write(chrgr->client, BQ03_PRE_CHARGE_TERM_CUR_REG,
			IPRECHG_MASK, reg_val);
}

static int bq24296_set_term_current(struct bq24296_charger *chrgr, int ma)
{
	u8 reg_val = 0;
	int set_ma = 0;

	if (ma < ITERM_MIN_MA)
		ma = ITERM_MIN_MA;
	if (ma > ITERM_MAX_MA)
		ma = ITERM_MAX_MA;

	reg_val = (ma - ITERM_MIN_MA) / ITERM_STEP_MA;
	set_ma = reg_val * ITERM_STEP_MA + ITERM_MIN_MA;

	pr_debug("%s: req_term_i = %d set_term_i = %d reg_val = 0x%02x\n",
		__func__, ma, set_ma, reg_val);

	return bq24296_masked_write(chrgr->client, BQ03_PRE_CHARGE_TERM_CUR_REG,
					ITERM_MASK, reg_val);
}

#define EN_TIMER_SHIFT 3
static int bq24296_set_chg_timer(struct bq24296_charger *chrgr, bool enable)
{
	int ret;
	u8 val = (u8)(!!enable << EN_TIMER_SHIFT);

	pr_debug("%s enable=%d\n", __func__, enable);

	ret = bq24296_masked_write(chrgr->client,
			BQ05_CHARGE_TERM_TIMER_CONT_REG,
				EN_CHG_TIMER_MASK, val);
	if (ret) {
		pr_err("%s: failed to set chg safety timer ret=%d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

#define CHG_TIMEOUT_SHIFT 1
static int bq24296_set_chg_timeout(struct bq24296_charger *chrgr)
{
	u8 reg_val = 1;

	pr_debug("%s: req_chg_timeout set_h = 8hrs\n", __func__);

	reg_val = reg_val << CHG_TIMEOUT_SHIFT;

	return bq24296_masked_write(chrgr->client,
					BQ05_CHARGE_TERM_TIMER_CONT_REG,
					CHG_TIMER_MASK, reg_val);
}

#define CHG_WATCHDOG_SHIFT 4
static int bq24296_set_chg_watchdog_timeout(struct bq24296_charger *chrgr)
{
#ifdef CONFIG_CHARGER_BQ24296_SUSPEND_WITH_CHARGING
	/* Disable WDT timer */
	u8 reg_val = 0;
#else
	u8 reg_val = 3;
#endif

	pr_debug("%s: set_val=0x%x\n", __func__, reg_val);

	reg_val = reg_val << CHG_WATCHDOG_SHIFT;

	bq24296_masked_write(chrgr->client,
		BQ05_CHARGE_TERM_TIMER_CONT_REG,
		I2C_TIMER_MASK, 0);

	return bq24296_masked_write(chrgr->client,
		BQ05_CHARGE_TERM_TIMER_CONT_REG,
		I2C_TIMER_MASK, reg_val);
}

#define EN_CHG_TERM_SHIFT 7
static int bq24296_set_chg_term(struct bq24296_charger *chrgr, bool enable)
{
	int ret;
	u8 val = (u8)(!!enable << EN_CHG_TERM_SHIFT);

	pr_debug("%s: enable=%d\n", __func__, enable);

	ret = bq24296_masked_write(chrgr->client,
					BQ05_CHARGE_TERM_TIMER_CONT_REG,
					EN_CHG_TERM_MASK, val);
	if (ret) {
		pr_err("failed to disable chg term  ret=%d\n", ret);
		return ret;
	}

	return 0;

}

static bool bq24296_is_charger_online(struct bq24296_charger *chrgr)
{
	int ret = 0;
	u8 sys_status, power_good;
	bool power_ok;

	ret = bq24296_read_reg(chrgr->client,
			       BQ08_SYSTEM_STATUS_REG, &sys_status);
	if (ret) {
		pr_err("failed to read BQ08_SYSTEM_STATUS_REG ret=%d\n", ret);
		return false;
	}

	power_good = (sys_status & PG_STAT_MASK);
	sys_status &= VBUS_STAT_MASK;

	if ((power_good == 0) && (sys_status == 0 || sys_status == 0xC0)) {
		power_ok = false;
		pr_debug("%s: DC is missing.\n", __func__);
	} else {
		power_ok = true;
		pr_debug("%s: DC is present.\n", __func__);
	}

	if (sys_status == 0x40)
		chrgr->state.cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_SDP;
	else if (sys_status == 0x80)
		chrgr->state.cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
	else
		chrgr->state.cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;

	return power_ok;
}

static inline bool bq24296_is_charger_present(struct bq24296_charger *chrgr,
		struct power_supply *psy)
{
	if (!(chrgr->state.health == POWER_SUPPLY_HEALTH_GOOD) ||
		!chrgr->state.charger_enabled) {
		pr_debug("%s: health = %d, charger_enable = %d\n",
			__func__, chrgr->state.health,
			chrgr->state.charger_enabled);
		return false;
	}

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

static void bq24296_charging_worker(struct work_struct *work)
{
	int ret;
	u8 reg09;
	struct bq24296_charger *chrgr =
	    container_of(work, struct bq24296_charger, charging_work.work);

	ret = bq24296_read_reg(chrgr->client, BQ09_FAULT_REG, &reg09);
	pr_debug("%s:, reg09=0x%x\n", __func__, reg09);

	/* if (!time_after(jiffies, chrgr->ack_time + (60*HZ))) { */
	down(&chrgr->prop_lock);
	pr_debug("%s: trigger wtd!, jiffies=%u\n", __func__, jiffies);
	ret = bq24296_trigger_wtd(chrgr);
	bq24296_set_chg_watchdog_timeout(chrgr);
	up(&chrgr->prop_lock);
	if (ret != 0) {
		pr_err("%s: bq24296_set_chg_watchdog_timeout fail\n", __func__);
		return;
	}
	/* } */

	power_supply_changed(chrgr->current_psy);
	CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
			  CHG_DBG_TRIG_POWER_SUPPLY_CHARGER, 0, 0);

	schedule_delayed_work(&chrgr->charging_work, CHRGR_WORK_DELAY);

	return;
}

int bq24296_enable_charging(struct bq24296_charger *chrgr, bool enable)
{
	int ret;

	pr_debug("%s: enable:%d\n", __func__, enable);

	ret = bq24296_configure_chip(chrgr, enable);

	if (enable) {
		chrgr->state.charging_enabled = 1;
#ifndef CONFIG_CHARGER_BQ24296_SUSPEND_WITH_CHARGING
		/*
		 * Obtain Wake Lock to prevent suspend during charging
		 * because the charger watchdog needs to be cont. retriggered.
		 */
		wake_lock(&chrgr->suspend_lock);
#endif
		schedule_delayed_work(&chrgr->charging_work, CHRGR_WORK_DELAY);

	} else {
		chrgr->state.charging_enabled = 0;

#ifndef CONFIG_CHARGER_BQ24296_SUSPEND_WITH_CHARGING
		/* Release Wake Lock to allow suspend during discharging */
		wake_unlock(&chrgr->suspend_lock);
#endif
		if (!enable) {
			/* Stop WDT timer to avoid WDT expiration */
			ret = bq24296_masked_write(chrgr->client,
				BQ05_CHARGE_TERM_TIMER_CONT_REG,
				I2C_TIMER_MASK, 0);
			if (ret)
				pr_err("failed to set Stop WDT ret=%d\n", ret);
		}

		cancel_delayed_work(&chrgr->charging_work);
	}
	return 0;
}

#define SYSTEM_VMIN_LOW_MV  3000
#define SYSTEM_VMIN_HIGH_MV  3700
#define SYSTEM_VMIN_STEP_MV  100
static int bq24296_set_system_vmin(struct bq24296_charger *chrgr, int mv)
{
	u8 reg_val = 0;
	int set_vmin = 0;

	if (mv < SYSTEM_VMIN_LOW_MV)
		mv = SYSTEM_VMIN_LOW_MV;
	if (mv > SYSTEM_VMIN_HIGH_MV)
		mv = SYSTEM_VMIN_HIGH_MV;

	reg_val = (mv - SYSTEM_VMIN_LOW_MV) / SYSTEM_VMIN_STEP_MV;
	set_vmin = reg_val * SYSTEM_VMIN_STEP_MV + SYSTEM_VMIN_LOW_MV;
	reg_val = reg_val << 1;

	pr_debug("%s: req_vmin = %d set_vmin = %d reg_val = 0x%02x\n",
		 __func__, mv, set_vmin, reg_val);

	return bq24296_masked_write(chrgr->client, BQ01_PWR_ON_CONF_REG,
				    SYS_MIN_VOL_MASK, reg_val);
}

#define OTG_ENABLE_SHIFT  5
static int bq24296_enable_otg(struct bq24296_charger *chrgr, bool enable)
{
	int ret;
	u8 val = (u8) (!!enable << OTG_ENABLE_SHIFT);

	pr_debug("otg enable = %d\n", enable);

	ret = bq24296_masked_write(chrgr->client, BQ01_PWR_ON_CONF_REG,
				   OTG_ENABLE_MASK, val);
	if (ret) {
		pr_err("failed to set OTG_ENABLE_MASK rc=%d\n", ret);
		return ret;
	}

	return 0;
}

static bool bq24296_is_otg_mode(struct bq24296_charger *chrgr)
{
	u8 temp, vbus_state;
	int ret;

	ret = bq24296_read_reg(chrgr->client, BQ01_PWR_ON_CONF_REG, &temp);
	if (ret) {
		pr_err("failed to read OTG_ENABLE_MASK rc=%d\n", ret);
		return false;
	}
	ret = bq24296_read_reg(chrgr->client,
			       BQ08_SYSTEM_STATUS_REG, &vbus_state);
	pr_err("%s: reg08=0x%x, reg01=0x%x\n", __func__, vbus_state, temp);

	return !!(temp & OTG_ENABLE_MASK);
}

static int bq24296_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	int value_to_set, value_set = 0, ret = 0;
	struct bq24296_charger *chrgr =
	    i2c_get_clientdata(to_i2c_client(psy->dev->parent));

	bool call_psy_changed = false;
	union power_supply_propval v;

	down(&chrgr->prop_lock);

	if (chrgr->state.status == BQ_CHG_STATUS_EXCEPTION) {
		ret = -EFAULT;
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROPERTY_ERROR, ret, 0);

		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_DRIVER_IN_FAULT_STATE, 0, 0);
		up(&chrgr->prop_lock);
		return ret;
	}

	switch (psp) {

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROP_CABLE_TYPE, val->intval, 0);

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
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROP_CHARGE_CURRENT,
				  val->intval, 0);

		if (!val->intval)
			break;

		value_to_set = fit_in_range(val->intval, 0,
					    chrgr->state.max_cc);
		if (value_to_set == chrgr->state.cc)
			break;
		bq24296_set_ibat_max(chrgr, value_to_set);
		chrgr->state.cc = value_to_set;
		bq24296_dump_all_reg(chrgr->client);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROP_CHARGE_VOLTAGE,
				  val->intval, 0);

		if (!val->intval)
			break;

		value_set = fit_in_range(val->intval,
					 chrgr->vbat_min_mv,
					 chrgr->vbat_max_mv);

		if (value_set == chrgr->state.cv)
			break;

		bq24296_set_vbat_max(chrgr, value_set);
		chrgr->state.cv = value_set;

		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROP_ENABLE_CHARGER,
				  val->intval, 0);

		pr_debug("%s, POWER_SUPPLY_PROP_ENABLE_CHARGER: charging %d\n",
			 __func__, val->intval);

		chrgr->state.charger_enabled = val->intval;
		bq24296_enable_charging(chrgr, val->intval);

		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROP_ENABLE_CHARGING,
				  val->intval, 0);

		pr_debug("%s, POWER_SUPPLY_PROP_ENABLE_CHARGING: charging %d\n",
			 __func__, val->intval);

		if (val->intval == chrgr->state.charging_enabled) {
			value_set = val->intval;
			break;
		}
		ret = bq24296_enable_charging(chrgr, val->intval);
		if (!ret) {
			chrgr->state.charging_enabled = val->intval;
			call_psy_changed = true;
		}
		break;

	case POWER_SUPPLY_PROP_INLMT:
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_SET_PROP_INLMT,
				  val->intval, 0);
		value_to_set = val->intval;
		ret = bq24296_set_ibus_limit(chrgr, value_to_set, &value_set);
		if (ret) {
			chrgr->state.status = BQ_CHG_STATUS_EXCEPTION;
			chrgr->state.health =
			    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}

		chrgr->state.inlmt = value_set;
		if (value_set > chrgr->max_vbus_ilimit)
			chrgr->state.max_cc =
			    fit_in_range(value_set, 0, chrgr->max_vbus_ilimit);
		break;

	case POWER_SUPPLY_PROP_CONTINUE_CHARGING:
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROP_CONTINUE_CHARGING,
				  val->intval, 0);

		chrgr->ack_time = jiffies;
		value_set = val->intval;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROP_CHARGE_TERM_CUR,
				  val->intval, 0);
		value_set = val->intval;
		bq24296_set_term_current(chrgr, value_set);
		chrgr->state.iterm = value_set;
		break;

	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROP_MIN_TEMP, val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROP_MAX_TEMP, val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROP_CHARGE_CONTROL_LIMIT,
				  val->intval, 0);

		chrgr->state.throttle = val->intval;
		chrgr->state.max_cc = chrgr->throttle_values[val->intval];

		up(&chrgr->prop_lock);

		v.intval = !!chrgr->state.max_cc;
		chrgr->current_psy->set_property(chrgr->current_psy,
			POWER_SUPPLY_PROP_ENABLE_CHARGING,
			&v);

		v.intval = chrgr->state.max_cc;
		chrgr->current_psy->set_property(chrgr->current_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
			&v);

		down(&chrgr->prop_lock);
		break;

	default:
		break;
	};

	if (!ret)
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROPERTY_VALUE_SET, value_set, 0);
	else
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SET_PROPERTY_ERROR, ret, 0);

	up(&chrgr->prop_lock);

	if (call_psy_changed)
		power_supply_changed(psy);

	return ret;
}

static int bq24296_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq24296_charger *chrgr =
	    i2c_get_clientdata(to_i2c_client(psy->dev->parent));

	down(&chrgr->prop_lock);

	switch (psp) {

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq24296_is_charger_present(chrgr, psy);
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_PRESENT, val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq24296_is_charger_online(chrgr);
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_ONLINE, val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chrgr->state.health;
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_HEALTH, val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_USB;
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_TYPE, val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		val->intval = chrgr->state.cable_type;
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_CABLE_TYPE, val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		val->intval = chrgr->state.charger_enabled;
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_ENABLE_CHARGER,
				  val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		val->intval = chrgr->state.charging_enabled;
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_ENABLE_CHARGING,
				  val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = chrgr->state.throttle;
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_CHARGE_CONTROL_LIMIT,
				  val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = chrgr->throttle_levels - 1;
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_CHARGE_CONTROL_LIMIT_MAX,
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
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_CHARGE_CURRENT,
				  val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = chrgr->state.cv;
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_CHARGE_VOLTAGE,
				  val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chrgr->state.max_cc;
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_MAX_CHARGE_CURRENT,
				  val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_INLMT:
		val->intval = chrgr->state.inlmt;
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_INLMT, val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		val->intval = chrgr->state.iterm;
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_CHARGE_TERM_CUR,
				  val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_PRIORITY:
		val->intval = 0;
		CHARGER_DEBUG_DEV(bq24296_chrgr_dbg,
				  CHG_DBG_GET_PROP_PRIORITY, val->intval, 0);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq24296_get_prop_charge_type(chrgr);
		break;

	default:
		ret = -EINVAL;
		break;
	};

	up(&chrgr->prop_lock);

	return ret;
}

static void bq24296_boost_worker(struct work_struct *work)
{
	struct bq24296_charger *chrgr =
	    container_of(work, struct bq24296_charger, boost_work.work);

	down(&chrgr->prop_lock);

	up(&chrgr->prop_lock);
	return;
}

static void bq24296_set_boost(struct work_struct *work)
{
	struct bq24296_charger *chrgr =
	    container_of(work, struct bq24296_charger,
			 boost_op_bh.work);
	int on = chrgr->state.to_enable_boost;
	int ret = 0;

	down(&chrgr->prop_lock);

	if (on) {
		pr_debug("%s: to_enable_boost=%d\n", __func__, on);
		/* Enable boost regulator */
		ret = bq24296_enable_otg(chrgr, 1);
		if (ret)
			goto exit_boost;

		/* Boost startup time is 2 ms max */
		mdelay(2);

		/* Ensure Boost is in regulation */
		ret = bq24296_is_otg_mode(chrgr);
		if (!ret)
			goto exit_boost;

		/* Enable boost mode flag */
		chrgr->state.boost_enabled = 1;

		wake_lock(&chrgr->suspend_lock);
		schedule_delayed_work(&chrgr->boost_work, BOOST_WORK_DELAY);
	} else {
		cancel_delayed_work(&chrgr->boost_work);

		/* Release Wake Lock */
		wake_unlock(&chrgr->suspend_lock);

		/* Disable boost mode flag */
		chrgr->state.boost_enabled = 0;

		/* Disable boost regulator */
		ret = bq24296_enable_otg(chrgr, 0);
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
 * bq24296_chgint_cb_work_func	function executed by
 *				bq24296_charger::chgint_cb_work work
 * @work			[in] pointer to associated 'work' structure
 */
static void bq24296_chgint_cb_work_func(struct work_struct *work)
{
	int ret, vbus_state_prev, health_prev;
	enum usb_phy_events event;
	u8 reg08, reg09;
	struct bq24296_charger *chrgr =
	    container_of(work, struct bq24296_charger,
			 chgint_bh.work);

	down(&chrgr->prop_lock);

	CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_CHGINT_CB, 0, 0);
	vbus_state_prev = chrgr->state.vbus;
	health_prev = chrgr->state.health;

	ret = bq24296_read_reg(chrgr->client, BQ08_SYSTEM_STATUS_REG, &reg08);
	if (ret)
		goto fail;
	ret = bq24296_read_reg(chrgr->client, BQ09_FAULT_REG, &reg09);
	if (ret)
		goto fail;

	pr_info("08:0x%02X, 09:0x%02X\n", reg08, reg09);

	/* re-initialize the vbus and do the cable detection */
	chrgr->state.vbus = 0;
	if ((reg08 & VBUS_STAT_MASK) == VBUS_STAT_MASK) {
		pr_info("otg detection!\n");
		chrgr->state.vbus = 1;
	} else if (reg08 & BIT(7)) {
		pr_info("adapter port detected!\n");
		chrgr->state.vbus = 1;
	} else if (reg08 & BIT(6)) {
		pr_info("usb host detected!\n");
		chrgr->state.vbus = 1;
	}

	chrgr->state.health = (chrgr->state.vbus == VBUS_ON) ?
	    POWER_SUPPLY_HEALTH_GOOD : POWER_SUPPLY_HEALTH_UNKNOWN;

	if ((reg08 & CHRG_STAT_MASK) == CHRG_STAT_MASK)
		pr_info("charging done!\n");
	else if (reg08 & FAST_CHARGE_MASK)
		pr_info("fast charging!\n");
	else if (reg08 & PRE_CHARGE_MASK)
		pr_info("pre-charging!\n");
	else
		pr_info("not charging!\n");

	if (reg08 & DPM_STAT_MASK)
		pr_info("dpm detected!\n");
	if (reg08 & PG_STAT_MASK)
		pr_info("power good!\n");
	if (reg08 & THERM_STAT_MASK)
		pr_info("thermal regulation!\n");
	if (reg08 & VSYS_STAT_MASK)
		pr_info("vsysmin regulation! battery is too low!\n");

	if (reg09 & BIT(7)) {
		pr_info("wdt expirations!\n");
		bq24296_enable_charging(chrgr, false);
	}
	if (reg09 & BIT(6))
		pr_info("vbus ocp/ovp!\n");
	if ((reg09 & CHRG_FAULT_MASK) == CHRG_FAULT_MASK)
		bq24296_trigger_wtd(chrgr);
	else if (reg09 & BIT(5))
		pr_info("thermal shutdown!\n");
	else if (reg09 & BIT(4))
		pr_info("input fault! vbus ovp!\n");
	if (reg09 & BIT(3))
		pr_info("battery ovp! cv:%d\n", chrgr->state.cv);

	/* If vbus status changed, then notify USB OTG */
	pr_debug("vbus:%d, vbus_state_prev:%d health:%d health_prev:%d\n",
		 chrgr->state.vbus, vbus_state_prev, chrgr->state.health,
		 health_prev);

	if (health_prev != chrgr->state.health)
		power_supply_changed(chrgr->current_psy);

/* #ifndef CONFIG_POWER_SUPPLY_CHARGER_WITHOUT_USB */
	if (chrgr->state.vbus != vbus_state_prev) {
		if (chrgr->otg_handle) {
			pr_debug("USB_EVENT_VBUS - chrgr->state.vbus:%d\n",
				 chrgr->state.vbus);

			if (chrgr->state.vbus)
				event = USB_EVENT_VBUS;
			else
				event = USB_EVENT_NONE;

			atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
						   event, &chrgr->state.vbus);

			CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
					  CHG_DBG_VBUS, chrgr->state.vbus, 0);
		}
	}
/* #endif */

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

	wake_lock_init(&bh->evt_wakelock, WAKE_LOCK_SUSPEND, wakelock_name);

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

static irqreturn_t bq24296_charger_chgint_cb(int irq, void *dev)
{
	struct bq24296_charger *chrgr = dev;

	pr_debug("%s, irq=%d\n", __func__, irq);
	unfreezable_bh_schedule(&chrgr->chgint_bh);

	return IRQ_HANDLED;
}

/**
 * bq24296_configure_pmu_irq - function configuring PMU's Charger status IRQ
 * @chrgr		[in] pointer to charger driver internal structure
 */
static int bq24296_configure_pmu_irq(struct bq24296_charger *chrgr)
{
	int ret;

	/* register callback with PMU for CHGINT irq */
#ifdef CONFIG_CHARGER_BQ24296_WITHOUT_WAKEUP_BY_CHGINT
	ret = request_irq(chrgr->irq, bq24296_charger_chgint_cb,
			  0, BQ24296_NAME, chrgr);
#else
	ret = request_irq(chrgr->irq, bq24296_charger_chgint_cb,
			  IRQF_NO_SUSPEND, BQ24296_NAME, chrgr);
#endif

	if (ret != 0) {
		pr_err("Failed to register @PMU for GHGINT irq! ret=%d", ret);
		return ret;
	}

	return 0;
}

/**
 * bq24296_configure_pmu_regs -function configuring PMU's Charger
 *				part registers
 * @chrgr			[in] pointer to charger driver's internal struct
 */
static int bq24296_configure_pmu_regs(struct bq24296_charger *chrgr)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	int ret;

	if (!chrgr->ididev)
		return -EINVAL;

	pm_state_en =
	    idi_peripheral_device_pm_get_state_handler(chrgr->ididev, "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return -EINVAL;
	}

	pm_state_dis =
	    idi_peripheral_device_pm_get_state_handler(chrgr->ididev,
						       "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return -EINVAL;
	}

	pr_debug("%s: Getting PM state handlers: OK\n", __func__);

	ret = idi_set_power_state(chrgr->ididev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return -EIO;
	}

	ret = idi_client_ioread(chrgr->ididev,
				CHARGER_CONTROL(chrgr->ctrl_io_res->start),
				&regval);

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
				 CHARGER_CONTROL(chrgr->ctrl_io_res->start),
				 regval);

	/* charger control WR strobe */
	ret = idi_client_iowrite(chrgr->ididev,
				 CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
				 BIT(CHARGER_CONTROL_WR_WS_O));
	ret = idi_client_iowrite(chrgr->ididev,
				 CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
				 BIT(CHARGER_CONTROL_WR_WS_O));

	/* charger WR */
	ret = idi_client_ioread(chrgr->ididev,
				CHARGER_WR(chrgr->ctrl_io_res->start), &regval);

	/* ChargerReset - CHARGER_WR_CHGRST_NORESET */
	regval &= ~(CHARGER_WR_CHGRST_M << CHARGER_WR_CHGRST_O);
	regval |= (CHARGER_WR_CHGRST_NORESET << CHARGER_WR_CHGRST_O);

	ret = idi_client_iowrite(chrgr->ididev,
				 CHARGER_WR(chrgr->ctrl_io_res->start), regval);

	msleep(10);

	/* ChargerReset - CHARGER_WR_CHGRST_RESET */
	/* This pin is acive low so we need to reset it */
	regval &= ~(CHARGER_WR_CHGRST_M << CHARGER_WR_CHGRST_O);
	regval |= (CHARGER_WR_CHGRST_RESET << CHARGER_WR_CHGRST_O);

	ret = idi_client_iowrite(chrgr->ididev,
				 CHARGER_WR(chrgr->ctrl_io_res->start), regval);

	ret = idi_set_power_state(chrgr->ididev, pm_state_dis, false);

	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

	return 0;
}

/**
 * bq24296_configure_chip - function configuring BQ24296 chip registers
 * @chrgr		[in] pointer to charger driver internal structure
 * @enable_charging	[in] controls if charging should be anebled or disabled
 */
#define CHG_ENABLE_SHIFT  4
int bq24296_configure_chip(struct bq24296_charger *chrgr, bool enable_charging)
{
	CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_CONFIGURE_CHIP, 0, 0);

	int ret;
	u8 val = (u8) (!!enable_charging << CHG_ENABLE_SHIFT);

	pr_info("%s enable_charging=%d\n", __func__, enable_charging);

	if (!enable_charging) {
		/* Stop WDT timer to avoid WDT expiration */
		ret = bq24296_masked_write(chrgr->client,
					   BQ05_CHARGE_TERM_TIMER_CONT_REG,
					   I2C_TIMER_MASK, 0);
		if (ret)
			pr_err("failed to set Stop WDT ret=%d\n", ret);
	}

	ret = bq24296_masked_write(chrgr->client, BQ01_PWR_ON_CONF_REG,
				   CHG_CONFIG_MASK, val);
	if (ret) {
		pr_err("failed to set CHG_CONFIG ret=%d\n", ret);
		return ret;
	}

	chrgr->charging_disabled = !enable_charging;

	return 0;
}

static int bq24296_set_pinctrl_state(struct i2c_client *client,
				     struct pinctrl_state *state)
{
	struct bq24296_charger *chrgr = i2c_get_clientdata(client);
	int ret;

	ret = pinctrl_select_state(chrgr->pinctrl, state);
	if (ret != 0) {
		pr_err("failed to configure CHGRESET pin !\n");
		return -EIO;
	}

	return 0;
}

static int bq24296_hw_init(struct bq24296_charger *chrgr)
{
	int ret = 0;
	int value = 0;

	ret = bq24296_set_input_vin_limit(chrgr, chrgr->vbus_in_limit_mv);
	if (ret) {
		pr_err("%s: failed to set input voltage limit\n", __func__);
		return ret;
	}

	ret = bq24296_set_ibus_limit(chrgr, chrgr->max_vbus_ilimit, &value);
	if (ret) {
		pr_err("%s: failed to set input current limit\n", __func__);
		return ret;
	}

	ret = bq24296_set_system_vmin(chrgr, chrgr->sys_vmin_mv);
	if (ret) {
		pr_err("%s: failed to set system min voltage\n", __func__);
		return ret;
	}

	ret = bq24296_set_ibat_max(chrgr, chrgr->chg_current_ma);
	if (ret) {
		pr_err("%s: failed to set charging current\n", __func__);
		return ret;
	}
	pr_debug("%s:\n", __func__);
	bq24296_dump_all_reg(chrgr->client);

	ret = bq24296_set_prechg_ilimit(chrgr, chrgr->pre_chg_current_ma);
	if (ret) {
		pr_err("%s: failed to set pre-charge current\n", __func__);
		return ret;
	}

	ret = bq24296_set_term_current(chrgr, chrgr->term_current_ma);
	if (ret) {
#if 0
		pr_err("%s: failed to set charge termination current\n",
		       __func__);
#endif
		return ret;
	}

	ret = bq24296_set_vbat_max(chrgr, chrgr->vbat_max_mv);
	if (ret) {
		pr_err("%s: failed to set vbat max\n", __func__);
		return ret;
	}

	ret = bq24296_set_chg_term(chrgr, 1);
	if (ret) {
		pr_err("%s: failed to enable chg termination\n", __func__);
		return ret;
	}

	ret = bq24296_set_chg_timer(chrgr, 1);	/* Safety Timer */
	if (ret) {
		pr_err("%s: failed to enable chg safety timer\n", __func__);
		return ret;
	}

	ret = bq24296_set_chg_timeout(chrgr);	/* Fast Charge Timer */
	if (ret) {
		pr_err("Failed to set CHG_TIMEOUT rc=%d\n", ret);
		return ret;
	}

	ret = bq24296_set_chg_watchdog_timeout(chrgr);	/* WDT Timer */
	if (ret) {
		pr_err("Failed to set CHG_WATCGDOG rc=%d\n", ret);
		return ret;
	}
	return 0;
}

int bq24296_otg_notification_handler(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	struct bq24296_charger *chrgr =
	    container_of(nb, struct bq24296_charger, otg_nb);
	pr_debug("%s: event=%d\n", __func__, event);
	switch (event) {
	case INTEL_USB_DRV_VBUS:
		if (!data)
			return NOTIFY_BAD;
		chrgr->state.to_enable_boost = *((bool *) data);
		pr_debug("set chrgr->state.to_enable_boost=%d\n",
			 chrgr->state.to_enable_boost);
		unfreezable_bh_schedule(&chrgr->boost_op_bh);
		break;

	default:
		break;
	}

	return NOTIFY_OK;
}

static ssize_t dbg_logs_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	size_t size_copied;
	int value;

	value = bq24296_chrgr_dbg.printk_logs_en;
	size_copied = sprintf(buf, "%d\n", value);

	return size_copied;
}

static ssize_t dbg_logs_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
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

	bq24296_chrgr_dbg.printk_logs_en = sysfs_val;

	pr_debug("sysfs attr %s=%d\n", attr->attr.name, sysfs_val);

	return count;
}

static struct device_attribute dbg_logs_on_off_attr = {
	.attr = {
		 .name = "dbg_logs_on_off",
		 .mode = S_IRUSR | S_IWUSR,
		 },
	.show = dbg_logs_show,
	.store = dbg_logs_store,
};

/**
 * bq24296_setup_dbglogs_sysfs_attr	Sets up dbg_logs_on_off sysfs entry
 *					for debug logs control for bq24296
 *					i2c device
 * @dev					[in] pointer to device structure
 *
 */
static void bq24296_setup_dbglogs_sysfs_attr(struct device *dev)
{
	int err;

	err = device_create_file(dev, &dbg_logs_on_off_attr);
	if (err)
		pr_err("Unable to create sysfs entry: '%s'\n",
		       dbg_logs_on_off_attr.attr.name);
}

static int bq24296_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct usb_phy *otg_handle;
	struct bq24296_charger *chrgr;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	u8 ic_info;
	int ret;

#ifdef CONFIG_CHARGER_BQ24296_BPROFILE_FROM_DTS
#define BPROF_HEADER_SIZE 11
	int i;
	u32 temp_buf[BPROF_HEADER_SIZE];
#endif

	INIT_CHARGER_DEBUG_ARRAY(bq24296_chrgr_dbg);

	CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_I2C_PROBE, 0, 0);

	chrgr = (struct bq24296_charger *)id->driver_data;
	if (!chrgr)
		return -ENODEV;

#ifdef CONFIG_CHARGER_BQ24296_BPROFILE_FROM_DTS
	if (of_property_read_u32_array(np, "prof-standrd", temp_buf,
				       BPROF_HEADER_SIZE)) {
		pr_err("dt: parsing 'prof-standrd' failed\n");
		return -EINVAL;
	}

	i = 0;
	chrgr->chg_current_ma = temp_buf[i++];
	chrgr->term_current_ma = temp_buf[i++];
	chrgr->pre_chg_current_ma = temp_buf[i++];
	chrgr->force_ichg_decrease = temp_buf[i++];
	chrgr->vbat_full_mv = temp_buf[i++];
	chrgr->vbat_min_mv = temp_buf[i++];
	chrgr->vbat_max_mv = temp_buf[i++];
	chrgr->max_vbus_ilimit = temp_buf[i++];
	chrgr->min_vbus_ilimit = temp_buf[i++];
	chrgr->sys_vmin_mv = temp_buf[i++];
	chrgr->vbus_in_limit_mv = temp_buf[i++];
#endif

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

	chrgr->pins_inactive = pinctrl_lookup_state(chrgr->pinctrl, "inactive");
	if (IS_ERR(chrgr->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

	chrgr->pins_active = pinctrl_lookup_state(chrgr->pinctrl, "active");
	if (IS_ERR(chrgr->pins_active))
		dev_err(dev, "could not get active pinstate\n");

	/* Register i2c clientdata before calling pinctrl */
	i2c_set_clientdata(client, chrgr);

	pr_debug("%s\n", __func__);

	/* Read HW id */
	ret = bq24296_read_reg(client,
			       BQ0A_VENDOR_PART_REV_STATUS_REG, &ic_info);
	if (ret != 0) {
		pr_err("bq24296 read error, ret = %d\n", ret);
		return -ENODEV;
	}
	pr_err("IC_INFO reg = 0x%x\n", ic_info);
	/* Check if the HW is supported */
	if ((ic_info & 0x60) != chrgr->vendor) {
		pr_err("bq24296 info not correct\n");
		return -ENODEV;
	}

	chrgr->client = client;

	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg_handle == NULL) {
		pr_err("ERROR!: getting OTG transceiver failed\n");
		return -EINVAL;
	}
	chrgr->otg_handle = otg_handle;

	/* Wait for bq24296 idi device to map io */
	if (chrgr->ctrl_io_res == NULL)
		BUG();

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

	/*
	 * Get throttle from device tree
	 */
	if (of_property_read_u32(np,
				 "throttle-levels", &chrgr->throttle_levels)) {
		ret = -EINVAL;
		pr_err("can't get throttle levels\n");
		goto remap_fail;
	}

	chrgr->throttle_values =
	    (int *)(kzalloc(sizeof(int) * chrgr->throttle_levels, GFP_KERNEL));

	if (!chrgr->throttle_values) {
		ret = -ENOMEM;
		pr_err("alloc mem failed\n");
		goto remap_fail;
	}

	if (of_property_read_u32_array(np,
				       "throttle-values",
				       chrgr->throttle_values,
				       chrgr->throttle_levels)) {
		ret = -EINVAL;
		pr_err("can't get throttle values\n");
		kfree(chrgr->throttle_values);
		goto remap_fail;
	}

	/*
	Setup the PMU registers. The charger IC reset line
	is deasserted at this point. From this point onwards
	the charger IC will act upon the values stored in its
	registers instead of displaying default behaviour
	*/
	ret = bq24296_configure_pmu_regs(chrgr);
	if (ret != 0)
		goto pre_fail;

	/*
	Trigger charger watchdog to prevent expiry if the watchdog
	is about to expire due to a reset sequence having taken a long time.
	Triggering the watchdog if it has already expired has no effect,
	i.e. charging will not start again and the watchdog expiration
	flag in the interrupt register will not be cleared.
	*/
	ret = bq24296_trigger_wtd(chrgr);
	if (ret != 0)
		goto pre_fail;

	INIT_DELAYED_WORK(&chrgr->charging_work, bq24296_charging_worker);
	INIT_DELAYED_WORK(&chrgr->boost_work, bq24296_boost_worker);

	sema_init(&chrgr->prop_lock, 1);

	/* Set up the wake lock to prevent suspend when charging. */
	wake_lock_init(&chrgr->suspend_lock,
		       WAKE_LOCK_SUSPEND, "bq24296_wake_lock");

	if (unfreezable_bh_create(&chrgr->chgint_bh, "chrgr_wq",
				  "bq24296_evt_lock",
				  bq24296_chgint_cb_work_func)) {
		ret = -ENOMEM;
		goto wq_creation_fail;
	}

	ret = bq24296_hw_init(chrgr);
	if (ret != 0)
		pr_err("%s: bq24296_hw_init error !!!\n", __func__);
#if 0
	ret = bq24296_get_clr_wdt_expiry_flag(chrgr);
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
		pr_err("%s: Charging watchdog expiration detected!\n",
		       __func__);
		chrgr->state.charging_enabled = false;
	}
#endif

#ifdef CONFIG_POWER_SUPPLY_CHARGER_WITHOUT_USB
	ret = bq24296_enable_charging(chrgr, true);
	if (!ret)
		chrgr->state.charger_enabled = true;
#else
	ret = bq24296_configure_chip(chrgr, chrgr->state.charging_enabled);
#endif

	if (ret != 0) {
		pr_err("%s: Charging enable failed !!!\n", __func__);
		goto fail;
	}

	ret = bq24296_set_pinctrl_state(client, chrgr->pins_active);
	if (ret != 0)
		return ret;

	chrgr->usb_psy.name = "usb_charger";
	chrgr->usb_psy.type = POWER_SUPPLY_TYPE_USB;
	chrgr->usb_psy.properties = bq24296_power_props;
	chrgr->usb_psy.num_properties = ARRAY_SIZE(bq24296_power_props);
	chrgr->usb_psy.get_property = bq24296_charger_get_property;
	chrgr->usb_psy.set_property = bq24296_charger_set_property;
	chrgr->usb_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->usb_psy.supplied_to = bq24296_supplied_to;
	chrgr->usb_psy.num_supplicants = ARRAY_SIZE(bq24296_supplied_to);
	chrgr->usb_psy.throttle_states = bq24296_dummy_throttle_states;
	chrgr->usb_psy.num_throttle_states =
	    ARRAY_SIZE(bq24296_dummy_throttle_states);

	chrgr->current_psy = &chrgr->usb_psy;

	ret = power_supply_register(&client->dev, &chrgr->usb_psy);
	if (ret)
		goto fail;

	chrgr->ac_psy.name = "ac_charger";
	chrgr->ac_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chrgr->ac_psy.properties = bq24296_power_props;
	chrgr->ac_psy.num_properties = ARRAY_SIZE(bq24296_power_props);
	chrgr->ac_psy.get_property = bq24296_charger_get_property;
	chrgr->ac_psy.set_property = bq24296_charger_set_property;
	chrgr->ac_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->ac_psy.supplied_to = bq24296_supplied_to;
	chrgr->ac_psy.num_supplicants = ARRAY_SIZE(bq24296_supplied_to);
	chrgr->ac_psy.throttle_states = bq24296_dummy_throttle_states;
	chrgr->ac_psy.num_throttle_states =
	    ARRAY_SIZE(bq24296_dummy_throttle_states);

	ret = power_supply_register(&client->dev, &chrgr->ac_psy);
	if (ret)
		goto fail_ac_registr;

	chrgr->ack_time = jiffies;

	if (chrgr->state.charging_enabled)
		schedule_delayed_work(&chrgr->charging_work, 0);

	ret = bq24296_configure_pmu_irq(chrgr);
	if (ret != 0)
		goto pmu_irq_fail;

	i2c_set_clientdata(client, chrgr);

	bq24296_setup_debugfs(chrgr, &bq24296_chrgr_dbg);
	bq24296_setup_fake_vbus_sysfs_attr(chrgr);

	chrgr->state.status = BQ_CHG_STATUS_FAST_CHARGE;

	/* Read the VBUS presence status for initial update by
	   making a dummy interrupt bottom half invocation */
	queue_work(chrgr->chgint_bh.wq, &chrgr->chgint_bh.work);

	if (unfreezable_bh_create(&chrgr->boost_op_bh, "boost_op_wq",
				  "bq24296_boost_lock", bq24296_set_boost)) {
		ret = -ENOMEM;
		goto pmu_irq_fail;
	}

	ret = usb_register_notifier(otg_handle, &chrgr->otg_nb);
	if (ret) {
		pr_err("ERROR!: registration for OTG notifications failed\n");
		goto boost_fail;
	}
	bq24296_setup_dbglogs_sysfs_attr(&client->dev);

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
pre_fail:
wq_creation_fail:
remap_fail:
	usb_put_phy(otg_handle);
	return ret;
}

static int __exit bq24296_i2c_remove(struct i2c_client *client)
{
	int ret = 0;
	struct bq24296_charger *chrgr = i2c_get_clientdata(client);

	CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_I2C_REMOVE, 0, 0);

	free_irq(client->irq, chrgr);
	power_supply_unregister(&chrgr->usb_psy);
	power_supply_unregister(&chrgr->ac_psy);
	wake_lock_destroy(&chrgr->suspend_lock);

	unfreezable_bh_destroy(&chrgr->chgint_bh);
	unfreezable_bh_destroy(&chrgr->boost_op_bh);

	cancel_delayed_work_sync(&chrgr->charging_work);
	if (chrgr->otg_handle)
		usb_put_phy(chrgr->otg_handle);

	ret = bq24296_set_pinctrl_state(client, chrgr->pins_inactive);
	if (ret != 0)
		return ret;
	bq24296_remove_debugfs_dir(chrgr);

	return 0;
}

static void idi_init(struct bq24296_charger *chrgr,
		     struct resource *ctrl_io_res,
		     struct idi_peripheral_device *ididev)
{
	chrgr->ctrl_io_res = ctrl_io_res;
	chrgr->ididev = ididev;
}

static int bq24296_idi_probe(struct idi_peripheral_device *ididev,
			     const struct idi_device_id *id)
{
	struct resource *res;
	int ret = 0;

	spin_lock_init(&bq24296_chrgr_dbg.lock);

	CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_IDI_PROBE, 0, 0);

	res = idi_get_resource_byname(&ididev->resources,
				      IORESOURCE_MEM, "registers");

	if (res == NULL) {
		pr_err("getting PMU's Charger registers resources failed!\n");
		return -EINVAL;
	}

	/* HACK: chargers sharing same IDI device ID */
	idi_init(&bq24296_chrgr_data, res, ididev);

	ret = idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
		       __func__);
		return ret;
	}

	return 0;
}

static int __exit bq24296_idi_remove(struct idi_peripheral_device *ididev)
{
	CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_IDI_REMOVE, 0, 0);
	pr_debug("%s\n", __func__);

	return 0;
}

/**
 * bq24296_suspend() - Called when the system is attempting to suspend.
 * If charging is in progress EBUSY is returned to abort the suspend and
 * an error is logged, as the wake lock should prevent the situation.
 * @dev		[in] Pointer to the device.(not used)
 * returns	EBUSY if charging is ongoing, else 0
 */
static int bq24296_suspend(struct device *dev)
{
	struct bq24296_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));

	pr_debug("%s charging_enable:%d\n", __func__,
		 chrgr->state.charging_enabled);

#ifdef CONFIG_CHARGER_BQ24296_SUSPEND_WITH_CHARGING
	if (0) {
#else
	if (chrgr->state.charging_enabled) {
#endif
		/* If charging is in progess, prevent suspend. */
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg,
				  CHG_DBG_SUSPEND_ERROR, 0, 0);
		return -EBUSY;
	} else {
		/* Not charging - allow suspend. */
		CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_SUSPEND_OK, 0, 0);
		if (device_may_wakeup(dev)) {
#ifndef CONFIG_CHARGER_BQ24296_WITHOUT_WAKEUP_BY_CHGINT
			pr_debug("bq: enable wakeirq\n");
			enable_irq_wake(chrgr->irq);
#endif
		}
		unfreezable_bh_suspend(&chrgr->chgint_bh);
		unfreezable_bh_suspend(&chrgr->boost_op_bh);

		return 0;
	}
}

/**
 * bq24296_resume() - Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int bq24296_resume(struct device *dev)
{
	struct bq24296_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
	int ret = 0;

	CHARGER_DEBUG_REL(bq24296_chrgr_dbg, CHG_DBG_RESUME, 0, 0);

	pr_debug("%s charging_enable:%d\n", __func__,
		 chrgr->state.charging_enabled);
	ret = bq24296_configure_chip(chrgr, true);
	if (ret != 0)
		pr_err("%s: Charging enabled failed !!!\n", __func__);

	unfreezable_bh_resume(&chrgr->chgint_bh);
	unfreezable_bh_resume(&chrgr->boost_op_bh);
	if (device_may_wakeup(dev)) {
#ifndef CONFIG_CHARGER_BQ24296_WITHOUT_WAKEUP_BY_CHGINT
		pr_debug("bq: disable wakeirq\n");
		disable_irq_wake(chrgr->irq);
#endif
	}

	return 0;
}

const struct dev_pm_ops bq24296_pm = {
	.suspend = bq24296_suspend,
	.resume = bq24296_resume,
};

static const struct i2c_device_id bq24296_id[] = {
	{"bq24296_charger", (kernel_ulong_t)&bq24296_chrgr_data},
	{}
};

MODULE_DEVICE_TABLE(i2c, bq24296_id);

static struct i2c_driver bq24296_i2c_driver = {
	.probe = bq24296_i2c_probe,
	.remove = bq24296_i2c_remove,
	.id_table = bq24296_id,
	.driver = {
		   .name = BQ24296_NAME,
		   .owner = THIS_MODULE,
		   .pm = &bq24296_pm,
		   },
};

static const struct idi_device_id idi_ids[] = {
	{
	 .vendor = IDI_ANY_ID,
	 .device = IDI_DEVICE_ID_INTEL_AG620,
	 .subdevice = IDI_SUBDEVICE_ID_INTEL_CHG,
	 },

	{ /* end: all zeroes */ },
};

static struct idi_peripheral_driver bq24296_idi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "chgr_idi",
		   .pm = NULL,
		   },
	.p_type = IDI_CHG,
	.id_table = idi_ids,
	.probe = bq24296_idi_probe,
	.remove = bq24296_idi_remove,
};

static int __init bq24296_init(void)
{
	int ret;

	ret = idi_register_peripheral_driver(&bq24296_idi_driver);
	if (ret)
		return ret;

	ret = i2c_add_driver(&bq24296_i2c_driver);
	if (ret)
		return ret;

	return 0;
}

late_initcall(bq24296_init);

static void __exit bq24296_exit(void)
{
	i2c_del_driver(&bq24296_i2c_driver);
}

module_exit(bq24296_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Charger Driver for BQ24296 charger IC");
