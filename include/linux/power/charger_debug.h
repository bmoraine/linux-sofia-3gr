/**
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

#ifndef _CHARGER_DEBUG_H
#define _CHARGER_DEBUG_H

#include <linux/ktime.h>
#include <linux/spinlock.h>

/* Size of debug data array (has to be power of 2!!!) */
#define CHARGER_DEBUG_DATA_SIZE (1<<9)

#define CHARGER_DEBUG(_array, _event, _event_str, _param, _param2) \
do { \
	spin_lock(&_array.lock); \
	_array.log_array[_array.write_index].time_stamp_jiffies = \
					(jiffies - INITIAL_JIFFIES); \
	_array.log_array[_array.write_index].event = (_event); \
	_array.log_array[_array.write_index].event_string = _event_str; \
	_array.log_array[_array.write_index].param = (long)(_param); \
	_array.log_array[_array.write_index].param2 = (long)(_param2); \
	_array.write_index++; \
	_array.write_index &= (CHARGER_DEBUG_DATA_SIZE-1); \
	_array.count = (_array.count == CHARGER_DEBUG_DATA_SIZE) ? \
						_array.count : \
						_array.count + 1; \
	\
	_array.read_index = (_array.count == CHARGER_DEBUG_DATA_SIZE) ? \
						_array.write_index : \
						_array.read_index; \
	pr_debug("%s param=0x%lx, param2=0x%lx\n", \
			_event_str, (long)_param, (long)_param2); \
	spin_unlock(&_array.lock); \
} while (0)



/* Macro to log debug data for release. Jiffy resolution is adequate
   for charger debugging. The value will then be converted to s.ms when creating
   a content of events_log file in debugfs */
#define CHARGER_DEBUG_REL(_array, _event, _param, _param2) \
		CHARGER_DEBUG(_array, _event, #_event, _param, _param2)


#ifdef CHARGER_DEBUG_DEVELOPMENT

/* Macro to log debug data for development. Jiffy resolution is adequate
   for charger debugging. The value will then be converted to s.ms when creating
   a content of events_log file in debugfs */
#define CHARGER_DEBUG_DEV(_array, _event, _param, _param2) \
		CHARGER_DEBUG(_array, _event, #_event, _param, _param2)

/* Macro to log i2c reads for development debugging. Jiffy resolution is
 * adequate for charger debugging. The value will then be converted to s.ms
 * when creating a content of events_log file in debugfs
 */
#define CHARGER_DEBUG_READ_REG(_array, _addr, _val) \
		CHARGER_DEBUG(_array, CHG_DBG_REG, "\t\t-------> READ: ",\
								_addr, _val)

/* Macro to log i2c writes for development debugging. Jiffy resolution is
 * adequate for charger debugging. The value will then be converted to s.ms whe
 * creating a content of events_log file in debugfs */
#define CHARGER_DEBUG_WRITE_REG(_array, _addr, _val) \
		CHARGER_DEBUG(_array, CHG_DBG_REG, "\t\t-------> WRITE: ",\
								_addr, _val)

#else

#define CHARGER_DEBUG_DEV(_array, _event, _param, _param2) \
do { } while (0)

#define CHARGER_DEBUG_READ_REG(_array, _addr, _val) \
do { } while (0)

#define CHARGER_DEBUG_WRITE_REG(_array, _addr, _val) \
do { } while (0)

#endif

#define INIT_CHARGER_DEBUG_ARRAY(_array) \
do { \
	spin_lock_init(&_array.lock); \
	_array.read_index = 0; \
	_array.write_index = 0; \
	_array.count = 0; \
} while (0)

/** Events for use in debug and tracing. */
enum charger_debug_event {
	/* Events with no parameters */

	CHG_DBG_NO_PARAM_EVENT,

	CHG_DBG_I2C_PROBE,
	CHG_DBG_IDI_PROBE,
	CHG_DBG_I2C_REMOVE,
	CHG_DBG_IDI_REMOVE,
	CHG_DBG_SUSPEND_OK,
	CHG_DBG_SUSPEND_ERROR,
	CHG_DBG_RESUME,
	CHG_DBG_CONFIGURE_CHIP,
	CHG_DBG_CHGINT,
	CHG_DBG_CHGINT_CB,

	CHG_DBG_TRIG_POWER_SUPPLY_CHARGER,
	CHG_DBG_T32_TIMER_EXPIRED,
	CHG_DBG_DRIVER_IN_FAULT_STATE,
	CHG_DBG_VBUS_OVP,
	CHG_DBG_VBUS_OVP_RECOV,
	CHG_DBG_TREG_IS_ON,
	CHG_DBG_TSD_IS_ON,
	CHG_DBG_OT_RECOVERY,

	CHG_DBG_SLEEP,
	CHG_DBG_POOR_INPUT_SRC,
	CHG_DBG_VBAT_OVP,
	CHG_DBG_NO_BAT,

	CHG_DBG_TRIGGERING_WTD,

	CHG_DBG_LAST_NO_PARAM_EVENT,

	CHG_DBG_BOOST_ENABLED,
	CHG_DBG_BOOST_DISABLED,

	/* Events with one paramater */
	CHG_DBG_VBUS_FAULT,

	CHG_DBG_VBUS,
	CHG_DBG_FAKE_VBUS,

	CHG_DBG_GET_PROP_PRESENT,
	CHG_DBG_GET_PROP_ONLINE,
	CHG_DBG_GET_PROP_HEALTH,
	CHG_DBG_GET_PROP_TYPE,
	CHG_DBG_GET_PROP_ENABLE_CHARGER,
	CHG_DBG_GET_PROP_ENABLE_CHARGING,
	CHG_DBG_GET_PROP_CABLE_TYPE,
	CHG_DBG_GET_PROP_CHARGE_CURRENT,
	CHG_DBG_GET_PROP_MAX_CHARGE_CURRENT,
	CHG_DBG_GET_PROP_CHARGE_VOLTAGE,
	CHG_DBG_GET_PROP_CHARGE_CONTROL_LIMIT,
	CHG_DBG_GET_PROP_CHARGE_CONTROL_LIMIT_MAX,
	CHG_DBG_GET_PROP_INLMT,
	CHG_DBG_GET_PROP_CHARGE_TERM_CUR,
	CHG_DBG_GET_PROP_PRIORITY,
	CHG_DBG_GET_PROPERTY_ERROR,

	CHG_DBG_SET_PROP_CHARGE_CURRENT,
	CHG_DBG_SET_PROP_CHARGE_VOLTAGE,
	CHG_DBG_SET_PROP_CHARGE_CONTROL_LIMIT,
	CHG_DBG_SET_PROP_INLMT,
	CHG_DBG_SET_PROP_CHARGE_TERM_CUR,
	CHG_DBG_SET_PROP_ENABLE_CHARGER,
	CHG_DBG_SET_PROP_ENABLE_CHARGING,
	CHG_DBG_SET_PROP_CABLE_TYPE,
	CHG_DBG_SET_PROP_CONTINUE_CHARGING,
	CHG_DBG_SET_PROP_MIN_TEMP,
	CHG_DBG_SET_PROP_MAX_TEMP,
	CHG_DBG_SET_PROPERTY_ERROR,
	CHG_DBG_SET_PROPERTY_VALUE_SET,

	CHG_DBG_I2C_READ_ERROR,
	CHG_DBG_I2C_WRITE_ERROR,
	CHG_DBG_REG,
};

/**
 * struct charger_debug_data - Structure to collect debug data
 * @lock			Spinlock for atomic access
 * @index			Index of logging array
 * @log_array			Debug data logging array
 * @time_stamp_jiffies		System Time Stamp in Jiffies
 * @event			Event which occurred
 * @event_string		String representation of event
 * @param			General purpose parameter
 * @param2			General purpose second parameter
 */
struct charger_debug_data {
	int		printk_logs_en;
	spinlock_t      lock;
	u32             read_index;
	u32             write_index;
	u32             count;
	struct {
		unsigned long			time_stamp_jiffies;
		enum charger_debug_event	event;
		const char			*event_string;
		long			param;
		long			param2;
	} log_array[CHARGER_DEBUG_DATA_SIZE];
};

#endif /* _CHARGER_DEBUG_H */
