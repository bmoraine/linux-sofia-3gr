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

#define DRIVER_NAME				"sw_fuel_gauge"
#define pr_fmt(fmt) DRIVER_NAME": "fmt

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kfifo.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/hrtimer.h>

#include <linux/power_supply.h>
#include <linux/power/battery_id.h>
#include <linux/power/sw_fuel_gauge_debug.h>
#include <linux/power/sw_fuel_gauge_hal.h>
#include <linux/power/sw_fuel_gauge_nvs.h>

#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>

#include <linux/wakelock.h>

/* Conversion factor for mAh to mC */
#define SCALE_MAH_TO_MC				(3600)
/* Number of elements for Capacity (%)  to Cell voltage (mV) table */
#define BAT_CAP_TO_VBAT_TABLE_SIZE		(101)
/* Percentage capacity for a full battery. */
#define POWER_SUPPLY_CAPACITY_1000_PERMIL	(1000)

/*
 * Maximum value for charge remaining (mAh).
 * Used when no battery is fitted to reduce.
 * capacity depletion due to the coulomb counter
 * between calibration points when powered by
 * a PSU.
 */
#define POWER_SUPPLY_CHARGE_MAX_MAH		(500000)

/* Maximum useful life time for NVM calibration data. */
#define SECS_PER_HOUR				(60 * 60)
#define SECS_PER_DAY				(SECS_PER_HOUR * 24)
#define MAX_CALIBRATION_POINT_AGE_DAYS		(1)
#define MAX_CALIBRATION_POINT_AGE_SECS \
				(MAX_CALIBRATION_POINT_AGE_DAYS * SECS_PER_DAY)

/* Maximum time between immediate NVM calibration data writes in hours. */
#define NVM_IMMEDIATE_MAX_AGE_HOURS		(12)
/* Maximum time between immediate NVM calibration data writes in seconds. */
#define NVM_IMMEDIATE_MAX_AGE_SECS \
				(NVM_IMMEDIATE_MAX_AGE_HOURS * SECS_PER_HOUR)

/* Dummy NVM data for unavailable data. */
#define NVM_NO_VALUE				(0x7FFFFFFF)
/* Maximum allowed calibration point error. */
#define NVM_IMMEDIATE_ERR_MAX_PERMIL		(50)
/* Level below the max allowed error, where NVM write is triggered. */
#define NVM_IMMEDIATE_ERR_BUFFER_PERMIL		(20)
/* Threshold of accumulated error to trigger immediate NVM cal point write. */
#define NVM_IMMEDIATE_ERR_THRESHOLD_PERMIL \
		(NVM_IMMEDIATE_ERR_MAX_PERMIL - NVM_IMMEDIATE_ERR_BUFFER_PERMIL)

/* High initial accumulated error to force an early calibration point. */
#define INITIAL_VBAT_TYP_ERROR_PERMIL		(500)

/* Multiplier / Divisor for percentage calculations. */
#define SCALE_PERCENT				(100)
/* Multiplier / Divisor for permil calculations. */
#define SCALE_PERMIL				(1000)
/* Scaling needed for battery temperature. POWER_SUPPLY_PROP_TEMP
is in tenths of a degree. */
#define SCALE_DECI_DEG_C			(10)

/* Temperature of 0 Kelvins in deci-degrees Celcius to mark
variable holding temperature as uninitialized */
#define TEMP_0_KELVIN_DDEGC			(-2730)

/* Multiplier / Divisor for milli-micro calculations. */
#define SCALE_MILLI				(1000)

/* Divisors for proportion of battery capacity to use for relaxed
current limits. */
#define SCALE_IBAT_AVG_RELAXED			(SCALE_PERCENT * 2)
#define SCALE_IBAT_NOW_RELAXED			(SCALE_PERCENT / 4)

/* Allowed consecutive TBAT sensor overflow errors. */
#define TBAT_MAX_OVERFLOW_ERROR_COUNT		(2)

/* Allowed consecutive TBAT sensor io errors.
I/O error can be caused by system suspend which
will not happen while in charging  */
#define TBAT_MAX_IO_ERROR_COUNT			(4)

/* Maximum allowed time before long term ibat load measurements must be
available after startup. */
#define MAX_IBAT_LONG_TERM_FAILURE_TIME		(600)

/*
 * Size of array of function payloads to implement work FIFO.
 */
#define WORK_FIFO_LENGTH			(32)

/* Macro to trace and log debug event and data. */
#define SW_FUEL_GAUGE_DEBUG_PARAM(_event, _param) \
	SWFG_DEBUG(sw_fuel_gauge_debug.dbg_array, _event, _param)

/* Macro to trace and log debug event without a parameter. */
#define SW_FUEL_GAUGE_DEBUG_NO_PARAM(_event) \
	SWFG_DEBUG(sw_fuel_gauge_debug.dbg_array, _event, 0)

/* Macro to log debug event with a parameter but no printk. */
#define SW_FUEL_GAUGE_DEBUG_NO_LOG_PARAM(_event, _param) \
	SWFG_DEBUG_NO_PRINTK(sw_fuel_gauge_debug.dbg_array, \
							_event, _param)

/* Macro to log debug event without a parameter or printk. */
#define SW_FUEL_GAUGE_DEBUG_NO_LOG_NO_PARAM(_event) \
	SWFG_DEBUG_NO_PRINTK(sw_fuel_gauge_debug.dbg_array, _event, 0)

#define SW_FUEL_GAUGE_ENQUEUE(p_func, param) \
	sw_fuel_gauge_enqueue_function((fp_scheduled_function)(p_func), \
								(long)(param))


/* Message payload for work scheduler queue. */
struct sw_fuel_gauge_fifo_payload {
	fp_scheduled_function	p_func;
	long		param;
};

/*
 * Message queue for scheduled work.
 * This is a ring buffer protected by a spinlock,
 * as access is required from interrupt and timer callback context.
 */
struct sw_fuel_gauge_work_fifo {
	DECLARE_KFIFO(fifo, struct sw_fuel_gauge_fifo_payload,
							WORK_FIFO_LENGTH);
	spinlock_t		lock;

	struct wake_lock kfifo_wakelock;

	struct workqueue_struct	*p_work_queue;
	struct work_struct	work;
	bool			removal_pending;

	struct mutex		deferred_exec_lock;
	bool			pm_prepare;
	bool			pending_dequeue;
};

/* State machine events. */
enum sw_fuel_gauge_stm_event {
	SW_FUEL_GAUGE_STM_EVENT_INITIAL_SOC_REPORT_DONE,
	SW_FUEL_GAUGE_STM_EVENT_BATTERY_NOLONGER_RELAXED,
	SW_FUEL_GAUGE_STM_EVENT_BATTERY_RELAXED,
	SW_FUEL_GAUGE_STM_EVENT_SOC_UPDATE,
	SW_FUEL_GAUGE_STM_EVENT_OCV_MEAS_DONE,
	SW_FUEL_GAUGE_STM_EVENT_OCV_DONE,
	SW_FUEL_GAUGE_STM_EVENT_EOC,
	SW_FUEL_GAUGE_STM_EVENT_SYSTEM,
};

/* State machine states. */
enum sw_fuel_gauge_stm_state {
	SW_FUEL_GAUGE_STM_STATE_UNITIALISED,
	SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC,
	SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED,
	SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV,
	SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED,
	SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE,
};

/* index for bat_type_strings array  */
enum bat_id_type {
	BAT_ID_STANDARD = 0,
	BAT_ID_PSU,
	BAT_ID_TEST,
	BAT_ID_UNINIT_NVM,
	BAT_ID_UNKNOWN,
	BAT_ID_MAX,
};

/**
 * struct bat_driver_data - Battery data including presence and model.
 * @initialised			True when the battery model data has been
 *				initialised
 * @fitted_state		True when a battery is fitted, else false
 * @bat_id_type			Enumerated version of battery ID string for
 *				easy comparison.
 * @p_fitted_model		Pointer to the data of the fitted battery.
 */
struct bat_driver_data {
	bool			initialised;
	bool			fitted_state;
	enum bat_id_type	bat_id_type;
	struct ps_pse_mod_prof	*p_fitted_model;
};

/* Battery temperature filter data, Unit: degC */
struct bat_temperature_data {
	bool	initialised;
	int	single_degc;
	u32	bat_temperature_overflow_error_count;
	u32	bat_temperature_io_error_count;
	struct iio_channel *p_iio_tbat;
	int	temperature_notif_ddegc;
};

/**
 * struct bat_voltage_data - Battery voltage data.
 * @vbat_typ_mv		VBAT TYP sensor value for measurement at boot time. (mV)
 * @vbat_ocv_mv		VBAT OCV sensor value for non-boot time
 *			measurements. (mV)
 * @error_code		VBAT sensor error code
 * @accuracy_mv		VBAT sensor absolute accuracy (mV)
 * @p_iio_vbat_typ	IIO channels for VBAT TYP
 * @p_iio_vbat_ocv	IIO channels for VBAT OCV
 */
struct bat_voltage_data {
	int	vbat_typ_mv;
	int	vbat_ocv_mv;
	int	error_code;
	int	accuracy_mv;
	struct iio_channel *p_iio_vbat_typ;
	struct iio_channel *p_iio_vbat_ocv;
};

/* Values of properties registered with the power supply class. */
struct power_supply_properties {
	/* UNKNOWN enum is available for these properties. */
	int	status;
	int	health;
	int	technology;

	/* true after the battery driver HAL has reported. */
	bool	battery_data_valid;
	int	present;
	int	charge_full_design;

	/* true after battery capacity has been calculated. */
	bool	capacity_valid;
	int	internal_capacity;
	int	reported_capacity;
	int	voltage_ocv;
	int	charge_now;

	/* true after battery temperature has been calculated. */
	bool	temperature_valid;
	int	temperature;

	/* NULL pointer indicates unknown. */
	const char *model_name;
	const char *manufacturer;
	const char *serial_number;

	/* For atomic access in different process contexts. */
	struct semaphore lock;
};

/*
 * struct state_machine_data 0 State machine information.
 * @state	Current state machine state
*/
struct state_machine_data {
	enum sw_fuel_gauge_stm_state	state;
};

/**
 * struct sw_fuel_gauge_data - SW Fuel Gauge control structure
 *
 * @p_platform_device			Pointer to platform device
 * @power_supply_bat			Battery power supply registered with
 *					power supply class
 * @p_hal_interface			Pointer to HAL interface
 * @notifier				Notifier block for battery driver
 * @bat					Battery data supplied by the battery
 *					driver HAL
 * @vbat				Battery voltage sensor data
 * @tbat				Battery temperature data and timer
 * @properties				Values of properties supplied to power
 *					supply class
 * @stm					Data used by the state machine
 * @nvm_valid_at_init			Indicates whether initial SoC was
 *					determined from the NVM cal point.
 * @nom_bat_capacity_mc			Nominal capacity of the fitted
 *					battery (mC)
 * @hal_set				Reporting delta threshold for coulomb
 *					counter (mC)
 * @cc_balanced_mc			Reported coulomb counter reading (mC)
 * @cc_acc_error_mc			Accumulated coulomb counter error (mC)
 * @ibat_load_avg_relaxed_limit_ma	Average current level at which the
 *					battery is deemed relaxed (mA)
 * @ibat_load_now_relaxed_limit_ma	Current Now level at which the
 *					battery is deemed relaxed (mA)
 *
 * @ibat_long_term_failures		Count of consecutive ibat long term
 *					current reading failures
 *
 * @latest_calculated_capacity_permil	The latest battery capacity calculated
 *					value (permil)
 * @latest_calculated_error_permil	The latest battery capacity error
 *					value (permil)
 * @nvm_immediate_write_required	True if NVM immediate write must be
 *					done to prevent error threshold breach.
 * @nvm_immediate_write_acc_error_mc	Accumulated coulomb counter error since
 *					last immediate calibration point (mC)
 * @nvm_immediate_last_cal_point_index	Index incremented every time a
 *					calibartion point is written
 * @last_soc_cal_index			Index which is incremented every time a
 *					calibration point is written
 * @last_soc_cal			Most recent calibration point
 * @last_immediate_cal			Most recent immediate NVM calibration
 *					point
 */
struct sw_fuel_gauge_data {
	struct platform_device		p_platform_device;
	struct power_supply		power_supply_bat;
	struct sw_fuel_gauge_hal_interface *p_hal_interface;
	struct notifier_block		notifier;
	struct bat_driver_data		bat;
	struct bat_voltage_data		vbat;
	struct bat_temperature_data	tbat;
	struct power_supply_properties	properties;
	struct state_machine_data	stm;
	bool				nvm_valid_at_init;
	int				nom_bat_capacity_mc;
	union sw_fuel_gauge_hal_set_params hal_set;
	int				cc_balanced_mc;
	int				cc_acc_error_mc;
	int				ibat_load_avg_relaxed_limit_ma;
	int				ibat_load_now_relaxed_limit_ma;
	struct timespec			hal_reg_timestamp;
	int				latest_calculated_capacity_permil;
	int				latest_calculated_error_permil;
	bool				nvm_immediate_write_required;
	int				nvm_immediate_write_acc_error_mc;
	u32				nvm_immediate_last_cal_point_index;
	u32				last_soc_cal_index;
	struct soc_cal_point		last_soc_cal;
	struct soc_cal_point		last_immediate_cal;
	int				charger_target_mv;
};

struct sw_fuel_gauge_debug {
	struct sw_fuel_gauge_debug_data dbg_array;
	struct {
		/* Nr of OCV measurements that ended up in EAGAIN error */
		uint again_error_cnt;
		/* Nr of OCV measurements that ended up in EIO error */
		uint io_error_cnt;
		/* kfifo queue high watermark */
		uint kfifo_high_watermark;
	} stats;
};

/*
 * Battery ID type strings
 */
static const char *bat_id_strings[BAT_ID_MAX] = {
	"STANDRD",		/* Battery is a recognised real battery. */

	"PWRSPLY",		/* No battery is detected. Must be being powered
				from a power supply. */

	"TEST",			/* Test battery or test jig is fitted */

	"NO_NNVM",		/* NVM battery model data uninitialised.
				Charging should not be enabled */

	"UNKNOWN",		/* Battery id is unrecognised. Charging should
				not be enabled */
};

/* Prototype for functions exported to the HAL. */
static void sw_fuel_gauge_enqueue_function(
				fp_scheduled_function p_function, long param);

static void sw_fuel_gauge_cc_hal_callback(enum sw_fuel_gauge_hal_cb_event event,
					union sw_fuel_gauge_hal_cb_param param);

/* Prototype for power supply class registered functions. */
static int sw_fuel_gauge_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val);

static int sw_fuel_gauge_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val);

/* Prototype for Batt ID notification registered function. */
static int sw_fuel_gauge_bat_id_notification(struct notifier_block *notifier,
						unsigned long val,
						void *p_data);

/* Prototype for state machine event processing. */
static void sw_fuel_gauge_stm_process_event(enum sw_fuel_gauge_stm_event event);

/* List of properties supplied to power supply class. */
static enum power_supply_property sw_fuel_gauge_battery_props[] = {
	/* Required for Android Battery Service. */
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,

	/* Additional Supported Properties. */
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,

	/* Strings. */
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

/* Functions exported to the HAL */
static struct sw_fuel_gauge_interface sw_fuel_gauge_exported_functions = {
	.event_cb = sw_fuel_gauge_cc_hal_callback,
	.enqueue = sw_fuel_gauge_enqueue_function,
};

/* SW Fuel Gauge instance */
static struct sw_fuel_gauge_data sw_fuel_gauge_instance = {
	.power_supply_bat = {
		.name			= "battery",
		.type			= POWER_SUPPLY_TYPE_BATTERY,
		.properties		= sw_fuel_gauge_battery_props,
		.num_properties		= ARRAY_SIZE(
						sw_fuel_gauge_battery_props),

		.get_property		= sw_fuel_gauge_get_property,
		.set_property		= sw_fuel_gauge_set_property,
		},
	.properties = {
		.status		= POWER_SUPPLY_STATUS_UNKNOWN,
		.health		= POWER_SUPPLY_HEALTH_UNKNOWN,
		.technology	= POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
		.capacity_valid	= false,
		.temperature_valid = false,
		.battery_data_valid = false,
		.manufacturer	= NULL,
		.serial_number	= NULL,
		.model_name	= NULL,
	},
	.notifier = {
		.notifier_call = sw_fuel_gauge_bat_id_notification,
	},
	.tbat = {
		.bat_temperature_overflow_error_count = 0,
		.bat_temperature_io_error_count = 0,
		.temperature_notif_ddegc = TEMP_0_KELVIN_DDEGC,
	},
};

/* Work and message queue for driver serialisation. */
static struct sw_fuel_gauge_work_fifo sw_fuel_gauge_work = {
	.removal_pending = false,
	.pm_prepare = false,
	.pending_dequeue = false,
};

/* Array to collect debug data */
static struct sw_fuel_gauge_debug sw_fuel_gauge_debug = {
	.stats = {
		.again_error_cnt = 0,
		.io_error_cnt = 0,
		.kfifo_high_watermark = 0,
	},
};

/**
 * scale_and_divide_with_rounding_s32 - Scales, divides and rounds using
 *					64 bit arithmetic.
 *					This function needed to implement the
 *					following macro, as 64 Division
 *					is not supported
 *					with the standard C operator:
 *
 * SCALE_AND_DIVIDE_WITH_ROUNDING_S32(_s,_n,_d)
 *		((int)( ( ( ( (s64)(_s) * (s64)(_n) * 2) / (_d)) +
						( (_n) < 0 ? -1 : 1 ) ) /2 ))
 *
 * @scale	[in] Scaling factor, e.g. 100 for percent, 1000 for permil.
 * @num		[in] Numerator
 * @div		[in] Divisor
 */
static int scale_and_divide_with_rounding_s32(int scale, int num, int div)
{
	s64 mul64 = (s64)scale * (s64)num * 2;
	int remainder;
	int sign = (num < 0)
			? -1
			: 1;

	return ((int)div_s64_rem(mul64, div, &remainder) + sign) / 2;
}

/**
 * sw_fuel_gauge_eoc_handler	stores the charging target voltage and creates
 *				eoc event
 * @p_function		[in] Function to be added to the execution FIFO.
 * @param		[in] Parameter value for the function.
 */
static void sw_fuel_gauge_eoc_handler(int target_voltage_mv)
{
	/* Store the charging target voltage */
	sw_fuel_gauge_instance.charger_target_mv = target_voltage_mv;

	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_CHARGER_TARGET_MV,
				sw_fuel_gauge_instance.charger_target_mv);

	/* Schedule the EOC event in the state machine */
	sw_fuel_gauge_stm_process_event(SW_FUEL_GAUGE_STM_EVENT_EOC);
}


/**
 * sw_fuel_gauge_enqueue_function -	Adds a function to the message queue for
 *					the serialisation work.
 *					This function is supplied to the HAL to
 *					allow processing in a single thread work
 *					for static data.
 *					Since it may be called from interrupt or
 *					resume context, the locking mechanism is
 *					spinlock.
 *
 * @p_function		[in] Function to be added to the execution FIFO.
 * @param		[in] Parameter value for the function.
 */
static void sw_fuel_gauge_enqueue_function(
				fp_scheduled_function p_function, long param)
{
	unsigned long flags;

	/* Functions may not be scheduled while the device is being removed. */
	if (!sw_fuel_gauge_work.removal_pending) {
		struct sw_fuel_gauge_fifo_payload payload = {
			.p_func = p_function,
			.param = param
		};
		uint fifo_length = 0;

		spin_lock_irqsave(&sw_fuel_gauge_work.lock, flags);

		/*
		 * Message queue is a critical region.
		 * kfifo() needs explicit locking when there
		 * are multiple consumers or producers.
		 */
		BUG_ON(0 == kfifo_in(&sw_fuel_gauge_work.fifo,
						&payload,
						1));

		fifo_length = kfifo_len(&sw_fuel_gauge_work.fifo);

		if (fifo_length > sw_fuel_gauge_debug.
					stats.kfifo_high_watermark)
			sw_fuel_gauge_debug.stats.kfifo_high_watermark =
								fifo_length;

		if (fifo_length >= WORK_FIFO_LENGTH)
			SW_FUEL_GAUGE_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_WARNING_FIFO_FULL);

		SW_FUEL_GAUGE_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_ENQUEUE_FUNCTION,
								p_function);

		SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_ENQUEUE_PARAM,
									param);
		/* Schedule the work queue to process the message. */
		(void)queue_work(sw_fuel_gauge_work.p_work_queue,
						&sw_fuel_gauge_work.work);

		wake_lock(&sw_fuel_gauge_work.kfifo_wakelock);


		spin_unlock_irqrestore(&sw_fuel_gauge_work.lock, flags);
	}
}

/**
 * sw_fuel_gauge_execute_function -	Remove from queue and execute all
 *					scheduled functions in a work.
 * @p_function				[in] Function to be added to the
 *					execution FIFO.
 * @param				[in] Parameter value for the function.
 */
static void sw_fuel_gauge_execute_function(struct work_struct *work)
{
	struct sw_fuel_gauge_fifo_payload payload;
	unsigned long flags;

	/* Not used. */
	(void)work;


	/* Repeatedly fetch one entry from the fifo and process it until the
	fifo is empty. */
	for (;;) {

		mutex_lock(&sw_fuel_gauge_work.deferred_exec_lock);

		if (sw_fuel_gauge_work.pm_prepare) {
			sw_fuel_gauge_work.pending_dequeue = true;
			SW_FUEL_GAUGE_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_DEFER_FUNCTION);
			mutex_unlock(&sw_fuel_gauge_work.deferred_exec_lock);
			return;
		}

		if (0 == kfifo_out_spinlocked(&sw_fuel_gauge_work.fifo,
					&payload,
					1,
					&sw_fuel_gauge_work.lock)) {
			mutex_unlock(&sw_fuel_gauge_work.deferred_exec_lock);
			return;
		}

		SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_EXECUTE_FUNCTION,
								payload.p_func);

		SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_EXECUTE_PARAM,
								payload.param);
		/* Execute the function. */
		payload.p_func(payload.param);

		mutex_unlock(&sw_fuel_gauge_work.deferred_exec_lock);

		/* wakelock acquisition must be in sync with kfifo operation */
		spin_lock_irqsave(&sw_fuel_gauge_work.lock, flags);

		/* If work fifo is empty the wakelock can be released */
		if (kfifo_is_empty(&sw_fuel_gauge_work.fifo))
			wake_unlock(&sw_fuel_gauge_work.kfifo_wakelock);

		spin_unlock_irqrestore(&sw_fuel_gauge_work.lock, flags);
	}
}

/**
 * sw_fuel_gauge_vbat_convert_to_capacity -	Converts a battery voltage value
 *						into a capacity remaining
 *						estimate.
 * @vbat_mv					[in] Battery cell voltage.
 *						Unit mV.
 * Returns					The capacity remaining as a
 *						permil value.
 */
static int sw_fuel_gauge_vbat_convert_to_capacity_permil(int vbat_mv)
{
	const u32 *p_cap_to_vbat_table =
			&sw_fuel_gauge_instance.bat.
				p_fitted_model->cap_to_vbat_ocv[0];
	int ret_capacity;
	/* Start the search in the middle of the table. */
	int i = (BAT_CAP_TO_VBAT_TABLE_SIZE / 2);
	/* Initialise the search step to half of the remaining values. */
	int j = (BAT_CAP_TO_VBAT_TABLE_SIZE / 4);

	/* Search capacity value that corresponds to VBAT in the look-up table
	Binary search is used, it will always find the value after 5 loops. */
	do {
		if (p_cap_to_vbat_table[i] > vbat_mv)
			i -= j;
		else
			i += j;

		j >>= 1;
	} while (j > 0);

	/* Refine capacity value found with the binary search. The result is
	very close and it should not take many iterations. */
	do {
		j = i;
		/* If the value is more than half way to the next lowest table
		entry, step down. */
		if ((i > 0)
			&& (vbat_mv < ((p_cap_to_vbat_table[i] +
					p_cap_to_vbat_table[i-1]) >> 1))) {
			i--;
		}
		/* If the value is more than half way to the next highest
		table entry, step up. */
		if ((i < (BAT_CAP_TO_VBAT_TABLE_SIZE - 1))
			&& (vbat_mv > ((p_cap_to_vbat_table[i] +
					p_cap_to_vbat_table[i+1]) >> 1))) {
			i++;
		}
	} while (j != i);

	/* Determine the two capacity points to interpolate making sure
	always j >= i */
	if ((p_cap_to_vbat_table[i] < vbat_mv) &&
				((BAT_CAP_TO_VBAT_TABLE_SIZE - 1) > i)) {
		j = i + 1;
	} else if ((p_cap_to_vbat_table[i] > vbat_mv) && (0 < i)) {
		j = i;
		i = i - 1;
	} else {
		j = i;
	}

	if (i != j) {
		/* Calculate the linear-interpolated capacity of VBAT
		(in permil) This arithmetic needs a signed integer. */
		ret_capacity = (i * SCALE_PERMIL/SCALE_PERCENT
			+ (((int) (2 * (SCALE_PERMIL/SCALE_PERCENT)*(j - i) *
					(vbat_mv - p_cap_to_vbat_table[i])))
					/ ((int) ((p_cap_to_vbat_table[j] -
					p_cap_to_vbat_table[i]))) + 1)/2);
	} else {
		ret_capacity = i * SCALE_PERMIL/SCALE_PERCENT;
	}

	return ret_capacity;
}

/**
 * sw_fuel_gauge_set_health -	Atomically updates the health property and
 *				notifies the power supply class of a property
 *				change.
 * @health			[in] POWER_SUPPY_PROP_HEALTH enum values.
 */
static void sw_fuel_gauge_set_health(int health)
{
	struct power_supply_properties *p_properties =
					&sw_fuel_gauge_instance.properties;

	/* Check for a change. */
	if (p_properties->health != health) {
		/* Obtain lock on properties data to ensure atomic access. */
		down(&p_properties->lock);
		/* Update the property. */
		p_properties->health = health;
		SW_FUEL_GAUGE_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_SET_PROPERTY_HEALTH, health);
		/* End of critical section for update. */
		up(&p_properties->lock);
		/* Inform power supply class of property change. */
		power_supply_changed(&sw_fuel_gauge_instance.power_supply_bat);
	}
}

/**
 * sw_fuel_gauge_set_capacity -	Atomically updates the capacity property and
 *				notifies the power supply class of a property
 *				change.
 * @capacity_permil	[in]	Value of capacity property to set.
 */
static void sw_fuel_gauge_set_capacity(int capacity_permil)
{
	struct power_supply_properties *p_properties =
				&sw_fuel_gauge_instance.properties;

	struct sw_fuel_gauge_data *sw_fg = &sw_fuel_gauge_instance;

	/* Power Supply Class unit is Percent. Convert from Permil */
	int capacity_percent = capacity_permil * SCALE_PERCENT / SCALE_PERMIL;

	/* Check for a change or the first setting. */
	if ((p_properties->internal_capacity != capacity_percent) ||
					(!p_properties->capacity_valid)) {
		/* Obtain lock on properties data to ensure atomic access. */
		down(&p_properties->lock);
		/* Update the property and mark it as valid. */
		p_properties->internal_capacity = capacity_percent;


		if (p_properties->status == POWER_SUPPLY_STATUS_CHARGING ||
			p_properties->status == POWER_SUPPLY_STATUS_FULL ||
			!p_properties->capacity_valid) {

			/* In case of initial run or battery charging
			reported capacity will be the same as the internal
			one */
			p_properties->reported_capacity =
					p_properties->internal_capacity;
		} else {
			/* If the battery is discharging we only allow
			reported capacity to change into lower value */
			if (p_properties->reported_capacity >
					p_properties->internal_capacity)
				p_properties->reported_capacity =
					p_properties->internal_capacity;
		}

		p_properties->capacity_valid = true;

		/* Supply VBAT by table lookup of capacity then scale to uV for
		PSY */
		if (sw_fg->bat.fitted_state && sw_fg->bat.p_fitted_model)
			p_properties->voltage_ocv = sw_fg->bat.
			p_fitted_model->cap_to_vbat_ocv[capacity_percent] *
								SCALE_MILLI;
		/* Calculate charge from capacity, if charge-full-design has
		been set. */
		if (p_properties->battery_data_valid) {
			p_properties->charge_now = (capacity_percent *
					p_properties->charge_full_design)
					/ SCALE_PERCENT;
		}
		SW_FUEL_GAUGE_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_SET_PROPERTY_CAPACITY,
							capacity_percent);
		/* End of critical section for update. */
		up(&p_properties->lock);
		/* Inform power supply class of property change. */
		power_supply_changed(&sw_fuel_gauge_instance.power_supply_bat);
	}
}

/**
 * sw_fuel_gauge_set_bat_properties -	Atomically updates the battery related
 *					properties and notifies the power supply
 *					class of a change.
 * @charge_full_design			[in] value of CHARGE_FULL_DESIGN (mAh)
 * @technology				[in] value of TECHNOLOGY
 * @present				[in] true if battery is fitted,
 *					else false.
 */
static void sw_fuel_gauge_set_bat_properties(int charge_full_design,
						int technology, bool present)
{
	struct power_supply_properties *p_properties =
					&sw_fuel_gauge_instance.properties;

	/* Check for any change or first setting. */
	if ((!p_properties->battery_data_valid)
		|| (present != p_properties->present)) {

		/* Obtain lock on properties data to ensure atomic access. */
		down(&sw_fuel_gauge_instance.properties.lock);

		/* Update the properties and mark them as valid. */
		p_properties->battery_data_valid = true;
		p_properties->present = present;
		p_properties->technology = technology;
		/* Charge Full Design must be converted to uAh*/
		p_properties->charge_full_design = charge_full_design *
								SCALE_MILLI;

		SW_FUEL_GAUGE_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_SET_PROPERTY_PRESENCE,
								present);

		SW_FUEL_GAUGE_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_SET_PROPERTY_TECHNOLOGY,
								technology);

		SW_FUEL_GAUGE_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_SET_PROPERTY_CHARGE_FULL_DESIGN,
							charge_full_design);

		/* End of critical section for update. */
		up(&sw_fuel_gauge_instance.properties.lock);
		/* Inform power supply class of property change. */
		power_supply_changed(&sw_fuel_gauge_instance.power_supply_bat);
	}
}

/**
 * sw_fuel_gauge_set_temperature -	Atomically updates the health property
 *					and notifies the power supply class
 *					of a property change.
 * @temperature_degc			[in] Battery temperature in degrees.
 * @valid				[in] True if the temperature is valid
 */
static void sw_fuel_gauge_set_temperature(int temperature_degc, bool valid)
{
	struct power_supply_properties *p_properties =
					&sw_fuel_gauge_instance.properties;

	int *temperature_notified = &sw_fuel_gauge_instance.
					tbat.temperature_notif_ddegc;

	if (valid) {
		/* Power Supply Class unit is tenths of a degree C. */
		int temperature = SCALE_DECI_DEG_C * temperature_degc;
		/* Check for a change or first setting. */
		if ((p_properties->temperature != temperature) ||
					(!p_properties->temperature_valid)) {
			/* Obtain lock on properties data to ensure atomic
			access. */
			down(&p_properties->lock);
			/* Update the property and mark it as valid. */
			p_properties->temperature = temperature;
			p_properties->temperature_valid = true;
			SW_FUEL_GAUGE_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_SET_PROPERTY_TEMPERATURE,
								temperature);

			if (abs(*temperature_notified - temperature) >
							1 * SCALE_DECI_DEG_C) {

				*temperature_notified = temperature;

				/* End of critical section for update. */
				up(&p_properties->lock);

				/* Inform power supply class of
				property change. */
				power_supply_changed(
					&sw_fuel_gauge_instance.
							power_supply_bat);

				return;
			}

			/* End of critical section for update. */
			up(&p_properties->lock);
		}
	} else {
		/* Obtain lock on properties data to ensure atomic access. */
		down(&p_properties->lock);
		/* Invalid the PSY property */
		p_properties->temperature_valid = false;
		SW_FUEL_GAUGE_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_SET_PROPERTY_TEMPERATURE_INVALID);
		/* End of critical section for update. */
		up(&p_properties->lock);
	}
}

/**
 * sw_fuel_gauge_tbat_monitor - Make a battery temperature sensor reading
 * and check it against the allowed range for the battery.
 * This function is triggered every time the power supply class battery
 * temperature is read and every time the HAL updates the status of the coulomb
 * counter.
 */
static void sw_fuel_gauge_tbat_monitor(void)
{
	struct bat_temperature_data *p_tbat = &sw_fuel_gauge_instance.tbat;
	struct ps_pse_mod_prof *p_bat_model = NULL;

	/*
	 * Make battery temperature measurement.
	 * NOTE: Initialised to prevent compiler warning.
	 */
	int tbat_value = 0;
	int health = POWER_SUPPLY_HEALTH_UNKNOWN;
	int tbat_error;

	if (!sw_fuel_gauge_instance.bat.initialised)
		return;

	tbat_error = iio_read_channel_processed(p_tbat->p_iio_tbat,
						 &tbat_value);
	tbat_value = 25;
	tbat_error = IIO_VAL_INT;
	/* Check for saturation. This is allowed when the phone is powered
	by a PSU. */
	if ((-EOVERFLOW == tbat_error) && (BAT_ID_PSU ==
				sw_fuel_gauge_instance.bat.bat_id_type)) {

		/* Measurement is saturated and a PSU is being used. The
		temperature is invalid and is not stored. The power supply
		property POWER_SUPPLY_PROP_TEMP is now unavailable. */
		sw_fuel_gauge_set_temperature(0, false);
		SW_FUEL_GAUGE_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_TBAT_SATURATED_PSU);

		/* The health of a PSU is always good. */
		health = POWER_SUPPLY_HEALTH_GOOD;
	} else {
		switch (tbat_error) {
		case IIO_VAL_INT:
			p_bat_model = sw_fuel_gauge_instance.bat.p_fitted_model;
			/* Clear any error condition on successful
			measurement */
			p_tbat->bat_temperature_overflow_error_count = 0;
			p_tbat->bat_temperature_io_error_count = 0;
			/* Push the power supply property
			POWER_SUPPLY_PROP_TEMP */
			sw_fuel_gauge_set_temperature(tbat_value, true);
			/*
			 * Check that temperature is within specified range
			 * and set HEALTH property accordingly.
			 */
			if (NULL != p_bat_model) {
				if (tbat_value > p_bat_model->disch_tmp_ul) {
					health = POWER_SUPPLY_HEALTH_OVERHEAT;

					SW_FUEL_GAUGE_DEBUG_PARAM(
					 SW_FUEL_GAUGE_DEBUG_TBAT_MODEL_UL,
					  p_bat_model->disch_tmp_ul);

				} else if (tbat_value <
						p_bat_model->disch_tmp_ll) {
					health = POWER_SUPPLY_HEALTH_COLD;

					SW_FUEL_GAUGE_DEBUG_PARAM(
					 SW_FUEL_GAUGE_DEBUG_TBAT_MODEL_LL,
					  p_bat_model->disch_tmp_ll);

				} else {
					health = POWER_SUPPLY_HEALTH_GOOD;
				}
			}
			break;
		case -EIO:
			/* Too many TBAT sensor io errors lead to a bad health
			and charging will stop. */
			p_tbat->bat_temperature_io_error_count++;
			SW_FUEL_GAUGE_DEBUG_PARAM(
					SW_FUEL_GAUGE_DEBUG_TBAT_ERROR,
								tbat_error);
			if (p_tbat->bat_temperature_io_error_count >=
						TBAT_MAX_IO_ERROR_COUNT) {

				health = POWER_SUPPLY_HEALTH_UNKNOWN;
				sw_fuel_gauge_set_temperature(0, false);
			}
			break;
		case -EOVERFLOW:
			/* fall-through is intentional */
		default:
			/* Too many TBAT sensor overflow or other errors lead to
			a bad health and charging will stop. */
			p_tbat->bat_temperature_overflow_error_count++;
			SW_FUEL_GAUGE_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_TBAT_ERROR, tbat_error);

			if (p_tbat->bat_temperature_overflow_error_count >=
						TBAT_MAX_OVERFLOW_ERROR_COUNT) {

				health = POWER_SUPPLY_HEALTH_UNKNOWN;
				sw_fuel_gauge_set_temperature(0, false);
			}
			break;
		};
	}
	sw_fuel_gauge_set_health(health);
}

/**
 * sw_fuel_gauge_cc_read_current_data - Read the current values of the couloumb
 * counters and the monotonic rtc timestamp.
 * @p_cal_data		[out] Coulomb counter values and time stamp.
 */
static void sw_fuel_gauge_cc_read_current_data(struct soc_cal_point *p_cal_data)
{
	struct timespec time_now;
	union sw_fuel_gauge_hal_get_params hal_get;

	/* Store the timestamp now. */
	time_now = ktime_to_timespec(ktime_get_boottime());
	p_cal_data->rtc_time_sec = time_now.tv_sec;

	/* Get the current coulomb counter values, if available. */
	BUG_ON(sw_fuel_gauge_instance.p_hal_interface->get(
					SW_FUEL_GAUGE_HAL_GET_CC_BALANCED_COUNT,
								&hal_get));
	p_cal_data->cc_balanced_mc = hal_get.cc_balanced_mc;

	p_cal_data->cc_up_mc =
		(sw_fuel_gauge_instance.p_hal_interface->get(
			SW_FUEL_GAUGE_HAL_GET_CC_UP_COUNT, &hal_get))
						? NVM_NO_VALUE
						: hal_get.cc_up_mc;

	p_cal_data->cc_down_mc =
		(sw_fuel_gauge_instance.p_hal_interface->get(
			SW_FUEL_GAUGE_HAL_GET_CC_DOWN_COUNT, &hal_get))
						? NVM_NO_VALUE
						: hal_get.cc_down_mc;

	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_CC_BALANCED_MC,
						p_cal_data->cc_balanced_mc);
	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_CC_DOWN_MC,
						p_cal_data->cc_down_mc);
	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_CC_UP_MC,
						p_cal_data->cc_up_mc);
	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_CC_RTC_SEC,
						p_cal_data->rtc_time_sec);
}

/**
 * sw_fuel_gauge_cc_convert_to_bat_capacity_permil -	Converts a balanced
 *							coulomb counter value
 *							into a capacity
 *							remaining estimate.
 * @cc_balanced_mc					[in] Balanced coulomb
 *							count. Unit mC.
 * Returns						The capacity remaining
 *							as a permil value.
*/
static int sw_fuel_gauge_cc_convert_to_bat_capacity_permil(int cc_balanced_mc)
{
	int ret_capacity;
	/* The capacity must be clamped at 0 and 1000 permil. This arithmetic
	needs a signed integer. */
	int battery_capacity_permil =
		sw_fuel_gauge_instance.last_soc_cal.soc_permil
		 + scale_and_divide_with_rounding_s32(SCALE_PERMIL,
		  sw_fuel_gauge_instance.last_soc_cal.cc_balanced_mc -
		   cc_balanced_mc, sw_fuel_gauge_instance.nom_bat_capacity_mc);

	/* Trace the raw calculated battery capacity, as this could be outside
	the valid permil range. */
	SW_FUEL_GAUGE_DEBUG_PARAM(
		SW_FUEL_GAUGE_DEBUG_RAW_CC_BAT_CAP_PERMIL,
					battery_capacity_permil);

	/* Prevent capacity exceeding allowed range. */
	if (battery_capacity_permil > 1000)
		ret_capacity = 1000;
	else if (battery_capacity_permil < 0)
		ret_capacity = 0;
	else
		ret_capacity = battery_capacity_permil;


	return ret_capacity;
}

/**
 * sw_fuel_gauge_calc_battery_values	Calculates parameters relating to the
 *					battery model data. This can only be
 *					done after the battery model is known.
*/
static void sw_fuel_gauge_calc_battery_values(void)
{
	/* Read the nominal battery capacity from the battery model. */
	u32 nom_bat_capacity_mah =
			sw_fuel_gauge_instance.bat.p_fitted_model->capacity;
	/* Convert the battery capacity to mC and store for use in coulomb
	counter calculations. */
	sw_fuel_gauge_instance.nom_bat_capacity_mc = nom_bat_capacity_mah *
								SCALE_MAH_TO_MC;
	/* Calculate the current load levels at which the battery is deemed to
	be relaxed. These are defined as a proportion of the nominal battery
	capacity. */
	sw_fuel_gauge_instance.ibat_load_avg_relaxed_limit_ma =
				nom_bat_capacity_mah / SCALE_IBAT_AVG_RELAXED;

	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_IBAT_AVG_RELAXED_LIM,
			sw_fuel_gauge_instance.ibat_load_avg_relaxed_limit_ma);


	sw_fuel_gauge_instance.ibat_load_now_relaxed_limit_ma =
			nom_bat_capacity_mah / SCALE_IBAT_NOW_RELAXED;

	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_IBAT_NOW_RELAXED_LIM,
		sw_fuel_gauge_instance.ibat_load_now_relaxed_limit_ma);


	/* Set coulomb counter capacity delta reporting threshold to one half of
	the reported SoC resolution. This ensures the reported SoC cannot change
	by more than the reported resolution between updates. */
	sw_fuel_gauge_instance.hal_set.delta_threshold_mc =
				sw_fuel_gauge_instance.nom_bat_capacity_mc /
							(SCALE_PERCENT * 2);
}

/**
 * sw_fuel_gauge_cc_soc_update - Called when the coulomb counter callback
 * occurs to trigger an SoC update. Sends an event to the state machine.
 * @coulomb_count_mc		[in] Coulomb counter value in mC
 */
static void sw_fuel_gauge_cc_soc_update(int coulomb_count_mc)
{
	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_DELTA_THRESHOLD_REACHED,
							coulomb_count_mc);
	/* Store latest coulomb count. */
	sw_fuel_gauge_instance.cc_balanced_mc = coulomb_count_mc;

	/* Monitor the battery temperature  */
	sw_fuel_gauge_tbat_monitor();

	/* Tick the state machine with this event. */
	sw_fuel_gauge_stm_process_event(SW_FUEL_GAUGE_STM_EVENT_SOC_UPDATE);
}

/**
 * sw_fuel_gauge_cc_hal_callback -	SW Fuel Gauge HAL interface callback
 *					function.
 * @event				[in] Event which has occured.
 * @param				[in] Parameter value for the event.
 */
static void sw_fuel_gauge_cc_hal_callback(enum sw_fuel_gauge_hal_cb_event event,
					union sw_fuel_gauge_hal_cb_param param)
{
	SW_FUEL_GAUGE_DEBUG_NO_LOG_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_CB_EVENT,
							event);

	SW_FUEL_GAUGE_DEBUG_NO_LOG_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_CB_PARAM,
							*(int *)(&param));

	/* Determine which event occured */
	switch (event) {
	case SW_FUEL_GAUGE_HAL_CB_EVENT_SOC_UPDATE:
		/* Schedule a work thread to process the event. */
		SW_FUEL_GAUGE_ENQUEUE(sw_fuel_gauge_cc_soc_update,
						param.cc_delta_mc);
		break;

	default:
	/* Unrecognised events should not occur. */
		BUG();
		break;
	};
}

/**
 * sw_fuel_gauge_calc_ocv_capacity_and_error -	Converts the given VBAT
 *						(OCV or TYP) value into permil
 *						capacity and error values.
 *
 * @vbat_mv			[in]	Battery voltage (mV).
 * @p_ocv_bat_cap_permil	[out]	Calculated battery capacity remaining
 *					(permil).
 * @p_ocv_error_permil		[out]	Capacity calculation error margin
 *					(permil).
 */
static void sw_fuel_gauge_calc_ocv_capacity_and_error(int vbat_mv,
						int *p_ocv_bat_cap_permil,
						 int *p_ocv_error_permil)
{
	/* Determine impact on SoC calculation for +ve and -ve error. */
	int ocv_bat_cap_permil =
			sw_fuel_gauge_vbat_convert_to_capacity_permil(vbat_mv);
	int ocv_bat_cap_min_permil =
			sw_fuel_gauge_vbat_convert_to_capacity_permil(vbat_mv -
			 sw_fuel_gauge_instance.vbat.accuracy_mv);

	int ocv_bat_cap_max_permil =
			sw_fuel_gauge_vbat_convert_to_capacity_permil(vbat_mv +
			 sw_fuel_gauge_instance.vbat.accuracy_mv);

	/* Use the worst case error. */
	if ((ocv_bat_cap_permil - ocv_bat_cap_min_permil) >
				(ocv_bat_cap_max_permil - ocv_bat_cap_permil))

		*p_ocv_error_permil = ocv_bat_cap_permil -
						ocv_bat_cap_min_permil;
	else
		*p_ocv_error_permil = ocv_bat_cap_max_permil -
						ocv_bat_cap_permil;

	/* Return calculated capacity value to caller. */
	*p_ocv_bat_cap_permil = ocv_bat_cap_permil;
	SW_FUEL_GAUGE_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_OCV_BAT_CAP_PERMIL,
						ocv_bat_cap_permil);

	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_OCV_ERROR_PERMIL,
						*p_ocv_error_permil);
}

/**
 * sw_fuel_gauge_init_swfg - send an event to the state
 *					machine that informs clients that
 *					the SW Fuel Gauge is initialised
 */
static void sw_fuel_gauge_init_swfg(void)
{
	/* Initialise the coulomb counter accumulated error to the
	 * mC equivalent of the calculated error.
	 */
	sw_fuel_gauge_instance.cc_acc_error_mc =
			sw_fuel_gauge_instance.latest_calculated_error_permil
			 * (sw_fuel_gauge_instance.nom_bat_capacity_mc /
								SCALE_PERMIL);

	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_CALC_INITIAL_CAPACITY,
		sw_fuel_gauge_instance.latest_calculated_capacity_permil);

	/* Push new value to the power supply class. */
	sw_fuel_gauge_set_capacity(
		sw_fuel_gauge_instance.latest_calculated_capacity_permil);

	/* Initialisation is done, so advance the state machine. */
	sw_fuel_gauge_stm_process_event(
			SW_FUEL_GAUGE_STM_EVENT_INITIAL_SOC_REPORT_DONE);

	/* Once the cc delta threshold is set the cc indication also becomes
	active. The statemachine must have sent its initial soc report before
	such an event. */
	BUG_ON(0 != sw_fuel_gauge_instance.p_hal_interface->set(
		SW_FUEL_GAUGE_HAL_SET_COULOMB_IND_DELTA_THRESHOLD,
			sw_fuel_gauge_instance.hal_set));

	/* Debug and trace information. */
	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_CONFIGURED_HAL,
			sw_fuel_gauge_instance.hal_set.delta_threshold_mc);
}

/**
 * sw_fuel_gauge_estimate_initial_capacity_and_error - Estimate the initial
 *					battery capacity and error based on
 *					the initial VBAT_TYP measurement
 *					then send an event to the state
 *					machine that informs clients that
 *					the SW Fuel Gauge is initialised.
 *					To be used if no NVM calibration
 *					data is present.
 */
static void sw_fuel_gauge_estimate_initial_capacity_and_error(void)
{
	struct soc_cal_point cc_data_now;
	struct soc_cal_point *p_last_soc_cal =
					&sw_fuel_gauge_instance.last_soc_cal;

	SW_FUEL_GAUGE_DEBUG_NO_PARAM(
		SW_FUEL_GAUGE_DEBUG_NVM_INVALID_ESTIMATE_CAPACITY_AND_ERROR);

	/* Get timestamped coulomb counter values. */
	sw_fuel_gauge_cc_read_current_data(&cc_data_now);

	/* Calculate capacity based on the initial VBAT_TYP
	measurement. */
	sw_fuel_gauge_calc_ocv_capacity_and_error(
	 sw_fuel_gauge_instance.vbat.vbat_typ_mv,
	  &sw_fuel_gauge_instance.latest_calculated_capacity_permil,
	   &sw_fuel_gauge_instance.latest_calculated_error_permil);
	/* The initial VBAT_TYP measurement can potentially be very
	inaccurate, so an OCV calibration measurement will be forced at
	the first opportunity by setting a high initial accumulated
	error. */
	sw_fuel_gauge_instance.latest_calculated_error_permil =
					INITIAL_VBAT_TYP_ERROR_PERMIL;

	/* Update the latest calibration point data. */
	p_last_soc_cal->cc_up_mc = cc_data_now.cc_up_mc;
	p_last_soc_cal->cc_down_mc = cc_data_now.cc_down_mc;
	p_last_soc_cal->cc_balanced_mc = cc_data_now.cc_balanced_mc;
	p_last_soc_cal->rtc_time_sec = cc_data_now.rtc_time_sec;
	p_last_soc_cal->soc_permil =
		sw_fuel_gauge_instance.latest_calculated_capacity_permil;

	p_last_soc_cal->soc_error_permil =
			sw_fuel_gauge_instance.latest_calculated_error_permil;
	p_last_soc_cal->full_battery_cap_mah =
			sw_fuel_gauge_instance.bat.p_fitted_model->capacity;

	/* Update the static IMMEDIATE data to match the last SOC
	 * calibration point.
	 * Setting the IMMEDIATE index to the last SOC index must
	 * also be done to prevent the less accurate VBAT capacity
	 * estimate from being flushed to NVM immediately. */
	sw_fuel_gauge_instance.last_immediate_cal =
			sw_fuel_gauge_instance.last_soc_cal;
	sw_fuel_gauge_instance.nvm_immediate_last_cal_point_index =
			sw_fuel_gauge_instance.last_soc_cal_index;

}

/**
 * sw_fuel_gauge_hardcode_initial_capacity_and_error - Hardcode the initial
 *					battery capacity to 50% and error to
 *					2% then send an event to the
 *					state machine that informs clients that
 *					the SW Fuel Gauge is initialised.
 *					To be used if we can access NVM
 *					calibration data, the correct data will
 *					then be read from NVM later.
 */
static void sw_fuel_gauge_hardcode_initial_capacity_and_error(void)
{
	struct soc_cal_point cc_data_now;
	struct soc_cal_point *p_last_soc_cal =
					&sw_fuel_gauge_instance.last_soc_cal;

	SW_FUEL_GAUGE_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_NVM_HARDCODE_CAPACITY_AND_ERROR);

	/* Get timestamped coulomb counter values. */
	sw_fuel_gauge_cc_read_current_data(&cc_data_now);

	/* Set capacity high so that it can be overwritten when the
	 * actual capacity is known. */
	sw_fuel_gauge_instance.latest_calculated_capacity_permil =
					POWER_SUPPLY_CAPACITY_1000_PERMIL;

	sw_fuel_gauge_instance.latest_calculated_error_permil =
					INITIAL_VBAT_TYP_ERROR_PERMIL;

	/* Update the latest calibration point data. */
	p_last_soc_cal->cc_up_mc = cc_data_now.cc_up_mc;
	p_last_soc_cal->cc_down_mc = cc_data_now.cc_down_mc;
	p_last_soc_cal->cc_balanced_mc = cc_data_now.cc_balanced_mc;
	p_last_soc_cal->rtc_time_sec = cc_data_now.rtc_time_sec;
	p_last_soc_cal->soc_permil =
	 sw_fuel_gauge_instance.latest_calculated_capacity_permil;

	p_last_soc_cal->soc_error_permil =
		sw_fuel_gauge_instance.latest_calculated_error_permil;
	p_last_soc_cal->full_battery_cap_mah =
		sw_fuel_gauge_instance.bat.p_fitted_model->capacity;

}

/**
 * sw_fuel_gauge_calculate_nvm_capacity_and_error - Calculate the initial
 *					battery capacity and error then
 *					send an event to the state
 *					machine that informs clients that
 *					the SW Fuel Gauge is initialised.
 */
static void sw_fuel_gauge_calculate_nvm_capacity_and_error(void)
{
	struct soc_cal_point cc_data_now;
	struct soc_cal_point *p_last_soc_cal =
					&sw_fuel_gauge_instance.last_soc_cal;
	struct soc_cal_point *p_last_immediate_cal =
				&sw_fuel_gauge_instance.last_immediate_cal;

	/* Get timestamped coulomb counter values. */
	sw_fuel_gauge_cc_read_current_data(&cc_data_now);

	/* If the UP and DOWN counters are not supported, then NVM calibration
	data cannot be used. */
	if ((NVM_NO_VALUE != cc_data_now.cc_up_mc) && (NVM_NO_VALUE !=
						cc_data_now.cc_down_mc)) {
		/* Check whether the NVS contains a valid calibration point. */
		if (sw_fuel_gauge_nvs_retrieve_last_calibration_point(
					p_last_soc_cal,
					p_last_immediate_cal)) {
			/* Store the last calibration point for debug. */
			pr_debug("Last Soc calibration done at %ld\n",
				p_last_soc_cal->rtc_time_sec);

			pr_debug("CC UP mc: %d, CC DOWN mc: %d, CC bal mc: %d, Bat Soc: %d, Bat SoC err: %d, Bat Cap: %u\n",
					p_last_soc_cal->cc_up_mc,
					p_last_soc_cal->cc_down_mc,
					p_last_soc_cal->cc_balanced_mc,
					p_last_soc_cal->soc_permil,
					p_last_soc_cal->soc_error_permil,
					p_last_soc_cal->full_battery_cap_mah);

			/* Compare the stored timestamp with the current time to
			determine if the calibration data is too old. */
			if ((cc_data_now.rtc_time_sec -
				p_last_soc_cal->rtc_time_sec) <=
					MAX_CALIBRATION_POINT_AGE_SECS) {
				/* The timestamp is recent enough. Check that
				the battery was not removed since the
				calibration point was stored. If either the
				current UP or DOWN coulomb count is less than
				the NVM value, then a reset has occurred and the
				stored calibration data cannot be used for the
				initial SOC calculation. */
				if ((cc_data_now.cc_up_mc >=
						p_last_soc_cal->cc_up_mc)
					&& (cc_data_now.cc_down_mc >=
						p_last_soc_cal->cc_down_mc)) {
					sw_fuel_gauge_instance.
						nvm_valid_at_init
									= true;
				} else {
					/* Log failure reason. */
					SW_FUEL_GAUGE_DEBUG_NO_LOG_NO_PARAM(
				 SW_FUEL_GAUGE_DEBUG_NVM_BATTERY_REMOVED);
					pr_err("%s:Battery removed\n",
							__func__);
				}
			} else {
				/* Log failure reason. */
				SW_FUEL_GAUGE_DEBUG_PARAM(
					SW_FUEL_GAUGE_DEBUG_NVM_OUT_OF_DATE,
						p_last_soc_cal->rtc_time_sec);
			}
		} else {

			/* Log error as NVS doesn't have a valid calibratrion
			 * point. */
			pr_err("%s: Couldn't retrieve calibration point\n",
					__func__);
		}
	}
	/* If NVM calibration point is valid, calculate capacity and error based
	 *  on stored SoC calibration point. */
	if (sw_fuel_gauge_instance.nvm_valid_at_init) {
		/* We have retrieved a valid calibration point from NVM. */
		SW_FUEL_GAUGE_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_NVM_CAL_POINT_RESTORED);
		sw_fuel_gauge_instance.latest_calculated_capacity_permil =
			sw_fuel_gauge_cc_convert_to_bat_capacity_permil(
						cc_data_now.cc_balanced_mc);

		sw_fuel_gauge_instance.latest_calculated_error_permil =
					p_last_soc_cal->soc_error_permil;
	} else {
		/* Invalidate the Calibration point if present in the NVM.*/
		if (sw_fuel_guage_nvs_cal_point_invalidate() == false) {
			pr_err("%s:unable to invalidate cal. point in NVM\n"
					, __func__);
		}

		/* No valid data in NVM -> fall back to calculate capacity
		 *  based on the initial VBAT_TYP measurement. */
		sw_fuel_gauge_estimate_initial_capacity_and_error();
	}

	/* Finish the initialization and advance the state machine. */
	sw_fuel_gauge_init_swfg();

}

static void swfg_nvs_ready_work(int param)
{
	/* unused */
	(void)param;

	SW_FUEL_GAUGE_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_NVS_READY_CB);

	/* Double check that we only calculate the capacity from
	NVM in case that we are waiting for the initial SOC. */
	if (SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC ==
		sw_fuel_gauge_instance.stm.state)
			sw_fuel_gauge_calculate_nvm_capacity_and_error();
	else
		pr_err("%s() called in invalid state %d\n", __func__,
			sw_fuel_gauge_instance.stm.state);
}

/**
 * Called when the NVS is initialized.
 */
static void sw_fuel_gauge_nvs_ready_cb(void)
{
	/* Handle event in the serialized workqueue */
	SW_FUEL_GAUGE_ENQUEUE(swfg_nvs_ready_work, 0);
}

/*
 * sw_fuel_gauge_check_hal_initialisation	Check if both the Battery Driver
 *						Hal and the SW Fuel Gauge HAL
 *						have registered and that a
 *						battery is present. If true,
 *						then the coulomb counter can be
 *						configured and SW Fuel Gauge can
 *						be started.
 */
static void sw_fuel_gauge_check_hal_initialisation(void)
{
	if ((SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC ==
		sw_fuel_gauge_instance.stm.state)
			&& (NULL != sw_fuel_gauge_instance.p_hal_interface)
			 && (sw_fuel_gauge_instance.bat.initialised)
			  && (sw_fuel_gauge_instance.bat.fitted_state)) {

			/* Try and register with the NVS interface to
			 * get notified when NVS is ready to be accessed.
			 */
			if (sw_fuel_gauge_register_nvs_ready_cb(
					sw_fuel_gauge_nvs_ready_cb)) {
				/* Temporarily set capacity above 0% to
				 * prevent user space triggered shutdown
				 * before the NVS is ready.
				 */
			sw_fuel_gauge_hardcode_initial_capacity_and_error();
			} else {
				/* We don't have persistent storage
				 * -> estimate initial capacity. */
			sw_fuel_gauge_estimate_initial_capacity_and_error();

				/* Finish the initialization and advance
				 * the state machine. */
				sw_fuel_gauge_init_swfg();
			}

			/* Make initial TBAT measurements */
			sw_fuel_gauge_tbat_monitor();

		}
}

/**
 * sw_fuel_gauge_bat_string_to_enum	Decode battery type string to enum value
 *					for comparision
 * @p_bat_type_string		[in]	Pointer to battery name.
 */
static enum bat_id_type sw_fuel_gauge_bat_id_string_to_enum(
						char *p_bat_id_string)
{
	int i;
	BUG_ON(NULL == p_bat_id_string);

	/* Scan acceptable list of battery strings for a match. */
	for (i = 0; i < BAT_ID_MAX; i++) {
		if (0 == strcmp(p_bat_id_string, bat_id_strings[i]))
			return i;
	}

	/* Battery was not found, so report unknown. */
	return BAT_ID_UNKNOWN;
}

/**
 * sw_fuel_gauge_bat_presence_report	Deal with battery driver battery status
 *					update. The Fuel Gauge Monitoring should
 *					continue even if a battery is not fitted
 *					to give an indication that e.g. the
 *					level of a bench power supply is too
 *					low. Monitoring of a power supply can
 *					also be useful to provide a functional
 *					test of the capacity indication,
 *					although of course the capacity of a
 *					power supply has no actual meaning.
 *					The battery model passed to this
 *					function is needed for the calculation.
 *
 * @p_data			[in]	Pointer to battery status data.
 */
static void sw_fuel_gauge_bat_presence_report(
				struct ps_pse_mod_prof *p_reported_model)
{
	struct bat_driver_data *p_bat = &sw_fuel_gauge_instance.bat;

	/* Battery ID notifier uses NULL to represent no battery fitted. */
	p_bat->p_fitted_model = p_reported_model;

	/* Check if this is the first battery driver notification. */
	if (p_bat->initialised) {
		/* This callback is not the first from the battery driver. This
		can happen but requires no action apart from logging. It is
		possible in a test situation where the platform is powered by a
		power supply, or the platform is powered from the charger
		source */
		SW_FUEL_GAUGE_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_BAT_REPORT_REPEATED);
	}

	if (NULL == p_reported_model) {
		/* The platform is running, so a PSU must be present. Set
		default property values and remaining capacity to 100%. The
		state machine will not be started, so this value will remain. */
		p_bat->fitted_state = false;
		p_bat->bat_id_type = BAT_ID_PSU;
		sw_fuel_gauge_set_bat_properties(POWER_SUPPLY_CHARGE_MAX_MAH,
						POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
						false);
		sw_fuel_gauge_set_capacity(POWER_SUPPLY_CAPACITY_1000_PERMIL);
		SW_FUEL_GAUGE_DEBUG_NO_PARAM(
					SW_FUEL_GAUGE_DEBUG_BAT_NOT_FITTED);
		/* Declare the battery data initialised. */
		p_bat->initialised = true;
	} else {
		/* Battery fitted and data supplied. */
		p_bat->fitted_state = true;
		p_bat->bat_id_type = sw_fuel_gauge_bat_id_string_to_enum(
						&p_reported_model->batt_id[0]);
		SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_BAT_ID_TYPE,
							p_bat->bat_id_type);

		sw_fuel_gauge_calc_battery_values();

		/* Set battery properties in power supply class */
		sw_fuel_gauge_set_bat_properties(p_reported_model->capacity,
						p_reported_model->battery_type,
						true);
		/* Declare the battery data initialised. */
		p_bat->initialised = true;
		/*
		 * Check if initialisation is complete and the SW Fuel Gauge can
		 * proceed to be configured and started.
		 */
		sw_fuel_gauge_check_hal_initialisation();
	}
}

/**
 * sw_fuel_gauge_bat_id_notification	Process event from power supply
 *					notifier.
 * @notifier			[in]	Pointer to the notifier block.
 * @val				[in]	Notifier event
 * @p_data			[in]	Pointer to the event payload.
 */
static int sw_fuel_gauge_bat_id_notification(struct notifier_block *notifier,
						unsigned long val,
						void *p_data)
{

	switch (val) {
	case BATT_ID_EVENT_BAT_PRESENCE_EVENT:
	{
		struct ps_batt_chg_prof *batt_prop =
					(struct ps_batt_chg_prof *)p_data;
		if (NULL == batt_prop) {
			SW_FUEL_GAUGE_DEBUG_NO_LOG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_BAT_NOTIFICATION_REMOVED);
			/* Schedule a work thread to process the event. */
			SW_FUEL_GAUGE_ENQUEUE(sw_fuel_gauge_bat_presence_report,
									NULL);
		} else if (batt_prop->chrg_prof_type == PSE_MOD_CHRG_PROF) {
			/* Only react to known event types. */
			SW_FUEL_GAUGE_DEBUG_NO_LOG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_BAT_NOTIFICATION_INSERTED);
			/* Schedule a work thread to process the event. */
			SW_FUEL_GAUGE_ENQUEUE(sw_fuel_gauge_bat_presence_report,
							batt_prop->batt_prof);
		} else {
			SW_FUEL_GAUGE_DEBUG_NO_LOG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_BAT_NOTIFICATION_UNKNOWN);
		}
	}
	break;

	default:
	break;
	}
	return 0;
}

/**
 * sw_fuel_gauge_stm_check_nvm_immediate_write_required
 * If the accumulated coulomb counter error since the last immediate calibration
 * point has previously been found to be outside the acceptable error range,
 * flush the latest deferred calibration point to NVM.
 */
static void sw_fuel_gauge_stm_check_nvm_immediate_write_required(void)
{
	struct sw_fuel_gauge_data *sw_fg = &sw_fuel_gauge_instance;

	if (sw_fuel_gauge_instance.nvm_immediate_write_required) {
		/* If the last immediate write was the latest calibration point,
		then there is nothing to do. Repeated writes of the same data
		would prematurely wear the NVM. */
		if (sw_fg->nvm_immediate_last_cal_point_index !=
						sw_fg->last_soc_cal_index) {

			sw_fuel_gauge_nvs_store_last_calibration_point(
				SW_FUEL_GAUGE_NVS_WRITE_IMMEDIATE,
				&sw_fuel_gauge_instance.last_soc_cal,
				&sw_fuel_gauge_instance.last_immediate_cal);

			/* Reset the NVM write request flag and coulomb counter
			error accumulator. */
			sw_fg->nvm_immediate_write_required = false;
			sw_fg->nvm_immediate_write_acc_error_mc = 0;
			/* Note that the last OCV calibration point is now
			written to immediate NVM */
			sw_fg->nvm_immediate_last_cal_point_index =
				sw_fuel_gauge_instance.last_soc_cal_index;
		}
	}
}

/**
 * sw_fuel_gauge_stm_check_nvm_immediate_error_threshold_and_age:
 * Checks whether the accuracy of the coulomb counter has degraded to the point
 * where the latest OCV calibration point must be flushed to NVM.
*/
static void sw_fuel_gauge_stm_check_nvm_immediate_error_threshold_and_age(void)
{
	int total_nvm_error_permil;
	bool nvm_immediate_over_error_threshold;
	bool nvm_immediate_over_age_limit;
	time_t nvm_immediate_age_sec;
	struct timespec time_now;
	union sw_fuel_gauge_hal_get_params hal_get;

	/* Get rtc timestamp now and calculate the last immediate NVM
	data age. */
	time_now = ktime_to_timespec(ktime_get_boottime());
	nvm_immediate_age_sec = time_now.tv_sec -
			sw_fuel_gauge_instance.last_immediate_cal.rtc_time_sec;

	SW_FUEL_GAUGE_DEBUG_PARAM(
		SW_FUEL_GAUGE_DEBUG_IMMEDIATE_AGE_SEC, nvm_immediate_age_sec);

	/* Check the age against the maximum allowed. */
	nvm_immediate_over_age_limit =
			(nvm_immediate_age_sec > NVM_IMMEDIATE_MAX_AGE_SECS);

	/* Fetch the accumulated error of the coulomb counter from the HAL
	driver. */
	BUG_ON(0 != sw_fuel_gauge_instance.p_hal_interface->get(
				SW_FUEL_GAUGE_HAL_GET_CC_ACCUMULATED_ERROR,
								&hal_get));
	/* Check whether the error since the last immediate calibration point
	has reached the threshold where the new calibration point must be
	committed to NVM. */
	total_nvm_error_permil =
		sw_fuel_gauge_instance.last_immediate_cal.soc_error_permil
		 + scale_and_divide_with_rounding_s32(SCALE_PERMIL,
		  sw_fuel_gauge_instance.nvm_immediate_write_acc_error_mc
		   + hal_get.cc_acc_error_mc,
		    sw_fuel_gauge_instance.nom_bat_capacity_mc);

	/* If the threshold has been crossed, note that an immediate NVM write
	is to be made. */
	nvm_immediate_over_error_threshold =
		(total_nvm_error_permil >= NVM_IMMEDIATE_ERR_THRESHOLD_PERMIL);

	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_IMMEDIATE_ERROR_PERMIL,
							total_nvm_error_permil);

	sw_fuel_gauge_instance.nvm_immediate_write_required =
				(nvm_immediate_over_error_threshold ||
						nvm_immediate_over_age_limit);

}

/*
 * sw_fuel_gauge_create_ocv_calibration_point - Create a calibration point with
 * associated error margins. Used for EOC and OCV calibrations.
 * @ocv_bat_cap_permil	[in]	SoC
 * @ocv_error_permil	[in]	Estimated SoC error.
 * @cc_acc_error_mc	[in]	Calculated coulomb counter error in mC
 */
static void sw_fuel_gauge_create_ocv_calibration_point(int ocv_bat_cap_permil,
							int ocv_error_permil,
							int cc_acc_error_mc)
{
	/* This parameter is not used, but still required. */
	union sw_fuel_gauge_hal_set_params dummy = {
		.dummy_value = 0,
	};
	/* Update the battery capacity and error from the OCV measurement. */
	sw_fuel_gauge_instance.latest_calculated_capacity_permil =
							ocv_bat_cap_permil;
	sw_fuel_gauge_instance.latest_calculated_error_permil =
							ocv_error_permil;

	/* Reset the coulomb counter accumulated error. */
	BUG_ON(0 != sw_fuel_gauge_instance.p_hal_interface->set(
				SW_FUEL_GAUGE_HAL_SET_ZERO_ACCUMULATED_CC_ERROR,
									dummy));

	/* When the error in the coulomb counter driver is reset, its value must
	be preserved by adding it to the accumulated error since the last
	immediate calibration point. */
	sw_fuel_gauge_instance.nvm_immediate_write_acc_error_mc +=
								cc_acc_error_mc;

	/* Set coulomb counter accumulated error to equivalent from OCV
	calibration. */
	sw_fuel_gauge_instance.cc_acc_error_mc = (ocv_error_permil
				* sw_fuel_gauge_instance.nom_bat_capacity_mc)
					/ SCALE_PERMIL;

	/* Get timestamp and current coulomb counter values. */
	sw_fuel_gauge_cc_read_current_data(
					&sw_fuel_gauge_instance.last_soc_cal);

	/* Set the corresponding the battery capacity and error. */
	sw_fuel_gauge_instance.last_soc_cal.soc_permil = ocv_bat_cap_permil;
	sw_fuel_gauge_instance.last_soc_cal.soc_error_permil = ocv_error_permil;

	/* Increase the number of calibration points which have been taken. */
	sw_fuel_gauge_instance.last_soc_cal_index++;


	/* Store the calibration point in the deferred write NVM group. */
	sw_fuel_gauge_nvs_store_last_calibration_point(
			SW_FUEL_GAUGE_NVS_WRITE_DEFERRED,
			&sw_fuel_gauge_instance.last_soc_cal,
			&sw_fuel_gauge_instance.last_immediate_cal);

	/* Push new capacity to power supply class. */
	sw_fuel_gauge_set_capacity(ocv_bat_cap_permil);
}

/**
 * sw_fuel_gauge_stm_check_ocv_calibration_point:
 * Checks whether an OCV measurement will provide more accuracy than the coulomb
 * counter and creates a calibration point if necessary.
 */
static void sw_fuel_gauge_stm_check_ocv_calibration_point(void)
{
	int ocv_error_permil;
	int ocv_bat_cap_permil;
	union sw_fuel_gauge_hal_get_params hal_get;
	int cc_error_permil;

	/* Calculate the OCV capacity and accuracy of the OCV measurement. */
	sw_fuel_gauge_calc_ocv_capacity_and_error(
					sw_fuel_gauge_instance.vbat.vbat_ocv_mv,
					 &ocv_bat_cap_permil,
					  &ocv_error_permil);

	/* Fetch the accumulated error of the coulomb counter from the
	driver. */
	BUG_ON(0 != sw_fuel_gauge_instance.p_hal_interface->get(
				SW_FUEL_GAUGE_HAL_GET_CC_ACCUMULATED_ERROR,
								&hal_get));

	/* Add the latest value to the accumulated error so far and convert into
	a permil value. */
	cc_error_permil = scale_and_divide_with_rounding_s32(SCALE_PERMIL,
				sw_fuel_gauge_instance.cc_acc_error_mc +
				 hal_get.cc_acc_error_mc,
				  sw_fuel_gauge_instance.nom_bat_capacity_mc);

	/* Debug and trace information. */
	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_CC_ERROR_PERMIL,
							cc_error_permil);

	/* If the OCV error is smaller than the coulomb counter error, create a
	calibration point. */
	if (ocv_error_permil < cc_error_permil) {
		/* Debug and trace information. */
		SW_FUEL_GAUGE_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_OCV_BASED_SOC_CALIBRATION,
				 sw_fuel_gauge_instance.vbat.vbat_ocv_mv);
		/* Create calibration point */
		sw_fuel_gauge_create_ocv_calibration_point(
						ocv_bat_cap_permil,
						 ocv_error_permil,
						  hal_get.cc_acc_error_mc);
	} else {
		SW_FUEL_GAUGE_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_CAL_POINT_LESS_ACCURATE);
	}
}

/**
 * sw_fuel_gauge_stm_check_battery_relaxed:
 * Check whether battery is relaxed enough to make an OCV measurement, by
 * testing the long term average battery load current.
 */
static void sw_fuel_gauge_stm_check_battery_relaxed(void)
{
	struct timespec time_now;
	union sw_fuel_gauge_hal_get_params hal_get;

	/*Check whether the last immediate calibration point is out of date or
	too inaccurate to use at the next boot time. NOTE: This is only a test
	for an out of range calibration point.*/
	sw_fuel_gauge_stm_check_nvm_immediate_error_threshold_and_age();

	/* Use this opportunity to check the battery temperature, too. */
	sw_fuel_gauge_tbat_monitor();

	time_now = ktime_to_timespec(ktime_get_boottime());
	/* Read the long term battery average current from the HAL driver. */
	if (sw_fuel_gauge_instance.p_hal_interface->get(
		SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE_AT_OCV,
								&hal_get)) {
		long delta_sec;

		SW_FUEL_GAUGE_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_IBAT_LONG_NOT_READY);

		/* The long term average takes several minutes to become
		available after boot. If after MAX_IBAT_LONG_TERM_FAILURE_TIME
		time it is still not available we trap. */
		delta_sec = time_now.tv_sec -
				sw_fuel_gauge_instance.hal_reg_timestamp.tv_sec;

		BUG_ON(delta_sec > MAX_IBAT_LONG_TERM_FAILURE_TIME);
	} else {

		/* Compare the current with the allowed maximum for a relaxed
		battery. */
		if ((hal_get.ibat_load_long_at_ocv_ma <=
			sw_fuel_gauge_instance.ibat_load_avg_relaxed_limit_ma)
			&& (hal_get.ibat_load_long_at_ocv_ma >= 0)) {
			/* Battery is relaxed. */
			SW_FUEL_GAUGE_ENQUEUE(
				sw_fuel_gauge_stm_process_event,
				 SW_FUEL_GAUGE_STM_EVENT_BATTERY_RELAXED);

			SW_FUEL_GAUGE_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_BATTERY_RELAXED,
				 hal_get.ibat_load_long_at_ocv_ma);
		} else {
			/* This parameter is not used, but still required. */
			union sw_fuel_gauge_hal_set_params dummy = {
				.dummy_value = 0,
			};

			/* Battery is not relaxed. */
			SW_FUEL_GAUGE_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_BATTERY_NOT_RELAXED,
				 hal_get.ibat_load_long_at_ocv_ma);

			/* If the current immediate calibration point was found
			to be outside the acceptable error range, flush any
			previous deferred calibration point to NVM. */
			sw_fuel_gauge_stm_check_nvm_immediate_write_required();

			/* Clear latched values at OCV to start looking for
			another OCV condition if supported in HW */
			sw_fuel_gauge_instance.p_hal_interface->set(
		 SW_FUEL_GAUGE_HAL_SET_VBAT_MAX_CLEAR, dummy);
		}
	}
}

/**
 * sw_fuel_gauge_stm_enter_ocv_received
 *	Causes the monitoring state machine to enter the OCV_RECEIVED state.
 */
static void sw_fuel_gauge_stm_enter_ocv_received(void)
{
	/* This parameter is not used, but still required. */
	union sw_fuel_gauge_hal_set_params dummy = {
		.dummy_value = 0,
	};

	/* Assume that battery is nolonger relaxed. */
	enum sw_fuel_gauge_stm_event event =
			SW_FUEL_GAUGE_STM_EVENT_BATTERY_NOLONGER_RELAXED;

	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_STM_STATE_OCV_RECEIVED,
				sw_fuel_gauge_instance.vbat.vbat_ocv_mv);

	/* Check state machine integrity. */
	switch (sw_fuel_gauge_instance.stm.state) {
	case SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV:
		break;

	/* These state transitions are invalid.	Fall through to panic. */
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE:
	default:
		BUG();
		break;
	}
	/* Enter the requested state. */
	sw_fuel_gauge_instance.stm.state = SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED;

	if (IIO_VAL_INT == sw_fuel_gauge_instance.vbat.error_code) {
		/* OCV measurement succeeded. Check that battery remained
		relaxed by reading the short term average load current and
		ensuring that it remained low during the OCV measurement. */
		union sw_fuel_gauge_hal_get_params hal_get;

		BUG_ON(0 != sw_fuel_gauge_instance.p_hal_interface->get(
		 SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE_AT_OCV,
		  &hal_get));

		/* Compare the current with the allowed maximum for a relaxed
		battery. */
		if ((hal_get.ibat_load_short_ma <=
		sw_fuel_gauge_instance.ibat_load_now_relaxed_limit_ma) &&
					(hal_get.ibat_load_short_ma >= 0)) {

			/* Even if calibration point eventually fails, OCV DONE
			event will be scheduled. */
			event = SW_FUEL_GAUGE_STM_EVENT_OCV_DONE;
			/* Battery remained relaxed, so OCV calibration point
			may be possible. */
			SW_FUEL_GAUGE_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_BATTERY_RELAXED_STILL,
						hal_get.ibat_load_short_ma);
			sw_fuel_gauge_stm_check_ocv_calibration_point();
		} else {
			SW_FUEL_GAUGE_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_BATTERY_NOLONGER_RELAXED,
						hal_get.ibat_load_short_ma);
		}
	} else if (-EAGAIN == sw_fuel_gauge_instance.vbat.error_code) {
		/*
		 * OCV measurement was not possible at this time.
		 * Nothing to do here. Battery became not relaxed.
		 */

		/* increment -EAGAIN errors counter */
		sw_fuel_gauge_debug.stats.again_error_cnt++;

		SW_FUEL_GAUGE_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_OCV_EAGAIN);
	} else if (-EIO == sw_fuel_gauge_instance.vbat.error_code) {
		/*
		 * OCV measurement was not possible at this time.
		 * Nothing to do here. Battery became not relaxed.
		 */

		/* increment -EIO errors counter */
		sw_fuel_gauge_debug.stats.io_error_cnt++;

		SW_FUEL_GAUGE_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_OCV_EIO);
	} else {
		/* No other error is allowed. */

		pr_err("%s() vbat.error_code = %d\n", __func__,
				sw_fuel_gauge_instance.vbat.error_code);
		BUG();
	}

	/* Clear latched values at OCV to start looking for another OCV
	condition if supported in HW */
	sw_fuel_gauge_instance.p_hal_interface->set(
		SW_FUEL_GAUGE_HAL_SET_VBAT_MAX_CLEAR, dummy);

	/* Process the next event depending on the measurement outcome. */
	SW_FUEL_GAUGE_ENQUEUE(sw_fuel_gauge_stm_process_event, event);
}

/**
 * sw_fuel_gauge_stm_enter_wait_for_ocv
 *	Causes the monitoring state machine to enter the WAIT_FOR_OCV state.
 */
static void sw_fuel_gauge_stm_enter_wait_for_ocv(void)
{
	struct bat_voltage_data *p_vbat = &sw_fuel_gauge_instance.vbat;

	SW_FUEL_GAUGE_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_STM_STATE_WAIT_FOR_OCV);

	/* Check state machine integrity. */
	switch (sw_fuel_gauge_instance.stm.state) {
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED:
		break;

	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC:
	case SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE:
	/* These state transitions are invalid. Fall through to panic. */
	default:
		BUG();
		break;
	}
	/* Enter the requested state. */
	sw_fuel_gauge_instance.stm.state = SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV;
	/* Make an OCV measurement. */
	p_vbat->error_code = iio_read_channel_processed(p_vbat->p_iio_vbat_ocv,
							&p_vbat->vbat_ocv_mv);

	/* Send the OCV done event */
	SW_FUEL_GAUGE_ENQUEUE(sw_fuel_gauge_stm_process_event,
					SW_FUEL_GAUGE_STM_EVENT_OCV_MEAS_DONE);
}

/**
 * sw_fuel_gauge_stm_enter_wait_for_battery_relaxed
 *	Causes the monitoring state machine to enter the
 *	WAIT_FOR_BATTERY_RELAXED state.
 */
static void sw_fuel_gauge_stm_enter_wait_for_battery_relaxed(
					enum sw_fuel_gauge_stm_event event)
{
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_STM_STATE_WAIT_FOR_BATTERY_RELAXED);

	/* Check state machine integrity. */
	switch (sw_fuel_gauge_instance.stm.state) {
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC:
	case SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE:
		break;

	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV:
	/* This state transition is invalid. Fall through to panic. */
	default:
		BUG();
		break;
	}

	switch (event) {
	case SW_FUEL_GAUGE_STM_EVENT_SOC_UPDATE:
	case SW_FUEL_GAUGE_STM_EVENT_SYSTEM:
	case SW_FUEL_GAUGE_STM_EVENT_INITIAL_SOC_REPORT_DONE:
		/* Check if battery is relaxed */
		sw_fuel_gauge_stm_check_battery_relaxed();
		break;

	case SW_FUEL_GAUGE_STM_EVENT_BATTERY_NOLONGER_RELAXED:
		/* If the current immediate calibration point was found to be
		 * outside the acceptable error range, flush any previous
		 * deferred calibration point to NVM. */
		sw_fuel_gauge_stm_check_nvm_immediate_write_required();
		break;

	case SW_FUEL_GAUGE_STM_EVENT_BATTERY_RELAXED:
	case SW_FUEL_GAUGE_STM_EVENT_OCV_MEAS_DONE:
	case SW_FUEL_GAUGE_STM_EVENT_OCV_DONE:
	case SW_FUEL_GAUGE_STM_EVENT_EOC:
	/* Invalid events for entering this state */
	default:
		BUG();
		break;
	}


	/* Enter the requested state. */
	sw_fuel_gauge_instance.stm.state =
			SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED;
}

/**
 * sw_fuel_gauge_stm_enter_wait_for_soc_update
 *	Causes the state machine to enter the WAIT_FOR_SOC_UPDATE state.
 */
static void sw_fuel_gauge_stm_enter_wait_for_soc_update(void)
{
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_STM_STATE_WAIT_FOR_SOC_UPDATE);

	/* Check state machine integrity. */
	switch (sw_fuel_gauge_instance.stm.state) {
	case SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE:
		break;

	/* These state transitions are invalid.	Fall through to panic. */
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC:
	default:
		BUG();
		break;
	}
	/* Enter the requested state. */
	sw_fuel_gauge_instance.stm.state =
				SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE;
	/*
	Check if the current immediate calibration point was found to be outside
	the acceptable error range. If so, flush latest calibration point
	to NVM. */
	sw_fuel_gauge_stm_check_nvm_immediate_write_required();
}

/**
 * sw_fuel_gauge_stm_enter_wait_for_initial_soc
 *	SW Fuel Gauge state machine initialisation.
 */
static void sw_fuel_gauge_stm_enter_wait_for_initial_soc(void)
{
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_STM_STATE_WAIT_FOR_INITIAL_SOC);

	/* Make initial TBAT measurements */
	sw_fuel_gauge_tbat_monitor();

	/* Make initial VBAT TYP measurement */
	BUG_ON(IIO_VAL_INT != iio_read_channel_processed(
				sw_fuel_gauge_instance.vbat.p_iio_vbat_ocv,
				 &sw_fuel_gauge_instance.vbat.vbat_typ_mv));
	/* Determine the accuracy of VBAT OCV */
	BUG_ON(IIO_VAL_INT !=
		iio_channel_read(
				sw_fuel_gauge_instance.vbat.p_iio_vbat_ocv,
				&sw_fuel_gauge_instance.vbat.accuracy_mv,
				NULL,
				IIO_CHAN_INFO_TOLERANCE));

	/* Set the starting state for the state machine. All further action is
	event driven. In order to progress, the battery presence and model must
	be notified and the Fuel Gauge HAL must register. */
	sw_fuel_gauge_instance.stm.state =
			SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC;
}

/**
 * sw_fuel_gauge_stm_process_event_soc_update
 *	Process the SoC update event in the state machine.
 */
static void sw_fuel_gauge_stm_process_event_soc_update(void)
{
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_STM_EVENT_SOC_UPDATE);

	/* Check state machine integrity. */
	switch (sw_fuel_gauge_instance.stm.state) {
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED:
		sw_fuel_gauge_stm_enter_wait_for_battery_relaxed(
					SW_FUEL_GAUGE_STM_EVENT_SOC_UPDATE);
	/* Fall through is intentional. */
	case SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV:
		/* Compute SoC and report new capacity to clients. */
		sw_fuel_gauge_instance.latest_calculated_capacity_permil =
				sw_fuel_gauge_cc_convert_to_bat_capacity_permil(
					sw_fuel_gauge_instance.cc_balanced_mc);

		/* Debug and trace information for latest capacity */
		SW_FUEL_GAUGE_DEBUG_PARAM(
		 SW_FUEL_GAUGE_DEBUG_CALC_CAPACITY,
		  sw_fuel_gauge_instance.latest_calculated_capacity_permil);

		/* Push new value to the power supply class. */
		sw_fuel_gauge_set_capacity(
		 sw_fuel_gauge_instance.latest_calculated_capacity_permil);
		break;

	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC:
	case SW_FUEL_GAUGE_STM_STATE_UNITIALISED:
	/* This state transition is invalid. Fall through to TRAP. */
	default:
		BUG();
		break;
	}
}

/**
 * sw_fuel_gauge_stm_process_event_battery_nolonger_relaxed
 * Process the Battery Nolonger Relaxed event in the state machine.
 */
static void sw_fuel_gauge_stm_process_event_battery_nolonger_relaxed(void)
{
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_STM_EVENT_BATTERY_NOLONGER_RELAXED);

	/* Check state machine integrity. */
	switch (sw_fuel_gauge_instance.stm.state) {
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED:
		/* battery still relaxed check done before transition
		to this state. Ignore it */
		break;

	/* Fall through is intentional. */
	case SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED:
		sw_fuel_gauge_stm_enter_wait_for_battery_relaxed(
			SW_FUEL_GAUGE_STM_EVENT_BATTERY_NOLONGER_RELAXED);
		break;

	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC:
	case SW_FUEL_GAUGE_STM_STATE_UNITIALISED:
	/* This state transition is invalid. Fall through to TRAP. */
	default:
		BUG();
		break;
	}
}

/**
 * sw_fuel_gauge_stm_process_event_battery_relaxed
 *	Process the Battery Relaxed event in the state machine.
 */
static void sw_fuel_gauge_stm_process_event_battery_relaxed(void)
{
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_STM_EVENT_BATTERY_RELAXED);

	switch (sw_fuel_gauge_instance.stm.state) {
	case SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV:
		/* battery relaxed check done before transition to this state.
		Ignore it */
		break;

	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED:
		sw_fuel_gauge_stm_enter_wait_for_ocv();
		break;

	case SW_FUEL_GAUGE_STM_STATE_UNITIALISED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC:
	/* This state transition is invalid. Fall through to TRAP. */
	default:
		BUG();
		break;
	}
}

/**
 * sw_fuel_gauge_stm_process_event_ocv_meas_done
 *	Process the OCV Done event in the state machine.
 */
static void sw_fuel_gauge_stm_process_event_ocv_meas_done(void)
{
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_STM_EVENT_OCV_MEAS_DONE);

	/* Check state machine integrity. */
	switch (sw_fuel_gauge_instance.stm.state) {
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED:
	case SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED:
		/* OCV meas invoked before transition to this state.
		Ignore it */
		break;

	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV:
		sw_fuel_gauge_stm_enter_ocv_received();
		break;

	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC:
	case SW_FUEL_GAUGE_STM_STATE_UNITIALISED:
	/* This state transition is invalid. Fall through to TRAP. */
	default:
		BUG();
		break;
	}
}

/**
 * sw_fuel_gauge_stm_process_event_ocv_done
 *	Process the OCV Done event in the state machine.
 */
static void sw_fuel_gauge_stm_process_event_ocv_done(void)
{
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_STM_EVENT_OCV_DONE);

	/* Check state machine integrity. */
	switch (sw_fuel_gauge_instance.stm.state) {
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV:
		/* battery still relaxed check done before transition
		to this state. Ignore it */
		break;

	case SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED:
		sw_fuel_gauge_stm_enter_wait_for_soc_update();
		break;

	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC:
	case SW_FUEL_GAUGE_STM_STATE_UNITIALISED:
	/* This state transition is invalid. Fall through to TRAP. */
	default:
		BUG();
		break;
	}
}



/**
 * sw_fuel_gauge_stm_process_event_eoc
 *	Process the End of Charge event in the state machine.
 */
static void sw_fuel_gauge_stm_process_event_eoc(void)
{
	const u32 *p_cap_to_vcell_table;

	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_STM_EVENT_EOC,
				sw_fuel_gauge_instance.charger_target_mv);


	/* Check state machine integrity. */
	switch (sw_fuel_gauge_instance.stm.state) {
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE:
	case SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED:
	/* Fall through is intentional. */
		p_cap_to_vcell_table =
			&sw_fuel_gauge_instance.bat.
				p_fitted_model->cap_to_vbat_ocv[0];

		/* EOC calibration is only valid when the target Voltage is 100%
		of SoC or more. */
		if (sw_fuel_gauge_instance.charger_target_mv >
						p_cap_to_vcell_table[100]) {

			union sw_fuel_gauge_hal_get_params hal_get;

			/* Get the current coulomb counter values, if
			available. */
			BUG_ON(sw_fuel_gauge_instance.p_hal_interface->get(
				SW_FUEL_GAUGE_HAL_GET_CC_ACCUMULATED_ERROR,
								&hal_get));

			/* Check whether the last immediate calibration point is
			too old or inaccurate to use at boot time.
			NOTE: This is only a test for an out of range
			calibration point. The write to NVM, if required will
			occur on the next state transition. */
		sw_fuel_gauge_stm_check_nvm_immediate_error_threshold_and_age();

			/* Create the 100% calibration point with an error
			marigin of 0% */
			sw_fuel_gauge_create_ocv_calibration_point(1000, 0,
						hal_get.cc_acc_error_mc);
			sw_fuel_gauge_stm_enter_wait_for_soc_update();
		} else {
			SW_FUEL_GAUGE_DEBUG_NO_PARAM(
			 SW_FUEL_GAUGE_DEBUG_WARNING_EOC_NOT_FULL_CHARGE);
		}
		break;

	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC:
	/*
	 * Until the battery driver and SW Fuel gauge HAL have reported
	 * there is nothing that can be done here except to note the event.
	 */
		SW_FUEL_GAUGE_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_WARNING_EOC_TOO_EARLY);
		break;

	case SW_FUEL_GAUGE_STM_STATE_UNITIALISED:
	/* This state transition is invalid. Fall through to TRAP. */
	default:
		BUG();
		break;
	}
}

/**
 * sw_fuel_gauge_stm_process_event_system
 *	Process a system event in the state machine.
 */
static void sw_fuel_gauge_stm_process_event_system(void)
{
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_STM_EVENT_SYSTEM);

	/* Check state machine integrity. */
	switch (sw_fuel_gauge_instance.stm.state) {
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_BATTERY_RELAXED:
		sw_fuel_gauge_stm_enter_wait_for_battery_relaxed(
					SW_FUEL_GAUGE_STM_EVENT_SYSTEM);
		break;
	case SW_FUEL_GAUGE_STM_STATE_OCV_RECEIVED:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_OCV:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_SOC_UPDATE:
	case SW_FUEL_GAUGE_STM_STATE_WAIT_FOR_INITIAL_SOC:
	case SW_FUEL_GAUGE_STM_STATE_UNITIALISED:
		/* Nothing to do */
		break;
	/* This state transition is invalid. Fall through to TRAP. */
	default:
		BUG();
		break;
	}

}

/**
 * sw_fuel_gauge_stm_process_event	Processes an event input to the
 *					monitoring state machine by calling the
 *					appropriate handler.
 * @event				[in] STM event to process.
 */
void sw_fuel_gauge_stm_process_event(enum sw_fuel_gauge_stm_event event)
{
	switch (event) {
	case SW_FUEL_GAUGE_STM_EVENT_INITIAL_SOC_REPORT_DONE:
		sw_fuel_gauge_stm_enter_wait_for_battery_relaxed(
			SW_FUEL_GAUGE_STM_EVENT_INITIAL_SOC_REPORT_DONE);
		break;
	case SW_FUEL_GAUGE_STM_EVENT_BATTERY_NOLONGER_RELAXED:
		sw_fuel_gauge_stm_process_event_battery_nolonger_relaxed();
		break;
	case SW_FUEL_GAUGE_STM_EVENT_BATTERY_RELAXED:
		sw_fuel_gauge_stm_process_event_battery_relaxed();
		break;
	case SW_FUEL_GAUGE_STM_EVENT_SOC_UPDATE:
		sw_fuel_gauge_stm_process_event_soc_update();
		break;
	case SW_FUEL_GAUGE_STM_EVENT_OCV_MEAS_DONE:
		sw_fuel_gauge_stm_process_event_ocv_meas_done();
		break;
	case SW_FUEL_GAUGE_STM_EVENT_OCV_DONE:
		sw_fuel_gauge_stm_process_event_ocv_done();
		break;
	case SW_FUEL_GAUGE_STM_EVENT_EOC:
		sw_fuel_gauge_stm_process_event_eoc();
		break;
	case SW_FUEL_GAUGE_STM_EVENT_SYSTEM:
		sw_fuel_gauge_stm_process_event_system();
		break;
	default:
		/* The event passed was not valid. */
		BUG();
		break;
	}
}


/**
 * sw_fuel_gauge_set_property - registered with power supply class to write
 *				driver properties values
 * @p_power_supply      [in]    Pointer to power supply structure.
 * @property            [in]    Property to be write.
 * @p_value             [out]   Value of property.
 */
static int sw_fuel_gauge_set_property(
			struct power_supply *p_power_supply,
			 enum power_supply_property property,
			  const union power_supply_propval *p_value)
{
	struct power_supply_properties *p_properties =
				&sw_fuel_gauge_instance.properties;
	int error = 0;

	/* Make sure that the correct power supply is specified. */
	BUG_ON(p_power_supply != &sw_fuel_gauge_instance.power_supply_bat);
	/* Make sure that the return value pointer is valid. */
	BUG_ON(NULL == p_value);

	/* Obtain lock on properties data to ensure atomic access. */
	down(&p_properties->lock);

	SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_SET_PROPERTY, property);

	/* Return the cached value of the requested property, if known. */
	switch (property) {
	case POWER_SUPPLY_PROP_STATUS:
		/* Update the battery charging status. */
		p_properties->status = p_value->intval;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_FULL:
		SW_FUEL_GAUGE_ENQUEUE(sw_fuel_gauge_eoc_handler,
						p_value->intval);
		break;


	/* Requested property is not supported. */
	default:
		error = -EINVAL;
		break;
	}

	if (error) {
		SW_FUEL_GAUGE_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_SET_PROPERTY_ERROR, error);
	} else {
		SW_FUEL_GAUGE_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_SET_PROPERTY_VALUE,
							p_value->intval);
	}

	/* End of critical section. */
	up(&p_properties->lock);

	return error;
}

/**
 * update_property_and_log -	function updating a property if @data_valid is
 *				true and logging succes or failure of update
 * @plock		[in]	Pointer to semaphore synchronizing
 *				access to properties.
 * @property		[in]	enum value of property to be updated.
 * @p_dest		[in]	pointer to internal variable holding
 *				the property state.
 * @src			[in]	new property value
 * @data_valid		[in]	decides if given property should be successfully
 *				updated or not
 * Returns 0 if property successfully updated, -ENODATA otherwise
 */
static int update_property_and_log(struct semaphore *plock,
			enum power_supply_property property, int *p_dest,
						int src, bool data_valid)
{
	int ret;

	BUG_ON(NULL == p_dest);

	down(plock);

	if (data_valid) {
		*p_dest = src;
		ret = 0;
	} else {
		ret = -ENODATA;

		SW_FUEL_GAUGE_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_GET_PROPERTY,
								property);
		SW_FUEL_GAUGE_DEBUG_PARAM
			(SW_FUEL_GAUGE_DEBUG_GET_PROPERTY_ERROR, ret);

	}

	up(plock);
	return ret;
}


/**
 * sw_fuel_gauge_get_property - registered with power supply class to read
 *				driver exported values
 * @p_power_supply	[in]	Pointer to power supply structure.
 * @property		[in]	Property to be read.
 * @p_value		[out]	Value of property.
 */
static int sw_fuel_gauge_get_property(struct power_supply *p_power_supply,
					enum power_supply_property property,
					union power_supply_propval *p_value)
{
	struct power_supply_properties *p_properties =
					&sw_fuel_gauge_instance.properties;
	int error = 0;

	/* Make sure that the correct power supply is specified. */
	BUG_ON(p_power_supply != &sw_fuel_gauge_instance.power_supply_bat);
	/* Make sure that the return value pointer is valid. */
	BUG_ON(NULL == p_value);

	/* Return the cached value of the requested property, if known. */
	switch (property) {
	case POWER_SUPPLY_PROP_STATUS:
		/* No validity check required. UNKNOWN value is part of
		the enum. */
		error = update_property_and_log(&p_properties->lock, property,
				&p_value->intval, p_properties->status, true);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		/* No validity check required. UNKNOWN value is part of
		the enum. */
		error = update_property_and_log(&p_properties->lock, property,
				&p_value->intval, p_properties->health, true);
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		/* No validity check required. UNKNOWN value is part of
		the enum. */
		error = update_property_and_log(&p_properties->lock, property,
				&p_value->intval, p_properties->technology,
									true);
		break;

	/* Properties availble when battery presence is known.*/
	case POWER_SUPPLY_PROP_PRESENT:

		error = update_property_and_log(&p_properties->lock, property,
				&p_value->intval, p_properties->present,
					p_properties->battery_data_valid);

		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:

		error = update_property_and_log(&p_properties->lock,
						property, &p_value->intval,
					p_properties->charge_full_design,
					p_properties->battery_data_valid);

		break;

	case POWER_SUPPLY_PROP_TEMP:
		/* Update the temperature property */
		sw_fuel_gauge_tbat_monitor();

		error = update_property_and_log(&p_properties->lock, property,
				&p_value->intval, p_properties->temperature,
					p_properties->temperature_valid);

		break;


	/* Properties availble when battery capacity is known.*/
	case POWER_SUPPLY_PROP_CAPACITY:

		error = update_property_and_log(&p_properties->lock, property,
				&p_value->intval,
				p_properties->reported_capacity,
				p_properties->capacity_valid);

		break;

	case POWER_SUPPLY_PROP_VOLTAGE_OCV:

		if (sw_fuel_gauge_instance.bat.fitted_state) {
			error = update_property_and_log(&p_properties->lock,
					property, &p_value->intval,
					p_properties->voltage_ocv,
					p_properties->capacity_valid);
		} else	{

			int iio_vbat_mv;

			error = iio_read_channel_processed(
				sw_fuel_gauge_instance.vbat.p_iio_vbat_typ,
								&iio_vbat_mv);

			error = update_property_and_log(&p_properties->lock,
				property, &p_value->intval,
				iio_vbat_mv * SCALE_MILLI, (0 <= error));
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:

		error = update_property_and_log(&p_properties->lock, property,
				&p_value->intval, p_properties->charge_now,
					p_properties->capacity_valid);

		break;

	/* Measured xxx_NOW properties. */
	case POWER_SUPPLY_PROP_CURRENT_NOW:

		down(&p_properties->lock);

		error = -ENODATA;
		/* If the HAL has already registered, CURRENT_AVG can be
		read. */
		if (NULL != sw_fuel_gauge_instance.p_hal_interface) {
			union sw_fuel_gauge_hal_get_params hal_get;
			if (0 == sw_fuel_gauge_instance.p_hal_interface->get(
			 SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE,
								&hal_get)) {
				/* The Android convention for current requires
				positive values when charging. Since the driver
				uses negative current for charging internally
				the sign must be changed for export to PSC */
				p_value->intval = (-1) *
					hal_get.ibat_load_short_ma *
								SCALE_MILLI;
				error = 0;
			}
		}

		if (error) {

			SW_FUEL_GAUGE_DEBUG_PARAM(
					SW_FUEL_GAUGE_DEBUG_GET_PROPERTY,
								property);
			SW_FUEL_GAUGE_DEBUG_PARAM
				(SW_FUEL_GAUGE_DEBUG_GET_PROPERTY_ERROR, error);
		}


		up(&p_properties->lock);
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:

		down(&p_properties->lock);

		error = -ENODATA;
		if (NULL != sw_fuel_gauge_instance.p_hal_interface) {
			union sw_fuel_gauge_hal_get_params hal_get;
			if (0 == sw_fuel_gauge_instance.p_hal_interface->get(
			 SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE,
								&hal_get)) {

				p_value->intval = -(hal_get.ibat_load_long_ma *
								SCALE_MILLI);
				error = 0;
			}
		}

		if (error) {
			SW_FUEL_GAUGE_DEBUG_PARAM(
					SW_FUEL_GAUGE_DEBUG_GET_PROPERTY,
								property);
			SW_FUEL_GAUGE_DEBUG_PARAM
				(SW_FUEL_GAUGE_DEBUG_GET_PROPERTY_ERROR, error);
		}

		up(&p_properties->lock);
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:

		down(&p_properties->lock);

		error = -ENODATA;
		if (NULL != sw_fuel_gauge_instance.p_hal_interface) {
			union sw_fuel_gauge_hal_get_params hal_get;
			if (0 == sw_fuel_gauge_instance.p_hal_interface->get(
		SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE_AT_OCV,
								&hal_get)) {
				p_value->intval =
				 -(hal_get.ibat_load_long_at_ocv_ma *
								SCALE_MILLI);
				error = 0;
			}
		}

		if (error) {
			SW_FUEL_GAUGE_DEBUG_PARAM(
					SW_FUEL_GAUGE_DEBUG_GET_PROPERTY,
								property);
			SW_FUEL_GAUGE_DEBUG_PARAM
				(SW_FUEL_GAUGE_DEBUG_GET_PROPERTY_ERROR, error);
		}

		up(&p_properties->lock);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW: {

		int iio_vbat_mv;

		/* IIO reading can fail due to suspend mode.
		In this case property is not updated. */
		error = iio_read_channel_processed(
				sw_fuel_gauge_instance.vbat.p_iio_vbat_typ,
								&iio_vbat_mv);

		error = update_property_and_log(&p_properties->lock, property,
				&p_value->intval, iio_vbat_mv * SCALE_MILLI,
							(0 <= error));
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_COUNTER:

		down(&p_properties->lock);

		error = -ENODATA;
		/* If the HAL has already registered, the coulomb counter can
		be read. */
		if (NULL != sw_fuel_gauge_instance.p_hal_interface) {
			union sw_fuel_gauge_hal_get_params hal_get;
			if (0 == sw_fuel_gauge_instance.p_hal_interface->get(
					SW_FUEL_GAUGE_HAL_GET_CC_BALANCED_COUNT,
								&hal_get)) {

				/* Convert mC to uAh for power supply class and
				invert its sign to match the sign of
				CURRENT_NOW */
				p_value->intval = (-1) * (
					hal_get.cc_balanced_mc * SCALE_MILLI) /
								SCALE_MAH_TO_MC;
				error = 0;
			}
		}

		if (error) {
			SW_FUEL_GAUGE_DEBUG_PARAM(
					SW_FUEL_GAUGE_DEBUG_GET_PROPERTY,
								property);
			SW_FUEL_GAUGE_DEBUG_PARAM
				(SW_FUEL_GAUGE_DEBUG_GET_PROPERTY_ERROR, error);
		}

		up(&p_properties->lock);


		break;

	/* Invariant Properties. */
	case POWER_SUPPLY_PROP_TYPE:
		p_value->intval = POWER_SUPPLY_TYPE_BATTERY;
		break;

	/* String properties. NULL pointer indicates unknown. */
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		if (NULL != p_properties->serial_number)
			p_value->strval = p_properties->serial_number;
		else
			error = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		if (NULL != p_properties->manufacturer)
			p_value->strval = p_properties->manufacturer;
		else
			error = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		if (NULL != p_properties->model_name)
			p_value->strval = p_properties->model_name;
		else
			error = -ENODATA;
		break;

	/* Requested property is not supported. */
	default:
		error = -EINVAL;
		break;
	}

	return error;
}

/**
 * Refer to documentation in <sw_fuel_gauge_hal.h>
 */
int sw_fuel_gauge_register_hal(
	struct sw_fuel_gauge_hal_interface *p_sw_fuel_gauge_hal_interface,
		struct sw_fuel_gauge_interface **pp_sw_fuel_gauge_interface)
{
	/* Panic if registration has already occurred once. */
	BUG_ON(NULL != sw_fuel_gauge_instance.p_hal_interface);

	/* All required registration function pointers are required. */
	BUG_ON((NULL == p_sw_fuel_gauge_hal_interface)
		|| (NULL == p_sw_fuel_gauge_hal_interface->get)
		|| (NULL == p_sw_fuel_gauge_hal_interface->set)
		|| (NULL == pp_sw_fuel_gauge_interface));

	/* Registered interface is valid. Store it and provide the callback
	functions to the HAL. */
	sw_fuel_gauge_instance.p_hal_interface = p_sw_fuel_gauge_hal_interface;
	*pp_sw_fuel_gauge_interface = &sw_fuel_gauge_exported_functions;
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_REGISTER_HAL);

	sw_fuel_gauge_instance.hal_reg_timestamp =
					ktime_to_timespec(ktime_get_boottime());
	/* Finish the HAL registration in the work thread. */
	SW_FUEL_GAUGE_ENQUEUE(sw_fuel_gauge_check_hal_initialisation, 0);

	/* There are no recoverable errors for this function. Failure will
	cause a panic. */
	return 0;
}



/**
 * sw_fuel_gauge_probe - Initialises the driver OS resources when the device
 * has been found, then starts the state machine in a single threaded work.
 */
static int __init sw_fuel_gauge_probe(struct platform_device *p_platform_dev)
{
	struct device *p_dev = &p_platform_dev->dev;

	/* Initialise spinlock. */
	spin_lock_init(&sw_fuel_gauge_work.lock);
	mutex_init(&sw_fuel_gauge_work.deferred_exec_lock);

	wake_lock_init(&sw_fuel_gauge_work.kfifo_wakelock,
			WAKE_LOCK_SUSPEND,
			"swfg_kfifo_wakelock");

	/* Initialise properties semaphore unlocked */
	sema_init(&sw_fuel_gauge_instance.properties.lock, 1);

	/* Trace and Debug data. */
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_PROBE);

	/* Get required sensor channel for battery voltage and temperature. */
	BUG_ON(IS_ERR(sw_fuel_gauge_instance.tbat.p_iio_tbat =
			iio_channel_get(NULL, "BATTEMP0_SENSOR")));
	BUG_ON(IS_ERR(sw_fuel_gauge_instance.vbat.p_iio_vbat_typ =
			iio_channel_get(NULL, "VBAT_SENSOR")));
	BUG_ON(IS_ERR(sw_fuel_gauge_instance.vbat.p_iio_vbat_ocv =
			iio_channel_get(NULL, "VBAT_OCV_SENSOR")));

	/* Initialise work queue. Must be single thread to ensure
	serialisation. */
	INIT_WORK(&sw_fuel_gauge_work.work, sw_fuel_gauge_execute_function);
	/*
	 * Create private, single-threaded workqueue instead of using one of
	 * the system predefined workqueues to reduce latency
	 */
	sw_fuel_gauge_work.p_work_queue =
			create_singlethread_workqueue(dev_name(p_dev));

	BUG_ON(NULL == sw_fuel_gauge_work.p_work_queue);
	INIT_KFIFO(sw_fuel_gauge_work.fifo);

	/* Register the battery with the power supply class. */
	BUG_ON(power_supply_register(
			p_dev, &sw_fuel_gauge_instance.power_supply_bat));

	/* Register for events from the battery ID notifier. */
	BUG_ON(batt_id_reg_notifier(&sw_fuel_gauge_instance.notifier));

	/* Start the state machine in work thread. */
	sw_fuel_gauge_stm_enter_wait_for_initial_soc();

	/* There are no recoverable errors for this function. Failure will
	cause a panic. */
	return 0;
}

/**
 * sw_fuel_gauge_remove - Deregister with the power supply class and release
 * allocated resources.
 */
static int __exit sw_fuel_gauge_remove(struct platform_device *p_platform_dev)
{
	/* Trace and Debug data. */
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_REMOVE);

	/* Deregister for events from the battery driver notifier. */
	batt_id_unreg_notifier(&sw_fuel_gauge_instance.notifier);

	/* Prevent any further work functions being scheduled.*/
	sw_fuel_gauge_work.removal_pending = true;
	/* The return value is true if the work was pending, which is
	unimportant here. */
	(void)cancel_work_sync(&sw_fuel_gauge_work.work);
	destroy_workqueue(sw_fuel_gauge_work.p_work_queue);

	/* Deregister the battery with the power supply class. */
	power_supply_unregister(&sw_fuel_gauge_instance.power_supply_bat);

	return 0;
}

/**
 * sw_fuel_gauge_suspend() - Called when the system is attempting to suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int sw_fuel_gauge_suspend(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Nothing to do here except logging */
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_SUSPEND);
	return 0;
}

/**
 * sw_fuel_gauge_resume() - Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int sw_fuel_gauge_resume(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	SW_FUEL_GAUGE_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_RESUME);

	/* A system resume can be used to check on status */
	SW_FUEL_GAUGE_ENQUEUE(sw_fuel_gauge_stm_process_event,
					SW_FUEL_GAUGE_STM_EVENT_SYSTEM);

	return 0;
}

/**
 * sw_fuel_gauge_prepare() - Called when the system is preparing for suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
int sw_fuel_gauge_prepare(struct device *dev)
{
	mutex_lock(&sw_fuel_gauge_work.deferred_exec_lock);

	sw_fuel_gauge_work.pm_prepare = true;

	mutex_unlock(&sw_fuel_gauge_work.deferred_exec_lock);

	SW_FUEL_GAUGE_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_PREPARE_SUSPEND);

	return 0;
}

/**
 * sw_fuel_gauge_complete() - Called when the system is completing the resume.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
void sw_fuel_gauge_complete(struct device *dev)
{
	mutex_lock(&sw_fuel_gauge_work.deferred_exec_lock);

	sw_fuel_gauge_work.pm_prepare = false;

	if (sw_fuel_gauge_work.pending_dequeue) {
		sw_fuel_gauge_work.pending_dequeue = false;
		mutex_unlock(&sw_fuel_gauge_work.deferred_exec_lock);

		SW_FUEL_GAUGE_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_SCHEDULE_DEFERRED_FUNCTION);

		/* Schedule the work queue to process the message. */
		(void)queue_work(sw_fuel_gauge_work.p_work_queue,
						&sw_fuel_gauge_work.work);
		return;
	}

	mutex_unlock(&sw_fuel_gauge_work.deferred_exec_lock);

	return;
}


const struct dev_pm_ops sw_fuel_gauge_pm = {
	.suspend = sw_fuel_gauge_suspend,
	.resume = sw_fuel_gauge_resume,
	.prepare = sw_fuel_gauge_prepare,
	.complete = sw_fuel_gauge_complete,
};

static const struct of_device_id sw_fuel_gauge_of_match[] = {
	{
		.compatible = "intel,fuel_gauge",
	},
	{}
};

MODULE_DEVICE_TABLE(of, sw_fuel_gauge_of_match);

static struct platform_driver sw_fuel_gauge_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(sw_fuel_gauge_of_match),
		.pm = &sw_fuel_gauge_pm,
	},
	.probe	= sw_fuel_gauge_probe,
	.remove	= sw_fuel_gauge_remove,
};

static int __init sw_fuel_gauge_init(void)
{
	/*
	 * Initialise spinlock.
	 * NOTE: Must be done before debug data logging.
	 */
	spin_lock_init(&sw_fuel_gauge_debug.dbg_array.lock);
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_INIT);
	return platform_driver_register(&sw_fuel_gauge_driver);
}

static void __exit sw_fuel_gauge_exit(void)
{
	SW_FUEL_GAUGE_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_EXIT);
	return platform_driver_unregister(&sw_fuel_gauge_driver);
}

device_initcall_sync(sw_fuel_gauge_init);
module_exit(sw_fuel_gauge_exit);

MODULE_DESCRIPTION("SW Fuel Gauge Driver");
MODULE_LICENSE("GPL");

