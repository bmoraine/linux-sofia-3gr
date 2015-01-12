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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#ifndef _SW_FUEL_GAUGE_HAL_H
#define _SW_FUEL_GAUGE_HAL_H

/**
 * enum sw_fuel_gauge_hal_get_key - Key used to get information from the
 * SW Fuel Gauge HAL driver.
 * See struct sw_fuel_gauge_hal_interface::get()
.*
 * @SW_FUEL_GAUGE_HAL_GET_CC_UP_COUNT
.*	Key for the amount of charge into the battery. Parameters for ::get are
 *	then defined as:
.*	::union sw_fuel_gauge_hal_get_params.cc_up_mc
.*		Amount of charge into the battery (mC)
.*
.*.@SW_FUEL_GAUGE_HAL_GET_CC_DOWN_COUNT
.*	Key for the amount of charge out of the battery. Parameters for ::get
 *	are then defined as:
.*	::union sw_fuel_gauge_hal_get_params.cc_down_mc
.*		Amount of charge out of the battery (mC)
.*
.*.@SW_FUEL_GAUGE_HAL_GET_CC_BALANCED_COUNT
.*	Key for the amount of charge in/out of the battery since the last
 *	reference point.
.*	Parameters for ::get are then defined as:
.*	::union sw_fuel_gauge_hal_get_params.cc_balanced_mc
.*		Amount of charge into, less the charge out of the battery (mC)
.*
.*.@SW_FUEL_GAUGE_HAL_GET_CC_ACCUMULATED_ERROR
.*	Key for the accumulated error since the last error reset. Parameters
 *	for ::get are then defined as:
.*	::union sw_fuel_gauge_hal_get_params.cc_acc_error_mc
.*		Accumulated error since the last error reset (mC)
.*
.*.@SW_FUEL_GAUGE_HAL_GET_COULOMB_IND_DELTA_THRESHOLD,
.*	Key for definition of the coulomb indication delta threshold. Parameters
 *	for ::get are then defined as:
.*	::union sw_fuel_gauge_hal_get_params.delta_threshold_mc
.*		Threshold set for delta notification (mC)
.*
.*.@SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE,
.*	Key to read the short term average battery current. Parameters
 *	for ::get are then defined as:
.*	::union sw_fuel_gauge_hal_get_params.ibat_load_short_ma
.*		Short term average battery current (mA).
.*
.*.@SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE_AT_OCV,
.*	Key to read the short term average battery current at OCV measurement.
 *	Parameters for ::get are then defined as:
.*	::union sw_fuel_gauge_hal_get_params.ibat_load_short_at_ocv_ma
.*		Short term average battery current at OCV (mA).
.*
.*.@SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE_AT_OCV
.*	Key to read the long term average battery current at OCV measurement.
 *	Parameters for ::get are then defined as:
.*	::union sw_fuel_gauge_hal_get_params.ibat_load_long_at_ocv_ma
.*		Long term average battery current (mA)
 */
enum sw_fuel_gauge_hal_get_key {
	SW_FUEL_GAUGE_HAL_GET_CC_UP_COUNT,
	SW_FUEL_GAUGE_HAL_GET_CC_DOWN_COUNT,
	SW_FUEL_GAUGE_HAL_GET_CC_BALANCED_COUNT,
	SW_FUEL_GAUGE_HAL_GET_CC_ACCUMULATED_ERROR,
	SW_FUEL_GAUGE_HAL_GET_COULOMB_IND_DELTA_THRESHOLD,
	SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE,
	SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE_AT_OCV,
	SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE,
	SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE_AT_OCV
};

/**
 * union sw_fuel_gauge_hal_get_params - Union type for get function parameters.
 *
 * @cc_up_mc			See enum sw_fuel_gauge_hal_get_key::
 *					SW_FUEL_GAUGE_HAL_GET_CC_UP_COUNT
 * @cc_down_mc			See enum sw_fuel_gauge_hal_get_key::
 *					SW_FUEL_GAUGE_HAL_GET_CC_DOWN_COUNT
 * @cc_balanced_mc		See enum sw_fuel_gauge_hal_get_key::
 *					SW_FUEL_GAUGE_HAL_GET_CC_BALANCED_COUNT
 * @cc_acc_error_mc		See enum sw_fuel_gauge_hal_get_key::
 *					SW_FUEL_GAUGE_HAL_GET_CC_ACC_ERROR_COUNT
 * @delta_threshold_mc		See enum sw_fuel_gauge_hal_get_key::
 *			SW_FUEL_GAUGE_HAL_GET_COULOMB_IND_DELTA_THRESHOLD
 *
 * @ibat_load_short_ma		See enum sw_fuel_gauge_hal_get_key::
 *			SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE
 *
 * @ibat_load_short_at_ocv_ma	See enum sw_fuel_gauge_hal_get_key::
 *		SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE_AT_OCV
 *
 * @ibat_load_long_ma		See enum sw_fuel_gauge_hal_get_key::
 *			SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE
 *
 * @ibat_load_long_at_ocv_ma	See enum sw_fuel_gauge_hal_get_key::
			SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE_AT_OCV
 */
union sw_fuel_gauge_hal_get_params {
	int cc_up_mc;
	int cc_down_mc;
	int cc_balanced_mc;
	int cc_acc_error_mc;
	int delta_threshold_mc;
	int ibat_load_short_ma;
	int ibat_load_short_at_ocv_ma;
	int ibat_load_long_ma;
	int ibat_load_long_at_ocv_ma;
};

/**
 * key used to set the configuration of the SW Fuel Gauge HAL driver.
 * See struct sw_fuel_gauge_hal_interface::set()
 *
 * @SW_FUEL_GAUGE_HAL_SET_COULOMB_IND_DELTA_THRESHOLD
 *	Key for definition of the coulomb indication delta threshold. The
 *	parameters for ::set are then defined as:
 *	::union sw_fuel_gauge_hal_set_params.delta_threshold_mc -Delta threshold
 *	for notification (mC)
 *
 * @SW_FUEL_GAUGE_HAL_SET_ZERO_ACCUMULATED_CC_ERROR
 *	Key to reset the accumulated coulomb counter error. The parameters
 *	for ::set are then defined as: None.
 *	Performs error reset without further parameters.
 *
 * @SW_FUEL_GAUGE_HAL_SET_VBAT_MAX_CLEAR
 *	Key to reset the max vbat stored and associated ibat averages latched.
 *	This might be dummy if not supported in HW. The parameters for ::set
 *	are then defined as: None.
 *	Performs error reset without further parameters.
 */
enum sw_fuel_gauge_hal_set_key {
	SW_FUEL_GAUGE_HAL_SET_COULOMB_IND_DELTA_THRESHOLD,
	SW_FUEL_GAUGE_HAL_SET_ZERO_ACCUMULATED_CC_ERROR,
	SW_FUEL_GAUGE_HAL_SET_VBAT_MAX_CLEAR,
};

/**
 * union sw_fuel_gauge_hal_set_params - Union type for set function parameters.
 * @delta_threshold_mc			see enum sw_fuel_gauge_hal_set_key::
 *					SW_FUEL_GAUGE_HAL_SET_DELTA_THRESHOLD
 *
 * @dummy_value				For functions that do not require an
 *					argument, a dummy parameter is passed.
 */
union sw_fuel_gauge_hal_set_params {
	int delta_threshold_mc;
	int dummy_value;
};

/**
 * struct sw_fuel_gauge_hal_interface - SW Fuel Gauge HAL exported interface
 *					structure. All functions will be
 *					executed in SW Fuel Gauge work queue.
 *
 * @set			Set parameters of the SW Fuel Gauge HAL driver
 *			@key		[in] Key to specify parameter(s) to set.
 *			@p_params	[in] Structure holding list of
 *					parameters.
 *			Returns:	Error number; may be tested as boolean
 *					with 0=success, other=fail.
 *					Some errors may need re-trial.
 *
 * @get			Get parameters and measured values from the
 *					SW Fuel Gauge HAL driver.
 *
 *			@key		[in] Key to specify parameter(s) to get.
 *			@p_params	[out] Structure holding list of
 *					parameters.
 *			Returns:	Error number; may be tested as boolean
 *					with 0=success, other=fail.
 *					Some errors may need re-trial.
 */
struct sw_fuel_gauge_hal_interface {
	int (*set)(enum sw_fuel_gauge_hal_set_key key,
				union sw_fuel_gauge_hal_set_params param);

	int (*get)(enum sw_fuel_gauge_hal_get_key key,
				union sw_fuel_gauge_hal_get_params *p_param);
};

/**
 * union sw_fuel_gauge_hal_cb_param - Parameters passed by callback events.
 * @cc_delta_mc		Coulomb count when the SoC update was made (mC)
 */
union sw_fuel_gauge_hal_cb_param {
	int cc_delta_mc;
};

/**
 *.enum sw_fuel_gauge_hal_cb_event - Possible callback events.
 *
 * @SW_FUEL_GAUGE_HAL_CB_EVENT_SOC_UPDATE - Callback event for delta
 *	threshold update, which will not be called before
 *	SW_FUEL_GAUGE_HAL_SET_COULOMB_IND_DELTA_THRESHOLD has been set.
 *	Callback will occur when or before the previously set coulomb delta
 *	threshold has been exceeded. It may occur earlier than the previously
 *	set threshold if:
 *	1. The threshold has not been met after 5 minutes
 *	2. The underlying hardware limit for coulomb counter interrupt threshold
 *	setting is reached.
 */
enum sw_fuel_gauge_hal_cb_event {
	SW_FUEL_GAUGE_HAL_CB_EVENT_SOC_UPDATE
};

/* Function type for scheduled execution by work. */
typedef void (*fp_scheduled_function)(long param);

/**
 * struct sw_fuel_gauge_interface -  SW Fuel Gauge interface function type.
 *
 * @event_cb		Callback function to report events to the SW Fuel Gauge
 * @event		[in] Event which has occured.
 * @param		[in] Parameter value for the event.
 * @enqueue		Schedule a function to be executed in the SW Fuel Gauge
 *			work queue
 * @p_function		[in] Function to be executed
 * @param		[in] Parameter for the executed function
 */
struct sw_fuel_gauge_interface {
	void (*event_cb)(enum sw_fuel_gauge_hal_cb_event event,
					union sw_fuel_gauge_hal_cb_param param);

	void (*enqueue)(fp_scheduled_function p_function, long param);
};

/**
 * sw_fuel_gauge_register_hal() - Register the supported operations and
 *				capabilities of the HAL with the SW Fuel Gauge.
 *
 * @p_sw_fuel_gauge_hal_interface	[in] Exported functions and HW specific
 *					information.
 * @p_sw_fuel_gauge_interface		[out] Callback functions in the
 *					SW Fuel Gauge Driver.
 *
 * Only one registration allowed. Assertion will occur on a second registration
 *
 * Returns Error number; may be tested as boolean with 0=success, other=fail.
 */
int sw_fuel_gauge_register_hal(
	struct sw_fuel_gauge_hal_interface *p_sw_fuel_gauge_hal_interface,
		struct sw_fuel_gauge_interface **p_sw_fuel_gauge_interface);

#endif /* _SW_FUEL_GAUGE_HAL_H */
