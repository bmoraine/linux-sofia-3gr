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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iio/machine.h>
#include <linux/iio/consumer.h>
#include <linux/iio/intel_adc_hal_interface.h>
#include "intel_adc_sensors.h"
#include "intel_adc_sensors_config.h"

#undef pr_fmt
#define pr_fmt(fmt) ADC_SENSORS_DRIVER_NAME": "fmt

/* Used to allocate IIO device with no user data area */
#define NO_PRIVATE_DATA			(0)

/* Marker for unused channel */
#define ADC_CHANNEL_UNMAPPED		(-1)

/* Size of debug data array (has to be power of 2!!!) */
#define ADC_SENSORS_DEBUG_DATA_SIZE	(1<<6)

/* Volt to micro volt factor */
#define UV_PER_V (1000000)

#define SYSFS_INPUT_VAL_LEN		(1)

/* Macro to trace and log debug data internally. Jiffy resolution is adequate */
#define ADC_SENSORS_DEBUG(_array, _event, _param) \
do { \
	spin_lock(&_array.lock); \
	_array.log_array[_array.index].time_stamp = jiffies; \
	_array.log_array[_array.index].event = (_event); \
	_array.log_array[_array.index].param = (int)(_param); \
	_array.index++; \
	_array.index &= (ADC_SENSORS_DEBUG_DATA_SIZE-1); \
	spin_unlock(&_array.lock); \
	pr_debug("%s 0x%lx  dec=%d\n", #_event, \
			(unsigned long)_param, (int)_param); \
} while (0)

/* Macro to trace and log debug data internally. Jiffy resolution is adequate */
#define ADC_SENSORS_DEBUG_STRING(_array, _event, _param) \
do { \
	spin_lock(&_array.lock); \
	_array.log_array[_array.index].time_stamp = jiffies; \
	_array.log_array[_array.index].event = (_event); \
	_array.log_array[_array.index].param = (int)(_param); \
	_array.index++; \
	_array.index &= (ADC_SENSORS_DEBUG_DATA_SIZE-1); \
	spin_unlock(&_array.lock); \
	pr_debug("%s %s\n", #_event, (char *)_param); \
} while (0)

/** Events for use in debug and tracing. */
enum adc_sensors_debug_event {
	ADC_SENSORS_DEBUG_EVENT_INIT,
	ADC_SENSORS_DEBUG_EVENT_EXIT,
	ADC_SENSORS_DEBUG_EVENT_PROBE,
	ADC_SENSORS_DEBUG_EVENT_REMOVE,

	ADC_SENSORS_DEBUG_EVENT_IIO_MAP_NUM_ENTRIES,
	ADC_SENSORS_DEBUG_EVENT_CONSUMER,
	ADC_SENSORS_DEBUG_EVENT_CONSUMER_ADC,
	ADC_SENSORS_DEBUG_EVENT_CONSUMER_SENSOR,
	ADC_SENSORS_DEBUG_EVENT_GET_ADC_CHANNEL,
	ADC_SENSORS_DEBUG_EVENT_ADD_SENSOR_CHANNEL,
	ADC_SENSORS_DEBUG_EVENT_NUM_SENSOR_CHANNELS,

	ADC_SENSORS_DEBUG_EVENT_READ_RAW,
	ADC_SENSORS_DEBUG_EVENT_READ_RAW_UV,
	ADC_SENSORS_DEBUG_EVENT_READ_RAW_NA,
	ADC_SENSORS_DEBUG_EVENT_READ_RAW_ERROR,
	ADC_SENSORS_DEBUG_EVENT_READ_RAW_TOLERANCE,
	ADC_SENSORS_DEBUG_EVENT_READ_RAW_CAL_OFFSET,
	ADC_SENSORS_DEBUG_EVENT_READ_RAW_CAL_GAIN,
	ADC_SENSORS_DEBUG_EVENT_READ_RAW_CALIBRATED,
	ADC_SENSORS_DEBUG_EVENT_READ_RAW_UNSUPPORTED,

	ADC_SENSORS_DEBUG_EVENT_WRITE_RAW,
	ADC_SENSORS_DEBUG_EVENT_WRITE_RAW_CAL_OFFSET,
	ADC_SENSORS_DEBUG_EVENT_WRITE_RAW_CAL_GAIN,
	ADC_SENSORS_DEBUG_EVENT_WRITE_RAW_CALIBRATED,
	ADC_SENSORS_DEBUG_EVENT_WRITE_RAW_UNSUPPORTED,

	ADC_SENSORS_DEBUG_EVENT_CONVERT_START,
	ADC_SENSORS_DEBUG_EVENT_CONVERT_NONE,
	ADC_SENSORS_DEBUG_EVENT_CONVERT_LINEAR_UV_MV,
	ADC_SENSORS_DEBUG_EVENT_CONVERT_LINEAR_UV_OHM,
	ADC_SENSORS_DEBUG_EVENT_CONVERT_TABLE_UV_DEGC,
	ADC_SENSORS_DEBUG_EVENT_CONVERT_TOTAL_OHMS,
	ADC_SENSORS_DEBUG_EVENT_CONVERT_THERMISTOR_OHMS,
	ADC_SENSORS_DEBUG_EVENT_CONVERT_THERMISTOR_ERROR,
	ADC_SENSORS_DEBUG_EVENT_CONVERT_THERMISTOR_NOT_IN_TABLE,
	ADC_SENSORS_DEBUG_EVENT_CONVERT_TABLE_VAL_OHMS,
	ADC_SENSORS_DEBUG_EVENT_CONVERT_CSPLINE_UV_MDEGC,
	ADC_SENSORS_DEBUG_EVENT_CONVERT_ERROR,
};

/**
 * struct adc_sensors_debug_data - Structure to collect debug data
 * @lock		Spinlock for atomic access
 * @index		Index of logging array
 * @log_array		Debug data logging array
 *	@time_stamp	System Time Stamp in Jiffies
 *	@event		Event which occurred
 *	@param		General purpose parameter
 */
struct adc_sensors_debug_data {
	spinlock_t lock;
	u32 index;
	struct {
		u32 time_stamp;
		enum adc_sensors_debug_event event;
		int param;
	} log_array[ADC_SENSORS_DEBUG_DATA_SIZE];
};

/* Macro to trace and log debug event and data. */
#define ADC_SENSORS_DEBUG_PARAM(_event, _param) \
		ADC_SENSORS_DEBUG(adc_sensors_debug_info, _event, _param)

/* Macro to trace and log debug event and text data */
#define ADC_SENSORS_DEBUG_STRING_PARAM(_event, _param) \
		ADC_SENSORS_DEBUG_STRING(adc_sensors_debug_info, _event, _param)

/* Macro to trace and log debug event without a parameter. */
#define ADC_SENSORS_DEBUG_NO_PARAM(_event) \
		ADC_SENSORS_DEBUG(adc_sensors_debug_info, _event, 0)

/*
 * adc_sensors_chan_info - Structure for mapping of ADC logical channels to
 * sensor channels
 */
struct adc_sensors_chan_info {
	const char *adc_sensor_name;
	const char *adc_name;
	const enum iio_chan_type iio_type;
};

/*
 * adc_sensors_config - ADC sensor channel calibration and configuration data
 */
struct adc_sensors_config {
	struct adc_sensors_channel *p_sensor_channel;
	struct adc_sensors_calibration cal_data;
	struct iio_channel *p_adc_iio_chan;
};

/*
 * adc_sensors_data - Internal state data for ADC sensors
 */
struct adc_sensors_data {
	struct iio_info info;
	struct iio_dev *p_iio_dev;
	struct iio_map *p_iio_map;
	struct adc_sensors_config *p_sensor_config;
	struct iio_chan_spec *p_iio_chan_spec;
	int num_sensor_chans;
};

/* Prototype declared for static data initialisation */
static int adc_sensor_read_raw(struct iio_dev *p_iiodev,
				struct iio_chan_spec const *p_chan,
				int *p_val, int *p_val2, long mask);

/* Prototype declared for static data initialisation */
static int adc_sensor_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask);
/*
 * adc_sensors_info - mapping of ADC logical channels to Sensor channels
 */
static const struct adc_sensors_chan_info
	adc_sensors_info[ADC_SENSORS_CHANNEL_MAX] = {
		{"VBAT_SENSOR", "VBAT_ADC", IIO_VOLTAGE},
		{"BATID_SENSOR", "BATID_ADC", IIO_RESISTANCE},
		{"PMICTEMP_SENSOR", "PMICTEMP_ADC", IIO_TEMP},
		{"PMICTEMP1_SENSOR", "PMICTEMP1_ADC", IIO_TEMP},
		{"BATTEMP0_SENSOR", "BATTEMP0_ADC", IIO_TEMP},
		{"SYSTEMP0_SENSOR", "SYSTEMP0_ADC", IIO_TEMP},
		{"SYSTEMP1_SENSOR", "SYSTEMP1_ADC", IIO_TEMP},
		{"SYSTEMP2_SENSOR", "SYSTEMP2_ADC", IIO_TEMP},
		{"VBAT_MIN_SENSOR", "VBAT_MIN_ADC", IIO_VOLTAGE},
		{"VBAT_OCV_SENSOR", "VBAT_OCV_ADC", IIO_VOLTAGE},
		{"ACCID_SENSOR", "ACCID_ADC", IIO_VOLTAGE},
		{"HWID_SENSOR", "HWID_ADC", IIO_RESISTANCE},
		{"USBID_SENSOR", "USBID_ADC", IIO_RESISTANCE},
		{"TBATID_SENSOR", "BATTEMP0_ADC", IIO_RESISTANCE},
};

/*
 * adc_sensors - internal state data for ADC sensors
 */
static struct adc_sensors_data adc_sensors = {
	.info = {
		.driver_module = THIS_MODULE,
		.read_raw = &adc_sensor_read_raw,
		.write_raw = &adc_sensor_write_raw,
	},
};

/* Array to collect debug data */
static struct adc_sensors_debug_data adc_sensors_debug_info;

/**
 * convert_to_iio_node_name() - Extract extended name of ADC.
 * e.g. ADC NAME - "VBAT_ADC", Extended name - "vbat"
 * @src:	Name of the ADC.
 * @return:	Extended name of ADC.
 */
static const char *convert_to_iio_node_name(const char *src)
{
	char *p, *dst;
	int i, len = 0;

	len	= strlen(src);
	p = kmalloc(len, GFP_KERNEL);
	BUG_ON(p == NULL);
	strcpy(p, src);
	dst = p;
	len = len - strlen("_ADC");
	for (i = 0; i < len; i++) {
		*p = tolower(*p);
		p++;
	}
	*p = '\0';
	return dst;
}


/**
 * adc_sensor_convert_linear_uv_mv - Convert uV to mV with linear transform
 * @adc_voltage_uv	Voltage read from ADC
 * @p_data		Platform configured conversion parameters
 * @p_val_mv		Converted result
 * returns		0 for success, or error code.
 */
static int adc_sensors_convert_linear_uv_mv(int adc_voltage_uv,
						struct
						adc_sensors_data_linear_uv_to_mv
						*p_data, int *p_val_mv)
{
	/* Platform data must be provided */
	BUG_ON(NULL == p_data);

	/* Multiply by internal divider ratio */
	*p_val_mv =
		(((((u64) p_data->scaled_conv_factor * (u64) adc_voltage_uv))
		>> (p_data->scaling_shift - 1)) + 1) >> 1;

	return 0;
}

/**
 * adc_sensors_convert_linear_uv_ohm - Convert uV to Ohm with linear transform
 * @adc_voltage_uv	Voltage read from ADC
 * @adc_current_na	Current source value, or 0 for voltage measurements
 * @p_data		Platform configured conversion parameters
 * @p_val_ohm		Converted result
 * returns		0 for success, or error code.
 */
static int adc_sensors_convert_linear_uv_ohm(
					int adc_voltage_uv,
					int adc_current_na,
					struct adc_sensors_data_linear_uv_to_ohm
					*p_data, int *p_val_ohm)
{
	int total_resistance_ohm;
	/* Platform data must be provided */
	BUG_ON(NULL == p_data);
	/*
	* Test for zero bias current. This would indicate an internal
	* ADC driver SW problem. Taking a resistor measurement
	* implicitly requires biasing the resistor with current.
	*/
	BUG_ON(0 == adc_current_na);
	/*
	* Calculate total resistance to ground
	* Dividing with rounding measurement result in mV by current bias in nA,
	* times scaling factor to bring result to Ohm
	* NOTE: uV value multiplied by 1000 to compensate as current unit is nA
	* (adc_value_uv * 1000)
	* To avoid overflow of 32-bit operation, when adc_voltage_uv is more
	 than 1V (1000000 uV), round off mechanism is not in place.
	*/

	if (adc_voltage_uv > UV_PER_V)
		total_resistance_ohm = ((adc_voltage_uv * 1000) /
			 adc_current_na);
	else
		total_resistance_ohm =
		(((adc_voltage_uv * 2000) / adc_current_na) + 1) >> 1;

	/* Calculate battery ID resistor value */
	if (p_data->series_resistor_ohm < total_resistance_ohm)
		*p_val_ohm = total_resistance_ohm - p_data->series_resistor_ohm;
	else
		*p_val_ohm = 0;

	return 0;
}

/**
 * adc_sensors_convert_linear_uv_degc - Convert uV to Deg C with linear
 *					transform
 *
 * @adc_voltage_uv	Voltage read from ADC
 * @p_data		Platform configured conversion parameters
 * @p_val_degc		Converted result
 * returns		0 for success, or error code.
 */
static int adc_sensors_convert_linear_uv_degc(
				int adc_voltage_uv,
				struct adc_sensors_data_linear_uv_to_degc
				*p_data, int *p_val_degc)
{
	/* Platform data must be provided */
	BUG_ON(NULL == p_data);

	/* Multiply by internal divider ratio */
	*p_val_degc = (int)((((adc_voltage_uv * p_data->scaled_conv_factor)
				>> (p_data->scaling_shift - 1)) + 1) >> 1)
		+ p_data->offset;
	return 0;
}

/**
 * adc_sensors_find_resistance_table_index - Find the correct value in the
 * table using a binary search.
 * NOTE:The table data is in reverse order due to the characteristics of a
 * thermistor,
 *	i.e. resistance decreases with increasing temperature.
 *
 * @p_data		Table structure to search
 * @resistance_ohms	Resistance (Ohms)
 * returns		Index of nearest value in table data or -ERANGE
*/
static int adc_sensors_find_resistance_table_index(
					struct adc_sensors_data_table_uv_to_degc
					*p_data, int r_ohms)
{
	int *p_table = p_data->p_table;
	/* Start the search in the middle of the table. */
	int index = (p_data->table_size / 2);
	int last_index;
	/* Initialise the search step to half of the remaining values. */
	int step = (index / 2);

	if ((r_ohms > p_table[0]) || (r_ohms < p_table[p_data->table_size]))
		return -ERANGE;

	/* Binary search table for value. */
	do {
		if (p_table[index] < r_ohms) {
			/* Check lower part of range */
			index -= step;
		} else if (p_table[index] > r_ohms) {
			/* Check upper part of range */
			index += step;
		} else {
			/* Exact match was found. Terminate the search. */
			return index;
		}
		step >>= 1;
	} while (step > 0);
	/*
	* Refine value found with the binary search. The result is very close
	* and it should not take many iterations.
	*/
	do {
		last_index = index;
		/* If the value is more than half way to the next lowest table
		entry, step down. */
		if ((index > 0)
			&& (r_ohms > ((p_table[index] +
				p_table[index - 1]) >> 1))) {
			index--;
		}
		/* If the value is more than half way to the next highest table
		entry, step up. */
		if ((index < (p_data->table_size - 1))
			&& (r_ohms < ((p_table[index] +
				p_table[index + 1]) >> 1))) {
			index++;
		}
	} while (last_index != index);

	return index;
}

/**
 * adc_sensors_convert_table_uv_degc - Convert uV to milli deg C using lookup
 *			table
 *
 * @adc_voltage_uv	Voltage read from ADC
 * @adc_current_na	Current source value, or 0 for voltage measurements
 * @p_data		Platform configured conversion parameters
 * @p_val		Converted result
 * returns		0 or -ERANGE if the value cannot be found
 */
static int adc_sensors_convert_table_uv_degc(
					int adc_voltage_uv,
					int adc_current_na,
					struct adc_sensors_data_table_uv_to_degc
					*p_data, int *p_val_degc)
{
	int total_resistance_ohm;
	int thermistor_ohm;
	int difference_ohm;
	int index;
	/*
	* Test for zero bias current. This would indicate an internal
	* ADC driver SW problem. Taking a temperature measurement
	* implicitly requires biasing the thermistor with current.
	*/
	BUG_ON(0 == adc_current_na);
	/* Calculate the total resistance from voltage and current */
	total_resistance_ohm = (adc_voltage_uv * 1000) / adc_current_na;
	thermistor_ohm = total_resistance_ohm;
	ADC_SENSORS_DEBUG_PARAM(ADC_SENSORS_DEBUG_EVENT_CONVERT_TOTAL_OHMS,
				total_resistance_ohm);

	/* If a parallel resistor is present, calculate the thermistor value */
	if (p_data->parallel_resistor_present) {
		difference_ohm =
			p_data->parallel_resistor_ohm - total_resistance_ohm;
		if (0 != difference_ohm) {
			u64 result_ohm_64bit =
				((u64) total_resistance_ohm *
				(u64) p_data->parallel_resistor_ohm);
			/*                              / difference_ohm; */
			/* Standard C divide is not supported fot 64 bit
			values */
			do_div(result_ohm_64bit, difference_ohm);
			thermistor_ohm = result_ohm_64bit;
		} else {
			ADC_SENSORS_DEBUG_PARAM(
			 ADC_SENSORS_DEBUG_EVENT_CONVERT_THERMISTOR_ERROR,
								-ERANGE);
			return -ERANGE;
		}
	}

	ADC_SENSORS_DEBUG_PARAM(ADC_SENSORS_DEBUG_EVENT_CONVERT_THERMISTOR_OHMS,
				thermistor_ohm);
	index = adc_sensors_find_resistance_table_index(p_data, thermistor_ohm);

	/* Check that the resistance value is in range */
	if (0 > index) {
		ADC_SENSORS_DEBUG_NO_PARAM(
		 ADC_SENSORS_DEBUG_EVENT_CONVERT_THERMISTOR_NOT_IN_TABLE);
		return index;
	} else {
		*p_val_degc = index + p_data->table_offset_degc;
	}

	return 0;
}

/* Floating point support is not currently available */
#if 0
/**
 * adc_sensors_find_cublic_spline_table_index - Find the correct cublic spline
 * in the table of splines using a binary search.
 *
 * This function MUST be re-entrant!.
 *
 * @resistance_ohms	Resistance (Ohms)
 * @index		Starting search index
 * @offset		Offset from the start of the table, usually the value
 *			from the last search
 *			as this function can be used recursively. This value
 *			is 0 if it is the first search)
 * returns		Index into cublic spline data
*/
static u32 adc_sensors_find_cublic_spline_table_index(
				struct adc_sensors_data_cspline_uv_to_mdegc
				*p_data,
				u32 resistance_ohms,
				u32 index, u32 offset)
{
	u32 result;
	u32 i;

	/* Platform data must be provided */
	BUG_ON(NULL == p_data);
	BUG_ON(NULL == p_data->p_cspline_table);

	index /= 2;
	i = index + offset;
	result = i;

	if (p_data->p_cspline_table[i].r_i > resistance_ohms) {
		result =
			adc_sensors_find_cublic_spline_table_index(p_data,
								resistance_ohms,
								index, offset);
	} else {
		if ((p_data->table_size - 1) > i) {
			if (!
				((p_data->p_cspline_table[i].r_i <=
								resistance_ohms)
				&& (p_data->p_cspline_table[i + 1].r_i >
				resistance_ohms))) {
				result =
				 adc_sensors_find_cublic_spline_table_index
				  (p_data, resistance_ohms, index, i);
			}
		}
	}
	return result;
}
#endif

/**
 * adc_sensors_convert_cspline_ohm_mdegc - Convert Ohm to milli deg C using
 *			cspline transform
 *
 * @adc_voltage_uv	Voltage read from ADC
 * @adc_current_na	Current source value. Must not be 0
 * @p_data		Platform configured conversion parameters
 * @p_val_mdegc		Converted result in milli degrees C
 * returns		0 for success, or error code.
 */
static int adc_sensors_convert_cspline_uv_mdegc(
			int adc_voltage_uv,
			int adc_current_na,
			struct adc_sensors_data_cspline_uv_to_mdegc *p_data,
			int *p_val_mdegc)
{
	int total_resistance_ohm;
	int thermistor_ohm;
	int difference_ohm;
	int ret = 0;
	/*
	* Test for zero bias current. This would indicate an internal
	* ADC driver SW problem. Taking a temperature measurement
	* implicitly requires biasing the thermistor with current.
	*/
	BUG_ON(0 == adc_current_na);
	/* Calculate the total resistance from voltage and current */
	total_resistance_ohm = (adc_voltage_uv * 1000) / adc_current_na;
	thermistor_ohm = total_resistance_ohm;
	ADC_SENSORS_DEBUG_PARAM(ADC_SENSORS_DEBUG_EVENT_CONVERT_TOTAL_OHMS,
				total_resistance_ohm);

	/* If a parallel resistor is present, calculate the thermistor value */
	if (p_data->parallel_resistor_present) {
		difference_ohm =
			p_data->parallel_resistor_ohm - total_resistance_ohm;
		if (0 != difference_ohm) {
			u64 result_ohm_64bit =
				((u64) total_resistance_ohm *
				(u64) p_data->parallel_resistor_ohm);
			/*                              / difference_ohm; */
			/* Standard C divide is not supported fot 64 bit
			values */
			do_div(result_ohm_64bit, difference_ohm);
			thermistor_ohm = result_ohm_64bit;
		}
	}
	ADC_SENSORS_DEBUG_PARAM(ADC_SENSORS_DEBUG_EVENT_CONVERT_THERMISTOR_OHMS,
				total_resistance_ohm);
	/* Check that the resistance value is in range */
	if (((p_data->parallel_resistor_present) && (0 == difference_ohm))
		|| ((p_data->r_min > thermistor_ohm)
		|| (p_data->r_max < thermistor_ohm))) {
		ret = -ERANGE;
	} else {
/* Floating point support is not currently available */
#if 0
		const struct adc_sensors_cspline *p_table =
			p_data->p_cspline_table;
		/*
		* Cubic Spline Interpolation
		*
		* Temp = Ai (R-Ri)^3 + Bi (R-Ri)^2 + Ci (R-Ri) + Di
		*
		* Where:
		* R: Measured thermistor resistance in Ohm
		* Ai, Bi, Ci, Di: coefficients for spline "i"
		* Ri: Start resistance value in Ohm for spline "i"
		* Temp: temperature in mDegC that corresponds to R
		*/
		u32 i;
		float r_calc;
		float r_calc2;
		float r_calc3;

		i = adc_sensors_find_cublic_spline_table_index(p_data,
								thermistor_ohm,
								p_data->
								table_size, 0);
		r_calc = thermistor_ohm - p_table[i].r_i;
		r_calc2 = r_calc * r_calc;
		r_calc3 = r_calc2 * r_calc;
		*p_val_mdegc = (int)((p_table[i].a_i * r_calc3)
					+ (p_table[i].b_i * r_calc2)
					+ (p_table[i].c_i * r_calc)
					+ p_table[i].d_i);
#endif
		ret = -EINVAL;
	}
	return ret;
}

/**
 * adc_sensors_convert() - Convert the ADC ready using supplied transform data
 * @conversion		Conversion to apply
 * @p_data		Platform configured conversion parameters
 * @adc_voltage_uv	Voltage read from ADC
 * @adc_current_na	Current source value, or 0 for voltage measurements
 * @p_val		Converted result
 * returns		0 for success, or error code.
 */
static int adc_sensors_convert(enum adc_sensors_conversion conversion,
				void *p_data,
				int adc_voltage_uv,
				int adc_current_na, int *p_val)
{
	int ret = 0;

	ADC_SENSORS_DEBUG_NO_PARAM(ADC_SENSORS_DEBUG_EVENT_CONVERT_START);

	switch (conversion) {
	case ADC_SENSORS_CONVERSION_NONE:
		*p_val = adc_voltage_uv;
		ADC_SENSORS_DEBUG_PARAM(ADC_SENSORS_DEBUG_EVENT_CONVERT_NONE,
					*p_val);
		break;

	case ADC_SENSORS_CONVERSION_LINEAR_UV_TO_MV:
		ret = adc_sensors_convert_linear_uv_mv(adc_voltage_uv,
			(struct adc_sensors_data_linear_uv_to_mv *)p_data,
									p_val);

		ADC_SENSORS_DEBUG_PARAM
			(ADC_SENSORS_DEBUG_EVENT_CONVERT_LINEAR_UV_MV, *p_val);
		break;

	case ADC_SENSORS_CONVERSION_LINEAR_UV_TO_OHM:
		ret = adc_sensors_convert_linear_uv_ohm(
			adc_voltage_uv, adc_current_na,
			 (struct adc_sensors_data_linear_uv_to_ohm *)p_data,
									p_val);

		ADC_SENSORS_DEBUG_PARAM
			(ADC_SENSORS_DEBUG_EVENT_CONVERT_LINEAR_UV_OHM, *p_val);
		break;

	case ADC_SENSORS_CONVERSION_LINEAR_UV_TO_DEGC:
		ret = adc_sensors_convert_linear_uv_degc(adc_voltage_uv,
			(struct adc_sensors_data_linear_uv_to_degc *)p_data,
									p_val);

		ADC_SENSORS_DEBUG_PARAM
			(ADC_SENSORS_DEBUG_EVENT_CONVERT_LINEAR_UV_MV, *p_val);
		break;

	case ADC_SENSORS_CONVERSION_TABLE_UV_TO_DEGC:
		ret = adc_sensors_convert_table_uv_degc(
			adc_voltage_uv, adc_current_na,
			 (struct adc_sensors_data_table_uv_to_degc *)p_data,
									p_val);

		ADC_SENSORS_DEBUG_PARAM
			(ADC_SENSORS_DEBUG_EVENT_CONVERT_TABLE_UV_DEGC, *p_val);
		break;

	case ADC_SENSORS_CONVERSION_CSPLINE_UV_TO_MDEGC:
		ret = adc_sensors_convert_cspline_uv_mdegc(
			adc_voltage_uv, adc_current_na,
			 (struct adc_sensors_data_cspline_uv_to_mdegc *)p_data,
									p_val);

		ADC_SENSORS_DEBUG_PARAM(
			ADC_SENSORS_DEBUG_EVENT_CONVERT_CSPLINE_UV_MDEGC,
									*p_val);
		break;

	default:
		/* Unrecogised conversion type - bad config data */
		BUG();
		break;
	}
	return ret;
}

/**
 * adc_sensor_read_raw() - data read function. The raw value is read from the
 * ADC driver and then converted to the real world value.
 * @p_iiodev:		Pointer to the sensor IIO device.
 * @p_chan_spec:	Channel descriptor.
 * @p_val:		Return value read from the channel
 * @p_val2:		Not used.
 * @mask:		Channel information to read e.g adc value,
 *			calibration data
 * returns		IIO_VAL_INT for success, or error code.
 */
static int adc_sensor_read_raw(struct iio_dev *p_iiodev,
				struct iio_chan_spec const *p_chan_spec,
				int *p_val, int *p_val2, long mask)
{
	int sensor_chan = p_chan_spec->channel;
	struct adc_sensors_config *p_config =
		&adc_sensors.p_sensor_config[sensor_chan];
	struct adc_sensors_channel *p_channel = p_config->p_sensor_channel;
	struct adc_sensors_calibration *p_cal = &p_config->cal_data;
	int ret = IIO_VAL_INT;
	/* Measurements are calibrated by default */
	bool apply_calibration = true;

	/* Not used */
	(void)p_iiodev;
	(void)p_val2;

	switch (mask) {
	case IIO_CHAN_INFO_NO_CALIBRATION:
		apply_calibration = false;
		/* Fall throught to chanel read is intentional */



	case IIO_CHAN_INFO_PROCESSED:{
			/* Read raw value from the ADC driver */
			int adc_voltage_uv;
			int adc_current_na;
			int adc_ret;

			ADC_SENSORS_DEBUG_STRING_PARAM
				(ADC_SENSORS_DEBUG_EVENT_READ_RAW,
				adc_sensors_info[p_channel->adc_log_chan].
				adc_sensor_name);

		adc_ret =
			iio_channel_read(
				p_config->p_adc_iio_chan,
				&adc_voltage_uv,
				&adc_current_na,
				IIO_CHAN_INFO_RAW);

			/* If the read was successful, convert the received
			values */
			if (IIO_VAL_INT == adc_ret) {
				ADC_SENSORS_DEBUG_PARAM
					(ADC_SENSORS_DEBUG_EVENT_READ_RAW_UV,
					adc_voltage_uv);
				ADC_SENSORS_DEBUG_PARAM
					(ADC_SENSORS_DEBUG_EVENT_READ_RAW_NA,
					adc_current_na);
				if (0 ==
					adc_sensors_convert(
							p_channel->conversion,
							p_channel->p_data,
							adc_voltage_uv,
							adc_current_na,
								p_val)) {
					/* Apply calibration if required */
					if (apply_calibration) {
						int cal_val =
							(*p_val +
							p_cal->offset);

					if (cal_val >= 0) {
							*p_val =
							((((int64_t) cal_val *
							 p_cal->gain) >>
								(p_cal->shift -
								1)) + 1) >> 1;
					} else {
							*p_val =
							-((-((int64_t)cal_val *
							p_cal->gain) >>
								(p_cal->shift -
								1)) + 1) >> 1;
						}
						ADC_SENSORS_DEBUG_PARAM(
				 ADC_SENSORS_DEBUG_EVENT_READ_RAW_CALIBRATED,
							*p_val);
					}
				} else {
					ret = -ERANGE;
					ADC_SENSORS_DEBUG_PARAM(
					 ADC_SENSORS_DEBUG_EVENT_CONVERT_ERROR,
									ret);
				}
			} else {
				ret = adc_ret;
				ADC_SENSORS_DEBUG_PARAM
					(ADC_SENSORS_DEBUG_EVENT_READ_RAW_ERROR,
					ret);
			}
			break;
		}
	case IIO_CHAN_INFO_TOLERANCE:
		/* Tolerance information is configured with the device */
		*p_val = p_channel->tolerance;
		ADC_SENSORS_DEBUG_PARAM
			(ADC_SENSORS_DEBUG_EVENT_READ_RAW_TOLERANCE, *p_val);
		break;

	case IIO_CHAN_INFO_CALIBBIAS:
		*p_val = p_cal->offset;
		ADC_SENSORS_DEBUG_PARAM
			(ADC_SENSORS_DEBUG_EVENT_READ_RAW_CAL_OFFSET, *p_val);
		break;

	case IIO_CHAN_INFO_CALIBSCALE:
		*p_val = p_cal->gain;
		ADC_SENSORS_DEBUG_PARAM
			(ADC_SENSORS_DEBUG_EVENT_READ_RAW_CAL_GAIN, *p_val);
		break;

	default:
		/* No other operation is supported */
		ADC_SENSORS_DEBUG_PARAM
			(ADC_SENSORS_DEBUG_EVENT_READ_RAW_UNSUPPORTED, mask);
		ret = -EINVAL;
		break;
	}
	return ret;
}

/**
 * adc_sensor_write_raw() - Parameter write function. The calculated calibration
 * offset and gain can be set after making uncalibrated measurements
 *
 * @p_iiodev:		Pointer to the sensor IIO device.
 * @p_chan_spec:	Channel descriptor
 * @val:		Value to write
 * @val2:		Not used
 * @mask:		Parameter to write: Calibration offset or gain.
 * returns		0 for success, or error code.
 */
static int adc_sensor_write_raw(struct iio_dev *p_iiodev,
				struct iio_chan_spec const *p_chan_spec,
				int val, int val2, long mask)
{
	int ret = 0;
	int sensor_chan = p_chan_spec->channel;
	struct adc_sensors_config *p_config =
		&adc_sensors.p_sensor_config[sensor_chan];
	struct adc_sensors_channel *p_channel = p_config->p_sensor_channel;
	struct adc_sensors_calibration *p_cal = &p_config->cal_data;

	/* Not used */
	(void)p_iiodev;
	(void)val2;

	ADC_SENSORS_DEBUG_STRING_PARAM(ADC_SENSORS_DEBUG_EVENT_WRITE_RAW,
					adc_sensors_info[p_channel->
							adc_log_chan].
					adc_sensor_name);

	switch (mask) {

	case IIO_CHAN_INFO_CALIBBIAS:
		p_cal->offset = val;
		ADC_SENSORS_DEBUG_PARAM
			(ADC_SENSORS_DEBUG_EVENT_WRITE_RAW_CAL_OFFSET, val);
		break;

	case IIO_CHAN_INFO_CALIBSCALE:
		p_cal->gain = val;
		ADC_SENSORS_DEBUG_PARAM
			(ADC_SENSORS_DEBUG_EVENT_WRITE_RAW_CAL_GAIN, val);
		break;

	default:
		/* No other operation is supported */
		ADC_SENSORS_DEBUG_PARAM
			(ADC_SENSORS_DEBUG_EVENT_WRITE_RAW_UNSUPPORTED, mask);
		ret = -EINVAL;
		break;
	}
	return ret;
}

/**
 * adc_sensors_probe - sensor device probe function.
 * @p_platform_dev:	The platform device that is used to
 *			pass conversion tables.
 *
 * returns		0 for success, or error code.
 */
static int adc_sensors_probe(struct platform_device *p_platform_dev)
{
	struct adc_sensors_platform_data *p_platform_data;
	struct adc_sensors_channel *p_channels;
	struct adc_sensors_channel
			*p_chan_map[ADC_SENSORS_CHANNEL_MAX] = { NULL, };
	struct device_node *np = p_platform_dev->dev.of_node;
	int i, ret;
	int adc_chan;
	int sensor_chan = 0;
	int iio_map_entries;
	const char *platform_name;

	ADC_SENSORS_DEBUG_NO_PARAM(ADC_SENSORS_DEBUG_EVENT_PROBE);

#ifdef CONFIG_OF
	ret = of_property_read_string(np,
			"intel,platform_name",
			&platform_name);
	if (ret) {
		pr_err("%s platform configuration not found\n", __func__);
		BUG_ON(1);
	}
	pr_err("platform_name = %s\n", platform_name);

	if (!strcmp(platform_name, "ag620"))
		p_platform_data = &xgold_intel_adc_sensors_ag620_data;
	else if (!strcmp(platform_name, "pmic"))
		p_platform_data = &xgold_intel_adc_sensors_pmic_data;
	else
		BUG_ON(1);
#else
	p_platform_data = &xgold_intel_adc_sensors_ag620_data;
#endif

	/* Platform data is required */
	BUG_ON(NULL == p_platform_data);
	p_channels = p_platform_data->p_channels;
	BUG_ON(NULL == p_channels);

	/* Construct the IIO consumer / channel map */
	iio_map_entries = p_platform_data->num_channels;
	/*
	* The IIO map is a NULL terminated list, allocate 1 extra structure
	* element. The last element is automatically set to NULL by kcalloc()
	*/
	adc_sensors.p_iio_map = kcalloc(iio_map_entries + 1,
					sizeof(struct iio_map), GFP_KERNEL);
	BUG_ON(NULL == adc_sensors.p_iio_map);
	ADC_SENSORS_DEBUG_PARAM(ADC_SENSORS_DEBUG_EVENT_IIO_MAP_NUM_ENTRIES,
				iio_map_entries);

	/*
	* Work through each consumer / channel entry to construct the IIO map
	* and determine the required ADC driver channels
	*/
	for (i = 0; i < iio_map_entries; i++) {
		/*
		* The consumer name must be specified and
		* the logical channel within the valid range
		*/
		BUG_ON(NULL == p_channels[i].consumer);
		adc_chan = p_channels[i].adc_log_chan;
		BUG_ON(ADC_SENSORS_CHANNEL_MAX <= adc_chan);
		BUG_ON(0 > adc_chan);

		adc_sensors.p_iio_map[i].consumer_channel =
			adc_sensors_info[adc_chan].adc_sensor_name;
		adc_sensors.p_iio_map[i].adc_channel_label =
			adc_sensors_info[adc_chan].adc_sensor_name;
		adc_sensors.p_iio_map[i].consumer_dev_name =
			p_channels[i].consumer;
		ADC_SENSORS_DEBUG_STRING_PARAM(ADC_SENSORS_DEBUG_EVENT_CONSUMER,
						adc_sensors.p_iio_map[i].
						consumer_dev_name);
		ADC_SENSORS_DEBUG_STRING_PARAM
			(ADC_SENSORS_DEBUG_EVENT_CONSUMER_ADC,
			adc_sensors_info[adc_chan].adc_name);
		ADC_SENSORS_DEBUG_STRING_PARAM
			(ADC_SENSORS_DEBUG_EVENT_CONSUMER_SENSOR,
			adc_sensors.p_iio_map[i].adc_channel_label);
		/*
		* Note the ADC channel corresponding to the sensor consumer
		* channel, unless it has already been done. The first time
		* an ADC logical channel is encountered determines its config.
		*/
		if (NULL == p_chan_map[adc_chan]) {
			p_chan_map[adc_chan] = &p_channels[i];
			/* Keep track of how many actual channels are used */
			sensor_chan++;
		}
	}
	ADC_SENSORS_DEBUG_PARAM(ADC_SENSORS_DEBUG_EVENT_NUM_SENSOR_CHANNELS,
				sensor_chan);
	/* Allocate enough IIO channel specifiers for all the sensors */
	adc_sensors.p_iio_chan_spec = kcalloc(sensor_chan,
						sizeof(struct iio_chan_spec),
							GFP_KERNEL);
	BUG_ON(NULL == adc_sensors.p_iio_chan_spec);
	/* Allocate memory for all channel config and calibration data. */
	adc_sensors.p_sensor_config = kcalloc(sensor_chan,
					sizeof(struct adc_sensors_config),
								GFP_KERNEL);
	BUG_ON(NULL == adc_sensors.p_sensor_config);
	adc_sensors.num_sensor_chans = sensor_chan;
	/*
	* A further pass through the ADC channels is necessary to fill
	* in the IIO channel specifiers and configuration for the allocated
	* sensor channels.
	*/
	sensor_chan = 0;
	for (adc_chan = 0; adc_chan < ADC_SENSORS_CHANNEL_MAX; adc_chan++) {

		if (NULL != p_chan_map[adc_chan]) {
			struct adc_sensors_config *p_config =
				&adc_sensors.p_sensor_config[sensor_chan];
			struct iio_chan_spec *p_spec =
				&adc_sensors.p_iio_chan_spec[sensor_chan];

			ADC_SENSORS_DEBUG_STRING_PARAM
				(ADC_SENSORS_DEBUG_EVENT_GET_ADC_CHANNEL,
				adc_sensors_info[adc_chan].adc_name);
			ADC_SENSORS_DEBUG_STRING_PARAM
				(ADC_SENSORS_DEBUG_EVENT_ADD_SENSOR_CHANNEL,
				adc_sensors_info[adc_chan].adc_sensor_name);
			/* Get the ADC channel for this sensor */
			p_config->p_adc_iio_chan =
				iio_channel_get(&p_platform_dev->dev,
					adc_sensors_info[adc_chan].adc_name);
			BUG_ON(IS_ERR(p_config->p_adc_iio_chan));
			/*
			* Store reference to the sensor configuration and
			* make a writable copy of the default calibration data.
			*/
			p_config->p_sensor_channel = p_chan_map[adc_chan];
			p_config->cal_data =
				p_chan_map[adc_chan]->default_cal_data;

			/* Fill in the necessary IIO fields */
			p_spec->datasheet_name =
				adc_sensors_info[adc_chan].adc_sensor_name;
			p_spec->extend_name = convert_to_iio_node_name(
				adc_sensors_info[adc_chan].adc_name);
			p_spec->type = adc_sensors_info[adc_chan].iio_type;
			p_spec->channel = sensor_chan;
			p_spec->indexed = false;
			p_spec->output  = true;
			p_spec->info_mask_separate =
					BIT(IIO_CHAN_INFO_PROCESSED);
			sensor_chan++;
		}
	}
	/*
	* Configure and register the device. Setting INDIO_DIRECT_MODE
	* exports the buffer to userspace through sysfs
	*/
	adc_sensors.p_iio_dev = iio_device_alloc(NO_PRIVATE_DATA);
	BUG_ON(NULL == adc_sensors.p_iio_dev);

	adc_sensors.p_iio_dev->name = ADC_SENSORS_DRIVER_NAME;
	adc_sensors.p_iio_dev->channels = adc_sensors.p_iio_chan_spec;
	adc_sensors.p_iio_dev->num_channels = adc_sensors.num_sensor_chans;
	adc_sensors.p_iio_dev->info = &adc_sensors.info;
	adc_sensors.p_iio_dev->modes = INDIO_DIRECT_MODE;
	adc_sensors.p_iio_dev->dev.parent = &p_platform_dev->dev;

	BUG_ON(0 != iio_device_register(adc_sensors.p_iio_dev));

	/* After device registration, the IIO consumer / channel maps can be
	added */
	BUG_ON(0 !=
		iio_map_array_register(adc_sensors.p_iio_dev,
					adc_sensors.p_iio_map));
	/*
	* There are no recoverable errors for this function.
	* Failure will cause a panic.
	*/

	return 0;
}

/**
 * adc_sensors_remove - sensor device remove function.
 * @p_platform_dev:	The platform device that is used to
 *			pass conversion tables.
 *
 * returns		0 for success, or error code.
 */
static int adc_sensors_remove(struct platform_device *dev)
{
	int sensor_chan;

	ADC_SENSORS_DEBUG_NO_PARAM(ADC_SENSORS_DEBUG_EVENT_PROBE);
	/* Free the IIO consumer to channel map memory */
	BUG_ON(0 != iio_map_array_unregister(adc_sensors.p_iio_dev));
	kfree(adc_sensors.p_iio_map);

	iio_device_unregister(adc_sensors.p_iio_dev);
	iio_device_free(adc_sensors.p_iio_dev);

	/* Free the acquired ADC channels */
	for (sensor_chan = 0; sensor_chan < adc_sensors.num_sensor_chans;
		sensor_chan++) {
		iio_channel_release(adc_sensors.p_sensor_config[sensor_chan].
					p_adc_iio_chan);
	}
	/* Free the ADC Sensor channel memory. */
	kfree(adc_sensors.p_iio_chan_spec);
	kfree(adc_sensors.p_sensor_config);

	return 0;
}

static const struct of_device_id xgold_sensor_of_match[] = {
	{
	 .compatible = "intel,adc_sensors",
	 },
	{}
};

MODULE_DEVICE_TABLE(of, xgold_sensor_of_match);

/**
 * adc_sensors_platform_driver - driver structure for ADC Sensors.
 */
static struct platform_driver adc_sensors_platform_driver = {
	.driver = {
		.name = ADC_SENSORS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(xgold_sensor_of_match),
	},
	.probe = adc_sensors_probe,
	.remove = adc_sensors_remove,
};

/**
 * adc_sensors_init - sensor device init function.
 * returns		0 for success, or error code.
 */
static int __init adc_sensors_init(void)
{
	/* NOTE: Must initialise debug data spinlock before any logging. */
	spin_lock_init(&adc_sensors_debug_info.lock);
	ADC_SENSORS_DEBUG_NO_PARAM(ADC_SENSORS_DEBUG_EVENT_INIT);
	return platform_driver_register(&adc_sensors_platform_driver);
}

/**
 * adc_sensors_exit - sensor device deinit function.
 */
static void __exit adc_sensors_exit(void)
{
	ADC_SENSORS_DEBUG_NO_PARAM(ADC_SENSORS_DEBUG_EVENT_EXIT);
	platform_driver_unregister(&adc_sensors_platform_driver);
}

module_init(adc_sensors_init);
module_exit(adc_sensors_exit);

MODULE_DESCRIPTION("Intel ADC Sensors Driver");
MODULE_LICENSE("GPL v2");

