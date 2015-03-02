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

#ifndef _INTEL_ADC_SENSORS_H_
#define _INTEL_ADC_SENSORS_H_

/* Driver registerd name and consumer name for ADC driver */
#define ADC_SENSORS_DRIVER_NAME	"intel_adc_sensors"

/**
 * adc_sensors_logical_channel - Full list of possible sensor channels
 */
enum adc_sensors_log_channel {
	ADC_SENSORS_CHANNEL_VBAT,
	ADC_SENSORS_CHANNEL_BATID,
	ADC_SENSORS_CHANNEL_PMICTEMP,
	ADC_SENSORS_CHANNEL_PMICTEMP1,
	ADC_SENSORS_CHANNEL_BATTEMP0,
	ADC_SENSORS_CHANNEL_SYSTEMP0,
	ADC_SENSORS_CHANNEL_SYSTEMP1,
	ADC_SENSORS_CHANNEL_SYSTEMP2,
	ADC_SENSORS_CHANNEL_VBAT_MIN,
	ADC_SENSORS_CHANNEL_VBAT_OCV,
	ADC_SENSORS_CHANNEL_ACCID,
	ADC_SENSORS_CHANNEL_HWID,
	ADC_SENSORS_CHANNEL_USBID,
	ADC_SENSORS_CHANNEL_TBATID,
	ADC_SENSORS_CHANNEL_MAX
};

/**
 * adc_sensors_conversion_type - Supported conversion methods
 *
 * NOTE: The CSPLINE conversion method will be fully enabled with a future
 * enhancement, as the Linux Kernel does not support floating point arithmetic.
 */
enum adc_sensors_conversion {
	ADC_SENSORS_CONVERSION_NONE,
	ADC_SENSORS_CONVERSION_LINEAR_UV_TO_MV,
	ADC_SENSORS_CONVERSION_LINEAR_UV_TO_OHM,
	ADC_SENSORS_CONVERSION_LINEAR_UV_TO_DEGC,
	ADC_SENSORS_CONVERSION_TABLE_UV_TO_DEGC,
	ADC_SENSORS_CONVERSION_CSPLINE_UV_TO_MDEGC,
};

/**
 * adc_sensors_data_linear_uv_to_mv - Conversion data for potential divider ratio
 * @scaled_conv_factor		Scaled up conversion factor (uV->mV)
 * @scaling_shift		Shift used for scaling down (power of two)
 */
struct adc_sensors_data_linear_uv_to_mv {
	u32 const scaled_conv_factor;
	u32 const scaling_shift;
};

/**
 * adc_sensors_data_linear_uv_to_ohm - Conversion data for current bias conversion
 * @series_resistor_ohm		Size of series resistor (Ohm)
 */
struct adc_sensors_data_linear_uv_to_ohm {
	int series_resistor_ohm;
};

/**
 * adc_sensors_data_linear_uv_to_degc - Conversion data for potential divider ratio
 * @scaled_conv_factor		Scaled up conversion factor (uV->degC)
 * @scaling_shift		Shift used for scaling down (power of two)
 * @offset			Offset for DegC
 */
struct adc_sensors_data_linear_uv_to_degc {
	u32 const scaled_conv_factor;
	u32 const scaling_shift;
	int const offset;
};

/**
 * adc_sensors_data_table_uv_to_degc - Simple lookup table for conversion
 * @parallel_resistor_present	True if a parallel resistor is fitted
 * @parallel_resistor_ohm	Value of parallel resistor, if fitted
 * @table_offset_degc		Temperature of the first table entry
 *				Each further entry is 1 degree higher
 * @table_size			Number of entries in the table
 * @p_table			Pointer to the start of the table
 */
struct adc_sensors_data_table_uv_to_degc {
	bool	parallel_resistor_present;
	int	parallel_resistor_ohm;
	int	table_offset_degc;
	size_t	table_size;
	int	*p_table;
};

/**
 * adc_sensors_cspline_segment - Cubic Spline Interpolation
 *
 * Temp = Ai (R-Ri)^3 + Bi (R-Ri)^2 + Ci (R-Ri) + Di
 *
 * Where:
 * R:	Measured thermistor resistance in Ohm
 * Ai, Bi, Ci, Di: coefficients for spline "i"
 * Ri: Start resistance value in Ohm for spline "i"
 * Temp: temperature in mDegC that corresponds to R
 *
 * @r_i		Start resistance (Ohm) of segment "i"
 * @a_i		A coefficient of segment "i"
 * @b_i		B coefficient of segment "i"
 * @c_i		C coefficient of segment "i"
 * @d_i		D coefficient of segment "i"
 */
struct adc_sensors_cspline {
	u32	r_i;
	float	a_i;
	float	b_i;
	float	c_i;
	float	d_i;
};

/**
 * adc_sensors_data_cspline_uv_to_mdegc - Conversion data using cubic spline method
 * @parallel_resistor_present		True if a parallel resistor is fitted
 * @parallel_resistor_ohm		Value of parallel resistor, if fitted
 * @size				Number of splines
 * @r_min				Min overall resistance (Ohm)
 * @r_max				Max overall resistance (Ohm)
 * @p_cspline_table			Pointer the cubic spline table
 */
struct adc_sensors_data_cspline_uv_to_mdegc {
	bool	parallel_resistor_present;
	int	parallel_resistor_ohm;
	u32	table_size;
	u32	r_min;
	u32	r_max;
	const struct adc_sensors_cspline *p_cspline_table;
};

/**
 * adc_sensors_calibration - Calibration offset and gain for two point calibration
 * @gain		Scaled up conversion factor
 * @offset		Offset in real world sensor units
 * @shift		Shift used for scaling down.
 */
struct adc_sensors_calibration {
	u32 gain;
	int offset;
	u32 shift;
};

/**
 * adc_sensors_channel - Generic descriptor type for single sensor channel
 * @consumer		The name of the consumer for this channel
 * @log_channel		The logical channel number
 * @conversion		Type of conversion method for the channel
 * @p_data		Pointer to data for first conversion method
 * @default_cal_data	Calibration data used if NVM values not available
 * @tolerance		Accuracy of sensor data (+/- in real units)
 */
struct adc_sensors_channel {
	char				*consumer;
	enum adc_sensors_log_channel	adc_log_chan;
	enum adc_sensors_conversion	conversion;
	void				*p_data;
	struct adc_sensors_calibration	default_cal_data;
	int				tolerance;
};

/**
 * adc_sensors_platform_data - Table of configured channels with allowed consumers
 * @p_channels		Pointer to table of logical channels and consumers
 * @num_channels		Number of entries in table
 */
struct adc_sensors_platform_data {
	struct adc_sensors_channel *p_channels;
	int num_channels;
};

#endif /* _INTEL_ADC_SENSORS_H_ */
