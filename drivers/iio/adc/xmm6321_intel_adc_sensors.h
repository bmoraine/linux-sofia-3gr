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

#ifndef _XMM6321_INTEL_ADC_SENSORS_H_
#define _XMM6321_INTEL_ADC_SENSORS_H_

#include "intel_adc_sensors.h"

/**
 * VBAT is measured internally in MUT HW
 * MUT's internal voltage divider is as follows:
 *
 *	VBAT (V)
 *
 *	(78.4k)
 *		---> System Voltage ADC Input (M8)
 *	(24k)
 *
 *	GND
*/
#define VBAT_UV_TO_MV_FACTOR	(1000)
/* Resistor divider pull-up (Ohms) */
#define VBAT_PULL_UP		(78400)
/* Resistor divider pull-down (Ohms) */
#define VBAT_PULL_DOWN		(24000)
/* Measurement factor's scaling shift */
#define VBAT_SCALING_SHIFT	(20)
/* Scaled inverse conversion factor:
((Pull-up + Pull-down) * Scaling Factor) / (Pull-down * uV_to_mV_factor) */
#define VBAT_SCALED_CONV_FACTOR \
	(u32)(((u64)(VBAT_PULL_UP + VBAT_PULL_DOWN) << VBAT_SCALING_SHIFT) \
	/ (VBAT_PULL_DOWN * VBAT_UV_TO_MV_FACTOR))

/* Default calibration data - no change to measurement */
#define CAL_DEFAULT_GAIN	(8192)
#define CAL_DEFAULT_OFFSET	(0)
#define CAL_DEFAULT_SHIFT	(13)
#define CAL_DEFAULT \
		{CAL_DEFAULT_GAIN, CAL_DEFAULT_OFFSET, CAL_DEFAULT_SHIFT}

/* VBAT accuracy (+/-) in mV */
#define VBAT_TOLERANCE_MV	(5)

static struct adc_sensors_data_linear_uv_to_mv adc_sensors_vbat_data = {
	VBAT_SCALED_CONV_FACTOR,
	VBAT_SCALING_SHIFT,
};

/**
 * Battery Temperature is measured as follows:
 *
 *	Agold620 - 10 kOhm thermistor, no parallel resistor.
 *
 * Resistance table (Ohm) has 166 entries from -40 to 125 Degrees C inclusive
 */
#define TBAT_PARALLEL_RESISTOR_PRESENT	(false)
#define TBAT_PARALLEL_RESISTOR_OHMS	(0)
#define TBAT_TABLE_START_TEMP_DEGC	(-40)
static int adc_sensors_tbat_table[] = {
	195652,	184917,	174845,	165391,	156513,	148171,	140330,	132958,	126022,
	119494,
	113347,	107565,	102116,	96978, 92132, 87559, 83242, 79166, 75316, 71677,
	68237, 64991, 61919, 59011, 56258, 53650, 51178, 48835, 46613, 44506,
	42506, 40600, 38791, 37073, 35442, 33892, 32420, 31020, 29689, 28423,
	27219, 26076, 24988, 23951, 22963, 22021, 21123, 20267, 19450, 18670,
	17926, 17214, 16534, 15886, 15266, 14674, 14108, 13566, 13049, 12554,
	12081, 11628, 11195, 10780, 10382, 10000, 9634, 9284, 8947, 8624,
	8315, 8018, 7734, 7461, 7199, 6948, 6707, 6475, 6253, 6039,
	5834, 5636, 5445, 5262, 5086, 4917, 4754, 4597, 4446, 4301,
	4161, 4026, 3896, 3771, 3651, 3535, 3423, 3315, 3211, 3111,
	3014, 2922, 2834, 2748, 2666, 2586, 2509, 2435, 2364, 2294,
	2228, 2163, 2100, 2040, 1981, 1925, 1870, 1817, 1766, 1716,
	1669, 1622, 1578, 1535, 1493, 1452, 1413, 1375, 1338, 1303,
	1268, 1234, 1202, 1170, 1139, 1110, 1081, 1053, 1026, 999,
	974, 949, 925, 902, 880, 858, 837, 816, 796, 777,
	758, 740, 722, 705, 688, 672, 656, 640, 625, 611,
	596, 583, 569, 556, 544, 531
	};


static struct adc_sensors_data_table_uv_to_degc adc_sensors_tbat_data = {
	TBAT_PARALLEL_RESISTOR_PRESENT,
	TBAT_PARALLEL_RESISTOR_OHMS,
	TBAT_TABLE_START_TEMP_DEGC,
	sizeof(adc_sensors_tbat_table) / sizeof(int),
	adc_sensors_tbat_table,
};

/**
 * Battery ID is measured as follows:
 *
 *	Agold620 - 10kOhm series resistor.
 */
#define BATID_SERIES_RESITOR_OHM	(10000)
static struct adc_sensors_data_linear_uv_to_ohm adc_sensors_batid_data = {
	BATID_SERIES_RESITOR_OHM
};

/**
 * Agold Die Temperature Sensor delivers 1mV / 1K
 */
/* Scaled conversion factor: Scaling Factor / (uV to K factor) */
#define PMIC_SCALING_SHIFT		(20)
#define PMIC_SCALED_CONV_FACTOR		((1 << PMIC_SCALING_SHIFT) / 1000)
/* Offset from Kelvin to Centigrade */
#define PMIC_OFFSET			(-273)

static struct adc_sensors_data_linear_uv_to_degc adc_sensors_agold_die_temp_data
	= {
		PMIC_SCALED_CONV_FACTOR,
		PMIC_SCALING_SHIFT,
		PMIC_OFFSET,
};

/**
 * ACCID(ACD) is measured internally in accessory detection block
 * Refer - AGOLD620_V1.0_BBHW_UM_PR_internal.pdf, page no - 596
 * ACD internal voltage divider is as follows:
 *
 *	ACD (V)
 *
 *	(470k)
 *		---> ACD voltage Input (M4)
 *	(270k)
 *
 *	GND
*/
#define ACCID_UV_TO_MV_FACTOR	(1000)
/* Resistor divider pull-up (Ohms) */
#define ACCID_PULL_UP		(470000)
/* Resistor divider pull-down (Ohms) */
#define ACCID_PULL_DOWN		(270000)
/* Measurement factor's scaling shift */
#define ACCID_SCALING_SHIFT	(20)
/* Scaled inverse conversion factor:
((Pull-up + Pull-down) * Scaling Factor) / (Pull-down * uV_to_mV_factor) */
/* 0.5 is added for rounding off the value with more accracy */
#define ACCID_SCALED_CONV_FACTOR \
	(u32)((((u64)(ACCID_PULL_UP + ACCID_PULL_DOWN) \
		<< (ACCID_SCALING_SHIFT+1)) \
			/ (ACCID_PULL_DOWN * ACCID_UV_TO_MV_FACTOR) + 1) >> 1)

/* ACCID accuracy (+/-) in mV */
#define ACCID_TOLERANCE_MV        (5)

static struct adc_sensors_data_linear_uv_to_mv adc_sensors_accid_data = {
	ACCID_SCALED_CONV_FACTOR,
	ACCID_SCALING_SHIFT,
};

/**
 * ADC Sensor configuration for xmm6321 platform
 */
static struct adc_sensors_channel intel_adc_sensors_channels[] = {
	{"sw_fuel_gauge", ADC_SENSORS_CHANNEL_VBAT,
	ADC_SENSORS_CONVERSION_LINEAR_UV_TO_MV, &adc_sensors_vbat_data,
	CAL_DEFAULT, VBAT_TOLERANCE_MV},

	{"sw_fuel_gauge", ADC_SENSORS_CHANNEL_VBAT_OCV,
	ADC_SENSORS_CONVERSION_LINEAR_UV_TO_MV, &adc_sensors_vbat_data,
	CAL_DEFAULT, VBAT_TOLERANCE_MV},

	{"sw_fuel_gauge", ADC_SENSORS_CHANNEL_BATTEMP0,
	ADC_SENSORS_CONVERSION_TABLE_UV_TO_DEGC, &adc_sensors_tbat_data,
	CAL_DEFAULT, 0},

	{"sw_fuel_gauge", ADC_SENSORS_CHANNEL_BATID,
	ADC_SENSORS_CONVERSION_LINEAR_UV_TO_OHM, &adc_sensors_batid_data,
	CAL_DEFAULT, 0},

	{"agold_meas_thermal", ADC_SENSORS_CHANNEL_PMICTEMP,
	ADC_SENSORS_CONVERSION_LINEAR_UV_TO_DEGC,
	&adc_sensors_agold_die_temp_data, CAL_DEFAULT, 0},

	{"agold_meas_thermal", ADC_SENSORS_CHANNEL_HWID,
	ADC_SENSORS_CONVERSION_LINEAR_UV_TO_OHM, &adc_sensors_batid_data,
	CAL_DEFAULT, 0},

	{"agold_meas_thermal", ADC_SENSORS_CHANNEL_SYSTEMP0,
	ADC_SENSORS_CONVERSION_LINEAR_UV_TO_DEGC,
	&adc_sensors_agold_die_temp_data, CAL_DEFAULT, 0},

	{"sim_charge_pump", ADC_SENSORS_CHANNEL_VBAT,
	ADC_SENSORS_CONVERSION_LINEAR_UV_TO_MV, &adc_sensors_vbat_data,
	CAL_DEFAULT, VBAT_TOLERANCE_MV},

	{"accessory", ADC_SENSORS_CHANNEL_ACCID,
	ADC_SENSORS_CONVERSION_LINEAR_UV_TO_MV, &adc_sensors_accid_data,
	CAL_DEFAULT, ACCID_TOLERANCE_MV}
};

static struct adc_sensors_platform_data xgold_intel_adc_sensors_platform_data
	= {
		.p_channels = intel_adc_sensors_channels,
		.num_channels = sizeof(intel_adc_sensors_channels)
		/ sizeof(struct adc_sensors_channel),
};

#endif /* _XMM6321_INTEL_ADC_SENSORS_H_ */
