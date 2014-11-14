/*
 * -------------------------------------------------------------------------
 * Copyright (C) 2013 Intel Mobile Communications GmbH
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
 */

#ifndef _ADC_HAL_INTERFACE_H
#define _ADC_HAL_INTERFACE_H

/*---------------------------- Includes: ---------------------------------*/

#include "linux/types.h"
#include <linux/device.h>
#include <linux/iio/types.h>
#include <linux/ctype.h>


/*---------------------------- Defines: ----------------------------------*/

/* Maximum signal settling time set to zero to indicate that settling is
disabled */
#define ADC_HAL_SIGNAL_SETTLING_DISABLED  0

/* Warning definition for this interface  */
#define	WNOTREQ		2000	/* Not required action. This is used to indicate
				that some action is not require on the given
				HAL */


/*---------------------------- Typedefs: ---------------------------------*/

/* Possible supported ADC channels. */
enum adc_channel {
	ADC_V_BAT_TYP = 0,
	ADC_V_BAT_MIN,
	ADC_V_BAT_OCV,
	ADC_V_CHR_NOM,
	ADC_V_CHR_USB,
	ADC_I_BAT,
	ADC_I_CHR,
	ADC_I_HW,
	ADC_T_BAT_0,
	ADC_T_BAT_1,
	ADC_T_DBB_IC_0,
	ADC_T_DBB_IC_1,
	ADC_T_PMIC_IC_0,
	ADC_T_PMIC_IC_1,
	ADC_T_RF,
	ADC_T_PCB,
	ADC_T_REF,
	ADC_T_SYS_0,
	ADC_T_SYS_1,
	ADC_T_SYS_2,
	ADC_T_CHR_IC,
	ADC_ID_BAT,
	ADC_ID_PCB,
	ADC_ID_ACC,
	ADC_ANAMON,
	ADC_ID_USB,
	ADC_MAX_NO_OF_CHANNELS
};


/*
 * Level indicating the number of averaging samples.
 * LOW =    Least number of averaging samples supported by the platform.
 * MEDIUM = Medium number of average samples supported by the platform.
 * HIGH =   Highest number of averaging samples supported by the platform.
 */
enum adc_hal_avg_sample_level {
	ADC_HAL_AVG_SAMPLE_LEVEL_LOW,
	ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM,
	ADC_HAL_AVG_SAMPLE_LEVEL_HIGH
};

/**
 * intel_adc_hal_channel_data - ADC channel specific configuration data.
 *
 * @consumer_dev_name		String that describes the measurement consumer
 *				device.
 * @consumer_channel		String that describes the measurement channel as
 *				referred by the consumer.
 * @datasheet_name		String that describes where the signal is
 *				connected on the HW, i.e.
 *				HW/ChipName_ChannelName.
 * @log_channel_id		Unique number that identifies the logical
 *				channel.
 * @phy_channel_num		Number for the physical channel that relates to
 *				the logical channel.
 * @autoscaling_on		Flag to enable autoscaling feature on the
 *				specific ADC channel.
 * @adc_bias_na			Channel current bias in nA, where zero means no
 *				current bias. If autoscaling is enabled, this is
 *				the initial and the lowest current bias value
 *				used.
 * @max_signal_settling_time_ms	Maximum time in ms to allow measurement signal
 *				to settle. Zero if settling is disabled.
 * @average_sample		Level indicating the number of averaging samples
 *				required for the channel.
 */
struct intel_adc_hal_channel_data {
	const char			*consumer_dev_name;
	const char			*consumer_channel;
	const char			*datasheet_name;
	enum adc_channel		log_channel_id;
	int				phy_channel_num;
	bool				autoscaling_on;
	int				adc_bias_na;
	uint				max_signal_settling_time_ms;
	enum adc_hal_avg_sample_level	average_sample;
	enum iio_chan_type		iio_type;
};

/**
 * adc_hal_channel_info -	Supported ADC channel information.
 * @nchan		Number of supported channels.
 * @p_data		List of supported channels with their settings.
 */
struct adc_hal_channel_info {
	unsigned int				nchan;
	struct intel_adc_hal_channel_data	*p_data;
};

/**
 * Platform data to be passed to the HAL device.
 * @p_adc_iio_dev:	Pointer to IIO device structure.
 * @p_iio_map:		Pointer to IIO mapping table.
 * @p_adc_chan_spec:	Pointer IIO Channel specification.
 * @calibration_period	Time in seconds between calibration measurement cycles
 * @settling_interval_ms	Time in milli seconds between samples while
 *				polling for signal settling.
 * @settling_accuracy_uv	Max voltage difference  between samples to
 *				declare a signal settled.
 * @channel_info		Supported channels information.
 */
struct intel_adc_hal_pdata {
	struct iio_dev				*p_adc_iio_dev;
	struct iio_map				*p_iio_map;
	struct iio_chan_spec			*p_adc_chan_spec;
	int					calibration_period_s;
	int					settling_interval_ms;
	int					settling_accuracy_uv;
	struct adc_hal_channel_info		channel_info;
};



/* Enum type for set function keys */
enum adc_hal_set_key {
	/* Key to set the ADC power mode to be used.The parameters for ::set
	are then defined as:
	::union adc_hal_set_params.power_mode - ADC power mode. */
	ADC_HAL_SET_POWER_MODE,
	/* Key to set up the ADC measurement.The parameters for ::set are then
	defined as:
	::union adc_hal_set_params.channel - ADC channel. */
	ADC_HAL_SET_UP_MEAS,
	/* Key to stop the ADC measurement.The parameters for ::set are then
	defined as:
	::union adc_hal_set_params.channel - ADC channel. */
	ADC_HAL_STOP_MEAS,
	/* Key to dump the the HW registers. There are no parameters */
	ADC_HAL_DUMP_REGISTER

};

/* Power modes. */
enum adc_hal_power_mode {
	ADC_HAL_POWER_MODE_OFF,
	ADC_HAL_POWER_MODE_ON
};

/* Union type for set function parameters. */
union adc_hal_set_params {
	enum adc_hal_power_mode power_mode;
	enum adc_channel	channel;
};

/* Enum type for get function keys */
enum adc_hal_get_key {
	/* Key to get the ADC power mode to be used. The parameters for ::get
	are then defined as:
	::union adc_hal_get_params.p_power_mode - ADC power mode. */
	ADC_HAL_GET_POWER_MODE,

	/* Key to get the an ADC measurement result. The parameters for ::get
	are then defined as:
	::union adc_hal_get_params.adc_meas  - ADC measurement. */
	ADC_HAL_GET_ADC_MEAS
};

/**
 * adc_hal_meas -ADC measurement data structure.
 * @channel		ADC channel to measure.
 * @p_adc_bias_na	ADC bias current to set in nA, or 0 for voltage mode.
 *			It returns the current actually used by the HW.
 * @p_result_uv		Pointer to measurement result if available synchronously
 */
struct adc_hal_meas {
	enum adc_channel	channel;
	int			*p_adc_bias_na;
	int			*p_result_uv;
};

/**
 * union adc_hal_get_params -Union type for get function parameters.
 * @p_power_mode	Pointer to get current HAL power mode.
 * @adc_meas		ADC measurement information.
 */
union adc_hal_get_params {
	enum adc_hal_power_mode *p_power_mode;
	struct adc_hal_meas	adc_meas;
};


/* Platform current scaling information. */
struct adc_hal_current_scaling_info {
	int high_threshold_uv;
	int low_threshold_uv;
	int *p_current_table;
	unsigned int table_size;
};

/**
 * union adc_hal_hw_info - ADC Hal capabilities to be passed to the ADC Driver
 * at registration time.
 *
 * @p_current_scaling_info	Pointer to the current scaling information.
 * @p_hw_name			Pointer to a NULL terminated string describing
 *				the HW.
 */
struct adc_hal_hw_info {
	struct adc_hal_current_scaling_info *p_current_scaling_info;
	char const *p_hw_name;
};

/**
 * struct adc_hal_interface	ADC HAL exported interface functions structure.
 * @set                         Set parameters of the ADC HAL driver.
 *				'key'[in] - Key to specify parameter(s) to set.
 *				'params'[in] - structure holding list of
 *				parameters.
 *				Returns Error number; may be tested as boolean
 *				with 0=success, other=fail. Some errors may
 *				need re-trial.
 *
 * @get				Get parameters of the ADC HAL driver.
 *				'key'[in] - Key to specify parameter(s) to get.
 *				'params'[in] - structure holding list of
 *				parameters.
 *				Returns Error number; may be tested as boolean
 *				with 0=success, ENODATA=data not available for
 *				immediate fetch, but it will reported via
 *				callback if applicable, other=fail.
 *				Some errors may need re-trial.
 *
 * @hw_info			HAL Identification information. Describes HW
 *				supported.
 */
struct adc_hal_interface {
	int (*set)(enum adc_hal_set_key key, union adc_hal_set_params params);
	int (*get)(enum adc_hal_get_key key, union adc_hal_get_params params);

	struct adc_hal_hw_info *hw_info;
};

/* Parameters passed by callback events. */
union adc_hal_cb_param {
	enum adc_hal_power_mode power_mode;
	int adc_result;
};

/* Possible callback events. */
enum adc_hal_cb_event {
	ADC_HAL_CB_EVENT_POWER_UP_DONE,		/* callback type for power up
						done indication. */
	ADC_HAL_CB_EVENT_MEAS_SET_UP_DONE,	/* callback type for set up done
						indication. */
	ADC_HAL_CB_EVENT_MEAS_DONE		/* callback type for conversion
						done indication. */
};

/**
 * ADC Hal interface callback function type.
 * @adc_hal_cb_event		[in] Event which has occured.
 * @adc_hal_cb_param		[in] Parameter value for the event.
 */
typedef void (*adc_hal_cb_t)(enum adc_hal_cb_event adc_hal_cb_event,
				union adc_hal_cb_param adc_hal_cb_param);


/**
 * adc_register_hal() - Register the supported operations and capabilities of a
 *				HAL with the ADC Driver.
 * @p_adc_hal_interface		[in] Funtions, channels and HW specific
 *				information.
 * @pdata			[in] HAL platform data.
 * @p_adc_driver_cb_function	[out] Callback function in the ADC Driver.
 *
 * Returns Error number; may be tested as boolean with 0=success, other=fail.
 * Some errors may need re-trial.
 */
int adc_register_hal(struct adc_hal_interface *p_adc_hal_interface,
			struct device *hal_device,
			adc_hal_cb_t *p_adc_driver_cb_function);

/**
 * adc_unregister_hal() - Deregister the supported operations and capabilities
 *				of a HAL with the ADC Driver.
 *
 * @p_adc_hal_interface		[in] Funtions, channels and HW specific
				information.
 * @pdata			[in] HAL platform data.
 *
 * Returns Error number; may be tested as boolean with 0=success, other=fail.
 * Some errors may need re-trial.
 */
void adc_unregister_hal(struct adc_hal_interface *p_adc_hal_interface,
			struct device *hal_device);


#endif /* _ADC_HAL_INTERFACE_H */
