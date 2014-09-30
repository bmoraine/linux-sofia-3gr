/*
 * meas_pmic.c - PMIC MEAS HAL.
 *
 * Copyright (C) 2013 Intel Corporation
 *
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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define MEAS_PMIC "pmic_meas"
#define pr_fmt(fmt) MEAS_PMIC": "fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/iio/intel_adc_hal_interface.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <sofia/vmm_pmic.h>
#include "meas_pmic_reg.h"

/* ADC resolution in bits */
#define MEAS_PMIC_ADC_RESOLUTION_BITS			(12)
/* ADC range for gain=1 (0db) in uV */
#define MEAS_PMIC_ADC_RANGE_GAIN_0DB_UV		(1200000)

/* Size of debug data array (has to be power of 2!!!) */
#define MEAS_PMIC_DEBUG_DATA_SIZE (1<<6)

#define SYSFS_INPUT_VAL_LEN (1)

/* Macro to log debug data */
#define MEAS_PMIC_DEBUG_DATA_LOG(_event, _context, _context2, \
						_context3, _context4) \
{ \
	spin_lock(&meas_pmic_debug_data.lock); \
	meas_pmic_debug_data.log_array[meas_pmic_debug_data.index].\
							time_stamp = jiffies; \
	meas_pmic_debug_data.log_array[meas_pmic_debug_data.index].\
							event = (_event); \
	\
	meas_pmic_debug_data.log_array[meas_pmic_debug_data.index].context = \
						(unsigned int)(_context); \
	meas_pmic_debug_data.log_array[meas_pmic_debug_data.index].context2 =\
						(unsigned int)(_context2); \
	meas_pmic_debug_data.log_array[meas_pmic_debug_data.index].context3 =\
						(unsigned int)(_context3); \
	meas_pmic_debug_data.log_array[meas_pmic_debug_data.index].context4 =\
						(unsigned int)(_context4); \
	meas_pmic_debug_data.index++; \
	meas_pmic_debug_data.index &= (MEAS_PMIC_DEBUG_DATA_SIZE-1); \
	spin_unlock(&meas_pmic_debug_data.lock); \
	if (meas_pmic_debug_data.printk_logs_en)\
		pr_debug("%s:%u,%u,%u,%u\n", #_event, \
			(unsigned int)_context, \
			(unsigned int)_context2, \
			(unsigned int)_context3, \
			(unsigned int)_context4); \
}

/**
 * meas_pmic_state - A structure representing the internal meas_pmic_state of
 *			the MEAS HAL.
 *
 * @lock:		Spin lock used to protect critical sections when
 *			modifying device state data.
 * @power_mode:		Power mode of the HW.
 * @suspended:		TRUE=Device is suspended, otherwise not.
 * @active_channel:	Channel currently being measured.
 * @p_channel_data:	Pointer to channel data of each of the supported
 *			channels.
 * @irq_num:		IRQ number for underlying HW. It comes from device data.
 * @meas_done:		Completion used to wait for measurement done.
 * @operation_done_cb:	Pointer to HAL callback.
 */
struct meas_pmic_state_data {
	spinlock_t lock;
	enum adc_hal_power_mode power_mode;
	bool suspended;
	enum adc_channel active_channel;
	struct intel_adc_hal_channel_data
		*p_channel_data[ADC_MAX_NO_OF_CHANNELS];
	struct platform_device *p_platform_device;
	int irq_num;
	struct completion meas_done;
	adc_hal_cb_t operation_done_cb;
	bool meas_pending;
};

static struct meas_pmic_state_data meas_pmic_state = {
	.power_mode = ADC_HAL_POWER_MODE_OFF,
	.suspended = false,
	.active_channel = ADC_MAX_NO_OF_CHANNELS,
	.meas_pending = false,
};

/**
 * meas_pmic_adc_reg - Structure contains the adc register information
 *			for corresponding physical channel.
 *
 * @adc_req_bit:		GPADC Conversion Control Bit request
 * @adc_sta_bit:		GPADC Conversion Control Bit status.
 * @rslt_hsb_addr:		GPADC Conversion Result Register Addr High.
 * @rslt_lsb_addr:	GPADC Conversion Result Register Addr Low
 */

static struct meas_pmic_adc_reg {
	int adc_req_bit;
	int adc_sta_bit;
	int rslt_hsb_addr;
	int rslt_lsb_addr;
} meas_pmic_adc_reg[ADC_PHY_MAX] = {
	{ADC_REQ_VBAT, ADC_STA_VBAT, VBATRSLTH_REG_ADDR, VBATRSLT_REG_ADDR},
	{ADC_REQ_BATID, ADC_STA_BATID,
		 BATTIDRSLTH_REG_ADDR, BATTIDRSLTL_REG_ADDR},
	{ADC_REQ_SYSTEMP, ADC_STA_SYSTEMP,
		 THRMRSLT0H_REG_ADDR, THRMRSLT0L_REG_ADDR},
	{ADC_REQ_SYSTEMP, ADC_STA_SYSTEMP,
		 THRMRSLT1H_REG_ADDR, THRMRSLT1L_REG_ADDR},
	{ADC_REQ_SYSTEMP, ADC_STA_SYSTEMP,
		 THRMRSLT2H_REG_ADDR, THRMRSLT2L_REG_ADDR},
	{ADC_REQ_SYSTEMP, ADC_STA_SYSTEMP,
		 THRMRSLT5H_REG_ADDR, THRMRSLT5L_REG_ADDR},
	{ADC_REQ_BATTEMP, ADC_STA_BATTEMP,
		 THRMRSLT3H_REG_ADDR, THRMRSLT3L_REG_ADDR},
	{ADC_REQ_BATTEMP, ADC_STA_BATTEMP,
		 THRMRSLT4H_REG_ADDR, THRMRSLT4L_REG_ADDR},
	{ADC_REQ_USBID, ADC_STA_USBID,
		 USBIDRSLTH_REG_ADDR, USBIDRSLTL_REG_ADDR},
	{ADC_REQ_PEAK,   ADC_STA_PEAK, PEAKRSLTH_REG_ADDR, PEAKRSLTL_REG_ADDR},
	{ADC_REQ_GPMEAS, ADC_STA_GPMEAS, Y0DATAH_REG_ADDR, Y0DATAL_REG_ADDR},
	{ADC_REQ_GPMEAS, ADC_STA_GPMEAS, Y1DATAH_REG_ADDR, Y1DATAL_REG_ADDR},
	{0, 0, VBATMAXH_REG_ADDR, VBATMAXL_REG_ADDR}
};

/** Enum for driver debug events */
enum meas_pmic_debug_event {
	MEAS_PMIC_INIT,
	MEAS_PMIC_DE_INIT,
	MEAS_PMIC_SUSPEND_OK,
	MEAS_PMIC_SUSPEND_EBUSY,
	MEAS_PMIC_RESUME,
	MEAS_PMIC_MEAS_DONE
};

/**
 * meas_pmic_debug_data - Structure to collect debug data.
 * @index:		Index of log array.
 * @lock:			Spin lock for atomic access.
   @printk_logs_en:	 Flag to enable logs
 * @log_array:		Debug data logging array.
 * @time_stamp:		System time stamp in jiffies.
 * @event:		Event that caused logging.
 * @hmeas:		Measurement handle (if available).
 * @context 1-4:		Event context parameters.
 */
struct meas_pmic_debug_data {
	uint index;
	int printk_logs_en;
	spinlock_t lock;
	struct {
		uint time_stamp;
		enum meas_pmic_debug_event event;
		int context;
		int context2;
		int context3;
		int context4;
	} log_array[MEAS_PMIC_DEBUG_DATA_SIZE];
};

/* Array to collect debug data */
static struct meas_pmic_debug_data meas_pmic_debug_data = {
	.printk_logs_en = 0,
};

/*
 * meas_pmic_current_table lists the current values that are used to the
 * ADC for scaling during the ADC measurement.
 */
static int meas_pmic_current_table[] = {
	0,
	1125,
	2250,
	4500,
	9000,
	18000,
	36000,
	72000,
	144000
};

/**
 * meas_pmic_dump_register() - Dump MEAS registers.
*/
static void meas_pmic_dump_register(void)
{
	/* Start dumping relevant register content */
	/* store first label interrupt and second label interrupt */
}

/**
 * meas_pmic_reg_write() - MEAS PMIC register write API.
 *  It is used to write the value into PMIC HW.
 * @addr			[in] Register address to be written
 * @val				[in] Value to be written in addr
 * @return		[out] return 0 for success and negative for errors.
 */
static int32_t meas_pmic_reg_write(uint32_t addr, uint32_t val)
{
	int32_t result = 0;
	int32_t count = 0;
	do {
		count++;
		result = vmm_pmic_reg_write(addr, val);
		} while ((result < 0) && (count <= 3));
	/*pr_err("WRITE addr = 0x%08x, val=0x%08x, count = %d\n",
		 addr, val, count);*/
	return result;
}

/**
 * meas_pmic_reg_read() - MEAS PMIC register read API.
 *  It is used to read the value into PMIC HW.
 * @addr			[in] Register address to be read
 * @val				[in] Value to be written in addr
 * @return		[out] return 0 for success and negative for errors.
 */
static int32_t meas_pmic_reg_read(uint32_t addr, uint32_t *val)
{
	int32_t result = 0;
	int32_t count = 0;
	do {
		count++;
		result = vmm_pmic_reg_read(addr, val);
		} while ((result < 0) && (count <= 3));
	/*pr_err("READ addr = 0x%08x, val=0x%08x count = %d\n",
		 addr, *val, count);*/
	return result;
}

/**
 * meas_pmic_intern_temperature_calc() - Translates the internal die
 * temperature from uV to milli degs Kelvin.
 *
 * @meas_val			[in] Measured value to convert.
 */
static int meas_pmic_intern_temperature_calc(int meas_result_uv)
{
	/* Calculate and report temperature in unit mV/K
	* On the PMIC the temperature in Deg C is calculated from the ADC
	* measurement (ADCout) as follows:
	*      T = ADCout * 0.24841 - 273.15 oC
	* The conversion from Kelvin to degC is done in the Logical PMICTEMP
	* sensor.
	* The value to be given back to the logical layer is therefore:
	* (ADCout * 0.24841)
	* The measurement is done with Gain 1 implying the full range
	* of the ADC, which is 1200000.The ADC measurement performed by the HAL
	* is in uV and therefore the above has to be converted to mV as follows:
	* (meas_result_uv * 0.24841 * 4096/1200000)
	* Additionally, the logical PMICTEMP sensor expects to get back the
	* measured temperature in units 1mv/K.This is incorporated using:
	* ((meas_result_uv *0.24841*4096/1200000) *1000uV/1K).
	* This results in (meas_result_uv)* 106 / 125 */
	return ((meas_result_uv) * 106) / 125;
}

#ifdef PMIC_IRQ_WORKING
/**
 * meas_pmic_irq_enable() - Function for enabling the MEAS irq.
 * @enable:	True if the irq is to be enabled, false for disabled.
 * @reg_channel: Physical channel of ADC
 */
static void meas_pmic_irq_enable(bool enable, int reg_channel)
{
	int32_t vmm_pmic_err = 0;
	uint32_t madcirq_val = 0, irqlvl1_val = 0;
	uint32_t read_val = 0;

	if (enable) {
		/* Better to have interrupt lock for reg, modify and write */
		vmm_pmic_err =
		  meas_pmic_reg_read(MADCIRQ_REG_ADDR, &madcirq_val);
		if (IS_ERR_VALUE(vmm_pmic_err)) {
			pr_err("%s %d VMM PMIC read error %d\n", __func__,
				__LINE__, vmm_pmic_err);
		}

		read_val = madcirq_val;
		madcirq_val = madcirq_val &
		 (~(1 << (meas_pmic_adc_reg[reg_channel].adc_sta_bit)));
		pr_err("%s MADCIRQ is changed from %d to %d\n",
			 __func__, read_val, madcirq_val);
		vmm_pmic_err =
			 meas_pmic_reg_write(MADCIRQ_REG_ADDR, madcirq_val);
		if (IS_ERR_VALUE(vmm_pmic_err)) {
			pr_err("%s %d VMM PMIC write error %d\n", __func__,
				__LINE__, vmm_pmic_err);
		}

		irqlvl1_val = (1 << IRQLVL1_REG_ADC_BIT_POS);
		pr_err("%s IRQLVL1 is set to %d\n", __func__, irqlvl1_val);
		vmm_pmic_err =
			 meas_pmic_reg_write(MIRQLVL1_REG_ADDR, irqlvl1_val);
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC write error %d\n", __func__,
				__LINE__, vmm_pmic_err);

	} else {
		vmm_pmic_err =
			meas_pmic_reg_read(MADCIRQ_REG_ADDR, &madcirq_val);
		if (IS_ERR_VALUE(vmm_pmic_err)) {
			pr_err("%s %d VMM PMIC read error %d\n", __func__,
				__LINE__, vmm_pmic_err);
		}

		read_val = madcirq_val;
		madcirq_val = madcirq_val |
			(1 << (meas_pmic_adc_reg[reg_channel].adc_sta_bit));
		pr_err("%s MADCIRQ is changed from %d to %d\n",
		 __func__, read_val, madcirq_val);
		vmm_pmic_err =
			meas_pmic_reg_write(MADCIRQ_REG_ADDR, madcirq_val);
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC write error %d\n", __func__,
				__LINE__, vmm_pmic_err);
	}
}
#endif

/**
 * meas_pmic_read_raw() - Configure the registers to perform a current
 *				measurement. *
 * @reg_channel:	The register value of the channel selector
 *			(i.e. enum intel_adc_phy_channel).
 * @p_result_uv:		The raw adc value in uV
 * @p_adc_bias_na:	The current source in nA used for measurement
 * @converted:		True=value is converted to uV otherwise is returned
 *			in counts.
 */
static void meas_pmic_read_raw(int reg_channel,
				int  *p_result_uv,
				int  *p_adc_bias_na,
				bool calibrated,
				enum adc_hal_avg_sample_level avg_sample_level,
				bool converted)
{
	int rslt_hsb = 0, rslt_lsb = 0;
	int reg_current = 0;
	unsigned int adc_irq_value = 0;
	int vmm_pmic_err = 0;

	BUG_ON(reg_channel < 0);

	pr_err("reg_channel = %d\n", reg_channel);

	if (ADC_PHY_OCV != reg_channel) {
		/* Enable the MEAS irq so that we're ready to receive
		 it as soon as it's triggered */

#ifdef PMIC_IRQ_WORKING
		meas_pmic_irq_enable(true, reg_channel);
#endif

		vmm_pmic_err = meas_pmic_reg_write(GPADCIRQ_REG_ADDR,
			(1 << (meas_pmic_adc_reg[reg_channel].adc_sta_bit)));
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC write error %d\n",
				 __func__, __LINE__, vmm_pmic_err);

		/* Start measurement */
		vmm_pmic_err = meas_pmic_reg_write(GPADCREQ_REG_ADDR,
			(1 << (meas_pmic_adc_reg[reg_channel].adc_req_bit)));
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC write error %d\n",
				 __func__, __LINE__, vmm_pmic_err);

		meas_pmic_state.meas_pending = true;

#ifdef PMIC_IRQ_WORKING
		/* Wait for measurement to be done  */
		wait_for_completion(&meas_pmic_state.meas_done);
#endif

		do {
			usleep_range(2000, 2500);
			meas_pmic_reg_read(GPADCIRQ_REG_ADDR, &adc_irq_value);
		} while (!(adc_irq_value &
				(1 <<
			 (meas_pmic_adc_reg[reg_channel].adc_sta_bit))));

		meas_pmic_state.meas_pending = false;
	}

	vmm_pmic_err = meas_pmic_reg_read(
		meas_pmic_adc_reg[reg_channel].rslt_lsb_addr, &rslt_lsb);
	if (IS_ERR_VALUE(vmm_pmic_err)) {
		pr_err("%s %d VMM PMIC read error %d\n", __func__,
			__LINE__, vmm_pmic_err);
	}

	vmm_pmic_err = meas_pmic_reg_read(
		meas_pmic_adc_reg[reg_channel].rslt_hsb_addr, &rslt_hsb);
	if (IS_ERR_VALUE(vmm_pmic_err)) {
		pr_err("%s %d VMM PMIC read error %d\n", __func__,
			__LINE__, vmm_pmic_err);
	}

/* According to datasheets: Vin (uV) = (measured Counts)
				* (measurement input range in uV) /
						(ADC resolution) */
	*p_result_uv = (int)((((long long int)
			(((rslt_hsb & 0xF) << 8) + rslt_lsb) *
			MEAS_PMIC_ADC_RANGE_GAIN_0DB_UV) >>
			(MEAS_PMIC_ADC_RESOLUTION_BITS-1)) + 1) >> 1;

	/* Get current bias used in measurement and converted to nA */
	reg_current = (int) ((rslt_hsb >> 4) & 0xF);
	BUG_ON(reg_current >= ARRAY_SIZE(meas_pmic_current_table));
	*p_adc_bias_na = meas_pmic_current_table[reg_current];

	if (converted) {
		/* Apply post processing functions to special channels. */
		switch (reg_channel) {
		case ADC_PHY_PMICTEMP:
			/* The internal die temperature must be converted to uV
			representing Kelvin */
			*p_result_uv =
				meas_pmic_intern_temperature_calc(*p_result_uv);
		default:
			/* Nothing to do here. */
			break;
		}
	}

	MEAS_PMIC_DEBUG_DATA_LOG(MEAS_PMIC_MEAS_DONE, reg_channel,
					*p_result_uv, *p_adc_bias_na, 0);
}

/**
 * meas_pmic_set_power_mode() - Set parameters of the ADC HAL driver.
 * @new_power_mode:			Key to specify parameter(s) to set.
 * @params:		Structure holding list of parameters.
 *			Returns Error number; may be tested as boolean with
 *			0=success, other=fail. Some errors may need re-trial.
 */
static int meas_pmic_set_power_mode(enum adc_hal_power_mode new_power_mode)
{
	int ret = WNOTREQ;

	/* Protect critical section when testing and modifying device
	state data */
	spin_lock(&meas_pmic_state.lock);

	/* Check whether device is suspended. If so, return an error as no
	operation is allowed on a suspended device */
	if (meas_pmic_state.suspended) {
		spin_unlock(&meas_pmic_state.lock);
		return -EIO;
	}

	/* Check whether new power mode requested is different from
	current mode */
	if (meas_pmic_state.power_mode != new_power_mode) {

		/* Update power mode variable */
		meas_pmic_state.power_mode = new_power_mode;

		/* End of critical section */
		spin_unlock(&meas_pmic_state.lock);

		/* Set new power mode */
		switch (new_power_mode) {
		case ADC_HAL_POWER_MODE_OFF:
		case ADC_HAL_POWER_MODE_ON:{
				/* Do nothing */
			}
			break;
		default:
			ret = -EINVAL;
			break;
		};
	} else {
		/* End of critical section */
		spin_unlock(&meas_pmic_state.lock);
		ret = -EINVAL;
	}
	return ret;
}

/**
 * meas_pmic_stop_meas() - Stop ADC measurement.
 * @channel:		ADC Channel to stop measurement on.
 *			Returns Error number; may be tested as boolean with
 *			0=success, other=fail. Some errors may need re-trial.
 */
static int meas_pmic_stop_meas(enum adc_channel channel)
{
	int i;

	/* wait until a measurement pending in meas_pmic_read_raw() */
	for (i = 0; i < 100 && meas_pmic_state.meas_pending; i++) {
		pr_warn("stop measurement on ADC running\n");
		udelay(300);
	}

	return 0;
}

/**
 * meas_pmic_get_meas() - Get measurement value.
 * @channel			ADC channel to measure.
 * @p_adc_bias_na		ADC bias current to set in nA, or 0 for voltage
 *				mode.
 * @p_result_uv			Pointer to measurement result.
 *				Returns Error number; may be tested as boolean
 *				with 0=success, other=fail. Some errors may
 *				need re-trial.
 */
static int meas_pmic_get_meas(enum adc_channel channel,
				int *p_adc_bias_na, int *p_result_uv)
{
	/* Update active channel */
	meas_pmic_state.active_channel = channel;
	/* Get measurement value */
		meas_pmic_read_raw(meas_pmic_state.p_channel_data[channel]->
				phy_channel_num, p_result_uv, p_adc_bias_na,
				true,
				meas_pmic_state.p_channel_data[channel]->
				average_sample, true);
	/* Clear active channel */
	meas_pmic_state.active_channel = ADC_MAX_NO_OF_CHANNELS;
	return 0;
}

/**
 * meas_pmic_set() - Set parameters of the ADC HAL driver.
 * @key:		Key to specify parameter(s) to set.
 * @params:		Structure holding list of parameters.
 *			Returns Error number; may be tested as boolean with
 *			0=success, other=fail. Some errors may need re-trial.
 */
static int meas_pmic_set(enum adc_hal_set_key key,
				union adc_hal_set_params params)
{
	int ret = 0;

	switch (key) {
	case ADC_HAL_SET_POWER_MODE:
		ret = meas_pmic_set_power_mode(params.power_mode);
		break;
	case ADC_HAL_SET_UP_MEAS:
		ret = WNOTREQ;
		break;
	case ADC_HAL_STOP_MEAS:
		ret = meas_pmic_stop_meas(params.channel);
		break;
	case ADC_HAL_DUMP_REGISTER:
		meas_pmic_dump_register();
	default:
		ret = -EINVAL;
		break;
	};
	return ret;
}

/**
 * meas_pmic_get() - Get parameters of the ADC HAL driver.
 * @key:		Key to specify parameter(s) to get.
 * @params:		Structure holding list of parameters.
 *			Returns Error number; may be tested as boolean with
 *			0=success, other=fail. Some errors may need re-trial.
 */
static int meas_pmic_get(enum adc_hal_get_key key,
				union adc_hal_get_params params)
{
	int ret = 0;
	switch (key) {
	case ADC_HAL_GET_POWER_MODE:
		*params.p_power_mode = meas_pmic_state.power_mode;
		break;
	case ADC_HAL_GET_ADC_MEAS:{
			/* Get measurement and copy value */
			ret = meas_pmic_get_meas(params.adc_meas.channel,
						params.adc_meas.p_adc_bias_na,
						 params.adc_meas.p_result_uv);
		}
		break;
	default:
		ret = -EINVAL;
		break;
	};
	return ret;
}

#ifdef PMIC_IRQ_WORKING
/**
 * meas_pmic_irq_handler - MEAS HW irq handler. It releases the "wait for
 *			completion" to process measurement done.
 * @irq:		irq number, should match IRQ number from device data.
 * @data:		data passed to the irq.
 */
static irqreturn_t meas_pmic_irq_handler(int irq, void *data)
{
	struct meas_pmic_state_data *st = (struct meas_pmic_state_data *)data;
	unsigned int adc_irq_value = 0;
	enum adc_channel channel = meas_pmic_state.active_channel;
	int reg_channel =
		 meas_pmic_state.p_channel_data[channel]->phy_channel_num;
	uint32_t irqlvl1_value = 0;

	pr_err("channel = %d, reg_channel=%d\n", channel, reg_channel);

	BUG_ON(st->irq_num != irq);

	meas_pmic_reg_read(IRQLVL1_REG_ADDR, &irqlvl1_value);
	if (!(irqlvl1_value & (1 << IRQLVL1_REG_ADC_BIT_POS)))
		return IRQ_NONE;

	meas_pmic_reg_read(GPADCIRQ_REG_ADDR, &adc_irq_value);

	pr_err("adc_irq_value = %d\n", adc_irq_value);

	if (!(adc_irq_value &
			(1 << (meas_pmic_adc_reg[reg_channel].adc_sta_bit))))
		return IRQ_NONE;

	/* First of all, clear the IRQ bit to avoid other interrupts */
	meas_pmic_irq_enable(false, reg_channel);

	/* Signal to release measurement done completion */
	complete(&st->meas_done);

	return IRQ_HANDLED;
}
#endif

static struct adc_hal_hw_info meas_pmic_hw_info = {
	.p_current_scaling_info = NULL,
	.p_hw_name = "PMIC MEAS HAL",
};

/* The final structure to register into the ADC layer */
static struct adc_hal_interface meas_pmic_adc_hal_interface = {
	.set = meas_pmic_set,
	.get = meas_pmic_get,
	.hw_info = &meas_pmic_hw_info,
};

static struct intel_adc_hal_channel_data channel_data[] = {
	{"intel_adc_sensors", "VBAT_ADC", "CH0_TYP", ADC_V_BAT_TYP,
	 ADC_PHY_VBAT, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM, IIO_VOLTAGE},

	{"intel_adc_sensors", "VBAT_MIN_ADC", "CH0_MIN", ADC_V_BAT_MIN,
	 ADC_PHY_VBAT, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM, IIO_VOLTAGE},

	{"intel_adc_sensors", "VBAT_OCV_ADC", "CH12_OCV", ADC_V_BAT_OCV,
	 ADC_PHY_OCV, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_HIGH, IIO_VOLTAGE},

	{"intel_adc_sensors", "BATTEMP0_ADC", "CH06", ADC_T_BAT_0,
	 ADC_PHY_BAT0TEMP, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM, IIO_TEMP},

	{"intel_adc_sensors", "BATID_ADC", "CH01", ADC_ID_BAT,
	 ADC_PHY_BATID, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM, IIO_RESISTANCE},

	{"intel_adc_sensors", "PMICTEMP_ADC", "CH05", ADC_T_PMIC_IC_0,
	 ADC_PHY_PMICTEMP, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM, IIO_TEMP},

	{"intel_adc_sensors", "ANAMON_ADC", "CH10", ADC_ANAMON, ADC_PHY_AGND,
	 false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM, IIO_VOLTAGE},

	{"intel_adc_sensors", "SYSTEMP0_ADC", "CH02", ADC_T_SYS_0,
	 ADC_PHY_SYS0TEMP, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM, IIO_TEMP},

	{"intel_adc_sensors", "SYSTEMP1_ADC", "CH03", ADC_T_SYS_1,
	 ADC_PHY_SYS1TEMP, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM, IIO_TEMP},

	{"intel_adc_sensors", "SYSTEMP2_ADC", "CH04", ADC_T_SYS_2,
	 ADC_PHY_SYS2TEMP, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM, IIO_TEMP},
};

static ssize_t dbg_logs_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	size_t size_copied;
	int value;

	value = meas_pmic_debug_data.printk_logs_en;
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

	size_to_cpy =
		(count > SYSFS_INPUT_VAL_LEN) ? SYSFS_INPUT_VAL_LEN : count;
	strncpy(strvalue, buf, size_to_cpy);
	strvalue[size_to_cpy] = '\0';

	ret = kstrtoint(strvalue, 10, &sysfs_val);
	pr_err("%s ret=%d\n", __func__, ret);
	if (ret != 0)
		return ret;

	sysfs_val = (sysfs_val == 0) ? 0 : 1;

	meas_pmic_debug_data.printk_logs_en = sysfs_val;

	pr_err("sysfs attr %s=%d\n", attr->attr.name, sysfs_val);

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
 * meas_pmic_setup_sysfs_attr	Sets up dbg_logs_on_off sysfs entry
 *				for MEAS idi device
 *
 * @dev				[in] pointer to device structure structure
 */
static void meas_pmic_setup_sysfs_attr(struct device *dev)
{
	int err;

	err = device_create_file(dev, &dbg_logs_on_off_attr);
	if (err)
		pr_err("Unable to create sysfs entry: '%s'\n",
			dbg_logs_on_off_attr.attr.name);
}

/**
 * meas_pmic_probe() - The function that starts it all: it maps the registers,
 * initializes the hardware, request interrupts and registers to its logical
 * layer.
 *
 * @ididev:		A pointer to the IDI device.
 * @id:			A handle to the device id.
 */
static int meas_pmic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct intel_adc_hal_pdata *pdata;
	struct adc_hal_channel_info *p_channel_info;
	int ret, chan;

	meas_pmic_state.p_platform_device = pdev;

	pdata = kzalloc(sizeof(struct intel_adc_hal_pdata), GFP_KERNEL);

	pdata->channel_info.nchan = ARRAY_SIZE(channel_data);
	pdata->channel_info.p_data = channel_data;
	p_channel_info = (struct adc_hal_channel_info *) &pdata->channel_info;
	dev->platform_data = pdata;

	/* Store device data in a convenient form for later use */
	for (chan = 0; chan < p_channel_info->nchan; chan++) {
		meas_pmic_state.p_channel_data[p_channel_info->p_data[chan].
						log_channel_id] =
			&p_channel_info->p_data[chan];
	}

#ifdef PMIC_IRQ_WORKING
	meas_pmic_state.irq_num = platform_get_irq_byname(pdev,
					"PMIC_ADC_HIRQ");

	if (IS_ERR_VALUE(meas_pmic_state.irq_num)) {
		pr_err("(%s) failed to get irq no\n", __func__);
		return -ENXIO;
	}

	/* Request and enable the MEAS irq */
	ret = request_irq(meas_pmic_state.irq_num, meas_pmic_irq_handler,
			IRQF_TRIGGER_RISING, "pmic_adc_irq", &meas_pmic_state);

	if (IS_ERR_VALUE(ret)) {
		pr_err("(%s) failed requesting interrupt\n", __func__);
		return -EINVAL;
	}
#endif

	/* Initialise measurement done completion */
	init_completion(&meas_pmic_state.meas_done);

	/* Register to HAL and return */
	ret = adc_register_hal(&meas_pmic_adc_hal_interface, dev,
				&meas_pmic_state.operation_done_cb);
	if (ret)
		goto err_register_hal;

	meas_pmic_setup_sysfs_attr(dev);

	pr_err("(%s) probe OK\n", __func__);

	return 0;

err_register_hal:
	free_irq(meas_pmic_state.irq_num, &meas_pmic_state);
	pr_info("probe failed\n");
	return ret;
}

/**
 * meas_pmic_remove() - The function that stops the device/driver, de-registers
 * from its logical layer and releases all associated resources
 *
 * @ididev:	A pointer to the IDI device.
 */
static int meas_pmic_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	adc_unregister_hal(&meas_pmic_adc_hal_interface, dev);

	return 0;
}

/**
 * meas_pmic_suspend() - Called when the system is attempting to suspend.
 * If a measurement is in progress EBUSY is returned to abort the suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	EBUSY if a measurement is ongoing, else 0
 */
static int meas_pmic_suspend(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Protect critical section when testing and modifying device state
	data */
	spin_lock(&meas_pmic_state.lock);

	/* If there is a a measurement in progess, prevent suspend. */
	if (ADC_HAL_POWER_MODE_ON == meas_pmic_state.power_mode) {

		/* End of critical section */
		spin_unlock(&meas_pmic_state.lock);

		MEAS_PMIC_DEBUG_DATA_LOG(MEAS_PMIC_SUSPEND_EBUSY, 0, 0, 0, 0);
		return -EBUSY;
	} else {
		/* No measurement ongoing - allow suspend. */
		meas_pmic_state.suspended = true;

		/* End of critical section */
		spin_unlock(&meas_pmic_state.lock);

		MEAS_PMIC_DEBUG_DATA_LOG(MEAS_PMIC_SUSPEND_OK, 0, 0, 0, 0);
		return 0;
	}
}

/**
 * meas_pmic_resume() - Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int meas_pmic_resume(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Nothing to do here */
	MEAS_PMIC_DEBUG_DATA_LOG(MEAS_PMIC_RESUME, 0, 0, 0, 0);

	/* Update suspend flag used to tell whether operations on device are
	allowed */
	meas_pmic_state.suspended = false;
	return 0;
}

/**
 * meas_pmic_pm - MEAS PMIC power management interface.
 */
const struct dev_pm_ops meas_pmic_pm = {
	.suspend = meas_pmic_suspend,
	.resume_early = meas_pmic_resume,
};

static const struct of_device_id meas_pmic_of_match[] = {
	{
		.compatible = "intel,meas_pmic",
	},
	{},
};

/**
 * meas_pmic_pfdrv - Platform driver structure for MEAS PMIC.
 */
static struct platform_driver meas_pmic_pfdrv = {
	.driver = {
		.owner = THIS_MODULE,
		.name = MEAS_PMIC,
		.pm = &meas_pmic_pm,
		.of_match_table = of_match_ptr(meas_pmic_of_match),
	},
	.probe = meas_pmic_probe,
	.remove = meas_pmic_remove,
};

/**
 * meas_pmic_init - MEAS PMIC HAL device init function.
 * returns		0 for success, or error code.
 */
static int __init meas_pmic_init(void)
{
	int ret;
	spin_lock_init(&meas_pmic_debug_data.lock);
	spin_lock_init(&meas_pmic_state.lock);
	ret = platform_driver_register(&meas_pmic_pfdrv);
	/* Do debug data logging */
	MEAS_PMIC_DEBUG_DATA_LOG(MEAS_PMIC_INIT, ret, 0, 0, 0);
	return ret;
}

/**
 * meas_pmic_exit - MEAS PMIC HAL device deinit function.
 */
static void __exit meas_pmic_exit(void)
{
	platform_driver_unregister(&meas_pmic_pfdrv);
	/* Do debug data logging */
	MEAS_PMIC_DEBUG_DATA_LOG(MEAS_PMIC_DE_INIT, 0, 0, 0, 0);
}

module_init(meas_pmic_init);
module_exit(meas_pmic_exit);
MODULE_DEVICE_TABLE(of, meas_pmic_of_match);

MODULE_LICENSE("GPL v2");
