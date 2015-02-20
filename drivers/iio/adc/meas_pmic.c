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
#include <sofia/vmm_pmic-ext.h>
#include "meas_pmic_reg.h"

/* ADC resolution in bits */
#define MEAS_PMIC_ADC_RESOLUTION_BITS			(12)
/* ADC range for gain=1 (0db) in uV */
#define MEAS_PMIC_ADC_RANGE_GAIN_0DB_UV		(1200000)
/* ADC saturation level */
#define MEAS_PMIC_ADC_SATURATION_LEVEL_UV	(1199000)
/* PMIC NVM version which works for BATT TEMP and SYS TEMP ADC
	meausrement without any change */
#define MEAS_PMIC_TLP2_SEQ_FIX_NVM_VERSION (0x4)
/* BATT TEMP ADC Auto current Scaling mode for measurement */
#define MEAS_PMIC_BATT_TEMP_SCALING_MODE (3)
/* SYS TEMP ADC Auto current Scaling mode for measurement */
#define MEAS_PMIC_SYS_TEMP_SCALING_MODE (2)

/* Size of debug data array (has to be power of 2!!!) */
#define MEAS_PMIC_DEBUG_DATA_SIZE (1<<6)

/* Max retry count for VMM read/write access */
#define VMM_RW_MAX_RETRY (3)

/* Max revision of PMIC chip */
#define PMIC_CHIP_REV_MAX  (8)

/* PMIC NVM size in byte */
#define PMIC_NVM_SIZE  (2048)
/* Gain scaling factor */
#define PMIC_GEC_SCALING (1024)
/* Production header ID */
#define PMIC_PRODUCT_SEC_ID (0x4)
/* Mask value to extract production header from NVM header */
#define PMIC_PRODUCT_SEC_ID_MASK (0x7C)
/* Subsection id for Fuel gauge data inside production header */
#define PMIC_FG_SUBSEC_ID (0x80)
/* Mask value to extract header length from NVM header */
#define PMIC_SEC_LENGTH_MASK (0x1F)
/* Byte position of Fuel gauge calibration voltage and its result */
#define FG_VHH_BYTE_POS (1)
#define FG_VHL_BYTE_POS (2)
#define FG_VLH_BYTE_POS (3)
#define FG_VLL_BYTE_POS (4)
#define FG_RDHH_BYTE_POS (9)
#define FG_RDHL_BYTE_POS (10)
#define FG_RDLH_BYTE_POS (11)
#define FG_RDLL_BYTE_POS (12)
/* Conversion factor from volt to milli volt */
#define PMIC_V_TO_MV_FACTOR  (1000)
/* Resistor divider pull-up (Ohms) */
#define PMIC_VBAT_PULL_UP_RES_MOHM (78400)
/* Resistor divider pull-down (Ohms) */
#define PMIC_VBAT_PULL_DOWN_RES_MOHM (24000)
/* Conversion factor in MV */
#define PMIC_VBATRATIO_IN_MV (((PMIC_VBAT_PULL_UP_RES_MOHM + \
	PMIC_VBAT_PULL_DOWN_RES_MOHM) * PMIC_V_TO_MV_FACTOR)/ \
	PMIC_VBAT_PULL_DOWN_RES_MOHM)
/* Max retry count to validate NVM read data */
#define PMIC_NVM_READ_MAX_RETRY_CNT (100)

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
	pr_debug("%s:%u,%u,%u,%u\n", #_event, \
			(unsigned int)_context, \
			(unsigned int)_context2, \
			(unsigned int)_context3, \
			(unsigned int)_context4); \
}

/**
 * meas_pmic_vbat_cal_data - Contains vbat calibration data
 * @gain: Gain Error Correction
 * @offset_uv: Offset Error Correction
 */
struct meas_pmic_vbat_cal_data {
	int gain;
	int offset_uv;
};

/**
 * meas_adc_irq_info - Contains PMIC adc irq number and irq name
 * @irq_num: irq number
 * @irq_name: irq_name
 */
struct meas_adc_irq_info {
	int irq_num;
	char *irq_name;
};

/**
 * meas_pmic_state - A structure representing the internal meas_pmic_state of
 *			the MEAS HAL.
 *
 * @otp_version:	NVM version of PMIC HW.
 * @lock:		Spin lock used to protect critical sections when
 *			modifying device state data.
 * @power_mode:		Power mode of the HW.
 * @suspended:		true=Device is suspended, otherwise not.
 * @active_channel:	Channel currently being measured.
 * @p_channel_data:	Pointer to channel data of each of the supported
 *			channels.
 * @vbat_cal:	 Structure for vbat calibration data
 * @p_platform_device:	Pointer to platform device
 * @adc_irq_info:	Contains ADC IRQ related information.
 * @meas_done:		Completion used to wait for measurement done.
 * @operation_done_cb:	Pointer to HAL callback.
 * @meas_pending:	measurement pending status.
 */
struct meas_pmic_state_data {
	int otp_version;
	spinlock_t lock;
	enum adc_hal_power_mode power_mode;
	bool suspended;
	enum adc_channel active_channel;
	struct intel_adc_hal_channel_data
		*p_channel_data[ADC_MAX_NO_OF_CHANNELS];
	struct meas_pmic_vbat_cal_data vbat_cal;
	struct platform_device *p_platform_device;
	struct meas_adc_irq_info adc_irq_info[ADC_STA_IRQ_MAX];
	struct completion meas_done;
	adc_hal_cb_t operation_done_cb;
	bool meas_pending;
	int acd_dummy_measurement;
};

static struct meas_pmic_state_data meas_pmic_state = {
	.power_mode = ADC_HAL_POWER_MODE_OFF,
	.suspended = false,
	.active_channel = ADC_MAX_NO_OF_CHANNELS,
	.meas_pending = false,
	.adc_irq_info =  {
		{0, "ADC_USBID_IRQ"},
		{0, NULL},
		{0, "ADC_BATTEMP_IRQ"},
		{0, "ADC_SYSTEMP_IRQ"},
		{0, "ADC_BATTID_IRQ"},
		{0, "ADC_VBATT_IRQ"},
		{0, "ADC_GPMEAS_IRQ"},
		{0, NULL},
	}
};

/* PMIC CHIP revision array */
static struct meas_pmic_chip_rev {
	char *str;
} meas_pmic_chip_rev[PMIC_CHIP_REV_MAX] = {
	{"A"},
	{"B"},
	{"C"},
	{"D"},
	{"E"},
	{"F"},
	{"G"},
	{"H"},
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
	{ADC_REQ_USBID, ADC_STA_USBID,
		ACDRSLTH_REG_ADDR, ACDRSLTL_REG_ADDR},
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
 * @log_array:		Debug data logging array.
 * @time_stamp:		System time stamp in jiffies.
 * @event:		Event that caused logging.
 * @hmeas:		Measurement handle (if available).
 * @context 1-4:		Event context parameters.
 */
struct meas_pmic_debug_data {
	uint index;
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
static struct meas_pmic_debug_data meas_pmic_debug_data;

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
		} while ((result < 0) && (count <= VMM_RW_MAX_RETRY));
	/*pr_err("WRITE addr = 0x%08x, val=0x%08x, count = %d\n",
		 addr, val, count);*/
	if (VMM_RW_MAX_RETRY == count) {
		pr_err("%s VMM PMIC register write failed !!!!\n", __func__);
		BUG();
	}
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
		} while ((result < 0) && (count <= VMM_RW_MAX_RETRY));
	/*pr_err("READ addr = 0x%08x, val=0x%08x count = %d\n",
		 addr, *val, count);*/
	if (VMM_RW_MAX_RETRY == count) {
		pr_err("%s VMM PMIC register read failed !!!!\n", __func__);
		BUG();
	}
	return result;
}


/**
 * meas_pmic_dump_register() - Dump PMIC registers.
*/
static void meas_pmic_dump_register(void)
{
/* This can be used to dump debug register in future.*/
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


/**
 * meas_pmic_irq_enable() - Function for enabling the MEAS irq.
 * @enable:	True if the irq is to be enabled, false for disabled.
 * @reg_channel: Physical channel of ADC
 */
static void meas_pmic_irq_enable(bool enable, int reg_channel)
{
	int32_t vmm_pmic_err = 0;
	uint32_t madcirq_mask, madcirq_data;
	uint32_t madcirq_val = (1 <<
		(meas_pmic_adc_reg[reg_channel].adc_sta_bit));

	if (enable) {
		madcirq_mask = madcirq_val;
		madcirq_data = 0;
	} else {
		madcirq_mask = madcirq_val;
		madcirq_data = madcirq_val;
	}

	vmm_pmic_err = pmic_reg_set_field(MADCIRQ_REG_ADDR,
		madcirq_mask, madcirq_data);
	if (IS_ERR_VALUE(vmm_pmic_err))
		pr_err("%s %d VMM PMIC write error %d\n", __func__,
			__LINE__, vmm_pmic_err);
}

/**
 * meas_pmic_cal_adc_bias_current() - Calculate ADC bias current in nA.
 * @reg_channel: Physical channel of ADC
 * @result_high:	MSB result value
 * @return:	 ADC bias current in nA
 */
int meas_pmic_cal_adc_bias_current(int reg_channel, int result_high)
{
	int reg_current;

	reg_current = (int) ((result_high >> 4) & 0xF);

/*	This function is required to compensate wrong NVM setting.
		This change would allow ADC driver to provide correct
		battery temperature and system temperature. This problem is
		fixed in newer NVM version. This needs to be cleaned up this
		code when all HW would have correct NVM setting */
	switch (reg_channel) {
	case ADC_PHY_BAT0TEMP:
	case ADC_PHY_BAT1TEMP:
		if (meas_pmic_state.otp_version <
			MEAS_PMIC_TLP2_SEQ_FIX_NVM_VERSION)
			reg_current = reg_current -
				MEAS_PMIC_BATT_TEMP_SCALING_MODE;
		break;

	case ADC_PHY_SYS0TEMP:
	case ADC_PHY_SYS1TEMP:
	case ADC_PHY_SYS2TEMP:
		if (meas_pmic_state.otp_version <
			MEAS_PMIC_TLP2_SEQ_FIX_NVM_VERSION)
			reg_current = reg_current -
				MEAS_PMIC_SYS_TEMP_SCALING_MODE;
		break;

	default:
		break;
	}

	BUG_ON(reg_current >= ARRAY_SIZE(meas_pmic_current_table));
	return meas_pmic_current_table[reg_current];
}

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
	int vmm_pmic_err = 0;

	BUG_ON(reg_channel < 0);
	BUG_ON(reg_channel >= ADC_PHY_MAX);

	/* Dummy measurement for ACD channel in PMIC A0 HW */
	if ((ADC_PHY_ACCID == reg_channel) &
			meas_pmic_state.acd_dummy_measurement) {
		*p_result_uv = 802660;
		*p_adc_bias_na = 0;
		MEAS_PMIC_DEBUG_DATA_LOG(MEAS_PMIC_MEAS_DONE, reg_channel,
				*p_result_uv, *p_adc_bias_na, 0);
		return;
		}

	if (ADC_PHY_OCV != reg_channel) {

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

		enable_irq(meas_pmic_state.adc_irq_info[
			meas_pmic_adc_reg[reg_channel].adc_sta_bit].irq_num);

		/* Enable the MEAS irq */
		meas_pmic_irq_enable(true, reg_channel);

		meas_pmic_state.meas_pending = true;

		/* Wait for measurement to be done  */
		wait_for_completion(&meas_pmic_state.meas_done);

		meas_pmic_state.meas_pending = false;
	}

	vmm_pmic_err = meas_pmic_reg_read(
		meas_pmic_adc_reg[reg_channel].rslt_lsb_addr, &rslt_lsb);
	if (IS_ERR_VALUE(vmm_pmic_err)) {
		pr_err("%s %d VMM PMIC read error %d\n", __func__,
			__LINE__, vmm_pmic_err);
	}

	if (ADC_PHY_ACCID == reg_channel) {
		vmm_pmic_err = meas_pmic_reg_read(
			USBIDRSLTL_REG_ADDR, &rslt_hsb);
		if (IS_ERR_VALUE(vmm_pmic_err)) {
			pr_err("%s %d VMM PMIC read error %d\n", __func__,
				__LINE__, vmm_pmic_err);
		}
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
	*p_result_uv = (int)((((int64_t)
			(((rslt_hsb & 0xF) << 8) + rslt_lsb) *
			MEAS_PMIC_ADC_RANGE_GAIN_0DB_UV) >>
			(MEAS_PMIC_ADC_RESOLUTION_BITS-1)) + 1) >> 1;

	/* Apply Vbat calibration */
	if ((ADC_PHY_VBAT == reg_channel) ||
		(ADC_PHY_OCV == reg_channel))
		/* Apply calibration factors based on NVM data
			 result_uv =
			 (measured value + offset)*gain/PMIC_GEC_SCALING
			i.e. result_uv =
			 ((measured value + offset)*gain) >> 10 */
		*p_result_uv = ((((*p_result_uv +
			meas_pmic_state.vbat_cal.offset_uv)*
			meas_pmic_state.vbat_cal.gain) >> 9)+1)>>1;

	/* Get current bias used in measurement and converted to nA */
	*p_adc_bias_na = meas_pmic_cal_adc_bias_current(reg_channel, rslt_hsb);

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

	/* Check for ADC saturation */
	if (*p_result_uv > MEAS_PMIC_ADC_SATURATION_LEVEL_UV)
		return -ERANGE;

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
		break;
	default:
		pr_err("%s Warning: Invalid request (%d)\n", __func__, key);
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

/**
 * meas_pmic_read_vbat_cal_data() - Calculate Vbat gain
 *  and offset error from PMIC NVM data.
 * @p_gec:		Pointer of gain error correction for Vbat.
 * @p_oec:		Pointer of offset error correction for Vbat.
 */
static void meas_pmic_read_vbat_cal_data(int *p_gec, int *p_oec)
{
	u32 otp_ready = 0, datum = 0, datumcheck = 0;
	u32 cnt = 0, retry_count = 0;
	int gain = PMIC_GEC_SCALING, offset_uv = 0;
	int vmm_pmic_err = 0;
	int NextSecCountdown = 0, inSectionCount = 0;
	int Cref1 = 0, Cref2 = 0, Cadc1 = 0, Cadc2 = 0;
	bool inHeader = false, inSection = false,
		inSubsection = false, dataValid = true;

	vmm_pmic_err = meas_pmic_reg_read(
		NVM_STAT0_REG_ADDR, &otp_ready);
	if (IS_ERR_VALUE(vmm_pmic_err))
		pr_err("%s %d VMM PMIC read error %d\n",
			__func__, __LINE__, vmm_pmic_err);

	if (otp_ready & NVM_STAT0_REG_OTP_PD_ACT_VAL) {
		/*	If OTP memory is in power down mode, power up the OTP
		memory after writting into the NVM read mailbox registers */
		vmm_pmic_err = meas_pmic_reg_write(
			NVM_MB_ADDRH_REG_ADDR, (u8)(0x0));
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC write error %d\n",
				 __func__, __LINE__, vmm_pmic_err);

		vmm_pmic_err = meas_pmic_reg_write(
			NVM_MB_ADDRL_REG_ADDR, (u8)(0x0));
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC write error %d\n",
				 __func__, __LINE__, vmm_pmic_err);

		retry_count = 0;
		do {
			/* Wait until OTP memory is powered up i.e.
				OTP_PD_ACT==0 */
			usleep_range(500, 1000);
			vmm_pmic_err = meas_pmic_reg_read(
				NVM_STAT0_REG_ADDR, &otp_ready);
			if (IS_ERR_VALUE(vmm_pmic_err))
				pr_err("%s %d VMM PMIC read error %d\n",
				__func__, __LINE__, vmm_pmic_err);

			if (!(otp_ready & NVM_STAT0_REG_OTP_PD_ACT_VAL))
				break;

			retry_count++;
			if (retry_count == PMIC_NVM_READ_MAX_RETRY_CNT)
				dataValid = false;
		} while (retry_count < PMIC_NVM_READ_MAX_RETRY_CNT);
	}

	/* fetch data from OTP */
	cnt = 0;
	while ((cnt < PMIC_NVM_SIZE) && (true == dataValid)) {
		retry_count = 0;

		do {
			/* Set NVM address to read */
		vmm_pmic_err = meas_pmic_reg_write(
				NVM_MB_ADDRH_REG_ADDR, (u8)(cnt>>8));
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC write error %d\n",
				 __func__, __LINE__, vmm_pmic_err);

		vmm_pmic_err = meas_pmic_reg_write(
				NVM_MB_ADDRL_REG_ADDR, (u8)(cnt));
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC write error %d\n",
				 __func__, __LINE__, vmm_pmic_err);

		/* Read NVM data */
		vmm_pmic_err = meas_pmic_reg_read(
				NVM_MB_DATA_REG_ADDR, &datum);
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC read error %d\n",
				 __func__, __LINE__, vmm_pmic_err);

		/* Re-read the data again and validate the the data with
		earlier data. Set NVM address to read again */
		vmm_pmic_err = meas_pmic_reg_write(
			NVM_MB_ADDRH_REG_ADDR, (u8)(cnt>>8));
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC write error %d\n",
		  __func__, __LINE__, vmm_pmic_err);

		vmm_pmic_err = meas_pmic_reg_write(
			NVM_MB_ADDRL_REG_ADDR, (u8)(cnt));
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC write error %d\n",
		__func__, __LINE__, vmm_pmic_err);

		/* Read NVM data */
		vmm_pmic_err = meas_pmic_reg_read(
				NVM_MB_DATA_REG_ADDR, &datumcheck);
		if (IS_ERR_VALUE(vmm_pmic_err))
			pr_err("%s %d VMM PMIC read error %d\n",
				__func__, __LINE__, vmm_pmic_err);

		if (datumcheck == datum)
			break;

		retry_count++;
		if (retry_count == PMIC_NVM_READ_MAX_RETRY_CNT)
			dataValid = false;
		} while (retry_count < PMIC_NVM_READ_MAX_RETRY_CNT);

		if (true == dataValid) {
			if (true == inHeader) {
				/* Bit 2 to 7 of 2nd byte of
					header data contains section length */
				NextSecCountdown = (datum &
					PMIC_SEC_LENGTH_MASK)+1;
				inHeader = false;
				cnt++;
			} else {
				if (NextSecCountdown == 0) {
					/* Bit 1 to 5 of 1st byte of
					header data contains section owner */
					inSection = false;
					inSubsection = false;
					inHeader = true;
					if ((datum &  PMIC_PRODUCT_SEC_ID_MASK)
						 == PMIC_PRODUCT_SEC_ID) {
						inSection = true;
						inSectionCount = 0;
					}
					cnt++;
				} else {
				if (inSection == true) {
					if ((inSectionCount == 0)
						&& (datum == PMIC_FG_SUBSEC_ID))
							inSubsection = true;

				if (inSubsection == true) {
					if ((inSectionCount ==
					FG_VHH_BYTE_POS) ||
					(inSectionCount == FG_VHL_BYTE_POS))
						Cref1 = (Cref1<<8)+((int)datum);

					if ((inSectionCount ==
					FG_VLH_BYTE_POS) ||
					(inSectionCount == FG_VLL_BYTE_POS))
						Cref2 = (Cref2<<8)+((int)datum);

					if ((inSectionCount ==
					FG_RDHH_BYTE_POS) ||
					(inSectionCount == FG_RDHL_BYTE_POS))
						Cadc1 = (Cadc1<<8)+((int)datum);

					if ((inSectionCount ==
					FG_RDLH_BYTE_POS) ||
					(inSectionCount == FG_RDLL_BYTE_POS))
						Cadc2 = (Cadc2<<8)+((int)datum);

					if (inSectionCount == FG_RDLL_BYTE_POS)
						break;

						}
						inSectionCount++;
						NextSecCountdown--;
						cnt++;
					} else {
						/* Skip the section data if
						section does not contains
						vbat calibration data */
						cnt = cnt+NextSecCountdown;
						NextSecCountdown = 0;
					}
				}
			}
		}
	};

	pr_err("%s Cref2=%d Cadc2=%d Cref1=%d Cadc1=%d\n",
		__func__, Cref2, Cadc2, Cref1, Cadc1);

	if ((true == dataValid) && (Cadc2 != Cadc1)
		&& (Cref2 != Cref1)) {
		/* Calculate gain error coef
		gain = 100*(Cref_2-Cref_1)/(Cadc_2-Cadc_1)/Vbat_ratio/
			(1200000/4096)*1024
		*/
		gain = (int)((2*100*(Cref2-Cref1)/(Cadc2-Cadc1)*
			(1<<MEAS_PMIC_ADC_RESOLUTION_BITS)/PMIC_VBATRATIO_IN_MV*
			PMIC_GEC_SCALING/(MEAS_PMIC_ADC_RANGE_GAIN_0DB_UV/1000))
			+1)>>1;

		/* Calculate offset error coef.
			offset_uv =-(Cadc_1*(Cref_2-Cref_1)/(Cadc_2-Cadc_1)-
			Cref_1)*100/(Vbat_ratio).
		Offset is converted to uV format.
		*/
		offset_uv = (int)((-(2*2*10*Cadc1*(Cref2-Cref1)/(Cadc2-Cadc1)-
			2*2*10*Cref1)*5*100/PMIC_VBATRATIO_IN_MV)*10);

		if (offset_uv >= 0)
			offset_uv = (offset_uv+1)/2;
		else
			offset_uv = (offset_uv-1)/2;
	}

	/* Pass result back */
	*p_gec = gain;
	*p_oec = offset_uv;
}

/**
 * meas_pmic_irq_handler - MEAS HW irq handler. It releases the "wait for
 *			completion" to process measurement done.
 * @irq:		irq number, should match IRQ number from device data.
 * @data:		data passed to the irq.
 */
static irqreturn_t meas_pmic_irq_handler(int irq, void *data)
{
	struct meas_pmic_state_data *st = (struct meas_pmic_state_data *)data;
	int reg_channel;

	if (meas_pmic_state.active_channel >= ADC_MAX_NO_OF_CHANNELS)
		return IRQ_NONE;

	reg_channel =
		meas_pmic_state.p_channel_data[
		meas_pmic_state.active_channel]->phy_channel_num;

	disable_irq_nosync(meas_pmic_state.adc_irq_info[
			meas_pmic_adc_reg[reg_channel].adc_sta_bit].irq_num);

	/* First of all, clear the IRQ bit to avoid other interrupts */
	meas_pmic_irq_enable(false, reg_channel);

	/* Signal to release measurement done completion */
	complete(&st->meas_done);
	return IRQ_HANDLED;
}

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

	{"intel_adc_sensors", "VBAT_OCV_ADC", "CH13_OCV", ADC_V_BAT_OCV,
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

	{"intel_adc_sensors", "USBID_ADC", "CH08", ADC_ID_USB,
	 ADC_PHY_USBID, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM, IIO_RESISTANCE},

	{"intel_adc_sensors", "ACCID_ADC", "CH12", ADC_ID_ACC,
	 ADC_PHY_ACCID, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	 ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM, IIO_VOLTAGE},
};

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
	int ret, chan, count = 0, count_2 = 0;
	int chipid = 0, chipid_major = 0, chipid_minor = 0;

	meas_pmic_state.p_platform_device = pdev;

	pdata = kzalloc(sizeof(struct intel_adc_hal_pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("(%s) Memory allocation failed!!\n", __func__);
		ret = (int)pdata;
		goto err_kzalloc;
	}

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

	for (count = 0; count < ADC_STA_IRQ_MAX; count++) {
		if (meas_pmic_state.adc_irq_info[count].irq_name != NULL) {
			meas_pmic_state.adc_irq_info[count].irq_num =
				platform_get_irq_byname(
				pdev,
				meas_pmic_state.adc_irq_info[count].irq_name);

			pr_info("%s irq_num =%d\n", __func__,
				meas_pmic_state.adc_irq_info[count].irq_num);

			if (IS_ERR_VALUE(
				meas_pmic_state.adc_irq_info[count].irq_num)) {
				pr_err("(%s) failed to get irq no\n",
					__func__);
				ret =
				meas_pmic_state.adc_irq_info[count].irq_num;
				goto err_request_irq;
			}

			/* Request and enable the MEAS irq */
			ret = request_threaded_irq(
				meas_pmic_state.adc_irq_info[count].irq_num,
				NULL,
				meas_pmic_irq_handler,
				IRQF_SHARED | IRQF_ONESHOT,
				meas_pmic_state.adc_irq_info[count].irq_name,
				&meas_pmic_state);
			if (IS_ERR_VALUE(ret)) {
				pr_err("(%s) failed requesting interrupt\n",
					__func__);
				goto err_request_irq;
			}
			disable_irq_nosync(
				meas_pmic_state.adc_irq_info[count].irq_num);
		}
	}

	/* Unmask ADC bit in first level interrupt register */
	ret = pmic_reg_set_field(MIRQLVL1_REG_ADDR,
		(1 << IRQLVL1_REG_ADC_BIT_POS), 0);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s %d VMM PMIC write error %d\n", __func__,
			__LINE__, ret);
		goto err_register_hal;
	}

	/* Read of PMIC CHIPID version from PMIC HW */
	ret = meas_pmic_reg_read(CHIPID_ID0_REG_ADDR,
		&chipid);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s %d VMM PMIC write error %d\n",
			 __func__, __LINE__, ret);
		goto err_register_hal;
	}

	chipid_major = ((chipid >> PMIC_CHIPID_ID0_MAJREV_BIT_POS) &
		((1 << PMIC_CHIPID_ID0_MAJREV_BIT_LEN) - 1));

	chipid_minor = ((chipid >> PMIC_CHIPID_ID0_MINREV_BIT_POS) &
		((1 << PMIC_CHIPID_ID0_MINREV_BIT_LEN) - 1));

	pr_info("%s PMIC CHIP ID = %s%d\n",
		__func__, meas_pmic_chip_rev[chipid_major].str, chipid_minor);

	/* ACD is not supported with older version than B0 PMIC */
	if (chipid_major < PMIC_CHIPID_ID0_B0_MAJREV)
		meas_pmic_state.acd_dummy_measurement = true;

	ret = meas_pmic_reg_read(OTPVERSION_REG_ADDR,
		&meas_pmic_state.otp_version);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s %d VMM PMIC write error %d\n",
			 __func__, __LINE__, ret);
		goto err_register_hal;
	}

	pr_info("%s OTP version=%d\n",
		__func__, meas_pmic_state.otp_version);

	/* Initialise measurement done completion */
	init_completion(&meas_pmic_state.meas_done);

	/* Read Vbat calibration data from PMIC NVM data and
		calculate gain and offset */
	meas_pmic_read_vbat_cal_data(&meas_pmic_state.vbat_cal.gain,
			&meas_pmic_state.vbat_cal.offset_uv);
	pr_info("VBAT GEC = %d\n", meas_pmic_state.vbat_cal.gain);
	pr_info("VBAT OEC = %d\n", meas_pmic_state.vbat_cal.offset_uv);

	/* Register to HAL and return */
	ret = adc_register_hal(&meas_pmic_adc_hal_interface, dev,
				&meas_pmic_state.operation_done_cb);
	if (ret) {
		pr_err("(%s) Registration of ADC HAL is failed!! %d\n",
			__func__, ret);
		goto err_register_hal;
	}

	pr_info("%s probe OK\n", __func__);
	return 0;

err_register_hal:
	for (count = 0; count < ADC_STA_IRQ_MAX; count++)
		if (meas_pmic_state.adc_irq_info[count].irq_name != NULL)
			free_irq(meas_pmic_state.adc_irq_info[count].irq_num,
				&meas_pmic_state);
err_request_irq:
	for (count_2 = 0; count_2 < count; count_2++)
		if (meas_pmic_state.adc_irq_info[count].irq_name != NULL)
			free_irq(meas_pmic_state.adc_irq_info[count].irq_num,
				&meas_pmic_state);
	kfree(pdata);
	dev->platform_data = NULL;
err_kzalloc:
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
	int count = 0;
	struct device *dev = &pdev->dev;

	for (count = 0; count < ADC_STA_IRQ_MAX; count++)
		if (meas_pmic_state.adc_irq_info[count].irq_name != NULL)
			free_irq(meas_pmic_state.adc_irq_info[count].irq_num,
				&meas_pmic_state);

	/* De-register the ADC HAL */
	adc_unregister_hal(&meas_pmic_adc_hal_interface, dev);

	/* De-allocate platform data */
	kfree(dev->platform_data);
	dev->platform_data = NULL;
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
