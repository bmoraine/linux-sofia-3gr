/*
 * meas_ag620.c - AGOLD620 MEAS HAL.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define DRIVER_NAME "ag620_meas"
#define pr_fmt(fmt) DRIVER_NAME": "fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/iio/intel_adc_hal_interface.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/time.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include "meas_ag620_reg.h"

/* This is used as the timeout for OCV measurement detecting in ms */
#define MEAS_AG620_OCV_DET_MS				(500)
/* This is used as the timeout for VBAT TYP measurement detecting in ms */
#define MEAS_AG620_VBAT_TYP_DET_MS			(10)
/* This is used as the timeout for VBAT MIN measurement detecting in ms */
#define MEAS_AG620_VBAT_MIN_DET_MS			(20)

/* ADC resolution in bits */
#define MEAS_AG620_ADC_RESOLUTION_BITS			(12)
/* ADC range for gain=1 (0db) in uV */
#define MEAS_AG620_ADC_RANGE_GAIN_0DB_UV		(1200000)
/* ADC range for gain=2 (6db) in uV */
#define MEAS_AG620_ADC_RANGE_GAIN_6DB_UV		(600000)
/*
 * Offset in uV to apply to measurement result when using 6db gain setting, as
 * per datasheets (see section "Gain and offset calibration of measurement
 * interface")
 */
#define MEAS_AG620_GAIN_6DB_MEAS_RESULT_OFFSET_UV	(300000)

/* Size of debug data array (has to be power of 2!!!) */
#define MEAS_AG620_DEBUG_DATA_SIZE (1<<6)

#define SYSFS_INPUT_VAL_LEN (1)

/* Macro to log debug data */
#define MEAS_AG620_DEBUG_DATA_LOG(_event, _context, _context2, \
						_context3, _context4) \
{ \
	spin_lock(&meas_ag620_debug_data.lock); \
	meas_ag620_debug_data.log_array[meas_ag620_debug_data.index].\
							time_stamp = jiffies; \
	meas_ag620_debug_data.log_array[meas_ag620_debug_data.index].\
							event = (_event); \
	\
	meas_ag620_debug_data.log_array[meas_ag620_debug_data.index].context = \
						(unsigned int)(_context); \
	meas_ag620_debug_data.log_array[meas_ag620_debug_data.index].context2 =\
						(unsigned int)(_context2); \
	meas_ag620_debug_data.log_array[meas_ag620_debug_data.index].context3 =\
						(unsigned int)(_context3); \
	meas_ag620_debug_data.log_array[meas_ag620_debug_data.index].context4 =\
						(unsigned int)(_context4); \
	meas_ag620_debug_data.index++; \
	meas_ag620_debug_data.index &= (MEAS_AG620_DEBUG_DATA_SIZE-1); \
	spin_unlock(&meas_ag620_debug_data.lock); \
	if (meas_ag620_debug_data.printk_logs_en)\
		pr_debug("%s:%u,%u,%u,%u\n", #_event, \
						(unsigned int)_context, \
						(unsigned int)_context2, \
						(unsigned int)_context3, \
						(unsigned int)_context4); \
}

/**
 * meas_ag620_calibration - A structure that contains internal calibration
 *			related information.
 *
 * @period_s:		The calibration period in seconds, i.e. how often a
 *			calibration is done.
 * @last_timestamp_s:	Last calibration timestamp in seconds. This is used to
 *			determine whether calibration is too old and requires
 *			re-doing.
 * @gnd:		Measured GND value during last calibration.
 * @vref:		Measured VREF value during last calibration.
 */
struct meas_ag620_calibration {
	int period_s;
	__kernel_time_t last_timestamp_s;
	int gnd;
	int vref;
};

/**
 * meas_ag620_state - A structure representing the internal meas_ag620_state of
 *			the MEAS HAL.
 *
 * @lock:		Spin lock used to protect critical sections when
 *			modifying device state data.
 * @power_mode:		Power mode of the HW.
 * @suspended:		TRUE=Device is suspended, otherwise not.
 * @active_channel:	Channel currently being measured.
 * @p_channel_data:	Pointer to channel data of each of the supported
 *			channels.
 * @regmap:		A pointer to the MEAS memory-mapped registers. For
 *			convenience purposes.
 * @ididev:		Pointer to IDI driver instance.
 * @calibration:	Calibration data.
 * @pd_timer:		Timer used to time peak detector search.
 * @irq_num:		IRQ number for underlying HW. It comes from device data.
 * @meas_done:		Completion used to wait for measurement done.
 * @operation_done_cb:	Pointer to HAL callback.
 */
struct meas_ag620_state_data {
	spinlock_t lock;
	enum adc_hal_power_mode power_mode;
	bool suspended;
	enum adc_channel active_channel;
	struct intel_adc_hal_channel_data
		*p_channel_data[ADC_MAX_NO_OF_CHANNELS];
	struct meas_regmap *regmap;
	struct idi_peripheral_device *ididev;
	struct device_state_pm_state *pm_state_en;
	struct device_state_pm_state *pm_state_dis;
	struct meas_ag620_calibration calibration;
	struct timer_list pd_timer;
	int irq_num;
	struct completion meas_done;
	adc_hal_cb_t operation_done_cb;
	bool meas_pending;
};

static struct meas_ag620_state_data meas_ag620_state = {
	.power_mode = ADC_HAL_POWER_MODE_OFF,
	.suspended = false,
	.active_channel = ADC_MAX_NO_OF_CHANNELS,
	.meas_pending = false,
};

/** Enum for driver debug events */
enum meas_ag620_debug_event {
	MEAS_AG620_INIT,
	MEAS_AG620_DE_INIT,
	MEAS_AG620_SUSPEND_OK,
	MEAS_AG620_SUSPEND_EBUSY,
	MEAS_AG620_RESUME,
	MEAS_AG620_CAL_DONE
};
/**
 * meas_ag620_debug_data - Structure to collect debug data.
 * @index:		Index of log array.
 * @lock:			Spin lock for atomic access.
 * @log_array:		Debug data logging array.
 * @time_stamp:		System time stamp in jiffies.
 * @event:		Event that caused logging.
 * @hmeas:		Measurement handle (if available).
 * @context-4:		Event context parameters.
 */
struct meas_ag620_debug_data {
	uint index;
	int printk_logs_en;
	spinlock_t lock;
	struct {
		uint time_stamp;
		enum meas_ag620_debug_event event;
		int context;
		int context2;
		int context3;
		int context4;
	} log_array[MEAS_AG620_DEBUG_DATA_SIZE];
};

/* Array to collect debug data */
static struct meas_ag620_debug_data meas_ag620_debug_data = {
	.printk_logs_en = 0,
};

/*
 * meas_ag620_current_table lists the current values that are exported to the
 * ADC for scaling. The associated list of register values is declared in
 * meas_ag620_current_reg_table.
 */
static int meas_ag620_current_table[] = {
	0,
	4500,
	9000,
	18000,
	36000,
	72000,
	144000
};

/** Peak detector modes */
enum meas_ag620_peak_detect_modes {
	ADC_HAL_PEAKDET_MODE_OFF,
	ADC_HAL_PEAKDET_MODE_MIN_DETECT,
	ADC_HAL_PEAKDET_MODE_MAX_DETECT,
	ADC_HAL_PEAKDET_MODE_READOUT
};

/**
 * meas_ag620_dump_register() - Dump MEAS registers.
*/
static void meas_ag620_dump_register(void)
{
	/* Start dumping relevant register content */

	pr_err("***Dump of TSMU registers\n");
	pr_err("CALI		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CALI));
	pr_err("CLC		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CLC));
	pr_err("CLK		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CLK));
	pr_err("CONF		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CONF));
	pr_err("RUN_CTRL	0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_RUN_CTRL));
	pr_err("STAT		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_STAT));
	pr_err("PEAK		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_PEAK));

	pr_err("CTRL_B0		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CTRL_B0));
	pr_err("CTRL_B1		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CTRL_B1));
	pr_err("CTRL_B2		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CTRL_B2));
	pr_err("CTRL_B3		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CTRL_B3));
	pr_err("CTRL_B4		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CTRL_B4));
	pr_err("CTRL_B5		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CTRL_B5));
	pr_err("CTRL_B6		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CTRL_B6));
	pr_err("CTRL_B7		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_CTRL_B7));

	pr_err("DATA_B0		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_DATA_B0));
	pr_err("DATA_B1		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_DATA_B1));
	pr_err("DATA_B2		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_DATA_B2));
	pr_err("DATA_B3		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_DATA_B3));
	pr_err("DATA_B4		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_DATA_B4));
	pr_err("DATA_B5		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_DATA_B5));
	pr_err("DATA_B6		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_DATA_B6));
	pr_err("DATA_B7		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_DATA_B7));

	pr_err("RIS		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_RIS));
	pr_err("IMSC		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_IMSC));
	pr_err("MIS		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_MIS));
	pr_err("ICR		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_ICR));
	pr_err("ISR		0x%08x\n",
		ioread32(&meas_ag620_state.regmap->MEAS_ISR));
}

/**
 * current_to_reg() - Helper function to turn a current value into the
 * corresponding register value.
 */
static inline int current_to_reg(int current_na)
{
	int i;
	int ret = ARRAY_SIZE(meas_ag620_current_table);

	for (i = 0; i < ARRAY_SIZE(meas_ag620_current_table)
		&& meas_ag620_current_table[i] <= current_na; i++) {
		if (meas_ag620_current_table[i] == current_na)
			ret = i;
	}

	BUG_ON(ret == ARRAY_SIZE(meas_ag620_current_table));

	return ret;
}

/**
 * avg_sample_level_to_reg() - Translates a platform-independent sample level
 * into its register version.
 */
static inline int avg_sample_level_to_reg(enum adc_hal_avg_sample_level
						avg_sample_level)
{
	int ret = -EINVAL;

	switch (avg_sample_level) {
	case ADC_HAL_AVG_SAMPLE_LEVEL_LOW:
		ret = 0;
		break;
	case ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM:
		ret = 1;
		break;
	case ADC_HAL_AVG_SAMPLE_LEVEL_HIGH:
		ret = 3;
		break;
	default:
		BUG();
	}

	return ret;
}

/**
 * meas_ag620_internal_temperature_calc() - Translates the internal die
 * temperature from uV to milli degs Kelvin.
 *
 * @meas_val			[in] Measured value to convert.
 */
static int meas_ag620_internal_temperature_calc(int meas_result_uv)
{
	/* Calculate and report temperature in unit mV/K
	* On the AG6x0 the temperature in Deg C is calculated from the ADC
	* measurement (ADCout) as follows:
	*      T = (ADCout - 2047.5) * 0.24908 - 273.15 oC
	* Note here that ADCout is the ADC count of the measurement and 2047.5
	* is half the range of the total ADC counts. The conversion from Kelvin
	* to deg C is done in the Logical PMICTEMP sensor.
	* The value to be given back to the logical layer is therefore:
	* (ADCout - 2047.5) * 0.24908
	* The measurement is done with Gain 1 implying the full range
	* of the ADC, which is 1200000.The ADC measurement performed by the HAL
	* is in uV and therefore the above has to be converted to mV as follows:
	* (meas_result_uv - 600000)* 0.24908 * 4096/1200000
	* Additionally, the logical PMICTEMP sensor expects to get back the
	* measured temperature in units 1mv/K.This is incorporated using:
	* (meas_result_uv - 600000)*4096/1200000 *1000uV/1K.
	* This results in (meas_result_uv - 600000)* 64 / 75 */
	return ((meas_result_uv -
		(MEAS_AG620_ADC_RANGE_GAIN_0DB_UV >> 1)) * 64) / 75;
}

/**
 * meas_ag620_irq_enable() - HAL-side function for enabling the MEAS irq.
 * @enable:	True if the irq is to be enabled, false for disabled.
 */
static void meas_ag620_irq_enable(bool enable)
{
	union meas_imsc meas_imsc_reg = { 0 };
	union meas_icr meas_icr_reg = { 0 };

	if (enable) {
		meas_icr_reg.s.MEAS_RDY = 1;
		iowrite32(meas_icr_reg.val, &meas_ag620_state.regmap->MEAS_ICR);
		meas_imsc_reg.s.MEAS_RDY = 1;
		iowrite32(meas_imsc_reg.val,
				&meas_ag620_state.regmap->MEAS_IMSC);
	} else {
		meas_imsc_reg.s.MEAS_RDY = 0;
		iowrite32(meas_imsc_reg.val,
				&meas_ag620_state.regmap->MEAS_IMSC);
	}
}

/**
 * meas_ag620_read_raw() - Configure the registers to perform a current
 *				measurement.
 *
 * @reg_channel:	The register value of the channel selector
 *			(e.g. M10 -> 11).
 * @reg_current:	The register value of the current selector
 *			(e.g. 4.5mA -> 1)
 * @avg_sample_level:	Number of samples for HW averaging.
 * @converted:		True=value is converted to uV otherwise is returned
 *			in counts.
 */
static int meas_ag620_read_raw(int reg_channel,
				int reg_current,
				bool calibrated,
				enum adc_hal_avg_sample_level avg_sample_level,
				bool converted)
{
	int ret;
	union meas_ctrl_bx reg_bx = { 0 };
	union meas_conf conf = { 0 };
	union meas_stat stat;
	union meas_data_b data_reg;

	/* The setting gain=2 or 6db is not currently used. */
	enum E_MEAS_CTRL_B_GBY gain = MEAS_CTRL_B_GBY_0;

	BUG_ON(reg_channel < 0);

	/* Configure the MEAS_CONF register */
	conf.s.RES_WPTRB = 1;	/* Reset Mode B write pointer */
	conf.s.STARTB = 0;	/* Not starting any measurement yet */
	conf.s.SWTRIGB = 0;	/* No software trigger */
	conf.s.ENTRIGB = MEAS_CONF_ENTRIGB_NOTRIG;	/* Trigger disabled */
	conf.s.ENSTOP = MEAS_CONF_ENSTOP_1;

	/* Keep interrupts disabled for now */
	conf.s.ENIRQB = MEAS_CONF_ENIRQB_0;
	conf.s.ADCON = MEAS_CONF_ADCON_1;	/* Turn ADC ON */
	iowrite32(conf.val, &meas_ag620_state.regmap->MEAS_CONF);

	/* Configure the MEAS_CTRL_Bx registers */
	reg_bx.s.MXBY = reg_channel;
	reg_bx.s.GBY = gain;	/* "0dB" gain or gain =1 */
	reg_bx.s.TCBY = reg_current;	/* Requested current */
	reg_bx.s.SETTLINGBY = MEAS_CTRL_B_SETTLINGBY_3;	/* 8us settling time */
	reg_bx.s.SYNCBY = MEAS_CTRL_B_SYNCBY_1;	/* Synchronization control ON */

	 /* "Continuation without trigger" OFF */
	reg_bx.s.CONTBY = MEAS_CTRL_B_CONTBY_1;

	/* Auto-calibration ON if calibrated false */
	reg_bx.s.BYPBY = !calibrated;
	reg_bx.s.MAVGBY = avg_sample_level_to_reg(avg_sample_level);
	reg_bx.s.AVGBY = MEAS_CTRL_B_AVGBY_0;	/* Do NOT bypass the averager */
	iowrite32(reg_bx.val, &meas_ag620_state.regmap->MEAS_CTRL_B0);

	/* Enable the MEAS irq so that we're ready to receive it as soon as
	it's triggered */
	meas_ag620_irq_enable(true);

	/* Reset the RES_WPTRB bit and finally start measurement */
	conf.s.RES_WPTRB = 0;
	conf.s.STARTB = 1;	/* Single conversion */
	conf.s.ENIRQB = MEAS_CONF_ENIRQB_1;	/* Enable interrupts now */
	iowrite32(conf.val, &meas_ag620_state.regmap->MEAS_CONF);

	meas_ag620_state.meas_pending = true;

	/* Wait for measurement to be done  */
	wait_for_completion(&meas_ag620_state.meas_done);

	meas_ag620_state.meas_pending = false;

	/* The first thing to do it to reat the status bit and trap if the READY
	bit is not set */
	stat.val = ioread32(&meas_ag620_state.regmap->MEAS_STAT);
	BUG_ON(stat.s.READYB == 0);

	/* Configure the ADC back into rest mode. */
	conf.s.ENTRIGB = MEAS_CONF_ENTRIGB_NOTRIG;
	conf.s.ENIRQB = MEAS_CONF_ENIRQB_0;
	conf.s.RES_WPTRB = 0;
	conf.s.ADCON = MEAS_CONF_ADCON_1;
	conf.s.ENSTOP = MEAS_CONF_ENSTOP_1;
	conf.s.SWTRIGB = 0;
	conf.s.STARTB = 0;
	iowrite32(conf.val, &meas_ag620_state.regmap->MEAS_CONF);

	/* Read the data back */
	data_reg.val = readl(&meas_ag620_state.regmap->MEAS_DATA_B0);
	ret = data_reg.s.DATA_BY;

	if (converted) {
		/* Convert counts into uV taking into account gain setting */
		switch (gain) {
		case MEAS_CTRL_B_GBY_0:
			/* According to datasheets: Vin (uV) = (measured Counts)
				* (measurement input range in uV) /
						(ADC resolution) */
			ret =
				(int)((((long long int)ret *
					MEAS_AG620_ADC_RANGE_GAIN_0DB_UV) >>
					(MEAS_AG620_ADC_RESOLUTION_BITS - 1)) +
					1) >> 1;
			break;
		case MEAS_CTRL_B_GBY_1:
			/* According to datasheets: Vin (uV) = (measured Counts)
				* (measurement input range in uV) /
						(ADC resolution) + Offset */
			ret =
				((int)
				((((long long int)ret *
				MEAS_AG620_ADC_RANGE_GAIN_6DB_UV) >>
				(MEAS_AG620_ADC_RESOLUTION_BITS - 1)) +
				1) >> 1) +
				MEAS_AG620_GAIN_6DB_MEAS_RESULT_OFFSET_UV;
			break;
		default:
			BUG();
		}

		/* Apply post processing functions to special channels. */
		switch (reg_channel) {
		case ADC_PHY_M13:
			/* The internal die temperature must be converted to uV
			representing Kelvin */
			ret = meas_ag620_internal_temperature_calc(ret);
		default:
			/* Nothing to do here. */
			break;
		}
	};
	return ret;
}

/**
 * meas_ag620_set_peak_detector() - Set peak detector modes, observing specific
 *				sequence and timing.
 * @command:		The command to execute (configure, read).
 * @peakmode:		Max/min/readout peak mode.
 */
static void meas_ag620_set_peak_detector(enum meas_ag620_peak_detect_modes
					peakmode)
{
	union meas_peak reg_peak = { 0 };

	switch (peakmode) {
	case ADC_HAL_PEAKDET_MODE_OFF:
		/* Disable peak detector */
		reg_peak.s.PEAKON = MEAS_PEAK_PEAKON_OFF;
		reg_peak.s.PEAKRESET = MEAS_PEAK_PEAKRESET_NORESET;
		reg_peak.s.PEAKMODE = MEAS_PEAK_PEAKMODE_READOUT;
		iowrite32(reg_peak.val, &meas_ag620_state.regmap->MEAS_PEAK);
		break;
	case ADC_HAL_PEAKDET_MODE_MIN_DETECT:
	case ADC_HAL_PEAKDET_MODE_MAX_DETECT:
		/* Set up the peak detector */

		/* Step 1: start peak detection */
		reg_peak.s.PEAKON = MEAS_PEAK_PEAKON_OFF;
		reg_peak.s.PEAKRESET = MEAS_PEAK_PEAKRESET_RESET;
		reg_peak.s.PEAKMODE =
			(peakmode ==
			ADC_HAL_PEAKDET_MODE_MIN_DETECT ?
			MEAS_PEAK_PEAKMODE_MINDETECT :
			MEAS_PEAK_PEAKMODE_MAXDETECT);
		iowrite32(reg_peak.val, &meas_ag620_state.regmap->MEAS_PEAK);

		/* Step 2: enter reset mode */
		reg_peak.s.PEAKON = MEAS_PEAK_PEAKON_ON;
		iowrite32(reg_peak.val, &meas_ag620_state.regmap->MEAS_PEAK);
		udelay(15);	/* Wait for at least 15us */

		/* Step 3: clear peakreset */
		reg_peak.s.PEAKRESET = MEAS_PEAK_PEAKRESET_NORESET;
		iowrite32(reg_peak.val, &meas_ag620_state.regmap->MEAS_PEAK);
		udelay(20);	/* Wait for at least 20us */
		break;
	case ADC_HAL_PEAKDET_MODE_READOUT:
		/* Read the result */
		/* Step 4: set into READOUT mode and acquire result. */
		reg_peak.s.PEAKON = MEAS_PEAK_PEAKON_ON;
		reg_peak.s.PEAKRESET = MEAS_PEAK_PEAKRESET_NORESET;
		reg_peak.s.PEAKMODE = MEAS_PEAK_PEAKMODE_READOUT;
		iowrite32(reg_peak.val, &meas_ag620_state.regmap->MEAS_PEAK);
		udelay(20);	/* Wait for at least 20us */
		break;
	default:
		BUG();
	}
}

/**
 * intel_adc_start_settling_meas() - Settling polling timer callback. Start a
 * measurement in a delayed fashion.
 */
static void meas_ag620_peak_detect_timeout(unsigned long dummy)
{
	/* Unsed parameter */
	(void)dummy;

	/* Set peak detector in read-out mode */
	meas_ag620_set_peak_detector(ADC_HAL_PEAKDET_MODE_READOUT);

	/* Run set up done callback */
	(meas_ag620_state.operation_done_cb) (ADC_HAL_CB_EVENT_MEAS_SET_UP_DONE,
						((union adc_hal_cb_param) {
						.power_mode =
						ADC_HAL_POWER_MODE_ON}));
}

/**
 * meas_ag620_calibration() - Perform calibration, if required.
 */
static void meas_ag620_calibration(void)
{

	struct timespec currtime;
	struct meas_ag620_calibration *p_calibration =
		&meas_ag620_state.calibration;
	union meas_cali cali_reg;

	/* Get current timestamp */
	ktime_get_ts(&currtime);
	/* Do calibration if required, i.e. when calibration is too old, or if
	it has never been done. */
	if ((0 == p_calibration->last_timestamp_s)
		|| ((currtime.tv_sec - p_calibration->last_timestamp_s) >
		p_calibration->period_s)) {
		/* Read the ground value */
		p_calibration->gnd = meas_ag620_read_raw(ADC_PHY_M10,
						0,
						false
						/* uncalibrated */ ,
						ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM,
						false);
		/* Read the VREF value */
		p_calibration->vref = meas_ag620_read_raw(ADC_PHY_M15,
						0,
						false
						/* uncalibrated */ ,
						ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM,
						false);

		/* Do debug data logging */
		MEAS_AG620_DEBUG_DATA_LOG(MEAS_AG620_CAL_DONE,
						p_calibration->last_timestamp_s,
						currtime.tv_sec,
						p_calibration->gnd,
						p_calibration->vref);
		/* Store current timestamp for calibration timestamp */
		p_calibration->last_timestamp_s = currtime.tv_sec;
	}
	/* Write down the results to the calibration register */
	cali_reg.val = 0;
	cali_reg.s.Y0 = p_calibration->gnd;
	cali_reg.s.Y1 = p_calibration->vref;
	cali_reg.s.BYP = MEAS_CTRL_B_BYPBY_0;
	cali_reg.s.XBON = MEAS_CALI_XBON_1;	/* Keep central biasing on */
	iowrite32(cali_reg.val, &meas_ag620_state.regmap->MEAS_CALI);
}

/**
 * meas_ag620_set() - Set parameters of the ADC HAL driver.
 * @key:			Key to specify parameter(s) to set.
 * @params:		Structure holding list of parameters.
 *			Returns Error number; may be tested as boolean with
 *			0=success, other=fail. Some errors may need re-trial.
 */
static int meas_ag620_set_power_mode(enum adc_hal_power_mode new_power_mode)
{
	int ret = WNOTREQ;

	/* Protect critical section when testing and modifying device
	state data */
	spin_lock(&meas_ag620_state.lock);

	/* Check whether device is suspended. If so, return an error as no
	operation is allowed on a suspended device */
	if (meas_ag620_state.suspended) {
		spin_unlock(&meas_ag620_state.lock);
		return -EIO;
	}

	/* Check whether new power mode requested is different from
	current mode */
	if (meas_ag620_state.power_mode != new_power_mode) {

		/* Update power mode variable */
		meas_ag620_state.power_mode = new_power_mode;

		/* End of critical section */
		spin_unlock(&meas_ag620_state.lock);

		/* Set new power mode */
		switch (new_power_mode) {
		case ADC_HAL_POWER_MODE_OFF:{
				union meas_conf conf = { 0 };
				union meas_cali cali_reg = { 0 };
				union meas_stat stat = { 0 };

				/*
				* BUSYADC: HIGH when ADC operation is going on
				* or ADC is in power save mode. No new ADC
				* conversion should be started during BUSYADC is
				* HIGH. Note: BUSYADC is equal to the inverted
				* signal EOC.
				*/
				stat.val =
					ioread32(&meas_ag620_state.regmap->
						MEAS_STAT);
				if (stat.s.BUSYADC == 1) {
					/* this should never happen, ignore the
					power off request */
					pr_warn("POWER_MODE_OFF request while ADC is still running or already off\n");
					pr_warn("CLC		0x%08x\n",
						ioread32
						(&meas_ag620_state.
						regmap->MEAS_CLC));
					pr_warn("CLK		0x%08x\n",
						ioread32
						(&meas_ag620_state.
						regmap->MEAS_CLK));
					pr_warn("CONF		0x%08x\n",
						ioread32
						(&meas_ag620_state.
						regmap->MEAS_CONF));
					pr_warn("RUN_CTRL	0x%08x\n",
						ioread32(&meas_ag620_state.
							regmap->MEAS_RUN_CTRL));
					pr_warn("STAT		0x%08x\n",
						ioread32
						(&meas_ag620_state.
						regmap->MEAS_STAT));
					pr_warn("\t channel=%d, pending=%d\n",
						meas_ag620_state.active_channel,
						meas_ag620_state.meas_pending);
				} else {
					/* Make sure peak detector is off */
					meas_ag620_set_peak_detector
						(ADC_HAL_PEAKDET_MODE_OFF);

					/* Disable relevant HW resources */

					/* Turn ADC OFF */
					conf.s.ADCON = MEAS_CONF_ADCON_0;

					iowrite32(conf.val,
					 &meas_ag620_state.regmap->
					  MEAS_CONF);
					/* Turn central biasing off */
					cali_reg.s.XBON = MEAS_CALI_XBON_0;
					iowrite32(cali_reg.val,
					 &meas_ag620_state.regmap->MEAS_CALI);

					iowrite32(0,
					 &meas_ag620_state.regmap->
					  MEAS_RUN_CTRL);
					/* Disable power domain */
					idi_set_power_state(meas_ag620_state.
							ididev,
							meas_ag620_state.
							pm_state_dis,
							false);
				}
			}
			break;
		case ADC_HAL_POWER_MODE_ON:{
				union meas_ctrl_bx reg_bx = { 0 };
				union meas_conf conf = { 0 };
				union meas_cali cali_reg = { 0 };
				union U_MEAS_CLC clc = { 0 };
				unsigned int retry = 0;

				/* Enable power domain */
				do {
					if (retry++ > 0) {
						pr_warn("MEAS power on failed CLC 0x%08x\n",
						 (unsigned int)(clc.val));
						 udelay(250);
						 BUG_ON(retry == 10);
					}
					BUG_ON(0 !=
						idi_set_power_state
						(meas_ag620_state.ididev,
						meas_ag620_state.pm_state_en,
						true));

					clc.val = ioread32(&meas_ag620_state.
							regmap->MEAS_CLC);

				} while (((clc.MEAS_CLC_STRUCTURE.DISS == 1)
					|| (clc.MEAS_CLC_STRUCTURE.DISR == 1)));

				/* Enable relevant HW resources */
				iowrite32(0, &meas_ag620_state.
							regmap->MEAS_CLK);
				iowrite32(1, &meas_ag620_state.
							regmap->MEAS_RUN_CTRL);

				/* Turn on central biasing */
				cali_reg.s.XBON = MEAS_CALI_XBON_1;
				iowrite32(cali_reg.val,
					&meas_ag620_state.regmap->MEAS_CALI);

				/* Configure the MEAS_CONF register */

				/* Not starting any measurement yet */
				conf.s.STARTB = 0;
				conf.s.SWTRIGB = 0; /* No software trigger */

				/* Trigger disabled */
				conf.s.ENTRIGB = MEAS_CONF_ENTRIGB_NOTRIG;
				conf.s.ENSTOP = MEAS_CONF_ENSTOP_1;
				/* Keep interrupts disabled for now */
				conf.s.ENIRQB = MEAS_CONF_ENIRQB_0;
				/* Turn ADC ON */
				conf.s.ADCON = MEAS_CONF_ADCON_1;
				iowrite32(conf.val,
					&meas_ag620_state.regmap->MEAS_CONF);

				/* Write sequence stop marker on 2nd control
				register entry */
				reg_bx.s.MXBY = ADC_PHY_OFF;
				iowrite32(reg_bx.val,
						&meas_ag620_state.regmap->
						MEAS_CTRL_B1);

				/* wait for the ADC HW to settle */
				udelay(150);

				/* Call calibration function to either set
				previous calibration values or get new ones */
				meas_ag620_calibration();
			}
			break;
		default:
			ret = -EINVAL;
			break;
		};
	} else {
		/* End of critical section */
		spin_unlock(&meas_ag620_state.lock);
		ret = -EINVAL;
	}

	return ret;
}

/**
 * meas_ag620_set_up_meas() -	Set up ADC measurement.
 * @channel:		ADC Channel to perform measurement on.
 *			Returns Error number; may be tested as boolean with
 *			0=success, other=fail. Some errors may need re-trial.
 */
static int meas_ag620_set_up_meas(enum adc_channel channel)
{
	int ret = 0;

	/* Store active channel */
	meas_ag620_state.active_channel = channel;

	if (ADC_V_BAT_OCV == channel) {
		/* Set up peak detector to do OCV measurement and start search
		timeout */
		meas_ag620_set_peak_detector(ADC_HAL_PEAKDET_MODE_MAX_DETECT);
		mod_timer(&meas_ag620_state.pd_timer,
			jiffies + msecs_to_jiffies(MEAS_AG620_OCV_DET_MS));
	} else if (ADC_V_BAT_MIN == channel) {
		/* Set up peak detector to do Vbat Min measurement and start
		search timeout */
		meas_ag620_set_peak_detector(ADC_HAL_PEAKDET_MODE_MIN_DETECT);
		mod_timer(&meas_ag620_state.pd_timer,
				jiffies +
				 msecs_to_jiffies(MEAS_AG620_VBAT_MIN_DET_MS));
	} else if (ADC_V_BAT_TYP == channel) {
		/* Set up peak detector to do Vbat Max measurement and start
		search timeout */
		meas_ag620_set_peak_detector(ADC_HAL_PEAKDET_MODE_MAX_DETECT);
		mod_timer(&meas_ag620_state.pd_timer,
				jiffies +
				 msecs_to_jiffies(MEAS_AG620_VBAT_TYP_DET_MS));
	} else {
		/* Set up not required */
		ret = WNOTREQ;
	}

	return ret;
}

/**
 * meas_ag620_stop_meas() - Stop ADC measurement.
 * @channel:		ADC Channel to stop measurement on.
 *			Returns Error number; may be tested as boolean with
 *			0=success, other=fail. Some errors may need re-trial.
 */
static int meas_ag620_stop_meas(enum adc_channel channel)
{
	int i;

	/* Stop running timer */
	(void)del_timer_sync(&meas_ag620_state.pd_timer);

	/* wait until a measurement pending in meas_ag620_read_raw() */
	for (i = 0; i < 100 && meas_ag620_state.meas_pending; i++) {
		pr_warn("stop measurement on ADC running\n");
		udelay(300);
	}

	return 0;
}

/**
 * meas_ag620_get_meas() - Get measurement value.
 * @channel			ADC channel to measure.
 * @p_adc_bias_na		ADC bias current to set in nA, or 0 for voltage
 *				mode.
 * @p_result_uv			Pointer to measurement result.
 *				Returns Error number; may be tested as boolean
 *				with 0=success, other=fail. Some errors may
 *				need re-trial.
 */
static int meas_ag620_get_meas(enum adc_channel channel,
				int *p_adc_bias_na, int *p_result_uv)
{
	/* Get measurement value */
	*p_result_uv =
		meas_ag620_read_raw(meas_ag620_state.p_channel_data[channel]->
				phy_channel_num, current_to_reg(*p_adc_bias_na),
				true,
				meas_ag620_state.p_channel_data[channel]->
				average_sample, true);
	/* Clear active channel */
	meas_ag620_state.active_channel = ADC_MAX_NO_OF_CHANNELS;

	return 0;
}

/**
 * meas_ag620_set() - Set parameters of the ADC HAL driver.
 * @key:		Key to specify parameter(s) to set.
 * @params:		Structure holding list of parameters.
 *			Returns Error number; may be tested as boolean with
 *			0=success, other=fail. Some errors may need re-trial.
 */
static int meas_ag620_set(enum adc_hal_set_key key,
				union adc_hal_set_params params)
{
	int ret = 0;

	switch (key) {
	case ADC_HAL_SET_POWER_MODE:
		ret = meas_ag620_set_power_mode(params.power_mode);
		break;
	case ADC_HAL_SET_UP_MEAS:
		ret = meas_ag620_set_up_meas(params.channel);
		break;
	case ADC_HAL_STOP_MEAS:
		ret = meas_ag620_stop_meas(params.channel);
		break;
	case ADC_HAL_DUMP_REGISTER:
		meas_ag620_dump_register();
	default:
		ret = -EINVAL;
		break;
	};

	return ret;
}

/**
 * meas_ag620_get() - Get parameters of the ADC HAL driver.
 * @key:		Key to specify parameter(s) to get.
 * @params:		Structure holding list of parameters.
 *			Returns Error number; may be tested as boolean with
 *			0=success, other=fail. Some errors may need re-trial.
 */
static int meas_ag620_get(enum adc_hal_get_key key,
				union adc_hal_get_params params)
{
	int ret = 0;

	switch (key) {
	case ADC_HAL_GET_POWER_MODE:
		*params.p_power_mode = meas_ag620_state.power_mode;
		break;
	case ADC_HAL_GET_ADC_MEAS:{
			/* Get measurement and copy value */
			ret = meas_ag620_get_meas(params.adc_meas.channel,
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
 * meas_ag620_irq_handler - MEAS HW irq handler. It releases the "wait for
 *			completion" to process measurement done.
 * @irq:		irq number, should match IRQ number from device data.
 * @data:		data passed to the irq.
 */
static irqreturn_t meas_ag620_irq_handler(int irq, void *data)
{
	struct meas_ag620_state_data *st = (struct meas_ag620_state_data *)data;

	BUG_ON(st->irq_num != irq);

	/* First of all, clear the IRQ bit to avoid other interrupts */
	meas_ag620_irq_enable(false);

	/* Signal to release measurement done completion */
	complete(&st->meas_done);

	return IRQ_HANDLED;
}

/* Scaling info table for the AG620 */
static struct adc_hal_current_scaling_info meas_ag620_current_scaling_info = {
	.high_threshold_uv = 700000,
	.low_threshold_uv = 300000,
	.p_current_table = meas_ag620_current_table,
	.table_size = ARRAY_SIZE(meas_ag620_current_table)
};

static struct adc_hal_hw_info meas_ag620_hw_info = {
	.p_current_scaling_info = &meas_ag620_current_scaling_info,
	.p_hw_name = "AGOLD 620 MEAS HAL",
};

/* The final structure to register into the ADC layer */
static struct adc_hal_interface meas_ag620_adc_hal_interface = {
	.set = meas_ag620_set,
	.get = meas_ag620_get,
	.hw_info = &meas_ag620_hw_info,
};

#ifdef CONFIG_OF
static struct intel_adc_hal_channel_data *channel_data;
static struct intel_adc_hal_channel_data*
meas_ag620_get_of_channel_data(struct device_node *np)
{
	struct device_node *child;
	struct intel_adc_hal_channel_data *ch_data;
	char prefix[] = "hal-channel";
	int count = of_get_child_count(np);
	unsigned long index;

	ch_data = (struct intel_adc_hal_channel_data *)
		kcalloc(count, sizeof(*ch_data), GFP_KERNEL);
	if (!ch_data)
		return (void *)-ENOMEM;

	index = 0;
	for_each_child_of_node(np, child) {
		if (kstrtoul(&child->name[sizeof(prefix)-1], 10, &index))
			goto err_exit;

		ch_data[index].consumer_dev_name = "intel_adc_sensors";
		ch_data[index].autoscaling_on = of_property_read_bool(
			child, "adc,autoscaling-on");
		if (of_property_read_string(
				child, "adc,consumer-channel",
				&ch_data[index].consumer_channel) ||
			 of_property_read_string(
				 child, "adc,datasheet-name",
				 &ch_data[index].datasheet_name) ||
			 of_property_read_u32(
				 child, "adc,log-channel-id",
				 &ch_data[index].log_channel_id) ||
			 of_property_read_u32(
				 child, "adc,phy-channel-num",
				 &ch_data[index].phy_channel_num) ||
			 of_property_read_u32(
				 child, "adc,adc-bias-na",
				 &ch_data[index].adc_bias_na) ||
			 of_property_read_u32(
				 child, "adc,max-signal-settling-time-ms",
				 &ch_data[index].max_signal_settling_time_ms) ||
			 of_property_read_u32(
				 child, "adc,average-sample",
				 &ch_data[index].average_sample))
			goto err_exit;
	}
	return ch_data;

err_exit:
	kfree(ch_data);
	return (void *)-EINVAL;
}
#else  /* CONFIG_OF */
static struct intel_adc_hal_channel_data channel_data[] = {
	{"intel_adc_sensors", "VBAT_ADC", "AG620_M11_TYP", ADC_V_BAT_TYP,
	ADC_PHY_M11, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM},

	{"intel_adc_sensors", "VBAT_MIN_ADC", "AG620_M11_MIN", ADC_V_BAT_MIN,
	ADC_PHY_M11, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM},

	{"intel_adc_sensors", "VBAT_OCV_ADC", "AG620_M11_OCV", ADC_V_BAT_OCV,
	ADC_PHY_M11, false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	ADC_HAL_AVG_SAMPLE_LEVEL_HIGH},

	{"intel_adc_sensors", "BATTEMP0_ADC", "AG620_M1", ADC_T_BAT_0,
	ADC_PHY_M1, true, 4500, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM},

	{"intel_adc_sensors", "BATID_ADC", "AG620_M0", ADC_ID_BAT, ADC_PHY_M0,
	true, 4500, 500, ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM},

	{"intel_adc_sensors", "PMICTEMP_ADC", "AG620_M13",
	ADC_T_PMIC_IC_0, ADC_PHY_M13, false, 0,
	ADC_HAL_SIGNAL_SETTLING_DISABLED, ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM},

	{"intel_adc_sensors", "ANAMON_ADC", "AG620_M6", ADC_ANAMON, ADC_PHY_M6,
	false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM},

	{"intel_adc_sensors", "ACCID_ADC", "AG620_M4", ADC_ID_ACC, ADC_PHY_M4,
	false, 0, ADC_HAL_SIGNAL_SETTLING_DISABLED,
	ADC_HAL_AVG_SAMPLE_LEVEL_MEDIUM},
};
#endif	/* CONFIG_OF */

static ssize_t dbg_logs_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	size_t size_copied;
	int value;

	value = meas_ag620_debug_data.printk_logs_en;
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
	if (ret != 0)
		return ret;

	sysfs_val = (sysfs_val == 0) ? 0 : 1;

	meas_ag620_debug_data.printk_logs_en = sysfs_val;

	pr_info("sysfs attr %s=%d\n", attr->attr.name, sysfs_val);

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
 * meas_ag620_setup_sysfs_attr	Sets up dbg_logs_on_off sysfs entry
 *				for MEAS idi device
 *
 * @dev				[in] pointer to device structure structure
 */
static void meas_ag620_setup_sysfs_attr(struct device *dev)
{
	int err;

	err = device_create_file(dev, &dbg_logs_on_off_attr);
	if (err)
		pr_err("Unable to create sysfs entry: '%s'\n",
			dbg_logs_on_off_attr.attr.name);
}

/**
 * meas_ag620_probe() - The function that starts it all: it maps the registers,
 * initializes the hardware, request interrupts and registers to its logical
 * layer.
 *
 * @ididev:		A pointer to the IDI device.
 * @id:			A handle to the device id.
 */
static int meas_ag620_probe(struct idi_peripheral_device *ididev,
				const struct idi_device_id *id)
{
	struct device *dev = &ididev->device;
	struct intel_adc_hal_pdata *pdata;
#ifndef CONFIG_OF
	struct idi_resource *p_res = &ididev->resources;
#endif
	struct resource *p_resource;
	struct device_node *np = ididev->device.of_node;
	struct adc_hal_channel_info *p_channel_info;
	int ret, chan;
	u32 period_s = 0;

	dev_dbg(dev, "Initializing MEAS AG620 HAL\n");

	meas_ag620_state.ididev = ididev;

#ifdef CONFIG_OF
	pdata = kzalloc(sizeof(struct intel_adc_hal_pdata), GFP_KERNEL);
	ret = of_property_read_u32(np,
			"intel,calibration_period_s",
			&period_s);

	if (ret) {
		dev_err(&ididev->device, "calibration period not found!\n");
		goto err_exit;
	}

	meas_ag620_state.calibration.period_s = period_s;

	/* Force calibration before the first measurement. */
	meas_ag620_state.calibration.last_timestamp_s = 0;

	channel_data = meas_ag620_get_of_channel_data(np);
	if (IS_ERR(channel_data)) {
		dev_err(&ididev->device,
				"get channel data error(%ld)!\n",
				PTR_ERR(channel_data));
		goto err_exit;
	}

	pdata->channel_info.nchan = of_get_child_count(np);

	dev_set_drvdata(dev, meas_ag620_state.regmap);
#else
	pdata = dev->platform_data;
	meas_ag620_state.calibration.period_s = pdata->calibration_period_s;
	pdata->channel_info.nchan = ARRAY_SIZE(channel_data);
#endif

	BUG_ON(meas_ag620_state.calibration.period_s == 0);

	/* Force calibration before the first measurement. */
	meas_ag620_state.calibration.last_timestamp_s = 0;

	dev_set_drvdata(dev, meas_ag620_state.regmap);

	/* Get IRQ number to use */
	BUG_ON(NULL ==
		(p_resource = idi_get_resource_byname(&ididev->resources,
						IORESOURCE_IRQ, "meas_irq")));
	meas_ag620_state.irq_num = p_resource->start;

	/* Get Peripheral address range */
	BUG_ON(NULL ==
		(p_resource = idi_get_resource_byname(&ididev->resources,
						IORESOURCE_MEM, "meas_reg")));
	meas_ag620_state.regmap =
		ioremap(p_resource->start, resource_size(p_resource));
	if (!meas_ag620_state.regmap) {
		dev_err(&ididev->device, "Unable to remap MEAS registers!\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	ret =  idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
			__func__);
		return -EINVAL;
	}

	pr_info("%s: getting handlers for dev pm states...\n", __func__);

	meas_ag620_state.pm_state_en =
		idi_peripheral_device_pm_get_state_handler(ididev, "enable");
	if (meas_ag620_state.pm_state_en == NULL) {
		pr_err("%s: Unable to get handler for PM state 'enable'!\n",
			__func__);
		ret = -EINVAL;
		goto err_dev_pm;
	}

	meas_ag620_state.pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(ididev, "disable");
	if (meas_ag620_state.pm_state_dis == NULL) {
		pr_err("%s: Unable to get handler for PM state 'disable'!\n",
			__func__);
		ret = -EINVAL;
		goto err_dev_pm;
	}

	/* Enable power domain */
	BUG_ON(0 != idi_set_power_state(meas_ag620_state.ididev,
					meas_ag620_state.pm_state_en, true));

	/* Request and enable the MEAS irq */
	ret = request_irq(meas_ag620_state.irq_num, meas_ag620_irq_handler,
			IRQF_TRIGGER_RISING, "meas_ag620", &meas_ag620_state);
	if (ret) {
		dev_err(&ididev->device, "Unable to register irq %d\n",
			meas_ag620_state.irq_num);
		goto err_request_irq;
	}
	pdata->channel_info.p_data = channel_data;
	p_channel_info = (struct adc_hal_channel_info *) &pdata->channel_info;
	dev->platform_data = pdata;
	meas_ag620_irq_enable(false);

	/* Disable power domain */
	WARN_ON(0 != idi_set_power_state(meas_ag620_state.ididev,
					meas_ag620_state.pm_state_dis, false));

	/* Store device data in a convenient form for later use */
	for (chan = 0; chan < p_channel_info->nchan; chan++) {
		meas_ag620_state.p_channel_data[p_channel_info->p_data[chan].
						log_channel_id] =
			&p_channel_info->p_data[chan];
	}

	/* Set up timer used for peak detecting */
	setup_timer(&meas_ag620_state.pd_timer, meas_ag620_peak_detect_timeout,
			0);

	/* Initialiase measurement done completion */
	init_completion(&meas_ag620_state.meas_done);

	/* Register to HAL and return */
	ret = adc_register_hal(&meas_ag620_adc_hal_interface, dev,
				&meas_ag620_state.operation_done_cb);
	if (ret)
		goto err_register_hal;

	meas_ag620_setup_sysfs_attr(&ididev->device);

	return 0;

err_register_hal:
	free_irq(meas_ag620_state.irq_num, &meas_ag620_state);
err_dev_pm:
err_request_irq:
	dev_set_drvdata(&ididev->device, NULL);
	iounmap(meas_ag620_state.regmap);
	meas_ag620_state.regmap = NULL;
err_exit:
	meas_ag620_state.ididev = NULL;
	return ret;
}

/**
 * meas_ag620_remove() - The function that stops the device/driver, de-registers
 * from its logical layer and releases all associated resources
 *
 * @ididev:	A pointer to the IDI device.
 */
static int meas_ag620_remove(struct idi_peripheral_device *ididev)
{
	struct device *dev = &ididev->device;

	void *registers = dev_get_drvdata(dev);

	adc_unregister_hal(&meas_ag620_adc_hal_interface, dev);

	del_timer_sync(&meas_ag620_state.pd_timer);

	dev_set_drvdata(dev, NULL);

	iounmap(registers);

	return 0;
}

/**
 * meas_ag620_suspend() - Called when the system is attempting to suspend.
 * If a measurement is in progress EBUSY is returned to abort the suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	EBUSY if a measurement is ongoing, else 0
 */
static int meas_ag620_suspend(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Protect critical section when testing and modifying device state
	data */
	spin_lock(&meas_ag620_state.lock);

	/* If there is a a measurement in progess, prevent suspend. */
	if (ADC_HAL_POWER_MODE_ON == meas_ag620_state.power_mode) {

		/* End of critical section */
		spin_unlock(&meas_ag620_state.lock);

		MEAS_AG620_DEBUG_DATA_LOG(MEAS_AG620_SUSPEND_EBUSY, 0, 0, 0, 0);
		return -EBUSY;
	} else {
		/* No measurement ongoing - allow suspend. */
		meas_ag620_state.suspended = true;

		/* End of critical section */
		spin_unlock(&meas_ag620_state.lock);

		MEAS_AG620_DEBUG_DATA_LOG(MEAS_AG620_SUSPEND_OK, 0, 0, 0, 0);
		return 0;
	}
}

/**
 * meas_ag620_resume() - Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int meas_ag620_resume(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Nothing to do here */
	MEAS_AG620_DEBUG_DATA_LOG(MEAS_AG620_RESUME, 0, 0, 0, 0);

	/* Update suspend flag used to tell whether operations on device are
	allowed */
	meas_ag620_state.suspended = false;
	return 0;
}

const struct dev_pm_ops meas_ag620_pm = {
	.suspend = meas_ag620_suspend,
	.resume_early = meas_ag620_resume,
};

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_MEAS,
	},

	{/* end: all zeroes */},
};

static struct idi_peripheral_driver meas_ag620_idiperdrv = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ag620_meas",
		.pm = &meas_ag620_pm,
	},
	.p_type = IDI_MEAS,
	.id_table = idi_ids,
	.probe = meas_ag620_probe,
	.remove = meas_ag620_remove,
};

MODULE_DEVICE_TABLE(idi, idi_ids);

static int __init meas_ag620_init(void)
{
	int ret;

	spin_lock_init(&meas_ag620_debug_data.lock);
	spin_lock_init(&meas_ag620_state.lock);
	ret = idi_register_peripheral_driver(&meas_ag620_idiperdrv);
	/* Do debug data logging */
	MEAS_AG620_DEBUG_DATA_LOG(MEAS_AG620_INIT, ret, 0, 0, 0);
	return ret;
}

static void __exit meas_ag620_exit(void)
{
	idi_unregister_peripheral_driver(&meas_ag620_idiperdrv);
	/* Do debug data logging */
	MEAS_AG620_DEBUG_DATA_LOG(MEAS_AG620_DE_INIT, 0, 0, 0, 0);
}

module_init(meas_ag620_init);
module_exit(meas_ag620_exit);

MODULE_LICENSE("GPL v2");
