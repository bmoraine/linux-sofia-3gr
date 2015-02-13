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

#define DRIVER_NAME					"ag620_swfgh"
#define pr_fmt(fmt) DRIVER_NAME": "fmt
#include <linux/device.h>
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/power/battery_id.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>

#include <linux/power/sw_fuel_gauge_debug.h>
#include <linux/power/sw_fuel_gauge_hal.h>
#include <linux/power/sw_fuel_gauge_platform.h>

#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/idi/idi_ids.h>
#include <linux/alarmtimer.h>
#include <linux/hrtimer.h>

/* Address of coulomb counter UP register. */
#define PMU_AG620_CC_UP_CNT					(0x00)
/* Address of coulomb counter DOWN register. */
#define PMU_AG620_CC_DOWN_CNT					(0x04)
/* Address of battery current register. */
#define PMU_AG620_CC_BAT_CURRENT				(0x08)
/* Address of coulomb counter CTRL register. */
#define PMU_AG620_CC_CTRL					(0x0C)
/* Address of coulomb counter ACC register. */
#define PMU_AG620_CC_ACC					(0x10)
/* Address of coulomb counter THR register. */
#define PMU_AG620_CC_THR					(0x14)
/* Address of coulomb counter CTRL2 register. */
#define PMU_AG620_CC_CTRL2					(0x18)

/* Coulomb counter CTRL register setting. */
/* Enable the coulomb counter with asymmetrical chopping */
#define PMU_AG620_CC_CTRL_ENABLE				(0x20004)
/* Coulomb counter CTRL2 register setting. */
#define PMU_AG620_CC_CTRL2_CLEAR				(0x00)
/* Bitfield mask for positive part of 13 bit battery current field. */
#define PMU_AG620_CC_CBAT_POS_MASK				(0xFFF)
/* Bitfield mask for negative part of 13 bit battery current field. */
#define PMU_AG620_CC_CBAT_NEG_MASK				(0x1000)
/* Maximum value of CC_THR register. 8 bit for AG620 ES2_0 */
#define PMU_AG620_ES2_CC_THR_LIMIT				(0xFF)

/* Coulomb Scaling factor from C to uC value. */
#define SCALING_C_TO_UC						(1000000)
/* Coulomb Scaling factor from uC to mC value. */
#define SCALING_UC_TO_MC					(1000)
/* Scaling factor from uA to mA. */
#define SCALING_UA_TO_MA					(1000)
/* Scaling factor from uS to S. */
#define SCALING_US_TO_S						(1000000)

/* Theoretical trigger level for coulomb counter increment in mV. */
#define COULOMB_COUNTER_INCREMENT_THRESHOLD_THEORETICAL_MV	(500)
/* Average error in trigger level for coulomb counter increment in mV. */
#define COULOMB_COUNTER_INCREMENT_THRESHOLD_COMPENSATION_MV_ES2	(0)
/* Error compensated trigger level for coulomb counter increment in mV. */
#define COULOMB_COUNTER_INCREMENT_THRESHOLD_MV_ES2 \
	(COULOMB_COUNTER_INCREMENT_THRESHOLD_THEORETICAL_MV + \
		COULOMB_COUNTER_INCREMENT_THRESHOLD_COMPENSATION_MV_ES2)
/* Sampling frequency of coulomb counter integrator in Hz. */
#define COULOMB_COUNTER_CLOCK_FREQ_HZ				(8192)
/* Delta coulomb counter threshold scaling factor,
according to the fixed hw divider before the cc
threshold comparator */
#define COULOMB_COUNTER_DELTA_THRESHOLD_SCALING_ES2		(512)

/* Minimum period required for a long term average Ibat measurement. */
#define IBAT_LONG_TERM_AVERAGE_MIN_PERIOD_SECS			(300)

/* Polling period long term average Ibat measurement. */
#define IBAT_LONG_TERM_AVERAGE_PERIOD_SECS			(600)
/* long term average Ibat period error margin in percent. */
#define IBAT_LONG_TERM_AVERAGE_ERROR_MARGIN_PERCENT		(10)
/* Timer polling period for long term average Ibat in Secs. */
#define IBAT_LONG_TERM_AVERAGE_POLLING_PERIOD_SECS_ES2 \
			(IBAT_LONG_TERM_AVERAGE_PERIOD_SECS +\
			((IBAT_LONG_TERM_AVERAGE_PERIOD_SECS\
			* IBAT_LONG_TERM_AVERAGE_ERROR_MARGIN_PERCENT) / 100))

/* Macro to trace and log debug event and data. */
#define SW_FUEL_GAUGE_HAL_DEBUG_PARAM(_event, _param) \
	SWFG_DEBUG(sw_fuel_gauge_hal_debug_data, _event, _param)

/* Macro to trace and log debug event without a parameter. */
#define SW_FUEL_GAUGE_HAL_DEBUG_NO_PARAM(_event) \
	SWFG_DEBUG(sw_fuel_gauge_hal_debug_data, _event, 0)

/* Macro to trace and log debug event without a parameter or printk. */
#define SW_FUEL_GAUGE_HAL_DEBUG_NO_LOG_NO_PARAM(_event) \
	SWFG_DEBUG_NO_PRINTK(sw_fuel_gauge_hal_debug_data, _event, 0)

#define sw_fg_hal_set_pm_state(_idi_dev, _pm_state, _en) \
do {\
	int __ret;\
	__ret = idi_set_power_state(_idi_dev, _pm_state, _en);\
\
	if (__ret) {\
		pr_err("%s: setting PM state '%s' failed!\n",\
			__FILE__, _pm_state->name);\
		if (_en)\
			BUG();\
	} \
} while (0)

#define SET_ALARM_TIMER(__alarm, tout_delay) \
	do {\
		struct timespec __delta;\
		__delta.tv_sec = tout_delay;\
		__delta.tv_nsec = 0;\
		alarm_start_relative(__alarm, timespec_to_ktime(__delta));\
	} while (0)

/* Ibat average reference point structure. */
struct ibat_avg_ref_element {
	/* Monotonic system time stamp in seconds. */
	u32 time_secs;
	/* Coulomb counter IN count. */
	u32 cc_up;
	/* Coulomb counter OUT count. */
	u32 cc_down;
};

/* Long term Ibat current average structure. */
struct ibat_long_term_average {
	/* Coulomb counter filter. */
	struct ibat_avg_ref_element ibat_avg_reference;
	/* Coulomb counter filter. */
	struct ibat_avg_ref_element ibat_avg_next_reference;
};

/* SW Fuel Gauge Hal control structure */
struct sw_fuel_gauge_hal_data {
	/* SW Fuel Gauge callback functions. */
	struct sw_fuel_gauge_interface *p_sw_fuel_gauge;
	/* pointer to idi device. */
	struct idi_peripheral_device *p_idi_device;

	struct device_state_pm_state *pm_state_en;
	struct device_state_pm_state *pm_state_dis;

	/* pointer to platform configuration parameters. */
	struct sw_fuel_gauge_platform_data platform_data;
	/* Wake lock to prevent suspend in critical sections. */
	struct wake_lock suspend_lock;
	/* PMU Coulomb Counter IO resource */
	struct resource *pmu_cc_res;
	/* Base timestamp for accumulated error. */
	time_t error_base_rtc_sec;
	/* Base count for accumulated error in charge IN to battery. */
	u32 error_base_cc_up;
	/* Base count for accumulated error in charge OUT of battery. */
	u32 error_base_cc_down;
	/* Coulomb delta threshold currently set (mC) */
	int delta_threshold_mc;
	/* true after coulomb delta threshold has been set. */
	bool delta_threshold_set;
	/* Scaling factor from counts to mC for delta threshold */
	int threshold_count_scaling_mc;
	/* Scaling factor from UP and DOWN counts to uC */
	int coulomb_count_scaling_uc;
	/* Alarm timer to regularly poll coulomb counter for IBAT long term
	average. */
	struct alarm ibat_long_term_average_polling_atimer;
	/* Ibat long term average data. */
	struct ibat_long_term_average ibat_long_term_average;
	/* Coulomb counter interrupt */
	int irq;
};

static unsigned pmu_ioread(struct sw_fuel_gauge_hal_data *sw_fg_hal,
					unsigned offset)
{
	int ret = 0;
	unsigned reg;
	unsigned addr;

	if (unlikely(sw_fg_hal == NULL))
		BUG();

	addr = sw_fg_hal->pmu_cc_res->start + offset;
	ret = idi_client_ioread(sw_fg_hal->p_idi_device, addr, &reg);
	if (ret)
		BUG();

	return reg;
}

static void pmu_iowrite(struct sw_fuel_gauge_hal_data *sw_fg_hal,
					unsigned offset,
					unsigned data)
{
	int ret = 0;
	unsigned addr;

	if (unlikely(sw_fg_hal == NULL))
		BUG();

	addr = sw_fg_hal->pmu_cc_res->start + offset;
	ret = idi_client_iowrite(sw_fg_hal->p_idi_device, addr, data);
	if (ret)
		BUG();
}


/**
 * Refer to documentation in <sw_fuel_gauge_hal.h>
 */
static int sw_fuel_gauge_hal_set(enum sw_fuel_gauge_hal_set_key key,
				union sw_fuel_gauge_hal_set_params params);

/**
 * Refer to documentation in <sw_fuel_gauge_hal.h>
 */
static int sw_fuel_gauge_hal_get(enum sw_fuel_gauge_hal_get_key key,
				union sw_fuel_gauge_hal_get_params *p_params);

/**
 * SW Fuel Gauge Hal exported interface.
 * Refer to documentation in <sw_fuel_gauge_hal.h>
 */
static struct sw_fuel_gauge_hal_interface ag620_sw_fuel_gauge_hal = {
	sw_fuel_gauge_hal_set,	/* set */
	sw_fuel_gauge_hal_get,	/* get */
};

/* SW Fuel Gauge Hal instance */
static struct sw_fuel_gauge_hal_data sw_fuel_gauge_hal_instance;

/* Array to collect debug data */
static struct sw_fuel_gauge_debug_data sw_fuel_gauge_hal_debug_data;

/**
 * sw_fuel_gauge_hal_get_coulomb_counts - Read the raw couloumb counter values.
 *
 * @cc_up_counts	[out] Raw value of coulomb UP counter in counts.
 * @cc_down_counts	[out] Raw value of coulomb DOWN counter in counts.
 */
static void sw_fuel_gauge_hal_get_coulomb_counts(u32 *cc_up_counts,
						u32 *cc_down_counts)
{
	struct sw_fuel_gauge_hal_data *sw_fg_hal = &sw_fuel_gauge_hal_instance;

	/* Obtain Wake Lock to ensure that both registers are read at the same
	time. */
	wake_lock(&sw_fg_hal->suspend_lock);

	/* Read registers and return values to the caller. */
	sw_fg_hal_set_pm_state(sw_fg_hal->p_idi_device,
				sw_fg_hal->pm_state_en, true);

	*cc_up_counts = pmu_ioread(sw_fg_hal, PMU_AG620_CC_UP_CNT);
	*cc_down_counts = pmu_ioread(sw_fg_hal, PMU_AG620_CC_DOWN_CNT);

	sw_fg_hal_set_pm_state(sw_fg_hal->p_idi_device,
				sw_fg_hal->pm_state_dis, false);

	/* End of critical section. Release Wake Lock. */
	wake_unlock(&sw_fg_hal->suspend_lock);

	SW_FUEL_GAUGE_HAL_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_CC_UP_COUNTS,
							*cc_up_counts);
	SW_FUEL_GAUGE_HAL_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_CC_DOWN_COUNTS,
							*cc_down_counts);
}

/**
 * sw_fuel_gauge_hal_counts_to_mc - Convert raw couloumb counter values to mC.
 *
 * NOTE: The HW counters are unsigned 32-bit values. Due to the possible
 * non-integer nature of the scaling factor when expressed in mC, uC are used
 * internally to maintain accuracy. This then requires 64-bit arithmetic to
 * prevent arithmetic overflow.
 *
 * Returning a signed 32-bit quantity (which may not represent the full
 * range of the HW) is deemed acceptable.
 *
 * Example:
 * Each count corresponds to 3.051 mC, giving a maximum value
 * of (2^32) x 3.051 mC, or approximately 13,000,000,000 mC.
 * In mAh: over 3,600,00 - which is currently almost 1000 x the size of a large
 * battery.
 *
 * The signed 32 bit return still allows for peak values
 * of over +/- 2x10^9 or nearly 600000 mAh.
 *
 * @cc_counts	[in] Raw value of coulomb count.
 * Returns:	Coulomb count in mC.
 */
static int sw_fuel_gauge_hal_counts_to_mc(int cc_counts)
{
	/* Scale the HW count value to uC using 64-bit arithmetic to maintain
	accuracy. */
	s64 result_uc_64 =
		(s64) cc_counts *
		(s64) sw_fuel_gauge_hal_instance.coulomb_count_scaling_uc;

	s32 remainder;
	/* Truncate to 32-bit signed mC value for result.
	NOTE: 64 Division is not supported with the standard C operator */
	return (int)div_s64_rem(result_uc_64, SCALING_UC_TO_MC, &remainder);
}

/**
 * sw_fuel_gauge_hal_read_coulomb_counter - Read the HW and return the requested
 * coulomb counter value converted to mC.
 *
 * @value_to_read	[in] Specifies which coulomb count to read.
 * Returns:		Signed coulomb count in mC.
 */
static int sw_fuel_gauge_hal_read_coulomb_counter(enum sw_fuel_gauge_hal_get_key
								value_to_read)
{
	u32 cc_up;
	u32 cc_down;
	int result_mc;

	/* Get the raw coulomb counter values from the HW . */
	sw_fuel_gauge_hal_get_coulomb_counts(&cc_up, &cc_down);

	switch (value_to_read) {
	case SW_FUEL_GAUGE_HAL_GET_CC_BALANCED_COUNT:
		/* Calculate balanced value. In AG620_ES2_0,the coulomb counter
		assumes positive IBAT when charging (according to the datasheet,
		this is correct). Our design works on the premise that the IBAT
		is negative when charging, so the sign is inverted by
		subtracting the negative minus the positive. */
		result_mc = sw_fuel_gauge_hal_counts_to_mc(
						cc_down - cc_up);

		SW_FUEL_GAUGE_HAL_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_HAL_GET_BALANCED_CC_MC, result_mc);
		break;

	case SW_FUEL_GAUGE_HAL_GET_CC_UP_COUNT:
		/* Number of counts into the battery. */
		result_mc = sw_fuel_gauge_hal_counts_to_mc(cc_up);
		SW_FUEL_GAUGE_HAL_DEBUG_PARAM
			(SW_FUEL_GAUGE_DEBUG_HAL_GET_CC_UP_MC, result_mc);
		break;

	case SW_FUEL_GAUGE_HAL_GET_CC_DOWN_COUNT:
		/* Number of counts out of the battery. */
		result_mc = sw_fuel_gauge_hal_counts_to_mc(cc_down);
		SW_FUEL_GAUGE_HAL_DEBUG_PARAM
			(SW_FUEL_GAUGE_DEBUG_HAL_GET_CC_DOWN_MC, result_mc);
		break;

	case SW_FUEL_GAUGE_HAL_GET_CC_ACCUMULATED_ERROR:
	case SW_FUEL_GAUGE_HAL_GET_COULOMB_IND_DELTA_THRESHOLD:
	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE:
	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE:
		/* Fall through to BUG() intentional */
	default:
		/* Invalid read parameter. */
		BUG();
		break;
	}
	return result_mc;
}

/**
 * sw_fuel_gauge_hal_read_battery_current - Read the HW register and return the
 * 1 second average battery current measured by the coulomb counter.
 *
 * Returns:		Battery current in mA.
 */
static int sw_fuel_gauge_hal_read_battery_current(void)
{
	int ibat_positive;
	int ibat_negative;
	int ibat_count_signed;
	int ibat_ma;
	u32 ibat_reg;
	struct sw_fuel_gauge_hal_data *sw_fg_hal = &sw_fuel_gauge_hal_instance;

	sw_fg_hal_set_pm_state(sw_fg_hal->p_idi_device,
					sw_fg_hal->pm_state_en, true);

	ibat_reg = pmu_ioread(sw_fg_hal, PMU_AG620_CC_BAT_CURRENT);

	sw_fg_hal_set_pm_state(sw_fg_hal->p_idi_device,
					sw_fg_hal->pm_state_dis, false);

	/* Extract two's complement fields from register value. */
	ibat_positive = ibat_reg & PMU_AG620_CC_CBAT_POS_MASK;
	ibat_negative = ibat_reg & PMU_AG620_CC_CBAT_NEG_MASK;

	/* Calculate signed count value. In AG620_ES2_0,the coulomb counter
	assumes positive IBAT when charging (according to the datasheet, this is
	correct). Our design works on the premise that the IBAT is negative when
	charging, so the sign is inverted by subtracting the negative minus the
	positive. */
	ibat_count_signed = ibat_negative - ibat_positive;

	/* Return the HW count value scaled to mA. */
	ibat_ma =
		(int)((ibat_count_signed * sw_fg_hal->coulomb_count_scaling_uc)
			/ SCALING_UA_TO_MA);

	SW_FUEL_GAUGE_HAL_DEBUG_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_GET_IBAT_SHORT_AV_MA, ibat_ma);

	return ibat_ma;
}

/**
 * sw_fuel_gauge_hal_init_long_term_ibat_average_and_accumulated_error
 * Initialise the long term Ibat current reference points and the accumulated
 * error data.
 */
static void
sw_fuel_gauge_hal_init_long_term_ibat_average_and_accumulated_error(void)
{
	u32 cc_up;
	u32 cc_down;
	struct timespec time_now;

	/* Get timestamp for error and IBAT calculation. Elapsed real time is
	not affected by changes in the user displayed time and date */
	time_now = ktime_to_timespec(ktime_get_boottime());

	/* Read raw coulomb counters. */
	sw_fuel_gauge_hal_get_coulomb_counts(&cc_up, &cc_down);
	/* Set the reference base for IBAT long term average calculations. */
	sw_fuel_gauge_hal_instance.ibat_long_term_average.ibat_avg_reference.
		cc_up = cc_up;
	sw_fuel_gauge_hal_instance.ibat_long_term_average.ibat_avg_reference.
		cc_down = cc_down;
	sw_fuel_gauge_hal_instance.ibat_long_term_average.ibat_avg_reference.
		time_secs = time_now.tv_sec;

	sw_fuel_gauge_hal_instance.ibat_long_term_average.
		ibat_avg_next_reference =
			sw_fuel_gauge_hal_instance.ibat_long_term_average.
							ibat_avg_reference;

	/* Set the reference base for accumulated error calculations. */
	sw_fuel_gauge_hal_instance.error_base_cc_up = cc_up;
	sw_fuel_gauge_hal_instance.error_base_cc_down = cc_down;
	sw_fuel_gauge_hal_instance.error_base_rtc_sec = time_now.tv_sec;
}

/**
 * sw_fuel_gauge_hal_update_long_term_ibat_average -
 * Add a timestamped coulomb counter reading to the long term battery current
 * reference points.
 */
static void sw_fuel_gauge_hal_update_long_term_ibat_average(void)
{
	u32 ibat_avg_next_reference_delta_t_secs;
	struct timespec time_now;
	struct ibat_avg_ref_element *p_ref =
		&sw_fuel_gauge_hal_instance.ibat_long_term_average.
							ibat_avg_reference;
	struct ibat_avg_ref_element *p_next =
		&sw_fuel_gauge_hal_instance.ibat_long_term_average.
						ibat_avg_next_reference;

	/* Elapsed real time is not affected by changes in the user displayed
	time and date */
	time_now = ktime_to_timespec(ktime_get_boottime());
	ibat_avg_next_reference_delta_t_secs =
		time_now.tv_sec - p_next->time_secs;

	if (ibat_avg_next_reference_delta_t_secs >=
		IBAT_LONG_TERM_AVERAGE_MIN_PERIOD_SECS) {
		/* Enough time has passed to update the reference point. */
		*p_ref = *p_next;
		/* Read and store coulomb counters in next reference point with
		timestamp. */
		p_next->time_secs = time_now.tv_sec;
		sw_fuel_gauge_hal_get_coulomb_counts(&p_next->cc_up,
							&p_next->cc_down);

		SW_FUEL_GAUGE_HAL_DEBUG_PARAM
			(SW_FUEL_GAUGE_DEBUG_HAL_UPDATE_IBAT_REF_SEC,
			 p_ref->time_secs);
		SW_FUEL_GAUGE_HAL_DEBUG_PARAM
			(SW_FUEL_GAUGE_DEBUG_HAL_UPDATE_IBAT_NEXT_SEC,
			 p_next->time_secs);
	}
}

/**
 * sw_fuel_gauge_hal_process_timer_and_irq_work - Read the coulomb counter to
 * process the long term IBAT average and call back the SW Fuel Gauge. This
 * function is executed in the SW Fuel Gauge work thread when the delta
 * threshold IRQ happens or when the polling timer expires, to ensure periodic
 * updates to the system when coulomb counter interrupts are infrequent, e.g.
 * in deep sleep.
 *
 * @param	[in]	Generic parameter passed from scheduler queue. Not used.
 */
static void sw_fuel_gauge_hal_process_timer_and_irq_work(long param)
{
	struct timespec now;

	/* Generic parameter is not used here. */
	(void)param;

	/* Elapsed real time is not affected by changes in the user displayed
	time and date */
	now = ktime_to_timespec(ktime_get_boottime());

	SW_FUEL_GAUGE_HAL_DEBUG_NO_LOG_NO_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_PROCESS_IRQ_AND_TIMER_WORK);

	pr_debug("processing timer and irq work, timestamp=%ld\n", now.tv_sec);

	/* Update long term current average calculation with the current coulomb
	counter values. */
	sw_fuel_gauge_hal_update_long_term_ibat_average();

	/* If the SW Fuel Gauge has set a threshold, report the current state of
	the coulomb counter. */
	if (sw_fuel_gauge_hal_instance.delta_threshold_set) {

		union sw_fuel_gauge_hal_cb_param cb_param = {
			.cc_delta_mc =
				sw_fuel_gauge_hal_read_coulomb_counter
				(SW_FUEL_GAUGE_HAL_GET_CC_BALANCED_COUNT),
		};
		/* Threshold has been crossed, inform the sw fuel gauge. */
		sw_fuel_gauge_hal_instance.p_sw_fuel_gauge->
			event_cb(SW_FUEL_GAUGE_HAL_CB_EVENT_SOC_UPDATE,
								cb_param);
	}
	/* Calculate and set the period for the next timeout. */
	SET_ALARM_TIMER(&sw_fuel_gauge_hal_instance.
				ibat_long_term_average_polling_atimer,
				IBAT_LONG_TERM_AVERAGE_POLLING_PERIOD_SECS_ES2);
}

/**
 * sw_fuel_gauge_hal_polling_atimer_expired_cb - Schedules a work to read the
 * coulomb counter to update the IBAT average and call back the SW Fuel Gauge.
 * This function is triggered by the expiry of the polling timer.
 *
 * @alrm	[in]	parameter passed from timer. Not used in this case.
 *
 * Returns:		Whether alarm timer should restart.
 */
static enum alarmtimer_restart sw_fuel_gauge_hal_polling_atimer_expired_cb(
		struct alarm *alrm, ktime_t t)
{
	SW_FUEL_GAUGE_HAL_DEBUG_NO_LOG_NO_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_TIMER_EXPIRED);

	(void)alrm;

	/* Schedule the SW Fuel Gauge work thread to execute the polling
	function. */
	sw_fuel_gauge_hal_instance.p_sw_fuel_gauge->
		enqueue(sw_fuel_gauge_hal_process_timer_and_irq_work, 0);
	return ALARMTIMER_NORESTART;
}

/**
 * sw_fuel_gauge_hal_polling_timer_expired_cb - Schedules a work to read the
 * coulomb counter to update the IBAT average and call back the SW Fuel Gauge.
 * This function is triggered by the coulomb counter threshold crossed IRQ.
 *
 * @irq		[in] (not used)
 * @dev		[in] (not used)
 */
static irqreturn_t sw_fuel_gauge_hal_coulomb_counter_delta_irq_cb(int irq,
								void *dev)
{
	/* Unused */
	(void)irq;
	(void)dev;

	SW_FUEL_GAUGE_HAL_DEBUG_NO_LOG_NO_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_CC_DELTA_IRQ);

	/* Schedule the SW Fuel Gauge work thread to execute the polling
	function. */
	sw_fuel_gauge_hal_instance.p_sw_fuel_gauge->
		enqueue(sw_fuel_gauge_hal_process_timer_and_irq_work, 0);
	return IRQ_HANDLED;
}

/**
 * sw_fuel_gauge_hal_calc_long_ibat_average - Perform calculation of the long
 * term IBAT average load current.
 *
 * @p_ibat_average_ma	[out]	Long term (>5mins) Ibat average current in mA.
 * Returns:			0 If enough samples have been taken.
 *				-EAGAIN if not enough time has passed to
 *				calculate the average.
 */
static int sw_fuel_gauge_hal_calc_long_ibat_average(int *p_ibat_average_ma)
{
	int error = 0;

	struct ibat_avg_ref_element *p_ref =
		&sw_fuel_gauge_hal_instance.ibat_long_term_average.
							ibat_avg_reference;
	struct ibat_avg_ref_element *p_next =
		&sw_fuel_gauge_hal_instance.ibat_long_term_average.
						ibat_avg_next_reference;
	struct timespec time_now;

	s32 ibat_avg_next_reference_delta_t_secs;
	s32 ibat_avg_reference_delta_t_secs;

	/* Calculate time passed since reference points were made. Elapsed
	real time is not affected by changes in the user displayed time and
	date */
	time_now = ktime_to_timespec(ktime_get_boottime());
	ibat_avg_next_reference_delta_t_secs =
		time_now.tv_sec - p_next->time_secs;
	ibat_avg_reference_delta_t_secs = time_now.tv_sec - p_ref->time_secs;

	/* Check pointer to parameter. */
	BUG_ON(NULL == p_ibat_average_ma);

	/* Enough time has passed to use the reference point. */
	if (ibat_avg_reference_delta_t_secs >=
		IBAT_LONG_TERM_AVERAGE_MIN_PERIOD_SECS) {
		u32 cc_up;
		u32 cc_down;
		/* Balanced count must be signed. */
		int cc_delta_balanced_mc;
		/* Read current coulomb counter values. */
		sw_fuel_gauge_hal_get_coulomb_counts(&cc_up, &cc_down);

		/* Check if enough time has passed to update the reference
		point. */
		if (ibat_avg_next_reference_delta_t_secs >=
			IBAT_LONG_TERM_AVERAGE_MIN_PERIOD_SECS) {
			/* Enough time has passed to update the reference
			point. */
			*p_ref = *p_next;
			p_next->cc_down = cc_down;
			p_next->cc_up = cc_up;
			p_next->time_secs = time_now.tv_sec;
			ibat_avg_reference_delta_t_secs =
				ibat_avg_next_reference_delta_t_secs;
			SW_FUEL_GAUGE_HAL_DEBUG_PARAM
				(SW_FUEL_GAUGE_DEBUG_HAL_UPDATE_IBAT_REF_SEC,
				 p_ref->time_secs);
			SW_FUEL_GAUGE_HAL_DEBUG_PARAM
				(SW_FUEL_GAUGE_DEBUG_HAL_UPDATE_IBAT_NEXT_SEC,
				 p_next->time_secs);
		}
		/* Calculate coulomb counter difference over average period. */
		cc_delta_balanced_mc = sw_fuel_gauge_hal_counts_to_mc((s32)
					((u32)(cc_down - p_ref->cc_down)-
					(u32)(cc_up - p_ref->cc_up)));


		/* Result in mA is mC per second. */
		*p_ibat_average_ma =
			cc_delta_balanced_mc / ibat_avg_reference_delta_t_secs;
		SW_FUEL_GAUGE_HAL_DEBUG_PARAM
			(SW_FUEL_GAUGE_DEBUG_HAL_IBAT_LONG_SEC,
			 ibat_avg_reference_delta_t_secs);
		SW_FUEL_GAUGE_HAL_DEBUG_PARAM
			(SW_FUEL_GAUGE_DEBUG_HAL_GET_IBAT_LONG_AV_MA,
			 *p_ibat_average_ma);

	} else {
		error = -EAGAIN;
		SW_FUEL_GAUGE_HAL_DEBUG_NO_PARAM
			(SW_FUEL_GAUGE_DEBUG_HAL_GET_IBAT_LONG_AV_FAILED);
	}
	return error;
}

/**
 * sw_fuel_gauge_hal_calc_accumulated_error - Perform calculation of accumulated
 * error in coulomb counter since last error reset.
 *
 * Returns: Accumulated coulomb count error in mC.
 */
static int sw_fuel_gauge_hal_calc_accumulated_error(void)
{
	u32 cc_up;
	u32 cc_down;
	int cc_delta_up_mc;
	int cc_delta_down_mc;
	/* 64 Bit arithmetic used to maintain accuracy in uC calculations. */
	s64 error_uc;
	int error_mc;
	time_t error_period_sec;
	struct timespec rtc_time;
	int remainder;

	/* Get timestamp. Elapsed real time is not affected by changes in the
	user displayed time and date */
	rtc_time = ktime_to_timespec(ktime_get_boottime());

	/* Calculate the time elapsed since the last error reset. */
	error_period_sec =
		rtc_time.tv_sec - sw_fuel_gauge_hal_instance.error_base_rtc_sec;

	/* Read the raw coulomb counter values. */
	sw_fuel_gauge_hal_get_coulomb_counts(&cc_up, &cc_down);

	/* Calculate coulomb counter differences in mC over the period since
	last error reset. */
	cc_delta_up_mc = sw_fuel_gauge_hal_counts_to_mc((u32)
				(cc_up - sw_fuel_gauge_hal_instance.
						error_base_cc_up));
	cc_delta_down_mc = sw_fuel_gauge_hal_counts_to_mc((u32)
				(cc_down - sw_fuel_gauge_hal_instance.
						error_base_cc_down));

	/* Calculate the offset error. */
	error_uc =
		(s64) sw_fuel_gauge_hal_instance.platform_data.
		offset_error_uc_per_s * (s64) error_period_sec;
	/* Add in gain the error for current IN to battery. */
	error_uc +=
		(s64) sw_fuel_gauge_hal_instance.platform_data.
		gain_error_1_uc_per_mc * (s64) cc_delta_up_mc;
	/* Add in the gain error for current OUT of the battery. */
	error_uc +=
		(s64) sw_fuel_gauge_hal_instance.platform_data.
		gain_error_2_uc_per_mc * (s64) cc_delta_down_mc;
	/* Convert error to mC for return value.
	NOTE: Standard C divide is not supported fot 64 bit values. */
	error_mc = (int)div_s64_rem(error_uc, SCALING_UC_TO_MC, &remainder);
	SW_FUEL_GAUGE_HAL_DEBUG_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_GET_ACCUMULATED_ERROR, error_mc);
	return error_mc;
}

/**
 * sw_fuel_gauge_hal_set_delta_threshold - Sets the delta reporting threshold
 * in the HW.
 *
 * @delta_threshold_mc		[in] Delta threshold to set. Unit mC.
 */
static void sw_fuel_gauge_hal_set_delta_threshold(int delta_threshold_mc)
{
	u32 delta_threshold_cc_thr;

	SW_FUEL_GAUGE_HAL_DEBUG_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_REQUESTED_DELTA_THRESHOLD_MC,
		 delta_threshold_mc);

	/* Negative or 0 deltas are not allowed. */
	BUG_ON(delta_threshold_mc <= 0);

	/* Calculate threshold value for CC_THR register. */
	delta_threshold_cc_thr = (u32)(delta_threshold_mc /
					sw_fuel_gauge_hal_instance.
						threshold_count_scaling_mc);

	/* CC_THR register field is 8 bits, ensure the calculated value
	fits. */
	if (delta_threshold_cc_thr > PMU_AG620_ES2_CC_THR_LIMIT)
		delta_threshold_cc_thr = PMU_AG620_ES2_CC_THR_LIMIT;

	/* Subtract one as zero counts as one LSB in CC_THR */
	delta_threshold_cc_thr -= 1;

	sw_fg_hal_set_pm_state(sw_fuel_gauge_hal_instance.p_idi_device,
			sw_fuel_gauge_hal_instance.pm_state_en, true);

	/* Write the calculated value into the CC_THR register */
	pmu_iowrite(&sw_fuel_gauge_hal_instance,
			PMU_AG620_CC_THR, delta_threshold_cc_thr);

	sw_fg_hal_set_pm_state(sw_fuel_gauge_hal_instance.p_idi_device,
			sw_fuel_gauge_hal_instance.pm_state_dis, false);

	/* Calculate and store the real delta threshold value set */
	sw_fuel_gauge_hal_instance.delta_threshold_mc =
		(delta_threshold_cc_thr + 1)
		* sw_fuel_gauge_hal_instance.threshold_count_scaling_mc;

	SW_FUEL_GAUGE_HAL_DEBUG_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_SET_DELTA_THRESHOLD_MC,
		 sw_fuel_gauge_hal_instance.delta_threshold_mc);

	if (!sw_fuel_gauge_hal_instance.delta_threshold_set) {

		int ret = 0;
		/* Register and enable the coulomb counter
		interrupt handler with the PMU */
		ret = request_irq(sw_fuel_gauge_hal_instance.irq,
			sw_fuel_gauge_hal_coulomb_counter_delta_irq_cb,
			0, DRIVER_NAME,
			&sw_fuel_gauge_hal_instance.platform_data);
		if (ret != 0) {
			pr_err("Failed to register coulomb counter interrupt: %d\n",
				sw_fuel_gauge_hal_instance.irq);
			BUG();
		}
		sw_fuel_gauge_hal_instance.delta_threshold_set = true;
	}
}

/**
 * sw_fuel_gauge_hal_reset_accumulated_error - Resets the calculated accumulated
 * error for the coulomb counter by storing new baseline values for the counts
 * and rtc.
 */
static void sw_fuel_gauge_hal_reset_accumulated_error(void)
{
	struct timespec rtc_time;

	SW_FUEL_GAUGE_HAL_DEBUG_NO_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_RESET_ACCUMULATED_ERROR);

	/* Reset base values for accumulated errors. */
	sw_fuel_gauge_hal_get_coulomb_counts(&sw_fuel_gauge_hal_instance.
						error_base_cc_up,
						 &sw_fuel_gauge_hal_instance.
						  error_base_cc_down);

	/* Timestamp the new base values. Elapsed real time is not affected by
	changes in the user displayed time and date */
	rtc_time = ktime_to_timespec(ktime_get_boottime());
	sw_fuel_gauge_hal_instance.error_base_rtc_sec = rtc_time.tv_sec;
}

/**
 * Refer to documentation in <sw_fuel_gauge_hal.h>
 */
static int sw_fuel_gauge_hal_set(enum sw_fuel_gauge_hal_set_key key,
				union sw_fuel_gauge_hal_set_params params)
{
	/* Check that interface is called only when the driver is registered */
	BUG_ON(NULL == sw_fuel_gauge_hal_instance.p_sw_fuel_gauge);

	switch (key) {
	case SW_FUEL_GAUGE_HAL_SET_COULOMB_IND_DELTA_THRESHOLD:
		sw_fuel_gauge_hal_set_delta_threshold(params.
						delta_threshold_mc);
		break;

	case SW_FUEL_GAUGE_HAL_SET_ZERO_ACCUMULATED_CC_ERROR:
		/* No parameters. */
		sw_fuel_gauge_hal_reset_accumulated_error();
		break;

	case SW_FUEL_GAUGE_HAL_SET_VBAT_MAX_CLEAR:
		/* Latching of Ibat averages not supported on this hardware,
		therefore nothing to clear */
		break;

	default:
		/* Invalid Set key */
		BUG();
		break;
	}
	/* There is no set key that can fail. */
	return 0;
}

/**
 * Refer to documentation in <sw_fuel_gauge_hal.h>
 */
static int sw_fuel_gauge_hal_get(enum sw_fuel_gauge_hal_get_key key,
				union sw_fuel_gauge_hal_get_params *p_params)
{
	int error = 0;

	SW_FUEL_GAUGE_HAL_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_GET, key);

	/* Check that interface is called only when the driver is registered */
	BUG_ON(NULL == sw_fuel_gauge_hal_instance.p_sw_fuel_gauge);

	/* Check pointer to return parameters. */
	BUG_ON(NULL == p_params);

	switch (key) {
	case SW_FUEL_GAUGE_HAL_GET_CC_ACCUMULATED_ERROR:
		/* Calculate the accumumated error. */
		p_params->cc_acc_error_mc =
			sw_fuel_gauge_hal_calc_accumulated_error();
		break;

	case SW_FUEL_GAUGE_HAL_GET_CC_BALANCED_COUNT:
		/* Call physical layer to read the value from the HW here. */
		p_params->cc_balanced_mc =
			sw_fuel_gauge_hal_read_coulomb_counter(key);
		break;
	case SW_FUEL_GAUGE_HAL_GET_CC_UP_COUNT:
		/* Call physical layer to read the value from the HW here. */
		p_params->cc_up_mc =
			sw_fuel_gauge_hal_read_coulomb_counter(key);
		break;
	case SW_FUEL_GAUGE_HAL_GET_CC_DOWN_COUNT:
		/* Call physical layer to read the value from the HW here. */
		p_params->cc_down_mc =
			sw_fuel_gauge_hal_read_coulomb_counter(key);
		break;

	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE_AT_OCV:
		/* Latching of the current when an OCV was captured is not
		supported on this hardware, therefore just return the latest
		single current measurement */
		p_params->ibat_load_short_at_ocv_ma =
			sw_fuel_gauge_hal_read_battery_current();
		break;

	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE:
		/* Return value of HW Ibat 1 second average current in mA. */
		p_params->ibat_load_short_ma =
			sw_fuel_gauge_hal_read_battery_current();
		break;

	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE_AT_OCV:
		/* Latching of the average current when an OCV was captured is
		not supported on this hardware, therefore just return the latest
		average current calculated */
		error =
			sw_fuel_gauge_hal_calc_long_ibat_average(&p_params->
						ibat_load_long_at_ocv_ma);
		break;

	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE:
		/* Perform calculation of long term Ibat average current. */
		error =
			sw_fuel_gauge_hal_calc_long_ibat_average(&p_params->
							ibat_load_long_ma);
		break;

	case SW_FUEL_GAUGE_HAL_GET_COULOMB_IND_DELTA_THRESHOLD:
		/* If the delta threshold has been set, return it. */
		if (sw_fuel_gauge_hal_instance.delta_threshold_set) {
			p_params->delta_threshold_mc =
				sw_fuel_gauge_hal_instance.delta_threshold_mc;
			SW_FUEL_GAUGE_HAL_DEBUG_PARAM
				(SW_FUEL_GAUGE_DEBUG_HAL_GET_DELTA_THRESHOLD_MC,
				 sw_fuel_gauge_hal_instance.delta_threshold_mc);
		} else {
			SW_FUEL_GAUGE_HAL_DEBUG_NO_PARAM
			 (SW_FUEL_GAUGE_DEBUG_HAL_GET_DELTA_THRESHOLD_FAILED);
			error = -EINVAL;
		}
		break;

	default:
		/* Invalid Get key */
		BUG();
		break;
	}

	return error;
}


/**
* Retrieves platform data from device tree
*
* @p_fg_hal		[in] Device data
* @np			[in] Node pointer
*/
static int swfg_hal_get_platform_data(
				struct sw_fuel_gauge_hal_data *p_fg_hal)
{
#ifdef CONFIG_OF

	u32 val;
	int ret;
	struct device_node *np = p_fg_hal->p_idi_device->device.of_node;

	ret = of_property_read_u32(np, "sense_resistor_mohm", &val);
	if (ret) {
		pr_err("dt: parsing 'sense_resistor_mohm' failed\n");
		return ret;
	}
	p_fg_hal->platform_data.sense_resistor_mohm = (int) val;

	ret = of_property_read_u32(np, "gain_error_1_uc_per_mc", &val);
	if (ret) {
		pr_err("dt: parsing 'gain_error_1_uc_per_mc' failed\n");
		return ret;
	}
	p_fg_hal->platform_data.gain_error_1_uc_per_mc = (int) val;

	ret = of_property_read_u32(np, "gain_error_2_uc_per_mc", &val);
	if (ret) {
		pr_err("dt: parsing 'gain_error_2_uc_per_mc' failed\n");
		return ret;
	}
	p_fg_hal->platform_data.gain_error_2_uc_per_mc = (int) val;

	ret = of_property_read_u32(np, "offset_error_uc_per_s", &val);
	if (ret) {
		pr_err("dt: parsing 'offset_error_uc_per_s' failed\n");
		return ret;
	}
	p_fg_hal->platform_data.offset_error_uc_per_s = (int) val;

	return 0;
#else

	struct sw_fuel_gauge_platform_data *p_platform_data =
			p_fg_hal->p_idi_device->device.platform_data;

	/* Check pointer to platform data */
	BUG_ON(NULL == p_platform_data);

	memcpy(&p_fg_hal->platform_data, p_platform_data,
			sizeof(struct sw_fuel_gauge_platform_data));

	return 0;
#endif
}


/**
 * sw_fuel_gauge_hal_probe - Initialises the driver, when the device has been
 * found.
 *
 * @ididev	[in]	Pointer to IDI device which triggered the probe.
 */
static int __init sw_fuel_gauge_hal_probe(struct idi_peripheral_device *ididev,
						const struct idi_device_id *id)
{
	struct sw_fuel_gauge_hal_data *p_fg_hal =
				&sw_fuel_gauge_hal_instance;
	struct resource *pmu_cc_res;
	struct resource *res;
	int ret;

	SW_FUEL_GAUGE_HAL_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_PROBE);

	/* Store platform device in static instance. */
	sw_fuel_gauge_hal_instance.p_idi_device = ididev;

	ret = swfg_hal_get_platform_data(p_fg_hal);
	if (ret)
		return ret;

	res = idi_get_resource_byname(&ididev->resources,
				IORESOURCE_IRQ, "cccl");
	if (res == NULL) {
		pr_err("%s: Unable to get idi resoure\n", __func__);
		return -EINVAL;
	}

	sw_fuel_gauge_hal_instance.irq = res->start;
	if (IS_ERR_VALUE(sw_fuel_gauge_hal_instance.irq)) {
		pr_err("setup of couloumb counter irq failed\n");
		BUG();
	}

	ret =  idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
			__func__);
		return -EINVAL;
	}

	sw_fuel_gauge_hal_instance.pm_state_en =
		idi_peripheral_device_pm_get_state_handler(ididev, "enable");
	if (sw_fuel_gauge_hal_instance.pm_state_en == NULL) {
		pr_err("%s: Unable to get handler for PM state 'enable'!\n",
				__func__);
		return -EINVAL;
	}

	sw_fuel_gauge_hal_instance.pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(ididev, "disable");
	if (sw_fuel_gauge_hal_instance.pm_state_dis == NULL) {
		pr_err("%s: Unable to get handler for PM state 'disable'!\n",
				__func__);
		return -EINVAL;
	}

	pr_info("%s: Getting PM state handlers: OK\n", __func__);

	pmu_cc_res =
		idi_get_resource_byname(&ididev->resources, IORESOURCE_MEM,
					"pmu_cc");
	if (pmu_cc_res == NULL) {
		pr_err("getting PMU's CC resource failed!\n");
		return -EINVAL;
	}

	sw_fuel_gauge_hal_instance.pmu_cc_res = pmu_cc_res;

	/* Set up the wake lock to prevent suspend between multiple register
	accesses. */
	wake_lock_init(&sw_fuel_gauge_hal_instance.suspend_lock,
			WAKE_LOCK_SUSPEND, "sw_fuel_gauge_hal_wake_lock");

	ret = idi_set_power_state(
			ididev, sw_fuel_gauge_hal_instance.pm_state_en,
				true);

	if (ret) {
		pr_err("%s: setting PM state '%s' failed!\n", __FILE__,
				sw_fuel_gauge_hal_instance.pm_state_en->name);
		return -EIO;
	}

	/* Switch on the coulomb counter */
	pmu_iowrite(&sw_fuel_gauge_hal_instance,
			PMU_AG620_CC_CTRL, PMU_AG620_CC_CTRL_ENABLE);

	SW_FUEL_GAUGE_HAL_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_CC_CTRL_REG,
		pmu_ioread(&sw_fuel_gauge_hal_instance, PMU_AG620_CC_CTRL));

	/* Clear the coulomb counter delta threshold */
	pmu_iowrite(&sw_fuel_gauge_hal_instance,
			PMU_AG620_CC_CTRL2, PMU_AG620_CC_CTRL2_CLEAR);

	sw_fg_hal_set_pm_state(ididev, sw_fuel_gauge_hal_instance.pm_state_dis,
					false);


	/* Calculate the coulomb counter scaling factor from platform data.
	The coulomb integrator detection threshold is 500mV.
	The SOC clock is given as 32758kHz/4 = 8.192kHz.
	Assume we have a constant DC current of i amperes (= 1000i mA) flowing
	through Rsense. The voltage at the input of the integrator is:

	Vin = (i A)*(Rsense mOhm) = (Rsense i ) mV

	Hence, to reach the threshold of 500 mV at its output and generate a
	clock pulse to the CCUP counter, the integrator will need a time of:

	(500mV / Rsense mOhm)/i clock pulses = (500mV / Rsense mOhm)*(1/8192) s

	For example, if Rsense is 20 mOhm, 1 LSB in the CCUP register is
	equivalent (in millicoulombs) to:

	(1000i mA)* ( 500mV/20mOhm)/i )*(1/8192) s = 25000/8192 mAs
	= 3.05175 mC. */

	sw_fuel_gauge_hal_instance.coulomb_count_scaling_uc =
			((SCALING_C_TO_UC *
			COULOMB_COUNTER_INCREMENT_THRESHOLD_MV_ES2) /
			p_fg_hal->platform_data.sense_resistor_mohm) /
				COULOMB_COUNTER_CLOCK_FREQ_HZ;

	SW_FUEL_GAUGE_HAL_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_CC_SCALE_UC,
			sw_fuel_gauge_hal_instance.coulomb_count_scaling_uc);

	/* Calculate the coulomb threshold scaling factor from coulomb counter
	scaling factor */
	sw_fuel_gauge_hal_instance.threshold_count_scaling_mc =
		(sw_fuel_gauge_hal_instance.coulomb_count_scaling_uc *
		COULOMB_COUNTER_DELTA_THRESHOLD_SCALING_ES2)
			/ SCALING_UC_TO_MC;

	SW_FUEL_GAUGE_HAL_DEBUG_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_CC_THRESHOLD_SCALE_MC,
			sw_fuel_gauge_hal_instance.threshold_count_scaling_mc);

	/* Read the initial values of both coulomb counters and set up the
	average filter. */
	sw_fuel_gauge_hal_init_long_term_ibat_average_and_accumulated_error();

	/* Initialise the alarm timer used for updating the IBAT average
	reference. */
	alarm_init(&sw_fuel_gauge_hal_instance.
			ibat_long_term_average_polling_atimer,
			 ALARM_REALTIME,
			  sw_fuel_gauge_hal_polling_atimer_expired_cb);

	/* Calculate and set the period for the first timeout */
	SET_ALARM_TIMER(&sw_fuel_gauge_hal_instance.
			ibat_long_term_average_polling_atimer,
			IBAT_LONG_TERM_AVERAGE_POLLING_PERIOD_SECS_ES2);

	/* Register the HAL with the SW Fuel Gauge */
	BUG_ON(sw_fuel_gauge_register_hal(&ag620_sw_fuel_gauge_hal,
					&sw_fuel_gauge_hal_instance.
							p_sw_fuel_gauge));

	/* There are no recoverable errors for this function. Failure will
	cause a panic. */
	return 0;
}

/**
 * sw_fuel_gauge_hal_remove - Release allocated resources on device removal.
 */
static int __exit sw_fuel_gauge_hal_remove(struct idi_peripheral_device *ididev)
{
	SW_FUEL_GAUGE_HAL_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_REMOVE);

	/* Deregister CC interrupt handler */
	free_irq(sw_fuel_gauge_hal_instance.irq,
			&sw_fuel_gauge_hal_instance.platform_data);

	wake_lock_destroy(&sw_fuel_gauge_hal_instance.suspend_lock);

	/* Delete allocated resources and mark driver as uninitialised. */
	if (NULL != sw_fuel_gauge_hal_instance.p_sw_fuel_gauge) {
		sw_fuel_gauge_hal_instance.p_sw_fuel_gauge = NULL;
		(void)alarm_cancel(&sw_fuel_gauge_hal_instance.
					ibat_long_term_average_polling_atimer);
	}
	return 0;
}

/**
 * sw_fuel_gauge_hal_suspend() - Called when the system is attempting to
 * suspend.
 *
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int sw_fuel_gauge_hal_suspend(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	disable_irq(sw_fuel_gauge_hal_instance.irq);
	/* Nothing to do here except logging */
	SW_FUEL_GAUGE_HAL_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_SUSPEND);
	return 0;
}

/**
 * sw_fuel_gauge_hal_resume() - Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int sw_fuel_gauge_hal_resume(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	enable_irq(sw_fuel_gauge_hal_instance.irq);
	/* Nothing to do here */
	SW_FUEL_GAUGE_HAL_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_RESUME);
	return 0;
}

const struct dev_pm_ops sw_fuel_gauge_hal_pm = {
	.suspend = sw_fuel_gauge_hal_suspend,
	.resume = sw_fuel_gauge_hal_resume,
};

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_CCD,
	},

	{ /* end: all zeroes */},
};

/**
 * sw_fuel_gauge_hal_driver - Driver structure for SW fuel gauge HAL.
 */
static struct idi_peripheral_driver sw_fuel_gauge_hal_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &sw_fuel_gauge_hal_pm,
	},
	.p_type = IDI_CCD,
	.id_table = idi_ids,
	.probe = sw_fuel_gauge_hal_probe,
	.remove = sw_fuel_gauge_hal_remove,
};

/**
 * sw_fuel_gauge_hal_init - SW fuel gauge HAL device init function.
 * returns		0 for success, or error code.
 */
static int __init sw_fuel_gauge_hal_init(void)
{
	int ret;

	/* NOTE: Must initialise debug data spinlock before any logging. */
	spin_lock_init(&sw_fuel_gauge_hal_debug_data.lock);
	SW_FUEL_GAUGE_HAL_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_INIT);

	ret = idi_register_peripheral_driver(&sw_fuel_gauge_hal_driver);

	return ret;
}

/**
 * sw_fuel_gauge_hal_exit - SW fuel gauge HAL device deinit function.
 */
static void __exit sw_fuel_gauge_hal_exit(void)
{
	SW_FUEL_GAUGE_HAL_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_EXIT);

	idi_unregister_peripheral_driver(&sw_fuel_gauge_hal_driver);
}

late_initcall(sw_fuel_gauge_hal_init);
module_exit(sw_fuel_gauge_hal_exit);

MODULE_DESCRIPTION("AGOLD620 SW Fuel Gauge HAL Driver");
MODULE_LICENSE("GPL v2");
