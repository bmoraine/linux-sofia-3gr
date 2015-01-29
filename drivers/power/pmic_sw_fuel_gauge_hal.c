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
 * with this program; If not, see <http://www.gnu.org/licenses/>.
 *
 */
#define DRIVER_NAME					"pmic_swfg_hal"
#define pr_fmt(fmt) DRIVER_NAME": "fmt

#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <sofia/vmm_pmic.h>
#include <sofia/vmm_pmic-ext.h>
#include <linux/of.h>

#include <linux/power/sw_fuel_gauge_debug.h>
#include <linux/power/sw_fuel_gauge_hal.h>
#include <linux/power/sw_fuel_gauge_platform.h>

/* Base address of DEV1 */
#define BASE_ADDRESS_DEV1				(0x4e)
#define BASE_ADDRESS_DEV1_VMM			(BASE_ADDRESS_DEV1<<24)

/* Address of ID0 register */
#define ID0_REG			(0x00)
#define ID0_VMM			(BASE_ADDRESS_DEV1_VMM | ID0_REG)

#define MAJREV_M		(0x7)
#define MAJREV_O		(3)

#define MINREV_M		(0x7)
#define MINREV_O		(0)

#define REV_MAJ_AX		(0)
#define REV_MIN_X0		(0)

/* Address of GPADC interrupt register */
#define ADCIRQ_REG		(0x08)
#define ADCIRQ_VMM		(BASE_ADDRESS_DEV1_VMM | ADCIRQ_REG)

#define CCTICK_M		(0x1)
#define CCTICK_O		(7)

#define IRQ_CLEAR		(1)

/* Address of Level 1 Interrupt Mask Register */
#define MIRQLVL1_REG		(0x0e)
#define MIRQLVL1_VMM		(BASE_ADDRESS_DEV1_VMM | MIRQLVL1_REG)

#define MADC_M			(0x1)
#define MADC_O			(4)
#define IRQ_UNMASK		(0)

/* Address of Level 2 Interrupt Mask Register */
#define MADCIRQ_REG		(0x16)
#define MADCIRQ_VMM		(BASE_ADDRESS_DEV1_VMM | MADCIRQ_REG)

#define MCCTICK_M		(0x1)
#define MCCTICK_O		(7)

/* Base address of DEV2 */
#define BASE_ADDRESS_DEV2				(0x4f)
#define BASE_ADDRESS_DEV2_VMM			(BASE_ADDRESS_DEV2<<24)

/* Bitfield mask for positive part of 13 bit battery current field. */
#define PMU_CC_CBAT_POS_MASK					(0xfff)
/* Bitfield mask for negative part of 13 bit battery current field. */
#define PMU_CC_CBAT_NEG_MASK					(0x1000)

#define LOWER_BYTE_MASK		(0xff)

/* Address of coulomb counter THR register. */
#define CC_THRH_REG		(0xe6)
#define CC_THRH_VMM		(BASE_ADDRESS_DEV2_VMM | CC_THRH_REG)

/* Maximum value of CC_THR register */
#define CC_THR_LIMIT		(0xffff)

/* Address of short term battery current register. */
#define CC_CURR_SHRTH_REG	(0xe8)
#define CC_CURR_SHRTH_VMM	(BASE_ADDRESS_DEV2_VMM | CC_CURR_SHRTH_REG)

/* Address of long term battery current register. */
#define CC_CURR_LNGH_REG	(0xea)
#define CC_CURR_LNGH_VMM	(BASE_ADDRESS_DEV2_VMM | CC_CURR_LNGH_REG)

/* Address of coulomb counter DOWN register. */
#define CC_PERSIST_DOWN_B3_REG	(0xd5)
#define CC_PERSIST_DOWN_B3_VMM	(BASE_ADDRESS_DEV2_VMM | CC_PERSIST_DOWN_B3_REG)

/* Address of coulomb counter UP register. */
#define CC_PERSIST_UP_B3_REG	(0xd9)
#define CC_PERSIST_UP_B3_VMM	(BASE_ADDRESS_DEV2_VMM | CC_PERSIST_UP_B3_REG)

/* VBATMAX registers */
#define VBATMAXH_REG		(0xf6)
#define VBATMAXH_REG_VMM	(BASE_ADDRESS_DEV2_VMM | VBATMAXH_REG)
#define CLR_VBATMAX_BIT		(1 << 7)

/* Address of latched short term battery current register. */
#define MAX_CURR_SHRTH_REG	(0xf8)
#define MAX_CURR_SHRTH_VMM	(BASE_ADDRESS_DEV2_VMM | MAX_CURR_SHRTH_REG)

/* Address of latched long term battery current register. */
#define MAX_CURR_LNGH_REG	(0xfa)
#define MAX_CURR_LNGH_VMM	(BASE_ADDRESS_DEV2_VMM | MAX_CURR_LNGH_REG)

/* Address of Coulomb Counter Control Register 0 */
#define CC_CTRL0_REG		(0xec)
#define CC_CTRL0_VMM		(BASE_ADDRESS_DEV2_VMM | CC_CTRL0_REG)

#define CC_PERSIST_CLR_O	(3)
#define CC_PERSIST_CLR_M	(0x1)

#define CC_OFF_O		(0)
#define CC_OFF_M		(0x1)

/* Base address of DEV4 */
#define BASE_ADDRESS_DEV4	(0x5f)
#define BASE_ADDRESS_DEV4_VMM	(BASE_ADDRESS_DEV4<<24)

/* Address of Cause of Death Status Register */
#define CODSRC_REG		(0x6c)
#define CODSRC_VMM		(BASE_ADDRESS_DEV4_VMM | CODSRC_REG)

#define BATTREP_O		(4)
#define BATTREP_M		(0x1)


/* Coulomb Scaling factor  from C to uC value. */
#define SCALING_C_TO_UC						(1000000)
/* Coulomb Scaling factor from uC to mC value. */
#define SCALING_UC_TO_MC					(1000)
/* Scaling factor from uA to mA. */
#define SCALING_UA_TO_MA					(1000)
/* Scaling factor  from uS to S. */
#define SCALING_US_TO_S						(1000000)

/* Trigger level for coulomb counter increment in mV. */
#define COULOMB_COUNTER_INCREMENT_THRESHOLD_MV			(250)

/* Sampling frequency of coulomb counter integrator in Hz. */
#define COULOMB_COUNTER_CLOCK_FREQ_HZ				(8192)

/* Delta coulomb counter threshold scaling factor */
#define COULOMB_COUNTER_DELTA_THRESHOLD_SCALING			(8)

/* Delta coulomb counter threshold (CCTHR) offset */
#define COULOMB_COUNTER_DELTA_THRESHOLD_OFFSET			(0)

/* Macro to trace and log debug event and data. */
#define SWFGH_DEBUG_PARAM(_event, _param)\
	SWFG_DEBUG(pmic_swfg_hal_debug_data, _event, _param)

/* Macro to trace and log debug event without a parameter. */
#define SWFGH_DEBUG_NO_PARAM(_event)\
	SWFG_DEBUG(pmic_swfg_hal_debug_data, _event, 0)

/* Macro to trace and log debug event without a parameter or printk. */
#define SWFGH_DEBUG_NO_LOG_NO_PARAM(_event)\
	SWFG_DEBUG_NO_PRINTK(pmic_swfg_hal_debug_data, _event, 0)

/* PMIC HW revision structure */
struct pmic_hw_rev {
	u8 major_rev;
	u8 minor_rev;
};

/* SW Fuel Gauge Hal control structure */
struct pmic_swfg_hal_data {
	/* SW Fuel Gauge callback functions. */
	struct sw_fuel_gauge_interface *p_sw_fuel_gauge;
	/* pointer to platform device. */
	struct platform_device	*p_platform_device;
	/* platform configuration parameters. */
	struct sw_fuel_gauge_platform_data	platform_data;
	/* Virtual address of PMU Coulomb Counter registers. */
	u16			pmu_cc_base;
	/* Base timestamp for accumulated error. */
	time_t			error_base_timestamp_sec;
	/* Base count for accumulated error in charge IN to battery. */
	u32			error_base_cc_up_counts;
	/* Base count for accumulated error in charge OUT of battery. */
	u32			error_base_cc_down_counts;
	/* Coulomb delta threshold currently set (mC) */
	int			delta_threshold_mc;
	/* true after coulomb delta threshold has been set. */
	bool			delta_threshold_set;
	/* Scaling factor from counts to mC for delta threshold */
	int			threshold_count_scaling_mc;
	/* Scaling factor from UP and DOWN counts to uC */
	int			coulomb_count_scaling_uc;
	/* Coulomb counter interrupt */
	int irq;
	/* IRQ enabled */
	bool irq_en;
	/* Hardware revision */
	struct pmic_hw_rev hw_rev;
};

/**
 * Refer to documentation in <sw_fuel_gauge_hal.h>
 */
static int pmic_swfg_hal_set(enum sw_fuel_gauge_hal_set_key key,
				union sw_fuel_gauge_hal_set_params params);

/**
 * Refer to documentation in <sw_fuel_gauge_hal.h>
 */
static int pmic_swfg_hal_get(enum sw_fuel_gauge_hal_get_key key,
				union sw_fuel_gauge_hal_get_params *p_params);

/**
 * SW Fuel Gauge Hal exported interface.
 * Refer to documentation in <sw_fuel_gauge_hal.h>
 */
static struct sw_fuel_gauge_hal_interface sc_pmic_swfg_hal = {
	pmic_swfg_hal_set,	/* set	*/
	pmic_swfg_hal_get,	/* get	*/
};

/* SW Fuel Gauge Hal instance */
static struct pmic_swfg_hal_data pmic_swfg_hal_instance = {
	.irq_en = -1,
};

/* Array to collect debug data */
static struct sw_fuel_gauge_debug_data pmic_swfg_hal_debug_data;


/**
 * Enable or disable the coulomb counter event
 * IRQ If the state has not changed since the last call,
 * no propgagation to the HW is made.
 *
 * @p_fg_hal	[in] Device data
 * @new_state	[in] new state of irq: enable or disable
 */
static void pmic_swfg_hal_cctick_irq_en(struct pmic_swfg_hal_data *p_fg_hal,
						bool new_state)
{
	if (new_state != p_fg_hal->irq_en) {
		if (new_state)
			enable_irq(p_fg_hal->irq);
		else
			disable_irq(p_fg_hal->irq);

		p_fg_hal->irq_en = new_state;
	}
}

/**
 * Read the raw couloumb counter values.
 *
 * @cc_up_counts	[out] Raw value of coulomb UP counter in counts.
 * @cc_down_counts	[out] Raw value of coulomb DOWN counter in counts.
 */
static void pmic_swfg_hal_get_coulomb_counts(u32 *cc_up_counts,
						u32 *cc_down_counts)
{
	u32 addr;
	u32 data0, data1, data2, data3, dummy;

	/*
	 * Read registers and return values to the caller.
	 */

	addr = CC_PERSIST_UP_B3_VMM;
	/* Read the LSB register 1st, as this read is used to latch MSBs */
	WARN_ON(vmm_pmic_reg_read(addr+3, &data0));
	/* Dummy read to workaround register double-buffer bug - START
	MSB registers are incorrectly doubled-buffered and a dummy read
	after the normal LSB read is required to get latest contents of MSBs */
	WARN_ON(vmm_pmic_reg_read(addr+3, &dummy));
	/* Dummy read required to workaround register double-buffer bug - END */
	WARN_ON(vmm_pmic_reg_read(addr+2, &data1));
	WARN_ON(vmm_pmic_reg_read(addr+1, &data2));
	WARN_ON(vmm_pmic_reg_read(addr, &data3));
	data0 &= 0xff;
	data1 &= 0xff;
	data2 &= 0xff;
	data3 &= 0xff;


	*cc_up_counts   = (((u32)data3 << 24)+((u32)data2 << 16)
				+((u32)data1 << 8)+data0);
	pr_debug("CC up cnt: addr=0x%x, (0x%x 0x%x 0x%x 0x%x)->0x%x\n",
			addr, data3, data2, data1, data0, *cc_up_counts);


	addr = CC_PERSIST_DOWN_B3_VMM;
	/* Read the LSB register 1st, as this read is used to latch MSBs */
	WARN_ON(vmm_pmic_reg_read(addr+3, &data0));
	/* Dummy read to workaround register double-buffer bug - START
	MSB registers are incorrectly doubled-buffered and a dummy read
	after the normal LSB read is required to get latest contents of MSBs */
	WARN_ON(vmm_pmic_reg_read(addr+3, &dummy));
	/* Dummy read required to workaround register double-buffer bug - END */
	WARN_ON(vmm_pmic_reg_read(addr+2, &data1));
	WARN_ON(vmm_pmic_reg_read(addr+1, &data2));
	WARN_ON(vmm_pmic_reg_read(addr, &data3));
	data0 &= 0xff;
	data1 &= 0xff;
	data2 &= 0xff;
	data3 &= 0xff;

	*cc_down_counts   = (((u32)data3 << 24)+((u32)data2 << 16)
				+((u32)data1 << 8)+data0);
	pr_debug("CC down cnt: addr=0x%x,(0x%x 0x%x 0x%x 0x%x)->0x%x\n"
				, addr, data3, data2,
				data1, data0, *cc_down_counts);

	SWFGH_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_CC_UP_COUNTS,
					*cc_up_counts);
	SWFGH_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_CC_DOWN_COUNTS,
					*cc_down_counts);
}

/**
 * Convert raw couloumb counter values to mC.
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
static int pmic_swfg_hal_counts_to_mc(int cc_counts)
{
	/* Scale the HW count value to uC using 64-bit arithmetic to maintain
	accuracy. */
	s64 result_uc_64
		= (s64)cc_counts *
		(s64)pmic_swfg_hal_instance.coulomb_count_scaling_uc;
	s32 remainder;
	/*
	 * Truncate to 32-bit signed mC value for result.
	 * NOTE: 64 Division is not supported with the standard C operator
	 */
	return (int)div_s64_rem(result_uc_64, SCALING_UC_TO_MC, &remainder);
}

/**
 * Read the HW and return the requested
 * coulomb counter value converted to mC.
 * @value_to_read	[in] Specifies which coulomb count to read.
 * Returns:		Signed coulomb count in mC.
 */
static int pmic_swfg_hal_read_coulomb_counter(enum sw_fuel_gauge_hal_get_key
								value_to_read)
{
	u32 cc_up_counts;
	u32 cc_down_counts;
	int result_mc;

	/* Get the raw coulomb counter values from the HW . */
	pmic_swfg_hal_get_coulomb_counts(&cc_up_counts, &cc_down_counts);

	switch (value_to_read) {
	case SW_FUEL_GAUGE_HAL_GET_CC_BALANCED_COUNT:
		/* Calculate balanced value.
		 * In SC, the coulomb counter assumes positive IBAT when
		 * charging (according to the datasheet, this is correct).
		 * This design works on the premise that the IBAT is negative
		 * when charging, so the sign is inverted by subtracting the
		 * negative minus the positive.
		 */
		result_mc = pmic_swfg_hal_counts_to_mc
				(cc_down_counts - cc_up_counts);
		SWFGH_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_HAL_GET_BALANCED_CC_MC, result_mc);
		break;

	case SW_FUEL_GAUGE_HAL_GET_CC_UP_COUNT:
		/* Number of counts into the battery. */
		result_mc = pmic_swfg_hal_counts_to_mc(cc_up_counts);
		SWFGH_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_HAL_GET_CC_UP_MC, result_mc);
		break;

	case SW_FUEL_GAUGE_HAL_GET_CC_DOWN_COUNT:
		/* Number of counts out of the battery. */
		result_mc = pmic_swfg_hal_counts_to_mc(cc_down_counts);
		SWFGH_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_HAL_GET_CC_DOWN_MC, result_mc);
		break;

	case SW_FUEL_GAUGE_HAL_GET_CC_ACCUMULATED_ERROR:
	case SW_FUEL_GAUGE_HAL_GET_COULOMB_IND_DELTA_THRESHOLD:
	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE_AT_OCV:
	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE:
	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE_AT_OCV:
		/* Fall through to BUG() intentional */
	default:
		/* Invalid read parameter. */
		BUG();
		break;
	}
	return result_mc;
}

/**
 * Read the HW registers and return the requested type of Ibat reading.
 *
 * @longterm [in]	Specifies whether long (TRUE) or short (FALSE) average
 *			current is to be read.
 * @latched [in]	Specifies whether latched (TRUE) or running (FALSE)
 *			measurement is to be read.
 * Returns:		Battery current in mA.
 */
static int pmic_swfg_hal_read_battery_current(bool longterm, bool latched)
{
	int ibat_positive;
	int ibat_negative;
	int ibat_count_signed;
	int ibat_ma = INT_MAX;
	u32 addr;
	u32 data0, data1, dummy;
	u32 ibat_reg;
	bool data_valid = true;

	/* Determine the address offset based on the type of measurement
	to be read.
	On A0 PMIC the short term average current measurement hardware
	doesn't work */
	if (longterm || (REV_MAJ_AX == pmic_swfg_hal_instance.hw_rev.major_rev
		&& REV_MIN_X0 == pmic_swfg_hal_instance.hw_rev.minor_rev)) {
		if (latched)
			addr = MAX_CURR_LNGH_VMM;
		else
			addr = CC_CURR_LNGH_VMM;
	} else {
		if (latched)
			addr = MAX_CURR_SHRTH_VMM;
		else
			addr = CC_CURR_SHRTH_VMM;
	}

	if (latched) {
		u32 max_vbat;

		/* Check whether latched values are valid and have been written
		yet by checking whether Max Voltage value is not zero
		(reset value) */
		WARN_ON(vmm_pmic_reg_read(VBATMAXH_REG_VMM + 1, &data0));
		data0 &= 0xff;

		WARN_ON(vmm_pmic_reg_read(VBATMAXH_REG_VMM, &data1));
		data1 &= 0xf;

		max_vbat = (((u32)data1 << 8)+data0);

		data_valid = (max_vbat != 0);
	}

	if (data_valid) {
		/* Read the LSB register 1st, as this read is used to
		latch MSB */
		WARN_ON(vmm_pmic_reg_read(addr+1, &data0));
		data0 &= 0xff;
		/* Dummy read to workaround register double-buffer bug - START
		MSB registers are incorrectly doubled-buffered and a dummy
		read after the normal LSB read is required to get latest
		contents of MSB */
		WARN_ON(vmm_pmic_reg_read(addr+1, &dummy));
		/*
		Dummy read required to workaround register double-buffer bug
		- END */
		WARN_ON(vmm_pmic_reg_read(addr, &data1));
		data1 &= 0x1f;

		ibat_reg = (((u32)data1 << 8)+data0);

		/* Extract two's complement fields from register value. */
		ibat_positive = ibat_reg & PMU_CC_CBAT_POS_MASK;
		ibat_negative = ibat_reg & PMU_CC_CBAT_NEG_MASK;

		/* Calculate signed count value.
		 * The coulomb counter assumes positive IBAT when charging
		 * (according to the datasheet, this is correct).
		 * Our design works on the premise that the IBAT is negative
		 * when charging, so the sign is inverted by subtracting the
		 * negative minus the positive.
		 */
		ibat_count_signed = ibat_negative - ibat_positive;

		/* Return the HW count value scaled to mA. */
		ibat_ma = (int)((ibat_count_signed * pmic_swfg_hal_instance
				.coulomb_count_scaling_uc)
				/ SCALING_UA_TO_MA);
	} else {
		pr_debug("ibat: longterm=%d, latched=%d, Not Valid\n",
				longterm, latched);
	}


	if (longterm) {
		if (latched) {
			SWFGH_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_HAL_GET_IBAT_LONG_AV_LATCHED_MA,
			ibat_ma);
		} else {
			SWFGH_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_HAL_GET_IBAT_LONG_AV_MA, ibat_ma);
		}
	} else {
		if (latched) {
			SWFGH_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_HAL_GET_IBAT_SHORT_AV_LATCHED_MA,
			ibat_ma);
		} else {
			SWFGH_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_HAL_GET_IBAT_SHORT_AV_MA, ibat_ma);
		}
	}

	return ibat_ma;
}

/**
 * Handle the coulomb counter IRQ interrupt
 *
 * @param	[in]	Generic parameter passed from scheduler queue. Not used.
 */
static void pmic_swfg_hal_cc_irq_work(int param)
{
	/* Generic parameter is not used here. */
	(void)param;

	SWFGH_DEBUG_NO_PARAM(
		SW_FUEL_GAUGE_DEBUG_HAL_PROCESS_IRQ_AND_TIMER_WORK);

	/* If the SW Fuel Gauge has set a threshold, report the current state
		of the coulomb counter. */
	if (pmic_swfg_hal_instance.delta_threshold_set) {

		union sw_fuel_gauge_hal_cb_param cb_param = {
			.cc_delta_mc = pmic_swfg_hal_read_coulomb_counter(
				SW_FUEL_GAUGE_HAL_GET_CC_BALANCED_COUNT),
		};
		/* Threshold has been crossed, inform the sw fuel gauge. */
		pmic_swfg_hal_instance.p_sw_fuel_gauge->event_cb(
			SW_FUEL_GAUGE_HAL_CB_EVENT_SOC_UPDATE,
			cb_param);
	}
}


/**
 * Schedules a work to handle the colomb counter delta IRQ event
 *
 * @irq		[in] (not used)
 * @dev		[in] driver data
 */
static irqreturn_t pmic_swfg_hal_cc_delta_irq_cb(int irq, void *dev)
{
	struct pmic_swfg_hal_data *p_fg_hal = (struct pmic_swfg_hal_data *) dev;

	/* Unused */
	(void)irq;

	SWFGH_DEBUG_NO_LOG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_CC_DELTA_IRQ);

	/* Schedule the SW Fuel Gauge work thread to execute the polling
	function. */
	p_fg_hal->p_sw_fuel_gauge->
		enqueue((fp_scheduled_function)pmic_swfg_hal_cc_irq_work, 0);

	return IRQ_HANDLED;
}

/**
 * Sets the delta reporting threshold
 * in the HW.
 * @p_fg_hal			[in] Device data
 * @delta_threshold_mc		[in] Delta threshold to set. Unit mC.
 */
static void pmic_swfg_hal_set_delta_threshold(
		struct pmic_swfg_hal_data *p_fg_hal,
		int delta_threshold_mc)
{
	u32 delta_threshold_cc_thr;

	SWFGH_DEBUG_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_REQUESTED_DELTA_THRESHOLD_MC,
					delta_threshold_mc);

	/* Negative or 0 deltas are not allowed. */
	BUG_ON(delta_threshold_mc <= 0);

	/* Calculate threshold value for CC_THR register. */
	delta_threshold_cc_thr = (u32)(delta_threshold_mc /
			p_fg_hal->threshold_count_scaling_mc);

	/*
	CC_THR register field is 16 bits, ensure the calculated value
	fits. */
	if (delta_threshold_cc_thr > CC_THR_LIMIT)
		delta_threshold_cc_thr = CC_THR_LIMIT;

	/* Adjust according CCTHR LSB meaning */
	delta_threshold_cc_thr -= COULOMB_COUNTER_DELTA_THRESHOLD_OFFSET;

	/* Write the calculated value into the CC_THR register */
	/* Write the high register first according to the datasheet */
	WARN_ON(vmm_pmic_reg_write(CC_THRH_VMM, delta_threshold_cc_thr>>8));
	WARN_ON(vmm_pmic_reg_write(CC_THRH_VMM+1, delta_threshold_cc_thr
							& LOWER_BYTE_MASK));

	/* Calculate and store the real delta threshold value set */
	p_fg_hal->delta_threshold_mc =
				(delta_threshold_cc_thr
				+ COULOMB_COUNTER_DELTA_THRESHOLD_OFFSET)
				* p_fg_hal->threshold_count_scaling_mc;

	SWFGH_DEBUG_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_SET_DELTA_THRESHOLD_MC,
		p_fg_hal->delta_threshold_mc);

	if (!p_fg_hal->delta_threshold_set) {
		pmic_swfg_hal_cctick_irq_en(p_fg_hal, true);
		p_fg_hal->delta_threshold_set = true;
	}
}


/**
 * Initialise the accumulated error data.
 *
 * @p_fg_hal		[in] Device data
 */
static void pmic_swfg_hal_init_accumulated_error
				(struct pmic_swfg_hal_data *p_fg_hal)
{
	u32 cc_up_counts;
	u32 cc_down_counts;
	struct timespec ts;

	/* Get new base timestamp */
	/* Monotonic time since boot including time in suspend */
	ts = ktime_to_timespec(ktime_get_boottime());

	/* Read raw coulomb counters. */
	pmic_swfg_hal_get_coulomb_counts(&cc_up_counts, &cc_down_counts);

	/* Set the reference base for accumulated error calculations. */
	p_fg_hal->error_base_cc_up_counts	= cc_up_counts;
	p_fg_hal->error_base_cc_down_counts	= cc_down_counts;
	p_fg_hal->error_base_timestamp_sec	= ts.tv_sec;
}

/**
 * Resets the calculated accumulated error for the coulomb counter by storing
 * new baseline values for the counts and timestamp.
 *
 * @p_fg_hal			[in] Device data
 */
static void pmic_swfg_hal_reset_accumulated_error
				(struct pmic_swfg_hal_data *p_fg_hal)
{
	struct timespec ts;

	SWFGH_DEBUG_NO_PARAM(
		SW_FUEL_GAUGE_DEBUG_HAL_RESET_ACCUMULATED_ERROR);

	/* Reset base values for accumulated errors. */
	pmic_swfg_hal_get_coulomb_counts(&p_fg_hal->error_base_cc_up_counts,
			&p_fg_hal->error_base_cc_down_counts);

	/* Get new base timestamp */
	/* Monotonic time since boot including time in suspend */
	ts = ktime_to_timespec(ktime_get_boottime());
	p_fg_hal->error_base_timestamp_sec = ts.tv_sec;
}

/**
 * Perform calculation of accumulated
 * error in coulomb counter since last error reset.
 *
 * @p_fg_hal			[in] Device data
 *
 * Returns: Accumulated coulomb count error in mC.
 */
static int pmic_swfg_hal_calc_accumulated_error
				(struct pmic_swfg_hal_data *p_fg_hal)
{
	u32 cc_up_counts;
	u32 cc_down_counts;
	int cc_delta_up_mc;
	int cc_delta_down_mc;
	/* 64 Bit arithmetic used to maintain accuracy in uC calculations. */
	s64 error_uc;
	int error_mc;
	time_t error_period_sec;
	struct timespec ts;
	int remainder;

	/* Get timestamp. Monotonic time since boot including time in suspend */
	ts = ktime_to_timespec(ktime_get_boottime());

	/* Calculate the time elapsed since the last error reset. */
	error_period_sec = ts.tv_sec - p_fg_hal->error_base_timestamp_sec;

	/* Read the raw coulomb counter values. */
	pmic_swfg_hal_get_coulomb_counts(&cc_up_counts, &cc_down_counts);

	/* Calculate coulomb counter differences in mC over the period since
	last error reset. */
	cc_delta_up_mc = pmic_swfg_hal_counts_to_mc(cc_up_counts -
		pmic_swfg_hal_instance.error_base_cc_up_counts);

	cc_delta_down_mc = pmic_swfg_hal_counts_to_mc(cc_down_counts -
		pmic_swfg_hal_instance.error_base_cc_down_counts);

	/* Calculate the offset error. */
	error_uc  = (s64)pmic_swfg_hal_instance.platform_data.
				offset_error_uc_per_s * (s64)error_period_sec;
	/* Add in gain the error for current IN to battery. */
	error_uc += (s64)pmic_swfg_hal_instance.platform_data.
				gain_error_1_uc_per_mc * (s64)cc_delta_up_mc;
	/* Add in the gain error for current OUT of the battery. */
	error_uc += (s64)pmic_swfg_hal_instance.platform_data.
				gain_error_2_uc_per_mc * (s64)cc_delta_down_mc;
	/*
	 * Convert error to mC for return value.
	 * NOTE: Standard C divide is not supported for 64 bit values.
	 */
	error_mc = (int)div_s64_rem(error_uc, SCALING_UC_TO_MC, &remainder);

	SWFGH_DEBUG_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_GET_ACCUMULATED_ERROR, error_mc);

	return error_mc;
}

/**
 * Resets vbat max and latched Ibat averages.
 */
static void pmic_swfg_hal_clear_vbat_max(void)
{
	SWFGH_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_CLR_VBAT_MAX);

	/* Clear latched vbat max and ibat averages. The bit field
	VBATMAXHW of VBATMAXHW_REG will also get written but this will have
	no effect, because the bit field VBATMAXLW of VBATMAXLW_REG, which
	would latch it, is not written */
	WARN_ON(vmm_pmic_reg_write(VBATMAXH_REG_VMM, CLR_VBATMAX_BIT));
}

/**
 * Refer to documentation in <sw_fuel_gauge_hal.h>
 */
static int pmic_swfg_hal_set(enum sw_fuel_gauge_hal_set_key key,
				union sw_fuel_gauge_hal_set_params params)
{
	struct pmic_swfg_hal_data *p_fg_hal = &pmic_swfg_hal_instance;

	/* Check that interface is called only when the driver is registered */
	BUG_ON(NULL == pmic_swfg_hal_instance.p_sw_fuel_gauge);

	switch (key) {
	case SW_FUEL_GAUGE_HAL_SET_COULOMB_IND_DELTA_THRESHOLD:
		pmic_swfg_hal_set_delta_threshold(p_fg_hal,
						params.delta_threshold_mc);
		break;

	case SW_FUEL_GAUGE_HAL_SET_ZERO_ACCUMULATED_CC_ERROR:
		/* No parameters. */
		pmic_swfg_hal_reset_accumulated_error(p_fg_hal);
		break;

	case SW_FUEL_GAUGE_HAL_SET_VBAT_MAX_CLEAR:
		/* No parameters. */
		pmic_swfg_hal_clear_vbat_max();
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
static int pmic_swfg_hal_get(enum sw_fuel_gauge_hal_get_key key,
				union sw_fuel_gauge_hal_get_params *p_params)
{
	int error = 0;

	struct pmic_swfg_hal_data *p_fg_hal = &pmic_swfg_hal_instance;

	SWFGH_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_GET, key);

	/* Check that interface is called only when the driver is registered */
	BUG_ON(NULL == pmic_swfg_hal_instance.p_sw_fuel_gauge);

	/* Check pointer to return parameters. */
	BUG_ON(NULL == p_params);

	switch (key) {
	case SW_FUEL_GAUGE_HAL_GET_CC_ACCUMULATED_ERROR:
		/* Calculate the accumulated error. */
		p_params->cc_acc_error_mc =
			pmic_swfg_hal_calc_accumulated_error(p_fg_hal);
		break;

	case SW_FUEL_GAUGE_HAL_GET_CC_BALANCED_COUNT:
		/* Call physical layer to read the value from the HW here. */
		p_params->cc_balanced_mc =
			pmic_swfg_hal_read_coulomb_counter(key);
		break;
	case SW_FUEL_GAUGE_HAL_GET_CC_UP_COUNT:
		/* Call physical layer to read the value from the HW here. */
		p_params->cc_up_mc =
			pmic_swfg_hal_read_coulomb_counter(key);
		break;
	case SW_FUEL_GAUGE_HAL_GET_CC_DOWN_COUNT:
		/* Call physical layer to read the value from the HW here. */
		p_params->cc_down_mc =
			pmic_swfg_hal_read_coulomb_counter(key);
		break;

	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE:
		/* Return value of HW Ibat 1 second average current in mA. */
		p_params->ibat_load_short_ma =
			pmic_swfg_hal_read_battery_current(false, false);
		break;

	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_SHORT_TERM_AVERAGE_AT_OCV:
		/* Return value of HW Ibat 1 second average current in
		mA at OCV measurement point, ie. the latched version. */
		p_params->ibat_load_short_at_ocv_ma =
			pmic_swfg_hal_read_battery_current(false, true);
		break;

	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE:
		/* Return value of HW Ibat 5 min average current in mA. */
		p_params->ibat_load_long_at_ocv_ma =
			pmic_swfg_hal_read_battery_current(true, false);
		break;

	case SW_FUEL_GAUGE_HAL_GET_IBAT_LOAD_LONG_TERM_AVERAGE_AT_OCV:
		/* Return value of HW Ibat 5 min average current in mA. */
		p_params->ibat_load_long_at_ocv_ma =
			pmic_swfg_hal_read_battery_current(true, true);
		break;

	case SW_FUEL_GAUGE_HAL_GET_COULOMB_IND_DELTA_THRESHOLD:
		/* If the delta threshold has been set, return it. */
		if (pmic_swfg_hal_instance.delta_threshold_set) {
			p_params->delta_threshold_mc =
				pmic_swfg_hal_instance.delta_threshold_mc;
			SWFGH_DEBUG_PARAM
				(SW_FUEL_GAUGE_DEBUG_HAL_GET_DELTA_THRESHOLD_MC,
				pmic_swfg_hal_instance.delta_threshold_mc);
		} else {
			SWFGH_DEBUG_NO_PARAM
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
 * @p_fg_hal	[in]	Device data
 * @np		[in]	Node pointer
 */
static int pmic_swfg_hal_get_platform_data(struct pmic_swfg_hal_data *p_fg_hal)
{
	u32 val;
	int ret;
	struct device_node *np = p_fg_hal->p_platform_device->dev.of_node;

	if (!IS_ENABLED(CONFIG_OF)) {
		struct sw_fuel_gauge_platform_data *p_platform_data =
			p_fg_hal->p_platform_device->dev.platform_data;

		/* Check pointer to platform data */
		BUG_ON(NULL == p_platform_data);

		memcpy(&p_fg_hal->platform_data, p_platform_data,
			sizeof(struct sw_fuel_gauge_platform_data));

		return 0;
	}

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
}

/**
 * Initialises coulomb counter IRQ
 *
 * @p_fg_hal	[in]	Device data
 */
static int pmic_swfg_hal_setup_irq(struct pmic_swfg_hal_data	*p_fg_hal)
{
	int ret;

	/* Coulomb counter interrupt is shared with ADC in hardware */
	p_fg_hal->irq = platform_get_irq_byname(p_fg_hal->p_platform_device,
					"PMIC_CC_HIRQ");

	if (IS_ERR_VALUE(p_fg_hal->irq)) {
		pr_err("(%s) failed to get irq no\n", __func__);
		return -ENXIO;
	}

	/* Must be able to wake up system from suspend
		to give low battery warning */
	ret = request_threaded_irq(p_fg_hal->irq, NULL,
		pmic_swfg_hal_cc_delta_irq_cb,
		IRQF_ONESHOT | IRQF_NO_SUSPEND,
		"pmic_coulomb_counter_irq", p_fg_hal);

	if (ret) {
		pr_err("(%s) failed requesting interrupt\n", __func__);
		return -EINVAL;
	}

	/* Prevent any interrupts before unmasking */
	pmic_swfg_hal_cctick_irq_en(p_fg_hal, false);

	/* clear any pending interrupt so far */
	ret = vmm_pmic_reg_write(ADCIRQ_VMM, CCTICK_M << CCTICK_O);
	if (ret)
		return ret;

	/* Unmask interrupt level 1 and 2 interrupts */
	ret = pmic_reg_set_field(MIRQLVL1_VMM,
		MADC_M << MADC_O, IRQ_UNMASK << MADC_O);
	if (ret)
		return ret;

	ret = pmic_reg_set_field(MADCIRQ_VMM,
			MCCTICK_M << MCCTICK_O, IRQ_UNMASK << MCCTICK_O);
	if (ret)
		return ret;

	return 0;
}

static int check_batt_was_removed(void)
{
	int ret;
	u32 codsrc;

	ret = vmm_pmic_reg_read(CODSRC_VMM, &codsrc);
	if (ret)
		return ret;

	if (!(codsrc & (BATTREP_M << BATTREP_O))) {
		pr_info("Battery not removed over power cycle, codsrc=0x%x\n",
								codsrc);
		return 0;
	}

	pr_info("Battery was removed over power cycle!, codsrc=0x%x\n",
								codsrc);
	/* Clear battery removed status */
	ret = vmm_pmic_reg_write(CODSRC_VMM, BATTREP_M << BATTREP_O);
	if (ret)
		return ret;

	/* Reset coulomb count to represent newly fitted battery */
	ret = vmm_pmic_reg_write(CC_CTRL0_VMM,
				CC_PERSIST_CLR_M << CC_PERSIST_CLR_O);
	if (ret)
		return ret;

	return 0;
}

/**
 * Initialises the driver, when the device has been
 * found.
 * @p_platform_dev	[in]	Pointer to platform device which triggered
				the probe.
 */
static int __init pmic_swfg_hal_probe(struct platform_device
							*p_platform_dev)
{
	int ret = 0;
	u32 reg;

	struct pmic_swfg_hal_data *p_fg_hal = &pmic_swfg_hal_instance;

	SWFGH_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_PROBE);

	dev_info(&p_platform_dev->dev, "%s: was executed\n", __func__);

	/* Store platform device in static instance. */
	p_fg_hal->p_platform_device = p_platform_dev;

	ret = pmic_swfg_hal_get_platform_data(p_fg_hal);

	if (ret)
		return ret;

	/* Read the PMIC HW revision */
	ret = vmm_pmic_reg_read(ID0_VMM, &reg);
	if (ret) {
		dev_err(&p_platform_dev->dev,
			"%s: fail to read PMIC HW revision\n", __func__);
		return ret;
	}

	p_fg_hal->hw_rev.major_rev = (reg >> MAJREV_O) & MAJREV_M;
	p_fg_hal->hw_rev.minor_rev = (reg >> MINREV_O) & MINREV_M;

	dev_dbg(&p_platform_dev->dev, "%s: PMIC majrev=0x%X, minrev=0x%X\n",
		__func__, p_fg_hal->hw_rev.major_rev,
		p_fg_hal->hw_rev.minor_rev);

	/* Enable coulomb counter HW */
	ret = pmic_reg_set_field(CC_CTRL0_VMM, CC_OFF_M << CC_OFF_O, 0);
	if (ret) {
		dev_err(&p_platform_dev->dev,
				"%s: fail to enable coulomb counter HW\n",
				__func__);
		return ret;
	}

	ret = pmic_swfg_hal_setup_irq(p_fg_hal);

	if (ret) {
		dev_err(&p_platform_dev->dev, "%s: irq setup failed\n",
				__func__);
		return ret;
	}

	/*
	 * Calculate the coulomb counter scaling factor from platform data.
	 *
	 * The SOC clock is given as 32758kHz/4 = 8.192kHz.
	 * Assume we have a constant DC current of i amperes (= 1000i mA)
	 * flowing through Rsense.
	 * The voltage at the input of the integrator is:
	 *
	 * Vin = (i A)*(Rsense mOhm) = (Rsense i ) mV
	 *
	 * Hence, to reach the threshold of 500 mV at its output and generate a
	 * clock pulse to the CCUP counter, the integrator will need a time of:
	 *
	 * (250mV / Rsense mOhm)/i clock pulses = (250mV/Rsense mOhm)*(1/8192) s
	 *
	 * For example, if Rsense is 10 mOhm, 1 LSB in the CCUP register is
	 * equivalent (in millicoulombs) to:
	 *
	 * (1000i mA)*( 250mV/10mOhm)/i )*(1/8192) s = 25000/8192 mAs = 3.05mC
	 */
	p_fg_hal->coulomb_count_scaling_uc =
		((SCALING_C_TO_UC * COULOMB_COUNTER_INCREMENT_THRESHOLD_MV)
			/ p_fg_hal->platform_data.sense_resistor_mohm)
			/ COULOMB_COUNTER_CLOCK_FREQ_HZ;

	SWFGH_DEBUG_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_CC_SCALE_UC,
			p_fg_hal->coulomb_count_scaling_uc);

	/* Calculate the coulomb threshold scaling factor (LSB) from coulomb
	counter scaling factor */
	p_fg_hal->threshold_count_scaling_mc =
		(pmic_swfg_hal_instance.coulomb_count_scaling_uc *
			COULOMB_COUNTER_DELTA_THRESHOLD_SCALING)
			/ SCALING_UC_TO_MC;

	SWFGH_DEBUG_PARAM
		(SW_FUEL_GAUGE_DEBUG_HAL_CC_THRESHOLD_SCALE_MC,
			p_fg_hal->threshold_count_scaling_mc);

	ret = check_batt_was_removed();
	if (ret) {
		dev_err(&p_platform_dev->dev,
			"%s: fail to check whether battery was removed\n",
			__func__);
		goto bat_removed_check_failed;
	}

	/* Read the initial values of both coulomb counters. */
	pmic_swfg_hal_init_accumulated_error(p_fg_hal);

	/* Register the HAL with the SW Fuel Gauge */
	BUG_ON(sw_fuel_gauge_register_hal(&sc_pmic_swfg_hal,
				&p_fg_hal->p_sw_fuel_gauge));

	dev_info(&p_platform_dev->dev, "%s OK\n", __func__);

	return 0;

bat_removed_check_failed:
	free_irq(p_fg_hal->irq, p_fg_hal);
	return ret;
}

/**
 * Release allocated resources on device removal.
 */
static int __exit pmic_swfg_hal_remove(struct platform_device
							*p_platform_dev)
{
	SWFGH_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_REMOVE);

	pmic_swfg_hal_cctick_irq_en(&pmic_swfg_hal_instance, false);

	free_irq(pmic_swfg_hal_instance.irq, &pmic_swfg_hal_instance);

	/* Delete allocated resources and mark driver as uninitialised. */
	if (NULL != pmic_swfg_hal_instance.p_sw_fuel_gauge)
		pmic_swfg_hal_instance.p_sw_fuel_gauge = NULL;

	return 0;
}

/**
 * Called when the system is attempting to suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int pmic_swfg_hal_suspend(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Nothing to do here except logging */
	SWFGH_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_SUSPEND);
	return 0;
}

/**
 * Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int pmic_swfg_hal_resume(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Nothing to do here */
	SWFGH_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_RESUME);
	return 0;
}

const struct dev_pm_ops pmic_swfg_hal_pm = {
	.suspend = pmic_swfg_hal_suspend,
	.resume = pmic_swfg_hal_resume,
};


static const struct of_device_id pmic_swfg_hal_of_match[] = {
	{
		.compatible = "intel,pmic_swfg_hal",
	},
	{}
};

MODULE_DEVICE_TABLE(of, pmic_swfg_hal_of_match);

/**
 * Driver structure for SW fuel gauge HAL .
 */
static struct platform_driver pmic_swfg_hal_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pmic_swfg_hal_of_match),
		.pm = &pmic_swfg_hal_pm,
	},
	.probe = pmic_swfg_hal_probe,
	.remove = pmic_swfg_hal_remove,
};

/**
 * SW fuel gauge HAL device init function.
 * returns		0 for success, or error code.
 */
static int __init pmic_swfg_hal_init(void)
{
	pr_info("%s: was executed\n", __func__);

	/* NOTE: Must initialise debug data spinlock before any logging. */
	spin_lock_init(&pmic_swfg_hal_debug_data.lock);
	SWFGH_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_INIT);

	return platform_driver_register(&pmic_swfg_hal_driver);
}

/**
 * SW fuel gauge HAL  device deinit function.
 */
static void __exit pmic_swfg_hal_exit(void)
{
	SWFGH_DEBUG_NO_PARAM(SW_FUEL_GAUGE_DEBUG_HAL_EXIT);
	return platform_driver_unregister(&pmic_swfg_hal_driver);
}

device_initcall_sync(pmic_swfg_hal_init);
module_exit(pmic_swfg_hal_exit);

MODULE_DESCRIPTION("SoFIA LTE PMIC SW Fuel Gauge HAL Driver");
MODULE_LICENSE("GPL v2");
