/**
 * -------------------------------------------------------------------------
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
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

#ifndef _SW_FUEL_GAUGE_NVS_H
#define _SW_FUEL_GAUGE_NVS_H

/* Type of NVS write required. */
enum sw_fuel_gauge_nvs_write_type {
	SW_FUEL_GAUGE_NVS_WRITE_IMMEDIATE,
	SW_FUEL_GAUGE_NVS_WRITE_DEFERRED,
};

/*					State of Charge (SoC) calibration point
 *					data.
 * @cc_up_mc			Coulomb counter UP register value (mC)
 * @c_down_mc			Coulomb counter DOWN register (mC)
 * @cc_balanced_mc		Coulomb counter balanced value (mC)
 * @rtc_time_sec		Monotonic RTC Timestamp for these
 *				values (Seconds)
 * @soc_permil			Battery state of charge (Permil)
 * @soc_error_permil		Battery state of charge error (Permil)
 * @full_battery_cap_mah	Battery capacity when fully charged (mAh)
 */
struct soc_cal_point {
	int	cc_up_mc;
	int	cc_down_mc;
	int	cc_balanced_mc;
	time_t	rtc_time_sec;
	int	soc_permil;
	int	soc_error_permil;
	u32	full_battery_cap_mah;
};

/**
 *					Retrieves the most recent NVS stored
 *					calibration data if it is valid.
 * @p_last_soc_cal			[in] Pointer to where the calibration data
 *					should be written.
 * @p_last_immediate_cal	[in] Pointer to the calibration data to be
 *					stored (only in case we have a valid
 *					immediate calibration point).
 * Returns				true if valid data was found,
 *					otherwise false.
 */
bool sw_fuel_gauge_nvs_retrieve_last_calibration_point(
				struct soc_cal_point *p_last_soc_cal,
				struct soc_cal_point *p_last_immediate_cal);

/**
 *					Writes latest calibration point
 *					to NVS. If an immediate write
 *					is specified, then the data will
 *					be committed to NVS, otherwise the
 *					RAM only is changed immediately.
 * @nvs_group_id			[in] Which type of calibration
 *					point data to store.
 * @p_last_soc_cal			[in] Pointer to the calibration data to be
 *					stored.
 * @p_last_immediate_cal		[in] Pointer to the immediate calibration
 *					point. Needed to check if the SOC
 *					in NVM is already up to date.
 * Returns				true if writing to NVS was
 *					successful, otherwise false.
 */
bool sw_fuel_gauge_nvs_store_last_calibration_point(
				enum sw_fuel_gauge_nvs_write_type nvs_group_id,
				struct soc_cal_point *p_last_soc_cal,
				struct soc_cal_point *p_last_immediate_cal);

/**
 *					Registers a callback function with
 *					the NVS so the client can be notified
 *					when the NVS is ready for access.
 *
 * @p_func			[in] The callback function to be executed on
 *					NVS ready state
 * Returns			true if the callback could be
 *					registered, otherwise false (e.g.
 *					in case no NVS is present).
 */
bool sw_fuel_gauge_register_nvs_ready_cb(void (*p_func)(void));

/**
 * Function to enable or disable debug logging.
 *
 * @debug_val			[in] The new debug log
 *						setting.
 */
void sw_fuel_gauge_nvs_dbg_set(int debug_val);

/*
 * Invalidate the last calibration point
 * in the NVM.
 *
 * Returns				true if the cal. point in NVM
 *					was successfully invalidated,
 *					otherwise false.
 */
bool sw_fuel_guage_nvs_cal_point_invalidate(void);

#endif /* _SW_FUEL_GAUGE_NVS_H */
