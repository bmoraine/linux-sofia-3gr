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

#include <linux/string.h>
#include <linux/power/sw_fuel_gauge_debug.h>
#include <linux/power/sw_fuel_gauge_nvs.h>

/* Non-volatile Memory interfaces */
#ifdef CONFIG_NVM
#include <linux/crc16.h>
#include <nvm/nvm.h>
#include "../nvm/sw_fuel_gauge_nvm_dyn_driverif.h"

/* NVM defines */
#define SW_FUEL_GAUGE_NVM_VERSION	1
#define SW_FUEL_GAUGE_NVM_REVISION	1

/* Checksum values */
#define SWFG_NVM_CHECKSUM_UNINITIALIZED	0xFFFF
#define SWFG_NVM_CHECKSUM_CAL_POINT_INVALID 0xFFFE

struct sw_fuel_gauge_nvs_data {
	bool sw_fuel_gauge_nvs_up;
	void (*sw_fg_nvs_ready_cb)(void);
};

static struct sw_fuel_gauge_nvs_data sw_fuel_gauge_nvs = {
	.sw_fuel_gauge_nvs_up = false,
	.sw_fg_nvs_ready_cb = NULL,
};

/* Array to collect debug data */
static struct sw_fuel_gauge_debug_data sw_fuel_gauge_nvs_debug_data = {
	.lock = __SPIN_LOCK_UNLOCKED(sw_fuel_gauge_nvs_debug_data.lock),
};

/* Macro to trace and log debug event and data. */
#define SW_FUEL_GAUGE_NVS_DEBUG_PARAM(_event, _param) \
	SWFG_DEBUG(sw_fuel_gauge_nvs_debug_data, _event, _param)

/* Macro to trace and log debug event without a parameter. */
#define SW_FUEL_GAUGE_NVS_DEBUG_NO_PARAM(_event) \
	SWFG_DEBUG(sw_fuel_gauge_nvs_debug_data, _event, 0)


/**
 * sw_fuel_gauge_nvm_is_initialized -	Checks if the fuel gauge group
 *					in NVM is initialized and the
 *					version is up to date.
 * Returns				true if the NVM group is initialized
 *					and up to date, otherwise false.
 */
static bool sw_fuel_gauge_nvm_is_initialized(void)
{
	T_NVM_RETURNCODE nvm_result = NVM_OK;
	u32 no_of_bytes = sizeof(T_SOC_CAL_PNT_NVM);
	T_VERSION  version  = 0;
	T_REVISION revision = 0;

	/* First check the version in NVM to ensure we are in sync. */
	nvm_result = nvm_get_ver_rev_lgt(NVM_DYN_SW_FUEL_GAUGE, &version,
		&revision, &no_of_bytes);

	if (NVM_OK != nvm_result) {
		pr_err("NVM error: fetching version info failed - result: %d\n",
				nvm_result);
		return false;
	} else {
		SW_FUEL_GAUGE_NVS_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_NVM_VERSION, version);

		/* If the data in NVM is older than what is in this file,
		 * or NVM has not been initialised, update NVM
		 */
		if ((SW_FUEL_GAUGE_NVM_VERSION	> version)	||
			(SW_FUEL_GAUGE_NVM_REVISION > revision) ||
			(0xFFFFFFFF == version) ||
			(0xFFFFFFFF == revision)) {
			pr_err("NVM error: outdated version %d, revision %d\n",
					version, revision);
			return false;
		}

		return true;

	}
}

/**
 * sw_fuel_gauge_nvm_init -	Initializes the fuel gauge group in NVM if it's
 *					not initialized yet or not up to date.
 * Returns				true if the initialization was
 *					successful or not necessary,
 *					otherwise false.
 */
static bool sw_fuel_gauge_nvm_init(void)
{
	T_NVM_RETURNCODE nvm_result = NVM_OK;
	u32 no_of_bytes = sizeof(T_SOC_CAL_PNT_NVM);

	T_SOC_CAL_PNT_NVM fuel_gauge_cal_data = {
		.cc_charge_mc = 0,
		.cc_discharge_mc = 0,
		.cc_balanced_mc = 0,
		.cc_error_mc = 0,
		.rtc_time_sec = 0,
		.soc_permil = 0,
		.soc_error_permil = 0,
		.qmax_n_minus_2 = 0,
		.qmax_n_minus_2_err_permil = 0,
		.qmax_n_minus_1 = 0,
		.qmax_n_minus_1_err_permil = 0,
		.checksum = SWFG_NVM_CHECKSUM_UNINITIALIZED,
	};

	if (!sw_fuel_gauge_nvm_is_initialized()) {

		/* The data is not in NVM, so initialize it */
		nvm_result = nvm_init(NVM_DYN_SW_FUEL_GAUGE, /* group id */
			(U8 *) &fuel_gauge_cal_data, /* src */
			0, /* offset */
			no_of_bytes,
			SW_FUEL_GAUGE_NVM_VERSION,
			SW_FUEL_GAUGE_NVM_REVISION);

		if (NVM_OK != nvm_result) {
			pr_err("NVM error: init failed - result: %d\n",
				nvm_result);
			return false;
		} else
			SW_FUEL_GAUGE_NVS_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_NVM_INITIALIZED);

	}

	return true;

}

/**
 * sw_fuel_gauge_nvm_read_group_if_valid -	Checks an NVM group and reads
 *						its contents if valid.
 * @group_id					[in] Which calibration point
 *						data to read.
 * @p_cal_data					[out] Coulomb counter data read.
 * Returns					true if the data is valid,
 *						otherwise false.
 */
static bool sw_fuel_gauge_nvm_read_group_if_valid(
				enum sw_fuel_gauge_nvs_write_type nvm_group_id,
				struct soc_cal_point *p_cal_data)
{
	T_NVM_RETURNCODE nvm_result = NVM_OK;

	/* The fields are initialized to zero to be sure if
	 * NVM was read correctly or not. */
	T_SOC_CAL_PNT_NVM fuel_gauge_cal_data  = {
		.cc_charge_mc = 0,
		.cc_discharge_mc = 0,
		.cc_balanced_mc = 0,
		.cc_error_mc = 0,
		.rtc_time_sec = 0,
		.soc_permil = 0,
		.soc_error_permil = 0,
		.qmax_n_minus_2 = 0,
		.qmax_n_minus_2_err_permil = 0,
		.qmax_n_minus_1 = 0,
		.qmax_n_minus_1_err_permil = 0,
		.checksum = SWFG_NVM_CHECKSUM_UNINITIALIZED,
	};

	if (!sw_fuel_gauge_nvs.sw_fuel_gauge_nvs_up) {
		pr_err("NVM error: NVM not running\n");
		return false;
	}

	/* Return if invalid group ID. */
	if (nvm_group_id != SW_FUEL_GAUGE_NVS_WRITE_DEFERRED &&
		nvm_group_id != SW_FUEL_GAUGE_NVS_WRITE_IMMEDIATE) {
		pr_err("NVM error: Invalid group ID: %d\n", nvm_group_id);
		return false;
	}

	if (sw_fuel_gauge_nvm_is_initialized()) {
		u16 crc = 0;
		u32 no_of_bytes = sizeof(T_SOC_CAL_PNT_NVM);

		/* NVM has been initialized -> let's get the data. */
		nvm_result = nvm_read(NVM_DYN_SW_FUEL_GAUGE,
			(u8 *)&fuel_gauge_cal_data, 0, no_of_bytes);
		if (NVM_OK != nvm_result) {
			pr_err("NVM error: Reading config data failed\n");
			return false;
		}
		SW_FUEL_GAUGE_NVS_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_NVM_DATA_RETRIEVED, no_of_bytes);

		if (fuel_gauge_cal_data.checksum
				== SWFG_NVM_CHECKSUM_CAL_POINT_INVALID) {
			pr_debug("%s:Invalid Calibration point\n", __func__);
			return false;
		}

		/* Map the data to our internal data type. */
		p_cal_data->cc_balanced_mc = fuel_gauge_cal_data.cc_balanced_mc;
		p_cal_data->cc_down_mc = fuel_gauge_cal_data.cc_discharge_mc;
		p_cal_data->cc_up_mc = fuel_gauge_cal_data.cc_charge_mc;
		p_cal_data->full_battery_cap_mah =
				fuel_gauge_cal_data.qmax_n_minus_1;
		p_cal_data->rtc_time_sec =
				(time_t) fuel_gauge_cal_data.rtc_time_sec;
		p_cal_data->soc_error_permil =
				fuel_gauge_cal_data.soc_error_permil;
		p_cal_data->soc_permil = fuel_gauge_cal_data.soc_permil;

		/* CRC check to verify sanity of data */
		crc = crc16(0, (u8 *) p_cal_data, sizeof(struct soc_cal_point));

		if (crc != fuel_gauge_cal_data.checksum) {
			pr_err("NVM error: Checksum mismatch - read 0x%x, expected 0x%x\n",
					fuel_gauge_cal_data.checksum, crc);
			/* The checksum will be invalid if we have never
			 * written a calibration point to NVM.
			 */
			return false;
		}

		return true;
	} else {
		return false;
	}

}

/**
 * sw_fuel_gauge_nvm_retrieve_last_calibration_point -	Retrieves the
 *						most recent NVM stored
 *						calibration data if it is valid.
 * Returns					true if valid data was found,
 *						otherwise false.
 */
static bool sw_fuel_gauge_nvm_retrieve_last_calibration_point(
				struct soc_cal_point *p_last_soc_cal,
				struct soc_cal_point *p_last_immediate_cal)
{
	/* Initial setting of NULL indicates no valid data */
	struct soc_cal_point cal_data_immediate;
	struct soc_cal_point cal_data_deferred;
	bool immediate_valid = false;
	bool deferred_valid = false;

	if (!sw_fuel_gauge_nvs.sw_fuel_gauge_nvs_up) {
		pr_err("NVM error: NVM not running\n");
		return false;
	}

	/* Check and read NVM data for valid data. */
	immediate_valid =
		sw_fuel_gauge_nvm_read_group_if_valid(
			SW_FUEL_GAUGE_NVS_WRITE_IMMEDIATE, &cal_data_immediate);

	/* Keeping this function in to keep the logic */
	deferred_valid =
		sw_fuel_gauge_nvm_read_group_if_valid(
			SW_FUEL_GAUGE_NVS_WRITE_DEFERRED, &cal_data_deferred);

	if (immediate_valid && deferred_valid) {
		/* Both data sets are valid in NVM. Use the newest for SoC. */
		if (cal_data_deferred.rtc_time_sec >
					cal_data_immediate.rtc_time_sec) {
			/* Use data from deferred write. */
			*p_last_soc_cal = cal_data_deferred;
			SW_FUEL_GAUGE_NVS_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_NVM_USE_DEFERRED_CAL_POINT);
		} else {
			/* Use data from immediate write. */
			*p_last_soc_cal = cal_data_immediate;
			SW_FUEL_GAUGE_NVS_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_NVM_USE_IMMEDIATE_CAL_POINT);
		}
	} else if (immediate_valid) {
		/* Only data from immediate write is valid in NVM. Use it for
		initial SoC. */
		SW_FUEL_GAUGE_NVS_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_NVM_USE_IMMEDIATE_CAL_POINT);
		*p_last_soc_cal = cal_data_immediate;
	} else if (deferred_valid) {
		/* Only data from deferred write valid in NVM. Use it for
		initial SoC. */
		SW_FUEL_GAUGE_NVS_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_NVM_USE_DEFERRED_CAL_POINT);
		*p_last_soc_cal = cal_data_deferred;
	} else {
		/* Both NVM groups were invalid */
		SW_FUEL_GAUGE_NVS_DEBUG_NO_PARAM(
				SW_FUEL_GAUGE_DEBUG_NVM_NO_VALID_CAL_POINT);
		return false;
	}

	if (immediate_valid) {
		/* When the immediate NVM data is valid,
		 * update the static data copy. */
		*p_last_immediate_cal = cal_data_immediate;
	}

	return true;
}

/**
 * sw_fuel_gauge_nvm_store_last_calibration_point -Writes latets
 *						calibration point to NVM. If an
 *						immediate write is specified,
 *						then the data will be committed
 *						to NVM, otherwise the RAM copy
 *						only is changed.
 *
 * @nvm_group_id				[in] Which type of calibration
 *						point data to store.
 */
static bool sw_fuel_gauge_nvm_store_last_calibration_point(
				enum sw_fuel_gauge_nvs_write_type nvm_group_id,
				struct soc_cal_point *p_last_soc_cal,
				struct soc_cal_point *p_last_immediate_cal)
{
	T_NVM_RETURNCODE nvm_result = NVM_OK;
	u32 no_of_bytes = sizeof(T_SOC_CAL_PNT_NVM);
	struct soc_cal_point current_nvm_cal_point;
	bool nvm_data_valid = false;

	/* Map the data to the NVM data type. */
	T_SOC_CAL_PNT_NVM fuel_gauge_cal_data  = {
		.cc_charge_mc = p_last_soc_cal->cc_up_mc,
		.cc_discharge_mc = p_last_soc_cal->cc_down_mc,
		.cc_balanced_mc = p_last_soc_cal->cc_balanced_mc,
		.cc_error_mc = 0,
		.rtc_time_sec = (long long) p_last_soc_cal->rtc_time_sec,
		.soc_permil = p_last_soc_cal->soc_permil,
		.soc_error_permil = p_last_soc_cal->soc_error_permil,
		.qmax_n_minus_2 = 0,
		.qmax_n_minus_2_err_permil = 0,
		.qmax_n_minus_1 = p_last_soc_cal->full_battery_cap_mah,
		.qmax_n_minus_1_err_permil = 0,
		.checksum = 0,
	};

	if (!sw_fuel_gauge_nvs.sw_fuel_gauge_nvs_up) {
		pr_err("NVM error: NVM not running\n");
		return false;
	}

	if (!sw_fuel_gauge_nvm_is_initialized()) {
		if (!sw_fuel_gauge_nvm_init()) {
			pr_err("NVM error: NVM could not be initialized\n");
			return false;
		}
	}

	/* Calculate and set the CRC checksum. */
	fuel_gauge_cal_data.checksum = crc16(0, (u8 *) p_last_soc_cal,
			sizeof(struct soc_cal_point));
	SW_FUEL_GAUGE_NVS_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_NVM_CALCULATED_CHECKSUM,
				fuel_gauge_cal_data.checksum);

	switch (nvm_group_id) {
	case SW_FUEL_GAUGE_NVS_WRITE_DEFERRED:

		nvm_result = nvm_write(NVM_DYN_SW_FUEL_GAUGE,
			(u8 *) &fuel_gauge_cal_data, 0, no_of_bytes);
		if (NVM_OK != nvm_result)
			pr_err("NVM error: write failed. Result %d\n",
					nvm_result);
		else
			SW_FUEL_GAUGE_NVS_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_NVM_WRITE_DEF_CAL_POINT_OK,
				no_of_bytes);

		break;
	case SW_FUEL_GAUGE_NVS_WRITE_IMMEDIATE:
		/* Check if the calibration data in NVM is already up to
		 * date to avoid double writes.
		 */
		nvm_data_valid =
			sw_fuel_gauge_nvm_read_group_if_valid(
				SW_FUEL_GAUGE_NVS_WRITE_DEFERRED,
				&current_nvm_cal_point);

		if (nvm_data_valid &&
				current_nvm_cal_point.rtc_time_sec ==
				p_last_immediate_cal->rtc_time_sec) {
			/* The current calibration point is already stored
			 * in NVM -> abort */
			SW_FUEL_GAUGE_NVS_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_NVM_NO_WRITE_CAL_POINT_UP_TO_DATE);
			/* Return true here since this is not an
			 * error condition. */
			return true;
		} else {
			nvm_result = nvm_write(NVM_DYN_SW_FUEL_GAUGE,
				(u8 *) &fuel_gauge_cal_data, 0, no_of_bytes);

			if (NVM_OK != nvm_result) {
				pr_err("NVM error: write failed. Result %d\n",
						nvm_result);
				return false;
			} else
				SW_FUEL_GAUGE_NVS_DEBUG_PARAM(
				SW_FUEL_GAUGE_DEBUG_NVM_WRITE_IMM_CAL_POINT_OK,
				no_of_bytes);
		}

		/* Record the last calibration point written.
		 *  The values are needed to decide when to update outdated
		 *  immediate NVM data. */
		*p_last_immediate_cal = *p_last_soc_cal;

		break;
	default:
		pr_err("NVM error: unsupported group ID: 0x%x\n", nvm_group_id);
		BUG();
		break;
	}

	return true;
}

/**
 * sw_fuel_gauge_nvs_ready_callback -	Called by NVM on NVM state changes.
  *					In case that NVM is ready for read and
  *					write, we retrieve the calibration data
  *					stored in NVM.
 */
static void sw_fuel_gauge_nvs_ready_callback(T_NVM_STATE state)
{
	SW_FUEL_GAUGE_NVS_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_NVM_NVS_CALLBACK, state);

	switch (state) {
	case NVM_IDLE:
	case NVM_INIT_TASK:
		/* NVM not ready for read or write operations -> return. */
		sw_fuel_gauge_nvs.sw_fuel_gauge_nvs_up = false;
		return;
	case NVM_INIT:
	case NVM_ACTIVE:
		/* NVM ready for read and write -> retrieve the calibration data
		 * unless we have already done it.
		 */
		if (!sw_fuel_gauge_nvs.sw_fuel_gauge_nvs_up) {
			sw_fuel_gauge_nvs.sw_fuel_gauge_nvs_up = true;
			if (sw_fuel_gauge_nvs.sw_fg_nvs_ready_cb != NULL)
				sw_fuel_gauge_nvs.sw_fg_nvs_ready_cb();
			else
				pr_err("NVM error: no NVM callback set!\n");
		}
		 break;
	case NVM_DEACTIVATE:
	case NVM_DEACTIVATED_READ_ONLY:
		/* ToDo - is there more action needed in case NVM
		 * deactivates? */
		sw_fuel_gauge_nvs.sw_fuel_gauge_nvs_up = false;
		break;
	default:
		break;
	}

}

#endif /* CONFIG_NVM */

/**
 * sw_fuel_gauge_nvs_retrieve_last_calibration_point -	Retrieves the
 *						most recent NVM stored
 *						calibration data if it is valid.
 * Returns					true if valid data was found,
 *						otherwise false.
 */
bool sw_fuel_gauge_nvs_retrieve_last_calibration_point(
				struct soc_cal_point *p_last_soc_cal,
				struct soc_cal_point *p_last_immediate_cal)
{

	if (NULL == p_last_soc_cal ||
		NULL == p_last_immediate_cal) {
		pr_err("NVS error: NULL pointer\n");
		return false;
	}

#ifdef CONFIG_NVM
	return sw_fuel_gauge_nvm_retrieve_last_calibration_point(p_last_soc_cal,
							p_last_immediate_cal);
#else
	pr_err("%s: NVM error: NVM expected to be enabled\n", __func__);
	return false;
#endif /* CONFIG_NVM */
}

/**
 * sw_fuel_gauge_nvs_store_last_calibration_point -Writes latets
 *						calibration point to NVM. If an
 *						immediate write is specified,
 *						then the data will be committed
 *						to NVM, otherwise the RAM copy
 *						only is changed.
 *
 * @nvm_group_id				[in] Which type of calibration
 *						point data to store.
 */
bool sw_fuel_gauge_nvs_store_last_calibration_point(
				enum sw_fuel_gauge_nvs_write_type nvs_group_id,
				struct soc_cal_point *p_last_soc_cal,
				struct soc_cal_point *p_last_immediate_cal)
{
	if (NULL == p_last_soc_cal ||
		NULL == p_last_immediate_cal) {
		pr_err("NVS error: NULL pointer\n");
		return false;
	}

#ifdef CONFIG_NVM
	return sw_fuel_gauge_nvm_store_last_calibration_point(nvs_group_id,
					p_last_soc_cal,
					p_last_immediate_cal);
#else
	pr_err("%s: NVM error: NVM expected to be enabled\n", __func__);
	return false;
#endif /* CONFIG_NVM */
}

/**
 * sw_fuel_gauge_register_nvs_ready_cb -Registers a callback function with
 *					the NVS so the client can be notified
 *					when theNVS is ready for access.
 *
 * @p_func				[in] The callback function to be executed on
 *					NVS ready state
 */

bool sw_fuel_gauge_register_nvs_ready_cb(void (*p_func)(void))
{
#ifdef CONFIG_NVM
	T_NVM_RETURNCODE ret;
#endif /* CONFIG_NVM */

	if (NULL == p_func) {
		pr_err("NVS error: NULL pointer\n");
		return false;
	}

#ifdef CONFIG_NVM
	sw_fuel_gauge_nvs.sw_fg_nvs_ready_cb = p_func;

	SW_FUEL_GAUGE_NVS_DEBUG_NO_PARAM(
			SW_FUEL_GAUGE_DEBUG_REGISTER_NVM_CB);
	ret = nvm_register_state_change_notify(NVM_DYN_SW_FUEL_GAUGE,
		sw_fuel_gauge_nvs_ready_callback, NVM_NOTIFY_ALL);

	if (ret == NVM_OK)
		return true;
	else
		return false;

#else
	pr_err("%s: NVM error: NVM expected to be enabled\n", __func__);
	return false;
#endif /* CONFIG_NVM */
}

bool sw_fuel_guage_nvs_cal_point_invalidate(void)
{
#ifdef CONFIG_NVM
	T_NVM_RETURNCODE nvm_result = NVM_OK;
	u32 no_of_bytes = sizeof(T_SOC_CAL_PNT_NVM);

	/* Map the data to the NVM data type. */
	T_SOC_CAL_PNT_NVM fuel_gauge_cal_data  = {
		.cc_charge_mc = 0,
		.cc_discharge_mc = 0,
		.cc_balanced_mc = 0,
		.cc_error_mc = 0,
		.rtc_time_sec = 0,
		.soc_permil = 0,
		.soc_error_permil = 0,
		.qmax_n_minus_2 = 0,
		.qmax_n_minus_2_err_permil = 0,
		.qmax_n_minus_1 = 0,
		.qmax_n_minus_1_err_permil = 0,
		.checksum = SWFG_NVM_CHECKSUM_CAL_POINT_INVALID,
	};

	if (!sw_fuel_gauge_nvs.sw_fuel_gauge_nvs_up) {
		pr_err("NVM error: NVM not running\n");
		return false;
	}

	/* If NVM is not initialized, we don't invalidate
	 * calibration point in NVM, because there is anyway no
	 * stored calibration point to be invalidated. */
	if (!sw_fuel_gauge_nvm_is_initialized()) {
		pr_debug("NVM is not initialised on invalidation request\n");
		return true;
	}

	nvm_result = nvm_write(NVM_DYN_SW_FUEL_GAUGE,
			(u8 *) &fuel_gauge_cal_data, 0, no_of_bytes);
	if (NVM_OK != nvm_result) {
		pr_err("NVM error: write failed. Result %d\n",
				nvm_result);
		return false;
	} else
		SW_FUEL_GAUGE_NVS_DEBUG_PARAM(
			SW_FUEL_GAUGE_DEBUG_NVM_CAL_POINT_INVALIDATED,
			no_of_bytes);
	return true;
#else
	pr_err("%s: NVM error: NVM expected to be enabled\n", __func__);
	return false;
#endif /* CONFIG_NVM */
}

