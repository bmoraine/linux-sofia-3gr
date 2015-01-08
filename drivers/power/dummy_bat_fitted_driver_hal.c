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

#define DRIVER_NAME						"ag6x0_bat_hal"
#define pr_fmt(fmt) DRIVER_NAME": "fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/stat.h>

#include <linux/power/battery_id.h>

/* Size of debug data array (has to be power of 2!!!) */
#define BAT_DRV_HAL_DEBUG_DATA_SIZE (1<<6)

/* Macro to trace and log debug data internally. Jiffy resolution is adequate
for Bat Drv HAL */
#define BAT_DRV_HAL_DEBUG(_array, _event, _param) \
do { \
	spin_lock(&_array.lock); \
	_array.log_array[_array.index].time_stamp = jiffies; \
	_array.log_array[_array.index].event = (_event); \
	_array.log_array[_array.index].param = (long)(_param); \
	_array.index++; \
	_array.index &= (BAT_DRV_HAL_DEBUG_DATA_SIZE-1); \
	spin_unlock(&_array.lock); \
	pr_debug("%s 0x%lx  dec=%ld\n", \
		#_event, (unsigned long)_param, (long)_param); \
} while (0)

/** Events for use in debug and tracing. */
enum bat_drv_hal_debug_event {
	BAT_DRV_HAL_DEBUG_EVENT_PROBE,
	BAT_DRV_HAL_DEBUG_EVENT_REMOVE,

	BAT_DRV_HAL_DEBUG_EVENT_REGISTER_HAL,
	BAT_DRV_HAL_DEBUG_EVENT_BAT_NOTIFICATION,
	BAT_DRV_HAL_DEBUG_EVENT_EXTERNAL_SUPPLY_CHANGED,

	BAT_DRV_HAL_DEBUG_EVENT_BAT_NOT_FITTED,
	BAT_DRV_HAL_DEBUG_EVENT_BAT_ID_TYPE,
	BAT_DRV_HAL_DEBUG_EVENT_BAT_REPORT_REPEATED,
	BAT_DRV_HAL_DEBUG_EVENT_BAT_CHANGED,
};

/**
 * struct bat_drv_hal_debug_data - Structure to collect debug data
 * @lock		Spinlock for atomic access
 * @index		Index of logging array
 * @log_array		Debug data logging array
 *	@time_stamp	System Time Stamp in Jiffies
 *	@event		Event which occurred
 *	@param		General purpose parameter
 */
struct bat_drv_hal_debug_data {
	spinlock_t			lock;
	u32				index;
	struct {
		u32				time_stamp;
		enum bat_drv_hal_debug_event	event;
		long			param;
	} log_array[BAT_DRV_HAL_DEBUG_DATA_SIZE];
};

/* Macro to trace and log debug event and data. */
#define BAT_DRV_HAL_DEBUG_PARAM(_event, _param) \
		BAT_DRV_HAL_DEBUG(bat_drv_hal_debug_info, _event, _param)

/* Macro to trace and log debug event without a parameter. */
#define BAT_DRV_HAL_DEBUG_NO_PARAM(_event) \
		BAT_DRV_HAL_DEBUG(bat_drv_hal_debug_info, _event, 0)

/**
 * struct bat_drv_hal_data - Battery Driver Hal control structure
 * @initialised		Driver initialisation state
 * @p_idi_device	Pointer to IDI device
 * @battery_id		Battery ID data
 */
struct bat_drv_hal_data {
	bool			initialised;
	struct ps_batt_chg_prof	battery_id;
};

static struct ps_pse_mod_prof bat_drv_hal_prof = {
	.batt_id = "STANDRD",
	.battery_type = POWER_SUPPLY_TECHNOLOGY_LION,
	.capacity = 1630,
	.voltage_max = 4250,
	.chrg_term_ma = 82,
	.low_batt_mv = 3200,
	.disch_tmp_ul = 60,
	.disch_tmp_ll = -20,
	.in_chrg_allowed_zone = false,
	.validated = BPROF_NOT_VALIDATED,
	.min_temp = 0,
	.min_temp_restart = 3,
	.max_temp_restart = 42,

	.num_temp_bound = 1,
	.temp_range = {
		{
			.max_temp = 45,
			.full_chrg_vol = 4160,
			.full_chrg_cur = 850,
			.maint_chrg_vol_ul = 4160,
			.charging_res_cap = 98,
			.maint_chrg_cur = 850,
		},
	},
};

/* Bat Driver Hal instance */
static struct bat_drv_hal_data bat_drv_hal_instance = {
	.battery_id = {
		.chrg_prof_type = PSE_MOD_CHRG_PROF,
	},
	.initialised = false,
};

/* Array to collect debug data */
static struct bat_drv_hal_debug_data bat_drv_hal_debug_info;

/**
 * bat_drv_hal_probe - Initialises the driver, when the device has been found.
 */
static int __init bat_drv_hal_probe(struct platform_device *p_platform_dev)
{
	/* NOTE: Must initialise debug data spinlock before any logging. */
	spin_lock_init(&bat_drv_hal_debug_info.lock);

	BAT_DRV_HAL_DEBUG_NO_PARAM(BAT_DRV_HAL_DEBUG_EVENT_PROBE);

	bat_drv_hal_instance.initialised = true;

	bat_drv_hal_instance.battery_id.batt_prof = &bat_drv_hal_prof;

	/* Signal the presence of the default battery. */
	battery_prop_changed(POWER_SUPPLY_BATTERY_INSERTED,
				&bat_drv_hal_instance.battery_id);

	pr_info("dummy hal probe OK\n");

	return 0;
}

/**
 * bat_drv_hal_remove - Release allocated resources.
 */
static int __exit bat_drv_hal_remove(struct platform_device *p_platform_dev)
{
	BAT_DRV_HAL_DEBUG_NO_PARAM(BAT_DRV_HAL_DEBUG_EVENT_REMOVE);

	/* Delete allocated resources and mark driver as uninitialised. */
	if (bat_drv_hal_instance.initialised)
		bat_drv_hal_instance.initialised = false;

	return 0;
}

static const struct of_device_id bat_drv_hal_of_match[] = {
	{
		.compatible = "intel,dummy-bat-fitted-hal",
	},
	{}
};

MODULE_DEVICE_TABLE(of, bat_drv_hal_of_match);


static struct platform_driver bat_hal_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bat_drv_hal_of_match),
	},

	.probe = bat_drv_hal_probe,
	.remove = bat_drv_hal_remove,
};


static int __init bat_drv_hal_init(void)
{
	return platform_driver_register(&bat_hal_driver);
}

static void __exit bat_drv_hal_exit(void)
{
	platform_driver_unregister(&bat_hal_driver);
}

late_initcall(bat_drv_hal_init);
module_exit(bat_drv_hal_exit);

MODULE_DESCRIPTION("AGOLD6x0, dummy Driver HAL, battery always connected");
MODULE_LICENSE("GPL v2");
