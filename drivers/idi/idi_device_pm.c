/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/device_state_pm.h>
#include <linux/idi/idi_interface.h>

/* PM Side wrapper layer for IDI */


/* States: D0, D0I0, D0I1, D0I2, D0I3, D3, NA - 0 to 6 */
#define	NUM_PM_IDI_STATES	7
#define	NUM_PM_IDI_PERS		22


/*  IDI Bus driver:
	TO register:
	platform_device_pm_set_class(struct platform_device *pdev, PER_IDI);
	To set the IDI Bus state:
	platform_device_pm_set_state(struct platform_device *pdev,
					IDI_D0/IDI_D3);

    IDI client device (PM wrapper on IDI side would be calling):
	To register the IDI peripheral:
	int idi_device_pm_set_class(struct idi_peripheral_device *peripheral);
	To set the IDI peripheral state:
	int idi_device_pm_set_state(struct idi_peripheral_device *peripheral,
					int state );
 */

/*
--------------------------------------------------------------------------
|sl. |  Peripheral      | D0      |     D0I0            | D0I1           |
--------------------------------------------------------------------------
|1   |  IDI_FMR         | enable  |     enable_intclk   | -              |
|2   |  IDI_AFE         | headset |     no_headset      | slpret_headset |
|3   |  IDI_BT          | enable  |     -               | -              |
|4   |  IDI_I2C         | enable  |     -               | -              |
|5   |  IDI_WLAN        | enable  |     -               | -              |
|6   |  IDI_GNSS        | enable  |     enable_freq_1   | -              |
|7   |  IDI_RTC         | -       |     -               | -              |
|8   |  IDI_MEAS        | enable  |     -               | -              |
|9   |  IDI_REG_FW      | -       |     -               | -              |
|10  |  IDI_VIBT        | -       |     -               | -              |
|11  |  IDI_BKL         | -       |     -               | -              |
|12  |  IDI_ACC         | -       |     -               | -              |
|13  |  IDI_CHG         | -       |     -               | -              |
|14  |  IDI_BAT         | -       |     -               | -              |
|15  |  IDI_CCD         | -       |     -               | -              |
|16  |  IDI_BNT         | -       |     -               | -              |
|17  |  IDI_DBG         | -       |     -               | -              |
|18  |  IDI_BT_STREAM   | enable  |     -               | -              |
|19  |  IDI_AFE_STREAM  | headset |     no_headset      | slpret_headset |
|20  |  IDI_FMR_STREAM  | enable  |     enable_intclk   | -              |
--------------------------------------------------------------------------

------------------------------------------------------------------
|sl. |  Peripheral      | D0I2                 | D0I3  | D3      |
------------------------------------------------------------------
|1   |  IDI_FMR         | -                    | -     | disable |
|2   |  IDI_AFE         | slpret_no_headset    | -     | disable |
|3   |  IDI_BT          | enable_psv           | idle  | disable |
|4   |  IDI_I2C         | -                    | -     | disable |
|5   |  IDI_WLAN        | -                    | idle  | disable |
|6   |  IDI_GNSS        | -                    | idle  | disable |
|7   |  IDI_RTC         | -                    | -     | -       |
|8   |  IDI_MEAS        | -                    | -     | disable |
|9   |  IDI_REG_FW      | -                    | -     | -       |
|10  |  IDI_VIBT        | -                    | -     | -       |
|11  |  IDI_BKL         | -                    | -     | -       |
|12  |  IDI_ACC         | -                    | -     | -       |
|13  |  IDI_CHG         | -                    | -     | -       |
|14  |  IDI_BAT         | -                    | -     | -       |
|15  |  IDI_CCD         | -                    | -     | -       |
|16  |  IDI_BNT         | -                    | -     | -       |
|17  |  IDI_DBG         | -                    | -     | -       |
|18  |  IDI_BT_STREAM   | -                    | -     | disable |
|19  |  IDI_AFE_STREAM  | slpret_no_headset    | -     | disable |
|20  |  IDI_FMR_STREAM  | -                    | -     | disable |
------------------------------------------------------------------
*/

/* Match with idi_peripheral_type enum in idi_interface.h */
static char *pm_dev_name[] = {
	"",		/* IDI_INTERNAL */
	"fmr",		/* IDI_FMR */
	"afe",		/* IDI_AFE */
	"btif",		/* IDI_BT */
	"i2cabb",	/* IDI_I2C */
	"wlan",		/* IDI_WLAN */
	"gnss",		/* IDI_GNSS */
	"rtc",		/* IDI_RTC */
	"meas",		/* IDI_MEAS */
	"reg_fw",	/* IDI_REG_FW */
	"vibt",		/* IDI_VIBT */
	"bkl",		/* IDI_BKL */
	"acc",		/* IDI_ACC */
	"chg",		/* IDI_CHG */
	"bat",		/* IDI_BAT */
	"ccd",		/* IDI_CCD */
	"bnt",		/* IDI_BNT */
	"dbg",		/* IDI_DBG */
	"btaud",	/* IDI_BT_STREAM */
	"afe",		/* IDI_AFE_STREAM TODO: Currently same as afe */
	"fmr",		/* IDI_FMR_STREAM TODO: Currently same as fmr */
	"",		/* IDI_MAX_PERIPHERAL */
};


/* PM Framework device states.
   Match with idi_devices_power_state enum in idi_interface.h
   States: D0, D0I0, D0I1, D0I2, D0I3, D3, NA

   NOTE: AFE_STREAM & FMR_STREAM is implemented same as afe & fmr.
	 This stream ids for afe & fmr are just for idi's info.
*/
static char *pm_dev_state[][NUM_PM_IDI_STATES] = {
	{""},					/* IDI_INTERNAL */
	{"enable", "enable_intclk", "", "", "", "disable", ""},	/* IDI_FMR */
	{"headset", "no_headset", "slpret_headset", "slpret_no_headset", "",
		"disable", ""}, /* IDI_AFE */
	{"enable", "", "", "enable_psv", "idle", "disable",
		""},	/* IDI_BT */
	{"enable", "", "", "", "", "disable", ""},	/* IDI_I2C */
#ifdef CONFIG_AGOLD610
	{""},	/* IDI_WLAN */
	{""},	/* IDI_GNSS */
#else
	{"enable", "", "", "", "idle", "disable", ""},	/* IDI_WLAN */
	{"enable_def_dclk_832_pclk_1248", "enable_dclk_96_pclk_1248", "", "",
					"idle", "disable", ""}, /* IDI_GNSS */
#endif
	{""},					/* IDI_RTC */
	{"enable", "", "", "", "", "disable", ""},	/* IDI_MEAS */
	{""},	/* IDI_REG_FW */
	{""},	/* IDI_VIBT */
	{""},	/* IDI_BKL */
	{""},	/* IDI_ACC */
	{""},	/* IDI_CHG */
	{""},	/* IDI_BAT */
	{""},	/* IDI_CCD */
	{""},	/* IDI_BNT */
	{""},	/* IDI_DBG */
	{"enable", "", "", "", "", "disable", ""},	/* IDI_BT_STREAM */
	{"headset", "no_headset", "slpret_headset", "slpret_no_headset", "",
		"disable", ""}, /* IDI_AFE_STREAM */
	{"enable", "enable_intclk", "", "", "", "disable",
		""},	/* IDI_FMR_STREAM */
	{""},					/* IDI_MAX_PERIPHERAL */
};


static struct device_state_pm_state
	*pm_dev_state_ptrs[NUM_PM_IDI_PERS][NUM_PM_IDI_STATES];

static int update_state_handlers(struct device *dev, int index)
{
	int i;
	dev_dbg(dev, "IDI device: %s\n", dev->pm_data.pm_user->name);
	for (i = 0; i < NUM_PM_IDI_STATES; i++) {
		if ((!pm_dev_state[index][i]) ||
				(!strcmp(pm_dev_state[index][i], "")))
			pm_dev_state_ptrs[index][i] = NULL;
		else {
			pm_dev_state_ptrs[index][i] =
			device_state_pm_get_state_handler(dev,
						pm_dev_state[index][i]);
			if (pm_dev_state_ptrs[index][i] == NULL) {
				dev_err(dev,
					"IDI PM: Error in state handler\n");
				return -1;
			}
		}
		/* -For debugging */
		if (pm_dev_state_ptrs[index][i] != NULL)
			dev_dbg(dev, "%s ", pm_dev_state_ptrs[index][i]->name);
		else
			dev_dbg(dev, "NULL pointer for pm dev state ");
	}
	return 0;
}

/* Registration */
int idi_device_pm_set_class(struct idi_peripheral_device *peripheral)
{
	int ret;
	if (!peripheral)
		return -1;
	/* device_state_pm_set_class(struct device *dev, int dev_id) */
	ret = device_state_pm_set_class(&(peripheral->device),
					pm_dev_name[peripheral->p_type]);
	if (ret) /* Return here for error */
		return ret;

	/* Get all the state handlers for the device */
	ret = update_state_handlers(&(peripheral->device), peripheral->p_type);
	return ret;
}
EXPORT_SYMBOL(idi_device_pm_set_class);

/* State Change call */
int idi_device_pm_set_state(struct idi_peripheral_device *peripheral,
				int idi_state)
{
	/* char *state; */
	struct device_state_pm_state *state;

	if (!peripheral)
		return -1;

	state = pm_dev_state_ptrs[peripheral->p_type][idi_state];
	return device_state_pm_set_state(&(peripheral->device), state);
}

/* ======================================================================== */
/* Set the class for idi client device */
int idi_peripheral_device_pm_set_class(struct idi_peripheral_device *idipdev,
						const char *class_name)
{
	return device_state_pm_set_class(&(idipdev->device), class_name);
}

/* Get state handler for idi client device */
struct device_state_pm_state *idi_peripheral_device_pm_get_state_handler(
					struct idi_peripheral_device *idipdev,
					const char *state_name)
{
	return (struct device_state_pm_state *)
				device_state_pm_get_state_handler(
				&(idipdev->device), state_name);
}
EXPORT_SYMBOL_GPL(idi_peripheral_device_pm_get_state_handler);

/* Set the device state of the given idi client device */
int idi_peripheral_device_pm_set_state(struct idi_peripheral_device *idipdev,
					void *state)
{
	return device_state_pm_set_state(&(idipdev->device),
				(struct device_state_pm_state *)state);
}

/* Set the device state of the given idi client device */
int idi_peripheral_device_pm_set_state_by_name(
					struct idi_peripheral_device *idipdev,
					const char *state_name)
{
	return device_state_pm_set_state_by_name(
					&(idipdev->device), state_name);
}

/* Can be used to select the required state when Runtime-pm is enabled */
int idi_peripheral_device_pm_set_active_state(
					struct idi_peripheral_device *idipdev,
					void *state)
{
	return device_state_pm_set_active_state(&(idipdev->device),
				(struct device_state_pm_state *)state);
}

/* Can be called from pm_runtime_get()/put() to enable/disable power */
int idi_peripheral_device_pm_activate(struct idi_peripheral_device *idipdev,
					bool activate)
{
	return device_state_pm_activate(&(idipdev->device), activate);
}
/* ======================================================================== */
