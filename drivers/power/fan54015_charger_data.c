/*
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
 */

#include "fan54x_charger.h"
#include <linux/delay.h>

#define REG_CHARGE_CTRL0	0x0
#define REG_CHARGE_CTRL1	0x1
#define REG_VOREG		0x2
#define REG_IC_INFO		0x3
#define REG_IBAT		0x4
#define REG_SP_CHARGER		0x5
#define REG_SAFETY		0x6
#define REG_MONITOR		0x10

/* REG_IC_INFO */
#define VENDOR 0x4
#define VENDOR_O 5
#define VENDOR_M 0x7

#define PN 0x5
#define PN_O 2
#define PN_M 0x7

#define REV_O 0
#define REV_M 0x3

/* REG_CHARGE_CTRL0 */
#define TMR_RST_O 7
#define TMR_RST_M 0x1

#define STAT_O	4
#define STAT_M	0x3

#define BOOST_UP_O 3
#define BOOST_UP_M 0x1

#define FAULT_O	0
#define FAULT_M	0x7

/* REG_CHARGE_CTRL1 */
#define CHG_EN_O 2
#define CHG_EN_M 0x1

#define HZ_MODE_O 1
#define HZ_MODE_M 0x1

#define IBUS_O 6
#define IBUS_M 0x3

#define VLOWV_O 4
#define VLOWV_M 0x3

#define BOOST_EN_O 0
#define BOOST_EN_M 0x1

/* REG_IBAT */
#define IOCHARGE_O 4
#define IOCHARGE_M 0x7

#define ITERM_O 0
#define ITERM_M 0x7

/* REG_OREG */
#define VOREG_O 2
#define VOREG_M 0x3F

#define OTG_EN_O 0
#define OTG_EN_M 1

/* REG_SP_CHARGER */
#define IO_LEVEL_O 5
#define IO_LEVEL_M 0x1

/* REG_MONITOR */
#define VBUS_VALID_O 1

enum {

	TSD_OCCURRED = 1,

	OVP_OCCURRED = 1,

	BOOSTOV_OCCURRED = 1,

	BATUV_OCCURRED = 1,

	BAT_OVP_OCCURRED = 1,

	T32_TO_OCCURRED = 1,

	VBUS_FAULT = 1,

	POOR_INPUT_SOURCE_OCCURRED = 1,

	SLEEP_MODE_OCCURRED = 1,

	NO_BAT_OCCURRED = 1,

	VBUS_OFF = 0,
	VBUS_ON,

};

enum FAN54015_FAULT_STAT {
	NO_FAULT,
	VBUS_OVP,
	SLEEP_MODE,
	BOOST_OV = 2,
	POOR_INPUT_SOURCE,
	BAT_UV = 3,
	BATTERY_OVP,
	THERMAL_SHUTDOWN,
	TIMER_FAULT,
	NO_BATTERY,
};

struct charger_attrmap fan54015_charger_attr_map[ATTR_MAX] = {
	[IC_INFO_REG] = {"IC_info_reg", FULL_REG, REG_IC_INFO, 0, 0xFF},
	[CHARGE_CTRL0_REG] = {"ctrl0_reg", FULL_REG, REG_CHARGE_CTRL0, 0, 0xFF},
	[CHARGE_CTRL1_REG] = {"ctrl1_reg", FULL_REG, REG_CHARGE_CTRL1, 0, 0xFF},
	[IBAT_REG] = {"ibat_reg", FULL_REG, REG_IBAT, 0, 0xFF},
	[VOREG_REG] = {"voreg_reg", FULL_REG, REG_VOREG, 0, 0xFF},
	[SAFETY_REG] = {"safety_reg", FULL_REG, REG_SAFETY, 0, 0xFF},
	[MONITOR_REG] = {"monitor_reg", FULL_REG, REG_MONITOR, 0, 0xFF},
	[SP_CHARGER_REG] = {"sp_chg_reg", FULL_REG, REG_SP_CHARGER, 0, 0xFF},
	[VENDOR_INFO] = {"Vendor", BITS, REG_IC_INFO, VENDOR_O, VENDOR_M},
	[PN_INFO] = {"PN", BITS, REG_IC_INFO, PN_O, PN_M},
	[REV_INFO] = {"Rev", BITS, REG_IC_INFO, REV_O, REV_M},
	[HZ_MODE] = {"hz_mode", BITS, REG_CHARGE_CTRL1, HZ_MODE_O, HZ_MODE_M},
	[BOOST_EN] = {
		"boost_mode", BITS, REG_CHARGE_CTRL1, BOOST_EN_O, BOOST_EN_M},
	[BOOST_UP] = {
		"boost_up", BITS, REG_CHARGE_CTRL0, BOOST_UP_O, BOOST_UP_M},
	[OTG_EN] = {"otg_en", BITS, REG_VOREG, OTG_EN_O, OTG_EN_M},
	[IOCHARGE] = {"Iocharge", BITS, REG_IBAT, IOCHARGE_O, IOCHARGE_M},
	[ITERM] = {"Iterm", BITS, REG_IBAT, ITERM_O, ITERM_M},
	[VOREG] = {"Voreg", BITS, REG_VOREG, VOREG_O, VOREG_M},
	[IBUS] = {"Ibus", BITS, REG_CHARGE_CTRL1, IBUS_O, IBUS_M},
	[VLOWV] = {"Vlowv", BITS, REG_CHARGE_CTRL1, VLOWV_O, VLOWV_M},
	[TMR_RST] = {"tmr_rst", BITS, REG_CHARGE_CTRL0, TMR_RST_O, TMR_RST_M},
	[CHG_EN] = {"chg_en", BITS, REG_CHARGE_CTRL1, CHG_EN_O, CHG_EN_M},
	[IO_LEVEL] = {"io_level", BITS, REG_SP_CHARGER, IO_LEVEL_O, IO_LEVEL_M},
};

static int fan54015_enable_charging(struct fan54x_charger *chrgr, bool enable)
{
	int ret;

		/* enabling and disabling charger based on HZ bit,
		 * so keeping the CE bit to default value always.
		 */

		ret = fan54x_attr_write(chrgr->client, CHG_EN, 0);
		if (ret)
			return ret;

	if (enable) {
		/* In case when CE or HZ bit is set,
		 * we have to manually clear it to enable charger again.
		 * No need to set the bit when disabling charger.
		 */

		ret = fan54x_attr_write(chrgr->client, HZ_MODE, 0);
		if (ret)
			return ret;

	} else if (!enable) {
		/* set HZ bit to disable charging*/
		ret = fan54x_attr_write(chrgr->client, HZ_MODE, 1);
		if (ret)
			return ret;
	}

	return 0;
}

static int fan54015_configure_chip(struct fan54x_charger *chrgr,
							bool enable_charging)
{
	return 0;
}

static int fan54015_get_charger_state(struct fan54x_charger *chrgr)
{
	u8 charge_ctrl0_reg, monitor_reg;
	int ret;
	int timeout = 3;

	ret = fan54x_attr_read(chrgr->client,
			CHARGE_CTRL0_REG, &charge_ctrl0_reg);
	if (ret != 0)
		goto fail;

	ret = fan54x_attr_read(chrgr->client, MONITOR_REG, &monitor_reg);
	if (ret != 0)
		goto fail;

	chrgr->state.vbus = (monitor_reg & (1 << VBUS_VALID_O)) ?
						VBUS_ON : VBUS_OFF;

	chrgr->state.health = (chrgr->state.vbus == VBUS_ON) ?
			POWER_SUPPLY_HEALTH_GOOD : POWER_SUPPLY_HEALTH_UNKNOWN;

	/* Clear state flags */
	chrgr->state.vbus_ovp = 0;
	chrgr->state.sleep_mode = 0;
	chrgr->state.poor_input_source = 0;
	chrgr->state.bat_ovp = 0;
	chrgr->state.tsd_flag = 0;
	chrgr->state.t32s_timer_expired = 0;
	chrgr->state.no_bat = 0;
	chrgr->state.boost_ov = 0;
	chrgr->state.bat_uv = 0;

	switch (charge_ctrl0_reg & (FAULT_M << FAULT_O)) {
	case NO_FAULT:
		break;
	case VBUS_OVP:
		chrgr->state.vbus_ovp = OVP_OCCURRED;
		break;
	case SLEEP_MODE:
		if (chrgr->state.boost_enabled)
			chrgr->state.boost_ov = BOOSTOV_OCCURRED;
		else
			chrgr->state.sleep_mode = SLEEP_MODE_OCCURRED;
		break;
	case POOR_INPUT_SOURCE:
		if (chrgr->state.boost_enabled)
			chrgr->state.bat_uv = BATUV_OCCURRED;
		else
			chrgr->state.poor_input_source =
				POOR_INPUT_SOURCE_OCCURRED;
		break;
	case BATTERY_OVP:
		chrgr->state.bat_ovp = BAT_OVP_OCCURRED;
		break;
	case THERMAL_SHUTDOWN:
		chrgr->state.tsd_flag = TSD_OCCURRED;
		break;
	case TIMER_FAULT:
		chrgr->state.t32s_timer_expired = T32_TO_OCCURRED;
		break;
	case NO_BATTERY:
		chrgr->state.no_bat = NO_BAT_OCCURRED;
		break;
	default:
		break;
	}

	/* Check three times that if the over current happens */
	if (chrgr->state.boost_enabled) {
		while (timeout--) {
			msleep(5);
			ret = fan54x_attr_read(chrgr->client,
				CHARGE_CTRL0_REG, &charge_ctrl0_reg);
			if (ret)
				goto fail;
			if ((charge_ctrl0_reg & (FAULT_M << FAULT_O)) == BOOST_OV)
				chrgr->state.boost_ov = BOOSTOV_OCCURRED;
			else
				chrgr->state.boost_ov = 0;

			pr_debug("timeout = %d, chrgr->state.boost_ov = %d\n",
				timeout, chrgr->state.boost_ov);
		}
	}

fail:
	return ret;
}

static int fan54015_iocharge_list[8] = {
	550, 650, 750, 850, 1050, 1150, 1350, 1450};

static int fan54015_calc_iocharge_regval(struct fan54x_charger *chrgr,
					int current_to_set_ma)
{
	int i;

	for (i = 0; i < 7; i++) {
		if (current_to_set_ma >= fan54015_iocharge_list[i] &&
			current_to_set_ma < fan54015_iocharge_list[i+1])
			break;
	}

	return i;
}

int disable_charger_fan54015(bool disable)
{
	u8 reg;
	if (fan54015_chrgr_data.client == NULL)
		return 0;
	down(&fan54015_chrgr_data.prop_lock);
	fan54x_attr_write(fan54015_chrgr_data.client, CHG_EN, disable);
	up(&fan54015_chrgr_data.prop_lock);
}

static int fan54015_get_iocharge_val(int regval)
{
	return fan54015_iocharge_list[regval];
}

struct fan54x_charger fan54015_chrgr_data = {
	.vendor = 4,
	.pn = 5,
	.rev = 0,
	.max_voreg = 4440,
	.min_voreg = 3500,
	.max_iocharge = 1450,
	.min_iocharge = 550,
	.max_ibus_limit = 900,
	.min_ibus_limit = 100,
	.default_cc = 550,
	.default_cv = 3540,
	.model_name = "FAN54015",
	.manufacturer = "FAIRCHILD",

	.otg_nb = {
		.notifier_call = fan54x_otg_notification_handler,
	},

	.chgint_bh = {
		.in_suspend = false,
		.pending_evt = false,
	},
	.boost_op_bh = {
		.in_suspend = false,
		.pending_evt = false,
	},
	.fake_vbus = -1,

	.state = {
		.status = FAN54x_STATUS_UNKNOWN,
		.vbus = -1,
		.cc = 550,
		.max_cc = 1450,
		.cv = 3500,
		.iterm = 0,
		.health = POWER_SUPPLY_HEALTH_UNKNOWN,
		.cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE,
		.charger_enabled = false,
		.charging_enabled = true, /* initially HZ mode is switched off*/
		.ovp_flag = 0,
		.t32s_timer_expired = 0,
		.tsd_flag = 0,
		.vbus_ovp = 0,
		.sleep_mode = 0,
		.poor_input_source = 0,
		.bat_ovp = 0,
		.no_bat = 0,
	},

	.attrmap = fan54015_charger_attr_map,
	.configure_chip = fan54015_configure_chip,
	.enable_charging = fan54015_enable_charging,
	.get_charger_state = fan54015_get_charger_state,
	.calc_iocharge_regval = fan54015_calc_iocharge_regval,
	.get_iocharge_val = fan54015_get_iocharge_val,
};

