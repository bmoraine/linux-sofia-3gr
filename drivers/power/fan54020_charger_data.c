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

#define REG_IC_INFO 0x00
#define REG_CHARGE_CTRL1 0x01
#define REG_CHARGE_CTRL2 0x02
#define REG_IBAT 0x03
#define REG_VOREG 0x04
#define REG_IBUS 0x05
#define REG_INT 0x06
#define REG_STATUS 0x07
#define REG_INT_MASK 0x08
#define REG_ST_MASK 0x09
#define REG_TMR_RST 0x0a
#define REG_SAFETY 0x0f
#define REG_MONITOR 0x10
#define REG_STATE 0x1f
#define REG_ADP_CTRL 0x20
#define REG_ADP_CNT 0x21
#define REG_TMR_CTRL 0x22

/* REG_IC_INFO */
#define VENDOR 0x4
#define VENDOR_O 5
#define VENDOR_M 0x7

#define PN 0x1
#define PN_O 3
#define PN_M 0x3

#define REV_O 0
#define REV_M 0x3

/* REG_CHARGE_CTRL1 */
#define HZ_MODE_O 6
#define HZ_MODE_M 0x1

/* REG_CHARGE_CTRL2 */
#define ITERM_DIS_M 0x1
#define ITERM_DIS_O 0
#define LDO_OFF_O 4

#define BOOST_UP_O 5
#define BOOST_UP_M 0x1

#define BOOST_EN_O 6
#define BOOST_EN_M 0x1

/* REG_IBAT */
#define IOCHARGE_MIN_MA 350
#define IOCHARGE_STEP_MA 100
#define IOCHARGE_STEP_START_MA 400
#define IOCHARGE_STEP_START_REGVAL 1
#define IOCHARGE_MAX_MA 1500

#define IOCHARGE_O 4
#define IOCHARGE_M 0xf

#define DEFAULT_CC 350

/* REG_VOREG  */
#define VOREG_MIN_MV 3380
#define VOREG_STEP_MV 20
#define VOREG_MAX_MV 4440
#define VOREG_M 0x3f

#define DEFAULT_CV 3380

/* REG_IBUS */
#define IBUS_MIN_LIMIT_MA 100
#define IBUS_LIMIT_STEP_MA 400
#define IBUS_MAX_LIMIT_MA 900
#define IBUS_M 0x3
#define IBUS_O 0
#define IBUS_NO_LIMIT 3

/* REG_INT and REG_INT_MASK */
#define TSD_FLAG_O 7
#define OVP_FLAG_O 6
#define OVP_RECOV_O 1
#define BOOSTOV_O 5
#define TC_TO_O 4
#define BAT_UV_O 3
#define DBP_TO_O 3
#define TREG_FLAG_O 5
#define OT_RECOV_O 2
#define INT_MASK_ALL 0xff

/* REG_STATUS and REG_ST_MASK */
#define POK_B_O 6
#define ST_MASK_ALL 0xff

/* REG_TMR_RST */
#define TMR_RST_O 7

/* REG_SAFETY  */
#define ISAFE_O 4
#define VSAFE_O 0
#define FAN54020_CUR_LIMIT 0xf /* 1500mA */
#define FAN54020_VOLT_LIMIT 0xf /* 4440mV */

/* REG_STATE */
#define ST_CODE_O 4
#define ST_CODE_M 0xf
#define ST_VBUS_FAULT 0x1a
#define ST_VBUS_FAULT2 0x1d

enum {
	POK_B_VALID = 0,
	POK_B_INVAL,

	BOOSTOV_OCCURED = 1,

	BATUV_OCCURED = 1,

	TSD_OCCURRED = 1,

	OVP_OCCURRED = 1,

	OVP_RECOV_OCCURRED = 1,

	T32_TO_OCCURRED = 1,

	VBUS_FAULT = 1,

	TREG_IS_ON = 1,

	OT_RECOV_OCCURRED = 1,

	VBUS_OFF = 0,
	VBUS_ON,

};


struct charger_attrmap fan54020_charger_attr_map[ATTR_MAX] = {
	[IC_INFO_REG] = {"IC_info_reg", FULL_REG, REG_IC_INFO, 0, 0xFF},
	[CHARGE_CTRL1_REG] = {"ctrl1_reg", FULL_REG, REG_CHARGE_CTRL1, 0, 0xFF},
	[CHARGE_CTRL2_REG] = {"ctrl2_reg", FULL_REG, REG_CHARGE_CTRL2, 0, 0xFF},
	[IBAT_REG] = {"ibat_reg", FULL_REG, REG_IBAT, 0, 0xFF},
	[VOREG_REG] = {"voreg_reg", FULL_REG, REG_VOREG, 0, 0xFF},
	[IBUS_REG] = {"ibus_reg", FULL_REG, REG_IBUS, 0, 0xFF},
	[INT_REG] = {"int_reg", FULL_REG, REG_INT, 0, 0xFF},
	[STATUS_REG] = {"status_reg", FULL_REG, REG_STATUS, 0, 0xFF},
	[INT_MASK_REG] = {"int_mask_reg", FULL_REG, REG_INT_MASK, 0, 0xFF},
	[ST_MASK_REG] = {"st_mask_reg", FULL_REG, REG_ST_MASK, 0, 0xFF},
	[TMR_RST_REG] = {"tmr_rst_reg", FULL_REG, REG_TMR_RST, 0, 0xFF},
	[SAFETY_REG] = {"safety_reg", FULL_REG, REG_SAFETY, 0, 0xFF},
	[MONITOR_REG] = {"monitor_reg", FULL_REG, REG_MONITOR, 0, 0xFF},
	[STATE_REG] = {"state_reg", FULL_REG, REG_STATE, 0, 0xFF},
	[ADP_CTRL_REG] = {"adp_ctrl_reg", FULL_REG, REG_ADP_CTRL, 0, 0xFF},
	[ADP_CNT_REG] = {"adp_cnt_reg", FULL_REG, REG_ADP_CNT, 0, 0xFF},
	[TMR_CTRL_REG] = {"tmr_ctrl_reg", FULL_REG, REG_TMR_CTRL, 0, 0xFF},
	[VENDOR_INFO] = {"Vendor", BITS, REG_IC_INFO, VENDOR_O, VENDOR_M},
	[PN_INFO] = {"PN", BITS, REG_IC_INFO, PN_O, PN_M},
	[REV_INFO] = {"Rev", BITS, REG_IC_INFO, REV_O, REV_M},
	[HZ_MODE] = {"hz_mode", BITS, REG_CHARGE_CTRL1, HZ_MODE_O, HZ_MODE_M},
	[BOOST_EN] = {
		"boost_mode", BITS, REG_CHARGE_CTRL2, BOOST_EN_O, BOOST_EN_M},
	[BOOST_UP] = {
		"boost_up", BITS, REG_CHARGE_CTRL2, BOOST_UP_O, BOOST_UP_M},
	[LDO] = {"ldo", BITS, REG_CHARGE_CTRL2, LDO_OFF_O, 0x1},
	[IOCHARGE] = {"Iocharge", BITS, REG_IBAT, IOCHARGE_O, IOCHARGE_M},
	[ITERM] = {"Iterm", BITS, REG_IBAT, 0, 0xF},
	[VOREG] = {"Voreg", BITS, REG_VOREG, 0, VOREG_M},
	[IBUS] = {"Ibus", BITS, REG_IBUS, IBUS_O, IBUS_M},
	[TMR_RST] = {"tmr_rst", BITS, REG_TMR_RST, TMR_RST_O, 0x1},
	[T32_TO] = {"t32_to", BITS, REG_INT, 4, 0x1},
};


static int fan54020_get_charger_state(struct fan54x_charger *chrgr)
{
	u8 interrupt_reg, vbus_stat_reg, state_reg;
	int ret, state;

	ret = fan54x_attr_read(chrgr->client, INT_REG, &interrupt_reg);
	if (ret != 0)
		goto fail;

	ret = fan54x_attr_read(chrgr->client, STATUS_REG, &vbus_stat_reg);
	if (ret != 0)
		goto fail;

	ret = fan54x_attr_read(chrgr->client, STATE_REG, &state_reg);
	if (ret != 0)
		goto fail;

	/* Checking for OVP_FLAG or OVP_RECOV occurrance */

	/* Common flags between charger mode and boost mode */
	chrgr->state.tsd_flag = (interrupt_reg & (1<<TSD_FLAG_O)) ?
							TSD_OCCURRED : 0;

	chrgr->state.ovp_flag = (interrupt_reg & (1<<OVP_FLAG_O)) ?
							OVP_OCCURRED : 0;

	chrgr->state.t32s_timer_expired = (interrupt_reg & (1<<TC_TO_O)) ?
							T32_TO_OCCURRED : 0;

	/* Handle interrupts specific to boost mode*/
	if (chrgr->state.boost_enabled) {
		chrgr->state.boost_ov = (interrupt_reg & (1<<BOOSTOV_O)) ?
							BOOSTOV_OCCURED : 0;
		chrgr->state.bat_uv = (interrupt_reg & (1<<BAT_UV_O)) ?
							BATUV_OCCURED : 0;
	}

	chrgr->state.ovp_recov = (interrupt_reg & (1<<OVP_RECOV_O)) ?
							OVP_RECOV_OCCURRED : 0;

	chrgr->state.treg_flag = (interrupt_reg & (1<<TREG_FLAG_O)) ?
							TREG_IS_ON : 0;

	chrgr->state.ot_recov_flag = (interrupt_reg & (1<<OT_RECOV_O)) ?
							OT_RECOV_OCCURRED : 0;

	/* Checking for POK_B status change */
	chrgr->state.pok_b = (vbus_stat_reg & (1<<POK_B_O)) ?
						POK_B_INVAL : POK_B_VALID;
	chrgr->state.vbus = (chrgr->state.pok_b == POK_B_VALID) ?
							VBUS_ON : VBUS_OFF;

	state = ((state_reg >> ST_CODE_O) & ST_CODE_M);
	chrgr->state.vbus_fault =
			(state >= ST_VBUS_FAULT && state < ST_VBUS_FAULT2) ?
								VBUS_FAULT : 0;
fail:
	return ret;
}

static int fan54020_configure_chip(struct fan54x_charger *chrgr,
							bool enable_charging)
{
	int ret;

	/*
	 * Set LDO_OFF bit to '0'
	 * 3.3V LDO is ON and biased from VBAT when:
	 * VBUS < VBAT && DBP pin is HIGH
	 * Setting valid for CHIP id 0x88
	 */
	ret = fan54x_attr_write(chrgr->client, LDO, 0);
	if (ret)
		return ret;

	ret = fan54x_attr_write(chrgr->client, HZ_MODE,
					(enable_charging ? 0 : 1));

	if (ret)
		return ret;

	ret = fan54x_attr_write(chrgr->client, INT_MASK_REG, 0x1);
	if (ret)
		return ret;

	ret = fan54x_attr_write(chrgr->client, ST_MASK_REG, 0xBF);
	if (ret)
		return ret;

	return 0;
}

static int fan54020_enable_charging(struct fan54x_charger *chrgr, bool enable)
{
	return fan54x_attr_write(chrgr->client, HZ_MODE, ((enable) ? 0 : 1));
}

static int fan54020_get_clr_wdt_expiry_flag(struct fan54x_charger *chrgr)
{
	u8 t32_to;
	int ret, wtd_expired;

	/* If the WDT interrupt was previously set,
	it will now be cleared upon reading */
	ret = fan54x_attr_read(chrgr->client, T32_TO, &t32_to);
	if (ret)
		return ret;

	wtd_expired = t32_to ? T32_TO_OCCURRED : 0;

	return wtd_expired;
}

struct fan54x_charger fan54020_chrgr_data = {
	.vendor = 4,
	.pn = 1,
	.rev = 1,
	.max_voreg = 4440,
	.min_voreg = 3380,
	.max_iocharge = 1500,
	.min_iocharge = 300,
	.max_ibus_limit = 900,
	.min_ibus_limit = 100,
	.default_cc = 350,
	.default_cv = 3380,
	.model_name = "FAN54020",
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
		.cc = 350,
		.max_cc = 1500,
		.cv = 3380,
		.iterm = 0,
		.health = POWER_SUPPLY_HEALTH_UNKNOWN,
		.cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE,
		.charger_enabled = false,
		.charging_enabled = true, /* initially HZ mode is switched off*/
		.pok_b = 0,
		.ovp_flag = 0,
		.ovp_recov = 0,
		.t32s_timer_expired = 0,
		.vbus_fault = 0,
		.treg_flag = 0,
		.ot_recov_flag = 0,
		.tsd_flag = 0,
	},

	.attrmap = fan54020_charger_attr_map,
	.configure_chip = fan54020_configure_chip,
	.enable_charging = fan54020_enable_charging,
	.get_charger_state = fan54020_get_charger_state,
	.get_clr_wdt_expiry_flag = fan54020_get_clr_wdt_expiry_flag,
};

