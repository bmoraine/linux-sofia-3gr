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

#include "bq24296_charger.h"

struct bq24296_charger bq24296_chrgr_data = {
	.vendor = 32,
	.chg_current_ma = 1536,
	.term_current_ma = 256,
	.pre_chg_current_ma = 256,
	.force_ichg_decrease = 0,
	.vbat_full_mv = 4208,
	.vbat_min_mv = 3504,
	.vbat_max_mv = 4150,
	.max_vbus_ilimit = 3000,
	.min_vbus_ilimit = 100,
	.sys_vmin_mv = 3500,
	.vbus_in_limit_mv = 4360,
	.model_name = "BQ24296",
	.manufacturer = "TI",
	.otg_nb = {
		   .notifier_call = bq24296_otg_notification_handler,
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
			.status = BQ_CHG_STATUS_EXCEPTION,
			.vbus = -1,
			.cc = 550,
			.max_cc = 1450,
			.cv = 4208,
			.iterm = 0,
			.inlmt = 4360,
			.health = POWER_SUPPLY_HEALTH_GOOD,
			.cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE,
			.charger_enabled = false,
			/* initially HZ mode is switched off */
			.charging_enabled = true,
			},
};
