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

#ifndef _BPROF_DTS_PARSER_H
#define _BPROF_DTS_PARSER_H

#include <linux/power/battery_id.h>
#include <linux/power_supply.h>

enum battery_type_id {
	BAT_TYPE_LC,
	BAT_TYPE_SMART,
	BAT_TYPE_MIPI,
};

struct battery_type {
	unsigned int batid_ohms;
	enum battery_type_id type;
	struct ps_batt_chg_prof profile;
};

int bprofile_parse_dt(struct battery_type **batteries, size_t *batteries_len,
			struct ps_pse_mod_prof **bprof, size_t *bprof_len,
						struct device_node *np);

#endif /*_BPROF_DTS_PARSER_H*/
