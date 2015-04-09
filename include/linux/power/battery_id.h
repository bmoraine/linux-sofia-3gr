/*
 * Backport from Linux 3.4
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

#ifndef __BATTERY_ID_H__

#define __BATTERY_ID_H__

/* Power supply events */
enum batt_id_xceiv_events {
	BATT_ID_EVENT_BAT_PRESENCE_EVENT,/* battery fitted (with model data)
					payload or removed */
};

enum {
	POWER_SUPPLY_BATTERY_REMOVED = 0,
	POWER_SUPPLY_BATTERY_INSERTED,
};

enum batt_chrg_prof_type {
	PSE_MOD_CHRG_PROF = 0,
};

/* Charging profile structure definition.
Payload for PS_EVENT_BAT_PRESENCE_EVENT */
struct ps_batt_chg_prof {
	enum batt_chrg_prof_type chrg_prof_type;
	void *batt_prof;
};

/* PSE Modified Algo Structure */
/* Parameters defining the charging range */
struct pse_temp_bound {
	/* upper temperature limit for each zone */
	short int max_temp;

	/* charge current and voltage */
	short int full_chrg_vol;
	short int full_chrg_cur;

	/* maintenance thresholds */

	/* Charging resumption capacity threshold. Once battery hits full,
	charging will be resumed when battery capacity is <= this threshold. */
	short int charging_res_cap;
	/* Charge current and voltage in maintenance mode */
	short int maint_chrg_vol_ul;
	short int maint_chrg_cur;
} __packed;

enum bprof_valid_status {
	BPROF_NOT_VALIDATED,
	BPROF_VALIDATION_OK,
	BPROF_VALIDATION_FAIL,
};


#define BATTID_STR_LEN		8
#define BATTMODEL_STR_LEN	8
#define BATT_TEMP_NR_RNG	6
#define BAT_CAP_TO_VBAT_TABLE_SIZE (101)
/* Charging Profile */
struct ps_pse_mod_prof {
	/* battery id */
	char batt_id[BATTID_STR_LEN];
	/* type of battery */
	u16 battery_type;
	u16 capacity;
	u16 voltage_max;
	/* charge termination current */
	u16 chrg_term_ma;
	/* Low battery level voltage */
	u16 low_batt_mv;
	/* upper and lower temperature limits on discharging */
	int disch_tmp_ul;
	int disch_tmp_ll;
	/* number of temperature bounds */
	u16 num_temp_bound;
	struct pse_temp_bound temp_range[BATT_TEMP_NR_RNG];

	u32 cap_to_vbat_ocv[BAT_CAP_TO_VBAT_TABLE_SIZE];

	/* The status of validation of battery profile */
	enum bprof_valid_status validated;

	/* Inside a zone that allows charging or not */
	bool in_chrg_allowed_zone;
	/* low temperature limit to stop charging */
	short int min_temp;
	/* low temperature limit to restart charging, after stopping */
	short int min_temp_restart;
	/* high temperature limit to restart charging, after stopping */
	short int max_temp_restart;
	/* battery model name */
	char model_name[BATTMODEL_STR_LEN];
} __packed;

/*For notification of power supply events */
extern struct atomic_notifier_head    batt_id_notifier;

extern void battery_prop_changed(int battery_conn_stat,
				struct ps_batt_chg_prof *batt_prop);
#ifdef CONFIG_POWER_SUPPLY_BATTID
extern int get_batt_prop(struct ps_batt_chg_prof *batt_prop);
#else
static inline int get_batt_prop(struct ps_batt_chg_prof *batt_prop)
{
	return -ENOMEM;
}
#endif
extern int batt_id_reg_notifier(struct notifier_block *nb);
extern void batt_id_unreg_notifier(struct notifier_block *nb);
#endif
