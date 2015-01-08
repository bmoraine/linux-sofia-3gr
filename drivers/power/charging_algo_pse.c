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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <linux/power/battery_id.h>
#include "power_supply.h"
#include "power_supply_charger.h"
#include <linux/hrtimer.h>

#define algo_dbg_printk(fmt, ...) \
	pr_debug(fmt, ##__VA_ARGS__)


/* 98% of CV is considered as voltage to detect Full */
#define FULL_CV_MIN 98

/* Offset to exit from maintenance charging. In maintenance charging
*  if the capacity is less than the (charging_res_cap -
*  MAINT_EXIT_OFFSET) then system can switch to normal charging.
*  This is required so that if the user removes the charger after
*  charging at a lower current than the hardware load, there isn't
*  an unexpected low capacity report after full having been
*  previously indicated. */

#define MAINT_EXIT_OFFSET 1  /* % capacity */

static int get_tempzone(struct ps_pse_mod_prof *bprof,
		int temp)
{
	int i = 0;
	int temp_range_cnt = min_t(u16, bprof->num_temp_bound,
					BATT_TEMP_NR_RNG);

	if (temp_range_cnt < 1)
		return -1;


	if ((temp < bprof->min_temp) ||
		(temp > bprof->temp_range[temp_range_cnt - 1].max_temp)) {
		bprof->in_chrg_allowed_zone = false;
		return -1;
	}


	if ((!bprof->in_chrg_allowed_zone) && (temp < bprof->min_temp_restart))
		return -1;


	if ((!bprof->in_chrg_allowed_zone) && (temp > bprof->max_temp_restart))
		return -1;

	 /* Charging is allowed at the current temperature.
	Find out the current PSE range */
	for (i = 0; i < temp_range_cnt; ++i) {
		if (temp <= bprof->temp_range[i].max_temp)
			break;
	}

	bprof->in_chrg_allowed_zone = true;
	return i;
}

static int validate_bprofile(struct ps_pse_mod_prof *bprof)
{
	int i;
	int temp_range_cnt = min_t(u16, bprof->num_temp_bound,
					BATT_TEMP_NR_RNG);
	bprof->validated = BPROF_VALIDATION_FAIL;

	if (temp_range_cnt < 1)
		return -1;

	if (bprof->min_temp >= bprof->min_temp_restart)
		return -1;

	if (bprof->min_temp_restart >= bprof->max_temp_restart)
		return -1;

	if (bprof->max_temp_restart >=
			bprof->temp_range[temp_range_cnt - 1].max_temp)
		return -1;

	for (i = 1;  i < temp_range_cnt; i++) {
		if (bprof->temp_range[i].max_temp <=
				bprof->temp_range[i-1].max_temp)
			return -1;
	}

	bprof->validated = BPROF_VALIDATION_OK;

	return 0;
}


static inline bool __is_battery_full
	(long volt, long cur, long iterm, unsigned long cv)
{
	algo_dbg_printk("%s:current=%ld pse_mod_bprof->chrg_term_ma =%ld voltage_now=%ld full_cond=%ld\n",
		__func__, cur, iterm, volt * 100, (FULL_CV_MIN * cv));

	return ((cur >= 0) && (cur <= iterm) &&
	((volt * 100)  >= (FULL_CV_MIN * cv)));

}

static inline bool is_battery_full(struct batt_props bat_prop,
		struct ps_pse_mod_prof *pse_mod_bprof, unsigned long cv)
{

	int i;
	/* Software full detection. Check the battery charge current to detect
	*  battery Full. The voltage also verified to avoid false charge
	*  full detection.
	*/
	algo_dbg_printk("%s:current=%ld pse_mod_bprof->chrg_term_ma =%d bat_prop.voltage_now=%ld full_cond=%lu",
		 __func__, bat_prop.current_now, (pse_mod_bprof->chrg_term_ma),
		  bat_prop.voltage_now * 100, (FULL_CV_MIN * cv));

	for (i = (MAX_CUR_VOLT_SAMPLES - 1); i >= 0; --i) {

		if (!(__is_battery_full(bat_prop.voltage_now_cache[i],
				bat_prop.current_now_cache[i],
				pse_mod_bprof->chrg_term_ma, cv)))
			return false;
	}

	return true;
}

static int  pse_get_bat_thresholds(struct ps_batt_chg_prof  bprof,
			struct psy_batt_thresholds *bat_thresh)
{
	int temp_range_cnt;
	struct ps_pse_mod_prof *pse_mod_bprof =
			(struct ps_pse_mod_prof *) bprof.batt_prof;

	if ((bprof.chrg_prof_type != PSE_MOD_CHRG_PROF) || (!pse_mod_bprof))
		return -EINVAL;

	temp_range_cnt = min_t(u16, pse_mod_bprof->num_temp_bound,
					BATT_TEMP_NR_RNG);

	if (temp_range_cnt < 1)
		return -EINVAL;
	bat_thresh->iterm = pse_mod_bprof->chrg_term_ma;
	bat_thresh->temp_min = pse_mod_bprof->min_temp;
	bat_thresh->temp_max = pse_mod_bprof->
			temp_range[temp_range_cnt - 1].max_temp;

	return 0;
}

static enum psy_algo_stat pse_get_next_cc_cv(struct batt_props bat_prop,
	struct ps_batt_chg_prof  bprof, unsigned long *cc, unsigned long *cv)
{
	int tzone;
	struct ps_pse_mod_prof *pse_mod_bprof =
			(struct ps_pse_mod_prof *) bprof.batt_prof;
	enum psy_algo_stat algo_stat = bat_prop.algo_stat;
	int maint_exit_cap;
	struct timespec time_now;

	time_now = ktime_to_timespec(ktime_get_boottime());

	*cc = *cv = 0;

	/* If STATUS is discharging, assume that charger is not connected.
	*  If charger is not connected, no need to take any action.
	*  If charge profile type is not PSE_MOD_CHRG_PROF or the charge profile
	*  is not present, no need to take any action.
	*/

	if ((bprof.chrg_prof_type != PSE_MOD_CHRG_PROF) || (!pse_mod_bprof))
		return PSY_ALGO_STAT_NOT_CHARGE;



	if (pse_mod_bprof->validated == BPROF_NOT_VALIDATED)
		if (validate_bprofile(pse_mod_bprof))
			pr_err("battery profile is invalid !\n");


	if (pse_mod_bprof->validated == BPROF_VALIDATION_FAIL)
		return PSY_ALGO_STAT_NOT_CHARGE;


	if (bat_prop.status == POWER_SUPPLY_STATUS_DISCHARGING ||
			PSY_ALGO_STAT_FULL == bat_prop.algo_stat) {
		/* if not previously charging due to non-temperature
		related reasons, consider the charging temperature
		ranges without hysteresis from last measurement
		temperature. */
		pse_mod_bprof->in_chrg_allowed_zone = true;
	}

	tzone = get_tempzone(pse_mod_bprof, bat_prop.temperature);

	if (tzone < 0 || tzone >= BATT_TEMP_NR_RNG)
		return PSY_ALGO_STAT_NOT_CHARGE;

	/* Change the algo status to not charging, if battery is
	*  not really charging or less than maintenance exit threshold.
	*  This way algorithm can switch to normal
	*  charging if current status is full/maintenace
	*/
	maint_exit_cap = pse_mod_bprof->
			temp_range[tzone].charging_res_cap  -
				MAINT_EXIT_OFFSET;

	algo_dbg_printk("%s entry: time=%ld battery status=%ld algo_status=%d capacity=%d maint_exit_cap=%d\n",
		__func__, time_now.tv_sec, bat_prop.status, algo_stat,
		bat_prop.capacity, maint_exit_cap);

	if ((bat_prop.status == POWER_SUPPLY_STATUS_DISCHARGING) ||
		(bat_prop.status == POWER_SUPPLY_STATUS_NOT_CHARGING) ||
			bat_prop.capacity < maint_exit_cap) {

		algo_stat = PSY_ALGO_STAT_NOT_CHARGE;

	}

	/* read cc and cv based on temperature and algorithm status*/
	if (algo_stat == PSY_ALGO_STAT_FULL ||
			algo_stat == PSY_ALGO_STAT_MAINT) {

		/* if status is full and capacity is lower than maintenance
		lower threshold change status to maintenenance */

		if (algo_stat == PSY_ALGO_STAT_FULL && (bat_prop.capacity <=
			pse_mod_bprof->temp_range[tzone].charging_res_cap))
				algo_stat = PSY_ALGO_STAT_MAINT;

		/* Read maintenance CC and CV */
		if (algo_stat == PSY_ALGO_STAT_MAINT) {
			*cv = pse_mod_bprof->temp_range
					[tzone].maint_chrg_vol_ul;
			*cc = pse_mod_bprof->temp_range
					[tzone].maint_chrg_cur;
			algo_dbg_printk("%s maint: capacity=%d charging_res_cap%d\n",
			__func__, bat_prop.capacity,
			pse_mod_bprof->temp_range[tzone].charging_res_cap);
		}
	} else {
		*cv = pse_mod_bprof->temp_range[tzone].full_chrg_vol;
		*cc = pse_mod_bprof->temp_range[tzone].full_chrg_cur;
		algo_stat = PSY_ALGO_STAT_CHARGE;

		algo_dbg_printk("%s charge: capacity=%d  maint_exit_cap%d\n",
		 __func__, bat_prop.capacity, maint_exit_cap);

	}

	if (is_battery_full(bat_prop, pse_mod_bprof, *cv)) {
		*cc = *cv = 0;
		algo_stat = PSY_ALGO_STAT_FULL;
	}

	return algo_stat;
}

static int __init pse_algo_init(void)
{
	struct charging_algo pse_algo;
	pse_algo.chrg_prof_type = PSE_MOD_CHRG_PROF;
	pse_algo.name = "pse_algo";
	pse_algo.get_next_cc_cv = pse_get_next_cc_cv;
	pse_algo.get_batt_thresholds = pse_get_bat_thresholds;
	power_supply_register_charging_algo(&pse_algo);
	return 0;
}

module_init(pse_algo_init);
