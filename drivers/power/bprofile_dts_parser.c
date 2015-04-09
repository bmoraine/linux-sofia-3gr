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

#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/power/battery_id.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "bprofile_dts_parser.h"

#ifdef CONFIG_OF

static int dt_add_bprofile(struct device_node *np,
	const char *prof_name, struct ps_pse_mod_prof *bprof, int len)
{
#define BPROF_HEADER_SIZE 10
#define TRANGE_SIZE 6

	int i, index;
	u32 nranges;
	u32 temp_buf[BPROF_HEADER_SIZE];
	unsigned char propname[64] = {0};
	struct ps_pse_mod_prof *bprofile_ptr;
	struct pse_temp_bound *trange_ptr;
	size_t strlen = strnlen(prof_name, BATTID_STR_LEN);
	struct property *pocv;
	int plen;
	const char *mname;

	/* Check if the profile was already added before */
	for (i = 0; i < len; ++i) {

		if (bprof[i].batt_id[0] == 0)
			break;

		if (0 == strncmp(bprof[i].batt_id, prof_name, strlen))
			return i;
	}

	if (i == len)
		return -ENOMEM;

	index = i;
	bprofile_ptr = &bprof[index];

	strncpy(bprofile_ptr->batt_id, prof_name, strlen);

	snprintf(propname, ARRAY_SIZE(propname),
		"prof-%s-ntemp_ranges", bprofile_ptr->batt_id);

	if (of_property_read_u32(np, propname, &nranges)) {
		pr_err("dt: parsing '%s' failed\n", propname);
		return -EINVAL;
	}

	bprofile_ptr->num_temp_bound = nranges;

	for (i = 0; i < nranges; ++i) {
		int idx = 0;

		snprintf(propname, ARRAY_SIZE(propname),
			"prof-%s-temp_range%d",
				bprofile_ptr->batt_id, i);

		if (of_property_read_u32_array(np, propname,
						temp_buf, TRANGE_SIZE)) {
			pr_err("dt: parsing '%s' failed\n", propname);
			return -EINVAL;
		}

		trange_ptr = &bprofile_ptr->temp_range[i];
		trange_ptr->max_temp = temp_buf[idx++];
		trange_ptr->full_chrg_vol = temp_buf[idx++];
		trange_ptr->full_chrg_cur = temp_buf[idx++];
		trange_ptr->charging_res_cap = temp_buf[idx++];
		trange_ptr->maint_chrg_vol_ul = temp_buf[idx++];
		trange_ptr->maint_chrg_cur = temp_buf[idx++];
	}

	snprintf(propname, ARRAY_SIZE(propname), "prof-%s",
						bprofile_ptr->batt_id);

	if (of_property_read_u32_array(np, propname, temp_buf,
						BPROF_HEADER_SIZE)) {
		pr_err("dt: parsing '%s' failed\n", propname);
		return -EINVAL;
	}

	i = 0;
	bprofile_ptr->battery_type = temp_buf[i++];
	bprofile_ptr->capacity = temp_buf[i++];
	bprofile_ptr->voltage_max = temp_buf[i++];
	bprofile_ptr->chrg_term_ma = temp_buf[i++];
	bprofile_ptr->low_batt_mv = temp_buf[i++];
	bprofile_ptr->disch_tmp_ul = temp_buf[i++];
	bprofile_ptr->disch_tmp_ll = temp_buf[i++];
	bprofile_ptr->min_temp = temp_buf[i++];
	bprofile_ptr->min_temp_restart = temp_buf[i++];
	bprofile_ptr->max_temp_restart = temp_buf[i++];


	snprintf(propname, ARRAY_SIZE(propname), "prof-%s-cap_to_vbat_ocv",
						bprofile_ptr->batt_id);

	pocv = of_find_property(np, propname, &plen);
	if (NULL == pocv || (plen/sizeof(u32)) != BAT_CAP_TO_VBAT_TABLE_SIZE) {
		pr_err("dt: parsing 'cap_to_vbat_ocv' failed\n");
		return -EINVAL;
	}

	of_property_read_u32_array(np, propname,
				&bprofile_ptr->cap_to_vbat_ocv[0],
					BAT_CAP_TO_VBAT_TABLE_SIZE);

	snprintf(propname, ARRAY_SIZE(propname), "prof-%s-model_name",
						bprofile_ptr->batt_id);
	if (of_property_read_string(np, propname, &mname)) {
		pr_err("dt: parsing 'model_name' failed\n");
		mname = "unknown";
	}
	strlen = strnlen(mname, BATTMODEL_STR_LEN);
	strncpy(bprofile_ptr->model_name, mname, strlen);

	return index;
}

int bprofile_parse_dt(struct battery_type **batteries, size_t *batteries_len,
			struct ps_pse_mod_prof **bprof, size_t *bprof_len,
						struct device_node *np)
{
	struct battery_type *supported_batids;
	struct ps_pse_mod_prof *bprofiles;
	size_t supported_batids_len;
	unsigned nprofiles;
	int i, ret, index;
	const char *pname;

	ret = of_property_count_strings(np, "supp_batids-map");

	if (ret <= 0) {
		pr_err("dt: parsing 'supp_batids-map' failed\n");
		return -ENODEV;
	}

	supported_batids_len = ret;
	supported_batids = kzalloc(supported_batids_len *
			sizeof(struct battery_type), GFP_KERNEL);

	if (!supported_batids)
		return -ENOMEM;

	*batteries = supported_batids;
	*batteries_len = supported_batids_len;

	ret = of_property_read_u32(np, "nprofiles", &nprofiles);
	if (ret) {
		pr_err("dt: parsing 'nprofiles' failed\n");
		goto nprofiles_fail;
	}

	bprofiles = kzalloc(nprofiles *
			sizeof(struct ps_pse_mod_prof), GFP_KERNEL);

	if (!bprofiles) {
		ret = -ENOMEM;
		goto bprofiles_alloc_fail;
	}

	*bprof = bprofiles;
	*bprof_len = nprofiles;

	for (i = 0; i < supported_batids_len; ++i) {
		u32 batid, battype;

		ret = of_property_read_string_index(np,
				"supp_batids-map", i, &pname);
		if (ret)
			goto supp_batids_fail;

		ret = dt_add_bprofile(np, pname, bprofiles, nprofiles);
		if (ret < 0)
			goto add_bprofile_fail;

		index = ret;

		supported_batids[i].profile.chrg_prof_type = PSE_MOD_CHRG_PROF;
		supported_batids[i].profile.batt_prof = &bprofiles[index];


		ret = of_property_read_u32_index(np, "supp_batids",
								i*2, &batid);
		if (ret < 0) {
			pr_err("dt: parsing 'supp_batids' failed\n");
			goto add_bprofile_fail;
		}

		supported_batids[i].batid_ohms = batid;


		ret = of_property_read_u32_index(np, "supp_batids",
							(i*2) + 1, &battype);
		if (ret < 0) {
			pr_err("dt: parsing 'supp_batids' failed\n");
			goto add_bprofile_fail;
		}

		supported_batids[i].type = battype;
	}

	return 0;

add_bprofile_fail:
supp_batids_fail:
	kfree(bprofiles);
	*bprof = NULL;
	*bprof_len = 0;
bprofiles_alloc_fail:
nprofiles_fail:
	kfree(supported_batids);
	*batteries = NULL;
	*batteries_len = 0;
	return ret;

}

#else

int bprofile_parse_dt(struct battery_type **batteries, size_t *batteries_len,
			struct ps_pse_mod_prof **bprof, size_t *bprof_len,
						struct device_node *np)
{
	return -EPERM;
}

#endif /*CONFIG_OF*/
