/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Notes:
 * Aug 25 2014: IMC: Native basic support of dvfs
 * Jul 16 2014: IMC: [OSS Scan] Add missing license type
 * Jun 02 2014: IMC: Splitup platform adaption for better readability
 */

#if !defined(CONFIG_PLATFORM_DEVICE_PM_VIRT)

#include <linux/platform_device.h>

#include <linux/of.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include "platform_intern.h"


#define PROP_GPU_SUPPLY	"gpu"
#define OF_KERNEL_CLK	"clk_kernel"
#define OF_AHB_CLK	"clk_ahb"


static struct mali_platform_data *p_plf_data;

static int mali_plf_set_pm_state(struct device *,
		struct device_state_pm_state *);

static struct device_state_pm_state *mali_plf_get_initial_state(
		struct device *);

static struct device_state_pm_ops gpu_pm_ops = {
	.set_state = mali_plf_set_pm_state,
	.get_initial_state = mali_plf_get_initial_state,
};

/* PM states & class */
static struct device_state_pm_state gpu_pm_states[] = {
	{ .name = "disable",   }, /* D3 */
	{ .name = "low_perf",  }, /* D0i3 */
	{ .name = "mid_perf",  }, /* D0i2 */
	{ .name = "high_perf", }, /* D0 */
	{ .name = "ultra_high_perf", }, /* D0 */
};

DECLARE_DEVICE_STATE_PM_CLASS(gpu);


static int mali_platform_clock_enable(struct clk *clock)
{
	int ret = 0;

	ret = clk_prepare(clock);
	if (ret) {
		mali_err("Error preparing clk\n");
		return ret;
	}

	ret = clk_enable(clock);
	if (ret)
		mali_err("Error enabling clk\n");

	return ret;
}

static int mali_platform_clock_set(struct mali_platform_data *pdata, int en)
{
	int ret = 0;

	if (en) {
		if (pdata && pdata->clk_kernel) {
			ret = mali_platform_clock_enable(pdata->clk_kernel);
			if (ret) {
				mali_err("Error enabling kernel clk\n");
				return -1;
			}
		}

		if (pdata && pdata->clk_ahb) {
			ret = mali_platform_clock_enable(pdata->clk_ahb);
			if (ret) {
				mali_err("Error enabling ahb clk\n");
				return -1;
			}
		}
		mali_info("Clock frequency is %li MHz\n",
				clk_get_rate(pdata->clk_kernel)/1000/1000);
	} else {
		if (!IS_ERR_OR_NULL(pdata->clk_kernel))
			clk_disable(pdata->clk_kernel);

		if (!IS_ERR_OR_NULL(pdata->clk_ahb))
			clk_disable(pdata->clk_ahb);
	}

	return ret;
}

static int mali_platform_power_set(struct mali_platform_data *pdata, int en)
{
	int ret;

	if (en) {
		if (pdata->regul) {
			ret = regulator_enable(pdata->regul);
			if (ret)
				return ret;
		}
	} else {
		if (pdata->regul) {
			/* disable LDOs */
			ret = regulator_disable(pdata->regul);
			if (ret)
				return ret;
			mali_err("Regulator OFF\n");
		}
	}

	return 0;
}

static int mali_plf_set_pm_state_by_num(int state_num)
{
	if (MALI_PLF_PM_STATE_D3) {
		/*mali_pmu_powerdown();*/
		mali_platform_clock_set(p_plf_data, 0);
		mali_platform_power_set(p_plf_data, 0);
	} else {
		mali_platform_power_set(p_plf_data, 1);
		mali_platform_clock_set(p_plf_data, 1);
		/*mali_pmu_powerup();*/
	}

	return 0;
}

static int mali_plf_get_pm_state_id(char *name)
{
	int id;

	for (id = MALI_PLF_PM_STATE_D3; id < GPU_NUM_PM_STATES; id++) {
		if (!strcmp(name, gpu_pm_states[id].name))
			return id;
	}
	return GPU_NUM_PM_STATES;
}

static int mali_plf_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	int state_num;
	int ret = 0;

	state_num = mali_plf_get_pm_state_id(state->name);
	mali_info("set pm state (%d) %s\n", state_num, state->name);
	ret = mali_plf_set_pm_state_by_num(state_num);
	if (ret < 0)
		mali_err("Unable to set pm state (%d)%s\n",
				state_num, state->name);

	return ret;
}

static struct device_state_pm_state *mali_plf_get_initial_state(
		struct device *dev)
{
	return &gpu_pm_states[MALI_PLF_PM_STATE_D3];
}


int mali_platform_native_init(struct mali_platform_data *pdata)
{
	int ret;

	mali_info("%s()\n", __func__);

	if (pdata == NULL)
		return -EINVAL;

	ret = device_state_pm_add_class(&gpu_pm_class);
	if (ret < 0)
		return ret;

	p_plf_data = pdata;

	return 0;
}

void mali_platform_native_probe(struct device *dev, struct device_node *np)
{
	/* power */
	p_plf_data->regul = regulator_get(dev, PROP_GPU_SUPPLY);
	if (IS_ERR(p_plf_data->regul)) {
		mali_err("%s can't get %s-supply handle\n",
					PROP_GPU_SUPPLY, np->name);
		p_plf_data->regul = NULL;
	}

	/* clock */
	p_plf_data->clk_kernel = of_clk_get_by_name(np, OF_KERNEL_CLK);
	if (IS_ERR(p_plf_data->clk_kernel)) {
		mali_err("Clk %s not found\n", OF_KERNEL_CLK);
		p_plf_data->clk_kernel = NULL;
	}
	p_plf_data->clk_ahb = of_clk_get_by_name(np, OF_AHB_CLK);
	if (IS_ERR(p_plf_data->clk_ahb)) {
		mali_err("Clk %s not found\n", OF_AHB_CLK);
		p_plf_data->clk_ahb = NULL;
	}
}

#endif
