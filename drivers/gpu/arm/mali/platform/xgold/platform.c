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
 * Nov 18 2014: IMC: Adaptions for Mali Utgard driver r5p0-01rel0
 *                   - Use Mali DT support instead of Intel bringup variant
 *                   - Add Mali DVFS support
 *                   - Simplify power management code on Kernel 3.14
 *                   - New method to enable/disable DVFS via debugfs
 *                   - Set NOC GPU QoS only on Probe and on os_resume
 *                   - Cleanup Mali PMU function calls
 * Aug 25 2014: IMC: Support various register base adresses
 *                   Native basic support of dvfs
 * Aug 14 2013: IMC: Set QoS config
 * Jul 31 2014: IMC: Add MPx support at runtime
 * Jul 23 2014: IMC: Update fb dts parsing
 * Jul 16 2014: IMC: [OSS Scan] Add missing license type
 * Jun 02 2014: IMC: Add pm and debugfs support
 *                   Splitup platform adaption for better readability
 *                   Change hooks to platform adaption to use platform_device
 * Mar 14 2014: IMC: Add basic DTS support for SoFIA
 */

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/kdev_t.h>

#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#if defined(CONFIG_PM_RUNTIME)
#include <linux/pm_runtime.h>
#endif

#include <linux/mali/mali_utgard.h>

#include "platform_intern.h"
#include "platform_native.h"
#include "platform_debugfs.h"


static struct mali_gpu_device_data xgold_mali_gpu_data;
static struct mali_platform_data plf_data;
static struct mali_platform_pm mali_dev_pm;

static bool restore_gpu_qos;

static struct of_device_id xgold_graphics_of_match[] = {
	{ .compatible = "intel,graphics", },
	{ },
};

#define GPU_MAX_PM_STATE (mali_dev_pm.pm_status_num-1)
#define GPU_INITIAL_PM_STATE GPU_MAX_PM_STATE


#define GPU_NUM_DVFS_STEPS (GPU_NUM_PM_STATES - 1)
struct work_struct mali_setting_clock_work;
struct mali_gpu_clk_item mali_dvfs[GPU_NUM_DVFS_STEPS];
struct mali_gpu_clock mali_clock_items = {
	.item = &mali_dvfs[0],
	.num_of_steps = GPU_NUM_DVFS_STEPS
};

static int mali_platform_dvfs_config(struct device_node *np)
{
	int i, length = 0;
	struct mali_gpu_clk_item *curr_item;
	struct property *prop;
	const __be32 *p;
	unsigned int val;

	of_property_for_each_u32(np, "dvfs_clock_config",
			prop, p, val)
		length++;

	if (!length) {
		mali_err("dvfs_clock_config is not set!!\n");
		return -1;
	}

	if (length%2) {
		mali_err("dvfs_clock_config uneven number of tokens!!\n");
		return -1;
	}

	if ((length/2) > mali_clock_items.num_of_steps) {
		mali_err("dvfs_clock_config supports maximum %d slots\n",
			mali_clock_items.num_of_steps);
		return -1;
	}

	mali_clock_items.num_of_steps = length / 2;

	mali_info("dvfs_clock_config from DT:\n");
	for (i = 0; i < mali_clock_items.num_of_steps; i++) {
		curr_item = &mali_clock_items.item[i];

		of_property_read_u32_index(np, "dvfs_clock_config",
			2*i, &curr_item->clock);
		of_property_read_u32_index(np, "dvfs_clock_config",
			2*i+1, &curr_item->vol);

		mali_info("{%d, %d}\n", curr_item->clock, curr_item->vol);
	}

	return 0;
}

static void mali_dev_do_dvfs(struct work_struct *work)
{
	unsigned int prev_pm_state;
	int ret;

	prev_pm_state = mali_dev_pm.curr_pm_state;
	if (prev_pm_state == mali_dev_pm.req_clock_index + 1) {
		mali_dbg("Device pm already at state %d DVFS\n",
			prev_pm_state);
		return;
	}

	mali_dev_pm.curr_pm_state = mali_dev_pm.req_clock_index + 1;
	mali_dbg("mali_gpu_set_clock_step(%d) DVFS \t-> Device pm set state to %d\n",
		mali_dev_pm.req_clock_index, mali_dev_pm.curr_pm_state);
	mali_dev_pause();
	ret = platform_device_pm_set_state(mali_dev_pm.pdev,
		mali_dev_pm.pm_states[
		mali_dev_pm.curr_pm_state]);
	mali_dev_resume();
	if (ret) {
		mali_dev_pm.curr_pm_state = prev_pm_state;
		mali_err("Device pm set state failed (%d) DVFS\n", ret);
	}
}

int mali_gpu_set_clock_step(int setting_clock_step)
{
	if (mali_dev_pm.curr_pm_state > 0) {
		mali_dev_pm.req_clock_index = setting_clock_step;

		if (mali_dev_pm.req_clock_index + 1 >
					mali_dev_pm.pm_limit_level) {
			mali_dbg("Thermal limit pm level to %d\n",
				mali_dev_pm.pm_limit_level);
			mali_dev_pm.req_clock_index =
				mali_dev_pm.pm_limit_level - 1;
		}
		/*
		Only schedule clock / pm level change if DVFS is on. If it is
		switched off, we just update the requested clock index, but
		do not queue the work.
		*/
		if (!mali_dev_pm.dvfs_off) {
			queue_work(mali_dev_pm.dvfs_wq,
				&mali_setting_clock_work);
		}

		return MALI_TRUE;
	} else
		return MALI_FALSE;
}

void mali_report_gpu_clock_info(struct mali_gpu_clock **data)
{
	int i;
	struct mali_gpu_clk_item *curr_item;

	mali_dbg("mali_report_gpu_clock_info:\n");
	for (i = 0; i < mali_clock_items.num_of_steps; i++) {
		curr_item = &mali_clock_items.item[i];
		mali_dbg("{%d, %d}\n", curr_item->clock, curr_item->vol);
	}

	*data = &mali_clock_items;
}

int mali_gpu_get_clock_step(void)
{
	/*
	Always report the last requested clock index. Might not match with the
	real clock level.
	*/
	return mali_dev_pm.req_clock_index;
}


static int mali_platform_memory_layout(struct mali_gpu_device_data *gpu_data)
{
	int ret = 0, length = 0;
	u32 use_fbapi;
	struct property *prop;
	const __be32 *p;
	unsigned int val;
	u32 array[2] = {0, 0};
	struct device_node *ngraphics =
		of_find_matching_node(NULL, xgold_graphics_of_match);

	if (!ngraphics) {
		mali_err("Can't find graphics matching node\n");
		return -1;
	}

	/* Read fb API flag */
	ret = of_property_read_u32(ngraphics, "intel,fb-api",
				&use_fbapi);

	if (use_fbapi) {
		of_property_for_each_u32(ngraphics, "intel,dcc-mem",
				prop, p, val)
			length++;

		if (length == 2) {
			ret = of_property_read_u32_array(ngraphics,
				"intel,dcc-mem", array, 2);
			gpu_data->fb_start = array[0];
			gpu_data->fb_size = array[1];
		}
	}

	/* protected memory (secvm)
	 * only considered if FB not set */
	if ((array[0] == 0) && (array[1] == 0)) {
		ret = of_property_read_u32_array(
				ngraphics, "intel,prot-mem", array, 2);
		if (ret && ret != -EINVAL) {
			/* property was specified, error
			 * occurred while reading */
			mali_err("could not read prot-mem property, err=%d",
					ret);
		} else {
			gpu_data->fb_start = array[0];
			gpu_data->fb_size = array[1];
		}
	}

	/* dedicated memory */
	ret = of_property_read_u32_array(ngraphics,
					"intel,gpu-rsvd-mem", array, 2);
	if (ret) {
		mali_dbg("Dedicated memory region not defined\n");
		gpu_data->dedicated_mem_start = 0;
		gpu_data->dedicated_mem_size = 0;
	} else {
		gpu_data->dedicated_mem_start = array[0];
		gpu_data->dedicated_mem_size = array[1];
	}

	/* shared memory */
	ret = of_property_read_u32_array(ngraphics,
					"intel,gpu-shared-mem", array, 1);
	if (ret) {
		mali_dbg("Shared memory region not defined\n");
		gpu_data->shared_mem_size = 0;
	} else {
		gpu_data->shared_mem_size = array[0];
	}

	mali_info("fb   region @0x%08lx length = 0x%08lx (%liMB)\n",
			gpu_data->fb_start,
			gpu_data->fb_size,
			gpu_data->fb_size/1024/1024);
	mali_info("rsvd region @0x%08lx length = 0x%08lx (%liMB)\n",
			gpu_data->dedicated_mem_start,
			gpu_data->dedicated_mem_size,
			gpu_data->dedicated_mem_size/1024/1024);
	mali_info("os   region length = 0x%08lx (%liMB)\n",
			gpu_data->shared_mem_size,
			gpu_data->shared_mem_size/1024/1024);
	return 0;

}

static ssize_t max_freq_level_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int max_freq_level = -1;
	int ret = 0;
	unsigned int prev_pm_state = 0;

	ret = sscanf(buf, "%d", &max_freq_level);
	if (ret != 1)
		return 0;

	mali_dbg("set max freq %d\n", max_freq_level);
	if (max_freq_level > 0 && max_freq_level <= GPU_MAX_PM_STATE) {
		mali_dev_pm.pm_limit_level = max_freq_level;
		mali_dev_pm.resume_pm_state = max_freq_level;
		flush_workqueue(mali_dev_pm.dvfs_wq);
		if (!mali_dev_pm.dvfs_off) {
			if (mali_dev_pm.curr_pm_state >
					mali_dev_pm.pm_limit_level)
				mali_gpu_set_clock_step(
					mali_dev_pm.pm_limit_level - 1);
		} else if (mali_dev_pm.curr_pm_state != max_freq_level) {
			prev_pm_state = mali_dev_pm.curr_pm_state;
			mali_dev_pm.curr_pm_state = max_freq_level;
			mali_dev_pause();
			ret = platform_device_pm_set_state(mali_dev_pm.pdev,
					mali_dev_pm.pm_states[max_freq_level]);
			mali_dev_resume();
			if (ret) {
				mali_dev_pm.curr_pm_state = prev_pm_state;
				mali_err("set pm state failed (%d)\n", ret);
				return 0;
			}
		}
	}

	return count;
}

static ssize_t max_freq_level_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", mali_dev_pm.pm_limit_level);
}

static DEVICE_ATTR(max_freq_level, S_IRUGO | S_IWUSR,
			max_freq_level_show, max_freq_level_store);

int mali_platform_init(void)
{
	mali_info("%s()\n", __func__);

#if !defined(CONFIG_PLATFORM_DEVICE_PM_VIRT)
	return mali_platform_native_init(&plf_data);
#else
	return 0;
#endif
}

void mali_qos_set(struct noc_qos_list *qos_root)
{
	struct noc_qos_list *qos;

	list_for_each_entry(qos, &qos_root->list, list) {
		mali_dbg("Set QoS config %s\n", qos->name);
		xgold_noc_qos_set((char *)qos->name);
	}
}

int mali_platform_device_init(struct platform_device *pdev)
{
	int ret = -1;
	struct device_node *np;
	u32 val32 = 0;
	bool ultra_high_flag = false;

	mali_info("%s()\n", __func__);

	np = pdev->dev.of_node;

	plf_data.gpu_data = &xgold_mali_gpu_data;


#if !defined(CONFIG_PLATFORM_DEVICE_PM_VIRT)
	mali_platform_native_probe(&(pdev->dev), np);
#endif

	plf_data.pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(plf_data.pm_platdata)) {
		mali_err("Device state pm setup from devicetree\n");
		return -1;
	}

	ret = platform_device_pm_set_class(pdev,
			plf_data.pm_platdata->pm_user_name);
	if (ret) {
		mali_err("Device state pm set class\n");
		return ret;
	}

	/* QoS */
	mali_err("Device qos\n");
	of_noc_qos_populate(&(pdev->dev), np, &plf_data.qos);

	ultra_high_flag = of_property_read_bool(np, "dvfs_ultra_high");
	mali_info("ultra_high_perf is %s\n",
		ultra_high_flag?"enabled":"disabled");

	if (ultra_high_flag)
		mali_clock_items.num_of_steps = 4;
	else
		mali_clock_items.num_of_steps = 3;

	mali_dev_pm.pm_status_num = mali_clock_items.num_of_steps+1;
	mali_dev_pm.pm_limit_level = GPU_MAX_PM_STATE;

	/* Is this the right way to get the state handlers when you use DTS? */
	mali_dev_pm.pm_states[0] =
		platform_device_pm_get_state_handler(pdev,
		"disable");
	mali_dev_pm.pm_states[1] =
		platform_device_pm_get_state_handler(pdev,
		"low_perf");
	mali_dev_pm.pm_states[2] =
		platform_device_pm_get_state_handler(pdev,
		"mid_perf");
	mali_dev_pm.pm_states[3] =
		platform_device_pm_get_state_handler(pdev,
		"high_perf");


	if (mali_dev_pm.pm_states[0] == 0
		|| mali_dev_pm.pm_states[1] == 0
		|| mali_dev_pm.pm_states[2] == 0
		|| mali_dev_pm.pm_states[3] == 0
		) {
		mali_err("Device pm unable to get state handler\n");
		return -1;
	}
	if (ultra_high_flag) {
		mali_dev_pm.pm_states[4] =
			platform_device_pm_get_state_handler(pdev,
			"ultra_high_perf");

		if (mali_dev_pm.pm_states[4] == 0) {
			mali_err("Device pm unable to get state handler\n");
			return -1;
		}
	}

#if defined(CONFIG_PM_RUNTIME)
	/* Runtime resume will switch things on */
	mali_dev_pm.curr_pm_state = MALI_PLF_PM_STATE_D3;
	mali_dev_pm.resume_pm_state = GPU_INITIAL_PM_STATE;

	/* Need to set GPU QoS on bootup */
	restore_gpu_qos = true;
#else
	mali_dev_pm.curr_pm_state = GPU_MAX_PM_STATE;
	mali_dev_pm.resume_pm_state = GPU_MAX_PM_STATE;

	ret = platform_device_pm_set_state(pdev,
		mali_dev_pm.pm_states[mali_dev_pm.curr_pm_state]);
	if (ret < 0) {
		mali_err("Device pm set state failed (%d)\n", ret);
		return ret;
	}

	/* Need to set GPU QoS on bootup */
	mali_qos_set(plf_data.qos);
	mali_dbg("Set GPU QoS\n");
#endif

	mali_dev_pm.req_clock_index = mali_dev_pm.resume_pm_state - 1;
	mali_dev_pm.pdev = pdev;


	mali_dev_pm.sys_class = class_create(THIS_MODULE, "mali");
	if (IS_ERR(mali_dev_pm.sys_class)) {
		mali_err("Create sys class failed\n");
		return -ENOMEM;
	}

	mali_dev_pm.sys_dev = device_create(mali_dev_pm.sys_class,
					NULL, MKDEV(0, 0), NULL, "pm");
	if (IS_ERR(mali_dev_pm.sys_dev)) {
		mali_err("Create sys pm device failed\n");
		return -ENOMEM;
	}

	if (device_create_file(mali_dev_pm.sys_dev, &dev_attr_max_freq_level)) {
		mali_err("Create device attribute failed\n");
		return -ENOMEM;
	}

	ret = mali_platform_memory_layout(plf_data.gpu_data);
	if (ret) {
		mali_err("Memory layout is not set !!\n");
		return ret;
	}

	/*
	Disable DVFS based on DT setting; Basically it's still registered
	and running but we ignore the clock switching requests from driver.
	This is a debug option!
	*/
	mali_dev_pm.dvfs_off = of_property_read_bool(np, "dvfs_off");
	mali_info("dvfs is %s\n", mali_dev_pm.dvfs_off?"disabled":"enabled");

	ret = mali_platform_dvfs_config(np);
	if (ret) {
		mali_err("DVFS config is not set !!\n");
		return ret;
	}

	/* Use AMR DVFS with 500ms intervall or by setting it by dtsi*/
	ret = of_property_read_u32(np, "dvfs_interval", &val32);
	if (ret)
		plf_data.gpu_data->control_interval = 500;
	else
		plf_data.gpu_data->control_interval = val32;

	plf_data.gpu_data->set_freq = mali_gpu_set_clock_step;
	plf_data.gpu_data->get_clock_info = mali_report_gpu_clock_info;
	plf_data.gpu_data->get_freq = mali_gpu_get_clock_step;

	INIT_WORK(&mali_setting_clock_work, mali_dev_do_dvfs);
	mali_dev_pm.dvfs_wq = alloc_ordered_workqueue("mali_setting_clock_wq",
		WQ_NON_REENTRANT);
	if (mali_dev_pm.dvfs_wq == 0) {
		mali_err("Unable to create workqueue for dvfs\n");
		return -1;
	}

	ret = platform_device_add_data(pdev, &xgold_mali_gpu_data,
		sizeof(xgold_mali_gpu_data));
	if (ret) {
		mali_err("Device platform_device_add_data\n");
		destroy_workqueue(mali_dev_pm.dvfs_wq);
		return ret;
	}

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_set_autosuspend_delay(&(pdev->dev), 1000);
	pm_runtime_use_autosuspend(&(pdev->dev));
	pm_runtime_enable(&(pdev->dev));
#endif

	platform_debugfs_register(&mali_dev_pm, pdev);

	return ret;
}

int mali_platform_device_deinit(struct platform_device *pdev)
{
	mali_info("%s()\n", __func__);

	mali_dev_pm.dvfs_off = true;
	flush_workqueue(mali_dev_pm.dvfs_wq);
	destroy_workqueue(mali_dev_pm.dvfs_wq);
	device_remove_file(mali_dev_pm.sys_dev, &dev_attr_max_freq_level);
	device_destroy(mali_dev_pm.sys_class, MKDEV(0, 0));
	class_destroy(mali_dev_pm.sys_class);

	return 0;
}


int mali_platform_suspend(struct platform_device *pdev)
{
	int ret;
	bool skip_suspend = false;

	mali_dbg("%s() \t\t-> Device pm set state to 0\n", __func__);

#if defined(CONFIG_PM_RUNTIME)
	/*
	If we were already runtime suspended, then we can skip switching
	to PM level zero again.
	*/
	skip_suspend = pm_runtime_suspended(&(pdev->dev));
#endif

	if (!skip_suspend) {
		flush_workqueue(mali_dev_pm.dvfs_wq);

		ret = platform_device_pm_set_state(pdev,
			mali_dev_pm.pm_states[0]);
		if (ret) {
			mali_err("Device pm set state failed (%d)\n", ret);
			return ret;
		} else
			mali_dev_pm.curr_pm_state = 0;
	} else
		mali_dbg("Skipped, as already runtime suspended\n");

	/* Need to restore GPU QoS on system resume */
	restore_gpu_qos = true;

	return 0;
}

int mali_platform_resume(struct platform_device *pdev)
{
	int ret;
	bool skip_resume = false;

	mali_dbg("%s() \t\t-> Device pm set state to %d\n", __func__,
		mali_dev_pm.resume_pm_state);

#if defined(CONFIG_PM_RUNTIME)
	/*
	If we were runtime suspended before system suspend, then we can skip
	switching things back on. Runtime resume will do this.
	*/
	skip_resume = pm_runtime_suspended(&(pdev->dev));
#endif

	if (!skip_resume) {
		ret = platform_device_pm_set_state(pdev,
			mali_dev_pm.pm_states[mali_dev_pm.resume_pm_state]);
		if (ret) {
			mali_err("Device pm set state failed (%d)\n", ret);
			return ret;
		}
		mali_dev_pm.curr_pm_state = mali_dev_pm.resume_pm_state;
		mali_dev_pm.req_clock_index = mali_dev_pm.curr_pm_state - 1;

		if (restore_gpu_qos) {
			restore_gpu_qos = false;
			mali_qos_set(plf_data.qos);
			mali_dbg("Restore GPU QoS\n");
		}
	} else
		mali_dbg("Skipped, as runtime suspended\n");

	return 0;
}


#if defined(CONFIG_PM_RUNTIME)
int mali_platform_runtime_suspend(struct platform_device *pdev)
{
	int ret;

	mali_dbg("%s() \t-> Device pm set state to 0\n", __func__);

	if (mali_dev_pm.curr_pm_state == 0)
		mali_warn("Already powered down!\n");

	flush_workqueue(mali_dev_pm.dvfs_wq);

	ret = platform_device_pm_set_state(pdev,
		mali_dev_pm.pm_states[0]);
	if (ret) {
		mali_err("Device pm set state failed (%d)\n", ret);
		return ret;
	} else
		mali_dev_pm.curr_pm_state = 0;

	/* Need to restore GPU QoS on system resume */
	restore_gpu_qos = true;

	return 0;
}

int mali_platform_runtime_resume(struct platform_device *pdev)
{
	int ret;

	mali_dbg("%s() \t-> Device pm set state to %d\n", __func__,
		mali_dev_pm.resume_pm_state);

	if (mali_dev_pm.curr_pm_state > 0)
		mali_warn("Already powered up at level %d!\n",
			mali_dev_pm.curr_pm_state);

	ret = platform_device_pm_set_state(pdev,
		mali_dev_pm.pm_states[mali_dev_pm.resume_pm_state]);
	if (ret) {
		mali_err("Device pm set state failed (%d)\n", ret);
		return ret;
	}
	mali_dev_pm.curr_pm_state = mali_dev_pm.resume_pm_state;
	mali_dev_pm.req_clock_index  = mali_dev_pm.curr_pm_state - 1;

	if (restore_gpu_qos) {
		restore_gpu_qos = false;
		mali_qos_set(plf_data.qos);
		mali_dbg("Restore GPU QoS\n");
	}

	return 0;
}

int mali_platform_runtime_idle(struct platform_device *pdev)
{
	mali_dbg("%s()\n", __func__);

	pm_runtime_autosuspend(&(pdev->dev));

	return 0;
}
#endif
