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

#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#if defined(CONFIG_PM_RUNTIME)
#include <linux/pm_runtime.h>
#endif

#include <linux/mali/mali_utgard.h>
#include <linux/xgold_noc.h>

#include "platform_intern.h"
#include "platform_native.h"
#include "platform_debugfs.h"


/* Throttle up and down thresholds must be in range 0..256 */
#define GPU_THROTTLE_UP_THRESHOLD 166 /*65%*/
#define GPU_THROTTLE_DOWN_THRESHOLD 76 /*30%*/


#if defined(GPU_NO_PMU)
static struct resource mali_gpu_0xE2E00000_mp1_res[] = {
MALI_GPU_RESOURCES_MALI400_MP1(0xE2E00000, -1, -1, -1, -1)
};

static struct resource mali_gpu_0xE2E00000_mp2_res[] = {
MALI_GPU_RESOURCES_MALI400_MP2(0xE2E00000, -1, -1, -1, -1, -1, -1)
};

static struct resource mali_gpu_0xE2E00000_mp4_res[] = {
MALI_GPU_RESOURCES_MALI400_MP4(0xE2E00000, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
};

static struct resource mali_gpu_0xEB300000_mp1_res[] = {
MALI_GPU_RESOURCES_MALI400_MP1(0xEB300000, -1, -1, -1, -1)
};

static struct resource mali_gpu_0xEB300000_mp2_res[] = {
MALI_GPU_RESOURCES_MALI400_MP2(0xEB300000, -1, -1, -1, -1, -1, -1)
};

static struct resource mali_gpu_0xEB300000_mp4_res[] = {
MALI_GPU_RESOURCES_MALI400_MP4(0xEB300000, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
};
#else
static struct resource mali_gpu_res[] = {
MALI_GPU_RESOURCES_MALI400_MP1_PMU(0xE2E00000, -1, -1, -1, -1)
};

static struct resource mali_gpu_mp2_res[] = {
MALI_GPU_RESOURCES_MALI400_MP2_PMU(0xE2E00000, -1, -1, -1, -1, -1, -1)
};

static struct resource mali_gpu_mp4_res[] = {
MALI_GPU_RESOURCES_MALI400_MP4_PMU(0xE2E00000, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
};

static struct resource mali_gpu_0xEB300000_mp1_res[] = {
MALI_GPU_RESOURCES_MALI400_MP1_PMU(0xEB300000, -1, -1, -1, -1)
};

static struct resource mali_gpu_0xEB300000_mp2_res[] = {
MALI_GPU_RESOURCES_MALI400_MP2_PMU(0xEB300000, -1, -1, -1, -1, -1, -1)
};

static struct resource mali_gpu_0xEB300000_mp4_res[] = {
MALI_GPU_RESOURCES_MALI400_MP4_PMU(0xEB300000, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
};
#endif


struct xgold_mali_resources {
	unsigned int base;
	unsigned id;
	struct resource *gpu_res;
	unsigned resource_sz;
} gpu_resources[] = {
	{ 0xE2E00000, 1, mali_gpu_0xE2E00000_mp1_res,
		ARRAY_SIZE(mali_gpu_0xE2E00000_mp1_res) },
	{ 0xE2E00000, 2, mali_gpu_0xE2E00000_mp2_res,
		ARRAY_SIZE(mali_gpu_0xE2E00000_mp2_res) },
	{ 0xE2E00000, 4, mali_gpu_0xE2E00000_mp4_res,
		ARRAY_SIZE(mali_gpu_0xE2E00000_mp4_res) },
	{ 0xEB300000, 1, mali_gpu_0xEB300000_mp1_res,
		ARRAY_SIZE(mali_gpu_0xEB300000_mp1_res) },
	{ 0xEB300000, 2, mali_gpu_0xEB300000_mp2_res,
		ARRAY_SIZE(mali_gpu_0xEB300000_mp2_res) },
	{ 0xEB300000, 4, mali_gpu_0xEB300000_mp4_res,
		ARRAY_SIZE(mali_gpu_0xEB300000_mp4_res) }
};

static struct mali_gpu_device_data xgold_mali_gpu_data;

static struct mali_platform_data plf_data;

static struct of_device_id xgold_graphics_of_match[] = {
	{ .compatible = "intel,graphics", },
	{ },
};

static struct mali_platform_pm mali_dev_pm;

struct mali_dev_dvfs_work_t {
	struct work_struct my_work;
	unsigned int util;
};


static int mali_platform_memory_layout(struct device_node *ngpu,
		struct mali_platform_data *pdata)
{
	int ret = 0, length = 0;
	u32 use_fbapi;
	struct property *prop;
	const __be32 *p;
	unsigned int val;
	u32 array[2];
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
				prop, p, val) {
			length++;
	};

	if (length == 2) {
			ret = of_property_read_u32_array(ngraphics,
					"intel,dcc-mem", array, 2);
			pdata->gpu_data->fb_start = array[0];
			pdata->gpu_data->fb_size = array[1];
	} else {
			pdata->gpu_data->fb_start = 0;
			pdata->gpu_data->fb_size = 0;
	}

	if (ret || (length != 2))
			mali_dbg("Can't read property:%s\n", "intel,dcc-mem");
	} else {
		mali_dbg("Framebuffer memory region not defined\n");
		pdata->gpu_data->fb_start = 0;
		pdata->gpu_data->fb_size = 0;
	}

	/* dedicated memory */
	ret = of_property_read_u32_array(ngraphics,
					"intel,gpu-rsvd-mem", array, 2);
	if (ret) {
		mali_dbg("Dedicated memory region not defined\n");
		pdata->gpu_data->dedicated_mem_start = 0;
		pdata->gpu_data->dedicated_mem_size = 0;
	} else {
		pdata->gpu_data->dedicated_mem_start = array[0];
		pdata->gpu_data->dedicated_mem_size = array[1];
	}

	/* shared memory */
	ret = of_property_read_u32_array(ngraphics,
					"intel,gpu-shared-mem", array, 1);
	if (ret) {
		mali_dbg("Shared memory region not defined\n");
		pdata->gpu_data->shared_mem_size = 0;
	} else {
		pdata->gpu_data->shared_mem_size = array[0];
	}

	mali_info("fb   region @0x%08lx length = 0x%08lx (%liMB)\n",
			pdata->gpu_data->fb_start,
			pdata->gpu_data->fb_size,
			pdata->gpu_data->fb_size/1024/1024);
	mali_info("rsvd region @0x%08lx length = 0x%08lx (%liMB)\n",
			pdata->gpu_data->dedicated_mem_start,
			pdata->gpu_data->dedicated_mem_size,
			pdata->gpu_data->dedicated_mem_size/1024/1024);
	mali_info("os   region length = 0x%08lx (%liMB)\n",
			pdata->gpu_data->shared_mem_size,
			pdata->gpu_data->shared_mem_size/1024/1024);
	return 0;

}

static int mali_platform_update_irq(struct platform_device *pdev, int irq)
{
	struct resource *p_curr_mali_gpu_resource;
	int i;

	for (i = 0; i < pdev->num_resources; i++) {
		p_curr_mali_gpu_resource = &pdev->resource[i];
		if (p_curr_mali_gpu_resource->flags == IORESOURCE_IRQ) {
			mali_dbg("Update %s from %d to %d\n",
				p_curr_mali_gpu_resource->name,
				p_curr_mali_gpu_resource->start, irq);
			p_curr_mali_gpu_resource->start =
				p_curr_mali_gpu_resource->end = irq;
		}
	}

	return 0;
}

static int once_glb = 1;
static void mali_dev_do_dvfs(struct work_struct *work)
{
	unsigned int util = ((struct mali_dev_dvfs_work_t *)work)->util;
	unsigned int prev_pm_state;
	int ret;

	if (mali_dev_pm.dvfs_off)
		return;

	if ((util < GPU_THROTTLE_DOWN_THRESHOLD) &&
		(mali_dev_pm.curr_pm_state > GPU_MIN_PM_STATE)) {
		/*
		Ramp down step by step, if utilization is lower
		than GPU_THROTTLE_DOWN_THRESHOLD
		*/
		mali_dev_pm.curr_pm_state--;
		mali_dbg("Utilization %d/256; Lowering power state to %d\n",
			util, mali_dev_pm.curr_pm_state);
		ret = platform_device_pm_set_state(mali_dev_pm.pdev,
			mali_dev_pm.pm_states[
			mali_dev_pm.curr_pm_state]);
		if (ret != 0) {
			mali_dev_pm.curr_pm_state++;
			mali_err("Utilization set state failed (%d)\n", ret);
			}
	} else if ((util > GPU_THROTTLE_UP_THRESHOLD) &&
		(mali_dev_pm.curr_pm_state < GPU_MAX_PM_STATE)) {
		/*
		Switch to maximum power level, if utilization is higher
		than GPU_THROTTLE_UP_THRESHOLD
		*/
		prev_pm_state = mali_dev_pm.curr_pm_state;
		mali_dev_pm.curr_pm_state = GPU_MAX_PM_STATE;
		mali_dbg("Utilization %d/256, Increasing power state to %d\n",
			util, mali_dev_pm.curr_pm_state);
		ret = platform_device_pm_set_state(mali_dev_pm.pdev,
			mali_dev_pm.pm_states[
			mali_dev_pm.curr_pm_state]);
		if (once_glb && (prev_pm_state == GPU_MIN_PM_STATE)) {
			xgold_noc_qos_set("GPU");
			once_glb--;
		}
		if (ret != 0) {
			mali_dev_pm.curr_pm_state = prev_pm_state;
			mali_err("Utilization PM set state failed (%d)\n", ret);
			}
	}

	kfree(work);
}

static void mali_gpu_utilization_callback(
	struct mali_gpu_utilization_data *data)
{
	mali_dev_pm.dvfs_work = kmalloc(sizeof(struct mali_dev_dvfs_work_t),
		GFP_ATOMIC);

	if (mali_dev_pm.dvfs_work) {
		INIT_WORK((struct work_struct *)mali_dev_pm.dvfs_work,
			mali_dev_do_dvfs);
		mali_dev_pm.dvfs_work->util = data->utilization_gpu;
		(void)queue_work(mali_dev_pm.dvfs_wq,
			(struct work_struct *)mali_dev_pm.dvfs_work);
	}

}


int mali_platform_init(void)
{
	mali_info("%s()\n", __func__);

#if !defined(CONFIG_PLATFORM_DEVICE_PM_VIRT)
	return mali_platform_native_init(&plf_data);
#else
	return 0;
#endif
}

int mali_platform_probe(struct platform_device *pdev,
	const struct dev_pm_ops *pdev_pm_ops)
{
	int ret = 0, i, mali_do_dvfs = 0;
	unsigned int irq;
	struct device_node *np;
	unsigned int nr_cores, regbase;

	mali_info("%s()\n", __func__);

	np = pdev->dev.of_node;

	ret = of_property_read_u32(np, "reg", &regbase);
	if (ret) {
		mali_err("Could not get register base adress\n");
		return -1;
	}

	ret = of_property_read_u32(np, "intel,mali,cores", &nr_cores);
	if (ret)
		nr_cores = 1;

	for (i = 0; i < ARRAY_SIZE(gpu_resources); i++)
		if (gpu_resources[i].base == regbase)
			if (gpu_resources[i].id == nr_cores)
				break;

	mali_info("Using %d cores\n",  nr_cores);

	pdev->name                  = MALI_GPU_NAME_UTGARD;
	pdev->id                    = 0;

	pdev->num_resources         = gpu_resources[i].resource_sz;
	pdev->resource              = gpu_resources[i].gpu_res;

	pdev->dev.platform_data     = &xgold_mali_gpu_data;

	plf_data.gpu_data = &xgold_mali_gpu_data;

	/* Get shared interrupt vector from DTS */
	irq = irq_of_parse_and_map(np, 0);
	ret = mali_platform_update_irq(pdev, irq);
	if (!irq || ret) {
		mali_err("Could not update irq\n");
		return -1;
	}

	if (of_property_read_bool(np, "intel,mali,dvfs"))
		mali_do_dvfs = 1;

	mali_info("dvfs %s\n", mali_do_dvfs?"ON":"OFF");


#if !defined(CONFIG_PLATFORM_DEVICE_PM_VIRT)
	mali_platform_native_probe(&(pdev->dev), np);
#endif

	plf_data.pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(plf_data.pm_platdata)) {
		mali_err("Device state pm setzo\n");
		return -1;
	}

	ret = platform_device_pm_set_class(pdev,
			plf_data.pm_platdata->pm_user_name);
	if (ret) {
		mali_err("Device state pm set class\n");
		return ret;
	}

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
#if defined(GPU_USE_ULTRA_HIGH_PERF)
	mali_dev_pm.pm_states[4] =
		platform_device_pm_get_state_handler(pdev,
		"ultra_high_perf");
#endif /* defined(GPU_USE_ULTRA_HIGH_PERF) */
	if (mali_dev_pm.pm_states[0] == 0
		|| mali_dev_pm.pm_states[1] == 0
		|| mali_dev_pm.pm_states[2] == 0
		|| mali_dev_pm.pm_states[3] == 0
#if defined(GPU_USE_ULTRA_HIGH_PERF)
		|| mali_dev_pm.pm_states[4] == 0
#endif /* defined(GPU_USE_ULTRA_HIGH_PERF) */
		) {
		mali_err("Device pm unable to get state handler\n");
		return -1;
	}

	/* Set utilization callback */
	plf_data.gpu_data->utilization_interval = 100;
	plf_data.gpu_data->utilization_callback =
		mali_gpu_utilization_callback;
	mali_dev_pm.dvfs_wq = create_workqueue("CONFIG_GPU_DVFS_work_queue");
	if (mali_dev_pm.dvfs_wq == 0) {
		mali_err("Unable to create workqueue for dvfs\n");
		return -1;
	}
	if (mali_do_dvfs) {

		mali_dev_pm.dvfs_off = false;

		mali_dev_pm.curr_pm_state = GPU_INITIAL_PM_STATE;
		mali_dev_pm.resume_pm_state = GPU_INITIAL_PM_STATE;
	} else {
		mali_dev_pm.dvfs_off = true;

		mali_dev_pm.curr_pm_state = GPU_MAX_PM_STATE;
		mali_dev_pm.resume_pm_state = GPU_MAX_PM_STATE;
	}

	ret = platform_device_pm_set_state(pdev,
		mali_dev_pm.pm_states[mali_dev_pm.curr_pm_state]);
	if (ret < 0) {
		mali_err("Device pm set state failed (%d)\n", ret);
		destroy_workqueue(mali_dev_pm.dvfs_wq);
		return ret;
	}

	mali_dev_pm.pdev = pdev;

	ret = mali_platform_memory_layout(np, &plf_data);
	if (ret)
		mali_err("Memory layout is not set !!\n");

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_set_autosuspend_delay(&(pdev->dev), 1000);
	pm_runtime_use_autosuspend(&(pdev->dev));
	pm_runtime_enable(&(pdev->dev));
#endif

	platform_debugfs_register(&mali_dev_pm, pdev, pdev_pm_ops);

	return ret;
}


int mali_platform_remove(struct platform_device *pdev)
{
	mali_info("%s()\n", __func__);

	destroy_workqueue(mali_dev_pm.dvfs_wq);

	return 0;
}

int mali_platform_suspend(struct platform_device *pdev)
{
	int ret;
	bool skip_suspend = false;

	mali_info("%s()\n", __func__);

#if defined(CONFIG_PM_RUNTIME)
	/* Check Runtime PM status */
	skip_suspend =
		mali_dev_pm.runtime_suspended =
			pm_runtime_suspended(&(pdev->dev));
#endif

	if (!skip_suspend) {
		flush_workqueue(mali_dev_pm.dvfs_wq);

#if !defined(GPU_NO_PMU)
		ret = mali_pmu_powerdown();
		if (ret != 0) {
			mali_err("Device pmu powerdown failed(%d)\n", ret);
			return ret;
		}
#endif

		ret = platform_device_pm_set_state(pdev,
			mali_dev_pm.pm_states[0]);
		if (ret) {
			mali_err("Device pm set state failed (%d)\n", ret);
				return ret;
		} else
			mali_dev_pm.curr_pm_state = 0;
	}

	return 0;
}


int mali_platform_resume(struct platform_device *pdev)
{
	int ret;
	once_glb = 1;
	mali_info("%s()\n", __func__);

	ret = platform_device_pm_set_state(pdev,
		mali_dev_pm.pm_states[mali_dev_pm.resume_pm_state]);
	if (ret != 0) {
		mali_err("Device pm set state failed (%d)\n", ret);
		return ret;
	}
	mali_dev_pm.curr_pm_state = mali_dev_pm.resume_pm_state;

	xgold_noc_qos_set("GPU");
#if !defined(GPU_NO_PMU)
	ret = mali_pmu_powerup();
	if (ret != 0) {
		mali_err("Device pmu powerup failed (%d)\n", ret);
		return ret;
	}
#endif

	/* ToDo: Verify if it's ok to call this before mali_pm_os_resume! */
#if defined(CONFIG_PM_RUNTIME)
	/*
	Update runtime PM status to reflect the actual post-system sleep status.
	That means avoid calling mali_platform_runtime_resume if we come out of
	OS suspend and were runtime suspended beforehand. See pm_runtime.txt
	*/
	if (mali_dev_pm.runtime_suspended) {
		mali_dbg("Skip runtime resume\n");
		pm_runtime_disable(&(pdev->dev));
		pm_runtime_set_active(&(pdev->dev));
		pm_runtime_enable(&(pdev->dev));
		mali_dev_pm.runtime_suspended = false;
	}
#endif

	return 0;
}

#if defined(CONFIG_PM_RUNTIME)
int mali_platform_runtime_suspend(struct platform_device *pdev)
{
	int ret;

	mali_dbg("%s()\n", __func__);

	if (mali_dev_pm.curr_pm_state == 0)
		mali_dbg("Already powered down!\n");

	flush_workqueue(mali_dev_pm.dvfs_wq);
	/* Note: mali_utilization_suspend not needed in runtime suspend */

#if !defined(GPU_NO_PMU)
	ret = mali_pmu_powerdown();
	if (ret != 0) {
		mali_err("Device pmu powerdown failed(%d)\n", ret);
		return ret;
	}
#endif

	ret = platform_device_pm_set_state(pdev,
		mali_dev_pm.pm_states[0]);
	if (ret != 0)
		mali_err("Device pm set state failed (%d)\n", ret);
	else
		mali_dev_pm.curr_pm_state = 0;

	return 0;
}


int mali_platform_runtime_resume(struct platform_device *pdev)
{
	int ret;

	mali_dbg("%s()\n", __func__);

	if (mali_dev_pm.curr_pm_state > 0)
		mali_dbg("Already powered up!\n");

	ret = platform_device_pm_set_state(pdev,
		mali_dev_pm.pm_states[mali_dev_pm.resume_pm_state]);
	if (ret != 0) {
		mali_err("Device pm set state failed (%d)\n", ret);
		return ret;
	}
	mali_dev_pm.curr_pm_state = mali_dev_pm.resume_pm_state;

#if !defined(GPU_NO_PMU)
	ret = mali_pmu_powerup();
	if (ret != 0) {
		mali_err("Device pmu powerup failed (%d)\n", ret);
		return ret;
	}
#endif

	return 0;
}

int mali_platform_runtime_idle(struct platform_device *pdev)
{
	mali_dbg("%s()\n", __func__);

	pm_runtime_autosuspend(&(pdev->dev));

	return 0;
}
#endif
