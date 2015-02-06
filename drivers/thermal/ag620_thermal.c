/*
* ag620_thermal.c - AG620 thermal protection driver.
*
* Copyright (C) 2015 Intel Corporation
*
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
* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define DRIVER_NAME "ag620_thermal"
#define pr_fmt(fmt) DRIVER_NAME": "fmt

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/wakelock.h>

/* PMIC temperature periodic measurement interval in MS */
#define AG620TP_SAMPLE_INTERVAL_MS (1000)

/* PMIC temperature measurement delay after resume in MS */
#define AG620TP_MEAS_DELAY_AFTER_RESUME_MS (5)

/* struct ag620tp_state_data - Agold620 Thermal protection data
 * @dw			Delayed workqueue.
 * @wakelock			Wake lock used for mandetory measurement during the resume.
 * @iio_client		iio client for PMIC0_TEMP sensor.
 * @resume_meas_req		Flag for indicating meausrement request during resume.
 */
struct ag620tp_state_data {
	struct delayed_work dw;
	struct wake_lock    wakelock;
	struct iio_channel  *iio_client;
	bool                resume_meas_req;
};

/* Agold620 Thermal protection data instance */
static struct ag620tp_state_data ag620tp_data = {
	.resume_meas_req = false,
};

/**
 * ag620tp_client_wq - Work queue that schedules pmic temp sensor
 *          measurement in every 1 sec for configuring
 *          MEAS_TEMP_ALERT register inside AG620.
 *
 * @param   [in] work queue
 */
static void ag620tp_client_wq(struct work_struct *wk)
{
	int ret = 0;
	int pmic_temp = 0;

	ret = iio_read_channel_raw(ag620tp_data.iio_client, &pmic_temp);
	if (ret < 0)
		pr_debug("PMICTEMP_SENSOR: iio temp read failed\n");
	else {
		pr_debug("PMICTEMP_SENSOR: iio temp read: %d\n",
			pmic_temp);
	}

	if (true == ag620tp_data.resume_meas_req) {
		pr_debug("PMICTEMP_SENSOR: iio temp read: %d\n",
			pmic_temp);
		ag620tp_data.resume_meas_req = false;
		wake_unlock(&ag620tp_data.wakelock);
	}
	schedule_delayed_work((struct delayed_work *)wk,
		msecs_to_jiffies(AG620TP_SAMPLE_INTERVAL_MS));
}

/**
 * ag620tp_probe - Initialises the driver OS resources when the device
 * has been found, then starts the state machine in a single threaded work.
 */
static int ag620tp_probe(struct platform_device *p_platform_dev)
{
	struct device *dev = &p_platform_dev->dev;
	const char *channel_name;
	int ret;

	ret = of_property_read_string(dev->of_node,
		"intel,sensor-names", &channel_name);
	if (ret)
		goto probe_err;

	pr_info("%s\n", channel_name);

	ag620tp_data.iio_client = iio_channel_get(
		NULL, channel_name);
	if (IS_ERR(ag620tp_data.iio_client)) {
		pr_err("iio channel get error\n");
		goto probe_err;
	}
	wake_lock_init(&ag620tp_data.wakelock, WAKE_LOCK_SUSPEND,
		"AG620_THERMAL_PROT_LOCK");
	/* Start low temperature handler client */
	INIT_DELAYED_WORK(&ag620tp_data.dw, ag620tp_client_wq);
	schedule_delayed_work(&ag620tp_data.dw,
		msecs_to_jiffies(AG620TP_SAMPLE_INTERVAL_MS));

	pr_debug("Thermal protection probing OK\n");
	return 0;

probe_err:
	pr_info("%s Probe failed\n", __func__);
	return -EIO;
}

/**
 * ag620tp_remove - Deregister with the power supply class and release
 * allocated resources.
 */
static int ag620tp_remove(struct platform_device *p_platform_dev)
{

	/* Unused parameter */
	(void)p_platform_dev;

	iio_channel_release(ag620tp_data.iio_client);
	cancel_delayed_work_sync(&ag620tp_data.dw);
	return 0;
}

/**
 * ag620tp_suspend() - Called when the system is attempting to suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int ag620tp_suspend(struct device *dev)
{
	/* Unused parameter */
	(void)dev;
	return 0;
}

/**
 * ag620tp_resume() - Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int ag620tp_resume(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	pr_debug("AG620: Thermal protection resume\n");
	wake_lock(&ag620tp_data.wakelock);
	ag620tp_data.resume_meas_req = true;

	schedule_delayed_work(&ag620tp_data.dw,
			msecs_to_jiffies(AG620TP_MEAS_DELAY_AFTER_RESUME_MS));
	return 0;
}

static const struct of_device_id ag620tp_id_table[] = {
	{ .compatible = "intel,ag620_thermal" },
	{}
};
MODULE_DEVICE_TABLE(of, ag620tp_id_table);

const struct dev_pm_ops ag620tp_pm = {
	.suspend = ag620tp_suspend,
	.resume  = ag620tp_resume,
};

static struct platform_driver ag620tp_driver = {
	.driver  = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ag620tp_id_table),
		.pm    = &ag620tp_pm,
	},
	.probe  = ag620tp_probe,
	.remove = ag620tp_remove,
};

/*
 * ag620tp_init - AG620 Thermal protection module
 *                intialisation function.
 */
static int __init ag620tp_init(void)
{
	pr_debug("AG620: Thermal protection init\n");
	return platform_driver_register(&ag620tp_driver);
}

/*
 * ag620tp_exit - AG620 Thermal protection module
 *                cleanup function.
 */
static void __exit ag620tp_exit(void)
{
	return platform_driver_unregister(&ag620tp_driver);
}

late_initcall(ag620tp_init);
module_exit(ag620tp_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Agold620 thermal protection");

