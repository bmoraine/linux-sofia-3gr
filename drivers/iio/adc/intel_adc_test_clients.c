/*
* intel_adc_test_clients.c - ADC stress test driver.
*
* Copyright (C) 2013 Intel Corporation
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/iio/intel_adc_hal_interface.h>
#include <linux/time.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/consumer.h>
#include <linux/iio/driver.h>
#define DRIVER_NAME "intel_adc_test"

#define INTEL_SENSOR_TEST

/*#define INTEL_ADC_TEST*/

#ifdef INTEL_ADC_TEST
static const char * const channel_data[] = {
	"VBAT_ADC",
	"ANAMON_ADC",
	"VBAT_MIN_ADC",
	"VBAT_OCV_ADC",
	"BATTEMP0_ADC",
	"BATID_ADC",
	"PMICTEMP_ADC",
	"BATTEMP1_ADC",
	"PMICTEMP1_ADC"
};

static struct delayed_work intel_adc_test_clients;
/**
 * intel_adc_iio_test_client_channels() - Workqueue that schedules measurements
 * over IIO for test purposes
 *
 * @work:	Pointer to workqueue struct.
 */
static void intel_adc_iio_test_client_channels(struct work_struct *work)
{
	enum adc_channel chan;

	for (chan = 0; chan < ARRAY_SIZE(channel_data); chan++) {
		struct iio_channel *p_adc_iio_chan;
		/* Get the ADC channel for this sensor*/
		p_adc_iio_chan = iio_channel_get(NULL,
							channel_data[chan]);
		if (IS_ERR(p_adc_iio_chan)) {
			pr_info("Intel_adc: testing inkernel channels: channel %s does not work\n",
						channel_data[chan]);
		} else {
			int adc_ret, error, adc_voltage_uv, adc_current_na;
			adc_ret = iio_channel_read(p_adc_iio_chan,
					&adc_voltage_uv, &adc_current_na,
					IIO_CHAN_INFO_RAW);

			error = (adc_ret == IIO_VAL_COMPOSITE) ? 0 : adc_ret;
			pr_info("Intel_adc: testing inkernel channels: channel %s->%duV,%dnA,error=%d\n",
					channel_data[chan], adc_voltage_uv,
							adc_current_na, error);

			iio_channel_release(p_adc_iio_chan);
		}
	}
	schedule_delayed_work((struct delayed_work *)work,
				msecs_to_jiffies(1000));
}
#endif

#ifdef INTEL_SENSOR_TEST
static const char * const sensor_channel_data[] = {
	"VBAT_SENSOR",
	"VBAT_OCV_SENSOR",
	"BATTEMP0_SENSOR",
	"BATID_SENSOR",
	"ACCID_SENSOR",
	"PMICTEMP_SENSOR",
	"SYSTEMP0_SENSOR",
	"SYSTEMP1_SENSOR",
};

static struct delayed_work intel_adc_sensors_test_clients;
/**
 * intel_adc_sensors_iio_test_client_channels() - Workqueue that schedules
 * sensors measurements over IIO for test purposes
 *
 * @work:	Pointer to workqueue struct.
 */
static void intel_adc_sensors_iio_test_client_channels(struct work_struct *work)
{
	int chan;

	for (chan = 0; chan < ARRAY_SIZE(sensor_channel_data); chan++) {
		struct iio_channel *p_sensors_iio_chan;
		/* Get the ADC channel for this sensor*/
		p_sensors_iio_chan = iio_channel_get(NULL,
						sensor_channel_data[chan]);
		if (IS_ERR(p_sensors_iio_chan)) {
			pr_info("Intel_adc_sensors: testing inkernel channels: channel %s does not work\n",
						sensor_channel_data[chan]);

		} else {
			int sensors_val;
			int sensors_ret =
				iio_read_channel_processed(p_sensors_iio_chan,
								&sensors_val);
			pr_info("Intel_adc_sensors: testing inkernel channels: channel %s->%d, error=%d\n",
					sensor_channel_data[chan],
						sensors_val, sensors_ret);

			iio_channel_release(p_sensors_iio_chan);
		}
	}

	schedule_delayed_work((struct delayed_work *)work,
				msecs_to_jiffies(1000));
}
#endif

/**
 * intel_adc_test_probe() - The function that starts periodic
 * measurement of IIO ADC.
 *
 * @p_platform_dev:		A pointer to the IDI device.
 */
static int __init intel_adc_test_probe(struct platform_device *p_platform_dev)

{
	/* Test code */

#ifdef INTEL_ADC_TEST
	/* Initialize the delayed work and start test client */
	INIT_DELAYED_WORK(&intel_adc_test_clients,
				intel_adc_iio_test_client_channels);
	schedule_delayed_work(&intel_adc_test_clients, msecs_to_jiffies(1000));
#endif

#ifdef INTEL_SENSOR_TEST
	/* Initialize the delayed work and start sensors test client */
	INIT_DELAYED_WORK(&intel_adc_sensors_test_clients,
				intel_adc_sensors_iio_test_client_channels);
	schedule_delayed_work(&intel_adc_sensors_test_clients,
				msecs_to_jiffies(1000));
#endif

	return 0;
}

/*
 * intel_adc_test_remove - Module remove function.
 */
static int __exit intel_adc_test_remove(struct platform_device *p_platform_dev)
{
	/* Nothing to do */
	return 0;
}

static const struct of_device_id adc_test_of_match[] = {
	{
	 .compatible = "intel,adc_test",
	 },
	{}
};

MODULE_DEVICE_TABLE(of, adc_test_of_match);

static struct platform_driver intel_adc_test_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(adc_test_of_match),
	},
	.probe = intel_adc_test_probe,
	.remove = intel_adc_test_remove,
};

/*
 * intel_adc_init - Module intialisation function.
 */
static int __init intel_adc_test_client_init(void)
{
	return platform_driver_register(&intel_adc_test_driver);
}

/*
 * intel_adc_exit - Module cleanup function.
 */
static void __exit intel_adc_test_client_exit(void)
{
	return platform_driver_unregister(&intel_adc_test_driver);
}

late_initcall(intel_adc_test_client_init);
module_exit(intel_adc_test_client_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ADC test clients for Intel family.");
