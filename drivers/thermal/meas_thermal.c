/*
 * Copyright (C) 2013 Intel Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/semaphore.h>

#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>

struct sensor_thermal_device {
	struct thermal_zone_device *tzd;
	struct iio_channel *channel;
};

struct meas_thermal {
	int num_sensors;
	struct sensor_thermal_device *sensors;
};

static struct meas_thermal meas_thermal;

static int show_temp(struct thermal_zone_device *tzd, unsigned long *temp)
{
	int result;
	int ret;
	struct sensor_thermal_device *sensor = tzd->devdata;

	ret = iio_read_channel_processed(sensor->channel, &result);

	/* Convert degree Celsius to milli degree Celsius. */
	if (ret == IIO_VAL_INT)
		result *= 1000;

	*temp = result;

	return 0;
}

static struct thermal_zone_device_ops ops = {
	.get_temp = show_temp,
};

static void remove_thermal_devices(void)
{
	int i;
	struct sensor_thermal_device *sensor;

	for (i = 0; i < meas_thermal.num_sensors; i++) {
		sensor = &meas_thermal.sensors[i];
		if (!sensor->channel)
			break;
		thermal_zone_device_unregister(sensor->tzd);
		iio_channel_release(sensor->channel);
	}
}

static int meas_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret, i;
	enum iio_chan_type type;
	struct sensor_thermal_device *sensor;
	int num_sensors = 0;
	const char *channel_name, *thermal_name;

	num_sensors = of_property_count_strings(dev->of_node,
							"intel,sensor-names");
	if (num_sensors < 0) {
		dev_err(dev, "no sensor to monitor\n");
		return -EINVAL;
	}

	meas_thermal.num_sensors = num_sensors;
	meas_thermal.sensors =
		devm_kzalloc(dev,
			sizeof(struct sensor_thermal_device)*num_sensors,
			GFP_KERNEL);
	if (!meas_thermal.sensors)
		return -ENOMEM;

	for (i = 0; i < meas_thermal.num_sensors; i++) {
		sensor = &meas_thermal.sensors[i];
		ret = of_property_read_string_index(dev->of_node,
				"intel,sensor-names", i, &channel_name);
		if (ret)
			goto err_exit;

		ret = of_property_read_string_index(dev->of_node,
				"intel,thermal-names", i, &thermal_name);
		if (ret)
			goto err_exit;

		dev_dbg(dev, "%d %s %s\n", i, thermal_name, channel_name);

		sensor->channel = iio_channel_get(NULL, channel_name);
		if (iio_get_channel_type(sensor->channel, &type) ||
			(type != IIO_TEMP))
			goto err_exit;

		sensor->tzd = thermal_zone_device_register(thermal_name, 0, 0,
						   sensor, &ops, NULL, 0, 0);
		if (IS_ERR(sensor->tzd)) {
			dev_err(dev, "Register thermal zone device failed.\n");
			goto err_exit;
		}
	}
	platform_set_drvdata(pdev, &meas_thermal);

	return 0;

err_exit:
	remove_thermal_devices();
	return ret;
}


static int meas_thermal_remove(struct platform_device *pdev)
{
	remove_thermal_devices();
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id meas_thermal_id_table[] = {
	{ .compatible = "intel,meas_thermal" },
	{}
};
MODULE_DEVICE_TABLE(of, meas_thermal_id_table);

static struct platform_driver meas_thermal_driver = {
	.probe = meas_thermal_probe,
	.remove = meas_thermal_remove,
	.driver = {
		.name = "meas_thermal",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(meas_thermal_id_table),
	},
};

static int meas_thermal_module_init(void)
{
	return platform_driver_register(&meas_thermal_driver);
}

static void meas_thermal_module_exit(void)
{
	platform_driver_unregister(&meas_thermal_driver);
}

late_initcall(meas_thermal_module_init);
module_exit(meas_thermal_module_exit);

MODULE_DESCRIPTION("Intel AG620 MEAS Thermal Driver");
MODULE_LICENSE("GPL");
