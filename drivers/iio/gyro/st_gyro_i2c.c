/*
 * STMicroelectronics gyroscopes driver
 *
 * Copyright 2012-2013 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>

#include <linux/iio/common/st_sensors.h>
#include <linux/iio/common/st_sensors_i2c.h>
#include "st_gyro.h"

static int st_gyro_i2c_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct st_sensor_data *gdata;
	int err;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*gdata));
	if (!indio_dev)
		return -ENOMEM;

	gdata = iio_priv(indio_dev);
	gdata->dev = &client->dev;
	mutex_init(&gdata->mutex);

	st_sensors_i2c_configure(indio_dev, client, gdata);

	err = st_gyro_common_probe(indio_dev,
				(struct st_sensors_platform_data *)&gyro_pdata);
	if (err < 0)
		return err;

	return 0;
}

static int st_gyro_i2c_remove(struct i2c_client *client)
{
	st_gyro_common_remove(i2c_get_clientdata(client));

	return 0;
}
#ifdef CONFIG_PM_SLEEP
static int st_gyro_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct st_sensor_data *adata = iio_priv(indio_dev);
	int ret;
	int enabled;

	mutex_lock(&adata->mutex);
	dev_err(dev, "%s enabled:%d\n", __func__, adata->enabled);
	if (!adata->enabled) {
		ret = 0;
		goto err;
	}

	enabled = adata->enabled;
	ret = st_sensors_set_enable(indio_dev, false);
	adata->enabled = enabled;

	if (indio_dev->trig && indio_dev->trig->ops &&
		indio_dev->trig->ops->set_trigger_state) {
		ret = indio_dev->trig->ops->set_trigger_state(indio_dev->trig,
				false);
		if (ret < 0) {
			dev_err(dev, "failed to stop trigger\n");
			goto err;
		}
	}

	if (adata->pm_platdata && adata->pm_platdata->pm_state_D3_name) {
		ret = device_state_pm_set_state_by_name(adata->dev,
			adata->pm_platdata->pm_state_D3_name);
		if (ret < 0)
			dev_err(dev, "failed to enter D3 mode\n");
	}

err:
	mutex_unlock(&adata->mutex);

	return ret;
}

static int st_gyro_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct st_sensor_data *adata = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&adata->mutex);
	dev_err(dev, "%s enabled:%d\n", __func__, adata->enabled);
	if (!adata->enabled) {
		ret = 0;
		goto err;
	}

	if (adata->pm_platdata && adata->pm_platdata->pm_state_D0_name) {
		ret = device_state_pm_set_state_by_name(adata->dev,
				adata->pm_platdata->pm_state_D0_name);
		if (ret < 0) {
			dev_err(dev, "failed to enter D0 mode\n");
			goto err;
		}
	}

	if (indio_dev->trig && indio_dev->trig->ops &&
		indio_dev->trig->ops->set_trigger_state) {
		ret = indio_dev->trig->ops->set_trigger_state(indio_dev->trig,
				true);
		if (ret < 0) {
			dev_err(dev, "failed to start trigger\n");
			goto err;
		}
	}

	ret = st_sensors_set_enable(indio_dev, true);
err:
	mutex_unlock(&adata->mutex);

	return ret;
}
#endif

#ifdef CONFIG_PM
static int st_gyro_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct st_sensor_data *adata = iio_priv(indio_dev);
	int ret;

	ret = st_sensors_set_enable(indio_dev, false);

	if (adata->pm_platdata && adata->pm_platdata->pm_state_D3_name) {
		ret = device_state_pm_set_state_by_name(adata->dev,
				adata->pm_platdata->pm_state_D3_name);
		if (ret < 0)
			dev_err(dev, "failed to enter D3 mode\n");
	}
	return ret;
}

static int st_gyro_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct st_sensor_data *adata = iio_priv(indio_dev);
	int sleep_val;
	int ret;

	if (adata->pm_platdata && adata->pm_platdata->pm_state_D0_name) {
		ret = device_state_pm_set_state_by_name(adata->dev,
				adata->pm_platdata->pm_state_D0_name);
		if (ret < 0) {
			dev_err(dev, "failed to enter D0 mode\n");
			return ret;
		}
	}
	return st_sensors_set_enable(indio_dev, true);
}
#endif
static const struct dev_pm_ops st_gyro_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_gyro_suspend, st_gyro_resume)
	SET_RUNTIME_PM_OPS(st_gyro_runtime_suspend,
			   st_gyro_runtime_resume, NULL)
};

static const struct i2c_device_id st_gyro_id_table[] = {
	{ L3G4200D_GYRO_DEV_NAME },
	{ LSM330D_GYRO_DEV_NAME },
	{ LSM330DL_GYRO_DEV_NAME },
	{ LSM330DLC_GYRO_DEV_NAME },
	{ L3GD20_GYRO_DEV_NAME },
	{ L3G4IS_GYRO_DEV_NAME },
	{ LSM330_GYRO_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(i2c, st_gyro_id_table);

static struct i2c_driver st_gyro_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "st-gyro-i2c",
		.pm	= &st_gyro_pm_ops,
	},
	.probe = st_gyro_i2c_probe,
	.remove = st_gyro_i2c_remove,
	.id_table = st_gyro_id_table,
};
module_i2c_driver(st_gyro_driver);

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics gyroscopes i2c driver");
MODULE_LICENSE("GPL v2");
