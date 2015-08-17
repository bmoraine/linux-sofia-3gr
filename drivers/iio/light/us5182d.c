/*
 * Copyright (c) 2015 Intel Corporation
 *
 * Driver for UPISEMI us5182d Proximity and Ambient Light Sensor.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * To do: Interrupt support.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/of.h>
#include <linux/mutex.h>

#define US5182D_REG_CFG0		0x00
#define US5182D_REG_CFG1		0x01
#define US5182D_REG_CFG2		0x02
#define US5182D_REG_CFG3		0x03
#define US5182D_REG_CFG4		0x10

/*
 * Registers for tuning the auto dark current cancelling feature.
 * DARK_TH(reg 0x27,0x28) - threshold (counts) for auto dark cancelling.
 * when ALS  > DARK_TH --> ALS_Code = ALS - Upper(0x2A) * Dark
 * when ALS < DARK_TH --> ALS_Code = ALS - Lower(0x29) * Dark
 */
#define US5182D_REG_UDARK_TH			0x27
#define US5182D_REG_DARK_AUTO_EN		0x2b
#define US5182D_REG_AUTO_LDARK_TH		0x29
#define US5182D_REG_AUTO_HDARK_TH		0x2a

/*
 * power-mode - oneshot, non-interrupt, word mode access for
 * als code, th registers.
 */
#define US5182D_REG_CFG0_DEFAULT		0x81

/*
 * no als fault queue, 16 bit als resolution (supports 12 and 14 also),
 * max-range 12354.
 */
#define US5182D_REG_CFG1_DEFAULT		0x10

/*
 * 16 bit resolution for PX, x16 sensing amplifier for px
 * supports [x1 x2 x4 x8 x16 x32 x64 x128]
 */
#define US5182D_REG_CFG2_DEFAULT		0x14

/*
 * led current : 100mA, supports [12.5 25 50 100] mA,
 * interrupt source selection "px approach",
 * [als or ps, als only, ps only, ps approach]
 */
#define US5182D_REG_CFG3_DEFAULT		0x3C

/*
 * led frequency ADC Clock/2 from [ADC Clock/2, /4, /8, /10]
 * automatic 50/60 Hz ripple rejection disabled.
 */
#define US5182D_REG_CFG4_DEFAULT		0x00

#define US5182D_REG_DARK_AUTO_EN_DEFAULT	0x80
#define US5182D_REG_AUTO_LDARK_TH_DEFAULT	0x16
#define US5182D_REG_AUTO_HDARK_TH_DEFAULT	0x00

#define US5182D_REG_ADL			0x0c
#define US5182D_REG_PDL			0x0e

#define US5182D_REG_MODE_STORE		0x21
#define US5182D_STORE_MODE		0x01

#define US5182D_REG_CHIPID		0xb2

#define US5182D_ONESHOT_EN_MASK		BIT(6)
#define US5182D_SHUTDOWN_EN_MASK	BIT(7)
#define US5182D_OPMODE_MASK		GENMASK(5, 4)
#define US5182D_AGAIN_MASK		0x07
#define US5182D_OPMODE_ALS		0x01
#define US5182D_OPMODE_PX		0x02
#define US5182D_OPMODE_SHIFT		4
#define US5182D_RESET_CHIP		0x01

#define US5182D_CHIPID			0x26
#define US5182D_DRV_NAME		"us5182d"

#define US5182D_GA_RESOLUTION		1000

#define READ_BYTE		1
#define READ_WORD		2
#define OPSTORE_SLEEP_TIME	20 /* ms */

/* available ranges: [12354, 7065, 3998, 2202, 1285, 498, 256, 138] lux*/
static const int us5182d_scales[] = {188500, 107800, 61000, 33600, 19600, 7600,
				     3900, 2100};

/*
 * experimental thresholds that work with US5182D sensor on evaluation board
 * roughly 12 - 32 lux
 */
static u16 us5182d_dark_ths_vals[] = {170, 200, 512, 512, 800, 2000, 4000,
				      8000};

enum mode {
	ALS_PX,
	ALS_ONLY,
	PX_ONLY
};

struct us5182d_data {
	struct i2c_client *client;
	/* protect opmode */
	struct mutex lock;

	/* Glass attenuation factor */
	u32 ga;

	/* Dark gain tuning */
	u16 lower_dark_th;
	u16 upper_dark_th;
	u16 *us5182d_dark_ths;

	u8 opmode;
};

static IIO_CONST_ATTR(in_illuminance_scale_available,
		      "0.0021 0.0039 0.0076 0.0196 0.0336 0.061 0.1078 0.1885");

static struct attribute *us5182d_attrs[] = {
	&iio_const_attr_in_illuminance_scale_available.dev_attr.attr,
	NULL
};

static const struct attribute_group us5182d_attr_group = {
	.attrs = us5182d_attrs,
};

static const struct {
	u8 reg;
	u8 val;
} us5182d_regvals[] = {
	{US5182D_REG_CFG0, US5182D_REG_CFG0_DEFAULT},
	{US5182D_REG_CFG1, US5182D_REG_CFG1_DEFAULT},
	{US5182D_REG_CFG2, US5182D_REG_CFG2_DEFAULT},
	{US5182D_REG_CFG3, US5182D_REG_CFG3_DEFAULT},
	{US5182D_REG_MODE_STORE, US5182D_STORE_MODE},
	{US5182D_REG_CFG4, US5182D_REG_CFG4_DEFAULT},
};

static const struct iio_chan_spec us5182d_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}
};

static int us5182d_get_als(struct us5182d_data *data)
{
	int ret;
	unsigned long result;

	ret = i2c_smbus_read_word_data(data->client,
				     US5182D_REG_ADL);
	if (ret < 0)
		return ret;

	result = ret * data->ga / US5182D_GA_RESOLUTION;
	if (result > 0xffff)
		result = 0xffff;

	return result;
}

static int us5182d_set_opmode(struct us5182d_data *data, u8 mode)
{
	int ret;

	ret = i2c_smbus_read_byte_data(data->client, US5182D_REG_CFG0);
	if (ret < 0)
		return ret;

	ret = ret | US5182D_ONESHOT_EN_MASK;

	/* update mode */
	ret = ret & ~US5182D_OPMODE_MASK;
	ret = ret | (mode << US5182D_OPMODE_SHIFT);

	ret = i2c_smbus_write_byte_data(data->client, US5182D_REG_CFG0, ret);
	if (ret < 0)
		return ret;

	if (mode == data->opmode)
		return 0;

	data->opmode = mode;
	ret = i2c_smbus_write_byte_data(data->client, US5182D_REG_MODE_STORE,
					US5182D_STORE_MODE);
	if (ret < 0)
		return ret;
	msleep(OPSTORE_SLEEP_TIME);

	return 0;
}

static int us5182d_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	struct us5182d_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_LIGHT:
			mutex_lock(&data->lock);
			ret = us5182d_set_opmode(data, US5182D_OPMODE_ALS);
			if (ret < 0)
				goto out_err;

			ret = us5182d_get_als(data);
			if (ret < 0)
				goto out_err;
			mutex_unlock(&data->lock);
			*val = ret;
			*val2 = 0;
			return IIO_VAL_INT;
		case IIO_PROXIMITY:
			mutex_lock(&data->lock);
			ret = us5182d_set_opmode(data, US5182D_OPMODE_PX);
			if (ret < 0)
				goto out_err;

			ret = i2c_smbus_read_word_data(data->client,
						     US5182D_REG_PDL);
			if (ret < 0)
				goto out_err;
			mutex_unlock(&data->lock);
			*val = ret;
			*val2 = 0;
			return  IIO_VAL_INT;
		default:
			break;
		}

	case IIO_CHAN_INFO_SCALE:
		ret = i2c_smbus_read_byte_data(data->client, US5182D_REG_CFG1);
		if (ret < 0)
			return ret;
		*val = 0;
		ret = (ret & US5182D_AGAIN_MASK);
		*val2 = us5182d_scales[ret];
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return -EINVAL;
out_err:
	mutex_unlock(&data->lock);
	return ret;
}

static int us5182d_update_dark_th(struct us5182d_data *data, int index)
{
	__be16 dark_th = cpu_to_be16(data->us5182d_dark_ths[index]);
	u8 *bytes = (u8 *)&dark_th;
	int ret;

	/* Registers Dark_Th (0x27 0x28) don't work in word mode accessing */
	ret = i2c_smbus_write_byte_data(data->client, US5182D_REG_UDARK_TH,
					bytes[0]);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(data->client, US5182D_REG_UDARK_TH + 1,
					bytes[1]);
}

static int us5182d_apply_scale(struct us5182d_data *data, int index)
{
	int ret;

	ret = i2c_smbus_read_byte_data(data->client, US5182D_REG_CFG1);
	if (ret < 0)
		return ret;

	ret = ret & (~US5182D_AGAIN_MASK);
	ret |= index;

	ret = i2c_smbus_write_byte_data(data->client, US5182D_REG_CFG1, ret);
	if (ret < 0)
		return ret;

	return us5182d_update_dark_th(data, index);
}

static int us5182d_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct us5182d_data *data = iio_priv(indio_dev);
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (val != 0)
			return -EINVAL;
		for (i = 0; i < ARRAY_SIZE(us5182d_scales); i++)
			if (val2 == us5182d_scales[i])
				return us5182d_apply_scale(data, i);
		break;
	default:
		break;
	}

	return -EINVAL;
}

static const struct iio_info us5182d_info = {
	.driver_module	= THIS_MODULE,
	.read_raw = us5182d_read_raw,
	.write_raw = us5182d_write_raw,
	.attrs = &us5182d_attr_group,
};

static int us5182d_reset(struct iio_dev *indio_dev)
{
	struct us5182d_data *data = iio_priv(indio_dev);
	int ret;

	ret = i2c_smbus_write_byte_data(data->client, US5182D_REG_CFG3,
					US5182D_RESET_CHIP);
	if (ret < 0)
		return ret;

	return 0;
}

static int us5182d_init(struct iio_dev *indio_dev)
{
	struct us5182d_data *data = iio_priv(indio_dev);
	int i, ret;

	ret = us5182d_reset(indio_dev);
	if (ret < 0)
		return ret;

	data->opmode = 0;
	for (i = 0; i < ARRAY_SIZE(us5182d_regvals); i++) {
		ret = i2c_smbus_write_byte_data(data->client,
						us5182d_regvals[i].reg,
						us5182d_regvals[i].val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static void us5182d_get_platform_data(struct iio_dev *indio_dev)
{
	struct us5182d_data *data = iio_priv(indio_dev);
	struct device_node *node = data->client->dev.of_node;

	if (!node) {
		data->ga = US5182D_GA_RESOLUTION;
		data->us5182d_dark_ths = us5182d_dark_ths_vals;
		data->upper_dark_th = US5182D_REG_AUTO_HDARK_TH_DEFAULT;
		data->lower_dark_th = US5182D_REG_AUTO_LDARK_TH_DEFAULT;
		return;
	}

	if (of_property_read_u32(node, "upisemi,glass-coef", &data->ga))
		data->ga = US5182D_GA_RESOLUTION;
	if (of_property_read_u16_array(node, "upisemi,dark-ths",
				       data->us5182d_dark_ths,
				       ARRAY_SIZE(us5182d_dark_ths_vals)))
		data->us5182d_dark_ths = us5182d_dark_ths_vals;
	if (of_property_read_u16(node, "upisemi,upper-dark-gain",
				 &data->upper_dark_th))
		data->upper_dark_th = US5182D_REG_AUTO_HDARK_TH_DEFAULT;
	if (of_property_read_u16(node, "upisemi,lower-dark-gain",
				 &data->lower_dark_th))
		data->lower_dark_th = US5182D_REG_AUTO_LDARK_TH_DEFAULT;
}

static int  us5182d_dark_gain_config(struct iio_dev *indio_dev)
{
	struct us5182d_data *data = iio_priv(indio_dev);
	u8 index = US5182D_REG_CFG1_DEFAULT & US5182D_AGAIN_MASK;
	int ret;

	ret = us5182d_update_dark_th(data, index);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(data->client, US5182D_REG_AUTO_LDARK_TH,
					data->lower_dark_th);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(data->client, US5182D_REG_AUTO_HDARK_TH,
					data->upper_dark_th);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(data->client, US5182D_REG_DARK_AUTO_EN,
					US5182D_REG_DARK_AUTO_EN_DEFAULT);
}

static int us5182d_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct us5182d_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	mutex_init(&data->lock);

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &us5182d_info;
	indio_dev->name = US5182D_DRV_NAME;
	indio_dev->channels = us5182d_channels;
	indio_dev->num_channels = ARRAY_SIZE(us5182d_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = i2c_smbus_read_byte_data(data->client, US5182D_REG_CHIPID);
	if (ret != US5182D_CHIPID)
		dev_err(&data->client->dev,
			"Failed to detect US5182 light chip\n");

	us5182d_get_platform_data(indio_dev);
	ret = us5182d_init(indio_dev);
	if (ret < 0)
		return ret;

	ret = us5182d_dark_gain_config(indio_dev);
	if (ret < 0)
		return ret;

	return iio_device_register(indio_dev);
}

static int us5182d_remove(struct i2c_client *client)
{
	iio_device_unregister(i2c_get_clientdata(client));
	return i2c_smbus_write_byte_data(client, US5182D_REG_CFG0,
					 US5182D_REG_CFG0_DEFAULT);
}

static const struct acpi_device_id us5182d_acpi_match[] = {
	{ "USD5182", 0},
	{}
};

MODULE_DEVICE_TABLE(acpi, us5182d_acpi_match);

static const struct i2c_device_id us5182d_id[] = {
		{"usd5182", 0},
		{}
};

MODULE_DEVICE_TABLE(i2c, us5182d_id);

static struct i2c_driver us5182d_driver = {
	.driver = {
		.name = US5182D_DRV_NAME,
		.acpi_match_table = ACPI_PTR(us5182d_acpi_match),
	},
	.probe = us5182d_probe,
	.remove = us5182d_remove,
	.id_table = us5182d_id,

};
module_i2c_driver(us5182d_driver);

MODULE_AUTHOR("Adriana Reus <adriana.reus@intel.com>");
MODULE_DESCRIPTION("Driver for us5182d Proximity and Light Sensor");
MODULE_LICENSE("GPL v2");
