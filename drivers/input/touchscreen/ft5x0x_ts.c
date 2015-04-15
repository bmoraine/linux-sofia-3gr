/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 */

#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/i2c/ft5x06_ts.h>

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#define CONFIG_FT5X0X_MULTITOUCH 1

struct ts_event {
	u16 x1;
	u16 y1;
	u16 x2;
	u16 y2;
	u16 x3;
	u16 y3;
	u16 x4;
	u16 y4;
	u16 x5;
	u16 y5;
	u16 pressure;
	s16 touch_ID1;
	s16 touch_ID2;
	s16 touch_ID3;
	s16 touch_ID4;
	s16 touch_ID5;
	u8 touch_point;
};

struct ft5x0x_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct delayed_work pen_event_work;
	struct workqueue_struct *ts_workqueue;
	spinlock_t btn_lock;
	spinlock_t en_lock;
	int btn_active;
	int enable;
};

static int const android_key[KEY_INDEX_MAX] = {
	KEY_HOMEPAGE,
	KEY_BACK,
	KEY_MENU,
	KEY_SEARCH,
};

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
struct device_state_pm_state ft5x0x_pm_states[] = {
	{.name = "disable", }, /* D3 */
	{.name = "enable", }, /* D0 */
};

#define FT5X0X_STATE_D0		1
#define FT5X0X_STATE_D3		0

/* Touchscreen PM states & class */
static int ft5x0x_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	struct ft5x0x_ts_platform_data *ft5x06_pdata = dev->platform_data;
	int id = device_state_pm_get_state_id(dev, state->name);
	int ret = 0;

	switch (id) {
	case FT5X0X_STATE_D0:
		if (ft5x06_pdata->power)
			ret = regulator_enable(ft5x06_pdata->power);
		if (ret)
			return ret;
		mdelay(50);

		if (ft5x06_pdata->power2)
			ret = regulator_enable(ft5x06_pdata->power2);
		if (ret)
			return ret;
		mdelay(50);

		break;

	case FT5X0X_STATE_D3:
		if (ft5x06_pdata->power)
			regulator_disable(ft5x06_pdata->power);

		if (ft5x06_pdata->power2)
			regulator_disable(ft5x06_pdata->power2);

		break;

	default:
		return id;
	}

	return 0;
}

static struct device_state_pm_state *ft5x0x_get_initial_state(
		struct device *dev)
{
	return &ft5x0x_pm_states[FT5X0X_STATE_D3];
}

static struct device_state_pm_ops ft5x0x_pm_ops = {
	.set_state = ft5x0x_set_pm_state,
	.get_initial_state = ft5x0x_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(ft5x0x);
#endif

#ifdef CONFIG_OF

#define OF_FT5X0X_SUPPLY	"i2c"
#define OF_FT5X0X_SUPPLY_A	"i2ca"
#define OF_FT5X0X_PIN_RESET	"intel,ts-gpio-reset"

static struct ft5x0x_ts_platform_data *ft5x0x_ts_of_get_platdata(
		struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct ft5x0x_ts_platform_data *ft5x06_pdata;
	struct regulator *pow_reg;
	int ret;

	ft5x06_pdata = devm_kzalloc(&client->dev,
			sizeof(*ft5x06_pdata), GFP_KERNEL);
	if (!ft5x06_pdata)
		return ERR_PTR(-ENOMEM);

	/* regulator */
	pow_reg = regulator_get(&client->dev, OF_FT5X0X_SUPPLY);
	ft5x06_pdata->power = pow_reg;
	if (IS_ERR(pow_reg)) {
		dev_warn(&client->dev, "%s can't get %s-supply handle\n",
			np->name, OF_FT5X0X_SUPPLY);
		ft5x06_pdata->power = NULL;
	}

	pow_reg = regulator_get(&client->dev, OF_FT5X0X_SUPPLY_A);
	ft5x06_pdata->power2 = pow_reg;
	if (IS_ERR(pow_reg)) {
		dev_warn(&client->dev, "%s can't get %s-supply handle\n",
			np->name, OF_FT5X0X_SUPPLY_A);
		ft5x06_pdata->power2 = NULL;
	}

	/* pinctrl */
	ft5x06_pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(ft5x06_pdata->pinctrl)) {
		ret = PTR_ERR(ft5x06_pdata->pinctrl);
		goto out;
	}

	ft5x06_pdata->pins_default = pinctrl_lookup_state(
			ft5x06_pdata->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(ft5x06_pdata->pins_default))
		dev_err(&client->dev, "could not get default pinstate\n");

	ft5x06_pdata->pins_sleep = pinctrl_lookup_state(
			ft5x06_pdata->pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR(ft5x06_pdata->pins_sleep))
		dev_err(&client->dev, "could not get sleep pinstate\n");

	ft5x06_pdata->pins_inactive = pinctrl_lookup_state(
			ft5x06_pdata->pinctrl, "inactive");
	if (IS_ERR(ft5x06_pdata->pins_inactive))
		dev_err(&client->dev, "could not get inactive pinstate\n");

	/* gpio reset */
	ft5x06_pdata->reset_pin = of_get_named_gpio_flags(client->dev.of_node,
			OF_FT5X0X_PIN_RESET, 0, NULL);
	if (ft5x06_pdata->reset_pin <= 0) {
		dev_err(&client->dev,
			"error getting gpio for %s\n", OF_FT5X0X_PIN_RESET);
		ret = -EINVAL;
		goto out;
	}

	/* interrupt mode */
	if (of_property_read_bool(np, "intel,polling-mode"))
		ft5x06_pdata->polling_mode = true;

	/* pm */
	ft5x06_pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(ft5x06_pdata->pm_platdata)) {
		dev_warn(&client->dev, "Error during device state pm init\n");
		ret = PTR_ERR(ft5x06_pdata->pm_platdata);
		goto out;
	}

	of_property_read_u32(np, "intel,x_pos_max",
			&ft5x06_pdata->x_pos_max);
	of_property_read_u32(np, "intel,x_pos_min",
			&ft5x06_pdata->x_pos_min);
	of_property_read_u32(np, "intel,y_pos_max",
			&ft5x06_pdata->y_pos_max);
	of_property_read_u32(np, "intel,y_pos_min",
			&ft5x06_pdata->y_pos_min);
	of_property_read_u32(np, "intel,screen_max_x",
			&ft5x06_pdata->screen_max_x);
	of_property_read_u32(np, "intel,screen_max_y",
			&ft5x06_pdata->screen_max_y);
	of_property_read_u32(np, "intel,key_y",
			&ft5x06_pdata->key_y);

	ret = of_property_read_u32(np, "intel,key_home",
			&ft5x06_pdata->key_x[KEY_HOME_INDEX]);
	if (!ret)
		dev_info(&client->dev, "home key x:%d\n",
				ft5x06_pdata->key_x[KEY_HOME_INDEX]);

	ret = of_property_read_u32(np, "intel,key_back",
			&ft5x06_pdata->key_x[KEY_BACK_INDEX]);
	if (!ret)
		dev_info(&client->dev, "back key x:%d\n",
				ft5x06_pdata->key_x[KEY_BACK_INDEX]);

	ret = of_property_read_u32(np, "intel,key_menu",
			&ft5x06_pdata->key_x[KEY_MENU_INDEX]);
	if (!ret)
		dev_info(&client->dev, "menu key x:%d\n",
				ft5x06_pdata->key_x[KEY_MENU_INDEX]);

	ret = of_property_read_u32(np, "intel,key_search",
			&ft5x06_pdata->key_x[KEY_SEARCH_INDEX]);
	if (!ret)
		dev_info(&client->dev, "search key x:%d\n",
				ft5x06_pdata->key_x[KEY_SEARCH_INDEX]);

	return ft5x06_pdata;

out:
	return ERR_PTR(ret);
}
#endif

static inline int ft5x0x_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	struct ft5x0x_ts_platform_data *pdata = dev_get_platdata(dev);
	int ret = 0;

	if (!IS_ERR(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}

	return ret;
}

static int ft5x0x_ts_power_off(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	struct ft5x0x_ts_platform_data *ft5x06_pdata =
		client->dev.platform_data;
	unsigned long flags = 0;
	int ret = 0;

	spin_lock_irqsave(&ft5x0x_ts->en_lock, flags);
	if (!ft5x0x_ts->enable)
		goto out;

	disable_irq_nosync(client->irq);
	cancel_delayed_work_sync(&ft5x0x_ts->pen_event_work);
	if (ft5x06_pdata->pins_sleep)
		ft5x0x_set_pinctrl_state(&client->dev,
				ft5x06_pdata->pins_sleep);

	if (ft5x06_pdata->pm_platdata &&
			ft5x06_pdata->pm_platdata->pm_state_D3_name) {
		ret = device_state_pm_set_state_by_name(&client->dev,
				ft5x06_pdata->pm_platdata->pm_state_D3_name);
	}

	if (!ret)
		ft5x0x_ts->enable = 0;

out:
	spin_unlock_irqrestore(&ft5x0x_ts->en_lock, flags);

	return ret;
}

static int ft5x0x_ts_power_on(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	struct ft5x0x_ts_platform_data *ft5x06_pdata =
		client->dev.platform_data;
	unsigned long flags = 0;
	int ret = 0;

	spin_lock_irqsave(&ft5x0x_ts->en_lock, flags);
	if (ft5x0x_ts->enable)
		goto out;

	if (ft5x06_pdata->pins_default)
		ret = ft5x0x_set_pinctrl_state(&client->dev,
					ft5x06_pdata->pins_default);

	if (ft5x06_pdata->pm_platdata &&
			ft5x06_pdata->pm_platdata->pm_state_D0_name)
		ret = device_state_pm_set_state_by_name(&client->dev,
			ft5x06_pdata->pm_platdata->pm_state_D0_name);

	if (!ret) {
		enable_irq(client->irq);
		ft5x0x_ts->enable++;
	}

out:
	spin_unlock_irqrestore(&ft5x0x_ts->en_lock, flags);

	return ret;
}

/*
 * SysFS support
 */

static ssize_t ft5x0x_ts_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);

	dev_dbg(dev, "%s\n", __func__);

	return sprintf(buf, "%d\n", ft5x0x_ts->enable);
}

static ssize_t ft5x0x_ts_store_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (ft5x0x_ts->enable == val)
		return count;

	switch (val) {
	case 0:
		ft5x0x_ts_power_off(client);
		break;
	case 1:
		ft5x0x_ts_power_on(client);
		break;
	default:
		dev_err(&client->dev, "%s: unsuppored value %ld\n",
				__func__, val);
		return count;
	}

	dev_info(dev, "%s\n", (val == 1) ? "enable" : "disable");

	return count;
}

static DEVICE_ATTR(enable, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		ft5x0x_ts_show_enable,
		ft5x0x_ts_store_enable);

static struct attribute *ft5x0x_ts_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};

static const struct attribute_group ft5x0x_ts_attr_group = {
	.attrs = ft5x0x_ts_attributes,
};

static int ft5x0x_i2c_rxdata(struct i2c_client *client, char *buf, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = buf,
		 },
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_set_u8(const struct i2c_client *client, u8 addr, u8 value)
{
	int ret = i2c_smbus_write_byte_data(client, addr, value);
	if (ret < 0)
		pr_err("%s: 0x%08x failed\n", __func__, value);
	else
		pr_debug("sent byte [0x%02x] @0x%x\n", value, addr);
	return ret;
}

static u16 ft5x0x_read_u16(const struct i2c_client *client, u8 addr)
{
	u16 value = i2c_smbus_read_word_data(client, addr);
	if (value < 0) {
		pr_err("%s: read @0x%x failed\n", __func__, addr);
	} else {
		value = swab16(value);
		pr_debug("[0x%X] = %04X\n", addr, value);
	}
	return value;
}

static u8 ft5x0x_read_u8(const struct i2c_client *client, u8 addr)
{
	u8 value = i2c_smbus_read_byte_data(client, addr);
	if (value < 0) {
		pr_err("%s: read @0x%x failed\n", __func__, addr);
		return value;
	}
	pr_debug("[0x%X] = %02X\n", addr, value);
	return value;
}

static void ft5x0x_ts_release(struct i2c_client *client)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(client);

	pr_debug("ft5x0x_ts_release\n");
#ifdef CONFIG_FT5X0X_MULTITOUCH
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 0);
#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
	input_sync(data->input_dev);
}

static void ft5x0x_ts_inactivate(struct i2c_client *client)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(client);
	struct ft5x0x_ts_platform_data *ft5x0x_pdata =
			client->dev.platform_data;
	unsigned long flags = 0;
	int i;

	dev_dbg(&client->dev, "%s\n", __func__);
	spin_lock_irqsave(&data->btn_lock, flags);
	if (data->btn_active) {
		for (i = 0; i < KEY_INDEX_MAX; i++) {
			if (ft5x0x_pdata->key_x[i])
				input_event(data->input_dev, EV_KEY,
						android_key[i], 0);
		}
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		data->btn_active = 0;
	}
	spin_unlock_irqrestore(&data->btn_lock, flags);
	input_mt_sync(data->input_dev);
	input_sync(data->input_dev);
}

static u16 ft5x0x_convert_y(struct i2c_client *client, u16 value)
{
	struct ft5x0x_ts_platform_data *ft5x0x_pdata =
			client->dev.platform_data;

	return (value - ft5x0x_pdata->y_pos_min) * ft5x0x_pdata->screen_max_y /
		(ft5x0x_pdata->y_pos_max - ft5x0x_pdata->y_pos_min);
}

static u16 ft5x0x_convert_x(struct i2c_client *client, u16 value)
{
	struct ft5x0x_ts_platform_data *ft5x0x_pdata =
			client->dev.platform_data;

	return (value - ft5x0x_pdata->x_pos_min) * ft5x0x_pdata->screen_max_x /
		(ft5x0x_pdata->x_pos_max - ft5x0x_pdata->x_pos_min);
}

static bool ft5x0x_check_position(struct i2c_client *client,
		u16 x_val, u16 y_val)
{
	struct ft5x0x_ts_platform_data *ft5x0x_pdata =
			client->dev.platform_data;

	if (x_val < ft5x0x_pdata->x_pos_min || x_val > ft5x0x_pdata->x_pos_max)
		return false;
	if (y_val < ft5x0x_pdata->y_pos_min || y_val > ft5x0x_pdata->y_pos_max)
		return false;

	return true;
}

static int ft5x0x_is_button(struct i2c_client *client, u16 x, u16 y)
{
	int i, key_x;
	struct ft5x0x_ts_platform_data *ft5x0x_pdata =
			client->dev.platform_data;

	if (y != ft5x0x_pdata->key_y)
		return 0;

	for (i = 0; i < KEY_INDEX_MAX; i++) {
		key_x = ft5x0x_pdata->key_x[i];
		if ((key_x != 0) && (x == key_x))
			return android_key[i];
	}

	return 0;
}

static int ft5x0x_read_data(struct i2c_client *client)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(client);
	struct ts_event *event = &data->event;
	u8 buf[32] = { 0 };
	int ret = -1;
	int status = 0;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	ret = ft5x0x_i2c_rxdata(client, buf, 31);
#else
	ret = ft5x0x_i2c_rxdata(client, buf, 7);
#endif
	if (ret < 0) {
		dev_err(&client->dev,
			"%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

	/* at least one touch point is detected.
	   Otherwise, in case of release, get position of latest active touch */
	event->x1 = (s16) (buf[3] & 0x0F) << 8 | (s16) buf[4];
	event->y1 = (s16) (buf[5] & 0x0F) << 8 | (s16) buf[6];

	if (event->touch_point == 0) {
		pr_debug("touch_point == 0 now inactivate\n");
		ft5x0x_ts_inactivate(client);
		return 1;
	}
#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch (event->touch_point) {
	case 5:
		event->x5 = (s16) (buf[0x1b] & 0x0F) << 8 | (s16) buf[0x1c];
		event->y5 = (s16) (buf[0x1d] & 0x0F) << 8 | (s16) buf[0x1e];
		status = (s16) ((buf[0x1b] & 0xc0) >> 6);
		event->touch_ID5 = (s16) (buf[0x1D] & 0xF0) >> 4;
		if (status == 1)
			ft5x0x_ts_release(client);
		/* fallthrough */
	case 4:
		event->x4 = (s16) (buf[0x15] & 0x0F) << 8 | (s16) buf[0x16];
		event->y4 = (s16) (buf[0x17] & 0x0F) << 8 | (s16) buf[0x18];
		status = (s16) ((buf[0x15] & 0xc0) >> 6);
		event->touch_ID4 = (s16) (buf[0x17] & 0xF0) >> 4;
		if (status == 1)
			ft5x0x_ts_release(client);
		/* fallthrough */
	case 3:
		event->x3 = (s16) (buf[0x0f] & 0x0F) << 8 | (s16) buf[0x10];
		event->y3 = (s16) (buf[0x11] & 0x0F) << 8 | (s16) buf[0x12];
		status = (s16) ((buf[0x0f] & 0xc0) >> 6);
		event->touch_ID3 = (s16) (buf[0x11] & 0xF0) >> 4;
		if (status == 1)
			ft5x0x_ts_release(client);
		/* fallthrough */
	case 2:
		event->x2 = (s16) (buf[9] & 0x0F) << 8 | (s16) buf[10];
		event->y2 = (s16) (buf[11] & 0x0F) << 8 | (s16) buf[12];
		status = (s16) ((buf[0x9] & 0xc0) >> 6);
		event->touch_ID2 = (s16) (buf[0x0b] & 0xF0) >> 4;
		if (status == 1)
			ft5x0x_ts_release(client);
		/* fallthrough */
	case 1:
		event->x1 = (s16) (buf[3] & 0x0F) << 8 | (s16) buf[4];
		event->y1 = (s16) (buf[5] & 0x0F) << 8 | (s16) buf[6];
		status = (s16) ((buf[0x3] & 0xc0) >> 6);
		event->touch_ID1 = (s16) (buf[0x05] & 0xF0) >> 4;
		if (status == 1)
			ft5x0x_ts_release(client);
		break;
	default:
		pr_err("Unexpected touch point\n");
		return -1;
	}
#else
	if (event->touch_point == 1) {
		event->x1 = (s16) (buf[3] & 0x0F) << 8 | (s16) buf[4];
		event->y1 = (s16) (buf[5] & 0x0F) << 8 | (s16) buf[6];
	}
#endif
	event->pressure = 200;

	dev_dbg(&client->dev, "%s: 1:%d %d 2:%d %d\n", __func__,
		event->x1, event->y1, event->x2, event->y2);

	return 0;
}

static void ft5x0x_report_value(struct i2c_client *client)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(client);
	struct ts_event *event = &data->event;
#ifdef CONFIG_FT5X0X_MULTITOUCH
	unsigned long flags;
	int nbreport = 0;
#endif
	int key_value = 0;

	dev_dbg(&client->dev, "%s\n", __func__);
#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch (event->touch_point) {
	case 5:
		if (ft5x0x_check_position(client, event->x5, event->y5)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->touch_ID5);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 ft5x0x_convert_x(client, event->x5));
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 ft5x0x_convert_y(client, event->y5));
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					 1);
			input_mt_sync(data->input_dev);
			dev_dbg(&client->dev, "=== x5 = %d, y5 = %d ===\n",
					event->x5, event->y5);
			nbreport++;
		}
		/* fallthrough */
	case 4:
		if (ft5x0x_check_position(client, event->x4, event->y4)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->touch_ID4);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 ft5x0x_convert_x(client, event->x4));
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 ft5x0x_convert_y(client, event->y4));
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					 1);
			input_mt_sync(data->input_dev);
			dev_dbg(&client->dev, "=== x4 = %d, y4 = %d ===\n",
					event->x4, event->y4);
			nbreport++;
		}
		/* fallthrough */
	case 3:
		if (ft5x0x_check_position(client, event->x3, event->y3)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->touch_ID3);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 ft5x0x_convert_x(client, event->x3));
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 ft5x0x_convert_y(client, event->y3));
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					 1);
			input_mt_sync(data->input_dev);
			dev_dbg(&client->dev, "=== x3 = %d, y3 = %d ===\n",
					event->x3, event->y3);
			nbreport++;
		}
		/* fallthrough */
	case 2:
		key_value = ft5x0x_is_button(client, event->x2, event->y2);
		if (key_value) {
			spin_lock_irqsave(&data->btn_lock, flags);
			input_event(data->input_dev, EV_KEY, key_value,
					1);
			input_report_key(data->input_dev, BTN_TOUCH, 1);
			data->btn_active++;
			nbreport++;
			spin_unlock_irqrestore(&data->btn_lock, flags);
			dev_dbg(&client->dev, "*** x2 = %d, y2 = %d ***\n",
					event->x2, event->y2);
		} else if (ft5x0x_check_position(client, event->x2,
				event->y2)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->touch_ID2);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 ft5x0x_convert_x(client, event->x2));
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 ft5x0x_convert_y(client, event->y2));
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					 1);
			input_mt_sync(data->input_dev);
			dev_dbg(&client->dev, "=== x2 = %d, y2 = %d ===\n",
					event->x2, event->y2);
			nbreport++;
		}
		/* fallthrough */
	case 1:
		key_value = ft5x0x_is_button(client, event->x1, event->y1);
		if (key_value) {
			spin_lock_irqsave(&data->btn_lock, flags);
			input_event(data->input_dev, EV_KEY, key_value,
					1);
			input_report_key(data->input_dev, BTN_TOUCH, 1);
			data->btn_active++;
			nbreport++;
			spin_unlock_irqrestore(&data->btn_lock, flags);
			dev_dbg(&client->dev, "*** x1 = %d, y1 = %d ***\n",
					event->x1, event->y1);
		} else if (ft5x0x_check_position(client,
					event->x1, event->y1)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->touch_ID1);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 ft5x0x_convert_x(client, event->x1));
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 ft5x0x_convert_y(client, event->y1));
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					 1);
			input_mt_sync(data->input_dev);
			dev_dbg(&client->dev, "=== x1 = %d, y1 = %d ===\n",
					event->x1, event->y1);
			nbreport++;
		}
		/* fallthrough */
	default:
		dev_dbg(&client->dev, "touch_point default\n");
		break;
	}

	if (nbreport)
		input_sync(data->input_dev);

#else /* CONFIG_FT5X0X_MULTITOUCH */
	if (event->touch_point == 1) {
		input_report_key(data->input_dev, BTN_TOUCH, 1);
		input_report_abs(data->input_dev, ABS_X, /*SCREEN_MAX_X - */
				 event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE,
				 !!event->pressure);
		input_sync(data->input_dev);
		dev_dbg(&client->dev, "imc-ts: x:%d y:%d p:%d\n",
			event->x1, event->y1, event->pressure);
	}
	input_sync(data->input_dev);
#endif /* CONFIG_FT5X0X_MULTITOUCH */

	dev_dbg(&client->dev,
		"1:(%d, %d) 2:(%d, %d) 3:(%d, %d) 4:(%d, %d) 5:(%d, %d)\n",
		event->x1, event->y1, event->x2, event->y2, event->x3,
		event->y3, event->x4, event->y4, event->x5, event->y5);
} /* end ft5x0x_report_value */

static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct ft5x0x_ts_platform_data *ft5x0x_pdata;
	struct i2c_client *client;
	int ret;

	ft5x0x_ts = container_of(to_delayed_work(work),
			struct ft5x0x_ts_data, pen_event_work);
	client = ft5x0x_ts->client;
	ft5x0x_pdata = client->dev.platform_data;

	dev_dbg(&client->dev, "Enter %s\n", __func__);
	ret = ft5x0x_read_data(client);
	if (ret == 0)
		ft5x0x_report_value(client);
	else if (ret < 0)
		dev_err(&client->dev, "data package read error %d\n", ret);

	if (ft5x0x_pdata->polling_mode == true &&
			ft5x0x_ts->event.touch_point > 0)
		schedule_delayed_work(&ft5x0x_ts->pen_event_work,
				msecs_to_jiffies(12));
}

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;

	pr_debug("Enter %s\n", __func__);
	schedule_delayed_work(&ft5x0x_ts->pen_event_work, 0);

	return IRQ_HANDLED;
}

static int ft5x06_ts_reset(struct i2c_client *client, bool show)
{
	struct ft5x0x_ts_platform_data *ft5x06_pdata =
		client->dev.platform_data;
	u8 mode;

	mdelay(100);
	gpio_set_value(ft5x06_pdata->reset_pin, 0);
	mdelay(50);
	gpio_set_value(ft5x06_pdata->reset_pin, 1);
	mdelay(100);

	ft5x0x_set_u8(client, FT5X0X_REG_DEVICE_MODE, 0);
	ft5x0x_set_u8(client, FT5X0X_REG_PMODE, 0x00);
	ft5x0x_set_u8(client, FT5X0X_REG_THGROUP, 0x10);
	ft5x0x_set_u8(client, FT5X0X_REG_PERIODACTIVE, 14);

	/* set interrupt mode */
	mode = (ft5x06_pdata->polling_mode == true) ? 0x0 : 0x1;
	ft5x0x_set_u8(client, FT5X0X_REG_MODE, mode);

	if (!show)
		goto out;

	dev_info(&client->dev, "LIB VER : 0x%04x\n",
			ft5x0x_read_u16(client,	FT5X0X_REG_LIB_VERSION_H));
	dev_info(&client->dev, "FIRMID  : 0x%02x\n",
			ft5x0x_read_u8(client, FT5X0X_REG_FIRMID));
	dev_info(&client->dev, "ID      : 0x%02x\n",
			ft5x0x_read_u8(client, FT5X0X_REG_FT5201ID));
	dev_info(&client->dev, "Touch threshold is : %d\n",
			ft5x0x_read_u8(client, FT5X0X_REG_THGROUP) * 4);
	dev_info(&client->dev, "Report rate is : %dHz\n",
			ft5x0x_read_u8(client, FT5X0X_REG_PERIODACTIVE) * 10);
	dev_info(&client->dev, "%s mode\n",
			(mode == 0) ? "Polling" : "Trigger");

out:
	return 0;
}

#ifdef CONFIG_PM
static int ft5x0x_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	dev_dbg(&client->dev, "%s\n", __func__);
	return ft5x0x_ts_power_off(client);
}

static int ft5x0x_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	dev_dbg(&client->dev, "%s\n", __func__);
	ret = ft5x0x_ts_power_on(client);
	if (ret) {
		dev_err(&client->dev, "%s: Error during power on\n",
				__func__);
		return ret;
	}

	return ft5x06_ts_reset(client, false);
}
#else

#define ft5x0x_ts_suspend	NULL
#define ft5x0x_ts_resume	NULL

#endif /* CONFIG_PM */

static const struct dev_pm_ops ft5x0x_ts_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(ft5x0x_ts_suspend, ft5x0x_ts_resume)
};

static int ft5x0x_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	struct ft5x0x_ts_platform_data *ft5x06_pdata;
	int err = 0;
	int i = 0;

	dev_dbg(&client->dev, "EDT FT5X06 touchscreen driver probing\n");

#ifdef CONFIG_OF
	ft5x06_pdata = client->dev.platform_data =
		ft5x0x_ts_of_get_platdata(client);
	if (IS_ERR(ft5x06_pdata)) {
		err = PTR_ERR(ft5x06_pdata);
		return err;
	}
#else
	ft5x06_pdata = client->dev.platform_data;
#endif

	ft5x0x_set_pinctrl_state(&client->dev, ft5x06_pdata->pins_default);

	if (ft5x06_pdata->pm_platdata) {
		err = device_state_pm_set_class(&client->dev,
			ft5x06_pdata->pm_platdata->pm_user_name);
		if (err) {
			dev_err(&client->dev,
				"Error while setting the pm class\n");
			goto exit_pm_class;
		}

		err = device_state_pm_set_state_by_name(&client->dev,
				ft5x06_pdata->pm_platdata->pm_state_D0_name);
		if (err)
			goto exit_pm_class;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (gpio_is_valid(ft5x06_pdata->reset_pin)) {
		/* this pulls reset down, enabling the low active reset */
		err = gpio_request(ft5x06_pdata->reset_pin,
				"edt-ft5x06 reset");
		if (err) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as reset pin, error %d\n",
				ft5x06_pdata->reset_pin, err);
			goto exit_reset_failed;
		}
	} else {
		err = -EINVAL;
		dev_err(&client->dev,
			"invalid GPIO %d as reset pin\n",
			ft5x06_pdata->reset_pin);
		goto exit_reset_failed;
	}

	err = ft5x06_ts_reset(client, true);
	if (err) {
		dev_err(&client->dev, "reset failed\n");
		goto exit_reset_failed;
	}

	dev_dbg(&client->dev, "%s: kzalloc\n", __func__);
	ft5x0x_ts = devm_kzalloc(&client->dev, sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	dev_dbg(&client->dev, "%s: i2c_set_clientdata\n", __func__);
	i2c_set_clientdata(client, ft5x0x_ts);
	ft5x0x_ts->client = client;

	dev_dbg(&client->dev, "%s: INIT_DELAYED_WORK\n", __func__);
	INIT_DELAYED_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	dev_dbg(&client->dev, "%s: create_singlethread_workqueue\n", __func__);
	ft5x0x_ts->ts_workqueue =
	    create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		dev_err(&client->dev, "singlethread error = %d\n", err);
		goto exit_create_singlethread;
	}

	spin_lock_init(&ft5x0x_ts->btn_lock);
	spin_lock_init(&ft5x0x_ts->en_lock);
	ft5x0x_ts->btn_active = 0;
	ft5x0x_ts->enable = 1;

	err = request_irq(client->irq, ft5x0x_ts_interrupt, IRQF_DISABLED,
			"ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	dev_dbg(&client->dev, "%s: input_allocate_device\n", __func__);
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			ft5x06_pdata->screen_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			ft5x06_pdata->screen_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 5, 0, 0);
#else
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_X, 0,
				ft5x06_pdata->screen_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0,
				ft5x06_pdata->screen_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_MAX, 0, 0);
#endif

	for (i = 0; i < KEY_INDEX_MAX; i++) {
		if (ft5x06_pdata->key_x[i])
			set_bit(android_key[i], input_dev->keybit);
	}

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_dev->name = FT5X0X_NAME;
	dev_dbg(&client->dev, "%s: TS_MAX_X_COORD %d TS_MAX_Y_COORD %d\n",
		input_dev->name, ft5x06_pdata->screen_max_x,
		ft5x06_pdata->screen_max_y);
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"%s: failed to register input device: %s\n",
			__func__, dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &ft5x0x_ts_attr_group);
	if (err)
		goto exit_sysfs_failed;

	dev_dbg(&client->dev, "%s: probe over %d, client->irq=%d\n",
			__func__, err, client->irq);
	goto out;

exit_sysfs_failed:
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, ft5x0x_ts);
exit_irq_request_failed:
	cancel_delayed_work(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
exit_alloc_data_failed:
exit_reset_failed:
exit_check_functionality_failed:
exit_pm_class:
out:
	return err;
}

static int ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	struct ft5x0x_ts_platform_data *pdata =
		dev_get_platdata(&client->dev);

	dev_dbg(&client->dev, "%s\n", __func__);
	ft5x0x_set_pinctrl_state(&client->dev, pdata->pins_inactive);
	free_irq(client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	cancel_delayed_work(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	kfree(ft5x0x_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{"ft5x06", 0}, {}
};

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe = ft5x0x_ts_probe,
	.remove = ft5x0x_ts_remove,
	.id_table = ft5x0x_ts_id,
	.driver = {
		.name = FT5X0X_NAME,
		.owner = THIS_MODULE,
		.pm = &ft5x0x_ts_pm,
	},
};

static int __init ft5x06_ts_init(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&ft5x0x_pm_class);
	if (ret)
		return ret;
#endif
	return i2c_add_driver(&ft5x0x_ts_driver);
}

static void __exit ft5x06_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x06_ts_init);
module_exit(ft5x06_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
