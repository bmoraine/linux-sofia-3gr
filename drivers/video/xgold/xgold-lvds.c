/*
 ****************************************************************
 *
 *  Component: DCC driver
 *
 *  Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define lvds_debug(fmt, arg...) \
	pr_debug("LVDS: " fmt, ##arg)

#define lvds_err(fmt, arg...) \
	pr_err("LVDS: " fmt, ##arg)

#define lvds_warning(fmt, arg...) \
	pr_warn("LVDS: " fmt, ##arg)

#define PROP_LVDS_GPIO_EN         "intel,lvds-gpio-en"
#define PROP_LVDS_GIOP_EN_HIGH    "intel,lvds-gpio-en-high"

struct lvds_msg_t {
	u8 reg;
	u8 val;
};

struct lvds_msg_array {
	int length;
	struct lvds_msg_t *array;
};

struct lvds_devices_data {
	u8 vendor_id;
	bool is_high_enable;
	int gpio_en;
	struct lvds_msg_array init_array;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#endif
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_gpio;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
};

static int lvds_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[4] = { 0, 0, 0, 0 };

	if (!client->adapter) {
		lvds_err("%s:client->adapter is null\n", __func__);
		return -ENODEV;
	}

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	/* High byte goes out first */
	data[0] = reg;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err < 0)
		return err;

	mdelay(3);
	msg->flags = I2C_M_RD;
	msg->len = 1;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err < 0)
		return err;

	*val = data[0];

	return err;
}


static int lvds_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[3];

	if (!client->adapter) {
		lvds_err("%s:client->adapter is null\n", __func__);
		return -ENODEV;
	}

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	data[0] = reg;
	data[1] = val;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err < 0)
		lvds_err("%s:write fail\n", __func__);
	return err;
}

static int lvds_write_regs(struct i2c_client *client,
		struct lvds_msg_array *msg_array)
{
	int i = 0;
	struct lvds_msg_t *p_array = msg_array->array;

	for (i = 0; i < msg_array->length; i++) {
		lvds_write_reg(client, p_array->reg, p_array->val);
		p_array++;
	}
	return 0;
}

static int lvds_of_parse_msg(struct device *dev,
	const char *name,
	struct lvds_msg_array *msg_array)
{
	int i, ret = 0, len = 0;
	unsigned int u;
	const __be32 *p;
	int *array;
	struct property *prop;

	msg_array->length = 0;
	if (name == NULL) {
		lvds_err("ERROR: Property Name is NULL!\n");
		return -EINVAL;
	}

	of_property_for_each_u32(dev->of_node, name, prop, p, u) {
		len++;
	}

	if (len == 0)
		return 0;

	if (len % 2) {
		lvds_err("msg array length should be even\n");
		return -EINVAL;
	}
	array = devm_kzalloc(dev, sizeof(int)*len, GFP_KERNEL);
	ret = of_property_read_u32_array(dev->of_node, name,
						array, len);
	if (ret) {
		lvds_err("Can't read property:%s\n", name);
		return 0;
	}

	len = len / 2;
	msg_array->length = len;
	msg_array->array = devm_kzalloc(dev,
				sizeof(struct lvds_msg_t)*len, GFP_KERNEL);

	lvds_debug("lvds_of_parse_msg dump start\n");
	for (i = 0; i < len; i++) {
		msg_array->array[i].reg = (u8)array[2*i];
		msg_array->array[i].val = (u8)array[2*i+1];
		lvds_debug("reg: 0x%02x	val:0x%02x\n", msg_array->array[i].reg,
			msg_array->array[i].val);
	}
	lvds_debug("lvds_of_parse_msg dump end\n");
	devm_kfree(dev, array);

	return 0;
}

static int lvds_of_parse_platdata(struct device *dev,
			struct lvds_devices_data *p_data)
{
	int ret = 0;

	if (!dev->of_node) {
		lvds_err("Device tree node is NULL!\n");
		goto out;
	}

	lvds_of_parse_msg(dev, "lvds,init-code", &p_data->init_array);

#ifdef CONFIG_PLATFORM_DEVICE_PM
	p_data->pm_platdata = of_device_state_pm_setup(dev->of_node);
	if (IS_ERR(p_data->pm_platdata)) {
		lvds_err("Error during device state pm init.\n");
		p_data->pm_platdata = NULL;
	}
#endif
	p_data->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(p_data->pinctrl)) {
		lvds_err("Can not get pinctrl.\n");
		p_data->pinctrl = NULL;
		goto out;
	}

	p_data->pins_default = pinctrl_lookup_state(
		p_data->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(p_data->pins_default))
		lvds_err("could not get default pinstate\n");

	p_data->pins_sleep = pinctrl_lookup_state(
		p_data->pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR(p_data->pins_sleep))
		lvds_err("could not get sleep pinstate\n");

	p_data->pins_inactive = pinctrl_lookup_state(
		p_data->pinctrl, "inactive");
	if (IS_ERR(p_data->pins_inactive))
		lvds_err("could not get inactive pinstate\n");

	p_data->gpio_en = of_get_named_gpio_flags(dev->of_node,
			PROP_LVDS_GPIO_EN, 0, NULL);
	if (p_data->gpio_en <= 0)
		lvds_err("error getting gpio for %s\n", PROP_LVDS_GPIO_EN);

	p_data->is_high_enable = of_property_read_bool(dev->of_node,
		PROP_LVDS_GIOP_EN_HIGH);

	return ret;

out:
	ret = -EINVAL;
	return ret;
}

static int lvds_setup(struct device *dev)
{
	int ret = 0;
	int gpio_val;
	struct lvds_devices_data *p_data = dev->platform_data;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (p_data->pm_platdata) {
		ret = device_state_pm_set_state_by_name(dev,
			p_data->pm_platdata->pm_state_D0_name);
		if (ret < 0) {
			lvds_err("Error while setting the pm class\n");
			goto error_return;
		}
	}
#endif
	if (p_data->pinctrl) {
		ret = pinctrl_select_state(p_data->pinctrl,
			p_data->pins_default);
		if (ret < 0) {
			lvds_err("Error while setting PIN default state\n");
			goto error_return;
		}
	}

	if (p_data->gpio_en > 0) {
		ret = gpio_request(p_data->gpio_en, "lvds_en");
		if (ret < 0) {
			lvds_err("failed to request gpio set %d\n",
				p_data->gpio_en);
			goto error_return;
		}
		gpio_val = p_data->is_high_enable ? 1 : 0;
		gpio_set_value(p_data->gpio_en, gpio_val);
	}
	return ret;

error_return:
	return ret;
}

static int __init lvds_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int ret = 0;
	struct lvds_devices_data *p_data;

	lvds_debug("lvds_probe : start\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		lvds_err("ERROR: doesn't support I2C functionality!\n");
		return -ENODEV;
	}

	p_data = devm_kzalloc(&client->dev, sizeof(*p_data), GFP_KERNEL);
	if (!p_data) {
		lvds_err("ERROR: no memory for LVDS device data!\n");
		return -ENOMEM;
	}

	ret = lvds_of_parse_platdata(&client->dev, p_data);
	if (ret < 0)
		goto error_free_mem;

	client->dev.platform_data = p_data;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (p_data->pm_platdata) {
		ret = device_state_pm_set_class(&client->dev,
			p_data->pm_platdata->pm_user_name);
		if (ret < 0) {
			lvds_err("ERROR while LVDS initialize its PM state!\n");
			/*It should go to error_free_mem
			after Platform Device PM supports LVDS*/
			kfree(p_data->pm_platdata);
			p_data->pm_platdata = NULL;
		}
	}
#endif
	lvds_setup(&client->dev);
	lvds_read_reg(client, 0, &p_data->vendor_id);
	lvds_debug("LVDS vender id :%02x\n", p_data->vendor_id);

	if (p_data->init_array.length > 0)
		lvds_write_regs(client, &p_data->init_array);

	lvds_debug("lvds_probe : end\n");
	return 0;

error_free_mem:
	devm_kfree(&client->dev, p_data);
	client->dev.platform_data = NULL;
	return ret;
}

static int __exit lvds_remove(struct i2c_client *client)
{
	struct lvds_devices_data *p_data = client->dev.platform_data;

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	if (p_data) {
		if (p_data->pinctrl)
			pinctrl_select_state(p_data->pinctrl,
				p_data->pins_inactive);

		devm_kfree(&client->dev, p_data);
	}

	i2c_set_clientdata(client, NULL);
	return 0;
}

static int lvds_suspend(struct i2c_client *client, pm_message_t state)
{
	int ret = 0;
	int gpio_val;
	struct lvds_devices_data *p_data = client->dev.platform_data;

	lvds_debug("%s, state:%d\n", __func__, state.event);
	if (!client->adapter)
		return -ENODEV;

	if (!p_data)
		return -EFAULT;

	if (p_data->gpio_en > 0) {
		gpio_val = p_data->is_high_enable ? 0 : 1;
		gpio_set_value(p_data->gpio_en, gpio_val);
		gpio_free(p_data->gpio_en);
	}

	if (p_data->pinctrl) {
		ret = pinctrl_select_state(p_data->pinctrl, p_data->pins_sleep);
		if (ret < 0) {
			lvds_err("Error while setting PIN sleep state\n");
			return ret;
		}
	}
#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (p_data->pm_platdata) {
		ret = device_state_pm_set_state_by_name(&client->dev,
			p_data->pm_platdata->pm_state_D3_name);

		if (ret < 0)
			lvds_err("%d:could not PM state: %s\n", __LINE__,
				p_data->pm_platdata->pm_state_D3_name);
	}
#endif

	return ret;
}

static int lvds_resume(struct i2c_client *client)
{
	int ret = 0;
	struct lvds_devices_data *p_data = client->dev.platform_data;

	lvds_debug("lvds_resume\n");
	if (!client->adapter)
		return -ENODEV;

	if (p_data) {
		lvds_setup(&client->dev);
		if (p_data->init_array.length > 0)
			lvds_write_regs(client, &p_data->init_array);
	}
	return ret;
}

static const struct i2c_device_id lvds_id[] = {
	{"lvds", 0},
	{},
};

static struct i2c_driver lvds_i2c_driver = {
	.driver = {
		   .name = "lvds",
		   .owner = THIS_MODULE,
	},
	.probe = lvds_probe,
	.remove = __exit_p(lvds_remove),
	.id_table = (struct i2c_device_id *)lvds_id,
	.suspend = lvds_suspend,
	.resume = lvds_resume,
};

static int __init lvds_init(void)
{
	return i2c_add_driver(&lvds_i2c_driver);
}
fs_initcall(lvds_init);

static void __exit lvds_exit(void)
{
	i2c_del_driver(&lvds_i2c_driver);
}
module_exit(lvds_exit);

MODULE_DESCRIPTION("LVDS transmitter driver");
MODULE_ALIAS("lvds");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(i2c, lvds_id);
