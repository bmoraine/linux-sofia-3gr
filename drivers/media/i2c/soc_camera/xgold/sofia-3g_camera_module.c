/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef CONFIG_OF
#error "this file requires device tree support"
#endif

#ifndef SOFIA_3G_CAMERA_MODULE_H
#define SOFIA_3G_CAMERA_MODULE_H

#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/device_state_pm.h>
#endif
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-subdev.h>
#include <linux/lcm.h>

#include <linux/platform_data/platform_camera_module.h>

#define OF_OV_GPIO_PD "intel,pd-gpio"
#define OF_OV_GPIO_DVDD "intel,pd-gpio_1v2"
#define OF_OV_GPIO_FLASH "intel,flash-gpio"
#define OF_OV_GPIO_TORCH "intel,torch-gpio"
#define OF_OV_GPIO_RESET "intel,rst-gpio"

const char *PLTFRM_CAMERA_MODULE_PIN_PD = OF_OV_GPIO_PD;
const char *PLTFRM_CAMERA_MODULE_PIN_DVDD = OF_OV_GPIO_DVDD;
const char *PLTFRM_CAMERA_MODULE_PIN_FLASH = OF_OV_GPIO_FLASH;
const char *PLTFRM_CAMERA_MODULE_PIN_TORCH = OF_OV_GPIO_TORCH;
const char *PLTFRM_CAMERA_MODULE_PIN_RESET = OF_OV_GPIO_RESET;

#define I2C_M_WR 0
#define I2C_MSG_MAX 300
#define I2C_DATA_MAX (I2C_MSG_MAX * 3)
#define I2C_MSG_LEN_8BIT 0x01
#define I2C_MSG_LEN_16BIT 0x02
#define I2C_MSG_LEN_32BIT 0x04

struct pltfrm_camera_module_gpio {
	int pltfrm_gpio;
	const char *label;
	bool active_low;
};

struct pltfrm_camera_module_data {
	struct pltfrm_camera_module_gpio gpios[5];
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct device_pm_platdata *pm_platdata;
	struct v4l2_subdev *af_ctrl;
	const char *flash_driver_name;
};

/* ======================================================================== */

static int pltfrm_camera_module_set_pinctrl_state(
	struct v4l2_subdev *sd,
	struct pinctrl_state *state)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct pltfrm_camera_module_data *pdata =
		dev_get_platdata(&client->dev);
	int ret = 0;

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (IS_ERR_VALUE(ret))
			pltfrm_camera_module_pr_debug(sd,
				"could not set pins\n");
	}

	return ret;
}

static int pltfrm_camera_module_init_gpio(
	struct v4l2_subdev *sd)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct pltfrm_camera_module_data *pdata =
		dev_get_platdata(&client->dev);
	int i;

	ret = pltfrm_camera_module_set_pinctrl_state(sd, pdata->pins_default);
	if (IS_ERR_VALUE(ret))
		goto err;

	for (i = 0; i < ARRAY_SIZE(pdata->gpios); i++) {
		if (gpio_is_valid(pdata->gpios[i].pltfrm_gpio)) {
			pltfrm_camera_module_pr_debug(sd,
				"requesting GPIO #%d ('%s')\n",
				pdata->gpios[i].pltfrm_gpio,
				pdata->gpios[i].label);
			ret = gpio_request_one(
				pdata->gpios[i].pltfrm_gpio,
				GPIOF_DIR_OUT,
				pdata->gpios[i].label);
			if (IS_ERR_VALUE(ret)) {
				pltfrm_camera_module_pr_err(sd,
					"failed to request GPIO #%d ('%s')\n",
					pdata->gpios[i].pltfrm_gpio,
					pdata->gpios[i].label);
				goto err;
			}
			if (pdata->gpios[i].label ==
				PLTFRM_CAMERA_MODULE_PIN_PD)
				ret = pltfrm_camera_module_set_pin_state(sd,
					pdata->gpios[i].label,
					PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
			else if (pdata->gpios[i].label ==
				PLTFRM_CAMERA_MODULE_PIN_RESET)
				ret = pltfrm_camera_module_set_pin_state(sd,
					pdata->gpios[i].label,
					PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
			else
				ret = pltfrm_camera_module_set_pin_state(sd,
					pdata->gpios[i].label,
					PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE
					);
		}
	}

	return 0;
err:
	pltfrm_camera_module_pr_err(sd, "failed with error %d\n", ret);
	return ret;
}

#define GPIO_HARDWARE_ID_0 55
#define GPIO_HARDWARE_ID_1 48
#define UNKNOWN (-1)
enum {
	SR,
	ER,
	PR,
	MP,
};

static int get_gpio_170(unsigned int pin) {
	int ret = 0;

	if(gpio_request(pin, "HW_ID")) {
		pr_info("Fail to get pcb id pin %d\n", pin);
		return 0;
	}

	gpio_direction_input(pin);
	ret = gpio_get_value_cansleep(pin);
	printk("SOC GPIO %d := %x\n", pin, ret);
	gpio_free(pin);

	return ret;
}

int get_hw_id_170(void) {
	int ret = -1;
	int tmp = 0;
	int hardware_id = 0;

	tmp = get_gpio_170(GPIO_HARDWARE_ID_1);
	hardware_id |= (tmp<<1);
	tmp = get_gpio_170(GPIO_HARDWARE_ID_0);
	hardware_id |= tmp;
	switch (hardware_id) {
	case 0:
		return SR;
	case 1:
		return ER;
	case 2:
		return PR;
	case 3:
		return MP;
	default:
		return UNKNOWN;
	}
	return ret;
}

static struct pltfrm_camera_module_data *pltfrm_camera_module_get_data(
	struct v4l2_subdev *sd)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device_node *np = of_node_get(client->dev.of_node);
	struct device_node *af_ctrl_node = NULL;
	struct i2c_client *af_ctrl_client = NULL;
	struct pltfrm_camera_module_data *pdata = NULL;

	pltfrm_camera_module_pr_debug(sd, "\n");

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (IS_ERR_OR_NULL(pdata)) {
		ret = -ENOMEM;
		goto err;
	}

	client->dev.platform_data = pdata;

	ret = of_property_read_string(np, "intel.flash-driver",
			&pdata->flash_driver_name);
	if (ret) {
		pltfrm_camera_module_pr_warn(sd,
				"cannot not get flash-driver property of node %s\n",
				np->name);
		pdata->flash_driver_name = "0";
		} else {
		pltfrm_camera_module_pr_info(sd,
			"camera module flash driver is %s\n",
			(char *)pdata->flash_driver_name);
		}

	af_ctrl_node = of_parse_phandle(np, "intel,af-ctrl", 0);
	if (!IS_ERR_OR_NULL(af_ctrl_node)) {
		af_ctrl_client = of_find_i2c_device_by_node(af_ctrl_node);
		of_node_put(af_ctrl_node);
		if (IS_ERR_OR_NULL(af_ctrl_client)) {
			pltfrm_camera_module_pr_err(sd,
				"cannot not get node\n");
			ret = -EFAULT;
			goto err;
		}
		pdata->af_ctrl = i2c_get_clientdata(af_ctrl_client);
		if (IS_ERR_OR_NULL(pdata->af_ctrl)) {
			pltfrm_camera_module_pr_warn(sd,
				"cannot not get camera i2c client, maybe not yet created, deferring device probing...\n");
			ret = -EPROBE_DEFER;
			goto err;
		}
		pltfrm_camera_module_pr_info(sd,
			"camera module has auto focus controller %s\n",
			pltfrm_dev_string(pdata->af_ctrl));
	}

	pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if ((client->addr == 0x21) && (get_hw_id_170() == ER)) {
		pltfrm_camera_module_pr_err(sd," disable pinctrl\n");
	} else {
		if (!IS_ERR(pdata->pinctrl)) {

			pdata->pins_default = pinctrl_lookup_state(
				pdata->pinctrl, PINCTRL_STATE_DEFAULT);
			if (IS_ERR(pdata->pins_default))
				pltfrm_camera_module_pr_warn(sd,
				"could not get default pinstate\n");

			pdata->pins_sleep = pinctrl_lookup_state(
				pdata->pinctrl, PINCTRL_STATE_SLEEP);
			if (IS_ERR(pdata->pins_sleep))
				pltfrm_camera_module_pr_warn(sd,
				"could not get sleep pinstate\n");

			pdata->pins_inactive = pinctrl_lookup_state(
				pdata->pinctrl, "inactive");
			if (IS_ERR(pdata->pins_inactive))
				pltfrm_camera_module_pr_warn(sd,
				"could not get inactive pinstate\n");
		}
	}


	pdata->gpios[0].label = PLTFRM_CAMERA_MODULE_PIN_PD;
	pdata->gpios[0].pltfrm_gpio =
		of_get_named_gpio_flags(np, pdata->gpios[0].label, 0, NULL);
	if ((client->addr == 0x21) && (pdata->gpios[0].pltfrm_gpio == 14) && (get_hw_id_170() == ER)) {
		pdata->gpios[0].pltfrm_gpio=63;
	}
	pdata->gpios[0].active_low =
		of_property_read_bool(np, OF_OV_GPIO_PD "-is_active_low");

	pdata->gpios[1].label = PLTFRM_CAMERA_MODULE_PIN_DVDD;
	pdata->gpios[1].pltfrm_gpio =
		of_get_named_gpio_flags(np, pdata->gpios[1].label, 0, NULL);
	pdata->gpios[1].active_low =
		of_property_read_bool(np, OF_OV_GPIO_DVDD "-is_active_low");

	pdata->gpios[2].label = PLTFRM_CAMERA_MODULE_PIN_FLASH;
	pdata->gpios[2].pltfrm_gpio =
		of_get_named_gpio_flags(np, pdata->gpios[2].label, 0, NULL);
	pdata->gpios[2].active_low =
		of_property_read_bool(np, OF_OV_GPIO_FLASH "-is_active_low");

	pdata->gpios[3].label = PLTFRM_CAMERA_MODULE_PIN_TORCH;
	pdata->gpios[3].pltfrm_gpio =
		of_get_named_gpio_flags(np, pdata->gpios[3].label, 0, NULL);
	pdata->gpios[3].active_low =
		of_property_read_bool(np, OF_OV_GPIO_TORCH "-is_active_low");

	pdata->gpios[4].label = PLTFRM_CAMERA_MODULE_PIN_RESET;
	pdata->gpios[4].pltfrm_gpio =
		of_get_named_gpio_flags(np, pdata->gpios[4].label, 0, NULL);
	pdata->gpios[4].active_low =
		of_property_read_bool(np, OF_OV_GPIO_RESET "-is_active_low");

	pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pdata->pm_platdata)) {
		pltfrm_camera_module_pr_err(sd,
			"could not get platform PM data\n");
		ret = -EINVAL;
		goto err;
	}

	of_node_put(np);
	return pdata;
err:
	pltfrm_camera_module_pr_err(sd, "failed with error %d\n", ret);
	if (!IS_ERR_OR_NULL(pdata))
		devm_kfree(&client->dev, pdata);
	of_node_put(np);
	return ERR_PTR(ret);
}

static int pltfrm_camera_module_pix_frmt2code(
	const char *pix_frmt)
{
	if (0 == strcmp(pix_frmt, "BAYER_BGGR8"))
		return V4L2_MBUS_FMT_SBGGR8_1X8;
	if (0 == strcmp(pix_frmt, "BAYER_GBRG8"))
		return V4L2_MBUS_FMT_SGBRG8_1X8;
	if (0 == strcmp(pix_frmt, "BAYER_GRBG8"))
		return V4L2_MBUS_FMT_SGRBG8_1X8;
	if (0 == strcmp(pix_frmt, "BAYER_RGGB8"))
		return V4L2_MBUS_FMT_SRGGB8_1X8;
	if (0 == strcmp(pix_frmt, "BAYER_BGGR10"))
		return V4L2_MBUS_FMT_SBGGR10_1X10;
	if (0 == strcmp(pix_frmt, "BAYER_GBRG10"))
		return V4L2_MBUS_FMT_SGBRG10_1X10;
	if (0 == strcmp(pix_frmt, "BAYER_GRBG10"))
		return V4L2_MBUS_FMT_SGRBG10_1X10;
	if (0 == strcmp(pix_frmt, "BAYER_RGGB10"))
		return V4L2_MBUS_FMT_SRGGB10_1X10;
	if (0 == strcmp(pix_frmt, "BAYER_BGGR12"))
		return V4L2_MBUS_FMT_SBGGR12_1X12;
	if (0 == strcmp(pix_frmt, "BAYER_GBRG12"))
		return V4L2_MBUS_FMT_SGBRG12_1X12;
	if (0 == strcmp(pix_frmt, "BAYER_GRBG12"))
		return V4L2_MBUS_FMT_SGRBG12_1X12;
	if (0 == strcmp(pix_frmt, "BAYER_RGGB12"))
		return V4L2_MBUS_FMT_SRGGB12_1X12;
	if (0 == strcmp(pix_frmt, "YUYV8"))
		return V4L2_MBUS_FMT_YUYV8_2X8;
	if (0 == strcmp(pix_frmt, "YUYV10"))
		return V4L2_MBUS_FMT_YUYV10_2X10;
	if (0 == strcmp(pix_frmt, "UYUV8"))
		return V4L2_MBUS_FMT_UYVY8_2X8;
	return -EINVAL;
}

static int pltfrm_camera_module_config_matches(
	struct v4l2_subdev *sd,
	struct device_node *config,
	struct v4l2_mbus_framefmt *frm_fmt,
	struct v4l2_subdev_frame_interval *frm_intrvl)
{
	int ret = 0;
	struct property *prop;
	const char *of_pix_fmt;
	bool match = true;
	u32 min, min2, max, max2;
	u32 numerator, denominator;

	pltfrm_camera_module_pr_debug(sd,
		"pix_frm %d, %dx%d@%d/%dfps, config %s\n",
		frm_fmt->code, frm_fmt->width, frm_fmt->height,
		frm_intrvl->interval.denominator,
		frm_intrvl->interval.numerator,
		config->name);

	/* check pixel format */
	of_property_for_each_string(config, "intel,frm-pixel-format",
		prop, of_pix_fmt) {
		if (pltfrm_camera_module_pix_frmt2code(of_pix_fmt) ==
			frm_fmt->code) {
			match = true;
			break;
		}
	}

	if (!match)
		return 0;

	/* check frame width */
	ret = of_property_read_u32(config, "intel,frm-width", &min);
	if (ret == -EINVAL) {
		ret = of_property_read_u32_index(config,
				"intel,frm-width-range", 0, &min);
		if (ret == -EINVAL) {
			min = 0;
			max = UINT_MAX;
		} else if (IS_ERR_VALUE(ret)) {
			pltfrm_camera_module_pr_err(sd,
					"malformed property 'intel,frm-width-range'\n");
			goto err;
		} else {
			ret = of_property_read_u32_index(config,
					"intel,frm-width-range", 1, &max);
			if (IS_ERR_VALUE(ret)) {
				pltfrm_camera_module_pr_err(sd,
				"malformed property 'intel,frm-width-range'\n");
				goto err;
			}
		}
	} else if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,
				"malformed property 'intel,frm-width'\n");
		goto err;
	} else
		max = min;
	if ((frm_fmt->width < min) || (frm_fmt->width > max))
		return 0;

	/* check frame height */
	ret = of_property_read_u32(config, "intel,frm-height", &min);
	if (ret == -EINVAL) {
		ret = of_property_read_u32_index(config,
				"intel,frm-height-range", 0, &min);
		if (ret == -EINVAL) {
			min = 0;
			max = UINT_MAX;
		} else if (IS_ERR_VALUE(ret)) {
			pltfrm_camera_module_pr_err(sd,
				"malformed property 'intel,frm-height-range'\n");
			goto err;
		} else {
			ret = of_property_read_u32_index(config,
					"intel,frm-height-range", 1, &max);
			if (IS_ERR_VALUE(ret)) {
				pltfrm_camera_module_pr_err(sd,
					"malformed property 'intel,frm-height-range'\n");
				goto err;
			}
		}
	} else if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,
				"malformed property 'intel,frm-height'\n");
		goto err;
	} else
		max = min;
	if ((frm_fmt->height < min) || (frm_fmt->height > max))
		return 0;

	/* check frame interval */
	ret = of_property_read_u32_index(config, "intel,frm-interval", 0, &min);
	if (ret == -EINVAL) {
		ret = of_property_read_u32_index(config,
				"intel,frm-interval-range", 0, &min);
		if (ret == -EINVAL) {
			min = min2 = 0;
			max = max2 = UINT_MAX;
		} else if (IS_ERR_VALUE(ret)) {
			pltfrm_camera_module_pr_err(sd,
					"malformed property 'intel,frm-interval-range'\n");
			goto err;
		} else {
			ret |= of_property_read_u32_index(config,
				"intel,frm-interval-range", 1, &min2);
			ret |= of_property_read_u32_index(config,
				"intel,frm-interval-range", 2, &max);
			ret |= of_property_read_u32_index(config,
				"intel,frm-interval-range", 3, &max2);
			if (IS_ERR_VALUE(ret)) {
				pltfrm_camera_module_pr_err(sd,
					"malformed property 'intel,frm-interval-range'\n");
				goto err;
			}
		}
	} else if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,
			"malformed property 'intel,frm-interval'\n");
		goto err;
	} else {
		ret = of_property_read_u32_index(config,
			"intel,frm-interval", 1, &min2);
		if (IS_ERR_VALUE(ret)) {
			pltfrm_camera_module_pr_err(sd,
				"malformed property 'intel,frm-interval'\n");
			goto err;
		}
		max = min;
		max2 = min2;
	}

	/* normalize frame intervals */
	denominator = lcm(min2, frm_intrvl->interval.denominator);
	denominator = lcm(max2, denominator);
	numerator = denominator / frm_intrvl->interval.denominator *
		frm_intrvl->interval.numerator;

	min = denominator / min2 * min;
	max = denominator / max2 * max;

	if ((numerator < min) || (numerator > max))
		return 0;

	return 1;
err:
	pltfrm_camera_module_pr_err(sd,
			"failed with error %d\n", ret);
	return ret;
}

static int pltfrm_camera_module_write_reglist_node(
	struct v4l2_subdev *sd,
	struct device_node *config_node)
{
	struct property *reg_table_prop;
	struct pltfrm_camera_module_reg *reg_table = NULL;
	u32 reg_table_num_entries;
	u32 i = 0;
	int ret = 0;

	reg_table_prop = of_find_property(config_node, "intel,reg-table",
		&reg_table_num_entries);
	if (!IS_ERR_OR_NULL(reg_table_prop)) {
		if (((reg_table_num_entries / 12) == 0) ||
			(reg_table_num_entries % 3)) {
				pltfrm_camera_module_pr_err(sd,
					"wrong register format in %s, must be 'type, address, value' per register\n",
					config_node->name);
				ret = -EINVAL;
				goto err;
		}

		reg_table_num_entries /= 12;
		reg_table = (struct pltfrm_camera_module_reg *)
			kmalloc(reg_table_num_entries *
				sizeof(struct pltfrm_camera_module_reg),
				GFP_KERNEL);
		if (IS_ERR_OR_NULL(reg_table)) {
			pltfrm_camera_module_pr_err(sd,
				"memory allocation failed\n");
			ret = -ENOMEM;
			goto err;
		}

		pltfrm_camera_module_pr_debug(sd,
			"patching config with %s (%d registers)\n",
			 config_node->name, reg_table_num_entries);
		for (i = 0; i < reg_table_num_entries; i++) {
			u32 val;
			ret |= of_property_read_u32_index(
				config_node, "intel,reg-table",
				3 * i, &val);
			reg_table[i].flag = val;
			ret |= of_property_read_u32_index(
				config_node, "intel,reg-table",
				3 * i + 1, &val);
			reg_table[i].reg = val;
			ret |= of_property_read_u32_index(
				config_node, "intel,reg-table",
				3 * i + 2, &val);
			reg_table[i].val = val;
			if (IS_ERR_VALUE(ret)) {
				pltfrm_camera_module_pr_err(sd,
					"error while reading property %s at index %d\n",
					"intel,reg-table", i);
				goto err;
			}
		}
		ret = pltfrm_camera_module_write_reglist(
			sd, reg_table, reg_table_num_entries);
		if (IS_ERR_VALUE(ret))
			goto err;
		kfree(reg_table);
		reg_table = NULL;
	}
	return 0;
err:
	pltfrm_camera_module_pr_err(sd,
			"failed with error %d\n", ret);
	if (NULL != reg_table)
		kfree(reg_table);
	return ret;
}



/* ======================================================================== */

const char *pltfrm_dev_string(
	struct v4l2_subdev *sd)
{
	struct i2c_client *client;
	if (IS_ERR_OR_NULL(sd))
		return "";
	client = v4l2_get_subdevdata(sd);
	if (IS_ERR_OR_NULL(client))
		return "";
	return dev_driver_string(&client->dev);
}

int pltfrm_camera_module_read_reg(
		struct v4l2_subdev *sd,
	u16 data_length,
	u16 reg,
	u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	struct i2c_msg msg[2];
	unsigned char data[4] = { 0, 0, 0, 0 };

	if (!client->adapter) {
		pltfrm_camera_module_pr_err(sd, "client->adapter NULL\n");
		return -ENODEV;
	}

	if (data_length != I2C_MSG_LEN_8BIT &&
	    data_length != I2C_MSG_LEN_16BIT &&
	    data_length != I2C_MSG_LEN_32BIT) {
		pltfrm_camera_module_pr_err(sd,
			"%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_WR;
	msg[0].len = 2;
	msg[0].buf = data;

	/* High byte goes out first */
	data[0] = (u8) (reg >> 8);
	data[1] = (u8) (reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = data_length;
	msg[1].buf = data;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		if (ret >= 0)
			ret = -EIO;

		pltfrm_camera_module_pr_err(sd,
			"i2c read from offset 0x%08x failed with error %d\n",
			reg, ret);

		return ret;
	}

	*val = 0;
	/* High byte comes first */
	if (data_length == I2C_MSG_LEN_8BIT)
		*val = data[0];
	else if (data_length == I2C_MSG_LEN_16BIT)
		*val = data[1] + (data[0] << 8);
	else
		*val = data[3] + (data[2] << 8) +
		    (data[1] << 16) + (data[0] << 24);

	return 0;
}

/* ======================================================================== */

int pltfrm_camera_module_write_reg(
	struct v4l2_subdev *sd,
	u16 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	struct i2c_msg msg[1];
	unsigned char data[3];

	if (!client->adapter) {
		pltfrm_camera_module_pr_err(sd, "client->adapter NULL\n");
		return -ENODEV;
	}

	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 3;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);
	data[1] = (u8) (reg & 0xff);
	data[2] = val;

	ret = i2c_transfer(client->adapter, msg, 1);

	if (ret != 1) {
		pltfrm_camera_module_pr_err(sd,
			"i2c write to offset 0x%08x failed with error %d\n",
			reg, ret);
		return ret;
	}

	return 0;
}

/* ======================================================================== */

int pltfrm_camera_module_write_reglist(
	struct v4l2_subdev *sd,
	const struct pltfrm_camera_module_reg reglist[],
	int len)
{
	static const struct regmap_config regmap_config = {
		.reg_bits = 16,
		.val_bits = 8,
		.max_register = 0xffff,
		.cache_type = REGCACHE_RBTREE,
		.use_single_rw = 0,
	};
	struct regmap *regmap;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i, r = 0;
	regmap = regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	regcache_cache_only(regmap, true);
	regcache_cache_bypass(regmap, false);
	for (i = 0; i < len; i++) {
		switch (reglist[i].flag) {
		case PLTFRM_CAMERA_MODULE_REG_TYPE_DATA:
			r = regcache_sync(regmap);
			if (r < 0)
				break;
			regcache_cache_only(regmap, false);
			regcache_cache_bypass(regmap, true);
			r = regmap_write(regmap, reglist[i].reg & 0xffff,
					 reglist[i].val & 0xff);

			regcache_cache_bypass(regmap, false);
			regcache_cache_only(regmap, true);
			break;
		case PLTFRM_CAMERA_MODULE_REG_TYPE_DATA_ASYNC:
			r = regmap_write(regmap, reglist[i].reg & 0xffff,
					 reglist[i].val & 0xff);
			break;
		case PLTFRM_CAMERA_MODULE_REG_TYPE_TIMEOUT:
			r = regcache_sync(regmap);
			mdelay(reglist[i].val);
			break;
		default:
			pltfrm_camera_module_pr_debug(sd, "unknown command\n");
			r = -EBADE;
			break;
		}
		if (r < 0)
			break;
	}
	if (r == 0)
		r = regcache_sync(regmap);
	regmap_exit(regmap);
	return r;
}

static int pltfrm_camera_module_init_pm(
	struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct pltfrm_camera_module_data *pdata =
		dev_get_platdata(&client->dev);
	int ret;

	ret = device_state_pm_set_class(&client->dev,
				pdata->pm_platdata->pm_user_name);
	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,
			"registering to PM class failed\n");
		return -EINVAL;
	}

	ret = device_pm_get_states_handlers(&client->dev, pdata->pm_platdata);
	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,
			"errors while retrieving PM states\n");
		return -EINVAL;
	}

	return 0;
}

int pltfrm_camera_module_set_pm_state(
	struct v4l2_subdev *sd,
	enum device_pm_state state)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct pltfrm_camera_module_data *pdata =
		dev_get_platdata(&client->dev);

	ret = device_state_pm_set_state(&client->dev,
			get_device_pm_state(pdata->pm_platdata, state));
	if (IS_ERR_VALUE(ret))
		pltfrm_camera_module_pr_err(sd,
			"set PM state to %d failed (%d)\n", state, ret);

	return ret;
}

int pltfrm_camera_module_set_pin_state(
	struct v4l2_subdev *sd,
	const char *pin,
	enum pltfrm_camera_module_pin_state state)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct pltfrm_camera_module_data *pdata =
		dev_get_platdata(&client->dev);
	int gpio_val;
	int i;

	for (i = 0; i < ARRAY_SIZE(pdata->gpios); i++) {
		if (pin == pdata->gpios[i].label) {
			if (!gpio_is_valid(pdata->gpios[i].pltfrm_gpio))
				return 0;
			if (state == PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE)
				gpio_val = pdata->gpios[i].active_low ? 0 : 1;
			else
				gpio_val = pdata->gpios[i].active_low ? 1 : 0;
			gpio_set_value(pdata->gpios[i].pltfrm_gpio, gpio_val);
			pltfrm_camera_module_pr_debug(sd,
				"set GPIO #%d ('%s') to %s\n",
				pdata->gpios[i].pltfrm_gpio,
				pdata->gpios[i].label,
				gpio_val ? "HIGH" : "LOW");
			return 0;
		}
	}

	pltfrm_camera_module_pr_err(sd,
		"unknown pin '%s'\n",
		pin);
	return -EINVAL;
}

int pltfrm_camera_module_s_power(
	struct v4l2_subdev *sd,
	int on)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct pltfrm_camera_module_data *pdata =
		dev_get_platdata(&client->dev);

	pltfrm_camera_module_pr_debug(sd, "%s\n", on ? "on" : "off");

	if (on) {
		/* Enable clock and voltage to Secondary Camera Sensor	*/
		ret = pltfrm_camera_module_set_pm_state(sd, PM_STATE_D0);
		if (IS_ERR_VALUE(ret))
			pltfrm_camera_module_pr_err(sd,
				"set PM state failed (%d), could not power on camera\n",
				ret);
		else {
			pltfrm_camera_module_pr_debug(sd,
				"set PM state to %d successful, camera module is on\n",
				PM_STATE_D0);
			ret = pltfrm_camera_module_set_pinctrl_state(
				sd, pdata->pins_default);
		}
	} else {
		/* Disable clock and voltage to Secondary Camera Sensor  */
		ret = pltfrm_camera_module_set_pinctrl_state(
			sd, pdata->pins_sleep);
		if (!IS_ERR_VALUE(ret)) {
			ret = pltfrm_camera_module_set_pm_state(
				sd, PM_STATE_D3);
			if (IS_ERR_VALUE(ret))
				pltfrm_camera_module_pr_err(sd,
					"set PM state failed (%d), could not power off camera\n",
					ret);
			else
				pltfrm_camera_module_pr_debug(sd,
					"set PM state to %d successful, camera module is off\n",
					PM_STATE_D3);
		}
	}
	return ret;
}

struct v4l2_subdev *pltfrm_camera_module_get_af_ctrl(
	struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct pltfrm_camera_module_data *pdata =
		dev_get_platdata(&client->dev);

	return pdata->af_ctrl;
}

char *pltfrm_camera_module_get_flash_driver_name(
	struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct pltfrm_camera_module_data *pdata =
		dev_get_platdata(&client->dev);

	return (char *)pdata->flash_driver_name;
}

int pltfrm_camera_module_patch_config(
	struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *frm_fmt,
	struct v4l2_subdev_frame_interval *frm_intrvl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device_node *parent_node = of_node_get(client->dev.of_node);
	struct device_node *child_node = NULL, *prev_node = NULL;
	int ret = 0;

	pltfrm_camera_module_pr_debug(sd, "pix_fmt %d, %dx%d@%d/%dfps\n",
		frm_fmt->code, frm_fmt->width, frm_fmt->height,
		frm_intrvl->interval.denominator,
		frm_intrvl->interval.numerator);

	while (!IS_ERR_OR_NULL(child_node =
		of_get_next_child(parent_node, prev_node))) {
		if (0 == strncasecmp(child_node->name,
			"intel,camera-module-config",
			strlen("intel,camera-module-config"))) {
			ret = pltfrm_camera_module_config_matches(
				sd, child_node, frm_fmt, frm_intrvl);
			if (IS_ERR_VALUE(ret))
				goto err;
			if (ret) {
				ret =
					pltfrm_camera_module_write_reglist_node(
						sd, child_node);
				if (!IS_ERR_VALUE(ret))
					goto err;
			}
		}
		of_node_put(prev_node);
		prev_node = child_node;
	}
	of_node_put(prev_node);
	of_node_put(parent_node);

	return 0;
err:
	pltfrm_camera_module_pr_err(sd,
			"failed with error %d\n", ret);
	of_node_put(prev_node);
	of_node_put(child_node);
	of_node_put(parent_node);
	return ret;
}

int pltfrm_camera_module_init(
	struct v4l2_subdev *sd,
	void **pldata)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct pltfrm_camera_module_data *pdata;

	pltfrm_camera_module_pr_debug(sd, "\n");

	pdata = pltfrm_camera_module_get_data(sd);
	if (IS_ERR_OR_NULL(pdata)) {
		pltfrm_camera_module_pr_err(sd,
			"unable to get platform data\n");
		if (NULL == pdata)
			return -EFAULT;
		else
			return (int)pdata;
	}

	ret = pltfrm_camera_module_init_pm(sd);
	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,
			"PM initialization failed (%d)\n", ret);
	} else {
		ret = pltfrm_camera_module_init_gpio(sd);
		if (IS_ERR_VALUE(ret)) {
			pltfrm_camera_module_pr_err(sd,
				"GPIO initialization failed (%d)\n", ret);
		}
	}

	if (IS_ERR_VALUE(ret))
		devm_kfree(&client->dev, pdata);
	else
		*(struct pltfrm_camera_module_data **)pldata = pdata;

	return ret;
}

void pltfrm_camera_module_release(
	struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct pltfrm_camera_module_data *pdata =
		dev_get_platdata(&client->dev);
	int i;

	/* GPIOs also needs to be freed for other sensors to use */
	for (i = 0; i < ARRAY_SIZE(pdata->gpios); i++) {
		if (gpio_is_valid(pdata->gpios[i].pltfrm_gpio)) {
			pltfrm_camera_module_pr_debug(sd,
				"free GPIO #%d ('%s')\n",
				pdata->gpios[i].pltfrm_gpio,
				pdata->gpios[i].label);
			gpio_free(
				pdata->gpios[i].pltfrm_gpio);
		}
	}
}

#endif
