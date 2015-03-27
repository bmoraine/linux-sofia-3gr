/*
* Copyright (C) 2011-2014 Intel Mobile Communications GmbH
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/input/matrix_keypad.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_data/xgold_keypad.h>
#include "xgold-keypad.h"
#include <linux/pinctrl/consumer.h>
#include <linux/of_irq.h>

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
#define XG_KPD_PM_D3	0
#define XG_KPD_PM_D0	1


static int xgold_kpd_set_pm_state(struct device *,
		struct device_state_pm_state *);
static struct device_state_pm_state *xgold_kpd_get_initial_state(
		struct device *);

static struct device_state_pm_ops xgold_kpd_pm_ops = {
	.set_state = xgold_kpd_set_pm_state,
	.get_initial_state = xgold_kpd_get_initial_state,
};

/* clocks PM states & class */
static struct device_state_pm_state xgold_kpd_pm_states[] = {
	{ .name = "disable", }, /* D3 */
	{ .name = "enable", }, /* D0 */
	{ .name = "enable_psv", },
};

DECLARE_DEVICE_STATE_PM_CLASS(xgold_kpd);

#endif


#define MAX_MATRIX_KEY_NUM 73

union row_bits {
	struct {
		u8 bit0:1;
		u8 bit1:1;
		u8 bit2:1;
		u8 bit3:1;
		u8 bit4:1;
		u8 bit5:1;
		u8 bit6:1;
		u8 bit7:1;
	} bits;
	u8 row;
};
struct key_rows {
	u8 row0;
	u8 row1;
	u8 row2;
	u8 row3;
	u8 row4;
	u8 row5;
	u8 row6;
	u8 row7;
};
struct key_num {
	u32 key_num1;
	u32 key_num2;
};
union key_matrix {
	struct key_rows key_rows;
	struct key_num key_num;
};
struct xgold_kpd_device {
	struct input_dev *input_dev;
	struct platform_device *pdev;
	struct xgold_keypad_platform_data *pdata;
	void __iomem *mmio_base;
	int kpd_irq;
	unsigned short keycodes[MAX_MATRIX_KEY_NUM];
	union key_matrix key_mail_matrix;
	union key_matrix key_old_mail_matrix;
	u8 special_key_matrix;
	u8 special_key_old_matrix;
	struct tasklet_struct xgold_report_key_tasklet;
	bool enabled;
	struct clk *clk_ahb;
#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	struct device_pm_platdata *pm_platdata;
#endif
};

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
static int xgold_kpd_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	struct xgold_kpd_device *platdata = dev_get_drvdata(dev);

	pr_info("%s: pm state %s\n", __func__, state->name);
	if (!strcmp(state->name, xgold_kpd_pm_states[XG_KPD_PM_D0].name)) {

		clk_prepare_enable(platdata->clk_ahb);

	} else if (!strcmp(state->name,
				xgold_kpd_pm_states[XG_KPD_PM_D3].name)) {
		clk_disable_unprepare(platdata->clk_ahb);

	} else
		return -EINVAL;

	return 0;
}

static struct device_state_pm_state *xgold_kpd_get_initial_state(
		struct device *dev)
{
	return &xgold_kpd_pm_states[XG_KPD_PM_D3];
}
#endif

static inline int xgold_keypad_set_pinctrl_state(struct device *dev,
						struct pinctrl_state *state)
{
	int ret = 0;
	struct xgold_keypad_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata) {
		dev_err(dev, "Unable to retrieve usif platform data\n");
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(pdata->pinctrl) && !IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	return ret;
}

static u32 kpd_read32(struct xgold_kpd_device *keypad, u16 offset)
{
	return ioread32((char *)keypad->mmio_base + offset);
}

static void kpd_write32(struct xgold_kpd_device *keypad, u16 offset, u32 val)
{
	iowrite32(val, (char *)keypad->mmio_base + offset);
}

static void xgold_ghost_key_check(struct xgold_kpd_device *keypad)
{
	u8 num_rows = keypad->pdata->key_cfg->num_row_gpios;
	u8 num_cols = keypad->pdata->key_cfg->num_col_gpios;
	const unsigned int *col_gpios = keypad->pdata->key_cfg->col_gpios;
	const unsigned int *row_gpios = keypad->pdata->key_cfg->row_gpios;
	u8 *key_matrix = (u8 *)&keypad->key_mail_matrix;
	union row_bits col_bits;
	u8 i, j;

	for (i = 0; i < num_cols; i++) {
		col_bits.bits.bit0 = (key_matrix[0] >> col_gpios[i]) & 0x1;
		col_bits.bits.bit1 = (key_matrix[1] >> col_gpios[i]) & 0x1;
		col_bits.bits.bit2 = (key_matrix[2] >> col_gpios[i]) & 0x1;
		col_bits.bits.bit3 = (key_matrix[3] >> col_gpios[i]) & 0x1;
		col_bits.bits.bit4 = (key_matrix[4] >> col_gpios[i]) & 0x1;
		col_bits.bits.bit5 = (key_matrix[5] >> col_gpios[i]) & 0x1;
		col_bits.bits.bit6 = (key_matrix[6] >> col_gpios[i]) & 0x1;
		col_bits.bits.bit7 = (key_matrix[7] >> col_gpios[i]) & 0x1;

		if ((((~col_bits.row & 0xFF)) &
			((~col_bits.row - 1) & 0xFF)) != 0) {
			for (j = 0; j < num_rows; j++) {
				if (((~key_matrix[row_gpios[j]] >>
					col_gpios[i]) & 0x1) != 0) {
					key_matrix[row_gpios[j]] |=
						(~(1<<col_gpios[i]));
					dev_info(&keypad->pdev->dev,
					"Ghost key detect in row %d col %d\n",
					i, j);
				}
			}
		}
	}
}

static void xgold_report_keyevent(struct xgold_kpd_device *keypad,
					u8 new, u8 old, u8 index)
{
	u8 i;
	for (i = 0; i < 8; i++) {
		if (TESTBIT(new, i) != TESTBIT(old, i)) {
			if (keypad->keycodes[index + i] != KEY_RESERVED) {
				input_report_key(keypad->input_dev,
						 keypad->keycodes[index + i],
						!(TESTBIT(new, i)));
				dev_dbg(&keypad->pdev->dev,
					"key event reported for %d, %d\n",
					keypad->keycodes[index + i],
					!(TESTBIT(new, i)));
			}
		}
	}
}

static void xgold_find_key_event(unsigned long data)
{
	struct xgold_kpd_device *keypad = (struct xgold_kpd_device *)data;
	struct key_num *key_num = &keypad->key_mail_matrix.key_num;
	struct key_rows *key_rows = &keypad->key_mail_matrix.key_rows;
	struct key_num *old_key_num = &keypad->key_old_mail_matrix.key_num;
	struct key_rows *old_key_rows = &keypad->key_old_mail_matrix.key_rows;
	u8 *special_key_matrix = &keypad->special_key_matrix;
	u8 *special_key_old_matrix = &keypad->special_key_old_matrix;

	key_num->key_num1 = kpd_read32(keypad, KPD_KEYNUM1);
	key_num->key_num2 = kpd_read32(keypad, KPD_KEYNUM2);
	*special_key_matrix = kpd_read32(keypad, KPD_KEYNUM3);

	dev_dbg(&keypad->pdev->dev, "key_num1 %x, old_key_num1%x\n",
				key_num->key_num1, old_key_num->key_num1);
	dev_dbg(&keypad->pdev->dev, "key_num2 %x , old_key_num2 %x\n",
				key_num->key_num2, old_key_num->key_num2);
	dev_dbg(&keypad->pdev->dev, "special_key %x, special_key_old %x\n",
				*special_key_matrix, *special_key_old_matrix);

	if (*special_key_matrix != *special_key_old_matrix) {
		xgold_report_keyevent(keypad, *special_key_matrix,
					*special_key_old_matrix, 64);
		old_key_num->key_num1 = 0xFFFFFFFF;
		old_key_num->key_num2 = 0xFFFFFFFF;
		*special_key_old_matrix = *special_key_matrix;
	} else {
		xgold_ghost_key_check(keypad);
		if (key_num->key_num1 != old_key_num->key_num1) {
			if (key_rows->row0 != old_key_rows->row0) {
				xgold_report_keyevent(keypad, key_rows->row0,
						       old_key_rows->row0, 0);
			}
			if (key_rows->row1 != old_key_rows->row1) {
				xgold_report_keyevent(keypad, key_rows->row1,
						       old_key_rows->row1, 8);
			}
			if (key_rows->row2 != old_key_rows->row2) {
				xgold_report_keyevent(keypad, key_rows->row2,
						      old_key_rows->row2, 16);
			}
			if (key_rows->row3 != old_key_rows->row3) {
				xgold_report_keyevent(keypad, key_rows->row3,
						      old_key_rows->row3, 24);
			}
		}

		if (key_num->key_num2 != old_key_num->key_num2) {
			if (key_rows->row4 != old_key_rows->row4) {
				xgold_report_keyevent(keypad, key_rows->row4,
						      old_key_rows->row4, 32);
			}
			if (key_rows->row5 != old_key_rows->row5) {
				xgold_report_keyevent(keypad, key_rows->row5,
						      old_key_rows->row5, 40);
			}
			if (key_rows->row6 != old_key_rows->row6) {
				xgold_report_keyevent(keypad, key_rows->row6,
						      old_key_rows->row6, 48);
			}
			if (key_rows->row7 != old_key_rows->row7) {
				xgold_report_keyevent(keypad, key_rows->row7,
						      old_key_rows->row7, 56);
			}
		}
		old_key_num->key_num1 = key_num->key_num1;
		old_key_num->key_num2 = key_num->key_num2;
	}
	input_sync(keypad->input_dev);
	kpd_write32(keypad, KPD_MISC, KPD_INTR_ENABLE);
}

static irqreturn_t xgold_keypad_irq_handler(int irq, void *dev_id)
{
	struct xgold_kpd_device *keypad = (struct xgold_kpd_device *)dev_id;
	u32 reg_val;

	reg_val = kpd_read32(keypad, KPD_MIS);

	dev_dbg(&keypad->pdev->dev, "KPD_MIS %x\n", reg_val);
	kpd_write32(keypad, KPD_MISC, KPD_INTR_DISABLE);
	kpd_write32(keypad, KPD_ICR, reg_val);
	tasklet_schedule(&keypad->xgold_report_key_tasklet);

	return IRQ_HANDLED;
}

static void xgold_keypad_config(struct xgold_kpd_device *keypad)
{
	u32 reg_val, val;

	kpd_write32(keypad, KPD_MISC, KPD_INTR_DISABLE);
	reg_val = kpd_read32(keypad, KPD_CONFIG);
	val = reg_val & DEB_LENGTH_MASK;
	kpd_write32(keypad, KPD_CONFIG, val | DEB_LENGTH);

	keypad->key_old_mail_matrix.key_num.key_num1 = 0xFFFFFFFF;
	keypad->key_old_mail_matrix.key_num.key_num2 = 0xFFFFFFFF;
	keypad->special_key_old_matrix = 0xFF;
	kpd_write32(keypad, KPD_MISC, KPD_INTR_ENABLE);

	return;
}

static void xgold_keypad_close(struct input_dev *dev)
{
	int ret;
	struct xgold_kpd_device *keypad =
			(struct xgold_kpd_device *)input_get_drvdata(dev);
	struct device *_dev = dev->dev.parent;
	struct xgold_keypad_platform_data *pdata = dev_get_platdata(_dev);

	dev_dbg(_dev, "keypad close\n");
	if (keypad->enabled) {
		kpd_write32(keypad, KPD_MISC, KPD_INTR_DISABLE);

#ifdef CONFIG_PLATFORM_PM
		device_state_pm_set_state_by_name(_dev,
			keypad->pm_platdata->pm_state_D3_name);
#endif
#ifdef CONFIG_PINCTRL_SINGLE
		ret = xgold_keypad_set_pinctrl_state(_dev,
						pdata->pins_inactive);
		if (ret)
			dev_err(_dev, "keypad pads inactive failed\n");
#endif

		keypad->enabled = false;
	}

	tasklet_kill(&keypad->xgold_report_key_tasklet);
	return;
}

static int xgold_keypad_open(struct input_dev *dev)
{
	int ret;
	struct xgold_kpd_device *keypad =
		(struct xgold_kpd_device *)input_get_drvdata(dev);
	struct device *_dev = dev->dev.parent;
	struct xgold_keypad_platform_data *pdata = dev_get_platdata(_dev);

	dev_dbg(_dev, "keypad open\n");
	tasklet_init(&keypad->xgold_report_key_tasklet,
				&xgold_find_key_event,
				(unsigned long)keypad);
	if (!keypad->enabled) {
#ifdef CONFIG_PLATFORM_PM
		device_state_pm_set_state_by_name(_dev,
			keypad->pm_platdata->pm_state_D0_name);
#endif
#if CONFIG_PINCTRL_SINGLE
		ret = xgold_keypad_set_pinctrl_state(_dev, pdata->pins_default);
		if (ret)
			dev_err(_dev, "keypad pads default failed\n");
#endif

		xgold_keypad_config(keypad);
		keypad->enabled = true;
	}

	return 0;
}

#ifdef CONFIG_PM
static int xgold_keypad_suspend(struct platform_device *pdev,
					  pm_message_t state)
{
	int ret;
	struct xgold_kpd_device *keypad = platform_get_drvdata(pdev);
	struct input_dev *input_dev = keypad->input_dev;
	struct device *_dev = &pdev->dev;
	struct xgold_keypad_platform_data *pdata = dev_get_platdata(_dev);

	dev_dbg(_dev, "keypad suspend\n");

	mutex_lock(&input_dev->mutex);

	if (keypad->enabled) {
		kpd_write32(keypad, KPD_MISC, KPD_INTR_DISABLE);

#ifdef CONFIG_PLATFORM_PM
		device_state_pm_set_state_by_name(_dev,
			keypad->pm_platdata->pm_state_D3_name);
#endif
#if CONFIG_PINCTRL_SINGLE
		ret = xgold_keypad_set_pinctrl_state(_dev, pdata->pins_sleep);
		if (ret)
			dev_err(_dev, "keypad pads sleep failed\n");
#endif
		keypad->enabled = false;
	}
	if (device_may_wakeup(_dev)) {
		kpd_write32(keypad, KPD_MISC, KPD_INTR_ENABLE);
		enable_irq_wake(keypad->kpd_irq);
	}
	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int xgold_keypad_resume(struct platform_device *pdev)
{
	struct xgold_kpd_device *keypad = platform_get_drvdata(pdev);
	struct input_dev *input_dev = keypad->input_dev;
	struct device *_dev = &pdev->dev;
	struct xgold_keypad_platform_data *pdata = dev_get_platdata(_dev);
	u32 reg_val, val;
	int ret;

	dev_info(_dev, "Keypad Resume\n");

	mutex_lock(&input_dev->mutex);

	if (device_may_wakeup(_dev)) {
		kpd_write32(keypad, KPD_MISC, KPD_INTR_DISABLE);
		disable_irq_wake(keypad->kpd_irq);
	}
	if (input_dev->users) {
		if (!keypad->enabled) {
#ifdef CONFIG_PLATFORM_PM
			device_state_pm_set_state_by_name(_dev,
				keypad->pm_platdata->pm_state_D0_name);
#endif
#if CONFIG_PINCTRL_SINGLE
			ret = xgold_keypad_set_pinctrl_state(_dev,
					pdata->pins_default);
			if (ret)
				dev_err(_dev, "keypad pads default failed\n");
#endif
			kpd_write32(keypad, KPD_MISC, KPD_INTR_DISABLE);
			reg_val = kpd_read32(keypad, KPD_CONFIG);
			val = reg_val & DEB_LENGTH_MASK;
			kpd_write32(keypad, KPD_CONFIG, val | DEB_LENGTH);
			kpd_write32(keypad, KPD_MISC, KPD_INTR_ENABLE);
			keypad->enabled = true;
		}
	}

	mutex_unlock(&input_dev->mutex);

	return 0;
}
#endif

#ifdef CONFIG_OF
static int xgold_keypad_parse_dt(struct device *dev,
				 struct xgold_kpd_device *keypad_data)
{
	struct device_node *np = dev->of_node;
	int len, error;
	unsigned int *used_rows, *used_cols;
	struct xgold_keypad_platform_data *pdata = dev_get_platdata(dev);

	if (!np) {
		dev_err(dev, "Can't find kpd matching node\n");
		return -EINVAL;
	}
	if (matrix_keypad_parse_of_params(dev,
			&pdata->key_cfg->num_row_gpios,
			    &pdata->key_cfg->num_col_gpios))
		return -EINVAL;

	used_rows = devm_kzalloc(dev, (sizeof(unsigned int) *
			pdata->key_cfg->num_row_gpios), GFP_KERNEL);
	used_cols = devm_kzalloc(dev, (sizeof(unsigned int) *
			pdata->key_cfg->num_col_gpios), GFP_KERNEL);
	if (!used_rows || !used_cols) {
		dev_err(dev, "not enough memory for driver data\n");
		return -ENOMEM;
	}
	pdata->key_cfg->row_gpios = used_rows;
	pdata->key_cfg->col_gpios = used_cols;

	if (!of_find_property(np, "keypad,rows", &len)) {
		dev_err(dev, "missing keypad row property\n");
		error = -EINVAL;
		goto failed;
	}
	len /= sizeof(u32);
	error = of_property_read_u32_array(np, "keypad,rows",
			(u32 *)pdata->key_cfg->row_gpios, len);
	if (error) {
		dev_err(dev, "missing keypad row property\n");
		goto failed;
	}
	if (!of_find_property(np, "keypad,columns", &len)) {
		dev_err(dev, "missing keypad columns property\n");
		error = -EINVAL;
		goto failed;
	}
	len /= sizeof(u32);
	error = of_property_read_u32_array(np, "keypad,columns",
			(u32 *)pdata->key_cfg->col_gpios, len);
	if (error) {
		dev_err(dev, "missing keypad columns property\n");

		goto failed;
	}

	/*kp_in for keys*/
	error = of_property_read_u32(np, "keypad,kp_in0",
			&(keypad_data->keycodes[64]));
	if (error != 0)
		dev_err(dev, "get keypad,kp_in0 error\n");

	error = of_property_read_u32(np, "keypad,kp_in1",
			&(keypad_data->keycodes[65]));
	if (error != 0)
		dev_err(dev, "get keypad,kp_in1 error\n");

	error = of_property_read_u32(np, "keypad,kp_in2",
			&(keypad_data->keycodes[66]));
	if (error != 0)
		dev_err(dev, "get keypad,kp_in2 error\n");

	error = of_property_read_u32(np, "keypad,kp_in3",
			&(keypad_data->keycodes[67]));
	if (error != 0)
		dev_err(dev, "get keypad,kp_in3 error\n");

	error = of_property_read_u32(np, "keypad,kp_in4",
			&(keypad_data->keycodes[68]));
	if (error != 0)
		dev_err(dev, "get keypad,kp_in4 error\n");

	error = of_property_read_u32(np, "keypad,kp_in5",
			&(keypad_data->keycodes[69]));
	if (error != 0)
		dev_err(dev, "get keypad,kp_in5 error\n");

	error = of_property_read_u32(np, "keypad,kp_in6",
			&(keypad_data->keycodes[70]));
	if (error != 0)
		dev_err(dev, "get keypad,kp_in6 error\n");

	error = of_property_read_u32(np, "keypad,kp_in7",
			&(keypad_data->keycodes[71]));
	if (error != 0)
		dev_err(dev, "get keypad,kp_in7 error\n");

	/* gpio */
	pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pdata->pinctrl))
		goto skip_pinctrl;

	pdata->pins_default = pinctrl_lookup_state(pdata->pinctrl,
						 PINCTRL_STATE_DEFAULT);
	if (IS_ERR(pdata->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	pdata->pins_sleep = pinctrl_lookup_state(pdata->pinctrl,
					       PINCTRL_STATE_SLEEP);
	if (IS_ERR(pdata->pins_sleep))
		dev_err(dev, "could not get sleep pinstate\n");

	pdata->pins_inactive = pinctrl_lookup_state(pdata->pinctrl,
					       "inactive");
	if (IS_ERR(pdata->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

#if !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	keypad_data->clk_ahb = of_clk_get_by_name(np, "clk_ahb");
	if (IS_ERR(keypad_data->clk_ahb)) {
		dev_err(dev, "Clk %s not found\n", "clk_ahb");
		keypad_data->clk_ahb = NULL;
	}
#endif

skip_pinctrl:

#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	keypad_data->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(keypad_data->pm_platdata)) {
		dev_err(dev, "Error during device state pm init\n");
		goto failed;
	}

	error = device_state_pm_set_class(dev,
			keypad_data->pm_platdata->pm_user_name);
	if (error) {
		dev_err(dev, "Error while setting the pm class\n");
		goto failed;
	}

#endif
	return 0;

failed:
	kfree(used_rows);
	kfree(used_cols);
	return error;

}
#endif

static int xgold_keypad_probe(struct platform_device *pdev)
{
	struct xgold_keypad_platform_data *pdata = dev_get_platdata(&pdev->dev);
	const struct matrix_keymap_data *keymap_data = NULL;
	struct xgold_kpd_device *keypad;
	struct resource *res;
	struct input_dev *input_dev;
	int irq, error;
#ifndef CONFIG_OF
	keymap_data = pdata->key_cfg->keymap_data;
	if (keymap_data == NULL) {
		dev_err(&pdev->dev, "no keymap defined\n");
		return -EINVAL;
	}
#endif
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq defined in platform data\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev,
			"no I/O memory defined in platform data\n");
		return -EINVAL;
	}
	input_dev = input_allocate_device();
	if (!input_dev)	{
		dev_err(&pdev->dev, "failed to allocate the input device\n");
		return -ENOMEM;
	}
	keypad = devm_kzalloc(&pdev->dev, sizeof(*keypad), GFP_KERNEL);
	if (!keypad) {
		dev_err(&pdev->dev, "not enough memory for driver data\n");
		error = -ENOMEM;
		goto failed_free_input;
	}

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "not enough memory for platform data\n");
		error = -ENOMEM;
		goto failed_free_kpd;
	}

	pdev->dev.platform_data = pdata;
	pdata->key_cfg = devm_kzalloc(&pdev->dev,
			sizeof(struct matrix_keypad_platform_data), GFP_KERNEL);
	if (!pdata->key_cfg) {
		dev_err(&pdev->dev, "not enough memory for platform data\n");
		error = -ENOMEM;
		goto failed_free_pdata;
	}
	keypad->pdata = pdata;
	error = xgold_keypad_parse_dt(&pdev->dev, keypad);
	if (error) {
		dev_err(&pdev->dev, "not enough memory for platform data\n");
		error = -EINVAL;
		goto failed_free_key;
	}
	keypad->input_dev = input_dev;
	keypad->kpd_irq = irq;
	keypad->pdev = pdev;
	keypad->pdata = pdata;
	keypad->mmio_base = ioremap(res->start, resource_size(res));
	if (keypad->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		error = -ENOMEM;
		goto failed_free_key;
	}
	input_dev->name = "kpd-xgold";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.product = kpd_read32(keypad, KPD_ID);
	input_dev->id.version = 1;
	input_dev->dev.parent = &pdev->dev;
	input_dev->open = xgold_keypad_open;
	input_dev->close = xgold_keypad_close;
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	input_dev->keycode = keypad->keycodes;
	input_dev->keycodesize = sizeof(keypad->keycodes[0]);
	input_dev->keycodemax = ARRAY_SIZE(keypad->keycodes);
	error = matrix_keypad_build_keymap(keymap_data, NULL,
				   keypad->pdata->key_cfg->num_row_gpios,
				   keypad->pdata->key_cfg->num_col_gpios,
				   &keypad->keycodes[0], input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to build keymap\n");
		goto failed_free_key;
	}
	input_set_capability(input_dev, EV_MSC, MSC_SCAN);
	input_set_drvdata(input_dev, keypad);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_unmap;
	}
	platform_set_drvdata(pdev, keypad);

	error = request_irq(irq, xgold_keypad_irq_handler,
			    IRQF_SHARED | IRQF_NO_SUSPEND, pdev->name, keypad);
	if (error) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		goto failed_free_irq;
	}
	device_init_wakeup(&pdev->dev, 1);
	dev_info(&pdev->dev, "keypad probed\n");
	return 0;

failed_free_irq:
	free_irq(keypad->kpd_irq, keypad);
failed_unmap:
	iounmap(keypad->mmio_base);
failed_free_key:
	kfree(pdata->key_cfg);
failed_free_pdata:
	kfree(pdata);
failed_free_kpd:
	kfree(keypad);
failed_free_input:
	input_free_device(input_dev);

	return error;
}

static int xgold_keypad_remove(struct platform_device *pdev)
{
	struct xgold_kpd_device *keypad = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "keypad remove\n");

	platform_set_drvdata(pdev, NULL);
	if (keypad->enabled) {
#ifdef CONFIG_PLATFORM_PM
		device_state_pm_set_state_by_name(dev->dev.parent,
			keypad->pm_platdata->pm_state_D3_name);
#endif
	}

	input_unregister_device(keypad->input_dev);
	free_irq(keypad->kpd_irq, keypad);
	iounmap(keypad->mmio_base);
	kfree(keypad);

	return 0;
}

static const struct of_device_id xgold_keypad_dt_match[] = {
	{ .compatible = "intel,keypad" },
	{},
};
MODULE_DEVICE_TABLE(of, xgold_keypad_dt_match);

static struct platform_driver xgold_keypad_driver = {
	.driver		= {
		.name	= "kpd-xgold",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(xgold_keypad_dt_match),
	},
	.probe		= xgold_keypad_probe,
	.remove		= xgold_keypad_remove,
#ifdef CONFIG_PM
	.suspend	= xgold_keypad_suspend,
	.resume		= xgold_keypad_resume,
#endif
};


static int __init xgold_keypad_init(void)
{
	int ret;
	pr_info("%s\n", __func__);
#if defined CONFIG_PLATFORM_DEVICE_PM && !defined CONFIG_PLATFORM_DEVICE_PM_VIRT
	ret = device_state_pm_add_class(&xgold_kpd_pm_class);
	if (ret)
		return ret;
#endif
	ret = platform_driver_register(&xgold_keypad_driver);
	return ret;
}

static void __exit xgold_keypad_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&xgold_keypad_driver);

}

module_init(xgold_keypad_init);
module_exit(xgold_keypad_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("keypad Driver");
