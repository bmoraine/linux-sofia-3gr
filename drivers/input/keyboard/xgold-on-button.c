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
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/input/matrix_keypad.h>

#define PROP_AGOLD620_ON_BUTTON "intel,agold620,on-button"
#define PROP_PMIC_ON_BUTTON "intel,pmic,on-button"

#define MAX_ON_BUTTON_IRQ 2
struct xgold_on_button_pdata {
	uint32_t irq[MAX_ON_BUTTON_IRQ];
	struct input_dev *input_dev;
	struct platform_device *pdev;
	struct matrix_keymap_data *keymap_data;
};

static uint16_t keycodes[] = { KEY_POWER };

static irqreturn_t on_button_1_isr(int32_t irq, void *d)
{
	struct xgold_on_button_pdata *data = (struct xgold_on_button_pdata *)d;
	if (data->input_dev) {
		input_report_key(data->input_dev, KEY_POWER, 1);
		dev_dbg(&data->pdev->dev, "Power key pressed - event reported for %d, %d\n",
			KEY_POWER, 1);
		input_sync(data->input_dev);
	}
	return IRQ_HANDLED;
}

static irqreturn_t on_button_0_isr(int32_t irq, void *d)
{
	struct xgold_on_button_pdata *data = (struct xgold_on_button_pdata *)d;
	if (data->input_dev) {
		input_report_key(data->input_dev, KEY_POWER, 0);
		dev_dbg(&data->pdev->dev, "Power key released - event reported for %d, %d\n",
			KEY_POWER, 0);
		input_sync(data->input_dev);
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int32_t xgold_on_button_suspend(struct platform_device *pdev,
					  pm_message_t state)
{
	struct device *dev = &pdev->dev;
	struct xgold_on_button_pdata *data = dev_get_platdata(dev);
	struct input_dev *input_dev = data->input_dev;
	uint32_t i;
	dev_dbg(dev, "%s\n", __func__);
	mutex_lock(&input_dev->mutex);
	if (device_may_wakeup(dev))
		for (i = 0; i < MAX_ON_BUTTON_IRQ; i++)
			if (data->irq[i])
				enable_irq_wake(data->irq[i]);
	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int32_t xgold_on_button_resume(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct xgold_on_button_pdata *data = dev_get_platdata(dev);
	struct input_dev *input_dev = data->input_dev;
	uint32_t i;
	dev_dbg(dev, "%s\n", __func__);
	mutex_lock(&input_dev->mutex);
	if (device_may_wakeup(dev))
		for (i = 0; i < MAX_ON_BUTTON_IRQ; i++)
			if (data->irq[i])
				disable_irq_wake(data->irq[i]);
	mutex_unlock(&input_dev->mutex);
	return 0;
}
#endif

#define ON_BUTTON_0 "on_button0"
#define ON_BUTTON_1 "on_button1"
#define ON_BUTTON_NAME "xgold-on-button"

static int32_t xgold_on_button_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct xgold_on_button_pdata *data;
	int32_t error = 0;
	uint32_t irq_count = 0;

	dev_dbg(dev, "%s\n", __func__);

	/* Alloc platform data */
	data = devm_kzalloc(dev, sizeof(struct xgold_on_button_pdata),
			GFP_KERNEL);
	if (!data) {
		dev_err(dev, "Alloc failed\n");
		error = -ENOMEM;
		goto fail_alloc;
	}
	data->pdev = pdev;
	/* Set platdata to device */
	dev->platform_data = data;
	/* Register PMU on_button_0 interrupt */
	data->irq[irq_count] = platform_get_irq_byname(pdev, ON_BUTTON_0);
	if (!IS_ERR_VALUE(data->irq[irq_count])) {
		if (devm_request_irq(dev, data->irq[irq_count], on_button_0_isr,
			IRQF_SHARED | IRQF_NO_SUSPEND, ON_BUTTON_0, data)) {
			dev_err(dev, "setup irq%d failed\n",
				data->irq[irq_count]);
			error = -EINVAL;
			goto fail_irq0;
		}
		irq_count++;
	} else {
		dev_dbg(dev, "on_button_0 irq not found\n");
	}
	/* Register on_button_1 interrupt */
	data->irq[irq_count] = platform_get_irq_byname(pdev, ON_BUTTON_1);
	if (!IS_ERR_VALUE(data->irq[irq_count])) {
		if (devm_request_irq(dev, data->irq[irq_count], on_button_1_isr,
			IRQF_SHARED | IRQF_NO_SUSPEND, ON_BUTTON_1, data)) {
			dev_err(dev, "setup irq%d failed\n",
				data->irq[irq_count]);
			error = -EINVAL;
			goto fail_irq1;
		}
		irq_count++;
	} else {
		dev_dbg(dev, "on_button_1 irq not found\n");
	}
	/* Check irq_count */
	if (!irq_count || irq_count > MAX_ON_BUTTON_IRQ) {
		dev_err(dev, "Wrong IRQ number (%d) registered\n", irq_count);
		error = -EINVAL;
		goto fail_irq_nr;
	}
	/* Allocate input */
	data->input_dev = devm_input_allocate_device(&pdev->dev);
	if (!data->input_dev) {
		dev_err(&pdev->dev, "failed to allocate input\n");
		error = -EINVAL;
		goto fail_input;
	}
	data->input_dev->name = ON_BUTTON_NAME;
	data->input_dev->dev.parent = &pdev->dev;
	data->input_dev->id.bustype = BUS_HOST;
	data->input_dev->evbit[0] = BIT_MASK(EV_KEY);
	data->input_dev->keycode = keycodes;
	data->input_dev->keycodesize = sizeof(uint16_t);
	data->input_dev->keycodemax = ARRAY_SIZE(keycodes);
	if (matrix_keypad_build_keymap(data->keymap_data, NULL, 1, 1,
				keycodes, data->input_dev)) {
		dev_err(&pdev->dev, "failed to build keymap\n");
		error = -EINVAL;
		goto fail_input;
	}

	input_set_capability(data->input_dev, EV_MSC, MSC_SCAN);
	input_set_drvdata(data->input_dev, data);
	/* Register input */
	if (input_register_device(data->input_dev)) {
		dev_err(&pdev->dev, "failed to register input device\n");
		error = -EINVAL;
		goto fail_reg_input;
	}
	device_init_wakeup(&pdev->dev, true);
	return error;

fail_reg_input:
	input_free_device(data->input_dev);
fail_input:
	free_irq(data->irq[1], data);
fail_irq1:
	free_irq(data->irq[0], data);
fail_irq0:
fail_irq_nr:
	kfree(data);
fail_alloc:
	return error;
}

static int32_t xgold_on_button_remove(struct platform_device *pdev)
{
	struct xgold_on_button_pdata *data = platform_get_drvdata(pdev);
	uint32_t i;
	for (i = 0; i < MAX_ON_BUTTON_IRQ; i++)
		free_irq(data->irq[i], data);
	input_unregister_device(data->input_dev);
	kfree(data);
	return 0;
}

static const struct of_device_id xgold_on_button_dt_match[] = {
	{ .compatible = "intel,agold620,on-button" },
	{ .compatible = "intel,pmic,on-button" },
	{},
};
MODULE_DEVICE_TABLE(of, xgold_on_button_dt_match);

static struct platform_driver xgold_on_button_driver = {
	.driver	= {
		.name = ON_BUTTON_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(xgold_on_button_dt_match),
	},
	.probe = xgold_on_button_probe,
	.remove = xgold_on_button_remove,
#ifdef CONFIG_PM
	.suspend = xgold_on_button_suspend,
	.resume = xgold_on_button_resume,
#endif
};

static int32_t __init xgold_on_button_init(void)
{
	return platform_driver_register(&xgold_on_button_driver);
}

static void __exit xgold_on_button_exit(void)
{
	platform_driver_unregister(&xgold_on_button_driver);
}

module_init(xgold_on_button_init);
module_exit(xgold_on_button_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("ON button driver");
