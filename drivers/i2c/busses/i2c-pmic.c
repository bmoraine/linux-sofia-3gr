/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>

#include <sofia/vmm_pmic.h>

/* PMIC I2C registers offsets */
#define I2COVRCTRL_REG		0x24
#define I2COVRDADDR_REG		0x25
#define I2COVROFFSET_REG	0x26
#define I2COVRWRDATA_REG	0x27
#define I2COVRRDDATA_REG	0x28

/* IRQ registers offsets */
#define CHGRIRQ0_REG		0x09
#define MCHGRIRQ0_REG		0x17
#define MIRQLVL1_REG		0x0e

/* Masks and bits */
#define CHGRIRQ0_I2CERROR_M	0x08
#define CHGRIRQ0_I2CRDCMP_M	0x04
#define CHGRIRQ0_I2CWRCMP_M	0x02
#define CHGRIRQ0_I2CALL_M \
	(CHGRIRQ0_I2CERROR_M | CHGRIRQ0_I2CRDCMP_M | CHGRIRQ0_I2CWRCMP_M)
#define I2COVRCTRL_RD_START	0x02
#define I2COVRCTRL_WR_START	0x01

#define BYTE_MASK		0xFF

struct i2c_pmic_platdata {
	unsigned char addr;
	unsigned char irq_addr;
	int nr;
};

struct i2c_pmic_dev {
	struct i2c_adapter adapter;
	struct completion cmd_complete;
	struct mutex mutex;
	struct i2c_pmic_platdata *platdata;
	int err;
};

#define to_i2c_pmic(d) container_of(d, struct i2c_pmic_dev, adapter)

int intel_i2c_pmic_reg_read(u32 dev_addr, u32 reg_addr, u8 *p_reg_val)
{
	u32 vmm_addr, reg_val = 0;
	int ret;

	vmm_addr = ((dev_addr & BYTE_MASK) << 24) | (reg_addr & BYTE_MASK);
	ret = vmm_pmic_reg_read(vmm_addr, &reg_val);
	*p_reg_val = (u8)(reg_val & BYTE_MASK);
	pr_debug("%s: read @%X return %X\n", __func__, reg_addr, reg_val);

	return ret;
}

int intel_i2c_pmic_reg_write(u32 dev_addr, u32 reg_addr, u8 reg_val)
{
	u32 vmm_addr, val = reg_val;

	vmm_addr = ((dev_addr & BYTE_MASK) << 24) | (reg_addr & BYTE_MASK);
	pr_debug("%s: write @%X value %X\n", __func__, reg_addr, val);
	return vmm_pmic_reg_write(vmm_addr, val);
}

static int intel_i2c_pmic_smbus_xfer(struct i2c_adapter *adap, u16 addr,
		  unsigned short flags, char read_write, u8 command, int size,
		  union i2c_smbus_data *data)
{
	struct i2c_pmic_dev *i2c_dev = to_i2c_pmic(adap);
	struct i2c_pmic_platdata *pdata = i2c_dev->platdata;
	int ret;

	dev_dbg(&adap->dev, "--> %s: %s, %d bytes, SA 0x%X, offset 0x%X\n",
			__func__,
			(read_write == I2C_SMBUS_WRITE) ? "WR" : "RD",
			size, addr, command);

	if (size != I2C_SMBUS_BYTE_DATA) {
		dev_err(&adap->dev, "%s: this size is not supported %d\n",
				__func__, size);
		return -EBADMSG;
	}

	/* FIXME spinlock */

	i2c_dev->err = 0;
	reinit_completion(&i2c_dev->cmd_complete);

	/* Set I2C slave address */
	ret = intel_i2c_pmic_reg_write(pdata->addr, I2COVRDADDR_REG,
			addr);
	if (ret)
		goto i2c_xfer_fail;

	/* Set offset to write */
	ret = intel_i2c_pmic_reg_write(pdata->addr, I2COVROFFSET_REG,
			command);
	if (ret)
		goto i2c_xfer_fail;

	if (read_write == I2C_SMBUS_WRITE) {
		/* Set data to write */
		ret = intel_i2c_pmic_reg_write(pdata->addr, I2COVRWRDATA_REG,
				data->byte);
		if (ret)
			goto i2c_xfer_fail;

		/* Trigger write action */
		ret = intel_i2c_pmic_reg_write(pdata->addr, I2COVRCTRL_REG,
				I2COVRCTRL_WR_START);
		if (ret)
			goto i2c_xfer_fail;

		/* Wait for transfer completion */
		ret = wait_for_completion_timeout(&i2c_dev->cmd_complete, 50);
		if (i2c_dev->err) {
			dev_err(&adap->dev, "WR command error\n");
			ret = i2c_dev->err;
			goto i2c_xfer_fail;
		}

		if (ret == 0) {
			dev_err(&adap->dev, "WR command timeout\n");
			ret = -EIO;
			goto i2c_xfer_fail;
		}
	} else {
		/* Trigger read action */
		ret = intel_i2c_pmic_reg_write(pdata->addr, I2COVRCTRL_REG,
				I2COVRCTRL_RD_START);
		if (ret)
			goto i2c_xfer_fail;

		/* Wait for transfer completion */
		ret = wait_for_completion_timeout(&i2c_dev->cmd_complete, 50);

		if (i2c_dev->err) {
			dev_err(&adap->dev, "RD command error\n");
			ret = i2c_dev->err;
			goto i2c_xfer_fail;
		}

		if (ret == 0) {
			dev_err(&adap->dev, "RD command timeout\n");
			ret = -EIO;
			goto i2c_xfer_fail;
		}

		/* Copy read data*/
		ret = intel_i2c_pmic_reg_read(pdata->addr, I2COVRRDDATA_REG,
				&data->byte);
	}

i2c_xfer_fail:
	dev_dbg(&adap->dev, "<-- %s\n", __func__);

	return ret;
}

#if 0
static int intel_i2c_pmic_xfer_msg(struct i2c_adapter *adap,
		struct i2c_msg *msg)
{
	struct i2c_pmic_dev *i2c_dev = i2c_get_adapdata(adap);

	return -EINVAL;
}

static int intel_i2c_pmic_xfer(struct i2c_adapter *adap,
		struct i2c_msg msgs[], int num)
{
	struct i2c_pmic_dev *i2c_dev = to_i2c_pmic(adap);

	return 0;
}
#endif

static u32 intel_i2c_pmic_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA;
}

static struct i2c_algorithm xgold_i2c_virt_algo = {
	.master_xfer = NULL, /*intel_i2c_pmic_xfer,*/
	.smbus_xfer = intel_i2c_pmic_smbus_xfer,
	.functionality = intel_i2c_pmic_func,
};

static struct i2c_pmic_platdata *intel_i2c_pmic_of_get_platdata(
		struct device *dev)
{
	struct i2c_pmic_platdata *platdata;
	struct device_node *np = dev->of_node;
	u32 value;
	int ret;

	platdata = devm_kzalloc(dev, sizeof(*platdata), GFP_KERNEL);
	if (!platdata)
		return ERR_PTR(-ENOMEM);

	/* PMIC device address */
	ret = of_property_read_u32_index(np, "intel,reg", 0, &value);
	if (ret)
		return ERR_PTR(ret);

	if (value > 0xFF)
		return ERR_PTR(-ERANGE);

	platdata->addr = (unsigned char)value;

	/* PMIC device address for IRQ handling */
	ret = of_property_read_u32_index(np, "intel,irq-reg", 0, &value);
	if (ret)
		return ERR_PTR(ret);

	if (value > 0xFF)
		return ERR_PTR(-ERANGE);

	platdata->irq_addr = (unsigned char)value;

	return platdata;
}

static irqreturn_t intel_i2c_pmic_irq_err_handler(int irq, void *dev)
{
	struct i2c_adapter *adap = (struct i2c_adapter *)dev;
	struct i2c_pmic_dev *i2c_dev = to_i2c_pmic(adap);

	i2c_dev->err = -EREMOTEIO;
	complete(&i2c_dev->cmd_complete);

	return IRQ_HANDLED;
}

static irqreturn_t intel_i2c_pmic_irq_handler(int irq, void *dev)
{
	struct i2c_adapter *adap = (struct i2c_adapter *)dev;
	struct i2c_pmic_dev *i2c_dev = to_i2c_pmic(adap);

	dev_dbg(&adap->dev, "Command complete\n");
	complete(&i2c_dev->cmd_complete);

	return IRQ_HANDLED;
}

static struct of_device_id intel_i2c_pmic_of_match[] = {
	{ .compatible = "intel,pmic_i2c",},
	{ },
};

int intel_i2c_pmic_probe(struct platform_device *pdev)
{
	struct i2c_pmic_dev *i2c_dev;
	struct i2c_pmic_platdata *platdata;
	struct i2c_adapter *adap;
	unsigned irq;
	int ret, tries = 0;
	char val = 0;

	dev_info(&pdev->dev, "%s\n", __func__);

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

#ifdef CONFIG_OF
	platdata = intel_i2c_pmic_of_get_platdata(&pdev->dev);
#endif

	if (IS_ERR_OR_NULL(platdata))
		return PTR_ERR(platdata);

	i2c_dev->platdata = platdata;
	init_completion(&i2c_dev->cmd_complete);

	adap = &i2c_dev->adapter;
	adap->owner = THIS_MODULE;
	strlcpy(adap->name, dev_name(&pdev->dev), sizeof(adap->name));
	adap->algo = &xgold_i2c_virt_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;
	adap->nr = platdata->nr;

	/* Request interrupt */
	irq = platform_get_irq_byname(pdev, "rd");
	if (!IS_ERR_VALUE(irq)) {
		ret = devm_request_threaded_irq(&pdev->dev, irq,
				NULL, intel_i2c_pmic_irq_handler,
				IRQF_SHARED | IRQF_ONESHOT | IRQF_NO_SUSPEND,
				"pmic_i2c RD", adap);
		if (ret) {
			dev_err(&pdev->dev, "Cannot request RD IRQ %d",
					irq);
			return ret;
		}
	} else
		dev_err(&pdev->dev, "i2c RD interrupt not found\n");

	irq = platform_get_irq_byname(pdev, "wr");
	if (!IS_ERR_VALUE(irq)) {
		ret = devm_request_threaded_irq(&pdev->dev, irq,
				NULL, intel_i2c_pmic_irq_handler,
				IRQF_SHARED | IRQF_ONESHOT | IRQF_NO_SUSPEND,
				"pmic_i2c WR", adap);
		if (ret) {
			dev_err(&pdev->dev, "Cannot request WR IRQ %d",
					irq);
			return ret;
		}
	} else
		dev_err(&pdev->dev, "i2c WR interrupt not found\n");

	irq = platform_get_irq_byname(pdev, "err");
	if (!IS_ERR_VALUE(irq)) {
		ret = devm_request_threaded_irq(&pdev->dev, irq,
				NULL, intel_i2c_pmic_irq_err_handler,
				IRQF_SHARED | IRQF_ONESHOT | IRQF_NO_SUSPEND,
				"pmic_i2c ERR", adap);
		if (ret) {
			dev_err(&pdev->dev, "Cannot request ERR IRQ %d",
					irq);
			return ret;
		}
	} else
		dev_err(&pdev->dev, "i2c ERR interrupt not found\n");

	/* Unmask PMIC I2C interrupt */
	dev_warn(&pdev->dev, "%s: may apply changes to MCHGIRQ0 register\n",
			__func__);
	intel_i2c_pmic_reg_read(platdata->irq_addr,
			MCHGRIRQ0_REG, &val);
	while ((val & CHGRIRQ0_I2CALL_M) && tries++ < 20) {
		intel_i2c_pmic_reg_write(platdata->irq_addr,
				MCHGRIRQ0_REG, val & ~CHGRIRQ0_I2CALL_M);

		/* read again to ensure Mask is correctly configured */
		intel_i2c_pmic_reg_read(platdata->irq_addr,
				MCHGRIRQ0_REG, &val);
		dev_dbg(&pdev->dev, "%s: MCHGRIRQ0 is 0x%02X\n", __func__, val);
	}

	if (tries >= 20)
		return -EIO;

	dev_dbg(&pdev->dev, "%s MCHGRIRQ0 is 0x%02X\n", __func__, val);

	intel_i2c_pmic_reg_write(platdata->irq_addr,
			CHGRIRQ0_REG, CHGRIRQ0_I2CALL_M);
	intel_i2c_pmic_reg_read(platdata->irq_addr,
			CHGRIRQ0_REG, &val);

	dev_info(&pdev->dev, "%s: CHGRIRQ0 is 0x%02X\n", __func__, val);

	/* FIXME: this part of code should be handled by irqchip level, when
	 * requesting for the PMIC CHRG interrupt, and not in this i2c driver */

	/* Unmask PMIC CHRG interrupt */
	dev_warn(&pdev->dev, "%s: Warning! may apply changes to MIRQLVL1 register\n",
			__func__);

	intel_i2c_pmic_reg_read(platdata->irq_addr,
			MIRQLVL1_REG, &val);
	tries = 0;
	while ((val & 0x20) && tries++ < 20) {
		intel_i2c_pmic_reg_write(platdata->irq_addr,
				MIRQLVL1_REG, val & ~0x20);

		/* read again to ensure Mask is correctly configured */
		intel_i2c_pmic_reg_read(platdata->irq_addr,
				MIRQLVL1_REG, &val);
		dev_dbg(&pdev->dev, "%s: MIRQLVL1 is 0x%02X\n", __func__, val);
	}

	if (tries >= 20)
		return -EIO;

	dev_info(&pdev->dev, "%s MIRQLVL1 is 0x%02X\n", __func__, val);
	/* end of FIXME */

	ret = i2c_add_numbered_adapter(adap);
	if (ret)
		dev_err(&pdev->dev, "Failed to add i2c PMIC bus\n");

	return 0;
}

static int intel_i2c_pmic_remove(struct platform_device *pdev)
{
#if 0
	struct intel_i2c_pmic_dev *i2c_pmic_dev = dev_get_drvdata(&pdev->dev);
	if (i2c_pmic_dev->core_ops->remove)
		return i2c_pmic_dev->core_ops->remove(&pdev->dev);
#endif
	return 0;
}

#ifdef CONFIG_PM
static int intel_i2c_pmic_suspend(struct device *dev)
{
#if 0
	struct intel_i2c_pmic_dev *i2c_pmic_dev = dev_get_drvdata(dev);
	struct intel_i2c_pmic_platdata *platdata = dev_get_platdata(dev);
	int ret = 0;

	if (i2c_pmic_dev->pm_ops && i2c_pmic_dev->pm_ops->suspend)
		ret = i2c_pmic_dev->pm_ops->suspend(dev);

	if (ret)
		return ret;

	ret = device_state_pm_set_state_by_name(dev,
			platdata->pm_platdata->pm_state_D3_name);

	if (ret)
		dev_err(dev, "Error during state transition %s\n",
				platdata->pm_platdata->pm_state_D3_name);

	return ret;
#else
	return 0;
#endif
}

static int intel_i2c_pmic_resume(struct device *dev)
{
#if 0
	struct intel_i2c_pmic_dev *i2c_pmic_dev = dev_get_drvdata(dev);
	struct intel_i2c_pmic_platdata *platdata = dev_get_platdata(dev);
	int ret = 0;

	if (i2c_pmic_dev->pm_ops && i2c_pmic_dev->pm_ops->resume)
		ret = i2c_pmic_dev->pm_ops->resume(dev);

	if (ret)
		return ret;

	ret = device_state_pm_set_state_by_name(dev,
			platdata->pm_platdata->pm_state_D0_name);

	if (ret)
		dev_err(dev, "Error during state transition %s\n",
				platdata->pm_platdata->pm_state_D0_name);
#endif
	return 0;
}

static const struct dev_pm_ops intel_i2c_pmic_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(intel_i2c_pmic_suspend,
			intel_i2c_pmic_resume)
};

#define XGOLD_PLAT_I2C_PM_OPS (&intel_i2c_pmic_pm_ops)
#else
#define XGOLD_PLAT_I2C_PM_OPS NULL
#endif

static struct platform_driver intel_i2c_pmic_driver = {
	.probe = intel_i2c_pmic_probe,
	.remove = intel_i2c_pmic_remove,
	.driver = {
		.name = "intel-i2c-pmic",
		.owner = THIS_MODULE,
		.of_match_table = intel_i2c_pmic_of_match,
		.pm = XGOLD_PLAT_I2C_PM_OPS
	},
};

static int __init intel_i2c_pmic_init_driver(void)
{
	return platform_driver_register(&intel_i2c_pmic_driver);
}

static void __exit intel_i2c_pmic_exit_driver(void)
{
	platform_driver_unregister(&intel_i2c_pmic_driver);
}

subsys_initcall(intel_i2c_pmic_init_driver);
module_exit(intel_i2c_pmic_exit_driver);

MODULE_DESCRIPTION("Intel i2c PMIC driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, intel_i2c_pmic_of_match);
