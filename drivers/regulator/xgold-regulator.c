/*
 * Copyright (c) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#ifdef CONFIG_I2C
#include <linux/i2c.h>
#endif
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>

/*
 * regulator_is_enabled_inv_regmap - standard !is_enabled() for regmap users
 */
int regulator_is_enabled_inv_regmap(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, rdev->desc->enable_reg, &val);
	if (ret != 0)
		return ret;

	return !((val & rdev->desc->enable_mask) != 0);
}

static int xgoldspcu_get_voltage(struct regulator_dev *rdev)
{
	int input_uv;
	if (rdev->supply)
		input_uv = regulator_get_voltage(rdev->supply);
	else
		return -EINVAL;

	return input_uv;

}

static int xgoldspcu_set_voltage(struct regulator_dev *rdev, int min_uv,
				 int max_uv, unsigned int *sel)
{
	unsigned ret;
	if (rdev->supply)
		ret = regulator_set_voltage(rdev->supply, min_uv, max_uv);
	else
		return -EINVAL;

	return ret;

}

static struct regulator_ops xgold_ldo_ops = {
	.list_voltage = regulator_list_voltage_table,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

/* Some LDOs have only one voltage in table */
static struct regulator_ops xgold_ldo_one_voltage_ops = {
	.list_voltage = regulator_list_voltage_table,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

/* Inverse enable/disable callbacks as some spcu registers are inversed */
static struct regulator_ops xgold_spcu_inv_ops = {
	.enable = regulator_disable_regmap,
	.disable = regulator_enable_regmap,
	.is_enabled = regulator_is_enabled_inv_regmap,
	.get_voltage = xgoldspcu_get_voltage,
	.set_voltage = xgoldspcu_set_voltage,
};

static struct regulator_ops xgold_spcu_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.get_voltage = xgoldspcu_get_voltage,
	.set_voltage = xgoldspcu_set_voltage,
};

/* Regmap to access PMIC over I2C */
struct regmap_config regmap_config_pmic_i2c = {
	.name = "pmic",
	.reg_bits = 8,
	.val_bits = 8,
	.reg_stride = 1,
};

/* Regmap to access PMU registers directly */
struct regmap_config regmap_config_pmu = {
	.name = "pmu",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_format_endian = REGMAP_ENDIAN_DEFAULT,
};

/* Regmap to access SPCU */
struct regmap_config regmap_config_spcu = {
	.name = "spcu",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_format_endian = REGMAP_ENDIAN_DEFAULT,
};

static struct regmap *regmap_pmu;
static unsigned id;

static int xgold_reg_generic_probe(struct device *dev, void __iomem
		*spcu_hw_base)
{
	struct device_node *np = dev->of_node;
	struct device_node *np_regs;
	struct regmap *regmap_spcu = NULL;
	unsigned ret = 0;


	if (of_device_is_compatible(np, "intel,xgold-reg")) {
		struct device_node *reg_node = NULL;

		np_regs = of_parse_phandle(np, "intel,regulators", 0);
		if (!np_regs)
			np_regs = np;

		/* Map SPCU registers using regmap */
		regmap_config_spcu.name = "spcu";

		regmap_spcu =
			regmap_init_mmio(dev, spcu_hw_base,
					&regmap_config_spcu);
		if (IS_ERR(regmap_spcu)) {
			ret = PTR_ERR(regmap_spcu);
			dev_err(dev, "regmap init regulator failed: %d\n", ret);
			return ret;
		}

		for_each_available_child_of_node(np_regs, reg_node) {
			struct platform_device *pdev_ldo;
			const char *name;
			name =
			    of_get_property(reg_node, "regulator-name", NULL);
			pdev_ldo = platform_device_alloc(name, id);
			pdev_ldo->dev.parent = dev;
			pdev_ldo->dev.of_node = reg_node;
			pdev_ldo->dev.platform_data = (void *)regmap_spcu;
			platform_device_add(pdev_ldo);
			id++;
		}
		return ret;
	}
	return ret;
}

#ifdef CONFIG_I2C
static int xgoldreg_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	int ret = 0;
	if (!strcmp(np->name, "regulator")) {
		void __iomem *spcu_hw_base;
		struct device_node *spcu_np;

		regmap_pmu = regmap_init_i2c(client, &regmap_config_pmic_i2c);
		if (IS_ERR(regmap_pmu)) {
			ret = PTR_ERR(regmap_pmu);
			dev_err(&client->dev,
				"regmap init regulator failed: %d\n", ret);
			return ret;
		}

		spcu_np = of_parse_phandle(np, "intel,csc-phys", 0);
		if (!spcu_np)
			return PTR_ERR(spcu_np);

		spcu_hw_base =  of_iomap(spcu_np, 0);
		if (IS_ERR_OR_NULL(spcu_hw_base))
			return PTR_ERR(spcu_hw_base);

		ret = xgold_reg_generic_probe(&client->dev, spcu_hw_base);
	}
	return ret;

}

static int xgoldreg_i2c_remove(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	if (strcmp(np->name, "regulator"))
		regulator_unregister(i2c_get_clientdata(client));
	return 0;
}

#endif

static int xgoldreg_platform_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct regulator_consumer_supply *rsupply = NULL;
	struct regulator_init_data *initdata;
	struct regulator_config *config;
	struct regulator_dev *rdev;
	struct regulator_desc *desc = NULL;
	struct device_node *consumer_node = NULL;
	struct regulation_constraints *constraints;
	struct regmap *regmap_spcu;
	int has_switch = 0;
	unsigned ret = 0;
	u32 *table = NULL;
	u32 array[3];
	u32 mask = 0;
	int i = 0;

	/* Parse regulator node and register regulators */
	if (of_device_is_compatible(np, "intel,xgold-reg")) {
		void __iomem *spcu_hw_base;
		void __iomem *pmu_hw_base;
		struct resource *res;
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
				"pmu");
		if (!res) {
			dev_err(&pdev->dev, "could not find pmu base address\n");
			return -EINVAL;
		}

		pmu_hw_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR_OR_NULL(pmu_hw_base))
			return PTR_ERR(pmu_hw_base);

		/* Map PMU registers using regmap */
		regmap_pmu =
		    regmap_init_mmio(&pdev->dev, pmu_hw_base,
				     &regmap_config_pmu);
		if (IS_ERR(regmap_pmu)) {
			ret = PTR_ERR(regmap_pmu);
			dev_err(&pdev->dev,
				"regmap init regulator failed: %d\n", ret);
			return ret;
		}

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
				"spcu");
		if (!res) {
			dev_err(&pdev->dev, "could not find spcu base address\n");
			return -EINVAL;
		}

		spcu_hw_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR_OR_NULL(spcu_hw_base))
			return PTR_ERR(spcu_hw_base);

		ret = xgold_reg_generic_probe(&pdev->dev, spcu_hw_base);

		return ret;
	}

	/* Allocate regulator config */
	config = devm_kzalloc(&pdev->dev,
			sizeof(struct regulator_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	/* Allocate regulator descriptor */
	desc = devm_kzalloc(&pdev->dev,
			sizeof(struct regulator_desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	/* Check node */
	if (!of_device_is_compatible(np, "intel,ldo")) {
		dev_err(&pdev->dev, "device node is not compatible with ldo\n");
		return -EINVAL;
	}
	/* Get regulator information from Device Tree */
	initdata = of_get_regulator_init_data(&pdev->dev, np);
	if (!initdata)
		return -ENOMEM;

	constraints = &initdata->constraints;
	regmap_spcu = (struct regmap *)pdev->dev.platform_data;

	/* Get enable register properties */
	ret = of_property_read_u32_array(np, "intel,enable", array, 3);
	if (ret != 0) {
		/*
		 * Some regulators could be always on but they need
		 * regulator-always-on property
		 */
		if (!of_find_property(np, "regulator-always-on", NULL)) {
			dev_err(&pdev->dev,
				"read intel,enable property failed: %d\n", ret);
			return -EINVAL;
		}
	} else {
		for (i = 0; i < array[2]; i++)
			mask |= 1 << i;

		desc->enable_mask = mask << array[1];
		desc->enable_reg = array[0];
	}

	/* Check if regulator has switch */
	ret = of_property_read_u32(np, "intel,has-switch", &has_switch);
	if (ret != 0) {
		dev_err(&pdev->dev, "read has-switch property failed: %d\n",
			ret);
		return ret;
	}

	/* Register power domain as regulator only if it has a switch */
	if (has_switch) {
		struct device_node *supply_node;
		struct regulator_init_data *initdata_supply = NULL;
		struct regulation_constraints *constraints_supply;
		int noinv = 0;

		supply_node = of_parse_phandle(np, "supply", 0);
		/* Get supply information from Device Tree */
		if (supply_node != NULL) {
			initdata_supply =
			    of_get_regulator_init_data(&pdev->dev, supply_node);
			constraints = &initdata->constraints;
			constraints_supply = &initdata_supply->constraints;
			constraints->min_uV = constraints_supply->min_uV;
			constraints->max_uV = constraints_supply->max_uV;
			constraints->valid_ops_mask =
			    constraints_supply->valid_ops_mask;
		}
		desc->supply_name =
		    of_get_property(supply_node, "regulator-name", NULL);
		of_property_read_u32(np, "intel,noinv", &noinv);
		if (noinv)
			desc->ops = &xgold_spcu_ops;
		else
			desc->ops = &xgold_spcu_inv_ops;

		config->regmap = regmap_spcu;
		goto register_ldo;
	}

	/* Get voltage table */
	ret = of_property_read_u32(np, "intel,table-len", &desc->n_voltages);
	if (ret != 0) {
		dev_err(&pdev->dev, "read table-len property failed: %d\n",
			ret);
		return ret;
	}

	table = devm_kzalloc(&pdev->dev,
			desc->n_voltages * sizeof(u32), GFP_KERNEL);
	if (!table)
		return -ENOMEM;
	ret =
	    of_property_read_u32_array(np, "intel,table", table,
				       desc->n_voltages);
	if (ret != 0) {
		dev_err(&pdev->dev, "read table property failed: %d\n", ret);
		return ret;
	}

	/* Handle the case where there is no register to change voltage */
	if (desc->n_voltages != 1) {
		/* Get voltage register properties */
		ret = of_property_read_u32_array(np, "intel,voltage", array, 3);
		if (ret != 0) {
			dev_err(&pdev->dev, "read voltage failed: %d\n", ret);
			return ret;
		}
		mask = 0;
		for (i = 0; i < array[2]; i++)
			mask |= 1 << i;

		desc->vsel_mask = mask << array[1];
		desc->vsel_reg = array[0];
		desc->ops = &xgold_ldo_ops;
	} else {
		desc->ops = &xgold_ldo_one_voltage_ops;
	}

	config->regmap = regmap_pmu;
	desc->type = REGULATOR_VOLTAGE;
	desc->volt_table = table;

register_ldo:
	/* Get number of consumers */
	for (i = 0;
	     (consumer_node =
	      of_parse_phandle(np, "intel,consumer", i)) != NULL; i++) {
		of_node_put(consumer_node);
	}
	if (i < 0)
		goto no_consumer;
	rsupply =
	    devm_kzalloc(&pdev->dev,
		i * sizeof(struct regulator_consumer_supply), GFP_KERNEL);
	if (!rsupply)
		return -ENOMEM;

	initdata->num_consumer_supplies = i;
	initdata->consumer_supplies = rsupply;

	for (i = 0; (consumer_node =
				of_parse_phandle
				(np, "intel,consumer", i)) != NULL; i++) {
		rsupply->dev_name = consumer_node->name;
		rsupply->supply = consumer_node->name;
		of_node_put(consumer_node);
		dev_dbg(&pdev->dev, "register consumer: %s\n",
			rsupply->supply);
		rsupply++;
	}

no_consumer:
	desc->id = pdev->id;
	desc->owner = THIS_MODULE;
	desc->name = constraints->name;

	/* Register regulator config */
	config->dev = &pdev->dev;
	config->init_data = initdata;
	config->of_node = np;
	rdev = regulator_register(desc, config);
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev,
			"%s(): regulator register failed for %s\n",
			__func__, desc->name);
		ret = PTR_ERR(rdev);
	}
	return ret;
}

static int xgoldreg_platform_remove(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	if (strcmp(np->name, "regulator"))
		regulator_unregister(platform_get_drvdata(pdev));

	return 0;
}

static const struct of_device_id xgold_reg_of_match[] = {
	{
	 .compatible = "intel,ldo",
	 },
	{
	 .compatible = "intel,xgold-reg",
	 },
	{}
};

MODULE_DEVICE_TABLE(of, xgold_reg_of_match);

static struct platform_driver xgoldreg_driver = {
	.probe = xgoldreg_platform_probe,
	.remove = xgoldreg_platform_remove,
	.driver = {
		   .name = "xgold-reg",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(xgold_reg_of_match),
		   },
};

#ifdef CONFIG_I2C
static const struct i2c_device_id xgold_reg_i2c_id[] = {
	{"intel,xgold-reg", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, xgold_reg_i2c_id);

static struct i2c_driver xgoldreg_i2c_driver = {
	.probe = xgoldreg_i2c_probe,
	.remove = xgoldreg_i2c_remove,
	.id_table = xgold_reg_i2c_id,
	.driver = {
		   .name = "xgold-reg",
		   .owner = THIS_MODULE,
	},
};
#endif

static int __init xgoldreg_init(void)
{
	unsigned ret = 0;
	ret = platform_driver_register(&xgoldreg_driver);
#ifdef CONFIG_I2C
	ret = i2c_add_driver(&xgoldreg_i2c_driver);
#endif
	return ret;
}

subsys_initcall(xgoldreg_init);

static void __exit xgoldreg_exit(void)
{
	platform_driver_unregister(&xgoldreg_driver);
#ifdef CONFIG_I2C
	i2c_del_driver(&xgoldreg_i2c_driver);
#endif
}

module_exit(xgoldreg_exit)

MODULE_ALIAS("platform:xgold-reg");
MODULE_DESCRIPTION("xgold regulator driver");
MODULE_LICENSE("GPL");
