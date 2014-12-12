#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>


struct regulator_device_pm_data {
	struct regulator_desc desc;
	struct regulator_dev *dev;
	int current_state;
	int disabled_state;
	int enabled_state;
	const char **state_names;
	bool is_enabled;
};

struct regulator_device_pm_config {
	const char *supply_name;
	const char *class_name;
	const char *user_name;
	const char **state_names;
	const int  *voltages;
	int n_voltages;
	struct regulator_init_data *init_data;
};

static int device_pm_list_voltage(struct regulator_dev *rdev, unsigned sel)
{
	struct regulator_device_pm_data *data = rdev_get_drvdata(rdev);

	return data->desc.volt_table[sel];
}

static int _device_pm_set_state(struct regulator_dev *rdev, unsigned sel)
{
	struct regulator_device_pm_data *data = rdev_get_drvdata(rdev);
	const char *state_name;
	int ret;

	state_name = data->state_names[sel];
	dev_dbg(&rdev->dev, "Set state %s state\n", state_name);

	ret = device_state_pm_set_state_by_name(&rdev->dev, state_name);
	if (ret) {
		dev_err(&rdev->dev,
			"Error while setting %s state\n", state_name);
		return ret;
	}

	data->current_state = sel;

	if (data->disabled_state == sel)
		data->is_enabled = false;
	else {
		data->is_enabled = true;
		data->enabled_state = sel;
	}

	return ret;
}

static int device_pm_set_voltage_sel(struct regulator_dev *rdev, unsigned sel)
{
	struct regulator_device_pm_data *data = rdev_get_drvdata(rdev);
	int ret;

	if (sel > data->desc.n_voltages)
		return -EINVAL;

	/* If regulator not yet enabled, don't call the PM */
	if (0 >= rdev->use_count) {
		data->current_state = sel;
		return 0;
	}
	ret = _device_pm_set_state(rdev, sel);

	return ret;
}

static int device_pm_get_voltage_sel(struct regulator_dev *rdev)
{
	struct regulator_device_pm_data *data = rdev_get_drvdata(rdev);

	return data->desc.volt_table[data->current_state];
}

static int device_pm_is_enabled(struct regulator_dev *rdev)
{
	struct regulator_device_pm_data *data = rdev_get_drvdata(rdev);

	return (int) data->is_enabled;
}

static int device_pm_enable(struct regulator_dev *rdev)
{
	struct regulator_device_pm_data *data = rdev_get_drvdata(rdev);

	return _device_pm_set_state(rdev, data->enabled_state);
}

static int device_pm_disable(struct regulator_dev *rdev)
{
	struct regulator_device_pm_data *data = rdev_get_drvdata(rdev);

	return _device_pm_set_state(rdev, data->disabled_state);
}

static struct regulator_ops device_pm_voltage_ops = {
	.get_voltage_sel = device_pm_get_voltage_sel,
	.list_voltage = device_pm_list_voltage,
	.set_voltage_sel = device_pm_set_voltage_sel,
	.is_enabled = device_pm_is_enabled,
	.enable = device_pm_enable,
	.disable = device_pm_disable,
};

#define PROP_STATE_NAMES "intel,states"
#define PROP_VOLTAGES "intel,voltages"
#define PROP_CLASS_NAME "intel,class_name"
#define PROP_USER_NAME "intel,user_name"

static struct regulator_device_pm_config *
of_get_regulator_device_pm_config(struct device *dev)
{
	struct regulator_device_pm_config *config;
	struct device_node *np = dev->of_node;
	struct regulator_init_data *init_data;
	struct property *pp;
	int ret = 0, n_states, i;

	config = devm_kzalloc(dev, sizeof(struct regulator_device_pm_config),
								 GFP_KERNEL);
	if (!config)
		return ERR_PTR(-ENOMEM);

	config->init_data = of_get_regulator_init_data(dev, dev->of_node);
	if (!config->init_data)
		return ERR_PTR(-EINVAL);

	init_data = config->init_data;
	config->supply_name = init_data->constraints.name;

	config->n_voltages = 0;
	pp = of_find_property(np, PROP_VOLTAGES, &config->n_voltages);
	if (!pp) {
		dev_err(dev, "no voltages table defined!, length %d",
				config->n_voltages);
		return ERR_PTR(-EINVAL);
	}

	config->voltages = devm_kzalloc(dev, config->n_voltages, GFP_KERNEL);
	if (config->voltages == NULL)
		return ERR_PTR(-ENOMEM);

	config->n_voltages /= sizeof(int);
	ret = of_property_read_u32_array(np, PROP_VOLTAGES,
				(uint *) config->voltages, config->n_voltages);
	if (ret) {
		dev_err(dev, "Error while reading the voltages table\n");
		return ERR_PTR(-EINVAL);
	}

	n_states = of_property_count_strings(np, PROP_STATE_NAMES);
	config->state_names = devm_kzalloc(dev, n_states * sizeof(char *),
								GFP_KERNEL);
	for (i = 0; i < n_states; i++) {
		ret = of_property_read_string_index(np, PROP_STATE_NAMES,
				  i, &config->state_names[i]);
		if (ret) {
			dev_err(dev, "Error while reading state names table");
			return ERR_PTR(-EINVAL);
		}

	};

	ret = of_property_read_string(np, PROP_CLASS_NAME, &config->class_name);
	if (ret) {
		dev_err(dev, "Property %s missing\n", PROP_CLASS_NAME);
		return ERR_PTR(-EINVAL);
	}

	ret = of_property_read_string(np, PROP_USER_NAME, &config->user_name);
	if (ret) {
		dev_err(dev, "Property %s missing\n", PROP_USER_NAME);
		return ERR_PTR(-EINVAL);
	}

	return config;
}

static int reg_device_pm_probe(struct platform_device *pdev)
{
	struct regulator_device_pm_config *config;
	struct regulator_device_pm_data *drvdata;
	struct regulator_dev *rdev;
	struct regulator_config cfg = { };
	int i, ret;

	if (pdev->dev.of_node) {
		config = of_get_regulator_device_pm_config(&pdev->dev);
		if (IS_ERR(config))
			return PTR_ERR(config);
	} else {
		config = pdev->dev.platform_data;
	}

	if (!config)
		return -ENOMEM;

	drvdata = devm_kzalloc(&pdev->dev,
			sizeof(struct regulator_device_pm_data), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->desc.name = kstrdup(config->supply_name, GFP_KERNEL);
	if (drvdata->desc.name == NULL) {
		dev_err(&pdev->dev, "Failed to allocate supply name\n");
		return -ENOMEM;
	}
	drvdata->desc.type = REGULATOR_VOLTAGE;
	drvdata->desc.owner = THIS_MODULE;
	drvdata->desc.ops = &device_pm_voltage_ops;
	drvdata->desc.n_voltages = config->n_voltages;
	drvdata->desc.volt_table = config->voltages;
	drvdata->state_names = config->state_names;
	drvdata->current_state = 0;
	drvdata->is_enabled = false;
	for (i = 0; i < config->n_voltages; i++) {
		if (config->voltages[i] == 0) {
			drvdata->disabled_state = i;
			drvdata->current_state = i;
			break;
		}
	}

	cfg.dev = &pdev->dev;
	cfg.init_data = config->init_data;
	cfg.driver_data = drvdata;
	cfg.of_node = pdev->dev.of_node;

	rdev = regulator_register(&drvdata->desc, &cfg);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(&pdev->dev, "Failed to register regulator: %d\n", ret);
		goto err;
	}

	drvdata->dev = rdev;

	platform_set_drvdata(pdev, drvdata);
	ret = device_state_pm_set_class(&rdev->dev, config->user_name);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register %s to the class\n",
							config->user_name);
		goto err_set_class;
	}

	return 0;
err_set_class:
	regulator_unregister(rdev);
err:
	kfree(drvdata->desc.name);
	return ret;
}

static int reg_device_pm_remove(struct platform_device *pdev)
{
	struct regulator_device_pm_data *drvdata = platform_get_drvdata(pdev);

	regulator_unregister(drvdata->dev);
	kfree(drvdata->desc.name);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id regulator_device_pm_of_match[] = {
	{ .compatible = "intel,regulator-device-pm", },
	{},
};
MODULE_DEVICE_TABLE(of, regulator_device_of_match);
#endif

static struct platform_driver regulator_device_pm_driver = {
	.probe		= reg_device_pm_probe,
	.remove		= reg_device_pm_remove,
	.driver		= {
		.name		= "reg-device-pm",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(regulator_device_pm_of_match),
	},
};

static int __init regulator_device_pm_init(void)
{
	pr_info("Registering device PM controlled regulator\n");
	return platform_driver_register(&regulator_device_pm_driver);
}
subsys_initcall(regulator_device_pm_init);

static void __exit regulator_device_pm_exit(void)
{
	platform_driver_unregister(&regulator_device_pm_driver);
}
module_exit(regulator_device_pm_exit);

MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("regulator managed by device pm");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:reg-device-pm");
