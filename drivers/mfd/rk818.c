/*
 * Regulator driver for rk818 PMIC chip for rk31xx
 *
 * Based on rk818.c that is work by zhangqing<zhangqing@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/rk818.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <sofia/vmm_pmic.h>

#if 0
#define DBG(x...)	pr_info(x)
#else
#define DBG(x...)
#endif
#define PM_CONTROL

struct rk818 *g_rk818;

static struct mfd_cell rk818s[] = {
	{
	 .name = "rk818-rtc",
	 },

	{
	 .name = "rk818-battery",
	 },
};

#define RK818_VMM_I2C_ADDR (0x1c << 24)
#define PARAMETER_NO_USE 1

int rk818_i2c_read(struct rk818 *rk818, u8 reg, int count, u8 *dest)
{
	int err = 0;
	uint32_t vmm_reg_address = RK818_VMM_I2C_ADDR | reg;

	err = vmm_pmic_reg_read_by_range(vmm_reg_address, dest, count);

	if (err < 0)
		pr_err("Read 0x%02x failed\n", reg);
	else
		DBG("reg read 0x%02x -> 0x%02x\n", reg, *dest);

	return err;
}
EXPORT_SYMBOL_GPL(rk818_i2c_read);

int rk818_i2c_write(struct rk818 *rk818, u8 reg, int count, u8 src)
{
	int err = 0;
	uint32_t vmm_reg_address = RK818_VMM_I2C_ADDR | reg;

	err = vmm_pmic_reg_write_by_range(vmm_reg_address, &src, count);

	if (err < 0)
		pr_err("Write for reg 0x%x failed\n", reg);

	return err;
}
EXPORT_SYMBOL_GPL(rk818_i2c_write);

u8 rk818_reg_read(struct rk818 *rk818, u8 reg)
{
	u8 val = 0;
	uint32_t vmm_reg_address = RK818_VMM_I2C_ADDR | reg;
	vmm_pmic_reg_read_by_range(vmm_reg_address, &val, 1);

	DBG("reg read 0x%02x -> 0x%02x\n", reg, val);

	return val;
}
EXPORT_SYMBOL_GPL(rk818_reg_read);

int rk818_reg_write(struct rk818 *rk818, u8 reg, u8 val)
{
	int err = 0;
	uint32_t vmm_reg_address = RK818_VMM_I2C_ADDR | reg;

	err = vmm_pmic_reg_write_by_range(vmm_reg_address, &val, 1);

	if (err < 0)
		pr_err("Write for reg 0x%x failed\n", reg);

	return err;
}
EXPORT_SYMBOL_GPL(rk818_reg_write);

int rk818_set_bits(struct rk818 *rk818, u8 reg, u8 mask, u8 val)
{
	u8 tmp;
	int ret;

	mutex_lock(&rk818->io_lock);

	ret = rk818_i2c_read(rk818, reg, 1, &tmp);
	DBG("1 reg read 0x%02x -> 0x%02x\n", reg, tmp);
	tmp = (tmp & ~mask) | val;
	if (ret == 0) {
		ret = rk818_i2c_write(rk818, reg, 1, tmp);
		DBG("reg write 0x%02x -> 0x%02x\n", reg, val);
	}
	rk818_i2c_read(rk818, reg, 1, &tmp);

	DBG("2 reg read 0x%02x -> 0x%02x\n", reg, tmp);

	mutex_unlock(&rk818->io_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(rk818_set_bits);

int rk818_clear_bits(struct rk818 *rk818, u8 reg, u8 mask)
{
	u8 data;
	int err;

	mutex_lock(&rk818->io_lock);

	err = rk818_i2c_read(rk818, reg, 1, &data);
	if (err < 0) {
		dev_err(rk818->dev, "read from reg %x failed\n", reg);
		goto out;
	}
	data &= ~mask;
	err = rk818_i2c_write(rk818, reg, 1, data);
	if (err < 0)
		dev_err(rk818->dev, "write to reg %x failed\n", reg);

out:
	mutex_unlock(&rk818->io_lock);
	return err;
}
EXPORT_SYMBOL_GPL(rk818_clear_bits);

static inline int rk818_set_pinctrl_state(struct pinctrl *pinctrl,
					  struct pinctrl_state *state)
{
	int ret = 0;

	if (!IS_ERR(state)) {
		ret = pinctrl_select_state(pinctrl, state);
		if (ret)
			pr_err("%d:could not set pins\n", __LINE__);
	}

	return ret;
}

#if 1
static ssize_t rk818_test_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	u32 getdata[8] = { 0 };
	u16 regAddr;
	u8 data;
	int ret;
	char cmd;
	const char *buftmp = buf;
	struct rk818 *rk818 = g_rk818;
	/**
	* W Addr(8Bit) regAddr(8Bit) data0(8Bit) data1(8Bit) data2(8Bit) data3(8Bit)
	* data can be less than 4 byte
	* R regAddr(8Bit)
	* C gpio_name(poweron/powerhold/sleep/boot0/boot1) value(H/L)
	*/
	regAddr = (u16) (getdata[0] & 0xff);
	if (strncmp(buf, "start", 5) == 0) {
		DBG("start\n");
	} else if (strncmp(buf, "stop", 4 == 0)) {
		DBG("stop\n");
	} else {
		ret = sscanf(buftmp, "%c ", &cmd);
		pr_info("------zhangqing: get cmd = %c\n", cmd);
		switch (cmd) {

		case 'w':
			ret = sscanf(buftmp, "%c %x %x ", &cmd, &getdata[0],
			       &getdata[1]);
			regAddr = (u16) (getdata[0] & 0xff);
			data = (u8) (getdata[1] & 0xff);
			pr_info("get value = %x\n", data);

			rk818_i2c_write(rk818, regAddr, 1, data);
			rk818_i2c_read(rk818, regAddr, 1, &data);
			pr_info("%x   %x\n", getdata[1], data);

			break;

		case 'r':
			ret = sscanf(buftmp, "%c %x ", &cmd, &getdata[0]);
			pr_info("CMD : %c %x\n", cmd, getdata[0]);

			regAddr = (u16) (getdata[0] & 0xff);
			rk818_i2c_read(rk818, regAddr, 1, &data);
			pr_info("%x %x\n", getdata[0], data);

			break;

		case 't':
			ret = sscanf(buftmp, "%c %x ", &cmd, &getdata[0]);
			pr_info("CMD : %c %x\n", cmd, getdata[0]);
			if (getdata[0] == 1) {
				rk818->bat_test_mode = 1;
				pr_info(
					"You open battery test mode, capacity will always report fixed value\n"
					);
			} else{
				rk818->bat_test_mode = 0;
				pr_info("close battery test mode\n");
			}
			break;

		default:
			pr_info("Unknown command\n");
			break;
		}
	}
	return n;

}

static ssize_t rk818_test_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	buf = "hello";
	return sprintf(s, "%s\n", buf);
}

static struct kobject *rk818_kobj;
struct rk818_attribute {
	struct attribute attr;
	 ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf);
	 ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n);
};

static struct rk818_attribute rk818_attrs[] = {
	/*node_name   permision   show_func   store_func */
	__ATTR(rk818_test, S_IRUGO | S_IWUSR, rk818_test_show,
	       rk818_test_store),
};
#endif

#ifdef CONFIG_OF
static struct of_device_id rk818_of_match[] = {
	{.compatible = "rockchip,rk818"},
	{},
};

MODULE_DEVICE_TABLE(of, rk818_of_match);
#endif

#ifdef CONFIG_OF
static struct rk818_board *rk818_parse_dt(struct rk818 *rk818)
{
	struct rk818_board *pdata = 0;
	struct device_node *rk818_pmic_np;

	rk818_pmic_np = of_node_get(rk818->dev->of_node);
	if (!rk818_pmic_np) {
		pr_err("could not find pmic sub-node\n");
		return NULL;
	}
	pdata = devm_kzalloc(rk818->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	/* pinctrl */
	/*pdata->pinctrl = devm_pinctrl_get(rk818->dev);
	 * if (IS_ERR(pdata->pinctrl)) {
	 * PTR_ERR(pdata->pinctrl);
	 * dev_err(rk818->dev, "could not get pinctrl\n");
	 * }
	 * pdata->pins_default = pinctrl_lookup_state(
	 * pdata->pinctrl, PINCTRL_STATE_DEFAULT);
	 * if (IS_ERR(pdata->pins_default))
	 * dev_err(rk818->dev, "could not get default pinstate\n");
	 * pdata->pins_sleep = pinctrl_lookup_state(
	 * pdata->pinctrl, PINCTRL_STATE_SLEEP);
	 * if (IS_ERR(pdata->pins_sleep))
	 * dev_err(rk818->dev, "could not get sleep pinstate\n");
	 * pdata->pins_inactive = pinctrl_lookup_state(
	 * pdata->pinctrl, "inactive");
	 * if (IS_ERR(pdata->pins_inactive))
	 * dev_err(rk818->dev, "could not get inactive pinstate\n"); */

	pdata->irq = irq_of_parse_and_map(rk818_pmic_np, 0);
	pr_info("rk818 irq number: %d\n", pdata->irq);

	return pdata;
}

#else
static struct rk818_board *rk818_parse_dt(struct i2c_client *i2c)
{
	return NULL;
}
#endif

void rk818_device_shutdown(void)
{
	int ret;
	struct rk818 *rk818 = g_rk818;

	pr_info("%s\n", __func__);
	/*close rtc int when power off*/
	ret = rk818_set_bits(rk818, RK818_INT_STS_MSK_REG1,
			(0x3 << 5), (0x3 << 5));
	ret = rk818_clear_bits(rk818, RK818_RTC_INT_REG, (0x3 << 2));
	ret = rk818_reg_read(rk818, RK818_DEVCTRL_REG);
	ret = rk818_set_bits(rk818, RK818_DEVCTRL_REG, (0x1 << 0), (0x1 << 0));
	/*ret = rk818_set_bits(rk818, RK818_DEVCTRL_REG,(0x1<<4),(0x1<<4));*/
	if (ret < 0)
		pr_err("rk818 power off error!\n");
}
EXPORT_SYMBOL_GPL(rk818_device_shutdown);

__weak void rk818_device_suspend(void)
{
}

__weak void rk818_device_resume(void)
{
}

#ifdef CONFIG_PM
static int rk818_suspend(struct platform_device *client, pm_message_t mesg)
{
	rk818_device_suspend();
	return 0;
}

static int rk818_resume(struct platform_device *client)
{
	rk818_device_resume();
	return 0;
}
#else
static int rk818_suspend(struct platform_device *client, pm_message_t mesg)
{
	return 0;
}

static int rk818_resume(struct platform_device *client)
{
	return 0;
}
#endif

static int rk818_pre_init(struct rk818 *rk818)
{
	int ret, val;
	pr_info("%s,line=%d\n", __func__, __LINE__);

	/*close charger when usb low then 3.4V*/
	ret = rk818_set_bits(rk818, 0xa1, (0x7 << 4), (0x7 << 4));
	/*no action when vref*/
	ret = rk818_set_bits(rk818, 0x52, (0x1 << 1), (0x1 << 1));

	/****************set vbat low **********/
	val = rk818_reg_read(rk818, RK818_VB_MON_REG);
	val &= (~(VBAT_LOW_VOL_MASK | VBAT_LOW_ACT_MASK));
	val |= (RK818_VBAT_LOW_3V5 | EN_VBAT_LOW_IRQ);
	ret = rk818_reg_write(rk818, RK818_VB_MON_REG, val);
	if (ret < 0) {
		pr_err("Unable to write RK818_VB_MON_REG reg\n");
		return ret;
	}
	/**************************************/

	/**********mask int****************/
	val = rk818_reg_read(rk818, RK818_INT_STS_MSK_REG1);
	val |= (0x1 << 0);	/*mask vout_lo_int*/
	ret = rk818_reg_write(rk818, RK818_INT_STS_MSK_REG1, val);
	if (ret < 0) {
		pr_err("Unable to write RK818_INT_STS_MSK_REG1 reg\n");
		return ret;
	}
	/**********enable clkout2****************/
	ret = rk818_reg_write(rk818, RK818_CLK32OUT_REG, 0x01);
	if (ret < 0) {
		pr_err("Unable to write RK818_CLK32OUT_REG reg\n");
		return ret;
	}

	/*open rtc int when power on*/
	ret = rk818_clear_bits(rk818, RK818_INT_STS_MSK_REG1, (0x3 << 5));
	ret = rk818_set_bits(rk818, RK818_RTC_INT_REG, (0x1 << 3), (0x1 << 3));
	return 0;
}

int rk818_remap_ioset(int pin, int value)
{
	void __iomem *vadd;
	vadd = ioremap_nocache(0xE4600200, 0x1000);

	iowrite32(value, vadd + pin);

	return 0;
}

static int rk818_probe(struct platform_device *client)
{
	struct rk818 *rk818;
	struct rk818_board *pdev = 0;
	const struct of_device_id *match;
	int ret, i = 0;

	rk818_remap_ioset(50, 0x3004);
	pr_info("%s,line=%d, here we add a hard code to make INT pull up\n",
	       __func__, __LINE__);

	if (client->dev.of_node) {
		match = of_match_device(rk818_of_match, &client->dev);
		if (!match) {
			dev_err(&client->dev,
				"Failed to find matching dt id\n");
			return -EINVAL;
		}
	}

	rk818 = devm_kzalloc(&client->dev, sizeof(struct rk818), GFP_KERNEL);
	if (rk818 == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	rk818->dev = &client->dev;
	dev_set_drvdata(&client->dev, rk818);

	mutex_init(&rk818->io_lock);

	ret = rk818_reg_read(rk818, 0x2f);
	if ((ret < 0) || (ret == 0xff)) {
		pr_err("The device is not rk818 %d\n", ret);
		goto err;
	}

	ret = rk818_pre_init(rk818);
	if (ret < 0) {
		pr_err("The rk818_pre_init failed %d\n", ret);
		goto err;
	}
	if (rk818->dev->of_node)
		pdev = rk818_parse_dt(rk818);

	ret = rk818_irq_init(rk818, PARAMETER_NO_USE, pdev);
	if (ret < 0)
		goto err;
	ret = mfd_add_devices(rk818->dev, -1,
			      rk818s, ARRAY_SIZE(rk818s), NULL, 0, NULL);

	g_rk818 = rk818;
	if (!pm_power_off)
		pm_power_off = rk818_device_shutdown;
#if 1
	rk818_kobj = kobject_create_and_add("rk818", NULL);
	if (!rk818_kobj)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(rk818_attrs); i++) {
		ret = sysfs_create_file(rk818_kobj, &rk818_attrs[i].attr);
		if (ret != 0) {
			pr_err("create index %d error\n", i);
			return ret;
		}
	}
#endif

	return 0;

err:
	mfd_remove_devices(rk818->dev);
	return ret;

}

static int rk818_remove(struct platform_device *client)
{
	struct rk818 *rk818 = dev_get_drvdata(&client->dev);

	dev_set_drvdata(&client->dev, NULL);
	kfree(rk818);

	return 0;
}

static struct platform_driver rk818_platform_driver = {
	.driver = {
		   .name = "rk818",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rk818_of_match),
		   },
	.probe = rk818_probe,
	.remove = rk818_remove,
#ifdef CONFIG_PM
	.suspend = rk818_suspend,
	.resume = rk818_resume,
#endif
};

static int __init rk818_module_init(void)
{
	int ret;
	ret = platform_driver_register(&rk818_platform_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);
	return ret;
}

late_initcall(rk818_module_init);

static void __exit rk818_module_exit(void)
{
	platform_driver_unregister(&rk818_platform_driver);
}

module_exit(rk818_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhangqing <zhangqing@rock-chips.com>");
MODULE_DESCRIPTION("rk818 PMIC driver");
