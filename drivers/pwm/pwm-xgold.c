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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sofia/mv_svc_hypercalls.h>

#define PWM_DUTY_WIDTH	31
#define PWM_MAX_COUNTER ((1 << PWM_DUTY_WIDTH) - 1)

#define NUM_PWM 1

struct xgold_pwm_chip {
	struct pwm_chip		chip;
	struct device		*dev;

	struct clk		*clk;

	void __iomem		*mmio_base;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
};

static inline struct xgold_pwm_chip *to_xgold_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct xgold_pwm_chip, chip);
}

static inline u32 pwm_readl(struct xgold_pwm_chip *chip, unsigned int num)
{
	return readl(chip->mmio_base + num);
}

static inline void pwm_writel(struct xgold_pwm_chip *chip, unsigned int num,
			     unsigned long val)
{
	writel(val, chip->mmio_base + num);
}

static int xgold_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			    int duty_ns, int period_ns)
{
	struct xgold_pwm_chip *pc = to_xgold_pwm_chip(chip);
	unsigned long c;
	unsigned long rate, hz;
	u32 val = 0;
	u32 range = 0;
	int err;

	dev_info(chip->dev, "capcom config: %dns/%dns", duty_ns, period_ns);

	if (!test_bit(PWMF_ENABLED, &pwm->flags)) {
		err = clk_prepare_enable(pc->clk);
		if (err < 0)
			return err;
	}

	mv_svc_pwm_access(PWM_CONFIG, duty_ns, period_ns);
	return 0;
}

static int xgold_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct xgold_pwm_chip *pc = to_xgold_pwm_chip(chip);
	int rc = 0;
	u32 val;

	rc = clk_prepare_enable(pc->clk);
	if (rc < 0)
		return rc;

	mv_svc_pwm_access(PWM_ENABLE, 0, 0);
	dev_info(chip->dev, "capcom enable");
	return 0;
}

static void xgold_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct xgold_pwm_chip *pc = to_xgold_pwm_chip(chip);

	mv_svc_pwm_access(PWM_DISABLE, 0, 0);

	clk_disable_unprepare(pc->clk);
	dev_info(chip->dev, "capcom disable");
}

static const struct pwm_ops xgold_pwm_ops = {
	.config = xgold_pwm_config,
	.enable = xgold_pwm_enable,
	.disable = xgold_pwm_disable,
	.owner = THIS_MODULE,
};

static int xgold_pwm_probe(struct platform_device *pdev)
{
	struct xgold_pwm_chip *pwm;
	struct device_node *np;
	struct resource *r;
	int ret;

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, pwm);
#ifdef CONFIG_OF
	np = pdev->dev.of_node;

	/* FIXME: get base from capcom chipdata */
	pwm->mmio_base = of_iomap(np, 0);
	if (!pwm->mmio_base) {
		pr_err("%s: unable to ioremap for base address\n", __func__);
		return -ENOMEM;
	}

	/* clock */
	pwm->clk = of_clk_get_by_name(np, "clk_kernel");
	if (IS_ERR(pwm->clk))
		pwm->clk = NULL;

	pwm->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pwm->pinctrl))
		return -ENODEV;

	pwm->pins_default = pinctrl_lookup_state(pwm->pinctrl,
			PINCTRL_STATE_DEFAULT);
	if (IS_ERR(pwm->pins_default))
		dev_err(&pdev->dev, "could not get default pinstate\n");
	pinctrl_select_state(pwm->pinctrl, pwm->pins_default);
#endif
	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &xgold_pwm_ops;
	pwm->chip.base = -1;
	pwm->chip.npwm = NUM_PWM;

	ret = pwmchip_add(&pwm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int xgold_pwm_remove(struct platform_device *pdev)
{
	struct xgold_pwm_chip *pc = platform_get_drvdata(pdev);
	int i;

	if (WARN_ON(!pc))
		return -ENODEV;

	for (i = 0; i < NUM_PWM; i++) {
		struct pwm_device *pwm = &pc->chip.pwms[i];

		if (!test_bit(PWMF_ENABLED, &pwm->flags))
			if (clk_prepare_enable(pc->clk) < 0)
				continue;

		pwm_writel(pc, i, 0);

		clk_disable_unprepare(pc->clk);
	}

	return pwmchip_remove(&pc->chip);
}

static const struct of_device_id xgold_pwm_of_match[] = {
	{ .compatible = "intel,capcom-pwm" },
	{ }
};

MODULE_DEVICE_TABLE(of, xgold_pwm_of_match);

static struct platform_driver xgold_pwm_driver = {
	.driver = {
		.name = "xgold-pwm",
		.of_match_table = xgold_pwm_of_match,
	},
	.probe = xgold_pwm_probe,
	.remove = xgold_pwm_remove,
};

module_platform_driver(xgold_pwm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("xgold pwm driver");
