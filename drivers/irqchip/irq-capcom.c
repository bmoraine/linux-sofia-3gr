/*
 * Copyright (c) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/cpu_pm.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/irqchip/irq_xgold.h>

#include "irqchip.h"

/* Capcom Capture interrupt types */
#define XGOLD_CC_IRQ_TYPE_EDGE_DISABLED	0
#define XGOLD_CC_IRQ_TYPE_EDGE_RISING		1
#define XGOLD_CC_IRQ_TYPE_EDGE_FALLING		2
#define XGOLD_CC_IRQ_TYPE_EDGE_BOTH		3

static DEFINE_SPINLOCK(capcom_lock);

#define XGOLD_ENTER pr_info("--> %s\n", __func__);
#define XGOLD_EXIT  pr_info("<-- %s\n", __func__);

struct xgold_capcom_pdata {
	void __iomem *base;
	struct device_pm_platdata *pm_platdata;
	struct clk *clk_kernel;
};

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
/* Capcom PM states & class */
static struct device_state_pm_state capcom_pm_states[] = {
	{ .name = "disable", }, /* D3 */
	{ .name = "enable", }, /* D0 */
	{ .name = "disable_psv", },
	{ .name = "enable_psv", },
};

/* PM states index */
#define XGOLD_CAPCOM_D0		1
#define XGOLD_CAPCOM_D3		0

static int xgold_capcom_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	struct xgold_capcom_pdata *pdata = dev_get_drvdata(dev);
	int id = device_state_pm_get_state_id(dev, state->name);

	pr_info("%s: pm state %s\n", __func__, state->name);
	switch (id) {
	case XGOLD_CAPCOM_D0:
		if (pdata->clk_kernel)
			clk_prepare_enable(pdata->clk_kernel);

		iowrite32(0x100, pdata->base);
		break;

	case XGOLD_CAPCOM_D3:
		iowrite32(0x0, pdata->base);
		if (pdata->clk_kernel)
			clk_disable_unprepare(pdata->clk_kernel);

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static struct device_state_pm_state *xgold_capcom_get_initial_state(
		struct device *dev)
{
	return &capcom_pm_states[XGOLD_CAPCOM_D3];
}

static struct device_state_pm_ops capcom_pm_ops = {
	.set_state = xgold_capcom_set_pm_state,
	.get_initial_state = xgold_capcom_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(capcom);

#endif /* CONFIG_PLATFORM_DEVICE_PM_VIRT */

static inline void xgold_irq_capcom_unmask(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	spin_lock(&capcom_lock);
	xgold_irq_write(chipdata, chipdata->mask[irq], 1, XGOLD_RW);
	spin_unlock(&capcom_lock);
}

static inline void xgold_irq_capcom_mask(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	spin_lock(&capcom_lock);
	xgold_irq_write(chipdata, chipdata->mask[irq], 0, XGOLD_RW);
	spin_unlock(&capcom_lock);
}

static void xgold_irq_capcom_ack(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	xgold_irq_write(chipdata, chipdata->ack[irq], 1, XGOLD_WO);
}

static void xgold_irq_capcom_maskack(struct irq_data *data)
{
	xgold_irq_capcom_mask(data);
	xgold_irq_capcom_ack(data);
}

static LIST_HEAD(set_type_list);
static DEFINE_MUTEX(set_type_list_mtx);
bool irq_capcom_power_ready;
struct xgold_irq_set_type_rq {
	uint32_t type;
	struct irq_data *data;
	struct list_head list;
};
static void xgold_irq_capcom_add_req(struct xgold_irq_set_type_rq *req)
{
	mutex_lock(&set_type_list_mtx);
	list_add_tail(&req->list, &set_type_list);
	mutex_unlock(&set_type_list_mtx);
}

static int xgold_irq_capcom_set_type(struct irq_data *data, unsigned int type)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	u32 edge = XGOLD_CC_IRQ_TYPE_EDGE_DISABLED;
	struct xgold_irq_set_type_rq *req;
	pr_debug("%s(%d, %#x)\n", __func__, irq, type);

	if (!irq_capcom_power_ready) {
		pr_debug("%s: Store the request to replay later\n", __func__);
		/* We are getting set_type requests too early while capcom is
		 * not yet powered ON as PRH is not ready to be used.
		 * Let's store them to replay them at capcom probe time */
		req = kzalloc(sizeof(struct xgold_irq_set_type_rq), GFP_KERNEL);
		if (!req)
			return -ENOMEM;
		req->type = type;
		req->data = data;
		xgold_irq_capcom_add_req(req);
		return 0;
	}
	if (!chipdata->edge[irq])
		return -EINVAL;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		pr_debug("%s: IRQ_TYPE_EDGE_RISING\n", __func__);
		edge = XGOLD_CC_IRQ_TYPE_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		pr_debug("%s: IRQ_TYPE_EDGE_FALLING\n", __func__);
		edge = XGOLD_CC_IRQ_TYPE_EDGE_FALLING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		pr_debug("%s: IRQ_TYPE_EDGE_BOTH\n", __func__);
		edge = XGOLD_CC_IRQ_TYPE_EDGE_BOTH;
		break;

	default:
		pr_err("%s: No such irq type %d. exit...", __func__, type);
		return -EINVAL;
	}
	spin_lock(&capcom_lock);
	xgold_irq_write(chipdata, chipdata->edge[irq], edge, XGOLD_RW);
	spin_unlock(&capcom_lock);

	return 0;
}

static inline int xgold_irq_capcom_retrigger(struct irq_data *data)
{
	struct xgold_irq_chip_data *chipdata = irq_data_get_irq_chip_data(data);
	u32 irq = data->hwirq;
	spin_lock(&capcom_lock);
	xgold_irq_write(chipdata, chipdata->set[irq], 1, XGOLD_WO);
	spin_unlock(&capcom_lock);
	return 0;
}

static struct irq_chip xgold_irq_capcom_chip = {
	.name = "CAPCOM",
	.irq_mask = xgold_irq_capcom_mask,
	.irq_disable = xgold_irq_capcom_mask,
	.irq_unmask = xgold_irq_capcom_unmask,
	.irq_mask_ack = xgold_irq_capcom_maskack,
	.irq_ack = xgold_irq_capcom_ack,
	.irq_set_type = xgold_irq_capcom_set_type,
	.irq_retrigger = xgold_irq_capcom_retrigger,
};

static struct irq_domain_ops xgold_irq_capcom_domain_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = xgold_irq_domain_map,
};

/*
 * Entry point for Capcom Capture IRQ. Called from of_irq_init
 */
static int __init xgold_irq_capcom_of_init(struct device_node *np,
					struct device_node *parent)
{
	int ret = 0;
	struct xgold_irq_chip_data *data;

	data = kzalloc(sizeof(struct xgold_irq_chip_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->chip = &xgold_irq_capcom_chip;
	data->type = XGOLD_IRQ_DOMAIN_N21;

	/* extract info from dt */
	xgold_irq_of_get(np, data);

	/* add linear domain */
	ret |= xgold_irq_domain_add_linear(np,
			data, &xgold_irq_capcom_domain_ops);

	/* Parse, Map and Cascade */
	if (parent)
		ret |= xgold_irq_parse_map_and_cascade(np, data);

	if (ret)
		goto out;

out:
	return ret;
}

IRQCHIP_DECLARE(xgold_capcom, "intel,capcom", xgold_irq_capcom_of_init);

static int xgold_capcom_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int xgold_capcom_resume(struct platform_device *pdev)
{
	return 0;
}

static int __init xgold_capcom_probe(struct platform_device *pdev)
{
	struct xgold_capcom_pdata *pdata;
	struct device_node *np;
	struct xgold_irq_set_type_rq *req;
	int ret;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, pdata);

#ifdef CONFIG_OF
	np = pdev->dev.of_node;

	/* FIXME: get base from capcom chipdata */
	pdata->base = of_iomap(np, 0);
	if (!pdata->base) {
		pr_err("%s: unable to ioremap for base address\n", __func__);
		return -ENOMEM;
	}

	/* clock */
	pdata->clk_kernel = of_clk_get_by_name(np, "clk_kernel");
	if (IS_ERR(pdata->clk_kernel))
		pdata->clk_kernel = NULL;

	/* extract pm info */
	pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pdata->pm_platdata)) {
		dev_err(&pdev->dev, "no pm info available\n");
		return -1;
	}
#endif

	/* pm */
	ret = device_state_pm_set_class(&pdev->dev,
			pdata->pm_platdata->pm_user_name);
	if (ret) {
		dev_err(&pdev->dev, "Error while setting the pm class\n");
		goto out;
	}

	/* Switch to enable state */
	ret = device_state_pm_set_state_by_name(&pdev->dev,
			pdata->pm_platdata->pm_state_D0_name);

	if (ret) {
		dev_err(&pdev->dev, "Set state return %d\n", ret);
		goto out;
	}
	irq_capcom_power_ready = true;

	/* Replay set_type requests and free them */
	mutex_lock(&set_type_list_mtx);
	list_for_each_entry(req, &set_type_list, list)
		xgold_irq_capcom_set_type(req->data, req->type);
	mutex_unlock(&set_type_list_mtx);
	return 0;

out:
	return ret;
}

static int xgold_capcom_remove(struct platform_device *pdev)
{
	struct xgold_capcom_pdata *pdata = dev_get_drvdata(&pdev->dev);
	int ret;

	ret = device_state_pm_set_state_by_name(&pdev->dev,
			pdata->pm_platdata->pm_state_D3_name);
	if (ret)
		return ret;

	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static struct of_device_id xgold_capcom_of_match[] = {
	{ .compatible = "intel,capcom",},
	{ },
};

static struct platform_driver xgold_capcom_driver = {
	.probe = xgold_capcom_probe,
	.remove = xgold_capcom_remove,
#ifdef CONFIG_PM
	.suspend = xgold_capcom_suspend,
	.resume = xgold_capcom_resume,
#endif
	.driver = {
		.name = "xgold-capcom",
		.owner = THIS_MODULE,
		.of_match_table = xgold_capcom_of_match,
	},
};

static int __init xgold_capcom_init_driver(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&capcom_pm_class);
	if (ret)
		return ret;
#endif

	return platform_driver_register(&xgold_capcom_driver);
}

static void __exit xgold_capcom_exit_driver(void)
{
	platform_driver_unregister(&xgold_capcom_driver);
}

subsys_initcall(xgold_capcom_init_driver);
module_exit(xgold_capcom_exit_driver);

MODULE_DEVICE_TABLE(of, xgold_capcom_of_match);
