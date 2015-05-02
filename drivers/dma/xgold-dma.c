/*
 * Copyright (C) 2012-2014 Intel Mobile Communications GmbH
 * Copyright (c) 2010 ST-Ericsson SA
 * Copyright (c) 2006 ARM Ltd.
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
/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is in this distribution in the file
 * called COPYING.
 *
 */

#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl08x.h>
#include <linux/amba/pl080.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define DRIVER_NAME	"xgold-dmac"

#define dma_warn(fmt, arg...)  \
	pr_warn("DMA: " fmt, ##arg);

#define dma_dbg(fmt, arg...) \
	pr_debug("DMA: " fmt, ##arg);

#define dma_info(fmt, arg...) \
	pr_info("DMA: " fmt, ##arg); \

#define dma_err(fmt, arg...) \
	pr_err("DMA: " fmt, ##arg); \

struct dma_req_mapping {
	struct list_head node;
	const char *bus_id;
	int index;
};

struct dma_reg {
	void __iomem *reg;
	unsigned char shift;
	unsigned char width;
};

struct pl08x_signal {
	struct list_head list;
	unsigned id;
	int busy;
	int val;
	struct dma_reg mux_reg;
	unsigned num_dma_req;
	struct dma_req_mapping *dma_req;
};

struct xgold_dma_irq {
	unsigned int num;
	unsigned int chx[AMBA_NR_IRQS - 1];
	unsigned int err;
};

struct xgold_dma_platdata {
	struct pl08x_platform_data pl08x_pdata;
	struct pl08x_channel_data *slave_channels;
	struct xgold_dma_irq irq;
	unsigned periphid;
	struct clk *clk_master1;
	struct clk *clk_master2;
	struct device_pm_platdata *pm_platdata;
};

static inline struct pl08x_platform_data *to_pl08x_pdata(
		struct xgold_dma_platdata *pdata)
{
	return &pdata->pl08x_pdata;
}

static inline struct xgold_dma_platdata *to_xgold_dma_pdata(
		struct pl08x_platform_data *pdata)
{
	return container_of(pdata, struct xgold_dma_platdata, pl08x_pdata);
}

struct xgold_dma_driver_data {
	struct amba_device adev;
	struct pl08x_signal *signals;
	int num_signals;
	spinlock_t lock;
};

static inline struct amba_device *xgold_dma_to_amba(
		struct xgold_dma_driver_data *data)
{
	return &data->adev;
}

static inline struct xgold_dma_driver_data *amba_to_xgold_dma(
		struct amba_device *adev)
{
	return container_of(adev, struct xgold_dma_driver_data, adev);
}

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
struct device_state_pm_state dma8ch_pm_states[] = {
	{.name = "disable", }, /* D3 */
	{.name = "low_perf", },
	{.name = "mid_perf", },
	{.name = "high_perf", }, /* D0 */
};

#define XGOLD_DMA_D0		3
#define XGOLD_DMA_D3		0

/* DMA8CH PM states & class */
static int xgold_dma_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	struct xgold_dma_driver_data *dma = dev_get_drvdata(dev);
	struct amba_device *adev = xgold_dma_to_amba(dma);
	struct pl08x_platform_data *pl08x_platdata = adev->dev.platform_data;
	struct xgold_dma_platdata *xgold_platdata =
		to_xgold_dma_pdata(pl08x_platdata);
	int id = device_state_pm_get_state_id(dev, state->name);

	switch (id) {
	case XGOLD_DMA_D0:
		if (xgold_platdata->clk_master1)
			clk_enable(xgold_platdata->clk_master1);

		if (xgold_platdata->clk_master2)
			clk_enable(xgold_platdata->clk_master2);

		break;
	case XGOLD_DMA_D3:
		if (xgold_platdata->clk_master1)
			clk_disable(xgold_platdata->clk_master1);

		if (xgold_platdata->clk_master2)
			clk_disable(xgold_platdata->clk_master2);

		break;
	default:
		return id;
	}

	return 0;
}

static struct device_state_pm_state *xgold_dma_get_initial_state(
		struct device *dev)
{
	return &dma8ch_pm_states[XGOLD_DMA_D3];
}

static struct device_state_pm_ops dma8ch_pm_ops = {
	.set_state = xgold_dma_set_pm_state,
	.get_initial_state = xgold_dma_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(dma8ch);
#endif

#ifdef CONFIG_OF
int dma_init_reg_array(struct device_node *np,
				struct dma_reg *dma_reg,
				const char *propname,
				unsigned nr,
				void __iomem *base)
{
	int ret, i;
	unsigned out_values[nr * 3];

	ret = of_property_read_u32_array(np, propname, out_values, nr*3);

	if (ret) {
		pr_devel("Could not find property %s\n", propname);
		for (i = 0; i < nr; i++)
			dma_reg[i].reg = NULL;

		return ret;
	}

	for (i = 0; i < nr; i++) {
		dma_reg->reg = base + out_values[0];
		dma_reg->shift = (unsigned char)out_values[1];
		dma_reg->width = (unsigned char)out_values[2];
	}

	return 0;
}

int dma_init_reg(struct device_node *np,
			struct dma_reg *dma_reg,
			const char *propname,
			void __iomem *base)
{
	return dma_init_reg_array(np, dma_reg, propname, 1, base);
}

#define OF_BUSES		"intel,dma-pl08x,buses"
#define OF_MEMCPY		"intel,dma-pl08x,memcpy"
#define OF_PERIPHID		"intel,dma-pl08x,periphid"
#define OF_MUXBASE		"intel,mux-base"

#define OF_CLK_MASTER1		"clk_master1"
#define OF_CLK_MASTER2		"clk_master2"

#define OF_SLAVE_NODE		"intel,dma-pl08x,slave-chan"
#define OF_SLAVE_LIMITS		"intel,dma-signals"
#define OF_SLAVE_BUS		"intel,dma-bus"

#define OF_REQUEST_NODE		"intel,dma-pl08x,dma-req"
#define OF_REQUEST_MUX		"intel,dma-mux"
#define OF_REQUEST_NAMES	"intel,dma-names"

static struct xgold_dma_platdata *xgold_dma_get_platdata(
		struct device *dev)
{
	struct xgold_dma_driver_data *dma = dev_get_drvdata(dev);
	struct amba_device *adev = xgold_dma_to_amba(dma);
	struct xgold_dma_platdata *xgold_platdata;
	struct pl08x_platform_data *pl08x_platdata;
	struct device_node *np, *child;
	void __iomem *mux_base = NULL;
	struct resource res;
	u32 data[7];
	int ret, i, j, n;

	/* Get device node pointer */
	np = dev->of_node;
	if (!np) {
		dev_err(dev, "Can't find xgold_dma matching node\n");
		ret = -ENODEV;
		goto out;
	}
	adev->dev.of_node = np;

	/* Prepare xgold pl08x platform data struct */
	xgold_platdata = devm_kzalloc(dev, sizeof(struct xgold_dma_platdata),
			GFP_KERNEL);
	if (!xgold_platdata) {
		dev_err(dev, "Can't find xgold_dma matching node\n");
		ret = -ENOMEM;
		goto out;
	}

	pl08x_platdata = to_pl08x_pdata(xgold_platdata);

	/* Init dma req lists */
	dma->signals = devm_kzalloc(dev, 16 * sizeof(struct pl08x_signal),
			GFP_KERNEL);
	if (!dma->signals) {
		dev_err(dev, "Not enough memory\n");
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < 16; i++) {
		INIT_LIST_HEAD(&dma->signals[i].list);
		dma->signals[i].busy = -1;
		dma->signals[i].val = -1;
		dma->signals[i].mux_reg.reg = NULL;
		dma->signals[i].num_dma_req = 0;
	}

	/* Slave channels */
	n = 0;
	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, OF_SLAVE_NODE))
			n++;
	}

	if (!n) {
		dev_err(dev, "Error parsing %s in node %s\n",
				OF_SLAVE_NODE, child->name);
		ret = -EINVAL;
		goto out;
	}

	pl08x_platdata->num_slave_channels = n;
	xgold_platdata->slave_channels = devm_kzalloc(dev,
			sizeof(struct pl08x_channel_data) * n, GFP_KERNEL);

	if (!xgold_platdata->slave_channels) {
		dev_err(dev, "cannot allocate slave channels\n");
		ret = -ENOMEM;
		goto out;
	}

	i = 0;
	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, OF_SLAVE_NODE)) {
			struct pl08x_channel_data *ch =
				&xgold_platdata->slave_channels[i];

			ch->bus_id = (const char *)child->name;
			ch->adev = adev;

			if (of_property_read_u32_array(child, OF_SLAVE_LIMITS,
						data, 2) < 0) {
				dev_err(dev, "Reading %s prop of node %s\n",
						OF_SLAVE_LIMITS, child->name);
				ret = -EINVAL;
				goto out;
			}

			if (data[0] > 15 || data[1] > 15) {
				dev_err(dev, "Error %s of %s node\n",
						OF_SLAVE_LIMITS, child->name);
				ret = -ERANGE;
				goto out;
			}

			ch->min_signal = data[0];
			ch->max_signal = data[1];

			if (of_property_read_u32(child, OF_SLAVE_BUS, data)) {
				dev_err(dev, "Error %s of node %s\n",
						OF_SLAVE_BUS, child->name);
				ret = -EINVAL;
				goto out;
			}

			ch->periph_buses = (*data == 1) ?
				PL08X_AHB1 : PL08X_AHB2;
			i++;
		}
	}

	/* Slave request MUX */
	ret = of_address_to_resource(of_parse_phandle(np, OF_MUXBASE, 0),
			0, &res);
	if (!ret)
		mux_base = devm_ioremap(dev, res.start, resource_size(&res));

	i = 0;
	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, OF_REQUEST_NODE)) {
			struct pl08x_signal *signal;

			if (sscanf(child->name, "req%d", &j) < 1) {
				dev_err(dev, "Cannot get slave req id %s\n",
						child->name);
				ret = -EINVAL;
				goto out;
			}

			if (j < 0 || j > 15) {
				dev_err(dev, "Slave req out of range %s\n",
						child->name);
				ret = -ERANGE;
				goto out;
			}

			signal = &dma->signals[j];

			if (!mux_base || dma_init_reg(child,
						&dma->signals[j].mux_reg,
						OF_REQUEST_MUX, mux_base))
				/* NULL means no DMA mux reg is used */
				signal->mux_reg.reg = NULL;

			n = of_property_count_strings(child, OF_REQUEST_NAMES);
			if (n <= 0) {
				dev_err(dev, "Error %s of node %s\n",
						OF_REQUEST_NAMES, child->name);
				ret = -EINVAL;
				goto out;
			}

			for (i = 0; i < n; i++) {
				struct dma_req_mapping *dma_req =
					devm_kzalloc(dev,
						sizeof(struct dma_req_mapping),
						GFP_KERNEL);

				if (!dma_req) {
					dev_err(dev, "Not enough memory\n");
					ret = -ENOMEM;
					goto out;
				}

				dma_req->index = i;

				if (of_property_read_string_index(
					child, OF_REQUEST_NAMES, i,
					&dma_req->bus_id)) {

					dma_err("Cannot get name of dma request index %d for child %s\n",
							i, child->name);
					ret = -EINVAL;
					goto out;
				}

				list_add(&dma_req->node, &signal->list);
			}

			signal->num_dma_req = n;
			signal->busy = 0;
		}
	}

	/* Buses (optional, if dualmaster only) */
	if (of_property_read_u32_array(np, OF_BUSES, data, 2) == 0) {
		if (data[0] > 2 || data[1] > 2) {
			dev_err(dev, "%s of node %s out of range\n",
					OF_BUSES, np->name);
			ret = -ERANGE;
			goto out;
		}
		pl08x_platdata->lli_buses =
			(data[0] == 1) ? PL08X_AHB1 : PL08X_AHB2;
		pl08x_platdata->mem_buses =
			(data[1] == 1) ? PL08X_AHB1 : PL08X_AHB2;
	}

	/* Memcpy configuration
	 * SB/DB_SIZE, S/D_WIDTH, PROT (USER, BUFF, CACHE)
	 */
	if (of_property_read_u32_array(np, OF_MEMCPY, data, 7) < 0) {
		dev_err(dev, "Error parsing memcpy property\n");
		ret = -EINVAL;
		goto out;
	}

	i = 0;
	pl08x_platdata->memcpy_channel.cctl_memcpy =
		data[i++] << PL080_CONTROL_SB_SIZE_SHIFT;
	pl08x_platdata->memcpy_channel.cctl_memcpy |=
		data[i++] << PL080_CONTROL_DB_SIZE_SHIFT;
	pl08x_platdata->memcpy_channel.cctl_memcpy |=
		data[i++] << PL080_CONTROL_SWIDTH_SHIFT;
	pl08x_platdata->memcpy_channel.cctl_memcpy |=
		data[i++] << PL080_CONTROL_DWIDTH_SHIFT;
	pl08x_platdata->memcpy_channel.cctl_memcpy |=
		(data[i++] == 1) ? PL080_CONTROL_PROT_SYS : 0;
	pl08x_platdata->memcpy_channel.cctl_memcpy |=
		(data[i++] == 1) ? PL080_CONTROL_PROT_BUFF : 0;
	pl08x_platdata->memcpy_channel.cctl_memcpy |=
		(data[i++] == 1) ? PL080_CONTROL_PROT_CACHE : 0;

	/* AMBA Peripheral ID */
	if (of_property_read_u32(np, OF_PERIPHID, data) < 0) {
		dev_err(dev, "Error parsing peripheral id\n");
		ret = -EINVAL;
		goto out;
	}
	xgold_platdata->periphid = data[0];

	/* Clocks */
	xgold_platdata->clk_master1 = of_clk_get_by_name(np, OF_CLK_MASTER1);
	if (IS_ERR(xgold_platdata->clk_master1))
		xgold_platdata->clk_master1 = NULL;
	else
		clk_prepare(xgold_platdata->clk_master1);

	xgold_platdata->clk_master2 = of_clk_get_by_name(np, OF_CLK_MASTER2);
	if (IS_ERR(xgold_platdata->clk_master2))
		xgold_platdata->clk_master2 = NULL;
	else
		clk_prepare(xgold_platdata->clk_master2);

	/* apb clock (slave clock) is driven by the amba bus driver */

	/* Interrupts */
	xgold_platdata->irq.num = n = of_irq_count(np);
	if (!n) {
		dev_err(dev,
				"Error parsing interrupts in %s\n", np->name);
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < n; i++) {
		const char *name = NULL;
		int irq = irq_of_parse_and_map(np, i);

		if (irq) {
			of_property_read_string_index(
					np, "interrupt-names", i, &name);
		} else  {
			dev_err(dev, "Cannot map irq index %d", i);
			ret = -EINVAL;
			goto out;
		}

		if (!strcmp("err", name))
			xgold_platdata->irq.err = irq;
		else if (!strcmp("chx", name)) {
			/* In case only 1 interrupt line for all CHi channels */
			int j;
			xgold_platdata->irq.chx[0] = irq;
			for (j = 1; j < 8; j++)
				xgold_platdata->irq.chx[j] = 0;
		} else if (!strncmp("ch", name, 2)) {
			/* 1 interrupt per channel */
			int index;
			if (sscanf(name, "ch%d", &index) < 1) {
				dev_err(dev, "Cannot get interrupt id\n");
				ret = -EINVAL;
				goto out;
			}

			xgold_platdata->irq.chx[index] = irq;
		} else {
			dev_err(dev, "Unknown interrupt name %s\n", name);
			ret = -EINVAL;
			goto out;
		}
	}

	xgold_platdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(xgold_platdata->pm_platdata)) {
		dev_err(dev, "Error during device state pm init\n");
		ret = -EINVAL;
		goto out;
	}

	return xgold_platdata;

out:
	return ERR_PTR(ret);
}
#else
static struct xgold_dma_platdata *xgold_dma_get_platdata(
		struct device *dev)
{
	struct xgold_dma_driver_data *dma = dev_get_drvdata(dev);
	struct amba_device *adev = xgold_dma_to_amba(dma);
	struct xgold_dma_platdata *xgold_platdata;
	struct pl08x_platform_data *pl08x_platdata;
	int i, n;

	/* TODO */
	return ERR_PTR(-EINVAL);
}
#endif

static void dma_reg_write(struct dma_reg *dma_reg, unsigned int val)
{
	unsigned reg = readl(dma_reg->reg);
	unsigned mask = (1 << dma_reg->width) - 1;
	reg &= ~(mask << dma_reg->shift);
	reg |= val << dma_reg->shift;
	writel(reg, dma_reg->reg);
}

static int xgold_dma_get_signal(const struct pl08x_channel_data *cd)
{
	struct xgold_dma_driver_data *dma = amba_to_xgold_dma(cd->adev);
	unsigned int i;
	unsigned long flags;

	spin_lock_irqsave(&dma->lock, flags);

	for (i = cd->min_signal; i <= cd->max_signal; i++) {
		if (!dma->signals[i].busy) {
			struct dma_req_mapping *dma_req;
			list_for_each_entry(dma_req,
					&dma->signals[i].list, node) {
				if (!strcmp(dma_req->bus_id, cd->bus_id)) {
					if (dma->signals[i].mux_reg.reg) {
						dma_reg_write(
						&dma->signals[i].mux_reg,
						dma_req->index);
					}
					dma->signals[i].busy++;
					dma->signals[i].val = dma_req->index;
					spin_unlock_irqrestore(&dma->lock,
							flags);
					return i;
				}
			}
		}
	}
	spin_unlock_irqrestore(&dma->lock, flags);
	return -EINVAL;
}

static void xgold_dma_put_signal(
		const struct pl08x_channel_data *cd, int i)
{
	struct xgold_dma_driver_data *dma = amba_to_xgold_dma(cd->adev);
	unsigned long flags;

	spin_lock_irqsave(&dma->lock, flags);

	/* if signal is not used */
	if (!dma->signals[i].busy)
		WARN_ON(true);
	else
		dma->signals[i].busy--;

	spin_unlock_irqrestore(&dma->lock, flags);
}

#ifdef CONFIG_PM
int xgold_dma_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct xgold_dma_driver_data *dma = dev_get_drvdata(&pdev->dev);
	struct amba_device *adev = xgold_dma_to_amba(dma);
	struct pl08x_platform_data *pl08x_platdata = adev->dev.platform_data;
	struct xgold_dma_platdata *xgold_platdata =
		to_xgold_dma_pdata(pl08x_platdata);
	int ret;

	ret = platform_device_pm_set_state_by_name(pdev,
			xgold_platdata->pm_platdata->pm_state_D3_name);

	if (ret)
		dev_err(&pdev->dev, "Error during state transition %s\n",
				xgold_platdata->pm_platdata->pm_state_D3_name);

	return ret;
}

int xgold_dma_resume(struct platform_device *pdev)
{
	struct xgold_dma_driver_data *dma = dev_get_drvdata(&pdev->dev);
	struct amba_device *adev = xgold_dma_to_amba(dma);
	struct pl08x_platform_data *pl08x_platdata = adev->dev.platform_data;
	struct xgold_dma_platdata *xgold_platdata =
		to_xgold_dma_pdata(pl08x_platdata);
	int ret;

	ret = platform_device_pm_set_state_by_name(pdev,
			xgold_platdata->pm_platdata->pm_state_D0_name);

	if (ret)
		dev_err(&pdev->dev, "Error during state transition %s\n",
				xgold_platdata->pm_platdata->pm_state_D0_name);

	return ret;

}
#else
#define xgold_dma_suspend NULL
#define xgold_dma_resume NULL
#endif

#define ADEV_NAME "pl08xdmac"

static int xgold_dma_probe(struct platform_device *pdev)
{
	struct xgold_dma_driver_data *dma;
	struct xgold_dma_platdata *xgold_platdata;
	struct pl08x_platform_data *pl08x_platdata;
	struct amba_device *adev;
	struct resource *res_io;
	int ret;

	pr_info("Enter %s\n", __func__);

	/* Get memory resources */
	res_io = platform_get_resource_byname(
			pdev, IORESOURCE_MEM, "register");

	if (!res_io) {
		ret = -EINVAL;
		goto out;
	}

	/* allocate xgold_dma_driver_data, where amba_device is included */
	dma = devm_kzalloc(&pdev->dev, sizeof(*dma), GFP_KERNEL);
	if (!dma) {
		ret = -ENOMEM;
		goto out;
	}

	spin_lock_init(&dma->lock);
	platform_set_drvdata(pdev, dma);

	xgold_platdata = xgold_dma_get_platdata(&pdev->dev);
	if (IS_ERR(xgold_platdata)) {
		dma_err("Error while getting platdata\n");
		ret = PTR_ERR(xgold_platdata);
		goto out;
	}

	/* Prepare pl08x amba device platdata */
	pl08x_platdata = to_pl08x_pdata(xgold_platdata);
	pl08x_platdata->get_xfer_signal = xgold_dma_get_signal;
	pl08x_platdata->put_xfer_signal = xgold_dma_put_signal;
	pl08x_platdata->slave_channels = xgold_platdata->slave_channels;

	/* Fill amba device struct with missing properties */
	adev = xgold_dma_to_amba(dma);
	adev->dev.platform_data = pl08x_platdata;
	memcpy(adev->irq, xgold_platdata->irq.chx, (AMBA_NR_IRQS - 1) *
			sizeof(unsigned int));
	adev->periphid = xgold_platdata->periphid;
	adev->dev.init_name = ADEV_NAME;
	adev->dev.coherent_dma_mask = ~0ULL;
	/* Must duplicate mem resource into amba device struct to avoid memory
	 * request failure during amba device registration */
	memcpy(&adev->res, res_io, sizeof(*res_io));

	if (xgold_platdata->pm_platdata) {
		ret = device_state_pm_set_class(&pdev->dev,
				xgold_platdata->pm_platdata->pm_user_name);
		if (ret) {
			dev_err(&pdev->dev,
				"Error while setting the pm class\n");
			goto out;
		}

		ret = device_state_pm_set_state_by_name(&pdev->dev,
				xgold_platdata->pm_platdata->pm_state_D0_name);
		dev_dbg(&pdev->dev, "%s set state return %d\n", __func__, ret);
	}

	ret = amba_device_register(adev, res_io);
	if (ret) {
		dma_err("Error while registering amba device\n");
		goto out;
	}

	return 0;

out:
	return ret;
}

static int xgold_dma_remove(struct platform_device *pdev)
{
	struct xgold_dma_driver_data *dma = platform_get_drvdata(pdev);
	struct amba_device *adev = xgold_dma_to_amba(dma);

	amba_device_unregister(adev);

	return 0;
}

static struct of_device_id xgold_dma_of_match[] = {
	{ .compatible = "intel,dma",},
	{ .compatible = "intel,pl08x",},
	{ },
};

static struct platform_driver xgold_dma_platform_driver = {
/*	.id_table	= xgold_dma_ids,*/
	.probe		= xgold_dma_probe,
	.remove		= xgold_dma_remove,
	.suspend	= xgold_dma_suspend,
	.resume		= xgold_dma_resume,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = xgold_dma_of_match,
	},
};

static int __init xgold_dma_init(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&dma8ch_pm_class);
	if (ret)
		return ret;
#endif
	return  platform_driver_register(&xgold_dma_platform_driver);
}

static void __exit xgold_dma_exit(void)
{
	platform_driver_unregister(&xgold_dma_platform_driver);
}

subsys_initcall(xgold_dma_init);
module_exit(xgold_dma_exit);

MODULE_DEVICE_TABLE(of, xgold_dma_of_match);
