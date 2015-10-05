/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/irqdomain.h>
#include <linux/pinctrl/consumer.h>
#include <linux/irqchip/irq_xgold.h>

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/mv_svc_hypercalls.h>
#endif

#define gpio_err(fmt, arg...)	pr_err("gpio: "  fmt, ##arg)
#define gpio_info(fmt, arg...)	pr_info("gpio: "  fmt, ##arg)
#define gpio_dbg(fmt, arg...)	pr_debug("gpio: "  fmt, ##arg)
#define MAX_GPIO_IRQS		16

#define PCL_IO_ACCESS_BY_VMM 0
#define PCL_IO_ACCESS_BY_LNX 1

struct xgold_pcl_field {
	char *name;
	char shift;
	u32 mask;
};
struct xgold_gpio_irq {
	u32 gpio;
	u32 irq;
};

struct xgold_pcl_gpio {
	struct gpio_chip pchip;
	void __iomem *gpio_base;
	phys_addr_t gpio_base_phys;
	u32 dir_base;
	struct xgold_pcl_field dir_field;
	u32 dir_out;

	u32 out_base;
	struct xgold_pcl_field out_field;

	u32 in_base;
	struct xgold_pcl_field in_field;

	u32 nirqs;
	struct xgold_gpio_irq gpio_irq[MAX_GPIO_IRQS];
	bool io_master;
};

static inline struct xgold_pcl_gpio *to_xgold_pcl_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct xgold_pcl_gpio, pchip);
}


static inline u32 pcl_read(struct gpio_chip *chip, unsigned offset)
{
	struct xgold_pcl_gpio *xgold_gpio = to_xgold_pcl_gpio(chip);
#ifdef CONFIG_X86_INTEL_SOFIA
	phys_addr_t read_addr = xgold_gpio->gpio_base_phys + offset;
	uint32_t tmp = 0;
	if (xgold_gpio->io_master == PCL_IO_ACCESS_BY_VMM) {
		if (mv_svc_pinctrl_service(PINCTRL_GET, read_addr, 0,
					(uint32_t *)&tmp))
			gpio_err("%s: mv_svc_pinctrl_service(PINCTRL_GET) failed at 0x%pa\n",
					__func__, &read_addr);
		return tmp;
	} else
#endif
		return readl_relaxed(xgold_gpio->gpio_base + offset);
}

static inline u32 pcl_read_pin(struct gpio_chip *chip, unsigned offset)
{
	struct xgold_pcl_gpio *xgold_gpio = to_xgold_pcl_gpio(chip);
#ifdef CONFIG_X86_INTEL_SOFIA
	phys_addr_t read_addr = xgold_gpio->gpio_base_phys + offset;
	uint32_t tmp = 0;
	if (xgold_gpio->io_master == PCL_IO_ACCESS_BY_VMM) {
		if (mv_svc_pinctrl_service(PINCTRL_GET_PIN, read_addr, 0,
					(uint32_t *)&tmp))
			gpio_err("%s: mv_svc_pinctrl_service(PINCTRL_GET_PIN) failed at 0x%pa\n",
					__func__, &read_addr);
		return tmp;
	} else
#endif
		return readl_relaxed(xgold_gpio->gpio_base + offset);
}
static inline void pcl_write(struct gpio_chip *chip, unsigned offset, u32 value)
{
	struct xgold_pcl_gpio *xgold_gpio = to_xgold_pcl_gpio(chip);
#ifdef CONFIG_X86_INTEL_SOFIA
	phys_addr_t write_addr = xgold_gpio->gpio_base_phys + offset;
	if (xgold_gpio->io_master == PCL_IO_ACCESS_BY_VMM) {
		if (mv_svc_pinctrl_service(PINCTRL_SET, write_addr, value,
					NULL))
			gpio_err("%s: mv_svc_pinctrl_service(PINCTRL_SET) fails at 0x%pa - value: 0x%x\n",
				__func__, &write_addr, value);
	} else
#endif
		writel(value, (xgold_gpio->gpio_base + offset));
}
static inline void field_set_val(struct xgold_pcl_field *f, u32 *reg, u32 val)
{
	*reg &= ~(f->mask);
	*reg |= val << f->shift;
}

static inline u32 field_get_val(struct xgold_pcl_field *f, u32 reg)
{
	return (reg & f->mask) >> f->shift;
}

static inline void tofield(struct xgold_pcl_field *f, u32 shift, u32 nbits)
{
	int j;
	u32 mask = 0;
	f->shift = shift;
	for (j = 0; j < nbits; j++)
		mask |= 1 << j;
	f->mask = mask << shift;
}


int xgold_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	gpio_dbg("%s to %s gpio %d (%d))\n",
		 __func__, chip->label, offset, chip->base + offset);
	return pinctrl_request_gpio(chip->base + offset);
}

void xgold_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_free_gpio(chip->base + offset);
}

static void xgold_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	u32 tmp;
	struct xgold_pcl_gpio *xgold_gpio = to_xgold_pcl_gpio(chip);
	tmp = pcl_read(chip, xgold_gpio->out_base + (offset * sizeof(u32)));
	field_set_val(&xgold_gpio->out_field, &tmp, value);
	pcl_write(chip, xgold_gpio->out_base + (offset * sizeof(u32)), tmp);
}

static int xgold_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	u32 reg;
	struct xgold_pcl_gpio *xgold_gpio = to_xgold_pcl_gpio(chip);
	reg = pcl_read_pin(chip, xgold_gpio->in_base + (offset * sizeof(u32)));
	return field_get_val(&xgold_gpio->in_field, reg);
}

static int xgold_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	u32 value;
	struct xgold_pcl_gpio *xgold_gpio = to_xgold_pcl_gpio(chip);
	value = pcl_read(chip, xgold_gpio->dir_base + (offset * sizeof(u32)));
	field_set_val(&xgold_gpio->dir_field, &value, !xgold_gpio->dir_out);
	pcl_write(chip, xgold_gpio->dir_base + (offset * sizeof(u32)), value);
	return 0;
}

static int xgold_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
				       int value)
{
	u32 tmp;
	struct xgold_pcl_gpio *xgold_gpio = to_xgold_pcl_gpio(chip);
	tmp = pcl_read(chip, xgold_gpio->dir_base + (offset * sizeof(u32)));
	field_set_val(&xgold_gpio->dir_field, &tmp, xgold_gpio->dir_out);
	field_set_val(&xgold_gpio->out_field, &tmp, value);
	pcl_write(chip, xgold_gpio->dir_base + (offset * sizeof(u32)), tmp);
	return 0;
}

static int xgold_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	u32 i, irq;
	int eint = -1;
	struct xgold_pcl_gpio *xgold_gpio = to_xgold_pcl_gpio(chip);
	struct irq_domain *domain = xgold_irq_eint_get_domain();

	for (i = 0; i < xgold_gpio->nirqs; i++) {
		if (xgold_gpio->gpio_irq[i].gpio == offset) {
			eint = xgold_gpio->gpio_irq[i].irq;
			break;
		}
	}

	if (eint < 0) {
		pr_err("%s: Can't bind gpio to interrupt\n", __func__);
		return -EINVAL;
	}

	if (domain) {
		irq = irq_find_mapping(domain, eint);
		pr_debug("%s:irq found is:%d for eint:%d\n",
				__func__, irq, eint);
		return irq;
	} else
		pr_err("%s: No eint domain found\n", __func__);

	return -EINVAL;
}

static struct of_device_id xgold_gpio_of_match[] = {
	{.compatible = "intel,gpio",},
	{},
};

#define GPIO_DT_DHIP		"intel,gpiochip"
#define GPIO_DT_PIN_BASE	"intel,gpiochip-base"
#define GPIO_DT_PIN_NUM		"intel,gpiochip-num"
#define PROP_GPIO_DIRECTION	"intel,gpio-direction"
#define PROP_GPIO_DIRECTION_OUT	"intel,gpio-direction-out"
#define PROP_GPIO_OUTPUT	"intel,gpio-output"
#define PROP_GPIO_INPUT		"intel,gpio-input"
#define PROP_GPIO_CHIP_ID	"intel,gpiochip-id"
#define GPIO_TO_IRQ_NUM		"intel,gpio-to-irq-num"
#define GPIO_TO_IRQ		"intel,gpio-to-irq"

/*
 * Get intel,io-access property if any from dts
 */
bool xgold_gpio_get_io_master(struct device_node *np)
{
	if (of_find_property(np, "intel,vmm-secured-access", NULL))
		return PCL_IO_ACCESS_BY_VMM;
	return PCL_IO_ACCESS_BY_LNX;

}
static int xgold_gpio_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np;
	struct gpio_chip *pchip;
	u32 ngpios;
	struct resource *res;
	u32 array[3], i;
	u32 tab_irq[2 * MAX_GPIO_IRQS];

	struct xgold_pcl_gpio *xgold_gpio;
	/* find pinctrl node devicetree */
	np = pdev->dev.of_node;
	if (!np) {
		gpio_err("Can't find gpio node\n");
		return -EINVAL;
	}

	xgold_gpio = devm_kzalloc(&pdev->dev, sizeof(*xgold_gpio), GFP_KERNEL);
	if (!xgold_gpio) {
		gpio_err("Can't alloc xgold_gpio\n");
		return -ENOMEM;
	}
	pchip = &xgold_gpio->pchip;
	ret = of_property_read_u32(np, GPIO_DT_PIN_BASE, &pchip->base);
	if (ret) {
		gpio_err("missing mdtry property:%s\n", GPIO_DT_PIN_BASE);
		goto error;
	}

	ret = of_property_read_u32(np, GPIO_DT_PIN_NUM, &ngpios);
	if (ret) {
		gpio_err("missing mandatory property:%s\n", GPIO_DT_PIN_NUM);
		goto error;
	}
	ret = of_property_read_u32(np, GPIO_TO_IRQ_NUM, &xgold_gpio->nirqs);
	if (ret) {
		gpio_info("missing property:%s\n", GPIO_TO_IRQ_NUM);
		pchip->to_irq = NULL;
		xgold_gpio->nirqs = 0;
	} else {
		if (xgold_gpio->nirqs > MAX_GPIO_IRQS) {
			gpio_info("maximum gpio irqs supported: %d\n",
				MAX_GPIO_IRQS);
			xgold_gpio->nirqs = MAX_GPIO_IRQS;
		}
		if (xgold_gpio->nirqs > 0) {
			ret = of_property_read_u32_array(np, GPIO_TO_IRQ,
				&tab_irq[0], xgold_gpio->nirqs * 2);
			if (ret) {
				gpio_err("error parsing %s property table\n",
					GPIO_TO_IRQ);
				goto error;
			}
		}
		for (i = 0; i < xgold_gpio->nirqs; i++) {
			xgold_gpio->gpio_irq[i].gpio = tab_irq[i * 2];
			xgold_gpio->gpio_irq[i].irq = tab_irq[i * 2 + 1];
		}
		pchip->to_irq = xgold_gpio_to_irq;
	}
	pchip->ngpio = ngpios;
	pchip->label = np->name;
	pchip->owner = THIS_MODULE;
	pchip->request = xgold_gpio_request;
	pchip->free = xgold_gpio_free;
	pchip->direction_input = xgold_gpio_direction_input;
	pchip->get = xgold_gpio_get;
	pchip->direction_output = xgold_gpio_direction_output;
	pchip->set = xgold_gpio_set;

	/* Static mapping, never released */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res)) {
		gpio_err("Invalid mem resource\n");
		return -ENODEV;
	}

/*
	 if (!devm_request_mem_region(&pdev->dev,
					res->start, resource_size(res),
				     pdev->name)) {
		gpio_err("Region already claimed\n");
		return -EBUSY;
	}
*/
	xgold_gpio->io_master = xgold_gpio_get_io_master(np);
	if (xgold_gpio->io_master == PCL_IO_ACCESS_BY_LNX) {
		xgold_gpio->gpio_base = devm_ioremap(&pdev->dev,
					res->start, resource_size(res));
		if (!xgold_gpio->gpio_base) {
			gpio_err("Could not ioremap\n");
			return -ENOMEM;
		}
		gpio_info("gpio: io: linux-@:%p\n",
				xgold_gpio->gpio_base);
	} else {
		xgold_gpio->gpio_base_phys = res->start;
		gpio_info("gpio: io: vmm-@:%pa\n",
				&xgold_gpio->gpio_base_phys);
	}

	gpio_info("XGold gpio probed base:%d num:%d gpios from %s\n",
		  pchip->base, pchip->ngpio, np->name);

	pchip->of_node = np;
	pchip->of_gpio_n_cells = 2;
	pchip->of_xlate = of_gpio_simple_xlate;
	gpiochip_add(pchip);
	platform_set_drvdata(pdev, xgold_gpio);
#ifdef CONFIG_X86_INTEL_SOFIA
	if (xgold_gpio->io_master == PCL_IO_ACCESS_BY_VMM) {
		if (mv_svc_pinctrl_service(PINCTRL_OPEN, 0, 0, 0))
			gpio_err("mv_svc_pinctrl_service PINCTRL_OPEN fails");
	} else
#endif
		pcl_write(pchip, 4, 0x100);
	/* read direction layout */
	ret = of_property_read_u32(np, PROP_GPIO_DIRECTION_OUT,
				&xgold_gpio->dir_out);
	if (ret) {
		gpio_err("Can't read property:%s\n", PROP_GPIO_DIRECTION_OUT);
		return -EINVAL;
	}
	ret = of_property_read_u32_array(np, PROP_GPIO_DIRECTION, array, 3);
	if (ret) {
		gpio_err("Can't read property:%s\n", PROP_GPIO_DIRECTION);
		return -EINVAL;
	}
	xgold_gpio->dir_base = array[0];
	tofield(&xgold_gpio->dir_field, array[1], array[2]);
	xgold_gpio->dir_field.name = "direction";

	/* read output layout */
	ret = of_property_read_u32_array(np, PROP_GPIO_OUTPUT, array, 3);
	if (ret) {
		gpio_err("Can't read property:%s\n", PROP_GPIO_OUTPUT);
		return -EINVAL;
	}
	xgold_gpio->out_base = array[0];
	tofield(&xgold_gpio->out_field, array[1], array[2]);
	xgold_gpio->out_field.name = "output";

	/* read input layout */
	ret = of_property_read_u32_array(np, PROP_GPIO_INPUT, array, 3);
	if (ret) {
		gpio_err("Can't read property:%s\n", PROP_GPIO_INPUT);
		return -EINVAL;
	}
	xgold_gpio->in_base = array[0];
	tofield(&xgold_gpio->in_field, array[1], array[2]);
	xgold_gpio->in_field.name = "input";

	if (of_find_property(np, "intel,lvds-ttl-as-gpio", NULL))
		pcl_write(pchip, 0xc, 0x4);

	return 0;

error:
	devm_kfree(&pdev->dev, pchip);
	return -EINVAL;

}

static struct platform_driver xgold_gpio_driver = {
	.driver = {
		   .name = "xgold-gpio",
		   .owner = THIS_MODULE,
		   .of_match_table = xgold_gpio_of_match,
		   },
	.probe = xgold_gpio_probe,
};

static int __init xgold_gpio_init(void)
{
	return platform_driver_register(&xgold_gpio_driver);
}

arch_initcall(xgold_gpio_init);

MODULE_DESCRIPTION("Intel XGold gpio driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, xgold_gpio_of_match);
