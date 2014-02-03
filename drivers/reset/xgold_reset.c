#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/reset-controller.h>

enum xgold_reset_write_mode {
	XGOLD_RESET_USE_RW_REG = 0,
	XGOLD_RESET_USE_SET_CLEAR_REG,
};

struct xgold_reset_ctrl {
	struct reset_controller_dev rcdev;
	void __iomem *ctrl_io;
	unsigned reg_status;
	unsigned reg_set;
	unsigned reg_clear;
	spinlock_t lock;
	int write_mode;
};


static struct of_device_id xgold_reset_ids[] = {
	{ .compatible = "intel,xgold-reset" },
};

static int xgold_rst_assert(struct reset_controller_dev *rcdev,
							 unsigned long id)
{
	unsigned reg;
	unsigned long flags;
	struct xgold_reset_ctrl *xgrc =
			container_of(rcdev, struct xgold_reset_ctrl, rcdev);

	spin_lock_irqsave(&xgrc->lock, flags);
	if (xgrc->write_mode == XGOLD_RESET_USE_RW_REG) {
		reg = ioread32(xgrc->ctrl_io + xgrc->reg_set);
		reg |= BIT(id);
	} else { /* XGOLD_RESET_USE_SET_CLEAR_REG */
		reg = BIT(id);
	}
	iowrite32(reg, xgrc->ctrl_io + xgrc->reg_set);
	spin_unlock_irqrestore(&xgrc->lock, flags);

	return 0;
}

static int xgold_rst_deassert(struct reset_controller_dev *rcdev,
							unsigned long id)
{
	unsigned reg;
	unsigned long flags;

	struct xgold_reset_ctrl *xgrc =
			container_of(rcdev, struct xgold_reset_ctrl, rcdev);

	spin_lock_irqsave(&xgrc->lock, flags);

	if (xgrc->write_mode == XGOLD_RESET_USE_RW_REG) {
		reg = ioread32(xgrc->ctrl_io + xgrc->reg_set);
		reg &= ~BIT(id);
		iowrite32(reg, xgrc->ctrl_io + xgrc->reg_set);
	} else { /* XGOLD_RESET_USE_SET_CLEAR_REG */
		reg = BIT(id);
		iowrite32(reg, xgrc->ctrl_io + xgrc->reg_clear);
	}

	spin_unlock_irqrestore(&xgrc->lock, flags);

	return 0;
}

static int xgold_rst_reset(struct reset_controller_dev *rcdev, unsigned long id)
{

	xgold_rst_assert(rcdev, id);
	udelay(1000);
	xgold_rst_deassert(rcdev, id);
	udelay(1000);

	return 0;
}

static struct reset_control_ops xgold_rst_ctrl_ops = {
	.reset = xgold_rst_reset,
	.assert = xgold_rst_assert,
	.deassert = xgold_rst_deassert,
};

static int xgold_reset_parse_dt(struct reset_controller_dev *rcdev)
{
	struct device_node *np = rcdev->of_node;
	struct xgold_reset_ctrl *xgrc =
			container_of(rcdev, struct xgold_reset_ctrl, rcdev);
	unsigned val;

#define PROPERTY_MISSING "xgold-reset: \"%s\" property is missing.\n"

	if (of_property_read_u32(np, "intel,reset-set", &val)) {
		pr_err(PROPERTY_MISSING, "intel,reset-set");
		return -EINVAL;
	}

	xgrc->reg_set = val;

	if (of_property_read_u32(np, "intel,reset-status", &val)) {
		pr_err(PROPERTY_MISSING, "intel,reset-status");
		return -EINVAL;
	}

	xgrc->reg_status = val;

	xgrc->ctrl_io = of_iomap(np, 0);
	if (!xgrc->ctrl_io) {
		pr_err("xgold-reset: I/O remapping failed");
		return -EINVAL;
	}

	/* Optional parameters */
	if (of_property_read_u32(np, "intel,reset-clear", &val)) {
		xgrc->write_mode = XGOLD_RESET_USE_RW_REG;
	} else {
		xgrc->write_mode = XGOLD_RESET_USE_SET_CLEAR_REG;
		xgrc->reg_clear = val;
	}

	return 0;
}

static int xgold_reset_register(struct device_node *np)
{
	int ret;
	struct xgold_reset_ctrl *xgrc;
	struct reset_controller_dev *rcdev;

	xgrc = kzalloc(sizeof(*xgrc), GFP_KERNEL);

	if (!xgrc)
		return -ENOMEM;

	rcdev = &xgrc->rcdev;
	rcdev->of_node = np;
	rcdev->ops = &xgold_rst_ctrl_ops;
	rcdev->nr_resets = 32;

	ret = xgold_reset_parse_dt(rcdev);
	if (ret) {
		ret = -EINVAL;
		goto free_ctrl;
	}

	spin_lock_init(&xgrc->lock);

	pr_info("Registering Xgold Reset controller @%p\n", xgrc->ctrl_io);

	ret = reset_controller_register(rcdev);
	if (!ret)
		return 0;

free_ctrl:
	kfree(xgrc);
	return ret;
}

int xgold_init_reset(void)
{
	unsigned i;
	struct device_node *np, *from = NULL;

	pr_info("Initializing Xgold Reset controller driver.\n");
	for (i = 0; i < ARRAY_SIZE(xgold_reset_ids); i++) {
		while ((np = of_find_matching_node(from,
					&xgold_reset_ids[i])) != NULL) {
			xgold_reset_register(np);
			from = np;
		}
	}

	return 0;
}
early_initcall(xgold_init_reset);
