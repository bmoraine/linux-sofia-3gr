#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/reset-controller.h>

#if defined(CONFIG_X86_INTEL_SOFIA)
#include <sofia/mv_svc_hypercalls.h>
#endif

#define XGOLD_RESET_HAS_RW_REG			BIT(0)
#define XGOLD_RESET_HAS_WO_REG			BIT(1)
#define XGOLD_RESET_HAS_WC_REG			BIT(2)
#define XGOLD_RESET_HAS_RO_REG			BIT(3)
#define XGOLD_RESET_NATIVE_ACCESS		BIT(4)
#define XGOLD_RESET_USE_HYPERCALL_ACCESS	BIT(5)

struct xgold_reset_ctrl {
	struct reset_controller_dev rcdev;
	void __iomem *ctrl_io;
	uint32_t ctrl_io_phys;
	unsigned reg_rw;
	unsigned reg_wo;
	unsigned reg_wc;
	unsigned reg_ro;
	spinlock_t lock;
	unsigned flags;
};


static struct of_device_id xgold_reset_ids[] = {
	{ .compatible = "intel,xgold-reset" },
};

static int xgold_rst_write(struct xgold_reset_ctrl *xgrc,
			unsigned value,
			unsigned offset)
{
	int ret = 0;

	if (xgrc->flags & XGOLD_RESET_NATIVE_ACCESS)
		iowrite32(value, xgrc->ctrl_io + offset);
	else if (xgrc->flags & XGOLD_RESET_USE_HYPERCALL_ACCESS)
		ret = mv_svc_reg_write(xgrc->ctrl_io_phys + offset,
						value,
						-1);
	else
		ret = -EINVAL;

	return ret;
}

static int xgold_rst_write_only(struct xgold_reset_ctrl *xgrc,
			unsigned value,
			unsigned offset)
{
	int ret = 0;

	if (xgrc->flags & XGOLD_RESET_NATIVE_ACCESS)
		iowrite32(value, xgrc->ctrl_io + offset);
	else if (xgrc->flags & XGOLD_RESET_USE_HYPERCALL_ACCESS)
		ret = mv_svc_reg_write_only(xgrc->ctrl_io_phys + offset,
						value,
						-1);
	else
		ret = -EINVAL;

	return ret;
}


static int xgold_rst_read(struct xgold_reset_ctrl *xgrc,
			unsigned *value,
			unsigned offset)
{
	int ret = 0;

	if (xgrc->flags & XGOLD_RESET_NATIVE_ACCESS)
		*value = ioread32(xgrc->ctrl_io + offset);
	else if (xgrc->flags & XGOLD_RESET_USE_HYPERCALL_ACCESS)
		ret = mv_svc_reg_read(xgrc->ctrl_io_phys + offset,
						value,
						-1);
	else
		ret = -EINVAL;

	return ret;
}

static int xgold_rst_wait_for_update(struct xgold_reset_ctrl *xgrc,
					unsigned mask,
					unsigned value)
{
	int ret;
	unsigned reg, timeout = 50;

	do {
		if (timeout-- == 0) {
			ret = -ETIMEDOUT;
			pr_err("%s: Time out while polling for %x/%x reset status bit\n",
							__func__, mask, value);
			break;
		}

		if (xgrc->flags & XGOLD_RESET_HAS_RW_REG)
			ret = xgold_rst_read(xgrc, &reg, xgrc->reg_rw);
		else
			ret = xgold_rst_read(xgrc, &reg, xgrc->reg_ro);

		udelay(1);
	} while ((reg & mask) != value);

	return ret;
}


static int xgold_rst_assert(struct reset_controller_dev *rcdev,
							 unsigned long id)
{
	struct xgold_reset_ctrl *xgrc =
			container_of(rcdev, struct xgold_reset_ctrl, rcdev);
	uint32_t reg;
	unsigned long flags;
	int32_t ret = 0;

	spin_lock_irqsave(&xgrc->lock, flags);

#if defined(CONFIG_X86_INTEL_SOFIA)
	if (xgrc->flags & XGOLD_RESET_HAS_RW_REG) {
		ret = xgold_rst_read(xgrc, &reg, xgrc->reg_rw);
		if (ret)
			goto error;
		reg |= BIT(id);
		ret = xgold_rst_write(xgrc, reg, xgrc->reg_rw);
		if (ret)
			goto error;
	} else {
		reg = BIT(id);
		ret = xgold_rst_write_only(xgrc, reg, xgrc->reg_wo);
		if (ret)
			goto error;
	}

#endif
	ret = xgold_rst_wait_for_update(xgrc, BIT(id), BIT(id));
error:
	spin_unlock_irqrestore(&xgrc->lock, flags);
	return ret == 0 ? 0 : -EPERM;
}

static int xgold_rst_deassert(struct reset_controller_dev *rcdev,
							unsigned long id)
{
	struct xgold_reset_ctrl *xgrc =
			container_of(rcdev, struct xgold_reset_ctrl, rcdev);
	unsigned ret = 0, reg;
	unsigned long flags;

	spin_lock_irqsave(&xgrc->lock, flags);
#if defined(CONFIG_X86_INTEL_SOFIA)
	if (xgrc->flags & XGOLD_RESET_HAS_RW_REG) {
		ret = xgold_rst_read(xgrc, &reg, xgrc->reg_rw);
		if (ret)
			goto error;
		reg &= ~BIT(id);
		ret = xgold_rst_write(xgrc, reg, xgrc->reg_rw);
	} else {
		reg = BIT(id);
		ret = xgold_rst_write_only(xgrc, reg, xgrc->reg_wc);
	}

	ret = xgold_rst_wait_for_update(xgrc, BIT(id), 0);
#endif
error:
	spin_unlock_irqrestore(&xgrc->lock, flags);
	return ret == 0 ? 0 : -EPERM;
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

static inline bool xgold_reset_do_sanity(struct xgold_reset_ctrl *xgrc)
{
	if (xgrc->flags & XGOLD_RESET_HAS_RW_REG) {
		pr_info("%s: Using R/W register access scheme\n", __func__);
		return 0;
	}

	if (xgrc->flags & (XGOLD_RESET_HAS_WO_REG
				| XGOLD_RESET_HAS_WC_REG
				| XGOLD_RESET_HAS_RO_REG)) {
		pr_info("%s: Using WO/WC/RO register access scheme\n",
				__func__);
		return 0;
	}

	pr_err("%s: Invalid register access scheme (%x)\n",
			__func__, xgrc->flags);

	return -EINVAL;
}

static int xgold_reset_parse_dt(struct reset_controller_dev *rcdev)
{
	struct device_node *np = rcdev->of_node;
	struct xgold_reset_ctrl *xgrc =
			container_of(rcdev, struct xgold_reset_ctrl, rcdev);
	unsigned val;
	struct resource regs;

#define PROPERTY_MISSING "xgold-reset: \"%s\" property is missing.\n"
	if (!of_property_read_u32(np, "intel,reset-rw", &val)) {
		xgrc->flags |= XGOLD_RESET_HAS_RW_REG;
		xgrc->reg_rw = val;
	}

	if (!of_property_read_u32(np, "intel,reset-wo", &val)) {
		xgrc->flags |= XGOLD_RESET_HAS_WO_REG;
		xgrc->reg_wo = val;
	}

	if (!of_property_read_u32(np, "intel,reset-wc", &val)) {
		xgrc->flags |= XGOLD_RESET_HAS_WC_REG;
		xgrc->reg_wc = val;
	}

	if (!of_property_read_u32(np, "intel,reset-ro", &val)) {
		xgrc->flags |= XGOLD_RESET_HAS_RO_REG;
		xgrc->reg_ro = val;
	}

	if (xgold_reset_do_sanity(xgrc))
		return -EINVAL;

	if (of_address_to_resource(np, 0, &regs)) {
		pr_err(PROPERTY_MISSING, "reg");
		return -EINVAL;
	} else
		xgrc->ctrl_io_phys = regs.start;

	xgrc->ctrl_io = of_iomap(np, 0);
	if (!xgrc->ctrl_io) {
		pr_err("xgold-reset: I/O remapping failed");
		return -EINVAL;
	}

	if (of_find_property(np, "intel,vmm-secured-access", NULL))
		xgrc->flags |= XGOLD_RESET_USE_HYPERCALL_ACCESS;
	else
		xgrc->flags |= XGOLD_RESET_NATIVE_ACCESS;

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
