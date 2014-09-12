/*
 * rk818-irq.c
 *
 * Author: zhangqing <zhangqing@rock-chips.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/bug.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mfd/rk818.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/irqdomain.h>
#include <linux/regmap.h>

#define VOUT_LO_INT 0
#define VB_LO_INT 296
#define PWRON_INT 0
#define PWRON_LP_INT 0
#define HOTDIE_INT 0
#define RTC_ALARM_INT 0
#define RTC_PERIOD_INT 0
#define USB_OV_INT 0

#define PLUG_IN_INT 297
#define PLUG_OUT_INT 298
#define CHGOK_INT 0
#define CHGTE_INT 0
#define CHGTS1_INT 0
#define TS2_INT 0
#define CHG_CVTLIM_INT 0
#define DISCHG_ILIM_INT 0
static int rk818_irq_num[16] = {
VOUT_LO_INT,
VB_LO_INT,
PWRON_INT,
PWRON_LP_INT,
HOTDIE_INT,
RTC_ALARM_INT,
RTC_PERIOD_INT,
USB_OV_INT,
PLUG_IN_INT,
PLUG_OUT_INT,
CHGOK_INT,
CHGTE_INT,
CHGTS1_INT,
TS2_INT,
CHG_CVTLIM_INT,
DISCHG_ILIM_INT
};

static inline int irq_to_rk818_irq(struct rk818 *rk818,
							int irq)
{
	return irq - rk818->chip_irq;
}

/*
 * This is a threaded IRQ handler so can access I2C/SPI.  Since all
 * interrupts are clear on read the IRQ line will be reasserted and
 * the physical IRQ will be handled again if another interrupt is
 * asserted while we run - in the normal course of events this is a
 * rare occurrence so we save I2C/SPI reads.  We're also assuming that
 * it's rare to get lots of interrupts firing simultaneously so try to
 * minimise I/O.
 */
static irqreturn_t rk818_irq(int irq, void *irq_data)
{
	struct rk818 *rk818 = irq_data;
	u32 irq_sts;
	u32 irq_mask;
	u8 reg;
	int i;

	wake_lock(&rk818->irq_wake);
	rk818_i2c_read(rk818, RK818_INT_STS_REG1, 1, &reg);
	irq_sts = reg;
	rk818_i2c_read(rk818, RK818_INT_STS_REG2, 1, &reg);
	irq_sts |= reg << 8;

	rk818_i2c_read(rk818, RK818_INT_STS_MSK_REG1, 1, &reg);
	irq_mask = reg;
	rk818_i2c_read(rk818, RK818_INT_STS_MSK_REG2, 1, &reg);
	irq_mask |= reg << 8;

	irq_sts &= ~irq_mask;

	pr_debug("irq_sts = 0x%2x, irq_mask = 0x%2x\n", irq_sts, irq_mask);

	if (!irq_sts) {
		wake_unlock(&rk818->irq_wake);
		return IRQ_NONE;
	}

	for (i = 0; i < rk818->irq_num; i++) {

		if (!(irq_sts & (1 << i)))
			continue;

		handle_nested_irq(rk818_irq_num[i]);
	}

	/* Write the STS register back to clear IRQs we handled */
	reg = irq_sts & 0xFF;
	irq_sts >>= 8;
	rk818_i2c_write(rk818, RK818_INT_STS_REG1, 1, reg);
	reg = irq_sts & 0xFF;
	rk818_i2c_write(rk818, RK818_INT_STS_REG2, 1, reg);
	wake_unlock(&rk818->irq_wake);

	return IRQ_HANDLED;
}

static void rk818_irq_lock(struct irq_data *data)
{
	struct rk818 *rk818 = irq_data_get_irq_chip_data(data);

	mutex_lock(&rk818->irq_lock);
}

static void rk818_irq_sync_unlock(struct irq_data *data)
{
	struct rk818 *rk818 = irq_data_get_irq_chip_data(data);
	u32 reg_mask;
	u8 reg;

	rk818_i2c_read(rk818, RK818_INT_STS_MSK_REG1, 1, &reg);
	reg_mask = reg;
	rk818_i2c_read(rk818, RK818_INT_STS_MSK_REG2, 1, &reg);
	reg_mask |= reg << 8;

	if (rk818->irq_mask != reg_mask) {
		reg = rk818->irq_mask & 0xff;
		reg = rk818->irq_mask >> 8 & 0xff;
	}
	mutex_unlock(&rk818->irq_lock);
}

static void rk818_irq_enable(struct irq_data *data)
{
	struct rk818 *rk818 = irq_data_get_irq_chip_data(data);

	rk818->irq_mask &= ~(1 << irq_to_rk818_irq(rk818, data->irq));
}

static void rk818_irq_disable(struct irq_data *data)
{
	struct rk818 *rk818 = irq_data_get_irq_chip_data(data);

	rk818->irq_mask |= (1 << irq_to_rk818_irq(rk818, data->irq));
}

#ifdef CONFIG_PM_SLEEP
static int rk818_irq_set_wake(struct irq_data *data, unsigned int enable)
{
	struct rk818 *rk818 = irq_data_get_irq_chip_data(data);
	return irq_set_irq_wake(rk818->chip_irq, enable);
}
#else
#define rk818_irq_set_wake NULL
#endif

static struct irq_chip rk818_irq_chip = {
	.name = "rk818",
	.irq_bus_lock = rk818_irq_lock,
	.irq_bus_sync_unlock = rk818_irq_sync_unlock,
	.irq_disable = rk818_irq_disable,
	.irq_enable = rk818_irq_enable,
	.irq_set_wake = rk818_irq_set_wake,
};

static int rk818_irq_domain_map(struct irq_domain *d, unsigned int irq,
					irq_hw_number_t hw)
{
	struct rk818 *rk818 = d->host_data;

	irq_set_chip_data(irq, rk818);
	irq_set_chip_and_handler(irq, &rk818_irq_chip, handle_edge_irq);
	irq_set_nested_thread(irq, 1);
#ifdef CONFIG_ARM
	set_irq_flags(irq, IRQF_VALID);
#else
	irq_set_noprobe(irq);
#endif
	return 0;
}

static struct irq_domain_ops rk818_irq_domain_ops = {
	.map = rk818_irq_domain_map,
};

int rk818_irq_init(struct rk818 *rk818, int irq, struct rk818_board *pdata)
{
	struct irq_domain *domain;
	int ret, flags = 0;
	u8 reg;

	if (!irq) {
		dev_warn(rk818->dev, "No interrupt support, no core IRQ\n");
		return 0;
	}

	/* Clear unattended interrupts */
	rk818_i2c_read(rk818, RK818_INT_STS_REG1, 1, &reg);
	rk818_i2c_write(rk818, RK818_INT_STS_REG1, 1, reg);
	rk818_i2c_read(rk818, RK818_INT_STS_REG2, 1, &reg);
	rk818_i2c_write(rk818, RK818_INT_STS_REG2, 1, reg);
	rk818_i2c_read(rk818, RK818_RTC_STATUS_REG, 1, &reg);
	/*clear alarm and timer interrupt*/
	rk818_i2c_write(rk818, RK818_RTC_STATUS_REG, 1, reg);

	/* Mask top level interrupts */
	rk818->irq_mask = 0xFFFFFF;
	mutex_init(&rk818->irq_lock);
	wake_lock_init(&rk818->irq_wake, WAKE_LOCK_SUSPEND, "rk818_irq_wake");
	rk818->irq_num = RK818_NUM_IRQ;
	rk818->chip_irq = pdata->irq;

	domain = irq_domain_add_linear(NULL, RK818_NUM_IRQ,
					&rk818_irq_domain_ops, rk818);
	if (!domain) {
		dev_err(rk818->dev, "could not create irq domain\n");
		return -ENODEV;
	}
	rk818->irq_domain = domain;


	ret = request_threaded_irq(rk818->chip_irq, NULL,
			rk818_irq, flags | IRQF_ONESHOT, "rk818", rk818);

	if (ret != 0)
		dev_err(rk818->dev, "Failed to request IRQ: %d\n", ret);

	return ret;
}

int rk818_irq_exit(struct rk818 *rk818)
{
	if (rk818->chip_irq)
		free_irq(rk818->chip_irq, rk818);
	return 0;
}
