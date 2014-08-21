/****************************************************************
 *
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 *  This program is free software: you can redistribute	it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the	Free Software Foundation.
 *
 *  This program is distributed	in the hope that it will be useful,
 *  but	WITHOUT	ANY WARRANTY; without even the implied warranty	of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR	PURPOSE.
 *
 *  You	should have received a copy of the GNU General Public License Version 2
 *  along with this program. If	not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

/*----------------------------------------------------------------------*/
/* INCLUDE								*/
/*----------------------------------------------------------------------*/
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/reset.h>
#include "phy-intel-usb.h"
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

/*----------------------------------------------------------------------*/
/* DEFINE								*/
/*----------------------------------------------------------------------*/
#define INTEL_USB3_FORCE_VBUS
/*#define INTEL_USB3_HW_STUB*/
/*#define INTEL_USB3_WA_RST*/
/*#define INTEL_USB3_WA_PM*/

/**
 * todo: comment
 */
struct intel_usb3_bf {
	u32	addr;
	u32	mask;
};

/**
 * todo: comment
 */
struct intel_usb3 {
	struct platform_device	*pdev;
	struct intel_phy	iphy;
	void __iomem		*iomem;
	struct reset_control	*reset_usb_core;
	struct reset_control	*reset_usb_susp;
	/* todo: add ID pin irq for OTG support */
	unsigned		irq_usb_resume;
	u32			scu_usb_ss_trim[3];
	unsigned long		state;
#define POWERED			(0)
#define SUSPENDED		(1)
	struct regulator	*reg_iso;
	struct regulator	*reg_phy;
	struct regulator	*reg_core;
	struct regulator	*reg_dig;
	struct clk		*clk_kernel;
	struct clk		*clk_bus;
};

/*----------------------------------------------------------------------*/
/* CONSTANTS								*/
/*----------------------------------------------------------------------*/
/**
 * todo: comment
 */
static const struct intel_usb3_bf scu_dmpulldown	= {0x0610UL, BIT(28)};
static const struct intel_usb3_bf scu_freq_sel		= {0x0610UL, BIT(27)};
static const struct intel_usb3_bf scu_ref_ssp_en	= {0x0610UL, BIT(26)};
static const struct intel_usb3_bf scu_otgdisable	= {0x0610UL, BIT(17)};
static const struct intel_usb3_bf scu_commononn		= {0x0610UL, BIT(16)};
static const struct intel_usb3_bf scu_pipe3_pwr_present	= {0x0610UL, BIT(15)};
static const struct intel_usb3_bf scu_gen_stat_u2pmu	= {0x0610UL, BIT(14)};
static const struct intel_usb3_bf scu_gen_stat_u3pmu	= {0x0610UL, BIT(13)};
static const struct intel_usb3_bf scu_gen_u2pmu		= {0x0610UL, BIT(12)};
static const struct intel_usb3_bf scu_gen_u3pmu		= {0x0610UL, BIT(11)};
static const struct intel_usb3_bf scu_pm_pwr_state_req	= {0x0610UL, BIT(9)};
static const struct intel_usb3_bf scu_get_state_u3pmu	= {0x0620UL, BIT(18)};
static const struct intel_usb3_bf scu_get_state_u2pmu	= {0x0620UL, BIT(17)};
static const struct intel_usb3_bf scu_usb_ss_trim[] = {
	{0x0630UL, ~0},
	{0x0634UL, ~0},
	{0x0638UL, ~0}
};

/*----------------------------------------------------------------------*/
/* LOCAL - UTILITIES							*/
/*----------------------------------------------------------------------*/
/**
 * pdev is initialized at probe
 */
static inline struct device *intel_usb3_get_device(struct intel_usb3 *iusb3)
{
	return iusb3 ? &iusb3->pdev->dev : NULL;
}

/**
 * todo: comment
 */
static struct usb_gadget *intel_usb3_get_gadget(struct intel_usb3 *iusb3)
{
	return iusb3 ? intel_phy_get_gadget(&iusb3->iphy) : NULL;
}

/**
 * todo: comment
 */
static enum usb_device_speed intel_usb3_get_speed(struct intel_usb3 *iusb3)
{
	return iusb3 ? intel_phy_get_speed(&iusb3->iphy) : USB_SPEED_UNKNOWN;
}

/*----------------------------------------------------------------------*/
/* LOCAL - HW UTILITIES							*/
/*----------------------------------------------------------------------*/
/**
 * todo: comment
 */
static u32 intel_usb3_hw_bf_read(
	struct intel_usb3 *iusb3, const struct intel_usb3_bf *bf)
{
	u32 data = 0;

	if (IS_ERR_OR_NULL(iusb3) || IS_ERR_OR_NULL(bf)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	ioread32(iusb3->iomem + bf->addr);
	return (data & bf->mask) >> __ffs(bf->mask);
}

/**
 * attention: not reentrant
 */
static void intel_usb3_hw_bf_write(
	struct intel_usb3 *iusb3, const struct intel_usb3_bf *bf, u32 data)
{
	if (IS_ERR_OR_NULL(iusb3) || IS_ERR_OR_NULL(bf)) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}
	if (~bf->mask) {
		data = (data << __ffs(bf->mask)) & bf->mask;
		data |= ioread32(iusb3->iomem + bf->addr) & ~bf->mask;
	}
	iowrite32(data, iusb3->iomem + bf->addr);
}

/**
 * Requests D0/D3 power state and waits for state change
 */
static int intel_usb3_hw_req_pwr_state(struct intel_usb3 *iusb3, bool active)
{
	const struct intel_usb3_bf *bf = NULL;
	/* 0 -> D0 (active), 1 -> D3 (inactive) */
	unsigned state = active ? 0 : 1;


	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	intel_usb3_hw_bf_write(iusb3, &scu_pm_pwr_state_req, state);
	/* wait here until power state has been acknowledged */
	if (USB_SPEED_SUPER == intel_usb3_get_speed(iusb3))
		bf = &scu_get_state_u3pmu;
	else
		bf = &scu_get_state_u2pmu;
	while (state != intel_usb3_hw_bf_read(iusb3, bf))
		cpu_relax();
	return 0;
}

/**
 * todo: comment
 */
static int intel_usb3_hw_reset(struct intel_usb3 *iusb3, bool reset)
{
#ifdef INTEL_USB3_RST_WA
	static const struct intel_usb3_bf scu_ussv     = {0x0220UL, BIT(25)};
	static const struct intel_usb3_bf scu_uss_vaux = {0x0220UL, BIT(23)};
#endif
	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (reset) {
#ifdef INTEL_USB3_RST_WA
		intel_usb3_hw_bf_write(iusb3, &scu_ussv, 1);
		intel_usb3_hw_bf_write(iusb3, &scu_uss_vaux, 1);
#else
		reset_control_assert(iusb3->reset_usb_core);
		reset_control_assert(iusb3->reset_usb_susp);
#endif
	} else {
#ifdef INTEL_USB3_RST_WA
		intel_usb3_hw_bf_write(iusb3, &scu_uss_vaux, 0);
		intel_usb3_hw_bf_write(iusb3, &scu_ussv, 0);
#else
		reset_control_deassert(iusb3->reset_usb_susp);
		reset_control_deassert(iusb3->reset_usb_core);
#endif
	}

	return 0;
}

/**
 * todo: comment
 */
static int intel_usb3_pm_set_state(struct intel_usb3 *iusb3, const char *state)
{
	if (IS_ERR_OR_NULL(iusb3) || IS_ERR_OR_NULL(state)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
#ifdef INTEL_USB3_WA_PM
	intel_phy_warn("pm set state stubbed");
	return 0;
#endif
	return platform_device_pm_set_state_by_name(iusb3->pdev, state);
}

/*----------------------------------------------------------------------*/
/* LOCAL - HW CONFIGURATION						*/
/*----------------------------------------------------------------------*/
/**
 * initialize the SCU USB registers
 * - disables otg & select 26Mhz PHY ref clock
 * USB core not required to be powered for this operation
 */
static int intel_usb3_hw_init(struct intel_usb3 *iusb3)
{
	unsigned i = 0;


	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

#ifdef INTEL_USB3_HW_STUB
	intel_phy_warn("hw init stubbed");
	return 0;
#endif

	/* set the TRIM values */
	if (ARRAY_SIZE(scu_usb_ss_trim) != ARRAY_SIZE(iusb3->scu_usb_ss_trim)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	for (i = 0; i < ARRAY_SIZE(scu_usb_ss_trim); i++)
		intel_usb3_hw_bf_write(
			iusb3, &scu_usb_ss_trim[i], iusb3->scu_usb_ss_trim[i]);

	/* To save consumption the signal OTGDISABLE should be tied to high */
	intel_usb3_hw_bf_write(iusb3, &scu_otgdisable, 1);

	/* should be forced when no Vbus is connected to the PHY */
	/* TODO: understand the necessity of vbus force */
	intel_usb3_hw_bf_write(iusb3, &scu_pipe3_pwr_present, 1);

	/* signal forces HS bias & PLL block to be
	 * powered down in suspend state */
	intel_usb3_hw_bf_write(iusb3, &scu_commononn, 1);

	/* select the 26Mhz or 24Mhz input frequency of PHY reference clock */
	intel_usb3_hw_bf_write(iusb3, &scu_freq_sel, 0);
	return 0;
}

/**
 * Reset USB module and switch on all USB clock & power domains
 * before using USB controller
 * Asynchronous HW reset sequence of USB SS (HW-team recommendation)
 */
static int intel_usb3_hw_powerup(struct intel_usb3 *iusb3)
{
	int ret = 0;


	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (test_and_set_bit(POWERED, &iusb3->state)) {
		intel_phy_warn("already powered");
		return 0;
	}

#ifdef INTEL_USB3_HW_STUB
	intel_phy_warn("hw powerup stubbed");
	return 0;
#endif

	intel_usb3_hw_reset(iusb3, true);

	/* power up USB core & PHY */
	ret = intel_usb3_pm_set_state(iusb3, "enable");
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("set pm state enable, %d", ret);
		return intel_phy_kernel_trap();
	}

	/* TODO: check with vmm power driver team to confirm whether
	 * the delay is required here or they ensure the power up of
	 * the USB HW completely including delay */
	/* actual msleep(1~20) implementation may sleep until ~20ms
	 * for higher accuraty usleep_range should be used instead
	 * but it is built on top of hrtimers so extra processing
	 * HW recommends to wait 1ms for LDO stabilization but
	 * if driver waits for a longer time has no side effect
	 * so 1ms waiting for better msleep(1~20) implementation */
	msleep(1);

	/* signal forces HS bias & PLL block to be
	 * powered down in suspend state */
	intel_usb3_hw_bf_write(iusb3, &scu_commononn, 1);

	/* disable D- pull-down resistor */
	intel_usb3_hw_bf_write(iusb3, &scu_dmpulldown, 0);

	intel_usb3_hw_reset(iusb3, false);
	udelay(100);
	return 0;
}

/**
 * switch off USB clock and power domains when not needed
 */
static int intel_usb3_hw_powerdown(struct intel_usb3 *iusb3)
{
	int ret = 0;


	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (!test_and_clear_bit(POWERED, &iusb3->state)) {
		intel_phy_warn("already unpowered");
		return 0;
	}

#ifdef INTEL_USB3_HW_STUB
	intel_phy_warn("hw powerdown stubbed");
	return 0;
#endif

	/* enable D- pull-down resistor back */
	intel_usb3_hw_bf_write(iusb3, &scu_dmpulldown, 1);

	/* power off USB core, restore isolation */
	ret = intel_usb3_pm_set_state(iusb3, "disable");
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("set pm state disable, %d", ret);
		return intel_phy_kernel_trap();
	}
	return 0;
}

/* suspend/resume as hibernation feature does not come along
 * with driver so currently these suspend/wakeup functions
 * will not work
 *
 * The function switches off all USB clocks & power domains
 * that are not required during USB suspend.
 *
 * The sequence for handling the state transition to suspend
 * state is described both in the
 * "Sofia LTE SoC User Manual" spec (Figure 167) and in the
 * Synopsys DWC SS USB3.0 Databook V2.51 chapter 12.2.3.
 * The sequence is split in two parts due to the long time
 * the PMU of the Synopsys core requires to switch to
 * D3. In order not to poll, the pmenn interrupt is used to
 * get notified when D3 power state is reached.
 */
static int intel_usb3_hw_suspend(struct intel_usb3 *iusb3)
{
	int ret = 0;


	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter iusb3\n");
		return intel_phy_kernel_trap();
	}

	if (test_and_set_bit(SUSPENDED, &iusb3->state)) {
		intel_phy_warn("already suspended");
		return 0;
	}

#ifdef INTEL_USB3_HW_STUB
	intel_phy_warn("hw suspend stubbed");
	return 0;
#endif

	/* TODO: Add required CSR register save function callback */

	/* disable USB core IRQ */
	/* todo: Is it okay to disable only device interrupts? */
	/* TODO: should it be done in udc when hibernation support added */

	/* enable D- pull-down resistor back */
	intel_usb3_hw_bf_write(iusb3, &scu_dmpulldown, 1);

	/* request suspend power state */
	intel_usb3_hw_req_pwr_state(iusb3, false);

	/* enter in suspend state disabling 26Mhz phy reference clock */
	intel_usb3_hw_bf_write(iusb3, &scu_ref_ssp_en, 0);

	/* select the SCU pmenn source
	 * The interrupt is unmasked if a valid source is selected and
	 * it is masked if an invalid source is selected.
	 * The sources for pmenn interrupt are:
	 * - signal Pme_generation_u3pmu: indicates resume in USB3.0 mode
	 * - signal Pme_generation_u2pmu: indicates resume in USB2.0 mode
	 * - signal Current_power_state_u3pmu: indicates transition to D3
	 *   power state in USB3.0 mode
	 * - signal Current_power_state_u2pmu: indicates transition to D3
	 *   power state in USB2.0 mode
	 */
	if (USB_SPEED_SUPER == intel_usb3_get_speed(iusb3))
		intel_usb3_hw_bf_write(iusb3, &scu_gen_u3pmu, 1);
	else
		intel_usb3_hw_bf_write(iusb3, &scu_gen_u2pmu, 1);

	/* enable resume IRQ */
	if (device_may_wakeup(intel_usb3_get_device(iusb3)))
		enable_irq_wake(iusb3->irq_usb_resume);

	/* TODO: what if wakeup interrupt occurs at this point? */
	/* power off USB core in suspend state */
	ret = intel_usb3_pm_set_state(iusb3, "suspend");
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("set pm state suspend, %d", ret);
		return intel_phy_kernel_trap();
	}
	return 0;
}

/**
 * switches on all USB clocks and USB power domains
 */
static int intel_usb3_hw_wakeup(struct intel_usb3 *iusb3)
{
	int ret = 0;


	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (!test_and_clear_bit(SUSPENDED, &iusb3->state)) {
		intel_phy_warn("already waked up");
		return 0;
	}

#ifdef INTEL_USB3_HW_STUB
	intel_phy_warn("hw wakeup stubbed");
	return 0;
#endif

	/* disable the pmenn source */
	if (device_may_wakeup(intel_usb3_get_device(iusb3)))
		disable_irq_wake(iusb3->irq_usb_resume);

	/* switch on VCC supply of USB core & remove isolation */
	ret = intel_usb3_pm_set_state(iusb3, "enable");
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("set pm state enable iso, %d", ret);
		return intel_phy_kernel_trap();
	}

	/* leave in suspend state enabling 26Mhz phy reference clock */
	intel_usb3_hw_bf_write(iusb3, &scu_ref_ssp_en, 1);

	/* request wakeup power state */
	intel_usb3_hw_req_pwr_state(iusb3, true);

	/* Disable D- pull-down resistor back */
	intel_usb3_hw_bf_write(iusb3, &scu_dmpulldown, 0);

	/* disable USB core IRQ */
	/* todo: is it okay to disable only device interrupts?  */

	/* TODO: Add required CSR registers restore function callback */
	return 0;
}

/*----------------------------------------------------------------------*/
/* MODULE INTERFACE - OTG FSM						*/
/*----------------------------------------------------------------------*/
/**
 * host starts enumerating peripheral
 */
int usb_bus_start_enum(struct usb_bus *bus, unsigned port_num)
{
	intel_phy_warn("implement for OTG / HOST support");
	return 0;
}

/**
 * host suspend/resume
 */
static int intel_usb3_otg_fsm_start_host(struct otg_fsm *fsm, int on)
{
	if (IS_ERR_OR_NULL(fsm)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	intel_phy_warn("todo: implementation");
	return 0;
}

/**
 * device suspend/resume
 */
static int intel_usb3_otg_fsm_start_gadget(struct otg_fsm *fsm, int on)
{
	struct intel_phy *iphy = container_of(fsm, struct intel_phy, fsm);
	struct intel_usb3 *iusb3 = container_of(iphy, struct intel_usb3, iphy);
	struct usb_gadget *gadget = intel_usb3_get_gadget(iusb3);


	if (IS_ERR_OR_NULL(fsm)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (IS_ERR_OR_NULL(gadget)) {
		intel_phy_warn("gadget not bind");
		return 0;
	}

	intel_phy_info("gadget %s", on ? "on" : "off");
	if (on) {
		/* do power up of USB core & PHY
		 * required for initializing USB stack */
		intel_usb3_hw_powerup(iusb3);
		/* TODO: Check whether using pull-up is enough to
		 * use on HW*/
#ifndef INTEL_USB3_FORCE_VBUS
		/* vbus session tested on VP using debug interface
		 * but currently disabled for the HW bring up as force vbus
		 * approach is followed during boot up */
		usb_gadget_vbus_connect(gadget);
#endif
	} else {
		/* TODO: Check whether using pull-up is enough to use */
#ifndef INTEL_USB3_FORCE_VBUS
		usb_gadget_vbus_disconnect(gadget);
#endif
		intel_usb3_hw_powerdown(iusb3);
	}
	return 0;
}

/**
 * @todo: comment
 */
static struct otg_fsm_ops intel_usb3_otg_fsm_ops = {
	.start_host = intel_usb3_otg_fsm_start_host,
	.start_gadget = intel_usb3_otg_fsm_start_gadget,
};

/*----------------------------------------------------------------------*/
/* MODULE INTERFACE - POWER MANAGEMENT					*/
/*----------------------------------------------------------------------*/
#ifdef CONFIG_PM_RUNTIME
/**
 * @todo: comment
 */
static int intel_usb3_runtime_suspend(struct device *dev)
{
	if (IS_ERR_OR_NULL(dev)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	return 0;
}

/**
 * @todo: comment
 */
static int intel_usb3_runtime_resume(struct device *dev)
{
	if (IS_ERR_OR_NULL(dev)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	return 0;
}

/**
 * @todo: comment
 */
static int intel_usb3_runtime_idle(struct device *dev)
{
	if (IS_ERR_OR_NULL(dev)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
/**
 * @todo: comment
 */
static int intel_usb3_system_sleep_suspend(struct device *dev)
{
	if (IS_ERR_OR_NULL(dev)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	return 0;
}

/**
 * @todo: comment
 */
static int intel_usb3_system_sleep_resume(struct device *dev)
{
	if (IS_ERR_OR_NULL(dev)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	return 0;
}
#endif

#ifdef CONFIG_PM
/**
 * @todo: comment
 */
static const struct dev_pm_ops intel_usb3_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(intel_usb3_runtime_suspend,
			   intel_usb3_runtime_resume,
			   intel_usb3_runtime_idle)
	SET_SYSTEM_SLEEP_PM_OPS(intel_usb3_system_sleep_suspend,
				intel_usb3_system_sleep_resume)
};
#endif

/*----------------------------------------------------------------------*/
/* MODULE INTERFACE - DRIVER						*/
/*----------------------------------------------------------------------*/
/**
 * @todo: comment
 */
static irqreturn_t intel_usb3_isr_usb_resume(int irq, void *dev_id)
{
	struct intel_usb3 *iusb3 = NULL;


	if (IS_ERR_OR_NULL(dev_id)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	iusb3 = dev_id;

	if (device_may_wakeup(intel_usb3_get_device(iusb3)))
		disable_irq_wake(iusb3->irq_usb_resume);

	intel_phy_warn("todo: implementation");

	return IRQ_HANDLED;
}

/**
 * responsible for
 * - allocate and initialize data
 * - get mandatory DTS information
 * - request platform resources
 * - enable dynamic handling
 */
static int intel_usb3_probe(struct platform_device *pdev)
{
	struct intel_usb3 *iusb3 = NULL;
	struct resource *res = NULL;
	int ret = 0;


	if (IS_ERR_OR_NULL(pdev)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	/* allocate space to intel phy device */
	iusb3 = devm_kzalloc(&pdev->dev,
				sizeof(struct intel_usb3), GFP_KERNEL);
	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("memory allocation failed");
		return intel_phy_kernel_trap();
	}
	platform_set_drvdata(pdev, iusb3);
	iusb3->pdev = pdev;

	/* map scu into io memory */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "scu_usb");
	if (IS_ERR_OR_NULL(res)) {
		intel_phy_err("platform resource iomem not found");
		goto error_kfree;
	}
	iusb3->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(iusb3->iomem)) {
		intel_phy_err("ioremap device resource");
		goto error_kfree;
	}

	iusb3->reset_usb_core = reset_control_get(&pdev->dev, "usb_core");
	if (IS_ERR_OR_NULL(iusb3->reset_usb_core)) {
		intel_phy_err("reset control usb core not found");
		goto error_iounmap;
	}

	iusb3->reset_usb_susp = reset_control_get(&pdev->dev, "usb_susp");
	if (IS_ERR_OR_NULL(iusb3->reset_usb_susp)) {
		intel_phy_err("reset control usb susp not found");
		goto error_iounmap;
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"scu_usb_ss_trim",
			iusb3->scu_usb_ss_trim,
			ARRAY_SIZE(iusb3->scu_usb_ss_trim));
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("DTS TRIM values property read, %d", ret);
		goto error_iounmap;
	}
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	iusb3->reg_dig = regulator_get(&pdev->dev, "digital");
	if (IS_ERR(iusb3->reg_dig)) {
		intel_phy_err("can't get dig regulator");
		goto error_iounmap;
	}
	regulator_enable(iusb3->reg_dig);

	iusb3->reg_phy = regulator_get(&pdev->dev, "phy");
	if (IS_ERR(iusb3->reg_phy)) {
		intel_phy_err("can't get phy regulator");
		goto error_iounmap;
	}
	regulator_enable(iusb3->reg_phy);

	iusb3->reg_iso = regulator_get(&pdev->dev, "iso");
	if (IS_ERR(iusb3->reg_iso)) {
		intel_phy_err("can't get iso regulator");
		goto error_iounmap;
	}
	regulator_enable(iusb3->reg_iso);

	iusb3->reg_core = regulator_get(&pdev->dev, "core");
	if (IS_ERR(iusb3->reg_core)) {
		intel_phy_err("can't get core regulator");
		goto error_iounmap;
	}
	regulator_enable(iusb3->reg_core);

	iusb3->clk_kernel = of_clk_get_by_name(pdev->dev.of_node, "clk_kernel");
	if (IS_ERR(iusb3->clk_kernel)) {
		intel_phy_err("Clk kernel not found\n");
	}
	iusb3->clk_bus = of_clk_get_by_name(pdev->dev.of_node, "clk_bus");
	if (IS_ERR(iusb3->clk_bus)) {
		intel_phy_err("Clk bus not found\n");
	}

	clk_prepare_enable(iusb3->clk_bus);
	clk_prepare_enable(iusb3->clk_kernel);
#endif
	/* todo: OTG wakelock does not support USB3 so USB2 */
	ret = intel_phy_init(&pdev->dev, &iusb3->iphy,
				&intel_usb3_otg_fsm_ops, USB_PHY_TYPE_USB2);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("intel probe, %d", ret);
		goto error_clkput;
	}

	/* required only during boot time */
	intel_usb3_hw_init(iusb3);

	/* required for udc driver probing */
	intel_usb3_hw_powerup(iusb3);

	/* allocate resume interrupt */
	iusb3->irq_usb_resume = platform_get_irq_byname(pdev, "usb_resume");
	if (IS_ERR_VALUE(iusb3->irq_usb_resume)) {
		intel_phy_err("platform resource irq usb resume not found");
		goto error_phy_exit;
	}
	ret = devm_request_irq(&pdev->dev, iusb3->irq_usb_resume,
				intel_usb3_isr_usb_resume, IRQF_SHARED,
				"usb_resume", iusb3);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("request irq%d, %d", iusb3->irq_usb_resume, ret);
		goto error_phy_exit;
	}

#ifdef INTEL_USB3_FORCE_VBUS
	intel_phy_warn("forced cable attach for hardware bring up");
	intel_phy_notify(&iusb3->iphy, USB_EVENT_VBUS, NULL);
#endif

	return 0;

error_phy_exit:
	intel_phy_exit(&pdev->dev, &iusb3->iphy);
error_clkput:
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	clk_put(iusb3->clk_kernel);
	clk_put(iusb3->clk_bus);
#endif
error_iounmap:
	devm_iounmap(&pdev->dev, iusb3->iomem);
error_kfree:
	devm_kfree(&pdev->dev, iusb3);
	return intel_phy_kernel_trap();
}

/**
 * @todo: comment
 */
static int intel_usb3_remove(struct platform_device *pdev)
{
	struct intel_usb3 *iusb3 = NULL;


	if (IS_ERR_OR_NULL(pdev)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	iusb3 = platform_get_drvdata(pdev);

	devm_free_irq(&pdev->dev, iusb3->irq_usb_resume, iusb3);
	intel_phy_exit(&pdev->dev, &iusb3->iphy);
	devm_iounmap(&pdev->dev, iusb3->iomem);
	devm_kfree(&pdev->dev, iusb3);

	return 0;
}

/**
 * @todo: comment
 */
static const struct of_device_id intel_usbphy_dt_match[] = {
	{
		.compatible = "intel,phy-usb3",
	},
	{},
};
MODULE_DEVICE_TABLE(of, intel_usbphy_dt_match);

static struct platform_driver intel_usb3_driver = {
	.probe		= intel_usb3_probe,
	.remove		= intel_usb3_remove,
	.driver		= {
		.name	= "intel-phy-usb3",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &intel_usb3_dev_pm_ops,
#endif
		.of_match_table = of_match_ptr(intel_usbphy_dt_match),
	},
};

/**
 * @todo: comment
 */
static int __init intel_usb3_driver_init(void)
{
	return platform_driver_register(&intel_usb3_driver);
}
fs_initcall(intel_usb3_driver_init);

/**
 * @todo: comment
 */
static void __exit intel_usb3_driver_exit(void)
{
	platform_driver_unregister(&intel_usb3_driver);
}
module_exit(intel_usb3_driver_exit);

MODULE_LICENSE("GPLv2");
MODULE_ALIAS("platform:intel-phy-usb3");
