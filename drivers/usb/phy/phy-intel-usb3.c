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
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/usb/phy-intel.h>
#include <linux/usb/debug.h>
#include <linux/usb/dwc3_extension.h>
#include <linux/usb/gadget_ioctl.h>
#include "phy-intel-usb.h"

/*----------------------------------------------------------------------*/
/* DEFINE								*/
/*----------------------------------------------------------------------*/
/*#define INTEL_USB3_FORCE_VBUS*/
#ifdef INTEL_USB3_FORCE_VBUS
#include <sofia/vmm_pmic.h>
#endif
/*#define INTEL_USB3_HW_STUB*/
/*#define INTEL_USB3_RST_WA*/
/*#define INTEL_USB3_PM_WA*/

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
	void __iomem		*iomem_scu;
	void __iomem		*iomem_lmu;
	dma_addr_t		dma_map_lmu;
	struct reset_control	*reset_usb_core;
	struct reset_control	*reset_usb_susp;
	/* todo: add ID pin irq for OTG support */
	unsigned		irq_usb_resume;
	u32			scu_usb_ss_trim[3];
	unsigned long		state;
#define POWERED			(0)
#define SUSPENDED		(1)
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	struct regulator	*reg_iso;
	struct regulator	*reg_phy;
	struct regulator	*reg_core;
	struct regulator	*reg_dig;
	struct clk		*clk_kernel;
	struct clk		*clk_bus;
#endif
	struct dwc3_ebc_ext	ebc_ext;
	struct dentry		*debugfs_root;
};

/*----------------------------------------------------------------------*/
/* CONSTANTS								*/
/*----------------------------------------------------------------------*/
#define INTEL_USB3_MPLL_LOOP_CTL	(0x0030UL)
#define INTEL_USB3_LMU_TRB_CONFIG	(0x0040UL)

/**
 * todo: comment
 */
static const struct intel_usb3_bf scu_dppulldown	= {0x0610UL, BIT(29)};
static const struct intel_usb3_bf scu_dmpulldown	= {0x0610UL, BIT(28)};
static const struct intel_usb3_bf scu_freq_sel		= {0x0610UL, BIT(27)};
static const struct intel_usb3_bf scu_ref_ssp_en	= {0x0610UL, BIT(26)};
static const struct intel_usb3_bf scu_trace_mux		= {0x0610UL,
							     GENMASK(22, 18)};
static const struct intel_usb3_bf scu_otgdisable	= {0x0610UL, BIT(17)};
static const struct intel_usb3_bf scu_commononn		= {0x0610UL, BIT(16)};
static const struct intel_usb3_bf scu_pipe3_pwr_present	= {0x0610UL, BIT(15)};
static const struct intel_usb3_bf scu_gen_stat_u2pmu	= {0x0610UL, BIT(14)};
static const struct intel_usb3_bf scu_gen_stat_u3pmu	= {0x0610UL, BIT(13)};
static const struct intel_usb3_bf scu_gen_u2pmu		= {0x0610UL, BIT(12)};
static const struct intel_usb3_bf scu_gen_u3pmu		= {0x0610UL, BIT(11)};
static const struct intel_usb3_bf scu_pm_pwr_state_req	= {0x0610UL, BIT(9)};
static const struct intel_usb3_bf scu_ssx		= {0x0610UL, BIT(5)};
static const struct intel_usb3_bf scu_cr_write		= {0x0610UL, BIT(3)};
static const struct intel_usb3_bf scu_cr_read		= {0x0610UL, BIT(2)};
static const struct intel_usb3_bf scu_cr_cap_data	= {0x0610UL, BIT(1)};
static const struct intel_usb3_bf scu_cr_cap_addr	= {0x0610UL, BIT(0)};
static const struct intel_usb3_bf scu_cr_data_in	= {0x0614UL, 0x0FFFFUL};
static const struct intel_usb3_bf scu_get_state_u3pmu	= {0x0620UL, BIT(18)};
static const struct intel_usb3_bf scu_get_state_u2pmu	= {0x0620UL, BIT(17)};
static const struct intel_usb3_bf scu_cr_ack		= {0x0620UL, BIT(21)};
static const struct intel_usb3_bf scu_cr_data_out	= {0x0624UL, 0x0FFFFUL};
static const struct intel_usb3_bf scu_usb_ss_trim[] = {
	{0x0630UL, ~0},
	{0x0634UL, ~0},
	{0x0638UL, ~0}
};

/*----------------------------------------------------------------------*/
/* PROTOTYPES								*/
/*----------------------------------------------------------------------*/
static int intel_usb3_debugfs_init(struct intel_usb3 *iusb3);
static int intel_usb3_debugfs_exit(struct intel_usb3 *iusb3);

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
static struct usb_bus *intel_usb3_get_host(struct intel_usb3 *iusb3)
{
	return iusb3 ? intel_phy_get_host(&iusb3->iphy) : NULL;
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

	data = ioread32(iusb3->iomem_scu + bf->addr);
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
		data |= ioread32(iusb3->iomem_scu + bf->addr) & ~bf->mask;
	}
	iowrite32(data, iusb3->iomem_scu + bf->addr);
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

#ifdef INTEL_USB3_PM_WA
	intel_phy_warn("pm set state stubbed");
	return 0;
#endif

	return platform_device_pm_set_state_by_name(iusb3->pdev, state);
}

/*----------------------------------------------------------------------*/
/* LOCAL - PHY UTILITIES						*/
/*----------------------------------------------------------------------*/
/**
 * @todo: comment
 */
static int intel_usb3_phy_hw_wait_ack(struct intel_usb3 *iusb3)
{
	unsigned i = 0;


	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	for (i = 10000; i > 0; i--) {
		if (intel_usb3_hw_bf_read(iusb3, &scu_cr_ack) != 0)
			break;
		cpu_relax();
	}
	if (0 == i) {
		intel_phy_err("phy did not ack");
		return intel_phy_kernel_trap();
	}
	return 0;
}

/**
 * @todo: comment
 */
static int intel_usb3_phy_hw_set_addr(struct intel_usb3 *iusb3, int addr)
{
	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	intel_usb3_hw_bf_write(iusb3, &scu_cr_data_in, addr);
	intel_usb3_hw_bf_write(iusb3, &scu_cr_cap_addr, 1);
	intel_usb3_phy_hw_wait_ack(iusb3);
	intel_usb3_hw_bf_write(iusb3, &scu_cr_cap_addr, 0);
	return 0;
}

/**
 * @todo: comment
 * for future struct usb_phy_io_ops.read implementation
 */
static int intel_usb3_phy_io_read(struct intel_usb3 *iusb3, u32 reg)
{
	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	intel_usb3_phy_hw_set_addr(iusb3, reg);

	intel_usb3_hw_bf_write(iusb3, &scu_cr_read, 1);
	intel_usb3_phy_hw_wait_ack(iusb3);
	intel_usb3_hw_bf_write(iusb3, &scu_cr_read, 0);

	return intel_usb3_hw_bf_read(iusb3, &scu_cr_data_out);
}

/**
 * @todo: comment
 * for future struct usb_phy_io_ops.write implementation
 */
static int intel_usb3_phy_io_write(struct intel_usb3 *iusb3, u32 val, u32 reg)
{
	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	intel_usb3_phy_hw_set_addr(iusb3, reg);

	intel_usb3_hw_bf_write(iusb3, &scu_cr_data_in, val);

	intel_usb3_hw_bf_write(iusb3, &scu_cr_cap_data, 1);
	intel_usb3_phy_hw_wait_ack(iusb3);
	intel_usb3_hw_bf_write(iusb3, &scu_cr_cap_data, 0);

	intel_usb3_hw_bf_write(iusb3, &scu_cr_write, 1);
	intel_usb3_phy_hw_wait_ack(iusb3);
	intel_usb3_hw_bf_write(iusb3, &scu_cr_write, 0);
	return 0;
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

	/* select LMU TRB registers solution or
	 * ensure LMU RAM read only for USB core */
	intel_usb3_hw_bf_write(iusb3, &scu_ssx, 1);
	/* disable OCT DvC interface */
	intel_usb3_hw_bf_write(iusb3, &scu_trace_mux, ~0);

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
static int intel_usb3_hw_powerup(struct intel_usb3 *iusb3, bool host)
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

	/* signal forces HS bias & PLL block to be
	 * powered down in suspend state */
	intel_usb3_hw_bf_write(iusb3, &scu_commononn, 1);

	if (host) {
		/* enable D- pull-down resistor */
		intel_usb3_hw_bf_write(iusb3, &scu_dmpulldown, 1);
		/* enabel D+ pull-up resistor */
		intel_usb3_hw_bf_write(iusb3, &scu_dppulldown, 1);
	} else {
		/* disable D- pull-down resistor */
		intel_usb3_hw_bf_write(iusb3, &scu_dmpulldown, 0);
	}


	intel_usb3_hw_reset(iusb3, false);
	udelay(100);

	/* configure phy for super speed support */
	intel_usb3_phy_io_write(iusb3, 0x40, INTEL_USB3_MPLL_LOOP_CTL);
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

	/* disable OCT DvC interface */
	intel_usb3_hw_bf_write(iusb3, &scu_trace_mux, ~0);

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
		intel_phy_err("invalid parameter");
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
		intel_phy_err("set pm state enable, %d", ret);
		return intel_phy_kernel_trap();
	}

	/* leave suspend state enabling 26Mhz phy reference clock */
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
/* MODULE INTERFACE - DWC3 EBC EXT					*/
/*----------------------------------------------------------------------*/
/**
 * see header file
 */
static bool intel_usb3_ebc_ext_needed(
	struct dwc3_ebc_ext *ext, const void *data)
{
	if (IS_ERR_OR_NULL(ext)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (data) {
		/* HACK enable EBC only for Debug class interface */
		/* either f_dvc_trace is aware of hw capabilites */
		/* or dwc3 must know interface being configured */
		const struct usb_interface_descriptor *d = data;

		if ((d->bDescriptorType == USB_DT_INTERFACE)
		&& (d->bInterfaceClass == USB_CLASS_DEBUG)
		&& (d->bInterfaceSubClass == USB_SUBCLASS_DVC_TRACE)) {
			intel_phy_warn("ebc mode on: %s",
					"undefined behavior if OCT DvC is off");
			return true;
		}
	}
	return false;
}

/**
 * see header file
 */
static void *intel_usb3_ebc_ext_trb_pool_alloc(
	struct dwc3_ebc_ext *ext, struct device *dev,
	size_t size, dma_addr_t *dma, gfp_t flags)
{
	struct intel_usb3 *iusb3 = NULL;

	if (IS_ERR_OR_NULL(ext) || IS_ERR_OR_NULL(dev) || !size
	|| (IS_ERR_OR_NULL(dma))) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return NULL;
	}
	iusb3 = container_of(ext, struct intel_usb3, ebc_ext);
	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("initialization issue");
		intel_phy_kernel_trap();
		return NULL;
	}
	*dma = iusb3->dma_map_lmu;
	return iusb3->iomem_lmu;
}

/**
 * see header file
 */
static void intel_usb3_ebc_ext_trb_pool_free(
	struct dwc3_ebc_ext *ext, struct device *dev,
	size_t size, void *cpu_addr, dma_addr_t dma)
{
	if (IS_ERR_OR_NULL(ext) || IS_ERR_OR_NULL(dev) || !size
	|| (IS_ERR_OR_NULL(cpu_addr))) {
		intel_phy_err("invalid parameter");
		intel_phy_kernel_trap();
		return;
	}
	/* nothing to do here */
}

/**
 * see header file
 */
static int intel_usb3_ebc_ext_xfer_run_stop(
	struct dwc3_ebc_ext *ext, unsigned ep, bool run)
{
	struct intel_usb3 *iusb3 = NULL;

	if (IS_ERR_OR_NULL(ext) || !ep) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	iusb3 = container_of(ext, struct intel_usb3, ebc_ext);
	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}
	if (run) {
		intel_phy_info("ebc mode for ep%din is active", ep);
		/* enable TRB write protection */
		iowrite32(0x33, iusb3->iomem_lmu + INTEL_USB3_LMU_TRB_CONFIG);
		/* enable OCT DvC interface */
		intel_usb3_hw_bf_write(iusb3, &scu_trace_mux, ep);
	} else {
		/* disable OCT DvC interface */
		intel_usb3_hw_bf_write(iusb3, &scu_trace_mux, ~0);
		/* disable TRB write protection */
		iowrite32(0x00, iusb3->iomem_lmu + INTEL_USB3_LMU_TRB_CONFIG);
		intel_phy_info("ebc mode for ep%din is idle", ep);
	}
	return 0;
}

/*----------------------------------------------------------------------*/
/* MODULE INTERFACE - OTG FSM						*/
/*----------------------------------------------------------------------*/
/**
 * host suspend/resume
 */
static int intel_usb3_otg_fsm_start_host(struct otg_fsm *fsm, int on)
{
	struct intel_phy *iphy = NULL;
	struct intel_usb3 *iusb3 = NULL;
	struct usb_bus *host = NULL;
	struct usb_hcd *hcd = NULL;
	int ret = 0;


	if (IS_ERR_OR_NULL(fsm)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	iphy = container_of(fsm, struct intel_phy, fsm);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}
	iusb3 = container_of(iphy, struct intel_usb3, iphy);
	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}
	host = intel_usb3_get_host(iusb3);
	if (IS_ERR_OR_NULL(host)) {
		intel_phy_err("host not bind");
		return intel_phy_kernel_trap();
	}
	hcd = bus_to_hcd(host);
	if (IS_ERR_OR_NULL(hcd) && IS_ERR_OR_NULL(hcd->self.controller)) {
		intel_phy_err("hcd not found");
		return intel_phy_kernel_trap();
	}

	intel_phy_info("host %s", on ? "on" : "off");
	if (on) {
		/* power up USB HW and prepare for host operations  */
		ret = intel_usb3_hw_powerup(iusb3, true);
		if (IS_ERR_VALUE(ret)) {
			intel_phy_err("failed to power up USB");
			return intel_phy_kernel_trap();
		}

		/* set host mode by sending event to controller driver */
		ret = atomic_notifier_call_chain(
			&iphy->phy.notifier, INTEL_USB_ID_SESSION, &on);
		if (NOTIFY_OK != ret) {
			intel_phy_err("failed to set host mode");
			goto power_down;
		}

		/* initialize USB host stack */
		ret = hcd->driver->start(hcd);
		if (IS_ERR_VALUE(ret)) {
			intel_phy_err("failed to initialize host");
			goto power_down;
		}
	} else {
		/* de-initialize host stack */
		usb_remove_hcd(hcd->shared_hcd);
		usb_put_hcd(hcd->shared_hcd);
		usb_remove_hcd(hcd);

		/* power-down USB HW */
		ret = intel_usb3_hw_powerdown(iusb3);
		if (IS_ERR_VALUE(ret)) {
			intel_phy_err("failed to powerdown USB");
			return intel_phy_kernel_trap();
		}
	}
	return 0;

power_down:
	intel_usb3_hw_powerdown(iusb3);
	return intel_phy_kernel_trap();

}

/**
 * device suspend/resume
 */
static int intel_usb3_otg_fsm_start_gadget(struct otg_fsm *fsm, int on)
{
	struct intel_phy *iphy = NULL;
	struct intel_usb3 *iusb3 = NULL;
	struct usb_gadget *gadget = NULL;
	int ret = 0;


	if (IS_ERR_OR_NULL(fsm)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	iphy = container_of(fsm, struct intel_phy, fsm);
	if (IS_ERR_OR_NULL(iphy)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}
	iusb3 = container_of(iphy, struct intel_usb3, iphy);
	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}
	gadget = intel_usb3_get_gadget(iusb3);
	if (IS_ERR_OR_NULL(gadget)) {
		intel_phy_err("gadget not bind");
		return intel_phy_kernel_trap();
	}

	intel_phy_info("gadget %s", on ? "on" : "off");
	if (on) {
		struct usb_gadget_ioctl_params ioctl = {
			.subcode = VENDOR_DWC3_EBC_EXT_ADD,
			.params = &iusb3->ebc_ext,
		};

		ret = usb_gadget_ioctl(gadget, IOCTL_CODE_DWC3, &ioctl);
		if (IS_ERR_VALUE(ret)
		&& (-EOPNOTSUPP != ret && -EALREADY != ret)) {
			intel_phy_err("ebc extension add failed, %d", ret);
			return intel_phy_kernel_trap();
		}

		/* do power up of USB core & PHY
		 * required for initializing USB stack */
		ret = intel_usb3_hw_powerup(iusb3, false);
		if (IS_ERR_VALUE(ret)) {
			intel_phy_err("failed to powerup USB");
			return intel_phy_kernel_trap();
		}
		ret = usb_gadget_vbus_connect(gadget);
		if (IS_ERR_VALUE(ret)) {
			intel_phy_err("failed to initilize USB gadget");
			intel_usb3_hw_powerdown(iusb3);
			return intel_phy_kernel_trap();
		}
	} else {
		ret = usb_gadget_vbus_disconnect(gadget);
		if (IS_ERR_VALUE(ret)) {
			intel_phy_err("failed to de-initialize USB gadget");
			return intel_phy_kernel_trap();
		}
		ret = intel_usb3_hw_powerdown(iusb3);
		if (IS_ERR_VALUE(ret)) {
			intel_phy_err("failed to powerdown USB");
			return intel_phy_kernel_trap();
		}
	}
	return ret;
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

	/* todo: implementation */
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

	/* todo: implementation */
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

	/* todo: implementation */
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

	/* todo: implementation */
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

	/* todo: implementation */
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
	/* EBC extension: 2 TRB, full HW control */
	iusb3->ebc_ext.needed = intel_usb3_ebc_ext_needed;
	iusb3->ebc_ext.trb_pool_size = 2;
	iusb3->ebc_ext.trb_pool_linked = true;
	iusb3->ebc_ext.trb_pool_alloc =	intel_usb3_ebc_ext_trb_pool_alloc;
	iusb3->ebc_ext.trb_pool_free = intel_usb3_ebc_ext_trb_pool_free;
	iusb3->ebc_ext.xfer_run_stop =	intel_usb3_ebc_ext_xfer_run_stop;

	/* map scu into io memory */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "scu_usb");
	if (IS_ERR_OR_NULL(res)) {
		intel_phy_err("platform resource iomem not found - scu");
		goto error_exit;
	}
	iusb3->iomem_scu = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(iusb3->iomem_scu)) {
		intel_phy_err("ioremap device resource - scu");
		goto error_exit;
	}

	/* map lmu into io memory */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lmu_usb");
	if (IS_ERR_OR_NULL(res)) {
		intel_phy_err("platform resource iomem not found - lmu");
		goto error_exit;
	}
	iusb3->iomem_lmu = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(iusb3->iomem_lmu)) {
		intel_phy_err("ioremap device resource - lmu");
		goto error_exit;
	}
	/* store lmu dma address */
	iusb3->dma_map_lmu = res->start;

	iusb3->reset_usb_core = reset_control_get(&pdev->dev, "usb_core");
	if (IS_ERR_OR_NULL(iusb3->reset_usb_core)) {
		intel_phy_err("reset control usb core not found");
		goto error_exit;
	}

	iusb3->reset_usb_susp = reset_control_get(&pdev->dev, "usb_susp");
	if (IS_ERR_OR_NULL(iusb3->reset_usb_susp)) {
		intel_phy_err("reset control usb susp not found");
		goto error_exit;
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"scu_usb_ss_trim",
			iusb3->scu_usb_ss_trim,
			ARRAY_SIZE(iusb3->scu_usb_ss_trim));
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("DTS TRIM values property read, %d", ret);
		goto error_exit;
	}
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	iusb3->reg_dig = regulator_get(&pdev->dev, "digital");
	if (IS_ERR(iusb3->reg_dig)) {
		intel_phy_err("cannot get digital regulator");
		goto error_exit;
	}
	regulator_enable(iusb3->reg_dig);

	iusb3->reg_phy = regulator_get(&pdev->dev, "phy");
	if (IS_ERR(iusb3->reg_phy)) {
		intel_phy_err("cannot get phy regulator");
		goto error_exit;
	}
	regulator_enable(iusb3->reg_phy);

	iusb3->reg_iso = regulator_get(&pdev->dev, "iso");
	if (IS_ERR(iusb3->reg_iso)) {
		intel_phy_err("cannot get iso regulator");
		goto error_exit;
	}
	regulator_enable(iusb3->reg_iso);

	iusb3->reg_core = regulator_get(&pdev->dev, "core");
	if (IS_ERR(iusb3->reg_core)) {
		intel_phy_err("cannot get core regulator");
		goto error_exit;
	}
	regulator_enable(iusb3->reg_core);

	iusb3->clk_kernel = of_clk_get_by_name(pdev->dev.of_node, "clk_kernel");
	if (IS_ERR(iusb3->clk_kernel)) {
		intel_phy_err("clk kernel not found");
		goto error_exit;
	}

	iusb3->clk_bus = of_clk_get_by_name(pdev->dev.of_node, "clk_bus");
	if (IS_ERR(iusb3->clk_bus)) {
		intel_phy_err("clk bus not found");
		goto error_exit;
	}

	clk_prepare_enable(iusb3->clk_bus);
	clk_prepare_enable(iusb3->clk_kernel);
#endif
	/* todo: OTG wakelock does not support USB3 so USB2 */
	ret = intel_phy_init(&pdev->dev, &iusb3->iphy,
				&intel_usb3_otg_fsm_ops, USB_PHY_TYPE_USB2);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("intel probe, %d", ret);
		goto error_clk_put;
	}

	ret = intel_usb3_debugfs_init(iusb3);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("debugfs initialization");
		goto error_phy_exit;
	}

	/* required only during boot time */
	intel_usb3_hw_init(iusb3);

	/* required for udc driver probing */
	ret = intel_usb3_hw_powerup(iusb3, false);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("failed to powerup USB");
		goto error_debugfs_exit;
	}

	/* allocate resume interrupt */
	iusb3->irq_usb_resume = platform_get_irq_byname(pdev, "usb_resume");
	if (IS_ERR_VALUE(iusb3->irq_usb_resume)) {
		intel_phy_err("platform resource irq usb resume not found");
		goto error_power_down;
	}
	ret = devm_request_irq(&pdev->dev, iusb3->irq_usb_resume,
				intel_usb3_isr_usb_resume, IRQF_SHARED,
				"usb_resume", iusb3);
	if (IS_ERR_VALUE(ret)) {
		intel_phy_err("request irq%d, %d", iusb3->irq_usb_resume, ret);
		goto error_power_down;
	}
#ifdef INTEL_USB3_FORCE_VBUS
	do {
		struct power_supply_cable_props cable_props = {
			.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT,
			.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_SDP,
			.ma = 0,
		};

		intel_phy_warn("fake cable attach for hardware bring up");
		intel_phy_notify(&iusb3->iphy, USB_EVENT_VBUS, &cable_props);

		intel_phy_warn("workaround pmic: connect usb data lines");
		/* vmm addr = SLAVE_DEV3 << 24 | USBPHYCTRL_REG */
		vmm_pmic_reg_write((0x5EUL << 24) | 0x08UL, 1);
		intel_phy_warn("pmic workaround applied!");
	} while (0);
#endif
	return 0;

error_power_down:
	intel_usb3_hw_powerdown(iusb3);
error_debugfs_exit:
	intel_usb3_debugfs_exit(iusb3);
error_phy_exit:
	intel_phy_exit(&pdev->dev, &iusb3->iphy);
error_clk_put:
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	clk_put(iusb3->clk_kernel);
	clk_put(iusb3->clk_bus);
#endif
error_exit:
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
	intel_usb3_debugfs_exit(iusb3);
	intel_phy_exit(&pdev->dev, &iusb3->iphy);
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	clk_put(iusb3->clk_kernel);
	clk_put(iusb3->clk_bus);
#endif
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

/*----------------------------------------------------------------------*/
/* LOCAL - DEBUGFS & TEST						*/
/*----------------------------------------------------------------------*/
static int intel_usb3_debugfs_open(struct inode *inode, struct file *file);
static ssize_t intel_usb3_debugfs_dbg_read(
	struct file *file, char __user *ubuf, size_t count, loff_t *ppos);
static ssize_t intel_usb3_debugfs_dbg_write(
	struct file *file, const char __user *ubuf, size_t count, loff_t *ppos);
static ssize_t intel_usb3_debugfs_reg_write(
	struct file *file, const char __user *ubuf, size_t count, loff_t *ppos);

/**
 * @todo: comment
 */
static int intel_usb3_test_zero(struct intel_usb3 *iusb3,
	unsigned dummy1, unsigned dummy2, unsigned dummy3)
{
	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	/* disable crash during this test */
	intel_phy_kernel_trap_enable(false);

	/* local - utilities */
	intel_usb3_get_device(NULL);
	intel_usb3_get_gadget(NULL);
	intel_usb3_get_speed(NULL);
	/* local - hw utilities */
	intel_usb3_hw_bf_read(NULL, NULL);
	intel_usb3_hw_bf_write(NULL, NULL, 0);
	intel_usb3_hw_req_pwr_state(NULL, false);
	intel_usb3_hw_reset(NULL, false);
	intel_usb3_pm_set_state(NULL, NULL);
	/* local - phy utilities */
	intel_usb3_phy_hw_wait_ack(NULL);
	intel_usb3_phy_hw_set_addr(NULL, 0);
	intel_usb3_phy_io_read(NULL, 0);
	intel_usb3_phy_io_write(NULL, 0, 0);
	/* local - hw configuration */
	intel_usb3_hw_init(NULL);
	intel_usb3_hw_powerup(NULL, true);
	intel_usb3_hw_powerdown(NULL);
	intel_usb3_hw_suspend(NULL);
	intel_usb3_hw_wakeup(NULL);

	/* ebc extension */
	intel_usb3_ebc_ext_needed(NULL, NULL);
	intel_usb3_ebc_ext_trb_pool_alloc(NULL, NULL, 0, NULL, 0);
	intel_usb3_ebc_ext_trb_pool_free(NULL, NULL, 0, NULL, 0);
	intel_usb3_ebc_ext_xfer_run_stop(NULL, 0, false);

	/* interface */
	intel_usb3_otg_fsm_start_host(NULL, 0);
	intel_usb3_otg_fsm_start_gadget(NULL, 0);
#ifdef CONFIG_PM_RUNTIME
	intel_usb3_runtime_suspend(NULL);
	intel_usb3_runtime_resume(NULL);
	intel_usb3_runtime_idle(NULL);
#endif
#ifdef CONFIG_PM_SLEEP
	intel_usb3_system_sleep_suspend(NULL);
	intel_usb3_system_sleep_resume(NULL);
#endif
	intel_usb3_isr_usb_resume(0, NULL);
	intel_usb3_probe(NULL);
	intel_usb3_remove(NULL);

	/* debugfs and test */
	intel_usb3_test_zero(NULL, 0, 0, 0);
	intel_usb3_debugfs_open(NULL, NULL);
	intel_usb3_debugfs_dbg_read(NULL, NULL, 0, NULL);
	intel_usb3_debugfs_dbg_write(NULL, NULL, 0, NULL);
	intel_usb3_debugfs_reg_write(NULL, NULL, 0, NULL);
	intel_usb3_debugfs_init(NULL);
	intel_usb3_debugfs_exit(NULL);

	/* re-enable crash after this test */
	intel_phy_kernel_trap_enable(true);

	return 0;
}

/**
 * @todo: comment
 */
static int intel_usb3_debugfs_open(struct inode *inode, struct file *file)
{
	if (IS_ERR_OR_NULL(inode) || IS_ERR_OR_NULL(file)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (IS_ERR_OR_NULL(inode->i_private)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	file->private_data = inode->i_private;
	return 0;
}

/**
 * @todo: comment
 */
static ssize_t intel_usb3_debugfs_dbg_read(
	struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	struct intel_usb3 *iusb3 = NULL;
	unsigned i = 0;


	if (IS_ERR_OR_NULL(file) || IS_ERR_OR_NULL(ubuf)
	|| (IS_ERR_OR_NULL(ppos))) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (IS_ERR_OR_NULL(file->private_data)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	iusb3 = file->private_data;

	/* todo: log in provided buffer instead */

	intel_phy_info("PHY state: 0x%04X", cpu_to_le32(iusb3->state));

	for (i = 0; i < ARRAY_SIZE(scu_usb_ss_trim); i++)
		intel_phy_info("USB_SS_TRIM%u  = 0x%08X", (i + 1),
			intel_usb3_hw_bf_read(iusb3, &scu_usb_ss_trim[i]));

	if (iusb3->state) /* access only when powered */
		intel_phy_info("MPLL_LOOP_CTL = 0x%08X",
			intel_usb3_phy_io_read(iusb3,
				INTEL_USB3_MPLL_LOOP_CTL));

	intel_phy_info("LMU_TRB_CONFIG = 0x%08X", (u32)iusb3->dma_map_lmu);
	for (i = 0; i <= INTEL_USB3_LMU_TRB_CONFIG; i += 0x4)
		intel_phy_info("%02X: %08X", i, ioread32(iusb3->iomem_lmu + i));

	return 0;
}

/**
 * @todo: comment
 */
static ssize_t intel_usb3_debugfs_dbg_write(
	struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct intel_usb3 *iusb3 = NULL;
	char buf[16] = {0};
	unsigned test, loop, t1, t2, end = 0;


	if (IS_ERR_OR_NULL(file) || IS_ERR_OR_NULL(ubuf)
	|| (IS_ERR_OR_NULL(ppos))) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (IS_ERR_OR_NULL(file->private_data)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	iusb3 = file->private_data;

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		intel_phy_err("copy from user");
		return intel_phy_kernel_trap();
	}

	if (sscanf(buf, "%u %u %u %u %u", &test, &loop, &t1, &t2, &end) != 4) {
		intel_phy_err("wrong parameter number!");
		goto info;
	}
	/* fix KW warnings */
	if (test > USHRT_MAX)
		test = USHRT_MAX;
	if (loop > USHRT_MAX)
		loop = USHRT_MAX;
	if (t1 > USHRT_MAX)
		t1 = USHRT_MAX;
	if (t2 > USHRT_MAX)
		t2 = USHRT_MAX;

	switch (test) {
	case 0:
		intel_usb3_test_zero(iusb3, loop, t1, t2);
		break;
	default:
		intel_phy_err("test not defined!");
		break;
	}

	return count;
info:
	intel_phy_info("<test> <loop> <t1> <t2>");
	intel_phy_info("test 0: test invalid parameter, boost coverage");
	return count;
}

/**
 * @todo: comment
 */
static const struct file_operations intel_usb3_debugfs_dbg_fops = {
	.open = intel_usb3_debugfs_open,
	.read = intel_usb3_debugfs_dbg_read,
	.write = intel_usb3_debugfs_dbg_write,
};

/**
 * @todo: comment
 */
static ssize_t intel_usb3_debugfs_reg_write(
	struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct intel_usb3 *iusb3 = NULL;
	char buf[16] = {0};
	unsigned int cnt, reg, val, end = 0;


	if (IS_ERR_OR_NULL(file) || IS_ERR_OR_NULL(ubuf)
	|| (IS_ERR_OR_NULL(ppos))) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	if (IS_ERR_OR_NULL(file->private_data)) {
		intel_phy_err("initialization issue");
		return intel_phy_kernel_trap();
	}

	iusb3 = file->private_data;

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		intel_phy_err("copy from user");
		return intel_phy_kernel_trap();
	}

	cnt = sscanf(buf, "%i %i %i", &reg, &val, &end);
	if (1 != cnt && 2 != cnt) {
		intel_phy_err("wrong parameter number!");
		goto info;
	}
	/* fix KW warnings */
	if (reg > UINT_MAX)
		reg = UINT_MAX;
	if (val > UINT_MAX)
		val = UINT_MAX;

	if (1 == cnt)
		intel_phy_info("PHY[0x%02X] = 0x%08X", reg,
				intel_usb3_phy_io_read(iusb3, reg));
	 else
		intel_usb3_phy_io_write(iusb3, val, reg);

	return count;
info:
	intel_phy_info("<reg>       - read  phy register");
	intel_phy_info("<reg> <val> - write phy register");
	return count;
}

/**
 * @todo: comment
 */
static const struct file_operations intel_usb3_debugfs_reg_fops = {
	.open = intel_usb3_debugfs_open,
	.write = intel_usb3_debugfs_reg_write,
};

/**
 * @todo: comment
 */
static int intel_usb3_debugfs_init(struct intel_usb3 *iusb3)
{
	struct dentry *root = NULL;
	struct dentry *file = NULL;


	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}

	root = debugfs_create_dir("intel_usb3", NULL);
	if (IS_ERR_OR_NULL(root)) {
		intel_phy_err("debugfs create dir");
		return intel_phy_kernel_trap();
	}

	file = debugfs_create_file("dbg", S_IRUGO | S_IWUSR,
		root, iusb3, &intel_usb3_debugfs_dbg_fops);
	if (IS_ERR_OR_NULL(file)) {
		intel_phy_err("debugfs create file");
		debugfs_remove(root);
		return intel_phy_kernel_trap();
	}

	file = debugfs_create_file("reg", S_IWUSR,
		root, iusb3, &intel_usb3_debugfs_reg_fops);
	if (IS_ERR_OR_NULL(file)) {
		intel_phy_err("debugfs create file");
		debugfs_remove_recursive(root);
		return intel_phy_kernel_trap();
	}

	iusb3->debugfs_root = root;
	return 0;
}

/**
 * @todo: comment
 */
static int intel_usb3_debugfs_exit(struct intel_usb3 *iusb3)
{
	if (IS_ERR_OR_NULL(iusb3)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	if (IS_ERR_OR_NULL(iusb3->debugfs_root)) {
		intel_phy_err("invalid parameter");
		return intel_phy_kernel_trap();
	}
	debugfs_remove_recursive(iusb3->debugfs_root);
	iusb3->debugfs_root = NULL;
	return 0;
}
