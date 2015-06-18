/*
 * linux/drivers/mmc/host/sdhci-of-imc.h - open firmware compliant sdhci for xgold
 * Copyright (c) 2015 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __SDHCI_OF_IMC_H
#define __SDHCI_OF_IMC_H

/*
 * Same interface is available for emmc, sd and sdio controllers.
 */
typedef enum {
	xgold_mmc_tap_ctrl_idx = 0,
	xgold_mmc_core_cfg1_idx = 1,
	xgold_mmc_core_cfg2_idx = 2,
	xgold_mmc_core_cfg3_idx = 3,
	xgold_mmc_core_cfg4_idx = 4,
	xgold_mmc_core_cfg5_idx = 5,
	xgold_mmc_core_cfg6_idx = 6,
	xgold_mmc_core_cfg7_idx = 7,
	xgold_mmc_pad_drv_idx = 8,
	xgold_mmc_total_regs
} xgold_mmc_regs_idx_t;

typedef enum {
	xgold_mmc_tap_ctrl_offset = 0 * sizeof(u32),
	xgold_mmc_core_cfg1_offset = 1 * sizeof(u32),
	xgold_mmc_core_cfg2_offset = 2 * sizeof(u32),
	xgold_mmc_core_cfg3_offset = 3 * sizeof(u32),
	xgold_mmc_core_cfg4_offset = 4 * sizeof(u32),
	xgold_mmc_core_cfg5_offset = 5 * sizeof(u32),
	xgold_mmc_core_cfg6_offset = 6 * sizeof(u32),
	xgold_mmc_core_cfg7_offset = 7 * sizeof(u32),
	xgold_mmc_pad_drv_offset = 8 * sizeof(u32),
} xgold_mmc_regs_offset_t;

#define MAX_MODES 10
struct xgold_mmc_pdata {
	const char *kernel_clk_name;
	const char *bus_clk_name;
	const char *master_clk_name;
	struct clk *bus_clk;
	struct clk *master_clk;
	unsigned int max_clock;
	unsigned int min_clock;
	unsigned int tap_values[MAX_MODES];
	int irq_wk;
	int irq_eint;
	void __iomem *scu_base;
	phys_addr_t scu_base_phys;
	int tap_reg_offset;
	unsigned int tap_values2[MAX_MODES];
	int tap_reg2_offset;
	unsigned int card_drive_strength[4];
	struct xgold_mmc_callbacks *mmc_cb;
	unsigned char id;
	const char *bus_regulator_name;
	const char *regulator_name;
	const char *ext_regulator_name;
	struct xg_gpio *mux_cfg;
	unsigned mux_nb;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	u8 pins_restore_default;
	bool io_master;
	struct device dev;
	u32 fixup;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata_ctrl;
	struct device_pm_platdata *pm_platdata_clock_ctrl;
	struct regulator *regulator;
#endif
	bool rpm_enabled;
	s32 irq_is_disable;
	spinlock_t irq_lock;
	int cd_gpio;
};

/* IO access depends on the platform - it may goes through an hypercall */
#define SCU_IO_ACCESS_BY_VMM 0
#define SCU_IO_ACCESS_BY_LNX 1


static inline void xgold_sdhci_scu_write(struct xgold_mmc_pdata *pdata,
		int offset, int value)
{
#ifdef CONFIG_X86_INTEL_SOFIA
	if (pdata->io_master
			== SCU_IO_ACCESS_BY_VMM) {
		phys_addr_t write_addr = pdata->scu_base_phys + offset;
		if (mv_svc_reg_write(write_addr,
					value, -1))
			dev_err(&pdata->dev,
					"mv_svc_reg_write failed @%pa\n",
					&write_addr);
	} else
#endif
	{
		void __iomem *write_addr = pdata->scu_base + offset;
		writel(value, write_addr);
	}
}

static inline u32 xgold_sdhci_scu_read(struct xgold_mmc_pdata *pdata,
		int offset)
{
#ifdef CONFIG_X86_INTEL_SOFIA
	u32 value;
	if (pdata->io_master
			== SCU_IO_ACCESS_BY_VMM) {
		phys_addr_t read_addr = pdata->scu_base_phys + offset;
		if (mv_svc_reg_read(read_addr,
					&value, -1))
			dev_err(&pdata->dev,
					"mv_svc_reg_read failed @%pa\n",
					&read_addr);
		return value;
	} else
#endif
	{
		void __iomem *read_addr = pdata->scu_base + offset;
		return readl(read_addr);
	}
}

/*
 * XGOLD_DEFAULT_REGS_FIXUP
 * Default values of corecfg registers are not correct to support
 * fully campliant sdio v3.0. This fixup shall be called in order
 * to restore these registers each time the controller is power
 * gated.
 */
#define XGOLD_DEFAULT_REGS_FIXUP	0x00000001

#endif /* __SDHCI_OF_IMC_H */
