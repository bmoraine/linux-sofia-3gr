struct xgold_mmc_pdata {
	const char *kernel_clk_name;
	const char *bus_clk_name;
	const char *master_clk_name;
	struct clk *bus_clk;
	struct clk *master_clk;
	unsigned int max_clock;
	unsigned int min_clock;
	unsigned int tap_values[6];
	int irq_wk;
	void __iomem *tap_reg;
	unsigned int tap_values2[6];
	void __iomem *tap_reg2;
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
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device dev;
	struct device_pm_platdata *pm_platdata_ctrl;
	struct device_pm_platdata *pm_platdata_clock_ctrl;
	struct regulator *regulator;
#endif
};


