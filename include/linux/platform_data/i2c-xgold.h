/*
* Copyright (C) 2012-2013 Intel Mobile Communications GmbH
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

#ifndef __I2C_XGOLD_PLATFORM_DATA_H
#define __I2C_XGOLD_PLATFORM_DATA_H

struct xgold_i2c_platdata {
	int irq;
	int irq_num;
	struct clk *clk_kernel;
	struct clk *clk_bus;
	const char *dma_txrx;
	unsigned cfg_baud[2][4]; /* INC, DEC, TIM_SDA, TIM_HS_SDA, for both
				    100kbps and 400kbps */
	struct reset_control *rst;
	struct regulator *regulator;
	struct regulator *regulator2;
	char id;
	struct device_pm_platdata *pm_platdata;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	int sda_pin;
	int scl_pin;
	u32 rxbs;
	u32 txbs;
	unsigned long flags;
};

#define FDIV_INC	0
#define FDIV_DEC	1
#define TIM_SDA		2
#define TIM_HS_SDA	3

#endif /* __I2C_XGOLD_PLATFORM_DATA_H */
