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

/*-----------------------------------------------------------------------------
				Enumerations
  ---------------------------------------------------------------------------*/

/* Return result of interface functions */
enum imc_idi_imc_idi_btauif_error_codes {
	imc_idi_btauif_result_ok		=  0,
	imc_idi_btauif_result_error		= -1,
	imc_idi_btauif_result_invalid_state	= -2,
	imc_idi_btauif_result_internal_error	= -3,
	imc_idi_btauif_result_invalid_param	= -4,
	imc_idi_btauif_result_not_supported	= -5,
};

/* Input parameter for setting clock rate */
enum imc_idi_btauif_clock_rate {
	imc_idi_btauif_clk_48k_def  = 0,
	imc_idi_btauif_clk_16k      = 1,
	imc_idi_btauif_clk_8k       = 2,
	imc_idi_btauif_clk_48k      = 3
};


/* Input parameter to enable/disable mute feature */
enum imc_idi_btauif_mute_enable {
	imc_idi_btauif_mute_disable = 0,
	imc_idi_btauif_mute_enable  = 1
};


/*-----------------------------------------------------------------------------
				Data Structures
  ---------------------------------------------------------------------------*/

/* User configuration for BT audio interface */
struct imc_idi_btauif_config {
	unsigned char clk_rate;
	bool mute_enable;
	unsigned short mute_sample;
	unsigned char mute_threshold;
};

/* BT audio interface operations */
struct imc_idi_btauif_ops {
	int (*enable)(unsigned char clk_rate, void __iomem *ctrl_io);
	int (*enable_ex)(struct imc_idi_btauif_config *config_params,
							void __iomem *ctrl_io);
	int (*disable)(void __iomem *ctrl_io);
};

/*-----------------------------------------------------------------------------
				Interface functions
  ---------------------------------------------------------------------------*/

/* Register BTAUIF configuration interface with DAI driver */
extern void imc_idi_bt_sco_register(struct imc_idi_btauif_ops *ops);

/* Unregister BTAUIF configuration interface from DAI driver */
extern void imc_idi_bt_sco_unregister(struct imc_idi_btauif_ops *ops);
