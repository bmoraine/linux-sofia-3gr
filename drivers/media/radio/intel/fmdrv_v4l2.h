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

#ifndef _V4L2_INTERFACES_H
#define _V4L2_INTERFACES_H

/*
 * This  file contains the user defined types of V4L2 interfaces.
 */

#include <linux/videodev2.h>
#include <linux/ioctl.h>

#include <fmdrv_xgold.h>
#include <fmdrv_xgold_sys.h>
#include <aud_app_fmr_hld_api.h>
#include <aud_app_fmr_hld_api_rx.h>
#include <aud_app_fmr_hld_rx.h>

/* Manufacturer ID String */
#define MANUFACTURER_ID_STRING	16

/* Hardware ID String */
#define HW_ID_STRING		16

/* Hardware Version String */
#define HW_VER_STRING		24

/* Software Version String */
#define SW_VER_STRING		100

/* Supported antenna types  */
enum fmr_antenna_type {
	FMR_ANT_INTERNAL, /* Embedded Diffrentail */
	FMR_ANT_EXTERNAL, /* HS single ended */
	FMRADIO_ANT_INVALID
};

/*
 * Structure for displaying the version information of FM Radio RX
 */
struct fmr_version {
	char man_id[MANUFACTURER_ID_STRING];	/* Manufacturer ID string */
	char hw_id[HW_ID_STRING];	/* HW ID string */
	char hw_ver[HW_VER_STRING];	/* HW version string */
	char sw_ver[SW_VER_STRING];	/* SW and FW version string */
};

/* Structure for the capability report */
struct fmr_feat_cap {
	unsigned short fmr_rx_present; /* FMR Rx support */
	unsigned short fmr_tx_present; /* FMR Tx support */
	unsigned short fmr_int_ant_present; /* FMR Internal antenna suppot */
	unsigned short fmr_ext_ant_support; /* FMR External antenna support */
	unsigned short fmr_lna_present;/* FMR LNA support */
};

/* Structure for configuring HW params */
struct fmr_hw_params {
	signed short pn_thr;/* Phase noise threshold for channel validation */
	signed short rssi_offset_int_ant; /* Overall offset for emb ant */
	signed short rssi_offset_ext_ant; /* Overall offset for ext ant */
	unsigned short int_lna_gain_reduction;/* Internal LNA gain reduction */
	unsigned short vol_ramping_cfg; /* Volume ramping configuration */
	u32 clk_switch_range_104; /* Clk switching range near 104Mhz*/
	unsigned short rx_if_selection; /* IF selection 275 or 575khz */
};

/* External LNA cfg */
struct fmr_ext_lna_params {
	enum fmr_antenna_type ant_type;
	struct fmrx_ext_lna_cfg lna_cfg;
};

/* sructure to get the data from the register address */
struct fmtrx_register_data {
	u32 reg_addr;	/* 32-Bit Address of the register */
	u32 *reg_data;	/* 32-Bit value */
};

/*
 * @brief
 * Private IOCTLs for functionalities that are not implemented by standard V4L2
 */
enum v4l2_cid_private_xgold_t {
	/* Set FM band limits/De-emp */
	V4L2_CID_PRIV_XGOLD_FMR_SET_BAND = (V4L2_CID_PRIVATE_BASE + 1),
	V4L2_CID_PRIV_XGOLD_FMR_SET_OUTPUT_MODE,/* Set FM output mode */
	V4L2_CID_PRIV_XGOLD_FMR_SET_ANTENNA,	/* Set antenna type */
	V4L2_CID_PRIV_XGOLD_FMR_SET_SB,		/* Set sideband injection */
	V4L2_CID_PRIV_XGOLD_FMR_SET_SNC, /* Set stereo noise cancellation */
	V4L2_CID_PRIV_XGOLD_FMR_SET_SM,		/* Set soft mute */
	V4L2_CID_PRIV_XGOLD_FMR_SET_GAIN_CTRL,	/* Set AGC gain control */
	V4L2_CID_PRIV_XGOLD_FMR_SET_RSSI, /* RSSI subscn with low/high thres */
	V4L2_CID_PRIV_XGOLD_FMR_SET_HWPARAMS,	/* Set other hardware params */
	V4L2_CID_PRIV_XGOLD_FMR_GET_VER_INFO,	/* Get sw/fw version */
	V4L2_CID_PRIV_XGOLD_FMR_GET_RDS_DATA,	/* Get RDS ch data */
	V4L2_CID_PRIV_XGOLD_FMR_GET_DYN_CFG, /* Reciever State - Dync data */
	V4L2_CID_PRIV_XGOLD_FMR_GET_STA_CFG,	/* Static cfg of reciever */
	V4L2_CID_PRIV_XGOLD_FMR_REG_READ,	/* Read from a FW register */
	V4L2_CID_PRIV_XGOLD_FMR_REG_WRITE,	/* Write to a FW register */
	V4L2_CID_PRIV_XGOLD_FMR_NVM_COMMIT, /* Commit to NVM data structure */
	V4L2_CID_PRIV_XGOLD_FMR_SET_LNA_CFG,	/* Set the ext LNA Config */
	V4L2_CID_PRIV_XGOLD_FMR_GET_LNA_CFG,	/* Get the ext LNA Config */
	V4L2_CID_PRIV_XGOLD_FMR_GET_OUTPUT_MODE,/* get the aud output mode */
	V4L2_CID_PRIV_XGOLD_FMRX_SET_OFFSETS,
	V4L2_CID_PRIV_XGOLD_FMRX_GET_OFFSETS,
	V4L2_CID_PRIV_XGOLD_FMRX_SET_RSSI_OFFSETS,
	V4L2_CID_PRIV_XGOLD_FMRX_GET_RSSI_OFFSETS,
	V4L2_CID_PRIV_XGOLD_FMR_MAX
};

#endif /* _V4L2_INTERFACES_H */

