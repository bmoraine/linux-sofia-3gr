/*
 *******************************************************************************
 *
 *  Component: XGold MIPI DSI driver
 *
 *  Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *******************************************************************************
 */

#ifndef __DSI_HWREGS_H__
#define __DSI_HWREGS_H__

#include "dsi_display.h"

/* register addresses external */
#define DSI_CLC               0x0
#define DSI_CLC_STAT          0x8
#define DSI_ID                0xC
#define DSI_FIFO_ID           0x10
#define DSI_SRB_MSCONF_ID     0x14
#define DSI_SWCID             0x1C
#define DSI_FIFO_CFG          0x30
#define DSI_FIFO_CTRL         0x34
#define DSI_MRPS_CTRL         0x38
#define DSI_RPS_STAT          0x3C
#define DSI_TPS_CTRL          0x40
#define DSI_FIFO_STAT         0x44
#define DSI_RIS               0x80
#define DSI_IMSC              0x84
#define DSI_MIS               0x88
#define DSI_ISR               0x90
#define DSI_DMAE              0x94
#define DSI_ICR               0x98
#define DSI_CFG               0x100
#define DSI_CLK               0x104
#define DSI_HEAD              0x108
#define DSI_TO0               0x10C
#define DSI_TO1               0x110
#define DSI_VID0              0x114
#define DSI_VID1              0x118
#define DSI_VID2              0x11C
#define DSI_VID3              0x120
#define DSI_VID4              0x124
#define DSI_VID5              0x128
#define DSI_VID6              0x12C
#define DSI_PHY0              0x130
#define DSI_PHY1              0x134
#define DSI_PHY2              0x138
#define DSI_PHY3              0x13C
#define DSI_STAT              0x140
#define DSI_TXD               0x4000
#define DSI_RXD               0x10000

/* Interrupt source bit masks */
#define DSI_IRQ_RXLSREQ         (1<<0)
#define DSI_IRQ_RXSREQ          (1<<1)
#define DSI_IRQ_RXLBREQ         (1<<2)
#define DSI_IRQ_RXBREQ          (1<<3)
#define DSI_IRQ_TXLSREQ         (1<<4)
#define DSI_IRQ_TXSREQ          (1<<5)
#define DSI_IRQ_TXLBREQ         (1<<6)
#define DSI_IRQ_TXBREQ          (1<<7)
#define DSI_IRQ_ERR_DSITR0      (1<<8)
#define DSI_IRQ_ERR_DSITR1      (1<<9)
#define DSI_IRQ_ERR_DSITR2      (1<<10)
#define DSI_IRQ_ERR_DSITR3      (1<<11)
#define DSI_IRQ_ERR_DSIUFL      (1<<12)
#define DSI_IRQ_ERR_DSIFIN      (1<<13)
#define DSI_IRQ_ERR_DSILTO      (1<<14)
#define DSI_IRQ_ERR_DSIHTO      (1<<15)
#define DSI_IRQ_ERR_DSIRTO      (1<<16)
#define DSI_IRQ_ERR_DSIESC      (1<<17)
#define DSI_IRQ_ERR_DSISYN      (1<<18)
#define DSI_IRQ_ERR_DSICTR      (1<<19)
#define DSI_IRQ_ERR_DSICON      (1<<20)
#define DSI_IRQ_ERR_DSIOFL      (1<<21)
#define DSI_IRQ_ERR_IDLE        (1<<22)

#define DSI_IRQ_ERR_MASK (\
		DSI_IRQ_RXLSREQ |\
		DSI_IRQ_RXSREQ |\
		DSI_IRQ_RXLBREQ |\
		DSI_IRQ_RXBREQ |\
		DSI_IRQ_TXLSREQ |\
		DSI_IRQ_TXSREQ |\
		DSI_IRQ_TXLBREQ |\
		DSI_IRQ_TXBREQ |\
		DSI_IRQ_ERR_DSITR0 |\
		DSI_IRQ_ERR_DSITR1 |\
		DSI_IRQ_ERR_DSITR2 |\
		DSI_IRQ_ERR_DSITR3 |\
		DSI_IRQ_ERR_DSIUFL |\
		DSI_IRQ_ERR_DSIFIN |\
		DSI_IRQ_ERR_DSILTO |\
		DSI_IRQ_ERR_DSIHTO |\
		DSI_IRQ_ERR_DSIRTO |\
		DSI_IRQ_ERR_DSIESC |\
		DSI_IRQ_ERR_DSISYN |\
		DSI_IRQ_ERR_DSICTR |\
		DSI_IRQ_ERR_DSICON |\
		DSI_IRQ_ERR_DSIOFL |\
		DSI_IRQ_ERR_IDLE)


#define DSI_MODE_CONF   2
#define DSI_MODE_RUN    1

#define DSI_DIR_TX 0
#define DSI_DIR_RX 1

/* Reg description table indices */
enum {
	EXR_DSI_CLC = 1,
	EXR_DSI_CLC_RUN,
	EXR_DSI_CLC_STAT,
	EXR_DSI_CLC_STAT_RUN,
	EXR_DSI_CLC_STAT_MODEN,
	EXR_DSI_CLC_STAT_KID,
	EXR_DSI_ID,
	EXR_DSI_FIFO_ID,
	EXR_DSI_SRB_MSCONF_ID,
	EXR_DSI_SWCID,
	EXR_DSI_FIFO_CFG,
	EXR_DSI_FIFO_CTRL,
	EXR_DSI_MRPS_CTRL,
	EXR_DSI_RPS_STAT,
	EXR_DSI_TPS_CTRL,
	EXR_DSI_TPS_CTRL_TPS,
	EXR_DSI_FIFO_STAT,
	EXR_DSI_FIFO_STAT_RXFFS,
	EXR_DSI_RIS,
	EXR_DSI_IMSC,
	EXR_DSI_MIS,
	EXR_DSI_ISR,
	EXR_DSI_DMAE,
	EXR_DSI_ICR,
	EXR_DSI_CFG,
	EXR_DSI_CFG_CFG_LAT,
	EXR_DSI_CFG_HEAD_LAT,
	EXR_DSI_CFG_GATE,
	EXR_DSI_CFG_TX,
	EXR_DSI_CFG_LP,
	EXR_DSI_CFG_MODE,
	EXR_DSI_CFG_EOT,
	EXR_DSI_CFG_TURN,
	EXR_DSI_CFG_VALID,
	EXR_DSI_CFG_DATA,
	EXR_DSI_CFG_STP,
	EXR_DSI_CFG_ULPS,
	EXR_DSI_CFG_EN,
	EXR_DSI_CFG_LANES,
	EXR_DSI_CFG_ID,
	EXR_DSI_CFG_TXS,
	EXR_DSI_CFG_TE,
	EXR_DSI_CFG_FIN,
	EXR_DSI_CFG_VSYNC,
	EXR_DSI_CFG_PSYNC,
	EXR_DSI_CFG_SOURCE,
	EXR_DSI_CLK,
	EXR_DSI_HEAD,
	EXR_DSI_HEAD_HEADER,
	EXR_DSI_HEAD_WCNT,
	EXR_DSI_HEAD_CMD,
	EXR_DSI_TO0,
	EXR_DSI_TO1,
	EXR_DSI_VID0,
	EXR_DSI_VID0_HFP_BYTES,
	EXR_DSI_VID0_HBP_BYTES,
	EXR_DSI_VID0_HSA_BYTES,
	EXR_DSI_VID0_HFP,
	EXR_DSI_VID0_HBP,
	EXR_DSI_VID0_HSA,
	EXR_DSI_VID0_HFP_LP,
	EXR_DSI_VID0_HBP_LP,
	EXR_DSI_VID0_HSA_LP,
	EXR_DSI_VID1,
	EXR_DSI_VID1_VACT_LINES,
	EXR_DSI_VID1_MODE,
	EXR_DSI_VID1_ID,
	EXR_DSI_VID1_PIXEL,
	EXR_DSI_VID1_FILL_BUFFER_TO,
	EXR_DSI_VID1_LAST_CS,
	EXR_DSI_VID2,
	EXR_DSI_VID2_VFP,
	EXR_DSI_VID2_VBP,
	EXR_DSI_VID2_VSA,
	EXR_DSI_VID3,
	EXR_DSI_VID3_PIXEL_PACKETS,
	EXR_DSI_VID3_PIXEL_BYTES,
	EXR_DSI_VID4,
	EXR_DSI_VID4_BLANK_PACKETS,
	EXR_DSI_VID4_BLANK_BYTES,
	EXR_DSI_VID5,
	EXR_DSI_VID5_LINE_TIME,
	EXR_DSI_VID5_BLLP_TIME,
	EXR_DSI_VID6,
	EXR_DSI_VID6_LAST_BLANK,
	EXR_DSI_VID6_LAST_PIXEL,
	EXR_DSI_PHY0,
	EXR_DSI_PHY0_SHARE,
	EXR_DSI_PHY0_M,
	EXR_DSI_PHY0_N,
	EXR_DSI_PHY0_POWERUP,
	EXR_DSI_PHY0_CALIB,
	EXR_DSI_PHY0_TO_LP_HS_REQ,
	EXR_DSI_PHY1,
	EXR_DSI_PHY1_TO_LP_HS_DIS,
	EXR_DSI_PHY1_TO_LP_EOT,
	EXR_DSI_PHY1_TO_HS_ZERO,
	EXR_DSI_PHY1_TO_HS_FLIP,
	EXR_DSI_PHY1_LP_CLK_DIV,
	EXR_DSI_PHY2,
	EXR_DSI_PHY2_HS_CLK_PRE,
	EXR_DSI_PHY2_HS_CLK_POST,
	EXR_DSI_PHY2_DAT_DELAY,
	EXR_DSI_PHY2_CLK_DELAY,
	EXR_DSI_PHY2_LPTX_TFALL,
	EXR_DSI_PHY3,
	EXR_DSI_PHY3_EN,
	EXR_DSI_PHY3_LPTX_TRISE,
	EXR_DSI_PHY3_LPTX_VREF,
	EXR_DSI_STAT,
	EXR_DSI_STAT_DSI_BSY,
	EXR_DSI_STAT_DSI_FULL,
	EXR_DSI_STAT_DSI_DIR,
	EXR_DSI_STAT_DSI_LOCK,
	EXR_DSI_TXD,
	EXR_DSI_RXD,
	NBREG_MAX,
};

struct dsi_command {
	char *name;             /* command name string */
	unsigned int addr;      /* command register address */
	unsigned int mask;      /* value field mask */
	int shift;              /* value field shift */
};

/* Declaration for Reg description table */
extern struct dsi_command dsi_regs[];
#define BITFLDS(_id_, _val_) \
	(((_val_) & dsi_regs[_id_].mask) << (dsi_regs[_id_].shift))

int dsi_wait_status(struct xgold_mipi_dsi_device *mipi_dsi, unsigned int reg,
		     unsigned int value, unsigned int mask, unsigned int delay,
		     unsigned int count);
unsigned int dsi_read_field(struct xgold_mipi_dsi_device *mipi_dsi,
			    unsigned int id);
void dsi_write_field(struct xgold_mipi_dsi_device *mipi_dsi,
		     unsigned int id, u32 val);
#endif
