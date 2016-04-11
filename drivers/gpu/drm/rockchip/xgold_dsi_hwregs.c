/*
 * Component: XGold MIPI DSI driver
 *
 * Copyright (C) 2014, Intel Mobile Communications GmbH.
 * Copyright (C) 2016, Fuzhou Rockchip Electronics Co.Ltd
 * Author: Wenlong Zhuang <daisen.zhuang@rock-chips.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/types.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <video/videomode.h>
#include <video/mipi_display.h>

#include "xgold_dsi_hwregs.h"
#include "xgold_dsi-rockchip.h"

static int dbg_thresd;
module_param(dbg_thresd, int, S_IRUGO | S_IWUSR);

#define DSI_DBG1(x...) do {	\
	if (unlikely(dbg_thresd >= 1))	\
		pr_info("[dsi] "x);		\
} while (0)

#define DSI_DBG2(x...) do {	\
	if (unlikely(dbg_thresd >= 2))	\
		pr_info("[dsi] "x);		\
} while (0)

#define DSI_DBG3(x...) do {	\
	if (unlikely(dbg_thresd >= 3))	\
		pr_info("[dsi] "x);		\
} while (0)

#define DSI_ERR(x...)	pr_err("[dsi] "x)

#define BYTES_TO_PIXELS(bytes, bpp) (DIV_ROUND_CLOSEST(bytes * 8, bpp))
#define PIXELS_TO_BYTES(pixels, bpp) (DIV_ROUND_CLOSEST(pixels * bpp, 8))

#define DSI_CFG_DATA_PIX	0
#define DSI_CFG_DATA_DAT	1
#define DSI_CFG_SOURCE_DPI	0
#define DSI_CFG_SOURCE_TXD	1

#define BITFLDS(_id_, _val_) \
	(((_val_) & dsi_regs[_id_].mask) << (dsi_regs[_id_].shift))

#define DSI_CFG_OFF(_mode_) (\
		BITFLDS(EXR_DSI_CFG_EN, 0) |\
		BITFLDS(EXR_DSI_CFG_LANES, 0) |\
		BITFLDS(EXR_DSI_CFG_LP, 0) |\
		BITFLDS(EXR_DSI_CFG_MODE, _mode_)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))

#define DSI_CFG_INIT(_nlanes_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (_nlanes_-1)) |\
		BITFLDS(EXR_DSI_CFG_LP, 0) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))
/* need_to_modify
#define DSI_CFG_RX_LP_DATA(_nlanes_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (_nlanes_-1)) |\
		BITFLDS(EXR_DSI_CFG_TX, 1) |\
		BITFLDS(EXR_DSI_CFG_LP, 1) |\
		BITFLDS(EXR_DSI_CFG_TURN, 1) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))
*/
#define DSI_CFG_RX_LP_STP(_stp_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, 0) |\
		BITFLDS(EXR_DSI_CFG_TX, 0) |\
		BITFLDS(EXR_DSI_CFG_LP, 1) |\
		BITFLDS(EXR_DSI_CFG_STP, _stp_) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))

#define DSI_CFG_TX_LP_DATA(_nlanes_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (0)) |\
		BITFLDS(EXR_DSI_CFG_LP, 1) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))

/* need_to_modify
#define DSI_CFG_TX_HS_DATA_ACK(_nlanes_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (_nlanes_-1)) |\
		BITFLDS(EXR_DSI_CFG_TX, 1) |\
		BITFLDS(EXR_DSI_CFG_LP, 0) |\
		BITFLDS(EXR_DSI_CFG_TURN, 1) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT))|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))
*/

#define DSI_CFG_TX_HS_DATA(_nlanes_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (_nlanes_-1)) |\
		BITFLDS(EXR_DSI_CFG_LP, 0) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))

#define DSI_CFG_TX_HS_PIXEL(_nlanes_, _mode_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (_nlanes_-1)) |\
		BITFLDS(EXR_DSI_CFG_TX, 1) |\
		BITFLDS(EXR_DSI_CFG_LP, 0) |\
		BITFLDS(EXR_DSI_CFG_MODE, _mode_)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_PIX)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_DPI))

#define DSI_REF_CLK_KHZ         (26000)
#define DSI_RATE_N_MAX          (0xFF)
#define DSI_RATE_M_MAX          (0xF)
#define DSI_RATE_MAX            (1000000000)
#define DSI_RATE_MIN            (160000000)
#define DSI_RATE(n, m)          (DSI_REF_CLK_KHZ * (n + 1) / (m + 1) * 1000)
#define DSI_RATE_OVERHEAD(r)    (r / 10 * 11)

#define TLPX_NS		50
#define PORCH_SYNC_MAX	0xFF

#define EXTREG(_id_, _addr_, _mask_, _shift_) \
	{#_id_, _addr_, _mask_, _shift_}

struct dsi_command {
	char *name;             /* command name string */
	unsigned int addr;      /* command register address */
	unsigned int mask;      /* value field mask */
	int shift;              /* value field shift */
};

/* Declaration for Reg description table */
static struct dsi_command dsi_regs[] = {
[EXR_DSI_CLC] =
	EXTREG(EXR_DSI_CLC, DSI_CLC, 0xFFFFFFFF, 0),
[EXR_DSI_CLC_RUN] =
	EXTREG(EXR_DSI_CLC_RUN, DSI_CLC, 0x3, 0),
[EXR_DSI_CLC_STAT] =
	EXTREG(EXR_DSI_CLC_STAT, DSI_CLC_STAT, 0xFFFFFFFF, 0),
[EXR_DSI_CLC_STAT_RUN] =
	EXTREG(EXR_DSI_CLC_STAT_RUN, DSI_CLC_STAT, 0x1, 0),
[EXR_DSI_CLC_STAT_MODEN] =
	EXTREG(EXR_DSI_CLC_STAT_MODEN, DSI_CLC_STAT, 0x1, 1),
[EXR_DSI_CLC_STAT_KID] =
	EXTREG(EXR_DSI_CLC_STAT_KID, DSI_CLC_STAT, 0x1, 7),
[EXR_DSI_ID] =
	EXTREG(EXR_DSI_ID, DSI_ID, 0xFFFFFFFF, 0),
[EXR_DSI_FIFO_ID] =
	EXTREG(EXR_DSI_FIFO_ID, DSI_FIFO_ID, 0xFFFFFFFF, 0),
[EXR_DSI_SRB_MSCONF_ID] =
	EXTREG(EXR_DSI_SRB_MSCONF_ID, DSI_SRB_MSCONF_ID, 0xFFFFFFFF, 0),
[EXR_DSI_SWCID] =
	EXTREG(EXR_DSI_SWCID, DSI_SWCID, 0xFFFFFFFF, 0),
[EXR_DSI_FIFO_CFG] =
	EXTREG(EXR_DSI_FIFO_CFG, DSI_FIFO_CFG, 0xFFFFFFFF, 0),
[EXR_DSI_FIFO_CTRL] =
	EXTREG(EXR_DSI_FIFO_CTRL, DSI_FIFO_CTRL, 0xFFFFFFFF, 0),
[EXR_DSI_MRPS_CTRL] =
	EXTREG(EXR_DSI_MRPS_CTRL, DSI_MRPS_CTRL, 0xFFFFFFFF, 0),
[EXR_DSI_RPS_STAT] =
	EXTREG(EXR_DSI_RPS_STAT, DSI_RPS_STAT, 0xFFFFFFFF, 0),
[EXR_DSI_TPS_CTRL] =
	EXTREG(EXR_DSI_TPS_CTRL, DSI_TPS_CTRL, 0xFFFFFFFF, 0),
[EXR_DSI_TPS_CTRL_TPS] =
	EXTREG(EXR_DSI_TPS_CTRL_TPS, DSI_TPS_CTRL, 0xFFFF, 0),
[EXR_DSI_FIFO_STAT] =
	EXTREG(EXR_DSI_FIFO_STAT, DSI_FIFO_STAT, 0xFFFFFFFF, 0),
[EXR_DSI_FIFO_STAT_RXFFS] =
	EXTREG(EXR_DSI_FIFO_STAT_RXFFS, DSI_FIFO_STAT, 0xFF, 0),
[EXR_DSI_RIS] =
	EXTREG(EXR_DSI_RIS, DSI_RIS, 0xFFFFFFFF, 0),
[EXR_DSI_IMSC] =
	EXTREG(EXR_DSI_IMSC, DSI_IMSC, 0xFFFFFFFF, 0),
[EXR_DSI_MIS] =
	EXTREG(EXR_DSI_MIS, DSI_MIS, 0xFFFFFFFF, 0),
[EXR_DSI_ISR] =
	EXTREG(EXR_DSI_ISR, DSI_ISR, 0xFFFFFFFF, 0),
[EXR_DSI_DMAE] =
	EXTREG(EXR_DSI_DMAE, DSI_DMAE, 0xFFFFFFFF, 0),
[EXR_DSI_ICR] =
	EXTREG(EXR_DSI_ICR, DSI_ICR, 0xFFFFFFFF, 0),
[EXR_DSI_CFG] =
	EXTREG(EXR_DSI_CFG, DSI_CFG, 0xFFFFFFFF, 0),
[EXR_DSI_CFG_CFG_LAT] =
	EXTREG(EXR_DSI_CFG_CFG_LAT, DSI_CFG, 0x1, 0),
[EXR_DSI_CFG_HEAD_LAT] =
	EXTREG(EXR_DSI_CFG_HEAD_LAT, DSI_CFG, 0x1, 1),
[EXR_DSI_CFG_GATE] =
	EXTREG(EXR_DSI_CFG_GATE, DSI_CFG, 0x1, 2),
[EXR_DSI_CFG_TX] =
	EXTREG(EXR_DSI_CFG_TX, DSI_CFG, 0x1, 3),
[EXR_DSI_CFG_LP] =
	EXTREG(EXR_DSI_CFG_LP, DSI_CFG, 0x1, 4),
[EXR_DSI_CFG_MODE] =
	EXTREG(EXR_DSI_CFG_MODE, DSI_CFG, 0x1, 5),
[EXR_DSI_CFG_EOT] =
	EXTREG(EXR_DSI_CFG_EOT, DSI_CFG, 0x1, 6),
[EXR_DSI_CFG_TURN] =
	EXTREG(EXR_DSI_CFG_TURN, DSI_CFG, 0x1, 7),
[EXR_DSI_CFG_VALID] =
	EXTREG(EXR_DSI_CFG_VALID, DSI_CFG, 0x1, 8),
[EXR_DSI_CFG_DATA] =
	EXTREG(EXR_DSI_CFG_DATA, DSI_CFG, 0x1, 9),
[EXR_DSI_CFG_STP] =
	EXTREG(EXR_DSI_CFG_STP, DSI_CFG, 0x1, 10),
[EXR_DSI_CFG_ULPS] =
	EXTREG(EXR_DSI_CFG_ULPS, DSI_CFG, 0x1, 11),
[EXR_DSI_CFG_EN] =
	EXTREG(EXR_DSI_CFG_EN, DSI_CFG, 0x1, 12),
[EXR_DSI_CFG_LANES] =
	EXTREG(EXR_DSI_CFG_LANES, DSI_CFG, 0x3, 13),
[EXR_DSI_CFG_ID] =
	EXTREG(EXR_DSI_CFG_ID, DSI_CFG, 0x3, 15),
[EXR_DSI_CFG_TXS] =
	EXTREG(EXR_DSI_CFG_TXS, DSI_CFG, 0x1, 17),
[EXR_DSI_CFG_TE] =
	EXTREG(EXR_DSI_CFG_TE, DSI_CFG, 0x1, 18),
[EXR_DSI_CFG_FIN] =
	EXTREG(EXR_DSI_CFG_FIN, DSI_CFG, 0x1, 19),
[EXR_DSI_CFG_VSYNC] =
	EXTREG(EXR_DSI_CFG_VSYNC, DSI_CFG, 0x1, 24),
[EXR_DSI_CFG_PSYNC] =
	EXTREG(EXR_DSI_CFG_PSYNC, DSI_CFG, 0x1, 25),
[EXR_DSI_CFG_SOURCE] =
	EXTREG(EXR_DSI_CFG_SOURCE, DSI_CFG, 0x1, 31),
[EXR_DSI_CLK] =
	EXTREG(EXR_DSI_CLK, DSI_CLK, 0xFFFFFFFF, 0),
[EXR_DSI_HEAD] =
	EXTREG(EXR_DSI_HEAD, DSI_HEAD, 0xFFFFFFFF, 0),
[EXR_DSI_HEAD_HEADER] =
	EXTREG(EXR_DSI_HEAD_HEADER, DSI_HEAD, 0xFF, 0),
[EXR_DSI_HEAD_WCNT] =
	EXTREG(EXR_DSI_HEAD_WCNT, DSI_HEAD, 0xFFFF, 8),
[EXR_DSI_HEAD_CMD] =
	EXTREG(EXR_DSI_HEAD_CMD, DSI_HEAD, 0xFF, 24),
[EXR_DSI_TO0] =
	EXTREG(EXR_DSI_TO0, DSI_TO0, 0xFFFFFFFF, 0),
[EXR_DSI_TO1] =
	EXTREG(EXR_DSI_TO1, DSI_TO1, 0xFFFFFFFF, 0),
[EXR_DSI_VID0] =
	EXTREG(EXR_DSI_VID0, DSI_VID0, 0xFFFFFFFF, 0),
[EXR_DSI_VID0_HFP_BYTES] =
	EXTREG(EXR_DSI_VID0_HFP_BYTES, DSI_VID0, 0xFF, 0),
[EXR_DSI_VID0_HBP_BYTES] =
	EXTREG(EXR_DSI_VID0_HBP_BYTES, DSI_VID0, 0xFF, 8),
[EXR_DSI_VID0_HSA_BYTES] =
	EXTREG(EXR_DSI_VID0_HSA_BYTES, DSI_VID0, 0xFF, 16),
[EXR_DSI_VID0_HFP] =
	EXTREG(EXR_DSI_VID0_HFP, DSI_VID0, 0x1, 24),
[EXR_DSI_VID0_HBP] =
	EXTREG(EXR_DSI_VID0_HBP, DSI_VID0, 0x1, 25),
[EXR_DSI_VID0_HSA] =
	EXTREG(EXR_DSI_VID0_HSA, DSI_VID0, 0x1, 26),
[EXR_DSI_VID0_HFP_LP] =
	EXTREG(EXR_DSI_VID0_HFP_LP, DSI_VID0, 0x1, 27),
[EXR_DSI_VID0_HBP_LP] =
	EXTREG(EXR_DSI_VID0_HBP_LP, DSI_VID0, 0x1, 28),
[EXR_DSI_VID0_HSA_LP] =
	EXTREG(EXR_DSI_VID0_HSA_LP, DSI_VID0, 0x1, 29),
[EXR_DSI_VID1] =
	EXTREG(EXR_DSI_VID1, DSI_VID1, 0xFFFFFFFF, 0),
[EXR_DSI_VID1_VACT_LINES] =
	EXTREG(EXR_DSI_VID1_VACT_LINES, DSI_VID1, 0xFFF, 0),
[EXR_DSI_VID1_MODE] =
	EXTREG(EXR_DSI_VID1_MODE, DSI_VID1, 0x3, 12),
[EXR_DSI_VID1_ID] =
	EXTREG(EXR_DSI_VID1_ID, DSI_VID1, 0x3, 14),
[EXR_DSI_VID1_PIXEL] =
	EXTREG(EXR_DSI_VID1_PIXEL, DSI_VID1, 0x3, 16),
[EXR_DSI_VID1_FILL_BUFFER_TO] =
	EXTREG(EXR_DSI_VID1_FILL_BUFFER_TO, DSI_VID1, 0x3FF, 18),
[EXR_DSI_VID1_LAST_CS] =
	EXTREG(EXR_DSI_VID1_LAST_CS, DSI_VID1, 0xF, 28),
[EXR_DSI_VID2] =
	EXTREG(EXR_DSI_VID2, DSI_VID2, 0xFFFFFFFF, 0),
[EXR_DSI_VID2_VFP] =
	EXTREG(EXR_DSI_VID2_VFP, DSI_VID2, 0xFF, 0),
[EXR_DSI_VID2_VBP] =
	EXTREG(EXR_DSI_VID2_VBP, DSI_VID2, 0xFF, 8),
[EXR_DSI_VID2_VSA] =
	EXTREG(EXR_DSI_VID2_VSA, DSI_VID2, 0xFF, 16),
[EXR_DSI_VID3] =
	EXTREG(EXR_DSI_VID3, DSI_VID3, 0xFFFFFFFF, 0),
[EXR_DSI_VID3_PIXEL_PACKETS] =
	EXTREG(EXR_DSI_VID3_PIXEL_PACKETS, DSI_VID3, 0xFFFF, 0),
[EXR_DSI_VID3_PIXEL_BYTES] =
	EXTREG(EXR_DSI_VID3_PIXEL_BYTES, DSI_VID3, 0xFFFF, 16),
[EXR_DSI_VID4] =
	EXTREG(EXR_DSI_VID4, DSI_VID4, 0xFFFFFFFF, 0),
[EXR_DSI_VID4_BLANK_PACKETS] =
	EXTREG(EXR_DSI_VID4_BLANK_PACKETS, DSI_VID4, 0xFFFF, 0),
[EXR_DSI_VID4_BLANK_BYTES] =
	EXTREG(EXR_DSI_VID4_BLANK_BYTES, DSI_VID4, 0xFFFF, 16),
[EXR_DSI_VID5] =
	EXTREG(EXR_DSI_VID5, DSI_VID5, 0xFFFFFFFF, 0),
[EXR_DSI_VID5_LINE_TIME] =
	EXTREG(EXR_DSI_VID5_LINE_TIME, DSI_VID5, 0xFFFF, 0),
[EXR_DSI_VID5_BLLP_TIME] =
	EXTREG(EXR_DSI_VID5_BLLP_TIME, DSI_VID5, 0xFFFF, 16),
[EXR_DSI_VID6] =
	EXTREG(EXR_DSI_VID6, DSI_VID6, 0xFFFFFFFF, 0),
[EXR_DSI_VID6_LAST_BLANK] =
	EXTREG(EXR_DSI_VID6_LAST_BLANK, DSI_VID6, 0xFFFF, 0),
[EXR_DSI_VID6_LAST_PIXEL] =
	EXTREG(EXR_DSI_VID6_LAST_PIXEL, DSI_VID6, 0xFFFF, 16),
[EXR_DSI_PHY0] =
	EXTREG(EXR_DSI_PHY0, DSI_PHY0, 0xFFFFFFFF, 0),
[EXR_DSI_PHY0_SHARE] =
	EXTREG(EXR_DSI_PHY0_SHARE, DSI_PHY0, 0x1, 0),
[EXR_DSI_PHY0_M] =
	EXTREG(EXR_DSI_PHY0_M, DSI_PHY0, 0xF, 1),
[EXR_DSI_PHY0_N] =
	EXTREG(EXR_DSI_PHY0_N, DSI_PHY0, 0xFF, 5),
[EXR_DSI_PHY0_POWERUP] =
	EXTREG(EXR_DSI_PHY0_POWERUP, DSI_PHY0, 0x3F, 13),
[EXR_DSI_PHY0_CALIB] =
	EXTREG(EXR_DSI_PHY0_CALIB, DSI_PHY0, 0x3F, 19),
[EXR_DSI_PHY0_TO_LP_HS_REQ] =
	EXTREG(EXR_DSI_PHY0_TO_LP_HS_REQ, DSI_PHY0, 0x3F, 25),
[EXR_DSI_PHY1] =
	EXTREG(EXR_DSI_PHY1, DSI_PHY1, 0xFFFFFFFF, 0),
[EXR_DSI_PHY1_TO_LP_HS_DIS] =
	EXTREG(EXR_DSI_PHY1_TO_LP_HS_DIS, DSI_PHY1, 0x3F, 0),
[EXR_DSI_PHY1_TO_LP_EOT] =
	EXTREG(EXR_DSI_PHY1_TO_LP_EOT, DSI_PHY1, 0x3F, 6),
[EXR_DSI_PHY1_TO_HS_ZERO] =
	EXTREG(EXR_DSI_PHY1_TO_HS_ZERO, DSI_PHY1, 0x3F, 12),
[EXR_DSI_PHY1_TO_HS_FLIP] =
	EXTREG(EXR_DSI_PHY1_TO_HS_FLIP, DSI_PHY1, 0x3F, 18),
[EXR_DSI_PHY1_LP_CLK_DIV] =
	EXTREG(EXR_DSI_PHY1_LP_CLK_DIV, DSI_PHY1, 0x3F, 24),
[EXR_DSI_PHY2] =
	EXTREG(EXR_DSI_PHY2, DSI_PHY2, 0xFFFFFFFF, 0),
[EXR_DSI_PHY2_HS_CLK_PRE] =
	EXTREG(EXR_DSI_PHY2_HS_CLK_PRE, DSI_PHY2, 0x3FF, 0),
[EXR_DSI_PHY2_HS_CLK_POST] =
	EXTREG(EXR_DSI_PHY2_HS_CLK_POST, DSI_PHY2, 0x3FF, 10),
[EXR_DSI_PHY2_DAT_DELAY] =
	EXTREG(EXR_DSI_PHY2_DAT_DELAY, DSI_PHY2, 0xF, 20),
[EXR_DSI_PHY2_CLK_DELAY] =
	EXTREG(EXR_DSI_PHY2_CLK_DELAY, DSI_PHY2, 0xF, 24),
[EXR_DSI_PHY2_LPTX_TFALL] =
	EXTREG(EXR_DSI_PHY2_LPTX_TFALL, DSI_PHY2, 0x7, 28),
[EXR_DSI_PHY3] =
	EXTREG(EXR_DSI_PHY3, DSI_PHY3, 0xFFFFFFFF, 0),
[EXR_DSI_PHY3_EN] =
	EXTREG(EXR_DSI_PHY3_EN, DSI_PHY3, 0x1, 0),
[EXR_DSI_PHY3_LPTX_TRISE] =
	EXTREG(EXR_DSI_PHY3_LPTX_TRISE, DSI_PHY3, 0x7, 1),
[EXR_DSI_PHY3_LPTX_VREF] =
	EXTREG(EXR_DSI_PHY3_LPTX_VREF, DSI_PHY3, 0x1F, 4),
[EXR_DSI_STAT] =
	EXTREG(EXR_DSI_STAT, DSI_STAT, 0xFFFFFFFF, 0),
[EXR_DSI_STAT_DSI_BSY] =
	EXTREG(EXR_DSI_STAT_DSI_STAT, DSI_STAT, 0x1, 0),
[EXR_DSI_STAT_DSI_FULL] =
	EXTREG(EXR_DSI_STAT_DSI_FULL, DSI_STAT, 0x1, 2),
[EXR_DSI_STAT_DSI_DIR] =
	EXTREG(EXR_DSI_STAT_DSI_DIR, DSI_STAT, 0x1, 3),
[EXR_DSI_STAT_DSI_LOCK] =
	EXTREG(EXR_DSI_STAT_DSI_LOCK, DSI_STAT, 0x1, 4),
[EXR_DSI_TXD] =
	EXTREG(EXR_DSI_TXD, DSI_TXD, 0xFFFFFFFF, 0),
[EXR_DSI_RXD] =
	EXTREG(EXR_DSI_RXD, DSI_RXD, 0xFFFFFFFF, 0),
};

static unsigned int dsi_read_field(struct xgold_mipi_dsi *dsi, unsigned int id)
{
	struct dsi_command *cmd = &dsi_regs[id];

	return (ioread32(dsi->regbase + cmd->addr) >> cmd->shift) &
		cmd->mask;
}

static void dsi_write_field(struct xgold_mipi_dsi *dsi,
			    unsigned int id, u32 val)
{
	u32 regval = 0;
	struct dsi_command *cmd = &dsi_regs[id];
	struct dsi_display *display = &dsi->display;

	if (id == EXR_DSI_CFG)
		val |= display->dif.dsi.dsi_cfg_reg;

	if (cmd->mask == 0xFFFFFFFF) {
		iowrite32(val, dsi->regbase + cmd->addr);
	} else {
		val = (val << cmd->shift) & (cmd->mask << cmd->shift);
		regval = ioread32(dsi->regbase + cmd->addr) &
			~(cmd->mask << cmd->shift);
		iowrite32(val | regval, dsi->regbase + cmd->addr);
	}
}

static int dsi_wait_status(struct xgold_mipi_dsi *dsi, unsigned int reg,
			   unsigned int value, unsigned int mask,
			   unsigned int delay, unsigned int count)
{
	unsigned int read_value;
	int found = 0;

	do {
		read_value = dsi_read_field(dsi, reg);
		if ((read_value & mask) == value) {
			found = 1;
			break;
		}
		if (delay)
			msleep(delay);
	} while (--count);

	if (!found) {
		DSI_ERR("%s() FAILED reg 0x%x 0x%08x != 0x%08x, mask=0x%x\n",
			__func__, reg, value, read_value, mask);
		found = -EBUSY;
	} else
		pr_debug("%s() reg 0x%x 0x%08x == 0x%08x\n", __func__,
			 reg, value, read_value);

	return found;
}

static inline void dsi_hal_irq_dsi_fin(struct xgold_mipi_dsi *dsi)
{
	complete(&dsi->sync.dsifin);
}

static inline void dsi_hal_irq_dsi_tr1(struct xgold_mipi_dsi *dsi)
{
	complete(&dsi->sync.dsitr1);
}

static inline void dsi_hal_irq_dsi_tr2(struct xgold_mipi_dsi *dsi)
{
	dsi_write_field(dsi, EXR_DSI_CFG, BITFLDS(EXR_DSI_CFG_TX, 1) |
		BITFLDS(EXR_DSI_CFG_LP, 1) |
		BITFLDS(EXR_DSI_CFG_MODE, 1) |
		BITFLDS(EXR_DSI_CFG_EOT, 1) |
		BITFLDS(EXR_DSI_CFG_TURN, 1) |
		BITFLDS(EXR_DSI_CFG_DATA, 1) |
		BITFLDS(EXR_DSI_CFG_EN, 1) |
		BITFLDS(EXR_DSI_CFG_SOURCE, 1));
}

#define DSI_INTERRUPT_CLEAR(_irq_) {\
if (dsi_irq_status & _irq_) { \
		dsi_irq_clear |= _irq_; \
	} }

#define DSI_LOG_INTERRUPT_DEBUG_HW_CB(_irq_, _cb_, _p_) {\
if (dsi_irq_status & _irq_) { \
		_cb_(_p_); \
		dsi_irq_clear |= _irq_; \
	} }

#define DSI_LOG_INTERRUPT_DEBUG_DBG_CB(_irq_, _cb_, _p_) {\
if (dsi_irq_status & _irq_) { \
		pr_info("[dsi]"#_irq_ "\n"); \
		_cb_(_p_); \
		dsi_irq_clear |= _irq_; \
	} }

#define DSI_LOG_INTERRUPT_ERROR(_irq_, _p_) {\
if (dsi_irq_status & _irq_) { \
	unsigned int stat, ris, txffs; \
	stat = dsi_read_field(_p_, EXR_DSI_STAT); \
	ris = dsi_read_field(_p_, EXR_DSI_RIS); \
	txffs = dsi_read_field(_p_, EXR_DSI_FIFO_STAT); \
printk_ratelimited(KERN_DEBUG "[dsi]"#_irq_\
" DSI_STAT:0x%08x DSI_RIS:0x%08x DSI_FIFO_STAT:0x%08x\n",\
	stat, ris, txffs); \
		dsi_irq_clear |= _irq_; \
	} }

static irqreturn_t dsi_hal_irq_err(int irq, void *dev_id)
{
	unsigned int dsi_irq_status = 0;
	unsigned int dsi_irq_clear = 0;
	struct xgold_mipi_dsi *dsi = (struct xgold_mipi_dsi *)dev_id;

	dsi_irq_status = dsi_read_field(dsi, EXR_DSI_RIS);

	/* clear treated handled interrupts */
	dsi_irq_clear = dsi_irq_status;
	dsi_write_field(dsi, EXR_DSI_ICR, dsi_irq_clear);
	/*pr_info("%s: Hardware Interrupt EXR_DSI_RIS = 0x%08x\n",
		__func__, dsi_irq_status);*/

	DSI_INTERRUPT_CLEAR(DSI_IRQ_ERR_IDLE);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSITR0, dsi);
	DSI_LOG_INTERRUPT_DEBUG_DBG_CB(DSI_IRQ_ERR_DSITR1,
				       dsi_hal_irq_dsi_tr1, dsi);
	DSI_LOG_INTERRUPT_DEBUG_DBG_CB(DSI_IRQ_ERR_DSITR2,
				       dsi_hal_irq_dsi_tr2, dsi);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSITR3, dsi);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSIUFL, dsi);
	DSI_LOG_INTERRUPT_DEBUG_HW_CB(DSI_IRQ_ERR_DSIFIN,
				       dsi_hal_irq_dsi_fin, dsi);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSILTO, dsi);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSIHTO, dsi);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSIRTO, dsi);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSIESC, dsi);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSISYN, dsi);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSICTR, dsi);
	DSI_INTERRUPT_CLEAR(DSI_IRQ_ERR_DSICON);
	DSI_LOG_INTERRUPT_ERROR(DSI_IRQ_ERR_DSIOFL, dsi);

	/* make sure clear interrupts complete */
	/* dsi_wait_status(dsi, EXR_DSI_RIS, 0, dsi_irq_clear, 0, 100);*/
	return IRQ_HANDLED;
}

static void dsi_hal_unregister_irq(struct xgold_mipi_dsi *dsi)
{
	/* TODO gra_write_field(EXR_DIF_IMSC,0); */
	free_irq(dsi->irq.rx_breq, 0);
	free_irq(dsi->irq.err, 0);
	free_irq(dsi->irq.tx, 0);
	free_irq(dsi->irq.rx, 0);
}

static struct irqaction dsi_err_irq = {
	.dev_id = 0,
	.name = "dsi.err",
	.handler = dsi_hal_irq_err,
	.flags = IRQF_SHARED,
};

#define DSI_SETUP_IRQ(_irq_, _action_, _data_) {\
	if (_irq_) { \
		_action_.dev_id = (void *)_data_; \
		ret = setup_irq(_irq_, &_action_); \
		if (ret != 0) { \
			pr_info("setup irq %s %d failed(ret = %d)\n", \
					_action_.name, _irq_, ret); \
			goto exit_stage_1; \
		} \
	} }

static int dsi_hal_install_irqs(struct xgold_mipi_dsi *dsi)
{
	int ret = 0;

	dsi_write_field(dsi, EXR_DSI_ICR, 0x7FFFFF);
	DSI_SETUP_IRQ(dsi->irq.err, dsi_err_irq, dsi);

	return 0;

exit_stage_1:
	dsi_hal_unregister_irq(dsi);
	return ret;
}

static int dsi_irq_probe(struct xgold_mipi_dsi *dsi)
{
	dsi_write_field(dsi, EXR_DSI_IMSC, 0); /* mask interrupt */

	return dsi_hal_install_irqs(dsi);
}

static int __maybe_unused dsi_irq_remove(struct xgold_mipi_dsi *dsi)
{
	/* unregister irq's */
	/* to detect errors on mode switching unregistering irqs must be done
	 * after switching to config mode */
	dsi_hal_unregister_irq(dsi);

	pr_info("dsi irq exit successfully!\n");
	return 0;
}

static void dsi_interrupt_setup(struct xgold_mipi_dsi *dsi)
{
	dsi_write_field(dsi, EXR_DSI_ICR, 0x7FFFFF);
}

static int dsi_get_rate(struct dsi_display *display)
{
	return DSI_RATE(display->dif.dsi.n, display->dif.dsi.m);
}

static int dsi_get_bllp(struct dsi_display *display,
			int nlines, int bytes, int clk,
			int fps, int bitrate, int nlanes,
			int *bllp_time, int *line_time)
{
	/* bits per frames */
	unsigned int bitpframe = bytes * nlines * 8;
	/* maximum framerate */
	unsigned int maxfrate = bitrate * nlanes / bitpframe;
	/* shortest line time */
	unsigned int slt = NSEC_PER_SEC / (maxfrate * nlines);
	/* target line time */
	unsigned int tlt = NSEC_PER_SEC / (fps * nlines);
	/* clock cycle duration in ps */
	unsigned int clk_time = 1000000000 / (clk / 1000);

	*line_time = tlt * 1000 / clk_time;
	*bllp_time = *line_time - DIV_ROUND_UP(slt * 1000, clk_time);
	if (display->dif.dsi.video_mode != DSI_BURST || *bllp_time < 0)
		*bllp_time = 0;

	DSI_DBG2("%d bytes / %d lines\n", bytes, nlines);
	DSI_DBG2("bits / frame = %d bits\n", bitpframe);
	DSI_DBG2("fps target %d fps (max=%d)\n", fps, maxfrate);
	DSI_DBG2("active time  = %d ns\n", slt);
	DSI_DBG2("target time  = %d ns\n", tlt);
	DSI_DBG2("clock cycle  = %d\n", clk_time);
	DSI_DBG2("bllp_time 0x%08x(%d)\n", *bllp_time, *bllp_time);
	DSI_DBG2("line_time 0x%08x(%d)\n", *line_time, *line_time);

	return 0;
}

static void dsi_dphy_calculation(struct dsi_display *display)
{
	int ui_ps = 0, ths_prepare_ns, ths_trail_ns, ths_prepare_zero_ns,
		tclk_post_ns, tclk_prepare_ns;

	if (display->dif.dsi.bitrate)
		ui_ps = DIV_ROUND_CLOSEST(1000000000,
					  display->dif.dsi.bitrate / 1000);

	/*
	 * THS-PREPARE is between 40ns + 4UI and 85ns + 6UI,
	 * we set the THS-PREPARE to average of THS-PREPARE.
	 * THS-PREPARE = (40ns + 4UI + 85ns + 6UI) / 2 = 62.5ns + 5UI
	 */
	ths_prepare_ns = DIV_ROUND_CLOSEST(62500 + 5 * ui_ps, 1000);

	/*
	 * THS-TRAIL is 60ns + 4UI, we set THS-TRAIL to 63ns + 4UI for safety
	 * margin.
	 */
	ths_trail_ns = DIV_ROUND_UP(63000 + 4 * ui_ps, 1000);

	/*
	 * THS-PREPARE + THS-ZERO is 145ns + 10UI, we set THS-PREPARE + THS-ZERO
	 * to 152.25ns + 11UI for safety margin.
	 */
	ths_prepare_zero_ns = DIV_ROUND_UP(152250 + 11 * ui_ps, 1000);

	/*
	 * TCLK-POST is 60ns + 52UI, we set TCLK-POST to 63ns + 55UI for safety
	 * margin.
	 */
	tclk_post_ns = DIV_ROUND_UP(63000 + 55 * ui_ps, 1000);

	/*
	 * TCLK-PREPARE is between 38ns and 95ns, we set TCLK-PREPARE to average
	 * of TCLK-PREPARE 67.
	 * margin.
	 */
	tclk_prepare_ns = 67;

	display->dif.dsi.pwup = 6;
	display->dif.dsi.calib = 3;
	display->dif.dsi.data_delay = 7;
	display->dif.dsi.clock_delay = 7;
	display->dif.dsi.lp_tx_tfall = 2;
	display->dif.dsi.lp_tx_trise = 2;
	display->dif.dsi.lp_tx_vref = 31;
	display->dif.dsi.lp_clk_div =
		DIV_ROUND_UP(display->dif.dsi.dc_clk_rate / 1000 * TLPX_NS,
			     1000000) - 1;
	display->dif.dsi.to_lp_hs_req = display->dif.dsi.lp_clk_div;
	display->dif.dsi.to_hs_flip = DIV_ROUND_UP(display->dif.dsi.bitrate /
		1000 * ths_trail_ns, 1000000 * 8);
	display->dif.dsi.to_hs_zero = DIV_ROUND_UP(display->dif.dsi.bitrate /
		1000 * ths_prepare_zero_ns, 1000000 * 8) - 5;
	display->dif.dsi.to_lp_hs_eot =
		DIV_ROUND_UP(display->dif.dsi.dc_clk_rate / 1000 *
		(ths_trail_ns + 18), 1000000) + DIV_ROUND_UP(3000000,
		display->dif.dsi.bitrate / 1000);
	display->dif.dsi.to_lp_hs_dis =
		DIV_ROUND_UP(display->dif.dsi.dc_clk_rate / 1000 *
		ths_prepare_ns, 1000000) - 1;
	display->dif.dsi.to_hs_clk_post =
		DIV_ROUND_UP(display->dif.dsi.dc_clk_rate / 1000 * tclk_post_ns,
		1000000);
	display->dif.dsi.to_hs_clk_pre =
		DIV_ROUND_UP(display->dif.dsi.dc_clk_rate / 1000 *
		tclk_prepare_ns, 1000000) + 5;
}

static void dsi_rate_calculation(struct dsi_display *display)
{
	int diff, diff_min = DSI_RATE_MAX, n = 0, m = 0;

	if (!display->dif.dsi.bitrate)
		display->dif.dsi.bitrate = DSI_RATE_OVERHEAD((display->xres +
			BYTES_TO_PIXELS(display->dif.dsi.hfp, display->bpp) +
			BYTES_TO_PIXELS(display->dif.dsi.hbp, display->bpp) +
			BYTES_TO_PIXELS(display->dif.dsi.hsa, display->bpp)) *
			(display->yres + display->dif.dsi.vfp +
			display->dif.dsi.vbp + display->dif.dsi.vsa) *
			display->fps / display->dif.dsi.nblanes * display->bpp);

	if (display->dif.dsi.bitrate > DSI_RATE_MAX)
		display->dif.dsi.bitrate = DSI_RATE_MAX;
	else if (display->dif.dsi.bitrate < DSI_RATE_MIN)
		display->dif.dsi.bitrate = DSI_RATE_MIN;

	for (m = 1; m <= DSI_RATE_M_MAX; m++) {
		for (n = 1; n <= DSI_RATE_N_MAX; n++) {
			diff = DSI_RATE(n, m) - display->dif.dsi.bitrate;

			if (diff < 0)
				continue;

			if (diff < diff_min) {
				diff_min = diff;
				display->dif.dsi.n = n;
				display->dif.dsi.m = m;
			}
		}
	}
}

static int dsi_configure_video_mode(struct xgold_mipi_dsi *dsi,
				    int stride, int nlines)
{
	struct dsi_display *display = &dsi->display;
	unsigned int vid0, vid1, vid2, vid3, vid4, vid5, vid6;
	struct dsi_display_if_mipi_dsi *dif = &display->dif.dsi;

	if (display->dif.dsi.mode != DSI_VIDEO) {
		DSI_DBG2("%s: not video mode\n", __func__);
		return -EINVAL;
	}

	vid0 = BITFLDS(EXR_DSI_VID0_HFP,
		       (!!display->dif.dsi.hfp))|
	       BITFLDS(EXR_DSI_VID0_HBP,
		       (!!display->dif.dsi.hbp))|
	       BITFLDS(EXR_DSI_VID0_HSA,
		       (!!display->dif.dsi.hsa))|
	       BITFLDS(EXR_DSI_VID0_HFP_LP, dif->hfp_lp)|
	       BITFLDS(EXR_DSI_VID0_HBP_LP, dif->hbp_lp)|
	       BITFLDS(EXR_DSI_VID0_HSA_LP, dif->hsa_lp)|
	       BITFLDS(EXR_DSI_VID0_HFP_BYTES, dif->hfp)|
	       BITFLDS(EXR_DSI_VID0_HBP_BYTES, dif->hbp)|
	       BITFLDS(EXR_DSI_VID0_HSA_BYTES, dif->hsa);

	vid1 = BITFLDS(EXR_DSI_VID1_VACT_LINES, nlines)|
	       BITFLDS(EXR_DSI_VID1_MODE, dif->video_mode)|
	       BITFLDS(EXR_DSI_VID1_ID, dif->id)|
	       BITFLDS(EXR_DSI_VID1_PIXEL, dif->video_pixel)|
	       BITFLDS(EXR_DSI_VID1_FILL_BUFFER_TO, 0xFF);

	vid2 = BITFLDS(EXR_DSI_VID2_VFP, dif->vfp)|
	       BITFLDS(EXR_DSI_VID2_VBP, dif->vbp)|
	       BITFLDS(EXR_DSI_VID2_VSA, dif->vsa);

	vid3 = BITFLDS(EXR_DSI_VID3_PIXEL_BYTES, stride)|
	       BITFLDS(EXR_DSI_VID3_PIXEL_PACKETS, 1);

	vid4 = BITFLDS(EXR_DSI_VID4_BLANK_BYTES, 0)|
	       BITFLDS(EXR_DSI_VID4_BLANK_PACKETS, 0);

	vid5 = BITFLDS(EXR_DSI_VID5_LINE_TIME, dif->line_time)|
	       BITFLDS(EXR_DSI_VID5_BLLP_TIME, dif->bllp_time);

	vid6 = BITFLDS(EXR_DSI_VID6_LAST_BLANK, stride)|
	       BITFLDS(EXR_DSI_VID6_LAST_PIXEL, stride);

	DSI_DBG2(
		"MIPI-DSI video:0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x)\n",
		vid0, vid1, vid2, vid3, vid4, vid5, vid6);

	dsi_write_field(dsi, EXR_DSI_VID0, vid0);
	dsi_write_field(dsi, EXR_DSI_VID1, vid1);
	dsi_write_field(dsi, EXR_DSI_VID2, vid2);
	dsi_write_field(dsi, EXR_DSI_VID3, vid3);
	dsi_write_field(dsi, EXR_DSI_VID4, vid4);
	dsi_write_field(dsi, EXR_DSI_VID5, vid5);
	dsi_write_field(dsi, EXR_DSI_VID6, vid6);

	return 0;
}

static void dsi_set_phy(struct xgold_mipi_dsi *dsi, int on)
{
	struct dsi_display *display = &dsi->display;
	unsigned int phy0 = 0, phy1 = 0, phy2 = 0, phy3 = 0;

	if (!on) {
		phy0 = BITFLDS(EXR_DSI_PHY0_SHARE, 0x0) |
		       BITFLDS(EXR_DSI_PHY0_M, 0) |
		       BITFLDS(EXR_DSI_PHY0_N, 0xFF) |
		       BITFLDS(EXR_DSI_PHY0_POWERUP,
			       display->dif.dsi.pwup) |
		       BITFLDS(EXR_DSI_PHY0_CALIB,
			       display->dif.dsi.calib) |
		       BITFLDS(EXR_DSI_PHY0_TO_LP_HS_REQ,
			       display->dif.dsi.to_lp_hs_req);
	} else {
		phy0 = BITFLDS(EXR_DSI_PHY0_SHARE, 0x0) |
		       BITFLDS(EXR_DSI_PHY0_M,
			       display->dif.dsi.m) |
		       BITFLDS(EXR_DSI_PHY0_N,
			       display->dif.dsi.n) |
		       BITFLDS(EXR_DSI_PHY0_POWERUP,
			       display->dif.dsi.pwup) |
		       BITFLDS(EXR_DSI_PHY0_CALIB,
			       display->dif.dsi.calib) |
		       BITFLDS(EXR_DSI_PHY0_TO_LP_HS_REQ,
			       display->dif.dsi.to_lp_hs_req);
	}

	phy1 = BITFLDS(EXR_DSI_PHY1_TO_LP_HS_DIS,
		       display->dif.dsi.to_lp_hs_dis) |
	       BITFLDS(EXR_DSI_PHY1_TO_LP_EOT,
		       display->dif.dsi.to_lp_hs_eot) |
	       BITFLDS(EXR_DSI_PHY1_TO_HS_ZERO,
		       display->dif.dsi.to_hs_zero) |
	       BITFLDS(EXR_DSI_PHY1_TO_HS_FLIP,
		       display->dif.dsi.to_hs_flip) |
	       BITFLDS(EXR_DSI_PHY1_LP_CLK_DIV,
		       display->dif.dsi.lp_clk_div);

	phy2 = BITFLDS(EXR_DSI_PHY2_HS_CLK_PRE,
		       display->dif.dsi.to_hs_clk_pre) |
	       BITFLDS(EXR_DSI_PHY2_HS_CLK_POST,
		       display->dif.dsi.to_hs_clk_post) |
	       BITFLDS(EXR_DSI_PHY2_DAT_DELAY,
		       display->dif.dsi.data_delay) |
	       BITFLDS(EXR_DSI_PHY2_CLK_DELAY,
		       display->dif.dsi.clock_delay) |
	       BITFLDS(EXR_DSI_PHY2_LPTX_TFALL,
		       display->dif.dsi.lp_tx_tfall);

	phy3 = BITFLDS(EXR_DSI_PHY3_EN, 0x1) |
	       BITFLDS(EXR_DSI_PHY3_LPTX_TRISE,
		       display->dif.dsi.lp_tx_trise) |
	       BITFLDS(EXR_DSI_PHY3_LPTX_VREF,
		       display->dif.dsi.lp_tx_vref);

	DSI_DBG2("MIPI-DSI @%d bps (%d,%d): 0x%08x 0x%08x 0x%08x 0x%08x)\n",
		 dsi_get_rate(display),
		 display->dif.dsi.n,
		 display->dif.dsi.m,
		 phy0, phy1, phy2, phy3);

	dsi_write_field(dsi, EXR_DSI_PHY0, phy0);
	dsi_write_field(dsi, EXR_DSI_PHY1, phy1);
	dsi_write_field(dsi, EXR_DSI_PHY2, phy2);
	dsi_write_field(dsi, EXR_DSI_PHY3, phy3);

	if (on) {
		/* wait for PLL lock */
		dsi_wait_status(dsi, EXR_DSI_STAT_DSI_LOCK, 1, 1, 0, 1000);
	}
}

int xgold_dsi_init_config(struct xgold_mipi_dsi *dsi,
			  struct mipi_dsi_device *device)
{
	struct dsi_display *display = &dsi->display;

	display->dif.dsi.nblanes = device->lanes;
	/* dsi->channel = device->channel; */

	/*
	 * The first bit of mode_flags specifies display configuration.
	 * If this bit is set[= MIPI_DSI_MODE_VIDEO], dsi will support video
	 * mode, otherwise it will support command mode.
	 */
	if (device->mode_flags & MIPI_DSI_MODE_VIDEO) {
		if (device->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
			display->dif.dsi.video_mode = DSI_BURST;
		if (device->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
			display->dif.dsi.video_mode = DSI_PULSES;
	}

	if (device->mode_flags & MIPI_DSI_MODE_EOT_PACKET)
		display->dif.dsi.eot = 1;

	/*
	 * Use non-continuous clock mode if the periparal wants and
	 * host controller supports
	 *
	 * In non-continous clock mode, host controller will turn off
	 * the HS clock between high-speed transmissions to reduce
	 * power consumption.
	 */
	if (device->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		display->dif.dsi.gate = 1;

	switch (device->format) {
	case MIPI_DSI_FMT_RGB888:
		display->bpp = 24;
		display->dif.dsi.video_pixel = DSI_PIX_BIT24P;
		break;
	case MIPI_DSI_FMT_RGB666:
		display->bpp = 18;
		display->dif.dsi.video_pixel = DSI_PIX_BIT18P;
		break;
	case MIPI_DSI_FMT_RGB565:
		display->bpp = 16;
		display->dif.dsi.video_pixel = DSI_PIX_BIT16P;
		break;
	default:
		dev_err(dsi->dev, "invalid pixel format\n");
		return -EINVAL;
	}

	display->dif.dsi.dsi_cfg_reg = BITFLDS(EXR_DSI_CFG_VSYNC, 1) |
		BITFLDS(EXR_DSI_CFG_PSYNC, 1) |
		BITFLDS(EXR_DSI_CFG_EOT, display->dif.dsi.eot) |
		BITFLDS(EXR_DSI_CFG_GATE, display->dif.dsi.gate);

	return 0;
}

void xgold_dsi_set_display_mode(struct xgold_mipi_dsi *dsi,
				struct videomode *vm, int fps)
{
	struct dsi_display *display = &dsi->display;
	struct dsi_display_if_mipi_dsi *dif = &display->dif.dsi;

	display->fps = fps;
	display->xres = vm->hactive;
	display->yres = vm->vactive;

	display->dif.dsi.hfp = PIXELS_TO_BYTES(vm->hfront_porch, display->bpp);
	if (display->dif.dsi.hfp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! HFP = %d, MAX HFP is %d\n", __func__,
			vm->hfront_porch,
			BYTES_TO_PIXELS(PORCH_SYNC_MAX, display->bpp));
		display->dif.dsi.hfp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.hbp = PIXELS_TO_BYTES(vm->hback_porch, display->bpp);
	if (display->dif.dsi.hbp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! HBP = %d, MAX HBP is %d\n", __func__,
			vm->hback_porch,
			BYTES_TO_PIXELS(PORCH_SYNC_MAX, display->bpp));
		display->dif.dsi.hbp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.hsa = PIXELS_TO_BYTES(vm->hsync_len, display->bpp);
	if (display->dif.dsi.hsa > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! HSA = %d, MAX HSA is %d\n", __func__,
			vm->hsync_len,
			BYTES_TO_PIXELS(PORCH_SYNC_MAX, display->bpp));
		display->dif.dsi.hsa = PORCH_SYNC_MAX;
	}

	display->dif.dsi.vfp = vm->vfront_porch;
	if (display->dif.dsi.vfp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! VFP = %d, MAX VFP is %d\n", __func__,
			vm->vfront_porch, PORCH_SYNC_MAX);
		display->dif.dsi.vfp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.vbp = vm->vback_porch;
	if (display->dif.dsi.vbp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! VBP = %d, MAX VBP is %d\n", __func__,
			vm->vback_porch, PORCH_SYNC_MAX);
		display->dif.dsi.vbp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.vsa = vm->vsync_len;
	if (display->dif.dsi.vsa > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! VSA = %d, MAX VSA is %d\n", __func__,
			vm->vsync_len, PORCH_SYNC_MAX);
		display->dif.dsi.vsa = PORCH_SYNC_MAX;
	}
	display->dif.dsi.hfp_lp = 0;
	display->dif.dsi.hbp_lp = 0;
	display->dif.dsi.hsa_lp = 0;

	dsi_rate_calculation(display);
	dsi_dphy_calculation(display);
	dsi_get_bllp(display,
		     display->yres + dif->vfp + dif->vbp + dif->vsa,
		     PIXELS_TO_BYTES(display->xres, display->bpp) +
		     dif->hfp + dif->hbp + dif->hsa,
		     dif->dc_clk_rate,
		     display->fps,
		     dsi_get_rate(display),
		     dif->nblanes, &dif->bllp_time, &dif->line_time);
}

void xgold_dsi_send_short_packet(struct xgold_mipi_dsi *dsi,
				 const struct mipi_dsi_msg *msg)
{
	struct dsi_display *display = &dsi->display;
	const unsigned char *data_msg = msg->tx_buf;
	unsigned int dsihead =
		BITFLDS(EXR_DSI_HEAD_WCNT, data_msg[0]) |
		BITFLDS(EXR_DSI_HEAD_HEADER, msg->type);
	unsigned int dsicfg = 0;

	if (msg->tx_len > 1)
		dsihead |= data_msg[1] << 16;

	if (msg->flags & MIPI_DSI_MSG_USE_LPM)
		dsicfg = DSI_CFG_TX_LP_DATA(1);
	else
		dsicfg = DSI_CFG_TX_HS_DATA(
				display->dif.dsi.nblanes);

	DSI_DBG2("dsi short pkt: (head:0x%08x cfg:0x%08x)\n",
		 dsihead, dsicfg);

	dsi_write_field(dsi, EXR_DSI_VID3,
			BITFLDS(EXR_DSI_VID3_PIXEL_PACKETS, 1));

	dsi_write_field(dsi, EXR_DSI_HEAD, dsihead);
	dsi_write_field(dsi, EXR_DSI_CFG,
			dsicfg | BITFLDS(EXR_DSI_CFG_HEAD_LAT, 1));
	dsi_write_field(dsi, EXR_DSI_CFG, dsicfg);
	dsi_write_field(dsi, EXR_DSI_CFG,
			dsicfg | BITFLDS(EXR_DSI_CFG_TX, 1) |
			BITFLDS(EXR_DSI_CFG_CFG_LAT, 1));
	dsi_write_field(dsi, EXR_DSI_CFG,
			dsicfg | BITFLDS(EXR_DSI_CFG_TX, 1));
}

void xgold_dsi_send_long_packet(struct xgold_mipi_dsi *dsi,
				const struct mipi_dsi_msg *msg)
{
	struct dsi_display *display = &dsi->display;
	const unsigned char *data_msg = msg->tx_buf;
	unsigned int length = msg->tx_len;
	unsigned int dsihead =
		BITFLDS(EXR_DSI_HEAD_WCNT, length) |
		BITFLDS(EXR_DSI_HEAD_HEADER, msg->type);
	unsigned int dsicfg = 0;

	/* the first data byte of DCS long packet must be put
	 * in CMDBYTE of DSI_HEAD */
	if (msg->type == MIPI_DSI_DCS_LONG_WRITE && length > 0) {
		dsihead |= BITFLDS(EXR_DSI_HEAD_CMD, *data_msg++);
		length--;
	}

	if (msg->flags & MIPI_DSI_MSG_USE_LPM)
		dsicfg = DSI_CFG_TX_LP_DATA(1);
	else
		dsicfg = DSI_CFG_TX_HS_DATA(
				display->dif.dsi.nblanes);

	DSI_DBG2("dsi long dma pkt: wcnt:0x%04x (head:0x%08x cfg:0x%08x)\n",
		 length, dsihead, dsicfg);

	dsi_write_field(dsi, EXR_DSI_VID3,
			BITFLDS(EXR_DSI_VID3_PIXEL_PACKETS, 1));
	dsi_write_field(dsi, EXR_DSI_VID6,
			BITFLDS(EXR_DSI_VID6_LAST_PIXEL, length));
	dsi_write_field(dsi, EXR_DSI_TPS_CTRL,
			BITFLDS(EXR_DSI_TPS_CTRL_TPS, length));

	while (length > 0) {
		int j = 0;
		unsigned int reg = 0;

		for (j = 0; j < 4 && length; j++) {
			length--;
			reg |= ((uint8_t) *data_msg++)<<(j*8);
		}

		dsi_write_field(dsi, EXR_DSI_TXD, reg);
		DSI_DBG2("payload 0x%08x\n", reg);
	}

	dsi_write_field(dsi, EXR_DSI_HEAD, dsihead);
	dsi_write_field(dsi, EXR_DSI_CFG,
			dsicfg | BITFLDS(EXR_DSI_CFG_HEAD_LAT, 1));
	dsi_write_field(dsi, EXR_DSI_CFG, dsicfg);
	dsi_write_field(dsi, EXR_DSI_CFG,
			dsicfg | BITFLDS(EXR_DSI_CFG_TX, 1) |
			BITFLDS(EXR_DSI_CFG_CFG_LAT, 1));
	dsi_write_field(dsi, EXR_DSI_CFG, dsicfg |
			BITFLDS(EXR_DSI_CFG_TX, 1));
}

void xgold_dsi_start_video(struct xgold_mipi_dsi *dsi)
{
	struct dsi_display *display = &dsi->display;
	unsigned int dsicfg;

	dsi_write_field(dsi, EXR_DSI_CFG, DSI_CFG_OFF(DSI_VIDEO));
	dsi_write_field(dsi, EXR_DSI_IMSC,
			DSI_IRQ_ERR_MASK & (~DSI_IRQ_ERR_DSIFIN));
	dsi_configure_video_mode(dsi,
				 PIXELS_TO_BYTES(display->xres,
						 display->bpp),
				 display->yres);
	dsi_set_phy(dsi, 1);
	dsicfg = DSI_CFG_TX_HS_PIXEL(display->dif.dsi.nblanes,
				     display->dif.dsi.mode);
	dsi_write_field(dsi, EXR_DSI_CFG, dsicfg);
}

void xgold_dsi_start_command(struct xgold_mipi_dsi *dsi)
{
	struct dsi_display *display = &dsi->display;

	dsi_write_field(dsi, EXR_DSI_CFG, DSI_CFG_OFF(DSI_CMD));
	dsi_write_field(dsi, EXR_DSI_IMSC,
			DSI_IRQ_ERR_MASK & (~DSI_IRQ_ERR_DSICON));
	dsi_set_phy(dsi, 1);
	dsi_write_field(dsi, EXR_DSI_CFG,
			DSI_CFG_INIT(display->dif.dsi.nblanes));
}

int xgold_dsi_stop(struct xgold_mipi_dsi *dsi)
{
	/* Reset and re-init for entering ULPS*/
	xgold_dsi_start_command(dsi);

	/* Enter ULPS */
	dsi_write_field(dsi, EXR_DSI_CFG_ULPS, 1);

	/* Swicth off PLL */
	dsi_set_phy(dsi, 0);

	/* Switch off phy */
	dsi_write_field(dsi, EXR_DSI_PHY3, BITFLDS(EXR_DSI_PHY3_EN, 0x0));

	return 0;
}

int xgold_dsi_init(struct xgold_mipi_dsi *dsi)
{
	unsigned int clcstat;

	if (dsi->dsi_reset) {
		reset_control_assert(dsi->dsi_reset);
		udelay(10);
		reset_control_deassert(dsi->dsi_reset);
		usleep_range(30000, 30001);
	}

	dsi_write_field(dsi, EXR_DSI_CLC,
			BITFLDS(EXR_DSI_CLC_RUN, DSI_MODE_RUN));
	clcstat = BITFLDS(EXR_DSI_CLC_STAT_RUN, 1) |
		BITFLDS(EXR_DSI_CLC_STAT_MODEN, 1) |
		BITFLDS(EXR_DSI_CLC_STAT_KID, 1);

	dsi_wait_status(dsi, EXR_DSI_CLC_STAT, clcstat, clcstat, 0, 1000);
	dsi_write_field(dsi, EXR_DSI_CLK, 0x000F000F);
	dsi_write_field(dsi, EXR_DSI_TO0, 0);
	dsi_write_field(dsi, EXR_DSI_TO1, 0);
	dsi_write_field(dsi, EXR_DSI_CFG, DSI_CFG_RX_LP_STP(1));
	dsi_interrupt_setup(dsi);

	return 0;
}

int xgold_dsi_pre_init(struct xgold_mipi_dsi *dsi)
{
	init_completion(&dsi->sync.dsifin);
	dsi->sync.dsifin_to = 200;
	dsi_irq_probe(dsi);

	return 0;
}
