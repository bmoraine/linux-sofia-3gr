/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:
 *      Wenlong Zhuang <daisen.zhuang@rock-chips.com>
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

#ifndef _ROCKCHIP_LVDS_
#define _ROCKCHIP_LVDS_

static inline u64 val_mask(int val, u64 msk, int shift)
{
	return (msk << (shift + 32)) | ((msk & val) << shift);
}

#define VAL_MASK(x, width, shift) val_mask(x, (1 << width) - 1, shift)

/* lvds register definition */
#define LVDS_REG_CTRL		0x00000000
#define V_LVDS_SELECT(x)		VAL_MASK(x, 2, 0)
#define V_LVDS_MSBSEL(x)		VAL_MASK(x, 1, 2)
#define V_LVDS_CLK_EDGE(x)		VAL_MASK(x, 1, 3)
#define V_LVDS_DATA_BITS(x)		VAL_MASK(x, 2, 4)
#define V_LVDS_OFFSET_VOLT(x)		VAL_MASK(x, 3, 6)
#define V_LVDS_SWING(x)			VAL_MASK(x, 3, 9)
#define V_LVDS_PRE_EMPHASIS(x)		VAL_MASK(x, 2, 12)
#define V_LVDS_CLK_DS1(x)		VAL_MASK(x, 3, 14)
#define V_LVDS_POWER_MODE(x)		VAL_MASK(x, 1, 17)
#define V_LVDS_TTL_MODE_EN(x)		VAL_MASK(x, 1, 18)

#define LVDS_REG_PLL_STA	0x00000004 /* LVDS PLL lock status */
#define M_LVDS_PLL_LOCK			BIT(0)
/* lvds register definition end */

/* lvds format definition */
#define LVDS_8BIT_1     0
#define LVDS_8BIT_2     1
#define LVDS_8BIT_3     2
#define LVDS_6BIT       3
/* lvds format definition end */

enum {
	LVDS_MSB_D0 = 0,
	LVDS_MSB_D7,
};

enum {
	LVDS_EDGE_FALL = 0,
	LVDS_EDGE_RISE,
};

enum {
	LVDS_6_BIT = 0,
	LVDS_8_BIT,
	LVDS_10_BIT,
};

enum {
	LVDS_OFFSET_800MV = 0,
	LVDS_OFFSET_650MV,
	LVDS_OFFSET_700MV,
	LVDS_OFFSET_750MV,
	LVDS_OFFSET_850MV,
	LVDS_OFFSET_900MV,
	LVDS_OFFSET_950MV,
	LVDS_OFFSET_1000MV,
};

enum {
	LVDS_SWING_200MV = 0,
	LVDS_SWING_150MV,
	LVDS_SWING_250MV,
	LVDS_SWING_300MV,
	LVDS_SWING_350MV,
	LVDS_SWING_400MV,
	LVDS_SWING_450MV,
	LVDS_SWING_500MV,
};

enum {
	LVDS_EMP_0DB = 0,	/* pre-emphasis */
	LVDS_EMP_2DB,
	LVDS_EMP_35DB,		/* 3.5db */
	LVDS_EMP_6DB,
};

enum {
	LVDS_SKEW_CLK_0PS = 0,	/* clk Behind */
	LVDS_SKEW_CLK_100PS,
	LVDS_SKEW_CLK_200PS,
	LVDS_SKEW_CLK_400PS,
	LVDS_SKEW_DATA_0PS,	/* data Behind */
	LVDS_SKEW_DATA_100PS,
	LVDS_SKEW_DATA_200PS,
	LVDS_SKEW_DATA_400PS,
};

#endif /* _ROCKCHIP_LVDS_ */
