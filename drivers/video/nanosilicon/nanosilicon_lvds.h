#ifndef _NANOSILICON_LVDS_H_
#define _NANOSILICON_LVDS_H_

#include <linux/rockchip_screen.h>

#define BITS(x, bit)		((x) << (bit))
#define BITS_MASK(x, mask, bit)	BITS((x) & (mask), bit)
#define BITS_EN(mask, bit)	BITS(mask, bit + 16)

/* LVDS control register */
#define LVDS_REG_CTRL		(0x00)
#define M_LVDS_SELECT		BITS(3, 0)
#define M_LVDS_MSBSEL		BITS(1, 2)
#define M_LVDS_CLK_EDGE		BITS(1, 3)
#define M_LVDS_DATA_BITS	BITS(3, 4)
#define M_LVDS_OFFSET_VOLT	BITS(7, 6)
#define M_LVDS_SWING		BITS(7, 9)
#define M_LVDS_PRE_EMPHASIS	BITS(3, 12)
#define M_LVDS_CLK_DS1		BITS(7, 14)
#define M_LVDS_POWER_MODE	BITS(1, 17)
#define M_LVDS_TTL_MODE_EN	BITS(1, 18)

#define V_LVDS_SELECT(x)	BITS_MASK(x, 3, 0)
#define V_LVDS_MSBSEL(x)	BITS_MASK(x, 1, 2)
#define V_LVDS_CLK_EDGE(x)	BITS_MASK(x, 1, 3)
#define V_LVDS_DATA_BITS(x)	BITS_MASK(x, 3, 4)
#define V_LVDS_OFFSET_VOLT(x)	BITS_MASK(x, 7, 6)
#define V_LVDS_SWING(x)		BITS_MASK(x, 7, 9)
#define V_LVDS_PRE_EMPHASIS(x)	BITS_MASK(x, 3, 12)
#define V_LVDS_CLK_DS1(x)	BITS_MASK(x, 7, 14)
#define V_LVDS_POWER_MODE(x)	BITS_MASK(x, 1, 17)
#define V_LVDS_TTL_MODE_EN(x)	BITS_MASK(x, 1, 18)

/* LVDS PLL lock status */
#define LVDS_REG_PLL_STA	(0x04)
#define M_LVDS_PLL_LOCK		BITS(1, 0)

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

struct lvds_device {
	struct device *dev;
	void __iomem *regbase;
	struct rockchip_screen screen;
	bool sys_state;

#ifdef CONFIG_PINCTRL
	struct dev_pin_info *pins;
#endif
};

#endif
