/*
 * Register definition file for rockchip VOP controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _ROCKCHIP_VOP_H_
#define _ROCKCHIP_VOP_H_

#include<linux/rockchip_fb.h>
#include<linux/io.h>
#include<linux/clk.h>

#define BITS(x, bit)            ((x) << (bit))
#define BITS_MASK(x, mask, bit) BITS((x) & (mask), bit)

/*******************Register Definition**********************/

/* System control */
#define VOP_SYS_CTRL		(0x00)
#define M_WIN0_EN		BITS(1, 0)
#define M_WIN1_EN		BITS(1, 1)
#define M_HWC_EN		BITS(1, 2)
#define M_WIN0_FORMAT		BITS(7, 3)
#define M_WIN1_FORMAT		BITS(7, 6)
#define M_HWC_LUT_EN		BITS(1, 9)
#define M_HWC_SIZE		BITS(1, 10)
#define M_WIN0_RB_SWAP		BITS(1, 15)
#define M_WIN0_ALPHA_SWAP	BITS(1, 16)
#define M_WIN0_Y8_SWAP		BITS(1, 17)
#define M_WIN0_UV_SWAP		BITS(1, 18)
#define M_WIN1_RB_SWAP		BITS(1, 19)
#define M_WIN1_ALPHA_SWAP	BITS(1, 20)
#define M_WIN0_OTSD_DISABLE	BITS(1, 22)
#define M_WIN1_OTSD_DISABLE	BITS(1, 23)
#define M_DMA_BURST_LENGTH	BITS(3, 24)
#define M_HWC_LODAD_EN		BITS(1, 26)
#define M_DSP_LUT_EN		BITS(1, 28)
#define M_DMA_STOP		BITS(1, 29)
#define M_LCDC_STANDBY		BITS(1, 30)
#define M_AUTO_GATING_EN	BITS(1, 31)

#define V_WIN0_EN(x)		BITS_MASK(x, 1, 0)
#define V_WIN1_EN(x)		BITS_MASK(x, 1, 1)
#define V_HWC_EN(x)		BITS_MASK(x, 1, 2)
#define V_WIN0_FORMAT(x)	BITS_MASK(x, 7, 3)
#define V_WIN1_FORMAT(x)	BITS_MASK(x, 7, 6)
#define V_HWC_LUT_EN(x)		BITS_MASK(x, 1, 9)
#define V_HWC_SIZE(x)		BITS_MASK(x, 1, 10)
#define V_WIN0_RB_SWAP(x)	BITS_MASK(x, 1, 15)
#define V_WIN0_ALPHA_SWAP(x)	BITS_MASK(x, 1, 16)
#define V_WIN0_Y8_SWAP(x)	BITS_MASK(x, 1, 17)
#define V_WIN0_UV_SWAP(x)	BITS_MASK(x, 1, 18)
#define V_WIN1_RB_SWAP(x)	BITS_MASK(x, 1, 19)
#define V_WIN1_ALPHA_SWAP(x)	BITS_MASK(x, 1, 20)
#define V_WIN0_OTSD_DISABLE(x)	BITS_MASK(x, 1, 22)
#define V_WIN1_OTSD_DISABLE(x)	BITS_MASK(x, 1, 23)
#define V_DMA_BURST_LENGTH(x)	BITS_MASK(x, 3, 24)
#define V_HWC_LODAD_EN(x)	BITS_MASK(x, 1, 26)
#define V_DSP_LUT_EN(x)		BITS_MASK(x, 1, 28)
#define V_DMA_STOP(x)		BITS_MASK(x, 1, 29)
#define V_LCDC_STANDBY(x)	BITS_MASK(x, 1, 30)
#define V_AUTO_GATING_EN(x)	BITS_MASK(x, 1, 31)

/* Display control */
#define VOP_DSP_CTRL0		(0x04)
#define M_DSP_OUT_FORMAT	BITS(0x0f, 0)
#define M_HSYNC_POL		BITS(1, 4)
#define M_VSYNC_POL		BITS(1, 5)
#define M_DEN_POL		BITS(1, 6)
#define M_DCLK_POL		BITS(1, 7)
#define M_WIN0_TOP		BITS(1, 8)
#define M_DITHER_UP_EN		BITS(1, 9)
#define M_DITHER_DOWN_MODE	BITS(1, 10)
#define M_DITHER_DOWN_EN	BITS(1, 11)
#define M_INTERLACE_DSP_EN	BITS(1, 12)
#define M_INTERLACE_FIELD_POL	BITS(1, 13)
#define M_WIN0_INTERLACE_EN	BITS(1, 14)
#define M_WIN1_INTERLACE_EN	BITS(1, 15)
#define M_WIN0_YRGB_DEFLICK_EN	BITS(1, 16)
#define M_WIN0_CBR_DEFLICK_EN	BITS(1, 17)
#define M_WIN0_ALPHA_MODE	BITS(1, 18)
#define M_WIN1_ALPHA_MODE	BITS(1, 19)
#define M_WIN0_CSC_MODE		BITS(3, 20)
#define M_WIN1_CSC_MODE		BITS(1, 22)
#define M_WIN0_YUV_CLIP		BITS(1, 23)
#define M_DSP_CCIR656_AVG	BITS(1, 24)
#define M_DITHER_DOWN_SEL	BITS(1, 27)
#define M_HWC_ALPHA_MODE	BITS(1, 28)
#define M_ALPHA_MODE_SEL0	BITS(1, 29)
#define M_ALPHA_MODE_SEL1	BITS(1, 30)
#define M_SW_OVERLAY_MODE	BITS(1, 31)

#define V_DSP_OUT_FORMAT(x)	BITS_MASK(x, 0x0f, 0)
#define V_HSYNC_POL(x)		BITS_MASK(x, 1, 4)
#define V_VSYNC_POL(x)		BITS_MASK(x, 1, 5)
#define V_DEN_POL(x)		BITS_MASK(x, 1, 6)
#define V_DCLK_POL(x)		BITS_MASK(x, 1, 7)
#define V_WIN0_TOP(x)		BITS_MASK(x, 1, 8)
#define V_DITHER_UP_EN(x)	BITS_MASK(x, 1, 9)
#define V_DITHER_DOWN_MODE(x)	BITS_MASK(x, 1, 10)
#define V_DITHER_DOWN_EN(x)	BITS_MASK(x, 1, 11)
#define V_INTERLACE_DSP_EN(x)	BITS_MASK(x, 1, 12)
#define V_INTERLACE_FIELD_POL(x)	BITS_MASK(x, 1, 13)
#define V_WIN0_INTERLACE_EN(x)		BITS_MASK(x, 1, 14)
#define V_WIN1_INTERLACE_EN(x)		BITS_MASK(x, 1, 15)
#define V_WIN0_YRGB_DEFLICK_EN(x)	BITS_MASK(x, 1, 16)
#define V_WIN0_CBR_DEFLICK_EN(x)	BITS_MASK(x, 1, 17)
#define V_WIN0_ALPHA_MODE(x)	BITS_MASK(x, 1, 18)
#define V_WIN1_ALPHA_MODE(x)	BITS_MASK(x, 1, 19)
#define V_WIN0_CSC_MODE(x)	BITS_MASK(x, 3, 20)
#define V_WIN1_CSC_MODE(x)	BITS_MASK(x, 1, 22)
#define V_WIN0_YUV_CLIP(x)	BITS_MASK(x, 1, 23)
#define V_DSP_CCIR656_AVG(x)	BITS_MASK(x, 1, 24)
#define V_DITHER_DOWN_SEL(x)	BITS_MASK(x, 1, 27)
#define V_HWC_ALPHA_MODE(x)	BITS_MASK(x, 1, 28)
#define V_ALPHA_MODE_SEL0(x)	BITS_MASK(x, 1, 29)
#define V_ALPHA_MODE_SEL1(x)	BITS_MASK(x, 1, 30)
#define V_SW_OVERLAY_MODE(x)	BITS_MASK(x, 1, 31)

/* Display control and backgrounp setting */
#define VOP_DSP_CTRL1		(0x08)
#define M_BG_COLOR		BITS(0xffffff, 0)
#define M_BG_B			BITS(0xff, 0)
#define M_BG_G			BITS(0xff, 8)
#define M_BG_R			BITS(0xff, 16)
#define M_BLANK_EN		BITS(1, 24)
#define M_BLACK_EN		BITS(1, 25)
#define M_DSP_BG_SWAP		BITS(1, 26)
#define M_DSP_RB_SWAP		BITS(1, 27)
#define M_DSP_RG_SWAP		BITS(1, 28)
#define M_DSP_OUT_ZERO		BITS(1, 31)

#define V_BG_COLOR(x)		BITS_MASK(x, 0xffffff, 0)
#define V_BG_B(x)		BITS_MASK(x, 0xff, 0)
#define V_BG_G(x)		BITS_MASK(x, 0xff, 8)
#define V_BG_R(x)		BITS_MASK(x, 0xff, 16)
#define V_BLANK_EN(x)		BITS_MASK(x, 1, 24)
#define V_BLACK_EN(x)		BITS_MASK(x, 1, 25)
#define V_DSP_BG_SWAP(x)	BITS_MASK(x, 1, 26)
#define V_DSP_RB_SWAP(x)	BITS_MASK(x, 1, 27)
#define V_DSP_RG_SWAP(x)	BITS_MASK(x, 1, 28)
#define V_DSP_OUT_ZERO(x)	BITS_MASK(x, 1, 31)

/* frame start interrupt mask */
#define VOP_INT_MASK		(0x0c)
#define M_FS_MASK_EN		BITS(1, 3)
#define V_FS_MASK_EN(x)		BITS_MASK(x, 1, 3)

/* Interrupt control and status */
#define VOP_INT_STATUS		(0x10)
#define M_HS_INT_STA		BITS(1, 0)
#define M_FS_INT_STA		BITS(1, 1)
#define M_LF_INT_STA		BITS(1, 2)
#define M_BUS_ERR_INT_STA	BITS(1, 3)
#define M_HS_INT_EN		BITS(1, 4)
#define M_FS_INT_EN		BITS(1, 5)
#define M_LF_INT_EN		BITS(1, 6)
#define M_BUS_ERR_INT_EN	BITS(1, 7)
#define M_HS_INT_CLEAR		BITS(1, 8)
#define M_FS_INT_CLEAR		BITS(1, 9)
#define M_LF_INT_CLEAR		BITS(1, 10)
#define M_BUS_ERR_INT_CLEAR	BITS(1, 11)
#define M_LF_INT_NUM		BITS(0xfff, 12)
#define M_WIN0_EMPTY_INT_EN	BITS(1, 24)
#define M_WIN1_EMPTY_INT_EN	BITS(1, 25)
#define M_WIN0_EMPTY_INT_CLEAR	BITS(1, 26)
#define M_WIN1_EMPTY_INT_CLEAR	BITS(1, 27)
#define M_WIN0_EMPTY_INT_STA	BITS(1, 28)
#define M_WIN1_EMPTY_INT_STA	BITS(1, 29)
#define M_FS_RAW_STA		BITS(1, 30)
#define M_LF_RAW_STA		BITS(1, 31)

#define V_HS_INT_EN(x)		BITS_MASK(x, 1, 4)
#define V_FS_INT_EN(x)		BITS_MASK(x, 1, 5)
#define V_LF_INT_EN(x)		BITS_MASK(x, 1, 6)
#define V_BUS_ERR_INT_EN(x)	BITS_MASK(x, 1, 7)
#define V_HS_INT_CLEAR(x)	BITS_MASK(x, 1, 8)
#define V_FS_INT_CLEAR(x)	BITS_MASK(x, 1, 9)
#define V_LF_INT_CLEAR(x)	BITS_MASK(x, 1, 10)
#define V_BUS_ERR_INT_CLEAR(x)	BITS_MASK(x, 1, 11)
#define V_LF_INT_NUM(x)		BITS_MASK(x, 0xfff, 12)
#define V_WIN0_EMPTY_INT_EN(x)	BITS_MASK(x, 1, 24)
#define V_WIN1_EMPTY_INT_EN(x)	BITS_MASK(x, 1, 25)
#define V_WIN0_EMPTY_INT_CLEAR(x)	BITS_MASK(x, 1, 26)
#define V_WIN1_EMPTY_INT_CLEAR(x)	BITS_MASK(x, 1, 27)

/* Alpha Blending control */
#define VOP_ALPHA_CTRL		(0x14)
#define M_WIN0_ALPHA_EN		BITS(1, 0)
#define M_WIN1_ALPHA_EN		BITS(1, 1)
#define M_HWC_ALPAH_EN		BITS(1, 2)
#define M_WIN0_ALPHA_VAL	BITS(0xff, 4)
#define M_WIN1_ALPHA_VAL	BITS(0xff, 12)
#define M_HWC_ALPAH_VAL		BITS(0xff, 20)

#define V_WIN0_ALPHA_EN(x)	BITS_MASK(x, 1, 0)
#define V_WIN1_ALPHA_EN(x)	BITS_MASK(x, 1, 1)
#define V_HWC_ALPAH_EN(x)	BITS_MASK(x, 1, 2)
#define V_WIN0_ALPHA_VAL(x)	BITS_MASK(x, 0xff, 4)
#define V_WIN1_ALPHA_VAL(x)	BITS_MASK(x, 0xff, 12)
#define V_HWC_ALPAH_VAL(x)	BITS_MASK(x, 0xff, 20)

/* Color key setting */
#define VOP_WIN0_COLOR_KEY	(0x18)
#define VOP_WIN1_COLOR_KEY	(0x1c)
#define M_COLOR_KEY_VAL		BITS(0xffffff, 0)
#define M_COLOR_KEY_EN		BITS(1, 24)

#define V_COLOR_KEY_VAL(x)	BITS_MASK(x, 0xffffff, 0)
#define V_COLOR_KEY_EN(x)	BITS_MASK(x, 1, 24)

/*
 * Layer Control Registers
 */
/* frame buffer memory start address */
#define VOP_WIN0_YRGB_MST	(0x20)
#define VOP_WIN0_CBR_MST	(0x24)
#define VOP_WIN1_MST		(0x4c)
#define VOP_HWC_MST		(0x58)

/* virtual stride */
#define VOP_WIN1_VIR		(0x28)
#define VOP_WIN0_VIR		(0x30)
#define M_YRGB_VIR		BITS(0x1fff, 0)
#define M_CBBR_VIR		BITS(0x1fff, 16)
#define V_YRGB_VIR(x)		BITS_MASK(x, 0x1fff, 0)
#define V_CBBR_VIR(x)		BITS_MASK(x, 0x1fff, 16)

#define V_ARGB888_VIRWIDTH(x)	BITS_MASK(x, 0x1fff, 0)
#define V_RGB888_VIRWIDTH(x)	BITS_MASK(((x*3)>>2)+((x)%3), 0x1fff, 0)
#define V_RGB565_VIRWIDTH(x)	BITS_MASK(DIV_ROUND_UP(x, 2), 0x1fff, 0)
#define V_YUV_VIRWIDTH(x)	BITS_MASK(DIV_ROUND_UP(x, 4), 0x1fff, 0)
#define V_CBCR_VIR(x)		BITS_MASK(x, 0x1fff, 16)

/* active window width/height */
#define VOP_WIN0_ACT_INFO	(0x34)
#define VOP_WIN1_ACT_INFO	(0xb4)
#define M_ACT_WIDTH		BITS(0x1fff, 0)
#define M_ACT_HEIGHT		BITS(0x1fff, 16)
#define V_ACT_WIDTH(x)		BITS_MASK(x - 1, 0x1fff, 0)
#define V_ACT_HEIGHT(x)		BITS_MASK(x - 1, 0x1fff, 16)

/* display width/height on panel */
#define VOP_WIN0_DSP_INFO	(0x38)
#define VOP_WIN1_DSP_INFO	(0x50)
#define M_DSP_WIDTH		BITS(0x7ff, 0)
#define M_DSP_HEIGHT		BITS(0x7ff, 16)
#define V_DSP_WIDTH(x)		BITS_MASK(x - 1, 0x7ff, 0)
#define V_DSP_HEIGHT(x)		BITS_MASK(x - 1, 0x7ff, 16)

/* display start point on panel */
#define VOP_WIN0_DSP_ST		(0x3c)
#define VOP_WIN1_DSP_ST		(0x54)
#define VOP_HWC_DSP_ST		(0x5c)
#define M_DSP_STX		BITS(0xfff, 0)
#define M_DSP_STY		BITS(0xfff, 16)
#define V_DSP_STX(x)		BITS_MASK(x, 0xfff, 0)
#define V_DSP_STY(x)		BITS_MASK(x, 0xfff, 16)

/* scaling factor */
#define VOP_WIN0_SCL_FACTOR_YRGB	(0x40)
#define VOP_WIN0_SCL_FACTOR_CBR		(0x44)
#define M_X_SCL_FACTOR		BITS(0xffff, 0)
#define M_Y_SCL_FACTOR		BITS(0xffff, 16)
#define V_X_SCL_FACTOR(x)	BITS_MASK(x, 0xffff, 0)
#define V_Y_SCL_FACTOR(x)	BITS_MASK(x, 0xffff, 16)

/* scaling start point offset */
#define VOP_WIN0_SCL_OFFSET	(0x48)
#define M_X_SCL_OFFSET_YRGB(x)	BITS_MASK(x, 0xff, 0)
#define M_X_SCL_OFFSET_CBR(x)	BITS_MASK(x, 0xff, 8)
#define M_Y_SCL_OFFSET_YRGB(x)	BITS_MASK(x, 0xff, 0)
#define M_Y_SCL_OFFSET_CBR(x)	BITS_MASK(x, 0xff, 8)
#define V_X_SCL_OFFSET_YRGB(x)	BITS_MASK(x, 0xff, 0)
#define V_X_SCL_OFFSET_CBR(x)	BITS_MASK(x, 0xff, 8)
#define V_Y_SCL_OFFSET_YRGB(x)	BITS_MASK(x, 0xff, 0)
#define V_Y_SCL_OFFSET_CBR(x)	BITS_MASK(x, 0xff, 8)

/*
 * LUT Registers
 */
/* Access entry for HWC LUT memory (0x800 - 0xbff) */
#define VOP_HWC_LUT_ADDR	(0x0800)
/* Access entry for DSP LUT memory (0xc00 - 0xfff) */
#define VOP_DSP_LUT_ADDR	(0x0c00)

/*
 * Display timing Registers
 */
/* Panel scanning horizontal width and hsync pulse end point */
#define VOP_DSP_HTOTAL_HS_END	(0x6c)
#define V_HSYNC(x)		BITS_MASK(x, 0xfff, 0)	/* hsync pulse width */
#define V_HORPRD(x)		BITS_MASK(x, 0xfff, 16)	/* horizontal period */

/* Panel active horizontal scanning start point and end point */
#define VOP_DSP_HACT_ST_END	(0x70)
#define V_HAEP(x)		BITS_MASK(x, 0xfff, 0)
#define V_HASP(x)		BITS_MASK(x, 0xfff, 16)

/* Panel scanning vertical height and vsync pulse end point */
#define VOP_DSP_VTOTAL_VS_END	(0x74)
#define V_VSYNC(x)		BITS_MASK(x, 0xfff, 0)
#define V_VERPRD(x)		BITS_MASK(x, 0xfff, 16)

/* Panel active vertical scanning start point and end point */
#define VOP_DSP_VACT_ST_END	(0x78)
#define V_VAEP(x)		BITS_MASK(x, 0xfff, 0)
#define V_VASP(x)		BITS_MASK(x, 0xfff, 16)

/* Vertical scanning start point and vsync pulse
 * end point of even filed in interlace mode
 */
#define VOP_DSP_VS_ST_END_F1	(0x7c)
#define V_VSYNC_END_F1(x)	BITS_MASK(x, 0xfff, 0)
#define V_VSYNC_ST_F1(x)	BITS_MASK(x, 0xfff, 16)

/* Vertical scanning active start point and end
 * point of even filed in interlace mode
 */
#define VOP_DSP_VACT_ST_END_F1	(0x80)
#define V_VAEP_F1(x)		BITS_MASK(x, 0xfff, 0)
#define V_VASP_F1(x)		BITS_MASK(x, 0xfff, 16)

/*
 * BCSH Control Registers
 */
/* Brightness/contrast/saturation/hue adjustment control */
#define VOP_BCSH_CTRL		(0xd0)
#define M_BCSH_EN		BITS(1, 0)
#define M_BCSH_R2Y_CSC_MODE     BITS(1, 1)
#define M_BCSH_OUT_MODE		BITS(3, 2)
#define M_BCSH_Y2R_CSC_MODE     BITS(3, 4)
#define M_BCSH_Y2R_EN		BITS(1, 6)
#define M_BCSH_R2Y_EN		BITS(1, 7)

#define V_BCSH_EN(x)		BITS_MASK(x, 1, 0)
#define V_BCSH_R2Y_CSC_MODE(x)  BITS_MASK(x, 1, 1)
#define V_BCSH_OUT_MODE(x)	BITS_MASK(x, 3, 2)
#define V_BCSH_Y2R_CSC_MODE(x)	BITS_MASK(x, 3, 4)
#define V_BCSH_Y2R_EN(x)	BITS_MASK(x, 1, 6)
#define V_BCSH_R2Y_EN(x)	BITS_MASK(x, 1, 7)

/* BCSH color bar */
#define VOP_BCSH_COLOR_BAR	(0xd4)
#define M_BCSH_COLOR_BAR_Y      BITS(0xff, 0)
#define M_BCSH_COLOR_BAR_U	BITS(0xff, 8)
#define M_BCSH_COLOR_BAR_V	BITS(0xff, 16)

#define V_BCSH_COLOR_BAR_Y(x)	BITS_MASK(x, 0xff, 0)
#define V_BCSH_COLOR_BAR_U(x)   BITS_MASK(x, 0xff, 8)
#define V_BCSH_COLOR_BAR_V(x)   BITS_MASK(x, 0xff, 16)

/* Brightness/contrast/saturation adjustment */
#define VOP_BCSH_BCS		(0xd8)
#define M_BCSH_BRIGHTNESS	BITS(0x1f, 0)
#define M_BCSH_CONTRAST		BITS(0xff, 8)
#define M_BCSH_SAT_CON		BITS(0x1ff, 16)

#define V_BCSH_BRIGHTNESS(x)	BITS_MASK(x, 0x1f, 0)
#define V_BCSH_CONTRAST(x)	BITS_MASK(x, 0xff, 8)
#define V_BCSH_SAT_CON(x)       BITS_MASK(x, 0x1ff, 16)

/* Hue adjustment */
#define VOP_BCSH_H		(0xdc)
#define M_BCSH_SIN_HUE		BITS(0xff, 0)
#define M_BCSH_COS_HUE		BITS(0xff, 8)

#define V_BCSH_SIN_HUE(x)	BITS_MASK(x, 0xff, 0)
#define V_BCSH_COS_HUE(x)	BITS_MASK(x, 0xff, 8)

/*
 * FRC Dithering Setting Registers
 */
#define VOP_FRC_LOWER01_0	(0xe0)
#define VOP_FRC_LOWER01_1	(0xe4)
#define VOP_FRC_LOWER10_0	(0xe8)
#define VOP_FRC_LOWER10_1	(0xec)
#define VOP_FRC_LOWER11_0	(0xf0)
#define VOP_FRC_LOWER11_1	(0xf4)

/*
 * Bus Control Registers
 */
/* Bus interface control */
#define VOP_BUS_INTF_CTRL	(0x2c)
#define M_IO_PAD_CLK			BITS(1, 31)
#define M_MIPI_DCLK_INVERT              BITS(1, 29)
#define M_MIPI_DCLK_EN                  BITS(1, 28)
#define M_LVDS_DCLK_INVERT              BITS(1, 27)
#define M_LVDS_DCLK_EN                  BITS(1, 26)
#define M_RGB_DCLK_INVERT               BITS(1, 25)
#define M_RGB_DCLK_EN                   BITS(1, 24)
#define M_AXI_OUTSTANDING_MAX_NUM	BITS(0x1f, 12)
#define M_AXI_MAX_OUTSTANDING_EN	BITS(1, 11)
#define M_MMU_EN			BITS(1, 10)
#define M_NOC_HURRY_THRESHOLD		BITS(0xf, 6)
#define M_NOC_HURRY_VALUE		BITS(3, 4)
#define M_NOC_HURRY_EN			BITS(1, 3)
#define M_NOC_QOS_VALUE			BITS(3, 1)
#define M_NOC_QOS_EN			BITS(1, 0)

#define V_IO_PAD_CLK(x)			BITS_MASK(x, 1, 31)
#define V_CORE_CLK_DIV_EN(x)		BITS_MASK(x, 1, 30)
#define V_MIPI_DCLK_INVERT(x)           BITS_MASK(x, 1, 29)
#define V_MIPI_DCLK_EN(x)               BITS_MASK(x, 1, 28)
#define V_LVDS_DCLK_INVERT(x)           BITS_MASK(x, 1, 27)
#define V_LVDS_DCLK_EN(x)               BITS_MASK(x, 1, 26)
#define V_RGB_DCLK_INVERT(x)            BITS_MASK(x, 1, 25)
#define V_RGB_DCLK_EN(x)                BITS_MASK(x, 1, 24)
#define V_AXI_OUTSTANDING_MAX_NUM(x)	BITS_MASK(x, 0x1f, 12)
#define V_AXI_MAX_OUTSTANDING_EN(x)	BITS_MASK(x, 1, 11)
#define V_MMU_EN(x)			BITS_MASK(x, 1, 10)
#define V_NOC_HURRY_THRESHOLD(x)	BITS_MASK(x, 0xf, 6)
#define V_NOC_HURRY_VALUE(x)		BITS_MASK(x, 3, 4)
#define V_NOC_HURRY_EN(x)		BITS_MASK(x, 1, 3)
#define V_NOC_QOS_VALUE(x)		BITS_MASK(x, 3, 1)
#define V_NOC_QOS_EN(x)			BITS_MASK(x, 1, 0)

/* AXI read transfer gather setting */
#define VOP_DMA_GATHER		(0x84)
#define M_WIN1_AXI_GATHER_NUM		BITS(0xf, 12)
#define M_WIN0_CBCR_AXI_GATHER_NUM	BITS(0x7, 8)
#define M_WIN0_YRGB_AXI_GATHER_NUM	BITS(0xf, 4)
#define M_WIN1_AXI_GATHER_EN		BITS(1, 2)
#define M_WIN0_CBCR_AXI_GATHER_EN	BITS(1, 1)
#define M_WIN0_YRGB_AXI_GATHER_EN	BITS(1, 0)

#define V_WIN1_AXI_GATHER_NUM(x)	BITS_MASK(x, 0xf, 12)
#define V_WIN0_CBCR_AXI_GATHER_NUM(x)	BITS_MASK(x, 0x7, 8)
#define V_WIN0_YRGB_AXI_GATHER_NUM(x)	BITS_MASK(x, 0xf, 4)
#define V_WIN1_AXI_GATHER_EN(x)		BITS_MASK(x, 1, 2)
#define V_WIN0_CBCR_AXI_GATHER_EN(x)	BITS_MASK(x, 1, 1)
#define V_WIN0_YRGB_AXI_GATHER_EN(x)	BITS_MASK(x, 1, 0)

/* Register Config Done Flag */
#define VOP_REG_CFG_DONE	(0x90)

/*
 * Transmitter interface Control Registers
 */
/* eDPI interface control for MIPI_DSI */
#define VOP_MIPI_EDPI_CTRL	(0xf8)
#define M_EDPI_HALT_EN		BITS(1, 0)
#define M_EDPI_TEAR_EN		BITS(1, 1)
#define M_EDPI_CTRL_MODE	BITS(1, 2)
#define M_EDPI_FRM_ST		BITS(1, 3)
#define M_DSP_HOLD_STATUS	BITS(1, 4)

#define V_EDPI_HALT_EN(x)	BITS_MASK(x, 1, 0)
#define V_EDPI_TEAR_EN(x)	BITS_MASK(x, 1, 1)
#define V_EDPI_CTRL_MODE(x)	BITS_MASK(x, 1, 2)
#define V_EDPI_FRM_ST(x)	BITS_MASK(x, 1, 3)
#define V_DSP_HOLD_STATUS(x)	BITS_MASK(x, 1, 4)

/* LVDS control register */
#define VOP_LVDS_CTRL		(0x140)
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
#define VOP_LVDS_PLL_STA	(0x144)
#define M_LVDS_PLL_LOCK		BITS(1, 0)

/*
 * MMU Control Registers
 */
/* MMU current page Table address */
#define VOP_MMU_DTE_ADDR	(0x0300)
#define M_MMU_DTE_ADDR			BITS(0xffffffff, 0)
#define V_MMU_DTE_ADDR(x)		BITS_MASK(x, 0xffffffff, 0)

/* MMU status register */
#define VOP_MMU_STATUS		(0x0304)
#define M_PAGING_ENABLED		BITS(1, 0)
#define M_PAGE_FAULT_ACTIVE		BITS(1, 1)
#define M_STAIL_ACTIVE			BITS(1, 2)
#define M_MMU_IDLE			BITS(1, 3)
#define M_REPLAY_BUFFER_EMPTY		BITS(1, 4)
#define M_PAGE_FAULT_IS_WRITE		BITS(1, 5)
#define M_PAGE_FAULT_BUS_ID		BITS(0x1f, 6)

#define V_PAGING_ENABLED(x)		BITS_MASK(x, 1, 0)
#define V_PAGE_FAULT_ACTIVE(x)		BITS_MASK(x, 1, 1)
#define V_STAIL_ACTIVE(x)		BITS_MASK(x, 1, 2)
#define V_MMU_IDLE(x)			BITS_MASK(x, 1, 3)
#define V_REPLAY_BUFFER_EMPTY(x)	BITS_MASK(x, 1, 4)
#define V_PAGE_FAULT_IS_WRITE(x)	BITS_MASK(x, 1, 5)
#define V_PAGE_FAULT_BUS_ID(x)		BITS_MASK(x, 0x1f, 6)

/* MMU command register */
#define VOP_MMU_COMMAND		(0x0308)
#define M_MMU_CMD			BITS(0x7, 0)
#define V_MMU_CMD(x)			BITS_MASK(x, 0x7, 0)

/* MMU logical address of last page fault */
#define VOP_MMU_PAGE_FAULT_ADDR	(0x030c)
#define M_PAGE_FAULT_ADDR		BITS(0xffffffff, 0)
#define V_PAGE_FAULT_ADDR(x)		BITS_MASK(x, 0xffffffff, 0)

/* MMU Zap cache line register */
#define VOP_MMU_ZAP_ONE_LINE	(0x0310)
#define M_MMU_ZAP_ONE_LINE		BITS(0xffffffff, 0)
#define V_MMU_ZAP_ONE_LINE(x)		BITS_MASK(x, 0xffffffff, 0)

/* MMU raw interrupt status register */
#define VOP_MMU_INT_RAWSTAT	(0x0314)
#define M_PAGE_FAULT_RAWSTAT		BITS(1, 0)
#define M_READ_BUS_ERROR_RAWSTAT	BITS(1, 1)

#define V_PAGE_FAULT_RAWSTAT(x)		BITS(x, 1, 0)
#define V_READ_BUS_ERROR_RAWSTAT(x)	BITS(x, 1, 1)

#define VOP_MMU_INT_CLEAR	(0x0318)
#define M_PAGE_FAULT_CLEAR		BITS(1, 0)
#define M_READ_BUS_ERROR_CLEAR		BITS(1, 1)

#define V_PAGE_FAULT_CLEAR(x)		BITS(x, 1, 0)
#define V_READ_BUS_ERROR_CLEAR(x)	BITS(x, 1, 1)

#define VOP_MMU_INT_MASK	(0x031c)
#define M_PAGE_FAULT_MASK		BITS(1, 0)
#define M_READ_BUS_ERROR_MASK		BITS(1, 1)

#define V_PAGE_FAULT_MASK(x)		BITS(x, 1, 0)
#define V_READ_BUS_ERROR_MASK(x)	BITS(x, 1, 1)

#define VOP_MMU_INT_STATUS	(0x0320)
#define M_PAGE_FAULT_STATUS		BITS(1, 0)
#define M_READ_BUS_ERROR_STATUS		BITS(1, 1)

#define V_PAGE_FAULT_STATUS(x)		BITS(x, 1, 0)
#define V_READ_BUS_ERROR_STATUS(x)	BITS(x, 1, 1)

/* MMU auto gating */
#define VOP_MMU_AUTO_GATING	(0x0324)
#define M_MMU_AUTO_GATING		BITS(1, 0)
#define V_MMU_AUTO_GATING(x)		BITS(x, 1, 0)

enum _vop_dma_burst {
	DMA_BURST_16 = 0,
	DMA_BURST_8,
	DMA_BURST_4
};

enum _vop_format_e {
	VOP_FORMAT_ARGB888 = 0,
	VOP_FORMAT_RGB888,
	VOP_FORMAT_RGB565,
	VOP_FORMAT_YCBCR420 = 4,
	VOP_FORMAT_YCBCR422,
	VOP_FORMAT_YCBCR444
};

enum _vop_tV_mode {
	TV_NTSC,
	TV_PAL,
};

enum _vop_r2y_csc_mode {
	VOP_R2Y_CSC_BT601 = 0,
	VOP_R2Y_CSC_BT709
};

enum _vop_y2r_csc_mode {
	VOP_Y2R_CSC_MPEG = 0,
	VOP_Y2R_CSC_JPEG,
	VOP_Y2R_CSC_HD,
	VOP_Y2R_CSC_BYPASS
};

enum _vop_hwc_size {
	VOP_HWC_SIZE_32,
	VOP_HWC_SIZE_64
};

enum _vop_overlay_mode {
	VOP_RGB_DOMAIN,
	VOP_YUV_DOMAIN
};

#define CALSCALE(x, y)	             ((((u32)(x - 1)) * 0x1000) / (y - 1))
#define INT_STA_MSK	(M_HS_INT_STA | M_FS_INT_STA |		\
			 M_LF_INT_STA | M_BUS_ERR_INT_STA)
#define INT_CLR_SHIFT	8

struct vop_device {
	int id;
	struct device *dev;
	struct rockchip_vop_driver driver;
	struct rockchip_screen *screen;

	void __iomem *regs;
	void *regsbak;		/* back up reg */
	u32 reg_phy_base;	/* physical basic address of lcdc register */
	u32 len;		/* physical map length of lcdc register */
	spinlock_t reg_lock;	/* one time only one process allowed to
				 * config the register */
	int __iomem *hwc_lut_addr_base;
	int __iomem *dsp_lut_addr_base;

	int prop;		/* used for primary or
				 * extended display device */
	bool pre_init;
	bool pwr18;		/* if lcdc use 1.8v power supply */
	bool clk_on;		/* if vop clk on */
	u8 atv_layer_cnt;	/* active layer counter,
				 * when atv_layer_cnt = 0,vop is disable */
	int irq;

	struct clk *dclk;	/* vop dclk */
	u32 pixclock;

	u32 standby;		/* 1:standby,0:work */
	u32 iommu_status;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#endif
};

static inline void vop_writel(struct vop_device *vop_dev, u32 offset, u32 v)
{
	u32 *_pv = (u32 *)vop_dev->regsbak;

	_pv += (offset >> 2);
	*_pv = v;
	writel(v, (u32 __iomem *)(vop_dev->regs + offset));
}

static inline u32 vop_readl_bak(struct vop_device *vop_dev, u32 offset)
{
	u32 v;
	u32 *_pv = (u32 *)vop_dev->regsbak;

	_pv += (offset >> 2);
	v = readl((u32 __iomem *)(vop_dev->regs + offset));
	*_pv = v;
	return v;
}

static inline u32 vop_readl(struct vop_device *vop_dev, u32 offset)
{
	return readl((u32 __iomem *)(vop_dev->regs + offset));
}

static inline u32 vop_read_bit(struct vop_device *vop_dev, u32 offset, u32 msk)
{
	u32 _v = readl((u32 __iomem *)(vop_dev->regs + offset));
	u32 bit = 0;

	_v &= msk;
	bit = _v ? 1 : 0;
	return bit;
}

static inline void vop_set_bit(struct vop_device *vop_dev, u32 offset, u32 msk)
{
	u32 *_pv = (u32 *)vop_dev->regsbak;

	_pv += (offset >> 2);
	(*_pv) |= msk;
	writel(*_pv, (u32 __iomem *)(vop_dev->regs + offset));
}

static inline void vop_clr_bit(struct vop_device *vop_dev, u32 offset, u32 msk)
{
	u32 *_pv = (u32 *)vop_dev->regsbak;

	_pv += (offset >> 2);
	(*_pv) &= (~msk);
	writel(*_pv, (u32 __iomem *)(vop_dev->regs + offset));
}

static inline void vop_msk_reg(struct vop_device *vop_dev, u32 offset,
			       u32 msk, u32 v)
{
	u32 *_pv = (u32 *)vop_dev->regsbak;

	_pv += (offset >> 2);
	(*_pv) &= (~msk);
	(*_pv) |= v;
	writel(*_pv, (u32 __iomem *)(vop_dev->regs + offset));
}

static inline void vop_cfg_done(struct vop_device *vop_dev)
{
	writel(0x01, (u32 __iomem *)(vop_dev->regs + VOP_REG_CFG_DONE));
}

#endif	/* _ROCKCHIP_VOP_H_ */
