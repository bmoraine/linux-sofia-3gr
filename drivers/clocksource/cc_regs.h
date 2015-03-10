/*
 * Copyright (c) 2015 Intel Mobile Communications GmbH
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CC_XGOLD_REGS_H
#define __CC_XGOLD_REGS_H
/*
This is a generated file - do not edit !
*/

#define CC_CLC_OFFSET   0x0
#define CC_CLC(_base) ((_base) + 0x0)
	#define CC_CLC_ORMC_OFFSET 0x10
	#define CC_CLC_ORMC_WIDTH 0x8
	#define CC_CLC_ORMC_MASK 0xff0000
	#define CC_CLC_ORMC(_reg) (((_reg) & 0xff0000) >> 0x10)
	#define CC_CLC_RMC_OFFSET 0x8
	#define CC_CLC_RMC_WIDTH 0x8
	#define CC_CLC_RMC_MASK 0xff00
	#define CC_CLC_RMC(_reg) (((_reg) & 0xff00) >> 0x8)
	#define CC_CLC_FSOE_OFFSET 0x5
	#define CC_CLC_FSOE_WIDTH 0x1
	#define CC_CLC_FSOE_MASK 0x20
	#define CC_CLC_FSOE(_reg) (((_reg) & 0x20) >> 0x5)
		#define CC_CLC_FSOE_DIS (0x0 << CC_CLC_FSOE_OFFSET)
		#define CC_CLC_FSOE_EN (0x1 << CC_CLC_FSOE_OFFSET)
	#define CC_CLC_SBWE_OFFSET 0x4
	#define CC_CLC_SBWE_WIDTH 0x1
	#define CC_CLC_SBWE_MASK 0x10
	#define CC_CLC_SBWE(_reg) (((_reg) & 0x10) >> 0x4)
		#define CC_CLC_SBWE_DIS (0x0 << CC_CLC_SBWE_OFFSET)
		#define CC_CLC_SBWE_EN (0x1 << CC_CLC_SBWE_OFFSET)
	#define CC_CLC_EDIS_OFFSET 0x3
	#define CC_CLC_EDIS_WIDTH 0x1
	#define CC_CLC_EDIS_MASK 0x8
	#define CC_CLC_EDIS(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_CLC_EDIS_EN (0x0 << CC_CLC_EDIS_OFFSET)
		#define CC_CLC_EDIS_DIS (0x1 << CC_CLC_EDIS_OFFSET)
	#define CC_CLC_SPEN_OFFSET 0x2
	#define CC_CLC_SPEN_WIDTH 0x1
	#define CC_CLC_SPEN_MASK 0x4
	#define CC_CLC_SPEN(_reg) (((_reg) & 0x4) >> 0x2)
		#define CC_CLC_SPEN_DIS (0x0 << CC_CLC_SPEN_OFFSET)
		#define CC_CLC_SPEN_EN (0x1 << CC_CLC_SPEN_OFFSET)
	#define CC_CLC_DISS_OFFSET 0x1
	#define CC_CLC_DISS_WIDTH 0x1
	#define CC_CLC_DISS_MASK 0x2
	#define CC_CLC_DISS(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_CLC_DISS_EN (0x0 << CC_CLC_DISS_OFFSET)
		#define CC_CLC_DISS_DIS (0x1 << CC_CLC_DISS_OFFSET)
	#define CC_CLC_DISR_OFFSET 0x0
	#define CC_CLC_DISR_WIDTH 0x1
	#define CC_CLC_DISR_MASK 0x1
	#define CC_CLC_DISR(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_CLC_DISR_NO_REQ (0x0 << CC_CLC_DISR_OFFSET)
		#define CC_CLC_DISR_REQ_DIS (0x1 << CC_CLC_DISR_OFFSET)

#define CC_ID_OFFSET   0x8
#define CC_ID(_base) ((_base) + 0x8)
	#define CC_ID_ID_OFFSET 0x8
	#define CC_ID_ID_WIDTH 0x8
	#define CC_ID_ID_MASK 0xff00
	#define CC_ID_ID(_reg) (((_reg) & 0xff00) >> 0x8)
		#define CC_ID_ID_CC_MODID (0x2 << CC_ID_ID_OFFSET)
	#define CC_ID_REV_OFFSET 0x0
	#define CC_ID_REV_WIDTH 0x8
	#define CC_ID_REV_MASK 0xff
	#define CC_ID_REV(_reg) (((_reg) & 0xff) >> 0x0)
		#define CC_ID_REV_CC_REV (0x1 << CC_ID_REV_OFFSET)

#define CC_PISEL_OFFSET   0x4
#define CC_PISEL(_base) ((_base) + 0x4)
	#define CC_PISEL_T1INIS_OFFSET 0x5
	#define CC_PISEL_T1INIS_WIDTH 0x1
	#define CC_PISEL_T1INIS_MASK 0x20
	#define CC_PISEL_T1INIS(_reg) (((_reg) & 0x20) >> 0x5)
		#define CC_PISEL_T1INIS_DEF (0x0 << CC_PISEL_T1INIS_OFFSET)
		#define CC_PISEL_T1INIS_ALT (0x1 << CC_PISEL_T1INIS_OFFSET)
	#define CC_PISEL_T0INIS_OFFSET 0x4
	#define CC_PISEL_T0INIS_WIDTH 0x1
	#define CC_PISEL_T0INIS_MASK 0x10
	#define CC_PISEL_T0INIS(_reg) (((_reg) & 0x10) >> 0x4)
		#define CC_PISEL_T0INIS_DEF (0x0 << CC_PISEL_T0INIS_OFFSET)
		#define CC_PISEL_T0INIS_ALT (0x1 << CC_PISEL_T0INIS_OFFSET)
	#define CC_PISEL_C7C6IS_OFFSET 0x3
	#define CC_PISEL_C7C6IS_WIDTH 0x1
	#define CC_PISEL_C7C6IS_MASK 0x8
	#define CC_PISEL_C7C6IS(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_PISEL_C7C6IS_DEF (0x0 << CC_PISEL_C7C6IS_OFFSET)
		#define CC_PISEL_C7C6IS_ALT (0x1 << CC_PISEL_C7C6IS_OFFSET)
	#define CC_PISEL_C5C4IS_OFFSET 0x2
	#define CC_PISEL_C5C4IS_WIDTH 0x1
	#define CC_PISEL_C5C4IS_MASK 0x4
	#define CC_PISEL_C5C4IS(_reg) (((_reg) & 0x4) >> 0x2)
		#define CC_PISEL_C5C4IS_DEF (0x0 << CC_PISEL_C5C4IS_OFFSET)
		#define CC_PISEL_C5C4IS_ALT (0x1 << CC_PISEL_C5C4IS_OFFSET)
	#define CC_PISEL_C3C2IS_OFFSET 0x1
	#define CC_PISEL_C3C2IS_WIDTH 0x1
	#define CC_PISEL_C3C2IS_MASK 0x2
	#define CC_PISEL_C3C2IS(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_PISEL_C3C2IS_DEF (0x0 << CC_PISEL_C3C2IS_OFFSET)
		#define CC_PISEL_C3C2IS_ALT (0x1 << CC_PISEL_C3C2IS_OFFSET)
	#define CC_PISEL_C1C0IS_OFFSET 0x0
	#define CC_PISEL_C1C0IS_WIDTH 0x1
	#define CC_PISEL_C1C0IS_MASK 0x1
	#define CC_PISEL_C1C0IS(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_PISEL_C1C0IS_DEF (0x0 << CC_PISEL_C1C0IS_OFFSET)
		#define CC_PISEL_C1C0IS_ALT (0x1 << CC_PISEL_C1C0IS_OFFSET)

#define CC_CCM0_OFFSET   0x14
#define CC_CCM0(_base) ((_base) + 0x14)
	#define CC_CCM0_ACC3_OFFSET 0xf
	#define CC_CCM0_ACC3_WIDTH 0x1
	#define CC_CCM0_ACC3_MASK 0x8000
	#define CC_CCM0_ACC3(_reg) (((_reg) & 0x8000) >> 0xf)
		#define CC_CCM0_ACC3_TIM0 (0x0 << CC_CCM0_ACC3_OFFSET)
		#define CC_CCM0_ACC3_TIM1 (0x1 << CC_CCM0_ACC3_OFFSET)
	#define CC_CCM0_MOD3_OFFSET 0xc
	#define CC_CCM0_MOD3_WIDTH 0x3
	#define CC_CCM0_MOD3_MASK 0x7000
	#define CC_CCM0_MOD3(_reg) (((_reg) & 0x7000) >> 0xc)
	#define CC_CCM0_ACC2_OFFSET 0xb
	#define CC_CCM0_ACC2_WIDTH 0x1
	#define CC_CCM0_ACC2_MASK 0x800
	#define CC_CCM0_ACC2(_reg) (((_reg) & 0x800) >> 0xb)
		#define CC_CCM0_ACC2_TIM0 (0x0 << CC_CCM0_ACC2_OFFSET)
		#define CC_CCM0_ACC2_TIM1 (0x1 << CC_CCM0_ACC2_OFFSET)
	#define CC_CCM0_MOD2_OFFSET 0x8
	#define CC_CCM0_MOD2_WIDTH 0x3
	#define CC_CCM0_MOD2_MASK 0x700
	#define CC_CCM0_MOD2(_reg) (((_reg) & 0x700) >> 0x8)
	#define CC_CCM0_ACC1_OFFSET 0x7
	#define CC_CCM0_ACC1_WIDTH 0x1
	#define CC_CCM0_ACC1_MASK 0x80
	#define CC_CCM0_ACC1(_reg) (((_reg) & 0x80) >> 0x7)
		#define CC_CCM0_ACC1_TIM0 (0x0 << CC_CCM0_ACC1_OFFSET)
		#define CC_CCM0_ACC1_TIM1 (0x1 << CC_CCM0_ACC1_OFFSET)
	#define CC_CCM0_MOD1_OFFSET 0x4
	#define CC_CCM0_MOD1_WIDTH 0x3
	#define CC_CCM0_MOD1_MASK 0x70
	#define CC_CCM0_MOD1(_reg) (((_reg) & 0x70) >> 0x4)
	#define CC_CCM0_ACC0_OFFSET 0x3
	#define CC_CCM0_ACC0_WIDTH 0x1
	#define CC_CCM0_ACC0_MASK 0x8
	#define CC_CCM0_ACC0(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_CCM0_ACC0_TIM0 (0x0 << CC_CCM0_ACC0_OFFSET)
		#define CC_CCM0_ACC0_TIM1 (0x1 << CC_CCM0_ACC0_OFFSET)
	#define CC_CCM0_MOD0_OFFSET 0x0
	#define CC_CCM0_MOD0_WIDTH 0x3
	#define CC_CCM0_MOD0_MASK 0x7
	#define CC_CCM0_MOD0(_reg) (((_reg) & 0x7) >> 0x0)

#define CC_CCM1_OFFSET   0x18
#define CC_CCM1(_base) ((_base) + 0x18)
	#define CC_CCM1_ACC7_OFFSET 0xf
	#define CC_CCM1_ACC7_WIDTH 0x1
	#define CC_CCM1_ACC7_MASK 0x8000
	#define CC_CCM1_ACC7(_reg) (((_reg) & 0x8000) >> 0xf)
		#define CC_CCM1_ACC7_TIM0 (0x0 << CC_CCM1_ACC7_OFFSET)
		#define CC_CCM1_ACC7_TIM1 (0x1 << CC_CCM1_ACC7_OFFSET)
	#define CC_CCM1_MOD7_OFFSET 0xc
	#define CC_CCM1_MOD7_WIDTH 0x3
	#define CC_CCM1_MOD7_MASK 0x7000
	#define CC_CCM1_MOD7(_reg) (((_reg) & 0x7000) >> 0xc)
	#define CC_CCM1_ACC6_OFFSET 0xb
	#define CC_CCM1_ACC6_WIDTH 0x1
	#define CC_CCM1_ACC6_MASK 0x800
	#define CC_CCM1_ACC6(_reg) (((_reg) & 0x800) >> 0xb)
		#define CC_CCM1_ACC6_TIM0 (0x0 << CC_CCM1_ACC6_OFFSET)
		#define CC_CCM1_ACC6_TIM1 (0x1 << CC_CCM1_ACC6_OFFSET)
	#define CC_CCM1_MOD6_OFFSET 0x8
	#define CC_CCM1_MOD6_WIDTH 0x3
	#define CC_CCM1_MOD6_MASK 0x700
	#define CC_CCM1_MOD6(_reg) (((_reg) & 0x700) >> 0x8)
	#define CC_CCM1_ACC5_OFFSET 0x7
	#define CC_CCM1_ACC5_WIDTH 0x1
	#define CC_CCM1_ACC5_MASK 0x80
	#define CC_CCM1_ACC5(_reg) (((_reg) & 0x80) >> 0x7)
		#define CC_CCM1_ACC5_TIM0 (0x0 << CC_CCM1_ACC5_OFFSET)
		#define CC_CCM1_ACC5_TIM1 (0x1 << CC_CCM1_ACC5_OFFSET)
	#define CC_CCM1_MOD5_OFFSET 0x4
	#define CC_CCM1_MOD5_WIDTH 0x3
	#define CC_CCM1_MOD5_MASK 0x70
	#define CC_CCM1_MOD5(_reg) (((_reg) & 0x70) >> 0x4)
	#define CC_CCM1_ACC4_OFFSET 0x3
	#define CC_CCM1_ACC4_WIDTH 0x1
	#define CC_CCM1_ACC4_MASK 0x8
	#define CC_CCM1_ACC4(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_CCM1_ACC4_TIM0 (0x0 << CC_CCM1_ACC4_OFFSET)
		#define CC_CCM1_ACC4_TIM1 (0x1 << CC_CCM1_ACC4_OFFSET)
	#define CC_CCM1_MOD4_OFFSET 0x0
	#define CC_CCM1_MOD4_WIDTH 0x3
	#define CC_CCM1_MOD4_MASK 0x7
	#define CC_CCM1_MOD4(_reg) (((_reg) & 0x7) >> 0x0)

#define CC_IOC_OFFSET   0x28
#define CC_IOC(_base) ((_base) + 0x28)
	#define CC_IOC_PDS_OFFSET 0x3
	#define CC_IOC_PDS_WIDTH 0x1
	#define CC_IOC_PDS_MASK 0x8
	#define CC_IOC_PDS(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_IOC_PDS_OUT (0x0 << CC_IOC_PDS_OFFSET)
		#define CC_IOC_PDS_IN (0x1 << CC_IOC_PDS_OFFSET)

#define CC_SEM_OFFSET   0x2c
#define CC_SEM(_base) ((_base) + 0x2c)
	#define CC_SEM_SEM7_OFFSET 0x7
	#define CC_SEM_SEM7_WIDTH 0x1
	#define CC_SEM_SEM7_MASK 0x80
	#define CC_SEM_SEM7(_reg) (((_reg) & 0x80) >> 0x7)
		#define CC_SEM_SEM7_DIS (0x0 << CC_SEM_SEM7_OFFSET)
		#define CC_SEM_SEM7_EN (0x1 << CC_SEM_SEM7_OFFSET)
	#define CC_SEM_SEM6_OFFSET 0x6
	#define CC_SEM_SEM6_WIDTH 0x1
	#define CC_SEM_SEM6_MASK 0x40
	#define CC_SEM_SEM6(_reg) (((_reg) & 0x40) >> 0x6)
		#define CC_SEM_SEM6_DIS (0x0 << CC_SEM_SEM6_OFFSET)
		#define CC_SEM_SEM6_EN (0x1 << CC_SEM_SEM6_OFFSET)
	#define CC_SEM_SEM5_OFFSET 0x5
	#define CC_SEM_SEM5_WIDTH 0x1
	#define CC_SEM_SEM5_MASK 0x20
	#define CC_SEM_SEM5(_reg) (((_reg) & 0x20) >> 0x5)
		#define CC_SEM_SEM5_DIS (0x0 << CC_SEM_SEM5_OFFSET)
		#define CC_SEM_SEM5_EN (0x1 << CC_SEM_SEM5_OFFSET)
	#define CC_SEM_SEM4_OFFSET 0x4
	#define CC_SEM_SEM4_WIDTH 0x1
	#define CC_SEM_SEM4_MASK 0x10
	#define CC_SEM_SEM4(_reg) (((_reg) & 0x10) >> 0x4)
		#define CC_SEM_SEM4_DIS (0x0 << CC_SEM_SEM4_OFFSET)
		#define CC_SEM_SEM4_EN (0x1 << CC_SEM_SEM4_OFFSET)
	#define CC_SEM_SEM3_OFFSET 0x3
	#define CC_SEM_SEM3_WIDTH 0x1
	#define CC_SEM_SEM3_MASK 0x8
	#define CC_SEM_SEM3(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_SEM_SEM3_DIS (0x0 << CC_SEM_SEM3_OFFSET)
		#define CC_SEM_SEM3_EN (0x1 << CC_SEM_SEM3_OFFSET)
	#define CC_SEM_SEM2_OFFSET 0x2
	#define CC_SEM_SEM2_WIDTH 0x1
	#define CC_SEM_SEM2_MASK 0x4
	#define CC_SEM_SEM2(_reg) (((_reg) & 0x4) >> 0x2)
		#define CC_SEM_SEM2_DIS (0x0 << CC_SEM_SEM2_OFFSET)
		#define CC_SEM_SEM2_EN (0x1 << CC_SEM_SEM2_OFFSET)
	#define CC_SEM_SEM1_OFFSET 0x1
	#define CC_SEM_SEM1_WIDTH 0x1
	#define CC_SEM_SEM1_MASK 0x2
	#define CC_SEM_SEM1(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_SEM_SEM1_DIS (0x0 << CC_SEM_SEM1_OFFSET)
		#define CC_SEM_SEM1_EN (0x1 << CC_SEM_SEM1_OFFSET)
	#define CC_SEM_SEM0_OFFSET 0x0
	#define CC_SEM_SEM0_WIDTH 0x1
	#define CC_SEM_SEM0_MASK 0x1
	#define CC_SEM_SEM0(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_SEM_SEM0_DIS (0x0 << CC_SEM_SEM0_OFFSET)
		#define CC_SEM_SEM0_EN (0x1 << CC_SEM_SEM0_OFFSET)

#define CC_SEE_OFFSET   0x30
#define CC_SEE(_base) ((_base) + 0x30)
	#define CC_SEE_SEE7_OFFSET 0x7
	#define CC_SEE_SEE7_WIDTH 0x1
	#define CC_SEE_SEE7_MASK 0x80
	#define CC_SEE_SEE7(_reg) (((_reg) & 0x80) >> 0x7)
		#define CC_SEE_SEE7_DIS (0x0 << CC_SEE_SEE7_OFFSET)
		#define CC_SEE_SEE7_EN (0x1 << CC_SEE_SEE7_OFFSET)
	#define CC_SEE_SEE6_OFFSET 0x6
	#define CC_SEE_SEE6_WIDTH 0x1
	#define CC_SEE_SEE6_MASK 0x40
	#define CC_SEE_SEE6(_reg) (((_reg) & 0x40) >> 0x6)
		#define CC_SEE_SEE6_DIS (0x0 << CC_SEE_SEE6_OFFSET)
		#define CC_SEE_SEE6_EN (0x1 << CC_SEE_SEE6_OFFSET)
	#define CC_SEE_SEE5_OFFSET 0x5
	#define CC_SEE_SEE5_WIDTH 0x1
	#define CC_SEE_SEE5_MASK 0x20
	#define CC_SEE_SEE5(_reg) (((_reg) & 0x20) >> 0x5)
		#define CC_SEE_SEE5_DIS (0x0 << CC_SEE_SEE5_OFFSET)
		#define CC_SEE_SEE5_EN (0x1 << CC_SEE_SEE5_OFFSET)
	#define CC_SEE_SEE4_OFFSET 0x4
	#define CC_SEE_SEE4_WIDTH 0x1
	#define CC_SEE_SEE4_MASK 0x10
	#define CC_SEE_SEE4(_reg) (((_reg) & 0x10) >> 0x4)
		#define CC_SEE_SEE4_DIS (0x0 << CC_SEE_SEE4_OFFSET)
		#define CC_SEE_SEE4_EN (0x1 << CC_SEE_SEE4_OFFSET)
	#define CC_SEE_SEE3_OFFSET 0x3
	#define CC_SEE_SEE3_WIDTH 0x1
	#define CC_SEE_SEE3_MASK 0x8
	#define CC_SEE_SEE3(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_SEE_SEE3_DIS (0x0 << CC_SEE_SEE3_OFFSET)
		#define CC_SEE_SEE3_EN (0x1 << CC_SEE_SEE3_OFFSET)
	#define CC_SEE_SEE2_OFFSET 0x2
	#define CC_SEE_SEE2_WIDTH 0x1
	#define CC_SEE_SEE2_MASK 0x4
	#define CC_SEE_SEE2(_reg) (((_reg) & 0x4) >> 0x2)
		#define CC_SEE_SEE2_DIS (0x0 << CC_SEE_SEE2_OFFSET)
		#define CC_SEE_SEE2_EN (0x1 << CC_SEE_SEE2_OFFSET)
	#define CC_SEE_SEE1_OFFSET 0x1
	#define CC_SEE_SEE1_WIDTH 0x1
	#define CC_SEE_SEE1_MASK 0x2
	#define CC_SEE_SEE1(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_SEE_SEE1_DIS (0x0 << CC_SEE_SEE1_OFFSET)
		#define CC_SEE_SEE1_EN (0x1 << CC_SEE_SEE1_OFFSET)
	#define CC_SEE_SEE0_OFFSET 0x0
	#define CC_SEE_SEE0_WIDTH 0x1
	#define CC_SEE_SEE0_MASK 0x1
	#define CC_SEE_SEE0(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_SEE_SEE0_DIS (0x0 << CC_SEE_SEE0_OFFSET)
		#define CC_SEE_SEE0_EN (0x1 << CC_SEE_SEE0_OFFSET)

#define CC_DRM_OFFSET   0x34
#define CC_DRM(_base) ((_base) + 0x34)
	#define CC_DRM_DR3M_OFFSET 0x6
	#define CC_DRM_DR3M_WIDTH 0x2
	#define CC_DRM_DR3M_MASK 0xc0
	#define CC_DRM_DR3M(_reg) (((_reg) & 0xc0) >> 0x6)
		#define CC_DRM_DR3M_CON (0x0 << CC_DRM_DR3M_OFFSET)
		#define CC_DRM_DR3M_DIS (0x1 << CC_DRM_DR3M_OFFSET)
		#define CC_DRM_DR3M_EN (0x2 << CC_DRM_DR3M_OFFSET)
		#define CC_DRM_DR3M_RES (0x3 << CC_DRM_DR3M_OFFSET)
	#define CC_DRM_DR2M_OFFSET 0x4
	#define CC_DRM_DR2M_WIDTH 0x2
	#define CC_DRM_DR2M_MASK 0x30
	#define CC_DRM_DR2M(_reg) (((_reg) & 0x30) >> 0x4)
		#define CC_DRM_DR2M_CON (0x0 << CC_DRM_DR2M_OFFSET)
		#define CC_DRM_DR2M_DIS (0x1 << CC_DRM_DR2M_OFFSET)
		#define CC_DRM_DR2M_EN (0x2 << CC_DRM_DR2M_OFFSET)
		#define CC_DRM_DR2M_RES (0x3 << CC_DRM_DR2M_OFFSET)
	#define CC_DRM_DR1M_OFFSET 0x2
	#define CC_DRM_DR1M_WIDTH 0x2
	#define CC_DRM_DR1M_MASK 0xc
	#define CC_DRM_DR1M(_reg) (((_reg) & 0xc) >> 0x2)
		#define CC_DRM_DR1M_CON (0x0 << CC_DRM_DR1M_OFFSET)
		#define CC_DRM_DR1M_DIS (0x1 << CC_DRM_DR1M_OFFSET)
		#define CC_DRM_DR1M_EN (0x2 << CC_DRM_DR1M_OFFSET)
		#define CC_DRM_DR1M_RES (0x3 << CC_DRM_DR1M_OFFSET)
	#define CC_DRM_DR0M_OFFSET 0x0
	#define CC_DRM_DR0M_WIDTH 0x2
	#define CC_DRM_DR0M_MASK 0x3
	#define CC_DRM_DR0M(_reg) (((_reg) & 0x3) >> 0x0)
		#define CC_DRM_DR0M_CON (0x0 << CC_DRM_DR0M_OFFSET)
		#define CC_DRM_DR0M_DIS (0x1 << CC_DRM_DR0M_OFFSET)
		#define CC_DRM_DR0M_EN (0x2 << CC_DRM_DR0M_OFFSET)
		#define CC_DRM_DR0M_RES (0x3 << CC_DRM_DR0M_OFFSET)

#define CC_WHBSSEE_OFFSET   0x38
#define CC_WHBSSEE(_base) ((_base) + 0x38)
	#define CC_WHBSSEE_SETSEE7_OFFSET 0x7
	#define CC_WHBSSEE_SETSEE7_WIDTH 0x1
	#define CC_WHBSSEE_SETSEE7_MASK 0x80
	#define CC_WHBSSEE_SETSEE7(_reg) (((_reg) & 0x80) >> 0x7)
		#define CC_WHBSSEE_SETSEE7_NOE (0x0 << CC_WHBSSEE_SETSEE7_OFFSET)
		#define CC_WHBSSEE_SETSEE7_SET (0x1 << CC_WHBSSEE_SETSEE7_OFFSET)
	#define CC_WHBSSEE_SETSEE6_OFFSET 0x6
	#define CC_WHBSSEE_SETSEE6_WIDTH 0x1
	#define CC_WHBSSEE_SETSEE6_MASK 0x40
	#define CC_WHBSSEE_SETSEE6(_reg) (((_reg) & 0x40) >> 0x6)
		#define CC_WHBSSEE_SETSEE6_NOE (0x0 << CC_WHBSSEE_SETSEE6_OFFSET)
		#define CC_WHBSSEE_SETSEE6_SET (0x1 << CC_WHBSSEE_SETSEE6_OFFSET)
	#define CC_WHBSSEE_SETSEE5_OFFSET 0x5
	#define CC_WHBSSEE_SETSEE5_WIDTH 0x1
	#define CC_WHBSSEE_SETSEE5_MASK 0x20
	#define CC_WHBSSEE_SETSEE5(_reg) (((_reg) & 0x20) >> 0x5)
		#define CC_WHBSSEE_SETSEE5_NOE (0x0 << CC_WHBSSEE_SETSEE5_OFFSET)
		#define CC_WHBSSEE_SETSEE5_SET (0x1 << CC_WHBSSEE_SETSEE5_OFFSET)
	#define CC_WHBSSEE_SETSEE4_OFFSET 0x4
	#define CC_WHBSSEE_SETSEE4_WIDTH 0x1
	#define CC_WHBSSEE_SETSEE4_MASK 0x10
	#define CC_WHBSSEE_SETSEE4(_reg) (((_reg) & 0x10) >> 0x4)
		#define CC_WHBSSEE_SETSEE4_NOE (0x0 << CC_WHBSSEE_SETSEE4_OFFSET)
		#define CC_WHBSSEE_SETSEE4_SET (0x1 << CC_WHBSSEE_SETSEE4_OFFSET)
	#define CC_WHBSSEE_SETSEE3_OFFSET 0x3
	#define CC_WHBSSEE_SETSEE3_WIDTH 0x1
	#define CC_WHBSSEE_SETSEE3_MASK 0x8
	#define CC_WHBSSEE_SETSEE3(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_WHBSSEE_SETSEE3_NOE (0x0 << CC_WHBSSEE_SETSEE3_OFFSET)
		#define CC_WHBSSEE_SETSEE3_SET (0x1 << CC_WHBSSEE_SETSEE3_OFFSET)
	#define CC_WHBSSEE_SETSEE2_OFFSET 0x2
	#define CC_WHBSSEE_SETSEE2_WIDTH 0x1
	#define CC_WHBSSEE_SETSEE2_MASK 0x4
	#define CC_WHBSSEE_SETSEE2(_reg) (((_reg) & 0x4) >> 0x2)
		#define CC_WHBSSEE_SETSEE2_NOE (0x0 << CC_WHBSSEE_SETSEE2_OFFSET)
		#define CC_WHBSSEE_SETSEE2_SET (0x1 << CC_WHBSSEE_SETSEE2_OFFSET)
	#define CC_WHBSSEE_SETSEE1_OFFSET 0x1
	#define CC_WHBSSEE_SETSEE1_WIDTH 0x1
	#define CC_WHBSSEE_SETSEE1_MASK 0x2
	#define CC_WHBSSEE_SETSEE1(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_WHBSSEE_SETSEE1_NOE (0x0 << CC_WHBSSEE_SETSEE1_OFFSET)
		#define CC_WHBSSEE_SETSEE1_SET (0x1 << CC_WHBSSEE_SETSEE1_OFFSET)
	#define CC_WHBSSEE_SETSEE0_OFFSET 0x0
	#define CC_WHBSSEE_SETSEE0_WIDTH 0x1
	#define CC_WHBSSEE_SETSEE0_MASK 0x1
	#define CC_WHBSSEE_SETSEE0(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_WHBSSEE_SETSEE0_NOE (0x0 << CC_WHBSSEE_SETSEE0_OFFSET)
		#define CC_WHBSSEE_SETSEE0_SET (0x1 << CC_WHBSSEE_SETSEE0_OFFSET)

#define CC_WHBCSEE_OFFSET   0x3c
#define CC_WHBCSEE(_base) ((_base) + 0x3c)
	#define CC_WHBCSEE_CLRSEE7_OFFSET 0x7
	#define CC_WHBCSEE_CLRSEE7_WIDTH 0x1
	#define CC_WHBCSEE_CLRSEE7_MASK 0x80
	#define CC_WHBCSEE_CLRSEE7(_reg) (((_reg) & 0x80) >> 0x7)
		#define CC_WHBCSEE_CLRSEE7_NOE (0x0 << CC_WHBCSEE_CLRSEE7_OFFSET)
		#define CC_WHBCSEE_CLRSEE7_CLR (0x1 << CC_WHBCSEE_CLRSEE7_OFFSET)
	#define CC_WHBCSEE_CLRSEE6_OFFSET 0x6
	#define CC_WHBCSEE_CLRSEE6_WIDTH 0x1
	#define CC_WHBCSEE_CLRSEE6_MASK 0x40
	#define CC_WHBCSEE_CLRSEE6(_reg) (((_reg) & 0x40) >> 0x6)
		#define CC_WHBCSEE_CLRSEE6_NOE (0x0 << CC_WHBCSEE_CLRSEE6_OFFSET)
		#define CC_WHBCSEE_CLRSEE6_CLR (0x1 << CC_WHBCSEE_CLRSEE6_OFFSET)
	#define CC_WHBCSEE_CLRSEE5_OFFSET 0x5
	#define CC_WHBCSEE_CLRSEE5_WIDTH 0x1
	#define CC_WHBCSEE_CLRSEE5_MASK 0x20
	#define CC_WHBCSEE_CLRSEE5(_reg) (((_reg) & 0x20) >> 0x5)
		#define CC_WHBCSEE_CLRSEE5_NOE (0x0 << CC_WHBCSEE_CLRSEE5_OFFSET)
		#define CC_WHBCSEE_CLRSEE5_CLR (0x1 << CC_WHBCSEE_CLRSEE5_OFFSET)
	#define CC_WHBCSEE_CLRSEE4_OFFSET 0x4
	#define CC_WHBCSEE_CLRSEE4_WIDTH 0x1
	#define CC_WHBCSEE_CLRSEE4_MASK 0x10
	#define CC_WHBCSEE_CLRSEE4(_reg) (((_reg) & 0x10) >> 0x4)
		#define CC_WHBCSEE_CLRSEE4_NOE (0x0 << CC_WHBCSEE_CLRSEE4_OFFSET)
		#define CC_WHBCSEE_CLRSEE4_CLR (0x1 << CC_WHBCSEE_CLRSEE4_OFFSET)
	#define CC_WHBCSEE_CLRSEE3_OFFSET 0x3
	#define CC_WHBCSEE_CLRSEE3_WIDTH 0x1
	#define CC_WHBCSEE_CLRSEE3_MASK 0x8
	#define CC_WHBCSEE_CLRSEE3(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_WHBCSEE_CLRSEE3_NOE (0x0 << CC_WHBCSEE_CLRSEE3_OFFSET)
		#define CC_WHBCSEE_CLRSEE3_CLR (0x1 << CC_WHBCSEE_CLRSEE3_OFFSET)
	#define CC_WHBCSEE_CLRSEE2_OFFSET 0x2
	#define CC_WHBCSEE_CLRSEE2_WIDTH 0x1
	#define CC_WHBCSEE_CLRSEE2_MASK 0x4
	#define CC_WHBCSEE_CLRSEE2(_reg) (((_reg) & 0x4) >> 0x2)
		#define CC_WHBCSEE_CLRSEE2_NOE (0x0 << CC_WHBCSEE_CLRSEE2_OFFSET)
		#define CC_WHBCSEE_CLRSEE2_CLR (0x1 << CC_WHBCSEE_CLRSEE2_OFFSET)
	#define CC_WHBCSEE_CLRSEE1_OFFSET 0x1
	#define CC_WHBCSEE_CLRSEE1_WIDTH 0x1
	#define CC_WHBCSEE_CLRSEE1_MASK 0x2
	#define CC_WHBCSEE_CLRSEE1(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_WHBCSEE_CLRSEE1_NOE (0x0 << CC_WHBCSEE_CLRSEE1_OFFSET)
		#define CC_WHBCSEE_CLRSEE1_CLR (0x1 << CC_WHBCSEE_CLRSEE1_OFFSET)
	#define CC_WHBCSEE_CLRSEE0_OFFSET 0x0
	#define CC_WHBCSEE_CLRSEE0_WIDTH 0x1
	#define CC_WHBCSEE_CLRSEE0_MASK 0x1
	#define CC_WHBCSEE_CLRSEE0(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_WHBCSEE_CLRSEE0_NOE (0x0 << CC_WHBCSEE_CLRSEE0_OFFSET)
		#define CC_WHBCSEE_CLRSEE0_CLR (0x1 << CC_WHBCSEE_CLRSEE0_OFFSET)

#define CC_T01CON_OFFSET   0x10
#define CC_T01CON(_base) ((_base) + 0x10)
	#define CC_T01CON_T1R_OFFSET 0xe
	#define CC_T01CON_T1R_WIDTH 0x1
	#define CC_T01CON_T1R_MASK 0x4000
	#define CC_T01CON_T1R(_reg) (((_reg) & 0x4000) >> 0xe)
		#define CC_T01CON_T1R_DIS (0x0 << CC_T01CON_T1R_OFFSET)
		#define CC_T01CON_T1R_EN (0x1 << CC_T01CON_T1R_OFFSET)
	#define CC_T01CON_T1M_OFFSET 0xb
	#define CC_T01CON_T1M_WIDTH 0x1
	#define CC_T01CON_T1M_MASK 0x800
	#define CC_T01CON_T1M(_reg) (((_reg) & 0x800) >> 0xb)
		#define CC_T01CON_T1M_TM (0x0 << CC_T01CON_T1M_OFFSET)
		#define CC_T01CON_T1M_CM (0x1 << CC_T01CON_T1M_OFFSET)
	#define CC_T01CON_T1I_OFFSET 0x8
	#define CC_T01CON_T1I_WIDTH 0x3
	#define CC_T01CON_T1I_MASK 0x700
	#define CC_T01CON_T1I(_reg) (((_reg) & 0x700) >> 0x8)
	#define CC_T01CON_T0R_OFFSET 0x6
	#define CC_T01CON_T0R_WIDTH 0x1
	#define CC_T01CON_T0R_MASK 0x40
	#define CC_T01CON_T0R(_reg) (((_reg) & 0x40) >> 0x6)
		#define CC_T01CON_T0R_DIS (0x0 << CC_T01CON_T0R_OFFSET)
		#define CC_T01CON_T0R_EN (0x1 << CC_T01CON_T0R_OFFSET)
	#define CC_T01CON_T0M_OFFSET 0x3
	#define CC_T01CON_T0M_WIDTH 0x1
	#define CC_T01CON_T0M_MASK 0x8
	#define CC_T01CON_T0M(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_T01CON_T0M_TM (0x0 << CC_T01CON_T0M_OFFSET)
		#define CC_T01CON_T0M_CM (0x1 << CC_T01CON_T0M_OFFSET)
	#define CC_T01CON_T0I_OFFSET 0x0
	#define CC_T01CON_T0I_WIDTH 0x3
	#define CC_T01CON_T0I_MASK 0x7
	#define CC_T01CON_T0I(_reg) (((_reg) & 0x7) >> 0x0)

#define CC_T0_OFFSET   0x40
#define CC_T0(_base) ((_base) + 0x40)
	#define CC_T0_OVF0_OFFSET 0x1f
	#define CC_T0_OVF0_WIDTH 0x1
	#define CC_T0_OVF0_MASK 0x80000000
	#define CC_T0_OVF0(_reg) (((_reg) & 0x80000000) >> 0x1f)
		#define CC_T0_OVF0_CLEARED (0x0 << CC_T0_OVF0_OFFSET)
		#define CC_T0_OVF0_SET (0x1 << CC_T0_OVF0_OFFSET)
	#define CC_T0_T0_OFFSET 0x0
	#define CC_T0_T0_WIDTH 0x1f
	#define CC_T0_T0_MASK 0x7fffffff
	#define CC_T0_T0(_reg) (((_reg) & 0x7fffffff) >> 0x0)

#define CC_T0REL_OFFSET   0x44
#define CC_T0REL(_base) ((_base) + 0x44)
	#define CC_T0REL_T0REL_OFFSET 0x0
	#define CC_T0REL_T0REL_WIDTH 0x1f
	#define CC_T0REL_T0REL_MASK 0x7fffffff
	#define CC_T0REL_T0REL(_reg) (((_reg) & 0x7fffffff) >> 0x0)

#define CC_T1_OFFSET   0x48
#define CC_T1(_base) ((_base) + 0x48)
	#define CC_T1_OVF1_OFFSET 0x1f
	#define CC_T1_OVF1_WIDTH 0x1
	#define CC_T1_OVF1_MASK 0x80000000
	#define CC_T1_OVF1(_reg) (((_reg) & 0x80000000) >> 0x1f)
		#define CC_T1_OVF1_CLEARED (0x0 << CC_T1_OVF1_OFFSET)
		#define CC_T1_OVF1_SET (0x1 << CC_T1_OVF1_OFFSET)
	#define CC_T1_T1_OFFSET 0x0
	#define CC_T1_T1_WIDTH 0x1f
	#define CC_T1_T1_MASK 0x7fffffff
	#define CC_T1_T1(_reg) (((_reg) & 0x7fffffff) >> 0x0)

#define CC_T1REL_OFFSET   0x4c
#define CC_T1REL(_base) ((_base) + 0x4c)
	#define CC_T1REL_T1REL_OFFSET 0x0
	#define CC_T1REL_T1REL_WIDTH 0x1f
	#define CC_T1REL_T1REL_MASK 0x7fffffff
	#define CC_T1REL_T1REL(_reg) (((_reg) & 0x7fffffff) >> 0x0)

#define CC_T01OCR_OFFSET   0x94
#define CC_T01OCR(_base) ((_base) + 0x94)
	#define CC_T01OCR_CT1_OFFSET 0x1
	#define CC_T01OCR_CT1_WIDTH 0x1
	#define CC_T01OCR_CT1_MASK 0x2
	#define CC_T01OCR_CT1(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_T01OCR_CT1_NOC (0x0 << CC_T01OCR_CT1_OFFSET)
		#define CC_T01OCR_CT1_CSR (0x1 << CC_T01OCR_CT1_OFFSET)
	#define CC_T01OCR_CT0_OFFSET 0x0
	#define CC_T01OCR_CT0_WIDTH 0x1
	#define CC_T01OCR_CT0_MASK 0x1
	#define CC_T01OCR_CT0(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_T01OCR_CT0_NOC (0x0 << CC_T01OCR_CT0_OFFSET)
		#define CC_T01OCR_CT0_CSR (0x1 << CC_T01OCR_CT0_OFFSET)

#define CC_OUT_OFFSET   0x24
#define CC_OUT(_base) ((_base) + 0x24)
	#define CC_OUT_O7_OFFSET 0x7
	#define CC_OUT_O7_WIDTH 0x1
	#define CC_OUT_O7_MASK 0x80
	#define CC_OUT_O7(_reg) (((_reg) & 0x80) >> 0x7)
	#define CC_OUT_O6_OFFSET 0x6
	#define CC_OUT_O6_WIDTH 0x1
	#define CC_OUT_O6_MASK 0x40
	#define CC_OUT_O6(_reg) (((_reg) & 0x40) >> 0x6)
	#define CC_OUT_O5_OFFSET 0x5
	#define CC_OUT_O5_WIDTH 0x1
	#define CC_OUT_O5_MASK 0x20
	#define CC_OUT_O5(_reg) (((_reg) & 0x20) >> 0x5)
	#define CC_OUT_O4_OFFSET 0x4
	#define CC_OUT_O4_WIDTH 0x1
	#define CC_OUT_O4_MASK 0x10
	#define CC_OUT_O4(_reg) (((_reg) & 0x10) >> 0x4)
	#define CC_OUT_O3_OFFSET 0x3
	#define CC_OUT_O3_WIDTH 0x1
	#define CC_OUT_O3_MASK 0x8
	#define CC_OUT_O3(_reg) (((_reg) & 0x8) >> 0x3)
	#define CC_OUT_O2_OFFSET 0x2
	#define CC_OUT_O2_WIDTH 0x1
	#define CC_OUT_O2_MASK 0x4
	#define CC_OUT_O2(_reg) (((_reg) & 0x4) >> 0x2)
	#define CC_OUT_O1_OFFSET 0x1
	#define CC_OUT_O1_WIDTH 0x1
	#define CC_OUT_O1_MASK 0x2
	#define CC_OUT_O1(_reg) (((_reg) & 0x2) >> 0x1)
	#define CC_OUT_O0_OFFSET 0x0
	#define CC_OUT_O0_WIDTH 0x1
	#define CC_OUT_O0_MASK 0x1
	#define CC_OUT_O0(_reg) (((_reg) & 0x1) >> 0x0)

#define CC_RCC0_OFFSET   0x50
#define CC_RCC0(_base) ((_base) + 0x50)
	#define CC_RCC0_CC0_OFFSET 0x0
	#define CC_RCC0_CC0_WIDTH 0x1f
	#define CC_RCC0_CC0_MASK 0x7fffffff
	#define CC_RCC0_CC0(_reg) (((_reg) & 0x7fffffff) >> 0x0)

#define CC_WHBSOUT_OFFSET   0x98
#define CC_WHBSOUT(_base) ((_base) + 0x98)
	#define CC_WHBSOUT_SET7O_OFFSET 0x7
	#define CC_WHBSOUT_SET7O_WIDTH 0x1
	#define CC_WHBSOUT_SET7O_MASK 0x80
	#define CC_WHBSOUT_SET7O(_reg) (((_reg) & 0x80) >> 0x7)
	#define CC_WHBSOUT_SET6O_OFFSET 0x6
	#define CC_WHBSOUT_SET6O_WIDTH 0x1
	#define CC_WHBSOUT_SET6O_MASK 0x40
	#define CC_WHBSOUT_SET6O(_reg) (((_reg) & 0x40) >> 0x6)
	#define CC_WHBSOUT_SET5O_OFFSET 0x5
	#define CC_WHBSOUT_SET5O_WIDTH 0x1
	#define CC_WHBSOUT_SET5O_MASK 0x20
	#define CC_WHBSOUT_SET5O(_reg) (((_reg) & 0x20) >> 0x5)
	#define CC_WHBSOUT_SET4O_OFFSET 0x4
	#define CC_WHBSOUT_SET4O_WIDTH 0x1
	#define CC_WHBSOUT_SET4O_MASK 0x10
	#define CC_WHBSOUT_SET4O(_reg) (((_reg) & 0x10) >> 0x4)
	#define CC_WHBSOUT_SET3O_OFFSET 0x3
	#define CC_WHBSOUT_SET3O_WIDTH 0x1
	#define CC_WHBSOUT_SET3O_MASK 0x8
	#define CC_WHBSOUT_SET3O(_reg) (((_reg) & 0x8) >> 0x3)
	#define CC_WHBSOUT_SET2O_OFFSET 0x2
	#define CC_WHBSOUT_SET2O_WIDTH 0x1
	#define CC_WHBSOUT_SET2O_MASK 0x4
	#define CC_WHBSOUT_SET2O(_reg) (((_reg) & 0x4) >> 0x2)
	#define CC_WHBSOUT_SET1O_OFFSET 0x1
	#define CC_WHBSOUT_SET1O_WIDTH 0x1
	#define CC_WHBSOUT_SET1O_MASK 0x2
	#define CC_WHBSOUT_SET1O(_reg) (((_reg) & 0x2) >> 0x1)
	#define CC_WHBSOUT_SET0O_OFFSET 0x0
	#define CC_WHBSOUT_SET0O_WIDTH 0x1
	#define CC_WHBSOUT_SET0O_MASK 0x1
	#define CC_WHBSOUT_SET0O(_reg) (((_reg) & 0x1) >> 0x0)

#define CC_WHBCOUT_OFFSET 0x9c
#define CC_WHBCOUT(_base) ((_base) + 0x9c)
	#define CC_WHBCOUT_CLR7O_OFFSET 0x7
	#define CC_WHBCOUT_CLR7O_WIDTH 0x1
	#define CC_WHBCOUT_CLR7O_MASK 0x80
	#define CC_WHBCOUT_CLR7O(_reg) (((_reg) & 0x80) >> 0x7)
	#define CC_WHBCOUT_CLR6O_OFFSET 0x6
	#define CC_WHBCOUT_CLR6O_WIDTH 0x1
	#define CC_WHBCOUT_CLR6O_MASK 0x40
	#define CC_WHBCOUT_CLR6O(_reg) (((_reg) & 0x40) >> 0x6)
	#define CC_WHBCOUT_CLR5O_OFFSET 0x5
	#define CC_WHBCOUT_CLR5O_WIDTH 0x1
	#define CC_WHBCOUT_CLR5O_MASK 0x20
	#define CC_WHBCOUT_CLR5O(_reg) (((_reg) & 0x20) >> 0x5)
	#define CC_WHBCOUT_CLR4O_OFFSET 0x4
	#define CC_WHBCOUT_CLR4O_WIDTH 0x1
	#define CC_WHBCOUT_CLR4O_MASK 0x10
	#define CC_WHBCOUT_CLR4O(_reg) (((_reg) & 0x10) >> 0x4)
	#define CC_WHBCOUT_CLR3O_OFFSET 0x3
	#define CC_WHBCOUT_CLR3O_WIDTH 0x1
	#define CC_WHBCOUT_CLR3O_MASK 0x8
	#define CC_WHBCOUT_CLR3O(_reg) (((_reg) & 0x8) >> 0x3)
	#define CC_WHBCOUT_CLR2O_OFFSET 0x2
	#define CC_WHBCOUT_CLR2O_WIDTH 0x1
	#define CC_WHBCOUT_CLR2O_MASK 0x4
	#define CC_WHBCOUT_CLR2O(_reg) (((_reg) & 0x4) >> 0x2)
	#define CC_WHBCOUT_CLR1O_OFFSET 0x1
	#define CC_WHBCOUT_CLR1O_WIDTH 0x1
	#define CC_WHBCOUT_CLR1O_MASK 0x2
	#define CC_WHBCOUT_CLR1O(_reg) (((_reg) & 0x2) >> 0x1)
	#define CC_WHBCOUT_CLR0O_OFFSET 0x0
	#define CC_WHBCOUT_CLR0O_WIDTH 0x1
	#define CC_WHBCOUT_CLR0O_MASK 0x1
	#define CC_WHBCOUT_CLR0O(_reg) (((_reg) & 0x1) >> 0x0)

#define CC_RIS_OFFSET   0x80
#define CC_RIS(_base) ((_base) + 0x80)
	#define CC_RIS_T1_OFFSET 0x9
	#define CC_RIS_T1_WIDTH 0x1
	#define CC_RIS_T1_MASK 0x200
	#define CC_RIS_T1(_reg) (((_reg) & 0x200) >> 0x9)
		#define CC_RIS_T1_NI (0x0 << CC_RIS_T1_OFFSET)
		#define CC_RIS_T1_IP (0x1 << CC_RIS_T1_OFFSET)
	#define CC_RIS_T0_OFFSET 0x8
	#define CC_RIS_T0_WIDTH 0x1
	#define CC_RIS_T0_MASK 0x100
	#define CC_RIS_T0(_reg) (((_reg) & 0x100) >> 0x8)
		#define CC_RIS_T0_NI (0x0 << CC_RIS_T0_OFFSET)
		#define CC_RIS_T0_IP (0x1 << CC_RIS_T0_OFFSET)
	#define CC_RIS_C7_OFFSET 0x7
	#define CC_RIS_C7_WIDTH 0x1
	#define CC_RIS_C7_MASK 0x80
	#define CC_RIS_C7(_reg) (((_reg) & 0x80) >> 0x7)
		#define CC_RIS_C7_NI (0x0 << CC_RIS_C7_OFFSET)
		#define CC_RIS_C7_IP (0x1 << CC_RIS_C7_OFFSET)
	#define CC_RIS_C6_OFFSET 0x6
	#define CC_RIS_C6_WIDTH 0x1
	#define CC_RIS_C6_MASK 0x40
	#define CC_RIS_C6(_reg) (((_reg) & 0x40) >> 0x6)
		#define CC_RIS_C6_NI (0x0 << CC_RIS_C6_OFFSET)
		#define CC_RIS_C6_IP (0x1 << CC_RIS_C6_OFFSET)
	#define CC_RIS_C5_OFFSET 0x5
	#define CC_RIS_C5_WIDTH 0x1
	#define CC_RIS_C5_MASK 0x20
	#define CC_RIS_C5(_reg) (((_reg) & 0x20) >> 0x5)
		#define CC_RIS_C5_NI (0x0 << CC_RIS_C5_OFFSET)
		#define CC_RIS_C5_IP (0x1 << CC_RIS_C5_OFFSET)
	#define CC_RIS_C4_OFFSET 0x4
	#define CC_RIS_C4_WIDTH 0x1
	#define CC_RIS_C4_MASK 0x10
	#define CC_RIS_C4(_reg) (((_reg) & 0x10) >> 0x4)
		#define CC_RIS_C4_NI (0x0 << CC_RIS_C4_OFFSET)
		#define CC_RIS_C4_IP (0x1 << CC_RIS_C4_OFFSET)
	#define CC_RIS_C3_OFFSET 0x3
	#define CC_RIS_C3_WIDTH 0x1
	#define CC_RIS_C3_MASK 0x8
	#define CC_RIS_C3(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_RIS_C3_NI (0x0 << CC_RIS_C3_OFFSET)
		#define CC_RIS_C3_IP (0x1 << CC_RIS_C3_OFFSET)
	#define CC_RIS_C2_OFFSET 0x2
	#define CC_RIS_C2_WIDTH 0x1
	#define CC_RIS_C2_MASK 0x4
	#define CC_RIS_C2(_reg) (((_reg) & 0x4) >> 0x2)
		#define CC_RIS_C2_NI (0x0 << CC_RIS_C2_OFFSET)
		#define CC_RIS_C2_IP (0x1 << CC_RIS_C2_OFFSET)
	#define CC_RIS_C1_OFFSET 0x1
	#define CC_RIS_C1_WIDTH 0x1
	#define CC_RIS_C1_MASK 0x2
	#define CC_RIS_C1(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_RIS_C1_NI (0x0 << CC_RIS_C1_OFFSET)
		#define CC_RIS_C1_IP (0x1 << CC_RIS_C1_OFFSET)
	#define CC_RIS_C0_OFFSET 0x0
	#define CC_RIS_C0_WIDTH 0x1
	#define CC_RIS_C0_MASK 0x1
	#define CC_RIS_C0(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_RIS_C0_NI (0x0 << CC_RIS_C0_OFFSET)
		#define CC_RIS_C0_IP (0x1 << CC_RIS_C0_OFFSET)

#define CC_IMSC_OFFSET   0x84
#define CC_IMSC(_base) ((_base) + 0x84)
	#define CC_IMSC_T1_OFFSET 0x9
	#define CC_IMSC_T1_WIDTH 0x1
	#define CC_IMSC_T1_MASK 0x200
	#define CC_IMSC_T1(_reg) (((_reg) & 0x200) >> 0x9)
		#define CC_IMSC_T1_DIS (0x0 << CC_IMSC_T1_OFFSET)
		#define CC_IMSC_T1_EN (0x1 << CC_IMSC_T1_OFFSET)
	#define CC_IMSC_T0_OFFSET 0x8
	#define CC_IMSC_T0_WIDTH 0x1
	#define CC_IMSC_T0_MASK 0x100
	#define CC_IMSC_T0(_reg) (((_reg) & 0x100) >> 0x8)
		#define CC_IMSC_T0_DIS (0x0 << CC_IMSC_T0_OFFSET)
		#define CC_IMSC_T0_EN (0x1 << CC_IMSC_T0_OFFSET)
	#define CC_IMSC_C7_OFFSET 0x7
	#define CC_IMSC_C7_WIDTH 0x1
	#define CC_IMSC_C7_MASK 0x80
	#define CC_IMSC_C7(_reg) (((_reg) & 0x80) >> 0x7)
		#define CC_IMSC_C7_DIS (0x0 << CC_IMSC_C7_OFFSET)
		#define CC_IMSC_C7_EN (0x1 << CC_IMSC_C7_OFFSET)
	#define CC_IMSC_C6_OFFSET 0x6
	#define CC_IMSC_C6_WIDTH 0x1
	#define CC_IMSC_C6_MASK 0x40
	#define CC_IMSC_C6(_reg) (((_reg) & 0x40) >> 0x6)
		#define CC_IMSC_C6_DIS (0x0 << CC_IMSC_C6_OFFSET)
		#define CC_IMSC_C6_EN (0x1 << CC_IMSC_C6_OFFSET)
	#define CC_IMSC_C5_OFFSET 0x5
	#define CC_IMSC_C5_WIDTH 0x1
	#define CC_IMSC_C5_MASK 0x20
	#define CC_IMSC_C5(_reg) (((_reg) & 0x20) >> 0x5)
		#define CC_IMSC_C5_DIS (0x0 << CC_IMSC_C5_OFFSET)
		#define CC_IMSC_C5_EN (0x1 << CC_IMSC_C5_OFFSET)
	#define CC_IMSC_C4_OFFSET 0x4
	#define CC_IMSC_C4_WIDTH 0x1
	#define CC_IMSC_C4_MASK 0x10
	#define CC_IMSC_C4(_reg) (((_reg) & 0x10) >> 0x4)
		#define CC_IMSC_C4_DIS (0x0 << CC_IMSC_C4_OFFSET)
		#define CC_IMSC_C4_EN (0x1 << CC_IMSC_C4_OFFSET)
	#define CC_IMSC_C3_OFFSET 0x3
	#define CC_IMSC_C3_WIDTH 0x1
	#define CC_IMSC_C3_MASK 0x8
	#define CC_IMSC_C3(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_IMSC_C3_DIS (0x0 << CC_IMSC_C3_OFFSET)
		#define CC_IMSC_C3_EN (0x1 << CC_IMSC_C3_OFFSET)
	#define CC_IMSC_C2_OFFSET 0x2
	#define CC_IMSC_C2_WIDTH 0x1
	#define CC_IMSC_C2_MASK 0x4
	#define CC_IMSC_C2(_reg) (((_reg) & 0x4) >> 0x2)
		#define CC_IMSC_C2_DIS (0x0 << CC_IMSC_C2_OFFSET)
		#define CC_IMSC_C2_EN (0x1 << CC_IMSC_C2_OFFSET)
	#define CC_IMSC_C1_OFFSET 0x1
	#define CC_IMSC_C1_WIDTH 0x1
	#define CC_IMSC_C1_MASK 0x2
	#define CC_IMSC_C1(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_IMSC_C1_DIS (0x0 << CC_IMSC_C1_OFFSET)
		#define CC_IMSC_C1_EN (0x1 << CC_IMSC_C1_OFFSET)
	#define CC_IMSC_C0_OFFSET 0x0
	#define CC_IMSC_C0_WIDTH 0x1
	#define CC_IMSC_C0_MASK 0x1
	#define CC_IMSC_C0(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_IMSC_C0_DIS (0x0 << CC_IMSC_C0_OFFSET)
		#define CC_IMSC_C0_EN (0x1 << CC_IMSC_C0_OFFSET)

#define CC_MIS_OFFSET   0x88
#define CC_MIS(_base) ((_base) + 0x88)
	#define CC_MIS_T1_OFFSET 0x9
	#define CC_MIS_T1_WIDTH 0x1
	#define CC_MIS_T1_MASK 0x200
	#define CC_MIS_T1(_reg) (((_reg) & 0x200) >> 0x9)
		#define CC_MIS_T1_NI (0x0 << CC_MIS_T1_OFFSET)
		#define CC_MIS_T1_IP (0x1 << CC_MIS_T1_OFFSET)
	#define CC_MIS_T0_OFFSET 0x8
	#define CC_MIS_T0_WIDTH 0x1
	#define CC_MIS_T0_MASK 0x100
	#define CC_MIS_T0(_reg) (((_reg) & 0x100) >> 0x8)
		#define CC_MIS_T0_NI (0x0 << CC_MIS_T0_OFFSET)
		#define CC_MIS_T0_IP (0x1 << CC_MIS_T0_OFFSET)
	#define CC_MIS_C7_OFFSET 0x7
	#define CC_MIS_C7_WIDTH 0x1
	#define CC_MIS_C7_MASK 0x80
	#define CC_MIS_C7(_reg) (((_reg) & 0x80) >> 0x7)
		#define CC_MIS_C7_NI (0x0 << CC_MIS_C7_OFFSET)
		#define CC_MIS_C7_IP (0x1 << CC_MIS_C7_OFFSET)
	#define CC_MIS_C6_OFFSET 0x6
	#define CC_MIS_C6_WIDTH 0x1
	#define CC_MIS_C6_MASK 0x40
	#define CC_MIS_C6(_reg) (((_reg) & 0x40) >> 0x6)
		#define CC_MIS_C6_NI (0x0 << CC_MIS_C6_OFFSET)
		#define CC_MIS_C6_IP (0x1 << CC_MIS_C6_OFFSET)
	#define CC_MIS_C5_OFFSET 0x5
	#define CC_MIS_C5_WIDTH 0x1
	#define CC_MIS_C5_MASK 0x20
	#define CC_MIS_C5(_reg) (((_reg) & 0x20) >> 0x5)
		#define CC_MIS_C5_NI (0x0 << CC_MIS_C5_OFFSET)
		#define CC_MIS_C5_IP (0x1 << CC_MIS_C5_OFFSET)
	#define CC_MIS_C4_OFFSET 0x4
	#define CC_MIS_C4_WIDTH 0x1
	#define CC_MIS_C4_MASK 0x10
	#define CC_MIS_C4(_reg) (((_reg) & 0x10) >> 0x4)
		#define CC_MIS_C4_NI (0x0 << CC_MIS_C4_OFFSET)
		#define CC_MIS_C4_IP (0x1 << CC_MIS_C4_OFFSET)
	#define CC_MIS_C3_OFFSET 0x3
	#define CC_MIS_C3_WIDTH 0x1
	#define CC_MIS_C3_MASK 0x8
	#define CC_MIS_C3(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_MIS_C3_NI (0x0 << CC_MIS_C3_OFFSET)
		#define CC_MIS_C3_IP (0x1 << CC_MIS_C3_OFFSET)
	#define CC_MIS_C2_OFFSET 0x2
	#define CC_MIS_C2_WIDTH 0x1
	#define CC_MIS_C2_MASK 0x4
	#define CC_MIS_C2(_reg) (((_reg) & 0x4) >> 0x2)
		#define CC_MIS_C2_NI (0x0 << CC_MIS_C2_OFFSET)
		#define CC_MIS_C2_IP (0x1 << CC_MIS_C2_OFFSET)
	#define CC_MIS_C1_OFFSET 0x1
	#define CC_MIS_C1_WIDTH 0x1
	#define CC_MIS_C1_MASK 0x2
	#define CC_MIS_C1(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_MIS_C1_NI (0x0 << CC_MIS_C1_OFFSET)
		#define CC_MIS_C1_IP (0x1 << CC_MIS_C1_OFFSET)
	#define CC_MIS_C0_OFFSET 0x0
	#define CC_MIS_C0_WIDTH 0x1
	#define CC_MIS_C0_MASK 0x1
	#define CC_MIS_C0(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_MIS_C0_NI (0x0 << CC_MIS_C0_OFFSET)
		#define CC_MIS_C0_IP (0x1 << CC_MIS_C0_OFFSET)

#define CC_ICR_OFFSET   0x8c
#define CC_ICR(_base) ((_base) + 0x8c)
	#define CC_ICR_T1_OFFSET 0x9
	#define CC_ICR_T1_WIDTH 0x1
	#define CC_ICR_T1_MASK 0x200
	#define CC_ICR_T1(_reg) (((_reg) & 0x200) >> 0x9)
		#define CC_ICR_T1_NOC (0x0 << CC_ICR_T1_OFFSET)
		#define CC_ICR_T1_CSR (0x1 << CC_ICR_T1_OFFSET)
	#define CC_ICR_T0_OFFSET 0x8
	#define CC_ICR_T0_WIDTH 0x1
	#define CC_ICR_T0_MASK 0x100
	#define CC_ICR_T0(_reg) (((_reg) & 0x100) >> 0x8)
		#define CC_ICR_T0_NOC (0x0 << CC_ICR_T0_OFFSET)
		#define CC_ICR_T0_CSR (0x1 << CC_ICR_T0_OFFSET)
	#define CC_ICR_C7_OFFSET 0x7
	#define CC_ICR_C7_WIDTH 0x1
	#define CC_ICR_C7_MASK 0x80
	#define CC_ICR_C7(_reg) (((_reg) & 0x80) >> 0x7)
		#define CC_ICR_C7_NOC (0x0 << CC_ICR_C7_OFFSET)
		#define CC_ICR_C7_CSR (0x1 << CC_ICR_C7_OFFSET)
	#define CC_ICR_C6_OFFSET 0x6
	#define CC_ICR_C6_WIDTH 0x1
	#define CC_ICR_C6_MASK 0x40
	#define CC_ICR_C6(_reg) (((_reg) & 0x40) >> 0x6)
		#define CC_ICR_C6_NOC (0x0 << CC_ICR_C6_OFFSET)
		#define CC_ICR_C6_CSR (0x1 << CC_ICR_C6_OFFSET)
	#define CC_ICR_C5_OFFSET 0x5
	#define CC_ICR_C5_WIDTH 0x1
	#define CC_ICR_C5_MASK 0x20
	#define CC_ICR_C5(_reg) (((_reg) & 0x20) >> 0x5)
		#define CC_ICR_C5_NOC (0x0 << CC_ICR_C5_OFFSET)
		#define CC_ICR_C5_CSR (0x1 << CC_ICR_C5_OFFSET)
	#define CC_ICR_C4_OFFSET 0x4
	#define CC_ICR_C4_WIDTH 0x1
	#define CC_ICR_C4_MASK 0x10
	#define CC_ICR_C4(_reg) (((_reg) & 0x10) >> 0x4)
		#define CC_ICR_C4_NOC (0x0 << CC_ICR_C4_OFFSET)
		#define CC_ICR_C4_CSR (0x1 << CC_ICR_C4_OFFSET)
	#define CC_ICR_C3_OFFSET 0x3
	#define CC_ICR_C3_WIDTH 0x1
	#define CC_ICR_C3_MASK 0x8
	#define CC_ICR_C3(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_ICR_C3_NOC (0x0 << CC_ICR_C3_OFFSET)
		#define CC_ICR_C3_CSR (0x1 << CC_ICR_C3_OFFSET)
	#define CC_ICR_C2_OFFSET 0x2
	#define CC_ICR_C2_WIDTH 0x1
	#define CC_ICR_C2_MASK 0x4
	#define CC_ICR_C2(_reg) (((_reg) & 0x4) >> 0x2)
		#define CC_ICR_C2_NOC (0x0 << CC_ICR_C2_OFFSET)
		#define CC_ICR_C2_CSR (0x1 << CC_ICR_C2_OFFSET)
	#define CC_ICR_C1_OFFSET 0x1
	#define CC_ICR_C1_WIDTH 0x1
	#define CC_ICR_C1_MASK 0x2
	#define CC_ICR_C1(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_ICR_C1_NOC (0x0 << CC_ICR_C1_OFFSET)
		#define CC_ICR_C1_CSR (0x1 << CC_ICR_C1_OFFSET)
	#define CC_ICR_C0_OFFSET 0x0
	#define CC_ICR_C0_WIDTH 0x1
	#define CC_ICR_C0_MASK 0x1
	#define CC_ICR_C0(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_ICR_C0_NOC (0x0 << CC_ICR_C0_OFFSET)
		#define CC_ICR_C0_CSR (0x1 << CC_ICR_C0_OFFSET)

#define CC_ISR_OFFSET   0x90
#define CC_ISR(_base) ((_base) + 0x90)
	#define CC_ISR_T1_OFFSET 0x9
	#define CC_ISR_T1_WIDTH 0x1
	#define CC_ISR_T1_MASK 0x200
	#define CC_ISR_T1(_reg) (((_reg) & 0x200) >> 0x9)
		#define CC_ISR_T1_NOC (0x0 << CC_ISR_T1_OFFSET)
		#define CC_ISR_T1_SI (0x1 << CC_ISR_T1_OFFSET)
	#define CC_ISR_T0_OFFSET 0x8
	#define CC_ISR_T0_WIDTH 0x1
	#define CC_ISR_T0_MASK 0x100
	#define CC_ISR_T0(_reg) (((_reg) & 0x100) >> 0x8)
		#define CC_ISR_T0_NOC (0x0 << CC_ISR_T0_OFFSET)
		#define CC_ISR_T0_SI (0x1 << CC_ISR_T0_OFFSET)
	#define CC_ISR_C7_OFFSET 0x7
	#define CC_ISR_C7_WIDTH 0x1
	#define CC_ISR_C7_MASK 0x80
	#define CC_ISR_C7(_reg) (((_reg) & 0x80) >> 0x7)
		#define CC_ISR_C7_NOC (0x0 << CC_ISR_C7_OFFSET)
		#define CC_ISR_C7_SI (0x1 << CC_ISR_C7_OFFSET)
	#define CC_ISR_C6_OFFSET 0x6
	#define CC_ISR_C6_WIDTH 0x1
	#define CC_ISR_C6_MASK 0x40
	#define CC_ISR_C6(_reg) (((_reg) & 0x40) >> 0x6)
		#define CC_ISR_C6_NOC (0x0 << CC_ISR_C6_OFFSET)
		#define CC_ISR_C6_SI (0x1 << CC_ISR_C6_OFFSET)
	#define CC_ISR_C5_OFFSET 0x5
	#define CC_ISR_C5_WIDTH 0x1
	#define CC_ISR_C5_MASK 0x20
	#define CC_ISR_C5(_reg) (((_reg) & 0x20) >> 0x5)
		#define CC_ISR_C5_NOC (0x0 << CC_ISR_C5_OFFSET)
		#define CC_ISR_C5_SI (0x1 << CC_ISR_C5_OFFSET)
	#define CC_ISR_C4_OFFSET 0x4
	#define CC_ISR_C4_WIDTH 0x1
	#define CC_ISR_C4_MASK 0x10
	#define CC_ISR_C4(_reg) (((_reg) & 0x10) >> 0x4)
		#define CC_ISR_C4_NOC (0x0 << CC_ISR_C4_OFFSET)
		#define CC_ISR_C4_SI (0x1 << CC_ISR_C4_OFFSET)
	#define CC_ISR_C3_OFFSET 0x3
	#define CC_ISR_C3_WIDTH 0x1
	#define CC_ISR_C3_MASK 0x8
	#define CC_ISR_C3(_reg) (((_reg) & 0x8) >> 0x3)
		#define CC_ISR_C3_NOC (0x0 << CC_ISR_C3_OFFSET)
		#define CC_ISR_C3_SI (0x1 << CC_ISR_C3_OFFSET)
	#define CC_ISR_C2_OFFSET 0x2
	#define CC_ISR_C2_WIDTH 0x1
	#define CC_ISR_C2_MASK 0x4
	#define CC_ISR_C2(_reg) (((_reg) & 0x4) >> 0x2)
		#define CC_ISR_C2_NOC (0x0 << CC_ISR_C2_OFFSET)
		#define CC_ISR_C2_SI (0x1 << CC_ISR_C2_OFFSET)
	#define CC_ISR_C1_OFFSET 0x1
	#define CC_ISR_C1_WIDTH 0x1
	#define CC_ISR_C1_MASK 0x2
	#define CC_ISR_C1(_reg) (((_reg) & 0x2) >> 0x1)
		#define CC_ISR_C1_NOC (0x0 << CC_ISR_C1_OFFSET)
		#define CC_ISR_C1_SI (0x1 << CC_ISR_C1_OFFSET)
	#define CC_ISR_C0_OFFSET 0x0
	#define CC_ISR_C0_WIDTH 0x1
	#define CC_ISR_C0_MASK 0x1
	#define CC_ISR_C0(_reg) (((_reg) & 0x1) >> 0x0)
		#define CC_ISR_C0_NOC (0x0 << CC_ISR_C0_OFFSET)
		#define CC_ISR_C0_SI (0x1 << CC_ISR_C0_OFFSET)

#endif /* __CC_XGOLD_REGS_H */
