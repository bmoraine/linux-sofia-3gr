/*
* Copyright (C) 2014 Intel Mobile Communications GmbH
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

#include <linux/init.h>
#ifndef _XGOLD_PMIC_H
#define _XGOLD_PMIC_H

#define PMIC_DEV1_SLAVE_ADDR (0x4E)
#define PMIC_DEV1_ADDR (PMIC_DEV1_SLAVE_ADDR<<24)

#define IRQLVL1_REG_OFFSET 0x2
#define IRQLVL1_REG(_base) ((_base) + 0x2)
#define IRQLVL1_REG_PWRBTN_OFFSET 0x0
#define IRQLVL1_REG_PWRBTN_WIDTH 0x1
#define IRQLVL1_REG_PWRBTN_MASK 0x1
#define IRQLVL1_REG_PWRBTN(_reg) (((_reg) & 0x1) >> 0x0)

#define MIRQLVL1_REG_OFFSET 0xe
#define MIRQLVL1_REG(_base) ((_base) + 0xe)
#define MIRQLVL1_REG_MPWRBTN_OFFSET 0x0
#define MIRQLVL1_REG_MPWRBTN_WIDTH 0x1
#define MIRQLVL1_REG_MPWRBTN_MASK 0x1
#define MIRQLVL1_REG_MPWRBTN(_reg) (((_reg) & 0x1) >> 0x0)
#define MIRQLVL1_REG_MTMU_OFFSET 0x1

#define PBIRQ_REG_OFFSET 0x3
#define PBIRQ_REG(_base) ((_base) + 0x3)
#define PBIRQ_REG_PBTN_OFFSET 0x0
#define PBIRQ_REG_PBTN_WIDTH 0x1
#define PBIRQ_REG_PBTN_MASK 0x1
#define PBIRQ_REG_PBTN(_reg) (((_reg) & 0x1) >> 0x0)
#define PBIRQ_REG_PBTN_NA (0x0 << PBIRQ_REG_PBTN_OFFSET)
#define PBIRQ_REG_PBTN_IA (0x1 << PBIRQ_REG_PBTN_OFFSET)
#define PBIRQ_REG_UBTN_OFFSET 0x1
#define PBIRQ_REG_UBTN_WIDTH 0x1
#define PBIRQ_REG_UBTN_MASK 0x2
#define PBIRQ_REG_UBTN(_reg) (((_reg) & 0x2) >> 0x1)
#define PBIRQ_REG_UBTN_NA (0x0 << PBIRQ_REG_UBTN_OFFSET)
#define PBIRQ_REG_UBTN_IA (0x1 << PBIRQ_REG_UBTN_OFFSET)
#define PBIRQ_REG_VEND_OFFSET 0x7
#define PBIRQ_REG_VEND_WIDTH 0x1
#define PBIRQ_REG_VEND_MASK 0x80
#define PBIRQ_REG_VEND(_reg) (((_reg) & 0x80) >> 0x7)
#define PBIRQ_REG_VEND_NA (0x0 << PBIRQ_REG_VEND_OFFSET)
#define PBIRQ_REG_VEND_IA (0x1 << PBIRQ_REG_VEND_OFFSET)

#define MPBIRQ_REG_OFFSET  0xf
#define MPBIRQ_REG(_base) ((_base) + 0xf)
#define MPBIRQ_REG_MPBTN_OFFSET 0x0
#define MPBIRQ_REG_MPBTN_WIDTH 0x1
#define MPBIRQ_REG_MPBTN_MASK 0x1
#define MPBIRQ_REG_MPBTN(_reg) (((_reg) & 0x1) >> 0x0)
#define MPBIRQ_REG_MPBTN_DIS (0x1 << MPBIRQ_REG_MPBTN_OFFSET)
#define MPBIRQ_REG_MPBTN_EN (0x0 << MPBIRQ_REG_MPBTN_OFFSET)
#define MPBIRQ_REG_MUBTN_OFFSET 0x1
#define MPBIRQ_REG_MUBTN_WIDTH 0x1
#define MPBIRQ_REG_MUBTN_MASK 0x2
#define MPBIRQ_REG_MUBTN(_reg) (((_reg) & 0x2) >> 0x1)
#define MPBIRQ_REG_MUBTN_DIS (0x1 << MPBIRQ_REG_MUBTN_OFFSET)
#define MPBIRQ_REG_MUBTN_EN (0x0 << MPBIRQ_REG_MUBTN_OFFSET)
#define MPBIRQ_REG_MVEND_OFFSET 0x7
#define MPBIRQ_REG_MVEND_WIDTH 0x1
#define MPBIRQ_REG_MVEND_MASK 0x80
#define MPBIRQ_REG_MVEND(_reg) (((_reg) & 0x80) >> 0x7)
#define MPBIRQ_REG_MVEND_DIS (0x1 << MPBIRQ_REG_MVEND_OFFSET)
#define MPBIRQ_REG_MVEND_EN (0x0 << MPBIRQ_REG_MVEND_OFFSET)

#define FLTCFG_REG_OFFSET 0x3d
#define FLTCFG_REG(_base) ((_base) + 0x3d)
#define FLTCFG_REG_FLTACT_OFFSET 0x0
#define FLTCFG_REG_FLTACT_WIDTH 0x2
#define FLTCFG_REG_FLTACT_MASK 0x3
#define FLTCFG_REG_FLTACT(_reg) (((_reg) & 0x3) >> 0x0)
#define FLTCFG_REG_FLTACT_PLFRST (0x0 << FLTCFG_REG_FLTACT_OFFSET)
#define FLTCFG_REG_FLTACT_RES1 (0x1 << FLTCFG_REG_FLTACT_OFFSET)
#define FLTCFG_REG_FLTACT_RES2 (0x2 << FLTCFG_REG_FLTACT_OFFSET)
#define FLTCFG_REG_FLTACT_CLDOFF (0x3 << FLTCFG_REG_FLTACT_OFFSET)

#define PBCONFIG1_REG_OFFSET 0x3e
#define PBCONFIG1_REG(_base) ((_base) + 0x3e)
#define PBCONFIG1_REG_FLT_OFFSET 0x0
#define PBCONFIG1_REG_FLT_WIDTH 0x4
#define PBCONFIG1_REG_FLT_MASK 0xf
#define PBCONFIG1_REG_FLT(_reg) (((_reg) & 0xf) >> 0x0)
#define PBCONFIG1_REG_PBHOLD_OFFSET 0x4
#define PBCONFIG1_REG_PBHOLD_WIDTH 0x3
#define PBCONFIG1_REG_PBHOLD_MASK 0x70
#define PBCONFIG1_REG_PBHOLD(_reg) (((_reg) & 0x70) >> 0x4)

#define PBCONFIG2_REG_OFFSET 0x3f
#define PBCONFIG2_REG(_base) ((_base) + 0x3f)
#define PBCONFIG2_REG_PBDIS_OFFSET 0x0
#define PBCONFIG2_REG_PBDIS_WIDTH 0x2
#define PBCONFIG2_REG_PBDIS_MASK 0x3
#define PBCONFIG2_REG_PBDIS(_reg) (((_reg) & 0x3) >> 0x0)
#define PBCONFIG2_REG_UIBTNDIS_OFFSET 0x2
#define PBCONFIG2_REG_UIBTNDIS_WIDTH 0x1
#define PBCONFIG2_REG_UIBTNDIS_MASK 0x4
#define PBCONFIG2_REG_UIBTNDIS(_reg) (((_reg) & 0x4) >> 0x2)

#define PBSTATUS_REG_OFFSET 0x40
#define PBSTATUS_REG(_base) ((_base) + 0x40)
#define PBSTATUS_REG_PBHT_OFFSET 0x0
#define PBSTATUS_REG_PBHT_WIDTH 0x4
#define PBSTATUS_REG_PBHT_MASK 0xf
#define PBSTATUS_REG_PBHT(_reg) (((_reg) & 0xf) >> 0x0)
#define PBSTATUS_REG_PBLVL_OFFSET 0x4
#define PBSTATUS_REG_PBLVL_WIDTH 0x1
#define PBSTATUS_REG_PBLVL_MASK 0x10
#define PBSTATUS_REG_PBLVL(_reg) (((_reg) & 0x10) >> 0x4)
#define PBSTATUS_REG_CLRFLT_OFFSET 0x5
#define PBSTATUS_REG_CLRFLT_WIDTH 0x1
#define PBSTATUS_REG_CLRFLT_MASK 0x20
#define PBSTATUS_REG_CLRFLT(_reg) (((_reg) & 0x20) >> 0x5)
#define PBSTATUS_REG_CLRHT_OFFSET 0x6
#define PBSTATUS_REG_CLRHT_WIDTH 0x1
#define PBSTATUS_REG_CLRHT_MASK 0x40
#define PBSTATUS_REG_CLRHT(_reg) (((_reg) & 0x40) >> 0x6)

#define UBSTATUS_REG_OFFSET 0x41
#define UBSTATUS_REG(_base) ((_base) + 0x41)
#define UBSTATUS_REG_UBHT_OFFSET 0x0
#define UBSTATUS_REG_UBHT_WIDTH 0x4
#define UBSTATUS_REG_UBHT_MASK 0xf
#define UBSTATUS_REG_UBHT(_reg) (((_reg) & 0xf) >> 0x0)
#define UBSTATUS_REG_UBLVL_OFFSET 0x4
#define UBSTATUS_REG_UBLVL_WIDTH 0x1
#define UBSTATUS_REG_UBLVL_MASK 0x10
#define UBSTATUS_REG_UBLVL(_reg) (((_reg) & 0x10) >> 0x4)

#define GPIO2CTLI_REG(_base) ((_base) + 0x53)
#define GPIO2CTLO_REG(_base) ((_base) + 0x46)
#define GPIO2CTLO_REG_ALTFUNCEN_OFFSET 0x6
#define GPIO2CTLO_REG_ALTFUNCEN_WIDTH 0x1
#define GPIO2CTLO_REG_ALTFUNCEN_MASK 0x40
#define GPIO2CTLO_REG_ALTFUNCEN(_reg) (((_reg) & 0x40) >> 0x6)

#endif /* _XGOLD_PMIC_H_ */
