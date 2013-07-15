/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/* #define BTIF_FORCE_DEBUG */
#ifdef BTIF_FORCE_DEBUG
#ifdef dev_dbg
#undef dev_dbg
#endif
#define dev_dbg(dev, format, arg...) printk(format, ##arg)

#ifdef dev_info
#undef dev_info
#endif
#define dev_info(dev, format, arg...) printk(format, ##arg)

#ifdef dev_warn
#undef dev_warn
#endif
#define dev_warn(dev, format, arg...) printk(format, ##arg)

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug printk
#endif /* BTIF_FORCE_DEBUG */

#define BTIF_ID_OFFSET 0xc
#define BTIF_ID(_base) ((_base) + 0xc)
	#define BTIF_ID_REV_NUMBER_OFFSET 0x0
	#define BTIF_ID_REV_NUMBER_WIDTH 0x8
	#define BTIF_ID_REV_NUMBER_MASK 0xff
	#define BTIF_ID_REV_NUMBER(_reg) (((_reg) & 0xff) >> 0x0)
	#define BTIF_ID_MOD_ID_OFFSET 0x8
	#define BTIF_ID_MOD_ID_WIDTH 0x8
	#define BTIF_ID_MOD_ID_MASK 0xff00
	#define BTIF_ID_MOD_ID(_reg) (((_reg) & 0xff00) >> 0x8)
	#define BTIF_ID_TS_REV_NR_OFFSET 0x10
	#define BTIF_ID_TS_REV_NR_WIDTH 0x10
	#define BTIF_ID_TS_REV_NR_MASK 0xffff0000
	#define BTIF_ID_TS_REV_NR(_reg) (((_reg) & 0xffff0000) >> 0x10)

#define BTIF_FIFO_ID_OFFSET 0x10
#define BTIF_FIFO_ID(_base) ((_base) + 0x10)
	#define BTIF_FIFO_ID_TX_STAGE_OFFSET 0x0
	#define BTIF_FIFO_ID_TX_STAGE_WIDTH 0x8
	#define BTIF_FIFO_ID_TX_STAGE_MASK 0xff
	#define BTIF_FIFO_ID_TX_STAGE(_reg) (((_reg) & 0xff) >> 0x0)
		#define BTIF_FIFO_ID_TX_STAGE_TX_STAGE_NUM \
			(0x4 << BTIF_FIFO_ID_TX_STAGE_OFFSET)
	#define BTIF_FIFO_ID_RX_STAGE_OFFSET 0x8
	#define BTIF_FIFO_ID_RX_STAGE_WIDTH 0x8
	#define BTIF_FIFO_ID_RX_STAGE_MASK 0xff00
	#define BTIF_FIFO_ID_RX_STAGE(_reg) (((_reg) & 0xff00) >> 0x8)
		#define BTIF_FIFO_ID_RX_STAGE_RX_STAGE_NUM \
			(0x4 << BTIF_FIFO_ID_RX_STAGE_OFFSET)
	#define BTIF_FIFO_ID_RPS_STAGE_OFFSET 0x10
	#define BTIF_FIFO_ID_RPS_STAGE_WIDTH 0x6
	#define BTIF_FIFO_ID_RPS_STAGE_MASK 0x3f0000
	#define BTIF_FIFO_ID_RPS_STAGE(_reg) (((_reg) & 0x3f0000) >> 0x10)
		#define BTIF_FIFO_ID_RPS_STAGE_RPS_STAGE_NUM \
			(0x0 << BTIF_FIFO_ID_RPS_STAGE_OFFSET)

#define BTIF_SRB_MSCONF_OFFSET 0x14
#define BTIF_SRB_MSCONF(_base) ((_base) + 0x14)
	#define BTIF_SRB_MSCONF_SEL_MULTISRC_IRQ_OFFSET 0x0
	#define BTIF_SRB_MSCONF_SEL_MULTISRC_IRQ_WIDTH 0x20
	#define BTIF_SRB_MSCONF_SEL_MULTISRC_IRQ_MASK 0xffffffff
	#define BTIF_SRB_MSCONF_SEL_MULTISRC_IRQ(_reg) \
			(((_reg) & 0xffffffff) >> 0x0)

#define BTIF_SRB_ERRCONF_OFFSET 0x18
#define BTIF_SRB_ERRCONF(_base) ((_base) + 0x18)
	#define BTIF_SRB_ERRCONF_SEL_ERR_IRQ_OFFSET 0x0
	#define BTIF_SRB_ERRCONF_SEL_ERR_IRQ_WIDTH 0x20
	#define BTIF_SRB_ERRCONF_SEL_ERR_IRQ_MASK 0xffffffff
	#define BTIF_SRB_ERRCONF_SEL_ERR_IRQ(_reg) \
			(((_reg) & 0xffffffff) >> 0x0)

#define BTIF_SWCID_OFFSET 0x1c
#define BTIF_SWCID(_base) ((_base) + 0x1c)
	#define BTIF_SWCID_DADWD_OFFSET 0x0
	#define BTIF_SWCID_DADWD_WIDTH 0x5
	#define BTIF_SWCID_DADWD_MASK 0x1f
	#define BTIF_SWCID_DADWD(_reg) (((_reg) & 0x1f) >> 0x0)
	#define BTIF_SWCID_SWCID_OFFSET 0x5
	#define BTIF_SWCID_SWCID_WIDTH 0x1b
	#define BTIF_SWCID_SWCID_MASK 0xffffffe0
	#define BTIF_SWCID_SWCID(_reg) (((_reg) & 0xffffffe0) >> 0x5)

#define BTIF_CLC_OFFSET 0x0
#define BTIF_CLC(_base) ((_base) + 0x0)
	#define BTIF_CLC_RUN_OFFSET 0x0
	#define BTIF_CLC_RUN_WIDTH 0x2
	#define BTIF_CLC_RUN_MASK 0x3
	#define BTIF_CLC_RUN(_reg) (((_reg) & 0x3) >> 0x0)
		#define BTIF_CLC_RUN_STOP (0x2 << BTIF_CLC_RUN_OFFSET)
		#define BTIF_CLC_RUN_RUN (0x1 << BTIF_CLC_RUN_OFFSET)
		#define BTIF_CLC_RUN_RESERVED1 (0x0 << BTIF_CLC_RUN_OFFSET)
		#define BTIF_CLC_RUN_RESERVED2 (0x3 << BTIF_CLC_RUN_OFFSET)
	#define BTIF_CLC_MOD_EN_OFFSET 0x2
	#define BTIF_CLC_MOD_EN_WIDTH 0x2
	#define BTIF_CLC_MOD_EN_MASK 0xc
	#define BTIF_CLC_MOD_EN(_reg) (((_reg) & 0xc) >> 0x2)
		#define BTIF_CLC_MOD_EN_DIS_REQ (0x2 << BTIF_CLC_MOD_EN_OFFSET)
		#define BTIF_CLC_MOD_EN_EN_REQ (0x1 << BTIF_CLC_MOD_EN_OFFSET)
		#define BTIF_CLC_MOD_EN_RESERVED1 \
				(0x0 << BTIF_CLC_MOD_EN_OFFSET)
		#define BTIF_CLC_MOD_EN_RESERVED2 \
				(0x3 << BTIF_CLC_MOD_EN_OFFSET)
	#define BTIF_CLC_SPEN_OFFSET 0x4
	#define BTIF_CLC_SPEN_WIDTH 0x2
	#define BTIF_CLC_SPEN_MASK 0x30
	#define BTIF_CLC_SPEN(_reg) (((_reg) & 0x30) >> 0x4)
		#define BTIF_CLC_SPEN_SPEN_DIS (0x2 << BTIF_CLC_SPEN_OFFSET)
		#define BTIF_CLC_SPEN_SPEN_EN (0x1 << BTIF_CLC_SPEN_OFFSET)
		#define BTIF_CLC_SPEN_RESERVED1 (0x0 << BTIF_CLC_SPEN_OFFSET)
		#define BTIF_CLC_SPEN_RESERVED2 (0x3 << BTIF_CLC_SPEN_OFFSET)
	#define BTIF_CLC_FSOE_OFFSET 0x6
	#define BTIF_CLC_FSOE_WIDTH 0x2
	#define BTIF_CLC_FSOE_MASK 0xc0
	#define BTIF_CLC_FSOE(_reg) (((_reg) & 0xc0) >> 0x6)
		#define BTIF_CLC_FSOE_FSOE_DIS (0x2 << BTIF_CLC_FSOE_OFFSET)
		#define BTIF_CLC_FSOE_FSOE_EN (0x1 << BTIF_CLC_FSOE_OFFSET)
		#define BTIF_CLC_FSOE_RESERVED1 (0x0 << BTIF_CLC_FSOE_OFFSET)
		#define BTIF_CLC_FSOE_RESERVED2 (0x3 << BTIF_CLC_FSOE_OFFSET)
	#define BTIF_CLC_EDREN_OFFSET 0x8
	#define BTIF_CLC_EDREN_WIDTH 0x2
	#define BTIF_CLC_EDREN_MASK 0x300
	#define BTIF_CLC_EDREN(_reg) (((_reg) & 0x300) >> 0x8)
		#define BTIF_CLC_EDREN_EDR_DIS (0x2 << BTIF_CLC_EDREN_OFFSET)
		#define BTIF_CLC_EDREN_EDR_EN (0x1 << BTIF_CLC_EDREN_OFFSET)
		#define BTIF_CLC_EDREN_RESERVED1 (0x0 << BTIF_CLC_EDREN_OFFSET)
		#define BTIF_CLC_EDREN_RESERVED2 (0x3 << BTIF_CLC_EDREN_OFFSET)
	#define BTIF_CLC_IDREN_OFFSET 0xa
	#define BTIF_CLC_IDREN_WIDTH 0x2
	#define BTIF_CLC_IDREN_MASK 0xc00
	#define BTIF_CLC_IDREN(_reg) (((_reg) & 0xc00) >> 0xa)
		#define BTIF_CLC_IDREN_IDR_DIS (0x2 << BTIF_CLC_IDREN_OFFSET)
		#define BTIF_CLC_IDREN_IDR_EN (0x1 << BTIF_CLC_IDREN_OFFSET)
		#define BTIF_CLC_IDREN_RESERVED1 (0x0 << BTIF_CLC_IDREN_OFFSET)
		#define BTIF_CLC_IDREN_RESERVED2 (0x3 << BTIF_CLC_IDREN_OFFSET)

#define BTIF_CLC_CNT_OFFSET 0x4
#define BTIF_CLC_CNT(_base) ((_base) + 0x4)
	#define BTIF_CLC_CNT_RMC_OFFSET 0x0
	#define BTIF_CLC_CNT_RMC_WIDTH 0x8
	#define BTIF_CLC_CNT_RMC_MASK 0xff
	#define BTIF_CLC_CNT_RMC(_reg) (((_reg) & 0xff) >> 0x0)
	#define BTIF_CLC_CNT_ORMC_OFFSET 0x8
	#define BTIF_CLC_CNT_ORMC_WIDTH 0x8
	#define BTIF_CLC_CNT_ORMC_MASK 0xff00
	#define BTIF_CLC_CNT_ORMC(_reg) (((_reg) & 0xff00) >> 0x8)

#define BTIF_CLC_STAT_OFFSET 0x8
#define BTIF_CLC_STAT(_base) ((_base) + 0x8)
	#define BTIF_CLC_STAT_RUN_OFFSET 0x0
	#define BTIF_CLC_STAT_RUN_WIDTH 0x1
	#define BTIF_CLC_STAT_RUN_MASK 0x1
	#define BTIF_CLC_STAT_RUN(_reg) (((_reg) & 0x1) >> 0x0)
		#define BTIF_CLC_STAT_RUN_CONF_MODE \
				(0x0 << BTIF_CLC_STAT_RUN_OFFSET)
		#define BTIF_CLC_STAT_RUN_RUN_MODE \
				(0x1 << BTIF_CLC_STAT_RUN_OFFSET)
	#define BTIF_CLC_STAT_MODEN_OFFSET 0x1
	#define BTIF_CLC_STAT_MODEN_WIDTH 0x1
	#define BTIF_CLC_STAT_MODEN_MASK 0x2
	#define BTIF_CLC_STAT_MODEN(_reg) (((_reg) & 0x2) >> 0x1)
		#define BTIF_CLC_STAT_MODEN_MOD_DIS \
				(0x0 << BTIF_CLC_STAT_MODEN_OFFSET)
		#define BTIF_CLC_STAT_MODEN_MOD_EN \
				(0x1 << BTIF_CLC_STAT_MODEN_OFFSET)
	#define BTIF_CLC_STAT_SPEN_OFFSET 0x2
	#define BTIF_CLC_STAT_SPEN_WIDTH 0x1
	#define BTIF_CLC_STAT_SPEN_MASK 0x4
	#define BTIF_CLC_STAT_SPEN(_reg) (((_reg) & 0x4) >> 0x2)
		#define BTIF_CLC_STAT_SPEN_SPEN_DIS \
				(0x0 << BTIF_CLC_STAT_SPEN_OFFSET)
		#define BTIF_CLC_STAT_SPEN_SPEN_EN \
				(0x1 << BTIF_CLC_STAT_SPEN_OFFSET)
	#define BTIF_CLC_STAT_FSOE_OFFSET 0x3
	#define BTIF_CLC_STAT_FSOE_WIDTH 0x1
	#define BTIF_CLC_STAT_FSOE_MASK 0x8
	#define BTIF_CLC_STAT_FSOE(_reg) (((_reg) & 0x8) >> 0x3)
		#define BTIF_CLC_STAT_FSOE_FSOE_DIS \
				(0x0 << BTIF_CLC_STAT_FSOE_OFFSET)
		#define BTIF_CLC_STAT_FSOE_FSOE_EN \
				(0x1 << BTIF_CLC_STAT_FSOE_OFFSET)
	#define BTIF_CLC_STAT_EDRE_OFFSET 0x4
	#define BTIF_CLC_STAT_EDRE_WIDTH 0x1
	#define BTIF_CLC_STAT_EDRE_MASK 0x10
	#define BTIF_CLC_STAT_EDRE(_reg) (((_reg) & 0x10) >> 0x4)
		#define BTIF_CLC_STAT_EDRE_EDR_DIS \
				(0x0 << BTIF_CLC_STAT_EDRE_OFFSET)
		#define BTIF_CLC_STAT_EDRE_EDR_EN \
				(0x1 << BTIF_CLC_STAT_EDRE_OFFSET)
	#define BTIF_CLC_STAT_IDRE_OFFSET 0x5
	#define BTIF_CLC_STAT_IDRE_WIDTH 0x1
	#define BTIF_CLC_STAT_IDRE_MASK 0x20
	#define BTIF_CLC_STAT_IDRE(_reg) (((_reg) & 0x20) >> 0x5)
		#define BTIF_CLC_STAT_IDRE_IDR_DIS \
				(0x0 << BTIF_CLC_STAT_IDRE_OFFSET)
		#define BTIF_CLC_STAT_IDRE_IDR_EN \
				(0x1 << BTIF_CLC_STAT_IDRE_OFFSET)
	#define BTIF_CLC_STAT_CUOK_OFFSET 0x6
	#define BTIF_CLC_STAT_CUOK_WIDTH 0x1
	#define BTIF_CLC_STAT_CUOK_MASK 0x40
	#define BTIF_CLC_STAT_CUOK(_reg) (((_reg) & 0x40) >> 0x6)
		#define BTIF_CLC_STAT_CUOK_CU_ACT \
				(0x0 << BTIF_CLC_STAT_CUOK_OFFSET)
		#define BTIF_CLC_STAT_CUOK_CU_RDY \
				(0x1 << BTIF_CLC_STAT_CUOK_OFFSET)
	#define BTIF_CLC_STAT_KID_OFFSET 0x7
	#define BTIF_CLC_STAT_KID_WIDTH 0x1
	#define BTIF_CLC_STAT_KID_MASK 0x80
	#define BTIF_CLC_STAT_KID(_reg) (((_reg) & 0x80) >> 0x7)
		#define BTIF_CLC_STAT_KID_KEACT \
				(0x0 << BTIF_CLC_STAT_KID_OFFSET)
		#define BTIF_CLC_STAT_KID_KEIDLE \
				(0x1 << BTIF_CLC_STAT_KID_OFFSET)
	#define BTIF_CLC_STAT_FIFOID_OFFSET 0x8
	#define BTIF_CLC_STAT_FIFOID_WIDTH 0x1
	#define BTIF_CLC_STAT_FIFOID_MASK 0x100
	#define BTIF_CLC_STAT_FIFOID(_reg) (((_reg) & 0x100) >> 0x8)
		#define BTIF_CLC_STAT_FIFOID_FIFOACT \
				(0x0 << BTIF_CLC_STAT_FIFOID_OFFSET)
		#define BTIF_CLC_STAT_FIFOID_FIFOIDLE \
				(0x1 << BTIF_CLC_STAT_FIFOID_OFFSET)

#define BTIF_RIS_OFFSET 0x80
#define BTIF_RIS(_base) ((_base) + 0x80)
	#define BTIF_RIS_RX_LSREQ_OFFSET 0x0
	#define BTIF_RIS_RX_LSREQ_WIDTH 0x1
	#define BTIF_RIS_RX_LSREQ_MASK 0x1
	#define BTIF_RIS_RX_LSREQ(_reg) (((_reg) & 0x1) >> 0x0)
	#define BTIF_RIS_RX_SREQ_OFFSET 0x1
	#define BTIF_RIS_RX_SREQ_WIDTH 0x1
	#define BTIF_RIS_RX_SREQ_MASK 0x2
	#define BTIF_RIS_RX_SREQ(_reg) (((_reg) & 0x2) >> 0x1)
	#define BTIF_RIS_RX_LBREQ_OFFSET 0x2
	#define BTIF_RIS_RX_LBREQ_WIDTH 0x1
	#define BTIF_RIS_RX_LBREQ_MASK 0x4
	#define BTIF_RIS_RX_LBREQ(_reg) (((_reg) & 0x4) >> 0x2)
	#define BTIF_RIS_RX_BREQ_OFFSET 0x3
	#define BTIF_RIS_RX_BREQ_WIDTH 0x1
	#define BTIF_RIS_RX_BREQ_MASK 0x8
	#define BTIF_RIS_RX_BREQ(_reg) (((_reg) & 0x8) >> 0x3)
	#define BTIF_RIS_TX_LSREQ_OFFSET 0x4
	#define BTIF_RIS_TX_LSREQ_WIDTH 0x1
	#define BTIF_RIS_TX_LSREQ_MASK 0x10
	#define BTIF_RIS_TX_LSREQ(_reg) (((_reg) & 0x10) >> 0x4)
	#define BTIF_RIS_TX_SREQ_OFFSET 0x5
	#define BTIF_RIS_TX_SREQ_WIDTH 0x1
	#define BTIF_RIS_TX_SREQ_MASK 0x20
	#define BTIF_RIS_TX_SREQ(_reg) (((_reg) & 0x20) >> 0x5)
	#define BTIF_RIS_TX_LBREQ_OFFSET 0x6
	#define BTIF_RIS_TX_LBREQ_WIDTH 0x1
	#define BTIF_RIS_TX_LBREQ_MASK 0x40
	#define BTIF_RIS_TX_LBREQ(_reg) (((_reg) & 0x40) >> 0x6)
	#define BTIF_RIS_TX_BREQ_OFFSET 0x7
	#define BTIF_RIS_TX_BREQ_WIDTH 0x1
	#define BTIF_RIS_TX_BREQ_MASK 0x80
	#define BTIF_RIS_TX_BREQ(_reg) (((_reg) & 0x80) >> 0x7)
	#define BTIF_RIS_RX_FUFL_OFFSET 0xb
	#define BTIF_RIS_RX_FUFL_WIDTH 0x1
	#define BTIF_RIS_RX_FUFL_MASK 0x800
	#define BTIF_RIS_RX_FUFL(_reg) (((_reg) & 0x800) >> 0xb)
	#define BTIF_RIS_TX_FOFL_OFFSET 0xe
	#define BTIF_RIS_TX_FOFL_WIDTH 0x1
	#define BTIF_RIS_TX_FOFL_MASK 0x4000
	#define BTIF_RIS_TX_FOFL(_reg) (((_reg) & 0x4000) >> 0xe)
	#define BTIF_RIS_FCI_OFFSET 0x10
	#define BTIF_RIS_FCI_WIDTH 0x1
	#define BTIF_RIS_FCI_MASK 0x10000
	#define BTIF_RIS_FCI(_reg) (((_reg) & 0x10000) >> 0x10)
	#define BTIF_RIS_FCI_N_OFFSET 0x11
	#define BTIF_RIS_FCI_N_WIDTH 0x1
	#define BTIF_RIS_FCI_N_MASK 0x20000
	#define BTIF_RIS_FCI_N(_reg) (((_reg) & 0x20000) >> 0x11)
	#define BTIF_RIS_RX_MRPSSTOP_OFFSET 0x18
	#define BTIF_RIS_RX_MRPSSTOP_WIDTH 0x1
	#define BTIF_RIS_RX_MRPSSTOP_MASK 0x1000000
	#define BTIF_RIS_RX_MRPSSTOP(_reg) (((_reg) & 0x1000000) >> 0x18)
	#define BTIF_RIS_RX_EOP_OFFSET 0x19
	#define BTIF_RIS_RX_EOP_WIDTH 0x1
	#define BTIF_RIS_RX_EOP_MASK 0x2000000
	#define BTIF_RIS_RX_EOP(_reg) (((_reg) & 0x2000000) >> 0x19)
	#define BTIF_RIS_RX_TMO_OFFSET 0x1f
	#define BTIF_RIS_RX_TMO_WIDTH 0x1
	#define BTIF_RIS_RX_TMO_MASK 0x80000000
	#define BTIF_RIS_RX_TMO(_reg) (((_reg) & 0x80000000) >> 0x1f)

#define BTIF_IMSC_OFFSET 0x84
#define BTIF_IMSC(_base) ((_base) + 0x84)
	#define BTIF_IMSC_IMSC_OFFSET 0x0
	#define BTIF_IMSC_IMSC_WIDTH 0x20
	#define BTIF_IMSC_IMSC_MASK 0xffffffff
	#define BTIF_IMSC_IMSC(_reg) (((_reg) & 0xffffffff) >> 0x0)
		#define BTIF_IMSC_IMSC_INT_DIS (0x0 << BTIF_IMSC_IMSC_OFFSET)
		#define BTIF_IMSC_IMSC_INT_EN (0x1 << BTIF_IMSC_IMSC_OFFSET)

#define BTIF_MIS_OFFSET 0x88
#define BTIF_MIS(_base) ((_base) + 0x88)
	#define BTIF_MIS_MIS_OFFSET 0x0
	#define BTIF_MIS_MIS_WIDTH 0x20
	#define BTIF_MIS_MIS_MASK 0xffffffff
	#define BTIF_MIS_MIS(_reg) (((_reg) & 0xffffffff) >> 0x0)
		#define BTIF_MIS_MIS_NO_INT (0x0 << BTIF_MIS_MIS_OFFSET)
		#define BTIF_MIS_MIS_INT_PEND (0x1 << BTIF_MIS_MIS_OFFSET)

#define BTIF_ISR_OFFSET 0x90
#define BTIF_ISR(_base) ((_base) + 0x90)
	#define BTIF_ISR_ISR_OFFSET 0x0
	#define BTIF_ISR_ISR_WIDTH 0x20
	#define BTIF_ISR_ISR_MASK 0xffffffff
	#define BTIF_ISR_ISR(_reg) (((_reg) & 0xffffffff) >> 0x0)
		#define BTIF_ISR_ISR_NO_CHANGE (0x0 << BTIF_ISR_ISR_OFFSET)
		#define BTIF_ISR_ISR_SET_IRQ (0x1 << BTIF_ISR_ISR_OFFSET)

#define BTIF_DMAE_OFFSET 0x94
#define BTIF_DMAE(_base) ((_base) + 0x94)
	#define BTIF_DMAE_RX_LSREQ_OFFSET 0x0
	#define BTIF_DMAE_RX_LSREQ_WIDTH 0x1
	#define BTIF_DMAE_RX_LSREQ_MASK 0x1
	#define BTIF_DMAE_RX_LSREQ(_reg) (((_reg) & 0x1) >> 0x0)
	#define BTIF_DMAE_RX_SREQ_OFFSET 0x1
	#define BTIF_DMAE_RX_SREQ_WIDTH 0x1
	#define BTIF_DMAE_RX_SREQ_MASK 0x2
	#define BTIF_DMAE_RX_SREQ(_reg) (((_reg) & 0x2) >> 0x1)
	#define BTIF_DMAE_RX_LBREQ_OFFSET 0x2
	#define BTIF_DMAE_RX_LBREQ_WIDTH 0x1
	#define BTIF_DMAE_RX_LBREQ_MASK 0x4
	#define BTIF_DMAE_RX_LBREQ(_reg) (((_reg) & 0x4) >> 0x2)
	#define BTIF_DMAE_RX_BREQ_OFFSET 0x3
	#define BTIF_DMAE_RX_BREQ_WIDTH 0x1
	#define BTIF_DMAE_RX_BREQ_MASK 0x8
	#define BTIF_DMAE_RX_BREQ(_reg) (((_reg) & 0x8) >> 0x3)
	#define BTIF_DMAE_TX_LSREQ_OFFSET 0x4
	#define BTIF_DMAE_TX_LSREQ_WIDTH 0x1
	#define BTIF_DMAE_TX_LSREQ_MASK 0x10
	#define BTIF_DMAE_TX_LSREQ(_reg) (((_reg) & 0x10) >> 0x4)
	#define BTIF_DMAE_TX_SREQ_OFFSET 0x5
	#define BTIF_DMAE_TX_SREQ_WIDTH 0x1
	#define BTIF_DMAE_TX_SREQ_MASK 0x20
	#define BTIF_DMAE_TX_SREQ(_reg) (((_reg) & 0x20) >> 0x5)
	#define BTIF_DMAE_TX_LBREQ_OFFSET 0x6
	#define BTIF_DMAE_TX_LBREQ_WIDTH 0x1
	#define BTIF_DMAE_TX_LBREQ_MASK 0x40
	#define BTIF_DMAE_TX_LBREQ(_reg) (((_reg) & 0x40) >> 0x6)
	#define BTIF_DMAE_TX_BREQ_OFFSET 0x7
	#define BTIF_DMAE_TX_BREQ_WIDTH 0x1
	#define BTIF_DMAE_TX_BREQ_MASK 0x80
	#define BTIF_DMAE_TX_BREQ(_reg) (((_reg) & 0x80) >> 0x7)

#define BTIF_ICR_OFFSET 0x98
#define BTIF_ICR(_base) ((_base) + 0x98)
	#define BTIF_ICR_ICR_OFFSET 0x0
	#define BTIF_ICR_ICR_WIDTH 0x20
	#define BTIF_ICR_ICR_MASK 0xffffffff
	#define BTIF_ICR_ICR(_reg) (((_reg) & 0xffffffff) >> 0x0)
		#define BTIF_ICR_ICR_NO_CHANGE (0x0 << BTIF_ICR_ICR_OFFSET)
		#define BTIF_ICR_ICR_CLR_INT (0x1 << BTIF_ICR_ICR_OFFSET)

#define BTIF_FIFO_CFG_OFFSET 0x30
#define BTIF_FIFO_CFG(_base) ((_base) + 0x30)
	#define BTIF_FIFO_CFG_RXBS_OFFSET 0x0
	#define BTIF_FIFO_CFG_RXBS_WIDTH 0x3
	#define BTIF_FIFO_CFG_RXBS_MASK 0x7
	#define BTIF_FIFO_CFG_RXBS(_reg) (((_reg) & 0x7) >> 0x0)
		#define BTIF_FIFO_CFG_RXBS_RXBS1 \
				(0x0 << BTIF_FIFO_CFG_RXBS_OFFSET)
		#define BTIF_FIFO_CFG_RXBS_RXBS2 \
				(0x1 << BTIF_FIFO_CFG_RXBS_OFFSET)
		#define BTIF_FIFO_CFG_RXBS_RXBS4 \
				(0x2 << BTIF_FIFO_CFG_RXBS_OFFSET)
		#define BTIF_FIFO_CFG_RXBS_RXBS8 \
				(0x3 << BTIF_FIFO_CFG_RXBS_OFFSET)
		#define BTIF_FIFO_CFG_RXBS_RXBS16 \
				(0x4 << BTIF_FIFO_CFG_RXBS_OFFSET)
		#define BTIF_FIFO_CFG_RXBS_RES \
				(0x5 << BTIF_FIFO_CFG_RXBS_OFFSET)
	#define BTIF_FIFO_CFG_RXFC_OFFSET 0x3
	#define BTIF_FIFO_CFG_RXFC_WIDTH 0x1
	#define BTIF_FIFO_CFG_RXFC_MASK 0x8
	#define BTIF_FIFO_CFG_RXFC(_reg) (((_reg) & 0x8) >> 0x3)
		#define BTIF_FIFO_CFG_RXFC_RXNFC \
				(0x0 << BTIF_FIFO_CFG_RXFC_OFFSET)
		#define BTIF_FIFO_CFG_RXFC_RXFC \
				(0x1 << BTIF_FIFO_CFG_RXFC_OFFSET)
	#define BTIF_FIFO_CFG_TXBS_OFFSET 0x4
	#define BTIF_FIFO_CFG_TXBS_WIDTH 0x3
	#define BTIF_FIFO_CFG_TXBS_MASK 0x70
	#define BTIF_FIFO_CFG_TXBS(_reg) (((_reg) & 0x70) >> 0x4)
		#define BTIF_FIFO_CFG_TXBS_TXBS1 \
				(0x0 << BTIF_FIFO_CFG_TXBS_OFFSET)
		#define BTIF_FIFO_CFG_TXBS_TXBS2 \
				(0x1 << BTIF_FIFO_CFG_TXBS_OFFSET)
		#define BTIF_FIFO_CFG_TXBS_TXBS4 \
				(0x2 << BTIF_FIFO_CFG_TXBS_OFFSET)
		#define BTIF_FIFO_CFG_TXBS_TXBS8 \
				(0x3 << BTIF_FIFO_CFG_TXBS_OFFSET)
		#define BTIF_FIFO_CFG_TXBS_TXBS16 \
				(0x4 << BTIF_FIFO_CFG_TXBS_OFFSET)
		#define BTIF_FIFO_CFG_TXBS_RES \
				(0x5 << BTIF_FIFO_CFG_TXBS_OFFSET)
	#define BTIF_FIFO_CFG_TXFC_OFFSET 0x7
	#define BTIF_FIFO_CFG_TXFC_WIDTH 0x1
	#define BTIF_FIFO_CFG_TXFC_MASK 0x80
	#define BTIF_FIFO_CFG_TXFC(_reg) (((_reg) & 0x80) >> 0x7)
		#define BTIF_FIFO_CFG_TXFC_TXNFC \
				(0x0 << BTIF_FIFO_CFG_TXFC_OFFSET)
		#define BTIF_FIFO_CFG_TXFC_TXFC \
				(0x1 << BTIF_FIFO_CFG_TXFC_OFFSET)
	#define BTIF_FIFO_CFG_RXFA_OFFSET 0x8
	#define BTIF_FIFO_CFG_RXFA_WIDTH 0x3
	#define BTIF_FIFO_CFG_RXFA_MASK 0x700
	#define BTIF_FIFO_CFG_RXFA(_reg) (((_reg) & 0x700) >> 0x8)
		#define BTIF_FIFO_CFG_RXFA_RXFA1 \
				(0x0 << BTIF_FIFO_CFG_RXFA_OFFSET)
		#define BTIF_FIFO_CFG_RXFA_RXFA2 \
				(0x1 << BTIF_FIFO_CFG_RXFA_OFFSET)
		#define BTIF_FIFO_CFG_RXFA_RXFA4 \
				(0x2 << BTIF_FIFO_CFG_RXFA_OFFSET)
		#define BTIF_FIFO_CFG_RXFA_RXFA8 \
				(0x3 << BTIF_FIFO_CFG_RXFA_OFFSET)
		#define BTIF_FIFO_CFG_RXFA_RES \
				(0x4 << BTIF_FIFO_CFG_RXFA_OFFSET)
	#define BTIF_FIFO_CFG_RX_SWAP_OFFSET 0xb
	#define BTIF_FIFO_CFG_RX_SWAP_WIDTH 0x1
	#define BTIF_FIFO_CFG_RX_SWAP_MASK 0x800
	#define BTIF_FIFO_CFG_RX_SWAP(_reg) (((_reg) & 0x800) >> 0xb)
		#define BTIF_FIFO_CFG_RX_SWAP_NO_RXSWAP \
				(0x0 << BTIF_FIFO_CFG_RX_SWAP_OFFSET)
		#define BTIF_FIFO_CFG_RX_SWAP_RXSWAP \
				(0x1 << BTIF_FIFO_CFG_RX_SWAP_OFFSET)
	#define BTIF_FIFO_CFG_TXFA_OFFSET 0xc
	#define BTIF_FIFO_CFG_TXFA_WIDTH 0x3
	#define BTIF_FIFO_CFG_TXFA_MASK 0x7000
	#define BTIF_FIFO_CFG_TXFA(_reg) (((_reg) & 0x7000) >> 0xc)
		#define BTIF_FIFO_CFG_TXFA_TXFA1 \
				(0x0 << BTIF_FIFO_CFG_TXFA_OFFSET)
		#define BTIF_FIFO_CFG_TXFA_TXFA2 \
				(0x1 << BTIF_FIFO_CFG_TXFA_OFFSET)
		#define BTIF_FIFO_CFG_TXFA_TXFA4 \
				(0x2 << BTIF_FIFO_CFG_TXFA_OFFSET)
		#define BTIF_FIFO_CFG_TXFA_TXFA8 \
				(0x3 << BTIF_FIFO_CFG_TXFA_OFFSET)
		#define BTIF_FIFO_CFG_TXFA_RES \
				(0x4 << BTIF_FIFO_CFG_TXFA_OFFSET)
	#define BTIF_FIFO_CFG_TX_SWAP_OFFSET 0xf
	#define BTIF_FIFO_CFG_TX_SWAP_WIDTH 0x1
	#define BTIF_FIFO_CFG_TX_SWAP_MASK 0x8000
	#define BTIF_FIFO_CFG_TX_SWAP(_reg) (((_reg) & 0x8000) >> 0xf)
		#define BTIF_FIFO_CFG_TX_SWAP_NO_TXSWAP \
				(0x0 << BTIF_FIFO_CFG_TX_SWAP_OFFSET)
		#define BTIF_FIFO_CFG_TX_SWAP_TXSWAP \
				(0x1 << BTIF_FIFO_CFG_TX_SWAP_OFFSET)
	#define BTIF_FIFO_CFG_RX_THR_OFFSET 0x10
	#define BTIF_FIFO_CFG_RX_THR_WIDTH 0x8
	#define BTIF_FIFO_CFG_RX_THR_MASK 0xff0000
	#define BTIF_FIFO_CFG_RX_THR(_reg) (((_reg) & 0xff0000) >> 0x10)
	#define BTIF_FIFO_CFG_TX_THR_OFFSET 0x18
	#define BTIF_FIFO_CFG_TX_THR_WIDTH 0x8
	#define BTIF_FIFO_CFG_TX_THR_MASK 0xff000000
	#define BTIF_FIFO_CFG_TX_THR(_reg) (((_reg) & 0xff000000) >> 0x18)

#define BTIF_FIFO_CTRL_OFFSET 0x34
#define BTIF_FIFO_CTRL(_base) ((_base) + 0x34)
	#define BTIF_FIFO_CTRL_TX_START_OFFSET 0x0
	#define BTIF_FIFO_CTRL_TX_START_WIDTH 0x1
	#define BTIF_FIFO_CTRL_TX_START_MASK 0x1
	#define BTIF_FIFO_CTRL_TX_START(_reg) (((_reg) & 0x1) >> 0x0)
	#define BTIF_FIFO_CTRL_TX_ABORT_OFFSET 0x1
	#define BTIF_FIFO_CTRL_TX_ABORT_WIDTH 0x1
	#define BTIF_FIFO_CTRL_TX_ABORT_MASK 0x2
	#define BTIF_FIFO_CTRL_TX_ABORT(_reg) (((_reg) & 0x2) >> 0x1)
	#define BTIF_FIFO_CTRL_TX_SETEOP_OFFSET 0x2
	#define BTIF_FIFO_CTRL_TX_SETEOP_WIDTH 0x1
	#define BTIF_FIFO_CTRL_TX_SETEOP_MASK 0x4
	#define BTIF_FIFO_CTRL_TX_SETEOP(_reg) (((_reg) & 0x4) >> 0x2)
	#define BTIF_FIFO_CTRL_RX_START_OFFSET 0x4
	#define BTIF_FIFO_CTRL_RX_START_WIDTH 0x1
	#define BTIF_FIFO_CTRL_RX_START_MASK 0x10
	#define BTIF_FIFO_CTRL_RX_START(_reg) (((_reg) & 0x10) >> 0x4)
	#define BTIF_FIFO_CTRL_RX_ABORT_OFFSET 0x5
	#define BTIF_FIFO_CTRL_RX_ABORT_WIDTH 0x1
	#define BTIF_FIFO_CTRL_RX_ABORT_MASK 0x20
	#define BTIF_FIFO_CTRL_RX_ABORT(_reg) (((_reg) & 0x20) >> 0x5)
	#define BTIF_FIFO_CTRL_RX_AR_OFFSET 0x6
	#define BTIF_FIFO_CTRL_RX_AR_WIDTH 0x2
	#define BTIF_FIFO_CTRL_RX_AR_MASK 0xc0
	#define BTIF_FIFO_CTRL_RX_AR(_reg) (((_reg) & 0xc0) >> 0x6)
		#define BTIF_FIFO_CTRL_RX_AR_AR_OFF \
				(0x2 << BTIF_FIFO_CTRL_RX_AR_OFFSET)
		#define BTIF_FIFO_CTRL_RX_AR_AR_ON \
				(0x1 << BTIF_FIFO_CTRL_RX_AR_OFFSET)
		#define BTIF_FIFO_CTRL_RX_AR_NOCHANGE \
				(0x0 << BTIF_FIFO_CTRL_RX_AR_OFFSET)
		#define BTIF_FIFO_CTRL_RX_AR_NO_CHANGE \
				(0x3 << BTIF_FIFO_CTRL_RX_AR_OFFSET)

#define BTIF_MRPS_CTRL_OFFSET 0x38
#define BTIF_MRPS_CTRL(_base) ((_base) + 0x38)
	#define BTIF_MRPS_CTRL_MRPS_OFFSET 0x0
	#define BTIF_MRPS_CTRL_MRPS_WIDTH 0x10
	#define BTIF_MRPS_CTRL_MRPS_MASK 0xffff
	#define BTIF_MRPS_CTRL_MRPS(_reg) (((_reg) & 0xffff) >> 0x0)

#define BTIF_RPS_STAT_OFFSET 0x3c
#define BTIF_RPS_STAT(_base) ((_base) + 0x3c)
	#define BTIF_RPS_STAT_RPS_OFFSET 0x0
	#define BTIF_RPS_STAT_RPS_WIDTH 0x10
	#define BTIF_RPS_STAT_RPS_MASK 0xffff
	#define BTIF_RPS_STAT_RPS(_reg) (((_reg) & 0xffff) >> 0x0)
	#define BTIF_RPS_STAT_FIFO_FULL_IND_OFFSET 0x1e
	#define BTIF_RPS_STAT_FIFO_FULL_IND_WIDTH 0x1
	#define BTIF_RPS_STAT_FIFO_FULL_IND_MASK 0x40000000
	#define BTIF_RPS_STAT_FIFO_FULL_IND(_reg) \
			(((_reg) & 0x40000000) >> 0x1e)
		#define BTIF_RPS_STAT_FIFO_FULL_IND_FIFO_FULL_IND \
				(0x1 << BTIF_RPS_STAT_FIFO_FULL_IND_OFFSET)
		#define BTIF_RPS_STAT_FIFO_FULL_IND_NO_FIFO_FULL \
				(0x0 << BTIF_RPS_STAT_FIFO_FULL_IND_OFFSET)
	#define BTIF_RPS_STAT_MRPS_STOP_IND_OFFSET 0x1f
	#define BTIF_RPS_STAT_MRPS_STOP_IND_WIDTH 0x1
	#define BTIF_RPS_STAT_MRPS_STOP_IND_MASK 0x80000000
	#define BTIF_RPS_STAT_MRPS_STOP_IND(_reg) \
			(((_reg) & 0x80000000) >> 0x1f)
		#define BTIF_RPS_STAT_MRPS_STOP_IND_MRPS_STOP_IND \
				(0x1 << BTIF_RPS_STAT_MRPS_STOP_IND_OFFSET)
		#define BTIF_RPS_STAT_MRPS_STOP_IND_NO_MRPS_STOP \
				(0x0 << BTIF_RPS_STAT_MRPS_STOP_IND_OFFSET)

#define BTIF_TPS_CTRL_OFFSET 0x40
#define BTIF_TPS_CTRL(_base) ((_base) + 0x40)
	#define BTIF_TPS_CTRL_TPS_OFFSET 0x0
	#define BTIF_TPS_CTRL_TPS_WIDTH 0x10
	#define BTIF_TPS_CTRL_TPS_MASK 0xffff
	#define BTIF_TPS_CTRL_TPS(_reg) (((_reg) & 0xffff) >> 0x0)

#define BTIF_FIFO_STAT_OFFSET 0x44
#define BTIF_FIFO_STAT(_base) ((_base) + 0x44)
	#define BTIF_FIFO_STAT_RXFFS_OFFSET 0x0
	#define BTIF_FIFO_STAT_RXFFS_WIDTH 0x8
	#define BTIF_FIFO_STAT_RXFFS_MASK 0xff
	#define BTIF_FIFO_STAT_RXFFS(_reg) (((_reg) & 0xff) >> 0x0)
	#define BTIF_FIFO_STAT_RX_AR_OFFSET 0xc
	#define BTIF_FIFO_STAT_RX_AR_WIDTH 0x1
	#define BTIF_FIFO_STAT_RX_AR_MASK 0x1000
	#define BTIF_FIFO_STAT_RX_AR(_reg) (((_reg) & 0x1000) >> 0xc)
		#define BTIF_FIFO_STAT_RX_AR_AR_OFF \
				(0x0 << BTIF_FIFO_STAT_RX_AR_OFFSET)
		#define BTIF_FIFO_STAT_RX_AR_AR_ON \
				(0x1 << BTIF_FIFO_STAT_RX_AR_OFFSET)
	#define BTIF_FIFO_STAT_RX_EOP_OFFSET 0xd
	#define BTIF_FIFO_STAT_RX_EOP_WIDTH 0x1
	#define BTIF_FIFO_STAT_RX_EOP_MASK 0x2000
	#define BTIF_FIFO_STAT_RX_EOP(_reg) (((_reg) & 0x2000) >> 0xd)
		#define BTIF_FIFO_STAT_RX_EOP_RX_EOP \
				(0x1 << BTIF_FIFO_STAT_RX_EOP_OFFSET)
		#define BTIF_FIFO_STAT_RX_EOP_RX_NOEOP \
				(0x0 << BTIF_FIFO_STAT_RX_EOP_OFFSET)
	#define BTIF_FIFO_STAT_RXTHR_EXC_OFFSET 0xe
	#define BTIF_FIFO_STAT_RXTHR_EXC_WIDTH 0x1
	#define BTIF_FIFO_STAT_RXTHR_EXC_MASK 0x4000
	#define BTIF_FIFO_STAT_RXTHR_EXC(_reg) (((_reg) & 0x4000) >> 0xe)
		#define BTIF_FIFO_STAT_RXTHR_EXC_RXTHR_NOTREACHED \
				(0x0 << BTIF_FIFO_STAT_RXTHR_EXC_OFFSET)
		#define BTIF_FIFO_STAT_RXTHR_EXC_RXTHR_EXCEEDED \
				(0x1 << BTIF_FIFO_STAT_RXTHR_EXC_OFFSET)
	#define BTIF_FIFO_STAT_MRPS_WE_OFFSET 0xf
	#define BTIF_FIFO_STAT_MRPS_WE_WIDTH 0x1
	#define BTIF_FIFO_STAT_MRPS_WE_MASK 0x8000
	#define BTIF_FIFO_STAT_MRPS_WE(_reg) (((_reg) & 0x8000) >> 0xf)
		#define BTIF_FIFO_STAT_MRPS_WE_MRPS_BUSY \
				(0x0 << BTIF_FIFO_STAT_MRPS_WE_OFFSET)
		#define BTIF_FIFO_STAT_MRPS_WE_MRPS_READY \
				(0x1 << BTIF_FIFO_STAT_MRPS_WE_OFFSET)
	#define BTIF_FIFO_STAT_TXFFS_OFFSET 0x10
	#define BTIF_FIFO_STAT_TXFFS_WIDTH 0x8
	#define BTIF_FIFO_STAT_TXFFS_MASK 0xff0000
	#define BTIF_FIFO_STAT_TXFFS(_reg) (((_reg) & 0xff0000) >> 0x10)
	#define BTIF_FIFO_STAT_TXTHR_EXC_OFFSET 0x1e
	#define BTIF_FIFO_STAT_TXTHR_EXC_WIDTH 0x1
	#define BTIF_FIFO_STAT_TXTHR_EXC_MASK 0x40000000
	#define BTIF_FIFO_STAT_TXTHR_EXC(_reg) (((_reg) & 0x40000000) >> 0x1e)
		#define BTIF_FIFO_STAT_TXTHR_EXC_TXTHR_NOTREACHED \
				(0x0 << BTIF_FIFO_STAT_TXTHR_EXC_OFFSET)
		#define BTIF_FIFO_STAT_TXTHR_EXC_TXTHR_EXCEEDED \
				(0x1 << BTIF_FIFO_STAT_TXTHR_EXC_OFFSET)
	#define BTIF_FIFO_STAT_TPS_WE_OFFSET 0x1f
	#define BTIF_FIFO_STAT_TPS_WE_WIDTH 0x1
	#define BTIF_FIFO_STAT_TPS_WE_MASK 0x80000000
	#define BTIF_FIFO_STAT_TPS_WE(_reg) (((_reg) & 0x80000000) >> 0x1f)
		#define BTIF_FIFO_STAT_TPS_WE_TPS_BUSY \
				(0x0 << BTIF_FIFO_STAT_TPS_WE_OFFSET)
		#define BTIF_FIFO_STAT_TPS_WE_TPS_READY \
				(0x1 << BTIF_FIFO_STAT_TPS_WE_OFFSET)

#define BTIF_TXD_OFFSET 0x40000
#define BTIF_TXD(_base) ((_base) + 0x40000)
	#define BTIF_TXD_TXD_OFFSET 0x0
	#define BTIF_TXD_TXD_WIDTH 0x20
	#define BTIF_TXD_TXD_MASK 0xffffffff
	#define BTIF_TXD_TXD(_reg) (((_reg) & 0xffffffff) >> 0x0)

#define BTIF_RXD_OFFSET 0x80000
#define BTIF_RXD(_base) ((_base) + 0x80000)
	#define BTIF_RXD_RXD_OFFSET 0x0
	#define BTIF_RXD_RXD_WIDTH 0x20
	#define BTIF_RXD_RXD_MASK 0xffffffff
	#define BTIF_RXD_RXD(_reg) (((_reg) & 0xffffffff) >> 0x0)

#define BTIF_MSS_SET_OFFSET 0x100
#define BTIF_MSS_SET(_base) ((_base) + 0x100)
	#define BTIF_MSS_SET_FCOSET_OFFSET 0x0
	#define BTIF_MSS_SET_FCOSET_WIDTH 0x1
	#define BTIF_MSS_SET_FCOSET_MASK 0x1
	#define BTIF_MSS_SET_FCOSET(_reg) (((_reg) & 0x1) >> 0x0)
		#define BTIF_MSS_SET_FCOSET_NE \
				(0x0 << BTIF_MSS_SET_FCOSET_OFFSET)
		#define BTIF_MSS_SET_FCOSET_FCO \
				(0x1 << BTIF_MSS_SET_FCOSET_OFFSET)

#define BTIF_MSS_CLR_OFFSET 0x104
#define BTIF_MSS_CLR(_base) ((_base) + 0x104)
	#define BTIF_MSS_CLR_FCOCLR_OFFSET 0x0
	#define BTIF_MSS_CLR_FCOCLR_WIDTH 0x1
	#define BTIF_MSS_CLR_FCOCLR_MASK 0x1
	#define BTIF_MSS_CLR_FCOCLR(_reg) (((_reg) & 0x1) >> 0x0)
		#define BTIF_MSS_CLR_FCOCLR_NE \
				(0x0 << BTIF_MSS_CLR_FCOCLR_OFFSET)
		#define BTIF_MSS_CLR_FCOCLR_FCO \
				(0x1 << BTIF_MSS_CLR_FCOCLR_OFFSET)

#define BTIF_MSS_STAT_OFFSET 0x108
#define BTIF_MSS_STAT(_base) ((_base) + 0x108)
	#define BTIF_MSS_STAT_FCO_OFFSET 0x0
	#define BTIF_MSS_STAT_FCO_WIDTH 0x1
	#define BTIF_MSS_STAT_FCO_MASK 0x1
	#define BTIF_MSS_STAT_FCO(_reg) (((_reg) & 0x1) >> 0x0)
		#define BTIF_MSS_STAT_FCO_INACT \
				(0x0 << BTIF_MSS_STAT_FCO_OFFSET)
		#define BTIF_MSS_STAT_FCO_ACT (0x1 << BTIF_MSS_STAT_FCO_OFFSET)
	#define BTIF_MSS_STAT_FCI_OFFSET 0x1
	#define BTIF_MSS_STAT_FCI_WIDTH 0x1
	#define BTIF_MSS_STAT_FCI_MASK 0x2
	#define BTIF_MSS_STAT_FCI(_reg) (((_reg) & 0x2) >> 0x1)
		#define BTIF_MSS_STAT_FCI_INACT \
				(0x0 << BTIF_MSS_STAT_FCI_OFFSET)
		#define BTIF_MSS_STAT_FCI_ACT (0x1 << BTIF_MSS_STAT_FCI_OFFSET)

#define BTIF_TMO_CFG_OFFSET 0x148
#define BTIF_TMO_CFG(_base) ((_base) + 0x148)
	#define BTIF_TMO_CFG_TMO_OFFSET 0x0
	#define BTIF_TMO_CFG_TMO_WIDTH 0x10
	#define BTIF_TMO_CFG_TMO_MASK 0xffff
	#define BTIF_TMO_CFG_TMO(_reg) (((_reg) & 0xffff) >> 0x0)

