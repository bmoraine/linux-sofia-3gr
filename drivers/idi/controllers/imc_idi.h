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

#ifndef _IDI_IMC_H_
#define _IDI_IMC_H_

#define IMC_IDI_MAX_CHANNELS (16)	/* move to imc_mid_idi.h */
#define IMC_IDI_MAX_IRQS (8)

/*
 * Register base addresses
 * These registers are (mostly) in the order found in the
 * ipdb_idi_arch_master.pdf document.
 */
#define IMC_IDI_ID(_base)              ((_base) + 0x0C)
#define IMC_IDI_SRB_MSCONF_ID(_base)   ((_base) + 0x14)
#define IMC_IDI_SRB_ERRCONF_ID(_base)  ((_base) + 0x18)
#define IMC_IDI_SWCID(_base)           ((_base) + 0x1C)

#define IMC_IDI_CLC(_base)             ((_base) + 0x00)
#define IMC_IDI_CLC_CNT(_base)         ((_base) + 0x04)
#define IMC_IDI_CLC_STAT(_base)        ((_base) + 0x08)

#define IMC_IDI_RIS(_base)             ((_base) + 0x80)
#define IMC_IDI_IMSC(_base)            ((_base) + 0x84)
#define IMC_IDI_MIS(_base)             ((_base) + 0x88)
#define IMC_IDI_ISR(_base)             ((_base) + 0x90)
#define IMC_IDI_ICR(_base)             ((_base) + 0x98)

#define IMC_IDI_RIS1(_base)            ((_base) + 0xA0)
#define IMC_IDI_IMSC1(_base)           ((_base) + 0xA4)
#define IMC_IDI_MIS1(_base)            ((_base) + 0xA8)
#define IMC_IDI_ISR1(_base)            ((_base) + 0xB0)
#define IMC_IDI_ICR1(_base)            ((_base) + 0xB8)

#define IMC_IDI_TXCONF(_base)          ((_base) + 0x100)
#define IMC_IDI_TXSTAT(_base)          ((_base) + 0x104)
#define IMC_IDI_TXCON(_base)           ((_base) + 0x108)
#define IMC_IDI_TXCON2(_base)          ((_base) + 0x10C)
#define IMC_IDI_TXD(_base)             ((_base) + 0x110)
#define IMC_IDI_TXDEBUG(_base)         ((_base) + 0x114)
#define IMC_IDI_TXIRQ_STAT(_base)      ((_base) + 0x118)
#define IMC_IDI_TXMASK_CON(_base)      ((_base) + 0x11C)

#define IMC_IDI_TX2MASK_CON(_base)     ((_base) + 0x170)
#define IMC_IDI_TX3MASK_CON(_base)     ((_base) + 0x174)

#define IMC_IDI_TXIRQ_CON(_base)       ((_base) + 0x120)

#define IMC_IDI_RXCONF(_base)          ((_base) + 0x130)
#define IMC_IDI_RXSTAT(_base)          ((_base) + 0x134)
#define IMC_IDI_RXCON(_base)           ((_base) + 0x138)
#define IMC_IDI_RXCON2(_base)          ((_base) + 0x13C)
#define IMC_IDI_RXD(_base)             ((_base) + 0x140)
#define IMC_IDI_RXDEBUG(_base)         ((_base) + 0x144)
#define IMC_IDI_RXIRQ_STAT(_base)      ((_base) + 0x148)
#define IMC_IDI_RXMASK_CON(_base)      ((_base) + 0x14C)

#define IMC_IDI_RX2MASK_CON(_base)     ((_base) + 0x178)
#define IMC_IDI_RX3MASK_CON(_base)     ((_base) + 0x17C)

#define IMC_IDI_RXIRQ_CON(_base)       ((_base) + 0x150)

#define IMC_IDI_ERR_CON(_base)         ((_base) + 0x160)

#define IMC_IDI_DEBUG5(_base)          ((_base) + 0x190)
#define IMC_IDI_DEBUG6(_base)          ((_base) + 0x194)
#define IMC_IDI_DEBUG7(_base)          ((_base) + 0x198)
#define IMC_IDI_DEBUG8(_base)          ((_base) + 0x19C)
#define IMC_IDI_DEBUG9(_base)          ((_base) + 0x1A0)
#define IMC_IDI_DEBUG10(_base)         ((_base) + 0x1A4)

/*
 * This regsiter has two different offsets in the document
 * (ipdb_idi_arch_mast.pdf).  Going to use the one in the main part of the doc
 * for now.
 *
 * #define IMC_IDI_EXT_CON(_base)         ((_base) + 0x180)
 */
#define IMC_IDI_EXT_CON(_base)         ((_base) + 0x15C)
#define IMC_IDI_CHANNEL_CON(_base)     ((_base) + 0x154)
#define IMC_IDI_SLMASK_CON(_base)      ((_base) + 0x168)
#define IMC_IDI_SL2MASK_CON(_base)     ((_base) + 0x180)

/*
 * The following registers are referenced via base and channel
 */
#define reg_offset(_channel) ((_channel * 2) << 4)

#define IMC_IDI_TXCH_BASE(_base, _channel)     ((_base) + 0x400 +	\
							 reg_offset((_channel)))
#define IMC_IDI_TXCH_SIZE(_base, _channel)     ((_base) + 0x404 +	\
							reg_offset((_channel)))
#define IMC_IDI_TXCH_RD_STAT(_base, _channel)  ((_base) + 0x408 +	\
							reg_offset((_channel)))
#define IMC_IDI_TXCH_WR_CON(_base, _channel)   ((_base) + 0x40C +	\
							reg_offset((_channel)))
#define IMC_IDI_TXCH_IRQ_CON(_base, _channel)  ((_base) + 0x410 +	\
							reg_offset((_channel)))
#define IMC_IDI_TXCH_NEXT(_base, _channel)     ((_base) + 0x414 +	\
							reg_offset((_channel)))
#define IMC_IDI_RXCH_BASE(_base, _channel)     ((_base) + 0x600 +	\
							reg_offset((_channel)))
#define IMC_IDI_RXCH_SIZE(_base, _channel)     ((_base) + 0x604 +	\
							reg_offset((_channel)))
#define IMC_IDI_RXCH_WR_STAT(_base, _channel)  ((_base) + 0x608 +	\
							reg_offset((_channel)))
#define IMC_IDI_RXCH_RD_CON(_base, _channel)   ((_base) + 0x60C +	\
							reg_offset((_channel)))
#define IMC_IDI_RXCH_IRQ_CON(_base, _channel)  ((_base) + 0x610 +	\
							reg_offset((_channel)))
#define IMC_IDI_RXCH_NEXT(_base, _channel)     ((_base) + 0x614 +	\
							reg_offset((_channel)))

/*
 * Register bits.
 */
#define IMC_IDI_ID_REV_NUMBER_MASK   (0x000000FF)	/* bits 7:0 */
#define IMC_IDI_ID_MOD_ID_MASK       (0x0000FF00)	/* bits 15:8 */
#define IMC_IDI_ID_TS_REV_NR_MASK    (0xFFFF0000)	/* bits 31:16 */

#define IMC_IDI_ID_REV_NUMBER_SHIFT    (0)
#define IMC_IDI_ID_MOD_ID_SHIFT        (8)
#define IMC_IDI_ID_TS_REV_NR_SHIFT    (16)

#define IMC_IDI_ID_REV_NUMBER(_reg) (((_reg) &	\
					IMC_IDI_ID_REV_NUMBER_MASK) >>	\
					IMC_IDI_ID_REV_NUMBER_SHIFT)

#define IMC_IDI_ID_MOD_ID(_reg)     (((_reg) &	\
					IMC_IDI_ID_MOD_ID_MASK) >>	\
					IMC_IDI_ID_MOD_ID_SHIFT)

#define IMC_IDI_ID_TS_REV_NR(_reg)  (((_reg) &	\
					IMC_IDI_ID_TS_REV_NR_MASK) >>	\
					IMC_IDI_ID_TS_REV_NR_SHIFT)

#define IMC_IDI_SWCID_CH            (1 << 0)

#define IMC_IDI_CLC_RUN(_bits)      (_bits)	/* bits 1:0 */
#define IMC_IDI_CLC_MOD_EN(_bits)   ((_bits) << 2)	/* bits 3:2 */
#define IMC_IDI_CLC_SPEN(_bits)     ((_bits) << 4)	/* bits 5:4 */
#define IMC_IDI_CLC_FSOE(_bits)     ((_bits) << 6)	/* bits 7:6 */
#define IMC_IDI_CLC_EDREN(_bits)    ((_bits) << 8)	/* bits 9:8 */
#define IMC_IDI_CLC_IDREN(_bits)    ((_bits) << 10)	/* bits 11:10 */

/*
 * IMC_IDI_CLC register takes two bit commands.
 */
#define IMC_IDI_CLC_IDREN_IDRDIS   (2)
#define IMC_IDI_CLC_IDREN_IDREN    (1)
#define IMC_IDI_CLC_EDREN_EDRDIS   (2)
#define IMC_IDI_CLC_EDREN_EDREN    (1)
#define IMC_IDI_CLC_FSOE_FSOE_DIS  (2)
#define IMC_IDI_CLC_FSOE_FSOE_EN   (1)
#define IMC_IDI_CLC_SPEN_SPEN_DIS  (2)
#define IMC_IDI_CLC_SPEN_SPEN_EN   (1)
#define IMC_IDI_CLC_MOD_EN_DIS_REQ (2)
#define IMC_IDI_CLC_MOD_EN_EN_REQ  (1)
#define IMC_IDI_CLC_RUN_STOP       (2)
#define IMC_IDI_CLC_RUN_RUN        (1)

#define IMC_IDI_CLC_CNT_RMC(_bits)  (_bits)	/* bits 7:0 */
#define IMC_IDI_CLC_CNT_ORMC(_bits) ((_bits) << 8)	/* bits 15:8 */

#define IMC_IDI_CLC_CNT_MODULE_DISABLE (0)
#define IMC_IDI_CLC_CNT_RMC_MASK       (0x000000FF)

#define IMC_IDI_CLC_STAT_RUN      (1 << 0)
#define IMC_IDI_CLC_STAT_CONFIG   (0)
#define IMC_IDI_CLC_STAT_MODEN    (1 << 1)
#define IMC_IDI_CLC_STAT_SPEN     (1 << 2)
#define IMC_IDI_CLC_STAT_FSOE     (1 << 3)
#define IMC_IDI_CLC_STAT_EDRE     (1 << 4)
#define IMC_IDI_CLC_STAT_IDRE     (1 << 5)
#define IMC_IDI_CLC_STAT_CUOK     (1 << 6)
#define IMC_IDI_CLC_STAT_KID      (1 << 7)
#define IMC_IDI_CLC_STAT_FIFOID   (1 << 8)

#define IMC_IDI_RIS_RX            (1 << 0)
#define IMC_IDI_RIS_TX            (1 << 1)
#define IMC_IDI_RIS_WAKE          (1 << 2)
#define IMC_IDI_RIS_IDLE          (1 << 3)
#define IMC_IDI_RIS_TIME          (1 << 4)
#define IMC_IDI_RIS_BREAK         (1 << 5)
#define IMC_IDI_RIS_STALL         (1 << 6)
#define IMC_IDI_RIS_TRAIL         (1 << 7)
#define IMC_IDI_RIS_MASTER        (1 << 8)
#define IMC_IDI_RIS_ADDR          (1 << 9)
#define IMC_IDI_RIS_ZERO          (1 << 10)
#define IMC_IDI_RIS_RX2           (1 << 11)
#define IMC_IDI_RIS_RX3           (1 << 12)
#define IMC_IDI_RIS_TX2           (1 << 13)
#define IMC_IDI_RIS_TX3           (1 << 14)

/*
 * The following macro(s) (_RIS_SLAVE(), _IMSC_SLAVE() etc) selects the given
 * IRQ bit from the Slave interrupt portion of various interrupt registers.
 * There are 32 possible forwared slave interrupts.  0 - 16 are in the original
 * register (bits 31:15).  17 - 31 are in the second register (xxx1) at bits
 * 14:0
 */
#define IMC_IDI_RIS_SLAVE(_irq)   (((_irq) < 17) ?	\
					(1 << ((_irq) + 15)) :	\
					(1 << ((_irq) - 17)))

#define IMC_IDI_IMSC_RX           (1 << 0)
#define IMC_IDI_IMSC_TX           (1 << 1)
#define IMC_IDI_IMSC_WAKE         (1 << 2)
#define IMC_IDI_IMSC_IDLE         (1 << 3)
#define IMC_IDI_IMSC_TIME         (1 << 4)
#define IMC_IDI_IMSC_BREAK        (1 << 5)
#define IMC_IDI_IMSC_STALL        (1 << 6)
#define IMC_IDI_IMSC_TRAIL        (1 << 7)
#define IMC_IDI_IMSC_MASTER       (1 << 8)
#define IMC_IDI_IMSC_ADDR         (1 << 9)
#define IMC_IDI_IMSC_ZERO         (1 << 10)
#define IMC_IDI_IMSC_RX2          (1 << 11)
#define IMC_IDI_IMSC_RX3          (1 << 12)
#define IMC_IDI_IMSC_TX2          (1 << 13)
#define IMC_IDI_IMSC_TX3          (1 << 14)

#define IMC_IDI_IMSC_SLAVE(_irq)  (((_irq) < 17) ?	\
					(1 << ((_irq) + 15)) :	\
					(1 << ((_irq) - 17)))

#define IMSC_SLAVE_MASK           (0xFFFF8000)
#define IMSC1_SLAVE_MASK          (0x00007FFF)

#define IMC_IDI_MIS_RX            (1 << 0)
#define IMC_IDI_MIS_TX            (1 << 1)
#define IMC_IDI_MIS_WAKE          (1 << 2)
#define IMC_IDI_MIS_IDLE          (1 << 3)
#define IMC_IDI_MIS_TIME          (1 << 4)
#define IMC_IDI_MIS_BREAK         (1 << 5)
#define IMC_IDI_MIS_STALL         (1 << 6)
#define IMC_IDI_MIS_TRAIL         (1 << 7)
#define IMC_IDI_MIS_MASTER        (1 << 8)
#define IMC_IDI_MIS_ADDR          (1 << 9)
#define IMC_IDI_MIS_ZERO          (1 << 10)
#define IMC_IDI_MIS_RX2           (1 << 11)
#define IMC_IDI_MIS_RX3           (1 << 12)
#define IMC_IDI_MIS_TX2           (1 << 13)
#define IMC_IDI_MIS_TX3           (1 << 14)

#define IMC_IDI_ERR_MASK (IMC_IDI_MIS_TIME   | IMC_IDI_MIS_BREAK |	\
			IMC_IDI_MIS_STALL  | IMC_IDI_MIS_TRAIL |	\
			IMC_IDI_MIS_MASTER | IMC_IDI_MIS_ADDR  |	\
			IMC_IDI_MIS_ZERO)

#define IMC_IDI_RX_ERR_MASK (IMC_IDI_MIS_TIME  | IMC_IDI_MIS_STALL |	\
				  IMC_IDI_MIS_TRAIL | IMC_IDI_MIS_ZERO)

#define IMC_IDI_MIS_SLAVE(_irq)  (((_irq) < 17) ?	\
					(1 << ((_irq) + 15)) :	\
					(1 << ((_irq) - 17)))

#define IMC_IDI_ISR_RX           (1 << 0)
#define IMC_IDI_ISR_TX           (1 << 1)
#define IMC_IDI_ISR_WAKE         (1 << 2)
#define IMC_IDI_ISR_IDLE         (1 << 3)
#define IMC_IDI_ISR_TIME         (1 << 4)
#define IMC_IDI_ISR_BREAK        (1 << 5)
#define IMC_IDI_ISR_STALL        (1 << 6)
#define IMC_IDI_ISR_TRAIL        (1 << 7)
#define IMC_IDI_ISR_MASTER       (1 << 8)
#define IMC_IDI_ISR_ADDR         (1 << 9)
#define IMC_IDI_ISR_ZERO         (1 << 10)
#define IMC_IDI_ISR_RX2          (1 << 11)
#define IMC_IDI_ISR_RX3          (1 << 12)
#define IMC_IDI_ISR_TX2          (1 << 13)
#define IMC_IDI_ISR_TX3          (1 << 14)

#define IMC_IDI_ISR_SLAVE(_irq)  (((_irq) < 17) ?	\
					(1 << ((_irq) + 15)) :	\
					(1 << ((_irq) - 17)))

#define IMC_IDI_ICR_RX           (1 << 0)
#define IMC_IDI_ICR_TX           (1 << 1)
#define IMC_IDI_ICR_WAKE         (1 << 2)
#define IMC_IDI_ICR_IDLE         (1 << 3)
#define IMC_IDI_ICR_TIME         (1 << 4)
#define IMC_IDI_ICR_BREAK        (1 << 5)
#define IMC_IDI_ICR_STALL        (1 << 6)
#define IMC_IDI_ICR_TRAIL        (1 << 7)
#define IMC_IDI_ICR_MASTER       (1 << 8)
#define IMC_IDI_ICR_ADDR         (1 << 9)
#define IMC_IDI_ICR_ZERO         (1 << 10)
#define IMC_IDI_ICR_RX2          (1 << 11)
#define IMC_IDI_ICR_RX3          (1 << 12)
#define IMC_IDI_ICR_TX2          (1 << 13)
#define IMC_IDI_ICR_TX3          (1 << 14)

#define IMC_IDI_ICR_SLAVE(_irq)      (((_irq) < 17) ?	\
					(1 << ((_irq) + 15)) :	\
					(1 << ((_irq) - 17)))

#define IMC_IDI_TXCONF_DESC(_bits)   ((_bits) << 0)
#define IMC_IDI_TXCONF_TRANS         (1 << 3)
#define IMC_IDI_TXCONF_BREAK(_bits)  ((_bits) << 4)
#define IMC_IDI_TXCONF_PAD           (1 << 7)

#define IMC_IDI_TXCONF_DESC_ZERO    (0)
#define IMC_IDI_TXCONF_DESC_ONE     (1)
#define IMC_IDI_TXCONF_DESC_TWO     (2)
#define IMC_IDI_TXCONF_DESC_THREE   (3)
#define IMC_IDI_TXCONF_DESC_FOUR    (4)

#define IMC_IDI_TXCONF_BREAK_33     (0)
#define IMC_IDI_TXCONF_BREAK_34     (1)
#define IMC_IDI_TXCONF_BREAK_35     (2)
#define IMC_IDI_TXCONF_BREAK_36     (3)
#define IMC_IDI_TXCONF_BREAK_37     (4)

#define IMC_IDI_TXSTAT_READY        (1 << 0)
#define IMC_IDI_TXSTAT_BSY_LOG      (1 << 1)
#define IMC_IDI_TXSTAT_BSY_LNK      (1 << 2)
#define IMC_IDI_TXSTAT_BSY_CH       (1 << 3)
#define IMC_IDI_TXSTAT_WORD         (1 << 4)

#define IMC_IDI_TXSTAT_BSY_MASK (			\
			IMC_IDI_TXSTAT_BSY_CH |		\
			IMC_IDI_TXSTAT_BSY_LNK |	\
			IMC_IDI_TXSTAT_BSY_LOG)

#define IMC_IDI_TXCON_EN_LNK           (1 << 0)
#define IMC_IDI_TXCON_PAUSED           (0)
#define IMC_IDI_TXCON_EN_CH            (1 << 1)
#define IMC_IDI_TXCON_N(_bits)         ((_bits) << 2)
#define IMC_IDI_TXCON_CHANNEL(_bits)   ((_bits) << 8)
#define IMC_IDI_TXCON_MASTER           (1 << 15)
#define IMC_IDI_TXCON_PRIORITY(_bits)  ((_bits) << 16)

/*
 * Should be defined by the client device?
 */
#define IMC_IDI_STREAM_CHANNELS  (0x00E0)	/* channel 5, 6, 7 */

#define IMC_IDI_TXCON2_CLR_LOG         (1 << 0)
#define IMC_IDI_TXCON2_CLR_CH          (1 << 1)
#define IMC_IDI_TXCON2_BREAK           (1 << 2)
#define IMC_IDI_TXCON2_EXT             (1 << 3)

#define IMC_IDI_TXIRQ_STAT_IRQ(_chan)   (1 << (_chan))

#define IMC_IDI_TXMASK_CON_IRQ(_chan)   (1 << (_chan))
#define IMC_IDI_TX2MASK_CON_IRQ(_chan)  (1 << (_chan))
#define IMC_IDI_TX3MASK_CON_IRQ(_chan)  (1 << (_chan))

#define IMC_IDI_TXIRQ_CON_CLR(_chan)    (1 << (_chan))

#define IMC_IDI_TXIRQ_CON_CLR_ALL       (0xFFFF)
#define IMC_IDI_RXIRQ_CON_CLR_ALL       (0xFFFF)

#define IMC_IDI_RXCONF_WAKE_WAKE        (1 << 0)
#define IMC_IDI_RXCONF_TRANS_FRAME      (1 << 1)
#define IMC_IDI_RXCONF_DESC(_bits)      ((_bits) << 2)
#define IMC_IDI_RXCONF_BREAK(_bits)     ((_bits) << 5)
#define IMC_IDI_RXCONF_LOOP_LOOPBACK    (1 << 8)

#define IMC_IDI_RXCONF_DESC_ZERO        (0)
#define IMC_IDI_RXCONF_DESC_ONE         (1)
#define IMC_IDI_RXCONF_DESC_TWO         (2)
#define IMC_IDI_RXCONF_DESC_THREE       (3)
#define IMC_IDI_RXCONF_DESC_FOUR        (4)

#define IMC_IDI_RXCONF_BREAK_33         (0)
#define IMC_IDI_RXCONF_BREAK_34         (1)
#define IMC_IDI_RXCONF_BREAK_35         (2)
#define IMC_IDI_RXCONF_BREAK_36         (3)
#define IMC_IDI_RXCONF_BREAK_37         (4)

/* ACWAKE is asserted */
#define IMC_IDI_RXSTAT_WAKE            (1 << 0)
/* Data being received on the logic or link layers */
#define IMC_IDI_RXSTAT_BSY             (1 << 1)
/* Data is being written to memory */
#define IMC_IDI_RXSTAT_BSY_CH          (1 << 2)
/* Data word is available. */
#define IMC_IDI_RXSTAT_WORD            (1 << 3)
#define IMC_IDI_RXSTAT_CHANNEL_SHIFT   (4)

#define IMC_IDI_RXSTAT_BSY_MASK        (IMC_IDI_RXSTAT_WAKE   |	\
					IMC_IDI_RXSTAT_BSY    |	\
					IMC_IDI_RXSTAT_BSY_CH)

#define IMC_IDI_RXCON_EN_LOG            (1 << 0)
#define IMC_IDI_RXCON_EN_CH             (1 << 1)
#define IMC_IDI_RXCON_FLOW(_bits)       ((_bits) << 2)
#define IMC_IDI_RXCON_SKIP_LIMIT(_bits) ((_bits) << 4)
#define IMC_IDI_RXCON_FLUSH_TO(_bits)   ((_bits) << 8)
#define IMC_IDI_RXCON_DIS_BURST(_bits)  ((_bits) << 16)

#define IMC_IDI_RXCON_FLOW_SYN          (0)
#define IMC_IDI_RXCON_FLOW_PIPE         (1)
#define IMC_IDI_RXCON_FLOW_REAL         (2)

#define IMC_IDI_RXCON2_CLR_LOG          (1 << 0)
#define IMC_IDI_RXCON2_CLR_CH           (1 << 1)
#define IMC_IDI_RXCON2_CLR_WORD         (1 << 2)
#define IMC_IDI_RXCON2_FLUSH(_chan)	(1 << (16 + (_chan)))

/* Bits 15:0 */
#define IMC_IDI_RXDEBUG_DATA_STATUS(_chan) (1 << (_chan))
/* Bits 31:16 */
#define IMC_IDI_RXDEBUG_FULL_STATUS(_chan) (1 << (16 + (_chan)))

#define IMC_IDI_RXIRQ_STAT_IRQ(_chan)   (1 << (_chan))

#define IMC_IDI_RXMASK_CON_IRQ(_chan)	(1 << (_chan))
#define IMC_IDI_RX2MASK_CON_IRQ(_chan)	(1 << (_chan))
#define IMC_IDI_RX3MASK_CON_IRQ(_chan)	(1 << (_chan))

#define IMC_IDI_RXIRQ_CON_CLR(_chan)   (1 << (_chan))

#define IMC_IDI_ERR_CON_EN_TOC(_bit)    ((_bit) << 0)
#define IMC_IDI_ERR_CON_EN_TRC(_bit)    ((_bit) << 1)
#define IMC_IDI_ERR_CON_EN_BUC(_bit)    ((_bit) << 2)
#define IMC_IDI_ERR_CON_TIMEOUT(_bits)  ((_bits) << 3)
#define IMC_IDI_ERR_CON_TRAILING(_bits) ((_bits) << 15)
#define IMC_IDI_ERR_CON_BURSTS(_bits)   ((_bits) << 24)

#define IMC_IDI_ERR_CON_EN_TOC_ENABLED  (1)
#define IMC_IDI_ERR_CON_EN_TOC_DISABLED (0)

#define IMC_IDI_ERR_CON_EN_TRC_ENABLED  (1)
#define IMC_IDI_ERR_CON_EN_TRC_DISABLED (0)

#define IMC_IDI_ERR_CON_EN_BUC_ENABLED  (1)
#define IMC_IDI_ERR_CON_EN_BUC_DISABLED (0)

#define IMC_IDI_EXT_CON_SIG(_bit)       ((_bit) << 0)
#define IMC_IDI_EXT_CON_INT(_bit)       ((_bit) << 1)
#define IMC_IDI_EXT_CON_STREAM(_bits)   ((_bits) << 2)
#define IMC_IDI_EXT_CON_DMA(_bits)      ((_bits) << 5)
#define IMC_IDI_EXT_CON_PRIORITY(_bits) ((_bits) << 8)
#define IMC_IDI_EXT_CON_TIMEOUT(_bits)  ((_bits) << 13)
#define IMC_IDI_EXT_CON_DMA4(_bit)      ((_bit) << 28)

#define IMC_IDI_ABB_EXT_CON_PRIORITY_REG	BIT(0)
#define IMC_IDI_ABB_EXT_CON_PRIORITY_SIG	BIT(1)
#define IMC_IDI_ABB_EXT_CON_PRIORITY_INT	BIT(2)
#define IMC_IDI_ABB_EXT_CON_PRIORITY_OUTSTANDING BIT(3)
#define IMC_IDI_ABB_EXT_CON_PRIORITY_DMA	BIT(4)

#define IMC_IDI_DBB_EXT_CON_PRIORITY_REG	BIT(0)
#define IMC_IDI_DBB_EXT_CON_PRIORITY_SIG	BIT(1)
#define IMC_IDI_DBB_EXT_CON_PRIORITY_DMA	BIT(2)

#define IMC_IDI_EXT_CON_SIG_ENABLED  (1)
#define IMC_IDI_EXT_CON_SIG_DISABLED (0)
#define IMC_IDI_EXT_CON_INT_ENABLED  (1)
#define IMC_IDI_EXT_CON_INT_DISABLED (0)

#define IMC_IDI_EXT_CON2_DIV1(_bits)     ((_bits) << 0)
#define IMC_IDI_EXT_CON2_SAMPLES1(_bits) ((_bits) << 4)
#define IMC_IDI_EXT_CON2_DIV2(_bits)     ((_bits) << 8)
#define IMC_IDI_EXT_CON2_SAMPLES2(_bits) ((_bits) << 12)
#define IMC_IDI_EXT_CON2_DIV3(_bits)     ((_bits) << 16)
#define IMC_IDI_EXT_CON2_SAMPLES3(_bits) ((_bits) << 20)

#define IMC_IDI_CHANNEL_CON_CHANNEL(_bits)  ((_bits) << 0)
#define IMC_IDI_CHANNEL_CON_CHANNEL2(_bits) ((_bits) << 16)

#define IMC_IDI_SLMASK_CON_IRQ(_irq)    (1 << (_irq))
#define IMC_IDI_SL2MASK_CON_IRQ(_irq)   (1 << (_irq))

#define IMC_IDI_SLMASK_CON_MASK (0xFFFFFFFF)

#define IMC_IDI_TXCH_RD_STAT_DAT           (1 << 0)
#define IMC_IDI_TXCH_RD_STAT_PTR_SHIFT     (2)
#define IMC_IDI_TXCH_RD_STAT_DAT_EMPTY     (0)
#define IMC_IDI_TXCH_RD_STAT_DAT_NOT_EMPTY (1)

#define IMC_IDI_TXCH_WR_CON_RST         (1 << 0)
#define IMC_IDI_TXCH_WR_CON_RST_INT     (1 << 1)
#define IMC_IDI_TXCH_WR_CON_PTR_SHIFT   (2)

#define IMC_IDI_TXCH_IRQ_CON_PTR_SHIFT  (2)

#define IMC_IDI_TXCH_NEXT_LAST(_bits)   ((_bits) << 0)
#define IMC_IDI_TXCH_NEXT_INT(_bits)    ((_bits) << 1)
#define IMC_IDI_TXCH_NEXT_NEXT(_bits)   ((_bits) << 4)

#define IMC_IDI_RXCH_WR_STAT_DAT        (1 << 0)
#define IMC_IDI_RXCH_WR_STAT_PTR_SHIFT  (2)

#define IMC_IDI_RXCH_RD_CON_RST         (1 << 0)
#define IMC_IDI_RXCH_RD_CON_RST_INT     (1 << 1)
#define IMC_IDI_RXCH_RD_CON_PTR_SHIFT   (2)

#define IMC_IDI_RXCH_IRQ_CON_PTR_SHIFT  (2)

#define IMC_IDI_RXCH_NEXT_LAST(_bits)   ((_bits) << 0)
#define IMC_IDI_RXCH_NEXT_INT(_bits)    ((_bits) << 1)
#define IMC_IDI_RXCH_NEXT_NEXT(_bits)   ((_bits) << 4)

#endif /* _IDI_IMC_H */
