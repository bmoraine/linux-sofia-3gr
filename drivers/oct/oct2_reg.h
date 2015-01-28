#define OCT2_ID_OFFSET  0xc
#define OCT2_ID(_base) ((_base) + 0xc)
	#define OCT2_ID_REV_NUMBER_OFFSET 0x0
	#define OCT2_ID_REV_NUMBER_WIDTH 0x8
	#define OCT2_ID_REV_NUMBER_MASK 0xff
	#define OCT2_ID_REV_NUMBER(_reg) (((_reg) & 0xff) >> 0x0)
	#define OCT2_ID_MOD_ID_OFFSET 0x8
	#define OCT2_ID_MOD_ID_WIDTH 0x8
	#define OCT2_ID_MOD_ID_MASK 0xff00
	#define OCT2_ID_MOD_ID(_reg) (((_reg) & 0xff00) >> 0x8)
	#define OCT2_ID_TS_REV_NUMBER_OFFSET 0x10
	#define OCT2_ID_TS_REV_NUMBER_WIDTH 0x10
	#define OCT2_ID_TS_REV_NUMBER_MASK 0xffff0000
	#define OCT2_ID_TS_REV_NUMBER(_reg) (((_reg) & 0xffff0000) >> 0x10)

#define OCT2_SWCID_OFFSET  0x1c
#define OCT2_SWCID(_base) ((_base) + 0x1c)
	#define OCT2_SWCID_SWCID_OFFSET 0x0
	#define OCT2_SWCID_SWCID_WIDTH 0x20
	#define OCT2_SWCID_SWCID_MASK 0xffffffff
	#define OCT2_SWCID_SWCID(_reg) (((_reg) & 0xffffffff) >> 0x0)

#define OCT2_CLC_OFFSET  0x0
#define OCT2_CLC(_base) ((_base) + 0x0)
	#define OCT2_CLC_RUN_OFFSET 0x0
	#define OCT2_CLC_RUN_WIDTH 0x2
	#define OCT2_CLC_RUN_MASK 0x3
	#define OCT2_CLC_RUN(_reg) (((_reg) & 0x3) >> 0x0)
		#define OCT2_CLC_RUN_STOP (0x2 << 0)
		#define OCT2_CLC_RUN_RUN (0x1 << 0)
	#define OCT2_CLC_MOD_EN_OFFSET 0x2
	#define OCT2_CLC_MOD_EN_WIDTH 0x2
	#define OCT2_CLC_MOD_EN_MASK 0xc
	#define OCT2_CLC_MOD_EN(_reg) (((_reg) & 0xc) >> 0x2)
		#define OCT2_CLC_MOD_EN_DIS_REQ (0x2 << 0)
		#define OCT2_CLC_MOD_EN_EN_REQ (0x1 << 0)
	#define OCT2_CLC_SPEN_OFFSET 0x4
	#define OCT2_CLC_SPEN_WIDTH 0x2
	#define OCT2_CLC_SPEN_MASK 0x30
	#define OCT2_CLC_SPEN(_reg) (((_reg) & 0x30) >> 0x4)
		#define OCT2_CLC_SPEN_SPDIS (0x2 << 0)
		#define OCT2_CLC_SPEN_SPEN (0x1 << 0)
	#define OCT2_CLC_FSOE_OFFSET 0x6
	#define OCT2_CLC_FSOE_WIDTH 0x2
	#define OCT2_CLC_FSOE_MASK 0xc0
	#define OCT2_CLC_FSOE(_reg) (((_reg) & 0xc0) >> 0x6)
		#define OCT2_CLC_FSOE_FSODIS (0x2 << 0)
		#define OCT2_CLC_FSOE_FSOEN (0x1 << 0)
	#define OCT2_CLC_EDREN_OFFSET 0x8
	#define OCT2_CLC_EDREN_WIDTH 0x2
	#define OCT2_CLC_EDREN_MASK 0x300
	#define OCT2_CLC_EDREN(_reg) (((_reg) & 0x300) >> 0x8)
		#define OCT2_CLC_EDREN_EDRDIS (0x2 << 0)
		#define OCT2_CLC_EDREN_EDREN (0x1 << 0)
	#define OCT2_CLC_IDREN_OFFSET 0xa
	#define OCT2_CLC_IDREN_WIDTH 0x2
	#define OCT2_CLC_IDREN_MASK 0xc00
	#define OCT2_CLC_IDREN(_reg) (((_reg) & 0xc00) >> 0xa)
		#define OCT2_CLC_IDREN_IDRDIS (0x2 << 0)
		#define OCT2_CLC_IDREN_IDREN (0x1 << 0)

#define OCT2_CLC_CNT_OFFSET  0x4
#define OCT2_CLC_CNT(_base) ((_base) + 0x4)
	#define OCT2_CLC_CNT_RMC_OFFSET 0x0
	#define OCT2_CLC_CNT_RMC_WIDTH 0x8
	#define OCT2_CLC_CNT_RMC_MASK 0xff
	#define OCT2_CLC_CNT_RMC(_reg) (((_reg) & 0xff) >> 0x0)
	#define OCT2_CLC_CNT_ORMCSMC_OFFSET 0x8
	#define OCT2_CLC_CNT_ORMCSMC_WIDTH 0x8
	#define OCT2_CLC_CNT_ORMCSMC_MASK 0xff00
	#define OCT2_CLC_CNT_ORMCSMC(_reg) (((_reg) & 0xff00) >> 0x8)

#define OCT2_CLC_STAT_OFFSET  0x8
#define OCT2_CLC_STAT(_base) ((_base) + 0x8)
	#define OCT2_CLC_STAT_RUN_OFFSET 0x0
	#define OCT2_CLC_STAT_RUN_WIDTH 0x1
	#define OCT2_CLC_STAT_RUN_MASK 0x1
	#define OCT2_CLC_STAT_RUN(_reg) (((_reg) & 0x1) >> 0x0)
		#define OCT2_CLC_STAT_RUN_CFG (0x0 << 0)
		#define OCT2_CLC_STAT_RUN_RUN (0x1 << 0)
	#define OCT2_CLC_STAT_MODEN_OFFSET 0x1
	#define OCT2_CLC_STAT_MODEN_WIDTH 0x1
	#define OCT2_CLC_STAT_MODEN_MASK 0x2
	#define OCT2_CLC_STAT_MODEN(_reg) (((_reg) & 0x2) >> 0x1)
		#define OCT2_CLC_STAT_MODEN_MOD_DIS (0x0 << 0)
		#define OCT2_CLC_STAT_MODEN_MOD_EN (0x1 << 0)
	#define OCT2_CLC_STAT_SPEN_OFFSET 0x2
	#define OCT2_CLC_STAT_SPEN_WIDTH 0x1
	#define OCT2_CLC_STAT_SPEN_MASK 0x4
	#define OCT2_CLC_STAT_SPEN(_reg) (((_reg) & 0x4) >> 0x2)
		#define OCT2_CLC_STAT_SPEN_SPEN (0x0 << 0)
		#define OCT2_CLC_STAT_SPEN_SPDIS (0x1 << 0)
	#define OCT2_CLC_STAT_FSOE_OFFSET 0x3
	#define OCT2_CLC_STAT_FSOE_WIDTH 0x1
	#define OCT2_CLC_STAT_FSOE_MASK 0x8
	#define OCT2_CLC_STAT_FSOE(_reg) (((_reg) & 0x8) >> 0x3)
		#define OCT2_CLC_STAT_FSOE_FSODDISABLED (0x0 << 0)
		#define OCT2_CLC_STAT_FSOE_FSOEENABLED (0x1 << 0)
	#define OCT2_CLC_STAT_EDRE_OFFSET 0x4
	#define OCT2_CLC_STAT_EDRE_WIDTH 0x1
	#define OCT2_CLC_STAT_EDRE_MASK 0x10
	#define OCT2_CLC_STAT_EDRE(_reg) (((_reg) & 0x10) >> 0x4)
		#define OCT2_CLC_STAT_EDRE_EDREENABLED (0x0 << 0)
		#define OCT2_CLC_STAT_EDRE_EDRDDISABLED (0x1 << 0)
	#define OCT2_CLC_STAT_IDRE_OFFSET 0x5
	#define OCT2_CLC_STAT_IDRE_WIDTH 0x1
	#define OCT2_CLC_STAT_IDRE_MASK 0x20
	#define OCT2_CLC_STAT_IDRE(_reg) (((_reg) & 0x20) >> 0x5)
		#define OCT2_CLC_STAT_IDRE_IDRE (0x0 << 0)
		#define OCT2_CLC_STAT_IDRE_IDRD (0x1 << 0)
	#define OCT2_CLC_STAT_CUOK_OFFSET 0x6
	#define OCT2_CLC_STAT_CUOK_WIDTH 0x1
	#define OCT2_CLC_STAT_CUOK_MASK 0x40
	#define OCT2_CLC_STAT_CUOK(_reg) (((_reg) & 0x40) >> 0x6)
		#define OCT2_CLC_STAT_CUOK_CUACT (0x0 << 0)
		#define OCT2_CLC_STAT_CUOK_CUOK (0x1 << 0)
	#define OCT2_CLC_STAT_KID_OFFSET 0x7
	#define OCT2_CLC_STAT_KID_WIDTH 0x1
	#define OCT2_CLC_STAT_KID_MASK 0x80
	#define OCT2_CLC_STAT_KID(_reg) (((_reg) & 0x80) >> 0x7)
		#define OCT2_CLC_STAT_KID_KEACT (0x0 << 0)
		#define OCT2_CLC_STAT_KID_KEIDLE (0x1 << 0)
	#define OCT2_CLC_STAT_FIFOID_OFFSET 0x8
	#define OCT2_CLC_STAT_FIFOID_WIDTH 0x1
	#define OCT2_CLC_STAT_FIFOID_MASK 0x100
	#define OCT2_CLC_STAT_FIFOID(_reg) (((_reg) & 0x100) >> 0x8)
		#define OCT2_CLC_STAT_FIFOID_FIFOACT (0x0 << 0)
		#define OCT2_CLC_STAT_FIFOID_FIFOIDLE (0x1 << 0)

#define OCT2_CRC_CNF_OFFSET  0x100
#define OCT2_CRC_CNF(_base) ((_base) + 0x100)
	#define OCT2_CRC_CNF_LEN_OFFSET 0x0
	#define OCT2_CRC_CNF_LEN_WIDTH 0x1
	#define OCT2_CRC_CNF_LEN_MASK 0x1
	#define OCT2_CRC_CNF_LEN(_reg) (((_reg) & 0x1) >> 0x0)
		#define OCT2_CRC_CNF_LEN_THIRTY_TWO 0x0
		#define OCT2_CRC_CNF_LEN_SIXTEEN 0x1
	#define OCT2_CRC_CNF_INIT_OFFSET 0x1
	#define OCT2_CRC_CNF_INIT_WIDTH 0x1
	#define OCT2_CRC_CNF_INIT_MASK 0x2
	#define OCT2_CRC_CNF_INIT(_reg) (((_reg) & 0x2) >> 0x1)
		#define OCT2_CRC_CNF_INIT_ZEROS 0x0
		#define OCT2_CRC_CNF_INIT_ONES 0x1

#define OCT2_CRC_POLY_CNF_OFFSET  0x104
#define OCT2_CRC_POLY_CNF(_base) ((_base) + 0x104)
	#define OCT2_CRC_POLY_CNF_CRCPOLY_OFFSET 0x0
	#define OCT2_CRC_POLY_CNF_CRCPOLY_WIDTH 0x20
	#define OCT2_CRC_POLY_CNF_CRCPOLY_MASK 0xffffffff
	#define OCT2_CRC_POLY_CNF_CRCPOLY(_reg) (((_reg) & 0xffffffff) >> 0x0)

#define OCT2_CNF_OFFSET  0x108
#define OCT2_CNF(_base) ((_base) + 0x108)
	#define OCT2_CNF_FRAME_TIMEOUT_OFFSET 0x0
	#define OCT2_CNF_FRAME_TIMEOUT_WIDTH 0x9
	#define OCT2_CNF_FRAME_TIMEOUT_MASK 0x1ff
	#define OCT2_CNF_FRAME_TIMEOUT(_reg) (((_reg) & 0x1ff) >> 0x0)
		#define OCT2_CNF_FRAME_TIMEOUT_MULTIPLIER (0xc << 0)
		#define OCT2_CNF_FRAME_TIMEOUT_DISABLED (0x0 << 0)
	#define OCT2_CNF_FRAME_TIMEOUT_FACTOR_OFFSET 0xb
	#define OCT2_CNF_FRAME_TIMEOUT_FACTOR_WIDTH 0x1
	#define OCT2_CNF_FRAME_TIMEOUT_FACTOR_MASK 0x800
	#define OCT2_CNF_FRAME_TIMEOUT_FACTOR(_reg) (((_reg) & 0x800) >> 0xb)
		#define OCT2_CNF_FRAME_TIMEOUT_FACTOR_FACTOR1 0x0
		#define OCT2_CNF_FRAME_TIMEOUT_FACTOR_FACTOR128 0x1
	#define OCT2_CNF_PAGE_TIMEOUT_OFFSET 0xc
	#define OCT2_CNF_PAGE_TIMEOUT_WIDTH 0x9
	#define OCT2_CNF_PAGE_TIMEOUT_MASK 0x1ff000
	#define OCT2_CNF_PAGE_TIMEOUT(_reg) (((_reg) & 0x1ff000) >> 0xc)
		#define OCT2_CNF_PAGE_TIMEOUT_MUTIPLIER (0xc << 0)
		#define OCT2_CNF_PAGE_TIMEOUT_DISABLED (0x0 << 0)
	#define OCT2_CNF_OCTM_MODE_OFFSET 0x16
	#define OCT2_CNF_OCTM_MODE_WIDTH 0x1
	#define OCT2_CNF_OCTM_MODE_MASK 0x400000
	#define OCT2_CNF_OCTM_MODE(_reg) (((_reg) & 0x400000) >> 0x16)
		#define OCT2_CNF_OCTM_MODE_RINGBUF 0x0
		#define OCT2_CNF_OCTM_MODE_FIFO 0x1
	#define OCT2_CNF_OCTM_CLR_OFFSET 0x17
	#define OCT2_CNF_OCTM_CLR_WIDTH 0x1
	#define OCT2_CNF_OCTM_CLR_MASK 0x800000
	#define OCT2_CNF_OCTM_CLR(_reg) (((_reg) & 0x800000) >> 0x17)
		#define OCT2_CNF_OCTM_CLR_IDLE 0x0
		#define OCT2_CNF_OCTM_CLR_CLR 0x1
	#define OCT2_CNF_PAGE_TIMEOUT_FACTOR_OFFSET 0x18
	#define OCT2_CNF_PAGE_TIMEOUT_FACTOR_WIDTH 0x1
	#define OCT2_CNF_PAGE_TIMEOUT_FACTOR_MASK 0x1000000
	#define OCT2_CNF_PAGE_TIMEOUT_FACTOR(_reg) \
			(((_reg) & 0x1000000) >> 0x18)
		#define OCT2_CNF_PAGE_TIMEOUT_FACTOR_FACTOR1  0x0
		#define OCT2_CNF_PAGE_TIMEOUT_FACTOR_FACTOR128 0x1
	#define OCT2_CNF_PAGE_TABLE_READY_OFFSET 0x1a
	#define OCT2_CNF_PAGE_TABLE_READY_WIDTH 0x1
	#define OCT2_CNF_PAGE_TABLE_READY_MASK 0x4000000
	#define OCT2_CNF_PAGE_TABLE_READY(_reg) (((_reg) & 0x4000000) >> 0x1a)
		#define OCT2_CNF_PAGE_TABLE_READY_CLEAR 0x0
		#define OCT2_CNF_PAGE_TABLE_READY_SET 0x1
	#define OCT2_CNF_ARBITER_ENDIAN_OFFSET 0x1c
	#define OCT2_CNF_ARBITER_ENDIAN_WIDTH 0x1
	#define OCT2_CNF_ARBITER_ENDIAN_MASK 0x10000000
	#define OCT2_CNF_ARBITER_ENDIAN(_reg) (((_reg) & 0x10000000) >> 0x1c)
		#define OCT2_CNF_ARBITER_ENDIAN_BIG 0x0
		#define OCT2_CNF_ARBITER_ENDIAN_LITTLE 0x1
	#define OCT2_CNF_TRF_DROP_CLR_OFFSET 0x1e
	#define OCT2_CNF_TRF_DROP_CLR_WIDTH 0x1
	#define OCT2_CNF_TRF_DROP_CLR_MASK 0x40000000
	#define OCT2_CNF_TRF_DROP_CLR(_reg) (((_reg) & 0x40000000) >> 0x1e)
		#define OCT2_CNF_TRF_DROP_CLR_CLEAR 0x0
		#define OCT2_CNF_TRF_DROP_CLR_SET 0x1
	#define OCT2_CNF_MANUAL_STALL_OFFSET 0x1f
	#define OCT2_CNF_MANUAL_STALL_WIDTH 0x1
	#define OCT2_CNF_MANUAL_STALL_MASK 0x80000000
	#define OCT2_CNF_MANUAL_STALL(_reg) (((_reg) & 0x80000000) >> 0x1f)
		#define OCT2_CNF_MANUAL_STALL_IDLE 0x0
		#define OCT2_CNF_MANUAL_STALL_SET 0x1
#define OCT2_CNF2_OFFSET  0x10c
#define OCT2_CNF2(_base) ((_base) + 0x10c)
	#define OCT2_CNF2_RSTRT_OFFSET 0x0
	#define OCT2_CNF2_RSTRT_WIDTH 0x1
	#define OCT2_CNF2_RSTRT_MASK 0x1
	#define OCT2_CNF2_RSTRT(_reg) (((_reg) & 0x1) >> 0x0)

#define OCT2_STAT_OFFSET  0x110
#define OCT2_STAT(_base) ((_base) + 0x110)
	#define OCT2_STAT_OCTM_FULL_OFFSET 0x0
	#define OCT2_STAT_OCTM_FULL_WIDTH 0x1
	#define OCT2_STAT_OCTM_FULL_MASK 0x1
	#define OCT2_STAT_OCTM_FULL(_reg) (((_reg) & 0x1) >> 0x0)
		#define OCT2_STAT_OCTM_FULL_IDLE (0x0 << 0)
		#define OCT2_STAT_OCTM_FULL_FULL (0x1 << 0)
	#define OCT2_STAT_TRF_RDY_OFFSET 0x1
	#define OCT2_STAT_TRF_RDY_WIDTH 0x1
	#define OCT2_STAT_TRF_RDY_MASK 0x2
	#define OCT2_STAT_TRF_RDY(_reg) (((_reg) & 0x2) >> 0x1)
		#define OCT2_STAT_TRF_RDY_READY (0x1 << 0)
		#define OCT2_STAT_TRF_RDY_NOTREADY (0x0 << 0)
	#define OCT2_STAT_OCTM_RDY_OFFSET 0x2
	#define OCT2_STAT_OCTM_RDY_WIDTH 0x1
	#define OCT2_STAT_OCTM_RDY_MASK 0x4
	#define OCT2_STAT_OCTM_RDY(_reg) (((_reg) & 0x4) >> 0x2)
		#define OCT2_STAT_OCTM_RDY_READY (0x1 << 0)
		#define OCT2_STAT_OCTM_RDY_IDLE (0x0 << 0)
	#define OCT2_STAT_TRF_DROP_OFFSET 0x3
	#define OCT2_STAT_TRF_DROP_WIDTH 0x1
	#define OCT2_STAT_TRF_DROP_MASK 0x8
	#define OCT2_STAT_TRF_DROP(_reg) (((_reg) & 0x8) >> 0x3)
		#define OCT2_STAT_TRF_DROP_IDLE (0x0 << 0)
		#define OCT2_STAT_TRF_DROP_DROPPING (0x1 << 0)
	#define OCT2_STAT_TRFORM_BUSY_OFFSET 0x4
	#define OCT2_STAT_TRFORM_BUSY_WIDTH 0x1
	#define OCT2_STAT_TRFORM_BUSY_MASK 0x10
	#define OCT2_STAT_TRFORM_BUSY(_reg) (((_reg) & 0x10) >> 0x4)
		#define OCT2_STAT_TRFORM_BUSY_BUSY (0x1 << 0)
		#define OCT2_STAT_TRFORM_BUSY_NOTBUSY (0x0 << 0)
	#define OCT2_STAT_OCT_STALL_OFFSET 0x5
	#define OCT2_STAT_OCT_STALL_WIDTH 0x1
	#define OCT2_STAT_OCT_STALL_MASK 0x20
	#define OCT2_STAT_OCT_STALL(_reg) (((_reg) & 0x20) >> 0x5)
		#define OCT2_STAT_OCT_STALL_IDLE (0x0 << 0)
		#define OCT2_STAT_OCT_STALL_STALL (0x1 << 0)
	#define OCT2_STAT_DMA_BUSY_OFFSET 0x10
	#define OCT2_STAT_DMA_BUSY_WIDTH 0x1
	#define OCT2_STAT_DMA_BUSY_MASK 0x10000
	#define OCT2_STAT_DMA_BUSY(_reg) (((_reg) & 0x10000) >> 0x10)
		#define OCT2_STAT_DMA_BUSY_IDLE (0x0 << 0)
		#define OCT2_STAT_DMA_BUSY_BUSY (0x1 << 0)
	#define OCT2_STAT_DMA_BUF_DATA_OFFSET 0x11
	#define OCT2_STAT_DMA_BUF_DATA_WIDTH 0x1
	#define OCT2_STAT_DMA_BUF_DATA_MASK 0x20000
	#define OCT2_STAT_DMA_BUF_DATA(_reg) (((_reg) & 0x20000) >> 0x11)
		#define OCT2_STAT_DMA_BUF_DATA_EMPTY (0x0 << 0)
		#define OCT2_STAT_DMA_BUF_DATA_RDY (0x1 << 0)
	#define OCT2_STAT_DMA_BUF_FULL_OFFSET 0x12
	#define OCT2_STAT_DMA_BUF_FULL_WIDTH 0x1
	#define OCT2_STAT_DMA_BUF_FULL_MASK 0x40000
	#define OCT2_STAT_DMA_BUF_FULL(_reg) (((_reg) & 0x40000) >> 0x12)
		#define OCT2_STAT_DMA_BUF_FULL_IDLE (0x0 << 0)
		#define OCT2_STAT_DMA_BUF_FULL_FULL (0x1 << 0)

#define OCT2_PG_INDEX_STAT_OFFSET  0x114
#define OCT2_PG_INDEX_STAT(_base) ((_base) + 0x114)
	#define OCT2_PG_INDEX_STAT_PG_INDEX_OFFSET 0x0
	#define OCT2_PG_INDEX_STAT_PG_INDEX_WIDTH 0x10
	#define OCT2_PG_INDEX_STAT_PG_INDEX_MASK 0xffff
	#define OCT2_PG_INDEX_STAT_PG_INDEX(_reg) (((_reg) & 0xffff) >> 0x0)

#define OCT2_M_WPTR_STAT_OFFSET  0x118
#define OCT2_M_WPTR_STAT(_base) ((_base) + 0x118)
	#define OCT2_M_WPTR_STAT_OCTM_WPTR_OFFSET 0x0
	#define OCT2_M_WPTR_STAT_OCTM_WPTR_WIDTH 0xc
	#define OCT2_M_WPTR_STAT_OCTM_WPTR_MASK 0xfff
	#define OCT2_M_WPTR_STAT_OCTM_WPTR(_reg) (((_reg) & 0xfff) >> 0x0)

#define OCT2_EXT_PG_WPTR_OFFSET  0x11c
#define OCT2_EXT_PG_WPTR(_base) ((_base) + 0x11c)
	#define OCT2_EXT_PG_WPTR_EXT_MEM_WPTR_OFFSET 0x0
	#define OCT2_EXT_PG_WPTR_EXT_MEM_WPTR_WIDTH 0x1e
	#define OCT2_EXT_PG_WPTR_EXT_MEM_WPTR_MASK 0x3fffffff
	#define OCT2_EXT_PG_WPTR_EXT_MEM_WPTR(_reg) \
		(((_reg) & 0x3fffffff) >> 0x0)

#define OCT2_OCTM_RXDL_OFFSET  0x120
#define OCT2_OCTM_RXDL(_base) ((_base) + 0x120)

#define OCT2_OCTM_RXDM_OFFSET  0x124
#define OCT2_OCTM_RXDM(_base) ((_base) + 0x124)

	#define OCT2_OCTM_RXDL_RXDL_OFFSET 0x0
	#define OCT2_OCTM_RXDL_RXDL_WIDTH 0x20
	#define OCT2_OCTM_RXDL_RXDL_MASK 0xffffffff
	#define OCT2_OCTM_RXDL_RXDL(_reg) (((_reg) & 0xffffffff) >> 0x0)

	#define OCT2_OCTM_RXDM_RXDM_OFFSET 0x0
	#define OCT2_OCTM_RXDM_RXDM_WIDTH 0x20
	#define OCT2_OCTM_RXDM_RXDM_MASK 0xffffffff
	#define OCT2_OCTM_RXDM_RXDM(_reg) (((_reg) & 0xffffffff) >> 0x0)

#define OCT2_RIS_OFFSET  0x80
#define OCT2_RIS(_base) ((_base) + 0x80)
	#define OCT2_RIS_BUFFER_INDEX_OFFSET 0x0
	#define OCT2_RIS_BUFFER_INDEX_WIDTH 0x10
	#define OCT2_RIS_BUFFER_INDEX_MASK 0xffff
	#define OCT2_RIS_BUFFER_INDEX(_reg) (((_reg) & 0xffff) >> 0x0)
	#define OCT2_RIS_REFILL_OFFSET 0x10
	#define OCT2_RIS_REFILL_WIDTH 0x1
	#define OCT2_RIS_REFILL_MASK 0x10000
	#define OCT2_RIS_REFILL(_reg) (((_reg) & 0x10000) >> 0x10)
	#define OCT2_RIS_FORWARD_OFFSET 0x11
	#define OCT2_RIS_FORWARD_WIDTH 0x1
	#define OCT2_RIS_FORWARD_MASK 0x20000
	#define OCT2_RIS_FORWARD(_reg) (((_reg) & 0x20000) >> 0x11)
	#define OCT2_RIS_TIMEOUT_OFFSET 0x12
	#define OCT2_RIS_TIMEOUT_WIDTH 0x1
	#define OCT2_RIS_TIMEOUT_MASK 0x40000
	#define OCT2_RIS_TIMEOUT(_reg) (((_reg) & 0x40000) >> 0x12)
	#define OCT2_RIS_END_OFFSET 0x13
	#define OCT2_RIS_END_WIDTH 0x1
	#define OCT2_RIS_END_MASK 0x80000
	#define OCT2_RIS_END(_reg) (((_reg) & 0x80000) >> 0x13)
	#define OCT2_RIS_RESTART_OFFSET 0x14
	#define OCT2_RIS_RESTART_WIDTH 0x1
	#define OCT2_RIS_RESTART_MASK 0x100000
	#define OCT2_RIS_RESTART(_reg) (((_reg) & 0x100000) >> 0x14)

#define OCT2_IMSC_OFFSET  0x84
#define OCT2_IMSC(_base) ((_base) + 0x84)
	#define OCT2_IMSC_BUFFER_INDEX_OFFSET 0x0
	#define OCT2_IMSC_BUFFER_INDEX_WIDTH 0x10
	#define OCT2_IMSC_BUFFER_INDEX_MASK 0xffff
	#define OCT2_IMSC_BUFFER_INDEX(_reg) (((_reg) & 0xffff) >> 0x0)
	#define OCT2_IMSC_REFILL_OFFSET 0x10
	#define OCT2_IMSC_REFILL_WIDTH 0x1
	#define OCT2_IMSC_REFILL_MASK 0x10000
	#define OCT2_IMSC_REFILL(_reg) (((_reg) & 0x10000) >> 0x10)
	#define OCT2_IMSC_FORWARD_OFFSET 0x11
	#define OCT2_IMSC_FORWARD_WIDTH 0x1
	#define OCT2_IMSC_FORWARD_MASK 0x20000
	#define OCT2_IMSC_FORWARD(_reg) (((_reg) & 0x20000) >> 0x11)
	#define OCT2_IMSC_TIMEOUT_OFFSET 0x12
	#define OCT2_IMSC_TIMEOUT_WIDTH 0x1
	#define OCT2_IMSC_TIMEOUT_MASK 0x40000
	#define OCT2_IMSC_TIMEOUT(_reg) (((_reg) & 0x40000) >> 0x12)
	#define OCT2_IMSC_END_OFFSET 0x13
	#define OCT2_IMSC_END_WIDTH 0x1
	#define OCT2_IMSC_END_MASK 0x80000
	#define OCT2_IMSC_END(_reg) (((_reg) & 0x80000) >> 0x13)
	#define OCT2_IMSC_RESTART_OFFSET 0x14
	#define OCT2_IMSC_RESTART_WIDTH 0x1
	#define OCT2_IMSC_RESTART_MASK 0x100000
	#define OCT2_IMSC_RESTART(_reg) (((_reg) & 0x100000) >> 0x14)

#define OCT2_MIS_OFFSET  0x88
#define OCT2_MIS(_base) ((_base) + 0x88)
	#define OCT2_MIS_BUFFER_INDEX_OFFSET 0x0
	#define OCT2_MIS_BUFFER_INDEX_WIDTH 0x10
	#define OCT2_MIS_BUFFER_INDEX_MASK 0xffff
	#define OCT2_MIS_BUFFER_INDEX(_reg) (((_reg) & 0xffff) >> 0x0)
	#define OCT2_MIS_REFILL_OFFSET 0x10
	#define OCT2_MIS_REFILL_WIDTH 0x1
	#define OCT2_MIS_REFILL_MASK 0x10000
	#define OCT2_MIS_REFILL(_reg) (((_reg) & 0x10000) >> 0x10)
	#define OCT2_MIS_FORWARD_OFFSET 0x11
	#define OCT2_MIS_FORWARD_WIDTH 0x1
	#define OCT2_MIS_FORWARD_MASK 0x20000
	#define OCT2_MIS_FORWARD(_reg) (((_reg) & 0x20000) >> 0x11)
	#define OCT2_MIS_TIMEOUT_OFFSET 0x12
	#define OCT2_MIS_TIMEOUT_WIDTH 0x1
	#define OCT2_MIS_TIMEOUT_MASK 0x40000
	#define OCT2_MIS_TIMEOUT(_reg) (((_reg) & 0x40000) >> 0x12)
	#define OCT2_MIS_END_OFFSET 0x13
	#define OCT2_MIS_END_WIDTH 0x1
	#define OCT2_MIS_END_MASK 0x80000
	#define OCT2_MIS_END(_reg) (((_reg) & 0x80000) >> 0x13)
	#define OCT2_MIS_RESTART_OFFSET 0x14
	#define OCT2_MIS_RESTART_WIDTH 0x1
	#define OCT2_MIS_RESTART_MASK 0x100000
	#define OCT2_MIS_RESTART(_reg) (((_reg) & 0x100000) >> 0x14)

#define OCT2_ISR_OFFSET  0x90
#define OCT2_ISR(_base) ((_base) + 0x90)
	#define OCT2_ISR_BUFFER_INDEX_OFFSET 0x0
	#define OCT2_ISR_BUFFER_INDEX_WIDTH 0x10
	#define OCT2_ISR_BUFFER_INDEX_MASK 0xffff
	#define OCT2_ISR_BUFFER_INDEX(_reg) (((_reg) & 0xffff) >> 0x0)
	#define OCT2_ISR_REFILL_OFFSET 0x10
	#define OCT2_ISR_REFILL_WIDTH 0x1
	#define OCT2_ISR_REFILL_MASK 0x10000
	#define OCT2_ISR_REFILL(_reg) (((_reg) & 0x10000) >> 0x10)
	#define OCT2_ISR_FORWARD_OFFSET 0x11
	#define OCT2_ISR_FORWARD_WIDTH 0x1
	#define OCT2_ISR_FORWARD_MASK 0x20000
	#define OCT2_ISR_FORWARD(_reg) (((_reg) & 0x20000) >> 0x11)
	#define OCT2_ISR_TIMEOUT_OFFSET 0x12
	#define OCT2_ISR_TIMEOUT_WIDTH 0x1
	#define OCT2_ISR_TIMEOUT_MASK 0x40000
	#define OCT2_ISR_TIMEOUT(_reg) (((_reg) & 0x40000) >> 0x12)
	#define OCT2_ISR_END_OFFSET 0x13
	#define OCT2_ISR_END_WIDTH 0x1
	#define OCT2_ISR_END_MASK 0x80000
	#define OCT2_ISR_END(_reg) (((_reg) & 0x80000) >> 0x13)
	#define OCT2_ISR_RESTART_OFFSET 0x14
	#define OCT2_ISR_RESTART_WIDTH 0x1
	#define OCT2_ISR_RESTART_MASK 0x100000
	#define OCT2_ISR_RESTART(_reg) (((_reg) & 0x100000) >> 0x14)

#define OCT2_ICR_OFFSET  0x98
#define OCT2_ICR(_base) ((_base) + 0x98)
	#define OCT2_ICR_BUFFER_INDEX_OFFSET 0x0
	#define OCT2_ICR_BUFFER_INDEX_WIDTH 0x10
	#define OCT2_ICR_BUFFER_INDEX_MASK 0xffff
	#define OCT2_ICR_BUFFER_INDEX(_reg) (((_reg) & 0xffff) >> 0x0)
	#define OCT2_ICR_REFILL_OFFSET 0x10
	#define OCT2_ICR_REFILL_WIDTH 0x1
	#define OCT2_ICR_REFILL_MASK 0x10000
	#define OCT2_ICR_REFILL(_reg) (((_reg) & 0x10000) >> 0x10)
	#define OCT2_ICR_FORWARD_OFFSET 0x11
	#define OCT2_ICR_FORWARD_WIDTH 0x1
	#define OCT2_ICR_FORWARD_MASK 0x20000
	#define OCT2_ICR_FORWARD(_reg) (((_reg) & 0x20000) >> 0x11)
	#define OCT2_ICR_TIMEOUT_OFFSET 0x12
	#define OCT2_ICR_TIMEOUT_WIDTH 0x1
	#define OCT2_ICR_TIMEOUT_MASK 0x40000
	#define OCT2_ICR_TIMEOUT(_reg) (((_reg) & 0x40000) >> 0x12)
	#define OCT2_ICR_END_OFFSET 0x13
	#define OCT2_ICR_END_WIDTH 0x1
	#define OCT2_ICR_END_MASK 0x80000
	#define OCT2_ICR_END(_reg) (((_reg) & 0x80000) >> 0x13)
	#define OCT2_ICR_RESTART_OFFSET 0x14
	#define OCT2_ICR_RESTART_WIDTH 0x1
	#define OCT2_ICR_RESTART_MASK 0x100000
	#define OCT2_ICR_RESTART(_reg) (((_reg) & 0x100000) >> 0x14)

#define OCT2_PGT_MEM0_OFFSET  0x200
#define OCT2_PGT_MEM0(_base) ((_base) + 0x200)

#define OCT2_PGT_MEM1_OFFSET  0x600
#define OCT2_PGT_MEM1(_base) ((_base) + 0x600)

/***/
/*setOct_OCT2_REG_BIT(void __iomem *base, unsigned value) */
#define DECLARE_OCT_SET_REG_ACC(REG, BIT)\
void setOct_##REG##_##BIT(void __iomem *base, unsigned value)\
{\
	unsigned tmp = ioread32(base + REG##_OFFSET);\
	tmp &= ~REG##_##BIT##_MASK;\
	tmp |= value << REG##_##BIT##_OFFSET;\
	iowrite32(tmp, base + REG##_OFFSET);\
}

DECLARE_OCT_SET_REG_ACC(OCT2_CNF, PAGE_TIMEOUT)
DECLARE_OCT_SET_REG_ACC(OCT2_CNF, FRAME_TIMEOUT)
DECLARE_OCT_SET_REG_ACC(OCT2_CNF, PAGE_TABLE_READY)
DECLARE_OCT_SET_REG_ACC(OCT2_CNF, TRF_DROP_CLR)
DECLARE_OCT_SET_REG_ACC(OCT2_CNF, OCTM_CLR)
DECLARE_OCT_SET_REG_ACC(OCT2_CNF, MANUAL_STALL)
DECLARE_OCT_SET_REG_ACC(OCT2_CNF, OCTM_MODE)
DECLARE_OCT_SET_REG_ACC(OCT2_CNF, FRAME_TIMEOUT_FACTOR)
DECLARE_OCT_SET_REG_ACC(OCT2_CNF, ARBITER_ENDIAN)
DECLARE_OCT_SET_REG_ACC(OCT2_CNF, PAGE_TIMEOUT_FACTOR)
DECLARE_OCT_SET_REG_ACC(OCT2_CNF2, RSTRT)
DECLARE_OCT_SET_REG_ACC(OCT2_CRC_POLY_CNF, CRCPOLY)
DECLARE_OCT_SET_REG_ACC(OCT2_CRC_CNF, LEN)
DECLARE_OCT_SET_REG_ACC(OCT2_CRC_CNF, INIT)

/*getOct_OCT2_REG_BIT(void __iomem *base) */
#define DECLARE_OCT_GET_REG_ACC(REG, BIT)\
unsigned getOct_##REG##_##BIT(void __iomem *base)\
{\
	unsigned tmp = ioread32(base + REG##_OFFSET);\
	tmp &= REG##_##BIT##_MASK;\
	tmp >>= REG##_##BIT##_OFFSET;\
	return tmp;\
}
DECLARE_OCT_GET_REG_ACC(OCT2_CNF, PAGE_TIMEOUT)
DECLARE_OCT_GET_REG_ACC(OCT2_EXT_PG_WPTR, EXT_MEM_WPTR)
DECLARE_OCT_GET_REG_ACC(OCT2_PG_INDEX_STAT, PG_INDEX)

/*setOct_OCT2_REG(void __iomem *base, unsigned value) */
#define DECLARE_OCT_SET_REG(REG)\
void setOct_##REG(void __iomem *base, unsigned value)\
{\
	iowrite32(value, base + REG##_OFFSET);\
}
DECLARE_OCT_SET_REG(OCT2_ICR)
DECLARE_OCT_SET_REG(OCT2_IMSC)
DECLARE_OCT_SET_REG(OCT2_CLC)

/*geOct_OCT2_REG(void __iomem *base) */
#define DECLARE_OCT_GET_REG(REG)\
unsigned getOct_##REG(void __iomem *base)\
{\
	return ioread32(base + REG##_OFFSET);\
}
DECLARE_OCT_GET_REG(OCT2_CNF)
DECLARE_OCT_GET_REG(OCT2_ICR)
DECLARE_OCT_GET_REG(OCT2_IMSC)
DECLARE_OCT_GET_REG(OCT2_CLC)
DECLARE_OCT_GET_REG(OCT2_RIS)
DECLARE_OCT_GET_REG(OCT2_CLC_STAT)


struct _sOct {
	int OCT2_CLC;
	int OCT2_CLC_CNT;
	int OCT2_CLC_STAT;
	int OCT2_ID;
	const char reservedArea0[12];
	int OCT2_SWCID;
	const char reservedArea1[96];
	int OCT2_RIS;
	int OCT2_IMSC;
	int OCT2_MIS;
	const char reservedArea2[4];
	int OCT2_ISR;
	const char reservedArea3[4];
	int OCT2_ICR;
	const char  reservedArea4[100];
	int OCT2_CRC_CNF;
	int OCT2_CRC_POLY_CNF;
	int OCT2_CNF;
	int OCT2_CNF2;
	int OCT2_STAT;
	int OCT2_PG_INDEX_STAT;
	int OCT2_M_WPTR_STAT;
	int OCT2_EXT_PG_WPTR;
	int OCT2_OCTM_RXDL;
	int OCT2_OCTM_RXDM;
	const char reservedArea5[216];
	int OCT2_PGT_MEM0[128];
	const char reservedArea6[512];
	int OCT2_PGT_MEM1[128];
};

void setOct_OCT2_PGT_MEM0(struct _sOct *_oct_, int value, int index)
{
	iowrite32(value, &_oct_->OCT2_PGT_MEM0[index]);
}

void setOct_OCT2_PGT_MEM1(struct _sOct *_oct_, int value, int index)
{
	iowrite32(value, &_oct_->OCT2_PGT_MEM1[index]);
}

