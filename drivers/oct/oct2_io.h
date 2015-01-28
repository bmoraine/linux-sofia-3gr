#ifndef _OCT2_IO_H_
#define _OCT2_IO_H_

#include "oct2_reg.h"
#include <linux/io.h>

/*IRQ related macros */
/* use refill IRQ to detect buffer full -> REFILL & END IRQs are swapped*/
enum eOCT2_ICRLsb {
	OCT2_ICR_LSB_BUFFER_INDEX = 0,
	OCT2_ICR_LSB_REFILL = 16,
	OCT2_ICR_LSB_FORWARD = 17,
	OCT2_ICR_LSB_TIMEOUT = 18,
	OCT2_ICR_LSB_END = 19,
	OCT2_ICR_LSB_RESTART = 20
};

#define OCT2_IRQ_END	(1<<OCT2_ICR_LSB_REFILL)
#define OCT2_IRQ_FW	(1<<OCT2_ICR_LSB_FORWARD)
#define OCT2_IRQ_TMO	(1<<OCT2_ICR_LSB_TIMEOUT)
#define OCT2_IRQ_REFILL	(1<<OCT2_ICR_LSB_END)
#define OCT2_IRQ_RES	(1<<OCT2_ICR_LSB_RESTART)
#define OCT2_ALL_IRQS	(OCT2_IRQ_REFILL | OCT2_IRQ_FW\
		| OCT2_IRQ_TMO | OCT2_IRQ_END | OCT2_IRQ_RES)

#define OCT2_CLEAR_IRQS(irq) \
	setOct_OCT2_ICR(oct2_reg, irq)
#define OCT2_ENABLE_IRQS(irq) \
	setOct_OCT2_IMSC(oct2_reg, irq | getOct_OCT2_IMSC(oct2_reg))
#define OCT2_DISABLE_IRQS(irq) \
	setOct_OCT2_IMSC(oct2_reg, ~irq & getOct_OCT2_IMSC(oct2_reg))
#define OCT2_ENABLE_ALL_IRQS  OCT2_ENABLE_IRQS(OCT2_ALL_IRQS)
#define OCT2_DISABLE_ALL_IRQS OCT2_DISABLE_IRQS(OCT2_ALL_IRQS)

/*config related macros */
#define OCT2_SET_CONF_MODE setOct_OCT2_CLC(oct2_reg, 0xAA6)
#define OCT2_SET_RUN_MODE  setOct_OCT2_CLC(oct2_reg, 0xAA5)
#define OCT2_SET_PG_TIMEOUT(value) \
	setOct_OCT2_CNF_PAGE_TIMEOUT(oct2_reg, value)
#define OCT2_SET_PG_TIMEOUT_FACTOR_128(onOff) \
		setOct_OCT2_CNF_PAGE_TIMEOUT_FACTOR(oct2_reg,\
	(onOff == oct2_on)?OCT2_CNF_PAGE_TIMEOUT_FACTOR_FACTOR128 :\
	OCT2_CNF_PAGE_TIMEOUT_FACTOR_FACTOR1)
#define OCT2_SET_FRAME_TIMEOUT(value) \
	setOct_OCT2_CNF_FRAME_TIMEOUT(oct2_reg, value)
#define OCT2_SET_PGT_MEM0(offset, value) \
	setOct_OCT2_PGT_MEM0(oct2_reg, value, offset)
#define OCT2_SET_PGT_MEM1(offset, value) \
	setOct_OCT2_PGT_MEM1(oct2_reg, value, offset)
#define OCT2_SET_CNF_PTBY \
	setOct_OCT2_CNF_PAGE_TABLE_READY(oct2_reg,\
		OCT2_CNF_PAGE_TABLE_READY_SET)
#define OCT2_CLR_CNF_PTBY \
	setOct_OCT2_CNF_PAGE_TABLE_READY(oct2_reg,\
		OCT2_CNF_PAGE_TABLE_READY_CLEAR)
#define OCT2_SET_CNF2_RESTART \
	setOct_OCT2_CNF2_RSTRT(oct2_reg, 1);
#define OCT2_GET_IRQ_STAT \
	(getOct_OCT2_RIS(oct2_reg) & 0x1F0000)
#define OCT2_GET_PAGE_INDEX \
	getOct_OCT2_PG_INDEX_STAT_PG_INDEX(oct2_reg)
#define OCT2_GET_PAGE_WPTR \
	((getOct_OCT2_EXT_PG_WPTR_EXT_MEM_WPTR(oct2_reg) << 3) & 0xFFFFFFF8)
#define OCT_CLR_TRFORM_DROP_STATUS {\
	setOct_OCT2_CNF_TRF_DROP_CLR(oct2_reg, OCT2_CNF_TRF_DROP_CLR_SET);\
	setOct_OCT2_CNF_TRF_DROP_CLR(oct2_reg, OCT2_CNF_TRF_DROP_CLR_CLEAR); }
#define OCT_CLR_OCT_ON \
	setOct_OCT2_CNF_OCTM_CLR(oct2_reg, OCT2_CNF_OCTM_CLR_CLR)
#define OCT_CLR_OCT_OFF \
	setOct_OCT2_CNF_OCTM_CLR(oct2_reg, OCT2_CNF_OCTM_CLR_IDLE)
#define OCT2_CONF_MANU_STALL(onOff) \
	setOct_OCT2_CNF_MANUAL_STALL(oct2_reg,\
	(onOff == oct2_on)?OCT2_CNF_MANUAL_STALL_SET :\
	OCT2_CNF_MANUAL_STALL_IDLE)
#define OCT2_CONF_OCTM_RINGBUFF(onOff) \
	setOct_OCT2_CNF_OCTM_MODE(oct2_reg,\
	(onOff == oct2_on)?OCT2_CNF_OCTM_MODE_RINGBUF :\
	OCT2_CNF_OCTM_MODE_FIFO)
#define OCT2_SET_CRC_POLY(poly) \
	setOct_OCT2_CRC_POLY_CNF_CRCPOLY(oct2_reg, poly)
#define OCT2_SET_CRC_LEN_16(onOff) \
	setOct_OCT2_CRC_CNF_LEN(oct2_reg,\
	(onOff == oct2_on)?OCT2_CRC_CNF_LEN_SIXTEEN :\
	OCT2_CRC_CNF_LEN_THIRTY_TWO)
#define OCT2_SET_FRAME_TIMEOUT_FACTOR_128(onOff) \
	setOct_OCT2_CNF_FRAME_TIMEOUT_FACTOR(oct2_reg,\
	(onOff == oct2_on)?OCT2_CNF_FRAME_TIMEOUT_FACTOR_FACTOR128 :\
	OCT2_CNF_FRAME_TIMEOUT_FACTOR_FACTOR1);
#define OCT2_CONF_ARB_ENDIAN_LITTLE(onOff) \
	setOct_OCT2_CNF_ARBITER_ENDIAN(oct2_reg,\
	(onOff == oct2_on)?OCT2_CNF_ARBITER_ENDIAN_LITTLE :\
	OCT2_CNF_ARBITER_ENDIAN_BIG)
#define OCT2_SET_CRC_INIT_ONES(onOff) \
	setOct_OCT2_CRC_CNF_INIT(oct2_reg,\
	(onOff == oct2_on)?OCT2_CRC_CNF_INIT_ONES :\
	OCT2_CRC_CNF_INIT_ZEROS)
#define OCT2_GET_PG_TIMEOUT getOct_OCT2_CNF_PAGE_TIMEOUT(oct2_reg)
#define OCT2_GET_CNF_VAL ((unsigned int)getOct_OCT2_CNF(oct2_reg))

#endif
