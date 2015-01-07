/*
 * early_printk_xgold.c - early consoles for Intel XGOLD platforms
 *
 * Copyright (c) 2013, Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kmsg_dump.h>
#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>

#include <asm/fixmap.h>

#define USIF_TXD(_base) ((_base) + 0x40000)
#define USIF_FIFO_STAT_TXFFS(_reg) (((_reg) & 0xff0000) >> 0x10)
#define USIF_FIFO_STAT(_base) ((_base) + 0x44)
#define USIF_TPS_CTRL(_base) ((_base) + 0x40)
#define USIF_CLC(_base) ((_base) + 0x0)
#define USIF_FDIV_CFG(_base) ((_base) + 0x140)
#define USIF_CLC_CNT(_base) ((_base) + 0x4)
#define USIF_MODE_CFG(_base) ((_base) + 0x110)
#define USIF_PRTC_CFG(_base) ((_base) + 0x114)
#define USIF_BC_CFG(_base) ((_base) + 0x144)
#define USIF_FDIV_CFG(_base) ((_base) + 0x140)
#define USIF_FIFO_CFG(_base) ((_base) + 0x30)
#define USIF_FIFO_CTRL(_base) ((_base) + 0x34)
#define USIF_IMSC(_base) ((_base) + 0x84)
#define USIF_ICR(_base) ((_base) + 0x98)

#define USIF_CLC_MOD_EN_OFFSET 0x2
#define USIF_CLC_MOD_EN_EN_REQ (0x1 << USIF_CLC_MOD_EN_OFFSET)
#define USIF_CLC_RUN_OFFSET 0x0
#define USIF_CLC_RUN_STOP (0x2 << USIF_CLC_RUN_OFFSET)
#define USIF_CLC_RUN_RUN (0x1 << USIF_CLC_RUN_OFFSET)

#define USIF_WORKING_STAT	(USIF_CLC_MOD_EN_EN_REQ | USIF_CLC_RUN_RUN)
#define USIF_CONFIG_STAT	(USIF_CLC_MOD_EN_EN_REQ | USIF_CLC_RUN_STOP)

static void __iomem *io_base;
static void __iomem *io_tx_base;

void xgold_early_console_init(unsigned long base)
{
	/* if uart_base is not specified,
	 * use CONFIG_DEBUG_XGOLD_USIFX_ADDR or USIF1 */
	if (!base)
		base = CONFIG_DEBUG_XGOLD_USIFX_ADDR;

	io_base = (void *)set_fixmap_offset_nocache(FIX_EARLYCON_MEM_BASE,
			base);

	io_tx_base = (void *)set_fixmap_offset_nocache(FIX_EARLYCON_TX_BASE,
			USIF_TXD(base));

	/* Reconfigure USIF termios
	 *	So far, let's assume the bootloader is doing it
	*/
	writel(0xAA6, USIF_CLC(io_base));
	writel(1, USIF_CLC_CNT(io_base));
	/* set up other parameters, like parity ...*/
	writel(0x10806, USIF_MODE_CFG(io_base));
	writel(0x8, USIF_PRTC_CFG(io_base));
	writel(0x0, USIF_IMSC(io_base));
	writel(0xFFFFFFFF, USIF_ICR(io_base));
	/* set up baudrate */
	writel(0x5, USIF_BC_CFG(io_base));
	writel(0x1d00037, USIF_FDIV_CFG(io_base));
	writel(0x2222, USIF_FIFO_CFG(io_base));
	writel(0xAA5, USIF_CLC(io_base));

	/* TODO: Add kmsg_dumper ?*/

}

static inline void early_xgold_usif_putc(char s)
{
	while (USIF_FIFO_STAT_TXFFS(readl(USIF_FIFO_STAT(io_base))))
		barrier();

	writel(1, USIF_FIFO_CTRL(io_base));
	writel(s, io_tx_base);
}

static void early_xgold_usif_write(struct console *con,
					const char *str, unsigned n)
{
	int i;

	for (i = 0; i < n && *str; i++) {
		if (*str == '\n')
			early_xgold_usif_putc('\r');
		early_xgold_usif_putc(*str);
		str++;
	}
}

struct console early_xgold_console = {
	.name =		"earlyxgold",
	.write =	early_xgold_usif_write,
	.flags	=	CON_PRINTBUFFER | CON_BOOT,
	.index =	-1,
};

