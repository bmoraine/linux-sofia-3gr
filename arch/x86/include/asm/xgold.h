/*
 * xgold.h: Intel XGOLD platform specific setup code
 *
 * (C) Copyright 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _ASM_X86_XGOLD_H
#define _ASM_X86_XGOLD_H
extern struct console early_xgold_console;
extern void xgold_early_console_init(unsigned long base);

extern bool xgold_platform_needs_broadcast_timer(void);
#endif
