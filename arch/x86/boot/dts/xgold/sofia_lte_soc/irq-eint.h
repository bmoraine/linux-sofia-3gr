/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _EINT_IRQ_H
#define _EINT_IRQ_H

/* Int types definitions for XGOLD IRQ subdomains only (EINT, ABB, PMU...) */
#define XGOLD_IRQ_TYPE_NONE 0
#define XGOLD_IRQ_TYPE_EDGE_RISING 1
#define XGOLD_IRQ_TYPE_EDGE_FALLING 2
#define XGOLD_IRQ_TYPE_EDGE_BOTH (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
#define XGOLD_IRQ_TYPE_LEVEL_HIGH 4
#define XGOLD_IRQ_TYPE_LEVEL_LOW 8

/* EINT nodes from EINT domain point of view */
#define EINT_EXI0 0
#define EINT_EXI1 1
#define EINT_EXI2 2
#define EINT_EXI3 3
#define EINT_EXI4 4
#define EINT_EXI5 5
#define EINT_EXI6 6
#define EINT_EXI7 7
#define EINT_USIF2_WK 8
#define EINT_USIF3_WK 9
#define EINT_WDT0 10
#define EINT_USIF4_WK 11
#define EINT_U2H 12
#define EINT_G3FP 13
#define EINT_USB_HS_RESUME 14
#define EINT_USIF7_INT 15
#define EINT_I2C5_WK 16
#define EINT_I2C4_WK 17
#define EINT_I2C3_WK 18
#define EINT_I2C2_WK 19
#define EINT_I2C1_WK 20
#define EINT_USB_VBUSDETECT 21
#define EINT_USIF1_WK 22
#define EINT_DAP_WK 23
#define EINT_EXI12 24
#define EINT_EXI13 25
#define EINT_EXI14 26
#define EINT_EXI15 27
#define EINT_EXI8 28
#define EINT_EXI9 29
#define EINT_EXI10 30
#define EINT_EXI11 31
#define EINT_OCP_DATA_ABORT 32
#define EINT_CORE0_C6_ENTRY_INT 33
#define EINT_CORE0_C6_EXIT_INT 34
#define EINT_CORE1_C6_ENTRY_INT 35
#define EINT_CORE1_C6_EXIT_INT 36
#define EINT_CORE2_C6_ENTRY 37
#define EINT_CORE2_C6_EXIT 38
#define EINT_CORE3_C6_ENTRY 39
#define EINT_CORE3_C6_EXIT 40
#define EINT_VM_ID_VIOLATION 41
#define EINT_SDMMC_DETECT 42
#define EINT_SDIO_DAT1 43
#define EINT_SDIO_DAT3 44

#endif /* _EINT_IRQ_H */
