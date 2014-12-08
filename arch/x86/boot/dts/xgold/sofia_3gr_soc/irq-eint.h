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
#define EINT_EXI8 8
#define EINT_EXI9 9
#define EINT_EXI10 10
#define EINT_EXI11 11
#define EINT_EXI12 12
#define EINT_EXI13 13
#define EINT_EXI14 14
#define EINT_EXI15 15
#define EINT_USB_HS_RESUME 16
#define EINT_SDMMC_DETECT 17
#define EINT_SDIO_DAT3 18
#define EINT_SDIO_DAT1 19
#define EINT_USIF1_WK 20
#define EINT_USIF2_WK 21
#define EINT_WUP_DBB 22
#define EINT_U2H 23
#define EINT_G3FP 24
#define EINT_DTXS 25
#define EINT_DRXS 26
#define EINT_WDT0 27
#define EINT_WDT1 28
#define EINT_USB_ID 29

#endif /* _EINT_IRQ_H */
