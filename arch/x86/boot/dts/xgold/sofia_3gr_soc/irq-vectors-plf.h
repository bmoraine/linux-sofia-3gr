/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Some vectors may be platform dependent such as EINT
 * Move those vectors from irq-vectors to there :g!/EXI/d
 * use #ifdef MY_PLATFORM in case of shared IRQ
 * */

#ifndef _IRQ_VECTORS_PLF_H
#define _IRQ_VECTORS_PLF_H
#define EXI13 0
#define EXI15 0
#define EXI9 0
#define EXI11 0
#define EXI0 56
#define EXI1 57
#define EXI2 58
#define EXI12 61
#define EXI8 118
#define EXI14 121
#define EXI10 122
#define EXI3 123
#ifdef SOFIA3GR_MRD6
#define EXI5 60
#define EXI6 0
#else
#define EXI5 0
#define EXI6 60
#endif
#define EXI4 194
#define EXI7 195
#endif /*_IRQ_VECTORS_PLF_H */
