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

#define EXI3 0
#define EXI4 0
#define EXI5 0
#define EXI7 0
#define EXI13 0
#define EXI15 0
#define EXI9 0
#define EXI10 0
#define EXI0 55
#define EXI1 56
#define EXI2 57
#define EXI12 60
#define EXI8 115
#define EXI14 116

#ifdef SOFIA3G_ES2_TAB_SVB
	#define EXI6 59
	#define EXI11 0
#else
	#define EXI6 0
	#define EXI11 59
#endif

#endif /*_IRQ_VECTORS_PLF_H */
