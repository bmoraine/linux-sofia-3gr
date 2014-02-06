/*
* Copyright (C) 2011-2013 Intel Mobile Communications GmbH
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

/* Register offsets*/

#define KPD_ID		0x0008
#define KPD_MIS		0x0088
#define KPD_MISC	0x0084
#define KPD_CONFIG	0x0010
#define KPD_ICR		0x008C
#define KPD_KEYNUM1 0x0018
#define KPD_KEYNUM2 0x001c
#define KPD_KEYNUM3 0x0020

/* Register values*/

#define KPD_INTR_DISABLE	0x0
#define KPD_INTR_ENABLE		0xB
#define DEB_LENGTH_MASK		(~0xF)
#define DEB_LENGTH		0x0

#define KEY_INT0			0x1
#define KEY_INT1			0x2
#define KEY_INT2			0x4
#define KEY_INT3			0x8

#define TESTBIT(A, B) ((A) & (1u << (B)))
