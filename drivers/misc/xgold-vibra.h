/*
* Copyright (C) 2014 Intel Mobile Communications GmbH
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

#ifndef _XGOLD_VIBRA_H_
#define _XGOLD_VIBRA_H_

#define VIBRA_CONTROL	0x0004
#define VIBRA_STATUS	0x0000
#define VIBRA_DOWN	0x400
#define VIBRA_UP	0x480
#define VIBRA_SELFTEST	0x40000400
#define VIBRA_ST_PASS	0x1

struct xgold_vibra_platform_data {
	int (*init)(struct device *dev);
	void (*exit)(void);
	int (*set_clk)(struct device *dev, bool on);
	int (*set_vibrator)(struct device *dev, void __iomem *mmio_base,
	spinlock_t *lock, int intensity);
};
#endif
