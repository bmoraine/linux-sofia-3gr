/*
 * rockchip RGA(2D raster graphic acceleration unit) hardware driver.
 *
 * Copyright (C) 2014 Rockchip Electronics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __RGA_MMU_INFO_H__
#define __RGA_MMU_INFO_H__

#include "rga.h"

#ifndef MIN
#define MIN(X, Y)           ((X) < (Y) ? (X) : (Y))
#endif	/*  */

#ifndef MAX
#define MAX(X, Y)           ((X) > (Y) ? (X) : (Y))
#endif	/*  */
int rga_set_mmu_info(struct rga_reg *reg, struct rga_req *req);

extern struct rga_mmu_buf_t rga_mmu_buf;

#endif	/*  */
