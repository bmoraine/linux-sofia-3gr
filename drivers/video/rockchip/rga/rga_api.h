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

#ifndef __RGA_API_H__
#define __RGA_API_H__

#include "rga_reg_info.h"
#include "rga.h"

#define ENABLE      1
#define DISABLE     0

int32_t rga_gen_two_pro(struct rga_req *msg, struct rga_req *msg1);

#endif	/*  */
