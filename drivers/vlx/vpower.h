/*
 ****************************************************************
 *
 *  Component: VLX virtual Power Management frontend driver
 *
 *  Copyright (C) 2011 Intel Mobile Communications GmbH
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************
 */

#ifndef __VPOWER_H
#define __VPOWER_H

#include <linux/types.h>

ePRH_RETURN_T vpower_call_prh(uint32_t user_id,
			uint32_t per_id,
			uint32_t * const p_per_mode_info,
			uint32_t size);

int vpower_pmuirq_config(unsigned int irqnum, bool mask);
int process_vpmu_irq(unsigned int irq);
int xgold_cpu_load_get(int id);
int vpower_set_chargepump_mode(const nku32_f mode);
#endif /* __VPOWER_H */
