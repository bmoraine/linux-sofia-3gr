/* ----------------------------------------------------------------------------
   Copyright (C) 2014 Intel Mobile Communications GmbH

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

  ---------------------------------------------------------------------------*/
#ifndef _SOFIA_IRQ_VECTORS_H
#define _SOFIA_IRQ_VECTORS_H

#define RESCHEDULE_VECTOR		(CONFIG_LOCAL_APIC_TIMER_VECTOR_NR+4)
#define CALL_FUNCTION_VECTOR	(CONFIG_LOCAL_APIC_TIMER_VECTOR_NR+3)
#define CALL_FUNCTION_SINGLE_VECTOR	(CONFIG_LOCAL_APIC_TIMER_VECTOR_NR+2)
#define REBOOT_VECTOR			(CONFIG_LOCAL_APIC_TIMER_VECTOR_NR+1)
#define LOCAL_TIMER_VECTOR		CONFIG_LOCAL_APIC_TIMER_VECTOR_NR

#endif /* _SOFIA_IRQ_VECTORS_H */
