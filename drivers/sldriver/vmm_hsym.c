/**
 * vmm_hsym.c  This file implements SL VMM hypercalls.
 * Copyright (C) 2015 Intel Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.>See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 *
 * Author:
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include "include/vmm_hsym.h"
#include "include/vmm_hsym_common.h"
#include <sofia/mv_svc_hypercalls.h>

#define  VMCALL_PLATFORM_SERVICE    (999)
#define VMM_SILENTLAKE_SERVICES     (1000)
enum {
	no_rights = 0,
	r_rights = 1,
	w_rights = 2,
	x_rights = 4,
	rw_rights = 3,
	rx_rights = 5,
	rwx_rights = 7,
	all_rights = 7
} perm_t;

inline int info_cpuid(sl_info_t *p_info)
{
	uint32_t eax = SL_CMD_HSEC_GET_INFO, ebx = 0, ecx = 0, edx = 0;

	if (!p_info)
		return -1;

	asm volatile ("cpuid\n" : "=a" (eax), "=b"(ebx), "=c"(ecx), "=d"(edx)
		      : "0"(eax)
		      : "memory");
	p_info->major_version = (eax >> 16);
	p_info->minor_version = (eax & 0xffff);
	p_info->vendor_id[0] = (ebx >> 16);
	p_info->vendor_id[1] = ((ebx >> 8) & 0xff);
	p_info->vendor_id[2] = (ebx & 0xff);
	return 0;
}

#ifdef __x86_64__
inline int cpuid_asm64(uint32_t leaf, uint32_t b_val, uint64_t c,
		       uint64_t d, uint64_t S, uint64_t D)
{
	int status;
	asm volatile ("cpuid" : "=a" (status), "+g"(b_val), "+c"(c), "+d"(d)
		      : "0"(leaf), "m"(b_val), "S"(S), "D"(D)
		      : );
	return status;
}
#endif

#ifdef __x86_64__
int request_sl_service(uint32_t service_leaf, uint64_t param_a,
		       uint64_t param_b, uint64_t S, uint64_t D)
{
	return cpuid_asm64(service_leaf, 0, param_a, param_b, S, D);
}
#else
#ifdef CONFIG_X86_INTEL_SOFIA
int request_sl_service(uint32_t service_leaf, uint32_t param_a,
		       uint32_t param_b, uint32_t
		       __attribute__ ((unused)) S, uint32_t
		       __attribute__ ((unused)) D)
{
	return mv_svc_silentlake_op(service_leaf, param_a, param_b);
}
#endif
#endif

void reg_vIDT(void *data)
{
	uint64_t param_addr;
	hsec_vIDT_param_t *vIDT_info = (hsec_vIDT_param_t *) data;

	if (!vIDT_info) {
		pr_err("hypersec: vIDT_info is invalid\n");
		return;
	}

	param_addr = virt_to_phys(vIDT_info);
#ifdef __x86_64__
	request_sl_service(SL_CMD_HSEC_REG_VIDT, (uint64_t) param_addr,
			   sizeof(hsec_vIDT_param_t), 0, vIDT_info->cpu);
#else
	request_sl_service(SL_CMD_HSEC_REG_VIDT, (uint32_t) param_addr,
			   sizeof(hsec_vIDT_param_t), 0, vIDT_info->cpu);
#endif
	return;
}
EXPORT_SYMBOL(reg_vIDT);

void reg_sl_global_info(hsec_sl_param_t *sl_info)
{
	uint64_t param_addr;

	if (!sl_info) {
		pr_err("hypersec: sl_info is invalid\n");
		return;
	}

	param_addr = virt_to_phys(sl_info);

#ifdef __x86_64__
	request_sl_service(SL_CMD_HSEC_REG_SL_INFO, (uint64_t) param_addr,
			   sizeof(hsec_sl_param_t), 0, 0);
#else
	request_sl_service(SL_CMD_HSEC_REG_SL_INFO, (uint32_t) param_addr,
			   sizeof(hsec_sl_param_t), 0, 0);
#endif
	return;
}
EXPORT_SYMBOL(reg_sl_global_info);
