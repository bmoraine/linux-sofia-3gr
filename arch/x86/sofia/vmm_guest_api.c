/* ----------------------------------------------------------------------------
 *  Copyright (C) 2014 Intel Mobile Communications GmbH

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

/*
 * NOTES:
 * 1) This source file is included in guests including Linux and purposely
 * kept in line with Linux kernel coding guidelines for ease of portability and
 * maintainability. See linux/Documentation/CodingStyle for coding guidelines.
 * Please check all further changes are compliant with Linux coding standard
 * and clear all commands reported by Linux checkpatch.pl
 * Use command: linux/scripts/checkpatch.pl -f <filename>
 * Clear ALL warnings and errors reported.
 *
 * 2) Use only C99 fixed width types for definition as this header needs to be
 * both 32bit/64bit portable.
 * Avoid the use of pointers/enum in structure as that make the structure
 * variable size based on 32/64bit toolchain used.
*/

#ifdef __KERNEL__
#include <linux/module.h>
#include <sofia/vmm_guest_api.h>
#include <sofia/vmcalls.h>
#include <linux/sysprofile.h>
#else
#include "vmm_guest_api.h"
#include "vmcalls.h"
#endif

#define VMCALL_OPCODE   ".byte 0x0f,0x01,0xc1\n"

#define UNUSED_ARG	0

static inline int32_t vmm_call(uint32_t nr, uint32_t arg0,
			   uint32_t arg1, uint32_t arg2,
			   uint32_t arg3)
{
	uint32_t ret = 0;

	asm volatile (VMCALL_OPCODE : "=a"(ret)
		      : "a"(nr), "b"(arg0), "c"(arg1), "d"(arg2), "S"(arg3)
		      : "memory");

	return ret;
}

static inline int vmm_call_5(uint32_t nr, uint32_t arg0,
			     uint32_t arg1, uint32_t arg2,
			     uint32_t arg3, uint32_t *ret0,
			     uint32_t *ret1, uint32_t *ret2,
			     uint32_t *ret3, uint32_t *ret4)
{
	uint32_t results[5];

	asm volatile (VMCALL_OPCODE : "=a"(results[0]), "=b"(results[1]),
		      "=c"(results[2]), "=d"(results[3]), "=S"(results[4])
		      : "a"(nr), "b"(arg0), "c"(arg1), "d"(arg2), "S"(arg3)
		      : "memory");

	if (ret0)
		*ret0 = results[0];
	if (ret1)
		*ret1 = results[1];
	if (ret2)
		*ret2 = results[2];
	if (ret3)
		*ret3 = results[3];
	if (ret4)
		*ret4 = results[4];

	return results[0];
}

void vmm_guest_idle(void)
{
	vmm_call(VMCALL_IDLE, UNUSED_ARG, UNUSED_ARG, UNUSED_ARG, UNUSED_ARG);
}

uint32_t vmm_vcpu_id(void)
{
	return vmm_call(VMCALL_VCPU_ID, UNUSED_ARG, UNUSED_ARG, UNUSED_ARG,
			UNUSED_ARG);
}

void vmm_guest_log(void)
{
	vmm_call(VMCALL_VM_LOG, UNUSED_ARG, UNUSED_ARG, UNUSED_ARG, UNUSED_ARG);
}

void vmm_guest_request_virq(uint32_t virq, uint32_t vaffinity)
{
	vmm_call(VMCALL_GUEST_REQUEST_VIRQ, virq, vaffinity, UNUSED_ARG,
		 UNUSED_ARG);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(vmm_guest_request_virq);
#endif

void vmm_virq_eoi(uint32_t virq)
{
	vmm_call(VMCALL_VIRQ_EOI, virq, UNUSED_ARG, UNUSED_ARG, UNUSED_ARG);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(vmm_virq_eoi);
#endif

void vmm_virq_mask(uint32_t virq)
{
	vmm_call(VMCALL_VIRQ_MASK, virq, UNUSED_ARG, UNUSED_ARG, UNUSED_ARG);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(vmm_virq_mask);
#endif

void vmm_virq_unmask(uint32_t virq)
{
	vmm_call(VMCALL_VIRQ_UNMASK, virq, UNUSED_ARG, UNUSED_ARG, UNUSED_ARG);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(vmm_virq_unmask);
#endif

void vmm_set_vaffinity(uint32_t virq, uint32_t vaffinity)
{
	vmm_call(VMCALL_SET_VAFFINITY, virq, vaffinity, UNUSED_ARG, UNUSED_ARG);
}

/* Lookup vlink by name */
/*
vmm_paddr_t vmm_vlink_lookup(vmm_paddr_t name, vmm_paddr_t vlink)
{
	return (vmm_paddr_t)vmm_call(VMCALL_VLINK_LOOKUP, (uint32_t)name,
				(uint32_t)vlink, UNUSED_ARG, UNUSED_ARG);
}
*/
/* retreives vlink db physical address */
vmm_paddr_t vmm_get_vlink_db()
{
	return (vmm_paddr_t) vmm_call(VMCALL_GET_VLINK_DB, UNUSED_ARG,
				      UNUSED_ARG, UNUSED_ARG, UNUSED_ARG);
}

/* Request virtual interrupt as per vlink & resource id combo <vlink, id> */
uint32_t vmm_xirq_alloc(vmm_paddr_t vlink, uint32_t resource_id,
			    uint32_t os_id, int32_t num_int)
{
	return vmm_call(VMCALL_XIRQ_ALLOC, (uint32_t)vlink, resource_id,
			os_id, num_int);
}

/* Trigger a virtual interrupt */
void vmm_xirq_post(uint32_t xirq, uint32_t os_id)
{
	sysprof_xirq_to_mex(xirq);
	vmm_call(VMCALL_XIRQ_POST, xirq, os_id, UNUSED_ARG, UNUSED_ARG);
}

/* Trigger an IPI to the specified vcpus bitmap */
void vmm_ipi_post(uint32_t virq, uint32_t vcpus)
{
	sysprof_xirq_to_linux(virq);
	vmm_call(VMCALL_IPI_POST, virq, vcpus, UNUSED_ARG, UNUSED_ARG);
}

vmm_paddr_t vmm_shared_mem_alloc(vmm_paddr_t vlink, uint32_t resource_id,
				 uint32_t size)
{
	return (vmm_paddr_t) vmm_call(VMCALL_SHARED_MEM_ALLOC,
				      (uint32_t)vlink, resource_id, size,
				      UNUSED_ARG);
}

void vmm_start_vcpu(uint32_t vcpu_id, uint32_t entry_addr)
{
	vmm_call(VMCALL_START_VCPU, vcpu_id, entry_addr, UNUSED_ARG,
		 UNUSED_ARG);
}

void vmm_stop_vcpu(uint32_t vcpu_id)
{
	vmm_call(VMCALL_STOP_VCPU, vcpu_id, UNUSED_ARG, UNUSED_ARG, UNUSED_ARG);
}

void vmm_guest_dummy_test(unsigned char *ptr)
{
	vmm_call(VMCALL_VM_DUMMY, (uint32_t)ptr, UNUSED_ARG, UNUSED_ARG,
		 UNUSED_ARG);
}

struct vmm_shared_data *vmm_get_vcpu_data(void)
{
	return (struct vmm_shared_data *) vmm_call(VMCALL_GET_VCPU_DATA,
						UNUSED_ARG, UNUSED_ARG,
						UNUSED_ARG, UNUSED_ARG);
}

int vmm_vcpu_has_irq_pending(void)
{
	return (int)vmm_call(VMCALL_VCPU_HAS_IRQ_PENDING, UNUSED_ARG,
			     UNUSED_ARG, UNUSED_ARG, UNUSED_ARG);
}

void vmm_call_multiple_return(uint32_t *ret0, uint32_t *ret1,
			      uint32_t *ret2, uint32_t *ret3,
			      uint32_t *ret4)
{
	vmm_call_5(VMCALL_VM_DUMMY, UNUSED_ARG, UNUSED_ARG, UNUSED_ARG,
		   UNUSED_ARG, ret0, ret1, ret2, ret3, ret4);
}

uint32_t vmm_platform_service(uint32_t service_type, uint32_t arg2,
			uint32_t arg3, uint32_t arg4,
			uint32_t *ret0, uint32_t *ret1,
			uint32_t *ret2, uint32_t *ret3,
			uint32_t *ret4)
{
	return vmm_call_5(VMCALL_PLATFORM_SERVICE, service_type, arg2,
			arg3, arg4, ret0, ret1, ret2, ret3, ret4);
}

uint32_t vmm_get_running_guests(void)
{
	return (uint32_t)vmm_call(VMCALL_GET_RUNNING_GUESTS, UNUSED_ARG,
			     UNUSED_ARG, UNUSED_ARG, UNUSED_ARG);
}

int vmm_initiate_reboot(uint32_t reboot_action)
{
	return (int)vmm_call(VMCALL_INITIATE_REBOOT, reboot_action,
			     UNUSED_ARG, UNUSED_ARG, UNUSED_ARG);
}

void vmm_virq_spurious(uint32_t virq)
{
	vmm_call(VMCALL_VIRQ_SPURIOUS, virq,
			UNUSED_ARG, UNUSED_ARG, UNUSED_ARG);
	return;
}

