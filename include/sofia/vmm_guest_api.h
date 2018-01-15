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

/* For inclusion by Guest VMs only! */

#ifndef _VMM_GUEST_API_H
#define _VMM_GUEST_API_H

#include "vmm_guest_ipc.h"

#define VMM_XIRQ_START      768
#define VMM_XIRQ_END        1024
#define VMM_XIRQ_SYSCONF    768
#define VMM_HIRQ_START      512
#define VMM_HIRQ_END        768

/* NOTE: must be consistent with mvconfig.h */
#define CONFIG_MAX_VCPUS_PER_VM	8

#define vmm_id_t uint32_t

/**
 * @brief Shared data between the MobileVisor and the guest
 *
 * This data structure defines the shared data between
 * the MobileVisor and the guest.
 */
struct vmm_shared_data {
	/** @brief Guest OS ID
	 *
	 * Each guest has a unique ID. This is used for various
	 * IPC APIs such as xirq posting.
	 * For SMP guests, each VCPU will have the same OS ID.
	 */
	const uint32_t os_id;

	/** @brief Shared memory start address
	 *
	 * This field contains the physical start address for
	 * the shared memory region.
	 * The guest is expected to map in the shared memory
	 * region into its virtual address space.
	 */
	const uint32_t pmem_paddr;

	/** @brief Shared memory size
	 *
	 * This field contains the size of the shared memory region.
	 * The guest is expected to map in the shared memory region
	 * into its virtual address space.
	 */
	const uint32_t pmem_size;

	/** @brief OS command line for the guest
	 *
	 * Each guest will have a command line. Apart from the usual
	 * parameters (e.g. in the case of Linux), it also contains
	  * the virtual device information
	 */
	const uint8_t vm_cmdline[1024];

	/** @brief Active xirq
	 *
	 * The latest xirq that has been triggered.
	 * Used only by the guest adaption layer to identify
	 * the xirq and trigger the handler chain.
	 */
	const uint32_t triggering_xirq;

	/** @brief PM control shared data
	 *
	 */
	/*pm_control_shared_data_t pm_control_shared_data;*/

	/** @brief System idle flag
	 *
	 * Only used by VCPU who is the power manager owner for a physical CPU
	 * If set, indicates that system is idle,
	 * i.e. no pending tasks or interrupts
	 */
	uint32_t system_idle;

	/** @brief Per processor data structure
	 *
	 * Used to identify GDT and IDT for each VCPU
	 */
	uint32_t pcr[CONFIG_MAX_VCPUS_PER_VM];

	/** @brief Logging buffer for debug purpose
	 *
	 * This is used internally by the MicroVisor logging API.
	 */
	char vm_log_str[512];

	/** @brief Shared data for pal
	 *
	 * This is used internally for PAL shared data
	 */
	uint32_t pal_shared_mem_data[256];

	/** @brief Platform reboot initiation status
	 *
	 * Indicates if a platform reboot has been initiated.
	 * If >0, platform reboot has been initiated,
	 * guests should perform reboot housecleaning accordingly.
	 * and finally invoke VMCALL_STOP_VCPU for each vcpu.
	 */
	const uint32_t system_reboot_action;
};

/** @brief Informs MobileVisor that the guest is now idle
 *
 * Upon the idle call, MobileVisor will no longer schedule the guest until
 * an interrupt for this guest arrives.
 */
void vmm_guest_idle(void);

/** @brief Retrieves the caller's VCPU ID
 *
 * The guest's VCPU ID is a zero-based number in the range between 0 to n-1
 * where n is the number of VCPUs the guest VM supports
 */
uint32_t vmm_vcpu_id(void);

/** @brief Print the vmm log into the shared buffer
 *
 *  Log to vmm console the print format string
 *  in guest vmm_shared_data->vm_log_str, then the
 *  guest can see the vmm core log.
 */
void vmm_guest_log(void);

/** @brief Connect the virtual interrupt to the guest
 *
 * The specified virq must have a properly configured entry in mvconfig
 * and the calling guest must be the owner of this vector
 *
 * Upon the call, vmm will program the IOAPIC for the corresponding
 * interrupt line with the specified vector number. The interrupt will
 * remain masked
 *
 *  @param virq irq vector number.
 *  @param vaffinity specify which vcpu will process this irq.
 */
void vmm_guest_request_virq(uint32_t virq, uint32_t vaffinity);

/** @brief EOI a virtual interrupt
 *
 * After virtual interrupt servicing, EOI must be performed in order
 * for the vmm to deliver the next interrupt
 *
 *  @param virq irq vector number.
 */
void vmm_virq_eoi(uint32_t virq);

/** @brief Mask a virtual interrupt
 *
 * Mask the specifid virtual interrupt. Upon the call, vmm will mask the
 * corresponding interrupt line in the IOAPIC
 *
 *  @param virq irq vector number.
 */
void vmm_virq_mask(uint32_t virq);

/** @brief Unmask a virtual interrupt
 *
 * Unmasking the specified virtual interrupt. Upon the call, vmm will
 * unmask the corresponding interrupt line in the IOAPIC
 *
 *  @param virq irq vector number.
 */
void vmm_virq_unmask(uint32_t virq);

/** @brief Set of the affinity of the virtual interrupt
 *
 * Set the VCPU affinity of the virtual interrupt.
 *
 *  @param virq irq vector number.
 *  @param vaffinity specify which vcpu will process this irq.
 */
void vmm_set_vaffinity(uint32_t virq, uint32_t vaffinity);

/** @brief Retrieve vlink db physical address
 *
 * Returns the physical address of the vlink database
 *
 *  @return Return vlink db physical address.
 */
vmm_paddr_t vmm_get_vlink_db(void);

/** @brief Allocate num of cross interrupt for the specified <vlink,id>
 *
 *  @param vlink Specify the vlink which needs these xirq
 *  @param resource_id Specify which resource will associate with these xirq
 *  @param os_id Specify the destination OS id.
 *  @param num_int Specify how many continuous xirqs need to be allocated.
 *  @return Return 0 if unsuccessful or the number of
 *  the first allocated xirq otherwise
 */
uint32_t vmm_xirq_alloc(vmm_paddr_t vlink, uint32_t resource_id,
			    uint32_t os_id, int32_t num_int);

/** @brief Trigger a cross interrupt.
 *
 *  @param xirq The xirq vector number.
 *  @param os_id Specify the destination OS id.
 */
void vmm_xirq_post(uint32_t xirq, uint32_t os_id);

/** @brief Trigger an IPI to the specified vcpus bitmap
 *
 *  @param virq The virq vector number.
 *  @param vcpus Specify which CPUs want to receive this virq.
 */
void vmm_ipi_post(uint32_t virq, uint32_t vcpus);

/** @brief Allocate shared memory
 *
 *  @param vlink Specify the vlink used to share this memory.
 *  @param resource_id Specify which destination resource
 *  the caller want to share.
 *  @param size Specify the size of the shared memory.
 *
 *  @return The physical address of the allocated shared memory.
 */
vmm_paddr_t vmm_shared_mem_alloc(vmm_paddr_t vlink, uint32_t resource_id,
				 uint32_t size);

/** @brief Start a secondary VCPU
 *
 *  @param vcpu_id Specify which vcpu is started.
 *  @param entry_addr Specify the vcpu start address.
 */
void vmm_start_vcpu(uint32_t vcpu_id, uint32_t entry_addr);

/** @brief Stop a secondary VCPU
 *
 *  @param vcpu_id Specify which vcpu needs to be stopped.
 */
void vmm_stop_vcpu(uint32_t vcpu_id);

/** @brief Returns VMM-vcpu shared data struct vmm_shared_data *
 *    for the calling vcpu
 *
 */
struct vmm_shared_data *vmm_get_vcpu_data(void);

/** @brief Check if the calling vcpu has any pending interrupts.
 *
 *  @return Return 1 if has.
 */
int32_t vmm_vcpu_has_irq_pending(void);

/** @brief Returns mask of currently running guests.
 *
 *  @return The ID of the currenly running guest.
 */
uint32_t vmm_get_running_guests(void);

/**
 *  @brief Initate a platform reboot based on desired reboot action.
 *
 *  @param reboot_action If it is set to 0 , then nothing is to be done.
 *  Actual reboot action definition depends on pal implmenetation
 *  This reboot action value is passed to pal_reboot().
 *
 *  @return Return -1 if unsucessful (eg. reboot action is already pending)
 */
int32_t vmm_initiate_reboot(uint32_t reboot_action);

/**
 *  @brief Report unexpected (spurious) vector to Mobilevisor.
 *
 *  @param vector - spurious vector number
 *
 *  @return None
 */
void vmm_virq_spurious(uint32_t vector);

/** @brief Used for platform service request. This should not be used directly.
 *	   Instead, guest should use functions from vmm_platform_service.h.
 */
uint32_t vmm_platform_service(uint32_t service_type, uint32_t arg2,
			 uint32_t arg3, uint32_t arg4,
			 uint32_t *ret0, uint32_t *ret1,
			 uint32_t *ret2, uint32_t *ret3,
			 uint32_t *ret4);
#endif /* _VMM_GUEST_API_H */
