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
*/

#ifdef __KERNEL__
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/sysprofile.h>
#include <sofia/vmm_al.h>
#include <asm/pat.h>
#else
#include "common.h"
#include "isr.h"
#include "vmm_al.h"
#endif

/* define VM_MULTIPLE_VCPUS if guest VM has more than 1 VCPU */
#if defined(__KERNEL__)
#ifdef CONFIG_SMP
#define VM_MULTIPLE_VCPUS 1
#endif
#endif

/* FIXME: Do not include platform specific information
 * So I removed the pal_vectors.h inclusion,
 * and hardcode the VECT_XIRQ number
 */

#define VECT_XIRQ 33

#define XIRQ_NUM2INDX(xirq_num)     ((xirq_num) - VMM_XIRQ_START)
#define XIRQ_INDX2NUM(xirq_indx)    ((xirq_indx) + VMM_XIRQ_START)

struct vmm_shared_data *vmm_shared_data[CONFIG_MAX_VCPUS_PER_VM];
const char *vm_command_line;

struct vmm_shared_data *get_vmm_shared_data(void)
{
#ifdef VM_MULTIPLE_VCPUS
	return vmm_shared_data[vmm_vcpu_id()];
#else
	return vmm_shared_data[0];
#endif
}

void vmm_panic(char *panic_msg)
{
	if (panic_msg)
		vmm_printk("PANIC: %s\n", panic_msg);
	else
		vmm_printk("PANIC\n");

	while (1)
		;
}

#if defined(__KERNEL__)
void *pmem_vbase;
#endif
void *vmm_ptov(vmm_paddr_t paddr)
{
#if !defined(__KERNEL__)
	return (void *)paddr;
#else
	if (pmem_vbase) {
		void *ptr;
		struct vmm_shared_data *p_shared_data = get_vmm_shared_data();
		if (paddr >= p_shared_data->pmem_paddr &&
		    paddr <= p_shared_data->pmem_paddr +
		    p_shared_data->pmem_size) {
			ptr = (void *)(pmem_vbase + (paddr -
					p_shared_data->pmem_paddr));
			return ptr;
		} else {
			return (void *)paddr;
		}
	} else
		return NULL;
#endif
}

vmm_paddr_t vmm_vtop(void *vaddr)
{
#if !defined(__KERNEL__)
	return (vmm_paddr_t) vaddr;
#else
	if (pmem_vbase) {
		vmm_paddr_t ptr;
		struct vmm_shared_data *p_shared_data = get_vmm_shared_data();
		if (vaddr >= pmem_vbase &&
			vaddr <= pmem_vbase + p_shared_data->pmem_size) {
			ptr =
				(vmm_paddr_t)(p_shared_data->pmem_paddr +
						(vaddr - pmem_vbase));
			return (vmm_paddr_t) ptr;
		} else
			return (vmm_paddr_t) vaddr;
	} else
		return 0;
#endif
}

struct vmm_xirq_callback_entry {
	unsigned int xirq;
	void (*cb)(void *, uint32_t);
	void *cookie;
	struct vmm_xirq_callback_entry *next;
};


static struct vmm_xirq_callback_entry *xirq_callbacks[VMM_XIRQ_END -
							VMM_XIRQ_START];
static unsigned int myid;

#if !defined(__KERNEL__)
#define MAX_XIRQ_CALLBACK_ENTRIES 128
static struct vmm_xirq_callback_entry
			available_entries[MAX_XIRQ_CALLBACK_ENTRIES];
static unsigned int used_entries;
#endif

#ifdef __KERNEL__
static irqreturn_t vmm_al_xirq_handler(int irq, void *data)
#else
static void vmm_al_xirq_handler(registers_t *regs)
#endif
{
	struct vmm_xirq_callback_entry *entry;
	unsigned int idx;
	struct vmm_shared_data *p_shared_data = get_vmm_shared_data();

	idx = XIRQ_NUM2INDX(p_shared_data->triggering_xirq);
	entry = xirq_callbacks[idx];
	while (entry) {
		if (entry->cb) {
			sysprof_interrupt(p_shared_data->triggering_xirq);
			sysprof_int_enter();
			entry->cb(entry->cookie,
				p_shared_data->triggering_xirq);
			sysprof_int_leave();
		}
		entry = entry->next;
	};

#ifdef __KERNEL__
	return IRQ_HANDLED;
#endif
}

void vmm_al_init(struct vmm_shared_data *data)
{
	memset(xirq_callbacks, 0, sizeof(xirq_callbacks));
#if !defined(__KERNEL__)
	vmm_printk("used_entries=0x%x\n", used_entries);
	memset(available_entries, 0, sizeof(available_entries));
	vmm_printk("VM cmdline: %s\n", get_vmm_shared_data()->vm_cmdline);
#endif

	myid = data->os_id;
	vm_command_line = vmm_ptov((vmm_paddr_t) data->vm_cmdline);

	/* register XIRQ handler */
#ifdef __KERNEL__
	if (request_irq
		(VECT_XIRQ, vmm_al_xirq_handler, 0, "xirq_handers", NULL)) {
		pr_err("Reqest IRQ VECT_XIRQ failed!\n");
	}
#else
	register_interrupt_handler(VECT_XIRQ, &vmm_al_xirq_handler);
	/* inform vmm that we want to handle VMM_XIRQ_VECTOR
	 * sincecos register_interrupt does not do it */
	vmm_guest_request_virq(VECT_XIRQ, 1);
	vmm_virq_unmask(VECT_XIRQ);
#endif
}

#if defined(__KERNEL__)
int sofia_vmm_init_secondary(void)
{
	void *ptr;
	unsigned long flag = _PAGE_CACHE_WB;

	pr_debug("In sofia_vmm_init_secondary\n");
	vmm_guest_request_virq(LOCAL_TIMER_VECTOR, 1);
	vmm_virq_unmask(LOCAL_TIMER_VECTOR);
	vmm_guest_request_virq(RESCHEDULE_VECTOR, 1);
	vmm_virq_unmask(RESCHEDULE_VECTOR);
	vmm_guest_request_virq(CALL_FUNCTION_VECTOR, 1);
	vmm_virq_unmask(CALL_FUNCTION_VECTOR);
	vmm_guest_request_virq(CALL_FUNCTION_SINGLE_VECTOR, 1);
	vmm_virq_unmask(CALL_FUNCTION_SINGLE_VECTOR);

	ptr = vmm_get_vcpu_data();
	if (io_reserve_memtype((resource_size_t)ptr, (resource_size_t)(ptr +
		sizeof(struct vmm_shared_data)), &flag))
		pr_err("Unable to map VMM shared data\n");
	vmm_shared_data[vmm_vcpu_id()] = phys_to_virt((phys_addr_t)ptr);
	pr_debug("ptr=0x%08X vmm_shared_data=0x%08X\n",
	       (unsigned int)ptr, (unsigned int)get_vmm_shared_data());

	return 0;
}

static int __init sofia_vmm_init(void)
{
	void *ptr;
	unsigned long flag = _PAGE_CACHE_WB;

	pr_debug("In sofia_vmm_init\n");
	ptr = vmm_get_vcpu_data();
	if (io_reserve_memtype((resource_size_t)ptr, (resource_size_t)(ptr +
		sizeof(struct vmm_shared_data)), &flag))
		pr_err("Unable to map VMM shared data\n");
	vmm_shared_data[vmm_vcpu_id()] = phys_to_virt((phys_addr_t)ptr);
	pr_debug("ptr=0x%08X vmm_shared_data=0x%08X\n",
	       (unsigned int)ptr, (unsigned int)get_vmm_shared_data());
	if (!get_vmm_shared_data())
		panic("Unable to map VMM shared data!\n");

	pmem_vbase =
	    ioremap_cache(get_vmm_shared_data()->pmem_paddr,
			  get_vmm_shared_data()->pmem_size);
	pr_debug(
	       "pmem_paddr=0x%08X pmem_size=0x%08X pmem_vbase=0x%08X\n",
	       get_vmm_shared_data()->pmem_paddr,
	       get_vmm_shared_data()->pmem_size,
	       (unsigned int)pmem_vbase);
	if (!pmem_vbase)
		pr_err("Unable to map PMEM\n");

	pr_debug("Calling vmm_al_init. os_id=%d\n",
	       get_vmm_shared_data()->os_id);
	vmm_al_init(get_vmm_shared_data());

	return 0;
}

early_initcall(sofia_vmm_init);
#endif

inline unsigned int vmm_myid(void)
{
	return myid;
}

void *vmm_register_xirq_callback(uint32_t xirq,
				void (*cb)(void *, uint32_t), void *cookie)
{
	struct vmm_xirq_callback_entry *newentry;
	struct vmm_xirq_callback_entry *entry;
	unsigned int idx;

	if (xirq < VMM_XIRQ_START || xirq >= VMM_XIRQ_END || !cb)
		return NULL;
	/* Allocate memory for each callback entry from OS. */
#ifdef __KERNEL__
	newentry = kmalloc(sizeof(*newentry), GFP_KERNEL);
	if (!newentry)
		return NULL;
#else /* test vm */
	if (used_entries > MAX_XIRQ_CALLBACK_ENTRIES) {
		vmm_printk("Not enough xirq_callback_entries!\n");
		return NULL;
	}
	newentry = &available_entries[used_entries];
	used_entries++;
#endif

	newentry->xirq = xirq;
	newentry->cb = cb;
	newentry->cookie = cookie;
	newentry->next = NULL;

	idx = XIRQ_NUM2INDX(xirq);
	entry = xirq_callbacks[idx];

	if (!entry) {
		xirq_callbacks[idx] = newentry;
	} else {
		while (entry->next)
			entry = entry->next;
		entry->next = newentry;
	}

	return newentry;
}

void vmm_xirq_detach(void *id)
{
	struct vmm_xirq_callback_entry *entry;
	struct vmm_xirq_callback_entry *old;
	unsigned int idx;

	if (id) {
		old = (struct vmm_xirq_callback_entry *)id;
		idx = XIRQ_NUM2INDX(old->xirq);
		entry = xirq_callbacks[idx];

		/* remove callback entry from list */
		if (entry == id) {
			xirq_callbacks[idx] = entry->next;
		} else {
			while (entry->next != id)
				entry = entry->next;
			if (entry->next)
				entry->next = old->next;
		}

		/* release the allocated callback entry struct space */
#ifdef __KERNEL__
		kfree(old);
#else /* test vm? */
		xirq_callbacks[idx] = NULL;
#endif
	}
}

vmm_paddr_t vmm_vlink_lookup(const char *name, vmm_paddr_t plnk)
{
	struct vmm_vlink *vlink;
	vmm_paddr_t paddr = vmm_get_vlink_db();

	if (!name)
		return 0;

	vlink = (struct vmm_vlink *) vmm_ptov(paddr);

	if (plnk) {
		while (paddr && paddr != plnk) {
			paddr = vlink->next;
			vlink = (struct vmm_vlink *) vmm_ptov(paddr);
			if (!vlink)
				return 0;
		}

		paddr = vlink->next;
		vlink = (struct vmm_vlink *) vmm_ptov(paddr);
	}

	while (paddr && vlink) {
		if (vlink->name &&
		strcmp(vmm_ptov((vmm_paddr_t) vlink->name), name) == 0)
			return paddr;
		paddr = vlink->next;
		vlink = (struct vmm_vlink *) vmm_ptov(paddr);
	}

	/* not found */
	return 0;
}
