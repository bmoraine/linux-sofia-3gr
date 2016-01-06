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

#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/smp.h>
#if defined(CONFIG_SYSTEM_PROFILING)
#include <linux/sysprofile.h>
#endif
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <sofia/mv_hypercalls.h>
#include <sofia/mv_gal.h>
#include <asm/pat.h>

/* define VM_MULTIPLE_VCPUS if guest VM has more than 1 VCPU */
#ifdef CONFIG_SMP
#define VM_MULTIPLE_VCPUS 1
#endif

struct vmm_shared_data *vmm_shared_data[CONFIG_MAX_VCPUS_PER_VM];
const char *vm_command_line;
struct irq_domain *hirq_domain;

#if 0
void irq_force_complete_move(int irq)
{
}
#endif

struct vmm_shared_data *mv_gal_get_shared_data(void)
{
#ifdef VM_MULTIPLE_VCPUS
	unsigned cpu = get_cpu();
	put_cpu();
	return vmm_shared_data[cpu];
#else
	return vmm_shared_data[0];
#endif
}

struct vmm_shared_data *mv_gal_get_system_shared_data(void)
{
	return vmm_shared_data[0];
}

struct hirq_handler_wrapper {
	int irq;
	irq_handler_t handler;
	void *cookie;
};

irqreturn_t generic_hirq_handler(int irq, void *cookie)
{
	struct hirq_handler_wrapper *hirq_wrapper =
		(struct hirq_handler_wrapper *)cookie;
	return hirq_wrapper->handler(irq, hirq_wrapper->cookie);
}

void mv_gal_panic(char *panic_msg)
{
	if (panic_msg)
		mv_gal_printk("PANIC: %s\n", panic_msg);
	else
		mv_gal_printk("PANIC\n");

	while (1)
		;
}

void *pmem_vbase;

void *mv_gal_ptov(vmm_paddr_t paddr)
{
	if (pmem_vbase) {
		void *ptr;
		struct vmm_shared_data *p_shared_data =
			mv_gal_get_shared_data();
		if (paddr >= p_shared_data->pmem_paddr &&
		    paddr <= p_shared_data->pmem_paddr +
		    p_shared_data->pmem_size) {
			ptr = (void *)(pmem_vbase + (paddr -
					p_shared_data->pmem_paddr));
			return ptr;
		} else {

			return (void *)paddr; /* How would this ever work? */
		}
	} else
		return NULL;
}

/* This function is never used! */
vmm_paddr_t mv_gal_vtop(void *vaddr)
{
	if (pmem_vbase) {
		vmm_paddr_t ptr;
		struct vmm_shared_data *p_shared_data =
			mv_gal_get_shared_data();
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
}
static unsigned int myid;

void mv_gal_init(struct vmm_shared_data *data)
{
	myid = data->os_id;
	vm_command_line = data->vm_cmdline; /* vm_command_line is never used! */
}

static int sofia_vmm_map_vcpu_shmem(void)
{
	phys_addr_t ptr;

	if (mv_gal_get_shared_data())
		return 0;

	ptr = (uintptr_t)mv_vcpu_get_data();
	if (memblock_reserve((resource_size_t)ptr, (resource_size_t)(ptr +
		sizeof(struct vmm_shared_data)))) {
		pr_err("Unable to map VMM shared data\n");
		return -EINVAL;
	}

	vmm_shared_data[smp_processor_id()] = phys_to_virt(ptr);
	pr_debug("ptr=0x%pa vmm_shared_data=0x%p\n",
		&ptr, mv_gal_get_shared_data());

	return 0;
}

int sofia_vmm_init_secondary(void)
{
	pr_debug("In sofia_vmm_init_secondary\n");
	mv_virq_request(LOCAL_TIMER_VECTOR, 1);
	mv_virq_unmask(LOCAL_TIMER_VECTOR);
	mv_virq_request(RESCHEDULE_VECTOR, 1);
	mv_virq_unmask(RESCHEDULE_VECTOR);
	mv_virq_request(CALL_FUNCTION_VECTOR, 1);
	mv_virq_unmask(CALL_FUNCTION_VECTOR);
	mv_virq_request(CALL_FUNCTION_SINGLE_VECTOR, 1);
	mv_virq_unmask(CALL_FUNCTION_SINGLE_VECTOR);
	mv_virq_request(REBOOT_VECTOR, 1);
	mv_virq_unmask(REBOOT_VECTOR);

	sofia_vmm_map_vcpu_shmem();

	mv_virq_ready();

	return 0;
}

int __init sofia_vmm_init(void)
{
	pr_debug("In sofia_vmm_init\n");

	if (sofia_vmm_map_vcpu_shmem())
		panic("Unable to map vcpu shared mem\n");

	pmem_vbase =
	    ioremap_cache(mv_gal_get_shared_data()->pmem_paddr,
			  mv_gal_get_shared_data()->pmem_size);
	pr_debug(
		"pmem_paddr=0x%x pmem_size=0x%x pmem_vbase=0x%p\n",
		mv_gal_get_shared_data()->pmem_paddr,
		mv_gal_get_shared_data()->pmem_size,
		pmem_vbase);
	if (!pmem_vbase)
		panic("Unable to map PMEM\n");

	pr_debug("Calling vmm_al_init. os_id=%d\n",
		mv_gal_get_shared_data()->os_id);
	mv_gal_init(mv_gal_get_shared_data());

	mv_virq_ready();

	return 0;
}

inline unsigned int mv_gal_os_id(void)
{
	return myid;
}

#define HIRQ_HANDLER_NAME_SIZE	16
void *mv_gal_register_hirq_callback(uint32_t hirq, irq_handler_t cb,
		void *cookie)
{
	int virq = irq_create_mapping(hirq_domain, hirq - VMM_HIRQ_START);
	char *handler_name = NULL;
	struct hirq_handler_wrapper *wrapper = NULL;

	wrapper = kmalloc(sizeof(struct hirq_handler_wrapper), GFP_KERNEL);
	if (!wrapper) {
		pr_err("failed to allocate wrapper\n");
		return NULL;
	}
	wrapper->irq = virq;
	wrapper->handler = cb;
	wrapper->cookie = cookie;
	handler_name = kmalloc(HIRQ_HANDLER_NAME_SIZE, GFP_KERNEL);
	if (!handler_name) {
		pr_err("failed to allocate handler_name\n");
		goto free_wrapper;
	}
	snprintf(handler_name, HIRQ_HANDLER_NAME_SIZE, "hirq-%d", hirq);
	if (request_irq(virq, generic_hirq_handler,
				IRQF_SHARED | IRQF_NO_SUSPEND, handler_name,
				(void *)wrapper)) {
		pr_err("failed to request irq\n");
		goto free_handler;
	}
	return wrapper;
free_handler:
	kfree(handler_name);
free_wrapper:
	kfree(wrapper);
	return NULL;
}

void mv_gal_hirq_detach(void *id)
{
	struct hirq_handler_wrapper *hirq_wrapper =
		(struct hirq_handler_wrapper *)id;
	free_irq(hirq_wrapper->irq, id);
	kfree(id);
}


static int32_t mv_gal_probe(struct platform_device *pdev)
{
	struct device_node *np;

	np = pdev->dev.of_node;
	hirq_domain = irq_find_host(of_irq_find_parent(np));

	mv_ipc_init();
	return 0;
}

static const struct of_device_id mv_gal_of_match[] = {
	{
		.compatible = "intel,mobilevisor",
	},
	{},
};

MODULE_DEVICE_TABLE(of, mv_gal_of_match);

static struct platform_driver mv_gal_driver = {
	.probe = mv_gal_probe,
	.driver = {
		.name = "mv_gal",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mv_gal_of_match),
	}
};

static int32_t __init mv_gal_driver_init(void)
{
	return platform_driver_register(&mv_gal_driver);
}

static void __exit mv_gal_driver_exit(void)
{
	 platform_driver_unregister(&mv_gal_driver);
}

core_initcall(mv_gal_driver_init);
module_exit(mv_gal_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mobilevisor guest adaption driver");
