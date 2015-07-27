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
 *
 * Architecture specific OF callbacks.
 *
 * based on original arch/x86/kernel/devicetree.c
 */

#include <linux/bootmem.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/of_pci.h>
#include <linux/initrd.h>
#include <linux/irqchip.h>

#include <asm/hpet.h>
#include <asm/apic.h>
#include <asm/pci_x86.h>
#include <asm/setup.h>

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/cpu.h>
#endif
__initdata u64 initial_dtb;
char __initdata cmd_line[COMMAND_LINE_SIZE];

int __initdata of_ioapic;

void __init early_init_dt_scan_chosen_arch(unsigned long node)
{
	BUG();
}

void * __init early_init_dt_alloc_memory_arch(u64 size, u64 align)
{
	return __alloc_bootmem(size, align, __pa(MAX_DMA_ADDRESS));
}

void __init add_dtb(u64 data)
{
	initial_dtb = data + offsetof(struct setup_data, data);
}

/*
 * CE4100 ids. Will be moved to machine_device_initcall() once we have it.
 */
static struct of_device_id __initdata ce4100_ids[] = {
	{ .compatible = "intel,ce4100-cp", },
	{ .compatible = "isa", },
	{ .compatible = "pci", },
	{},
};

static int __init add_bus_probe(void)
{
	if (!of_have_populated_dt())
		return 0;

	return of_platform_bus_probe(NULL, ce4100_ids, NULL);
}
module_init(add_bus_probe);

#ifdef CONFIG_PCI
struct device_node *pcibios_get_phb_of_node(struct pci_bus *bus)
{
	struct device_node *np;

	for_each_node_by_type(np, "pci") {
		const void *prop;
		unsigned int bus_min;

		prop = of_get_property(np, "bus-range", NULL);
		if (!prop)
			continue;
		bus_min = be32_to_cpup(prop);
		if (bus->number == bus_min)
			return np;
	}
	return NULL;
}

static int x86_of_pci_irq_enable(struct pci_dev *dev)
{
	u32 virq;
	int ret;
	u8 pin;

	ret = pci_read_config_byte(dev, PCI_INTERRUPT_PIN, &pin);
	if (ret)
		return ret;
	if (!pin)
		return 0;

	virq = of_irq_parse_and_map_pci(dev, 0, 0);
	if (virq == 0)
		return -EINVAL;
	dev->irq = virq;
	return 0;
}

static void x86_of_pci_irq_disable(struct pci_dev *dev)
{
}

void x86_of_pci_init(void)
{
	pcibios_enable_irq = x86_of_pci_irq_enable;
	pcibios_disable_irq = x86_of_pci_irq_disable;
}
#endif

#ifdef CONFIG_X86_LOCAL_APIC
static void __init dtb_setup_cpus(void)
{
#ifdef CONFIG_X86_INTEL_SOFIA
	unsigned nr_vcpus, vcpu;
#else
	struct device_node *cpu, *cpus;
#endif
	unsigned apic_id;

	smp_found_config = 0;
	num_processors = 0;
#ifdef CONFIG_X86_INTEL_SOFIA
	if (sofia_cpu_mapping_init())
		panic("Unable to retrieve vcpu mapping");

	nr_vcpus = sofia_get_nr_vcpus();
	if (nr_vcpus == 0)
		panic("Number of vcpus can't be null");

	for (vcpu = 0; vcpu < nr_vcpus; vcpu++) {
		apic_id = sofia_cpu_get_apicid(vcpu);
		generic_processor_info(apic_id,
			       GET_APIC_VERSION(apic_read(APIC_LVR)));
	}

#else
	cpus = of_find_node_by_path("/cpus");

	if (!cpus) {
		generic_processor_info(boot_cpu_physical_apicid,
			       GET_APIC_VERSION(apic_read(APIC_LVR)));
		return;
	}

	for_each_child_of_node(cpus, cpu) {
		if (of_node_cmp(cpu->type, "cpu"))
			continue;

		if (of_property_read_u32(cpu, "reg", &apic_id)) {
			pr_debug(" * %s missing reg property\n",
				     cpu->full_name);
			return;
		}

		generic_processor_info(apic_id,
			       GET_APIC_VERSION(apic_read(APIC_LVR)));

	}
#endif
	if (num_processors > 1)
		smp_found_config = 1;
}
#endif

static void __init dtb_setup_hpet(void)
{
#ifdef CONFIG_HPET_TIMER
	struct device_node *dn;
	struct resource r;
	int ret;

	dn = of_find_compatible_node(NULL, NULL, "intel,ce4100-hpet");
	if (!dn)
		return;
	ret = of_address_to_resource(dn, 0, &r);
	if (ret) {
		WARN_ON(1);
		return;
	}
	hpet_address = r.start;
#endif
}

static void __init dtb_lapic_setup(void)
{
#ifdef CONFIG_X86_LOCAL_APIC
	struct device_node *dn;
	struct resource r;
	int ret;

	dn = of_find_compatible_node(NULL, NULL, "intel,ce4100-lapic");
	if (!dn)
		return;

	ret = of_address_to_resource(dn, 0, &r);
	if (WARN_ON(ret))
		return;

	/* Did the boot loader setup the local APIC ? */
#ifndef CONFIG_X86_64
	if (!cpu_has_apic) {
		if (apic_force_enable(r.start))
			return;
	}
#endif
	smp_found_config = 1;
	if (of_property_read_bool(dn, "no_pic_mode"))
		pic_mode = 0;
	else
		pic_mode = 1;

	if (of_property_read_bool(dn, "no_apic_setup"))
		noapicsetup = 1;

	register_lapic_address(r.start);
	dtb_setup_cpus();
#endif
}

#ifdef CONFIG_X86_IO_APIC
static unsigned int ioapic_id;

static void __init dtb_add_ioapic(struct device_node *dn)
{
	struct resource r;
	int ret;

	ret = of_address_to_resource(dn, 0, &r);
	if (ret) {
		printk(KERN_ERR "Can't obtain address from node %s.\n",
				dn->full_name);
		return;
	}
	mp_register_ioapic(++ioapic_id, r.start, gsi_top);
	ret = of_address_to_resource(dn, 1, &r);
	if (!ret)
		set_fixmap_nocache(FIX_IO_APIC_REG_0 + nr_ioapics - 1, r.start);
}

static void __init dtb_ioapic_setup(void)
{
	struct device_node *dn;

	for_each_compatible_node(dn, NULL, "intel,ce4100-ioapic")
		dtb_add_ioapic(dn);

	if (nr_ioapics) {
		of_ioapic = 1;
		return;
	}
	printk(KERN_ERR "Error: No information about IO-APIC in OF.\n");
}
#else
static void __init dtb_ioapic_setup(void) {}
#endif

static void __init dtb_apic_setup(void)
{
	dtb_lapic_setup();
	dtb_ioapic_setup();
}

#ifdef CONFIG_OF_FLATTREE
static struct boot_param_header *fdt_header;
static int fdt_header_size;

void * __init x86_fdt_header(void)
{
	u32 size, map_len;
	struct boot_param_header *dt;

	if (!initial_dtb) {
		printk(KERN_ERR "Error: initial_dtb not initialized.\n");
		return NULL;
	}

	map_len = max(PAGE_SIZE - (initial_dtb & ~PAGE_MASK),
			(u64)sizeof(struct boot_param_header));

	dt = early_memremap(initial_dtb, map_len);
	if (!dt) {
		pr_err("Error: early_memremap of devictree failed.\n");
		return NULL;
	}
	size = be32_to_cpu(dt->totalsize);
	if (map_len < size) {
		early_iounmap(dt, map_len);
		dt = early_memremap(initial_dtb, size);
		map_len = size;
	}

	fdt_header = dt;
	fdt_header_size = map_len;
	return (void *)dt;
}

static void __init x86_fdt_free_header(void)
{
	if (!fdt_header) {
		pr_err("%s, header NULL\n", __func__);
		return;
	}
	early_iounmap(fdt_header, fdt_header_size);
}

static void __init x86_flattree_get_config(void)
{
	u32 size, map_len;
	struct boot_param_header *dt;

	if (!initial_dtb)
		return;

	map_len = max(PAGE_SIZE - (initial_dtb & ~PAGE_MASK),
			(u64)sizeof(struct boot_param_header));

	dt = early_memremap(initial_dtb, map_len);
	if (!dt) {
		pr_err("Error: early_memremap of devicetree failed.\n");
		return;
	}
	size = be32_to_cpu(dt->totalsize);
	if (map_len < size) {
		early_iounmap(dt, map_len);
		dt = early_memremap(initial_dtb, size);
		map_len = size;
	}

	initial_boot_params = dt;
	unflatten_and_copy_device_tree();
	early_iounmap(dt, map_len);
	x86_fdt_free_header();
}
#else
static inline void x86_flattree_get_config(void) { }
#endif

void __init x86_dtb_init(void)
{
	x86_flattree_get_config();

	if (!of_have_populated_dt())
		return;

	dtb_setup_hpet();
	dtb_apic_setup();
}

#ifdef CONFIG_X86_IO_APIC

struct of_ioapic_type {
	u32 out_type;
	u32 trigger;
	u32 polarity;
};

static struct of_ioapic_type of_ioapic_type[] =
{
	{
		.out_type	= IRQ_TYPE_EDGE_RISING,
		.trigger	= IOAPIC_EDGE,
		.polarity	= 1,
	},
	{
		.out_type	= IRQ_TYPE_LEVEL_LOW,
		.trigger	= IOAPIC_LEVEL,
		.polarity	= 0,
	},
	{
		.out_type	= IRQ_TYPE_LEVEL_HIGH,
		.trigger	= IOAPIC_LEVEL,
		.polarity	= 1,
	},
	{
		.out_type	= IRQ_TYPE_EDGE_FALLING,
		.trigger	= IOAPIC_EDGE,
		.polarity	= 0,
	},
};

static int ioapic_xlate(struct irq_domain *domain,
			struct device_node *controller,
			const u32 *intspec, u32 intsize,
			irq_hw_number_t *out_hwirq, u32 *out_type)
{
	struct io_apic_irq_attr attr;
	struct of_ioapic_type *it;
	u32 line, idx;
	int rc;

	if (WARN_ON(intsize < 2))
		return -EINVAL;

	line = intspec[0];

	if (intspec[1] >= ARRAY_SIZE(of_ioapic_type))
		return -EINVAL;

	it = &of_ioapic_type[intspec[1]];

	idx = (u32) domain->host_data;
	set_io_apic_irq_attr(&attr, idx, line, it->trigger, it->polarity);

	rc = io_apic_setup_irq_pin_once(irq_find_mapping(domain, line),
					cpu_to_node(0), &attr);
	if (rc)
		return rc;

	*out_hwirq = line;
	*out_type = it->out_type;
	return 0;
}

const struct irq_domain_ops ioapic_irq_domain_ops = {
	.xlate = ioapic_xlate,
};

static void dt_add_ioapic_domain(unsigned int ioapic_num,
		struct device_node *np)
{
	struct irq_domain *id;
	struct mp_ioapic_gsi *gsi_cfg;
	int ret;
	int num;

	gsi_cfg = mp_ioapic_gsi_routing(ioapic_num);
	num = gsi_cfg->gsi_end - gsi_cfg->gsi_base + 1;

	id = irq_domain_add_linear(np, num, &ioapic_irq_domain_ops,
			(void *)ioapic_num);
	BUG_ON(!id);
	if (gsi_cfg->gsi_base == 0) {
		/*
		 * The first NR_IRQS_LEGACY irq descs are allocated in
		 * early_irq_init() and need just a mapping. The
		 * remaining irqs need both. All of them are preallocated
		 * and assigned so we can keep the 1:1 mapping which the ioapic
		 * is having.
		 */
		irq_domain_associate_many(id, 0, 0, NR_IRQS_LEGACY);

		if (num > NR_IRQS_LEGACY) {
			ret = irq_create_strict_mappings(id, NR_IRQS_LEGACY,
					NR_IRQS_LEGACY, num - NR_IRQS_LEGACY);
			if (ret)
				pr_err("Error creating mapping for the "
						"remaining IRQs: %d\n", ret);
		}
		irq_set_default_host(id);
	} else {
		ret = irq_create_strict_mappings(id, gsi_cfg->gsi_base, 0, num);
		if (ret)
			pr_err("Error creating IRQ mapping: %d\n", ret);
	}
}

static void __init ioapic_add_ofnode(struct device_node *np)
{
	struct resource r;
	int i, ret;

	ret = of_address_to_resource(np, 0, &r);
	if (ret) {
		printk(KERN_ERR "Failed to obtain address for %s\n",
				np->full_name);
		return;
	}

	for (i = 0; i < nr_ioapics; i++) {
		if (r.start == mpc_ioapic_addr(i)) {
			dt_add_ioapic_domain(i, np);
			return;
		}
	}
	printk(KERN_ERR "IOxAPIC at %s is not registered.\n", np->full_name);
}

void __init x86_add_irq_domains(void)
{
	struct device_node *dp;

	if (!of_have_populated_dt())
		return;

	for_each_node_with_property(dp, "interrupt-controller") {
		if (of_device_is_compatible(dp, "intel,ce4100-ioapic"))
			ioapic_add_ofnode(dp);
	}

	if (IS_ENABLED(CONFIG_OF))
		irqchip_init();
}
/*
 * empty function at time time
 * The io-apic configuration is already done with ioapic_add_ofnode
 * To be filled if we decide to not use the "intel,ce4100-ioapic"
 * compatibility anymore
 */
int __init ioapic_of_init(struct device_node *node, struct device_node *parent)
{
	return 0;
}

static const struct of_device_id irqchip_of_match_sofia_native_ioapic
	__used __section(__irqchip_of_table) = {
	.compatible = "intel,ce4100-ioapic",
	.data = ioapic_of_init
};

#else
void __init x86_add_irq_domains(void)
{
	if (IS_ENABLED(CONFIG_OF))
		irqchip_init();
}
#endif
