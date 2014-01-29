#ifndef _APIC_IRQ_H
#define _APIC_IRQ_H

/* Int types definitions for IOAPIC domain */
/* As VMM is taking care for us, this parameter does not mater for VMM+Linux */
#define IRQ_TYPE_DEFAULT IRQ_TYPE_LEVEL_LOW
#define IRQ_TYPE_EDGE_RISING 0
#define IRQ_TYPE_LEVEL_LOW 1
#define IRQ_TYPE_LEVEL_HIGH 2
#define IRQ_TYPE_EDGE_FALLING 3
#define XGOLD_IRQ_TYPE_LEVEL_LOW 8

#endif /* _APIC_IRQ_H */
