/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef NOC_H_
#define NOC_H_

#define XGOLD_NOC_ERROR_REGISTERS 5
#define XGOLDREG_ADDR(_reg)	((void __iomem *) (reg->base + reg->offset))
#define MAX_ITERATION		(1<<28)	/* Arbitrary value */
#define DURATION		24

struct xgold_bitfield {
	struct xgold_register *parent;
	const char *name;
	const char *description;
	unsigned offset;
	unsigned length;
	unsigned mask;
	const char **lut;
	u16 aperture_size;
	u16 *aperture_idx;
	unsigned *aperture_base;
	struct list_head link;
	spinlock_t lock;
};

struct xgold_register {
	struct device *parent;
	void __iomem *base;
	const char *name;
	const char *description;
	unsigned offset;
	unsigned length;
	u8 aperture_link;
	struct list_head bitfields;
	struct list_head link;
	spinlock_t lock;
	spinlock_t hw_lock;
};

struct xgold_noc_probe;

/* One measure for the NOC sniffer */
struct xgold_noc_stat_measure {
	u16 idx_trace_port_sel;
	u8 idx_init_flow;
	u8 idx_target_flow;
	char *init_flow;
	char *target_flow;
	unsigned min;
	unsigned max;
	u64 mean;
	unsigned now;
	unsigned iteration;
	struct list_head link;
	spinlock_t lock;
};

/* A set of statistics assigned to a probe
 * (as a probe can measure severals paths)
 */
struct xgold_noc_stat_probe {
	struct xgold_noc_probe *probe;
	struct list_head measure;
	struct list_head link;
	spinlock_t lock;
};

/* The main structure for the NOC sniffer */
struct xgold_noc_stat {
	struct list_head probe;
	struct dentry *dir;
	spinlock_t lock;
	bool run;
	unsigned clock_rate;
};

struct xgold_noc_filter {
	unsigned id;
	struct xgold_noc_probe *parent;
	struct xgold_register *route_id;
	struct xgold_bitfield *route_mask;
	struct xgold_bitfield *addr_base;
	struct xgold_bitfield *window_size;
	struct xgold_register *security_base;
	struct xgold_bitfield *security_mask;
	struct xgold_register *op_code;
	struct xgold_register *status;
	struct xgold_bitfield *length;
	struct xgold_bitfield *urgency;
};

struct xgold_noc_counter {
	unsigned id;
	struct xgold_noc_probe *parent;
	struct xgold_bitfield *portsel;
	struct xgold_bitfield *alarm_mode;
	struct xgold_bitfield *source_event;
	struct xgold_bitfield *value;
};

struct xgold_noc_probe {
	unsigned id;
	const char **available_portsel;
	struct xgold_register *main_ctl;
	struct xgold_register *cfg_ctl;
	struct xgold_register *trace_port_sel;
	struct xgold_register *filter_lut;
	struct xgold_register *stat_period;
	struct xgold_register *stat_alarm_max;
	struct xgold_register *stat_alarm_min;
	struct xgold_noc_device *parent;
	struct xgold_noc_counter *counters;
	struct xgold_noc_filter *filters;
	struct tasklet_struct tasklet;
};

struct dev_qos_cfg {
	struct list_head list;
	const char *name;
	struct regcfg *config;
	int noc_owner;
};

struct regcfg {
	struct list_head list;
	unsigned offset;
	unsigned value;
};

struct xgold_noc_device {
	struct device *dev;
	void __iomem *hw_base;
	unsigned probe_offset;
	struct clk *clock;
	int err_irq;
	int stat_irq;
	struct xgold_noc_probe *probes;
	struct list_head err_queue;
	spinlock_t lock;
	struct list_head registers;
	struct xgold_register *error_registers[XGOLD_NOC_ERROR_REGISTERS];
	struct dentry *dir;
	unsigned nr_probes;	/* How many probes */
	unsigned nr_filters;	/* How many filters per probe */
	unsigned nr_counters;	/* How many counters per probe */
	struct xgold_noc_stat *stat;
	struct dev_qos_cfg *qos;
	bool trap_on_error;
};

struct xgold_noc_error {
	struct list_head link;
	u64 timestamp;
	unsigned err[XGOLD_NOC_ERROR_REGISTERS];
};

int xgold_noc_debug_init(struct device *);
int xgold_noc_stat_init(struct device *);
void xgold_noc_stat_measure(struct xgold_noc_probe *);
int xgold_noc_stat_debugfs_init(struct device *);
int xgold_noc_stat_configure_measure(struct xgold_noc_device *,
				     struct xgold_noc_stat_probe *);

#endif /* NOC_H_ */
