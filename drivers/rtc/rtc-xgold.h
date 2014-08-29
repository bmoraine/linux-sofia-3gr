#ifndef __RTC_XGOLD_H
#define __RTC_XGOLD_H

#define OF_SCU_RTCIF "intel,scu_rtcif"
#define OF_SCU_PHYS "intel,scu-phys"
#define OF_PM_IRQ_NR "intel,pm_irq_nr"
struct xgold_rtc_ops {
	int (*probe)(struct device *);
	int (*remove)(struct device *);
	void (*shutdown)(struct device *);
};

struct rtc_driver_data {
	int irq;
	int  pm_irq;
	unsigned char  __iomem *rtc_base;
	void  __iomem *scu_base;
	unsigned int   scu_rtcif_offset;
	spinlock_t lock;
	struct rtc_device *rtc;
	struct xgold_rtc_ops *core_ops;
};

extern struct rtc_driver_data *xgold_rtc_init_driver(struct device *);
#endif /* __RTC_XGOLD_H */
