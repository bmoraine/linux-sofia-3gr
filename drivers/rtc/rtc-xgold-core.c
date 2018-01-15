#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/device.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <sofia/vmm_platform_service.h>
#include "rtc-xgold.h"

#define XGOLD_RTC_ENTER		pr_debug("rtc: %s\n", __func__)
#define rtc_dbg(fmt, arg...)	pr_debug("rtc: "fmt, ##arg);
#define rtc_info(fmt, arg...)	pr_info("rtc: "fmt, ##arg);
#define rtc_err(fmt, arg...)	pr_err("rtc: ERROR"fmt, ##arg);

#define DELTA_Y 1900
#define DELTA_M 1

static irqreturn_t xgold_rtc_irq(int irq, void *rtc_d)
{
	struct rtc_driver_data *rtcd = rtc_d;
	unsigned long events = RTC_AF;

	spin_lock(&rtcd->lock);
	XGOLD_RTC_ENTER;
	rtc_update_irq(rtcd->rtc, 1, events);
	spin_unlock(&rtcd->lock);
	return IRQ_HANDLED;
}

static int xgold_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);
	unsigned long long time_us = 0;
	unsigned long time_s = 0;

	spin_lock(&rtcd->lock);
	vmm_rtc_get_time_us(&time_us);
	do_div(time_us, 1000000);
	time_s = time_us;
	rtc_time_to_tm(time_s, tm);
	rtc_dbg("%s: rtc time %4d-%02d-%02d %02d:%02d:%02d (%lds)\n", __func__,
			tm->tm_year, tm->tm_mon, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec
			, time_s);
	spin_unlock(&rtcd->lock);

	return rtc_valid_tm(tm);
}

#if 0
static int xgold_rtc_read_date(struct device *dev, struct rtc_time *tm)
{
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);
	unsigned long time_s = 0;
	struct rtc_datetime_shared_data rtc_data;

	spin_lock(&rtcd->lock);
	vmm_rtc_get_datetime(&rtc_data);
	rtc_dbg("%s: hum time %4d-%2d-%2d %2d:%2d:%2d\n", __func__,
			rtc_data.m_year, rtc_data.m_month, rtc_data.m_day,
			rtc_data.m_hour, rtc_data.m_minute, rtc_data.m_second);
	tm->tm_year = rtc_data.m_year - DELTA_Y;
	tm->tm_mon = rtc_data.m_month - DELTA_M;
	tm->tm_mday = rtc_data.m_day;
	tm->tm_hour = rtc_data.m_hour;
	tm->tm_min = rtc_data.m_minute;
	tm->tm_sec = rtc_data.m_second;
	rtc_tm_to_time(tm, &time_s);
	rtc_dbg("%s: rtc time %4d-%2d-%2d %2d:%2d:%2d (%lds)\n", __func__,
			tm->tm_year, tm->tm_mon, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec,
			time_s);
	spin_unlock(&rtcd->lock);

	return rtc_valid_tm(tm);
}
#endif

static int xgold_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long time;
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);

	if (rtc_tm_to_time(tm, &time)) {
		dev_info(dev, " RTC: conversion from time to seconds failed");
		return -EINVAL;
	}

	spin_lock(&rtcd->lock);
	rtc_dbg("%s: rtc time %4d-%02d-%02d %02d:%02d:%02d (%lds)\n", __func__,
			tm->tm_year, tm->tm_mon, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec,
			time);

	if (!rtc_valid_tm(tm)) {
		struct rtc_datetime_shared_data rtc_data;

		rtc_data.m_year = tm->tm_year + DELTA_Y;
		rtc_data.m_month = tm->tm_mon + DELTA_M;
		rtc_data.m_day = tm->tm_mday;
		rtc_data.m_hour = tm->tm_hour;
		rtc_data.m_minute = tm->tm_min;
		rtc_data.m_second = tm->tm_sec;
		rtc_data.m_msecond = 0;
		rtc_dbg("%s: hum time %4d-%02d-%02d %02d:%02d:%02d\n", __func__,
			rtc_data.m_year, rtc_data.m_month, rtc_data.m_day,
			rtc_data.m_hour, rtc_data.m_minute, rtc_data.m_second);
		vmm_rtc_set_datetime(&rtc_data);
	} else {
		pr_err("%s: Invalid RTC value !\n", __func__);
	}
	spin_unlock(&rtcd->lock);

	return 0;
}

static int xgold_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);
	struct rtc_time *tm;
	struct rtc_datetime_shared_data rtc_data;
	unsigned long time;

	spin_lock(&rtcd->lock);
	tm = &alm->time;
	vmm_rtc_get_alarm(&rtc_data);
	rtc_dbg("%s: hum time %4d-%02d-%02d %02d:%02d:%02d\n", __func__,
		rtc_data.m_year, rtc_data.m_month, rtc_data.m_day,
		rtc_data.m_hour, rtc_data.m_minute, rtc_data.m_second);

	tm->tm_year = rtc_data.m_year - DELTA_Y;
	tm->tm_mon = rtc_data.m_month - DELTA_M;
	tm->tm_mday = rtc_data.m_day;
	tm->tm_hour = rtc_data.m_hour;
	tm->tm_min = rtc_data.m_minute;
	tm->tm_sec = rtc_data.m_second;
	if (rtc_tm_to_time(tm, &time)) {
		dev_info(dev, " RTC: conversion from time to seconds failed");
		return -EINVAL;
	}
	rtc_dbg("%s: rtc time %4d-%02d-%02d %02d:%02d:%02d (%lds)\n", __func__,
			tm->tm_year, tm->tm_mon, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec,
			time);

	spin_unlock(&rtcd->lock);

	if (!rtc_valid_tm(tm))
		return 0;
	else
		return -EINVAL;
}

static int xgold_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{

	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);
	struct rtc_time *tm;
	unsigned long time;
	struct rtc_datetime_shared_data rtc_data2;

	if (rtc_tm_to_time(&alm->time, &time)) {
		dev_info(dev, "conversion failed");
		return -EINVAL;
	}

	spin_lock(&rtcd->lock);
	tm = &alm->time;
	rtc_dbg("%s: rtc time %4d-%02d-%02d %02d:%02d:%02d (%lds)\n", __func__,
			tm->tm_year, tm->tm_mon, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec,
			time);

	vmm_rtc_get_alarm(&rtc_data2);
	if (rtc_data2.m_year != 0) {
		vmm_rtc_clear_alarm();
		rtc_dbg("%s: clear alarm\n", __func__);
	}


	if (!rtc_valid_tm(tm)) {
		struct rtc_datetime_shared_data rtc_data;

		rtc_data.m_year = tm->tm_year + DELTA_Y;
		rtc_data.m_month = tm->tm_mon + DELTA_M;
		rtc_data.m_day = tm->tm_mday;
		rtc_data.m_hour = tm->tm_hour;
		rtc_data.m_minute = tm->tm_min;
		rtc_data.m_second = tm->tm_sec;
		rtc_data.m_msecond = 0;
		rtc_info("%s: hum time %4d-%02d-%02d %02d:%02d:%02d\n",
			__func__,
			rtc_data.m_year, rtc_data.m_month, rtc_data.m_day,
			rtc_data.m_hour, rtc_data.m_minute, rtc_data.m_second);
		vmm_rtc_set_alarm(&rtc_data);
	}
	spin_unlock(&rtcd->lock);

	return 0;
}

static int xgold_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	XGOLD_RTC_ENTER;
	/* NOTHING TO DO */

	return 0;
}

static struct rtc_class_ops xgold_rtc_class_ops = {
	.read_time = xgold_rtc_read_time,
/*	.read_time = xgold_rtc_read_date, */
	.set_time = xgold_rtc_set_time,
	.read_alarm = xgold_rtc_read_alarm,
	.set_alarm = xgold_rtc_set_alarm,
	.alarm_irq_enable = xgold_rtc_alarm_irq_enable,
};
static struct class_interface alarmtimer_rtc_interface = {
	.add_dev = NULL,
};
static int xgold_rtc_core_probe(struct device *dev)
{
	unsigned ret = 0;
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);

	XGOLD_RTC_ENTER;

	spin_lock_init(&rtcd->lock);
	if (!device_can_wakeup(dev))
		ret = device_init_wakeup(dev, 1);

	rtcd->rtc = devm_rtc_device_register(dev, dev_name(dev),
					&xgold_rtc_class_ops, THIS_MODULE);
	if (IS_ERR(rtcd->rtc)) {
		dev_dbg(dev, "%s: can't register RTC device, err %ld\n",
			dev_name(dev), PTR_ERR(rtcd->rtc));
		goto fail0;
	}

	alarmtimer_rtc_interface.class = rtc_class;
	class_interface_register(&alarmtimer_rtc_interface);

	/* register rtc irq  by one handle(the same handle) */
	ret = irq_of_parse_and_map(dev->of_node, 0);
	if (!ret) {
		dev_err(dev, "cannot map irq index %d", 0);
	} else {
		dev_info(dev, "map irq index %d", ret);
		rtcd->irq = ret;
		ret = request_irq(rtcd->irq, xgold_rtc_irq,
				IRQF_NO_SUSPEND|IRQF_SHARED,
				dev_name(dev), rtcd);
		if (ret) {
			dev_err(dev, "failure in requesting rtc IRQ\n");
			goto fail0;
		}
	}

	dev_set_drvdata(&rtcd->rtc->dev, rtcd);
	rtc_info("RTC virtualized driver probed\n");
	return ret;

fail0:
	return -EIO;

}

static int xgold_rtc_core_remove(struct device *dev)
{
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);

	XGOLD_RTC_ENTER;

	device_init_wakeup(dev, 0);

	/* leave rtc running, but disable irqs */
	free_irq(rtcd->irq, rtcd);
	rtc_device_unregister(rtcd->rtc);
	return 0;
}

static void xgold_rtc_core_shutdown(struct device *dev)
{
	XGOLD_RTC_ENTER;
}

static struct xgold_rtc_ops xgold_rtc_core_ops = {
	.probe = xgold_rtc_core_probe,
	.shutdown = xgold_rtc_core_shutdown,
	.remove = xgold_rtc_core_remove,
};

struct rtc_driver_data *xgold_rtc_init_driver(struct device *dev)
{
	struct rtc_driver_data *rtcd;

	rtcd = kzalloc(sizeof(struct rtc_driver_data), GFP_KERNEL);
	if (rtcd == NULL)
		return ERR_PTR(-ENOMEM);

	rtcd->core_ops = &xgold_rtc_core_ops;
	dev_set_drvdata(dev, rtcd);
	return rtcd;
}

MODULE_LICENSE("GPL");
