#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
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
#include <sofia/mv_gal.h>
#include <sofia/pal_shared_data.h>

#include <sofia/mv_svc_hypercalls.h>
#include "rtc-xgold.h"

#define XGOLD_RTC_ENTER		pr_debug("rtc: %s\n", __func__)
#define rtc_dbg(fmt, arg...)	pr_debug("rtc: "fmt, ##arg);
#define rtc_info(fmt, arg...)	pr_info("rtc: "fmt, ##arg);
#define rtc_err(fmt, arg...)	pr_err("rtc: ERROR: "fmt, ##arg);

#define DELTA_Y 1900
#define DELTA_M 1


static struct rtc_datetime_shared_data *vmm_get_rtc_shared_data(void)
{
	struct vmm_shared_data *vmm_shared_data;
	struct pal_shared_data *pal_shared_data;
	vmm_shared_data = mv_gal_get_shared_data();
	pal_shared_data = (struct pal_shared_data *)
			(&vmm_shared_data->pal_shared_mem_data);
	return &(pal_shared_data->rtc_shared_data);
}

static irqreturn_t xgold_rtc_completion_irq(int irq, void *rtc_d)
{
	struct rtc_driver_data *rtcd = rtc_d;

	XGOLD_RTC_ENTER;
	if (rtcd->async_mode) {
		rtcd->service_completed = 1;
		switch (rtcd->opcode) {
		case RTC_GET_DATETIME:
		case RTC_SET_DATETIME:
		case RTC_GET_DATETIME_US:
		case RTC_GET_ALARM:
		case RTC_SET_ALARM:
		case RTC_CLEAR_ALARM:
		case RTC_SET_DATETIME_US:
			rtc_dbg("service=%d\n", rtcd->opcode);
			break;
		default:
			rtc_dbg("interrupt for an unknown service (%d)\n",
					rtcd->opcode);
			break;
		}
		wake_up(&rtcd->waitqueue);
	} else
		rtc_err("Must not reach this point if in sync mode\n");
	return IRQ_HANDLED;
}

static irqreturn_t xgold_rtc_irq(int irq, void *rtc_d)
{
	struct rtc_driver_data *rtcd = rtc_d;
	unsigned long events = RTC_AF;

	XGOLD_RTC_ENTER;
	spin_lock(&rtcd->lock);
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
	mv_svc_rtc_get_time_us(&time_us);
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

static int xgold_rtc_read_time_async(struct device *dev, struct rtc_time *tm)
{
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);
	unsigned long time_s = 0;
	struct rtc_datetime_shared_data *rtc_data = vmm_get_rtc_shared_data();
	int result = 0, ret_wait = 0;

	if (!rtc_data) {
		rtc_err("rtc shared mem unavailable\n");
		return  -EINVAL;
	}
	mutex_lock(&rtcd->rtc_mutex);
	rtcd->opcode = RTC_GET_DATETIME;
	rtcd->service_completed = 0;
	mv_svc_rtc_get_datetime_async();
	ret_wait = wait_event_interruptible_timeout(rtcd->waitqueue,
			rtcd->service_completed, msecs_to_jiffies(2000));
	if (ret_wait == 0) { /* timeout */
		rtc_err("incomplete rtc services after 2 seconds\n");
		result = -ETIME;
		goto read_time_end;
	} else if (ret_wait == -ERESTARTSYS) { /* interrupted by signal */
		rtc_err("wait_event interrupted by signal\n");
		result = -ERESTARTSYS;
		goto read_time_end;
	}
	tm->tm_year = rtc_data->m_year - DELTA_Y;
	tm->tm_mon = rtc_data->m_month - DELTA_M;
	tm->tm_mday = rtc_data->m_day;
	tm->tm_hour = rtc_data->m_hour;
	tm->tm_min = rtc_data->m_minute;
	tm->tm_sec = rtc_data->m_second;
	if (rtc_tm_to_time(tm, &time_s)) {
		dev_info(dev, " RTC: conversion from time to seconds failed");
		result = -EINVAL;
		goto read_time_end;
	}

	rtc_dbg("rtc time %4d-%02d-%02d %02d:%02d:%02d (%lds)\n",
			tm->tm_year, tm->tm_mon, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec
			, time_s);
	result = rtc_valid_tm(tm);

read_time_end:
	mutex_unlock(&rtcd->rtc_mutex);
	return result;
}

#if 0
static int xgold_rtc_read_date(struct device *dev, struct rtc_time *tm)
{
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);
	unsigned long time_s = 0;
	struct rtc_datetime_shared_data rtc_data;

	spin_lock(&rtcd->lock);
	mv_svc_rtc_get_datetime(&rtc_data);
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
		mv_svc_rtc_set_datetime(&rtc_data);
	} else {
		pr_err("%s: Invalid RTC value !\n", __func__);
	}
	spin_unlock(&rtcd->lock);

	return 0;
}

static int xgold_rtc_set_time_async(struct device *dev, struct rtc_time *tm)
{
	unsigned long time;
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);
	int result = 0, ret_wait = 0;

	if (rtc_tm_to_time(tm, &time)) {
		dev_info(dev, " RTC: conversion from time to seconds failed");
		return -EINVAL;
	}
	mutex_lock(&rtcd->rtc_mutex);

	rtc_dbg("rtc time %4d-%02d-%02d %02d:%02d:%02d (%lds)\n",
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
		rtc_dbg("hum time %4d-%02d-%02d %02d:%02d:%02d\n",
			rtc_data.m_year, rtc_data.m_month, rtc_data.m_day,
			rtc_data.m_hour, rtc_data.m_minute, rtc_data.m_second);
		rtcd->service_completed = 0;
		rtcd->opcode = RTC_SET_DATETIME;
		mv_svc_rtc_set_datetime(&rtc_data);
		ret_wait = wait_event_interruptible_timeout(rtcd->waitqueue,
				rtcd->service_completed,
				msecs_to_jiffies(2000));
		if (ret_wait == 0) { /* timeout */
			rtc_err("incomplete rtc services after 2 seconds\n");
			result = -ETIME;
			goto set_time_end;
		} else if (ret_wait == -ERESTARTSYS) {
			/* interrupted by signal */
			rtc_err("wait_event interrupted by signal\n");
			result = -ERESTARTSYS;
			goto set_time_end;
		}
	} else {
		pr_err("%s: Invalid RTC value !\n", __func__);
	}
set_time_end:
	mutex_unlock(&rtcd->rtc_mutex);

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
	mv_svc_rtc_get_alarm(&rtc_data);
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

static int xgold_rtc_read_alarm_async(struct device *dev,
		struct rtc_wkalrm *alm)
{
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);
	struct rtc_time *tm;
	unsigned long time_s = 0;
	struct rtc_datetime_shared_data *rtc_data = vmm_get_rtc_shared_data();
	int result = 0, ret_wait = 0, read_res;

	if (!rtc_data) {
		rtc_err("rtc shared mem unavailable\n");
		return  -EINVAL;
	}
	mutex_lock(&rtcd->rtc_mutex);
	tm = &alm->time;
	rtcd->opcode = RTC_GET_ALARM;
	rtcd->service_completed = 0;
	read_res = mv_svc_rtc_get_alarm_async();
	if (read_res != 0) {
		tm->tm_year = 0;
		tm->tm_mon = 0;
		tm->tm_mday = 0;
		tm->tm_hour = 0;
		tm->tm_min = 0;
		tm->tm_sec = 0;
		rtc_dbg("No alarm set\n");
		goto read_alarm_end;
	}
	ret_wait = wait_event_interruptible_timeout(rtcd->waitqueue,
			rtcd->service_completed, msecs_to_jiffies(2000));
	if (ret_wait == 0) { /* timeout */
		rtc_err("incomplete rtc services after 2 seconds\n");
		result = -ETIME;
		goto read_alarm_end;
	} else if (ret_wait == -ERESTARTSYS) { /* interrupted by signal */
		rtc_err("wait_event interrupted by signal\n");
		result = -ERESTARTSYS;
		goto read_alarm_end;
	}

	tm->tm_year = rtc_data->m_year - DELTA_Y;
	tm->tm_mon = rtc_data->m_month - DELTA_M;
	tm->tm_mday = rtc_data->m_day;
	tm->tm_hour = rtc_data->m_hour;
	tm->tm_min = rtc_data->m_minute;
	tm->tm_sec = rtc_data->m_second;
	if (rtc_tm_to_time(tm, &time_s)) {
		dev_info(dev, "RTC: conversion from time to seconds failed");
		result = -EINVAL;
		goto read_alarm_end;
	}

	rtc_dbg("rtc time %4d-%02d-%02d %02d:%02d:%02d (%lds)\n",
			tm->tm_year, tm->tm_mon, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec
			, time_s);
	result = rtc_valid_tm(tm);

read_alarm_end:
	mutex_unlock(&rtcd->rtc_mutex);
	return result;
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

	mv_svc_rtc_get_alarm(&rtc_data2);
	if (rtc_data2.m_year != 0) {
		mv_svc_rtc_clear_alarm();
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
		mv_svc_rtc_set_alarm(&rtc_data);
	}
	spin_unlock(&rtcd->lock);

	return 0;
}

static int xgold_rtc_clear_alarm_async(struct device *dev)
{
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);
	int result = 0, ret_wait = 0, clr_res = 0;

	mutex_lock(&rtcd->rtc_mutex);
	rtcd->service_completed = 0;
	rtcd->opcode = RTC_CLEAR_ALARM;
	clr_res = mv_svc_rtc_clear_alarm();
	if (clr_res != 0) {
		rtc_err(
			"clear alarm failed\n");
		goto clr_alarm_end;
	}
	ret_wait = wait_event_interruptible_timeout(rtcd->waitqueue,
			rtcd->service_completed, msecs_to_jiffies(2000));
	if (ret_wait == 0) { /* timeout */
		rtc_err("incomplete rtc services after 2 seconds\n");
		result = -ETIME;
		goto clr_alarm_end;
	} else if (ret_wait == -ERESTARTSYS) {
		/* interrupted by signal */
		rtc_err("wait_event interrupted by signal\n");
		result = -ERESTARTSYS;
		goto clr_alarm_end;
	}
clr_alarm_end:
	mutex_unlock(&rtcd->rtc_mutex);
	return result;
}

static int xgold_rtc_set_alarm_async(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned long time;
	struct rtc_driver_data *rtcd = dev_get_drvdata(dev);
	struct rtc_time *tm;
	int result = 0, ret_wait = 0, set_res = 0;
	struct rtc_wkalrm alm2;

	if (rtc_tm_to_time(&alm->time, &time)) {
		dev_info(dev, "RTC: conversion from time to seconds failed");
		return -EINVAL;
	}

	/* clear any previously set alarm */
	xgold_rtc_read_alarm_async(dev, &alm2);
	if (alm2.time.tm_hour != 0) {
		xgold_rtc_clear_alarm_async(dev);
		rtc_dbg("clear alarm\n");
	}

	/* now set new alarm into rtc */
	mutex_lock(&rtcd->rtc_mutex);
	tm = &alm->time;
	rtc_dbg("rtc time %4d-%02d-%02d %02d:%02d:%02d (%lds)\n",
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
		rtc_dbg("hum time %4d-%02d-%02d %02d:%02d:%02d\n",
			rtc_data.m_year, rtc_data.m_month, rtc_data.m_day,
			rtc_data.m_hour, rtc_data.m_minute, rtc_data.m_second);
		rtcd->service_completed = 0;
		rtcd->opcode = RTC_SET_ALARM;
		set_res = mv_svc_rtc_set_alarm(&rtc_data);
		if (set_res != 0) {
			rtc_err("set alarm failed\n");
			goto set_alarm_end;
		}
		ret_wait = wait_event_interruptible_timeout(rtcd->waitqueue,
				rtcd->service_completed,
				msecs_to_jiffies(2000));
		if (ret_wait == 0) { /* timeout */
			rtc_err("incomplete rtc services after 2 seconds\n");
			result = -ETIME;
			goto set_alarm_end;
		} else if (ret_wait == -ERESTARTSYS) {
			/* interrupted by signal */
			rtc_err("wait_event interrupted by signal\n");
			result = -ERESTARTSYS;
			goto set_alarm_end;
		}
	} else {
		pr_err("%s: Invalid RTC value !\n", __func__);
	}
set_alarm_end:
	mutex_unlock(&rtcd->rtc_mutex);
	return 0;
}

static int xgold_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	XGOLD_RTC_ENTER;
	/* NOTHING TO DO */

	return 0;
}

static struct rtc_class_ops xgold_rtc_class_ops_async = {
	.read_time = xgold_rtc_read_time_async,
	.set_time = xgold_rtc_set_time_async,
	.read_alarm = xgold_rtc_read_alarm_async,
	.set_alarm = xgold_rtc_set_alarm_async,
	.alarm_irq_enable = xgold_rtc_alarm_irq_enable,
};

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
	struct rtc_class_ops *ptr_class_ops = NULL;

	XGOLD_RTC_ENTER;
	spin_lock_init(&rtcd->lock);
	if (!device_can_wakeup(dev))
		ret = device_init_wakeup(dev, 1);

	/* register rtc irq by one handle (the same handle) */
	ret = irq_of_parse_and_map(dev->of_node, 0);
	if (!ret) {
		dev_err(dev, "cannot map rtc alarm irq 0");
	} else {
		dev_info(dev, "map rtc alarm irq index %d", ret);
		rtcd->irq = ret;
		ret = request_irq(rtcd->irq, xgold_rtc_irq,
				IRQF_NO_SUSPEND | IRQF_SHARED,
				dev_name(dev), rtcd);
		if (ret) {
			dev_err(dev, "failure in requesting rtc alarm irq\n");
			goto fail0;
		}
	}
	if (rtcd->async_mode) {
		ret = irq_of_parse_and_map(dev->of_node, 1);
		if (!ret) {
			dev_err(dev,
				"cannot map rtc service completion irq 1");
		} else {
			dev_info(dev,
				"map rtc service completion irq index %d",
				ret);
			dev_info(dev, "map irq index %d", ret);
			rtcd->completion_irq = ret;
			ret = request_irq(rtcd->completion_irq,
					xgold_rtc_completion_irq,
					IRQF_NO_SUSPEND | IRQF_SHARED,
					dev_name(dev), rtcd);
			if (ret) {
				dev_err(dev,
					"failure in requesting rtc completion irq\n"
					);
				goto fail0;
			}
		}
		ptr_class_ops = &xgold_rtc_class_ops_async;
		rtcd->service_completed = 0;
		mutex_init(&rtcd->rtc_mutex);
		init_waitqueue_head(&rtcd->waitqueue);
	} else
		ptr_class_ops = &xgold_rtc_class_ops;

	rtcd->rtc = devm_rtc_device_register(dev, dev_name(dev),
					ptr_class_ops, THIS_MODULE);
	if (IS_ERR(rtcd->rtc)) {
		dev_dbg(dev, "%s: can't register RTC device, err %ld\n",
			dev_name(dev), PTR_ERR(rtcd->rtc));
		goto fail0;
	}

	alarmtimer_rtc_interface.class = rtc_class;
	class_interface_register(&alarmtimer_rtc_interface);

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
