/*
 * Copyright (C) 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define DRVNAME	"spcu_thermal"
#define TZDNAME "coretemp"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio-fan.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/thermal.h>
#include <linux/math64.h>
#include <sofia/mv_hypercalls.h>
#include <sofia/mv_svc_hypercalls.h>

#define DEFAULT_TEMP 40000		/* 40 degrees */
#define MEAS_INTERVAL 3			/* 80 ms */
#define MEAS_DELAY 90			/* us */
#define SW_HYST_DELAY 100		/* ms */
#define INVALID_TRIP -1

#define SW_HYST_VAL 4			/* LSB */
#define ACCURACY 8				/* LSB */

#define TRIP_RW_MASK ((0x1 << THRESHOLD_COUNT) - 1)

#define AF_RDONLY   00000000
#define AF_WRONLY   00000001
#define AF_RDWR     00000002

enum {
	TRIGGER_BELOW = 0,
	TRIGGER_ABOVE,
};

enum {
	THRESHOLD_LOW = 0,
	THRESHOLD_HIGH,
	THRESHOLD_COUNT,
};

enum {
	CONFIG_REG,
	CTRL_REG,
	STAT_REG,
	REG_COUNT,
};

static char *dts_reg[REG_COUNT] = {
	[CONFIG_REG] = "intel,config-reg",
	[CTRL_REG] = "intel,ctrl-reg",
	[STAT_REG] = "intel,stat-reg",
};

struct spcu_thermal_platform_data {
	int id;
	int reg_offset[REG_COUNT];
};

static struct {
	int trigger_type;
	char *virq_name;
} threshold_init[THRESHOLD_COUNT] = {
	[THRESHOLD_LOW] = {
		.virq_name = "TLOW",
		.trigger_type = TRIGGER_BELOW,
	},
	[THRESHOLD_HIGH] = {
		.virq_name = "THIGH",
		.trigger_type = TRIGGER_ABOVE,
	},
};

struct spcu_thermal_device;
struct threshold {
	struct spcu_thermal_device *tdev;
	int virq;
	int type;
	int trip_temp;
};

struct spcu_thermal_device {
	struct platform_device	*pdev;
	struct thermal_zone_device *tzd;
	struct mutex		lock;
	struct threshold *triggered;
	struct threshold threshold[THRESHOLD_COUNT];
	struct delayed_work isr_work;

	int id;
	u32 phy_base;
	u32 *reg_offset;
	int cached_temp;
};

struct field_attr {
	u32 af;
	u32 owner;
	u32 mask;
	u32 (*parse)(u32 regval);
	u32 (*make)(u32 val);
};

#define DEFINE_SPCU_REG_FIELD(_reg, _field, _shift, _width, _af) \
static const  u32 _field##_MASK = (~(((u32)-1) << _width)) << _shift; \
static inline u32 _field##_PARSE(u32 regval) \
{ \
	return (regval & _field##_MASK) >> _shift; \
} \
static inline u32 _field##_MAKE(u32 val) \
{ \
	return val << _shift; \
} \
static struct field_attr _field##_ATTR = { \
	.af     = _af, \
	.owner  = _reg,	\
	.mask   = (~(((u32)-1) << _width)) << _shift, \
	.parse  = _field##_PARSE, \
	.make   = _field##_MAKE, \
};

#define DEFINE_THRESHOLD_ATTR_OPS(_name, _field1, _field2) \
static struct field_attr *_name[] = { &_field1##_ATTR, &_field2##_ATTR, }; \
static inline void set_##_name(struct threshold *threshold, u32 val) \
{ \
	struct spcu_thermal_device *dev = threshold->tdev; \
	int type = threshold->type;	\
	spcu_reg_write(dev, val, _name[type]); \
} \
static inline int get_##_name(struct threshold *threshold) \
{ \
	struct spcu_thermal_device *dev = threshold->tdev; \
	int type = threshold->type;	\
	return _name[type]->parse(spcu_reg_read(dev, _name[type]));	\
}

#define DEFINE_SENSOR_ATTR_OPS(_name, _field) \
static inline void set_##_name(struct spcu_thermal_device *dev, u32 val) \
{ spcu_reg_write(dev, val, &_field##_ATTR); } \
static inline u32 get_##_name(struct spcu_thermal_device *dev) \
{ return _field##_ATTR.parse(spcu_reg_read(dev, &_field##_ATTR)); }	\

/* SpcuTSenseXConfig */
DEFINE_SPCU_REG_FIELD(CONFIG_REG, CONFIG_TLOW,  0, 8, AF_RDWR)
DEFINE_SPCU_REG_FIELD(CONFIG_REG, CONFIG_THIGH, 8, 8, AF_RDWR)
DEFINE_SPCU_REG_FIELD(CONFIG_REG, CONFIG_TMEASTIMVAL, 16, 3, AF_RDWR)
#ifdef NOTYET
DEFINE_SPCU_REG_FIELD(CONFIG_REG, CONFIG_TLOWINTEN, 20, 1, AF_RDWR)
DEFINE_SPCU_REG_FIELD(CONFIG_REG, CONFIG_THIGHINTEN, 25, 1, AF_RDWR)
#endif
DEFINE_SPCU_REG_FIELD(CONFIG_REG, CONFIG_TLOWINTSEL, 21, 1, AF_RDWR)
DEFINE_SPCU_REG_FIELD(CONFIG_REG, CONFIG_THIGHINTSEL, 24, 1, AF_RDWR)
DEFINE_SPCU_REG_FIELD(CONFIG_REG, CONFIG_TWAKEUPEN, 26, 1, AF_RDWR)

/* SpcuTSenseXCtrl */
DEFINE_SPCU_REG_FIELD(CTRL_REG, CTRL_TMEASLOWSTART, 0, 1, AF_WRONLY)
DEFINE_SPCU_REG_FIELD(CTRL_REG, CTRL_TMEASHIGHSTART, 1, 1, AF_WRONLY)
DEFINE_SPCU_REG_FIELD(CTRL_REG, CTRL_TSENSEOFF, 2, 1, AF_WRONLY)
DEFINE_SPCU_REG_FIELD(CTRL_REG, CTRL_TMEASTIMSTART, 4, 1, AF_WRONLY)
DEFINE_SPCU_REG_FIELD(CTRL_REG, CTRL_TMEASTIMSTOP, 5, 1, AF_WRONLY)
DEFINE_SPCU_REG_FIELD(CTRL_REG, CTRL_TMEASRES, 8, 1, AF_WRONLY)

/* SpcuTSenseXStat */
DEFINE_SPCU_REG_FIELD(STAT_REG, STAT_THIGHVALID, 0, 1, AF_RDONLY)
DEFINE_SPCU_REG_FIELD(STAT_REG, STAT_TLOWVALID, 1, 1, AF_RDONLY)
DEFINE_SPCU_REG_FIELD(STAT_REG, STAT_THIGH, 2, 1, AF_RDONLY)
DEFINE_SPCU_REG_FIELD(STAT_REG, STAT_TLOW, 3, 1, AF_RDONLY)

#define round_div_u64(n, d) (div_u64(((n) + (div_u64((d), 2))), (d)))

/* regval = (degree + 21.79) / (degree * 0.001461 + 0.412003)  */
static inline u32 temp2regval(u32 temp)
{
	u64 temp64 = (u64)temp;
	u64 regval64;

	regval64 = round_div_u64((temp64 * 1000000ULL + 21790000000ULL),
			 (temp64 * 1461ULL + 412003000ULL));
	return (u32)regval64;
}

/* degree = (regval * 0.01315 + 251.21) / (1 - regval * 0.001461) - 273 */
static inline u32 regval2temp(u32 regval)
{
	u64 regval64 = (u64)regval;
	u64 temp64;

	temp64 = round_div_u64((13150000ULL * regval64 + 251210000000ULL),
		   (1000000ULL - regval64 * 1461ULL)) - 273000ULL;
	return (u32)temp64;
}

static inline u32 spcu_reg_read(struct spcu_thermal_device *dev,
				struct field_attr *attr)
{
	u32 val;
	int ret;

	ret = mv_svc_reg_read(dev->phy_base + dev->reg_offset[attr->owner],
		   &val, (u32)-1);

	if (ret)
		dev_err(&dev->pdev->dev,
			"%s: failed, ret = %d\n", __func__, ret);

	return val;
}

static int spcu_reg_write(struct spcu_thermal_device *dev,
			  u32 val, struct field_attr *attr)
{
	int ret;

	if (attr->af == AF_WRONLY)
		ret = mv_svc_reg_write_only(
			dev->phy_base+dev->reg_offset[attr->owner],
			attr->make(val), attr->mask);
	else
		ret = mv_svc_reg_write(
			dev->phy_base+dev->reg_offset[attr->owner],
			attr->make(val), attr->mask);

	if (ret)
		dev_err(&dev->pdev->dev,
			"%s: failed, ret = %d\n", __func__, ret);

	return ret;
}

DEFINE_THRESHOLD_ATTR_OPS(trigger_val, CONFIG_TLOW, CONFIG_THIGH)
DEFINE_THRESHOLD_ATTR_OPS(trigger_type,
	  CONFIG_TLOWINTSEL, CONFIG_THIGHINTSEL)
DEFINE_THRESHOLD_ATTR_OPS(sw_meas_start,
	  CTRL_TMEASLOWSTART, CTRL_TMEASHIGHSTART)
DEFINE_THRESHOLD_ATTR_OPS(sw_meas_valid, STAT_TLOWVALID, STAT_THIGHVALID)
DEFINE_THRESHOLD_ATTR_OPS(sw_meas_result, STAT_TLOW, STAT_THIGH)

DEFINE_SENSOR_ATTR_OPS(hw_meas_interval, CONFIG_TMEASTIMVAL)
DEFINE_SENSOR_ATTR_OPS(wakeup_en, CONFIG_TWAKEUPEN)
DEFINE_SENSOR_ATTR_OPS(sensor_power_off, CTRL_TSENSEOFF)
DEFINE_SENSOR_ATTR_OPS(hw_meas_start, CTRL_TMEASTIMSTART)
DEFINE_SENSOR_ATTR_OPS(hw_meas_stop, CTRL_TMEASTIMSTOP)
DEFINE_SENSOR_ATTR_OPS(meas_reset, CTRL_TMEASRES)

static void set_intr_enable(struct threshold *threshold, u32 enable)
{
	struct spcu_thermal_device *dev = threshold->tdev;
	int type = threshold->type;

	mv_svc_spcu_thermal_service(enable ?
		 SPCU_THERMAL_ENABLE_INTR : SPCU_THERMAL_DISABLE_INTR,
		 dev->id, type);
}

static int sw_confirm_meas(struct threshold *threshold)
{
	struct spcu_thermal_device *dev = threshold->tdev;
	u32 saved, verify;

	saved = get_trigger_val(threshold);
	dev_dbg(&dev->pdev->dev,
		"old value of crossed threshold is %d\n", saved);

	switch (get_trigger_type(threshold)) {
	case TRIGGER_BELOW:
		verify = saved + SW_HYST_VAL;
		break;
	case TRIGGER_ABOVE:
		verify = saved - SW_HYST_VAL;
		break;
	default:
		verify = saved;
	}

	set_trigger_val(threshold, verify);
	set_sw_meas_start(threshold, 1);

	udelay(MEAS_DELAY);

	set_trigger_val(threshold, saved);

	return get_sw_meas_valid(threshold) &&
	(get_sw_meas_result(threshold) ==
	 get_trigger_type(threshold));
}

static void update_thresholds(struct spcu_thermal_device *dev)
{
	/* In this approach, moving thresholds towards
	   the triggered direction. The interval between
	   high and low thresholds is defined by ACCURACY. */
	switch (dev->triggered->type) {
	case THRESHOLD_LOW:
		dev_dbg(&dev->pdev->dev, "LOW threshold triggered\n");
		dev_dbg(&dev->pdev->dev,
			"setting high to %d\n",
			get_trigger_val(&dev->threshold[THRESHOLD_LOW])+
				ACCURACY);
		set_trigger_val(&dev->threshold[THRESHOLD_HIGH],
			get_trigger_val(&dev->threshold[THRESHOLD_LOW])+
				ACCURACY);
		dev_dbg(&dev->pdev->dev,
			"setting low to %d\n",
			get_trigger_val(&dev->threshold[THRESHOLD_LOW])-
				ACCURACY);
		set_trigger_val(&dev->threshold[THRESHOLD_LOW],
			get_trigger_val(&dev->threshold[THRESHOLD_LOW])-
				ACCURACY);
		break;
	case THRESHOLD_HIGH:
		dev_dbg(&dev->pdev->dev, "HIGH threshold triggered\n");
		dev_dbg(&dev->pdev->dev,
			"setting low to %d\n",
			get_trigger_val(&dev->threshold[THRESHOLD_HIGH])-
				ACCURACY);
		set_trigger_val(&dev->threshold[THRESHOLD_LOW],
			get_trigger_val(&dev->threshold[THRESHOLD_HIGH])-
				ACCURACY);
		dev_dbg(&dev->pdev->dev,
			"setting high to %d\n",
			get_trigger_val(&dev->threshold[THRESHOLD_HIGH])+
				ACCURACY);
		set_trigger_val(&dev->threshold[THRESHOLD_HIGH],
			get_trigger_val(&dev->threshold[THRESHOLD_HIGH])+
				ACCURACY);
		break;
	}
}

static void update_temp(struct spcu_thermal_device *dev)
{
	int low_temp = regval2temp(get_trigger_val(
				   &dev->threshold[THRESHOLD_LOW]));
	int high_temp = regval2temp(get_trigger_val(
					&dev->threshold[THRESHOLD_HIGH]));

	dev->cached_temp = (low_temp+high_temp)/2;
	dev_dbg(&dev->pdev->dev, "new temp set to %d\n", dev->cached_temp);
}

static void notify_thermal_event(struct threshold *threshold)
{
	struct thermal_zone_device *tzd = threshold->tdev->tzd;
	struct spcu_thermal_device *dev = threshold->tdev;
	char *thermal_event[4];

	thermal_event[0] = kasprintf(GFP_KERNEL,
						 "NAME=%s",
						 tzd->type);
	thermal_event[1] = kasprintf(GFP_KERNEL,
						 "TEMP=%d",
						 dev->cached_temp);
	thermal_event[2] =
		kasprintf(GFP_KERNEL,
				 "EVENT=%d",
				 get_trigger_type(dev->triggered));
	thermal_event[3] = NULL;

	kobject_uevent_env(&tzd->device.kobj, KOBJ_CHANGE, thermal_event);

	kfree(thermal_event[2]);
	kfree(thermal_event[1]);
	kfree(thermal_event[0]);
}

static void check_trip_temp(struct threshold *threshold)
{
	struct spcu_thermal_device *dev = threshold->tdev;
	int type;

	if (threshold->trip_temp == INVALID_TRIP)
		return;

	type = get_trigger_type(threshold);
	if ((type == TRIGGER_BELOW &&
		 dev->cached_temp < threshold->trip_temp) ||
		(type == TRIGGER_ABOVE &&
		 dev->cached_temp > threshold->trip_temp))
		notify_thermal_event(threshold);
}

static void spcu_thermal_device_isr_work(struct work_struct *work)
{
	struct spcu_thermal_device *dev =
		container_of(work, struct spcu_thermal_device, isr_work.work);
	int ret;

	mutex_lock(&dev->lock);

	if (!dev->triggered) {
		dev_warn(&dev->pdev->dev, "invalid trigger\n");
		goto out;
	}

	ret = sw_confirm_meas(dev->triggered);
	if (!ret) {
		dev_warn(&dev->pdev->dev, "unstable trigger\n");
		goto out;
	}

	update_thresholds(dev);
	update_temp(dev);
	check_trip_temp(dev->triggered);

out:
	/* re-enable interrupt and HW measurement timer */
	set_hw_meas_start(dev, 1);
	set_intr_enable(dev->triggered, 1);

	mutex_unlock(&dev->lock);
}

static ssize_t show_debug(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct thermal_zone_device *thermal =
		container_of(dev, struct thermal_zone_device, device);
	struct spcu_thermal_device *info = thermal->devdata;
	int desc = 0;
	u32 val;

	mv_svc_reg_read(info->phy_base+info->reg_offset[CONFIG_REG], &val, -1);
	desc += sprintf(buf + desc,
				"config addr: 0x%x val: 0x%x\n",
				info->reg_offset[CONFIG_REG], val);

	mv_svc_reg_read(info->phy_base+info->reg_offset[STAT_REG], &val, -1);
	desc += sprintf(buf + desc,
				"stat addr: 0x%x val: 0x%x\n",
				info->reg_offset[STAT_REG], val);

	return desc;
}

static DEVICE_ATTR(debug, 0444, show_debug, NULL);

static int show_temp(struct thermal_zone_device *thermal, unsigned long *temp)
{
	struct spcu_thermal_device *dev = thermal->devdata;

	*temp = dev->cached_temp;

	return 0;
}

static int show_trip_type(struct thermal_zone_device *thermal,
				  int trip, enum thermal_trip_type *type)
{
	*type = THERMAL_TRIP_ACTIVE;

	return 0;
}

static int show_trip_temp(struct thermal_zone_device *thermal,
				  int trip, unsigned long *temp)
{
	struct spcu_thermal_device *dev = thermal->devdata;

	*temp = dev->threshold[trip].trip_temp;
	return 0;
}

static int store_trip_temp(struct thermal_zone_device *thermal,
				   int trip, unsigned long temp)
{
	struct spcu_thermal_device *dev = thermal->devdata;

	mutex_lock(&dev->lock);
	dev->threshold[trip].trip_temp = temp;
	check_trip_temp(&dev->threshold[trip]);
	mutex_unlock(&dev->lock);

	return 0;
}

static struct thermal_zone_device_ops tzd_ops = {
	.get_temp = show_temp,
	.get_trip_type = show_trip_type,
	.get_trip_temp = show_trip_temp,
	.set_trip_temp = store_trip_temp,
};

static void spcu_thermal_device_init(struct spcu_thermal_device *dev,
				 struct spcu_thermal_platform_data *pdata)
{
	struct threshold *threshold;
	int i;

	dev->id = pdata->id;
	dev->reg_offset = pdata->reg_offset;

	set_wakeup_en(dev, 1);
	/* request service of interrupt config */
	mv_svc_spcu_thermal_service(SPCU_THERMAL_REQUEST, dev->id, 0);

	for (i = 0; i < THRESHOLD_COUNT; i++) {
		threshold = &dev->threshold[i];
		threshold->tdev = dev;
		threshold->type = i;
		threshold->trip_temp = INVALID_TRIP;
		threshold->virq =
			platform_get_irq_byname(dev->pdev,
					threshold_init[i].virq_name);
		set_trigger_val(threshold, temp2regval(DEFAULT_TEMP));
		set_trigger_type(threshold, threshold_init[i].trigger_type);
	}

	dev->cached_temp = DEFAULT_TEMP;
}

static irqreturn_t threshold_irq_handler(int irq, void *dev_id)
{
	struct threshold *threshold = dev_id;
	struct spcu_thermal_device *dev = threshold->tdev;

	/* disable interrupt because later SW measurement may trigger it */
	/* vmm had already disabled it before routing to Linux */
	/* set_intr_enable(threshold, 0); */
	set_hw_meas_stop(dev, 1);

	dev->triggered = threshold;
	schedule_delayed_work(&dev->isr_work, msecs_to_jiffies(SW_HYST_DELAY));

	return IRQ_HANDLED;
}

static int spcu_thermal_irq_init(struct spcu_thermal_device *dev)
{
	struct platform_device *pdev = dev->pdev;
	int ret, i;
	struct threshold *threshold;

	for (i = 0; i < THRESHOLD_COUNT; i++) {
		threshold = &dev->threshold[i];
		ret = request_irq(threshold->virq,
				  threshold_irq_handler,
				  IRQF_SHARED,
				  dev_name(&pdev->dev), threshold);
		if (ret) {
			dev_err(&pdev->dev,
				"request irq %d failed, ret = %d\n",
				threshold->virq, ret);
			return ret;
		}
		set_intr_enable(threshold, 1);
	}

	return 0;
}

static void spcu_thermal_irq_deinit(struct spcu_thermal_device *dev)
{
	int i;
	struct threshold *threshold;

	for (i = 0; i < THRESHOLD_COUNT; i++) {
		threshold = &dev->threshold[i];
		set_intr_enable(threshold, 0);
		free_irq(threshold->virq, threshold);
	}
}

#ifdef CONFIG_OF
static int spcu_thermal_get_of_pdata(struct device *dev,
				 struct spcu_thermal_platform_data *pdata)
{
	struct device_node *node;
	int i;

	node = dev->of_node;

	if (of_property_read_u32(node, "intel,thermal-id", &pdata->id))
		return -EINVAL;

	for (i = 0; i < REG_COUNT; i++)
		if (of_property_read_u32(node, dts_reg[i],
				 &pdata->reg_offset[i]))
			return -EINVAL;

	return 0;
}
#endif

static int spcu_thermal_probe(struct platform_device *pdev)
{
	int err;
	struct spcu_thermal_device *thermal_device;
	struct spcu_thermal_platform_data *pdata = pdev->dev.platform_data;
	struct resource *res;
	char thermal_type[THERMAL_NAME_LENGTH];

#ifdef CONFIG_OF
	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	err = spcu_thermal_get_of_pdata(&pdev->dev, pdata);
	if (err)
		return err;
#endif

	thermal_device = devm_kzalloc(&pdev->dev,
					  sizeof(*thermal_device), GFP_KERNEL);
	if (!thermal_device)
		return -ENOMEM;

	thermal_device->pdev = pdev;
	platform_set_drvdata(pdev, thermal_device);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no I/O memory defined in platform data\n");
		return -EINVAL;
	}

	thermal_device->phy_base = res->start;

	mutex_init(&thermal_device->lock);
	INIT_DELAYED_WORK(&thermal_device->isr_work,
		  spcu_thermal_device_isr_work);

	spcu_thermal_device_init(thermal_device, pdata);
	spcu_thermal_irq_init(thermal_device);

	sprintf(thermal_type, "%s%d", TZDNAME, thermal_device->id);
	thermal_device->tzd =
		thermal_zone_device_register(thermal_type,
			 THRESHOLD_COUNT, TRIP_RW_MASK,
			 thermal_device, &tzd_ops, NULL, 0, 0);

	if (IS_ERR(thermal_device->tzd)) {
		dev_err(&pdev->dev, "Register thermal zone device failed.\n");
		err = PTR_ERR(thermal_device->tzd);
		return err;
	}
	dev_info(&pdev->dev, "Thermal zone device registered.\n");

	err = device_create_file(&thermal_device->tzd->device, &dev_attr_debug);

	set_meas_reset(thermal_device, 1);
	set_hw_meas_interval(thermal_device, MEAS_INTERVAL);
	set_hw_meas_start(thermal_device, 1);

	return 0;
}

static int spcu_thermal_remove(struct platform_device *pdev)
{
	struct spcu_thermal_device *dev = platform_get_drvdata(pdev);

	set_hw_meas_stop(dev, 1);
	spcu_thermal_irq_deinit(dev);
	device_remove_file(&dev->tzd->device, &dev_attr_debug);
	thermal_zone_device_unregister(dev->tzd);

	return 0;
}

static struct of_device_id of_spcu_thermal_match[] = {
	{ .compatible = "intel,spcu-thermal", },
	{},
};

static struct platform_driver spcu_thermal_driver = {
	.probe		= spcu_thermal_probe,
	.remove		= spcu_thermal_remove,
	.driver	= {
		.name	= DRVNAME,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(of_spcu_thermal_match),
#endif
	},
};

module_platform_driver(spcu_thermal_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SoFIA SPCU Thermal Driver");
