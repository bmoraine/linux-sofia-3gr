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
#define MEAS_INTERVAL 0			/* 80 ms */
#define MEAS_DELAY 90			/* us */
#define SW_HYST_DELAY 70		/* ms */

#define SW_HYST_VAL 0			/* LSB */
#define ACCURACY 2				/* LSB */


#define AF_RDONLY   00000000
#define AF_WRONLY   00000001
#define AF_RDWR     00000002

#define MIN_TEMP    (-22400)
#define MAX_TEMP    (133000)

#define MAX_SPCU_THERMAL_DEVICE (10)

enum {
	SPCU_TDEV_THERMAL = 0,
	SPCU_TDEV_OVERHEAT,
};

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
	OHCONF_REG,
	REG_COUNT,
};

static char *dts_reg[REG_COUNT] = {
	[CONFIG_REG] = "intel,config-reg",
	[CTRL_REG] = "intel,ctrl-reg",
	[STAT_REG] = "intel,stat-reg",
	[OHCONF_REG] = "intel,ohconf-reg",
};

struct spcu_thermal_platform_data {
	int id;
	const char *type;
	int reg_offset[REG_COUNT];

	int device_type;
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


#define TRIP_COUNT	2
#define TRIP_RW_MASK ((0x1 << TRIP_COUNT) - 1)

#define INVALID_TRIP (-1)

#define TRIP_STATE_UNKNOWN	(-1)
#define TRIP_STATE_LOW		(0)
#define TRIP_STATE_HIGH		(1)

#define SPCU_THERMAL_DEV_DBG

#ifdef SPCU_THERMAL_DEV_DBG
#define SPCU_THERMAL_TRACE(dev, format, args...) \
	dev_dbg(dev, format, ##args)
#else
#define SPCU_THERMAL_TRACE(dev, format, args...) \
	pr_info("%s: dev=0x%08x " format, __func__, (unsigned int)dev, ##args)
#endif

struct thermal_trip {
	int temp;
	int state;
};

struct spcu_thermal_device;
struct threshold {
	struct spcu_thermal_device *tdev;
	int virq;
	int type;
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
	int trend;
	struct thermal_trip trip[TRIP_COUNT];

	int device_type;
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
static inline int set_##_name(struct threshold *threshold, u32 val) \
{ \
	struct spcu_thermal_device *dev = threshold->tdev; \
	int type = threshold->type;	\
	return spcu_reg_write(dev, val, _name[type]); \
} \
static inline int get_##_name(struct threshold *threshold, u32 *val) \
{ \
	struct spcu_thermal_device *dev = threshold->tdev; \
	int type = threshold->type;	\
	u32 value = 0; \
	int ret = spcu_reg_read(dev, _name[type], &value); \
	*val = _name[type]->parse(value); \
	return ret; \
}

#define DEFINE_SENSOR_ATTR_OPS(_name, _field) \
static inline int set_##_name(struct spcu_thermal_device *dev, u32 val) \
{ \
	return spcu_reg_write(dev, val, &_field##_ATTR); \
} \
static inline int get_##_name(struct spcu_thermal_device *dev, u32 *val) \
{ \
	u32 value = 0; \
	int ret = spcu_reg_read(dev, &_field##_ATTR, &value); \
	*val = _field##_ATTR.parse(value); \
	return ret; \
}

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
DEFINE_SPCU_REG_FIELD(CONFIG_REG, CONFIG_HWLOWEN, 28, 1, AF_RDWR)
DEFINE_SPCU_REG_FIELD(CONFIG_REG, CONFIG_HWHIGHEN, 29, 1, AF_RDWR)

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

/* SpcuTSenseXohconf */
DEFINE_SPCU_REG_FIELD(OHCONF_REG, OHCONF_MEASOHEN, 0, 1, AF_RDWR)
DEFINE_SPCU_REG_FIELD(OHCONF_REG, OHCONF_OHRSTEN, 1, 1, AF_RDWR)
DEFINE_SPCU_REG_FIELD(OHCONF_REG, OHCONF_OHTHRESHOLD, 24, 8, AF_RDWR)

#define round_div_s64(n, d) (div_s64(((n) + (div_s64((d), 2))), (d)))


struct spcu_thermal_device *spcu_thermal_device_array[MAX_SPCU_THERMAL_DEVICE];
int spcu_thermal_device_idx = 0;


/* regval = (degree + 22.4) / (degree *  0.001447 + 0.4192)*/
static inline s32 temp2regval(s32 temp)
{
	s64 temp64 = (s64)temp;
	s64 regval64;

	if (MIN_TEMP > temp64)
		temp64 = MIN_TEMP;
	if (MAX_TEMP < temp64)
		temp64 = MAX_TEMP;

	regval64 = round_div_s64((temp64 * 1000000LL + 22400000000LL),
			 (temp64 * 1447LL + 419200000LL));
	return (s32)regval64;
}

/* degree = (regval * 0.4192 - 22.4) / (1 - regval * 0.001447) */
static inline s32 regval2temp(s32 regval)
{
	s64 regval64 = (s64)regval;
	s64 temp64;

	temp64 = round_div_s64((419200000LL * regval64 - 22400000000LL),
		   (1000000LL - regval64 * 1447LL));
	return (s32)temp64;
}

static inline int spcu_reg_read(struct spcu_thermal_device *dev,
				struct field_attr *attr, u32 *val)
{
	int ret;

	ret = mv_svc_reg_read(dev->phy_base + dev->reg_offset[attr->owner],
		   val, (u32)-1);

	if (ret)
		dev_err(&dev->pdev->dev,
			"%s: failed, ret = %d\n", __func__, ret);

	return ret;
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
DEFINE_THRESHOLD_ATTR_OPS(hw_en, CONFIG_HWHIGHEN, CONFIG_HWLOWEN)
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
DEFINE_SENSOR_ATTR_OPS(meas_oh_en, OHCONF_MEASOHEN)
DEFINE_SENSOR_ATTR_OPS(oh_rst_en, OHCONF_OHRSTEN)
DEFINE_SENSOR_ATTR_OPS(oh_threshold, OHCONF_OHTHRESHOLD)


static void set_intr_enable(struct threshold *threshold, u32 enable)
{
	struct spcu_thermal_device *dev = threshold->tdev;
	int type = threshold->type;

	mv_svc_spcu_thermal_service(enable ?
		 SPCU_THERMAL_ENABLE_INTR : SPCU_THERMAL_DISABLE_INTR,
		 dev->id, type);
}

static int sw_meas_oneshot(struct threshold *threshold, u32 meas_val)
{
	int ret;
	u32 sw_meas_result;
	u32 sw_meas_valid;

	ret = set_trigger_val(threshold, meas_val);
	ret |= set_sw_meas_start(threshold, 1);

	udelay(MEAS_DELAY);

	ret |= get_sw_meas_valid(threshold, &sw_meas_valid);
	ret |= get_sw_meas_result(threshold, &sw_meas_result);

	if (ret || (!sw_meas_valid))
		return -1;

	return sw_meas_result;
}
static bool sw_confirm_meas(struct threshold *threshold)
{
	struct spcu_thermal_device *dev = threshold->tdev;
	u32 saved;
	u32 verify;
	u32 trigger_type;
	int ret;
	bool result;

	ret = get_trigger_val(threshold, &saved);
	ret |= get_trigger_type(threshold, &trigger_type);

	if (ret)
		return false;

	SPCU_THERMAL_TRACE(&dev->pdev->dev,
		"type=%s trigger_type=%d old value of crossed threshold is %d\n",
		dev->tzd->type, trigger_type, saved);

	switch (trigger_type) {
	case TRIGGER_BELOW:
		verify = saved + SW_HYST_VAL;
		break;
	case TRIGGER_ABOVE:
		verify = saved - SW_HYST_VAL;
		break;
	default:
		verify = saved;
	}

	if (sw_meas_oneshot(threshold, verify) == trigger_type)
		result = true;
	else
		result = false;

	set_trigger_val(threshold, saved);

	return result;
}


/*#define TEMP_CALC_STEP
*/
#define TEMP_CALC_BISECT

#if defined(TEMP_CALC_STEP)
static void update_thresholds(struct spcu_thermal_device *dev)
{
	u32 val;

	/* In this approach, moving thresholds towards
	   the triggered direction. The interval between
	   high and low thresholds is defined by ACCURACY. */
	switch (dev->triggered->type) {
	case THRESHOLD_LOW:
		if (get_trigger_val(&dev->threshold[THRESHOLD_LOW], &val)) {
			dev_err(&dev->pdev->dev, "%s[%d] error\n",
					__func__, __LINE__);
			return;
		}

		SPCU_THERMAL_TRACE(&dev->pdev->dev,
				"type=%s LOW threshold triggered\n",
				dev->tzd->type);

		SPCU_THERMAL_TRACE(&dev->pdev->dev, "setting high to %d\n",
				val + ACCURACY);
		set_trigger_val(&dev->threshold[THRESHOLD_HIGH],
				val + ACCURACY);
		SPCU_THERMAL_TRACE(&dev->pdev->dev, "setting low to %d\n",
				val - ACCURACY);
		set_trigger_val(&dev->threshold[THRESHOLD_LOW],
				val - ACCURACY);
		break;
	case THRESHOLD_HIGH:
		if (get_trigger_val(&dev->threshold[THRESHOLD_HIGH], &val)) {
			dev_err(&dev->pdev->dev, "%s[%d] error\n",
					__func__, __LINE__);
			return;
		}

		SPCU_THERMAL_TRACE(&dev->pdev->dev,
				"type = %s HIGH threshold triggered\n",
				dev->tzd->type);
		SPCU_THERMAL_TRACE(&dev->pdev->dev, "setting low to %d\n",
				val - ACCURACY);
		set_trigger_val(&dev->threshold[THRESHOLD_LOW],
				val - ACCURACY);
		SPCU_THERMAL_TRACE(&dev->pdev->dev, "setting high to %d\n",
				val + ACCURACY);
		set_trigger_val(&dev->threshold[THRESHOLD_HIGH],
				val + ACCURACY);
		break;
	}
}

static void update_temp(struct spcu_thermal_device *dev)
{
	u32 val;
	int cur_temp;
	int low_temp;
	int high_temp;

	if (get_trigger_val(&dev->threshold[THRESHOLD_LOW], &val)) {
		dev_err(&dev->pdev->dev, "%s[%d] error\n", __func__, __LINE__);
		return;
	}

	low_temp = regval2temp(val);

	if (get_trigger_val(&dev->threshold[THRESHOLD_HIGH], &val)) {
		dev_err(&dev->pdev->dev, "%s[%d] error\n", __func__, __LINE__);
		return;
	}

	high_temp = regval2temp(val);
	cur_temp = (low_temp+high_temp)/2;

	dev->trend =  (cur_temp >= dev->cached_temp) ?
		TRIGGER_ABOVE : TRIGGER_BELOW;

	dev->cached_temp = cur_temp;

	SPCU_THERMAL_TRACE(&dev->pdev->dev,
		"type=%s new temp set to %d mDegC\n",
		dev->tzd->type, dev->cached_temp);
}

static void calculate_temp_and_set_threshold(struct spcu_thermal_device *dev)
{
	update_thresholds(dev);
	update_temp(dev);
}

#elif defined(TEMP_CALC_BISECT)
static int do_bisect_temp_val(struct threshold *test_thres, u32 thres_low_val,
			u32 thres_high_val)
{
	int test_val;
	int high_val;
	int low_val;

	high_val = thres_high_val;
	low_val = thres_low_val;

	while ((high_val - low_val) > 1) {
		test_val = ((low_val + high_val) >> 1);
		if (sw_meas_oneshot(test_thres, test_val) == TRIGGER_ABOVE)
			low_val = test_val;
		else
			high_val = test_val;
	}

	if (high_val < 0)
		return -1;

	return high_val;
}

static int bisect_calc_temp_val(struct spcu_thermal_device *dev,
			u32 *cur_temp_val)
{
	u32 trigger_val;
	int trigger_type;
	u32 temp_val;
	int ret;

	ret = get_trigger_val(dev->triggered, &trigger_val);
	ret |= get_trigger_type(dev->triggered, &trigger_type);
	if (ret)
		return ret;

	if (trigger_type == TRIGGER_BELOW)
		temp_val = do_bisect_temp_val(
					&dev->threshold[THRESHOLD_LOW],
					0, trigger_val);
	else
		temp_val = do_bisect_temp_val(
					&dev->threshold[THRESHOLD_LOW],
					trigger_val, 255);

	if (temp_val < 0)
		return -1;

	*cur_temp_val = temp_val;

	return 0;

}

static void update_thresholds(struct spcu_thermal_device *dev, int cur_temp_val)
{
	SPCU_THERMAL_TRACE(&dev->pdev->dev, "setting low to %d\n",
				cur_temp_val - ACCURACY*2);
	set_trigger_val(&dev->threshold[THRESHOLD_LOW],
				cur_temp_val - ACCURACY*2);
	SPCU_THERMAL_TRACE(&dev->pdev->dev, "setting high to %d\n",
				cur_temp_val + ACCURACY);
	set_trigger_val(&dev->threshold[THRESHOLD_HIGH],
				cur_temp_val + ACCURACY);

}

static void calculate_temp_and_set_threshold(struct spcu_thermal_device *dev)
{
	u32 cur_temp_val;
	u32 cur_temp;

	if (bisect_calc_temp_val(dev, &cur_temp_val))
		return;

	cur_temp = regval2temp(cur_temp_val);

	dev->trend = (cur_temp >= dev->cached_temp) ?
		TRIGGER_ABOVE : TRIGGER_BELOW;
	dev->cached_temp = cur_temp;

	update_thresholds(dev, cur_temp_val);

	SPCU_THERMAL_TRACE(&dev->pdev->dev,
		"type = %s new temp set to %d mDegC\n",
		dev->tzd->type, dev->cached_temp);

}
#endif

static void notify_thermal_event(struct spcu_thermal_device *dev, int trip_id)
{
	struct thermal_zone_device *tzd = dev->tzd;
	char *thermal_event[4];

	thermal_event[0] = kasprintf(GFP_KERNEL,
						 "NAME=%s",
						 tzd->type);
	thermal_event[1] = kasprintf(GFP_KERNEL,
						 "TEMP=%d",
						 dev->cached_temp);
	thermal_event[2] =
		kasprintf(GFP_KERNEL,
				 "EVENT=%d", trip_id);
	thermal_event[3] = NULL;

	pr_info("%s: type=%s, trip_id=%d, temp=%d\n",
		__func__, tzd->type, trip_id, dev->cached_temp);

	kobject_uevent_env(&tzd->device.kobj, KOBJ_CHANGE, thermal_event);

	kfree(thermal_event[2]);
	kfree(thermal_event[1]);
	kfree(thermal_event[0]);
}

static inline void check_one_trip(struct spcu_thermal_device *dev, int trip_id)
{
	struct thermal_trip *trip;

	trip = &dev->trip[trip_id];

	if (trip->temp == INVALID_TRIP)
		return;

	if (dev->cached_temp < trip->temp &&
			trip->state != TRIP_STATE_HIGH) {
		trip->state = TRIP_STATE_HIGH;
		notify_thermal_event(dev, trip_id);
	} else if (dev->cached_temp >= trip->temp &&
			trip->state != TRIP_STATE_LOW) {
		trip->state = TRIP_STATE_LOW;
		notify_thermal_event(dev, trip_id);
	} else {
		return;
	}
}

static void check_trips(struct spcu_thermal_device *dev)
{
	int i;
	struct thermal_trip *trip;
	int notify_trip_id = -1;

	if (dev->trend == TRIGGER_BELOW) {
		for (i = (TRIP_COUNT-1); i >= 0; i--) {
			trip = &dev->trip[i];
			if (trip->temp == INVALID_TRIP)
				continue;

			if (dev->cached_temp < trip->temp &&
				trip->state != TRIP_STATE_HIGH) {
				trip->state = TRIP_STATE_HIGH;
				notify_trip_id = i;
			}
		}
	} else if (dev->trend == TRIGGER_ABOVE) {
		for (i = 0; i < TRIP_COUNT; i++) {
			trip = &dev->trip[i];
			if (trip->temp == INVALID_TRIP)
				continue;

			if (dev->cached_temp >= trip->temp &&
				trip->state != TRIP_STATE_LOW) {
				trip->state = TRIP_STATE_LOW;
				notify_trip_id = i;
			}
		}
	} else {
		return;
	}

	if (notify_trip_id >= 0)
		notify_thermal_event(dev, notify_trip_id);
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

	SPCU_THERMAL_TRACE(&dev->pdev->dev,
		"type=%s, trigger_type=%d\n",
		dev->tzd->type, dev->triggered->type);

	ret = sw_confirm_meas(dev->triggered);
	if (!ret) {
		SPCU_THERMAL_TRACE(&dev->pdev->dev,
			"type=%s, unstable trigger\n",
			dev->tzd->type);
		goto out;
	}

	calculate_temp_and_set_threshold(dev);
	check_trips(dev);
	if (dev->cached_temp >= 90000) {
		/* set the max thermal freq to lowest one */
		extern void set_thermal_scaling_max_freq_to_lowest(void);
		set_thermal_scaling_max_freq_to_lowest();
	}
out:
	/* re-enable interrupt and HW measurement timer */
	set_hw_meas_start(dev, 1);

	if (0 <= temp2regval(dev->cached_temp) - ACCURACY)
		set_intr_enable(&dev->threshold[THRESHOLD_LOW], 1);
	if (255 >= temp2regval(dev->cached_temp) + ACCURACY)
		set_intr_enable(&dev->threshold[THRESHOLD_HIGH], 1);

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

	if (info->reg_offset[OHCONF_REG] != 0) {
		mv_svc_reg_read(info->phy_base+info->reg_offset[OHCONF_REG],
				&val, -1);
		desc += sprintf(buf + desc,
					"ohconf addr: 0x%x val: 0x%x\n",
					info->reg_offset[OHCONF_REG], val);
	}

	return desc;
}

static DEVICE_ATTR(debug, S_IRUGO, show_debug, NULL);

static struct device_attribute *thermal_attributes[] = {
	&dev_attr_debug,
	NULL
};

static int show_temp(struct thermal_zone_device *thermal, unsigned long *temp)
{
	struct spcu_thermal_device *dev = thermal->devdata;

	*temp = dev->cached_temp;

	SPCU_THERMAL_TRACE(&dev->pdev->dev,
		"type=%s, temp=%d mDegC\n", dev->tzd->type,
		dev->cached_temp);

	return 0;
}

static int show_trip_type(struct thermal_zone_device *thermal,
				  int trip, enum thermal_trip_type *type)
{
	struct spcu_thermal_device *dev = thermal->devdata;

	*type = THERMAL_TRIP_ACTIVE;

	SPCU_THERMAL_TRACE(&dev->pdev->dev,
		"type=%s, trip_type=%d\n",
		dev->tzd->type, THERMAL_TRIP_ACTIVE);

	return 0;
}

static int show_trip_temp(struct thermal_zone_device *thermal,
				  int trip, unsigned long *temp)
{
	struct spcu_thermal_device *dev = thermal->devdata;

	*temp = dev->trip[trip].temp;
	SPCU_THERMAL_TRACE(&dev->pdev->dev,
		"type=%s, trip_id=%d, temp=%d mDegC\n",
		dev->tzd->type, trip, dev->trip[trip].temp);

	return 0;
}

static int store_trip_temp(struct thermal_zone_device *thermal,
				   int trip, unsigned long temp)
{
	struct spcu_thermal_device *dev = thermal->devdata;

	mutex_lock(&dev->lock);

	dev->trip[trip].temp = temp;
	check_one_trip(dev, trip);
	mutex_unlock(&dev->lock);
	SPCU_THERMAL_TRACE(&dev->pdev->dev,
		"type=%s, trip_id=%d, temp=%d mDegC\n",
		dev->tzd->type, trip, dev->trip[trip].temp);

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
		threshold->virq =
			platform_get_irq_byname(dev->pdev,
					threshold_init[i].virq_name);
		set_trigger_val(threshold, temp2regval(DEFAULT_TEMP));
		set_trigger_type(threshold, threshold_init[i].trigger_type);
	}

	for (i = 0; i < TRIP_COUNT; i++) {
		dev->trip[i].temp = INVALID_TRIP;
		dev->trip[i].state = TRIP_STATE_UNKNOWN;
	}

	dev->cached_temp = DEFAULT_TEMP;
	dev->trend = TRIGGER_ABOVE;
}

static irqreturn_t threshold_irq_handler(int irq, void *dev_id)
{
	struct threshold *threshold = dev_id;
	struct spcu_thermal_device *dev = threshold->tdev;

	/* disable interrupt because later SW measurement may trigger it */
	/* vmm had already disabled it before routing to Linux */
	/* set_intr_enable(threshold, 0); */

	set_hw_meas_stop(dev, 1);

	SPCU_THERMAL_TRACE(&dev->pdev->dev,
		"type=%s, trigger_type=%d\n",
		dev->tzd->type, threshold->type);

	dev->triggered = threshold;
	schedule_delayed_work(&dev->isr_work, msecs_to_jiffies(SW_HYST_DELAY));

	return IRQ_HANDLED;
}

static int spcu_thermal_irq_init(struct spcu_thermal_device *dev)
{
	struct platform_device *pdev = dev->pdev;
	int ret, i;
	struct threshold *threshold;
	int set_en = 0;

#ifdef CONFIG_OF
	{
		struct device_node *node;

		node = dev->pdev->dev.of_node;
		of_property_read_u32(node, "intel,config-set_hw", &set_en);
	}
#endif

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
		if (set_en)
			set_hw_en(threshold, 1);
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

	of_property_read_string(node, "intel,thermal-type", &pdata->type);

	if (of_property_read_u32(node, "intel,thermal-id", &pdata->id))
		return -EINVAL;

	if (of_property_read_u32(node, "intel,dev-type", &pdata->device_type))
		pdata->device_type = SPCU_TDEV_THERMAL;

	for (i = 0; i < REG_COUNT; i++)
		if (of_property_read_u32(node, dts_reg[i],
				 &pdata->reg_offset[i])) {

			if ((pdata->device_type != SPCU_TDEV_OVERHEAT) &&
				(i == OHCONF_REG))
				continue;

			return -EINVAL;
	}
	return 0;
}
#endif


/*****for thermal hardware overheat*********************/

static int enable_hw_overheat_reset(struct spcu_thermal_device *dev,
		bool enable)
{
	int ret;

	if (enable) {
		set_wakeup_en(dev, 1);
		set_meas_reset(dev, 1);
		set_hw_meas_interval(dev, MEAS_INTERVAL);
		set_hw_meas_start(dev, 1);

		ret = set_oh_rst_en(dev, 1);
		if (ret)
			return -EINVAL;

		ret = set_meas_oh_en(dev, 1);
		if (ret)
			return -EINVAL;
	} else {
		set_wakeup_en(dev, 0);
		set_hw_meas_stop(dev, 1);

		ret = set_oh_rst_en(dev, 0);
		if (ret)
			return -EINVAL;

		ret = set_meas_oh_en(dev, 0);
		if (ret)
			return -EINVAL;
	}

	return 0;
}

static int set_hw_overheat_thres(struct spcu_thermal_device *dev,
		int temp)
{
	int ret;
	u32 regval;

	regval = temp2regval(temp);
	ret = set_oh_threshold(dev, regval);
	if (ret)
		return -EINVAL;

	return 0;
}

static ssize_t show_debug_oh(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct spcu_thermal_device *info = platform_get_drvdata(pdev);
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

	mv_svc_reg_read(info->phy_base+info->reg_offset[OHCONF_REG], &val, -1);
	desc += sprintf(buf + desc,
				"ohconf addr: 0x%x val: 0x%x\n",
				info->reg_offset[OHCONF_REG], val);

	return desc;
}

static ssize_t show_hw_oh_reset_en(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct spcu_thermal_device *info = platform_get_drvdata(pdev);
	int desc = 0;
	int hw_oh_en;
	int ret;

	ret = get_oh_rst_en(info, &hw_oh_en);
	if (ret)
		return -EINVAL;

	desc += sprintf(buf, "%u\n", hw_oh_en);

	return desc;
}

static ssize_t show_hw_oh_reset_thres(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct spcu_thermal_device *info = platform_get_drvdata(pdev);
	int desc = 0;
	int hw_oh_threshold;
	int ret;

	ret = get_oh_threshold(info, &hw_oh_threshold);
	if (ret)
		return -EINVAL;

	hw_oh_threshold = regval2temp(hw_oh_threshold);

	desc += sprintf(buf, "%d\n", hw_oh_threshold);

	return desc;
}

static DEVICE_ATTR(debug_oh, S_IRUGO, show_debug_oh, NULL);
static DEVICE_ATTR(hw_oh_reset_en, S_IRUGO, show_hw_oh_reset_en,
						NULL);
static DEVICE_ATTR(hw_oh_reset_thres, S_IRUGO, show_hw_oh_reset_thres,
					NULL);

static struct device_attribute *thermal_oh_attributes[] = {
	&dev_attr_debug_oh,
	&dev_attr_hw_oh_reset_en,
	&dev_attr_hw_oh_reset_thres,
	NULL
};

static int spcu_thermal_overheat_init(struct platform_device *pdev,
				 struct spcu_thermal_platform_data *pdata)
{
	int i;
	int err;
	struct spcu_thermal_device *dev = platform_get_drvdata(pdev);

	dev->id = pdata->id;
	dev->reg_offset = pdata->reg_offset;

	for (i = 0; thermal_oh_attributes[i]; i++) {
		err = device_create_file(&pdev->dev, thermal_oh_attributes[i]);
		if (err)
			return err;
	}

	set_hw_overheat_thres(dev, 115000);
	enable_hw_overheat_reset(dev, 1);

	return 0;
}

static int spcu_thermal_overheat_deinit(struct platform_device *pdev)
{
	int i;

	for (i = 0; thermal_oh_attributes[i]; i++)
		device_remove_file(&pdev->dev, thermal_oh_attributes[i]);

	return 0;
}
/*****!!for thermal hardware overheat*********************/

static int spcu_thermal_panic_notify(struct notifier_block *n,
		unsigned long val, void *v)
{
	uint32_t i = 0;
	pr_info("%s\n", __func__);
	for (i = 0; i < MAX_SPCU_THERMAL_DEVICE; i++) {
		struct spcu_thermal_device *dev = spcu_thermal_device_array[i];
		if (dev)
			dev_emerg(&dev->pdev->dev, "type=%s, temp=%d mDegC\n",
				dev->tzd->type, dev->cached_temp);
	}
	return NOTIFY_OK;
}

static struct notifier_block spcu_thermal_panic_notifier = {
	.notifier_call = spcu_thermal_panic_notify,
};

void spcu_thermal_register_panic_notifier(void)
{
	atomic_notifier_chain_register(&panic_notifier_list,
			&spcu_thermal_panic_notifier);
}

void spcu_thermal_unregister_panic_notifier(void)
{
	atomic_notifier_chain_unregister(&panic_notifier_list,
			&spcu_thermal_panic_notifier);
}

static int spcu_thermal_probe(struct platform_device *pdev)
{
	int err;
	struct spcu_thermal_device *thermal_device;
	struct spcu_thermal_platform_data *pdata = pdev->dev.platform_data;
	struct resource *res;
	char thermal_type[THERMAL_NAME_LENGTH];
	int i;

#ifdef CONFIG_OF
	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	pdata->type = devm_kzalloc(&pdev->dev, THERMAL_NAME_LENGTH, GFP_KERNEL);
	if (!pdata->type)
		return -ENOMEM;

	err = spcu_thermal_get_of_pdata(&pdev->dev, pdata);
	if (err)
		return err;
#endif

	thermal_device = devm_kzalloc(&pdev->dev,
					  sizeof(*thermal_device), GFP_KERNEL);
	if (!thermal_device)
		return -ENOMEM;

	if (MAX_SPCU_THERMAL_DEVICE == spcu_thermal_device_idx) {
		pr_err("increase the MAX_SPCU_THERMAL_DEVICE");
		return -ENOMEM;
	}
	spcu_thermal_device_array[spcu_thermal_device_idx++] = thermal_device;

	thermal_device->pdev = pdev;
	platform_set_drvdata(pdev, thermal_device);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no I/O memory defined in platform data\n");
		return -EINVAL;
	}
	thermal_device->phy_base = res->start;

	mutex_init(&thermal_device->lock);

	thermal_device->device_type = pdata->device_type;

	if (thermal_device->device_type == SPCU_TDEV_OVERHEAT) {
		err = spcu_thermal_overheat_init(pdev, pdata);
		if (err)
			return err;
	}

	INIT_DELAYED_WORK(&thermal_device->isr_work,
		  spcu_thermal_device_isr_work);

	spcu_thermal_device_init(thermal_device, pdata);
	spcu_thermal_irq_init(thermal_device);

	if (strlen(pdata->type))
		snprintf(thermal_type, sizeof(thermal_type), "%s", pdata->type);
	else
		snprintf(thermal_type, sizeof(thermal_type),
			 "%s%d", TZDNAME, thermal_device->id);

	thermal_device->tzd =
		thermal_zone_device_register(thermal_type,
			 TRIP_COUNT, TRIP_RW_MASK,
			 thermal_device, &tzd_ops, NULL, 0, 0);

	if (IS_ERR(thermal_device->tzd)) {
		dev_err(&pdev->dev, "Register thermal zone device failed.\n");
		err = PTR_ERR(thermal_device->tzd);
		return err;
	}
	dev_info(&pdev->dev,
		"Thermal zone device registered, type=%s\n",
		thermal_type);

	for (i = 0; thermal_attributes[i]; i++) {
		err = device_create_file(&thermal_device->tzd->device,
				thermal_attributes[i]);
		if (err)
			return err;
	}

	set_meas_reset(thermal_device, 1);
	set_hw_meas_interval(thermal_device, MEAS_INTERVAL);
	set_hw_meas_start(thermal_device, 1);

	return 0;
}

static int spcu_thermal_remove(struct platform_device *pdev)
{
	struct spcu_thermal_device *dev = platform_get_drvdata(pdev);
	int i;

	if (dev->device_type == SPCU_TDEV_OVERHEAT) {
		spcu_thermal_overheat_deinit(pdev);
		return 0;
	}

	set_hw_meas_stop(dev, 1);
	spcu_thermal_irq_deinit(dev);

	for (i = 0; thermal_attributes[i]; i++)
		device_remove_file(&dev->tzd->device, thermal_attributes[i]);

	thermal_zone_device_unregister(dev->tzd);

	return 0;
}

#ifdef CONFIG_PM
static int spcu_thermal_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct spcu_thermal_device *dev = platform_get_drvdata(pdev);
	int i;

	if (dev->device_type != SPCU_TDEV_THERMAL)
		return 0;

	for (i = 0; i < THRESHOLD_COUNT; i++)
		disable_irq(dev->threshold[i].virq);

	cancel_delayed_work_sync(&dev->isr_work);

	set_hw_meas_stop(dev, 1);

	for (i = 0; i < THRESHOLD_COUNT; i++)
		set_intr_enable(&dev->threshold[i], 0);

	return 0;
}

static int spcu_thermal_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct spcu_thermal_device *dev = platform_get_drvdata(pdev);
	int i;

	if (dev->device_type != SPCU_TDEV_THERMAL)
		return 0;

	set_hw_meas_start(dev, 1);

	for (i = 0; i < THRESHOLD_COUNT; i++)
		set_intr_enable(&dev->threshold[i], 1);

	for (i = 0; i < THRESHOLD_COUNT; i++)
		enable_irq(dev->threshold[i].virq);

	return 0;
}


static const struct dev_pm_ops spcu_thermal_pm_ops = {
	.suspend = spcu_thermal_suspend,
	.resume = spcu_thermal_resume,
};

#define SPCU_THERMAL_PM_OPS		(&spcu_thermal_pm_ops)

#else
#define SPCU_THERMAL_PM_OPS		NULL
#endif


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
		.pm = SPCU_THERMAL_PM_OPS,
	},
};

static int __init spcu_thermal_driver_init(void)
{
	spcu_thermal_register_panic_notifier();
	return platform_driver_register(&spcu_thermal_driver);
}
module_init(spcu_thermal_driver_init);

static void __exit spcu_thermal_driver_exit(void)
{
	spcu_thermal_unregister_panic_notifier();
	platform_driver_unregister(&spcu_thermal_driver);
}
module_exit(spcu_thermal_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SoFIA SPCU Thermal Driver");
