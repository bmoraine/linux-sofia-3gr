/*
 * rk818  battery driver
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/power/rk818_battery.h>
#include <linux/mfd/rk818.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/usb/otg.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>

/*if you want to disable, don't set it as 0,
just be: "static int dbg_enable;" is ok*/
static int dbg_enable;

module_param_named(dbg_level, dbg_enable, int, 0644);
#define DBG(args...) \
	do { \
		if (dbg_enable) { \
			pr_info(args); \
		} \
	} while (0)

#define INTERPOLATE_MAX			1000
#define MAX_INT				0x7FFF
#define POWER_OFF_3300			0x05
#define POWER_OFF_3400			0x06

enum {
	VBUS_OFF = 0,
	VBUS_ON,
};

struct battery_info {
	struct device *dev;
	struct cell_state cell;
	struct power_supply bat;
	struct power_supply ac;
	struct power_supply usb;
	struct delayed_work work;
	struct usb_phy *otg_handle;
	int vbus;
	int vbus_state_prev;
	int cable_type;
	unsigned int ts1_adc_val;
	unsigned int now_res_val;
	unsigned int pre_res_val;

	struct rk818 *rk818;
	struct battery_platform_data *platform_data;

	int work_on;
	int irq;
	int health;
	int present;
	int status;

	int bat_current;
	int current_avg;
	int current_offset;

	uint16_t voltage;
	uint16_t voltage_avg;
	uint16_t voltage_offset;
	uint16_t voltage_ocv;

	uint16_t warnning_voltage;

	int design_capacity;
	int fcc;
	int qmax;
	int remain_capacity;
	int nac;
	int temp_nac;

	int real_soc;
	int display_soc;
	int temp_soc;

	int soc_counter;

	int relax_in;
	int relax_out;

	int dod0;
	int dod0_status;
	int dod0_voltage;
	int dod0_capacity;
	unsigned long dod0_time;
	u8 dod0_level;

	int dod1;
	int dod1_status;
	int dod1_voltage;
	int dod1_capacity;
	unsigned long dod1_time;

	int update_cur_offset;

	int enter_flatzone;
	int exit_flatzone;

	int temperature;

	int time2empty;
	int time2full;

	int *ocv_table;
	int *res_table;

	int current_k;		/* (ICALIB0, ICALIB1) */
	int current_b;

	int voltage_k;		/* VCALIB0 VCALIB1 */
	int voltage_b;

	int resmode_vol1;
	int resmode_cur1;
	int resmode_vol2;
	int resmode_cur2;
	int resmode;
	int max_currunt_time;
	int min_current_time;

	int update_k;
	int line_k;
	int line_q;
	int update_q;

	int voltage_old;

	u8 check_count;
	struct timeval soc_timer;
	struct timeval change_timer;

	int vol_smooth_time;
	int charge_smooth_time;

	int suspend_capacity;
	int resume_capacity;
	struct timespec suspend_time;
	struct timespec resume_time;
	unsigned long suspend_test_time;
	unsigned long count_sleep_time;
	int sleep_soc;
	int sleep_status;
	int sleep_charge_current;
	int resume_soc;
	/* int bat_res; */
	bool charge_smooth_status;
	bool resume;

	struct notifier_block battery_nb;

	struct workqueue_struct *wq;
	struct delayed_work battery_monitor_work;
	struct delayed_work charge_check_work;
	int charge_otg;

	struct wake_lock resume_wake_lock;

};

struct battery_info *data;

#define	SPORT_USB_CHARGE

u32 interpolate(int value, u32 *table, int size)
{
	uint8_t i;
	uint16_t d;

	DBG("%s(): the battery OCV TABLE size : %d\n", __func__, size);
	DBG("%s(): the battery OCV TABLE :\n", __func__);
	for (i = 0; i < size; i++)
		DBG("%d ", table[i]);
	DBG("\n\n\n");

	DBG("%s(): i = %d, table[%d] = %d\n", __func__, i, i, table[i]);
	for (i = 0; i < size; i++) {
		if (value < table[i])
			break;
	}

	if ((i > 0) && (i < size)) {
		d = (value - table[i - 1]) * (INTERPOLATE_MAX / (size - 1));
		DBG("%s(): d = %d\n", __func__, d);
		d /= table[i] - table[i - 1];
		DBG("%s(): d /= %d\n", __func__, d);
		d = d + (i - 1) * (INTERPOLATE_MAX / (size - 1));
		DBG("%s(): d = %d\n", __func__, d);
	} else {
		d = i * ((INTERPOLATE_MAX + size / 2) / size);
	}

	if (d > 1000)
		d = 1000;
	DBG("%s(): d >= %d\n", __func__, d);
	return d;
}

/* Returns (a * b) / c */
int32_t ab_div_c(u32 a, u32 b, u32 c)
{
	bool sign;
	u32 ans = MAX_INT;
	int32_t tmp;

	sign = ((((a ^ b) ^ c) & 0x80000000) != 0);

	if (c != 0) {
		if (sign)
			c = -c;

		tmp = ((int32_t) a * b + (c >> 1)) / c;

		if (tmp < MAX_INT)
			ans = tmp;
	}

	if (sign)
		ans = -ans;

	return ans;
}

static int32_t abs_int(int32_t x)
{
	return (x > 0) ? x : -x;
}

static int abs32_int(int x)
{
	return (x > 0) ? x : -x;
}

/* Returns diviation between 'size' array members */
uint16_t diff_array(int16_t *arr, uint8_t size)
{
	uint8_t i;
	uint32_t diff = 0;

	for (i = 0; i < size - 1; i++)
		diff += abs_int(arr[i] - arr[i + 1]);

	if (diff > MAX_UNSIGNED_INT)
		diff = MAX_UNSIGNED_INT;

	return (uint16_t) diff;
}

static int battery_read(struct rk818 *rk818, u8 reg, u8 buf[], unsigned len)
{
	int ret;

	ret = rk818_i2c_read(rk818, reg, len, buf);
	return ret;
}

static int
battery_write(struct rk818 *rk818, u8 reg, u8 const buf[], unsigned len)
{
	int ret;
	ret = rk818_i2c_write(rk818, reg, (int) len, *buf);
	return ret;
}

#if 0
static void dump_gauge_register(struct battery_info *di)
{
	int i = 0;
	char buf;
	DBG("%s dump charger register start:\n", __func__);
	for (i = 0xAC; i < 0xDE; i++) {
		battery_read(di->rk818, i, &buf, 1);
		DBG(" the register is  0x%02x, the value is 0x%02x\n ", i, buf);
	}
	DBG("demp end!\n");
}

static void dump_charger_register(struct battery_info *di)
{

	int i = 0;
	char buf;
	DBG("%s dump the register start:\n", __func__);
	for (i = 0x99; i < 0xAB; i++) {
		battery_read(di->rk818, i, &buf, 1);
		DBG(" the register is  0x%02x, the value is 0x%02x\n ", i, buf);
	}
	DBG("demp end!\n");

}
#endif

#define BATT_NUM  11

static int batt_table[22];

static ssize_t
bat_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	for (i = 0; i < BATT_NUM; i++)
		pr_info("i=%d batt_table=%d\n", i, batt_table[i]);

	for (i = 0; i < BATT_NUM; i++)
		pr_info("i=%d batt_table=%d\n", i + BATT_NUM,
			batt_table[i + BATT_NUM]);
	return 0;
}

static ssize_t
bat_param_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

DEVICE_ATTR(batparam, 0664, bat_param_read, bat_param_write);
static uint16_t get_relax_voltage(struct battery_info *di);

static ssize_t
show_state_attrs(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_info("get_relax_voltage relax voltage = %d\n",
		get_relax_voltage(data));

	if (0 == get_relax_voltage(data)) {
		return sprintf(buf,
			       "voltage=%d,remain_capacity=%d,status=%d\n",
			       data->voltage, data->remain_capacity,
			       data->status);

	} else
		return sprintf(buf,
			       "voltage=%d,remain_capacity=%d,status=%d\n",
			       get_relax_voltage(data), data->remain_capacity,
			       data->status);
}

static ssize_t
restore_state_attrs(struct device *dev,
		    struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static struct device_attribute rkbatt_attrs[] = {
	__ATTR(state, 0664, show_state_attrs, restore_state_attrs),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int liTmep;
	for (liTmep = 0; liTmep < ARRAY_SIZE(rkbatt_attrs); liTmep++) {

		if (device_create_file(dev, rkbatt_attrs + liTmep))
			goto error;
	}

	return 0;

error:
	for (; liTmep >= 0; liTmep--)
		device_remove_file(dev, rkbatt_attrs + liTmep);

	dev_err(dev, "%s:Unable to create sysfs interface\n", __func__);
	return -1;
}

/*enabsle GG_EN*/
static int _gauge_enable(struct battery_info *di)
{
	int ret;
	u8 buf;

	ret = battery_read(di->rk818, TS_CTRL_REG, &buf, 1);
	DBG("_gauge_enable read-%d\n", buf);

	if (ret < 0) {
		dev_err(di->dev, "error reading TS_CTRL_REG");
		return ret;
	}
	if (!(buf & GG_EN)) {
		buf |= GG_EN;
		ret = battery_write(di->rk818, TS_CTRL_REG, &buf, 1);
		ret = battery_read(di->rk818, TS_CTRL_REG, &buf, 1);
		return 0;
	}

	DBG("%s,%d\n", __func__, buf);
	return 0;

}

static void save_level(struct battery_info *di, u8 save_soc)
{
	u8 soc;

	soc = save_soc;
	battery_write(di->rk818, UPDAT_LEVE_REG, &soc, 1);
	/*battery_read(di->rk818, SOC_REG, &soc, 1); */
	DBG(" the save UPDAT_LEVE_REG = %d\n", soc);

}

static u8 get_level(struct battery_info *di)
{
	u8 soc;

	battery_read(di->rk818, UPDAT_LEVE_REG, &soc, 1);
	DBG(" read UPDAT_LEVE_REG = %d\n", soc);

	return soc;
}

static int _get_vcalib0(struct battery_info *di)
{
	int ret;
	int temp = 0;
	u8 buf;

	ret = battery_read(di->rk818, VCALIB0_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, VCALIB0_REGH, &buf, 1);
	temp |= buf << 8;

	DBG("%s voltage0 offset vale is %d\n", __func__, temp);
	return temp;
}

static int _get_vcalib1(struct battery_info *di)
{
	int ret;
	int temp = 0;
	u8 buf;

	ret = battery_read(di->rk818, VCALIB1_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, VCALIB1_REGH, &buf, 1);
	temp |= buf << 8;

	DBG("%s voltage1 offset vale is %d\n", __func__, temp);
	return temp;
}

static int _get_ioffset(struct battery_info *di)
{
	int ret;
	int temp = 0;
	u8 buf;

	ret = battery_read(di->rk818, IOFFSET_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, IOFFSET_REGH, &buf, 1);
	temp |= buf << 8;

	DBG("%s IOFFSET value is %d\n", __func__, temp);
	return temp;
}

static uint16_t _get_cal_offset(struct battery_info *di)
{
	int ret;
	uint16_t temp = 0;
	u8 buf;

	ret = battery_read(di->rk818, CAL_OFFSET_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, CAL_OFFSET_REGH, &buf, 1);
	temp |= buf << 8;

	DBG("%s CAL_OFFSET value is %d\n", __func__, temp);
	return temp;
}

static int _set_cal_offset(struct battery_info *di, u32 value)
{
	int ret;
	int temp = 0;
	u8 buf;
	DBG("%s\n", __func__);

	buf = value & 0xff;
	ret = battery_write(di->rk818, CAL_OFFSET_REGL, &buf, 1);
	buf = (value >> 8) & 0xff;
	ret = battery_write(di->rk818, CAL_OFFSET_REGH, &buf, 1);
	DBG("%s set CAL_OFFSET_REG %d\n", __func__, temp);

	return 0;
}

static void _get_voltage_offset_value(struct battery_info *di)
{
	int vcalib0, vcalib1;

	vcalib0 = _get_vcalib0(di);
	vcalib1 = _get_vcalib1(di);

	di->voltage_k = (4200 - 3000) * 1000 / (vcalib1 - vcalib0);
	di->voltage_b = 4200 - (di->voltage_k * vcalib1) / 1000;
	DBG("voltage_k = %d(x1000) voltage_b = %d\n", di->voltage_k,
	    di->voltage_b);
	return;
}

static uint16_t _get_OCV_voltage(struct battery_info *di)
{
	int ret;
	u8 buf;
	uint16_t temp;
	uint16_t voltage_now = 0;

	ret = battery_read(di->rk818, BAT_OCV_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, BAT_OCV_REGH, &buf, 1);
	temp |= buf << 8;

	if (ret < 0) {
		dev_err(di->dev, "error reading BAT_OCV_REGH");
		return ret;
	}

	voltage_now = di->voltage_k * temp / 1000 + di->voltage_b;
	DBG("the OCV voltage is %d\n", voltage_now);

	return voltage_now;
}

static int rk818_battery_voltage(struct battery_info *di)
{
	int ret;
	int voltage_now = 0;
	u8 buf;
	int temp;

	ret = battery_read(di->rk818, BAT_VOL_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, BAT_VOL_REGH, &buf, 1);
	temp |= buf << 8;

	if (ret < 0) {
		dev_err(di->dev, "error reading BAT_VOL_REGH");
		return ret;
	}

	voltage_now = di->voltage_k * temp / 1000 + di->voltage_b;

	DBG("the rea-time voltage is %d\n", voltage_now);
	return voltage_now;
}

/* OCV Lookup table
 * Open Circuit Voltage (OCV) correction routine. This function estimates SOC,
 * based on the voltage.
 */
static int _voltage_to_capacity(struct battery_info *di, int voltage)
{
	u32 *ocv_table;
	int ocv_size;
	u32 tmp;

	ocv_table = di->platform_data->battery_ocv;
	ocv_size = di->platform_data->ocv_size;
	di->warnning_voltage = ocv_table[3];
	tmp = interpolate(voltage, ocv_table, ocv_size);
	di->temp_soc = ab_div_c(tmp, MAX_PERCENTAGE, INTERPOLATE_MAX);
	di->temp_nac = ab_div_c(tmp, di->fcc, INTERPOLATE_MAX);
	DBG("voltage = %d, temp = %d real-soc =%d nac= %d, fcc = %d\n", voltage,
	    tmp, di->temp_soc, di->temp_nac, di->fcc);
	return 0;
}

static uint16_t _get_relax_vol1(struct battery_info *di)
{
	int ret;
	u8 buf;
	uint16_t temp = 0, voltage_now;

	ret = battery_read(di->rk818, RELAX_VOL1_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, RELAX_VOL1_REGH, &buf, 1);
	temp |= (buf << 8);

	voltage_now = di->voltage_k * temp / 1000 + di->voltage_b;

	return voltage_now;
}

static uint16_t _get_relax_vol2(struct battery_info *di)
{
	int ret;
	uint16_t temp = 0, voltage_now;
	u8 buf;

	ret = battery_read(di->rk818, RELAX_VOL2_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, RELAX_VOL2_REGH, &buf, 1);
	temp |= (buf << 8);

	voltage_now = di->voltage_k * temp / 1000 + di->voltage_b;

	return voltage_now;
}

static bool _is_relax_mode(struct battery_info *di)
{
	int ret;
	u8 status;
	u8 ggcon;

	ret = battery_read(di->rk818, GGSTS, &status, 1);
	ret = battery_read(di->rk818, GGCON, &ggcon, 1);

	DBG(" GGSTS the value is 0x%x the ggcon = 0x%x\n", status, ggcon);

	if ((!(status & RELAX_VOL1_UPD)) || (!(status & RELAX_VOL2_UPD)))
		return false;
	else
		return true;
}

static uint16_t get_relax_voltage(struct battery_info *di)
{
	int ret;
	u8 status;
	uint16_t relax_vol1, relax_vol2;
	u8 ggcon;
	ret = battery_read(di->rk818, GGSTS, &status, 1);
	ret = battery_read(di->rk818, GGCON, &ggcon, 1);

	relax_vol1 = _get_relax_vol1(di);
	relax_vol2 = _get_relax_vol2(di);
	DBG("relax_vol1 %d, relaxe_v2 %d\n", relax_vol1, relax_vol2);

	if (_is_relax_mode(di)) {
		relax_vol1 = _get_relax_vol1(di);
		relax_vol2 = _get_relax_vol2(di);
		ggcon &= ~(RELAX_VOL1_UPD | RELAX_VOL2_UPD);
		battery_read(di->rk818, GGSTS, &status, 1);
		battery_read(di->rk818, GGCON, &ggcon, 1);
		DBG("relax_vol1 %d, relaxe_v2 %d\n", relax_vol1, relax_vol2);

		if (relax_vol1 > relax_vol2)
			return relax_vol1;
		else
			return relax_vol2;
	} else
		return 0;

}

static void _set_relax_thres(struct battery_info *di)
{
	u8 buf;
	int enter_thres, exit_thres;
	struct cell_state *cell = &di->cell;

	enter_thres = (cell->config->ocv->sleep_enter_current) * 1000 / 1506;
	exit_thres = (cell->config->ocv->sleep_exit_current) * 1000 / 1506;

	buf = enter_thres & 0xff;
	battery_write(di->rk818, RELAX_ENTRY_THRES_REGL, &buf, 1);
	buf = (enter_thres >> 8) & 0xff;
	battery_write(di->rk818, RELAX_ENTRY_THRES_REGH, &buf, 1);

	buf = exit_thres & 0xff;
	battery_write(di->rk818, RELAX_EXIT_THRES_REGL, &buf, 1);
	buf = (exit_thres >> 8) & 0xff;
	battery_write(di->rk818, RELAX_EXIT_THRES_REGH, &buf, 1);

	battery_read(di->rk818, GGCON, &buf, 1);
	buf &= ~(3 << 2);
	battery_write(di->rk818, GGCON, &buf, 1);
}

static void restart_relax(struct battery_info *di)
{
	u8 ggcon;
	u8 ggsts;

	battery_read(di->rk818, GGCON, &ggcon, 1);
	battery_read(di->rk818, GGSTS, &ggsts, 1);

	ggcon &= ~0x0c;
	ggsts &= ~0x0c;

	battery_write(di->rk818, GGCON, &ggcon, 1);
	battery_write(di->rk818, GGSTS, &ggsts, 1);

	battery_read(di->rk818, GGCON, &ggcon, 1);
	battery_read(di->rk818, GGSTS, &ggsts, 1);
	DBG("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
	DBG("GGCON =0x%x GGSTS = 0x%x\n", ggcon, ggsts);
	DBG("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");

}

static int _get_average_current(struct battery_info *di)
{
	u8 buf;
	int ret;
	int current_now;
	int temp;

	ret = battery_read(di->rk818, BAT_CUR_AVG_REGL, &buf, 1);
	if (ret < 0) {
		dev_err(di->dev, "error reading BAT_CUR_AVG_REGL");
		return ret;
	}
	current_now = buf;
	ret = battery_read(di->rk818, BAT_CUR_AVG_REGH, &buf, 1);
	if (ret < 0) {
		dev_err(di->dev, "error reading BAT_CUR_AVG_REGH");
		return ret;
	}
	current_now |= (buf << 8);

	DBG("adc vale current:  0x%x\n", current_now);
	if (current_now & 0x800)
		current_now -= 4096;

	temp = current_now * 1506 / 1000;	/*1000*90/14/4096*500/521 */

	if (ret < 0) {
		dev_err(di->dev, "error reading BAT_CUR_AVG_REGH");
		return ret;
	}

	DBG("%s, average current current_now = %d current = %d\n", __func__,
	    current_now, temp);
	return temp;
}

static bool _is_first_poweron(struct battery_info *di)
{
	u8 buf;
	u8 temp;
	u8 ret;

	ret = battery_read(di->rk818, GGSTS, &buf, 1);
	DBG("%s GGSTS value is %2x\n", __func__, buf);
	if (buf & BAT_CON) {
		buf &= ~(BAT_CON);
		do {
			battery_write(di->rk818, GGSTS, &buf, 1);
			battery_read(di->rk818, GGSTS, &temp, 1);
		} while (temp & BAT_CON);
		return true;
	}
	return false;
}

static void flatzone_voltage_init(struct battery_info *di)
{
	u32 *ocv_table;
	int ocv_size;
	int temp_table[21];
	int i, j;

	ocv_table = di->platform_data->battery_ocv;
	ocv_size = di->platform_data->ocv_size;

	for (j = 0; j < 21; j++)
		temp_table[j] = 0;
	j = 0;

	for (i = 1; i < ocv_size - 1; i++) {
		if (ocv_table[i + 1] < ocv_table[i] + 20)
			temp_table[j++] = i;
	}

	temp_table[j] = temp_table[j - 1] + 1;
	i = temp_table[0];
	di->enter_flatzone = ocv_table[i];
	j = 0;

	for (i = 0; i <= 20; i++) {
		if (temp_table[i] < temp_table[i + 1])
			j = i + 1;
	}

	i = temp_table[j];

	di->exit_flatzone = ocv_table[i];

	DBG("enter_flatzone = %d exit_flatzone =%d\n", di->enter_flatzone,
	    di->exit_flatzone);
}

static int is_not_flatzone(struct battery_info *di, int voltage)
{
	if ((voltage >= di->enter_flatzone) && (voltage <= di->exit_flatzone)) {
		DBG("is in flat zone\n");
		return 0;
	} else {
		DBG(" is not in flat zone\n");
		return 1;
	}
}

static void power_on_save(struct battery_info *di, int voltage)
{
	u8 buf;
	u8 save_soc;

	battery_read(di->rk818, NON_ACT_TIMER_CNT_REGL, &buf, 1);

	/*first power-on or power off time > 30min */
	if (_is_first_poweron(di)
	    || buf > 30) {
		_voltage_to_capacity(di, voltage);
		if (di->temp_soc < 20) {
			di->dod0_voltage = voltage;
			di->dod0_capacity = di->nac;
			di->dod0_status = 1;
			di->dod0 = di->temp_soc;
			di->dod0_level = 80;

			if (di->temp_soc <= 0)
				di->dod0_level = 100;
			else if (di->temp_soc < 5)
				di->dod0_level = 95;
			else if (di->temp_soc < 10)
				di->dod0_level = 90;
			save_soc = get_level(di);
			if (save_soc < di->dod0_level)
				save_soc = di->dod0_level;
			save_level(di, save_soc);
			DBG("UPDATE-FCC POWER ON : voltage = %d, capacity =%d ",
			    di->dod0_voltage, di->dod0_capacity);
		}
	}
}

static int _get_soc(struct battery_info *di)
{
	return di->remain_capacity * 100 / di->fcc;
}

unsigned int TempTab[] = {
	28704, 27417, 26197, 25039, 23940, 22897, 21906, 20964, 20070, 19219,
	18410, 17641, 16909, 16212, 15548, 14916, 14313, 13739, 13192, 12669,
	12171, 11696, 11242, 10809, 10395, 10000, 9622, 9261, 8916, 8585,
	8269, 7967, 7678, 7400, 7135, 6881, 6637, 6403, 6179, 5965,
	5759, 5561, 5372, 5189, 5015, 4847, 4686, 4531, 4382, 4239,
	4101, 3969, 3842, 3719, 3601, 3488, 3379, 3274, 3172, 3075,
	2981, 2890, 2803, 2719, 2638, 2559, 2484, 2411, 2341, 2273,
	2207, 2144, 2083, 2024, 1967, 1912, 1858, 1807, 1757, 1709,
	1662, 1617, 1574, 1532, 1491, 1451, 1413, 1376, 1340, 1305,
	1272, 1239, 1208, 1177, 1147, 1118, 1091, 1063, 1037, 1012,
	987, 963, 940, 917, 895, 874, 853, 833, 814, 795,
	776, 758, 741, 724, 708, 692, 676, 661, 646, 632,
	618,
};

unsigned int CheckTem(struct battery_info *di, unsigned int ResVal)
{
	unsigned int TempMin = 0;
	unsigned int TempMax = 120;
	unsigned int MidVal = 0;

	/*avoid little adc change, reduce calc */
	if (ResVal - di->pre_res_val < 30 || ResVal - di->pre_res_val > -30) {
		di->pre_res_val = ResVal;
		return di->temperature;
	}

	while (ResVal >= TempTab[TempMax]) {
		MidVal = (TempMax + TempMin) / 2;

		if ((MidVal == TempMin) || (MidVal == TempMax))
			break;

		if (ResVal >= TempTab[MidVal])
			TempMax = MidVal;
		else
			TempMin = MidVal;

		DBG("ResVal = %d, TempMax = %d,  TempMin = %d, MidVal = %d\n",
		    ResVal, TempMax, TempMin, MidVal);
	}
	di->pre_res_val = ResVal;
	return MidVal;
}

unsigned int VolToRes(struct battery_info *di)
{
	unsigned int ResVal;
	unsigned int AdcVal;
	u8 buf;
	int ret;

	ret = battery_read(di->rk818, TS1_ADC_REGL, &buf, 1);
	AdcVal = buf;
	ret = battery_read(di->rk818, TS1_ADC_REGH, &buf, 1);
	AdcVal |= buf << 8;

	if (di->ts1_adc_val != AdcVal) {
		ResVal = (AdcVal * 11 * 10000) / (4096 * 4);
		DBG("VolToRes AdcVal = %d, ResVal = %d\n", AdcVal, ResVal);
		di->ts1_adc_val = AdcVal;
		di->now_res_val = ResVal;
	}
	return di->now_res_val;
}

#define BAT_OVERHEAT_TEMP 95

unsigned int rk818_battery_health(struct battery_info *di)
{
	if (di->temperature > BAT_OVERHEAT_TEMP)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	return POWER_SUPPLY_HEALTH_GOOD;
}

static enum power_supply_property rk818_battery_props[] = {

	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

#define to_device_info(x) container_of((x), \
	struct battery_info, bat);

static int
rk818_battery_get_property(struct power_supply *psy,
			   enum power_supply_property psp,
			   union power_supply_propval *val)
{
	int ret = 0;
	struct battery_info *di = to_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_avg;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->voltage;
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		/*bat_test_mode = 1 means power
		supplyed by external power source */
		if (di->rk818->bat_test_mode == 1) {
			val->intval = 100;
			break;
		}

		if (di->real_soc < 0)
			di->real_soc = 0;
		if (di->real_soc > 100)
			di->real_soc = 100;
		val->intval = di->real_soc;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = rk818_battery_health(di);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->status;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temperature;
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static int
rk818_battery_set_property(struct power_supply *psy,
			   enum power_supply_property psp,
			   const union power_supply_propval *val)
{
	int ret = 0;
	/*struct battery_info *di = to_device_info(psy); */

	DBG("rk818_battery_set_property\n");

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		break;

	case POWER_SUPPLY_PROP_STATUS:
		break;
	case POWER_SUPPLY_PROP_TEMP:
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static enum power_supply_property rk818_battery_charger_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
};

#define to_usb_device_info(x) container_of((x), \
		struct battery_info, usb);

#define to_ac_device_info(x) container_of((x), \
		struct battery_info, ac);

static inline bool
rk818_is_online(struct battery_info *di, struct power_supply *psy)
{

	if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		return (((di->cable_type ==
			  POWER_SUPPLY_CHARGER_TYPE_USB_CDP) ||
			 (di->cable_type ==
			  POWER_SUPPLY_CHARGER_TYPE_USB_DCP)));

	else if (psy->type == POWER_SUPPLY_TYPE_USB)
		return ((di->cable_type ==
			 POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
			(di->cable_type ==
			 POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING));

	return false;
}

static int
rk818_battery_ac_get_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	int ret = 0;
	struct battery_info *di = to_ac_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = ((di->cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
					(di->cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING));

		else if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (di->cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_DCP ||
					di->cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_CDP);
		else
			val->intval = 0;

		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = rk818_is_online(di, psy);
		break;

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		val->intval = di->cable_type;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_MAINS;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int
rk818_battery_usb_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	int ret = 0;
	struct battery_info *di = to_usb_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = ((di->cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_SDP) ||
					(di->cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING));

		else if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (di->cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_DCP ||
					di->cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_CDP);
		else
			val->intval = 0;

		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = rk818_is_online(di, psy);
		break;

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		val->intval = di->cable_type;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_USB;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void set_charge_current(struct battery_info *di, int charge_current);

#if 0
static int rk818_enable_charging(struct battery_info *di, bool enable)
{
	u8 chr_reg;

	battery_read(di->rk818, CHRG_CTRL_REG1, &chr_reg, 1);

	DBG("chr_reg = %x\n", chr_reg);

	chr_reg &= ~(1 << 7);
	chr_reg |= ((enable) ? (1 << 7) : (0 << 7));

	DBG("chr_reg 1 = %x\n", chr_reg);
	battery_write(di->rk818, CHRG_CTRL_REG1, &chr_reg, 1);

	return 0;
}
#endif

static int
rk818_battery_usb_set_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *val)
{
	int ret = 0;
	struct battery_info *di = to_usb_device_info(psy);

	DBG("rk818_battery_usb_set_property\n");

	switch (psp) {

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		pr_info("set CABLE_TYPE = %x\n", val->intval);
		if (di->cable_type == val->intval)
			break;
		di->cable_type = val->intval;

		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		pr_info("set CONSTANT_CHARGE_CURRENT = %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		pr_info("set CONSTANT_CHARGE_VOLTAGE = %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		pr_info("set ENABLE_CHARGER = %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		pr_info("set ENABLE_CHARGING = %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_INLMT:
		pr_info("set POWER_SUPPLY_INLMT = %d\n", val->intval);
		if (val->intval == 100)
			set_charge_current(di, ILIM_80MA);
		else if (val->intval == 500)
			set_charge_current(di, ILIM_450MA);
		else if (val->intval == 1500)
			set_charge_current(di, ILIM_1500MA);
		break;
	case POWER_SUPPLY_PROP_CONTINUE_CHARGING:
		pr_info("set CONTINUE_CHARGING = %d\n", val->intval);
		break;
	default:
		pr_info("set property default\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int
rk818_battery_ac_set_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      const union power_supply_propval *val)
{
	int ret = 0;
	struct battery_info *di = to_ac_device_info(psy);

	pr_info("rk818_battery_ac_set_property\n");

	switch (psp) {

	case POWER_SUPPLY_PROP_CABLE_TYPE:
		pr_info("set CABLE_TYPE = %x\n", val->intval);
		if (di->cable_type == val->intval)
			break;
		di->cable_type = val->intval;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		pr_info("set CONSTANT_CHARGE_CURRENT = %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		pr_info("set CONSTANT_CHARGE_VOLTAGE = %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		pr_info("set ENABLE_CHARGER = %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		pr_info("set ENABLE_CHARGING = %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_INLMT:
		pr_info("set POWER_SUPPLY_INLMT = %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CONTINUE_CHARGING:
		pr_info("set CONTINUE_CHARGING = %d\n", val->intval);
		break;

	default:
		pr_info("set property default\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct power_supply_throttle rk818_dummy_throttle_states[] = {
	{
	 .throttle_action = PSY_THROTTLE_CC_LIMIT,
	 },
};

static char *rk818_supplied_to[] = {
	"battery",
};

static void battery_powersupply_init(struct battery_info *di)
{
	di->bat.name = "BATTERY";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = rk818_battery_props;
	di->bat.num_properties = ARRAY_SIZE(rk818_battery_props);
	di->bat.get_property = rk818_battery_get_property;
	di->bat.set_property = rk818_battery_set_property;

	di->ac.name = "AC";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties = rk818_battery_charger_props;
	di->ac.num_properties = ARRAY_SIZE(rk818_battery_charger_props);
	di->ac.get_property = rk818_battery_ac_get_property;
	di->ac.set_property = rk818_battery_ac_set_property;
	di->ac.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
	di->ac.supplied_to = rk818_supplied_to;
	di->ac.num_supplicants = ARRAY_SIZE(rk818_supplied_to);
	di->ac.throttle_states = rk818_dummy_throttle_states;
	di->ac.num_throttle_states = ARRAY_SIZE(rk818_dummy_throttle_states);

	di->usb.name = "USB";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = rk818_battery_charger_props;
	di->usb.num_properties = ARRAY_SIZE(rk818_battery_charger_props);
	di->usb.get_property = rk818_battery_usb_get_property;
	di->usb.set_property = rk818_battery_usb_set_property;
	di->usb.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
	di->usb.supplied_to = rk818_supplied_to;
	di->usb.num_supplicants = ARRAY_SIZE(rk818_supplied_to);
	di->usb.throttle_states = rk818_dummy_throttle_states;
	di->usb.num_throttle_states = ARRAY_SIZE(rk818_dummy_throttle_states);
}

static void res_mode_init(struct battery_info *di)
{
	u8 ggcon;
	u8 ggsts;

	battery_read(di->rk818, GGCON, &ggcon, 1);
	battery_read(di->rk818, GGSTS, &ggsts, 1);

	ggcon |= 0x01;
	ggsts &= ~0x01;
	ggsts |= 0x60;
	battery_write(di->rk818, GGCON, &ggcon, 1);
	battery_write(di->rk818, GGSTS, &ggsts, 1);
}

static void _capacity_init(struct battery_info *di, u32 capacity)
{
	u8 buf;
	u32 capacity_ma;

	di->update_k = 0;
	di->update_q = 0;
	di->voltage_old = 0;
	di->display_soc = 0;

	capacity_ma = capacity * 2390;	/* 2134;//36*14/900*4096/521*500; */
	DBG("%s WRITE GANCNT_CAL_REG  %d\n", __func__, capacity_ma);
	do {
		buf = (capacity_ma >> 24) & 0xff;
		battery_write(di->rk818, GASCNT_CAL_REG3, &buf, 1);
		buf = (capacity_ma >> 16) & 0xff;
		battery_write(di->rk818, GASCNT_CAL_REG2, &buf, 1);
		buf = (capacity_ma >> 8) & 0xff;
		battery_write(di->rk818, GASCNT_CAL_REG1, &buf, 1);
		buf = (capacity_ma & 0xff) | 0x01;
		battery_write(di->rk818, GASCNT_CAL_REG0, &buf, 1);

		DBG("\n %s ------------ read GANCNT_CAL_REG0 entry -------\n",
		    __func__);
		battery_read(di->rk818, GASCNT_CAL_REG0, &buf, 1);
		DBG("\n %s ------------ read GANCNT_CAL_REG0 ok -------\n",
		    __func__);
	} while (buf == 0);

	return;
}

static void _save_remain_capacity(struct battery_info *di, u32 capacity)
{
	u8 buf;
	u32 capacity_ma;

	if (capacity >= di->qmax)
		capacity = di->qmax;
	capacity_ma = capacity;
	DBG("%s WRITE GANCNT_CAL_REG  %d\n", __func__, capacity_ma);
	buf = (capacity_ma >> 24) & 0xff;
	battery_write(di->rk818, REMAIN_CAP_REG3, &buf, 1);
	buf = (capacity_ma >> 16) & 0xff;
	battery_write(di->rk818, REMAIN_CAP_REG2, &buf, 1);
	buf = (capacity_ma >> 8) & 0xff;
	battery_write(di->rk818, REMAIN_CAP_REG1, &buf, 1);
	buf = (capacity_ma & 0xff) | 0x01;
	battery_write(di->rk818, REMAIN_CAP_REG0, &buf, 1);

	return;
}

static int _get_remain_capacity(struct battery_info *di)
{
	int ret;
	int temp = 0;
	u8 buf;
	u32 capacity;

	ret = battery_read(di->rk818, REMAIN_CAP_REG3, &buf, 1);
	temp = buf << 24;
	ret = battery_read(di->rk818, REMAIN_CAP_REG2, &buf, 1);
	temp |= buf << 16;
	ret = battery_read(di->rk818, REMAIN_CAP_REG1, &buf, 1);
	temp |= buf << 8;
	ret = battery_read(di->rk818, REMAIN_CAP_REG0, &buf, 1);
	temp |= buf;

	capacity = temp;	/* 4096*900/14/36*500/521; */
	DBG("%s GASCNT_CAP_REG %d  capacity = %d\n", __func__, temp, capacity);

	return capacity;
}

static void _save_FCC_capacity(struct battery_info *di, u32 capacity)
{
	u8 buf;
	u32 capacity_ma;

	capacity_ma = capacity;
	DBG("%s WRITE NEW_FCC_REG  %d\n", __func__, capacity_ma);
	buf = (capacity_ma >> 24) & 0xff;
	battery_write(di->rk818, NEW_FCC_REG3, &buf, 1);
	buf = (capacity_ma >> 16) & 0xff;
	battery_write(di->rk818, NEW_FCC_REG2, &buf, 1);
	buf = (capacity_ma >> 8) & 0xff;
	battery_write(di->rk818, NEW_FCC_REG1, &buf, 1);
	buf = (capacity_ma & 0xff) | 0x01;
	battery_write(di->rk818, NEW_FCC_REG0, &buf, 1);

	return;
}

static int _get_FCC_capacity(struct battery_info *di)
{
	int ret;
	int temp = 0;
	u8 buf;
	u32 capacity;

	ret = battery_read(di->rk818, NEW_FCC_REG3, &buf, 1);
	temp = buf << 24;
	ret = battery_read(di->rk818, NEW_FCC_REG2, &buf, 1);
	temp |= buf << 16;
	ret = battery_read(di->rk818, NEW_FCC_REG1, &buf, 1);
	temp |= buf << 8;
	ret = battery_read(di->rk818, NEW_FCC_REG0, &buf, 1);
	temp |= buf;

	capacity = temp; /* 4096*900/14/36*500/521 */
	DBG("%s NEW_FCC_REG %d  capacity = %d\n", __func__, temp, capacity);

	return capacity;
}

static int _get_realtime_capacity(struct battery_info *di)
{
	int ret;
	int temp = 0;
	u8 buf;
	u32 capacity;

#if 1
	ret = battery_read(di->rk818, GASCNT3, &buf, 1);
	temp = buf << 24;
	ret = battery_read(di->rk818, GASCNT2, &buf, 1);
	temp |= buf << 16;
	ret = battery_read(di->rk818, GASCNT1, &buf, 1);
	temp |= buf << 8;
	ret = battery_read(di->rk818, GASCNT0, &buf, 1);
	temp |= buf;
#endif
	/* ret = battery_read(di->rk818, GASCNT_CAL_REG3, &buf, 4);
	 * temp = buf[0] << 24 | buf[1] << 24 | buf[2] << 24 |buf[3] ; */
	capacity = temp / 2390;	/* 4096*900/14/36*500/521; */
	/* capacity = temp/2134;///4096*900/14/36*500/521; */
	return capacity;
}

static void update_remaincapacity(struct battery_info *di, uint16_t voltage)
{
	/* int temp_soc; */
	int remain_capacity;
	_voltage_to_capacity(di, voltage);
	/* temp_soc = di->temp_soc; */
	remain_capacity =
	    di->temp_soc * 25 / 100 + di->remain_capacity * 75 / 100;
	_capacity_init(di, remain_capacity);	/* update remain capacity */

	DBG("%s new remaincapacity = %d\n", __func__, di->temp_nac);
	di->remain_capacity = _get_realtime_capacity(di);
	DBG(" resuem remain capacity =  %d\n", di->remain_capacity);
	DBG(" please check the relax capcacity and the remain_capacity\n");
}

static int soc_to_OCV(struct battery_info *di)
{
	int temp_soc;
	u32 *ocv_table;
	int ocv_size;
	int voltage_ocv;

	ocv_table = di->platform_data->battery_ocv;
	ocv_size = di->platform_data->ocv_size;
	temp_soc = _get_soc(di);

	if (temp_soc == 0) {
		voltage_ocv = ocv_table[0];
		return voltage_ocv;
	}
	if (temp_soc == 100) {
		voltage_ocv = ocv_table[ocv_size - 1];
		return voltage_ocv;
	}
	voltage_ocv = ocv_table[temp_soc / 5] + (ocv_table[temp_soc / 5 + 1] -
						 ocv_table[temp_soc / 5]) *
	    (temp_soc % 5) / 5;

	return voltage_ocv;
}

static int _copy_soc(struct battery_info *di, u8 save_soc)
{
	u8 soc;

	soc = save_soc;
	/* soc = 85; */
	battery_write(di->rk818, SOC_REG, &soc, 1);
	battery_read(di->rk818, SOC_REG, &soc, 1);

	return 0;
}

static int _rsoc_init(struct battery_info *di)
{
	int vol;
	u8 temp;
	u32 remain_capacity;
	u8 buf;

	vol = di->voltage_ocv;	/* _get_OCV_voltage(di); */
	DBG("OCV voltage = %d\n", di->voltage_ocv);
	if (_is_first_poweron(di)) {

		DBG(" %s this is first poweron\n", __func__);
		_voltage_to_capacity(di, di->voltage_ocv);
		di->real_soc = di->temp_soc;
		di->nac = di->temp_nac;
		DBG(" first poweron: SOC = %d, CAPACITY = %d\n", di->real_soc,
		    di->nac);

	} else {
		DBG(" %s this is  not not not first poweron\n", __func__);
		battery_read(di->rk818, SOC_REG, &temp, 1);
		DBG("======> SOC_REG = 0x%2x\n", temp);
		remain_capacity = _get_remain_capacity(di);
		if (remain_capacity >= di->qmax)
			remain_capacity = di->fcc + 50;
		battery_read(di->rk818, NON_ACT_TIMER_CNT_REGL, &buf, 1);
#if 1
		_voltage_to_capacity(di, di->voltage_ocv);

		/* if (temp == 0) */
		if (remain_capacity >= di->temp_nac * 120 / 100)
			remain_capacity = di->temp_nac * 110 / 100;

		if (buf > 30) {
			remain_capacity = di->temp_nac;
			DBG(
			"update remain capacity , power on ,the remain-capacity = %d\n"
			, remain_capacity);
		}
#endif
		di->real_soc = temp;
		/* if (remain_capacity >= (di->fcc*temp*12/10/100)) */

		di->nac = remain_capacity;
		if (di->nac <= 0)
			di->nac = 0;

		DBG("saved SOC_REG = 0x%8x\n", temp);
		DBG("saved remain_capacity = %d\n", remain_capacity);

	}
	return 0;
}

static u8 get_charge_status(struct battery_info *di)
{
	u8 status;
	u8 ret = 0;

	battery_read(di->rk818, SUP_STS_REG, &status, 1);
	DBG("%s----- SUP_STS_REG(0xA0) = 0x%02x", __func__, status);
	status &= (0x70);
	switch (status) {
	case CHARGE_OFF:
		ret = CHARGE_OFF;
		DBG("    CHARGE-OFF\n");
		break;

	case DEAD_CHARGE:
		ret = DEAD_CHARGE;
		DBG("  DEAD CHARGE\n");
		break;

	case TRICKLE_CHARGE:	/* (0x02 << 4) */
		ret = DEAD_CHARGE;
		DBG("  TRICKLE CHARGE...\n ");
		break;

	case CC_OR_CV:		/* (0x03 << 4) */
		ret = CC_OR_CV;
		DBG("   CC or CV ...\n");
		break;

	case CHARGE_FINISH:	/* (0x04 << 4) */
		ret = CHARGE_FINISH;
		DBG("  CHARGE FINISH...\n");
		break;

	case USB_OVER_VOL:	/* (0x05 << 4) */
		ret = USB_OVER_VOL;
		DBG("  USB OVER VOL...\n");
		break;

	case BAT_TMP_ERR:	/* (0x06 << 4) */
		ret = BAT_TMP_ERR;
		DBG("  BAT TMP ERROR...\n");
		break;

	case TIMER_ERR:	/* (0x07 << 4) */
		ret = TIMER_ERR;
		DBG(" TIMER ERROR...\n");
		break;

	case USB_EXIST:	/* (1 << 1)// usb is exists */
		ret = USB_EXIST;
		DBG("  USB EXIST ...\n");
		break;

	case USB_EFF:		/* (1 << 0)// usb is effective */
		ret = USB_EFF;
		DBG(" USB EFF...\n");
		break;

	default:
		return -EINVAL;
	}

	return ret;

}

static void set_charge_current(struct battery_info *di, int charge_current)
{
	u8 usb_ctrl_reg;
	/*      battery_read(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);
	 * DBG("-oooooooooo OOOOOOOcharge status  0x%x\n:", usb_ctrl_reg);
	 */

	battery_read(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);

	/* (VLIM_4400MV | ILIM_1200MA) |(0x01 << 7); */
	usb_ctrl_reg &= (~0x0f);
	usb_ctrl_reg |= (charge_current | (1 << 7));
	battery_write(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);

	battery_read(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);
	DBG("----charge status  0x%x\n:", usb_ctrl_reg);
}

static void rk818_battery_charger_init(struct battery_info *di)
{
	u8 chrg_ctrl_reg1, usb_ctrl_reg, chrg_ctrl_reg2;
	u8 sup_sts_reg, thermal_reg;

	DBG("%s  start\n", __func__);
	battery_read(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);
	battery_read(di->rk818, CHRG_CTRL_REG1, &chrg_ctrl_reg1, 1);
	battery_read(di->rk818, CHRG_CTRL_REG2, &chrg_ctrl_reg2, 1);
	battery_read(di->rk818, SUP_STS_REG, &sup_sts_reg, 1);

	battery_read(di->rk818, THERMAL_REG, &thermal_reg, 1);
	thermal_reg &= 0xe3;
	thermal_reg |= (TSD_TEMP_140 | HOTDIE_TEMP_85);
	battery_write(di->rk818, THERMAL_REG, &thermal_reg, 1);
	DBG("THERMAL_REG = 0x%2x\n ", thermal_reg);
	DBG("old usb_ctrl_reg = 0x%2x, CHRG_CTRL_REG1= 0x%2x\n ",
	usb_ctrl_reg, chrg_ctrl_reg1);
	usb_ctrl_reg &= (~0x0f);
#ifdef SPORT_USB_CHARGE
	usb_ctrl_reg |= (VLIM_4400MV | ILIM_450MA) | (0x01 << 7);
#else
	usb_ctrl_reg |= (VLIM_4400MV | ILIM_1000MA) | (0x01 << 7);
#endif
	chrg_ctrl_reg1 &= (0x00);
	chrg_ctrl_reg1 |= (0x01 << 7) | (CHRG_VOL4200 | CHRG_CUR1400mA);

	chrg_ctrl_reg2 &= ~(0xc0);
	chrg_ctrl_reg2 |= 0xc0;	/* 250 charge finish */

	sup_sts_reg &= ~(0x01 << 3);
	sup_sts_reg |= (0x01 << 2);

	battery_write(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);

	battery_write(di->rk818, CHRG_CTRL_REG1, &chrg_ctrl_reg1, 1);
	battery_write(di->rk818, CHRG_CTRL_REG2, &chrg_ctrl_reg2, 1);
	battery_write(di->rk818, SUP_STS_REG, &sup_sts_reg, 1);

	battery_read(di->rk818, CHRG_CTRL_REG1, &chrg_ctrl_reg1, 1);
	/* battery_read(di->rk818, CHRG_CTRL_REG2, &chrg_ctrl_reg2, 1); */
	battery_read(di->rk818, SUP_STS_REG, &sup_sts_reg, 1);
	battery_read(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);
	DBG(
	"new  usb_ctrl_reg = 0x%2x, CHRG_CTRL_REG1= 0x%2x, SUP_STS_REG= 0x%2x\n"
	, usb_ctrl_reg, chrg_ctrl_reg1, sup_sts_reg);

	/*ADC_TS1_EN*/
	rk818_set_bits(di->rk818, ADC_CTRL_REG, 1 << 5, 1 << 5);
	DBG("%s  end\n", __func__);
}

void charge_disable_open_otg(struct battery_info *di, int value)
{
	/* u8 chrg_ctrl_reg1, dcdc_en_reg; */
	if (value == 1) {
		DBG("1   ---- charge disable\n");
		/*  enable OTG */
		rk818_set_bits(di->rk818, CHRG_CTRL_REG1, 1 << 7, 0 << 7);
		rk818_set_bits(di->rk818, 0x23, 1 << 7, 1 << 7);
	}
	if (value == 0) {
		DBG("1   ---- charge enable\n");
		/* disable OTG */
		rk818_set_bits(di->rk818, 0x23, 1 << 7, 0 << 7);
		rk818_set_bits(di->rk818, CHRG_CTRL_REG1, 1 << 7, 1 << 7);
	}
}

static void low_waring_init(struct battery_info *di)
{
	u8 vb_mon_reg;		/* chrg_ctrl_reg2 */
	u8 vb_mon_reg_init;

	battery_read(di->rk818, VB_MOD_REG, &vb_mon_reg, 1);

	/* 2.8v, 3.5v, interrupt */
	vb_mon_reg_init = (((vb_mon_reg | (1 << 4)) & (~0x07)) | 0x06);

	battery_write(di->rk818, VB_MOD_REG, &vb_mon_reg_init, 1);
}

static void fg_init(struct battery_info *di)
{
	int now_current;

#if 1
	u8 adc_value;
	/* adc_value = 0xf7; */
	battery_read(di->rk818, 0xAD, &adc_value, 1);
	DBG("%s start   %x\n", __func__, adc_value);
	adc_value = 0x30;
	battery_write(di->rk818, 0xAD, &adc_value, 1);
	battery_read(di->rk818, 0xAD, &adc_value, 1);
	DBG("%s start   %x\n", __func__, adc_value);
#endif
	_gauge_enable(di);
	_get_voltage_offset_value(di);	/* get the volatege offset */
	/* _autosleep_enable(di); */
	rk818_battery_charger_init(di);
	_set_relax_thres(di);
	/* get the current offset , the value write to the CAL_OFFSET */
	di->current_offset = _get_ioffset(di);
	_set_cal_offset(di, di->current_offset + 51);

	di->voltage = rk818_battery_voltage(di);
	di->voltage_ocv = _get_OCV_voltage(di);

	_rsoc_init(di);
	_capacity_init(di, di->nac);
	res_mode_init(di);
	di->remain_capacity = _get_realtime_capacity(di);
	do_gettimeofday(&di->soc_timer);
	di->change_timer = di->soc_timer;
	now_current = _get_average_current(di);

	low_waring_init(di);
	restart_relax(di);
	power_on_save(di, di->voltage_ocv);

	/*dump_gauge_register(di);
	 * dump_charger_register(di); */

	/* di->bat_res = (di->voltage_ocv - di->voltage) * 1000 / now_current;
	DBG(" the res is : %d(mo)\n", di->bat_res); */
	DBG("%s() :\n"
	    "nac = %d , remain_capacity = %d\n"
	    " OCV_voltage = %d, voltage = %d\n"
	    "SOC=%d, fcc=%d\n",
	    __func__,
	    di->nac, di->remain_capacity,
	    di->voltage_ocv, di->voltage, di->real_soc, di->fcc);
}

#if 1
/* int R_soc, D_soc, r_soc, zq, k, Q_err, Q_ocv; */
static void zero_get_soc(struct battery_info *di)
{
	int ocv_voltage, check_voltage;
	int temp_soc = -1, real_soc;
	int currentold, currentnow, voltage;
	int i;
	int voltage_k;
	int count_num = 0;

	DBG("%s\n", __func__);
	{
		do {
			currentold = _get_average_current(di);
			_get_cal_offset(di);
			_get_ioffset(di);
			msleep(100);
			currentnow = _get_average_current(di);
			count_num++;
		} while ((currentold == currentnow) && (count_num < 11));

		voltage = 0;
		for (i = 0; i < 10; i++)
			voltage += rk818_battery_voltage(di);
		voltage /= 10;

		if (di->voltage_old == 0)
			di->voltage_old = voltage;
		voltage_k = voltage;
		voltage = (di->voltage_old * 2 + 8 * voltage) / 10;
		di->voltage_old = voltage;
		/*DBG("Zero: voltage = %d\n", voltage); */

		currentnow = _get_average_current(di);

		ocv_voltage = 3400 + abs32_int(currentnow)
		* 200 / 1000;
#if 1
		check_voltage = voltage + abs32_int(currentnow)
		* (200 - 65) / 1000;
		_voltage_to_capacity(di, check_voltage);

		di->update_q = di->remain_capacity - di->temp_nac;

#endif
		_voltage_to_capacity(di, ocv_voltage);
		if (di->display_soc == 0)
			di->display_soc = di->real_soc * 1000;

		real_soc = di->display_soc;

		if (di->remain_capacity > di->temp_nac + di->update_q) {
#if 1
			if (di->update_k == 0 || di->update_k >= 10) {
				if (di->update_k == 0) {
					di->line_q = di->temp_nac
					+ di->update_q;
					temp_soc = (di->remain_capacity
					- di->line_q)
					* 1000 / di->fcc;
					di->line_k = (real_soc
					+ temp_soc / 2) / temp_soc;

				} else {
					temp_soc = ((di->remain_capacity
					- di->line_q) * 1000
					+ di->fcc / 2) / di->fcc;
					real_soc = (di->line_k * temp_soc);
					di->display_soc = real_soc;
					/* if (real_soc != di->real_soc) */
					if ((real_soc + 500) / 1000 <
					    di->real_soc)
						di->real_soc--;
					_voltage_to_capacity(di, ocv_voltage);
					di->line_q = di->temp_nac
					+ di->update_q;
					temp_soc = ((di->remain_capacity
					- di->line_q)
					* 1000 + di->fcc / 2) / di->fcc;
					di->line_k = (di->display_soc +
					temp_soc / 2) / temp_soc;


				}
				di->update_k = 0;

			}
#endif

			di->update_k++;
			if (di->update_k == 1 || di->update_k != 10) {
				temp_soc = ((di->remain_capacity - di->line_q)
				* 1000 + di->fcc / 2) / di->fcc;
				di->display_soc = di->line_k * temp_soc;
				pr_info("display-soc = %d, real-soc = %d\n",
				       di->display_soc, di->real_soc);
				if ((di->display_soc + 500) / 1000 <
				    di->real_soc)
					di->real_soc--;
			}
		} else {
			/*DBG("three..\n"); */
			di->update_k++;
			if (di->update_k > 10) {
				di->update_k = 0;
				di->real_soc--;
			}
		}

		DBG("\n\n Zero : update_k = %d\n", di->update_k);
		DBG(
		" ZERO : remain_capacity = %d , nac = %d, update_q = %d\n ",
		di->remain_capacity, di->line_q, di->update_q);
		DBG(
		" ZERO :  Warnning_voltage = %d, line_k = %d, temp_soc = %d real_soc = %d\n",
		di->warnning_voltage, di->line_k, temp_soc, di->real_soc);
		DBG("+++++++zero mode++++++display soc+++++++++++\n");
	}
}

#endif

static void voltage_to_soc_discharge_smooth(struct battery_info *di)
{
	int voltage;
	int now_current, soc_time = -1;
	DBG("-----%s-----\n", __func__);
	voltage = di->voltage;
	if (di->voltage < 3880) {	/* di->warnning_voltage) */
		zero_get_soc(di);
	} else {
		DBG("di->temp_soc  OR di->real_soc\n");
		di->update_k = 0;
		di->update_q = 0;
		di->voltage_old = 0;
		di->display_soc = 0;
		di->temp_soc = _get_soc(di);
		do_gettimeofday(&di->soc_timer);
		if (di->temp_soc == di->real_soc) {

			DBG("di->temp_soc  == di->real_soc\n");
		} else {
			if (di->temp_soc > di->real_soc) {
				DBG("di->temp_soc > di->real_soc\n");
				now_current = _get_average_current(di);
				soc_time = di->fcc * 3600
				/ 100 / (abs_int(now_current));

				di->vol_smooth_time++;

				if (di->vol_smooth_time > soc_time * 3) {
					di->real_soc--;
					di->vol_smooth_time = 0;
				}
				DBG(
				"line = %d, charge temp_soc > real_soc di->temp_soc = %d, di->real_soc = %d\n"
				, __LINE__, di->temp_soc, di->real_soc);
				return;
			} else {

				DBG("di->temp_soc < di->real_soc\n");
				DBG("5\n");
#if 1
				if (di->real_soc == (di->temp_soc + 1)) {
					di->change_timer = di->soc_timer;
					di->real_soc = di->temp_soc;
				} else {
					DBG("2\n");
					now_current = _get_average_current(di);
					soc_time = di->fcc * 3600
					/ 100 / (abs_int(now_current));

					di->vol_smooth_time++;

					if (di->vol_smooth_time >
						soc_time / 3) {
						di->real_soc--;
						di->vol_smooth_time = 0;
					}
					DBG(
					"di->vol_smooth_time = %d ,soc_time = %d\n",
					di->vol_smooth_time, soc_time);
				}
#endif
			}
			DBG(
			"line = %d, charge temp_soc > real_soc di->temp_soc = %d, di->real_soc = %d\n",
			__LINE__, di->temp_soc, di->real_soc);
		}

		return;
	}

	DBG("line = %d, di->temp_soc = %d, di->real_soc = %d\n", __LINE__,
	    di->temp_soc, di->real_soc);
	DBG("%s, di->vol_smooth_time = %d, soc_time = %d\n", __func__,
	    di->vol_smooth_time, soc_time);
}

static void voltage_to_soc_charge_discharge_smooth(struct battery_info *di)
{
	DBG(">>>>>>>>>>>>>%s  start << << << << << << << << << <<\n",
	    __func__);
	voltage_to_soc_discharge_smooth(di);
	DBG(">>>>>>>>>>>>>%s  end << << << << << << << << << <<\n", __func__);
}

static void voltage_to_soc_charge_smooth(struct battery_info *di)
{
	/* int voltage, vol_soc; */
	int now_current, soc_time;

	DBG("-----%s-----\n", __func__);
	/* voltage = di->voltage; */
	di->temp_soc = _get_soc(di);
	now_current = _get_average_current(di);
	di->update_k = 0;
	di->update_q = 0;
	di->voltage_old = 0;
	di->display_soc = 0;

	if (di->temp_soc >= 85) {
		if (di->real_soc >= 85) {
			di->temp_soc = di->real_soc +
			((di->remain_capacity - di->fcc * (di->real_soc) / 100)
			* 100 / di->fcc) * 4 / 10;
			DBG(
			"1 the value  capactiy  = %d, di->fcc*realsoc = %d\n",
			(di->remain_capacity - di->fcc * (di->real_soc) / 100),
			di->fcc * (di->real_soc));
			DBG("1 the SOC %d\n",
			((di->remain_capacity - di->fcc *
			(di->real_soc) / 100) * 100 / di->fcc) * 4 / 10);
			DBG(
			"1  real  SOC  %d  tmpSOC = %d  reamin to soc = %d\n",
			di->real_soc, di->temp_soc, _get_soc(di));
		} else {
			di->temp_soc = 85 + ((di->remain_capacity -
			di->fcc * 85 / 100) * 100 / di->fcc) * 4 / 10;
			DBG(
			"2 real  SOC  %d  tmpSOC = %d  reamin to soc = %d\n",
			di->real_soc, di->temp_soc, _get_soc(di));
		}

	}
	DBG("charge status SUP_STS_REG = 0x%x\n", get_charge_status(di));
	do_gettimeofday(&di->soc_timer);
	if ((di->temp_soc != di->real_soc) && (now_current != 0)) {
		DBG("di->tempsoc = %d  realsoce = %d\n", di->temp_soc,
		    di->real_soc);
		if (di->temp_soc < di->real_soc + 1) {
			DBG("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
			soc_time = di->fcc * 3600 / 100
			/ (abs_int(now_current));

			di->charge_smooth_time++;

			if (di->charge_smooth_time > soc_time * 2) {
				di->real_soc++;
				di->charge_smooth_time = 0;
			}
			di->charge_smooth_status = true;

			DBG(
			"di->charge_smooth_time = %d, soc_time = %d soc_time/3 = %d\n ",
			di->charge_smooth_time, soc_time, soc_time * 3);
			DBG(
			"di->charge_smooth_status = %d, charge temp_soc > real_soc di->temp_soc = %d, di->real_soc = %d\n",
			di->charge_smooth_status, di->temp_soc, di->real_soc);
			return;
		}

		if (di->temp_soc > di->real_soc + 1) {

			/* now_current = _get_average_current(di); */
			soc_time = di->fcc * 3600 / 100
			/ (abs_int(now_current));

			di->charge_smooth_time++;

			if (di->charge_smooth_time > soc_time / 3) {
				di->real_soc++;
				di->charge_smooth_time = 0;
			}
			di->charge_smooth_status = true;

			DBG(
			"di->charge_smooth_time = %d, soc_time = %d soc_time/3 = %d\n",
			di->charge_smooth_time, soc_time, soc_time / 3);
			DBG(
			"di->charge_smooth_status = %d, charge temp_soc > real_soc di->temp_soc = %d, di->real_soc = %d\n",
			di->charge_smooth_status,
			di->temp_soc, di->real_soc);
			return;
		} else if (di->temp_soc == di->real_soc + 1) {
			if (di->charge_smooth_status) {

				now_current = _get_average_current(di);
				soc_time = di->fcc * 3600 / 100 /
				(abs_int(now_current));

				di->charge_smooth_time++;

				if (di->charge_smooth_time > soc_time / 3) {
					di->real_soc = di->temp_soc;
					di->charge_smooth_time = 0;
					di->charge_smooth_status = false;
				}
				DBG(
				"di->charge_smooth_time = %d, soc_time = %d soc_time/3 = %d\n",
				di->charge_smooth_time, soc_time, soc_time / 3);
			} else {
				di->real_soc = di->temp_soc;
				di->charge_smooth_status = false;

			}
		}
		DBG(
		"di->charge_smooth_status = %d, charge temp_soc > real_soc di->temp_soc = %d, di->real_soc = %d\n",
		di->charge_smooth_status, di->temp_soc, di->real_soc);

	}
	return;
}

static void rk818_battery_display_smooth(struct battery_info *di)
{
	int status;
	u8 charge_status;

	status = di->status;
	if ((status == POWER_SUPPLY_STATUS_CHARGING)
	    || (di->status == POWER_SUPPLY_STATUS_FULL)) {
		/* DBG("charging smooth ...\n"); */
		if (1) {
			/* DBG("   BATTERY NOT RELAX MODE\n"); */
			DBG("di->remain_capacity = %d, di->fcc  = %d\n",
			di->remain_capacity, di->fcc);
			di->temp_soc = _get_soc(di);
			charge_status = get_charge_status(di);
			if (di->temp_soc >= 100) {
				di->temp_soc = 100;
				/* di->status = POWER_SUPPLY_STATUS_FULL; */
			}

			do_gettimeofday(&di->soc_timer);

#if 1
			if ((di->current_avg < -10)
			    && (charge_status != CHARGE_FINISH))
				voltage_to_soc_charge_discharge_smooth(di);
			else
#endif
				voltage_to_soc_charge_smooth(di);
			if ((charge_status == CHARGE_FINISH)
			    && (di->dod0_status == 1)) {
				if (get_level(di) >= di->dod0_level) {
					di->fcc =
					    (di->remain_capacity -
					    di->dod0_capacity) * 100
					    / (100 - di->dod0);
					DBG("NEW FCC: %d\n", di->fcc);
					_capacity_init(di, di->fcc);
					_save_FCC_capacity(di, di->fcc);
					DBG(
					"update FCC: dod0_capacity = %d ,voltage = %d, level = %d\n",
					di->dod0_capacity,
					di->dod0_voltage,
					di->dod0_level);
				}
				di->dod0_status = 0;
			}
			/* if ((charge_status == CHARGE_FINISH)
			 * && (di->real_soc == 100))
			 * di->status = POWER_SUPPLY_STATUS_FULL; */
			if ((charge_status == CHARGE_FINISH)
			    && (di->real_soc < 100)) {
				DBG("CHARGE_FINISH  di->real_soc < 100\n ");
				if ((di->soc_counter < 100)) {
					di->soc_counter++;
				} else {
					di->soc_counter = 0;
					if (di->real_soc < 100) {
						di->real_soc++;
						/* _save_rsoc_nac(di); */
					}
				}
			}
		}
		if (di->real_soc <= 0)
			di->real_soc = 0;
		if (di->real_soc >= 100) {
			di->real_soc = 100;
			di->status = POWER_SUPPLY_STATUS_FULL;
		}

	}
	if (status == POWER_SUPPLY_STATUS_DISCHARGING) {
		di->update_cur_offset = 0;
		voltage_to_soc_discharge_smooth(di);
		if (di->real_soc == 1) {
			di->time2empty++;
			if (di->time2empty == 300)
				di->real_soc = 0;
		} else {
			di->time2empty = 0;
		}

		if (di->real_soc <= 0)
			di->real_soc = 0;
		if (di->real_soc >= 100)
			di->real_soc = 100;
	}
	/* DBG("%s   exit\n", __func__); */
}

static void check_resume(struct battery_info *di)
{
	unsigned long sleep_soc;
	unsigned long sleep_time;
	int relax_voltage;
	u8 charge_status;
	DBG("resume------------------------------------checkstart\n");
	sleep_time = get_seconds() - di->suspend_test_time;
	di->resume = false;

	if (di->sleep_status == POWER_SUPPLY_STATUS_DISCHARGING) {
		DBG("POWER_SUPPLY_STATUS_DISCHARGING\n");

		sleep_soc = 20 * sleep_time * 100 / 3600 / di->fcc;
		if (sleep_soc > 3) {
			di->temp_soc = _get_soc(di);
			if (di->temp_soc < di->real_soc)
				di->real_soc = di->temp_soc;
		}

		relax_voltage = get_relax_voltage(di);
		if (relax_voltage > di->voltage) {
			if (is_not_flatzone(di, relax_voltage))
				update_remaincapacity(di,
						      relax_voltage);
		}
		DBG(
		"POWER_SUPPLY_STATUS_DISCHARGING: sleep_soc = %ld, di->temp_soc  = %d, di->real_soc = %d\n ",
		sleep_soc, di->temp_soc, di->real_soc);
	}
	if (di->sleep_status == POWER_SUPPLY_STATUS_CHARGING) {
		DBG("POWER_SUPPLY_STATUS_CHARGING\n");
		if (di->sleep_charge_current >= 0) {

			di->temp_soc = _get_soc(di);
			charge_status = get_charge_status(di);

			DBG("cable_type = %d\n", di->cable_type);
			if (((di->sleep_charge_current < 800)
			     && (di->cable_type ==
				 POWER_SUPPLY_CHARGER_TYPE_USB_DCP
				 || di->cable_type ==
				 POWER_SUPPLY_CHARGER_TYPE_USB_CDP))
			    || (charge_status == CHARGE_FINISH)) {
				DBG(
				"sleep : ac online charge current < 1000\n"
				);
				if (sleep_time > 0) {
					di->count_sleep_time +=
					    sleep_time;
					sleep_soc =
					    1000 *
					    di->count_sleep_time * 100 /
					    3600 / di->fcc;
					if (sleep_soc > 0)
						di->count_sleep_time =
						    0;
					di->real_soc += sleep_soc;
					if (di->real_soc > 100)
						di->real_soc = 100;
				}
			} else {

				DBG("sleep : charge current\n");
				if (di->sleep_soc + 15 < di->temp_soc)
					di->real_soc +=
					    (di->temp_soc -
					     di->sleep_soc) * 3 / 2;
				else
					di->real_soc +=
					    (di->temp_soc -
					     di->sleep_soc);
			}

			DBG(
			"POWER_SUPPLY_STATUS_CHARGING:di->temp_soc  = %d, di->real_soc = %d, sleep_time = %ld\n",
			di->temp_soc, di->real_soc, sleep_time);
		}
	}
	if (di->real_soc <= 0)
		di->real_soc = 0;
	if (di->real_soc >= 100)
		di->real_soc = 100;
	DBG("resume------------------------------------checkend\n");
}

static void rk818_battery_update_status(struct battery_info *di)
{
	di->voltage = rk818_battery_voltage(di);
	di->current_avg = _get_average_current(di);
	di->remain_capacity = _get_realtime_capacity(di);
	di->temperature = CheckTem(di, VolToRes(di));
	DBG(" xxx  = %d\n", di->remain_capacity);

	if (di->resume)
		check_resume(di);

	rk818_battery_display_smooth(di);
	DBG("###################read#########################\n");
	DBG("%s\n"
	    "relax-voltage = %d, voltage = %d, current-avg = %d\n"
	    "fcc = %d , remain_capacity = %d, temperature = %d\n"
	    "diplay_soc = %d, cpapacity_soc = %d\n"
	    "charging: cable_type = %d\n",
	    __func__,
	    get_relax_voltage(di),
	    di->voltage, di->current_avg,
	    di->fcc, di->remain_capacity,
	    di->temperature, di->real_soc, _get_soc(di), di->cable_type);
	DBG("###############################################\n");
}

static void battery_poweron_status(struct battery_info *di)
{
#ifndef SPORT_USB_CHARGE
	u8 buf;
#endif

#ifdef SPORT_USB_CHARGE
	DBG(" CHARGE:  SPORT_USB_CHARGE\n");
	set_charge_current(di, ILIM_450MA);
#else
	battery_read(di->rk818, VB_MOD_REG, &buf, 1);

	DBG(" CHARGE:  NOT SPORT_USB_CHARGE\n");
	if (buf & PLUG_IN_STS) {
		/*di->ac_online = 1;
		 * di->usb_online = 0; */
		di->status = POWER_SUPPLY_STATUS_CHARGING;
		if (di->real_soc == 100)
			di->status = POWER_SUPPLY_STATUS_FULL;
	} else {
		di->status = POWER_SUPPLY_STATUS_DISCHARGING;
		/*di->ac_online = 0;
		 * di->usb_online = 0; */
	}
#endif
}

static void check_battery_status(struct battery_info *di)
{

	u8 buf;
	int ret;
	ret = battery_read(di->rk818, VB_MOD_REG, &buf, 1);
	/* int vbus_status =  dwc_vbus_status();
	 * u8 usb_ctrl_reg;// chrg_ctrl_reg2; */
#ifdef SPORT_USB_CHARGE
	DBG(" CHARGE:  SPORT_USB_CHARGE\n");
	if (strstr(saved_command_line, "charger")) {
		if ((buf & PLUG_IN_STS) == 0) {
			di->status = POWER_SUPPLY_STATUS_DISCHARGING;
			/*di->ac_online = 0;
			 * di->usb_online = 0; */
		}

	} else {
		if (buf & PLUG_IN_STS) {
			di->status = POWER_SUPPLY_STATUS_CHARGING;
			if (di->real_soc == 100)
				di->status = POWER_SUPPLY_STATUS_FULL;
		} else {
			di->status = POWER_SUPPLY_STATUS_DISCHARGING;
			/*di->ac_online = 0;
			 * di->usb_online = 0; */
		}
	}
#else
	DBG(" CHARGE:  NOT SPORT_USB_CHARGE\n");
	if (buf & PLUG_IN_STS) {
		/*di->ac_online = 1;
		 * di->usb_online = 0; */
		di->status = POWER_SUPPLY_STATUS_CHARGING;
		if (di->real_soc == 100)
			di->status = POWER_SUPPLY_STATUS_FULL;
	} else {
		di->status = POWER_SUPPLY_STATUS_DISCHARGING;
		/*di->ac_online = 0;
		 * di->usb_online = 0; */
	}
#endif
}

static void rk818_battery_work(struct work_struct *work)
{
	u8 usb_ctrl_reg;
	u8 buf;

	struct battery_info *di = container_of(work,
					       struct battery_info,
					       battery_monitor_work.work);

	di->work_on = 1;
	if (1)
		flatzone_voltage_init(di);
	/*DBG("\n-----------  flatzone_voltage_init() ok --------------\n"); */
	battery_read(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);
	DBG(" ****************   usb_ctrl_reg = 0x%x\n", usb_ctrl_reg);
	_get_voltage_offset_value(di);	/* get the volatege offset */

	check_battery_status(di);
	battery_read(di->rk818, 0x00, &buf, 1);
	DBG("RTC  = 0x%2x\n ", buf);

	/*battery_read(di->rk818, VB_MOD_REG, &buf, 1);
	 * DBG("VB_MOD_REG  = %2x, the value is %2x\n ", VB_MOD_REG, buf);
	 * battery_read(di->rk818, SUP_STS_REG, &buf, 1);
	 * DBG("SUP_STS_REG  = %2x, the value is %2x\n ", SUP_STS_REG, buf); */

	rk818_battery_update_status(di);
	DBG("\n-----------  rk818_battery_update_status() ok --------------\n");

	if ((di->cable_type == POWER_SUPPLY_CHARGER_TYPE_NONE)
	    && (di->remain_capacity >= di->fcc + 50)) {
		_capacity_init(di, di->fcc + 30);
		di->remain_capacity = _get_realtime_capacity(di);
	}

	DBG("\n =====================> copy-soc  =  %d", di->real_soc);
	_copy_soc(di, di->real_soc);
	DBG("\n =====================> _save_remain_capacity  =  %d",
	    di->remain_capacity);
	_save_remain_capacity(di, di->remain_capacity);
	DBG("\n-----------  _save_remain_capacity() ok --------------\n");

	/* _is_relax_mode(di); */

	DBG("soc to OCV %d\n", soc_to_OCV(di));
	_get_cal_offset(di);
	_get_ioffset(di);

	/* update_resmode(di); */
	power_supply_changed(&di->bat);
	power_supply_changed(&di->usb);
	power_supply_changed(&di->ac);
	queue_delayed_work(di->wq, &di->battery_monitor_work,
			   msecs_to_jiffies(TIMER_MS_COUNTS * 5));

}

static void rk818_battery_charge_check_work(struct work_struct *work)
{
	struct battery_info *di = container_of(work,
					       struct battery_info,
					       charge_check_work.work);

	pr_info("rk818_battery_charge_check_workxxxxx\n");
	charge_disable_open_otg(di, di->charge_otg);
}

static BLOCKING_NOTIFIER_HEAD(battery_chain_head);

int register_battery_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&battery_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_battery_notifier);

int unregister_battery_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&battery_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_battery_notifier);

int battery_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&battery_chain_head, val, NULL)
		== NOTIFY_BAD) ? -EINVAL : 0;
}
EXPORT_SYMBOL_GPL(battery_notifier_call_chain);

#if 0
static void poweron_lowerpoer_handle(struct battery_info *di)
{

#ifdef CONFIG_LOGO_LOWERPOWER_WARNING
	if ((di->real_soc <= 2)
	    && (di->status == POWER_SUPPLY_STATUS_DISCHARGING)) {
		mdelay(1500);
		/* kernel_power_off(); */
	}
#endif
}

static int
battery_notifier_call(struct notifier_block *nb,
		      unsigned long event, void *data)
{
	struct battery_info *di =
	    container_of(nb, struct battery_info, battery_nb);

	switch (event) {
	case 0:
		DBG(" CHARGE enable\n");
		di->charge_otg = 0;
		queue_delayed_work(di->wq, &di->charge_check_work,
				   msecs_to_jiffies(50));
		break;

	case 1:
		di->charge_otg = 1;
		queue_delayed_work(di->wq, &di->charge_check_work,
				   msecs_to_jiffies(50));

		DBG("charge disable OTG enable\n");
		break;

	case 2:
		poweron_lowerpoer_handle(di);
		break;

	default:
		return NOTIFY_OK;
	}
	return NOTIFY_OK;
}
#endif

int count = 0;
static void disable_vbat_low_irq(struct battery_info *di)
{
	/* mask vbat low */
	rk818_set_bits(di->rk818, 0x4d, (0x1 << 1), (0x1 << 1));
	/*rk818_set_bits(di->rk818,0x4c,(0x1 << 1),(0x1 << 1));*/
}

static void enable_vbat_low_irq(struct battery_info *di)
{
	/* clr vbat low interrupt */
	rk818_set_bits(di->rk818, 0x4c, (0x1 << 1), (0x1 << 1));
	/* mask vbat low */
	rk818_set_bits(di->rk818, 0x4d, (0x1 << 1), (0x0 << 1));
}

#ifdef CONFIG_OF
static int rk818_battery_parse_dt(struct rk818 *rk818)
{
	struct device_node *regs, *rk818_pmic_np;
	struct battery_platform_data *data;
	struct cell_config *cell_cfg;
	struct ocv_config *ocv_cfg;
	struct property *prop;
	u32 out_value;
	int i, length, ret;

	rk818_pmic_np = of_node_get(rk818->dev->of_node);
	if (!rk818_pmic_np) {
		pr_info("could not find pmic sub-node\n");
		return -EINVAL;
	}

	regs = of_find_node_by_name(rk818_pmic_np, "battery");
	if (!regs) {
		pr_info("could not find battery sub-node\n");
		return -EINVAL;
	}

	data = devm_kzalloc(rk818->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	memset(data, 0, sizeof(*data));

	cell_cfg = devm_kzalloc(rk818->dev, sizeof(*cell_cfg), GFP_KERNEL);
	if (!cell_cfg)
		return -ENOMEM;

	ocv_cfg = devm_kzalloc(rk818->dev, sizeof(*ocv_cfg), GFP_KERNEL);
	if (!ocv_cfg)
		return -ENOMEM;

	/* get the ocv_table */
	prop = of_find_property(regs, "ocv_table", &length);
	if (!prop)
		return -EINVAL;
	data->ocv_size = length / sizeof(u32);
	/* read brightness levels from DT property */
	if (data->ocv_size > 0) {
		size_t size = sizeof(*data->battery_ocv) * data->ocv_size;
		data->battery_ocv = devm_kzalloc(rk818->dev, size, GFP_KERNEL);
		if (!data->battery_ocv)
			return -ENOMEM;
		ret =
		    of_property_read_u32_array(regs, "ocv_table",
					       data->battery_ocv,
					       data->ocv_size);
		DBG("the battery OCV TABLE size : %d\n", data->ocv_size);
		DBG("the battery OCV TABLE : ");
		for (i = 0; i < data->ocv_size; i++)
			DBG("%d ", data->battery_ocv[i]);
		DBG("\n");
		if (ret < 0)
			return ret;
	}

	DBG("\n--------- the battery OCV TABLE dump end ----------\n");
	ret = of_property_read_u32(regs, "max_charge_currentmA", &out_value);
	if (ret < 0)
		return ret;
	data->max_charger_currentmA = out_value;
	ret = of_property_read_u32(regs, "max_charge_voltagemV", &out_value);
	if (ret < 0)
		return ret;
	data->max_charger_voltagemV = out_value;
	ret = of_property_read_u32(regs, "design_capacity", &out_value);
	if (ret < 0)
		return ret;
	cell_cfg->design_capacity = out_value;
	ret = of_property_read_u32(regs, "design_qmax", &out_value);
	if (ret < 0)
		return ret;
	cell_cfg->design_qmax = out_value;
	ret = of_property_read_u32(regs, "sleep_enter_current", &out_value);
	if (ret < 0)
		return ret;
	ocv_cfg->sleep_enter_current = out_value;
	/* cell->config->ocv->sleep_enter_current */
	ret = of_property_read_u32(regs, "sleep_exit_current", &out_value);
	if (ret < 0)
		return ret;
	ocv_cfg->sleep_exit_current = out_value;

	cell_cfg->ocv = ocv_cfg;
	data->cell_cfg = cell_cfg;
	rk818->battery_data = data;

	/* fixed battery capacity */
	if (of_find_property(regs, "battery-test-mode", NULL))
		rk818->bat_test_mode = 1;

	DBG("max_charge_currentmA :%d\n", data->max_charger_currentmA);
	DBG("max_charge_voltagemV :%d\n", data->max_charger_voltagemV);
	DBG("design_capacity :%d\n", cell_cfg->design_capacity);
	DBG("design_qmax :%d\n", cell_cfg->design_qmax);
	DBG("sleep_enter_current :%d\n", cell_cfg->ocv->sleep_enter_current);
	DBG("sleep_exit_current :%d\n", cell_cfg->ocv->sleep_exit_current);

	DBG("\n--------- rk818_battery dt_parse ok !----------\n");
	return 0;
}

/*
static struct of_device_id rk818_battery_of_match[] = {
{ .compatible = "rk818_battery" },
{ }
};

MODULE_DEVICE_TABLE(of, rk818_battery_of_match);
*/
#else
static int rk818_battery_parse_dt(struct device *dev)
{
	return -ENODEV;
}
#endif

static irqreturn_t rk818_hot_die_irq(int irq, void *di)
{
	struct battery_info *info = ((struct battery_info *) di);
	u8 thermal_reg;

	pr_info("======rk818_hot_die_irq======\n");
	battery_read(info->rk818, THERMAL_REG, &thermal_reg, 1);
	if (thermal_reg & HOTDIE_STS) {
		pr_info("kernel_power_off\n");
		kernel_power_off();
	}
	return IRQ_HANDLED;
}

static irqreturn_t rk818_vbat_plug_in(int irq, void *di)
{
	struct battery_info *info = ((struct battery_info *) di);
	info->vbus_state_prev = info->vbus;
	info->vbus = VBUS_ON;

	if (info->vbus != info->vbus_state_prev) {

		if (info->otg_handle) {
			pr_info("usb notifier_call_chain called\n");
			atomic_notifier_call_chain(&info->otg_handle->notifier,
						   USB_EVENT_VBUS, &info->vbus);
		}
		info->vbus_state_prev = info->vbus;
	}
	return IRQ_HANDLED;
}

static irqreturn_t rk818_vbat_plug_out(int irq, void *di)
{
	struct battery_info *info = ((struct battery_info *) di);
	info->vbus_state_prev = info->vbus;
	info->vbus = VBUS_OFF;

	if (info->vbus != info->vbus_state_prev) {

		if (info->otg_handle) {
			pr_info("usb notifier_call_chain called\n");
			atomic_notifier_call_chain(&info->otg_handle->notifier,
						   USB_EVENT_VBUS, &info->vbus);
		}
		info->vbus_state_prev = info->vbus;
	}
	return IRQ_HANDLED;
}

static int battery_probe(struct platform_device *pdev)
{
	struct rk818 *chip = dev_get_drvdata(pdev->dev.parent);
	struct battery_info *di;
	int fcc_capacity;
	int ret, i;
	int hot_die_irq, plug_in_irq, plug_out_irq;
	struct usb_phy *otg_handle;
	u8 usb_status;

	DBG("%s is  the  battery driver version %s\n", __func__,
	    DRIVER_VERSION);
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	ret = rk818_battery_parse_dt(chip);
	if (ret < 0) {
		DBG("rk818_battery_parse_dt() failed!\n");
		return -EINVAL;
	}

	di->rk818 = chip;
	DBG("%s(0): the battery OCV TABLE size : %d\n", __func__,
	    chip->battery_data->ocv_size);
	DBG("%s(0): the battery OCV TABLE :\n", __func__);
	for (i = 0; i < chip->battery_data->ocv_size; i++)
		DBG("%d ", chip->battery_data->battery_ocv[i]);
	DBG("\n\n\n");

	platform_set_drvdata(pdev, di);

	di->platform_data = chip->battery_data;
	di->cell.config = di->platform_data->cell_cfg;
	di->design_capacity = di->platform_data->cell_cfg->design_capacity;
	di->qmax = di->platform_data->cell_cfg->design_qmax;
	di->fcc = di->design_capacity;
	di->vol_smooth_time = 0;
	di->charge_smooth_time = 0;
	di->charge_smooth_status = false;
	di->sleep_status = 0;
	di->work_on = 0;
	di->status = POWER_SUPPLY_STATUS_DISCHARGING;

	DBG("%s(1): the battery OCV TABLE size : %d\n", __func__,
	    di->platform_data->ocv_size);
	DBG("%s(1): the battery OCV TABLE :\n", __func__);
	for (i = 0; i < di->platform_data->ocv_size; i++)
		DBG("%d ", di->platform_data->battery_ocv[i]);
	DBG("\n\n\n");

	fcc_capacity = _get_FCC_capacity(di);
	if (fcc_capacity > 1000)
		di->fcc = fcc_capacity;
	else
		di->fcc = di->design_capacity;

	wake_lock_init(&di->resume_wake_lock, WAKE_LOCK_SUSPEND,
		       "resume_charging");

	/* battery_powersupply_init(di); */
	/* flatzone_voltage_init(di); */
	fg_init(di);
	DBG("\n--------- fg_init() ok ----------\n");
	battery_poweron_status(di);
	DBG("\n--------- battery_poweron_status() ok ----------\n");
	battery_powersupply_init(di);

	ret = power_supply_register(&pdev->dev, &di->bat);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register main battery\n");
		goto batt_failed;
	}
	ret = power_supply_register(&pdev->dev, &di->usb);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register usb power supply\n");
		goto usb_failed;
	}
	ret = power_supply_register(&pdev->dev, &di->ac);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register ac power supply\n");
		goto ac_failed;
	}

	di->wq = create_singlethread_workqueue("battery-work");
	INIT_DELAYED_WORK(&di->battery_monitor_work, rk818_battery_work);
	queue_delayed_work(di->wq, &di->battery_monitor_work,
			   msecs_to_jiffies(TIMER_MS_COUNTS * 5));
	/*queue_delayed_work(di->wq, &di->charge_check_work,
		msecs_to_jiffies(TIMER_MS_COUNTS*5)); */
	INIT_DELAYED_WORK(&di->charge_check_work,
			  rk818_battery_charge_check_work);
	/*INIT_DELAYED_WORK(&di->charge_check_work,rk818_usb_work);
	 * di->battery_nb.notifier_call = battery_notifier_call;
	 * register_battery_notifier(&di->battery_nb); */

	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg_handle == NULL) {
		pr_err("ERROR!: getting OTG transceiver failed\n");
		return -EINVAL;
	}
	di->otg_handle = otg_handle;
	di->vbus = VBUS_OFF;
	di->vbus_state_prev = VBUS_OFF;

	battery_read(di->rk818, 0xa0, &usb_status, 1);
	if (usb_status & 0x02) {
		di->vbus = VBUS_ON;
		if (di->otg_handle) {
			atomic_notifier_call_chain(&di->otg_handle->notifier,
						   USB_EVENT_VBUS, &di->vbus);
		}
		di->vbus_state_prev = VBUS_ON;
	} else {
		di->vbus = VBUS_OFF;
		if (di->otg_handle) {
			atomic_notifier_call_chain(&di->otg_handle->notifier,
						   USB_EVENT_VBUS, &di->vbus);
		}
		di->vbus_state_prev = VBUS_OFF;
	}


#if 1
	hot_die_irq = chip->irq_base + RK818_IRQ_HOTDIE;
	plug_in_irq = chip->irq_base + RK818_IRQ_PLUG_IN;
	plug_out_irq = chip->irq_base + RK818_IRQ_PLUG_OUT;

	pr_info("hot_die_irq = %d, plug_in_irq = %d, plug_out_irq = %d\n",
		hot_die_irq, plug_in_irq, plug_out_irq);

	ret = request_threaded_irq(hot_die_irq, NULL, rk818_hot_die_irq,
				   IRQF_TRIGGER_RISING, "rk818_hotdie", di);
	if (ret != 0) {
		dev_err(chip->dev,
			"Failed to request periodic RK818_IRQ_HOTDIE IRQ %d: %d\n",
			hot_die_irq, ret);

	}
	ret = request_threaded_irq(plug_in_irq, NULL, rk818_vbat_plug_in,
		IRQF_TRIGGER_RISING, "rk818_vbat_plug_in", di);
	if (ret != 0) {
		dev_err(chip->dev,
			"Failed to request periodic RK818_IRQ_PLUG_IN IRQ %d: %d\n",
			plug_in_irq, ret);

	}

	ret = request_threaded_irq(plug_out_irq, NULL, rk818_vbat_plug_out,
				   IRQF_TRIGGER_RISING, "rk818_vbat_plug_out",
				   di);
	if (ret != 0) {
		dev_err(chip->dev,
			"Failed to request periodic RK818_IRQ_PLUG_OUT IRQ %d: %d\n",
			plug_out_irq, ret);

	}
#endif
	/*********************************************/
	ret = device_create_file(&pdev->dev, &dev_attr_batparam);
	if (ret) {
		ret = -EINVAL;
		pr_info(KERN_ERR "failed to create bat param file\n");
		goto err_battery_failed;
	}

	ret = create_sysfs_interfaces(&pdev->dev);
	if (ret < 0) {
		ret = -EINVAL;
		dev_err(&pdev->dev,
			"device RK818 battery sysfs register failed\n");
		goto err_sysfs;
	}

	pr_info("--------------------- battery probe ok...\n");
	return ret;
err_battery_failed:
err_sysfs:
ac_failed:
	power_supply_unregister(&di->ac);
usb_failed:
	power_supply_unregister(&di->usb);
batt_failed:
	power_supply_unregister(&di->bat);
	return ret;
}

#ifdef CONFIG_PM
/* static int rk818_battery_suspend(struct device *dev) */
static int battery_suspend(struct platform_device *dev, pm_message_t state)
{

	u8 adc_value;
	struct battery_info *di = platform_get_drvdata(dev);
	DBG("%s--------------------\n", __func__);
	/* di->sleep_time = get */
	/* enable_irq(di->irq); */
	enable_vbat_low_irq(di);
	battery_read(di->rk818, 0x4D, &adc_value, 1);
	DBG("2 0x4d = 0x%x\n ", adc_value);

	di->sleep_status = di->status;
	di->sleep_charge_current = _get_average_current(di);
	DBG("sleep_charge_current = %d\n", di->sleep_charge_current);
	/* do_gettimeofday(&di->suspend_time); */
	di->suspend_capacity = di->remain_capacity;
	getnstimeofday(&di->suspend_time);
	di->sleep_soc = _get_soc(di);
	di->suspend_test_time = get_seconds();
	restart_relax(di);
	cancel_delayed_work(&di->battery_monitor_work);
	di->update_k = 0;
	di->update_q = 0;
	di->voltage_old = 0;
	di->display_soc = 0;

	return 0;
}

static int battery_resume(struct platform_device *dev)
{
	u8 buf;
	int ret;
	/*      uint16_t voltage; */
	struct battery_info *di = platform_get_drvdata(dev);

	DBG("%s------------------start--\n", __func__);
	ret = battery_read(di->rk818, VB_MOD_REG, &buf, 1);
	/* disable_irq(di->irq); */
	disable_vbat_low_irq(di);	/* battery */
	queue_delayed_work(di->wq, &di->battery_monitor_work,
			   msecs_to_jiffies(TIMER_MS_COUNTS / 2));
	di->resume = true;
	_get_OCV_voltage(di);
	if (di->sleep_status == POWER_SUPPLY_STATUS_CHARGING
	    || di->real_soc <= 5)
		wake_lock_timeout(&di->resume_wake_lock, 5 * HZ);
	di->remain_capacity = _get_realtime_capacity(di);

	if (di->sleep_status == POWER_SUPPLY_STATUS_CHARGING) {
		if (di->suspend_capacity > di->remain_capacity + 5)
			_capacity_init(di, di->real_soc * di->fcc);
	}

	DBG(" resuem remain capacity =  %d\n", di->remain_capacity);
	DBG("%s------------------end--\n", __func__);
	return 0;

}

static int battery_remove(struct platform_device *dev)
{
	struct battery_info *di = platform_get_drvdata(dev);
	DBG("%s--------------------\n", __func__);
	cancel_delayed_work_sync(&di->battery_monitor_work);
	return 0;
}

static void battery_shutdown(struct platform_device *dev)
{
	u8 adc_value;

	struct battery_info *di = platform_get_drvdata(dev);
	DBG("%s--------------------\n", __func__);
	/* adc_value = 0xf7; */
	battery_read(di->rk818, 0xAD, &adc_value, 1);
	DBG("%s start   %x\n", __func__, adc_value);
	adc_value = 0xf1;
	battery_write(di->rk818, 0xAD, &adc_value, 1);
	battery_read(di->rk818, 0xAD, &adc_value, 1);

	DBG("%s start   %x\n", __func__, adc_value);

	cancel_delayed_work_sync(&di->battery_monitor_work);
}
#endif

static struct platform_driver battery_driver = {
	.driver = {
		   .name = "rk818-battery",
		   .owner = THIS_MODULE,
		   },

	.probe = battery_probe,
	.remove = battery_remove,
	.suspend = battery_suspend,
	.resume = battery_resume,
	.shutdown = battery_shutdown,
};

static int __init battery_init(void)
{
	return platform_driver_register(&battery_driver);
}

/*module_init(battery_init);*/
late_initcall(battery_init);
/*subsys_initcall_sync(battery_init);*/
static void __exit battery_exit(void)
{
	platform_driver_unregister(&battery_driver);
}

module_exit(battery_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rk818-battery");
MODULE_AUTHOR("ROCKCHIP");
