/*
* rk818  battery driver
*
* Copyright (C) 2014 Rockchip Electronics Co., Ltd
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
#include <linux/mfd/rk818.h>
/* #include <linux/power/rk818_battery.h> */
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/usb/otg.h>

static int dbg_enable = 1;
module_param_named(dbg_level, dbg_enable, int, 0644);
#define DBG(fmt, ...) \
	do { \
		if (dbg_enable) { \
			pr_info("%s:" fmt, __func__, ##__VA_ARGS__); \
		} \
	} while (0)

#define VB_MOD_REG					0x21

#define CHRG_COMP_REG1				0x99
#define CHRG_COMP_REG2				0x9A
#define SUP_STS_REG					0xA0
#define USB_CTRL_REG				0xA1
#define CHRG_CTRL_REG1				0xA3
#define CHRG_CTRL_REG2				0xA4
#define CHRG_CTRL_REG3				0xA5
#define BAT_CTRL_REG				0xA6
#define BAT_HTS_TS1_REG			0xA8
#define BAT_LTS_TS1_REG			0xA9
#define BAT_HTS_TS2_REG			0xAA
#define BAT_LTS_TS2_REG			0xAB

#define TS_CTRL_REG					0xAC
#define ADC_CTRL_REG				0xAD

#define ON_SOURCE					0xAE
#define OFF_SOURCE					0xAF

#define GGCON						0xB0
#define GGSTS						0xB1
#define FRAME_SMP_INTERV_REG		0xB2
#define AUTO_SLP_CUR_THR_REG		0xB3

#define GASCNT_CAL_REG3			0xB4
#define GASCNT_CAL_REG2			0xB5
#define GASCNT_CAL_REG1			0xB6
#define GASCNT_CAL_REG0			0xB7
#define GASCNT3						0xB8
#define GASCNT2						0xB9
#define GASCNT1						0xBA
#define GASCNT0						0xBB

#define BAT_CUR_AVG_REGH			0xBC
#define BAT_CUR_AVG_REGL			0xBD

#define TS1_ADC_REGH				0xBE
#define TS1_ADC_REGL				0xBF
#define TS2_ADC_REGH				0xC0
#define TS2_ADC_REGL				0xC1

#define BAT_OCV_REGH				0xC2
#define BAT_OCV_REGL				0xC3
#define BAT_VOL_REGH				0xC4
#define BAT_VOL_REGL				0xC5

#define RELAX_ENTRY_THRES_REGH	0xC6
#define RELAX_ENTRY_THRES_REGL	0xC7
#define RELAX_EXIT_THRES_REGH		0xC8
#define RELAX_EXIT_THRES_REGL		0xC9

#define RELAX_VOL1_REGH			0xCA
#define RELAX_VOL1_REGL			0xCB
#define RELAX_VOL2_REGH			0xCC
#define RELAX_VOL2_REGL			0xCD

#define BAT_CUR_R_CALC_REGH		0xCE
#define BAT_CUR_R_CALC_REGL		0xCF
#define BAT_VOL_R_CALC_REGH		0xD0
#define BAT_VOL_R_CALC_REGL		0xD1

#define CAL_OFFSET_REGH			0xD2
#define CAL_OFFSET_REGL			0xD3

#define NON_ACT_TIMER_CNT_REGL	0xD4

#define VCALIB0_REGH				0xD5
#define VCALIB0_REGL				0xD6
#define VCALIB1_REGH				0xD7
#define VCALIB1_REGL				0xD8

#define IOFFSET_REGH				0xDD
#define IOFFSET_REGL				0xDE

/*0xE0 ~0xF2  data register,*/
#define  SOC_REG						0xE0

#define  REMAIN_CAP_REG3			0xE1
#define  REMAIN_CAP_REG2			0xE2
#define  REMAIN_CAP_REG1			0xE3
#define  REMAIN_CAP_REG0			0xE4

#define  FCC_REGL					0xE1
#define  FCC_REGH					0xE2

/*gasgauge module enable bit 0: disable  1:enabsle   TS_CTRL_REG  0xAC*/
#define GG_EN						(1<<7)
/*ADC_CTRL_REG*/
/*if GG_EN = 0 , then the ADC of BAT voltage controlled by
the bit 0:diabsle 1:enable*/
#define ADC_VOL_EN					(1<<7)
/* if GG_EN = 0, then the ADC of BAT current controlled by the bit
0: disable 1: enable */
#define ADC_CUR_EN					(1<<6)
/* the ADC of TS1 controlled by the bit 0:disabsle 1:enable */
#define ADC_TS1_EN					(1<<5)
/* the ADC of TS2 controlled by the bit 0:disabsle 1:enable */
#define ADC_TS2_EN					(1<<4)
/* ADC colock phase  0:normal 1:inverted */
#define ADC_PHASE					(1<<3)
#define ADC_CLK_SEL					7
/*******************************************************************
#define ADC_CLK_SEL_2M				0x000
#define ADC_CLK_SEL_1M				0x001
#define ADC_CLK_SEL_500K			0x002
#define ADC_CLK_SEL_250K			0x003
#define ADC_CLK_SEL_125K			0x004
**********************************************************************/
/* GGCON */
/* ADC bat current continue sample times  00:8  01:16 10:32 11:64 */
#define CUR_SAMPL_CON_TIMES	       (3<<6)
/* ADC offset calibreation interval time 00:8min 01:16min 10:32min 11:48min */
#define ADC_OFF_CAL_INTERV			(3<<4)
/* OCV sampling interval time 00:8min 01:16min 10:32min :11:48min */
#define OCV_SAMPL_INTERV			(3<<2)

/* ADC working in current voltage collection mode */
#define ADC_CUR_VOL_MODE			(1<<1)
/* ADC working in resistor calculation mode 0:disable 1:enable */
#define ADC_RES_MODE				1

/* GGSTS */
/*average current filter times 00:1/2  01:1/4 10:1/8 11:1/16*/
#define RES_CUR_AVG_SEL		       (3<<5)
/*battery first connection,edge trigger 0:NOT  1:YES*/
#define BAT_CON						(1<<4)
/*battery voltage1 update in relax status 0: NOT 1:YES*/
#define RELAX_VOL1_UPD				(1<<3)
/*battery voltage2 update in relax status 0: NOT 1:YES*/
#define RELAX_VOL2_UPD				(1<<2)
/*battery coming into relax status  0: NOT 1:YES*/
#define RELAX_STS					(1<<1)
/*battery average voltage and current updated status 0: NOT 1:YES*/
#define IV_AVG_UPD_STS				(1<<0)

/* FRAME_SMP_INTERV_REG */
/* auto sleep mode 0:disable 1:enable */
#define AUTO_SLP_EN					(1<<5)
#define FRAME_SMP_INTERV_TIME			0x1F

#define PLUG_IN_STS					(1<<6)

/* SUP_STS_REG */
#define BAT_EXS						(1<<7)
#define CHARGE_OFF					(0x00<<4)
#define DEAD_CHARGE				(0x01<<4)
#define TRICKLE_CHARGE				(0x02<<4)
#define CC_OR_CV					(0x03<<4)
#define CHARGE_FINISH				(0x04<<4)
#define USB_OVER_VOL				(0x05<<4)
#define BAT_TMP_ERR					(0x06<<4)
#define TIMER_ERR					(0x07<<4)
/* usb is exists */
#define USB_EXIST					(1<<1)
/* usb is effective */
#define USB_EFF						(1<<0)

/* USB_CTRL_REG */
#define CHRG_CT_EN					(1<<7)
/* USB_VLIM_SEL */
#define VLIM_4000MV					(0x00<<4)
#define VLIM_4100MV					(0x01<<4)
#define VLIM_4200MV					(0x02<<4)
#define VLIM_4300MV					(0x03<<4)
#define VLIM_4400MV					(0x04<<4)
#define VLIM_4500MV					(0x05<<4)
#define VLIM_4600MV					(0x06<<4)
#define VLIM_4700MV					(0x07<<4)
/* USB_ILIM_SEL*/
#define ILIM_45MA					(0x00)
#define ILIM_300MA					(0x01)
#define ILIM_80MA					(0x02)
#define ILIM_820MA					(0x03)
#define ILIM_1000MA					(0x04)
#define ILIM_1200MA					(0x05)
#define ILIM_1400MA					(0x06)
#define ILIM_1600MA					(0x07)
#define ILIM_1800MA					(0x08)
#define ILIM_2000MA					(0x09)
#define ILIM_2200MA					(0x0A)
#define ILIM_2400MA					(0x0B)
#define ILIM_2600MA					(0x0C)
#define ILIM_2800MA					(0x0D)
#define ILIM_3000MA					(0x0E)

/* CHRG_CTRL_REG */
#define CHRG_EN						(0x01<<7)
/* CHRG_VOL_SEL */

#define CHRG_VOL4050				(0x00<<4)
#define CHRG_VOL4100				(0x01<<4)
#define CHRG_VOL4150				(0x02<<4)
#define CHRG_VOL4200				(0x03<<4)
#define CHRG_VOL4300				(0x04<<4)
#define CHRG_VOL4350				(0x05<<4)

/* CHRG_CUR_SEL */
#define CHRG_CUR1000mA			(0x00)
#define CHRG_CUR1200mA			(0x01)
#define CHRG_CUR1400mA			(0x02)
#define CHRG_CUR1600mA			(0x03)
#define CHRG_CUR1800mA			(0x04)
#define CHRG_CUR2000mA			(0x05)
#define CHRG_CUR2200mA			(0x06)
#define CHRG_CUR2400mA			(0x07)
#define CHRG_CUR2600mA			(0x08)
#define CHRG_CUR2800mA			(0x09)
#define CHRG_CUR3000mA			(0x0A)

#define DRIVER_VERSION				"1.0.0"
#define ROLEX_SPEED			        (100 * 1000)

#define CHARGING					0x01
#define DISCHARGING					0x00

#define	TIMER_MS_COUNTS					1000
#define MAX_CHAR					0x7F
#define MAX_UNSIGNED_CHAR			0xFF
#define MAX_INT						0x7FFFFFFF
#define MAX_UNSIGNED_INT			0xFFFF
#define MAX_INT8					0x7F
#define MAX_UINT8					0xFF

/* Voltage and Current buffers */
#define AV_SIZE						5

static int16_t av_v[AV_SIZE];
static int16_t av_c[AV_SIZE];

static uint16_t av_v_index;
static uint16_t av_c_index;

#define INTERPOLATE_MAX		1000
/* #define OCV_TABLE_SIZE */

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
	/*struct i2c_client     *client; */
	struct usb_phy *otg_handle;
	int vbus;
	int vbus_state_prev;

	struct rk818 *rk818;
	struct battery_platform_data *platform_data;
	struct notifier_block battery_nb;
	struct workqueue_struct *wq;
	struct delayed_work battery_monitor_work;
	struct delayed_work charge_check_work;

	int ac_online;
	int usb_online;
	int health;
	int tempreture;
	int present;
	int status;

	int bat_current;
	int current_avg;
	int current_offset;

	int voltage;
	int voltage_avg;
	int voltage_offset;
	int voltage_ocv;

	int poweroff_voltage;
	int warnning_voltage;
	int poweron_voltage;

	int design_capacity;
	int fcc;
	int new_fcc;
	u32 qmax;
	int remain_capacity;
	int warnning_capacity;
	int nac;
	int temp_nac;

	int real_soc;
	int display_soc;
	int temp_soc;

	int soc_counter;

	int dod0;
	int dod0_capacity;
	int dod1;
	int dod1_capacity;

	int temperature;

	int time2empty;
	int time2full;

	int *ocv_table;
	int ocv_size;
	int *res_table;

	int current_k;		/*(ICALIB0,ICALIB1) */
	int current_b;

	int voltage_k;		/*VCALIB0 VCALIB1 */
	int voltage_b;

	int relax_entry_thres;
	int relax_exit_thres;

	int relax_vol1;
	int relax_vol2;

	u8 sleep_cur;
	u8 sleep_smp_time;
	u8 check_count;
	/*u32                           status; */
	struct timeval soc_timer;
	struct timeval change_timer;

	bool resume;
	int charge_otg;

};
struct battery_info *data;

u32
interpolate(int value, u32 *table, int size)
{
	uint8_t i;
	uint16_t d;

	for (i = 0; i < size; i++) {
		if (value < table[i])
			break;
	}

	if ((i > 0) && (i < size)) {
		d = (value - table[i - 1]) * (INTERPOLATE_MAX / (size - 1));
		d /= table[i] - table[i - 1];
		d = d + (i - 1) * (INTERPOLATE_MAX / (size - 1));
	} else {
		d = i * ((INTERPOLATE_MAX + size / 2) / size);
	}

	if (d > 1000)
		d = 1000;

	return d;
}

/* Returns (a * b) / c */
int32_t
ab_div_c(u32 a, u32 b, u32 c)
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

static int32_t
abs_int(int32_t x)
{
	return (x > 0) ? x : -x;
}

/* Returns diviation between 'size' array members */
uint16_t
diff_array(int16_t *arr, uint8_t size)
{
	uint8_t i;
	uint32_t diff = 0;

	for (i = 0; i < size - 1; i++)
		diff += abs_int(arr[i] - arr[i + 1]);

	if (diff > MAX_UNSIGNED_INT)
		diff = MAX_UNSIGNED_INT;

	return (uint16_t) diff;
}

static enum power_supply_property rk818_battery_props[] = {

	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
#if 0
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	/*POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW, */
	/*POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG, */
	/*POWER_SUPPLY_PROP_TIME_TO_FULL_NOW, */
#endif

};

static enum power_supply_property rk818_battery_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property rk818_battery_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int
battery_read(struct rk818 *rk818, u8 reg, u8 buf[], unsigned len)
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

static void
dump_gauge_register(struct battery_info *di)
{
	int i = 0;
	char buf;
	DBG("dump charger register start:\n");
	for (i = 0xAC; i < 0xDE; i++) {
		battery_read(di->rk818, i, &buf, 1);
		DBG(" the register is  0x%02x, the value is 0x%02x\n ", i, buf);
	}
	DBG("demp end!\n");
}

static void
dump_charger_register(struct battery_info *di)
{

	int i = 0;
	char buf;
	DBG("dump the register start:\n");
	for (i = 0x99; i < 0xAB; i++) {
		battery_read(di->rk818, i, &buf, 1);
		DBG(" the register is  0x%02x, the value is 0x%02x\n ", i, buf);
	}
	DBG("demp end!\n");

}

#if 0
/*POWER_SUPPLY_PROP_STATUS*/
static int
rk818_battery_status(struct battery_info *di)
{
	return di->status;
}

/*POWER_SUPPLY_PROP_PRESENT,*/
static int
rk818_battery_present(struct rk818_battery_info *di)
{
	return 1;
}
#endif
/* OCV Lookup table
* Open Circuit Voltage (OCV) correction routine. This function estimates SOC,
* based on the voltage.
*/
static int
_voltage_to_capacity(struct battery_info *di, int voltage)
{
	u32 *ocv_table;
	int ocv_size;
	u32 tmp;

	ocv_table = di->platform_data->battery_ocv;
	ocv_size = di->platform_data->ocv_size;
	/*ocv_table = di->ocv_table; */
	/*ocv_size = di->ocv_size; */
	tmp = interpolate(voltage, ocv_table, ocv_size);
	di->temp_soc = ab_div_c(tmp, MAX_PERCENTAGE, INTERPOLATE_MAX);
	di->temp_nac = ab_div_c(tmp, di->fcc, INTERPOLATE_MAX);
	DBG("temp = %d real-soc =%d nac= %d, fcc = %d\n", tmp,
	    di->temp_soc, di->temp_nac, di->fcc);
	return 0;
}

/*POWER_SUPPLY_PROP_CURRENT_NOW,*/
static int
_get_average_current(struct battery_info *di)
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

	if (current_now & 0x800)
		current_now -= 4096;

	/*temp = current_now*1000*90/14/4096*500/521; */
	temp = current_now * 1506 / 1000;	/* 1000*90/14/4096*500/521;*/

	if (ret < 0) {
		dev_err(di->dev, "error reading BAT_CUR_AVG_REGH");
		return ret;
	}

	DBG("average current current_now = %d current = %d\n",
	    current_now, temp);
	return temp;

}

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
		val->intval = di->voltage;	/*rk818_battery_voltage(di); */
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (di->real_soc < 0)
			di->real_soc = 0;
		if (di->real_soc > 100)
			di->real_soc = 100;
		val->intval = di->real_soc;
		/*DBG("POWER_SUPPLY_PROP_CAPACITY = %d,   val->intval =%d\n",
		   di->real_soc, val->intval); */
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->status;
		/*DBG("gBatStatus=%d\n",val->intval); */
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

#define to_ac_device_info(x) container_of((x), \
struct battery_info, ac);

static int
rk818_battery_ac_get_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	/*DBG("%d psp = %d\n",__LINE__,psp); */
	int ret = 0;
	struct battery_info *di = to_ac_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->ac_online;	/*discharging */
		/*DBG("%d val->intval = %d   di->status = %d\n",
		   __LINE__,val->intval, di->status); */
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

#define to_usb_device_info(x) container_of((x), \
struct battery_info, usb);

static int
rk818_battery_usb_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	int ret = 0;
	struct battery_info *di = to_usb_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->usb_online;	/*discharging */
		/*DBG(":%d val->intval = %d\n",__LINE__,val->intval); */
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int
rk818_battery_set_property(struct power_supply *psy,
			   enum power_supply_property psp,
			   union power_supply_propval *val)
{
	int ret = 0;
	struct battery_info *di = to_device_info(psy);

	DBG("\n");

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		DBG("POWER_SUPPLY_PROP_CURRENT_NOW\n");
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		DBG("POWER_SUPPLY_PROP_VOLTAGE_NOW\n");
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		DBG("POWER_SUPPLY_PROP_CAPACITY\n");
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		DBG("POWER_SUPPLY_PROP_HEALTH\n");
		break;

	case POWER_SUPPLY_PROP_STATUS:
		DBG("POWER_SUPPLY_PROP_STATUS\n");
		break;

	default:
		DBG("default\n");
		return -EINVAL;
	}

	return ret;
}

static int
rk818_battery_usb_ac_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int ret = 0;
	struct battery_info *di = to_usb_device_info(psy);

	DBG("===============\n");

	switch (psp) {
	case POWER_SUPPLY_PROP_CABLE_TYPE:
		DBG("POWER_SUPPLY_PROP_CABLE_TYPE\n");
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		DBG("POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT\n");
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		DBG("POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE\n");
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		DBG("set property POWER_SUPPLY_PROP_ENABLE_CHARGER\n");
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		DBG("set property POWER_SUPPLY_PROP_ENABLE_CHARGING\n");
		break;
	case POWER_SUPPLY_PROP_INLMT:
		DBG("set property POWER_SUPPLY_PROP_INLMT\n");
		break;
	case POWER_SUPPLY_PROP_CONTINUE_CHARGING:
		DBG("set property POWER_SUPPLY_PROP_CONTINUE_CHARGING\n");
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		DBG("set property POWER_SUPPLY_PROP_CHARGE_TERM_CUR\n");
		break;
/*
	case POWER_SUPPLY_PROP_MIN_TEMP:
		DBG("set property POWER_SUPPLY_PROP_MIN_TEMP\n");
		break;
	case POWER_SUPPLY_PROP_MAX_TEMP:
		DBG("set property POWER_SUPPLY_PROP_MAX_TEMP\n");
		break;
*/
	default:
		DBG("set property default\n");
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

static void
battery_powersupply_init(struct battery_info *di)
{
	di->bat.name = "BATTERY";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = rk818_battery_props;
	di->bat.num_properties = ARRAY_SIZE(rk818_battery_props);
	di->bat.get_property = rk818_battery_get_property;
	di->bat.set_property = rk818_battery_set_property;

	di->ac.name = "AC";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties = rk818_battery_ac_props;
	di->ac.num_properties = ARRAY_SIZE(rk818_battery_ac_props);
	di->ac.get_property = rk818_battery_ac_get_property;
	di->ac.set_property = rk818_battery_usb_ac_set_property;
	di->ac.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
	di->ac.supplied_to = rk818_supplied_to;
	di->ac.num_supplicants = ARRAY_SIZE(rk818_supplied_to);
	di->ac.throttle_states = rk818_dummy_throttle_states;
	di->ac.num_throttle_states = ARRAY_SIZE(rk818_dummy_throttle_states);

	di->usb.name = "USB";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = rk818_battery_usb_props;
	di->usb.num_properties = ARRAY_SIZE(rk818_battery_usb_props);
	di->usb.get_property = rk818_battery_usb_get_property;
	di->usb.set_property = rk818_battery_usb_ac_set_property;
	di->usb.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
	    POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
	di->usb.supplied_to = rk818_supplied_to;
	di->usb.num_supplicants = ARRAY_SIZE(rk818_supplied_to);
	di->usb.throttle_states = rk818_dummy_throttle_states;
	di->usb.num_throttle_states = ARRAY_SIZE(rk818_dummy_throttle_states);
}

/* enabsle GG_EN */
static int
_gauge_enable(struct battery_info *di)
{
	int ret;
	u8 buf;
	DBG("start\n");
	ret = battery_read(di->rk818, TS_CTRL_REG, &buf, 1);
	DBG("_gauge_enable read-%d\n", buf);

	if (ret < 0) {
		dev_err(di->dev, "error reading TS_CTRL_REG");
		return ret;
	}
	if (!(buf & GG_EN)) {
		buf |= GG_EN;
		ret = battery_write(di->rk818, TS_CTRL_REG,
				&buf, 1);	/*enable */
		ret = battery_read(di->rk818, TS_CTRL_REG, &buf, 1);
		return 0;
	}

	DBG("%d\n", buf);
	return 0;

}

#if 0

static int
_gauge_disable(struct battery_info *di)
{
	int ret;
	u8 buf;

	ret = battery_read(di->rk818, TS_CTRL_REG, &buf, 1);
	if (ret < 0) {
		dev_err(di->dev, "error reading TS_CTRL_REG");
		return ret;
	}
	if ((buf & GG_EN)) {
		buf &= (~0x80);	/*GG_EN */
		ret = battery_write(di->rk818, TS_CTRL_REG,
				&buf, 1);	/*enable */
		return 0;
	}
	return 0;
}

static int
_set_auto_sleep_cur(struct battery_info *di, u8 value)
{
	int ret;
	u8 buf;
	buf = value;
	ret = battery_write(di->rk818, AUTO_SLP_CUR_THR_REG,
			&buf, 1);	/*enable */
	return 0;
}

static int
_set_sleep_smp_time(struct battery_info *di, u8 value)
{

	int ret;
	u8 temp;
	u8 buf;

	ret = battery_read(di->rk818, FRAME_SMP_INTERV_REG, &buf, 1);
	if (ret < 0) {
		dev_err(di->dev, "error reading FRAME_SMP_INTERV_REG");
		return ret;
	}

	temp = (buf & (AUTO_SLP_EN)) | value;
	ret = battery_write(di->rk818, FRAME_SMP_INTERV_REG,
			&temp, 1);	/*enable */

	return 0;
}

static int
_autosleep_enable(struct battery_info *di)
{
	int ret;
	u8 buf;

	ret = battery_read(di->rk818, FRAME_SMP_INTERV_REG, &buf, 1);
	if (ret < 0) {
		dev_err(di->dev, "error reading FRAME_SMP_INTERV_REG");
		return ret;
	}
	if (!(buf & AUTO_SLP_EN)) {
		buf |= AUTO_SLP_EN;
		ret = battery_write(di->rk818, FRAME_SMP_INTERV_REG,
				&buf, 1);	/*enable */
		return 0;
	}

	_set_auto_sleep_cur(di, di->sleep_cur);
	/*<di->sleep_cur  , into sleep-mode */
	_set_sleep_smp_time(di, di->sleep_smp_time);
	/*time of adc work , sleep-mode */

	return 0;

}

static int
_autosleep_disable(struct battery_info *di)
{
	int ret;
	u8 buf;

	ret = battery_read(di->rk818, FRAME_SMP_INTERV_REG, &buf, 1);
	if (ret < 0) {
		dev_err(di->dev, "error reading FRAME_SMP_INTERV_REG");
		return ret;
	}
	if ((buf & AUTO_SLP_EN)) {
		buf &= (~AUTO_SLP_EN);
		ret = battery_write(di->rk818, FRAME_SMP_INTERV_REG,
				&buf, 1);	/*enable */
		return 0;
	}
	return 0;

}

#endif
static void
_set_relax_thres(struct battery_info *di)
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

	/*set sample time */
	battery_read(di->rk818, GGCON, &buf, 1);
	buf &= ~(3 << 2);
	battery_write(di->rk818, GGCON, &buf, 1);
}

static int
rk818_battery_voltage(struct battery_info *di)
{
	int ret;
	int voltage_now = 0;
	u8 buf;
	int temp;
#if 1
	ret = battery_read(di->rk818, BAT_VOL_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, BAT_VOL_REGH, &buf, 1);
	temp |= buf << 8;
#endif

	/*ret = battery_read(di->rk818,BAT_VOL_REGH, buf, 2); */
	if (ret < 0) {
		dev_err(di->dev, "error reading BAT_VOL_REGH");
		return ret;
	}

	/*voltage_now = temp;//(buf[0]<<8)|buf[1]; */
	voltage_now = di->voltage_k * temp + di->voltage_b;

	DBG("the rea-time voltage is %d\n", voltage_now);
	return voltage_now;
}

static int
_get_ocv_voltage(struct battery_info *di)
{
	int ret;
	int voltage_now = 0;
	u8 buf;
	int temp;
#if 1
	ret = battery_read(di->rk818, BAT_OCV_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, BAT_OCV_REGH, &buf, 1);
	temp |= buf << 8;
#endif

	/*ret = battery_read(di->rk818,BAT_OCV_REGH, &buf, 2); */
	if (ret < 0) {
		dev_err(di->dev, "error reading BAT_OCV_REGH");
		return ret;
	}

	/*voltage_now = temp;//(buf[0]<<8)|buf[1]; */
	voltage_now = di->voltage_k * temp + di->voltage_b;
	DBG("the OCV voltage is %d\n", voltage_now);

	return voltage_now;
}

#if 0
static int
_get_ts1_adc(struct battery_info *di)
{
	int ret;
	int temp = 0;
	u8 buf;

	ret = battery_read(di->rk818, TS1_ADC_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, TS1_ADC_REGH, &buf, 1);
	temp = (buf << 8);

	return temp;
}

static int
_get_ts2_adc(struct battery_info *di)
{
	int ret;
	int temp = 0;
	u8 buf;
#if 1
	ret = battery_read(di->rk818, TS2_ADC_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, TS2_ADC_REGH, &buf, 1);
	temp |= buf << 8;
#endif

	return temp;
}
#endif
static void
_capacity_init(struct battery_info *di, u32 capacity)
{

	u8 buf;
	u32 capacity_ma;

	capacity_ma = capacity * 2201;	/* 36*14/900*4096/521*500; */
	DBG("WRITE GANCNT_CAL_REG  %d\n", capacity_ma);
	do {
		buf = (capacity_ma >> 24) & 0xff;
		battery_write(di->rk818, GASCNT_CAL_REG3, &buf, 1);
		buf = (capacity_ma >> 16) & 0xff;
		battery_write(di->rk818, GASCNT_CAL_REG2, &buf, 1);
		buf = (capacity_ma >> 8) & 0xff;
		battery_write(di->rk818, GASCNT_CAL_REG1, &buf, 1);
		buf = (capacity_ma & 0xff) | 0x01;
		battery_write(di->rk818, GASCNT_CAL_REG0, &buf, 1);
		battery_read(di->rk818, GASCNT_CAL_REG0, &buf, 1);

	} while (buf == 0);
	return;

}

static void
_save_remain_capacity(struct battery_info *di, u32 capacity)
{

	u8 buf;
	u32 capacity_ma;

	if (capacity >= di->qmax)
		capacity = di->qmax;
	capacity_ma = capacity;
	/*DBG("WRITE GANCNT_CAL_REG  %d\n", capacity_ma); */
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

static int
_get_remain_capacity(struct battery_info *di)
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
	DBG("GASCNT_CAL_REG %d  capacity =%d\n", temp, capacity);
	return capacity;

}

static int
_get_capacity(struct battery_info *di)
{
	int ret;
	int temp = 0;
	u8 buf;
	u32 capacity;

	ret = battery_read(di->rk818, GASCNT_CAL_REG3, &buf, 1);
	temp = buf << 24;
	ret = battery_read(di->rk818, GASCNT_CAL_REG2, &buf, 1);
	temp |= buf << 16;
	ret = battery_read(di->rk818, GASCNT_CAL_REG1, &buf, 1);
	temp |= buf << 8;
	ret = battery_read(di->rk818, GASCNT_CAL_REG0, &buf, 1);
	temp |= buf;

	capacity = temp / 2201;	/*4096*900/14/36*500/521;*/
	/*DBG("GASCNT_CAL_REG %d  capacity =%d\n",
	 * temp, capacity); */
	return capacity;

}

static int
_get_realtime_capacity(struct battery_info *di)
{

	int ret;
	int temp = 0;
	u8 buf;
	u32 capacity;

	ret = battery_read(di->rk818, GASCNT3, &buf, 1);
	temp = buf << 24;
	ret = battery_read(di->rk818, GASCNT2, &buf, 1);
	temp |= buf << 16;
	ret = battery_read(di->rk818, GASCNT1, &buf, 1);
	temp |= buf << 8;
	ret = battery_read(di->rk818, GASCNT0, &buf, 1);
	temp |= buf;
	/*ret = battery_read(di->rk818,GASCNT_CAL_REG3, &buf, 4); */
	/*temp = buf[0] << 24 | buf[1] << 24 | buf[2] << 24 |buf[3] ; */
	capacity = temp / 2201;	/*4096*900/14/36*500/521;*/
	/*DBG("GASCNT =  0x%4x  capacity =%d\n",temp,capacity); */
	return capacity;

}

static int
_get_relax_vol1(struct battery_info *di)
{
	int ret;
	int temp = 0, voltage_now;
	u8 buf;

	ret = battery_read(di->rk818, RELAX_VOL1_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, RELAX_VOL1_REGH, &buf, 1);
	temp |= buf << 8;
	/*ret = battery_read(di->rk818,RELAX_VOL1_REGH, &buf, 2); */
	/*temp  = (buf[0]<<8)|buf[1]; */
	voltage_now = di->voltage_k * temp + di->voltage_b;

	return voltage_now;
}

static int
_get_relax_vol2(struct battery_info *di)
{
	int ret;
	int temp = 0, voltage_now;
	u8 buf;

	ret = battery_read(di->rk818, RELAX_VOL2_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, RELAX_VOL2_REGH, &buf, 1);
	temp |= buf << 8;
	/*ret = battery_read(di->rk818,RELAX_VOL2_REGH, &buf, 2); */
	/*temp  = (buf[0]<<8)|buf[1]; */
	voltage_now = di->voltage_k * temp + di->voltage_b;

	return temp;
}

static bool
_is_relax_mode(struct battery_info *di)
{
	int ret;
	u8 status;
	int relax_vol1, relax_vol2;

	struct cell_state *cell = &di->cell;

	ret = battery_read(di->rk818, GGSTS, &status, 1);
	DBG(" GGSTS the value is %2x the realsoc = %d\n",
	    status, di->real_soc);

	if (!(status & RELAX_STS))
		return false;
	if ((!(status & RELAX_VOL1_UPD)) || (!(status & RELAX_VOL2_UPD)))
		return false;
	else {
		if ((di->real_soc >= cell->config->ocv->flat_zone_low)
		    && (di->real_soc <= cell->config->ocv->flat_zone_high))
			return false;
		relax_vol1 = _get_relax_vol1(di);
		relax_vol2 = _get_relax_vol2(di);
		DBG("relax_vol1 = %d relax_vol2 =%d\n", relax_vol1,
		    relax_vol2);
		if ((abs_int((relax_vol2 - relax_vol1))) / 8 / 60 > 4)
			return false;
	}

	return true;
}

static int
relax_soc(struct battery_info *di)
{
	/*int relax_soc; */
	_voltage_to_capacity(di, di->voltage);
	return di->temp_soc;
}

static int
_get_vcalib0(struct battery_info *di)
{

	int ret;
	int temp = 0;
	u8 buf;
#if 1
	ret = battery_read(di->rk818, VCALIB0_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, VCALIB0_REGH, &buf, 1);
	temp |= buf << 8;
#endif
	/*ret = battery_read(di->rk818,VCALIB0_REGH, &buf,2); */
	/*temp  = (buf[0]<<8)|buf[1]; */

	DBG("voltage0 offset vale is %d\n", temp);
	return temp;
}

static int
_get_vcalib1(struct battery_info *di)
{

	int ret;
	int temp = 0;
	u8 buf;
#if 1
	ret = battery_read(di->rk818, VCALIB1_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, VCALIB1_REGH, &buf, 1);
	temp |= buf << 8;
#endif
	/*ret = battery_read(di->rk818,VCALIB1_REGH, &buf, 2); */
	/*temp  = (buf[0]<<8)|buf[1]; */
	DBG("voltage1 offset vale is %d\n", temp);
	return temp;
}

static void
_get_voltage_offset_value(struct battery_info *di)
{
	int vcalib0, vcalib1;

	vcalib0 = _get_vcalib0(di);
	vcalib1 = _get_vcalib1(di);

	di->voltage_k = (4200 - 3000) / (vcalib1 - vcalib0);
	di->voltage_b = 4200 - di->voltage_k * vcalib1;

	return;
}

static int
_get_ioffset(struct battery_info *di)
{

	int ret;
	int temp = 0;
	u8 buf;

	ret = battery_read(di->rk818, IOFFSET_REGL, &buf, 1);
	temp = buf;
	ret = battery_read(di->rk818, IOFFSET_REGH, &buf, 1);
	temp |= buf << 8;

	/*ret = battery_read(di->rk818,IOFFSET_REGH, &buf, 2); */
	/*temp  = (buf[0]<<8)|buf[1]; */

	DBG("IOFFSET value is %d\n", temp);
	return temp;
}

static int
_set_cal_offset(struct battery_info *di, u32 value)
{
	int ret;
	int temp = 0;
	u8 buf;
	DBG("\n");
	buf = value & 0xff;
	ret = battery_write(di->rk818, CAL_OFFSET_REGL, &buf, 1);/*enable*/
	buf = (value >> 8) & 0xff;
	ret = battery_write(di->rk818, CAL_OFFSET_REGH, &buf, 1);/*enable*/
	DBG("set CAL_OFFSET_REG %d\n", temp);

	return 0;
}

static bool
_is_first_poweron(struct battery_info *di)
{
	u8 buf;
	u8 temp;
	u8 ret;

	ret = battery_read(di->rk818, GGSTS, &buf, 1);
	DBG("GGSTS value is %2x\n", buf);
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

#if 0
static bool
fg_check_relaxed(struct battery_info *di)
{
	struct cell_state *cell = &di->cell;

	struct timeval now;

	if (!cell->sleep) {
		if (abs_int(di->current_avg) <=
		    cell->config->ocv->sleep_enter_current) {
			if (cell->sleep_samples < MAX_UINT8)
				cell->sleep_samples++;

			if (cell->sleep_samples >=
			    cell->config->ocv->sleep_enter_samples) {
				/* Entering sleep mode */
				do_gettimeofday(&cell->sleep_timer);
				do_gettimeofday(&cell->el_sleep_timer);
				cell->sleep = true;
				cell->calibrate = true;
			}
		} else {
			cell->sleep_samples = 0;
		}
	} else {
		/* The battery cell is Sleeping, checking if need to exit
		   sleep mode count number of seconds that cell spent in
		   sleep */
		do_gettimeofday(&now);
		cell->cumulative_sleep +=
		    now.tv_sec + cell->el_sleep_timer.tv_sec;
		do_gettimeofday(&cell->el_sleep_timer);

		/* Check if we need to reset Sleep */
		if (abs_int(di->current_avg) >
		    cell->config->ocv->sleep_exit_current) {

			if (abs_int(di->current_avg) >
			    cell->config->ocv->sleep_exit_current) {

				if (cell->sleep_samples < MAX_UINT8)
					cell->sleep_samples++;

			} else {
				cell->sleep_samples = 0;
			}

			/* Check if we need to reset a Sleep timer */
			if (cell->sleep_samples >
			    cell->config->ocv->sleep_exit_samples) {
				/* Exit sleep mode */

				cell->sleep_timer.tv_sec = 0;
				cell->sleep = false;
				cell->relax = false;
			}
		} else {
			cell->sleep_samples = 0;

			if (!cell->relax) {

				if (now.tv_sec - cell->sleep_timer.tv_sec >
				    cell->config->ocv->relax_period) {
					cell->relax = true;
					cell->calibrate = true;
				}
			}
		}
	}

	return cell->relax;
}

/* Checks for right conditions for OCV correction */
static bool
fg_can_ocv(struct battery_info *di)
{				/*(struct cell_state *cell) */
	struct cell_state *cell = &di->cell;
#if  1
	/* Voltage should be stable */
	if (cell->config->ocv->voltage_diff <= diff_array(av_v, AV_SIZE))
		return false;

	/* Current should be stable */
	if (cell->config->ocv->current_diff <= diff_array(av_c, AV_SIZE))
		return false;
#endif
	/* SOC should be out of Flat Zone */
	if ((di->real_soc >= cell->config->ocv->flat_zone_low)
	    && (di->real_soc <= cell->config->ocv->flat_zone_high))
		return false;

	/* Current should be less then SleepEnterCurrent */
	if (abs_int(di->current_avg) >= cell->config->ocv->sleep_enter_current)
		return false;

	/* Don't allow OCV below EDV1, unless OCVbelowEDV1 is set */
	/*if (cell->edv1 && !cell->config->ocv_below_edv1)
		return false;*/

	return true;
}

#endif

/* Sets the battery Voltage, and recalculates the average voltage */
void
fg_set_voltage(int16_t voltage)
{
	int16_t i;
	int32_t tmp = 0;

	/* put voltage reading int the buffer and update average */
	av_v_index++;
	av_v_index %= AV_SIZE;
	av_v[av_v_index] = voltage;
	for (i = 0; i < AV_SIZE; i++)
		tmp += av_v[i];
}

/* Sets the battery Current, and recalculates the average current */
void
fg_set_current(int16_t cur)
{
	int16_t i;
	int32_t tmp = 0;

	/* put current reading int the buffer and update average */
	av_c_index++;
	av_c_index %= AV_SIZE;
	av_c[av_c_index] = cur;
	for (i = 0; i < AV_SIZE; i++)
		tmp += av_c[i];

}

static int
_copy_soc(struct battery_info *di, u8 save_soc)
{
	u8 soc;

	soc = save_soc;
	/*soc = 85; */
	battery_write(di->rk818, SOC_REG, &soc, 1);
	battery_read(di->rk818, SOC_REG, &soc, 1);
	DBG(" the save soc-reg = %d\n", soc);

	return 0;
}

static void
_save_rsoc_nac(struct battery_info *di)
{
	u8 buf;

	buf = di->real_soc;

	battery_write(di->rk818, SOC_REG, &buf, 1);

}

static int
_rsoc_init(struct battery_info *di)
{
	int vol;
	u8 temp;
	u32 remain_capacity;

	vol = di->voltage_ocv; /*_get_ocv_voltage(di);*/
	DBG("OCV voltage = %d\n", di->voltage_ocv);
	if (_is_first_poweron(di)) {

		DBG("this is first poweron\n");
		_voltage_to_capacity(di, di->voltage_ocv);
		di->real_soc = di->temp_soc;
		di->nac = di->temp_nac;
	} else {
		DBG("this is  not not not first poweron\n");
		battery_read(di->rk818, SOC_REG, &temp, 1);
		remain_capacity = _get_remain_capacity(di);
		if (remain_capacity >= di->qmax)
			remain_capacity = di->qmax;
		DBG("saved SOC_REG = 0x%8x\n", temp);
		DBG("saved remain_capacity = %d\n", remain_capacity);

		di->real_soc = temp;
		/*di->nac = di->fcc*temp/100; */
		di->nac = remain_capacity;
	}
	return 0;
}

static int
_get_soc(struct battery_info *di)
{

	return di->remain_capacity * 100 / di->fcc;
}

static u8
get_charge_status(struct battery_info *di)
{
	u8 status;
	u8 ret = 0;

	battery_read(di->rk818, SUP_STS_REG, &status, 1);
	DBG("----- SUP_STS_REG(0xA0) = 0x%02x\n", status);
	status &= ~(0x07 << 4);
	switch (status) {
	case CHARGE_OFF:
		ret = CHARGE_OFF;
		break;
	case DEAD_CHARGE:
		ret = DEAD_CHARGE;
		break;
	case TRICKLE_CHARGE:	/*(0x02<<4) */
		ret = DEAD_CHARGE;
		break;
	case CC_OR_CV:		/*(0x03<<4) */
		ret = CC_OR_CV;
		break;
	case CHARGE_FINISH:	/*(0x04<<4) */
		ret = CHARGE_FINISH;
		break;

	case USB_OVER_VOL:	/*(0x05<<4) */
		ret = USB_OVER_VOL;
		break;

	case BAT_TMP_ERR:	/*(0x06<<4) */
		ret = BAT_TMP_ERR;
		break;

	case TIMER_ERR:	/*(0x07<<4) */
		ret = TIMER_ERR;
		break;

	case USB_EXIST:	/*(1<<1)// usb is exists */
		ret = USB_EXIST;
		break;

	case USB_EFF:		/*(1<<0)// usb is effective */
		ret = USB_EFF;
		break;
	default:
		return -EINVAL;
	}

	return ret;

}

static void
rk818_battery_charger_init(struct battery_info *di)
{
	u8 chrg_ctrl_reg1, usb_ctrl_reg;	/*chrg_ctrl_reg2; */
	u8 sup_sts_reg;

	DBG("start\n");

	battery_read(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);
	battery_read(di->rk818, CHRG_CTRL_REG1, &chrg_ctrl_reg1, 1);
	/*battery_read(di->rk818, CHRG_CTRL_REG2, &chrg_ctrl_reg2, 1); */
	battery_read(di->rk818, SUP_STS_REG, &sup_sts_reg, 1);

	DBG("old usb_ctrl_reg =0x%2x,CHRG_CTRL_REG1=0x%2x\n",
	    usb_ctrl_reg, chrg_ctrl_reg1);
	/* usb_ctrl_reg &= (0x01<<7); */
	usb_ctrl_reg |= (VLIM_4400MV | ILIM_300MA) | (0x01 << 7);

	chrg_ctrl_reg1 &= (0x00);
	chrg_ctrl_reg1 |= (0x01 << 7) | (CHRG_VOL4200 | CHRG_CUR1400mA);

	sup_sts_reg &= ~(0x01 << 3);
	sup_sts_reg |= (0x01 << 2);

	battery_write(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);

	battery_write(di->rk818, CHRG_CTRL_REG1, &chrg_ctrl_reg1, 1);
	/*battery_write(di->rk818, CHRG_CTRL_REG2, &chrg_ctrl_reg2, 1); */
	battery_write(di->rk818, SUP_STS_REG, &sup_sts_reg, 1);

	battery_read(di->rk818, CHRG_CTRL_REG1, &chrg_ctrl_reg1, 1);
	/*battery_read(di->rk818, CHRG_CTRL_REG2, &chrg_ctrl_reg2, 1); */
	battery_read(di->rk818, SUP_STS_REG, &sup_sts_reg, 1);
	battery_read(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);
	DBG("new usb_ctrl_reg =0x%2x,CHRG_CTRL_REG1=0x%2x,SUP_STS_REG=0x%2x\n",
			usb_ctrl_reg, chrg_ctrl_reg1, sup_sts_reg);

	DBG("end\n");

}

void
charge_disable_open_otg(struct battery_info *di, int value)
{
	/*u8 chrg_ctrl_reg1,dcdc_en_reg; */
	if (value == 1) {
		DBG("1    ---- charge disable\n");
		/* ldo9 */
		rk818_set_bits(di->rk818, CHRG_CTRL_REG1, 1 << 7, 0 << 7);
		rk818_set_bits(di->rk818, 0x23, 1 << 7, 1 << 7);
	}
	if (value == 0) {
		DBG("1    ---- charge disable\n");
		rk818_set_bits(di->rk818, 0x23, 1 << 7, 0 << 7);
		rk818_set_bits(di->rk818, CHRG_CTRL_REG1, 1 << 7, 1 << 7);
	}

}

static void
fg_init(struct battery_info *di)
{
	DBG("start\n");
	_gauge_enable(di);
	_get_voltage_offset_value(di);	/*get the volatege offset */
	/*_autosleep_enable(di);*/
	rk818_battery_charger_init(di);
	/*_set_relax_thres(di);*/
	/*get the current offset , the value write to the CAL_OFFSET */
	di->current_offset = _get_ioffset(di);
	_set_cal_offset(di, di->current_offset + 42);

	di->voltage = rk818_battery_voltage(di);
	di->voltage_ocv = _get_ocv_voltage(di);
	_rsoc_init(di);
	_capacity_init(di, di->nac);
	/*_get_realtime_capacity( di);*/
	di->remain_capacity = _get_capacity(di);
	/*_get_realtime_capacity( di);*/
	do_gettimeofday(&di->soc_timer);
	di->change_timer = di->soc_timer;
#if 0
	for (i = 0; i < AV_SIZE; i++) {
		av_v[i] = di->voltage;
		av_c[i] = 0;
	}
	av_v_index = 0;
	av_c_index = 0;
#endif
	/*dump_gauge_register(di); */
	/*dump_charger_register(di); */
	DBG("nac =%d , remain_capacity = %d\n"
	    " OCV_voltage =%d, voltage =%d\n",
	    di->nac, di->remain_capacity, di->voltage_ocv, di->voltage);
}

#if 0
static int
capacity_changed(struct battery_info *di)
{
	s32 acc_value, samples = 0;
	int ret;
	int acc_q;

	/*fg_set_voltage(&di->cell, di->voltage_mV); */
	/*fg_set_current(&di->cell, (int16_t)(di->current_uA/1000)); */

	return 0;
}

static void
rk818_battery_info(struct battery_info *di)
{
	/*di->status = rk818_battery_status(di); */
	/*di->voltage = rk818_battery_voltage(di); */
	di->present = rk818_battery_present(di);
	di->bat_current = _get_average_current(di);
	di->temp_soc = rk818_battery_soc(di);
	di->tempreture = rk818_battery_temperature(di);
	di->health = rk818_battery_health(di);
}
#endif

static void
rk818_battery_display_smooth(struct battery_info *di)
{
	int status;
	u8 charge_status;
	/*int relaxmode_soc; */
	/*int coulomp_soc, soc; */

	status = di->status;
	if (status == POWER_SUPPLY_STATUS_CHARGING) {
		/*DBG("charging smooth ...\n"); */
		if (1) {
			/*DBG("   BATTERY NOT RELAX MODE\n"); */
			DBG("di->remain_capacity =%d, di->fcc  = %d\n",
			    di->remain_capacity, di->fcc);
			di->temp_soc = _get_soc(di);
			charge_status = get_charge_status(di);
			if (di->temp_soc >= 100) {
				di->temp_soc = 100;
				/*di->status = POWER_SUPPLY_STATUS_FULL; */
			}

			do_gettimeofday(&di->soc_timer);

			if (di->temp_soc != di->real_soc) {
				di->change_timer = di->soc_timer;
				if (di->real_soc < di->temp_soc)
					di->real_soc++;
				else
					di->real_soc = di->temp_soc;
			}

			/*DBG("charge_status =0x%x\n", charge_status); */
			if ((charge_status == CHARGE_FINISH)
			    && (di->real_soc < 100)) {
				DBG("CHARGE_FINISH  di->real_soc < 100\n ");
				if ((di->soc_counter < 10)) {
					di->soc_counter++;
				} else {
					di->soc_counter = 0;
					if (di->real_soc < 100) {
						di->real_soc++;
						/* _save_rsoc_nac( di);*/
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
		/*DBG("discharging smooth ...\n"); */
		di->temp_soc = _get_soc(di);
		do_gettimeofday(&di->soc_timer);
		if (di->temp_soc != di->real_soc) {
			di->change_timer = di->soc_timer;
			di->real_soc = di->temp_soc;
			/*_save_rsoc_nac( di);*/
		}
		if (di->real_soc <= 0)
			di->real_soc = 0;
		if (di->real_soc >= 100)
			di->real_soc = 100;
#if 0
		if (!_is_relax_mode(di)) {
			DBG("   BATTERY NOT RELAX MODE\n");
			di->temp_soc = _get_soc(di);
			do_gettimeofday(&di->soc_timer);
			if (di->temp_soc != di->real_soc) {
				di->change_timer = di->soc_timer;
				di->real_soc = di->temp_soc;
				_save_rsoc_nac(di);
			}

		} else {
			DBG("BATTERY  RELAX MODE\n ");
			/*relaxmode_soc = relax_soc(di); */
			coulomp_soc = _get_soc(di);
			soc = coulomp_soc;

			if ((soc > di->real_soc) && (di->soc_counter < 10)) {
				di->soc_counter++;

			} else {
				di->soc_counter = 0;
				if (di->real_soc < 100) {
					di->real_soc--;
					_save_rsoc_nac(di);
				}
			}
			DBG(" remaxmode_soc = %d , coulomp-soc =%d  soc = %d\n",
			    relaxmode_soc, coulomp_soc, soc);
		}
#endif

	}
	/*DBG("exit\n"); */
}

static void
rk818_battery_update_status(struct battery_info *di)
{

	di->voltage = rk818_battery_voltage(di);
	di->current_avg = _get_average_current(di);
	di->remain_capacity = _get_realtime_capacity(di);
	_get_capacity(di);

	rk818_battery_display_smooth(di);

	DBG("\n"
	    "voltage = %d, current-avg = %d\n"
	    "fcc = %d ,remain_capacity =%d\n"
	    "real_soc = %d\n",
	    di->voltage, di->current_avg,
	    di->fcc, di->remain_capacity, di->real_soc);
}

/*extern int dwc_vbus_status(void);*/
/*extern int get_gadget_connect_flag(void);*/

static int
dwc_vbus_status(void)
{
	return 1;
}

/* state of charge ----running */
static int
get_charging_status(struct battery_info *di)
{
#if 0
	u8 usb_ctrl_reg;	/* chrg_ctrl_reg2; */

	battery_read(di->rk818, USB_CTRL_REG, &usb_ctrl_reg, 1);
	/*battery_read(di->rk818, CHRG_CTRL_REG2, &chrg_ctrl_reg2, 1); */

	DBG("old usb_ctrl_reg =0x%2x,CHRG_CTRL_REG1=0x%2x\n ", usb_ctrl_reg,
	    chrg_ctrl_reg1);
	usb_ctrl_reg &= (0x01 << 7);
	usb_ctrl_reg |= (ILIM_300MA);
#endif

	/*struct rk30_adc_battery_platform_data *pdata = bat->pdata; */
	/* 0--dischage ,1 ---usb charge, 2 ---ac charge */
	int usb_status = 0;
	int vbus_status = di->vbus;
	if (1 == vbus_status) {
		di->ac_online = 0;
		di->usb_online = 1;
		/*if (0 == get_gadget_connect_flag()){
		   if (++di->check_count >= 5){

		   di->ac_online = 1;
		   di->usb_online = 0;
		   }else{
		   di->ac_online =0;
		   di->usb_online = 1;

		   }
		   }else{

		   di->ac_online =0;
		   di->usb_online = 1;
		   } */

	} else {
		di->ac_online = 0;
		di->usb_online = 0;
		di->check_count = 0;

	}
	return usb_status;

}

static void
get_battery_status(struct battery_info *di)
{

	u8 buf;
	int ret;
	ret = battery_read(di->rk818, VB_MOD_REG, &buf, 1);

	if (buf & PLUG_IN_STS) {
		get_charging_status(di);
		di->status = POWER_SUPPLY_STATUS_CHARGING;
		if (di->real_soc == 100)
			di->status = POWER_SUPPLY_STATUS_FULL;
		DBG("enter, POWER_SUPPLY_STATUS_CHARGING\n");
	} else {
		di->status = POWER_SUPPLY_STATUS_DISCHARGING;
		di->ac_online = 0;
		di->usb_online = 0;

	}
	/*DBG("di->status = %d\n",di->status); */
}

static void
rk818_battery_work(struct work_struct *work)
{
	u8 buf;
	struct battery_info *di = container_of(work,
					       struct battery_info,
					       battery_monitor_work.work);
	int vbus_status;

	DBG("*************vbus = %d, prev = %d, di->otg_handle = 0x%x\n",
	       di->vbus, di->vbus_state_prev, di->otg_handle);
	get_battery_status(di);
	battery_read(di->rk818, 0x00, &buf, 1);
	DBG("RTC  =0x%2x\n ", buf);
	battery_read(di->rk818, VB_MOD_REG, &buf, 1);
	/*DBG("VB_MOD_REG  =%2x, the value is %2x\n ", VB_MOD_REG,buf); */
	battery_read(di->rk818, SUP_STS_REG, &buf, 1);
	/*DBG("SUP_STS_REG  =%2x, the value is %2x\n ", SUP_STS_REG,buf); */
	vbus_status = di->vbus;
	/*DBG("vbus_status  =%2x\n ", vbus_status); */

	rk818_battery_update_status(di);

	DBG("vbus = %d, prev = %d, di->otg_handle = 0x%x\n", di->vbus,
	       di->vbus_state_prev, di->otg_handle);
	if (di->vbus != di->vbus_state_prev) {

		if (di->otg_handle) {
			DBG("usb notifier_call_chain called\n");
			atomic_notifier_call_chain(&di->otg_handle->notifier,
						   USB_EVENT_VBUS, &di->vbus);
		}
		di->vbus_state_prev = di->vbus;
	}

	if (di->resume) {
		di->resume = false;
		di->real_soc = _get_soc(di);
		if (di->real_soc <= 0)
			di->real_soc = 0;
		if (di->real_soc >= 100)
			di->real_soc = 100;
	}
	if ((di->ac_online == 0) && (di->usb_online == 0)
	    && (di->remain_capacity > di->qmax + 10)) {
		_capacity_init(di, di->qmax);
		di->remain_capacity = _get_realtime_capacity(di);
	}

	/*DBG("soc  =  %d", di->real_soc); */
	_copy_soc(di, di->real_soc);
	_save_remain_capacity(di, di->remain_capacity);
	power_supply_changed(&di->bat);
	power_supply_changed(&di->usb);
	power_supply_changed(&di->ac);
	queue_delayed_work(di->wq, &di->battery_monitor_work,
			   msecs_to_jiffies(TIMER_MS_COUNTS * 5));

}

static void
rk818_battery_charge_check_work(struct work_struct *work)
{
	struct battery_info *di = container_of(work,
					       struct battery_info,
					       charge_check_work.work);
	charge_disable_open_otg(di, di->charge_otg);
}

/*static void rk818_usb_work(struct work_struct *work)
{
struct battery_info *info = container_of(work,
struct battery_info, charge_check_work.work);

DBG("rk818_usb_work\n");

if (info->vbus != info->vbus_state_prev) {

if (info->otg_handle) {
atomic_notifier_call_chain(&info->otg_handle->notifier,
USB_EVENT_VBUS, &info->vbus);
}
}
}*/
static BLOCKING_NOTIFIER_HEAD(battery_chain_head);

int
register_battery_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&battery_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_battery_notifier);

int
unregister_battery_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&battery_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_battery_notifier);

int
battery_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&battery_chain_head, val, NULL)
		== NOTIFY_BAD) ? -EINVAL : 0;
}
EXPORT_SYMBOL_GPL(battery_notifier_call_chain);

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
	default:
		return NOTIFY_OK;
	}
	return NOTIFY_OK;
}

#ifdef CONFIG_OF
static int
rk818_battery_parse_dt(struct rk818 *rk818)
{
	struct device_node *regs, *rk818_pmic_np;
	struct battery_platform_data *data;
	struct cell_config *cell_cfg;
	struct property *prop;
	u32 out_value;
	int i, length, ret;

	rk818_pmic_np = of_node_get(rk818->dev->of_node);
	if (!rk818_pmic_np) {
		DBG("could not find pmic sub-node\n");
		return -EINVAL;
	}

	regs = of_find_node_by_name(rk818_pmic_np, "battery");
	if (!regs) {
		DBG("could not find battery sub-node\n");
		return -EINVAL;
	}

	data = devm_kzalloc(rk818->dev, sizeof(*data), GFP_KERNEL);
	memset(data, 0, sizeof(*data));

	cell_cfg = devm_kzalloc(rk818->dev, sizeof(*cell_cfg), GFP_KERNEL);
	/* determine the number of brightness levels */
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
		ret = of_property_read_u32_array(regs, "ocv_table",
						 data->battery_ocv,
						 data->ocv_size);
		DBG("the battery OCV TABLE : ");
		for (i = 0; i < data->ocv_size; i++)
			DBG("%d ", data->battery_ocv[i]);
		DBG("\n");
		if (ret < 0)
			return ret;
	}
	DBG("rk818_battery_parse_dt 0\n");
	ret = of_property_read_u32(regs, "max_charge_currentma", &out_value);
	if (ret < 0)
		return ret;
	data->max_charger_currentma = out_value;
	DBG("rk818_battery_parse_dt 1\n");
	ret = of_property_read_u32(regs, "max_charge_voltagemv", &out_value);
	if (ret < 0)
		return ret;
	data->max_charger_voltagemv = out_value;
	DBG("rk818_battery_parse_dt 2\n");
	ret = of_property_read_u32(regs, "design_capacity", &out_value);
	if (ret < 0)
		return ret;
	cell_cfg->design_capacity = out_value;
	DBG("rk818_battery_parse_dt 3\n");
	ret = of_property_read_u32(regs, "design_qmax", &out_value);
	if (ret < 0)
		return ret;
	cell_cfg->design_qmax = out_value;
	data->cell_cfg = cell_cfg;
	rk818->battery_data = data;
	DBG("max_charge_currentmA :%d\n", data->max_charger_currentma);
	DBG("max_charge_voltagemV :%d\n", data->max_charger_voltagemv);
	DBG("design_capacity :%d\n", cell_cfg->design_capacity);
	DBG("design_qmax :%d\n", cell_cfg->design_qmax);
	DBG("rk818_battery_parse_dt 4\n");
	return 0;
}

static struct of_device_id rk818_battery_of_match[] = {
	{.compatible = "rk818_battery"},
	{}
};

MODULE_DEVICE_TABLE(of, rk818_battery_of_match);
#else
static int
rk818_battery_parse_dt(struct device *dev)
{
	return -ENODEV;
}
#endif

static irqreturn_t
rk818_vbat_lo_irq(int irq, void *di)
{
	DBG("lower power XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX...\n");
	DBG("lower power XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX...\n");
	DBG("lower power XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX...\n");

	return IRQ_HANDLED;
}

static irqreturn_t
rk818_vbat_plug_in(int irq, void *di)
{
	((struct battery_info *) di)->vbus_state_prev =
	    ((struct battery_info *) di)->vbus;
	((struct battery_info *) di)->vbus = VBUS_ON;

	DBG("^^^^^^^^^^^^^^^^:irq=%d, vbus = %d, prev = %d\n",
	       irq,
	       ((struct battery_info *) di)->vbus,
	       ((struct battery_info *) di)->vbus_state_prev);

	return IRQ_HANDLED;
}

static irqreturn_t
rk818_vbat_plug_out(int irq, void *di)
{
	((struct battery_info *) di)->vbus_state_prev =
	    ((struct battery_info *) di)->vbus;
	((struct battery_info *) di)->vbus = VBUS_OFF;

	DBG("^^^^^^^^^^^^^^^^:irq=%d, vbus = %d, prev = %d\n",
	       irq,
	       ((struct battery_info *) di)->vbus,
	       ((struct battery_info *) di)->vbus_state_prev);

	return IRQ_HANDLED;
}

static int
battery_probe(struct platform_device *pdev)
{
	struct rk818 *chip = dev_get_drvdata(pdev->dev.parent);
	struct battery_info *di;
	struct ocv_config *ocv;
	struct edv_config *edv;
	int ret;
	int vb_lo_irq, plug_in_irq, plug_out_irq;
	struct usb_phy *otg_handle;

	DBG("is the battery driver version %s\n", DRIVER_VERSION);
	rk818_battery_parse_dt(chip);
	DBG("battery_probe 0_0\n");
	di = devm_kzalloc(&pdev->dev, sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		return ret;
	}
	ocv = devm_kzalloc(&pdev->dev, sizeof(*ocv), GFP_KERNEL);
	if (!ocv) {
		dev_err(&pdev->dev, "ocv  no memory for state\n");
		ret = -ENOMEM;
		return ret;
	}
	edv = devm_kzalloc(&pdev->dev, sizeof(*edv), GFP_KERNEL);
	if (!edv) {
		dev_err(&pdev->dev, "edv  no memory for state\n");
		ret = -ENOMEM;
		return ret;
	}
	DBG("battery_probe 0_1\n");
	di->rk818 = chip;

	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg_handle == NULL) {
		pr_err("ERROR!: getting OTG transceiver failed\n");
		return -EINVAL;
	}
	di->otg_handle = otg_handle;
	di->vbus = VBUS_OFF;
	di->vbus_state_prev = VBUS_OFF;

#if 0
	di->platform_data = kmemdup(pdata, sizeof(*pdata), GFP_KERNEL);
	if (!di->platform_data) {
		kfree(di);
		return -ENOMEM;
	}
#endif
	/* data = di; */
	platform_set_drvdata(pdev, di);
	DBG("battery_probe 0_2\n");
	/*apply battery cell configuration */
	/*di->cell.config = di->platform_data->cell_cfg; */
	di->platform_data = chip->battery_data;
	di->platform_data->cell_cfg = chip->battery_data->cell_cfg;
	di->platform_data->cell_cfg->ocv = ocv;
	di->platform_data->cell_cfg->edv = edv;
	di->design_capacity = chip->battery_data->cell_cfg->design_capacity;
	di->qmax = chip->battery_data->cell_cfg->design_qmax;
	di->fcc = di->design_capacity;
	di->status = POWER_SUPPLY_STATUS_DISCHARGING;
	DBG("battery_probe 1\n");
	battery_powersupply_init(di);
	fg_init(di);
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
	DBG("battery_probe 2\n");
	di->wq = create_singlethread_workqueue("battery-work");
	INIT_DELAYED_WORK(&di->battery_monitor_work, rk818_battery_work);
	queue_delayed_work(di->wq, &di->battery_monitor_work,
			   msecs_to_jiffies(TIMER_MS_COUNTS * 5));
	/*queue_delayed_work(di->wq, &di->charge_check_work,
	   msecs_to_jiffies(TIMER_MS_COUNTS*5)); */
	INIT_DELAYED_WORK(&di->charge_check_work,
			  rk818_battery_charge_check_work);
	/*INIT_DELAYED_WORK(&di->charge_check_work,rk818_usb_work); */

	/*di->battery_nb.notifier_call = battery_notifier_call; */
	/*register_battery_notifier(&di->battery_nb); */

#if 1
	vb_lo_irq = irq_create_mapping(chip->irq_domain, RK818_IRQ_VB_LO);
	plug_in_irq = irq_create_mapping(chip->irq_domain, RK818_IRQ_PLUG_IN);
	plug_out_irq = irq_create_mapping(chip->irq_domain, RK818_IRQ_PLUG_OUT);

	DBG("vb_lo_irq = %d, plug_in_irq = %d, plug_out_irq = %d\n",
	       vb_lo_irq, plug_in_irq, plug_out_irq);

	ret = request_threaded_irq(vb_lo_irq, NULL, rk818_vbat_lo_irq,
				   IRQF_TRIGGER_HIGH, "rk818_vbatlow", di);
	if (ret != 0) {
		dev_err(chip->dev,
			"Failed to request periodic RK818_IRQ_VB_LO IRQ %d: %d\n",
			vb_lo_irq, ret);

	}
	ret = request_threaded_irq(plug_in_irq, NULL, rk818_vbat_plug_in,
				   IRQF_TRIGGER_RISING /*IRQF_TRIGGER_FALLING */
				   ,
				   "rk818_vbatlow", di);
	if (ret != 0) {
		dev_err(chip->dev,
			"Failed to request periodic RK818_IRQ_PLUG_IN IRQ %d: %d\n",
			plug_in_irq, ret);

	}

	ret = request_threaded_irq(plug_out_irq, NULL, rk818_vbat_plug_out,
				   IRQF_TRIGGER_RISING, "rk818_vbatlow", di);
	if (ret != 0) {
		dev_err(chip->dev,
			"Failed to request periodic RK818_IRQ_PLUG_OUT IRQ %d: %d\n",
			plug_out_irq, ret);

	}
#endif
	DBG("battery probe ok...\n");
	return ret;

ac_failed:
	power_supply_unregister(&di->ac);
usb_failed:
	power_supply_unregister(&di->usb);
batt_failed:
	power_supply_unregister(&di->bat);
	return ret;
}

static int
battery_remove(struct platform_device *dev)
{
	return 0;
}

#if 1
static int
battery_suspend(struct platform_device *dev, pm_message_t state)
{
	struct battery_info *di = platform_get_drvdata(dev);
	DBG("--------------------\n");
	if (di == NULL)
		DBG("battery NULL di\n");
	cancel_delayed_work(&di->battery_monitor_work);
	DBG("---------end--------\n");

	return 0;
}

static int
battery_resume(struct platform_device *dev)
{
	u8 buf;
	int ret;
	struct battery_info *di = platform_get_drvdata(dev);

	ret = battery_read(di->rk818, VB_MOD_REG, &buf, 1);

	/*struct battery_info *di  = platform_get_drvdata(dev); */
	DBG("--------------------\n");
	queue_delayed_work(di->wq, &di->battery_monitor_work,
			   msecs_to_jiffies(TIMER_MS_COUNTS));
	di->resume = true;
	DBG("charge--status       0x%02x--------------------buf = 0x%02x\n",
	    get_charge_status(di), buf);

	return 0;
}
#endif
static struct platform_driver battery_driver = {
	.probe = battery_probe,
	.remove = battery_remove,
	.suspend = battery_suspend,
	.resume = battery_resume,

	.driver = {
		   .name = "rk818-battery",
		   /*.pm   = &pm_ops, */
		   /*.of_match_table = of_match_ptr(rk818_battery_parse_dt), */
		   },
};

static int __init
battery_init(void)
{
	return platform_driver_register(&battery_driver);
}

fs_initcall_sync(battery_init);
static void __exit
battery_exit(void)
{
	platform_driver_unregister(&battery_driver);
}

module_exit(battery_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rk818-battery");
MODULE_AUTHOR("ROCKCHIP");
