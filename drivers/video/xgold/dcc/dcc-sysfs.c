/*
 ****************************************************************
 *
 *  Component: DCC driver
 *
 *  Copyright (C) 2011, Intel Mobile Communications GmbH.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include "dcc-core.h"
#include "dcc-display.h"
#include "dcc-sysfs.h"


/**
 * Hardware Informations
 */
static ssize_t dcc_sys_hw_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dcc_drvdata *pdata = dev_get_drvdata(dev);

	if (!pdata)
		return 0;

	return sprintf(buf, "Hardware DCC IP version : 0x%x\n", pdata->id);
}

static DEVICE_ATTR(hw, S_IRUSR, dcc_sys_hw_show, NULL);

/**
 * Memory Informations
 */
static ssize_t dcc_sys_mem_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i = 0;
	int n = 0;
	char str[128];
	struct dcc_drvdata *pdata = dev_get_drvdata(dev);

	if (!pdata)
		return 0;

	n = sprintf(str, "Video Memory Informations\n");
	strcpy(&buf[i], str);
	i += n;
	n = sprintf(str, "  - base       : 0x%08x\n", pdata->mem.pbase);
	strcpy(&buf[i], str);
	i += n;
	n = sprintf(str, "  - size       : %d Kb\n",
		    pdata->mem.size >> 10);
	strcpy(&buf[i], str);
	i += n;
	n = sprintf(str, "  - mmapped at : 0x%p\n", pdata->mem.vbase);
	strcpy(&buf[i], str);
	i += n;

	n = sprintf(str, "Register Memory Informations\n");
	strcpy(&buf[i], str);
	i += n;
	n = sprintf(str, "  - base       : 0x%08x\n", pdata->reg.pbase);
	strcpy(&buf[i], str);
	i += n;
	n = sprintf(str, "  - size       : %d Kb\n",
		    pdata->reg.size >> 10);
	strcpy(&buf[i], str);
	i += n;
	n = sprintf(str, "  - mmapped at : 0x%p\n", pdata->reg.vbase);
	strcpy(&buf[i], str);
	i += n;

	return i;
}

static DEVICE_ATTR(mem, S_IRUSR, dcc_sys_mem_show, NULL);
/**
 * Frame update request number
 */
static ssize_t dcc_sys_frame_update_number_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct timeval timestamp;
	struct dcc_drvdata *pdata = dev_get_drvdata(dev);

	if (!pdata)
		return 0;

	measdelay_start(&timestamp);

	return sprintf(buf, "%d t:%ld.%06ld\n",
			pdata->debug.frame_update_number,
			timestamp.tv_sec, timestamp.tv_usec);

}

static ssize_t dcc_sys_frame_update_number_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int ret;
	struct dcc_drvdata *pdata = dev_get_drvdata(dev);

	if (!pdata)
		return 0;

	ret = sscanf(buf, "%u", &pdata->debug.frame_update_number);
	if (!ret)
		return 0;

	return count;
}

static DEVICE_ATTR(frame_update_number, S_IRUGO | S_IWUSR,
			dcc_sys_frame_update_number_show,
			dcc_sys_frame_update_number_store);


static ssize_t dcc_sys_dif_rate_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	int rate;
	struct dcc_drvdata *pdata = dev_get_drvdata(dev);

	if (!pdata)
		return 0;

	rate = pdata->display.get_rate(&pdata->display);
	return sprintf(buf, "Display interface rate : %d\n", rate);
}

static ssize_t dcc_sys_dif_rate_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{

	struct dcc_drvdata *pdata = dev_get_drvdata(dev);
	int rate;

	if (!pdata)
		return 0;

	sscanf(buf, "%d", &rate);
	pdata->display.set_rate(&pdata->display, rate);

	return count;
}

static DEVICE_ATTR(dif_rate, S_IWUSR | S_IRUSR, dcc_sys_dif_rate_show,
			 dcc_sys_dif_rate_store);

static ssize_t dcc_sys_display_power_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{

	struct dcc_drvdata *pdata = dev_get_drvdata(dev);
	int en;

	if (!pdata)
		return 0;

	sscanf(buf, "%d", &en);

	if (en)
		pdata->display.power_on(&pdata->display);
	else
		pdata->display.power_off(&pdata->display);

	return count;
}

static DEVICE_ATTR(display_power, S_IWUSR, NULL, dcc_sys_display_power_store);


static ssize_t dcc_sys_display_sleep_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{

	struct dcc_drvdata *pdata = dev_get_drvdata(dev);
	int en;

	if (!pdata)
		return 0;

	sscanf(buf, "%d", &en);

	if (en)
		pdata->display.sleep_in(&pdata->display);
	else
		pdata->display.sleep_out(&pdata->display);

	return count;
}

static DEVICE_ATTR(display_sleep, S_IWUSR, NULL, dcc_sys_display_sleep_store);

#define DCC_SYSFS_GET_SET_VAL(_name_, _var_) \
static ssize_t dcc_sys_##_name_##_show(struct device *dev, \
				     struct device_attribute *attr, \
				     char *buf) \
{ \
	struct dcc_drvdata *pdata = dev_get_drvdata(dev); \
\
	if (!pdata) \
		return 0; \
\
	return sprintf(buf, "%d\n", _var_); \
} \
\
static ssize_t dcc_sys_##_name_##_store(struct device *dev, \
				      struct device_attribute *attr, \
				      const char *buf, size_t count) \
{ \
	struct dcc_drvdata *pdata = dev_get_drvdata(dev); \
\
	if (!pdata) \
		return 0; \
\
	sscanf(buf, "%d", &_var_); \
	return count; \
} \
\
static DEVICE_ATTR(_name_, S_IWUSR | S_IRUSR, dcc_sys_##_name_##_show, \
			 dcc_sys_##_name_##_store);

#define DCC_SYSFS_GET_SET_LLVAL(_name_, _var_) \
static ssize_t dcc_sys_##_name_##_show(struct device *dev, \
				     struct device_attribute *attr, \
				     char *buf) \
{ \
	struct dcc_drvdata *pdata = dev_get_drvdata(dev); \
\
	if (!pdata) \
		return 0; \
\
	return sprintf(buf, "%lli\n", _var_); \
} \
\
static ssize_t dcc_sys_##_name_##_store(struct device *dev, \
				      struct device_attribute *attr, \
				      const char *buf, size_t count) \
{ \
	struct dcc_drvdata *pdata = dev_get_drvdata(dev); \
\
	if (!pdata) \
		return 0; \
\
	sscanf(buf, "%lli", &_var_); \
	return count; \
} \
\
static DEVICE_ATTR(_name_, S_IWUSR | S_IRUSR, dcc_sys_##_name_##_show, \
			 dcc_sys_##_name_##_store);


/* debug verbosity level */
DCC_SYSFS_GET_SET_VAL(dbglevel, pdata->debug.level)
/* enable vsync */
DCC_SYSFS_GET_SET_VAL(use_fences, pdata->use_fences)
DCC_SYSFS_GET_SET_LLVAL(vsync_us, pdata->vsync_us)


static ssize_t dcc_sys_vsyncts0_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct dcc_drvdata *pdata = dev_get_drvdata(dev);

	if (!pdata)
		return 0;

	return sprintf(buf, "%llu\n", pdata->vsync_ts);
}

static DEVICE_ATTR(vsyncts0, S_IRUGO, dcc_sys_vsyncts0_show,
			 NULL);

static ssize_t dcc_sys_enable_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct dcc_drvdata *pdata = dev_get_drvdata(dev);

	if (!pdata)
		return 0;

	return sprintf(buf, "%d\n",
			(pdata->drv_state == DRV_DCC_SUSPENDED) ? 1 : 0);
}

static ssize_t dcc_sys_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int en;
	struct dcc_drvdata *pdata = dev_get_drvdata(dev);

	if (!pdata)
		return 0;

	sscanf(buf, "%d", &en);

	if (en)
		pdata->drv_resume(dev);
	else
		pdata->drv_suspend(dev);

	return count;
}
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		dcc_sys_enable_show, dcc_sys_enable_store);


static struct device_attribute *dcc_attrs[] = {
	&dev_attr_hw,
	&dev_attr_mem,
	&dev_attr_dbglevel,
	&dev_attr_frame_update_number,
	&dev_attr_use_fences,
	&dev_attr_dif_rate,
	&dev_attr_display_power,
	&dev_attr_display_sleep,
	&dev_attr_vsyncts0,
	&dev_attr_vsync_us,
	&dev_attr_enable,
	NULL
};


/**
 * MIPI-DSI phy settings
 */

#define KOBJ_ATTR(_name_, _p_, _show_, _store_) \
const struct kobj_attribute attr_##_name_ = __ATTR(_name_, _p_, \
		_show_,	_store_)

#define MIPIDSI_PHY_CBS(_name_) \
static ssize_t dcc_sys_##_name_##_show(struct kobject *kobj, \
					struct kobj_attribute *attr, \
					char *buf) \
{ \
	int i, val = 0; \
	struct dcc_drvdata *pdata = gradata; \
	if (DISPLAY_IS_MIPI_DSI_IF(pdata->display.dif.type)) \
		val = pdata->display.dif.u.dsi._name_; \
	i = sprintf(buf, #_name_" : %d\n", val); \
	return i; \
} \
\
static ssize_t dcc_sys_##_name_##_store(struct kobject *kobj, \
					struct kobj_attribute *attr, \
					const char *buf, size_t count) \
{ \
	struct dcc_drvdata *pdata = gradata; \
	int val; \
\
	sscanf(buf, "%d", &val); \
	if (DISPLAY_IS_MIPI_DSI_IF(pdata->display.dif.type)) { \
		pdata->display.dif.u.dsi._name_ = val; \
		dcc_dsi_set_phy_lock(&pdata->display); \
	} \
	return count; \
} \
\
KOBJ_ATTR(_name_, S_IWUSR | S_IRUSR, \
		dcc_sys_##_name_##_show, dcc_sys_##_name_##_store)


MIPIDSI_PHY_CBS(share);
MIPIDSI_PHY_CBS(m);
MIPIDSI_PHY_CBS(n);
MIPIDSI_PHY_CBS(pwup);
MIPIDSI_PHY_CBS(calib);
MIPIDSI_PHY_CBS(to_lp_hs_req);
MIPIDSI_PHY_CBS(to_lp_hs_dis);
MIPIDSI_PHY_CBS(to_lp_hs_eot);
MIPIDSI_PHY_CBS(to_hs_zero);
MIPIDSI_PHY_CBS(to_hs_flip);
MIPIDSI_PHY_CBS(lp_clk_div);
MIPIDSI_PHY_CBS(to_hs_clk_pre);
MIPIDSI_PHY_CBS(to_hs_clk_post);
MIPIDSI_PHY_CBS(data_delay);
MIPIDSI_PHY_CBS(clock_delay);
MIPIDSI_PHY_CBS(lp_tx_tfall);
MIPIDSI_PHY_CBS(en);
MIPIDSI_PHY_CBS(lp_tx_trise);
MIPIDSI_PHY_CBS(lp_tx_vref);


const struct attribute *phy_attrs[] = {
	&attr_share.attr,
	&attr_m.attr,
	&attr_n.attr,
	&attr_calib.attr,
	&attr_pwup.attr,
	&attr_to_lp_hs_req.attr,
	&attr_to_lp_hs_dis.attr,
	&attr_to_lp_hs_eot.attr,
	&attr_to_hs_zero.attr,
	&attr_to_hs_flip.attr,
	&attr_lp_clk_div.attr,
	&attr_to_hs_clk_pre.attr,
	&attr_to_hs_clk_post.attr,
	&attr_data_delay.attr,
	&attr_clock_delay.attr,
	&attr_lp_tx_tfall.attr,
	&attr_en.attr,
	&attr_lp_tx_trise.attr,
	&attr_lp_tx_vref.attr,
	NULL
};



/**
 * Sysfs init
 */
int dcc_sysfs_create(struct device *dev)
{
	int i;
	struct dcc_drvdata *pdata = dev_get_drvdata(dev);

	if (!pdata)
		return -EINVAL;

	DCC_DBG2("sysfs initialization\n");

	/* device root directory */
	i = 0;
	while (dcc_attrs[i] != NULL) {
		if (device_create_file(dev, dcc_attrs[i++]))
			return -ENOMEM;
	}

	/* mipidsi-phy directory */
	pdata->kobj_mipidsi_phy =
		kobject_create_and_add("mipidsi-phy", &dev->kobj);
	if (!pdata->kobj_mipidsi_phy)
		return -ENOMEM;

	i = 0;
	while (phy_attrs[i] != NULL) {
		if (sysfs_create_file(pdata->kobj_mipidsi_phy,
					phy_attrs[i++]))
			return -ENOMEM;
	}

	return 0;
}

void dcc_sysfs_delete(struct device *dev)
{
	int i = 0;

	while (dcc_attrs[i] != NULL)
		device_remove_file(dev, dcc_attrs[i++]);
}
