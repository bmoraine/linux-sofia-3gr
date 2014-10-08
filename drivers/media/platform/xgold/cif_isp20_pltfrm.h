/*
 ****************************************************************
 *
 *  Intel CIF ISP 2.0 driver - Platform interface
 *
 *  Copyright (C) 2014 Intel Mobile GmbH
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
 * Note:
 *     07/07/2014: initial version.
 *
 ****************************************************************
 */

#ifndef _CIF_ISP20_PLTFRM_H
#define _CIF_ISP20_PLTFRM_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/string.h>

struct cif_isp20_strm_fmt;
struct cif_isp20_csi_config;
struct cif_isp20_device;
enum cif_isp20_pinctrl_state;
enum cif_isp20_inp;
enum cif_isp20_pm_state;
enum cif_isp20_irq;

#define CIF_ISP20_PLTFRM_DEVICE struct device *
#define CIF_ISP20_PLTFRM_MEM_IO_ADDR void __iomem *
#define CIF_ISP20_PLTFRM_EVENT wait_queue_head_t

#define cif_isp20_pltfrm_pr_dbg(dev, fmt, arg...) \
	pr_debug("CIF ISP2.0 %s: " fmt, \
		__func__, ## arg)
#define cif_isp20_pltfrm_pr_info(dev, fmt, arg...) \
	pr_info("CIF ISP2.0 %s: " fmt, \
		__func__, ## arg)
#define cif_isp20_pltfrm_pr_warn(dev, fmt, arg...) \
	pr_warn("CIF ISP2.0 %s WARN: " fmt, \
		__func__, ## arg)
#define cif_isp20_pltfrm_pr_err(dev, fmt, arg...) \
	pr_err("CIF ISP2.0 %s(%d) ERR: " fmt, \
		__func__, __LINE__, ## arg)

inline void cif_isp20_pltfrm_write_reg(
	struct device *dev,
	u32 data,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr);

inline void cif_isp20_pltfrm_write_reg_OR(
	struct device *dev,
	u32 data,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr);

inline void cif_isp20_pltfrm_write_reg_AND(
	struct device *dev,
	u32 data,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr);

inline u32 cif_isp20_pltfrm_read_reg(
	struct device *dev,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr);

#define cif_iowrite32(d, a) \
	cif_isp20_pltfrm_write_reg(NULL, d, a)
#define cif_ioread32(a) \
	cif_isp20_pltfrm_read_reg(NULL, a)
#define cif_iowrite32OR(d, a) \
	cif_isp20_pltfrm_write_reg_OR(NULL, d, a)
#define cif_iowrite32AND(d, a) \
	cif_isp20_pltfrm_write_reg_AND(NULL, d, a)

#ifdef CONFIG_CIF_ISP20_REG_TRACE
int
cif_isp20_pltfrm_reg_trace_printf(
	struct device *dev,
	const char *fmt,
	...);
#else
#define cif_isp20_pltfrm_reg_trace_printf(dev, str, ...)
#endif

int cif_isp20_pltfrm_dev_init(
	struct cif_isp20_device *cif_isp_dev,
	struct device **dev,
	void __iomem **reg_base_addr);

void cif_isp20_pltfrm_dev_release(
	struct device *dev);

int cif_isp20_pltfrm_pm_set_state(
	struct device *dev,
	enum cif_isp20_pm_state state,
	struct cif_isp20_strm_fmt *frm_fmt);

int cif_isp20_pltfrm_write_cif_ana_bandgap_bias(
	struct device *dev,
	u32 val);

int cif_isp20_pltfrm_pinctrl_set_state(
	struct device *dev,
	enum cif_isp20_pinctrl_state pinctrl_state);

struct device *cif_isp20_pltfrm_get_img_src_device(
	struct device *dev,
	enum cif_isp20_inp inp);

int cif_isp20_pltfrm_s_csi_config(
	struct device *dev,
	enum cif_isp20_inp inp,
	struct cif_isp20_strm_fmt *strm_fmt,
	struct cif_isp20_csi_config *csi_config);

int cif_isp20_pltfrm_g_csi_config(
	struct device *dev,
	enum cif_isp20_inp inp,
	struct cif_isp20_strm_fmt *frm_fmt,
	struct cif_isp20_csi_config *csi_config);

void cif_isp20_reset_csi_configs(
	struct device *dev,
	enum cif_isp20_inp inp);

int cif_isp20_pltfrm_irq_register_isr(
	struct device *dev,
	enum cif_isp20_irq irq,
	int (*isr)(void *cntxt),
	void *cntxt);

int (*cif_isp20_pltfrm_irq_g_isr(
	struct device *dev,
	enum cif_isp20_irq irq))(void *);

const char *cif_isp20_pltfrm_get_device_type(
	struct device *dev);

const char *cif_isp20_pltfrm_dev_string(
	struct device *dev);

void cif_isp20_pltfrm_event_init(
	struct device *dev,
	wait_queue_head_t *event);

void cif_isp20_pltfrm_event_clear(
	struct device *dev,
	wait_queue_head_t *event);

void cif_isp20_pltfrm_event_signal(
	struct device *dev,
	wait_queue_head_t *event);

int cif_isp20_pltfrm_event_wait_timeout(
	struct device *dev,
	wait_queue_head_t *event,
	bool condition,
	unsigned long timeout_us);

#endif
