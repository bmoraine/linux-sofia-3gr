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

#ifdef CONFIG_CIF_ISP20_REG_TRACE
int
cif_isp20_pltfrm_rtrace_printf(
	struct device *dev,
	const char *fmt,
	...);

int
cif_isp20_pltfrm_ftrace_printf(
	struct device *dev,
	const char *fmt,
	...);

#else
#define cif_isp20_pltfrm_rtrace_printf(dev, str, ...)
#define cif_isp20_pltfrm_ftrace_printf(dev, str, ...)
#endif

#define cif_isp20_pltfrm_pr_dbg(dev, fmt, arg...) \
	do { \
		pr_debug_ratelimited("CIF ISP2.0 %s: " fmt, \
			__func__, ## arg); \
		cif_isp20_pltfrm_ftrace_printf(dev, "%s: " fmt, \
			__func__, ## arg); \
	} while (0)
#define cif_isp20_pltfrm_pr_info(dev, fmt, arg...) \
	do { \
		pr_info_ratelimited("CIF ISP2.0 %s: " fmt, \
			__func__, ## arg); \
		cif_isp20_pltfrm_ftrace_printf(dev, "%s: " fmt, \
			__func__, ## arg); \
	} while (0)
#define cif_isp20_pltfrm_pr_warn(dev, fmt, arg...) \
	do { \
		pr_warn_ratelimited("CIF ISP2.0 %s WARN: " fmt, \
			__func__, ## arg); \
		cif_isp20_pltfrm_ftrace_printf(dev, "%s WARN: " fmt, \
			__func__, ## arg); \
	} while (0)
#define cif_isp20_pltfrm_pr_err(dev, fmt, arg...) \
	do { \
		pr_err_ratelimited("CIF ISP2.0 %s(%d) ERR: " fmt, \
			__func__, __LINE__, ## arg); \
		cif_isp20_pltfrm_ftrace_printf(dev, "%s(%d) ERR: " fmt, \
			__func__, __LINE__, ## arg); \
	} while (0)

void cif_isp20_pltfrm_write_reg(
	struct device *dev,
	u32 data,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr);

void cif_isp20_pltfrm_write_reg_OR(
	struct device *dev,
	u32 data,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr);

void cif_isp20_pltfrm_write_reg_AND(
	struct device *dev,
	u32 data,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr);

u32 cif_isp20_pltfrm_read_reg(
	struct device *dev,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr);

#define cif_iowrite32(d, a) \
	cif_isp20_pltfrm_write_reg(NULL, d, a);
#define cif_ioread32(a) \
	cif_isp20_pltfrm_read_reg(NULL, a)
#define cif_iowrite32OR(d, a) \
	cif_isp20_pltfrm_write_reg_OR(NULL, d, a)
#define cif_iowrite32AND(d, a) \
	cif_isp20_pltfrm_write_reg_AND(NULL, d, a)
/* BUG: Register write seems to fail sometimes w/o read before write. */
#define cif_iowrite32_verify(d, a, mask) \
	{ \
		unsigned i = 0; \
		do { \
			cif_iowrite32(d, a); \
			udelay(1); \
			if (i++ == 50) { \
				pr_err("Error in writing %x@0x%p, read %x\n", \
					(d) & (mask), a, ioread32(a)); \
					BUG(); \
			} \
		} while ((ioread32(a) & mask) != ((d) & mask)); \
	}
#define cif_iowrite32OR_verify(d, a, mask) \
	cif_iowrite32_verify((d) | cif_ioread32(a), a, mask)
#define cif_iowrite32AND_verify(d, a, mask) \
	cif_iowrite32_verify((d) & cif_ioread32(a), a, mask)

#define cif_isp20_pltfrm_event_init(_dev, _event) \
	init_waitqueue_head(_event)

#define cif_isp20_pltfrm_event_clear(_dev, _event)

#define cif_isp20_pltfrm_event_signal(_dev, _event) \
	wake_up_interruptible(_event)

#define cif_isp20_pltfrm_event_wait_timeout( \
	_dev, _event, _condition, _timeout_us) \
	wait_event_interruptible_timeout( \
		*(_event), _condition, (_timeout_us * HZ) / 1000000)

void
cif_isp20_pltfrm_debug_register_print_cb(
	struct device *dev,
	void (*print)(void *cntxt, const char *block_name),
	void *cntxt);

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

#endif
