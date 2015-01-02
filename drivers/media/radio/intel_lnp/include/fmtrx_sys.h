/*
 * Copyright (C) 2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
** =============================================================================
**
**				INTERFACE DESCRIPTION
**
** =============================================================================
*/
/**
 * @file fmtrx_sys.h
 *
 * This	file contains interfaces that are system related.
 *
 **/

#ifndef	_FM_TRX_SYS_H_
#define	_FM_TRX_SYS_H_

/*
** =============================================================================
**
**				INCLUDE STATEMENTS
**
** =============================================================================
*/
#include <linux/types.h>
#include <linux/version.h>
#include <linux/skbuff.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <asm-generic/errno-base.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <string.h>

#ifdef LD_DRIVER
#include "linux/lbf_ldisc.h"
#endif
/* Frequency manager interfaces */
#ifdef CONFIG_IUI_FM_FMR
#include <linux/fm/iui_fm.h>
#endif

/*
** =============================================================================
**
**				DEFINES
**
** =============================================================================
*/
#define u8 __u8
#define u16 __u16
#define u32 __u32
#define s8 __s8
#define s16 __s16
#define s32 __s32
#define true 1
#define false 0

#define ECHRNG 44
#define EALREADY 114

#define fmtrx_sys_log(fmt, arg...) \
		pr_debug("radio: fmr: "fmt, ##arg)

#define FILE (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

/*
** =============================================================================
**
**				EXPORTED ENUM DEFINITIONS
**
** =============================================================================
*/
enum fmtrx_type;
struct fmrx_config;

enum clk_source {
	CLK_SRC_MAINCLK,
	CLK_SRC_INTERNAL_RFCLK,
	CLK_SRC_INVALID
};

enum interrupt_vector {
	FMTRX_VECTOR_SWINT0,
	FMTRX_VECTOR_SWINT1,
	FMTRX_VECTOR_SWINT2,
	FMTRX_VECTOR_SWINT3,
	FMTRX_VECTOR_SWINT4,
	FMTRX_VECTOR_DED0,
	FMTRX_VECTOR_DED1,
	FMTRX_VECTOR_DED2,
	FMTRX_VECTOR_DED3,
	FMTRX_VECTOR_INVALID
};

enum bit_type {
	WIDTH_16BIT,
	WIDTH_32BIT,
	WIDTH_INVALID
};

enum component {
	COMPONENT_DCDC,
	COMPONENT_FM,
	COMPONENT_INVALID
};

struct dcdc_setting {
	u16 dcdc_div;
};
struct fm_setting {
	u32 frequency;
	u32 side;
};

union component_data {
	struct dcdc_setting dcdc_cfg;
	struct fm_setting fm_cfg;
};

/*
** =============================================================================
**
**					EXPORTED FUNCTION DECLARATIONS
**
** =============================================================================
*/
/* Read from a 16-bit register
 * @addr Address offset
 * @data Pointer to 16-bit value
 */
int fmtrx_sys_reg_read16(
		u32 addr_offs,
		u16 *data);

/* Read from a 32-bit register
 * @addr Address offset
 * @data Pointer to 32-bit value
 */
int fmtrx_sys_reg_read32(
		u32 addr_offs,
		u32 *data);

/* Write from a 16-bit register
 * @addr Address offset
 * @data Pointer to 16-bit value
 */
int fmtrx_sys_reg_write16(
		u32 addr_offs,
		u16 data);

/* Write from a 32-bit register
 * @addr Address offset
 * @data Pointer to 32-bit value
 */
int fmtrx_sys_reg_write32(
		u32 addr_offs,
		u32 data);

/* Write to a block of FMR IP memory
 * @addr_offs Address offset
 * @data Pointer to block of memory
 * @size Size of the block of memory
 */
int fmtrx_sys_mem_write(
		u32 addr_offs,
		const u8 *data,
		u32 size);

/* Read from a block of FMR IP memory
 * @addr_offs Address offset
 * @data Pointer to block of memory
 * @size Size of the block of memory
 */
int fmtrx_sys_mem_read(
		u32 addr_offs,
		u8 *data,
		u32 size);

/* Wait for the interrupt (in interrupt mode) or sleep for specified duration
 * @ms Timeout/Sleep in milli seconds
 */
int fmtrx_sys_wait_for_event(
		int ms);

/* Wake up the event
 */
int fmtrx_sys_wakeup_event(
		void);

/* Enable global interrupts
 */
int fmtrx_sys_irq_enable(
		void);

/* Disable global interrupts
 */
int fmtrx_sys_irq_disable(
		void);

/* Change clock source
 * @src Main or RF clock
 */
int fmtrx_sys_clk_sel(
		enum clk_source src);

/* Get FM TRX driver version
 * @drv_name Driver name
 * @card_name Type of chip
 */
int fmtrx_sys_get_driver_ver_info(
		u8 *drv_name,
		u8 *card_name);

/* Supply/release power/clocks to FMR IP
 */
int fmtrx_sys_power_enable(
		bool enable);

/* Enable/Disable host audio
 */
int fmtrx_sys_audio_enable(
		bool enable);

/* Frequency manager
 */
int fmtrx_sys_mitigate_interference(
		enum component type,
		union component_data *data);

/* Initializes the system level data structures
 */
int fmtrx_sys_init(
		void);

/* De-initializes the system level data structures
 */
int fmtrx_sys_deinit(
		void);

/* Fetch Firmware
 * @type FM RX or TX module
 * @data Pointer to the Firmware binary
 * @size Size of the Firmware binary
 */
int fmtrx_sys_fetch_fw(
		enum fmtrx_type type,
		const u8 **data,
		u32 *size);

/* Release Firmware
 */
int fmtrx_sys_release_fw(
		void);

/* Wait for 'ms' milli seconds
 * @ms Time in milliseconds
 */
void fmtrx_sys_idle_wait(
		u32 ms);

/* Get default RX configuration for this platform
 * @fmrx_cfg Pointer to the address of the configuration structure
 */
int fmtrx_sys_get_rx_default_config(
		struct fmrx_config **fmrx_cfg);

/* Print number of bytes received/sent to FMR IP
 */
void fmtrx_sys_log_traffic(
		void);

#endif	/* _FM_TRX_SYS_H_	*/
