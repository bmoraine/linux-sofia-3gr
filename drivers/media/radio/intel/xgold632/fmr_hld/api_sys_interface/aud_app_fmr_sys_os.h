/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
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
** ============================================================================
**
**				MODULE DESCRIPTION
**
** ============================================================================
*/

/* @file
 *
 * FMR HLD API -  FMR System Interface
 *
 * Basic System Interface.
 */

#ifndef AUD_APP_FMR_SYS_OS_H
#define AUD_APP_FMR_SYS_OS_H

/*
** ============================================================================
**
**				INCLUDE STATEMENTS
**
** ============================================================================
*/

#include <linux/kernel.h>	/* sprintf */
#include <linux/string.h>	/* memcpy */
#include "aud_app_fmr_sys.h"



/* @defgroup SysOS Basic System and OS Interface
 * @brief
 * This section describes the abstraction layer for basic system and operating
 * system functionality required by the FMR driver.
 *
 */

/* @brief
  Pointer to the GSM timer callback function

  Currently no parameters are supported.
*/
typedef void (*fmtrx_sys_timer_cb)(void *args);

/*   @brief
  FMR driver request to wait specific micro seconds.

  @param	us  minimum time to wait in microseconds
*/
void fmr_sys_busy_wait(u32 us);

/*   @brief
  FMR driver request to wait specific milli seconds.

  System might choose to do sleep for this

  @param   ms  minimum time to sleep in milli seconds
*/
void fmr_sys_idle_wait(u32 ms);

/*  @brief
  Registers a function to be called by system after a given time.

  @param time_ms  time in ms after which the callback is called
  @param timer_cb   callback function to be called by system when time_ms
		elapsed
 */
void fmr_sys_timer_schedule_cb(u32 time_us, fmtrx_sys_timer_cb timer_cb);


/*  @brief
  FMR interrupt vector definition
*/
enum fmtrx_int_vec {
	FMR_INT_SWINT,		/* FMR SWINT shared interrupt */
	FMR_INT_DED0,		/* FMR 1st dedicated interrupt */
	FMR_INT_DED1,		/* FMR 2nd dedicated interrupt */
	FMR_INT_DED2,		/* FMR 3rd dedicated interrupt */
	FMR_INT_DED3		/* FMR 4th dedicated interrupt */
};

/*   @brief
  the type of the FMR interrupt handler function.

  @param  vec  Interrupt vector number enum.

  @see enum fmtrx_int_vec
*/
typedef void (*T_FMTRX_SYS_IRQ_HANDLER)(enum fmtrx_int_vec vec);

/*   @brief
  Registers a handler for interrupts from FMR macro

  @param  irq_handler  Interrupt handler.
 */
void fmr_sys_irq_register(T_FMTRX_SYS_IRQ_HANDLER irq_handler);

/* Allocate interrupt lines */
int fmr_sys_request_irq(void);

/* Release the allocated interrupt lines */
void fmr_sys_release_irq(void);

/*   @} */


/*  @ingroup SysOS
  @defgroup SysOtherHW Top Level HW Access
  @brief   HW related functions provided by system

  The FMR driver shall access all hardware registers outside of the FMR macro
  only by function calls to the OS abstration layer.

  This section contains functions to allow FMR driver to access hardware
  outside of the FMR macro.

  @{
*/

/*    @brief
  Clock source selection
 */
enum fmtrx_clk_src {
	FMR_CLK_SEL_RF = 3,   /* use FMR RF clock */
	FMR_CLK_SEL_PLL = 1   /* use FMR PLL clock */
};

/*   @brief
  Request 26 MHz system clock for FMR macro.

  Implements access to FSYSEN_FMR in clock control unit.

  @param   enable  if true, enables the FMR system clock, else disables it.
*/
void fmr_sys_clock_enable(s32 enable);

/*   @brief
  Selects the clock source.

  Implements access to SEL_CFMR register in clock control unit.

  @param   clk_src  the clock source
 */
void fmr_sys_clock_sel(enum fmtrx_clk_src clk_src);

/*   @brief
  Power up/down FMR macro.

  Implements access to FMRPWRDN  field in power management unit.

  @param   enable  if true, turns on power supply for FMR macro, else turn off
  @param   enable  if true, whether IDI bus is required, else turn off
*/
int fmr_sys_power_enable(bool enable, bool idi_bus_required);

/*   @brief
  Fetches FW.

  @param  fw_type   FMRX or FMTX
  @param  fw_data   FW binary data
  @param  fw_size   FW binary size
*/
int fmr_sys_fetch_fw(enum fmtrx_mod_type fw_type, const u8 **fw_data,
	u16 *fw_size);

/*   @brief
  Release resources held with fetch of firmware.

  @param  fw_type   FMRX or FMTX
*/
void fmr_sys_release_fw(enum fmtrx_mod_type fw_type);

/* @} */

/*  @ingroup SysOS
  @defgroup SysEventTrigger API Semaphore / Event Functions
  @brief   The interface to the FMR HLD thread is via an API event.

  The HLD thread can sleep until the API event has been triggered, if
  it has no active tasks.

  This would allow the switching to other threads during the settle
  down period of FMR measurements (e.g. oneshot RSSI & frequency tracking).

  @{
*/

/* @brief
  Trigger events.
*/
enum fmtrx_trigger_events {
	FMR_IR_CMD_DONE = 0,
	FMR_IR_CMD2_DONE = 1
};

/* Interrupt event ID */
enum fmtrx_int_ev {
	FMR_I_EV_RX_RSSI = 0,
	FMR_I_EV_RX_PILOT = 1,
	FMR_I_EV_RX_RDS = 2,
	FMR_I_EV_RX_RDS_SYNC = 3,
	FMR_I_EV_RX_RDS_FASTPI = 4,
	FMR_I_EV_TX_RDS  = 5,
	FMR_I_EV_TX_TRACE = 6,
	FMR_I_EV_RX_TRACE = 7,
	FMR_I_EV_ANT_TRACKING = 8,
	FMR_I_EV_INVALID
};

/*
*
* @brief Performs an asynchronous trigger of the events in event handler.
*
* Behaviour is undefined if there is already an event pending
*
* @param event_id Event ID to be triggered.
*/
void fmr_sys_trigger_event(enum fmtrx_trigger_events event_id);

/*
*
* @brief Waits until the specified event has been set.
*
* If no event has occured within a time-out interval (e.g. ~100ms),
* a hardware error is indicated.
*
* @note *not* expected to be thread safe. Only a single event may be
* outstanding at any time.
*
* @param event_id Event ID to be triggered.
*
**/
s32 fmr_sys_wait_for_event(enum fmtrx_trigger_events event_id);

/*
*
* @brief Send interrupt events to be processed out of interrupt context.
*
*/
void fmr_sys_int_event_send(const enum fmtrx_int_ev fmtrx_int_evt_id,
	const struct fmtrx_msg_params fmtrx_int_params);

/*  @ingroup SysOS
  @defgroup Remote memory and register access
  @brief   Access ABB memory and registers through IDI channel

  The FMR driver would use IDI DMA channel for memory section read/write,
  and programed IDI channel for register access.

  @{
*/

/* @brief Read 16bit FMR register

  Read 16bit value from the given register address.

  @note -# The system has to implement this as synchronous interface.

  @param   addr Register address
  @param   data Register data
 */
void fmr_sys_reg_read16(u32 addr, u16 *data);


/* @brief Read 32bit FMR register

  Read 32bit value from the given register address.

  @note -# The system has to implement this as synchronous interface.

  @param   addr Register address
  @param   data Register data
 */
void fmr_sys_reg_read32(u32 addr, u32 *data);


/* @brief Write 16bit FMR register
  @param   addr Register address
  @param   data Register data
 */
void fmr_sys_reg_write16(u32 addr, u16 data);


/* @brief Write 32bit FMR register
  @param   addr Register address
  @param   data Register data
*/
void fmr_sys_reg_write32(u32 addr, u32 data);

/* @brief Read a block of memory region in FMR
  @param  dst  Source address
  @param  src  FMR memory address
  @param  size Total memory size to read
*/
int fmr_sys_mem_read(void *dst, void *src, u32 size);


/* @brief Write to a block of memory region in FMR

  @param  dst  FMR memory address
  @param  src  Source address
  @param  size Total memory size to write
*/
int fmr_sys_mem_write(u8 *dst, u8 *src, u32 size);

/* @} */

#endif

