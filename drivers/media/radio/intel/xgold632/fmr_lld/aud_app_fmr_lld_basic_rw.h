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
**				INTERFACE DESCRIPTION
**
** ============================================================================
*/
/*
 * @file aud_app_fmr_lld_basic_rw.h
 *
 * This file defines the API necessary for accessing FMR register/memory
 * location.
 *
 */

#ifndef _AUD_APP_FMR_LLD_BASIC_HW_H_
#define _AUD_APP_FMR_LLD_BASIC_HW_H_

/*
** ============================================================================
**
**				INCLUDE STATEMENTS
**
** ============================================================================
*/
#include <types.h>

/*
** ============================================================================
**
**				DEFINES
**
** ============================================================================
*/

/*
** ============================================================================
**
**				DEFINES
**
** ============================================================================
*/
/*
** ============================================================================
**
**			   EXPORTED FUNCTION DECLARATIONS
**
** ============================================================================
*/

/*
 * Reads a 16-bit value from FMR memory with effective address
 * (FMR_BASE + addroffs).
 *
 * @param  addroffs	Offset address of 16-bit FMR memory region
 *
 * @return u16	 16-bit value read from given location
 */
u16 fmtrx_read16(u16 addroffs);

/*
 * Reads a 32-bit value from FMR memory with effective address
 * (FMR_BASE + addroffs).
 *
 * @param  addroffs	Offset address of 16-bit FMR memory region
 *
 * @return u32	 32-bit value read from given location
 */
u32 fmtrx_read32(u16 addroffs);

/*
 * Reads a 32-bit value from FMR memory with effective address
 * (FMR_BASE + addroffs).The value read from memory is AND'd with the mask.
 *
 * @param  addroffs	Offset address of 16-bit FMR memory region
 * @param  mask		Mask pattern to 'AND' with the read value
 *
 * @return u32	 32-bit value read from given location
 */
u32 fmtrx_read32_masked(u16 addroffs, u32 mask);

/*
 * Reads a bit field from a 32-bit value fetched from FMR memory region with
 * effective address  (FMR_BASE + addroffs).
 *
 * @param  addroffs	Offset address of 16-bit FMR memory region
 * @param  pos		Bit position
 * @param  width	Width of bit-field to read
 *
 * @return u32	 Bit-field value read
 */
u32 fmtrx_read_field(u16 addroffs, u16 pos, u16 width);

/*
 * Reads a bit from a 32-bit value fetched from FMR memory region with
 * effective address (FMR_BASE + addroffs).
 *
 * @param  addroffs	Offset address of 16-bit FMR memory region
 * @param  pos		Bit position
 *
 * @return u32	 Bit value read
 */
u32 fmtrx_read_bit(u16 addroffs, u16 pos);

/*
 * Writes a 16-bit value to a effective address (FMR_BASE + addroffs) in FMR
 * memory.
 *
 * @param  addroffs	Offset address of 16-bit FMR memory region
 * @param  data		16-bit value to write
 *
 * @return void
 */
void fmtrx_write16(u16 addroffs, u16 data);

/*
 * Writes a 32-bit value to a effective address (FMR_BASE + addroffs) in FMR
 * memory.
 *
 * @param  addroffs	Offset address of 16-bit FMR memory region
 * @param  data		32-bit value to write
 *
 * @return void
 */
void fmtrx_write32(u16 addroffs, u32 data);

/*
 * Writes masked data to FMR address space (Read-modify-write).
 *
 * @param  addroffs	Offset address of 16-bit FMR memory region
 * @param  data		32-bit value to write
 * @param  mask		32-bit mask to apply
 *
 * @return void
 */
void fmtrx_write32_masked(u16 addroffs, u32 data, u32 mask);

/*
 * Writes a bit field to FMR memory region with effective address (FMR_BASE +
 * addroffs).
 *
 * @param  addroffs	 Offset address of 16-bit FMR memory region
 * @param  pos		 Bit position
 * @param  width	 Width of bit-field to write
 * @param  value	 32-bit value to write
 *
 * @return none
 */
void fmtrx_write_field(u16 addroffs, u16 pos, u16 width, u32 value);

/*
  * Writes a bit to FMR memory region with effective address (FMR_BASE +
  * addroffs).
  *
  * @param  addroffs	  Offset address of 16-bit FMR memory region
  * @param  pos		  Bit position
  * @param  value	  32-bit value to write
  *
  * @return none
*/
void fmtrx_write_bit(u16 addroffs, u16 pos, u32 value);

/*
 * Internal memory copy
 *
 * @param  dst		    Destination memory location
 * @param  src		    Source memory location
 * @param  size		    Size of memory region to copy
 *
 * @return void
 */
void fmtrx_memcpy(void *dst, const void *src, u32 size);

/*
 * Internal memory set
 *
 * @param  mem		    Pointer to a memory location
 * @param  val		    Value to set
 * @param  size		    Size of memory region to set
 *
 * @return void
 */
void fmtrx_memset(void *mem, u8 val, u32 size);

#endif	/* _AUD_APP_FMR_LLD_BASIC_HW_H_ */

