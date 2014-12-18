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
/* This file contains the basic read/write functionalities to FMR
 * register/memory area
 */

/*
** ============================================================================
**
**				INCLUDE STATEMENTS
**
** ============================================================================
*/
#include <string.h>	/* Prototypes for mem functions */
#include "aud_app_fmr_lld_basic_rw.h"
#include "aud_app_fmr_sys_os.h"

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
**				LOCAL FUNCTION DEFINITIONS
**
** ============================================================================
*/
#if !defined AUD_APP_HOST_TEST
/* Read a 16-bit value */
u16 fmtrx_read16(u16 addr_off)/* Offset address of 16-bit FMR memory region */
{
	u16 data = 0;

	fmr_sys_reg_read16((u32)addr_off, &data);

	return data;
}

/* Read a 32-bit value */
u32 fmtrx_read32(u16 addr_off)/* Offset address of 16-bit FMR memory region */
{
	u32 data = 0;

	fmr_sys_reg_read32((u32)addr_off, &data);

	return data;
}

/* Write a 16-bit value */
void fmtrx_write16(
	u16 addr_off,	/* Offset address of 16-bit FMR memory region */
	u16 data)	/* 16-bit value to write */
{
	fmr_sys_reg_write16((u32)addr_off, data);
}

/* Write a 32-bit value */
void fmtrx_write32(
	u16 addr_off,	/* Offset address of 16-bit FMR memory region */
	u32 data)	/* 32-bit value to write */
{
	fmr_sys_reg_write32((u32)addr_off, data);
}
#endif	/* !defined AUD_APP_HOST_TEST */

/* Read a 32-bit value and apply the given mask */
u32 fmtrx_read32_masked(
	u16 addr_off,	/* Offset address of 16-bit FMR memory region */
	u32 mask)	/* To mask the read 16-bit value */
{
	return fmtrx_read32(addr_off) & mask;
}

/* Read bit fields */
u32 fmtrx_read_field(
	u16 addr_off,	/* Offset address of 16-bit FMR memory region */
	u16 pos,	/* Bit field position */
	u16 width)	/* Width of bit field to read */
{
	return (fmtrx_read32(addr_off) >> pos) & ((1 << width) - 1);
}

/* Read a bit */
u32 fmtrx_read_bit(
	u16 addr_off,	/* Offset address of 16-bit FMR memory region */
	u16 pos)	/* Bit position */
{
	return fmtrx_read_field(addr_off, pos, 1);
}

/* Write a 32-bit value after applying the given mask */
void fmtrx_write32_masked(
	u16 addr_off,	/* Offset address of 16-bit FMR memory region */
	u32 data,	/* 32-bit data to write */
	u32 mask)	/* To mask the written data */
{
	fmtrx_write32(addr_off,
		      (fmtrx_read32(addr_off) & ~mask) | (data & mask));
}

/* Write a 32-bit value after applying the given mask */
void fmtrx_write_field(
	u16 addr_off,	/* Offset address of 16-bit FMR memory region */
	u16 pos,	/* Bit field position */
	u16 width,	/* Width of bit field to read */
	u32 value)	/* 32-bit value to write */
{
	u32 mask_tmp;
	u32 val_tmp;
	val_tmp = fmtrx_read32(addr_off);
	mask_tmp = ((1 << width) - 1) << pos;
	val_tmp = (val_tmp & (~mask_tmp)) | ((value << pos) & mask_tmp);
	fmtrx_write32(addr_off, val_tmp);
}

/* Write a 32-bit value after applying the given mask */
void fmtrx_write_bit(
	u16 addr_off,    /* Offset address of 16-bit FMR memory region */
	u16 pos,	 /* Bit position */
	u32 value)	 /* 32-bit value to write */
{
	fmtrx_write_field(addr_off, pos, 1, value);
}

/* Copy a block of memory from source to destination address */
void fmtrx_memcpy(void *dst, /* Destination memory location */
	const void *src,  /* Source memory location */
	u32 size)  /* Size of memory region to copy */
{
	memcpy(dst, src, size);
}

/* Set memory region to the given value */
void fmtrx_memset(void *mem,	/* Memory location to set */
	u8 val,			/* Value to set */
	u32 size)		/* Total memory size */
{
	memset(mem, val, size);
}

