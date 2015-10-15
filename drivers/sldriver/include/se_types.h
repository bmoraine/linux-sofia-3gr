/*
 * Copyright (c) 2015, Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#pragma once

#ifndef _SE_TYPE_H_
#define _SE_TYPE_H_

#if defined(SE_64)

#define	PADDED_POINTER(t, p)	(t* p)
#define	PADDED_DWORD(d)		uint64_t d
#define	PADDED_LONG(l)		int64_t l
#else /* !defined(SE_64) */

#define	PADDED_POINTER(t, p) t* p;       void    *___##p##_pad_to64_bit
#define	PADDED_DWORD(d)      uint32_t d; uint32_t ___##d##_pad_to64_bit
#define	PADDED_LONG(l)       int32_t l;  int32_t  ___##l##_pad_to64_bit
#endif /* !defined(SE_64) */

#endif
