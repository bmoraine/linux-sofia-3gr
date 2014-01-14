/*
 ****************************************************************
 *
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
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
#ifndef BASTYPES_H
#define BASTYPES_H

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL    ((void *)0)
#endif

/*
 * Data types as defined by SDHB
 * =============================
 */
typedef signed char         S8;
typedef signed short        S16;
typedef unsigned char       U8;
typedef unsigned short      U16;
typedef signed int          S32;
typedef unsigned int        U32;
typedef int                 BOOL;               /* TRUE or FALSE */
typedef char                CHAR;               /* for char types */

#endif /* BASTYPES_H */