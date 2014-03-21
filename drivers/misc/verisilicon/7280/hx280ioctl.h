/*
 * Copyright (C) 2013, 2014 Intel Mobile Communications GmbH
 *
 * Notes:
 * Jun	1 2013: IMC: extracted ioctl definitions from hx280enc.h
 * Mar 13 2014: IMC: Review Comments & Clean up
 */

/*
 * Encoder device driver (kernel module header)
 *
 * Copyright (C) 2011  On2 Technologies Finland Oy.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *						Boston, MA  02110-1301, USA.
 *
 ----------------------------------------------------------------------------
 --
 --  Version control information, please leave untouched.
 --
 --  $RCSfile: hx280enc.h,v $
 --  $Date: 2011/03/10 14:05:43 $
 --  $Revision: 1.1 $
 --
 --------------------------------------------------------------------------*/

#ifndef _HX280IOCTL_H_
#define _HX280IOCTL_H_

#include <linux/ioctl.h>    /* needed for the _IOW etc stuff used later */

/*
 * Ioctl definitions
 */

/* Use 'k' as magic number */
#define HX280ENC_IOC_MAGIC  'k'
/*
 * S means "Set" through a ptr,
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": G and S atomically
 * H means "sHift": T and Q atomically
 */
/*
 * #define HX280ENC_IOCGBUFBUSADDRESS _IOR(HX280ENC_IOC_MAGIC,	1, \
 *						       unsigned long *)
 * #define HX280ENC_IOCGBUFSIZE	      _IOR(HX280ENC_IOC_MAGIC,	2, \
 *							unsigned int *)
 */
#define HX280ENC_IOCGHWOFFSET	  _IOR(HX280ENC_IOC_MAGIC,  3, unsigned long *)
#define HX280ENC_IOCGHWIOSIZE	  _IOR(HX280ENC_IOC_MAGIC,  4, unsigned int *)
#define HX280ENC_IOC_CLI	  _IO(HX280ENC_IOC_MAGIC,   5)
#define HX280ENC_IOC_STI	  _IO(HX280ENC_IOC_MAGIC,   6)
#define HX280ENC_IOCXVIRT2BUS	  _IOWR(HX280ENC_IOC_MAGIC, 7, unsigned long *)

/* debugging tool */
#define HX280ENC_IOCHARDRESET	  _IO(HX280ENC_IOC_MAGIC,   8)

/* enable HW; PM */
#define HX280ENC_IOC_PM_DISABLE	  _IO(HX280ENC_IOC_MAGIC,   9)
/* enable HW; PM */
#define HX280ENC_IOC_PM_ULTRA	  _IO(HX280ENC_IOC_MAGIC,   10)

#define HX280ENC_IOC_RESERVE	  _IOW(HX280ENC_IOC_MAGIC,  11, unsigned int)
#define HX280ENC_IOC_RELEASE	  _IOW(HX280ENC_IOC_MAGIC,  12, unsigned int)

#define HX280_IOCT_SECVM_CMD	  _IOWR(HX280ENC_IOC_MAGIC, 13, \
		struct vvpu_secvm_cmd)

#define HX280ENC_IOC_MAXNR 13

/*
 * parameters for PM IOCTL
 */
#define HX280DEC_PM_DISABLE	   0
#define HX280DEC_PM_ULTRA_HIGH	   1



#endif /* !_HX280IOCTL_H_ */
