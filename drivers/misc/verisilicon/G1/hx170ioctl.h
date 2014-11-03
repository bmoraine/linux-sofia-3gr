/*
 *
 * Copyright (C) 2013, 2014 Intel Mobile Communications GmbH
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
 * Notes:
 * Jun	1 2013: IMC: moved ioctl definitions from hx170dec.h
 * Aug 29 2013: IMC: semaphore and ioctl for reserve/release HW
 * Oct	9 2013: IMC: fix kernel code formatting issues
 * Mar 13 2014: IMC: Review Comments & Clean up
 */

/*
 * Decoder device driver (kernel module headers)
 *
 * Copyright (C) 2012 Google Finland Oy.
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
 *					     Boston, MA	 02110-1301, USA.
 *
 --------------------------------------------------------------------------*/

#ifndef _HX170IOCTL_H_
#define _HX170IOCTL_H_

#include <linux/ioctl.h>    /* needed for the _IOW etc stuff used later */

/*
 * Ioctl definitions
 */

struct pphwc_cmd {
	int release_fence_fd;
	uint32_t sync_value;
	uint64_t instance;
};

/* Use 'k' as magic number */
#define HX170DEC_IOC_MAGIC  'k'
/*
 * S means "Set" through a ptr,
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": G and S atomically
 * H means "sHift": T and Q atomically
 */

/* the client is pp instance */
#define HX170DEC_PP_INSTANCE	 _IO(HX170DEC_IOC_MAGIC, 1)
/* decode/pp time for HW performance */
#define HX170DEC_HW_PERFORMANCE	 _IO(HX170DEC_IOC_MAGIC, 2)
#define HX170DEC_IOCGHWOFFSET	 _IOR(HX170DEC_IOC_MAGIC,  3, unsigned long *)
#define HX170DEC_IOCGHWIOSIZE	 _IOR(HX170DEC_IOC_MAGIC,  4, unsigned int *)

#define HX170DEC_IOC_CLI	 _IO(HX170DEC_IOC_MAGIC,  5)
#define HX170DEC_IOC_STI	 _IO(HX170DEC_IOC_MAGIC,  6)

#define HX170DEC_IOC_PM_DISABLE	 _IO(HX170DEC_IOC_MAGIC,  7)
#define HX170DEC_IOC_PM_ULTRA	 _IO(HX170DEC_IOC_MAGIC,  8)

#define HX170DEC_IOCH_DEC_RESERVE _IO(HX170DEC_IOC_MAGIC, 9)
#define HX170DEC_IOCT_DEC_RELEASE _IO(HX170DEC_IOC_MAGIC, 10)
#define HX170DEC_IOCQ_PP_RESERVE  _IO(HX170DEC_IOC_MAGIC, 11)
#define HX170DEC_IOCT_PP_RELEASE  _IO(HX170DEC_IOC_MAGIC, 12)

#define HX170DEC_IOCT_SECVM_CMD	 _IOWR(HX170DEC_IOC_MAGIC, 13, \
		struct vvpu_secvm_cmd)

#define HX170DEC_IOCT_PPHWC_START _IOWR(HX170DEC_IOC_MAGIC, 14, \
		struct pphwc_cmd)
#define HX170DEC_IOCT_PPHWC_DONE  _IOWR(HX170DEC_IOC_MAGIC, 15, \
		struct pphwc_cmd)
#define HX170DEC_IOCT_PPHWC_RELEASE  _IOWR(HX170DEC_IOC_MAGIC, 16, \
		struct pphwc_cmd)

#define HX170DEC_IOC_MAXNR 16

/*
 * parameters for PM IOCTL
 */
#define HX170DEC_PM_DISABLE	   0
#define HX170DEC_PM_ULTRA_HIGH	   1


#endif	/* _HX170IOCTL_H_ */
