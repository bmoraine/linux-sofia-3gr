/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
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

#ifndef _PAL_COREDUMP_H
#define _PAL_COREDUMP_H

enum cd_dev {
	CD_DEV_DISABLED  = 0,
	CD_DEV_USIF1     = 1,
	CD_DEV_USIF2     = 2,
	CD_DEV_USIF3     = 3,
	CD_DEV_USBHS     = 4,
	CD_DEV_SDCARD    = 5,
	CD_DEV_MIPIPTI   = 6  /* Supports PTI1 and PTI2 */
};

struct cd_config {
	enum cd_dev device;
	int device_baud;
	enum cd_dev debug_device;
	int debug_device_baud;
	int usb_config;
	unsigned char failsafe_imei[8];
};

struct cd_ram {
	void *logical_start;
	void *physical_start;
	unsigned int length;
};

/* FUNCTION PROTOTYPES */

/*
    PAL Coredump driver initialization
*/
void pal_cd_init(void);

/*
    Start coredump
*/
void pal_cd_start(struct sys_vm_dump *vm_dump, struct sys_trap *trap);


#endif /* _PAL_COREDUMP_H*/
