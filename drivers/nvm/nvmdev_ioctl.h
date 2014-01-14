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

#ifndef _NVM_DEV_IOCTL_H
#define _NVM_DEV_IOCTL_H

/* Not required for kernel space */
#if !defined CONFIG_NVM
#include <fcntl.h>      /* For O_RDWR */
#include <unistd.h>     /* For open(), creat() */
#include <sys/ioctl.h>
#endif

#define DEVICE_PREFIX		"/dev"
#define DEVICE_NAME 		"nvmdev"
//#define NVM_DEVICE_NAME 	DEVICE_PREFIX##DEVICE_NAME
#define NVM_DEVICE_NAME     "/dev/nvmdev"

#define IOC_MAGIC 'k'

/* ioctl commands */

#define NVM_COPY_TO_KERNEL_MIRROR        _IOW(IOC_MAGIC, 4, int)
#define NVM_UA_WRITE_TO_FLASH            _IOW(IOC_MAGIC, 5, int)
#define NVM_DEBUG                        _IOW(IOC_MAGIC, 6, int)
#define NVM_KERNEL_MIRROR_UPDATE         _IOW(IOC_MAGIC, 7, int)
#define NVM_FACTORY_DEFAULT              _IOW(IOC_MAGIC, 8, int)
#define NVM_CHANGE_DEV_STATE             _IOW(IOC_MAGIC, 9, int)

/* No of group update in single user-kernel switch */
#define UPDATE_LIMIT    1

/*
 * NVM device ioctl structure
 */
 #define NVM_MAX_GROUP_SIZE      102400

 typedef struct t_nvmdev_ioctl {
    unsigned int  group_id;                        /* Group ID */
    unsigned int  offset;                          /* Offset of group data */
    unsigned int  no_of_bytes;                     /* No. of bytes */
    unsigned int  valid;                           /* Flag defines whether it contains valid data or not */
    unsigned int  version;
    unsigned int  revision;
    unsigned int  length;
    int           device_initialing;
    unsigned char data_ptr[NVM_MAX_GROUP_SIZE];    /* User has to fill the data pointer */
} T_NVMDEV_IOCTL;

#endif /* _NVM_DEV_IOCTL_H */