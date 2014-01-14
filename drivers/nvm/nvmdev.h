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

#ifndef _NVM_DEV_H
#define _NVM_DEV_H

#include "bastypes.h"
#include "nvmdev_ioctl.h"

#define DRIVER_DESC "NVM Kernel Module"

#if defined(CONFIG_NVM_DEBUG)
    #define INFO(...)   printk(KERN_INFO "[nvmdev driver] " __VA_ARGS__);
    #define ALERT(...)  printk(KERN_ALERT"[nvmdev driver] " __VA_ARGS__);
#else
    #define INFO(...)
    #define ALERT(...)
#endif  /* NVM_KERNEL_DEBUG */


#define SUCCESS  0
#define FAILURE -1

/*
 * NVM group info structure used to maintain nvm groups.
 */
typedef struct {
    volatile U32   update_count;   /* No. of user updates */
    volatile S32            ready_to_read;  /* Indicate the group is ready for read */
} T_NVMDEV_GROUP_INFO;


/*
 * NVM kernel device system call implementations.
 */
static S32 nvmdev_open(struct inode *, struct file *);                          /* Open system call prototype */
static S32 nvmdev_release(struct inode *, struct file *);                       /* Release system call prototype */
static ssize_t nvmdev_read(struct file *, char *, size_t, loff_t *);            /* Not supported !! */
static ssize_t nvmdev_write(struct file *, const char *, size_t, loff_t *);     /* Not supported !! */
static long nvmdev_ioctl(struct file *, U32, unsigned long);           /* Ioctl system call prototype */

#endif /* _NVM_DEV_H */
