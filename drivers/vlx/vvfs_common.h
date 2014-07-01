 /*
  ****************************************************************
  *
  *  Component: VirtualLogix VVFS-BE driver
  *
  *  Copyright (C) 2012 - 2013 Intel Mobile Communications GmbH
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

#ifndef VVFS_COMMON_H
#define VVFS_COMMON_H

    /* Defines */

#define NKDEV_VFS_NAME_LIMIT            28

#define NKDEV_VFS_DEVID(major,minor)    (((major) << 8) | (minor))
#define NKDEV_VFS_DEVID_MAJOR(devid)    ((devid) >> 8)
#define NKDEV_VFS_DEVID_MINOR(devid)    ((devid) & 0xff)

    /* Link Configuration
     * 1) RX config on one side has to be same as TX config on other side
     * 2) Message count has to be > 0 & even number
     */

#define NKDEV_VFS_TX_MSG_COUNT          4
#define NKDEV_VFS_TX_DATA_COUNT         0
#define NKDEV_VFS_TX_DATA_MAX           0

#define NKDEV_VFS_RX_MSG_COUNT          4
#define NKDEV_VFS_RX_DATA_COUNT         4
#define NKDEV_VFS_RX_DATA_MAX           512

    /* Operation Code - "op" field */

#define NKDEV_VFS_OP_UNUSED             0
#define NKDEV_VFS_OP_READ_DATA          1
#define NKDEV_VFS_OP_WRITE_DATA         2
#define NKDEV_VFS_OP_ERASE_DATA         3
        /* VLX-specific options */
#define NKDEV_VFS_OP_GET_DEVICE_ID      4
#define NKDEV_VFS_OP_DEVICE_INFO        5
#define NKDEV_VFS_OP_DEVICE_DOWN        6
#define NKDEV_VFS_OP_PRINT_STATE        7
#define NKDEV_VFS_OP_TESTSUITE          8
#define NKDEV_VFS_OP_MAX                9   /* last+1 */

#define NKDEV_VFS_OP_IS_STANDARD(x) ((x) < NKDEV_VFS_OP_GET_DEVICE_ID)

#define NKDEV_VFS_OP_NAMES \
        "UNUSED", "READ_DATA", "WRITE_DATA", \
        "ERASE_DATA", "GET_DEVICE_ID", "DEVICE_INFO", \
        "DEVICE_DOWN", "PRINT_STATE", "TESTSUITE"

       /* Sub-ops for OP_TESTSUITE */
#define NKDEV_VFS_TEST_GET_STATUS        1
#define NKDEV_VFS_TEST_DEV_PROBE         2
#define NKDEV_VFS_TEST_DEV_INIT          3
#define NKDEV_VFS_TEST_DEV_DOWN          4

       /* Bits for the "flags" field */
#define NKDEV_VFS_FLAGS_REQUEST          0x1
#define NKDEV_VFS_FLAGS_REPLY            0x2
#define NKDEV_VFS_FLAGS_NOTIFICATION     0x4

   /*
     * In general, in shared memory, all fields must be aligned on their
     * size. The structure should also be padded up to the size of the
     * largest member.
     */

    /* 32-bit alignment */
typedef struct {
    nku8_f      name [NKDEV_VFS_NAME_LIMIT];
    nku32_f     devid;
    nku32_f     size;
} nkdev_vfs_dev_info;

    /* 64-bit alignment */
typedef struct {
    nku32_f op;         /* NKDEV_VFS_OP_... */
    nku32_f flags;      /* NKDEV_VFS_FLAGS_... */
    nku32_f dataOffset; /* offset into communications area */
    nku32_f isData;     /* data present */
    nku64_f cookie;     /* nkdev_vfs_request::cookie, opaque for backend */

    /* Above members & order are similar as in reply */

    nku32_f addr;
    nku32_f size;
} nkdev_vfs_request;

    /* 64-bit alignment */
typedef struct {
    nku32_f op;         /* NKDEV_VFS_OP_... */
    nku32_f flags;      /* NKDEV_VFS_FLAGS_... */
    nku32_f dataOffset; /* offset into communications area */
    nku32_f isData;     /* data present */
    nku64_f cookie;     /* nkdev_vfs_request::cookie, opaque for backend */

    /* Above members & order are similar as in request */

    nku32_f retcode;    /* 0 if success, errno code otherwise */
    nku32_f retvalue;
} nkdev_vfs_reply;

typedef union {
    nkdev_vfs_request req;
    nkdev_vfs_reply   reply;
} nkdev_vfs_msg;

#endif /* VVFS_COMMON_H */

