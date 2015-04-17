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

#ifndef VNVM_COMMON_H
#define VNVM_COMMON_H

    /* Defines */

#define VNVM_DEV_NAME_LIMIT            128

#define VNVM_DEVID(major,minor)    (((major) << 8) | (minor))
#define VNVM_DEVID_MAJOR(devid)    ((devid) >> 8)
#define VNVM_DEVID_MINOR(devid)    ((devid) & 0xff)

    /* Operation Code - "op" field */

#define VNVM_OP_UNUSED             0
#define VNVM_OP_READ_DATA          1
#define VNVM_OP_WRITE_DATA         2
#define VNVM_OP_ERASE_DATA         3
#define VNVM_OP_GET_DEVICE_ID      4
#define VNVM_OP_DEVICE_INFO        5
#define VNVM_OP_DEVICE_DOWN        6
#define VNVM_OP_PRINT_STATE        7
#define VNVM_OP_MAX                8   /* last+1 */

#define VNVM_OP_IS_STANDARD(x) ((x) < VNVM_OP_GET_DEVICE_ID)

#define VNVM_OP_NAMES \
        "UNUSED", "READ_DATA", "WRITE_DATA", \
        "ERASE_DATA", "GET_DEVICE_ID", "DEVICE_INFO", \
        "DEVICE_DOWN", "PRINT_STATE"

       /* Bits for the "flags" field */
#define VNVM_FLAGS_REQUEST          0x1
#define VNVM_FLAGS_REPLY            0x2
#define VNVM_FLAGS_NOTIFICATION     0x4
#define NVM_MAX_NB_PART 3

   /*
     * In general, in shared memory, all fields must be aligned on their
     * size. The structure should also be padded up to the size of the
     * largest member.
     */

    /* 32-bit alignment */
typedef struct {
    uint8_t      name [VNVM_DEV_NAME_LIMIT];
    uint32_t     devid;
    uint32_t     size;
} nvm_dev_info;

    /* 64-bit alignment */
typedef struct {
    uint32_t op;         /* VNVM_OP_... */
    uint32_t flags;      /* VNVM_FLAGS_... */
    uint64_t cookie;     /* vnvm_request::cookie, opaque for backend */
    uint32_t addr;
    uint32_t size;
    uint32_t part_id;
    uint32_t devsize[NVM_MAX_NB_PART];
    uint8_t data[0];
} nvm_request;

    /* 64-bit alignment */
typedef struct {
    uint32_t op;         /* VNVM_OP_... */
    uint32_t flags;      /* VNVM_FLAGS_... */
    uint64_t cookie;     /* vnvm_request::cookie, opaque for backend */
    uint32_t retcode;    /* 0 if success, errno code otherwise */
    uint32_t retvalue;   /* only used for 'simple' data return */
    uint32_t size;
    uint32_t part_id;
    uint32_t devsize[NVM_MAX_NB_PART];
    uint8_t data[0];
} nvm_reply;

#endif /* VNVM_COMMON_H */
