/*
 * Copyright (C) 2013, 2014 Intel Mobile Communications GmbH
 *
 * Notes:
 * Aug 22 2014: IMC: mvpipe interface to secure vm
 * Oct 30 2014: IMC: += vrga_ping()
 * Nov 10 2014: IMC: add type & commands for secure memory allocator
 */

/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#ifndef _VRGA_FE_H_
#define _VRGA_FE_H_

/*
 * max number if integers in secvm command
 */
#define VRGA_SECVM_CMD_LEN 128

/*
 * encapsulate a secure vm command
 */
struct vrga_secvm_cmd {
	unsigned int payload[VRGA_SECVM_CMD_LEN];
};

/*
 * first handshake with vrga on secure VM, for debug
 */
#define VRGA_VTYPE_INIT_HANDSHAKE 0x1000
#define VRGA_VOP_INIT_HANDSHAKE	  0x1000
#define VRGA_CNF_INIT_HANDSHAKE	  0xFAB00000

#define VRGA_VTYPE_REG			0x2000
#define VRGA_VOP_REG_WRITE		5
#define VRGA_VOP_REG_READ		6
#define VRGA_VOP_CLR_INT		7

#define VRGA_VTYPE_MEM		  0x3000
#define VRGA_VOP_MEM_ALLOC	  37
#define VRGA_VOP_MEM_FREE	  38


/*
 * initialize vrga connection to the secure vm
 */
int vrga_fe_init(struct device *dev);

/*
 * release vrga connection to the secure vm
 */
int vrga_fe_release(struct device *dev);

/*
 * send a vrga command to secure vm and wait for response
 */
int vrga_call(struct device *dev, struct vrga_secvm_cmd *cmd_p);

/*
 * send a ping request to the secure vm which immediately returns
 * the payload cmd is for future use
 * (takes the turn around time on the kernel log)
 */
int vrga_ping(struct device *dev, uint32_t cmd);


#endif /* _VRGA_FE_H_ */
