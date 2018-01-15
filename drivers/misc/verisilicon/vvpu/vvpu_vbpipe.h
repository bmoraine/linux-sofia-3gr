/*
 * Copyright (C) 2013, 2014 Intel Mobile Communications GmbH
 *
 * Notes:
 * Aug 22 2014: IMC: vbpipe interface to secure vm
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *					 Boston, MA  02110-1301, USA.
 */

#ifndef _VVPU_VBPIPE_H_
#define _VVPU_VBPIPE_H_

/*
 * max number if integers in secvm command
 */
#define VVPU_SECVM_CMD_LEN 128

/*
 * encapsulate a secure vm command
 */
struct vvpu_secvm_cmd {
	unsigned int payload[VVPU_SECVM_CMD_LEN];
};

/*
 * first handshake with vvpu on secure VM, for debug
 */
#define VVPU_VTYPE_INIT_HANDSHAKE 0x1000
#define VVPU_VTYPE_DEC		  0x1001
#define VVPU_VTYPE_ENC		  0x1002
#define VVPU_VOP_INIT_HANDSHAKE	  0x1000
#define VVPU_CNF_INIT_HANDSHAKE	  0xFAB00000
#define VVPU_VOP_INIT_PROBE	  0x2000


/*
 * initialize vvpu connection to the secure vm
 */
int vvpu_vbpipe_init(struct device *dev);

/*
 * release vvpu connection to the secure vm
 */
int vvpu_vbpipe_release(struct device *dev);

/*
 * send a vvpu command to secure vm and wait for response
 */
int vvpu_call(struct device *dev, struct vvpu_secvm_cmd *cmd_p);


#endif /* _VVPU_VBPIPE_H_ */
