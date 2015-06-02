
/*
 * rockchip vpu/hevc driver.
 *
 * Copyright (C) 2014-2015 Fuzhou Rockchip Electronics Co., Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _VVPU_PP_PIPE_H_
#define _VVPU_PP_PIPE_H_
#include <sofia/vvpu_vbpipe.h>

/*
 * initialize vvpu connection to the secure vm
 */
int vvpu_pp_vbpipe_init(struct device *dev);

/*
 * release vvpu connection to the secure vm
 */
int vvpu_pp_vbpipe_release(struct device *dev);

/*
 * send a vvpu command to secure vm and wait for response
 */
int vvpu_pp_call(struct device *dev, struct vvpu_secvm_cmd *cmd_p);


#endif /* _VVPU_VBPIPE_H_ */
