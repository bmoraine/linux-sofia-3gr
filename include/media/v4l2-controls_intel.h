/*
 ****************************************************************
 *
 *  Intel custom V4L2 controls
 *
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
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
 * Note:
 *     16/10/2014: intiial version.
 *
 ****************************************************************
 */

#ifndef _V4L2_CONTROLS_INTEL_H
#define _V4L2_CONTROLS_INTEL_H

#include <linux/videodev2.h>

#define V4L2_CID_USER_INTEL_BASE (V4L2_CID_USER_BASE + 0x1090)

#define INTEL_VIDIOC_SENSOR_MODE_DATA _IOR('v', BASE_VIDIOC_PRIVATE, struct isp_supplemental_sensor_mode_data)

#define INTEL_V4L2_CID_AUTO_FPS (V4L2_CID_USER_INTEL_BASE+0)

#endif
