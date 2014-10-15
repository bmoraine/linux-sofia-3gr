#
#  Copyright (C) 2014 Intel Mobile GmbH
#  Copyright (C) 2012-2013 Intel Mobile Communications GmbH
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License Version 2
#  as published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
#  You should have received a copy of the GNU General Public License Version 2
#  along with this program. If not, see <http://www.gnu.org/licenses/>.
#

obj-$(CONFIG_VIDEO_XGOLD) += video_xgold.o
ifeq ($(CONFIG_PLATFORM_DEVICE_PM_VIRT),y)
  video_xgold-objs += cif_isp20_vmm.o
endif
video_xgold-objs += cif_isp20_img_src_v4l2-subdev.o cif_isp20_img_src.o cif_isp20_isp.o cif_isp20_v4l2.o cif_isp20.o