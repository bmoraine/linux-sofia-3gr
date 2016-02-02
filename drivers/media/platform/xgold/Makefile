#
#  Copyright (C) 2014-2015 Intel Mobile Communications GmbH
#
# This software is licensed under the terms of the GNU General Public
# License version 2, as published by the Free Software Foundation, and
# may be copied, distributed, and modified under those terms.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#

#These flags switch on driver's internal tracing to buffer
#ccflags-y := -DCONFIG_DEBUG_FS -DCONFIG_CIF_ISP20_REG_TRACE

obj-$(CONFIG_VIDEO_XGOLD) += video_xgold.o
ifeq ($(CONFIG_PLATFORM_DEVICE_PM_VIRT),y)
  video_xgold-objs += cif_isp20_vmm.o
endif
video_xgold-objs += cif_isp20_img_src_v4l2-subdev.o cif_isp20_img_src.o cif_isp20_isp.o cif_isp20_v4l2.o cif_isp20.o
