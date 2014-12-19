#
#  Copyright (C) 2012-2013 Intel Mobile Communications GmbH
#  Copyright (C) 2011, Red Bend Ltd.
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

obj-y       += proc.o

#VLX virtual Real Time Clock frontend
obj-$(CONFIG_VRTC_FE) += vrtc-fe.o

#VLX virtual OEMCrypto library frontend
obj-$(CONFIG_VOEMCRYPTO_FE) += voemcrypto-fe.o

#VLX virtual stream
obj-$(CONFIG_VBPIPE) += vbpipe.o

#VLX virtual message queue
obj-$(CONFIG_VMQ) += vlx-vmq.o

#VLX virtual VFS
obj-$(CONFIG_VVFS_BE) += vvfs-be.o vlx-vipc.o

#VLX VRPC communication
obj-$(CONFIG_VLX_VRPC) += vrpc.o

#VLX virtual Power management frontend driver
obj-$(CONFIG_VPOWER)		+= vpower.o

#VREBOOT MODULE
obj-$(CONFIG_VREBOOT) += vreboot-fe.o

#VDUMP MODULE
obj-$(CONFIG_VDUMP) += vdump.o

#VLX virtual USB frontend for SIO Extender
obj-$(CONFIG_VUSB_SIO_EXTENDER) += vusb_se_fe.o

#VLX virtual ethernet
obj-$(CONFIG_NKERNEL_VETH)      += veth.o

#VLX virtual System Profiling
obj-$(CONFIG_SYSTEM_PROFILING) += vsysprof-be.o
obj-$(CONFIG_VBPIPE) += modem.o

#RPC driver interface
obj-$(CONFIG_TEE_RPC) += tee_rpc.o
tee_rpc-objs := tee_rpc_driver.o tee_rpc_ioctl.o tee_rpc_memory.o sec_rpc.o vsec.o

