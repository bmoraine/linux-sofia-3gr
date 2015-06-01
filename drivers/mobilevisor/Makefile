#
#  Copyright (C) 2014 Intel Mobile Communications GmbH
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

obj-$(CONFIG_MOBILEVISOR_VDRIVER_PIPE)      += mvpipe.o
obj-$(CONFIG_MOBILEVISOR_VDRIVER_NVM)       += vnvm-be.o
obj-$(CONFIG_MOBILEVISOR_VDRIVER_POWER)     += vpower.o
obj-$(CONFIG_MOBILEVISOR_VDRIVER_OEMCRYPTO) += voemcrypto-fe.o
obj-$(CONFIG_MOBILEVISOR_VDRIVER_DUMP)      += vdump.o
obj-$(CONFIG_MOBILEVISOR_VDRIVER_DUMP)      += vdump_stp.o
obj-$(CONFIG_MOBILEVISOR_VDRIVER_DUMP)      += vdump_istp.o
obj-$(CONFIG_MOBILEVISOR_VDRIVER_DUMP)      += vdump_file.o
obj-$(CONFIG_MOBILEVISOR_VDRIVER_USB)       += vusb_se_fe.o
obj-$(CONFIG_MOBILEVISOR_VDRIVER_ETHERNET)  += veth.o
obj-$(CONFIG_MOBILEVISOR_VDRIVER_MODEM_CTL) += modem.o
obj-$(CONFIG_SYSTEM_PROFILING)              += vsysprof-be.o
obj-$(CONFIG_TEE_RPC) += tee_rpc.o
tee_rpc-objs := tee_rpc_driver.o tee_rpc_ioctl.o tee_rpc_memory.o sec_shared_mem.o sec_rpc.o vsec.o rpmb_rpc.o

