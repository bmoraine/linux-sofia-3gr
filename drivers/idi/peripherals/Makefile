#
# Copyright (C) 2012-2013 Intel Mobile Communications GmbH
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

ifneq ("$(BUILD_KERNEL_WITH_BULLSEYE)","1")
CFLAGS_imc_idi_btif.o := -DIMC_IDI_BTIF_PARANOID
endif

btif_drv-objs := imc_idi_btif.o
obj-$(CONFIG_IDI_IMC_BTIF) += btif_drv.o
obj-y += imc_idi_error.o

