# Copyright (C) 2013 Intel Mobile Communications GmbH
#
# This software is licensed under the terms of the GNU General Public
# License version 2, as published by the Free Software Foundation, and
# may be copied, distributed, and modified under those terms.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# Overall
obj-$(CONFIG_RADIO_LNP)+= fmdrv_intel.o

# FM Radio Includes
## Common
FMR_DRV_PATH	:= $(srctree)/drivers/media/radio/intel_lnp
CHIPID		:= lnp
#PLATFORM:= xmm6321a2
#PLATFORM := moor
PLATFORM := sofialte

## Search paths
ccflags-y	= -I$(FMR_DRV_PATH)
ccflags-y	+= -I$(FMR_DRV_PATH)/include
ccflags-y	+= -I$(FMR_DRV_PATH)/external_include
ccflags-y	+= -I$(FMR_DRV_PATH)/$(CHIPID)
ccflags-y	+= -I$(FMR_DRV_PATH)/$(CHIPID)/hw

## V4L2
ccflags-y	+= -Iarch/arm/mach-xgold
ccflags-y	+= -Iarch/arm/mach-xgold/include/mach
ccflags-y	+= -Iinclude/linux

# FM Radio objects
FMROBJS	= fmtrx_v4l2.o
FMROBJS	+= fmtrx_common.o
FMROBJS	+= $(join fmtrx_, $(PLATFORM).o)
FMROBJS	+= $(CHIPID)/fmtrx_hw_rx.o

# Defines
#ccflags-y	+= -DFMR_HOST_TEST
ccflags-y	+= -DFMR_INTR_MODE
ccflags-y	+= -DFMR_AUDIO_ENABLE
ccflags-y	+= -DFMR_ALIGN=""
ccflags-y	+= -DFMR_DEBUG_MEAS
# LVL1 - print high level information - lesser logs
ccflags-y	+= -DFMR_DEBUG_LVL1
# LVL2 - print low level information - detailed logs
ccflags-y	+= -DFMR_DEBUG_LVL2
ifeq ($(CHIPID), lnp)
ccflags-y	+= -DFMR_ABB_BASE="0x00000000"
ccflags-y	+= -DFMR_BASE="0x00000000"
ccflags-y	+= -DFMR_DCDC_TOP_REGISTER="0x0000F000"
ccflags-y	+= -DLD_DRIVER
#ccflags-y	+= -DLNP_EVB
#ccflags-y       += -DFMR_104MHZ_WORKAROUND
else
ccflags-y	+= -DFMR_ABB_BASE="0x01000000"
ccflags-y	+= -DFMR_BASE="0xE7000000"
endif

fmdrv_intel-objs	:= $(FMROBJS)

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
