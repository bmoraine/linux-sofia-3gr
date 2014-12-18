#
# Copyright  (C)  2013  Intel  Mobile  Communications  GmbH
#
# This program is free software; you can redistribute it and/or modify it under
# the terms and conditions of the GNU General Public License, version 2, as
# published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU General Public License for more details. You
# should have received a copy of the GNU General Public License along with this
# program; if not, write to the Free Software Foundation, Inc., 51 Franklin St
# Fifth  Floor, Boston, MA 02110-1301 USA.

# Makefile for Intel AG FM Radio driver

# Overall
obj-$(CONFIG_XGOLD_FMRADIO) += fmdrv_xgold-mod.o

# FM Radio Includes
## Common
FMR_DRV_PATH   := drivers/media/radio/intel/
CHIP_ID        := xgold632
PROJECTFOLDER  := xmm6321a2

ccflags-y += -I$(FMR_DRV_PATH)

## FM Radio Rx Firmware
ccflags-y += -I$(FMR_DRV_PATH)/$(CHIP_ID)/fmr_fw

## FM Radio driver
ccflags-y += -I$(FMR_DRV_PATH)/$(CHIP_ID)/fmr_lld
ccflags-y += -I$(FMR_DRV_PATH)/$(CHIP_ID)/fmr_hld
ccflags-y += -I$(FMR_DRV_PATH)/$(CHIP_ID)/fmr_hld/api_sys_interface

## V4L2
ccflags-y    += -Iarch/arm/mach-xgold
ccflags-y    += -Iarch/arm/mach-xgold/include/mach
ccflags-y    += -Iinclude/linux

# FM Radio objects
## Common
FMROBJS         := fmdrv_xgold.o
FMROBJS         += fmdrv_xgold_sys.o

## FM Radio LLD driver
FMROBJS         += $(CHIP_ID)/fmr_lld/aud_app_fmr_lld_api.o
FMROBJS         += $(CHIP_ID)/fmr_lld/aud_app_fmr_lld_basic_rw.o
FMROBJS         += $(CHIP_ID)/fmr_lld/aud_app_fmr_lld_rx_rf.o

## FM Radio HLD driver
FMROBJS         += $(CHIP_ID)/fmr_hld/aud_app_fmr_hld.o
FMROBJS         += $(CHIP_ID)/fmr_hld/aud_app_fmr_hld_rx.o
FMROBJS         += $(CHIP_ID)/fmr_hld/aud_app_fmr_hld_rx_api.o

## FM Radio defines
#ccflags-y   += -DDEBUG_FMR_HLD

fmdrv_xgold-mod-objs := $(FMROBJS)
