#
# Makefile for the Linux kernel device drivers.
#
# Copyright (C) 2012-2013, Intel Mobile Communications GmbH.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License Version 2
# as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
# You should have received a copy of the GNU General Public License Version 2
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#
# Notes:
# Apr 17 2013: IMC: += drivers for versisilicon HW de-/encoder
#
# 15 Sep 2000, Christoph Hellwig <hch@infradead.org>
# Rewritten to use lists instead of if-statements.
# Copyright (C) 2011, Red Bend Ltd.
#

obj-y				+= irqchip/
obj-y				+= bus/

obj-$(CONFIG_GENERIC_PHY)	+= phy/

# GPIO must come after pinctrl as gpios may need to mux pins etc
obj-y				+= pinctrl/
obj-y				+= gpio/
obj-y               += nvm/
obj-y				+= pwm/
obj-$(CONFIG_PCI)		+= pci/
obj-$(CONFIG_PARISC)		+= parisc/
obj-$(CONFIG_RAPIDIO)		+= rapidio/
obj-y				+= video/
obj-y				+= idle/

# IPMI must come before ACPI in order to provide IPMI opregion support
obj-$(CONFIG_IPMI_HANDLER)	+= char/ipmi/

obj-$(CONFIG_ACPI)		+= acpi/
obj-$(CONFIG_SFI)		+= sfi/
# PnP must come after ACPI since it will eventually need to check if acpi
# was used and do nothing if so
obj-$(CONFIG_PNP)		+= pnp/
obj-y				+= amba/
# Many drivers will want to use DMA so this has to be made available
# really early.
obj-$(CONFIG_DMADEVICES)	+= dma/

obj-$(CONFIG_VIRTIO)		+= virtio/
obj-$(CONFIG_MOBILEVISOR_VDRIVER) += mobilevisor/
obj-$(CONFIG_XEN)		+= xen/

# idi early, since some subsystems needs to access register though it
obj-$(CONFIG_IDI)		+= idi/


# regulators early, since some subsystems rely on them to initialize
obj-$(CONFIG_REGULATOR)		+= regulator/

# reset controllers early, since gpu drivers might rely on them to initialize
obj-$(CONFIG_RESET_CONTROLLER)	+= reset/

# tty/ comes before char/ so that the VT console is the boot-time
# default.
obj-y				+= tty/
obj-y				+= char/

# gpu/ comes after char for AGP vs DRM startup
obj-y				+= gpu/

obj-$(CONFIG_CONNECTOR)		+= connector/

# i810fb and intelfb depend on char/agp/
obj-$(CONFIG_FB_I810)           += video/i810/
obj-$(CONFIG_FB_INTEL)          += video/intelfb/

obj-$(CONFIG_PARPORT)		+= parport/
obj-y				+= base/ block/ misc/ mfd/ nfc/
obj-$(CONFIG_NUBUS)		+= nubus/
obj-y				+= macintosh/
obj-$(CONFIG_IDE)		+= ide/
obj-$(CONFIG_SCSI)		+= scsi/
obj-$(CONFIG_ATA)		+= ata/
obj-$(CONFIG_TARGET_CORE)	+= target/
obj-$(CONFIG_MTD)		+= mtd/
obj-$(CONFIG_SPI)		+= spi/
obj-y				+= hsi/
obj-y				+= net/
obj-$(CONFIG_ATM)		+= atm/
obj-$(CONFIG_FUSION)		+= message/
obj-y				+= firewire/
obj-$(CONFIG_UIO)		+= uio/
obj-$(CONFIG_VFIO)		+= vfio/
obj-y				+= cdrom/
obj-y				+= auxdisplay/
obj-$(CONFIG_PCCARD)		+= pcmcia/
obj-$(CONFIG_DIO)		+= dio/
obj-$(CONFIG_SBUS)		+= sbus/
obj-$(CONFIG_ZORRO)		+= zorro/
obj-$(CONFIG_MAC)		+= macintosh/
obj-$(CONFIG_ATA_OVER_ETH)	+= block/aoe/
obj-$(CONFIG_PARIDE) 		+= block/paride/
obj-$(CONFIG_TC)		+= tc/
obj-$(CONFIG_UWB)		+= uwb/
obj-$(CONFIG_USB_PHY)		+= usb/
obj-$(CONFIG_USB)		+= usb/
obj-$(CONFIG_PCI)		+= usb/
obj-$(CONFIG_USB_GADGET)	+= usb/
obj-$(CONFIG_SERIO)		+= input/serio/
obj-$(CONFIG_GAMEPORT)		+= input/gameport/
obj-$(CONFIG_INPUT)		+= input/
obj-$(CONFIG_I2O)		+= message/
obj-$(CONFIG_RTC_LIB)		+= rtc/
obj-y				+= i2c/ media/
obj-$(CONFIG_SLDRIVER)	+= sldriver/
obj-$(CONFIG_PPS)		+= pps/
obj-$(CONFIG_PTP_1588_CLOCK)	+= ptp/
obj-$(CONFIG_W1)		+= w1/
obj-$(CONFIG_POWER_SUPPLY)	+= power/
obj-$(CONFIG_HWMON)		+= hwmon/
obj-$(CONFIG_THERMAL)		+= thermal/
obj-$(CONFIG_WATCHDOG)		+= watchdog/
obj-$(CONFIG_MD)		+= md/
# fm/ must come before any drivers that interface with Frequency Manager via the
# iui_fm_* API's, including FM Radio (FMR), WLAN, IDI, CP-Audio, Class-D and
# SD/eMMC.
obj-$(CONFIG_FM)       += fm/

obj-$(CONFIG_BT)		+= bluetooth/
obj-$(CONFIG_ACCESSIBILITY)	+= accessibility/
obj-$(CONFIG_ISDN)		+= isdn/
obj-$(CONFIG_EDAC)		+= edac/
obj-$(CONFIG_EISA)		+= eisa/
obj-y				+= lguest/
obj-$(CONFIG_CPU_FREQ)		+= cpufreq/
obj-$(CONFIG_CPU_IDLE)		+= cpuidle/
obj-y				+= mmc/
obj-$(CONFIG_MEMSTICK)		+= memstick/
obj-y				+= leds/
obj-$(CONFIG_INFINIBAND)	+= infiniband/
obj-$(CONFIG_SGI_SN)		+= sn/
obj-y				+= firmware/
obj-$(CONFIG_CRYPTO)		+= crypto/
obj-$(CONFIG_SUPERH)		+= sh/
obj-$(CONFIG_ARCH_SHMOBILE_LEGACY)	+= sh/
ifndef CONFIG_ARCH_USES_GETTIMEOFFSET
obj-y				+= clocksource/
endif
obj-$(CONFIG_DCA)		+= dca/
obj-$(CONFIG_HID)		+= hid/
obj-$(CONFIG_PPC_PS3)		+= ps3/
obj-$(CONFIG_OF)		+= of/
obj-$(CONFIG_SSB)		+= ssb/
obj-$(CONFIG_BCMA)		+= bcma/
obj-$(CONFIG_VHOST_RING)	+= vhost/
obj-$(CONFIG_VLYNQ)		+= vlynq/
obj-$(CONFIG_STAGING)		+= staging/
obj-y				+= platform/
#common clk code
obj-y				+= clk/

obj-$(CONFIG_MAILBOX)		+= mailbox/
obj-$(CONFIG_HWSPINLOCK)	+= hwspinlock/
obj-$(CONFIG_NFC)		+= nfc/
obj-$(CONFIG_IOMMU_SUPPORT)	+= iommu/
obj-$(CONFIG_REMOTEPROC)	+= remoteproc/
obj-$(CONFIG_RPMSG)		+= rpmsg/

# Virtualization drivers
obj-$(CONFIG_VIRT_DRIVERS)	+= virt/
obj-$(CONFIG_HYPERV)		+= hv/

obj-$(CONFIG_PM_DEVFREQ)	+= devfreq/
obj-$(CONFIG_EXTCON)		+= extcon/
obj-$(CONFIG_MEMORY)		+= memory/
obj-$(CONFIG_IIO)		+= iio/
obj-$(CONFIG_VME_BUS)		+= vme/
obj-$(CONFIG_IPACK_BUS)		+= ipack/
obj-$(CONFIG_NTB)		+= ntb/
obj-$(CONFIG_FMC)		+= fmc/
obj-$(CONFIG_POWERCAP)		+= powercap/
obj-$(CONFIG_OCT)		+= oct/

obj-$(CONFIG_GATOR)		+= gator/
