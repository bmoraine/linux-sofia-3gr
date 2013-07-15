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

#
# Makefile for the IDI bus driver.
#

subdir-ccflags-$(CONFIG_IDI_DEBUG) := -DDEBUG

obj-$(CONFIG_IDI)  += idi_driver.o
idi_driver-objs    += idi_bus.o idi_interface.o idi_tty.o idi_debug.o idi_device_pm.o

obj-y  += clients/ controllers/ peripherals/
obj-$(CONFIG_OF)  += idi_of.o
