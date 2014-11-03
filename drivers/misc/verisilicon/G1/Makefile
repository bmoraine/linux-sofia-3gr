# Copyright (C) 2013 Intel Mobile Communications GmbH
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#
# Notes:
# Jan 15 2013: IMC: initial version vor kernel 3.x
# Nov  8 2013: IMC: += hx170debugfs

#-------------------------------------------------------------------------------
#-
#--	 Abstract : Makefile for x170 decoder kernel driver
#--
#-----------------------------------------------------------------------------*/

# define required files
# obj-n: do not build
# obj-y: include in kernel
# obj-m: build as module
# obj-$(CONFIG_VERISILICON_G1) ... define in kernel .config file

ccflags-y += -Idrivers/staging
obj-$(CONFIG_VERISILICON_G1) += hx170dec.o
obj-$(CONFIG_VERISILICON_G1_DEBUG_FS) += hx170debugfs.o
