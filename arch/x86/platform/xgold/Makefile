#
#  Copyright (C) 2014, Intel Mobile Communications GmbH.
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

obj-$(CONFIG_X86_INTEL_XGOLD) := xgold.o
obj-$(CONFIG_X86_INTEL_XGOLD_EARLY_PRINTK) += early_printk_xgold.o
obj-$(CONFIG_X86_INTEL_XGOLD_PSTORE_RAM) += xgold_pstore_ram.o

ifeq ($(CONFIG_X86_INTEL_XGOLD_EXPERIMENTAL),y)
obj-$(CONFIG_SUSPEND) += pm-suspend.o
endif

obj-y += pm/
