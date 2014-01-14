#
# Copyright (C) 2013 Intel Mobile Communications GmbH
#
# This program is free software: you can redistribute it and/or modify
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

# TODO Header files must be referred from the actual location instead of copy.
# Once the location is available remove '-I$(srctree)/driver/nvm/'

EXTRA_CFLAGS := -I$(srctree)/drivers/media/radio/xgold/ \
	-I$(srctree)/drivers/nvm/

obj-$(CONFIG_NVM)	   += nvmdev.o
obj-$(CONFIG_NVMTEST)  += nvmtest.o

