/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/ioctl.h>

#define IMC_IDI_MAGIC 'i'

#define IMC_IDI_BT_SET_POWER_STATE _IOWR(IMC_IDI_MAGIC, 1, unsigned long)
#define IMC_IDI_BT_SET_BT_WUP _IOW(IMC_IDI_MAGIC, 2, unsigned long)
#define IMC_IDI_BT_GET_HOST_WUP _IOR(IMC_IDI_MAGIC, 3, unsigned long)
#define IMC_IDI_BT_SET_RTS _IOW(IMC_IDI_MAGIC, 4, unsigned long)
#define IMC_IDI_BT_GET_RTS _IOR(IMC_IDI_MAGIC, 5, unsigned long)
#define IMC_IDI_BT_GET_CTS _IOR(IMC_IDI_MAGIC, 6, unsigned long)
#define IMC_IDI_BT_DISABLE_SIGNALING _IO(IMC_IDI_MAGIC, 7)
#define IMC_IDI_BT_SET_SCU_FWCTL _IOW(IMC_IDI_MAGIC, 8, unsigned long)
#define IMC_IDI_BT_GET_SCU_FWCTL _IOR(IMC_IDI_MAGIC, 9, unsigned long)
#define IMC_IDI_BT_CONFIG_EXT_UART _IOR(IMC_IDI_MAGIC, 10, unsigned long)

