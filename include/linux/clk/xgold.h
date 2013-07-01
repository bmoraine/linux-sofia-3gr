/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_CLK_XGOLD_H_
#define __LINUX_CLK_XGOLD_H_

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
void __init xgold_init_clocks(void);
#else
void __init xgold_init_clocks(void)
{
	/* Do not register clocks when in virt PM */
}
#endif

#endif

