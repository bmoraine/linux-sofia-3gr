/*
 *   Adaptations were made to original apds990x.h with
 *  apds990x.h - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __APDS_990x_H__
#define __APDS_990x_H__

#define APDS_990X_DEV_NAME		"apds990x"

#define APDS_990X_I2C_SAD		0x39

struct als_platform_data {
	/* gpio ports for interrupt pads */
	int gpio_int;
	/* glass factor is counted into als_gain */
	unsigned int als_gain;
	unsigned int coeff_b;
	unsigned int coeff_c;
	unsigned int coeff_d;
	struct device_pm_platdata *pm_platdata;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
};
#endif  /* __APDS_990x_H__ */
