/**
 * -------------------------------------------------------------------------
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#ifndef _SW_FUEL_GAUGE_PLATFORM_H
#define _SW_FUEL_GAUGE_PLATFORM_H

/**
 * struct sw_fuel_gauge_platform_data - Coulomb counter HAL configuration data.
 * @sense_resistor_mohm		External sense resistor determines the scaling
 *				of counts to mC
 * @gain_error_1_uc_per_mc	For dual (UP/DOWN) coulomb counter HW:
 *					Gain error for current IN to the
					battery.
 *				Related to quantity of charge.
 *				For single (signed) coulomb counter HW:
 *					Gain error for the balanced coulomb
 *					count.
 * @gain_error_2_uc_per_mc	For dual (UP/DOWN) coulomb counter HW:
 *					Gain error for current OUT of the
 *					battery. Related to quantity of charge.
 *				For single (signed) coulomb counter HW:
 *					Not used. Should be set to zero.
 * @offset_error_uc_per_s	Offest error related to time elapsed
 */
struct sw_fuel_gauge_platform_data {
	int sense_resistor_mohm;
	int gain_error_1_uc_per_mc;
	int gain_error_2_uc_per_mc;
	int offset_error_uc_per_s;
};

#endif /* _SW_FUEL_GAUGE_PLATFORM_H */
