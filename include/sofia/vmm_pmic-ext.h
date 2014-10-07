/* ----------------------------------------------------------------------------
   Copyright (C) 2014 Intel Mobile Communications GmbH

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

  ---------------------------------------------------------------------------*/

/**
 * pmic_reg_set_field - MobileVisor pmic 32bit register access service
 * @reg_address		[in] reg_address, bit 24-31 slave address,
 *			bit 0-23 reg address to be accessed
 * @mask		[in] mask for the fields to be set
 * @value		[in] value to be set in masked fields
 *
 * returns 0 if success, -1 if access disallowed or -EINVAL if register
 * type is rwh, wh, rhr or rwhs (so all the registers that can be modified
 * by both serial IF and the HW)
**/
int32_t pmic_reg_set_field(uint32_t reg_address,
				uint8_t mask, uint8_t value);
