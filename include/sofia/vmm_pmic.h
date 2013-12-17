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
 @brief  MobileVisor pmic 32bit register access service
 @param  reg_address, bit 24-31 slave address, bit 0-23 reg address to be access
 @param  reg_val value to be written
 @return 0 if success, -1 if access disallowed
**/
int32_t vmm_pmic_reg_write(uint32_t reg_address, uint32_t reg_val);

/**
 @brief  MobileVisor pmic 32bit register access service
 @param  reg_address, bit 24-31 slave address, bit 0-23 reg address to be access
 @param  point to write data
 @param  size to write in byte
 @return 0 if success, -1 if access disallowed
**/
int32_t vmm_pmic_reg_write_by_range(uint32_t reg_address,
			uint8_t *data, uint32_t size_in_byte);


/**
 @brief  MobileVisor pmic 32bit register access service
 @param  reg_address, bit 24-31 slave address, bit 0-23 reg address to be access
 @param  p_reg_val read value
 @return 0 if success, -1 if access disallowed
**/
int32_t vmm_pmic_reg_read(uint32_t reg_address, uint32_t *p_reg_val);


/**
 @brief  MobileVisor pmic 32bit register access service
 @param  reg_address, bit 24-31 slave address, bit 0-23 reg address to be access
 @param  point to read data
 @param  size to read in byte
 @return 0 if success, -1 if access disallowed
**/
int32_t vmm_pmic_reg_read_by_range(uint32_t reg_address,
			uint8_t *data, uint32_t size_in_byte);

