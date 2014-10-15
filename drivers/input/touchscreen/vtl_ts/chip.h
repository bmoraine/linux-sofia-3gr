/*
 * VTL CTP driver
 *
 * Copyright (C) 2013 VTL Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 */

#ifndef	_CHIP_H_
#define	_CHIP_H_

extern int chip_init(void);
extern int chip_get_fwchksum(struct i2c_client *client, int *fwchksum);
extern int chip_get_checksum(struct i2c_client *client,
	int *bin_checksum, int *fw_checksum);
extern int update(struct i2c_client *client);
extern int chip_update(struct i2c_client *client);
extern int chip_enter_sleep_mode(void);
extern int chip_solfware_reset(struct i2c_client *client);

#endif
