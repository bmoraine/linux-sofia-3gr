/*
 * Intel RPMB access dispatcher code
 * Copyright (c) 2015, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

int rpc_rpmb_init(void);
int rpc_rpmb_release(void);
int rpc_rpmb_dispatch(u32 opcode, u8 *io_data, u32 *io_data_len);
